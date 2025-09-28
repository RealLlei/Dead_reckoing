/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file vt_sample_generator.cc
 **/

#include "planning/tasks/optimizers/speed_data_optimizer/generators/vt_sample/forward_vt_sample_generator.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <ios>
#include <limits>
#include <memory>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

#include "common/file/log.h"
#include "common/math/double_type.h"
#include "common/math/linear_interpolation.h"
#include "common/math/math_utils.h"
#include "common/thread/thread_pool.h"
#include "common/util/macros.h"
#include "planning/common/frame.h"
#include "planning/common/path/frenet_frame_path.h"
#include "planning/common/planning_gflags.h"
#include "planning/common/reference_line_info.h"
#include "planning/reference_line_info_decider/common/reference_line_info_decider_helper.h"
#include "planning/tasks/optimizers/speed_data_optimizer/caches/slt_obstacle_cache.h"
#include "planning/tasks/optimizers/speed_data_optimizer/caches/speed_cache.h"
#include "planning/tasks/optimizers/speed_data_optimizer/caches/st_obstacle_cache.h"
#include "planning/tasks/optimizers/speed_data_optimizer/costs/speed_curve.h"
#include "planning/tasks/optimizers/speed_data_optimizer/evaluators/speed_evaluator.h"

#include "planning/proto/speed_evaluator_config.pb.h"
#include "planning/proto/task_config.pb.h"
#include "proto/common/pnc_point.pb.h"
#include "proto/planning/decision.pb.h"
#include "proto/planning/warning.pb.h"

namespace TL {
namespace planning {

using common::math::double_type::DefinitelyLess;
using common::math::double_type::DefinitelyLessEqual;

ForwardVtSampleGenerator::ForwardVtSampleGenerator(
    const SpeedDataGeneratorConfig& config)
    : ForwardGearSpeedDataGenerator(config),
      vehicle_param_(common::VehicleConfigHelper::GetConfig().vehicle_param()) {
  if (config.has_forward_vt_sample_generator_config()) {
    config_.CopyFrom(config.forward_vt_sample_generator_config());
  }

  if (config_.has_forward_vt_sampler_config()) {
    current_sampler_ = std::make_shared<ForwardVtSampler>(
        config_.forward_vt_sampler_config(), config_.speed_curve_config());
  }
  extend_sample_times_.assign(2, std::vector<double>{});
  double delta_extend_time = 0.5;
  for (int i = 0; i < 2; i++) {
    std::vector<double> extend_times{};
    extend_times.assign(10, 0.0);
    for (int j = 0; j < 10; j++) {
      extend_times.at(j) =
          static_cast<double>(j + 1) *
              (static_cast<double>(i + 1) * delta_extend_time) +
          (static_cast<double>(i) * 5.0 + 20.0);
    }
    extend_sample_times_.at(i) = extend_times;
  }
}

bool ForwardVtSampleGenerator::Process(
    const std::shared_ptr<DependencyInjector>& injector, Frame* frame,
    ReferenceLineInfo* reference_line_info,
    const common::TrajectoryPoint& init_point, SpeedCache* cache,
    const std::shared_ptr<SpeedEvaluator>& evaluator, SpeedData* speed_data) {
  if (frame == nullptr || reference_line_info == nullptr || cache == nullptr ||
      evaluator == nullptr || speed_data == nullptr) {
    AERROR << "input error, ForwardVtSampleGenerator::Process failed";
    return false;
  }
  best_curve_ = nullptr;
  evaluator_ = evaluator;

  // calculate some info from last frame such as time_from_last_frame_,
  // distance_from_last_frame
  CalculateInfoFromLastFrame(injector, frame, init_point, *cache);

  // adjust init speed and accel if path data is reverse
  auto cur_init_point = init_point;
  if (!reference_line_info->path_data().frenet_frame_path().is_forward_path()) {
    cur_init_point.set_v(std::fabs(cur_init_point.v()));
    cur_init_point.set_a(-cur_init_point.a());
  }

  if (!Optimize(frame, reference_line_info, cur_init_point, cache, evaluator,
                speed_data)) {
    AERROR << "vt sample optimize, ForwardVtSampleGenerator::Process failed";
    return false;
  }

  return true;
}

bool ForwardVtSampleGenerator::GeneratorSpeedDataWithoutBackObstacle(
    const Frame* frame, const ReferenceLineInfo* reference_line_info,
    const common::TrajectoryPoint& init_point, const SpeedCache& cache,
    const std::shared_ptr<SpeedEvaluator>& evaluator, SpeedData* speed_data) {
  UNUSED(cache);
  if (frame == nullptr || reference_line_info == nullptr ||
      speed_data == nullptr || current_sampler_ == nullptr) {
    return false;
  }

  // get min cost result for front obstacle
  auto* cost_results = current_sampler_->GetMutableGuideCurveCostResults();
  std::size_t start_curve_index = 0;
  std::size_t end_curve_index = current_sampler_->GetGuideCurveCount();
  const SpeedCurveCostResult* min_cost_result = nullptr;
  for (std::size_t i = start_curve_index; i < end_curve_index; ++i) {
    auto& cost_result = cost_results->at(i);
    evaluator->ResetToFrontCost(&cost_result);
    if (evaluator->CompareCostResult(&cost_result, min_cost_result)) {
      min_cost_result = &cost_result;
    }
  }

  cost_results = current_sampler_->GetMutableNormalCurveCostResults();
  end_curve_index = current_sampler_->GetNormalCurveCount();
  for (std::size_t i = start_curve_index; i < end_curve_index; ++i) {
    auto& cost_result = cost_results->at(i);
    evaluator->ResetToFrontCost(&cost_result);
    if (evaluator->CompareCostResult(&cost_result, min_cost_result)) {
      min_cost_result = &cost_result;
    }
  }

  // select min cost curve
  if (min_cost_result == nullptr || min_cost_result->curve == nullptr ||
      std::isinf(min_cost_result->total_cost)) {
    return false;
  }

  return GenerateSpeedData(init_point, min_cost_result->curve, cache,
                           speed_data);
}

bool ForwardVtSampleGenerator::GeneratorMergeStopSpeedData(
    const Frame* frame, const ReferenceLineInfo* reference_line_info,
    const common::TrajectoryPoint& init_point, const SpeedCache& cache,
    const std::shared_ptr<SpeedEvaluator>& evaluator,
    SpeedData* const speed_data) {
  if (frame == nullptr || reference_line_info == nullptr ||
      evaluator == nullptr || speed_data == nullptr ||
      current_sampler_ == nullptr) {
    AERROR << "input is nullptr";
    return false;
  }
  min_cost_result_ = nullptr;
  SelectBestNormalCurveWithMergeStopS(reference_line_info, cache, evaluator);

  // sample merge stop curves
  current_sampler_->SampleMergeStopCurves(init_point, *reference_line_info,
                                          cache);

  auto* cost_results = current_sampler_->GetMutableMergeStopCurveCostResults();
  std::size_t start_curve_index = 0;
  std::size_t end_curve_index = current_sampler_->GetMergeStopCurveCount();
  CalculateCost(reference_line_info, cache, evaluator, cost_results,
                start_curve_index, end_curve_index);

  for (std::size_t i = start_curve_index; i < end_curve_index; ++i) {
    auto& cost_result = cost_results->at(i);
    if (evaluator->CompareCostResult(&cost_result, min_cost_result_)) {
      min_cost_result_ = &cost_result;
    }
  }

  if (min_cost_result_ == nullptr || min_cost_result_->curve == nullptr ||
      (std::isinf(min_cost_result_->total_cost) &&
       (std::isinf(min_cost_result_->obstacle_safe_distance_cost) ||
        std::isinf(min_cost_result_->collision_cost)))) {
    GenerateFallbackSpeedData(reference_line_info, init_point, cache, evaluator,
                              speed_data);
  } else {
    GenerateSpeedData(init_point, min_cost_result_->curve, cache, speed_data);
  }
  last_cost_result_.curve.reset();
  return true;
}

bool ForwardVtSampleGenerator::GenerateFallbackSpeedData(
    const ReferenceLineInfo* reference_line_info,
    const common::TrajectoryPoint& init_point, const SpeedCache& cache,
    const std::shared_ptr<SpeedEvaluator>& evaluator, SpeedData* speed_data) {
  if (speed_data != nullptr && current_sampler_ != nullptr &&
      reference_line_info != nullptr &&
      current_sampler_->GenerateFallbackSpeedData(
          init_point, *reference_line_info, cache, evaluator, speed_data)) {
    speed_data->SetIsFallback(true);
    return true;
  }
  return false;
}

bool ForwardVtSampleGenerator::Optimize(
    Frame* frame, ReferenceLineInfo* const reference_line_info,
    const common::TrajectoryPoint& init_point, SpeedCache* const cache,
    const std::shared_ptr<SpeedEvaluator>& evaluator, SpeedData* speed_data) {
  // record init point
  ADEBUG << "[init_point]v:" << init_point.v() << ", a:" << init_point.a()
         << ", seq_num:" << frame->SequenceNum()
         << ", reference_line_info:" << reference_line_info
         << ", is_change_lane_path:" << reference_line_info->IsChangeLanePath();

  if (frame == nullptr || reference_line_info == nullptr || cache == nullptr ||
      evaluator == nullptr || speed_data == nullptr) {
    AERROR << "frame is nullptr";
    return false;
  }

  // check replan
  if (frame->GetIsLongitudinalReplan()) {
    last_cost_result_.curve.reset();
    ADEBUG << "replan happen";
  }

  // check sampler
  if (current_sampler_ == nullptr) {
    AERROR << "can not find sampler";
    return false;
  }

  // calculate obstacle locations
  CalculateObstacleSTLocations(reference_line_info, init_point, cache,
                               evaluator);

  // calculate follow obstacle
  CalculateFollowObstacle(*frame, reference_line_info, init_point, cache);

  if ((CheckIfFollowDangerous(init_point, *cache) ||
       (CheckIfStopDangerous(init_point, *cache, reference_line_info))) &&
      current_sampler_->GenerateFallbackSpeedData(cache->GetOriginInitPoint(),
                                                  *reference_line_info, *cache,
                                                  evaluator, speed_data)) {
    speed_data->SetIsFallback(true);
    last_cost_result_.curve.reset();
    ADEBUG << "follow dangerous, use fallback vt sample curve";
    return true;
  }

  // update st boundary
  cache->UpdateSTBoundary(reference_line_info);

  // calculate safe curves
  CalculateSafeCurves(reference_line_info, init_point, cache, evaluator);

  // calculate curve cost and select curve with min cost
  const auto best_curve =
      SelectBestCurve(reference_line_info, init_point, *cache, evaluator);

  // check if best_curve is dangerous
  is_dangerous_ = CheckIfDangerous(*frame, *cache, best_curve);
  if (is_dangerous_ && current_sampler_->GenerateFallbackSpeedData(
                           cache->GetOriginInitPoint(), *reference_line_info,
                           *cache, evaluator, speed_data)) {
    speed_data->SetIsFallback(true);
    last_cost_result_.curve.reset();
    ADEBUG << "dangerous, use fallback vt sample curve";
    return true;
  }

  // record debug
  RecordDebug(reference_line_info, *cache);

  // if normal curve is valid, return ok
  if (best_curve != nullptr &&
      GenerateSpeedData(init_point, best_curve, *cache, speed_data)) {
    best_curve_ = best_curve;
    const auto& last_frame_block_obstacle_id =
        cache->GetBasicCache().GetLastFrameBlockObstacleId();
    const auto& block_obstacle_id = cache->GetBasicCache().GetBlockObstacleId();
    if (!block_obstacle_id.empty() &&
        last_frame_block_obstacle_id == block_obstacle_id &&
        best_curve->GetTarget().mode == SpeedCurveTarget::Mode::STOP) {
      current_sampler_->SetStopTime(best_curve->GetTimeLength());
    } else {
      current_sampler_->SetStopTime(std::numeric_limits<double>::infinity());
    }
    return true;
  }

  // use common fallback
  last_cost_result_.curve.reset();
  current_sampler_->SetStopTime(std::numeric_limits<double>::infinity());
  if (GenerateFallbackSpeedData(reference_line_info,
                                cache->GetOriginInitPoint(), *cache, evaluator,
                                speed_data)) {
    ADEBUG << "use fallback vt sample curve";
    return true;
  }
  return false;
}

void ForwardVtSampleGenerator::CalculateInfoFromLastFrame(
    const std::shared_ptr<DependencyInjector>& injector, const Frame* frame,
    const common::TrajectoryPoint& init_point, const SpeedCache& cache) {
  if (frame == nullptr || injector == nullptr ||
      injector->frame_history() == nullptr) {
    return;
  }

  const auto* last_frame = injector->frame_history()->Latest();
  if (last_frame == nullptr) {
    return;
  }

  if (last_frame->local_view()
              .GetFunctionManagerIn()
              ->fct_avp_in()
              .sys_run_state() == functionmanager::AvpFctIn::PAUSE &&
      frame->local_view()
              .GetFunctionManagerIn()
              ->fct_avp_in()
              .sys_run_state() != functionmanager::AvpFctIn::PAUSE) {
    last_cost_result_.curve.reset();
    ADEBUG << "function resume, do not use last curve";
  }

  // calculate time and distance from last frame
  if (last_frame->SequenceNum() == frame->SequenceNum()) {
    return;
  }

  const auto time_from_last_frame =
      (frame->vehicle_state().timestamp() + init_point.relative_time()) -
      (last_frame->vehicle_state().timestamp() +
       last_frame->PlanningStartPoint().relative_time());
  if (last_cost_result_.curve == nullptr) {
    return;
  }

  if (last_cost_result_.curve->SetStartPoint(
          last_cost_result_.curve->GetStartTime() + time_from_last_frame)) {
    const auto& last_target = last_cost_result_.curve->GetTarget();
    if (last_target.mode == SpeedCurveTarget::Mode::CRUISE &&
        fabs(last_target.speed -
             cache.GetSpeedLimitCache().cruise_target_speed()) > 0.1) {
      last_cost_result_.curve.reset();
    }
  } else {
    last_cost_result_.curve.reset();
  }

  ADEBUG << FIXED << SETPRECISION(3)
         << "duration_from_last_frame:" << time_from_last_frame;
}

void ForwardVtSampleGenerator::CalculateCost(
    const ReferenceLineInfo* reference_line_info, const SpeedCache& cache,
    const std::shared_ptr<SpeedEvaluator>& evaluator,
    std::vector<SpeedCurveCostResult>* const cost_results,
    const std::size_t start_curve_index, const std::size_t end_curve_index) {
  ADEBUG << "vt_sample_curve_count:" << end_curve_index - start_curve_index;
  if (start_curve_index >= end_curve_index || reference_line_info == nullptr ||
      evaluator == nullptr || cost_results == nullptr) {
    return;
  }

  const auto thread_count = config_.thread_count();
  const int count_per_thread =
      static_cast<int>(end_curve_index - start_curve_index) / thread_count + 1;
  std::vector<int> thread_indexes;
  thread_indexes.reserve(thread_count);
  for (int i = 0; i < thread_count; ++i) {
    thread_indexes.emplace_back(i);
  }

  TL::common::thread::ThreadPool::ForEach(
      thread_indexes.begin(), thread_indexes.end(),
      [&](const auto& thread_index) {
        common::sub_thread_name = "_planning";
        const auto start_index =
            thread_index * count_per_thread + start_curve_index;
        const auto end_index =
            std::min(start_index + count_per_thread, end_curve_index);
        for (int i = start_index; i < end_index; ++i) {
          evaluator->CalculateCost(cache, *reference_line_info,
                                   &cost_results->at(i));
        }
      });
}

void ForwardVtSampleGenerator::CalculateObstacleSTLocations(
    const ReferenceLineInfo* reference_line_info,
    const common::TrajectoryPoint& init_point, SpeedCache* const cache,
    const std::shared_ptr<SpeedEvaluator>& evaluator) {
  if (current_sampler_ == nullptr || evaluator == nullptr || cache == nullptr) {
    return;
  }

  current_sampler_->SampleGuideCurves(init_point, *cache);
  auto* cost_results = current_sampler_->GetMutableGuideCurveCostResults();
  const auto start_curve_index = 0;
  const auto end_curve_index = current_sampler_->GetGuideCurveCount();
  CalculateCost(reference_line_info, *cache, evaluator, cost_results,
                start_curve_index, end_curve_index);

  // select min cost current curve
  min_cost_result_ = nullptr;
  for (std::size_t i = start_curve_index; i < end_curve_index; ++i) {
    auto& cost_result = cost_results->at(i);
    if (evaluator->CompareCostResult(&cost_result, min_cost_result_)) {
      min_cost_result_ = &cost_result;
    }
  }

  if (min_cost_result_ == nullptr || min_cost_result_->curve == nullptr ||
      std::isinf(min_cost_result_->total_cost)) {
    cache->IgnoreDecision();
    CalculateCost(reference_line_info, *cache, evaluator, cost_results,
                  start_curve_index, end_curve_index);

    // select min cost current curve
    min_cost_result_ = nullptr;
    for (std::size_t i = start_curve_index; i < end_curve_index; ++i) {
      auto& cost_result = cost_results->at(i);
      if (evaluator->CompareCostResult(&cost_result, min_cost_result_)) {
        min_cost_result_ = &cost_result;
      }
    }
  }

  if (min_cost_result_ == nullptr || min_cost_result_->curve == nullptr ||
      std::isinf(min_cost_result_->total_cost)) {
    return;
  }

  const auto& points = min_cost_result_->curve->GetDensePoints();
  const auto point_count = min_cost_result_->curve->GetMinDensePointCount();

  auto* st_obstacle_caches = cache->GetMutableSTObstacleCaches();
  auto* slt_obstacle_caches = cache->GetMutableSLTObstacleCaches();
  if (st_obstacle_caches == nullptr || slt_obstacle_caches == nullptr) {
    return;
  }

  for (auto& obstacle_cache : *st_obstacle_caches) {
    for (int i = 0; i < point_count; ++i) {
      const auto& point = points.at(i);
      if (point.t() < obstacle_cache.GetMinT() ||
          point.t() > obstacle_cache.GetMaxT()) {
        continue;
      }

      const auto& obstacle_info =
          obstacle_cache.GetObstacleInfoAtTime(point.t());
      obstacle_cache.SetSTObstacleLocation(
          (point.s() < obstacle_info.s_lower)
              ? STObstacleLocation::ABOVE
              : ((point.s() > obstacle_info.s_upper)
                     ? STObstacleLocation::BELOW
                     : STObstacleLocation::CROSS));
      ADEBUG << "id:" << obstacle_cache.GetObstacle()->Id() << ", location:"
             << static_cast<int>(obstacle_cache.GetSTObstacleLocation());
      break;
    }
  }

  for (auto& obstacle_cache : *slt_obstacle_caches) {
    for (int i = 0; i < point_count; ++i) {
      const auto& point = points.at(i);
      if (point.t() < obstacle_cache.GetMinT() ||
          point.t() > obstacle_cache.GetMaxT()) {
        continue;
      }

      const auto s_lower = point.s() - vehicle_param_.back_edge_to_center();
      const auto s_upper = point.s() + vehicle_param_.front_edge_to_center();
      const auto l_lower = -vehicle_param_.width() / 2.0;
      const auto l_upper = -l_lower;

      const auto& obstacle_info =
          obstacle_cache.GetObstacleInfoAtTime(point.t());
      obstacle_cache.SetSTObstacleLocation(
          (s_upper < obstacle_info.s_lower)
              ? LTObstacleLocation::ABOVE
              : ((s_lower > obstacle_info.s_upper)
                     ? STObstacleLocation::BELOW
                     : STObstacleLocation::CROSS));
      obstacle_cache.SetLTObstacleLocation(
          (l_upper < obstacle_info.l_lower)
              ? LTObstacleLocation::ABOVE
              : ((l_lower > obstacle_info.l_upper)
                     ? LTObstacleLocation::BELOW
                     : LTObstacleLocation::CROSS));
      ADEBUG << "id:" << obstacle_cache.GetObstacle()->Id() << ", st_location:"
             << static_cast<int>(obstacle_cache.GetSTObstacleLocation())
             << ", lt_location:"
             << static_cast<int>(obstacle_cache.GetLTObstacleLocation());
      break;
    }
  }
}

bool ForwardVtSampleGenerator::CheckIfFollow(
    const ReferenceLineInfo& reference_line_info, const SpeedCache& cache,
    const STObstacleCache& obstacle_cache) {
  static constexpr double kFollowObstacleIgnoreTime = 1.5;
  const auto* obstacle = obstacle_cache.GetObstacle();
  if (obstacle == nullptr || obstacle->IsStatic() || obstacle->IsVirtual() ||
      obstacle_cache.GetSTObstacleLocation() != STObstacleLocation::ABOVE ||
      obstacle->path_st_boundary().min_t() > kFollowObstacleIgnoreTime ||
      obstacle->GetLateralIntention() == LateralIntention::ALONGSIDE ||
      obstacle->GetIsCrossObstacle()) {
    return false;
  }

  const auto& trajectory_envelope = obstacle->GetTrajectoryEnvelope();
  const auto& trajectory_points = obstacle->Trajectory().trajectory_point();
  if (trajectory_envelope.empty() || trajectory_points.empty()) {
    return false;
  }

  bool is_cutout = false;
  reference_line_info_decider_helper::IsObsCutOut(&reference_line_info,
                                                  obstacle, &is_cutout);
  if (is_cutout) {
    return CheckIfFollowCutoutObstacle(reference_line_info, cache,
                                       obstacle_cache);
  }

  // reverse obstacle don't need follow
  const auto& ego_vehicle_state = reference_line_info.vehicle_state();
  const auto ego_moving_heading = common::math::NormalizeAngle(
      ego_vehicle_state.gear() == soc::Chassis::GEAR_REVERSE
          ? ego_vehicle_state.heading() + M_PI
          : ego_vehicle_state.heading());
  const auto obstacle_start_moving_heading =
      trajectory_points.size() > 1
          ? trajectory_points.at(1).path_point().theta()
          : trajectory_points.at(0).path_point().theta();
  const auto obstacle_end_moving_heading =
      trajectory_points.at(trajectory_points.size() - 1).path_point().theta();
  const auto start_heading_diff = fabs(common::math::NormalizeAngle(
      ego_moving_heading - obstacle_start_moving_heading));
  const auto end_heading_diff = fabs(common::math::NormalizeAngle(
      ego_moving_heading - obstacle_end_moving_heading));
  UNUSED(end_heading_diff);
  const auto start_end_heading_diff = fabs(common::math::NormalizeAngle(
      obstacle_end_moving_heading - obstacle_start_moving_heading));
  UNUSED(start_end_heading_diff);
  static constexpr double kHeadingDiffThreshold1st = 0.333 * M_PI;
  return start_heading_diff < kHeadingDiffThreshold1st;
}

bool ForwardVtSampleGenerator::CheckIfFollowCutoutObstacle(
    const ReferenceLineInfo& reference_line_info, const SpeedCache& cache,
    const STObstacleCache& obstacle_cache) {
  const auto* obstacle = obstacle_cache.GetObstacle();
  if (obstacle == nullptr || obstacle->IsStatic() || obstacle->IsVirtual()) {
    return false;
  }

  const auto& bounding_boxes = obstacle->GetTrajectoryBoundingBox();
  const auto& trajectory_points = obstacle->Trajectory().trajectory_point();
  if (bounding_boxes.empty() || trajectory_points.empty()) {
    return false;
  }

  const auto& corners = bounding_boxes.front().GetAllCorners();
  if (corners.size() != 4) {
    return false;
  }

  const auto l_thresthold =
      (obstacle->PerceptionId() == cache.GetOldFollowObstacleId()) ? 0.2 : 0.0;

  const auto center_s = (obstacle->PerceptionSLBoundary().start_s() +
                         obstacle->PerceptionSLBoundary().end_s()) *
                        0.5;
  const auto reference_point =
      reference_line_info.reference_line().GetReferencePoint(center_s);
  if (common::math::AngleDiff(reference_point.heading(),
                              trajectory_points.at(0).path_point().theta()) >
      0.0) {
    // cutout to left lane, check left front corner
    common::SLPoint left_front_corner;
    reference_line_info.reference_line().XYToSL(corners.at(1),
                                                &left_front_corner);
    double lane_left_width = 0.0;
    double lane_right_width = 0.0;
    reference_line_info.reference_line().GetLaneWidth(
        left_front_corner.s(), &lane_left_width, &lane_right_width);
    return left_front_corner.l() < lane_left_width + l_thresthold;
  }

  // cutout to right lane, check right front corner
  common::SLPoint right_front_corner;
  reference_line_info.reference_line().XYToSL(corners.at(0),
                                              &right_front_corner);
  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  reference_line_info.reference_line().GetLaneWidth(
      right_front_corner.s(), &lane_left_width, &lane_right_width);
  return right_front_corner.l() > -lane_right_width - l_thresthold;
}

void ForwardVtSampleGenerator::CalculateFollowObstacle(
    const Frame& frame, ReferenceLineInfo* const reference_line_info,
    const common::TrajectoryPoint& init_point, SpeedCache* const cache) {
  if (cache == nullptr || reference_line_info == nullptr ||
      frame.GetReferenceLineProvider() == nullptr ||
      frame.GetReferenceLineProvider()->GetPncMap() == nullptr) {
    return;
  }

  std::vector<hdmap::LaneInfoConstPtr> adc_lanes;
  reference_line_info->reference_line().GetLaneFromS(
      (reference_line_info->AdcSlBoundary().start_s() +
       reference_line_info->AdcSlBoundary().end_s()) *
          0.5,
      &adc_lanes);
  if (adc_lanes.empty()) {
    return;
  }

  const auto& adc_lane_id = adc_lanes.front()->id().id();
  const auto& lane_ids = frame.GetReferenceLineProvider()
                             ->GetPncMap()
                             ->GetAdcPassageRoutingInfo()
                             ->GetLaneIds();
  bool find =
      std::any_of(lane_ids.begin(), lane_ids.end(),
                  [&](const auto& lane_id) { return lane_id == adc_lane_id; });

  // if (!find) {
  //   return;
  // }

  // select current follow obstacle according to current_reference_line_info,
  // for obstacles which perception id == last_follow_obstacle_id,
  // choose obstacle which has biggest st_boundary_max_s
  auto follow_start_s = std::numeric_limits<double>::infinity();
  auto follow_ttc = std::numeric_limits<double>::infinity();
  const auto& adc_sl_boundary = reference_line_info->AdcSlBoundary();
  const auto& vehicle_state = reference_line_info->vehicle_state();

  const auto& obstacle_info = frame.GetObstacleInfo();
  const auto* cipv =
      reference_line_info->path_decision()->Find(obstacle_info.GetCIPV());

  std::string follow_obstacle_id;
  for (const auto* obstacle_cache : cache->GetSafeSTObstacleCaches()) {
    if (obstacle_cache == nullptr) {
      continue;
    }
    const auto* obstacle = obstacle_cache->GetObstacle();
    if (obstacle == nullptr ||
        (cipv != nullptr && obstacle_cache->GetId() != cipv->Id() &&
         obstacle->PerceptionSLBoundary().start_s() >
             cipv->PerceptionSLBoundary().start_s()) ||
        obstacle->PerceptionSLBoundary().end_s() < adc_sl_boundary.end_s() ||
        !CheckIfFollow(*reference_line_info, *cache, *obstacle_cache)) {
      continue;
    }

    auto ttc = 0.0;
    const auto obstacle_sl_boundary = obstacle->PerceptionSLBoundary();
    if (obstacle_sl_boundary.start_s() > adc_sl_boundary.end_s()) {
      const auto ds = obstacle_sl_boundary.start_s() - adc_sl_boundary.end_s();
      const double obstacle_speed =
          common::math::Vec2d::CreateUnitVec2d(init_point.path_point().theta())
              .InnerProd(Vec2d(obstacle->Perception().velocity().x(),
                               obstacle->Perception().velocity().y()));
      const auto dv = vehicle_state.linear_velocity() - obstacle_speed;
      ttc = DefinitelyLessEqual(dv, 0.0)
                ? std::numeric_limits<double>::infinity()
                : ds / dv;
    }

    static constexpr double kTTCThreshold = 5.0;
    if (std::isinf(follow_start_s) ||
        (follow_ttc < kTTCThreshold && ttc < follow_ttc) ||
        (follow_ttc > kTTCThreshold &&
         (ttc < kTTCThreshold ||
          obstacle_sl_boundary.start_s() < follow_start_s))) {
      follow_ttc = ttc;
      follow_start_s = obstacle_sl_boundary.start_s();
      follow_obstacle_id = obstacle_cache->GetId();
    }
  }

  cache->SetFollowObstacle(frame, *reference_line_info, init_point,
                           follow_obstacle_id);
  reference_line_info->SetLonFollowObsId(follow_obstacle_id);

  ADEBUG << "follow obstacle id:" << follow_obstacle_id;
}

bool ForwardVtSampleGenerator::CheckIfFollowDangerous(
    const common::TrajectoryPoint& init_point, const SpeedCache& cache) {

  const auto* follow_slt_obstacle_cache = cache.GetFollowSLTObstacleCache();
  if (follow_slt_obstacle_cache == nullptr) {
    return false;
  }

  const auto& obstacle_info =
      follow_slt_obstacle_cache->GetObstacleInfoAtTimeCeil(
          follow_slt_obstacle_cache->GetMinT());

  if (is_follow_dangerous_ &&
      (init_point.v() < config_.follow_safe_threshold().adc_speed(0) ||
       obstacle_info.ds >
           common::math::InterpolationOne(
               init_point.v(), config_.follow_safe_threshold().adc_speed(),
               config_.follow_safe_threshold().obstacle_speed()))) {
    is_follow_dangerous_ = false;
  } else if (!is_follow_dangerous_ &&
             init_point.v() >
                 config_.follow_dangerous_threshold().adc_speed(0) &&
             obstacle_info.ds <
                 common::math::InterpolationOne(
                     init_point.v(),
                     config_.follow_dangerous_threshold().adc_speed(),
                     config_.follow_dangerous_threshold().obstacle_speed())) {
    is_follow_dangerous_ = true;
  }

  return is_follow_dangerous_;
}

void ForwardVtSampleGenerator::CalculateSafeCurves(
    const ReferenceLineInfo* reference_line_info,
    const common::TrajectoryPoint& init_point, SpeedCache* const cache,
    const std::shared_ptr<SpeedEvaluator>& evaluator) {
  if (current_sampler_ == nullptr || cache == nullptr) {
    return;
  }

  auto* st_obstacle_caches = cache->GetMutableSTObstacleCaches();
  auto* slt_obstacle_caches = cache->GetMutableSLTObstacleCaches();
  if (slt_obstacle_caches == nullptr) {
    return;
  }
  constexpr auto kMaxCost = 2.5e8;
  for (auto& slt_obstacle_cache : *slt_obstacle_caches) {
    bool need_sample_safe_curve = false;
    for (auto& st_obstacle_cache : *st_obstacle_caches) {
      if (st_obstacle_cache.GetObstacle() == slt_obstacle_cache.GetObstacle() &&
          st_obstacle_cache.GetSTObstacleLocation() ==
              STObstacleLocation::ABOVE) {
        need_sample_safe_curve = true;
        break;
      }
    }

    if (!need_sample_safe_curve) {
      continue;
    }
    bool set_safe_extend = false;
    const std::vector<double> sample_extend_time{0.0, 0.0};
    current_sampler_->SampleSafeCurves(init_point, *cache, slt_obstacle_cache,
                                       set_safe_extend, sample_extend_time);
    auto* cost_results = current_sampler_->GetMutableSafeCurveCostResults();
    std::size_t start_curve_index = 0;
    std::size_t end_curve_index = current_sampler_->GetSafeCurveCount();
    CalculateCost(reference_line_info, *cache, evaluator, cost_results,
                  start_curve_index, end_curve_index);

    SpeedCurveCostResult* min_cost_result = nullptr;
    for (std::size_t i = start_curve_index; i < end_curve_index; ++i) {
      auto& cost_result = cost_results->at(i);
      if (evaluator->CompareCostResult(&cost_result, min_cost_result)) {
        min_cost_result = &cost_result;
      }
    }
    bool use_extend_sample_times = false;
    std::vector<double> extend_sample_time_select{};
    SpeedCurveCostResult* extend_min_cost_result = nullptr;
    if (min_cost_result == nullptr || min_cost_result->curve == nullptr) {
      set_safe_extend = true;
      for (auto& extend_sample_time : extend_sample_times_) {
        current_sampler_->SampleSafeCurves(init_point, *cache,
                                           slt_obstacle_cache, set_safe_extend,
                                           extend_sample_time);
        auto* cost_results = current_sampler_->GetMutableSafeCurveCostResults();
        std::size_t start_curve_index = 0;
        std::size_t end_curve_index = current_sampler_->GetSafeCurveCount();
        CalculateCost(reference_line_info, *cache, evaluator, cost_results,
                      start_curve_index, end_curve_index);
        for (std::size_t i = start_curve_index; i < end_curve_index; ++i) {
          auto& cost_result = cost_results->at(i);
          if (evaluator->CompareCostResult(&cost_result,
                                           extend_min_cost_result)) {
            extend_min_cost_result = &cost_result;
          }
        }
        if (extend_min_cost_result != nullptr &&
            extend_min_cost_result->curve != nullptr) {
          use_extend_sample_times = true;
          extend_sample_time_select = extend_sample_time;
          break;
        }
      }
    } else {
      extend_min_cost_result = min_cost_result;
    }
    if (extend_min_cost_result == nullptr ||
        extend_min_cost_result->curve == nullptr) {
      continue;
    }
    min_cost_result = extend_min_cost_result;
    ADEBUG << "use_extend_sample_times " << use_extend_sample_times
           << " GetTimeLength " << min_cost_result->curve->GetTimeLength();
    bool flag = false;
    const auto count =
        std::min(min_cost_result->obstacle_s_safe_distance_costs.size(),
                 min_cost_result->obstacle_l_safe_distance_costs.size());
    for (std::size_t i = 0; i < count; ++i) {
      auto& obstacle_s_safe_distance_cost =
          min_cost_result->obstacle_s_safe_distance_costs.at(i);
      auto& obstacle_l_safe_distance_cost =
          min_cost_result->obstacle_l_safe_distance_costs.at(i);
      if (obstacle_s_safe_distance_cost > kMaxCost &&
          obstacle_l_safe_distance_cost > kMaxCost) {
        flag = true;
      }
      obstacle_s_safe_distance_cost =
          fmin(obstacle_s_safe_distance_cost, kMaxCost);
      obstacle_l_safe_distance_cost =
          fmin(obstacle_l_safe_distance_cost, kMaxCost);
    }

    if (&slt_obstacle_cache == cache->GetFollowSLTObstacleCache()) {
      auto follow_times = current_sampler_->GetSampleFollowTimes();
      if (use_extend_sample_times) {
        follow_times = extend_sample_time_select;
      }
      cache->SetNeedFollowCurve(
          !follow_times.empty() &&
          (follow_times.back() != min_cost_result->curve->GetTimeLength() ||
           !flag));
    }

    slt_obstacle_cache.SetSSafeDistanceCosts(
        min_cost_result->obstacle_s_safe_distance_costs);
    slt_obstacle_cache.SetLSafeDistanceCosts(
        min_cost_result->obstacle_l_safe_distance_costs);
  }
}

std::shared_ptr<SpeedCurve> ForwardVtSampleGenerator::SelectBestCurve(
    const ReferenceLineInfo* reference_line_info,
    const common::TrajectoryPoint& init_point, const SpeedCache& cache,
    const std::shared_ptr<SpeedEvaluator>& evaluator) {
  static constexpr double kEpsilon = 1e-2;
  // calculate last cost
  last_cost_result_.already_calculated = false;
  evaluator->CalculateCost(cache, *reference_line_info, &last_cost_result_);

  // check if reuse stop curve
  if (CheckIfReuseStopCurve(cache)) {
    ADEBUG << "reuse stop curve";
    return last_cost_result_.curve;
  }

  // calculate guide curve cost
  ADEBUG << "sample guide curves:" << current_sampler_->GetGuideCurveCount();
  auto* cost_results = current_sampler_->GetMutableGuideCurveCostResults();
  std::size_t start_curve_index = 0;
  std::size_t end_curve_index = current_sampler_->GetGuideCurveCount();
  CalculateCost(reference_line_info, cache, evaluator, cost_results,
                start_curve_index, end_curve_index);

  // select min cost curve
  min_cost_result_ = nullptr;
  for (std::size_t i = start_curve_index; i < end_curve_index; ++i) {
    auto& cost_result = cost_results->at(i);
    if (evaluator->CompareCostResult(&cost_result, min_cost_result_)) {
      min_cost_result_ = &cost_result;
    }
  }
  bool set_follow_extend = false;
  const std::vector<double> sample_extend_time{0.0, 0.0};
  current_sampler_->SampleNormalCurves(init_point, *reference_line_info, cache,
                                       set_follow_extend, sample_extend_time);
  ADEBUG << "sample normal curves:" << current_sampler_->GetNormalCurveCount();

  // calculate nromal curve cost
  cost_results = current_sampler_->GetMutableNormalCurveCostResults();
  end_curve_index = current_sampler_->GetNormalCurveCount();
  CalculateCost(reference_line_info, cache, evaluator, cost_results,
                start_curve_index, end_curve_index);

  // select min cost curve
  for (std::size_t i = start_curve_index; i < end_curve_index; ++i) {
    auto& cost_result = cost_results->at(i);
    if (evaluator->CompareCostResult(&cost_result, min_cost_result_)) {
      min_cost_result_ = &cost_result;
    }
  }
  const SpeedCurveCostResult* extend_normal_min_cost_result = min_cost_result_;
  if (cache.GetFollowSLTObstacleCache() != nullptr &&
      cache.GetNeedFollowCurve()) {
    if (min_cost_result_ == nullptr || min_cost_result_->curve == nullptr ||
        std::isinf(min_cost_result_->total_cost) ||
        min_cost_result_->curve->GetTarget().mode !=
            SpeedCurveTarget::Mode::FOLLOW) {
      set_follow_extend = true;
      for (auto& extend_sample_time : extend_sample_times_) {
        current_sampler_->SampleNormalCurves(init_point, *reference_line_info,
                                             cache, set_follow_extend,
                                             extend_sample_time);
        ADEBUG << "sample normal curves:"
               << current_sampler_->GetNormalCurveCount();

        // calculate nromal curve cost
        cost_results = current_sampler_->GetMutableNormalCurveCostResults();
        end_curve_index = current_sampler_->GetNormalCurveCount();
        CalculateCost(reference_line_info, cache, evaluator, cost_results,
                      start_curve_index, end_curve_index);

        // select min cost curve
        for (std::size_t i = start_curve_index; i < end_curve_index; ++i) {
          auto& cost_result = cost_results->at(i);
          if (evaluator->CompareCostResult(&cost_result, min_cost_result_)) {
            min_cost_result_ = &cost_result;
          }
        }
      }
      if (min_cost_result_ == nullptr || min_cost_result_->curve == nullptr ||
          std::isinf(min_cost_result_->total_cost) ||
          min_cost_result_->curve->GetTarget().mode !=
              SpeedCurveTarget::Mode::FOLLOW) {
        min_cost_result_ = extend_normal_min_cost_result;
      }
    }
  }
  // select min cost curve
  if (min_cost_result_ == nullptr || min_cost_result_->curve == nullptr ||
      std::isinf(min_cost_result_->total_cost)) {
    return nullptr;
  }

  if (cache.GetIsLastFallback() &&
      min_cost_result_->curve->GetStartV() < kEpsilon &&
      min_cost_result_->curve->GetEndV() < kEpsilon &&
      min_cost_result_->curve->GetEndS() < kEpsilon) {
    ADEBUG << " last is fallback, no space to move, use fallback curve";
    return nullptr;
  }

  // determin whether use last curve
  if (last_cost_result_.curve == nullptr) {
    ADEBUG << "last curve is nullptr";
    return min_cost_result_->curve;
  }

  if (last_cost_result_.curve->GetTarget().mode !=
      min_cost_result_->curve->GetTarget().mode) {
    ADEBUG << "last curve mode is different from current";
    return min_cost_result_->curve;
  }

  if (!current_sampler_->AddLastCurve(last_cost_result_.curve, cache)) {
    ADEBUG << "last curve not meeting conditions";
    return min_cost_result_->curve;
  }

  if (evaluator->CompareCostResult(&last_cost_result_, min_cost_result_)) {
    ADEBUG << "last curve has smaller cost";
    return last_cost_result_.curve;
  }
  ADEBUG << "current curve has smaller cost";
  return min_cost_result_->curve;
}

bool ForwardVtSampleGenerator::CheckIfReuseStopCurve(const SpeedCache& cache) {
  const auto& basic_cache = cache.GetBasicCache();
  if (!basic_cache.GetIsStopPrefinish() ||
      basic_cache.GetLastFrameBlockObstacleId() !=
          basic_cache.GetBlockObstacleId() ||
      basic_cache.GetBlockObstacleId().empty() ||
      last_cost_result_.curve == nullptr ||
      last_cost_result_.curve->GetTarget().mode !=
          SpeedCurveTarget::Mode::STOP ||
      std::isinf(last_cost_result_.total_cost) || cache.StartAfterStop()) {
    return false;
  }

  const auto& obstacle_caches = cache.GetSafeSTObstacleCaches();
  return std::none_of(obstacle_caches.begin(), obstacle_caches.end(),
                      [&](const auto& obstacle_cache) {
                        const auto* obstacle = obstacle_cache->GetObstacle();
                        return obstacle != nullptr &&
                               !obstacle->path_st_boundary().IsEmpty() &&
                               obstacle->path_st_boundary().min_s() <
                                   basic_cache.GetExpectedStopS();
                      });
}

bool ForwardVtSampleGenerator::GenerateSpeedData(
    const common::TrajectoryPoint& init_point,
    const std::shared_ptr<SpeedCurve>& curve, const SpeedCache& cache,
    SpeedData* const speed_data) {
  if (curve == nullptr || speed_data == nullptr) {
    return false;
  }
  speed_data->clear();
  last_cost_result_.curve = curve->Clone();
  last_cost_result_.curve->Discretize(FLAGS_trajectory_time_length,
                                      FLAGS_trajectory_time_resolution,
                                      speed_data);

  if (last_cost_result_.curve->GetTarget().mode ==
      SpeedCurveTarget::Mode::FOLLOW) {
    Clamp(init_point, cache, speed_data);
  }

  return true;
}

bool ForwardVtSampleGenerator::GenerateStandStillSpeedData(
    const common::TrajectoryPoint& init_point, SpeedData* const speed_data) {
  if (speed_data == nullptr) {
    return false;
  }

  last_cost_result_.curve.reset();
  const auto a = fmin(config_.standstill_accel(), -1e-6);
  const auto max_decel_t = -init_point.v() / a;
  const auto max_s = 0.5 * init_point.v() * max_decel_t;
  common::SpeedPoint speed_point;
  const auto t_count = static_cast<int>(std::round(
      FLAGS_trajectory_time_length / FLAGS_trajectory_time_resolution));
  for (int i = 0; i <= t_count; ++i) {
    const auto t = i * FLAGS_trajectory_time_resolution;
    if (t < max_decel_t) {
      speed_point.set_s(init_point.v() * t + 0.5 * a * t * t);
      speed_point.set_v(init_point.v() + a * t);
    } else {
      speed_point.set_s(max_s);
      speed_point.set_v(0.0);
    }
    speed_point.set_t(t);
    speed_point.set_a(a);
    speed_point.set_da(0.0);
    speed_data->push_back(speed_point);
  }
  return true;
}

bool ForwardVtSampleGenerator::CheckIfDangerous(
    const Frame& frame, const SpeedCache& cache,
    const std::shared_ptr<SpeedCurve>& curve) {
  return CheckIfDangerousWithStaticObstacles(frame, cache, curve) ||
         CheckIfDangerousWithDynamicObstacles(cache, curve);
}

bool ForwardVtSampleGenerator::CheckIfDangerousWithStaticObstacles(
    const Frame& frame, const SpeedCache& cache,
    const std::shared_ptr<SpeedCurve>& curve) {
  UNUSED(frame);
  if (curve == nullptr) {
    return false;
  }
  if (curve->GetTarget().mode != SpeedCurveTarget::Mode::STOP) {
    return false;
  }
  constexpr double kEpsilon = 1e-5;
  const auto expected_stop_s =
      curve->GetTarget().mode == SpeedCurveTarget::Mode::STOP
          ? curve->GetSLength()
          : cache.GetBasicCache().GetExpectedStopS();
  if (std::isinf(expected_stop_s)) {
    return false;
  }

  // if (cache.GetBasicCache().GetBlockObstacleId() !=
  //     frame.GetObstacleInfo().GetCIPV()) {
  //   return false;
  // }
  const auto& obstacle_caches = cache.GetSafeSTObstacleCaches();
  if (std::any_of(obstacle_caches.begin(), obstacle_caches.end(),
                  [&](const auto& obstacle_cache) {
                    const auto* obstacle = obstacle_cache->GetObstacle();
                    return obstacle != nullptr &&
                           !obstacle->path_st_boundary().IsEmpty() &&
                           obstacle->path_st_boundary().min_s() <
                               cache.GetBasicCache().GetExpectedStopS();
                  })) {
    return false;
  }

  const auto& dense_points = curve->GetDensePoints();
  const auto curve_dense_point_count = curve->GetCurveDensePointCount();
  for (int i = 0; i < curve_dense_point_count; ++i) {
    const auto& point = dense_points.at(i);
    if (point.s() > expected_stop_s + kEpsilon) {
      return true;
    }
    const auto a = pow(point.v(), 2) /
                   (-2.0 * fmax(expected_stop_s - point.s(), kEpsilon));
    ADEBUG << "t:" << point.t() << ", v:" << point.v()
           << ", ds:" << (point.s() - expected_stop_s) << ", a:" << a;
    if ((is_dangerous_ && a < config_.safe_decel_for_static_obstacle()) ||
        a < config_.dangerous_decel_for_static_obstacle()) {
      return true;
    }
  }

  return false;
}

bool ForwardVtSampleGenerator::CheckIfDangerousWithDynamicObstacles(
    const SpeedCache& cache, const std::shared_ptr<SpeedCurve>& curve) {
  UNUSED(cache);
  UNUSED(curve);
  if (curve == nullptr || min_cost_result_ == nullptr ||
      min_cost_result_->curve == nullptr) {
    return false;
  }

  const auto* obstacle_cache = cache.GetFollowSLTObstacleCache();
  if (obstacle_cache == nullptr || obstacle_cache->GetObstacle() == nullptr) {
    return false;
  }
  static constexpr double kLBuffer = 0.4;
  const auto& vehicle_param_ =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  const auto adc_l_lower = -vehicle_param_.width() / 2.0 - kLBuffer;
  const auto adc_l_upper = -adc_l_lower;
  const auto& dense_points = curve->GetDensePoints();
  const auto curve_dense_point_count = curve->GetCurveDensePointCount();
  double min_a = 0.0;
  for (int i = 0; i < curve_dense_point_count; ++i) {
    const auto& point = dense_points.at(i);
    if (point.t() < obstacle_cache->GetMinT() ||
        point.t() > obstacle_cache->GetMaxT() ||
        point.t() > config_.max_dangerous_time_for_dynamic_obstacle()) {
      continue;
    }

    const auto& obstacle_info =
        obstacle_cache->GetObstacleInfoAtTime(point.t());
    if (obstacle_info.l_upper < adc_l_lower ||
        obstacle_info.l_lower > adc_l_upper) {
      continue;
    }
    const auto dv = point.v() - obstacle_info.ds;
    double min_follow_distance = 0.0;
    if (point.v() > 2.0) {
      min_follow_distance = obstacle_cache->GetMinFollowDistance() *
                            (std::min(point.v(), 4.0) - 2.0) / (4.0 - 2.0);
    }
    const auto ds = obstacle_info.s_lower -
                    (point.s() + vehicle_param_.front_edge_to_center()) -
                    min_follow_distance;
    if (dv > 0.0) {
      min_a = ds > 0.0 ? fmin(min_a, -pow(dv, 2) / (2.0 * ds))
                       : std::numeric_limits<double>::lowest();
    }
  }

  return (is_dangerous_ && min_a < config_.safe_decel_for_dynamic_obstacle()) ||
         min_a < config_.dangerous_decel_for_dynamic_obstacle();
}

void ForwardVtSampleGenerator::Clamp(const common::TrajectoryPoint& init_point,
                                     const SpeedCache& cache,
                                     SpeedData* const speed_data) const {
  if (speed_data == nullptr || speed_data->size() < 2) {
    return;
  }

  if (common::math::double_type::DefinitelyGreater(
          last_cost_result_.curve->GetStartTime(), 0.0)) {
    auto& first_point = speed_data->front();
    first_point.set_v(init_point.v());
    first_point.set_a(init_point.a());
    first_point.set_da(init_point.da());
  }

  const auto unit_t = speed_data->at(1).t() - speed_data->at(0).t();
  std::size_t i = 1;
  for (; i < speed_data->size(); ++i) {
    const auto& last_point = speed_data->at(i - 1);
    auto& current_point = speed_data->at(i);

    const auto& accel_limit =
        cache.GetBasicCache().GetAccelLimit(last_point.v());
    const auto min_a = accel_limit.first;
    const auto max_a = accel_limit.second;
    auto new_a = common::math::Clamp(current_point.a(), min_a, max_a);

    const auto& jerk_limit = cache.GetBasicCache().GetJerkLimit(last_point.v());
    const auto min_j = jerk_limit.first;
    const auto max_j = jerk_limit.second;

    auto new_j =
        common::math::Clamp((new_a - last_point.a()) / unit_t, min_j, max_j);
    new_a = last_point.a() + new_j * unit_t;
    auto new_v =
        last_point.v() + last_point.a() * unit_t + new_j * unit_t * unit_t / 2;
    if (DefinitelyLess(new_v, 0.0)) {
      break;
    }
    current_point.set_da(new_j);
    current_point.set_a(new_a);
    current_point.set_v(new_v);
    current_point.set_s(last_point.s() + last_point.v() * unit_t +
                        last_point.a() * unit_t * unit_t / 2.0 +
                        new_j * unit_t * unit_t * unit_t / 6.0);
  }

  for (; i < speed_data->size(); ++i) {
    const auto& last_point = speed_data->at(i - 1);
    auto& current_point = speed_data->at(i);
    current_point.set_da(0.0);
    current_point.set_a(last_point.a());
    current_point.set_v(0.0);
    current_point.set_s(last_point.s());
  }
}

std::string ForwardVtSampleGenerator::PrintCurveInfo(
    const ReferenceLineInfo* reference_line_info, const SpeedCache& cache,
    const SpeedCurveCostResult& cost_result) {
  if (cost_result.curve == nullptr) {
    return "";
  }

  // for (const auto& speed_cost : evaluator_->GetSpeedCosts()) {
  //   speed_cost->SetPrintDebug(true);
  // }
  // evaluator_->CalculateCost(cache, *reference_line_info,
  //                           (SpeedCurveCostResult*)&cost_result);
  // for (const auto& speed_cost : evaluator_->GetSpeedCosts()) {
  //   speed_cost->SetPrintDebug(false);
  // }

  ADEBUG << cost_result.curve->DebugString();

  std::stringstream ss;
  ss << std::fixed << std::setprecision(3) << DebugString(cost_result)
     << ", start_v:" << cost_result.curve->GetStartV()
     << ", end_v:" << cost_result.curve->GetEndV()
     << ", end_t:" << cost_result.curve->GetEndTime()
     << ", min_v:" << cost_result.curve->GetMinV()
     << ", max_v:" << cost_result.curve->GetMaxV()
     << ", min_a:" << cost_result.curve->GetMinAccel()
     << ", max_a:" << cost_result.curve->GetMaxAccel()
     << ", min_j:" << cost_result.curve->GetMinJerk()
     << ", max_j:" << cost_result.curve->GetMaxJerk()
     << ", ends:" << cost_result.curve->GetEndS()
     << ", TimeLength:" << cost_result.curve->GetTimeLength()
     << ", mode:" << static_cast<int>(cost_result.curve->GetTarget().mode);
  ADEBUG << "[current_curve_cost]" << ss.str();

  const auto point_count =
      std::max(cost_result.curve->GetCurveDensePointCount(),
               cost_result.curve->GetMinDensePointCount());
  const auto& dense_points = cost_result.curve->GetDensePoints();

  const auto& speed_limit_cache = cache.GetSpeedLimitCache();
  for (std::size_t i = 0; i < point_count; ++i) {
    const auto& point = dense_points[i];
    const auto t = point.t();
    ADEBUG << FIXED << SETPRECISION(3) << "t:" << t << ", s:" << point.s()
           << ", v:" << point.v() << ", a:" << point.a() << ", j:" << point.j();

    for (const auto& obstacle_cache : cache.GetSTObstacleCaches()) {
      const auto* obstacle = obstacle_cache.GetObstacle();
      const auto& boundary = obstacle->path_st_boundary();
      // check if obstacle is in st graph at time t
      if (t < obstacle_cache.GetMinT() || t > obstacle_cache.GetMaxT()) {
        continue;
      }

      // calucalate s_upper, s_lower, obstacle_v
      const auto& obstacle_info = obstacle_cache.GetObstacleInfoAtTime(t);
      ADEBUG << FIXED << SETPRECISION(3)
             << "    [st_obstacle]obs:" << obstacle->Id()
             << ", s_lower:" << obstacle_info.s_lower
             << ", s_upper:" << obstacle_info.s_upper
             << ", v:" << obstacle_info.v << ", min_t:" << boundary.min_t()
             << ", max_t:" << boundary.max_t();
    }

    for (const auto& obstacle_cache : cache.GetNudgeObstacleCaches()) {
      const auto* obstacle = obstacle_cache.GetObstacle();
      const auto& boundary = obstacle->path_st_boundary();
      // check if obstacle is in st graph at time t
      if (t < obstacle_cache.GetMinT() || t > obstacle_cache.GetMaxT()) {
        continue;
      }

      // calucalate s_upper, s_lower, obstacle_v
      const auto& obstacle_info = obstacle_cache.GetObstacleInfoAtTime(t);
      ADEBUG << FIXED << SETPRECISION(3)
             << "    [nudge_obstacle]obs:" << obstacle->Id()
             << ", s_lower:" << obstacle_info.s_lower
             << ", s_upper:" << obstacle_info.s_upper
             << ", ds:" << obstacle_info.ds << ", min_t:" << boundary.min_t()
             << ", max_t:" << boundary.max_t();
    }

    for (const auto& obstacle_cache : cache.GetSLTObstacleCaches()) {
      const auto* obstacle = obstacle_cache.GetObstacle();
      // check if obstacle is in st graph at time t
      if (t < obstacle_cache.GetMinT() || t > obstacle_cache.GetMaxT()) {
        continue;
      }

      // calucalate s_upper, s_lower, obstacle_v
      const auto& obstacle_info = obstacle_cache.GetObstacleInfoAtTime(t);
      ADEBUG << FIXED << SETPRECISION(3)
             << "    [slt_obstacle]obs:" << obstacle->Id()
             << ", s_lower:" << obstacle_info.s_lower
             << ", s_upper:" << obstacle_info.s_upper
             << ", l_lower:" << obstacle_info.l_lower
             << ", l_upper:" << obstacle_info.l_upper
             << ", ds:" << obstacle_info.ds << ", dl:" << obstacle_info.dl
             << ", min_t:" << obstacle_cache.GetMinT()
             << ", max_t:" << obstacle_cache.GetMaxT();
    }

    const auto& speed_limit_info =
        speed_limit_cache.GetPositionSpeedLimit(point.s());
    ADEBUG << FIXED << SETPRECISION(3)
           << "    map_speed_limit: " << speed_limit_info.map_speed_limit
           << ", curvature_speed_limit: "
           << speed_limit_info.curvature_speed_limit
           << ", decision_speed_limit: "
           << speed_limit_info.decision_speed_limit
           << ", cruise_speed_limit: " << reference_line_info->GetCruiseSpeed();
  }
  return ss.str();
}

void ForwardVtSampleGenerator::RecordDebug(
    ReferenceLineInfo* const reference_line_info, const SpeedCache& cache) {
  auto* vt_sample_debug = reference_line_info->mutable_debug()
                              ->mutable_planning_data()
                              ->mutable_speed_data_optimizer_debug()
                              ->mutable_vt_sample_debug();
  vt_sample_debug->set_guide_curve_count(
      current_sampler_->GetGuideCurveCount());
  vt_sample_debug->set_safe_curve_count(current_sampler_->GetSafeCurveCount());
  vt_sample_debug->set_normal_curve_count(
      current_sampler_->GetNormalCurveCount());

  if (last_cost_result_.curve != nullptr) {
    ADEBUG << "print last curve cost info";
    vt_sample_debug->set_last_curve_info(
        PrintCurveInfo(reference_line_info, cache, last_cost_result_));
  }
  if (min_cost_result_ != nullptr) {
    ADEBUG << "print min cost curve info";
    vt_sample_debug->set_min_cost_curve_info(
        PrintCurveInfo(reference_line_info, cache, *min_cost_result_));
  }
}

void ForwardVtSampleGenerator::CalculateCostForMergeStopDistance(
    const ReferenceLineInfo* reference_line_info, const SpeedCache& cache,
    const std::shared_ptr<SpeedEvaluator>& evaluator,
    std::vector<SpeedCurveCostResult>* const cost_results,
    const std::size_t start_curve_index, const std::size_t end_curve_index) {
  ADEBUG << "vt_sample_curve_count:" << end_curve_index - start_curve_index;
  if (start_curve_index >= end_curve_index || reference_line_info == nullptr ||
      evaluator == nullptr || cost_results == nullptr) {
    return;
  }
  const auto thread_count = config_.thread_count();
  const int count_per_thread =
      static_cast<int>(end_curve_index - start_curve_index) / thread_count + 1;
  std::vector<int> thread_indexes;
  thread_indexes.reserve(thread_count);
  for (int i = 0; i < thread_count; ++i) {
    thread_indexes.emplace_back(i);
  }
  TL::common::thread::ThreadPool::ForEach(
      thread_indexes.begin(), thread_indexes.end(),
      [&](const auto& thread_index) {
        common::sub_thread_name = "_planning";
        const auto start_index =
            thread_index * count_per_thread + start_curve_index;
        const auto end_index =
            std::min(start_index + count_per_thread, end_curve_index);
        for (int i = start_index; i < end_index; ++i) {
          evaluator->CalculateMergeStopDistanceCost(cache, *reference_line_info,
                                                    &cost_results->at(i));
        }
      });
}

void ForwardVtSampleGenerator::SelectBestNormalCurveWithMergeStopS(
    const ReferenceLineInfo* reference_line_info, const SpeedCache& cache,
    const std::shared_ptr<SpeedEvaluator>& evaluator) {
  if (reference_line_info == nullptr || evaluator == nullptr) {
    return;
  }
  min_cost_result_ = nullptr;
  // calculate guide curve cost
  auto* cost_results = current_sampler_->GetMutableGuideCurveCostResults();
  std::size_t start_curve_index = 0;
  std::size_t end_curve_index = current_sampler_->GetGuideCurveCount();

  // select min cost curve
  CalculateCostForMergeStopDistance(reference_line_info, cache, evaluator,
                                    cost_results, start_curve_index,
                                    end_curve_index);
  for (std::size_t i = start_curve_index; i < end_curve_index; ++i) {
    auto& cost_result = cost_results->at(i);
    if (evaluator->CompareCostResult(&cost_result, min_cost_result_)) {
      min_cost_result_ = &cost_result;
    }
  }

  // calculate nromal curve cost
  cost_results = current_sampler_->GetMutableNormalCurveCostResults();
  end_curve_index = current_sampler_->GetNormalCurveCount();

  // select min cost curve
  CalculateCostForMergeStopDistance(reference_line_info, cache, evaluator,
                                    cost_results, start_curve_index,
                                    end_curve_index);
  for (std::size_t i = start_curve_index; i < end_curve_index; ++i) {
    auto& cost_result = cost_results->at(i);
    if (evaluator->CompareCostResult(&cost_result, min_cost_result_)) {
      min_cost_result_ = &cost_result;
    }
  }
}

bool ForwardVtSampleGenerator::CheckIfStopDangerous(
    const common::TrajectoryPoint& init_point, const SpeedCache& cache,
    const ReferenceLineInfo* reference_line_info) {
  if (reference_line_info == nullptr) {
    is_stop_dangerous_ = false;
    return false;
  }
  const auto* stop_obs = reference_line_info->path_decision().Find(
      cache.GetBasicCache().GetBlockObstacleId());
  if (stop_obs == nullptr || stop_obs->IsVirtual()) {
    is_stop_dangerous_ = false;
    return false;
  }
  if (std::isinf(cache.GetBasicCache().GetExpectedStopS())) {
    is_stop_dangerous_ = false;
    return false;
  }
  if (init_point.v() < 2.0 && !is_stop_dangerous_) {
    is_stop_dangerous_ = false;
    return false;
  }
  const auto stop_dec =
      -pow(init_point.v(), 2) /
      (2 * fmax(cache.GetBasicCache().GetExpectedStopS(), 0.001));
  if (is_stop_dangerous_ &&
      stop_dec >
          config_.safe_stop_dec_threshold().safe_decel_for_static_obstacle()) {
    is_stop_dangerous_ = false;
  } else if (!is_follow_dangerous_ &&
             stop_dec < config_.safe_stop_dec_threshold()
                            .dangerous_decel_for_static_obstacle()) {
    is_stop_dangerous_ = true;
  }
  return is_stop_dangerous_;
}
}  // namespace planning
}  // namespace TL
