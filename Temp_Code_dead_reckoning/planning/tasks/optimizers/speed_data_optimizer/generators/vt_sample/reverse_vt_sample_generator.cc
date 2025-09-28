/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file vt_sample_generator.cc
 **/

#include "planning/tasks/optimizers/speed_data_optimizer/generators/vt_sample/reverse_vt_sample_generator.h"

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
#include "common/thread/thread_pool.h"
#include "planning/common/frame.h"
#include "planning/common/path/frenet_frame_path.h"
#include "planning/common/planning_gflags.h"
#include "planning/common/reference_line_info.h"
#include "planning/tasks/optimizers/speed_data_optimizer/caches/speed_cache.h"
#include "planning/tasks/optimizers/speed_data_optimizer/costs/speed_curve.h"
#include "planning/tasks/optimizers/speed_data_optimizer/evaluators/speed_evaluator.h"

#include "planning/proto/speed_evaluator_config.pb.h"
#include "planning/proto/task_config.pb.h"
#include "proto/common/pnc_point.pb.h"
#include "proto/perception/perception_obstacle.pb.h"

namespace TL {
namespace planning {

ReverseVtSampleGenerator::ReverseVtSampleGenerator(
    const SpeedDataGeneratorConfig& config)
    : ReverseGearSpeedDataGenerator(config),
      vehicle_param_(common::VehicleConfigHelper::GetConfig().vehicle_param()) {
  if (config.has_reverse_vt_sample_generator_config()) {
    config_.CopyFrom(config.reverse_vt_sample_generator_config());
  }

  if (config_.has_reverse_vt_sampler_config()) {
    current_sampler_ = std::make_shared<ReverseVtSampler>(
        config_.reverse_vt_sampler_config(), config_.speed_curve_config());
  }
}

bool ReverseVtSampleGenerator::Process(
    const std::shared_ptr<DependencyInjector>& injector, Frame* frame,
    ReferenceLineInfo* reference_line_info,
    const common::TrajectoryPoint& init_point, SpeedCache* cache,
    const std::shared_ptr<SpeedEvaluator>& evaluator, SpeedData* speed_data) {
  if (frame == nullptr || reference_line_info == nullptr || cache == nullptr ||
      evaluator == nullptr || speed_data == nullptr) {
    AERROR << "input error, ReverseVtSampleGenerator::Process failed";
    return false;
  }

  evaluator_ = evaluator;

  // calculate some info from last frame such as time_from_last_frame_,
  // distance_from_last_frame
  CalculateInfoFromLastFrame(injector, frame, init_point, *cache);

  if (!Optimize(frame, reference_line_info, init_point, cache, evaluator,
                speed_data)) {
    AERROR << "vt sample optimize, ReverseVtSampleGenerator::Process failed";
    return false;
  }

  return true;
}

bool ReverseVtSampleGenerator::GenerateFallbackSpeedData(
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

bool ReverseVtSampleGenerator::Optimize(
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
  CalculateFollowObstacle(*frame, *reference_line_info, init_point, cache);

  // calculate safe curves
  CalculateSafeCurves(reference_line_info, init_point, cache, evaluator);

  // calculate curve cost and select curve with min cost
  const auto best_curve =
      SelectBestCurve(reference_line_info, init_point, *cache, evaluator);

  // check if best_curve is dangerous
  is_dangerous_ = CheckIfDangerous(*cache, best_curve);
  if (is_dangerous_ && current_sampler_->GenerateFallbackSpeedData(
                           cache->GetOriginInitPoint(), *reference_line_info,
                           *cache, evaluator, speed_data)) {
    last_cost_result_.curve.reset();
    ADEBUG << "dangerous, use fallback vt sample curve";
    return true;
  }

  // record debug
  RecordDebug(reference_line_info, *cache);

  // if normal curve is valid, return ok
  if (best_curve != nullptr &&
      GenerateSpeedData(init_point, best_curve, speed_data)) {
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

void ReverseVtSampleGenerator::CalculateInfoFromLastFrame(
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

void ReverseVtSampleGenerator::CalculateCost(
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

void ReverseVtSampleGenerator::CalculateObstacleSTLocations(
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

  if (min_cost_result_ == nullptr || min_cost_result_->curve == nullptr) {
    return;
  }

  auto* st_obstacle_caches = cache->GetMutableSTObstacleCaches();
  auto* slt_obstacle_caches = cache->GetMutableSLTObstacleCaches();
  if (st_obstacle_caches == nullptr || slt_obstacle_caches == nullptr) {
    return;
  }

  const auto& points = min_cost_result_->curve->GetDensePoints();
  const auto point_count = min_cost_result_->curve->GetMinDensePointCount();
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
              ? STObstacleLocation::ABOVE
              : ((s_lower > obstacle_info.s_upper)
                     ? STObstacleLocation::BELOW
                     : STObstacleLocation::CROSS));
      obstacle_cache.SetLTObstacleLocation(
          (l_upper < obstacle_info.l_lower)
              ? LTObstacleLocation::ABOVE
              : ((l_lower > obstacle_info.l_upper)
                     ? LTObstacleLocation::BELOW
                     : LTObstacleLocation::CROSS));
      ADEBUG << "id:" << obstacle_cache.GetObstacle()->Id() << ", location:"
             << static_cast<int>(obstacle_cache.GetSTObstacleLocation());
      break;
    }
  }
}

bool ReverseVtSampleGenerator::CheckIfFollow(
    const Frame& frame, const ReferenceLineInfo& reference_line_info,
    const STObstacleCache& obstacle_cache) {
  UNUSED(reference_line_info);
  const auto* obstacle = obstacle_cache.GetObstacle();
  if (obstacle == nullptr || obstacle->IsStatic() || obstacle->IsVirtual()) {
    return false;
  }

  const auto& trajectory_envelope = obstacle->GetTrajectoryEnvelope();
  const auto& trajectory_points = obstacle->Trajectory().trajectory_point();
  if (trajectory_envelope.empty() || trajectory_points.empty()) {
    return false;
  }

  // check if obstacle trajectory is on reference line
  if (frame.GetReferenceLineProvider() == nullptr ||
      frame.GetReferenceLineProvider()->GetPncMap() == nullptr ||
      frame.GetReferenceLineProvider()
              ->GetPncMap()
              ->GetAdcPassageRoutingInfo() == nullptr) {
    return false;
  }

  const auto& lane_ids = frame.GetReferenceLineProvider()
                             ->GetPncMap()
                             ->GetAdcPassageRoutingInfo()
                             ->GetLaneIds();
  const auto& trajectory_point =
      trajectory_points.at(trajectory_points.size() - 1);

  if (std::none_of(lane_ids.begin(), lane_ids.end(), [&](const auto& lane_id) {
        return lane_id == trajectory_point.path_point().lane_id();
      })) {
    return false;
  }

  const auto& envelope = trajectory_envelope.back();
  const auto lane = hdmap::HDMapUtil::MapForPlanning().GetLaneById(
      hdmap::MakeMapId(trajectory_point.path_point().lane_id()));
  if (lane == nullptr) {
    return false;
  }
  double accumulate_s = 0.0;
  double lateral = 0.0;

  if (!lane->GetProjection({trajectory_point.path_point().x(),
                            trajectory_point.path_point().y()},
                           &accumulate_s, &lateral)) {
    return false;
  }

  double left_width = 0.0;
  double right_width = 0.0;
  lane->GetWidth(accumulate_s, &left_width, &right_width);
  return (envelope.center_p.l() < left_width &&
          envelope.center_p.l() > -right_width);
}

void ReverseVtSampleGenerator::CalculateFollowObstacle(
    const Frame& frame, const ReferenceLineInfo& reference_line_info,
    const common::TrajectoryPoint& init_point, SpeedCache* const cache) {
  if (cache == nullptr) {
    return;
  }

  // select current follow obstacle according to current_reference_line_info,
  // for obstacles which perception id == last_follow_obstacle_id,
  // choose obstacle which has biggest st_boundary_max_s
  auto smallest_min_s = std::numeric_limits<double>::max();
  const auto& obstacle_caches = cache->GetSafeSTObstacleCaches();
  std::string follow_obstacle_id;
  for (const auto& obstacle_cache : obstacle_caches) {
    const auto* obstacle = obstacle_cache->GetObstacle();
    if (obstacle == nullptr ||
        obstacle_cache->GetSTObstacleLocation() != STObstacleLocation::ABOVE ||
        !CheckIfFollow(frame, reference_line_info, *obstacle_cache)) {
      continue;
    }

    if (obstacle->PerceptionSLBoundary().start_s() < smallest_min_s) {
      smallest_min_s = obstacle->PerceptionSLBoundary().start_s();
      follow_obstacle_id = obstacle->Id();
    }
  }
  cache->SetFollowObstacle(frame, reference_line_info, init_point,
                           follow_obstacle_id);
  ADEBUG << "follow obstacle id:" << follow_obstacle_id;
}

void ReverseVtSampleGenerator::CalculateSafeCurves(
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

    min_cost_result_ = nullptr;
    for (std::size_t i = start_curve_index; i < end_curve_index; ++i) {
      auto& cost_result = cost_results->at(i);
      if (evaluator->CompareCostResult(&cost_result, min_cost_result_)) {
        min_cost_result_ = &cost_result;
      }
    }

    if (min_cost_result_ == nullptr) {
      continue;
    }

    slt_obstacle_cache.SetSSafeDistanceCosts(
        min_cost_result_->obstacle_s_safe_distance_costs);
    slt_obstacle_cache.SetLSafeDistanceCosts(
        min_cost_result_->obstacle_l_safe_distance_costs);
  }
}

std::shared_ptr<SpeedCurve> ReverseVtSampleGenerator::SelectBestCurve(
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

bool ReverseVtSampleGenerator::CheckIfReuseStopCurve(const SpeedCache& cache) {
  const auto& basic_cache = cache.GetBasicCache();
  return basic_cache.GetIsStopPrefinish() &&
         basic_cache.GetLastFrameBlockObstacleId() ==
             basic_cache.GetBlockObstacleId() &&
         !basic_cache.GetBlockObstacleId().empty() &&
         cache.GetSafeSTObstacleCaches().empty() &&
         last_cost_result_.curve != nullptr &&
         last_cost_result_.curve->GetTarget().mode ==
             SpeedCurveTarget::Mode::STOP &&
         !std::isinf(last_cost_result_.total_cost);
}

bool ReverseVtSampleGenerator::GenerateSpeedData(
    const common::TrajectoryPoint& init_point,
    const std::shared_ptr<SpeedCurve>& curve, SpeedData* const speed_data) {
  UNUSED(init_point);
  if (curve == nullptr || speed_data == nullptr) {
    return false;
  }
  speed_data->clear();
  last_cost_result_.curve = curve->Clone();
  last_cost_result_.curve->Discretize(FLAGS_trajectory_time_length,
                                      FLAGS_trajectory_time_resolution,
                                      speed_data);
  return true;
}

bool ReverseVtSampleGenerator::GenerateStandStillSpeedData(
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

bool ReverseVtSampleGenerator::CheckIfDangerous(
    const SpeedCache& cache, const std::shared_ptr<SpeedCurve>& curve) {
  return CheckIfDangerousWithStaticObstacles(cache, curve) ||
         CheckIfDangerousWithDynamicObstacles(cache, curve);
}

bool ReverseVtSampleGenerator::CheckIfDangerousWithStaticObstacles(
    const SpeedCache& cache, const std::shared_ptr<SpeedCurve>& curve) {
  if (curve == nullptr) {
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

bool ReverseVtSampleGenerator::CheckIfDangerousWithDynamicObstacles(
    const SpeedCache& cache, const std::shared_ptr<SpeedCurve>& curve) {
  UNUSED(cache);
  UNUSED(curve);
  // if (curve == nullptr || min_cost_result_ == nullptr ||
  //     min_cost_result_->curve == nullptr) {
  //   return true;
  // }

  // if (min_cost_result_->curve->GetTarget().mode ==
  //     SpeedCurveTarget::Mode::FOLLOW) {
  //   double min_end_time = std::numeric_limits<double>::max();
  //   for (const auto& cost_result :
  //        current_sampler_->GetNormalCurveCostResults()) {
  //     if (cost_result.curve != nullptr &&
  //         cost_result.curve->GetTarget().mode ==
  //             SpeedCurveTarget::Mode::FOLLOW &&
  //         cost_result.curve->GetEndTime() < min_end_time) {
  //       min_end_time = cost_result.curve->GetEndTime();
  //     }
  //   }

  //   if (common::math::double_type::SeemsEqual(
  //           min_end_time, min_cost_result_->curve->GetEndTime())) {
  //     return true;
  //   }
  // }
  return false;
}

std::string ReverseVtSampleGenerator::PrintCurveInfo(
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

void ReverseVtSampleGenerator::RecordDebug(
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

}  // namespace planning
}  // namespace TL
