/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  planning path lane keep bound processor
 * Author: ROC
 */

#include "planning/tasks/deciders/path_bounds_decider/bound_processor/lane_keep_bound_processor.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <set>
#include <tuple>
#include <utility>

#include "common/time/clock.h"
#include "planning/common/obstacle_blocking_analyzer.h"
#include "planning/common/planning_gflags.h"
#include "planning/tasks/deciders/path_bounds_decider/util/path_info.h"
#include "planning/tasks/deciders/utils/path_decider_obstacle_utils.h"

#include "common/util/point_factory.h"

namespace TL {
namespace planning {

using TL::common::Clock;
using TL::common::ErrorCode;
using TL::common::Status;
using TL::common::VehicleConfigHelper;
using TL::planning::PathInfo;
using perception::FreeSpaceOutArray;
#ifdef BOUNDARY_PLOT
std::unique_ptr<PyPlot> LaneKeepBoundProcessor::py_plot_;
#endif
namespace {
constexpr double kEpsilon = 1e-5;
constexpr double kHalfOne = 0.5;
constexpr size_t kNumHead = 2;
constexpr double kTriangleHeight = 5.0;
constexpr double kFreeSpaceOnRefLineFilterDist = 1.0;
constexpr double kPathBoundPointBuffer = 0.15;  // node_reasonable_buffer = 0.1
}  // namespace

LaneKeepBoundProcessor::LaneKeepBoundProcessor(
    const std::shared_ptr<DependencyInjector>& injector,
    const TaskConfig& config)
    : BoundProcessor(injector, config),
      process_bound_(new ProcessBound(injector, config)),
      obs_static_process_(new ObsStaticProcessor(injector, config)),
      obs_dynamic_processor_(new ObsDynamicProcessor(injector, config)) {
#ifdef BOUNDARY_PLOT
  py_plot_ = std::make_unique<PyPlot>("/home/jding/SenseAD/data/");
#endif
}

Status LaneKeepBoundProcessor::Process(
    ReferenceLineInfo* const reference_line_info, PathBound* const path_bound,
    Frame* const frame, std::vector<LaneType>* const lane_type_pool) {
  if (reference_line_info == nullptr || path_bound == nullptr ||
      frame == nullptr) {
    const std::string msg =
        "reference_line_info or path_bound or frame is nullptr.";
    AERROR << msg;
    return Status(ErrorCode::PLANNER_CRUISING_PATHBOUNDS_ERROR, msg);
  }

  // bound process init.
  process_bound_->InitPathBounds(frame, reference_line_info);
  std::vector<double> towing_line;
  towing_line.reserve(path_bound->size());

  // 1. Initialize the path boundaries to be an indefinitely large area.
  if (!process_bound_->InitPathBoundary(path_bound, GetInjector())) {
    const std::string msg = "Failed to initialize path boundaries.";
    AERROR << msg;
    return Status(ErrorCode::PLANNER_CRUISING_PATHBOUNDS_ERROR, msg);
  }

  process_bound_->SetIsAllowLeftVirtualLaneBound(false);
  process_bound_->SetIsAllowRightVirtualLaneBound(false);
  process_bound_->SetIsAllowExpandLeftLaneBound(false);
  process_bound_->SetIsAllowExpandRightLaneBound(false);

  // 2. Get boundary from lanes and ADC for lane keep path bound
  if (!process_bound_->GetBoundaryFromLanesAndADC(
          lane_borrow_info_, PathBoundType::LANE_KEEP_PATH_BOUND, path_bound,
          borrow_lane_type_,
          GetConfig().path_bounds_decider_config().adc_lane_keep_buffer(),
          lane_type_pool)) {
    const std::string msg =
        "Failed to decide a rough boundary based on "
        "road information.";
    AERROR << msg;
    return Status(ErrorCode::PLANNER_CRUISING_PATHBOUNDS_ERROR, msg);
  }

  PathInfo::PathBoundsDebugString(*path_bound);

  // limit the path boundary based on the actual road boundary to avoid getting off-road.
  if (frame->GetMachineStateType() !=
      functionmanager::MachineStateType::HISTORY_TRACE_TYPE) {
    if (obs_static_process_ == nullptr || obs_dynamic_processor_ == nullptr) {
      const std::string msg =
          "Failed process lane keep path bound, obs_static_process_ or "
          "obs_dynamic_processor_ ptr is nullptr.";
      AERROR << msg;
      return Status(ErrorCode::PLANNER_CRUISING_PATHBOUNDS_ERROR, msg);
    }

    obs_static_process_->GetProcessBound()->InitPathBounds(frame,
                                                           reference_line_info);

    const auto& indexed_obstacles =
        reference_line_info->path_decision()->obstacles();
    obs_static_process_->SortObstaclesForSweepLine(*reference_line_info,
                                                   indexed_obstacles);

    process_bound_->IsAllowVirtualLaneBound(
        *path_bound, obs_static_process_->GetObstacleEdges());
    for (auto& path_bound_point : *path_bound) {
      std::get<2>(path_bound_point) = std::numeric_limits<double>::max();
      std::get<1>(path_bound_point) = std::numeric_limits<double>::lowest();
    }
    if (!process_bound_->GetBoundaryFromLanesAndADC(
            lane_borrow_info_, PathBoundType::LANE_KEEP_PATH_BOUND, path_bound,
            borrow_lane_type_,
            GetConfig().path_bounds_decider_config().adc_lane_keep_buffer(),
            lane_type_pool)) {
      const std::string msg =
          "Failed to decide a rough boundary based on "
          "road information.";
      AERROR << msg;
      return Status(ErrorCode::PLANNER_CRUISING_PATHBOUNDS_ERROR, msg);
    }

    process_bound_->IsAllowExpendLaneBound(
        *path_bound, obs_static_process_->GetObstacleEdges());
    for (auto& path_bound_point : *path_bound) {
      std::get<2>(path_bound_point) = std::numeric_limits<double>::max();
      std::get<1>(path_bound_point) = std::numeric_limits<double>::lowest();
    }
    if (!process_bound_->GetBoundaryFromLanesAndADC(
            lane_borrow_info_, PathBoundType::LANE_KEEP_PATH_BOUND, path_bound,
            borrow_lane_type_,
            GetConfig().path_bounds_decider_config().adc_lane_keep_buffer(),
            lane_type_pool)) {
      const std::string msg =
          "Failed to decide a rough boundary based on "
          "road information.";
      AERROR << msg;
      return Status(ErrorCode::PLANNER_CRUISING_PATHBOUNDS_ERROR, msg);
    }

#ifdef BOUNDARY_PLOT
    PrepareBoundaryData(*reference_line_info, frame, path_bound, 0);
#endif
    // 3. occ path bound process
    if (fLB::FLAGS_use_freespace_process_bound) {
      ProcessBoundWithFreeSpace(*reference_line_info, frame, path_bound);
    }
#ifdef BOUNDARY_PLOT
    PrepareBoundaryData(*reference_line_info, frame, path_bound, 1);
    PlotData();
#endif

    TowingPointsInfo towing_points;
    towing_points.resize(
        path_bound->size(),
        std::tuple<double, double, double, std::string>(0.0, 0.0, 0.0, "0"));
    // 3. Fine-tune the boundary based on static obstacles
    if (!obs_static_process_->Process(reference_line_info, frame, path_bound,
                                      blocking_obstacle_id_, &towing_points,
                                      true)) {
      const std::string msg =
          "Failed to decide fine tune the boundaries after "
          "taking into consideration all static obstacles.";
      AERROR << msg;
      return Status(ErrorCode::PLANNER_CRUISING_PATHBOUNDS_ERROR, msg);
    }

    // 4. Fine-tune the boundary based on certain dynamic obstacles.
    PathBound temp_path_bound = *path_bound;
    const double start_timestamp = Clock::NowInSeconds();
    obs_dynamic_processor_->DynamicObsInit(
        GetConfig().path_bounds_decider_config().adc_lane_keep_buffer());
    if (!obs_dynamic_processor_->Process(
            reference_line_info, process_bound_->GetMutableFrame(), path_bound,
            blocking_obstacle_id_, &towing_points, true)) {
      const std::string msg =
          "Failed to decide fine tune the boundaries after "
          "taking into consideration certain dynamic obstacles.";
      AERROR << msg;
      path_bound->swap(temp_path_bound);
    }
    const double time_diff_ms =
        (Clock::NowInSeconds() - start_timestamp) * 1000;
    obs_dynamic_processor_->RecordElapseTimeInfo(
        reference_line_info, "DYNAMIC_OBSTACLE_CONSTRAINT_PROCESSOR",
        time_diff_ms);

    // 5.steering wheel angle speed limit bound process.
    if (frame->GetMachineStateType() !=
            functionmanager::MachineStateType::PERCEPTION_TYPE &&
        GetConfig()
            .path_bounds_decider_config()
            .steering_wheel_angle_speed_limit_config()
            .enable_steering_wheel_angle_speed_limit()) {
      if (!process_bound_->SteeringWheelSpeedLimitBoundProcess(path_bound)) {
        AERROR << "Steering wheel speed limit bound process error.";
      }
    }
    // Append some extra path bound points to avoid zero-length path data.
    int counter = 0;
    while (!blocking_obstacle_id_->empty() &&
           path_bound->size() < temp_path_bound.size() &&
           counter < kNumExtraTailBoundPoint) {
      path_bound->push_back(temp_path_bound[path_bound->size()]);
      counter++;
    }
    // 6. Calculate and process towing line
    std::vector<double> towing_line;
    towing_line.resize(towing_points.size());
    TowingPointsFilterProcess(reference_line_info, &towing_points,
                              &towing_line);
    if (FLAGS_enable_towing_line_debug) {
      PathInfo::TowingLineDebugInfo(*path_bound, towing_line,
                                    reference_line_info);
    }
    reference_line_info->SetTowingLine(&towing_line);
  }

  ADEBUG << "Get boundary from obstacles for lane keep path bound.";
  PathInfo::PathBoundsDebugString(*path_bound);

  PathInfo::PathBoundDebugInfo(PathBoundType::LANE_KEEP_PATH_BOUND, *path_bound,
                               reference_line_info);
  return Status::OK();
}

void LaneKeepBoundProcessor::ProcessBoundWithFreeSpace(
    const ReferenceLineInfo& reference_line_info, Frame* const frame,
    PathBound* const path_bound) {
  if (path_bound == nullptr) {
    AERROR << "path_bound == nullptr!";
    return;
  }
  std::vector<common::math::LineSegment2d> freespace_line_segments;
  if (!ConvertFreeSpaceToLineSegments(frame, &freespace_line_segments)) {
    ADEBUG << "no freesapce!";
    return;
  }
  if (freespace_line_segments.size() < 2) {
    ADEBUG << "freespace_line_segments not enough!";
  }
  const double adc_start_l = reference_line_info.AdcSlBoundary().start_l();
  const double adc_end_l = reference_line_info.AdcSlBoundary().end_l();
  const double half_adc_width =
      VehicleConfigHelper::GetConfig().vehicle_param().width() / 2;
  constexpr double kFreeSpaceBuffer = 0.3;
  const auto front_delta_index = static_cast<size_t>(lround(
      common::VehicleConfigHelper::GetConfig().vehicle_param().wheel_base() /
          reference_line_info.PathBoundsDeciderResolution() +
      kHalfOne));
#ifdef BOUNDARY_PLOT
  std::vector<common::math::Polygon2d> triangles_vec = {};
#endif
  for (size_t i = 0; i < path_bound->size() / 2; ++i) {
    auto& path_bound_i = path_bound->at(i);
    common::math::Polygon2d path_bound_up_triangle;
    common::math::Polygon2d path_bound_down_triangle;
    const double path_bound_triangle_bottom_angle = 20.0 / 180.0 * M_PI;
    auto common_point =
        reference_line_info.reference_line()
            .GetReferencePointForGreaterThanRefMaxS(std::get<0>(path_bound_i));

    if (!DoubleTriangleConstructor(
            common_point, kTriangleHeight, path_bound_triangle_bottom_angle,
            &path_bound_up_triangle, &path_bound_down_triangle)) {
      continue;
    }
#ifdef BOUNDARY_PLOT
    triangles_vec.push_back(path_bound_up_triangle);
    triangles_vec.push_back(path_bound_down_triangle);
#endif
    const common::math::Vec2d triangle_height_uint =
        common::math::Vec2d::CreateUnitVec2d(common_point.heading() + M_PI_2);
    const double path_bound_bottom_edge_width =
        std::fabs(triangle_height_uint.CrossProd(
            path_bound_up_triangle.points().at(1) - common_point));

    for (const auto& line_segment : freespace_line_segments) {
      const double line_segment_to_point_distance =
          line_segment.DistanceTo(common_point);

      if (line_segment_to_point_distance < kFreeSpaceOnRefLineFilterDist) {
        continue;
      }

      if (!CalculateFreeSpaceFlag(line_segment, common_point,
                                  triangle_height_uint,
                                  path_bound_bottom_edge_width)) {
        if (i <= front_delta_index + kNumHead) {
          // near the host vehicle
          if (path_bound_up_triangle.HasOverlap(line_segment)) {
            const double left_bound =
                line_segment_to_point_distance - kFreeSpaceBuffer;
            process_bound_->UpdateObstaclePathBoundaryWithBuffer(
                i, left_bound, std::numeric_limits<double>::lowest(),
                path_bound);
          } else if (path_bound_down_triangle.HasOverlap(line_segment)) {
            const double right_bound =
                -line_segment_to_point_distance + kFreeSpaceBuffer;
            process_bound_->UpdateObstaclePathBoundaryWithBuffer(
                i, std::numeric_limits<double>::max(), right_bound, path_bound);
          }
        } else {
          // far away from the host vehicle
          if (path_bound_up_triangle.HasOverlap(line_segment)) {
            // const double l_max_tmp = std::get<2>(path_bound_i);
            const double left_bound =
                line_segment_to_point_distance - kFreeSpaceBuffer;
            process_bound_->UpdateObstaclePathBoundaryWithBuffer(
                i, left_bound, std::numeric_limits<double>::lowest(),
                path_bound);
            // extend several point to pre-process bound
            // if (std::abs(l_max_tmp - std::get<2>(path_bound_i)) > 0) {
            //   for (int j = 1; j < 5; ++j) {
            //     int max_index = std::max(static_cast<int>(i - j), 0);
            //     int min_index = std::min(static_cast<int>(i + j),
            //                              static_cast<int>(path_bound->size()));
            //     std::get<2>(path_bound->at(max_index)) =
            //         std::get<2>(path_bound_i);
            //     std::get<2>(path_bound->at(min_index)) =
            //         std::get<2>(path_bound_i);
            //   }
            // }
          } else if (path_bound_down_triangle.HasOverlap(line_segment)) {
            // const double l_min_tmp = std::get<1>(path_bound_i);
            const double right_bound =
                -line_segment_to_point_distance + kFreeSpaceBuffer;
            process_bound_->UpdateObstaclePathBoundaryWithBuffer(
                i, std::numeric_limits<double>::max(), right_bound, path_bound);
            // extend several point to pre-process bound
            // if (std::abs(l_min_tmp - std::get<1>(path_bound_i)) > 0) {
            //   for (int j = 1; j < 5; ++j) {
            //     int max_index = std::max(static_cast<int>(i - j), 0);
            //     int min_index = std::min(static_cast<int>(i + j),
            //                              static_cast<int>(path_bound->size()));
            //     std::get<1>(path_bound->at(max_index)) =
            //         std::get<1>(path_bound_i);
            //     std::get<1>(path_bound->at(min_index)) =
            //         std::get<1>(path_bound_i);
            //   }
            // }
          }
        }
      }
    }
  }
#ifdef BOUNDARY_PLOT
  common::PathPoint vehicle_state;
  vehicle_state.set_x(reference_line_info.vehicle_state().x());
  vehicle_state.set_y(reference_line_info.vehicle_state().y());
  vehicle_state.set_theta(reference_line_info.vehicle_state().heading());

  common::math::Box2d vehicle_line =
      common::VehicleConfigHelper::GetBoundingBox(vehicle_state);
  PrepareLineAndTriData(frame, &freespace_line_segments, triangles_vec,
                        vehicle_line);
#endif
}

bool LaneKeepBoundProcessor::ConvertFreeSpaceToLineSegments(
    Frame* const frame,
    std::vector<common::math::LineSegment2d>* const freespace_line_segments) {
  if (freespace_line_segments == nullptr) {
    return false;
  }

  if (!frame->local_view().HasFreeSpaceOutArray()) {
    return false;
  }
  const double kLongLineSegmentFilterBuffer = 20.0;
  std::shared_ptr<const FreeSpaceOutArray> freespace_out_array =
      frame->local_view().GetFreeSpaceOutArray();
  for (int i = 1; i < freespace_out_array->freespace_out().size(); ++i) {
    const auto& freespace_out = freespace_out_array->freespace_out().at(i);
    const auto& freespace_keypoints = freespace_out.freespace_point();
    const int freespace_keypoints_size = freespace_keypoints.size();
    if (freespace_keypoints_size < 2) {
      continue;
    }
    for (int j = 1; j < freespace_keypoints_size; ++j) {
      common::math::LineSegment2d line_segment = common::math::LineSegment2d(
          {freespace_keypoints[j].x(), freespace_keypoints[j].y()},
          {freespace_keypoints[j - 1].x(), freespace_keypoints[j - 1].y()});
      if (line_segment.length() > kLongLineSegmentFilterBuffer) {
        continue;
      }
      freespace_line_segments->emplace_back(line_segment);
    }
  }
  return !freespace_line_segments->empty();
}

bool LaneKeepBoundProcessor::CalculateFreeSpaceFlag(
    const common::math::LineSegment2d& line_segment,
    const TL::planning::ReferencePoint& common_point,
    const common::math::Vec2d& triangle_height_uint,
    const double half_bottom_edge_width) {
  const auto vec_a = line_segment.start() - common_point;
  const auto vec_b = line_segment.end() - common_point;
  const double cross_prod_a = triangle_height_uint.CrossProd(vec_a);
  const double cross_prod_b = triangle_height_uint.CrossProd(vec_b);
  const bool is_left_or_right_line_segment =
      std::fabs(cross_prod_a) > half_bottom_edge_width &&
      std::fabs(cross_prod_b) > half_bottom_edge_width &&
      cross_prod_a * cross_prod_b > 0.0;

  const double inner_prod_a = triangle_height_uint.InnerProd(vec_a);
  const double inner_prod_b = triangle_height_uint.InnerProd(vec_b);
  const bool is_up_or_down_line_segment =
      std::fabs(inner_prod_a) > kTriangleHeight &&
      std::fabs(inner_prod_b) > kTriangleHeight &&
      inner_prod_a * inner_prod_b > 0.0;

  return is_left_or_right_line_segment || is_up_or_down_line_segment;
}

bool LaneKeepBoundProcessor::DoubleTriangleConstructor(
    const ReferencePoint& common_point, const double triangle_height,
    const double bottom_angle, common::math::Polygon2d* const up_triangle,
    common::math::Polygon2d* const down_triangle) {
  if (up_triangle == nullptr || down_triangle == nullptr) {
    return false;
  }

  const double edge_length =
      std::fabs(triangle_height / std::fmax(std::sin(bottom_angle), kEpsilon));

  // 计算上三角形点
  const common::math::Vec2d left_up_edge_unit =
      common::math::Vec2d::CreateUnitVec2d(common_point.heading() + M_PI -
                                           bottom_angle);
  const common::math::Vec2d left_up_edge_vec = left_up_edge_unit * edge_length;
  const common::math::Vec2d left_up_point = {
      common_point.x() + left_up_edge_vec.x(),
      common_point.y() + left_up_edge_vec.y()};

  const common::math::Vec2d right_up_edge_unit =
      common::math::Vec2d::CreateUnitVec2d(common_point.heading() +
                                           bottom_angle);
  const common::math::Vec2d right_up_edge_vec =
      right_up_edge_unit * edge_length;
  const common::math::Vec2d right_up_point = {
      common_point.x() + right_up_edge_vec.x(),
      common_point.y() + right_up_edge_vec.y()};

  // 计算下三角形角点
  const common::math::Vec2d left_down_edge_unit =
      common::math::Vec2d::CreateUnitVec2d(common_point.heading() - M_PI +
                                           bottom_angle);
  const common::math::Vec2d left_down_edge_vec =
      left_down_edge_unit * edge_length;
  const common::math::Vec2d left_down_point = {
      common_point.x() + left_down_edge_vec.x(),
      common_point.y() + left_down_edge_vec.y()};

  const common::math::Vec2d right_down_edge_unit =
      common::math::Vec2d::CreateUnitVec2d(common_point.heading() -
                                           bottom_angle);
  const common::math::Vec2d right_down_edge_vec =
      right_down_edge_unit * edge_length;
  const common::math::Vec2d right_down_point = {
      common_point.x() + right_down_edge_vec.x(),
      common_point.y() + right_down_edge_vec.y()};

  *up_triangle = common::math::Polygon2d(
      {{common_point.x(), common_point.y()}, right_up_point, left_up_point});
  *down_triangle =
      common::math::Polygon2d({{common_point.x(), common_point.y()},
                               right_down_point,
                               left_down_point});
  return true;
}

bool LaneKeepBoundProcessor::BoundInit(
    const PathInfo::LaneBorrowInfo& lane_borrow_info,
    std::string* const blocking_obstacle_id,
    std::string* const borrow_lane_type) {
  if (blocking_obstacle_id == nullptr || borrow_lane_type == nullptr) {
    AERROR << "blocking_obstacle_id or borrow_lane_type is nullptr";
    return false;
  }

  lane_borrow_info_ = lane_borrow_info;
  blocking_obstacle_id_ = blocking_obstacle_id;
  borrow_lane_type_ = borrow_lane_type;
  return true;
}

void LaneKeepBoundProcessor::TowingPointsFilterProcess(
    ReferenceLineInfo* const reference_line_info,
    TowingPointsInfo* const towing_points,
    std::vector<double>* const towing_line) {
  if (reference_line_info == nullptr || towing_points == nullptr ||
      towing_line == nullptr) {
    AERROR << "TowingPointsFilterProcess nullptr check is failed!";
    return;
  }
  if (towing_points->empty() || towing_line->empty()) {
    AERROR << "towing_points or towing_line is empty.";
    return;
  }
  const int n = static_cast<int>(towing_points->size());
  int towing_l_peak_point_cnt = 0;  // 突变值的个数
  int towing_l_peak_start = 0;      // 起点
  bool is_positive_error = false;
  // 从第0个数据点开始遍历
  for (int i = 0; i < n - 1; ++i) {
    ADEBUG << "curr index i = " << i
           << ", towing_l_peak_start = " << towing_l_peak_start
           << ", towing_l_peak_point_cnt = " << towing_l_peak_point_cnt
           << ", towing_l_min = " << std::get<0>(towing_points->at(i))
           << ", towing_l_max = " << std::get<1>(towing_points->at(i));
    towing_line->at(i) =
        std::get<1>(towing_points->at(i)) + std::get<2>(towing_points->at(i));
    const double next_towing_value = std::get<1>(towing_points->at(i + 1)) +
                                     std::get<2>(towing_points->at(i + 1));
    const double curr_and_next_err = next_towing_value - towing_line->at(i);
    ADEBUG << "curr_and_next_err = " << curr_and_next_err;

    // process left bound
    if (towing_l_peak_point_cnt == 0 &&
        std::abs(curr_and_next_err) >
            process_bound_->GetObsTowingConf().obstacle_towing_filter_error()) {
      // 找出首个突变点 err < -0.09
      ++towing_l_peak_point_cnt;
      towing_l_peak_start = i;                    // 起点
      is_positive_error = curr_and_next_err > 0;  // 第一次误差是上升还是下降
      ADEBUG << "first peak value index = " << towing_l_peak_start + 1
             << ", towing_l_peak_point_cnt = " << towing_l_peak_point_cnt;
    } else if (towing_l_peak_point_cnt > 0) {
      if (((is_positive_error &&
            curr_and_next_err < -process_bound_->GetObsTowingConf()
                                     .obstacle_towing_filter_error()) ||
           (!is_positive_error &&
            curr_and_next_err > process_bound_->GetObsTowingConf()
                                    .obstacle_towing_filter_error())) &&
          towing_l_peak_point_cnt <=
              static_cast<int>(
                  process_bound_->GetObsTowingConf()
                      .towing_filter_process_distance_threshold() /
                  reference_line_info->PathBoundsDeciderResolution())) {
        // 形成尖峰进行并过滤
        ADEBUG << " i before filter = " << i;
        for (int j = towing_l_peak_start + 1; j <= i + 1; ++j) {
          ADEBUG << "filter index = " << j;
          towing_line->at(j) = towing_line->at(towing_l_peak_start);
          std::get<1>(towing_points->at(j)) =
              std::get<1>(towing_points->at(towing_l_peak_start));
          std::get<2>(towing_points->at(j)) =
              std::get<2>(towing_points->at(towing_l_peak_start));
        }
        i = std::max(towing_l_peak_start - 2, 0);
        towing_l_peak_point_cnt = 0;
        ADEBUG << " i after filter = " << i;
      } else if (((is_positive_error &&
                   curr_and_next_err > process_bound_->GetObsTowingConf()
                                           .obstacle_towing_filter_error()) ||
                  (!is_positive_error &&
                   curr_and_next_err < -process_bound_->GetObsTowingConf()
                                            .obstacle_towing_filter_error())) &&
                 towing_l_peak_point_cnt <=
                     static_cast<int>(
                         process_bound_->GetObsTowingConf()
                             .towing_filter_process_distance_threshold() /
                         reference_line_info->PathBoundsDeciderResolution())) {
        // 递增或者递减的情况下 不断更新起点
        towing_l_peak_start = i;
        towing_l_peak_point_cnt = 1;
        ADEBUG << "new start = " << towing_l_peak_start + 1
               << ", towing_l_peak_point_cnt = " << towing_l_peak_point_cnt;
      } else if (towing_l_peak_point_cnt >
                 static_cast<int>(
                     (process_bound_->GetObsTowingConf()
                          .towing_filter_process_distance_threshold() /
                      reference_line_info->PathBoundsDeciderResolution()))) {
        // 超过阈值长度未产生尖峰，之前的突变是拐点，归零重新计数。
        towing_l_peak_start = i;
        towing_l_peak_point_cnt = 0;
        ADEBUG << "not a peak, index = " << towing_l_peak_start
               << ", towing_l_peak_point_cnt = " << towing_l_peak_point_cnt;
      } else {
        // 突变后的点误差小直接计数
        ++towing_l_peak_point_cnt;
        ADEBUG << "is another peak value and towing_l_peak_point_cnt = "
               << towing_l_peak_point_cnt;
      }
    }
  }
}
#ifdef BOUNDARY_PLOT
void LaneKeepBoundProcessor::PrepareLineAndTriData(
    Frame* frame,
    std::vector<common::math::LineSegment2d>* const freespace_line_segments,
    const std::vector<common::math::Polygon2d>& triangles_vec,
    const common::math::Box2d& vehicle_line) {
  py_plot_->SetLineSegment(*freespace_line_segments);
  py_plot_->SetTrianglePolygon(triangles_vec);
  py_plot_->SetVehicleState(vehicle_line);
  py_plot_->ResetCounter(frame->SequenceNum());
}

void LaneKeepBoundProcessor::PrepareBoundaryData(
    const ReferenceLineInfo& reference_line_info, Frame* frame,
    PathBound* path_bound, int index) {
  if (frame == nullptr || path_bound == nullptr) {
    return;
  }
  std::vector<double> x_points = {};
  std::vector<double> y_points = {};
  for (int i = 0; i < path_bound->size(); i++) {
    ADEBUG << std::get<0>(path_bound->at(i)) << " "
           << std::get<1>(path_bound->at(i));
    const common::SLPoint sl_point_lower =
        TL::common::util::PointFactory::ToSLPoint(
            std::get<0>(path_bound->at(i)), std::get<1>(path_bound->at(i)));
    const common::SLPoint sl_point_upper =
        TL::common::util::PointFactory::ToSLPoint(
            std::get<0>(path_bound->at(i)), std::get<2>(path_bound->at(i)));
    common::math::Vec2d cartesian_point_lower;
    if (!reference_line_info.reference_line().SLToXY(sl_point_lower,
                                                     &cartesian_point_lower)) {
      AERROR << "Fail to convert sl point to xy point";
      return;
    }
    x_points.push_back(cartesian_point_lower.x());
    y_points.push_back(cartesian_point_lower.y());
    common::math::Vec2d cartesian_point_upper;
    if (!reference_line_info.reference_line().SLToXY(sl_point_upper,
                                                     &cartesian_point_upper)) {
      AERROR << "Fail to convert sl point to xy point";
      return;
    }
    x_points.push_back(cartesian_point_upper.x());
    y_points.push_back(cartesian_point_upper.y());
  }
  py_plot_->SetBoundary(index, x_points, y_points);
}

void LaneKeepBoundProcessor::PlotData() {
  py_plot_->Plot();
}
#endif
}  // namespace planning
}  // namespace TL
