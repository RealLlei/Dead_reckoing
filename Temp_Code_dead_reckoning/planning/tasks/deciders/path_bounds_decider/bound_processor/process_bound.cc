/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  planning path bound process
 * Author: ROC
 */

#include "planning/tasks/deciders/path_bounds_decider/bound_processor/process_bound.h"
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <set>
#include <tuple>
#include <utility>
#include "common/file/log.h"
#include "common/math/double_type.h"
#include "common/math/linear_interpolation.h"
#include "common/math/math_utils.h"
#include "common/math/vec2d.h"
#include "common/util/point_factory.h"
#include "glog/logging.h"
#include "map/hdmap/hdmap_common.h"
#include "map/hdmap/hdmap_util.h"
#include "planning/common/planning_gflags.h"
#include "planning/common/reference_line_info.h"
#include "planning/common/util/common.h"
#include "planning/tasks/deciders/path_bounds_decider/util/path_info.h"
#include "planning/tasks/deciders/utils/path_decider_obstacle_utils.h"
#include "planning/warning/lbs/common/tue_common_libs.h"
#include "proto/common/vehicle_state.pb.h"
#include "proto/map/map_road.pb.h"
#include "proto/perception/perception_obstacle.pb.h"
#include "proto/planning/decider_debug.pb.h"

namespace TL {
namespace planning {

using common::math::double_type::Compare;
using common::math::double_type::DefinitelyGreater;
using common::math::double_type::DefinitelyLess;
using TL::common::Clock;
using TL::common::VehicleConfigHelper;
using TL::common::math::double_type::ComparedToZero;
using TL::hdmap::HDMapUtil;
using TL::hdmap::LaneBoundaryType;
using TL::planning::util::IsNormalTurn;
using TL::planning::util::IsRoadCurvedSection;

ProcessBound::ProcessBound(const std::shared_ptr<DependencyInjector>& injector,
                           const TaskConfig& config)
    : injector_(injector), config_(config) {}

bool ProcessBound::InitPathBounds(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  if (frame == nullptr || reference_line_info == nullptr) {
    AERROR << "obs static process init failed.";
    return false;
  }
  if (reference_line_info->reference_line().reference_points().empty()) {
    AERROR << "reference points is empty.";
    return false;
  }

  frame_ = frame;
  reference_line_info_ = reference_line_info;

  const ReferenceLine& reference_line = reference_line_info_->reference_line();
  common::TrajectoryPoint planning_start_point = frame_->PlanningStartPoint();
  if (frame_->GetMachineStateType() ==
      functionmanager::MachineStateType::HISTORY_TRACE_TYPE) {
    planning_start_point.set_a(-planning_start_point.a());
    planning_start_point.set_v(std::fabs(planning_start_point.v()));
  }
  if (FLAGS_use_front_axe_center_in_path_planning) {
    planning_start_point =
        InferFrontAxeCenterFromRearAxeCenter(planning_start_point);
  }

  const auto& fct_out = injector_->planning_context()
                            ->mutable_planning_status()
                            ->function_manager_out();
  if (fct_out.map_road_type() == functionmanager::NCP_ROADTYPE) {
    lane_bound_conf_ = config_.path_bounds_decider_config()
                           .lane_bound_process_config()
                           .ncp_lane_bound_process();
    obs_towing_conf_ = config_.path_bounds_decider_config()
                           .obs_towing_process_config()
                           .ncp_obs_towing_process();
  } else {
    lane_bound_conf_ = config_.path_bounds_decider_config()
                           .lane_bound_process_config()
                           .nnp_lane_bound_process();
    obs_towing_conf_ = config_.path_bounds_decider_config()
                           .obs_towing_process_config()
                           .nnp_obs_towing_process();
  }

  // if (fct_out.has_fsm_state() &&
  //     fct_out.fsm_state() == functionmanager::MachineStateType::HDMAP_TYPE &&
  //     fct_out.has_localization_maptype() &&
  //     (fct_out.localization_maptype() ==
  //          navigation_hdmap::MapMsg::FUSION_NNP_MAP ||
  //      fct_out.localization_maptype() ==
  //          navigation_hdmap::MapMsg::FUSION_NCP_MAP)) {
  //   lane_bound_conf_.set_use_camera_lane_type(true);
  // } else {
  //   lane_bound_conf_.set_use_camera_lane_type(false);
  // }

  // Initialize some private variables.
  // ADC s/l info.
  auto adc_sl_info = reference_line.ToFrenetFrame(planning_start_point);
  adc_frenet_s_ = adc_sl_info.first[0];
  adc_frenet_l_ = adc_sl_info.second[0];
  adc_frenet_sd_ = adc_sl_info.first[1];
  adc_frenet_ld_ = adc_sl_info.second[1] * adc_frenet_sd_;
  start_point_dl_ = common::math::Clamp(
      adc_sl_info.second[1], -kLimitStartPointdl, kLimitStartPointdl);

  // double offset_to_map = 0.0;
  // reference_line_info_->reference_line().GetOffsetToMap(adc_frenet_s_,
  //                                                       &offset_to_map);

  if (reference_line.GetLaneWidth(adc_frenet_s_, &adc_lane_left_width_,
                                  &adc_lane_right_width_)) {
    // adc_lane_left_width_ -= offset_to_map;
    // adc_lane_right_width_ += offset_to_map;
  }

  is_clear_to_expand_left_lane_bound_ = true;
  is_clear_to_expand_right_lane_bound_ = true;
  IsClearToExpandLaneBound();

  // adc_l_to_lane_center_ = adc_frenet_l_ + offset_to_map;
  adc_l_to_lane_center_ = adc_frenet_l_;
  adc_default_lane_width_ = lane_bound_conf_.path_default_lane_width();
  adc_lane_width_ = adc_lane_left_width_ + adc_lane_right_width_;
  return true;
}

bool ProcessBound::InitPathBoundary(
    PathBound* const path_bound,
    const std::shared_ptr<DependencyInjector>& injector) {
  if (path_bound == nullptr) {
    AERROR << "path_bound is nullptr!";
    return false;
  }

  path_bound->clear();
  const auto& reference_line = reference_line_info_->reference_line();

  // Starting from ADC's current position, increment until the horizon, and
  // set lateral bounds to be infinite at every spot.

  const double init_path_start_timestamp = Clock::NowInSeconds();
  const double pnc_length = hdmap::PncMap::LookForwardDistance(
      injector->planning_context()
          ->mutable_planning_status()
          ->function_manager_out()
          .fsm_state(),
      std::fabs(reference_line_info_->vehicle_state().linear_velocity()),
      reference_line_info_->GetCruiseSpeed());
  double start_s = adc_frenet_s_;
  double end_s = fmin(
      adc_frenet_s_ + pnc_length,
      reference_line.Length() - config_.path_bounds_decider_config()
                                    .path_end_distance_from_reference_end());
  if (frame_->GetMachineStateType() ==
      functionmanager::MachineStateType::HISTORY_TRACE_TYPE) {
    ReversePathBound(pnc_length, path_bound);
  } else {
    const int length_iter =
        std::ceil((end_s - start_s) /
                  reference_line_info_->PathBoundsDeciderResolution());
    for (int curr_s_i = 0; curr_s_i < length_iter; ++curr_s_i) {
      path_bound->emplace_back(
          start_s +
              curr_s_i * reference_line_info_->PathBoundsDeciderResolution(),
          std::numeric_limits<double>::lowest(),
          std::numeric_limits<double>::max());
    }
  }
  // Return.
  if (path_bound->empty()) {
    AERROR << "adc_s: " << adc_frenet_s_
           << ", reference_length: " << reference_line.Length()
           << ", Empty path boundary in InitPathBoundary";
    return false;
  }
  const double init_path_end_timestamp = Clock::NowInSeconds();
  ADEBUG << "start_s: " << std::get<0>(path_bound->front())
         << ", end_s: " << std::get<0>(path_bound->back())
         << ", adc_frenet_s_: " << adc_frenet_s_
         << ", pnc_length: " << pnc_length;
  ADEBUG << "-------------InitPathBoundary--------- time =  "
         << (init_path_end_timestamp - init_path_start_timestamp);

  return true;
}

// TODO(ROC): this function is to be retired soon.
bool ProcessBound::GetBoundaryFromLanesAndADC(
    const PathInfo::LaneBorrowInfo& lane_borrow_info,
    const PathBoundType& path_bound_type, PathBound* const path_bound,
    std::string* const borrow_lane_type, const double ADC_buffer,
    std::vector<LaneType>* const lane_type_pool,
    const bool is_fallback_lanechange) {
  // Sanity checks.
  if (path_bound == nullptr || borrow_lane_type == nullptr ||
      path_bound->empty()) {
    AERROR << "path_bound or borrow_lane_type is nullptr or empty!";
    return false;
  }

  const ReferenceLine& reference_line = reference_line_info_->reference_line();
  const auto& hdmap = hdmap::HDMapUtil::MapForPlanning();

  // Go through every point, update the boundary based on lane info and
  // ADC's position.
  double past_lane_left_width = adc_default_lane_width_ / 2;
  double past_lane_right_width = adc_default_lane_width_ / 2;
  int path_blocked_idx = -1;
  bool borrowing_reverse_lane = false;
  std::vector<double> curr_lane_left_widths(path_bound->size());
  std::vector<double> curr_lane_right_widths(path_bound->size());
  std::vector<double> curr_neighbor_lane_widths(path_bound->size());
  std::vector<double> ADC_speed_buffers(path_bound->size());
  std::vector<double> ADC_buffers(path_bound->size());
  std::vector<double> curr_left_bound_lanes(path_bound->size());
  std::vector<double> curr_right_bound_lanes(path_bound->size());
  std::vector<double> curr_left_bounds(path_bound->size());
  std::vector<double> curr_right_bounds(path_bound->size());

  // check whether lane_type_pool is empty.
  CheckLaneTypePool(*path_bound, lane_type_pool);

  for (size_t i = 0; i < path_bound->size(); ++i) {
    double curr_s = std::get<0>((*path_bound)[i]);
    // 1. Get the current lane width at current point.
    double curr_lane_left_width = 0.0;
    double curr_lane_right_width = 0.0;

    // judge whether road line is lane line
    bool is_left_lane_boundary = true;
    bool is_right_lane_boundary = true;
    const auto& curr_path_point = reference_line.GetReferencePoint(curr_s);
    for (const auto& lane_waypoint : curr_path_point.lane_waypoints()) {
      if (lane_waypoint.lane == nullptr) {
        ADEBUG << "lane_waypoint.lane is nullptr.";
        continue;
      }
      if (lane_waypoint.lane->lane().left_neighbor_forward_lane_id().empty()) {
        is_left_lane_boundary = false;
      } else {
        ADEBUG << "left_neighbor_lane id = "
               << lane_waypoint.lane->lane()
                      .left_neighbor_forward_lane_id()
                      .at(0)
                      .id();
        auto left_neighbor_lane = hdmap.GetLaneById(
            lane_waypoint.lane->lane().left_neighbor_forward_lane_id().at(0));
        if (left_neighbor_lane == nullptr) {
          is_left_lane_boundary = false;
        }
      }
      if (lane_waypoint.lane->lane().right_neighbor_forward_lane_id().empty()) {
        is_right_lane_boundary = false;
      } else {
        ADEBUG << "right_neighbor_lane id = "
               << lane_waypoint.lane->lane()
                      .right_neighbor_forward_lane_id()
                      .at(0)
                      .id();
        auto right_neighbor_lane = hdmap.GetLaneById(
            lane_waypoint.lane->lane().right_neighbor_forward_lane_id().at(0));
        if (right_neighbor_lane == nullptr) {
          is_right_lane_boundary = false;
        }
      }
    }

    // double offset_to_lane_center = 0.0;
    if (!reference_line.GetLaneWidth(curr_s, &curr_lane_left_width,
                                     &curr_lane_right_width)) {
      AWARN << "Failed to get lane width at s = " << curr_s;
      curr_lane_left_width = past_lane_left_width;
      curr_lane_right_width = past_lane_right_width;
    } else {
      // check if lane boundary is also road boundary
      // reference_line.GetOffsetToMap(curr_s, &offset_to_lane_center);
      // curr_lane_left_width -= offset_to_lane_center;
      // curr_lane_right_width += offset_to_lane_center;
      past_lane_left_width = curr_lane_left_width;
      past_lane_right_width = curr_lane_right_width;
    }

    // 2. Get the neighbor lane widths at the current point.
    //    Get the proper lane boundary.
    double curr_neighbor_lane_width = 0.0;
    double curr_left_bound_lane = curr_lane_left_width;
    double curr_right_bound_lane = -curr_lane_right_width;
    if (FLAGS_enable_original_lane_borrow_process) {
      if (CheckLaneBoundaryType(*reference_line_info_, curr_s,
                                lane_borrow_info)) {
        hdmap::Id neighbor_lane_id;
        if (lane_borrow_info == PathInfo::LaneBorrowInfo::LEFT_BORROW) {
          // Borrowing left neighbor lane.
          if (reference_line_info_->GetNeighborLaneInfo(
                  ReferenceLineInfo::LaneType::LeftForward, curr_s,
                  &neighbor_lane_id, &curr_neighbor_lane_width)) {
            ADEBUG << "Borrow left forward neighbor lane.";
          } else if (reference_line_info_->GetNeighborLaneInfo(
                         ReferenceLineInfo::LaneType::LeftReverse, curr_s,
                         &neighbor_lane_id, &curr_neighbor_lane_width)) {
            borrowing_reverse_lane = true;
            ADEBUG << "Borrow left reverse neighbor lane.";
          } else {
            ADEBUG << "There is no left neighbor lane.";
          }
        } else if (lane_borrow_info == PathInfo::LaneBorrowInfo::RIGHT_BORROW) {
          // Borrowing right neighbor lane.
          if (reference_line_info_->GetNeighborLaneInfo(
                  ReferenceLineInfo::LaneType::RightForward, curr_s,
                  &neighbor_lane_id, &curr_neighbor_lane_width)) {
            ADEBUG << "Borrow right forward neighbor lane.";
          } else if (reference_line_info_->GetNeighborLaneInfo(
                         ReferenceLineInfo::LaneType::RightReverse, curr_s,
                         &neighbor_lane_id, &curr_neighbor_lane_width)) {
            borrowing_reverse_lane = true;
            ADEBUG << "Borrow right reverse neighbor lane.";
          } else {
            ADEBUG << "There is no right neighbor lane.";
          }
        }
      }
      curr_left_bound_lane +=
          (lane_borrow_info == PathInfo::LaneBorrowInfo::LEFT_BORROW
               ? curr_neighbor_lane_width
               : 0.0);
      curr_right_bound_lane -=
          (lane_borrow_info == PathInfo::LaneBorrowInfo::RIGHT_BORROW
               ? curr_neighbor_lane_width
               : 0.0);
    }

    // 3. Calculate the proper boundary based on lane-width, ADC's position,
    //    and ADC's velocity.
    static constexpr double kMaxLateralAccelerations = 1.5;
    // double offset_to_map = 0.0;
    // reference_line.GetOffsetToMap(curr_s, &offset_to_map);

    double ADC_speed_buffer = (adc_frenet_ld_ > 0 ? 1.0 : -1.0) *
                              adc_frenet_ld_ * adc_frenet_ld_ /
                              kMaxLateralAccelerations / 2;

    LaneType lane_type;
    // lane_type reuse
    if (path_bound_type != PathBoundType::LANE_CHANGE_PATH_BOUND &&
        i < lane_type_pool->size()) {
      lane_type = lane_type_pool->at(i);
    } else {
      // otherwise, check lane type.
      CheckLaneSolidType(*reference_line_info_, curr_s, &lane_type);
    }

    double curr_left_bound = 0.0;
    double curr_right_bound = 0.0;
    if (config_.path_bounds_decider_config()
            .is_extend_lane_bounds_to_include_adc() ||
        is_fallback_lanechange) {
      // extend path bounds to include ADC in fallback or change lane path
      // bounds.
      double curr_left_bound_adc =
          std::fmax(adc_l_to_lane_center_,
                    adc_l_to_lane_center_ + ADC_speed_buffer) +
          ProcessBound::GetBufferBetweenADCCenterAndEdge() + ADC_buffer;
      curr_left_bound =
          (lane_type.is_left_line_solid
               ? std::fmax(curr_left_bound_lane, adc_l_to_lane_center_)
               : std::fmax(curr_left_bound_lane, curr_left_bound_adc));

      double curr_right_bound_adc =
          std::fmin(adc_l_to_lane_center_,
                    adc_l_to_lane_center_ + ADC_speed_buffer) -
          ProcessBound::GetBufferBetweenADCCenterAndEdge() - ADC_buffer;
      curr_right_bound =
          (lane_type.is_right_line_solid
               ? std::fmin(curr_right_bound_lane, adc_l_to_lane_center_)
               : std::fmin(curr_right_bound_lane, curr_right_bound_adc));
    } else {
      curr_left_bound = curr_left_bound_lane;
      curr_right_bound = curr_right_bound_lane;
    }
    // 4. Update the boundary.
    if (!UpdatePathBoundaryWithBuffer(
            i, curr_left_bound, curr_right_bound, path_bound, lane_type,
            is_left_lane_boundary, is_right_lane_boundary)) {
      path_blocked_idx = static_cast<int>(i);
    }
    if (FLAGS_enable_path_bound_debug) {
      curr_neighbor_lane_widths.emplace_back(curr_neighbor_lane_width);
      ADC_speed_buffers.emplace_back(ADC_speed_buffer);
      ADC_buffers.emplace_back(ADC_buffer);
      curr_left_bound_lanes.emplace_back(curr_left_bound_lane);
      curr_right_bound_lanes.emplace_back(curr_right_bound_lane);
      curr_left_bounds.emplace_back(curr_left_bound);
      curr_right_bounds.emplace_back(curr_right_bound);
      curr_lane_left_widths.emplace_back(curr_lane_left_width);
      curr_lane_right_widths.emplace_back(curr_lane_right_width);
    }
    if (path_blocked_idx != -1) {
      break;
    }
  }

  // filter the peak value
  FilterPathBoundPeaks(path_bound);
  if (frame_->GetMachineStateType() ==
      functionmanager::MachineStateType::HISTORY_TRACE_TYPE) {
    ShrinkPathBoundAtEnd(
        config_.path_bounds_decider_config().shrink_distance(),
        config_.path_bounds_decider_config().shrink_end_distance(), path_bound);
  }
  TrimPathBounds(path_blocked_idx, path_bound);

  if (lane_borrow_info == PathInfo::LaneBorrowInfo::NO_BORROW) {
    *borrow_lane_type = "";
  } else {
    *borrow_lane_type = borrowing_reverse_lane ? "reverse" : "forward";
  }

  PathInfo::LanesAndADCPathBoundDebugInfoInput input{&path_bound_type,
                                                     &lane_borrow_info,
                                                     path_bound,
                                                     reference_line_info_,
                                                     borrowing_reverse_lane,
                                                     &curr_neighbor_lane_widths,
                                                     &ADC_speed_buffers,
                                                     &ADC_buffers,
                                                     &curr_left_bound_lanes,
                                                     &curr_right_bound_lanes,
                                                     &curr_left_bounds,
                                                     &curr_right_bounds,
                                                     &curr_lane_left_widths,
                                                     &curr_lane_right_widths};

  PathInfo::LanesAndADCPathBoundDebugInfo(input);
  return true;
}

void ProcessBound::CheckLaneTypePool(
    const PathBound& path_bound, std::vector<LaneType>* const lane_type_pool) {
  // if lane_type_pool is empty, fill it.
  if (lane_type_pool->empty()) {
    lane_type_pool->reserve(path_bound.size());
    for (const auto& bound_point : path_bound) {
      double curr_s = std::get<0>(bound_point);
      LaneType lane_type;
      CheckLaneSolidType(*reference_line_info_, curr_s, &lane_type);
      lane_type_pool->emplace_back(lane_type);
    }
  }
  // else
  // lane_type_pool is full and can be reused.
}

double ProcessBound::EvaluateLaneChangeDistance(
    const ReferenceLineInfo& reference_line_info,
    TL::common::VehicleState* const vehicle_state_ptr) const {
  if (vehicle_state_ptr == nullptr) {
    AERROR << "EvaluateLaneChangeDistance nullptr check is failed!";
    return 0.0;
  }
  double adc_heading = vehicle_state_ptr->heading();
  double adc_steering_percentage = vehicle_state_ptr->steering_percentage();
  ADEBUG << "adc steering percentage: " << adc_steering_percentage;
  auto vehicle_param = VehicleConfigHelper::GetConfig().vehicle_param();
  double adc_steering_angle = vehicle_param.max_steer_angle();
  double steer_ratio = vehicle_param.steer_ratio();
  static constexpr double kMinFrontWheelAngleInlaneChange =
      2.0;  // kMinSteerAngleInlaneChange unit:deg
  static constexpr double kSpeed = 11.0;
  static constexpr double kDeltaAngel = 3.0;
  static constexpr double kHalfCircleAngle = 180.0;
  double front_wheel_angle = kMinFrontWheelAngleInlaneChange;
  if (vehicle_state_ptr->linear_velocity() < kSpeed) {
    front_wheel_angle += kDeltaAngel;
  }
  double wheel_angle =
      adc_steering_percentage / 100.0 * adc_steering_angle / steer_ratio;
  ADEBUG << "front wheel angle: " << wheel_angle;
  if (std::fabs(wheel_angle) < front_wheel_angle / kHalfCircleAngle * M_PI) {
    wheel_angle = front_wheel_angle / kHalfCircleAngle * M_PI;
  } else {
    wheel_angle = std::fabs(wheel_angle) / kHalfCircleAngle * M_PI;
  }
  ADEBUG << "final front wheel angle: " << wheel_angle;
  const ReferenceLine& reference_line = reference_line_info.reference_line();
  const auto& reference_point = reference_line.GetReferencePoint(
      {vehicle_state_ptr->x(), vehicle_state_ptr->y()});
  double ref_point_heading = reference_point.heading();
  hdmap::LaneInfoConstPtr lane = nullptr;
  double s = 0.0;
  double l = 0.0;
  auto point = common::util::PointFactory::ToPointENU(*vehicle_state_ptr);
  static constexpr double kSearchRadius = 5.0;
  if (HDMapUtil::MapForPlanning().GetNearestLaneWithHeading(
          point, kSearchRadius, ref_point_heading, M_PI_2, &lane, &s, &l) ==
      0) {
    ref_point_heading = lane->Heading(s);
  }
  double heading_diff = common::math::AngleDiff(adc_heading, ref_point_heading);
  if (std::fabs(heading_diff) > M_PI / 2) {
    ADEBUG << "Angle diff too large: " << heading_diff;
    return 0.0;
  }
  double adc_lat_speed =
      vehicle_state_ptr->linear_velocity() * std::sin(wheel_angle);
  double adc_lon_speed =
      vehicle_state_ptr->linear_velocity() * std::cos(wheel_angle);
  double lane_left_width = adc_default_lane_width_ / 2;
  double lane_right_width = adc_default_lane_width_ / 2;
  reference_line.GetLaneWidth(adc_frenet_s_, &lane_left_width,
                              &lane_right_width);
  double lat_distance =
      std::fabs(adc_frenet_l_ -
                (adc_frenet_l_ > 0.0 ? lane_left_width : -lane_right_width));
  ADEBUG << "change lane lat distance: " << lat_distance
         << " adc sl_l: " << adc_frenet_l_;
  double min_lane_change_time =
      lat_distance / std::fmax(std::fabs(adc_lat_speed), 0.01);
  ADEBUG << "change lane lat speed: " << adc_lat_speed
         << " min lane change time: " << min_lane_change_time;
  static constexpr double kLimitLaneChangeTime = 5.0;
  min_lane_change_time = std::fmin(kLimitLaneChangeTime, min_lane_change_time);
  return std::fabs(min_lane_change_time * adc_lon_speed);
}

bool ProcessBound::UpdatePathBoundaryWithBuffer(
    const size_t idx, const double left_bound, const double right_bound,
    PathBound* const path_boundaries, const LaneType& lane_type,
    const bool is_left_lane_bound, const bool is_right_lane_bound) {
  if (path_boundaries == nullptr) {
    AERROR << "path_boundaries is empty.";
    return false;
  }

  ADEBUG << "[UpdatePathBoundaryWithBuffer]" << idx;

  ADEBUG << "     left_bound = " << left_bound
         << ", right_bound = " << right_bound;

  ADEBUG << "    is_allow_expand_right_lane_bound = "
         << is_allow_expand_right_lane_bound_
         << ", is_allow_expand_left_lane_bound = "
         << is_allow_expand_left_lane_bound_;
  // substract vehicle width when bound does not come from the lane boundary
  const double default_adc_buffer_coeff = 1.0;
  const double left_line_bound_buffer =
      lane_type.is_left_line_virtual && !lane_type.is_left_line_solid &&
              is_allow_left_virtual_lane_bound_
          ? (lane_type.is_curved_road
                 ? lane_bound_conf_.lane_curved_virtual_bound_buffer()
                 : lane_bound_conf_.lane_normal_virtual_bound_buffer())
          : (lane_type.is_left_line_solid
                 ? (lane_type.is_curved_road
                        ? lane_bound_conf_.lane_curved_solid_bound_buffer()
                    : (is_allow_expand_left_lane_bound_)
                        ? lane_bound_conf_
                              .lane_allow_expand_solid_bound_buffer()
                        : lane_bound_conf_.lane_normal_solid_bound_buffer())
                 : (lane_type.is_curved_road
                        ? lane_bound_conf_.lane_curved_dotted_bound_buffer()
                    : (is_allow_expand_left_lane_bound_)
                        ? lane_bound_conf_
                              .lane_allow_expand_dotted_bound_buffer()
                        : lane_bound_conf_.lane_normal_dotted_bound_buffer()));

  const double right_line_bound_buffer =
      lane_type.is_right_line_virtual && !lane_type.is_right_line_solid &&
              is_allow_right_virtual_lane_bound_
          ? (lane_type.is_curved_road
                 ? lane_bound_conf_.lane_curved_virtual_bound_buffer()
                 : lane_bound_conf_.lane_normal_virtual_bound_buffer())
          : (lane_type.is_right_line_solid
                 ? (lane_type.is_curved_road
                        ? lane_bound_conf_.lane_curved_solid_bound_buffer()
                    : (is_allow_expand_right_lane_bound_)
                        ? lane_bound_conf_
                              .lane_allow_expand_solid_bound_buffer()
                        : lane_bound_conf_.lane_normal_solid_bound_buffer())
                 : (lane_type.is_curved_road
                        ? lane_bound_conf_.lane_curved_dotted_bound_buffer()
                    : (is_allow_expand_right_lane_bound_)
                        ? lane_bound_conf_
                              .lane_allow_expand_dotted_bound_buffer()
                        : lane_bound_conf_.lane_normal_dotted_bound_buffer()));
  ADEBUG << "    right_line_bound_buffer = " << right_line_bound_buffer
         << ", left_line_bound_buffer = " << left_line_bound_buffer;
  const double left_adc_buffer_coeff =
      (is_left_lane_bound
           ? config_.path_bounds_decider_config().adc_buffer_coeff()
           : default_adc_buffer_coeff);
  const double right_adc_buffer_coeff =
      (is_right_lane_bound
           ? config_.path_bounds_decider_config().adc_buffer_coeff()
           : default_adc_buffer_coeff);

  // Update the right bound (l_min):
  double new_l_min =
      right_bound +
      right_adc_buffer_coeff * GetBufferBetweenADCCenterAndEdge() +
      right_line_bound_buffer;
  double new_l_max =
      left_bound - left_adc_buffer_coeff * GetBufferBetweenADCCenterAndEdge() -
      left_line_bound_buffer;

  ADEBUG << "    new_l_max = " << new_l_max << ", new_l_min = " << new_l_min;
  if (new_l_max - new_l_min <
      2.0 * lane_bound_conf_.lane_bound_protect_buffer()) {
    if (idx > 0 && idx < path_boundaries->size()) {
      new_l_min = std::get<1>(path_boundaries->at(idx - 1));
      new_l_max = std::get<2>(path_boundaries->at(idx - 1));
    } else {
      const double center_bound = (new_l_max + new_l_min) / 2;
      new_l_min = center_bound - lane_bound_conf_.lane_bound_protect_buffer();
      new_l_max = center_bound + lane_bound_conf_.lane_bound_protect_buffer();
    }
  }

  ADEBUG << "    new_l_max = " << new_l_max << ", new_l_min = " << new_l_min;

  new_l_min =
      std::fmin(std::fmax(std::get<1>((*path_boundaries)[idx]), new_l_min),
                adc_l_to_lane_center_);
  // Update the left bound (l_max):
  new_l_max =
      std::fmax(std::fmin(std::get<2>((*path_boundaries)[idx]), new_l_max),
                adc_l_to_lane_center_);
  ADEBUG << "    new_l_max = " << new_l_max << ", new_l_min = " << new_l_min;

  // Check if ADC is blocked.
  // If blocked, don't update anything, return false.
  if (new_l_min > new_l_max) {
    ADEBUG << "Path is blocked at idx = " << idx;
    return false;
  }
  // Otherwise, update path_boundaries and center_line; then return true.
  std::get<1>((*path_boundaries)[idx]) = new_l_min;
  std::get<2>((*path_boundaries)[idx]) = new_l_max;

  ADEBUG << "    std::get<2>((*path_boundaries)[idx]) = "
         << std::get<2>((*path_boundaries)[idx])
         << ", std::get<1>((*path_boundaries)[idx]) = "
         << std::get<1>((*path_boundaries)[idx]);

  return true;
}

bool ProcessBound::UpdatePathBoundaryWithBuffer(
    const size_t idx, const double left_bound, const double right_bound,
    PathBound* const path_boundaries,
    const ReferenceLineInfo& reference_line_info, const bool is_left_lane_bound,
    const bool is_right_lane_bound) {
  if (path_boundaries == nullptr) {
    AERROR << "path_boundaries is empty.";
    return false;
  }
  LaneType lane_type;
  CheckLaneSolidType(reference_line_info, std::get<0>((*path_boundaries)[idx]),
                     &lane_type);
  return UpdatePathBoundaryWithBuffer(idx, left_bound, right_bound,
                                      path_boundaries, lane_type,
                                      is_left_lane_bound, is_right_lane_bound);
}

bool ProcessBound::UpdateObstaclePathBoundaryWithBuffer(
    const size_t idx, const double left_bound, const double right_bound,
    PathBound* const path_boundaries) {
  if (path_boundaries == nullptr) {
    AERROR << "path_boundaries is empty.";
    return false;
  }

  // Update the right bound (l_min):
  double new_l_min =
      std::fmax(std::get<1>((*path_boundaries)[idx]),
                right_bound + GetBufferBetweenADCCenterAndEdge());
  double new_l_max = std::fmin(std::get<2>((*path_boundaries)[idx]),
                               left_bound - GetBufferBetweenADCCenterAndEdge());
  if (new_l_max - new_l_min <
      2.0 * lane_bound_conf_.lane_bound_protect_buffer()) {
    if (idx > 0 && idx < path_boundaries->size()) {
      new_l_min = std::get<1>(path_boundaries->at(idx - 1));
      new_l_max = std::get<2>(path_boundaries->at(idx - 1));
    } else {
      const double center_bound = (new_l_max + new_l_min) / 2;
      new_l_min = center_bound - lane_bound_conf_.lane_bound_protect_buffer();
      new_l_max = center_bound + lane_bound_conf_.lane_bound_protect_buffer();
    }
  }

  // Check if ADC is blocked.
  // If blocked, don't update anything, return false.
  if (new_l_min > new_l_max) {
    ADEBUG << "Path is blocked at idx = " << idx;
    return false;
  }
  // Otherwise, update path_boundaries and center_line; then return true.
  std::get<1>((*path_boundaries)[idx]) = new_l_min;
  std::get<2>((*path_boundaries)[idx]) = new_l_max;
  return true;
}

bool ProcessBound::CheckLaneSolidType(
    const ReferenceLineInfo& reference_line_info, const double check_s,
    LaneType* const lane_type) {
  if (lane_type == nullptr) {
    AERROR << "CheckLaneSolidType nullptr check is failed!";
    return false;
  }

  if (injector_->planning_context()
              ->mutable_planning_status()
              ->function_manager_out()
              .fsm_state() ==
          functionmanager::MachineStateType::HISTORY_TRACE_TYPE ||
      injector_->planning_context()
              ->mutable_planning_status()
              ->function_manager_out()
              .fsm_state() ==
          functionmanager::MachineStateType::PERCEPTION_TYPE) {
    lane_type->is_left_line_solid = true;
    lane_type->is_left_line_virtual = false;
    lane_type->is_right_line_solid = true;
    lane_type->is_right_line_virtual = false;
    return true;
  }

  const ReferenceLine& reference_line = reference_line_info.reference_line();

  const double solid_forward_s =
      std::min(check_s + lane_bound_conf_.solid_line_forward_extend_length(),
               reference_line.Length());
  const double solid_backward_s = std::max(
      check_s - lane_bound_conf_.solid_line_backward_extend_length(), 0.0);
  const auto& curr_waypoint =
      reference_line.GetNearestReferencePoint(check_s).lane_waypoints().front();
  const auto& solid_forward_waypoint =
      reference_line.GetNearestReferencePoint(solid_forward_s)
          .lane_waypoints()
          .front();
  const auto& solid_backward_waypoint =
      reference_line.GetNearestReferencePoint(solid_backward_s)
          .lane_waypoints()
          .front();

  LaneBoundaryType::Type left_bound_type =
      hdmap::LeftBoundaryType(curr_waypoint);
  LaneBoundaryType::Type right_bound_type =
      hdmap::RightBoundaryType(curr_waypoint);
  LaneBoundaryType::Type left_forward_bound_type = hdmap::LeftBoundaryType(
      lane_bound_conf_.enable_lane_solid_bound_forward_extend()
          ? solid_forward_waypoint
          : curr_waypoint);
  LaneBoundaryType::Type right_forward_bound_type = hdmap::RightBoundaryType(
      lane_bound_conf_.enable_lane_solid_bound_forward_extend()
          ? solid_forward_waypoint
          : curr_waypoint);
  LaneBoundaryType::Type left_backward_bound_type = hdmap::LeftBoundaryType(
      lane_bound_conf_.enable_lane_solid_bound_backward_extend()
          ? solid_backward_waypoint
          : curr_waypoint);
  LaneBoundaryType::Type right_backward_bound_type = hdmap::RightBoundaryType(
      lane_bound_conf_.enable_lane_solid_bound_backward_extend()
          ? solid_backward_waypoint
          : curr_waypoint);

  // virtual line process
  const int split_num =
      std::max(lane_bound_conf_.virtual_line_extend_split_num() + 1, 1);
  // LaneWaypoint virtual_forward_points;
  // LaneWaypoint virtual_backward_points;
  bool is_left_extend_virtual = false;
  bool is_right_extend_virtual = false;
  for (int i = 0; i < split_num; ++i) {
    const double virtual_forward_s = std::min(
        check_s + (lane_bound_conf_.virtual_line_forward_extend_length() /
                   split_num) *
                      (i + 1),
        reference_line.Length());
    const double virtual_backward_s = std::max(
        check_s - (lane_bound_conf_.virtual_line_backward_extend_length() /
                   split_num) *
                      (i + 1),
        0.0);
    const auto& virtual_forward_points =
        reference_line.GetNearestReferencePoint(virtual_forward_s)
            .lane_waypoints()
            .front();
    const auto& virtual_backward_points =
        reference_line.GetNearestReferencePoint(virtual_backward_s)
            .lane_waypoints()
            .front();

    if (curr_waypoint.lane->lane().left_boundary().virtual_() ||
        virtual_forward_points.lane->lane().left_boundary().virtual_() ||
        virtual_backward_points.lane->lane().left_boundary().virtual_()) {
      is_left_extend_virtual = true;
    }
    if (curr_waypoint.lane->lane().right_boundary().virtual_() ||
        virtual_forward_points.lane->lane().right_boundary().virtual_() ||
        virtual_backward_points.lane->lane().right_boundary().virtual_()) {
      is_right_extend_virtual = true;
    }
    if (is_left_extend_virtual && is_right_extend_virtual) {
      break;
    }
  }

  // left lane type process
  lane_type->is_left_line_solid =
      (left_bound_type != LaneBoundaryType::DOTTED_YELLOW &&
       left_bound_type != LaneBoundaryType::DOTTED_WHITE) ||
      (left_backward_bound_type != LaneBoundaryType::DOTTED_YELLOW &&
       left_backward_bound_type != LaneBoundaryType::DOTTED_WHITE) ||
      (left_forward_bound_type != LaneBoundaryType::DOTTED_YELLOW &&
       left_forward_bound_type != LaneBoundaryType::DOTTED_WHITE) ||
      (CheckCameraLaneBoundaryType(false) &&
       lane_bound_conf_.use_camera_lane_type()) ||
      (lane_bound_conf_.enable_normal_turn_line_is_solid() &&
       IsNormalTurn(curr_waypoint));
  lane_type->is_left_line_virtual = is_left_extend_virtual;
  // right lane type process
  lane_type->is_right_line_solid =
      (right_bound_type != LaneBoundaryType::DOTTED_YELLOW &&
       right_bound_type != LaneBoundaryType::DOTTED_WHITE) ||
      (right_backward_bound_type != LaneBoundaryType::DOTTED_YELLOW &&
       right_backward_bound_type != LaneBoundaryType::DOTTED_WHITE) ||
      (right_forward_bound_type != LaneBoundaryType::DOTTED_YELLOW &&
       right_forward_bound_type != LaneBoundaryType::DOTTED_WHITE) ||
      (CheckCameraLaneBoundaryType(true) &&
       lane_bound_conf_.use_camera_lane_type()) ||
      (lane_bound_conf_.enable_normal_turn_line_is_solid() &&
       IsNormalTurn(curr_waypoint));
  lane_type->is_right_line_virtual = is_right_extend_virtual;
  // road section type process
  lane_type->is_curved_road = IsRoadCurvedSection(curr_waypoint);
  return true;
}

bool ProcessBound::GetBoundaryFromLanes(
    const ReferenceLineInfo& reference_line_info,
    const PathInfo::LaneBorrowInfo& lane_borrow_info,
    PathBound* const path_bound, std::string* const borrow_lane_type) const {
  // Sanity checks.
  if (path_bound == nullptr || path_bound->empty()) {
    AERROR << "There is no path_bound points!";
    return false;
  }
  const ReferenceLine& reference_line = reference_line_info.reference_line();

  // Go through every point, update the boundary based on lane-info.
  double past_lane_left_width = adc_default_lane_width_ / 2;
  double past_lane_right_width = adc_default_lane_width_ / 2;
  int path_blocked_idx = -1;
  bool borrowing_reverse_lane = false;
  for (size_t i = 0; i < path_bound->size(); ++i) {
    double curr_s = std::get<0>((*path_bound)[i]);

    // 1. Get the current lane width at current point.
    double curr_lane_left_width = 0.0;
    double curr_lane_right_width = 0.0;
    if (!reference_line.GetLaneWidth(curr_s, &curr_lane_left_width,
                                     &curr_lane_right_width)) {
      AWARN << "Failed to get lane width at s = " << curr_s;
      curr_lane_left_width = past_lane_left_width;
      curr_lane_right_width = past_lane_right_width;
    } else {
      // double offset_to_lane_center = 0.0;
      // reference_line.GetOffsetToMap(curr_s, &offset_to_lane_center);
      // The left-width and right-width are w.r.t. lane-center, not ref-line.
      // curr_lane_left_width -= offset_to_lane_center;
      // curr_lane_right_width += offset_to_lane_center;
      past_lane_left_width = curr_lane_left_width;
      past_lane_right_width = curr_lane_right_width;
    }

    // 2. Get the neighbor lane widths at the current point.
    // 3. Get the proper boundary
    double curr_neighbor_lane_width = 0.0;
    double curr_left_bound = curr_lane_left_width;
    double curr_right_bound = -curr_lane_right_width;
    if (FLAGS_enable_original_lane_borrow_process) {
      if (CheckLaneBoundaryType(reference_line_info, curr_s,
                                lane_borrow_info)) {
        hdmap::Id neighbor_lane_id;
        if (lane_borrow_info == PathInfo::LaneBorrowInfo::LEFT_BORROW) {
          // Borrowing left neighbor lane.
          if (reference_line_info.GetNeighborLaneInfo(
                  ReferenceLineInfo::LaneType::LeftForward, curr_s,
                  &neighbor_lane_id, &curr_neighbor_lane_width)) {
            ADEBUG << "Borrow left forward neighbor lane.";
          } else if (reference_line_info.GetNeighborLaneInfo(
                         ReferenceLineInfo::LaneType::LeftReverse, curr_s,
                         &neighbor_lane_id, &curr_neighbor_lane_width)) {
            borrowing_reverse_lane = true;
            ADEBUG << "Borrow left reverse neighbor lane.";
          } else {
            ADEBUG << "There is no left reverse neighbor lane.";
          }
        } else if (lane_borrow_info == PathInfo::LaneBorrowInfo::RIGHT_BORROW) {
          // Borrowing right neighbor lane.
          if (reference_line_info.GetNeighborLaneInfo(
                  ReferenceLineInfo::LaneType::RightForward, curr_s,
                  &neighbor_lane_id, &curr_neighbor_lane_width)) {
            ADEBUG << "Borrow right forward neighbor lane.";
          } else if (reference_line_info.GetNeighborLaneInfo(
                         ReferenceLineInfo::LaneType::RightReverse, curr_s,
                         &neighbor_lane_id, &curr_neighbor_lane_width)) {
            borrowing_reverse_lane = true;
            ADEBUG << "Borrow right reverse neighbor lane.";
          } else {
            ADEBUG << "There is no right reverse neighbor lane.";
          }
        }
      }
      curr_left_bound +=
          (lane_borrow_info == PathInfo::LaneBorrowInfo::LEFT_BORROW
               ? curr_neighbor_lane_width
               : 0.0);
      curr_right_bound -=
          (lane_borrow_info == PathInfo::LaneBorrowInfo::RIGHT_BORROW
               ? curr_neighbor_lane_width
               : 0.0);
    }

    ADEBUG << "At s = " << curr_s << ", left_lane_bound = " << curr_left_bound
           << ", right_lane_bound = " << curr_right_bound;

    // 4. Update the boundary.
    if (!UpdatePathBoundary(i, curr_left_bound, curr_right_bound, path_bound)) {
      path_blocked_idx = static_cast<int>(i);
    }
    if (path_blocked_idx != -1) {
      break;
    }
  }

  TrimPathBounds(path_blocked_idx, path_bound);

  if (lane_borrow_info == PathInfo::LaneBorrowInfo::NO_BORROW) {
    *borrow_lane_type = "";
  } else {
    *borrow_lane_type = borrowing_reverse_lane ? "reverse" : "forward";
  }
  return true;
}

bool ProcessBound::CheckLaneBoundaryType(
    const ReferenceLineInfo& reference_line_info, const double check_s,
    const PathInfo::LaneBorrowInfo& lane_borrow_info) {
  if (lane_borrow_info == PathInfo::LaneBorrowInfo::NO_BORROW) {
    return false;
  }

  const ReferenceLine& reference_line = reference_line_info.reference_line();
  const auto& ref_point = reference_line.GetNearestReferencePoint(check_s);
  if (ref_point.lane_waypoints().empty()) {
    return false;
  }

  const auto waypoint = ref_point.lane_waypoints().front();
  hdmap::LaneBoundaryType::Type lane_boundary_type =
      hdmap::LaneBoundaryType::UNKNOWN;
  if (lane_borrow_info == PathInfo::LaneBorrowInfo::LEFT_BORROW) {
    lane_boundary_type = hdmap::LeftBoundaryType(waypoint);
  } else if (lane_borrow_info == PathInfo::LaneBorrowInfo::RIGHT_BORROW) {
    lane_boundary_type = hdmap::RightBoundaryType(waypoint);
  }
  if (lane_boundary_type == hdmap::LaneBoundaryType::SOLID_YELLOW ||
      lane_boundary_type == hdmap::LaneBoundaryType::SOLID_WHITE) {
    return false;
  }
  return true;
}

bool ProcessBound::CheckLaneBoundaryType(
    const ReferenceLineInfo& reference_line_info, const double check_s,
    const routing::ChangeLaneType& lane_change_type) {
  PathInfo::LaneBorrowInfo lane_borrow_type =
      PathInfo::LaneBorrowInfo::NO_BORROW;
  switch (lane_change_type) {
    case routing::LEFT:
      lane_borrow_type = PathInfo::LaneBorrowInfo::RIGHT_BORROW;
      break;
    case routing::RIGHT:
      lane_borrow_type = PathInfo::LaneBorrowInfo::LEFT_BORROW;
      break;
    default:
      break;
  }
  return CheckLaneBoundaryType(reference_line_info, check_s, lane_borrow_type);
}

routing::ChangeLaneType ProcessBound::JudgeLaneChangeType(
    const double adc_frenet_l) {
  const int result = ComparedToZero(adc_frenet_l);
  if (result > 0) {
    return routing::RIGHT;
  }
  if (result < 0) {
    return routing::LEFT;
  }
  return routing::FORWARD;
}

double ProcessBound::GetBufferBetweenADCCenterAndEdge() {
  return VehicleConfigHelper::GetConfig().vehicle_param().width() / 2;
}

bool ProcessBound::UpdatePathBoundary(size_t idx, double left_bound,
                                      double right_bound,
                                      PathBound* const path_boundaries) {
  if (path_boundaries == nullptr) {
    AERROR << "path_boundaries is empty.";
    return false;
  }
  // Update the right bound (l_min):
  double new_l_min =
      std::fmax(std::get<1>((*path_boundaries)[idx]), right_bound);
  // Update the left bound (l_max):
  double new_l_max =
      std::fmin(std::get<2>((*path_boundaries)[idx]), left_bound);

  // Check if ADC is blocked.
  // If blocked, don't update anything, return false.
  if (new_l_min > new_l_max) {
    ADEBUG << "Path is blocked at idx = " << idx;
    return false;
  }
  // Otherwise, update path_boundaries and center_line; then return true.
  std::get<1>((*path_boundaries)[idx]) = new_l_min;
  std::get<2>((*path_boundaries)[idx]) = new_l_max;
  return true;
}

void ProcessBound::TrimPathBounds(const int path_blocked_idx,
                                  PathBound* const path_boundaries) {
  if (path_boundaries == nullptr) {
    AERROR << "path_boundaries is empty.";
    return;
  }
  if (path_blocked_idx != -1) {
    if (path_blocked_idx == 0) {
      ADEBUG << "Completely blocked. Cannot move at all.";
    }
    int range = static_cast<int>(path_boundaries->size()) - path_blocked_idx;
    for (int i = 0; i < range; ++i) {
      path_boundaries->pop_back();
    }
  }
}

void ProcessBound::TrimTowingPoints(const int path_blocked_idx,
                                    TowingPointsInfo* const towing_points) {
  if (towing_points == nullptr) {
    AERROR << "towing_points is empty.";
    return;
  }
  if (path_blocked_idx != -1) {
    if (path_blocked_idx == 0) {
      ADEBUG << "Completely blocked. Cannot move at all.";
    }
    int range = static_cast<int>(towing_points->size()) - path_blocked_idx;
    for (int i = 0; i < range; ++i) {
      towing_points->pop_back();
    }
  }
}

common::TrajectoryPoint ProcessBound::InferFrontAxeCenterFromRearAxeCenter(
    const common::TrajectoryPoint& traj_point) {
  double front_to_rear_axe_distance =
      VehicleConfigHelper::GetConfig().vehicle_param().wheel_base();
  common::TrajectoryPoint ret = traj_point;
  ret.mutable_path_point()->set_x(
      traj_point.path_point().x() +
      front_to_rear_axe_distance * std::cos(traj_point.path_point().theta()));
  ret.mutable_path_point()->set_y(
      traj_point.path_point().y() +
      front_to_rear_axe_distance * std::sin(traj_point.path_point().theta()));
  return ret;
}

bool ProcessBound::GetBoundaryFromADC(
    const ReferenceLineInfo& reference_line_info, double ADC_extra_buffer,
    PathBound* const path_bound) const {
  UNUSED(reference_line_info);
  // Sanity checks.
  if (path_bound == nullptr || path_bound->empty()) {
    AERROR << "There is no path_bound points!";
    return false;
  }

  // Calculate the ADC's lateral boundary.
  static constexpr double kMaxLateralAccelerations = 1.5;
  double ADC_lat_decel_buffer = (adc_frenet_ld_ > 0 ? 1.0 : -1.0) *
                                adc_frenet_ld_ * adc_frenet_ld_ /
                                kMaxLateralAccelerations / 2;
  double curr_left_bound_adc =
      ProcessBound::GetBufferBetweenADCCenterAndEdge() + ADC_extra_buffer +
      std::fmax(adc_l_to_lane_center_,
                adc_l_to_lane_center_ + ADC_lat_decel_buffer);
  double curr_right_bound_adc =
      -ProcessBound::GetBufferBetweenADCCenterAndEdge() - ADC_extra_buffer +
      std::fmin(adc_l_to_lane_center_,
                adc_l_to_lane_center_ + ADC_lat_decel_buffer);

  // Expand the boundary in case ADC falls outside.
  for (size_t i = 0; i < path_bound->size(); ++i) {
    double curr_left_bound = std::get<2>((*path_bound)[i]);
    curr_left_bound = std::fmax(curr_left_bound_adc, curr_left_bound);
    double curr_right_bound = std::get<1>((*path_bound)[i]);
    curr_right_bound = std::fmin(curr_right_bound_adc, curr_right_bound);
    ProcessBound::UpdatePathBoundary(i, curr_left_bound, curr_right_bound,
                                     path_bound);
  }
  return true;
}

void ProcessBound::ReversePathBound(const double pnc_length,
                                    PathBound* const path_bound) {
  double start_s = fmax(adc_frenet_s_ - pnc_length,
                        config_.path_bounds_decider_config()
                            .path_end_distance_from_reference_end());
  double end_s = adc_frenet_s_;
  const int length_iter = std::ceil(
      (end_s - start_s) / reference_line_info_->PathBoundsDeciderResolution());
  for (int curr_s_i = 0; curr_s_i < length_iter; ++curr_s_i) {
    path_bound->emplace_back(
        end_s - curr_s_i * reference_line_info_->PathBoundsDeciderResolution(),
        std::numeric_limits<double>::lowest(),
        std::numeric_limits<double>::max());
  }
  std::reverse(path_bound->begin(), path_bound->end());
}

void ProcessBound::FilterPathBoundPeaks(PathBound* const path_bound) {
  if (path_bound == nullptr) {
    AERROR << "filterPeaks nullptr check is failed!";
    return;
  }
  const int n = static_cast<int>(path_bound->size());
  int l_max_peak_point_cnt = 0;  // 突变值的个数
  int l_min_peak_point_cnt = 0;
  int l_max_peak_start = 0;  // 起点
  int l_min_peak_start = 0;
  // 从第0个数据点开始遍历
  for (int i = 0; i < n - 1; ++i) {
    ADEBUG << "curr index i = " << i
           << ", l_max_peak_start = " << l_max_peak_start
           << ", l_max_peak_point_cnt = " << l_max_peak_point_cnt
           << ", l_min_peak_start = " << l_min_peak_start
           << ", l_min_peak_point_cnt = " << l_min_peak_point_cnt
           << ", l_min = " << std::get<1>(path_bound->at(i))
           << ", l_max = " << std::get<2>(path_bound->at(i));
    const double l_max_curr_and_next_err =
        std::get<2>(path_bound->at(i + 1)) - std::get<2>(path_bound->at(i));
    const double l_min_curr_and_next_err =
        std::get<1>(path_bound->at(i + 1)) - std::get<1>(path_bound->at(i));
    ADEBUG << "l_max_curr_and_next_err = " << l_max_curr_and_next_err
           << "l_min_curr_and_next_err = " << l_min_curr_and_next_err;

    // process left bound
    if (l_max_peak_point_cnt == 0 &&
        Compare(l_max_curr_and_next_err, -config_.path_bounds_decider_config()
                                              .start_count_peaks_threshold()) <
            0) {
      // 找出首个突变点 err < -0.09
      ++l_max_peak_point_cnt;
      l_max_peak_start = i;  // 起点
      ADEBUG << "first peak value index = " << l_max_peak_start + 1
             << ", l_max_peak_point_cnt = " << l_max_peak_point_cnt;
    } else if (l_max_peak_point_cnt > 0) {
      if (Compare(l_max_curr_and_next_err, config_.path_bounds_decider_config()
                                               .stop_count_peaks_threshold()) >
              0 &&
          l_max_peak_point_cnt <=
              config_.path_bounds_decider_config().peak_point_cnt()) {
        // err > 0.09 且 cnt<=3 形成尖峰进行并过滤
        ADEBUG << " i before filter = " << i;
        for (int j = l_max_peak_start + 1; j <= i; ++j) {
          ADEBUG << "filter index = " << j;
          std::get<2>(path_bound->at(j)) =
              std::get<2>(path_bound->at(l_max_peak_start));
        }
        i = l_max_peak_start;
        l_max_peak_point_cnt = 0;
        ADEBUG << " i after filter = " << i;
      } else if (l_max_peak_point_cnt >
                 config_.path_bounds_decider_config().peak_point_cnt()) {
        // 是拐点
        l_max_peak_start = i;
        l_max_peak_point_cnt = 0;
        ADEBUG << "not a peak, index = " << l_max_peak_start
               << ", l_max_peak_point_cnt = " << l_max_peak_point_cnt;
      } else {
        ++l_max_peak_point_cnt;
        ADEBUG << "is another peak value and l_max_peak_point_cnt = "
               << l_max_peak_point_cnt;
      }
    }

    // process right bound
    if (l_min_peak_point_cnt == 0 &&
        Compare(l_min_curr_and_next_err, config_.path_bounds_decider_config()
                                             .start_count_peaks_threshold()) >
            0) {
      // 找出首个突变点 err > 0.09
      ++l_min_peak_point_cnt;
      l_min_peak_start = i;  // 起点
      ADEBUG << "first peak value index = " << l_min_peak_start + 1
             << ", l_min_peak_point_cnt = " << l_min_peak_point_cnt;
    } else if (l_min_peak_point_cnt > 0) {
      if (Compare(l_min_curr_and_next_err, -config_.path_bounds_decider_config()
                                                .stop_count_peaks_threshold()) <
              0 &&
          l_min_peak_point_cnt <=
              config_.path_bounds_decider_config().peak_point_cnt()) {
        // err < -0.09 且 cnt<=3 形成尖峰进行并过滤
        ADEBUG << " i before filter = " << i;
        for (int j = l_min_peak_start + 1; j <= i; ++j) {
          ADEBUG << "filter index = " << j;
          std::get<1>(path_bound->at(j)) =
              std::get<1>(path_bound->at(l_min_peak_start));
        }
        i = l_min_peak_start;
        l_min_peak_point_cnt = 0;
        ADEBUG << " i after filter = " << i;
      } else if (l_min_peak_point_cnt >
                 config_.path_bounds_decider_config().peak_point_cnt()) {
        // 是拐点
        l_min_peak_start = i;
        l_min_peak_point_cnt = 0;
        ADEBUG << "not a peak, index = " << l_min_peak_start
               << ", l_min_peak_point_cnt = " << l_min_peak_point_cnt;
      } else {
        ++l_min_peak_point_cnt;
        ADEBUG << "is another peak value and l_min_peak_point_cnt = "
               << l_min_peak_point_cnt;
      }
    }
  }
}

bool ProcessBound::ShrinkPathBoundAtEnd(const double shrink_distance,
                                        const double shrink_end_distance,
                                        PathBound* const path_bound) {
  const double extend_length = common::VehicleConfigHelper::GetConfig()
                                   .vehicle_param()
                                   .back_edge_to_center() +
                               FLAGS_history_trace_path_extend_buffer;
  if ((adc_frenet_s_ - (3 * shrink_end_distance + extend_length) <=
       KAlmostZero1ENegtive6) &&
      (std::fabs(adc_frenet_l_) -
           lane_bound_conf_.path_default_lane_width() / 2 >
       KAlmostZero1ENegtive6)) {
    return true;
  }
  const double min_shrink_bound =
      config_.path_bounds_decider_config().min_shrink_bound();
  // Sanity checks.
  if (path_bound == nullptr) {
    return false;
  }
  if (path_bound->size() < 3) {
    return true;
  }
  if ((std::get<1>(path_bound->front()) + min_shrink_bound >
       KAlmostZero1ENegtive6) &&
      (std::get<2>(path_bound->front()) - min_shrink_bound <
       KAlmostZero1ENegtive6)) {
    return true;
  }
  size_t i_end = 0;
  while ((i_end < path_bound->size() - 1) &&
         (std::get<0>((*path_bound)[i_end]) - shrink_distance - extend_length <
          KAlmostZero1ENegtive6)) {
    ++i_end;
  }
  for (size_t i = 0; i < path_bound->size(); ++i) {
    const double distance_to_start = std::get<0>((*path_bound)[i]);
    if ((distance_to_start - shrink_distance - extend_length <
         KAlmostZero1ENegtive6) &&
        (distance_to_start - shrink_end_distance - extend_length >=
         KAlmostZero1ENegtive6)) {
      double l_min_shrink =
          std::get<1>((*path_bound)[i]) *
          (distance_to_start - shrink_end_distance - extend_length) /
          SafeDiv(static_cast<float>(std::get<0>((*path_bound)[i_end]) -
                                     shrink_end_distance - extend_length));
      double l_max_shrink =
          std::get<2>((*path_bound)[i]) *
          (distance_to_start - shrink_end_distance - extend_length) /
          SafeDiv(static_cast<float>(std::get<0>((*path_bound)[i_end]) -
                                     shrink_end_distance - extend_length));
      UpdatePathBoundary(i, fmax(l_max_shrink, min_shrink_bound / 2),
                         fmin(l_min_shrink, -min_shrink_bound / 2), path_bound);
    } else if (distance_to_start - shrink_end_distance - extend_length <
               KAlmostZero1ENegtive6) {
      UpdatePathBoundary(i, min_shrink_bound / 2, -min_shrink_bound / 2,
                         path_bound);
    } else {
      break;
    }
  }
  return true;
}

void ProcessBound::DeleteOutOfScopeObstacle(
    std::vector<ObstacleEdge>* const obstacle_edges,
    std::vector<size_t>* const processing_edge_idx,
    std::multiset<std::pair<double, std::string>, std::greater<>>* const
        right_bounds,
    std::multiset<std::pair<double, std::string>>* const left_bounds,
    double curr_s) const {
  if (processing_edge_idx == nullptr || right_bounds == nullptr ||
      left_bounds == nullptr || obstacle_edges == nullptr ||
      processing_edge_idx->empty()) {
    ADEBUG << "DeleteOutOfScopeObstacle empty check is failed!";
    return;
  }
  static constexpr double erase_obstacle_s_buffer = 1.0;
  for (auto iter = processing_edge_idx->begin();
       iter != processing_edge_idx->end();) {
    const auto& curr_obstacle = obstacle_edges->at(*iter);
    if (Compare(curr_obstacle.obstacle_edge_end_s + erase_obstacle_s_buffer,
                curr_s) < 0) {
      const double curr_obstacle_l_min = curr_obstacle.obstacle_edge_l_min;
      const double curr_obstacle_l_max = curr_obstacle.obstacle_edge_l_max;
      const std::string curr_obstacle_id = curr_obstacle.obstacle_id;
      if (Compare((curr_obstacle_l_min + curr_obstacle_l_max),
                  adc_frenet_l_ * 2) < 0) {
        // Obstacle is to the right of center-line, should pass from left.
        right_bounds->erase(
            std::make_pair(curr_obstacle_l_max, curr_obstacle_id));
      } else {
        // Obstacle is to the left of center-line, should pass from right.
        left_bounds->erase(
            std::make_pair(curr_obstacle_l_min, curr_obstacle_id));
      }
      iter = processing_edge_idx->erase(iter);
    } else {
      ++iter;
    }
  }
}

bool ProcessBound::SteeringWheelSpeedLimitBoundProcess(
    PathBound* const path_bound) {
  if (path_bound == nullptr || path_bound->empty()) {
    AERROR << "path bound is empty.";
    return false;
  }

  const auto wheel_base =
      VehicleConfigHelper::GetConfig().vehicle_param().wheel_base();
  const auto steer_ratio =
      VehicleConfigHelper::GetConfig().vehicle_param().steer_ratio();
  const auto max_wheel_angle =
      (VehicleConfigHelper::GetConfig().vehicle_param().max_steer_angle() -
       config_.path_bounds_decider_config()
           .steering_wheel_angle_speed_limit_config()
           .max_steering_protect_angle()) /
      steer_ratio;
  const auto cur_v = fmax(config_.path_bounds_decider_config()
                              .steering_wheel_angle_speed_limit_config()
                              .curvature_calculate_min_vehicle_speed(),
                          frame_->PlanningStartPoint().v());
  auto max_wheel_angle_speed =
      VehicleConfigHelper::GetConfig()
              .vehicle_param()
              .has_steer_wheel_speed_segment()
          ? common::math::InterpolationOne(
                cur_v,
                VehicleConfigHelper::GetConfig()
                    .vehicle_param()
                    .steer_wheel_speed_segment()
                    .vehicle_speed_segment(),
                VehicleConfigHelper::GetConfig()
                    .vehicle_param()
                    .steer_wheel_speed_segment()
                    .steering_wheel_speed_limit_segment()) /
                180.0 * M_PI
          : VehicleConfigHelper::GetConfig()
                .vehicle_param()
                .max_steer_angle_rate();
  max_wheel_angle_speed /= steer_ratio;
  const auto steering_wheel_angle_line_protect_buffer =
      config_.path_bounds_decider_config()
          .steering_wheel_angle_speed_limit_config()
          .steering_wheel_angle_line_protect_buffer();
  const auto delta_s = reference_line_info_->PathBoundsDeciderResolution();

  common::math::Vec2d left_steer_bound_xy;
  left_steer_bound_xy.set_x(frame_->PlanningStartPoint().path_point().x());
  left_steer_bound_xy.set_y(frame_->PlanningStartPoint().path_point().y());
  common::SLPoint left_steer_bound_sl;
  left_steer_bound_sl.set_s(adc_frenet_s_);
  left_steer_bound_sl.set_s(adc_frenet_l_);
  double left_steer_bound_angle =
      frame_->PlanningStartPoint().path_point().theta();
  double left_steer_bound_kappa =
      frame_->PlanningStartPoint().path_point().kappa();
  double left_wheel_angle = atan(wheel_base * left_steer_bound_kappa);

  common::math::Vec2d right_steer_bound_xy = left_steer_bound_xy;
  common::SLPoint right_steer_bound_sl = left_steer_bound_sl;
  double right_steer_bound_angle = left_steer_bound_angle;
  double right_steer_bound_kappa = left_steer_bound_kappa;
  double right_wheel_angle = left_wheel_angle;

  for (std::size_t i = 0; i < path_bound->size(); ++i) {
    auto& path_bound_point = path_bound->at(i);
    const auto cur_s = std::get<0>(path_bound_point);
    const auto& ref_point =
        reference_line_info_->reference_line().GetNearestReferencePoint(cur_s);

    // 计算left steer bound
    const auto left_delta_heading =
        common::math::AngleDiff(ref_point.heading(), left_steer_bound_angle);
    if (fabs(left_delta_heading) <
        config_.path_bounds_decider_config()
            .steering_wheel_angle_speed_limit_config()
            .cutoff_heading_of_vehicle_motion_curve()) {
      const auto left_delta_path_s = delta_s / cos(left_delta_heading);
      left_steer_bound_angle += left_steer_bound_kappa * left_delta_path_s;
      left_steer_bound_xy.set_x(left_steer_bound_xy.x() +
                                left_delta_path_s *
                                    cos(left_steer_bound_angle));
      left_steer_bound_xy.set_y(left_steer_bound_xy.y() +
                                left_delta_path_s *
                                    sin(left_steer_bound_angle));
      reference_line_info_->reference_line().XYToSLForSmooth(
          left_steer_bound_xy, &left_steer_bound_sl);
      left_wheel_angle = fmin(
          max_wheel_angle,
          left_wheel_angle + max_wheel_angle_speed * left_delta_path_s / cur_v);
      left_steer_bound_kappa = tan(left_wheel_angle) / wheel_base;
    }

    // 计算right steer bound
    const auto right_delta_heading =
        common::math::AngleDiff(ref_point.heading(), right_steer_bound_angle);
    if (fabs(right_delta_heading) <
        config_.path_bounds_decider_config()
            .steering_wheel_angle_speed_limit_config()
            .cutoff_heading_of_vehicle_motion_curve()) {
      const auto right_delta_path_s = delta_s / cos(right_delta_heading);
      right_steer_bound_angle += right_steer_bound_kappa * right_delta_path_s;
      right_steer_bound_xy.set_x(right_steer_bound_xy.x() +
                                 right_delta_path_s *
                                     cos(right_steer_bound_angle));
      right_steer_bound_xy.set_y(right_steer_bound_xy.y() +
                                 right_delta_path_s *
                                     sin(right_steer_bound_angle));
      reference_line_info_->reference_line().XYToSLForSmooth(
          right_steer_bound_xy, &right_steer_bound_sl);
      right_wheel_angle = fmin(
          max_wheel_angle, right_wheel_angle - max_wheel_angle_speed *
                                                   right_delta_path_s / cur_v);
      right_steer_bound_kappa = tan(right_wheel_angle) / wheel_base;
    }

    auto left_bound =
        common::math::Clamp(std::get<2>(path_bound_point),
                            left_steer_bound_sl.l(), right_steer_bound_sl.l());
    auto right_bound =
        common::math::Clamp(std::get<1>(path_bound_point),
                            left_steer_bound_sl.l(), right_steer_bound_sl.l());
    const auto bound_width =
        fmin(std::get<2>(path_bound_point) - std::get<1>(path_bound_point),
             steering_wheel_angle_line_protect_buffer);
    const auto extend_width =
        fmax((bound_width - (left_bound - right_bound)), 0.0) * 0.5;
    std::get<2>(path_bound_point) = left_bound + extend_width;
    std::get<1>(path_bound_point) = right_bound - extend_width;
  }

  return true;
}

void ProcessBound::IsClearToExpandLaneBound() {
  // Judge whether has obstacle behind adc in neighbor lane
  const double kJudgeBehindObstacleLateralThre = 1.5;
  const auto& indexed_obstacles =
      reference_line_info_->path_decision()->obstacles();

  for (const auto* const obstacle : indexed_obstacles.Items()) {
    // Ignore obstacle whose distance to reference line more than
    // obs_right_filter_distance or obs_left_filter_distance.
    if (common::math::double_type::DefinitelyLess(
            obstacle->PerceptionSLBoundary().end_l(),
            config_.path_bounds_decider_config()
                .dynamic_obs_process_config()
                .obs_right_filter_distance()) ||
        common::math::double_type::DefinitelyGreater(
            obstacle->PerceptionSLBoundary().start_l(),
            config_.path_bounds_decider_config()
                .dynamic_obs_process_config()
                .obs_left_filter_distance())) {
      ADEBUG << " Obs: " << obstacle->Id() << " isn't in lateral consider area";
      continue;
    }

    const double cur_obstacle_start_s =
        obstacle->PerceptionSLBoundary().start_s();
    const double cur_obstacle_start_l =
        obstacle->PerceptionSLBoundary().start_l();
    const double cur_obstacle_end_l = obstacle->PerceptionSLBoundary().end_l();

    if (cur_obstacle_start_s < GetAdcFrenetS()) {
      ADEBUG << "---------------- in adc behind ----------------"
             << "obs id = " << obstacle->Id();
      if (Compare((cur_obstacle_start_l + cur_obstacle_end_l) / 2,
                  GetAdcFrenetL()) < 0) {
        // in the right of adc
        if (cur_obstacle_end_l < -kJudgeBehindObstacleLateralThre) {
          is_clear_to_expand_right_lane_bound_ = false;
        }

      } else {
        // in the left of adc
        if (cur_obstacle_start_l > kJudgeBehindObstacleLateralThre) {
          is_clear_to_expand_left_lane_bound_ = false;
        }
      }
    }
    if (!is_clear_to_expand_left_lane_bound_ &&
        !is_clear_to_expand_right_lane_bound_) {
      return;
    }
  }
}

void ProcessBound::IsAllowVirtualLaneBound(
    const PathBound& path_boundaries,
    const std::vector<ObstacleEdge>& obstacle_edges) {

  bool is_block_by_left_vehicle = false;
  bool is_block_by_right_vehicle = false;
  const auto center_line = GetAdcFrenetL();
  for (const auto& path_bound_point : path_boundaries) {
    double curr_s = std::get<0>(path_bound_point);
    size_t obs_idx = 0;

    // filter the obstacles not in the curr_s scope.
    while (obs_idx < obstacle_edges.size() &&
           Compare(obstacle_edges[obs_idx].obstacle_edge_start_s, curr_s) <=
               0) {
      // if obstacle totally behind curr_s, do not process.
      if (Compare(obstacle_edges[obs_idx].obstacle_edge_end_s, curr_s) < 0 ||
          obstacle_edges[obs_idx].obstacle == nullptr ||
          obstacle_edges[obs_idx].obstacle->IsCone()) {
        ++obs_idx;
        continue;
      }

      // only process the obstacle that curr_s between obs_start_s and obs_end_s
      const auto& curr_obstacle = obstacle_edges[obs_idx];
      const auto curr_obstacle_box_l_min = curr_obstacle.obstacle_edge_l_min;
      const auto curr_obstacle_box_l_max = curr_obstacle.obstacle_edge_l_max;
      const std::string curr_obstacle_id = curr_obstacle.obstacle_id;

      if (Compare((curr_obstacle_box_l_min + curr_obstacle_box_l_max) / 2,
                  center_line) < 0) {
        // obstacle is to the right of center-line, should pass from left.
        if (std::get<2>(path_bound_point) <
            curr_obstacle_box_l_max +
                ProcessBound::GetBufferBetweenADCCenterAndEdge()) {
          is_block_by_right_vehicle = true;
        }
      } else {
        // obstacle is to the left of center-line, should pass from right.
        if (std::get<1>(path_bound_point) >
            curr_obstacle_box_l_min -
                ProcessBound::GetBufferBetweenADCCenterAndEdge()) {
          is_block_by_left_vehicle = true;
        }
      }
      ++obs_idx;
    }
  }

  if (!is_block_by_right_vehicle) {
    is_allow_left_virtual_lane_bound_ = true;
  }
  if (!is_block_by_left_vehicle) {
    is_allow_right_virtual_lane_bound_ = true;
  }
}

void ProcessBound::IsAllowExpendLaneBound(
    const PathBound& path_boundaries,
    const std::vector<ObstacleEdge>& obstacle_edges) {
  const auto center_line = GetAdcFrenetL();
  for (const auto& path_bound_point : path_boundaries) {
    double curr_s = std::get<0>(path_bound_point);
    size_t obs_idx = 0;

    // filter the obstacles not in the curr_s scope.
    while (obs_idx < obstacle_edges.size() &&
           Compare(obstacle_edges[obs_idx].obstacle_edge_start_s, curr_s) <=
               0) {
      // if obstacle totally behind curr_s, do not process.
      if (Compare(obstacle_edges[obs_idx].obstacle_edge_end_s, curr_s) < 0 ||
          obstacle_edges[obs_idx].obstacle == nullptr ||
          !obstacle_edges[obs_idx].obstacle->IsCone()) {
        ++obs_idx;
        continue;
      }

      // only process the obstacle that curr_s between obs_start_s and obs_end_s
      const auto& curr_obstacle = obstacle_edges[obs_idx];
      const auto curr_obstacle_box_l_min = curr_obstacle.obstacle_edge_l_min;
      const auto curr_obstacle_box_l_max = curr_obstacle.obstacle_edge_l_max;
      const std::string curr_obstacle_id = curr_obstacle.obstacle_id;

      if (Compare((curr_obstacle_box_l_min + curr_obstacle_box_l_max) / 2,
                  center_line) < 0) {
        // obstacle is to the right of center-line, should pass from left.
        if (std::get<2>(path_bound_point) <
                curr_obstacle_box_l_max +
                    ProcessBound::GetBufferBetweenADCCenterAndEdge() &&
            is_clear_to_expand_left_lane_bound_) {
          is_allow_expand_left_lane_bound_ = true;
        }
      } else {
        // obstacle is to the left of center-line, should pass from right.
        if (std::get<1>(path_bound_point) >
                curr_obstacle_box_l_min -
                    ProcessBound::GetBufferBetweenADCCenterAndEdge() &&
            is_clear_to_expand_right_lane_bound_) {
          is_allow_expand_right_lane_bound_ = true;
        }
      }
      ++obs_idx;
    }
  }
}

bool ProcessBound::CheckCameraLaneBoundaryType(const bool is_right_lane) {
  if (is_right_lane) {
    if (!frame_->local_view().GetLaneMarkers()->has_front_right_lane_marker()) {
      ADEBUG << "local view camera has no front_right_lane_marker.";
      return false;
    }
    const auto& adc_right_lane_boundary_type = frame_->local_view()
                                                   .GetLaneMarkers()
                                                   ->front_right_lane_marker()
                                                   .lane_type();
    ADEBUG << "local view camera adc_right_lane_boundary_type ["
           << hdmap::LaneBoundaryType_Type_Name(adc_right_lane_boundary_type)
           << "]";
    return adc_right_lane_boundary_type ==
               hdmap::LaneBoundaryType::SOLID_YELLOW ||
           adc_right_lane_boundary_type == hdmap::LaneBoundaryType::SOLID_WHITE;
  }

  if (!frame_->local_view().GetLaneMarkers()->has_front_left_lane_marker()) {
    ADEBUG << "local view camera has no front_left_lane_marker.";
    return false;
  }
  const auto& adc_left_lane_boundary_type = frame_->local_view()
                                                .GetLaneMarkers()
                                                ->front_left_lane_marker()
                                                .lane_type();
  ADEBUG << "local view camera adc_left_lane_boundary_type ["
         << hdmap::LaneBoundaryType_Type_Name(adc_left_lane_boundary_type)
         << "]";
  return adc_left_lane_boundary_type == hdmap::LaneBoundaryType::SOLID_YELLOW ||
         adc_left_lane_boundary_type == hdmap::LaneBoundaryType::SOLID_WHITE;
}

bool ProcessBound::CheckIfInJunction() const {
  // 判断是否是路口
  bool is_virtual_lane = false;
  // bool is_left_turn = false;
  const double kExtendVirtualLineArea = 20.0;
  const auto& adc_waypoint =
      reference_line_info_->reference_line().GetADCWaypoint();
  const double virtual_forward_s =
      std::min(adc_frenet_s_ + kExtendVirtualLineArea,
               GetReferenceLineInfo()->reference_line().Length());
  const auto& virtual_forward_points =
      reference_line_info_->reference_line()
          .GetNearestReferencePoint(virtual_forward_s)
          .lane_waypoints()
          .front();
  if (adc_waypoint.lane != nullptr) {
    auto adc_lane = adc_waypoint.lane;
    is_virtual_lane =
        (adc_lane->lane().left_boundary().virtual_() &&
         adc_lane->lane().right_boundary().virtual_()) ||
        (virtual_forward_points.lane->lane().left_boundary().virtual_() &&
         virtual_forward_points.lane->lane().right_boundary().virtual_());
    // if (adc_lane->lane().turn() == hdmap::Lane_LaneTurn_LEFT_TURN) {
    //   is_left_turn = true;
    // }
  }
  return is_virtual_lane;
}

}  // namespace planning
}  // namespace TL
