/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  planning path pullover bound processor
 * Author: ROC
 */

#include "planning/tasks/deciders/path_bounds_decider/bound_processor/pullover_bound_processor.h"

#include <algorithm>
#include <cstddef>
#include <functional>
#include <limits>
#include <memory>
#include <set>

#include "common/configs/vehicle_config_helper.h"
#include "common/util/point_factory.h"
#include "planning/common/planning_context.h"
#include "planning/tasks/deciders/utils/path_decider_obstacle_utils.h"

namespace TL {
namespace planning {

// using TL::common::Clock;
using TL::common::ErrorCode;
using TL::common::Status;
using TL::common::VehicleConfigHelper;
using TL::hdmap::HDMapUtil;
using TL::hdmap::JunctionInfo;
using TL::planning::PathInfo;

PullOverBoundProcessor::PullOverBoundProcessor(
    const std::shared_ptr<DependencyInjector>& injector,
    const TaskConfig& config)
    : BoundProcessor(injector, config),
      process_bound_(new ProcessBound(injector, config)),
      obs_static_process_(new ObsStaticProcessor(injector, config)) {}

Status PullOverBoundProcessor::Process(
    ReferenceLineInfo* const reference_line_info, PathBound* const path_bound,
    Frame* const frame, std::vector<LaneType>* const lane_type_pool) {
  if (reference_line_info == nullptr || path_bound == nullptr ||
      frame == nullptr) {
    const std::string msg =
        "reference_line_info or path_bound or frame is nullptr.";
    AERROR << msg;
    return Status(ErrorCode::PLANNER_CRUISING_PATHBOUNDS_ERROR, msg);
  }

  UNUSED(lane_type_pool);

  // bound process init.
  process_bound_->InitPathBounds(frame, reference_line_info);

  // 1. Initialize the path boundaries to be an indefinitely large area.
  if (!process_bound_->InitPathBoundary(path_bound, GetInjector())) {
    const std::string msg = "Failed to initialize path boundaries.";
    AERROR << msg;
    return Status(ErrorCode::PLANNER_CRUISING_PATHBOUNDS_ERROR, msg);
  }

  // 2. Decide a rough boundary based on road boundary
  if (!GetBoundaryFromRoads(*reference_line_info, path_bound)) {
    const std::string msg =
        "Failed to decide a rough boundary based on road boundary.";
    AERROR << msg;
    return Status(ErrorCode::PLANNER_CRUISING_PATHBOUNDS_ERROR, msg);
  }
  ADEBUG << "Get boundary from road for pull over path bound.";
  PathInfo::PathBoundsDebugString(*path_bound);

  ConvertBoundarySAxisFromLaneCenterToRefLine(*reference_line_info, path_bound);
  if (process_bound_->GetAdcFrenetL() < std::get<1>(path_bound->front()) ||
      process_bound_->GetAdcFrenetL() > std::get<2>(path_bound->front())) {
    const std::string msg =
        "ADC is outside road boundary already. Cannot generate pull-over "
        "path";
    AERROR << msg;
    return Status(ErrorCode::PLANNER_CRUISING_PATHBOUNDS_ERROR, msg);
  }

  // 2. Update boundary by lane boundary for pull_over
  UpdatePullOverBoundaryByLaneBoundary(*reference_line_info, path_bound);
  ADEBUG << "Get boundary from lane boundary for pull over path bound.";
  PathInfo::PathBoundsDebugString(*path_bound);

  // 3. Fine-tune the boundary based on static obstacles
  PathBound temp_path_bound = *path_bound;
  std::string blocking_obstacle_id;
  TowingPointsInfo towing_points;
  towing_points.resize(
      path_bound->size(),
      std::tuple<double, double, double, std::string>(0.0, 0.0, 0.0, "0"));
  if (!obs_static_process_->Process(reference_line_info, frame, path_bound,
                                    &blocking_obstacle_id, &towing_points,
                                    false)) {
    const std::string msg =
        "Failed to decide fine tune the boundaries after "
        "taking into consideration all static obstacles.";
    AERROR << msg;
    return Status(ErrorCode::PLANNER_CRUISING_PATHBOUNDS_ERROR, msg);
  }
  ADEBUG << "Get boundary finally for pull over path bound.";
  PathInfo::PathBoundsDebugString(*path_bound);

  auto* pull_over_status = GetInjector()
                               ->planning_context()
                               ->mutable_planning_status()
                               ->mutable_pull_over();
  // If already found a pull-over position, simply check if it's valid.
  int curr_idx = -1;
  if (pull_over_status->has_position()) {
    curr_idx = IsPointWithinPathBound(
        *reference_line_info, pull_over_status->position().x(),
        pull_over_status->position().y(), *path_bound);
  }

  // If haven't found a pull-over position, search for one.
  if (curr_idx < 0) {
    auto pull_over_type = pull_over_status->pull_over_type();
    pull_over_status->Clear();
    pull_over_status->set_pull_over_type(pull_over_type);
    pull_over_status->set_plan_pull_over_path(true);

    std::tuple<double, double, double, int> pull_over_configuration;
    if (!SearchPullOverPosition(*process_bound_->GetFrame(),
                                *reference_line_info, *path_bound,
                                &pull_over_configuration)) {
      const std::string msg = "Failed to find a proper pull-over position.";
      AERROR << msg;
      return Status(ErrorCode::PLANNER_CRUISING_PATHBOUNDS_ERROR, msg);
    }

    curr_idx = std::get<3>(pull_over_configuration);

    // If have found a pull-over position, update planning-context
    pull_over_status->mutable_position()->set_x(
        std::get<0>(pull_over_configuration));
    pull_over_status->mutable_position()->set_y(
        std::get<1>(pull_over_configuration));
    pull_over_status->mutable_position()->set_z(0.0);
    pull_over_status->set_theta(std::get<2>(pull_over_configuration));
    pull_over_status->set_length_front(GetConfig()
                                           .path_bounds_decider_config()
                                           .obstacle_buffer_process_config()
                                           .obs_static_lon_start_buffer());
    pull_over_status->set_length_back(GetConfig()
                                          .path_bounds_decider_config()
                                          .obstacle_buffer_process_config()
                                          .obs_static_lon_end_buffer());
    pull_over_status->set_width_left(
        VehicleConfigHelper::GetConfig().vehicle_param().width() / 2.0);
    pull_over_status->set_width_right(
        VehicleConfigHelper::GetConfig().vehicle_param().width() / 2.0);

    ADEBUG << "Pull Over: x[" << pull_over_status->position().x() << "] y["
           << pull_over_status->position().y() << "] theta["
           << pull_over_status->theta() << "]";
  }

  // Trim path-bound properly
  while (static_cast<int>(path_bound->size()) - 1 >
         curr_idx + kNumExtraTailBoundPoint) {
    path_bound->pop_back();
  }
  for (size_t idx = curr_idx + 1; idx < path_bound->size(); ++idx) {
    std::get<1>((*path_bound)[idx]) = std::get<1>((*path_bound)[curr_idx]);
    std::get<2>((*path_bound)[idx]) = std::get<2>((*path_bound)[curr_idx]);
  }
  PathInfo::PathBoundDebugInfo(PathBoundType::PULL_OVER_PATH_BOUND, *path_bound,
                               reference_line_info);
  return Status::OK();
}

bool PullOverBoundProcessor::GetBoundaryFromRoads(
    const ReferenceLineInfo& reference_line_info, PathBound* const path_bound) {
  // Sanity checks.
  if (path_bound == nullptr || path_bound->empty()) {
    AERROR << "path_bound is nullptr or empty.";
    return false;
  }
  const ReferenceLine& reference_line = reference_line_info.reference_line();

  // Go through every point, update the boudnary based on the road boundary.
  double past_road_left_width = process_bound_->GetAdcDefaultLaneWidth() / 2;
  double past_road_right_width = process_bound_->GetAdcDefaultLaneWidth() / 2;
  int path_blocked_idx = -1;
  for (size_t i = 0; i < path_bound->size(); ++i) {
    // 1. Get road boundary.
    double curr_s = std::get<0>((*path_bound)[i]);
    double curr_road_left_width = 0.0;
    double curr_road_right_width = 0.0;
    double refline_offset_to_lane_center = 0.0;
    reference_line.GetOffsetToMap(curr_s, &refline_offset_to_lane_center);
    if (!reference_line.GetRoadWidth(curr_s, &curr_road_left_width,
                                     &curr_road_right_width)) {
      AWARN << "Failed to get lane width at s = " << curr_s;
      curr_road_left_width = past_road_left_width;
      curr_road_right_width = past_road_right_width;
    } else {
      curr_road_left_width += refline_offset_to_lane_center;
      curr_road_right_width -= refline_offset_to_lane_center;
      past_road_left_width = curr_road_left_width;
      past_road_right_width = curr_road_right_width;
    }
    double curr_left_bound = curr_road_left_width;
    double curr_right_bound = -curr_road_right_width;
    ADEBUG << "At s = " << curr_s
           << ", left road bound = " << curr_road_left_width
           << ", right road bound = " << curr_road_right_width
           << ", offset from refline to lane-center = "
           << refline_offset_to_lane_center;

    // 2. Update into path_bound.
    if (!process_bound_->UpdatePathBoundaryWithBuffer(
            i, curr_left_bound, curr_right_bound, path_bound,
            reference_line_info)) {
      path_blocked_idx = static_cast<int>(i);
    }
    if (path_blocked_idx != -1) {
      break;
    }
  }

  ProcessBound::TrimPathBounds(path_blocked_idx, path_bound);
  return true;
}

void PullOverBoundProcessor::ConvertBoundarySAxisFromLaneCenterToRefLine(
    const ReferenceLineInfo& reference_line_info, PathBound* const path_bound) {
  if (path_bound == nullptr || path_bound->empty()) {
    AERROR << "There is no path_bound points!";
    return;
  }
  const ReferenceLine& reference_line = reference_line_info.reference_line();
  for (auto& bound : *path_bound) {
    // 1. Get road boundary.
    double curr_s = std::get<0>(bound);
    double refline_offset_to_lane_center = 0.0;
    reference_line.GetOffsetToMap(curr_s, &refline_offset_to_lane_center);
    std::get<1>(bound) -= refline_offset_to_lane_center;
    std::get<2>(bound) -= refline_offset_to_lane_center;
  }
}

// update boundaries with corresponding one-side lane boundary for pull over
// (1) use left lane boundary for normal PULL_OVER type
// (2) use left/right(which is opposite to pull over direction
//     (pull over at closer road side) lane boundary for EMERGENCY_PULL_OVER
void PullOverBoundProcessor::UpdatePullOverBoundaryByLaneBoundary(
    const ReferenceLineInfo& reference_line_info, PathBound* const path_bound) {
  if (path_bound == nullptr) {
    AERROR << "UpdatePullOverBoundaryByLaneBoundary nullptr check is failed!";
    return;
  }
  const ReferenceLine& reference_line = reference_line_info.reference_line();
  const auto& pull_over_status =
      GetInjector()->planning_context()->planning_status().pull_over();
  const auto pull_over_type = pull_over_status.pull_over_type();
  if (pull_over_type != PullOverStatus::PULL_OVER &&
      pull_over_type != PullOverStatus::EMERGENCY_PULL_OVER) {
    return;
  }

  for (auto& bound : *path_bound) {
    const double curr_s = std::get<0>(bound);
    double left_bound = 3.0;
    double right_bound = 3.0;
    double curr_lane_left_width = 0.0;
    double curr_lane_right_width = 0.0;
    if (reference_line.GetLaneWidth(curr_s, &curr_lane_left_width,
                                    &curr_lane_right_width)) {
      double offset_to_lane_center = 0.0;
      reference_line.GetOffsetToMap(curr_s, &offset_to_lane_center);
      left_bound = curr_lane_left_width + offset_to_lane_center;
      right_bound = curr_lane_right_width + offset_to_lane_center;
    }
    ADEBUG << "left_bound[" << left_bound << "] right_bound[" << right_bound
           << "]";
    if (pull_over_type == PullOverStatus::PULL_OVER ||
        pull_over_type == PullOverStatus::EMERGENCY_PULL_OVER) {
      std::get<2>(bound) = left_bound;
    }
  }
}

int PullOverBoundProcessor::IsPointWithinPathBound(
    const ReferenceLineInfo& reference_line_info, const double x,
    const double y,
    const std::vector<std::tuple<double, double, double>>& path_bound) {
  common::SLPoint point_sl;
  reference_line_info.reference_line().XYToSL({x, y}, &point_sl);
  if (point_sl.s() > std::get<0>(path_bound.back()) ||
      point_sl.s() <
          std::get<0>(path_bound.front()) -
              reference_line_info.PathBoundsDeciderResolution() * 2) {
    ADEBUG << "Longitudinally outside the boundary.";
    return -1;
  }
  int idx_after = 0;
  while (idx_after < static_cast<int>(path_bound.size()) &&
         std::get<0>(path_bound[idx_after]) < point_sl.s()) {
    ++idx_after;
  }
  ADEBUG << "The idx_after = " << idx_after;
  ADEBUG << "The boundary is: "
         << "[" << std::get<1>(path_bound[idx_after]) << ", "
         << std::get<2>(path_bound[idx_after]) << "].";
  ADEBUG << "The point is at: " << point_sl.l();
  int idx_before = idx_after - 1;
  if (std::get<1>(path_bound[idx_before]) <= point_sl.l() &&
      std::get<2>(path_bound[idx_before]) >= point_sl.l() &&
      std::get<1>(path_bound[idx_after]) <= point_sl.l() &&
      std::get<2>(path_bound[idx_after]) >= point_sl.l()) {
    return idx_after;
  }
  ADEBUG << "Laterally outside the boundary.";
  return -1;
}

bool PullOverBoundProcessor::SearchPullOverPosition(
    const Frame& frame, const ReferenceLineInfo& reference_line_info,
    const std::vector<std::tuple<double, double, double>>& path_bound,
    std::tuple<double, double, double, int>* const pull_over_configuration) {
  if (pull_over_configuration == nullptr) {
    AERROR << "pull_over_configuration is nullptr.";
    return false;
  }

  const auto& pull_over_status =
      GetInjector()->planning_context()->planning_status().pull_over();

  // search direction
  bool search_backward = false;  // search FORWARD by default

  double pull_over_s = 0.0;
  if (pull_over_status.pull_over_type() ==
      PullOverStatus::EMERGENCY_PULL_OVER) {
    if (!FindEmergencyPullOverS(reference_line_info, &pull_over_s)) {
      AERROR << "Failed to find emergency_pull_over s";
      return false;
    }
    search_backward = false;  // search FORWARD from target position
  } else if (pull_over_status.pull_over_type() == PullOverStatus::PULL_OVER) {
    if (!FindDestinationPullOverS(frame, reference_line_info, path_bound,
                                  &pull_over_s)) {
      AERROR << "Failed to find pull_over s upon destination arrival";
      return false;
    }
    search_backward = true;  // search BACKWARD from target position
  } else {
    return false;
  }

  int idx = 0;
  if (search_backward) {
    // 1. Locate the first point before destination.
    idx = static_cast<int>(path_bound.size()) - 1;
    while (idx >= 0 && std::get<0>(path_bound[idx]) > pull_over_s) {
      --idx;
    }
  } else {
    // 1. Locate the first point after emergency_pull_over s.
    while (idx < static_cast<int>(path_bound.size()) &&
           std::get<0>(path_bound[idx]) < pull_over_s) {
      ++idx;
    }
  }
  if (idx < 0 || idx >= static_cast<int>(path_bound.size())) {
    AERROR << "Failed to find path_bound index for pull over s";
    return false;
  }

  // Search for a feasible location for pull-over.
  const double pull_over_space_length =
      kPulloverLonSearchCoeff *
          VehicleConfigHelper::GetConfig().vehicle_param().length() -
      GetConfig()
          .path_bounds_decider_config()
          .obstacle_buffer_process_config()
          .obs_static_lon_start_buffer() -
      GetConfig()
          .path_bounds_decider_config()
          .obstacle_buffer_process_config()
          .obs_static_lon_end_buffer();
  const double pull_over_space_width =
      (kPulloverLatSearchCoeff - 1.0) *
      VehicleConfigHelper::GetConfig().vehicle_param().width();
  const double adc_half_width =
      VehicleConfigHelper::GetConfig().vehicle_param().width() / 2.0;

  // 2. Find a window that is close to road-edge.
  // (not in any intersection)
  bool has_a_feasible_window = false;
  while ((search_backward && idx >= 0 &&
          std::get<0>(path_bound[idx]) - std::get<0>(path_bound.front()) >
              pull_over_space_length) ||
         (!search_backward && idx < static_cast<int>(path_bound.size()) &&
          std::get<0>(path_bound.back()) - std::get<0>(path_bound[idx]) >
              pull_over_space_length)) {
    int j = idx;
    bool is_feasible_window = true;

    // Check if the point of idx is within intersection.
    double pt_ref_line_s = std::get<0>(path_bound[idx]);
    double pt_ref_line_l = 0.0;
    common::SLPoint pt_sl;
    pt_sl.set_s(pt_ref_line_s);
    pt_sl.set_l(pt_ref_line_l);
    common::math::Vec2d pt_xy;
    reference_line_info.reference_line().SLToXY(pt_sl, &pt_xy);
    common::PointENU hdmap_point;
    hdmap_point.set_x(pt_xy.x());
    hdmap_point.set_y(pt_xy.y());
    ADEBUG << "Pull-over position might be around (" << pt_xy.x() << ", "
           << pt_xy.y() << ")";
    std::vector<std::shared_ptr<const JunctionInfo>> junctions;
    HDMapUtil::MapForPlanning().GetJunctions(hdmap_point, 1.0, &junctions);
    if (!junctions.empty()) {
      AWARN << "Point is in PNC-junction.";
      idx = search_backward ? idx - 1 : idx + 1;
      continue;
    }

    while ((search_backward && j >= 0 &&
            std::get<0>(path_bound[idx]) - std::get<0>(path_bound[j]) <
                pull_over_space_length) ||
           (!search_backward && j < static_cast<int>(path_bound.size()) &&
            std::get<0>(path_bound[j]) - std::get<0>(path_bound[idx]) <
                pull_over_space_length)) {
      double curr_s = std::get<0>(path_bound[j]);
      double curr_right_bound = std::fabs(std::get<1>(path_bound[j]));
      double curr_road_left_width = 0;
      double curr_road_right_width = 0;
      reference_line_info.reference_line().GetRoadWidth(
          curr_s, &curr_road_left_width, &curr_road_right_width);
      ADEBUG << "s[" << curr_s << "] curr_road_left_width["
             << curr_road_left_width << "] curr_road_right_width["
             << curr_road_right_width << "]";
      if (curr_road_right_width - (curr_right_bound + adc_half_width) >
          GetConfig()
              .path_bounds_decider_config()
              .pull_over_process_config()
              .pull_over_road_edge_buffer()) {
        AERROR << "Not close enough to road-edge. Not feasible for pull-over.";
        is_feasible_window = false;
        break;
      }
      const double right_bound = std::get<1>(path_bound[j]);
      const double left_bound = std::get<2>(path_bound[j]);
      ADEBUG << "left_bound[" << left_bound << "] right_bound[" << right_bound
             << "]";
      if (left_bound - right_bound < pull_over_space_width) {
        AERROR << "Not wide enough to fit ADC. Not feasible for pull-over.";
        is_feasible_window = false;
        break;
      }

      j = search_backward ? j - 1 : j + 1;
    }
    if (j < 0) {
      return false;
    }
    if (is_feasible_window) {
      has_a_feasible_window = true;
      const auto& reference_line = reference_line_info.reference_line();
      // estimate pull over point to have the vehicle keep same safety
      // distance to front and back
      const auto& vehicle_param =
          VehicleConfigHelper::GetConfig().vehicle_param();
      const double back_clear_to_total_length_ratio =
          (0.5 * (kPulloverLonSearchCoeff - 1.0) * vehicle_param.length() +
           vehicle_param.back_edge_to_center()) /
          vehicle_param.length() / kPulloverLonSearchCoeff;

      int start_idx = j;
      int end_idx = idx;
      if (!search_backward) {
        start_idx = idx;
        end_idx = j;
      }
      auto pull_over_idx = static_cast<size_t>(
          back_clear_to_total_length_ratio * static_cast<double>(end_idx) +
          (1.0 - back_clear_to_total_length_ratio) *
              static_cast<double>(start_idx));

      const auto& pull_over_point = path_bound[pull_over_idx];
      const double pull_over_s = std::get<0>(pull_over_point);
      const double pull_over_l =
          std::get<1>(pull_over_point) + pull_over_space_width / 2.0;
      common::SLPoint pull_over_sl_point;
      pull_over_sl_point.set_s(pull_over_s);
      pull_over_sl_point.set_l(pull_over_l);

      common::math::Vec2d pull_over_xy_point;
      reference_line.SLToXY(pull_over_sl_point, &pull_over_xy_point);
      const double pull_over_x = pull_over_xy_point.x();
      const double pull_over_y = pull_over_xy_point.y();

      // set the pull over theta to be the nearest lane theta rather than
      // reference line theta in case of reference line theta not aligned with
      // the lane
      const auto& reference_point =
          reference_line.GetReferencePoint(pull_over_s);
      double pull_over_theta = reference_point.heading();
      hdmap::LaneInfoConstPtr lane;
      double s = 0.0;
      double l = 0.0;
      auto point =
          common::util::PointFactory::ToPointENU(pull_over_x, pull_over_y);
      static constexpr double kSearchRadius = 5.0;
      if (HDMapUtil::MapForPlanning().GetNearestLaneWithHeading(
              point, kSearchRadius, pull_over_theta, M_PI_2, &lane, &s, &l) ==
          0) {
        pull_over_theta = lane->Heading(s);
      }
      *pull_over_configuration =
          std::make_tuple(pull_over_x, pull_over_y, pull_over_theta,
                          static_cast<int>(pull_over_idx));
      break;
    }

    idx = search_backward ? idx - 1 : idx + 1;
  }

  return has_a_feasible_window;
}

bool PullOverBoundProcessor::FindEmergencyPullOverS(
    const ReferenceLineInfo& reference_line_info, double* pull_over_s) {
  if (pull_over_s == nullptr) {
    AERROR << "FindEmergencyPullOverS nullptr check is failed!";
    return false;
  }
  const double adc_end_s = reference_line_info.AdcSlBoundary().end_s();
  const double min_turn_radius = common::VehicleConfigHelper::GetConfig()
                                     .vehicle_param()
                                     .min_turn_radius();
  const double adjust_factor =
      GetConfig()
          .path_bounds_decider_config()
          .pull_over_process_config()
          .pull_over_approach_lon_distance_adjust_factor();
  const double pull_over_distance = min_turn_radius * 2 * adjust_factor;
  *pull_over_s = adc_end_s + pull_over_distance;

  return true;
}

bool PullOverBoundProcessor::FindDestinationPullOverS(
    const Frame& frame, const ReferenceLineInfo& reference_line_info,
    const std::vector<std::tuple<double, double, double>>& path_bound,
    double* pull_over_s) {
  if (pull_over_s == nullptr) {
    AERROR << "pull_over_s is nullptr.";
    return false;
  }

  // destination_s based on routing_end
  const auto& reference_line = reference_line_info.reference_line();
  common::SLPoint destination_sl;
  const auto routing = frame.local_view().GetRoutingResponse();
  const auto& routing_end = *(routing->routing_request().waypoint().rbegin());
  reference_line.XYToSL(routing_end.pose(), &destination_sl);
  const double destination_s = destination_sl.s();
  const double adc_end_s = reference_line_info.AdcSlBoundary().end_s();

  // Check if destination is some distance away from ADC.
  ADEBUG << "Destination s[" << destination_s << "] adc_end_s[" << adc_end_s
         << "]";
  if (destination_s - adc_end_s < GetConfig()
                                      .path_bounds_decider_config()
                                      .pull_over_process_config()
                                      .pull_over_destination_to_adc_buffer()) {
    AERROR << "Destination is too close to ADC. distance["
           << destination_s - adc_end_s << "]";
    return false;
  }

  // Check if destination is within path-bounds searching scope.
  const double destination_to_pathend_buffer =
      GetConfig()
          .path_bounds_decider_config()
          .pull_over_process_config()
          .pull_over_destination_to_pathend_buffer();
  if (destination_s + destination_to_pathend_buffer >=
      std::get<0>(path_bound.back())) {
    AERROR << "Destination is not within path_bounds search scope";
    return false;
  }

  *pull_over_s = destination_s;
  return true;
}
}  // namespace planning
}  // namespace TL
