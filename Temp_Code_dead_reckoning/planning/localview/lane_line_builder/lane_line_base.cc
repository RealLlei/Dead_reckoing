/*
 * Copyright (c) TL auto Co., Ltd. 2022-2023. All rights reserved.
 */
#include "planning/localview/lane_line_builder/lane_line_base.h"
#include <vector>

namespace TL {
namespace planning {
static constexpr double kSearchRadius = 10.0;
static constexpr double kMaxHeadingDiff = M_PI;

bool LaneLineBase::JudgeIsInMapFirstly(
    const std::shared_ptr<LocalView>& local_view,
    const std::shared_ptr<hdmap::HDMap>& hd_map) {
  if (hd_map == nullptr || hd_map->Empty() || !local_view->HasLocalization()) {
    ADEBUG << "hd_map_ is nullptr or empty or has no Localization!!!";
    return false;
  }
  auto localization = local_view->GetLocalization();
  TL::common::math::Vec2d position_point(
      localization->pose().position().x(), localization->pose().position().y());
  common::PointENU point;
  point.set_x(localization->pose().position().x());
  point.set_y(localization->pose().position().y());
  // std::vector<std::shared_ptr<const hdmap::LaneInfo>> lanes;
  // const double search_distance = 15.0;
  // hdmap_->GetLanes(point, search_distance, &lanes);
  TL::hdmap::LaneInfoConstPtr nearest_lane;
  double s = 0;
  double l = 0;

  // hd_map->GetNearestLane(point, &nearest_lane, &s, &l);
  // if (nearest_lane) {
  //   AERROR << std::fixed << std::setprecision(9)
  //          << "test heading  = " << nearest_lane->headings().at(0)
  //          << "  vehicle heading = " << localization->pose().heading()
  //          << " x = " << localization->pose().position().x()
  //          << " y = " << localization->pose().position().y() << " s = " << s
  //          << " nearest_lane " << nearest_lane->lane().id().id();
  // }

  hd_map->GetNearestLaneWithHeading(
      localization->pose().position(), kSearchRadius,
      localization->pose().heading(), kMaxHeadingDiff, &nearest_lane, &s, &l);
  double angle_diff = 0.0;
  double angle_diff_delta = 0.8;
  bool is_diff_less_threshold = true;
  if (nearest_lane) {
    angle_diff = std::fabs(TL::common::math::NormalizeAngle(
        std::fabs(nearest_lane->Heading(s) - localization->pose().heading())));
    ADEBUG << "angle_diff = " << angle_diff << " is on lane "
           << nearest_lane->IsOnLane(position_point) << " successor "
           << nearest_lane->lane().successor_id_size() << " length "
           << nearest_lane->total_length() << ", s = " << s << ", l = " << l;
    is_diff_less_threshold = angle_diff < angle_diff_delta;
    if (!is_diff_less_threshold) {
      AERROR << "lane_heading:" << nearest_lane->Heading(s)
             << "  veh_heading:" << localization->pose().heading()
             << "  angle_diff:" << angle_diff;
    }
  }
#ifdef FOR_BAIDU_SIMULATION
  bool is_lane = nearest_lane && nearest_lane->IsOnLane(position_point) &&
                 angle_diff < 0.8;
  AERROR << "is_lane: " << is_lane
         << " ,is nearest_lane: " << (nearest_lane != nullptr)
         << " ,angle_diff: " << angle_diff << point.ShortDebugString();
  return is_lane;
#endif

  return nearest_lane && nearest_lane->IsOnLane(position_point) &&
         is_diff_less_threshold &&
         (nearest_lane->lane().successor_id_size() > 0 ||
          nearest_lane->total_length() - s > 50);
}

bool LaneLineBase::JudgeIsInMapContinuously(
    const std::shared_ptr<LocalView>& local_view,
    const std::shared_ptr<hdmap::HDMap>& hd_map) {
  if (hd_map == nullptr || hd_map->Empty() || !local_view->HasLocalization()) {
    AERROR << "hd_map_ is nullptr or empty or has no Localization!!!"
           << " ,is_hd_map nullptr: " << (hd_map == nullptr)
           << " ,is hd_map Empty: " << hd_map->Empty()
           << " ,HasLocalization: " << local_view->HasLocalization();
    return false;
  }
  bool is_in_hdmap = false;
  auto localization = local_view->GetLocalization();
  common::PointENU point;
  point.set_x(localization->pose().position().x());
  point.set_y(localization->pose().position().y());
  // std::vector<std::shared_ptr<const hdmap::LaneInfo>> lanes;
  std::vector<TL::hdmap::LaneInfoConstPtr> nearest_lanes;
  // const double kHeadingBuffer = M_PI / 10.0;
  // const double search_distance = 15.0;
  // hdmap_->GetLanesWithHeading(point, search_distance,
  //                             localization->pose().heading(),
  //                             M_PI / 2.0 + kHeadingBuffer, &lanes);
  hd_map->GetLanesWithHeading(localization->pose().position(), kSearchRadius,
                              localization->pose().heading(), kMaxHeadingDiff,
                              &nearest_lanes);
  // bool has_left_neighbor_lane = false;
  // bool has_right_neighbor_lane = false;
  for (const auto& lane : nearest_lanes) {
    if (lane != nullptr) {
      is_in_hdmap = true;
      break;
    }
    // if (lane->lane().successor_id_size() <= 0) {
    //   is_in_hdmap = true;
    //   break;
    // }
    // if (lane->lane().left_neighbor_forward_lane_id_size() > 0 &&
    //     hd_map->GetLaneById(
    //         lane->lane().left_neighbor_forward_lane_id().at(0)) != nullptr) {
    //   has_left_neighbor_lane =
    //       hd_map
    //           ->GetLaneById(lane->lane().left_neighbor_forward_lane_id().at(0))
    //           ->lane()
    //           .successor_id_size() > 0;
    //   if (has_left_neighbor_lane) {
    //     is_in_hdmap = true;
    //     break;
    //   }
    // }
    // if (lane->lane().right_neighbor_forward_lane_id_size() > 0 &&
    //     hd_map->GetLaneById(
    //         lane->lane().right_neighbor_forward_lane_id().at(0)) != nullptr) {
    //   has_right_neighbor_lane =
    //       hd_map
    //           ->GetLaneById(lane->lane().right_neighbor_forward_lane_id().at(0))
    //           ->lane()
    //           .successor_id_size() > 0;
    //   if (has_right_neighbor_lane) {
    //     is_in_hdmap = true;
    //     break;
    //   }
    // }
  }

  // if (local_view->HasValidLaneMarkersHeader() &&
  //     local_view->GetLaneMarkers()->has_front_right_lane_marker()) {
  //   is_in_hdmap = nearest_lane && std::fabs(l) < tolerance_width &&
  //                 (nearest_lane->lane().successor_id_size() > 0 ||
  //                  has_left_neighbor_lane || has_right_neighbor_lane ||
  //                  nearest_lane->total_length() - s >
  //                      fmin(30, nearest_lane->total_length() / 2));
  // } else {
  //   is_in_hdmap = nearest_lane && std::fabs(l) < tolerance_width &&
  //                 (nearest_lane->lane().successor_id_size() > 0 ||
  //                  has_left_neighbor_lane || has_right_neighbor_lane ||
  //                  nearest_lane->total_length() - s >
  //                      fmin(1, nearest_lane->total_length() / 2));
  // }
  // lblb for test
  // 这段代码，可以频繁退出地图模式，验证状态机切换是否有异常
  // if (is_in_hdmap && (nearest_lane->total_length() - s < 10.0)) {
  //   is_in_hdmap = false;
  // }
  ADEBUG << "is_in_hdmap_mapmode = " << is_in_hdmap;
  return is_in_hdmap;
}

bool LaneLineBase::IsNewRoutingRequest(
    const routing::RoutingRequest& prev_routing_request,
    const routing::RoutingRequest& routing_request) {
  if (prev_routing_request.waypoint_size() != routing_request.waypoint_size()) {
    return true;
  }
  for (int i = 0; i < routing_request.waypoint_size(); i++) {
    if (!common::util::IsProtoEqual(routing_request.waypoint().at(i),
                                    prev_routing_request.waypoint().at(i))) {
      return true;
    }
  }
  return false;
}

}  // namespace planning
}  // namespace TL
