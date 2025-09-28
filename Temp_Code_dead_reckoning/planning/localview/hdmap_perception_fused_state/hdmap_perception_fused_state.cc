/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * @LastEditors: Xu Xin
 *****************************************************************************/

#include "planning/localview/hdmap_perception_fused_state/hdmap_perception_fused_state.h"

#include <list>
#include <memory>

#include "absl/strings/match.h"
#include "absl/strings/str_cat.h"
#include "common/util/message_util.h"
#include "common/util/util.h"
#include "map/hdmap/hdmap_common.h"
#include "map/hdmap/hdmap_util.h"
#include "planning/common/util/util.h"

namespace TL {
namespace planning {

bool HDMapPerceptionFusedState::Init() {
  return true;
}

Status HDMapPerceptionFusedState::BuildLocalView(
    const std::shared_ptr<LocalView>& local_view_of_perception) {
  UNUSED(local_view_of_perception);
  return Status(ErrorCode::LOCALVIEW_FSM_PERCEPTIONFUSED_ERROR,
                "HDMapPerceptionFusedState buildlocalview fail");
}

Status HDMapPerceptionFusedState::BuildLocalView(
    const std::shared_ptr<LocalView>& local_view_of_perception,
    const routing::ChangeLaneType change_lane_type) {
  ADEBUG << " before UpdateRouting in hamap";
  routing::ChangeLaneType final_change_lane_type = change_lane_type;
  // routing::RoadSegment road = local_view_of_hdmap->GetRouting().road().at(0);
  auto routing_response = std::make_shared<routing::RoutingResponse>(
      *local_view_of_perception->GetRoutingResponse());
  auto map_msg = local_view_of_perception->GetMapMsg();
  if (current_change_lane_type_ == routing::ChangeLaneType::FORWARD) {
    current_change_lane_type_ = final_change_lane_type;
  } else {
    final_change_lane_type = current_change_lane_type_;
  }
  TL::hdmap::Map perception_map =
      local_view_of_perception->GetMapMsg()->hdmap();
  JudgePassage(&perception_map, routing_response.get(), final_change_lane_type);

  auto hdmap = std::make_shared<hdmap::HDMap>();
  hdmap->LoadMapFromProto(local_view_of_perception->GetMapMsg()->hdmap());
  hdmap::PncMap pnc_map_by_perception(hdmap);
  if (hdmap->GetLaneById(hdmap::MakeMapId("0_current")) == nullptr) {
    AERROR << "not find lane in perception map";
  }
  pnc_map_by_perception.UpdateReferenceLineInfoConfig(
      functionmanager::MachineStateType::HDMAP_PERCEPTION_FUSION,
      functionmanager::AvpFctOut::CRUISING,
      std::fabs(local_view_of_perception->GetVehicleState()->linear_velocity()),
      local_view_of_perception->GetFunctionManagerIn()
          ->fct_nnp_in()
          .longitud_ctrl_cruise_speedms());
  pnc_map_by_perception.UpdateRoutingResponse(*routing_response);
  std::list<TL::hdmap::RouteSegments> segments_by_perception;
  if (pnc_map_by_perception.GetRouteSegments(
          *local_view_of_perception->GetVehicleState(), &segments_by_perception,
          local_view_of_perception->GetFunctionManagerIn()
              ->fct_nnp_in()
              .longitud_ctrl_cruise_speedms()) &&
      segments_by_perception.begin()->NextAction() ==
          routing::ChangeLaneType::FORWARD) {
    current_change_lane_type_ = routing::ChangeLaneType::FORWARD;
  }
  common::util::FillHeader("from_navigation_routing", routing_response.get());
  local_view_of_perception->SetRoutingResponsePtr(routing_response);
  return Status::OK();
}

void HDMapPerceptionFusedState::JudgePassage(
    hdmap::Map* hd_map, routing::RoutingResponse* routing_response,
    routing::ChangeLaneType chgtype) {
  int lane_num = hd_map->lane_size();
  switch (chgtype) {
    case routing::ChangeLaneType::LEFT:
      ADEBUG << "***********changetypeleft";
      for (int i = 0; i < lane_num; ++i) {
        if (absl::EndsWith(hd_map->lane(i).id().id(), "left")) {
          routing_response->mutable_road()
              ->at(0)
              .mutable_passage()
              ->at(0)
              .set_change_lane_type(routing::ChangeLaneType::LEFT);
          AddOtherPassage(hd_map, routing_response, i);
        }
      }
      break;
    case routing::ChangeLaneType::RIGHT:
      ADEBUG << "***********changetypeRIGHT";
      for (int i = 0; i < lane_num; ++i) {
        if (absl::EndsWith(hd_map->lane(i).id().id(), "right")) {
          routing_response->mutable_road()
              ->at(0)
              .mutable_passage()
              ->at(0)
              .set_change_lane_type(routing::ChangeLaneType::RIGHT);
          AddOtherPassage(hd_map, routing_response, i);
        }
      }
      break;
    case routing::ChangeLaneType::FORWARD:
      ADEBUG << "***********changetypeFORWARD";
      break;
    default:
      break;
  }
}

void HDMapPerceptionFusedState::AddOtherPassage(
    hdmap::Map* hd_map, routing::RoutingResponse* routing_response, int index) {
  routing_response->clear_routing_request();
  auto* passage = routing_response->mutable_road()->at(0).add_passage();
  passage->set_can_exit(false);
  passage->set_change_lane_type(routing::ChangeLaneType::FORWARD);
  auto* segment = passage->add_segment();
  segment->set_id(hd_map->lane(index).id().id());
  auto adc_lane_segment_points = hd_map->lane(index)
                                     .central_curve()
                                     .segment()
                                     .at(0)
                                     .line_segment()
                                     .point();
  common::PointENU start_point = adc_lane_segment_points.at(0);
  int max_index = adc_lane_segment_points.size() - 1;
  common::PointENU end_point = adc_lane_segment_points.at(max_index);
  auto* routing_request = routing_response->mutable_routing_request();
  routing::LaneWaypoint waypoint;
  waypoint.set_id(hd_map->lane(index).id().id());
  waypoint.mutable_pose()->set_x(start_point.x());
  waypoint.mutable_pose()->set_y(start_point.y());
  waypoint.set_s(0.0);
  segment->set_start_s(0.0);
  routing_request->add_waypoint()->CopyFrom(waypoint);
  waypoint.set_s(hd_map->lane(index).length());
  segment->set_end_s(hd_map->lane(index).length());
  waypoint.mutable_pose()->set_x(end_point.x());
  waypoint.mutable_pose()->set_y(end_point.y());
  routing_request->add_waypoint()->CopyFrom(waypoint);
}
}  // namespace planning
}  // namespace TL
