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
 *****************************************************************************/

#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/navigation_hdmap_lane_line.h"

#include <cmath>
#include <list>
#include <utility>
#include <vector>

#include "common/configs/config_gflags.h"
#include "common/file/file.h"
#include "common/math/vec2d.h"
#include "common/status/status.h"
#include "common/util/message_util.h"
#include "common/util/util.h"
#include "planning/common/planning_gflags.h"
#include "proto/perception/perception_obstacle.pb.h"

namespace TL {
namespace planning {

using TL::common::ErrorCode;
using TL::common::Status;
// using TL::common::monitor::MonitorMessageItem;
using TL::common::Clock;
using TL::perception::LaneMarkers;

NavigationHdmap::NavigationHdmap()
    : laneline_rise_debounce_{FLAGS_lane_line_rise_limit_time, 0.0, 0.1},
      lane_status_to_mcu_debounce_{0.3, 0.0, 0.1} {}  // laneline模式居中判断3帧

Status NavigationHdmap::Init() {
  config_.Clear();
  if (!common::GetProtoFromFile(FLAGS_navigation_hdmap_config_filename,
                                &config_)) {
    return Status(ErrorCode::LOCALVIEW_MAP_LANE_ERROR,
                  "Unable to load relative map conf file: " +
                      FLAGS_navigation_hdmap_config_filename);
  }
  // lanemarker_logical_decider_
  lanemarker_new_decider_ =
      std::make_unique<lanelineprocess::LanemarkerNewDecider>(config_);
  lanemarker_new_decider_->Init();
  navigation_lane_path_generator_ =
      std::make_unique<NavigationLanePathGenerator>(config_);
  navigation_lane_path_generator_->Init();
  navigation_lane_map_creator_ =
      std::make_unique<NavigationLaneMapCreator>(config_);
  navigation_lane_map_creator_->Init();
  return Status::OK();
}

void LogErrorStatus(navigation_hdmap::MapMsg* current_map_msg_,
                    const std::string& error_msg) {
  auto* status = current_map_msg_->mutable_header()->mutable_status();
  status->set_msg(error_msg);
  status->set_error_code(ErrorCode::LOCALVIEW_MAP_LANE_ERROR);
}

TL::common::Status NavigationHdmap::Start() {
  // monitor_logger_buffer_.INFO("NavigationHdmap started");
  return Status::OK();
}

bool NavigationHdmap::Process(
    const std::shared_ptr<LocalView>& local_view,
    functionmanager::FunctionManagerOut* const to_fct) {
  current_routing_response_ = std::make_shared<routing::RoutingResponse>();

  // if (current_map_msg_.use_count() != 1) {
  //   current_map_msg_ = std::make_shared<navigation_hdmap::MapMsg>();
  // } else {
  //   current_map_msg_->Clear();
  // }

  if (local_view->HasArena()) {
    current_map_msg_ =
        common::memory::ArenaAdapter::CreateMessage<navigation_hdmap::MapMsg>(
            local_view->GetArena());
  } else {
    current_map_msg_ = std::make_shared<navigation_hdmap::MapMsg>();
  }

  // fct模式下，只跑地图模式，不会跳转到感知，所以感知不运行
  if (FLAGS_use_fct_mode) {
    DebounceStatus(to_fct, false);
    return false;
  }

  current_map_msg_->mutable_lane_marker()->CopyFrom(
      *local_view->GetLaneMarkers());
  if (!NewLanemarkerDecider(local_view, to_fct)) {
    DebounceStatus(to_fct, false);
    ADEBUG << " LaneMarker Decision faild";
    return false;
  }
  ADEBUG << "There is/are " << current_map_msg_->navigation_path().size()
         << " navigation path(s) in the current reltative map.";
  // load navigation_hdmap every cycle time
  if (current_map_msg_->hdmap().lane_size() > 0) {
    ADEBUG << "get routing:" << current_map_msg_->routing().DebugString();
    common::util::FillHeader(
        "from_navigation_hdmap",
        current_map_msg_->mutable_hdmap()->mutable_header());
    common::util::FillHeader("navigation_hdmap", current_map_msg_.get());
    current_routing_response_->CopyFrom(current_map_msg_->routing());
    ADEBUG << "navigation map success";
    DebounceStatus(to_fct, FLAGS_enable_laneline_mode,
                   LaneLineStatusToMcu(local_view, to_fct));
    return FLAGS_enable_laneline_mode;
  }
  DebounceStatus(to_fct, false);
  AERROR << "No navigation map";
  return false;
}

void NavigationHdmap::Stop() {
  // monitor_logger_buffer_.INFO("NavigationHdmap stopped");
}

planning::PerceptionMapConfig NavigationHdmap::GetConfig() const {
  return config_;
}

bool NavigationHdmap::CheckLanemarkers(
    LaneMarkers* lane_markers,
    const std::shared_ptr<const perception::LaneMarkers>& lanemarkers_input) {
  if (lanemarkers_input == nullptr) {
    AERROR << "check lanemarker is nullptr!!!";
    return false;
  }
  if (!lanemarkers_input->has_header()) {
    common::util::FillHeader("lane", lane_markers);
    ADEBUG << "Check lanemarker has no header!!!";
    // return false;
  } else {
    lane_markers->mutable_header()->CopyFrom(lanemarkers_input->header());
  }
  // front_left_lane_marker
  if ((!lanemarkers_input->has_front_left_lane_marker()) ||
      (!lanemarkers_input->front_left_lane_marker().has_c0_position())) {
    DefaultLanemarker(lane_markers->mutable_front_left_lane_marker());
  } else {
    CopyLanemarker(lane_markers->mutable_front_left_lane_marker(),
                   lanemarkers_input->front_left_lane_marker());
  }
  // front_right_lane_marker
  if (!lanemarkers_input->has_front_right_lane_marker() ||
      !lanemarkers_input->front_right_lane_marker().has_c0_position()) {
    DefaultLanemarker(lane_markers->mutable_front_right_lane_marker());
  } else {
    CopyLanemarker(lane_markers->mutable_front_right_lane_marker(),
                   lanemarkers_input->front_right_lane_marker());
  }
  // front_next_left_lane_marker
  if (lanemarkers_input->front_next_left_lane_marker_size() < 1) {
    DefaultLanemarker(lane_markers->add_front_next_left_lane_marker());
  } else {
    lane_markers->mutable_front_next_left_lane_marker()->Reserve(
        lanemarkers_input->front_next_left_lane_marker_size());
    for (const auto& lanemarker :
         lanemarkers_input->front_next_left_lane_marker()) {
      CopyLanemarker(lane_markers->add_front_next_left_lane_marker(),
                     lanemarker);
    }
  }
  // front_next_right_lane_marker
  if (lanemarkers_input->front_next_right_lane_marker_size() < 1) {
    DefaultLanemarker(lane_markers->add_front_next_right_lane_marker());
  } else {
    lane_markers->mutable_front_next_right_lane_marker()->Reserve(
        lanemarkers_input->front_next_right_lane_marker_size());
    for (const auto& lanemarker :
         lanemarkers_input->front_next_right_lane_marker()) {
      CopyLanemarker(lane_markers->add_front_next_right_lane_marker(),
                     lanemarker);
    }
  }
  // front_left_road_edge
  if (lanemarkers_input->has_front_left_road_edge()) {
    CopyLanemarker(lane_markers->mutable_front_left_road_edge(),
                   lanemarkers_input->front_left_road_edge());
  }
  // front_right_road_edge
  if (lanemarkers_input->has_front_right_road_edge()) {
    CopyLanemarker(lane_markers->mutable_front_right_road_edge(),
                   lanemarkers_input->front_right_road_edge());
  }
  if (lanemarkers_input->has_is_lanechange_to_right()) {
    lane_markers->set_is_lanechange_to_right(
        lanemarkers_input->is_lanechange_to_right());
  }
  if (lanemarkers_input->has_is_lanechange_to_left()) {
    lane_markers->set_is_lanechange_to_left(
        lanemarkers_input->is_lanechange_to_left());
  }
  if (lanemarkers_input->has_lane_count()) {
    lane_markers->set_lane_count(lanemarkers_input->lane_count());
  }
  return true;
}

void NavigationHdmap::CopyLanemarker(LaneMarker* lane_marker,
                                     const LaneMarker& lanemarker_input) {
  lane_marker->set_lane_type(lanemarker_input.lane_type());
  lane_marker->set_quality(lanemarker_input.quality());
  lane_marker->set_model_degree(lanemarker_input.model_degree());
  lane_marker->set_c0_position(lanemarker_input.c0_position());
  lane_marker->set_c1_heading_angle(lanemarker_input.c1_heading_angle());
  lane_marker->set_c2_curvature(lanemarker_input.c2_curvature());
  lane_marker->set_c3_curvature_derivative(
      lanemarker_input.c3_curvature_derivative());
  lane_marker->set_view_range(lanemarker_input.view_range());
  lane_marker->set_longitude_start(lanemarker_input.longitude_start());
  lane_marker->set_longitude_end(lanemarker_input.longitude_end());
  lane_marker->set_line_seq(lanemarker_input.line_seq());
  lane_marker->set_dev_c0_position(lanemarker_input.dev_c0_position());
  lane_marker->set_dev_c1_heading_angle(
      lanemarker_input.dev_c1_heading_angle());
  lane_marker->set_dev_c2_curvature(lanemarker_input.dev_c2_curvature());
  lane_marker->set_dev_c3_curvature_derivative(
      lanemarker_input.dev_c3_curvature_derivative());
  lane_marker->set_adf_type(lanemarker_input.adf_type());
  lane_marker->set_adf_color(lanemarker_input.adf_color());
}

bool NavigationHdmap::NewLanemarkerDecider(
    const std::shared_ptr<LocalView>& local_view,
    functionmanager::FunctionManagerOut* const to_fct) {
  ADEBUG << "new lanemarker decider!!!";
  navigation_lane_path_generator_->SetVehicleState(
      local_view->GetVehicleState());
  navigation_lane_map_creator_->SetVehicleState(local_view->GetVehicleState());
  navigation_lane_map_creator_->SetFctIn(local_view->GetFunctionManagerIn());
  lanemarker_new_decider_->SetDecisionMg(local_view, to_fct, laneline_status_);
  LaneMarkers lane_markers;
  if (!CheckLanemarkers(&lane_markers, local_view->GetLaneMarkers())) {
    return false;
  }
  auto start_time = Clock::NowInMicroseconds();
  std::unordered_map<std::string, std::vector<Vec2d>> lane_marker_points;
  std::pair<bool, bool> lane_markers_points_copy;
  if (!lanemarker_new_decider_->Decision(&lane_markers, &lane_marker_points,
                                         local_view, &lane_markers_points_copy)) {
    ADEBUG << " LaneMarker Logical Decision faild";
    LogErrorStatus(current_map_msg_.get(), "Failed to lanemarker new decider.");
    lanemarkerdebug_ = lanemarker_new_decider_->GetLanemarkerDebugPtr();
    lanemarkerdebug_->set_lanemarker_new_decider_time(
        Clock::NowInMicroseconds() - start_time);
    lanemarkerdebug_->mutable_header()->mutable_status()->CopyFrom(
        current_map_msg_->header().status());
    return false;
  }
  auto new_decider_time = Clock::NowInMicroseconds();
  lanemarkerdebug_ = lanemarker_new_decider_->GetLanemarkerDebugPtr();
  lanemarkerdebug_->set_lanemarker_new_decider_time(new_decider_time -
                                                    start_time);
  if (fLB::FLAGS_using_camera_lanemarker_points) {
    lane_marker_points.clear();
    Lanemarkers2Points(*local_view->GetLaneMarkers(), &lane_marker_points);
    if (std::get<0>(lane_markers_points_copy) &&
        !std::get<1>(lane_markers_points_copy)) {
      std::vector<Vec2d> tem_points;
      DoProjection_test(lane_marker_points.at("right"), -3.75, &tem_points);
      lane_marker_points.at("left").swap(tem_points);
    }
    if (!std::get<0>(lane_markers_points_copy) &&
        std::get<1>(lane_markers_points_copy)) {
      std::vector<Vec2d> tem_points;
      DoProjection_test(lane_marker_points.at("left"), 3.75, &tem_points);
      lane_marker_points.at("right").swap(tem_points);
    }
  }
  // 2.generator lane path from lanemarker
  std::list<NaviPathTuple> navigation_path_list;
  TL::common::Status out_status =
      navigation_lane_path_generator_->ConvertLaneMarkerToPath(
          lane_markers, &lane_marker_points, &navigation_path_list,
          lanemarker_new_decider_->GetLaneWidth());
  if (!out_status.ok()) {
    ADEBUG << "Failed to generate a navigation path";
    LogErrorStatus(
        current_map_msg_.get(),
        out_status.error_message() + ", Failed to generate a navigation path.");
    lanemarkerdebug_->set_lane_path_generator_time(Clock::NowInMicroseconds() -
                                                   new_decider_time);
    lanemarkerdebug_->mutable_header()->mutable_status()->CopyFrom(
        current_map_msg_->header().status());
    return false;
  }
  auto lane_path_generator_time = Clock::NowInMicroseconds();
  lanemarkerdebug_->set_lane_path_generator_time(lane_path_generator_time -
                                                 new_decider_time);

  // 3.create map proto from navigation_path
  if (to_fct->has_nnp_fct_out() &&
      to_fct->nnp_fct_out().has_curr_lane_spd_km()) {
    navigation_lane_map_creator_->SetLaneSpeedLimit(
        to_fct->nnp_fct_out().curr_lane_spd_km() / 3.6);
  }
  if (to_fct->has_nnp_fct_out() &&
      to_fct->nnp_fct_out().has_change_lane_types()) {
    navigation_lane_map_creator_->SetChangeLaneType(
        to_fct->nnp_fct_out().change_lane_types());
  }
  navigation_lane_map_creator_->SetAdcIsInTunnel(
      to_fct->has_nnp_fct_out()
          ? to_fct->nnp_fct_out().perception_adc_is_in_tunnel()
          : false);
  auto lane_map_creator_start_time = Clock::NowInMicroseconds();
  if (!navigation_lane_map_creator_->CreateMap(navigation_path_list,
                                               current_map_msg_.get())) {
    LogErrorStatus(current_map_msg_.get(),
                   "Failed to create map from current navigation path.");
    lanemarkerdebug_->set_lane_map_creator_time(Clock::NowInMicroseconds() -
                                                lane_map_creator_start_time);
    lanemarkerdebug_->mutable_header()->mutable_status()->CopyFrom(
        current_map_msg_->header().status());
    lanemarkerdebug_->mutable_creat_map_time()->CopyFrom(
        navigation_lane_map_creator_->GetCreatMapTime());
    AERROR << "Failed to create map from navigation path.";
    return false;
  }
  lanemarkerdebug_->mutable_change_lane_type_info()->CopyFrom(
      navigation_lane_map_creator_->GetChangeLaneTypeInfo());
  lanemarkerdebug_->set_lane_map_creator_time(Clock::NowInMicroseconds() -
                                              lane_map_creator_start_time);
  lanemarkerdebug_->mutable_creat_map_time()->CopyFrom(
      navigation_lane_map_creator_->GetCreatMapTime());

  ADEBUG << "new decider end ";
  return true;
}

void NavigationHdmap::DefaultLanemarker(LaneMarker* const lane_marker) {
  lane_marker->set_c0_position(0.0);
  lane_marker->set_c1_heading_angle(0.0);
  lane_marker->set_c2_curvature(0.0);
  lane_marker->set_c3_curvature_derivative(0.0);
  lane_marker->set_lane_type(
      TL::hdmap::LaneBoundaryType_Type::LaneBoundaryType_Type_UNKNOWN);
  lane_marker->set_line_seq(0);
  lane_marker->set_longitude_end(0.0);
  lane_marker->set_longitude_start(0.0);
  lane_marker->set_quality(0.0);
  lane_marker->set_view_range(0.0);
}

void NavigationHdmap::DebounceStatus(
    functionmanager::FunctionManagerOut* const to_fct, bool status,
    bool lane_line_status_to_mcu) {
  if (history_perception_sub_state_ == functionmanager::NOLANE_TYPE) {
    status = lane_line_status_to_mcu && status;
  }
  laneline_status_ = laneline_rise_debounce_.DealDebounce(
      status && !vehicle_pos_or_heading_err_);
  to_fct->set_perception_status(laneline_status_);
  to_fct->set_laneline_status(laneline_status_);
  auto soc_04_val = to_fct->soc_2_fct_tbd_u32_04();
  bool lane_status_to_mcu_result =
      lane_status_to_mcu_debounce_.DealDebounce(lane_line_status_to_mcu);
  uint32_t lane_status_out =
      (laneline_status_ && lane_status_to_mcu_result) ? kLaneSuccess : 0;
  to_fct->set_soc_2_fct_tbd_u32_04(soc_04_val | lane_status_out);
}

void NavigationHdmap::Lanemarkers2Points(
    const LaneMarkers& lanemarkers,
    std::unordered_map<std::string, std::vector<Vec2d>>* lane_points_map) {
  lane_points_map->insert(std::make_pair(
      "left", Lanemarker2Points(lanemarkers.front_left_lane_marker())));
  lane_points_map->insert(std::make_pair(
      "right", Lanemarker2Points(lanemarkers.front_right_lane_marker())));
  if (!lanemarkers.front_next_left_lane_marker().empty()) {
    lane_points_map->insert(std::make_pair(
        "next_left",
        Lanemarker2Points(lanemarkers.front_next_left_lane_marker().at(0))));
  }
  if (!lanemarkers.front_next_right_lane_marker().empty()) {
    lane_points_map->insert(std::make_pair(
        "next_right",
        Lanemarker2Points(lanemarkers.front_next_right_lane_marker().at(0))));
  }
}

std::vector<Vec2d> NavigationHdmap::Lanemarker2Points(  // NOLINT
    const LaneMarker& lanemarker) {
  std::vector<Vec2d> output_points;
  for (const auto& point : lanemarker.points_vehicle_coord()) {
    output_points.emplace_back(point.x(), point.y());
  }
  return output_points;
}

bool NavigationHdmap::DoProjection_test(const std::vector<Vec2d>& ref_v,
                                        const double& width,
                                        std::vector<Vec2d>* const results) {
  int max_length = static_cast<int>(ref_v.size());
  if (ref_v.empty() || (max_length < 2)) {
    AERROR << "Input ref_v is empty";
    return false;
  }
  const auto refv_begin = ref_v.begin();
  const auto refv_end = ref_v.end();

  // Initialize the projection of the first line segment
  // 找出的垂向量都是朝向中间的。
  std::vector<Vec2d>::const_iterator p0 = refv_begin;
  auto p1 = (p0 + 1);
  Vec2d project_0;
  bool is_perpendicular = FindPerpendicular_test(*p0, *p1, width, &project_0);
  if (!is_perpendicular) {
    AERROR << "Cannot Find Perpendicular Line";
    return false;
  }

  Vec2d a = project_0 + *p0;
  Vec2d b = project_0 + *p1;
  results->push_back(a);
  results->push_back(b);
  for (auto i = refv_begin + 1; (i + 1) != refv_end; i++) {
    // AERROR << "Before p2 " << index;
    // AERROR << "max_length" << max_length;
    const auto p2 = (i + 1);
    p1 = p2 - 1;
    p0 = p1 - 1;
    // AERROR << "DoProjection step two p2-p1";
    // AERROR << p2->x() << "," << p2->y() << "."<< p1->x() << "," <<
    // p1->y();
    const Vec2d v2_p2p1 = *p2 - *p1;
    const Vec2d v1_p1p0 = *p1 - *p0;
    const double dot_v2v1 = v2_p2p1.InnerProd(v1_p1p0);

    const double cross_v2v1 = v2_p2p1.CrossProd(v1_p1p0);
    // 外积很小，说明两个向量平行，则可以直接使用上一个点的垂向量。
    if (cross_v2v1 <= 1e-08) {
      // these two line segments are colinear, sharing the same projection
      b = project_0 + *p2;
      results->push_back(b);
      // AERROR << "DoProjection step continue";
      continue;
    }
    Vec2d project_1;
    // 找出中间点与下一个点的垂向量
    is_perpendicular = FindPerpendicular_test(*p1, *p2, width, &project_1);
    if (!is_perpendicular) {
      AERROR << "Cannot Find Perpendicular Line";
      return false;
    }
    const Vec2d project_2 = Vec2d(0, 0) - project_1;
    const double dot_pro1pro0 = project_1.InnerProd(project_0);
    Vec2d project(0, 0);
    // 内积为零，两向量垂直
    if (abs(dot_v2v1) <= 1e-08) {
      // two line segments are perpendicular
      project = project_1.InnerProd(v1_p1p0) > 0 ? project_1 : project_2;
      project =
          project_0.InnerProd(v2_p2p1) < 0 ? project : Vec2d(0, 0) - project;
    } else {
      // general siutation: two line segements are neither colinear, nor
      // perpendicular
      project = dot_pro1pro0 * dot_v2v1 > 0 ? project_1 : project_2;
    }
    a = project + *p1;
    b = project + *p2;
    // found projection vector, start looking for intersection
    const Vec2d& res_back = results->back();
    const double x_a = res_back.x();
    const double y_a = res_back.y();
    const double x_b = a.x();
    const double y_b = a.y();
    const double x_c = (*p1).x();
    const double y_c = (*p1).y();

    const Vec2d a_b = (a - res_back);
    if (a_b.Length() <= 1e-08) {
      // two line segments are colinear, sharing the same projection
      b = project_0 + (*p2);
      results->push_back(b);
      continue;
    }
    const double d = width;
    const double m =
        (d * d + (x_a * x_a + y_a * y_a) - (x_c * x_c + y_c * y_c));
    const double n = ((x_b * x_b + y_b * y_b) - (x_a * x_a + y_a * y_a));
    const double a =
        2 * (x_a - x_c) * (y_a - y_b) - 2 * (x_b - x_a) * (y_c - y_a);

    const double y = (m * (x_b - x_a) - n * (x_a - x_c)) / a;
    const double x = (m * (y_a - y_b) - n * (y_c - y_a)) / a;

    // the projection does not make sense if the intersection falls on the
    // reversed extended line
    auto i_pro_now = results->end() - 1;
    const auto p0_1 = (i_pro_now - 1);
    const Vec2d p1_reset(x, y);
    const Vec2d v_p1reset_p0 = p1_reset - *p0_1;
    const double direction_p0p1 = v_p1reset_p0.InnerProd(v1_p1p0);
    if (direction_p0p1 < 0) {
      // Cannot Projection
      // Reduce width
      // Or Resize Polygon
      AERROR << "Cannot sweep, adjust width or check points";
      return false;
    }
    (*i_pro_now).set_x(x);
    (*i_pro_now).set_y(y);
    results->push_back(b);
    project_0 = project;
  }

  return true;
}

bool NavigationHdmap::FindPerpendicular_test(const Vec2d& p0,  // NOLINT
                                             const Vec2d& p1,  // NOLINT
                                             const double& distance,
                                             Vec2d* const res) {
  const double x0 = p0.x();
  const double y0 = p0.y();
  const double x1 = p1.x();
  const double y1 = p1.y();
  const double d = distance;

  double x(0.0);
  x = d * d * (y0 - y1) * (y0 - y1);
  x /= ((y0 - y1) * (y0 - y1) + (x1 - x0) * (x1 - x0));
  x = sqrt(x);

  double y(0.0);
  y = d * d * (x1 - x0) * (x1 - x0);
  y /= ((y0 - y1) * (y0 - y1) + (x1 - x0) * (x1 - x0));
  if ((y0 - y1) * (x1 - x0) > 0) {
    y = sqrt(y);
  } else {
    y = -sqrt(y);
  }

  Vec2d p(x, y);
  const Vec2d p1_p0 = p1 - p0;

  double threshold = abs(p.InnerProd(p1_p0));
  if (threshold > 1e-08) {
    AERROR << "Projecting Fatal Error (No Perpendicular Bisector)";
    AERROR << "Dot Product in Finding Perpendicular = " << threshold;
    return false;
  }

  if (p.CrossProd(p1_p0) * d < 0.0) {
    p.set_x(-p.x());
    p.set_y(-p.y());
  }

  res->set_x(p.x());
  res->set_y(p.y());

  return true;
}

bool NavigationHdmap::LaneLineStatusToMcu(
    const std::shared_ptr<LocalView>& local_view,
    functionmanager::FunctionManagerOut* const to_fct) {
  const double max_heading_err_rad =
      config_.lanemarker_decider_config().max_heading_err_rad();
  const double half_lane_width =
      (local_view->GetLaneMarkers()->front_left_lane_marker().c0_position() -
       local_view->GetLaneMarkers()->front_right_lane_marker().c0_position()) /
      2;
  // 如果上一帧不是视觉车道线模式，说明从其它转态跳转过来，那么必须满足车道线的角度偏差都小于0.15rad，
  // 车辆距离车道中心的距离小于0.6m，才会生成地图，否则车道线质量为bad.
  // 目前是只要功能激活就不会再检查车辆位置和角度条件，后面可能更改为只要上次成功就不检查，这样车辆压线时pilot也能激活
  history_nnp_state_ =
      local_view->GetFunctionManagerIn()->fct_nnp_in().nnp_sysstate();
  auto current_pilot_state =
      local_view->GetFunctionManagerIn()->fct_nnp_in().npilot_state();
  history_perception_sub_state_ = to_fct->perception_sub_state();
  uint32_t k_nnp_steer_torque = 0x20;
  bool steer_torque_override = false;
  if (local_view->HasAdasSomeipFromMCU() &&
      local_view->GetAdasSomeipFromMCU()->adas_someip().size() > 740) {
    steer_torque_override =
        ((local_view->GetAdasSomeipFromMCU()->adas_someip().at(740)) &
         k_nnp_steer_torque) != 0;
  }

  ADEBUG << "steer_torque_override: " << steer_torque_override;
  if (history_pilot_state_ == functionmanager::FctToNnpInput::PILOT_ACTIVE &&
      current_pilot_state == functionmanager::FctToNnpInput::PILOT_SUSPEND &&
      steer_torque_override) {
    steer_torque_to_suspend_ = true;
  }
  if (current_pilot_state != functionmanager::FctToNnpInput::PILOT_SUSPEND) {
    steer_torque_to_suspend_ = false;
  }
  ADEBUG << "steer_torque_to_suspend_: " << steer_torque_to_suspend_;
  bool is_drive_auto =
      history_nnp_state_ == functionmanager::NNPS_ACTIVE ||
      history_nnp_state_ == functionmanager::NNPS_OVERRIDE ||
      history_nnp_state_ == functionmanager::NNPS_LAT_OVERRIDE ||
      history_nnp_state_ == functionmanager::NNPS_LON_OVERRIDE ||
      current_pilot_state == functionmanager::FctToNnpInput::PILOT_ACTIVE ||
      (current_pilot_state == functionmanager::FctToNnpInput::PILOT_SUSPEND &&
       !steer_torque_to_suspend_);
  history_pilot_state_ = current_pilot_state;
  vehicle_pos_or_heading_err_ = false;
  if ((std::fabs(local_view->GetLaneMarkers()
                     ->front_left_lane_marker()
                     .c1_heading_angle()) >= max_heading_err_rad ||
       std::fabs(local_view->GetLaneMarkers()
                     ->front_left_lane_marker()
                     .c1_heading_angle()) >= max_heading_err_rad) &&
      local_view->HasFunctionManagerIn() &&
      local_view->GetFunctionManagerIn()->ta_pilot_mode() ==
          functionmanager::NO_CONTROL) {
    vehicle_pos_or_heading_err_ = true;
  }
  if (!is_drive_auto &&
      (std::fabs(local_view->GetLaneMarkers()
                     ->front_left_lane_marker()
                     .c1_heading_angle()) >= max_heading_err_rad ||
       std::fabs(local_view->GetLaneMarkers()
                     ->front_left_lane_marker()
                     .c1_heading_angle()) >= max_heading_err_rad ||
       fabs(local_view->GetLaneMarkers()
                ->front_left_lane_marker()
                .c0_position() -
            half_lane_width) > 0.6) &&
      (local_view->GetLaneMarkers()->front_left_lane_marker().quality() >=
       0.5) &&
      (local_view->GetLaneMarkers()->front_right_lane_marker().quality() >=
       0.5)) {
    ADEBUG << "bad lane marker.";
    return false;
  }
  return true;
}
}  // namespace planning
}  // namespace TL
