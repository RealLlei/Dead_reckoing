/*
 * Copyright (c) TL auto Co., Ltd. 2023-2024. All rights reserved.
 */
#include "planning/localview/lane_line_builder/obstacle_following_lane_line/missile_mode/missile_mode_lane_line.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>
#include "common/math/vec2d.h"
#include "common/status/status.h"
#include "common/util/util.h"
#include "planning/math/discrete_points_math.h"
#include "planning/pnc_map/pnc_map.h"

namespace TL {
namespace planning {
namespace missilelane {
using TL::common::ErrorCode;
using TL::common::Status;
using TL::common::util::operator+;  // NOLINT

MissileMode::MissileMode() : nolane_rise_debounce_{0.5, 0.0, 0.1} {}

Status MissileMode::Init(
    const std::shared_ptr<LocalViewData>& local_view_data) {
  config_.Clear();
  if (!common::GetProtoFromFile(FLAGS_navigation_hdmap_config_filename,
                                &config_)) {
    return Status(ErrorCode::LOCALVIEW_MAP_LANE_ERROR,
                  "Unable to load relative map conf file: " +
                      FLAGS_navigation_hdmap_config_filename);
  }
  missile_vehicle_state_ = std::make_shared<MissileVehicleState>();
  obs_decider_ = std::make_shared<ObstacleDecider>();
  obstacles_state_ =
      std::make_shared<ObstaclesState>(config_, missile_vehicle_state_);
  obstacles_state_->Init();
  missile_vehicle_state_->Init(local_view_data);
  obs_decider_->Init(obstacles_state_);
  current_map_msg_ = std::make_shared<navigation_hdmap::MapMsg>();
  current_routing_response_ = std::make_shared<routing::RoutingResponse>();
  points_filter_ =
      std::make_shared<PointsFilter>(config_, missile_vehicle_state_);
  points_filter_->Init();
  local_view_data_ = local_view_data;
  return Status::OK();
}

Status MissileMode::Init() {
  return Status::OK();
}

Status MissileMode::Start() {
  return Status::OK();
}

void MissileMode::Stop() {}

bool MissileMode::Process(const std::shared_ptr<LocalView>& local_view,
                          functionmanager::FunctionManagerOut* to_fct) {
  ADEBUG << "----------------start missile mode!!!------------------------";
  if (current_map_msg_.use_count() != 1) {
    current_map_msg_ = std::make_shared<navigation_hdmap::MapMsg>();
  } else {
    current_map_msg_->Clear();
  }
  common::util::FillHeader("nolane_hdmap", current_map_msg_.get());
  auto* hd_map = current_map_msg_->mutable_hdmap();
  common::util::FillHeader("from_nolane_hdmap", hd_map->mutable_header());
  auto* lane = hd_map->add_lane();
  // 1.更新车辆状态
  if (!missile_vehicle_state_->Update(local_view, *to_fct)) {
    ADEBUG << "-----HAS NO vehicle state!!!-----";
    obstacles_state_->ClearObsInfo();
    return DebounceStatus(to_fct, false);
  }
  for (const auto& id : *local_view_data_->get_cruise_target_ids()) {
    ADEBUG << "target_ids: " << id;
  }
  if (!local_view->HasPerceptionObstacles() ||
      local_view->GetPerceptionObstacles() == nullptr) {
    ADEBUG << "-----HAS NO OBS ID or OBS!!!-----";
    obstacles_state_->ClearObsInfo();
    return DebounceStatus(to_fct, false);
  }
  // 2.更新目标障碍物信息
  obstacles_state_->Update(local_view->GetPerceptionObstacles(),
                           *local_view_data_->get_cruise_target_ids());
  ADEBUG << "obs id size: " << local_view_data_->get_cruise_target_ids()->size()
         << " , obs_id: " << local_view_data_->get_cruise_target_ids()->front();

  // 3.计算跟车目标轨迹点
  if (!obs_decider_->UpdateFollowingObs(local_view, to_fct)) {
    ADEBUG << "UpdateFollowingObs ERR!!!";
    return DebounceStatus(to_fct, false);
  }
  // if (!obs_decider_->FollowingObs().has_id()) {
  //   return DebounceStatus(to_fct, false);
  // }
  // TL::perception::LaneMarker lane_marker;
  // GenerateLaneMarker(&lane_marker);

  // 4.使用跟车目标轨迹点生成path
  common::Path path;
  std::vector<Vec2d> points = obs_decider_->GetObsPoints();
  // auto points = points_filter_->Filter(obs_decider_->GetObsLanemarker());
  bool is_generate_path = GeneratePath(&points, &path);
  // bool is_generate_path{true};
  // obs_decider_->GeneratePoints(&path);
  if (!is_generate_path || (!CheckerLanemarkers(local_view) &&
                            !missile_vehicle_state_->is_pilot_active())) {
    ADEBUG << "is_generate_path" << is_generate_path;
    return DebounceStatus(to_fct, false);
  }

  // 5.生成地图以及构建routing
  const bool is_generate_map = GenerateMap(path, lane, hd_map);
  if (current_map_msg_->hdmap().lane_size() > 0 && is_generate_map) {
    ADEBUG << "success!!!"
           << ", lane size: " << current_map_msg_->hdmap().lane_size();
    current_routing_response_ =
        std::make_shared<routing::RoutingResponse>(current_map_msg_->routing());
    return DebounceStatus(to_fct, true);
  }
  ADEBUG << "missile mode filed!!!";
  return DebounceStatus(to_fct, false);
}

bool MissileMode::DebounceStatus(
    functionmanager::FunctionManagerOut* const to_fct, const bool status) {
  auto nolane_status = nolane_rise_debounce_.DealDebounce(status);
  // &&missile_vehicle_state_->is_pilot_active();
  bool perception_status = to_fct->perception_status();
  to_fct->set_perception_status(nolane_status || perception_status);
  to_fct->set_nolane_status(nolane_status);
  ADEBUG << "nolane_status: " << nolane_status;
  auto soc_04_val = to_fct->soc_2_fct_tbd_u32_04();
  uint32_t nolane_status_out = nolane_status ? kLaneSuccess : 0;
  to_fct->set_soc_2_fct_tbd_u32_04(soc_04_val | nolane_status_out);
  return nolane_status;
}

bool MissileMode::GeneratePath(std::vector<Vec2d>* points, common::Path* path) {
  // AERROR << "ConvertVec2Path!!!";
  return Addpoints(points) && ConvertVec2Path(*points, path);
}

bool MissileMode::GenerateMap(const common::Path& path, hdmap::Lane* lane,
                              TL::hdmap::Map* hdmap) {
  double extension_lane_width =
      (obs_decider_ == nullptr) ? 3.2 : (obs_decider_->half_lane_width() * 2.0);
  extension_lane_width = config_.missile_mode_default_lane_width();
  // AERROR << "GenerateOneLane!!!";
  bool is_generate = GenerateOneLane(path, lane, extension_lane_width);
  auto* routing = current_map_msg_->mutable_routing();
  // AERROR << "hdmap segment size: "
  //        << hdmap->lane(0).central_curve().segment().size()
  //        << " , lane segment size: " << lane->central_curve().segment().size();
  bool is_setting = SetRouting(routing, hdmap);
  return is_generate && is_setting;
}

bool MissileMode::Addpoints(std::vector<Vec2d>* points) {
  if (points == nullptr || points->size() < 3) {
    return false;
  }

  const double central_length =
      FLAGS_buffer_gainst_lookford_distance +
      hdmap::PncMap::LookForwardDistance(
          functionmanager::MachineStateType::PERCEPTION_TYPE,
          config_.default_speed_limit(), config_.default_max_cruise_speed());
  const double back_length = 5.0;
  size_t start_index = 0;
  Vec2d front_delta_point(
      (*points)[points->size() - 1].x() - (*points)[points->size() - 2].x(),
      (*points)[points->size() - 1].y() - (*points)[points->size() - 2].y());
  front_delta_point.Normalize();
  Vec2d back_delta_point((*points)[1].x() - (*points)[0].x(),
                         (*points)[1].y() - (*points)[0].y());
  back_delta_point.Normalize();
  // double x = back_delta_point.x();
  // double y = back_delta_point.y();
  // AERROR << "back_delta_point_x_y:" << x << " , " << y;
  for (size_t i = 0; i < points->size(); i++) {
    if ((*points)[i].x() > 0) {
      start_index = i;
      // AERROR << "start_index" << start_index;
      break;
    }
  }
  double now_back_length = 0;
  for (size_t i = 1; i <= start_index; i++) {
    now_back_length += (*points)[i - 1].DistanceTo((*points)[i]);
  }
  double now_front_length = 0;
  for (size_t i = start_index; i < points->size() - 1; i++) {
    now_front_length += (*points)[i].DistanceTo((*points)[i + 1]);
  }

  while (now_back_length < back_length) {
    points->insert(points->begin(),
                   Vec2d{points->front().x() - back_delta_point.x(),
                         points->front().y() - back_delta_point.y()});
    now_back_length += 1;
  }
  while (now_front_length < (central_length - back_length)) {
    points->emplace_back(Vec2d{points->back().x() + front_delta_point.x(),
                               points->back().y() + front_delta_point.y()});
    now_front_length += 1;
  }
  return true;
}

bool MissileMode::ConvertVec2Path(const std::vector<Vec2d>& points,
                                  common::Path* path) {
  const auto& vehicle_state = missile_vehicle_state_->vehicle_state();
  double accumulated_s = 0.0;
  for (size_t i = 1; i < points.size(); ++i) {
    if (i == 1) {
      auto* point = path->add_path_point();
      Eigen::Vector2d enu_coordinate = common::math::RotateVector2d(
          {points[i - 1].x(), points[i - 1].y()}, vehicle_state->heading());
      double x_enu = enu_coordinate.x() + vehicle_state->x();
      double y_enu = enu_coordinate.y() + vehicle_state->y();
      point->set_x(x_enu);
      point->set_y(y_enu);
      point->set_theta(vehicle_state->heading());
      point->set_s(accumulated_s);
    }
    double x1 = points[i].x();
    double y1 = points[i].y();
    auto* point = path->add_path_point();
    Vec2d vec = points[i] - points[i - 1];
    double theta = std::atan2(vec.y(), vec.x());
    Eigen::Vector2d enu_coordinate =
        common::math::RotateVector2d({x1, y1}, vehicle_state->heading());
    double x_enu = enu_coordinate.x() + vehicle_state->x();
    double y_enu = enu_coordinate.y() + vehicle_state->y();
    double theta_enu = common::math::NormalizeAngle(
        common::math::NormalizeAngle(theta) + vehicle_state->heading());
    if (path->path_point_size() > 1) {
      const auto& pre_point = path->path_point(path->path_point_size() - 2);
      accumulated_s += std::hypot(x_enu - pre_point.x(), y_enu - pre_point.y());
    }
    point->set_x(x_enu);
    point->set_y(y_enu);
    point->set_theta(theta_enu);
    point->set_s(accumulated_s);
  }
  return true;
}

bool MissileMode::GenerateOneLane(const common::Path& path, hdmap::Lane* lane,
                                  double extension_one_lane_width) {
  static uint8_t lane_id_count{0};
  lane_id_count++;
  // ADEBUG << "path_point_size " << path.path_point_size();
  if (path.path_point_size() < 2) {
    // AERROR << "The path length of line index is invalid";
    return false;
  }
  // AERROR << " path_point_size: " << path.path_point_size();

  lane->mutable_id()->set_id(absl::StrCat("0_current_", lane_id_count));

  // lane types
  lane->set_type(hdmap::Lane::CITY_DRIVING);
  lane->set_turn(hdmap::Lane::NO_TURN);

  // speed limit
  lane->set_speed_limit(config_.default_speed_limit());

  // center line
  auto* curve_segment = lane->mutable_central_curve()->add_segment();
  curve_segment->set_heading(path.path_point(0).theta());
  curve_segment->set_length(path.path_point(path.path_point_size() - 1).s());
  lane->set_length(path.path_point(path.path_point_size() - 1).s());
  auto* line_segment = curve_segment->mutable_line_segment();

  // left boundary
  auto* left_boundary = lane->mutable_left_boundary();
  auto* left_boundary_type = left_boundary->add_boundary_type();
  left_boundary->set_virtual_(false);
  left_boundary_type->set_s(0.0);
  left_boundary_type->add_types(hdmap::LaneBoundaryType::DOTTED_WHITE);
  auto* left_segment =
      left_boundary->mutable_curve()->add_segment()->mutable_line_segment();

  // right boundary
  auto* right_boundary = lane->mutable_right_boundary();
  auto* right_boundary_type = right_boundary->add_boundary_type();
  right_boundary->set_virtual_(false);
  right_boundary_type->set_s(0.0);
  right_boundary_type->add_types(hdmap::LaneBoundaryType::DOTTED_WHITE);
  auto* right_segment =
      right_boundary->mutable_curve()->add_segment()->mutable_line_segment();

  const double half_lane_width = extension_one_lane_width * 0.5;

  for (int i = 0; i < path.path_point_size(); ++i) {
    auto* point = line_segment->add_point();
    point->set_x(path.path_point(i).x());
    point->set_y(path.path_point(i).y());
    point->set_z(path.path_point(i).z());
    auto* left_sample = lane->add_left_sample();
    left_sample->set_s(path.path_point(i).s());
    left_sample->set_width(half_lane_width);
    left_segment->add_point()->CopyFrom(
        *point + half_lane_width * Vec2d::CreateUnitVec2d(
                                       path.path_point(i).theta() + M_PI_2));

    auto* right_sample = lane->add_right_sample();
    right_sample->set_s(path.path_point(i).s());
    right_sample->set_width(half_lane_width);
    right_segment->add_point()->CopyFrom(
        *point + half_lane_width * Vec2d::CreateUnitVec2d(
                                       path.path_point(i).theta() - M_PI_2));
  }
  // AERROR << "iner lane segment size: "
  //        << lane->central_curve().segment().size();
  return true;
}

bool MissileMode::SetRouting(TL::routing::RoutingResponse* inrouting,
                             TL::hdmap::Map* hd_map) {
  if (hd_map->lane(0).central_curve().segment().empty()) {
    return false;
  }
  // Set road boundary
  int lane_num = hd_map->lane_size();
  auto* road = hd_map->add_road();
  road->mutable_id()->set_id("road_nolane");
  auto* section = road->add_section();
  ADEBUG << "lane_num: " << lane_num;
  for (int i = 0; i < lane_num; ++i) {
    auto* lane_id = section->add_lane_id();
    lane_id->CopyFrom(hd_map->lane(i).id());
  }
  auto* outer_polygon = section->mutable_boundary()->mutable_outer_polygon();
  auto* left_edge = outer_polygon->add_edge();
  left_edge->set_type(TL::hdmap::BoundaryEdge::LEFT_BOUNDARY);
  left_edge->mutable_curve()->CopyFrom(hd_map->lane(0).left_boundary().curve());
  auto* right_edge = outer_polygon->add_edge();
  right_edge->set_type(TL::hdmap::BoundaryEdge::RIGHT_BOUNDARY);
  right_edge->mutable_curve()->CopyFrom(
      hd_map->lane(0).right_boundary().curve());
  // Set routing info
  auto* routing_road = inrouting->add_road();
  routing_road->set_id(road->id().id());
  for (int i = 0; i < hd_map->lane_size(); i++) {
    // set passage and routing
    auto* passage = routing_road->add_passage();
    passage->set_can_exit(false);
    passage->set_change_lane_type(routing::ChangeLaneType::FORWARD);
    auto* segment = passage->add_segment();
    segment->set_id(hd_map->lane(i).id().id());
    segment->set_start_s(0.0);
    segment->set_end_s(hd_map->lane(0).length());
    auto segment_tp = hd_map->lane(i).central_curve().segment();
    auto adc_lane_segment_points =
        hd_map->lane(i).central_curve().segment().at(0).line_segment().point();
    common::PointENU start_point = adc_lane_segment_points.at(0);
    int max_index = adc_lane_segment_points.size() - 1;
    common::PointENU end_point = adc_lane_segment_points.at(max_index);
    auto* routing_request = inrouting->mutable_routing_request();
    routing::LaneWaypoint waypoint;
    waypoint.set_id(hd_map->lane(i).id().id());
    waypoint.mutable_pose()->set_x(start_point.x());
    waypoint.mutable_pose()->set_y(start_point.y());
    waypoint.set_s(0.0);
    routing_request->add_waypoint()->CopyFrom(waypoint);
    waypoint.set_s(hd_map->lane(i).length());
    waypoint.mutable_pose()->set_x(end_point.x());
    waypoint.mutable_pose()->set_y(end_point.y());
    routing_request->add_waypoint()->CopyFrom(waypoint);
  }
  auto* routing_request = inrouting->mutable_routing_request();
  common::util::FillHeader("from_nolane_routingrequest", routing_request);
  common::util::FillHeader("from_nolane_routing", inrouting);
  return true;
}

bool MissileMode::CheckerLanemarkers(
    const std::shared_ptr<LocalView>& local_view) {
  bool is_left_lanemarker_ok =
      (local_view->HasLaneMarkers() &&
       local_view->GetLaneMarkers()->has_front_left_lane_marker())
          ? CheckerLanemarker(
                local_view->GetLaneMarkers()->front_left_lane_marker())
          : true;
  bool is_right_lanemarker_ok =
      (local_view->HasLaneMarkers() &&
       local_view->GetLaneMarkers()->has_front_right_lane_marker())
          ? CheckerLanemarker(
                local_view->GetLaneMarkers()->front_right_lane_marker())
          : true;
  return is_left_lanemarker_ok && is_right_lanemarker_ok;
}

bool MissileMode::CheckerLanemarker(
    const TL::perception::LaneMarker& lanemarker) {
  return lanemarker.longitude_start() > 10.0 ||
         lanemarker.longitude_end() < 0.0 ||
         std::fabs(lanemarker.c0_position()) > 1.0;
}

bool MissileMode::GenerateLaneMarker(
    TL::perception::LaneMarker* lanemarker) {
  const auto& follow_obs = obs_decider_->FollowingObs();
  double x0 = follow_obs.position_flu().x();
  double y0 = follow_obs.position_flu().y();
  // double heading =
  //     std::tan(follow_obs.velocity_flu().y() / follow_obs.velocity_flu().x());
  double heading = follow_obs.theta_flu();
  double c1 = std::tan(y0 / x0);
  double c2 = (c1 * x0 + 3 * y0 - 3 * c1 * x0 - heading * x0) / (x0 * x0);
  double c3 = (heading - c1 - 2 * c2 * x0) / (3 * x0 * x0);
  lanemarker->set_c0_position(0.0);
  lanemarker->set_c1_heading_angle(c1);
  lanemarker->set_c2_curvature(c2);
  lanemarker->set_c3_curvature_derivative(c3);
  lanemarker->set_view_range(x0);
  lanemarker->set_longitude_start(0.0);
  lanemarker->set_longitude_end(x0);
  return true;
}

}  // namespace missilelane
}  // namespace planning
}  // namespace TL
