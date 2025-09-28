//  Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.
#include "planning/localview/history_trace_lane_line_state/history_trace_lane_line_state.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "common/configs/vehicle_config_helper.h"
#include "common/math/vec2d.h"
#include "common/util/message_util.h"
#include "planning/common/planning_gflags.h"
#include "planning/math/discretized_points_smoothing/ipopt_pos_optimize_smoother.h"
#include "proto/common/pnc_point.pb.h"
#include "proto/map/map_id.pb.h"
#include "proto/perception/perception_parking_lot.pb.h"

namespace TL::planning {

using TL::common::math::Vec2d;  // NOLINT

bool HistoryTraceLanelineState::Init() {
  return true;
}

bool HistoryTraceLanelineState::HistoryTraceCreatMap(
    const LocalView& local_view, TL::hdmap::Map* const hd_map,
    std::string* const error_msg) {
  if (hd_map == nullptr || error_msg == nullptr) {
    *error_msg = "hd_map || error_msg is null";
    return false;
  }
  if (!local_view.HasValidLocalizationHeader()) {
    *error_msg = "Localization has no header, return false.";
    return false;
  }
  bool use_baidu_mode = false;
#ifdef FOR_BAIDU_SIMULATION
  use_baidu_mode = true;
#endif
  if (FLAGS_enable_history_trace_state_sim) {
    // local simulation
    return ProcessLocalSim(local_view, hd_map, error_msg);
  }
  if (use_baidu_mode) {
    // baidu simulation
    return ProcessBaiduSim(local_view, hd_map, error_msg);
  }
  // real car
  return ProcessRealCar(local_view, hd_map, error_msg);
}

bool HistoryTraceLanelineState::SetRoutingAndRoad(  // NOLINT
    TL::routing::RoutingResponse* const inrouting,
    TL::hdmap::Map* const hd_map) {
  if (inrouting == nullptr || hd_map == nullptr) {
    AERROR << "inrouting || hd_map pointer is null";
    return false;
  }
  // Set road boundary
  int lane_num = hd_map->lane_size();
  ADEBUG << "The history trace Lane number is: " << lane_num;
  hd_map->clear_road();
  auto* road = hd_map->add_road();
  road->mutable_id()->set_id("road_history_trace");
  auto* section = road->add_section();
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
      hd_map->lane(lane_num - 1).right_boundary().curve());
  // Set routing info
  auto* routing_road = inrouting->add_road();
  routing_road->set_id(road->id().id());

  auto* passage = routing_road->add_passage();
  passage->set_can_exit(false);
  passage->set_change_lane_type(routing::ChangeLaneType::FORWARD);
  auto* segment = passage->add_segment();
  segment->set_id(hd_map->lane(0).id().id());
  auto adc_lane_segment_points =
      hd_map->lane(0).central_curve().segment().at(0).line_segment().point();
  common::PointENU start_point = adc_lane_segment_points.at(0);
  int max_index = adc_lane_segment_points.size() - 1;
  common::PointENU end_point = adc_lane_segment_points.at(max_index);
  auto* routing_request = inrouting->mutable_routing_request();
  routing::LaneWaypoint waypoint;
  waypoint.set_id(hd_map->lane(0).id().id());
  waypoint.mutable_pose()->set_x(start_point.x());
  waypoint.mutable_pose()->set_y(start_point.y());
  waypoint.set_s(0.0);
  segment->set_start_s(0.0);
  routing_request->add_waypoint()->CopyFrom(waypoint);
  waypoint.set_s(hd_map->lane(0).length());
  segment->set_end_s(hd_map->lane(0).length());

  waypoint.mutable_pose()->set_x(end_point.x());
  waypoint.mutable_pose()->set_y(end_point.y());
  routing_request->add_waypoint()->CopyFrom(waypoint);

  common::util::FillHeader("history_trace_routing", inrouting);
  return true;
}

Status HistoryTraceLanelineState::BuildLocalView(
    const std::shared_ptr<LocalView>& local_view) {
  if (!tba_map_ptr_) {
    TL::navigation_hdmap::MapMsg history_trace_hdmap;
    std::string error_msg;
    if (!HistoryTraceCreatMap(*local_view, history_trace_hdmap.mutable_hdmap(),
                              &error_msg)) {
      ADEBUG << error_msg;
      return Status(ErrorCode::LOCALVIEW_FSM_HISTORYTRACE_ERROR, error_msg);
    }
    common::util::FillHeader("history_trace_map", &history_trace_hdmap);
    common::util::FillHeader(
        "history_trace_map",
        history_trace_hdmap.mutable_hdmap()->mutable_header());
    tba_map_ptr_ =
        std::make_shared<TL::navigation_hdmap::MapMsg>(history_trace_hdmap);

    if (!SetRoutingAndRoad(&routing_response_,
                           history_trace_hdmap.mutable_hdmap())) {
      const std::string msg = {"build history trace lane response fail "};
      ADEBUG << msg;
      return Status(ErrorCode::LOCALVIEW_FSM_HISTORYTRACE_ERROR, msg);
    }
  }

  local_view->SetRoutingResponsePtr(
      std::make_shared<routing::RoutingResponse>(routing_response_));
  local_view->SetMapMsgPtr(tba_map_ptr_);
  return Status::OK();
}

bool HistoryTraceLanelineState::IsLaneForPoint(
    const std::vector<hdmap::LaneInfoConstPtr>& lanes, const Vec2d& point,
    hdmap::LaneInfoConstPtr* const nearest_lane, double* const nearest_s,
    double* const nearest_l) {
  if (nearest_lane == nullptr) {
    return false;
  }
  if (nearest_s == nullptr) {
    return false;
  }
  if (nearest_l == nullptr) {
    return false;
  }
  double min_distance = 2;
  bool has_nearest_lane = false;
  double s_in_lane = 0.0;
  double l_in_lane = 0.0;

  for (const auto& lane : lanes) {
    ADEBUG << " pnc map lane id: " << lane->id().id();
    if (!lane->GetProjection(point, &s_in_lane, &l_in_lane)) {
      AERROR << "fail to get projection";
      continue;
    }
    ADEBUG << "s: " << s_in_lane << " l: " << l_in_lane;

    // Use large epsilonmin_distance to allow projection diff
    constexpr double kEpsilon = 0.2;
    if (s_in_lane > (lane->total_length() + kEpsilon) ||
        (s_in_lane + kEpsilon) < 0.0) {
      ADEBUG << "s " << s_in_lane << " lane " << lane->total_length();
      continue;
    }
    const auto distance = fabs(l_in_lane);
    if (distance < min_distance) {
      min_distance = distance;
      *nearest_lane = lane;
      *nearest_s = s_in_lane;
      *nearest_l = l_in_lane;
      has_nearest_lane = true;
    }
  }
  return has_nearest_lane;
}

bool HistoryTraceLanelineState::GenerateTracedPathFromBaiduMap(
    const std::shared_ptr<hdmap::HDMap>& map_ptr,
    ParkingPathPointSetPro* const traced_paths, std::string* const error_msg) {
  routing::POI poi;
  if (error_msg == nullptr) {
    AERROR << "error_msg is nullptr!";
    return false;
  }
  if (traced_paths == nullptr) {
    *error_msg = "traced_paths is nullptr!";
    return false;
  }
  if (!TL::common::GetProtoFromASCIIFile(hdmap::EndWayPointFile(), &poi)) {
    *error_msg = "can not get EndWayPointFile!";
    return false;
  }
  if (poi.landmark_size() != 1 || poi.landmark(0).waypoint_size() != 2) {
    *error_msg = "EndWayPointFile not valid!";
    return false;
  }
  const auto& start_pose = poi.landmark(0).waypoint(1).pose();
  const auto& end_pose = poi.landmark(0).waypoint(0).pose();
  const Vec2d start_point{start_pose.x(), start_pose.y()};
  const Vec2d end_point{end_pose.x(), end_pose.y()};
  ADEBUG << "start p x: " << start_point.x() << "start p y: " << start_point.y()
         << "end p x: " << end_point.x() << "end p y: " << end_point.y();
  constexpr double kLaneDistance = 2;
  std::vector<hdmap::LaneInfoConstPtr> start_lanes;
  std::vector<hdmap::LaneInfoConstPtr> end_lanes;
  hdmap::LaneInfoConstPtr start_lane;
  hdmap::LaneInfoConstPtr end_lane;
  double start_lane_s = 0.0;
  double start_lane_l = 0.0;
  double end_lane_s = 0.0;
  double end_lane_l = 0.0;
  if (map_ptr->GetLanes(end_pose, kLaneDistance, &end_lanes) != 0) {
    *error_msg = "can not get lanes around end point within 2 meters!";
    return false;
  }
  if (!IsLaneForPoint(end_lanes, end_point, &end_lane, &end_lane_s,
                      &end_lane_l)) {
    return false;
  }
  if (map_ptr->GetLanes(start_pose, kLaneDistance, &start_lanes) != 0) {
    *error_msg = "can not get lanes around start point within 2 meters!";
    return false;
  }
  if (!IsLaneForPoint(start_lanes, start_point, &start_lane, &start_lane_s,
                      &start_lane_l)) {
    return false;
  }
  std::queue<std::vector<hdmap::Id>> lane_id_queue;
  std::string start_lane_id{start_lane->id().id()};
  ADEBUG << " end size: " << end_lanes.size()
         << " start size: " << start_lanes.size();
  ADEBUG << " end_lane_id: " << end_lane->id().id();
  std::vector<hdmap::Id> target_lane_ids;
  lane_id_queue.push({end_lane->id()});
  while (!lane_id_queue.empty()) {
    const auto cur_id_vec = lane_id_queue.front();
    lane_id_queue.pop();
    const hdmap::Id& cur_lane_id = cur_id_vec.back();
    const hdmap::LaneInfoConstPtr cur_lane = {
        map_ptr->GetLaneById(cur_lane_id)};
    if (cur_lane_id.id() == start_lane_id) {
      target_lane_ids = cur_id_vec;
      break;
    }
    for (int i = 0; i < cur_lane->lane().predecessor_id_size(); ++i) {
      const auto& next_lane_id = cur_lane->lane().predecessor_id().at(i);
      ADEBUG << " predecessor lane id : " << next_lane_id.id();
      hdmap::LaneInfoConstPtr next_lane = {map_ptr->GetLaneById(next_lane_id)};
      std::string str_next_lane_id{next_lane_id.id()};
      std::vector<hdmap::Id> next_id_vec{cur_id_vec};
      next_id_vec.push_back(next_lane_id);
      lane_id_queue.push(next_id_vec);
    }
  }
  ADEBUG << " start_lane_id: " << start_lane->id().id();
  if (target_lane_ids.empty()) {
    *error_msg = "can not find target_lane_ids, return false! ";
    AERROR << *error_msg;
    return false;
  }
  const auto start_p = start_lane->GetSmoothPoint(start_lane_s);
  const auto end_p = end_lane->GetSmoothPoint(end_lane_s);
  auto* p1 = traced_paths->Add();
  p1->set_x(end_p.x());
  p1->set_y(end_p.y());
  for (const auto& id : target_lane_ids) {
    const hdmap::LaneInfoConstPtr cur_lane =
        hdmap::HDMapUtil::BaseMapPtr()->GetLaneById(id);
    for (int i = static_cast<int>(cur_lane->points().size()) - 2; i >= 0; --i) {
      if (cur_lane == end_lane &&
          cur_lane->accumulate_s()[i] > end_lane_s - 1e-6) {
        continue;
      }
      if (cur_lane == start_lane &&
          cur_lane->accumulate_s()[i] < start_lane_s + 1e-6) {
        break;
      }
      auto* cur_point = traced_paths->Add();
      cur_point->set_x(cur_lane->points()[i].x());
      cur_point->set_y(cur_lane->points()[i].y());
      cur_point->set_yaw(cur_lane->headings()[i]);
    }
  }
  if (traced_paths->size() < 2) {
    *error_msg = "traced_paths size less than 2";
    return false;
  }
  auto* p2 = traced_paths->Add();
  p2->set_x(start_p.x());
  p2->set_y(start_p.y());

  p1->set_yaw(traced_paths->at(1).yaw());
  p2->set_yaw(traced_paths->at(traced_paths->size() - 2).yaw());
  return true;
}

bool HistoryTraceLanelineState::GlobalPathGenerator(
    const ParkingPathPointSetPro& traced_path, std::string* const error_msg) {
  ACHECK(error_msg);
  if (traced_path.size() < 2) {
    *error_msg = "traced_path.size is less than 2, return false.";
    return false;
  }
  global_path_.clear();
  const auto start_time = common::Clock::NowInMicroseconds();
  std::vector<std::tuple<double, double, double>> xytheta_vector;
  const double start_extend_length = common::VehicleConfigHelper::GetConfig()
                                         .vehicle_param()
                                         .back_edge_to_center() +
                                     FLAGS_history_trace_path_extend_buffer;
  const auto start_unit_vec =
      common::math::Vec2d::CreateUnitVec2d(traced_path.rbegin()->yaw());
  const auto fake_start_point = common::math::Vec2d(traced_path.rbegin()->x(),
                                                    traced_path.rbegin()->y()) -
                                start_extend_length * start_unit_vec;
  xytheta_vector.emplace_back(fake_start_point.x(), fake_start_point.y(),
                              traced_path.rbegin()->yaw());
  for (auto it = traced_path.rbegin(); it != traced_path.rend(); it++) {
    xytheta_vector.emplace_back(it->x(), it->y(), it->yaw());
  }
  constexpr double kExpandSBuffer = 1.0;
  const double end_extend_length =
      kExpandSBuffer + common::VehicleConfigHelper::GetConfig()
                           .vehicle_param()
                           .front_edge_to_center();
  const auto end_unit_vec =
      common::math::Vec2d::CreateUnitVec2d(traced_path.begin()->yaw());
  const auto fake_end_point =
      common::math::Vec2d(traced_path.begin()->x(), traced_path.begin()->y()) +
      end_extend_length * end_unit_vec;
  xytheta_vector.emplace_back(fake_end_point.x(), fake_end_point.y(),
                              traced_path.begin()->yaw());

  std::pair<double, bool> init_kappa_constrain(0.0, true);
  IpoptPosOptimizeSmoother smoother =
      IpoptPosOptimizeSmoother(smoother_config_);
  std::vector<common::PathPoint> rough_path;
  if (!IpoptPosOptimizeSmoother::RoughPathProcessor(
          init_kappa_constrain, xytheta_vector, true, &rough_path)) {
    *error_msg = "IpoptPosOptimizeSmoother::RoughPathProcessor failed.";
    return false;
  }
  global_path_ = std::move(rough_path);

  AINFO << "IpoptPosOptimizeSmoother time(ms): "
        << common::Clock::NowInMicroseconds() - start_time;

  ADEBUG << "global_path.size(): " << global_path_.size();
  for (const auto& point : global_path_) {
    ADEBUG << "s: " << point.s() << ", x: " << point.x() << ", y: " << point.y()
           << ", theta: " << point.theta() << ", kappa: " << point.kappa();
  }
  return true;
}

bool HistoryTraceLanelineState::ConstructLane(const LocalView& local_view,
                                              TL::hdmap::Map* const hd_map,
                                              std::string* const error_msg) {
  if (hd_map == nullptr) {
    AERROR << "hd_map is nullptr!";
    return false;
  }
  if (error_msg == nullptr) {
    AERROR << "error_msg is nullptr!";
    return false;
  }
  if (global_path_.size() < 2) {
    *error_msg = "size of forward path is less than 2, return false.";
    return false;
  }

  std::vector<TL::common::PointENU> central_point;
  std::vector<TL::common::PointENU> left_boundary_point;
  std::vector<TL::common::PointENU> right_boundary_point;
  std::vector<double> sample_s;
  const double half_lane_width = 3.0;
  for (const auto& cur_point : global_path_) {
    sample_s.emplace_back(cur_point.s());

    TL::common::PointENU central;
    central.set_x(cur_point.x());
    central.set_y(cur_point.y());
    central.set_z(cur_point.z());
    central_point.push_back(std::move(central));

    TL::common::PointENU left_boundary;
    left_boundary.set_x(half_lane_width * cos(cur_point.theta() + M_PI_2) +
                        cur_point.x());
    left_boundary.set_y(half_lane_width * sin(cur_point.theta() + M_PI_2) +
                        cur_point.y());
    left_boundary.set_z(cur_point.z());
    left_boundary_point.push_back(std::move(left_boundary));

    TL::common::PointENU right_boundary;
    right_boundary.set_x(half_lane_width * cos(cur_point.theta() - M_PI_2) +
                         cur_point.x());
    right_boundary.set_y(half_lane_width * sin(cur_point.theta() - M_PI_2) +
                         cur_point.y());
    right_boundary.set_z(cur_point.z());
    right_boundary_point.push_back(std::move(right_boundary));
  }
  if (central_point.size() < 2) {
    *error_msg = "size of central_point is zero, return false.";
    return false;
  }

  const auto& local_position = local_view.GetLocalization();
  hd_map->clear_lane();
  auto* lane = hd_map->add_lane();
  lane->mutable_id()->set_id(absl::StrCat("history_trace_lane"));
  lane->set_type(hdmap::Lane::CITY_DRIVING);
  lane->set_turn(hdmap::Lane::NO_TURN);
  lane->set_speed_limit(3.0);

  // central line
  auto* curve_segment = lane->mutable_central_curve()->add_segment();
  curve_segment->set_heading(local_position->pose().heading());
  curve_segment->set_length(sample_s.back());
  lane->set_length(sample_s.back());
  auto* line_segment = curve_segment->mutable_line_segment();

  // left boundary
  auto* left_boundary = lane->mutable_left_boundary();
  auto* left_boundary_type = left_boundary->add_boundary_type();
  left_boundary->set_virtual_(false);
  left_boundary_type->set_s(0.0);
  left_boundary_type->add_types(
      TL::hdmap::LaneBoundaryType_Type::LaneBoundaryType_Type_DOTTED_WHITE);
  auto* left_segment =
      left_boundary->mutable_curve()->add_segment()->mutable_line_segment();

  // right boundary
  auto* right_boundary = lane->mutable_right_boundary();
  auto* right_boundary_type = right_boundary->add_boundary_type();
  right_boundary->set_virtual_(false);
  right_boundary_type->set_s(0.0);
  right_boundary_type->add_types(
      TL::hdmap::LaneBoundaryType_Type::LaneBoundaryType_Type_DOTTED_WHITE);
  auto* right_segment =
      right_boundary->mutable_curve()->add_segment()->mutable_line_segment();

  for (size_t i = 0; i < central_point.size(); i++) {
    line_segment->add_point()->CopyFrom(central_point.at(i));
    left_segment->add_point()->CopyFrom(left_boundary_point.at(i));
    right_segment->add_point()->CopyFrom(right_boundary_point.at(i));

    auto* right_sample = lane->add_right_sample();
    right_sample->set_s(sample_s.at(i));
    right_sample->set_width(half_lane_width);

    auto* left_sample = lane->add_left_sample();
    left_sample->set_s(sample_s.at(i));
    left_sample->set_width(half_lane_width);
  }
  return true;
}

bool HistoryTraceLanelineState::ProcessRealCar(const LocalView& local_view,
                                               TL::hdmap::Map* const hd_map,
                                               std::string* const error_msg) {
  if (error_msg == nullptr || hd_map == nullptr) {
    AERROR << "error_msg || hd_map is nullptr!";
    return false;
  }
  ParkingPathPointSetPro traced_path;
  if (!local_view.HasValidParkingLotOutArrayHeader()) {
    *error_msg =
        "ProcessRealCar ParkingLotOutArray has no header, return false.";
    return false;
  }
  traced_path = local_view.GetParkingLotOutArray()->traced_path();
  if (!GlobalPathGenerator(traced_path, error_msg)) {
    return false;
  }
  return ConstructLane(local_view, hd_map, error_msg);
}

bool HistoryTraceLanelineState::ProcessBaiduSim(const LocalView& local_view,
                                                TL::hdmap::Map* const hd_map,
                                                std::string* const error_msg) {
  if (error_msg == nullptr || hd_map == nullptr) {
    AERROR << "error_msg || hd_map is nullptr!";
    return false;
  }
  ParkingPathPointSetPro traced_path;
  if (!local_view.HasValidParkingLotOutArrayHeader()) {
    *error_msg =
        "ProcessBaiduSim ParkingLotOutArray has no header, return false.";
    return false;
  }

  if (local_view.GetParkingLotOutArray()->traced_path().size() < 2) {
    // world sim
    if (!GenerateTracedPathFromBaiduMap(hdmap::HDMapUtil::BaseMapPtr(),
                                        &traced_path, error_msg)) {
      return false;
    }
  } else {
    // log sim
    traced_path = local_view.GetParkingLotOutArray()->traced_path();
  }

  if (!GlobalPathGenerator(traced_path, error_msg)) {
    return false;
  }
  return ConstructLane(local_view, hd_map, error_msg);
}

bool HistoryTraceLanelineState::ProcessLocalSim(const LocalView& local_view,
                                                TL::hdmap::Map* const hd_map,
                                                std::string* const error_msg) {
  if (error_msg == nullptr || hd_map == nullptr) {
    AERROR << "error_msg || hd_map is nullptr!";
    return false;
  }
  perception::ParkingLotOutArray ploa;
  if (!common::GetProtoFromFile(FLAGS_valet_parking_history_trace_config_file,
                                &ploa)) {
    *error_msg = {"Get ParkingLotOutArray for file: " +
                  FLAGS_valet_parking_history_trace_config_file + " failed."};
    return false;
  }
  if (!GlobalPathGenerator(ploa.traced_path(), error_msg)) {
    return false;
  }
  return ConstructLane(local_view, hd_map, error_msg);
}

}  // namespace TL::planning
