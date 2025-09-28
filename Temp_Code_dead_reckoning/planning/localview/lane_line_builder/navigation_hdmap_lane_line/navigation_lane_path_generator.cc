/*
 * Copyright (c) TL auto Co., Ltd. 2021-2022. All rights reserved.
 */
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/navigation_lane_path_generator.h"

#include <algorithm>
#include <unordered_map>
#include <vector>

#include "absl/strings/match.h"
#include "absl/strings/str_cat.h"
#include "common/status/status.h"
#include "common/time/clock.h"
// #include "common/log.h"
#include "common/math/curve_fitting.h"
#include "common/math/math_utils.h"
#include "common/math/vec2d.h"
#include "common/util/message_util.h"
#include "common/util/points_downsampler.h"
#include "common/util/util.h"
#include "planning/common/planning_gflags.h"

#include "proto/common/types.pb.h"
#include "proto/map/map_lane.pb.h"

// #include "cyber/common/macros.h"

namespace TL {
namespace planning {
using TL::common::Status;
constexpr double kMinNum = 0.0001;

Status NavigationLanePathGenerator::Init() {
  // Low pass filter
  default_left_width_ = config_.default_left_width();
  default_right_width_ = config_.default_right_width();
  current_lane_width_dq_.clear();
  left_lane_width_dq_.clear();
  right_lane_width_dq_.clear();
  current_lane_width_delay_ = lanelineprocess::Delay<double>(3);
  left_lane_width_delay_ = lanelineprocess::Delay<double>(3);
  right_lane_width_delay_ = lanelineprocess::Delay<double>(3);
  navi_central_line_constructor_.Init();
  return Status::OK();
}

NavigationLanePathGenerator::NavigationLanePathGenerator(
    const planning::PerceptionMapConfig& config)
    : config_(config) {}

void NavigationLanePathGenerator::SetVehicleState(
    const std::shared_ptr<const common::VehicleState>& vehicle_state) {
  vehicle_state_ = vehicle_state;
  // vehicle_state_.set_x(0.0);
  // vehicle_state_.set_y(0.0);
  // vehicle_state_.set_z(0.0);
  // vehicle_state_.set_heading(0.0);
  // vehicle_state_.set_yaw(0.0);
}

double NavigationLanePathGenerator::GetKappa(const double c1, const double c2,
                                             const double c3, const double x) {
  const double dy = 3 * c3 * x * x + 2 * c2 * x + c1;
  const double d2y = 6 * c3 * x + 2 * c2;
  return d2y / std::pow((1 + dy * dy), 1.5);
}

Status NavigationLanePathGenerator::ConvertLaneMarkerToPath(
    const perception::LaneMarkers& lane_marker,
    std::unordered_map<std::string, std::vector<Vec2d>>* lane_marker_points,
    std::list<NaviPathTuple>* navigation_path_list, LneWdeValid lane_width) {
  // Generate the centerline of the lane, the vehicle is currently located
  const double current_speed = vehicle_state_->linear_velocity();
  double central_length =
      FLAGS_buffer_gainst_lookford_distance +
      hdmap::PncMap::LookForwardDistance(
          functionmanager::MachineStateType::PERCEPTION_TYPE, current_speed,
          config_.default_max_cruise_speed());
  // 使用车道线预测的宽度
  double default_lane_width =
      config_.default_left_width() + config_.default_right_width();
  double central_lane_width = lane_width.ego_lane_widthpredict_valid == 1
                                  ? lane_width.ego_lane_widthpredict_sg
                                  : default_lane_width;
  double left_lane_width = lane_width.next_leftlane_widthpredict_valid == 1
                               ? lane_width.next_leftlane_widthpredict_sg
                               : default_lane_width;
  double right_lane_width = lane_width.next_rightlane_widthpredict_valid == 1
                                ? lane_width.next_rightlane_widthpredict_sg
                                : default_lane_width;

  ADEBUG << "central_lane_width: " << central_lane_width
         << ", left_lane_width: " << left_lane_width
         << ", right_lane_width: " << right_lane_width;
  //
  double current_lane_width_average = current_lane_width_delay_.GetAverageValue(
      (lane_width.ego_lane_widthpredict_valid == 1 ||
       lane_width.ego_lane_widthpredict_valid == 2)
          ? lane_width.ego_lane_widthpredict_sg
          : default_lane_width);
  double left_lane_width_average = left_lane_width_delay_.GetAverageValue(
      lane_width.next_leftlane_widthpredict_valid == 1
          ? lane_width.next_leftlane_widthpredict_sg
          : default_lane_width);
  UNUSED(left_lane_width_average);
  double right_lane_width_average = right_lane_width_delay_.GetAverageValue(
      lane_width.next_rightlane_widthpredict_valid == 1
          ? lane_width.next_rightlane_widthpredict_sg
          : default_lane_width);
  UNUSED(right_lane_width_average);
  ADEBUG << "current_lane_width_average: " << current_lane_width_average;
  //
  central_line_pts_.clear();
  auto central_navi_path = std::make_shared<navigation_hdmap::NavigationPath>();
  auto* central_path = central_navi_path->mutable_path();
  central_path->set_name("current");
  EndPonitTuple end_point_tuple{};
  Vec2d delta_front_point;
  // ADEBUG << "central_line";
  std::vector<Vec2d> left_boundary_point;
  std::vector<Vec2d> right_boundary_point;
  std::vector<double> central_lane_left_width;
  std::vector<double> central_lane_right_width;
  auto left_boundary_of_centrallane = std::make_shared<hdmap::LineSegment>();
  auto right_boundary_of_centrallane = std::make_shared<hdmap::LineSegment>();
  if ((lane_marker_points->find("left") == lane_marker_points->end()) ||
      (lane_marker_points->find("right") == lane_marker_points->end())) {
    navi_central_line_constructor_.ClearHistoryPoint();
    return Status(common::ErrorCode::LOCALVIEW_MAP_LANE_ERROR,
                  "has no left/right lane_marker_points!");
  }
  auto out_status = navi_central_line_constructor_.BuildReferenceLine(
      central_length, current_lane_width_average,
      lane_marker.front_right_lane_marker(),
      lane_marker.front_left_lane_marker(), &(lane_marker_points->at("left")),
      &(lane_marker_points->at("right")), &central_line_pts_,
      &left_boundary_point, &right_boundary_point, &end_point_tuple,
      &delta_front_point, true);
  if (!out_status.ok()) {
    navi_central_line_constructor_.ClearHistoryPoint();
    ADEBUG << "Central line constructor failed";
    return out_status;
  }
  CalculateLaneWidth(left_boundary_point, right_boundary_point,
                     central_line_pts_, &central_lane_left_width,
                     &central_lane_right_width);
  TranslatedReferenceLine(lane_marker, central_line_pts_, central_path,
                          left_boundary_point, right_boundary_point,
                          left_boundary_of_centrallane.get(),
                          right_boundary_of_centrallane.get());
  central_navi_path->set_path_priority(FLAGS_current_navi_path_priority);  //
  auto central_navi_path_two =
      std::make_shared<navigation_hdmap::NavigationPath>();
  central_navi_path_two->set_path_priority(FLAGS_current_navi_path_priority);
  auto central_lane_type =
      std::make_pair(lane_marker.front_left_lane_marker().lane_type(),
                     lane_marker.front_right_lane_marker().lane_type());
  navigation_path_list->emplace_back(
      2, std::move(central_lane_left_width),
      std::move(central_lane_right_width), central_navi_path,
      left_boundary_of_centrallane, right_boundary_of_centrallane,
      central_lane_type);  // 2

  // 判断车辆是否在地图内
  auto left_it = std::upper_bound(
      left_boundary_point.begin(), left_boundary_point.end(), 0.0,
      [](const int& val, const Vec2d& point) { return point.x() > val; });
  auto right_it = std::upper_bound(
      right_boundary_point.begin(), right_boundary_point.end(), 0.0,
      [](const int& val, const Vec2d& point) { return point.x() > val; });
  if (left_it == left_boundary_point.end() ||
      right_it == right_boundary_point.end()) {
    return Status(common::ErrorCode::LOCALVIEW_MAP_LANE_ERROR,
                  "central_lane does not exist!");
  }
  if (left_it->y() > 5 || left_it->y() < -0.5 || right_it->y() < -5 ||
      right_it->y() > 0.5) {
    return Status(common::ErrorCode::LOCALVIEW_MAP_LANE_ERROR,
                  "vehicle is not in lane!");
  }

  /// construct left centerline
  left_central_line_pts_.clear();
  auto left_central_navi_path =
      std::make_shared<navigation_hdmap::NavigationPath>();
  auto* left_central_path = left_central_navi_path->mutable_path();
  left_central_path->set_name("left");
  // ADEBUG << "left_central_line";
  std::vector<Vec2d> left_boundary_point_of_left_lane;
  std::vector<double> left_lane_left_width;
  std::vector<double> left_lane_right_width;
  auto left_boundary_of_leftlane = std::make_shared<hdmap::LineSegment>();
  auto right_boundary_of_leftlane = std::make_shared<hdmap::LineSegment>();
  if (lane_marker_points->find("next_left") != lane_marker_points->end() &&
      navi_central_line_constructor_.ExtendNextReferenceLine(
          left_boundary_point, -left_lane_width, &left_central_line_pts_,
          &left_boundary_point_of_left_lane)) {
    CalculateLaneWidth(left_boundary_point_of_left_lane, left_boundary_point,
                       left_central_line_pts_, &left_lane_left_width,
                       &left_lane_right_width);
    if (!left_central_line_pts_.empty()) {
      // double left_central_line_one_y = left_central_line_pts_.front().y();
      TranslatedReferenceLine(
          lane_marker, left_central_line_pts_, left_central_path,
          left_boundary_point_of_left_lane, left_boundary_point,
          left_boundary_of_leftlane.get(), right_boundary_of_leftlane.get());
      left_central_navi_path->set_path_priority(FLAGS_left_navi_path_priority);
      auto left_lane_type = std::make_pair(
          lane_marker.front_next_left_lane_marker().at(0).lane_type(),
          lane_marker.front_left_lane_marker().lane_type());
      navigation_path_list->emplace_front(
          1, std::move(left_lane_left_width), std::move(left_lane_right_width),
          left_central_navi_path, left_boundary_of_leftlane,
          right_boundary_of_leftlane, left_lane_type);
    }
  }

  /// construct right centerline
  right_central_line_pts_.clear();
  auto right_central_navi_path =
      std::make_shared<navigation_hdmap::NavigationPath>();
  auto* right_central_path = right_central_navi_path->mutable_path();
  right_central_path->set_name("right");
  // bool right_c0 = lane_marker.next_right_lane_marker().empty();
  std::vector<Vec2d> right_boundary_point_of_right_lane;
  std::vector<double> right_lane_left_width;
  std::vector<double> right_lane_right_width;
  auto left_boundary_of_rightlane = std::make_shared<hdmap::LineSegment>();
  auto right_boundary_of_rightlane = std::make_shared<hdmap::LineSegment>();
  // !lane_marker.next_right_lane_marker().at(0).longitude_end() < 0.01 &&
  if (lane_marker_points->find("next_right") != lane_marker_points->end() &&
      navi_central_line_constructor_.ExtendNextReferenceLine(
          right_boundary_point, right_lane_width, &right_central_line_pts_,
          &right_boundary_point_of_right_lane)) {
    CalculateLaneWidth(right_boundary_point, right_boundary_point_of_right_lane,
                       right_central_line_pts_, &right_lane_left_width,
                       &right_lane_right_width);
    if (!right_central_line_pts_.empty()) {
      TranslatedReferenceLine(
          lane_marker, right_central_line_pts_, right_central_path,
          right_boundary_point, right_boundary_point_of_right_lane,
          left_boundary_of_rightlane.get(), right_boundary_of_rightlane.get());
      right_central_navi_path->set_path_priority(
          FLAGS_right_navi_path_priority);  //
      auto right_lane_type = std::make_pair(
          lane_marker.front_right_lane_marker().lane_type(),
          lane_marker.front_next_right_lane_marker().at(0).lane_type());
      navigation_path_list->emplace_back(
          3, std::move(right_lane_left_width),
          std::move(right_lane_right_width), right_central_navi_path,
          left_boundary_of_rightlane, right_boundary_of_rightlane,
          right_lane_type);
    }
  }

  ADEBUG << "Central line constructor end";
  return Status::OK();
}

void NavigationLanePathGenerator::TranslatedReferenceLine(
    const perception::LaneMarkers& lane_marker,
    const std::vector<Vec2d>& ref_line_point, common::Path* const path,
    const std::vector<Vec2d>& left_line_point,
    const std::vector<Vec2d>& right_line_point,
    hdmap::LineSegment* const left_path, hdmap::LineSegment* const right_path) {
  const auto& left_lane = lane_marker.front_left_lane_marker();
  const auto& right_lane = lane_marker.front_right_lane_marker();

  double left_quality = left_lane.quality() + kMinNum;
  double right_quality = right_lane.quality() + kMinNum;

  double quality_divider = left_quality + right_quality;

  double path_c1 = (left_lane.c1_heading_angle() * left_quality +
                    right_lane.c1_heading_angle() * right_quality) /
                   quality_divider;

  double path_c2 = (left_lane.c2_curvature() * left_quality +
                    right_lane.c2_curvature() * right_quality) /
                   quality_divider;

  double path_c3 = (left_lane.c3_curvature_derivative() * left_quality +
                    right_lane.c3_curvature_derivative() * right_quality) /
                   quality_divider;

  const double start_s = left_lane.longitude_start();
  double accumulated_s = start_s;
  path->mutable_path_point()->Reserve(ref_line_point.size());  //NOLINT
  for (size_t i = 1; i < ref_line_point.size(); ++i) {
    if (i == 1) {
      auto* point = path->add_path_point();
      Eigen::Vector2d enu_coordinate = common::math::RotateVector2d(
          {ref_line_point[i - 1].x(), ref_line_point[i - 1].y()},
          vehicle_state_->heading());
      double x_enu = enu_coordinate.x() + vehicle_state_->x();
      double y_enu = enu_coordinate.y() + vehicle_state_->y();
      point->set_x(x_enu);
      point->set_y(y_enu);
      point->set_theta(vehicle_state_->heading());
      point->set_s(accumulated_s);
      point->set_kappa(
          GetKappa(path_c1, path_c2, path_c3, ref_line_point[i - 1].x()));
      point->set_dkappa(0.0);
    }
    double x1 = ref_line_point[i].x();
    double y1 = ref_line_point[i].y();
    auto* point = path->add_path_point();
    Vec2d vec = ref_line_point[i] - ref_line_point[i - 1];
    double theta = std::atan2(vec.y(), vec.x());
    Eigen::Vector2d enu_coordinate =
        common::math::RotateVector2d({x1, y1}, vehicle_state_->heading());
    double x_enu = enu_coordinate.x() + vehicle_state_->x();
    double y_enu = enu_coordinate.y() + vehicle_state_->y();
    double theta_enu = common::math::NormalizeAngle(
        common::math::NormalizeAngle(theta) + vehicle_state_->heading());
    if (path->path_point_size() > 1) {
      const auto& pre_point = path->path_point(path->path_point_size() - 2);
      accumulated_s += std::hypot(x_enu - pre_point.x(), y_enu - pre_point.y());
    }
    point->set_x(x_enu);
    point->set_y(y_enu);
    point->set_theta(theta_enu);
    point->set_s(accumulated_s);
    point->set_kappa(GetKappa(path_c1, path_c2, path_c3, x1));

    const double k1 = GetKappa(path_c1, path_c2, path_c3, x1 - kMinNum);
    const double k2 = GetKappa(path_c1, path_c2, path_c3, x1 + kMinNum);
    point->set_dkappa((k2 - k1) / (kMinNum * 2));
  }
  BoundaryLineBus2Earth(left_line_point, left_path);
  BoundaryLineBus2Earth(right_line_point, right_path);
}

void NavigationLanePathGenerator::BoundaryLineBus2Earth(
    const std::vector<Vec2d>& line_point, hdmap::LineSegment* const path) {
  path->mutable_point()->Reserve(line_point.size());  // NOLINT
  for (auto i : line_point) {
    double x1 = i.x();
    double y1 = i.y();
    const auto& point = path->add_point();
    Eigen::Vector2d enu_coordinate =
        common::math::RotateVector2d({x1, y1}, vehicle_state_->heading());
    double x_enu = enu_coordinate.x() + vehicle_state_->x();
    double y_enu = enu_coordinate.y() + vehicle_state_->y();
    point->set_x(x_enu);
    point->set_y(y_enu);
  }
}

void NavigationLanePathGenerator::CalculateLaneWidth(
    const std::vector<Vec2d>& left_line_point,
    const std::vector<Vec2d>& right_line_point,
    const std::vector<Vec2d>& cetral_line_point,
    std::vector<double>* left_line_width,
    std::vector<double>* right_line_width) {
  if (left_line_point.empty() || right_line_point.empty()) {
    return;
  }
  double left_lane_width = 0.0;
  double right_lane_width = 0.0;
  const auto min_size =
      std::min(left_line_point.size(), right_line_point.size());
  left_line_width->reserve(min_size);
  right_line_width->reserve(min_size);
  for (size_t i = 0; i < min_size; ++i) {
    left_lane_width = left_line_point[i].DistanceTo(cetral_line_point[i]);
    right_lane_width = right_line_point[i].DistanceTo(cetral_line_point[i]);
    left_line_width->emplace_back(left_lane_width);
    right_line_width->emplace_back(right_lane_width);
  }
}

void NavigationLanePathGenerator::CutNexLaneLength(
    std::vector<Vec2d>* left_line_point, std::vector<Vec2d>* right_line_point,
    std::vector<Vec2d>* central_line_point,
    std::vector<double>* left_line_width, std::vector<double>* right_line_width,
    double* history_lane_length) {
  ADEBUG << "lane point size,left_line_point : " << left_line_point->size()
         << ", right_line_point: " << right_line_point->size()
         << ", central_line_point: " << central_line_point->size();
  int start_index = 0;
  for (uint i = 0; i < central_line_point->size(); ++i) {
    if (std::fabs(central_line_point->at(i).x()) <= kMinNum ||
        central_line_point->at(i).x() > 0.0) {
      start_index = i;  // NOLINT
      break;
    }
  }
  ADEBUG << "start_index: " << start_index;
  int increase_index = central_line_point->size() - 1;  // NOLINT
  int reduce_index = central_line_point->size() - 1;    // NOLINT
  double lane_width = 0.0;
  bool increase_index_flag = false;
  double new_lane_index = 0.0;
  for (size_t i = start_index; i < left_line_width->size(); ++i) {
    lane_width = (*left_line_width)[i] + (*right_line_width)[i];
    // AERROR << "lane_width[" << i << "]: " << lane_width;
    if (!increase_index_flag && lane_width < 2.5) {
      increase_index = i;  // NOLINT
      increase_index_flag = true;
      ADEBUG << "increase_index: " << increase_index;
    }
    if (lane_width < 2.2) {
      reduce_index = i;  // NOLINT
      ADEBUG << "reduce_index: " << reduce_index;
      break;
    }
  }
  double increase_length = central_line_point->at(increase_index).x();
  double reduce_length = central_line_point->at(reduce_index).x();
  if (reduce_length < *history_lane_length) {
    *history_lane_length = reduce_length;
    new_lane_index = reduce_index;
  } else if (increase_length > *history_lane_length) {
    *history_lane_length = increase_length;
    new_lane_index = increase_index;
  }
  if (new_lane_index >= central_line_point->size() - 1) {  // NOLINT
    return;
  }

  while (central_line_point->size() > new_lane_index) {  // NOLINT
    central_line_point->pop_back();
    left_line_point->pop_back();
    right_line_point->pop_back();
    left_line_width->pop_back();
    right_line_width->pop_back();
  }
}

double NavigationLanePathGenerator::LaneWidthAverage(
    const int lane_width_valid, const double lane_width,
    std::deque<double>* lane_width_dq) {
  // std::cout << "lane_width_valid " << lane_width_valid << std::endl;
  if (lane_width_valid == 1) {
    lane_width_dq->push_front(lane_width);
  } else {
    lane_width_dq->push_front(3.75);
  }

  if (lane_width_dq->size() > 10) {
    lane_width_dq->pop_back();
  }

  double lane_width_average = 0.0;
  if (!lane_width_dq->empty()) {
    double lane_width_sum = 0.0;
    for (double i : *lane_width_dq) {
      lane_width_sum = lane_width_sum + i;
      // lane_width_sum = lane_width_sum + lane_width_dq->at(i);
    }
    lane_width_average = lane_width_sum / lane_width_dq->size();  // NOLINT
  } else {
    lane_width_average = 3.75;
  }

  return lane_width_average;
}

// void NavigationLanePathGenerator::GetCentralLaneFitLanemarker(
//     const double view_range, const std::vector<Vec2d>& cetral_line_point,
//     perception::LaneMarker* const central_lanemarker) {
//   if (central_lanemarker == nullptr) {
//     AERROR << "central_lanemarkers is nullptr!!!";
//     return;
//   }
//   std::vector<Vec2d> line_points;
//   for (const auto& point : cetral_line_point) {
//     if (point.x() < view_range) {
//       line_points.emplace_back(point);
//     }
//   }
//   const int N = 3;
//   std::vector<double> coff =
//       TL::common::math::FitPolynomial<N>(line_points);
//   central_lanemarker->set_c0_position(coff[0]);
//   central_lanemarker->set_c1_heading_angle(coff[1]);
//   central_lanemarker->set_c2_curvature(coff[2]);
//   central_lanemarker->set_c3_curvature_derivative(coff[3]);
//   ADEBUG << "central line c0 = " << coff[0] << ", c1 = " << coff[1]
//          << ", c2 = " << coff[2] << ", c3 = " << coff[3];
// }
}  // namespace planning
}  // namespace TL
