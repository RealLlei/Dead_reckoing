//  Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.

#include "planning/common/open_space_chart.h"
#include <sys/types.h>
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include "absl/strings/str_cat.h"
#include "common/math/vec2d.h"

#include "planning/common/open_space_info.h"
#include "planning/proto/task_config.pb.h"
#include "proto/planning/planning_internal.pb.h"

constexpr double kEpsilon = 1.0e-3;

namespace TL {
namespace planning {
using TL::common::math::Vec2d;

OpenSpaceChart::OpenSpaceChart(
    Frame* frame, const std::shared_ptr<DependencyInjector>& injector)
    : frame_(frame), injector_(injector) {}

void OpenSpaceChart::ExportOpenSpaceChart(
    const planning_internal::Debug& debug_info,
    const planning::ADCTrajectory& trajectory_pb,
    planning_internal::Debug* debug_chart) {
  // Export Trajectory Visualization Chart.
  if (FLAGS_enable_record_debug) {
    AddOpenSpaceOptimizerResult(debug_info, debug_chart);
    AddPartitionedPath(debug_info, debug_chart);
    AddPublishedSpeed(trajectory_pb, debug_chart);
    AddPublishedAcceleration(trajectory_pb, debug_chart);
    AddSpeedPlanDebugInfo(trajectory_pb, debug_chart);
  }
}

void OpenSpaceChart::AddOpenSpaceOptimizerResult(
    const planning_internal::Debug& debug_info,
    planning_internal::Debug* const debug_chart) {
  if (nullptr == debug_chart) {
    return;
  }

  // if open space info provider success run
  auto open_space_debug = debug_info.planning_data().open_space();
  bool openspace_hy_flag = false;
  const auto& warm_start_path =
      open_space_debug.warm_start_path().warm_start_path_points();
  const auto& rs_connection_point =
      open_space_debug.warm_start_path().rs_connect_point();
  int connect_point_index = warm_start_path.size() - 1;
  for (int i = 0; i < warm_start_path.size(); i++) {
    if (DiscretizedPath::IsSamePoint(rs_connection_point,
                                     warm_start_path.at(i))) {
      connect_point_index = i;
      break;
    }
  }
  auto warm_start_size = warm_start_path.size();
  if (!open_space_debug.xy_boundary().empty()) {
    auto* chart_warm = debug_chart->mutable_planning_data()->add_chart();
    chart_warm->set_title("WarmStart");
    if (warm_start_size > 0) {
      openspace_hy_flag = true;
      size_t adc_label = 0;
      // distinct exploration from  rs points by WarmStart_a and  WarmStart_b
      auto* warm_start_line_a = chart_warm->add_line();
      warm_start_line_a->set_label("WarmStart_a");
      for (int i = 0; i <= connect_point_index; i++) {
        // Draw vehicle shape along the trajectory
        if (adc_label == 0) {
          auto* adc_shape = chart_warm->add_car();
          adc_shape->set_x(warm_start_path.at(i).x());
          adc_shape->set_y(warm_start_path.at(i).y());
          adc_shape->set_heading(warm_start_path.at(i).theta());
          adc_shape->set_color("rgba(54, 162, 235, 1)");
          adc_shape->set_label(std::to_string(adc_label));
          adc_shape->set_hide_label_in_legend(true);
        }
        ++adc_label;
        auto* point_debug = warm_start_line_a->add_point();
        point_debug->set_x(warm_start_path.at(i).x());
        point_debug->set_y(warm_start_path.at(i).y());
      }
      auto* warm_start_line_b = chart_warm->add_line();
      warm_start_line_b->set_label("WarmStart_b");
      for (int i = connect_point_index + 1; i < warm_start_path.size(); i++) {
        auto* point_debug = warm_start_line_b->add_point();
        point_debug->set_x(warm_start_path.at(i).x());
        point_debug->set_y(warm_start_path.at(i).y());
      }
      // Set chartJS's dataset properties
      auto* warm_start_properties_a = warm_start_line_a->mutable_properties();
      auto* warm_start_properties_b = warm_start_line_b->mutable_properties();
      (*warm_start_properties_a)["borderWidth"] = "2";
      (*warm_start_properties_a)["pointRadius"] = "0";
      (*warm_start_properties_a)["lineTension"] = "0";
      (*warm_start_properties_a)["fill"] = "false";
      (*warm_start_properties_a)["showLine"] = "true";

      (*warm_start_properties_b)["borderWidth"] = "2";
      (*warm_start_properties_b)["pointRadius"] = "0";
      (*warm_start_properties_b)["lineTension"] = "0";
      (*warm_start_properties_b)["fill"] = "false";
      (*warm_start_properties_b)["showLine"] = "true";
      // Set different color for exploration and rs points
      (*warm_start_properties_a)["color"] = "rgba(255, 0, 0, 1)";
      (*warm_start_properties_b)["color"] = "rgba(54, 162, 235, 1)";
    }

    if (open_space_debug.has_trajectory_stitching_point() &&
        (!openspace_hy_flag)) {
      size_t adc_label = 0;
      auto x = open_space_debug.trajectory_stitching_point().path_point().x();
      auto y = open_space_debug.trajectory_stitching_point().path_point().y();
      auto heading =
          open_space_debug.trajectory_stitching_point().path_point().theta();

      auto x_origin = open_space_debug.roi_shift_point().path_point().x();
      auto y_origin = open_space_debug.roi_shift_point().path_point().y();
      auto heading_origin =
          open_space_debug.roi_shift_point().path_point().theta();

      x -= x_origin;
      y -= y_origin;
      double tmp_x = x;
      x = x * std::cos(-heading_origin) - (y)*std::sin(-heading_origin);
      y = tmp_x * std::sin(-heading_origin) + (y)*std::cos(-heading_origin);
      heading = common::math::NormalizeAngle(heading - heading_origin);

      // Draw vehicle shape along the trajectory
      auto* adc_shape = chart_warm->add_car();
      adc_shape->set_x(x);
      adc_shape->set_y(y);
      adc_shape->set_heading(heading);
      adc_shape->set_color("rgba(54, 162, 235, 1)");
      adc_shape->set_label(std::to_string(adc_label));
      adc_shape->set_hide_label_in_legend(true);
    }

    auto* xyboundary_start_line = chart_warm->add_line();
    auto* xy_start_properties = xyboundary_start_line->mutable_properties();
    (*xy_start_properties)["borderWidth"] = "2";
    (*xy_start_properties)["pointRadius"] = "0";
    (*xy_start_properties)["lineTension"] = "0";
    (*xy_start_properties)["fill"] = "false";
    (*xy_start_properties)["showLine"] = "true";

    PopulateChartOptions(open_space_debug.xy_boundary(0) - 1.0,
                         open_space_debug.xy_boundary(1) + 1.0, "x (meter)",
                         open_space_debug.xy_boundary(2) - 1.0,
                         open_space_debug.xy_boundary(3) + 1.0, "y (meter)",
                         true, chart_warm);

    UpdateObstacle(debug_info.planning_data().open_space().obstacles(),
                   chart_warm);
    AINFO << "Openspace chart obstacle size = "
          << open_space_debug.obstacles().size();
  } else {
    if (!open_space_debug.multi_search_info().empty()) {
      AddOpenSpaceMultiSearchResult(debug_info, debug_chart);
    }
  }

  if (!open_space_debug.xy_boundary().empty()) {
    auto* chart = debug_chart->mutable_planning_data()->add_chart();
    chart->set_title("Open Space Path Optimizer Visualization");
    PopulateChartOptions(open_space_debug.xy_boundary(0) - 1.0,
                         open_space_debug.xy_boundary(1) + 1.0, "x (meter)",
                         open_space_debug.xy_boundary(2) - 1.0,
                         open_space_debug.xy_boundary(3) + 1.0, "y (meter)",
                         true, chart);
    UpdateObstacle(debug_info.planning_data().open_space().obstacles(), chart);
    auto smoothed_path = open_space_debug.smoothed_path();
    auto* smoothed_line = chart->add_line();
    smoothed_line->set_label("Smooth");
    size_t adc_label = 0;
    for (const auto& point : smoothed_path) {
      // Draw vehicle shape along the trajectory
      if (adc_label == 0) {
        auto* adc_shape = chart->add_car();
        adc_shape->set_x(point.x());
        adc_shape->set_y(point.y());
        adc_shape->set_heading(point.theta());
        adc_shape->set_color("rgba(54, 162, 235, 1)");
        adc_shape->set_label(std::to_string(adc_label));
        adc_shape->set_hide_label_in_legend(true);
      }
      ++adc_label;

      // Draw vehicle trajectory points
      auto* point_debug = smoothed_line->add_point();
      point_debug->set_x(point.x());
      point_debug->set_y(point.y());
    }

    // Set chartJS's dataset properties
    auto* smoothed_properties = smoothed_line->mutable_properties();
    (*smoothed_properties)["borderWidth"] = "2";
    (*smoothed_properties)["pointRadius"] = "0";
    (*smoothed_properties)["lineTension"] = "0";
    (*smoothed_properties)["fill"] = "false";
    (*smoothed_properties)["showLine"] = "true";
  }
}

void PathDeNormal(const common::PathPoint& origin,
                  google::protobuf::RepeatedPtrField<common::PathPoint>* const
                      warm_path_ptr) {
  if (warm_path_ptr == nullptr) {
    AERROR << "PathDeNormal input check fails";
    return;
  }
  for (auto& point : *warm_path_ptr) {
    double x = point.x();
    double y = point.y();
    point.set_x(x * std::cos(origin.theta()) - y * std::sin(origin.theta()) +
                origin.x());
    point.set_y(x * std::sin(origin.theta()) + y * std::cos(origin.theta()) +
                origin.y());
    point.set_theta(
        common::math::NormalizeAngle(point.theta() + origin.theta()));
  }
}

void PathNormal(const common::PathPoint& origin,
                google::protobuf::RepeatedPtrField<common::PathPoint>* const
                    warm_path_ptr) {
  if (warm_path_ptr == nullptr) {
    AERROR << "PathNormal input check fails";
    return;
  }
  for (auto& point : *warm_path_ptr) {
    double x = point.x() - origin.x();
    double y = point.y() - origin.y();
    point.set_x(x * std::cos(-origin.theta()) - y * std::sin(-origin.theta()));
    point.set_y(x * std::sin(-origin.theta()) + y * std::cos(-origin.theta()));
    point.set_theta(
        common::math::NormalizeAngle(point.theta() - origin.theta()));
  }
}

void OpenSpaceChart::AddOpenSpaceMultiSearchResult(
    const planning_internal::Debug& debug_info,
    planning_internal::Debug* const debug_chart) {
  if (nullptr == debug_chart) {
    return;
  }
  const auto multi_search_info =
      debug_info.planning_data().open_space().multi_search_info();
  const int opt_park_id = frame_->open_space_info().open_space_path_info_id();
  auto opt_lot_search_info_iter = std::find_if(
      multi_search_info.begin(), multi_search_info.end(),
      [opt_park_id](const planning_internal::MultiSearchDebug& search_info) {
        return search_info.park_id() == opt_park_id;
      });
  if (opt_lot_search_info_iter == multi_search_info.end()) {
    ADEBUG << "fail to find opt park lot in search result(s)";
    return;
  }

  auto* chart_warm = debug_chart->mutable_planning_data()->add_chart();
  chart_warm->set_title("WarmStart");
  std::vector<std::string> colot_map_a = {
      "rgba(255, 0, 0, 1)", "rgba(0, 255, 0, 1)", "rgba(0, 0, 255, 1)"};
  for (int i_search_info = 0; i_search_info < multi_search_info.size();
       ++i_search_info) {
    const auto& open_space_debug = multi_search_info.at(i_search_info);

    // if open space info provider success run
    bool openspace_hy_flag = false;
    const auto& warm_start_path_local =
        open_space_debug.warm_start_path().warm_start_path_points();
    const auto& rs_connection_point =
        open_space_debug.warm_start_path().rs_connect_point();
    int connect_point_index = warm_start_path_local.size() - 1;
    for (int i = 0; i < warm_start_path_local.size(); i++) {
      if (DiscretizedPath::IsSamePoint(rs_connection_point,
                                       warm_start_path_local.at(i))) {
        connect_point_index = i;
        break;
      }
    }

    auto warm_start_path = warm_start_path_local;
    ADEBUG << opt_park_id << "," << open_space_debug.park_id();
    if (open_space_debug.park_id() != opt_park_id) {
      PathDeNormal(open_space_debug.roi_shift_point().path_point(),
                   &warm_start_path);
      PathNormal(opt_lot_search_info_iter->roi_shift_point().path_point(),
                 &warm_start_path);
    }

    auto warm_start_size = warm_start_path.size();
    if (!open_space_debug.xy_boundary().empty()) {
      if (warm_start_size > 0) {
        openspace_hy_flag = true;
        size_t adc_label = 0;
        // distinct exploration from  rs points by WarmStart_a and  WarmStart_b
        auto* warm_start_line_a = chart_warm->add_line();
        warm_start_line_a->set_label(
            absl::StrCat("WarmStart_a", open_space_debug.park_id()));
        for (int i = 0; i <= connect_point_index; i++) {
          // Draw vehicle shape along the trajectory
          if (adc_label == 0) {
            auto* adc_shape = chart_warm->add_car();
            adc_shape->set_x(warm_start_path.at(i).x());
            adc_shape->set_y(warm_start_path.at(i).y());
            adc_shape->set_heading(warm_start_path.at(i).theta());
            adc_shape->set_color("rgba(54, 162, 235, 1)");
            adc_shape->set_label(std::to_string(adc_label));
            adc_shape->set_hide_label_in_legend(true);
          }
          ++adc_label;
          auto* point_debug = warm_start_line_a->add_point();
          point_debug->set_x(warm_start_path.at(i).x());
          point_debug->set_y(warm_start_path.at(i).y());
        }
        auto* warm_start_line_b = chart_warm->add_line();
        warm_start_line_b->set_label(
            absl::StrCat("WarmStart_b", open_space_debug.park_id()));
        for (int i = connect_point_index + 1; i < warm_start_path.size(); i++) {
          auto* point_debug = warm_start_line_b->add_point();
          point_debug->set_x(warm_start_path.at(i).x());
          point_debug->set_y(warm_start_path.at(i).y());
        }
        // Set chartJS's dataset properties
        auto* warm_start_properties_a = warm_start_line_a->mutable_properties();
        auto* warm_start_properties_b = warm_start_line_b->mutable_properties();
        (*warm_start_properties_a)["borderWidth"] = "2";
        (*warm_start_properties_a)["pointRadius"] = "0";
        (*warm_start_properties_a)["lineTension"] = "0";
        (*warm_start_properties_a)["fill"] = "false";
        (*warm_start_properties_a)["showLine"] = "true";

        (*warm_start_properties_b)["borderWidth"] = "2";
        (*warm_start_properties_b)["pointRadius"] = "0";
        (*warm_start_properties_b)["lineTension"] = "0";
        (*warm_start_properties_b)["fill"] = "false";
        (*warm_start_properties_b)["showLine"] = "true";
        // Set different color for exploration and rs points
        (*warm_start_properties_a)["color"] = colot_map_a[i_search_info];
        (*warm_start_properties_b)["color"] = "rgba(54, 162, 235, 1)";
      }

      if (open_space_debug.has_trajectory_stitching_point() &&
          (!openspace_hy_flag)) {
        size_t adc_label = 0;
        auto x = open_space_debug.trajectory_stitching_point().path_point().x();
        auto y = open_space_debug.trajectory_stitching_point().path_point().y();
        auto heading =
            open_space_debug.trajectory_stitching_point().path_point().theta();

        auto x_origin =
            opt_lot_search_info_iter->roi_shift_point().path_point().x();
        auto y_origin =
            opt_lot_search_info_iter->roi_shift_point().path_point().y();
        auto heading_origin =
            opt_lot_search_info_iter->roi_shift_point().path_point().theta();

        x -= x_origin;
        y -= y_origin;
        double tmp_x = x;
        x = x * std::cos(-heading_origin) - (y)*std::sin(-heading_origin);
        y = tmp_x * std::sin(-heading_origin) + (y)*std::cos(-heading_origin);
        heading = common::math::NormalizeAngle(heading - heading_origin);

        // Draw vehicle shape along the trajectory
        auto* adc_shape = chart_warm->add_car();
        adc_shape->set_x(x);
        adc_shape->set_y(y);
        adc_shape->set_heading(heading);
        adc_shape->set_color("rgba(54, 162, 235, 1)");
        adc_shape->set_label(std::to_string(adc_label));
        adc_shape->set_hide_label_in_legend(true);
      }
    }
  }
  if (opt_lot_search_info_iter->xy_boundary().empty()) {
    return;
  }
  auto* xyboundary_start_line = chart_warm->add_line();
  auto* xy_start_properties = xyboundary_start_line->mutable_properties();
  (*xy_start_properties)["borderWidth"] = "2";
  (*xy_start_properties)["pointRadius"] = "0";
  (*xy_start_properties)["lineTension"] = "0";
  (*xy_start_properties)["fill"] = "false";
  (*xy_start_properties)["showLine"] = "true";

  PopulateChartOptions(opt_lot_search_info_iter->xy_boundary(0) - 1.0,
                       opt_lot_search_info_iter->xy_boundary(1) + 1.0,
                       "x (meter)",
                       opt_lot_search_info_iter->xy_boundary(2) - 1.0,
                       opt_lot_search_info_iter->xy_boundary(3) + 1.0,
                       "y (meter)", true, chart_warm);

  UpdateObstacle(opt_lot_search_info_iter->obstacles(), chart_warm);
}

void Rotate2DPoint(Vec2d* const car_point, double heading) {
  double tmp_x = car_point->x();
  car_point->set_x(car_point->x() * std::cos(heading) -
                   car_point->y() * std::sin(heading));
  car_point->set_y(tmp_x * std::sin(heading) +
                   car_point->y() * std::cos(heading));
}

std::vector<Vec2d> CalculateRadarPoints(double positionX, double positionY,
                                        double heading) {
  const auto& vehicle_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  std::vector<Vec2d> car_points;
  car_points.emplace_back(-vehicle_param.left_edge_to_center(),
                          vehicle_param.front_edge_to_center());
  car_points.emplace_back(-vehicle_param.left_edge_to_center() * 2.0 / 5.0,
                          vehicle_param.front_edge_to_center());
  car_points.emplace_back(vehicle_param.right_edge_to_center(),
                          vehicle_param.front_edge_to_center());
  car_points.emplace_back(vehicle_param.right_edge_to_center() * 2.0 / 5.0,
                          vehicle_param.front_edge_to_center());
  car_points.emplace_back(vehicle_param.right_edge_to_center(),
                          vehicle_param.front_edge_to_center() * 3.0 / 4.0);
  car_points.emplace_back(vehicle_param.right_edge_to_center(), 0);
  car_points.emplace_back(vehicle_param.right_edge_to_center(),
                          -vehicle_param.back_edge_to_center());
  car_points.emplace_back(vehicle_param.right_edge_to_center() * 2.0 / 5.0,
                          -vehicle_param.back_edge_to_center());
  car_points.emplace_back(-vehicle_param.left_edge_to_center() * 2.0 / 5.0,
                          -vehicle_param.back_edge_to_center());
  car_points.emplace_back(-vehicle_param.left_edge_to_center(),
                          -vehicle_param.back_edge_to_center());
  car_points.emplace_back(-vehicle_param.left_edge_to_center(), 0);
  car_points.emplace_back(-vehicle_param.left_edge_to_center(),
                          vehicle_param.front_edge_to_center() * 3.0 / 4.0);
  for (auto& point : car_points) {
    Rotate2DPoint(&point, heading - M_PI / 2);
    point.set_x(positionX + point.x());
    point.set_y(positionY + point.y());
  }
  return car_points;
}

void OpenSpaceChart::UpdateObstacle(
    const google::protobuf::RepeatedPtrField<planning_internal::ObstacleDebug>&
        obstacles,
    dreamview::Chart* const chart) {
  if (nullptr == chart) {
    return;
  }

  int obstacle_index = 0;
  for (const auto& obstacle : obstacles) {
    auto* obstacle_outline = chart->add_line();
    if (0 == obstacle_index) {
      obstacle_outline->set_label("Bdr");
    } else {
      obstacle_outline->set_label(absl::StrCat("Bdr", obstacle_index));
      obstacle_outline->set_hide_label_in_legend(true);
    }
    obstacle_index += 1;
    for (int vertice_index = 0;
         vertice_index < obstacle.vertices_x_coords_size(); vertice_index++) {
      auto* point_debug = obstacle_outline->add_point();
      point_debug->set_x(obstacle.vertices_x_coords(vertice_index));
      point_debug->set_y(obstacle.vertices_y_coords(vertice_index));
    }
    // Set chartJS's dataset properties
    auto* obstacle_properties = obstacle_outline->mutable_properties();
    if (obstacle.id() == "wheel_mask") {
      (*obstacle_properties)["borderWidth"] = "5";
      (*obstacle_properties)["pointRadius"] = "6";
      (*obstacle_properties)["lineTension"] = "0";
      (*obstacle_properties)["fill"] = "false";
      (*obstacle_properties)["showLine"] = "false";
      (*obstacle_properties)["color"] = "rgba(255, 97, 3, 1)";
    } else {
      (*obstacle_properties)["borderWidth"] = "2";
      (*obstacle_properties)["pointRadius"] = "0";
      (*obstacle_properties)["lineTension"] = "0";
      (*obstacle_properties)["fill"] = "false";
      (*obstacle_properties)["showLine"] = "true";
      (*obstacle_properties)["color"] = "rgba(255, 255, 25, 1)";
    }
  }
}

void OpenSpaceChart::UpdateObstacleRealTime(dreamview::Chart* const chart) {
  if (nullptr == chart) {
    return;
  }
  int obstacle_index = 0;
  for (const auto& obstacle_seg : frame_->open_space_info()
                                      .open_space_path_info()
                                      .obstacles_segments_vec) {
    auto* obstacle_outline = chart->add_line();
    if (0 == obstacle_index) {
      obstacle_outline->set_label("obs");
    } else {
      obstacle_outline->set_label(absl::StrCat("obs", obstacle_index));
      obstacle_outline->set_hide_label_in_legend(true);
    }
    const auto& obstacle = obstacle_seg.first;
    const auto& obstacle_buffer = obstacle_seg.second;
    auto* point_debug = obstacle_outline->add_point();
    point_debug->set_x(obstacle.start().x());
    point_debug->set_y(obstacle.start().y());
    auto* point_debug_1 = obstacle_outline->add_point();
    point_debug_1->set_x(obstacle.end().x());
    point_debug_1->set_y(obstacle.end().y());
    ++obstacle_index;
    // Set chartJS's dataset properties
    auto* obstacle_properties = obstacle_outline->mutable_properties();
    if (fabs(obstacle_buffer - 0.3) < kEpsilon &&
        obstacle.length() < kEpsilon) {
      // draw wheel mask circles
      ADEBUG << "draw wheel mask point at " << obstacle.start().x() << ","
             << obstacle.start().y() << " with buffer " << obstacle_buffer;
      (*obstacle_properties)["borderWidth"] = "5";
      (*obstacle_properties)["pointRadius"] = "6";
      (*obstacle_properties)["lineTension"] = "0";
      (*obstacle_properties)["fill"] = "false";
      (*obstacle_properties)["showLine"] = "false";
      (*obstacle_properties)["color"] = "rgba(255, 97, 3, 1)";
    } else {
      (*obstacle_properties)["borderWidth"] = "2";
      (*obstacle_properties)["pointRadius"] = "0";
      (*obstacle_properties)["lineTension"] = "0";
      (*obstacle_properties)["fill"] = "false";
      (*obstacle_properties)["showLine"] = "true";
      (*obstacle_properties)["color"] = "rgba(255, 255, 25, 1)";
    }
  }
}

void OpenSpaceChart::UpdateRadarPos(dreamview::Line* const radar_dot,
                                    const std::vector<Vec2d>& points) {
  if (nullptr == radar_dot) {
    return;
  }

  radar_dot->set_label("radar_obs");
  auto* dot_properties = radar_dot->mutable_properties();
  // auto heading = vehicle_state.heading();
  // auto car_points =
  //     CalculateRadarPoints(vehicle_state.x(), vehicle_state.y(), heading);
  if (!frame_->obstacles().empty()) {
    for (const auto* obstacle : frame_->obstacles()) {
      const auto& perception_obs = (*obstacle)->Perception();
      if (perception_obs.has_sub_type() &&
          perception_obs.sub_type() != perception::PerceptionObstacle::ST_USS) {
        continue;
      }
      if (perception_obs.has_current_detect_sensor() &&
          perception_obs.has_position()) {
        const auto& detect_sensor_cur = perception_obs.current_detect_sensor();
        uint detect_sensor = 0U;
        if (detect_sensor_cur.from_uss_fol()) {
          detect_sensor |= 0x02U;
        }
        if (detect_sensor_cur.from_uss_fcl()) {
          detect_sensor |= 0x04U;
        }
        if (detect_sensor_cur.from_uss_fcr()) {
          detect_sensor |= 0x08U;
        }
        if (detect_sensor_cur.from_uss_for()) {
          detect_sensor |= 0x10U;
        }
        if (detect_sensor_cur.from_uss_fsr()) {
          detect_sensor |= 0x20U;
        }
        if (detect_sensor_cur.from_uss_rsr()) {
          detect_sensor |= 0x40U;
        }
        if (detect_sensor_cur.from_uss_ror()) {
          detect_sensor |= 0x80U;
        }
        if (detect_sensor_cur.from_uss_rcr()) {
          detect_sensor |= 0x100U;
        }
        if (detect_sensor_cur.from_uss_rcl()) {
          detect_sensor |= 0x200U;
        }
        if (detect_sensor_cur.from_uss_rol()) {
          detect_sensor |= 0x400U;
        }
        if (detect_sensor_cur.from_uss_rsl()) {
          detect_sensor |= 0x800U;
        }
        if (detect_sensor_cur.from_uss_fsl()) {
          detect_sensor |= 0x01U;
        }
        if (detect_sensor != 0U) {
          auto* point_dot = radar_dot->add_point();
          point_dot->set_x(perception_obs.position().x());
          point_dot->set_y(perception_obs.position().y());
          for (uint i = 0; i < 12U; i++) {
            if (((detect_sensor >> i) & 0x01U) != 0) {
              point_dot = radar_dot->add_point();
              point_dot->set_x(points[i].x());
              point_dot->set_y(points[i].y());
              if (!detect_sensor_cur.from_uss_fusion()) {
                break;
              }
            }
          }
        }
      }
    }
  }
  // for (auto& point : car_points) {
  //   auto* point_dot = radar_dot->add_point();
  //   point_dot->set_x(point.x());
  //   point_dot->set_y(point.y());
  // }
  (*dot_properties)["borderWidth"] = "2";
  (*dot_properties)["pointRadius"] = "2.5";
  (*dot_properties)["lineTension"] = "0";
  (*dot_properties)["fill"] = "false";
  (*dot_properties)["showLine"] = "false";
  (*dot_properties)["color"] = "rgba(0, 0, 255, 1)";
}

void OpenSpaceChart::AddPartitionedPath(
    const planning_internal::Debug& debug_info,
    planning_internal::Debug* const debug_chart) {
  if (nullptr == debug_chart) {
    return;
  }

  // if open space info provider success run
  const auto& open_space_debug = debug_info.planning_data().open_space();
  const auto& chosen_path = open_space_debug.chosen_path();
  if (chosen_path.empty()) {
    return;
  }

  const auto& vehicle_state = frame_->vehicle_state();
  const auto& real_time_end_point =
      frame_->open_space_info().open_space_path_info().end_point;
  auto* chart = debug_chart->mutable_planning_data()->add_chart();
  chart->set_title("Open Space Partitioned path");
  auto* options = chart->mutable_options();
  options->mutable_x()->set_label_string("x (meter)");
  options->mutable_y()->set_label_string("y (meter)");
  options->set_sync_xy_window_size(true);
  options->set_aspect_ratio(0.9);
  options->set_legend_display(true);
#ifdef FOR_BAIDU_SIMULATION
  // baidu need roi for chart visualization
  const auto& roi_xy_boundary =
      frame_->open_space_info().open_space_path_info().roi_xy_boundary;
  if (roi_xy_boundary.size() == 4) {
    auto* roi_line = chart->add_line();
    roi_line->set_label("ROI_edge");
    std::vector<Vec2d> roi_vertex_points;
    roi_vertex_points.emplace_back(roi_xy_boundary[0], roi_xy_boundary[2]);
    roi_vertex_points.emplace_back(roi_xy_boundary[1], roi_xy_boundary[2]);
    roi_vertex_points.emplace_back(roi_xy_boundary[1], roi_xy_boundary[3]);
    roi_vertex_points.emplace_back(roi_xy_boundary[0], roi_xy_boundary[3]);
    const auto& origin =
        frame_->open_space_info().open_space_path_info().origin;
    const double rotate_angle =
        frame_->open_space_info().open_space_path_info().rotate_angle;
    for (auto& point : roi_vertex_points) {
      point.SelfRotate(rotate_angle);
      point += origin;
      auto* point_debug = roi_line->add_point();
      point_debug->set_x(point.x());
      point_debug->set_y(point.y());
    }
    auto* chosen_properties = roi_line->mutable_properties();
    (*chosen_properties)["borderWidth"] = "2";
    (*chosen_properties)["pointRadius"] = "0";
    (*chosen_properties)["lineTension"] = "0";
    (*chosen_properties)["fill"] = "false";
    (*chosen_properties)["showLine"] = "true";
    auto* new_roi_line = chart->add_line();
    *new_roi_line = *roi_line;
    new_roi_line->set_label("New_ROI_edge");
  }
#endif

  // (1) add init vehicle state
  if (open_space_debug.has_trajectory_stitching_point()) {
    auto x = open_space_debug.trajectory_stitching_point().path_point().x();
    auto y = open_space_debug.trajectory_stitching_point().path_point().y();
    auto heading =
        open_space_debug.trajectory_stitching_point().path_point().theta();
    auto* adc_shape = chart->add_car();
    adc_shape->set_x(x);
    adc_shape->set_y(y);
    adc_shape->set_heading(heading);
    adc_shape->set_color("rgba(44, 152, 225, 1)");
    adc_shape->set_label("init_ADV_state");
    adc_shape->set_hide_label_in_legend(true);
  }
  // (2) add end point state
  if (open_space_debug.has_end_point()) {
    auto x = open_space_debug.end_point().path_point().x();
    auto y = open_space_debug.end_point().path_point().y();
    auto heading = open_space_debug.end_point().path_point().theta();
    // Draw vehicle shape
    auto* adc_shape = chart->add_car();
    adc_shape->set_x(x);
    adc_shape->set_y(y);
    adc_shape->set_heading(heading);
    adc_shape->set_color("rgba(255, 0, 0, 1)");
    adc_shape->set_label("end_ADV_state");
    adc_shape->set_hide_label_in_legend(true);
  }

  // (3) add real time end point state
  {
    // Draw vehicle shape
    auto* adc_shape = chart->add_car();
    adc_shape->set_x(real_time_end_point.x());
    adc_shape->set_y(real_time_end_point.y());
    adc_shape->set_heading(real_time_end_point.theta());
    adc_shape->set_color("rgba(34, 12, 215, 1)");
    adc_shape->set_label("real_time_ADV_state");
    adc_shape->set_hide_label_in_legend(true);
  }

  // Draw vehicle state
  // auto* adc_shape = chart->add_car();
  // adc_shape->set_x(vehicle_state.x());
  // adc_shape->set_y(vehicle_state.y());
  // adc_shape->set_heading(vehicle_state.heading());
  // adc_shape->set_label("ADV");
  // adc_shape->set_color("rgba(54, 162, 25, 1)");
  const auto points = common::VehicleConfigHelper::GetAllRadarPos(
      vehicle_state.x(), vehicle_state.y(), vehicle_state.heading());
  if (!points.empty()) {
    auto* adc_spot = chart->add_line();
    adc_spot->set_label("ADV");
    auto* dot_properties = adc_spot->mutable_properties();
    for (const auto& point : points) {
      auto* point_dot = adc_spot->add_point();
      point_dot->set_x(point.x());
      point_dot->set_y(point.y());
    }
    auto* point_dot = adc_spot->add_point();
    point_dot->set_x(points[0].x());
    point_dot->set_y(points[0].y());
    (*dot_properties)["borderWidth"] = "1";
    (*dot_properties)["pointRadius"] = "0.7";
    (*dot_properties)["lineTension"] = "0";
    (*dot_properties)["fill"] = "false";
    (*dot_properties)["showLine"] = "true";
    (*dot_properties)["color"] = "rgba(54, 162, 25, 1)";
    UpdateObstacleRealTime(chart);
    auto* radar_dot = chart->add_line();
    UpdateRadarPos(radar_dot, points);

    // Draw wheel mask box
    const auto& wheel_mask_box =
        frame_->open_space_info().open_space_wheel_mask_box();
    bool consider_wheel_mask =
        frame_->open_space_info().is_consider_wheel_mask();
    if (consider_wheel_mask && wheel_mask_box.area() > 0) {
      auto* wheel_mask_line = chart->add_line();
      wheel_mask_line->set_label("wheel_mask_edge");
      const auto& wheel_mask_box_corners = wheel_mask_box.GetAllCorners();
      for (const auto& point : wheel_mask_box_corners) {
        auto* point_debug = wheel_mask_line->add_point();
        point_debug->set_x(point.x());
        point_debug->set_y(point.y());
      }
      auto* chosen_properties = wheel_mask_line->mutable_properties();
      (*chosen_properties)["borderWidth"] = "5";
      (*chosen_properties)["pointRadius"] = "0";
      (*chosen_properties)["lineTension"] = "0";
      (*chosen_properties)["fill"] = "false";
      (*chosen_properties)["showLine"] = "true";
    }
  }

  // Draw the chosen path
  auto* chosen_line = chart->add_line();
  chosen_line->set_label("Chosen");
  for (const auto& point : chosen_path) {
    auto* point_debug = chosen_line->add_point();
    point_debug->set_x(point.x());
    point_debug->set_y(point.y());
  }
  auto* chosen_properties = chosen_line->mutable_properties();
  (*chosen_properties)["borderWidth"] = "2";
  (*chosen_properties)["pointRadius"] = "0";
  (*chosen_properties)["lineTension"] = "0";
  (*chosen_properties)["fill"] = "false";
  (*chosen_properties)["showLine"] = "true";

  // Draw partitioned path
  size_t partitioned_path_label = 0;
  for (const auto& partitioned_path :
       open_space_debug.partitioned_paths().path()) {
    auto* partition_line = chart->add_line();
    partition_line->set_label(
        absl::StrCat("Partitioned ", partitioned_path_label));
    ++partitioned_path_label;
    for (const auto& point : partitioned_path.path_point()) {
      auto* point_debug = partition_line->add_point();
      point_debug->set_x(point.x());
      point_debug->set_y(point.y());
    }

    auto* partition_properties = partition_line->mutable_properties();
    (*partition_properties)["borderWidth"] = "2";
    (*partition_properties)["pointRadius"] = "0";
    (*partition_properties)["lineTension"] = "0";
    (*partition_properties)["fill"] = "false";
    (*partition_properties)["showLine"] = "true";
  }

  // Draw trajectory stitching point (line with only one point)
  auto* stitching_line = chart->add_line();
  stitching_line->set_label("TrajectoryStitchingPoint");
  auto* trajectory_stitching_point = stitching_line->add_point();
  trajectory_stitching_point->set_x(
      open_space_debug.trajectory_stitching_point().path_point().x());
  trajectory_stitching_point->set_y(
      open_space_debug.trajectory_stitching_point().path_point().y());
  // Set chartJS's dataset properties
  auto* stitching_properties = stitching_line->mutable_properties();
  (*stitching_properties)["borderWidth"] = "3";
  (*stitching_properties)["pointRadius"] = "5";
  (*stitching_properties)["lineTension"] = "0";
  (*stitching_properties)["fill"] = "true";
  (*stitching_properties)["showLine"] = "true";

  // Draw fallback trajectory compared with the partitioned and potential
  // collision_point (line with only one point)
  if (AvpSpeedPlanCollisionInfo::NO_COLLISION !=
      open_space_debug.speed_plan_collision_info().collision_type()) {
    auto* collision_line = chart->add_line();
    collision_line->set_label("FutureCollisionPoint");
    auto* future_collision_point = collision_line->add_point();
    future_collision_point->set_x(
        open_space_debug.future_collision_point().path_point().x());
    future_collision_point->set_y(
        open_space_debug.future_collision_point().path_point().y());
    // Set chartJS's dataset properties
    auto* collision_properties = collision_line->mutable_properties();
    (*collision_properties)["borderWidth"] = "3";
    (*collision_properties)["pointRadius"] = "8";
    (*collision_properties)["lineTension"] = "0";
    (*collision_properties)["fill"] = "true";
    (*stitching_properties)["showLine"] = "true";
    (*stitching_properties)["pointStyle"] = "cross";
  } else {
    auto* collision_line = chart->add_line();
    collision_line->set_label("FutureCollisionPoint");
    auto* collision_properties = collision_line->mutable_properties();
    (*collision_properties)["borderWidth"] = "3";
    (*collision_properties)["pointRadius"] = "8";
    (*collision_properties)["lineTension"] = "0";
    (*collision_properties)["fill"] = "false";
    (*stitching_properties)["showLine"] = "false";
    (*stitching_properties)["pointStyle"] = "cross";
  }

  const auto& speed_optimizer_trajectories =
      open_space_debug.speed_optimizer_trajectory().trajectory();
  if (speed_optimizer_trajectories.empty() ||
      speed_optimizer_trajectories[0].trajectory_point().empty()) {
    auto* speed_optimizer_line = chart->add_line();
    speed_optimizer_line->set_label("Speed Optimizer");
    auto* speed_optimizer_properties =
        speed_optimizer_line->mutable_properties();
    (*speed_optimizer_properties)["borderWidth"] = "3";
    (*speed_optimizer_properties)["pointRadius"] = "2";
    (*speed_optimizer_properties)["lineTension"] = "0";
    (*speed_optimizer_properties)["fill"] = "false";
    (*speed_optimizer_properties)["showLine"] = "false";
  } else {
    const auto& speed_optimizer_trajectory = speed_optimizer_trajectories[0];
    // has to define chart boundary first
    auto* speed_optimizer_line = chart->add_line();
    speed_optimizer_line->set_label("Speed Optimizer");
    for (const auto& point : speed_optimizer_trajectory.trajectory_point()) {
      auto* point_debug = speed_optimizer_line->add_point();
      point_debug->set_x(point.path_point().x());
      point_debug->set_y(point.path_point().y());
    }

    // Set chartJS's dataset properties
    auto* speed_optimizer_properties =
        speed_optimizer_line->mutable_properties();
    (*speed_optimizer_properties)["borderWidth"] = "3";
    (*speed_optimizer_properties)["pointRadius"] = "2";
    (*speed_optimizer_properties)["lineTension"] = "0";
    (*speed_optimizer_properties)["fill"] = "false";
    (*speed_optimizer_properties)["showLine"] = "true";
  }

  // set park dest region
  if (open_space_debug.dest_polygon_point_size() > 2) {
    auto* dest_region = chart->add_line();
    dest_region->set_label("dest_region");
    size_t point_size = open_space_debug.dest_polygon_point().size();
    for (size_t i = 0; i <= point_size; ++i) {
      auto* point_debug = dest_region->add_point();
      int mod_i = static_cast<int>(i % point_size);
      point_debug->set_x(open_space_debug.dest_polygon_point()[mod_i].x());
      point_debug->set_y(open_space_debug.dest_polygon_point()[mod_i].y());
    }
    auto* dest_region_properties = dest_region->mutable_properties();
    (*dest_region_properties)["borderWidth"] = "2";
    (*dest_region_properties)["pointRadius"] = "0";
    (*dest_region_properties)["lineTension"] = "0";
    (*dest_region_properties)["fill"] = "false";
    (*dest_region_properties)["showLine"] = "true";
    (*dest_region_properties)["color"] = "rgba(255, 0, 255, 1)";
  }
}

void OpenSpaceChart::AddPublishedSpeed(
    const planning::ADCTrajectory& trajectory_pb,
    planning_internal::Debug* const debug_chart) {
  if (nullptr == debug_chart) {
    return;
  }

  auto* chart = debug_chart->mutable_planning_data()->add_chart();
  chart->set_title("Speed Partition Visualization");
  auto* options = chart->mutable_options();
  options->mutable_x()->set_window_size(20.0);
  options->mutable_x()->set_label_string("time (s)");
  options->mutable_y()->set_label_string("speed (m/s)");

  auto* speed_profile = chart->add_line();
  speed_profile->set_label("Speed Profile");
  for (const auto& point : trajectory_pb.trajectory_point()) {
    auto* point_debug = speed_profile->add_point();
    point_debug->set_x(point.relative_time());
    point_debug->set_y(point.v());
  }

  // Set chartJS's dataset properties
  auto* speed_profile_properties = speed_profile->mutable_properties();
  (*speed_profile_properties)["borderWidth"] = "2";
  (*speed_profile_properties)["pointRadius"] = "0";
  (*speed_profile_properties)["lineTension"] = "0";
  (*speed_profile_properties)["fill"] = "false";
  (*speed_profile_properties)["showLine"] = "true";
}

void OpenSpaceChart::AddPublishedAcceleration(
    const planning::ADCTrajectory& trajectory_pb,
    planning_internal::Debug* const debug) {
  if (nullptr == debug) {
    return;
  }

  auto* chart = debug->mutable_planning_data()->add_chart();
  chart->set_title("Acceleration Partition Visualization");
  auto* options = chart->mutable_options();
  options->mutable_x()->set_window_size(20.0);
  options->mutable_x()->set_label_string("time (s)");
  options->mutable_y()->set_label_string("Acceleration (m/s^2)");

  auto* acceleration_profile = chart->add_line();
  acceleration_profile->set_label("Acceleration Profile");
  for (const auto& point : trajectory_pb.trajectory_point()) {
    auto* point_debug = acceleration_profile->add_point();
    point_debug->set_x(point.relative_time());
    point_debug->set_y(point.a());
  }
  // Set chartJS's dataset properties
  auto* acceleration_profile_properties =
      acceleration_profile->mutable_properties();
  (*acceleration_profile_properties)["borderWidth"] = "2";
  (*acceleration_profile_properties)["pointRadius"] = "0";
  (*acceleration_profile_properties)["lineTension"] = "0";
  (*acceleration_profile_properties)["fill"] = "false";
  (*acceleration_profile_properties)["showLine"] = "true";
}

void OpenSpaceChart::AddSpeedPlanDebugInfo(
    const planning::ADCTrajectory& trajectory_pb,
    planning_internal::Debug* const debug_chart) {
  if (nullptr == debug_chart) {
    return;
  }

  auto* chart = debug_chart->mutable_planning_data()->add_chart();
  chart->set_title("StSample Debug");
  auto* options = chart->mutable_options();
  options->mutable_x()->set_window_size(20.0);
  options->mutable_x()->set_label_string("s (m)");
  options->mutable_y()->set_label_string("speed (m/s)");
  options->mutable_x()->set_min(0.0);
  options->mutable_x()->set_step_size(0.2);
  options->mutable_y()->set_min(0.0);

  const auto& open_sapce_debug =
      trajectory_pb.debug().planning_data().open_space();
  const auto& trajectorys =
      open_sapce_debug.speed_optimizer_trajectory().trajectory();

  auto* speed_profile = chart->add_line();
  speed_profile->set_label("SV");
  for (const auto& trajectory : trajectorys) {
    for (const auto& point : trajectory.trajectory_point()) {
      auto* point_debug = speed_profile->add_point();
      point_debug->set_x(point.path_point().s());
      point_debug->set_y(std::fabs(point.v()));
    }
  }

  auto* speed_profile_properties = speed_profile->mutable_properties();
  (*speed_profile_properties)["borderWidth"] = "2";
  (*speed_profile_properties)["pointRadius"] = "0";
  (*speed_profile_properties)["lineTension"] = "0";
  (*speed_profile_properties)["fill"] = "false";
  (*speed_profile_properties)["showLine"] = "true";

  auto* speed_limits_profile = chart->add_line();
  speed_limits_profile->set_label("speed limit");
  const auto& limit_info =
      open_sapce_debug.st_sample_debug().speed_limit_points();

  for (const auto& p : limit_info) {
    auto* point_debug = speed_limits_profile->add_point();
    point_debug->set_x(p.s());
    point_debug->set_y(p.limit_v());
  }
}

void OpenSpaceChart::PopulateChartOptions(double x_min, double x_max,
                                          const std::string& x_label,
                                          double y_min, double y_max,
                                          const std::string& y_label,
                                          bool display,
                                          TL::dreamview::Chart* chart) {
  if (nullptr == chart) {
    return;
  }

  auto* options = chart->mutable_options();
  options->mutable_x()->set_min(x_min);
  options->mutable_x()->set_max(x_max);
  options->mutable_y()->set_min(y_min);
  options->mutable_y()->set_max(y_max);
  options->mutable_x()->set_label_string(x_label);
  options->mutable_y()->set_label_string(y_label);
  options->set_legend_display(display);
}

}  // namespace planning
}  // namespace TL
