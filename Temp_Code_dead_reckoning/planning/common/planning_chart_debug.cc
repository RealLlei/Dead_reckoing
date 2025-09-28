/******************************************************************************
 * Copyright 2022 The TL Authors. All Rights Reserved.
 *
 *****************************************************************************/

#include "planning/common/planning_chart_debug.h"
#include <sys/param.h>
#include <string>
#include "common/time/clock.h"
#include "planning/common/speed/speed_data.h"

namespace TL {
namespace planning {
using TL::dreamview::Chart;
using TL::planning_internal::SLFrameDebug;
using TL::planning_internal::SpeedPlan;
using TL::planning_internal::STGraphDebug;

void PopulateChartOptions(double x_min, double x_max,
                          const std::string& x_label, double y_min,
                          double y_max, const std::string& y_label,
                          bool display, Chart* chart) {
  auto* options = chart->mutable_options();
  options->mutable_x()->set_min(x_min);
  options->mutable_x()->set_max(x_max);
  options->mutable_y()->set_min(y_min);
  options->mutable_y()->set_max(y_max);
  options->mutable_x()->set_label_string(x_label);
  options->mutable_y()->set_label_string(y_label);
  options->set_legend_display(display);
}

void AddSTGraph(const STGraphDebug& st_graph, const SpeedData& speed_data,
                const TL::functionmanager::TaPilotMode& ta_pilot_mode,
                Chart* chart) {
  if (st_graph.name() == "DP_ST_SPEED_OPTIMIZER") {
    chart->set_title("Speed Heuristic");
  } else if (st_graph.name().empty()) {
    chart->set_title("Planning S-T Graph");
  } else {
    chart->set_title(st_graph.name());
  }

  if (TL::functionmanager::AVP != ta_pilot_mode) {
    PopulateChartOptions(-2.0, 10.0, "t (second)", -10.0, 200.0, "s (meter)",
                         false, chart);
  } else {
    PopulateChartOptions(-2.0, 10.0, "t (second)", -10.0, 50.0, "s (meter)",
                         false, chart);
  }

  for (const auto& boundary : st_graph.boundary()) {
    // from 'ST_BOUNDARY_TYPE_' to the end
    std::string type =
        StGraphBoundaryDebug_StBoundaryType_Name(boundary.type()).substr(17);

    auto* boundary_chart = chart->add_polygon();
    auto* properties = boundary_chart->mutable_properties();
    (*properties)["borderWidth"] = "2";
    (*properties)["pointRadius"] = "0";
    (*properties)["lineTension"] = "0";
    (*properties)["cubicInterpolationMode"] = "monotone";
    (*properties)["showLine"] = "true";
    (*properties)["showText"] = "true";
    (*properties)["fill"] = "false";

    if (type == "DRIVABLE_REGION") {
      (*properties)["color"] = "\"rgba(0, 255, 0, 0.5)\"";
    } else {
      (*properties)["color"] = "\"rgba(255, 0, 0, 0.8)\"";
    }

    boundary_chart->set_label(boundary.name() + "_" + type);
    for (const auto& point : boundary.point()) {
      auto* point_debug = boundary_chart->add_point();
      point_debug->set_x(point.t());
      point_debug->set_y(point.s());
    }
  }

  auto* speed_profile = chart->add_line();
  auto* properties = speed_profile->mutable_properties();
  (*properties)["color"] = "\"rgba(255, 255, 255, 0.5)\"";
  for (const auto& point : st_graph.speed_profile()) {
    auto* point_debug = speed_profile->add_point();
    point_debug->set_x(point.t());
    point_debug->set_y(point.s());
  }

  speed_profile = chart->add_line();
  properties = speed_profile->mutable_properties();
  (*properties)["color"] = "\"rgba(255, 255, 255, 0.5)\"";
  for (const auto& point : speed_data) {
    auto* point_debug = speed_profile->add_point();
    point_debug->set_x(point.t());
    point_debug->set_y(point.s());
  }
}

void AddSLFrame(const SLFrameDebug& sl_frame, Chart* chart) {
  chart->set_title(sl_frame.name());
  PopulateChartOptions(0.0, 80.0, "s (meter)", -8.0, 8.0, "l (meter)", false,
                       chart);
  auto* sl_line = chart->add_line();
  sl_line->set_label("SL Path");
  for (const auto& sl_point : sl_frame.sl_path()) {
    auto* point_debug = sl_line->add_point();
    point_debug->set_x(sl_point.s());
    point_debug->set_x(sl_point.l());
  }
}

void AddSpeedPlan(
    const ::google::protobuf::RepeatedPtrField<SpeedPlan>& speed_plans,
    const planning_internal::SpeedDataOptimizerDebug&
        speed_data_Optimizer_debug,
    const TL::functionmanager::TaPilotMode& ta_pilot_mode, Chart* chart) {
  chart->set_title("Speed Plan");
  if (TL::functionmanager::AVP != ta_pilot_mode) {
    PopulateChartOptions(0.0, 150.0, "s (meter)", 0.0, 40.0, "v (m/s)", true,
                         chart);
  } else {
    PopulateChartOptions(0.0, 50.0, "s (meter)", 0.0, 5.0, "v (m/s)", true,
                         chart);
  }

  for (const auto& speed_plan : speed_plans) {
    auto* line = chart->add_line();
    line->set_label(speed_plan.name());
    for (const auto& point : speed_plan.speed_point()) {
      auto* point_debug = line->add_point();
      point_debug->set_x(point.s());
      point_debug->set_y(point.v());
    }

    // Set chartJS's dataset properties
    auto* properties = line->mutable_properties();
    (*properties)["borderWidth"] = "4";
    (*properties)["pointRadius"] = "0";
    (*properties)["fill"] = "false";
    (*properties)["showLine"] = "true";
    if (speed_plan.name() == "DpStSpeedOptimizer") {
      (*properties)["color"] = "\"rgba(27, 249, 105, 0.5)\"";
    } else if (speed_plan.name() == "QpSplineStSpeedOptimizer") {
      (*properties)["color"] = "\"rgba(54, 162, 235, 1)\"";
    }
  }
  if (speed_data_Optimizer_debug.speed_limit_point_size() > 10) {
    auto* map_speed_limit_line = chart->add_line();
    auto* curvature_speed_limit_line = chart->add_line();
    auto* decision_speed_limit_line = chart->add_line();
    auto* cruise_speed_limit_line = chart->add_line();

    map_speed_limit_line->set_label("Map");
    curvature_speed_limit_line->set_label("Curvature");
    decision_speed_limit_line->set_label("Decision");
    cruise_speed_limit_line->set_label("Cruise");

    auto* map_properties = map_speed_limit_line->mutable_properties();
    (*map_properties)["borderWidth"] = "2";
    (*map_properties)["pointRadius"] = "0";
    (*map_properties)["fill"] = "false";
    (*map_properties)["showLine"] = "true";
    *curvature_speed_limit_line->mutable_properties() = *map_properties;
    *decision_speed_limit_line->mutable_properties() = *map_properties;
    *cruise_speed_limit_line->mutable_properties() = *map_properties;

    for (const auto& point : speed_data_Optimizer_debug.speed_limit_point()) {
      auto* map_speed_limit_point = map_speed_limit_line->add_point();
      map_speed_limit_point->set_x(point.s());
      map_speed_limit_point->set_y(point.map_speed_limit());

      auto* curvature_speed_limit_point =
          curvature_speed_limit_line->add_point();
      curvature_speed_limit_point->set_x(point.s());
      curvature_speed_limit_point->set_y(point.curvature_speed_limit());

      auto* decision_speed_limit_point = decision_speed_limit_line->add_point();
      decision_speed_limit_point->set_x(point.s());
      decision_speed_limit_point->set_y(point.decision_speed_limit());

      auto* cruise_speed_limit_point = cruise_speed_limit_line->add_point();
      cruise_speed_limit_point->set_x(point.s());
      cruise_speed_limit_point->set_y(point.cruise_speed_limit());
    }
  }
}

void PlanningChartDebug::ExportFailedLaneChangeSTChart(
    const ReferenceLineInfo& best_ref_line_info,
    const planning_internal::Debug& debug_info,
    const TL::functionmanager::TaPilotMode& ta_pilot_mode,
    planning_internal::Debug* debug_chart) {
  const auto& src_data = debug_info.planning_data();
  auto* dst_data = debug_chart->mutable_planning_data();
  for (const auto& st_graph : src_data.st_graph()) {
    AddSTGraph(st_graph, best_ref_line_info.speed_data(), ta_pilot_mode,
               dst_data->add_chart());
  }
}

void PlanningChartDebug::ExportOnLaneChart(
    const ReferenceLineInfo& best_ref_line_info,
    const TL::functionmanager::TaPilotMode& ta_pilot_mode,
    planning_internal::Debug* debug_chart) {
  const auto& src_data = best_ref_line_info.debug().planning_data();
  auto* dst_data = debug_chart->mutable_planning_data();
  for (const auto& st_graph : src_data.st_graph()) {
    AddSTGraph(st_graph, best_ref_line_info.speed_data(), ta_pilot_mode,
               dst_data->add_chart());
  }
  for (const auto& sl_frame : src_data.sl_frame()) {
    AddSLFrame(sl_frame, dst_data->add_chart());
  }
  AddSpeedPlan(src_data.speed_plan(), src_data.speed_data_optimizer_debug(),
               ta_pilot_mode, dst_data->add_chart());

  // Add debug information.
  if (FLAGS_enable_record_debug) {
    auto* reference_line = debug_chart->mutable_planning_data()->add_path();
    reference_line->set_name("planning_reference_line");
    const auto& reference_points =
        best_ref_line_info.reference_line().reference_points();
    double s = 0.0;
    double prev_x = 0.0;
    double prev_y = 0.0;
    bool empty_path = true;
    for (const auto& reference_point : reference_points) {
      auto* path_point = reference_line->add_path_point();
      path_point->set_x(reference_point.x());
      path_point->set_y(reference_point.y());
      path_point->set_theta(reference_point.heading());
      path_point->set_kappa(reference_point.kappa());
      path_point->set_dkappa(reference_point.dkappa());
      if (empty_path) {
        path_point->set_s(0.0);
        empty_path = false;
      } else {
        double dx = reference_point.x() - prev_x;
        double dy = reference_point.y() - prev_y;
        s += std::hypot(dx, dy);
        path_point->set_s(s);
      }
      prev_x = reference_point.x();
      prev_y = reference_point.y();
    }
    const auto& anchor_point_left_bound =
        best_ref_line_info.reference_line().GetAnchorPointLeftBound();
    RecordAnchorPointBoundDebugInfo(anchor_point_left_bound,
                                    "anchor_point_left_bound", debug_chart);
    const auto& anchor_point_right_bound =
        best_ref_line_info.reference_line().GetAnchorPointRightBound();
    RecordAnchorPointBoundDebugInfo(anchor_point_right_bound,
                                    "anchor_point_right_bound", debug_chart);
    const auto& center_line =
        best_ref_line_info.reference_line().GetCenterLine();
    RecordAnchorPointBoundDebugInfo(center_line, "center_line", debug_chart);
  }
}

void PlanningChartDebug::ExportTrajectoryDebug(const Frame* frame) {
  ADEBUG << "trajectory_start_time: " << FIXED << SETPRECISION(3)
         << TL::common::Clock::NowInSeconds();
  const auto* ref_line_info = frame->DriveReferenceLineInfo();
  ADEBUG << FIXED << SETPRECISION(3) << "planning_start_v "
         << frame->PlanningStartPoint().v();
  static int frame_seque = 0;
  uint max_num = 500;
  uint index = 0;
  ADEBUG << "reference line info";
  if ((ref_line_info == nullptr) ||
      ref_line_info->reference_line().reference_points().empty()) {
    return;
  }
  ADEBUG << "reference line info id: " << ref_line_info->Lanes().Id()
         << " tag: " << ref_line_info->reference_line().Tag();
  ADEBUG << "is_ref_change_lane " << ref_line_info->IsChangeLanePath();
  for (const auto& ref_line_point :
       ref_line_info->reference_line().reference_points()) {
    if (index > 100) {
      ADEBUG << FIXED << SETPRECISION(5) << "ref_line_point_x "
             << ref_line_point.x() << " ref_line_point_y " << ref_line_point.y()
             << " ref_line_point_kappa " << ref_line_point.kappa()
             << " ref_line_point_dkappa " << ref_line_point.dkappa()
             << " ref_line_point_heading " << ref_line_point.heading();
    }
    ++index;
  }
  ADEBUG << "frame_cnt: " << frame_seque;
  ADEBUG << "path info";
  for (const auto& path_boundaries :
       ref_line_info->GetCandidatePathBoundaries()) {
    ADEBUG << "path_boundaries_label " << path_boundaries.label();
    if (path_boundaries.label().find("regular") != std::string::npos) {
      int i = 0;
      for (const auto& path_bound : path_boundaries.boundary()) {
        ADEBUG << "path_bound_idx " << i << " path_bound_s:"
               << path_boundaries.start_s() + i * path_boundaries.delta_s()
               << " path_bound_l_min:" << std::get<0>(path_bound)
               << " path_bound_l_max:" << std::get<1>(path_bound);
        ++i;
      }
    }
  }

  for (const auto& path_data : ref_line_info->GetCandidatePathData()) {
    ADEBUG << "path_data_label: " << path_data.path_label();
  }

  ADEBUG << "trajectory info";
  ADEBUG << "Planning pb:"
         << frame->current_frame_planned_trajectory()->header().DebugString();
  index = 0;
  max_num = 100;
  for (const auto& trajectory_point :
       frame->current_frame_planned_trajectory()->trajectory_point()) {
    if (index < max_num) {
      ++index;
    } else {
      break;
    }
    ADEBUG << FIXED << SETPRECISION(5) << " t "
           << trajectory_point.relative_time() << " trajectory_s "
           << trajectory_point.path_point().s() << " trajectory_v "
           << trajectory_point.v() << " trajectory_a " << trajectory_point.a()
           << " trajectory_x " << trajectory_point.path_point().x()
           << " trajectory_y " << trajectory_point.path_point().y()
           << " trajectory_kappa " << trajectory_point.path_point().kappa()
           << " trajectory_dkappa " << trajectory_point.path_point().dkappa()
           << " trajectory_ddkappa " << trajectory_point.path_point().ddkappa()
           << " trajectory_heading " << trajectory_point.path_point().theta();
  }
  index = 0;
  max_num = 100;
  for (const auto& trajectory_frenet_point :
       ref_line_info->path_data().frenet_frame_path()) {
    if (index < max_num) {
      ++index;
    } else {
      break;
    }
    ADEBUG << FIXED << SETPRECISION(5) << "trajectory_frenet_point_s "
           << trajectory_frenet_point.s() << " trajectory_frenet_point_l "
           << trajectory_frenet_point.l() << " trajectory_frenet_point_dl "
           << trajectory_frenet_point.dl() << " trajectory_frenet_point_ddl "
           << trajectory_frenet_point.ddl();
  }
  ADEBUG << "trajectory_end";
  ++frame_seque;
}

void PlanningChartDebug::ExportReferenceLineDebug(
    const Frame* frame, planning_internal::Debug* debug) {
  if (!FLAGS_enable_record_debug) {
    return;
  }
  for (const auto& reference_line_info : frame->reference_line_info()) {
    auto* rl_debug = debug->mutable_planning_data()->add_reference_line_debug();
    rl_debug->set_id(reference_line_info.Lanes().Id());
    rl_debug->set_length(reference_line_info.reference_line().Length());
    rl_debug->set_cost(reference_line_info.Cost());
    rl_debug->set_is_change_lane_path(reference_line_info.IsChangeLanePath());
    rl_debug->set_is_drivable(reference_line_info.IsDrivable());
    rl_debug->set_is_protected(reference_line_info.GetRightOfWayStatus() ==
                               ADCTrajectory::PROTECTED);

    // store kappa and dkappa for performance evaluation
    const auto& reference_points =
        reference_line_info.reference_line().reference_points();
    double kappa_rms = 0.0;
    double dkappa_rms = 0.0;
    double kappa_max_abs = std::numeric_limits<double>::lowest();
    double dkappa_max_abs = std::numeric_limits<double>::lowest();
    for (const auto& reference_point : reference_points) {
      double kappa_sq = reference_point.kappa() * reference_point.kappa();
      double dkappa_sq = reference_point.dkappa() * reference_point.dkappa();
      kappa_rms += kappa_sq;
      dkappa_rms += dkappa_sq;
      kappa_max_abs = kappa_max_abs < kappa_sq ? kappa_sq : kappa_max_abs;
      dkappa_max_abs = dkappa_max_abs < dkappa_sq ? dkappa_sq : dkappa_max_abs;
    }
    auto reference_points_size = static_cast<double>(reference_points.size());
    kappa_rms /= reference_points_size;
    dkappa_rms /= reference_points_size;
    kappa_rms = std::sqrt(kappa_rms);
    dkappa_rms = std::sqrt(dkappa_rms);
    rl_debug->set_kappa_rms(kappa_rms);
    rl_debug->set_dkappa_rms(dkappa_rms);
    rl_debug->set_kappa_max_abs(kappa_max_abs);
    rl_debug->set_dkappa_max_abs(dkappa_max_abs);

    bool is_off_road = false;
    double minimum_boundary = std::numeric_limits<double>::infinity();

    const double adc_half_width =
        common::VehicleConfigHelper::GetConfig().vehicle_param().width() / 2.0;
    const auto& reference_line_path =
        reference_line_info.reference_line().GetMapPath();
    const auto sample_s = 0.1;
    const auto reference_line_length =
        reference_line_info.reference_line().Length();
    double average_offset = 0.0;
    double sample_count = 0.0;
    auto s = 0.0;
    static constexpr double kEpsilon = 1.0e-6;
    const auto size = static_cast<int>(std::round(
                          ((reference_line_length - kEpsilon) / sample_s))) +
                      1;
    for (int i = 0; i < size; i++) {
      s += (i * sample_s);
      double left_width = reference_line_path.GetLaneLeftWidth(s);
      double right_width = reference_line_path.GetLaneRightWidth(s);
      average_offset += 0.5 * std::abs(left_width - right_width);
      if (left_width < adc_half_width || right_width < adc_half_width) {
        is_off_road = true;
      }
      if (left_width < minimum_boundary) {
        minimum_boundary = left_width;
      }
      if (right_width < minimum_boundary) {
        minimum_boundary = right_width;
      }
      ++sample_count;
    }
    rl_debug->set_is_offroad(is_off_road);
    rl_debug->set_minimum_boundary(minimum_boundary);
    rl_debug->set_average_offset(average_offset / sample_count);
  }
}

bool PlanningChartDebug::RecordAnchorPointBoundDebugInfo(
    const std::vector<std::pair<double, double>>& anchor_point_bound,
    const std::string& name, planning_internal::Debug* const debug_chart) {
  if (debug_chart == nullptr) {
    AERROR << "debug_chart is null";
    return false;
  }
  auto* anchor_point_bound_ptr =
      debug_chart->mutable_planning_data()->add_path();
  anchor_point_bound_ptr->set_name(name);
  for (const auto& anchor_point_point : anchor_point_bound) {
    auto* path_point = anchor_point_bound_ptr->add_path_point();
    path_point->set_x(anchor_point_point.first);
    path_point->set_y(anchor_point_point.second);
    path_point->set_theta(0.0);
    path_point->set_kappa(0.0);
    path_point->set_dkappa(0.0);
    path_point->set_s(0.0);
  }
  return true;
}
}  // namespace planning
}  // namespace TL
