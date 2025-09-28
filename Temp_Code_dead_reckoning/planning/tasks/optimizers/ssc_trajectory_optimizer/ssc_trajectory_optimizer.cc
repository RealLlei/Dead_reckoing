/*
 * Copyright (c) TL Technologies Co., Ltd. 2023. All rights reserved.
 * Description:  guide_line_path_optimizer.cc
 */

#include "planning/tasks/optimizers/ssc_trajectory_optimizer/ssc_trajectory_optimizer.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

#include "common/math/box2d.h"
#include "common/math/cartesian_frenet_conversion.h"
#include "common/math/double_type.h"
#include "common/math/vec2d.h"
#include "planning/common/planning_gflags.h"
#include "planning/common/speed/speed_data.h"
#include "planning/common/trajectory1d/piecewise_jerk_trajectory1d.h"
#include "planning/math/piecewise_jerk/piecewise_jerk_path_problem.h"
#include "planning/tasks/optimizers/ssc_trajectory_optimizer/bezier_spline_generator/bezier_spline_generator.h"
#include "planning/tasks/optimizers/ssc_trajectory_optimizer/ssc_map/semantics.h"
#include "planning/proto/ipopt_pos_optimize_smoother_config.pb.h"
#include "proto/common/pnc_point.pb.h"
#include "proto/common/vehicle_config.pb.h"
#include "proto/common/vehicle_state.pb.h"

namespace TL::planning {

using TL::common::ErrorCode;
using TL::common::Status;

SscTrajectoryOptimizer::SscTrajectoryOptimizer(
    const TaskConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : PathSpeedOptimizer(config, injector),
      optimizer_config_(config.ssc_trajectory_optimizer_config()),
      ssc_map_(std::make_shared<SscMap>(
          config.ssc_trajectory_optimizer_config().map_config())) {}

common::Status SscTrajectoryOptimizer::Process(
    const ReferenceLine& reference_line,
    const common::TrajectoryPoint& init_point, PathData* const path_data,
    SpeedData* const speed_data) {
  std::vector<game_common::Vehicle> forward_trajectory;
  std::unordered_map<int, std::vector<game_common::Vehicle>>
      surround_forward_trajectories;
  if (!CombinePathAndSpeedData(&forward_trajectory,
                               &surround_forward_trajectories)) {
    const std::string msg =
        "fs_forward_trajectory_vec.size() != "
        "fs_surround_forward_trajectories_vec.size()";
    AERROR << msg;
    return Status(ErrorCode::PLANNER_CRUISING_VTSAMPLEOPTIMIZER_ERROR, msg);
  }

  // frame_->GetMockEudmOutput(&forward_trajectory,
  //                           &surround_forward_trajectories);
  // if (forward_trajectory.empty()) {
  //   const std::string msg = "forward_trajectory is empty";
  //   AERROR << msg;
  //   return Status(ErrorCode::PLANNER_CRUISING_VTSAMPLEOPTIMIZER_ERROR, msg);
  // }

  common::SLPoint sl_point;
  reference_line.XYToSL(
      {init_point.path_point().x(), init_point.path_point().y()}, &sl_point);
  ReferencePoint ref_point = reference_line.GetReferencePoint(sl_point.s());
  common::math::CartesianFrenetConverter::cartesian_to_frenet(
      sl_point.s(), ref_point.x(), ref_point.y(), ref_point.heading(),
      ref_point.kappa(), ref_point.dkappa(), init_point.path_point().x(),
      init_point.path_point().y(), init_point.v(), init_point.a(),
      init_point.path_point().theta(), init_point.path_point().kappa(),
      &init_frenet_state_.s_state, &init_frenet_state_.l_state);
  ssc_map_->ResetSscMap(init_frenet_state_);

  if (forward_trajectory.front().state().time_stamp > 0.0) {
    game_common::Vehicle vehicle;
    vehicle.mutable_state().position.set_x(init_point.path_point().x());
    vehicle.mutable_state().position.set_y(init_point.path_point().y());
    vehicle.mutable_state().angle = init_point.path_point().theta();
    vehicle.set_frenet_state(init_frenet_state_);
    forward_trajectory.insert(forward_trajectory.begin(), vehicle);
  }

  fs_forward_trajectory_vec_.clear();
  fs_forward_trajectory_vec_.emplace_back();
  auto& fs_forward_trajectory = fs_forward_trajectory_vec_.front();
  const auto& vehicle_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  for (const auto& vehicle : forward_trajectory) {
    common::math::Box2d bounding_box(
        vehicle.state().position, vehicle.state().angle, vehicle_param.length(),
        vehicle_param.width());
    fs_forward_trajectory.emplace_back(
        FsVehicle{vehicle.frenet_state(), bounding_box.GetAllCorners()});
    fs_forward_trajectory.back().frenet_state.time_stamp =
        vehicle.state().time_stamp;
  }

  fs_surround_forward_trajectories_vec_.clear();
  fs_surround_forward_trajectories_vec_.emplace_back();
  auto& fs_surround_forward_trajectories =
      fs_surround_forward_trajectories_vec_.front();
  for (const auto& surround_forward_trajectory :
       surround_forward_trajectories) {
    const auto* perception_obstacle =
        reference_line_info_->path_decision()->FindPerceptionObstacle(
            surround_forward_trajectory.first);
    if (perception_obstacle == nullptr) {
      continue;
    }

    auto& fs_vehicles =
        fs_surround_forward_trajectories[surround_forward_trajectory.first];
    for (const auto& vehicle : surround_forward_trajectory.second) {
      common::math::Box2d bounding_box(
          vehicle.state().position, vehicle.state().angle,
          perception_obstacle->length(), perception_obstacle->width());
      fs_vehicles.emplace_back(
          FsVehicle{vehicle.frenet_state(), bounding_box.GetAllCorners()});
      fs_vehicles.back().frenet_state.time_stamp = vehicle.state().time_stamp;
    }
  }

  if (fs_forward_trajectory_vec_.size() !=
      fs_surround_forward_trajectories_vec_.size()) {
    const std::string msg =
        "fs_forward_trajectory_vec.size() != "
        "fs_surround_forward_trajectories_vec.size()";
    AERROR << msg;
    return Status(ErrorCode::PLANNER_CRUISING_VTSAMPLEOPTIMIZER_ERROR, msg);
  }

  std::vector<std::array<double, 3>> obstacle_grids;
  for (std::size_t i = 0; i < fs_forward_trajectory_vec_.size(); ++i) {
    ssc_map_->ConstructSscMap(fs_surround_forward_trajectories_vec_.at(i),
                              obstacle_grids);
    if (!ssc_map_->ConstructCorridorUsingInitialTrajectory(
            ssc_map_->p_3d_grid(), fs_forward_trajectory_vec_.at(i))) {
      AERROR << "fail to construct corridor for behavior ";
      continue;
    }
  }

  if (!ssc_map_->GetFinalGlobalMetricCubesList()) {
    const std::string msg = "fail to get final corridor";
    AERROR << msg;
    return Status(ErrorCode::PLANNER_CRUISING_VTSAMPLEOPTIMIZER_ERROR, msg);
  }

  if (fs_surround_forward_trajectories_vec_.empty() || !RunOptimization() ||
      optimized_trajectories_.empty()) {
    const std::string msg = "fail to optimize trajectories";
    AERROR << msg;
    return Status(ErrorCode::PLANNER_CRUISING_VTSAMPLEOPTIMIZER_ERROR, msg);
  }

  return Status::OK();

  FrenetFramePath frenet_frame_path;
  common::FrenetFramePoint frenet_frame_point;
  common::SpeedPoint speed_point;
  std::array<double, 3> s_state{0.0, 0.0, 0.0};
  std::array<double, 3> l_state{0.0, 0.0, 0.0};
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  double kappa = 0.0;
  double v = 0.0;
  double a = 0.0;
  const auto& optimized_trajectory = optimized_trajectories_.front();
  constexpr int time_count = 71;
  std::array<double, 2> point{0.0, 0.0};
  for (int i = 0; i < time_count; ++i) {
    frenet_frame_path.emplace_back();
    double t = i * FLAGS_trajectory_time_resolution;
    optimized_trajectory.Evaluate(t, 0, &point);
    frenet_frame_point.set_s(point.at(0));
    frenet_frame_point.set_l(point.at(1));
    s_state.at(0) = point.at(0);
    l_state.at(0) = point.at(1);

    optimized_trajectory.Evaluate(t, 1, &point);
    frenet_frame_point.set_dl(point.at(1));
    s_state.at(1) = point.at(0);
    l_state.at(1) = point.at(1);

    optimized_trajectory.Evaluate(t, 2, &point);
    frenet_frame_point.set_ddl(point.at(1));
    s_state.at(2) = point.at(0);
    l_state.at(2) = point.at(1);

    const auto ref_point =
        reference_line_info_->reference_line().GetReferencePoint(
            frenet_frame_path.at(i).s());
    common::math::CartesianFrenetConverter::frenet_to_cartesian(
        frenet_frame_point.s(), ref_point.x(), ref_point.y(),
        ref_point.heading(), ref_point.kappa(), ref_point.dkappa(), s_state,
        l_state, &x, &y, &theta, &kappa, &v, &a);
    speed_point.set_t(t);
    speed_point.set_v(v);
    speed_point.set_a(a);
    speed_data->emplace_back(speed_point);
  }
  path_data->SetFrenetPath(frenet_frame_path);

  return Status::OK();
}

bool SscTrajectoryOptimizer::RunOptimization() {
  auto final_corridors = ssc_map_->final_corridor_vec();
  auto if_corridor_valid = ssc_map_->if_corridor_valid();
  if (final_corridors.empty() || if_corridor_valid.empty() ||
      if_corridor_valid.front() == 0) {
    AERROR << "cube_list is empty";
    return false;
  }

  if (final_corridors.size() != if_corridor_valid.size()) {
    AERROR << "cube_list is empty";
    return false;
  }

  optimized_trajectories_.clear();

  for (int i = 0; i < static_cast<int>(final_corridors.size()); i++) {
    if (if_corridor_valid.at(i) == 0) {
      AERROR << "[Ssc]fail: for behavior has no valid corridor";
      continue;
    }

    const auto& fs_vehicle_trajectory = fs_forward_trajectory_vec_.at(i);
    int num_states = static_cast<int>(fs_vehicle_trajectory.size());

    std::vector<std::array<double, 2>> start_constraints;
    const auto start_constraint_count = std::min(
        init_frenet_state_.s_state.size(), init_frenet_state_.l_state.size());
    for (std::size_t i = 0; i < start_constraint_count; ++i) {
      start_constraints.emplace_back(std::array<double, 2>{
          init_frenet_state_.s_state.at(i), init_frenet_state_.l_state.at(i)});
    }

    std::vector<std::array<double, 2>> end_constraints;
    const auto end_frenet_state = fs_vehicle_trajectory.back().frenet_state;
    const auto end_constraint_count = std::min(end_frenet_state.s_state.size(),
                                               end_frenet_state.l_state.size());
    for (std::size_t i = 0; i < end_constraint_count; ++i) {
      end_constraints.emplace_back(std::array<double, 2>{
          end_frenet_state.s_state.at(i), end_frenet_state.l_state.at(i)});
    }

    auto& final_corridor = final_corridors.at(i);
    if (final_corridor.empty()) {
      AERROR << "fail: corridor not valid for optimization";
      continue;
    }
    final_corridor.back().t_ub =
        fs_vehicle_trajectory.back().frenet_state.time_stamp;

    if (!CorridorFeasibilityCheck(final_corridor)) {
      AERROR << "fail: corridor not valid for optimization";
      continue;
    }

    std::vector<double> ref_stamps;
    std::vector<std::array<double, 2>> ref_points;
    for (int n = 0; n < num_states; n++) {
      ref_stamps.emplace_back(
          fs_vehicle_trajectory.at(n).frenet_state.time_stamp);
      ref_points.emplace_back(std::array<double, 2>{
          fs_vehicle_trajectory.at(n).frenet_state.s_state[0],
          fs_vehicle_trajectory.at(n).frenet_state.l_state[0]});
    }
    BezierSplineGenerator<5, 2> spline_generator;
    BezierSpline<5, 2> bezier_spline;
    if (!spline_generator.GetBezierSplineUsingCorridor(
            final_corridor, start_constraints, end_constraints, ref_stamps,
            ref_points, &bezier_spline)) {
      AERROR << "GetBezierSplineUsingCorridor failed";
      continue;
    }
    optimized_trajectories_.emplace_back(std::move(bezier_spline));
  }
  return true;
}

bool SscTrajectoryOptimizer::CorridorFeasibilityCheck(
    const std::vector<SpatioTemporalSemanticCubeNd<2>>& corridor) {
  int cube_count = static_cast<int>(corridor.size());
  if (cube_count < 1) {
    AERROR << "number of cubes not enough.";
    return false;
  }
  for (int i = 1; i < cube_count; ++i) {
    const auto& prev_cube = corridor.at(i - 1);
    const auto& next_cube = corridor.at(i);
    if (!common::math::double_type::SeemsEqual(prev_cube.t_ub,
                                               next_cube.t_lb)) {
      AERROR << "corridor not consist.";
      AERROR << "error - t: [" << prev_cube.t_lb << ", " << prev_cube.t_ub
             << "], x: [" << prev_cube.p_lb[0] << ", " << prev_cube.p_ub[0]
             << "], y: [" << prev_cube.p_lb[1] << ", " << prev_cube.p_ub[1]
             << "]";
      AERROR << "error - t: [" << next_cube.t_lb << ", " << next_cube.t_ub
             << "], x: [" << next_cube.p_lb.at(0) << ", "
             << next_cube.p_ub.at(0) << "], y: [" << next_cube.p_lb.at(1)
             << ", " << next_cube.p_ub.at(1) << "]";
      return false;
    }
  }
  return true;
}

bool SscTrajectoryOptimizer::CombinePathAndSpeedData(
    std::vector<game_common::Vehicle>* const forward_trajectory,
    std::unordered_map<int, std::vector<game_common::Vehicle>>* const
        surround_forward_trajectories) {
  if (forward_trajectory == nullptr ||
      surround_forward_trajectories == nullptr) {
    return false;
  }

  const auto& path_data = reference_line_info_->path_data();
  const auto& speed_data = reference_line_info_->speed_data();

  // const double kSparseTimeResolution = 0.2;
  const auto unit_t = 0.2;
  const auto total_size =
      static_cast<int>(std::round(speed_data.TotalTime() / unit_t));

  if (path_data.discretized_path().empty()) {
    AERROR << "path data is empty";
    return false;
  }

  if (speed_data.empty()) {
    AERROR << "speed profile is empty";
    return false;
  }

  const auto& reference_line = reference_line_info_->reference_line();

  game_common::Vehicle vehicle;
  for (int i = 0; i < total_size; ++i) {
    const auto cur_rel_time = i * unit_t;
    common::SpeedPoint speed_point;
    if (!speed_data.EvaluateByTime(cur_rel_time, &speed_point)) {
      AERROR << "Fail to get speed point with relative time " << cur_rel_time;
      return false;
    }

    if (speed_point.s() > path_data.discretized_path().Length()) {
      break;
    }
    common::PathPoint path_point =
        path_data.GetPathPointWithPathS(speed_point.s());

    vehicle.mutable_state().time_stamp = cur_rel_time;
    vehicle.mutable_state().position.set_x(path_point.x());
    vehicle.mutable_state().position.set_y(path_point.y());
    vehicle.mutable_state().angle = path_point.theta();
    vehicle.mutable_state().kappa = path_point.kappa();

    common::SLPoint sl_point;
    reference_line.XYToSL({path_point.x(), path_point.y()}, &sl_point);
    ReferencePoint ref_point = reference_line.GetReferencePoint(sl_point.s());
    common::math::CartesianFrenetConverter::cartesian_to_frenet(
        sl_point.s(), ref_point.x(), ref_point.y(), ref_point.heading(),
        ref_point.kappa(), ref_point.dkappa(), path_point.x(), path_point.y(),
        speed_point.v(), speed_point.a(), path_point.theta(),
        path_point.kappa(), &vehicle.mutable_frenet_state().s_state,
        &vehicle.mutable_frenet_state().l_state);
    vehicle.mutable_frenet_state().time_stamp = cur_rel_time;

    forward_trajectory->emplace_back(vehicle);
  }

  const auto time_diff =
      frame_->local_view().GetPredictionObstacles()->header().data_stamp() -
      (frame_->vehicle_state().timestamp() +
       frame_->PlanningStartPoint().relative_time());

  for (const auto* obstacle :
       reference_line_info_->path_decision()->obstacles().Items()) {
    if (obstacle == nullptr ||
        surround_forward_trajectories->find(obstacle->PerceptionId()) !=
            surround_forward_trajectories->end()) {
      continue;
    }

    const auto& trajectory_points = obstacle->Trajectory().trajectory_point();
    if (trajectory_points.empty()) {
      continue;
    }

    const auto last_trajectory_point =
        trajectory_points.at(trajectory_points.size() - 1);
    auto& surround_forward_trajectory =
        (*surround_forward_trajectories)[obstacle->PerceptionId()];

    for (int i = 0; i < total_size; ++i) {
      const auto cur_rel_time = i * unit_t - time_diff;
      if (common::math::double_type::DefinitelyGreater(
              cur_rel_time, last_trajectory_point.relative_time())) {
        continue;
      }
      const auto trajectory_point = obstacle->GetPointAtTime(cur_rel_time);
      const auto& path_point = trajectory_point.path_point();
      vehicle.mutable_state().time_stamp = cur_rel_time;
      vehicle.mutable_state().position.set_x(path_point.x());
      vehicle.mutable_state().position.set_y(path_point.y());
      vehicle.mutable_state().angle = path_point.theta();
      vehicle.mutable_state().kappa = path_point.kappa();

      common::SLPoint sl_point;
      reference_line.XYToSL({path_point.x(), path_point.y()}, &sl_point);
      ReferencePoint ref_point = reference_line.GetReferencePoint(sl_point.s());
      common::math::CartesianFrenetConverter::cartesian_to_frenet(
          sl_point.s(), ref_point.x(), ref_point.y(), ref_point.heading(),
          ref_point.kappa(), ref_point.dkappa(), path_point.x(), path_point.y(),
          trajectory_point.v(), trajectory_point.a(), path_point.theta(),
          path_point.kappa(), &vehicle.mutable_frenet_state().s_state,
          &vehicle.mutable_frenet_state().l_state);
      vehicle.mutable_frenet_state().time_stamp = cur_rel_time;

      surround_forward_trajectory.emplace_back(vehicle);
    }
  }
  return true;
}

}  // namespace TL::planning
