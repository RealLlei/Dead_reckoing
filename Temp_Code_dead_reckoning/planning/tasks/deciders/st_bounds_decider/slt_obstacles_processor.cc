/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file slt_obstacles_processor.cc
 **/

#include "planning/tasks/deciders/st_bounds_decider/slt_obstacles_processor.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <unordered_set>

#include "absl/strings/match.h"
#include "common/configs/vehicle_config_helper.h"
#include "common/file/log.h"
#include "common/math/double_type.h"
#include "common/math/math_utils.h"
#include "common/thread/thread_pool.h"
#include "common/util/util.h"
#include "planning/common/obstacle.h"
#include "planning/common/path/path_data.h"
#include "planning/common/planning_gflags.h"
#include "planning/common/speed/st_boundary.h"
#include "planning/common/util/common.h"

#include "planning/proto/speed_evaluator_config.pb.h"
#include "proto/planning/decision.pb.h"
#include "proto/routing/routing.pb.h"

namespace TL {
namespace planning {

using TL::common::ErrorCode;
using TL::common::Status;

constexpr double kSTBoundaryEpsilon = 1e-3;

bool SLTObstaclesProcessor::Init(const double planning_distance,
                                 const double planning_time,
                                 ReferenceLineInfo* const reference_line_info,
                                 const Frame* frame, const Frame* last_frame) {
  planning_distance_ = planning_distance;
  planning_time_ = planning_time;

  if (frame == nullptr) {
    AERROR << "frame is nullptr";
    return false;
  }

  if (reference_line_info == nullptr) {
    AERROR << "reference_line_info is nullptr";
    return false;
  }
  last_frame_ = last_frame;
  vehicle_param_ = common::VehicleConfigHelper::GetConfig().vehicle_param();
  const auto& discretized_path =
      reference_line_info->path_data().discretized_path();
  if (discretized_path.empty()) {
    AERROR << "discretized_path is empty";
    return false;
  }

  reference_line_info_ = reference_line_info;
  frame_ = frame;

  adc_reference_line_info_ = nullptr;
  for (const auto& ref_line_info : frame_->reference_line_info()) {
    if (!ref_line_info.IsChangeLanePath()) {
      adc_reference_line_info_ = &ref_line_info;
      break;
    }
  }

  return true;
}

Status SLTObstaclesProcessor::MapObstaclesToSLTBoundaries(
    PathDecision* const path_decision) {
  // Sanity checks.
  if (path_decision == nullptr) {
    const std::string msg = "path_decision is nullptr";
    AERROR << msg;
    return Status(ErrorCode::PLANNER_CRUISING_STBOUNDS_ERROR, msg);
  }
  if (planning_time_ < 0.0) {
    const std::string msg = "Negative planning time.";
    AERROR << msg;
    return Status(ErrorCode::PLANNER_CRUISING_STBOUNDS_ERROR, msg);
  }
  if (planning_distance_ < 0.0) {
    const std::string msg = "Negative planning distance.";
    AERROR << msg;
    return Status(ErrorCode::PLANNER_CRUISING_STBOUNDS_ERROR, msg);
  }
  if (reference_line_info_ == nullptr) {
    const std::string msg = "reference_line_info is nullptr";
    AERROR << msg;
    return Status(ErrorCode::PLANNER_CRUISING_STBOUNDS_ERROR, msg);
  }

  if (reference_line_info_->path_data().discretized_path().size() <= 1) {
    const std::string msg = "Number of path points is too few.";
    AERROR << msg;
    return Status(ErrorCode::PLANNER_CRUISING_STBOUNDS_ERROR, msg);
  }

  // Map obstacles into ST-graph.
  // Go through every obstacle and plot them in ST-graph.
  std::unordered_set<std::string> non_ignore_obstacles;
  std::tuple<std::string, STBoundary, Obstacle*> closest_stop_obstacle;
  std::get<0>(closest_stop_obstacle) = "NULL";
  ErrorCode code_obs = ErrorCode::OK;
  std::string error_msg;
  CalculateIfOccludedObstacle(path_decision);
  if (!FLAGS_use_multi_thread_obs_build_stbound) {
    for (const auto* obs_item_ptr : path_decision->obstacles().Items()) {
      Obstacle* obs_ptr = path_decision->Find(obs_item_ptr->Id());
      ADEBUG << "obs_id:" << obs_item_ptr->Id()
             << ", s_min:" << obs_item_ptr->PerceptionSLBoundary().start_s()
             << ", s_max:" << obs_item_ptr->PerceptionSLBoundary().end_s()
             << ", l_min:" << obs_item_ptr->PerceptionSLBoundary().start_l()
             << ", l_max:" << obs_item_ptr->PerceptionSLBoundary().end_l()
             << ", is_static:" << obs_item_ptr->IsStatic()
             << ", is_traj_empty : "
             << obs_item_ptr->Trajectory().trajectory_point().empty()
             << ", front_to_edge:" << vehicle_param_.front_edge_to_center()
             << ", back_to_edge:" << vehicle_param_.back_edge_to_center();

      if (obs_ptr == nullptr) {
        const std::string msg = "Null obstacle pointer.";
        AERROR << msg;
        return Status(ErrorCode::PLANNER_CRUISING_STBOUNDS_ERROR, msg);
      }

      std::set<SLTBoundary::BoundaryType> boundary_types;
      if (CheckIfIgnore(obs_ptr, &boundary_types)) {
        continue;
      }

      // Draw the obstacle's st-boundary.
      SLTBoundary slt_boundary;
      if (!ComputeObstacleSLTBoundary(*obs_ptr, &slt_boundary)) {
        // Obstacle doesn't appear on ST-Graph.
        continue;
      }

      slt_boundary.SetBoundaryTypes(std::move(boundary_types));

      ADEBUG << "st_boundary, obs_id:" << obs_item_ptr->Id()
             << ", s_min:" << slt_boundary.GetSTBoundary().min_s()
             << ", s_max:" << slt_boundary.GetSTBoundary().max_s()
             << ", t_min:" << slt_boundary.GetSTBoundary().min_t()
             << ", t_max:" << slt_boundary.GetSTBoundary().max_t();
      ADEBUG << "lt_boundary, obs_id:" << obs_item_ptr->Id()
             << ", l_min:" << slt_boundary.GetLTBoundary().min_s()
             << ", l_max:" << slt_boundary.GetLTBoundary().max_s()
             << ", t_min:" << slt_boundary.GetLTBoundary().min_t()
             << ", t_max:" << slt_boundary.GetLTBoundary().max_t();
      ADEBUG << "Adding " << obs_ptr->Id() << " into the ST-graph.";

      obs_ptr->SetPathSLTBoundary(std::move(slt_boundary));
    }
    return Status::OK();
  }

  auto ComputerSTBoundaryEachObs = [&](auto& obs_item_ptr) {  // Sanity checks.
    common::sub_thread_name = "_planning";
    if (obs_item_ptr == nullptr) {
      std::lock_guard<std::mutex> guard(lock_0_);
      code_obs = ErrorCode::PLANNER_CRUISING_STBOUNDS_ERROR;
      return;
    }
    Obstacle* obs_ptr = path_decision->Find(obs_item_ptr->Id());
    ADEBUG << "obs_id:" << obs_item_ptr->Id()
           << ", s_min:" << obs_item_ptr->PerceptionSLBoundary().start_s()
           << ", s_max:" << obs_item_ptr->PerceptionSLBoundary().end_s()
           << ", l_min:" << obs_item_ptr->PerceptionSLBoundary().start_l()
           << ", l_max:" << obs_item_ptr->PerceptionSLBoundary().end_l()
           << ", is_static:" << obs_item_ptr->IsStatic()
           << ", speed:" << obs_item_ptr->speed() << ", is_traj_empty:"
           << obs_item_ptr->Trajectory().trajectory_point().empty();
    if (obs_ptr == nullptr) {
      const std::string msg = "Null obstacle pointer.";
      AERROR << msg;
      {
        std::lock_guard<std::mutex> guard(lock_0_);
        error_msg = obs_item_ptr->Id();
        code_obs = ErrorCode::PLANNER_CRUISING_STBOUNDS_ERROR;
      }
      return;
    }

    std::set<SLTBoundary::BoundaryType> boundary_types;
    if (CheckIfIgnore(obs_ptr, &boundary_types)) {
      return;
    }

    // Draw the obstacle's st-boundary.
    SLTBoundary slt_boundary;
    if (!ComputeObstacleSLTBoundary(*obs_ptr, &slt_boundary)) {
      // Obstacle doesn't appear on ST-Graph.
      return;
    }

    slt_boundary.SetBoundaryTypes(std::move(boundary_types));

    ADEBUG << "st_boundary, obs_id:" << obs_item_ptr->Id()
           << ", s_min:" << slt_boundary.GetSTBoundary().min_s()
           << ", s_max:" << slt_boundary.GetSTBoundary().max_s()
           << ", t_min:" << slt_boundary.GetSTBoundary().min_t()
           << ", t_max:" << slt_boundary.GetSTBoundary().max_t();
    ADEBUG << "lt_boundary, obs_id:" << obs_item_ptr->Id()
           << ", l_min:" << slt_boundary.GetLTBoundary().min_s()
           << ", l_max:" << slt_boundary.GetLTBoundary().max_s()
           << ", t_min:" << slt_boundary.GetLTBoundary().min_t()
           << ", t_max:" << slt_boundary.GetLTBoundary().max_t();
    ADEBUG << "Adding " << obs_ptr->Id() << " into the ST-graph.";

    obs_ptr->SetPathSLTBoundary(std::move(slt_boundary));
  };

  TL::common::thread::ThreadPool::Instance()->ForEach(
      path_decision->obstacles().Items().begin(),
      path_decision->obstacles().Items().end(), ComputerSTBoundaryEachObs);

  if (code_obs == ErrorCode::PLANNER_CRUISING_STBOUNDS_ERROR) {
    return Status(ErrorCode::PLANNER_CRUISING_STBOUNDS_ERROR,
                  error_msg + " Null obstacle pointer.");
  }
  return Status::OK();
}

bool SLTObstaclesProcessor::ComputeStaticObstacleSLTBoundary(
    const Obstacle& obstacle, SLTBoundary* const slt_boundary) {
  if (reference_line_info_ == nullptr || slt_boundary == nullptr ||
      common::math::double_type::SeemsEqual(
          obstacle.PerceptionSLBoundary().start_s(),
          obstacle.PerceptionSLBoundary().end_s()) ||
      common::math::double_type::SeemsEqual(
          obstacle.PerceptionSLBoundary().start_l(),
          obstacle.PerceptionSLBoundary().end_l())) {
    return false;
  }

  std::vector<STPoint> st_lower_points;
  std::vector<STPoint> st_upper_points;
  std::vector<STPoint> lt_lower_points;
  std::vector<STPoint> lt_upper_points;

  const auto time_diff =
      frame_->local_view().GetPredictionObstacles()->header().data_stamp() -
      (frame_->vehicle_state().timestamp() +
       frame_->PlanningStartPoint().relative_time());
  st_lower_points.emplace_back(obstacle.PerceptionSLBoundary().start_s(),
                               time_diff);
  st_upper_points.emplace_back(obstacle.PerceptionSLBoundary().end_s(),
                               time_diff);
  lt_lower_points.emplace_back(obstacle.PerceptionSLBoundary().start_l(),
                               time_diff);
  lt_upper_points.emplace_back(obstacle.PerceptionSLBoundary().end_l(),
                               time_diff);
  st_lower_points.emplace_back(obstacle.PerceptionSLBoundary().start_s(),
                               planning_time_ + time_diff);
  st_upper_points.emplace_back(obstacle.PerceptionSLBoundary().end_s(),
                               planning_time_ + time_diff);
  lt_lower_points.emplace_back(obstacle.PerceptionSLBoundary().start_l(),
                               planning_time_ + time_diff);
  lt_upper_points.emplace_back(obstacle.PerceptionSLBoundary().end_l(),
                               planning_time_ + time_diff);
  auto st_boundary =
      STBoundary::CreateInstanceAccurate(st_lower_points, st_upper_points);
  auto lt_boundary =
      STBoundary::CreateInstanceAccurate(lt_lower_points, lt_upper_points);
  st_boundary.set_id(obstacle.Id());
  lt_boundary.set_id(obstacle.Id());
  slt_boundary->SetSTBoundary(std::move(st_boundary));
  slt_boundary->SetLTBoundary(std::move(lt_boundary));

  std::vector<STPoint> ds_points;
  std::vector<STPoint> dl_points;
  std::vector<STPoint> theta_points;
  ds_points.emplace_back(0.0, 0.0);
  dl_points.emplace_back(0.0, 0.0);
  theta_points.emplace_back(0.0, 0.0);
  ds_points.emplace_back(0.0, planning_time_ + time_diff);
  dl_points.emplace_back(0.0, planning_time_ + time_diff);
  theta_points.emplace_back(0.0, planning_time_ + time_diff);

  slt_boundary->SetDsPoints(std::move(ds_points));
  slt_boundary->SetDlPoints(std::move(dl_points));
  slt_boundary->SetThetaPoints(std::move(theta_points));

  return (!st_lower_points.empty() && !st_upper_points.empty() &&
          !lt_lower_points.empty() && !lt_upper_points.empty());
}

bool SLTObstaclesProcessor::ComputeDynamicObstacleSLTBoundary(
    const Obstacle& obstacle, SLTBoundary* const slt_boundary) {
  if (reference_line_info_ == nullptr || slt_boundary == nullptr) {
    return false;
  }

  const auto& obs_trajectory = obstacle.Trajectory();
  const auto time_diff =
      frame_->local_view().GetPredictionObstacles()->header().data_stamp() -
      (frame_->vehicle_state().timestamp() +
       frame_->PlanningStartPoint().relative_time());

  // Processing a dynamic obstacle.
  // Go through every occurrence of the obstacle at all timesteps, and
  // figure out the overlapping s-max and s-min one by one.
  ADEBUG << " Processing a dynamic obstacle.";

  const auto& trajectory_points = obs_trajectory.trajectory_point();
  const auto& trajectory_envelopes = obstacle.GetTrajectoryEnvelope();

  std::vector<STPoint> st_lower_points;
  std::vector<STPoint> st_upper_points;
  std::vector<STPoint> lt_lower_points;
  std::vector<STPoint> lt_upper_points;
  std::vector<STPoint> ds_points;
  std::vector<STPoint> dl_points;
  std::vector<STPoint> theta_points;
  st_lower_points.reserve(trajectory_points.size());
  st_upper_points.reserve(trajectory_points.size());
  lt_lower_points.reserve(trajectory_points.size());
  lt_upper_points.reserve(trajectory_points.size());
  ds_points.reserve(trajectory_points.size());
  dl_points.reserve(trajectory_points.size());

  for (int i = 0; i < trajectory_points.size(); ++i) {
    const auto& obs_traj_pt = trajectory_points.at(i);
    const auto& envelope = trajectory_envelopes.at(i);
    // TODO(jiacheng): Currently, if the obstacle overlaps with ADC at
    // disjoint segments (happens very rarely), we merge them into one.
    // In the future, this could be considered in greater details rather
    // than being approximated.
    const auto time = obs_traj_pt.relative_time() + time_diff;
    const auto s_lower =
        std::min({envelope.low_left_p.s(), envelope.low_right_p.s(),
                  envelope.upper_left_p.s(), envelope.upper_right_p.s()});
    const auto s_upper =
        std::max({envelope.low_left_p.s(), envelope.low_right_p.s(),
                  envelope.upper_left_p.s(), envelope.upper_right_p.s(),
                  s_lower + kSTBoundaryEpsilon});
    st_lower_points.emplace_back(s_lower, time);
    st_upper_points.emplace_back(s_upper, time);

    const auto l_lower =
        std::min({envelope.low_left_p.l(), envelope.low_right_p.l(),
                  envelope.upper_left_p.l(), envelope.upper_right_p.l()});
    const auto l_upper =
        std::max({envelope.low_left_p.l(), envelope.low_right_p.l(),
                  envelope.upper_left_p.l(), envelope.upper_right_p.l(),
                  l_lower + kSTBoundaryEpsilon});
    lt_lower_points.emplace_back(l_lower, time);
    lt_upper_points.emplace_back(l_upper, time);

    const auto reference_point =
        reference_line_info_->reference_line().GetReferencePoint(
            envelope.center_p.s());
    const auto angle = ((i == 0 && trajectory_points.size() > 1)
                            ? trajectory_points.at(i + 1).path_point().theta()
                            : obs_traj_pt.path_point().theta()) -
                       reference_point.heading();
    ds_points.emplace_back(obs_traj_pt.v() * cos(angle), time);
    dl_points.emplace_back(obs_traj_pt.v() * sin(angle), time);
    theta_points.emplace_back(angle, time);

    ADEBUG << "t:" << time << ", s_lower:" << s_lower << ", s_upper:" << s_upper
           << ", l_lower:" << l_lower << ", l_upper:" << l_upper;
  }

  const auto reference_line_length =
      reference_line_info_->reference_line().Length();
  const int point_size = static_cast<int>(st_lower_points.size());

  for (int i = 0; i < point_size; ++i) {
    if (st_lower_points.at(i).s() > 0 &&
        st_upper_points.at(i).s() < reference_line_length) {
      const auto ds = ds_points.at(i).s();
      const auto dl = dl_points.at(i).s();
      for (int j = 0; j < i; ++j) {
        const auto dt = st_lower_points.at(j).t() - st_lower_points.at(i).t();
        st_lower_points.at(j).set_s(st_lower_points.at(i).s() + dt * ds);
        st_upper_points.at(j).set_s(st_upper_points.at(i).s() + dt * ds);
        lt_lower_points.at(j).set_s(lt_lower_points.at(i).s() + dt * dl);
        lt_upper_points.at(j).set_s(lt_upper_points.at(i).s() + dt * dl);
      }
      break;
    }
  }

  for (int i = point_size - 1; i >= 0; --i) {
    if (st_lower_points.at(i).s() > 0 &&
        st_upper_points.at(i).s() < reference_line_length) {
      const auto ds = ds_points.at(i).s();
      const auto dl = dl_points.at(i).s();
      for (int j = i + 1; j < point_size; ++j) {
        const auto dt = st_lower_points.at(j).t() - st_lower_points.at(i).t();
        st_lower_points.at(j).set_s(st_lower_points.at(i).s() + dt * ds);
        st_upper_points.at(j).set_s(st_upper_points.at(i).s() + dt * ds);
        lt_lower_points.at(j).set_s(lt_lower_points.at(i).s() + dt * dl);
        lt_upper_points.at(j).set_s(lt_upper_points.at(i).s() + dt * dl);
      }
      break;
    }
  }

  if (st_lower_points.size() == 1) {
    st_lower_points.emplace_back(st_lower_points.front().s(),
                                 st_lower_points.front().t() + 0.1);
  }
  if (st_upper_points.size() == 1) {
    st_upper_points.emplace_back(st_upper_points.front().s(),
                                 st_upper_points.front().t() + 0.1);
  }
  if (lt_lower_points.size() == 1) {
    lt_lower_points.emplace_back(lt_lower_points.front().s(),
                                 lt_lower_points.front().t() + 0.1);
  }
  if (lt_upper_points.size() == 1) {
    lt_upper_points.emplace_back(lt_upper_points.front().s(),
                                 lt_upper_points.front().t() + 0.1);
  }
  if (ds_points.size() == 1) {
    ds_points.emplace_back(ds_points.front().s(), ds_points.front().t() + 0.1);
  }
  if (dl_points.size() == 1) {
    dl_points.emplace_back(dl_points.front().s(), dl_points.front().t() + 0.1);
  }
  if (theta_points.size() == 1) {
    theta_points.emplace_back(theta_points.front().s(),
                              theta_points.front().t() + 0.1);
  }

  auto st_boundary =
      STBoundary::CreateInstanceAccurate(st_lower_points, st_upper_points);
  auto lt_boundary =
      STBoundary::CreateInstanceAccurate(lt_lower_points, lt_upper_points);
  st_boundary.set_id(obstacle.Id());
  lt_boundary.set_id(obstacle.Id());
  slt_boundary->SetSTBoundary(std::move(st_boundary));
  slt_boundary->SetLTBoundary(std::move(lt_boundary));
  slt_boundary->SetDsPoints(std::move(ds_points));
  slt_boundary->SetDlPoints(std::move(dl_points));
  slt_boundary->SetThetaPoints(std::move(theta_points));

  return (!st_lower_points.empty() && !st_upper_points.empty() &&
          !lt_lower_points.empty() && !lt_upper_points.empty());
}

bool SLTObstaclesProcessor::ComputeStaticObstacleSLTBoundaryForPath(
    const Obstacle& obstacle, SLTBoundary* const slt_boundary) {
  if (reference_line_info_ == nullptr || slt_boundary == nullptr ||
      common::math::double_type::SeemsEqual(
          obstacle.PerceptionSLBoundary().start_s(),
          obstacle.PerceptionSLBoundary().end_s()) ||
      common::math::double_type::SeemsEqual(
          obstacle.PerceptionSLBoundary().start_l(),
          obstacle.PerceptionSLBoundary().end_l())) {
    return false;
  }

  std::vector<STPoint> st_lower_points;
  std::vector<STPoint> st_upper_points;
  std::vector<STPoint> lt_lower_points;
  std::vector<STPoint> lt_upper_points;

  const auto time_diff =
      frame_->local_view().GetPredictionObstacles()->header().data_stamp() -
      (frame_->vehicle_state().timestamp() +
       frame_->PlanningStartPoint().relative_time());
  const auto& bounding_box = obstacle.PerceptionBoundingBox();

  auto s_lower = std::numeric_limits<double>::max();
  auto s_upper = std::numeric_limits<double>::lowest();
  auto l_lower = std::numeric_limits<double>::max();
  auto l_upper = std::numeric_limits<double>::lowest();

  const auto& discretized_path =
      reference_line_info_->path_data().discretized_path();

  double s = 0.0;
  double l = 0.0;
  int index_min = -1;
  double distance = 0.0;
  double radius = bounding_box.length() * 2.0;
  for (const auto& corner : bounding_box.GetAllCorners()) {
    discretized_path.GetProjection(corner, &s, &l, &distance, &index_min,
                                   radius, index_min);
    s_lower = fmin(s_lower, s);
    s_upper = fmax(s_upper, s);
    l_lower = fmin(l_lower, l);
    l_upper = fmax(l_upper, l);
  }
  s_upper = fmax(s_upper, s_lower + kSTBoundaryEpsilon);
  l_upper = fmax(l_upper, l_lower + kSTBoundaryEpsilon);

  st_lower_points.emplace_back(s_lower, time_diff);
  st_upper_points.emplace_back(s_upper, time_diff);
  lt_lower_points.emplace_back(l_lower, time_diff);
  lt_upper_points.emplace_back(l_upper, time_diff);
  st_lower_points.emplace_back(s_lower, planning_time_ + time_diff);
  st_upper_points.emplace_back(s_upper, planning_time_ + time_diff);
  lt_lower_points.emplace_back(l_lower, planning_time_ + time_diff);
  lt_upper_points.emplace_back(l_upper, planning_time_ + time_diff);
  auto st_boundary =
      STBoundary::CreateInstanceAccurate(st_lower_points, st_upper_points);
  auto lt_boundary =
      STBoundary::CreateInstanceAccurate(lt_lower_points, lt_upper_points);
  st_boundary.set_id(obstacle.Id());
  lt_boundary.set_id(obstacle.Id());
  slt_boundary->SetSTBoundary(std::move(st_boundary));
  slt_boundary->SetLTBoundary(std::move(lt_boundary));

  std::vector<STPoint> ds_points;
  std::vector<STPoint> dl_points;
  std::vector<STPoint> theta_points;
  ds_points.emplace_back(0.0, 0.0);
  dl_points.emplace_back(0.0, 0.0);
  theta_points.emplace_back(0.0, 0.0);
  ds_points.emplace_back(0.0, planning_time_ + time_diff);
  dl_points.emplace_back(0.0, planning_time_ + time_diff);
  theta_points.emplace_back(0.0, planning_time_ + time_diff);

  slt_boundary->SetDsPoints(std::move(ds_points));
  slt_boundary->SetDlPoints(std::move(dl_points));
  slt_boundary->SetThetaPoints(std::move(theta_points));

  return (!st_lower_points.empty() && !st_upper_points.empty() &&
          !lt_lower_points.empty() && !lt_upper_points.empty());
}

bool SLTObstaclesProcessor::ComputeDynamicObstacleSLTBoundaryForPath(
    const Obstacle& obstacle, SLTBoundary* const slt_boundary) {
  if (reference_line_info_ == nullptr || slt_boundary == nullptr) {
    return false;
  }

  const auto& obs_trajectory = obstacle.Trajectory();
  const auto time_diff =
      frame_->local_view().GetPredictionObstacles()->header().data_stamp() -
      (frame_->vehicle_state().timestamp() +
       frame_->PlanningStartPoint().relative_time());

  const auto& trajectory_points = obs_trajectory.trajectory_point();
  const auto& trajectory_bounding_boxs = obstacle.GetTrajectoryBoundingBox();

  std::vector<STPoint> st_lower_points;
  std::vector<STPoint> st_upper_points;
  std::vector<STPoint> lt_lower_points;
  std::vector<STPoint> lt_upper_points;
  std::vector<STPoint> ds_points;
  std::vector<STPoint> dl_points;
  std::vector<STPoint> theta_points;
  st_lower_points.reserve(trajectory_points.size());
  st_upper_points.reserve(trajectory_points.size());
  lt_lower_points.reserve(trajectory_points.size());
  lt_upper_points.reserve(trajectory_points.size());
  ds_points.reserve(trajectory_points.size());
  dl_points.reserve(trajectory_points.size());

  const auto& discretized_path =
      reference_line_info_->path_data().discretized_path();
  for (int i = 0; i < trajectory_points.size(); ++i) {
    const auto& obs_traj_pt = trajectory_points.at(i);
    const auto& bounding_box = trajectory_bounding_boxs.at(i);
    const auto time = obs_traj_pt.relative_time() + time_diff;

    auto s_lower = std::numeric_limits<double>::max();
    auto s_upper = std::numeric_limits<double>::lowest();
    auto l_lower = std::numeric_limits<double>::max();
    auto l_upper = std::numeric_limits<double>::lowest();

    double s = 0.0;
    double l = 0.0;
    int index_min = -1;
    double distance = 0.0;
    double radius = bounding_box.length() * 2.0;
    if (!discretized_path.GetProjection(bounding_box.center(), &s, &l,
                                        &distance, &index_min, radius,
                                        index_min)) {
      continue;
    }

    const auto path_point = discretized_path.Evaluate(s);
    auto angle = ((i == 0 && trajectory_points.size() > 1)
                      ? trajectory_points.at(i + 1).path_point().theta()
                      : obs_traj_pt.path_point().theta()) -
                 path_point.theta();
    angle = TL::common::math ::NormalizeAngle(angle);
    ds_points.emplace_back(obs_traj_pt.v() * cos(angle), time);
    dl_points.emplace_back(obs_traj_pt.v() * sin(angle), time);
    theta_points.emplace_back(angle, time);

    for (const auto& corner : bounding_box.GetAllCorners()) {
      discretized_path.GetProjection(corner, &s, &l, &distance, &index_min,
                                     radius, index_min);
      s_lower = fmin(s_lower, s);
      s_upper = fmax(s_upper, s);
      l_lower = fmin(l_lower, l);
      l_upper = fmax(l_upper, l);
    }
    s_upper = fmax(s_upper, s_lower + kSTBoundaryEpsilon);
    l_upper = fmax(l_upper, l_lower + kSTBoundaryEpsilon);

    st_lower_points.emplace_back(s_lower, time);
    st_upper_points.emplace_back(s_upper, time);
    lt_lower_points.emplace_back(l_lower, time);
    lt_upper_points.emplace_back(l_upper, time);
  }

  if (st_lower_points.size() == 1) {
    st_lower_points.emplace_back(st_lower_points.front().s(),
                                 st_lower_points.front().t() + 0.1);
  }
  if (st_upper_points.size() == 1) {
    st_upper_points.emplace_back(st_upper_points.front().s(),
                                 st_upper_points.front().t() + 0.1);
  }
  if (lt_lower_points.size() == 1) {
    lt_lower_points.emplace_back(lt_lower_points.front().s(),
                                 lt_lower_points.front().t() + 0.1);
  }
  if (lt_upper_points.size() == 1) {
    lt_upper_points.emplace_back(lt_upper_points.front().s(),
                                 lt_upper_points.front().t() + 0.1);
  }
  if (ds_points.size() == 1) {
    ds_points.emplace_back(ds_points.front().s(), ds_points.front().t() + 0.1);
  }
  if (dl_points.size() == 1) {
    dl_points.emplace_back(dl_points.front().s(), dl_points.front().t() + 0.1);
  }
  if (theta_points.size() == 1) {
    theta_points.emplace_back(theta_points.front().s(),
                              theta_points.front().t() + 0.1);
  }

  auto st_boundary =
      STBoundary::CreateInstanceAccurate(st_lower_points, st_upper_points);
  auto lt_boundary =
      STBoundary::CreateInstanceAccurate(lt_lower_points, lt_upper_points);
  st_boundary.set_id(obstacle.Id());
  lt_boundary.set_id(obstacle.Id());
  slt_boundary->SetSTBoundary(std::move(st_boundary));
  slt_boundary->SetLTBoundary(std::move(lt_boundary));
  slt_boundary->SetDsPoints(std::move(ds_points));
  slt_boundary->SetDlPoints(std::move(dl_points));
  slt_boundary->SetThetaPoints(std::move(theta_points));

  return (!st_lower_points.empty() && !st_upper_points.empty() &&
          !lt_lower_points.empty() && !lt_upper_points.empty());
}

bool SLTObstaclesProcessor::ComputeCruiseTargetSLTBoundaryForPath(
    const Obstacle& obstacle, SLTBoundary* slt_boundary) {
  if (frame_ == nullptr || reference_line_info_ == nullptr ||
      slt_boundary == nullptr) {
    return false;
  }

  const auto time_diff =
      frame_->local_view().GetPredictionObstacles()->header().data_stamp() -
      (frame_->vehicle_state().timestamp() +
       frame_->PlanningStartPoint().relative_time());

  const auto& trajectory_points = obstacle.Trajectory().trajectory_point();
  const auto& trajectory_bounding_boxs = obstacle.GetTrajectoryBoundingBox();
  if (trajectory_points.empty() || trajectory_bounding_boxs.empty()) {
    return false;
  }

  std::vector<STPoint> st_lower_points;
  std::vector<STPoint> st_upper_points;
  std::vector<STPoint> lt_lower_points;
  std::vector<STPoint> lt_upper_points;
  std::vector<STPoint> ds_points;
  std::vector<STPoint> dl_points;
  std::vector<STPoint> theta_points;
  st_lower_points.reserve(trajectory_points.size());
  st_upper_points.reserve(trajectory_points.size());
  lt_lower_points.reserve(trajectory_points.size());
  lt_upper_points.reserve(trajectory_points.size());
  ds_points.reserve(trajectory_points.size());
  dl_points.reserve(trajectory_points.size());

  const auto& discretized_path =
      reference_line_info_->path_data().discretized_path();
  const auto& first_trajectory_point = trajectory_points.at(0);
  const auto& bounding_box = trajectory_bounding_boxs.at(0);
  auto time = first_trajectory_point.relative_time() + time_diff;

  auto s_lower = std::numeric_limits<double>::max();
  auto s_upper = std::numeric_limits<double>::lowest();
  auto l_upper = obstacle.Perception().width() * 0.5;
  auto l_lower = -l_upper;

  double s = 0.0;
  double l = 0.0;
  int index_min = -1;
  double distance = 0.0;
  double radius = bounding_box.length() * 2.0;
  if (!discretized_path.GetProjection(bounding_box.center(), &s, &l, &distance,
                                      &index_min, radius, index_min)) {
    return false;
  }

  const auto cos_theta = cos(obstacle.PerceptionBoundingBox().heading() -
                             discretized_path.Evaluate(s).theta());
  for (const auto& corner : bounding_box.GetAllCorners()) {
    discretized_path.GetProjection(corner, &s, &l, &distance, &index_min,
                                   radius, index_min);
    s_lower = fmin(s_lower, s);
    s_upper = fmax(s_upper, s);
  }
  s_upper = fmax(s_upper, s_lower + kSTBoundaryEpsilon);
  l_upper = fmax(l_upper, l_lower + kSTBoundaryEpsilon);

  st_lower_points.emplace_back(s_lower, time);
  st_upper_points.emplace_back(s_upper, time);
  lt_lower_points.emplace_back(l_lower, time);
  lt_upper_points.emplace_back(l_upper, time);
  ds_points.emplace_back(first_trajectory_point.v() * cos_theta, time);
  dl_points.emplace_back(0.0, time);
  theta_points.emplace_back(0.0, time);

  constexpr auto kMaxTimeLength = 14.0;
  constexpr auto kEndTime = 7.0;
  constexpr auto kTimeInterval = 0.1;
  const auto total_time = fmin(kMaxTimeLength, kEndTime - time);
  std::vector<std::vector<double>> vec_vec_state;
  if ((common::math::double_type::DefinitelyGreaterEqual(
           first_trajectory_point.a(), 0.0) &&
       !TL::planning::util::GetStateAtMinJerk(
           first_trajectory_point.v(), first_trajectory_point.a(), 0.0,
           std::numeric_limits<double>::max(), 0.0, -1.0, total_time,
           kTimeInterval, 0.0, &vec_vec_state)) ||
      (common::math::double_type::DefinitelyLess(first_trajectory_point.a(),
                                                 0.0) &&
       !TL::planning::util::GetStateAtMaxJerk(
           first_trajectory_point.v(), first_trajectory_point.a(), 0.0,
           std::numeric_limits<double>::max(), 0.0, 1.0, total_time,
           kTimeInterval, 0.0, &vec_vec_state)) ||
      vec_vec_state.size() < 4) {
    return false;
  }

  const auto& vec_t = vec_vec_state.at(0);
  const auto& vec_s = vec_vec_state.at(1);
  const auto& vec_v = vec_vec_state.at(2);
  if (vec_t.size() != vec_s.size() || vec_t.size() != vec_v.size() ||
      vec_t.empty()) {
    return false;
  }

  for (int i = 1; i < vec_t.size(); ++i) {
    const auto delta_s = vec_s.at(i) * cos_theta;
    time = first_trajectory_point.relative_time() + vec_t.at(i) + time_diff;
    st_lower_points.emplace_back(s_lower + delta_s, time);
    st_upper_points.emplace_back(s_upper + delta_s, time);
    lt_lower_points.emplace_back(l_lower, time);
    lt_upper_points.emplace_back(l_upper, time);
    ds_points.emplace_back(vec_v.at(i) * cos_theta, time);
    dl_points.emplace_back(0.0, time);
    theta_points.emplace_back(0.0, time);
  }

  if (st_lower_points.size() == 1) {
    st_lower_points.emplace_back(st_lower_points.front().s(),
                                 st_lower_points.front().t() + 0.1);
  }
  if (st_upper_points.size() == 1) {
    st_upper_points.emplace_back(st_upper_points.front().s(),
                                 st_upper_points.front().t() + 0.1);
  }
  if (lt_lower_points.size() == 1) {
    lt_lower_points.emplace_back(lt_lower_points.front().s(),
                                 lt_lower_points.front().t() + 0.1);
  }
  if (lt_upper_points.size() == 1) {
    lt_upper_points.emplace_back(lt_upper_points.front().s(),
                                 lt_upper_points.front().t() + 0.1);
  }
  if (ds_points.size() == 1) {
    ds_points.emplace_back(ds_points.front().s(), ds_points.front().t() + 0.1);
  }
  if (dl_points.size() == 1) {
    dl_points.emplace_back(dl_points.front().s(), dl_points.front().t() + 0.1);
  }
  if (theta_points.size() == 1) {
    theta_points.emplace_back(theta_points.front().s(),
                              theta_points.front().t() + 0.1);
  }

  auto st_boundary =
      STBoundary::CreateInstanceAccurate(st_lower_points, st_upper_points);
  auto lt_boundary =
      STBoundary::CreateInstanceAccurate(lt_lower_points, lt_upper_points);
  st_boundary.set_id(obstacle.Id());
  lt_boundary.set_id(obstacle.Id());
  slt_boundary->SetSTBoundary(std::move(st_boundary));
  slt_boundary->SetLTBoundary(std::move(lt_boundary));
  slt_boundary->SetDsPoints(std::move(ds_points));
  slt_boundary->SetDlPoints(std::move(dl_points));
  slt_boundary->SetThetaPoints(std::move(theta_points));

  return (!st_lower_points.empty() && !st_upper_points.empty() &&
          !lt_lower_points.empty() && !lt_upper_points.empty());
}

bool SLTObstaclesProcessor::ComputeObstacleSLTBoundary(
    const Obstacle& obstacle, SLTBoundary* const slt_boundary) {
  if (obstacle.IsStatic()) {
    return ComputeStaticObstacleSLTBoundaryForPath(obstacle, slt_boundary);
  }
  if (frame_ != nullptr && frame_->local_view().HasFunctionManagerOut() &&
      frame_->local_view().GetFunctionManagerOut() != nullptr &&
      frame_->local_view().GetFunctionManagerOut()->fsm_state() ==
          functionmanager::MachineStateType::PERCEPTION_TYPE &&
      frame_->local_view().GetFunctionManagerOut()->perception_sub_state() ==
          functionmanager::PerceptionSubState::CRUISE_TYPE &&
      (!frame_->local_view().GetCruiseTargetId().empty() &&
       std::find(frame_->local_view().GetCruiseTargetId().begin(),
                 frame_->local_view().GetCruiseTargetId().end(),
                 obstacle.PerceptionId()) !=
           frame_->local_view().GetCruiseTargetId().end())) {
    return ComputeCruiseTargetSLTBoundaryForPath(obstacle, slt_boundary);
  }
  return ComputeDynamicObstacleSLTBoundaryForPath(obstacle, slt_boundary);
}

bool SLTObstaclesProcessor::CheckIfIgnore(
    Obstacle* obstacle,
    std::set<SLTBoundary::BoundaryType>* const boundary_types) {
  if (boundary_types == nullptr || obstacle == nullptr ||
      (obstacle->LongitudinalDecision().has_ignore()) ||
      obstacle->LongitudinalDecision().has_stop()) {
    return true;
  }

  if (!obstacle->path_st_boundary().IsEmpty()) {
    boundary_types->insert(SLTBoundary::BoundaryType::SAFE_CAUTION);
    ADEBUG << "obstacle safe caution, not ignore";
  } else {
    const auto& decisions = obstacle->decisions();
    const auto& decider_tags = obstacle->decider_tags();
    const auto decision_size = std::min(decisions.size(), decider_tags.size());
    for (std::size_t i = 0; i < decision_size; ++i) {
      if (decisions.at(i).has_caution() &&
          decider_tags.at(i) == "st_obstacle_processor") {
        boundary_types->insert(SLTBoundary::BoundaryType::SAFE_CAUTION);
        ADEBUG << "obstacle safe caution, not ignore";
        break;
      }
    }
  }

  if (CheckIfNudge(*obstacle)) {
    boundary_types->insert(SLTBoundary::BoundaryType::NUDGE_CAUTION);
    ADEBUG << "obstacle nudge caution, not ignore";
  }

  if (obstacle->path_st_boundary().IsEmpty() &&
      CheckIfNudgeSpeedLimit(obstacle)) {
    boundary_types->insert(SLTBoundary::BoundaryType::SPEED_LIMIT_CAUTION);
    ADEBUG << "obstacle speed limit caution, not ignore";
  }

  return boundary_types->empty();
}

bool SLTObstaclesProcessor::CheckIfNudge(const Obstacle& obstacle) {
  if (!FLAGS_enable_lon_nudge_parallel_obstacle ||
      !obstacle.path_st_boundary().IsEmpty()) {
    return false;
  }

  const auto& decisions = obstacle.decisions();
  const auto& decider_tags = obstacle.decider_tags();
  if (decisions.empty() || decisions.size() != decider_tags.size()) {
    return false;
  }

  for (std::size_t i = 0; i < decisions.size(); ++i) {
    if ((decisions.at(i).has_overtake() &&
         decider_tags.at(i) == "obstacles_decider/-overtake") ||
        (decisions.at(i).has_yield() &&
         decider_tags.at(i) == "obstacles_decider/-yield")) {
      return true;
    }
  }

  return false;
}

bool SLTObstaclesProcessor::CheckIfNudgeSpeedLimit(Obstacle* obstacle) const {
  if (reference_line_info_ == nullptr) {
    return false;
  }
  const auto& path_data = reference_line_info_->path_data();
  if (path_data.Empty() || obstacle->GetIsOccludedObstacle() ||
      obstacle->IsCone() || obstacle == nullptr ||
      CheckIfIgnoreNudgeSpeedLimit(obstacle)) {
    return false;
  }

  bool is_last_nudgelimit_obstacle = true;
  if (last_frame_ != nullptr &&
      last_frame_->DriveReferenceLineInfo() != nullptr) {
    const auto& last_path_decision =
        last_frame_->DriveReferenceLineInfo()->path_decision();
    for (const auto* const last_obs_item_ptr :
         last_path_decision.obstacles().Items()) {
      const auto* last_obs_ptr =
          last_path_decision.Find(last_obs_item_ptr->Id());
      if (last_obs_ptr == nullptr ||
          last_obs_item_ptr->PerceptionId() != obstacle->PerceptionId()) {
        continue;
      }
      is_last_nudgelimit_obstacle = last_obs_ptr->GetIsNudgeLimitObstacle();
    }
  }
  const double nudge_limit_static_buffer = (obstacle->IsStatic()) ? 1.05 : 0.8;
  const double nudge_limit_l_space = (is_last_nudgelimit_obstacle)
                                         ? nudge_limit_static_buffer + 0.3
                                         : nudge_limit_static_buffer;

  const auto& path_envelope = path_data.GetPathEnvelope();
  auto adc_frenet_l_upper = path_envelope.max_ref_l + nudge_limit_l_space;
  auto adc_frenet_l_lower = path_envelope.min_ref_l - nudge_limit_l_space;
  const double obstacle_start_s = obstacle->PerceptionSLBoundary().start_s();
  const double obstacle_end_s = obstacle->PerceptionSLBoundary().end_s();
  double start_s_lane_left_width = 0.0;
  double start_s_lane_right_width = 0.0;
  double end_s_lane_left_width = 0.0;
  double end_s_lane_right_width = 0.0;
  double obstacle_start_l_to_lane_distance = 1.0;
  double obstacle_end_l_to_lane_distance = 1.0;
  if (reference_line_info_->reference_line().GetLaneWidth(
          obstacle_start_s, &start_s_lane_left_width,
          &start_s_lane_right_width) &&
      reference_line_info_->reference_line().GetLaneWidth(
          obstacle_end_s, &end_s_lane_left_width, &end_s_lane_right_width)) {
    if (obstacle->PerceptionSLBoundary().end_l() < 0.0) {
      if ((obstacle->PerceptionSLBoundary().end_l() > -end_s_lane_right_width &&
           obstacle->PerceptionSLBoundary().start_l() <
               -start_s_lane_right_width) ||
          obstacle->PerceptionSLBoundary().start_l() >
              -start_s_lane_right_width) {
        obstacle_start_l_to_lane_distance = 0.0;
        obstacle_start_l_to_lane_distance = 0.0;
      } else {
        obstacle_start_l_to_lane_distance =
            obstacle->PerceptionSLBoundary().start_l() +
            start_s_lane_right_width;
        obstacle_end_l_to_lane_distance =
            obstacle->PerceptionSLBoundary().end_l() + end_s_lane_right_width;
      }
    } else if (obstacle->PerceptionSLBoundary().start_l() > 0.0) {
      if ((obstacle->PerceptionSLBoundary().start_l() <
               start_s_lane_left_width &&
           obstacle->PerceptionSLBoundary().end_l() > end_s_lane_left_width) ||
          obstacle->PerceptionSLBoundary().end_l() < end_s_lane_left_width) {
        obstacle_start_l_to_lane_distance = 0.0;
        obstacle_start_l_to_lane_distance = 0.0;
      } else {
        obstacle_start_l_to_lane_distance =
            obstacle->PerceptionSLBoundary().start_l() -
            start_s_lane_left_width;
        obstacle_end_l_to_lane_distance =
            obstacle->PerceptionSLBoundary().end_l() - end_s_lane_left_width;
      }
    }
  }
  const double obstacle_to_lane_distance =
      std::min(std::abs(obstacle_start_l_to_lane_distance),
               std::abs(obstacle_end_l_to_lane_distance));

  if (!is_last_nudgelimit_obstacle &&
      ((obstacle->GetTrajMinL() < adc_frenet_l_upper &&
        obstacle->GetTrajMaxL() > adc_frenet_l_lower) ||
       ((obstacle->GetTrajMinL() < adc_frenet_l_upper + 0.3 &&
         obstacle->GetTrajMaxL() > adc_frenet_l_lower - 0.3) &&
        obstacle_to_lane_distance < 0.2))) {
    obstacle->SetIsNudgeLimitObstacle(true);

  } else if (is_last_nudgelimit_obstacle &&
             (obstacle->GetTrajMinL() > adc_frenet_l_upper ||
              obstacle->GetTrajMaxL() < adc_frenet_l_lower)) {
    obstacle->SetIsNudgeLimitObstacle(false);
  }
  return (obstacle->GetTrajMinL() < adc_frenet_l_upper &&
          obstacle->GetTrajMaxL() > adc_frenet_l_lower) ||
         ((obstacle->GetTrajMinL() < adc_frenet_l_upper + 0.3 &&
           obstacle->GetTrajMaxL() > adc_frenet_l_lower - 0.3) &&
          obstacle_to_lane_distance < 0.2);
}

bool SLTObstaclesProcessor::CheckIfIgnoreNudgeSpeedLimit(
    const Obstacle* obstacle) {
  if (!FLAGS_enable_lon_nudge_parallel_obstacle || obstacle == nullptr ||
      !obstacle->path_st_boundary().IsEmpty()) {
    return false;
  }
  const auto& decisions = obstacle->decisions();
  const auto& decider_tags = obstacle->decider_tags();
  if (decisions.empty() || decisions.size() != decider_tags.size()) {
    return false;
  }

  for (std::size_t i = 0; i < decisions.size(); ++i) {
    if ((decisions.at(i).has_overtake() &&
         decider_tags.at(i) == "obstacles_decider/-overtake") ||
        (decisions.at(i).has_yield() &&
         decider_tags.at(i) == "obstacles_decider/-yield") ||
        absl::StrContains(decider_tags.at(i),
                          "obstacles_decider/dynamic-left-nudge") ||
        absl::StrContains(decider_tags.at(i),
                          "obstacles_decider/dynamic-night-nudge")) {
      return true;
    }
  }
  return false;
}

void SLTObstaclesProcessor::CalculateIfOccludedObstacle(
    PathDecision* const path_decision) {
  if (path_decision == nullptr) {
    return;
  }

  double perception_occluded_prob = -1;
  for (const auto* const obs_item_ptr : path_decision->obstacles().Items()) {
    Obstacle* obs_ptr = path_decision->Find(obs_item_ptr->Id());
    if (!obs_ptr->path_st_boundary().IsEmpty() || obs_ptr == nullptr ||
        obs_ptr->IsCone()) {
      continue;
    }

    bool is_last_occluded_obstacle = true;
    if (last_frame_ != nullptr &&
        last_frame_->DriveReferenceLineInfo() != nullptr) {
      const auto& last_path_decision =
          last_frame_->DriveReferenceLineInfo()->path_decision();
      for (const auto* const last_obs_item_ptr :
           last_path_decision.obstacles().Items()) {
        const auto* last_obs_ptr =
            last_path_decision.Find(last_obs_item_ptr->Id());
        if (last_obs_ptr == nullptr ||
            last_obs_item_ptr->PerceptionId() != obs_ptr->PerceptionId()) {
          continue;
        }
        is_last_occluded_obstacle = last_obs_ptr->GetIsOccludedObstacle();
      }
    }
    if (obs_ptr->Perception().has_occluded_prob()) {
      if (obs_ptr->Perception().occluded_prob() > 1.0 ||
          obs_ptr->Perception().occluded_prob() < 0.0) {
        perception_occluded_prob = -1;
      } else {
        perception_occluded_prob = obs_ptr->Perception().occluded_prob();
      }
    }

    const double obstacle_occluded_pro =
        (is_last_occluded_obstacle) ? 0.3 : 0.5;
    if (perception_occluded_prob >= obstacle_occluded_pro &&
        !is_last_occluded_obstacle) {
      obs_ptr->SetIsOccludedObstacle(true);
    } else if (perception_occluded_prob < obstacle_occluded_pro &&
               is_last_occluded_obstacle) {
      obs_ptr->SetIsOccludedObstacle(false);
    }
  }
}

}  // namespace planning
}  // namespace TL
