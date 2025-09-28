/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

/**
 * @file
 **/

#include "planning/common/open_space_info.h"
#include <cstddef>
#include <string>
#include "common/time/clock.h"
#include "proto/planning/planning_internal.pb.h"

namespace TL {
namespace planning {
using TL::common::Clock;

void CopyTrajectory(const DiscretizedTrajectory& trajectory_src,
                    TL::common::Trajectory* trajectory_tgt_ptr) {
  const auto horizon = trajectory_src.NumOfPoints();
  for (size_t i = 0; i < horizon; ++i) {
    *trajectory_tgt_ptr->add_trajectory_point() =
        trajectory_src.TrajectoryPointAt(i);
  }
}

void CopyPath(const DiscretizedPath& path_src,
              TL::common::Path* const path_tgt_ptr) {
  if (path_tgt_ptr == nullptr) {
    AERROR << "CopyPath function input check fails";
    return;
  }
  const size_t horizon = path_src.size();
  for (size_t i = 0; i < horizon; ++i) {
    *path_tgt_ptr->add_path_point() = path_src.at(i);
  }
}

// record more trajectory information to info debug
void OpenSpaceInfo::RecordDebug(
    TL::planning_internal::Debug* const ptr_debug) {
  // 1, Copy info into ptr_debug
  if (ptr_debug == nullptr) {
    AERROR << "RecordDebug function input check fails";
    return;
  }
  *ptr_debug = debug_instance_;
  auto* open_space = ptr_debug->mutable_planning_data()->mutable_open_space();

  // 2, record partitioned paths into ptr_debug
  auto* ptr_partitioned_paths = open_space->mutable_partitioned_paths();

  for (auto& iter : partitioned_paths_.path_set) {
    const auto& picked_path = iter.first;
    auto* ptr_added_path = ptr_partitioned_paths->add_path();
    CopyPath(picked_path, ptr_added_path);
  }

  // 3, record chosen partitioned into ptr_debug
  for (const auto& path_p : chosen_partitioned_path_.first) {
    auto* p = open_space->add_chosen_path();
    *(p) = path_p;
  }

  open_space->set_replan_triggered_by_speed_plan(
      replan_triggered_by_speed_plan_);
  open_space->set_current_path_has_collision_risk(
      current_path_has_collision_risk_);
  *(open_space->mutable_speed_plan_collision_info()) =
      speed_plan_collision_info_;

  auto* ptr_speed_optimizer_trajectory =
      open_space->mutable_speed_optimizer_trajectory()->add_trajectory();
  const auto& src_trajectory = speed_optimizer_trajectory_.first;
  CopyTrajectory(src_trajectory, ptr_speed_optimizer_trajectory);

  if (AvpSpeedPlanCollisionInfo::NO_COLLISION !=
      speed_plan_collision_info_.collision_type()) {
    *open_space->mutable_future_collision_point() = future_collision_point_;
  }

  if (open_space_path_info_map_.find(open_space_path_info_id_) !=
      open_space_path_info_map_.end()) {
    const auto& open_space_path_info =
        open_space_path_info_map_.at(open_space_path_info_id_);
    open_space->mutable_origin_point()->set_x(open_space_path_info.origin.x());
    open_space->mutable_origin_point()->set_y(open_space_path_info.origin.y());
    open_space->mutable_origin_point()->set_z(0.0);
    open_space->set_origin_heading_rad(open_space_path_info.rotate_angle);
  } else {
    open_space->mutable_origin_point()->set_x(0.0);
    open_space->mutable_origin_point()->set_y(0.0);
    open_space->mutable_origin_point()->set_z(0.0);
    open_space->set_origin_heading_rad(0.0);
  }

  open_space->mutable_vehicle_follow_error()->CopyFrom(vehicle_follow_error_);
  open_space->mutable_vehicle_to_current_end_error()->CopyFrom(
      vehicle_to_current_end_error_);

  open_space->set_partition_path_idx(chosen_partitioned_path_idx_.first);
  open_space->set_partition_path_point_idx(chosen_partitioned_path_idx_.second);
  open_space->set_path_replan_reason(replan_reason_);
}

void OpenSpaceInfo::UpdateReplanStatus(
    const TL::planning::OpenSpaceStatus::Replan& update_replan_status,
    TL::planning::OpenSpaceStatus* const open_space_status_ptr) {
  if (nullptr == open_space_status_ptr) {
    return;
  }
  auto replan =
      open_space_status_ptr->has_replan() ? open_space_status_ptr->replan() : 0;
  const auto update_replan = static_cast<uint32_t>(update_replan_status);
  if ((replan & update_replan) == 0U) {
    // update
    replan += update_replan;
  }
  open_space_status_ptr->set_replan(replan);
}

// LCOV_EXCL_START
// Decide the file name to write one shoot log
ExpansionInfo::ExpansionInfo(const common::PathPoint& start_point) {
  std::string temp_name = FLAGS_one_shoot_log_root_dir + "/" +
                          "pos_x= " + std::to_string(start_point.x()) +
                          "_pos_y= " + std::to_string(start_point.y()) +
                          "_phi= " + std::to_string(start_point.theta());
  current_node_filename_ = temp_name + "_current_node.txt";
  extension_filename_ = temp_name + "_extension_log.txt";
  path_filename_ = temp_name + "_path.txt";
  environment_filename_ = temp_name + "_environment.txt";
}

void ExpansionInfo::load_extension_node_info(
    const double iter_num, const std::string& current_node_id,
    const std::shared_ptr<Node3d>& next_node) {
  char* end_str = nullptr;
  std::vector<double> node_data = {
      iter_num,
      next_node->GetX(),
      next_node->GetY(),
      next_node->GetPhi(),
      strtod(next_node->GetIndex().c_str(), &end_str),
      strtod(current_node_id.c_str(), &end_str),
      static_cast<double>(next_node->GetDirec()),
      next_node->GetSteer(),
      static_cast<double>(next_node->GetStepSize()),
      static_cast<double>(next_node->GetXs().back()),
      static_cast<double>(next_node->GetYs().back())};
  std::ofstream ofs(extension_filename_, std::ios::app);
  for (const double j : node_data) {
    ofs << j << "\t";
  }
  ofs << std::endl;

  ofs.close();
}

void ExpansionInfo::load_current_node_info(
    const double iter_num, const std::string& current_node_id,
    const double astar_start_time,
    const std::shared_ptr<Node3d>& current_node) {
  std::vector<double> node_data;
  char* end_str = nullptr;
  if (iter_num == 0) {
    node_data = {iter_num,
                 current_node->GetX(),
                 current_node->GetY(),
                 current_node->GetPhi(),
                 strtod(current_node_id.c_str(), &end_str),
                 -1.0,
                 -1.0,
                 0.0,
                 0.0};
  } else {
    std::shared_ptr<Node3d> pre_node = current_node->GetPreNode();
    node_data = {iter_num,
                 current_node->GetX(),
                 current_node->GetY(),
                 current_node->GetPhi(),
                 strtod(current_node->GetIndex().c_str(), &end_str),
                 strtod(pre_node->GetIndex().c_str(), &end_str),
                 static_cast<double>(pre_node->GetDirec()),
                 pre_node->GetSteer(),
                 static_cast<double>(pre_node->GetStepSize()),
                 Clock::NowInSeconds() - astar_start_time};
  }
  std::ofstream ofs(current_node_filename_, std::ios::app);
  for (const double j : node_data) {
    ofs << j << "\t";
  }
  ofs << std::endl;
  ofs.close();
}

void ExpansionInfo::load_extension_environment_info(
    const std::vector<std::pair<common::math::LineSegment2d, double>>&
        obstacles_segments_vec) {
  std::ofstream ofs(environment_filename_, std::ios::app);
  for (const auto& obstacle_segment : obstacles_segments_vec) {
    ofs << obstacle_segment.first.start().x() << "\t"
        << obstacle_segment.first.start().y() << "\t"
        << obstacle_segment.first.end().x() << "\t"
        << obstacle_segment.first.end().y();
    ofs << std::endl;
  }
  ofs.close();
}

void ExpansionInfo::load_coarse_path_info(
    const std::vector<double>& result_x, const std::vector<double>& result_y,
    const std::vector<double>& result_phi) {
  if (result_x.size() != result_y.size() ||
      result_x.size() != result_phi.size()) {
    AERROR << "hybrid a start result size does not match";
    return;
  }
  std::ofstream ofs(path_filename_, std::ios::app);
  for (size_t i = 0; i < result_x.size(); i++) {
    ofs << result_x[i] << "\t" << result_y[i] << "\t" << result_phi[i];
    ofs << std::endl;
  }
  ofs.close();
}

// LCOV_EXCL_STOP
}  // namespace planning
}  // namespace TL
