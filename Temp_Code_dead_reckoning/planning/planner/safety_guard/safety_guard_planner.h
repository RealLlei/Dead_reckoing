/*
 * Copyright (c) TL Technologies Co., Ltd. 2022. All rights reserved.
 * Description:  safety_guard_planner.h
 */

#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "common/status/status.h"
#include "planning/planner/planner.h"

#include "planning/proto/planning_config.pb.h"
#include "proto/common/pnc_point.pb.h"
#include "proto/planning/planning.pb.h"

namespace TL {
namespace planning {

/**
 * @brief Safety Guard Planner
 *
 */
class SafetyGuardPlanner final : public Planner {
 public:
  SafetyGuardPlanner() = delete;

  /**
   * @brief Construct a new Safety Guard Planner object
   *
   * @param injector DependencyInjector
   */
  explicit SafetyGuardPlanner(
      const std::shared_ptr<DependencyInjector>& injector);

  /**
   * @brief Destroy the Safety Guard Planner object
   *
   */
  ~SafetyGuardPlanner() override = default;

  /**
   * @brief override Stop()
   *
   */
  void Stop() override {}

  /**
   * @brief override Name()
   *
   * @return std::string planner name
   */
  std::string Name() override { return "SAFETY_GUARD"; }

  /**
   * @brief override Init() init class
   *
   * @param config PlanningConfig
   * @return common::Status errorcode and msg
   */
  common::Status Init(const PlanningConfig& config) override;

  /**
   * @brief override Plan() main process
   *
   * @param planning_init_point init point for planner
   * @param frame frame data
   * @param ptr_computed_trajectory output trajectory info
   * @return common::Status errorcode and msg
   */
  common::Status Plan(const common::TrajectoryPoint& /*planning_init_point*/,
                      Frame* frame,
                      ADCTrajectory* ptr_computed_trajectory) override {
    DCHECK(nullptr != frame);
    DCHECK(nullptr != ptr_computed_trajectory);
    return Plan(frame->vehicle_state(), frame->local_view(),
                ptr_computed_trajectory);
  }

 private:
  /**
   * @brief main process
   *
   * @param vehicle_state vehicle state
   * @param local_view input data
   * @param ptr_computed_trajectory output trajectory
   * @return common::Status errorcode and msg
   */
  common::Status Plan(const common::VehicleState& vehicle_state,
                      const TL::planning::LocalView& local_view,
                      ADCTrajectory* ptr_computed_trajectory) const;

  /**
   * @brief process uss obstacle
   *
   * @param perception_obstacles perception obstacle
   * @param discretized_paths path points based kinematic model
   * @param gear_position gear_position
   * @param safety_guard_info_ptr safety guard info
   */
  void ProcessUSSObstacle(
      const perception::PerceptionObstacles& perception_obstacles,
      const std::vector<std::pair<std::string, std::vector<common::PathPoint>>>&
          discretized_paths,
      const soc::Chassis::GearPosition& gear_position,
      planning_internal::SafetyGuardInfo* safety_guard_info_ptr) const;

  /**
   * @brief process freespace
   *
   * @param free_space freesapce info
   * @param discretized_paths path points based kinematic model
   * @param vehicle_state vehicle state
   * @param safety_guard_info_ptr safety guard info
   */
  void ProcessFreeSpace(
      const perception::FreeSpaceOutArray& free_space,
      const std::vector<std::pair<std::string, std::vector<common::PathPoint>>>&
          discretized_paths,
      const common::VehicleState& vehicle_state,
      planning_internal::SafetyGuardInfo* safety_guard_info_ptr) const;

  /**
   * @brief generate discretized path based kinematic model by phi
   *
   * @param gear_position gear position
   * @param steer_angle steer angle
   * @param path_points path points
   */
  void GenerateDiscretizedPath(
      const soc::Chassis::GearPosition& gear_position, double steer_angle,
      std::vector<common::PathPoint>* path_points) const;

  /**
   * @brief generate discretized path based kinematic model by different phi
   *
   * @param vehicle_state vehicle state
   * @param dphi delta steer angle
   * @param paths all discretized path(current, left, right)
   */
  void GenerateDiscretizedPaths(
      const common::VehicleState& vehicle_state, double dphi,
      std::vector<std::pair<std::string, std::vector<common::PathPoint>>>*
          paths) const;

  /**
   * @brief CheckCollisionWithUSSObstacle
   * 
   * @param perception_obstacles perception obstacle
   * @param discretized_paths path points based kinematic model
   * @param gear_position gear_position
   * @param vehicle_state vehicle state
   * @param distance_to_uss min distance_from vehcile to uss
   * @return true collision enbale
   * @return false collision disabled
   */
  bool CheckCollisionWithUSSObstacle(
      const perception::PerceptionObstacles& perception_obstacles,
      const std::vector<std::pair<std::string, std::vector<common::PathPoint>>>&
          discretized_paths,
      const soc::Chassis::GearPosition& gear_position, double* distance_to_uss,
      std::string* path_label, int32_t* collision_uss_id) const;

  /**
   * @brief check collision with freeSpace
   *
   * @param free_space freespace
   * @param discretized_paths path points based kinematic model
   * @param vehicle_state vehicle state
   * @param distance_to_free_space min distance_from vehcile to freespace
   * @return true collision enbale
   * @return false collision disabled
   */
  bool CheckCollisionWithFreeSpace(
      const perception::FreeSpaceOutArray& free_space,
      const std::vector<std::pair<std::string, std::vector<common::PathPoint>>>&
          discretized_paths,
      const common::VehicleState& vehicle_state,
      double* distance_to_free_space) const;

  /**
   * @brief Is far away from vehicle
   *
   * @param vehicle_x vehicle x
   * @param vehicle_y vehicle y
   * @param vehicle_theta vehicle theta
   * @param point obstacle point
   * @param collision_buffer collision_buffer
   * @return true obstacle far away from vehicle
   * @return false obstacle not far away from vehicle
   */
  bool IsFarAwayFromVehicle(double vehicle_x, double vehicle_y,
                            double vehicle_theta,
                            const common::math::Vec2d& point,
                            double collision_buffer) const;

  const common::VehicleParam vehicle_param_;
  const size_t path_point_num_ = 0;
};

}  // namespace planning
}  // namespace TL
