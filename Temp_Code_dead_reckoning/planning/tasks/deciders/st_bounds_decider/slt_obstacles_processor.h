/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file slt_obstacles_processor.h
 **/

#pragma once

#include <limits>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "common/status/status.h"
#include "planning/common/frame.h"
#include "planning/common/obstacle.h"
#include "planning/common/path/path_data.h"
#include "planning/common/path_decision.h"
#include "planning/common/reference_line_info.h"
#include "planning/common/speed/st_boundary.h"

#include "proto/common/pnc_point.pb.h"
#include "proto/common/vehicle_config.pb.h"
#include "proto/planning/decision.pb.h"

namespace TL {
namespace planning {

class SLTObstaclesProcessor {
  std::pair<double, double> overlapping_l;

 public:
  SLTObstaclesProcessor() = default;

  bool Init(double planning_distance, double planning_time,
            ReferenceLineInfo* reference_line_info, const Frame* frame,
            const Frame* last_frame);

  virtual ~SLTObstaclesProcessor() = default;

  common::Status MapObstaclesToSLTBoundaries(PathDecision* path_decision);

  std::unordered_map<std::string, STBoundary> GetAllSTBoundaries();

  /** 
   * @brief Given a single obstacle, compute its SLT-boundary.
   * @param obstacle An obstacle (if moving, should contain predicted trajectory).
   * @param slt_boundary slt boundary
   * @return If do not ignore, return true; otherwise, false.
   */
  bool ComputeObstacleSLTBoundary(const Obstacle& obstacle,
                                  SLTBoundary* slt_boundary);

  /** 
   * @brief Given a static obstacle, compute its SLT-boundary.
   * @param obstacle An obstacle (if moving, should contain predicted trajectory).
   * @param slt_boundary slt boundary
   * @return If do not ignore, return true; otherwise, false.
   */
  bool ComputeStaticObstacleSLTBoundary(const Obstacle& obstacle,
                                        SLTBoundary* slt_boundary);

  /** 
   * @brief Given a dynamic obstacle, compute its SLT-boundary.
   * @param obstacle An obstacle (if moving, should contain predicted trajectory).
   * @param slt_boundary slt boundary
   * @return If do not ignore, return true; otherwise, false.
   */
  bool ComputeDynamicObstacleSLTBoundary(const Obstacle& obstacle,
                                         SLTBoundary* slt_boundary);

  bool ComputeStaticObstacleSLTBoundaryForPath(const Obstacle& obstacle,
                                               SLTBoundary* slt_boundary);

  bool ComputeDynamicObstacleSLTBoundaryForPath(const Obstacle& obstacle,
                                                SLTBoundary* slt_boundary);

  bool ComputeCruiseTargetSLTBoundaryForPath(const Obstacle& obstacle,
                                             SLTBoundary* slt_boundary);

 private:
  bool CheckIfIgnore(Obstacle* obstacle,
                     std::set<SLTBoundary::BoundaryType>* boundary_types);

  /**
   * @brief 
   * 
   * @param obstacle 
   * @return true 
   * @return false 
   */
  static bool CheckIfNudge(const Obstacle& obstacle);
  /**
   * @brief Check if obstacle is nudge speed limit
   * 
   * @param obstacle 
   * @return true 
   * @return false 
   */
  bool CheckIfNudgeSpeedLimit(Obstacle* obstacle) const;
  /**
   * @brief Check if ignore obstacle is nudge speed limit
   * 
   * @param obstacle 
   * @return true 
   * @return false 
   */
  static bool CheckIfIgnoreNudgeSpeedLimit(const Obstacle* obstacle);

  /**
   * @brief 计算 Nudge 限制障碍物的范围
   * 
   * @param path_decision 
   */
  void CalculateIfOccludedObstacle(PathDecision* path_decision);

  std::mutex lock_0_;
  const Frame* frame_ = nullptr;
  const Frame* last_frame_ = nullptr;
  double planning_time_ = 0.0;
  double planning_distance_ = 0.0;
  common::VehicleParam vehicle_param_;
  ReferenceLineInfo* reference_line_info_ = nullptr;
  const ReferenceLineInfo* adc_reference_line_info_ = nullptr;
  bool has_left_road_right_ = false;
  bool has_right_road_right_ = false;
};

}  // namespace planning
}  // namespace TL
