/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  planning obstacle processor base class
 * Author: ROC
 */

#pragma once

#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "planning/common/dependency_injector.h"
#include "planning/common/frame.h"
#include "planning/common/obstacle.h"
#include "planning/tasks/deciders/path_bounds_decider/bound_processor/process_bound.h"
#include "planning/tasks/deciders/path_bounds_decider/util/path_info.h"

namespace TL {
namespace planning {

class ObsProcessor {
 public:
  explicit ObsProcessor(const TaskConfig& config);
  ObsProcessor(const std::shared_ptr<DependencyInjector>& injector,
               const TaskConfig& config);
  virtual ~ObsProcessor() = default;

  const std::shared_ptr<ProcessBound>& GetProcessBound() const {
    return process_bound_;
  }

 protected:
  /**
   * @brief obstacle process with towing function
   * 
   * @param reference_line_info 
   * @param frame 
   * @param path_boundaries 
   * @param blocking_obstacle_id 
   * @param towing_points 
   * @param is_enable_towing_process
   * @return true 
   * @return false 
   */
  virtual bool Process(ReferenceLineInfo* reference_line_info, Frame* frame,
                       PathBound* path_boundaries,
                       std::string* blocking_obstacle_id,
                       TowingPointsInfo* towing_points,
                       bool is_enable_towing_process) = 0;

  /**
   * @brief Set the Towing Points object
   * 
   * @param towing_prepare_distance: pair of forward and backward towing prepare distance 
   * @param path_boundaries 
   * @param towing_points 
   * @param cur_obs_edges: pair of start s and end s 
   * @param obstacle_id 
   * @param is_bigcar_left_nudge 
   */
  void SetTowingPoints(const std::pair<double, double>& towing_prepare_distance,
                       const std::pair<double, double>& cur_obs_edges,
                       const std::string& obstacle_id,
                       ReferenceLineInfo* reference_line_info,
                       PathBound* path_boundaries,
                       TowingPointsInfo* towing_points, double expect_towing_l,
                       bool is_obstacle_left_nudge);

  /**
   * @brief Get short distance threshold
   * 
   * @param cur_obs 
   *
   * @return short distance threshold 
  */
  double ShortDistanceThreshold(Obstacle* cur_obs) const;

  const std::shared_ptr<DependencyInjector>& GetInjector() const {
    return injector_;
  }

  /**
   * @brief Use ttc calculate towing prepare distance
   * 
   * @param reference_line_info
   * @param towing_prepare_distance 
   * @param is_static_obstacle 
   */
  void UseTtcCalculateTowingPrepareDistance(
      ReferenceLineInfo* reference_line_info,
      std::pair<double, double>* towing_prepare_distance,
      bool is_static_obstacle);

  double LookUpTowingLDistance(
      const double& input_x,
      const google::protobuf::RepeatedField<double>& input_y,
      const google::protobuf::RepeatedField<double>& output_y);

  const TaskConfig& GetConfig() const { return config_; }

 private:
  std::shared_ptr<DependencyInjector> injector_;
  TaskConfig config_;
  std::shared_ptr<ProcessBound> process_bound_;
};
}  // namespace planning
}  // namespace TL
