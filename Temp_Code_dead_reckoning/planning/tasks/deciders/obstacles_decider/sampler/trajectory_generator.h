/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2023. All rights reserved.
 * Description:  trajectory_generator.h
 */

#pragma once

#include <memory>
#include <set>
#include <string>
#include <vector>

#include "planning/common/reference_line_info.h"
#include "planning/common/trajectory/discretized_trajectory.h"
#include "planning/tasks/deciders/obstacles_decider/sampler/at_sampler.h"
#include "planning/tasks/deciders/obstacles_decider/sampler/sl_curve.h"
#include "planning/tasks/deciders/obstacles_decider/sampler/sl_sampler.h"
#include "proto/common/pnc_point.pb.h"

namespace TL {
namespace planning {

class TrajectoryGenerator {
 public:
  TrajectoryGenerator() = default;
  explicit TrajectoryGenerator(const ObstaclesDeciderConfig& config);
  /**
   * @brief Init.
   * @param init_lon_state 
   * @param init_lat_state 
   * @param reference_line_info 
   */
  void Init(const std::array<double, 3>& init_lon_state,
            const std::array<double, 3>& init_lat_state,
            const ReferenceLineInfo& reference_line_info);

  /**
   * @brief GetDiscretizedTrajectories.
   * @return std::vector<DiscretizedTrajectory>.
   */
  const std::vector<DiscretizedTrajectory>& GetDiscretizedTrajectories() const {
    return discretized_trajectories_;
  }

 private:
  /**
   * @brief Sample.
   * @param reference_line 
   * @param init_relative_time 
   */
  void Sample(const ReferenceLine& reference_line, double sampling_interval);

  /**
   * @brief Combine.
   * @param reference_line
   * @param lon_trajectory 
   * @param lat_trajectory 
   * @param init_relative_time
   */
  void Combine(const ReferenceLine& reference_line,
               const QuadraticVTCurve& lon_trajectory,
               const SlCurve& lat_trajectory, double init_relative_time,
               DiscretizedTrajectory* combined_trajectory) const;
  ObstaclesDeciderConfig config_;
  std::unique_ptr<AtSampler> at_sampler_ = nullptr;
  std::unique_ptr<SlSampler> sl_sampler_ = nullptr;
  double sampling_interval_ = 0.0;

  std::vector<DiscretizedTrajectory> discretized_trajectories_ = {};
  std::array<double, 3> init_lon_state_ = {};
  std::array<double, 3> init_lat_state_ = {};
  double trajectory_time_resolution_ = 0.0;
};

}  // namespace planning
}  // namespace TL
