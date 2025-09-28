/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2023. All rights reserved.
 * Description:  sl_sampler.h
 */
#pragma once

#include <array>
#include <memory>
#include <utility>
#include <vector>
#include "planning/tasks/deciders/obstacles_decider/sampler/sl_curve.h"
#include "planning/proto/task_config.pb.h"

namespace TL::planning {
using State = std::array<double, 3>;
using Condition = std::pair<State, double>;

class SlSampler {
 public:
  SlSampler() = default;
  explicit SlSampler(
      const TL::planning::ObstaclesDeciderConfig_SlSamplerConfig&
          sl_sampler_config);
  ~SlSampler() = default;

  /**
   * @brief GenerateLateralTrajectoryBundle.
   * @param sl_curves 
   */
  void GenerateLateralTrajectoryBundle(
      std::vector<std::shared_ptr<SlCurve>>* sl_curves) const;
  void Init(const std::array<double, 3>& init_lat_state);

 private:
  std::array<double, 3> init_lat_state_ = {};
  std::vector<std::vector<Condition>> sample_conditions_ = {};
};
}  // namespace TL::planning
