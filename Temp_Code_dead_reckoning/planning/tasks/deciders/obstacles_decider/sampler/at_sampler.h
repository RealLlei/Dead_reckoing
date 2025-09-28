/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2023. All rights reserved.
 * Description:  at_sampler.h
 */
#pragma once

#include <array>
#include <cstddef>
#include <memory>
#include <utility>
#include <vector>

#include "planning/tasks/optimizers/speed_data_optimizer/generators/vt_sample/curves/quadratic_vt_curve.h"
#include "proto/common/pnc_point.pb.h"

namespace TL::planning {
class AtSampler {
 public:
  AtSampler() = default;
  explicit AtSampler(
      const ObstaclesDeciderConfig_AtSamplerConfig& at_sampler_config);
  ~AtSampler() = default;

  /**
   * @brief GenerateLongitudinalTrajectoryBundle.
   * @param ptr_lon_trajectory_bundle 
   */
  void GenerateLongitudinalTrajectoryBundle(
      std::vector<std::shared_ptr<QuadraticVTCurve>>* vt_curves);

  /**
   * @brief Init.
   * @param init_lon_state 
   */
  void Init(const std::array<double, 3>& init_lon_state);

 private:
  std::array<double, 3> init_lon_state_ = {};
  std::vector<std::vector<std::pair<double, double>>> at_sampling_strategies_ =
      {};
  bool enable_debug_ = false;
};
}  // namespace TL::planning
