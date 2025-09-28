/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file vt_sample_recorder.h
 **/

#pragma once

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "planning/tasks/optimizers/speed_data_optimizer/caches/speed_cache.h"
#include "planning/tasks/optimizers/speed_data_optimizer/costs/speed_curve.h"
#include "planning/proto/task_config.pb.h"

namespace TL {
namespace planning {

/**
 * @class VtSampleRule
 * @brief this class is used to record vt sample debug information
 */
class VtSampleRule {
 public:
  /**
   * @brief Construct a new Vt Sample Rule object
   *
   * @param config vt sample config
   */
  explicit VtSampleRule(const VtSamplerConfig& config);
  virtual ~VtSampleRule() = default;

  virtual bool ApplyRule(const std::shared_ptr<SpeedCurve>& curve) const = 0;

  virtual void Update(const common::TrajectoryPoint& init_point,
                      const SpeedCache& cache) = 0;

 protected:
  const VtSamplerConfig& config_;
};

}  // namespace planning
}  // namespace TL
