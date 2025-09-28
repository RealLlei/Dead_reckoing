/******************************************************************************
 * Copyright 2019 The TL Authors. All Rights Reserved.
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
#pragma once

#include <memory>
#include <vector>

#include "planning/tasks/optimizers/open_space_speed_optimizer/st_sample_curves.h"

namespace TL {
namespace planning {
class StSampleCost {
 public:
  /**
   * @brief Construct a new St Sample Cost object
   *
   * @param sample_params
   * @param speed_limit
   * @param last_curve
   * @param nuit_s
   * @param unit_t
   * @param max_sample_t
   */
  StSampleCost(const StSampleParams& sample_params,
               const std::vector<double>& speed_limit,
               const std::shared_ptr<StCurve>& last_curve, double unit_s,
               double unit_t, double max_sample_t);
  /**
   * @brief cal curve cost
   *
   * @param st_curve
   * @return double
   */
  double CalCurveCost(StCurve* st_curve);

 private:
  // fliter over speed cost
  const StSampleParams& sample_params_;
  // const std::shared_ptr<StCurve>& last_curve_;
  const std::vector<double>& speed_limit_;

  const double max_sample_t_ = 0.0;
  const double unit_s_ = 0.1;
  const double unit_t_ = 0.2;
  std::vector<std::array<double, 4>> last_tsva_;
  size_t tsva_size_ = 0;
  std::vector<std::array<double, 4>> tsva_;

  // sum ((max_v - v) / max_v) / N
  double efficiency_cost_ = 0.0;
  // sum(fabs(acc))/N
  double acc_cost_ = 0.0;
  // jerk cost
  double jerk_cost_ = 0.0;
  // over_speed_cost
  double over_speed_cost_ = 0.0;
  // diff cost
  double diff_cost_ = 0.0;
};

}  // namespace planning
}  // namespace TL
