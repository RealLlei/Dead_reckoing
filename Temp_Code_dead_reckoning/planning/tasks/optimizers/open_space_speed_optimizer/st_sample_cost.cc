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
#include "planning/tasks/optimizers/open_space_speed_optimizer/st_sample_cost.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

#include "common/math/double_type.h"
#include "planning/tasks/optimizers/open_space_speed_optimizer/st_sample_curves.h"

namespace TL {
namespace planning {
using TL::common::math::double_type::DefinitelyGreater;

StSampleCost::StSampleCost(const StSampleParams& sample_params,
                           const std::vector<double>& speed_limit,
                           const std::shared_ptr<StCurve>& last_curve,
                           const double unit_s, double unit_t,
                           double max_sample_t)
    : sample_params_(sample_params),
      speed_limit_(speed_limit),
      max_sample_t_(max_sample_t),
      unit_s_(unit_s),
      unit_t_(unit_t) {
  const auto size_of_tsva = static_cast<size_t>(max_sample_t_ / unit_t_) + 2;
  tsva_.reserve(size_of_tsva);
  tsva_.assign(size_of_tsva, std::array<double, 4>{0, 0, 0, 0});
  if (nullptr != last_curve) {
    last_curve->Discrete(unit_t_, &last_tsva_);
  }
}

double StSampleCost::CalCurveCost(StCurve* const st_curve) {
  double total_cost = std::numeric_limits<double>::max();
  tsva_size_ = 0;
  if (!st_curve->Discrete(unit_t_, &tsva_, &tsva_size_)) {
    return total_cost;
  }

  efficiency_cost_ = 0.0;
  acc_cost_ = 0.0;
  over_speed_cost_ = 0.0;
  jerk_cost_ = 0.0;
  diff_cost_ = 0.0;
  static constexpr double kA = -12.0;
  static constexpr double kB = 4.8;
  static constexpr double kMinRation = 0.0;
  for (size_t i = 0; i < tsva_size_; i++) {
    efficiency_cost_ +=
        sample_params_.efficiency_cost *
        (sample_params_.speed_bound_info.max_sample_speed() - tsva_[i][2]) /
        sample_params_.speed_bound_info.max_sample_speed();
    acc_cost_ += sample_params_.acc_cost * fabs(tsva_[i][3]);

    if (i > 0) {
      jerk_cost_ += sample_params_.jerk_cost *
                    fabs((tsva_[i][3] - tsva_[i - 1][3]) / unit_t_);
    } else {
      jerk_cost_ += sample_params_.jerk_cost *
                    fabs((tsva_[i][3] - sample_params_.start_acc) / unit_t_);
    }

    size_t index = std::min(static_cast<size_t>(floor(tsva_[i][1] / unit_s_)),
                            speed_limit_.size() - 1);
    if (DefinitelyGreater(tsva_[i][2], speed_limit_[index])) {
      double ratio =
          std::max(kMinRation, sample_params_.over_speed_cost_max -
                                   sample_params_.over_speed_cost_min);
      const auto over_speed_cost =
          ratio * (1.0 / (1.0 + exp(kA + kB * tsva_[i][1]))) +
          sample_params_.over_speed_cost_min;
      over_speed_cost_ += over_speed_cost * (tsva_[i][2] - speed_limit_[index]);
    }
  }

  if (!last_tsva_.empty()) {
    size_t size = std::min(last_tsva_.size(), tsva_size_);
    for (size_t i = 0; i < size; i++) {
      diff_cost_ +=
          sample_params_.diff_cost * fabs(last_tsva_[i][1] - tsva_[i][1]);
    }
  }

  const auto size = static_cast<double>(tsva_size_);
  efficiency_cost_ /= size;
  acc_cost_ /= size;
  jerk_cost_ /= size;
  over_speed_cost_ /= size;
  diff_cost_ /= size;

  total_cost =
      efficiency_cost_ + acc_cost_ + over_speed_cost_ + jerk_cost_ + diff_cost_;
  st_curve->SetCostInfo(efficiency_cost_, acc_cost_, jerk_cost_,
                        over_speed_cost_, diff_cost_);

  return total_cost;
}
}  // namespace planning
}  // namespace TL
