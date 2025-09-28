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
#include "planning/tasks/optimizers/open_space_speed_optimizer/st_sample_curves.h"

#include <termios.h>
#include <cmath>

#include <algorithm>
#include <array>
#include <cstddef>
#include <iterator>
#include <limits>
#include <memory>
#include <ostream>
#include <utility>
#include <vector>

#include "common/configs/vehicle_config_helper.h"

#include "common/math/double_type.h"
#include "planning/common/speed/speed_data.h"

namespace TL {
namespace planning {

using TL::common::math::double_type::DefinitelyGreater;
using TL::common::math::double_type::DefinitelyLess;
using TL::common::math::double_type::DefinitelyLessEqual;
using TL::common::math::double_type::SeemsEqual;
using TL::common::math::double_type::SeemsNotEqual;

bool StCurve::Init(const double start_s, const double start_v,
                   const double end_s, const double end_v,
                   const std::vector<std::array<double, 2>>& at_params) {
  if (at_params.empty() || at_params.size() > kMaxTShapeParamsSize) {
    return false;
  }

  ResetCostInfo();

  start_s_ = start_s;
  start_v_ = start_v;
  end_s_ = end_s;
  end_v_ = end_v;

  if (at_params.size() != origins_.size()) {
    origins_.reserve(at_params.size());
    origins_.assign(at_params.size(), UniformSpeedOrigin{0, 0, 0, 0});
  }
  origins_[0].s = start_s;
  origins_[0].v = start_v;

  total_t_ = 0.0;
  for (size_t i = 0; i < at_params.size(); ++i) {
    origins_[i].t = total_t_;
    origins_[i].a = at_params[i][0];

    if (i > 0) {
      // v1 = v0 + a0 * t0
      // s1 = s0 + (v1 + v0) * t0 * 0.5
      origins_[i].v =
          origins_[i - 1].v + at_params[i - 1][0] * at_params[i - 1][1];
      origins_[i].s = origins_[i - 1].s + (origins_[i].v + origins_[i - 1].v) *
                                              at_params[i - 1][1] * 0.5;
    }
    total_t_ += at_params[i][1];
  }

  return true;
}

void StCurve::UpdateOriginByDiffTime(const double diff_time) {
  if (!DefinitelyGreater(diff_time, 0.0)) {
    return;
  }

  auto index = std::distance(
      origins_.begin(),
      std::lower_bound(origins_.begin(), origins_.end(), diff_time,
                       [](const UniformSpeedOrigin& ori, double time) {
                         return ori.t < time;
                       }));
  if (index <= 0 || origins_.size() <= index) {
    return;
  }

  std::vector<UniformSpeedOrigin> new_origins(origins_.begin() + index,
                                              origins_.end());
  const double diff_s = EvaluateFast(0, diff_time, origins_[index - 1]);
  const double diff_v = EvaluateFast(1, diff_time, origins_[index - 1]);
  std::for_each(new_origins.begin(), new_origins.end(),
                [&](UniformSpeedOrigin& ori) {
                  ori.s -= diff_s;
                  ori.t -= diff_time;
                });
  new_origins.insert(new_origins.begin(), origins_[index - 1]);
  new_origins.front().t = 0.0;
  new_origins.front().s = 0.0;
  new_origins.front().v = diff_v;

  origins_ = new_origins;
  start_s_ = 0.0;
  start_v_ = diff_v;
  end_s_ -= diff_s;
  total_t_ -= diff_time;
}

double StCurve::EvaluateFast(const std::uint32_t order, const double t,
                             const UniformSpeedOrigin& tsva) {
  switch (order) {
    case 0: {
      return (0.5 * tsva.a * t + tsva.v) * t + tsva.s;
    }
    case 1: {
      return tsva.a * t + tsva.v;
    }
    case 2: {
      return tsva.a;
    }
    default:
      return 0.0;
  }
}

bool StCurve::Discrete(const double unit_t,
                       std::vector<std::array<double, 4>>* const tsva_ptr,
                       size_t* const tsva_size) const {
  // LCOV_EXCL_START
  if (tsva_ptr == nullptr || (*tsva_size) != 0 || origins_.empty()) {
    AERROR << "input check failed!";
    return false;
  }
  // LCOV_EXCL_STOP

  const size_t num_of_knots = static_cast<size_t>(total_t_ / unit_t) + 1;
  if (tsva_ptr->size() < num_of_knots + 1) {
    AERROR << "size not match!";
    return false;
  }

  size_t curr_piece = 0;
  for (size_t i = 0; i < num_of_knots; ++i) {
    auto& curr_tsva = tsva_ptr->at(i);
    auto curr_t = static_cast<double>(i) * unit_t;

    // origin[n+1].t is the origin[n] end time
    // if curr_t is bigger than origin[n+1].t, use next origin
    if (curr_piece + 1 < origins_.size() &&
        origins_[curr_piece + 1].t < curr_t) {
      curr_piece++;
    }

    const auto& params = origins_[curr_piece];
    curr_tsva[0] = curr_t;
    auto dt = curr_t - params.t;
    curr_tsva[1] = EvaluateFast(0, dt, params);
    curr_tsva[2] = EvaluateFast(1, dt, params);
    curr_tsva[3] = EvaluateFast(2, dt, params);

    if (i > 0 && DefinitelyGreater(tsva_ptr->at(i - 1)[1], curr_tsva[1])) {
      AERROR << "st sample curve's decrease!";
      (*tsva_size) = 0;
      return false;
    }

    ++(*tsva_size);
  }

  // add the last point t due to the unit_t * (num_of_knots - 1) small total_t
  if ((*tsva_size) < 1 || ((*tsva_size - 1) < tsva_ptr->size() - 1 &&
                           tsva_ptr->at(*tsva_size - 1)[0] < total_t_ &&
                           tsva_ptr->at(*tsva_size - 1)[1] < end_s_)) {
    tsva_ptr->at(*tsva_size)[0] = total_t_;
    tsva_ptr->at(*tsva_size)[1] = end_s_;
    tsva_ptr->at(*tsva_size)[2] = end_v_;
    tsva_ptr->at(*tsva_size)[3] = origins_.back().a;
    *tsva_size = (*tsva_size) + 1;
  }

  return (*tsva_size) > 0;
}

bool StCurve::Discrete(
    const double unit_t,
    std::vector<std::array<double, 4>>* const tsva_ptr) const {
  if (tsva_ptr == nullptr) {
    return false;
  }

  const size_t num_of_knots = static_cast<size_t>(total_t_ / unit_t) + 2;
  ADEBUG << "total_t_: " << total_t_ << ", unit_t: " << unit_t;
  tsva_ptr->reserve(num_of_knots);
  tsva_ptr->assign(num_of_knots, std::array<double, 4>{0, 0, 0, 0});

  size_t tsva_size = 0;
  if (!Discrete(unit_t, tsva_ptr, &tsva_size)) {
    return false;
  }
  tsva_ptr->resize(tsva_size, std::array<double, 4>{0, 0, 0, 0});

  return !tsva_ptr->empty();
}

bool StCurve::Discrete(const double unit_t,
                       SpeedData* const speed_data_ptr) const {
  if (speed_data_ptr == nullptr) {
    return false;
  }
  speed_data_ptr->clear();
  std::vector<std::array<double, 4>> tsva;
  if (!Discrete(unit_t, &tsva)) {
    return false;
  }

  for (const auto& p : tsva) {
    if (speed_data_ptr->empty()) {
      speed_data_ptr->AppendSpeedPoint(p[1], p[0], p[2], p[3], 0.0);
    } else {
      auto back_a = speed_data_ptr->back().a();
      speed_data_ptr->AppendSpeedPoint(p[1], p[0], p[2], p[3],
                                       (p[3] - back_a) / unit_t);
    }
  }

  return !speed_data_ptr->empty();
}

std::string StCurve::DebugInfo() const {
  std::stringstream ss;
  ss << std::fixed << std::setprecision(3);
  for (const auto& origin : origins_) {
    ss << " ["
       << " s:" << origin.s << " v:" << origin.v << " a:" << origin.a
       << " t:" << origin.t << "]";
  }
  ss << " total_t:" << total_t_;

  if (cost_valid_flag_) {
    ss << " efficiency_cost:" << efficiency_cost_ << " acc_cost:" << acc_cost_
       << " jerk_cost:" << jerk_cost_ << " over_speed_cost:" << over_speed_cost_
       << " diff_cost:" << diff_cost_ << " total_cost:" << total_cost_
       << std::endl;
    return ss.str();
  }

  ss << " cost invalid";
  return ss.str();
}

StSampleCurves::StSampleCurves() {
  t_shape_curves_.assign(2000, StCurve());
}

bool StSampleCurves::Init(const StSampleParams& sample_params) {
  sample_params_ = sample_params;
  t_shape_curve_size_ = 0;
  min_sample_t_ = std::numeric_limits<double>::max();
  max_sample_t_ = 0.0;
  SampleAcc();
  SampleMaxVel();

  // params check
  return DefinitelyLess(sample_params_.start_s, sample_params.end_s) &&
         DefinitelyGreater(sample_params_.end_s - sample_params_.start_s,
                           kMinSampleS) &&
         !DefinitelyLess(sample_params_.start_v, 0.0) &&
         !DefinitelyLess(sample_params_.end_v, 0.0) &&
         DefinitelyGreater(sample_params_.speed_bound_info.max_sample_speed(),
                           0.0) &&
         DefinitelyGreater(sample_params_.speed_bound_info.min_sample_speed(),
                           0.0) &&
         DefinitelyGreater(sample_params_.speed_bound_info.max_sample_acc(),
                           0.0) &&
         DefinitelyGreater(sample_params_.speed_bound_info.min_sample_acc(),
                           0.0) &&
         DefinitelyLess(sample_params_.speed_bound_info.min_sample_dec(),
                        0.0) &&
         DefinitelyLess(sample_params_.speed_bound_info.max_sample_dec(),
                        0.0) &&
         DefinitelyLess(sample_params_.speed_bound_info.min_sample_speed(),
                        sample_params_.speed_bound_info.max_sample_speed()) &&
         DefinitelyLess(sample_params_.speed_bound_info.min_sample_acc(),
                        sample_params_.speed_bound_info.max_sample_acc()) &&
         DefinitelyLess(sample_params_.speed_bound_info.max_sample_dec(),
                        sample_params_.speed_bound_info.min_sample_dec());
}

size_t StSampleCurves::SampleProcess(const StSampleParams& sample_params) {
  if (!Init(sample_params)) {
    return 0;
  }

  for (double max_v : candidate_max_v_) {
    auto count_t = SampleCurvesUseMaxV(max_v);
    ADEBUG << " max_v: " << max_v << " sample size:" << count_t;
  }

  ADEBUG << "sample t shape curve size: " << t_shape_curve_size_;

  return t_shape_curve_size_;
}

void StSampleCurves::SampleAcc() {
  candidate_max_accs_.clear();
  candidate_min_accs_.clear();
  for (double acc = sample_params_.speed_bound_info.min_sample_acc();
       acc < sample_params_.speed_bound_info.max_sample_acc();) {
    candidate_max_accs_.emplace_back(acc);
    acc += sample_params_.unit_acc;
  }

  candidate_max_accs_.emplace_back(
      sample_params_.speed_bound_info.max_sample_acc());
  candidate_max_accs_.emplace_back(0.0);

  for (double acc = sample_params_.speed_bound_info.min_sample_dec();
       acc > sample_params_.speed_bound_info.max_sample_dec();) {
    candidate_min_accs_.emplace_back(acc);
    acc -= sample_params_.unit_acc;
  }
  candidate_min_accs_.emplace_back(
      sample_params_.speed_bound_info.max_sample_dec());
}

void StSampleCurves::SampleMaxVel() {
  candidate_max_v_.clear();

  for (double v = sample_params_.speed_bound_info.min_sample_speed();
       v < sample_params_.speed_bound_info.max_sample_speed();) {
    candidate_max_v_.emplace_back(v);
    v += sample_params_.unit_max_v;
  }
  candidate_max_v_.emplace_back(
      sample_params_.speed_bound_info.max_sample_speed());
}

bool StSampleCurves::IsTShapeParamsValid(
    const std::vector<std::array<double, 2>>& at_params) const {
  if (kMaxTShapeParamsSize < at_params.size() ||
      at_params.size() < kMinTShapeParamsSize) {
    return false;
  }

  static constexpr double kDecAccuracy = 0.01;
  auto expect_s = sample_params_.end_s - sample_params_.start_s;
  // check acc and time, acc in [min_acc, max_acc], t must bigger than zero
  for (const auto& at : at_params) {
    if (DefinitelyGreater(at[0],
                          sample_params_.speed_bound_info.max_sample_acc()) ||
        DefinitelyLess(at[0], sample_params_.speed_bound_info.max_sample_dec() -
                                  kDecAccuracy) ||
        !DefinitelyGreater(at[1], 0.0)) {
      return false;
    }
  }

  if (kMinTShapeParamsSize == at_params.size()) {
    double end_v = sample_params_.start_v + at_params[0][0] * at_params[0][1];
    if (SeemsNotEqual(end_v, sample_params_.end_v)) {
      return false;
    }

    double end_s = sample_params_.start_v * sample_params_.start_v /
                   (2 * fabs(at_params[0][0]));
    return SeemsEqual(end_s, expect_s);
  }

  if (kMidTShapeParamsSize == at_params.size()) {
    double peak_v = sample_params_.start_v + at_params[0][0] * at_params[0][1];
    double end_v = peak_v + at_params[1][0] * at_params[1][1];
    if (DefinitelyGreater(at_params[0][0], 0.0) &&
        DefinitelyLess(at_params[1][0], 0.0) &&
        DefinitelyGreater(peak_v,
                          sample_params_.speed_bound_info.min_sample_speed())) {
      return false;
    }

    if (DefinitelyGreater(peak_v,
                          sample_params_.speed_bound_info.max_sample_speed()) ||
        !DefinitelyGreater(peak_v, 0.0) ||
        SeemsNotEqual(end_v, sample_params_.end_v)) {
      return false;
    }

    double end_s = 0.5 * ((sample_params_.start_v + peak_v) * at_params[0][1] +
                          (peak_v + end_v) * at_params[1][1]);
    return SeemsEqual(end_s, expect_s);
  }

  if (kMaxTShapeParamsSize == at_params.size()) {
    // acc_2 must be zero, cruise
    if (SeemsNotEqual(at_params[1][0], 0.0)) {
      return false;
    }

    double peak_v = sample_params_.start_v + at_params[0][0] * at_params[0][1];
    double end_v = peak_v + at_params[2][0] * at_params[2][1];
    if (DefinitelyGreater(peak_v,
                          sample_params_.speed_bound_info.min_sample_speed()) &&
        DefinitelyLess(at_params[1][1], kMinCruiseTime)) {
      return false;
    }
    if (DefinitelyGreater(peak_v,
                          sample_params_.speed_bound_info.max_sample_speed()) ||
        !DefinitelyGreater(peak_v, 0.0) ||
        SeemsNotEqual(end_v, sample_params_.end_v)) {
      return false;
    }

    double end_s = 0.5 * ((sample_params_.start_v + peak_v) * at_params[0][1] +
                          (peak_v + end_v) * at_params[2][1]) +
                   peak_v * at_params[1][1];
    return SeemsEqual(end_s, expect_s);
  }

  return false;
}

void StSampleCurves::PrintTShapeParams(
    const std::vector<std::array<double, 2>>& at_params) {
  if (kMaxTShapeParamsSize < at_params.size() ||
      at_params.size() < kMinTShapeParamsSize) {
    return;
  }

  std::stringstream ss;
  ss << std::fixed << std::setprecision(3) << " TShape: ";
  for (size_t i = 0; i < at_params.size(); i++) {
    ss << " i:" << i << " acc: " << at_params[i][0]
       << " time: " << at_params[i][1];
  }

  ADEBUG << ss.str();
}

bool StSampleCurves::CalStParamsDirectly(
    const double max_acc, const double min_acc, const double max_v,
    std::vector<std::array<double, 2>>* const at_params) const {
  if (nullptr == at_params || DefinitelyLess(max_acc, 0.0) ||
      !DefinitelyLess(min_acc, 0.0) || DefinitelyLessEqual(max_v, 0.0)) {
    return false;
  }

  double s_dist = sample_params_.end_s - sample_params_.start_s;
  double comfort_stop_dist =
      sample_params_.start_v * sample_params_.start_v / (2 * fabs(min_acc));

  if (DefinitelyLessEqual(s_dist, 0.0)) {
    return false;
  }

  at_params->clear();

  static constexpr double kSAccuracy = 0.01;
  if (fabs(s_dist - comfort_stop_dist) < kSAccuracy) {
    double dec =
        sample_params_.start_v * sample_params_.start_v / (2.0 * s_dist);
    at_params->emplace_back(
        std::array<double, 2>{-dec, sample_params_.start_v / dec});
    return true;
  }

  if (comfort_stop_dist > s_dist) {
    double stop_d =
        sample_params_.start_v * sample_params_.start_v / (2 * s_dist);
    double stop_t = sample_params_.start_v / stop_d;
    // to do: limit min_acc
    stop_d = fmin(stop_d, fabs(common::VehicleConfigHelper::GetConfig()
                                   .vehicle_param()
                                   .max_deceleration()));
    at_params->emplace_back(std::array<double, 2>{-stop_d, stop_t});

    return true;
  }

  if (sample_params_.start_v > max_v) {
    double t_cruise = (s_dist - comfort_stop_dist) / max_v;
    double t_rampdown = (sample_params_.start_v - max_v) / fabs(min_acc);
    double t_dec = max_v / fabs(min_acc);

    at_params->emplace_back(std::array<double, 2>{min_acc, t_rampdown});
    at_params->emplace_back(std::array<double, 2>{0.0, t_cruise});
    at_params->emplace_back(std::array<double, 2>{min_acc, t_dec});

    return true;
  }

  if (SeemsEqual(sample_params_.start_v, max_v)) {
    double t_cruise = (s_dist - comfort_stop_dist) / max_v;
    double t_dec = max_v / fabs(min_acc);

    at_params->emplace_back(std::array<double, 2>{0.0, t_cruise});
    at_params->emplace_back(std::array<double, 2>{min_acc, t_dec});

    return true;
  }

  if (SeemsEqual(max_acc, 0.0) &&
      !DefinitelyGreater(sample_params_.start_acc, 0.0)) {
    double t_rampup = (max_v - sample_params_.start_v) /
                      sample_params_.speed_bound_info.min_sample_acc();
    if (DefinitelyLess(t_rampup, kMinShapeTime)) {
      double t_cruise = (s_dist - comfort_stop_dist) / sample_params_.start_v;
      double t_dec = sample_params_.start_v / fabs(min_acc);

      at_params->emplace_back(std::array<double, 2>{0.0, t_cruise});
      at_params->emplace_back(std::array<double, 2>{min_acc, t_dec});
      return true;
    }

    return false;
  }

  double t_rampup = (max_v - sample_params_.start_v) / max_acc;
  double t_rampdown = (max_v - sample_params_.start_v) / fabs(min_acc);
  double s_ramp =
      (sample_params_.start_v + max_v) * (t_rampup + t_rampdown) * 0.5;
  double s_rest = s_dist - s_ramp - comfort_stop_dist;

  if (s_rest > 0) {
    double t_cruise = s_rest / max_v;
    double t_dec = max_v / fabs(min_acc);

    at_params->emplace_back(std::array<double, 2>{max_acc, t_rampup});
    at_params->emplace_back(std::array<double, 2>{0, t_cruise});
    at_params->emplace_back(std::array<double, 2>{min_acc, t_dec});

    return true;
  }

  double s_rampup_rampdown = s_dist - comfort_stop_dist;
  double v_max = std::sqrt(sample_params_.start_v * sample_params_.start_v +
                           2.0 * max_acc * fabs(min_acc) * s_rampup_rampdown /
                               (max_acc + fabs(min_acc)));

  double t_acc = (v_max - sample_params_.start_v) / max_acc;
  double t_dec = v_max / fabs(min_acc);

  at_params->emplace_back(std::array<double, 2>{max_acc, t_acc});
  at_params->emplace_back(std::array<double, 2>{min_acc, t_dec});

  return true;
}

size_t StSampleCurves::SampleCurvesUseMaxV(const double max_v) {
  if (candidate_max_accs_.empty() || candidate_min_accs_.empty() ||
      candidate_max_v_.empty() ||
      DefinitelyLess(max_v,
                     sample_params_.speed_bound_info.min_sample_speed())) {
    return 0;
  }

  size_t pre_curve_count = t_shape_curve_size_;
  std::vector<std::array<double, 2>> at_params;
  if (DefinitelyLess(sample_params_.start_v, max_v)) {
    for (double max_acc : candidate_max_accs_) {
      for (double min_acc : candidate_min_accs_) {
        if (!CalStParamsDirectly(max_acc, min_acc, max_v, &at_params)) {
          continue;
        }

        if (IsTShapeParamsValid(at_params) &&
            t_shape_curve_size_ < t_shape_curves_.size() &&
            t_shape_curves_[t_shape_curve_size_].Init(
                sample_params_.start_s, sample_params_.start_v,
                sample_params_.end_s, sample_params_.end_v, at_params)) {
          t_shape_curve_size_++;
          double total_t = 0.0;
          for (const auto& at : at_params) {
            total_t += at[1];
          }
          max_sample_t_ = std::fmax(total_t, max_sample_t_);
          min_sample_t_ = std::fmin(total_t, min_sample_t_);
          PrintTShapeParams(at_params);
        }
      }
    }
  } else {
    for (double acc_3 : candidate_min_accs_) {
      if (!CalStParamsDirectly(sample_params_.speed_bound_info.max_sample_acc(),
                               acc_3, max_v, &at_params)) {
        continue;
      }

      if (IsTShapeParamsValid(at_params) &&
          t_shape_curves_[t_shape_curve_size_].Init(
              sample_params_.start_s, sample_params_.start_v,
              sample_params_.end_s, sample_params_.end_v, at_params)) {
        t_shape_curve_size_++;
        double total_t = 0.0;
        for (const auto& at : at_params) {
          total_t += at[1];
        }
        max_sample_t_ = std::fmax(total_t, max_sample_t_);
        min_sample_t_ = std::fmin(total_t, min_sample_t_);
        PrintTShapeParams(at_params);
      }
    }
  }

  ADEBUG << "start v: " << sample_params_.start_v
         << " start acc:" << sample_params_.start_acc
         << " end_s: " << sample_params_.end_s << " max_v: " << max_v
         << " count: " << t_shape_curve_size_ - pre_curve_count;

  return t_shape_curve_size_ - pre_curve_count;
}

}  // namespace planning
}  // namespace TL
