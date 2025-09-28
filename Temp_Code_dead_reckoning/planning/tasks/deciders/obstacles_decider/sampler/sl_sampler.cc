/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2023. All rights reserved.
 * Description:  sl_sampler.cc
 */

#include "planning/tasks/deciders/obstacles_decider/sampler/sl_sampler.h"

#include <algorithm>
#include <memory>
#include <utility>
#include "common/file/log.h"

namespace TL::planning {

using State = std::array<double, 3>;
using Condition = std::pair<State, double>;

namespace {
constexpr double kSampleMaxS = 150.0;
}  // namespace

SlSampler::SlSampler(
    const TL::planning::ObstaclesDeciderConfig_SlSamplerConfig&
        sl_sampler_config) {
  sample_conditions_.clear();
  if (sl_sampler_config.has_sample_relative_l() &&
      sl_sampler_config.has_sample_relative_s()) {
    std::vector<Condition> left_nudge;
    Condition left_nudge_first_condition = {
        {sl_sampler_config.sample_relative_l(), 0.0, 0.0},
        sl_sampler_config.sample_relative_s()};
    Condition left_nudge_second_condition = {
        {0.0, 0.0, 0.0},
        std::max(kSampleMaxS - sl_sampler_config.sample_relative_s(), 0.0)};
    left_nudge.push_back(std::move(left_nudge_first_condition));
    left_nudge.push_back(std::move(left_nudge_second_condition));

    std::vector<Condition> right_nudge;
    Condition right_nudge_first_condition = {
        {-sl_sampler_config.sample_relative_l(), 0.0, 0.0},
        sl_sampler_config.sample_relative_s()};
    Condition right_nudge_second_condition = {
        {0.0, 0.0, 0.0},
        std::max(kSampleMaxS - sl_sampler_config.sample_relative_s(), 0.0)};
    right_nudge.push_back(std::move(right_nudge_first_condition));
    right_nudge.push_back(std::move(right_nudge_second_condition));

    std::vector<Condition> go_straight;
    Condition go_straight_first = {{0.0, 0.0, 0.0},
                                   sl_sampler_config.sample_relative_s()};
    Condition go_straight_second = {
        {0.0, 0.0, 0.0},
        std::max(kSampleMaxS - sl_sampler_config.sample_relative_s(), 0.0)};
    go_straight.push_back(std::move(go_straight_first));
    go_straight.push_back(std::move(go_straight_second));

    sample_conditions_.push_back(std::move(left_nudge));
    sample_conditions_.push_back(std::move(right_nudge));
    sample_conditions_.push_back(std::move(go_straight));
  }
}

void SlSampler::Init(const std::array<double, 3>& init_lat_state) {
  init_lat_state_ = init_lat_state;
}

void SlSampler::GenerateLateralTrajectoryBundle(
    std::vector<std::shared_ptr<SlCurve>>* const sl_curves) const {
  if (sl_curves == nullptr) {
    return;
  }
  sl_curves->reserve(sample_conditions_.size());
  for (const auto& l_condition : sample_conditions_) {
    ADEBUG << "first seg end l:" << l_condition.at(0).first.at(0)
           << ", second seg end l:" << l_condition.at(1).first.at(0);
    std::shared_ptr<SlCurve> sl_curve = std::make_shared<SlCurve>();
    sl_curve->InitPiecewiseSlCurve(init_lat_state_, l_condition);
    sl_curves->push_back(std::move(sl_curve));
  }
}

}  // namespace TL::planning
