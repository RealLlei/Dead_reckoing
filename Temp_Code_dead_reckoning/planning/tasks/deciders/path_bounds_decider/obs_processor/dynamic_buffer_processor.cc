/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  planning dynamic obstacle buffer processor
 * Author: ROC
 */

#include "planning/tasks/deciders/path_bounds_decider/obs_processor/dynamic_buffer_processor.h"
#include "common/file/file.h"
#include "common/math/linear_interpolation.h"
#include "common/math/math_utils.h"
#include "planning/common/planning_gflags.h"

namespace TL {
namespace planning {

DynamicBufferProcessor::DynamicBufferProcessor() : init_status_(Init()) {}

bool DynamicBufferProcessor::Init() {
  if (!common::GetProtoFromFile(FLAGS_dynamic_buffer_calculate_config_file,
                                &dynamic_buffer_config_)) {
    AERROR << "Failed to load config file "
           << FLAGS_dynamic_buffer_calculate_config_file;
    return false;
  }

  lower_threshold_ = dynamic_buffer_config_.lower_threshold();
  upper_threshold_ = dynamic_buffer_config_.upper_threshold();

  return true;
}

double DynamicBufferProcessor::BufferCalculate(const double obs_s,
                                               const double obs_l,
                                               const double cur_v) {
  if (!init_status_) {
    return 0.0;
  }

  double s_sbuffer = common::math::InterpolationOne(
      obs_s, dynamic_buffer_config_.dynamic_buffer_vsl().threshold_s(),
      dynamic_buffer_config_.dynamic_buffer_vsl().buffer_s());
  double l_sbuffer = common::math::InterpolationOne(
      obs_l, dynamic_buffer_config_.dynamic_buffer_vsl().threshold_l(),
      dynamic_buffer_config_.dynamic_buffer_vsl().buffer_l());
  double v_sbuffer = common::math::InterpolationOne(
      cur_v, dynamic_buffer_config_.dynamic_buffer_vsl().threshold_v(),
      dynamic_buffer_config_.dynamic_buffer_vsl().buffer_v());

  double buffer_calculate = v_sbuffer * (l_sbuffer + s_sbuffer);
  buffer_calculate =
      common::math::Clamp(buffer_calculate, lower_threshold_, upper_threshold_);

  return buffer_calculate;
}

double DynamicBufferProcessor::BufferCalculate(const double cur_v,
                                               const double relative_v,
                                               const Obstacle& obstacle) {
  UNUSED(obstacle);
  UNUSED(cur_v);
  UNUSED(relative_v);
  if (!init_status_) {
    return 0.0;
  }

  double v_sbuffer = common::math::InterpolationOne(
      cur_v, dynamic_buffer_config_.dynamic_buffer_vrv().threshold_v(),
      dynamic_buffer_config_.dynamic_buffer_vrv().buffer_v());

  double rv_sbuffer = common::math::InterpolationOne(
      cur_v, dynamic_buffer_config_.dynamic_buffer_vrv().threshold_rv(),
      dynamic_buffer_config_.dynamic_buffer_vrv().buffer_rv());

  double buffer_calculate = v_sbuffer * rv_sbuffer;
  buffer_calculate =
      common::math::Clamp(buffer_calculate, lower_threshold_, upper_threshold_);

  return buffer_calculate;
}

double DynamicBufferProcessor::StaticObsDynamicBufferCalculate(
    const double cur_v) const {
  if (!init_status_) {
    return 0.0;
  }

  double v_sbuffer = common::math::InterpolationOne(
      cur_v, dynamic_buffer_config_.static_obs_buffer().threshold_v(),
      dynamic_buffer_config_.static_obs_buffer().buffer_v());

  double buffer_calculate =
      common::math::Clamp(v_sbuffer, lower_threshold_, upper_threshold_);

  return buffer_calculate;
}

double DynamicBufferProcessor::DynamicObsDynamicBufferCalculate(
    const double cur_v) const {
  if (!init_status_) {
    return 0.0;
  }

  double v_sbuffer = common::math::InterpolationOne(
      cur_v, dynamic_buffer_config_.dynimic_obs_buffer().threshold_v(),
      dynamic_buffer_config_.dynimic_obs_buffer().buffer_v());

  double buffer_calculate =
      common::math::Clamp(v_sbuffer, lower_threshold_, upper_threshold_);

  return buffer_calculate;
}

}  // namespace planning
}  // namespace TL
