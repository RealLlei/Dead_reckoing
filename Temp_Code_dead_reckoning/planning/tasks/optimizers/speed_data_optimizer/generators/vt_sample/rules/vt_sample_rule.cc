/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file vt_sample_recorder.cc
 **/

#include "planning/tasks/optimizers/speed_data_optimizer/generators/vt_sample/rules/vt_sample_rule.h"

namespace TL {
namespace planning {

VtSampleRule::VtSampleRule(const VtSamplerConfig& config) : config_(config) {}

}  // namespace planning
}  // namespace TL
