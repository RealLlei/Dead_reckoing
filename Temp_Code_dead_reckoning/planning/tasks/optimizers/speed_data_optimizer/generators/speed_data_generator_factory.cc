/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

/**
 * @file speed_Stage_factory.cc
 **/

#include "planning/tasks/optimizers/speed_data_optimizer/generators/speed_data_generator_factory.h"

#include "planning/tasks/optimizers/speed_data_optimizer/generators/vt_sample/forward_vt_sample_generator.h"
#include "planning/tasks/optimizers/speed_data_optimizer/generators/vt_sample/reverse_vt_sample_generator.h"

#include "planning/proto/speed_evaluator_config.pb.h"
#include "planning/proto/task_config.pb.h"

namespace TL {
namespace planning {

TL::common::util::Factory<
    SpeedDataGeneratorConfig::GeneratorType, SpeedDataGenerator,
    SpeedDataGenerator* (*)(const SpeedDataGeneratorConfig& config),
    std::unordered_map<
        SpeedDataGeneratorConfig::GeneratorType,
        SpeedDataGenerator* (*)(const SpeedDataGeneratorConfig& config),
        std::hash<int>>>
    SpeedDataGeneratorFactory::speed_data_generator_factory_;

void SpeedDataGeneratorFactory::Init() {
  // deciders
  speed_data_generator_factory_.Register(
      SpeedDataGeneratorConfig::FORWARD_VT_SAMPLE_GENERATOR,
      [](const SpeedDataGeneratorConfig& config) -> SpeedDataGenerator* {
        return new ForwardVtSampleGenerator(config);
      });
  speed_data_generator_factory_.Register(
      SpeedDataGeneratorConfig::REVERSE_VT_SAMPLE_GENERATOR,
      [](const SpeedDataGeneratorConfig& config) -> SpeedDataGenerator* {
        return new ReverseVtSampleGenerator(config);
      });
}

std::unique_ptr<SpeedDataGenerator>
SpeedDataGeneratorFactory::CreateSpeedDataGenerator(
    const SpeedDataGeneratorConfig& speed_data_generator_config) {
  return speed_data_generator_factory_.CreateObject(
      speed_data_generator_config.generator_type(),
      speed_data_generator_config);
}

}  // namespace planning
}  // namespace TL
