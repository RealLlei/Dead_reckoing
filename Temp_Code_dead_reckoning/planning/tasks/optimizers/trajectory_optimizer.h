/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
 * @file trajectory_optimizer.h
 **/

#pragma once

#include <memory>

#include "common/status/status.h"
#include "planning/tasks/task.h"
#include "proto/common/pnc_point.pb.h"

namespace TL {
namespace planning {

class TrajectoryOptimizer : public Task {
 public:
  explicit TrajectoryOptimizer(const TaskConfig& config);
  TrajectoryOptimizer(const TaskConfig& config,
                      const std::shared_ptr<DependencyInjector>& injector);
  ~TrajectoryOptimizer() override = default;

  TL::common::Status Execute(Frame* frame) override;

 protected:
  virtual TL::common::Status Process() = 0;
};

}  // namespace planning
}  // namespace TL
