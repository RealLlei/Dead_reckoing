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
 * @file
 **/

#pragma once

#include <memory>

#include "common/status/status.h"
#include "planning/common/frame.h"
#include "planning/tasks/task.h"

namespace TL {
namespace planning {

class Decider : public Task {
 public:
  explicit Decider(const TaskConfig& config);
  Decider(const TaskConfig& config,
          const std::shared_ptr<DependencyInjector>& injector);
  ~Decider() override = default;

  TL::common::Status Execute(
      Frame* frame, ReferenceLineInfo* reference_line_info) override;

  TL::common::Status Execute(Frame* frame) override;

 protected:
  virtual TL::common::Status Process(
      Frame* frame, ReferenceLineInfo* reference_line_info) {
    UNUSED(frame);
    UNUSED(reference_line_info);
    return TL::common::Status::OK();
  }

  virtual TL::common::Status Process(Frame* frame) {
    UNUSED(frame);
    return TL::common::Status::OK();
  }
};

}  // namespace planning
}  // namespace TL
