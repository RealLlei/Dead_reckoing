/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "planning/tasks/task.h"

#include <memory>

#include "planning/proto/planning_config.pb.h"

namespace TL {
namespace planning {

using TL::common::Status;

Task::Task(const TaskConfig& config)
    : config_(config), name_(TaskConfig::TaskType_Name(config.task_type())) {}

Task::Task(const TaskConfig& config,
           const std::shared_ptr<DependencyInjector>& injector)
    : config_(config),
      name_(TaskConfig::TaskType_Name(config.task_type())),
      injector_(injector) {}

const std::string& Task::Name() const {
  return name_;
}

Status Task::Execute(Frame* frame, ReferenceLineInfo* reference_line_info) {
  frame_ = frame;
  reference_line_info_ = reference_line_info;
  return Status::OK();
}

Status Task::Execute(Frame* frame) {
  frame_ = frame;
  return Status::OK();
}

void Task::RecordValueInfo(ReferenceLineInfo* reference_line_info,
                           const std::string& name, const double value) {
  // if (!FLAGS_enable_record_debug) {
  //   ADEBUG << "Skip record debug info";
  //   return;
  // }
  if (reference_line_info == nullptr) {
    AERROR << "Reference line info is null.";
    return;
  }

  auto* ptr_latency_stats = reference_line_info->mutable_latency_stats();

  auto* ptr_var = ptr_latency_stats->add_var_name();
  ptr_var->set_name(name);
  ptr_var->set_time_ms(value);
}

}  // namespace planning
}  // namespace TL
