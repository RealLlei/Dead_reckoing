/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file path_speed_optimizer.cc
 **/

#include "planning/tasks/optimizers/path_speed_optimizer.h"

#include <memory>
#include "planning/common/trajectory/discretized_trajectory.h"

namespace TL::planning {

using TL::common::Status;

PathSpeedOptimizer::PathSpeedOptimizer(const TaskConfig& config)
    : Task(config) {}

PathSpeedOptimizer::PathSpeedOptimizer(
    const TaskConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : Task(config, injector) {}

Status PathSpeedOptimizer::Execute(
    Frame* frame, ReferenceLineInfo* const reference_line_info) {
  Task::Execute(frame, reference_line_info);
  DiscretizedTrajectory discretized_trajectory;
  auto ret = Process(reference_line_info->reference_line(),
                     frame->PlanningStartPoint(), &discretized_trajectory);
  // RecordDebugInfo(reference_line_info->path_data());
  if (ret != Status::OK() && reference_line_info->path_data().Empty()) {
    reference_line_info->SetDrivable(false);
    AERROR << "Reference Line " << reference_line_info->Lanes().Id()
           << " is not drivable after " << Name();
  }
  reference_line_info->SetTrajectory(discretized_trajectory);
  return ret;
}

Status PathSpeedOptimizer::Execute(Frame* frame) {
  Task::Execute(frame);
  return Process(frame);
}

common::Status PathSpeedOptimizer::Process(Frame* frame) {
  UNUSED(frame);
  return common::Status::OK();
}

TL::common::Status PathSpeedOptimizer::Process(
    const ReferenceLine& reference_line,
    const common::TrajectoryPoint& init_point, PathData* path_data,
    SpeedData* speed_data) {
  UNUSED(reference_line);
  UNUSED(init_point);
  UNUSED(path_data);
  UNUSED(speed_data);
  return common::Status::OK();
}

TL::common::Status PathSpeedOptimizer::Process(
    const ReferenceLine& reference_line,
    const common::TrajectoryPoint& init_point,
    DiscretizedTrajectory* discretized_trajectory) {
  UNUSED(reference_line);
  UNUSED(init_point);
  UNUSED(discretized_trajectory);
  return common::Status::OK();
}

void PathSpeedOptimizer::RecordDebugInfo(const PathData& path_data,
                                         const SpeedData& speed_data) {
  const auto& path_points = path_data.discretized_path();
  auto* ptr_optimized_path = reference_line_info_->mutable_debug()
                                 ->mutable_planning_data()
                                 ->add_path();
  ptr_optimized_path->set_name(Name());
  ptr_optimized_path->mutable_path_point()->CopyFrom(
      {path_points.begin(), path_points.end()});

  auto* ptr_optimized_speed = reference_line_info_->mutable_debug()
                                  ->mutable_planning_data()
                                  ->add_speed_plan();
  ptr_optimized_speed->set_name(Name());
  ptr_optimized_speed->mutable_speed_point()->CopyFrom(
      {speed_data.begin(), speed_data.end()});
}

}  // namespace TL::planning
