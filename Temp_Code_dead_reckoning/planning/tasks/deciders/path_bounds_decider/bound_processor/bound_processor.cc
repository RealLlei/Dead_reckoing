/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  planning bound processor base class
 * Author: ROC
 */

#include "planning/tasks/deciders/path_bounds_decider/bound_processor/bound_processor.h"

#include <memory>

namespace TL {
namespace planning {

BoundProcessor::BoundProcessor(const TaskConfig& config) : config_(config) {}

BoundProcessor::BoundProcessor(
    const std::shared_ptr<DependencyInjector>& injector,
    const TaskConfig& config)
    : injector_(injector), config_(config) {}

bool BoundProcessor::BoundInit(const PathInfo::LaneBorrowInfo& lane_borrow_info,
                               std::string* const blocking_obstacle_id,
                               std::string* const borrow_lane_type) {
  if (blocking_obstacle_id == nullptr || borrow_lane_type == nullptr) {
    AERROR << "blocking_obstacle_id or borrow_lane_type is nullptr";
    return false;
  }

  UNUSED(lane_borrow_info);
  UNUSED(blocking_obstacle_id);
  UNUSED(borrow_lane_type);
  return true;
}

bool BoundProcessor::BlockingIDInit(std::string* const blocking_obstacle_id) {
  if (blocking_obstacle_id == nullptr) {
    AERROR << "blocking_obstacle_id is nullptr";
    return false;
  }

  UNUSED(blocking_obstacle_id);
  return true;
}

}  // namespace planning
}  // namespace TL
