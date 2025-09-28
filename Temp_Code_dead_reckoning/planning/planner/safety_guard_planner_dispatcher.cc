/*
 * Copyright (c) TL Technologies Co., Ltd. 2022. All rights reserved.
 * Description:  safety_guard_planner_dispatcher.cc
 */

#include "planning/planner/safety_guard_planner_dispatcher.h"

#include <memory>

#include "planning/proto/planning_config.pb.h"

namespace TL {
namespace planning {

std::unique_ptr<Planner> SafetyGuardPlannerDispatcher::DispatchPlanner(
    const PlanningConfig& planning_config,
    const std::shared_ptr<DependencyInjector>& injector) {
  return planner_factory_.CreateObject(
      planning_config.safety_guard_planning_config().planner_type(), injector);
}

}  // namespace planning
}  // namespace TL
