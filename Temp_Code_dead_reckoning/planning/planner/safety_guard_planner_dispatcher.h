/*
 * Copyright (c) TL Technologies Co., Ltd. 2022. All rights reserved.
 * Description:  safety_guard_planner_dispatcher.h
 */

#pragma once

#include <memory>

#include "planning/planner/planner_dispatcher.h"

namespace TL {
namespace planning {

class SafetyGuardPlannerDispatcher final : public PlannerDispatcher {
 public:
  SafetyGuardPlannerDispatcher() = default;
  ~SafetyGuardPlannerDispatcher() override = default;

  std::unique_ptr<Planner> DispatchPlanner(
      const PlanningConfig& planning_config,
      const std::shared_ptr<DependencyInjector>& injector) override;
};

}  // namespace planning
}  // namespace TL
