/*
 * Copyright (c) 2022 TL
 *
 * Author: Ling Peng
 */

#pragma once

#include "planning/self_simulator/sim_dummy_obs/obstacle/dummy_obstacle_base.h"

namespace TL {
namespace simdummy {
class PlanningObstacleBase : public DummyObstacleBase {
 public:
  void SetState(int32_t id, double length_, double width, double height,
                double x, double y, double theta, double v,
                boost::any type) override;

  void Update(double x, double y, double theta, double v) override;

  void Update(double delta_time, double theta) override;

  int GetObsId() override { return planning_obs_.PerceptionId(); }

  bool IsNormal(const call_func_& func_) final { return func_(); }

  bool StartDisplay(const DummyObsInputDataBase& data) override = 0;

  bool StopDisplay(const DummyObsInputDataBase& data) override = 0;

  void UpdateState(const DummyObsInputDataBase& data) override = 0;

  boost::any GetObs() override { return boost::any(planning_obs_); }

  PlanningObsManager* GetObstacleManagerPtr() const {
    return reinterpret_cast<PlanningObsManager*>(
        DummyObstacleBase::GetObstacleManagerPtr());
  }

  ObstacleBaseType GetObsBaseType() override {
    return ObstacleBaseType::PlanningObstacleType;
  }

 private:
  TL::planning::Obstacle planning_obs_;
};
}  // namespace simdummy
}  // namespace TL
