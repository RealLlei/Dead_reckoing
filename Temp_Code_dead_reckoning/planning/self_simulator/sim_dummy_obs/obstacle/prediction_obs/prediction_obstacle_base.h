/*
 * Copyright (c) 2022 TL
 *
 * Author: Ling Peng
 */

#pragma once

#include "planning/self_simulator/sim_dummy_obs/obstacle/dummy_obstacle_base.h"

namespace TL {
namespace simdummy {

class PredictionObstacleBase : public DummyObstacleBase {
 public:
  PredictionObstacleBase() = default;

  virtual ~PredictionObstacleBase();

  void SetState(int32_t id, double length_, double width, double height,
                double x, double y, double theta, double v,
                boost::any type) override;

  void Update(double x, double y, double theta, double v) override;

  void Update(double delta_time, double theta) override;

  int GetObsId() override { return prediction_obs_.perception_obstacle().id(); }

  bool StartDisplay(const DummyObsInputDataBase& data) override = 0;

  bool StopDisplay(const DummyObsInputDataBase& data) override = 0;

  void UpdateState(const DummyObsInputDataBase& data) override = 0;

  boost::any GetObs() override { return boost::any(prediction_obs_); }

  PredictionObsManager* GetObstacleManagerPtr() const {
    return reinterpret_cast<PredictionObsManager*>(
        DummyObstacleBase::GetObstacleManagerPtr());
  }

  ObstacleBaseType GetObsBaseType() override {
    return ObstacleBaseType::PredictionObstacleType;
  }

 private:
  TL::prediction::PredictionObstacle prediction_obs_;
};
}  // namespace simdummy
}  // namespace TL
