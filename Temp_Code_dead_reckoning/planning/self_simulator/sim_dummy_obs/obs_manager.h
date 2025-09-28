/*
 * Copyright (c) 2022 TL
 *
 * Author: Ling Peng
 */

#pragma once

#include <deque>
#include <list>
#include <memory>
#include <set>
#include <typeinfo>

#include "planning/self_simulator/sim_dummy_gflags.h"
#include "planning/self_simulator/sim_dummy_obs/common/util_com.h"
#include "planning/self_simulator/sim_dummy_obs/obstacle/dummy_obstacle_base.h"
#include "planning/self_simulator/sim_dummy_obs/obstacle/perception_obs/front_obs.h"
#include "planning/self_simulator/sim_dummy_obs/obstacle/perception_obs/lane_follow_obs.h"
#include "planning/self_simulator/sim_dummy_obs/obstacle/perception_obs/opposite_lane_obs_1.h"
#include "planning/self_simulator/sim_dummy_obs/obstacle/perception_obs/relative_position_obs.h"
#include "planning/self_simulator/sim_dummy_obs/obstacle/perception_obs/side_lane_obs.h"
#include "planning/self_simulator/sim_dummy_obs/obstacle/perception_obs/side_lane_obs_2.h"
#include "planning/self_simulator/sim_dummy_obs/obstacle/planning_obs/front_obs_planning.h"

namespace TL {
namespace simdummy {

/**
 * Dummy Obstacles Manager (base class)
 */
class ObsManagerBase {
 public:
  virtual ~ObsManagerBase() = default;

  /**
   * initial dummy obstacles deque.
   */
  virtual void RegisterObsList() = 0;

  /**
   * @brief Register obs and execute obstacles check
   * @return
   */
  bool InitializeObsList();

  /**
   * erase dummy obstacle forever in obstacle list when obstacle's
   * is_needed_remove_ is true.
   */
  virtual void RemoveFromList();

  /**
   * interface
   */
  virtual void Update(DummyObsInputDataBase* Data) = 0;

  /**
   * check obstacle type is compatible with manager type.
   */
  virtual bool CheckObsTypeIsOk(std::shared_ptr<DummyObstacleBase> obs_ptr) = 0;

  /**
   * check the obstacle list if the obs id is duplicated or the type is
   * compatible with manage type.
   */
  void ObstacleListCheck();

  void SortObsList();

  const std::deque<std::shared_ptr<DummyObstacleBase>>& getDummyObstaclesList()
      const;

 protected:
  DummyObsInputDataBase data_;

  std::deque<std::shared_ptr<DummyObstacleBase>> dummy_obstacles_;
};

/**
 * lp: Perception dummy obs manager
 */
class PerceptionObsManager final : public ObsManagerBase {
 public:
  void RegisterObsList() override;

  void Update(
      double delta_time,
      std::shared_ptr<const TL::planning::ADCTrajectory> current_trajectory_,
      std::shared_ptr<common::PathPoint> adc_position_,
      std::shared_ptr<TL::perception::PerceptionObstacles> perception,
      std::shared_ptr<TL::planning::LocalView> local_view,
      std::shared_ptr<hdmap::HDMap> map_ptr);

  void Update(DummyObsInputDataBase* Data) override;

  bool CheckObsTypeIsOk(std::shared_ptr<DummyObstacleBase> obs_ptr) override {
    if (obs_ptr != nullptr) {
      return obs_ptr->GetObsBaseType() ==
             ObstacleBaseType::PerceptionObstacleType;
    } else {
      return false;
    }
  }

  void SpawnNewObstacle(const std::shared_ptr<hdmap::HDMap>& map_ptr);

  GET_SINGLETON(PerceptionObsManager);

 private:
  PerceptionObsManager() : ObsManagerBase() {}

  uint obs_id_ = 0;
};

/**
 * lp: PredictionObstacleType dummy obs manager
 */
class PredictionObsManager final : public ObsManagerBase {
 public:
  void RegisterObsList() override;

  void Update(DummyObsInputDataBase* Data) override;

  GET_SINGLETON(PredictionObsManager);

  bool CheckObsTypeIsOk(std::shared_ptr<DummyObstacleBase> obs_ptr) override {
    if (obs_ptr != nullptr) {
      return obs_ptr->GetObsBaseType() ==
             ObstacleBaseType::PredictionObstacleType;
    } else {
      return false;
    }
  }

 private:
  PredictionObsManager() : ObsManagerBase() {}
};

/**
 * lp: PlanningObstacleType dummy obs manager
 */
class PlanningObsManager final : public ObsManagerBase {
 public:
  void RegisterObsList() override;

  void Update(DummyObsInputDataBase* Data) override;

  GET_SINGLETON(PlanningObsManager);

  bool CheckObsTypeIsOk(std::shared_ptr<DummyObstacleBase> obs_ptr) override {
    if (obs_ptr != nullptr) {
      return obs_ptr->GetObsBaseType() ==
             ObstacleBaseType::PlanningObstacleType;
    } else {
      return false;
    }
  }

 private:
  PlanningObsManager() : ObsManagerBase() {}
};

}  // namespace simdummy
}  // namespace TL
