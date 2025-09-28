/*
 * Copyright (c) 2022 TL
 *
 * Author: Ling Peng
 */

#ifndef EUROPA_DUMMY_OBSTACLE_BASE_H
#define EUROPA_DUMMY_OBSTACLE_BASE_H

#include <cmath>

#include <functional>
#include <iomanip>
#include <memory>
#include <typeinfo>
#include <utility>

#include <boost/any.hpp>

#include "common/adapters/adapter_gflags.h"
#include "common/util/util.h"
#include "dreamview/backend/common/dreamview_gflags.h"
#include "planning/common/obstacle.h"
#include "planning/localview/local_view.h"
#include "planning/self_simulator/sim_dummy_obs/base_sim_data.h"
#include "planning/self_simulator/sim_dummy_obs/common/util_com.h"

#include "proto/localization/localization.pb.h"
#include "proto/map/navigation.pb.h"
#include "proto/planning/planning.pb.h"
#include "proto/prediction/prediction_obstacle.pb.h"

namespace TL {
namespace simdummy {

// lp: loop
class ObsManagerBase;
class PerceptionObsManager;
class PlanningObsManager;
class PredictionObsManager;

union ObsType {
  TL::perception::PerceptionObstacle perception_obs_;
  TL::prediction::PredictionObstacles prediction_obs;
  TL::planning::Obstacle planning_obs_;
};

using call_func_ = std::function<bool()>;

enum class ObstacleBaseType {
  PerceptionObstacleType,
  PredictionObstacleType,
  PlanningObstacleType
};

/**
 * abstract obstacle class, all obstacle class base.
 */
class DummyObstacleBase {
 public:
  virtual ~DummyObstacleBase() {}

  void InitializeObs(int32_t id, double length_, double width, double height,
                     double x, double y, double theta, double v,
                     boost::any type);

  /**
   * lp: set obstacle init state.
   */
  virtual void SetState(int32_t id, double length_, double width, double height,
                        double x, double y, double theta, double v,
                        boost::any type) = 0;

  /**
   * @brief check id is duplicated with other obs id or not.
   * @return true, duplicated with other's, can't display. false, not duplicated
   * with other, can display.
   */
  bool IdDuplicatedCheck(int id);

  /**
   * @brief check ego obs is collision with dummy_obs_ptr.
   */
  bool ObsCollisionCheck(std::shared_ptr<DummyObstacleBase> dummy_obs_ptr);

  /**
   * @func update obstacle state
   */
  virtual void Update(double x, double y, double theta, double v) = 0;

  /**
   * @func update obstacle state
   */
  virtual void Update(double delta_time, double theta) = 0;

  /**
   * determine whether the obstacle state is normal.
   */
  virtual bool IsNormal(const call_func_& func_) = 0;

  /**
   * determine whether the obstacle should be display.
   */
  virtual bool StartDisplay(const DummyObsInputDataBase& data) = 0;

  /**
   * determine whether the obstacle should be disappear.
   */
  virtual bool StopDisplay(const DummyObsInputDataBase& data) = 0;

  /**
   * obstacle interface.
   */
  virtual void UpdateState(const DummyObsInputDataBase& data) = 0;

  /**
   * @return obs base type for list check.
   */
  virtual ObstacleBaseType GetObsBaseType() = 0;

 public:
  // lp: inherited class has this GetObs
  virtual boost::any GetObs() = 0;

  virtual int GetObsId() = 0;

  double GetTheta() const;

  double GetX() const;

  double GetY() const;

  double GetVx() const;

  double GetVy() const;

  double GetV() const;

  void SetV(double v);

  bool IsDisplay() const;

  void setIsDisplay(bool isDisplay);

  bool IsNeededRemove() const;

  void SetIsNeededRemove(bool isNeededRemove);

  bool IsStatic() const;

  void SetIsStatic(bool isStatic);

  ObsManagerBase* GetObstacleManagerPtr() const;

  void SetObstacleManagerPtr(ObsManagerBase* obstacleManagerPtr);

  double GetS() const;

  void SetS(double s);

  double GetL() const;

  void SetL(double s);

 protected:
  double theta_;
  double x_;
  double y_;
  double vx_;
  double vy_;
  double v_;
  double s_;
  double l_;

  // lp: true, the obstacle will start update and add to cyber.
  // false, the obstacles will disappear in dreamview.
  bool is_display_ = false;

  // lp: obstacle is static
  bool is_static_ = false;

  // lp: true, the obstacle will erase in manager.
  bool is_needed_remove_ = false;

 private:
  // lp: remember don't delete the pointer-object in destructor!!!!!!
  ObsManagerBase* obstacle_manager_ptr_;
};
}  // namespace simdummy
}  // namespace TL

#endif  // EUROPA_DUMMY_OBSTACLE_BASE_H
