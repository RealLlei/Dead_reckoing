/*
 * Copyright (c) 2022 TL
 *
 * Author: Ling Peng
 */

#include <memory>
#include <set>

#include "planning/self_simulator/sim_dummy_obs/obs_manager.h"
#include "planning/self_simulator/sim_dummy_obs/obstacle/dummy_obstacle_base.h"

namespace TL {
namespace simdummy {
double DummyObstacleBase::GetTheta() const {
  return theta_;
}

double DummyObstacleBase::GetX() const {
  return x_;
}

double DummyObstacleBase::GetY() const {
  return y_;
}

double DummyObstacleBase::GetVx() const {
  return vx_;
}

double DummyObstacleBase::GetVy() const {
  return vy_;
}

double DummyObstacleBase::GetV() const {
  return v_;
}

void DummyObstacleBase::SetV(double v) {
  v_ = v;
}

bool DummyObstacleBase::IsDisplay() const {
  return is_display_;
}

void DummyObstacleBase::setIsDisplay(bool isDisplay) {
  is_display_ = isDisplay;
}

bool DummyObstacleBase::IsNeededRemove() const {
  return is_needed_remove_;
}

void DummyObstacleBase::SetIsNeededRemove(bool isNeededRemove) {
  is_needed_remove_ = isNeededRemove;
}

bool DummyObstacleBase::IsStatic() const {
  return is_static_;
}

void DummyObstacleBase::SetIsStatic(bool isStatic) {
  is_static_ = isStatic;
}

ObsManagerBase* DummyObstacleBase::GetObstacleManagerPtr() const {
  return obstacle_manager_ptr_;
}

void DummyObstacleBase::SetObstacleManagerPtr(
    ObsManagerBase* obstacleManagerPtr) {
  obstacle_manager_ptr_ = obstacleManagerPtr;
}

void DummyObstacleBase::InitializeObs(int32_t id, double length_, double width,
                                      double height, double x, double y,
                                      double theta, double v, boost::any type) {
  if (!IdDuplicatedCheck(id)) {
    SetState(id, length_, width, height, x, y, theta, v, type);
    setIsDisplay(true);
  }
}

bool DummyObstacleBase::IdDuplicatedCheck(int id) {
  auto dummy_obs_list = obstacle_manager_ptr_->getDummyObstaclesList();
  std::set<int> id_set;
  for (auto& obs : dummy_obs_list) {
    id_set.insert(obs->GetObsId());
  }
  if (!id_set.insert(id).second) {
    AERROR << "create obs failed.  obs_id:" << id
           << " is duplicated with other obs id.";
    return true;
  }
  return false;
}

bool DummyObstacleBase::ObsCollisionCheck(
    std::shared_ptr<DummyObstacleBase> dummy_obs_ptr) {
  return false;
}

double DummyObstacleBase::GetS() const {
  return s_;
}

void DummyObstacleBase::SetS(double s) {
  s_ = s;
}

double DummyObstacleBase::GetL() const {
  return l_;
}

void DummyObstacleBase::SetL(double l) {
  l_ = l;
}

}  // namespace simdummy
}  // namespace TL
