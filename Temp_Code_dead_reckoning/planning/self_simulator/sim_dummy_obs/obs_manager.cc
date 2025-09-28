/*
 * Copyright (c) 2022 TL
 *
 * Author: Ling Peng
 */

#include "planning/self_simulator/sim_dummy_obs/obs_manager.h"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include "common/math/math_utils.h"
#include "common/time/clock.h"
#include "map/hdmap/hdmap_util.h"
#include "map/hdmap/path.h"
#include "planning/self_simulator/sim_dummy_obs/obstacle/perception_obs/lane_follow_obs.h"
#include "proto/perception/perception_obstacle.pb.h"

namespace TL {
namespace simdummy {

void ObsManagerBase::RemoveFromList() {
  for (auto it = dummy_obstacles_.begin(); it != dummy_obstacles_.end();) {
    if ((*it)->IsNeededRemove()) {
      it = dummy_obstacles_.erase(it);
    } else {
      ++it;
    }
  }
}

void ObsManagerBase::ObstacleListCheck() {
  // lp: set obstacle manager pointer in every obstacle in obstacle list.
  std::set<signed int> obs_id_set;
  for (auto it = dummy_obstacles_.begin(); it != dummy_obstacles_.end(); ++it) {
    if (CheckObsTypeIsOk(*it)) {
      (*it)->SetObstacleManagerPtr(this);
    } else {
      AERROR << "this obstacle (index:" << it - dummy_obstacles_.begin()
             << ") is not belong this manager.";
      dummy_obstacles_.erase(it);
    }
  }
}

bool ObsManagerBase::InitializeObsList() {
  if (!FLAGS_only_enable_sim_adc_side_dummy_obs) {
    return true;
  }
  RegisterObsList();
  ObstacleListCheck();
  return !dummy_obstacles_.empty();
}

const std::deque<std::shared_ptr<DummyObstacleBase>>&
ObsManagerBase::getDummyObstaclesList() const {
  return dummy_obstacles_;
}

void ObsManagerBase::SortObsList() {
  /*auto comp = [](std::shared_ptr<DummyObstacleBase> obs1,
                 std::shared_ptr<DummyObstacleBase> obs2) -> bool { ; };
  std::sort(dummy_obstacles_.begin(), dummy_obstacles_.end(), comp);*/
}

// xk : Spawn One New Obstacle
void PerceptionObsManager::SpawnNewObstacle(
    const std::shared_ptr<hdmap::HDMap>& map_ptr) {
  // 找一个合适的spawn点，距离自车在100~300m之间
  common::PointENU spawn_point;
  double heading = 0.0;
  double min_speed = FLAGS_random_obstacle_min_speed_km_h / 3.6;
  double aim_speed = FLAGS_random_obstacle_max_speed_km_h / 3.6;
  int spawn_count = 0;
  std::string lane_id;

  while (true) {
    double d_x, d_y = 0.0;
    std::vector<std::shared_ptr<const TL::hdmap::LaneInfo>> candidate_lanes;
    d_x = common::math::RandomDouble(-200, 200,
                                     common::Clock::NowInNanoseconds());
    d_y = common::math::RandomDouble(-200, 200,
                                     common::Clock::NowInNanoseconds());

    double valid_range = 1;
    if (true) {
      if (-0.01 < d_x && d_x < valid_range) {
        d_x += valid_range;
      } else if (-valid_range < d_x && d_x < 0.) {
        d_x -= valid_range;
      }
      if (-0.01 < d_y && d_y < valid_range) {
        d_y += valid_range;
      } else if (-valid_range < d_y && d_y < 0.) {
        d_y -= valid_range;
      }
    }

    common::PointENU try_spawn_point;
    try_spawn_point.set_x(data_.adc_position_->x() + d_x);
    try_spawn_point.set_y(data_.adc_position_->y() + d_y);
    map_ptr->GetLanes(try_spawn_point, 10.0, &candidate_lanes);
    if (candidate_lanes.empty() || candidate_lanes.front() == nullptr) {
      if (spawn_count++ > 100) {
        return;
      } else {
        continue;
      }
    }
    double distance = 0.0;
    lane_id = candidate_lanes.front()->lane().id().id();
    spawn_point = candidate_lanes.front()->GetNearestPoint(
        {try_spawn_point.x(), try_spawn_point.y()}, &distance);
    double s = 0.0, l = 0.0;
    candidate_lanes.front()->GetProjection({spawn_point.x(), spawn_point.y()},
                                           &s, &l);
    heading = candidate_lanes.front()->Heading(s);
    aim_speed =
        std::max(aim_speed, candidate_lanes.front()->lane().speed_limit());
    min_speed = std::min(aim_speed * 0.9, min_speed);
    break;
  }

  std::shared_ptr<LaneFollowObs> obs_ptr = std::make_shared<LaneFollowObs>(
      obs_id_, spawn_point.x(), spawn_point.y(), heading,
      common::math::RandomDouble(min_speed, aim_speed,
                                 common::Clock::NowInNanoseconds()),
      lane_id);
  obs_id_++;
  dummy_obstacles_.emplace_back(std::move(obs_ptr));
}

// lp: Perception obstacle module
void PerceptionObsManager::RegisterObsList() {
  dummy_obstacles_.clear();
  std::shared_ptr<SideObs> side1 = std::make_shared<SideObs>();
  std::shared_ptr<SideObs_2> side2 = std::make_shared<SideObs_2>();
  std::shared_ptr<FrontObs> front1 = std::make_shared<FrontObs>();
  std::shared_ptr<OppositeLaneObs1> opposite_lane_obs1 =
      std::make_shared<OppositeLaneObs1>();

  int obstacle_num = 5;

  for (int i = 0; i < obstacle_num; ++i) {
    if (i < floor(obstacle_num * 0.2)) {
      std::shared_ptr<RelativePositionObstacle> randum_obs =
          std::make_shared<RelativePositionObstacle>(
              i + 23, 40 / 3.6 * 0.5 * (i + 2), 0.0, 40 / 3.6);
      dummy_obstacles_.emplace_back(std::move(randum_obs));
    }
    // else if (i < floor(obstacle_num * 0.99)) {
    //   // std::shared_ptr<RelativePositionObstacle> randum_obs =
    //   //     std::make_shared<RelativePositionObstacle>(
    //   //         i + 23, 80 / 3.6 * 0.5 * (i - 2 - obstacle_num * 0.2), 4.0,
    //   //         40 / 3.6);
    //   // dummy_obstacles_.emplace_back(std::move(randum_obs));
    // } else {
    //   std::shared_ptr<RelativePositionObstacle> randum_obs =
    //       std::make_shared<RelativePositionObstacle>(
    //           i + 23, 80 / 3.6 * 0.5 * (i + 1 + obstacle_num * 0.99), -4.0,
    //           40 / 3.6);
    //   // dummy_obstacles_.emplace_back(std::move(randum_obs));
    // }
  }

  // std::shared_ptr<RelativePositionObstacle> right_lane_back_obs =
  //     std::make_shared<RelativePositionObstacle>(
  //         111123, FLAGS_right_lane_near_obstacle_display_dis, -3.75,
  //         FLAGS_right_lane_near_obstacle_speed_km_h / 3.6);
  // dummy_obstacles_.emplace_back(std::move(right_lane_back_obs));
  // dummy_obstacles_.emplace_back(std::move(side1));
  // dummy_obstacles_.emplace_back(std::move(side2));
  dummy_obstacles_.emplace_back(std::move(front1));
  // dummy_obstacles_.emplace_back(std::move(opposite_lane_obs1));
}

void PerceptionObsManager::Update(
    double delta_time,
    std::shared_ptr<const TL::planning::ADCTrajectory> current_trajectory_,
    std::shared_ptr<common::PathPoint> adc_position,
    std::shared_ptr<TL::perception::PerceptionObstacles> perception,
    std::shared_ptr<TL::planning::LocalView> local_view,
    std::shared_ptr<hdmap::HDMap> map_ptr) {
  if (map_ptr == nullptr) {
    return;
  }

  // lp: DummyObsInputDataBase update
  data_.Update(delta_time, std::move(current_trajectory_),
               std::move(adc_position), std::move(perception),
               std::move(local_view), map_ptr);

  if (!FLAGS_only_enable_sim_adc_side_dummy_obs) {
    if (dummy_obstacles_.size() < std::max(FLAGS_obstacle_number, 5)) {
      SpawnNewObstacle(data_.map_ptr_);
    }
  }

  RemoveFromList();

  // lp: send to cyber
  for (auto& obs : dummy_obstacles_) {
    obs->UpdateState(data_);
    SortObsList();
    if (obs->IsDisplay()) {
      auto* perception_obstacle = data_.perception->add_perception_obstacle();
      perception_obstacle->CopyFrom(
          boost::any_cast<TL::perception::PerceptionObstacle>(
              obs->GetObs()));
    }
  }
}

void PerceptionObsManager::Update(DummyObsInputDataBase* Data) {
  for (auto& obs : dummy_obstacles_) {
    obs->UpdateState(data_);
  }
}

// lp: prediction obstacle module
void PredictionObsManager::RegisterObsList() {
  dummy_obstacles_.clear();
  std::shared_ptr<SideObs> side1 = std::make_shared<SideObs>();
  std::shared_ptr<SideObs_2> side2 = std::make_shared<SideObs_2>();
  std::shared_ptr<FrontObs> front1 = std::make_shared<FrontObs>();

  dummy_obstacles_.emplace_back(side1);
  dummy_obstacles_.emplace_back(side2);
  // dummy_obstacles_.emplace_back(front1);
}

void PredictionObsManager::Update(DummyObsInputDataBase* Data) {
  for (auto& obs : dummy_obstacles_) {
    obs->UpdateState(data_);
  }
}

// lp: planning obstacle module
void PlanningObsManager::RegisterObsList() {
  dummy_obstacles_.clear();
  std::shared_ptr<SideObs> side1 = std::make_shared<SideObs>();
  std::shared_ptr<SideObs_2> side2 = std::make_shared<SideObs_2>();
  std::shared_ptr<FrontObs> front1 = std::make_shared<FrontObs>();

  dummy_obstacles_.emplace_back(side1);
  dummy_obstacles_.emplace_back(side2);
  // dummy_obstacles_.emplace_back(front1);
}

void PlanningObsManager::Update(DummyObsInputDataBase* Data) {
  for (auto& obs : dummy_obstacles_) {
    obs->UpdateState(data_);
  }
}

}  // namespace simdummy
}  // namespace TL
