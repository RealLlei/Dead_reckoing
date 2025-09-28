/*
 * Copyright (c) 2022 TL
 *
 * Author: Ling Peng
 */

#include "planning/self_simulator/sim_dummy_obs/obstacle/perception_obs/front_obs.h"

namespace TL {
namespace simdummy {
bool FrontObs::StartDisplay(const DummyObsInputDataBase& data) {
  if (!IsDisplay()) {
    if (data.current_trajectory_.get() != nullptr &&
        data.current_trajectory_->trajectory_point().size() > 1 &&
        data.current_trajectory_->trajectory_point().at(0).v() > 10) {
      for (auto traj_point : data.current_trajectory_->trajectory_point()) {
        if (traj_point.path_point().s() > data.left_lane_far_obstacle_s) {
          double t_x = traj_point.path_point().x();
          double t_y = traj_point.path_point().y();
          double t_heading = traj_point.path_point().theta();
          // double t_obs_v =
          //     data.current_trajectory_->trajectory_point().at(0).v();
          InitializeObs(001, 12, 2, 3, t_x, t_y, t_heading,
                        data.front_obstacle_speed,
                        TL::perception::PerceptionObstacle::VEHICLE);
          break;
        }
      }
    }
  } else {
    return true;
  }
  return IsDisplay();
}

bool FrontObs::StopDisplay(const DummyObsInputDataBase& data) {
  SetIsNeededRemove(false);
  return false;
}

void FrontObs::UpdateState(const DummyObsInputDataBase& data) {
  if (StartDisplay(data) && !StopDisplay(data)) {
    const auto func_is_normal = [&]() {
      return (fabs(percep_obs_.position().x()) > 1000 &&
              fabs(percep_obs_.position().y()) > 1000);
    };
    // AERROR << "lp_percep_obs:" << perception_obs_.DebugString();
    // AERROR << "lp_IsNormal:" << IsNormal(func_is_normal);
    if (IsNormal(func_is_normal)) {
      Update(data.delta_time, GetTheta());
    } else {
      auto traj_points = data.current_trajectory_->trajectory_point();
      size_t size_traj_points = traj_points.size();
      if (size_traj_points > 1 &&
          traj_points.at(size_traj_points - 1).path_point().s() >
              data.left_lane_far_obstacle_s) {
        for (auto traj_point : data.current_trajectory_->trajectory_point()) {
          if (traj_point.path_point().s() > data.left_lane_far_obstacle_s) {
            double t_x = traj_point.path_point().x();
            double t_y = traj_point.path_point().y();
            double t_heading = traj_point.path_point().theta();
            AERROR << "reset front obs."
                   << "   t_x:" << t_x << "   t_y:" << t_y
                   << "   t_heading:" << t_heading;
            Update(t_x, t_y, t_heading, data.left_lane_far_obstacle_speed);
          }
        }
      } else {
        double t_x = traj_points.at(size_traj_points - 1).path_point().x();
        double t_y = traj_points.at(size_traj_points - 1).path_point().y();
        double t_heading =
            traj_points.at(size_traj_points - 1).path_point().theta();
        AERROR << "reset front obs."
               << "   t_x:" << t_x << "   t_y:" << t_y
               << "   t_heading:" << t_heading;
        Update(t_x, t_y, t_heading, data.left_lane_far_obstacle_speed);
      }
    }
  }
}
}  // namespace simdummy
}  // namespace TL
