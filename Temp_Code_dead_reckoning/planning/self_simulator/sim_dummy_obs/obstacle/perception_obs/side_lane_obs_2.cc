/*
 * Copyright (c) 2022 TL
 *
 * Author: Ling Peng
 */

#include "planning/self_simulator/sim_dummy_obs/obstacle/perception_obs/side_lane_obs_2.h"

namespace TL {
namespace simdummy {
bool SideObs_2::StartDisplay(const DummyObsInputDataBase& data) {
  if (!IsDisplay()) {
    if (data.current_trajectory_.get() != nullptr &&
        data.current_trajectory_->trajectory_point().size() > 1) {
      for (auto traj_point : data.current_trajectory_->trajectory_point()) {
        if (traj_point.path_point().s() > data.front_obstacle_s) {
          double t_x = traj_point.path_point().x();
          double t_y = traj_point.path_point().y();
          double t_heading = traj_point.path_point().theta();
          InitializeObs(102, 10, 4, 8, t_x + 3.75 * cos(t_heading + 90 / 57.3),
                        t_y + 3.75 * sin(t_heading + 90 / 57.3), t_heading,
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

bool SideObs_2::StopDisplay(const DummyObsInputDataBase& data) {
  SetIsNeededRemove(false);
  return false;
}

void SideObs_2::UpdateState(const DummyObsInputDataBase& data) {
  if (StartDisplay(data) && !StopDisplay(data)) {
    const auto func_is_normal = [&]() {
      return (percep_obs_.position().x() > 1000 &&
              percep_obs_.position().y() > 1000);
    };
    if (IsNormal(func_is_normal)) {
      Update(data.delta_time, GetTheta());
    } else {
      for (auto traj_point : data.current_trajectory_->trajectory_point()) {
        if (traj_point.path_point().s() > data.front_obstacle_s) {
          AERROR << "reset side obs.";
          double t_x = traj_point.path_point().x();
          double t_y = traj_point.path_point().y();
          double t_heading = traj_point.path_point().theta();
          Update(t_x, t_y, t_heading, data.front_obstacle_speed);
        }
      }
    }
  }
}
}  // namespace simdummy
}  // namespace TL
