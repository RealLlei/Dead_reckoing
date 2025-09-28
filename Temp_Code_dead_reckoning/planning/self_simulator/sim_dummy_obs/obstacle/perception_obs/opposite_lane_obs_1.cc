/*
 * Copyright (c) 2022 TL
 *
 * Author: ZhuYakun
 */

#include "planning/self_simulator/sim_dummy_obs/obstacle/perception_obs/opposite_lane_obs_1.h"
#include <utility>

namespace TL {
namespace simdummy {
bool OppositeLaneObs1::StartDisplay(const DummyObsInputDataBase& data) {
  if (!IsDisplay()) {
    if (data.current_trajectory_.get() != nullptr &&
        data.current_trajectory_->trajectory_point().size() > 1 &&
        data.current_trajectory_->trajectory_point().at(0).v() > 0.0) {
      for (auto traj_point : data.current_trajectory_->trajectory_point()) {
        if (traj_point.path_point().s() > data.right_lane_far_obstacle_s) {
          double t_x = traj_point.path_point().x();
          double t_y = traj_point.path_point().y();
          double t_heading = traj_point.path_point().theta();
          // double t_obs_v =
          //     data.current_trajectory_->trajectory_point().at(0).v();
          InitializeObs(201, 10, 3, 8, t_x + 3.75 * cos(t_heading - 90 / 57.3),
                        t_y + 3.75 * sin(t_heading - 90 / 57.3), t_heading,
                        data.right_lane_far_obstacle_speed,
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

bool OppositeLaneObs1::StopDisplay(const DummyObsInputDataBase& data) {
  SetIsNeededRemove(false);
  return false;
}

void OppositeLaneObs1::UpdateState(const DummyObsInputDataBase& data) {
  if (StartDisplay(data) && !StopDisplay(data)) {
    const auto func_is_normal = [&]() {
      std::pair<double, double> obs_pos_rel;
      auto traj_points = data.current_trajectory_->trajectory_point();
      double t_x = 0;
      double t_y = 0;
      double t_heading = 0;
      for (auto point : traj_points) {
        if (point.path_point().s() > 0) {
          t_x = point.path_point().x();
          t_y = point.path_point().y();
          t_heading = point.path_point().theta();
          break;
        }
      }
      if (CoordinateSystemEarth2Ego(data.adc_position_->x(),
                                    data.adc_position_->y(), t_heading,
                                    percep_obs_.position().x(),
                                    percep_obs_.position().y(), &obs_pos_rel)) {
        // if (obs_pos_rel.first < -50 || fabs(obs_pos_rel.second) > 400) {
        //   return false;
        // }
      } else {
        AERROR << "obs_position cache is not exit.";
      }
      return true;
    };
    if (IsNormal(func_is_normal)) {
      Update(data.delta_time, GetTheta());
    } else {
      for (auto traj_point : data.current_trajectory_->trajectory_point()) {
        if (traj_point.path_point().s() > data.right_lane_far_obstacle_s) {
          double t_x = traj_point.path_point().x();
          double t_y = traj_point.path_point().y();
          double t_heading = traj_point.path_point().theta();
          Update(t_x + 3.75 * cos(t_heading - 90 / 57.3),
                 t_y + 3.75 * sin(t_heading - 90 / 57.3), t_heading,
                 data.right_lane_far_obstacle_speed);
        }
      }
    }
  }
}
}  // namespace simdummy
}  // namespace TL
