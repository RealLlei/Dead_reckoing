/*
 * Copyright (c) 2022 TL
 *
 * Author: Ling Peng
 */

#include "planning/self_simulator/sim_dummy_obs/obstacle/perception_obs/relative_position_obs.h"

#include <limits>
#include <utility>

namespace TL {
namespace simdummy {
bool RelativePositionObstacle::StartDisplay(const DummyObsInputDataBase& data) {
  if (!IsDisplay()) {
    double t_heading = 0.0;
    if (data.current_trajectory_.get() != nullptr &&
        data.current_trajectory_->trajectory_point().size() > 1 &&
        data.current_trajectory_->trajectory_point().at(0).v() > 10) {
      for (auto traj_point : data.current_trajectory_->trajectory_point()) {
        if (traj_point.path_point().s() > 0) {
          t_heading = traj_point.path_point().theta();
          break;
        }
      }
      std::pair<double, double> t_pair;
      CoordinateSystemEgo2Earth(data.adc_position_->x(),
                                data.adc_position_->y(), t_heading, init_s_,
                                init_l_, &t_pair);
      // AERROR << FIXED << SETPRECISION(3) << "id:" << init_id_
      //        << "   s:" << init_s_ << "   l:" << init_l_ << "   x:" <<
      //        t_pair.first
      //        << "   y:" << t_pair.second;
      InitializeObs(init_id_, 5, 3, 3, t_pair.first, t_pair.second, t_heading,
                    init_v_, TL::perception::PerceptionObstacle::VEHICLE);
    }
  } else {
    return true;
  }
  return IsDisplay();
}

bool RelativePositionObstacle::StopDisplay(const DummyObsInputDataBase& data) {
  SetIsNeededRemove(false);
  return false;
}

void RelativePositionObstacle::UpdateState(const DummyObsInputDataBase& data) {
  if (StartDisplay(data) && !StopDisplay(data)) {
    const auto func_is_normal = [&]() {
      double t_heading = 0.0;
      double min_distance = std::numeric_limits<double>::infinity();
      for (auto traj_point : data.current_trajectory_->trajectory_point()) {
        double dis = common::util::DistanceXY(traj_point.path_point(),
                                              percep_obs_.position());
        if (dis < min_distance) {
          t_heading = traj_point.path_point().theta();
          min_distance = dis;
        }
      }
      init_heading_ = t_heading;

      std::pair<double, double> t_pair;
      CoordinateSystemEarth2Ego(
          data.adc_position_->x(), data.adc_position_->y(), t_heading,
          percep_obs_.position().x(), percep_obs_.position().y(), &t_pair);
      if (t_pair.second > 6.0 || t_pair.second < -6.0 || t_pair.first > 150) {
        init_heading_ = t_heading;
        return false;
      } else {
        return true;
      }
    };
    if (IsNormal(func_is_normal)) {
      Update(data.delta_time, init_heading_);
    } else {
      std::pair<double, double> t_pair;
      CoordinateSystemEgo2Earth(data.adc_position_->x(),
                                data.adc_position_->y(), init_heading_, init_s_,
                                init_l_, &t_pair);
      Update(t_pair.first, t_pair.second, init_heading_, init_v_);
    }
  }
}  // namespace simdummy
}  // namespace simdummy
}  // namespace TL
