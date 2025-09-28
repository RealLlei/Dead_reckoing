/*
 * Copyright (c) 2022 TL
 *
 * Author: Ling Peng
 */

#include "planning/self_simulator/sim_dummy_obs/obstacle/perception_obs/perception_obstacle_base.h"

namespace TL {
namespace simdummy {

void PerceptionObstacleBase::Update(double x, double y, double theta,
                                    double v) {
  percep_obs_.set_theta(theta);
  percep_obs_.mutable_position()->set_x(x);
  percep_obs_.mutable_position()->set_y(y);
  percep_obs_.mutable_velocity()->set_x(v * cos(theta));
  percep_obs_.mutable_velocity()->set_y(v * sin(theta));

  theta_ = theta;
  x_ = x;
  y_ = y;
  if (v > -10000) {
    v_ = v;
    vx_ = v * cos(theta);
    vy_ = v * sin(theta);
  }
}

void PerceptionObstacleBase::Update(double delta_time, double theta) {
  x_ += v_ * delta_time * cos(theta_);
  y_ += v_ * delta_time * sin(theta_);
  vx_ = v_ * cos(theta_);
  vy_ = v_ * sin(theta_);
  theta_ = theta;

  percep_obs_.set_theta(theta_);
  percep_obs_.mutable_position()->set_x(x_);
  percep_obs_.mutable_position()->set_y(y_);
  percep_obs_.mutable_velocity()->set_x(vx_);
  percep_obs_.mutable_velocity()->set_y(vy_);
}

void PerceptionObstacleBase::SetState(int32_t id, double length_, double width,
                                      double height, double x, double y,
                                      double theta, double v, boost::any type) {
  x_ = x;
  y_ = y;
  v_ = v;
  theta_ = theta;
  percep_obs_.set_id(id + 20000000);
  percep_obs_.set_length(length_);
  percep_obs_.set_width(width);
  percep_obs_.set_height(height);
  percep_obs_.set_type(
      boost::any_cast<TL::perception::PerceptionObstacle_Type>(type));
  percep_obs_.set_sub_type(TL::perception::PerceptionObstacle::ST_CAR);
  percep_obs_.set_theta(theta);
  percep_obs_.mutable_position()->set_x(x);
  percep_obs_.mutable_position()->set_y(y);
  percep_obs_.mutable_position()->set_z(1);
  percep_obs_.mutable_velocity()->set_x(v * cos(theta));
  percep_obs_.mutable_velocity()->set_y(v * sin(theta));
  percep_obs_.mutable_velocity()->set_z(0.1);
  percep_obs_.mutable_acceleration()->set_x(0.0);
  percep_obs_.mutable_acceleration()->set_y(0.0);
  percep_obs_.mutable_acceleration()->set_z(0.0);
  vx_ = v * cos(theta);
  vy_ = v * sin(theta);
}

}  // namespace simdummy
}  // namespace TL
