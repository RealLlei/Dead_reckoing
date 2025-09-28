/*
 * Copyright (c) TL auto Co., Ltd. 2023-2024. All rights reserved.
 */
#include "planning/localview/lane_line_builder/obstacle_following_lane_line/missile_mode/obstacle_decider.h"

#include <algorithm>
#include <cmath>
#include <memory>

#include "common/math/double_type.h"
#include "common/status/status.h"
#include "planning/pnc_map/pnc_map.h"

namespace TL {
namespace planning {
// using common::math::double_type::IsZero;
namespace missilelane {
Status ObstacleDecider::Init(
    const std::shared_ptr<ObstaclesState>& obstacles_state_ptr) {
  history_points_generator_ = std::make_shared<ObsHisPointsGenerator>();
  history_points_generator_->Init(obstacles_state_ptr);
  vehicle_state_ = obstacles_state_ptr->vehicle_state();

  // Low pass filter
  std::vector<double> den(3, 0.0);
  std::vector<double> num(3, 0.0);
  TL::common::LpfCoefficients(0.03, 10, &den, &num);  // ts_ , cutoff_freq;
  heading_filter_.set_coefficients(den, num);
  return Status::OK();
}

bool ObstacleDecider::UpdateFollowingObs(
    const std::shared_ptr<LocalView>& local_view,
    functionmanager::FunctionManagerOut* to_fct) {
  // following_obs_.Clear();
  perception_obstacles_ = local_view->GetPerceptionObstacles();
  // 先放进所有的障碍物供测试使用
  history_points_generator_->SetPerceptionObsPtr(
      perception_obstacles_, local_view->GetFunctionManagerIn(), to_fct);
  // bool has_follow_obs{false};
  // int32_t follow_obs_id = target_ids.empty() ? -1 : target_ids.front();
  // for (const auto& obs : perception_obstacles_->perception_obstacle()) {
  //   if (obs.id() == follow_obs_id) {
  //     following_obs_ = obs;
  //     has_follow_obs = true;
  //     // AERROR << "follow obs id: " << following_obs_.id()
  //     //        << ", theta_flu:" << following_obs_.theta_flu();
  //     break;
  //   }
  // }

  // std::vector<perception::PerceptionObstacle> vec_obs{following_obs_};
  return history_points_generator_->Process(local_view);
  // // DeciderObsBeforeVehicle();
  // if (following_obs_.has_id() && has_follow_obs) {
  //   double heading{0.0};
  //   const auto speed = following_obs_.velocity_flu().x();
  //   if (obs_theta_flu_state_) {
  //     if (speed > 3.0) {
  //       heading = std::atan(following_obs_.velocity_flu().y() /
  //                           following_obs_.velocity_flu().x());
  //     } else {
  //       obs_theta_flu_state_ = false;
  //     }
  //   } else {
  //     if (speed > 5.0) {
  //       obs_theta_flu_state_ = true;
  //     }
  //   }
  //   double theta_flu = heading_filter_.Filter(heading);
  //   following_obs_.set_theta_flu(theta_flu);
  //   // AERROR << "speed_x: " << speed
  //   //        << " , speed_y: " << following_obs_.velocity_flu().y();
  // } else {
  //   obs_theta_flu_state_ = false;
  // }

  // AERROR << "----follow obs id: " << following_obs_.id()
  //        << ", theta_flu deading:" << following_obs_.theta_flu();
  // return true;
}

void ObstacleDecider::GeneratePoints(common::Path* init_points) {
  double obs_x = history_points_generator_->obs_x();
  double obs_y = history_points_generator_->obs_y();
  double dy = std::max(std::abs(0.1 * obs_y), 0.000001);
  double radius = (obs_x * obs_x + dy * dy) / (2 * dy);
  AERROR << "RADIUS: " << radius;
  radius = std::max(radius, 200.0);
  double angle = obs_y > 0.01 ? 1.0 : -1.0;
  double max_arc_length = std::max(30.0, vehicle_state_->spd() * 8.0);
  const double central_length =
      20.0 +
      hdmap::PncMap::LookForwardDistance(
          functionmanager::MachineStateType::PERCEPTION_TYPE, 27.7, 36.1);
  const double back_length = std::max(10.0, 5.0);
  const double delta_distance = 1.0;
  const double delta_angle = delta_distance / radius;
  double s = -1.0;
  // 自车轨迹曲率较大,半径大于 3000 ;认为是直行
  if (radius > 5000.0) {
    int max_index = floor((back_length + central_length) / delta_distance);
    for (int i = 0; i < max_index; i++) {
      auto* point = init_points->add_path_point();
      auto x = i * delta_distance - back_length;
      s += delta_distance;
      point->set_x(x);
      point->set_y(0.0);
      point->set_theta(0.0);
      point->set_s(s);
      point->set_kappa(0);
    }
    return;
  }
  // 自车后方 back_length 曲率设置为 0
  int back_max_index = floor(back_length / delta_distance);
  for (int i = 0; i < back_max_index; i++) {
    auto* point = init_points->add_path_point();
    s += delta_distance;
    auto x = i * delta_distance - back_length;
    point->set_x(x);
    point->set_y(0.0);
    point->set_theta(0.0);
    point->set_s(s);
    point->set_kappa(1 / radius);
  }
  // 自车前方轨迹
  double x = 0.0;
  double y = 0.0;
  double beta = 0.0;
  int phi_num = M_PI_2 / delta_angle;
  for (int i = 0; i < phi_num && s <= max_arc_length; i++) {
    double phi = i * delta_angle;
    x = radius * std::sin(phi);
    y = copysign(radius * (1 - std::cos(phi)), angle);
    // AERROR << "****X:" << x << " ,Y :" << y;
    beta = copysign(phi, angle);
    // cneter lane point
    auto pre_point =
        init_points->path_point().at(init_points->path_point_size() - 1);
    auto* point = init_points->add_path_point();
    // cneter lane point
    point->set_x(x);
    point->set_y(y);
    point->set_theta(beta);
    point->set_kappa(1 / radius);
    s += std::hypot(x - pre_point.x(), y - pre_point.y());
    point->set_s(s);
  }
  // 裁剪
  if (s < central_length) {
    // cneter lane point
    auto pre_point =
        init_points->path_point().at(init_points->path_point_size() - 2);
    auto end_point =
        init_points->path_point().at(init_points->path_point_size() - 1);
    double dx = end_point.x() - pre_point.x();
    double dy = end_point.y() - pre_point.y();
    double end_phi = end_point.theta();
    while (s < central_length) {
      // cneter lane point
      auto last_point =
          init_points->path_point().at(init_points->path_point_size() - 1);
      auto* point = init_points->add_path_point();
      // cneter lane point
      point->set_x(last_point.x() + dx);
      point->set_y(last_point.y() + dy);
      point->set_theta(end_phi);
      s += std::hypot(dx, dy);
      point->set_s(s);
      point->set_kappa(0);
    }
  }
}

bool ObstacleDecider::DeciderObsBeforeVehicle() {
  const double vehicle_speed = vehicle_state_->spd();
  const double yaw_rate = vehicle_state_->yaw_rate();
  constexpr double kMinVehicleSpeed = 30.0;
  constexpr double kMaxFrontDistance = kMinVehicleSpeed * 4;
  bool has_obs_flag{false};

  TL::perception::LaneMarker lanemarker;
  double half_lane_width{2.15};
  auto curvature =
      common::math::Clamp((yaw_rate / vehicle_speed) / 2, -0.01, 0.01);
  lanemarker.set_c0_position(0.0);
  lanemarker.set_c1_heading_angle(0.0);
  lanemarker.set_c2_curvature(curvature);
  lanemarker.set_c3_curvature_derivative(0.0);

  if (perception_obstacles_ != nullptr) {
    double min_obs_x = 100.0;
    for (const auto& obs : perception_obstacles_->perception_obstacle()) {
      const auto& obs_y = obs.position_flu().y();
      const auto& obs_x = obs.position_flu().x();
      ADEBUG << "obs_: " << obs.id() << " , obs_x: " << obs_x
             << " , y: " << obs_y;
      if (obs_x < 0.0 || obs_x > kMaxFrontDistance) {
        continue;
      }
      auto obs_width_dis = CalculateObsY(lanemarker, obs_x, obs_y);
      if (obs_width_dis < half_lane_width) {
        has_obs_flag = true;
      }
      if (obs_x < min_obs_x && std::fabs(obs_width_dis) < half_lane_width) {
        ADEBUG << "min_obs_x: " << min_obs_x;
        min_obs_x = obs_x;
        following_obs_ = obs;
      }
    }
  }
  ADEBUG << "following_obs_: " << following_obs_.id()
         << " , obs_x: " << following_obs_.position_flu().x()
         << " , y: " << following_obs_.position_flu().y();
  return has_obs_flag;
}

double ObstacleDecider::CalculateObsY(
    const TL::perception::LaneMarker& lane_marker, double obs_x,
    double obs_y) {
  auto c0 = lane_marker.c0_position();
  auto c1 = lane_marker.c1_heading_angle();
  auto c2 = lane_marker.c2_curvature();
  auto c3 = lane_marker.c3_curvature_derivative();
  auto obs_x2 = obs_x * obs_x;
  auto obs_x3 = obs_x2 * obs_x;
  double obs_c0 = obs_y - c1 * obs_x - c2 * obs_x2 - c3 * obs_x3;
  double angle = c1 + c2 * obs_x + c3 * obs_x2;
  return std::fabs(obs_c0 - c0) * cos(angle);
}

}  // namespace missilelane
}  // namespace planning
}  // namespace TL
