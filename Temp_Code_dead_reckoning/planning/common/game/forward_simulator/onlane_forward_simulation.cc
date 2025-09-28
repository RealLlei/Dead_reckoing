/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2024. All rights reserved.
 * Description:  onlane_forward_simulation.cc
 */
#include "planning/common/game/forward_simulator/onlane_forward_simulation.h"
#include <algorithm>
#include <cmath>
#include "common/math/math_utils.h"
#include "common/math/vec2d.h"
#include "planning/common/game/game_common/idm/idm.h"
#include "planning/common/game/game_common/semantics/semantics.h"
#include "planning/common/game/vehicle_model/ctx_idm_model.h"
#include "planning/common/game/vehicle_model/ideal_steer_model.h"

namespace TL::planning::game_simulation {
using common::math::NormalizeAngle;
using game_common::ContextIntelligentDriverModel;
using game_common::ContextIntelligentVelocityControl;
using game_common::ForwardSimAgent;
using game_common::IdealSteerModel;
using game_common::IDM;
using game_common::IntelligentVelocityControl;
using game_common::kInvalidAgentId;
using game_common::PurePursuitControl;
using game_common::SimParam;
using game_common::State;
using game_common::Vehicle;

bool Simulation::PropagateOnceAdvancedLM(
    const ForwardSimAgent& agent, const ForwardSimAgent& leading_agent,
    const State& dest_state, const ForwardSimAgent& gap_front_agent,
    const ForwardSimAgent& gap_rear_agent, const double& dt,
    const SimParam& sim_param, State* desired_state) {
  if (desired_state == nullptr) {
    return false;
  }
  State current_state = agent.vehicle.state();
  double wheelbase_len = agent.vehicle.param().wheel_base();

  // step 1 计算方向盘转角
  double steer = 0.0;
  if (!CalculateSteer(dest_state, agent.vehicle, &steer)) {
    return false;
  }

  // step 2 计算速度
  double velocity = 0.0;
  double target_s = agent.vehicle.frenet_state().s_state.at(0);
  double target_v = agent.vehicle.state().velocity;
  GetTargetStateOnTargetLane(agent.vehicle, gap_front_agent.vehicle,
                             gap_rear_agent.vehicle, sim_param, &target_s,
                             &target_v);
  ContextIntelligentDriverModel::CtxParam ctx_param(0.4, 0.8);
  if (leading_agent.vehicle.id() == kInvalidAgentId) {
    // 无前车
    CalculateVelocityUsingCtxIdm(agent.vehicle.frenet_state().s_state.at(0),
                                 agent.vehicle.state().velocity, target_s,
                                 target_v, dt, sim_param, ctx_param, &velocity);
  } else {
    // 有前车
    // ~ With leading vehicle
    // * For IDM, vehicle length is subtracted to get the 'net' distance
    // * between ego vehicle and the leading vehicle.
    CalculateVelocityUsingCtxIdm(
        agent.vehicle.frenet_state().s_state.at(0),
        agent.vehicle.state().velocity,
        leading_agent.vehicle.frenet_state().s_state.at(0),
        leading_agent.vehicle.state().velocity, target_s, target_v, dt,
        sim_param, ctx_param, &velocity);
  }

  // * Step: Get desired state using vehicle kinematic model
  CalculateDesiredState(current_state, steer, velocity, wheelbase_len, dt,
                        sim_param, desired_state);

  return true;
}

bool Simulation::PropagateOnceAdvancedLC(
    const ForwardSimAgent& agent, const ForwardSimAgent& leading_agent,
    const ForwardSimAgent& gap_front_agent,
    const ForwardSimAgent& gap_rear_agent, const State& dest_state,
    const double& dt, const SimParam& sim_param, State* desired_state) {
  State current_state = agent.vehicle.state();
  double wheelbase_len = agent.vehicle.param().wheel_base();

  double steer = 0.0;
  // step 1 计算方向盘转角
  if (!CalculateSteer(dest_state, agent.vehicle, &steer)) {
    return false;
  }

  // step 2 计算速度
  double velocity = 0.0;
  double target_s = agent.vehicle.frenet_state().s_state.at(0);
  double target_v = agent.vehicle.state().velocity;
  GetTargetStateOnTargetLane(agent.vehicle, gap_front_agent.vehicle,
                             gap_rear_agent.vehicle, sim_param, &target_s,
                             &target_v);
  ContextIntelligentDriverModel::CtxParam ctx_param(0.4, 0.8);
  if (leading_agent.vehicle.id() == kInvalidAgentId) {
    // 无前车
    CalculateVelocityUsingCtxIdm(agent.vehicle.frenet_state().s_state.at(0),
                                 agent.vehicle.state().velocity, target_s,
                                 target_v, dt, sim_param, ctx_param, &velocity);
  } else {
    // 有前车
    // ~ With leading vehicle
    // * For IDM, vehicle length is subtracted to get the 'net' distance
    // * between ego vehicle and the leading vehicle.
    CalculateVelocityUsingCtxIdm(
        agent.vehicle.frenet_state().s_state.at(0),
        agent.vehicle.state().velocity,
        leading_agent.vehicle.frenet_state().s_state.at(0),
        leading_agent.vehicle.state().velocity, target_s, target_v, dt,
        sim_param, ctx_param, &velocity);
  }

  // * Step: Get desired state using vehicle kinematic model
  CalculateDesiredState(current_state, steer, velocity, wheelbase_len, dt,
                        sim_param, desired_state);

  return true;
}

bool Simulation::GetTargetStateOnTargetLane(const Vehicle& ego_vehicle,
                                            const Vehicle& gap_front_vehicle,
                                            const Vehicle& gap_rear_vehicle,
                                            const SimParam& param,
                                            double* target_s,
                                            double* target_v) {
  if (target_s == nullptr || target_v == nullptr) {
    return false;
  }

  double ahead_time = param.idm_param.desired_ahead_time;
  double min_spacing = param.idm_param.minimum_spacing;

  bool has_front = false;
  double s_ref_front = -1;  // tail of front vehicle
  double s_thres_front = -1;
  if (gap_front_vehicle.id() != kInvalidAgentId) {
    has_front = true;
    s_ref_front = gap_front_vehicle.frenet_state().s_state.at(0) -
                  (gap_front_vehicle.param().length() / 2.0);
    s_thres_front =
        s_ref_front - min_spacing - ahead_time * ego_vehicle.state().velocity;
  }

  bool has_rear = false;
  double s_ref_rear = -1;  // head of rear vehicle
  double s_thres_rear = -1;
  if (gap_rear_vehicle.id() != kInvalidAgentId) {
    has_rear = true;
    s_ref_rear = gap_rear_vehicle.frenet_state().s_state.at(0) +
                 gap_rear_vehicle.param().length() / 2.0;
    s_thres_rear = s_ref_rear + min_spacing +
                   ahead_time * gap_rear_vehicle.state().velocity;
  }

  double desired_s = ego_vehicle.frenet_state().s_state.at(0);
  double desired_v = ego_vehicle.state().velocity;

  // ~ params
  double k_v = 0.1;      // coeff for dv. k_v * s_err
  double p_v_ego = 0.1;  // coeff for user preferred vel
  double dv_lb = -3.0;   // dv lower bound
  double dv_ub = 5.0;    // dv upper bound

  double ego_desired_vel =
      ego_vehicle.state().velocity +
      (param.idm_param.desired_velocity - ego_vehicle.state().velocity) *
          p_v_ego;
  if (has_front && has_rear) {
    // * has both front and rear vehicle
    if (s_ref_front < s_ref_rear) {
      return false;
    }

    double ds = fabs(s_ref_front - s_ref_rear);
    double s_star = s_ref_rear + ds / 2.0;

    s_thres_front = std::max(s_star, s_thres_front);
    s_thres_rear = std::min(s_star, s_thres_rear);

    desired_s = std::min(
        std::max(s_thres_rear, ego_vehicle.frenet_state().s_state.at(0)),
        s_thres_front);

    double s_err_front =
        s_thres_front - ego_vehicle.frenet_state().s_state.at(0);
    double v_ref_front = std::max(
        0.0, gap_front_vehicle.state().velocity +
                 std::fmin(std::fmax(s_err_front * k_v, dv_lb), dv_ub));

    double s_err_rear = s_thres_rear - ego_vehicle.frenet_state().s_state.at(0);
    double v_ref_rear =
        std::max(0.0, gap_rear_vehicle.state().velocity +
                          std::fmin(std::fmax(s_err_rear * k_v, dv_lb), dv_ub));

    desired_v = std::min(std::max(v_ref_rear, ego_desired_vel), v_ref_front);

  } else if (has_front) {
    // * only has front vehicle
    desired_s =
        std::min(ego_vehicle.frenet_state().s_state.at(0), s_thres_front);

    double s_err_front =
        s_thres_front - ego_vehicle.frenet_state().s_state.at(0);
    double v_ref_front = std::max(
        0.0, gap_front_vehicle.state().velocity +
                 std::fmin(std::fmax(s_err_front * k_v, dv_lb), dv_ub));
    desired_v = std::min(ego_desired_vel, v_ref_front);

  } else if (has_rear) {
    // * only has rear vehicle
    desired_s =
        std::max(ego_vehicle.frenet_state().s_state.at(0), s_thres_rear);

    double s_err_rear = s_thres_rear - ego_vehicle.frenet_state().s_state.at(0);
    double v_ref_rear =
        std::max(0.0, gap_rear_vehicle.state().velocity +
                          std::fmin(std::fmax(s_err_rear * k_v, dv_lb), dv_ub));
    desired_v = std::max(v_ref_rear, ego_desired_vel);
  }

  *target_s = desired_s;
  *target_v = desired_v;

  return true;
}

bool Simulation::PropagateOnceAdvancedLK(const ForwardSimAgent& agent,
                                         const ForwardSimAgent& leading_agent,
                                         const State& dest_state,
                                         const double dt, SimParam sim_param,
                                         State* desired_state) {
  // step 1 计算方向盘转角
  double steer = 0.0;
  if (!CalculateSteer(dest_state, agent.vehicle, &steer)) {
    return false;
  }
  // step 2 计算行驶速度
  double velocity = 0.0;
  if (leading_agent.vehicle.id() == kInvalidAgentId) {
    // 无前车
    CalculateVelocityUsingIdm(agent.vehicle.state().velocity, dt,
                              sim_param.idm_param, &velocity);
  } else {
    // 有前车
    sim_param.idm_param.vehicle_length =
        (agent.vehicle.param().length() +
         leading_agent.vehicle.param().length()) *
        0.5;
    CalculateVelocityUsingIdm(
        agent.vehicle.frenet_state().s_state.at(0),
        agent.vehicle.state().velocity,
        leading_agent.vehicle.frenet_state().s_state.at(0),
        leading_agent.vehicle.state().velocity, dt, sim_param.idm_param,
        &velocity);
  }
  // step 3 使用运动学模型输出期望状态
  CalculateDesiredState(agent.vehicle.state(), steer, velocity,
                        agent.vehicle.param().wheel_base(), dt, sim_param,
                        desired_state);
  return true;
}

bool Simulation::PropagateEgoAgent(const ForwardSimAgent& agent,
                                   const State& dest_state, const double dt,
                                   SimParam sim_param, State* desired_state) {
  // step 1 计算方向盘转角
  double steer = 0.0;
  if (!CalculateSteer(dest_state, agent.vehicle, &steer)) {
    return false;
  }
  // step 2 计算行驶速度
  const double velocity =
      std::fmax(0.0, agent.vehicle.state().acceleration * dt +
                         agent.vehicle.state().velocity);

  // step 3 使用运动学模型输出期望状态
  CalculateDesiredState(agent.vehicle.state(), steer, velocity,
                        agent.vehicle.param().wheel_base(), dt, sim_param,
                        desired_state);
  return true;
}

bool Simulation::CalculateVelocityUsingCtxIdm(
    const double current_pos, const double current_vel, const double target_pos,
    const double target_vel, const double dt, const SimParam& sim_param,
    const ContextIntelligentDriverModel::CtxParam& ctx_param,
    double* velocity) {
  const double virtual_leading_pos = current_pos + 100.0 + 100.0 * current_vel;

  return ContextIntelligentVelocityControl::CalculateDesiredVelocity(
      sim_param.idm_param, ctx_param, current_pos, virtual_leading_pos,
      target_pos, current_vel, current_vel, target_vel, dt, velocity);
}

bool Simulation::CalculateVelocityUsingCtxIdm(
    const double& current_pos, const double& current_vel,
    const double& leading_pos, const double& leading_vel,
    const double& target_pos, const double& target_vel, const double& dt,
    const SimParam& param,
    const ContextIntelligentDriverModel::CtxParam& ctx_param,
    double* velocity) {
  double leading_vel_fin = leading_vel;
  if (leading_vel < 0) {
    leading_vel_fin = 0;
  }
  // ~ note that we cannot use frenet state velocity for idm model, since the
  // ~ velocity in the frenet state may be larger than body velocity if the
  // ~ vehicle is in a highly curvy road (ref to the state transformer) which
  // ~ cannot be directly fed back to body velocity.
  return ContextIntelligentVelocityControl::CalculateDesiredVelocity(
      param.idm_param, ctx_param, current_pos, leading_pos, target_pos,
      current_vel, leading_vel_fin, target_vel, dt, velocity);
}

bool Simulation::CalculateDesiredState(
    const State& current_state, const double steer, const double velocity,
    const double wheelbase_len, const double dt, const SimParam& sim_param,
    State* state) {
  IdealSteerModel model(
      wheelbase_len, sim_param.idm_param.acceleration,
      sim_param.idm_param.hard_breaking_decelerate, sim_param.max_lon_acc_jerk,
      sim_param.max_lon_brake_jerk, sim_param.max_lat_acceleration_abs,
      sim_param.max_lat_jerk_abs, sim_param.max_steer_angle_abs,
      sim_param.max_steer_rate, sim_param.max_curvature_abs);
  model.set_state(current_state);
  model.set_control(IdealSteerModel::Control(steer, velocity));
  model.Step(dt);
  *state = model.state();
  state->time_stamp = current_state.time_stamp + dt;
  return true;
}

bool Simulation::CalculateSteer(const State& dest_state, const Vehicle& vehicle,
                                double* const steer) {
  if (steer == nullptr) {
    return false;
  }

  const double lookahead_distance =
      (dest_state.position - vehicle.state().position).Length();
  const double cur_to_dest_angle =
      (dest_state.position - vehicle.state().position).Angle();
  const double angle_diff =
      NormalizeAngle(cur_to_dest_angle - vehicle.state().angle);
  PurePursuitControl::CalculateDesiredSteer(
      vehicle.param().wheel_base(), angle_diff, lookahead_distance, steer);
  return true;
}

// 无前车
void Simulation::CalculateVelocityUsingIdm(const double& current_vel,
                                           const double& dt,
                                           const IDM::Param& param,
                                           double* velocity) {
  const double virtual_leading_dist = 100.0 + 100.0 * current_vel;
  IntelligentVelocityControl::CalculateDesiredVelocity(
      param, 0.0, 0.0 + virtual_leading_dist, current_vel, current_vel, dt,
      velocity);
}

// 有前车
void Simulation::CalculateVelocityUsingIdm(
    const double& current_pos, const double& current_vel,
    const double& leading_pos, const double& leading_vel, const double& dt,
    const IDM::Param& param, double* velocity) {
  const double leading_vel_fin = std::fmax(leading_vel, 0.0);

  // ~ note that we cannot use frenet state velocity for idm model, since the
  // ~ velocity in the frenet state may be larger than body velocity if the
  // ~ vehicle is in a highly curvy road (ref to the state transformer) which
  // ~ cannot be directly fed back to body velocity.
  IntelligentVelocityControl::CalculateDesiredVelocity(
      param, current_pos, leading_pos, current_vel, leading_vel_fin, dt,
      velocity);
}

}  // namespace TL::planning::game_simulation
