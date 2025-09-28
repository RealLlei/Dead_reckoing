/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2024. All rights reserved.
 * Description:  onlane_forward_simulation.h
 */
#include "common/math/math_utils.h"
#include "planning/common/game/controllers/ctx_idm_velocity_controller.h"
#include "planning/common/game/controllers/intelligent_velocity_controller.h"
#include "planning/common/game/controllers/pure_pursuit_controller.h"
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

class Simulation {
 public:
  /**
   * @brief PropagateEgoAgent.
   * @param agent
   * @param dest_state
   * @param dt
   * @param sim_param
   * @param desired_state
   * @return true: success.
   */
  static bool PropagateEgoAgent(const ForwardSimAgent& agent,
                                const State& dest_state, double dt,
                                SimParam sim_param, State* desired_state);
  /**
   * @brief PropagateOnceAdvancedLM.
   * @param agent
   * @param leading_agent
   * @param gap_front_agent
   * @param gap_rear_agent
   * @param dt
   * @param sim_param
   * @param desired_state
   * @return true: success.
   */
  static bool PropagateOnceAdvancedLM(
      const ForwardSimAgent& agent, const ForwardSimAgent& leading_agent,
      const State& dest_state, const ForwardSimAgent& gap_front_agent,
      const ForwardSimAgent& gap_rear_agent, const double& dt,
      const SimParam& sim_param, State* desired_state);
  /**
   * @brief PropagateOnceAdvancedLC.
   * @param agent
   * @param leading_agent
   * @param gap_front_agent
   * @param gap_rear_agent
   * @param dest_state
   * @param dt
   * @param sim_param
   * @param desired_state
   * @return true: success.
   */
  static bool PropagateOnceAdvancedLC(const ForwardSimAgent& agent,
                                      const ForwardSimAgent& leading_agent,
                                      const ForwardSimAgent& gap_front_agent,
                                      const ForwardSimAgent& gap_rear_agent,
                                      const State& dest_state, const double& dt,
                                      const SimParam& sim_param,
                                      State* desired_state);
  /**
   * @brief GetTargetStateOnTargetLane.
   * @param ego_vehicle
   * @param gap_front_vehicle
   * @param gap_rear_vehicle
   * @param param
   * @param target_s
   * @param target_v
   * @return true: success.
   */
  static bool GetTargetStateOnTargetLane(const Vehicle& ego_vehicle,
                                         const Vehicle& gap_front_vehicle,
                                         const Vehicle& gap_rear_vehicle,
                                         const SimParam& param,
                                         double* target_s, double* target_v);
  /**
   * @brief PropagateOnceAdvancedLK.
   * @param agent
   * @param leading_agent
   * @param dest_state
   * @param dt
   * @param sim_param
   * @param desired_state
   * @return true: success.
   */
  static bool PropagateOnceAdvancedLK(const ForwardSimAgent& agent,
                                      const ForwardSimAgent& leading_agent,
                                      const State& dest_state, double dt,
                                      SimParam sim_param, State* desired_state);

  /**
   * @brief CalculateVelocityUsingCtxIdm.
   * @param current_pos
   * @param current_vel
   * @param target_pos
   * @param target_vel
   * @param dt
   * @param sim_param
   * @param ctx_param
   * @param velocity
   * @return true: success.
   */
  static bool CalculateVelocityUsingCtxIdm(
      double current_pos, double current_vel, double target_pos,
      double target_vel, double dt, const SimParam& sim_param,
      const ContextIntelligentDriverModel::CtxParam& ctx_param,
      double* velocity);

  /**
   * @brief CalculateVelocityUsingCtxIdm.
   * @param current_pos
   * @param current_vel
   * @param leading_pos
   * @param leading_vel
   * @param target_pos
   * @param target_vel
   * @param dt
   * @param param
   * @param ctx_param
   * @param velocity
   * @return true: success.
   */
  static bool CalculateVelocityUsingCtxIdm(
      const double& current_pos, const double& current_vel,
      const double& leading_pos, const double& leading_vel,
      const double& target_pos, const double& target_vel, const double& dt,
      const SimParam& param,
      const ContextIntelligentDriverModel::CtxParam& ctx_param,
      double* velocity);

  /**
   * @brief CalculateDesiredState.
   * @param current_state
   * @param steer
   * @param velocity
   * @param wheelbase_len
   * @param dt
   * @param sim_param
   * @param state
   * @return true: success.
   */
  static bool CalculateDesiredState(const State& current_state, double steer,
                                    double velocity, double wheelbase_len,
                                    double dt, const SimParam& sim_param,
                                    State* state);

  /**
   * @brief CalculateSteer.
   * @param dest_state
   * @param vehicle
   * @param steer
   * @return true: success.
   */
  static bool CalculateSteer(const State& dest_state, const Vehicle& vehicle,
                             double* steer);

  /**
   * @brief CalculateVelocityUsingIdm.
   * @param current_vel
   * @param dt
   * @param param
   * @param velocity
   */
  static void CalculateVelocityUsingIdm(const double& current_vel,
                                        const double& dt,
                                        const IDM::Param& param,
                                        double* velocity);
  /**
   * @brief CalculateVelocityUsingIdm.
   * @param current_pos
   * @param current_vel
   * @param leading_pos
   * @param leading_vel
   * @param dt
   * @param param
   * @param velocity
   */
  static void CalculateVelocityUsingIdm(
      const double& current_pos, const double& current_vel,
      const double& leading_pos, const double& leading_vel, const double& dt,
      const IDM::Param& param, double* velocity);
};
}  // namespace TL::planning::game_simulation
