/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2024. All rights reserved.
 * Description:  obstacle_game_decider.h
 */
#include <cstdint>
#include <limits>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "planning/common/game/forward_simulator/onlane_forward_simulation.h"
#include "planning/common/game/game_common/semantics/semantics.h"
#include "planning/common/reference_line_info.h"
#include "planning/tasks/deciders/decider.h"

namespace TL::planning {
using CartesianState = TL::planning::game_common::State;
using game_common::ForwardSimAgent;
using game_common::LongitudinalBehavior;
using game_common::Vehicle;
using ForwardSimAgentMap = std::unordered_map<int, ForwardSimAgent>;
using game_common::IndexedMap;

class ObstacleGameDecider : public Decider {
 public:
  ObstacleGameDecider(const TaskConfig& config,
                      const std::shared_ptr<DependencyInjector>& injector);
  common::Status Process(Frame* frame,
                         ReferenceLineInfo* reference_line_info) override;

  /**
   * @brief Run.
   * @param cutin_obstacle
   * @param mergein_obstacle
   * @param cross_obstacle
   */
  void Run(ReferenceLineInfo* reference_line_info,
           const std::unordered_set<int>& cutin_obstacle,
           const std::unordered_set<int>& mergein_obstacle,
           const std::unordered_set<int>& cross_obstacle);

  /**
   * @brief GetCutinObstacle.
   * @param frame
   * @param reference_line_info
   * @param cutin_obstacle
   */
  void GetCutinObstacle(const Frame& frame,
                        const ReferenceLineInfo& reference_line_info,
                        std::unordered_set<int>* cutin_obstacle) const;
  // get adc front mergein_obs and back mergein_obs and their leading_obs
  void GetMergeinObstacle(const Frame& frame,
                          const ReferenceLineInfo& reference_line_info,
                          std::unordered_set<int>* mergein_obstacle);
  static void ObsFilterConflict(std::unordered_set<int>* cutin_obstacle,
                                std::unordered_set<int>* mergein_obstacle);

  /**
   * @brief CalculateCutinProbByIntentionModel.
   * @param obstacle
   */
  double CalculateCutinProbByIntentionModel(const Obstacle& obstacle) const;

  /**
   * @brief FilterObstacle.
   * @param cutin_obstacle
   * @param mergein_obstacle
   * @param cross_obstacle
   */
  void FilterObstacle(const std::unordered_set<int>& cutin_obstacle,
                      const std::unordered_set<int>& mergein_obstacle,
                      const std::unordered_set<int>& cross_obstacle);

  /**
   * @brief FilterEgoLane.
   */
  void FilterEgoLane();

  /**
   * @brief FilterNeighborLane.
   * @param cutin_obstacles
   * @param neighbor_lane_vehicles
   */
  static void FilterNeighborLane(
      const std::unordered_set<int>& cutin_obstacles,
      game_common::IndexedMap* neighbor_lane_vehicles);

  /**
   * @brief SafetyCost.
   * @param cutin_obstacle
   * @param mergein_obstacle
   * @param ego_trajectory
   * @param surround_trajectories
   * @return safety cost.
   */
  double SafetyCost(const std::unordered_set<int>& cutin_obstacle,
                    const std::unordered_set<int>& mergein_obstacle,
                    const std::vector<Vehicle>& ego_trajectory,
                    const std::unordered_map<int, std::vector<Vehicle>>&
                        surround_trajectories);

  /**
   * @brief EfficiencyCost.
   * @param ego_trajectory
   * @return efficiency cost.
   */
  double EfficiencyCost(const std::vector<Vehicle>& ego_trajectory);

  /**
   * @brief AccComfortCost.
   * @param ego_trajectory
   * @return acc comfort cost.
   */
  double AccComfortCost(const std::vector<Vehicle>& ego_trajectory);

  /**
   * @brief CutinIntentionCost.
   * @param surrounding_agents
   * @param surround_trajectories
   * @return cutin intention cost.
   */
  double CutinIntentionCost(const ForwardSimAgentMap& surrounding_agents,
                            const std::unordered_map<int, std::vector<Vehicle>>&
                                surround_trajectories);

  double MergeinIntentionCost(
      const ForwardSimAgentMap& surrounding_agents,
      const std::unordered_map<int, std::vector<Vehicle>>&
          surround_trajectories);

  double ConsistencyCost(const ForwardSimAgent& ego_agent);

  /**
   * @brief ProcessEgoAgent.
   * @param ego_agent
   * @param surrounding_agents
   * @param ego_trajectory
   * @param ego_lane_vehicles
   * @return true: success.
   */
  bool ProcessEgoAgent(ForwardSimAgent* ego_agent,
                       ForwardSimAgentMap* surrounding_agents,
                       std::vector<Vehicle>* ego_trajectory,
                       IndexedMap* ego_lane_vehicles);

  /**
   * @brief UpdateSurroundingAgentInfo.
   * @param others_state
   * @param ego_lane_vehicles
   * @param right_lane_vehicles
   * @param left_lane_vehicles
   * @param ego_agent
   * @param surrounding_agents
   * @param surround_trajectories
   * @return true: success.
   */
  bool UpdateSurroundingAgentInfo(
      const std::unordered_map<int, CartesianState>& others_state,
      IndexedMap* ego_lane_vehicles, IndexedMap* right_lane_vehicles,
      IndexedMap* left_lane_vehicles, IndexedMap* merge_lane_vehicles,
      ForwardSimAgent* ego_agent, ForwardSimAgentMap* surrounding_agents,
      std::unordered_map<int, std::vector<Vehicle>>* surround_trajectories);

  /**
   * @brief BuildingSimEnvironment.
   * @param cutin_obstacle
   * @param mergein_obstacle
   * @param cross_obstacle
   */
  void BuildingSimEnvironment(const std::unordered_set<int>& cutin_obstacle,
                              const std::unordered_set<int>& mergein_obstacle,
                              const std::unordered_set<int>& cross_obstacle);

  /**
   * @brief PrepareEgoAgent.
   */
  void PrepareEgoAgent();

  /**
   * @brief BuildingSimEnvironment.
   * @param cutin_obstacle
   * @param mergein_obstacle
   */
  void PrepareSurroundingAgent(const Obstacle& obstacle,
                               const std::unordered_set<int>& cutin_obstacle,
                               const std::unordered_set<int>& mergein_obstacle,
                               const std::unordered_set<int>& cross_obstacle);

  /**
   * @brief SimulateSequence.
   * @param ego_agent
   * @param surrounding_agents
   * @param ego_trajectory
   * @param surround_trajectories
   * @return true: success.
   */
  bool SimulateSequence(
      ForwardSimAgent* ego_agent, ForwardSimAgentMap* surrounding_agents,
      std::vector<Vehicle>* ego_trajectory,
      std::unordered_map<int, std::vector<Vehicle>>* surround_trajectories);

  static void SimulationDebug(
      const std::vector<Vehicle>& ego_optimal_trajectory,
      const std::unordered_map<int, std::vector<Vehicle>>& trajectories,
      ReferenceLineInfo* reference_line_info);

  /**
   * @brief EgoAgentForwardSim.
   * @param ego_agent
   * @param agent_map
   * @param ego_lane_vehicle
   * @param sim_time_step
   * @param state_out
   * @return true: success.
   */
  bool EgoAgentForwardSim(const ForwardSimAgent& ego_agent,
                          const ForwardSimAgentMap& agent_map,
                          const IndexedMap& ego_lane_vehicle,
                          double sim_time_step, CartesianState* state_out);

  /**
   * @brief CheckCollisionUsingState.
   * @param agent_a
   * @param agent_b
   * @return true: collision.
   */
  static bool CheckCollisionUsingState(const ForwardSimAgent& agent_a,
                                       const ForwardSimAgent& agent_b);

  /**
   * @brief SurroundingAgentForwardSim.
   * @param ego_agent
   * @param agents_map
   * @param sim_time_step
   * @param ego_lane_vehicles
   * @param right_lane_vehicles
   * @param left_lane_vehicles
   * @param surrounding_agent
   * @param state_out
   * @return true: success.
   */
  bool SurroundingAgentForwardSim(const ForwardSimAgent& ego_agent,
                                  const ForwardSimAgentMap& agents_map,
                                  double sim_time_step, int step_num,
                                  const IndexedMap& ego_lane_vehicles,
                                  const IndexedMap& right_lane_vehicles,
                                  const IndexedMap& left_lane_vehicles,
                                  const IndexedMap& merge_lane_vehicles,
                                  const ForwardSimAgent& surrounding_agent,
                                  CartesianState* state_out);

  /**
   * @brief GetTargetState.
   * @param agent
   * @param target_l
   * @param state_out
   * @return true: success.
   */
  bool GetTargetState(const ForwardSimAgent& agent, double target_l,
                      CartesianState* state_out);

  /**
   * @brief GetLeadingAgent.
   * @param cur_agent
   * @param agents_map
   * @param ego_lane_vehicles
   * @param left_lane_vehicles
   * @param right_lane_vehicles
   * @param leading_agent
   * @return true: success.
   */
  bool GetLeadingAgent(const ForwardSimAgent& cur_agent,
                       const ForwardSimAgentMap& agents_map,
                       const IndexedMap& ego_lane_vehicles,
                       const IndexedMap& left_lane_vehicles,
                       const IndexedMap& right_lane_vehicles,
                       ForwardSimAgent* leading_agent) const;

  /**
   * @brief GetLeadingAgent.
   * @param ego_agent
   * @param agents_map
   * @param ego_lane_vehicles
   * @param surrounding_agent
   * @return true: success.
   */
  static bool SurroundingAgentIntentionUpdate(
      const ForwardSimAgent& ego_agent, const ForwardSimAgentMap& agents_map,
      const IndexedMap& ego_lane_vehicle, ForwardSimAgent* surrounding_agent);

  /**
   * @brief GetEgoLeadingAgent.
   * @param ego_agent
   * @param agents_map
   * @param ego_lane_vehicle
   * @param leading_agent
   * @return true: success.
   */
  static bool GetEgoLeadingAgent(const ForwardSimAgent& ego_agent,
                                 const ForwardSimAgentMap& agents_map,
                                 const IndexedMap& ego_lane_vehicle,
                                 ForwardSimAgent* leading_agent);
  /**
   * @brief SimEnvironmentContextUpdate.
   * @param ego_lane_vehicle
   * @param right_lane_vehicle
   * @param left_lane_vehicle
   * @param agent
   * @return true: success.
   */
  bool SimEnvironmentContextUpdate(IndexedMap* ego_lane_vehicle,
                                   IndexedMap* right_lane_vehicle,
                                   IndexedMap* left_lane_vehicle,
                                   IndexedMap* merge_lane_vehicle,
                                   ForwardSimAgent* agent,
                                   ForwardSimAgent* ego_agent) const;
  /**
   * @brief CalculateVehicleDistanceUseBox.
   * @param vehicle_a
   * @param vehicle_b
   * @return true: success.
   */
  static double CalculateVehicleDistanceUseBox(const Vehicle& vehicle_a,
                                               const Vehicle& vehicle_b);

  /**
   * @brief CalculateVehicleDistanceUseBox.
   * @param surrounding_agents
   * @param cutin_obstacle
   * @param mergein_obstacle
   * @param ego_trajectory
   * @param surround_trajectories
   * @return cost.
   */
  double CostFunction(const ForwardSimAgentMap& surrounding_agents,
                      const std::unordered_set<int>& cutin_obstacle,
                      const std::unordered_set<int>& mergein_obstacle,
                      const std::vector<Vehicle>& ego_trajectory,
                      const std::unordered_map<int, std::vector<Vehicle>>&
                          surround_trajectories,
                      const ForwardSimAgent& ego_agent);

  /**
   * @brief PassOptimalSimResultToMotionPlanner.
   * @param cutin_obstacle
   * @param ego_trajectory
   * @param trajectories
   */
  void PassOptimalSimResultToMotionPlanner(
      const std::unordered_set<int>& cutin_obstacle,
      const std::unordered_set<int>& mergein_obstacle,
      const std::vector<Vehicle>& ego_trajectory,
      const ForwardSimAgent& opt_ego_agent,
      const std::unordered_map<int, std::vector<Vehicle>>& trajectories);

  void GetMergeinTargetSLPoint(int32_t sim_obs_id, double sim_time,
                               common::SLPoint* sl_point);
  /**
   * @brief CutinPostProcess.
   * @param obstacle
   */
  void CutinPostProcess(const Obstacle& obstacle);

  /**
   * @brief ProcessDangerousApproachingCutinObstacles.
   * @param cutin_obstacle
   */
  void ProcessDangerousApproachingCutinObstacles(
      const std::unordered_set<int>& cutin_obstacle);

 private:
  std::vector<ForwardSimAgent> ego_sim_agents_ = {};  // 自车的多种仿真行为
  ForwardSimAgentMap surrounding_sim_agents_ = {};  // 自车周边智能体集合

  game_common::VehicleParam ego_vehicle_param_;
  std::vector<double> sample_acc_;
  double sim_time_resolution_;
  uint32_t sim_step_size_;
  bool enable_obstacle_game_decider_ = false;

  game_common::IndexedMap ego_lane_vehicles_;
  game_common::IndexedMap left_lane_vehicles_;
  game_common::IndexedMap right_lane_vehicles_;
  game_common::IndexedMap merge_lane_vehicles_;

  double lane_left_width_ = 1.75;
  double lane_right_width_ = 1.75;
  double right_neighbor_lane_width_ = 3.5;
  double left_neighbor_lane_width_ = 3.5;
  double last_optimal_sim_acc_ = 0.0;

  bool vehicle_in_merge_lane_ = false;
  double first_merge_point_s_ = std::numeric_limits<double>::infinity();
  std::unordered_map<int, std::pair<int, double>> close_to_counter_;
};

}  // namespace TL::planning
