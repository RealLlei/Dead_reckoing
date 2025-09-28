/******************************************************************************
 * Copyright (c) TL Technologies Co., Ltd. 2019-2022. All rights reserved.
 * Author: LingPeng
 * Created Time: 2022/4/21
 *****************************************************************************/

#pragma once

#include <deque>
#include <iomanip>
#include <limits>
#include <list>
#include <map>
#include <memory>
#include <numeric>
#include <queue>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "boost/lockfree/lockfree_forward.hpp"
#include "boost/lockfree/queue.hpp"
#include "boost/lockfree/spsc_queue.hpp"
#include "boost/lockfree/stack.hpp"
#include "common/file/log.h"
#include "common/math/box2d.h"
#include "common/math/double_type.h"
#include "common/math/vec2d.h"
#include "common/time/clock.h"
#include "common/util/util.h"
#include "planning/common/planning_gflags.h"
#include "planning/localview/lane_line_builder/obstacle_following_lane_line/common_util/util.h"
#include "planning/localview/lane_line_builder/obstacle_following_lane_line/coordinate_system_convert.h"
#include "planning/localview/lane_line_builder/obstacle_following_lane_line/ego_vehicle_state.h"
#include "planning/localview/lane_line_builder/obstacle_following_lane_line/fit_curve/fit_manager.h"
#include "planning/reference_line/reference_line.h"

#include "proto/common/pnc_point.pb.h"
#include "proto/perception/perception_obstacle.pb.h"

namespace TL {
namespace planning {
namespace nolane {

class ObstaclePerceptionManager;

class ObstaclePerceptionNoLane {
 public:
  ObstaclePerceptionNoLane(
      int id_unique, int id, StateType state,
      std::shared_ptr<std::queue<int>> id_pool,
      const TL::perception::PerceptionObstacle& perception_obstacle,
      FitType fit_type = FitType::PolyNomial, signed int order = 1);

  // lp: attention obstacle can't be copy construct or been assigned.
  ObstaclePerceptionNoLane(const ObstaclePerceptionNoLane& rhs) = delete;
  ObstaclePerceptionNoLane& operator=(const ObstaclePerceptionNoLane& rhs) =
      delete;

  ~ObstaclePerceptionNoLane();

  /**
   * @brief
   * @param isUpdateConsistent
   */
  void SetIsUpdateConsistent(bool isUpdateConsistent);

  /**
   * @brief
   * @return
   */
  bool IsUpdateConsistent() const;

  /**
   * @brief
   * @param obstacle
   * @return true, add state_history successfully; false state can't be one obs.
   */
  bool UpdateState(const ObstaclePerceptionNoLane& obstacle);

  /**
   * @brief
   */
  void TrajectoryExtendFuture();

  /**
   * @brief check trajectory need projection
   */
  void IsTrajectoryProject();

  /**
   * @brief
   */
  void TrajectoryExtendPast();

  /**
   * @brief some point need to be trimmed when past the ego.
   */
  void TrajectoryTrim(const EgoVehicleState& ego_state);

  /**
   * @brief filter each point from whole trajectory aspect.
   */
  void TrajectoryFit();

  /**
   * @brief check trajectory is straight.
   */
  void CheckTrajectoryIsStraight();

  struct PointDirection {
    double x;
    double y;
    double slope;
  };

  /**
   * @brief CollisionCheckWithEgos
   *
   * @param lane_line_points
   * @param obstacle
   * @param veh_param
   * @return true
   * @return false
   */
  bool CollisionCheckWithEgo(
      const std::vector<PointDirection>& lane_line_points,
      const perception::PerceptionObstacle& obstacle,
      const TL::common::VehicleParam& veh_param);

  /**
   * @brief
   * @param obstacles_raw
   * @return true if any obstacle but this has collision with ego in lane line.
   */
  inline bool CollisionCheckWithAllObstacles(
      const std::vector<PointDirection>& lane_line_points,
      const perception::PerceptionObstacles& obstacles_raw,
      const TL::common::VehicleParam& veh_param);

  /**
   * @brief find the most suitable point in trajectory.
   * @param ego_state
   * @return the optimal point in obstacle trajectory.
   * (true/false, index, cost, is_closed)
   */
  std::tuple<bool, int, int, bool> IsEgoInTrajectory(
      const EgoVehicleState& ego_state,
      const TL::perception::PerceptionObstacles& obstacles_raw,
      bool lane_change_followed);

  enum class MovingBehavior {
    Cruise = 1,
    LeftCutOut = 2,
    LeftCutIn = 3,
    RightCutOut = 4,
    RightCutIn = 5,
    Unknown = 0
  };
  enum class LaneType {
    LLeftLane = 2,
    LeftLane = 1,
    EgoLane = 0,
    RightLane = -1,
    RRightLane = -2,
    Unknown = -3,
    TooFarUnDefined = -4
  };
  enum class TrajectoryType { Straight = 0, Curve = 1, Unknown = 2 };

  bool IsTooFar() const;

  int CostByTwoState(double distance, const common::VehicleState& veh_state,
                     const Vec2d& obs_v, const Vec2d& delta_pos,
                     const Vec2d& obs_rel);

  /**
   * @brief set obstacle moving-behavior base on sl-info.
   * @param ref_length
   */
  void IdentifyLaneBehavior(const double ref_length);

  void ProjectOnReferenceLine(const ReferenceLine& reference_line,
                              const double max_proj_s);

  void ObsIsTooFarFromEgo(const EgoVehicleState& ego_state);

  void StateIsStable();

  void TrimTrajectoryHistory(size_t index_traj_extend);

  /**
   * @brief Project each point in the line which connect trajectory start and
   * end point.
   */
  void ProjectionTrajectoryPoint();

  /* lp: **********-----set functions------------------**********/
  void SetIdUnique(int idUnique);
  void SetIdPerception(int idPerception);
  void SetStatePresent(const StateType& statePresent);
  void SetMovingBehavious(MovingBehavior movingBehavious);
  void SetStateHistory(const std::deque<StateType>& stateHistory);
  void SetStateHistoryExtended(
      const std::deque<StateType>& stateHistoryExtended);
  void SetLaneType(LaneType laneType);
  void SetStateSl(const std::vector<std::tuple<SLPoint, LaneType>>& stateSl);
  void SetPerceptionObstaclePtr(
      const std::shared_ptr<TL::perception::PerceptionObstacle>&
          perceptionObstaclePtr);
  void SetIsNeedRemove(bool isNeedRemove);

  /* lp: **********-----get functions------------------**********/
  const StateType& GetStatePresent() const;
  MovingBehavior GetMovingBehavious() const;
  const std::deque<StateType>& GetStateHistory() const;
  LaneType GetLaneType() const;
  auto GetStateSl() const -> const std::vector<std::tuple<SLPoint, LaneType>>&;
  const std::deque<StateType>& GetStateHistoryExtended() const;
  int GetIdUnique() const;
  int GetIdPerception() const;
  inline double GetWidth() const;
  bool IsNeedRemove() const;
  bool IsStable() const;
  const std::shared_ptr<TL::perception::PerceptionObstacle>&
  GetPerceptionObstaclePtr() const;

  /* lp: **********-----------auxiliary function------------**********/

  /**
   * @brief debug function
   * @param file the file where called-function is in
   * @param line the line where called-function is in
   * @param custom custom information
   */
  void DebugObstacle(const std::string& file = "", int line = 0,
                     const std::string& custom = "") const;

 private:
  int id_unique_;
  int id_perception_;
  StateType state_present_;
  bool is_stable_;
  bool is_too_far_;
  bool is_update_consistent_;
  MovingBehavior moving_behavior_;
  std::deque<StateType> state_history_;
  std::deque<StateType> state_history_extended_;
  bool is_need_remove_;
  std::weak_ptr<std::queue<int>> id_unique_pool_ptr_;
  LaneType lane_type_;
  std::vector<std::tuple<SLPoint, LaneType>> state_sl_;
  TL::perception::PerceptionObstacle perception_obstacle_;
  FitManager* fit_curve_;
  signed int polynomial_order_;
  int default_cost_;
  double trajectory_distance_length_ = 0.0;
  double trajectory_time_length_ = 0.0;
  bool trajectory_need_project_ = false;
  TrajectoryType trajectory_type_ = TrajectoryType::Unknown;
  StaticFeature::StaticRes trajectory_static_feature_;

  // lp: for debug
  static constexpr bool debug_cost_flag_ = false;
  std::map<std::string, std::vector<double>> cost_debug_;
  std::vector<std::string> map_name_list_;
  int counter_;
  CoordinateTransform<2, double> coordinate_sys_;
};

using obstacles_ptr_list = std::list<std::shared_ptr<ObstaclePerceptionNoLane>>;

/**
 * @brief FilterObstacleFromMultiTrajectory
 *
 * @param lhs
 */
void FilterObstacleFromMultiTrajectory(obstacles_ptr_list* const lhs);

struct StitchPointInfo {
  std::shared_ptr<ObstaclePerceptionNoLane> obstacle_ptr = nullptr;
  int index;
  int cost = std::numeric_limits<int>::infinity();
  bool is_closed = false;
};

}  // namespace nolane
}  // namespace planning
}  // namespace TL
