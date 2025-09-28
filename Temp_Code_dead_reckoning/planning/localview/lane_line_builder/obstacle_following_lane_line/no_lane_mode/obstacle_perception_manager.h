/******************************************************************************
 * Copyright (c) TL Technologies Co., Ltd. 2019-2022. All rights reserved.
 * Author: LingPeng
 * Created Time: 2022/4/21
 *****************************************************************************/

#pragma once

#include <list>
#include <map>
#include <memory>
#include <queue>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "planning/localview/lane_line_builder/obstacle_following_lane_line/coordinate_system_convert.h"
#include "planning/localview/lane_line_builder/obstacle_following_lane_line/ego_vehicle_state.h"
#include "planning/localview/lane_line_builder/obstacle_following_lane_line/lane_center_line.h"
#include "planning/localview/lane_line_builder/obstacle_following_lane_line/obstacle_perception_no_lane.h"
#include "planning/reference_line/reference_line.h"

#include "proto/common/pnc_point.pb.h"

namespace TL {
namespace planning {
namespace nolane {
class ObstaclePerceptionManager {
 public:
  ObstaclePerceptionManager();

  ~ObstaclePerceptionManager() = default;

  ObstaclePerceptionManager(const ObstaclePerceptionManager& rhs) = delete;
  ObstaclePerceptionManager& operator=(const ObstaclePerceptionManager& rhs) =
      delete;
  /**
   * @brief construct raw perception obstacle to ObstaclePerceptionNoLane type.
   * @param obstacles_raw
   * @param obstacle_present
   */
  bool UpdateObstaclePresent(
      const std::shared_ptr<const perception::PerceptionObstacles>&
          obstacles_raw,
      const CoordinateSystemConvert& coordinate_system_convert,
      const EgoVehicleState& ego_state);

  /**
   * @brief delete some obstacle which has unstable state.
   * @param obstacle_present
   */
  void DeleteUnqualifiedObstacle(obstacles_ptr_list* const obstacle_merged);

  /**
   * @brief merge obstacles in case of obstacle trajectory's value jump.
   * @param obstacles_present
   * @param obstacle_previous
   * @param obstacle_merged
   */
  void MergeObstacles(obstacles_ptr_list* const obstacle_merged);

  /**
   * @brief extend obstacle trajectory both in future and past.
   * @param obstacle_ptr
   */
  void ExtendObstacleTrajectory(obstacles_ptr_list* const obstacle_ptr);

  /**
   * @brief Calculate the cost between ego and obstacle in obstacle_merge
   * loopy.
   * @param obstacle_merge
   * @param ego_state
   * @return
   */
  bool IsEgoInInAllObstacleTrajectory(
      const obstacles_ptr_list& obstacle_merge,
      const EgoVehicleState& ego_state,
      const TL::perception::PerceptionObstacles& obstacles_raw);

  const std::list<StitchPointInfo>& GetObstacleTrajectoryCrossCar() const;

  void DebugString(const obstacles_ptr_list& obstacle_list) const;

  /**
   * @brief project all obstacle in specific reference line.
   * @param prev_lane_ptr
   * @param obstacle_ptr
   */
  bool ProjectTrajectory(const std::shared_ptr<LaneCenterLine>& prev_lane_ptr,
                         obstacles_ptr_list* const obstacle_ptr,
                         EgoVehicleState* const ego_state);

  void SetObstacleTrajectoryCrossCar(
      const std::list<StitchPointInfo>& obstacleTrajectoryCrossCar);
  void SetTimestampPrevious(double timestampPrevious);
  void SetSequenceNumPrevious(unsigned int sequenceNumPrevious);
  void SetObstaclesPresent(const obstacles_ptr_list& obstaclesPresent);
  void SetPerceptionIdObsPrevious(
      const std::map<int,
                     std::vector<std::shared_ptr<ObstaclePerceptionNoLane>>>&
          perceptionIdObsPrevious);
  void SetMaxObstacleNums(int maxObstacleNums);
  void SetIdUniquePool(const std::shared_ptr<std::queue<int>>& idUniquePool);
  void SetIsCloseEnough(bool isCloseEnough);

  const obstacles_ptr_list& GetObstaclesPresent() const;
  const std::map<int, std::vector<std::shared_ptr<ObstaclePerceptionNoLane>>>&
  GetPerceptionIdObsPrevious() const;
  int GetMaxObstacleNums() const;

  /**
   * @brief filter obstacles which is front of ego and call obstacle trim
   * function to delete some points.
   * @param lane_previous
   * @param ego_state
   * @param obstacle_ptr
   */
  void DeleteObsBackEgoAndTrimTraj(
      const std::shared_ptr<LaneCenterLine>& lane_previous,
      const EgoVehicleState& ego_state, obstacles_ptr_list* const obstacle_ptr);

  double GetTimestampPrevious() const;

  std::vector<int> ChooseObstaclesToFollowed();

  const std::shared_ptr<std::queue<int>>& GetIdUniquePool() const;

  unsigned int GetSequenceNumPrevious() const;

  bool IsCloseEnough() const;

  void EgoIsCloseReferenceLine(const EgoVehicleState& ego_state,
                               double start_s);

  void DebugIdMap(const std::string& file = "", int line = 0,
                  const std::string& custom = "") const;

  /**
   * @brief fit each obstacle trajectory in specific curve.
   * @param obstacle_ptr
   */
  void TrajectoryFitForEachObstacle(obstacles_ptr_list* const obstacle_ptr);

  /**
   * @brief DecideFollowType
   *
   * @param v_index
   * @param ego_state
   * @return std::tuple<std::shared_ptr<ObstaclePerceptionNoLane>, int, int>
   */
  StitchPointInfo DecideFollowType(const std::vector<int>& v_index,
                                   const EgoVehicleState& ego_state);

  void ObstaclesPresentToCyber();

  void ObstaclesMergeToCyber(const obstacles_ptr_list& obstacle_ptr);

  void ObstaclesPreviousToCyber();

  void ObstacleAssignToProto(
      const std::shared_ptr<ObstaclePerceptionNoLane>& obs_source,
      ObstacleWithoutLane* const obs_destination) const;

  /**
   * @brief ObstacleRelativePosition
   *
   * @param obs_ptr_raw
   * @param ego_state
   */
  void ObstacleRelativePosition(
      const std::shared_ptr<const TL::perception::PerceptionObstacles>&
          obs_ptr_raw,
      const EgoVehicleState& ego_state);

  /**
   * @brief LateralLimitCheck
   *
   * @param ego_state
   * @return true
   * @return false
   */
  bool LateralLimitCheck(const EgoVehicleState& ego_state);

 private:
  std::shared_ptr<std::queue<int>> id_unique_pool_;

  std::list<StitchPointInfo> obstacle_trajectory_cross_car_;

  double timestamp_previous_;

  unsigned int sequence_num_previous_;

  std::shared_ptr<TL::perception::PerceptionObstacles> obstacles_raw_ptr_;

  obstacles_ptr_list obstacles_present_;

  bool is_close_enough_;

  std::map<int, std::vector<std::shared_ptr<ObstaclePerceptionNoLane>>>
      perception_id_obs_previous_;
  int max_obstacle_nums_;

  std::vector<PercepObsRelPos> obstacle_relative_state_;
};
}  // namespace nolane
}  // namespace planning
}  // namespace TL
