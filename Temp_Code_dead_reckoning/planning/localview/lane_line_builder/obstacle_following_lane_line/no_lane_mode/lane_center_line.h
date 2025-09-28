/******************************************************************************
 * Copyright (c) TL Technologies Co., Ltd. 2019-2022. All rights reserved.
 * Author: LingPeng
 * Created Time: 2022/04/21
 *****************************************************************************/

#pragma once

#include <Eigen/Dense>

#include <limits>
#include <memory>
#include <tuple>
#include <utility>
#include <vector>

#include "common/math/vec2d.h"
#include "common/util/message_util.h"
#include "planning/localview/lane_line_builder/obstacle_following_lane_line/fit_curve/fit_manager.h"
#include "planning/localview/lane_line_builder/obstacle_following_lane_line/obstacle_perception_no_lane.h"

#include "proto/common/pnc_point.pb.h"
#include "proto/map/navigation.pb.h"

namespace TL {
namespace planning {
namespace nolane {
class LaneCenterLine {
 public:
  LaneCenterLine(
      std::vector<Vec2d> points, const EgoVehicleState& ego_state,
      const std::shared_ptr<LaneCenterLine>& previous_lane = nullptr);

  LaneCenterLine(
      std::vector<std::pair<common::SLPoint, Vec2d>> points,
      const EgoVehicleState& ego_state,
      const std::shared_ptr<LaneCenterLine>& previous_lane = nullptr);

  /**
   * @brief construct point-set base on trajectory from specific point, and fit
   * ego position and the point in trajectory.
   * @param obs_followed_index
   * @param ego_state
   */
  LaneCenterLine(const StitchPointInfo& obs_followed_index,
                 const EgoVehicleState& ego_state);

  LaneCenterLine(const LaneCenterLine& rhs) = delete;
  LaneCenterLine& operator=(const LaneCenterLine& rhs) = delete;

  enum class LaneLocation { LeftLane = 0, EgoLane, RightLane, Unknown };

  /**
   * @brief
   * @param obs_state
   * @return true, the obstacle is total in lane.
   */
  bool TrajectoryIsInLane(StateType obs_state);

  /**
   * @brief
   * @param obs_state
   * @return true, part of trajectory is in lane and the rest of trajectory is
   * not in lane.
   */
  LaneLocation IsTrajectoryCrossLane(StateType obs_state);

  /**
   * @brief fit lane-curve base multi-trajectories.
   * @return
   */
  void ConstructLaneLine(bool based_on_traj_line = false);

  bool EgoIsStillInPreviousLane(const EgoVehicleState& veh_state);

  bool CheckPointsIsInLane();

  /**
   * @brief extend lane length to meet downstream requirements.
   */
  void ExtendLaneLength();

  /**
   * @brief creat no lane map and routing by reference_line_ptr_extend.
   */
  bool CreatNoLaneMapAndRouting();

  void RestorePointPosition();

  /**----------get function-------------**/
  const std::shared_ptr<FitManager>& GetFitCurve() const;
  double GetLength() const;
  double GetWidth() const;
  LaneLocation GetLaneLocation() const;
  unsigned int GetId() const;
  const obstacles_ptr_list& GetObstaclesInLane() const;
  const std::vector<std::pair<double, double>>& GetTrajectoriesInLane() const;
  const std::vector<std::pair<double, double>>& GetTrajectoriesCrossed() const;
  const std::vector<Vec2d>& GetTrajectoryPointsRaw() const;
  const std::vector<Vec2d>& GetTrajectoryPointsFit() const;
  const std::vector<Vec2d>& GetTrajectoryPointsXy() const;
  const std::vector<double>& GetTrajectoryPointsHeading() const;
  const std::shared_ptr<ReferenceLine>& GetReferenceLinePtr() const;
  bool IsConstructLaneSuccess() const;
  const std::shared_ptr<ReferenceLine>& GetReferenceLinePtrExtend() const;
  const std::shared_ptr<navigation_hdmap::MapMsg>& GetMapMsgPtr() const;
  const std::shared_ptr<navigation_hdmap::MapMsg>& GetNoLaneMapMsg() const;
  double GetStartS() const;
  /**
   * @brief IsCurveConnectEgoTrajectory
   *
   * @return true
   * @return false
   */
  bool IsCurveConnectEgoTrajectory() const;
  /**
   * @brief get lane line update last time.
   * @return
   */
  double GetTimeUpdateLatest() const;

  /**----------set function-------------**/
  void SetLength(double length);
  void SetWidth(double width);
  void SetLaneLocation(LaneLocation laneLocation);
  void SetId(unsigned int id);
  void SetObstaclesInLane(const obstacles_ptr_list& obstaclesInLane);
  void SetTrajectoriesInLane(
      const std::vector<std::pair<double, double>>& trajectoriesInLane);
  void SetTrajectoriesCrossed(
      const std::vector<std::pair<double, double>>& trajectoriesCrossed);
  void SetTrajectoryPointsRaw(const std::vector<Vec2d>& trajectoryPointsRaw);
  void SetTrajectoryPointsFit(const std::vector<Vec2d>& trajectoryPointsFit);
  void SetTrajectoryPointsXy(const std::vector<Vec2d>& trajectoryPointsXy);
  void SetTrajectoryPointsHeading(
      const std::vector<double>& trajectoryPointsHeading);
  void SetReferenceLinePtr(
      const std::shared_ptr<ReferenceLine>& referenceLinePtr);
  void SetConstructLaneSuccess(bool constructLaneSuccess);
  void SetLaneCenterLinePtr(
      const std::shared_ptr<LaneCenterLine>& laneCenterLinePtr);

  /**
   * @brief Set the Curve Connect Ego Trajectory object
   *
   * @param curveConnectEgoTrajectory
   */
  void SetCurveConnectEgoTrajectory(bool curveConnectEgoTrajectory);

  /**
   * @brief set lane line latest update time
   * @param timeUpdateLatest
   */
  void SetTimeUpdateLatest(double timeUpdateLatest);

 private:
  /**
   * @brief generate path based map-path points
   * @param map_path
   * @return
   */
  common::Path GeneratePathFromPoints(const hdmap::Path& map_path);

  bool GenerateOneLane(const common::Path& path, hdmap::Lane* lane);
  bool SetNoLaneRouting(TL::routing::RoutingResponse* inrouting,
                        TL::hdmap::Map* hd_map);

  unsigned int id_ = 0;
  std::shared_ptr<FitManager> fit_curve_;
  double length_ = 300.0;
  double back_ego_length_ = 2.0;
  double width_ = 3.75;
  Vec2d ego_position_{0, 0};
  SLPoint ego_sl_pos_;
  obstacles_ptr_list obstacles_in_lane_;
  LaneLocation lane_location_{LaneLocation::Unknown};
  std::vector<TL::planning::ReferencePoint> ref_path_point_;
  std::vector<Vec2d> trajectory_points_raw_;
  std::vector<Vec2d> trajectory_points_fit_;
  std::vector<Vec2d> trajectory_points_output_xy_;
  std::vector<double> trajectory_points_heading_;
  std::shared_ptr<ReferenceLine> reference_line_ptr_;
  std::shared_ptr<ReferenceLine> reference_line_ptr_extend_;
  std::vector<std::pair<double, double>> trajectories_in_lane_;
  std::vector<std::pair<double, double>> trajectories_crossed_;
  bool construct_lane_success_ = false;
  bool curve_connect_ego_trajectory_ = false;
  std::shared_ptr<LaneCenterLine> previous_lane_center_line_;
  double time_update_latest_ = 0.0;

  std::vector<std::pair<common::SLPoint, Vec2d>> fit_raw_points_;

  // lp: map member
  std::shared_ptr<navigation_hdmap::MapMsg> no_lane_map_msg_;

  StitchPointInfo obs_followed_index_;
  double start_s_ = std::numeric_limits<double>::infinity();
};
}  // namespace nolane
}  // namespace planning
}  // namespace TL
