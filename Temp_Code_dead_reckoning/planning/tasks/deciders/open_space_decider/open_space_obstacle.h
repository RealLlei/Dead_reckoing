/*
 * Copyright (c) TL Technologies Co., Ltd. 2022. All rights reserved.
 * Description:  open_space_obstacle.h
 */

#pragma once

#include <algorithm>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "common/math/line_segment2d.h"
#include "common/math/vec2d.h"
#include "planning/common/dependency_injector.h"
#include "proto/perception/perception_freespace.pb.h"
#include "proto/perception/perception_obstacle.pb.h"

namespace TL {
namespace planning {

static constexpr int kParkingLotVertexNum = 4;
static constexpr int kParkingLotVertexWithWheekMaskNum = 6;
static constexpr double kMinVirtualObsLength = 0.1;
static constexpr double kBtmLowFsBuffer = 0.05;
static constexpr double kBTMLowFsBufferForLatParkOut = 0.02;
static constexpr double kHighCurbBuffer = 0.25;
static constexpr double kADCBoxEps = 0.001;
static constexpr double kNarrowObsLength = 3.5;
static constexpr double kUseBBoxDistThreshold = 3.5;
static constexpr double kEps = 1e-5;

using ParkingLotVertexType = std::array<common::math::Vec2d, 4>;

enum ParkLotStatus {
  NORMAL = 0,
  INCOMPLETE = 1,
  POSITION_ERROR = 2,
  SMALL = 3,
  UNFREE = 4,
  NONCONVEX = 5,
};

// Convert ParkLotStatus enum value to string representation
std::string ParkLotStatusToString(const ParkLotStatus& status);

struct ParkLotInfo {
  perception::ParkingLotOut::ParkType park_type =
      perception::ParkingLotOut::NONE;
  // 4 vertices: lt: ld, rd, rt
  ParkingLotVertexType vertices;
  // 0: is valid; left wheel mask, right wheel mask
  std::tuple<bool, Vec2d, Vec2d> wheel_mask = {false, {0, 0}, {0, 0}};
  ParkLotStatus status = NORMAL;
  bool is_parking_lot_update = false;
  bool is_high_quality_triggered = false;
  bool is_right_side = false;
  bool is_narrow_spot = false;
  perception::ParkingLotOut::SenType sensor_type;
};

enum OpenSpaceFilterObsType {
  COMMON_OBS = 0,
  USS_OBS = 1,
  BOX_OBS = 2,
  FREE_SPACE_OBS = 3,
  LOW_FREE_SPACE_OBS = 4,
  WHEEL_MASK_OBS = 5,
  HIGH_CURB_FREE_SPACE_OBS = 6,
};

struct ObsFilter {
  bool use_obstacle = false;
  // filter obs has interact with area
  // true: ignore inner obs; false: ignore outer
  std::vector<std::pair<common::math::Polygon2d, bool>> filter_areas;
  // true: filter upper; false: filter lower
  std::vector<std::pair<common::math::LineSegment2d, bool>> filter_planes;

  std::string ShortDebugString() const {
    return absl::StrCat(
        "ObsFilter: \n use_obstacle = ", use_obstacle ? "True" : "False",
        " \n filter_areas size = ", filter_areas.size(),
        " \n filter_planes = ", filter_planes.size());
  }
};

using ObsFilterMap = std::unordered_map<OpenSpaceFilterObsType, ObsFilter>;

class OpenSpaceObstacle {
 public:
  explicit OpenSpaceObstacle(const TaskConfig& config);
  ~OpenSpaceObstacle() = default;

  /**
   * @brief 
   * 
   * @param free_space_array_ptr 
   * @param obstacles 
   * @param spd_collision_info 
   * @param cur_adc_pose 
   * @return TL::common::Status 
   */
  TL::common::Status Init(
      const std::shared_ptr<const perception::FreeSpaceOutArray>&
          free_space_array_ptr,
      const ThreadSafeIndexedObstacles* obstacles,
      const common::PathPoint& cur_adc_pose);

  /**
   * @brief Load obstacle
   * 
   * @param parking_spot_enu 
   * @param obs_filter_map
   * @param spd_collision_info 
   * @param is_lateral_park_out 
   * @param replan_triggered_by_speed_plan 
   * @param is_nns_adjust_scenario search path in cruising
   * @param obstacles 
   * @param obs_ptr 
   * @param linked_obs_ptr 
   * @param high_curb_fs_obs_ptr 
   * @param low_fs_obs_ptr
   * @return apollo::common::Status 
   */
  TL::common::Status LoadObs(
      const ParkingLotVertexType& parking_spot_enu,
      const ObsFilterMap& obs_filter_map,
      const AvpSpeedPlanCollisionInfo& spd_collision_info,
      bool is_lateral_park_out, bool replan_triggered_by_speed_plan,
      bool is_nns_adjust_scenario, const ThreadSafeIndexedObstacles* obstacles,
      std::vector<std::pair<common::math::LineSegment2d, double>>* obs_ptr,
      std::vector<std::pair<common::math::LineSegment2d, double>>*
          linked_obs_ptr,
      std::vector<std::pair<common::math::LineSegment2d, double>>*
          high_curb_fs_obs_ptr,
      std::vector<std::pair<common::math::LineSegment2d, double>>*
          low_fs_obs_ptr);

  /**
 * @brief 
 * 
 * @param parking_type 
 * @param park_lot_info 
 * @param open_space_path_info 
 * @param free_space_array_ptr 
 * @param obstacles 
 * @param roi_boundary 
 * @param init_adc_pose 
 * @param cur_adc_pose 
 * @param is_consider_wheel_mask 
 * @param wheel_mask_box 
 * @param spd_collision_info 
 * @param obs_ptr 
 */
  void AddVirtualObs(
      const AVPStatus::ParkingType& parking_type,
      const ParkLotInfo& park_lot_info,
      const OpenSpacePathInfo& open_space_path_info,
      const std::shared_ptr<const perception::FreeSpaceOutArray>&
          free_space_array_ptr,
      const ThreadSafeIndexedObstacles* obstacles,
      const std::vector<common::math::LineSegment2d>& roi_boundary,
      const common::PathPoint& init_adc_pose,
      const common::PathPoint& cur_adc_pose, bool is_consider_wheel_mask,
      const common::math::Box2d& wheel_mask_box,
      const planning_internal::AvpSpeedPlanCollisionInfo& spd_collision_info,
      std::vector<std::pair<common::math::LineSegment2d, double>>* obs_ptr);

  /**
   * @brief Update open space info for spd tasks
   * 
   * @param parking_type 
   * @param park_lot_type 
   * @param parking_spot_enu 
   * @param free_space_array_ptr 
   * @param init_adc_pose 
   * @param open_space_info_ptr 
   */
  static void UpdateOpenSpaceInfoForSpd(
      const AVPStatus::ParkingType& parking_type,
      const perception::ParkingLotOut::ParkType& park_lot_type,
      const ParkingLotVertexType& parking_spot_enu,
      const std::shared_ptr<const perception::FreeSpaceOutArray>&
          free_space_array_ptr,
      const common::PathPoint& init_adc_pose,
      OpenSpaceInfo* open_space_info_ptr);

  /**
   * @brief 
   * 
   * @param parking_type 
   * @param park_lot_info 
   * @param free_space_array_ptr 
   * @return uint32_t 
   */
  uint32_t ScenarioDiffcultyDecison(
      const AVPStatus::ParkingType& parking_type,
      const ParkLotInfo& park_lot_info,
      const std::shared_ptr<const perception::FreeSpaceOutArray>&
          free_space_array_ptr);

  /**
    * @brief 
    * 
    * @param parking_type 
    * @param park_lot_type 
    * @param parking_spot_enu 
    * @param free_space_array_ptr 
    * @param reference_curb 
    * @return true 
    * @return false 
    */
  static bool GetParkOutCurbSeg(
      const AVPStatus::ParkingType& parking_type,
      const perception::ParkingLotOut::ParkType& park_lot_type,
      const ParkingLotVertexType& parking_spot_enu,
      const std::shared_ptr<const perception::FreeSpaceOutArray>&
          free_space_array_ptr,
      common::math::LineSegment2d* reference_curb);

 private:
  /**
   * @brief 
   * 
   * @param adc_polygon2d 
   * @param obstacle 
   */
  void InitUssObs(const common::math::Polygon2d& adc_polygon2d,
                  const TL::planning::Obstacle& obstacle);

  /**
   * @brief 
   * 
   * @param obstacle 
   */
  void InitWheelMaskObs(const TL::planning::Obstacle& obstacle);

  /**
   * @brief 
   * 
   * @param obstacle 
   */
  void InitBBoxObs(const TL::planning::Obstacle& obstacle);

  /**
   * @brief 
   * 
   * @param free_space_array_ptr 
   */
  void InitFsObs(const std::shared_ptr<const perception::FreeSpaceOutArray>&
                     free_space_array_ptr);

  /**
   * @brief 
   * 
   * @param obs_filter_map 
   * @param obs_ptr 
   */
  void LoadUssObs(
      const ObsFilterMap& obs_filter_map,
      std::vector<std::pair<common::math::LineSegment2d, double>>* obs_ptr);

  /**
   * @brief 
   * @param obs_ptr 
   */
  void LoadWheelMaskObs(
      std::vector<std::pair<common::math::LineSegment2d, double>>* obs_ptr);

  /**
   * @brief 
   * 
   * @param obs_filter_map 
   * @param obs_ptr 
   */
  void LoadBBoxObs(
      const ObsFilterMap& obs_filter_map,
      std::vector<std::pair<common::math::LineSegment2d, double>>* obs_ptr);

  /**
   * @brief 
   * 
   * @param obs_filter_map 
   * @param obs_ptr 
   * @param linked_obs_ptr 
   * @param high_curb_fs_obs_ptr 
   */
  void LoadFsObs(
      const ObsFilterMap&,
      std::vector<std::pair<common::math::LineSegment2d, double>>* obs_ptr,
      std::vector<std::pair<common::math::LineSegment2d, double>>*
          linked_obs_ptr,
      std::vector<std::pair<common::math::LineSegment2d, double>>*
          high_curb_fs_obs_ptr);

  /**
   * @brief 
   * 
   * @param obs_filter_map 
   * @param park_lot_type
   * @param is_lateral_park_out
   * @param parking_spot_enu 
   * @param obs_ptr
   * @param low_fs_obs_ptr 
   */
  void LoadLowFsObs(
      const ObsFilterMap& obs_filter_map,
      const ParkingLotVertexType& parking_spot_enu, bool is_lateral_park_out,
      std::vector<std::pair<common::math::LineSegment2d, double>>* obs_ptr,
      std::vector<std::pair<common::math::LineSegment2d, double>>*
          low_fs_obs_ptr);

  /**
   * @brief 
   * 
   * @param obs_filter_map 
   * @param obs_ptr 
   */
  void LoadWheelMaskObs(
      const ObsFilterMap& obs_filter_map,
      std::vector<std::pair<common::math::LineSegment2d, double>>* obs_ptr);

  /**
   * @brief unfify obstacle to line segment
   * 
   * @param parking_type 
   * @param park_lot_type 
   * @param parking_spot_enu 
   * @param free_space_array_ptr 
   * @param obstacles 
   * @param obs_filter_map 
   * @param init_adc_pose 
   * @param cur_adc_pose 
   * @param open_space_info_ptr 
   */
  void UnifyObstacles(
      const AVPStatus::ParkingType& parking_type,
      const perception::ParkingLotOut::ParkType& park_lot_type,
      const ParkingLotVertexType& parking_spot_enu,
      const std::shared_ptr<const perception::FreeSpaceOutArray>&
          free_space_array_ptr,
      const ThreadSafeIndexedObstacles* obstacles,
      const ObsFilterMap& obs_filter_map,
      const common::PathPoint& init_adc_pose,
      const common::PathPoint& cur_adc_pose,
      OpenSpaceInfo* open_space_info_ptr);

  /**
   * @brief 
   * 
   * @param is_right_slot 
   * @param is_parking_inwards
   * @param is_high_quality_triggered
   * @param parking_spot_enu 
   * @param end_pose_enu 
   * @param free_space_array_ptr 
   * @param is_narrow_spot_scenario 
   * @param left_virtual_obstacle_length
   * @param right_virtual_obstacle_length
   */
  void GetVerticalSpotParkInVirtualObsLength(
      bool is_right_slot, bool is_parking_inwards,
      bool is_high_quality_triggered,
      const ParkingLotVertexType& parking_spot_enu,
      const common::PathPoint& end_pose_enu,
      const std::shared_ptr<const perception::FreeSpaceOutArray>&
          free_space_array_ptr,
      bool is_narrow_spot_scenario, double* left_virtual_obstacle_length,
      double* right_virtual_obstacle_length);

  /**
   * @brief 
   * 
   * @param parking_spot_enu 
   * @param left_corridor_filter 
   * @param right_corridor_filter 
   * @return true 
   * @return false 
   */
  static bool BuildVerticalSpotParkInVirtualObsFSFilterArea(
      const ParkingLotVertexType& parking_spot_enu,
      common::math::Polygon2d* left_corridor_filter,
      common::math::Polygon2d* right_corridor_filter);

  /**
   * @brief 
   * 
   * @param is_parking_inwards
   * @param end_pose_enu 
   * @param fs_point 
   * @return double 
   */
  double GetMinVirtualObsLengthBasedOnFS(bool is_parking_inwards,
                                         const common::PathPoint& end_pose_enu,
                                         const Vec2d& fs_point);

  /**
   * @brief Add vertical spot park-in virtual obs
   * 
   * @param is_right_slot 
   * @param is_parking_inwards
   * @param lateral_nearest_dist_to_boundary 
   * @param adc_polygon 
   * @param end_pose_enu 
   * @param parking_spot_enu 
   * @param free_space_array_ptr 
   * @param is_narrow_spot_scenario 
   * @param is_high_quality_triggered 
   * @param virtual_obstacles_ptr 
   */
  void AddVerticalSpotParkInVirtualObs(
      bool is_right_slot, bool is_parking_inwards,
      double lateral_nearest_dist_to_boundary,
      const common::math::Polygon2d& adc_polygon,
      const common::PathPoint& end_pose_enu,
      const ParkingLotVertexType& parking_spot_enu,
      const std::shared_ptr<const perception::FreeSpaceOutArray>&
          free_space_array_ptr,
      bool is_narrow_spot_scenario, bool is_high_quality_triggered,
      std::vector<common::math::LineSegment2d>* virtual_obstacles_ptr);

  /**
   * @brief Add lateral spot park-in virtual obs
   * 
   * @param cur_adc_pose 
   * @param end_pose_enu 
   * @param is_consider_wheel_mask 
   * @param wheel_mask_box 
   * @param virtual_obstacles_ptr 
   */
  void AddLateralSpotParkInVirtualObs(
      const common::PathPoint& cur_adc_pose,
      const common::PathPoint& end_pose_enu, bool is_consider_wheel_mask,
      const common::math::Box2d& wheel_mask_box,
      std::vector<common::math::LineSegment2d>* virtual_obstacles_ptr);

  /**
   * @brief Add virtual obstacle in vertical park out
   * 
   * @param init_adc_pose 
   * @param is_narrow_spot_scenario 
   * @param virtual_obstacles_ptr 
   */
  void AddVerticalSpotParkOutVirtualObs(
      const common::PathPoint& init_adc_pose, bool is_narrow_spot_scenario,
      std::vector<common::math::LineSegment2d>* virtual_obstacles_ptr);

  /**
   * @brief set speed collision obstacle as virtual obstacles unless it's curb
   * 
   * @param obstacles 
   * @param spd_collision_info 
   * @param adc_pose 
   */
  void LoadSpdCollisionObsSegments(
      const ThreadSafeIndexedObstacles* obstacles,
      const planning_internal::AvpSpeedPlanCollisionInfo& spd_collision_info,
      const common::PathPoint& adc_pose);
  /**
   * @brief Get the Nearest Adc Boundary object
   * 
   * @param adc_polygon 
   * @param point 
   * @param nearest_seg 
   * @return true 
   * @return false 
   */
  static bool GetNearestAdcBoundary(const common::math::Polygon2d& adc_polygon,
                                    const Vec2d& point,
                                    common::math::LineSegment2d* nearest_seg);
  /**
   * @brief construct obstacle wall parallel to the nearest side of the vehicle
   *
   * @param adc_polygon ego vehicle polygon2d
   * @param uss_point_flu uss dectect point in flu coordinate
   * @return common::math::LineSegment2d uss obstacle wall
   */
  common::math::LineSegment2d ConstructUssWall(
      const common::math::Polygon2d& adc_polygon, const Vec2d& uss_point_enu);

  /**
   * @brief 
   * 
   * @param parking_type 
   * @param park_lot_type 
   * @param is_right_turn_side 
   * @param free_space_array_ptr 
   * @param cur_adc_pose 
   * @param end_pose_enu 
   * @param parking_spot_enu 
   * @param lateral_nearest_dist_to_boundary 
   */
  void AddTurnSideVirtualObs(
      const AVPStatus::ParkingType& parking_type,
      const perception::ParkingLotOut::ParkType& park_lot_type,
      bool is_right_turn_side,
      const std::shared_ptr<const perception::FreeSpaceOutArray>&
          free_space_array_ptr,
      const common::PathPoint& cur_adc_pose,
      const common::PathPoint& end_pose_enu,
      const ParkingLotVertexType& parking_spot_enu,
      double* lateral_nearest_dist_to_boundary);

  /**
   * @brief 
   * 
   * @param cur_adc_polygon 
   * @param endpose_adc_polygon 
   * @param fs_point 
   * @param parking_type 
   * @param park_lot_type 
   * @param parking_spot_enu 
   */
  void InsertTurnSideVirtualObs(
      const common::math::Polygon2d& cur_adc_polygon,
      const common::math::Polygon2d& endpose_adc_polygon,
      const common::math::Vec2d& fs_point,
      const AVPStatus::ParkingType& parking_type,
      const perception::ParkingLotOut::ParkType& park_lot_type,
      const ParkingLotVertexType& parking_spot_enu);

  /**
   * @brief Represent the obstacles in vertices and load it into 
   * obstacles_vertices_vec_ in clock wise order. Take different approach
   * towards warm start and distance approach
   * 
   * @param roi_boundary 
   * @param open_space_info_ptr 
   * @return true 
   * @return false 
   */
  bool LoadObstacleInVertices(
      const std::vector<common::math::LineSegment2d>& roi_boundary,
      OpenSpaceInfo* open_space_info_ptr);

  /**
   * @brief 
   * 
   * @param adc_polygon2d 
   * @param obstacles 
   * @param obs_filter_map 
   */
  void LoadUssObs(const common::math::Polygon2d& adc_polygon2d,
                  const TL::planning::Obstacle& obstacles,
                  const ObsFilterMap& obs_filter_map);

  /**
   * @brief 
   * 
   * @param obs_filter_map 
   */
  void UssObsFilter(const ObsFilterMap& obs_filter_map);

  /**
   * @brief  Load Obstacle Box Segments
   *
   * @param obstacle
   */
  void LoadObstacleBoxSegments(const TL::planning::Obstacle& obstacle,
                               const ObsFilterMap& obs_filter_map);

  /**
   * @brief Load Freespace Segments
   * 
   * @param parking_type 
   * @param park_lot_type 
   * @param parking_spot_enu 
   * @param free_space_array_ptr 
   * @param obs_filter_map 
   * @param init_adc_pose 
   * @param open_space_info_ptr 
   */
  void LoadFreespacePointSegments(
      const AVPStatus::ParkingType& parking_type,
      const perception::ParkingLotOut::ParkType& park_lot_type,
      const ParkingLotVertexType& parking_spot_enu,
      const std::shared_ptr<const perception::FreeSpaceOutArray>&
          free_space_array_ptr,
      const ObsFilterMap& obs_filter_map,
      const common::PathPoint& init_adc_pose,
      OpenSpaceInfo* open_space_info_ptr);

  /**
   * @brief 
   * 
   * @param filter_areas 
   * @param obs 
   * @return true 
   * @return false 
   */
  static bool FilterPointBaseArea(
      const std::vector<std::pair<common::math::Polygon2d, bool>>& filter_areas,
      const common::math::Vec2d& obs);

  /**
   * @brief 
   * 
   * @param filter_planes 
   * @param obs 
   * @return true 
   * @return false 
   */
  static bool FilterPointBasePlane(
      const std::vector<std::pair<common::math::LineSegment2d, bool>>&
          filter_planes,
      const common::math::Vec2d& obs);

  /**
   * @brief 
   * 
   * @param filter_areas 
   * @param obs_seg 
   * @return true 
   * @return false 
   */
  static bool FilterSegtBaseArea(
      const std::vector<std::pair<common::math::Polygon2d, bool>>& filter_areas,
      const common::math::LineSegment2d& obs_seg);

  /**
   * @brief 
   * 
   * @param filter_planes 
   * @param obs_seg 
   * @return true 
   * @return false 
   */
  static bool FilterSegBasePlane(
      const std::vector<std::pair<common::math::LineSegment2d, bool>>&
          filter_planes,
      const common::math::LineSegment2d& obs_seg);

  /**
   * @brief filters obstacles based on a given obstacle filter map.
   * 
   * @param obs_filter_map 
   * @param obs_ptr 
   */
  static void FilterObs(
      const ObsFilterMap& obs_filter_map,
      std::vector<std::pair<common::math::LineSegment2d, double>>* obs_ptr);

  /**
   * @brief Adjust the obstacle buffer based on the parking spot
   * 
   * @param parking_spot_enu 
   * @param spd_collision_info 
   * @param replan_triggered_by_speed_plan 
   * @param obs_ptr 
   */
  static void AdjustObsBuffer(
      const ParkingLotVertexType& parking_spot_enu,
      const AvpSpeedPlanCollisionInfo& spd_collision_info,
      bool replan_triggered_by_speed_plan,
      const ThreadSafeIndexedObstacles* obstacles,
      std::vector<std::pair<common::math::LineSegment2d, double>>* obs_ptr);

  /**
   * @brief 
   * 
   * @param plane 
   * @param spd_collision_info 
   * @param replan_triggered_by_speed_plan 
   * @param obstacles 
   */
  static bool IsBlockByOutsideObs(
      const common::math::LineSegment2d& plane,
      const AvpSpeedPlanCollisionInfo& spd_collision_info,
      bool replan_triggered_by_speed_plan,
      const ThreadSafeIndexedObstacles* obstacles);

  /**
 * @brief 
 * 
 * @param parking_type 
 * @param park_lot_type 
 * @param parking_spot_enu 
 * @param is_right_turn_side 
 * @param longitudal_filter_length 
 * @param lateral_filter 
 * @return true 
 * @return false 
 */
  bool BuildTurnSideFSFilterArea(
      const AVPStatus::ParkingType& parking_type,
      const perception::ParkingLotOut::ParkType& park_lot_type,
      const ParkingLotVertexType& parking_spot_enu, bool is_right_turn_side,
      double longitudal_filter_length, common::math::Polygon2d* lateral_filter);
  /**
  * @brief 
  * 
  * @param free_space_array_ptr 
  * @param filter 
  * @param boundary 
  * @param nearest_dist_to_boundary 
  * @param nearest_fs_point 
  */
  static void FindNearestFSPoint(
      const std::shared_ptr<const perception::FreeSpaceOutArray>&
          free_space_array_ptr,
      const common::math::Polygon2d& filter,
      const common::math::LineSegment2d& boundary,
      double* nearest_dist_to_boundary, Vec2d* nearest_fs_point);

  /**
  * @brief 
  * 
  * @param free_space_item 
  * @param filter 
  * @param boundary 
  * @param nearest_dist_to_boundary 
  * @param nearest_fs_point 
  */
  static void FindNearestFSPointInItem(
      const perception::FreeSpaceOut& free_space_item,
      const common::math::Polygon2d& filter,
      const common::math::LineSegment2d& boundary,
      double* nearest_dist_to_boundary, Vec2d* nearest_fs_point);

  /**
   * @brief 
   * 
   * @param fs_points free space point array
   * @param lmid_to_rmid spot left mid point to right mid point
   * @return true 
   * @return false 
   */
  static bool IsFsLowerThanSpot(
      const RepeatedPtrField<::TL::common::Point3D>& fs_points,
      const common::math::LineSegment2d& lmid_to_rmid) {
    return !fs_points.empty() &&
           std::all_of(fs_points.begin(), fs_points.end(), [&](const auto& p) {
             return lmid_to_rmid.ProductOntoUnit({p.x(), p.y()}) < 0.0;
           });
  }

  /**
 * @brief 
 * 
 * @param is_narrow_spot_scenario 
 * @param lateral_nearest_dist_to_boundary 
 * @param parking_spot_enu 
 * @return double 
 */
  double ParkInObsLengthLateralConstraint(
      bool is_narrow_spot_scenario, double lateral_nearest_dist_to_boundary,
      const ParkingLotVertexType& parking_spot_enu);

  /**
 * @brief 
 * 
 * @param is_right_turn_side 
 * @param free_space_array_ptr 
 * @param parking_spot_enu 
 * @return double 
 */
  double ParkInObsLengthLongitudinalConstraint(
      bool is_right_turn_side,
      const std::shared_ptr<const perception::FreeSpaceOutArray>&
          free_space_array_ptr,
      const ParkingLotVertexType& parking_spot_enu);

  /**
   * @brief 
   * 
   * @param is_right_turn_side 
   * @param parking_spot_enu 
   * @param corridor_filter 
   * @return true 
   * @return false 
   */
  bool BuildParkInCorridorFSFilterArea(
      bool is_right_turn_side, const ParkingLotVertexType& parking_spot_enu,
      common::math::Polygon2d* corridor_filter);

  /**
   * @brief 
   * 
   * @param is_right_slot 
   * @param parking_spot_enu 
   * @param corridor_filter 
   * @return true 
   * @return false 
   */
  static bool BuildDeadendScenarioDeciderFSFilterArea(
      bool is_right_slot, const ParkingLotVertexType& parking_spot_enu,
      common::math::Polygon2d* corridor_filter);

  /**
   * @brief 
   * 
   * @param parking_spot_enu 
   * @param left_corridor_filter 
   * @param right_corridor_filter 
   * @return true 
   * @return false 
   */
  static bool BuildNarrowSpotScenarioDeciderFSFilterArea(
      const ParkingLotVertexType& parking_spot_enu,
      common::math::Polygon2d* left_corridor_filter,
      common::math::Polygon2d* right_corridor_filter);

  /**
   * @brief 
   * 
   * @param parking_spot_enu
   * @param passage_corridor_filter
   * @param filter_left_down_top_vec
   * @return true 
   * @return false 
   */
  static bool BuildNarrowPassageScenarioDeciderFSFilterArea(
      const ParkingLotVertexType& parking_spot_enu,
      common::math::Polygon2d* passage_corridor_filter,
      common::math::LineSegment2d* filter_left_edge);

  /**
   * @brief 
   * 
   * @param parking_type
   * @param park_lot_type
   * @param parking_spot_enu
   * @param free_space_array_ptr
   * @return true 
   * @return false 
   */
  static bool IsNarrowPassageScenario(
      const AVPStatus::ParkingType& parking_type,
      const perception::ParkingLotOut::ParkType& park_lot_type,
      const ParkingLotVertexType& parking_spot_enu,
      const std::shared_ptr<const perception::FreeSpaceOutArray>&
          free_space_array_ptr);

  /**
   * @brief 
   * 
   * @param park_lot_info 
   * @return true 
   * @return false 
   */
  static bool IsNarrowSpotScenario(const ParkLotInfo& park_lot_info);

  /**
   * @brief 
   * 
   * @param parking_type 
   * @param park_lot_type 
   * @param is_right_slot 
   * @param parking_spot_enu 
   * @param free_space_array_ptr 
   * @return true 
   * @return false 
   */
  bool IsDeadendScenario(
      const AVPStatus::ParkingType& parking_type,
      const perception::ParkingLotOut::ParkType& park_lot_type,
      bool is_right_slot, const ParkingLotVertexType& parking_spot_enu,
      const std::shared_ptr<const perception::FreeSpaceOutArray>&
          free_space_array_ptr);

  /**
    * @brief 
    * 
    * @param parking_type 
    * @param park_lot_type 
    * @param parking_spot_enu 
    * @param corridor_filter 
    * @return true 
    * @return false 
    */
  static bool BuildParkOutCurbFSFilterArea(
      const AVPStatus::ParkingType& parking_type,
      const perception::ParkingLotOut::ParkType& park_lot_type,
      const ParkingLotVertexType& parking_spot_enu,
      common::math::Polygon2d* corridor_filter);

  /**
   * @brief 
   * 
   * @param fs_cls
   * @param fs_height
   * @return true 
   * @return false 
   */
  static bool IsLowFs(const perception::FreeSpaceOut::ClassType& fs_cls,
                      const perception::FreeSpaceOut::HeightType& fs_height) {
    if (fs_cls == perception::FreeSpaceOut::CURBSTONE ||
        fs_cls == perception::FreeSpaceOut::OTHER_CLASS) {
      return fs_height == perception::FreeSpaceOut::UNDERDRIVABLE;
    }
    return false;
  }

  /**
   * @brief 
   * 
   * @param fs_cls
   * @param fs_height
   * @return true 
   * @return false 
   */
  static bool IsHighCurbFs(
      const perception::FreeSpaceOut::ClassType& fs_cls,
      const perception::FreeSpaceOut::HeightType& fs_height) {
    return (fs_cls == perception::FreeSpaceOut::CURBSTONE) &&
           (fs_height == perception::FreeSpaceOut::OVERDRIVABLE);
  }

  std::shared_ptr<DependencyInjector> injector_;
  TaskConfig config_;
  TL::common::VehicleParam vehicle_params_;
  std::vector<std::pair<common::math::LineSegment2d, double>> uss_obs_;
  std::vector<std::pair<common::math::LineSegment2d, double>> fs_obs_;
  std::vector<std::pair<common::math::LineSegment2d, double>> low_fs_obs_;
  std::vector<std::pair<common::math::LineSegment2d, double>> high_curb_fs_obs_;
  std::vector<std::pair<common::math::LineSegment2d, double>> box_obs_;
  std::vector<std::pair<common::math::LineSegment2d, double>> virtual_obs_;
  std::vector<std::pair<common::math::LineSegment2d, double>> wheel_mask_obs_;
  std::unordered_set<int> linked_fs_obs_idx_set_;
  std::unordered_set<int> high_curb_fs_obs_idx_set_;
};

}  // namespace planning
}  // namespace TL
