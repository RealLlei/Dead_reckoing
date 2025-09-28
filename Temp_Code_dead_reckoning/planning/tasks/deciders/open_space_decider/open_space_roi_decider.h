/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 **/

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>
#include "Eigen/Dense"

#include "common/math/vec2d.h"
#include "planning/common/open_space_info.h"
#include "planning/tasks/deciders/decider.h"
#include "planning/tasks/deciders/open_space_decider/open_space_fine_tuning.h"
#include "planning/tasks/deciders/open_space_decider/open_space_obstacle.h"

#include "planning/proto/map_parking.pb.h"
#include "proto/common/pnc_point.pb.h"
#include "proto/common/vehicle_config.pb.h"
#include "proto/common/vehicle_state.pb.h"
#include "proto/perception/perception_freespace.pb.h"
#include "proto/perception/perception_obstacle.pb.h"
#include "proto/perception/perception_parking_lot.pb.h"

namespace TL {
namespace planning {
enum ParkingLotType {
  vertical_type = 0,
  lateral_type = 1,
  oblique_type = 2,
};

static constexpr double kMinTraceDis = 1.0;

enum SensorConfigState {
  NONINITAILZED = 0,
  USS = 1,
  CAMERA = 2,
  LIDAR = 3,
};

class OpenSpaceRoiDecider : public Decider {
 public:
  OpenSpaceRoiDecider(const TaskConfig& config,
                      const std::shared_ptr<DependencyInjector>& injector);

  TL::common::Status Reset() override;

 private:
  TL::common::Status Process(Frame* frame) override;

  /**
   * @brief Get the Parking Spot object
   * 
   * @param frame 
   * @param spot_vertices parking lot vertices
   * @param wheel_mask_ptr parking lot wheel mask pointer
   * @param is_parking_lot_update is parking lot has been update in current frame
   * @return std::string error code msg
   */
  std::string GetParkingSpot(Frame* frame, ParkingLotVertexType* spot_vertices,
                             std::pair<Vec2d, Vec2d>* wheel_mask_ptr,
                             bool* is_parking_lot_update);

  /**
   * @brief 
   * 
   * @param left_top_enu 
   * @param right_top_enu 
   * @return true 
   * @return false 
   */
  bool IsParkLotInRightSide(const Vec2d& left_top_enu,
                            const Vec2d& right_top_enu);
  /**
   * @brief Get the Parking Spot From Space object
   * 
   * @param frame 
   * @param parking_spot_enu parking lot vertices coordinate enu
   * @param wheel_mask_ptr parking lot wheel mask pointer
   * @param is_parking_lot_update is parking lot has been update in current frame
   * @return std::string error code msg
   */
  std::string GetParkingSpotFromSpace(
      Frame* frame, std::vector<common::math::Vec2d>* parking_spot_enu,
      std::pair<Vec2d, Vec2d>* wheel_mask_ptr, bool* is_parking_lot_update);

  /**
   * @brief Get the Select Slot Idx object
   * 
   * @param exceptspace_info 
   * @return int 
   */
  int GetSelectSlotIdx(
      std::shared_ptr<const ::TL::perception::ParkingLotOutArray>&
          exceptspace_info);

  /**
   * @brief check current frame is valid or not
   *
   * @param frame frame with parking lot
   * @param parking_spot_enu parking lot in enu coordinate
   * @param wheel_mask_ptr parking lot wheel mask
   * @param isHighQuality all vetext is high quality
   * @return std::string is valid frame or not
   */
  std::string IsValidFrame(Frame* frame,
                           std::vector<common::math::Vec2d>* parking_spot_enu,
                           std::pair<Vec2d, Vec2d>* wheel_mask_ptr,
                           bool* is_parking_lot_vertex_high_quality,
                           bool* is_wheel_mask_high_quality);

  /**
   * @brief check parking lot shape is proper to parking in
   * 
   * @param park_type 
   * @param parking_spot_enu 
   * @param park_lot_status_ptr 
   */
  void CheckReceiveParkinglot(
      const perception::ParkingLotOut::ParkType& park_type,
      const ParkingLotVertexType& parking_spot_enu,
      ParkLotStatus* park_lot_status_ptr);

  /**
   * @brief Set the Origin object
   * 
   * @param vertices parking lot vertices
   * @param origin_point 
   * @param origin_heading 
   */
  static void SetOrigin(const ParkingLotVertexType& vertices,
                        common::math::Vec2d* origin_point,
                        double* const origin_heading) {
    if (nullptr == origin_point || nullptr == origin_heading) {
      return;
    }
    *origin_point = vertices[0];
    *origin_heading = (vertices[3] - vertices[0]).Angle();
  }

  /**
   * @brief Init target pose based on parking lot verteices
   * 
   * @param frame 
   * @param park_type 
   * @param vertices parking lot vertices
   * @param is_right_side is parking lot has been update in current frame
   * @param wheel_mask 
   * @param end_pose_enu_ptr 
   * @param is_parking_inwards_ptr
   * @return true 
   * @return false 
   */
  bool InitEndPoseBaseSlot(Frame* frame,
                           const perception::ParkingLotOut::ParkType& park_type,
                           const ParkingLotVertexType& vertices,
                           bool is_right_side,
                           const std::tuple<bool, Vec2d, Vec2d>& wheel_mask,
                           common::PathPoint* end_pose_enu_ptr,
                           bool* is_parking_inwards_ptr);

  /**
   * @brief Caculate parking out target pose
   * 
   * @param frame 
   * @param park_type 
   * @param vertices parking lot vertices
   * @param kSTargetBuffer park out distance
   * @param end_pose_ptr end pose coodinate pointer
   */
  void CaculateParkingOutTarget(
      Frame* frame, const perception::ParkingLotOut::ParkType& park_type,
      const ParkingLotVertexType& vertices, double kSTargetBuffer,
      common::PathPoint* end_pose_ptr);

  /**
   * @brief Set the Parking Spot End Pose object
   * 
   * @param frame 
   * @param park_type 
   * @param vertices 
   * @param is_right_side
   * @param wheel_mask parking lot wheel mask pointer
   * @param end_pose_enu_ptr end pose coodinate pointer
   * @param is_parking_inwards_ptr
   */
  void SetParkingSpotEndPose(
      Frame* frame, const perception::ParkingLotOut::ParkType& park_type,
      const ParkingLotVertexType& vertices, bool is_right_side,
      const std::tuple<bool, Vec2d, Vec2d>& wheel_mask,
      common::PathPoint* end_pose_ptr, bool* is_parking_inwards);

  /**
   * @brief Set the Lateral Slot End Pose object
   * 
   * @param is_right_side is right side
   * @param left_top left top vertex
   * @param left_down left_down vertex
   * @param right_top right_top vertex
   * @param right_down right_down vertex
   * @param stop_left stop_left vertex
   * @param stop_right stop_right vertex
   * @param end_pose_ptr end pose coodinate pointer
   */
  void SetLateralSlotEndPose(bool is_right_side, const Vec2d& left_top,
                             const Vec2d& left_down, const Vec2d& right_top,
                             const Vec2d& right_down, const Vec2d& stop_left,
                             const Vec2d& stop_right,
                             common::PathPoint* end_pose_ptr);

  /**
   * @brief Set the Non Lateral Slot End Pose object
   * 
   * @param park_type 
   * @param left_top left top vertex
   * @param left_down left_down vertex
   * @param right_top right_top vertex
   * @param right_down right_down vertex
   * @param stop_left stop_left vertex
   * @param stop_right stop_right vertex
   * @param bool is_right_side
   * @param end_pose_ptr end pose coodinate pointer
   * @param is_parking_inwards_ptr
   */
  void SetNonLateralSlotEndPose(
      const perception::ParkingLotOut::ParkType& park_type,
      const Vec2d& left_top, const Vec2d& left_down, const Vec2d& right_top,
      const Vec2d& right_down, const Vec2d& stop_left, const Vec2d& stop_right,
      bool is_right_side, common::PathPoint* end_pose_ptr,
      bool* is_parking_inwards_ptr);

  /**
   * @brief update parking boundary based on parking lot vertices
   * 
   * @param frame 
   * @param origin_point 
   * @param origin_heading 
   * @param park_type 
   * @param vertices 
   * @param end_pose_enu 
   * @param inner_roi_boundary 
   * @param outer_roi_boundary 
   * @param roi_xy_boundary 
   */
  void GetParkingBoundary(
      Frame* frame, const Vec2d& origin_point, double origin_heading,
      const perception::ParkingLotOut::ParkType& park_type,
      const ParkingLotVertexType& vertices,
      const common::PathPoint& end_pose_enu,
      std::vector<common::math::LineSegment2d>* inner_roi_boundary,
      std::vector<common::math::LineSegment2d>* outer_roi_boundary,
      std::vector<double>* roi_xy_boundary);

  /**
   * @brief TransRoiAndSetBoundary
   * 
   * @param origin_point 
   * @param origin_heading 
   * @param inner_roi_vertex 
   * @param outer_roi_vertex 
   * @param inner_roi_boundary 
   * @param outer_roi_boundary 
   * @param roi_xy_boundary 
   */
  static void TransRoiAndSetBoundary(
      const Vec2d& origin_point, double origin_heading,
      std::vector<Vec2d>* inner_roi_vertex,
      std::vector<Vec2d>* outer_roi_vertex,
      std::vector<common::math::LineSegment2d>* inner_roi_boundary,
      std::vector<common::math::LineSegment2d>* outer_roi_boundary,
      std::vector<double>* roi_xy_boundary);

  /**
   * @brief caculate inner roi
   * 
   * @param park_type 
   * @param spot_vertices 
   * @param is_use_larger_roi 
   * @param inner_roi 
   */
  void CalculateInnerRoi(const perception::ParkingLotOut::ParkType& park_type,
                         const ParkingLotVertexType& spot_vertices,
                         bool is_use_larger_roi, std::vector<Vec2d>* inner_roi);

  /**
   * @brief use larger roi
   * 
   * @param spot_vertices 
   * @return true 
   * @return false 
   */
  bool IsUseLargerRoi(const ParkingLotVertexType& spot_vertices);

  /**
 * @brief 
 * 
 * @param frame 
 * @param adc_point
 * @param left_to_right_bottom 
 * @return double 
 */
  static double CalculateAdcHeight(
      const common::PathPoint& adc_point,
      const common::math::LineSegment2d& left_to_right_bottom);

  /**
   * @brief 
   * 
   * @param adc_point 
   * @param left_to_right_bottom 
   * @return double 
   */
  static double CalculateAdcDis2ParkBottom(
      const common::PathPoint& adc_point,
      const common::math::LineSegment2d& left_to_right_bottom);

  /**
   * @brief Get the Static Boundary object
   * 
   * @param origin_point 
   * @param origin_heading 
   * @param roi_xy_boundary 
   */
  void GetStaticBoundary(const Vec2d& origin_point, double origin_heading,
                         std::vector<double>* roi_xy_boundary);

  /**
   * @brief Get the Dynamic Boundary object
   * @param roi_box:box from cruise decision
   * @param roi_vertex
   */
  static void GetDynamicBoundary(const std::vector<double>& roi_box,
                                 std::vector<common::math::Vec2d>* roi_vertex);
  /**
   * @brief Slack roi boundary based on adc pose and target pose
   * 
   * @param end_pose_enu 
   * @param is_use_larger_roi 
   * @param inner_roi 
   */
  void RoiBoundarySlack(const common::PathPoint& end_pose_enu,
                        bool is_use_larger_roi, std::vector<Vec2d>* inner_roi);

  /**
  * @brief Caculate outer roi based on inner roi
  * 
  * @param inner_roi 
  * @param outer_roi 
  */
  static void CalculateOuterRoi(const std::vector<Vec2d>& inner_roi,
                                std::vector<Vec2d>* outer_roi);

  /**
   * @brief 
   * 
   * @param origin_point 
   * @param origin_heading 
   * @param roi_xy_boundary 
   * @return true 
   * @return false 
   */
  bool IsVehicleInRoi(const Vec2d& origin_point, double origin_heading,
                      const std::vector<double>& roi_xy_boundary);

  /**
   * @brief 
   * 
   * @param origin_trace_path 
   * @param vertices 
   * @param trace_path_ptr 
   * @return true 
   * @return false 
   */
  bool IsValidTracePath(
      const RepeatedPtrField<::TL::perception::ParkingPathPoint>&
          origin_trace_path,
      const ParkingLotVertexType& vertices, DiscretizedPath* trace_path_ptr);

  /**
   * @brief 
   * 
   * @param trace_path
   * @param chosen_trace_path
   * @return true 
   * @return false 
   */
  bool ChoosePartitionTracePath(
      const RepeatedPtrField<::TL::perception::ParkingPathPoint>& trace_path,
      DiscretizedPath* chosen_trace_path);

  /**
   * @brief 
   * 
   * @param parking_gear
   * @return soc::Chassis::GearPosition 
   */
  static soc::Chassis::GearPosition MapParkingGearToChassisGear(
      const uint32_t parking_gear) {
    soc::Chassis::GearPosition chassis_gear = soc::Chassis::GEAR_NONE;
    switch (parking_gear) {
      case 1:
        chassis_gear = soc::Chassis::GEAR_PARKING;
        break;
      case 2:
        chassis_gear = soc::Chassis::GEAR_REVERSE;
        break;
      case 3:
        chassis_gear = soc::Chassis::GEAR_NEUTRAL;
        break;
      case 4:
        chassis_gear = soc::Chassis::GEAR_DRIVE;
        break;
      default:
        chassis_gear = soc::Chassis::GEAR_NONE;
    }
    return chassis_gear;
  }

  /**
   * @brief 
   * 
   * @param obstacles_segments_vec 
   * @param trace_path_ptr 
   */
  static void CutOffTracePath(
      const std::vector<std::pair<common::math::LineSegment2d, double>>&
          obstacles_segments_vec,
      DiscretizedPath* trace_path_ptr);

  /**
   * @brief Get the Parking Replan Status object
   * 
   * @param park_type 
   * @param vertices 
   * @param pre_end_pose_enu 
   * @param end_pose_enu 
   * @return OpenSpaceStatus::Replan 
   */
  uint32_t GetParkingReplanStatus(
      const perception::ParkingLotOut::ParkType& park_type,
      const ParkingLotVertexType& vertices,
      const common::PathPoint& pre_end_pose_enu,
      const common::PathPoint& end_pose_enu);

  /**
   * @brief Get the NNS Adjust Replan Status object
   * 
   * @return OpenSpaceStatus::Replan 
   */
  uint32_t GetNNSAdjustReplanStatus();

  /**
   * @brief Get the NNS Adjust Replan Status object
   * 
   * @return OpenSpaceStatus::Replan 
   */
  bool HasNNSAdjustTraceReplan();

  /**
   * @brief Slack end pose threshold manager
   * 
   * @param park_type
   * @param vertices 
   * @param lon_threhold 
   * @param lat_threhold 
   * @param angle_threhold 
   */
  void EndPoseThresholdManager(
      const perception::ParkingLotOut::ParkType& park_type,
      const ParkingLotVertexType& vertices, double* lon_threhold,
      double* lat_threhold, double* angle_threhold);

  /**
   * @brief Is target pose diff larger than threshold
   * 
   * @param p_a 
   * @param p_b 
   * @param lon_dis_threshold 
   * @param lat_dis_threshold 
   * @param angle_thresold 
   * @return true 
   * @return false 
   */
  bool IsTargetPoseDiffLargerThanThreshold(const common::PathPoint& p_a,
                                           const common::PathPoint& p_b,
                                           double lon_dis_threshold,
                                           double lat_dis_threshold,
                                           double angle_thresold,
                                           bool is_lat_spot);

  /**
   * @brief Caculate dest region
   * 
   * @param frame 
   * @param end_pose_enu 
   * @param park_type 
   * @param vertices 
   * @param dest_region_with_angle 
   */
  void CaculateDestRegion(Frame* frame, const common::PathPoint& end_pose_enu,
                          const perception::ParkingLotOut::ParkType& park_type,
                          const ParkingLotVertexType& vertices,
                          DestRegionWithAng* dest_region_with_angle);

  /**
   * @brief calculate parking outRegion
   * 
   * @param frame 
   * @param min_lat_dis minimal lateral distance to road edge
   * @param max_lat_dis maximal lateral distance to road edge
   * @param end_pose_enu end pose in enu coordinate
   * @param vertices parking slot vertice
   * @param dest_region_with_angle 
   */
  void CalculateParkingOutRegion(Frame* frame, double min_lat_dis,
                                 double max_lat_dis,
                                 const common::PathPoint& end_pose_enu,
                                 const ParkingLotVertexType& vertices,
                                 DestRegionWithAng* dest_region_with_angle);

  /**
   * @brief Calculate parking in region
   * 
   * @param park_type 
   * @param end_pose_enu 
   * @param dest_region_with_angle 
   */
  void CalculateParkingInRegion(
      const perception::ParkingLotOut::ParkType& park_type,
      const common::PathPoint& end_pose_enu,
      DestRegionWithAng* dest_region_with_angle);

  /**
   * @brief Get the Parking Spots object
   * 
   * @param frame 
   * @param park_lot_map_ptr 
   */
  void GetParkingSpots(
      Frame* frame,
      std::unordered_map<uint32_t, ParkLotInfo>* park_lot_map_ptr);

  /**
   * @brief 
   * 
   * @param park_lot_map 
   * @return common::Status 
   */
  common::Status InputValidCheck(
      const std::unordered_map<uint32_t, ParkLotInfo>& park_lot_map);

  /**
   * @brief 
   * 
   * @param park_lot_array 
   * @param park_lot_map_ptr 
   */
  void TransParkLotsToOpenSpace(
      const std::shared_ptr<const perception::ParkingLotOutArray>&
          park_lot_array,
      std::unordered_map<uint32_t, ParkLotInfo>* park_lot_map_ptr);

  /**
   * @brief 
   * 
   * @param frame 
   * @return std::vector<uint32_t> 
   */
  std::vector<uint32_t> TopKParkingLots(
      Frame* frame,
      const std::unordered_map<uint32_t, ParkLotInfo>& park_lot_map);

  /**
   * @brief Set the Open Space Path Info object
   * 
   * @param frame 
   * @param park_lot_map 
   * @param open_space_path_info_map_ptr 
   * @return common::Status 
   */
  common::Status SetOpenSpacePathInfo(
      Frame* frame,
      const std::unordered_map<uint32_t, ParkLotInfo>& park_lot_map,
      OpenSpacePathInfoMap* open_space_path_info_map_ptr);

  /**
   * @brief 
   * 
   * @param frame 
   * @param park_lot_map 
   * @param open_space_path_info_ptr 
   * @return common::Status 
   */
  common::Status UpdateTargetPathInfo(
      Frame* frame,
      const std::unordered_map<uint32_t, ParkLotInfo>& park_lot_map,
      OpenSpacePathInfo* open_space_path_info_ptr);

  /**
   * @brief
   * 
   * @param frame 
   * @param park_lot_info 
   */
  void UpdateWheelMaskToOpenSpace(Frame* frame,
                                  const ParkLotInfo& park_lot_info);
  /**
   * @brief Set the Open Space Path Info Based On ParkLot object
   * 
   * @param frame 
   * @param park_lot_map 
   * @param open_space_path_info_map_ptr 
   * @return common::Status 
   */
  common::Status SetOpenSpacePathInfoBasedOnParklot(
      Frame* frame,
      const std::unordered_map<uint32_t, ParkLotInfo>& park_lot_map,
      OpenSpacePathInfoMap* open_space_path_info_map_ptr);

  /**
   * @brief Set the Open Space Path Info For Test Mode object
   * 
   * @param frame 
   * @param open_space_path_info_map_ptr 
   * @return common::Status 
   */
  common::Status SetOpenSpacePathInfoForTestMode(
      Frame* frame, OpenSpacePathInfoMap* open_space_path_info_map_ptr);

  /**
   * @brief Set the Open Space Path Info For NNS Adjust object
   * 
   * @param frame 
   * @return common::Status 
   */
  common::Status SetOpenSpacePathInfoForNNSAdjust(Frame* frame);
  /**
   * @brief 
   * 
   * @param is_right_side 
   * @param park_type 
   * @param parking_scenario_type_ptr 
   */
  void ParkingScenarioTypeDecision(
      bool is_right_side, const perception::ParkingLotOut::ParkType& park_type,
      ParkingScenarioType* parking_scenario_type_ptr);

  /**
   * @brief 
   * 
   * @param is_right_side 
   * @param parking_type 
   * @param park_lot_type 
   * @param origin_point 
   * @param origin_heading 
   * @param real_end_replan_pose
   * @param end_pose_enu_ptr 
   */
  void FineTuneTargetBasedOnEndReplanPose(
      bool is_right_side, const AVPStatus::ParkingType& parking_type,
      const perception::ParkingLotOut::ParkType& park_lot_type,
      const Vec2d& origin_point, double origin_heading,
      const Vec2d& real_end_replan_pose, common::PathPoint* end_pose_enu_ptr);

  /**
   * @brief 
   * 
   * @param frame 
   */
  void UpdateReplanInfo(Frame* frame);

  /**
   * @brief 
   * 
   * @param frame 
   */
  void UpdateSpeedBumpInfo(Frame* frame);

  /**
   * @brief Set the Obs Filter Strategy object
   * 
   * @param frame 
   * @param ParkLotInfo
   * @param roi_boundary 
   * @param is_slot_inner_fs_valid 
   * @param is_parking_inwards
   * @param is_narrow_spot_scenario
   * @param obs_filter_map_ptr 

   */
  void SetObsFilterStrategy(
      Frame* frame, const ParkLotInfo& park_lot_info,
      const std::vector<common::math::LineSegment2d>& roi_boundary,
      bool is_slot_inner_fs_valid, bool is_parking_inwards,
      bool is_narrow_spot_scenario, ObsFilterMap* obs_filter_map_ptr);

  /**
   * @brief 
   * 
   * @param is_parking_inwards
   * @param park_lot_type 
   * @param parking_spot_enu 
   * @param is_high_quality_triggered 
   */
  bool IsSlotInnerFsValid(
      bool is_parking_inwards,
      const perception::ParkingLotOut::ParkType& park_lot_type,
      const ParkingLotVertexType& parking_spot_enu,
      bool is_high_quality_triggered);

  /**
   * @brief 
   * 
   * @param parking_spot_enu 
   * @param is_slot_inner_fs_valid 
   * @param is_parking_inwards 
   * @param is_narrow_spot_scenario 
   * @param obs_filter
   */
  void VerticalParkInFilter(const ParkingLotVertexType& parking_spot_enu,
                            bool is_slot_inner_fs_valid,
                            bool is_parking_inwards,
                            bool is_narrow_spot_scenario,
                            ObsFilter* obs_filter);

  /**
   * @brief 
   * 
   * @param parking_spot_enu 
   * @param is_slot_inner_fs_valid 
   * @param sensor_type 
   * @param obs_filter 
   */
  static void LateralParkInFilter(
      const ParkingLotVertexType& parking_spot_enu, bool is_slot_inner_fs_valid,
      const perception::ParkingLotOut::SenType& sensor_type,
      ObsFilter* obs_filter);

  void SensorStateDecider();

  /**
   * @brief 
   * 
   * @param obs_filter 
   */
  void VerticalParkOutFilter(ObsFilter* obs_filter);

  /**
   * @brief 
   * 
   * @param obs_filter 
   */
  void VerticalParkOutLowFsFilter(ObsFilter* obs_filter);

  /**
   * @brief Get the Pre End Pose object
   * 
   * @return common::PathPoint 
   */
  common::PathPoint GetPreEndPose();

  void RecordDebugInfo(Frame* frame);

  // keep last frame things
  common::PathPoint init_adc_point_;

  bool is_park_out_along_road_ = false;
  int park_lot_loc_seq_ = 0;

  TL::common::VehicleParam vehicle_params_;
  common::VehicleState vehicle_state_;
  std::shared_ptr<DependencyInjector> injector_;
  std::shared_ptr<OpenSpaceObstacle> open_space_obstacle_ = nullptr;
  std::shared_ptr<OpenSpaceFineTuning> open_space_fine_tuning_ = nullptr;
  bool is_entered_lateral_slot_domain_ = false;
  bool is_use_larger_roi_ = false;
  bool has_valid_history_path_ = false;
  double end_pose_lat_error_ = 0.0;
  double end_pose_lon_error_ = 0.0;
  double end_pose_yaw_error_ = 0.0;
  double road_width_ = FLAGS_open_space_lane_width;
  double target_slot_witdh_ = INFINITY;
  std::vector<double> last_slack_dist_vec_;
  std::unordered_map<uint32_t, ParkLotInfo> park_lot_map_;
  SensorConfigState sensor_config_state_ = SensorConfigState::NONINITAILZED;
  AVPStatus::ParkingType parking_type_ = planning::AVPStatus::NOSTATE;
  double left_entered_depth_ = -10.0;
  double right_entered_depth_ = -10.0;
};

}  // namespace planning
}  // namespace TL
