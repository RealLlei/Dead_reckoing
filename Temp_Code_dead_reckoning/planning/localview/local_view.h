//  Copyright (c) TL Technologies Co., Ltd. 2019-2022. All rights reserved.
#pragma once

#include <cstdint>
#include <list>
#include <memory>
#include <vector>

#include "common/file/log.h"
#include "common/memory/arena_adapter.h"
#include "common/time/lock/atomic_rw_lock.h"
#include "common/time/lock/rw_lock_guard.h"
#include "map/hdmap/hdmap.h"
#include "proto/common/types.pb.h"
#include "proto/common/vehicle_state.pb.h"
#include "proto/control/adas_someip.pb.h"
#include "proto/control/mbd_control_debug.pb.h"
#include "proto/control/mcu_to_soc.pb.h"
#include "proto/control/mcu_to_soc_pnc.pb.h"
#include "proto/fsm/function_manager.pb.h"
#include "proto/fsm/soc_to_mcu.pb.h"
#include "proto/hmi/nns_location.pb.h"
#include "proto/hmi/nns_router.pb.h"
#include "proto/local_mapping/local_map.pb.h"
#include "proto/localization/localization.pb.h"
#include "proto/map/map.pb.h"
#include "proto/map/navigation.pb.h"
#include "proto/perception/perception_freespace.pb.h"
#include "proto/perception/perception_obstacle.pb.h"
#include "proto/perception/perception_parking_lot.pb.h"
#include "proto/perception/transport_element.pb.h"
#include "proto/planning/lanemarkers_lane_line.pb.h"
#include "proto/planning/pad_msg.pb.h"
#include "proto/planning/planning.pb.h"
#include "proto/planning/warning.pb.h"
#include "proto/planning/without_lane_follow.pb.h"
#include "proto/prediction/prediction_obstacle.pb.h"
#include "proto/routing/routing.pb.h"
#include "proto/soc/chassis.pb.h"

namespace TL {
namespace planning {

/**
 * @struct local_view
 * @brief LocalView contains all necessary data as planning input
 */
// NOLINTBEGIN
using AtomicRWLock = ::TL::common::base::AtomicRWLock;
using AtomicWriteLockGuard =
    ::TL::common::base::WriteLockGuard<AtomicRWLock>;
using AtomicReadLockGuard = ::TL::common::base::ReadLockGuard<AtomicRWLock>;

#define COPYSETUP(NAMESPACE, MSG_TYPE, MEB_VAL)                      \
  if (lv.Has##MSG_TYPE()) {                                          \
    Set##MSG_TYPE##Ptr(                                              \
        std::make_shared<NAMESPACE::MSG_TYPE>(*lv.Get##MSG_TYPE())); \
  }

#define LOCALVIEWSETUP(NAMESPACE, MSG_TYPE, MORE, MEB_VAL)                 \
 private:                                                                  \
  std::shared_ptr<NAMESPACE::MSG_TYPE> MEB_VAL = nullptr;                  \
  mutable ::TL::common::base::AtomicRWLock MEB_VAL##rwlock_;            \
                                                                           \
 public:                                                                   \
  inline std::shared_ptr<const NAMESPACE::MSG_TYPE> Get##MSG_TYPE##MORE()  \
      const {                                                              \
    AtomicReadLockGuard lg(MEB_VAL##rwlock_);                              \
    ACHECK(MEB_VAL);                                                       \
    return MEB_VAL;                                                        \
  }                                                                        \
  inline void Set##MSG_TYPE##MORE##Ptr(                                    \
      const std::shared_ptr<NAMESPACE::MSG_TYPE>& MEB_VAL##ptr) {          \
    AtomicWriteLockGuard lg(MEB_VAL##rwlock_);                             \
    MEB_VAL = MEB_VAL##ptr;                                                \
  }                                                                        \
  inline bool HasValid##MSG_TYPE##MORE##Header() const {                   \
    AtomicReadLockGuard lg(MEB_VAL##rwlock_);                              \
    if (MEB_VAL == nullptr) {                                              \
      return false;                                                        \
    } else if (!MEB_VAL->has_header() || MEB_VAL->header().has_status()) { \
      return false;                                                        \
    } else {                                                               \
      return true;                                                         \
    }                                                                      \
  }                                                                        \
  inline bool Has##MSG_TYPE##MORE() const {                                \
    AtomicReadLockGuard lg(MEB_VAL##rwlock_);                              \
    return MEB_VAL != nullptr;                                             \
  }

class LocalView {
 public:
  LocalView() = default;
  virtual ~LocalView() = default;

 private:
  std::shared_ptr<google::protobuf::Arena> arena_ = nullptr;
  mutable ::TL::common::base::AtomicRWLock arena_rwlock_;
  std::shared_ptr<hdmap::HDMap> hd_map_ = nullptr;
  mutable ::TL::common::base::AtomicRWLock hdmap_rwlock_;
  std::shared_ptr<common::Pose> slam_map_pose_ = nullptr;
  mutable ::TL::common::base::AtomicRWLock slam_map_pose_rwlock_;
  bool is_guard_triggered_ = false;
  mutable ::TL::common::base::AtomicRWLock guard_trigger_flag_rwlock_;

  std::vector<int32_t> cruise_target_id_{-1, -1};
  mutable ::TL::common::base::AtomicRWLock cruise_target_id_rwlock_;
  double subject_kappa_ = 0;
  mutable ::TL::common::base::AtomicRWLock subject_kappa_rwlock_;

 public:
  void SetArenaPtr(const std::shared_ptr<google::protobuf::Arena>& arena) {
    AtomicWriteLockGuard lg(arena_rwlock_);
    arena_ = arena;
  }

  bool HasArena() const {
    AtomicWriteLockGuard lg(arena_rwlock_);
    return arena_ != nullptr;
  }

  const std::shared_ptr<google::protobuf::Arena>& GetArena() {
    AtomicReadLockGuard lg(arena_rwlock_);
    return arena_;
  }

  void SetSlamMapPosePtr(const std::shared_ptr<common::Pose>& slam_map_pose) {
    AtomicWriteLockGuard lg(slam_map_pose_rwlock_);
    slam_map_pose_ = slam_map_pose;
  }

  bool HasSlamMapPose() const {
    AtomicReadLockGuard lg(slam_map_pose_rwlock_);
    return slam_map_pose_ != nullptr && slam_map_pose_->has_position() &&
           slam_map_pose_->has_heading();
  }

  const std::shared_ptr<common::Pose>& GetSlamMapPosePtr() {
    ACHECK(slam_map_pose_);
    AtomicReadLockGuard lg(slam_map_pose_rwlock_);
    return slam_map_pose_;
  }

  void SetHDMapPtr(const std::shared_ptr<hdmap::HDMap>& hd_map) {
    AtomicWriteLockGuard lg(hdmap_rwlock_);
    hd_map_ = hd_map;
  }

  bool HasHDMap() const {
    AtomicReadLockGuard lg(hdmap_rwlock_);
    return hd_map_ != nullptr;
  }

  const std::shared_ptr<hdmap::HDMap>& GetHDMapPtr() {
    ACHECK(hd_map_);
    AtomicReadLockGuard lg(hdmap_rwlock_);
    return hd_map_;
  }

  void SetCruiseTargetId(const std::vector<int32_t>& cruise_target_id) {
    AtomicWriteLockGuard lg(cruise_target_id_rwlock_);
    cruise_target_id_ = cruise_target_id;
  }

  const std::vector<int32_t>& GetCruiseTargetId() const {
    AtomicWriteLockGuard lg(cruise_target_id_rwlock_);
    return cruise_target_id_;
  }

  void SetSubjectKappa(const double& subject_kappa) {
    AtomicWriteLockGuard lg(subject_kappa_rwlock_);
    subject_kappa_ = subject_kappa;
  }

  const double& GetSubjectKappa() const {
    AtomicWriteLockGuard lg(subject_kappa_rwlock_);
    return subject_kappa_;
  }

  void SetGuardTriggeredFlag(const bool is_guard_triggered) {
    AtomicWriteLockGuard lg(guard_trigger_flag_rwlock_);
    is_guard_triggered_ = is_guard_triggered;
  }

  bool GetGuardTriggeredFlag() const {
    AtomicWriteLockGuard lg(guard_trigger_flag_rwlock_);
    return is_guard_triggered_;
  }

  LOCALVIEWSETUP(control, McuToSocPnc, , mcu_to_soc_pnc_);
  LOCALVIEWSETUP(prediction, PredictionObstacles, , prediction_obstacles_);
  LOCALVIEWSETUP(perception, PerceptionObstacles, , perception_obstacles_);
  LOCALVIEWSETUP(perception, PerceptionObstacles, Minieye,
                 perception_obstacles_minieye_);
  LOCALVIEWSETUP(mapping, LocalMap, , perception_local_map_);
  LOCALVIEWSETUP(soc, Chassis, , chassis_);
  LOCALVIEWSETUP(localization, Localization, , localization_estimate_);
  LOCALVIEWSETUP(perception, TransportElement, , transport_element_);
  LOCALVIEWSETUP(perception, TrafficLightDetection, , traffic_light_detection_);
  LOCALVIEWSETUP(routing, RoutingResponse, , routing_response_);
  LOCALVIEWSETUP(routing, RoutingRequest, , routing_request_);
  LOCALVIEWSETUP(planning, PadMessage, , pad_message_);
  LOCALVIEWSETUP(perception, LaneMarkers, , lane_markers_);
  LOCALVIEWSETUP(perception, LaneMarkers, Minieye, lane_markers_minieye_);
  LOCALVIEWSETUP(navigation_hdmap, MapMsg, , map_msg_);
  LOCALVIEWSETUP(navigation_hdmap, MapMsg, MapFusion, map_fusion_);
  LOCALVIEWSETUP(common, VehicleState, , vehicle_state_);
  LOCALVIEWSETUP(planning, WarningOutput, , warning_output_);
  LOCALVIEWSETUP(perception, ParkingLotOutArray, , parking_lot_out_array_);
  LOCALVIEWSETUP(perception, FreeSpaceOutArray, , free_space_out_array_);
  LOCALVIEWSETUP(planning, ADCTrajectory, , adc_trajectory_);
  LOCALVIEWSETUP(planning, ADCTrajectory, Guard, guard_adc_trajectory_);
  LOCALVIEWSETUP(functionmanager, FunctionManagerOut, , function_manager_out_);
  LOCALVIEWSETUP(functionmanager, FunctionManagerIn, , function_manager_in_);
  LOCALVIEWSETUP(control, MbdDebugFromMCU, , mbd_debug_);
  LOCALVIEWSETUP(common, mcu_to_soc_DebugData, , adas_data_);
  LOCALVIEWSETUP(planning, LanemarkersLaneLine, , lanemarkers_laneline_);
  LOCALVIEWSETUP(planning, WithoutLaneFollow, , without_lane_follow_);
  LOCALVIEWSETUP(hdmap, MapStateData, , map_state_data_);
  LOCALVIEWSETUP(hmi, NNSLocFrame, , nns_location_);
  LOCALVIEWSETUP(hmi, NNSRouteInfo, , nns_route_);
  LOCALVIEWSETUP(control, AdasSomeipFromMCU, , adas_7k_data_);

  void CopyFrom(const LocalView& lv) {
    COPYSETUP(prediction, PredictionObstacles, prediction_obstacles_);
    COPYSETUP(control, McuToSocPnc, mcu_to_soc_pnc_);
    COPYSETUP(perception, PerceptionObstacles, perception_obstacles_);
    COPYSETUP(soc, Chassis, chassis_);
    COPYSETUP(localization, Localization, localization_estimate_);
    COPYSETUP(perception, TransportElement, transport_element_);
    COPYSETUP(perception, TrafficLightDetection, traffic_light_detection_);
    COPYSETUP(routing, RoutingResponse, routing_response_);
    COPYSETUP(routing, RoutingRequest, routing_request_);
    COPYSETUP(planning, PadMessage, pad_message_);
    COPYSETUP(perception, LaneMarkers, lane_markers_);
    COPYSETUP(navigation_hdmap, MapMsg, map_msg_);
    COPYSETUP(common, VehicleState, vehicle_state_);
    COPYSETUP(planning, WarningOutput, warning_output_);
    COPYSETUP(perception, ParkingLotOutArray, parking_lot_out_array_);
    COPYSETUP(perception, FreeSpaceOutArray, free_space_out_array_);
    COPYSETUP(planning, ADCTrajectory, adc_trajectory_);
    COPYSETUP(functionmanager, FunctionManagerOut, function_manager_out_);
    COPYSETUP(functionmanager, FunctionManagerIn, function_manager_in_);
    COPYSETUP(control, MbdDebugFromMCU, mbd_debug_);
    COPYSETUP(common, mcu_to_soc_DebugData, adas_data_);
    COPYSETUP(planning, LanemarkersLaneLine, lanemarkers_laneline_);
    COPYSETUP(hdmap, MapStateData, map_state_data_);
  }

  // NOLINTEND
};

}  // namespace planning
}  // namespace TL
