/******************************************************************************
 * Copyright 2022 The TL Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_set>
#include <unordered_map>
#include <vector>

#include "planning/common/dependency_injector.h"
#include "planning/common/frame.h"
#include "planning/common/trajectory_stitcher.h"
#include "planning/localview/local_view.h"

#include "proto/fsm/avp_fct.pb.h"
#include "proto/fsm/function_manager.pb.h"
#include "proto/fsm/trigger_config.pb.h"
#include "proto/planning/planning.pb.h"
#include "proto/planning/trigger.pb.h"

namespace TL {
namespace planning {

class EventCollect {
 public:
  EventCollect() = default;
  ~EventCollect() = default;

  void Init(ADCTrajectory* ptr_trajectory_pb);

  /**
   * @brief event trigger collect inter
   *
   * @param localview
   * @param ptr_trajectory_pb
   */
  void ProcessEventCollect(const std::shared_ptr<LocalView>& localview,
                           ADCTrajectory* ptr_trajectory_pb);

 private:
  enum TriggerType {
    NO_EVENT = 0,
    AEB_EVENT = 2001,
    // Driver emergency braking
    DRIVER_EMERGENCY_BRAKING = 2002,
    // Driver emergency steering
    DRIVER_EMERGENCY_STEERING = 2003,
    // NNP finally requests to take over and downgrade
    ODD_DOWNDRAGE = 2004,
    // The number of parking handles exceeds the limit
    PARKING_NUM_EXCEED = 2005,
    // Multiple parking takeover
    MUTI_PARKING_TAKEOVER = 2006,
    // Parking ability class failed to take over
    PARKING_CAPABILITY_FAILURE_TAKEOVER = 2007,
    // Parking function failure request takeover
    PARKING_FUCTION_FAILURE_TAKEOVER = 2008,
    // AVP finally requests to take over and downgrade
    AVP_TAKEOVER_REQUEST_AND_DEMOTION = 2009,
    // NNP function failure request takeover
    NNP_FUCTION_FAULT = 2010,
    // N-pilot function failure request takeover
    TAKEOVER_REMIND = 2011,
    // Algorithm running active request collection
    ALGORITHM_TRIGGER_EVENT = 2012,
    //
    CLOUD_POSSIBLE_COLLISION_EVENT = 2013,
    // NNP function driver takes over actively
    NNP_DRIVER_ACTIVE_TAKEOVER = 2014,
    // Control deviation is too large, request to take over
    EXCESSIVE_DEVIATION_TAKEOVER = 2015,
    PARKING_COLLISION_EVENT = 2016,
    FCW_EVENT = 2017,
    QUICKLY_ACC = 2018,
    // Parking Unintended Error which got from control or fm
    PARKING_UNINTENDED_ERROR = 2020,
    // Parking pause over time
    PARKING_PAUSE_OVER_TIME = 2021,
    // Parking takeover request with collision risk
    PARKING_TAKEOVER_REQUEST_WITH_COLLISION_RISK = 2022,
    CONTROL_FAULT_TAKEOVER = 2023,
    NNP_COLLISION_EVENT = 2024,
    LOCAL_POSSIBLE_COLLISION_EVENT = 2025,
    NNP_ACTIVE = 2041,
    NCP_ACTIVE = 2042,
    PILOT_ACTIVE = 2043,
    ACC_ACTIVE = 2044,
    AVP_OVERSPEED = 2028,
    AVP_OVERSLOPE = 2032,
    AVP_VEHICLE_BLOCKED = 2029,
    NTP_PAUSE_OVER_TIME = 2027,
    EUROPA_MANUAL_TRIGGER = 2047
  };

  /**
   * @brief update ctrlerr trigger
   * @param localview
   */
  bool UpdateCtrlErrTrigger(const std::shared_ptr<LocalView>& localview);

  /**
   * @brief 控制跟随误差trigger id:2015
   * @param localview
   */
  bool UpdateCtrlFollowErrTrigger(const std::shared_ptr<LocalView>& localview);

  /**
   * @brief 控制板间通信异常trigger id:2039
   * @param localview
   */
  bool UpdateCtrlCommunicationErrTrigger(
      const std::shared_ptr<LocalView>& localview);

  /**
   * @brief 控制输入校验失败trigger id:2040
   * @param localview
   */
  bool UpdateCtrlInputCheckErrTrigger(
      const std::shared_ptr<LocalView>& localview);

  /**
   * @brief update nnp trigger
   * @param localview
   */
  void UpdateNnpTrigger(const std::shared_ptr<LocalView>& localview);

  /**
   * @brief 更新智驾激活状态
   * @param localview
   */
  void UpdateFunctionActiveStatus(const std::shared_ptr<LocalView>& localview);

  /**
   * @brief 紧急制动trigger id: 2002
   * @param localview
   */
  void UpdateEmergencyBrakingTrigger(
      const std::shared_ptr<LocalView>& localview);

  /**
   * @brief 紧急转向trigger id: 2003
   * @param localview
   */
  void UpdateEmergencySteeringTrigger(
      const std::shared_ptr<LocalView>& localview);
  /**
   * @brief AEB触发trigger id: 2001;FCW二级告警触发trigger id: 2017
   * @param localview
   */
  void UpdateAEBOrFCWTrigger(const std::shared_ptr<LocalView>& localview);

  /**
   * @brief odd降级触发trigger id：2004;故障导致的降级触发trigger id：2010
   * @param localview
   */
  void UpdateDowngradeTrigger(const std::shared_ptr<LocalView>& localview);

  /**
   * @brief 检测输出轨迹是否正常
   * @param localview
   */
  void UpdateOutputTrajectoryError(const std::shared_ptr<LocalView>& localview);

  /**
   * @brief 检测怀挡下拨次数
   * @param localview
   */
  void CheckGearReqCnt(const std::shared_ptr<LocalView>& localview);

  /**
   * @brief 接管提醒触发trigger id：2011
   * @param localview
   */
  void UpdateTakeOverRemindTrigger(const std::shared_ptr<LocalView>& localview);

  /**
   * @brief 算法主动请求采集的trigger id：2012
   * @param localview
   */
  void UpdateAlgorithmRequestTrigger(
      const std::shared_ptr<LocalView>& localview);

  /**
   * @brief 发生碰撞触发trigger id：2013（激活过）/2045（未激活)/2046（未激活）
   * @param localview
   */
  void UpdateCollisionTrigger(const std::shared_ptr<LocalView>& localview);

  /**
   * @brief 驾驶员接管触发trigger id：2014
   * @param localview
   */
  void UpdateTakeOverTrigger(const std::shared_ptr<LocalView>& localview);

  /**
   * @brief 控制跟踪误差大trigger id:2015 ABS/ESC/TCS故障触发trigger id：2023
   * @param localview
   */
  void UpdateControlErrTrigger(const std::shared_ptr<LocalView>& localview);

  /**
   * @brief 急加速触发trigger id: 2018
   * @param localview
   */
  void UpdateRapidAccTrigger(const std::shared_ptr<LocalView>& localview);

  /**
   * @brief 根据dssad需求更新时间锁存事件
   * @param localview
   */
  void UpdateTimeSaveEvent(const std::shared_ptr<LocalView>& localview);

  /**
   * @brief 更新横纵向接管trigger
   * @param localview
   */
  void UpdateLatAndLonOverride(const std::shared_ptr<LocalView>& localview);

  /**
   * @brief 更新变道失败trigger
   * @param localview
   */
  void UpdateLaneChangeFail(const std::shared_ptr<LocalView>& localview);

  /**
   * @brief 更新点刹trigger
   * @param localview
   */
  void UpdateBrake(const std::shared_ptr<LocalView>& localview);

  /**
   * @brief update avp trigger
   * @param localview
   */
  void UpdateAvpTrigger(const std::shared_ptr<LocalView>& localview);

  /**
   * @brief add new event trigger type
   * @param type
   */
  void AddEventTriger(uint32_t type);

  /**
   * @brief 检测nnp trigger配置参数是否有效
   * @param localview
   * @return true
   * @return false
   */
  bool ValidNnpTriggerConfig(const std::shared_ptr<LocalView>& localview);

  static const std::unordered_set<
      functionmanager::AvpFctIn::WarningInfoErrorType>
      valid_avp_warininginfo_set_;

  ADCTrajectory* ptr_trajectory_pb_ = nullptr;
  bool is_nnp_active_ = false;
  bool is_pilot_active_ = false;
  bool is_acc_active_ = false;
  bool is_function_active_ = false;
  bool is_ok_trigger_config_ = false;
  trigger::TriggerConfig trigger_config_{};
  std::vector<double> emerger_steer_speeds_{};
  std::vector<double> emerger_steer_rates_{};
  double trigger_time_ = 0.0;
  std::unordered_set<uint32_t> emergency_ids_{};
  std::unordered_map<uint32_t, double> last_emergency_trigger_{};
  double trigger_end_time_ = 0.0;
  functionmanager::AvpFctIn::WarningInfoErrorType pre_warning_info_ =
      functionmanager::AvpFctIn::NO_ERROR;
  bool has_first_trigger_ = true;
  uint32_t trigger_counter_ = 0;
};
}  // namespace planning
}  // namespace TL
