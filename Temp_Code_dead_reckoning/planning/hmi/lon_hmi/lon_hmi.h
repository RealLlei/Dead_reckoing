/******************************************************************************
 * Copyright 2022 The TL Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <sys/types.h>
#include <cstdint>
#include <list>
#include <memory>
#include <string>
#include <utility>

#include "common/math/double_type.h"
#include "map/hdmap/path.h"
#include "planning/common/frame.h"
#include "planning/common/obstacle.h"
#include "planning/common/planning_gflags.h"
#include "planning/common/reference_line_info.h"
#include "planning/hmi/lon_hmi/overtaking_assistance/overtaking_assistance.h"
#include "planning/hmi/lon_hmi/spd_adapt/spd_adapt.h"
#include "planning/warning/lbs/common/TM_Global_Types.h"
#include "planning/proto/hmi_config.pb.h"
#include "proto/common/vehicle_state.pb.h"
#include "proto/fsm/function_manager.pb.h"
#include "proto/fsm/nnp_fct.pb.h"
#include "proto/planning/planning.pb.h"
#include "proto/soc/chassis.pb.h"

namespace TL {
namespace planning {
using google::protobuf::int32;
using google::protobuf::uint32;
static constexpr int kGearCheckCnt = 10;
static constexpr int kMaxLonCtrlTimeLevel = 5;
static constexpr int kMimLonCtrlTimeLevel = 1;
static constexpr int kDefaultLonCtrlTimeLevel = 3;
static constexpr double kAccResumeTime = 0.2;
static constexpr uint32_t kObsBroadcast = 0x800;
static constexpr double kObsBroadcastTime = 0.35;
static constexpr uint32_t kObsSpdStableCnt = 3;

/**
 * @brief 智能车速匹配
 *
 */
class LonHmi {
 public:
  LonHmi() = default;
  ~LonHmi() = default;

  enum LonHmiScenario {
    ACC_NOT_STANDBY = 0,
    ACC_STANDBY = 1,
    ACC_ACTIVE = 2,
    ACC_RESUME = 3,
    ACC_ACTIVATING = 4,
    NNP_ACTIVE = 5,
    NNP_ACTIVATING = 6,
  };

  /**
   * @brief init
   *
   * @param nnp_fct_in fct in 输入信息
   * @param speed_adapt_config 
   */
  void Init(const functionmanager::HmiConfig& hmi_config);
  /**
   * @brief 更新输入信息
   *
   * @param nnp_fct_in fct in 输入信息
   * @param chassis 底盘
   */
  void UpdateInput(functionmanager::FunctionManagerIn* nnp_fct_in,
                   const functionmanager::FunctionManagerOut* fct_out,
                   const TL::common::VehicleState& vehicle_state,
                   const TL::control::McuToSocPnc& control_data);
  /**
   * @brief 更新fct 的输出信息
   *
   * @param fct_out fct out输出信息
   * @param pnc_map pnc_map 地图
   */
  void UpdateOutput(functionmanager::FunctionManagerOut* fct_out,
                    const std::shared_ptr<hdmap::PncMap>& pnc_map,
                    planning::Frame* frame,
                    functionmanager::SocToFctBus* soc_to_fct_bus);

 private:
  /**
   * @brief 
   * 
   * @param fct_in 
   * @param chassis 
   */
  LonHmiScenario SelectCurrentScenario(
      functionmanager::FunctionManagerIn* fct_in,
      const TL::soc::Chassis& chassis);
  /**
   * @brief 
   * 
   * @param fct_in 
   */
  void UpdateHmiCruiseSpeed(functionmanager::FunctionManagerOut* fct_out) const;
  /**
   * @brief 
   * 
   * @param orin_speeed_km 
   * @return uint32 
   */

  int32 CruiseSpeedLimiter(int32 orin_speeed_km);
  /**
   * @brief 
   * 
   * @param nnp_fct_in 
   */
  void UpdateAccNotStandby(functionmanager::FctToNnpInput* nnp_fct_in);
  /**
   * @brief 
   * 
   * @param nnp_fct_in 
   */
  void UpdateAccStandby(functionmanager::FctToNnpInput* nnp_fct_in);
  /**
   * @brief 
   * 
   * @param nnp_fct_in 
   */
  void UpdateResume(functionmanager::FctToNnpInput* nnp_fct_in);
  /**
   * @brief 
   * 
   * @param nnp_fct_in 
   */
  void UpdateAccActive(functionmanager::FctToNnpInput* nnp_fct_in);
  /**
   * @brief 
   * 
   * @param nnp_fct_in 
   */
  void UpdateAccActivating(functionmanager::FctToNnpInput* nnp_fct_in,
                           const TL::soc::Chassis& chassis);
  /**
   * @brief 
   * 
   * @param nnp_fct_in 
   */
  void UpdateNnpActive(functionmanager::FctToNnpInput* nnp_fct_in);
  /**
   * @brief 
   * 
   * @param nnp_fct_in 
   */
  void UpdateNnpActivating(functionmanager::FctToNnpInput* nnp_fct_in,
                           const TL::soc::Chassis& chassis);
  /**
   * @brief 
   * 
   * @param chassis 
   */
  void CheckUsrAdjustCruiseSpeed(functionmanager::FctToNnpInput* nnp_fct_in,
                                 const TL::soc::Chassis& chassis);
  /**
   * @brief 
   * 
   * @param nnp_fct_in 
   * @param chassis 
   */
  void UpdateLonCtrlTimeLevel(functionmanager::FctToNnpInput* nnp_fct_in,
                              const TL::soc::Chassis& chassis);
  /**
   * @brief 
   * 
   * @param fct_input 
   */
  void ProcessLonFollowInfo(
      TL::functionmanager::FunctionManagerIn* fct_input,
      const TL::soc::Chassis& chassis);

  /**
   * @brief Get the Acc Overtaking Assistance Start object
   * 
   * @return true 
   * @return false 
   */
  bool GetAccOvertakingAssistanceStart() const {
    return acc_overtaking_assistance_.GetStart();
  }

  /**
   * @brief 
   * 
   * @param nnp_fct_in 
   * @param chassis 
   */
  void ProcessAccStandstill(functionmanager::FunctionManagerIn* fct_in,
                            const TL::soc::Chassis& chassis);

  /**
   * @brief 
   * 
   * @param fct_in 
   */
  static void ProcessBaiduSim(functionmanager::FunctionManagerIn* fct_in);
  /**
   * @brief 
   * 
   * @param fct_out 
   * @param frame 
   */
  void ProcessStandstillHmi(functionmanager::FunctionManagerOut* fct_out,
                            planning::Frame* frame);
  /**
   * @brief 
   * 
   * @param fct_out 
   * @param stop_obs 
   * @param speed_plan_standstill 
   * @param ref_info 
   * @return true 
   * @return false 
   */
  bool ProcessPedestrian(functionmanager::FunctionManagerOut* fct_out,
                         const planning::Obstacle* stop_obs,
                         bool speed_plan_standstill,
                         const ReferenceLineInfo* ref_info);
  /**
   * @brief 
   * 
   * @param fct_out 
   * @param frame  
   * @return true 
   * @return false 
   */
  bool ProcessStandstillReusme(functionmanager::FunctionManagerOut* fct_out,
                               const planning::Frame* frame);
  /**
   * @brief 
   * 
   * @param fct_out 
   * @param frame 
   */
  static void UpdateAccActiveObs(functionmanager::FunctionManagerOut* fct_out,
                                 planning::Frame* frame);
  /**
   * @brief 
   * 
   * @param soc_to_fct_bus 
   * @param frame 
   */
  void ProcessDriveOffAlert(functionmanager::SocToFctBus* soc_to_fct_bus,
                            planning::Frame* frame);
  /**
   * @brief 
   * 
   * @param reference_line_info 
   * @return true 
   * @return false 
   */
  bool CheckObsDriveOffAlert(const ReferenceLineInfo* reference_line_info,
                             planning::Frame* frame, bool* has_targrt_obs);
  /**
   * @brief 
   * 
   * @param reference_line_info 
   * @param frame 
   * @return true 
   * @return false 
   */
  bool CheckTrafficLightDriveOffAlert(
      const ReferenceLineInfo* reference_line_info, planning::Frame* frame);
  /**
   * @brief 
   * 
   * @param soc_to_fct_bus 
   */
  void CruiseDistanceDisplayReq(
      functionmanager::SocToFctBus* soc_to_fct_bus,
      functionmanager::FunctionManagerOut* fct_out) const;
  /**
   * @brief 
   * 
   */
  void ProcessTrafficJam(const functionmanager::FunctionManagerIn* fct_in);

  /**
   * @brief 
   * 
   * @param obs_speed 
   * @param target_speed 
   * @return true 
   * @return false 
   */
  static bool ObsSpeedStableGreater(double obs_speed,
                                    double min_speed_threshold,
                                    double max_speed_threshold) {
    static uint32 cnt = 0;
    if (common::math::double_type::DefinitelyGreaterEqual(
            obs_speed, max_speed_threshold)) {
      cnt = 0;
      return true;
    }
    if (common::math::double_type::DefinitelyGreaterEqual(
            obs_speed, min_speed_threshold)) {
      cnt++;
    } else {
      cnt = 0;
    }
    return cnt > kObsSpdStableCnt;
  }

  /**
   * @brief 
   * 
   * @param fct_in 
   * @param chassis 
   */
  void CheckUsrReset(functionmanager::FunctionManagerIn* fct_in,
                     const TL::soc::Chassis& chassis);
  /**
   * @brief 
   * 
   * @param frame 
   * @param fct_out 
   */
  void CheckForceReplan(planning::Frame* frame);

  /**
   * @brief 
   * 
   * @param  
   * @return true 
   * @return false 
   */
  void CheckStopObsDisappear(planning::Frame* frame,
                             functionmanager::FunctionManagerOut* fct_out);

 private:
  functionmanager::HmiConfig hmi_config_;
  LonHmiScenario current_scenario_ = ACC_NOT_STANDBY;
  bool last_nnp_active_ = false;
  bool last_acc_active_ = false;
  int32 target_cruise_speed_ = 0;
  bool is_acc_resume_ = false;
  int32 speed_display_ = 0;
  int32 acc_mem_cruise_speed_ = 0;
  int32 acc_mem_ctrl_time_level_ = 0;
  TL::functionmanager::NnpToFctOutput last_nnp_fct_out_;
  bool usr_changed_cruise_speed_ = false;
  int32 lon_ctrl_time_level_ = 3;
  SpdAdaptScenario spd_adapt_{};
  bool is_acc_first_active_ = true;
  bool use_mem_ = false;
  int32 nnp_cruise_speed_ = 0;
  bool nnp_active_ = false;
  bool acc_active_ = false;
  int32 lon_ctrl_set_dis_from_mem_ = kDefaultLonCtrlTimeLevel;
  OvertakingAssistance acc_overtaking_assistance_;
  bool is_speed_plan_standstill_ = false;
  bool need_broadcast_ = false;
  functionmanager::FctToNnpInput::ADCS8_ACCState acc_state_ =
      functionmanager::FctToNnpInput::ACC_OFF;
  bool lon_ctrl_distance_display_req_ = false;
  // 模式
  uint8_t tgtspdctgset_ = 2;
  // 值
  int tgtspddrftset_ = 0;
  // drive off
  uint8_t drvoffalertswitch_ = 1;

  bool tgtspd_valid_ = false;

  bool reset_ = false;
  bool is_pre_stop_ = false;
  int32_t stop_obs_id_ = 0;

  // std::string last_follow_obs_;
};  // namespace TL
}  // namespace planning
}  // namespace TL
