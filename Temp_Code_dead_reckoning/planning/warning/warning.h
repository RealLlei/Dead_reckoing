/******************************************************************************
 * Copyright 2022 The TL Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "cta/cta.h"
#include "lbs/src/bsd/lbs_bsd_main.h"
#include "lbs/src/bsd/lbs_bsd_par.h"
#include "lbs/src/lbs_main.h"
#include "lbs/src/lbs_par.h"
#include "lbs/src/lbs_wrapper.h"
#include "lbs/src/rcw/lbs_rcw_main.h"

#include "common/status/status.h"
#include "planning/localview/local_view.h"

extern LBSInReq_st reqPorts;
extern LBSParam_st params;
extern LBSOutPro_t proPorts;
extern LBSDebug_t debugInfo;
extern CTA_Switch CTA_function_switch;
extern LBS_Switch LBS_function_switch;

// CTA interface
extern CTAInReq_t CTA_reqPorts;
extern CTAParam_t CTA_params;
extern CTAOutPro_t CTA_proPorts;
extern CTADebug_t CTA_debugInfo;
extern FCTAInReq_t FCTAreqPorts;
extern RCTAInReq_t RCTAreqPorts;

extern FCTAObjCycle_t obs_move_cycle_arr;

namespace TL {
namespace planning {
namespace warning {

using TL::soc::WarningSwitch;
using TL::soc::WarningSwitchMemory;

struct WarningStateSwitch {
  bool LCA_state = true;
  bool OSE_state = true;
  bool RCW_state = true;
  bool FCTA_state = true;
  bool RCTA_state = true;
};

struct ObsExtendedInfo {
  int life_cycle;
  int index_in_proto;
  int id;
};

class Warning {
 public:
  /**
   * @brief module name
   */
  static std::string Name() { return "Warning"; }

  /**
   * @brief module initialization function
   * @return initialization status
   */
  TL::common::Status Init();
  /**
   * @brief module start function
   * @return start status
   */
  TL::common::Status Start();

  /**
   * @brief module stop function
   */
  void Stop();

  /**
   * @brief main logic of the warning module, runs periodically
   * triggered by timer.
   */
  bool Process(const soc::WarningFault& warning_fault,
               const std::shared_ptr<LocalView>& local_view,
               WarningOutput* warning_output);

 private:
  /**
   * @brief Initialize LBS input signals
   *
   * @return TL::common::Status if the initialization is done normally
   */
  TL::common::Status InitLBSInput();
  /**
   * @brief Initialize CTA input signals
   *
   * @return TL::common::Status if the initialization is done normally
   */
  TL::common::Status InitCTAInput();
  /**
   * @brief No use data, give it constant
   *
   */
  void SigHeaderInfo();
  /**
   * @brief Transform the coordinate system
   *
   * @param origin_x x of the origin point in the old coordinate system
   * @param origin_y y of the origin point in the old coordinate system
   * @param origin_thea heading of the x axis of the origin point of the new
   * coordinate system in the old coordinate system
   * @param transform_before_x x of the point to transform; in the old coordinate
   * system
   * @param transform_before_y y of the point to transform; in the old coordinate
   * system
   * @param transform_after_x x of the point after transform; in the new
   * coordinate system
   * @param transform_after_y y of the point after transform; in the new
   * coordinate system
   */
  void TransformCoordinateSystem(double origin_x, double origin_y,
                                 double origin_thea, double transform_before_x,
                                 double transform_before_y,
                                 double* transform_after_x,
                                 double* transform_after_y);
  /**
   * @brief Calculate the relative speed of the obstacle to ego vehicle. All the
   * parameters are in the same coordinate system
   *
   * @param x_obs x of the obstacle
   * @param y_obs y of the obstacle
   * @param vx_obs_abs x axis projection of the obstacle speed
   * @param vy_obs_abs y axis projection of the obstacle speed
   * @param x_ego x of ego vehicle
   * @param y_ego y of ego vehicle
   * @param vx_ego x axis projection of ego vehicle speed
   * @param vy_ego y axis projection of ego vehicle speed
   * @param w ego vehicle angular speed
   * @param vx_obs_rel x axis projection of the relative speed of the obstacle
   * to ego vehicle
   * @param vy_obs_rel y axis projection of the relative speed of the obstacle
   * to ego vehicle
   */
  void CalculateRelativeSpeed(double x_obs, double y_obs, double vx_obs_abs,
                              double vy_obs_abs, double x_ego, double y_ego,
                              double vx_ego, double vy_ego, double w,
                              double* vx_obs_rel, double* vy_obs_rel);
  /**
   * @brief Calculate the relative acceleration of the obstacle to ego vehicle.
   * All the parameters are in the same coordinate system
   *
   * @param ax_obs_abs x axis projection of the obstacle acceleration
   * @param ay_obs_abs y axis projection of the obstacle acceleration
   * @param ax_ego x axis projection of ego vehicle acceleration
   * @param ay_ego y axis projection of ego vehicle acceleration
   * @param vx_obs_rel x axis projection of the relative speed of the obstacle
   * to ego vehicle
   * @param vy_obs_rel y axis projection of the relative speed of the obstacle
   * to ego vehicle
   * @param w ego vehicle angular speed
   * @param ax_obs_rel x axis projection of the relative acceleration of the
   * obstacle to ego vehicle
   * @param ay_obs_rel y axis projection of the relative acceleration of the
   * obstacle to ego vehicle
   */
  void RelativeAcceleration(double ax_obs_abs, double ay_obs_abs, double ax_ego,
                            double ay_ego, double vx_obs_rel, double vy_obs_rel,
                            double w, double* ax_obs_rel, double* ay_obs_rel);
  /**
   * @brief process the switch status of the warning functions, which indicates
   * if the warning functions are active
   *
   * @param warning_fault the fault type of the warning functions
   * @param localview the input of planning
   */
  void ProcessWarningSwitch(const soc::WarningFault& warning_fault,
                            const std::shared_ptr<LocalView>& localview);
  /**
   * @brief sort the obstacles according to the distance to ego vehicle;
   * calculate the lifecycle of the obstacles
   *
   * @param perception_obstacles the obstacles
   * @param localization the input from localization
   * @return bool if the calculation is success
   */
  bool CalculateExtendedObsInfo(
      const std::shared_ptr<const TL::perception::PerceptionObstacles>&
          perception_obstacles,
      const std::shared_ptr<const localization::Localization>& localization);
  /**
   * @brief convert the obstacle type information to the code in the C program
   *
   * @param obs_index obstacle ID
   * @param fusion_obj_maintence_type obstacle maintainence type in fusion
   * protocol
   * @return uint8_t obstacle type code
   */
  uint8_t ConvertObjMaintenceType(uint32_t obs_index,
                                  uint8 fusion_obj_maintence_type);
  /**
   * @brief 根据mcu的按键记忆信息和座舱的按键信息更新warning的按键信息
   * 
   * @param status_from_cdcs 座舱信息
   * @param status_from_mem mcu记忆信息
   * @param last_status 上一帧的按键信息
   * @param is_mem_update 按键记忆值是否更新
   * @return WarningSwitch::Status 最终使用的按键信息
   */
  WarningSwitch::Status UpdateWarningSwitch(
      const WarningSwitch::Status& status_from_cdcs,
      const WarningSwitchMemory::Status& status_from_mem, bool last_status,
      bool is_mem_update);

  WarningStateSwitch warning_switch_;
  WarningStateSwitch last_warning_switch_;
  WarningSwitchMemory last_warning_switch_mem_;
  std::unordered_map<uint32, uint32> obs_first_history_sequence_;
  std::vector<ObsExtendedInfo> obs_extended_info_;
};

}  // namespace warning
}  // namespace planning
}  // namespace TL
