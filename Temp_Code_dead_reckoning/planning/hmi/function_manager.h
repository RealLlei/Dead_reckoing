/******************************************************************************
 * Copyright 2022 The TL Authors. All Rights Reserved.
 *****************************************************************************/
#pragma once
#include <memory>
#include <string>

#include "planning/hmi/can_hmi.h"
#include "proto/fsm/soc_to_mcu.pb.h"

namespace TL {
namespace planning {
class FunctionManager {
 public:
  FunctionManager();
  /**
   * @brief 处理fct out
   * 
   * @param frame 
   * @param pnc_map 
   * @param injector 
   * @param trajectory_pb 
   */
  void ProcessFctOutput(Frame* frame,
                        const std::shared_ptr<hdmap::PncMap>& pnc_map,
                        const std::shared_ptr<DependencyInjector>& injector,
                        const std::shared_ptr<ADCTrajectory>& trajectory_pb);
  /**
   * @brief 预处理fct输入信息
   * 
   * @param local_view 
   */
  void ProcessFctInput(const std::shared_ptr<LocalView>& local_view);
  /**
   * @brief Set the Simulation Fct Value object 预处理 fct仿真相关信息
   * 
   * @param local_view 
   */
  void SetSimulationFctValue(const std::shared_ptr<LocalView>& local_view,
                             const functionmanager::FunctionManagerOut*);
  /**
   * @brief 
   * 
   * @param fct_in 
   * @param parkstatus planning feed back state
   */
  void UpdateAVPState(
      const functionmanager::AvpFctOut::ParkState& parkstatus,
      const std::shared_ptr<functionmanager::FunctionManagerIn>& fct_in);

 private:
  std::unique_ptr<CanNnpHmi> can_nnp_hmi_ = nullptr;
  std::unique_ptr<CanAvpHmi> can_avp_hmi_ = nullptr;

  functionmanager::TaPilotMode ta_pilot_mode_ = functionmanager::NO_CONTROL;

  functionmanager::AvpFctIn::SysRunState avp_run_state_ =
      functionmanager::AvpFctIn::STOP;
#ifdef FOR_BAIDU_SIMULATION
  functionmanager::AvpFctIn::StateType avp_sys_mode_ =
      functionmanager::AvpFctIn::NOSTTYPE;
  functionmanager::AvpFctIn::SysCmdType avp_sys_cmd_ =
      functionmanager::AvpFctIn::NOCMDTYPE;
#endif
};
}  // namespace planning
}  // namespace TL
