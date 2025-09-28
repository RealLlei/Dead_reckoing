/*
 * Copyright (c) TL Technologies Co., Ltd. 2022. All rights reserved.
 * Description:  safety_guard_planning.h
 */

#pragma once

#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "common/time/clock.h"
#include "common/vehicle_state/vehicle_state_provider.h"
#include "planning/core/planning_base.h"

#include "proto/fsm/avp_fct.pb.h"

namespace TL {
namespace planning {

/**
 * @brief Safety Guard Planning module
 *
 */
class SafetyGuardPlanning final : public PlanningBase {
 public:
  enum StageType {
    UNKNOWN = 0,
    BRAKING = 1,
    WAIT_REPLAN = 2,
    WAIT_OBSTACLE = 3,
    RUNNING = 4,
  };

  /**
   * @brief Construct a new Safety Guard Planning object
   *
   * @param injector DependencyInjector
   */
  explicit SafetyGuardPlanning(
      const std::shared_ptr<DependencyInjector>& injector);

  /**
   * @brief Destroy the Safety Guard Planning object
   *
   */
  ~SafetyGuardPlanning() override;

  SafetyGuardPlanning(const SafetyGuardPlanning&) = delete;
  SafetyGuardPlanning(SafetyGuardPlanning&&) = delete;
  SafetyGuardPlanning operator=(const SafetyGuardPlanning&) = delete;
  SafetyGuardPlanning operator=(const SafetyGuardPlanning&&) = delete;

  /**
   * @brief override Name()
   *
   * @return std::string module name
   */
  std::string Name() const override;

  /**
   * @brief override Init()
   *
   * @param config PlanningConfig
   * @return common::Status errorcode and msg
   */
  common::Status Init(const PlanningConfig& config) override;

  /**
   * @brief override Stop()
   *
   * @return TL::common::Status errorcode and msg
   */
  TL::common::Status Stop() override;

  /**
   * @brief override RunOnce()
   *
   * @param local_view input data
   */
  void RunOnce(const std::shared_ptr<LocalView>& local_view) override;

  /**
   * @brief override Plan()
   *
   * @param current_time_stamp current time stamp
   * @param stitching_trajectory stitch trajectory
   * @param trajectory output trajectory
   * @return common::Status errorcode and msg
   */
  common::Status Plan(const double /*current_time_stamp*/,
                      const ::google::protobuf::RepeatedPtrField<
                          common::TrajectoryPoint>& /*stitching_trajectory*/,
                      ADCTrajectory* const /*trajectory*/) override {
    return common::Status::OK();
  }

  /**
   * @brief override UpdateEHPData()
   *
   * @param ehp_message ehp message
   * @return true update succeed
   * @return false update failed
   */
  bool UpdateEHPData(
      const std::shared_ptr<TL::ehp::EHP>& /*ehp_message*/) override {
    return true;
  }

 private:
  /**
   * @brief init parameter
   *
   */
  void InitParameter();

  /**
   * @brief check planning config
   *
   * @param config PlanningConfig
   * @return true check succeed
   * @return false check failed
   */
  bool CheckPlanningConfig(const PlanningConfig& config);

  /**
   * @brief generate stop trajectory
   *
   * @param vehicle_state vehicle
   * @param ADCTrajectory* const adc_trajectory_pb
   */
  void GenerateStopTrajectory(const common::VehicleState& vehicle_state,
                              ADCTrajectory* adc_trajectory_pb);

  /**
   * @brief update state machine
   *
   * @param is_real_time_triggered real time triggered flag
   * @param safety_guard_info_ptr safety guard info ptr
   * @param adc_trajectory_pb_ptr adc trajectory pb ptr
   */
  void UpdateStateMachine(
      bool is_real_time_triggered,
      planning_internal::SafetyGuardInfo* safety_guard_info_ptr,
      planning::ADCTrajectory* adc_trajectory_pb_ptr);

  /**
   * @brief main process
   *
   * @param local_view input data
   */
  void Process(const std::shared_ptr<LocalView>& local_view);

  std::unique_ptr<common::VehicleStateProvider> vehicle_state_provider_;
  common::VehicleState cur_vehicle_state_;

  std::atomic<bool> is_safety_guard_triggered_{false};
  functionmanager::AvpFctIn::SysRunState last_sys_run_state_ =
      functionmanager::AvpFctIn::STOP;
  planning_internal::SafetyGuardInfo::TriggeredType triggered_hold_type_ =
      planning_internal::SafetyGuardInfo::NONE;
  uint32_t existence_frame_num_ = 0;
  std::queue<bool> triggered_state_queue_;
  StageType cur_stage_ = StageType::UNKNOWN;
  soc::Chassis::GearPosition triggered_gear_ = soc::Chassis::GEAR_NEUTRAL;
  double triggered_start_time_ = common::Clock::NowInSeconds();
};

}  // namespace planning
}  // namespace TL
