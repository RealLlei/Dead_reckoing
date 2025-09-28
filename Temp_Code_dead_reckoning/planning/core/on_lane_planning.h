/******************************************************************************
 * Copyright 2022 The Apollo Authors. All Rights Reserved.
 *
 *****************************************************************************/
#pragma once

#include <future>
#include <list>
#include <memory>
#include <shared_mutex>
#include <string>
#include <thread>
#include <vector>
// NOLINTBEGIN
#include "planning/common/function_statistics.h"
#include "planning/common/open_space_chart.h"
#include "planning/common/real_jerk/real_jerk.h"
#include "planning/common/reference_line_info.h"
#include "planning/common/smoothers/smoother.h"
#include "planning/core/planning_base.h"
#include "planning/hmi/function_manager.h"
#include "planning/lane_change_safety_decider/lane_change_safety_decider.h"
#include "planning/localview/local_view.h"
#include "planning/localview/state_machine/local_view_state_machine.h"
#include "planning/online_hdmap_generator/online_hdmap_generator.h"
#include "planning/planner/on_lane_planner_dispatcher.h"
#include "planning/prediction/prediction.h"
#include "planning/reference_line_info_decider/reference_line_info_decider.h"
#include "planning/self_simulator/self_simulator.h"
#include "planning/traffic_rules/rerouting.h"

// NOLINTEND
/**
 * @namespace TL::planning
 * @brief TL::planning
 */
namespace TL {
namespace planning {
using ::google::protobuf::RepeatedPtrField;
using TL::common::TrajectoryPoint;

/**
 * @class planning
 *
 * @brief Planning module main class. It processes GPS and IMU as input,
 * to generate planning info.
 */
class OnLanePlanning : public PlanningBase {
 public:
  explicit OnLanePlanning(const std::shared_ptr<DependencyInjector>& injector);
  ~OnLanePlanning() override;
  OnLanePlanning(const OnLanePlanning&) = delete;
  OnLanePlanning(OnLanePlanning&&) = delete;
  OnLanePlanning operator=(const OnLanePlanning&) = delete;
  OnLanePlanning operator=(const OnLanePlanning&&) = delete;

  /**
   * @brief Planning name.
   */
  std::string Name() const override;

  /**
   * @brief module initialization function
   * @return initialization status
   */
  common::Status Init(const PlanningConfig& config) override;
  TL::common::Status Stop() override;

  /*
   * @brief check the validation of stitching-trajectory initial point.
   *
   */
  static void CheckStitchPointValid(
      double veh_max_acce, double veh_max_dece, bool is_forward,
      RepeatedPtrField<TrajectoryPoint>* stitching_trajectory);

  /**
   * @brief main logic of the planning module, runs periodically triggered by
   * timer.
   */
  void RunOnce(const std::shared_ptr<LocalView>& local_view) override;

  common::Status Plan(
      double current_time_stamp,
      const RepeatedPtrField<TrajectoryPoint>& stitching_trajectory,
      ADCTrajectory* ptr_trajectory_pb) override;

  bool UpdateEHPData(
      const std::shared_ptr<TL::ehp::EHP>& ehp_message) override;

 private:
  static bool UpdateRefListFromDecider(
      std::list<ReferenceLineInfo>* select_ref_list,
      std::list<std::shared_ptr<ReferenceLine>>* reference_lines,
      std::list<hdmap::RouteSegments>* segments);

  common::Status InitFrame(uint32_t sequence_num,
                           const TrajectoryPoint& planning_start_point,
                           const common::VehicleState& vehicle_state);

  void CheckRerouting();
  static bool CheckPlanningConfig(const PlanningConfig& config);
  void GenerateStopTrajectory(const common::VehicleState& vehicle_state,
                              ADCTrajectory* ptr_trajectory_pb);

  // Prediction线程
  void GeneratePredictionThread();
  // Planning线程
  void GeneratePlanningThread();
  std::shared_ptr<LocalView> GetLatestPerceptionLocalView(
      const common::Header& last_header);
  Status CheckPredictionUpdated(const common::Header& last_header);
  void PostProcess(bool is_plan_run_ok, const double& planning_cycle_start_time,
                   const std::shared_ptr<ADCTrajectory>& ptr_trajectory_pb);
  /**
   * @brief Check if local_view is valid or not
   *
   * @param local_view
   * @return Status
   */
  static Status Validate(const std::shared_ptr<LocalView>& local_view);
  /**
 * @brief check perception and localization
 * 
 * @param local_view 
 * @return Status 
 */
  static Status CheckPerceptionAndLocalization(
      const std::shared_ptr<LocalView>& local_view);
  /**
 * @brief check if all inputs are valid
 * 
 * @param local_view 
 * @param status 
 * @param parking_status avp parking status
 */
  void InputValidateCheck(
      const std::shared_ptr<LocalView>& local_view, Status* status,
      functionmanager::AvpFctOut::ParkState* parking_status);
  void SetMapState(functionmanager::FunctionManagerOut* to_fct);
  void PrintPlanningTimeout(const std::shared_ptr<LocalView>& local_view);
  void SendTorqueLimitToControl(
      const std::shared_ptr<ADCTrajectory>& ptr_trajectory_pb);

  void AdjuestTrajectoryHeading(const ReferenceLineInfo* best_ref_info,
                                ADCTrajectory* ptr_trajectory_pb);

 private:
  routing::RoutingResponse last_routing_;
  std::unique_ptr<ReferenceLineProvider> reference_line_provider_;
  std::unique_ptr<common::VehicleStateProvider> vehicle_state_provider_;
  std::atomic<bool> routing_request_ready_{false};
  std::atomic<bool> routing_response_finished_{false};
  std::mutex rerouting_mutex_;
  std::shared_ptr<routing::RoutingRequest> rerouting_ptr_;
  std::unique_ptr<ReferenceLineInfoDecider> reference_line_info_decider_ =
      nullptr;
  std::unique_ptr<LaneChangeSafetyDecider> lane_change_safety_decider_ =
      nullptr;
  LocalViewStateMachine local_view_constructor_;
  OnlineHdmapGenerator online_hdmap_generator_;
  std::unique_ptr<prediction::Prediction> predictor_;
  std::shared_ptr<hdmap::PncMap> pnc_map_;
  SelfSimulator self_simulator_;
  std::unique_ptr<FunctionManager> can_hmi_nnp_avp_;
  std::thread prediction_thread_;
  std::thread planning_thread_;
  std::atomic<bool> is_prediction_thread_stop_{false};
  std::atomic<bool> is_planning_thread_stop_{false};
  uint64_t pre_fsm_sequence_num_ = 0;
  bool is_fsm_state_changed_ = false;
  bool is_pre_reference_line_ready_ = false;
  std::atomic<int> received_ehp_counter_ = -1;
  int previous_utm_zone_ = 0;
  ReferenceLineInfoConfig reference_info_config_;
  double reference_line_info_decider_use_time_ = 0.0;
  double lane_change_safety_use_time_ = 0.0;
  common::VehicleModelConfig vehicle_model_config_;
  TL::common::VehicleParam vehicle_param_;

  std::vector<std::string> debug_pnc_infor_;
  common::base::BoundedQueue<std::shared_ptr<LocalView>>
      prediction_input_queue_;
  common::base::BoundedQueue<std::shared_ptr<LocalView>> planning_input_queue_;
  uint64_t fsm_avp_sequence_num_ = 0;
  std::unique_ptr<RealJerk> real_jerk_;
  double last_planning_timeout_ = 0.0;
  double last_planning_end_timestamp_ = 0.0;
  TL::planning::lanelineprocess::DebounceModule hdmap_roadtype_debounce_;
  TL::functionmanager::TaPilotMode previous_ta_pilot_mode_ =
      TL::functionmanager::TaPilotMode::NO_CONTROL;
  ForceRplanType force_replan_type_ = NO_FORCE_REPLAN;
};

}  // namespace planning
}  // namespace TL
