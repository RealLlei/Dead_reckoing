/*
 * Copyright (c) TL Technologies Co., Ltd. 2022. All rights reserved.
 * Description:  safety_guard_planning.cc
 */

#include "planning/core/safety_guard_planning.h"

#include <algorithm>
#include <limits>
#include <list>
#include <memory>

#include "common/file/log.h"
#include "common/util/message_util.h"
#include "glog/logging.h"
#include "planning/common/trajectory_stitcher.h"
#include "planning/planner/safety_guard_planner_dispatcher.h"
#include "proto/fsm/avp_fct.pb.h"

namespace TL {
namespace planning {

using TL::common::Clock;
using TL::common::ErrorCode;
using TL::common::Status;
using TL::common::TrajectoryPoint;  // NOLINT
using TL::common::VehicleState;
using TL::common::VehicleStateProvider;
using TL::common::math::Vec2d;  // NOLINT
using TL::soc::Chassis;         // NOLINT

SafetyGuardPlanning::SafetyGuardPlanning(
    const std::shared_ptr<DependencyInjector>& injector)
    : PlanningBase(injector) {
  planner_dispatcher_ = std::make_unique<SafetyGuardPlannerDispatcher>();
}

SafetyGuardPlanning::~SafetyGuardPlanning() {
  Stop();
  planner_->Stop();
}

Status SafetyGuardPlanning::Stop() {
  return Status(ErrorCode::OK);
}

std::string SafetyGuardPlanning::Name() const {
  return "safety_guard_planning";
}

Status SafetyGuardPlanning::Init(const PlanningConfig& config) {
  config_ = config;
  if (!CheckPlanningConfig(config_)) {
    return Status(ErrorCode::CORE_SAFETYGUARD_CONFIGINIT_ERROR,
                  "planning config error: " + config_.DebugString());
  }

  planner_dispatcher_->Init();
  vehicle_state_provider_ = std::make_unique<common::VehicleStateProvider>();
  planner_ = planner_dispatcher_->DispatchPlanner(config_, injector_);
  if (!planner_) {
    return Status(
        ErrorCode::CORE_SAFETYGUARD_CREATEINIT_ERROR,
        "planning is not initialized with config : " + config_.DebugString());
  }

  return planner_->Init(config_);
}

void SafetyGuardPlanning::GenerateStopTrajectory(
    const VehicleState& vehicle_state, ADCTrajectory* const adc_trajectory_pb) {
  DCHECK(nullptr != adc_trajectory_pb);
  common::util::FillHeader("planning", adc_trajectory_pb);
  adc_trajectory_pb->set_gear(vehicle_state.gear() == Chassis::GEAR_NEUTRAL
                                  ? Chassis::GEAR_PARKING
                                  : vehicle_state.gear());
  const double acc =
      vehicle_state.gear() == soc::Chassis::GEAR_DRIVE
          ? config_.safety_guard_planning_config().max_deceleration()
          : -config_.safety_guard_planning_config().max_deceleration();

  DiscretizedTrajectory trajectory;
  trajectory.SetStopTrajectory(vehicle_state.x(), vehicle_state.y(),
                               vehicle_state.heading(), vehicle_state.kappa(),
                               0.0, acc);
  PublishableTrajectory stop_trajectory(
      adc_trajectory_pb->header().data_stamp(), trajectory);
  stop_trajectory.PopulateTrajectoryProtobuf(adc_trajectory_pb);

  adc_trajectory_pb->set_driving_mode(0);
  adc_trajectory_pb->set_function_mode(10);
}

void SafetyGuardPlanning::RunOnce(
    const std::shared_ptr<LocalView>& local_view) {
  if (!config_.safety_guard_planning_config().enable_safety_guard()) {
    AINFO << "safety_guard is disabled.";
    return;
  }
  if (!local_view->HasFunctionManagerIn() ||
      !local_view->HasValidLocalizationHeader() ||
      !local_view->HasValidChassisHeader()) {
    AINFO << "has_fctin: " << local_view->HasFunctionManagerIn()
          << ", has_loc: " << local_view->HasValidLocalizationHeader()
          << ", has_chassis: " << local_view->HasValidChassisHeader();
    InitParameter();
    return;
  }
  if (!local_view->GetChassis()->has_gear_location() ||
      (local_view->GetChassis()->gear_location() != soc::Chassis::GEAR_DRIVE &&
       local_view->GetChassis()->gear_location() !=
           soc::Chassis::GEAR_REVERSE)) {
    AINFO << "Gear is not GEAR_DRIVE or GEAR_REVERSE.";
    InitParameter();
    return;
  }
  const auto& fct_in = local_view->GetFunctionManagerIn();
  if (fct_in->driver_mode() != functionmanager::DriveMode::AVP_LAT_LGT_ACTIVE ||
      (fct_in->fct_avp_in().sys_mode() == functionmanager::AvpFctIn::RPA &&
       fct_in->fct_avp_in().sys_run_state() ==
           functionmanager::AvpFctIn::PARKSTART)) {
    ADEBUG << "driver_mode: " << DriveMode_Name(fct_in->driver_mode())
           << ", sys_mode: "
           << functionmanager::AvpFctIn::StateType_Name(
                  fct_in->fct_avp_in().sys_mode());
    InitParameter();
    return;
  }
  if (is_safety_guard_triggered_) {
    const bool hmi_disabled =
        (last_sys_run_state_ == functionmanager::AvpFctIn::PAUSE &&
         fct_in->fct_avp_in().sys_run_state() !=
             functionmanager::AvpFctIn::PAUSE) ||
        fct_in->fct_avp_in().sys_run_state() == functionmanager::AvpFctIn::STOP;
    const bool plan_disabled =
        (triggered_gear_ == soc::Chassis::GEAR_DRIVE &&
         local_view->GetChassis()->gear_location() ==
             soc::Chassis::GEAR_REVERSE) ||
        (triggered_gear_ == soc::Chassis::GEAR_REVERSE &&
         local_view->GetChassis()->gear_location() == soc::Chassis::GEAR_DRIVE);
    if (hmi_disabled || plan_disabled) {
      AINFO << "hmi_disabled: " << hmi_disabled
            << ", plan_disabled: " << plan_disabled;
      InitParameter();
    }
  }
  last_sys_run_state_ =
      local_view->GetFunctionManagerIn()->fct_avp_in().sys_run_state();

  if (vehicle_state_provider_->Update(*local_view->GetLocalization(),
                                      *local_view->GetChassis()) !=
      Status::OK()) {
    AINFO << "vehicle_state_provider update failed,  localization: "
          << local_view->GetLocalization()->DebugString()
          << ", chassis: " << local_view->GetChassis()->DebugString();
    InitParameter();
    return;
  }
  cur_vehicle_state_ = vehicle_state_provider_->vehicle_state();
  auto vehicle_state = std::make_shared<common::VehicleState>();
  vehicle_state->CopyFrom(vehicle_state_provider_->vehicle_state());
  local_view->SetVehicleStatePtr(vehicle_state);
  Process(local_view);
}

void SafetyGuardPlanning::Process(
    const std::shared_ptr<LocalView>& local_view) {
  local_view_ = local_view;
  ADCTrajectory adc_trajectory_pb;
  start_time_ = Clock::NowInSeconds();
  const auto planning_init_point =
      TrajectoryStitcher::ComputeTrajectoryPointFromVehicleState(
          0, *local_view->GetVehicleState());
  const auto frame_num = static_cast<uint32_t>(seq_num_++);
  frame_ = std::make_unique<Frame>(frame_num, local_view, planning_init_point,
                                   *local_view->GetVehicleState());
  planner_->Plan(planning_init_point, frame_.get(), &adc_trajectory_pb);

  auto* safety_guard_info_ptr =
      adc_trajectory_pb.mutable_debug()->mutable_safety_guard_info();
  const bool is_real_time_triggered =
      safety_guard_info_ptr->is_real_time_triggered_uss() ||
      safety_guard_info_ptr->is_real_time_triggered_free_space();
  FillPlanningPb(&adc_trajectory_pb);
  safety_guard_info_ptr->set_frame_num(frame_num);
  const double diff = (Clock::NowInSeconds() - start_time_) * 1000;
  safety_guard_info_ptr->set_total_time_ms(diff);
  UpdateStateMachine(is_real_time_triggered, safety_guard_info_ptr,
                     &adc_trajectory_pb);
  ADEBUG << "safety_guard_info_ptr: " << safety_guard_info_ptr->DebugString()
         << "fct_out parking_status: "
         << functionmanager::AvpFctOut_ParkState_Name(
                adc_trajectory_pb.function_manager_out()
                    .avp_fct_out()
                    .parking_status());
  if (is_safety_guard_triggered_) {
    GenerateStopTrajectory(cur_vehicle_state_, &adc_trajectory_pb);
    local_view->SetADCTrajectoryGuardPtr(
        std::make_shared<planning::ADCTrajectory>(adc_trajectory_pb));
    publish_queue_->ForceEnqueue(local_view);
  }
}

void SafetyGuardPlanning::InitParameter() {
  is_safety_guard_triggered_ = false;
  last_sys_run_state_ = functionmanager::AvpFctIn::STOP;
  triggered_hold_type_ = planning_internal::SafetyGuardInfo::NONE;
  existence_frame_num_ = 0;
  while (!triggered_state_queue_.empty()) {
    triggered_state_queue_.pop();
  }
  cur_stage_ = StageType::UNKNOWN;
  triggered_gear_ = soc::Chassis::GEAR_NEUTRAL;
  triggered_start_time_ = Clock::NowInSeconds();
}

void SafetyGuardPlanning::UpdateStateMachine(
    const bool is_real_time_triggered,
    planning_internal::SafetyGuardInfo* const safety_guard_info_ptr,
    planning::ADCTrajectory* const adc_trajectory_pb_ptr) {
  DCHECK(nullptr != safety_guard_info_ptr);
  DCHECK(nullptr != adc_trajectory_pb_ptr);
  const auto& fct_in = local_view_->GetFunctionManagerIn()->fct_avp_in();
  const bool is_in_ntp_tba_function =
      (fct_in.sys_mode() == functionmanager::AvpFctIn::LAPA &&
       fct_in.sys_command() == functionmanager::AvpFctIn::PARKINPOLIT) ||
      (fct_in.sys_mode() == functionmanager::AvpFctIn::TBA &&
       fct_in.sys_command() == functionmanager::AvpFctIn::TBACONTROL);
  const auto& guard_config = config_.safety_guard_planning_config();
  static constexpr double kConfidenceEpsilon = 1e-5;
  uint32_t guard_enable_statistical_frame_num =
      guard_config.apa_guard_enable_statistical_frame_num();
  double min_guard_enable_confidence =
      guard_config.apa_min_guard_enable_confidence();
  if (is_in_ntp_tba_function) {
    guard_enable_statistical_frame_num =
        guard_config.cruise_guard_enable_statistical_frame_num();
    min_guard_enable_confidence =
        guard_config.cruise_min_guard_enable_confidence();
  }
  switch (cur_stage_) {
    case StageType::UNKNOWN: {
      if (local_view_->GetFunctionManagerIn()->fct_avp_in().sys_run_state() ==
          functionmanager::AvpFctIn::PAUSE) {
        break;
      }
      existence_frame_num_ = is_real_time_triggered ? (existence_frame_num_ + 1)
                                                    : existence_frame_num_;
      triggered_state_queue_.push(is_real_time_triggered);
      while (triggered_state_queue_.size() >
             guard_enable_statistical_frame_num) {
        existence_frame_num_ = triggered_state_queue_.front()
                                   ? (existence_frame_num_ - 1)
                                   : existence_frame_num_;
        triggered_state_queue_.pop();
      }
      double guard_enable_confidence = 0.0;
      if (triggered_state_queue_.size() == guard_enable_statistical_frame_num) {
        guard_enable_confidence = static_cast<double>(existence_frame_num_) /
                                  guard_enable_statistical_frame_num;
      }
      if (guard_enable_confidence >
          min_guard_enable_confidence - kConfidenceEpsilon) {
        if (is_real_time_triggered) {
          if (safety_guard_info_ptr->is_real_time_triggered_uss()) {
            triggered_hold_type_ = planning_internal::SafetyGuardInfo::USS;
          } else {
            triggered_hold_type_ =
                planning_internal::SafetyGuardInfo::FREE_SPACE;
          }
          existence_frame_num_ = 0;
          while (!triggered_state_queue_.empty()) {
            triggered_state_queue_.pop();
          }
          cur_stage_ = StageType::BRAKING;
          triggered_gear_ = frame_->vehicle_state().gear();
          safety_guard_info_ptr->set_stage_type(
              planning_internal::SafetyGuardInfo::BRAKING);
          safety_guard_info_ptr->set_triggered_hold_type(triggered_hold_type_);
          adc_trajectory_pb_ptr->mutable_function_manager_out()
              ->mutable_avp_fct_out()
              ->set_parking_status(functionmanager::AvpFctOut::RUNNING);
          is_safety_guard_triggered_ = true;
        }
      }
    } break;
    case StageType::BRAKING: {
      constexpr double kSpeedEpsilon = 0.05;
      if (std::fabs(frame_->vehicle_state().linear_velocity()) <
          kSpeedEpsilon) {
        if (is_in_ntp_tba_function) {
          cur_stage_ = StageType::WAIT_OBSTACLE;
          triggered_start_time_ = Clock::NowInSeconds();
          safety_guard_info_ptr->set_stage_type(
              planning_internal::SafetyGuardInfo::WAIT_OBSTACLE);
        } else {
          cur_stage_ = StageType::WAIT_REPLAN;
          triggered_start_time_ = Clock::NowInSeconds();
          safety_guard_info_ptr->set_stage_type(
              planning_internal::SafetyGuardInfo::WAIT_REPLAN);
        }
      } else {
        safety_guard_info_ptr->set_stage_type(
            planning_internal::SafetyGuardInfo::BRAKING);
      }
      adc_trajectory_pb_ptr->mutable_function_manager_out()
          ->mutable_avp_fct_out()
          ->set_parking_status(functionmanager::AvpFctOut::RUNNING);
      safety_guard_info_ptr->set_triggered_hold_type(triggered_hold_type_);
    } break;
    case StageType::WAIT_REPLAN: {
      const double wait_time = Clock::NowInSeconds() - triggered_start_time_;
      if ((!guard_config.enable_wait_for_replan() &&
           wait_time > guard_config.force_wait_time()) ||
          (guard_config.enable_wait_for_replan() &&
           wait_time > guard_config.max_wait_time_for_replan())) {
        cur_stage_ = StageType::WAIT_OBSTACLE;
        safety_guard_info_ptr->set_stage_type(
            planning_internal::SafetyGuardInfo::WAIT_OBSTACLE);
        adc_trajectory_pb_ptr->mutable_function_manager_out()
            ->mutable_avp_fct_out()
            ->set_parking_status(functionmanager::AvpFctOut::WAITOBSTACLE);
      } else {
        local_view_->SetGuardTriggeredFlag(
            guard_config.enable_wait_for_replan());
        safety_guard_info_ptr->set_stage_type(
            planning_internal::SafetyGuardInfo::WAIT_REPLAN);
        adc_trajectory_pb_ptr->mutable_function_manager_out()
            ->mutable_avp_fct_out()
            ->set_parking_status(functionmanager::AvpFctOut::RUNNING);
      }
      safety_guard_info_ptr->set_triggered_hold_type(triggered_hold_type_);
    } break;
    case StageType::WAIT_OBSTACLE: {
      const uint32_t guard_disabled_statistical_frame_num =
          guard_config.guard_disabled_statistical_frame_num();
      double min_disabled_distance =
          (guard_config.min_control_distance() +
           guard_config.sensor_dead_zone_distance()) *
          guard_config.min_disabled_distance_ratio();
      if ((fct_in.sys_mode() == functionmanager::AvpFctIn::RPA &&
           (fct_in.sys_command() == functionmanager::AvpFctIn::FORWARDCONTROL ||
            fct_in.sys_command() ==
                functionmanager::AvpFctIn::BACKWARDCONTROL)) ||
          is_in_ntp_tba_function) {
        min_disabled_distance = guard_config.min_control_distance() +
                                safety_guard_info_ptr->min_safe_distance();
      }
      bool guard_triggered =
          std::min(safety_guard_info_ptr->min_distance_to_free_space(),
                   safety_guard_info_ptr->min_distance_to_uss()) <
          min_disabled_distance;
      existence_frame_num_ =
          guard_triggered ? (existence_frame_num_ + 1) : existence_frame_num_;
      triggered_state_queue_.push(guard_triggered);
      while (triggered_state_queue_.size() >
             guard_disabled_statistical_frame_num) {
        existence_frame_num_ = triggered_state_queue_.front()
                                   ? (existence_frame_num_ - 1)
                                   : existence_frame_num_;
        triggered_state_queue_.pop();
      }
      if (triggered_state_queue_.size() ==
          guard_disabled_statistical_frame_num) {
        safety_guard_info_ptr->set_guard_disabled_confidence(
            1.0 - static_cast<double>(existence_frame_num_) /
                      guard_disabled_statistical_frame_num);
      } else {
        safety_guard_info_ptr->set_guard_disabled_confidence(0.0);
      }
      if (safety_guard_info_ptr->guard_disabled_confidence() >
          guard_config.min_guard_disabled_confidence() - kConfidenceEpsilon) {
        cur_stage_ = StageType::RUNNING;
        safety_guard_info_ptr->set_stage_type(
            planning_internal::SafetyGuardInfo::RUNNING);
        adc_trajectory_pb_ptr->mutable_function_manager_out()
            ->mutable_avp_fct_out()
            ->set_parking_status(functionmanager::AvpFctOut::RUNNING);
      } else {
        if (is_in_ntp_tba_function) {
          safety_guard_info_ptr->set_stage_type(
              planning_internal::SafetyGuardInfo::WAIT_OBSTACLE);
          adc_trajectory_pb_ptr->mutable_function_manager_out()
              ->mutable_avp_fct_out()
              ->set_parking_status(functionmanager::AvpFctOut::RUNNING);
        } else {
          safety_guard_info_ptr->set_stage_type(
              planning_internal::SafetyGuardInfo::WAIT_OBSTACLE);
          adc_trajectory_pb_ptr->mutable_function_manager_out()
              ->mutable_avp_fct_out()
              ->set_parking_status(functionmanager::AvpFctOut::WAITOBSTACLE);
        }
      }
      safety_guard_info_ptr->set_triggered_hold_type(triggered_hold_type_);
    } break;
    case StageType::RUNNING: {
      safety_guard_info_ptr->set_stage_type(
          planning_internal::SafetyGuardInfo::RUNNING);
      adc_trajectory_pb_ptr->mutable_function_manager_out()
          ->mutable_avp_fct_out()
          ->set_parking_status(functionmanager::AvpFctOut::RUNNING);
      const auto& fct_in = local_view_->GetFunctionManagerIn()->fct_avp_in();
      if (is_in_ntp_tba_function) {
        InitParameter();
      }
    } break;
    default:
      break;
  }
}

bool SafetyGuardPlanning::CheckPlanningConfig(  // NOLINT
    const PlanningConfig& config) {             // NOLINT
  if (!config.has_safety_guard_planning_config()) {
    return false;
  }
  if (!config.safety_guard_planning_config().has_planner_type()) {
    return false;
  }
  if (config.safety_guard_planning_config().planner_type() !=
      planning::PlannerType::SAFETY_GUARD) {
    return false;
  }

  return true;
}

}  // namespace planning
}  // namespace TL
