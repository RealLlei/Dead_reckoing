/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "planning/core/on_lane_planning.h"
#include <sys/types.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <utility>

#include "common/configs/config_gflags.h"
#include "common/file/file.h"
#include "common/file/log.h"
#include "common/math/double_type.h"
#include "common/math/vec2d.h"
#include "common/status/status.h"
#include "common/time/clock.h"
// #include "gtest/gtest_prod.h"
#include "common/time/time.h"
#include "common/utm_projection/coordinate_convertor.h"
#include "common/vehicle_state/vehicle_state_provider.h"
#include "map/hdmap/hdmap_util.h"
#include "planning/common/ego_info.h"
#include "planning/common/frame.h"
#include "planning/common/history.h"
#include "planning/common/planning_context.h"
#include "planning/common/planning_gflags.h"
#include "planning/common/real_jerk/real_jerk.h"
#include "planning/common/reference_line_info.h"
#include "planning/common/trajectory_stitcher.h"
#include "planning/common/util/util.h"

#include "proto/fsm/avp_fct.pb.h"
#include "proto/fsm/function_manager.pb.h"
#include "proto/fsm/nnp_fct.pb.h"
#include "proto/fsm/soc_to_mcu.pb.h"
#include "proto/map/navigation.pb.h"
#include "proto/perception/perception_obstacle.pb.h"
#include "proto/planning/planning_internal.pb.h"
#include "proto/soc/chassis.pb.h"
// #include "proto/planning/planning_semantic_map_config.pb.h"
#include "common/utm_projection/utm_zone.h"
#include "planning/common/planning_chart_debug.h"
#include "planning/lane_change_safety_decider/lane_change_safety_decider.h"
#include "planning/localview/local_view.h"
#include "planning/reference_line/reference_line_provider.h"
#include "planning/traffic_rules/traffic_decider.h"
#include "planning/warning/lbs/common/TM_Global_Types.h"

#include "proto/routing/poi.pb.h"
#include "proto/routing/routing.pb.h"
#define TEST (0)

namespace TL {
namespace planning {
// NOLINTBEGIN
using TL::common::Clock;
using TL::common::EngageAdvice;
using TL::common::ErrorCode;
using TL::common::Status;
using TL::common::TrajectoryPoint;
using TL::common::VehicleState;
using TL::common::VehicleStateProvider;
using TL::functionmanager::SocToFctBus;
using TL::routing::RoutingRequest;
using TL::soc::Chassis;

// NOLINTEND
OnLanePlanning::OnLanePlanning(
    const std::shared_ptr<DependencyInjector>& injector)
    : PlanningBase(injector) {
  planner_dispatcher_ = std::make_unique<OnLanePlannerDispatcher>();
  pnc_map_ = std::make_shared<hdmap::PncMap>();
  reference_line_info_decider_ = std::make_unique<ReferenceLineInfoDecider>();
  lane_change_safety_decider_ = std::make_unique<LaneChangeSafetyDecider>();
  prediction_input_queue_.Init(
      FLAGS_local_view_queue_size,
      new common::base::TimeoutBlockWaitStrategy(  // NOLINT
          FLAGS_local_view_queue_timeout));        // NOLINT
  planning_input_queue_.Init(
      FLAGS_local_view_queue_size,
      new common::base::TimeoutBlockWaitStrategy(  // NOLINT
          FLAGS_local_view_queue_timeout));        // NOLINT
}

OnLanePlanning::~OnLanePlanning() {
  FunctionStatistics done_guard("~OnLanePlanning()");
  Stop();
  if (reference_line_provider_) {
    reference_line_provider_->Stop();
  }
  planner_->Stop();
  injector_->frame_history()->Clear();
  injector_->history()->Clear();
  injector_->planning_context()->mutable_planning_status()->Clear();
  last_routing_.Clear();
  if (FLAGS_enable_online_hdmap_generator) {
    online_hdmap_generator_.Stop();
  }
}

Status OnLanePlanning::Stop() {
  FunctionStatistics done_guard("OnLanePlanning::Stop");
  local_view_constructor_.Stop();
  prediction_input_queue_.BreakAllWait();
  planning_input_queue_.BreakAllWait();
  is_prediction_thread_stop_ = true;
  is_planning_thread_stop_ = true;
  if (FLAGS_enable_prediction_output && prediction_thread_.joinable()) {
    prediction_thread_.join();
  }

  if (planning_thread_.joinable()) {
    planning_thread_.join();
  }
  injector_->ego_info()->Clear();
  return Status(ErrorCode::OK);
}

std::string OnLanePlanning::Name() const {
  return "on_lane_planning";
}

Status OnLanePlanning::Init(const PlanningConfig& config) {
  hdmap_roadtype_debounce_.ResetTime(0.0, 0.3, 0.1);
  config_ = config;
  if (!CheckPlanningConfig(config_)) {
    return Status(ErrorCode::CORE_ONLANE_INIT_ERROR,
                  "planning config error: " + config_.DebugString());
  }

  PlanningBase::Init(config_);

  planner_dispatcher_->Init();

  ACHECK(TL::common::GetProtoFromFile(FLAGS_traffic_rule_config_filename,
                                         &traffic_rule_configs_))
      << "Failed to load traffic rule config file "
      << FLAGS_traffic_rule_config_filename;
  ACHECK(common::GetProtoFromFile(FLAGS_vehicle_model_config_filename,
                                  &vehicle_model_config_))
      << "Failed to load vehicle model config file "
      << FLAGS_vehicle_model_config_filename;
  vehicle_param_ =
      TL::common::VehicleConfigHelper::GetConfig().vehicle_param();
  // clear planning history
  injector_->history()->Clear();

  // clear planning status
  injector_->planning_context()->mutable_planning_status()->Clear();

  reference_line_provider_ = std::make_unique<ReferenceLineProvider>(pnc_map_);
  reference_line_provider_->Start();

  vehicle_state_provider_ = std::make_unique<common::VehicleStateProvider>();
  // dispatch planner
  planner_ = planner_dispatcher_->DispatchPlanner(config_, injector_);
  if (!planner_) {
    return Status(
        ErrorCode::CORE_ONLANE_INIT_ERROR,
        "planning is not initialized with config : " + config_.DebugString());
  }
  local_view_constructor_.Init();
  local_view_constructor_.start();

  predictor_ = std::make_unique<prediction::Prediction>();
  auto status = predictor_->Init();

  if (!status.ok()) {
    return status;
  }

  if (FLAGS_enable_prediction_output) {
    prediction_thread_ =
        std::thread(&OnLanePlanning::GeneratePredictionThread, this);
  }
  planning_thread_ = std::thread(&OnLanePlanning::GeneratePlanningThread, this);

  if (FLAGS_enable_online_hdmap_generator) {
    online_hdmap_generator_.Init();
  }
  if (FLAGS_enable_planning_self_simulator) {
    self_simulator_.Init();
    self_simulator_.Start();
  }

  can_hmi_nnp_avp_ = std::make_unique<FunctionManager>();
  real_jerk_ = std::make_unique<RealJerk>();
  return planner_->Init(config_);
}

Status OnLanePlanning::InitFrame(const uint32_t sequence_num,
                                 const TrajectoryPoint& planning_start_point,
                                 const VehicleState& vehicle_state) {
  std::list<std::shared_ptr<ReferenceLine>> reference_lines;
  std::list<hdmap::RouteSegments> segments;
  reference_line_provider_->UpdateCruiseSpeed(
      local_view_->GetFunctionManagerIn()
          ->fct_nnp_in()
          .longitud_ctrl_cruise_speedms());
  const bool enable_smooth_select_reference_line =
      FLAGS_enable_smooth_select_reference_line &&
      local_view_->HasFunctionManagerOut() &&
      local_view_->GetFunctionManagerOut()->has_hdmap_sub_state() &&
      local_view_->GetFunctionManagerOut()->hdmap_sub_state() ==
          functionmanager::HdmapSubState::MAP_FUSION_TYPE;
  if (!((!FLAGS_enable_smooth_reference_line ||
         enable_smooth_select_reference_line)
            ? reference_line_provider_->GetUnsmoothReferenceLines(
                  &reference_lines, &segments)
            : reference_line_provider_->GetReferenceLines(&reference_lines,
                                                          &segments))) {
    const std::string msg = "Failed to create reference line";
    AERROR << msg;
    return Status(ErrorCode::CORE_FRAME_REFLINESEGEMNT_ERROR, msg);
  }
  if (reference_lines.size() != segments.size()) {
    const std::string msg =
        "Failed to create reference line,reference_lines.size()!=  "
        "segments.size()";
    AERROR << msg;
    return Status(ErrorCode::CORE_FRAME_REFLINESEGEMNT_ERROR, msg);
  }

  frame_ =
      std::make_unique<Frame>(sequence_num, local_view_, planning_start_point,
                              vehicle_state, reference_line_provider_.get());
  if (frame_ == nullptr) {
    return Status(ErrorCode::CORE_FRAME_INITNULLPTR_ERROR,
                  "Fail to init frame: nullptr.");
  }
  frame_->SetMachineStateType(
      local_view_->GetFunctionManagerOut()->fsm_state());
  frame_->SetIsStateChange(is_fsm_state_changed_);
  // for (auto& ref_line : reference_lines) {
  //   if (!ref_line.Segment(Vec2d(vehicle_state.x(), vehicle_state.y()),
  //                         reference_info_config_.backward_distance(),
  //                         reference_info_config_.forward_distance())) {
  //     const std::string msg = "Fail to shrink reference line.";
  //     AERROR << msg;
  //     return Status(ErrorCode::CORE_FRAME_REFLINESEGEMNT_ERROR, msg);
  //   }
  // }
  // for (auto& seg : segments) {
  //   if (!seg.Shrink(Vec2d(vehicle_state.x(), vehicle_state.y()),
  //                   reference_info_config_.backward_distance(),
  //                   reference_info_config_.forward_distance())) {
  //     const std::string msg = "Fail to shrink routing segments.";
  //     AERROR << msg;
  //     return Status(ErrorCode::CORE_FRAME_SEGMENTSHRINK_ERROR, msg);
  //   }
  // }
  auto status = frame_->Init(
      injector_->vehicle_state(), reference_lines, segments,
      reference_line_provider_->FutureRouteWaypoints(), injector_->ego_info());
  if (!status.ok()) {
    AERROR << "failed to init frame:" << status.ToString();
    return status;
  }

  auto start_time = common::Clock::NowInMicroseconds();
  if (!reference_line_info_decider_->ProcessReferenceLineInfoDecider(
          injector_, frame_.get())) {
    return Status(ErrorCode::CORE_FRAME_FREFERENCEDECIDER_ERROR,
                  "Fail to make a reference line info decider.");
  }
  reference_line_info_decider_use_time_ =
      common::Clock::NowInMicroseconds() - start_time;

  if (FLAGS_enable_smooth_reference_line &&
      enable_smooth_select_reference_line) {
    UpdateRefListFromDecider(frame_->mutable_reference_line_info(),
                             &reference_lines, &segments);

    if (!reference_line_provider_->GetSelectSmoothReferenceLines(
            &reference_lines, &segments)) {
      const std::string msg = "Failed to create reference line";
      AERROR << msg;
      return Status(ErrorCode::CORE_FRAME_REFLINESEGEMNT_ERROR, msg);
    }

    if (reference_lines.size() != segments.size()) {
      const std::string msg =
          "Failed to create reference line,reference_lines.size()!=  "
          "segments.size()";
      AERROR << msg;
      return Status(ErrorCode::CORE_FRAME_REFLINESEGEMNT_ERROR, msg);
    }
  }
  for (auto& ref_line_info : *frame_->mutable_reference_line_info()) {
    if (ref_line_info.mutable_reference_line() == nullptr) {
      continue;
    }
    ref_line_info.mutable_reference_line()->UpdateOverlapsPosition();
  }

  start_time = common::Clock::NowInMicroseconds();
  ADEBUG << "[safety decider]:Execute Safety Decider.";
  lane_change_safety_decider_->Execute(injector_, frame_.get());
  lane_change_safety_use_time_ =
      common::Clock::NowInMicroseconds() - start_time;
  return Status::OK();
}

bool OnLanePlanning::UpdateRefListFromDecider(
    std::list<ReferenceLineInfo>* const select_ref_list,
    std::list<std::shared_ptr<ReferenceLine>>* const reference_lines,
    std::list<hdmap::RouteSegments>* const segments) {
  if (select_ref_list == nullptr || reference_lines == nullptr ||
      segments == nullptr) {
    AERROR << "input pointer is nullptr.";
    return false;
  }

  // 遍历列表reference_lines和segments，保留与列表select_ref_list匹配的元素
  reference_lines->erase(
      std::remove_if(reference_lines->begin(), reference_lines->end(),
                     [&select_ref_list](const auto& ptrRef) {
                       return ptrRef &&
                              std::find_if(
                                  select_ref_list->begin(),
                                  select_ref_list->end(),
                                  [ptrRef](const auto& ptrSelect) {
                                    return ptrSelect.reference_line().Tag() ==
                                           ptrRef->Tag();
                                  }) == select_ref_list->end();
                     }),
      reference_lines->end());

  segments->erase(
      std::remove_if(segments->begin(), segments->end(),
                     [&select_ref_list](const auto& ptrSeg) {
                       return std::find_if(
                                  select_ref_list->begin(),
                                  select_ref_list->end(),
                                  [ptrSeg](const auto& ptrSelect) {
                                    return ptrSelect.reference_line().Tag() ==
                                           ptrSeg.Tag();
                                  }) == select_ref_list->end();
                     }),
      segments->end());

  return true;
}

// TODO(all): fix this! this will cause unexpected behavior from controller
void OnLanePlanning::GenerateStopTrajectory(
    const VehicleState& vehicle_state, ADCTrajectory* const ptr_trajectory_pb) {
  ptr_trajectory_pb->set_is_vehicle_reference_frame(
      vehicle_state.is_vehicle_reference_frame());
  if (!ptr_trajectory_pb->has_header()) {
    common::util::FillHeader("planning", ptr_trajectory_pb);
  }
  ptr_trajectory_pb->clear_trajectory_point();
  ptr_trajectory_pb->set_gear(vehicle_state.gear() == Chassis::GEAR_NEUTRAL
                                  ? Chassis::GEAR_PARKING
                                  : vehicle_state.gear());
  if (ptr_trajectory_pb->has_function_manager_in() &&
      ptr_trajectory_pb->function_manager_in().has_ta_pilot_mode() &&
      ptr_trajectory_pb->function_manager_in().ta_pilot_mode() ==
          functionmanager::AVP) {
    ptr_trajectory_pb->set_function_mode(9);
  }
  DiscretizedTrajectory trajectory;
  trajectory.SetStopTrajectory(vehicle_state.x(), vehicle_state.y(),
                               vehicle_state.heading(), vehicle_state.kappa());
  PublishableTrajectory stop_trajectory(
      ptr_trajectory_pb->header().data_stamp(), trajectory);
  stop_trajectory.PopulateTrajectoryProtobuf(ptr_trajectory_pb);
  ptr_trajectory_pb->set_received_ehp_counter(received_ehp_counter_);
  if (FLAGS_publish_estop) {
    AERROR << "Planning failed and set estop";
    EStop* estop = ptr_trajectory_pb->mutable_estop();
    estop->set_is_estop(true);
    estop->set_reason("GenerateStopTrajectory");
  }
}

/**
 * @func check the longitudinal validation of init point of stitching
 * trajectory.
 * @param veh_max_acce
 * @param veh_max_dece
 * @param stitching_trajectory
 */
void OnLanePlanning::CheckStitchPointValid(
    const double veh_max_acce, const double veh_max_dece, const bool is_forward,
    RepeatedPtrField<TrajectoryPoint>* const stitching_trajectory) {
  if (!stitching_trajectory->empty() && veh_max_acce > 0 && veh_max_dece < 0) {
    static constexpr double epsilon = 1e-5;
    const auto& init_point = *stitching_trajectory->rbegin();
    if (is_forward) {
      if (init_point.a() >= veh_max_acce - epsilon) {
        stitching_trajectory->rbegin()->set_a(veh_max_acce - epsilon);
      } else if (init_point.a() <= veh_max_dece + epsilon) {
        stitching_trajectory->rbegin()->set_a(veh_max_dece + epsilon);
      }
      if (init_point.v() < epsilon && init_point.a() < epsilon) {
        stitching_trajectory->rbegin()->set_v(epsilon);
        stitching_trajectory->rbegin()->set_a(init_point.a());
      }
    } else {
      if (init_point.a() <= -veh_max_acce + epsilon) {
        stitching_trajectory->rbegin()->set_a(-veh_max_acce + epsilon);
      } else if (init_point.a() >= -veh_max_dece - epsilon) {
        stitching_trajectory->rbegin()->set_a(-veh_max_dece - epsilon);
      }
      if (init_point.v() > -epsilon && init_point.a() > -epsilon) {
        stitching_trajectory->rbegin()->set_v(-epsilon);
        stitching_trajectory->rbegin()->set_a(init_point.a());
      }
    }
  } else {
    AERROR << "stitching trajectory size less than 1. or acce set error -> "
              "stitching trajectory size "
           << stitching_trajectory->size() << "  veh_max_acce:" << veh_max_acce
           << "  veh_max_dece:" << veh_max_dece << ".";
  }
}

void OnLanePlanning::PrintPlanningTimeout(  // NOLINT
    const std::shared_ptr<LocalView>& local_view) {
  if (local_view == nullptr || !local_view->HasFunctionManagerIn() ||
      !local_view->HasAdasSomeipFromMCU()) {
    return;
  }
  static double last_nnp_active_timestamp = 0.0;
  static double last_avp_active_timestamp = 0.0;
  const auto& nnp_switch_conditions =
      local_view->GetFunctionManagerIn()->nnp_switch_conditions();
  const auto& ta_pilot_mode =
      local_view->GetFunctionManagerIn()->ta_pilot_mode();
  if (ta_pilot_mode != functionmanager::TaPilotMode::NO_CONTROL &&
      ta_pilot_mode != functionmanager::TaPilotMode::AVP) {
    last_nnp_active_timestamp = Clock::NowInSeconds();
  }
  if (Clock::NowInSeconds() - last_nnp_active_timestamp < 0.5 &&
      nnp_switch_conditions.has_is_planning_count_ok() &&
      nnp_switch_conditions.is_planning_count_ok()) {
    AERROR << "NNP control receiving trajectory timeout";
  }
  if (local_view->GetAdasSomeipFromMCU()->adas_someip_size() < 2343) {
    return;
  }
  const auto& time_gap = local_view->GetAdasSomeipFromMCU()->adas_someip(2342);
  float time_gap_float = 0.0F;
  std::memcpy(&time_gap_float, &time_gap, sizeof(float));
  if (ta_pilot_mode == functionmanager::TaPilotMode::AVP) {
    last_avp_active_timestamp = Clock::NowInSeconds();
  }
  if (Clock::NowInSeconds() - last_avp_active_timestamp < 0.5 &&
      time_gap_float > 2.0) {
    AERROR << "AVP control receiving trajectory timeout: " << time_gap_float;
  }
}

void OnLanePlanning::RunOnce(const std::shared_ptr<LocalView>& local_view) {
  // 如果使用is_record_replay，直接加载map和routing，跳过lcoalview线程
  // if (FLAGS_enable_fault_collect) {
  //   fault_collect_.DiagnosisInputData(local_view);
  // }
  const auto start_times_runonce = Clock::NowInSeconds();
  double localview_start_time = start_times_runonce;
  double localview_end_time = start_times_runonce;
  PrintPlanningTimeout(local_view);
  if (!local_view->HasLocalization() ||
      (!local_view->GetLocalization()->has_header() ||
       !local_view->GetLocalization()->has_pose())) {
    localization::Localization localization;
    common::util::FillHeader("vrf_localization", &localization);
    local_view->SetLocalizationPtr(
        std::make_shared<localization::Localization>(localization));
  }
  if (local_view->HasHDMap() && local_view->HasRoutingResponse()) {
    auto status = vehicle_state_provider_->Update(
        *local_view->GetLocalization(), *local_view->GetChassis());
    if (status.ok()) {
      auto vehicle_state = std::make_shared<common::VehicleState>();
      vehicle_state->CopyFrom(vehicle_state_provider_->vehicle_state());
      local_view->SetVehicleStatePtr(vehicle_state);
      prediction_input_queue_.WaitEnqueue(local_view);
      return;
    }
  }

  ADEBUG << "fct signal is "
         << local_view->GetFunctionManagerIn()->DebugString();
  static std::shared_ptr<hdmap::HDMap> map_ptr_for_self_simulator = nullptr;
  if (FLAGS_enable_planning_self_simulator) {
    self_simulator_.Process(local_view, map_ptr_for_self_simulator);
  }
  if (FLAGS_enable_planning_injector_obs) {
    self_simulator_.InjectorPerceptionObs(local_view,
                                          map_ptr_for_self_simulator);
  }
#ifdef FOR_BAIDU_SIMULATION
  {
    std::lock_guard<std::mutex> lock(last_function_manager_out_mutex_);
    can_hmi_nnp_avp_->SetSimulationFctValue(local_view,
                                            last_function_manager_out_.get());
  }
#else
  if (FLAGS_enable_dreamview_to_planning_zmq) {
    std::lock_guard<std::mutex> lock(last_function_manager_out_mutex_);
    can_hmi_nnp_avp_->SetSimulationFctValue(local_view,
                                            last_function_manager_out_.get());
  }
#endif
// 选择新的utm zone，并据此补全定位信息，并更新vehicle state
#ifdef ISMDC
  u_int32_t new_utm_zone = 0;
  common::utm_zone::SelectNewZoneID(
      previous_utm_zone_, local_view->GetLocalization(), &new_utm_zone);
  auto new_localization = std::make_shared<localization::Localization>(
      *local_view->GetLocalization());
  auto new_obs = std::make_shared<perception::PerceptionObstacles>(
      *local_view->GetPerceptionObstacles());
  if (common::utm_zone::SetLocalizationAndPerceptionByZoneID(
          new_utm_zone, new_localization, new_obs)) {
    local_view->SetLocalizationPtr(new_localization);
    local_view->SetPerceptionObstaclesPtr(new_obs);
  } else {
    // 此处如果utm相关操作失败，则直接丢弃 local_view，不做任何处理，目前 planning 处于不受控状态
    AERROR << "utm zone error, new " << new_utm_zone << " receive "
           << local_view->GetLocalization()->ShortDebugString();
    return;
  }
#endif
  // 在线生成地图（使用自车轨迹）
  if (FLAGS_enable_online_hdmap_generator) {
    online_hdmap_generator_.Update(*local_view->GetLocalization(),
                                   *local_view->GetLaneMarkers());
    return;
  }
  {
    std::lock_guard<std::mutex> lock(rerouting_mutex_);
    if (rerouting_ptr_ && (!rerouting_ptr_->waypoint().empty())) {
      local_view->SetRoutingRequestPtr(rerouting_ptr_);
    }
  }
  auto status = vehicle_state_provider_->Update(*local_view->GetLocalization(),
                                                *local_view->GetChassis());
  if (status.ok()) {
    auto vehicle_state = std::make_shared<common::VehicleState>();
    vehicle_state->CopyFrom(vehicle_state_provider_->vehicle_state());
    local_view->SetVehicleStatePtr(vehicle_state);
    localview_start_time = Clock::NowInSeconds();
    status = local_view_constructor_.BuildLocalView(local_view);
    localview_end_time = Clock::NowInSeconds();
    // 如果 build localview 失败，进入停车轨迹
    if (!status.ok()) {
      AERROR
          << "build local view failed "
          << local_view->GetPerceptionObstacles()->header().ShortDebugString();
    }

    if (FLAGS_use_fake_perception) {
      auto perception = std::make_shared<perception::PerceptionObstacles>();
      common::util::FillHeader("fake_perception", perception.get());
      local_view->SetPerceptionObstaclesPtr(perception);
    }
    // 即使选择好了新的utm zone，由于ehr构建地图需要时间，real lane line
    // builder中会把定位重新设置到老的utm zone，这时候需要重新更新一下vehicle
    // state到老的utm zone
    if (local_view->GetFunctionManagerOut()->fsm_state() ==
            MachineStateType::HDMAP_TYPE &&
        local_view->GetVehicleState()->pose().using_utm_zone() !=
            local_view->GetLocalization()->pose().using_utm_zone()) {
      if (vehicle_state_provider_
              ->Update(*local_view->GetLocalization(),
                       *local_view->GetChassis())
              .ok()) {
        auto vehicle_state = std::make_shared<common::VehicleState>();
        vehicle_state->CopyFrom(vehicle_state_provider_->vehicle_state());
        local_view->SetVehicleStatePtr(vehicle_state);
      } else {
        status = Status(ErrorCode::CORE_ONLANE_VEHICLESTATECHECK_ERROR,
                        "vehicle state update error when updating utm zone");
      }
    }

    // 地图指针传给SelfSimulator
    if (FLAGS_enable_planning_self_simulator ||
        FLAGS_enable_planning_injector_obs) {
      map_ptr_for_self_simulator = local_view->GetHDMapPtr();
    }
  }
  if (local_view->HasFunctionManagerOut()) {
    const auto run_once_end_time = Clock::NowInSeconds();
    auto fct_out = *local_view->GetFunctionManagerOut();
    fct_out.mutable_nnp_fct_out()->mutable_localview_time()->set_run_once_time(
        (run_once_end_time - start_times_runonce) * 1000);
    fct_out.mutable_nnp_fct_out()->mutable_localview_time()->set_localview_time(
        (localview_end_time - localview_start_time) * 1000);
    local_view->SetFunctionManagerOutPtr(
        std::make_shared<functionmanager::FunctionManagerOut>(fct_out));
  }

  functionmanager::AvpFctOut::ParkState parking_status =
      functionmanager::AvpFctOut::RUNNING;
  InputValidateCheck(local_view, &status, &parking_status);

  if (!status.ok()) {
    AERROR << "Failed to validate local view for Planning";
    auto ptr_trajectory_pb = std::make_shared<ADCTrajectory>();
    if (local_view->HasFunctionManagerIn()) {
      ptr_trajectory_pb->mutable_function_manager_in()->CopyFrom(
          *local_view->GetFunctionManagerIn());
    }
    if (local_view->HasFunctionManagerOut()) {
      ptr_trajectory_pb->mutable_function_manager_out()->CopyFrom(
          *local_view->GetFunctionManagerOut());
      ptr_trajectory_pb->mutable_function_manager_out()
          ->mutable_avp_fct_out()
          ->set_parking_status(parking_status);
    }
    std::string msg = status.error_message();
    AERROR << msg;
    ptr_trajectory_pb->mutable_decision()
        ->mutable_main_decision()
        ->mutable_not_ready()
        ->set_reason(msg);
    GenerateStopTrajectory(vehicle_state_provider_->vehicle_state(),
                           ptr_trajectory_pb.get());
    status.Save(ptr_trajectory_pb->mutable_header()->mutable_status());
    local_view->SetADCTrajectoryPtr(ptr_trajectory_pb);
    publish_queue_->WaitEnqueue(local_view);
  }
}

void OnLanePlanning::InputValidateCheck(
    const std::shared_ptr<LocalView>& local_view, Status* const status,
    functionmanager::AvpFctOut::ParkState* const parking_status) {
  // check if perception has valid header
  if (local_view->HasFunctionManagerOut() &&
      (local_view->GetFunctionManagerOut()->fsm_state() ==
           functionmanager::MachineStateType::APA_TYPE ||
       local_view->GetFunctionManagerOut()->fsm_state() ==
           functionmanager::MachineStateType::HISTORY_TRACE_TYPE ||
       local_view->GetFunctionManagerOut()->fsm_state() ==
           functionmanager::MachineStateType::HDMAP_AVP_TYPE)) {
    fsm_avp_sequence_num_++;
  } else {
    fsm_avp_sequence_num_ = 0;
  }
  if (status->ok()) {
    *status = CheckPerceptionAndLocalization(local_view);
    ADEBUG << " perception and localization status: " << status->ok();
  }
  static constexpr uint64_t kMaxWaitCounter = 4;
  if (!status->ok() && fsm_avp_sequence_num_ > kMaxWaitCounter) {
    *parking_status = functionmanager::AvpFctOut::RUNONCEFAILED;
  }
  if (status->ok()) {
    *status = Validate(local_view);
    ADEBUG << " other input status: " << status->ok();
  }
  if (status->ok() && !prediction_input_queue_.WaitEnqueue(local_view)) {
    std::string msg = "prediction input queue fail";
    *status = Status(ErrorCode::PREDICTION_PREDICTOR_ERROR, msg);
    AERROR << msg;
  }
  if (!status->ok() && local_view->HasFunctionManagerOut() &&
      *parking_status == functionmanager::AvpFctOut::RUNNING &&
      fsm_avp_sequence_num_ > kMaxWaitCounter) {
    *parking_status = functionmanager::AvpFctOut::PLANNINGFAILED;
  }
  if (local_view->HasFunctionManagerIn() &&
      (local_view->GetFunctionManagerIn()->fct_avp_in().sys_mode() ==
           functionmanager::AvpFctIn::LOCALIZATION ||
       local_view->GetFunctionManagerIn()->fct_avp_in().sys_mode() ==
           functionmanager::AvpFctIn::LOCALIZATION_BACKGROUND)) {
    *parking_status = functionmanager::AvpFctOut::RUNNING;
  }
}

Status OnLanePlanning::Validate(const std::shared_ptr<LocalView>& local_view) {
  // check if perception has valid header
  std::string msg = "local view";
  if (!local_view->HasRoutingResponse() || !local_view->HasChassis() ||
      !local_view->HasHDMap() ||
      (local_view->HasADCTrajectory() &&
       local_view->GetADCTrajectory()->has_header())) {
    msg += !local_view->HasRoutingResponse() ? ", has no Routing" : "";
    msg += !local_view->HasChassis() ? ", has no Chassis" : "";
    msg += !local_view->HasHDMap() ? ", has no HDMap" : "";
    msg += (local_view->HasADCTrajectory() &&
            local_view->GetADCTrajectory()->has_header())
               ? ", already has a valid trajectory"
               : "";
    AERROR << msg;
    return Status(ErrorCode::LOCALVIEW_INVALID_INPUT_ERROR, msg);
  }
  return Status::OK();
}

Status OnLanePlanning::CheckPerceptionAndLocalization(  // NOLINT
    const std::shared_ptr<LocalView>& local_view) {
  // check if perception has valid header
  std::string msg = "local view";
  const auto fsm_state = local_view->HasFunctionManagerOut()
                             ? local_view->GetFunctionManagerOut()->fsm_state()
                             : MachineStateType::INITIAL_TYPE;
  if (!local_view->HasValidPerceptionObstaclesHeader() ||
      (fsm_state != MachineStateType::PERCEPTION_TYPE &&
       !local_view->HasValidLocalizationHeader())) {
    msg += !local_view->HasValidPerceptionObstaclesHeader()
               ? ", has no Perception Obstacles"
               : "";
    msg += (fsm_state != MachineStateType::PERCEPTION_TYPE &&
            !local_view->HasValidLocalizationHeader())
               ? ", has no localization"
               : "";
    AERROR << msg
           << local_view->GetPerceptionObstacles()->header().ShortDebugString();
    return Status(ErrorCode::LOCALVIEW_INVALID_INPUT_ERROR, msg);
  }
#if defined(ISMDC) || defined(ISORIN)
  if (local_view->HasFunctionManagerIn() &&
      local_view->GetFunctionManagerIn()->ta_pilot_mode() ==
          functionmanager::AVP &&
      (!local_view->HasValidFreeSpaceOutArrayHeader())) {
    msg += " has no freespace";
    return Status(ErrorCode::LOCALVIEW_INVALID_INPUT_ERROR, msg);
  }
#endif

  return Status::OK();
}

void OnLanePlanning::CheckRerouting() {
  auto* rerouting = injector_->planning_context()
                        ->mutable_planning_status()
                        ->mutable_rerouting();
  if (!rerouting->need_rerouting()) {
    if (rerouting_ptr_ != nullptr) {
      std::lock_guard<std::mutex> lock(rerouting_mutex_);
      rerouting_ptr_ = nullptr;
    }
    return;
  }
  common::util::FillHeader("rerouting", rerouting->mutable_routing_request());

  {
    std::lock_guard<std::mutex> lock(rerouting_mutex_);
    rerouting_ptr_ =
        std::make_shared<routing::RoutingRequest>(rerouting->routing_request());
  }
  rerouting->set_need_rerouting(false);
}

Status OnLanePlanning::Plan(
    const double current_time_stamp,
    const RepeatedPtrField<TrajectoryPoint>& stitching_trajectory,
    ADCTrajectory* const ptr_trajectory_pb) {
  const auto& init_point = *stitching_trajectory.rbegin();
  auto* ptr_debug = ptr_trajectory_pb->mutable_debug();
  if (FLAGS_enable_record_debug) {
    frame_->mutable_open_space_info()->set_debug(ptr_debug);
    ptr_debug->mutable_planning_data()->mutable_init_point()->CopyFrom(
        init_point);
    frame_->mutable_open_space_info()->sync_debug_instance();
  }
  const double timestamp_all_task = Clock::NowInSeconds();
  auto status = planner_->Plan(init_point, frame_.get(), ptr_trajectory_pb);
  auto* total_task_time_latency =
      ptr_trajectory_pb->mutable_latency_stats()->add_task_stats();
  total_task_time_latency->set_name("total_tasks_time");
  total_task_time_latency->set_time_ms(
      (Clock::NowInSeconds() - timestamp_all_task) * 1000);
  if (!status.ok()) {
    if (last_publishable_trajectory_) {
      last_publishable_trajectory_->clear();
    }
    return status;
  }
  const double timestamp_debug_chart = Clock::NowInSeconds();
  ptr_debug->mutable_planning_data()->set_front_clear_distance(
      injector_->ego_info()->front_clear_distance());

  const auto* best_ref_info = frame_->FindDriveReferenceLineInfo();

  if (frame_->open_space_info().is_on_open_space_trajectory()) {
    // frame_->mutable_open_space_info()->sync_debug_instance();
    auto& publishable_trajectory = frame_->mutable_open_space_info()
                                       ->mutable_publishable_trajectory_data()
                                       ->first;
    if (!frame_->open_space_info().is_gear_changed()) {
      publishable_trajectory.PrependTrajectoryPoints(
          {stitching_trajectory.begin(), stitching_trajectory.end() - 1});
    }
    publishable_trajectory.PopulateTrajectoryProtobuf(ptr_trajectory_pb);
    // TODO(QiL): refine engage advice in open space trajectory optimizer.
    auto* engage_advice = ptr_trajectory_pb->mutable_engage_advice();

    // enable start auto from open_space planner.
    if (injector_->vehicle_state()->driving_mode() !=
        Chassis::DrivingMode::Chassis_DrivingMode_COMPLETE_AUTO_DRIVE) {
      engage_advice->set_advice(EngageAdvice::READY_TO_ENGAGE);
      engage_advice->set_reason(
          "Ready to engage when staring with OPEN_SPACE_PLANNER");
    } else {
      engage_advice->set_advice(EngageAdvice::KEEP_ENGAGED);
      engage_advice->set_reason("Keep engage while in parking");
    }
    // TODO(QiL): refine the export decision in open space info
    ptr_trajectory_pb->mutable_decision()
        ->mutable_main_decision()
        ->mutable_parking()
        ->set_status(MainParking::IN_PARKING);
    ptr_trajectory_pb->set_trajectory_type(
        frame_->reference_line_info().front().trajectory_type());

    if (FLAGS_enable_record_debug) {
      // ptr_debug->MergeFrom(frame_->open_space_info().debug_instance());
      frame_->mutable_open_space_info()->RecordDebug(ptr_debug);
      if (local_view_->GetFunctionManagerIn()->fct_avp_in().sys_run_state() ==
          functionmanager::AvpFctIn::CRUSING) {
        ptr_debug->MergeFrom(best_ref_info->debug());
      }
      ADEBUG << "Open space debug information added!";
      OpenSpaceChart openspace_chart(frame_.get(), injector_);
      openspace_chart.ExportOpenSpaceChart(ptr_trajectory_pb->debug(),
                                           *ptr_trajectory_pb, ptr_debug);
    }
    ptr_trajectory_pb->mutable_latency_stats()->MergeFrom(
        frame_->open_space_info().latency_stats());
    last_publishable_trajectory_ = std::make_unique<PublishableTrajectory>(
        current_time_stamp, publishable_trajectory);
  } else {
    const auto* target_ref_info = frame_->FindTargetReferenceLineInfo();
    if (best_ref_info == nullptr) {
      const std::string msg = "planner failed to make a driving plan";
      AERROR << msg;
      if (last_publishable_trajectory_) {
        last_publishable_trajectory_->clear();
      }
      return Status(ErrorCode::CORE_PLANNING_SOLVEFAIL_ERROR, msg);
    }
    // Store current frame stitched path for possible speed fallback in next
    // frames
    auto current_frame_planned_path = std::make_shared<DiscretizedPath>();
    for (const auto& trajectory_point : stitching_trajectory) {
      current_frame_planned_path->push_back(trajectory_point.path_point());
    }
    const auto& best_ref_path = best_ref_info->path_data().discretized_path();
    const auto best_ref_path_size = best_ref_path.size();
    for (size_t i = 1; i < best_ref_path_size; i++) {
      current_frame_planned_path->push_back(best_ref_path.at(i));
    }
    // std::copy(best_ref_path.begin() + 1, best_ref_path.end(),
    // std::back_inserter(*current_frame_planned_path));
    frame_->set_current_frame_planned_path(current_frame_planned_path);
    if (FLAGS_enable_record_debug) {
      // ptr_debug->Swap(best_ref_info->mutable_debug());
      ptr_debug->MergeFrom(best_ref_info->debug());
    }
    if (FLAGS_export_chart) {
      PlanningChartDebug::ExportOnLaneChart(
          *best_ref_info,
          frame_->local_view().GetFunctionManagerIn()->ta_pilot_mode(),
          ptr_debug);
    } else {
      PlanningChartDebug::ExportReferenceLineDebug(frame_.get(), ptr_debug);
      // Export additional ST-chart for failed lane-change speed planning
      const auto* failed_ref_info = frame_->FindFailedReferenceLineInfo();
      if (failed_ref_info != nullptr) {
        PlanningChartDebug::ExportFailedLaneChangeSTChart(
            *failed_ref_info, failed_ref_info->debug(),
            frame_->local_view().GetFunctionManagerIn()->ta_pilot_mode(),
            ptr_debug);
      }
    }
    if (FLAGS_enable_reference_line_debug) {
      reference_line_provider_->GetReferenceLineDebug(
          ptr_debug->mutable_planning_data()
              ->mutable_reference_line_debug_info());
    }

    ptr_debug->mutable_planning_data()->mutable_planning_status()->CopyFrom(
        injector_->planning_context()->planning_status());

    ptr_trajectory_pb->mutable_latency_stats()->MergeFrom(
        best_ref_info->latency_stats());
    // set right of way status
    ptr_trajectory_pb->set_right_of_way_status(
        best_ref_info->GetRightOfWayStatus());

    for (const auto& id : best_ref_info->TargetLaneId()) {
      ptr_trajectory_pb->add_lane_id()->CopyFrom(id);
    }

    for (const auto& id : target_ref_info->TargetLaneId()) {
      ptr_trajectory_pb->add_target_lane_id()->CopyFrom(id);
    }

    ptr_trajectory_pb->set_trajectory_type(best_ref_info->trajectory_type());

    if (FLAGS_enable_rss_info) {
      *ptr_trajectory_pb->mutable_rss_info() = best_ref_info->rss_info();
    }

    last_publishable_trajectory_ = std::make_unique<PublishableTrajectory>(
        current_time_stamp, best_ref_info->trajectory());

    last_publishable_trajectory_->PrependTrajectoryPoints(
        {stitching_trajectory.begin(), stitching_trajectory.end() - 1});

    last_publishable_trajectory_->PopulateTrajectoryProtobuf(ptr_trajectory_pb);

    AdjuestTrajectoryHeading(best_ref_info, ptr_trajectory_pb);

    last_vehicle_state_ =
        std::make_unique<VehicleState>(*injector_->vehicle_state());
    best_ref_info->ExportEngageAdvice(
        ptr_trajectory_pb->mutable_engage_advice(),
        injector_->planning_context());
  }

  best_ref_info->ExportDecision(
      ptr_trajectory_pb->mutable_decision(), injector_->planning_context(),
      frame_->local_view().GetFunctionManagerIn()->ta_pilot_mode());
  auto* debug_chart_time =
      ptr_trajectory_pb->mutable_latency_stats()->add_task_stats();
  debug_chart_time->set_name("debug_chart");
  debug_chart_time->set_time_ms(
      (Clock::NowInSeconds() - timestamp_debug_chart) * 1000);
  return status;
}

bool OnLanePlanning::UpdateEHPData(
    const std::shared_ptr<TL::ehp::EHP>& ehp_message) {
  int counter = 0;
  local_view_constructor_.UpdateEHPData(ehp_message, &counter);
  received_ehp_counter_ = counter;
  return true;
}

bool OnLanePlanning::CheckPlanningConfig(const PlanningConfig& config) {
  if (!config.has_standard_planning_config()) {
    return false;
  }
  if (!config.standard_planning_config().has_planner_public_road_config()) {
    return false;
  }
  // TODO(All): check other config params
  return true;
}

void OnLanePlanning::GeneratePredictionThread() {
  common::sub_thread_name = "_prediction";
  FunctionStatistics done_guard("GeneratePredictionThread");
  pthread_setname_np(pthread_self(), "GPrediction");
  std::shared_ptr<LocalView> cur_local_view;
  uint64 last_ms(0);
  const unsigned gap(75);
  while (!is_prediction_thread_stop_) {
    if (!prediction_input_queue_.WaitDequeue(&cur_local_view) ||
        cur_local_view == nullptr) {
      continue;
    }

    hdmap::HDMapUtil::SetMapForPrediction(cur_local_view->GetHDMapPtr());
    std::shared_ptr<prediction::PredictionObstacles> prediction_obstacles =
        nullptr;
    //     std::make_shared<prediction::PredictionObstacles>();
    if (cur_local_view->HasArena()) {
      prediction_obstacles = common::memory::ArenaAdapter::CreateMessage<
          prediction::PredictionObstacles>(cur_local_view->GetArena());
    } else {
      prediction_obstacles =
          std::make_shared<prediction::PredictionObstacles>();
    }
    ADEBUG << "prediction start work "
           << cur_local_view->GetPerceptionObstacles()
                  ->header()
                  .ShortDebugString();
    predictor_->Proc(cur_local_view, prediction_obstacles.get());
    ADEBUG << "prediction end work";
    cur_local_view->SetPredictionObstaclesPtr(prediction_obstacles);
#ifndef FOR_BAIDU_SIMULATION
    // 为了保证频率大致稳定，策略是如果两帧调起 Europa 时间间隔小于 75 毫秒，则丢弃一帧；
    const auto now_ms = common::Time::Now().ToMillisecond();
    if (cur_local_view->GetFunctionManagerIn()->ta_pilot_mode() ==
            TL::functionmanager::AVP ||
        now_ms > (last_ms + gap)) {
      last_ms = now_ms;
      ADEBUG << " add one frame localview ,now_ms:" << now_ms
             << " , last_ms: " << last_ms;
      planning_input_queue_.ForceEnqueue(cur_local_view);
    } else {
      ADEBUG << " discard one frame localview ,now_ms:" << now_ms
             << " , last_ms: " << last_ms;
    }
#else
    planning_input_queue_.ForceEnqueue(cur_local_view);
#endif
  }
  AINFO << "GeneratePredictionThread stoped."
        << "\n is_prediction_thread_stop_ " << is_prediction_thread_stop_;
}

void OnLanePlanning::GeneratePlanningThread() {
  common::sub_thread_name = "_planning";
  FunctionStatistics done_guard("GeneratePlanningThread");
  pthread_setname_np(pthread_self(), "GenerPlanning");
  while (!is_planning_thread_stop_) {
    local_view_ = nullptr;
    if (!planning_input_queue_.WaitDequeue(&local_view_) ||
        local_view_ == nullptr) {
      continue;
    }
    double planning_cycle_start_time = common::Clock::NowInSeconds();
    auto status = Status::OK();
    ADEBUG
        << "Planning thread run once, localview perception header "
        << local_view_->GetPerceptionObstacles()->header().ShortDebugString();

    const auto& vehicle_state = local_view_->GetVehicleState();
    injector_->vehicle_state()->CopyFrom(*vehicle_state);
    can_hmi_nnp_avp_->ProcessFctInput(local_view_);
    hdmap::HDMapUtil::SetMapForPlanning(local_view_->GetHDMapPtr());
    pnc_map_->UpdateHDMap(hdmap::HDMapUtil::MapPtrForPlanning());
    const auto& fct_in = local_view_->GetFunctionManagerIn();
    functionmanager::AvpFctOut::FsmStageType last_stage_type =
        functionmanager::AvpFctOut::INIT;
    {
      std::lock_guard<std::mutex> lock(last_function_manager_out_mutex_);
      last_stage_type = last_function_manager_out_->avp_fct_out().stage_type();
    }
    // Temporary plan: adapt to Map_Fusion mode
    bool curret_mode_is_map_fusion = false;
    if (local_view_->GetFunctionManagerOut()->has_hdmap_sub_state()) {
      if (local_view_->GetFunctionManagerOut()->hdmap_sub_state() ==
          TL::functionmanager::HdmapSubState::LOCAL_HDMAP_TYPE) {
        pnc_map_->SetCurrentMachineMode(true);
      } else {
        pnc_map_->SetCurrentMachineMode(false);
        curret_mode_is_map_fusion =
            (local_view_->GetFunctionManagerOut()->hdmap_sub_state() ==
             TL::functionmanager::HdmapSubState::MAP_FUSION_TYPE);
      }
    } else {
      pnc_map_->SetCurrentMachineMode(false);
    }
    if (local_view_->HasMapMsg() && local_view_->GetMapMsg()->has_map_type() &&
        local_view_->GetMapMsg()->map_type() ==
            TL::navigation_hdmap::MapMsg_MapType_FUSION_NCP_MAP) {
      pnc_map_->SetCurrentMachineMode(true);
    }
    pnc_map_->UpdateReferenceLineInfoConfig(
        local_view_->GetFunctionManagerOut()->fsm_state(), last_stage_type,
        std::fabs(vehicle_state->linear_velocity()),
        fct_in->fct_nnp_in().longitud_ctrl_cruise_speedms(),
        curret_mode_is_map_fusion);
    reference_info_config_ = pnc_map_->GetReferenceLineInfoConfig();
    // auto ptr_trajectory_pb = std::make_shared<ADCTrajectory>();
    std::shared_ptr<ADCTrajectory> ptr_trajectory_pb = nullptr;
    if (local_view_->HasArena()) {
      ptr_trajectory_pb =
          common::memory::ArenaAdapter::CreateMessage<ADCTrajectory>(
              local_view_->GetArena());
    } else {
      ptr_trajectory_pb = std::make_shared<ADCTrajectory>();
    }
    if (local_view_->HasADCTrajectory() &&
        local_view_->GetADCTrajectory()->has_avp_to_hmi()) {
      ptr_trajectory_pb->mutable_avp_to_hmi()->CopyFrom(
          local_view_->GetADCTrajectory()->avp_to_hmi());
    }
    common::util::FillHeader("planning", ptr_trajectory_pb.get());
    ptr_trajectory_pb->mutable_header()->set_data_stamp(
        vehicle_state->timestamp());
    double start_timestamp = ptr_trajectory_pb->header().data_stamp();
    ptr_trajectory_pb->mutable_function_manager_in()->CopyFrom(
        *local_view_->GetFunctionManagerIn());

    // 记录runonce耗时
    if (local_view_->HasFunctionManagerOut()) {
      const auto& fct_out = local_view_->GetFunctionManagerOut();

      auto* runonce_latency =
          ptr_trajectory_pb->mutable_latency_stats()->add_task_stats();
      runonce_latency->set_name("RUN_ONCE");
      runonce_latency->set_time_ms(
          fct_out->nnp_fct_out().localview_time().run_once_time());
    }

    // 记录prediction的耗时
    const auto prediction_msg = local_view_->GetPredictionObstacles();
    double prediction_ms =
        (prediction_msg->end_timestamp() - prediction_msg->start_timestamp()) *
        1000;

    auto* prediction_latency =
        ptr_trajectory_pb->mutable_latency_stats()->add_task_stats();
    prediction_latency->set_name("PREDICTION");
    prediction_latency->set_time_ms(prediction_ms);

    if (!util::IsVehicleStateValid(*vehicle_state)) {
      const std::string msg =
          "Update VehicleState failed or the vehicle state is not valid.";
      AERROR << msg << injector_->vehicle_state()->ShortDebugString();
      ptr_trajectory_pb->mutable_decision()
          ->mutable_main_decision()
          ->mutable_not_ready()
          ->set_reason(msg);
      status =
          common::Status(ErrorCode::CORE_ONLANE_VEHICLESTATECHECK_ERROR, msg);
      status.Save(ptr_trajectory_pb->mutable_header()->mutable_status());
      PostProcess(false, start_timestamp, ptr_trajectory_pb);
      continue;
    }

    // construct localview
    if (!local_view_->HasHDMap()) {
      const std::string msg = "Donot have hdmap.";
      AERROR << msg;
      ptr_trajectory_pb->mutable_decision()
          ->mutable_main_decision()
          ->mutable_not_ready()
          ->set_reason(msg);
      status = common::Status(ErrorCode::CORE_ONLANE_NOHDMAP_ERROR, msg);
      status.Save(ptr_trajectory_pb->mutable_header()->mutable_status());
      PostProcess(false, start_timestamp, ptr_trajectory_pb);
      continue;
    }

    is_fsm_state_changed_ =
        local_view_->GetFunctionManagerOut()->fsm_sequence_num() !=
            pre_fsm_sequence_num_ ||
        !is_pre_reference_line_ready_;
    pre_fsm_sequence_num_ =
        local_view_->GetFunctionManagerOut()->fsm_sequence_num();
    if (common::Clock::NowInSeconds() - start_timestamp >
        FLAGS_message_latency_threshold) {
      ADEBUG << "start_timestamp is after system time by "
             << common::Clock::NowInSeconds() - start_timestamp << " secs";
    }
    ADEBUG << "vehicle state: " << FIXED << SETPRECISION(3)
           << vehicle_state->ShortDebugString();
    CheckRerouting();
    const auto ta_pilot_mode =
        local_view_->GetFunctionManagerIn()->ta_pilot_mode();
    bool is_change_mode = previous_ta_pilot_mode_ != ta_pilot_mode;
    ADEBUG << std::boolalpha << "ta_pilot_mode is [" << is_change_mode
           << "]changed.";
    previous_ta_pilot_mode_ = ta_pilot_mode;
    constexpr double kTimeOut = 1.0;
    if ((!FLAGS_is_record_replay && !FLAGS_is_mcap_replay &&
         planning_cycle_start_time - last_planning_end_timestamp_ > kTimeOut) ||
        (is_fsm_state_changed_ &&
         (is_change_mode ||
          TL::functionmanager::TaPilotMode::AVP == ta_pilot_mode))) {
      injector_->history()->Clear();
      // reserve avp status before clear planning status
      auto* planning_status =
          injector_->planning_context()->mutable_planning_status();
      const auto change_lane_status = planning_status->change_lane();
      planning_status->Clear();
      planning_status->mutable_change_lane()->CopyFrom(change_lane_status);
      injector_->frame_history()->Clear();
      reference_line_provider_->ClearHistory();
      last_publishable_trajectory_.reset();
      ADEBUG << "local view state has already changed, frame histroy is to be "
                "clear.";
    }
    // Update reference line provider and reset pull over if necessary
    reference_line_provider_->UpdateVehicleState(*injector_->vehicle_state());
    reference_line_provider_->UpdateFctIn(*fct_in);

    // process routing
    // if use ehp, last_routing_ != local_view_->GetRouting() doesn't mean
    // rerouting happened. Maybe planning module received extended routing
    // response. if use local hd,map, last_routing_ != local_view_->GetRouting()
    // means rerouting happened
    const auto current_routing = local_view_->GetRoutingResponse();
    const auto& fct_out = local_view_->GetFunctionManagerOut();
    bool is_not_need_routing_check =
        functionmanager::TaPilotMode::AVP != ta_pilot_mode &&
        fct_out->fsm_state() == functionmanager::MachineStateType::HDMAP_TYPE &&
        (fct_out->localization_maptype() ==
             navigation_hdmap::MapMsg::FUSION_NNP_MAP ||
         fct_out->localization_maptype() ==
             navigation_hdmap::MapMsg::FUSION_NCP_MAP);
    if (is_not_need_routing_check ||
        hdmap::PncMap::IsNewRouting(last_routing_, *current_routing)) {
      if (FLAGS_enable_planning_self_simulator) {
        self_simulator_.UpdateRoutingResponse(current_routing);
      }
      last_routing_.CopyFrom(*current_routing);
      pnc_map_->UpdateRoutingResponse(*current_routing);
      ADEBUG << "last_routing: " << last_routing_.ShortDebugString();
      reference_line_provider_->UpdateRoutingResponse(*current_routing);
      injector_->history()->Clear();
      auto* planning_status =
          injector_->planning_context()->mutable_planning_status();
      if (planning_status->has_change_lane() ||
          planning_status->has_open_space() ||
          planning_status->has_avp_status()) {
        const auto change_lane_status = planning_status->change_lane();
        const auto openspace_status = planning_status->open_space();
        const auto avp_status = planning_status->avp_status();
        const auto avp_to_hmi = planning_status->avp_to_hmi();
        planning_status->Clear();
        // AERROR << " planning status is cleaned";
        planning_status->mutable_change_lane()->CopyFrom(change_lane_status);
        planning_status->mutable_open_space()->CopyFrom(openspace_status);
        planning_status->mutable_avp_status()->CopyFrom(avp_status);
        planning_status->mutable_avp_to_hmi()->CopyFrom(avp_to_hmi);
      } else {
        injector_->history()->Clear();
        auto* planning_status =
            injector_->planning_context()->mutable_planning_status();
        planning_status->Clear();
      }
    }
    injector_->planning_context()
        ->mutable_planning_status()
        ->mutable_function_manager_out()
        ->CopyFrom(*local_view_->GetFunctionManagerOut());
    SetMapState(injector_->planning_context()
                    ->mutable_planning_status()
                    ->mutable_function_manager_out());
    reference_line_provider_->UpdateFctOut(
        *injector_->planning_context()
             ->mutable_planning_status()
             ->mutable_function_manager_out());

    // when rerouting, reference line might not be updated. In this case,
    // planning module maintains not-ready until be restarted.
    static bool failed_to_update_reference_line = false;
    failed_to_update_reference_line =
        (!reference_line_provider_->UpdatedReferenceLine());

    // early return when reference line fails to update after rerouting
    if (failed_to_update_reference_line) {
      const std::string msg =
          "Failed to update reference line after rerouting.";
      AERROR << msg;
      ptr_trajectory_pb->mutable_decision()
          ->mutable_main_decision()
          ->mutable_not_ready()
          ->set_reason(msg);
      status = common::Status(ErrorCode::CORE_ONLANE_REFLINEUPDATE_ERROR, msg);
      status.Save(ptr_trajectory_pb->mutable_header()->mutable_status());
      PostProcess(false, start_timestamp, ptr_trajectory_pb);
      continue;
    }

    // planning is triggered by prediction data, but we can still use an
    // estimated cycle time for stitching
    const double planning_cycle_time =
        1.0 / static_cast<double>(FLAGS_planning_loop_rate);

    bool is_forward = true;
    if (functionmanager::AVP ==
            local_view_->GetFunctionManagerIn()->ta_pilot_mode() &&
        nullptr != injector_ && nullptr != injector_->frame_history() &&
        nullptr != injector_->frame_history()->Latest()) {
      auto last_gear = injector_->frame_history()->Latest()->GetTargetGear();
      is_forward = soc::Chassis::GEAR_DRIVE == last_gear;
    }

    // first:  1->横向replan  2->纵向replan  3->横纵向replan
    std::pair<int, std::string> replan_reason;
    RepeatedPtrField<TrajectoryPoint> stitching_trajectory =
        TrajectoryStitcher::ComputeStitchingTrajectory(
            *injector_->vehicle_state(), last_vehicle_state_, start_timestamp,
            planning_cycle_time, FLAGS_trajectory_stitching_preserved_length,
            true, is_forward, *local_view_->GetFunctionManagerIn(),
            vehicle_model_config_, last_stage_type,
            last_publishable_trajectory_.get(), &replan_reason,
            force_replan_type_);

    CheckStitchPointValid(vehicle_param_.max_acceleration(),
                          vehicle_param_.max_deceleration(), is_forward,
                          &stitching_trajectory);
    const auto& init_point = *stitching_trajectory.rbegin();
    injector_->ego_info()->Update(init_point, *injector_->vehicle_state());
    const auto frame_num = static_cast<uint32_t>(seq_num_++);

    const double timestamp_init_frame = Clock::NowInSeconds();
    // create reference line and segment
    status = InitFrame(frame_num, init_point, *injector_->vehicle_state());
    frame_->SetStitchingTrajectoryPointsPtr(&stitching_trajectory);
    if (status.ok()) {
      injector_->ego_info()->CalculateFrontObstacleClearDistance(
          frame_->obstacles());
      is_pre_reference_line_ready_ = true;
    }
    auto* init_frame_latency =
        ptr_trajectory_pb->mutable_latency_stats()->add_task_stats();
    init_frame_latency->set_name("INIT_FRAME_" + std::to_string(frame_num));
    init_frame_latency->set_time_ms(
        (Clock::NowInSeconds() - timestamp_init_frame) * 1000);

    if (FLAGS_enable_record_debug) {
      frame_->RecordInputDebug(ptr_trajectory_pb->mutable_debug());
    }

    if (!status.ok()) {
      AERROR << status.ToString();
      is_pre_reference_line_ready_ = false;
      ptr_trajectory_pb->mutable_decision()
          ->mutable_main_decision()
          ->mutable_not_ready()
          ->set_reason(status.ToString());
      status.Save(ptr_trajectory_pb->mutable_header()->mutable_status());
      PostProcess(false, start_timestamp, ptr_trajectory_pb);
      continue;
    }
    const auto& driving_mode = frame_->vehicle_state().driving_mode();
    // 0->未replan   1->横向replan  2->纵向replan  3->横纵向replan
    uint32_t replan_type = 0;
    if (frame_ != nullptr && stitching_trajectory.size() == 1) {
      if (driving_mode == soc::Chassis::COMPLETE_MANUAL ||
          driving_mode == soc::Chassis::COMPLETE_AUTO_DRIVE ||
          driving_mode == soc::Chassis::EMERGENCY_MODE ||
          (driving_mode == soc::Chassis::AUTO_STEER_ONLY &&
           replan_reason.first == 1) ||
          (driving_mode == soc::Chassis::AUTO_SPEED_ONLY &&
           replan_reason.first == 2) ||
          replan_reason.first == 3) {
        frame_->SetIsLongitudinalReplan(true);
        frame_->SetIsLateralReplan(true);
        replan_type = 3;
      } else if (driving_mode == soc::Chassis::AUTO_STEER_ONLY) {
        frame_->SetIsLongitudinalReplan(true);
        replan_type = 2;
      } else if (driving_mode == soc::Chassis::AUTO_SPEED_ONLY) {
        frame_->SetIsLateralReplan(true);
        replan_type = 1;
      }
    }
    const double timestamp_traffic = Clock::NowInSeconds();
    for (auto& ref_line_info : *frame_->mutable_reference_line_info()) {
      TrafficDecider traffic_decider;
      traffic_decider.Init(traffic_rule_configs_);
      auto traffic_status =
          traffic_decider.Execute(frame_.get(), &ref_line_info, injector_);
      if (!traffic_status.ok()) {
        ref_line_info.SetDrivable(false);
        AWARN << "Reference line " << ref_line_info.Lanes().Id()
              << " traffic decider failed";
      }
    }
    auto* traffic_decider_latency =
        ptr_trajectory_pb->mutable_latency_stats()->add_task_stats();
    traffic_decider_latency->set_name("traffic_decider");
    traffic_decider_latency->set_time_ms(
        (Clock::NowInSeconds() - timestamp_traffic) * 1000);

    frame_->mutable_open_space_info()->set_global_publish_timestamp(
        ptr_trajectory_pb->header().data_stamp());
    status =
        Plan(start_timestamp, stitching_trajectory, ptr_trajectory_pb.get());
    auto change_lane_status = injector_->planning_context()
                                  ->mutable_planning_status()
                                  ->mutable_change_lane()
                                  ->status();
    ADEBUG << "current change lane status: " << change_lane_status;

    // for (const auto& p : ptr_trajectory_pb->trajectory_point()) {
    //   ADEBUG << p.DebugString();
    // }
    status.Save(ptr_trajectory_pb->mutable_header()->mutable_status());
    ADEBUG << "Plan status: "
           << ptr_trajectory_pb->header().status().DebugString();
    if (!status.ok()) {
      AERROR << "Planning failed:"
             << ptr_trajectory_pb->header().status().DebugString();
      ptr_trajectory_pb->mutable_decision()
          ->mutable_main_decision()
          ->mutable_not_ready()
          ->set_reason(status.ToString());
    }
    bool is_replan = stitching_trajectory.size() == 1;
    if (frame_->open_space_info().is_on_open_space_trajectory()) {
      // avp mode, parking state
      is_replan = false;
      replan_reason.second.clear();
      const auto* previous_frame = injector_->frame_history()->Latest();
      if (previous_frame != nullptr) {
        replan_reason.second =
            previous_frame->open_space_info().get_replan_reason();
        is_replan = !replan_reason.second.empty();
      }
    }
    ptr_trajectory_pb->set_is_replan(is_replan);
    ptr_trajectory_pb->set_replan_type(replan_type);
    if (ptr_trajectory_pb->is_replan()) {
      ptr_trajectory_pb->set_replan_reason(replan_reason.second);
    } else {
      ptr_trajectory_pb->set_replan_reason("no replan");
    }
    bool is_plan_run_ok = status.ok();
    PostProcess(is_plan_run_ok, planning_cycle_start_time, ptr_trajectory_pb);
  }
  AINFO << "GeneratePlanningThread stoped."
        << "\n is_planning_thread_stop_ " << is_planning_thread_stop_;
}

void OnLanePlanning::PostProcess(
    const bool is_plan_run_ok, const double& planning_cycle_start_time,
    const std::shared_ptr<ADCTrajectory>& ptr_trajectory_pb) {
  ptr_trajectory_pb->set_is_vehicle_reference_frame(
      injector_->vehicle_state()->is_vehicle_reference_frame());
  ptr_trajectory_pb->set_utm_zone_id(
      local_view_->GetLocalization()->pose().using_utm_zone());
  ptr_trajectory_pb->mutable_function_manager_out()->CopyFrom(
      injector_->planning_context()
          ->mutable_planning_status()
          ->function_manager_out());
  bool is_cruisestart_spd_ok = false;
  static constexpr double kCruiseStartMaxSpd = 4.4;
  if (injector_ != nullptr) {
    is_cruisestart_spd_ok =
        std::fabs(injector_->vehicle_state()->linear_velocity()) <
        kCruiseStartMaxSpd;
  }
  if (local_view_->HasFunctionManagerIn()) {
    const auto& avp_fct_in = local_view_->GetFunctionManagerIn()->fct_avp_in();
    if ((avp_fct_in.sys_mode() == functionmanager::AvpFctIn::LOCALIZATION ||
         avp_fct_in.sys_mode() ==
             functionmanager::AvpFctIn::LOCALIZATION_BACKGROUND ||
         avp_fct_in.sys_mode() == functionmanager::AvpFctIn::LAPA ||
         avp_fct_in.sys_mode() == functionmanager::AvpFctIn::NTP) &&
        ptr_trajectory_pb->function_manager_out().fsm_state() ==
            MachineStateType::HDMAP_AVP_TYPE) {
      ptr_trajectory_pb->mutable_function_manager_out()
          ->mutable_avp_fct_out()
          ->set_control_enable(is_plan_run_ok && is_cruisestart_spd_ok);
    }
  }

  // fct_fsm_nnp will judge whether timeout occurs using trajectory sequence_num.
  ptr_trajectory_pb->mutable_function_manager_out()->set_soc_2_fct_tbd_u32_02(
      ptr_trajectory_pb->header().seq());
  can_hmi_nnp_avp_->ProcessFctOutput(frame_.get(), pnc_map_, injector_,
                                     ptr_trajectory_pb);
  real_jerk_->UpdateAccData(local_view_, ptr_trajectory_pb);
  injector_->planning_context()
      ->mutable_planning_status()
      ->clear_function_manager_out();
  ptr_trajectory_pb->mutable_debug()
      ->mutable_planning_data()
      ->mutable_planning_status()
      ->clear_function_manager_out();
  if (is_plan_run_ok) {
    auto* ref_line_task =
        ptr_trajectory_pb->mutable_latency_stats()->add_task_stats();
    ref_line_task->set_time_ms(reference_line_provider_->LastTimeDelay() *
                               1000.0);
    ref_line_task->set_name("REFERENCE_LINE_PROVIDER");
    ADEBUG << "Planning latency: "
           << ptr_trajectory_pb->latency_stats().DebugString();
    if (frame_) {
      ptr_trajectory_pb->set_gear(frame_->GetTargetGear());
    }
  } else {
    GenerateStopTrajectory(*injector_->vehicle_state(),
                           ptr_trajectory_pb.get());
  }

  FillPlanningPb(ptr_trajectory_pb.get());
  if (frame_) {
    frame_->set_current_frame_planned_trajectory(ptr_trajectory_pb);
    if (FLAGS_enable_planning_smoother) {
      Smoother::Smooth(injector_->frame_history(), frame_.get(),
                       ptr_trajectory_pb.get());
    }
    ptr_trajectory_pb->mutable_path()->set_is_path_vaild(frame_->IsPathValid());
  }

  if (ptr_trajectory_pb->trajectory_point_size() >
      FLAGS_publish_trajectory_points_number) {
    auto* ptr_points = ptr_trajectory_pb->mutable_trajectory_point();
    ptr_points->erase(
        ptr_points->begin() + FLAGS_publish_trajectory_points_number,
        ptr_points->end());
  } else if (ptr_trajectory_pb->trajectory_point_size() == 0) {
    for (int i = 0; i < FLAGS_publish_trajectory_points_number; ++i) {
      ptr_trajectory_pb->add_trajectory_point()->CopyFrom(
          TrajectoryStitcher::ComputeTrajectoryPointFromVehicleState(
              i * FLAGS_fallback_time_unit, *injector_->vehicle_state()));
    }
  } else {
    int count = FLAGS_publish_trajectory_points_number -
                ptr_trajectory_pb->trajectory_point_size();
    double dt = 0.0;
    while (count != 0) {
      auto tmp_point = ptr_trajectory_pb->trajectory_point(
          ptr_trajectory_pb->trajectory_point_size() - 1);
      dt = tmp_point.relative_time() + FLAGS_fallback_time_unit;
      tmp_point.set_relative_time(dt);
      auto* ptr_points = ptr_trajectory_pb->add_trajectory_point();
      ptr_points->CopyFrom(tmp_point);
      count--;
    }
  }

#if TEST
  PlanningChartDebug::ExportTrajectoryDebug(frame_.get());
#endif
  ptr_trajectory_pb->set_received_ehp_counter(received_ehp_counter_);
#ifndef ISMDC
  if (local_view_->HasLaneMarkers() &&
      local_view_->GetLaneMarkers()->has_front_left_lane_marker() &&
      local_view_->GetLaneMarkers()->has_front_right_lane_marker()) {
    const auto& lane_marker = local_view_->GetLaneMarkers();
    const auto& front_left_lane_marker = lane_marker->front_left_lane_marker();
    const auto& front_right_lane_marker =
        lane_marker->front_right_lane_marker();
    const auto c2_average = (front_left_lane_marker.c2_curvature() +
                             front_right_lane_marker.c2_curvature()) /
                            2.0;
    uint32 c2_average_u32 = 0;
    *(float*)(&c2_average_u32) = static_cast<float>(c2_average);  // NOLINT
    const auto c0_average = (front_left_lane_marker.c0_position() +
                             front_right_lane_marker.c0_position()) /
                            2.0;
    uint32 c0_average_u32 = 0;
    *(float*)(&c0_average_u32) = static_cast<float>(c0_average);  // NOLINT
    auto* mutable_u32 = ptr_trajectory_pb->mutable_soc_to_fct_bus()
                            ->mutable_soc_to_fct_bus_u32();
    mutable_u32->set_reserved01(c0_average_u32);
    mutable_u32->set_reserved02(c2_average_u32);
    // 临时代码，0301前删除
    ptr_trajectory_pb->set_total_path_time(c0_average);
    ptr_trajectory_pb->set_utm2gcs_heading_offset(c2_average);
  } else {
    auto* mutable_u32 = ptr_trajectory_pb->mutable_soc_to_fct_bus()
                            ->mutable_soc_to_fct_bus_u32();
    mutable_u32->set_reserved01(0);
    mutable_u32->set_reserved02(0);
    // 临时代码，0301前删除
    ptr_trajectory_pb->set_total_path_time(0.0);
    ptr_trajectory_pb->set_utm2gcs_heading_offset(0.0);
  }
#endif
  ptr_trajectory_pb->set_utm2gcs_heading_offset(0.0);
#ifdef ISMDC
  if (ptr_trajectory_pb->function_manager_out().fsm_state() ==
          MachineStateType::HDMAP_TYPE &&
      local_view_->GetLocalization()->pose().has_gcj02() &&
      !ptr_trajectory_pb->is_vehicle_reference_frame()) {
    ptr_trajectory_pb->set_utm2gcs_heading_offset(
        -common::coordinate_convertor::HeadingError(
            static_cast<int>(
                local_view_->GetLocalization()->pose().using_utm_zone()),
            local_view_->GetLocalization()->pose().gcj02().x(),
            local_view_->GetLocalization()->pose().gcj02().y()));
  } else {
    ptr_trajectory_pb->set_utm2gcs_heading_offset(0.0);
  }
#endif
  const auto end_timestamp = Clock::NowInSeconds();
  const auto time_diff_ms = (end_timestamp - planning_cycle_start_time) * 1000;
  ptr_trajectory_pb->mutable_latency_stats()->set_total_time_ms(time_diff_ms);
  ptr_trajectory_pb->mutable_function_manager_in()
      ->mutable_header()
      ->set_data_stamp(planning_cycle_start_time);
  ptr_trajectory_pb->mutable_function_manager_out()
      ->mutable_nnp_fct_out()
      ->mutable_localview_time()
      ->set_last_planning_end_timestamp(last_planning_end_timestamp_);
  ptr_trajectory_pb->mutable_function_manager_out()
      ->mutable_nnp_fct_out()
      ->mutable_localview_time()
      ->set_last_planning_timeout(last_planning_timeout_);
  auto* reference_line_info_decider_status =
      ptr_trajectory_pb->mutable_latency_stats()->add_task_stats();
  reference_line_info_decider_status->set_name("REFERENCE_LINE_INFO_DECIDER");
  reference_line_info_decider_status->set_time_ms(
      reference_line_info_decider_use_time_);
  auto* lane_change_safety_status =
      ptr_trajectory_pb->mutable_latency_stats()->add_task_stats();
  lane_change_safety_status->set_name("LANE_CHANGE_SAFETY_DECIDER");
  lane_change_safety_status->set_time_ms(lane_change_safety_use_time_);

  // TODO(hy):  adapter 0124demo using location_Seq and valid_points_size as
  // drivingMode and sysModeCmd for function decider, 0-idle, 3-ADAS, 6-NNP,
  // 9-AVP
  const auto& ta_pilot_mode =
      local_view_->GetFunctionManagerIn()->ta_pilot_mode();
  ptr_trajectory_pb->set_driving_mode(static_cast<::google::protobuf::uint32>(
      ptr_trajectory_pb->is_vehicle_reference_frame()));
  if (ta_pilot_mode == functionmanager::AVP) {
    ptr_trajectory_pb->set_function_mode(9);
    SendTorqueLimitToControl(ptr_trajectory_pb);
  } else if (is_plan_run_ok && (ta_pilot_mode == functionmanager::ADAS ||
                                ta_pilot_mode == functionmanager::NNP)) {
    ptr_trajectory_pb->set_function_mode(6);
  }

  ADEBUG << "planning_end_time:" << FIXED << SETPRECISION(3) << end_timestamp
         << ", trajectory size : "
         << ptr_trajectory_pb->trajectory_point_size();
  AINFO << "------planner_period_end------seq "
        << ptr_trajectory_pb->header().seq() << ", run " << time_diff_ms
        << " ms, length " << ptr_trajectory_pb->total_path_length()
        << " m, planning_end_time:" << FIXED << SETPRECISION(3)
        << end_timestamp;
  local_view_->SetADCTrajectoryPtr(ptr_trajectory_pb);
  force_replan_type_ = frame_->GetForceRplanType();
  if (FLAGS_enable_planning_self_simulator) {
    self_simulator_.UpdatePlanning(ptr_trajectory_pb);
  }
  publish_queue_->WaitEnqueue(local_view_);
  // record in history
  auto* history = injector_->history();
  history->Add(ptr_trajectory_pb);
  if (frame_) {
    const uint32_t n = frame_->SequenceNum();
    injector_->frame_history()->Add(n, std::move(frame_));
  }
  last_planning_timeout_ =
      (Clock::NowInSeconds() - planning_cycle_start_time) * 1000;
  last_planning_end_timestamp_ = Clock::NowInSeconds();
}

void OnLanePlanning::SetMapState(functionmanager::FunctionManagerOut* to_fct) {
  auto adc_waypoint = pnc_map_->GetADCWaypoint();
  functionmanager::MapRoadType road_type = functionmanager::INITIAL_ROADTYPE;
  if (adc_waypoint.lane != nullptr) {
    auto type = adc_waypoint.lane->GetRoadType();
    ADEBUG << "RoadType: " << type;
    if (hdmap_roadtype_debounce_.DealDebounce(type == hdmap::Road::HIGHWAY ||
                                              type ==
                                                  hdmap::Road::CITY_HIGHWAY)) {
      road_type = functionmanager::NNP_ROADTYPE;
    } else {
      road_type = functionmanager::NCP_ROADTYPE;
    }
  } else {
    road_type = hdmap_roadtype_debounce_.LastStatus()
                    ? functionmanager::NNP_ROADTYPE
                    : functionmanager::NCP_ROADTYPE;
    ADEBUG << "adc_waypoint.lane == nullptr";
  }
  TL::navigation_hdmap::MapMsg_MapType localization_maptype =
      TL::navigation_hdmap::MapMsg_MapType_INVALID;
  to_fct->set_map_road_type(road_type);
  if (!to_fct->has_localization_maptype()) {
    switch (road_type) {
      case functionmanager::NNP_ROADTYPE:
        localization_maptype =
            TL::navigation_hdmap::MapMsg_MapType_FUSION_NNP_MAP;
        break;

      case functionmanager::NCP_ROADTYPE:
        // 状态2的子状态 a:integral_staticobj
        localization_maptype =
            TL::navigation_hdmap::MapMsg_MapType_FUSION_NCP_MAP;
        break;
      case functionmanager::INITIAL_ROADTYPE:
        localization_maptype =
            TL::navigation_hdmap::MapMsg_MapType_PERCEP_MAP;
        break;
    }
    to_fct->set_localization_maptype(localization_maptype);
    ADEBUG << " out end localization_maptype: "
           << to_fct->localization_maptype();
  }

  ADEBUG << " out end road_type: " << to_fct->map_road_type()
         << " has localization_maptype: " << to_fct->has_localization_maptype();
}

void OnLanePlanning::AdjuestTrajectoryHeading(
    const ReferenceLineInfo* best_ref_info,
    ADCTrajectory* const ptr_trajectory_pb) {
  if (best_ref_info == nullptr || ptr_trajectory_pb == nullptr ||
      local_view_ == nullptr || !local_view_->HasFunctionManagerIn() ||
      local_view_->GetFunctionManagerIn()->ta_pilot_mode() ==
          functionmanager::AVP) {
    return;
  }

  auto discretized_path = best_ref_info->path_data().discretized_path();
  if (discretized_path.empty()) {
    return;
  }

  double theta = 0.0;
  for (std::size_t i = 0; i + 1 < discretized_path.size(); ++i) {
    auto& current_point = discretized_path.at(i);
    auto& next_point = discretized_path.at(i + 1);
    theta = common::math::NormalizeAngle(common::math::Vec2d{
        next_point.x() - current_point.x(), next_point.y() - current_point.y()}
                                             .Angle());
    current_point.set_theta(theta);
  }
  discretized_path.back().set_theta(theta);

  auto* mutable_trajectory_point =
      ptr_trajectory_pb->mutable_trajectory_point();
  if (mutable_trajectory_point == nullptr) {
    return;
  }

  for (auto& trajectory_point : *mutable_trajectory_point) {
    if (common::math::double_type::DefinitelyGreaterEqual(
            trajectory_point.path_point().s(), 0.0)) {
      const auto path_point =
          discretized_path.Evaluate(trajectory_point.path_point().s());
      trajectory_point.mutable_path_point()->set_theta(path_point.theta());
    }
  }
}

void OnLanePlanning::SendTorqueLimitToControl(
    const std::shared_ptr<ADCTrajectory>& ptr_trajectory_pb) {
  if (ptr_trajectory_pb == nullptr ||
      frame_->get_parking_lot_vertices().size() < 4) {
    ADEBUG << "failed to send torque limit to control";
    return;
  }
  common::math::Vec2d vehicle_rear_axis_center{frame_->vehicle_state().x(),
                                               frame_->vehicle_state().y()};
  const double wheel_base = TL::common::VehicleConfigHelper::GetConfig()
                                .vehicle_param()
                                .wheel_base();
  common::math::Vec2d vehicle_front_axis_center =
      vehicle_rear_axis_center +
      wheel_base * common::math::Vec2d::CreateUnitVec2d(
                       frame_->vehicle_state().heading());
  common::math::Vec2d left_top = frame_->get_parking_lot_vertices().at(0);
  common::math::Vec2d right_top = frame_->get_parking_lot_vertices().at(3);
  Vec2d left_right_top_unit_vec = (right_top - left_top);
  left_right_top_unit_vec.Normalize();
  Vec2d front_axis_vec(vehicle_front_axis_center - left_top);
  static constexpr double kTorqueLimitUpperBoundray = 5;
  // initialize signal
  size_t upper_limit_signal = 0;
  size_t lower_limit_signal = 0;
  if (left_right_top_unit_vec.CrossProd(front_axis_vec) < 0) {
    lower_limit_signal = 1;
  }
  if (left_right_top_unit_vec.CrossProd(front_axis_vec) >
      kTorqueLimitUpperBoundray) {
    upper_limit_signal = 1;
  }
  ptr_trajectory_pb->mutable_soc_to_fct_bus()
      ->mutable_soc_to_fct_bus_u8()
      ->set_ldp_left_finish_flag(lower_limit_signal);
  ptr_trajectory_pb->mutable_soc_to_fct_bus()
      ->mutable_soc_to_fct_bus_u8()
      ->set_ldp_right_trigger_flag(upper_limit_signal);
}

}  // namespace planning
}  // namespace TL
