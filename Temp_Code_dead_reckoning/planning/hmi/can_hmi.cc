/******************************************************************************
 * Copyright 2022 The TL Authors. All Rights Reserved.
 *****************************************************************************/

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include <boost/algorithm/string.hpp>

#include "absl/strings/match.h"
#include "common/configs/config_gflags.h"
#include "common/configs/vehicle_config_helper.h"
#include "common/math/curve_fitting.h"
#include "common/math/double_type.h"
#include "common/math/vec2d.h"
#include "common/time/clock.h"
#include "common/util/util.h"
#include "planning/common/planning_gflags.h"
#include "planning/hmi/can_hmi.h"
#include "proto/common/types.pb.h"
#include "proto/common/vehicle_config.pb.h"
#include "proto/common/vehicle_signal.pb.h"
#include "proto/fsm/nnp_fct.pb.h"
#include "proto/fsm/soc_to_mcu.pb.h"
#include "proto/hmi/nnp.pb.h"
#include "proto/perception/perception_obstacle.pb.h"
#include "proto/planning/decision.pb.h"
#include "proto/planning/planning.pb.h"
#include "proto/planning/without_lane_follow.pb.h"
#include "proto/routing/routing.pb.h"
#include "proto/soc/chassis.pb.h"

namespace TL {
namespace planning {
using common ::math::double_type::DefinitelyLessEqual;
using TL::common::Clock;
using TL::common::VehicleConfigHelper;
using TL::common::math::Box2d;
using TL::common::math::Vec2d;
using TL::functionmanager::ChangeLaneInfor;
using TL::functionmanager::DclcAudioPlay;
using TL::functionmanager::LaneChangeDir;
using TL::functionmanager::LcAudioPlay;
using TL::functionmanager::NNPScenarios;
using TL::functionmanager::NNPSysState;
using TL::routing::ChangeLaneType;
// using common::util::DistanceXY;
using Eigen::Vector2d;

namespace {
constexpr double kMinLenToOddStart = 10.0;
constexpr double kMaxLenToOddStart = 100.0;
constexpr double kLaneReduceTimestamp = 16.0;  // s
constexpr double kToMainRoadTimestamp = 10.0;  // s
constexpr double kPlayTime = 3.0;              // s
constexpr double kEpsilon = 1e-6;
constexpr double kMinSecond = 5;  // s
constexpr int kMaxEventCnt = 5;
constexpr double kMinDistance = 0.1;  // 0.1m
constexpr double kThreshold = 50;     // m
constexpr double kToRampFailLen = 250.0;
constexpr double kTurnDist = 5.0;
constexpr double kTurnKappa = 0.1;
constexpr int32_t kHour2Sec = 3600;
constexpr int32_t kHour2Min = 60;
constexpr int32_t kDay2Hour = 24;
constexpr int32_t kHourDiff = 8;
constexpr double kAvoidDist = 5.0;
}  // namespace

void CanNnpHmi::Init() {
  if (!TL::common::GetProtoFromFile(FLAGS_hmi_config_file, &hmi_config_)) {
    AERROR << "Failed to load obs follow time config file "
           << FLAGS_hmi_config_file;
  }
  vehicle_param_ = VehicleConfigHelper::GetConfig().vehicle_param();
  lon_hmi_.Init(hmi_config_);
}

void CanNnpHmi::ProcessFctOutput(
    Frame* const frame, const std::shared_ptr<hdmap::PncMap>& pnc_map,
    const std::shared_ptr<DependencyInjector>& injector,
    const std::shared_ptr<ADCTrajectory>& trajectory_pb) {
  frame_ = frame;
  pnc_map_ = pnc_map;
  injector_ = injector;
  ptr_trajectory_pb_ = trajectory_pb;
  ResetFctHmiOut();
  CheckNnpActiveCondition();
  // MergeNnpActiveCondition();
  CheckLaneMsg();
  ChangeLaneOutputDecision();
  CheckLaneChangeWaningObs();
  CheckTakeOver();
  CheckScenariosOrAudioPlay();
  CtlLightLaneSplitAndMerge();
  ResetFctHmiOutByNNPSysState();
  UpdateNnpTrajectoryForHmi();
  UpdateLonFollowObsHighLight();
  DetectWhetherToExitFunction();
}

bool CanNnpHmi::CheckTraPointWhetherCrimping(
    const common::PathPoint& path_point) {
  const auto* best_ref_info = frame_->FindDriveReferenceLineInfo();
  if (best_ref_info == nullptr) {
    return false;
  }
  Vec2d position(path_point.x(), path_point.y());
  Vec2d vec_to_center((vehicle_param_.front_edge_to_center() -
                       vehicle_param_.back_edge_to_center()) /
                          2.0,
                      (vehicle_param_.left_edge_to_center() -
                       vehicle_param_.right_edge_to_center()) /
                          2.0);
  Vec2d center(position + vec_to_center.rotate(path_point.theta()));
  Box2d box(center, path_point.theta(), vehicle_param_.length(),
            vehicle_param_.width());
  SLBoundary tra_sl_boundary;
  if (!best_ref_info->reference_line().GetSLBoundary(box, &tra_sl_boundary)) {
    AERROR << "Failed to get ADC boundary from box: " << box.DebugString();
    return false;
  }
  double left_width = 0.0;
  double right_width = 0.0;
  if (best_ref_info->reference_line().GetLaneWidth(tra_sl_boundary.end_s(),
                                                   &left_width, &right_width)) {
    if (DefinitelyLessEqual(left_width, std::fabs(tra_sl_boundary.end_l())) ||
        DefinitelyLessEqual(right_width,
                            std::fabs(tra_sl_boundary.start_l()))) {
      return true;
    }
  }
  if (best_ref_info->reference_line().GetLaneWidth(tra_sl_boundary.start_s(),
                                                   &left_width, &right_width)) {
    if (DefinitelyLessEqual(left_width, std::fabs(tra_sl_boundary.end_l())) ||
        DefinitelyLessEqual(right_width,
                            std::fabs(tra_sl_boundary.start_l()))) {
      return true;
    }
  }
  return false;
}

bool CanNnpHmi::CheckTraWhetherCrimping() {
  bool is_crimping = false;
  if (frame_ == nullptr || ptr_trajectory_pb_ == nullptr) {
    AERROR << "ptr is nullptr";
    return is_crimping;
  }
  for (const auto& tra_point : ptr_trajectory_pb_->trajectory_point()) {
    if (tra_point.path_point().s() < kEpsilon) {
      continue;
    }
    if (CheckTraPointWhetherCrimping(tra_point.path_point())) {
      is_crimping = true;
      break;
    }
  }
  return is_crimping;
}

// NNP 激活条件
void CanNnpHmi::CheckNnpActiveCondition() {  // NOLINT
  if (frame_ == nullptr || pnc_map_ == nullptr) {
    AERROR << "frame or pnc_map ptr is nullptr";
    return;
  }
  auto* fct_out = ptr_trajectory_pb_->mutable_function_manager_out();
  is_lat_override_ = false;
  is_nnp_active_ = false;
  is_pilot_lat_override_ = false;
  if (frame_->local_view().HasFunctionManagerIn()) {
    const auto& nnp_sys_state = frame_->local_view()
                                    .GetFunctionManagerIn()
                                    ->fct_nnp_in()
                                    .nnp_sysstate();
    is_lat_override_ = (nnp_sys_state == NNPSysState::NNPS_OVERRIDE ||
                        nnp_sys_state == NNPSysState::NNPS_LAT_OVERRIDE);
    is_override_ =
        (is_lat_override_ || nnp_sys_state == NNPSysState::NNPS_LON_OVERRIDE);
    is_nnp_active_ =
        (is_lat_override_ || nnp_sys_state == NNPSysState::NNPS_ACTIVE ||
         nnp_sys_state == NNPSysState::NNPS_LON_OVERRIDE);

    const auto& adas_mode =
        frame_->local_view().GetFunctionManagerIn()->adas_mode();
    const auto& driver_mode =
        frame_->local_view().GetFunctionManagerIn()->driver_mode();
    const auto& ta_pilot_mode =
        frame_->local_view().GetFunctionManagerIn()->ta_pilot_mode();
    is_soc_pilot_active_ =
        ta_pilot_mode == functionmanager::TaPilotMode::ADAS &&
        adas_mode == functionmanager::AdasMode::PILOT;
    is_pilot_lat_override_ =
        driver_mode == functionmanager::DriveMode::ADAS_LGT_ACTIVE_LAT_OVERRIDE;
    is_pilot_override_ =
        is_pilot_lat_override_ ||
        driver_mode == functionmanager::DriveMode::ADAS_LAT_ACTIVE_LGT_OVERRIDE;
    static bool last_active_nnp_pilot = false;
    if (last_active_nnp_pilot && adas_mode == functionmanager::AdasMode::ACC) {
      ptr_trajectory_pb_->mutable_soc_to_fct_bus()
          ->mutable_soc_to_fct_bus_u8()
          ->set_no_lane_status(0x01);
    }
    last_active_nnp_pilot = is_nnp_active_ || is_soc_pilot_active_;
  }
  const auto* best_ref_info = frame_->DriveReferenceLineInfo();
  if (best_ref_info != nullptr) {
    fct_out->set_cur_pos_in_ref_projection(
        best_ref_info->AdcSlBoundary().end_s());
  }
}

void CanNnpHmi::DetectWhetherToExitFunction() {  // NOLINT
  if (ptr_trajectory_pb_ == nullptr || frame_ == nullptr ||
      pnc_map_ == nullptr) {
    AERROR << "ptr is nullptr";
    return;
  }
  static double start_exit_function_timtstamp = 0.0;
  static constexpr double kMinCur = 0.0001;
  static constexpr double kMaxCur = 0.0006;
  const auto& odd_info = ptr_trajectory_pb_->function_manager_out().odd_info();
  const auto* best_ref_info = frame_->DriveReferenceLineInfo();
  const auto& valid_routing = ptr_trajectory_pb_->function_manager_out()
                                  .nnp_fct_out()
                                  .nnp_activation_conditions()
                                  .valid_of_lane_routing();
  const auto& to_ramp_len = ptr_trajectory_pb_->mutable_function_manager_out()
                                ->nnp_fct_out()
                                .nnp_d_distance2_downramp_sg();
  auto* nnp_fct_out =
      ptr_trajectory_pb_->mutable_function_manager_out()->mutable_nnp_fct_out();
  bool is_active = is_nnp_active_ || is_soc_pilot_active_;
  if (is_active && valid_routing && best_ref_info != nullptr &&
      odd_info.type() == routing::LaneWaypointType::ODD_START &&
      odd_info.odd_type() == routing::LaneWaypoint::SPECIAL_AREA) {
    double check_start_len = kMaxLenToOddStart + FLAGS_function_quit_distance;
    double check_end_len = kMinLenToOddStart + FLAGS_function_quit_distance;
    nnp_fct_out->set_nnp_d_distance_outof_odd_sg(kEpsilon);
    double odd_remain_len = odd_info.to_end_len() - check_end_len;
    nnp_fct_out->set_nnp_d_distance_into_odd_sg(
        static_cast<float>(odd_remain_len));
    const auto& ref_points = best_ref_info->reference_line().reference_points();
    const auto& adc_sl_boundary = best_ref_info->AdcSlBoundary();
    double ref_len =
        best_ref_info->reference_line().Length() - adc_sl_boundary.end_s();
    if (!ref_points.empty() && ref_len > kEpsilon &&
        odd_info.to_end_len() <= check_start_len &&
        odd_info.to_end_len() > 0.0) {
      double length = 0.0;
      auto adc_index = best_ref_info->reference_line().GetNearestReferenceIndex(
          adc_sl_boundary.end_s());
      double min_cur = std::numeric_limits<double>::max();
      double to_min_cur_len = std::numeric_limits<double>::max();
      for (size_t i = adc_index; i + 1 < ref_points.size(); ++i) {
        length +=
            common::util::DistanceXY(ref_points.at(i), ref_points.at(i + 1));
        double curvature = ref_points.at(i).kappa();
        if (odd_remain_len < 50.0 && length <= odd_remain_len &&
            odd_remain_len <= check_start_len) {
          if (std::fabs(curvature) < std::fabs(min_cur) ||
              std::fabs(curvature) < kMinCur) {
            min_cur = curvature;
            to_min_cur_len = length;
          }
        }
      }
      nnp_fct_out->set_nnp_d_distance_into_odd_sg(static_cast<float>(std::max(
          0.2,
          std::min(to_min_cur_len, odd_info.to_end_len() - check_end_len))));
      static constexpr double kMinCurLen = 3.0;
      if (to_min_cur_len < kMinCurLen) {
        start_exit_function_timtstamp = Clock::NowInSeconds();
      }
    } else if (ref_len < kEpsilon) {
      start_exit_function_timtstamp = Clock::NowInSeconds();
    }
  }
  // 收费站在主路提前8秒时距退出功能
  static bool in_mainroad_exit = false;
  static constexpr double kMainRoadQuitDistance = 250.0;
  bool odd_in_mainroad = (to_ramp_len < 0.1);
  if (odd_in_mainroad && pnc_map_->AdcInMainRoad() &&
      odd_info.type() == routing::LaneWaypointType::ODD_START &&
      odd_info.odd_type() == routing::LaneWaypoint::SPECIAL_AREA &&
      best_ref_info != nullptr) {
    double max_dis =
        std::max(0.0, odd_info.to_end_len() - kMainRoadQuitDistance);
    nnp_fct_out->set_nnp_d_distance_outof_odd_sg(kEpsilon);
    nnp_fct_out->set_nnp_d_distance_into_odd_sg(static_cast<float>(max_dis));
    const auto& ref_points = best_ref_info->reference_line().reference_points();
    const auto& adc_sl_boundary = best_ref_info->AdcSlBoundary();
    if (best_ref_info->reference_line().Length() > 0) {
      auto adc_index = best_ref_info->reference_line().GetNearestReferenceIndex(
          adc_sl_boundary.end_s());
      double curvature = ref_points.at(adc_index).kappa();
      if (is_active && odd_info.to_end_len() < kMainRoadQuitDistance &&
          std::fabs(curvature) < kMaxCur) {
        in_mainroad_exit = true;
        start_exit_function_timtstamp = Clock::NowInSeconds();
      } else if (odd_info.to_end_len() > kMainRoadQuitDistance) {
        in_mainroad_exit = false;
      }
      if (in_mainroad_exit) {
        start_exit_function_timtstamp = Clock::NowInSeconds();
      }
    }
  } else {
    in_mainroad_exit = false;
  }
  bool is_layer_quit = FLAGS_switch_layer_function_quit_distance &&
                       odd_info.next_info() == "QuitImmediate";
  if (is_layer_quit &&
      (odd_info.next_type() == routing::LaneWaypointType::ODD_END ||
       (odd_info.next_type() == routing::LaneWaypointType::ODD_START &&
        odd_info.to_next_len() > 1.0 &&
        odd_info.to_next_len() < FLAGS_layer_function_quit_distance))) {
    start_exit_function_timtstamp = Clock::NowInSeconds();
  }
  if (is_layer_quit &&
      odd_info.next_type() == routing::LaneWaypointType::ODD_START) {
    nnp_fct_out->set_nnp_d_distance_outof_odd_sg(kEpsilon);
    nnp_fct_out->set_nnp_d_distance_into_odd_sg(
        static_cast<float>(odd_info.to_next_len()));
  }
  static constexpr double kMinExitFunctionTimestamp = 2.0;
  if (Clock::NowInSeconds() - start_exit_function_timtstamp <
      kMinExitFunctionTimestamp) {
    static const uint32_t kExitFunction = 0x100;
    uint32_t soc_03_val = 0x00;
    auto* fct_out = ptr_trajectory_pb_->mutable_function_manager_out();
    if (fct_out->has_soc_2_fct_tbd_u32_03()) {
      soc_03_val = fct_out->soc_2_fct_tbd_u32_03();
    }
    fct_out->set_soc_2_fct_tbd_u32_03(soc_03_val | kExitFunction);
    fct_out->mutable_nnp_fct_out()
        ->mutable_nnp_activation_conditions()
        ->set_vehicle_in_hdmap(false);
  }
}

void CanNnpHmi::MergeNnpActiveCondition() {
  if (ptr_trajectory_pb_ == nullptr || frame_ == nullptr) {
    AERROR << "ptr is nullptr";
    return;
  }
  auto* fct_out = ptr_trajectory_pb_->mutable_function_manager_out();
  const auto& inside_active_condition =
      fct_out->nnp_fct_out().nnp_statechange_conditions();
  auto* nnp_active_condition =
      fct_out->mutable_nnp_fct_out()->mutable_nnp_activation_conditions();
  if (inside_active_condition.has_is_change_dueto_internalreasons() &&
      inside_active_condition.is_change_dueto_internalreasons() &&
      FLAGS_enable_odd_area_internal_to_perception &&
      fct_out->fsm_state() ==
          functionmanager::MachineStateType::PERCEPTION_TYPE) {
    nnp_active_condition->set_valid_of_lane_localization(true);
    nnp_active_condition->set_vehicle_in_hdmap(true);
    nnp_active_condition->set_valid_of_lane_routing(true);
    nnp_active_condition->set_vehicle_not_in_forbidlane(true);
    nnp_active_condition->set_vehicle_not_in_otherforbidarea(true);
    if (inside_active_condition.location_err_state() == 2) {
      fct_out->mutable_nnp_fct_out()
          ->mutable_nnp_activation_conditions()
          ->set_valid_of_lane_localization(false);
    }
  }
}

void CanNnpHmi::CheckSplitLaneChangeTakeOver() {
  if (ptr_trajectory_pb_ == nullptr || pnc_map_ == nullptr) {
    AERROR << "ptr is nullptr";
    return;
  }
  const auto& lane_change_info =
      ptr_trajectory_pb_->function_manager_out().hmi_lane_change_debug();
  bool is_split_not_neighbor = false;
  is_split_not_neighbor = pnc_map_->GetIsSplitLaneAndNotNeighbor();
  bool is_lane_split_take_over =
      is_split_not_neighbor &&
      (lane_change_info.hmi_lane_change_status() ==
           functionmanager::HmiChangeLaneStatus::HMI_IN_CHANGE_LANE ||
       lane_change_info.hmi_lane_change_status() ==
           functionmanager::HmiChangeLaneStatus::HMI_CHANGE_LANE_START);
  static double last_take_ovet_timestamp = 0.0;
  if (is_lane_split_take_over) {
    last_take_ovet_timestamp = Clock::NowInSeconds();
  }
  if (Clock::NowInSeconds() - last_take_ovet_timestamp < 2.0) {
    auto* nnp_fct_out = ptr_trajectory_pb_->mutable_function_manager_out()
                            ->mutable_nnp_fct_out();
    nnp_fct_out->set_nnp_rino_status(
        functionmanager::NNPRinoStatus::TASK_TAKE_OVER_REQ);
  }
}

void CanNnpHmi::LaneWidthTakeOver() {
  if (frame_ == nullptr) {
    return;
  }
  const auto* best_ref_info = frame_->FindDriveReferenceLineInfo();
  if (best_ref_info != nullptr) {
    const auto& adc_sl_boundary = best_ref_info->AdcSlBoundary();
    double left_width = 0.0;
    double right_width = 0.0;
    double lane_width = 0.0;
    static constexpr double kBase = 5.0;
    const auto& vehicle_state = frame_->local_view().GetVehicleState();
    double cur_speed = vehicle_state->linear_velocity();
    double len = std::min(
        cur_speed * kPlayTime,
        best_ref_info->reference_line().Length() - adc_sl_boundary.end_s());
    static double last_lane_width_timestamp = 0.0;
    if (best_ref_info->reference_line().Length() > adc_sl_boundary.end_s()) {
      double s = adc_sl_boundary.end_s();
      while (s < len) {
        if (best_ref_info->reference_line().GetLaneWidth(s, &left_width,
                                                         &right_width)) {
          lane_width = left_width + right_width;
          if (lane_width > kBase) {
            last_lane_width_timestamp = Clock::NowInSeconds();
            break;
          }
        }
        s += kBase;
      }
    }
    if (Clock::NowInSeconds() - last_lane_width_timestamp < 2.0) {
      auto* nnp_fct_out = ptr_trajectory_pb_->mutable_function_manager_out()
                              ->mutable_nnp_fct_out();
      nnp_fct_out->set_dclc_audio_play(
          functionmanager::DclcAudioPlay::CAREFUL_DRIVING);
    }
  }
}

void CanNnpHmi::RampDowngradeTakeOver() {
  if (frame_ == nullptr || ptr_trajectory_pb_ == nullptr) {
    AERROR << "ptr is nullptr";
    return;
  }
  static functionmanager::TaPilotMode last_ta_pilot_mode =
      functionmanager::TaPilotMode::NO_CONTROL;
  static double last_to_ramp_len = 0.0;
  static double last_takeover_timestamp = -1.0;
  static double last_downgrade_timestamp = -1.0;
  static uint32_t last_to_ramp_lane_count = 0;
  auto* nnp_fct_out =
      ptr_trajectory_pb_->mutable_function_manager_out()->mutable_nnp_fct_out();
  const auto& ta_pilot_mode =
      ptr_trajectory_pb_->function_manager_in().ta_pilot_mode();
  const auto& to_ramp_lane_count = nnp_fct_out->to_ramp_lane_count();
  double to_ramp_len = nnp_fct_out->nnp_d_distance2_downramp_sg();
  if (last_ta_pilot_mode == functionmanager::TaPilotMode::NNP &&
      ta_pilot_mode == functionmanager::TaPilotMode::ADAS &&
      last_to_ramp_len < 1005.0 && last_to_ramp_len > 0.1) {
    if (last_to_ramp_lane_count > 0) {
      last_takeover_timestamp = Clock::NowInSeconds();
    } else if (last_to_ramp_len > 500.0) {
      last_downgrade_timestamp = Clock::NowInSeconds();
    }
  }
  if (last_downgrade_timestamp > 0.0 &&
      Clock::NowInSeconds() - last_downgrade_timestamp > 2.0) {
    last_takeover_timestamp = Clock::NowInSeconds();
    last_downgrade_timestamp = -1.0;
  }
  last_ta_pilot_mode = ta_pilot_mode;
  last_to_ramp_lane_count = to_ramp_lane_count;
  last_to_ramp_len = to_ramp_len;
  const auto take_over_timestamp =
      Clock::NowInSeconds() - last_takeover_timestamp;
  if (take_over_timestamp > 3.0 && take_over_timestamp < 3.5) {
    nnp_fct_out->set_nnp_rino_status(
        functionmanager::NNPRinoStatus::TASK_TAKE_OVER_REQ);
    nnp_fct_out->set_dclc_audio_play(functionmanager::DclcAudioPlay::DCLC_NONE);
  }
}

void CanNnpHmi::RampExitTakeOver() {
  if (frame_ == nullptr || ptr_trajectory_pb_ == nullptr) {
    AERROR << "ptr is nullptr";
    return;
  }
  auto* nnp_fct_out =
      ptr_trajectory_pb_->mutable_function_manager_out()->mutable_nnp_fct_out();
  const auto to_ramp_lane_count = nnp_fct_out->to_ramp_lane_count();
  double to_ramp_len = nnp_fct_out->nnp_d_distance2_downramp_sg();
  bool is_take_over =
      ((to_ramp_lane_count <= 2 && to_ramp_len < 200.0 * to_ramp_lane_count) ||
       (to_ramp_lane_count == 3 && to_ramp_len < 550.0)) &&
      to_ramp_len > 0.1;

  // double remain_len =
  //     ptr_trajectory_pb_->function_manager_out().adc_passage_remain_len();
  // const auto& odd_type =
  //     ptr_trajectory_pb_->function_manager_out().odd_info().type();
  // is_take_over =
  //     is_take_over || (remain_len < 100 && remain_len > 0.1 &&
  //                      odd_type == routing::LaneWaypointType::NORMAL);

  const auto& fsm_state =
      ptr_trajectory_pb_->function_manager_out().fsm_state();
  static double last_is_tunnel_timestamp = 0.0;
  static bool last_is_tunnel = false;
  const auto& is_tunnel =
      ptr_trajectory_pb_->function_manager_out().adc_is_in_tunnel();
  if (last_is_tunnel && !is_tunnel) {
    last_is_tunnel_timestamp = Clock::NowInSeconds();
  }
  last_is_tunnel = is_tunnel;
  // 出隧道后，降纯视觉并且前方要下匝道 或者 距离匝道口的距离小于500m
  bool is_check_ok =
      to_ramp_len > 0.1 &&
      (fsm_state == functionmanager::MachineStateType::PERCEPTION_TYPE ||
       to_ramp_len < 500.0);
  is_take_over = is_take_over ||
                 (is_nnp_active_ && is_check_ok &&
                  (Clock::NowInSeconds() - last_is_tunnel_timestamp < 2.0));
  const auto& odd_info = ptr_trajectory_pb_->function_manager_out().odd_info();
  bool is_odd_start =
      (odd_info.type() == routing::LaneWaypointType::ODD_START ||
       odd_info.type() == routing::LaneWaypointType::ROUTE_BREAK);
  // 匝道口前降级，报接管提醒
  bool is_fail_to_ramp = is_odd_start && odd_info.to_end_len() > 1.0 &&
                         to_ramp_len > odd_info.to_end_len() &&
                         to_ramp_len < (kToRampFailLen + 50.0);
  is_take_over = is_take_over || is_fail_to_ramp;
  if (is_take_over) {
    nnp_fct_out->set_nnp_rino_status(
        functionmanager::NNPRinoStatus::TASK_TAKE_OVER_REQ);
    nnp_fct_out->set_dclc_audio_play(functionmanager::DclcAudioPlay::DCLC_NONE);
  }
}

void CanNnpHmi::LocationFaultForTakeOver() {
  if (ptr_trajectory_pb_ == nullptr || frame_ == nullptr ||
      !frame_->local_view().HasMapMsg() || pnc_map_ == nullptr) {
    return;
  }
  const auto& map_msg = frame_->local_view().GetMapMsg();
  const auto& to_ramp_len = ptr_trajectory_pb_->function_manager_out()
                                .nnp_fct_out()
                                .nnp_d_distance2_downramp_sg();
  static uint32 last_map_fault_level = 0;
  const auto& map_fault_level = map_msg->fault_level();
  bool is_take_over =
      (last_map_fault_level != 1 && map_fault_level == 1 &&
       ((to_ramp_len > 0.1 && to_ramp_len < 200.0) || soon_in_carriageWay_));
  last_map_fault_level = map_fault_level;
  auto* nnp_fct_out =
      ptr_trajectory_pb_->mutable_function_manager_out()->mutable_nnp_fct_out();
  if (is_take_over && nnp_fct_out != nullptr) {
    nnp_fct_out->set_nnp_rino_status(
        functionmanager::NNPRinoStatus::TASK_TAKE_OVER_REQ);
    nnp_fct_out->set_dclc_audio_play(functionmanager::DclcAudioPlay::DCLC_NONE);
  }
}

void CanNnpHmi::LongDirTakeOver() {
  if (frame_ == nullptr || ptr_trajectory_pb_ == nullptr) {
    AERROR << "ptr is nullptr";
    return;
  }
  // 纵向接管
  static constexpr uint32_t long_take_over = 0x01;
  static constexpr int kPlayLonTakeoverTime = 30;  // 10s
  int lon_fallback_stable = std::numeric_limits<int>::max();
  static constexpr int LonMergeStopFallbackStable = 5;
  static constexpr int LonStopFallbackStable = 30;
  static int play_lon_takeover_count = 0;
  static int speed_fallback_cnt = 0;
  static int merge_stop_fallback_cnt = 0;
  static int stop_fallback_cnt = 0;
  const auto driving_mode = frame_->local_view().GetChassis()->driving_mode();
  const auto need_send_lon_takeover =
      (driving_mode == soc::Chassis::COMPLETE_AUTO_DRIVE ||
       driving_mode == soc::Chassis::AUTO_SPEED_ONLY);
  static constexpr double kLonTakeOverMinSpeed = 3.0;
  const auto& vehicle_state = frame_->local_view().GetVehicleState();
  double cur_speed = vehicle_state->linear_velocity();
  if (TL::common::math::double_type::DefinitelyLessEqual(
          vehicle_state->linear_acceleration(), -3) &&
      frame_->GetIsCIPVSpeedFallback()) {
    lon_fallback_stable = 2;
  } else if (TL::common::math::double_type::DefinitelyLessEqual(
                 vehicle_state->linear_acceleration(), -2) &&
             frame_->GetIsCIPVSpeedFallback()) {
    lon_fallback_stable = 3;
  } else if (TL::common::math::double_type::DefinitelyLessEqual(
                 vehicle_state->linear_acceleration(), -1)) {
    lon_fallback_stable = 4;
  }

  if (frame_->GetIsSpeedFallback() && need_send_lon_takeover &&
      cur_speed > kLonTakeOverMinSpeed) {
    play_lon_takeover_count = kPlayLonTakeoverTime;
    ++speed_fallback_cnt;
  } else {
    speed_fallback_cnt = 0;
  }
  if (frame_->GetIsMergeStopFallback() && need_send_lon_takeover &&
      cur_speed > kLonTakeOverMinSpeed) {
    play_lon_takeover_count = kPlayLonTakeoverTime;
    ++merge_stop_fallback_cnt;

  } else {
    merge_stop_fallback_cnt = 0;
  }
  if (frame_->GetIsStopFallback() && need_send_lon_takeover &&
      cur_speed > kLonTakeOverMinSpeed) {
    play_lon_takeover_count = kPlayLonTakeoverTime;
    ++stop_fallback_cnt;

  } else {
    stop_fallback_cnt = 0;
  }

  auto* fct_out = ptr_trajectory_pb_->mutable_function_manager_out();
  const auto origin = fct_out->soc_2_fct_tbd_u32_03();
  if (play_lon_takeover_count > 0 &&
      (speed_fallback_cnt >= lon_fallback_stable ||
       merge_stop_fallback_cnt >= LonMergeStopFallbackStable ||
       stop_fallback_cnt >= LonStopFallbackStable)) {
    play_lon_takeover_count--;
    fct_out->set_soc_2_fct_tbd_u32_03(origin | long_take_over);
  }
}

bool CanNnpHmi::IsNviLayer() {
  if (ptr_trajectory_pb_ == nullptr) {
    return false;
  }
  const auto& odd_info = ptr_trajectory_pb_->function_manager_out().odd_info();
  bool is_layer_nvi =
      odd_info.has_next_info() && (odd_info.next_info() == "Navi_MostLeft" ||
                                   odd_info.next_info() == "Navi_MostRight" ||
                                   odd_info.next_info() == "Navi_LR_2");
  return is_layer_nvi;
}

void CanNnpHmi::MergeLaneTakeOver() {
  if (injector_ == nullptr || ptr_trajectory_pb_ == nullptr ||
      frame_ == nullptr || !frame_->local_view().HasVehicleState() ||
      pnc_map_ == nullptr || !frame_->local_view().HasFunctionManagerIn()) {
    return;
  }
  static int play_count = 0;
  bool is_take_over = false;
  static bool last_is_navigation = false;
  static double last_navigation_time_stamp = -1;
  const auto& change_lane =
      injector_->planning_context()->planning_status().change_lane();
  const auto& hmi_change_lane_status = change_lane.hmi_change_lane_status();
  const auto& change_lane_reason = change_lane.change_lane_reason();
  auto* nnp_fct_out =
      ptr_trajectory_pb_->mutable_function_manager_out()->mutable_nnp_fct_out();
  bool is_navigation =
      (hmi_change_lane_status == functionmanager::HMI_CHANGE_LANE_START &&
       change_lane_reason ==
           functionmanager::HmiChangeLaneReason::HMI_NAVIGATION);
  if (last_is_navigation != is_navigation && is_navigation) {
    last_navigation_time_stamp = Clock::NowInSeconds();
  } else if (!is_navigation) {
    last_is_navigation = false;
    last_navigation_time_stamp = -1;
  }
  static constexpr double kMinTakeoverTime = 20.0;
  if (last_navigation_time_stamp > 0 &&
      Clock::NowInSeconds() - last_navigation_time_stamp > kMinTakeoverTime) {
    play_count = kMaxEventCnt;
    last_navigation_time_stamp = -1;
  }
  if (play_count > 0) {
    play_count--;
    is_take_over = true;
  }
  // 3s时距和100.0取最大值
  static constexpr double kMinLenToMerge = 100.0;
  last_is_navigation = is_navigation;
  const auto& merge_lane_info = pnc_map_->IsMergingLaneChange();
  const auto& lane_change_infor = nnp_fct_out->lane_change_infor();
  static bool is_lane_change_finish = false;
  static constexpr double kToMainRaodTakeOver = 5.0;  // s
  if (!IsNviLayer() && merge_lane_info.first != 0 &&
      (IsRightnessTimeToDis(merge_lane_info.second, kToMainRaodTakeOver) ||
       merge_lane_info.second < kMinLenToMerge) &&
      lane_change_infor !=
          functionmanager::ChangeLaneInfor::LANE_CHANGE_ONGOING) {
    if (lane_change_infor ==
        functionmanager::ChangeLaneInfor::LANE_CHANGE_END) {
      is_lane_change_finish = true;
    }
    is_take_over = is_take_over || !is_lane_change_finish;
  } else {
    is_lane_change_finish = false;
  }

  const auto& adc_remain_len =
      ptr_trajectory_pb_->function_manager_out().adc_passage_remain_len();
  const auto& map_type =
      ptr_trajectory_pb_->function_manager_out().localization_maptype();
  is_take_over =
      is_take_over || ((map_type == navigation_hdmap::MapMsg::FUSION_NNP_MAP ||
                        map_type == navigation_hdmap::MapMsg::FUSION_NCP_MAP) &&
                       is_navigation &&
                       (IsRightnessTimeToDis(adc_remain_len, kMinSecond) ||
                        adc_remain_len < 200.0));
  if ((merge_lane_info.first != 0 &&
       IsRightnessTimeToDis(merge_lane_info.second, kMinSecond) &&
       pnc_map_->ADCInRamp())) {
    nnp_fct_out->set_dclc_audio_play(
        functionmanager::DclcAudioPlay::CAREFUL_DRIVING);
  } else if (is_take_over) {
    nnp_fct_out->set_nnp_rino_status(
        functionmanager::NNPRinoStatus::TASK_TAKE_OVER_REQ);
    nnp_fct_out->set_dclc_audio_play(functionmanager::DclcAudioPlay::DCLC_NONE);
  }
}

// 获取中控语音播报和显示内容
void CanNnpHmi::CheckScenariosOrAudioPlay() {
  CheckTargetObstacle();
  CheckSpeedAdapt();
  CheckLaneReduce();
  // CheckBestPathTracing();
}

// lane reduce
void CanNnpHmi::CheckLaneReduce() {
  if (pnc_map_ == nullptr || ptr_trajectory_pb_ == nullptr) {
    AERROR << "ptr is nullptr";
    return;
  }
  // const auto& lane_reduce = pnc_map_->GetLaneReduceInfo();
  // const auto& ta_pilot_mode =
  //     ptr_trajectory_pb_->function_manager_in().ta_pilot_mode();
  const auto& merge_lane_info = std::make_pair(routing::FORWARD, 0.0);
  // ta_pilot_mode == functionmanager::TaPilotMode::ADAS
  //     ? std::make_pair(routing::FORWARD, 0.0)
  //     : pnc_map_->GetLaneMergeDirAndLength();
  if (!soon_in_carriageWay_ && merge_lane_info.first != routing::FORWARD &&
      IsPlayTimeToDis(merge_lane_info.second, kLaneReduceTimestamp)) {
    auto* nnp_fct_out = ptr_trajectory_pb_->mutable_function_manager_out()
                            ->mutable_nnp_fct_out();
    nnp_fct_out->set_nnp_scenarios_audio_play(
        functionmanager::NNPScenarios::LANE_REDUCE);
    nnp_fct_out->set_nnp_scenarios(functionmanager::NNPScenarios::LANE_REDUCE);
  }
}

void CanNnpHmi::CheckTakeOver() {
  InOutRamp();
  LaneWidthTakeOver();
  RampExitTakeOver();
  RampDowngradeTakeOver();
  MergeLaneTakeOver();
  LongDirTakeOver();
  LocationFaultForTakeOver();
  CheckSplitLaneChangeTakeOver();
  CheckEnterRampOrExitMissed();
}

bool CanNnpHmi::IsRightnessTimeToDis(double len, double time_threshold) {
  if (frame_ == nullptr) {
    AERROR << "frame_ is nullptr";
    return false;
  }
  const auto& vehicle_state = frame_->local_view().GetVehicleState();
  double cur_speed = vehicle_state->linear_velocity();
  return cur_speed > kEpsilon && len / cur_speed <= time_threshold;
}

bool CanNnpHmi::IsPlayTimeToDis(double len, double time_threshold) {
  if (frame_ == nullptr) {
    AERROR << "frame_ is nullptr";
    return false;
  }
  const auto& vehicle_state = frame_->local_view().GetVehicleState();
  double cur_speed = vehicle_state->linear_velocity();
  if (cur_speed < kEpsilon) {
    return false;
  }
  double to_time = len / cur_speed;
  return to_time <= time_threshold && to_time >= time_threshold - kPlayTime;
}

void CanNnpHmi::InOutRamp() {
  if (pnc_map_ == nullptr || ptr_trajectory_pb_ == nullptr) {
    AERROR << "ptr is nullptr";
    return;
  }
  // static int broadcast_cnt = -1;
  auto* nnp_fct_out =
      ptr_trajectory_pb_->mutable_function_manager_out()->mutable_nnp_fct_out();
  static constexpr double kMinInOutRampTime = 15.0;  // 15s
  // static bool last_is_mainroad_before = true;
  // const auto& to_ramp_lane_count = nnp_fct_out->to_ramp_lane_count();
  // double to_ramp_len = nnp_fct_out->nnp_d_distance2_downramp_sg();
  double to_mainroad_len = nnp_fct_out->nnp_d_distance2_onramp_sg();
  const auto& merge_lane_info = pnc_map_->IsMergingLaneChange();
  // 即将汇入主路
  double check_len =
      (to_mainroad_len > kMinDistance && merge_lane_info.first != 0)
          ? merge_lane_info.second
          : to_mainroad_len;
  if (IsPlayTimeToDis(check_len, kToMainRoadTimestamp)) {
    soon_in_carriageWay_ = true;
    nnp_fct_out->set_nnp_rino_status(
        functionmanager::NNPRinoStatus::RAMP_IN_START);
  }
  // 汇入主路成功
  static double last_into_carriageWay_timeout = 0.0;
  // static double last_into_ramp_timeout = 0.0;
  if (!soon_in_carriageWay_ || to_mainroad_len > kMinDistance) {
    last_into_carriageWay_timeout = Clock::NowInSeconds();
  }

  if (Clock::NowInSeconds() - last_into_carriageWay_timeout >
          kMinInOutRampTime &&
      pnc_map_->AdcInMainRoad()) {
    // broadcast_cnt = kMaxEventCnt;
    soon_in_carriageWay_ = false;
  }
  // 即将汇入匝道
  // static constexpr double kMaxSoonInRampLen = 1000.0;
  // static constexpr double kMinSoonInRampLen = 200.0;
  // if (to_ramp_len < kMaxSoonInRampLen && to_ramp_len > kMinSoonInRampLen) {
  //   nnp_fct_out->set_nnp_rino_status(
  //       functionmanager::NNPRinoStatus::RAMP_OUT_START);
  // }

  // 进入匝道成功
  // bool is_mainroad_cur =
  //     ((to_ramp_lane_count == 0 && to_ramp_len > kMinDistance) ||
  //      to_ramp_lane_count != 0);
  // if (last_is_mainroad_before && !is_mainroad_cur) {
  //   last_into_ramp_timeout = Clock::NowInSeconds();
  //   soon_in_ramp_ = true;
  // }
  // last_is_mainroad_before = is_mainroad_cur;

  // if (soon_in_ramp_ &&
  //     Clock::NowInSeconds() - last_into_ramp_timeout > kMinInOutRampTime &&
  //     broadcast_cnt <= 0) {
  //   broadcast_cnt = kMaxEventCnt;
  // }

  // if (broadcast_cnt > 0) {  // 发送5帧
  //   if (soon_in_carriageWay_) {
  //     nnp_fct_out->set_nnp_rino_status(
  //         functionmanager::NNPRinoStatus::RAMP_IN_DONE);
  //   } else if (soon_in_ramp_ && pnc_map_->ADCInRamp()) {
  //     nnp_fct_out->set_nnp_rino_status(
  //         functionmanager::NNPRinoStatus::RAMP_OUT_DONE);
  //   }
  //   broadcast_cnt--;
  //   if (broadcast_cnt == 0) {
  //     soon_in_carriageWay_ = false;
  //     soon_in_ramp_ = false;
  //   }
  // }
}

void CanNnpHmi::CheckEnterRampOrExitMissed() {
  if (ptr_trajectory_pb_ == nullptr) {
    AERROR << "ptr_trajectory_pb_ is nullptr";
    return;
  }
  static constexpr double kMinMissedExitLen = 1.0;
  static constexpr double kMinTimeStamp = 2.0;
  static double last_timestamp = 0.0;
  auto* nnp_fct_out =
      ptr_trajectory_pb_->mutable_function_manager_out()->mutable_nnp_fct_out();
  const auto to_ramp_lane_count = nnp_fct_out->to_ramp_lane_count();
  double to_ramp_len = nnp_fct_out->nnp_d_distance2_downramp_sg();
  auto check_distance = [&]() -> bool {
    if (to_ramp_lane_count < 2) {
      return false;
    }
    return to_ramp_len < (to_ramp_lane_count * kThreshold) &&
           to_ramp_len > kMinMissedExitLen;
  };
  // 错过匝道
  if (pnc_map_->AdcInMainRoad() && check_distance()) {
    last_timestamp = Clock::NowInSeconds();
  }
  if (Clock::NowInSeconds() - last_timestamp < kMinTimeStamp) {
    nnp_fct_out->set_nnp_rino_status(
        functionmanager::NNPRinoStatus::EXIT_MISSED);
  }
}

std::pair<routing::ChangeLaneType, double> CanNnpHmi::CtlLightForMerge() {
  auto light_dir = std::make_pair(routing::FORWARD, 0.0);
  if (pnc_map_ == nullptr) {
    return light_dir;
  }
  const auto& merge_lane_info = pnc_map_->GetLaneMergeDirAndLength();
  if (!merge_lane_info.has_value() || merge_lane_info->empty()) {
    return light_dir;
  }
  const auto& merge_info_val = merge_lane_info.value();
  // AERROR << "------------start------";
  // for (const auto& merge_val : merge_info_val) {
  //   AERROR << "====" << merge_val.first << " " << merge_val.second;
  // }
  return merge_info_val.front();
}

void CanNnpHmi::CtlLightLaneSplitAndMerge() {  // NOLINT
  if (frame_ == nullptr) {
    AERROR << "ptr is nullptr";
    return;
  }
  if (is_lat_override_) {
    return;
  }
  constexpr static double kLightSecond = 7.0;
  constexpr static double kMaxLightSecond = 12.0;

  auto light_dir_ctl = functionmanager::LightReq::LIGHT_OFF;
  auto find_dir = [&](routing::ChangeLaneType dir) {
    auto light_dir = functionmanager::LightReq::LIGHT_OFF;
    if (dir == routing::LEFT) {
      light_dir = functionmanager::LightReq::LEFT_LIGHT;
    } else if (dir == routing::RIGHT) {
      light_dir = functionmanager::LightReq::RIGHT_LIGHT;
    }
    return light_dir;
  };
  const auto& ta_pilot_mode =
      ptr_trajectory_pb_->function_manager_in().ta_pilot_mode();
  const auto& split_lane_info =
      ta_pilot_mode == functionmanager::TaPilotMode::ADAS
          ? std::make_pair(routing::FORWARD, 0.0)
          : pnc_map_->GetLaneSplitDirAndLength();

  const auto& merge_lane_info =
      ta_pilot_mode == functionmanager::TaPilotMode::ADAS
          ? std::make_pair(routing::FORWARD, 0.0)
          : CtlLightForMerge();
  auto* nnp_fct_out =
      ptr_trajectory_pb_->mutable_function_manager_out()->mutable_nnp_fct_out();
  double to_ramp_len = nnp_fct_out->nnp_d_distance2_downramp_sg();
  // 时距或100.0m满足之一即打灯
  static constexpr double kCtlLightMinLen = 100.0;
  bool is_ok_ctrl_light =
      IsRightnessTimeToDis(split_lane_info.second, kLightSecond) ||
      split_lane_info.second < kCtlLightMinLen;
  static double last_split_ctl_light_timestamp = 0.0;
  static auto last_light_dir_ctl = functionmanager::LightReq::LIGHT_OFF;
  if (split_lane_info.first != routing::FORWARD && is_ok_ctrl_light) {
    last_light_dir_ctl = find_dir(split_lane_info.first);
    last_split_ctl_light_timestamp = Clock::NowInSeconds();
  } else if (merge_lane_info.first != routing::FORWARD &&
             (IsRightnessTimeToDis(merge_lane_info.second, kLightSecond) ||
              merge_lane_info.second < kCtlLightMinLen)) {
    light_dir_ctl = find_dir(merge_lane_info.first);
  }
  if (Clock::NowInSeconds() - last_split_ctl_light_timestamp < 2.0) {
    light_dir_ctl = last_light_dir_ctl;
  }
  static double last_ctrl_light_timestap = 0.0;
  if (to_ramp_len > 0.01 && to_ramp_len < 250.0 &&
      light_dir_ctl == functionmanager::LightReq::LIGHT_OFF &&
      split_lane_info.second < to_ramp_len && split_lane_info.second > 0.01 &&
      is_ok_ctrl_light) {
    last_ctrl_light_timestap = Clock::NowInSeconds();
  }
  if (Clock::NowInSeconds() - last_ctrl_light_timestap < 3.0) {
    light_dir_ctl = functionmanager::LightReq::RIGHT_LIGHT;
  }
  bool is_nvi_layer = IsNviLayer();
  if (is_nvi_layer) {
    light_dir_ctl = functionmanager::LightReq::LIGHT_OFF;
  }
  if (light_dir_ctl != functionmanager::LightReq::LIGHT_OFF) {
    // 停止线前直行车道不进行split和merge打灯
    const auto to_stop_line_dis = GetToStopLineDis();
    if (to_stop_line_dis.has_value() && to_stop_line_dis.value() < 60.0) {
      light_dir_ctl = functionmanager::LightReq::LIGHT_OFF;
    }
    // junction范围内，如果自车在直行车道，则不进行merge和spilt打灯
    const auto is_straight_lane_in_junction = IsStraightLaneInJunction();
    if (is_straight_lane_in_junction.has_value() &&
        is_straight_lane_in_junction.value()) {
      light_dir_ctl = functionmanager::LightReq::LIGHT_OFF;
    }
  }
  const auto& turn_info = pnc_map_->GetLaneTurnDirAndLength();
  if (turn_info.first != routing::FORWARD && turn_info.second < 300) {
    if ((IsRightnessTimeToDis(turn_info.second, kLightSecond) ||
         turn_info.second < kCtlLightMinLen)) {
      light_dir_ctl = find_dir(turn_info.first);
    }
  }
  const auto& merge_lane_change_info = pnc_map_->IsMergingLaneChange();
  if (merge_lane_change_info.first != 0 &&
      light_dir_ctl != functionmanager::LightReq::LIGHT_OFF &&
      merge_lane_change_info.second - merge_lane_info.second < 200.0) {
    light_dir_ctl = functionmanager::LightReq::LIGHT_OFF;
  }
  const auto& cur_light_ctl = CtrlChangeLaneLight(light_dir_ctl);
  if (cur_light_ctl.first) {
    light_dir_ctl = cur_light_ctl.second;
  }
  if (merge_lane_change_info.first != 0 &&
      (IsRightnessTimeToDis(merge_lane_change_info.second, kMaxLightSecond) ||
       merge_lane_change_info.second < 200.0)) {
    light_dir_ctl = merge_lane_change_info.first < 0
                        ? functionmanager::LightReq::LEFT_LIGHT
                        : functionmanager::LightReq::RIGHT_LIGHT;
  }

  nnp_fct_out->set_light_request(light_dir_ctl);
}

std::optional<bool> CanNnpHmi::IsStraightLaneInJunction() {
  const auto* reference_line_info = frame_->FindDriveReferenceLineInfo();
  if (reference_line_info == nullptr) {
    return std::nullopt;
  }
  const double adc_end_edge_s = reference_line_info->AdcSlBoundary().end_s();
  const double adc_start_edge_s =
      reference_line_info->AdcSlBoundary().start_s();
  double min_junction_start_s = std::numeric_limits<double>::max();
  double min_junction_end_s = 0.0;
  bool has_junction = false;
  for (const auto& junction :
       reference_line_info->reference_line().map_path().junction_overlaps()) {
    if (min_junction_start_s > junction.start_s) {
      min_junction_start_s = junction.start_s;
      min_junction_end_s = junction.end_s;
      has_junction = true;
    }
  }
  if (!has_junction) {
    return std::nullopt;
  }
  const auto junction_end_s =
      min_junction_end_s + (adc_end_edge_s - adc_start_edge_s);
  bool is_adc_in_junction = (adc_start_edge_s >= min_junction_start_s &&
                             adc_start_edge_s <= junction_end_s);
  if (is_adc_in_junction) {
    const auto to_front_junction_dis = junction_end_s - adc_start_edge_s;
    const auto dir = pnc_map_->GetFrontLaneDir(to_front_junction_dis);
    if (dir.has_value()) {
      return dir == routing::ChangeLaneType::FORWARD;
    }
  }
  return std::nullopt;
}

std::optional<double> CanNnpHmi::GetToStopLineDis() {
  const auto* reference_line_info = frame_->FindDriveReferenceLineInfo();
  if (reference_line_info == nullptr) {
    return std::nullopt;
  }
  const double adc_end_edge_s = reference_line_info->AdcSlBoundary().start_s();
  double min_dis_to_stop_line = std::numeric_limits<double>::max();
  bool has_stop_line = false;
  for (const auto& signal :
       reference_line_info->reference_line().map_path().signal_overlaps()) {
    if (min_dis_to_stop_line > signal.start_s) {
      min_dis_to_stop_line = signal.start_s;
      has_stop_line = true;
    }
  }
  const auto to_stop_line_dis = min_dis_to_stop_line - adc_end_edge_s;
  if (has_stop_line && to_stop_line_dis > 0.0) {
    return to_stop_line_dis;
  }
  return std::nullopt;
}

void CanNnpHmi::CheckLaneChangeWaningObs() {
  const auto& lane_change_dir = ptr_trajectory_pb_->function_manager_out()
                                    .nnp_fct_out()
                                    .lane_change_direction();
  const auto& lc_risk_obs = injector_->planning_context()
                                ->planning_status()
                                .change_lane()
                                .hmi_lane_change_risk_obstacles();
  auto* nnp_fct_out =
      ptr_trajectory_pb_->mutable_function_manager_out()->mutable_nnp_fct_out();

  bool has_lane_change_warning =
      lc_risk_obs.front_obstacle_id() > 0 || lc_risk_obs.back_obstacle_id() > 0;
  hmi::NNPHmiOutput_HighLightReasion high_light_reason =
      hmi::NNPHmiOutput::MAX_REASON;
  if (lane_change_dir == functionmanager::LaneChangeDir::LEFT_DIR &&
      has_lane_change_warning) {
    high_light_reason = hmi::NNPHmiOutput::ALC_LEFT;
    nnp_fct_out->set_lane_change_warning(
        functionmanager::LaneChangeWarn::LEFT_WARN);
  } else if (lane_change_dir == functionmanager::LaneChangeDir::RIGHT_DIR &&
             has_lane_change_warning) {
    high_light_reason = hmi::NNPHmiOutput::ALC_RIGHT;
    nnp_fct_out->set_lane_change_warning(
        functionmanager::LaneChangeWarn::RIGHT_WARN);
  } else {
    nnp_fct_out->set_lane_change_warning(
        functionmanager::LaneChangeWarn::NO_WARN);
  }
  if (high_light_reason == hmi::NNPHmiOutput::MAX_REASON) {
    return;
  }
  if (lc_risk_obs.front_obstacle_id() > 0) {
    auto* mutable_nnp_alc_obs_higtlight =
        ptr_trajectory_pb_->mutable_nnp_hmi_output()
            ->add_nnp_alc_obs_hightlight();
    mutable_nnp_alc_obs_higtlight->set_obs_hightlight_id(
        lc_risk_obs.front_obstacle_id());
    mutable_nnp_alc_obs_higtlight->set_highlight_reason(high_light_reason);
  }
  if (lc_risk_obs.back_obstacle_id() > 0) {
    auto* mutable_nnp_alc_obs_higtlight =
        ptr_trajectory_pb_->mutable_nnp_hmi_output()
            ->add_nnp_alc_obs_hightlight();
    mutable_nnp_alc_obs_higtlight->set_obs_hightlight_id(
        lc_risk_obs.back_obstacle_id());
    mutable_nnp_alc_obs_higtlight->set_highlight_reason(high_light_reason);
  }
}

// 障碍物避让和高亮
void CanNnpHmi::CheckTargetObstacle() {  // NOLINT
  if (frame_ == nullptr || ptr_trajectory_pb_ == nullptr) {
    AERROR << "ptr is nullptr";
    return;
  }
  if (is_override_ || is_pilot_override_) {
    return;
  }
  auto* nnp_fct_out =
      ptr_trajectory_pb_->mutable_function_manager_out()->mutable_nnp_fct_out();

  const auto* reference_line_info = frame_->FindDriveReferenceLineInfo();
  if (reference_line_info == nullptr) {
    return;
  }
  const auto& obstcales =
      reference_line_info->path_decision().obstacles().Items();
  for (const auto& obs : obstcales) {
    if (obs == nullptr) {
      continue;
    }
    const auto& id = obs->PerceptionId();
    // bool is_high_obs_type = obs->IsOversizedVehicle();
    const bool is_ok_avoid = (!obs->IsShortDistanceNudge());
    const bool is_static_avoid = obs->GetForbidStaticObsNudgeDisplay();
    for (const auto& decider_tag : obs->decider_tags()) {
      bool is_high_light =
          (absl::StrContains(decider_tag, "dynamic-left-nudge")) ||
          (absl::StrContains(decider_tag, "dynamic-right-nudge"));
      bool is_static_high_light =
          !is_static_avoid &&
          ((absl::StrContains(decider_tag, "static-left-nudge")) ||
           (absl::StrContains(decider_tag, "static-right-nudge")));
      if (is_ok_avoid && is_high_light) {
        last_obs_avoid_ids_[id] = {Clock::NowInSeconds(), false};
      }
      if (is_ok_avoid && is_static_high_light) {
        last_obs_avoid_ids_[id] = {Clock::NowInSeconds(), true};
      }
    }
  }
  std::vector<int32_t> timeout_ids{};
  bool has_obs_avoid = false;
  for (const auto& obs_avoid_id : last_obs_avoid_ids_) {
    // 避让高亮后持续2s，避免闪跳
    if (Clock::NowInSeconds() - obs_avoid_id.second.first < 2.0) {
      has_obs_avoid = true;
      auto* mutable_nnp_obs_higtlight =
          ptr_trajectory_pb_->mutable_nnp_hmi_output()
              ->add_nnp_obs_hightlight();
      mutable_nnp_obs_higtlight->set_obs_hightlight_id(obs_avoid_id.first);
      mutable_nnp_obs_higtlight->set_highlight_reason(
          TL::hmi::NNPHmiOutput::LAT_NUDGE);
      if (obs_avoid_id.second.second) {
        nnp_fct_out->set_nnp_scenarios(
            functionmanager::NNPScenarios::STATIC_OBSTACLE);
      } else {
        nnp_fct_out->set_nnp_scenarios(
            functionmanager::NNPScenarios::OVERSIZED_VEHICLE_ONE_SIDE);
      }
    } else {
      timeout_ids.emplace_back(obs_avoid_id.first);
    }
  }
  for (const auto& id : timeout_ids) {
    last_obs_avoid_ids_.erase(id);
  }
  if (!has_obs_avoid) {
    last_obs_avoid_ids_.clear();
  }
}

// 智能车速匹配
void CanNnpHmi::CheckSpeedAdapt() {
  if (frame_ == nullptr || pnc_map_ == nullptr ||
      ptr_trajectory_pb_ == nullptr) {
    AERROR << "ptr is nullptr";
    return;
  }
#ifndef FOR_BAIDU_SIMULATION
  const auto& fct_nnp_in =
      frame_->local_view().GetFunctionManagerIn()->fct_nnp_in();
  if (!fct_nnp_in.has_longitud_ctrl_cruise_speedms()) {
    return;
  }
#endif
  lon_hmi_.UpdateOutput(ptr_trajectory_pb_->mutable_function_manager_out(),
                        pnc_map_, frame_,
                        ptr_trajectory_pb_->mutable_soc_to_fct_bus());
}

// 当前车道信息
void CanNnpHmi::CheckLaneMsg() {
  if (pnc_map_ == nullptr || frame_ == nullptr) {
    AERROR << "ptr is nullptr";
    return;
  }
  const auto& adc_waypoint = pnc_map_->GetADCWaypoint();
  if (adc_waypoint.lane == nullptr) {
    AERROR << "adc finds no lane.";
    return;
  }

  if (!pnc_map_->GetDebugPncInfor().empty()) {
    auto* pnc_info = ptr_trajectory_pb_->mutable_debug()
                         ->mutable_planning_data()
                         ->mutable_pncmap_debug();
    for (const auto& single_debug_pnc_infor : pnc_map_->GetDebugPncInfor()) {
      pnc_info->add_pnc_debug_info(single_debug_pnc_infor.str());
    }
  }

  auto* nnp_fct_out =
      ptr_trajectory_pb_->mutable_function_manager_out()->mutable_nnp_fct_out();
  auto* cur_lane_msg = nnp_fct_out->mutable_crrent_lane_mg();
  double curvature = std::numeric_limits<double>::max();
  double adc_lane_heading = 0.0;
  double lan_width = 0.0;
  const auto* best_ref_info = frame_->FindDriveReferenceLineInfo();
  if (best_ref_info != nullptr) {
    const auto& ref_points = best_ref_info->reference_line().reference_points();
    const auto& adc_sl_boundary = best_ref_info->AdcSlBoundary();
    double middle_s =
        (adc_sl_boundary.start_s() + adc_sl_boundary.end_s()) / 2.0;
    if (best_ref_info->reference_line().Length() > 0) {
      auto adc_index =
          best_ref_info->reference_line().GetNearestReferenceIndex(middle_s);
      if (ref_points.size() > adc_index) {
        curvature = ref_points[adc_index].kappa();
        adc_lane_heading = ref_points[adc_index].Angle();
      }
    }
    double left_width = 0.0;
    double right_width = 0.0;
    if (best_ref_info->reference_line().GetLaneWidth(middle_s, &left_width,
                                                     &right_width)) {
      lan_width = left_width + right_width;
    }
  }
  cur_lane_msg->set_nnp_crv_crrntlanecurve_sg(static_cast<float>(curvature));
  cur_lane_msg->set_nnp_d_crrntlanewidth_sg(static_cast<float>(lan_width));
  cur_lane_msg->set_nnp_rad_crrntlanehead_sg(
      static_cast<float>(adc_lane_heading));
}

void CanNnpHmi::UpdateChangeLaneOutput(const ChangeLaneInfor& cl_type) {
  if (ptr_trajectory_pb_ == nullptr || injector_ == nullptr) {
    AERROR << "function_manager_out is nullptr";
    return;
  }
  if ((is_lat_override_ || !is_nnp_active_) &&
      (is_pilot_lat_override_ || !is_soc_pilot_active_)) {
    return;
  }
  static bool lane_change_status_is_ongoing = false;
  LcAudioPlay output_cl_audio_play = LcAudioPlay::LC_NONE;
  LaneChangeDir output_cl_dir = LaneChangeDir::NONE_DIR;
  DclcAudioPlay output_dclc_audio_play = DclcAudioPlay::DCLC_NONE;
  auto* nnp_fct_out =
      ptr_trajectory_pb_->mutable_function_manager_out()->mutable_nnp_fct_out();
  const auto& change_lane_dir = injector_->planning_context()
                                    ->planning_status()
                                    .change_lane()
                                    .change_lane_type();
  const auto& lane_change_reason = injector_->planning_context()
                                       ->planning_status()
                                       .change_lane()
                                       .change_lane_reason();
  const auto& hmi_lane_change =
      ptr_trajectory_pb_->function_manager_out().hmi_lane_change_debug();
  bool lane_change_dir_is_solid =
      hmi_lane_change.is_lane_change_on_solid_lane();
  // mapping
  switch (change_lane_dir) {
    case ChangeLaneType::LEFT:
      output_dclc_audio_play = DclcAudioPlay::LEFT_BRO;
      output_cl_audio_play = LcAudioPlay::LEFT_LANE_CHANGE_ACTIVE;
      output_cl_dir = LaneChangeDir::LEFT_DIR;
      break;
    case ChangeLaneType::RIGHT:
      output_dclc_audio_play = DclcAudioPlay::RIGHT_BRO;
      output_cl_audio_play = LcAudioPlay::RIGHT_LANE_CHANGE_ACTIVE;
      output_cl_dir = LaneChangeDir::RIGHT_DIR;
      break;
    default:
      break;
  }
  if (expand_change_lane_dir_ == LaneChangeDir::NONE_DIR ||
      (expand_change_lane_dir_ != LaneChangeDir::NONE_DIR &&
       output_cl_dir != LaneChangeDir::NONE_DIR)) {
    expand_change_lane_dir_ = output_cl_dir;
  }
  if (cl_type != ChangeLaneInfor::LANE_CHANGE_PENDING_ALERT) {
    nnp_fct_out->set_lane_change_infor(cl_type);
  }
  functionmanager::AlcMode alc_mode = functionmanager::AlcMode::AUTO_ALC;
  if (frame_->local_view().HasFunctionManagerIn()) {
    alc_mode =
        frame_->local_view().GetFunctionManagerIn()->fct_nnp_in().alc_mode();
  }
  const auto& early_signal_light = ptr_trajectory_pb_->debug()
                                       .planning_data()
                                       .planning_status()
                                       .change_lane()
                                       .turn_signal();
  switch (cl_type) {
    case ChangeLaneInfor::LANE_CHANGE_START:
      lane_change_status_is_ongoing = false;
      nnp_fct_out->set_lane_change_direction(expand_change_lane_dir_);
      nnp_fct_out->set_lc_audio_play(output_cl_audio_play);
      break;
    case ChangeLaneInfor::LANE_CHANGE_PENDING:
      lane_change_status_is_ongoing =
          (lane_change_reason ==
           functionmanager::HmiChangeLaneReason::HMI_TURN_SIGNAL_SWITCH);
      if (lane_change_dir_is_solid) {
        nnp_fct_out->set_lane_change_infor(
            ChangeLaneInfor::LANE_CHANGE_PENDING_ALERT);
      }
      if (early_signal_light != common::VehicleSignal::TURN_NONE &&
          (alc_mode != functionmanager::AlcMode::NEED_CONFIRM_ALC ||
           change_lane_reason_ != functionmanager::HMI_EFFICIENCY)) {
        double to_ramp_len = nnp_fct_out->nnp_d_distance2_downramp_sg();
        const auto& to_ramp_lane_count = nnp_fct_out->to_ramp_lane_count();
        if (!soon_in_carriageWay_ &&
            change_lane_reason_ == functionmanager::HMI_NAVIGATION &&
            (to_ramp_len > kMinDistance || to_ramp_lane_count > 0)) {
          nnp_fct_out->set_nnp_rino_status(
              functionmanager::NNPRinoStatus::RAMP_OUT_START);
        } else {
          nnp_fct_out->set_dclc_audio_play(output_dclc_audio_play);
        }
      }
      nnp_fct_out->set_lane_change_direction(expand_change_lane_dir_);
      break;
    case ChangeLaneInfor::LANE_CHANGE_ONGOING:
      lane_change_status_is_ongoing = true;
      if (alc_mode != functionmanager::AlcMode::NEED_CONFIRM_ALC ||
          change_lane_reason_ != functionmanager::HMI_EFFICIENCY) {
        double to_ramp_len = nnp_fct_out->nnp_d_distance2_downramp_sg();
        const auto& to_ramp_lane_count = nnp_fct_out->to_ramp_lane_count();
        if (!soon_in_carriageWay_ &&
            change_lane_reason_ == functionmanager::HMI_NAVIGATION &&
            (to_ramp_len > kMinDistance || to_ramp_lane_count > 0)) {
          nnp_fct_out->set_nnp_rino_status(
              functionmanager::NNPRinoStatus::RAMP_OUT_START);
        } else {
          nnp_fct_out->set_dclc_audio_play(output_dclc_audio_play);
        }
      }
      nnp_fct_out->set_lane_change_direction(expand_change_lane_dir_);
      break;
    case ChangeLaneInfor::LANE_CHANGE_CANCEL:
      is_expand_change_lane_ = false;
      expand_change_lane_dir_ = LaneChangeDir::NONE_DIR;
      change_lane_reason_ = functionmanager::HMI_NONE;
      nnp_fct_out->set_lcsndrequest(true);
      if (lane_change_status_is_ongoing) {
        nnp_fct_out->set_dclc_audio_play(
            functionmanager::DclcAudioPlay::CANCEL_BRO);
      } else {
        nnp_fct_out->set_lane_change_infor(ChangeLaneInfor::INFOR_NONE);
      }
      break;
    case ChangeLaneInfor::LANE_CHANGE_PENDING_ALERT:
      is_expand_change_lane_ = false;
      expand_change_lane_dir_ = LaneChangeDir::NONE_DIR;
      change_lane_reason_ = functionmanager::HMI_NONE;
      nnp_fct_out->set_lcsndrequest(true);
      nnp_fct_out->set_lane_change_pending_alert(true);
      nnp_fct_out->set_lane_change_infor(ChangeLaneInfor::LANE_CHANGE_CANCEL);
    default:
      is_expand_change_lane_ = false;
      expand_change_lane_dir_ = LaneChangeDir::NONE_DIR;
      change_lane_reason_ = functionmanager::HMI_NONE;
      break;
  }
}

void CanNnpHmi::UpdateContinuousLaneChange() {
  const auto& change_lane =
      injector_->planning_context()->planning_status().change_lane();
  const auto& hmi_change_lane_status = change_lane.hmi_change_lane_status();
  if (is_expand_change_lane_ &&
      (hmi_change_lane_status ==
           functionmanager::HmiChangeLaneStatus::HMI_CHANGE_LANE_START ||
       hmi_change_lane_status ==
           functionmanager::HmiChangeLaneStatus::HMI_IN_CHANGE_LANE)) {
    UpdateChangeLaneOutput(ChangeLaneInfor::LANE_CHANGE_END);
  }
}

void CanNnpHmi::ExpandChangeLaneOutput() {
  if (ptr_trajectory_pb_ == nullptr) {
    AERROR << "ptr_trajectory_pb_ is nullptr";
    return;
  }
  if (is_expand_change_lane_) {
    auto* function_manager_out =
        ptr_trajectory_pb_->mutable_function_manager_out()
            ->mutable_nnp_fct_out();
    if (!ExpandChangeLaneOngoing()) {
      UpdateChangeLaneOutput(ChangeLaneInfor::LANE_CHANGE_END);
    } else {
      function_manager_out->set_lane_change_infor(
          functionmanager::ChangeLaneInfor::LANE_CHANGE_ONGOING);
      function_manager_out->set_lane_change_direction(expand_change_lane_dir_);
    }
  }
}

std::pair<bool, functionmanager::LightReq> CanNnpHmi::CtrlChangeLaneLight(
    const functionmanager::LightReq& light_dir_ctl) {
  auto ctrl_light_status =
      std::make_pair(false, functionmanager::LightReq::LIGHT_OFF);
  if (ptr_trajectory_pb_ == nullptr) {
    AERROR << "ptr_trajectory_pb_ is nullptr";
    return ctrl_light_status;
  }
  const auto& early_signal_light = ptr_trajectory_pb_->debug()
                                       .planning_data()
                                       .planning_status()
                                       .change_lane()
                                       .turn_signal();
  const auto& signal_light =
      ptr_trajectory_pb_->decision().vehicle_signal().turn_signal();
  const auto& lane_change_infor = ptr_trajectory_pb_->function_manager_out()
                                      .nnp_fct_out()
                                      .lane_change_infor();
  const auto& lane_change_reason = ptr_trajectory_pb_->function_manager_out()
                                       .hmi_lane_change_debug()
                                       .hmi_lane_change_reason();
  if (early_signal_light == common::VehicleSignal::TURN_NONE &&
      signal_light != common::VehicleSignal::TURN_NONE &&
      light_dir_ctl == functionmanager::LightReq::LIGHT_OFF &&
      lane_change_infor !=
          functionmanager::ChangeLaneInfor::LANE_CHANGE_ONGOING &&
      lane_change_infor != functionmanager::ChangeLaneInfor::INFOR_NONE &&
      lane_change_reason !=
          functionmanager::HmiChangeLaneReason::HMI_TURN_SIGNAL_SWITCH) {
    ctrl_light_status.first = true;
    ctrl_light_status.second = functionmanager::LightReq::LIGHT_OFF;
    return ctrl_light_status;
  }
  const auto& cur_signal_light =
      early_signal_light == common::VehicleSignal::TURN_NONE
          ? signal_light
          : early_signal_light;
  switch (cur_signal_light) {
    case common::VehicleSignal::TURN_LEFT:
      ctrl_light_status.first = true;
      ctrl_light_status.second = functionmanager::LightReq::LEFT_LIGHT;
      return ctrl_light_status;
    case common::VehicleSignal::TURN_RIGHT:
      ctrl_light_status.first = true;
      ctrl_light_status.second = functionmanager::LightReq::RIGHT_LIGHT;
      return ctrl_light_status;
    default:
      if (lane_change_infor != functionmanager::ChangeLaneInfor::INFOR_NONE) {
        ctrl_light_status.first = true;
        ctrl_light_status.second = light_dir_ctl;
      }
      return ctrl_light_status;
  }
}

void CanNnpHmi::ChangeLaneLightRemind() {
  if (ptr_trajectory_pb_ == nullptr || frame_ == nullptr ||
      !frame_->local_view().HasChassis()) {
    return;
  }
  static constexpr double kMaxRemindTime = 10.0;  // 变道完成10秒提醒
  static double timeout_light_remind = -1.0;
  static double last_start_stamp = 0.0;
  static int remind_cnt = 0;
  static common::VehicleSignal::TurnSignal last_turn_signal =
      common::VehicleSignal::TURN_NONE;
  static functionmanager::HmiChangeLaneReason last_change_lane_reason =
      functionmanager::HmiChangeLaneReason::HMI_NONE;
  const auto& chassis = frame_->local_view().GetChassis();
  const auto& turn_signal = chassis->signal().turn_signal();
  const auto& change_lane =
      injector_->planning_context()->planning_status().change_lane();
  const auto& change_lane_reason = change_lane.change_lane_reason();
  if (!is_nnp_active_ && !is_soc_pilot_active_) {
    timeout_light_remind = -1.0;
    last_change_lane_reason = functionmanager::HmiChangeLaneReason::HMI_NONE;
    remind_cnt = 0;
    last_start_stamp = 0;
  }
  if (last_change_lane_reason ==
          functionmanager::HmiChangeLaneReason::HMI_NONE &&
      change_lane_reason ==
          functionmanager::HmiChangeLaneReason::HMI_TURN_SIGNAL_SWITCH) {
    timeout_light_remind = -1.0;
    last_change_lane_reason = change_lane_reason;
  }
  if (timeout_light_remind > 0.0) {
    if (turn_signal != common::VehicleSignal::TURN_NONE &&
        last_turn_signal == turn_signal) {
      if (Clock::NowInSeconds() - last_start_stamp > timeout_light_remind) {
        auto* function_manager_out =
            ptr_trajectory_pb_->mutable_function_manager_out()
                ->mutable_nnp_fct_out();
        function_manager_out->set_nnp_light_remind(true);
        if (remind_cnt > kMaxEventCnt) {
          timeout_light_remind = -1.0;
          remind_cnt = 0;
        }
        remind_cnt++;
      }
    } else {
      timeout_light_remind = -1.0;
      remind_cnt = 0;
    }
    last_turn_signal = turn_signal;
  } else {
    last_start_stamp = Clock::NowInSeconds();
  }
  if (last_change_lane_reason == functionmanager::HMI_TURN_SIGNAL_SWITCH) {
    const auto& change_lane_type = ptr_trajectory_pb_->function_manager_out()
                                       .nnp_fct_out()
                                       .lane_change_infor();
    if (change_lane_type == functionmanager::LANE_CHANGE_CANCEL ||
        change_lane_type == functionmanager::LANE_CHANGE_END) {
      timeout_light_remind = kMaxRemindTime;
      last_change_lane_reason = functionmanager::HMI_NONE;
      last_turn_signal = turn_signal;
    }
  }
}

void CanNnpHmi::ChangeLaneOutputDecision() {
  if (frame_ == nullptr || injector_ == nullptr ||
      ptr_trajectory_pb_ == nullptr ||
      injector_->planning_context() == nullptr) {
    AERROR << "ptr is nullptr";
    return;
  }
  static functionmanager::HmiChangeLaneStatus last_hmi_change_lane_status =
      functionmanager::HMI_CHANGE_LANE_FINISHED;
  static bool last_is_ongoing = false;
  bool is_cancel_lane_change = false;
  const auto& change_lane =
      injector_->planning_context()->planning_status().change_lane();
  const auto& hmi_change_lane_status = change_lane.hmi_change_lane_status();
  const auto& change_lane_reason = change_lane.change_lane_reason();

  auto* change_lane_debug = ptr_trajectory_pb_->mutable_function_manager_out()
                                ->mutable_hmi_lane_change_debug();
  change_lane_debug->set_hmi_lane_change_reason(change_lane_reason);
  change_lane_debug->set_hmi_lane_change_status(hmi_change_lane_status);
  change_lane_debug->set_hmi_cancel_lane_change_reason(
      change_lane.hmi_cancel_change_lane_reason());
  change_lane_debug->set_is_lane_change_on_solid_lane(
      change_lane.is_lane_change_on_solid_lane());
  change_lane_debug->set_is_current_opt_succeed(
      change_lane.is_current_opt_succeed());
  change_lane_debug->set_target_lane_has_lane_marker(
      change_lane.target_lane_has_lane_marker());
  change_lane_debug->set_is_auto_lane_change_mode(
      change_lane.is_auto_lane_change_mode());
  change_lane_debug->set_is_clear_to_change_lane(
      change_lane.is_clear_to_change_lane());
  change_lane_debug->set_failed_lane_change_reason(
      change_lane.failed_lane_change_reason());
  change_lane_debug->mutable_is_lane_change_sidepass_obstacle()
      ->set_is_lane_change_sidepass_obstacle(
          change_lane.is_lane_change_sidepass_obstacle()
              .is_lane_change_sidepass_obstacle());
  change_lane_debug->set_gap_debug_info(change_lane.gap_debug_info());
  change_lane_debug->mutable_is_reject_cancel_lane_change()
      ->set_is_reject_cancel_lane_change(
          change_lane.is_reject_cancel_lane_change()
              .is_reject_cancel_lane_change());

  if ((is_lat_override_ || !is_nnp_active_) &&
      (is_pilot_lat_override_ || !is_soc_pilot_active_)) {
    is_expand_change_lane_ = false;
    expand_change_lane_dir_ = LaneChangeDir::NONE_DIR;
    change_lane_reason_ = functionmanager::HMI_NONE;
    last_is_ongoing = false;
  }
  static double last_lane_change_fail_timestamp = 0.0;
  // 扩展LANE_CHANGE_ONGOING状态
  ExpandChangeLaneOutput();
  if ((change_lane_reason == functionmanager::HMI_NAVIGATION ||
       change_lane_reason == functionmanager::HMI_EFFICIENCY) &&
      hmi_change_lane_status ==
          functionmanager::
              HMI_CHANGE_LANE_REQUEST) {  // 效率变道或导航变道，在需要确认模式下发送
    UpdateChangeLaneOutput(ChangeLaneInfor::LANE_CHANGE_START);
  } else if (hmi_change_lane_status ==
             functionmanager::HMI_CHANGE_LANE_START) {  // 即将变道
    UpdateChangeLaneOutput(ChangeLaneInfor::LANE_CHANGE_PENDING);
    change_lane_reason_ = change_lane_reason;
  } else if (hmi_change_lane_status ==
             functionmanager::HMI_IN_CHANGE_LANE) {  // 变道中
    change_lane_reason_ = change_lane_reason;
    UpdateChangeLaneOutput(ChangeLaneInfor::LANE_CHANGE_ONGOING);
    last_is_ongoing = true;
  } else if (last_is_ongoing &&
             hmi_change_lane_status ==
                 functionmanager::HMI_CHANGE_LANE_FINISHED) {  // 变道完成
    last_is_ongoing = false;
    is_expand_change_lane_ = true;
    UpdateChangeLaneOutput(ChangeLaneInfor::LANE_CHANGE_ONGOING);
  } else if (hmi_change_lane_status ==
             functionmanager::HMI_CHANGE_LANE_CANCELED) {  // 变道取消
    last_is_ongoing = false;
    if (last_hmi_change_lane_status == functionmanager::HMI_IN_CHANGE_LANE) {
      // 变道过程中变道返回导致的变道失败
      if (change_lane.hmi_cancel_change_lane_reason() ==
              functionmanager::CancelLaneChangeReason::
                  SAFETY_DECIDER_CANCEL_LANE_CHANGE ||
          change_lane.hmi_cancel_change_lane_reason() ==
              functionmanager::CancelLaneChangeReason::
                  EFFICIENCY_LANE_CHANGE_CANCELED ||
          !change_lane.is_current_opt_succeed()) {
        last_lane_change_fail_timestamp = Clock::NowInSeconds();
      } else {
        is_cancel_lane_change = true;
      }
      is_expand_change_lane_ = false;
      expand_change_lane_dir_ = LaneChangeDir::NONE_DIR;
    } else {
      if (last_hmi_change_lane_status !=
              functionmanager::HMI_CHANGE_LANE_FINISHED &&
          last_hmi_change_lane_status != hmi_change_lane_status) {
        is_cancel_lane_change = true;
      } else {
        is_expand_change_lane_ = false;
        expand_change_lane_dir_ = LaneChangeDir::NONE_DIR;
      }
    }
  } else if (last_hmi_change_lane_status ==
                 functionmanager::HMI_IN_CHANGE_LANE &&
             hmi_change_lane_status ==
                 functionmanager::HMI_CHANGE_LANE_FAILED) {
    // 变道过程中变道返回导致的变道失败
    last_is_ongoing = false;
    is_cancel_lane_change = true;
    is_expand_change_lane_ = false;
    expand_change_lane_dir_ = LaneChangeDir::NONE_DIR;
  }
  UpdateContinuousLaneChange();
  OptimizationChangeLaneAudioPlay(is_cancel_lane_change);
  const auto& change_lane_type = ptr_trajectory_pb_->function_manager_out()
                                     .nnp_fct_out()
                                     .lane_change_infor();
  static double last_close_turn_light_timestamp = 0.0;
  static bool last_is_reject_cancel_lane_change = false;
  static double last_pending_timestamp = -1.0;
  if (change_lane_type == functionmanager::LANE_CHANGE_ONGOING) {
    if (change_lane.is_reject_cancel_lane_change()
            .is_reject_cancel_lane_change()) {
      last_is_reject_cancel_lane_change = true;
    }
  } else {
    last_is_reject_cancel_lane_change = false;
  }
  if (last_pending_timestamp < 0.0 &&
      hmi_change_lane_status == functionmanager::HMI_CHANGE_LANE_START) {
    last_pending_timestamp = Clock::NowInSeconds();
  } else if (hmi_change_lane_status ==
             functionmanager::HMI_CHANGE_LANE_FINISHED) {
    last_pending_timestamp = -1.0;
  }
  bool is_cancel =
      hmi_change_lane_status == functionmanager::HMI_CHANGE_LANE_CANCELED ||
      change_lane_type == functionmanager::LANE_CHANGE_CANCEL;
  bool is_cloce_light =
      change_lane_reason_ == functionmanager::HMI_TURN_SIGNAL_SWITCH &&
      (Clock::NowInSeconds() - last_pending_timestamp < 0.5 ||
       last_pending_timestamp < 0.0);
  if ((!is_cloce_light && is_cancel) ||
      change_lane_type == functionmanager::LANE_CHANGE_END) {
    last_close_turn_light_timestamp = Clock::NowInSeconds();
  }
  if (Clock::NowInSeconds() - last_close_turn_light_timestamp < 0.5) {
    ptr_trajectory_pb_->mutable_soc_to_fct_bus()
        ->mutable_soc_to_fct_bus_u8()
        ->set_soc_close_turn_light_req(0x01);
  }
  if (Clock::NowInSeconds() - last_lane_change_fail_timestamp < 0.5) {
    ptr_trajectory_pb_->mutable_soc_to_fct_bus()
        ->mutable_soc_to_fct_bus_u8()
        ->set_cruise_status(0x01);
  }
  if (last_is_reject_cancel_lane_change) {
    ptr_trajectory_pb_->mutable_soc_to_fct_bus()
        ->mutable_soc_to_fct_bus_u8()
        ->set_cruise_status(0x02);
    // 由于座舱“请注意安全“信号适配错误，sop阶段，临时发送此信号
    ptr_trajectory_pb_->mutable_function_manager_out()
        ->mutable_nnp_fct_out()
        ->set_dclc_audio_play(functionmanager::DclcAudioPlay::CAREFUL_DRIVING);
  }
  last_hmi_change_lane_status = hmi_change_lane_status;
  // CheckLaneHighNoLaneChange();
  // 变道完成或者变道取消转向灯长时间未关闭(拨杆情况下判断)
  // ChangeLaneLightRemind();
}

// 判断取消变道和即将变道间隔时间是否大于阈值，不满足则延迟播放
void CanNnpHmi::OptimizationChangeLaneAudioPlay(bool is_cancel_lane_change) {
  static double last_audio_timestamp = -1.0;
  static double last_play_timestamp = -1.0;
  static bool cur_is_cancel_status = false;
  constexpr static double kMaxTimes = 0.5;  // 3秒
  static ChangeLaneInfor last_change_lane_infor = ChangeLaneInfor::INFOR_NONE;
  static ChangeLaneInfor change_lane_infor = ChangeLaneInfor::INFOR_NONE;
  const auto& nnp_fct_out =
      ptr_trajectory_pb_->function_manager_out().nnp_fct_out();
  const auto& cur_change_lane_infor = nnp_fct_out.lane_change_infor();
  const auto& change_lane =
      injector_->planning_context()->planning_status().change_lane();
  const auto& hmi_cancel_change_lane_reason =
      change_lane.hmi_cancel_change_lane_reason();
  if ((is_lat_override_ || !is_nnp_active_) &&
      (is_pilot_lat_override_ || !is_soc_pilot_active_)) {
    last_audio_timestamp = -1.0;
    change_lane_infor = ChangeLaneInfor::INFOR_NONE;
    last_change_lane_infor = ChangeLaneInfor::INFOR_NONE;
    cur_is_cancel_status = false;
    return;
  }
  if (is_cancel_lane_change) {
    cur_is_cancel_status = true;
  }
  if ((last_change_lane_infor == ChangeLaneInfor::LANE_CHANGE_END ||
       last_change_lane_infor == ChangeLaneInfor::INFOR_NONE) &&
      (cur_change_lane_infor == ChangeLaneInfor::LANE_CHANGE_START ||
       cur_change_lane_infor == ChangeLaneInfor::LANE_CHANGE_PENDING ||
       cur_change_lane_infor == ChangeLaneInfor::LANE_CHANGE_PENDING_ALERT ||
       cur_change_lane_infor == ChangeLaneInfor::LANE_CHANGE_ONGOING)) {
    last_audio_timestamp = Clock::NowInSeconds();
  }
  if (cur_is_cancel_status && hmi_cancel_change_lane_reason ==
                                  functionmanager::CancelLaneChangeReason::
                                      CHECK_OVERTIME_BEFORE_LANE_CHANGE) {
    cur_is_cancel_status = false;
    change_lane_infor = ChangeLaneInfor::LANE_CHANGE_PENDING_ALERT;
    last_play_timestamp = Clock::NowInSeconds();
    last_audio_timestamp = -1.0;
  }
  if (cur_is_cancel_status && last_audio_timestamp > 0) {
    if (Clock::NowInSeconds() - last_audio_timestamp >= kMaxTimes) {
      change_lane_infor = ChangeLaneInfor::LANE_CHANGE_CANCEL;
      last_play_timestamp = Clock::NowInSeconds();
    }
    cur_is_cancel_status = false;
    last_audio_timestamp = -1.0;
  }
  if (Clock::NowInSeconds() - last_play_timestamp < 0.5) {
    UpdateChangeLaneOutput(change_lane_infor);
  } else {
    change_lane_infor = ChangeLaneInfor::INFOR_NONE;
  }
  last_change_lane_infor = cur_change_lane_infor;
}

bool CanNnpHmi::ExpandChangeLaneOngoing() {
  if (frame_ == nullptr) {
    AERROR << "frame_ is nullptr";
    return false;
  }
  static constexpr double kLBuffer = 0.3;
  static constexpr double kAdcHalf = 2.0;
  const auto& vehicle_param = common::VehicleConfigHelper::GetConfig();
  double adc_half_width = vehicle_param.vehicle_param().width() / kAdcHalf;
  const auto* target_ref_info = frame_->FindTargetReferenceLineInfo();
  if (target_ref_info != nullptr) {
    const auto& adc_sl_boundary = target_ref_info->AdcSlBoundary();
    bool status =
        (std::fabs(adc_sl_boundary.start_l()) < (adc_half_width + kLBuffer) &&
         std::fabs(adc_sl_boundary.end_l()) < (adc_half_width + kLBuffer)) ||
        (std::fabs(adc_sl_boundary.start_l() + adc_sl_boundary.end_l()) <
         kLBuffer);
    return !status;
  }
  return false;
}

void CanNnpHmi::CheckLaneHighNoLaneChange() {
  if (ptr_trajectory_pb_ == nullptr || frame_ == nullptr) {
    return;
  }
  const auto& hmi_lane_change =
      ptr_trajectory_pb_->function_manager_out().hmi_lane_change_debug();
  bool lane_change_dir_is_solid =
      hmi_lane_change.is_lane_change_on_solid_lane() ||
      !hmi_lane_change.target_lane_has_lane_marker();
  functionmanager::LaneChangeDir dir = functionmanager::LaneChangeDir::NONE_DIR;
  if (frame_->local_view().HasChassis()) {
    const auto& chassis = frame_->local_view().GetChassis();
    if (chassis->signal().turn_signal() != common::VehicleSignal::TURN_NONE) {
      dir = chassis->signal().turn_signal() == common::VehicleSignal::TURN_LEFT
                ? functionmanager::LaneChangeDir::LEFT_DIR
                : functionmanager::LaneChangeDir::RIGHT_DIR;
    }
  }
  auto* nnp_fct_out =
      ptr_trajectory_pb_->mutable_function_manager_out()->mutable_nnp_fct_out();
  if (lane_change_dir_is_solid &&
      ptr_trajectory_pb_->function_manager_out()
              .nnp_fct_out()
              .lane_change_infor() != ChangeLaneInfor::LANE_CHANGE_ONGOING &&
      dir != functionmanager::LaneChangeDir::NONE_DIR &&
      nnp_fct_out != nullptr) {
    nnp_fct_out->set_lane_change_infor(
        ChangeLaneInfor::LANE_CHANGE_PENDING_ALERT);
    nnp_fct_out->set_lane_change_direction(dir);
  }
}

void CanNnpHmi::ProcessFctInput(const std::shared_ptr<LocalView>& local_view) {
  // process lon control info
  ProcessFctInputLonCtrlInfo(local_view);
}

void CanNnpHmi::UpdatePilotLaneChangeStatus(  // NOLINT
    const functionmanager::FunctionManagerIn& fct_in,
    functionmanager::FctToNnpInput* const nnp_fct_in) {
  const auto& ta_pilot_mode = fct_in.ta_pilot_mode();
  if (ta_pilot_mode != functionmanager::TaPilotMode::ADAS ||
      !fct_in.has_fct_2_soc_tbd_u32_02()) {
    nnp_fct_in->clear_pilot_lane_change_status();
    return;
  }
  static constexpr size_t kPilotSeq = 25;
  static constexpr uint32_t kMask = 0xff;
  static constexpr uint32_t kMask1 = 0x01;
  static constexpr uint32_t kMask2 = 0x02;
  static constexpr uint32_t kMask3 = 0x04;
  static constexpr uint32_t kMask4 = 0x08;
  static constexpr uint32_t kMask5 = 0x10;
  static constexpr uint32_t kMask6 = 0x20;
  static constexpr uint32_t kMask7 = 0x40;
  const auto& pilot_lc_status =
      (fct_in.fct_2_soc_tbd_u32_02() >> kPilotSeq) & kMask;
  auto* pilot_lane_change_status =
      nnp_fct_in->mutable_pilot_lane_change_status();
  if ((pilot_lc_status & kMask1) > 0) {
    pilot_lane_change_status->set_pilot_lc_infor(
        functionmanager::ChangeLaneInfor::LANE_CHANGE_END);
  } else if ((pilot_lc_status & kMask2) > 0) {
    pilot_lane_change_status->set_pilot_lc_infor(
        functionmanager::ChangeLaneInfor::LANE_CHANGE_CANCEL);
  } else if ((pilot_lc_status & kMask3) > 0) {
    pilot_lane_change_status->set_pilot_lc_infor(
        functionmanager::ChangeLaneInfor::LANE_CHANGE_PENDING);
  } else if ((pilot_lc_status & kMask4) > 0) {
    pilot_lane_change_status->set_pilot_lc_infor(
        functionmanager::ChangeLaneInfor::LANE_CHANGE_ONGOING);
  } else {
    pilot_lane_change_status->set_pilot_lc_infor(
        functionmanager::ChangeLaneInfor::INFOR_NONE);
  }
  if ((pilot_lc_status & kMask5) > 0) {
    pilot_lane_change_status->set_pilot_lc_dir(
        functionmanager::PilotLaneChangeDir::PILOT_DIR_LEFT);
  } else if ((pilot_lc_status & kMask6) > 0) {
    pilot_lane_change_status->set_pilot_lc_dir(
        functionmanager::PilotLaneChangeDir::PILOT_DIR_RIGHT);
  } else {
    pilot_lane_change_status->set_pilot_lc_dir(
        functionmanager::PilotLaneChangeDir::PILOT_DIR_NONE);
  }
  if ((pilot_lc_status & kMask7) > 0) {
    pilot_lane_change_status->set_pilot_lc_warning(true);
  } else {
    pilot_lane_change_status->set_pilot_lc_warning(false);
  }
}

void CanNnpHmi::ProcessFctInputLonCtrlInfo(
    const std::shared_ptr<LocalView>& local_view) {
  TL::functionmanager::FunctionManagerIn fct_input;
  if (local_view->HasFunctionManagerIn()) {
    fct_input.CopyFrom(*local_view->GetFunctionManagerIn());
  }

  // if (local_view->HasTransportElement()) {
  //   fct_input.mutable_transport_element()->CopyFrom(
  //       *local_view->GetTransportElement());
  // }
  if (local_view->HasRoutingRequest()) {
    fct_input.mutable_routing_request()->CopyFrom(
        *local_view->GetRoutingRequest());
  }
  if (local_view->HasFunctionManagerOut() && local_view->HasVehicleState() &&
      local_view->HasMcuToSocPnc()) {
    lon_hmi_.UpdateInput(&fct_input, local_view->GetFunctionManagerOut().get(),
                         *local_view->GetVehicleState(),
                         *local_view->GetMcuToSocPnc());
  }
  UpdatePilotLaneChangeStatus(fct_input, fct_input.mutable_fct_nnp_in());
  local_view->SetFunctionManagerInPtr(
      std::make_shared<TL::functionmanager::FunctionManagerIn>(fct_input));
}  // namespace planning

void CanNnpHmi::ResetFctHmiOut() {
  if (ptr_trajectory_pb_ == nullptr) {
    AERROR << "ptr_trajectory_pb is nullptr";
    return;
  }
  auto* nnp_fct_out =
      ptr_trajectory_pb_->mutable_function_manager_out()->mutable_nnp_fct_out();
  nnp_fct_out->set_lane_change_infor(
      TL::functionmanager::ChangeLaneInfor::INFOR_NONE);
  nnp_fct_out->set_lane_change_direction(
      functionmanager::LaneChangeDir::NONE_DIR);
  nnp_fct_out->set_lc_audio_play(functionmanager::LcAudioPlay::LC_NONE);
  nnp_fct_out->set_lcsndrequest(false);
  nnp_fct_out->set_dclc_audio_play(functionmanager::DclcAudioPlay::DCLC_NONE);
  nnp_fct_out->set_lane_change_warning(
      functionmanager::LaneChangeWarn::NO_WARN);
  nnp_fct_out->set_light_request(functionmanager::LightReq::LIGHT_OFF);
  nnp_fct_out->set_lane_change_pending_alert(false);
  nnp_fct_out->set_nnp_light_remind(false);
  nnp_fct_out->set_lateralctr_takeover(false);
  nnp_fct_out->set_nnp_scenarios_audio_play(NNPScenarios::NO_REQUEST);
  nnp_fct_out->set_nnp_scenarios(NNPScenarios::NO_REQUEST);
  nnp_fct_out->set_nnp_rino_status(
      TL::functionmanager::NNPRinoStatus::RINO_NO_REQUEST);
  // nnp_fct_out->set_paymode_confirm_feedback(0);
  // nnp_fct_out->set_spdadapt_comfirm_feedback(0);
  nnp_fct_out->set_aalc_mode(false);
  nnp_fct_out->mutable_light_signal_reqst()->set_hazardlampreqst(false);
  nnp_fct_out->mutable_light_signal_reqst()->set_highbeamreqst(false);
  nnp_fct_out->mutable_light_signal_reqst()->set_lowhighbeamreqst(false);
  nnp_fct_out->mutable_light_signal_reqst()->set_hornreqst(false);
  nnp_fct_out->mutable_light_signal_reqst()->set_lowbeamreqst(false);
  static constexpr uint32_t kClearBase = 0x103;
  const auto& cur_fct_u32_03 =
      ptr_trajectory_pb_->function_manager_out().soc_2_fct_tbd_u32_03();
  ptr_trajectory_pb_->mutable_function_manager_out()->set_soc_2_fct_tbd_u32_03(
      cur_fct_u32_03 & (~kClearBase));
  ptr_trajectory_pb_->mutable_soc_to_fct_bus()
      ->mutable_soc_to_fct_bus_u8()
      ->set_soc_close_turn_light_req(0);
  ptr_trajectory_pb_->mutable_soc_to_fct_bus()
      ->mutable_soc_to_fct_bus_u8()
      ->set_soc_close_turn_light_req(0);
  ptr_trajectory_pb_->mutable_soc_to_fct_bus()
      ->mutable_soc_to_fct_bus_u8()
      ->set_cruise_status(0x00);
  ptr_trajectory_pb_->mutable_soc_to_fct_bus()
      ->mutable_soc_to_fct_bus_u8()
      ->set_no_lane_status(0x00);
}

void CanNnpHmi::ResetOtherFctHmiOut() {
  if (ptr_trajectory_pb_ == nullptr) {
    AERROR << "ptr_trajectory_pb is nullptr";
    return;
  }
  auto* nnp_fct_out =
      ptr_trajectory_pb_->mutable_function_manager_out()->mutable_nnp_fct_out();
  nnp_fct_out->set_lateralctr_takeover(false);
  // nnp_fct_out->set_nnp_scenarios_audio_play(NNPScenarios::NO_REQUEST);
  // nnp_fct_out->set_nnp_scenarios(NNPScenarios::NO_REQUEST);
  if (nnp_fct_out->nnp_rino_status() !=
      functionmanager::NNPRinoStatus::RAMP_OUT_START) {
    nnp_fct_out->set_nnp_rino_status(
        TL::functionmanager::NNPRinoStatus::RINO_NO_REQUEST);
  }
  // nnp_fct_out->set_paymode_confirm_feedback(0);
  // nnp_fct_out->set_spdadapt_comfirm_feedback(0);
  nnp_fct_out->set_aalc_mode(false);
  nnp_fct_out->mutable_light_signal_reqst()->set_hazardlampreqst(false);
  nnp_fct_out->mutable_light_signal_reqst()->set_highbeamreqst(false);
  nnp_fct_out->mutable_light_signal_reqst()->set_lowhighbeamreqst(false);
  nnp_fct_out->mutable_light_signal_reqst()->set_hornreqst(false);
  nnp_fct_out->mutable_light_signal_reqst()->set_lowbeamreqst(false);
  // static constexpr uint32_t kClearBase = 0x103;
  // const auto& cur_fct_u32_03 =
  //     ptr_trajectory_pb_->function_manager_out().soc_2_fct_tbd_u32_03();
  // ptr_trajectory_pb_->mutable_function_manager_out()->set_soc_2_fct_tbd_u32_03(
  //     cur_fct_u32_03 & (~kClearBase));
}

void CanNnpHmi::ResetFctHmiOutByNNPSysState() {
  // nnp不是active或者override的情况下，不下发交互类的报文,只给fct更新激活条件
  if (!is_nnp_active_) {
    if (is_soc_pilot_active_) {
      ResetOtherFctHmiOut();
      return;
    }
    ResetFctHmiOut();
    is_expand_change_lane_ = false;
    expand_change_lane_dir_ = LaneChangeDir::NONE_DIR;
    change_lane_reason_ = functionmanager::HMI_NONE;
  }
}

void CanNnpHmi::UpdateNnpTrajectoryForHmi() {
  if (ptr_trajectory_pb_ == nullptr) {
    return;
  }
  static constexpr double kIsZero = -0.01;
  // 处理轨迹点转换
  auto* decision_info =
      ptr_trajectory_pb_->mutable_nnp_hmi_output()->mutable_decision_info();
  for (const auto& point : ptr_trajectory_pb_->trajectory_point()) {
    if (point.relative_time() < kIsZero) {
      continue;
    }
    auto* pos_local = decision_info->add_coords();
    pos_local->set_x(point.path_point().x());
    pos_local->set_y(point.path_point().y());
    pos_local->set_z(point.path_point().z());
  }
}

bool CanNnpHmi::UpdateLonBigCarAvoid(const Obstacle* const obs) {
  bool is_high_obs_type = obs->IsOversizedVehicle();
  if (is_nnp_active_ && is_high_obs_type) {
    const auto& object_decisions =
        ptr_trajectory_pb_->decision().object_decision();
    for (const auto& decision : object_decisions.decision()) {
      if (!decision.has_id()) {
        continue;
      }
      const auto& id = decision.perception_id();
      for (const auto& obj_decison : decision.object_decision()) {
        if (obj_decison.has_lon_nudge() && obs->PerceptionId() == id) {
          return true;
        }
      }
    }
  }
  return false;
}

bool CanNnpHmi::CheckFollowObsNeedHighLight(
    const ReferenceLineInfo* reference_line_info) {
  if (reference_line_info == nullptr) {
    return false;
  }
  static constexpr double kPerceptionMaxDistance = 100.0;
  static constexpr int32_t kFollowStableCnt = 2;
  static constexpr double kPreviewTime = 0.5;
  static constexpr double kMinPreviewDistance = 15.0;
  static int32_t follow_cnt = 0;
  static int32_t last_follow_id = -1;
  const auto& vehicle_state = reference_line_info->vehicle_state();
  const auto adc_fronts = reference_line_info->AdcSlBoundary().end_s();
  const auto* obs = reference_line_info->path_decision().Find(
      reference_line_info->GetLonFollowObsId());
  if (obs == nullptr) {
    return false;
  }

  const auto dis = obs->PerceptionSLBoundary().start_s() - adc_fronts;
  const auto middle_s = (obs->PerceptionSLBoundary().start_s() +
                         obs->PerceptionSLBoundary().end_s()) /
                        2.0;
  const auto middle_l = (obs->PerceptionSLBoundary().start_l() +
                         obs->PerceptionSLBoundary().end_l()) /
                        2.0;
  double left_lane_width = 0.0;
  double right_lane_width = 0.0;
  const auto ret =
      reference_line_info->reference_line().map_path().GetLaneWidth(
          middle_s, &left_lane_width, &right_lane_width);
  if (!ret) {
    follow_cnt = 0;
    last_follow_id = -1;
    return false;
  }
  auto ratio = fmax(vehicle_state.linear_velocity() * kPreviewTime,
                    kMinPreviewDistance) /
               (fmax(dis, 0.0001));
  ratio = fmin(1.0, ratio);
  const auto left_range = left_lane_width + left_lane_width * ratio;
  const auto right_range = right_lane_width + right_lane_width * ratio;
  if ((middle_l > left_range) || (middle_l < -right_range)) {
    return false;
  }

  // 100m以外的多过滤2帧
  if (last_follow_id != obs->PerceptionId()) {
    follow_cnt = 0;
  }
  last_follow_id = obs->PerceptionId();
  follow_cnt++;
  return dis > kPerceptionMaxDistance ? follow_cnt > kFollowStableCnt : true;
}

void CanNnpHmi::UpdateLonFollowObsHighLight() {  // NOLINT
  if (frame_ == nullptr || ptr_trajectory_pb_ == nullptr) {
    return;
  }
  // 针对纵向高亮的目标，纵向越远，对切入要求越高
  uint32_t target_perception_obs_id = 0;
  double nearest_obs_dis = 1e5;
  std::string target_obs_id;
  static uint32_t last_target_id = 0;
  const auto* reference_line_info = frame_->FindDriveReferenceLineInfo();
  if (reference_line_info == nullptr) {
    last_target_id = 0;
    return;
  }
  const auto* blocking_obs = reference_line_info->GetBlockingObstacle();
  const auto& obstcales =
      reference_line_info->path_decision().obstacles().Items();
  common::SLPoint adc_pos;
  reference_line_info->reference_line().XYToSL(
      {frame_->vehicle_state().pose().position().x(),
       frame_->vehicle_state().pose().position().y()},
      &adc_pos);
  const auto adc_front_s = adc_pos.s() + vehicle_param_.front_edge_to_center();
  const auto lon_follow_obs = reference_line_info->GetLonFollowObsId();
  static constexpr double kLBuffer = 0.2;
  if (frame_->local_view().HasFunctionManagerOut() &&
      frame_->local_view().GetFunctionManagerOut() != nullptr &&
      frame_->local_view().GetFunctionManagerOut()->fsm_state() ==
          functionmanager::MachineStateType::PERCEPTION_TYPE &&
      frame_->local_view().GetFunctionManagerOut()->perception_sub_state() ==
          functionmanager::PerceptionSubState::CRUISE_TYPE) {
    target_perception_obs_id =
        frame_->local_view().GetCruiseTargetId().empty()
            ? 0
            : frame_->local_view().GetCruiseTargetId().at(0);
  } else {
    for (const auto& obs : obstcales) {
      if (obs == nullptr || obs->IsVirtual() ||
          obs->path_st_boundary().IsEmpty() ||
          !obs->HasLongitudinalDecision() ||
          obs->LongitudinalDecision().has_ignore() ||
          obs->PerceptionSLBoundary().end_s() < adc_front_s ||
          (obs->Perception().type() ==
               perception::PerceptionObstacle::STATIC_OBSTACLE &&
           (obs->Perception().sub_type() ==
                perception::PerceptionObstacle::ST_TRAFFICCONE ||
            obs->Perception().sub_type() ==
                perception::PerceptionObstacle::ST_WATERHORSE))) {
        continue;
      }
      const auto buffer =
          last_target_id != 0 &&
                  last_target_id == static_cast<uint32_t>(obs->PerceptionId())
              ? -kLBuffer / 2
              : -kLBuffer;
      if (blocking_obs != nullptr &&
          blocking_obs->PerceptionId() == obs->PerceptionId()) {
        const auto dis = obs->PerceptionSLBoundary().start_s() - adc_front_s;
        if (DefinitelyLessEqual(dis, nearest_obs_dis)) {
          nearest_obs_dis = dis;
          target_perception_obs_id = obs->PerceptionId();
          target_obs_id = obs->Id();
        }
        continue;
      }
      if (!IsObsOnLaneWithBuffer(reference_line_info, obs, buffer, buffer)) {
        continue;
      }
      if (lon_follow_obs == obs->Id() &&
          CheckFollowObsNeedHighLight(reference_line_info)) {
        const auto dis = obs->PerceptionSLBoundary().start_s() - adc_front_s;
        if (DefinitelyLessEqual(dis, nearest_obs_dis)) {
          nearest_obs_dis = dis;
          target_perception_obs_id = obs->PerceptionId();
          target_obs_id = obs->Id();
        }
        continue;
      }
      const auto dis = obs->PerceptionSLBoundary().start_s() - adc_front_s;
      if (DefinitelyLessEqual(dis, nearest_obs_dis)) {
        nearest_obs_dis = dis;
        target_perception_obs_id = obs->PerceptionId();
        target_obs_id = obs->Id();
      }
    }
  }

  last_target_id = target_perception_obs_id;
  if (target_perception_obs_id != 0) {
    bool is_has_obs_id = false;
    auto* mutable_nnp_hmi_output = ptr_trajectory_pb_->mutable_nnp_hmi_output();
    for (int i = 0; i < mutable_nnp_hmi_output->nnp_obs_hightlight_size();
         i++) {
      auto* mutable_obs_higtlight =
          mutable_nnp_hmi_output->mutable_nnp_obs_hightlight(i);
      if (target_perception_obs_id ==
          mutable_obs_higtlight->obs_hightlight_id()) {
        mutable_obs_higtlight->set_highlight_reason(
            TL::hmi::NNPHmiOutput::LON_FOLLOW);
        is_has_obs_id = true;
      }
    }
    if (!is_has_obs_id) {
      auto* mutable_nnp_obs_higtlight =
          mutable_nnp_hmi_output->add_nnp_obs_hightlight();
      mutable_nnp_obs_higtlight->set_obs_hightlight_id(
          target_perception_obs_id);
      mutable_nnp_obs_higtlight->set_highlight_reason(
          TL::hmi::NNPHmiOutput::LON_FOLLOW);
    }
  }
  // const auto* lon_stop_obs =
  //     reference_line_info->path_decision().Find(frame_->LonStopObsId());
  // if (lon_stop_obs != nullptr &&
  //     lon_stop_obs->PerceptionSLBoundary().end_s() < adc_front_s &&
  //     lon_stop_obs->Perception().type() ==
  //         perception::PerceptionObstacle::PEDESTRIAN) {
  //   // 车后的行人要红色高亮
  //   auto* mutable_nnp_hmi_output = ptr_trajectory_pb_->mutable_nnp_hmi_output();
  //   auto* mutable_nnp_obs_higtlight =
  //       mutable_nnp_hmi_output->add_nnp_obs_hightlight();
  //   mutable_nnp_obs_higtlight->set_obs_hightlight_id(
  //       lon_stop_obs->PerceptionId());
  //   mutable_nnp_obs_higtlight->set_highlight_reason(
  //       TL::hmi::NNPHmiOutput::LON_NUDGE);
  // }
}

bool CanNnpHmi::IsObsOnLaneWithBuffer(
    const ReferenceLineInfo* reference_line_info, const Obstacle* obs,
    const double left_buffer, double right_buffer) {
  if (reference_line_info == nullptr || obs == nullptr) {
    return false;
  }
  double left_lane_width = 0.0;
  double right_lane_width = 0.0;
  const auto middle_s = (obs->PerceptionSLBoundary().start_s() +
                         obs->PerceptionSLBoundary().end_s()) /
                        2.0;
  reference_line_info->reference_line().map_path().GetLaneWidth(
      middle_s, &left_lane_width, &right_lane_width);
  const auto left_range = left_lane_width + left_buffer;
  const auto right_range = right_lane_width + right_buffer;
  return (obs->PerceptionSLBoundary().start_l() < left_range) &&
         (obs->PerceptionSLBoundary().end_l() > -right_range);
}

void CanAvpHmi::Init() {
  vehicle_param_ = common::VehicleConfigHelper::GetConfig().vehicle_param();
  reminder_x_ = vehicle_param_.front_edge_to_center() + kTurnDist;
}

void CanAvpHmi::UpdateAvoidAndFollowReminder(
    const std::shared_ptr<ADCTrajectory>& trajectory_pb) {
  if (trajectory_pb == nullptr || frame_ == nullptr || pnc_map_ == nullptr) {
    AERROR << "ptr is nullptr";
    return;
  }
  if (frame_->reference_line_info().empty()) {
    AERROR << "reference_line_info empty.";
    return;
  }
  // get stop and nudge obstacle that s < 5m
  // 优先级：避让行人 > 跟车
  bool stop_nudge_ped_flag = false;
  bool follow_flag = false;
  static double last_stop_nudge_ped_flagon_timestamp = 0.0;
  static double last_follow_flagon_timestamp = 0.0;
  // ObjectDecisions
  const auto& object_decisions = trajectory_pb->decision().object_decision();
  // repeated ObjectDecision
  if (object_decisions.decision().empty()) {
    return;
  }
  // repeted ObjectDecision(each object)
  for (const auto& obj_decision : object_decisions.decision()) {
    // filter the non obstacle case
    int32_t obj_id = obj_decision.perception_id();
    if (obj_id < 0) {
      continue;
    }
    // 用决策里的障碍物
    const auto& reference_line_info = frame_->reference_line_info().front();
    for (const auto& obstacle :
         reference_line_info.path_decision().obstacles().Items()) {
      if (obstacle == nullptr || obj_id != obstacle->PerceptionId()) {
        continue;
      }
      // 过滤掉纵向距离车头 > 5m
      const auto dist_s = obstacle->PerceptionSLBoundary().start_s() -
                          reference_line_info.AdcSlBoundary().end_s();
      if (dist_s > kAvoidDist) {
        continue;
      }

      for (const auto& cur_decision : obj_decision.object_decision()) {
        // repeated ObjectDecisionType(longitudinal and lateral)
        switch (cur_decision.object_tag_case()) {
          case TL::planning::ObjectDecisionType::ObjectTagCase::kStop:
          case TL::planning::ObjectDecisionType::ObjectTagCase::kNudge: {
            // filter non ped and vehicle
            if (obstacle->Perception().type() !=
                TL::perception::PerceptionObstacle::PEDESTRIAN) {
              continue;
            }
            stop_nudge_ped_flag = true;
            break;
          }
          case TL::planning::ObjectDecisionType::ObjectTagCase::kFollow: {
            if (obstacle->Perception().type() !=
                TL::perception::PerceptionObstacle::VEHICLE) {
              continue;
            }
            follow_flag = true;
            break;
          }
          default:
            continue;
        }
      }
    }
  }
  if (stop_nudge_ped_flag) {
    last_stop_nudge_ped_flagon_timestamp = Clock::NowInSeconds();
  } else {
    stop_nudge_ped_flag = std::abs(Clock::NowInSeconds() -
                                   last_stop_nudge_ped_flagon_timestamp) <= 1.5;
  }
  if (follow_flag) {
    last_follow_flagon_timestamp = Clock::NowInSeconds();
  } else {
    follow_flag =
        std::abs(Clock::NowInSeconds() - last_follow_flagon_timestamp) <= 1.5;
  }
  if (stop_nudge_ped_flag) {
    trajectory_pb->mutable_function_manager_out()
        ->mutable_avp_fct_out()
        ->set_iuss_state_obs(2);
  } else if (follow_flag) {
    trajectory_pb->mutable_function_manager_out()
        ->mutable_avp_fct_out()
        ->set_iuss_state_obs(1);
  }
}

void CanAvpHmi::ProcessFctOutput(
    Frame* const frame, const std::shared_ptr<hdmap::PncMap>& pnc_map,
    const std::shared_ptr<DependencyInjector>& injector,
    const std::shared_ptr<ADCTrajectory>& trajectory_pb) {
  if (trajectory_pb == nullptr || !trajectory_pb->has_function_manager_in()) {
    return;
  }
  if (trajectory_pb->header().status().error_code() != common::ErrorCode::OK &&
      trajectory_pb->function_manager_in().fct_avp_in().sys_mode() !=
          functionmanager::AvpFctIn::LOCALIZATION &&
      trajectory_pb->function_manager_in().fct_avp_in().sys_mode() !=
          functionmanager::AvpFctIn::LOCALIZATION_BACKGROUND) {
    switch (trajectory_pb->header().status().error_code()) {
      case common::ErrorCode::PLANNER_PARKING_PATHPROVIDER_ERROR:
      case common::ErrorCode::PLANNER_PARKING_PATHGENERATOR_ERROR: {
        trajectory_pb->mutable_function_manager_out()
            ->mutable_avp_fct_out()
            ->set_parking_status(functionmanager::AvpFctOut::PARKINGNOSPACE);
        break;
      }
      case common::ErrorCode::PLANNER_PARKING_COLLISION_ERROR: {
        trajectory_pb->mutable_function_manager_out()
            ->mutable_avp_fct_out()
            ->set_parking_status(functionmanager::AvpFctOut::COLLISION);
        break;
      }
      default: {
        trajectory_pb->mutable_function_manager_out()
            ->mutable_avp_fct_out()
            ->set_parking_status(functionmanager::AvpFctOut::PLANNINGFAILED);
        break;
      }
    }
    ADEBUG << "avp parking status is falied";
  }

  if (frame == nullptr || injector == nullptr || pnc_map == nullptr) {
    AERROR << "ptr is nullptr";
    return;
  }
  frame_ = frame;
  pnc_map_ = pnc_map;
  injector_ = injector;
  ptr_trajectory_pb_ = trajectory_pb;
  const auto driving_mode = injector_->vehicle_state()->driving_mode();

  if (trajectory_pb->header().status().error_code() == common::ErrorCode::OK &&
      trajectory_pb->function_manager_out().avp_fct_out().stage_type() ==
          TL::functionmanager::AvpFctOut_FsmStageType_CRUISING) {
    // update obstacle avoidance and car follow parking status
    if (driving_mode == TL::soc::Chassis::COMPLETE_AUTO_DRIVE) {
      UpdateAvoidAndFollowReminder(trajectory_pb);
    }
  }

  UpdateAvpHmiData();
  AINFO << "parking_status = "
        << trajectory_pb->function_manager_out().avp_fct_out().parking_status();
}

void CanAvpHmi::UpdateAvpHmiData() {
  const auto driving_mode = injector_->vehicle_state()->driving_mode();
  const auto& avp_to_hmi =
      injector_->planning_context()->planning_status().avp_to_hmi();
  ptr_trajectory_pb_->mutable_avp_to_hmi()->MergeFrom(avp_to_hmi);
  if (frame_->open_space_info().is_on_open_space_trajectory()) {
    // GetTrajEquation();
  } else {
    if (driving_mode == TL::soc::Chassis::COMPLETE_AUTO_DRIVE) {
      UpdateTimeAndDistance();
      UpdateTurnLeftRightReminder();
    }
  }
}

void CanAvpHmi::UpdateTBAParkBar(const double remain_distance) {
  if (ptr_trajectory_pb_ == nullptr || injector_ == nullptr ||
      frame_ == nullptr) {
    AERROR << "ptr is nullptr";
    return;
  }
  if (injector_->planning_context()
          ->planning_status()
          .function_manager_out()
          .avp_fct_out()
          .parking_status() == functionmanager::AvpFctOut::MISSIONFINISHED) {
    static constexpr double kAlmostCompleted = 11;
    ptr_trajectory_pb_->mutable_avp_to_hmi()->set_park_bar_percent(
        kAlmostCompleted);
    return;
  }
  if (frame_->GetIsStateChange()) {
    total_distance_ = remain_distance;
    last_park_bar_ = 1;
  }
  static constexpr double kMinTotalDis = 1.0;
  if (total_distance_ > kMinTotalDis) {
    double curr_precent = 1.0 - remain_distance / total_distance_;
    static constexpr int kMultiplier = 10;
    static constexpr double kBias = 0.9;
    auto current_bar = static_cast<int>(curr_precent * kMultiplier + kBias);
    current_bar = std::max(1, current_bar);
    if (current_bar > last_park_bar_) {
      last_park_bar_ = current_bar;
    }
    ptr_trajectory_pb_->mutable_avp_to_hmi()->set_park_bar_percent(
        last_park_bar_);
  }
}

void CanAvpHmi::UpdateTimeAndDistance() {
  if (ptr_trajectory_pb_ == nullptr || frame_ == nullptr ||
      pnc_map_ == nullptr || !frame_->local_view().HasChassis()) {
    AERROR << "ptr is nullptr";
    return;
  }
  const auto* best_ref_info = frame_->FindDriveReferenceLineInfo();
  if (best_ref_info == nullptr) {
    AERROR << "failed to find target reference line";
    return;
  }
  auto* avp_to_hmi = ptr_trajectory_pb_->mutable_avp_to_hmi();
  const auto& driving_mode = frame_->local_view().GetChassis()->driving_mode();
  if (frame_->GetMachineStateType() ==
          functionmanager::MachineStateType::HISTORY_TRACE_TYPE &&
      driving_mode == soc::Chassis::COMPLETE_AUTO_DRIVE) {
    double remain_distance = pnc_map_->GetDistanceFromADCToStartPoint() -
                             FLAGS_history_trace_path_extend_buffer;
    UpdateTBAParkBar(remain_distance);
    auto remain_time = static_cast<int32_t>(
        remain_distance / (fabs(best_ref_info->GetCruiseSpeed()) + kEpsilon));
    avp_to_hmi->set_nns_distance(static_cast<int32_t>(remain_distance));
    avp_to_hmi->set_park_time_remaining(remain_time);
    auto end_timestamp = static_cast<int32_t>(
        TL::common::Clock::NowInSeconds() + remain_time);
    int32_t hour = (end_timestamp + kHourDiff * kHour2Sec) / (kHour2Sec);
    hour %= kDay2Hour;
    int32_t minute = (end_timestamp % kHour2Sec);
    minute /= kHour2Min;
    // int second = end_timestamp % 60;
    avp_to_hmi->set_hour_of_day(hour);
    avp_to_hmi->set_minute_of_hour(minute);
    avp_to_hmi->set_second_of_minute(0);
  }
}

void CanAvpHmi::UpdateTurnLeftRightReminder() {
  if (ptr_trajectory_pb_ == nullptr || frame_ == nullptr ||
      pnc_map_ == nullptr) {
    AERROR << "ptr is nullptr";
    return;
  }
  if (frame_->GetMachineStateType() ==
      functionmanager::MachineStateType::HISTORY_TRACE_TYPE) {
    return;
  }
  auto* nnp_fct_out =
      ptr_trajectory_pb_->mutable_function_manager_out()->mutable_nnp_fct_out();
  auto light_dir_ctl = functionmanager::LightReq::LIGHT_OFF;
  if (frame_->reference_line_info().empty() || frame_->reference_line_info()
                                                   .front()
                                                   .path_data()
                                                   .discretized_path()
                                                   .empty()) {
    return;
  }
  common::math::Vec2d xy_point(frame_->PlanningStartPoint().path_point().x(),
                               frame_->PlanningStartPoint().path_point().y());
  common::SLPoint start_point;
  if (!frame_->reference_line_info().front().reference_line().XYToSL(
          xy_point, &start_point)) {
    return;
  }
  const double reminder_x_ = common::VehicleConfigHelper::GetConfig()
                                 .vehicle_param()
                                 .front_edge_to_center() +
                             kTurnDist;
  const std::vector<ReferencePoint> reference_points =
      frame_->reference_line_info().front().reference_line().GetReferencePoints(
          start_point.s(), start_point.s() + reminder_x_);
  double max_kappa = 0;
  for (const auto& pt : reference_points) {
    if (abs(pt.kappa()) > abs(max_kappa)) {
      max_kappa = pt.kappa();
    }
  }
  if (max_kappa > kTurnKappa) {
    light_dir_ctl = functionmanager::LightReq::LEFT_LIGHT;
  } else if (max_kappa < -1.0 * kTurnKappa) {
    light_dir_ctl = functionmanager::LightReq::RIGHT_LIGHT;
  }
  nnp_fct_out->set_light_request(light_dir_ctl);
}

void CanAvpHmi::GetTrajEquation() {
  if (ptr_trajectory_pb_ == nullptr || frame_ == nullptr) {
    AERROR << "ptr is nullptr";
    return;
  }
  if (frame_->GetTargetGear() != soc::Chassis::GEAR_DRIVE &&
      frame_->GetTargetGear() != soc::Chassis::GEAR_REVERSE) {
    AERROR << "not target gear";
    return;
  }
  bool is_forward = frame_->GetTargetGear() == soc::Chassis::GEAR_DRIVE;
  double x_max = 0.0;
  x_max = vehicle_param_.front_edge_to_center();
  double x_min = 0;
  if (!is_forward) {
    x_max = -vehicle_param_.back_edge_to_center();
  }
  const auto proto_to_can = [](const double input) {
#ifdef ISORIN
    static constexpr double kOffset = 327.68;
    static constexpr double kFactor = 10.0;
    return input * kFactor + kOffset;
#else
    return input;
#endif
  };
  auto* avp_to_hmi = ptr_trajectory_pb_->mutable_avp_to_hmi();
  static constexpr double kMinDistance = 0.2;
  if (ptr_trajectory_pb_->total_path_length() < kMinDistance) {
    avp_to_hmi->set_guild_line_xmin(proto_to_can(x_min));
    avp_to_hmi->set_guild_line_xmax(proto_to_can(x_max));
    avp_to_hmi->set_guild_line_a(proto_to_can(0.0));
    avp_to_hmi->set_guild_line_b(proto_to_can(0.0));
    avp_to_hmi->set_guild_line_c(proto_to_can(0.0));
    avp_to_hmi->set_guild_line_d(proto_to_can(0.0));
    return;
  }
  std::vector<TL::common::math::Vec2d> points{};
  for (const auto& point : ptr_trajectory_pb_->trajectory_point()) {
    double vehicle_heading = frame_->local_view().GetVehicleState()->heading();
    double dx =
        point.path_point().x() - frame_->local_view().GetVehicleState()->x();
    double dy =
        point.path_point().y() - frame_->local_view().GetVehicleState()->y();
    points.emplace_back(dx * cos(vehicle_heading) + dy * sin(vehicle_heading),
                        -dx * sin(vehicle_heading) + dy * cos(vehicle_heading));
  }
  const int N = 3;
  std::vector<double> coff{};
  coff = TL::common::math::FitPolynomial<N>(points);
  static constexpr double kMaxLimitY = 10.0;
  auto get_y = [&](double x) {
    return coff.at(3) * std::pow(x, 3) + coff.at(2) * std::pow(x, 2) +
           coff.at(1) * std::pow(x, 1) + coff.at(0);
  };
  if (std::fabs(points.back().x()) < kMinDistance ||
      std::fabs(get_y(points.back().x())) > kMaxLimitY) {
    avp_to_hmi->set_guild_line_xmin(proto_to_can(x_min));
    avp_to_hmi->set_guild_line_xmax(proto_to_can(x_max));
    avp_to_hmi->set_guild_line_a(proto_to_can(0.0));
    avp_to_hmi->set_guild_line_b(proto_to_can(0.0));
    avp_to_hmi->set_guild_line_c(proto_to_can(0.0));
    avp_to_hmi->set_guild_line_d(proto_to_can(0.0));
    return;
  }
  avp_to_hmi->set_guild_line_a(proto_to_can(coff.at(3)));
  avp_to_hmi->set_guild_line_b(proto_to_can(coff.at(2)));
  avp_to_hmi->set_guild_line_c(proto_to_can(coff.at(1)));
  avp_to_hmi->set_guild_line_d(proto_to_can(coff.at(0)));

  static constexpr double kYmax = 2.5;
  x_max = points.back().x();
  static constexpr double kStepX = 0.3;
  if (is_forward) {
    double extend_max = std::min(FLAGS_avp_max_guild_line_display, x_max);
    while (std::fabs(get_y(x_max)) < kYmax &&
           x_max < (extend_max + vehicle_param_.front_edge_to_center())) {
      x_max += kStepX;
    }
    x_max = std::min(x_max, FLAGS_avp_max_guild_line_display +
                                vehicle_param_.front_edge_to_center());

  } else {
    double extend_max = std::max(-FLAGS_avp_max_guild_line_display, x_max);
    while (std::fabs(get_y(x_max)) < kYmax &&
           x_max > (extend_max - vehicle_param_.back_edge_to_center())) {
      x_max -= kStepX;
    }
    x_max = std::max(x_max, -(FLAGS_avp_max_guild_line_display +
                              vehicle_param_.back_edge_to_center()));
  }
  avp_to_hmi->set_guild_line_xmin(proto_to_can(x_min));
  avp_to_hmi->set_guild_line_xmax(proto_to_can(x_max));
}

void CanAvpHmi::ProcessFctInput(  // NOLINT
    const std::shared_ptr<LocalView>& local_view) {
  UNUSED(local_view);
  // add parking fct state machine to send state
  AINFO << local_view->GetFunctionManagerIn()->fct_avp_in().DebugString();
}

}  // namespace planning
}  // namespace TL
