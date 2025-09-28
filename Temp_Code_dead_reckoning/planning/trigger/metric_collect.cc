/******************************************************************************
 * Copyright 2023 The TL Authors. All Rights Reserved.
 *****************************************************************************/
#include "planning/trigger/metric_collect.h"
#include <sys/types.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <fstream>
#include <limits>
#include <memory>
#include <numeric>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "common/file/file.h"
#include "common/math/double_type.h"
#include "common/time/clock.h"
#include "planning/common/real_jerk/real_jerk.h"
#include "planning/middleware/common/cpu_recorder.h"
#include "proto/fsm/avp_fct.pb.h"
#include "proto/fsm/metric.pb.h"
#include "proto/fsm/nnp_fct.pb.h"
#include "proto/perception/perception_obstacle.pb.h"
#include "proto/perception/perception_parking_lot.pb.h"
#include "proto/soc/chassis.pb.h"

namespace TL {
namespace planning {
// NOLINTBEGIN
namespace {
constexpr uint32_t kCnt = 1;
std::string kPedestrian = "Pedestrian";  // NOLINT
std::string kVehicle = "Vehicle";        // NOLINT
std::string kCone = "Cone";              // NOLINT
constexpr double kNudgeWidth = 3.0;
}  // namespace

using TL::common::Clock;
using TL::metric::NnpMetric;

MetricCollect::MetricCollect() {
  ClearMetric();
  ACHECK(TL::common::GetProtoFromFile(FLAGS_metric_config_file,
                                         &avp_metric_conf_))
      << "Failed to load obs follow time config file "
      << FLAGS_metric_config_file;
}

void MetricCollect::ClearMetric() {
  nnp_metric_.clear();
  pilot_metric_.clear();
}

void MetricCollect::InitMetric(const std::shared_ptr<LocalView>& local_view) {
  double now_timestamp = 0.0;
#ifndef ISORIN
  now_timestamp = Clock::NowInSeconds();
#else
  now_timestamp = Clock::NowInSecondsForBeiJing();
#endif
  double lat = 0.0;
  double lon = 0.0;
  double heading = 0.0;
  double height = 0.0;
  if (local_view->HasLocalization() &&
      local_view->GetLocalization()->has_pose()) {
    lat = local_view->GetLocalization()->pose().gcj02().x();
    lon = local_view->GetLocalization()->pose().gcj02().y();
    height = local_view->GetLocalization()->pose().gcj02().z();
    heading = local_view->GetLocalization()->pose().heading();
  }
  if (nnp_metric_.empty()) {
    const auto& tmp_metric = std::make_shared<NnpMetricS>();
    tmp_metric->lat_jerk.clear();
    tmp_metric->lon_jerk.clear();
    tmp_metric->start_time = now_timestamp;
    tmp_metric->coordinate.first = lat;
    tmp_metric->coordinate.second = lon;
    tmp_metric->height = height;
    tmp_metric->heading_angle = heading;
    nnp_metric_.emplace_back(tmp_metric);
  }
  if (pilot_metric_.empty()) {
    const auto& tmp_metric = std::make_shared<PilotMetric>();
    tmp_metric->lat_jerk.clear();
    tmp_metric->lon_jerk.clear();
    tmp_metric->start_time = now_timestamp;
    tmp_metric->coordinate.first = lat;
    tmp_metric->coordinate.second = lon;
    tmp_metric->height = height;
    tmp_metric->heading_angle = heading;
    pilot_metric_.emplace_back(tmp_metric);
  }
  if (avp_metric_ == nullptr) {
    avp_metric_ = std::make_unique<AvpMetric>();
  }
  if (avp_real_jerk_ == nullptr) {
    avp_real_jerk_ = std::make_unique<RealJerk>();
  }
}

void MetricCollect::UpdateMetricData(
    const std::shared_ptr<LocalView>& local_view) {
  InitMetric(local_view);
  GetCarAlias(local_view);
  GetSoftwareVersion(local_view);
  UpdateNnpPilotMetricData(local_view);
  UpdateAvpMetricData(local_view);
  CheckSaveStatus();
  MetricDataToJson();
}

void MetricCollect::GetSoftwareVersion(
    const std::shared_ptr<LocalView>& local_view) {
  if (local_view != nullptr && local_view->HasADCTrajectory()) {
    sw_ver_ = local_view->GetADCTrajectory()->function_manager_out().version();
  }
}

void MetricCollect::GetCarAlias(
    const std::shared_ptr<LocalView>& local_view) {
  if (local_view != nullptr && local_view->HasADCTrajectory()) {
    const auto& fct_out = local_view->GetADCTrajectory()->function_manager_out();
    if (fct_out.has_nnp_metric() && fct_out.nnp_metric().has_car_name()) {
      car_alias_ = fct_out.nnp_metric().car_name();
    }
  }
}

void MetricCollect::GetStopNudgeObstacle(  // NOLINT
    const std::shared_ptr<LocalView>& local_view,
    std::vector<std::tuple<std::string, int32_t, common::SLPoint>>* const
        stop_obstacle_sl,
    std::vector<std::tuple<std::string, int32_t, common::math::Vec2d>>* const
        nudge_obstacle_xy) {
  if (stop_obstacle_sl == nullptr || nudge_obstacle_xy == nullptr) {
    AERROR << "stop and nudge obstacle ptr is null";
    return;
  }
  // ObjectDecisions
  const auto& object_decisions =
      local_view->GetADCTrajectory()->decision().object_decision();
  // repeated ObjectDecision
  if (object_decisions.decision_size() <= 0) {
    return;
  }
  // planning_internal.proto massage Debug.PlanningData
  const auto& planning_data =
      local_view->GetADCTrajectory()->debug().planning_data();
  // path point
  std::vector<common::PathPoint> total_path_point;
  for (const auto& path_points : planning_data.path()) {
    if (path_points.name() != "Planning PathData") {
      continue;
    }
    for (const auto& path_point : path_points.path_point()) {
      total_path_point.push_back(path_point);
    }
    break;
  }
  TL::planning::DiscretizedPath trace_path(total_path_point);
  if (trace_path.size() < 2) {
    AERROR << "Planning PathData size less than 2";
    return;
  }
  // repeted ObjectDecision(each object)
  for (const auto& obj_decision : object_decisions.decision()) {
    // filter the non obstacle case
    int32_t obj_id = obj_decision.perception_id();
    if (obj_id < 0) {
      continue;
    }
    const auto& obstacles =
        local_view->GetPerceptionObstacles()->perception_obstacle();
    for (const auto& obstacle : obstacles) {
      const auto obstacle_id = obstacle.id();
      if (obj_id != obstacle_id) {
        continue;
      }
      // filter non ped and vehicle
      std::tuple<std::string, int32_t, common::SLPoint> cur_obstacle_sl;
      std::tuple<std::string, int32_t, common::math::Vec2d> cur_obstacle_xy;
      switch (obstacle.sub_type()) {
        case TL::perception::PerceptionObstacle::ST_PEDESTRIAN: {
          // pedestrian
          std::get<0>(cur_obstacle_sl) = kPedestrian;
          std::get<1>(cur_obstacle_sl) = obj_id;
          std::get<0>(cur_obstacle_xy) = kPedestrian;
          std::get<1>(cur_obstacle_xy) = obj_id;
          break;
        }
        case TL::perception::PerceptionObstacle::ST_CAR:
        case TL::perception::PerceptionObstacle::ST_VAN:
        case TL::perception::PerceptionObstacle::ST_BIG_TRUCK:
        case TL::perception::PerceptionObstacle::ST_BUS:
        case TL::perception::PerceptionObstacle::ST_MIDDLE_BUS:
        case TL::perception::PerceptionObstacle::ST_MINIBUS:
        case TL::perception::PerceptionObstacle::ST_PICKUP:
        case TL::perception::PerceptionObstacle::ST_AMBULANCE:
        case TL::perception::PerceptionObstacle::ST_POLICECAR:
        case TL::perception::PerceptionObstacle::ST_FIRE_ENGINE:
        case TL::perception::PerceptionObstacle::ST_SPECIAL_CAR:
        case TL::perception::PerceptionObstacle::ST_TRUCK: {
          // vehicle
          std::get<0>(cur_obstacle_sl) = kVehicle;
          std::get<1>(cur_obstacle_sl) = obj_id;
          std::get<0>(cur_obstacle_xy) = kVehicle;
          std::get<1>(cur_obstacle_xy) = obj_id;
          break;
        }
        case TL::perception::PerceptionObstacle::ST_TRAFFICCONE: {
          // cone
          std::get<0>(cur_obstacle_sl) = kCone;
          std::get<1>(cur_obstacle_sl) = obj_id;
          std::get<0>(cur_obstacle_xy) = kCone;
          std::get<1>(cur_obstacle_xy) = obj_id;
          break;
        }
        default: {
          // others, check the next obstacle
          continue;
        }
      }
      for (const auto& cur_decision : obj_decision.object_decision()) {
        // repeated ObjectDecisionType(longitudinal and lateral)
        switch (cur_decision.object_tag_case()) {
          case TL::planning::ObjectDecisionType::ObjectTagCase::kStop: {
            // 让行SL计算
            common::SLPoint pt_sl;
            double pt_x = cur_decision.stop().stop_point().x();
            double pt_y = cur_decision.stop().stop_point().y();
            if (trace_path.XYToSL(pt_x, pt_y, &pt_sl)) {
              std::get<2>(cur_obstacle_sl) = pt_sl;
              stop_obstacle_sl->push_back(cur_obstacle_sl);
            }
            break;
          }
          case TL::planning::ObjectDecisionType::ObjectTagCase::kNudge: {
            // 绕行（车体坐标系下纵向距离在车身范围内）
            const auto& vehicle_pose = local_view->GetLocalization()->pose();
            const auto& perception_obstacle_point = obstacle.position();
            double object_x = perception_obstacle_point.x();
            double object_y = perception_obstacle_point.y();
            std::pair<double, double> vehicle_pt =
                TL::common::math::ENUToFLU(
                    object_x, object_y, vehicle_pose.position().x(),
                    vehicle_pose.position().y(), vehicle_pose.heading());
            common::math::Vec2d vehicle_pt_vec(vehicle_pt.first,
                                               vehicle_pt.second);
            std::get<2>(cur_obstacle_xy) = vehicle_pt_vec;
            nudge_obstacle_xy->push_back(cur_obstacle_xy);
            break;
          }
          default:
            continue;
        }
      }
      break;
    }
  }
}

void MetricCollect::CalculateStopMetric(
    const std::vector<std::tuple<std::string, int32_t, common::SLPoint>>&
        stop_obstacle_sl) {
  // 让行指标计算，s最小的,同一个id算一次
  double stop_s = 100;
  int32_t add_stop_ind = -1;
  for (int32_t i = 0; i < stop_obstacle_sl.size(); i++) {
    const auto& obstacle = stop_obstacle_sl.at(i);
    if (stop_s > std::get<2>(obstacle).s()) {
      add_stop_ind = i;
      stop_s = std::get<2>(obstacle).s();
    }
  }
  if (add_stop_ind < 0) {
    stop_id_ = -1;
    return;
  }
  const auto& obstacle = stop_obstacle_sl.at(add_stop_ind);
  auto obstacle_type = std::get<0>(obstacle);
  if (stop_id_ == std::get<1>(obstacle)) {
    return;
  }
  if (obstacle_type == kPedestrian) {
    avp_metric_->cruise_avoidance_pedestrian_counter++;
    stop_id_ = std::get<1>(obstacle);
  } else if (obstacle_type == kVehicle) {
    avp_metric_->cruise_avoidance_vehicle_counter++;
    stop_id_ = std::get<1>(obstacle);
  }
}

void MetricCollect::CalculateNudgeMetric(
    const std::vector<std::tuple<std::string, int32_t, common::math::Vec2d>>&
        nudge_obstacle_xy) {
  // 绕行指标计算，统计所有在自车区域内的绕行目标数量
  const auto& vehicle_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  const auto max_nudge_x = vehicle_param.front_edge_to_center();
  const auto min_nudge_x = vehicle_param.back_edge_to_center();
  std::unordered_set<int32_t> nudge_ids;
  for (const auto& obstacle : nudge_obstacle_xy) {
    const auto obstacle_id = std::get<1>(obstacle);
    const auto obstacle_xy = std::get<2>(obstacle);
    // NOLINTBEGIN
    if (!(obstacle_xy.x() <= max_nudge_x && obstacle_xy.x() >= -min_nudge_x &&
          abs(obstacle_xy.y()) < kNudgeWidth)) {
      continue;
    }
    // NOLINTEND
    if (nudge_ids_.find(obstacle_id) == nudge_ids_.end()) {
      auto obstacle_type = std::get<0>(obstacle);
      if (obstacle_type == kPedestrian) {
        avp_metric_->cruise_nudge_pedestrian_counter++;
        nudge_ids.insert(obstacle_id);
      } else if (obstacle_type == kVehicle) {
        avp_metric_->cruise_nudge_vehicle_counter++;
        nudge_ids.insert(obstacle_id);
      } else if (obstacle_type == kCone) {
        avp_metric_->cruise_nudge_cone_or_barricade_counter++;
        nudge_ids.insert(obstacle_id);
      }
    } else {
      nudge_ids.insert(obstacle_id);
    }
  }
  nudge_ids_.clear();
  nudge_ids_ = nudge_ids;
}

void MetricCollect::UpdateStopNudgeMetricData(
    const std::shared_ptr<LocalView>& local_view) {
  if (!local_view->HasADCTrajectory() ||
      !local_view->GetADCTrajectory()->has_debug()) {
    AERROR << "local_view no ADCTrajectory or ADC no debug";
    return;
  }

  if (!local_view->HasFunctionManagerIn()) {
    AERROR << "local_view no FunctionManagerIn";
    return;
  }

  if (!local_view->HasFunctionManagerOut()) {
    AERROR << "local_view no FunctionManagerOut";
    return;
  }

  if (!local_view->HasLocalization()) {
    AERROR << "local_view no Localization";
    return;
  }

  std::vector<std::tuple<std::string, int32_t, common::SLPoint>>
      stop_obstacle_sl;  // (type, id, sl)
  std::vector<std::tuple<std::string, int32_t, common::math::Vec2d>>
      nudge_obstacle_xy;  // (type, id, [x, y])

  GetStopNudgeObstacle(local_view, &stop_obstacle_sl, &nudge_obstacle_xy);
  // 让行指标计算，s最小的,同一个id算一次
  CalculateStopMetric(stop_obstacle_sl);
  // 绕行指标计算，统计所有在自车区域内的绕行目标数量
  CalculateNudgeMetric(nudge_obstacle_xy);
}

void MetricCollect::UpdateNnpPilotMetricData(
    const std::shared_ptr<LocalView>& local_view) {
  NnpMetric* mutable_nnp_metric = nullptr;
  if (local_view->HasADCTrajectory()) {
    auto ptr_adc_trajectroy =
        std::const_pointer_cast<ADCTrajectory>(local_view->GetADCTrajectory());
    mutable_nnp_metric = ptr_adc_trajectroy->mutable_function_manager_out()
                             ->mutable_nnp_metric();
  }
  if (mutable_nnp_metric == nullptr) {
    AERROR << "local_view no ADCTrajectory";
    return;
  }
  is_insert_nnp_metric_ = false;
  is_insert_pilot_metric_ = false;
  if (local_view->HasChassis() && local_view->HasFunctionManagerIn() &&
      local_view->HasADCTrajectory()) {
    const auto odo_meter = local_view->GetChassis()->odometer();
    UpdateMileageMetric(odo_meter, *local_view->GetFunctionManagerIn(),
                        local_view->GetADCTrajectory()->function_manager_out(),
                        mutable_nnp_metric);
  }
  if (local_view->HasFunctionManagerIn() && local_view->HasADCTrajectory()) {
    UpdateNnpFunctionMetric(*local_view->GetFunctionManagerIn(),
                            *local_view->GetADCTrajectory(),
                            mutable_nnp_metric);
    UpdatePilotFunctionMetric(*local_view->GetFunctionManagerIn(),
                              *local_view->GetFunctionManagerOut());
    UpdateInOutRampMetric(
        *local_view->GetFunctionManagerIn(),
        local_view->GetADCTrajectory()->function_manager_out(),
        mutable_nnp_metric);
  }
  UpdateTakeoverMetric(local_view, mutable_nnp_metric);
  if (local_view->HasADCTrajectory()) {
    UpdateLaneChangeMetric(
        local_view->GetADCTrajectory()->function_manager_out(),
        mutable_nnp_metric);
    UpdateTaskMetric(*local_view->GetADCTrajectory());
    UpdateJerkMetric(local_view->GetADCTrajectory()->function_manager_out());
  }
#ifdef ISMDC
  if (local_view->HasMbdDebugFromMCU() &&
      local_view->Hasmcu_to_soc_DebugData() && local_view->HasLocalization() &&
      local_view->HasFunctionManagerIn() && local_view->HasADCTrajectory()) {
    UpdateScenarioMetric(
        *local_view->GetMbdDebugFromMCU(),
        local_view->GetADCTrajectory()->function_manager_out());
    UpdateFaultMetric(*local_view->GetLocalization(),
                      *local_view->GetFunctionManagerIn(),
                      local_view->GetADCTrajectory()->function_manager_out(),
                      *local_view->Getmcu_to_soc_DebugData());
  }
#else
  if (local_view->HasMcuToSocPnc() && local_view->HasLocalization() &&
      local_view->HasFunctionManagerIn() && local_view->HasADCTrajectory()) {
    UpdateScenarioMetric(
        *local_view->GetMcuToSocPnc(),
        local_view->GetADCTrajectory()->function_manager_out());
    if (local_view->HasAdasSomeipFromMCU() &&
        local_view->GetAdasSomeipFromMCU()->adas_someip_size() > 739) {
      double hand_torque =
          static_cast<double>(
              local_view->GetAdasSomeipFromMCU()->adas_someip(738)) /
          10.0;
      UpdateFaultMetric(*local_view->GetLocalization(),
                        *local_view->GetFunctionManagerIn(),
                        local_view->GetADCTrajectory()->function_manager_out(),
                        *local_view->GetMcuToSocPnc(), hand_torque);
    }
  }
#endif

  double lat = 0.0;
  double lon = 0.0;
  double height = 0.0;
  double heading = 0.0;
  if (local_view->HasLocalization() &&
      local_view->GetLocalization()->has_pose()) {
    lat = local_view->GetLocalization()->pose().gcj02().x();
    lon = local_view->GetLocalization()->pose().gcj02().y();
    height = local_view->GetLocalization()->pose().gcj02().z();
    heading = local_view->GetLocalization()->pose().heading();
  }
  if (is_insert_nnp_metric_) {
    InsertNnpMetric(lat, lon, heading, height);
  }
  if (is_insert_pilot_metric_) {
    InsertPilotMetric(lat, lon, heading, height);
  }
}

void MetricCollect::UpdateMileageMetric(
    double odo_meter, const functionmanager::FunctionManagerIn& fct_in,
    const functionmanager::FunctionManagerOut& fct_out,
    NnpMetric* const mutable_nnp_metric) {
  static double last_odometer = -1.0;
  static double last_nnp_active_timestamp = 0.0;
  static double last_pilot_active_timestamp = 0.0;
  static constexpr double kTimeStamp = 0.3;
  const auto& nnp_metric = nnp_metric_.back();
  const auto& pilot_metric = pilot_metric_.back();
  const auto& hdmap = fct_out.fsm_state();
  const auto& map_type = fct_out.localization_maptype();
  const auto& nnp_sys_state = fct_in.fct_nnp_in().nnp_sysstate();
  bool nnp_active_status =
      (nnp_sys_state == functionmanager::NNPSysState::NNPS_ACTIVE ||
       nnp_sys_state == functionmanager::NNPSysState::NNPS_LAT_OVERRIDE ||
       nnp_sys_state == functionmanager::NNPSysState::NNPS_LON_OVERRIDE ||
       nnp_sys_state == functionmanager::NNPSysState::NNPS_OVERRIDE);
  const auto& ta_pilot_mode = fct_in.ta_pilot_mode();
  is_nnp_active_ =
      (nnp_active_status ||
       (Clock::NowInSeconds() - last_nnp_active_timestamp < kTimeStamp));
  if (is_nnp_active_) {
    nnp_metric->driver_mode = 2;
    if (ta_pilot_mode == functionmanager::TaPilotMode::NCP) {
      nnp_metric->driver_mode = 4;
    }
  }
  bool pilot_active_status =
      (fct_in.has_adas_mode() &&
       fct_in.adas_mode() == functionmanager::AdasMode::PILOT &&
       ta_pilot_mode == functionmanager::TaPilotMode::ADAS);
  is_pilot_active_ =
      (pilot_active_status ||
       (Clock::NowInSeconds() - last_pilot_active_timestamp < kTimeStamp));
  if (nnp_active_status) {
    last_nnp_active_timestamp = Clock::NowInSeconds();
  }
  if (pilot_active_status) {
    last_pilot_active_timestamp = Clock::NowInSeconds();
  }
  double odometer_diff = 0.00000;
  if (odo_meter - last_odometer > 0.00000) {
    odometer_diff = (odo_meter - last_odometer);
  }
  if (((hdmap == functionmanager::MachineStateType::HDMAP_TYPE &&
        (map_type == navigation_hdmap::MapMsg::FUSION_NNP_MAP ||
         map_type == navigation_hdmap::MapMsg::FUSION_NCP_MAP)) ||
       nnp_active_status) &&
      last_odometer > 0.00) {
    nnp_metric->hdmap_mile = nnp_metric->hdmap_mile + odometer_diff;
    mutable_nnp_metric->set_hdmap_mile(odometer_diff);
  }
  if (nnp_active_status && last_odometer > 0.00) {
    nnp_metric->active_mile = nnp_metric->active_mile + odometer_diff;
    mutable_nnp_metric->set_active_mile(odometer_diff);
  }
  if (!nnp_active_status && pilot_active_status && last_odometer > 0.00) {
    pilot_metric->active_mile = pilot_metric->active_mile + odometer_diff;
    mutable_nnp_metric->set_pilot_mile(odometer_diff);
  }
  if (!nnp_active_status && last_odometer > 0.0) {
    pilot_metric->total_mile = pilot_metric->total_mile + odometer_diff;
    mutable_nnp_metric->set_total_mile(odometer_diff);
  }
  last_odometer = odo_meter;
}

void MetricCollect::UpdatePilotFunctionMetric(
    const functionmanager::FunctionManagerIn& fct_in,
    const functionmanager::FunctionManagerOut& fct_out) {
  const auto& pilot_metric = pilot_metric_.back();
  const auto& driver_mode = fct_in.driver_mode();
  const auto& nnp_sys_state = fct_in.fct_nnp_in().nnp_sysstate();
  const auto& nnp_scenarios = fct_out.nnp_fct_out().nnp_scenarios();
  bool nnp_active_status =
      (nnp_sys_state == functionmanager::NNPSysState::NNPS_ACTIVE ||
       nnp_sys_state == functionmanager::NNPSysState::NNPS_LAT_OVERRIDE ||
       nnp_sys_state == functionmanager::NNPSysState::NNPS_LON_OVERRIDE ||
       nnp_sys_state == functionmanager::NNPSysState::NNPS_OVERRIDE);
  static bool last_pilot_lat_lon_active = false;
  static bool is_static_obstacle_scenario_active = false;
  static bool last_pilot_active = false;
  static functionmanager::NNPScenarios last_nnp_scenarios =
      functionmanager::NNPScenarios::NO_REQUEST;
  if (is_pilot_active_ &&
      nnp_scenarios ==
          functionmanager::NNPScenarios::STATIC_OBSTACLE) {
          is_static_obstacle_scenario_active = true;  // 记忆状态
  }
  if (is_static_obstacle_scenario_active) {
      const auto& static_obj = fct_out.nnp_fct_out().lane_change_infor();
      if (static_obj == functionmanager::ChangeLaneInfor::LANE_CHANGE_CANCEL) {
          pilot_metric->static_obs_avoid = 1;
          is_static_obstacle_scenario_active = false;
      } else if (static_obj == functionmanager::ChangeLaneInfor::LANE_CHANGE_END) {
          pilot_metric->static_obs_avoid = 2;
          is_static_obstacle_scenario_active = false;  // 清空状态
      }
  }
  if (is_pilot_active_ &&
      nnp_scenarios == functionmanager::NNPScenarios::OVERSIZED_VEHICLE_ONE_SIDE &&
      last_nnp_scenarios != functionmanager::NNPScenarios::OVERSIZED_VEHICLE_ONE_SIDE) {
        pilot_metric->dynamic_obs_avoid++;
  }
  last_nnp_scenarios = nnp_scenarios;
  bool acc_active_status =
      (fct_in.has_adas_mode() &&
       fct_in.adas_mode() == functionmanager::AdasMode::ACC);
  bool is_only_lat_active =
      (driver_mode == functionmanager::DriveMode::ADAS_LAT_ACTIVE_LGT_OVERRIDE);
  bool is_only_lon_active =
      (driver_mode == functionmanager::DriveMode::ADAS_LGT_ACTIVE_LAT_OVERRIDE);
  if (last_pilot_lat_lon_active && is_only_lat_active) {
    pilot_metric->lon_override++;
  }
  if (last_pilot_lat_lon_active && is_only_lon_active) {
    pilot_metric->lat_override++;
  }
  if (last_pilot_active && acc_active_status) {
    pilot_metric->downgrade++;
  }
  if (last_pilot_active &&
      fct_in.ta_pilot_mode() != functionmanager::TaPilotMode::ADAS &&
      !nnp_active_status) {
    pilot_metric->take_over++;
  }
  last_pilot_active = (fct_in.has_adas_mode() &&
                       fct_in.adas_mode() == functionmanager::AdasMode::PILOT);
  last_pilot_lat_lon_active =
      (driver_mode == functionmanager::DriveMode::ADAS_LAT_LGT_ACTIVE);
}

void MetricCollect::UpdateNnpFunctionMetric(  // NOLINT
    const functionmanager::FunctionManagerIn& fct_in,
    const ADCTrajectory& adc_trajectory, NnpMetric* const mutable_nnp_metric) {
  const auto& nnp_metric = nnp_metric_.back();
  static bool is_static_obstacle_scenario_active = false;
  static bool last_active_status = false;
  static navigation_hdmap::MapMsg_MapType last_map_type =
      navigation_hdmap::MapMsg::INVALID;
  static functionmanager::NNPScenarios last_nnp_scenarios =
      functionmanager::NNPScenarios::NO_REQUEST;
  const auto& fct_out = adc_trajectory.function_manager_out();
  const auto& ta_pilot_mode = fct_in.ta_pilot_mode();
  const auto& nnp_original_state = fct_in.fct_nnp_in().nnp_sysstate();
  const auto& adas_mode = fct_in.adas_mode();
  const auto& fsm_state = fct_out.fsm_state();
  const auto& nnp_scenarios = fct_out.nnp_fct_out().nnp_scenarios();
  const auto& map_type = fct_out.localization_maptype();
  if (is_nnp_active_ &&
      nnp_scenarios ==
          functionmanager::NNPScenarios::STATIC_OBSTACLE) {
          is_static_obstacle_scenario_active = true;  // 记忆状态
  }
  if (is_static_obstacle_scenario_active) {
      const auto& static_obj = fct_out.nnp_fct_out().lane_change_infor();
      if (static_obj == functionmanager::ChangeLaneInfor::LANE_CHANGE_CANCEL) {
          nnp_metric->static_obs_avoid = 1;
          is_static_obstacle_scenario_active = false;
      } else if (static_obj == functionmanager::ChangeLaneInfor::LANE_CHANGE_END) {
          nnp_metric->static_obs_avoid = 2;
          is_static_obstacle_scenario_active = false;  // 清空状态
      }
  }
  if (is_nnp_active_ &&
      nnp_scenarios == functionmanager::NNPScenarios::OVERSIZED_VEHICLE_ONE_SIDE &&
      last_nnp_scenarios != functionmanager::NNPScenarios::OVERSIZED_VEHICLE_ONE_SIDE) {
        nnp_metric->dynamic_obs_avoid++;
  }
  last_nnp_scenarios = nnp_scenarios;
  if (is_nnp_active_ &&
      fsm_state == functionmanager::MachineStateType::HDMAP_TYPE &&
      last_map_type == navigation_hdmap::MapMsg::FUSION_NNP_MAP &&
      map_type == navigation_hdmap::MapMsg::PERCEP_MAP) {
    nnp_metric->internal_downgrade++;
  }
  last_map_type = map_type;
  if (last_active_status && !is_nnp_active_ &&
      ta_pilot_mode == functionmanager::TaPilotMode::ADAS) {
    const auto& location = fct_out.nnp_fct_out()
                               .nnp_activation_conditions()
                               .valid_of_lane_localization();
    if (!location) {
      nnp_metric->location_downgrade++;
      mutable_nnp_metric->set_location_downgrade(kCnt);
    } else if (adas_mode == functionmanager::AdasMode::PILOT ||
               nnp_original_state ==
                   functionmanager::NNPSysState::NNPS_NPILOT) {
      nnp_metric->lcc_downgrade++;
      mutable_nnp_metric->set_other_downgrade(kCnt);
    } else if (adas_mode == functionmanager::AdasMode::ACC) {
      nnp_metric->acc_downgrade++;
      mutable_nnp_metric->set_other_downgrade(kCnt);
    }
  }
  for (const auto& event_info : adc_trajectory.event_trigger().event_info()) {
    if (event_info.type() == 2034) {
      nnp_metric->lat_override++;
      mutable_nnp_metric->set_lat_override(kCnt);
    } else if (event_info.type() == 2035) {
      nnp_metric->lon_override++;
      mutable_nnp_metric->set_lon_override(kCnt);
    }
  }
  last_active_status = is_nnp_active_;
}

void MetricCollect::UpdateTakeoverMetric(
    const std::shared_ptr<LocalView>& local_view,
    metric::NnpMetric* mutable_nnp_metric) {
  if (!local_view->HasADCTrajectory()) {
    return;
  }
  const auto& nnp_metric = nnp_metric_.back();
  static double last_take_over_remind = -1.0;
  const auto& adc_trajectory = local_view->GetADCTrajectory();
  const auto& fct_out = adc_trajectory->function_manager_out();
  const auto& nnp_rino_status = fct_out.nnp_fct_out().nnp_rino_status();
  const auto& nnp_lon_take_over = fct_out.soc_2_fct_tbd_u32_03();
  const auto& nnp_switch_status =
      adc_trajectory->function_manager_in().nnp_switch_conditions();
  const auto& nnp_software_fault = fct_out.nnp_fct_out().nnp_software_fault();
  bool is_fm_fault =
      (nnp_switch_status.has_fm_is_nnp_fault() &&
       nnp_switch_status.fm_is_nnp_fault()) ||
      (nnp_switch_status.has_is_planning_count_ok() &&
       nnp_switch_status.is_planning_count_ok()) ||
      (nnp_switch_status.has_is_nnp_state_system_fault() &&
       nnp_switch_status.is_nnp_state_system_fault()) ||
      (nnp_switch_status.has_ctr_enable() && nnp_switch_status.ctr_enable()) ||
      !nnp_software_fault.planning_success() ||
      !nnp_software_fault.plan_trajectory_success();
  static constexpr uint32_t kLonTakeOver = 0x01;
  static constexpr double kMinTimeStamp = 60;  // 60s
  const auto& fct_in = adc_trajectory->function_manager_in();
  bool is_exit_function =
      fct_in.has_nnp_hmi_signals() &&
      fct_in.nnp_hmi_signals().lateralctrhandoffreleasewarning();
  if (nnp_rino_status == functionmanager::NNPRinoStatus::TASK_TAKE_OVER_REQ ||
      (nnp_lon_take_over & kLonTakeOver) > 0 || is_exit_function) {
    last_take_over_remind = Clock::NowInSeconds();
  }
  bool is_ok_take_over_remind =
      (Clock::NowInSeconds() - last_take_over_remind < kMinTimeStamp);
  const auto& trigger_event = adc_trajectory->event_trigger();
  for (const auto& event_info : trigger_event.event_info()) {
    if (event_info.type() == 2014U) {
      if (is_fm_fault) {
        nnp_metric->fault_take_over++;
        mutable_nnp_metric->set_take_over(kCnt);
      } else if (is_ok_take_over_remind) {
        nnp_metric->key_take_over++;
        mutable_nnp_metric->set_take_over(kCnt);
      } else {
        nnp_metric->take_over++;
        mutable_nnp_metric->set_take_over(kCnt);
      }
      return;
    }
  }
}

void MetricCollect::UpdateInOutRampMetric(  // NOLINT
    const functionmanager::FunctionManagerIn& fct_in,
    const functionmanager::FunctionManagerOut& fct_out,
    NnpMetric* const mutable_nnp_metric) {
  static int is_takeover_to_ramp_cnt = 0;
  static int is_downgrade_to_ramp_cnt = 0;
  static int is_lat_intervene_to_ramp_cnt = 0;
  static int is_lon_intervene_to_ramp_cnt = 0;
  static bool front_is_ramp = false;
  static int is_takeover_to_mainroad_cnt = 0;
  static int is_downgrade_to_mainroad_cnt = 0;
  static int is_lat_intervene_to_mainroad_cnt = 0;
  static int is_lon_intervene_to_mainroad_cnt = 0;
  static bool front_is_mainroad = false;
  const auto& driver_mode = fct_in.driver_mode();
  const auto& nnp_metric = nnp_metric_.back();
  const auto& to_ramp_len = fct_out.nnp_fct_out().nnp_d_distance2_downramp_sg();
  const auto& to_main_road_len =
      fct_out.nnp_fct_out().nnp_d_distance2_onramp_sg();
  const auto& road_type = fct_out.road_type();
  static constexpr double kMaxDistance = 200;    // m
  static constexpr double kMidDistance = 50;     // m
  static constexpr double kMinDistance = 0.001;  // m
  if (is_nnp_active_ && to_ramp_len < kMaxDistance &&
      to_ramp_len > kMinDistance) {
    front_is_ramp = true;
  } else if (to_ramp_len > kMaxDistance) {
    front_is_ramp = false;
    is_takeover_to_ramp_cnt = 0;
    is_downgrade_to_ramp_cnt = 0;
    is_lat_intervene_to_ramp_cnt = 0;
    is_lon_intervene_to_ramp_cnt = 0;
  }
  if (front_is_ramp && !is_nnp_active_ && !is_pilot_active_) {
    is_takeover_to_ramp_cnt++;
  }
  if (front_is_ramp && !is_nnp_active_ && is_pilot_active_) {
    is_downgrade_to_ramp_cnt++;
  }
  if (front_is_ramp &&
      driver_mode == functionmanager::DriveMode::NNP_LAT_ACTIVE_LGT_OVERRIDE) {
    is_lon_intervene_to_ramp_cnt++;
  }
  if (front_is_ramp &&
      driver_mode == functionmanager::DriveMode::NNP_LGT_ACTIVE_LAT_OVERRIDE) {
    is_lat_intervene_to_ramp_cnt++;
  }
  if (front_is_ramp && (road_type != hdmap::RoadSection::MultipleCarriageWay &&
                        road_type != hdmap::RoadSection::SingleCarriageWay)) {
    if (is_takeover_to_ramp_cnt == 0 && is_downgrade_to_ramp_cnt == 0 &&
        is_lat_intervene_to_ramp_cnt == 0 &&
        is_lon_intervene_to_ramp_cnt == 0) {
      nnp_metric->in_ramp_success++;
      mutable_nnp_metric->set_in_ramp_success(kCnt);
    } else {
      u_int32_t fail_reason = 0;
      if (is_lon_intervene_to_ramp_cnt > 0) {
        fail_reason = 3;
      }
      if (is_lat_intervene_to_ramp_cnt > 0) {
        fail_reason = 2;
      }
      if (is_downgrade_to_ramp_cnt > 0) {
        fail_reason = 4;
      }
      if (is_takeover_to_ramp_cnt > 0) {
        fail_reason = 1;
      }
      if (fail_reason > 0) {
        nnp_metric->in_ramp_fail = fail_reason;
        mutable_nnp_metric->set_in_ramp_fail(fail_reason);
      }
    }
    front_is_ramp = false;
    is_takeover_to_ramp_cnt = 0;
    is_downgrade_to_ramp_cnt = 0;
    is_lat_intervene_to_ramp_cnt = 0;
    is_lon_intervene_to_ramp_cnt = 0;
    is_insert_nnp_metric_ = true;
  }

  const auto& light_request = fct_out.nnp_fct_out().light_request();
  if (is_nnp_active_ && to_main_road_len < kMidDistance &&
      to_main_road_len > kMinDistance) {
    front_is_mainroad = true;
  } else if (to_main_road_len > kMidDistance) {
    front_is_mainroad = false;
    is_takeover_to_mainroad_cnt = 0;
    is_downgrade_to_mainroad_cnt = 0;
    is_lat_intervene_to_mainroad_cnt = 0;
    is_lon_intervene_to_mainroad_cnt = 0;
  }
  if (front_is_mainroad && !is_nnp_active_ && !is_pilot_active_) {
    is_takeover_to_mainroad_cnt++;
  }
  if (front_is_mainroad && !is_nnp_active_ && is_pilot_active_) {
    is_downgrade_to_mainroad_cnt++;
  }
  if (front_is_mainroad &&
      driver_mode == functionmanager::DriveMode::NNP_LAT_ACTIVE_LGT_OVERRIDE) {
    is_lon_intervene_to_mainroad_cnt++;
  }
  if (front_is_mainroad &&
      driver_mode == functionmanager::DriveMode::NNP_LGT_ACTIVE_LAT_OVERRIDE) {
    is_lat_intervene_to_mainroad_cnt++;
  }
  bool is_ok = (front_is_mainroad &&
                (road_type == hdmap::RoadSection::MultipleCarriageWay ||
                 road_type == hdmap::RoadSection::SingleCarriageWay));
  if (is_ok && light_request != functionmanager::LightReq::LEFT_LIGHT &&
      light_request != functionmanager::LightReq::RIGHT_LIGHT) {
    if (is_takeover_to_mainroad_cnt == 0 && is_downgrade_to_mainroad_cnt == 0 &&
        is_lat_intervene_to_mainroad_cnt == 0 &&
        is_lon_intervene_to_mainroad_cnt == 0) {
      nnp_metric->in_mainroad_success++;
      mutable_nnp_metric->set_in_mainroad_success(kCnt);
    } else {
      u_int32_t fail_reason = 0;
      if (is_lon_intervene_to_mainroad_cnt > 0) {
        fail_reason = 3;
      }
      if (is_lat_intervene_to_mainroad_cnt > 0) {
        fail_reason = 2;
      }
      if (is_downgrade_to_mainroad_cnt > 0) {
        fail_reason = 4;
      }
      if (is_takeover_to_mainroad_cnt > 0) {
        fail_reason = 1;
      }
      if (fail_reason > 0) {
        nnp_metric->in_mainroad_fail = fail_reason;
        mutable_nnp_metric->set_in_mainroad_fail(fail_reason);
      }
    }
    front_is_mainroad = false;
    is_takeover_to_mainroad_cnt = 0;
    is_downgrade_to_mainroad_cnt = 0;
    is_lat_intervene_to_mainroad_cnt = 0;
    is_lon_intervene_to_mainroad_cnt = 0;
    is_insert_nnp_metric_ = true;
  }
  if (is_insert_nnp_metric_) {
    double now_timestamp = 0.0;
#ifndef ISORIN
    now_timestamp = Clock::NowInSeconds();
#else
    now_timestamp = Clock::NowInSecondsForBeiJing();
#endif
    nnp_metric->end_time = now_timestamp;
  }
}

void MetricCollect::UpdateLaneChangeMetric(
    const functionmanager::FunctionManagerOut& fct_out,
    NnpMetric* const mutable_nnp_metric) {
  if (!is_nnp_active_ && !is_pilot_active_) {
    return;
  }
  const auto& nnp_metric = nnp_metric_.back();
  const auto& pilot_metric = pilot_metric_.back();
  const auto& lane_change_reason =
      fct_out.hmi_lane_change_debug().hmi_lane_change_reason();
  const auto& lane_change_infor = fct_out.nnp_fct_out().lane_change_infor();
  const auto& cancel_lane_change_reason =
      fct_out.hmi_lane_change_debug().hmi_cancel_lane_change_reason();
  const auto& hmi_lane_change_status =
      fct_out.hmi_lane_change_debug().hmi_lane_change_status();
  bool is_current_opt_succeed = fct_out.hmi_lane_change_debug().is_current_opt_succeed();
  static bool last_is_current_opt_succeed = false;
  static functionmanager::HmiChangeLaneReason hmi_lane_change_reason =
      functionmanager::HmiChangeLaneReason::HMI_NONE;
  static bool is_ok_deal = false;
  if (hmi_lane_change_reason ==
          functionmanager::HmiChangeLaneReason::HMI_NONE &&
      (lane_change_infor ==
           functionmanager::ChangeLaneInfor::LANE_CHANGE_PENDING ||
       lane_change_infor ==
           functionmanager::ChangeLaneInfor::LANE_CHANGE_ONGOING)) {
    hmi_lane_change_reason = lane_change_reason;
    is_ok_deal = false;
  }
  bool is_lane_change_success = false;
  if (lane_change_infor == functionmanager::ChangeLaneInfor::LANE_CHANGE_END) {
    is_lane_change_success = true;
    is_ok_deal = true;
  } else if ((last_is_current_opt_succeed && !is_current_opt_succeed) ||
            hmi_lane_change_status ==
                functionmanager::HmiChangeLaneStatus::HMI_CHANGE_LANE_CANCELED &&
            cancel_lane_change_reason ==
                functionmanager::CancelLaneChangeReason::SAFETY_DECIDER_CANCEL_LANE_CHANGE) {
    is_ok_deal = true;
  }
  last_is_current_opt_succeed = is_current_opt_succeed;
  if (hmi_lane_change_reason ==
          functionmanager::HmiChangeLaneReason::HMI_NONE ||
      !is_ok_deal) {
    return;
  }
  switch (hmi_lane_change_reason) {
    case functionmanager::HmiChangeLaneReason::HMI_TURN_SIGNAL_SWITCH:
      if (is_lane_change_success) {
        nnp_metric->switch_lane_change_success++;
        pilot_metric->switch_lane_change_success++;
        mutable_nnp_metric->set_switch_lane_change_success(kCnt);
      } else {
        nnp_metric->switch_lane_change_fail++;
        pilot_metric->switch_lane_change_fail++;
        mutable_nnp_metric->set_switch_lane_change_fail(kCnt);
      }
      hmi_lane_change_reason = functionmanager::HmiChangeLaneReason::HMI_NONE;
      break;
    case functionmanager::HmiChangeLaneReason::HMI_EFFICIENCY:
      if (is_lane_change_success) {
        nnp_metric->effi_lane_change_success++;
        // pilot_metric->effi_lane_change_success++;
        mutable_nnp_metric->set_effi_lane_change_success(kCnt);
      } else {
        nnp_metric->effi_lane_change_fail++;
        // pilot_metric->effi_lane_change_fail++;
        mutable_nnp_metric->set_effi_lane_change_fail(kCnt);
      }
      hmi_lane_change_reason = functionmanager::HmiChangeLaneReason::HMI_NONE;
      break;
    case functionmanager::HmiChangeLaneReason::HMI_NAVIGATION:
      if (is_lane_change_success) {
        nnp_metric->navi_lane_change_success++;
        // pilot_metric->navi_lane_change_success++;
        mutable_nnp_metric->set_navi_lane_change_success(kCnt);
      } else {
        nnp_metric->navi_lane_change_fail++;
        // pilot_metric->navi_lane_change_fail++;
        mutable_nnp_metric->set_navi_lane_change_fail(kCnt);
      }
      hmi_lane_change_reason = functionmanager::HmiChangeLaneReason::HMI_NONE;
      break;
    case functionmanager::HmiChangeLaneReason::HMI_AUDIO_REQUEST:
      if (is_lane_change_success) {
        nnp_metric->audio_lane_change_success++;
        pilot_metric->audio_lane_change_success++;
        mutable_nnp_metric->set_audio_lane_change_success(kCnt);
      } else {
        nnp_metric->audio_lane_change_fail++;
        pilot_metric->audio_lane_change_fail++;
        mutable_nnp_metric->set_audio_lane_change_fail(kCnt);
      }
      hmi_lane_change_reason = functionmanager::HmiChangeLaneReason::HMI_NONE;
      break;
    default:
      hmi_lane_change_reason = functionmanager::HmiChangeLaneReason::HMI_NONE;
      break;
  }
}

void MetricCollect::UpdateTaskMetric(const ADCTrajectory& adc_trajectory) {
  if (!is_nnp_active_ && !is_pilot_active_) {
    all_cpu_used_.clear();
    planning_cpus_.clear();
    return;
  }
  static constexpr double kMinTimeout = 100.0;
  static constexpr double kMaxTimeout = 120.0;
  const auto& nnp_metric = nnp_metric_.back();
  const auto& pilot_metric = pilot_metric_.back();
  const auto& latency_stats = adc_trajectory.latency_stats();
  const auto& mdc_cpu_info =
      adc_trajectory.debug().planning_data().mdc_cpu_info();
  planning_cpus_.emplace_back(mdc_cpu_info.planning_cpu());
  all_cpu_used_.emplace_back(mdc_cpu_info.all_cpu_used());
  if (planning_cpus_.size() == 10) {
    double planning_cpu =
        std::accumulate(planning_cpus_.begin(), planning_cpus_.end(), 0.0) /
        50.0;
    if (planning_cpu > 1.0) {
      nnp_metric->planning_cpu_err = planning_cpu;
      pilot_metric->planning_cpu_err = planning_cpu;
      is_insert_nnp_metric_ = true;
      is_insert_pilot_metric_ = is_pilot_active_;
    }
    planning_cpus_.clear();
  }
  if (all_cpu_used_.size() == 50) {
    double all_cpu =
        std::accumulate(all_cpu_used_.begin(), all_cpu_used_.end(), 0.0) / 50.0;
    if (all_cpu > 1.0) {
      nnp_metric->all_cpu_err = all_cpu;
      pilot_metric->all_cpu_err = all_cpu;
      is_insert_nnp_metric_ = true;
      is_insert_pilot_metric_ = is_pilot_active_;
    }
    all_cpu_used_.clear();
  }
  if (latency_stats.total_time_ms() > kMinTimeout) {
    is_insert_nnp_metric_ = true;
    is_insert_pilot_metric_ = is_pilot_active_;
    nnp_metric->total_timeout = latency_stats.total_time_ms();
    pilot_metric->total_timeout = latency_stats.total_time_ms();
  }
  for (const auto& task_stats : latency_stats.task_stats()) {
    if (task_stats.time_ms() < kMinTimeout) {
      continue;
    }
    if (task_stats.name() == "total_regular_planning_time" &&
        task_stats.time_ms() > kMaxTimeout) {
      nnp_metric->total_regular_time = task_stats.time_ms();
      pilot_metric->total_regular_time = task_stats.time_ms();
    }
    if (task_stats.name() == "PREDICTION") {
      nnp_metric->prediction_timeout = task_stats.time_ms();
      pilot_metric->prediction_timeout = task_stats.time_ms();
    }
    if (task_stats.name() == "RUN_ONCE") {
      nnp_metric->run_once_time = task_stats.time_ms();
      pilot_metric->run_once_time = task_stats.time_ms();
    }
    is_insert_nnp_metric_ = true;
    is_insert_pilot_metric_ = is_pilot_active_;
  }
  double now_timestamp = 0.0;
#ifndef ISORIN
  now_timestamp = Clock::NowInSeconds();
#else
  now_timestamp = Clock::NowInSecondsForBeiJing();
#endif
  if (is_insert_nnp_metric_) {
    nnp_metric->end_time = now_timestamp;
  }
  if (is_insert_pilot_metric_) {
    pilot_metric->end_time = now_timestamp;
  }
}

void MetricCollect::UpdateJerkMetric(  // NOLINT
    const functionmanager::FunctionManagerOut& fct_out) {
  if (!is_nnp_active_ && !is_pilot_active_) {
    return;
  }
  const auto& nnp_metric = fct_out.nnp_metric();
  const auto& mutable_nnp_metric = nnp_metric_.back();
  const auto& pilot_metric = pilot_metric_.back();

  if (nnp_metric.has_jerk_err_info()) {
    const auto& jerk_err_info = nnp_metric.jerk_err_info();

    if (jerk_err_info.has_lat()) {
      if (is_nnp_active_) {
        mutable_nnp_metric->lat_jerk.emplace_back(jerk_err_info.lat());
      }
      if (is_pilot_active_) {
        pilot_metric->lat_jerk.emplace_back(jerk_err_info.lat());
      }
    }
    if (jerk_err_info.has_lon()) {
      const auto& filter_acc = nnp_metric.filter_acc_info();
      if (is_nnp_active_) {
        if (filter_acc.lon_acc() < -1.34) {
          mutable_nnp_metric->emerger_braking++;
        }
        mutable_nnp_metric->lon_jerk.emplace_back(jerk_err_info.lon());
      }
      if (is_pilot_active_) {
        pilot_metric->lon_jerk.emplace_back(jerk_err_info.lon());
      }
    }
  }
}

#ifdef ISMDC
void MetricCollect::UpdateScenarioMetric(
    const control::MbdDebugFromMCU& control,
    const functionmanager::FunctionManagerOut& fct_out)
#else
void MetricCollect::UpdateScenarioMetric(
    const control::McuToSocPnc& control,
    const functionmanager::FunctionManagerOut& fct_out)
#endif
{
  if (!is_nnp_active_) {
    return;
  }
  const auto& nnp_metric = nnp_metric_.back();
  const auto& road_type = fct_out.road_type();
  const auto& adc_is_in_tunel = fct_out.adc_is_in_tunnel();
  const auto& curv =
      fct_out.nnp_fct_out().crrent_lane_mg().nnp_crv_crrntlanecurve_sg();
  const auto& scenario = fct_out.nnp_fct_out().nnp_scenarios();
  nnp_metric->scenario_obj.first.clear();
  if (road_type == hdmap::RoadSection::Ramp ||
      road_type == hdmap::RoadSection::SlipRoad ||
      road_type == hdmap::RoadSection::RoundaboutCircle ||
      road_type == hdmap::RoadSection::Other ||
      road_type == hdmap::RoadSection::JCT ||
      road_type == hdmap::RoadSection::Service) {
    nnp_metric->scenario_obj.first.emplace_back(ScenarioEnum::RAMP);
  } else {
    nnp_metric->scenario_obj.first.emplace_back(ScenarioEnum::MAINROAD);
  }
  if (adc_is_in_tunel) {
    nnp_metric->scenario_obj.first.emplace_back(ScenarioEnum::TUNNEL);
  }
  if (std::fabs(curv) > 0.005) {
    nnp_metric->scenario_obj.first.emplace_back(ScenarioEnum::LARGE_CURV);
  }
  if (scenario == functionmanager::NNPScenarios::OVERSIZED_VEHICLE_ONE_SIDE) {
    nnp_metric->scenario_obj.first.emplace_back(ScenarioEnum::CAR_AVOID);
  }
  static constexpr uint32_t kCtrlErr = 0x01;
  static bool last_is_ctrl_err = false;
#ifdef ISMDC
  const auto& is_ctrl_err =
      (control.ctrl_dec_debug().ctrldec_ctrl_err() & kCtrlErr) > 0;
#else
  const auto& is_ctrl_err = (control.ctrldiag().ctrl_err_u8()) > 0;
#endif
  if (last_is_ctrl_err != is_ctrl_err && is_ctrl_err) {
    nnp_metric->scenario_obj.first.emplace_back(ScenarioEnum::DRIVING_DRAGON);
    last_is_ctrl_err = is_ctrl_err;
  }
  double now_timestamp = 0.0;
#ifndef ISORIN
  now_timestamp = Clock::NowInSeconds();
#else
  now_timestamp = Clock::NowInSecondsForBeiJing();
#endif
  nnp_metric->scenario_obj.second = now_timestamp;
}

//
#ifdef ISMDC
void MetricCollect::UpdateFaultMetric(  // NOLINT
    const localization::Localization& location,
    const functionmanager::FunctionManagerIn& fct_in,
    const functionmanager::FunctionManagerOut& fct_out,
    const common::mcu_to_soc_DebugData& adas)
#else
void MetricCollect::UpdateFaultMetric(  // NOLINT
    const localization::Localization& location,
    const functionmanager::FunctionManagerIn& fct_in,
    const functionmanager::FunctionManagerOut& fct_out,
    const control::McuToSocPnc& adas, const double hand_torque)
#endif
{
  if (!is_nnp_active_) {
    return;
  }
  double now_timestamp = 0.0;
#ifndef ISORIN
  now_timestamp = Clock::NowInSeconds();
#else
  now_timestamp = Clock::NowInSecondsForBeiJing();
#endif
  const auto& nnp_metric = nnp_metric_.back();
  const auto& nnp_software_fault = fct_out.nnp_fct_out().nnp_software_fault();
  static bool last_nnp_software_fault = false;
  bool nnp_software_fault_status =
      fct_in.has_nnp_switch_conditions() &&
      fct_in.nnp_switch_conditions().has_is_planning_count_ok() &&
      fct_in.nnp_switch_conditions().is_planning_count_ok();
  nnp_software_fault_status = nnp_software_fault_status ||
                              !nnp_software_fault.plan_trajectory_success() ||
                              !nnp_software_fault.planning_success();
  if (last_nnp_software_fault != nnp_software_fault_status &&
      nnp_software_fault_status) {
    nnp_metric->fault_obj.first = 501;
    nnp_metric->fault_obj.second = now_timestamp;
  }
  static bool last_is_system_fault = false;
  bool is_system_fault = fct_in.has_nnp_switch_conditions() &&
                         fct_in.nnp_switch_conditions().has_fm_is_nnp_fault() &&
                         fct_in.nnp_switch_conditions().fm_is_nnp_fault();
  if (!last_is_system_fault && is_system_fault) {
    nnp_metric->fault_obj.first = 509;
    nnp_metric->fault_obj.second = now_timestamp;
  }
  last_is_system_fault = is_system_fault;
  static bool last_is_planning_timeout = false;
  bool is_planning_timeout =
      fct_in.has_nnp_switch_conditions() &&
      fct_in.nnp_switch_conditions().has_is_planning_count_ok() &&
      fct_in.nnp_switch_conditions().is_planning_count_ok();
  if (!last_is_planning_timeout && is_planning_timeout) {
    nnp_metric->fault_obj.first = 510;
    nnp_metric->fault_obj.second = now_timestamp;
  }
  last_is_planning_timeout = is_planning_timeout;
  last_nnp_software_fault = nnp_software_fault_status;
  const auto& pilot_state = fct_in.fct_nnp_in().npilot_state();
  const auto& acc_state = fct_in.fct_nnp_in().acc_state();
  static functionmanager::FctToNnpInput::NPILOT_State last_npilot_state =
      functionmanager::FctToNnpInput::PILOT_OFF;
  if (last_npilot_state == functionmanager::FctToNnpInput::PILOT_ACTIVE &&
      pilot_state == functionmanager::FctToNnpInput::PILOT_FAULT) {
    nnp_metric->fault_obj.first = 502;
    nnp_metric->fault_obj.second = now_timestamp;
  }
  last_npilot_state = pilot_state;
  static functionmanager::FctToNnpInput::ADCS8_ACCState last_acc_state =
      functionmanager::FctToNnpInput::ACC_OFF;
  if (last_acc_state == functionmanager::FctToNnpInput::ACC_ACTIVE &&
      acc_state == functionmanager::FctToNnpInput::ACC_FAULT) {
    nnp_metric->fault_obj.first = 503;
    nnp_metric->fault_obj.second = now_timestamp;
  }
  last_acc_state = acc_state;
  static bool last_ctrl_err = false;
  bool ctrl_err = fct_in.nnp_switch_conditions().has_ctr_enable() &&
                  fct_in.nnp_switch_conditions().ctr_enable();
  if (last_ctrl_err != ctrl_err && ctrl_err) {
    nnp_metric->fault_obj.first = 504;
    nnp_metric->fault_obj.second = now_timestamp;
  }
  last_ctrl_err = ctrl_err;
#ifdef ISMDC
  const auto& eps1_torsionbartorque =
      adas.radptrin_veh_can().eps1_torsionbartorque();
  const auto& eps2_adas_available =
      adas.radptrin_veh_can().eps2_adas_available();
  static bool last_eps_status = false;
  bool eps_status =
      eps2_adas_available == 0 && std::fabs(eps1_torsionbartorque) < 1.0;
  if (last_eps_status != eps_status && eps_status) {
    nnp_metric->fault_obj.first = 505;
    nnp_metric->fault_obj.second = now_timestamp;
  }
  last_eps_status = eps_status;
#else
  static double last_hand_torque = 0.0;
  if (hand_torque > 2.0) {
    last_hand_torque = Clock::NowInSeconds();
  }
  const auto& eps1_torsionbartorque =
      adas.can_input_from_mcu().eps1_torsionbartorque();
  const auto& eps2_adas_available =
      adas.can_input_from_mcu().eps2_adas_available();
  static bool last_eps_status = false;
  bool eps_status = eps2_adas_available == 0 &&
                    std::fabs(eps1_torsionbartorque) < 1.0 &&
                    Clock::NowInSeconds() - last_hand_torque < 3.0;
  if (last_eps_status != eps_status && eps_status) {
    nnp_metric->fault_obj.first = 505;
    nnp_metric->fault_obj.second = now_timestamp;
  }
  last_eps_status = eps_status;
#endif
  static uint32_t last_location_state = 0;
  const auto& location_state = location.location_state();
  bool is_add_ok =
      (location_state >= 101 && location_state <= 150) || location_state == 12;
  if (is_add_ok && last_location_state != location_state) {
    nnp_metric->fault_obj.first = location_state;
    nnp_metric->fault_obj.second = now_timestamp;
    last_location_state = location_state;
  }
#ifdef ISMDC
  const auto& tcs_active = adas.radptrin_veh_can().idb7_tcsactive();
  const auto& tcs_fail = adas.radptrin_veh_can().idb7_tcsfail();
  const auto& abs_active = adas.radptrin_veh_can().idb7_absactive();
  const auto& abs_fail = adas.radptrin_veh_can().idb7_absfail();
  const auto& esc_active = adas.radptrin_veh_can().idb7_escactive();
  const auto& esc_fail = adas.radptrin_veh_can().idb7_escfail();
#else
  const auto& tcs_active = adas.can_input_from_mcu().idb7_tcsactive();
  const auto& tcs_fail = adas.can_input_from_mcu().idb7_tcsfail();
  const auto& abs_active = adas.can_input_from_mcu().idb7_absactive();
  const auto& abs_fail = adas.can_input_from_mcu().idb7_absfail();
  const auto& esc_active = adas.can_input_from_mcu().idb7_escactive();
  const auto& esc_fail = adas.can_input_from_mcu().idb7_escfail();
#endif
  bool is_abs_err = abs_active != 0 || abs_fail != 0;
  bool is_esc_err = esc_active != 0 || esc_fail != 0;
  bool is_tcs_err = tcs_active != 0 || tcs_fail != 0;
  static bool last_is_abs_err = false;
  static bool last_is_tcs_err = false;
  static bool last_is_esc_err = false;
  if (!last_is_abs_err && is_abs_err) {
    nnp_metric->fault_obj.first = 508;
    nnp_metric->fault_obj.second = now_timestamp;
  }
  if (!last_is_tcs_err && is_tcs_err) {
    nnp_metric->fault_obj.first = 507;
    nnp_metric->fault_obj.second = now_timestamp;
  }
  if (!last_is_esc_err && is_esc_err) {
    nnp_metric->fault_obj.first = 506;
    nnp_metric->fault_obj.second = now_timestamp;
  }
  last_is_abs_err = is_abs_err;
  last_is_esc_err = is_esc_err;
  last_is_tcs_err = is_tcs_err;
  is_insert_nnp_metric_ = nnp_metric->fault_obj.first > 0;
  if (is_insert_nnp_metric_) {
    nnp_metric->end_time = now_timestamp;
  }
}

void MetricCollect::InsertNnpMetric(const double lat, const double lon, const double heading, const double height) {
  const auto& tmp_metric = std::make_shared<NnpMetricS>();
  tmp_metric->lat_jerk.clear();
  tmp_metric->lon_jerk.clear();
  double now_timestamp = 0.0;
#ifndef ISORIN
  now_timestamp = Clock::NowInSeconds();
#else
  now_timestamp = Clock::NowInSecondsForBeiJing();
#endif
  if (!nnp_metric_.empty()) {
    nnp_metric_.back()->end_time = now_timestamp;
  }
  tmp_metric->start_time = now_timestamp;
  tmp_metric->coordinate.first = lat;
  tmp_metric->coordinate.second = lon;
  tmp_metric->height = height;
  tmp_metric->heading_angle = heading;
  nnp_metric_.emplace_back(tmp_metric);
}

void MetricCollect::InsertPilotMetric(const double lat, const double lon, const double heading, const double height) {
  const auto& tmp_metric = std::make_shared<PilotMetric>();
  tmp_metric->lat_jerk.clear();
  tmp_metric->lon_jerk.clear();
  double now_timestamp = 0.0;
#ifndef ISORIN
  now_timestamp = Clock::NowInSeconds();
#else
  now_timestamp = Clock::NowInSecondsForBeiJing();
#endif
  if (!pilot_metric_.empty()) {
    pilot_metric_.back()->end_time = now_timestamp;
  }
  tmp_metric->start_time = now_timestamp;
  tmp_metric->coordinate.first = lat;
  tmp_metric->coordinate.second = lon;
  tmp_metric->height = height;
  tmp_metric->heading_angle = heading;
  pilot_metric_.emplace_back(tmp_metric);
}

void MetricCollect::UpdateAvpMetricData(  // NOLINT
    const std::shared_ptr<LocalView>& local_view) {
  if (!local_view->HasFunctionManagerIn() ||
      local_view->GetFunctionManagerIn()->ta_pilot_mode() !=
          functionmanager::TaPilotMode::AVP) {
    // not in avp mode, clear data
    is_avp_mode_ = false;
    avp_metric_->ClearData();
    last_fsm_state_ = functionmanager::MachineStateType::INITIAL_TYPE;
    return;
  }
  if (!local_view->HasChassis()) {
    // lack of basic data, return directly.
    return;
  }
  const auto& fct_in = local_view->GetFunctionManagerIn()->fct_avp_in();
  const auto stage_type = local_view->HasADCTrajectory()
                              ? local_view->GetADCTrajectory()
                                    ->function_manager_out()
                                    .avp_fct_out()
                                    .stage_type()
                              : functionmanager::AvpFctOut::FsmStageType::
                                    AvpFctOut_FsmStageType_INIT;
  const auto fsm_state = local_view->HasFunctionManagerOut()
                             ? local_view->GetFunctionManagerOut()->fsm_state()
                             : functionmanager::MachineStateType::INITIAL_TYPE;
  if (fsm_state != last_fsm_state_) {
    last_fsm_state_ = fsm_state;
    is_avp_mode_ = false;
    avp_metric_->ClearData();
  }
  if (fct_in.sys_mode() == functionmanager::AvpFctIn_StateType_LAPA_MAPPING ||
      fct_in.sys_mode() == functionmanager::AvpFctIn::LOCALIZATION ||
      fct_in.sys_mode() == functionmanager::AvpFctIn::LOCALIZATION_BACKGROUND ||
      fct_in.sys_run_state() ==
          functionmanager::AvpFctIn_SysRunState_PARKSTART) {
    //  when mapping, localization : no metric collect;
    return;
  }
  if (fsm_state == functionmanager::MachineStateType::APA_TYPE ||
      fsm_state == functionmanager::MachineStateType::HISTORY_TRACE_TYPE ||
      fsm_state == functionmanager::MachineStateType::HDMAP_AVP_TYPE) {
    if (!is_avp_mode_) {
      avp_metric_->ClearData();
      is_avp_mode_ = true;
#ifndef ISORIN
      avp_metric_->start_time = Clock::NowInSeconds();
#else
      avp_metric_->start_time = Clock::NowInSecondsForBeiJing();
#endif
      avp_metric_->sys_mode = fct_in.sys_mode();
      AvpInit();  // init class member
    }
  }
  if (!is_avp_mode_) {
    return;
  }
  if (stage_type == functionmanager::AvpFctOut::FsmStageType::
                        AvpFctOut_FsmStageType_PARKING) {
    UpdateAvpParkingMetricData(local_view);  // update parking metric
  } else if (stage_type == functionmanager::AvpFctOut::FsmStageType::
                               AvpFctOut_FsmStageType_CRUISING) {
    UpdateAvpCrusingMetricData(local_view);  // update crusing metric
  }
  avp_metric_->avp_running_status.is_running =
      avp_metric_->avp_running_status.is_running ||
      std::abs(local_view->GetChassis()->speed_mps()) >
          avp_metric_conf_.ntp_metric_lat_lon().moving_spd() ||
      avp_metric_->parking_shift_count > 0;

  UpdateFinishStatusBaseFct(fct_in);
  if (is_ok_to_save_) {
    avp_metric_->parking_time =
        avp_metric_->parking_end_time - avp_metric_->parking_start_time;
    avp_metric_->cruise_mileage = std::abs(avp_metric_->crusing_end_distance -
                                           avp_metric_->crusing_start_distance);
    avp_metric_->cruise_time =
        avp_metric_->crusing_end_time - avp_metric_->crusing_start_time;
#ifndef ISORIN
    avp_metric_->end_time = Clock::NowInSeconds();
#else
    avp_metric_->end_time = Clock::NowInSecondsForBeiJing();
#endif
    avp_metric_->final_v = local_view->GetChassis()->speed_mps();
    if (avp_metric_->parking_type == 0 || avp_metric_->parking_type == 12) {
      uint32_t new_parking_type = GetAVPParkingType(
          *local_view->GetParkingLotOutArray(), fct_in.sys_command());
      if (new_parking_type != 0) {
        avp_metric_->parking_type = new_parking_type;
      }
    }
    if (avp_metric_->average_cpu_info.cnt > 0) {
      avp_metric_->average_cpu_info.planning_cpu /=
          avp_metric_->average_cpu_info.cnt;
      avp_metric_->average_cpu_info.planning_mem /=
          avp_metric_->average_cpu_info.cnt;
      avp_metric_->average_cpu_info.all_cpu_used /=
          avp_metric_->average_cpu_info.cnt;
      avp_metric_->average_cpu_info.mem_free /=
          avp_metric_->average_cpu_info.cnt;
      avp_metric_->average_cpu_info.total_mem /=
          avp_metric_->average_cpu_info.cnt;
    }
    return;
  }
  if (fct_in.sys_warning_info() ==
      functionmanager::AvpFctIn::WAIT_OBSTALE_0xA) {
    avp_metric_->avp_running_status.wait_obs = true;
  } else if (fct_in.sys_warning_info() == functionmanager::AvpFctIn::NO_ERROR) {
    avp_metric_->avp_running_status.wait_obs = false;
  }
  UpdateOpenSpaceDebugInfo(local_view);
}

void MetricCollect::AvpInit() {
  stop_id_ = -1;
  nudge_ids_ = {};
  is_ok_to_save_ = false;
  is_avp_parking_ = false;
  is_avp_crusing_ = false;
  last_trajacc_over_flag_ = false;
  start_id_ = 0;
  end_id_ = 0;
  parking_gear_ = soc::Chassis::GEAR_NEUTRAL;
  is_just_now_crusing_ = false;
  last_force_enable_dest_lat_constrian_flag_ = false;
}

void MetricCollect::UpdateAvpCrusingMetricData(
    const std::shared_ptr<LocalView>& local_view) {
  is_avp_parking_ = false;
  const auto fsm_state = local_view->HasFunctionManagerOut()
                             ? local_view->GetFunctionManagerOut()->fsm_state()
                             : functionmanager::MachineStateType::INITIAL_TYPE;
  double now_timestamp = 0.0;
#ifndef ISORIN
  now_timestamp = Clock::NowInSeconds();
#else
  now_timestamp = Clock::NowInSecondsForBeiJing();
#endif
  // crusing start
  if (!is_avp_crusing_ && local_view->HasADCTrajectory()) {
    is_avp_crusing_ = true;
    is_just_now_crusing_ = true;
    avp_metric_->crusing_start_time = now_timestamp;
    if (fsm_state == functionmanager::MachineStateType::HISTORY_TRACE_TYPE) {
      avp_metric_->crusing_start_distance =
          local_view->GetADCTrajectory()->avp_to_hmi().nns_distance();
    } else {
      avp_metric_->crusing_start_distance =
          local_view->GetADCTrajectory()->avp_to_hmi().hpa_distance();
    }
  }
  if (!is_avp_crusing_ || !local_view->HasADCTrajectory()) {
    return;
  }
  UpdateStopNudgeMetricData(local_view);
  // crusing is running
  if (local_view->GetADCTrajectory()->function_mode() == 10 &&
      !avp_metric_->avp_running_status.onceflag) {
    avp_metric_->emergency_brake_counter++;
    avp_metric_->avp_running_status.onceflag = true;
  }
  if (local_view->GetADCTrajectory()->function_mode() != 10) {
    avp_metric_->avp_running_status.onceflag = false;
  }
  double lat_jerk_value = 0.0;
  if (avp_real_jerk_->CalcNtpLatJerk(local_view, &lat_jerk_value)) {
    avp_metric_->cruise_abnormal_latjerk.push_back(lat_jerk_value);
  }
  double lon_jerk_value = 0.0;
  if (avp_real_jerk_->CalcNtpLonJerk(local_view, &lon_jerk_value)) {
    avp_metric_->cruise_abnormal_lonjerk.push_back(lon_jerk_value);
  }

  is_all_acc_over_threshd = true;
  for (const auto& avp_traj_point :
       local_view->GetADCTrajectory()->trajectory_point()) {
    if (avp_traj_point.a() <
        avp_metric_conf_.ntp_metric_lat_lon().traj_acc_threshold()) {
      is_all_acc_over_threshd = false;
      break;
    }
  }
  const auto traj_point_id = local_view->GetADCTrajectory()->header().seq();
  if (!is_all_acc_over_threshd && !last_trajacc_over_flag_) {
    last_trajacc_over_flag_ = true;
    start_id_ = traj_point_id;
    end_id_ = traj_point_id;
  }
  if (is_all_acc_over_threshd && last_trajacc_over_flag_) {
    end_id_ = traj_point_id;
    avp_metric_->cruise_planning_dec_over_threshold_frame_set.emplace_back(
        start_id_, end_id_);
    last_trajacc_over_flag_ = false;
  }
#ifdef ISMDC
  if (local_view->HasMbdDebugFromMCU() &&
      std::abs(local_view->GetChassis()->speed_mps()) >
          avp_metric_conf_.ntp_metric_lat_lon().crtl_moving_spd()) {
    double latctrl_pure_yawerr = local_view->GetMbdDebugFromMCU()
                                     ->lat_ctrl_debug()
                                     .latictrl_tors_pure_yawerr();
    double latctrl_sys_poserr = local_view->GetMbdDebugFromMCU()
                                    ->ctrl_dec_debug()
                                    .ctrldec_lat_sys_poserr();
    double lonctrl_sys_poserr = local_view->GetMbdDebugFromMCU()
                                    ->ctrl_dec_debug()
                                    .ctrldec_lon_sys_poserr();
    double lonctrl_vel_err = local_view->GetMbdDebugFromMCU()
                                 ->lon_ctrl_debug()
                                 .lonctrl_vel_vel_err();
    avp_metric_->crusing_max_latictrl_tors_pure_yawerr =
        (std::abs(avp_metric_->crusing_max_latictrl_tors_pure_yawerr) >
         std::abs(latctrl_pure_yawerr))
            ? (avp_metric_->crusing_max_latictrl_tors_pure_yawerr)
            : (latctrl_pure_yawerr);
    avp_metric_->crusing_max_ctrldec_lat_sys_poserr =
        (std::abs(avp_metric_->crusing_max_ctrldec_lat_sys_poserr) >
         std::abs(latctrl_sys_poserr))
            ? (avp_metric_->crusing_max_ctrldec_lat_sys_poserr)
            : (latctrl_sys_poserr);
    avp_metric_->crusing_max_ctrldec_lon_sys_poserr =
        (std::abs(avp_metric_->crusing_max_ctrldec_lon_sys_poserr) >
         std::abs(lonctrl_sys_poserr))
            ? (avp_metric_->crusing_max_ctrldec_lon_sys_poserr)
            : (lonctrl_sys_poserr);
    avp_metric_->crusing_max_lonctrl_vel_vel_err =
        (std::abs(avp_metric_->crusing_max_lonctrl_vel_vel_err) >
         std::abs(lonctrl_vel_err))
            ? (avp_metric_->crusing_max_lonctrl_vel_vel_err)
            : (lonctrl_vel_err);
  }
#endif

#ifdef ISORIN
  if (local_view->HasMcuToSocPnc() &&
      std::abs(local_view->GetChassis()->speed_mps()) >
          avp_metric_conf_.ntp_metric_lat_lon().crtl_moving_spd()) {
    double latctrl_pure_yawerr =
        local_view->GetMcuToSocPnc()->ctrllat().latictrl_tors_pure_yawerr_sg();
    double latctrl_sys_poserr =
        local_view->GetMcuToSocPnc()->ctrllat().latctrl_sys_poserr_sg();
    double lonctrl_sys_poserr =
        local_view->GetMcuToSocPnc()->ctrldec().ctrldec_lon_sys_poserr_sg();
    double lonctrl_vel_err =
        local_view->GetMcuToSocPnc()->ctrllon().lonctrl_vel_vel_err_sg();
    avp_metric_->crusing_max_latictrl_tors_pure_yawerr =
        (std::abs(avp_metric_->crusing_max_latictrl_tors_pure_yawerr) >
         std::abs(latctrl_pure_yawerr))
            ? (avp_metric_->crusing_max_latictrl_tors_pure_yawerr)
            : (latctrl_pure_yawerr);
    avp_metric_->crusing_max_ctrldec_lat_sys_poserr =
        (std::abs(avp_metric_->crusing_max_ctrldec_lat_sys_poserr) >
         std::abs(latctrl_sys_poserr))
            ? (avp_metric_->crusing_max_ctrldec_lat_sys_poserr)
            : (latctrl_sys_poserr);
    avp_metric_->crusing_max_ctrldec_lon_sys_poserr =
        (std::abs(avp_metric_->crusing_max_ctrldec_lon_sys_poserr) >
         std::abs(lonctrl_sys_poserr))
            ? (avp_metric_->crusing_max_ctrldec_lon_sys_poserr)
            : (lonctrl_sys_poserr);
    avp_metric_->crusing_max_lonctrl_vel_vel_err =
        (std::abs(avp_metric_->crusing_max_lonctrl_vel_vel_err) >
         std::abs(lonctrl_vel_err))
            ? (avp_metric_->crusing_max_lonctrl_vel_vel_err)
            : (lonctrl_vel_err);
  }
#endif

  avp_metric_->crusing_end_time = now_timestamp;
  if (fsm_state == functionmanager::MachineStateType::HISTORY_TRACE_TYPE) {
    avp_metric_->crusing_end_distance =
        local_view->GetADCTrajectory()->avp_to_hmi().nns_distance();
  } else {
    avp_metric_->crusing_end_distance =
        local_view->GetADCTrajectory()->avp_to_hmi().hpa_distance();
  }
}

void MetricCollect::UpdateAvpParkingMetricData(
    const std::shared_ptr<LocalView>& local_view) {
  const auto& fct_in = local_view->GetFunctionManagerIn()->fct_avp_in();
  const auto fsm_state = local_view->HasFunctionManagerOut()
                             ? local_view->GetFunctionManagerOut()->fsm_state()
                             : functionmanager::MachineStateType::INITIAL_TYPE;
  is_avp_crusing_ = false;
  double now_timestamp = 0.0;
#ifndef ISORIN
  now_timestamp = Clock::NowInSeconds();
#else
  now_timestamp = Clock::NowInSecondsForBeiJing();
#endif
  // parking start
  if (fct_in.sys_run_state() == functionmanager::AvpFctIn::PARKING &&
      !is_avp_parking_) {
    is_avp_parking_ = true;
    avp_metric_->parking_start_time = now_timestamp;
    parking_gear_ = soc::Chassis::GEAR_NEUTRAL;
    avp_metric_->parking_type = GetAVPParkingType(
        *local_view->GetParkingLotOutArray(), fct_in.sys_command());

    // update cruising to apa metric
    if (fsm_state == functionmanager::MachineStateType::HDMAP_AVP_TYPE &&
        is_just_now_crusing_) {
      is_just_now_crusing_ = false;
      if (std::abs(local_view->GetChassis()->speed_mps()) >=
              avp_metric_conf_.ntp_metric_lat_lon().modchg_moving_spd() ||
          local_view->GetADCTrajectory()->gear() != soc::Chassis::GEAR_DRIVE) {
        avp_metric_->cruise2parking_smooth_switch = true;
      }

      const auto& parking_lot_info_ = local_view->GetParkingLotOutArray();
      if (parking_lot_info_->has_opt_parking_seq()) {
        const auto& space_opt_id = parking_lot_info_->opt_parking_seq();
        for (int i = 0; i < parking_lot_info_->parking_lots_size(); i++) {
          if (parking_lot_info_->parking_lots().at(i).parking_seq() ==
              space_opt_id) {
            avp_metric_->cruise2parking_fixed_parklot =
                parking_lot_info_->parking_lots().at(i).is_private_ps();
            break;
          }
        }
      }
    }
  }

  if (!is_avp_parking_) {
    return;
  }

  //  parking is running shiftcount
  if (local_view->GetChassis()->gear_location() != parking_gear_ &&
      std::abs(local_view->GetChassis()->speed_mps()) >
          avp_metric_conf_.ntp_metric_lat_lon().moving_spd()) {
    parking_gear_ = local_view->GetChassis()->gear_location();
    avp_metric_->parking_shift_count++;
  }
#ifdef ISMDC
  if (local_view->HasMbdDebugFromMCU() &&
      std::abs(local_view->GetChassis()->speed_mps()) >
          avp_metric_conf_.ntp_metric_lat_lon().crtl_moving_spd()) {
    double latctrl_pure_yawerr = local_view->GetMbdDebugFromMCU()
                                     ->lat_ctrl_debug()
                                     .latictrl_tors_pure_yawerr();
    double latctrl_sys_poserr = local_view->GetMbdDebugFromMCU()
                                    ->ctrl_dec_debug()
                                    .ctrldec_lat_sys_poserr();
    double lonctrl_sys_poserr = local_view->GetMbdDebugFromMCU()
                                    ->ctrl_dec_debug()
                                    .ctrldec_lon_sys_poserr();
    double lonctrl_vel_err = local_view->GetMbdDebugFromMCU()
                                 ->lon_ctrl_debug()
                                 .lonctrl_vel_vel_err();
    avp_metric_->parking_max_latictrl_tors_pure_yawerr =
        (std::abs(avp_metric_->parking_max_latictrl_tors_pure_yawerr) >
         std::abs(latctrl_pure_yawerr))
            ? (avp_metric_->parking_max_latictrl_tors_pure_yawerr)
            : (latctrl_pure_yawerr);
    avp_metric_->parking_max_ctrldec_lat_sys_poserr =
        (std::abs(avp_metric_->parking_max_ctrldec_lat_sys_poserr) >
         std::abs(latctrl_sys_poserr))
            ? (avp_metric_->parking_max_ctrldec_lat_sys_poserr)
            : (latctrl_sys_poserr);
    avp_metric_->parking_max_ctrldec_lon_sys_poserr =
        (std::abs(avp_metric_->parking_max_ctrldec_lon_sys_poserr) >
         std::abs(lonctrl_sys_poserr))
            ? (avp_metric_->parking_max_ctrldec_lon_sys_poserr)
            : (lonctrl_sys_poserr);
    avp_metric_->parking_max_lonctrl_vel_vel_err =
        (std::abs(avp_metric_->parking_max_lonctrl_vel_vel_err) >
         std::abs(lonctrl_vel_err))
            ? (avp_metric_->parking_max_lonctrl_vel_vel_err)
            : (lonctrl_vel_err);
  }
#endif

#ifdef ISORIN
  if (local_view->HasMcuToSocPnc() &&
      std::abs(local_view->GetChassis()->speed_mps()) >
          avp_metric_conf_.ntp_metric_lat_lon().crtl_moving_spd()) {
    double latctrl_pure_yawerr =
        local_view->GetMcuToSocPnc()->ctrllat().latictrl_tors_pure_yawerr_sg();
    double latctrl_sys_poserr =
        local_view->GetMcuToSocPnc()->ctrllat().latctrl_sys_poserr_sg();
    double lonctrl_sys_poserr =
        local_view->GetMcuToSocPnc()->ctrldec().ctrldec_lon_sys_poserr_sg();
    double lonctrl_vel_err =
        local_view->GetMcuToSocPnc()->ctrllon().lonctrl_vel_vel_err_sg();
    avp_metric_->parking_max_latictrl_tors_pure_yawerr =
        (std::abs(avp_metric_->parking_max_latictrl_tors_pure_yawerr) >
         std::abs(latctrl_pure_yawerr))
            ? (avp_metric_->parking_max_latictrl_tors_pure_yawerr)
            : (latctrl_pure_yawerr);
    avp_metric_->parking_max_ctrldec_lat_sys_poserr =
        (std::abs(avp_metric_->parking_max_ctrldec_lat_sys_poserr) >
         std::abs(latctrl_sys_poserr))
            ? (avp_metric_->parking_max_ctrldec_lat_sys_poserr)
            : (latctrl_sys_poserr);
    avp_metric_->parking_max_ctrldec_lon_sys_poserr =
        (std::abs(avp_metric_->parking_max_ctrldec_lon_sys_poserr) >
         std::abs(lonctrl_sys_poserr))
            ? (avp_metric_->parking_max_ctrldec_lon_sys_poserr)
            : (lonctrl_sys_poserr);
    avp_metric_->parking_max_lonctrl_vel_vel_err =
        (std::abs(avp_metric_->parking_max_lonctrl_vel_vel_err) >
         std::abs(lonctrl_vel_err))
            ? (avp_metric_->parking_max_lonctrl_vel_vel_err)
            : (lonctrl_vel_err);
  }
#endif

  avp_metric_->parking_end_time = now_timestamp;
  // 强制使用终点区域约束次数
  if (!local_view->HasADCTrajectory() ||
      !local_view->GetADCTrajectory()->has_debug()) {
    return;
  }
  const auto& debug_openspace =
      local_view->GetADCTrajectory()->debug().planning_data().open_space();
  if (!last_force_enable_dest_lat_constrian_flag_ &&
      debug_openspace.smoother_force_enable_dest_lat_constrain()) {
    avp_metric_->force_enable_dest_lat_constrian_counter++;
  }
  last_force_enable_dest_lat_constrian_flag_ =
      debug_openspace.smoother_force_enable_dest_lat_constrain();
}

void MetricCollect::UpdateFinishStatusBaseFct(
    const functionmanager::AvpFctIn& fct_in) {
  if (fct_in.sys_run_state() == functionmanager::AvpFctIn::QUIT &&
      avp_metric_->finish_status == 0) {
    switch (fct_in.sys_warning_info()) {
      case functionmanager::AvpFctIn::NO_ERROR: {
        static constexpr u_int32_t kQuitByDriverBeforeRunning = 249;
        static constexpr u_int32_t kQuitByDriverInRunning = 248;
        if (!avp_metric_->avp_running_status.is_running) {
          avp_metric_->finish_status = kQuitByDriverBeforeRunning;
        } else if (avp_metric_->avp_running_status.wait_obs) {
          avp_metric_->finish_status =
              functionmanager::AvpFctIn::PAUSE_OVER_TIME_0xC;
        } else {
          avp_metric_->finish_status = kQuitByDriverInRunning;
        }
        break;
      }
      case functionmanager::AvpFctIn::DRIVER_INTERVENTION_0x12:
      case functionmanager::AvpFctIn::NTP_PARKING_LATOVERRIDE_0x49:
      case functionmanager::AvpFctIn::NTP_PARKING_GEARINTERVENE_0x4A:
      case functionmanager::AvpFctIn::NTP_PARKING_EPBINTERVENE_0x58: {
        static constexpr u_int32_t kDriverInterventionWithRisk = 100;
        if (avp_metric_->avp_running_status.wait_obs) {
          avp_metric_->finish_status =
              functionmanager::AvpFctIn::PAUSE_OVER_TIME_0xC;
        } else if (avp_metric_->avp_running_status.has_collision_risk) {
          avp_metric_->finish_status = kDriverInterventionWithRisk;
        } else {
          // DRIVER_INTERVENTION
          avp_metric_->finish_status = fct_in.sys_warning_info();
        }
        break;
      }
      default:
        avp_metric_->finish_status = fct_in.sys_warning_info();
    }
    is_ok_to_save_ = true;
  } else if ((fct_in.sys_run_state() == functionmanager::AvpFctIn::PARKFINISH ||
              fct_in.sys_run_state() ==
                  functionmanager::AvpFctIn::TBAFINISHED ||
              fct_in.sys_run_state() ==
                  functionmanager::AvpFctIn::NNSFINISHED ||
              fct_in.sys_run_state() ==
                  functionmanager::AvpFctIn::NTPFINISHED) &&
             avp_metric_->finish_status == 0) {
    static constexpr u_int32_t kFinish = 250;
    avp_metric_->finish_status = kFinish;
    is_ok_to_save_ = true;
  }
}

void MetricCollect::UpdateOpenSpaceDebugInfo(  // NOLINT
    const std::shared_ptr<LocalView>& local_view) {
  std::shared_ptr<const planning::ADCTrajectory> trajectory_ptr = nullptr;
  if (local_view->HasADCTrajectoryGuard()) {
    trajectory_ptr = local_view->GetADCTrajectoryGuard();
  } else if (local_view->HasADCTrajectory()) {
    trajectory_ptr = local_view->GetADCTrajectory();
  }
  if (nullptr == trajectory_ptr || !trajectory_ptr->has_debug()) {
    // has no debug info
    return;
  }
  const auto polygon2d =
      common::VehicleConfigHelper::GetPolygon2dWithBuffer(0, 0, 0);
  avp_metric_->min_collison_point_x = std::numeric_limits<double>::infinity();
  avp_metric_->min_collison_point_y = std::numeric_limits<double>::infinity();
  double closest_dis = std::numeric_limits<double>::infinity();
  avp_metric_->avp_running_status.has_collision_risk = false;
  const auto& open_space_debug =
      trajectory_ptr->debug().planning_data().open_space();
  if (open_space_debug.has_speed_plan_collision_info() &&
      open_space_debug.speed_plan_collision_info()
          .has_collision_fs_point_flu()) {
    auto collision_fs_point_flu =
        open_space_debug.speed_plan_collision_info().collision_fs_point_flu();
    double dis = polygon2d.DistanceTo(
        {collision_fs_point_flu.x(), collision_fs_point_flu.y()});
    if (dis < closest_dis) {
      avp_metric_->min_collison_point_x = collision_fs_point_flu.x();
      avp_metric_->min_collison_point_y = collision_fs_point_flu.y();
      closest_dis = dis;
    }
  }
  double min_closest_dis = closest_dis;
  if (trajectory_ptr->debug().has_safety_guard_info() &&
      trajectory_ptr->debug().safety_guard_info().has_collision_uss_id() &&
      local_view->HasPerceptionObstacles()) {
    const auto collision_uss_id =
        trajectory_ptr->debug().safety_guard_info().collision_uss_id();
    for (const auto& obs :
         local_view->GetPerceptionObstacles()->perception_obstacle()) {
      if (obs.id() == collision_uss_id) {
        if (!obs.has_position_flu()) {
          break;
        }
        double dis = polygon2d.DistanceTo(
            {obs.position_flu().x(), obs.position_flu().y()});
        if (dis < min_closest_dis) {
          avp_metric_->min_collison_point_x = obs.position_flu().x();
          avp_metric_->min_collison_point_y = obs.position_flu().y();
          min_closest_dis = dis;
        }
        break;
      }
    }
  }

  double collision_point_x = std::numeric_limits<double>::infinity();
  double collision_point_y = std::numeric_limits<double>::infinity();
  if (IsAdcHasCollisionRisk(local_view, &collision_point_x,
                            &collision_point_y)) {
    avp_metric_->avp_running_status.has_collision_risk = true;
  }
  const double dis = std::hypot(collision_point_x, collision_point_y);
  if (dis < min_closest_dis) {
    avp_metric_->min_collison_point_x = collision_point_x;
    avp_metric_->min_collison_point_y = collision_point_y;
  }

  if (open_space_debug.has_coarse_total_time() &&
      open_space_debug.has_smooth_total_time() &&
      TL::common::math::double_type::AlmostEqual(
          avp_metric_->init_coarse_path_time, 0.0) &&
      TL::common::math::double_type::AlmostEqual(
          avp_metric_->init_smooth_path_time, 0.0)) {
    avp_metric_->init_coarse_path_time = open_space_debug.coarse_total_time();
    avp_metric_->init_smooth_path_time = open_space_debug.smooth_total_time();
  }
  const auto planning_cpu = TL::planning::CpuRecorder::GetCpuUsageRatio();
  const auto planning_mem = TL::planning::CpuRecorder::GetMemoryUsage();
  const auto all_cpu = TL::planning::CpuRecorder::GetAllCpuData();
  const auto mem_free = TL::planning::CpuRecorder::GetSystemMemData().first;
  const auto mem_total =
      TL::planning::CpuRecorder::GetSystemMemData().second;
  if (planning_cpu > avp_metric_->top_cpu_info.planning_cpu) {
    avp_metric_->top_cpu_info.planning_cpu = planning_cpu;
    avp_metric_->top_cpu_info.planning_mem = planning_mem;
    avp_metric_->top_cpu_info.all_cpu_used = all_cpu;
    avp_metric_->top_cpu_info.mem_free = mem_free;
    avp_metric_->top_cpu_info.total_mem = mem_total;
  }
  avp_metric_->average_cpu_info.planning_cpu += planning_cpu;
  avp_metric_->average_cpu_info.planning_mem += planning_mem;
  avp_metric_->average_cpu_info.all_cpu_used += all_cpu;
  avp_metric_->average_cpu_info.mem_free += mem_free;
  avp_metric_->average_cpu_info.total_mem += mem_total;
  ++avp_metric_->average_cpu_info.cnt;
}

bool MetricCollect::IsAdcHasCollisionRisk(
    const std::shared_ptr<LocalView>& local_view,
    double* const collision_point_x, double* const collision_point_y) {
  // currently, only check uss info for collision
  bool res = false;
  if (collision_point_x == nullptr || collision_point_y == nullptr) {
    AWARN << "collision_point_x y is nullptr";
    return res;
  }
  if (!local_view->HasPerceptionObstacles()) {
    AWARN << "has no perception obstacle";
    return res;
  }
  if (!local_view->HasChassis()) {
    AWARN << "has no chassis";
    return res;
  }
  const auto gear_location = local_view->GetChassis()->gear_location();
  if (gear_location != soc::Chassis::GEAR_DRIVE &&
      gear_location != soc::Chassis::GEAR_REVERSE) {
    AWARN << "invalid gear location: " << gear_location;
    return res;
  }
  static constexpr double kLonBuffer = 0.5;
  static constexpr double kLatBuffer = 0.2;
  const auto adc_polygon2d_inflate =
      gear_location == soc::Chassis::GEAR_DRIVE
          ? common::VehicleConfigHelper::GetPolygon2dWithBuffer(
                0, 0, 0, kLonBuffer, 0, kLatBuffer, kLatBuffer)
          : common::VehicleConfigHelper::GetPolygon2dWithBuffer(
                0, 0, 0, 0, kLonBuffer, kLatBuffer, kLatBuffer);
  const auto& vehicle_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  const auto& perceptions_obs =
      local_view->GetPerceptionObstacles()->perception_obstacle();
  for (const auto& obs : perceptions_obs) {
    if (!obs.has_sub_type() ||
        obs.sub_type() != perception::PerceptionObstacle::ST_USS ||
        !obs.has_position_flu()) {
      continue;
    }
    if ((gear_location == soc::Chassis::GEAR_DRIVE &&
         obs.position_flu().x() < -1 * vehicle_param.back_edge_to_center()) ||
        (gear_location == soc::Chassis::GEAR_REVERSE &&
         obs.position_flu().x() > vehicle_param.front_edge_to_center())) {
      continue;
    }
    res = adc_polygon2d_inflate.IsPointIn(
        {obs.position_flu().x(), obs.position_flu().y()});
    if (res) {
      *collision_point_x = obs.position_flu().x();
      *collision_point_y = obs.position_flu().y();
      break;
    }
  }
  return res;
}

uint32_t MetricCollect::GetAVPParkingType(  // NOLINT
    const perception::ParkingLotOutArray& parking_lot_info,
    const functionmanager::AvpFctIn::SysCmdType& sys_command) const {
  perception::ParkingLotOut::ParkType parking_lot_type =
      perception::ParkingLotOut::NONE;
  if (parking_lot_info.has_opt_parking_seq()) {
    const auto& space_opt_id = parking_lot_info.opt_parking_seq();
    for (int i = 0; i < parking_lot_info.parking_lots_size(); i++) {
      if (parking_lot_info.parking_lots().at(i).parking_seq() == space_opt_id) {
        parking_lot_type = parking_lot_info.parking_lots().at(i).type();
        break;
      }
    }
  }
  uint32_t parking_type = 0;
  if (parking_lot_type == perception::ParkingLotOut::LATERAL) {
    switch (sys_command) {
      case functionmanager::AvpFctIn::PARKINCONTROL:
        parking_type = 1;
        break;
      case functionmanager::AvpFctIn::LEFTPARKOUTCONTROL:
        parking_type = 2;
        break;
      case functionmanager::AvpFctIn::RIGHTPARKOUTCONTROL:
        parking_type = 3;
        break;
      default:
        break;
    }
  }
  if (parking_lot_type == perception::ParkingLotOut::VERTICAL) {
    switch (sys_command) {
      case functionmanager::AvpFctIn::PARKINCONTROL:
        parking_type = 4;
        break;
      case functionmanager::AvpFctIn::LEFTPARKOUTCONTROL:
        parking_type = 5;
        break;
      case functionmanager::AvpFctIn::FRONTPARKOUTCONTROL:
        parking_type = 6;
        break;
      case functionmanager::AvpFctIn::RIGHTPARKOUTCONTROL:
        parking_type = 7;
        break;
      default:
        break;
    }
  }
  if (parking_lot_type == perception::ParkingLotOut::OBLIQUE) {
    switch (sys_command) {
      case functionmanager::AvpFctIn::PARKINCONTROL:
        parking_type = 8;
        break;
      case functionmanager::AvpFctIn::LEFTPARKOUTCONTROL:
        parking_type = 9;
        break;
      case functionmanager::AvpFctIn::FRONTPARKOUTCONTROL:
        parking_type = 10;
        break;
      case functionmanager::AvpFctIn::RIGHTPARKOUTCONTROL:
        parking_type = 11;
        break;
      default:
        break;
    }
  }
  if (sys_command == functionmanager::AvpFctIn::FORWARDCONTROL ||
      sys_command == functionmanager::AvpFctIn::BACKWARDCONTROL) {
    parking_type = 12;
  }
  return parking_type;
}

void MetricCollect::CheckSaveStatus() {
  static double last_timestamp = Clock::NowInSeconds();
  static constexpr double kMinTimeStamp = 60;  // s
  if (!is_avp_mode_ && Clock::NowInSeconds() - last_timestamp > kMinTimeStamp) {
    double now_timestamp = 0.0;
#ifndef ISORIN
    now_timestamp = Clock::NowInSeconds();
#else
    now_timestamp = Clock::NowInSecondsForBeiJing();
#endif
    const auto& nnp_metric = nnp_metric_.back();
    const auto& pilot_metric = pilot_metric_.back();
    if ((nnp_metric_.size() > 1 || nnp_metric->hdmap_mile > 0.001) &&
        nnp_metric->end_time < 1.0) {
      nnp_metric->end_time = now_timestamp;
    }
    if ((pilot_metric_.size() > 1 || pilot_metric->active_mile > 0.001) &&
        pilot_metric->end_time < 1.0) {
      pilot_metric->end_time = now_timestamp;
    }
    is_ok_to_save_ = true;
  }
  if (is_ok_to_save_) {
    last_timestamp = Clock::NowInSeconds();
  }
}

void MetricCollect::MetricDataToJson() {
  if (!is_ok_to_save_) {
    return;
  }
  bool has_data_to_save = false;
  is_ok_to_save_ = false;
  Json::array_t metric_array = {};
  // nnp
  for (const auto& nnp_metric : nnp_metric_) {
    if (nnp_metric->start_time < 1.0 || nnp_metric->end_time < 1.0) {
      continue;
    }
    has_data_to_save = true;
    Json::object_t obj_val = {};
    Json::object_t data_val = {};
    Json::object_t task_val = {};
    data_val["car_name"] = car_alias_;
    data_val["sw_version"] = sw_ver_;
    data_val["start_time"] = nnp_metric->start_time;
    data_val["heading_angle"] = nnp_metric->heading_angle;
    data_val["static_obs_avoid"] = nnp_metric->static_obs_avoid;
    data_val["dynamic_obs_avoid"] = nnp_metric->dynamic_obs_avoid;
    data_val["height"] = nnp_metric->height;
    data_val["end_time"] = nnp_metric->end_time < 0.0
                               ? Clock::NowInSecondsForBeiJing()
                               : nnp_metric->end_time;
    data_val["active_milleage"] = nnp_metric->active_mile;
    data_val["hdmap_mileage"] = nnp_metric->hdmap_mile;
    Json::array_t lat_val = {};
    lat_val.clear();
    for (const auto& lat : nnp_metric->lat_jerk) {
      lat_val.emplace_back(lat);
    }
    data_val["number_of_latjerk_exception"] = lat_val;
    Json::array_t lon_val = {};
    lon_val.clear();
    for (const auto& lon : nnp_metric->lon_jerk) {
      lon_val.emplace_back(lon);
    }
    data_val["number_of_longthjerk_exception"] = lon_val;
    data_val["number_of_downgrading_lcc"] = nnp_metric->lcc_downgrade;
    data_val["number_of_downgrading_acc"] = nnp_metric->acc_downgrade;
    data_val["number_of_downgrading_location"] = nnp_metric->location_downgrade;
    data_val["number_of_downgrading_internal"] = nnp_metric->internal_downgrade;
    data_val["number_of_takeover_level1"] = nnp_metric->lat_override;
    data_val["number_of_takeover_level2"] = nnp_metric->lon_override;
    data_val["number_of_takeover_level3"] = nnp_metric->key_take_over;
    data_val["number_of_takeover_level4"] = nnp_metric->take_over;
    data_val["number_of_takeover_level5"] = nnp_metric->fault_take_over;
    task_val["prediction"] = nnp_metric->prediction_timeout;
    task_val["total"] = nnp_metric->total_timeout;
    task_val["total_regular_planning_time"] = nnp_metric->total_regular_time;
    task_val["run_once_time"] = nnp_metric->run_once_time;
    task_val["planning_cpu_err"] = nnp_metric->planning_cpu_err;
    task_val["all_cpu_err"] = nnp_metric->all_cpu_err;
    data_val["task_timeout"] = task_val;
    data_val["number_of_tomainroad_success"] = nnp_metric->in_mainroad_success;
    data_val["number_of_tomainroad_fail"] = nnp_metric->in_mainroad_fail;
    data_val["number_of_toramp_success"] = nnp_metric->in_ramp_success;
    data_val["number_of_toramp_fail"] = nnp_metric->in_ramp_fail;
    data_val["number_of_navigation_lane_change_success"] =
        nnp_metric->navi_lane_change_success;
    data_val["number_of_navigation_lane_change_fail"] =
        nnp_metric->navi_lane_change_fail;
    data_val["number_of_efficiency_lane_change_success"] =
        nnp_metric->effi_lane_change_success;
    data_val["number_of_efficiency_lane_change_fail"] =
        nnp_metric->effi_lane_change_fail;
    data_val["number_of_swtich_lane_change_success"] =
        nnp_metric->switch_lane_change_success;
    data_val["number_of_swtich_lane_change_fail"] =
        nnp_metric->switch_lane_change_fail;
    data_val["number_of_audio_lane_change_success"] =
        nnp_metric->audio_lane_change_success;
    data_val["number_of_audio_lane_change_fail"] =
        nnp_metric->audio_lane_change_fail;
    data_val["emerger_braking"] = nnp_metric->emerger_braking;
    Json::object_t fault_obj = {};
    fault_obj["fault_enum"] = nnp_metric->fault_obj.first;
    fault_obj["timestamp"] = nnp_metric->fault_obj.second;
    data_val["fault_obj"] = fault_obj;
    Json::object_t coordinate = {};
    coordinate["lat"] = nnp_metric->coordinate.first;
    coordinate["lon"] = nnp_metric->coordinate.second;
    data_val["coordinate"] = coordinate;

    Json::object_t scenario_obj = {};
    Json::array_t scenario_num = {};
    for (const auto& num : nnp_metric->scenario_obj.first) {
      scenario_num.emplace_back(num);
    }
    scenario_obj["scenario_enum"] = scenario_num;
    scenario_obj["timestamp"] = nnp_metric->scenario_obj.second;
    data_val["scenario_obj"] = scenario_obj;
    nnp_metric->scenario_obj.first.clear();
    data_val["reserve"] = "";

    obj_val["driving_mode"] = nnp_metric->driver_mode;
    obj_val["data"] = data_val;
    metric_array.emplace_back(obj_val);
  }
  // pilot
  for (const auto& pilot_metric : pilot_metric_) {
    if (pilot_metric->start_time < 1.0 || pilot_metric->end_time < 1.0) {
      continue;
    }
    has_data_to_save = true;
    Json::object_t obj_val;
    Json::object_t data_val;
    data_val["car_name"] = car_alias_;
    data_val["sw_version"] = sw_ver_;
    data_val["start_time"] = pilot_metric->start_time;
    data_val["heading_angle"] = pilot_metric->heading_angle;
    data_val["static_obs_avoid"] = pilot_metric->static_obs_avoid;
    data_val["dynamic_obs_avoid"] = pilot_metric->dynamic_obs_avoid;
    data_val["height"] = pilot_metric->height;
    data_val["end_time"] = pilot_metric->end_time < 0.0
                               ? Clock::NowInSecondsForBeiJing()
                               : pilot_metric->end_time;
    data_val["active_milleage"] = pilot_metric->active_mile;
    data_val["total_mileage"] = pilot_metric->total_mile;
    Json::array_t lat_val;
    lat_val.clear();
    for (const auto& lat : pilot_metric->lat_jerk) {
      lat_val.emplace_back(lat);
    }
    data_val["number_of_latjerk_exception"] = lat_val;
    Json::array_t lon_val;
    lon_val.clear();
    for (const auto& lon : pilot_metric->lon_jerk) {
      lon_val.emplace_back(lon);
    }
    data_val["number_of_longthjerk_exception"] = lon_val;
    data_val["number_of_downgrading"] = pilot_metric->downgrade;
    data_val["number_of_takeover_level1"] = pilot_metric->lat_override;
    data_val["number_of_takeover_level2"] = pilot_metric->lon_override;
    data_val["number_of_takeover_level3"] = pilot_metric->take_over;
    Json::object_t task_val = {};
    task_val["prediction"] = pilot_metric->prediction_timeout;
    task_val["total"] = pilot_metric->total_timeout;
    task_val["total_regular_planning_time"] = pilot_metric->total_regular_time;
    task_val["run_once_time"] = pilot_metric->run_once_time;
    task_val["planning_cpu_err"] = pilot_metric->planning_cpu_err;
    task_val["all_cpu_err"] = pilot_metric->all_cpu_err;
    data_val["task_timeout"] = task_val;
    data_val["number_of_swtich_lane_change_success"] =
        pilot_metric->switch_lane_change_success;
    data_val["number_of_swtich_lane_change_fail"] =
        pilot_metric->switch_lane_change_fail;
    data_val["number_of_audio_lane_change_success"] =
        pilot_metric->audio_lane_change_success;
    data_val["number_of_audio_lane_change_fail"] =
        pilot_metric->audio_lane_change_fail;
    data_val["emerger_braking"] = pilot_metric->emerger_braking;
    Json::object_t coordinate = {};
    coordinate["lat"] = pilot_metric->coordinate.first;
    coordinate["lon"] = pilot_metric->coordinate.second;
    data_val["coordinate"] = coordinate;
    data_val["reserve"] = "";

    obj_val["driving_mode"] = 1;
    obj_val["data"] = data_val;
    metric_array.emplace_back(obj_val);
  }
  // avp
  if (is_avp_mode_) {
    has_data_to_save = true;
    Json::object_t obj_val;
    Json::object_t data_val;
    data_val["sw_version"] = sw_ver_;
    data_val["start_time"] = avp_metric_->start_time;
    data_val["end_time"] = avp_metric_->end_time < 0.0
                               ? Clock::NowInSecondsForBeiJing()
                               : avp_metric_->end_time;
    data_val["avp_sys_mode"] = avp_metric_->sys_mode;
    data_val["avp_parking_type"] = avp_metric_->parking_type;
    data_val["avp_finish_status"] = avp_metric_->finish_status;
    data_val["avp_parking_time"] = avp_metric_->parking_time;
    data_val["avp_parking_shift_counter"] = avp_metric_->parking_shift_count;
    data_val["min_collision_point_x"] = avp_metric_->min_collison_point_x;
    data_val["min_collision_point_y"] = avp_metric_->min_collison_point_y;
    data_val["final_v"] = avp_metric_->final_v;
    data_val["init_coarse_path_time"] = avp_metric_->init_coarse_path_time;
    data_val["init_smooth_path_time"] = avp_metric_->init_smooth_path_time;
    data_val["average_planning_cpu"] =
        avp_metric_->average_cpu_info.planning_cpu;
    data_val["average_planning_mem"] =
        avp_metric_->average_cpu_info.planning_mem;
    data_val["average_all_cpu_used"] =
        avp_metric_->average_cpu_info.all_cpu_used;
    data_val["average_mem_free"] = avp_metric_->average_cpu_info.mem_free;
    data_val["average_total_mem"] = avp_metric_->average_cpu_info.total_mem;
    data_val["top_planning_cpu"] = avp_metric_->top_cpu_info.planning_cpu;
    data_val["top_planning_mem"] = avp_metric_->top_cpu_info.planning_mem;
    data_val["top_all_cpu_used"] = avp_metric_->top_cpu_info.all_cpu_used;
    data_val["top_mem_free"] = avp_metric_->top_cpu_info.mem_free;
    data_val["top_total_mem"] = avp_metric_->top_cpu_info.total_mem;
    data_val["reserve"] = "";
    data_val["emergency_brake_counter"] = avp_metric_->emergency_brake_counter;
    data_val["cruise_mileage"] = avp_metric_->cruise_mileage;
    data_val["cruise_time"] = avp_metric_->cruise_time;
    data_val["cruise_abnormal_latjerk"] = avp_metric_->cruise_abnormal_latjerk;
    data_val["cruise_abnormal_lonjerk"] = avp_metric_->cruise_abnormal_lonjerk;
    data_val["force_enable_dest_lat_constrian_counter"] =
        avp_metric_->force_enable_dest_lat_constrian_counter;
    data_val["cruise_nudge_pedestrian_counter"] =
        avp_metric_->cruise_nudge_pedestrian_counter;
    data_val["cruise_avoidance_pedestrian_counter"] =
        avp_metric_->cruise_avoidance_pedestrian_counter;
    data_val["cruise_nudge_vehicle_counter"] =
        avp_metric_->cruise_nudge_vehicle_counter;
    data_val["cruise_avoidance_vehicle_counter"] =
        avp_metric_->cruise_avoidance_vehicle_counter;
    data_val["cruise_nudge_cone_or_barricade_counter"] =
        avp_metric_->cruise_nudge_cone_or_barricade_counter;
    data_val["cruise_pass_speed_bump_counter"] =
        avp_metric_->cruise_pass_speed_bump_counter;
    data_val["cruise2parking_smooth_switch"] =
        avp_metric_->cruise2parking_smooth_switch;
    data_val["cruise2parking_fixed_parklot"] =
        avp_metric_->cruise2parking_fixed_parklot;
    data_val["cruise_planning_dec_over_threshold_frame_set"] =
        avp_metric_->cruise_planning_dec_over_threshold_frame_set;
    data_val["crusing_max(latictrl_tors_pure_yawerr)"] =
        avp_metric_->crusing_max_latictrl_tors_pure_yawerr;
    data_val["crusing_max(ctrldec_lat_sys_poserr)"] =
        avp_metric_->crusing_max_ctrldec_lat_sys_poserr;
    data_val["crusing_max(ctrldec_lon_sys_poserr)"] =
        avp_metric_->crusing_max_ctrldec_lon_sys_poserr;
    data_val["crusing_max(lonctrl_vel_vel_err)"] =
        avp_metric_->crusing_max_lonctrl_vel_vel_err;
    data_val["parking_max(latictrl_tors_pure_yawerr)"] =
        avp_metric_->parking_max_latictrl_tors_pure_yawerr;
    data_val["parking_max(ctrldec_lat_sys_poserr)"] =
        avp_metric_->parking_max_ctrldec_lat_sys_poserr;
    data_val["parking_max(ctrldec_lon_sys_poserr)"] =
        avp_metric_->parking_max_ctrldec_lon_sys_poserr;
    data_val["parking_max(lonctrl_vel_vel_err)"] =
        avp_metric_->parking_max_lonctrl_vel_vel_err;
    obj_val["driving_mode"] = 3;
    obj_val["data"] = data_val;
    metric_array.emplace_back(obj_val);
  }

  if (has_data_to_save) {
    SaveJsonToFile(metric_array);
  }
  ClearMetric();
}

void MetricCollect::SaveJsonToFile(  // NOLINT
    const Json::array_t& metric_json) {
  std::ofstream os;
  std::stringstream ss;
  ss << std::fixed << std::setprecision(2) << Clock::NowInSeconds();
  std::string root_dir = "/opt/usr/col/planning/";
#ifdef ISORIN
  root_dir = "/opt/usr/col/planning/stat/";
#endif
#ifdef ISX86
  auto* var = std::getenv("CYBER_PATH");  // NOLINT
  std::string work_dir(var);
  root_dir = work_dir + "/data/metric/";
#endif
  std::string tmp_json_name = root_dir + ss.str() + ".json.active";
  std::string json_name = root_dir + ss.str() + ".json";
  os.open(tmp_json_name, std::ios::out);
  if (os.is_open()) {
    os << metric_json;
  }
  os.flush();
  os.close();
  std::rename(tmp_json_name.data(), json_name.data());  // NOLINT
}

}  // namespace planning
}  // namespace TL
