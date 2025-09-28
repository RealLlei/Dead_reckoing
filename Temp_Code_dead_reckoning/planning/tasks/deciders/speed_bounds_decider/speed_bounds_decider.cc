/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "planning/tasks/deciders/speed_bounds_decider/speed_bounds_decider.h"
#include <math.h>  // NOLINT

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "common/configs/config_gflags.h"
#include "common/file/file.h"
#include "common/math/double_type.h"
#include "common/time/clock.h"
#include "planning/common/path/path_data.h"
#include "planning/common/st_graph_data.h"
#include "planning/tasks/deciders/speed_bounds_decider/speed_limit_decider.h"
#include "planning/tasks/deciders/speed_bounds_decider/st_boundary_mapper.h"

#include "planning/hmi/lon_hmi/speed_convertor/speed_convertor.h"
#include "planning/proto/hmi_config.pb.h"
#include "proto/fsm/function_manager.pb.h"
#include "proto/map/map_road.pb.h"
#include "proto/perception/perception_obstacle.pb.h"
#include "proto/soc/chassis.pb.h"

namespace TL {
namespace planning {

using TL::common::ErrorCode;
using TL::common::Status;
using TL::common::TrajectoryPoint;
using TL::planning_internal::StGraphBoundaryDebug;
using TL::planning_internal::STGraphDebug;

SpeedBoundsDecider::SpeedBoundsDecider(
    const TaskConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : Decider(config, injector) {
  if (config.has_speed_bounds_decider_config()) {
    speed_bounds_config_.CopyFrom(config.speed_bounds_decider_config());
  }

  if (!LoadCentricAccelCalibrationTable()) {
    hdmap_centric_accel_calibration_table_.first = {
        0.0, std::numeric_limits<double>::max()};
    hdmap_centric_accel_calibration_table_.second = {
        speed_bounds_config_.max_centric_acceleration_limit(),
        speed_bounds_config_.max_centric_acceleration_limit()};
    perception_centric_accel_calibration_table_.first = {
        0.0, std::numeric_limits<double>::max()};
    perception_centric_accel_calibration_table_.second = {
        speed_bounds_config_.max_centric_acceleration_limit(),
        speed_bounds_config_.max_centric_acceleration_limit()};
  }
}

Status SpeedBoundsDecider::Process(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  // retrieve data from frame and reference_line_info
  const PathData& path_data = reference_line_info->path_data();
  TrajectoryPoint init_point = frame->PlanningStartPoint();
  if (!reference_line_info->path_data().frenet_frame_path().is_forward_path()) {
    init_point.set_v(std::fabs(init_point.v()));
    init_point.set_a(-init_point.a());
  }
  const ReferenceLine& reference_line = reference_line_info->reference_line();
  PathDecision* const path_decision = reference_line_info->path_decision();

  // 1. Map obstacles into st graph
  auto time1 = common::Clock::NowInSeconds();
  STBoundaryMapper boundary_mapper(speed_bounds_config_, reference_line,
                                   path_data,
                                   path_data.discretized_path().Length(),
                                   FLAGS_trajectory_time_length, injector_);

  if (!FLAGS_use_st_drivable_boundary) {
    path_decision->EraseStBoundaries();
  }

  if (boundary_mapper.ComputeSTBoundary(path_decision).code() ==
      ErrorCode::PLANNER_CRUISING_STBOUNDS_ERROR) {
    const std::string msg = "Mapping obstacle failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNER_CRUISING_SPEEDBOUNDSPRIORI_ERROR, msg);
  }
  auto time2 = common::Clock::NowInSeconds();
  const double diff = time2 - time1;
  ADEBUG << "Time for ST Boundary Mapping = " << diff * 1000 << " msec.";

  std::vector<const STBoundary*> boundaries;
  for (const auto* obstacle : path_decision->obstacles().Items()) {
    const auto& id = obstacle->Id();
    const auto& st_boundary = obstacle->path_st_boundary();
    if (!st_boundary.IsEmpty()) {
      if (st_boundary.boundary_type() == STBoundary::BoundaryType::KEEP_CLEAR) {
        path_decision->Find(id)->SetBlockingObstacle(false);
      } else {
        path_decision->Find(id)->SetBlockingObstacle(true);
      }
      boundaries.push_back(&st_boundary);
    }
  }

  const double min_s_on_st_boundaries = SetSpeedFallbackDistance(path_decision);
  SpeedLimitByCrossObstacle(frame, reference_line_info);
  // 2. Create speed limit along path
  speed_limit_decider_.Init(speed_bounds_config_, reference_line, path_data);

  const auto* centric_accel_calibration_table_ =
      &hdmap_centric_accel_calibration_table_;
  if (frame_->local_view().HasFunctionManagerOut()) {
    const auto& fsm_out = frame_->local_view().GetFunctionManagerOut();
    if (fsm_out != nullptr &&
        ((fsm_out->fsm_state() ==
              functionmanager::MachineStateType::PERCEPTION_TYPE &&
          fsm_out->perception_sub_state() ==
              functionmanager::PerceptionSubState::LANELINE_TYPE) ||
         (fsm_out->fsm_state() ==
              functionmanager::MachineStateType::HDMAP_TYPE &&
          fsm_out->hdmap_sub_state() ==
              functionmanager::HdmapSubState::MAP_FUSION_TYPE &&
          fsm_out->localization_maptype() ==
              navigation_hdmap::MapMsg::PERCEP_MAP))) {
      centric_accel_calibration_table_ =
          &perception_centric_accel_calibration_table_;
    }
  }

  SpeedLimit speed_limit;
  if (!speed_limit_decider_
           .GetSpeedLimits(path_decision->obstacles(), *reference_line_info_,
                           init_point, frame_->local_view(),
                           *centric_accel_calibration_table_, frame,
                           &speed_limit)
           .ok()) {
    const std::string msg = "Getting speed limits failed!";
    AERROR << msg;
    return Status(ErrorCode::PLANNER_CRUISING_SPEEDBOUNDSPRIORI_ERROR, msg);
  }

  // 3. Get path_length as s axis search bound in st graph
  const double path_data_length = path_data.discretized_path().Length();

  // 4. Get time duration as t axis search bound in st graph
  const double total_time_by_conf =
      FLAGS_trajectory_time_length;  // speed_bounds_config_.total_time();

  // Load generated st graph data back to frame
  StGraphData* st_graph_data = reference_line_info_->mutable_st_graph_data();

  // Add a st_graph debug info and save the pointer to st_graph_data for
  // optimizer logging
  auto* debug = reference_line_info_->mutable_debug();
  STGraphDebug* st_graph_debug = debug->mutable_planning_data()->add_st_graph();

  st_graph_data->LoadData(boundaries, min_s_on_st_boundaries, init_point,
                          speed_limit, reference_line_info->GetCruiseSpeed(),
                          path_data_length, total_time_by_conf, st_graph_debug);

  // Create and record st_graph debug info
  RecordSTGraphDebug(*st_graph_data, st_graph_debug);
  CacheFrontLaneInfo(frame, reference_line_info);
  return Status::OK();
}

double SpeedBoundsDecider::SetSpeedFallbackDistance(
    PathDecision* const path_decision) {
  // Set min_s_on_st_boundaries to guide speed fallback.
  static constexpr double kEpsilon = 1.0e-6;
  double min_s_non_reverse = std::numeric_limits<double>::infinity();
  double min_s_reverse = std::numeric_limits<double>::infinity();

  for (const auto* obstacle : path_decision->obstacles().Items()) {
    const auto& st_boundary = obstacle->path_st_boundary();

    if (st_boundary.IsEmpty()) {
      continue;
    }

    const auto left_bottom_point_s = st_boundary.bottom_left_point().s();
    const auto right_bottom_point_s = st_boundary.bottom_right_point().s();
    const auto lowest_s = std::min(left_bottom_point_s, right_bottom_point_s);

    if (left_bottom_point_s - right_bottom_point_s > kEpsilon) {
      if (min_s_reverse > lowest_s) {
        min_s_reverse = lowest_s;
      }
    } else if (min_s_non_reverse > lowest_s) {
      min_s_non_reverse = lowest_s;
    }
  }

  min_s_reverse = std::max(min_s_reverse, 0.0);
  min_s_non_reverse = std::max(min_s_non_reverse, 0.0);

  return min_s_non_reverse > min_s_reverse ? 0.0 : min_s_non_reverse;
}

void SpeedBoundsDecider::RecordSTGraphDebug(const StGraphData& st_graph_data,
                                            STGraphDebug* st_graph_debug) {
  if (!FLAGS_enable_record_debug || (st_graph_debug == nullptr)) {
    ADEBUG << "Skip record debug info";
    return;
  }

  st_graph_debug->set_name("Speed Bounds Boundary");

  for (const auto& boundary : st_graph_data.st_boundaries()) {
    auto* boundary_debug = st_graph_debug->add_boundary();
    boundary_debug->set_name(boundary->id());
    switch (boundary->boundary_type()) {
      case STBoundary::BoundaryType::FOLLOW:
        boundary_debug->set_type(StGraphBoundaryDebug::ST_BOUNDARY_TYPE_FOLLOW);
        break;
      case STBoundary::BoundaryType::OVERTAKE:
        boundary_debug->set_type(
            StGraphBoundaryDebug::ST_BOUNDARY_TYPE_OVERTAKE);
        break;
      case STBoundary::BoundaryType::STOP:
        boundary_debug->set_type(StGraphBoundaryDebug::ST_BOUNDARY_TYPE_STOP);
        break;
      case STBoundary::BoundaryType::UNKNOWN:
        boundary_debug->set_type(
            StGraphBoundaryDebug::ST_BOUNDARY_TYPE_UNKNOWN);
        break;
      case STBoundary::BoundaryType::YIELD:
        boundary_debug->set_type(StGraphBoundaryDebug::ST_BOUNDARY_TYPE_YIELD);
        break;
      case STBoundary::BoundaryType::KEEP_CLEAR:
        boundary_debug->set_type(
            StGraphBoundaryDebug::ST_BOUNDARY_TYPE_KEEP_CLEAR);
        break;
      default:
        break;
    }

    for (const auto& point : boundary->points()) {
      auto* point_debug = boundary_debug->add_point();
      point_debug->set_t(point.x());
      point_debug->set_s(point.y());
    }
  }

  for (const auto& point : st_graph_data.speed_limit().speed_limit_points()) {
    common::SpeedPoint* speed_point = st_graph_debug->add_speed_limit();
    speed_point->set_s(point.first);
    speed_point->set_v(point.second);
  }
}

bool SpeedBoundsDecider::LoadCentricAccelCalibrationTable() {
#ifdef FOR_BAIDU_SIMULATION
  return false;
#endif
  TL::functionmanager::HmiConfig hmi_config;
  if (!TL::common::GetProtoFromFile(FLAGS_hmi_config_file, &hmi_config)) {
    AERROR << "Failed to load obs follow time config file "
           << FLAGS_hmi_config_file;
    return false;
  }
  const auto& hdmap_curvatures =
      hmi_config.curvature_velocity_conf().curvature_r();
  const auto& hdmap_centric_accels =
      hmi_config.curvature_velocity_conf().centripetal_acceleration();
  const auto hdmap_element_size =
      std::min(hdmap_curvatures.size(), hdmap_centric_accels.size());
  if (hdmap_element_size < 2) {
    return false;
  }

  hdmap_centric_accel_calibration_table_.first.reserve(hdmap_element_size);
  hdmap_centric_accel_calibration_table_.second.reserve(hdmap_element_size);

  for (int i = hdmap_element_size - 1; i >= 0; --i) {
    if (common::math::double_type::SeemsEqual(hdmap_curvatures.at(i), 0.0)) {
      hdmap_centric_accel_calibration_table_.first.emplace_back(
          std::numeric_limits<double>::max());
    } else {
      hdmap_centric_accel_calibration_table_.first.emplace_back(
          1.0 / hdmap_curvatures.at(i));
    }
    hdmap_centric_accel_calibration_table_.second.emplace_back(
        hdmap_centric_accels.at(i));
  }

  const auto& perception_curvatures =
      hmi_config.perception_curvature_velocity_conf().curvature_r();
  const auto& perception_centric_accels =
      hmi_config.perception_curvature_velocity_conf()
          .centripetal_acceleration();
  const auto perception_element_size =
      std::min(perception_curvatures.size(), perception_centric_accels.size());
  if (perception_element_size < 2) {
    return false;
  }

  perception_centric_accel_calibration_table_.first.reserve(
      perception_element_size);
  perception_centric_accel_calibration_table_.second.reserve(
      perception_element_size);

  for (int i = perception_element_size - 1; i >= 0; --i) {
    if (common::math::double_type::SeemsEqual(perception_curvatures.at(i),
                                              0.0)) {
      perception_centric_accel_calibration_table_.first.emplace_back(
          std::numeric_limits<double>::max());
    } else {
      perception_centric_accel_calibration_table_.first.emplace_back(
          1.0 / perception_curvatures.at(i));
    }
    perception_centric_accel_calibration_table_.second.emplace_back(
        perception_centric_accels.at(i));
  }
  return true;
}

void SpeedBoundsDecider::SpeedLimitByCrossObstacle(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
#ifdef FOR_BAIDU_SIMULATION
  return;
#endif
  if (frame == nullptr || reference_line_info == nullptr) {
    return;
  }
  const auto& local_view = frame->local_view();
  if (!local_view.HasFunctionManagerOut() ||
      local_view.GetFunctionManagerOut()->hdmap_sub_state() !=
          functionmanager::LOCAL_HDMAP_TYPE) {
    return;
  }
  const auto& junction_overlaps =
      reference_line_info->reference_line().map_path().junction_overlaps();
  const auto adc_start_s = reference_line_info->AdcSlBoundary().start_s();
  const auto adc_end_s = reference_line_info->AdcSlBoundary().end_s();
  bool adc_in_junction = false;
  for (const auto& junction : junction_overlaps) {
    if (adc_start_s <= junction.end_s && adc_end_s >= junction.start_s) {
      adc_in_junction = true;
      break;
    }
  }
  if (!adc_in_junction) {
    return;
  }
  auto collision_time = std::numeric_limits<double>::infinity();
  const auto& obstacles =
      reference_line_info->path_decision()->obstacles().Items();
  for (const auto* obstacle : obstacles) {
    if (obstacle == nullptr || obstacle->path_st_boundary().IsEmpty() ||
        !obstacle->IsBicycle() || obstacle->IsStatic() ||
        obstacle->path_st_boundary().IsEmpty()) {
      continue;
    }
    // 小于80度认为是同向的
    const auto front_same_direction =
        (obstacle->Perception().has_theta_flu() &&
         std::fabs(obstacle->Perception().theta_flu()) < 1.4);
    // 车头前方，同向的障碍物，正常处理
    if (obstacle->PerceptionSLBoundary().start_s() > adc_start_s &&
        front_same_direction) {
      continue;
    }
    auto* mutable_obstacle =
        reference_line_info->path_decision()->Find(obstacle->Id());
    if (mutable_obstacle != nullptr) {
      mutable_obstacle->SetIsBelievable(false);
    }
    auto nearest_point = obstacle->path_st_boundary().bottom_left_point();
    if (nearest_point.s() >
        std::fmax(reference_line_info->vehicle_state().linear_velocity() *
                      nearest_point.t(),
                  15)) {
      continue;
    }
    collision_time = std::fmin(collision_time,
                               fmax(0.0, obstacle->path_st_boundary().min_t()));
  }
  if (std::isinf(collision_time)) {
    return;
  }
  if (collision_time > FLAGS_trajectory_time_length) {
    return;
  }
  const auto collision_percent =
      std::floor(collision_time) / FLAGS_trajectory_time_length;
  auto target_cruise_speed =
      reference_line_info->GetCruiseSpeed() * collision_percent;
  reference_line_info->SetCruiseSpeed(target_cruise_speed);
}

void SpeedBoundsDecider::CacheFrontLaneInfo(
    const Frame* frame, ReferenceLineInfo* const reference_line_info) {
  static constexpr double kMaxSearchDistance = 800.0;
  static constexpr int kMaxSpeedKm = 130;
  if (frame == nullptr || reference_line_info == nullptr) {
    return;
  }
  const auto* reference_line_provider = frame->GetReferenceLineProvider();
  if (reference_line_provider == nullptr) {
    return;
  }

  const auto& pnc_map = reference_line_provider->GetPncMap();
  if (pnc_map == nullptr) {
    return;
  }
  // 车速偏移设置项，要求控车和偏移一致
  uint32_t offset_mode = 0;
  auto offset_val = 0;
  auto tgtspd_valid = false;
  if (frame != nullptr && frame->local_view().HasMcuToSocPnc() &&
      frame->local_view().GetMcuToSocPnc() != nullptr &&
      reference_line_info != nullptr) {
    const auto& control_data = frame->local_view().GetMcuToSocPnc();
    offset_mode = control_data->can_input_from_mcu().cdcs11_tgtspdctgset();
    offset_val =
        control_data->can_input_from_mcu().has_cdcs11_tgtspddrftset()
            ? static_cast<int>(
                  control_data->can_input_from_mcu().cdcs11_tgtspddrftset() *
                      5 -
                  30)
            : 0;
    tgtspd_valid = offset_val >= -10 && offset_val <= 10;
  }
  auto ramp_info = pnc_map->GetFrontLaneRangeInfo(kMaxSearchDistance,
                                                  TL::hdmap::LaneType::RAMP);
  if (ramp_info.start_lane != nullptr) {
    auto ramp_speed_km =
        static_cast<int>(std::round(fmin(ramp_info.speed_limit, 100) * 3.6));
    const auto spd_offset =
        tgtspd_valid ? (offset_mode == 0)
                           ? offset_val
                           : static_cast<int>(
                                 std::ceil(ramp_speed_km * offset_val * 0.01))
                     : 0;
    ramp_speed_km = std::min(ramp_speed_km + spd_offset, kMaxSpeedKm);
    ramp_speed_km = ramp_info.start_lane->IsRampRoad()
                        ? ramp_speed_km
                        : std::min(ramp_speed_km, 80);
    ramp_info.speed_limit =
        SpeedConventor::ConvertDisplaySpdToReal(ramp_speed_km);
  }
  auto tunnel_info = pnc_map->GetFrontLaneRangeInfo(
      kMaxSearchDistance, TL::hdmap::LaneType::TUNNEL_LANE);
  if (tunnel_info.start_lane != nullptr) {
    auto tunnel_speed_km = static_cast<int>(std::round(
        fmin(tunnel_info.start_lane->GetSectionMaxSpeed(), 100) * 3.6));
    const auto spd_offset =
        tgtspd_valid ? (offset_mode == 0)
                           ? offset_val
                           : static_cast<int>(
                                 std::ceil(tunnel_speed_km * offset_val * 0.01))
                     : 0;
    tunnel_speed_km = std::min(tunnel_speed_km + spd_offset, kMaxSpeedKm);
    tunnel_info.speed_limit =
        SpeedConventor::ConvertDisplaySpdToReal(tunnel_speed_km);
  }
  reference_line_info->SetFrontRamp(ramp_info);
  reference_line_info->SetFrontTunnel(tunnel_info);
}
}  // namespace planning
}  // namespace TL
