/*
 * Copyright (c) TL auto Co., Ltd. 2021-2022. All rights reserved.
 */
#include "planning/localview/lane_line_builder/adaptive_cruise_lane_line/adaptive_cruise_lane_line.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <ostream>
#include <string>
#include <utility>

#include "absl/strings/match.h"
#include "absl/strings/numbers.h"
#include "absl/strings/strip.h"
#include "common/configs/config_gflags.h"
#include "common/math/math_utils.h"
#include "common/math/vec2d.h"
#include "common/util/perf_util.h"
#include "common/util/util.h"
#include "map/hdmap/path.h"
#include "proto/common/pnc_point.pb.h"
#include "proto/common/vehicle_state.pb.h"
#include "proto/control/adas_someip.pb.h"
#include "proto/fsm/function_manager.pb.h"
#include "proto/prediction/vector_net.pb.h"

namespace TL {
namespace planning {
using TL::common::math::Vec2d;
using TL::common::util::operator+;  // NOLINT
using TL::common::Status;
using TL::common::math::Clamp;
using std::tuple;

Status AdaptiveCruise::Init() {
  return Status::OK();
}

Status AdaptiveCruise::Init(
    const planning::PerceptionMapConfig& config,
    const std::shared_ptr<LocalViewData>& local_view_data) {
  navi_hdmap_config_ = config;
  vehicle_param_ = common::VehicleConfigHelper::GetConfig().vehicle_param();
  std::vector<double> den(3, 0.0);
  std::vector<double> num(3, 0.0);
  common::LpFirstOrderCoefficients(0.01, 0.1, 0.02, &den, &num);
  steer_angle_filter_.set_coefficients(den, num);
  kappa_by_yaw_rate_filter_.SetCoefficientAndFlag(0.12, false);    // 0.085
  kappa_by_steer_angle_filter_.SetCoefficientAndFlag(0.5, false);  // 0.0476
  radius_after_fusion_filter_.SetCoefficientAndFlag(0.5, false);
  cruise_target_select_.Init();
  local_view_data_ = local_view_data;
  return Status::OK();
}

Status AdaptiveCruise::Start() {
  return Status::OK();
}

void AdaptiveCruise::Stop() {}

void AdaptiveCruise::KappaPreprocessing(
    const std::shared_ptr<LocalView>& local_view) {
  // 首先根据方向盘转角计算前轮转角和车辆行驶半径
  double original_steering_percentage =
      local_view->GetChassis()->has_steering_percentage()
          ? local_view->GetChassis()->steering_percentage()
          : 0.0;
  original_steering_percentage =
      acc_function_active_ && !acc_function_active_last_
          ? original_steering_percentage
          : steer_angle_filter_.Filter(original_steering_percentage);
  auto vehicle_state = local_view->GetVehicleState();
  wheel_angle_ = CalculateWheelAngle(original_steering_percentage,
                                     vehicle_state->linear_velocity());
  KappaByYawrate(YawrateFilter(local_view->GetChassis()->yaw_rate()),
                 vehicle_state->linear_velocity());
  const double lf = 1.465;
  const double lr = vehicle_param_.wheel_base() - lf;
  const double Cf = 104000.0;
  const double Cr = 160000.0;
  const double mass = 1980.0;
  // ADEBUG << "wheel_angle " << wheel_angle;
  KappaBySteerAngle(wheel_angle_, vehicle_state->linear_velocity(), lf, lr, Cf,
                    Cr, mass);

  std::pair<double, double> stability_factor =
      StabilityFactor(wheel_angle_, lf, lr, -Cf, -Cr, mass);

  double omega = YawRateByWheelAngle(stability_factor.first, wheel_angle_,
                                     vehicle_state->linear_velocity(), lf, lr);
  UNUSED(omega);
  std::pair<double, double> kappa_radius_by_steer_stability_factor =
      KappaAndRadiusBySteerStabilityFactor(stability_factor.first, lf, lr, -Cf,
                                           -Cr, mass,
                                           vehicle_state->linear_velocity());
  UNUSED(kappa_radius_by_steer_stability_factor);
  SubjTrajKappaFusion(kappa_by_yaw_rate_, kappa_by_steer_angle_,
                      vehicle_state->linear_velocity());
  ADEBUG << "kappa_fusion_ " << kappa_fusion_ << " wheel_angle "
         << wheel_angle_;
  ADEBUG << "original_steering_percentage: " << original_steering_percentage;
  local_view->SetSubjectKappa(kappa_fusion_);
}

bool AdaptiveCruise::Process(
    const std::shared_ptr<LocalView>& local_view,
    functionmanager::FunctionManagerOut* const to_fct) {
  ADEBUG << "Adaptive Cruise!";
  // if (current_map_msg_.use_count() != 1) {
  //   current_map_msg_ = std::make_shared<navigation_hdmap::MapMsg>();
  // } else {
  //   current_map_msg_->Clear();
  // }

  if (local_view->HasArena()) {
    current_map_msg_ =
        common::memory::ArenaAdapter::CreateMessage<navigation_hdmap::MapMsg>(
            local_view->GetArena());
  } else {
    current_map_msg_ = std::make_shared<navigation_hdmap::MapMsg>();
  }

  acc_function_active_ =
      local_view->HasFunctionManagerIn() &&
      local_view->GetFunctionManagerIn() != nullptr &&
      local_view->GetFunctionManagerIn()->fct_nnp_in().has_acc_state() &&
      (local_view->GetFunctionManagerIn()->fct_nnp_in().acc_state() ==
           functionmanager::FctToNnpInput::ACC_ACTIVE ||
       local_view->GetFunctionManagerIn()->fct_nnp_in().acc_state() ==
           functionmanager::FctToNnpInput::ACC_OVERRIDE ||
       local_view->GetFunctionManagerIn()->fct_nnp_in().acc_state() ==
           functionmanager::FctToNnpInput::ACC_STANDSTILL_ACTIVE ||
       local_view->GetFunctionManagerIn()->fct_nnp_in().acc_state() ==
           functionmanager::FctToNnpInput::ACC_STANDSTILL_WAIT);
  common::util::FillHeader("adaptive_cruise_hdmap", current_map_msg_.get());
  auto* hd_map = current_map_msg_->mutable_hdmap();
  common::util::FillHeader("from_adaptive_cruise_hdmap",
                           hd_map->mutable_header());
  auto* lane = hd_map->add_lane();
  auto vehicle_state = local_view->GetVehicleState();
  // ADEBUG << "radius_Fusion_ " << radius_Fusion_;
  const double turn_radius = radius_Fusion_;
  ADEBUG << "kappa_fusion_ " << kappa_fusion_ << " turn_radius " << turn_radius
         << " wheel_angle " << wheel_angle_;
  ResetCruiseTargetId(&cruise_target_id_);
  if (!cruise_target_select_.Process(local_view, kappa_fusion_, wheel_angle_,
                                     &cruise_target_id_)) {
    ResetCruiseTargetId(&cruise_target_id_);
  }
  for (size_t i = 0; i < cruise_target_id_.size(); i++) {
    ADEBUG << "cruise_id " << i << " = " << cruise_target_id_[i];
  }

  if (!cruise_target_id_.empty()) {
    std::vector<int32_t> cruise_target_id;
    cruise_target_id.resize(2);
    cruise_target_id.at(0) = cruise_target_id_.at(0);
    cruise_target_id.at(1) = cruise_target_id_.at(1);
    local_view->SetCruiseTargetId(cruise_target_id);
  }
  local_view_data_->set_cruise_target_ids(cruise_target_id_);
  // CalculateRadius(wheel_angle * FLAGS_steering_wheel_reduction_factor);
  ADEBUG << ", wheel_angle: " << wheel_angle_
         << ", turn_radius: " << turn_radius;
  std::tuple<bool, double, bool, double> adjacent_lane_estension_tuple =
      AdjacentLaneExtensionSign(
          local_view->GetLaneMarkers()->front_left_road_edge().c0_position(),
          local_view->GetLaneMarkers()->front_left_road_edge().quality(),
          local_view->GetLaneMarkers()->front_right_road_edge().c0_position(),
          local_view->GetLaneMarkers()->front_right_road_edge().quality());
  std::tuple<double, double, double, double, double> extension_lane_width =
      ExtensionLaneWidth(adjacent_lane_estension_tuple);
  // 根据行驶半径生成离散点path
  std::tuple<common::Path, common::Path, common::Path> center_path;
  GeneratePathPoints(turn_radius, wheel_angle_, *vehicle_state,
                     extension_lane_width, &center_path);
  // 根据path生成lane
  GenerateOneLane(std::get<0>(center_path), lane,
                  std::get<0>(extension_lane_width), 0, 5);
  if (std::get<0>(adjacent_lane_estension_tuple)) {
    auto* left_lane = hd_map->add_lane();
    GenerateOneLane(std::get<1>(center_path), left_lane,
                    std::get<1>(extension_lane_width), 1, 1);
    left_lane->add_right_neighbor_forward_lane_id()->set_id(lane->id().id());
    lane->add_left_neighbor_forward_lane_id()->set_id(left_lane->id().id());
  }
  if (std::get<2>(adjacent_lane_estension_tuple)) {
    auto* right_lane = hd_map->add_lane();
    GenerateOneLane(std::get<2>(center_path), right_lane,
                    std::get<2>(extension_lane_width), 2, 1);
    right_lane->add_left_neighbor_forward_lane_id()->set_id(lane->id().id());
    lane->add_right_neighbor_forward_lane_id()->set_id(right_lane->id().id());
  }
  auto* routing = current_map_msg_->mutable_routing();
  // 生成routing
  SetRouting(routing, hd_map);

  if (current_map_msg_->hdmap().lane_size() > 0) {
    ADEBUG << "success!!!"
           << ", lane size: " << current_map_msg_->hdmap().lane_size();
    current_routing_response_ =
        std::make_shared<routing::RoutingResponse>(current_map_msg_->routing());
    to_fct->set_cruise_status(FLAGS_enable_cruise_mode);
    auto soc_04_val = to_fct->soc_2_fct_tbd_u32_04();
    uint32_t cruise_status = FLAGS_enable_cruise_mode ? 0x2 : 0;
    to_fct->set_soc_2_fct_tbd_u32_04(soc_04_val | cruise_status);
    return FLAGS_enable_cruise_mode;
  }
  to_fct->set_cruise_status(false);
  AERROR << "adaptive cruise mode filed!!!";
  acc_function_active_last_ = acc_function_active_;
  return false;
}

void AdaptiveCruise::ResetCruiseTargetId(
    std::vector<int32_t>* cruise_target_id) {
  cruise_target_id->clear();
  *cruise_target_id = {-1, -1, -1, -1, -1, -1, -1, -1,
                       -1, -1, -1, -1, -1, -1, -1, -1};
}

double AdaptiveCruise::CalculateWheelAngle(double steer_angle, double speed) {
  const double steering_angle_rad = SteerPct2Wheel(steer_angle);
  return copysign(std::min(fabs(steering_angle_rad), WheelAngleLimit(speed)),
                  steering_angle_rad);
}

double AdaptiveCruise::SteerPct2Wheel(double steer_percentage) {
  auto steer = common::math::Clamp(steer_percentage, -100.0, 100.0);
  return steer / 100 * vehicle_param_.max_steer_angle() /
         vehicle_param_.steer_ratio();
}

double AdaptiveCruise::WheelAngleLimit(double speed) {
  const double max_lat_acc = 5.0;
  return std::atan(max_lat_acc * vehicle_param_.wheel_base() /
                   std::pow(std::max(0.1, speed), 2));
}

double AdaptiveCruise::CalculateRadius(double wheel_angle) {
  const double min_wheel_angle = 0.0003;
  return std::fabs(wheel_angle) < min_wheel_angle
             ? 10001.0
             : vehicle_param_.wheel_base() / std::tan(std::fabs(wheel_angle));
}

// 参考左右测路沿信息,计算巡航模式下可扩展的车道宽度
std::tuple<double, double, double, double, double>
AdaptiveCruise::ExtensionLaneWidth(
    std::tuple<bool, double, bool, double> adjacent_lane_estension_tuple) {
  double extenstion_left_lane_width = 0.0;
  double extenstion_right_lane_width = 0.0;
  double extenstion_current_lane_width = FLAGS_adaptive_cruise_lane_width;
  // 左侧道路中心线到自车道中心线距离
  double extenstion_left_center_to_current_center = 0.0;
  // 右侧道路中心线到自车道中心线距离
  double extenstion_right_center_to_current_center = 0.0;
  if (std::get<0>(adjacent_lane_estension_tuple)) {
    extenstion_left_lane_width =
        std::min(std::get<1>(adjacent_lane_estension_tuple),
                 FLAGS_adaptive_cruise_lane_width);
    extenstion_left_center_to_current_center =
        0.5 * (std::min(std::get<1>(adjacent_lane_estension_tuple),
                        FLAGS_adaptive_cruise_lane_width) +
               FLAGS_adaptive_cruise_lane_width);
  } else {
    extenstion_left_lane_width = 0.0;
    extenstion_left_center_to_current_center = 0.0;
  }

  if (std::get<2>(adjacent_lane_estension_tuple)) {
    extenstion_right_lane_width =
        std::min(std::get<3>(adjacent_lane_estension_tuple),
                 FLAGS_adaptive_cruise_lane_width);
    extenstion_right_center_to_current_center =
        0.5 * (std::min(std::get<3>(adjacent_lane_estension_tuple),
                        FLAGS_adaptive_cruise_lane_width) +
               FLAGS_adaptive_cruise_lane_width);
  } else {
    extenstion_right_lane_width = 0.0;
    extenstion_right_center_to_current_center = 0.0;
  }
  return std::make_tuple(
      extenstion_current_lane_width, extenstion_left_lane_width,
      extenstion_right_lane_width, extenstion_left_center_to_current_center,
      extenstion_right_center_to_current_center);
}

void AdaptiveCruise::GenerateCurrentLaneTrajectoryPoints(
    double radius, double angle, common::Path* const init_points,
    const double max_arc_length, const double extension_lane_width) {
  UNUSED(extension_lane_width);
  const double central_length =
      FLAGS_buffer_gainst_lookford_distance +
      hdmap::PncMap::LookForwardDistance(
          functionmanager::MachineStateType::PERCEPTION_TYPE,
          navi_hdmap_config_.default_speed_limit(),
          navi_hdmap_config_.default_max_cruise_speed());
  const double back_length =
      std::max(navi_hdmap_config_.lanemarker_back_length(), 5.0);
  const double delta_distance = 1.0;
  const double delta_angle = delta_distance / radius;
  double s = -1.0;
  // 自车轨迹曲率较大,半径大于 3000 ;认为是直行
  if (radius > 3000.0) {
    int max_index = floor((back_length + central_length) / delta_distance);
    for (int i = 0; i < max_index; i++) {
      auto* point = init_points->add_path_point();
      auto x = i * delta_distance - back_length;
      s += delta_distance;
      point->set_x(x);
      point->set_y(0.0);
      point->set_theta(0.0);
      point->set_s(s);
      point->set_kappa(0);
    }
    return;
  }
  // 自车后方 back_length 曲率设置为 0
  int back_max_index = floor(back_length / delta_distance);
  for (int i = 0; i < back_max_index; i++) {
    auto* point = init_points->add_path_point();
    s += delta_distance;
    auto x = i * delta_distance - back_length;
    point->set_x(x);
    point->set_y(0.0);
    point->set_theta(0.0);
    point->set_s(s);
    point->set_kappa(1 / radius);
  }
  // 自车前方轨迹
  double x = 0.0;
  double y = 0.0;
  double beta = 0.0;
  int phi_num = M_PI_2 / delta_angle;
  for (int i = 0; i < phi_num && s <= max_arc_length; i++) {
    double phi = i * delta_angle;
    x = radius * std::sin(phi);
    y = copysign(radius * (1 - std::cos(phi)), angle);
    beta = copysign(phi, angle);
    // cneter lane point
    auto pre_point =
        init_points->path_point().at(init_points->path_point_size() - 1);
    auto* point = init_points->add_path_point();
    // cneter lane point
    point->set_x(x);
    point->set_y(y);
    point->set_theta(beta);
    point->set_kappa(1 / radius);
    s += std::hypot(x - pre_point.x(), y - pre_point.y());
    point->set_s(s);
  }
  // 裁剪
  if (s < central_length) {
    // cneter lane point
    auto pre_point =
        init_points->path_point().at(init_points->path_point_size() - 2);
    auto end_point =
        init_points->path_point().at(init_points->path_point_size() - 1);
    double dx = end_point.x() - pre_point.x();
    double dy = end_point.y() - pre_point.y();
    double end_phi = end_point.theta();
    while (s < central_length) {
      // cneter lane point
      auto last_point =
          init_points->path_point().at(init_points->path_point_size() - 1);
      auto* point = init_points->add_path_point();
      // cneter lane point
      point->set_x(last_point.x() + dx);
      point->set_y(last_point.y() + dy);
      point->set_theta(end_phi);
      s += std::hypot(dx, dy);
      point->set_s(s);
      point->set_kappa(0);
    }
  }
}

void AdaptiveCruise::GenerateAdjacentLaneTrajectoryPoints(
    const common::Path& init_points, common::Path* const adjacent_init_points,
    std::tuple<double, double, double, double, double> extension_lane_width,
    const int flag, const double radius, const double angle,
    const int discrete_step) {
  UNUSED(angle);
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  double s = 0.0;
  double kappa = 0.0;
  int init_points_size = init_points.path_point_size();
  if (init_points_size < 1) {
    return;
  }
  adjacent_init_points->mutable_path_point()->Reserve(init_points_size);
  if (radius > 3000.0) {
    for (int i = 0; i < init_points_size;) {
      auto* point = adjacent_init_points->add_path_point();
      double center_x = init_points.path_point(i).x();
      double center_y = init_points.path_point(i).y();
      double center_theta = init_points.path_point(i).theta();
      double center_s = init_points.path_point(i).s();
      double center_kappa = init_points.path_point(i).kappa();
      // flag == 1 生成左道中心线 flag = 2 生成右道中心线
      point->set_x(center_x);
      point->set_y(center_y + std::get<3>(extension_lane_width));
      point->set_theta(center_theta);
      point->set_s(center_s);
      point->set_kappa(center_kappa);
      if (flag == 2) {
        point->set_x(center_x);
        point->set_y(center_y - std::get<4>(extension_lane_width));
        point->set_theta(center_theta);
        point->set_s(center_s);
        point->set_kappa(center_kappa);
      }
      i += discrete_step;
    }
    return;
  }
  //
  for (int i = 0; i < init_points.path_point_size();) {
    auto* point = adjacent_init_points->add_path_point();
    double center_x = init_points.path_point(i).x();
    double center_y = init_points.path_point(i).y();
    double center_theta = init_points.path_point(i).theta();
    double center_s = init_points.path_point(i).s();
    double center_kappa = init_points.path_point(i).kappa();
    // 后方
    if (center_x <= 0.0) {
      // flag == 1 生成左道中心线 flag = 2 生成右道中心线
      point->set_x(center_x);
      point->set_y(center_y + std::get<3>(extension_lane_width));
      point->set_theta(center_theta);
      point->set_s(center_s);
      point->set_kappa(center_kappa);
      if (flag == 2) {
        point->set_x(center_x);
        point->set_y(center_y - std::get<4>(extension_lane_width));
        point->set_theta(center_theta);
        point->set_s(center_s);
        point->set_kappa(center_kappa);
      }
    } else {
      // flag == 1 生成左道中心线 flag = 2 生成右道中心线
      x = center_x +
          std::get<3>(extension_lane_width) * cos(center_theta + M_PI_2);
      y = center_y +
          std::get<3>(extension_lane_width) * sin(center_theta + M_PI_2);
      if (flag == 2) {
        x = center_x +
            std::get<4>(extension_lane_width) * cos(center_theta - M_PI_2);
        y = center_y +
            std::get<4>(extension_lane_width) * sin(center_theta - M_PI_2);
      }
      s = center_s;
      theta = center_theta;
      kappa = center_kappa;
      point->set_x(x);
      point->set_y(y);
      point->set_theta(theta);
      point->set_s(s);
      point->set_kappa(kappa);
    }
    i += discrete_step;
  }
}

void AdaptiveCruise::GenerateLeftLaneTrajectoryPoints(
    double radius, double angle, common::Path* const init_points,
    const double max_arc_length, const double extension_lane_width) {
  const double central_length =
      FLAGS_buffer_gainst_lookford_distance +
      hdmap::PncMap::LookForwardDistance(
          functionmanager::MachineStateType::PERCEPTION_TYPE,
          navi_hdmap_config_.default_speed_limit(),
          navi_hdmap_config_.default_max_cruise_speed());
  const double back_length =
      std::max(navi_hdmap_config_.lanemarker_back_length(), 5.0);
  const double delta_distance = 1.0;
  const double delta_angle = delta_distance / radius;
  double s = -1.0;
  // 自车轨迹曲率较大,半径大于 3000 ;认为是直行
  if (radius > 3000.0) {
    int max_index = floor((back_length + central_length) / delta_distance);
    for (int i = 0; i < max_index; i++) {
      auto* point = init_points->add_path_point();
      auto x = i * delta_distance - back_length;
      s += delta_distance;

      point->set_x(x);
      point->set_y(extension_lane_width);
      point->set_theta(0.0);
      point->set_s(s);
      point->set_kappa(0);
    }
    return;
  }
  // 自车后方 back_length 曲率设置为 0
  int back_max_index = floor(back_length / delta_distance);
  for (int i = 0; i < back_max_index; i++) {
    auto* point = init_points->add_path_point();
    s += delta_distance;
    auto x = i * delta_distance - back_length;
    point->set_x(x);
    point->set_y(extension_lane_width);
    point->set_theta(0.0);
    point->set_s(s);
    point->set_kappa(1 / radius);
  }
  // 自车前方轨迹
  // double center_x = 0.0;
  // double center_y = 0.0;
  double x = 0.0;
  double y = 0.0;
  double beta = 0.0;
  int phi_num = M_PI_2 / delta_angle;
  for (int i = 0; i < phi_num && s <= max_arc_length; i++) {
    double phi = i * delta_angle;
    // center_x = radius * std::sin(phi);
    // center_y = copysign(radius * (1 - std::cos(phi)), angle);
    //
    x = radius * std::sin(phi) -
        copysign(extension_lane_width * std::sin(phi), angle);
    if (copysign(1.0, angle) >= 0.0) {
      y = copysign(radius * (1 - std::cos(phi)), angle) +
          copysign(extension_lane_width * std::cos(phi), angle);

    } else {
      y = copysign(radius * (1 - std::cos(phi)), angle) -
          copysign(extension_lane_width * std::cos(phi), angle);
    }
    beta = copysign(phi, angle);
    // cneter lane point
    auto pre_point =
        init_points->path_point().at(init_points->path_point_size() - 1);
    auto* point = init_points->add_path_point();
    // cneter lane point
    point->set_x(x);
    point->set_y(y);
    point->set_theta(beta);
    point->set_kappa(1 / radius);
    s += std::hypot(x - pre_point.x(), y - pre_point.y());
    point->set_s(s);
  }
  // 裁剪
  if (s < central_length) {
    // cneter lane point
    auto pre_point =
        init_points->path_point().at(init_points->path_point_size() - 2);
    auto end_point =
        init_points->path_point().at(init_points->path_point_size() - 1);
    double dx = end_point.x() - pre_point.x();
    double dy = end_point.y() - pre_point.y();
    double end_phi = end_point.theta();
    while (s < central_length) {
      // cneter lane point
      auto last_point =
          init_points->path_point().at(init_points->path_point_size() - 1);
      auto* point = init_points->add_path_point();
      // cneter lane point
      point->set_x(last_point.x() + dx);
      point->set_y(last_point.y() + dy);
      point->set_theta(end_phi);
      s += std::hypot(dx, dy);
      point->set_s(s);
      point->set_kappa(0);
    }
  }
}

void AdaptiveCruise::GenerateRightLaneTrajectoryPoints(
    double radius, double angle, common::Path* const init_points,
    const double max_arc_length, const double extension_lane_width) {
  const double central_length =
      FLAGS_buffer_gainst_lookford_distance +
      hdmap::PncMap::LookForwardDistance(
          functionmanager::MachineStateType::PERCEPTION_TYPE,
          navi_hdmap_config_.default_speed_limit(),
          navi_hdmap_config_.default_max_cruise_speed());
  const double back_length =
      std::max(navi_hdmap_config_.lanemarker_back_length(), 5.0);
  const double delta_distance = 1.0;
  const double delta_angle = delta_distance / radius;
  double s = -1.0;
  // 自车轨迹曲率较大,半径大于 3000 ;认为是直行
  if (radius > 3000.0) {
    int max_index = floor((back_length + central_length) / delta_distance);
    for (int i = 0; i < max_index; i++) {
      auto* point = init_points->add_path_point();
      auto x = i * delta_distance - back_length;
      s += delta_distance;
      point->set_x(x);
      point->set_y(-extension_lane_width);
      point->set_theta(0.0);
      point->set_s(s);
      point->set_kappa(0);
    }
    return;
  }
  // 自车后方 back_length 曲率设置为 0
  int back_max_index = floor(back_length / delta_distance);
  for (int i = 0; i < back_max_index; i++) {
    auto* point = init_points->add_path_point();
    s += delta_distance;
    auto x = i * delta_distance - back_length;
    point->set_x(x);
    point->set_y(-extension_lane_width);
    point->set_theta(0.0);
    point->set_s(s);
    point->set_kappa(1 / radius);
  }
  // 自车前方轨迹
  double x = 0.0;
  double y = 0.0;
  double beta = 0.0;
  int phi_num = M_PI_2 / delta_angle;
  for (int i = 0; i < phi_num && s <= max_arc_length; i++) {
    double phi = i * delta_angle;

    if (copysign(1.0, angle) >= 0.0) {
      y = copysign(radius * (1 - std::cos(phi)), angle) -
          copysign(extension_lane_width * std::cos(phi), angle);
    } else {
      y = copysign(radius * (1 - std::cos(phi)), angle) +
          copysign(extension_lane_width * std::cos(phi), angle);
    }
    x = radius * std::sin(phi) +
        copysign(extension_lane_width * std::sin(phi), angle);
    beta = copysign(phi, angle);
    // cneter lane point
    auto pre_point =
        init_points->path_point().at(init_points->path_point_size() - 1);
    auto* point = init_points->add_path_point();
    // cneter lane point
    point->set_x(x);
    point->set_y(y);
    point->set_theta(beta);
    point->set_kappa(1 / radius);
    s += std::hypot(x - pre_point.x(), y - pre_point.y());
    point->set_s(s);
  }
  // 裁剪
  if (s < central_length) {
    // cneter lane point
    auto pre_point =
        init_points->path_point().at(init_points->path_point_size() - 2);
    auto end_point =
        init_points->path_point().at(init_points->path_point_size() - 1);
    double dx = end_point.x() - pre_point.x();
    double dy = end_point.y() - pre_point.y();
    double end_phi = end_point.theta();
    while (s < central_length) {
      // cneter lane point
      auto last_point =
          init_points->path_point().at(init_points->path_point_size() - 1);
      auto* point = init_points->add_path_point();
      // cneter lane point
      point->set_x(last_point.x() + dx);
      point->set_y(last_point.y() + dy);
      point->set_theta(end_phi);
      s += std::hypot(dx, dy);
      point->set_s(s);
      point->set_kappa(0);
    }
  }
}

void AdaptiveCruise::GenerateInitPoints(
    double radius, double angle, common::Path* const init_points,
    common::Path* const init_left_points, common::Path* const init_right_points,
    const double max_arc_length,
    std::tuple<double, double, double, double, double> extension_lane_width) {

  const double central_length =
      FLAGS_buffer_gainst_lookford_distance +
      hdmap::PncMap::LookForwardDistance(
          functionmanager::MachineStateType::PERCEPTION_TYPE,
          navi_hdmap_config_.default_speed_limit(),
          navi_hdmap_config_.default_max_cruise_speed());
  const double back_length =
      std::max(navi_hdmap_config_.lanemarker_back_length(), 5.0);
  const double delta_distance = 1.0;
  const double delta_angle = delta_distance / radius;
  double s = -1.0;
  double left_s = -1.0;
  double right_s = -1.0;
  ADEBUG << "central_length: " << central_length;
  std::vector<Vec2d> points;
  if (radius > 3000) {
    int max_index = floor((back_length + central_length) / delta_distance);
    for (int i = 0; i < max_index; i++) {
      auto* point = init_points->add_path_point();
      auto* left_point = init_left_points->add_path_point();
      auto* right_point = init_right_points->add_path_point();
      auto x = i * delta_distance - back_length;
      s += delta_distance;
      // center lane point
      point->set_x(x);
      point->set_y(0.0);
      point->set_theta(0.0);
      point->set_s(s);
      point->set_kappa(0);
      // left lane point
      left_point->set_x(x);
      left_point->set_y(std::get<3>(extension_lane_width));
      left_point->set_theta(0.0);
      left_s += delta_distance;
      left_point->set_s(left_s);
      left_point->set_kappa(0);
      // right lane point
      right_point->set_x(x);
      right_point->set_y(-std::get<4>(extension_lane_width));
      right_point->set_theta(0.0);
      right_s += delta_distance;
      right_point->set_s(right_s);
      right_point->set_kappa(0);
    }
    return;
  }
  int back_max_index = floor(back_length / delta_distance);
  for (int i = 0; i < back_max_index; i++) {
    auto* point = init_points->add_path_point();
    auto* left_point = init_left_points->add_path_point();
    auto* right_point = init_right_points->add_path_point();
    s += delta_distance;
    auto x = i * delta_distance - back_length;
    point->set_x(x);

    point->set_y(0.0);
    point->set_theta(0.0);
    point->set_s(s);
    point->set_kappa(1 / radius);
    // left lane point
    left_point->set_x(x);
    left_point->set_y(std::get<3>(extension_lane_width));
    left_point->set_theta(0.0);
    left_s += delta_distance;
    left_point->set_s(left_s);
    left_point->set_kappa(1 / radius);
    // right lane point
    right_point->set_x(x);
    right_point->set_y(-std::get<4>(extension_lane_width));
    right_point->set_theta(0.0);
    right_s += delta_distance;
    right_point->set_s(right_s);
    right_point->set_kappa(1 / radius);
  }
  ADEBUG << "back path size: " << init_points->path_point_size();
  double x = 0.0;
  double y = 0.0;
  double left_x = 0.0;
  double left_y = 0.0;
  double right_x = 0.0;
  double right_y = 0.0;
  double beta = 0.0;
  int phi_num = M_PI_2 / delta_angle;
  for (int i = 0; i < phi_num && s <= max_arc_length; i++) {
    double phi = i * delta_angle;
    x = radius * std::sin(phi);
    y = copysign(radius * (1 - std::cos(phi)), angle);

    left_x = radius * std::sin(phi) -
             copysign(std::get<3>(extension_lane_width) * std::sin(phi), angle);
    if (copysign(1.0, angle) >= 0.0) {
      left_y =
          copysign(radius * (1 - std::cos(phi)), angle) +
          copysign(std::get<3>(extension_lane_width) * std::cos(phi), angle);
      right_y =
          copysign(radius * (1 - std::cos(phi)), angle) -
          copysign(std::get<4>(extension_lane_width) * std::cos(phi), angle);
    } else {
      left_y =
          copysign(radius * (1 - std::cos(phi)), angle) -
          copysign(std::get<3>(extension_lane_width) * std::cos(phi), angle);
      right_y =
          copysign(radius * (1 - std::cos(phi)), angle) +
          copysign(std::get<4>(extension_lane_width) * std::cos(phi), angle);
    }
    right_x =
        radius * std::sin(phi) +
        copysign(std::get<4>(extension_lane_width) * std::sin(phi), angle);
    beta = copysign(phi, angle);
    // cneter lane point
    auto pre_point =
        init_points->path_point().at(init_points->path_point_size() - 1);
    auto* point = init_points->add_path_point();
    // left lane point
    auto left_pre_point = init_left_points->path_point().at(
        init_left_points->path_point_size() - 1);
    auto* left_point = init_left_points->add_path_point();
    // right lane point
    auto right_pre_point = init_right_points->path_point().at(
        init_right_points->path_point_size() - 1);
    auto* right_point = init_right_points->add_path_point();
    // cneter lane point
    point->set_x(x);
    point->set_y(y);
    point->set_theta(beta);
    point->set_kappa(1 / radius);
    s += std::hypot(x - pre_point.x(), y - pre_point.y());
    point->set_s(s);
    // left lane point
    left_point->set_x(left_x);
    left_point->set_y(left_y);
    left_point->set_theta(beta);
    left_point->set_kappa(1 / radius);
    left_s +=
        std::hypot(left_x - left_pre_point.x(), left_y - left_pre_point.y());
    // left_s = s;
    left_point->set_s(left_s);
    // right lane point
    right_point->set_x(right_x);
    right_point->set_y(right_y);
    right_point->set_theta(beta);
    right_point->set_kappa(1 / radius);
    right_s += std::hypot(right_x - right_pre_point.x(),
                          right_y - right_pre_point.y());
    // right_s = s;
    right_point->set_s(right_s);
  }
  ADEBUG << "arc path size: " << init_points->path_point_size();
  if (s < central_length) {
    // cneter lane point
    auto pre_point =
        init_points->path_point().at(init_points->path_point_size() - 2);
    auto end_point =
        init_points->path_point().at(init_points->path_point_size() - 1);
    double dx = end_point.x() - pre_point.x();
    double dy = end_point.y() - pre_point.y();
    double end_phi = end_point.theta();
    // left lane point
    auto left_pre_point = init_left_points->path_point().at(
        init_left_points->path_point_size() - 2);
    auto left_end_point = init_left_points->path_point().at(
        init_left_points->path_point_size() - 1);
    double left_dx = left_end_point.x() - left_pre_point.x();
    double left_dy = left_end_point.y() - left_pre_point.y();
    double left_end_phi = left_end_point.theta();
    // right lane point
    auto right_pre_point = init_right_points->path_point().at(
        init_right_points->path_point_size() - 2);
    auto right_end_point = init_right_points->path_point().at(
        init_right_points->path_point_size() - 1);
    double right_dx = right_end_point.x() - right_pre_point.x();
    double right_dy = right_end_point.y() - right_pre_point.y();
    double right_end_phi = right_end_point.theta();
    while (s < central_length) {
      // cneter lane point
      auto last_point =
          init_points->path_point().at(init_points->path_point_size() - 1);
      auto* point = init_points->add_path_point();
      // left lane point
      auto last_left_point = init_left_points->path_point().at(
          init_left_points->path_point_size() - 1);
      auto* left_point = init_left_points->add_path_point();
      // right lane point
      auto last_right_point = init_right_points->path_point().at(
          init_right_points->path_point_size() - 1);
      auto* right_point = init_right_points->add_path_point();
      // cneter lane point
      point->set_x(last_point.x() + dx);
      point->set_y(last_point.y() + dy);
      point->set_theta(end_phi);
      s += std::hypot(dx, dy);
      point->set_s(s);
      point->set_kappa(0);
      // left lane point
      left_point->set_x(last_left_point.x() + left_dx);
      left_point->set_y(last_left_point.y() + left_dy);
      left_point->set_theta(left_end_phi);
      left_s += std::hypot(left_dx, left_dy);
      left_point->set_s(left_s);
      left_point->set_kappa(0);
      // right lane point
      right_point->set_x(last_right_point.x() + right_dx);
      right_point->set_y(last_right_point.y() + right_dy);
      right_point->set_theta(right_end_phi);
      right_s += std::hypot(right_dx, right_dy);
      right_point->set_s(right_s);
      right_point->set_kappa(0);
    }
  }
}

bool AdaptiveCruise::GeneratePathPoints(
    double radius, double angle, const common::VehicleState& vehicle_state,
    std::tuple<double, double, double, double, double> extension_lane_width,
    std::tuple<common::Path, common::Path, common::Path>* center_path) {
  auto* init_path = &(std::get<0>(*center_path));
  auto* init_left_path = &(std::get<1>(*center_path));
  auto* init_right_path = &(std::get<2>(*center_path));
  double arc_length = std::max(30.0, vehicle_state.linear_velocity() * 8.0);
  // GenerateInitPoints(radius, angle, &init_path, &init_left_path,
  //                    &init_right_path, arc_length, extension_lane_width);
  GenerateCurrentLaneTrajectoryPoints(radius, angle, init_path, arc_length,
                                      0.0);
  // GenerateLeftLaneTrajectoryPoints(radius, angle, &init_left_path, arc_length,
  //                                  std::get<3>(extension_lane_width));
  // GenerateRightLaneTrajectoryPoints(radius, angle, &init_right_path, arc_length,
  //                                   std::get<4>(extension_lane_width));
  GenerateAdjacentLaneTrajectoryPoints(
      *init_path, init_left_path, extension_lane_width, 1, radius, angle, 5);
  GenerateAdjacentLaneTrajectoryPoints(
      *init_path, init_right_path, extension_lane_width, 2, radius, angle, 5);
  ADEBUG << "init_path size: " << init_path->path_point_size();
  TrajectoryPointTransforENU(init_path, vehicle_state);
  TrajectoryPointTransforENU(init_left_path, vehicle_state);
  TrajectoryPointTransforENU(init_right_path, vehicle_state);
  return true;
}

bool AdaptiveCruise::GenerateOneLane(const common::Path& path,
                                     hdmap::Lane* lane,
                                     double extension_one_lane_width,
                                     int lane_id, const int discrete_step) {
  static uint8_t lane_id_count{0};
  lane_id_count++;
  // ADEBUG << "path_point_size " << path.path_point_size();
  if (path.path_point_size() < 2) {
    AERROR << "The path length of line index is invalid";
    return false;
  }
  if (lane_id == 0) {
    lane->mutable_id()->set_id(absl::StrCat("0_current_", lane_id_count));
  } else if (lane_id == 1) {
    lane->mutable_id()->set_id(absl::StrCat("1_left_", lane_id_count));
  } else {
    lane->mutable_id()->set_id(absl::StrCat("2_right_", lane_id_count));
  }
  // lane types
  lane->set_type(hdmap::Lane::CITY_DRIVING);
  lane->set_turn(hdmap::Lane::NO_TURN);

  // speed limit
  lane->set_speed_limit(navi_hdmap_config_.default_speed_limit());

  // center line
  auto* curve_segment = lane->mutable_central_curve()->add_segment();
  curve_segment->set_heading(path.path_point(0).theta());
  curve_segment->set_length(path.path_point(path.path_point_size() - 1).s());
  lane->set_length(path.path_point(path.path_point_size() - 1).s());
  auto* line_segment = curve_segment->mutable_line_segment();

  // left boundary
  auto* left_boundary = lane->mutable_left_boundary();
  auto* left_boundary_type = left_boundary->add_boundary_type();
  left_boundary->set_virtual_(false);
  left_boundary_type->set_s(0.0);
  left_boundary_type->add_types(hdmap::LaneBoundaryType::DOTTED_WHITE);
  auto* left_segment =
      left_boundary->mutable_curve()->add_segment()->mutable_line_segment();

  // right boundary
  auto* right_boundary = lane->mutable_right_boundary();
  auto* right_boundary_type = right_boundary->add_boundary_type();
  right_boundary->set_virtual_(false);
  right_boundary_type->set_s(0.0);
  right_boundary_type->add_types(hdmap::LaneBoundaryType::DOTTED_WHITE);
  auto* right_segment =
      right_boundary->mutable_curve()->add_segment()->mutable_line_segment();

  const double half_lane_width = extension_one_lane_width * 0.5;
  double index = 0;
  line_segment->mutable_point()->Reserve(path.path_point_size());
  for (int i = 0; i < path.path_point_size();) {
    auto* point = line_segment->add_point();
    point->set_x(path.path_point(i).x());
    point->set_y(path.path_point(i).y());
    point->set_z(path.path_point(i).z());
    index = index + discrete_step;
    auto* left_sample = lane->add_left_sample();
    left_sample->set_s(path.path_point(i).s());
    left_sample->set_width(half_lane_width);
    left_segment->add_point()->CopyFrom(
        *point + half_lane_width * Vec2d::CreateUnitVec2d(
                                       path.path_point(i).theta() + M_PI_2));

    auto* right_sample = lane->add_right_sample();
    right_sample->set_s(path.path_point(i).s());
    right_sample->set_width(half_lane_width);
    right_segment->add_point()->CopyFrom(
        *point + half_lane_width * Vec2d::CreateUnitVec2d(
                                       path.path_point(i).theta() - M_PI_2));
    i += discrete_step;
  }
  return true;
}

bool AdaptiveCruise::SetRouting(TL::routing::RoutingResponse* inrouting,
                                TL::hdmap::Map* hd_map) {
  // Set road boundary
  int lane_num = hd_map->lane_size();
  if (lane_num < 1) {
    return false;
  }
  auto* road = hd_map->add_road();
  road->mutable_id()->set_id("road_adaptive_cruise");
  auto* section = road->add_section();
  bool has_left_lane = false;
  bool has_right_lane = false;
  for (int i = 0; i < lane_num; ++i) {
    auto* lane_id = section->add_lane_id();
    lane_id->CopyFrom(hd_map->lane(i).id());
    if (absl::StartsWith(hd_map->lane(i).id().id(), "1_left")) {
      has_left_lane = true;
    }
    if (absl::StartsWith(hd_map->lane(i).id().id(), "2_right")) {
      has_right_lane = true;
    }
  }
  auto* outer_polygon = section->mutable_boundary()->mutable_outer_polygon();
  auto* left_edge = outer_polygon->add_edge();
  left_edge->set_type(TL::hdmap::BoundaryEdge::LEFT_BOUNDARY);
  left_edge->mutable_curve()->CopyFrom(
      has_left_lane ? hd_map->lane(1).left_boundary().curve()
                    : hd_map->lane(0).left_boundary().curve());

  auto* right_edge = outer_polygon->add_edge();
  right_edge->set_type(TL::hdmap::BoundaryEdge::RIGHT_BOUNDARY);
  right_edge->mutable_curve()->CopyFrom(
      has_right_lane ? hd_map->lane(lane_num - 1).right_boundary().curve()
                     : hd_map->lane(0).right_boundary().curve());
  // Set routing info
  auto* routing_road = inrouting->add_road();
  routing_road->set_id(road->id().id());
  routing_road->mutable_passage()->Reserve(lane_num);
  for (int i = 0; i < lane_num; i++) {
    // set passage and routing
    auto* passage = routing_road->add_passage();
    passage->set_can_exit(false);
    passage->set_change_lane_type(routing::ChangeLaneType::FORWARD);
    auto* segment = passage->add_segment();
    segment->set_id(hd_map->lane(i).id().id());
    segment->set_start_s(0.0);
    segment->set_end_s(hd_map->lane(0).length());
    auto adc_lane_segment_points =
        hd_map->lane(i).central_curve().segment().at(0).line_segment().point();
    common::PointENU start_point = adc_lane_segment_points.at(0);
    int max_index = adc_lane_segment_points.size() - 1;
    common::PointENU end_point = adc_lane_segment_points.at(max_index);
    auto* routing_request = inrouting->mutable_routing_request();
    routing::LaneWaypoint waypoint;
    waypoint.set_id(hd_map->lane(i).id().id());
    waypoint.mutable_pose()->set_x(start_point.x());
    waypoint.mutable_pose()->set_y(start_point.y());
    waypoint.set_s(0.0);
    routing_request->add_waypoint()->CopyFrom(waypoint);
    waypoint.set_s(hd_map->lane(i).length());
    waypoint.mutable_pose()->set_x(end_point.x());
    waypoint.mutable_pose()->set_y(end_point.y());
    routing_request->add_waypoint()->CopyFrom(waypoint);
  }
  auto* routing_request = inrouting->mutable_routing_request();
  common::util::FillHeader("from_adaptive_cruise_routingrequest",
                           routing_request);
  common::util::FillHeader("from_adaptive_cruise_routing", inrouting);
  return true;
}

void AdaptiveCruise::SubjTrajKappaFusion(double kappa_yaw_rate,
                                         double kappa_by_steer_angle,
                                         double speed) {
  const double fusion_coeff = InterplLinear(speed, 3, 0, 8.3, 1);
  kappa_fusion_ =
      fusion_coeff * kappa_yaw_rate + (1 - fusion_coeff) * kappa_by_steer_angle;
  const double radius_tmp = SafeDivide(1, kappa_fusion_, 0.0001);
  radius_Fusion_ = acc_function_active_ && !acc_function_active_last_
                       ? Clamp(std::abs(radius_tmp), 6.0, 10000.0)
                       : radius_after_fusion_filter_.Filter(
                             Clamp(std::abs(radius_tmp), 6.0, 10000.0));
}

/*zhangyu calculate subject kappa by steer wheel angle*/
void AdaptiveCruise::KappaBySteerAngle(double angle, double speed, double lf,
                                       double lr, double Cf, double Cr,
                                       double mass) {
  const double curvaure_cog = SafeDivide(
      angle * (lf + lr),
      std::pow(lf + lr, 2) + mass * std::pow(speed, 2) * (lf / Cr - lr / Cf),
      0.00001);
  // ADEBUG << "curvaure_cog " << curvaure_cog;
  const double ratius_cog = SafeDivide(1, curvaure_cog, (0.00001));
  // ADEBUG << "ratius_cog " << ratius_cog;
  const double slip =
      curvaure_cog * (lr - (mass * std::pow(speed, 2) * lf / ((lf + lr) * Cr)));
  const double kappa_before_limit = SafeDivide(
      1,
      std::copysign(std::sqrt(std::pow(lr - ratius_cog * std::sin(slip), 2) +
                              std::pow(ratius_cog * std::cos(slip), 2)),
                    ratius_cog),
      (0.00001));
  const double kappa_after_limit = (0.6) * Clamp(kappa_before_limit, -0.5, 0.5);
  kappa_by_steer_angle_ =
      acc_function_active_ && !acc_function_active_last_
          ? kappa_after_limit
          : kappa_by_steer_angle_filter_.Filter(kappa_after_limit);
  radius_by_steer_angle_ = SafeDivide(1, kappa_by_steer_angle_, 0.0001);
}

std::pair<double, double> AdaptiveCruise::StabilityFactor(double delta,
                                                          double a, double b,
                                                          double k1, double k2,
                                                          double m) {
  const double K_delta =
      m / std::pow(a + b, 2) *
      (a / k2 - b / k1 * (SafeDivide(1.0, std::cos(delta), 0.00001)));
  const double K = m / std::pow(a + b, 2) * (a / k2 - b / k1);
  return std::make_pair(K_delta, K);
}

double AdaptiveCruise::YawRateByWheelAngle(double K_delta, double delta,
                                           double v, double a, double b) {
  const double L = a + b;
  const double omega =
      delta / L * SafeDivide(v, 1.0 + K_delta * std::pow(v, 2), 0.00001);
  return omega;
}

std::pair<double, double> AdaptiveCruise::KappaAndRadiusBySteerStabilityFactor(
    double delta, double a, double b, double k1, double k2, double m,
    double v) {
  const double L = a + b;
  const double Radius =
      L / delta *
      ((k1 * std::pow(L, 2) / (b * m)) +
       (a * k1 / (b * k2) - (SafeDivide(1.0, cos(delta), 0.00001))) *
           std::pow(v, 2)) /
      ((k1 * std::pow(L, 2) / (b * m)) +
       (1.0 - (SafeDivide(1.0, cos(delta), 0.00001))) * std::pow(v, 2));
  const double kappa = SafeDivide(1.0, Radius, 0.0001);
  return std::make_pair(Radius, kappa);
}

double AdaptiveCruise::YawrateFilter(double yaw_rate) {
  /*yaw_rate unit rads*/
  double filte_yaw_rate = 0.0;
  double yaw_rate_Filter = 0.0;
  static double yaw_rate_full = 0.0087;
  static double yaw_rate_negligable = 0.0055;
  const double yaw_rate_fast =
      (0.1667) * yaw_rate + (1 - 0.1667) * yaw_rate_filter_fast_last_;
  const double yaw_rate_slow =
      (0.3333) * yaw_rate + (1 - 0.3333) * filte_yaw_rate_last_;
  if (std::abs(yaw_rate_fast - yaw_rate_slow) > 0.0087 ||
      (std::abs(yaw_rate_fast) < 0.0035 &&
       std::abs(yaw_rate_fast) < std::abs(yaw_rate_slow))) {
    filte_yaw_rate = yaw_rate_fast;
  } else {
    filte_yaw_rate = yaw_rate_slow;
  }

  if (std::abs(filte_yaw_rate) >= yaw_rate_full) {
    yaw_rate_Filter = filte_yaw_rate;
  } else if (std::abs(filte_yaw_rate) >= yaw_rate_negligable) {
    yaw_rate_Filter = std::copysign(
        yaw_rate_full *
            std::max(0.0,
                     std::min(1.0, ((std::abs(filte_yaw_rate) -
                                     yaw_rate_negligable) /
                                    (yaw_rate_full - yaw_rate_negligable)))),
        filte_yaw_rate);
  } else {
    yaw_rate_Filter = 0.0;
  }
  yaw_rate_Filter = acc_function_active_ && !acc_function_active_last_
                        ? yaw_rate
                        : yaw_rate_Filter;
  yaw_rate_filter_fast_last_ = yaw_rate_fast;
  filte_yaw_rate_last_ = filte_yaw_rate;
  return yaw_rate_Filter;
}

void AdaptiveCruise::KappaByYawrate(double yaw_rate, double speed) {
  /*yaw_rate unit rads*/
  static double P_PLN_YawRateStartLPFilter = 5 / 57.3;
  static double P_PLN_YawRateFiltCutOffFreq = 0.085;
  UNUSED(P_PLN_YawRateFiltCutOffFreq);
  const double kappa_before_filter = SafeDivide(yaw_rate, speed, (0.01));
  if (acc_function_active_ && !acc_function_active_last_) {
    kappa_by_yaw_rate_ = kappa_before_filter;
  } else {
    if (yaw_rate > P_PLN_YawRateStartLPFilter) {
      kappa_by_yaw_rate_ =
          kappa_by_yaw_rate_filter_.Filter(kappa_before_filter);
    } else {
      kappa_by_yaw_rate_ = kappa_before_filter;
    }
  }
  radius_by_yaw_rate_ = SafeDivide(1, kappa_by_yaw_rate_, 0.0001);
}

std::tuple<bool, double, bool, double>
AdaptiveCruise::AdjacentLaneExtensionSign(const double left_c0,
                                          const double left_quality,
                                          const double right_c0,
                                          const double right_quality) {
  // 左侧路沿到自车巡航道路左侧边线的距离 abs
  double distance_cruise_left_to_left_edge =
      left_c0 - 0.5 * FLAGS_adaptive_cruise_lane_width;
  // 右侧路沿到自车巡航道路右侧边线的距离 abs
  double distance_cruise_right_to_right_edge =
      right_c0 + 0.5 * FLAGS_adaptive_cruise_lane_width;
  // 左侧巡航道路是否可扩展
  bool left_extension =
      (distance_cruise_left_to_left_edge > 2.5 && left_quality > 0.5) ||
      left_quality < 0.2;
  distance_cruise_left_to_left_edge = left_quality < 0.2
                                          ? FLAGS_adaptive_cruise_lane_width
                                          : distance_cruise_left_to_left_edge;
  // 右侧巡航道路是否可扩展
  bool right_extension =
      (distance_cruise_right_to_right_edge < -2.5 && right_quality > 0.5) ||
      right_quality < 0.2;
  distance_cruise_right_to_right_edge =
      right_quality < 0.2 ? FLAGS_adaptive_cruise_lane_width
                          : std::abs(distance_cruise_right_to_right_edge);
  return std::make_tuple(left_extension, distance_cruise_left_to_left_edge,
                         right_extension, distance_cruise_right_to_right_edge);
}

double AdaptiveCruise::SafeDivide(double nom, double denom, double threshold) {
  double denominator = 0.0;
  if (fabs(denom) > threshold) {
    denominator = denom;
  } else if (denom > 0) {
    denominator = threshold;
  } else {
    denominator = -threshold;
  }
  return nom / denominator;
}

void AdaptiveCruise::TrajectoryPointTransforENU(
    common::Path* const path_point, const common::VehicleState& vehicle_state) {
  for (int i = 0; i < path_point->path_point_size(); ++i) {
    double x1 = path_point->path_point().at(i).x();
    double y1 = path_point->path_point().at(i).y();
    double theta = path_point->path_point().at(i).theta();
    Eigen::Vector2d enu_coordinate =
        common::math::RotateVector2d({x1, y1}, vehicle_state.heading());
    double x_enu = enu_coordinate.x() + vehicle_state.x();
    double y_enu = enu_coordinate.y() + vehicle_state.y();
    double theta_enu = common::math::NormalizeAngle(
        common::math::NormalizeAngle(theta) + vehicle_state.heading());
    path_point->mutable_path_point()->at(i).set_x(x_enu);
    path_point->mutable_path_point()->at(i).set_y(y_enu);
    path_point->mutable_path_point()->at(i).set_theta(theta_enu);
  }
}

}  // namespace planning
}  // namespace TL
