/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "common/configs/vehicle_config_helper.h"

#include <algorithm>
#include <cmath>
#include <vector>
#ifdef ISORIN
#include "cfg/config_param.h"
#endif
#include "common/configs/config_gflags.h"
#include "common/file/file.h"
#include "common/math/math_utils.h"
#include "common/math/vec2d.h"

namespace TL {
namespace common {

VehicleConfig VehicleConfigHelper::vehicle_config_;  // NOLINT
bool VehicleConfigHelper::is_init_ = false;

void VehicleConfigHelper::Init() {
#ifdef ISORIN
  // ConfigParam
  auto* cfg_client = TL::netaos::cfg::ConfigParam::Instance();
  if (cfg_client == nullptr) {
    return;
  }
  uint8_t cfg = 0;
  TL::netaos::cfg::CfgResultCode ret =
      TL::netaos::cfg::CfgResultCode::CONFIG_TIME_OUT;
  ret = cfg_client->GetParam<uint8_t>("vehiclecfg/intelligentControlPlatform",
                                      cfg);
  AINFO << " read intelligentControlPlatform status:" << ret << "value:" << cfg;

  if ((TL::netaos::cfg::CfgResultCode::CONFIG_OK != ret) || (0x2 != cfg)) {
    // 读取配置字不成功 或 非滑板底盘 都返回为 主项目
    // 通过配置字区分EP41主项目和CTC项目,0X2:SICC代表滑板底盘，0X1:XPC代表主项目
    // 默认值为1
    cfg = 1;
  }
  //
  if (cfg == 0x2) {
    Init(FLAGS_vehicle_config_path_EP41_CTC);
  } else {
    Init(FLAGS_vehicle_config_path);
  }
#else
  Init(FLAGS_vehicle_config_path);
#endif
}

void VehicleConfigHelper::Init(const std::string& config_file) {
  VehicleConfig params;
  ACHECK(common::GetProtoFromFile(config_file, &params))
      << "Unable to parse vehicle config file " << config_file;
  Init(params);
}

void VehicleConfigHelper::Init(const VehicleConfig& vehicle_params) {
  vehicle_config_ = vehicle_params;
  is_init_ = true;
}

const VehicleConfig& VehicleConfigHelper::GetConfig() {
  if (!is_init_) {
    Init();
  }
  return vehicle_config_;
}

double VehicleConfigHelper::MinSafeTurnRadius() {
  const auto& param = vehicle_config_.vehicle_param();
  double lat_edge_to_center =
      std::max(param.left_edge_to_center(), param.right_edge_to_center());
  double lon_edge_to_center =
      std::max(param.front_edge_to_center(), param.back_edge_to_center());
  return std::sqrt((lat_edge_to_center + param.min_turn_radius()) *
                       (lat_edge_to_center + param.min_turn_radius()) +
                   lon_edge_to_center * lon_edge_to_center);
}

common::math::Box2d VehicleConfigHelper::GetBoundingBox(
    const common::PathPoint& path_point) {
  const auto& vehicle_param = vehicle_config_.vehicle_param();
  double diff_truecenter_and_pointX = (vehicle_param.front_edge_to_center() -
                                       vehicle_param.back_edge_to_center()) /
                                      2.0;
  common::math::Vec2d true_center(
      path_point.x() +
          diff_truecenter_and_pointX * std::cos(path_point.theta()),
      path_point.y() +
          diff_truecenter_and_pointX * std::sin(path_point.theta()));
  common::math::Box2d box_2d(true_center, path_point.theta(),
                             vehicle_param.length(), vehicle_param.width());
  return box_2d;
}

common::math::Box2d VehicleConfigHelper::GetBoundingBox(
    const common::PathPoint& path_point, const double extra_length,
    const double extra_width) {
  const auto& vehicle_param = vehicle_config_.vehicle_param();
  double diff_truecenter_and_pointX = (vehicle_param.front_edge_to_center() -
                                       vehicle_param.back_edge_to_center()) /
                                      2.0;
  common::math::Vec2d true_center(
      path_point.x() +
          diff_truecenter_and_pointX * std::cos(path_point.theta()),
      path_point.y() +
          diff_truecenter_and_pointX * std::sin(path_point.theta()));
  common::math::Box2d box_2d(true_center, path_point.theta(),
                             vehicle_param.length() + extra_length,
                             vehicle_param.width() + extra_width);
  return box_2d;
}

common::math::Polygon2d VehicleConfigHelper::GetPolygon2dWithBuffer(
    const double x, const double y, const double theta,
    const double front_buffer, const double rear_buffer,
    const double left_buffer, const double right_buffer) {
  std::vector<common::math::Vec2d> points;
  const auto& ultrasonic_position =
      vehicle_config_.vehicle_param().ultrasonic_position();
  auto s1_enu_pair = common::math::FRDToENU(
      ultrasonic_position.ultrasonic_s1().point().x() + front_buffer,
      ultrasonic_position.ultrasonic_s1().point().y() - left_buffer, x, y,
      theta);
  points.emplace_back(s1_enu_pair.first, s1_enu_pair.second);
  auto s2_enu_pair = common::math::FRDToENU(
      ultrasonic_position.ultrasonic_s2().point().x() + front_buffer,
      ultrasonic_position.ultrasonic_s2().point().y() - left_buffer, x, y,
      theta);
  points.emplace_back(s2_enu_pair.first, s2_enu_pair.second);
  auto s3_enu_pair = common::math::FRDToENU(
      ultrasonic_position.ultrasonic_s3().point().x() + front_buffer,
      ultrasonic_position.ultrasonic_s3().point().y() - left_buffer, x, y,
      theta);
  points.emplace_back(s3_enu_pair.first, s3_enu_pair.second);
  auto s4_enu_pair = common::math::FRDToENU(
      ultrasonic_position.ultrasonic_s4().point().x() + front_buffer,
      ultrasonic_position.ultrasonic_s4().point().y() + right_buffer, x, y,
      theta);
  points.emplace_back(s4_enu_pair.first, s4_enu_pair.second);
  auto s5_enu_pair = common::math::FRDToENU(
      ultrasonic_position.ultrasonic_s5().point().x() + front_buffer,
      ultrasonic_position.ultrasonic_s5().point().y() + right_buffer, x, y,
      theta);
  points.emplace_back(s5_enu_pair.first, s5_enu_pair.second);
  auto s6_enu_pair = common::math::FRDToENU(
      ultrasonic_position.ultrasonic_s6().point().x() + front_buffer,
      ultrasonic_position.ultrasonic_s6().point().y() + right_buffer, x, y,
      theta);
  points.emplace_back(s6_enu_pair.first, s6_enu_pair.second);
  auto s7_enu_pair = common::math::FRDToENU(
      ultrasonic_position.ultrasonic_s7().point().x() - rear_buffer,
      ultrasonic_position.ultrasonic_s7().point().y() + right_buffer, x, y,
      theta);
  points.emplace_back(s7_enu_pair.first, s7_enu_pair.second);
  auto s8_enu_pair = common::math::FRDToENU(
      ultrasonic_position.ultrasonic_s8().point().x() - rear_buffer,
      ultrasonic_position.ultrasonic_s8().point().y() + right_buffer, x, y,
      theta);
  points.emplace_back(s8_enu_pair.first, s8_enu_pair.second);
  auto s9_enu_pair = common::math::FRDToENU(
      ultrasonic_position.ultrasonic_s9().point().x() - rear_buffer,
      ultrasonic_position.ultrasonic_s9().point().y() + right_buffer, x, y,
      theta);
  points.emplace_back(s9_enu_pair.first, s9_enu_pair.second);
  auto s10_enu_pair = common::math::FRDToENU(
      ultrasonic_position.ultrasonic_s10().point().x() - rear_buffer,
      ultrasonic_position.ultrasonic_s10().point().y() - left_buffer, x, y,
      theta);
  points.emplace_back(s10_enu_pair.first, s10_enu_pair.second);
  auto s11_enu_pair = common::math::FRDToENU(
      ultrasonic_position.ultrasonic_s11().point().x() - rear_buffer,
      ultrasonic_position.ultrasonic_s11().point().y() - left_buffer, x, y,
      theta);
  points.emplace_back(s11_enu_pair.first, s11_enu_pair.second);
  auto s12_enu_pair = common::math::FRDToENU(
      ultrasonic_position.ultrasonic_s12().point().x() - rear_buffer,
      ultrasonic_position.ultrasonic_s12().point().y() - left_buffer, x, y,
      theta);
  points.emplace_back(s12_enu_pair.first, s12_enu_pair.second);
  return common::math::Polygon2d(points);
}

std::vector<std::pair<common::math::Vec2d, double>>
VehicleConfigHelper::GetMirrorCirclesWithBuffer(double x, double y,
                                                double theta,
                                                double left_buffer,
                                                double right_buffer) {
  const auto& vehicle_param = vehicle_config_.vehicle_param();
  const auto left_mirror_center =
      common::math::RFUToENU(-0.5 * vehicle_param.width(),
                             vehicle_param.mirror_to_center(), x, y, theta);
  const auto right_mirror_center =
      common::math::RFUToENU(0.5 * vehicle_param.width(),
                             vehicle_param.mirror_to_center(), x, y, theta);
  std::vector<std::pair<common::math::Vec2d, double>> circles;
  circles.emplace_back(
      common::math::Vec2d(left_mirror_center.first, left_mirror_center.second),
      left_buffer + vehicle_param.mirror_radius());
  circles.emplace_back(common::math::Vec2d(right_mirror_center.first,
                                           right_mirror_center.second),
                       right_buffer + vehicle_param.mirror_radius());
  return circles;
}

std::vector<common::math::Vec2d> VehicleConfigHelper::GetAllRadarPos(
    const double x, const double y, const double theta) {
  std::vector<common::math::Vec2d> points;
  const auto& ultrasonic_position =
      vehicle_config_.vehicle_param().ultrasonic_position();
  auto s1_enu_pair = common::math::FRDToENU(
      ultrasonic_position.ultrasonic_s1().point().x(),
      ultrasonic_position.ultrasonic_s1().point().y(), x, y, theta);
  points.emplace_back(s1_enu_pair.first, s1_enu_pair.second);
  auto s2_enu_pair = common::math::FRDToENU(
      ultrasonic_position.ultrasonic_s2().point().x(),
      ultrasonic_position.ultrasonic_s2().point().y(), x, y, theta);
  points.emplace_back(s2_enu_pair.first, s2_enu_pair.second);
  auto s3_enu_pair = common::math::FRDToENU(
      ultrasonic_position.ultrasonic_s3().point().x(),
      ultrasonic_position.ultrasonic_s3().point().y(), x, y, theta);
  points.emplace_back(s3_enu_pair.first, s3_enu_pair.second);
  auto s4_enu_pair = common::math::FRDToENU(
      ultrasonic_position.ultrasonic_s4().point().x(),
      ultrasonic_position.ultrasonic_s4().point().y(), x, y, theta);
  points.emplace_back(s4_enu_pair.first, s4_enu_pair.second);
  auto s5_enu_pair = common::math::FRDToENU(
      ultrasonic_position.ultrasonic_s5().point().x(),
      ultrasonic_position.ultrasonic_s5().point().y(), x, y, theta);
  points.emplace_back(s5_enu_pair.first, s5_enu_pair.second);
  auto s6_enu_pair = common::math::FRDToENU(
      ultrasonic_position.ultrasonic_s6().point().x(),
      ultrasonic_position.ultrasonic_s6().point().y(), x, y, theta);
  points.emplace_back(s6_enu_pair.first, s6_enu_pair.second);
  auto s7_enu_pair = common::math::FRDToENU(
      ultrasonic_position.ultrasonic_s7().point().x(),
      ultrasonic_position.ultrasonic_s7().point().y(), x, y, theta);
  points.emplace_back(s7_enu_pair.first, s7_enu_pair.second);
  auto s8_enu_pair = common::math::FRDToENU(
      ultrasonic_position.ultrasonic_s8().point().x(),
      ultrasonic_position.ultrasonic_s8().point().y(), x, y, theta);
  points.emplace_back(s8_enu_pair.first, s8_enu_pair.second);
  auto s9_enu_pair = common::math::FRDToENU(
      ultrasonic_position.ultrasonic_s9().point().x(),
      ultrasonic_position.ultrasonic_s9().point().y(), x, y, theta);
  points.emplace_back(s9_enu_pair.first, s9_enu_pair.second);
  auto s10_enu_pair = common::math::FRDToENU(
      ultrasonic_position.ultrasonic_s10().point().x(),
      ultrasonic_position.ultrasonic_s10().point().y(), x, y, theta);
  points.emplace_back(s10_enu_pair.first, s10_enu_pair.second);
  auto s11_enu_pair = common::math::FRDToENU(
      ultrasonic_position.ultrasonic_s11().point().x(),
      ultrasonic_position.ultrasonic_s11().point().y(), x, y, theta);
  points.emplace_back(s11_enu_pair.first, s11_enu_pair.second);
  auto s12_enu_pair = common::math::FRDToENU(
      ultrasonic_position.ultrasonic_s12().point().x(),
      ultrasonic_position.ultrasonic_s12().point().y(), x, y, theta);
  points.emplace_back(s12_enu_pair.first, s12_enu_pair.second);
  return points;
}

}  // namespace common
}  // namespace TL
