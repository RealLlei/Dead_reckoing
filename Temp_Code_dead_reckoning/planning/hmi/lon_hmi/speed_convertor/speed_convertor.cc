/******************************************************************************
 * Copyright 2022 The TL Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/hmi/lon_hmi/speed_convertor/speed_convertor.h"
#include <sys/types.h>
#include <algorithm>
#include <cmath>
#include "common/configs/config_gflags.h"
#include "common/file/log.h"
#include "proto/soc/chassis.pb.h"

namespace TL {
namespace planning {

bool SpeedConventor::is_hdmap_mode_ = false;
double SpeedConventor::ratio_ = 1.05;
std::vector<std::pair<double, double>> SpeedConventor::off_line_hd_map_table_ =
    {};
std::vector<std::pair<double, double>>
    SpeedConventor::off_line_perception_table_ = {};
std::unordered_map<int, SpeedConventor::SpeedInfo>
    SpeedConventor::on_line_hd_map_table_ = {};
std::unordered_map<int, SpeedConventor::SpeedInfo>
    SpeedConventor::on_line_perception_table_ = {};
uint SpeedConventor::min_cruise_speed_km_ = 25;

void SpeedConventor::InitTable(
    const functionmanager::SpeedAdaptConfig& speed_adapt_config) {
  ratio_ = speed_adapt_config.speed_convertor_ratio();
  min_cruise_speed_km_ = speed_adapt_config.min_speed();
  const auto& hd_map_config =
      speed_adapt_config.hd_map_speed_convertor_config();
  const auto hd_map_config_size = hd_map_config.speed_size();
  if (hd_map_config_size == 0 ||
      hd_map_config_size != hd_map_config.offset_size()) {
    AERROR << " init hd map table error";
    return;
  }
  for (auto i = 0; i < hd_map_config_size; i++) {
    off_line_hd_map_table_.emplace_back(hd_map_config.speed().at(i),
                                        hd_map_config.offset().at(i));
  }

  const auto& perception_map_config =
      speed_adapt_config.perception_speed_convertor_config();
  const auto perception_map_config_size = perception_map_config.speed_size();
  if (perception_map_config_size == 0 ||
      perception_map_config_size != perception_map_config.offset_size()) {
    AERROR << " init perception table error";
    return;
  }
  for (auto i = 0; i < perception_map_config_size; i++) {
    off_line_perception_table_.emplace_back(
        perception_map_config.speed().at(i),
        perception_map_config.offset().at(i));
  }
  on_line_hd_map_table_.reserve(140);
  on_line_perception_table_.reserve(140);
}

double SpeedConventor::GetOffset(
    const std::vector<std::pair<double, double>>& table,
    const int spd_km_display) {
  for (const auto& config : table) {
    if (spd_km_display < config.first) {
      return config.second;
    }
  }
  return 0.3;
}

double SpeedConventor::ConvertDisplaySpdToReal(const int spd_km_display) {
#ifdef FOR_BAIDU_SIMULATION
  return spd_km_display / 3.6;
#endif
  if (is_hdmap_mode_ && on_line_hd_map_table_.count(spd_km_display) > 0) {
    return on_line_hd_map_table_.at(spd_km_display).target_speed;
  }

  if (!is_hdmap_mode_ && on_line_perception_table_.count(spd_km_display) > 0) {
    return on_line_perception_table_.at(spd_km_display).target_speed;
  }
  return spd_km_display / 3.6;

  // const auto offset =
  //     is_hdmap_mode_ ? GetOffset(off_line_hd_map_table_, spd_km_display)
  //                    : GetOffset(off_line_perception_table_, spd_km_display);

  // ADEBUG << "is_hdmap_mode_ :" << is_hdmap_mode_ << " constant : " << offset;
  // return spd_km_display / (3.6 * ratio_) + offset;
}

void SpeedConventor::UpdateOnlineTable(
    const TL::common::VehicleState& vehicle_state,
    const bool is_hdmap_mode) {
#ifdef FOR_BAIDU_SIMULATION
  return;
#endif

  const auto& chassis = vehicle_state.chassis();
  is_hdmap_mode_ = is_hdmap_mode;
  if (!chassis.has_speed_display() ||
      chassis.speed_display() < min_cruise_speed_km_) {
    return;
  }
  static int last_display = chassis.speed_display();
  const auto speed_display = chassis.speed_display();
  const auto ratio = chassis.speed_display() /
                     fmax((3.6 * vehicle_state.linear_velocity()), 0.01);
  const auto speed_diff =
      3.6 * vehicle_state.linear_velocity() - chassis.speed_display();
  if (ratio > kMaxRatio || ratio < kMinRatio || speed_diff > kMaxSpeedDiff ||
      fabs(vehicle_state.linear_acceleration()) > 1.0) {
    last_display = chassis.speed_display();
    return;
  }
  UpdateTable(chassis.speed_mps(), speed_display, last_display,
              &on_line_perception_table_);
  if (is_hdmap_mode_) {
    UpdateTable(vehicle_state.linear_velocity(), speed_display, last_display,
                &on_line_hd_map_table_);
  }
  last_display = chassis.speed_display();
}

void SpeedConventor::UpdateTable(
    const double real_speedms, const int speed_display,
    const int last_speed_display,
    std::unordered_map<int, SpeedInfo>* const table) {
  if (table == nullptr) {
    return;
  }
  if (table->count(speed_display) < 1) {
    auto& speed_info = (*table)[speed_display];
    speed_info.min_speed = real_speedms;
    speed_info.target_speed = real_speedms;
    speed_info.max_speed = real_speedms;
    speed_info.calibrated = true;
    const auto offset = speed_display / 3.6 - real_speedms;
    for (int i = -kPreviewSpeed; i <= kPreviewSpeed; i++) {
      const auto speed = speed_display + i;
      if (speed < min_cruise_speed_km_) {
        continue;
      }
      auto& init_speed_info = (*table)[speed];
      if (init_speed_info.calibrated) {
        continue;
      }
      init_speed_info.target_speed = speed / 3.6 - offset;
    }
  } else {
    auto& speed_info = table->at(speed_display);
    if (!speed_info.calibrated) {
      speed_info.min_speed = real_speedms;
      speed_info.max_speed = real_speedms;
    }
    speed_info.calibrated = true;
    speed_info.max_speed = speed_info.max_speed * 0.5 +
                           fmax(speed_info.max_speed, real_speedms) * 0.5;
    speed_info.min_speed = speed_info.min_speed * 0.5 +
                           fmin(speed_info.min_speed, real_speedms) * 0.5;

    speed_info.target_speed =
        (speed_info.max_speed + speed_info.min_speed) * 0.5;
  }
  // 45-> 46，更新45的max和46的min，当前是46
  // 46-> 45，更新45的max和46的min,当前是45
  if ((abs(last_speed_display - speed_display) == 1) &&
      table->count(last_speed_display) > 0 &&
      table->at(last_speed_display).calibrated) {
    auto& speed_info = table->at(speed_display);
    auto& last_speed = table->at(last_speed_display);
    auto& lower = last_speed_display < speed_display ? last_speed : speed_info;
    lower.max_speed = real_speedms;
    auto& upper = last_speed_display < speed_display ? speed_info : last_speed;
    upper.min_speed = real_speedms;
    last_speed.target_speed =
        (last_speed.max_speed + last_speed.min_speed) * 0.5;
    speed_info.target_speed =
        (speed_info.max_speed + speed_info.min_speed) * 0.5;
  }
}

}  // namespace planning
}  // namespace TL
