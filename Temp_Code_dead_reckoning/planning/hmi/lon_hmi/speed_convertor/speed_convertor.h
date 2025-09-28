/******************************************************************************
 * Copyright 2022 The TL Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

/**
 * @namespace apollo::common
 * @brief apollo::common
 */
#include <sys/types.h>
#include <unordered_map>
#include <utility>
#include <vector>
#include "planning/proto/hmi_config.pb.h"
#include "proto/common/vehicle_state.pb.h"
#include "proto/fsm/function_manager.pb.h"
#include "proto/soc/chassis.pb.h"

namespace TL {
namespace planning {
static constexpr int kMinCruiseSpd = 30;
static constexpr double kMinRatio = 0.5;
static constexpr double kMaxRatio = 1.5;
static constexpr double kMaxSpeedDiff = 5.0;
static constexpr int kPreviewSpeed = 10;

class SpeedConventor {
 public:
  struct SpeedInfo {
    double target_speed = 0.0;
    double min_speed = 0.0;
    double max_speed = 0.0;
    bool calibrated = false;
  };

  /**
   * @brief 
   * 
   * @param spd_ms_display 
   * @return double 
   */
  static double ConvertDisplaySpdToReal(int spd_km_display);
  /**
   * @brief 
   * 
   * @param speed_adapt_config 
   */
  static void InitTable(
      const functionmanager::SpeedAdaptConfig& speed_adapt_config);
  /**
   * @brief Get the Offset object
   * 
   * @param table 
   * @param spd_ms_display 
   * @return double 
   */
  static double GetOffset(const std::vector<std::pair<double, double>>& table,
                          int spd_km_display);
  /**
   * @brief 
   * 
   * @param vehicle_state 
   */
  static void UpdateOnlineTable(
      const TL::common::VehicleState& vehicle_state, bool is_hdmap_mode);
  /**
   * @brief 
   * 
   * @param real_speedms 
   * @param speed_display 
   * @param table 
   */
  static void UpdateTable(double real_speedms, int speed_display,
                          int last_speed_display,
                          std::unordered_map<int, SpeedInfo>* table);

 private:
  static bool is_hdmap_mode_;
  static std::vector<std::pair<double, double>> off_line_hd_map_table_;
  static std::vector<std::pair<double, double>> off_line_perception_table_;
  static double ratio_;
  static std::unordered_map<int, SpeedInfo> on_line_hd_map_table_;
  static std::unordered_map<int, SpeedInfo> on_line_perception_table_;
  static uint min_cruise_speed_km_;
};
}  // namespace planning
}  // namespace TL
