/*
 * Copyright (c) TL auto Co., Ltd. 2021-2022. All rights reserved.
 */
#pragma once
#include <deque>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "common/file/log.h"
#include "common/math/vec2d.h"
#include "common/time/clock.h"

#include "proto/perception/perception_obstacle.pb.h"
#include "proto/planning/lanemarkers_lane_line.pb.h"

namespace TL {
namespace planning {
namespace lanelineprocess {
using TL::common::Clock;  // NOLINT
using TL::common::math::Vec2d;
using TL::perception::LaneMarker;
using TL::perception::LaneMarkers;
constexpr double kInitMinLoopTime = 0.1;
constexpr double kInitGoodQualityThrd = 0.3;
constexpr double kInitBatterQualityThrd = 0.6;
constexpr double kInitLaneWidth = 1.875;
constexpr double kMinLaneDis = 0.1;
constexpr int kMagicNumber4 = 4;
constexpr int kMagicNumber5 = 5;
constexpr int kMagicNumber6 = 6;
constexpr double kCurvatureNum = 1.5;
constexpr double kMinC0 = 0.005;
constexpr double kKph2ms = 3.6;
constexpr double kHalfNum = 0.5;

class LowPassFilter {
 public:
  LowPassFilter() = default;
  ~LowPassFilter() = default;
  double Filter(double x_insert);
  void set_fs_and_fc(double fs, double fc);

 private:
  const double pi = 3.14159265358979324;
  double last_x_insert_{0.0};
  double last_y_output_{0.0};
  double fs_{0.0};
  double fc_{0.0};
  bool is_first_filter_ = true;
};

class FirstOrderLowerPassFilter {
 public:
  FirstOrderLowerPassFilter() = default;
  ~FirstOrderLowerPassFilter() = default;
  double Filter(double x_insert);
  void SetCoefficientAndFlag(double filter_coefficient, bool filter_reset);

 private:
  double last_output_{0.0};
  double filter_coefficient_{0.0};
  bool filter_reset_ = false;
  bool is_first_step_ = false;
};

class DebounceModule {
 public:
  DebounceModule() = default;
  DebounceModule(double rise_time, double fall_time, double main_loop_time);
  ~DebounceModule() = default;
  void ResetTime(double rise_time, double fall_time, double main_loop_time);
  void Reset();
  bool DealDebounce(bool input);

  bool LastStatus() const { return in_pre_; }

 private:
  bool in_pre_ = false;
  double rise_time_val_ = 0.0;
  double fall_time_val_ = 0.0;
  double rise_time_limit_{kInitMinLoopTime};
  double fall_time_limit_{0.0};
  double main_loop_time_{kInitMinLoopTime};
};

class RunningTimeMeter {
 public:
  RunningTimeMeter() = default;
  ~RunningTimeMeter() = default;
  void Start();
  [[nodiscard]] double DiffTime() const;
  void Stop();

 private:
  bool has_started_ = false;
  double start_time_ = 0.0;
};

enum LaneMarkerState {
  BAD_LANEMARKER = 0,
  GOOD_LANEMARKER = 1,
  BETTER_LANEMARKER = 2
};

struct LaneMarkersState {
  LaneMarkerState left_lanemarker_state;
  LaneMarkerState right_lanemarker_state;
  LaneMarkerState next_left_lanemarker_state;
  LaneMarkerState next_right_lanemarker_state;
};

struct VehicleState {
  // m/s
  double vehicle_speed_average;
  // rad/s
  double vehicle_yaw_rate;
};

// lane width predict and vaild output
struct LneWdeValid {
  int next_leftlane_widthpredict_valid = 0;
  double next_leftlane_widthpredict_sg = 2.75;
  int next_rightlane_widthpredict_valid = 0;
  double next_rightlane_widthpredict_sg = 2.75;
  int ego_lane_widthpredict_valid = 1;
  double ego_lane_widthpredict_sg = 2.75;
};

// lane reset and quality monitor output
struct LaneReset {
  bool is_lane_reset = false;
  bool is_lane_coast = false;
  double lane_quality{1.0};
  double lane_length = 50.0;
  bool split_lane_flag = false;
};

struct ResetAndQualityOut {
  LaneReset left_lane_reset;
  LaneReset right_lane_reset;
  LaneReset next_left_lane_reset;
  LaneReset next_right_lane_reset;
};

// lane chnage output
struct LaneChangeOut {
  bool right_lanechange_mnt_flag = false;
  LaneMarker right_coff_fix_lanemarker;
  bool left_lanechange_mnt_flag = true;
  LaneMarker left_coff_fix_lanemarker;
};

struct DeciderData {
  // speed and yaw rate
  VehicleState filter_vehicle_state{};
  // original lanemarkers view
  LaneMarkers original_lanemarkers;
  // after coordinate transformation lanemarkers view
  LaneMarkers trans_lanemarkers;
  // lanemarker after copy decider
  LaneMarkers copy_lanemarkers;
  // lanemarker after filter
  LaneMarkers filter_lanemarkers;
  // lanemarker out
  LaneMarkers out_lanemarkers;
  LneWdeValid lane_width_valid_out;
  ResetAndQualityOut lane_reset_out;
  LaneChangeOut lane_change_out;
  LaneMarkersState lane_markers_state;
  std::unordered_map<std::string, std::vector<Vec2d>> lane_marker_points;

  bool is_lanechange_to_left = false;
  bool is_lanechange_to_right = false;
  bool is_host_lanechange = false;
  // 存储由c2和车速查表得到的当前车道，左车道，右车道的距离，在该距离下计算车道宽度
  std::tuple<double, double, double> predict_width_distance_;
  double lane_speed_cost_time{0.0};

  double road_average_curvature = 0.0;
  bool do_egolane_update = true;
  bool do_nextlane_update = false;
  std::shared_ptr<LanemarkersLaneLine> lanemarker_lanline_debug = nullptr;
};
}  // namespace lanelineprocess
}  // namespace planning
}  // namespace TL
