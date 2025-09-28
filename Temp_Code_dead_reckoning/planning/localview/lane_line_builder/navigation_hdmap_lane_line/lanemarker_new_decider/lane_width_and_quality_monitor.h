/*
 * @Author: 80040285 zhangyu@TLauto.com
 * @Date: 2023-08-31 13:56:57
 * @LastEditors: 80040285 zhangyu@TLauto.com
 * @LastEditTime: 2023-08-31 17:51:49
 * @FilePath: /europa/planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/lane_width_and_quality_monitor.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*
 * Copyright (c) TL auto Co., Ltd. 2021-2022. All rights reserved.
 */
#pragma once
#include <algorithm>
#include <cassert>
#include <iostream>
#include <string>
#include <vector>

#include "common/math/math_utils.h"
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/decider_data.h"
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/lane_line_delay.h"
#include "planning/proto/navigation_hdmap_config.pb.h"

namespace TL {
namespace planning {
namespace lanelineprocess {
// Lane Determine Reset module
class LaneWidthQualityMonitor {
 public:
  LaneWidthQualityMonitor() = default;
  explicit LaneWidthQualityMonitor(const planning::PerceptionMapConfig& config);
  void Init(const planning::PerceptionMapConfig& config);
  ~LaneWidthQualityMonitor() = default;
  void SetConf(double width_var_thd);
  void DealCommonData(double lane_coast_time,
                      double max_predict_width_distance);
  bool Update(const LaneMarker& origin_lanemarker,
              const LaneMarker& side_lanemarker, int lane_width_predict_valid,
              double lane_width_predict_sg, bool lane_change,
              LaneSplit* lane_split_debug, LaneMarkerState lanemarker_state);

 private:
  bool LaneWidthMonitor(const LaneMarker& origin_lanemarker,
                        const LaneMarker& side_lanemarker,
                        double lane_width_predict_sg);
  static double CalculateLanemarkerY(double distance,
                                     const LaneMarker& lane_marker);
  static bool LaneWdeTooMonitorSM(bool* state, bool lane_wide_variance_terrible,
                                  bool lane_wide_terrible);
  static bool LaneWdeTooNarrowMonitorSM(bool* state,
                                        bool is_prelne_wde_variance_terrible,
                                        bool is_lane_reliable,
                                        bool is_prelne_wde_normal);

  bool LaneWidthCoffMonitor(bool* state, double* time, bool is_coff_terrible,
                            double lane_coast_time,
                            bool is_lane_wde_terrible) const {
    bool is_lane_coff_monitor_valid = false;
    if (*state) {
      if (is_coff_terrible && is_lane_wde_terrible) {
        *state = false;
        *time = 0.0;
        is_lane_coff_monitor_valid = true;
      } else {
        *time = 0.0;
        is_lane_coff_monitor_valid = false;
      }
    } else {
      if (*time > lane_coast_time || !is_lane_wde_terrible) {
        *time = 0.0;
        *state = true;
        is_lane_coff_monitor_valid = false;
      } else {
        *time += main_loop_time_;
        is_lane_coff_monitor_valid = true;
      }
    }
    return is_lane_coff_monitor_valid;
  }

  bool LaneWidthqualityMonitor(bool* state, double* time, bool is_coff_terrible,
                               double coast_time, double quality,
                               bool is_lane_change) const {
    bool is_lane_quality_monitor_valid = false;
    if (*state) {
      if (is_coff_terrible && !is_lane_change) {
        *state = false;
        *time = 0.0;
        is_lane_quality_monitor_valid = true;
      } else {
        *time = 0.0;
        is_lane_quality_monitor_valid = false;
      }
    } else {
      if (*time > coast_time || quality < good_quality_threshold_ ||
          quality > batter_quality_threshold_ || is_lane_change) {
        *time = 0.0;
        *state = true;
        is_lane_quality_monitor_valid = false;
      } else {
        *time += main_loop_time_;
        is_lane_quality_monitor_valid = true;
      }
    }
    return is_lane_quality_monitor_valid;
  }

  // config
  planning::PerceptionMapConfig config_;
  double good_quality_threshold_{kInitGoodQualityThrd};
  double batter_quality_threshold_{kInitBatterQualityThrd};
  double lane_a0_coff_var_thd_{0.16};
  double lane_a1_coff_var_thd_{0.0035};
  double lane_a2_coff_var_thd_{0.0000007};
  double main_loop_time_{kInitMinLoopTime};
  double lane_width_var_thd_{0.95};
  double lane_width_max_thd_{5.0};
  double lane_width_min_thd_{5.0};
  double lane_width_delta_distance_{kMinLaneDis};
  double lane_width_rise_time_{kInitMinLoopTime};
  double lane_width_fall_time_{0.0};
  // during init set parameter
  double lane_coast_time_{kInitMinLoopTime};
  double max_predict_width_distance_{20.0};

  // local
  Delay<double> origin_lanemarker_a0_delay_{kMagicNumber5};
  Delay<double> origin_lanemarker_a1_delay_{kMagicNumber5};
  Delay<double> origin_lanemarker_a2_delay_{kMagicNumber5};
  Delay<double> origin_lanemarker_quality_{kMagicNumber5};
  Delay<double> side_lanemarker_quality_{kMagicNumber5};
  Delay<double> lane_wide_delta_delay_{kMagicNumber5};

  DebounceModule debounce_module_;
  bool lane_wde_too_monitor_sm_state_ = true;
  bool lane_wde_too_narrow_monitor_sm_state_ = true;
  bool lane_wde_coff_monitor_state_ = true;
  double coff_monitor_time_ = 0.0;
  bool lane_wde_quality_monitor_state_ = true;
  double quality_monitor_time_ = 0.0;
};
}  // namespace lanelineprocess
}  // namespace planning
}  // namespace TL
