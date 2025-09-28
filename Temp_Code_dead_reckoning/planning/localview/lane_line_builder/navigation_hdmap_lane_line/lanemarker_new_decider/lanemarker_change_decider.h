/*
 * @Author: 80040285 zhangyu@TLauto.com
 * @Date: 2023-08-31 13:56:57
 * @LastEditors: 80040285 zhangyu@TLauto.com
 * @LastEditTime: 2023-08-31 15:52:04
 * @FilePath: /europa/planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/lanemarker_change_decider.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*
 * Copyright (c) TL auto Co., Ltd. 2021-2022. All rights reserved.
 */
#pragma once
#include <memory>
#include <tuple>
#include <utility>
#include <vector>

#include "common/filters/mean_filter.h"
#include "common/interpolation/interpolation_1d.h"
#include "common/math/vec2d.h"
#include "common/status/status.h"
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/decider_data.h"
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/lane_line_delay.h"
#include "planning/proto/navigation_hdmap_config.pb.h"

namespace TL {
namespace planning {
namespace lanelineprocess {
using TL::common::Interpolation1D;
using TL::common::Status;

class LaneChangeDecider {
 public:
  LaneChangeDecider() = default;
  explicit LaneChangeDecider(const planning::PerceptionMapConfig& config);
  ~LaneChangeDecider() = default;
  Status Init();
  void LoadLaneChangeGainScheduler(
      const planning::PerceptionMapLaneChangeDeciderConfig& lane_change_conf);
  void ComputerCoastTime(DeciderData* decider_data);
  void GetLeftChangeCoff(DeciderData* decider_data);
  void GetLeftChangeFlag(DeciderData* decider_data);
  void GetRightChangeCoff(DeciderData* decider_data);
  void GetRightChangeFlag(DeciderData* decider_data);

 private:
  bool LaneChangeCoffMonitorSM(std::tuple<bool, double, int>* inside_valus,
                               bool is_coff_terrible, bool lane_wide_terrible,
                               bool lanechange_flag, double lane_quality,
                               double lane_coast_time);

  void LaneWidePredictSM(std::pair<bool, double>* inside_value,
                         bool lane_change_flag,
                         int* next_lane_width_predict_flag,
                         int next_lane_width_predict_last_flag,
                         double* next_lane_predict_width,
                         double next_lane_predict_width_last) const;
  void CreatInitLaneMarkerPoints(const LaneMarker& input_lanemarker,
                                 double pos_a0,
                                 std::vector<Vec2d>* init_points);
  DeciderData decider_data_;
  planning::PerceptionMapConfig config_;
  double good_quality_threshold_ = 0.3;
  double main_loop_time_ = 0.1;
  double next_lane_width_hldtm_ = 3.0;
  double default_lane_width_ = 3.5;
  std::unique_ptr<Interpolation1D> lane_change_interpolation_{nullptr};
  // left lane change flag
  Delay<double> next_left_lane_width_predict_sg_delay_;
  Delay<int> next_left_lane_width_predict_valid_delay_;
  Delay<bool> host_lane_change_to_left_delay_;
  double last_next_left_lanewdepre_sg_{0.0};
  int last_next_left_lanewdepre_valid_{0};
  // 1:state 2:time
  std::pair<bool, double> left_lanewde_inside_{};
  Delay<double> left_a0_delay_;
  // first:state, second:timer, thr:terrible count
  std::tuple<bool, double, int> left_coff_monitor_inside_{};
  double predict_host_left_pos_a0_{0.0};
  Delay<LaneMarker> left_lanemarker_delay_{3};

  // right lane change flag
  Delay<double> next_right_lane_width_predict_sg_delay_;
  Delay<int> next_right_lane_width_predict_valid_delay_;
  Delay<bool> host_lane_change_to_right_delay_;
  double last_next_right_lanewdepre_sg_{0.0};
  int last_next_right_lanewdepre_valid_{0};
  // 1:state 2:time
  std::pair<bool, double> right_lanewde_inside_{};
  Delay<double> right_a0_delay_;
  // first:state, second:timer, thr:terrible count
  std::tuple<bool, double, int> right_coff_monitor_inside_{};
  double predict_host_right_pos_a0_{0.0};
  Delay<LaneMarker> right_lanemarker_delay_{3};

  common::MeanFilter A0_mean_filter_;
};
}  // namespace lanelineprocess
}  // namespace planning
}  // namespace TL
