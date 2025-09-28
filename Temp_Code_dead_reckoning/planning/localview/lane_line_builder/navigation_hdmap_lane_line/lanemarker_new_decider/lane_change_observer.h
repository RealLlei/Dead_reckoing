/*
 * @Author: 80040285 zhangyu@TLauto.com
 * @Date: 2023-08-31 13:56:57
 * @LastEditors: 80040285 zhangyu@TLauto.com
 * @LastEditTime: 2023-08-31 15:02:28
 * @FilePath: /europa/planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/lane_change_observer.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*
 * Copyright (c) TL auto Co., Ltd. 2021-2022. All rights reserved.
 */
#pragma once
#include <tuple>
#include <utility>

#include "common/status/status.h"
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/lane_line_delay.h"
#include "planning/proto/navigation_hdmap_config.pb.h"
#include "proto/perception/perception_obstacle.pb.h"

namespace TL {
namespace planning {
namespace lanelineprocess {
enum ChangeLaneState { None, Start, In, End };

class LaneChangeObserver {
 public:
  LaneChangeObserver() = default;
  ~LaneChangeObserver() = default;
  TL::common::Status Init();
  std::pair<bool, bool> Observer(
      const TL::perception::LaneMarkers& lane_markers);
  void Reset();

 private:
  Delay<std::tuple<double, double, double, double>> change_lane_delay_{5};
  ChangeLaneState change_lane_state_ = ChangeLaneState::None;
  double change_laneline_jump_width_ = 1.5;
  double better_quality_threshold_ = 0.6;
  double good_quality_threshold_ = 0.3;
  bool is_left_change_ = false;
  bool is_right_change_ = false;
  int change_count_ = 0;
};

}  // namespace lanelineprocess
}  // namespace planning
}  // namespace TL
