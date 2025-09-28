/*
 * @Author: 80040285 zhangyu@TLauto.com
 * @Date: 2023-08-31 13:56:57
 * @LastEditors: 80040285 zhangyu@TLauto.com
 * @LastEditTime: 2023-08-31 15:49:48
 * @FilePath: /europa/planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/lane_detm_reset.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*
 * Copyright (c) TL auto Co., Ltd. 2021-2022. All rights reserved.
 */
#pragma once
#include <cassert>
#include <iostream>
#include <string>
#include <vector>

#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/decider_data.h"
#include "planning/proto/navigation_hdmap_config.pb.h"
#include "proto/planning/lanemarkers_lane_line.pb.h"

namespace TL {
namespace planning {
namespace lanelineprocess {
/* deired path definition
enum DesiredPath {
        DP_noPath,
        DP_vehFolPath,
        DP_fusionPath,
        DP_currentLaneCenter,
        DP_leftLaneCenter,
        DP_rightLaneCenter,
};
*/

// lane line status definition
// enum LineStatus {
//   LS_update,
//   LS_rstLowQlty,
//   LS_laneChange,
// };

// line confidence status definition
enum LineConfidence {
  LC_notReliabel,
  LC_lowQuality,
  LC_highQuality,
};

// Lane Determine Reset module
class LaneDetmReset {
 public:
  LaneDetmReset();
  explicit LaneDetmReset(const planning::PerceptionMapConfig& config);
  double FusionLineCnfd();  // fusioned lane line confidence
  bool IsResetFusionLine() const;
  bool IsFusionLineCoast() const;
  LaneReset GetLaneResetOut();

  int GetLaneRetState() { return lineState_; }

  double GetLaneResetTime() const { return localTimer_; }

  bool GetQualityTrig() const { return loQltyTrig_; }

  double FusionLineRangeEnd() const;
  void Update(double egoVehSpdKph, bool isLaneChange, double originLinequality,
              double sideLinequality, double originLineRangeEnd,
              double quality_coast_time,
              TL::planning::LaneReset* lane_reset_info);
  ~LaneDetmReset() = default;

 private:
  // parameters
  double good_quality_threshold_{kInitGoodQualityThrd};
  double batter_quality_threshold_{kInitBatterQualityThrd};
  double mainLoopTm_{kInitMinLoopTime};
  double lneFsnLneChgRstTm_ = 0.15;
  double lneFsnHiQltyRstTm_ = 0.2;

  // out
  LineConfidence fusionLineCnfd_ = LC_notReliabel;
  bool isResetFusionLine_{false};
  bool isFusionLineCoast_{false};
  double fusionLineRangeEnd_{0.0};

  // history vars
  double fusionLineRangeEndLast_{0.0};

  // local
  TL::planning::Resetstate lineState_ = Resetstate::LS_update;
  double localTimer_{0.0};
  double loQltyCoastRestTm_{0.0};
  bool loQltyTrig_{false};
  double lineLoQltyCoastTm_lkup_{0.0};
  // state transition
  void DoEntryInUpdate_();
  void DoEntryInLneChg_();
  void DoEntryInRstLowQlty_();
  void DoDuringUpdate_(LineConfidence originLineCnfd,
                       LineConfidence sideLineCnfd);
  void DoDuringLneChg_(bool isLaneChange, LineConfidence originLineCnfd);
  void DoDuringRstLowQlty_(LineConfidence originLineCnfd);
};
}  // namespace lanelineprocess
}  // namespace planning
}  // namespace TL
