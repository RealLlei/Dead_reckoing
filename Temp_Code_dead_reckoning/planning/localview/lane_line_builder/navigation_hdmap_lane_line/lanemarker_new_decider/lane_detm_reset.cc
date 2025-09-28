/*
 * Copyright (c) TL auto Co., Ltd. 2021-2022. All rights reserved.
 */
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/lane_detm_reset.h"

namespace TL {
namespace planning {
namespace lanelineprocess {
LaneDetmReset::LaneDetmReset() {
  // do default state UPDATE
  DoEntryInUpdate_();
}

LaneDetmReset::LaneDetmReset(const planning::PerceptionMapConfig& config)
    : good_quality_threshold_{config.lanemarker_quality_not_reliable_value()},
      batter_quality_threshold_{
          config.lanemarker_quality_reliableforwarning_value()},
      mainLoopTm_{config.main_loop_time()},
      lneFsnLneChgRstTm_{
          config.reset_quality_config().lane_fsn_lanechange_reset_time()},
      lneFsnHiQltyRstTm_{
          config.reset_quality_config().lane_fsn_hight_quality_reset_time()} {
  // do default state UPDATE
  DoEntryInUpdate_();
}

// return fusion line is reset or not
bool LaneDetmReset::IsResetFusionLine() const {
  return isResetFusionLine_;
}

// return fusioned line confidence
double LaneDetmReset::FusionLineCnfd() {
  double quality{0.0};
  if (fusionLineCnfd_ == LC_highQuality) {
    quality = kInitGoodQualityThrd * 3;
  } else if (fusionLineCnfd_ == LC_notReliabel) {
    quality = 0.0;
  } else {
    quality = (kInitGoodQualityThrd + kInitBatterQualityThrd) / 2;
  }
  return quality;
}

// return if fusion line coast or not
bool LaneDetmReset::IsFusionLineCoast() const {
  return isFusionLineCoast_;
}

// return fusioned line range end distance
double LaneDetmReset::FusionLineRangeEnd() const {
  return fusionLineRangeEnd_;
}

LaneReset LaneDetmReset::GetLaneResetOut() {
  LaneReset lane_reset;
  lane_reset.is_lane_coast = isFusionLineCoast_;
  lane_reset.is_lane_reset = isResetFusionLine_;
  lane_reset.lane_length = fusionLineRangeEnd_;
  double quality{0.0};
  if (fusionLineCnfd_ == LC_highQuality) {
    quality = kInitGoodQualityThrd * 3;
  } else if (fusionLineCnfd_ == LC_notReliabel) {
    quality = 0.0;
  } else {
    quality = (kInitGoodQualityThrd + kInitBatterQualityThrd) / 2;
  }
  lane_reset.lane_quality = quality;
  return lane_reset;
}

void LaneDetmReset::DoEntryInUpdate_() {
  // entry exec
  lineState_ = Resetstate::LS_update;
  localTimer_ = 0.0;
  loQltyCoastRestTm_ = lineLoQltyCoastTm_lkup_;
  loQltyTrig_ = false;
  isResetFusionLine_ = false;
  fusionLineCnfd_ = LC_highQuality;  // controls allowed
}

void LaneDetmReset::DoEntryInLneChg_() {
  // entry exec
  lineState_ = Resetstate::LS_laneChange;
  localTimer_ = 0.0;
  loQltyCoastRestTm_ = lineLoQltyCoastTm_lkup_;
  loQltyTrig_ = false;
  isResetFusionLine_ = true;
  fusionLineCnfd_ = LC_highQuality;  // controls allowed
}

void LaneDetmReset::DoEntryInRstLowQlty_() {
  // entry exec
  lineState_ = Resetstate::LS_rstLowQlty;
  localTimer_ = 0.0;
  loQltyCoastRestTm_ = lineLoQltyCoastTm_lkup_;
  loQltyTrig_ = false;
  isResetFusionLine_ = true;
  fusionLineCnfd_ = LC_notReliabel;  // controls allowed
}

// 如果车道线质量有问题超过一定时间就转到quality reset
void LaneDetmReset::DoDuringUpdate_(LineConfidence originLineCnfd,
                                    LineConfidence sideLineCnfd) {
  // douring in UPDATE
  if (originLineCnfd != LC_highQuality) {
    if (sideLineCnfd != LC_highQuality) {
      localTimer_ += 3 * mainLoopTm_;
    } else {
      localTimer_ += mainLoopTm_;
    }
  } else {
    localTimer_ = 0;
  }

  loQltyCoastRestTm_ = lineLoQltyCoastTm_lkup_ - localTimer_;

  // judge LoQltyTrig
  loQltyTrig_ = localTimer_ > lineLoQltyCoastTm_lkup_;
}

void LaneDetmReset::DoDuringLneChg_(bool isLaneChange,
                                    LineConfidence originLineCnfd) {
  if (!isLaneChange) {
    if (originLineCnfd == LC_notReliabel) {
      loQltyTrig_ = true;
      localTimer_ = 0.0;
    } else {
      loQltyTrig_ = false;
      localTimer_ += mainLoopTm_;
    }

  } else {
    loQltyTrig_ = false;
    localTimer_ = 0.0;
  }
}

void LaneDetmReset::DoDuringRstLowQlty_(LineConfidence originLineCnfd) {
  if (originLineCnfd == LC_highQuality) {
    localTimer_ += mainLoopTm_;
  } else {
    localTimer_ = 0.0;
  }
}

void LaneDetmReset::Update(const double egoVehSpdKph, const bool isLaneChange,
                           const double originLinequality,
                           const double sideLinequality,
                           const double originLineRangeEnd,
                           const double quality_coast_time,
                           TL::planning::LaneReset* const lane_reset_info) {
  LineConfidence originLineCnfd{LC_notReliabel};
  LineConfidence sideLineCnfd{LC_notReliabel};
  if (originLinequality < good_quality_threshold_) {
    originLineCnfd = LC_notReliabel;
  } else if (originLinequality > batter_quality_threshold_) {
    originLineCnfd = LC_highQuality;
  } else {
    originLineCnfd = LC_lowQuality;
  }

  if (sideLinequality < good_quality_threshold_) {
    sideLineCnfd = LC_notReliabel;
  } else if (sideLinequality > batter_quality_threshold_) {
    sideLineCnfd = LC_highQuality;
  } else {
    sideLineCnfd = LC_lowQuality;
  }

  lineLoQltyCoastTm_lkup_ = quality_coast_time;
  ADEBUG << "before reset state: " << lineState_
         << ", coast time: " << lineLoQltyCoastTm_lkup_
         << ", is_change: " << isLaneChange;
  lane_reset_info->set_last_state(lineState_);
  lane_reset_info->set_is_lane_change(isLaneChange);
  lane_reset_info->set_original_quality(originLinequality);
  lane_reset_info->set_side_quality(sideLinequality);
  lane_reset_info->set_quality_coast_time(lneFsnHiQltyRstTm_);
  lane_reset_info->set_lane_change_coast_time(lneFsnLneChgRstTm_);
  // state transition
  if (lineState_ == LS_update) {
    // transiton to RESET_LneChg cond
    if (isLaneChange) {
      DoEntryInLneChg_();
    } else if (loQltyTrig_) {
      DoEntryInRstLowQlty_();
    } else {
      DoDuringUpdate_(originLineCnfd, sideLineCnfd);
    }

  } else if (lineState_ == LS_laneChange) {  // state in lane change reset
    if (loQltyTrig_) {
      DoEntryInRstLowQlty_();
    } else if (localTimer_ >= lneFsnLneChgRstTm_) {
      DoEntryInUpdate_();
    } else {
      DoDuringLneChg_(isLaneChange, originLineCnfd);
    }

  } else {
    // transition to UPDATE
    if (localTimer_ > lneFsnHiQltyRstTm_) {
      DoEntryInUpdate_();
    } else {
      DoDuringRstLowQlty_(originLineCnfd);
    }
  }
  ADEBUG << "reset state: " << lineState_ << ", time: " << localTimer_
         << ", quality_max_time: " << lneFsnHiQltyRstTm_
         << ", lane change max time: " << lneFsnLneChgRstTm_;
  // 如果当前车道线的质量很差且处于update状态，那么range_end应该是上一时刻
  // 的range_end减去车速乘以周期，正常情况下range_end不变。
  // caculate line coast flag and line range end
  isFusionLineCoast_ = fusionLineCnfd_ != originLineCnfd;

  // caculate fusioned lane line range end.
  if (originLineCnfd == LC_notReliabel && isFusionLineCoast_) {
    fusionLineRangeEnd_ =
        fusionLineRangeEndLast_ - mainLoopTm_ * egoVehSpdKph / kKph2ms;
    if (fusionLineRangeEnd_ < 0.0) {
      fusionLineRangeEnd_ = 0.0;
    }

  } else {
    fusionLineRangeEnd_ = originLineRangeEnd;
  }
  // update delay status
  fusionLineRangeEndLast_ = fusionLineRangeEnd_;
  lane_reset_info->set_is_lane_reset(isResetFusionLine_);
  lane_reset_info->set_is_fusion_linecoast(isFusionLineCoast_);
  lane_reset_info->set_fusion_view_range(fusionLineRangeEnd_);
  lane_reset_info->set_is_quality_trig(loQltyTrig_);
  lane_reset_info->set_now_state(lineState_);
  lane_reset_info->set_time(localTimer_);
}
}  // namespace lanelineprocess
}  // namespace planning
}  // namespace TL
