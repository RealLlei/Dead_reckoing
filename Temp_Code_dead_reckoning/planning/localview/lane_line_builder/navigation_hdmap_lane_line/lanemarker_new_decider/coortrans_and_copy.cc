/*
 * Copyright (c) TL auto Co., Ltd. 2021-2022. All rights reserved.
 */
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/coortrans_and_copy.h"

#include <cmath>
#include <tuple>
#include <utility>

#include "common/time/clock.h"
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/decider_data.h"
#include "proto/perception/perception_obstacle.pb.h"

namespace TL {
namespace planning {
namespace lanelineprocess {
CoorTransAndCopy::CoorTransAndCopy(const planning::PerceptionMapConfig& config)
    : config_(config) {
  ego_lanewidth_prediction_last_ = Delay<double>(3);
  left_lanewidth_prediction_last_ = Delay<double>(3);
  right_lanewidth_prediction_last_ = Delay<double>(3);
}

Status CoorTransAndCopy::Init() {
  lanemarker_back_length_ = config_.lanemarker_back_length();
  max_allow_copy_lane_time_ =
      config_.coortrans_copy_config().max_allow_copy_lane_time();
  main_loop_time_ = config_.main_loop_time();
  good_quality_threshold_ = config_.lanemarker_quality_not_reliable_value();
  batter_quality_threshold_ =
      config_.lanemarker_quality_reliableforwarning_value();
  return Status::OK();
}

bool CoorTransAndCopy::LaneMarkerCoorTrans(DeciderData* decider_data) {
  //
  ADEBUG << "LaneMarkerCoorTrans Process!"
         << decider_data->original_lanemarkers.header().seq();
  const auto& ori_lanemarkers = decider_data->original_lanemarkers;
  const auto& left_lanemarker = ori_lanemarkers.front_left_lane_marker();
  const auto& right_lanemarker = ori_lanemarkers.front_right_lane_marker();
  const auto& next_left_lanemarker =
      ori_lanemarkers.front_next_left_lane_marker().at(0);
  const auto& next_right_lanemarker =
      ori_lanemarkers.front_next_right_lane_marker().at(0);

  auto& tra_lanemarkers = decider_data->trans_lanemarkers;
  tra_lanemarkers.mutable_header()->CopyFrom(ori_lanemarkers.header());
  tra_lanemarkers.set_is_lanechange_to_left(
      ori_lanemarkers.is_lanechange_to_left());
  tra_lanemarkers.set_is_lanechange_to_right(
      ori_lanemarkers.is_lanechange_to_right());
  auto* trans_left_lanemarker =
      tra_lanemarkers.mutable_front_left_lane_marker();
  auto* trans_right_lanemarker =
      tra_lanemarkers.mutable_front_right_lane_marker();
  auto* trans_next_left_lanemarker =
      tra_lanemarkers.add_front_next_left_lane_marker();
  auto* trans_next_right_lanemarker =
      tra_lanemarkers.add_front_next_right_lane_marker();

  LaneCoordinateTransformation(left_lanemarker, trans_left_lanemarker);
  LaneCoordinateTransformation(right_lanemarker, trans_right_lanemarker);
  LaneCoordinateTransformation(next_left_lanemarker,
                               trans_next_left_lanemarker);
  LaneCoordinateTransformation(next_right_lanemarker,
                               trans_next_right_lanemarker);

  decider_data->road_average_curvature =
      (RoadCurvatureCalculate(left_lanemarker, 0) +
       RoadCurvatureCalculate(right_lanemarker, 0)) /
      2;
  ADEBUG << "After LaneMarkerCoorTrans! curve: "
         << decider_data->road_average_curvature;
  return true;
}

void CoorTransAndCopy::EgoLaneMarkerCopy(DeciderData* decider_data) {
  ADEBUG << "EgoLaneMarkerCopy Process!"
         << decider_data->original_lanemarkers.header().seq();
  const auto left_lanemarker =
      decider_data->trans_lanemarkers.front_left_lane_marker();
  const auto right_lanemarker =
      decider_data->trans_lanemarkers.front_right_lane_marker();
  auto* copy_left_lane_marker =
      decider_data->copy_lanemarkers.mutable_front_left_lane_marker();
  auto* copy_right_lane_marker =
      decider_data->copy_lanemarkers.mutable_front_right_lane_marker();
  decider_data->copy_lanemarkers.mutable_header()->CopyFrom(
      decider_data->trans_lanemarkers.header());
  const auto ego_lanewidth_prediction_valid =
      decider_data->lane_width_valid_out.ego_lane_widthpredict_valid;
  const auto ego_lanewidth_prediction =
      decider_data->lane_width_valid_out.ego_lane_widthpredict_sg;
  if (ego_lanewidth_prediction_valid > 0) {
    ego_lanewidth_prediction_last_.Deal(ego_lanewidth_prediction);
  }

  LanemarkerTuple left_bridge_copy_tuple =
      std::make_tuple(left_lanemarker, history_left_lanemarker_, &left_timer_,
                      &left_copy_state_, 1, ego_lanewidth_prediction_last_);
  LanemarkerTuple right_bridge_copy_tuple = std::make_tuple(
      right_lanemarker, history_right_lanemarker_, &right_timer_,
      &right_copy_state_, -1, ego_lanewidth_prediction_last_);
  auto left_copy_flag = std::make_pair(false, false);
  auto right_copy_flag = std::make_pair(false, false);
  auto left_lane_marker_state =
      std::make_tuple(decider_data->lane_markers_state.left_lanemarker_state,
                      history_lanemarkers_state_.left_lanemarker_state,
                      decider_data->lane_markers_state.right_lanemarker_state,
                      history_lanemarkers_state_.right_lanemarker_state);
  auto right_lane_marker_state =
      std::make_tuple(decider_data->lane_markers_state.right_lanemarker_state,
                      history_lanemarkers_state_.right_lanemarker_state,
                      decider_data->lane_markers_state.left_lanemarker_state,
                      history_lanemarkers_state_.left_lanemarker_state);
  // 左侧车道线桥接
  auto better_right_lanemarker = right_lanemarker;
  bool is_right_quality_ok =
      right_good_lanemaker_quality_debounce_.DealDebounce(
          better_right_lanemarker.quality() > 0.5);

  better_right_lanemarker.set_quality(
      (is_right_quality_ok && better_right_lanemarker.quality() > 0.3) ? 0.7
                                                                       : 0.5);
  BridgeCopy(&left_bridge_copy_tuple, better_right_lanemarker,
             copy_left_lane_marker, ego_lanewidth_prediction_valid,
             &left_copy_flag, left_lane_marker_state);
  // 右侧车道线桥接
  auto better_left_lanemarker = left_lanemarker;
  bool is_left_quality_ok = left_good_lanemaker_quality_debounce_.DealDebounce(
      better_left_lanemarker.quality() > 0.5);

  better_left_lanemarker.set_quality(
      (is_left_quality_ok && better_left_lanemarker.quality() > 0.3) ? 0.7
                                                                     : 0.5);
  BridgeCopy(&right_bridge_copy_tuple, better_left_lanemarker,
             copy_right_lane_marker, ego_lanewidth_prediction_valid,
             &right_copy_flag, right_lane_marker_state);
  //   if (std::get<1>(left_copy_flag)) {
  //     for (double start_s = 0.0;
  //          start_s < std::fmin(right_lanemarker.view_range(),
  //                              copy_left_lane_marker->view_range());
  //          start_s += 5.0) {
  //       double lane_width =
  //           CalculateLanemarkerY(start_s, *copy_left_lane_marker) -
  //           CalculateLanemarkerY(start_s, right_lanemarker);
  //       AERROR << "lane_width[" << start_s << "]: " << lane_width;
  //     }
  //   }
  ADEBUG << " left_is_copy: " << std::get<1>(left_copy_flag)
         << " , right_is_copy: " << std::get<1>(right_copy_flag);
  // 添加debug输出信息
  auto* lanemarker_copy_debug =
      decider_data->lanemarker_lanline_debug->mutable_lanemarker_decider_debug()
          ->mutable_lanemarkers_copy_info();
  lanemarker_copy_debug->set_left_is_copy(std::get<1>(left_copy_flag));
  lanemarker_copy_debug->set_right_is_copy(std::get<1>(right_copy_flag));
  auto* left_lane_marker_copy_debug =
      lanemarker_copy_debug->add_lanemarker_copy_info();
  left_lane_marker_copy_debug->set_name("left_lanemarker_copy");
  left_lane_marker_copy_debug->mutable_state_info()->set_state_name(
      "left_lanemarker_copy_state");
  left_lane_marker_copy_debug->mutable_state_info()->set_now_state(
      left_copy_state_ ? 1 : 0);
  left_lane_marker_copy_debug->mutable_state_info()->set_time(left_timer_);
  left_lane_marker_copy_debug->set_is_trigger_copy(std::get<0>(left_copy_flag));
  left_lane_marker_copy_debug->set_do_is_bridge_copy(
      std::get<1>(left_copy_flag));
  left_lane_marker_copy_debug->set_lanewidth_prediction_last(
      ego_lanewidth_prediction_last_.GetDelay(2));
  auto* right_lane_marker_copy_debug =
      lanemarker_copy_debug->add_lanemarker_copy_info();
  right_lane_marker_copy_debug->set_name("right_lanemarker_copy");
  right_lane_marker_copy_debug->mutable_state_info()->set_state_name(
      "right_lanemarker_copy_state");
  right_lane_marker_copy_debug->mutable_state_info()->set_now_state(
      right_copy_state_ ? 1 : 0);
  right_lane_marker_copy_debug->mutable_state_info()->set_time(right_timer_);
  right_lane_marker_copy_debug->set_is_trigger_copy(
      std::get<0>(right_copy_flag));
  right_lane_marker_copy_debug->set_do_is_bridge_copy(
      std::get<1>(right_copy_flag));
  right_lane_marker_copy_debug->set_lanewidth_prediction_last(
      ego_lanewidth_prediction_last_.GetDelay(2));

  history_left_lanemarker_ = left_lanemarker;
  history_right_lanemarker_ = right_lanemarker;
  history_lanemarkers_state_ = decider_data->lane_markers_state;
}

void CoorTransAndCopy::NextLaneMarkerCopy(DeciderData* decider_data) {
  ADEBUG << "good_quality_threhold:" << good_quality_threshold_
         << ", batter_quality_threshold: " << batter_quality_threshold_;
  const auto left_lanemarker =
      decider_data->trans_lanemarkers.front_left_lane_marker();
  const auto right_lanemarker =
      decider_data->trans_lanemarkers.front_right_lane_marker();
  const auto next_left_lanemarker =
      decider_data->trans_lanemarkers.front_next_left_lane_marker().at(0);
  const auto next_right_lanemarker =
      decider_data->trans_lanemarkers.front_next_right_lane_marker().at(0);
  auto* copy_next_left_lane_marker =
      decider_data->copy_lanemarkers.add_front_next_left_lane_marker();
  auto* copy_next_right_lane_marker =
      decider_data->copy_lanemarkers.add_front_next_right_lane_marker();
  const auto left_lanewidth_prediction_valid =
      decider_data->lane_width_valid_out.next_leftlane_widthpredict_valid;
  const auto left_lanewidth_prediction =
      decider_data->lane_width_valid_out.next_leftlane_widthpredict_sg;
  const auto right_lanewidth_prediction_valid =
      decider_data->lane_width_valid_out.next_rightlane_widthpredict_valid;
  const auto right_lanewidth_prediction =
      decider_data->lane_width_valid_out.next_rightlane_widthpredict_sg;
  if (left_lanewidth_prediction_valid > 0) {
    left_lanewidth_prediction_last_.Deal(left_lanewidth_prediction);
  }
  if (right_lanewidth_prediction_valid > 0) {
    right_lanewidth_prediction_last_.Deal(right_lanewidth_prediction);
  }

  LanemarkerTuple nextleft_bridge_copy_tuple = std::make_tuple(
      next_left_lanemarker, history_next_left_lanemarker_, &next_left_timer_,
      &next_left_copy_state_, 2, left_lanewidth_prediction_last_);
  LanemarkerTuple nextright_bridge_copy_tuple = std::make_tuple(
      next_right_lanemarker, history_next_right_lanemarker_, &next_right_timer_,
      &next_right_copy_state_, -2, right_lanewidth_prediction_last_);
  auto left_copy_flag = std::make_pair(false, false);
  auto right_copy_flag = std::make_pair(false, false);
  auto left_lane_marker_state = std::make_tuple(
      decider_data->lane_markers_state.next_left_lanemarker_state,
      history_lanemarkers_state_.next_left_lanemarker_state,
      decider_data->lane_markers_state.left_lanemarker_state,
      history_lanemarkers_state_.left_lanemarker_state);
  auto right_lane_marker_state = std::make_tuple(
      decider_data->lane_markers_state.next_right_lanemarker_state,
      history_lanemarkers_state_.next_right_lanemarker_state,
      decider_data->lane_markers_state.right_lanemarker_state,
      history_lanemarkers_state_.right_lanemarker_state);
  BridgeCopy(&nextleft_bridge_copy_tuple, left_lanemarker,
             copy_next_left_lane_marker, left_lanewidth_prediction_valid,
             &left_copy_flag, left_lane_marker_state);
  BridgeCopy(&nextright_bridge_copy_tuple, right_lanemarker,
             copy_next_right_lane_marker, right_lanewidth_prediction_valid,
             &right_copy_flag, right_lane_marker_state);

  // 添加debug输出信息
  auto* lanemarker_copy_debug =
      decider_data->lanemarker_lanline_debug->mutable_lanemarker_decider_debug()
          ->mutable_lanemarkers_copy_info();
  lanemarker_copy_debug->set_next_left_is_copy(std::get<1>(left_copy_flag));
  lanemarker_copy_debug->set_next_right_is_copy(std::get<1>(right_copy_flag));
  auto* left_lane_marker_copy_debug =
      lanemarker_copy_debug->add_lanemarker_copy_info();
  left_lane_marker_copy_debug->set_name("next_left_lanemarker_copy");
  left_lane_marker_copy_debug->mutable_state_info()->set_state_name(
      "next_left_copy_state");
  left_lane_marker_copy_debug->mutable_state_info()->set_now_state(
      next_left_copy_state_ ? 1 : 0);
  left_lane_marker_copy_debug->mutable_state_info()->set_time(next_left_timer_);
  left_lane_marker_copy_debug->set_is_trigger_copy(std::get<0>(left_copy_flag));
  left_lane_marker_copy_debug->set_do_is_bridge_copy(
      std::get<1>(left_copy_flag));
  left_lane_marker_copy_debug->set_lanewidth_prediction_last(
      left_lanewidth_prediction_last_.GetDelay(2));
  auto* right_lane_marker_copy_debug =
      lanemarker_copy_debug->add_lanemarker_copy_info();
  right_lane_marker_copy_debug->set_name("next_right_lanemarker_copy");
  right_lane_marker_copy_debug->mutable_state_info()->set_state_name(
      "next_right_copy_state");
  right_lane_marker_copy_debug->mutable_state_info()->set_now_state(
      next_right_copy_state_ ? 1 : 0);
  right_lane_marker_copy_debug->mutable_state_info()->set_time(
      next_right_timer_);
  right_lane_marker_copy_debug->set_is_trigger_copy(
      std::get<0>(right_copy_flag));
  right_lane_marker_copy_debug->set_do_is_bridge_copy(
      std::get<1>(right_copy_flag));
  right_lane_marker_copy_debug->set_lanewidth_prediction_last(
      right_lanewidth_prediction_last_.GetDelay(2));

  history_next_left_lanemarker_ = next_left_lanemarker;
  history_next_right_lanemarker_ = next_right_lanemarker;
}

void CoorTransAndCopy::TimerScheduler(DeciderData* decider_data) {
  ADEBUG << "Set do ego/next lane update!"
         << decider_data->original_lanemarkers.header().seq();
  decider_data->do_egolane_update = false;
  decider_data->do_nextlane_update = false;
  bool hostlane_is_change2left = decider_data->is_lanechange_to_left;
  bool hostlane_is_change2right = decider_data->is_lanechange_to_right;
  double time1 = Clock::NowInSeconds();
  if (hostlane_is_change2left || hostlane_is_change2right) {
    // lane process loop time 50ms
    if (Clock::NowInSeconds() - time1 > kInitMinLoopTime / 2) {
      decider_data->do_egolane_update = true;
      decider_data->do_nextlane_update = true;
      std::cout << "true time:" << Clock::NowInSeconds() << std::endl;
    }
  } else {
    // should be changed smaller for NNP planning which is 100ms
    if (Clock::NowInSeconds() - time1 > kInitMinLoopTime) {
      decider_data->do_egolane_update = true;
      decider_data->do_nextlane_update = true;
      std::cout << "false time:" << Clock::NowInSeconds() << std::endl;
    }
  }
  decider_data->do_egolane_update = true;  // 临时赋值 0407
  decider_data->do_nextlane_update = true;
  ADEBUG << "TimerScheduler end!";
}

bool CoorTransAndCopy::LaneCoordinateTransformation(
    const LaneMarker& lanemarker_input,
    LaneMarker* const lanemarker_output) const {
  if (lanemarker_output == nullptr) {
    AERROR << "lanemarker output is nullptr!!!";
    return false;
  }
  double quality = lanemarker_input.quality();
  double c0 = (camera_position_offset_ + lanemarker_input.c0_position()) *
              (((quality < good_quality_threshold_) &&
                (fabs(lanemarker_input.c0_position()) < kMinC0))
                   ? 0
                   : 1);
  lanemarker_output->CopyFrom(lanemarker_input);
  lanemarker_output->set_c0_position(c0);
  lanemarker_output->set_view_range(lanemarker_input.longitude_end());
  return true;
}

double CoorTransAndCopy::RoadCurvatureCalculate(const LaneMarker& lane_marker,
                                                const double length) {
  double y_d = lane_marker.c1_heading_angle() +
               2 * lane_marker.c2_curvature() * length +
               3 * lane_marker.c3_curvature_derivative() * length * length;
  double y_dd = 2 * lane_marker.c2_curvature() +
                2 * 3 * lane_marker.c3_curvature_derivative() * length;
  return abs(y_dd) / pow((1 + y_d * y_d), kCurvatureNum);
}

bool CoorTransAndCopy::BridgeCopy(LanemarkerTuple* bridge_copy_tuple,
                                  const LaneMarker& good_lanemarker,
                                  LaneMarker* const copy_lanemarker,
                                  const int lanewidth_prediction_valid,
                                  std::pair<bool, bool>* copy_flag,
                                  const LanemarkerState& lanemarker_state) {
  // left_bridge_copy_tuple=std::make_tuple(left_lanemarker,
  // history_left_lanemarker_, left_timer_, left_copy_state_, 1);
  // 0:LaneMarker, 1:history_LaneMarker, 2:timer, 3:state, 4:bridgecopy_type,
  // 5:history_lane_width
  // 宽度预测有效，需要copy的lanemarker质量较差，用来copy的lanemaker质量较好才会触发
  bool is_trigger_copy =
      ((lanewidth_prediction_valid == 1) || (lanewidth_prediction_valid == 2) ||
       (lanewidth_prediction_valid == 3) ||
       (lanewidth_prediction_valid == 4)) &&
      (good_lanemarker.quality() >= batter_quality_threshold_) &&
      (good_lanemarker.longitude_start() <= 8) &&
      (((std::get<0>(*bridge_copy_tuple).quality() < good_quality_threshold_) &&
        ((std::get<1>(*bridge_copy_tuple).view_range() < kMinLaneDis
              ? std::get<0>(*bridge_copy_tuple).quality()
              : std::get<1>(*bridge_copy_tuple).quality()) >=
         good_quality_threshold_)) ||
       (std::get<0>(lanemarker_state) == BAD_LANEMARKER &&
        std::get<1>(lanemarker_state) != BAD_LANEMARKER)) &&
      (std::get<0>(lanemarker_state) != BAD_LANEMARKER ||
       std::get<2>(lanemarker_state) != BAD_LANEMARKER);
  // is_trigger_copy为true的时候进行copy，但是当超过一定时间或者宽度预测无效或者需要copy
  // 的lanemerker质量变好或者用来copy的lanemerker质量变差都会停止copy。
  ADEBUG << "before copydecider,state: " << *(std::get<3>(*bridge_copy_tuple))
         << ", time: " << *(std::get<2>(*bridge_copy_tuple))
         << " , is_trigger_copy: " << is_trigger_copy
         << " , lanewidth_prediction_valid: " << lanewidth_prediction_valid
         << " , good_lanemarker.quality: " << good_lanemarker.quality()
         << " , bad_lane_quality: " << std::get<0>(*bridge_copy_tuple).quality()
         << " , lanemarker_state: " << std::get<0>(lanemarker_state);
  const bool do_is_bridge_copy = CopyDecider(
      (std::get<3>(*bridge_copy_tuple)), (std::get<2>(*bridge_copy_tuple)),
      is_trigger_copy, lanewidth_prediction_valid, good_lanemarker.quality(),
      good_lanemarker.longitude_start(),
      std::get<0>(*bridge_copy_tuple).quality(), lanemarker_state);
  // copy的时候需要注意c0的正负，根据预测的车道宽度进行求取
  /*  bad lane          good lane         bad lane
  **  ---|----lanewidth----|----lanewidth----|---
  ** c0 + lanewidth       c0          c0 - lanewidth
  */
  if (do_is_bridge_copy) {
    *copy_lanemarker = good_lanemarker;
    (*copy_lanemarker)
        .set_c0_position(
            copysign(std::get<kMagicNumber5>(*bridge_copy_tuple).GetDelay(2),
                     std::get<4>(*bridge_copy_tuple)) +
            good_lanemarker.c0_position());
    (*copy_lanemarker)
        .set_lane_type(TL::hdmap::LaneBoundaryType::SOLID_WHITE);
  } else {
    *copy_lanemarker = std::get<0>(*bridge_copy_tuple);
  }
  std::get<0>(*copy_flag) = is_trigger_copy;
  std::get<1>(*copy_flag) = do_is_bridge_copy;
  ADEBUG << " bridge_copy type[" << std::get<4>(*bridge_copy_tuple)
         << "], do_is_bridge_copy: " << do_is_bridge_copy
         << ",copy time: " << *(std::get<2>(*bridge_copy_tuple))
         << ", max_copy_time: " << max_allow_copy_lane_time_
         << ", copy_state: " << *(std::get<3>(*bridge_copy_tuple));
  return true;
}

bool CoorTransAndCopy::CopyDecider(
    bool* state, double* timer, const bool is_trigger_copy,
    const int lane_wide_predict_valid, const double good_lane_quality,
    const double good_lane_start, const double bad_lane_quality,
    const LanemarkerState& lanemarker_state) const {
  bool is_do_copy_lane = false;
  if (*state) {
    if (*timer > max_allow_copy_lane_time_ ||
        good_lane_quality < batter_quality_threshold_ ||
        good_lane_start >= 10 ||
        ((std::get<0>(lanemarker_state) == BETTER_LANEMARKER ||
          std::get<0>(lanemarker_state) == GOOD_LANEMARKER) &&
         bad_lane_quality > 0.3) ||
        lane_wide_predict_valid == 0) {
      *state = false;
      *timer = 0.0;
      is_do_copy_lane = false;
    } else {
      *timer += main_loop_time_;
      is_do_copy_lane = true;
    }
  } else {
    if (is_trigger_copy) {
      *state = true;
      *timer = 0.0;
      is_do_copy_lane = true;
    } else {
      *timer = 0.0;
      is_do_copy_lane = false;
    }
  }
  return is_do_copy_lane;
}

// double CoorTransAndCopy::CalculateLanemarkerY(const double distance,
//                                               const LaneMarker& lane_marker) {
//   double x_2 = distance * distance;
//   double x_3 = x_2 * distance;
//   return lane_marker.c0_position() + lane_marker.c1_heading_angle() * distance +
//          lane_marker.c2_curvature() * x_2 +
//          lane_marker.c3_curvature_derivative() * x_3;
// }
}  // namespace lanelineprocess
}  // namespace planning
}  // namespace TL
