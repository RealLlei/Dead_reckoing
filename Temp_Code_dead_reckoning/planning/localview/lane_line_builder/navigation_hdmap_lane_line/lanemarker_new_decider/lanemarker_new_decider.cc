/*
 * Copyright (c) TL auto Co., Ltd. 2021-2022. All rights reserved.
 */

#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/lanemarker_new_decider.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "absl/strings/match.h"
#include "absl/strings/str_cat.h"
#include "common/time/clock.h"
// #include "common/log.h"
#include "common/configs/config_gflags.h"
#include "common/math/math_utils.h"
#include "common/util/message_util.h"
#include "map/hdmap/hdmap_util.h"
#include "planning/common/planning_gflags.h"
#include "proto/fsm/nnp_fct.pb.h"

namespace TL {
namespace planning {
namespace lanelineprocess {
using TL::common::Clock;
using TL::common::Status;

LanemarkerNewDecider::LanemarkerNewDecider(
    const planning::PerceptionMapConfig& config)
    : config_(config),
      better_quality_threshold_{
          config.lanemarker_quality_reliableforwarning_value()},
      good_quality_threshold_{config.lanemarker_quality_not_reliable_value()},
      main_loop_time_{config.main_loop_time()},
      coortrans_and_copy_{std::make_unique<CoorTransAndCopy>(config)},
      lanechange_decider_{std::make_unique<LaneChangeDecider>(config)},
      lanewidth_predictor_{std::make_unique<LaneWidthPredictor>(config)},
      lanemarker_filter_{std::make_unique<LaneMarkerFilter>(config)},
      reset_and_quality_monitor_{
          std::make_unique<ResetAndQualityMonitor>(config)},
      lanemarker_laneline_{nullptr} {}

Status LanemarkerNewDecider::Init() {
  // Low pass filter
  std::vector<double> den(3, 0.0);
  std::vector<double> num(3, 0.0);
  TL::common::LpfCoefficients(0.03, 10, &den, &num);  // ts_ , cutoff_freq;
  speed_digital_filter_.set_coefficients(den, num);
  yawrate_digital_filter_.set_coefficients(den, num);
  coortrans_and_copy_->Init();
  lanewidth_predictor_->Init();
  reset_and_quality_monitor_->Init();
  lanemarker_filter_->Init();
  lanechange_decider_->Init();
  lanechange_observer_.Init();
  return Status::OK();
}

void LanemarkerNewDecider::SetDecisionMg(
    const std::shared_ptr<LocalView>& local_view,
    functionmanager::FunctionManagerOut* const to_fct, bool laneline_status) {
  vehicle_state_ = local_view->GetVehicleState();
  localization_ = local_view->GetLocalization();
  perception_obstacles_ = local_view->GetPerceptionObstacles();
  history_perception_sub_state_ = to_fct->perception_sub_state();
  history_laneline_status_ = laneline_status;
  history_noline_status_ = to_fct->nolane_status();
  auto fct_in = local_view->GetFunctionManagerIn()->fct_nnp_in();
  history_nnp_state_ = fct_in.nnp_sysstate();
  history_acc_sytate_ = fct_in.acc_state();
  history_pilot_state_ = fct_in.npilot_state();
  history_drive_mode_ = local_view->GetChassis()->driving_mode();
}

bool LanemarkerNewDecider::Decision(
    LaneMarkers* lane_marker, LanePointsMap* lane_points_map,
    const std::shared_ptr<LocalView>& local_view,
    std::pair<bool, bool>* lane_markers_points_copy) {
  ADEBUG << "lane_marker_time:" << FIXED << SETPRECISION(3)
         << Clock::NowInSeconds();
  ADEBUG << "lane marker:" << lane_marker->DebugString();
  DeciderData decider_data;
  lanemarker_laneline_ =
        common::memory::ArenaAdapter::CreateMessage<LanemarkersLaneLine>(
            local_view->GetArena());
  decider_data.lanemarker_lanline_debug = lanemarker_laneline_;
  ADEBUG << "before filter, speed: " << vehicle_state_->linear_velocity()
         << ", yaw_rate: " << vehicle_state_->angular_velocity();
  decider_data.filter_vehicle_state.vehicle_speed_average =
      vehicle_state_->linear_velocity();
  decider_data.filter_vehicle_state.vehicle_yaw_rate =
      vehicle_state_->angular_velocity();
  ADEBUG << "after filter, speed: "
         << decider_data.filter_vehicle_state.vehicle_speed_average
         << ", yaw_rate: "
         << decider_data.filter_vehicle_state.vehicle_yaw_rate;
  decider_data.is_lanechange_to_left = lane_marker->is_lanechange_to_left();
  decider_data.is_lanechange_to_right = lane_marker->is_lanechange_to_right();
  decider_data.is_host_lanechange = (decider_data.is_lanechange_to_left ||
                                     decider_data.is_lanechange_to_right);

  lanemarker_filter_->ComputeFilterData(decider_data);
  PreLanemarkers(&decider_data, *lane_marker, vehicle_state_->linear_velocity(),
                 vehicle_state_->angular_velocity());
  coortrans_and_copy_->LaneMarkerCoorTrans(&decider_data);
  ADEBUG << "coortrans lanemarker: "
         << decider_data.trans_lanemarkers.DebugString();
  IsLaneChange(&decider_data);

  DeciderObsBeforeVehicle(*lane_marker);
  // JudgeLaneMarkersState
  JudgeLaneMarkersState(&decider_data);
  lanewidth_predictor_->WdePredict(&decider_data);
  lanechange_decider_->ComputerCoastTime(&decider_data);
  reset_and_quality_monitor_->PublicProcess(&decider_data);
  // lanemarker copy
  coortrans_and_copy_->EgoLaneMarkerCopy(&decider_data);
  coortrans_and_copy_->NextLaneMarkerCopy(&decider_data);
  // return false;
  coortrans_and_copy_->TimerScheduler(&decider_data);
  if (decider_data.do_egolane_update) {
    DoEgolaneUpdate(&decider_data);
  }
  if (decider_data.do_nextlane_update) {
    DoNextlaneUpdate(&decider_data);
  }
  lane_width_ = decider_data.lane_width_valid_out;
  lane_points_map->swap(decider_data.lane_marker_points);
  std::get<0>(*lane_markers_points_copy) =
      decider_data.lanemarker_lanline_debug->lanemarker_decider_debug()
          .lanemarkers_copy_info()
          .left_is_copy();
  std::get<1>(*lane_markers_points_copy) =
      decider_data.lanemarker_lanline_debug->lanemarker_decider_debug()
          .lanemarkers_copy_info()
          .right_is_copy();
  // lane_marker->CopyFrom(decider_data.out_lanemarkers);
  // LogLanemarkerErr(*lane_marker,*lane_points_map);
  ADEBUG << "New decider LogicalDecision end";
  OutDebugInfo(&decider_data);
  return true;
}

void LanemarkerNewDecider::DoEgolaneUpdate(DeciderData* decider_data) {
  LeftlaneDecider(decider_data);
  RightlaneDecider(decider_data);
}

void LanemarkerNewDecider::DoNextlaneUpdate(DeciderData* decider_data) {
  NLeftlaneDecider(decider_data);
  NRightlaneDecider(decider_data);
}

void LanemarkerNewDecider::LeftlaneDecider(DeciderData* decider_data) {
  reset_and_quality_monitor_->GetLLResetAndQuality(decider_data);
  lanechange_decider_->GetLeftChangeFlag(decider_data);
  lanemarker_filter_->LeftLaneFilter(decider_data);
  lanechange_decider_->GetLeftChangeCoff(decider_data);
}

void LanemarkerNewDecider::RightlaneDecider(DeciderData* decider_data) {
  reset_and_quality_monitor_->GetRLResetAndQuality(decider_data);
  lanechange_decider_->GetRightChangeFlag(decider_data);
  lanemarker_filter_->RightLaneFilter(decider_data);
  lanechange_decider_->GetRightChangeCoff(decider_data);
}

void LanemarkerNewDecider::NLeftlaneDecider(DeciderData* decider_data) {
  reset_and_quality_monitor_->GetNLLResetAndQuality(decider_data);
  lanemarker_filter_->NLeftLaneFilter(decider_data);
}

void LanemarkerNewDecider::NRightlaneDecider(DeciderData* decider_data) {
  reset_and_quality_monitor_->GetNRLResetAndQuality(decider_data);
  lanemarker_filter_->NRightLaneFilter(decider_data);
}

void LanemarkerNewDecider::OutDebugInfo(DeciderData* decider_data) {
  common::util::FillHeader("lanemarker decider debug",
                           lanemarker_laneline_.get());
  auto* lanemarker_debug =
      lanemarker_laneline_->mutable_lanemarker_decider_debug();
  auto* original_lanemarkers = lanemarker_debug->add_lanemarkerinfo();
  original_lanemarkers->set_name("original_lanemarkers");
  original_lanemarkers->mutable_lanemarkers()->CopyFrom(
      decider_data->original_lanemarkers);

  auto* trans_lanemarkers = lanemarker_debug->add_lanemarkerinfo();
  trans_lanemarkers->set_name("trans_lanemarkers");
  trans_lanemarkers->mutable_lanemarkers()->CopyFrom(
      decider_data->trans_lanemarkers);

  auto* copy_lanemarkers = lanemarker_debug->add_lanemarkerinfo();
  copy_lanemarkers->set_name("copy_lanemarkers");
  copy_lanemarkers->mutable_lanemarkers()->CopyFrom(
      decider_data->copy_lanemarkers);

  lanemarker_debug->set_vehicle_speed(
      decider_data->filter_vehicle_state.vehicle_speed_average);
  lanemarker_debug->set_yaw_rate(
      decider_data->filter_vehicle_state.vehicle_yaw_rate);
  lanemarker_debug->set_is_lanechange_to_left(
      decider_data->trans_lanemarkers.is_lanechange_to_left());
  lanemarker_debug->set_is_lanechange_to_right(
      decider_data->trans_lanemarkers.is_lanechange_to_right());
  lanemarker_debug->set_original_left_lane_quality(
      decider_data->original_lanemarkers.front_left_lane_marker().quality());
  lanemarker_debug->set_original_right_lane_quality(
      decider_data->original_lanemarkers.front_right_lane_marker().quality());
  lanemarker_debug->set_original_next_left_lane_quality(
      decider_data->original_lanemarkers.front_next_left_lane_marker()
          .begin()
          ->quality());
  lanemarker_debug->set_original_next_right_lane_quality(
      decider_data->original_lanemarkers.front_next_right_lane_marker()
          .begin()
          ->quality());

  auto* lanes_width_predict =
      lanemarker_debug->mutable_lanes_width_predict_info();
  auto lane_width_out = decider_data->lane_width_valid_out;
  lanes_width_predict->set_ego_lane_width_pre_valid(
      lane_width_out.ego_lane_widthpredict_valid);
  lanes_width_predict->set_ego_lane_width_pre_value(
      lane_width_out.ego_lane_widthpredict_sg);
  lanes_width_predict->set_left_lane_width_pre_valid(
      lane_width_out.next_leftlane_widthpredict_valid);
  lanes_width_predict->set_left_lane_width_pre_value(
      lane_width_out.next_leftlane_widthpredict_sg);
  lanes_width_predict->set_right_lane_width_pre_valid(
      lane_width_out.next_rightlane_widthpredict_valid);
  lanes_width_predict->set_right_lane_width_pre_value(
      lane_width_out.next_rightlane_widthpredict_sg);
  auto* lane_reset_debug = lanemarker_debug->mutable_lanes_reset_info();
  auto* lane_split_debug = lanemarker_debug->mutable_lane_split_info();
  auto lane_reset_out = decider_data->lane_reset_out;
  lane_reset_debug->set_is_left_reset(
      lane_reset_out.left_lane_reset.is_lane_reset);
  lane_reset_debug->set_is_right_reset(
      lane_reset_out.right_lane_reset.is_lane_reset);
  lane_reset_debug->set_is_next_left_reset(
      lane_reset_out.next_left_lane_reset.is_lane_reset);
  lane_reset_debug->set_is_next_right_reset(
      lane_reset_out.next_right_lane_reset.is_lane_reset);
  lane_split_debug->set_is_left_split(
      lane_reset_out.left_lane_reset.split_lane_flag);
  lane_split_debug->set_is_right_split(
      lane_reset_out.right_lane_reset.split_lane_flag);
  lane_split_debug->set_is_next_left_split(
      lane_reset_out.next_left_lane_reset.split_lane_flag);
  lane_split_debug->set_is_next_right_split(
      lane_reset_out.next_right_lane_reset.split_lane_flag);
  auto* lane_change_debug = lanemarker_debug->mutable_lane_change_info();
  lane_change_debug->set_left_lanechange_mnt_flag(
      decider_data->lane_change_out.left_lanechange_mnt_flag);
  lane_change_debug->set_right_lanechange_mnt_flag(
      decider_data->lane_change_out.right_lanechange_mnt_flag);
  // 根据选择不发送数据
  if (FLAGS_lane_line_decider_debug_flag == 0) {
    lanemarker_laneline_->mutable_lanemarker_decider_debug()->Clear();
  }
}

void LanemarkerNewDecider::LogLanemarkerErr(const LaneMarkers& lane_marker,
                                            const LanePointsMap& lane_points) {
  if (lane_points.find("left") == lane_points.end() ||
      lane_points.find("right") == lane_points.end()) {
    return;
  }
  auto left_filter_points = lane_points.at("left");
  auto right_filter_points = lane_points.at("right");
  std::vector<Vec2d> left_lanemarker_points;
  std::vector<Vec2d> right_lanemarker_points;
  GetMeasurePoints(lane_marker.front_left_lane_marker(), left_filter_points,
                   &left_lanemarker_points);
  GetMeasurePoints(lane_marker.front_right_lane_marker(), right_filter_points,
                   &right_lanemarker_points);

  std::vector<Vec2d> left_map_points;
  std::vector<Vec2d> right_map_points;
  const double max_map_length =
      std::max(lane_marker.front_left_lane_marker().view_range(),
               lane_marker.front_right_lane_marker().view_range());
  const double step_length = 1.0;
  TL::common::math::Vec2d position_point(
      localization_->pose().position().x(),
      localization_->pose().position().y());
  common::PointENU point;
  point.set_x(localization_->pose().position().x());
  point.set_y(localization_->pose().position().y());
  // std::vector<std::shared_ptr<const hdmap::LaneInfo>> lanes;
  // const double search_distance = 15.0;
  // hdmap_->GetLanes(point, search_distance, &lanes);
  TL::hdmap::LaneInfoConstPtr nearest_lane;
  double s = 0;
  double l = 0;
  hdmap::HDMapUtil::BaseMapPtr()->GetNearestLane(
      localization_->pose().position(), &nearest_lane, &s, &l);
  double total_length = nearest_lane->total_length();
  double start_s = s;
  while (start_s < std::min(total_length, s + max_map_length)) {
    auto point = nearest_lane->GetSmoothPoint(start_s);
    auto bus_point = PointEarth2Bus(point, localization_->pose());
    double left_width{0.0};
    double right_width{0.0};
    nearest_lane->GetWidth(start_s, &left_width, &right_width);
    left_map_points.emplace_back(bus_point.x(), bus_point.y() + left_width);
    right_map_points.emplace_back(bus_point.x(), bus_point.y() - right_width);
    start_s += step_length;
  }
  if (total_length - s < max_map_length &&
      nearest_lane->lane().successor_id_size() > 0) {
    auto successor_lane = hdmap::HDMapUtil::BaseMapPtr()->GetLaneById(
        nearest_lane->lane().successor_id().at(0));
    double next_start_s = 0.0;
    while (next_start_s < std::min(successor_lane->total_length(),
                                   max_map_length - (total_length - s))) {
      auto point = nearest_lane->GetSmoothPoint(next_start_s);
      auto bus_point = PointEarth2Bus(point, localization_->pose());
      double left_width{0.0};
      double right_width{0.0};
      nearest_lane->GetWidth(next_start_s, &left_width, &right_width);
      left_map_points.emplace_back(bus_point.x(), bus_point.y() + left_width);
      right_map_points.emplace_back(bus_point.x(), bus_point.y() - right_width);
      next_start_s += step_length;
    }
  }

  for (auto& point : left_filter_points) {
    AERROR << "left_filter_points_x: " << point.x();
    AERROR << "left_filter_points_y: " << point.y();
  }
  for (auto& point : right_filter_points) {
    AERROR << "right_filter_points_x: " << point.x();
    AERROR << "right_filter_points_y: " << point.y();
  }

  for (auto& point : left_lanemarker_points) {
    AERROR << "left_lanemarker_points_x: " << point.x();
    AERROR << "left_lanemarker_points_y: " << point.y();
  }
  for (auto& point : right_lanemarker_points) {
    AERROR << "right_lanemarker_points_x: " << point.x();
    AERROR << "right_lanemarker_points_y: " << point.y();
  }

  for (auto& point : left_map_points) {
    AERROR << "left_map_points_x: " << point.x();
    AERROR << "left_map_points_y: " << point.y();
  }
  for (auto& point : right_map_points) {
    AERROR << "right_map_points_x: " << point.x();
    AERROR << "right_map_points_y: " << point.y();
  }
}

bool LanemarkerNewDecider::GetMeasurePoints(
    const LaneMarker& lanemarker, const std::vector<Vec2d>& filter_points,
    std::vector<Vec2d>* lanemarker_points) {
  if (filter_points.size() < 2) {
    return false;
  }
  for (auto filter_point : filter_points) {
    double x = filter_point.x();
    double y = lanemarker.c0_position() + lanemarker.c1_heading_angle() * x +
               lanemarker.c2_curvature() * std::pow(x, 2) +
               lanemarker.c3_curvature_derivative() * std::pow(x, 3);
    lanemarker_points->push_back(Vec2d(x, y));
  }
  return true;
}

void LanemarkerNewDecider::TranslatedPoints(
    std::vector<Vec2d>* lanemarker_points) {
  std::vector<Vec2d> trans_points;
  for (auto& lanemarker_point : *lanemarker_points) {
    Eigen::Vector2d enu_coordinate = common::math::RotateVector2d(
        {lanemarker_point.x(), lanemarker_point.y()},
        vehicle_state_->heading());
    double x_enu = enu_coordinate.x() + vehicle_state_->x();
    double y_enu = enu_coordinate.y() + vehicle_state_->y();
    trans_points.emplace_back(x_enu, y_enu);
  }
  lanemarker_points->swap(trans_points);
}

void LanemarkerNewDecider::IsLaneChange(DeciderData* decider_data) {
  // change first:is_left_change, second:is_right_change
  const auto change =
      lanechange_observer_.Observer(decider_data->trans_lanemarkers);
  decider_data->trans_lanemarkers.set_is_lanechange_to_left(change.first);
  decider_data->trans_lanemarkers.set_is_lanechange_to_right(change.second);
  decider_data->is_lanechange_to_left = change.first;
  decider_data->is_lanechange_to_right = change.second;
  decider_data->is_host_lanechange = change.first || change.second;
}

bool LanemarkerNewDecider::LeftLaneLineJump(
    const DeciderData& decider_data) const {
  return std::abs(decider_data.original_lanemarkers.front_left_lane_marker()
                      .c0_position() -
                  left_line_c0_old_) > 3 &&
         !decider_data.is_host_lanechange;
}

bool LanemarkerNewDecider::RightLaneLineJump(
    const DeciderData& decider_data) const {
  return std::abs(decider_data.original_lanemarkers.front_right_lane_marker()
                      .c0_position() -
                  right_line_c0_old_) > 3 &&
         !decider_data.is_host_lanechange;
}

std::pair<bool, bool> LanemarkerNewDecider::LineStartIsGood(
    double left_start, double right_start) {
  std::pair<bool, bool> line_start_good(true, true);
  if ((left_start >= 10) && (right_start <= 5)) {
    line_start_good.first = false;
  }
  if ((left_start <= 5) && (right_start >= 10)) {
    line_start_good.second = false;
  }
  return line_start_good;
}

void LanemarkerNewDecider::JudgeLaneMarkersState(DeciderData* decider_data) {
  // const double max_lat_err_m =
  //     config_.lanemarker_decider_config().max_lat_err_m();
  const auto left_lanemarker =
      decider_data->original_lanemarkers.front_left_lane_marker();
  const auto right_lanemarker =
      decider_data->original_lanemarkers.front_right_lane_marker();
  const auto next_left_lanemarker =
      decider_data->original_lanemarkers.front_next_left_lane_marker().at(0);
  const auto next_right_lanemarker =
      decider_data->original_lanemarkers.front_next_right_lane_marker().at(0);
  const double rise_time =
      config_.lanemarker_decider_config().limit_time_rise();
  const double down_time =
      config_.lanemarker_decider_config().limit_time_down();
  std::pair<bool, bool> lane_line_start = LineStartIsGood(
      left_lanemarker.longitude_start(), right_lanemarker.longitude_start());
  is_nolane_ok_ = history_noline_status_ && !FLAGS_not_using_nolane &&
                  left_lanemarker.longitude_end() < 8.0 &&
                  right_lanemarker.longitude_end() < 8.0;
  JudgeLaneMarkerState(left_lanemarker,
                       &lane_markers_state_.left_lanemarker_state,
                       lane_markers_state_limit_.data(), rise_time, down_time,
                       LeftLaneLineJump(*decider_data), lane_line_start.first);
  JudgeLaneMarkerState(
      right_lanemarker, &lane_markers_state_.right_lanemarker_state,
      &lane_markers_state_limit_[1], rise_time, down_time,
      RightLaneLineJump(*decider_data), lane_line_start.second);
  JudgeLaneMarkerState(next_left_lanemarker,
                       &lane_markers_state_.next_left_lanemarker_state,
                       &lane_markers_state_limit_[2]);
  JudgeLaneMarkerState(next_right_lanemarker,
                       &lane_markers_state_.next_right_lanemarker_state,
                       &lane_markers_state_limit_[3]);
  ADEBUG << "left_jump :" << LeftLaneLineJump(*decider_data);
  ADEBUG << "right_jump :" << RightLaneLineJump(*decider_data);
  left_line_c0_old_ =
      decider_data->original_lanemarkers.front_left_lane_marker().c0_position();
  right_line_c0_old_ =
      decider_data->original_lanemarkers.front_right_lane_marker()
          .c0_position();
  ADEBUG << "left_lanemarker_state: "
         << lane_markers_state_.left_lanemarker_state
         << ", right_lanemarker_state: "
         << lane_markers_state_.right_lanemarker_state
         << ", next_left_lanemarker_state: "
         << lane_markers_state_.next_left_lanemarker_state
         << ", next_left_lanemarker_state: "
         << lane_markers_state_.next_right_lanemarker_state;

  ADEBUG << " ori left_lanemarker state: "
         << lane_markers_state_.left_lanemarker_state
         << " right_lanemarker state: "
         << lane_markers_state_.right_lanemarker_state;

  decider_data->lane_markers_state = lane_markers_state_;
  // 如果上一帧是视觉车道线模式，有一个lanemarker为better，两个都是better，为了避免一边车道线过短
  // 或者短暂消失，未触发copy时就退出视觉车道线模式，但是不能改变本地的车道线状态，只能改变传给下游的
  // 具体逻辑由下游处理，突然的消失或变差下游会有6s的copy时间。
  // if (history_lane_line_state_ && (lane_markers_state_.left_lanemarker_state ==
  //                                      LaneMarkerState::BETTER_LANEMARKER ||
  //                                  lane_markers_state_.right_lanemarker_state ==
  //                                      LaneMarkerState::BETTER_LANEMARKER)) {
  //   decider_data->lane_markers_state.right_lanemarker_state =
  //       LaneMarkerState::BETTER_LANEMARKER;
  //   decider_data->lane_markers_state.left_lanemarker_state =
  //       LaneMarkerState::BETTER_LANEMARKER;
  // }
  ADEBUG << "OUT--- left_lanemarker state: "
         << decider_data->lane_markers_state.left_lanemarker_state
         << " right_lanemarker state: "
         << decider_data->lane_markers_state.right_lanemarker_state;
}

void LanemarkerNewDecider::JudgeLaneMarkerState(
    const LaneMarker& lane_marker, LaneMarkerState* state,
    std::pair<double, double>* time, const double rise_time,
    const double down_time, bool is_jump_line, bool line_start_is_good) {
  const auto& lane_conf = config_.lanemarker_decider_config();
  const double limit_time_rise = rise_time;  // 0.8
  const double limit_time_down = down_time;  // 0.4
  const double min_lanemarker_length =
      lane_conf.min_lanemarker_length();  // 20m
  const double break_mode_lanemarker_length =
      lane_conf.break_mode_lanemarker_length();  // 15
  const double min_lanemarker_quality =
      lane_conf.min_lanemarker_quality();  // 0.4
  const double quality = lane_marker.quality();
  const double lanemarker_length =
      lane_marker.longitude_end() - lane_marker.longitude_start();
  const bool has_no_lanemarker =
      quality < 0.01 && lane_marker.view_range() < 0.1;
  ADEBUG << "before: quality: " << quality << ", state: " << *state
         << ", time_0: " << std::get<0>(*time)
         << ", time_1: " << std::get<0>(*time)
         << ", main_loop_time: " << main_loop_time_;
  switch (*state) {
    case LaneMarkerState::BAD_LANEMARKER:
      if ((quality >= better_quality_threshold_) && line_start_is_good) {
        if (std::get<0>(*time) >= limit_time_down &&
            lanemarker_length > (min_lanemarker_length - 5)) {
          *state = LaneMarkerState::BETTER_LANEMARKER;
          *time = std::make_pair(0.0, 0.0);
        } else {
          std::get<0>(*time) += main_loop_time_;
        }
      }
      if ((quality >= min_lanemarker_quality) && line_start_is_good) {
        if (std::get<1>(*time) >= limit_time_rise &&
            lanemarker_length > (min_lanemarker_length - 5)) {
          *state = LaneMarkerState::GOOD_LANEMARKER;
          *time = std::make_pair(0.0, 0.0);
        } else {
          std::get<1>(*time) += main_loop_time_;
        }
      } else {
        *time = std::make_pair(0.0, 0.0);
      }
      break;
    case LaneMarkerState::GOOD_LANEMARKER:
      if (is_jump_line || !line_start_is_good || is_nolane_ok_) {
        *state = LaneMarkerState::BAD_LANEMARKER;
        break;
      }
      if (quality >= better_quality_threshold_) {
        if (std::get<0>(*time) >= limit_time_down) {
          *state = LaneMarkerState::BETTER_LANEMARKER;
          *time = std::make_pair(0.0, 0.0);
        } else {
          std::get<0>(*time) += main_loop_time_;
          std::get<1>(*time) = 0.0;
        }
      } else if (quality < good_quality_threshold_ ||
                 (lanemarker_length < (break_mode_lanemarker_length - 5) &&
                  !has_obs_flag_)) {
        if (std::get<1>(*time) >= limit_time_rise) {
          *state = LaneMarkerState::BAD_LANEMARKER;
          *time = std::make_pair(0.0, 0.0);
        } else {
          std::get<1>(*time) +=
              has_no_lanemarker ? 2 * main_loop_time_ : main_loop_time_;
          std::get<0>(*time) = 0.0;
        }
      } else {
        *time = std::make_pair(0.0, 0.0);
      }
      break;
    case LaneMarkerState::BETTER_LANEMARKER:
      if (is_jump_line || !line_start_is_good || is_nolane_ok_) {
        *state = LaneMarkerState::BAD_LANEMARKER;
        break;
      }
      if (has_no_lanemarker ||
          (lanemarker_length < (break_mode_lanemarker_length - 5) &&
           !has_obs_flag_)) {
        if (std::get<0>(*time) > limit_time_rise) {
          *state = LaneMarkerState::BAD_LANEMARKER;
          *time = std::make_pair(0.0, 0.0);
        } else {
          std::get<0>(*time) += 2 * main_loop_time_;
          std::get<1>(*time) = 0.0;
        }
      } else if (quality < better_quality_threshold_) {
        if (std::get<1>(*time) > limit_time_rise) {
          *state = LaneMarkerState::GOOD_LANEMARKER;
          *time = std::make_pair(0.0, 0.0);
        } else {
          std::get<1>(*time) += main_loop_time_;
          std::get<0>(*time) = 0.0;
        }
      } else {
        *time = std::make_pair(0.0, 0.0);
      }
      break;
    default:
      break;
  }
  ADEBUG << "After: quality: " << quality << ", state: " << *state
         << ", time_0: " << std::get<0>(*time)
         << ", time_1: " << std::get<1>(*time)
         << ", main_loop_time: " << main_loop_time_;
}

Vec2d LanemarkerNewDecider::PointEarth2Bus(const common::PointENU& point,
                                           const common::Pose& pose) {
  Vec2d bus_point;
  double x = point.x() - pose.position().x();
  double y = point.y() - pose.position().y();
  bus_point.set_x(x * std::cos(pose.heading()) + y * std::sin(pose.heading()));
  bus_point.set_y(-x * std::sin(pose.heading()) + y * std::cos(pose.heading()));
  return bus_point;
}

void LanemarkerNewDecider::PreLanemarkers(DeciderData* const decider_data,
                                          const LaneMarkers& lane_markers,
                                          const double v_spd,
                                          const double yaw_rate) {
  const double pre_time = config_.lanemarkers_pre_time();
  LaneMarkers pre_lanemarkers;
  if (lane_markers.has_front_right_lane_marker() &&
      lane_markers.front_right_lane_marker().view_range() > kMinLaneDis) {
    ADEBUG << "pre right lanemarker!";
    PreLanemarker(lane_markers.front_right_lane_marker(),
                  pre_lanemarkers.mutable_front_right_lane_marker(), v_spd,
                  yaw_rate, pre_time);
  } else {
    pre_lanemarkers.mutable_front_right_lane_marker()->CopyFrom(
        lane_markers.front_right_lane_marker());
  }
  if (lane_markers.has_front_left_lane_marker() &&
      lane_markers.front_left_lane_marker().view_range() > kMinLaneDis) {
    ADEBUG << "pre left lanemarker!";
    PreLanemarker(lane_markers.front_left_lane_marker(),
                  pre_lanemarkers.mutable_front_left_lane_marker(), v_spd,
                  yaw_rate, pre_time);
  } else {
    pre_lanemarkers.mutable_front_left_lane_marker()->CopyFrom(
        lane_markers.front_left_lane_marker());
  }
  pre_lanemarkers.add_front_next_left_lane_marker()->CopyFrom(
      lane_markers.front_next_left_lane_marker().at(0));
  pre_lanemarkers.add_front_next_right_lane_marker()->CopyFrom(
      lane_markers.front_next_right_lane_marker().at(0));
  /**
  AERROR << "--------------left----------------";
  AERROR << "ori lanemarkers left c0 = "
         << lane_markers.front_left_lane_marker().c0_position() << " , c1 = "
         << lane_markers.front_left_lane_marker().c1_heading_angle()
         << " , c2 = " << lane_markers.front_left_lane_marker().c2_curvature();
  AERROR << "pre lanemarkers left c0 = "
         << pre_lanemarkers.front_left_lane_marker().c0_position() << " , c1 = "
         << pre_lanemarkers.front_left_lane_marker().c1_heading_angle()
         << " , c2 = "
         << pre_lanemarkers.front_left_lane_marker().c2_curvature();
  AERROR << "--------------right----------------";
  AERROR << "ori lanemarkers right c0 = "
         << lane_markers.front_right_lane_marker().c0_position() << " , c1 = "
         << lane_markers.front_right_lane_marker().c1_heading_angle()
         << " , c2 = " << lane_markers.front_right_lane_marker().c2_curvature();
  AERROR << "pre lanemarkers right c0 = "
         << pre_lanemarkers.front_right_lane_marker().c0_position()
         << " , c1 = "
         << pre_lanemarkers.front_right_lane_marker().c1_heading_angle()
         << " , c2 = "
         << pre_lanemarkers.front_right_lane_marker().c2_curvature();
  */
  decider_data->original_lanemarkers.CopyFrom(pre_lanemarkers);
}

void LanemarkerNewDecider::PreLanemarker(const LaneMarker& ori_lane_marker,
                                         LaneMarker* const pre_lanemarker,
                                         const double speed,
                                         const double yaw_rate,
                                         const double pre_time) {
  pre_lanemarker->CopyFrom(ori_lane_marker);
  const double ori_c0 = ori_lane_marker.c0_position();
  const double ori_c1 = std::atan(ori_lane_marker.c1_heading_angle());
  const double ori_c2 = ori_lane_marker.c2_curvature() * 2;
  const double ori_c3 = ori_lane_marker.c3_curvature_derivative() * 6;
  const double dx = speed * pre_time;
  double c0 = ori_c0 + ori_c1 * dx + kHalfNum * ori_c2 * dx * dx +
              ori_c3 * dx * dx * dx / kMagicNumber6 -
              kHalfNum * pre_time * dx * yaw_rate;
  double c1 =
      std::tan(std::tan(ori_c1) + ori_c2 * dx + kHalfNum * ori_c3 * dx * dx -
               kHalfNum * pre_time * yaw_rate);
  pre_lanemarker->set_c0_position(c0);
  pre_lanemarker->set_c1_heading_angle(c1);
  pre_lanemarker->set_c2_curvature(kHalfNum * (ori_c2 + dx * ori_c3));
}

void LanemarkerNewDecider::DeciderObsBeforeVehicle(
    const LaneMarkers& lane_markers) {
  const double vehicle_speed = vehicle_state_->linear_velocity();
  const double yaw_rate = vehicle_state_->angular_velocity();
  constexpr double kMinVehicleSpeed = 12.0;
  constexpr double kMaxFrontDistance = kMinVehicleSpeed * 2;
  constexpr double delta_width = 0.3;
  has_obs_flag_ = false;
  if (vehicle_speed > kMinVehicleSpeed) {
    return;
  }
  const auto& left_lanemarker = lane_markers.front_left_lane_marker();
  const auto& right_lanemarker = lane_markers.front_right_lane_marker();
  const bool left_quality_good =
      left_lanemarker.quality() > 0.6 && left_lanemarker.view_range() > 20.0;
  const bool right_quality_good =
      right_lanemarker.quality() > 0.6 && right_lanemarker.view_range() > 20.0;
  LaneMarker lanemarker;
  double half_lane_width{2.15};
  if (left_quality_good && right_quality_good) {
    lanemarker.set_c0_position(
        (left_lanemarker.c0_position() + right_lanemarker.c0_position()) / 2);
    lanemarker.set_c1_heading_angle((left_lanemarker.c1_heading_angle() +
                                     right_lanemarker.c1_heading_angle()) /
                                    2);
    lanemarker.set_c2_curvature(
        (left_lanemarker.c2_curvature() + right_lanemarker.c2_curvature()) / 2);
    lanemarker.set_c3_curvature_derivative(
        (left_lanemarker.c3_curvature_derivative() +
         right_lanemarker.c3_curvature_derivative()) /
        2);
    half_lane_width =
        (left_lanemarker.c0_position() - right_lanemarker.c0_position()) / 2 +
        delta_width;
  } else if (left_quality_good && !right_quality_good) {
    lanemarker = left_lanemarker;
    lanemarker.set_c0_position(0.0);
  } else if (!left_quality_good && right_quality_good) {
    lanemarker = right_lanemarker;
    lanemarker.set_c0_position(0.0);
  } else if (!left_quality_good && !right_quality_good) {
    auto curvature =
        common::math::Clamp((yaw_rate / vehicle_speed) / 2, -0.01, 0.01);
    lanemarker.set_c0_position(0.0);
    lanemarker.set_c1_heading_angle(0.0);
    lanemarker.set_c2_curvature(curvature);
    lanemarker.set_c3_curvature_derivative(0.0);
  }

  for (const auto& obs : perception_obstacles_->perception_obstacle()) {
    const auto& obs_y = obs.position_flu().y();
    const auto& obs_x = obs.position_flu().x();
    if (obs_x < 0.0 || obs_x > kMaxFrontDistance) {
      continue;
    }
    auto obs_width_dis = CalculateObsY(lanemarker, obs_x, obs_y);
    if (obs_width_dis < half_lane_width) {
      has_obs_flag_ = true;
      break;
    }
  }
  ADEBUG << "has_obs_flag: " << has_obs_flag_;
}

double LanemarkerNewDecider::CalculateObsY(const LaneMarker& lane_marker,
                                           double obs_x, double obs_y) {
  auto c0 = lane_marker.c0_position();
  auto c1 = lane_marker.c1_heading_angle();
  auto c2 = lane_marker.c2_curvature();
  auto c3 = lane_marker.c3_curvature_derivative();
  auto obs_x2 = obs_x * obs_x;
  auto obs_x3 = obs_x2 * obs_x;
  double obs_c0 = obs_y - c1 * obs_x - c2 * obs_x2 - c3 * obs_x3;
  double angle = c1 + c2 * obs_x + c3 * obs_x2;
  return std::fabs(obs_c0 - c0) * cos(angle);
}
}  // namespace lanelineprocess
}  // namespace planning
}  // namespace TL
