/*
 * Copyright (c) TL auto Co., Ltd. 2021-2022. All rights reserved.
 */
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/reset_and_quality_monitor.h"

#include <memory>

namespace TL {
namespace planning {
namespace lanelineprocess {
ResetAndQualityMonitor::ResetAndQualityMonitor(
    const planning::PerceptionMapConfig& config)
    : config_(config),
      left_lane_detm_reset_{std::make_unique<LaneDetmReset>(config)},
      right_lane_detm_reset_{std::make_unique<LaneDetmReset>(config)},
      next_left_lane_detm_reset_{std::make_unique<LaneDetmReset>(config)},
      next_right_lane_detm_reset_{std::make_unique<LaneDetmReset>(config)},
      left_lane_width_quality_monitor_{
          std::make_unique<LaneWidthQualityMonitor>(config)},
      right_lane_width_quality_monitor_{
          std::make_unique<LaneWidthQualityMonitor>(config)},
      next_left_lane_width_quality_monitor_{
          std::make_unique<LaneWidthQualityMonitor>(config)},
      next_right_lane_width_quality_monitor_{
          std::make_unique<LaneWidthQualityMonitor>(config)} {}

Status ResetAndQualityMonitor::Init() {
  const double ego_lane_width_var_thd =
      config_.lanewidth_predictor_config().pre_ego_lanewd_varthd();
  const double next_lane_width_var_thd =
      config_.lanewidth_predictor_config().next_lane_width_varthd();
  left_lane_width_quality_monitor_->SetConf(ego_lane_width_var_thd);
  right_lane_width_quality_monitor_->SetConf(ego_lane_width_var_thd);
  next_left_lane_width_quality_monitor_->SetConf(next_lane_width_var_thd);
  next_right_lane_width_quality_monitor_->SetConf(next_lane_width_var_thd);
  return Status::OK();
}

void ResetAndQualityMonitor::PublicProcess(DeciderData* decider_data) {
  const double ego_distance =
      std::get<0>(decider_data->predict_width_distance_);
  left_lane_width_quality_monitor_->DealCommonData(
      decider_data->lane_speed_cost_time, ego_distance);
  right_lane_width_quality_monitor_->DealCommonData(
      decider_data->lane_speed_cost_time, ego_distance);
  next_left_lane_width_quality_monitor_->DealCommonData(
      decider_data->lane_speed_cost_time,
      std::get<1>(decider_data->predict_width_distance_));
  next_right_lane_width_quality_monitor_->DealCommonData(
      decider_data->lane_speed_cost_time,
      std::get<2>(decider_data->predict_width_distance_));
}

void ResetAndQualityMonitor::GetLLResetAndQuality(DeciderData* decider_data) {
  ADEBUG << "GetLLResetAndQuality Process!"
         << decider_data->original_lanemarkers.header().seq();
  const double vehicle_speed =
      decider_data->filter_vehicle_state.vehicle_speed_average;
  const bool host_lane_change = decider_data->is_host_lanechange;
  const auto& left_lanemarker =
      decider_data->copy_lanemarkers.front_left_lane_marker();
  const auto& right_lanemarker =
      decider_data->trans_lanemarkers.front_right_lane_marker();
  auto* left_reset_info =
      decider_data->lanemarker_lanline_debug->mutable_lanemarker_decider_debug()
          ->mutable_lanes_reset_info()
          ->add_lane_reset();
  left_reset_info->set_name("left_reset");
  left_lane_detm_reset_->Update(
      vehicle_speed, host_lane_change, left_lanemarker.quality(),
      right_lanemarker.quality(), left_lanemarker.longitude_end(),
      decider_data->lane_speed_cost_time, left_reset_info);
  decider_data->lane_reset_out.left_lane_reset =
      left_lane_detm_reset_->GetLaneResetOut();
  const int lane_width_predict_valid =
      decider_data->lane_width_valid_out.ego_lane_widthpredict_valid;
  const double lane_width_predict_sg =
      decider_data->lane_width_valid_out.ego_lane_widthpredict_sg;
  auto* left_split_info =
      decider_data->lanemarker_lanline_debug->mutable_lanemarker_decider_debug()
          ->mutable_lane_split_info()
          ->add_lane_split();
  left_split_info->set_name("left_split_info");
  decider_data->lane_reset_out.left_lane_reset.split_lane_flag =
      left_lane_width_quality_monitor_->Update(
          left_lanemarker, right_lanemarker, lane_width_predict_valid,
          lane_width_predict_sg, host_lane_change, left_split_info,
          decider_data->lane_markers_state.left_lanemarker_state);
  ADEBUG << "is_left_lane_reset: "
         << decider_data->lane_reset_out.left_lane_reset.is_lane_reset
         << ", is_left_lane_split_flag: "
         << decider_data->lane_reset_out.left_lane_reset.split_lane_flag;
}

void ResetAndQualityMonitor::GetRLResetAndQuality(DeciderData* decider_data) {
  ADEBUG << "GetRLResetAndQuality Process!"
         << decider_data->original_lanemarkers.header().seq();
  const double vehicle_speed =
      decider_data->filter_vehicle_state.vehicle_speed_average;
  const bool host_lane_change = decider_data->is_host_lanechange;
  const auto left_lanemarker =
      decider_data->trans_lanemarkers.front_left_lane_marker();
  const auto right_lanemarker =
      decider_data->copy_lanemarkers.front_right_lane_marker();
  auto* right_reset_info =
      decider_data->lanemarker_lanline_debug->mutable_lanemarker_decider_debug()
          ->mutable_lanes_reset_info()
          ->add_lane_reset();
  right_reset_info->set_name("right_reset");
  right_lane_detm_reset_->Update(
      vehicle_speed, host_lane_change, right_lanemarker.quality(),
      left_lanemarker.quality(), right_lanemarker.longitude_end(),
      decider_data->lane_speed_cost_time, right_reset_info);
  decider_data->lane_reset_out.right_lane_reset =
      right_lane_detm_reset_->GetLaneResetOut();
  const int lane_width_predict_valid =
      decider_data->lane_width_valid_out.ego_lane_widthpredict_valid;
  const double lane_width_predict_sg =
      decider_data->lane_width_valid_out.ego_lane_widthpredict_sg;
  auto* right_split_info =
      decider_data->lanemarker_lanline_debug->mutable_lanemarker_decider_debug()
          ->mutable_lane_split_info()
          ->add_lane_split();
  right_split_info->set_name("right_split_info");
  decider_data->lane_reset_out.right_lane_reset.split_lane_flag =
      right_lane_width_quality_monitor_->Update(
          right_lanemarker, left_lanemarker, lane_width_predict_valid,
          lane_width_predict_sg, host_lane_change, right_split_info,
          decider_data->lane_markers_state.right_lanemarker_state);
  ADEBUG << "is_right_lane_reset: "
         << decider_data->lane_reset_out.right_lane_reset.is_lane_reset
         << ", is_right_lane_split_flag: "
         << decider_data->lane_reset_out.right_lane_reset.split_lane_flag;
}

void ResetAndQualityMonitor::GetNLLResetAndQuality(DeciderData* decider_data) {
  ADEBUG << "GetNLLResetAndQuality Process!"
         << decider_data->original_lanemarkers.header().seq();
  const double vehicle_speed =
      decider_data->filter_vehicle_state.vehicle_speed_average;
  const bool host_lane_change = decider_data->is_host_lanechange;
  const auto left_lanemarker =
      decider_data->copy_lanemarkers.front_next_left_lane_marker().at(0);
  const auto right_lanemarker =
      decider_data->trans_lanemarkers.front_left_lane_marker();
  auto* next_left_reset_info =
      decider_data->lanemarker_lanline_debug->mutable_lanemarker_decider_debug()
          ->mutable_lanes_reset_info()
          ->add_lane_reset();
  next_left_reset_info->set_name("next_left_reset");
  next_left_lane_detm_reset_->Update(
      vehicle_speed, host_lane_change, left_lanemarker.quality(),
      right_lanemarker.quality(), left_lanemarker.longitude_end(),
      decider_data->lane_speed_cost_time, next_left_reset_info);
  decider_data->lane_reset_out.next_left_lane_reset =
      next_left_lane_detm_reset_->GetLaneResetOut();
  const int lane_width_predict_valid =
      decider_data->lane_width_valid_out.next_leftlane_widthpredict_valid;
  const double lane_width_predict_sg =
      decider_data->lane_width_valid_out.next_leftlane_widthpredict_sg;
  auto* next_left_split_info =
      decider_data->lanemarker_lanline_debug->mutable_lanemarker_decider_debug()
          ->mutable_lane_split_info()
          ->add_lane_split();
  next_left_split_info->set_name("next_left_split_info");
  decider_data->lane_reset_out.next_left_lane_reset.split_lane_flag =
      next_left_lane_width_quality_monitor_->Update(
          left_lanemarker, right_lanemarker, lane_width_predict_valid,
          lane_width_predict_sg, host_lane_change, next_left_split_info,
          decider_data->lane_markers_state.next_left_lanemarker_state);
  ADEBUG << "is_next_left_lane_reset: "
         << decider_data->lane_reset_out.next_left_lane_reset.is_lane_reset
         << ", is_next_left_lane_split_flag: "
         << decider_data->lane_reset_out.next_left_lane_reset.split_lane_flag;
}

void ResetAndQualityMonitor::GetNRLResetAndQuality(DeciderData* decider_data) {
  ADEBUG << "GetNRLResetAndQuality Process!"
         << decider_data->original_lanemarkers.header().seq();
  const double vehicle_speed =
      decider_data->filter_vehicle_state.vehicle_speed_average;
  const bool host_lane_change = decider_data->is_host_lanechange;
  const auto left_lanemarker =
      decider_data->trans_lanemarkers.front_right_lane_marker();
  const auto right_lanemarker =
      decider_data->copy_lanemarkers.front_next_right_lane_marker().at(0);
  auto* next_right_reset_info =
      decider_data->lanemarker_lanline_debug->mutable_lanemarker_decider_debug()
          ->mutable_lanes_reset_info()
          ->add_lane_reset();
  next_right_reset_info->set_name("next_right_reset");
  next_right_lane_detm_reset_->Update(
      vehicle_speed, host_lane_change, right_lanemarker.quality(),
      left_lanemarker.quality(), left_lanemarker.longitude_end(),
      decider_data->lane_speed_cost_time, next_right_reset_info);
  decider_data->lane_reset_out.next_right_lane_reset =
      next_right_lane_detm_reset_->GetLaneResetOut();
  const int lane_width_predict_valid =
      decider_data->lane_width_valid_out.next_rightlane_widthpredict_valid;
  const double lane_width_predict_sg =
      decider_data->lane_width_valid_out.next_rightlane_widthpredict_sg;
  auto* next_right_split_info =
      decider_data->lanemarker_lanline_debug->mutable_lanemarker_decider_debug()
          ->mutable_lane_split_info()
          ->add_lane_split();
  next_right_split_info->set_name("next_right_split_info");
  decider_data->lane_reset_out.next_right_lane_reset.split_lane_flag =
      next_right_lane_width_quality_monitor_->Update(
          right_lanemarker, left_lanemarker, lane_width_predict_valid,
          lane_width_predict_sg, host_lane_change, next_right_split_info,
          decider_data->lane_markers_state.next_right_lanemarker_state);
  ADEBUG << "is_next_right_lane_reset: "
         << decider_data->lane_reset_out.next_right_lane_reset.is_lane_reset
         << ", is_next_right_lane_split_flag: "
         << decider_data->lane_reset_out.next_right_lane_reset.split_lane_flag;
}
}  // namespace lanelineprocess
}  // namespace planning
}  // namespace TL
