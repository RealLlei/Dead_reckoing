/*
 * Copyright (c) TL auto Co., Ltd. 2021-2022. All rights reserved.
 */
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/lane_width_predictor.h"
#include <cmath>
#include <cstddef>
#include "common/configs/vehicle_config_helper.h"
#include "proto/perception/perception_obstacle.pb.h"

// #include "cyber/common/macros.h"

namespace TL {
namespace planning {
namespace lanelineprocess {

LaneWidthPredictor::LaneWidthPredictor(
    const planning::PerceptionMapConfig& config)
    : config_(config) {
  ADEBUG << "LaneWidthPredictor CONSTRUCTOR";
  left_lane_left_line_quality_delay_ = Delay<double>(5);
  left_lane_right_line_quality_delay_ = Delay<double>(kMagicNumber5);
  left_lane_wide_delay_ = Delay<double>(kMagicNumber5);

  right_lane_left_line_quality_delay_ = Delay<double>(kMagicNumber5);
  right_lane_right_line_quality_delay_ = Delay<double>(kMagicNumber5);
  right_lane_wide_delay_ = Delay<double>(kMagicNumber5);

  ego_lane_left_line_quality_delay_ = Delay<double>(kMagicNumber5);
  ego_lane_right_line_quality_delay_ = Delay<double>(kMagicNumber5);
  ego_lane_wide_delta_delay_ = Delay<double>(kMagicNumber5);
}

Status LaneWidthPredictor::Init() {
  auto conf = config_.lanewidth_predictor_config();
  LoadDistanceCalibrationTable(conf);
  ego_debounce_module_.ResetTime(conf.fall_time(), conf.rise_time(),
                                 config_.main_loop_time());
  left_debounce_module_.ResetTime(conf.fall_time(), conf.rise_time(),
                                  config_.main_loop_time());
  right_debounce_module_.ResetTime(conf.fall_time(), conf.rise_time(),
                                   config_.main_loop_time());
  main_loop_time_ = config_.main_loop_time();
  left_lane_wide_coast_time_ = conf.lane_width_coast_time();
  good_quality_threshold_ = config_.lanemarker_quality_not_reliable_value();
  batter_quality_threshold_ =
      config_.lanemarker_quality_reliableforwarning_value();
  vehicle_param_ = common::VehicleConfigHelper::GetConfig().vehicle_param();
  ego_left_lane_exist_debounce_module_.ResetTime(0.3, 0.0,
                                                 config_.main_loop_time());
  ego_right_lane_exist_debounce_module_.ResetTime(0.3, 0.0,
                                                  config_.main_loop_time());
  left_left_lane_exist_debounce_module_.ResetTime(0.3, 0.0,
                                                  config_.main_loop_time());
  left_right_lane_exist_debounce_module_.ResetTime(0.3, 0.0,
                                                   config_.main_loop_time());
  right_left_lane_exist_debounce_module_.ResetTime(0.3, 0.0,
                                                   config_.main_loop_time());
  right_right_lane_exist_debounce_module_.ResetTime(0.3, 0.0,
                                                    config_.main_loop_time());
  return Status::OK();
}

void LaneWidthPredictor::LoadDistanceCalibrationTable(
    const planning::LaneWidthPredictorConfig& lnewde_conf) {
  const auto& distance_table = lnewde_conf.wde_calibration_table();
  ADEBUG << "distance calibration table loaded";
  ADEBUG << "distance calibration table size is "
         << distance_table.calibration_size();
  Interpolation2D::DataType xyz{};
  for (const auto& calibration : distance_table.calibration()) {
    xyz.emplace_back(std::make_tuple(calibration.a2(), calibration.speed(),
                                     calibration.distance()));
  }
  // distance_interpolation_.reset(new Interpolation2D);
  distance_interpolation_ = std::make_unique<Interpolation2D>();
  ACHECK(distance_interpolation_->Init(xyz))
      << "Fail to load distance calibration table";
}

void LaneWidthPredictor::WdePredict(DeciderData* decider_data) {
  ADEBUG << "WdePredict Process!"
         << decider_data->original_lanemarkers.header().seq();
  const auto left_lanemarker =
      decider_data->trans_lanemarkers.front_left_lane_marker();
  const auto right_lanemarker =
      decider_data->trans_lanemarkers.front_right_lane_marker();
  const auto next_left_lanemarker =
      decider_data->trans_lanemarkers.front_next_left_lane_marker().at(0);
  const auto next_right_lanemarker =
      decider_data->trans_lanemarkers.front_next_right_lane_marker().at(0);

  auto change_to_left = decider_data->is_lanechange_to_left;
  auto change_to_right = decider_data->is_lanechange_to_right;
  change_delay_.Deal(std::make_tuple((change_to_left || change_to_right),
                                     change_to_left, change_to_right));
  average_speed_ = decider_data->filter_vehicle_state.vehicle_speed_average;
  // 由两个c2以及当前车速查表得到左右车道线的距离，用以查表求出该距离下的车道宽度
  const double left_lane_pre_dist = distance_interpolation_->Interpolate(
      std::make_pair(abs(left_lanemarker.c2_curvature()), average_speed_));
  const double right_lane_pre_dist = distance_interpolation_->Interpolate(
      std::make_pair(abs(right_lanemarker.c2_curvature()), average_speed_));
  const double next_left_lane_pre_dist = distance_interpolation_->Interpolate(
      std::make_pair(abs(next_left_lanemarker.c2_curvature()), average_speed_));
  const double next_right_lane_pre_dist =
      distance_interpolation_->Interpolate(std::make_pair(
          abs(next_right_lanemarker.c2_curvature()), average_speed_));
  ego_lane_width_predict_distance_ =
      std::max(left_lane_pre_dist, right_lane_pre_dist);
  const double next_left_lane_width_predict_distance =
      std::max(left_lane_pre_dist, next_left_lane_pre_dist);
  const double next_right_lane_width_predict_distance =
      std::max(right_lane_pre_dist, next_right_lane_pre_dist);
  LeftLaneWdePredict(next_left_lanemarker, left_lanemarker,     //  NOLINT
                     decider_data);                             //  NOLINT
  RightLaneWdePredict(right_lanemarker, next_right_lanemarker,  //  NOLINT
                      decider_data);                            //  NOLINT
  EgoLaneWdePredict(left_lanemarker, right_lanemarker, decider_data);

  decider_data->predict_width_distance_ = std::make_tuple(
      ego_lane_width_predict_distance_, next_left_lane_width_predict_distance,
      next_right_lane_width_predict_distance);
}

void LaneWidthPredictor::LeftLaneWdePredict(const LaneMarker& left_lanemarker,
                                            const LaneMarker& right_lanemarker,
                                            DeciderData* decider_data) {
  // 车道宽度方差阈值
  const double next_lane_wide_var_thd =
      config_.lanewidth_predictor_config().next_lane_width_varthd();
  const auto left_lane_a0 = left_lanemarker.c0_position();
  UNUSED(left_lane_a0);
  const auto right_lane_a0 = right_lanemarker.c0_position();
  UNUSED(right_lane_a0);
  const auto left_lane_quality = left_lanemarker.quality();
  const auto right_lane_quality = right_lanemarker.quality();
  auto* left_lanemarker_debug =
      decider_data->lanemarker_lanline_debug->mutable_lanemarker_decider_debug()
          ->mutable_lanes_width_predict_info()
          ->add_lane_width_predict_info();
  left_lanemarker_debug->set_name("left_lane_width_predict");
  const bool left_lane_exist =
      left_left_lane_exist_debounce_module_.DealDebounce(
          left_lanemarker.quality() > good_quality_threshold_);
  const bool right_lane_exist =
      left_right_lane_exist_debounce_module_.DealDebounce(
          right_lanemarker.quality() > good_quality_threshold_);
  // 求车道线reset标志
  const bool lane_reset_bl =
      ((left_lane_left_line_quality_delay_.Deal(left_lane_quality, 4) >=
        good_quality_threshold_) &&
       (left_lane_quality >= good_quality_threshold_) &&
       (left_lane_right_line_quality_delay_.Deal(right_lane_quality, 4) >=
        good_quality_threshold_) &&
       (right_lane_quality >= good_quality_threshold_));
  // 由当前左右车道线的A0求车道线宽度以及宽度的方差是否大于阈值
  // const double lane_wide_sg = std::fmax(left_lane_a0 - right_lane_a0, 0.0);
  const double lane_wide_sg = CalculateLaneWidth(
      vehicle_param_.front_edge_to_center(), left_lanemarker, right_lanemarker,
      &last_left_lane_wdith_, left_lane_exist, right_lane_exist);
  ADEBUG << "left lane reset = " << lane_reset_bl
         << ", now left lane wide sg = " << lane_wide_sg;
  left_lanemarker_debug->set_quality_reset(lane_reset_bl);
  const bool pre_lane_wide_variance =
      (std::pow((lane_wide_sg - left_lane_wide_delay_.GetAverageValue()), 2) >
       next_lane_wide_var_thd);
  left_lane_wide_delay_.Deal(lane_wide_sg);
  for (size_t i = 0; i < left_lane_wide_delay_.GetDataSize(); i++) {
    left_lanemarker_debug->add_lane_width(left_lane_wide_delay_.GetDelay(i));
  }
  left_lanemarker_debug->set_pre_lane_wide_variance(pre_lane_wide_variance);
  // debounce
  // module,判断当前车道线宽度的方差是否不可靠,即在车道线质量好的时候但是车道线
  // 的方差变化较大，说明车道线宽度确实发生了突变。该debounce的作用是只有突变一直发生超过
  // 一定时间才会确定为突变
  const bool is_pre_lane_wde_variance_terrible =
      left_debounce_module_.DealDebounce(lane_reset_bl &&
                                         pre_lane_wide_variance);
  // 当前车道线宽度值是否不可靠 >5m
  const bool is_pre_lane_terrible =
      (lane_wide_sg > 2 * config_.max_lane_half_width() ||
       lane_wide_sg < 2 * config_.min_lane_half_width());
  ADEBUG << "left now pre_lane_wide_variance upper var thd: "
         << pre_lane_wide_variance
         << ", is_pre_lane_wde_variance_terrible after debiunce: "
         << is_pre_lane_wde_variance_terrible
         << ", is_pre_lane_terrible: " << is_pre_lane_terrible;
  // 当前车道线是否不可靠，当车道线宽度发生突变且宽度大于宽度阈值就认为不可靠；如果仅仅是
  // 是车道线宽度大于阈值，但是未判定发生突变，车道线宽度依然认为可靠
  bool predict_lane_wide_terrible_bl = LaneQualityTerrible(
      &left_lane_good_path_, is_pre_lane_wde_variance_terrible,
      is_pre_lane_terrible);
  ADEBUG << "now predict_lane_wide_terrible_bl: "
         << predict_lane_wide_terrible_bl
         << ", left lane good path: " << left_lane_good_path_;
  auto* quality_terriblr_state = left_lanemarker_debug->mutable_normal_state();
  quality_terriblr_state->set_state_name("LaneQualityTerrible_state");
  quality_terriblr_state->set_now_state(left_lane_good_path_ ? 1 : 0);
  quality_terriblr_state->mutable_input_one()->set_name(
      "is_pre_lane_wde_variance_terrible");
  quality_terriblr_state->mutable_input_one()->set_value_b(
      is_pre_lane_wde_variance_terrible);
  quality_terriblr_state->mutable_input_two()->set_name("is_pre_lane_terrible");
  quality_terriblr_state->mutable_input_two()->set_value_b(
      is_pre_lane_terrible);
  quality_terriblr_state->mutable_output_one()->set_name(
      "predict_lane_wide_terrible_bl");
  quality_terriblr_state->mutable_output_one()->set_value_b(
      predict_lane_wide_terrible_bl);
  // 车道线预测是否有效，分三个状态0:init,
  // 2:coast和1:update。正常情况(质量好，宽度可靠)下是
  // update状态，有一个车道线质量不好且处于换道状态下就返回init状态；如果仅仅质量不好或者宽度不可靠
  // 就跳转到coast状态；coast状态下，如果超过一定时间或者质量不好且处于换道状态就跳到init，如果
  // 变好了就返回到update状态
  int now_lane_wide_predict_valid = NextLaneWidePredictValid(
      &left_lane_wide_predict_fir_state_, &left_lane_wide_predict_fir_timer_,
      predict_lane_wide_terrible_bl, left_lane_quality, right_lane_quality,
      std::get<0>(change_delay_.GetDelay(0)));
  ADEBUG << "now_lane_wide_predict_valid: " << now_lane_wide_predict_valid
         << ", left_lane_wide_predict_fir_timer: "
         << left_lane_wide_predict_fir_timer_;
  auto* valid_state_info = left_lanemarker_debug->mutable_valid_state();
  valid_state_info->set_state_name("valid_state");
  valid_state_info->set_pre_now_state(left_lane_wide_predict_fir_state_);
  valid_state_info->set_time(left_lane_wide_predict_fir_timer_);
  // 只有当valid从update跳到coast状态时才会更新一次left_lane_wide_last_input_sg_,让其等于宽度的
  // 历史值，否则不更新。所以在coast状态下车道宽度会一直使用最后一帧的update的车道宽度
  if ((now_lane_wide_predict_valid == 2) &&
      (left_lane_wide_predict_valid_delay_.GetDelay(0) == 1)) {
    left_lane_wide_last_input_sg_ = left_lane_wide_delay_.GetDelay(1);
  }
  left_lane_wide_predict_valid_delay_.Deal(now_lane_wide_predict_valid);
  ADEBUG << "left_lane_wide_last_input_sg: " << left_lane_wide_last_input_sg_;
  // 预测的车道线宽度，车道宽度有效状态为init时，宽度值为0；update状态时宽度为当前的车道宽度；状态
  // 为coast状态时使用历史车道宽度。
  double lane_wide_predict_sg = NextLaneWidePredictValue(
      &left_lane_wide_predict_sec_state_, lane_wide_sg,
      now_lane_wide_predict_valid, left_lane_wide_last_input_sg_);
  auto* valus_state_info = left_lanemarker_debug->mutable_value_state();
  valus_state_info->set_state_name("valus_state_info");
  valus_state_info->set_pre_now_state(left_lane_wide_predict_sec_state_);
  valus_state_info->mutable_input_one()->set_name(
      "left_lane_wide_last_input_sg");
  valus_state_info->mutable_input_one()->set_value_d(
      left_lane_wide_last_input_sg_);
  // 输出
  decider_data->lane_width_valid_out.next_leftlane_widthpredict_valid =
      now_lane_wide_predict_valid;
  decider_data->lane_width_valid_out.next_leftlane_widthpredict_sg =
      lane_wide_predict_sg;
  ADEBUG << "next_leftlane_widthpredict_valid: " << now_lane_wide_predict_valid
         << ", next_leftlane_widthpredict_sg: " << lane_wide_predict_sg;
  left_lanemarker_debug->set_lane_width_predict_valid(
      now_lane_wide_predict_valid);
  left_lanemarker_debug->set_lane_width_predict_value(lane_wide_predict_sg);
}

void LaneWidthPredictor::RightLaneWdePredict(const LaneMarker& left_lanemarker,
                                             const LaneMarker& right_lanemarker,
                                             DeciderData* decider_data) {
  // 车道宽度方差阈值
  const double next_lane_wide_var_thd =
      config_.lanewidth_predictor_config().next_lane_width_varthd();
  const auto left_lane_a0 = left_lanemarker.c0_position();
  UNUSED(left_lane_a0);
  const auto right_lane_a0 = right_lanemarker.c0_position();
  UNUSED(right_lane_a0);
  const auto left_lane_quality = left_lanemarker.quality();
  const auto right_lane_quality = right_lanemarker.quality();
  auto* right_lanemarker_debug =
      decider_data->lanemarker_lanline_debug->mutable_lanemarker_decider_debug()
          ->mutable_lanes_width_predict_info()
          ->add_lane_width_predict_info();
  right_lanemarker_debug->set_name("right_lane_width_predict");
  const bool left_lane_exist =
      right_left_lane_exist_debounce_module_.DealDebounce(
          left_lanemarker.quality() > good_quality_threshold_);
  const bool right_lane_exist =
      right_right_lane_exist_debounce_module_.DealDebounce(
          right_lanemarker.quality() > good_quality_threshold_);
  // 求车道线reset标志
  const bool lane_reset_bl =
      ((right_lane_left_line_quality_delay_.Deal(left_lane_quality, 4) >=
        good_quality_threshold_) &&
       (left_lane_quality >= good_quality_threshold_) &&
       (right_lane_right_line_quality_delay_.Deal(right_lane_quality, 4) >=
        good_quality_threshold_) &&
       (right_lane_quality >= good_quality_threshold_));
  // 由当前左右车道线的A0求车道线宽度以及宽度的方差是否大于阈值
  // const double lane_wide_sg = std::fmax(left_lane_a0 - right_lane_a0, 0.0);
  const double lane_wide_sg = CalculateLaneWidth(
      vehicle_param_.front_edge_to_center(), left_lanemarker, right_lanemarker,
      &last_right_lane_wdith_, left_lane_exist, right_lane_exist);

  ADEBUG << "right lane reset = " << lane_reset_bl
         << ", now right lane wide sg = " << lane_wide_sg;
  right_lanemarker_debug->set_quality_reset(lane_reset_bl);
  const bool pre_lane_wide_variance =
      (std::pow((lane_wide_sg - right_lane_wide_delay_.GetAverageValue()), 2) >
       next_lane_wide_var_thd);
  right_lane_wide_delay_.Deal(lane_wide_sg);
  for (int i = 0; i < kMagicNumber5; i++) {
    right_lanemarker_debug->add_lane_width(right_lane_wide_delay_.GetDelay(i));
  }
  right_lanemarker_debug->set_pre_lane_wide_variance(pre_lane_wide_variance);
  // debounce module,判断当前车道线宽度的方差是否不可靠
  const bool is_pre_lane_wde_variance_terrible =
      right_debounce_module_.DealDebounce(lane_reset_bl &&
                                          pre_lane_wide_variance);
  // 当前车道宽度值是否不可靠
  const bool is_pre_lane_terrible =
      (lane_wide_sg > 2 * config_.max_lane_half_width() ||
       lane_wide_sg < 2 * config_.min_lane_half_width());
  ADEBUG << "right now pre_lane_wide_variance upper var thd: "
         << pre_lane_wide_variance
         << ", is_pre_lane_wde_variance_terrible after debiunce: "
         << is_pre_lane_wde_variance_terrible
         << ", is_pre_lane_terrible: " << is_pre_lane_terrible;
  // 当前车道线是否不可靠
  bool predict_lane_wide_terrible_bl = LaneQualityTerrible(
      &right_lane_good_path_, is_pre_lane_wde_variance_terrible,
      is_pre_lane_terrible);
  ADEBUG << "right now predict_lane_wide_terrible_bl: "
         << predict_lane_wide_terrible_bl
         << ", right lane good path: " << right_lane_good_path_;
  auto* quality_terriblr_state = right_lanemarker_debug->mutable_normal_state();
  quality_terriblr_state->set_state_name("LaneQualityTerrible_state");
  quality_terriblr_state->set_now_state(right_lane_good_path_ ? 1 : 0);
  quality_terriblr_state->mutable_input_one()->set_name(
      "is_pre_lane_wde_variance_terrible");
  quality_terriblr_state->mutable_input_one()->set_value_b(
      is_pre_lane_wde_variance_terrible);
  quality_terriblr_state->mutable_input_two()->set_name("is_pre_lane_terrible");
  quality_terriblr_state->mutable_input_two()->set_value_b(
      is_pre_lane_terrible);
  quality_terriblr_state->mutable_output_one()->set_name(
      "predict_lane_wide_terrible_bl");
  quality_terriblr_state->mutable_output_one()->set_value_b(
      predict_lane_wide_terrible_bl);
  // 车道线预测是否有效
  int now_lane_wide_predict_valid = NextLaneWidePredictValid(
      &right_lane_wide_predict_fir_state_, &right_lane_wide_predict_fir_timer_,
      predict_lane_wide_terrible_bl, left_lane_quality, right_lane_quality,
      std::get<0>(change_delay_.GetDelay(0)));
  ADEBUG << "right now_lane_wide_predict_valid: " << now_lane_wide_predict_valid
         << ", right_lane_wide_predict_fir_timer: "
         << right_lane_wide_predict_fir_timer_;
  auto* valid_state_info = right_lanemarker_debug->mutable_valid_state();
  valid_state_info->set_state_name("valid_state");
  valid_state_info->set_pre_now_state(right_lane_wide_predict_fir_state_);
  valid_state_info->set_time(right_lane_wide_predict_fir_timer_);
  if ((now_lane_wide_predict_valid == 2) &&
      (right_lane_wide_predict_valid_delay_.GetDelay(0) == 1)) {
    right_lane_wide_last_input_sg_ = right_lane_wide_delay_.GetDelay(1);
  }
  right_lane_wide_predict_valid_delay_.Deal(now_lane_wide_predict_valid);
  ADEBUG << "right last_lane_wide_sg: " << right_lane_wide_last_input_sg_;
  // 预测的车道线宽度
  double lane_wide_predict_sg = NextLaneWidePredictValue(
      &right_lane_wide_predict_sec_state_, lane_wide_sg,
      now_lane_wide_predict_valid, right_lane_wide_last_input_sg_);
  auto* valus_state_info = right_lanemarker_debug->mutable_value_state();
  valus_state_info->set_state_name("valus_state_info");
  valus_state_info->set_pre_now_state(right_lane_wide_predict_sec_state_);
  valus_state_info->mutable_input_one()->set_name(
      "left_lane_wide_last_input_sg");
  valus_state_info->mutable_input_one()->set_value_d(
      right_lane_wide_last_input_sg_);
  // 输出
  decider_data->lane_width_valid_out.next_rightlane_widthpredict_valid =
      now_lane_wide_predict_valid;
  decider_data->lane_width_valid_out.next_rightlane_widthpredict_sg =
      lane_wide_predict_sg;
  ADEBUG << "next_rightlane_widthpredict_valid: " << now_lane_wide_predict_valid
         << ", next_rightlane_widthpredict_sg: " << lane_wide_predict_sg;
  right_lanemarker_debug->set_lane_width_predict_valid(
      now_lane_wide_predict_valid);
  right_lanemarker_debug->set_lane_width_predict_value(lane_wide_predict_sg);
}

void LaneWidthPredictor::EgoLaneWdePredict(const LaneMarker& left_lanemarker,
                                           const LaneMarker& right_lanemarker,
                                           DeciderData* decider_data) {
  const double ego_lane_wide_var_thd =
      config_.lanewidth_predictor_config().pre_ego_lanewd_varthd();
  const double lane_width_delta_distance =
      config_.lanewidth_predictor_config().lane_width_delta_distance();
  const bool left_lane_exist =
      ego_left_lane_exist_debounce_module_.DealDebounce(
          left_lanemarker.quality() > good_quality_threshold_);
  const bool right_lane_exist =
      ego_right_lane_exist_debounce_module_.DealDebounce(
          right_lanemarker.quality() > good_quality_threshold_);
  auto* lane_wide_predict = &decider_data->lane_width_valid_out;
  auto* ego_lanemarker_debug =
      decider_data->lanemarker_lanline_debug->mutable_lanemarker_decider_debug()
          ->mutable_lanes_width_predict_info()
          ->add_lane_width_predict_info();
  ego_lanemarker_debug->set_name("ego_lane_width_predict");
  const auto left_lane_quality = left_lanemarker.quality();
  const auto right_lane_quality = right_lanemarker.quality();
  // 由两个c2以及当前车速查表得到左右车道线的距离，用以查表求出该距离下的车道宽度
  double max_lane_pre_dist = ego_lane_width_predict_distance_;
  const double left_lane_y =
      CalculateLanemarkerY(max_lane_pre_dist, left_lanemarker);
  const double right_lane_y =
      CalculateLanemarkerY(max_lane_pre_dist, right_lanemarker);
  // const double lane_wide_delta = std::fmax(left_lane_y - right_lane_y, 0.0);
  const double lane_wide_delta = CalculateLaneWidth(
      (average_speed_ * 0.8 + vehicle_param_.front_edge_to_center()),
      left_lanemarker, right_lanemarker, &last_lane_wide_delta_,
      left_lane_exist, right_lane_exist);

  const double lane_wide_pre_variance = std::pow(
      lane_wide_delta - ego_lane_wide_delta_delay_.GetAverageValue(), 2);
  ego_lane_wide_delta_delay_.Deal(lane_wide_delta);
  // 当车道宽度变化的方差超过一定的阈值且不是处于换道状态，那么车道宽度的方差就有问题
  const bool pre_lane_wide_variance_terrible =
      ((lane_wide_pre_variance > ego_lane_wide_var_thd) &&
       (!std::get<0>(change_delay_.GetDelay(0))));
  ADEBUG << "lane wide delta: " << lane_wide_delta
         << ", left_lane_y: " << left_lane_y
         << ", right_lane_y: " << right_lane_y
         << ", lane_wide_pre_variance: " << lane_wide_pre_variance
         << ", pre_lane_wide_variance_terrible: "
         << pre_lane_wide_variance_terrible;
  for (int i = 0; i < kMagicNumber5; i++) {
    ego_lanemarker_debug->add_dis_lane_width(
        ego_lane_wide_delta_delay_.GetDelay(i));
  }
  ego_lanemarker_debug->set_pre_lane_wide_variance(
      pre_lane_wide_variance_terrible);
  // 由当前和历史的车道线质量判断lane reliable
  const bool lane_reset =
      (ego_lane_left_line_quality_delay_.Deal(left_lane_quality, 4) >
       good_quality_threshold_) &&
      (left_lane_quality > good_quality_threshold_) &&
      (ego_lane_right_line_quality_delay_.Deal(right_lane_quality, 4) >
       good_quality_threshold_) &&
      (right_lane_quality > good_quality_threshold_);
  ego_lanemarker_debug->set_quality_reset(lane_reset);
  // 车道线预测是否不可靠,车道线质量好的情况下如果宽度有问题那预测就不可靠
  const bool is_prelane_wide_change_terrible =
      pre_lane_wide_variance_terrible && lane_reset;
  // 车道线宽度是否正常，当前帧大于历史帧的时候才会更新上一时刻的车道宽度输入
  // ego_lane_wide_last_input_
  // const double lane_wide_sg =
  //     left_lanemarker.c0_position() - right_lanemarker.c0_position();
  const double lane_wide_sg = CalculateLaneWidth(
      vehicle_param_.front_edge_to_center(), left_lanemarker, right_lanemarker,
      &last_ego_lane_wdith_, left_lane_exist, right_lane_exist);
  const bool up_is_prelane_wide_change_terrible =
      (!is_prelane_wide_change_terrible_.Deal(
           is_prelane_wide_change_terrible) &&
       is_prelane_wide_change_terrible);
  ego_lane_wide_delay_.Deal(lane_wide_sg);
  if (up_is_prelane_wide_change_terrible) {
    ego_lane_wide_last_input_ = ego_lane_wide_delay_.GetDelay(1);
  }
  ADEBUG << "lane_reset: " << lane_reset
         << ", is_prelane_wide_change_terrible: "
         << is_prelane_wide_change_terrible
         << ", lane_wide_sg: " << lane_wide_sg
         << ", up_is_prelane_wide_change_terrible: "
         << up_is_prelane_wide_change_terrible
         << ", ego_lane_wide_last_input: " << ego_lane_wide_last_input_;
  // ego lane wide
  // lock,当is_prelane_wide_change_terrible发生突变时，lane_wide_lock的值就是上
  // 一时刻的车道宽度，如果质量很差就是当前车道的宽度。
  double lane_wide_lock = EgoLaneWideLock(
      &ego_lane_lock_state_, is_prelane_wide_change_terrible,
      ego_lane_wide_last_input_, ego_lane_wide_delay_.GetDelay(1),
      left_lane_quality, right_lane_quality);
  const bool is_lane_wide_normal =
      (abs(lane_wide_lock - lane_wide_sg) < lane_width_delta_distance);
  ADEBUG << "is_lane_wide_normal: " << is_lane_wide_normal
         << ", lane_wide_lock: " << lane_wide_lock;
  ego_lanemarker_debug->add_lane_width(lane_wide_sg);
  ego_lanemarker_debug->add_lane_width(ego_lane_wide_delay_.GetDelay(1));
  auto* lane_lock_state_info = ego_lanemarker_debug->mutable_lock_state();
  lane_lock_state_info->set_state_name("lane_lock_state_info");
  lane_lock_state_info->set_pre_now_state(ego_lane_lock_state_);
  lane_lock_state_info->mutable_input_one()->set_name(
      "is_prelane_wide_change_terrible");
  lane_lock_state_info->mutable_input_one()->set_value_b(
      is_prelane_wide_change_terrible);
  lane_lock_state_info->mutable_input_two()->set_name(
      "ego_lane_wide_last_input");
  lane_lock_state_info->mutable_input_two()->set_value_d(
      ego_lane_wide_last_input_);
  lane_lock_state_info->mutable_output_one()->set_name("lane_wide_lock");
  lane_lock_state_info->mutable_output_one()->set_value_d(lane_wide_lock);
  // ego lane wide is normal
  // monitor，车道线预测不可靠且宽度不正常时，则车道线宽度不可靠
  bool is_lane_wide_terrible = false;
  if (ego_lanewde_is_nomal_monitor_state_) {
    if (is_prelane_wide_change_terrible && (!is_lane_wide_normal)) {
      ego_lanewde_is_nomal_monitor_state_ = false;  // terrible path
      is_lane_wide_terrible = true;
    } else {
      is_lane_wide_terrible = false;
    }
  } else {
    if (is_lane_wide_normal) {
      ego_lanewde_is_nomal_monitor_state_ = true;  // good path
      is_lane_wide_terrible = false;
    } else {
      is_lane_wide_terrible = true;
    }
  }
  ADEBUG << ", is_lane_wide_terrible: " << is_lane_wide_terrible
         << ", ego_lanewde_is_nomal_monitor_state: "
         << ego_lanewde_is_nomal_monitor_state_;
  auto* normal_state_info = ego_lanemarker_debug->mutable_normal_state();
  normal_state_info->set_state_name("lane_width_normal_state_info");
  normal_state_info->set_now_state(ego_lanewde_is_nomal_monitor_state_ ? 1 : 0);
  normal_state_info->mutable_input_one()->set_name("is_lane_wide_normal");
  normal_state_info->mutable_input_one()->set_value_b(is_lane_wide_normal);
  ego_lanemarker_debug->set_is_lane_width_terrible(is_lane_wide_terrible);
  // 左侧车道宽度预测是否有效
  int history_next_left_lane_wide_predict_valid =
      next_left_lane_predict_valid_delay_.Deal(
          lane_wide_predict->next_leftlane_widthpredict_valid);
  if (std::get<1>(change_delay_.GetDelay(0)) &&
      !std::get<1>(change_delay_.GetDelay(1))) {
    next_left_lane_wide_predict_valid_record_ =
        history_next_left_lane_wide_predict_valid;
  }
  bool next_left_lane_wide_predict_valid_input =
      ((next_left_lane_wide_predict_valid_record_ == 1) ||
       (next_left_lane_wide_predict_valid_record_ == 2));
  // 右侧车道宽度预测是否有效
  int history_next_right_lane_wide_predict_valid =
      next_right_lane_predict_valid_delay_.Deal(
          lane_wide_predict->next_rightlane_widthpredict_valid);
  if (std::get<2>(change_delay_.GetDelay(0)) &&
      !std::get<2>(change_delay_.GetDelay(1))) {
    next_right_lane_wide_predict_valid_record_ =
        history_next_right_lane_wide_predict_valid;
  }
  bool next_right_lane_wide_predict_valid_input =
      ((next_right_lane_wide_predict_valid_record_ == 1) ||
       (next_right_lane_wide_predict_valid_record_ == 2));
  ADEBUG << "next_left_lane_wide_predict_valid_input: "
         << next_left_lane_wide_predict_valid_input
         << ", next_right_lane_wide_predict_valid_input: "
         << next_right_lane_wide_predict_valid_input;
  /* ego lane wide predict for valid
   * 只要两个质量都好，就是update状态；有一个质量不好，但是处于左换道且左边宽度预测有效，那就是left状态；
   * 有一个质量不好，但是处于右换道且右边宽度预测有效，那就是right状态；有一个质量不好或者terrible就是
   * coast状态；质量不好且处于换道状态下是init状态。
   **/
  int ego_lane_wide_predict_valid = EgoLaneWidePredictValid(
      left_lane_quality, right_lane_quality, is_lane_wide_terrible,
      next_right_lane_wide_predict_valid_input,
      next_left_lane_wide_predict_valid_input);
  ADEBUG << "ego_lane_wide_predict_valid: " << ego_lane_wide_predict_valid
         << ", ego_lane_wide_predict_fir_timer: "
         << ego_lane_wide_predict_fir_timer_
         << ", ego_lane_wide_predict_fir_state: "
         << ego_lane_wide_predict_fir_state_;
  auto* valid_state_info = ego_lanemarker_debug->mutable_valid_state();
  valid_state_info->set_state_name("valid_state_info");
  valid_state_info->set_pre_now_state(ego_lane_wide_predict_fir_state_);
  valid_state_info->set_time(ego_lane_wide_predict_fir_timer_);
  valid_state_info->mutable_input_one()->set_name(
      "next_left_lane_wide_predict_valid_input");
  valid_state_info->mutable_input_one()->set_value_b(
      next_left_lane_wide_predict_valid_input);
  valid_state_info->mutable_input_two()->set_name(
      "next_right_lane_wide_predict_valid_input");
  valid_state_info->mutable_input_two()->set_value_b(
      next_right_lane_wide_predict_valid_input);
  // 上一周期当前车道线宽度值
  ego_lane_predict_valid_delay_.Deal(ego_lane_wide_predict_valid);
  if (ego_lane_wide_predict_valid == 2 &&
      ego_lane_predict_valid_delay_.GetDelay(1) == 1) {
    ego_lane_wide_last_sec_sg_ = ego_lane_wide_delay_.GetDelay(1);
  }
  // 上一周期右侧车道线宽度值
  if (std::get<2>(change_delay_.GetDelay(0)) &&
      !std::get<2>(change_delay_.GetDelay(1))) {
    next_right_lane_wide_predict_sec_sg_input_ =
        next_right_lane_predict_sg_delay_.Deal(
            lane_wide_predict->next_rightlane_widthpredict_sg);
  }
  // 上一周期左侧车道线宽度值
  if (std::get<1>(change_delay_.GetDelay(0)) &&
      !std::get<1>(change_delay_.GetDelay(1))) {
    next_left_lane_wide_predict_sec_sg_input_ =
        next_left_lane_predict_sg_delay_.Deal(
            lane_wide_predict->next_leftlane_widthpredict_sg);
  }
  ADEBUG << "ego_lane_wide_last_sec_sg: " << ego_lane_wide_last_sec_sg_
         << ", next_right_lane_wide_predict_sec_sg_input: "
         << next_right_lane_wide_predict_sec_sg_input_
         << ", next_left_lane_wide_predict_sec_sg_input: "
         << next_left_lane_wide_predict_sec_sg_input_;
  // ego lane wide predict for value：valid = 1,使用当前车道宽度，valid =
  // 2,使用上一时刻当前 车道宽度;valid = 3,使用上一时刻左侧车道宽度；valid =
  // 4,使用上一时刻右侧车道宽度
  double ego_lane_lanewde_predict_sg =
      EgoLaneWidePredictValue(lane_wide_sg, ego_lane_wide_predict_valid);
  auto* value_state_info = ego_lanemarker_debug->mutable_value_state();
  value_state_info->set_state_name("value_state_info");
  value_state_info->set_pre_now_state(ego_lane_wide_predict_sec_state_);
  auto* value_state_input_one = value_state_info->mutable_input_one();
  value_state_input_one->set_name("ego_lane_wide_last_sec_sg");
  value_state_input_one->set_value_d(ego_lane_wide_last_sec_sg_);
  auto* value_state_input_two = value_state_info->mutable_input_two();
  value_state_input_two->set_name("next_left_lane_wide_predict_sec_sg_input");
  value_state_input_two->set_value_d(next_left_lane_wide_predict_sec_sg_input_);
  auto* value_state_input_thr = value_state_info->mutable_input_thr();
  value_state_input_thr->set_name("next_right_lane_wide_predict_sec_sg_input");
  value_state_input_thr->set_value_d(
      next_right_lane_wide_predict_sec_sg_input_);
  // out
  lane_wide_predict->ego_lane_widthpredict_valid = ego_lane_wide_predict_valid;
  lane_wide_predict->ego_lane_widthpredict_sg = ego_lane_lanewde_predict_sg;
  ADEBUG << "ego_lane_widthpredict_valid: " << ego_lane_wide_predict_valid
         << ", ego_lane_widthpredict_sg: " << ego_lane_lanewde_predict_sg;
  ego_lanemarker_debug->set_lane_width_predict_valid(
      ego_lane_wide_predict_valid);
  ego_lanemarker_debug->set_lane_width_predict_value(
      ego_lane_lanewde_predict_sg);
}

double LaneWidthPredictor::EgoLaneWideLock(PredictState* state,
                                           const bool is_lane_change_terrible,
                                           const double lane_wide_last_const,
                                           const double lane_wide_last,
                                           const double left_quality,
                                           const double right_quality) const {
  double lane_wide_lock = 0;
  switch (*state) {
    case PredictState::Init:
      if (left_quality > batter_quality_threshold_ &&
          right_quality > batter_quality_threshold_ &&
          !is_lane_change_terrible) {
        *state = PredictState::Update;
        lane_wide_lock = lane_wide_last;
      } else {
        lane_wide_lock = 0;
      }
      break;
    case PredictState::Update:
      if (is_lane_change_terrible) {
        *state = PredictState::Coast;
        lane_wide_lock = lane_wide_last_const;
      } else if (left_quality < good_quality_threshold_ ||
                 right_quality < good_quality_threshold_) {
        *state = PredictState::Init;
        lane_wide_lock = 0;
      } else {
        lane_wide_lock = lane_wide_last;
      }
      break;
    case PredictState::Coast:
      if (left_quality > batter_quality_threshold_ &&
          right_quality > batter_quality_threshold_ &&
          !is_lane_change_terrible) {
        *state = PredictState::Update;
        lane_wide_lock = lane_wide_last;
      } else if (left_quality < good_quality_threshold_ ||
                 right_quality < good_quality_threshold_) {
        *state = PredictState::Init;
        lane_wide_lock = 0;
      } else {
        lane_wide_lock = lane_wide_last_const;
      }
      break;
    default:
      break;
  }
  return lane_wide_lock;
}

int LaneWidthPredictor::EgoLaneWidePredictValid(
    const double& left_lane_quality, const double& right_lane_quality,
    const bool is_lane_wide_terrible,
    const bool next_right_lane_wide_predict_valid,
    const bool next_left_lane_wide_predict_valid) {
  int ego_lane_wide_predict_valid = 0;
  switch (ego_lane_wide_predict_fir_state_) {
    case PredictState::Init:
      if (left_lane_quality > good_quality_threshold_ &&
          right_lane_quality > good_quality_threshold_) {
        ego_lane_wide_predict_fir_state_ = PredictState::Update;
        ego_lane_wide_predict_fir_timer_ = 0.0;
        ego_lane_wide_predict_valid = 1;
      } else {
        ego_lane_wide_predict_fir_timer_ = 0.0;
        ego_lane_wide_predict_valid = 0;
      }
      break;
    case PredictState::Update:
      if ((left_lane_quality < good_quality_threshold_ ||
           right_lane_quality < good_quality_threshold_) &&
          std::get<2>(change_delay_.GetDelay(0)) &&
          next_right_lane_wide_predict_valid) {
        ego_lane_wide_predict_fir_state_ = PredictState::RgtChange;
        ego_lane_wide_predict_fir_timer_ = 0.0;
        ego_lane_wide_predict_valid = 3;
      } else if ((left_lane_quality < good_quality_threshold_ ||
                  right_lane_quality < good_quality_threshold_) &&
                 std::get<1>(change_delay_.GetDelay(0)) &&
                 next_left_lane_wide_predict_valid) {
        ego_lane_wide_predict_fir_state_ = PredictState::LftChange;
        ego_lane_wide_predict_fir_timer_ = 0.0;
        ego_lane_wide_predict_valid = 4;
      } else if ((left_lane_quality < good_quality_threshold_ ||
                  right_lane_quality < good_quality_threshold_) &&
                 std::get<0>(change_delay_.GetDelay(0))) {
        ego_lane_wide_predict_fir_state_ = PredictState::Init;
        ego_lane_wide_predict_fir_timer_ = 0.0;
        ego_lane_wide_predict_valid = 0;
      } else if (left_lane_quality < good_quality_threshold_ ||
                 right_lane_quality < good_quality_threshold_ ||
                 is_lane_wide_terrible) {
        ego_lane_wide_predict_fir_state_ = PredictState::Coast;
        ego_lane_wide_predict_fir_timer_ = 0.0;
        ego_lane_wide_predict_valid = 2;
      } else {
        ego_lane_wide_predict_valid = 1;
      }
      break;
    case PredictState::Coast:
      if (ego_lane_wide_predict_fir_timer_ > left_lane_wide_coast_time_ ||
          ((left_lane_quality < good_quality_threshold_ ||
            right_lane_quality < good_quality_threshold_) &&
           std::get<0>(change_delay_.GetDelay(0)))) {
        ego_lane_wide_predict_fir_state_ = PredictState::Init;
        ego_lane_wide_predict_fir_timer_ = 0.0;
        ego_lane_wide_predict_valid = 0;
      } else if (left_lane_quality > good_quality_threshold_ &&
                 right_lane_quality > good_quality_threshold_ &&
                 !is_lane_wide_terrible) {
        ego_lane_wide_predict_fir_state_ = PredictState::Update;
        ego_lane_wide_predict_fir_timer_ = 0.0;
        ego_lane_wide_predict_valid = 1;
      } else {
        ego_lane_wide_predict_fir_timer_ += main_loop_time_;
        ego_lane_wide_predict_valid = 2;
      }
      break;
    case PredictState::RgtChange:
      if (left_lane_quality > good_quality_threshold_ &&
          right_lane_quality > good_quality_threshold_) {
        ego_lane_wide_predict_fir_state_ = PredictState::Update;
        ego_lane_wide_predict_fir_timer_ = 0.0;
        ego_lane_wide_predict_valid = 1;
      } else if (ego_lane_wide_predict_fir_timer_ >
                 left_lane_wide_coast_time_) {
        ego_lane_wide_predict_fir_state_ = PredictState::Init;
        ego_lane_wide_predict_fir_timer_ = 0.0;
        ego_lane_wide_predict_valid = 0;
      } else {
        ego_lane_wide_predict_fir_timer_ += main_loop_time_;
        ego_lane_wide_predict_valid = 3;
      }
      break;
    case PredictState::LftChange:
      if (ego_lane_wide_predict_fir_timer_ > left_lane_wide_coast_time_) {
        ego_lane_wide_predict_fir_state_ = PredictState::Init;
        ego_lane_wide_predict_fir_timer_ = 0.0;
        ego_lane_wide_predict_valid = 0;
      } else if (left_lane_quality > good_quality_threshold_ &&
                 right_lane_quality > good_quality_threshold_) {
        ego_lane_wide_predict_fir_state_ = PredictState::Update;
        ego_lane_wide_predict_fir_timer_ = 0.0;
        ego_lane_wide_predict_valid = 1;
      } else {
        ego_lane_wide_predict_fir_timer_ += main_loop_time_;
        ego_lane_wide_predict_valid = 4;
      }
      break;
    default:
      break;
  }
  return ego_lane_wide_predict_valid;
}

double LaneWidthPredictor::EgoLaneWidePredictValue(const double lane_wide,
                                                   const int lane_wide_valid) {
  double ego_lane_lanewde_predict_sg = 0.0;
  switch (ego_lane_wide_predict_sec_state_) {
    case PredictState::Init:
      if (lane_wide_valid == 1) {
        ego_lane_wide_predict_sec_state_ = PredictState::Update;
        ego_lane_lanewde_predict_sg = lane_wide;
      } else {
        ego_lane_lanewde_predict_sg = 0.0;
      }
      break;
    case PredictState::Update:
      if (lane_wide_valid == 2) {
        ego_lane_wide_predict_sec_state_ = PredictState::Coast;
        ego_lane_lanewde_predict_sg = ego_lane_wide_last_sec_sg_;
      } else if (lane_wide_valid == 3) {
        ego_lane_wide_predict_sec_state_ = PredictState::RgtChange;
        ego_lane_lanewde_predict_sg =
            next_right_lane_wide_predict_sec_sg_input_;
      } else if (lane_wide_valid == 4) {
        ego_lane_wide_predict_sec_state_ = PredictState::LftChange;
        ego_lane_lanewde_predict_sg = next_left_lane_wide_predict_sec_sg_input_;
      } else {
        ego_lane_lanewde_predict_sg = lane_wide;
      }
      break;
    case PredictState::Coast:
      if (lane_wide_valid == 0) {
        ego_lane_wide_predict_sec_state_ = PredictState::Init;
        ego_lane_lanewde_predict_sg = 0.0;
      } else if (lane_wide_valid == 1) {
        ego_lane_wide_predict_sec_state_ = PredictState::Update;
        ego_lane_lanewde_predict_sg = lane_wide;
      } else {
        ego_lane_lanewde_predict_sg = ego_lane_wide_last_sec_sg_;
      }
      break;
    case PredictState::RgtChange:
      if (lane_wide_valid == 0) {
        ego_lane_wide_predict_sec_state_ = PredictState::Init;
        ego_lane_lanewde_predict_sg = 0.0;
      } else if (lane_wide_valid == 1) {
        ego_lane_wide_predict_sec_state_ = PredictState::Update;
        ego_lane_lanewde_predict_sg = lane_wide;
      } else {
        ego_lane_lanewde_predict_sg =
            next_right_lane_wide_predict_sec_sg_input_;
      }
      break;
    case PredictState::LftChange:
      if (lane_wide_valid == 0) {
        ego_lane_wide_predict_sec_state_ = PredictState::Init;
        ego_lane_lanewde_predict_sg = 0.0;
      } else if (lane_wide_valid == 1) {
        ego_lane_wide_predict_sec_state_ = PredictState::Update;
        ego_lane_lanewde_predict_sg = lane_wide;
      } else {
        ego_lane_lanewde_predict_sg = next_left_lane_wide_predict_sec_sg_input_;
      }
      break;
    default:
      break;
  }
  return ego_lane_lanewde_predict_sg;
}

bool LaneWidthPredictor::LaneQualityTerrible(
    bool* state, const bool lane_wide_variance_terrible,
    const bool lane_wide_terrible) {
  bool predict_lane_wide_terrible_bl = false;
  if (*state) {
    if (lane_wide_variance_terrible && lane_wide_terrible) {
      *state = false;
      predict_lane_wide_terrible_bl = true;
    } else {
      predict_lane_wide_terrible_bl = false;
    }
  } else {
    if (!lane_wide_terrible) {
      *state = true;
      predict_lane_wide_terrible_bl = false;
    } else {
      predict_lane_wide_terrible_bl = true;
    }
  }
  return predict_lane_wide_terrible_bl;
}

int LaneWidthPredictor::NextLaneWidePredictValid(
    PredictState* state, double* timer, const bool predict_lane_wide_terrible,
    const double left_lane_quality, const double right_lane_quality,
    const bool host_lane_change) const {
  int lane_wide_predict_valid = 0;
  // stateflow: 1_input:predict_lane_wide_terrible_bl,
  // 2_input:right_lane_quality, 3_input:left_lane_quality
  //            4_input:host_change_lane, 5_input:left_lane_wide_coast_time_,
  //            6_input:main_loop_time_
  switch (*state) {
    case PredictState::Init:
      if (right_lane_quality > good_quality_threshold_ &&
          left_lane_quality > good_quality_threshold_ &&
          !predict_lane_wide_terrible) {
        *state = PredictState::Update;
        *timer = 0.0;
        lane_wide_predict_valid = 1;
      } else {
        *timer = 0.0;
        lane_wide_predict_valid = 0;
      }
      break;
    case PredictState::Update:
      if ((right_lane_quality < good_quality_threshold_ ||
           left_lane_quality < good_quality_threshold_) &&
          host_lane_change) {
        *state = PredictState::Init;
        *timer = 0.0;
        lane_wide_predict_valid = 0;
      } else if (predict_lane_wide_terrible ||
                 right_lane_quality < good_quality_threshold_ ||
                 left_lane_quality < good_quality_threshold_) {
        *state = PredictState::Coast;
        *timer = 0.0;
        lane_wide_predict_valid = 2;
      } else {
        *timer = 0.0;
        lane_wide_predict_valid = 1;
      }
      break;
    case PredictState::Coast:
      if (((*timer) > left_lane_wide_coast_time_) ||
          ((right_lane_quality < good_quality_threshold_ ||
            left_lane_quality < good_quality_threshold_) &&
           host_lane_change)) {
        *state = PredictState::Init;
        *timer = 0.0;
        lane_wide_predict_valid = 0;
      } else if (right_lane_quality > good_quality_threshold_ &&
                 left_lane_quality > good_quality_threshold_ &&
                 !predict_lane_wide_terrible) {
        *state = PredictState::Update;
        *timer = 0.0;
        lane_wide_predict_valid = 1;
      } else {
        *timer += main_loop_time_;
        lane_wide_predict_valid = 2;
      }
      break;
    default:
      break;
  }
  return lane_wide_predict_valid;
}

double LaneWidthPredictor::NextLaneWidePredictValue(
    PredictState* state, const double now_lane_wide_value,
    const int lane_wide_predict_valid, const double last_lane_wide_value) {
  double lane_wide_predict_sg = 0.0;
  // stateflow: 1_input:lane_wide_sg, 2_input:lane_wide_predidict_valid
  //            3_input:left_lane_wide_last_input_sg_
  switch (*state) {
    case PredictState::Init:
      if (lane_wide_predict_valid == 1) {
        *state = PredictState::Update;
        lane_wide_predict_sg = now_lane_wide_value;
      } else {
        lane_wide_predict_sg = 0.0;
      }
      break;
    case PredictState::Update:
      if (lane_wide_predict_valid == 2) {
        *state = PredictState::Coast;
        lane_wide_predict_sg = last_lane_wide_value;
      } else {
        lane_wide_predict_sg = now_lane_wide_value;
      }
      break;
    case PredictState::Coast:
      if (lane_wide_predict_valid == 0) {
        *state = PredictState::Init;
        lane_wide_predict_sg = 0.0;
      } else if (lane_wide_predict_valid == 1) {
        *state = PredictState::Update;
        lane_wide_predict_sg = now_lane_wide_value;
      } else {
        lane_wide_predict_sg = last_lane_wide_value;
      }
      break;
    default:
      break;
  }
  return lane_wide_predict_sg;
}

double LaneWidthPredictor::CalculateLanemarkerY(const double distance,
                                                const LaneMarker& lane_marker) {
  double x_2 = distance * distance;
  double x_3 = x_2 * distance;
  return lane_marker.c0_position() + lane_marker.c1_heading_angle() * distance +
         lane_marker.c2_curvature() * x_2 +
         lane_marker.c3_curvature_derivative() * x_3;
}

double LaneWidthPredictor::CalculateLanemarkerHeadingAngle(
    const double distance, const LaneMarker& lane_marker) {

  return lane_marker.c1_heading_angle() +
         2 * lane_marker.c2_curvature() * distance +
         3 * lane_marker.c3_curvature_derivative() * std::pow(distance, 2);
}

bool LaneWidthPredictor::LaneWidthExtraWide(double lane_width) {
  return (ego_lane_wide_delay_.GetDelay(1) > 4.8) &&
         (ego_lane_wide_delay_.GetDelay(2) > 4.8) && (lane_width > 4.8);
}

bool LaneWidthPredictor::LaneWidthUltraNarrow(double lane_width) {
  return (ego_lane_wide_delay_.GetDelay(1) < 2.5) &&
         (ego_lane_wide_delay_.GetDelay(2) < 2.5) && (lane_width < 2.5);
}

double LaneWidthPredictor::CalculateLaneWidth(
    const double distance, const LaneMarker& left_lane_marker,
    const LaneMarker& right_lane_marker, double* last_lane_wdith,
    const bool left_lane_exist, const bool right_lane_exist) {
  double lane_width = 0;
  const double left_lane_y = CalculateLanemarkerY(distance, left_lane_marker);
  const double right_lane_y = CalculateLanemarkerY(distance, right_lane_marker);
  const double left_lane_heading_angle =
      CalculateLanemarkerHeadingAngle(distance, left_lane_marker);
  const double right_lane_heading_angle =
      CalculateLanemarkerHeadingAngle(distance, right_lane_marker);
  bool extra_wide = LaneWidthExtraWide(lane_width);
  bool ultra_narrow = LaneWidthUltraNarrow(lane_width);
  if (left_lane_exist && right_lane_exist) {
    lane_width = left_lane_y * std::cos(std::atan(left_lane_heading_angle)) -
                 right_lane_y * std::cos(std::atan(right_lane_heading_angle));
    if (((lane_width <= 4.8) && (lane_width >= 2.5)) || extra_wide ||
        ultra_narrow) {
      *last_lane_wdith = lane_width;
    }
    return *last_lane_wdith;
  }
  if (left_lane_exist || right_lane_exist) {
    if (*last_lane_wdith > 0.1) {
      lane_width = *last_lane_wdith;
    } else {
      lane_width = config_.default_left_width() + config_.default_right_width();
    }
  } else {
    lane_width = 0.0;
  }
  return lane_width;
}
}  // namespace lanelineprocess
}  // namespace planning
}  // namespace TL
