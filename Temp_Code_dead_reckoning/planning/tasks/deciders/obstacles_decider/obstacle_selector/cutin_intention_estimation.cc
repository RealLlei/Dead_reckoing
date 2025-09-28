/*
 * Copyright (c) 2023 TL Technologies Co., Ltd. All rights reserved.
 */
#include "planning/tasks/deciders/obstacles_decider/obstacle_selector/cutin_intention_estimation.h"
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <limits>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>
#include "common/math/linear_interpolation.h"
#include "common/math/vec2d.h"
#include "common/time/clock.h"
#include "common/util/util.h"
#include "planning/common/obstacle.h"
#include "planning/common/path/discretized_path.h"
#include "planning/common/reference_line_info.h"
#include "planning/reference_line/reference_line.h"
#include "planning/tasks/deciders/obstacles_decider/obstacle_selector/obstacle_selector.h"
#include "planning/tasks/deciders/obstacles_decider/obstacles_decider.h"
#include "proto/common/pnc_point.pb.h"

namespace TL::planning {

namespace {
constexpr size_t kIntentionKeepSize = 9;
};

using TL::common::Clock;

bool CutinIntentionEstimation::CalculateGoalPoint(
    const ReferenceLineInfo& reference_line_info,
    const std::pair<std::array<double, 3UL>, std::array<double, 3UL>>&
        init_state,
    const Obstacle& consider_obstacle, common::SLPoint* const cutin_point,
    common::SLPoint* const regression_point, double* const get_map_info_time) {
  if (cutin_point == nullptr || regression_point == nullptr ||
      get_map_info_time == nullptr) {
    return false;
  }
  bool is_left_obstacle = false;
  bool is_right_obstacle = false;
  if (!IsLeftOrRightObstacle(consider_obstacle, &is_left_obstacle,
                             &is_right_obstacle) ||
      (!is_left_obstacle && !is_right_obstacle)) {
    return false;
  }
  const double start_l = consider_obstacle.PerceptionSLBoundary().start_l();
  const double end_l = consider_obstacle.PerceptionSLBoundary().end_l();
  const double distance_to_ref_line = std::fabs((start_l + end_l) * 0.5);
  const std::vector<double> lat_distance_range = {0.0, 3.5};
  const std::vector<double> time_range = {0.0, 5.0};
  const double cutin_time = common::math::InterpolationOne(
      distance_to_ref_line, lat_distance_range, time_range);
  const double obstacle_speed = consider_obstacle.speed();
  constexpr double kMaxCutinDistance = 300.0;
  constexpr double kMinCutinDistance = 10.0;
  ADEBUG << "obstacle id: " << consider_obstacle.Id()
         << ", cutin time: " << cutin_time
         << ", obstacle_speed: " << obstacle_speed;
  const double cutin_lon_distance =
      std::fmin(std::fmax(cutin_time * obstacle_speed, kMinCutinDistance),
                kMaxCutinDistance);
  const double regression_time = cutin_time;
  const double regression_lon_distance =
      std::fmin(std::fmax(regression_time * obstacle_speed, kMinCutinDistance),
                kMaxCutinDistance);
  const double cutin_goal_s =
      std::get<0>(init_state.first) + cutin_lon_distance;
  const double regression_goal_s =
      std::get<0>(init_state.first) + regression_lon_distance;
  double lane_left_width = 1.75;
  double lane_right_width = 1.75;
  double right_neighbor_lane_width = 3.5;
  double left_neighbor_lane_width = 3.5;
  const double current_time = Clock::NowInSeconds();
  if (!ObstacleSelector::GetMapInfo(
          regression_lon_distance, reference_line_info, &lane_left_width,
          &lane_right_width, &left_neighbor_lane_width,
          &right_neighbor_lane_width)) {
    ADEBUG << "can't GetMapInfo.";
    return false;
  }
  *get_map_info_time = (Clock::NowInSeconds() - current_time) * 1000;
  if (is_left_obstacle) {
    regression_point->set_l(lane_left_width + 0.5 * left_neighbor_lane_width);
    regression_point->set_s(regression_goal_s);
  }
  if (is_right_obstacle) {
    regression_point->set_l(-lane_right_width -
                            0.5 * right_neighbor_lane_width);
    regression_point->set_s(regression_goal_s);
  }
  cutin_point->set_l(0.0);
  cutin_point->set_s(cutin_goal_s);
  return true;
}

bool CutinIntentionEstimation::CalculateSampleStartPoint(
    const ReferenceLine& reference_line, const Obstacle& consider_obstacle) {
  sample_start_point_ = consider_obstacle.GetPointAtTime(0.0);
  common::SLPoint start_point_sl;
  if (!reference_line.XYToSL({sample_start_point_.path_point().x(),
                              sample_start_point_.path_point().y()},
                             &start_point_sl)) {
    return false;
  }
  sample_start_point_.mutable_path_point()->set_s(start_point_sl.s());
  sample_start_point_.mutable_path_point()->set_l(start_point_sl.l());
  return true;
}

bool CutinIntentionEstimation::SampleIntentionPath(
    const std::pair<std::array<double, 3UL>, std::array<double, 3UL>>&
        init_state,
    const common::SLPoint& cutin_point, const common::SLPoint& regression_point,
    QuinticPolynomialCurve1d* const cutin_path,
    QuinticPolynomialCurve1d* const regression_path) {
  if (cutin_path == nullptr || regression_path == nullptr) {
    return false;
  }

  const std::array<double, 3>& init_s = init_state.first;
  const std::array<double, 3>& init_l = init_state.second;
  const bool is_left_obstacle = init_l.at(0) > 0.0;
  cutin_path->SetParam(init_l.at(0),
                       is_left_obstacle ? std::fmin(0.0, init_l.at(1))
                                        : std::fmax(0.0, init_l.at(1)),
                       0.0, cutin_point.l(), 0.0, 0.0,
                       cutin_point.s() - init_s.at(0));
  regression_path->SetParam(init_l.at(0),
                            is_left_obstacle ? std::fmax(0.0, init_l.at(1))
                                             : std::fmin(0.0, init_l.at(1)),
                            0.0, regression_point.l(), 0.0, 0.0,
                            regression_point.s() - init_s.at(0));
  return true;
}

bool CutinIntentionEstimation::IsLeftOrRightObstacle(
    const Obstacle& consider_obstacle, bool* const is_left_obstacle,
    bool* const is_right_obstacle) {
  if (is_left_obstacle == nullptr || is_right_obstacle == nullptr) {
    return false;
  }
  const double start_l = consider_obstacle.PerceptionSLBoundary().start_l();
  const double end_l = consider_obstacle.PerceptionSLBoundary().end_l();
  *is_left_obstacle = start_l > 0.0 && end_l > 0.0;
  *is_right_obstacle = start_l < 0.0 && end_l < 0.0;
  return true;
}

bool CutinIntentionEstimation::CalculateInitState(
    const ReferenceLineInfo& reference_line_info,
    const Obstacle& consider_obstacle,
    std::pair<std::array<double, 3UL>, std::array<double, 3UL>>* const
        init_state) {
  if (init_state == nullptr) {
    return false;
  }
  if (!CalculateSampleStartPoint(reference_line_info.reference_line(),
                                 consider_obstacle)) {
    return false;
  }
  const common::PathPoint& matched_point =
      reference_line_info.reference_line()
          .GetReferencePoint(sample_start_point_.path_point().s())
          .ToPathPoint(sample_start_point_.path_point().s());
  ObstaclesDecider::ComputeInitFrenetState(matched_point, sample_start_point_,
                                           &init_state->first,
                                           &init_state->second);
  return true;
}

void CutinIntentionEstimation::Debug(
    const DiscretizedPath& path_data, const std::string& path_label,
    ReferenceLineInfo* const reference_line_info) {
  if (reference_line_info == nullptr) {
    return;
  }
  auto* mutable_path =
      reference_line_info->mutable_debug()->mutable_planning_data()->add_path();
  mutable_path->set_name(path_label);
  int size = static_cast<int>(path_data.size());
  mutable_path->mutable_path_point()->Reserve(size);
  for (const auto& path_point : path_data) {
    auto* point = mutable_path->add_path_point();
    point->set_x(path_point.x());
    point->set_y(path_point.y());
  }
}

void CutinIntentionEstimation::CutinIntentionDebug(
    const DiscretizedPath& history_cutin_path,
    const DiscretizedPath& history_regression_path,
    const std::string& obstacle_id, const double prob,
    const double cutin_prior_prob, const double cutin_likelihood,
    const double regression_likelihood,
    ReferenceLineInfo* const reference_line_info) {
  if (reference_line_info == nullptr) {
    return;
  }
  auto* cutin_intention_debug = reference_line_info->mutable_debug()
                                    ->mutable_cutin_intention_estimation();
  auto* debug_cutin_path = cutin_intention_debug->add_paths();
  for (const auto& path_point : history_cutin_path) {
    auto* debug_point = debug_cutin_path->add_point();
    debug_point->set_x(path_point.x());
    debug_point->set_y(path_point.y());
  }
  auto* debug_regression_path = cutin_intention_debug->add_paths();
  for (const auto& path_point : history_regression_path) {
    auto* debug_point = debug_regression_path->add_point();
    debug_point->set_x(path_point.x());
    debug_point->set_y(path_point.y());
  }
  auto* cutin_info = cutin_intention_debug->add_cutin_info();
  cutin_info->set_obs_id(obstacle_id);
  cutin_info->set_cutin_prob(prob);
  cutin_info->set_cutin_prior_prob(cutin_prior_prob);
  cutin_info->set_cutin_likelihood(cutin_likelihood);
  cutin_info->set_regression_likelihood(regression_likelihood);
}

bool CutinIntentionEstimation::GetDiscretizedPath(
    const QuinticPolynomialCurve1d& cutin_path,
    const QuinticPolynomialCurve1d& regression_path,
    const ReferenceLine& reference_line, const Obstacle& obstacle,
    DiscretizedPath* cutin_discretized_path,
    DiscretizedPath* regression_discretized_path) {
  if (cutin_discretized_path == nullptr ||
      regression_discretized_path == nullptr) {
    return false;
  }
  if (obstacle.speed() < 0.0) {
    return false;
  }
  cutin_discretized_path->clear();
  regression_discretized_path->clear();
  constexpr double kMaxAcc = 9.8 * 1.8;                   // 1.8g的加速度
  constexpr double kMinAcc = -9.8 * 0.5;                  // 0.5g的减速度
  const double compare_time = kIntentionKeepSize / 10.0;  // planning为10hz
  const double start_s =
      std::fmax(0.0, obstacle.speed() * compare_time +
                         0.5 * kMinAcc * compare_time * compare_time);
  double end_s = obstacle.speed() * compare_time +
                 0.5 * kMaxAcc * compare_time * compare_time;
  constexpr double kSampleResolution = 0.5;
  const int start_index = static_cast<int>(start_s / kSampleResolution);
  const int end_index = static_cast<int>(end_s / kSampleResolution);
  cutin_discretized_path->reserve(end_index - start_index + 1);
  regression_discretized_path->reserve(end_index - start_index + 1);
  common::SLPoint sl_point;
  common::math::Vec2d xy_point;
  for (int i = start_index; i <= end_index; ++i) {
    const double delta_s = kSampleResolution * i;
    const double s = delta_s + sample_start_point_.path_point().s();

    // cutin sample
    const double cutin_l = cutin_path.Evaluate(0, delta_s);
    sl_point.set_s(s);
    sl_point.set_l(cutin_l);
    if (!reference_line.SLToXY(sl_point, &xy_point)) {
      return false;
    }
    cutin_discretized_path->emplace_back();
    auto& cutin_path_point = cutin_discretized_path->back();
    cutin_path_point.set_s(s);
    cutin_path_point.set_l(cutin_l);
    cutin_path_point.set_x(xy_point.x());
    cutin_path_point.set_y(xy_point.y());
    if (cutin_discretized_path->size() > 1) {
      common::math::Vec2d vec = {
          xy_point.x() - cutin_discretized_path->back().x(),
          xy_point.y() - cutin_discretized_path->back().y()};
      cutin_path_point.set_theta(vec.Angle());
    }

    // regression sample
    const double regression_l = regression_path.Evaluate(0, delta_s);
    sl_point.set_l(regression_l);
    if (!reference_line.SLToXY(sl_point, &xy_point)) {
      return false;
    }
    regression_discretized_path->emplace_back();
    auto& regression_path_point = regression_discretized_path->back();
    regression_path_point.set_s(s);
    regression_path_point.set_l(regression_l);
    regression_path_point.set_x(xy_point.x());
    regression_path_point.set_y(xy_point.y());
    if (regression_discretized_path->size() > 1) {
      common::math::Vec2d vec = {
          xy_point.x() - regression_discretized_path->back().x(),
          xy_point.y() - regression_discretized_path->back().y()};
      regression_path_point.set_theta(vec.Angle());
    }
  }
  if (cutin_discretized_path->size() < 2 ||
      regression_discretized_path->size() < 2) {
    return false;
  }
  cutin_discretized_path->at(0).set_theta(
      cutin_discretized_path->at(1).theta());
  regression_discretized_path->at(0).set_theta(
      regression_discretized_path->at(1).theta());
  return true;
}

void CutinIntentionEstimation::NaiveBayesProcessObstacle(
    const Obstacle& consider_obstacle,
    std::unordered_map<int, double>* const obstacle_cutin_intentions,
    ReferenceLineInfo* const reference_line_info) {
  if (obstacle_cutin_intentions == nullptr || reference_line_info == nullptr) {
    AERROR << "obstacle_cutin_intentions == nullptr || reference_line_info == "
              "nullptr";
    return;
  }
  auto* time_delay = reference_line_info->mutable_debug()
                         ->mutable_cutin_intention_estimation()
                         ->add_time_delay();
  time_delay->set_obs_id(consider_obstacle.Id());
  double start_time = Clock::NowInSeconds();
  double current_time = start_time;
  const int perception_id = consider_obstacle.PerceptionId();
  std::pair<std::array<double, 3UL>, std::array<double, 3UL>> init_state;
  if (!CalculateInitState(*reference_line_info, consider_obstacle,
                          &init_state)) {
    history_path_queue_.erase(perception_id);
    return;
  }
  const double init_time_delay = (Clock::NowInSeconds() - current_time) * 1000;
  time_delay->set_calculate_init_state(init_time_delay);
  current_time = Clock::NowInSeconds();
  ADEBUG << "obs id: " << consider_obstacle.Id()
         << ", start s: " << consider_obstacle.PerceptionSLBoundary().start_s()
         << ", init_state s:" << std::get<0>(init_state.first)
         << ", ds: " << std::get<1>(init_state.first)
         << ", dds: " << std::get<2>(init_state.first)
         << ", l: " << std::get<0>(init_state.second)
         << ", dl: " << std::get<1>(init_state.second)
         << ", ddl: " << std::get<2>(init_state.second);

  common::SLPoint cutin_point;
  common::SLPoint regression_point;
  double get_map_info_time = 0.0;
  if (!CalculateGoalPoint(*reference_line_info, init_state, consider_obstacle,
                          &cutin_point, &regression_point,
                          &get_map_info_time)) {
    history_path_queue_.erase(perception_id);
    return;
  }
  time_delay->set_get_map_info(get_map_info_time);
  const double goal_point_time_delay =
      (Clock::NowInSeconds() - current_time) * 1000;
  time_delay->set_calculate_goal_point(goal_point_time_delay);
  current_time = Clock::NowInSeconds();
  // 当切入线和回归线是一个方向时，返回不处理
  const bool is_left_obstacle =
      (consider_obstacle.PerceptionSLBoundary().start_l() +
       consider_obstacle.PerceptionSLBoundary().end_l()) *
          0.5 >
      0.0;
  if ((is_left_obstacle && init_state.second.at(0) >= regression_point.l()) ||
      (!is_left_obstacle && init_state.second.at(0) <= regression_point.l())) {
    history_path_queue_.erase(perception_id);
    ADEBUG << "cutin path regression path same direction, return.";
    return;
  }

  ADEBUG << "obs id: " << consider_obstacle.Id()
         << "cutin s:" << cutin_point.s() << ", l: " << cutin_point.l();
  ADEBUG << "obs id: " << consider_obstacle.Id()
         << "regression s:" << regression_point.s()
         << ", l: " << regression_point.l();
  QuinticPolynomialCurve1d cutin_path;
  QuinticPolynomialCurve1d regression_path;
  if (!SampleIntentionPath(init_state, cutin_point, regression_point,
                           &cutin_path, &regression_path)) {
    history_path_queue_.erase(perception_id);
    return;
  }
  const double sample_path_time_delay =
      (Clock::NowInSeconds() - current_time) * 1000;
  time_delay->set_sample_intention_path(sample_path_time_delay);
  current_time = Clock::NowInSeconds();
  history_path_queue_[perception_id].first.emplace();
  history_path_queue_[perception_id].second.emplace();
  if (!GetDiscretizedPath(
          cutin_path, regression_path, reference_line_info->reference_line(),
          consider_obstacle, &history_path_queue_[perception_id].first.back(),
          &history_path_queue_[perception_id].second.back())) {
    return;
  }
  const double get_discretized_path_time_delay =
      (Clock::NowInSeconds() - current_time) * 1000;
  time_delay->set_get_discretized_path(get_discretized_path_time_delay);
  current_time = Clock::NowInSeconds();

  if (history_path_queue_[perception_id].first.size() < kIntentionKeepSize) {
    return;
  }
  const auto& history_cutin_path =
      history_path_queue_[perception_id].first.front();
  const auto& history_regression_path =
      history_path_queue_[perception_id].second.front();

  // 此处为临时debug代码，可视化做好后删除
  // if (perception_id == 61) {
  //   AERROR << "now is draw obs:" << consider_obstacle.Id();
  //   Debug(history_cutin_path, "astar_guide_line", reference_line_info);
  //   Debug(history_regression_path, "Planning PathData", reference_line_info);
  // }

  const double cutin_prior_prob =
      CalculatePriorProbability(*reference_line_info, consider_obstacle);
  const double cutin_likelihood =
      CalculateLikelihood(reference_line_info->reference_line(),
                          history_cutin_path, consider_obstacle, true);
  const double regression_likelihood =
      CalculateLikelihood(reference_line_info->reference_line(),
                          history_regression_path, consider_obstacle, false);
  const double cutin_intention = CalculateCutinIntention(
      cutin_prior_prob, cutin_likelihood, regression_likelihood);
  CutinIntentionDebug(history_cutin_path, history_regression_path,
                      consider_obstacle.Id(), cutin_intention, cutin_prior_prob,
                      cutin_likelihood, regression_likelihood,
                      reference_line_info);
  history_path_queue_[perception_id].first.pop();
  history_path_queue_[perception_id].second.pop();
  (*obstacle_cutin_intentions)[perception_id] = cutin_intention;
  ADEBUG << "obstacle id: " << perception_id
         << ", cutin_intention: " << cutin_intention
         << ", cutin_prior_prob: " << cutin_prior_prob
         << ", cutin_likelihood: " << cutin_likelihood
         << ", regression_likelihood: " << regression_likelihood;
  const double cutin_intention_calculate_debug =
      (Clock::NowInSeconds() - current_time) * 1000;
  time_delay->set_cutin_intention_calculate_debug(
      cutin_intention_calculate_debug);
  const double single_obstacle = (Clock::NowInSeconds() - start_time) * 1000;
  time_delay->set_single_obstacle(single_obstacle);
}

std::unordered_map<int, double>
CutinIntentionEstimation::NaiveBayesCutinProbEstimate(
    const std::unordered_set<int>& consider_obstacles,
    ReferenceLineInfo* const reference_line_info) {
  std::unordered_map<int, double> obstacle_cutin_intentions;
  // 输入合理性检查
  if (reference_line_info == nullptr) {
    return obstacle_cutin_intentions;
  }
  // 删除缓存中不需要识别意图的障碍物
  std::unordered_set<int> not_consider_obs_ids;
  for (const auto& his_consider_obs_path_info : history_path_queue_) {
    const auto& his_obs_id = his_consider_obs_path_info.first;
    if (consider_obstacles.find(his_obs_id) == consider_obstacles.end()) {
      not_consider_obs_ids.emplace(his_obs_id);
    }
  }
  for (const auto& obs_id : not_consider_obs_ids) {
    history_path_queue_.erase(obs_id);
  }
  // 处理本帧中需要做意图识别的障碍物，注意按照感知ID处理，避免重复处理同一个障碍物
  for (auto obs_id : consider_obstacles) {
    for (const auto& consider_obstacle :
         reference_line_info->path_decision()->obstacles().Items()) {
      if (consider_obstacle == nullptr) {
        continue;
      }
      if (consider_obstacle->PerceptionId() == obs_id) {
        NaiveBayesProcessObstacle(*consider_obstacle,
                                  &obstacle_cutin_intentions,
                                  reference_line_info);
        break;
      }
    }
  }
  return obstacle_cutin_intentions;
}

double CutinIntentionEstimation::CalculateCutinIntention(
    const double cutin_prior_prob, const double cutin_likelihood,
    const double regression_likelihood) {
  const double cutin_p = (cutin_prior_prob * cutin_likelihood);
  const double regression_p = (1 - cutin_prior_prob) * regression_likelihood;
  return common::util::IsFloatEqual(cutin_p + regression_p, 0.0)
             ? 0.0
             : cutin_p / (cutin_p + regression_p);
}

double CutinIntentionEstimation::CalculatePriorProbability(
    const ReferenceLineInfo& reference_line_info, const Obstacle& obstacle) {
  const double adc_start_s = reference_line_info.AdcSlBoundary().start_s();
  const double adc_end_s = reference_line_info.AdcSlBoundary().end_s();
  const double obs_start_s = obstacle.PerceptionSLBoundary().start_s();
  const double obs_end_s = obstacle.PerceptionSLBoundary().end_s();
  double cutin_prior_probability = 0.5;
  if (obs_end_s < adc_start_s) {
    return 0.0;
  }
  const double ttc =
      common::util::IsFloatEqual(
          reference_line_info.vehicle_state().linear_velocity(),
          obstacle.speed())
          ? std::numeric_limits<double>::max()
          : std::fmax(obs_start_s - adc_end_s, 0.0) /
                (reference_line_info.vehicle_state().linear_velocity() -
                 obstacle.speed());
  const double hwt =
      common::util::IsFloatEqual(
          reference_line_info.vehicle_state().linear_velocity(), 0.0)
          ? std::numeric_limits<double>::max()
          : std::fmax(obs_start_s - adc_end_s, 0.0) /
                reference_line_info.vehicle_state().linear_velocity();
  const std::vector<double> ttc_time = {0.0, 0.5};
  const std::vector<double> hwt_time = {0.0, 0.3};
  const std::vector<double> cutin_ttc_k = {0.0, 1.0};
  const std::vector<double> cutin_hwt_k = {0.0, 1.0};
  const double ttc_k =
      common::math::InterpolationOne(ttc, ttc_time, cutin_ttc_k);
  const double hwt_k =
      common::math::InterpolationOne(hwt, ttc_time, cutin_ttc_k);
  ADEBUG << "obs_id: " << obstacle.Id() << ", ttc_k: " << ttc_k
         << ", hwt_k: " << hwt_k << ", ttc:" << ttc << ", hwt:" << hwt
         << ", obs speed:" << obstacle.speed() << ", adc speed: "
         << reference_line_info.vehicle_state().linear_velocity();
  return std::fmin(ttc_k, hwt_k) * cutin_prior_probability;
}

double CutinIntentionEstimation::CalculateLikelihood(
    const ReferenceLine& reference_line,
    const DiscretizedPath& history_path_data, const Obstacle& obstacle,
    const bool is_cutin_path) {
  double min_distance_square = std::numeric_limits<double>::max();
  int min_distance_index = -1;
  for (size_t i = 0; i < history_path_data.size(); ++i) {
    const auto& path_point = history_path_data.at(i);
    const double distance_square =
        (path_point.x() - sample_start_point_.path_point().x()) *
            (path_point.x() - sample_start_point_.path_point().x()) +
        (path_point.y() - sample_start_point_.path_point().y()) *
            (path_point.y() - sample_start_point_.path_point().y());
    if (distance_square < min_distance_square) {
      min_distance_square = distance_square;
      min_distance_index = static_cast<int>(i);
    }
  }
  if (min_distance_index == -1) {
    return 0.0;
  }
  const auto& min_distance_path_point =
      history_path_data.at(min_distance_index);
  common::SLPoint sl_point;
  if (!reference_line.XYToSL(
          {min_distance_path_point.x(), min_distance_path_point.y()},
          &sl_point)) {
    return 0.0;
  }
  const double obstacle_middle_l = sample_start_point_.path_point().l();
  const bool is_left_line_obstacle = obstacle_middle_l > sl_point.l();
  const bool is_left_ref_obstacle = obstacle_middle_l > 0.0;
  double heading_diff = std::numeric_limits<double>::max();
  double lateral_distance_diff = std::numeric_limits<double>::max();
  if (is_left_ref_obstacle) {
    if (is_cutin_path) {
      if (is_left_line_obstacle) {
        heading_diff = std::fmax(
            obstacle.Perception().theta() - min_distance_path_point.theta(),
            0.0);
        lateral_distance_diff =
            std::fmax(sample_start_point_.path_point().l() - sl_point.l(), 0.0);
      } else {
        heading_diff = 0.0;
        lateral_distance_diff = 0.0;
      }
    } else {
      if (!is_left_line_obstacle) {
        heading_diff = std::fmax(
            min_distance_path_point.theta() - obstacle.Perception().theta(),
            0.0);
        lateral_distance_diff =
            std::fmax(sl_point.l() - sample_start_point_.path_point().l(), 0.0);
      } else {
        heading_diff = 0.0;
        lateral_distance_diff = 0.0;
      }
    }
  } else {
    if (is_cutin_path) {
      if (is_left_line_obstacle) {
        heading_diff = 0.0;
        lateral_distance_diff = 0.0;
      } else {
        heading_diff = std::fmax(
            min_distance_path_point.theta() - obstacle.Perception().theta(),
            0.0);
        lateral_distance_diff =
            std::fmax(sl_point.l() - sample_start_point_.path_point().l(), 0.0);
      }
    } else {
      if (!is_left_line_obstacle) {
        heading_diff = 0.0;
        lateral_distance_diff = 0.0;
      } else {
        heading_diff = std::fmax(
            obstacle.Perception().theta() - min_distance_path_point.theta(),
            0.0);
        lateral_distance_diff =
            std::fmax(sample_start_point_.path_point().l() - sl_point.l(), 0.0);
      }
    }
  }
  heading_diff = heading_diff / M_PI * 180.0;  // 单位换算成角度
  lateral_distance_diff *= 100.0;              // 单位换算成厘米
  const double gaussian_heading =
      (1.0 - NormalDistributionCDF(std::fabs(heading_diff), 0.0, 3)) * 2.0;
  const double gaussian_lateral =
      (1.0 - NormalDistributionCDF(std::fabs(lateral_distance_diff), 0.0, 15)) *
      2.0;
  ADEBUG << "cutin: " << is_cutin_path
         << ", gaussian_heading: " << gaussian_heading
         << ", heading_diff: " << heading_diff
         << ", gaussian_lateral: " << gaussian_lateral
         << ", lateral_distance_diff: " << lateral_distance_diff;
  return gaussian_heading * gaussian_lateral;
}

double CutinIntentionEstimation::NormalDistributionCDF(
    const double x, const double mean, const double standard_deviation) {
  // 均值为 mean，标准差为 standard_deviation 的正态分布中 X <= x 的概率
  if (common::util::IsFloatEqual(0.0, standard_deviation)) {
    return 0.0;
  }
  const double z = (x - mean) / (standard_deviation * std::sqrt(2));
  const double result = 0.5 * (1.0 + std::erf(z));
  return result;
}

}  // namespace TL::planning
