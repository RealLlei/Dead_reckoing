/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/*
 * @file
 */
#include "planning/open_space/coarse_path_generator/hybrid_a_star.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "common/math/double_type.h"
#include "common/math/math_utils.h"
#include "common/math/polygon2d.h"
#include "common/math/vec2d.h"
#include "common/util/util.h"
#include "common/vehicle_state/vehicle_state_provider.h"
#include "planning/common/open_space_info.h"
#include "planning/common/planning_gflags.h"
#include "proto/common/pnc_point.pb.h"

namespace TL {
namespace planning {

using TL::common::Clock;

namespace {
constexpr double kEpsilon = 1.0e-3;
constexpr double kDefaultIgnorableDist = 100;
constexpr double kExtensionExtraDist = 0.3;
constexpr double kDisableOverTimeExtraDistHeight = -1.0;
}  // namespace

HybridAStar::HybridAStar(const WarmStartConfig& warm_start_config)
    : PathGenerator(warm_start_config),
      step_size_(warm_start_config_.step_size()) {
  reed_shepp_generator_ =
      std::make_unique<ReedShepp>(vehicle_param_, warm_start_config_);
}

void HybridAStar::InitParam() {
  const double max_steer_angle_margin =
      path_search_strategy_.is_nns_adjust_senario
          ? 0.0
          : warm_start_config_.max_steer_angle_margin();
  const double max_rs_angle =
      vehicle_param_.max_steer_angle() -
      (path_search_strategy_.space_structure == SpaceStructure::LAT_PARK_LOT
           ? lateral_rs_steer_angle_margin_
           : max_steer_angle_margin);
  const double max_exploration_angle =
      vehicle_param_.max_steer_angle() - max_steer_angle_margin;
  const double max_rs_beta_angle =
      common::VehicleStateProvider::EstimateBetaAngle(max_rs_angle,
                                                      vehicle_param_);
  const double max_exploration_beta_angle =
      common::VehicleStateProvider::EstimateBetaAngle(max_exploration_angle,
                                                      vehicle_param_);
  ADEBUG << "rs radius: "
         << vehicle_param_.wheel_base() /
                (fabs(tan(max_rs_beta_angle)) + kEpsilon);
  ADEBUG << "explore radius: "
         << vehicle_param_.wheel_base() /
                (fabs(tan(max_exploration_beta_angle)) + kEpsilon);

  // lateral_spot_steer_angle_margin
  reed_shepp_generator_->UpdateConfig(max_rs_beta_angle);
  steer_seq_.clear();
  next_node_num_ = 0;
  static constexpr double kEpsilonPhi = 0.0001;
  if (phi_grid_resolution_ > kEpsilonPhi) {
    double max_delta_phi = step_size_ / vehicle_param_.wheel_base() *
                           std::tan(max_exploration_beta_angle);
    auto size_phi =
        static_cast<size_t>(std::ceil(max_delta_phi / phi_grid_resolution_));
    static constexpr size_t kTwo = 2.0;
    size_t size_steer = kTwo * size_phi + 1;
    steer_seq_.resize(size_steer);
    steer_seq_[0] = 0;
    for (size_t i = 1; i <= size_phi; i++) {
      double angle = std::atan(max_delta_phi * static_cast<double>(i) /
                               static_cast<double>(size_phi) *
                               vehicle_param_.wheel_base() / step_size_);
      steer_seq_.at(kTwo * i - 1) = angle;
      steer_seq_.at(kTwo * i) = -1 * angle;
    }
    next_node_num_ = size_steer * kTwo;
  }
  double max_kappa =
      fabs(std::tan(max_rs_beta_angle) / vehicle_param_.wheel_base());
  // same as designed in reeds_shepp_path.cc
  rs_step_size_ = fmax(step_size_ * max_kappa * 0.2, 0.01);
}

bool HybridAStar::AnalyticExpansion(
    const std::shared_ptr<Node3d>& current_node,
    const std::vector<std::pair<common::math::LineSegment2d, double>>&
        obstacles_segments_vec) {
  if (CalculateParkingPrefinishCondition(current_node)) {
    ADEBUG << "finish with search conditions";
    if (end_node_pq_.empty()) {
      end_node_pq_.push(current_node);
    } else {
      std::shared_ptr<Node3d> node = std::make_shared<Node3d>(
          current_node->GetX(), current_node->GetY(), current_node->GetPhi(),
          xy_bounds_, xy_grid_resolution_, phi_grid_resolution_);
      node->SetPre(current_node->GetPreNode());
      double cur_to_end_dis =
          std::sqrt(pow(node->GetX() - end_node_->GetX(), 2) +
                    pow(node->GetY() - end_node_->GetY(), 2));
      double ang_diff =
          fabs(common::math::AngleDiff(node->GetPhi(), end_node_->GetPhi()));
      const auto norm_angle_to_dis = [](double angle) {
        // ratio means 1 rad / 1 meters
        static constexpr double kNormRatio = 10.0;
        return fabs(angle) / M_PI * kNormRatio;
      };
      node->SetPathCost(current_node->GetPathCost() +
                        warm_start_config_.end_pose_diff_penalty() *
                            (cur_to_end_dis + norm_angle_to_dis(ang_diff)));
      end_node_pq_.push(std::move(node));
    }
    return true;
  }

  if (!is_valid_end_pose_) {
    return false;
  }

  std::shared_ptr<ReedSheppPath> reeds_shepp_to_check =
      std::make_shared<ReedSheppPath>();
  if (!reed_shepp_generator_->ShortestRSP(current_node, end_node_,
                                          reeds_shepp_to_check)) {
    ADEBUG << "ShortestRSP failed";
    return false;
  }
  if (!RSPCheck(current_node, reeds_shepp_to_check, obstacles_segments_vec)) {
    ADEBUG << " RS fail";
    return false;
  }
  ADEBUG << "RStypes  =  " << reeds_shepp_to_check->segs_lengths.size();
  // load the whole RSP as nodes and add to the close set
  auto node = LoadRSPinCS(reeds_shepp_to_check, current_node);
  const bool check_start_point_kappa_cost =
      path_search_strategy_.enable_init_kappa_cost &&
      (!path_search_strategy_.is_plan_from_start ||
       (path_search_strategy_.is_plan_from_start &&
        *current_node == *start_node_));
  const double steer_margin =
      check_start_point_kappa_cost
          ? CaculateRsPathSteerMargion(reeds_shepp_to_check)
          : 0.0;
  node->SetPathCost(current_node->GetPathCost() +
                    EvaluateRsPathCost(reeds_shepp_to_check, current_node) +
                    warm_start_config_.steer_margin_penalty() * steer_margin);
  end_node_pq_.push(std::move(node));
  return false;
}

double HybridAStar::CaculateRsPathSteerMargion(
    const std::shared_ptr<ReedSheppPath>& reeds_shepp_path) {
  const auto rs_path_size = reeds_shepp_path->x.size();
  double steer_margin = 0.0;
  if (rs_path_size > 2) {
    double dtheta = common::math::NormalizeAngle(
        reeds_shepp_path->phi.at(rs_path_size - 1) -
        reeds_shepp_path->phi.at(rs_path_size - 2));
    double ds = std::hypot(reeds_shepp_path->x.at(rs_path_size - 1) -
                               reeds_shepp_path->x.at(rs_path_size - 2),
                           reeds_shepp_path->y.at(rs_path_size - 1) -
                               reeds_shepp_path->y.at(rs_path_size - 2));
    double kappa =
        reeds_shepp_path->gear.back() ? dtheta / ds : -1 * dtheta / ds;
    double steer_angle = TransKappaToSteering(kappa);
    steer_margin = fabs(steer_angle - cur_steer_angle_);
  }
  ADEBUG << "steer_margin is " << steer_margin;
  return steer_margin;
}

double HybridAStar::EvaluateRsPathCost(
    const std::shared_ptr<ReedSheppPath>& reeds_shepp_path,
    const std::shared_ptr<Node3d>& node_ptr) {
  int gear_shift_cnt = 0;
  std::vector<double> rs_seg_length;
  RSShiftTimes(node_ptr, reeds_shepp_path, &gear_shift_cnt, &rs_seg_length);
  double path_gear_switch_penalty =
      path_search_strategy_.is_nns_adjust_senario
          ? warm_start_config_.path_gear_switch_penalty_nns_adjust()
          : warm_start_config_.path_gear_switch_penalty();
  double rs_cost = reeds_shepp_path->total_length +
                   gear_shift_cnt * path_gear_switch_penalty;
  for (const auto seg_length : rs_seg_length) {
    rs_cost += CalculatePathLengthCost(seg_length);
  }
  if (!path_search_strategy_.is_nns_adjust_senario) {
    return rs_cost;
  }
  for (int i = 0; i < reeds_shepp_path->x.size(); ++i) {
    rs_cost += GetReferenceLineCost(reeds_shepp_path->x[i],
                                    reeds_shepp_path->y[i], rs_step_size_,
                                    path_search_strategy_.reference_line);
  }
  return rs_cost;
}

bool HybridAStar::RSPCheck(
    const std::shared_ptr<Node3d>& current_node,
    const std::shared_ptr<ReedSheppPath>& reeds_shepp_to_end,
    const std::vector<std::pair<common::math::LineSegment2d, double>>&
        obstacles_segments_vec) {
  if (reeds_shepp_to_end->x.empty()) {
    return false;
  }
  if (forced_path_direction_ != 0) {
    // check init path direction
    if (path_search_strategy_.is_plan_from_start &&
        *current_node == *start_node_) {
      int init_gear = reeds_shepp_to_end->gear.front() ? 1 : -1;
      if (init_gear * forced_path_direction_ < 0) {
        ADEBUG << "gear not satisfied init_gear: " << init_gear;
        return false;
      }
    }
    if (!path_search_strategy_.is_plan_from_start) {
      int init_gear = reeds_shepp_to_end->gear.back() ? -1 : 1;
      if (init_gear * forced_path_direction_ < 0) {
        ADEBUG << "gear not satisfied init_gear: " << init_gear;
        return false;
      }
    }
  }
  // ensure only no gear shift path be used
  if (path_search_strategy_.is_nns_adjust_senario) {
    int gear_shift_cnt = 0;
    RSShiftTimes(current_node, reeds_shepp_to_end, &gear_shift_cnt);
    if (gear_shift_cnt > 0) {
      return false;
    }
  }

  std::shared_ptr<Node3d> node = std::make_shared<Node3d>(
      reeds_shepp_to_end->x, reeds_shepp_to_end->y, reeds_shepp_to_end->phi,
      xy_bounds_, xy_grid_resolution_, phi_grid_resolution_);
  return ValidityCheck(node, obstacles_segments_vec);
}

void HybridAStar::RSShiftTimes(
    const std::shared_ptr<Node3d>& current_node,
    const std::shared_ptr<ReedSheppPath>& reeds_shepp_to_end,
    int* const rs_shift_times, std::vector<double>* const rs_seg_length) {
  if (nullptr == rs_shift_times) {
    return;
  }
  *rs_shift_times = 0;
  std::vector<double> seg_lengths;
  bool prev_gear = true;
  double seg_length = current_node->GetNearestGearShiftLength();
  for (size_t i = 0; i < reeds_shepp_to_end->gear.size(); ++i) {
    if (i == 0) {
      if (nullptr != current_node && nullptr != current_node->GetPreNode() &&
          current_node->GetDirec() != reeds_shepp_to_end->gear[0]) {
        ++(*rs_shift_times);
        seg_length = 0.0;
      }
    } else if (prev_gear != reeds_shepp_to_end->gear[i]) {
      seg_lengths.emplace_back(seg_length);
      seg_length = 0.0;
      ++(*rs_shift_times);
    } else {
      seg_length +=
          std::hypot(reeds_shepp_to_end->x[i] - reeds_shepp_to_end->x[i - 1],
                     reeds_shepp_to_end->y[i] - reeds_shepp_to_end->y[i - 1]);
    }
    prev_gear = reeds_shepp_to_end->gear[i];
  }
  seg_lengths.emplace_back(seg_length);
  if (nullptr != rs_seg_length) {
    rs_seg_length->assign(seg_lengths.begin(), seg_lengths.end());
  }
}

std::shared_ptr<Node3d> HybridAStar::LoadRSPinCS(
    const std::shared_ptr<ReedSheppPath>& reeds_shepp_to_end,
    const std::shared_ptr<Node3d>& current_node) {
  std::shared_ptr<Node3d> end_node = std::make_shared<Node3d>(
      reeds_shepp_to_end->x, reeds_shepp_to_end->y, reeds_shepp_to_end->phi,
      xy_bounds_, xy_grid_resolution_, phi_grid_resolution_);
  end_node->SetPre(current_node);
  close_set_.emplace(end_node->GetIndex(), end_node);
  return end_node;
}

std::shared_ptr<Node3d> HybridAStar::Next_node_generator(
    const std::shared_ptr<Node3d>& current_node, size_t next_node_index,
    const bool is_collison_free_exploration) {
  // TODO(lsy): cut off large dkappa node in nns adjust scenario
  double steering = 0.0;
  double traveled_distance = 0.0;
  if (static_cast<double>(next_node_index) <
      static_cast<double>(next_node_num_) / 2) {
    if (!current_node->GetDirec() &&
        current_node->GetNearestGearShiftLength() > 0 &&
        (current_node->GetNearestGearShiftLength() <
             warm_start_config_.min_one_direction_length() ||
         is_collison_free_exploration)) {
      ADEBUG << "forward expansion cutted off once, current node length: "
             << current_node->GetNearestGearShiftLength();
      return nullptr;
    }
    if (forced_path_direction_ == -1 &&
        path_search_strategy_.is_plan_from_start &&
        *current_node == *start_node_) {
      AINFO << "forward expansion cutted off by initial path search strategy";
      return nullptr;
    }
    steering = steer_seq_[next_node_index];

    // cut off forward node generation by scenario and  action
    if (!is_collison_free_exploration &&
        ForwardSteerActionCutoff(current_node, steering)) {
      return nullptr;
    }
    traveled_distance = step_size_;
  } else {
    // TODO(lsy): some nns adjust scenario will enable backward node
    if (path_search_strategy_.is_nns_adjust_senario) {
      return nullptr;
    }
    if (current_node->GetDirec() &&
        current_node->GetNearestGearShiftLength() > 0 &&
        (current_node->GetNearestGearShiftLength() <
             warm_start_config_.min_one_direction_length() ||
         is_collison_free_exploration)) {
      ADEBUG << "Backward expansion cutted off once, current node length: "
             << current_node->GetNearestGearShiftLength();
      return nullptr;
    }
    if (forced_path_direction_ == 1 &&
        path_search_strategy_.is_plan_from_start &&
        *current_node == *start_node_) {
      AINFO << "Backward expansion cutted off by initial path search strategy";
      return nullptr;
    }
    size_t index = next_node_index - next_node_num_ / 2;
    steering = steer_seq_[index];

    // cut off backward node generation by scenario and  action
    if (!is_collison_free_exploration &&
        BackwardSteerActionCutoff(current_node, steering)) {
      return nullptr;
    }

    traveled_distance = -step_size_;
  }

  // take above motion primitive to generate a curve driving the car to a
  // different grid
  double arc = std::sqrt(2) * xy_grid_resolution_;
  std::vector<double> intermediate_x;
  std::vector<double> intermediate_y;
  std::vector<double> intermediate_phi;
  double last_x = current_node->GetX();
  double last_y = current_node->GetY();
  double last_phi = current_node->GetPhi();
  intermediate_x.push_back(last_x);
  intermediate_y.push_back(last_y);
  intermediate_phi.push_back(last_phi);
  for (size_t i = 0; i <= static_cast<int>(arc / step_size_); ++i) {
    const double next_x = last_x + traveled_distance * std::cos(last_phi);
    const double next_y = last_y + traveled_distance * std::sin(last_phi);
    const double next_phi = common::math::NormalizeAngle(
        last_phi +
        traveled_distance / vehicle_param_.wheel_base() * std::tan(steering));
    intermediate_x.push_back(next_x);
    intermediate_y.push_back(next_y);
    intermediate_phi.push_back(next_phi);
    last_x = next_x;
    last_y = next_y;
    last_phi = next_phi;
  }
  // check if the vehicle runs outside of XY boundary
  if (intermediate_x.back() > xy_bounds_[1] ||
      intermediate_x.back() < xy_bounds_[0] ||
      intermediate_y.back() > xy_bounds_[3] ||
      intermediate_y.back() < xy_bounds_[2]) {
    return nullptr;
  }
  std::shared_ptr<Node3d> next_node = std::make_shared<Node3d>(
      intermediate_x, intermediate_y, intermediate_phi, xy_bounds_,
      xy_grid_resolution_, phi_grid_resolution_);
  next_node->SetPre(current_node);
  next_node->SetDirec(traveled_distance > 0.0);
  next_node->SetSteer(steering);
  if (current_node->GetDirec() != next_node->GetDirec()) {
    current_node->SetNearestGearShiftLength(0);
    next_node->SetNearestGearShiftLength(step_size_);
  } else {
    next_node->SetNearestGearShiftLength(
        current_node->GetNearestGearShiftLength() + step_size_);
  }

  return next_node;
}

void HybridAStar::CalculateNodeCost(const std::shared_ptr<Node3d>& current_node,
                                    const std::shared_ptr<Node3d>& next_node,
                                    const double distance_to_obstalce,
                                    const bool is_collison_free_exploration) {
  if (is_collison_free_exploration) {
    next_node->SetPathCost(warm_start_config_.obstacle_filter_distance() -
                           distance_to_obstalce);
    next_node->SetHeuCost(0.0);
    return;
  }
  next_node->SetPathCost(
      current_node->GetPathCost() +
      PathCost(current_node, next_node, distance_to_obstalce));
  // evaluate heuristic cost
  double optimal_path_cost =
      std::sqrt(2.0) * std::hypot(next_node->GetX() - end_node_->GetX(),
                                  next_node->GetY() - end_node_->GetY());
  constexpr double kGain = 1.0;
  common::SLPoint next_node_sl;
  common::SLPoint end_node_sl;
  if (path_search_strategy_.is_nns_adjust_senario &&
      !path_search_strategy_.reference_line.reference_points().empty() &&
      path_search_strategy_.reference_line.XYToSL(
          {next_node->GetX(), next_node->GetY()}, &next_node_sl) &&
      path_search_strategy_.reference_line.XYToSL(
          {end_node_->GetX(), end_node_->GetY()}, &end_node_sl)) {
    optimal_path_cost = kGain * (end_node_sl.s() - next_node_sl.s());
  }
  auto angle_cost = [](double from_angle, double to_angle) {
    double diff_angle_norm =
        fabs(common::math::AngleDiff(from_angle, to_angle)) / M_PI;
    return 5.0 * (exp(diff_angle_norm) - 1.0);
  };
  optimal_path_cost += angle_cost(next_node->GetPhi(), end_node_->GetPhi());

  next_node->SetHeuCost(optimal_path_cost);
}

double HybridAStar::PathCost(const std::shared_ptr<Node3d>& current_node,
                             const std::shared_ptr<Node3d>& next_node,
                             const double distance_to_obstalce) const {
  // evaluate cost on the path and add current cost

  double piecewise_cost = 0.0;
  if (next_node->GetDirec()) {
    piecewise_cost += static_cast<double>(next_node->GetStepSize() - 1) *
                      step_size_ * warm_start_config_.path_forward_penalty();
  } else {
    piecewise_cost += static_cast<double>(next_node->GetStepSize() - 1) *
                      step_size_ * warm_start_config_.path_back_penalty();
  }
  if (current_node->GetDirec() != next_node->GetDirec()) {
    piecewise_cost +=
        path_search_strategy_.is_nns_adjust_senario
            ? warm_start_config_.path_gear_switch_penalty_nns_adjust()
            : warm_start_config_.path_gear_switch_penalty();
  }
  if (distance_to_obstalce < warm_start_config_.obstacle_filter_distance() &&
      warm_start_config_.obstacle_filter_distance() > kEpsilon) {
    piecewise_cost +=
        (warm_start_config_.obstacle_filter_distance() - distance_to_obstalce) /
        warm_start_config_.obstacle_filter_distance() * step_size_ *
        warm_start_config_.obstacle_distance_penalty();
  }
  piecewise_cost += warm_start_config_.path_steer_change_penalty() *
                    std::fabs(common::math::AngleDiff(
                        next_node->GetSteer(), current_node->GetSteer()));
  piecewise_cost +=
      next_node->GetDirec() == current_node->GetDirec()
          ? CalculatePathLengthCost(next_node->GetNearestGearShiftLength()) -
                CalculatePathLengthCost(
                    current_node->GetNearestGearShiftLength())
          : CalculatePathLengthCost(next_node->GetNearestGearShiftLength());
  if (path_search_strategy_.is_nns_adjust_senario) {
    piecewise_cost +=
        GetReferenceLineCost(next_node->GetX(), next_node->GetY(), step_size_,
                             path_search_strategy_.reference_line);
  }
  return piecewise_cost;
}

double HybridAStar::GetReferenceLineCost(
    const double x, const double y, const double step_size,
    const ReferenceLine& reference_line) const {
  if (reference_line.reference_points().empty()) {
    return 0.0;
  }
  constexpr double kNormalLateral = 0.5;
  constexpr double kMinLateral = 0.2;
  constexpr double kGain = 3.0;
  common::SLPoint current_node_sl;
  const auto current_node = common::math::Vec2d(x, y);
  if (!reference_line.XYToSL(current_node, &current_node_sl) ||
      std::fabs(current_node_sl.l()) < kMinLateral) {
    return 0.0;
  }
  return warm_start_config_.reference_line_bias_penalty() * step_size *
         exp(kGain * (std::fabs(current_node_sl.l()) - kNormalLateral) /
             kNormalLateral);
}

bool HybridAStar::GetResult(PathGeneratorResult* const result,
                            const bool is_collison_free_exploration_path) {
  if (result == nullptr) {
    AERROR << "GetResult input check fails";
    return false;
  }
  std::shared_ptr<Node3d> current_node = final_node_;
  ADEBUG << "final_node: " << final_node_->GetX() << ", " << final_node_->GetY()
         << ", " << final_node_->GetPhi();
  std::vector<double> hybrid_a_x;
  std::vector<double> hybrid_a_y;
  std::vector<double> hybrid_a_phi;
  while (current_node->GetPreNode() != nullptr) {
    std::vector<double> x = current_node->GetXs();
    std::vector<double> y = current_node->GetYs();
    std::vector<double> phi = current_node->GetPhis();
    if (x.empty() || y.empty() || phi.empty()) {
      AERROR << "result size check failed";
      return false;
    }
    if (x.size() != y.size() || x.size() != phi.size()) {
      AERROR << "states sizes are not equal";
      return false;
    }
    std::reverse(x.begin(), x.end());
    std::reverse(y.begin(), y.end());
    std::reverse(phi.begin(), phi.end());
    if (x.size() > 1) {
      x.pop_back();
      y.pop_back();
      phi.pop_back();
    }
    hybrid_a_x.insert(hybrid_a_x.end(), x.begin(), x.end());
    hybrid_a_y.insert(hybrid_a_y.end(), y.begin(), y.end());
    hybrid_a_phi.insert(hybrid_a_phi.end(), phi.begin(), phi.end());
    current_node = current_node->GetPreNode();
  }
  hybrid_a_x.push_back(current_node->GetX());
  hybrid_a_y.push_back(current_node->GetY());
  hybrid_a_phi.push_back(current_node->GetPhi());
  // remove first point for collision free path
  if (is_collison_free_exploration_path) {
    auto iter = hybrid_a_x.begin();
    hybrid_a_x.erase(iter);
    iter = hybrid_a_y.begin();
    hybrid_a_y.erase(iter);
    iter = hybrid_a_phi.begin();
    hybrid_a_phi.erase(iter);
  }
  if (path_search_strategy_.is_plan_from_start ||
      is_collison_free_exploration_path) {
    std::reverse(hybrid_a_x.begin(), hybrid_a_x.end());
    std::reverse(hybrid_a_y.begin(), hybrid_a_y.end());
    std::reverse(hybrid_a_phi.begin(), hybrid_a_phi.end());
  }
  if (!path_search_strategy_.is_plan_from_start &&
      !is_collison_free_exploration_path) {
    // force adc pose as path start point
    hybrid_a_x[0] = end_node_->GetX();
    hybrid_a_y[0] = end_node_->GetY();
    hybrid_a_phi[0] = end_node_->GetPhi();
  }
  ADEBUG << " origin result size: " << result->x.size();
  (*result).x.insert((*result).x.end(), hybrid_a_x.begin(), hybrid_a_x.end());
  (*result).y.insert((*result).y.end(), hybrid_a_y.begin(), hybrid_a_y.end());
  (*result).phi.insert((*result).phi.end(), hybrid_a_phi.begin(),
                       hybrid_a_phi.end());
  ADEBUG << "new result size: " << result->x.size();
  if (result->x.size() != result->y.size() ||
      result->x.size() != result->phi.size()) {
    AERROR << "state sizes not equal, "
           << "result->x.size(): " << result->x.size() << "result->y.size()"
           << result->y.size() << "result->phi.size()" << result->phi.size();
    return false;
  }
  return true;
}

bool HybridAStar::CalculateParkingPrefinishCondition(
    const std::shared_ptr<Node3d>& current_node) {
  if (path_search_strategy_.trace_adjust_search_strategy.is_trace_adjust) {
    common::SLPoint current_node_sl;
    if (path_search_strategy_.trace_adjust_search_strategy.trace_path.XYToSL(
            current_node->GetX(), current_node->GetY(), &current_node_sl)) {
      const auto matched_point =
          path_search_strategy_.trace_adjust_search_strategy.trace_path
              .Evaluate(current_node_sl.s());
      const double diff_l = std::fabs(current_node_sl.l());
      const double diff_theta = std::fabs(common::math::AngleDiff(
          matched_point.theta(), current_node->GetPhi()));
      if (diff_l < path_search_strategy_.trace_adjust_search_strategy
                       .finish_l_threshold &&
          diff_theta < path_search_strategy_.trace_adjust_search_strategy
                           .finish_theta_threshold) {
        return true;
      }
    }
  }
  bool direction_check = true;
  const auto& polygon = std::get<0>(dest_region_with_angle_);
  const double from_angle = std::get<1>(dest_region_with_angle_);
  const double to_angle = std::get<2>(dest_region_with_angle_);
  bool reach_dest =
      (polygon.num_points() > 2 &&
       common::math::AngleInRange(current_node->GetPhi(), from_angle,
                                  to_angle) &&
       polygon.IsPointIn({current_node->GetX(), current_node->GetY()}));
  if (reach_dest && forced_path_direction_ != 0 &&
      !path_search_strategy_.is_plan_from_start) {
    const int init_gear = current_node->GetDirec() ? -1 : 1;
    // need check init direction
    direction_check = init_gear * forced_path_direction_ > 0;
    if (!direction_check) {
      AERROR << "direction_check failed " << init_gear;
    }
  }
  return reach_dest && direction_check;
}

bool HybridAStar::Plan(
    const std::atomic<bool>& atomic_early_stop_flag,
    const common::PathPoint& start_point, const common::PathPoint& end_point,
    const std::vector<double>& xy_bounds,
    const std::vector<std::pair<common::math::LineSegment2d, double>>&
        obstacles_segments_vec,
    const DestRegionWithAng& dest_region_with_angle,
    const PathSearchStrategy& path_search_strategy,
    PathGeneratorResult* const result) {
  if (xy_bounds.size() != 4 || result == nullptr) {
    AERROR << "hybrid a star input check fails";
    return false;
  }
  path_search_strategy_ = path_search_strategy;
  forced_path_direction_ = path_search_strategy_.init_path_direction;
  ADEBUG << path_search_strategy_.DebugString();
  // clear containers and load and  init  data
  open_set_.clear();
  close_set_.clear();
  result->reset();
  bool has_search_extension_path = false;
  if (path_search_strategy_.is_plan_from_start) {
    dest_region_with_angle_ = dest_region_with_angle;
  } else {
    dest_region_with_angle_ = DestRegionWithAng();
  }
  open_pq_ = decltype(open_pq_)();
  final_node_ = nullptr;
  is_valid_end_pose_ = true;
  // load xy_bounds
  xy_bounds_ = xy_bounds;
  static constexpr double kEpision = 1e-3;
  auto obstacles_segments_vec_rs = obstacles_segments_vec;
  for (auto& obstacles_segments : obstacles_segments_vec_rs) {
    obstacles_segments.second +=
        obstacles_segments.second <= kEpision
            ? 0
            : warm_start_config_.extra_distance_for_rs();
  }
  // load exploration conf
  NodeGenerationParameterDecision(start_point, end_point);

  // if replan due to obstacle collision, extend start point by exploration
  common::PathPoint intermediate_point = start_point;

  if (!GenerateLocalExtensionPath(
          start_point, obstacles_segments_vec, &forced_path_direction_, result,
          &intermediate_point, &has_search_extension_path)) {
    AERROR << "generate collision free local Path fails";
    return false;
  }
  ADEBUG << "intermediate_point: " << intermediate_point.x() << ", "
         << intermediate_point.y() << "," << intermediate_point.theta();
  ADEBUG << "forced_path_direction: " << forced_path_direction_;
  open_set_.clear();
  close_set_.clear();
  final_node_ = nullptr;
  open_pq_ = decltype(open_pq_)();
  end_node_pq_ = decltype(end_node_pq_)();
  // generate start and  end node
  start_node_ =
      path_search_strategy_.is_plan_from_start
          ? std::make_shared<Node3d>(intermediate_point.x(),
                                     intermediate_point.y(),
                                     intermediate_point.theta(), xy_bounds_,
                                     xy_grid_resolution_, phi_grid_resolution_)
          : std::make_shared<Node3d>(end_point.x(), end_point.y(),
                                     end_point.theta(), xy_bounds_,
                                     xy_grid_resolution_, phi_grid_resolution_);
  end_node_ = path_search_strategy_.is_plan_from_start
                  ? std::make_shared<Node3d>(
                        end_point.x(), end_point.y(), end_point.theta(),
                        xy_bounds_, xy_grid_resolution_, phi_grid_resolution_)
                  : std::make_shared<Node3d>(
                        intermediate_point.x(), intermediate_point.y(),
                        intermediate_point.theta(), xy_bounds_,
                        xy_grid_resolution_, phi_grid_resolution_);
  if (path_search_strategy_.trace_adjust_search_strategy.is_trace_adjust) {
    xy_bounds_ = path_search_strategy.trace_adjust_search_strategy.xy_bounds;
    common::SLPoint start_node_sl;
    if (!path_search_strategy_.trace_adjust_search_strategy.trace_path.XYToSL(
            start_point.x(), start_point.y(), &start_node_sl)) {
      return false;
    }
    const auto target_s = std::min(
        start_node_sl.s() +
            path_search_strategy_.trace_adjust_search_strategy.target_s,
        path_search_strategy.trace_adjust_search_strategy.trace_path.Length());
    const auto trace_adjust_target =
        path_search_strategy_.trace_adjust_search_strategy.trace_path.Evaluate(
            target_s);
    end_node_ = std::make_shared<Node3d>(
        trace_adjust_target.x(), trace_adjust_target.y(),
        trace_adjust_target.theta(), xy_bounds_, xy_grid_resolution_,
        phi_grid_resolution_);
  }
  if (!ValidityCheck(start_node_, obstacles_segments_vec)) {
    AERROR << "start_node in collision with obstacles " << start_node_->GetX()
           << " " << start_node_->GetY();
    result->reset();
    return false;
  }
  if (!ValidityCheck(end_node_, obstacles_segments_vec_rs)) {
    // check target region is valid or not
    const bool has_valid_target_region =
        path_search_strategy_.is_plan_from_start &&
        std::get<0>(dest_region_with_angle).num_points() > 2;
    auto min_distance = common::math::GetMinDistance2ObstaclesSegments(
        end_node_->GetX(), end_node_->GetY(), end_node_->GetPhi(),
        obstacles_segments_vec, warm_start_config_.obstacle_filter_distance());
    is_valid_end_pose_ =
        common::math::double_type::ComparedToZero(min_distance) > 0;
    if (!has_valid_target_region && !is_valid_end_pose_) {
      AERROR << "end_node in collision with obstacles";
      result->reset();
      return false;
    }
    obstacles_segments_vec_rs = obstacles_segments_vec;
    static constexpr double kMinBuffer = 0.2;
    const double rs_buffer = common::util::BoundedValue(
        0.0, warm_start_config_.extra_distance_for_rs(),
        min_distance - kMinBuffer);
    for (auto& obstacles_segments : obstacles_segments_vec_rs) {
      obstacles_segments.second +=
          obstacles_segments.second <= kEpision ? 0 : rs_buffer;
    }
    ADEBUG << " RS buffer : " << rs_buffer;
    AINFO << (is_valid_end_pose_
                  ? "modified rs buffer to acclerate search "
                  : "end node is invaild, it will skip reeds shepp connecting");
  }
  // load open set, pq
  start_node_->SetPathCost(
      CalculatePathLengthCost(start_node_->GetNearestGearShiftLength()));
  open_set_.emplace(start_node_->GetIndex(), start_node_);
  open_pq_.emplace(start_node_->GetIndex(), start_node_->GetCost());
  // init exploration shown up debug info
  auto expansion_info = ExpansionInfo(intermediate_point);
  // Hybrid A* begininq
  size_t explored_node_num = 0;
  double astar_start_time = Clock::NowInSeconds();
  double heuristic_time = 0.0;
  double rs_time = 0.0;
  result->rs_connect_point.set_x(end_node_->GetX());
  result->rs_connect_point.set_y(end_node_->GetY());
  result->rs_connect_point.set_theta(end_node_->GetPhi());
  std::vector<double> angle_vec;
  static constexpr double kExplorationStepTime = 1.0;
  static constexpr int kMaxStepNum = 4;
  double iter_num = 0;
  const double max_explore_time =
      path_search_strategy_.is_dead_end_scenario ||
              path_search_strategy_.is_narrow_passage_scenario
          ? warm_start_config_.dead_end_scenario_max_exploration_time()
          : warm_start_config_.max_exploration_time();
// LCOV_EXCL_START
#ifndef ISMDC
  if (FLAGS_enable_one_shoot_log) {
    expansion_info.load_extension_environment_info(obstacles_segments_vec);
  }
#endif
  // LCOV_EXCL_STOP
  double search_time = 0.0;
  while (!open_pq_.empty()) {
    if (atomic_early_stop_flag) {
      break;
    }
    search_time = Clock::NowInSeconds() - astar_start_time;
    if (search_time >
        max_explore_time + kExplorationStepTime * explored_overtime_times_) {
      if (explored_overtime_times_ < kMaxStepNum) {
        explored_overtime_times_++;
      }
      AERROR << "exploration time exceeds max accepatble time";
      result->reset();
      return false;
    }

    if (search_time > warm_start_config_.accept_exploration_time() &&
        !end_node_pq_.empty()) {
      ADEBUG << "exploration time reach accepatble time and result ";
      break;
    }
    // take out the lowest cost neighboring node
    const std::string current_id = open_pq_.top().first;
    open_pq_.pop();
    const std::shared_ptr<Node3d>& current_node = open_set_[current_id];
    // LCOV_EXCL_START
#ifndef ISMDC
    if (FLAGS_enable_one_shoot_log) {
      expansion_info.load_current_node_info(iter_num, current_id,
                                            astar_start_time, current_node);
    }
#endif
    // LCOV_EXCL_STOP

    // check if an analystic curve could be connected from current
    // pose  to the end pose without collision.
    const double rs_start_time = Clock::NowInSeconds();
    if (AnalyticExpansion(current_node, obstacles_segments_vec_rs)) {
      break;
    }

    const double rs_end_time = Clock::NowInSeconds();
    rs_time += rs_end_time - rs_start_time;
    close_set_.emplace(current_node->GetIndex(), current_node);
    double distance_to_obstalce = kDefaultIgnorableDist;
    for (size_t i = 0; i < next_node_num_; ++i) {
      if (atomic_early_stop_flag) {
        break;
      }
      const auto& next_node = Next_node_generator(current_node, i);
      if (next_node == nullptr) {
        continue;
      }
      if (close_set_.find(next_node->GetIndex()) != close_set_.end()) {
        continue;
      }
      if (open_set_.find(next_node->GetIndex()) == open_set_.end()) {
        if (!ValidityCheck(next_node, obstacles_segments_vec,
                           &distance_to_obstalce)) {
          continue;
        }
        explored_node_num++;
        const double start_time = Clock::NowInSeconds();
        CalculateNodeCost(current_node, next_node, distance_to_obstalce);
        const double end_time = Clock::NowInSeconds();
        heuristic_time += end_time - start_time;
        open_set_.emplace(next_node->GetIndex(), next_node);
        open_pq_.emplace(next_node->GetIndex(), next_node->GetCost());
        // LCOV_EXCL_START
#ifndef ISMDC
        if (FLAGS_enable_one_shoot_log) {
          expansion_info.load_extension_node_info(iter_num, current_id,
                                                  next_node);
        }
#endif
        // LCOV_EXCL_STOP
      }
    }
    iter_num = iter_num + 1;
  }
  ADEBUG << "end_node_pq_ " << end_node_pq_.size();
  if (nullptr == final_node_ && !end_node_pq_.empty()) {
    final_node_ = end_node_pq_.top();
  }
  if (final_node_ == nullptr) {
    explored_failure_times_++;
    AERROR << "Hybrid A searching return null ptr(open_set ran out) open_set_ "
              "size "
           << open_set_.size();
    result->reset();
    return false;
  }
  if (!final_node_->GetXs().empty()) {
    result->rs_connect_point.set_x(final_node_->GetXs().front());
    result->rs_connect_point.set_y(final_node_->GetYs().front());
    result->rs_connect_point.set_theta(final_node_->GetPhis().front());
  }
  if (!GetResult(result)) {
    ADEBUG << "GetResult failed";
    result->reset();
    return false;
  }
  result->path_type =
      has_search_extension_path
          ? planning_internal::PathUpdateStatus::SEARCH_EXTENSION_PATH
          : planning_internal::PathUpdateStatus::SEARCH_PATH;
  // LCOV_EXCL_START
#ifndef ISMDC
  if (FLAGS_enable_one_shoot_log) {
    expansion_info.load_coarse_path_info((*result).x, (*result).y,
                                         (*result).phi);
  }
#endif
  // LCOV_EXCL_STOP
  explored_overtime_times_ = 0;
  explored_failure_times_ = 0;
  ADEBUG << "explored node num is " << explored_node_num;
  ADEBUG << "heuristic time is " << heuristic_time;
  ADEBUG << "reed shepp time is " << rs_time;
  ADEBUG << "hybrid astar total time is "
         << Clock::NowInSeconds() - astar_start_time;
  ADEBUG << "end_node_pq size is " << end_node_pq_.size();
  return true;
}

void HybridAStar::NodeGenerationParameterDecision(
    const common::PathPoint& start_point, const common::PathPoint& end_point) {
  static constexpr double kCutOffYChangeStepTime = 2.0;
  if (path_search_strategy_.space_structure == SpaceStructure::LAT_PARK_LOT) {
    step_size_ = warm_start_config_.horizontal_step_size();
    xy_grid_resolution_ = warm_start_config_.horizontal_xy_grid_resolution();
    phi_grid_resolution_ = warm_start_config_.horizontal_phi_grid_resolution();
    max_y_cut_off_ = warm_start_config_.horizontal_max_y_cut_off();
  } else {
    step_size_ = warm_start_config_.step_size();
    xy_grid_resolution_ = warm_start_config_.xy_grid_resolution();
    phi_grid_resolution_ = warm_start_config_.phi_grid_resolution();
    max_y_cut_off_ = (path_search_strategy_.is_dead_end_scenario
                          ? warm_start_config_.dead_end_scenario_max_y_cut_off()
                          : warm_start_config_.max_y_cut_off()) -
                     kCutOffYChangeStepTime * std::max(explored_overtime_times_,
                                                       explored_failure_times_);
  }
  lateral_rs_steer_angle_margin_ =
      path_search_strategy_.use_larger_curvature
          ? warm_start_config_.max_steer_angle_margin()
          : warm_start_config_.lateral_spot_steer_angle_margin();
  double cur_kappa = path_search_strategy_.is_plan_from_start
                         ? end_point.kappa()
                         : start_point.kappa();
  cur_steer_angle_ = TransKappaToSteering(cur_kappa);
  InitParam();
}

bool HybridAStar::GenerateLocalExtensionPath(
    const common::PathPoint& start_point,
    const std::vector<std::pair<common::math::LineSegment2d, double>>&
        obstacles_segments_vec,
    int* const forced_path_direction_ptr, PathGeneratorResult* const result,
    common::PathPoint* const intermediate_point_ptr,
    bool* const has_search_extension_path) {
  if (result == nullptr || intermediate_point_ptr == nullptr ||
      forced_path_direction_ptr == nullptr ||
      has_search_extension_path == nullptr) {
    AERROR << "GenerateLocalExtensionPath input check fails";
    return false;
  }
  if (path_search_strategy_.is_nns_adjust_senario) {
    return true;
  }
  start_node_ = std::make_shared<Node3d>(
      start_point.x(), start_point.y(), start_point.theta(), xy_bounds_,
      xy_grid_resolution_, phi_grid_resolution_);
  if (!path_search_strategy_.collision_free_search_strategy
           .replan_due_to_collision &&
      explored_overtime_times_ == 0 &&
      ValidityCheck(start_node_, obstacles_segments_vec)) {
    return true;
  }
  std::vector<std::pair<common::math::LineSegment2d, double>>
      obstacles_segments_vec_pure;
  std::vector<std::pair<common::math::LineSegment2d, double>>
      obstacles_segments_vec_virtual;
  obstacles_segments_vec_pure.reserve(obstacles_segments_vec.size());
  obstacles_segments_vec_virtual.reserve(obstacles_segments_vec.size());
  for (const auto& obstacles_segment : obstacles_segments_vec) {
    if (path_search_strategy_.park_direction == PARKIN ||
        obstacles_segment.second > kEpsilon) {
      obstacles_segments_vec_pure.emplace_back(obstacles_segment.first,
                                               kEpsilon);
    } else {
      obstacles_segments_vec_virtual.emplace_back(obstacles_segment);
    }
  }
  double collision_free_dist =
      path_search_strategy_.collision_free_search_strategy.collision_free_dist;
  if (path_search_strategy_.park_direction == PARKIN ||
      start_point.y() > kDisableOverTimeExtraDistHeight) {
    collision_free_dist += explored_overtime_times_ * kExtensionExtraDist;
  }
  ADEBUG << "collision_free_dist: " << collision_free_dist;
  double exploration_start_time = Clock::NowInSeconds();
  size_t explored_node_num = 0;
  double exploration_max_time = 0.2;
  double current_dist_to_obstacle = kDefaultIgnorableDist;
  if (!ValidityCheck(start_node_, obstacles_segments_vec_pure,
                     &current_dist_to_obstacle)) {
    AERROR << "start point is collision with obstacle";
    return false;
  }
  *has_search_extension_path = true;
  if (current_dist_to_obstacle >= collision_free_dist &&
      ValidityCheck(start_node_, obstacles_segments_vec)) {
    ADEBUG
        << "start point is already collision free, current_dist_to_obstacle: "
        << current_dist_to_obstacle;
    *has_search_extension_path = false;
  }
  open_set_.clear();
  open_pq_ = decltype(open_pq_)();
  open_set_.emplace(start_node_->GetIndex(), start_node_);
  open_pq_.emplace(start_node_->GetIndex(), start_node_->GetCost());
  bool is_path_found = false;
  while (!open_pq_.empty()) {
    if (Clock::NowInSeconds() - exploration_start_time > exploration_max_time) {
      AERROR << "exploration time exceeds max accepatble time";
      return false;
    }
    // take out the lowest cost neighboring node
    const std::string current_id = open_pq_.top().first;
    open_pq_.pop();
    auto& current_node = open_set_[current_id];
    close_set_.emplace(current_node->GetIndex(), current_node);
    ValidityCheck(current_node, obstacles_segments_vec_pure,
                  &current_dist_to_obstacle);
    ADEBUG << "current_dist_to_obstacle: " << current_dist_to_obstacle;
    double distance_to_obstalce = kDefaultIgnorableDist;
    for (size_t i = 0; i < next_node_num_; ++i) {
      const auto& next_node = Next_node_generator(current_node, i, true);
      if (next_node == nullptr) {
        continue;
      }
      if (close_set_.find(next_node->GetIndex()) != close_set_.end()) {
        continue;
      }
      if (open_set_.find(next_node->GetIndex()) == open_set_.end()) {
        if (!ValidityCheck(next_node, obstacles_segments_vec_pure,
                           &distance_to_obstalce) ||
            distance_to_obstalce < current_dist_to_obstacle) {
          continue;
        }
        if (path_search_strategy_.park_direction != PARKIN &&
            !ValidityCheck(next_node, obstacles_segments_vec_virtual)) {
          continue;
        }
        explored_node_num++;
        CalculateNodeCost(current_node, next_node, distance_to_obstalce, true);
        open_set_.emplace(next_node->GetIndex(), next_node);
        open_pq_.emplace(next_node->GetIndex(), next_node->GetCost());
        ADEBUG << "generate node: " << next_node->GetX() << ", "
               << next_node->GetY() << ", " << next_node->GetPhi();
        ADEBUG << "distance_to_obstalce: " << distance_to_obstalce;
        if (distance_to_obstalce >= collision_free_dist &&
            next_node->GetNearestGearShiftLength() >=
                warm_start_config_.min_one_direction_length() &&
            ValidityCheck(next_node, obstacles_segments_vec)) {
          final_node_ = next_node;
          intermediate_point_ptr->set_x(next_node->GetXs().back());
          intermediate_point_ptr->set_y(next_node->GetYs().back());
          intermediate_point_ptr->set_theta(next_node->GetPhis().back());
          *forced_path_direction_ptr = next_node->GetDirec() ? 1 : -1;
          is_path_found = true;
          AINFO << "collision free path is found";
          break;
        }
      }
    }
    if (is_path_found) {
      break;
    }
  }
  if (nullptr == final_node_) {
    AERROR << " coliision free path is not found";
    return !(*has_search_extension_path);
  }
  ADEBUG << "collsion free path search cost time: "
         << (Clock::NowInSeconds() - exploration_start_time) * 1000 << " ms";
  ADEBUG << "explored_node_num :" << explored_node_num;
  return GetResult(result, true);
}

}  // namespace planning
}  // namespace TL
