/*
 * Copyright (c) 2022 TL Technologies Co., Ltd. All rights reserved.
 */

#include "planning/tasks/deciders/obstacles_decider/evaluator/behavior_evaluator.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <vector>

#include "planning/common/path_decision.h"
#include "proto/common/pnc_point.pb.h"
#include "proto/planning/sl_boundary.pb.h"

namespace TL {
namespace planning {

namespace {
constexpr double kEpsilon = 0.0000001;
constexpr double kHalf = 0.5;
constexpr double kAdcMaxAccel = 1.0;
constexpr double kAdcMaxDecel = 2.0;
constexpr double kObsMaxAccel = 1.0;
constexpr double kObsMaxDecel = 2.0;
constexpr double kMotionTime = 2.0;
constexpr double kAdcObsMaxDecelThd = 2.0;
constexpr double kTwoObsLgtSafeDistance = 5.0;
constexpr double kRSSMaxSpeed = 33.33;

constexpr int kTrajSize = 80;

// weight
constexpr double kWeightDriveStyle = 1.0;
constexpr double kWeightTrafficRule = 0.3;
constexpr double kWeightSafety = 1.0;
constexpr double kWeightComfort = 0.5;
constexpr double kWeightEfficiency = 1.0;
constexpr double kWeightConsistency = 0.5;
}  // namespace

bool BehaviorEvaluator::Evaluate(
    ReferenceLineInfo* reference_line_info,
    const std::string& interactive_obs_id,
    const std::unordered_map<std::string, ObstacleInfo>& obstacles_info,
    const std::pair<std::string, std::string>& leading_obs_id,
    std::pair<bool, double> adc_merge_info, BehaviorPair* adc_obs_behavior,
    double* min_cost) {
  ADEBUG << "++++++++++EvaluateAdcObsBehavior";
  ADEBUG << "interactive_obs_id=" << interactive_obs_id;
  ADEBUG << "inter_leading_id=" << leading_obs_id.second;
  ADEBUG << "adc_leading_id=" << leading_obs_id.first;
  std::vector<InteractiveObsBehavior::Behavior> behavior_option = {
      InteractiveObsBehavior::ConstantSpeed, InteractiveObsBehavior::Accelerate,
      InteractiveObsBehavior::Decelerate};
  const size_t adc_behavior_size = 3;  // rows
  const size_t obs_behavior_size = 3;  // columns
  std::vector<std::vector<std::pair<BehaviorPair, CostInfo>>> cost_vec(
      adc_behavior_size);
  size_t adc_index = 0;
  std::vector<double> adc_cost_vec;
  adc_cost_vec.reserve(adc_behavior_size);
  BehaviorCost behavior_cost;
  // calculate cost of all combinations
  for (const auto& adc_behavior : behavior_option) {
    double adc_cost_sum = 0.0;
    cost_vec.at(adc_index).reserve(obs_behavior_size);
    for (const auto& obs_behavior : behavior_option) {
      BehaviorPair behavior_pair = {adc_behavior, obs_behavior};
      double cost = behavior_cost.CalculateCost(
          *reference_line_info, behavior_pair, last_adc_behavior_,
          interactive_obs_id, obstacles_info, leading_obs_id, adc_merge_info);
      CostInfo cost_info = {cost, behavior_cost.GetAdcIsLatecomer()};
      cost_vec.at(adc_index).emplace_back(behavior_pair, cost_info);
      adc_cost_sum += cost;
      ADEBUG << "behavior combination (adc:"
             << InteractiveObsBehavior::Behavior_Name(adc_behavior) << ","
             << "obs:" << InteractiveObsBehavior::Behavior_Name(obs_behavior)
             << "):"
             << "CalculateCost=" << cost;
    }
    adc_cost_vec.emplace_back(adc_cost_sum);
    adc_index++;
  }

  // find min cost combination behavior
  const auto min_adc_cost_index =
      std::min_element(adc_cost_vec.begin(), adc_cost_vec.end()) -
      adc_cost_vec.begin();
  ADEBUG << "min_adc_cost_index=" << min_adc_cost_index;
  const auto min_obs_cost_index =
      std::min_element(cost_vec.at(min_adc_cost_index).begin(),
                       cost_vec.at(min_adc_cost_index).end(),
                       CostComparator()) -
      cost_vec.at(min_adc_cost_index).begin();
  ADEBUG << "min_cost_index=" << min_obs_cost_index;

  // default is_yield = true; is_yield == false means is_overtake
  bool is_yield = true;
  if (adc_cost_vec.at(min_adc_cost_index) ==
      std::numeric_limits<double>::infinity()) {
    *min_cost = std::numeric_limits<double>::infinity();
    *adc_obs_behavior = {InteractiveObsBehavior::Decelerate,
                         InteractiveObsBehavior::ConstantSpeed};
    ADEBUG << "Minimum cost = inf, obs_" << interactive_obs_id << ":"
           << "collision!!!";
    is_yield = true;
  } else {
    *min_cost =
        cost_vec.at(min_adc_cost_index).at(min_obs_cost_index).second.first;
    *adc_obs_behavior =
        cost_vec.at(min_adc_cost_index).at(min_obs_cost_index).first;
    ADEBUG << "Minimum cost combination: "
           << "(adc:"
           << InteractiveObsBehavior::Behavior_Name(adc_obs_behavior->first)
           << ", obs_" << interactive_obs_id << ":"
           << InteractiveObsBehavior::Behavior_Name(adc_obs_behavior->second)
           << "), min_cost=" << *min_cost;
    // adc_is_latecomer
    if (cost_vec.at(min_adc_cost_index).at(min_obs_cost_index).second.second) {
      ADEBUG << "obstacle_decider/interactive/yield";
      is_yield = true;
    } else {
      ADEBUG << "obstacle_decider/interactive/overtake";
      is_yield = false;
    }
  }
  last_adc_behavior_ = adc_obs_behavior->first;
  return is_yield;
}

double BehaviorCost::CalculateCost(
    const ReferenceLineInfo& reference_line_info,
    const BehaviorPair& adc_obs_behavior,
    const InteractiveObsBehavior::Behavior& last_adc_behavior,
    const std::string& interactive_obs_id,
    const std::unordered_map<std::string, ObstacleInfo>& obstacles_info,
    const std::pair<std::string, std::string>& leading_obs_id,
    std::pair<bool, double> adc_merge_info) {
  double adc_a = 0.0;
  const double time_gap = kMotionTime;
  double obs_a = 0.0;
  const auto& adc_behavior = adc_obs_behavior.first;
  if (adc_behavior == InteractiveObsBehavior::Accelerate) {
    adc_a = kAdcMaxAccel;
  } else if (adc_behavior == InteractiveObsBehavior::Decelerate) {
    adc_a = -kAdcMaxDecel;
  }
  const auto& obs_behavior = adc_obs_behavior.second;
  if (obs_behavior == InteractiveObsBehavior::Accelerate) {
    obs_a = kObsMaxAccel;
  } else if (obs_behavior == InteractiveObsBehavior::Decelerate) {
    obs_a = -kObsMaxDecel;
  }

  const auto& adc_iter = obstacles_info.find("adc");
  if (adc_iter == obstacles_info.end()) {
    return std::numeric_limits<double>::infinity();
  }
  const double adc_s =
      (adc_iter->second.start_s + adc_iter->second.end_s) * kHalf;
  const double adc_v = adc_iter->second.speed;

  const double cost_safety = GetSafetyCost(
      reference_line_info.path_decision(), adc_s, adc_v, adc_a, obs_a,
      interactive_obs_id, obstacles_info, leading_obs_id, adc_merge_info);
  if (cost_safety == std::numeric_limits<double>::infinity()) {
    ADEBUG << "cost_safety is infinity";
    return std::numeric_limits<double>::infinity();
  }

  return GetDriveStyleCost() +
         GetTrafficRuleCost(adc_a, obs_a, adc_merge_info.first) + cost_safety +
         GetComfortCost(adc_a, obs_a) +
         GetEfficiencyCost(adc_a, time_gap, obs_a, reference_line_info,
                           interactive_obs_id, obstacles_info) +
         GetConsistencyCost(adc_behavior, last_adc_behavior);
}

double BehaviorCost::GetDriveStyleCost() {
  ADEBUG << "GetDriveStyleCost=" << 0.0;
  return kWeightDriveStyle * 0.0;
}

double BehaviorCost::GetTrafficRuleCost(double adc_a, double obs_a,
                                        bool adc_in_dying_road) {
  double adc_a_cost = 0.0;
  double obs_a_cost = 0.0;
  const double kAdcNoRightOfWayCost = 2.0;
  // right of way
  if (adc_in_dying_road) {
    if (adc_a > 0) {
      adc_a_cost += kAdcNoRightOfWayCost;
    }
    if (obs_a < 0) {
      obs_a_cost += kAdcNoRightOfWayCost;
    }
  }
  double adc_weight = 0.7;
  double obs_weight = 0.3;
  double cost_traffic_rule =
      kWeightTrafficRule *
      std::fmin((adc_weight * adc_a_cost + obs_weight * obs_a_cost), 1.0);
  ADEBUG << "GetTrafficRuleCost=" << cost_traffic_rule;
  return cost_traffic_rule;
}

double BehaviorCost::GetRSSLgtSafeDistance(const double v_r, const double v_f) {
  // https://zhuanlan.zhihu.com/p/638514363
  // RSS: s_min=v_r*t_resp+0.5*a_r_max*t_resp^2
  // +(v_r+a_r_max*t_resp)^2/(2*a_r_brake_min)-v_f^2/(2*a_f_brake_max)
  // in which v_r means the speed of rear car and v_f means the front car, and the same direction
  double t_resp = 0.1;
  double a_r_acc_max = 1.0;
  double a_r_brake_min = 4.0;
  double a_f_brake_max = 2.0;

  double v_r_at_resp_time = v_r + a_r_acc_max * t_resp;
  double s_r_distance_driven =
      0.5 * (v_r + v_r_at_resp_time) * t_resp +
      v_r_at_resp_time * v_r_at_resp_time / (2 * a_r_brake_min);
  double s_f_distance_driven = v_f * v_f / (2 * a_f_brake_max);
  double s_min = s_r_distance_driven - s_f_distance_driven;
  return s_min;
}

double BehaviorCost::GetRSSLgtSafeVelocity(const double v_f,
                                           const double lgt_distance_abs) {
  // RssChecker from https://github.com/HKUST-Aerial-Robotics/EPSILON
  // vel_upp_r*t_resp*(1+a_r_acc_max/a_r_brake_min)
  // +0.5*a_r_acc_max*pow(t_resp, 2)*(1+a_r_acc_max/a_r_brake_min)
  // +vel_upp_r^2/(2.0*a_r_brake_min)
  // =s_f_distance_driven+lgt_distance_abs
  double t_resp = 0.1;
  double a_r_acc_max = 1.0;
  double a_r_brake_min = 4.0;
  double a_f_brake_max = 2.0;
  // ego ---->(lon_distance) front --->
  // front hard brake
  double s_f_distance_driven = (v_f * v_f) / (2 * a_f_brake_max);
  // ego has vel upp
  double a = 1.0 / (2.0 * a_r_brake_min);
  double b = t_resp + (a_r_acc_max * t_resp / a_r_brake_min);
  double c = 0.5 * (a_r_acc_max + a_r_acc_max * a_r_acc_max / a_r_brake_min) *
                 t_resp * t_resp -
             s_f_distance_driven - lgt_distance_abs;
  double vel_upp_r =
      std::fmin((-b + std::sqrt(b * b - 4 * a * c)) / (2 * a), kRSSMaxSpeed);
  return vel_upp_r;
}

double BehaviorCost::CalArrivalTime(double curr_v, double curr_a, double* new_v,
                                    double delta_s, double curr_arrival_time) {
  if (std::fabs(curr_a) > kEpsilon && delta_s > kEpsilon) {
    if (delta_s > 0.5 * (curr_v + *new_v) * kMotionTime) {
      // curr_arrival_time > kMotionTime, namely curr_a in kMotionTime and then constant speed
      // (v+v_new)/2*T+v_new*t=s
      curr_arrival_time = (delta_s - 0.5 * (curr_v + *new_v) * kMotionTime) /
                              std::fmax(*new_v, kEpsilon) +
                          kMotionTime;
    } else {
      // curr_arrival_time < kMotionTime, namely curr_a in curr_arrival_time
      // v*t+0.5*a*t^2=s, t=(-v+sqrt(v*v+4*0.5*a*s))/(2*0.5*a)
      double temp = curr_v * curr_v + 2 * curr_a * delta_s;
      if (temp > kEpsilon) {
        curr_arrival_time = (-curr_v + std::sqrt(temp)) / curr_a;
        *new_v = curr_v + curr_a * curr_arrival_time;
      }
    }
  }
  curr_arrival_time = std::fmax(0.0, curr_arrival_time);
  return curr_arrival_time;
}

double BehaviorCost::GetSafetyCost(
    const PathDecision& path_decision, const double adc_s, double adc_v,
    const double adc_a, const double obs_a,
    const std::string& interactive_obs_id,
    const std::unordered_map<std::string, ObstacleInfo>& obstacles_info,
    const std::pair<std::string, std::string>& leading_obs_id,
    std::pair<bool, double> adc_merge_info) {
  UNUSED(obstacles_info);
  const auto* obs_interactive = path_decision.Find(interactive_obs_id);
  if (obs_interactive == nullptr) {
    return std::numeric_limits<double>::infinity();
  }
  ADEBUG << "adc_s=" << adc_s;
  const double obs_v = obs_interactive->speed();
  const double obs_s = (obs_interactive->PerceptionSLBoundary().start_s() +
                        obs_interactive->PerceptionSLBoundary().end_s()) *
                       kHalf;

  ADEBUG << "obs_s=" << obs_s;
  // double obs_st_min_s = obs_interactive->reference_line_st_boundary().min_s();
  double obs_st_min_s = std::numeric_limits<double>::infinity();
  if (adc_merge_info.first) {
    obs_st_min_s = adc_merge_info.second + adc_s;
  }
  ADEBUG << "obs_st_min_s=" << obs_st_min_s;
  // ADEBUG << "min_t=" << obs_interactive->reference_line_st_boundary().min_t();

  double obs_v_new = std::fmax(obs_v + obs_a * kMotionTime, kEpsilon);
  ADEBUG << "obs_v_new=" << obs_v_new;
  double obs_delta_s = obs_st_min_s - obs_s;

  double adc_v_new = std::fmax(adc_v + adc_a * kMotionTime, kEpsilon);
  ADEBUG << "adc_v_new=" << adc_v_new;
  double adc_delta_s = obs_st_min_s - adc_s;

  // get obs arrival time
  double obs_arrival_time = obs_delta_s / std::fmax(obs_v, kEpsilon);
  obs_arrival_time =
      CalArrivalTime(obs_v, obs_a, &obs_v_new, obs_delta_s, obs_arrival_time);
  ADEBUG << "obs_arrival_time=" << obs_arrival_time;
  // get adc arrival time
  double adc_arrival_time = adc_delta_s / std::fmax(adc_v, kEpsilon);
  adc_arrival_time =
      CalArrivalTime(adc_v, adc_a, &adc_v_new, adc_delta_s, adc_arrival_time);
  ADEBUG << "adc_arrival_time=" << adc_arrival_time;
  adc_is_latecomer_ = obs_arrival_time < adc_arrival_time + kEpsilon;
  ADEBUG << std::boolalpha << "adc_is_latecomer=" << adc_is_latecomer_;
  double latecomer_s = 0.0;
  if (obs_arrival_time < adc_arrival_time + kEpsilon) {
    // adc_is_latecomer_ = true
    if (obs_arrival_time < kMotionTime) {
      latecomer_s = adc_s + adc_v * obs_arrival_time +
                    0.5 * adc_a * obs_arrival_time * obs_arrival_time;
      adc_v_new = adc_v + adc_a * obs_arrival_time;
    } else {
      latecomer_s = adc_s + adc_v * kMotionTime +
                    0.5 * adc_a * kMotionTime * kMotionTime +
                    adc_v_new * (obs_arrival_time - kMotionTime);
    }
  } else {
    // adc_is_latecomer_ = false
    if (adc_arrival_time < kMotionTime) {
      latecomer_s = obs_s + obs_v * adc_arrival_time +
                    0.5 * obs_a * adc_arrival_time * adc_arrival_time;
      obs_v_new = obs_v + obs_a * adc_arrival_time;
    } else {
      latecomer_s = obs_s + obs_v * kMotionTime +
                    0.5 * obs_a * kMotionTime * kMotionTime +
                    obs_v_new * (adc_arrival_time - kMotionTime);
    }
  }

  const double delta_s = std::fmax(obs_st_min_s - latecomer_s, kEpsilon);
  const double delta_v = adc_v_new - obs_v_new;
  ADEBUG << "delta_s=" << delta_s;
  ADEBUG << "delta_v=" << delta_v;
  // judge whether the latecomer whose speed is higher can decrease
  // without collision when first one arrives
  double collision_cost_a = 0.0;
  if (((obs_arrival_time > adc_arrival_time + kEpsilon) &&
       (obs_v_new > adc_v_new + kEpsilon)) ||
      ((obs_arrival_time + kEpsilon < adc_arrival_time) &&
       (obs_v_new + kEpsilon < adc_v_new))) {
    const double a_decel = delta_v * delta_v / (2.0 * delta_s);
    ADEBUG << "a_decel=" << a_decel;
    if (a_decel > kAdcObsMaxDecelThd) {
      ADEBUG << "collision!!! adc or obs decel value is big";
      return std::numeric_limits<double>::infinity();
    }
    const double collision_a_cost_k = 5.0;
    const double collision_a_cost_a = 1.5;
    const double collision_a_cost_b = -2.0;
    collision_cost_a = collision_a_cost_k *
                       (common::math::Sigmoid(collision_a_cost_a * a_decel +
                                              collision_a_cost_b) -
                        common::math::Sigmoid(collision_a_cost_b));
    ADEBUG << "collision_cost_a=" << collision_cost_a;
  }

  double rear_v = adc_is_latecomer_ ? adc_v_new : obs_v_new;
  double front_v = !adc_is_latecomer_ ? adc_v_new : obs_v_new;
  // collision of obs and adc when one has arrived in obs_st_min_s
  const double s_safe_follow_distance =
      std::fmax(kTwoObsLgtSafeDistance, GetRSSLgtSafeDistance(rear_v, front_v));
  ADEBUG << "s_safe_follow_distance=" << s_safe_follow_distance;
  if (delta_s < s_safe_follow_distance) {
    ADEBUG << "collision!!! delta_s of adc and obs_inter is small";
    return std::numeric_limits<double>::infinity();
  }
  const double collision_s_cost_k = 10.0;
  const double collision_s_cost_a = -0.6;
  const double collision_s_cost_b = 0.9;
  double collision_cost_delta_s =
      collision_s_cost_k *
      common::math::Sigmoid((collision_s_cost_a)*delta_s + collision_s_cost_b);
  ADEBUG << "collision_cost_delta_s=" << collision_cost_delta_s;

  const double rear_speed_upp = GetRSSLgtSafeVelocity(front_v, delta_s);
  ADEBUG << "rear_speed_upp=" << rear_speed_upp;
  if (rear_v > rear_speed_upp) {
    ADEBUG << "collision!!! rear_v is over threshold";
    return std::numeric_limits<double>::infinity();
  }

  ADEBUG << "start to calculate collision of leading_obs";
  const auto& adc_leading_id = leading_obs_id.first;
  const auto& inter_leading_id = leading_obs_id.second;
  // first_s,first_arrival_time,first_v,first_v_new,first_a
  std::array<double, 5> obs_info = {obs_s, obs_arrival_time, obs_v, obs_v_new,
                                    obs_a};
  std::array<double, 5> adc_info = {adc_s, adc_arrival_time, adc_v, adc_v_new,
                                    adc_a};
  std::vector<double> obs_s_vec(kTrajSize);
  std::vector<double> adc_s_vec(kTrajSize);
  LongitudinalSFunction(obs_info, &obs_s_vec);
  LongitudinalSFunction(adc_info, &adc_s_vec);

  ADEBUG << "calculate collision of adc and adc_leading_obs";
  if (JudgeFrontCollision(adc_s_vec, path_decision, adc_leading_id)) {
    ADEBUG << "collision!!! delta_s of adc and adc_leading_obs is small";
    return std::numeric_limits<double>::infinity();
  }
  ADEBUG << "calculate collision of inter and inter_leading_obs";
  if (JudgeFrontCollision(obs_s_vec, path_decision, inter_leading_id)) {
    ADEBUG << "collision!!! delta_s of inter and inter_leading_obs is small";
    return std::numeric_limits<double>::infinity();
  }

  double collision_cost =
      kWeightSafety * (collision_cost_delta_s + collision_cost_a);
  ADEBUG << "GetSafetyCost=" << collision_cost;
  return collision_cost;
}

void BehaviorCost::LongitudinalSFunction(std::array<double, 5> obs_info,
                                         std::vector<double>* const obs_s_vec) {
  double obs_s = obs_info[0];
  double obs_arrival_time = obs_info[1];
  double obs_v = obs_info[2];
  double obs_v_new = obs_info[3];
  double obs_a = obs_info[4];

  const auto time_cycle = 1.0 / static_cast<double>(FLAGS_planning_loop_rate);
  const double acc_time = std::fmin(obs_arrival_time, kMotionTime);
  (*obs_s_vec)[0] = obs_s;
  for (int i = 1; i < kTrajSize; ++i) {
    if (time_cycle * i < acc_time) {
      (*obs_s_vec)[i] = obs_s + obs_v * (time_cycle * i) +
                        0.5 * obs_a * (time_cycle * i) * (time_cycle * i);
    } else {
      (*obs_s_vec)[i] = (*obs_s_vec)[i - 1] + obs_v_new * time_cycle;
    }
  }
}

bool BehaviorCost::JudgeFrontCollision(const std::vector<double>& first_one,
                                       const PathDecision& path_decision,
                                       const std::string& leading_obs_id) {
  const auto* leading_obs = path_decision.Find(leading_obs_id);
  if (leading_obs == nullptr) {
    return false;
  }
  const auto& leading_obs_traj = leading_obs->GetTrajectoryEnvelope();
  if (leading_obs_traj.empty()) {
    return false;
  }
  const size_t traj_size = leading_obs_traj.size();
  const size_t max_size = std::min<size_t>(traj_size, kTrajSize);
  for (size_t i = 0; i < max_size; ++i) {
    const double leading_obs_s = leading_obs_traj[i].center_p.s();
    const double delta_s = leading_obs_s - first_one[i];
    if ((delta_s < kTwoObsLgtSafeDistance && delta_s > kEpsilon) ||
        delta_s < kEpsilon) {
      return true;
    }
  }
  return false;
}

bool BehaviorCost::JudgeLeadingObsCollision(int first_time_index,
                                            double first_v,
                                            double merge_point_s,
                                            const Obstacle* leading_obstacle) {
  if (leading_obstacle == nullptr ||
      leading_obstacle->GetTrajectoryEnvelope().empty()) {
    return false;
  }
  const auto& leading_obs_traj = leading_obstacle->GetTrajectoryEnvelope();
  const auto traj_size = static_cast<int>(leading_obs_traj.size());
  const auto traj_index = std::max(
      (traj_size <= first_time_index ? traj_size - 1 : first_time_index), 0);
  const double leading_obs_s = leading_obs_traj.at(traj_index).center_p.s();
  ADEBUG << "leading_obs_s=" << leading_obs_s;

  double leading_v = leading_obstacle->speed();
  if (!leading_obstacle->Trajectory().trajectory_point().empty()) {
    const auto traj_point_size =
        leading_obstacle->Trajectory().trajectory_point().size();
    const auto traj_point_index =
        std::max(traj_point_size <= first_time_index ? traj_point_size - 1
                                                     : first_time_index,
                 0);
    leading_v = leading_obstacle->Trajectory()
                    .trajectory_point()
                    .at(traj_point_index)
                    .v();
  }
  ADEBUG << "leading_v=" << leading_v;

  const double s_safe_follow_distance = std::fmax(
      kTwoObsLgtSafeDistance, GetRSSLgtSafeDistance(first_v, leading_v));
  ADEBUG << "s_safe_follow_distance=" << s_safe_follow_distance;
  const double leading_obs_delta_s =
      std::fmax(leading_obs_s - merge_point_s, kEpsilon);
  if (leading_obs_delta_s < s_safe_follow_distance) {
    ADEBUG << "collision!!! delta_s of rear and leading_obs is small";
    return true;
  }

  const double rear_speed_upp =
      GetRSSLgtSafeVelocity(leading_v, leading_obs_delta_s);
  ADEBUG << "rear_speed_upp=" << rear_speed_upp;
  if (first_v > rear_speed_upp) {
    ADEBUG << "collision!!! rear_v with leading_obs is over threshold";
    return true;
  }
  return false;
}

double BehaviorCost::GetComfortCost(const double adc_a, const double obs_a) {
  // comfort reduces in the order of ConstantSpeed, Accelerate, Decelerate
  constexpr double kDecelAccelRatio = 2.0;
  double adc_a_cost = std::fabs(adc_a);
  double obs_a_cost = std::fabs(obs_a);
  if (adc_a < 0) {
    adc_a_cost *= kDecelAccelRatio;
  }

  obs_a_cost *= kDecelAccelRatio;

  // prefer [adc:Accel, obs:ConstSpeed] not [adc:ConstSpeed,limit obs:Accel]
  double adc_weight = 0.1;
  double obs_weight = 0.4;
  double cost_comfort =
      kWeightComfort *
      std::fmin((adc_weight * adc_a_cost + obs_weight * obs_a_cost), 1.0);
  ADEBUG << "GetComfortCost=" << cost_comfort;
  return cost_comfort;
}

double BehaviorCost::GetEfficiencyCost(
    const double adc_a, const double time_gap, const double obs_a,
    const ReferenceLineInfo& reference_line_info,
    const std::string& interactive_obs_id,
    const std::unordered_map<std::string, ObstacleInfo>& obstacles_info) {
  const double adc_speed =
      reference_line_info.vehicle_state().linear_velocity();
  const double adc_v_avg = adc_speed + adc_a * time_gap * kHalf;
  ADEBUG << "adc_v_avg=" << adc_v_avg;
  const double adc_v_limit =
      std::fmin(reference_line_info.GetCruiseSpeed(),
                reference_line_info.reference_line().GetLaneSpeedLimitFromS(
                    reference_line_info.reference_line().Length()));
  ADEBUG << "adc_v_limit=" << adc_v_limit;
  const double adc_efficiency_cost_k = 1.0;
  const double adc_efficiency_cost_a = 0.35;
  const double adc_efficiency_cost_b = -4.0;
  double adc_efficiency_cost =
      adc_efficiency_cost_k *
      (common::math::Sigmoid(adc_efficiency_cost_a *
                                 std::fabs((adc_v_limit - adc_v_avg)) +
                             adc_efficiency_cost_b) -
       common::math::Sigmoid(adc_efficiency_cost_b));
  ADEBUG << "adc_efficiency_cost=" << adc_efficiency_cost;

  // !!! via pnc_map to get obs_v_limit and whether take obs_efficiency_cost into account
  const auto& obs = obstacles_info.find(interactive_obs_id);
  if (obs == obstacles_info.end()) {
    return std::numeric_limits<double>::infinity();
  }
  const double obs_v_avg = obs->second.speed + obs_a * time_gap * kHalf;
  ADEBUG << "obs_v_avg=" << obs_v_avg;
  // obs_v_limit
  const double obs_v_limit = adc_v_limit;
  const double obs_efficiency_cost_k = 1.0;
  const double obs_efficiency_cost_a = 0.35;
  const double obs_efficiency_cost_b = -4.0;
  double obs_efficiency_cost =
      obs_efficiency_cost_k *
      (common::math::Sigmoid(obs_efficiency_cost_a *
                                 std::fabs((obs_v_limit - obs_v_avg)) +
                             obs_efficiency_cost_b) -
       common::math::Sigmoid(obs_efficiency_cost_b));
  ADEBUG << "obs_efficiency_cost=" << obs_efficiency_cost;

  double adc_weight = 1.0;
  double obs_weight = 0.0;
  double cost_efficiency =
      kWeightEfficiency *
      (adc_weight * adc_efficiency_cost + obs_weight * obs_efficiency_cost);
  ADEBUG << "GetEfficiencyCost=" << cost_efficiency;
  return cost_efficiency;
}

double BehaviorCost::GetConsistencyCost(
    const InteractiveObsBehavior::Behavior& adc_behavior,
    const InteractiveObsBehavior::Behavior& last_adc_behavior) {
  // (last_behavior,current_behavior)
  // 0 means ConstantSpeed, + means Accelerate, - means Decelerate
  // {(0,0),(0,+),(0,-),(+,0),(+,+),(+,-),(-,0),(-,+),(-,-)}
  std::array<std::array<double, 3>, 3> calibration_table = {
      {{0, 0.15, 0.25}, {0.05, 0.1, 0.5}, {0.05, 0.5, 0.2}}};
  std::unordered_map<InteractiveObsBehavior::Behavior, int> behavior_map = {
      {InteractiveObsBehavior::ConstantSpeed, 0},
      {InteractiveObsBehavior::Accelerate, 1},
      {InteractiveObsBehavior::Decelerate, 2}};
  const auto& last_iter = behavior_map.find(last_adc_behavior);
  const auto& current_iter = behavior_map.find(adc_behavior);
  double adc_consistency_cost = 0.0;
  if (last_iter != behavior_map.end() && current_iter != behavior_map.end()) {
    adc_consistency_cost =
        kWeightConsistency *
        (calibration_table.at(last_iter->second).at(current_iter->second));
  }
  ADEBUG << "GetConsistencyCost=" << adc_consistency_cost;
  // if consider obs_consistency, should use PerceptionID
  return adc_consistency_cost;
}

}  // namespace planning
}  // namespace TL
