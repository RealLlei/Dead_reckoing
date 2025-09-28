/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2024. All rights reserved.
 * Description:  obstacle_game_decider.cc
 */
#include "planning/tasks/deciders/obstacle_game_decider/obstacle_game_decider.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>
#include "common/math/box2d.h"
#include "common/math/linear_interpolation.h"
#include "common/math/math_utils.h"
#include "common/math/vec2d.h"
#include "common/util/util.h"
#include "planning/common/frame.h"
#include "planning/common/game/game_common/rss/responsibility_sensitive_safety.h"
#include "planning/common/game/game_common/semantics/semantics.h"
#include "planning/common/obstacle.h"
#include "planning/common/obstacle_blocking_analyzer.h"
#include "planning/common/reference_line_info.h"
#include "planning/localview/state_machine/initial_state.h"
#include "planning/tasks/deciders/obstacles_decider/obstacle_selector/obstacle_selector.h"
#include "planning/proto/learning_data.pb.h"
#include "proto/common/pnc_point.pb.h"
#include "proto/planning/decider_debug.pb.h"

namespace TL::planning {
using common::math::Box2d;
using game_common::ForwardSimAgent;
using game_common::IndexedMap;
using game_common::kEgoAgentId;
using game_common::kInvalidAgentId;
using game_common::kInvalidIndex;
using game_common::LateralBehavior;
using game_common::RssChecker;
using game_common::Vehicle;
using game_common::VehicleParam;
using TL::common::math::double_type::DefinitelyGreaterEqual;
using planning::ObstacleSelector;
using std::unordered_map;
using std::vector;

namespace {
constexpr double kMaxSimObstacleDistance = 100.0;
constexpr double kMinSimObstacleDistance = 10.0;
constexpr double kGameTtc = 6.0;
constexpr double kCutinCooperationProb = 0.8;
constexpr int kCloseToCounter = 5;
}  // namespace

ObstacleGameDecider::ObstacleGameDecider(
    const TaskConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : Decider(config, injector),
      sim_time_resolution_(
          config.obstacle_game_decider_config().sim_time_resolution()),
      sim_step_size_(config.obstacle_game_decider_config().sim_step_size()),
      enable_obstacle_game_decider_(config.obstacle_game_decider_config()
                                        .enable_obstacle_game_decider()) {
  const auto& vehicle_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  ego_vehicle_param_.set_length(vehicle_param.length());
  ego_vehicle_param_.set_width(vehicle_param.width());
  ego_vehicle_param_.set_wheel_base(vehicle_param.wheel_base());
  ego_vehicle_param_.set_max_lateral_acc(2.0);
  ego_vehicle_param_.set_max_longitudinal_acc(vehicle_param.max_acceleration());
  ego_vehicle_param_.set_d_cr(vehicle_param.back_edge_to_center());
  ego_vehicle_param_.set_max_steering_angle(vehicle_param.max_steer_angle());
  sample_acc_.clear();
  for (const auto acc : config.obstacle_game_decider_config().sample_acc()) {
    sample_acc_.push_back(acc);
  }
}

double ObstacleGameDecider::CalculateCutinProbByIntentionModel(
    const Obstacle& obstacle) const {
  const double obstacle_s = 0.5 * (obstacle.PerceptionSLBoundary().start_s() +
                                   obstacle.PerceptionSLBoundary().end_s());
  const double obstacle_start_l = obstacle.PerceptionSLBoundary().start_l();
  const double obstacle_end_l = obstacle.PerceptionSLBoundary().end_l();
  const double obstacle_middle_l = (obstacle_start_l + obstacle_end_l) * 0.5;
  const bool is_left_obstacle = obstacle_middle_l > 0.0;
  ADEBUG << "obs id: " << obstacle.Id() << ", start l: " << obstacle_start_l
         << ", end l: " << obstacle_end_l;
  constexpr double kLaneInsideDis = 0.40;
  constexpr double kLaneOutsideDis = 0.60;
  double lat_dis = std::numeric_limits<double>::max();
  if (is_left_obstacle && obstacle_start_l > 0.0 &&
      obstacle_start_l < lane_left_width_ + kLaneOutsideDis) {
    lat_dis = obstacle_start_l - lane_left_width_ + kLaneInsideDis;
  } else if (!is_left_obstacle &&
             obstacle_end_l > -lane_right_width_ - kLaneOutsideDis &&
             obstacle_end_l < 0.0) {
    lat_dis = -lane_right_width_ - obstacle_end_l + kLaneInsideDis;
  }
  const vector<double> lat_x = {0.0, kLaneInsideDis + kLaneOutsideDis};
  const vector<double> lat_y = {1.0, 0.0};
  if (lat_dis > kLaneInsideDis + kLaneOutsideDis) {
    lat_dis = kLaneInsideDis + kLaneOutsideDis;
  }
  const double prob_lat = common::math::InterpolationOne(lat_dis, lat_x, lat_y);

  const double obs_heading =
      common::math::NormalizeAngle(obstacle.PerceptionBoundingBox().heading());
  const double ref_point_heading =
      common::math::NormalizeAngle(reference_line_info_->reference_line()
                                       .GetReferencePoint(obstacle_s)
                                       .heading());
  double heading_diff =
      common::math::NormalizeAngle(obs_heading - ref_point_heading);
  if (is_left_obstacle && (heading_diff > 0.0 || heading_diff < -M_PI_4)) {
    heading_diff = 0.0;
  }
  if (!is_left_obstacle && (heading_diff < 0.0 || heading_diff > M_PI_4)) {
    heading_diff = 0.0;
  }
  heading_diff = std::fabs(heading_diff);

  constexpr double kMaxCutinHeading = 25.0 * M_PI / 180.0;
  const vector<double> heading_x = {0.0, kMaxCutinHeading};
  const vector<double> heading_y = {0.0, 1.0};
  const double prob_heading =
      common::math::InterpolationOne(heading_diff, heading_x, heading_y);

  const double lat_speed = std::fabs(obstacle.speed() * sin(heading_diff));

  constexpr double kMaxCutinLatSpeed = 1.5;
  const vector<double> lat_speed_x = {0.0, kMaxCutinLatSpeed};
  const vector<double> lat_speed_y = {0.0, 1.0};
  const double prob_lat_speed =
      common::math::InterpolationOne(lat_speed, lat_speed_x, lat_speed_y);

  const double prob_cutin =
      1.0 - (1.0 - prob_lat) * (1.0 - prob_heading) * (1.0 - prob_lat_speed);
  ADEBUG << "###obs id: " << obstacle.Id() << ", pro cutin: " << prob_cutin
         << ", prob lat dis: " << prob_lat << ", lat dis: " << lat_dis
         << ", prob heading:" << prob_heading
         << ", heading diff: " << heading_diff
         << ", prob lat speed: " << prob_lat_speed
         << ", lat speed: " << lat_speed;
  return prob_cutin;
}

void ObstacleGameDecider::GetCutinObstacle(
    const Frame& frame, const ReferenceLineInfo& reference_line_info,
    std::unordered_set<int>* const cutin_obstacle) const {
  if (cutin_obstacle == nullptr) {
    return;
  }

  ADEBUG << "obstacle game decider start!";
  *cutin_obstacle = frame.GetObstacleInfo().GetCutinObsPerceptionIds();
  std::string follow_obs_id = frame.GetObstacleInfo().GetNearestBlockingObs();
  const auto* const follow_obs =
      reference_line_info.path_decision().obstacles().Find(follow_obs_id);
  constexpr double kFrontIgnoreDistance = 15.0;
  constexpr double kBackIgnoreDistance = 0.0;
  constexpr double kInMergeRoadIgnoreDistance = 50.0;  // TBD
  for (const Obstacle* obstacle :
       reference_line_info.path_decision().obstacles().Items()) {
    if (obstacle == nullptr) {
      continue;
    }
    if (obstacle->IsIgnore()) {
      ADEBUG << "obs id: " << obstacle->Id() << ", ignore!";
      cutin_obstacle->erase(obstacle->PerceptionId());
      continue;
    }
    if (obstacle->IsStatic()) {
      ADEBUG << "obs id: " << obstacle->Id() << ", static!";
      cutin_obstacle->erase(obstacle->PerceptionId());
      continue;
    }
    // FOLLOW前的障碍物忽略
    if (follow_obs != nullptr &&
        obstacle->PerceptionSLBoundary().start_s() >
            follow_obs->PerceptionSLBoundary().end_s()) {
      cutin_obstacle->erase(obstacle->PerceptionId());
      ADEBUG << "obs id: " << obstacle->Id() << " , before follow.";
      continue;
    }
    if (vehicle_in_merge_lane_ &&
        !obstacle->reference_line_st_boundary().IsEmpty()) {
      if (obstacle->reference_line_st_boundary().min_s() +
              kInMergeRoadIgnoreDistance >
          first_merge_point_s_) {
        cutin_obstacle->erase(obstacle->PerceptionId());
        ADEBUG << "obs id: " << obstacle->Id() << " , merge";
        continue;
      }
    }

    const double ego_s = (reference_line_info.AdcSlBoundary().start_s() +
                          reference_line_info.AdcSlBoundary().end_s()) *
                         0.5;
    const double obstacle_s =
        0.5 * (obstacle->PerceptionSLBoundary().start_s() +
               obstacle->PerceptionSLBoundary().end_s());
    const bool is_front_obs = obstacle_s > ego_s;

    if (is_front_obs) {
      const double obs_distance_to_adc =
          GetDistanceBetweenADCAndObstacle(&reference_line_info, obstacle);
      const double ttc =
          common::util::IsFloatEqual(
              obstacle->speed(),
              reference_line_info.vehicle_state().linear_velocity())
              ? std::numeric_limits<double>::max()
              : obs_distance_to_adc /
                    (reference_line_info.vehicle_state().linear_velocity() -
                     obstacle->speed());
      ADEBUG << "obs id: " << obstacle->Id() << ", ttc: " << ttc;
      if (obstacle->PerceptionSLBoundary().start_s() -
                  reference_line_info.AdcSlBoundary().end_s() >
              kFrontIgnoreDistance + obstacle->speed() &&
          (ttc > kGameTtc || ttc < 0.0)) {
        cutin_obstacle->erase(obstacle->PerceptionId());
        ADEBUG << "obs id: " << obstacle->Id() << ", front away! "
               << "ttc: " << ttc;
        continue;
      }
    } else {
      if (reference_line_info.AdcSlBoundary().start_s() -
              obstacle->PerceptionSLBoundary().end_s() >
          kBackIgnoreDistance) {
        cutin_obstacle->erase(obstacle->PerceptionId());
        ADEBUG << "obs id: " << obstacle->Id() << ", back away!";
        continue;
      }
    }

    const double start_l = obstacle->PerceptionSLBoundary().start_l();
    const double end_l = obstacle->PerceptionSLBoundary().end_l();
    const double middle_l = (start_l + end_l) * 0.5;
    if (middle_l < lane_left_width_ && middle_l > -lane_right_width_) {
      cutin_obstacle->erase(obstacle->PerceptionId());
      ADEBUG << "obs id: " << obstacle->Id() << ", block adc!";
    }
  }
}

void ObstacleGameDecider::ProcessDangerousApproachingCutinObstacles(
    const std::unordered_set<int>& cutin_obstacle) {
  std::unordered_set<int> processed;
  constexpr int kHastrend = 3;
  constexpr double kConsiderCloseToTrendDist = 0.05;
  for (const auto& obstacle :
       reference_line_info_->path_decision()->obstacles().Items()) {
    if (obstacle == nullptr) {
      continue;
    }
    const double start_l = obstacle->PerceptionSLBoundary().start_l();
    const double end_l = obstacle->PerceptionSLBoundary().end_l();
    const bool is_left_obstacle = (start_l + end_l) * 0.5 > 0.0;
    const double curr_l = is_left_obstacle ? start_l : end_l;
    if (processed.find(obstacle->PerceptionId()) != processed.end()) {
      continue;
    }
    if (obstacle->PerceptionSLBoundary().start_s() +
            obstacle->PerceptionSLBoundary().end_s() <
        reference_line_info_->AdcSlBoundary().start_s() +
            reference_line_info_->AdcSlBoundary().end_s()) {
      continue;
    }
    if (start_l > lane_left_width_ + kConsiderCloseToTrendDist ||
        end_l < -lane_right_width_ - kConsiderCloseToTrendDist) {
      close_to_counter_.erase(obstacle->PerceptionId());
      continue;
    }
    if (cutin_obstacle.find(obstacle->PerceptionId()) == cutin_obstacle.end()) {
      close_to_counter_.erase(obstacle->PerceptionId());
      continue;
    }
    if (close_to_counter_.find(obstacle->PerceptionId()) ==
        close_to_counter_.end()) {
      close_to_counter_[obstacle->PerceptionId()].first = 0;
      close_to_counter_[obstacle->PerceptionId()].second = curr_l;
    } else {
      auto& obs_history_close_to_counter =
          close_to_counter_[obstacle->PerceptionId()].first;
      auto& obs_history_close_to_l =
          close_to_counter_[obstacle->PerceptionId()].second;
      const double maintain_trend_distance =
          obs_history_close_to_counter >= kHastrend ? -0.005 : 0.01;
      if (is_left_obstacle) {
        if (obs_history_close_to_l - curr_l > maintain_trend_distance) {
          obs_history_close_to_counter =
              std::min(obs_history_close_to_counter + 1, kCloseToCounter);
        } else {
          obs_history_close_to_counter = 0;
        }
        obs_history_close_to_l = curr_l;
      } else {
        ADEBUG << "obs id: " << obstacle->Id() << " at right, cur l: " << curr_l
               << ", history l: " << obs_history_close_to_l;
        if (curr_l - obs_history_close_to_l > maintain_trend_distance) {
          obs_history_close_to_counter =
              std::min(obs_history_close_to_counter + 1, kCloseToCounter);
        } else {
          obs_history_close_to_counter = 0;
        }
        obs_history_close_to_l = curr_l;
      }
    }
    processed.emplace(obstacle->PerceptionId());
  }
  for (auto [id, close_to_info] : close_to_counter_) {
    ADEBUG << "id: " << id << ", close to times: " << close_to_info.first;
  }
}

common::Status ObstacleGameDecider::Process(
    Frame* frame, ReferenceLineInfo* reference_line_info) {
  if (reference_line_info_ == nullptr || !enable_obstacle_game_decider_ ||
      frame == nullptr || frame_ == nullptr || reference_line_info == nullptr ||
      injector_ == nullptr ||
      injector_->vehicle_state()->driving_mode() !=
          TL::soc::Chassis::COMPLETE_AUTO_DRIVE) {
    return Status::OK();
  }
  if (!ObstacleSelector::GetMapInfo(
          0.5 * (reference_line_info_->AdcSlBoundary().start_s() +
                 reference_line_info_->AdcSlBoundary().end_s()),
          *reference_line_info_, &lane_left_width_, &lane_right_width_,
          &left_neighbor_lane_width_, &right_neighbor_lane_width_)) {
    ADEBUG << "can't GetMapInfo.";
    return Status::OK();
  }
  ADEBUG << "lane left width: " << lane_left_width_
         << ", lane right width: " << lane_right_width_;
  const auto& change_lane =
      injector_->planning_context()->planning_status().change_lane();
  const auto& change_lane_reason = change_lane.change_lane_reason();
  const auto& lane_change_status = change_lane.status();
  if (lane_change_status == ChangeLaneStatus::IN_CHANGE_LANE ||
      change_lane_reason != functionmanager::HmiChangeLaneReason::HMI_NONE) {
    ADEBUG << "adc is in change lane";
    return common::Status::OK();
  }
  const auto& adc_sl_boundary = reference_line_info->AdcSlBoundary();
  const double adc_end_s = adc_sl_boundary.end_s();
  const auto& adc_map_common_info =
      reference_line_info->reference_line().GetAdcMapCommonInfo();
  ADEBUG << "distance 2 merge begin: "
         << adc_map_common_info.dis_first_merge_begin_point;
  vehicle_in_merge_lane_ = adc_map_common_info.vehicle_in_merge_lane;
  first_merge_point_s_ =
      std::fmin(adc_map_common_info.dis_first_merge_point,
                adc_map_common_info.dis_first_merge_begin_point) +
      adc_end_s;
  std::unordered_set<int> cutin_obstacle;
  std::unordered_set<int> mergein_obstacle;
  std::unordered_set<int> cross_obstacle;
  GetCutinObstacle(*frame, *reference_line_info, &cutin_obstacle);
  ProcessDangerousApproachingCutinObstacles(cutin_obstacle);
  GetMergeinObstacle(*frame, *reference_line_info, &mergein_obstacle);
  ObsFilterConflict(&cutin_obstacle, &mergein_obstacle);

  if (!cutin_obstacle.empty() || !mergein_obstacle.empty()) {
    for (auto id : cutin_obstacle) {
      ADEBUG << "###cutin obs id: " << id;
    }
    for (auto id : mergein_obstacle) {
      ADEBUG << "###mergein obs id: " << id;
    }
    Run(reference_line_info, cutin_obstacle, mergein_obstacle, cross_obstacle);
  }
  return Status::OK();
}

void ObstacleGameDecider::GetMergeinObstacle(
    const Frame& frame, const ReferenceLineInfo& reference_line_info,
    std::unordered_set<int>* mergein_obstacle) {
  merge_lane_vehicles_.Clear();
  if (mergein_obstacle == nullptr) {
    return;
  }
  const double adc_speed =
      reference_line_info.vehicle_state().linear_velocity();
  const double kTTM = 10.0;
  const double kDistance2MergePoint = 100.0;
  const auto& adc_sl_boundary = reference_line_info.AdcSlBoundary();
  const double adc_end_s = adc_sl_boundary.end_s();
  if (!vehicle_in_merge_lane_) {
    ADEBUG << "vehicle_in_merge_lane: " << vehicle_in_merge_lane_;
    return;
  }
  if (first_merge_point_s_ - adc_end_s >
      std::fmax(adc_speed * kTTM, kDistance2MergePoint)) {
    ADEBUG << "not in merge ODD";
    return;
  }
  // mergein obs sorted via s
  auto mergein_front = frame.GetObstacleInfo().GetNeighborFrontMergeObs();
  auto mergein_back = frame.GetObstacleInfo().GetNeighborBackMergeObs();
  sort(mergein_front.begin(), mergein_front.end(),
       [](std::pair<std::string, double>& left,
          std::pair<std::string, double>& right) {
         return left.second > right.second;
       });
  sort(mergein_back.begin(), mergein_back.end(),
       [](std::pair<std::string, double>& left,
          std::pair<std::string, double>& right) {
         return left.second > right.second;
       });
  ADEBUG << "merge front num: " << mergein_front.size();
  ADEBUG << "merge back num: " << mergein_back.size();
  for (const auto& [id, s] : mergein_front) {
    ADEBUG << "mergein_front id:" << id;
  }
  for (const Obstacle* obstacle :
       reference_line_info.path_decision().obstacles().Items()) {
    if (obstacle == nullptr || obstacle->IsVirtual() || obstacle->IsIgnore() ||
        obstacle->IsStatic()) {
      continue;
    }
    const double start_s = obstacle->PerceptionSLBoundary().start_s();
    const double end_s = obstacle->PerceptionSLBoundary().end_s();
    const double middle_s = (start_s + end_s) * 0.5;
    const double start_l = obstacle->PerceptionSLBoundary().start_l();
    const double end_l = obstacle->PerceptionSLBoundary().end_l();
    const double middle_l = (start_l + end_l) * 0.5;
    if (!mergein_back.empty() && mergein_back.front().first == obstacle->Id()) {
      mergein_obstacle->emplace(obstacle->PerceptionId());
      merge_lane_vehicles_.Insert(obstacle->PerceptionId(), middle_s);
    }
    if (!mergein_front.empty()) {
      if (mergein_front.back().first == obstacle->Id()) {
        if (middle_l > lane_left_width_ || middle_l < -lane_right_width_) {
          mergein_obstacle->emplace(obstacle->PerceptionId());
        }
        merge_lane_vehicles_.Insert(obstacle->PerceptionId(), middle_s);
      }
      // the leading of front mergein_obs, not in mergein_obstacle
      if (mergein_front.size() > 1) {
        if (mergein_front.at(mergein_front.size() - 2).first ==
            obstacle->Id()) {
          merge_lane_vehicles_.Insert(obstacle->PerceptionId(), middle_s);
        }
      }
    }
  }
}

void ObstacleGameDecider::ObsFilterConflict(
    std::unordered_set<int>* cutin_obstacle,
    std::unordered_set<int>* mergein_obstacle) {
  std::unordered_set<int> remove_cutin;
  for (const auto cut_in_obs : *cutin_obstacle) {
    // give priority to mergein
    if (mergein_obstacle->find(cut_in_obs) != mergein_obstacle->end()) {
      remove_cutin.emplace(cut_in_obs);
    }
  }
  for (const auto cut_in : remove_cutin) {
    cutin_obstacle->erase(cut_in);
  }
}

void ObstacleGameDecider::PrepareEgoAgent() {
  ego_sim_agents_.clear();
  // 构造 ego agent
  ForwardSimAgent ego_agent;
  ego_agent.vehicle.set_param(ego_vehicle_param_);
  // ego agent id 默认为 -2
  ego_agent.vehicle.set_id(kEgoAgentId);
  const auto& vehicle_state = reference_line_info_->vehicle_state();
  CartesianState state;
  state.acceleration = vehicle_state.linear_acceleration();
  state.velocity = vehicle_state.linear_velocity();
  state.angle = vehicle_state.heading();
  state.kappa = vehicle_state.kappa();
  state.position = {vehicle_state.x(), vehicle_state.y()};
  state.steer = vehicle_state.steering_percentage() *
                ego_vehicle_param_.max_steer_angle();
  state.time_stamp = 0.0;
  // sl坐标为车辆几何中心
  game_common::FrenetState frenet_state;
  frenet_state.s_state.at(0) =
      (reference_line_info_->AdcSlBoundary().start_s() +
       reference_line_info_->AdcSlBoundary().end_s()) *
      0.5;
  frenet_state.l_state.at(0) =
      (reference_line_info_->AdcSlBoundary().start_l() +
       reference_line_info_->AdcSlBoundary().end_l()) *
      0.5;
  ego_agent.vehicle.set_frenet_state(frenet_state);
  ego_agent.vehicle.set_state(state);

  ego_agent.lat_behavior = LateralBehavior::kLaneKeeping;
  ego_agent.lat_mode = game_common::LatSimMode::kAlongSide;

  ego_agent.sim_param.idm_param.vehicle_length = ego_vehicle_param_.length();

  ego_sim_agents_.reserve(sample_acc_.size());
  for (const auto& acc : sample_acc_) {
    ego_agent.vehicle.mutable_state().acceleration = acc;
    ego_sim_agents_.emplace_back(ego_agent);
  }
}

void ObstacleGameDecider::PrepareSurroundingAgent(
    const Obstacle& obstacle, const std::unordered_set<int>& cutin_obstacle,
    const std::unordered_set<int>& mergein_obstacle,
    const std::unordered_set<int>& cross_obstacle) {
  ForwardSimAgent agent;
  agent.vehicle.set_id(obstacle.PerceptionId());
  VehicleParam vehicle_param;
  vehicle_param.set_length(obstacle.PerceptionBoundingBox().length());
  vehicle_param.set_width(obstacle.PerceptionBoundingBox().width());
  agent.vehicle.set_param(vehicle_param);
  CartesianState state;
  state.position = {obstacle.Perception().position().x(),
                    obstacle.Perception().position().y()};
  state.velocity = obstacle.speed();
  state.angle = obstacle.Perception().theta();
  state.steer = 0.0;
  agent.vehicle.set_state(state);
  game_common::FrenetState frenet_state;
  frenet_state.s_state.at(0) =
      0.5 * (obstacle.PerceptionSLBoundary().start_s() +
             obstacle.PerceptionSLBoundary().end_s());
  frenet_state.l_state.at(0) =
      0.5 * (obstacle.PerceptionSLBoundary().start_l() +
             obstacle.PerceptionSLBoundary().end_l());
  agent.vehicle.set_frenet_state(frenet_state);
  agent.vehicle.set_is_overside_vehicle(obstacle.IsOversizedVehicle());

  if (cutin_obstacle.find(agent.vehicle.id()) != cutin_obstacle.end()) {
    if (left_lane_vehicles_.GetIndexByKey(agent.vehicle.id()) !=
        kInvalidIndex) {
      agent.lat_mode = game_common::LatSimMode::kCutIn;
      agent.lat_behavior = LateralBehavior::kLaneChangeRight;
    }

    if (right_lane_vehicles_.GetIndexByKey(agent.vehicle.id()) !=
        kInvalidIndex) {
      agent.lat_mode = game_common::LatSimMode::kCutIn;
      agent.lat_behavior = LateralBehavior::kLaneChangeLeft;
    }
  } else if (mergein_obstacle.find(agent.vehicle.id()) !=
             mergein_obstacle.end()) {
    agent.lat_mode = game_common::LatSimMode::kMergeIn;
    agent.lat_behavior = LateralBehavior::kLaneKeeping;
  } else if (cross_obstacle.find(agent.vehicle.id()) != cross_obstacle.end()) {
    agent.lat_mode = game_common::LatSimMode::kCross;
    agent.lat_behavior = LateralBehavior::kLaneKeeping;
  } else {
    agent.lat_mode = game_common::LatSimMode::kAlongSide;
    agent.lat_behavior = LateralBehavior::kLaneKeeping;
  }
  agent.sim_param.idm_param.desired_velocity = obstacle.speed();
  agent.sim_param.idm_param.vehicle_length =
      obstacle.PerceptionBoundingBox().length();
  agent.sim_param.init_cutin_prob =
      CalculateCutinProbByIntentionModel(obstacle);
  // 危险接近的障碍物不改变切入意图
  if (close_to_counter_.find(obstacle.PerceptionId()) !=
          close_to_counter_.end() &&
      close_to_counter_[obstacle.PerceptionId()].first == kCloseToCounter) {
    agent.sim_param.init_cutin_prob = 1.0;
  }
  ADEBUG << "###obs id: " << obstacle.Id()
         << ", init cutin prob: " << agent.sim_param.init_cutin_prob;
  surrounding_sim_agents_.emplace(agent.vehicle.id(), agent);
}

void ObstacleGameDecider::BuildingSimEnvironment(
    const std::unordered_set<int>& cutin_obstacle,
    const std::unordered_set<int>& mergein_obstacle,
    const std::unordered_set<int>& cross_obstacle) {
  PrepareEgoAgent();
  ego_lane_vehicles_.Clear();
  left_lane_vehicles_.Clear();
  right_lane_vehicles_.Clear();
  // merge_lane_vehicles_ have been updated in GetMergeinObstacle
  for (const auto& obstacle :
       reference_line_info_->path_decision()->obstacles().Items()) {
    if (obstacle == nullptr) {
      continue;
    }
    if (obstacle->IsVirtual()) {
      continue;
    }
    const double start_s = obstacle->PerceptionSLBoundary().start_s();
    const double end_s = obstacle->PerceptionSLBoundary().end_s();
    const double start_l = obstacle->PerceptionSLBoundary().start_l();
    const double end_l = obstacle->PerceptionSLBoundary().end_l();
    if (start_s - reference_line_info_->AdcSlBoundary().end_s() >
            kMaxSimObstacleDistance ||
        reference_line_info_->AdcSlBoundary().start_s() - end_s >
            kMinSimObstacleDistance) {
      continue;
    }
    const double middle_l = (start_l + end_l) * 0.5;
    const double middle_s = (start_s + end_s) * 0.5;
    if (middle_l > left_neighbor_lane_width_ + lane_left_width_ ||
        middle_l < -right_neighbor_lane_width_ - lane_right_width_) {
      continue;
    }

    if (middle_l < lane_left_width_ && middle_l > -lane_right_width_) {
      ego_lane_vehicles_.Insert(obstacle->PerceptionId(), middle_s);
    } else if (middle_l >= lane_left_width_) {
      left_lane_vehicles_.Insert(obstacle->PerceptionId(), middle_s);
    } else {
      right_lane_vehicles_.Insert(obstacle->PerceptionId(), middle_s);
    }
  }
  // 当前只放出来cutin和mergein
  FilterObstacle(cutin_obstacle, mergein_obstacle, cross_obstacle);
}

void ObstacleGameDecider::FilterEgoLane() {
  // 自车车道只保留自车前车
  const double ego_s = 0.5 * (reference_line_info_->AdcSlBoundary().start_s() +
                              reference_line_info_->AdcSlBoundary().end_s());
  auto ego_front_obs_id =
      ego_lane_vehicles_.GetPreviousAndNextKeys(kEgoAgentId, ego_s).second;
  auto index = ego_lane_vehicles_.GetIndexByKey(ego_front_obs_id);
  auto ego_front_obs_info = ego_lane_vehicles_.GetElementByIndex(index);
  ADEBUG << "ego_front_obs_id: " << ego_front_obs_info.first;
  ego_lane_vehicles_.Clear();
  ego_lane_vehicles_.Insert(ego_front_obs_info.first,
                            ego_front_obs_info.second);
}

void ObstacleGameDecider::FilterNeighborLane(
    const std::unordered_set<int>& cutin_obstacles,
    game_common::IndexedMap* const neighbor_lane_vehicles) {
  // 邻车道只保留cutin车及cutin前车
  if (neighbor_lane_vehicles == nullptr) {
    return;
  }
  std::vector<std::pair<int, double>> obs_infos;
  for (const auto& obstacle_id : cutin_obstacles) {
    const auto& index = neighbor_lane_vehicles->GetIndexByKey(obstacle_id);
    if (index == kInvalidIndex) {
      continue;
    }
    const auto& cutin_obs_info =
        neighbor_lane_vehicles->GetElementByIndex(index);
    if (cutin_obs_info.first == kInvalidAgentId) {
      continue;
    }
    obs_infos.push_back(cutin_obs_info);

    const auto& cutin_front_obs_id =
        neighbor_lane_vehicles
            ->GetPreviousAndNextKeys(cutin_obs_info.first,
                                     cutin_obs_info.second)
            .second;
    if (cutin_front_obs_id == kInvalidAgentId) {
      continue;
    }
    const auto& front_index =
        neighbor_lane_vehicles->GetIndexByKey(cutin_front_obs_id);
    if (front_index == kInvalidIndex) {
      continue;
    }
    const auto& cutin_front_obs_info =
        neighbor_lane_vehicles->GetElementByIndex(front_index);
    if (cutin_front_obs_info.first == kInvalidAgentId) {
      continue;
    }

    obs_infos.push_back(cutin_front_obs_info);
  }
  neighbor_lane_vehicles->Clear();
  for (auto obs_info : obs_infos) {
    neighbor_lane_vehicles->Insert(obs_info.first, obs_info.second);
  }
}

void ObstacleGameDecider::FilterObstacle(
    const std::unordered_set<int>& cutin_obstacle,
    const std::unordered_set<int>& mergein_obstacle,
    const std::unordered_set<int>& cross_obstacle) {
  FilterEgoLane();
  FilterNeighborLane(cutin_obstacle, &left_lane_vehicles_);
  FilterNeighborLane(cutin_obstacle, &right_lane_vehicles_);
  surrounding_sim_agents_.clear();
  for (const auto& obstacle :
       reference_line_info_->path_decision()->obstacles().Items()) {
    const bool is_environment_vehicle =
        ego_lane_vehicles_.Has(obstacle->PerceptionId()) ||
        left_lane_vehicles_.Has(obstacle->PerceptionId()) ||
        right_lane_vehicles_.Has(obstacle->PerceptionId()) ||
        merge_lane_vehicles_.Has(obstacle->PerceptionId());
    const bool is_processed =
        surrounding_sim_agents_.find(obstacle->PerceptionId()) !=
        surrounding_sim_agents_.end();

    if (is_environment_vehicle && !is_processed) {
      PrepareSurroundingAgent(*obstacle, cutin_obstacle, mergein_obstacle,
                              cross_obstacle);
    }
  }
}

bool ObstacleGameDecider::EgoAgentForwardSim(
    const ForwardSimAgent& ego_agent, const ForwardSimAgentMap& agent_map,
    const IndexedMap& ego_lane_vehicle, const double sim_time_step,
    CartesianState* const state_out) {
  if (state_out == nullptr) {
    return false;
  }
  ForwardSimAgent leading_agent;
  int leading_vehicle_id =
      ego_lane_vehicle
          .GetPreviousAndNextKeys(
              kEgoAgentId, ego_agent.vehicle.frenet_state().s_state.at(0))
          .second;
  if (leading_vehicle_id != kInvalidAgentId &&
      agent_map.find(leading_vehicle_id) != agent_map.end()) {
    leading_agent = agent_map.at(leading_vehicle_id);
    if (CheckCollisionUsingState(ego_agent, leading_agent)) {
      ADEBUG << "ego collision with front vehicle!";
      return false;
    }
  }

  game_common::State target_state;
  if (!GetTargetState(ego_agent, 0.0, &target_state)) {
    return false;
  }
  game_simulation::Simulation::PropagateEgoAgent(
      ego_agent, target_state, sim_time_step, ego_agent.sim_param, state_out);
  return true;
}

double ObstacleGameDecider::SafetyCost(
    const std::unordered_set<int>& cutin_obstacle,
    const std::unordered_set<int>& mergein_obstacle,
    const std::vector<Vehicle>& ego_trajectory,
    const std::unordered_map<int, std::vector<Vehicle>>&
        surround_trajectories) {
  // 安全性cost
  double min_distance = std::numeric_limits<double>::max();
  for (const auto& surround_trajectory_info : surround_trajectories) {
    const auto& surround_trajectory = surround_trajectory_info.second;
    const auto& surround_vehicle_id = surround_trajectory_info.first;
    size_t surround_trajectory_size = surround_trajectory.size();
    if (surround_trajectory_size != ego_trajectory.size()) {
      continue;
    }
    if (cutin_obstacle.find(surround_vehicle_id) == cutin_obstacle.end() &&
        mergein_obstacle.find(surround_vehicle_id) == mergein_obstacle.end()) {
      continue;
    }
    for (int i = 0; i < surround_trajectory_size; ++i) {
      if (i < surround_trajectory_size / 4) {
        continue;
      }

      const double distance = CalculateVehicleDistanceUseBox(
          surround_trajectory[i], ego_trajectory[i]);
      if (distance < min_distance) {
        min_distance = distance;
      }
    }
  }
  ADEBUG << "min distance: " << min_distance;
  const auto& shape_param =
      config_.obstacle_game_decider_config().cost_weight_shape_param();
  const double safety_cost =
      shape_param.safety_cost_k() *
      (common::math::Sigmoid(shape_param.safety_cost_a() *
                                 std::fabs(min_distance) +
                             shape_param.safety_cost_b()));
  return safety_cost;
}

double ObstacleGameDecider::EfficiencyCost(
    const std::vector<Vehicle>& ego_trajectory) {
  const auto& shape_param =
      config_.obstacle_game_decider_config().cost_weight_shape_param();
  const double target_speed = reference_line_info_->GetCruiseSpeed();
  double speed_error =
      std::fmax(0.0, target_speed - ego_trajectory.back().state().velocity);
  const double efficiency_cost =
      shape_param.efficiency_cost_k() *
      (common::math::Sigmoid(shape_param.efficiency_cost_a() *
                                 std::fabs(speed_error) +
                             shape_param.efficiency_cost_b()) -
       common::math::Sigmoid(shape_param.efficiency_cost_b()));
  return efficiency_cost;
}

double ObstacleGameDecider::AccComfortCost(
    const std::vector<Vehicle>& ego_trajectory) {
  const auto& shape_param =
      config_.obstacle_game_decider_config().cost_weight_shape_param();
  double max_acc = 0.0;
  for (int i = 1; i < ego_trajectory.size(); ++i) {
    const auto& vehicle = ego_trajectory[i];
    max_acc = fmax(max_acc, std::fabs(vehicle.state().acceleration));
  }
  const double acc_comfort_cost =
      shape_param.acc_comfort_cost_k() *
      (common::math::Sigmoid(shape_param.acc_comfort_cost_a() *
                                 std::fabs(max_acc) +
                             shape_param.acc_comfort_cost_b()) -
       common::math::Sigmoid(shape_param.acc_comfort_cost_b()));
  return acc_comfort_cost;
}

double ObstacleGameDecider::CutinIntentionCost(
    const ForwardSimAgentMap& surrounding_agents,
    const std::unordered_map<int, std::vector<Vehicle>>&
        surround_trajectories) {
  const auto& shape_param =
      config_.obstacle_game_decider_config().cost_weight_shape_param();
  double regression_distance = 0.0;
  for (const auto& agent_info : surrounding_agents) {
    const auto& agent_id = agent_info.first;
    const auto& agent = agent_info.second;
    const bool is_intention_cost_obstacle =
        agent.lat_mode == game_common::LatSimMode::kCutIn &&
        agent.lat_behavior == LateralBehavior::kLaneKeeping &&
        surround_trajectories.find(agent_id) != surround_trajectories.end();
    if (!is_intention_cost_obstacle) {
      continue;
    }

    const auto& traj = surround_trajectories.at(agent_id);
    if (traj.size() < 2) {
      continue;
    }
    const auto& first_point = traj.front();
    const auto& end_point = traj.back();
    const bool is_left = first_point.frenet_state().l_state.at(0) > 0.0;

    if (is_left) {
      regression_distance +=
          std::fmax(0.0, end_point.frenet_state().l_state.at(0) -
                             first_point.frenet_state().l_state.at(0));
    } else {
      regression_distance +=
          std::fmax(0.0, first_point.frenet_state().l_state.at(0) -
                             end_point.frenet_state().l_state.at(0));
    }
  }
  const double cutin_intention_cost =
      shape_param.cutin_intention_cost_k() *
      (common::math::Sigmoid(shape_param.cutin_intention_cost_a() *
                                 std::fabs(regression_distance) +
                             shape_param.cutin_intention_cost_b()) -
       common::math::Sigmoid(shape_param.cutin_intention_cost_b()));
  return cutin_intention_cost;
}

double ObstacleGameDecider::MergeinIntentionCost(
    const ForwardSimAgentMap& surrounding_agents,
    const std::unordered_map<int, std::vector<Vehicle>>&
        surround_trajectories) {
  const auto& shape_param =
      config_.obstacle_game_decider_config().cost_weight_shape_param();
  double max_deceleration = 0.0;
  for (const auto& agent_info : surrounding_agents) {
    const auto& agent_id = agent_info.first;
    const auto& agent = agent_info.second;
    const bool is_intention_cost_obstacle =
        agent.lat_mode == game_common::LatSimMode::kMergeIn &&
        surround_trajectories.find(agent_id) != surround_trajectories.end();
    if (!is_intention_cost_obstacle) {
      continue;
    }

    const auto& traj = surround_trajectories.at(agent_id);
    if (traj.size() < 2) {
      continue;
    }
    double sum_acc = 0;
    for (const auto& traj_point : traj) {
      sum_acc += traj_point.state().acceleration;
    }
    double acc = sum_acc / static_cast<double>(traj.size());
    if (acc < max_deceleration) {
      max_deceleration = acc;
    }
    ADEBUG << "agent_id=" << agent_id << ", min_acc=" << max_deceleration;
  }
  const double mergein_intention_cost =
      shape_param.mergein_intention_cost_k() *
      (common::math::Sigmoid(shape_param.mergein_intention_cost_a() *
                                 std::fabs(max_deceleration) +
                             shape_param.mergein_intention_cost_b()) -
       common::math::Sigmoid(shape_param.mergein_intention_cost_b()));
  return mergein_intention_cost;
}

double ObstacleGameDecider::ConsistencyCost(const ForwardSimAgent& ego_agent) {
  const double current_optimal_sim_acc = ego_agent.vehicle.state().acceleration;
  const double acc_diff =
      std::abs(current_optimal_sim_acc - last_optimal_sim_acc_);
  const auto& shape_param =
      config_.obstacle_game_decider_config().cost_weight_shape_param();
  const double consistency_cost =
      shape_param.mergein_consistency_cost_k() *
      (common::math::Sigmoid(shape_param.mergein_consistency_cost_a() *
                                 std::fabs(acc_diff) +
                             shape_param.mergein_consistency_cost_b()) -
       common::math::Sigmoid(shape_param.mergein_consistency_cost_b()));
  return consistency_cost;
}

double ObstacleGameDecider::CostFunction(
    const ForwardSimAgentMap& surrounding_agents,
    const std::unordered_set<int>& cutin_obstacle,
    const std::unordered_set<int>& mergein_obstacle,
    const std::vector<Vehicle>& ego_trajectory,
    const std::unordered_map<int, std::vector<Vehicle>>& surround_trajectories,
    const ForwardSimAgent& ego_agent) {
  // cost 计算
  // 1.安全性cost
  const double safety_cost = SafetyCost(cutin_obstacle, mergein_obstacle,
                                        ego_trajectory, surround_trajectories);
  // 2.效率性cost
  const double efficiency_cost = EfficiencyCost(ego_trajectory);
  // 3.舒适性cost
  const double acc_comfort_cost = AccComfortCost(ego_trajectory);
  // 4.意图损失cost
  const double cutin_intention_cost =
      CutinIntentionCost(surrounding_agents, surround_trajectories);
  const double mergein_intention_cost =
      MergeinIntentionCost(surrounding_agents, surround_trajectories);
  // 5. consistency cost, which cutin not consider
  const double mergein_consistency_cost = ConsistencyCost(ego_agent);

  const double cost = safety_cost + efficiency_cost + acc_comfort_cost +
                      cutin_intention_cost + mergein_intention_cost +
                      mergein_consistency_cost;
  const double sim_acc = ego_agent.vehicle.state().acceleration;
  ADEBUG << "#####ego sim_acc: " << sim_acc << ", total cost: " << cost
         << ", safety cost: " << safety_cost
         << ", efficiency cost: " << efficiency_cost
         << ", acc comfort cost: " << acc_comfort_cost
         << ", cutin intention cost: " << cutin_intention_cost
         << ", mergein intention cost: " << mergein_intention_cost
         << ", mergein consistency cost: " << mergein_consistency_cost;

  return cost;
}

void ObstacleGameDecider::PassOptimalSimResultToMotionPlanner(
    const std::unordered_set<int>& cutin_obstacle,
    const std::unordered_set<int>& mergein_obstacle,
    const std::vector<Vehicle>& ego_trajectory,
    const ForwardSimAgent& opt_ego_agent,
    const std::unordered_map<int, std::vector<Vehicle>>& trajectories) {
  const double acc = opt_ego_agent.vehicle.state().acceleration;
  last_optimal_sim_acc_ = acc;
  ADEBUG << "opt acc [ " << acc << " ]";

  ObjectDecisionType decision;
  InteractiveObsBehavior::Behavior adc_behavior =
      InteractiveObsBehavior::ConstantSpeed;
  InteractiveObsBehavior::Behavior obs_behavior =
      InteractiveObsBehavior::ConstantSpeed;

  std::unordered_map<int, GameObstacleIntention> game_obs_intention;
  std::unordered_map<int,
                     std::pair<ObjectDecisionType,
                               std::pair<InteractiveObsBehavior::Behavior,
                                         InteractiveObsBehavior::Behavior>>>
      mergein_obs_intention;

  // 此处根据前向仿真轨迹得出以下：
  // 对于cutin博弈：给出障碍物的博弈意图
  // 对于mergein博弈：给出抢让决策
  for (const auto& traj_info : trajectories) {
    int obstacle_id = traj_info.first;
    const auto& obstacle_traj = traj_info.second;
    if (obstacle_traj.empty()) {
      continue;
    }
    if (cutin_obstacle.find(obstacle_id) != cutin_obstacle.end()) {
      auto end_position_l = obstacle_traj.back().frenet_state().l_state.at(0);
      if (end_position_l < lane_left_width_ &&
          end_position_l > -lane_right_width_) {
        game_obs_intention[obstacle_id] = GameObstacleIntention::Cutin;
        ADEBUG << "###obstcle: " << obstacle_id
               << ", GameObstacleIntention::Cutin";
      } else {
        game_obs_intention[obstacle_id] = GameObstacleIntention::AlongsideObs;
        ADEBUG << "###obstcle: " << obstacle_id
               << ", GameObstacleIntention::AlongsideObs";
      }
    } else if (mergein_obstacle.find(obstacle_id) != mergein_obstacle.end()) {
      // 考虑大车的情况
      bool is_dangerous_oversized_obstacle = false;
      if (obstacle_traj.at(0).is_overside_vehicle()) {
        constexpr double kHwt = 1.0;
        constexpr double kTimeToMergePoint = 6.0;
        constexpr double kTtmDiff = 1.0;

        for (const auto& obs :
             reference_line_info_->path_decision()->obstacles().Items()) {
          if (obs->PerceptionId() == obstacle_id) {
            // 1.1 s范围危险
            const double adc_start_s =
                reference_line_info_->AdcSlBoundary().start_s();
            const double adc_end_s =
                reference_line_info_->AdcSlBoundary().end_s();
            const double adc_speed =
                reference_line_info_->vehicle_state().linear_velocity();
            const double obs_start_s = obs->PerceptionSLBoundary().start_s();
            const double obs_end_s = obs->PerceptionSLBoundary().end_s();
            const bool is_dangerous_s_range =
                adc_end_s < obs_end_s &&
                adc_end_s + adc_speed * kHwt > obs_start_s;

            // 1.2 到达冲突点的时间危险
            bool is_dangerous_ttm = false;
            if (obs->speed() >= KRAlmostZero1ENegtive10 &&
                adc_speed >= KRAlmostZero1ENegtive10) {
              const double obs_time_to_merge_point =
                  (first_merge_point_s_ - obs_start_s) / obs->speed();
              const double ego_time_to_merge_point =
                  (first_merge_point_s_ - adc_start_s) / adc_speed;
              is_dangerous_ttm =
                  (obs_time_to_merge_point <= kTimeToMergePoint) ||
                  std::fabs(ego_time_to_merge_point - obs_time_to_merge_point) <
                      kTtmDiff;
            }

            // 大车 && 符合1.1和1.2的条件
            is_dangerous_oversized_obstacle =
                is_dangerous_s_range && is_dangerous_ttm;
            ADEBUG << "###oversized_mergein_obstacle: " << obstacle_id
                   << ", is_dangerous_s_range: " << is_dangerous_s_range
                   << ", is_dangerous_TTM: " << is_dangerous_ttm;
          }
        }
      }

      auto obs_end_s = obstacle_traj.back().frenet_state().s_state.at(0);
      auto ego_end_s = ego_trajectory.back().frenet_state().s_state.at(0);
      const bool is_back_obstacle =
          reference_line_info_->AdcSlBoundary().end_s() >
          obstacle_traj.at(0).frenet_state().s_state.at(0) +
              obstacle_traj.at(0).param().length() * 0.5;
      const bool is_opt_ignore_obstacle =
          acc > -game_common::kBigEpsilon && acc < game_common::kBigEpsilon;
      const bool is_opt_overtake_obstacle = obs_end_s < ego_end_s &&
                                            acc > -game_common::kBigEpsilon &&
                                            !is_dangerous_oversized_obstacle;
      const bool is_opt_yeild_obstacle = !is_opt_overtake_obstacle;
      ADEBUG << "###mergein_obstacle: " << obstacle_id
             << ", is_opt_overtake_obstacle: " << is_opt_overtake_obstacle
             << ", is_back_obstacle: " << is_back_obstacle
             << ", is_opt_ignore_obstacle: " << is_opt_ignore_obstacle;

      if (is_opt_overtake_obstacle ||
          (is_back_obstacle && is_opt_ignore_obstacle)) {
        ADEBUG << "###mergein_obstacle: " << obstacle_id << ", overtake";
        decision.mutable_overtake();
      } else if (is_opt_yeild_obstacle) {
        ADEBUG << "###mergein_obstacle: " << obstacle_id << ", yield";
        decision.mutable_yield();
      }
      if (acc < -game_common::kBigEpsilon) {
        adc_behavior = InteractiveObsBehavior::Decelerate;
      } else if (acc > game_common::kBigEpsilon) {
        adc_behavior = InteractiveObsBehavior::Accelerate;
      } else {
        adc_behavior = InteractiveObsBehavior::ConstantSpeed;
      }
      mergein_obs_intention[obstacle_id] = {decision,
                                            {adc_behavior, obs_behavior}};
    }
  }

  frame_->SetObsGameIntention(game_obs_intention);

  auto* adc_obs_behavior_pair_vec =
      reference_line_info_->GetMutableAdcObsBehavior();
  adc_obs_behavior_pair_vec->clear();
  // 1.cutin给障碍物打决策标签和可视化输出
  // 2.mergein传给下游
  const auto& path_decision = reference_line_info_->path_decision();
  ObjectDecisionType object_decision;
  ObstaclesDeciderDebug* mutable_obstacles_decider_debug =
      injector_->planning_context()
          ->mutable_planning_status()
          ->mutable_decider_debug()
          ->mutable_obstacles_decider_debug();
  for (const Obstacle* obstacle :
       reference_line_info_->path_decision()->obstacles().Items()) {
    if (obstacle == nullptr) {
      continue;
    }

    const int percepetion_id = obstacle->PerceptionId();
    if (cutin_obstacle.find(percepetion_id) != cutin_obstacle.end()) {
      InteractiveObsBehavior* mutable_interactive_obs_behavior_debug =
          mutable_obstacles_decider_debug->add_interactive_obs_behavior();
      mutable_interactive_obs_behavior_debug->set_interactive_obs_id(
          obstacle->Id());
      mutable_interactive_obs_behavior_debug->set_obs_sim_mode(
          InteractiveObsBehavior_ObsSimMode_CutIn);
      if (acc > game_common::kBigEpsilon) {
        if (game_obs_intention[percepetion_id] !=
            GameObstacleIntention::Cutin) {
          ADEBUG << "###obstacle id : " << obstacle->Id()
                 << " cutin, overtake!";
          object_decision.mutable_overtake();
          path_decision->AddLongitudinalDecision(
              "obstacle_game_decider/overtake", obstacle->Id(),
              object_decision);
          mutable_interactive_obs_behavior_debug->set_obs_decision(
              InteractiveObsBehavior::Overtake);
        }
      } else if (acc < -game_common::kBigEpsilon) {
        ADEBUG << "###obstacle id : " << obstacle->Id() << " cutin, yield!";
        object_decision.mutable_yield();
        path_decision->AddLongitudinalDecision("obstacle_game_decider/yield",
                                               obstacle->Id(), object_decision);
        mutable_interactive_obs_behavior_debug->set_obs_decision(
            InteractiveObsBehavior::Yield);
      } else {
        ADEBUG << "###obstacle id : " << obstacle->Id() << " cutin, ignore!";
        mutable_interactive_obs_behavior_debug->set_obs_decision(
            InteractiveObsBehavior::Ignore);
      }
      CutinPostProcess(*obstacle);
    }
    if (mergein_obs_intention.find(percepetion_id) !=
        mergein_obs_intention.end()) {
      adc_obs_behavior_pair_vec->emplace_back(std::make_pair(
          obstacle->Id(), mergein_obs_intention[percepetion_id]));
    }
  }
  // mergein可视化
  if (adc_obs_behavior_pair_vec != nullptr &&
      !adc_obs_behavior_pair_vec->empty()) {
    for (const auto& obs : *adc_obs_behavior_pair_vec) {
      InteractiveObsBehavior* mutable_interactive_obs_behavior_debug =
          mutable_obstacles_decider_debug->add_interactive_obs_behavior();
      mutable_interactive_obs_behavior_debug->set_interactive_obs_id(obs.first);
      mutable_interactive_obs_behavior_debug->set_obs_sim_mode(
          InteractiveObsBehavior_ObsSimMode_MergeIn);
      if (obs.second.first.has_yield()) {
        mutable_interactive_obs_behavior_debug->set_obs_decision(
            InteractiveObsBehavior::Yield);
        ADEBUG << "###obstacle id : " << obs.first << " mergein, yield!";
      } else if (obs.second.first.has_overtake()) {
        mutable_interactive_obs_behavior_debug->set_obs_decision(
            InteractiveObsBehavior::Overtake);
        ADEBUG << "###obstacle id : " << obs.first << " mergein, overtake!";
      }
    }
  }
}

void ObstacleGameDecider::CutinPostProcess(const Obstacle& obstacle) {
  const auto percepetion_id = static_cast<int>(obstacle.PerceptionId());
  const auto obs_start_s = obstacle.PerceptionSLBoundary().start_s();
  const auto obs_end_s = obstacle.PerceptionSLBoundary().end_s();
  const auto obs_start_l = obstacle.PerceptionSLBoundary().start_l();
  const auto obs_end_l = obstacle.PerceptionSLBoundary().end_l();
  const auto ego_end_s = reference_line_info_->AdcSlBoundary().end_s();
  constexpr double kHwt = 1.0;
  // s危险
  const auto is_dangerous_s_range =
      ego_end_s < obs_end_s &&
      ego_end_s +
              reference_line_info_->vehicle_state().linear_velocity() * kHwt >
          obs_start_s;
  // 压线
  const bool is_overlap_lane =
      (obs_start_l < lane_left_width_ && obs_start_l > -lane_right_width_) ||
      (obs_end_l < lane_left_width_ && obs_end_l > -lane_right_width_);
  // 入侵
  constexpr double kInvasionDistace = 0.15;
  const bool is_invasion_lane =
      (obs_start_l < lane_left_width_ - kInvasionDistace &&
       obs_start_l > -lane_right_width_ + kInvasionDistace) ||
      (obs_end_l < lane_left_width_ - kInvasionDistace &&
       obs_end_l > -lane_right_width_ + kInvasionDistace);
  // 危险类型
  const bool is_dangerous_type = obstacle.IsOversizedVehicle();

  auto Intentions = frame_->GetObsGameIntention();
  ObjectDecisionType object_decision;
  const auto& path_decision = reference_line_info_->path_decision();
  ObstaclesDeciderDebug* mutable_obstacles_decider_debug =
      injector_->planning_context()
          ->mutable_planning_status()
          ->mutable_decider_debug()
          ->mutable_obstacles_decider_debug();
  InteractiveObsBehavior* mutable_interactive_obs_behavior_debug =
      mutable_obstacles_decider_debug->add_interactive_obs_behavior();
  mutable_interactive_obs_behavior_debug->set_interactive_obs_id(obstacle.Id());
  mutable_interactive_obs_behavior_debug->set_obs_sim_mode(
      InteractiveObsBehavior_ObsSimMode_CutIn);

  const bool is_dangerous_oversize_obs =
      is_dangerous_type && is_overlap_lane && is_dangerous_s_range &&
      Intentions[percepetion_id] == GameObstacleIntention::AlongsideObs &&
      !obstacle.LongitudinalDecision().has_overtake();
  const bool is_dangerous_normal_obs =
      is_invasion_lane && is_dangerous_s_range &&
      Intentions[percepetion_id] == GameObstacleIntention::AlongsideObs &&
      !obstacle.LongitudinalDecision().has_overtake();
  const bool is_left_obstacle =
      obs_start_l + obs_end_l >
      reference_line_info_->AdcSlBoundary().start_l() +
          reference_line_info_->AdcSlBoundary().end_l();
  if (is_dangerous_oversize_obs || is_dangerous_normal_obs) {
    ADEBUG << "###obstacle id : " << obstacle.Id()
           << " cutin, post process yield!";
    object_decision.mutable_yield();
    path_decision->AddLongitudinalDecision("obstacle_game_decider/yield",
                                           obstacle.Id(), object_decision);
    mutable_interactive_obs_behavior_debug->set_obs_decision(
        InteractiveObsBehavior::Yield);
    // 危险接近增加横向避让
    ObjectNudge* object_nudge_ptr = object_decision.mutable_nudge();
    object_nudge_ptr->set_distance_l(0.1);
    if (!is_left_obstacle) {
      object_nudge_ptr->set_type(ObjectNudge::DYNAMIC_LEFT_NUDGE);
      path_decision->AddLateralDecision(
          "obstacles_game_decider/dynamic-left-nudge", obstacle.Id(),
          object_decision);
    } else if (is_left_obstacle) {
      object_nudge_ptr->set_type(ObjectNudge::DYNAMIC_RIGHT_NUDGE);
      path_decision->AddLateralDecision(
          "obstacles_game_decider/dynamic-right-nudge", obstacle.Id(),
          object_decision);
    }
  }
}

void ObstacleGameDecider::Run(ReferenceLineInfo* reference_line_info,
                              const std::unordered_set<int>& cutin_obstacle,
                              const std::unordered_set<int>& mergein_obstacle,
                              const std::unordered_set<int>& cross_obstacle) {
  ADEBUG << "FRAME:" << frame_->SequenceNum();
  // 1.仿真环境构建
  BuildingSimEnvironment(cutin_obstacle, mergein_obstacle, cross_obstacle);
  for (const auto& sur_agent : surrounding_sim_agents_) {
    ADEBUG << "sur_agent_id: " << sur_agent.first;
  }

  // 2.闭环仿真推演
  size_t ego_agent_nums = ego_sim_agents_.size();
  std::vector<std::vector<Vehicle>> ego_trajectories(ego_agent_nums);
  std::vector<std::unordered_map<int, std::vector<Vehicle>>> surrounding_info(
      ego_agent_nums);
  double min_cost = std::numeric_limits<double>::max();
  int min_cost_index = kInvalidIndex;
  for (int i = 0; i < ego_agent_nums; ++i) {
    std::vector<Vehicle>& ego_trajectory = ego_trajectories.at(i);
    std::unordered_map<int, std::vector<Vehicle>>& surround_trajectories =
        surrounding_info.at(i);
    ForwardSimAgent ego_agent = ego_sim_agents_.at(i);
    ForwardSimAgentMap surrounding_agents = surrounding_sim_agents_;
    if (!SimulateSequence(&ego_agent, &surrounding_agents, &ego_trajectory,
                          &surround_trajectories)) {
      continue;
    }
    if (ego_trajectory.size() < 2) {
      continue;
    }
    const double cost = CostFunction(
        surrounding_agents, cutin_obstacle, mergein_obstacle, ego_trajectory,
        surround_trajectories, ego_sim_agents_.at(i));
    ADEBUG << "cost: " << cost << ", i: " << i;
    if (cost < min_cost) {
      min_cost = cost;
      min_cost_index = i;
    }
  }

  if (min_cost_index == kInvalidIndex) {
    ADEBUG << "can't find vaild simulation!";
    return;
  }
  ADEBUG << "min cost: " << min_cost << ", min index: " << min_cost_index;

  for (int i = 0; i < ego_agent_nums; ++i) {
    for (const auto& vehicle : ego_trajectories.at(i)) {
      // 此处为Python脚本可视化代码
      ADEBUG << i << "speed:" << vehicle.state().velocity
             << "time:" << vehicle.state().time_stamp;
    }
  }
  // 3.最优结果可视
  SimulationDebug(ego_trajectories.at(min_cost_index),
                  surrounding_info.at(min_cost_index), reference_line_info);

  // 4.整合下游输出
  PassOptimalSimResultToMotionPlanner(
      cutin_obstacle, mergein_obstacle, ego_trajectories.at(min_cost_index),
      ego_sim_agents_.at(min_cost_index), surrounding_info.at(min_cost_index));
}

void ObstacleGameDecider::SimulationDebug(
    const std::vector<Vehicle>& ego_optimal_trajectory,
    const std::unordered_map<int, std::vector<Vehicle>>& trajectories,
    ReferenceLineInfo* const reference_line_info) {
  if (reference_line_info == nullptr ||
      reference_line_info->mutable_debug() == nullptr) {
    ADEBUG << "debug input nullptr, failed!";
    return;
  }
  auto* avoid_alongside_debug = reference_line_info->mutable_debug()
                                    ->mutable_avoid_alongside_decider_info();

  for (const auto& vehicle : ego_optimal_trajectory) {
    common::math::Box2d vehicle_box(
        vehicle.state().position, vehicle.state().angle,
        vehicle.param().length(), vehicle.param().width());
    auto* debug_adc_box = avoid_alongside_debug->add_adc_box();
    for (const auto& point : vehicle_box.GetAllCorners()) {
      auto* debug_adc_point = debug_adc_box->add_point();
      debug_adc_point->set_x(point.x());
      debug_adc_point->set_y(point.y());
    }
    debug_adc_box->set_id(
        std::to_string(vehicle.state().time_stamp).substr(0, 3));
  }

  for (const auto& debug_info : trajectories) {
    const auto& debug_id = std::to_string(debug_info.first);
    const auto& trajectory = debug_info.second;

    auto* decision_info = avoid_alongside_debug->mutable_decision_info()->Add();
    decision_info->set_id(debug_id);

    for (const auto& vehicle : trajectory) {
      common::math::Box2d vehicle_box(
          vehicle.state().position, vehicle.state().angle,
          vehicle.param().length(), vehicle.param().width());
      auto* debug_adc_box = avoid_alongside_debug->add_adc_box();
      for (const auto& point : vehicle_box.GetAllCorners()) {
        auto* debug_adc_point = debug_adc_box->add_point();
        debug_adc_point->set_x(point.x());
        debug_adc_point->set_y(point.y());
      }
      debug_adc_box->set_id(
          std::to_string(vehicle.state().time_stamp).substr(0, 3));
    }
  }
}

bool ObstacleGameDecider::CheckCollisionUsingState(
    const ForwardSimAgent& agent_a, const ForwardSimAgent& agent_b) {
  const Box2d box_a =
      Box2d(agent_a.vehicle.state().position, agent_a.vehicle.state().angle,
            agent_a.vehicle.param().length(), agent_a.vehicle.param().width());
  const Box2d box_b =
      Box2d(agent_b.vehicle.state().position, agent_b.vehicle.state().angle,
            agent_b.vehicle.param().length(), agent_b.vehicle.param().width());
  return box_a.HasOverlap(box_b);
}

double ObstacleGameDecider::CalculateVehicleDistanceUseBox(
    const Vehicle& vehicle_a, const Vehicle& vehicle_b) {
  const Box2d box_a =
      Box2d(vehicle_a.state().position, vehicle_a.state().angle,
            vehicle_a.param().length(), vehicle_a.param().width());
  const Box2d box_b =
      Box2d(vehicle_b.state().position, vehicle_b.state().angle,
            vehicle_b.param().length(), vehicle_b.param().width());
  return box_a.DistanceTo(box_b);
}

bool ObstacleGameDecider::ProcessEgoAgent(
    ForwardSimAgent* const ego_agent,
    ForwardSimAgentMap* const surrounding_agents,
    std::vector<Vehicle>* const ego_trajectory,
    IndexedMap* const ego_lane_vehicles) {
  if (ego_agent == nullptr || surrounding_agents == nullptr ||
      ego_trajectory == nullptr || ego_lane_vehicles == nullptr) {
    return false;
  }
  CartesianState ego_state;
  // 前向仿真
  if (!EgoAgentForwardSim(*ego_agent, *surrounding_agents, *ego_lane_vehicles,
                          sim_time_resolution_, &ego_state)) {
    ADEBUG << "ego agent forward sim faild!";
    return false;
  }
  // 状态刷新
  common::SLPoint sl_point;
  reference_line_info_->reference_line().XYToSL(ego_state.position, &sl_point);
  ego_agent->vehicle.set_state(ego_state);
  ego_agent->vehicle.set_frenet_state(
      {{sl_point.s(), 0.0, 0.0}, {sl_point.l(), 0.0, 0.0}});
  // 环境刷新
  ego_lane_vehicles->UpdateValue(kEgoAgentId, sl_point.s());
  // 仿真轨迹
  ego_trajectory->emplace_back(ego_agent->vehicle);
  return true;
}

bool ObstacleGameDecider::UpdateSurroundingAgentInfo(
    const std::unordered_map<int, CartesianState>& others_state,
    IndexedMap* const ego_lane_vehicles, IndexedMap* const right_lane_vehicles,
    IndexedMap* const left_lane_vehicles, IndexedMap* const merge_lane_vehicles,
    ForwardSimAgent* const ego_agent,
    ForwardSimAgentMap* const surrounding_agents,
    std::unordered_map<int, std::vector<Vehicle>>* const
        surround_trajectories) {
  if (ego_lane_vehicles == nullptr || right_lane_vehicles == nullptr ||
      left_lane_vehicles == nullptr || ego_agent == nullptr ||
      surrounding_agents == nullptr || surround_trajectories == nullptr) {
    return false;
  }
  for (auto& other_agent_info : *surrounding_agents) {
    auto& sur_agent = other_agent_info.second;
    const int sur_agent_id = other_agent_info.first;
    // 意图刷新
    if (sur_agent.lat_behavior != LateralBehavior::kLaneKeeping) {
      if (!SurroundingAgentIntentionUpdate(*ego_agent, *surrounding_agents,
                                           *ego_lane_vehicles, &sur_agent)) {
        ADEBUG << "intention update faild!";
        return false;
      }
    }
    // 环境刷新
    if (!SimEnvironmentContextUpdate(ego_lane_vehicles, right_lane_vehicles,
                                     left_lane_vehicles, merge_lane_vehicles,
                                     &sur_agent, ego_agent)) {
      ADEBUG << "context update faild!";
      return false;
    }
    // 状态刷新
    common::SLPoint sl_point;
    const CartesianState& other_agent_state = others_state.at(sur_agent_id);
    reference_line_info_->reference_line().XYToSL(other_agent_state.position,
                                                  &sl_point);
    sur_agent.vehicle.set_frenet_state(
        {{sl_point.s(), 0.0, 0.0}, {sl_point.l(), 0.0, 0.0}});
    sur_agent.vehicle.set_state(other_agent_state);
    if (ego_lane_vehicles->Has(sur_agent_id)) {
      ego_lane_vehicles->UpdateValue(
          sur_agent_id, sur_agent.vehicle.frenet_state().s_state.at(0));
    }
    if (left_lane_vehicles->Has(sur_agent_id)) {
      left_lane_vehicles->UpdateValue(
          sur_agent_id, sur_agent.vehicle.frenet_state().s_state.at(0));
    }
    if (right_lane_vehicles->Has(sur_agent_id)) {
      right_lane_vehicles->UpdateValue(
          sur_agent_id, sur_agent.vehicle.frenet_state().s_state.at(0));
    }
    if (merge_lane_vehicles->Has(sur_agent_id)) {
      merge_lane_vehicles->UpdateValue(
          sur_agent_id, sur_agent.vehicle.frenet_state().s_state.at(0));
    }

    // 轨迹收集
    (*surround_trajectories)[sur_agent_id].emplace_back(sur_agent.vehicle);
  }
  return true;
}

bool ObstacleGameDecider::SimulateSequence(
    ForwardSimAgent* const ego_agent,
    ForwardSimAgentMap* const surrounding_agents,
    std::vector<Vehicle>* const ego_trajectory,
    std::unordered_map<int, std::vector<Vehicle>>* const
        surround_trajectories) {
  if (ego_agent == nullptr || surrounding_agents == nullptr ||
      ego_trajectory == nullptr || surround_trajectories == nullptr) {
    return false;
  }
  IndexedMap ego_lane_vehicles = ego_lane_vehicles_;
  IndexedMap right_lane_vehicles = right_lane_vehicles_;
  IndexedMap left_lane_vehicles = left_lane_vehicles_;
  IndexedMap merge_lane_vehicles = merge_lane_vehicles_;
  ego_trajectory->clear();
  ego_trajectory->reserve(sim_step_size_);
  ADEBUG << "ego agent acc: " << ego_agent->vehicle.state().acceleration;
  for (int i = 0; i < sim_step_size_; ++i) {
    // 主车仿真及更新
    if (!ProcessEgoAgent(ego_agent, surrounding_agents, ego_trajectory,
                         &ego_lane_vehicles)) {
      return false;
    }
    if (merge_lane_vehicles.Has(kEgoAgentId)) {
      merge_lane_vehicles.UpdateValue(
          kEgoAgentId, ego_agent->vehicle.frenet_state().s_state.at(0));
    }
    std::unordered_map<int, CartesianState> others_state;
    // 障碍物仿真
    for (auto& other_agent_info : *surrounding_agents) {
      auto& sur_agent = other_agent_info.second;
      const int sur_agent_id = other_agent_info.first;
      CartesianState other_agent_state;
      if (!SurroundingAgentForwardSim(
              *ego_agent, *surrounding_agents, sim_time_resolution_, i,
              ego_lane_vehicles, right_lane_vehicles, left_lane_vehicles,
              merge_lane_vehicles, sur_agent, &other_agent_state)) {
        ADEBUG << "sur agent forward sim faild!";
        return false;
      }
      others_state[sur_agent_id] = other_agent_state;
    }
    // 障碍物更新
    if (!UpdateSurroundingAgentInfo(
            others_state, &ego_lane_vehicles, &right_lane_vehicles,
            &left_lane_vehicles, &merge_lane_vehicles, ego_agent,
            surrounding_agents, surround_trajectories)) {
      ADEBUG << "update surrounding agent info faild!";
      return false;
    }
  }
  return true;
}

bool ObstacleGameDecider::GetLeadingAgent(
    const ForwardSimAgent& cur_agent, const ForwardSimAgentMap& agents_map,
    const IndexedMap& ego_lane_vehicles, const IndexedMap& left_lane_vehicles,
    const IndexedMap& right_lane_vehicles,
    ForwardSimAgent* const leading_agent) const {
  if (leading_agent == nullptr) {
    return false;
  }
  leading_agent->vehicle.set_id(kInvalidAgentId);

  const double agent_l = cur_agent.vehicle.frenet_state().l_state.at(0);

  if (agent_l > lane_left_width_) {
    int leading_vehicle_id =
        left_lane_vehicles
            .GetPreviousAndNextKeys(
                cur_agent.vehicle.id(),
                cur_agent.vehicle.frenet_state().s_state.at(0))
            .second;
    if (leading_vehicle_id != kInvalidAgentId &&
        agents_map.find(leading_vehicle_id) != agents_map.end()) {
      *leading_agent = agents_map.at(leading_vehicle_id);
    }
  } else if (agent_l < -lane_right_width_) {
    int leading_vehicle_id =
        right_lane_vehicles
            .GetPreviousAndNextKeys(
                cur_agent.vehicle.id(),
                cur_agent.vehicle.frenet_state().s_state.at(0))
            .second;
    if (leading_vehicle_id != kInvalidAgentId &&
        agents_map.find(leading_vehicle_id) != agents_map.end()) {
      *leading_agent = agents_map.at(leading_vehicle_id);
    }
  } else {
    int leading_vehicle_id =
        ego_lane_vehicles
            .GetPreviousAndNextKeys(
                cur_agent.vehicle.id(),
                cur_agent.vehicle.frenet_state().s_state.at(0))
            .second;
    if (leading_vehicle_id != kInvalidAgentId &&
        agents_map.find(leading_vehicle_id) != agents_map.end()) {
      *leading_agent = agents_map.at(leading_vehicle_id);
    }
  }
  return true;
}

bool ObstacleGameDecider::SimEnvironmentContextUpdate(
    IndexedMap* const ego_lane_vehicle, IndexedMap* const right_lane_vehicle,
    IndexedMap* const left_lane_vehicle, IndexedMap* const merge_lane_vehicle,
    ForwardSimAgent* const agent, ForwardSimAgent* const ego_agent) const {
  if (ego_lane_vehicle == nullptr || right_lane_vehicle == nullptr ||
      left_lane_vehicle == nullptr || agent == nullptr) {
    return false;
  }
  // 判定换道是否成功，换道成功，刷新仿真环境上下文信息
  const double agent_l = agent->vehicle.frenet_state().l_state.at(0);
  const double agent_s = agent->vehicle.frenet_state().s_state.at(0);
  if (agent_l > -lane_right_width_ && agent_l < lane_left_width_) {
    if (agent->lat_behavior == LateralBehavior::kLaneChangeLeft) {
      right_lane_vehicle->Remove(agent->vehicle.id());
      ego_lane_vehicle->Insert(agent->vehicle.id(), agent_s);
      agent->lat_mode = game_common::LatSimMode::kAlongSide;
      agent->lat_behavior = LateralBehavior::kLaneKeeping;
    } else if (agent->lat_behavior == LateralBehavior::kLaneChangeRight) {
      left_lane_vehicle->Remove(agent->vehicle.id());
      ego_lane_vehicle->Insert(agent->vehicle.id(), agent_s);
      agent->lat_mode = game_common::LatSimMode::kAlongSide;
      agent->lat_behavior = LateralBehavior::kLaneKeeping;
    }
  }
  // merge博弈车辆，到达汇入点，刷新仿真环境上下文信息
  if (agent->lat_mode == game_common::LatSimMode::kMergeIn) {
    // merge_lane_vehicle->Remove(agent->vehicle.id());
    const double agent_start_l = agent_l - 0.5 * agent->vehicle.param().width();
    const double agent_end_l = agent_l + 0.5 * agent->vehicle.param().width();
    const double ego_agent_s = ego_agent->vehicle.frenet_state().s_state.at(0);
    const bool overlap_ego_lane =
        (agent_start_l > -lane_right_width_ &&
         agent_start_l < lane_left_width_) ||
        (agent_end_l > -lane_right_width_ && agent_end_l < lane_left_width_);
    const bool ego_arrive_merge_point =
        agent_s + 0.5 * agent->vehicle.param().length() > first_merge_point_s_;
    const bool sur_arrive_merge_point =
        ego_agent_s + 0.5 * ego_agent->vehicle.param().length() >
        first_merge_point_s_;
    ADEBUG << "overlap_ego_lane: " << overlap_ego_lane
           << ", ego_arrive_merge_point: " << ego_arrive_merge_point
           << ", sur_arrive_merge_point: " << sur_arrive_merge_point;
    if (overlap_ego_lane || ego_arrive_merge_point || sur_arrive_merge_point) {
      ego_lane_vehicle->Insert(agent->vehicle.id(), agent_s);
      merge_lane_vehicle->Insert(ego_agent->vehicle.id(), ego_agent_s);
    }
  }
  return true;
}

bool ObstacleGameDecider::GetEgoLeadingAgent(
    const ForwardSimAgent& ego_agent, const ForwardSimAgentMap& agents_map,
    const IndexedMap& ego_lane_vehicles, ForwardSimAgent* const leading_agent) {
  if (leading_agent == nullptr) {
    return false;
  }
  leading_agent->vehicle.set_id(kInvalidAgentId);
  // leading定义为拥有绝对路权的车辆，若merge in场景，leading agent亦可能为临车道的agent(后完善)
  const int leading_vehicle_id =
      ego_lane_vehicles
          .GetPreviousAndNextKeys(
              kEgoAgentId, ego_agent.vehicle.frenet_state().s_state.at(0))
          .second;
  if (leading_vehicle_id != kInvalidAgentId &&
      agents_map.find(leading_vehicle_id) != agents_map.end()) {
    *leading_agent = agents_map.at(leading_vehicle_id);
  }
  return true;
}

bool ObstacleGameDecider::SurroundingAgentIntentionUpdate(
    const ForwardSimAgent& ego_agent, const ForwardSimAgentMap& agents_map,
    const IndexedMap& ego_lane_vehicle,
    ForwardSimAgent* const surrounding_agent) {
  if (surrounding_agent == nullptr) {
    return false;
  }
  // 社会车目标变道（主车车道）的gap前车和gap后车
  RssChecker::RssConfig config;  // rss 默认配置
  const ForwardSimAgent& gap_rear_agent = ego_agent;
  ForwardSimAgent gap_front_agent;
  if (!GetEgoLeadingAgent(ego_agent, agents_map, ego_lane_vehicle,
                          &gap_front_agent)) {
    return false;
  }

  if (surrounding_agent->sim_param.init_cutin_prob > kCutinCooperationProb) {
    return true;
  }

  // 存在gap前车
  if (gap_front_agent.vehicle.id() != kInvalidAgentId) {
    // RSS安全校验
    double safe_distance = 0.0;
    RssChecker::CalculateSafeLongitudinalDistance(
        surrounding_agent->vehicle.state().velocity,
        gap_front_agent.vehicle.state().velocity,
        RssChecker::LongitudinalDirection::Front, config, &safe_distance);
    ADEBUG << "###agent id: " << surrounding_agent->vehicle.id()
           << ", find gap front agent! safe distance: " << safe_distance;
    // RSS危险、取消变道（模拟社会车的意图刷新过程）
    if (gap_front_agent.vehicle.frenet_state().s_state.at(0) -
            surrounding_agent->vehicle.frenet_state().s_state.at(0) <
        safe_distance) {
      surrounding_agent->lat_behavior = LateralBehavior::kLaneKeeping;
    }
  }

  if (gap_rear_agent.vehicle.id() != kInvalidAgentId) {
    // 存在gap后车，这里一定存在，gap后车为主车
    double safe_distance = 0.0;
    RssChecker::CalculateSafeLongitudinalDistance(
        surrounding_agent->vehicle.state().velocity,
        gap_rear_agent.vehicle.state().velocity,
        RssChecker::LongitudinalDirection::Rear, config, &safe_distance);
    ADEBUG << "###agent id: " << surrounding_agent->vehicle.id()
           << ", find gap rear agent! safe distance: " << safe_distance;
    // RSS危险、取消变道（模拟社会车的意图刷新过程）
    if (surrounding_agent->vehicle.frenet_state().s_state.at(0) -
            gap_rear_agent.vehicle.frenet_state().s_state.at(0) <
        safe_distance) {
      surrounding_agent->lat_behavior = LateralBehavior::kLaneKeeping;
    }
  }
  return true;
}

bool ObstacleGameDecider::GetTargetState(const ForwardSimAgent& agent,
                                         const double target_l,
                                         CartesianState* state_out) {
  if (state_out == nullptr) {
    return false;
  }

  const double approx_lookahead_distance =
      std::fmin(std::fmax(agent.sim_param.steer_control_min_lookahead_dist,
                          agent.vehicle.state().velocity *
                              agent.sim_param.look_forward_time),
                agent.sim_param.steer_control_max_lookahead_dist);

  common::SLPoint sl_point;
  sl_point.set_s(approx_lookahead_distance +
                 agent.vehicle.frenet_state().s_state.at(0));
  sl_point.set_l(target_l);
  reference_line_info_->reference_line().SLToXY(sl_point, &state_out->position);
  return true;
}

bool ObstacleGameDecider::SurroundingAgentForwardSim(
    const ForwardSimAgent& ego_agent, const ForwardSimAgentMap& agents_map,
    double sim_time_step, int step_num, const IndexedMap& ego_lane_vehicles,
    const IndexedMap& right_lane_vehicles, const IndexedMap& left_lane_vehicles,
    const IndexedMap& merge_lane_vehicles,
    const ForwardSimAgent& surrounding_agent, CartesianState* const state_out) {
  if (state_out == nullptr) {
    return false;
  }

  // cut in场景
  if (surrounding_agent.lat_mode == game_common::LatSimMode::kCutIn ||
      surrounding_agent.lat_mode == game_common::LatSimMode::kAlongSide) {
    ForwardSimAgent leading_agent;
    if (!GetLeadingAgent(surrounding_agent, agents_map, ego_lane_vehicles,
                         left_lane_vehicles, right_lane_vehicles,
                         &leading_agent)) {
      ADEBUG << "get leading agent failed!";
      return false;
    }
    if (surrounding_agent.lat_behavior == LateralBehavior::kLaneKeeping) {
      if (leading_agent.vehicle.id() != kInvalidAgentId &&
          CheckCollisionUsingState(surrounding_agent, leading_agent)) {
        ADEBUG << "collision with leading vehicle! leading id: "
               << leading_agent.vehicle.id()
               << ", sur agent id: " << surrounding_agent.vehicle.id()
               << ", collision time: "
               << surrounding_agent.vehicle.state().time_stamp
               << ", sur agent s:"
               << surrounding_agent.vehicle.frenet_state().s_state.at(0)
               << ", leading agent s: "
               << leading_agent.vehicle.frenet_state().s_state.at(0);
        return false;
      }
      game_common::State target_state;
      if (merge_lane_vehicles.Has(surrounding_agent.vehicle.id())) {
        common::SLPoint sl_point;
        GetMergeinTargetSLPoint(surrounding_agent.vehicle.id(),
                                sim_time_step * step_num, &sl_point);
        reference_line_info_->reference_line().SLToXY(sl_point,
                                                      &target_state.position);
      } else {
        const double surrounding_agent_l =
            surrounding_agent.vehicle.frenet_state().l_state.at(0);
        double target_l = 0.0;
        if (surrounding_agent_l > lane_left_width_) {
          target_l = lane_left_width_ + 0.5 * left_neighbor_lane_width_;
        } else if (surrounding_agent_l < -lane_right_width_) {
          target_l = -lane_right_width_ - 0.5 * right_neighbor_lane_width_;
        }
        if (!GetTargetState(surrounding_agent, target_l, &target_state)) {
          return false;
        }
      }
      game_simulation::Simulation::PropagateOnceAdvancedLK(
          surrounding_agent, leading_agent, target_state, sim_time_step,
          surrounding_agent.sim_param, state_out);
    } else {
      ForwardSimAgent gap_front_agent;
      GetEgoLeadingAgent(ego_agent, agents_map, ego_lane_vehicles,
                         &gap_front_agent);
      game_common::State target_state;
      if (!GetTargetState(surrounding_agent, 0.0, &target_state)) {
        return false;
      }
      game_simulation::Simulation::PropagateOnceAdvancedLC(
          surrounding_agent, leading_agent, gap_front_agent, ego_agent,
          target_state, sim_time_step, surrounding_agent.sim_param, state_out);
    }
  } else if (surrounding_agent.lat_mode == game_common::LatSimMode::kMergeIn) {
    // merge in
    // get leading_veh of merge_agent
    ForwardSimAgent leading_agent;
    int leading_vehicle_id =
        merge_lane_vehicles
            .GetPreviousAndNextKeys(
                surrounding_agent.vehicle.id(),
                surrounding_agent.vehicle.frenet_state().s_state.at(0))
            .second;
    if (leading_vehicle_id != kInvalidAgentId &&
        agents_map.find(leading_vehicle_id) != agents_map.end()) {
      leading_agent = agents_map.at(leading_vehicle_id);
    }
    if (leading_vehicle_id == kEgoAgentId) {
      leading_agent = ego_agent;
    }
    ADEBUG << "sur agent id: " << surrounding_agent.vehicle.id()
           << ", leading id: " << leading_agent.vehicle.id()
           << ", time : " << surrounding_agent.vehicle.state().time_stamp
           << ", sur agent s:"
           << surrounding_agent.vehicle.frenet_state().s_state.at(0)
           << ", leading agent s: "
           << leading_agent.vehicle.frenet_state().s_state.at(0);
    // judge whether ego is the leading of agent
    bool is_ego_in_back_of_merge_leading = true;
    if (leading_agent.vehicle.id() != kInvalidAgentId) {
      is_ego_in_back_of_merge_leading =
          ego_agent.vehicle.frenet_state().s_state.at(0) <
          leading_agent.vehicle.frenet_state().s_state.at(0);
    }
    bool is_ego_in_between =
        is_ego_in_back_of_merge_leading &&
        (ego_agent.vehicle.frenet_state().s_state.at(0) >
         surrounding_agent.vehicle.frenet_state().s_state.at(0));

    if (is_ego_in_between) {
      double box_distance = CalculateVehicleDistanceUseBox(
          ego_agent.vehicle, surrounding_agent.vehicle);
      constexpr double kMinMergeSafeBoxDistance = 1.0;
      bool is_box_close = box_distance < kMinMergeSafeBoxDistance;

      double lateral_distance =
          std::abs(ego_agent.vehicle.frenet_state().l_state.at(0) -
                   surrounding_agent.vehicle.frenet_state().l_state.at(0));
      constexpr double kMinMergeSafeLateralDistance = 2.0;  // vehicle_width
      bool is_lateral_close = lateral_distance < kMinMergeSafeLateralDistance;

      if (is_box_close && is_lateral_close) {
        leading_agent = ego_agent;
      }
    }

    common::SLPoint sl_point;
    GetMergeinTargetSLPoint(surrounding_agent.vehicle.id(),
                            sim_time_step * step_num, &sl_point);
    game_common::State target_state;
    reference_line_info_->reference_line().SLToXY(sl_point,
                                                  &target_state.position);

    ADEBUG << "PropagateOnceAdvancedLM: "
           << "merge_agent=" << surrounding_agent.vehicle.id()
           << ", leading vehicle=" << leading_agent.vehicle.id();
    ForwardSimAgent gap_front_agent;
    ForwardSimAgent gap_back_agent;
    game_simulation::Simulation::PropagateOnceAdvancedLM(
        surrounding_agent, leading_agent, target_state, gap_front_agent,
        gap_back_agent, sim_time_step, surrounding_agent.sim_param, state_out);
  }
  return true;
}

void ObstacleGameDecider::GetMergeinTargetSLPoint(
    int32_t sim_obs_id, double sim_time, common::SLPoint* const sl_point) {
  sl_point->set_s(std::numeric_limits<double>::infinity());
  sl_point->set_l(std::numeric_limits<double>::infinity());
  const PathDecision* path_decision = reference_line_info_->path_decision();
  const double kPreviewTime = 2.0;
  for (const auto* obs : path_decision->obstacles().Items()) {
    if (obs->PerceptionId() == sim_obs_id) {
      const auto& trajectory_envelope = obs->GetTrajectoryEnvelope();
      if (!trajectory_envelope.empty() &&
          DefinitelyGreaterEqual(trajectory_envelope.back().center_p.s(),
                                 trajectory_envelope.front().center_p.s())) {
        const auto traj_size = trajectory_envelope.size();
        for (int i = 0; i < traj_size; ++i) {
          if (trajectory_envelope[i].time > sim_time + kPreviewTime) {
            if (std::fabs(trajectory_envelope[i].center_p.l()) <
                std::fabs(sl_point->l())) {
              sl_point->set_s(trajectory_envelope[i].center_p.s());
              sl_point->set_l(trajectory_envelope[i].center_p.l());
            }
            break;
          }
        }
        if (sl_point->l() == std::numeric_limits<double>::infinity()) {
          sl_point->set_s(trajectory_envelope.back().center_p.s());
          sl_point->set_l(trajectory_envelope.back().center_p.l());
        }
      }
    }
  }
  ADEBUG << "target_s=" << sl_point->s();
  ADEBUG << "target_l=" << sl_point->l();
}

}  // namespace TL::planning
