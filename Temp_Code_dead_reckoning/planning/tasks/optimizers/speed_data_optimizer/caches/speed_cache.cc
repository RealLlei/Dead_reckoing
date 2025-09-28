/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file vt_sample_cost.h
 **/
#include "planning/tasks/optimizers/speed_data_optimizer/caches/speed_cache.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "absl/strings/match.h"
#include "common/file/log.h"
#include "common/math/double_type.h"
#include "common/math/linear_interpolation.h"
#include "planning/common/obstacle.h"
#include "planning/common/planning_gflags.h"
#include "planning/common/reference_line_info.h"
#include "planning/common/speed/st_boundary.h"
#include "planning/common/util/common.h"
#include "planning/tasks/deciders/st_bounds_decider/st_obstacles_processor.h"

#include "planning/proto/speed_evaluator_config.pb.h"

namespace TL {
namespace planning {

SpeedCache::SpeedCache(const SpeedCacheConfig& config)
    : config_(config),
      vehicle_param_(common::VehicleConfigHelper::GetConfig().vehicle_param()) {
  CalculateCalibrationTable();
}

void SpeedCache::Init(const std::shared_ptr<DependencyInjector>& injector,
                      const Frame& frame,
                      ReferenceLineInfo* reference_line_info,
                      const common::TrajectoryPoint& init_point) {
  Clear();
  last_start_after_stop_ = start_after_stop_;
  start_after_stop_ = false;
  origin_init_point = init_point;
  basic_cache_.Init(config_, injector, frame, *reference_line_info, init_point);

  if (basic_cache_.GetIsChangeLanePath() ||
      time_from_last_lane_change_finish_ >
          config_.obstacle_reserve_time_after_lane_change_finish()) {
    obstacle_from_last_lane_change_finish_.clear();
    time_from_last_lane_change_finish_ = 0.0;
  } else if (!basic_cache_.GetIsChangeLanePath() &&
             !obstacle_from_last_lane_change_finish_.empty()) {
    time_from_last_lane_change_finish_ += basic_cache_.GetTimeFromLastFrame();
  } else if (basic_cache_.GetIsLastChangeLanePath() &&
             !basic_cache_.GetIsChangeLanePath()) {
    for (const auto& slt_obstacle_cache : slt_obstacle_caches_) {
      if (!slt_obstacle_cache.GetIsFront()) {
        obstacle_from_last_lane_change_finish_.insert(
            slt_obstacle_cache.GetPerceptionId());
      }
    }
    time_from_last_lane_change_finish_ = basic_cache_.GetTimeFromLastFrame();
  }
}

void SpeedCache::InitObstacle(
    const std::shared_ptr<DependencyInjector>& injector, const Frame& frame,
    const ReferenceLineInfo& reference_line_info,
    const common::TrajectoryPoint& init_point) {
  UNUSED(injector);
  follow_st_obstacle_cache_ = nullptr;
  follow_slt_obstacle_cache_ = nullptr;

  intention_obstacle_ids_.clear();
  for (const auto* obstacle :
       reference_line_info.path_decision().obstacles().Items()) {
    if (obstacle != nullptr && obstacle->GetHasIntention()) {
      intention_obstacle_ids_.insert(obstacle->PerceptionId());
    }
  }

  CacheSTObstacles(reference_line_info);
  CacheSLTObstacles(reference_line_info);

  speed_limit_cache_.Init(config_, frame, reference_line_info, init_point,
                          basic_cache_.GetLastFrame(),
                          basic_cache_.GetDistanceFromLastFrame(),
                          speed_limit_slt_obstacle_caches_);
  CacheNudgeObstacles(reference_line_info, basic_cache_.GetTimeFromLastFrame());
  CacheGapObstacles(injector, reference_line_info, init_point);
  ADEBUG << "obstacles_size:" << GetSTObstacleCaches().size();

  need_follow_curve_ = true;

  CheckIfIgnoreCollision(frame, reference_line_info);
}

void SpeedCache::Clear() {
  st_obstacle_caches_.clear();
  safe_st_obstacle_caches_.clear();
  nudge_obstacle_caches_.clear();
  safe_slt_obstacle_caches_with_decision_.clear();
  safe_slt_obstacle_caches_without_decision_.clear();
  cross_slt_obstacle_caches_.clear();
  speed_limit_slt_obstacle_caches_.clear();
}

bool SpeedCache::IsEmpty() const {
  return GetSTObstacleCaches().empty() &&
         GetGapFrontObstacleInfo() == nullptr &&
         GetGapRearObstacleInfo() == nullptr &&
         GetNudgeObstacleCaches().empty();
}

void SpeedCache::CacheSTObstacles(
    const ReferenceLineInfo& reference_line_info) {
  const auto time_unit = config_.obstacle_time_unit();
  const auto time_count =
      static_cast<int>(
          round(config_.obstacle_cache_time_length() / time_unit)) +
      1;

  for (const auto* obstacle :
       reference_line_info.path_decision().obstacles().Items()) {
    if (obstacle == nullptr || obstacle->IsStatic() ||
        obstacle->path_st_boundary().IsEmpty() ||
        obstacle->path_st_boundary().min_s() >
            FLAGS_speed_lon_decision_horizon ||
        obstacle->path_st_boundary().boundary_type() ==
            STBoundary::BoundaryType::KEEP_CLEAR ||
        obstacle->LongitudinalDecision().has_ignore() ||
        obstacle->LongitudinalDecision().has_stop()) {
      continue;
    }
    st_obstacle_caches_.emplace_back(config_, reference_line_info, obstacle,
                                     time_unit, time_count);

    if (obstacle->GetHasIntention() &&
        obstacle->GetLateralIntention() == LateralIntention::ALONGSIDE) {
      st_obstacle_caches_.back().SetEnableCollisionCheck(false);
    }
  }
}

void SpeedCache::CacheSLTObstacles(
    const ReferenceLineInfo& reference_line_info) {
  old_slt_obstacle_caches_.clear();
  old_slt_obstacle_caches_.swap(slt_obstacle_caches_);

  const auto cost_weight =
      fmax(0.0, (config_.obstacle_reserve_time_after_lane_change_finish() -
                 time_from_last_lane_change_finish_) /
                    config_.obstacle_reserve_time_after_lane_change_finish());

  const auto time_unit = config_.obstacle_time_unit();
  const auto time_count =
      static_cast<int>(
          round(config_.obstacle_cache_time_length() / time_unit)) +
      1;
  int index = 0;
  for (const auto* obstacle :
       reference_line_info.path_decision().obstacles().Items()) {
    if (obstacle == nullptr || obstacle->LongitudinalDecision().has_ignore()) {
      continue;
    }
    const auto& slt_boundary = obstacle->GetPathSLTBoundary();
    if (slt_boundary.IsEmpty()) {
      continue;
    }
    slt_obstacle_caches_.emplace_back(config_, reference_line_info, obstacle,
                                      time_unit, time_count, index++);
  }

  const auto get_slt_index = [&](const auto& id) -> int {
    for (const auto& slt_obstacle_cache : slt_obstacle_caches_) {
      if (slt_obstacle_cache.GetId() == id) {
        return slt_obstacle_cache.GetIndex();
      }
    }
    return -1;
  };

  for (auto& st_obstacle_cache : st_obstacle_caches_) {
    st_obstacle_cache.SetSLTIndex(get_slt_index(st_obstacle_cache.GetId()));
  }

  const auto get_obstacle = [&](const auto perception_id) -> const Obstacle* {
    for (const auto* obstacle :
         reference_line_info.path_decision().obstacles().Items()) {
      if (obstacle != nullptr && obstacle->PerceptionId() == perception_id) {
        return obstacle;
      }
    }
    return nullptr;
  };

  for (const auto& old_slt_obstacle_cache : old_slt_obstacle_caches_) {
    if (obstacle_from_last_lane_change_finish_.count(
            old_slt_obstacle_cache.GetPerceptionId()) <= 0) {
      continue;
    }
    const auto* obstacle =
        get_obstacle(old_slt_obstacle_cache.GetPerceptionId());
    if (obstacle == nullptr) {
      continue;
    }

    slt_obstacle_caches_.emplace_back(old_slt_obstacle_cache);
    auto& slt_obstacle_cache = slt_obstacle_caches_.back();
    slt_obstacle_cache.Forecast(obstacle, basic_cache_.GetTimeFromLastFrame());
    slt_obstacle_cache.SetCostWeight(cost_weight);
    slt_obstacle_cache.UpdateIndex(index++);
    ADEBUG << "SetCostWeight:" << slt_obstacle_cache.GetCostWeight();
  }

  for (auto& slt_obstacle_cache : slt_obstacle_caches_) {
    if (slt_obstacle_cache.GetObstacle() != nullptr &&
        slt_obstacle_cache.GetObstacle()->GetIsCrossObstacle()) {
      cross_slt_obstacle_caches_.emplace_back(&slt_obstacle_cache);
    }
  }

  for (const auto& obstacle_cache : slt_obstacle_caches_) {
    const auto* obstacle = obstacle_cache.GetObstacle();
    if (obstacle == nullptr) {
      continue;
    }
    const auto& boundary_types =
        obstacle->GetPathSLTBoundary().GetBoundaryTypes();
    if (boundary_types.count(SLTBoundary::BoundaryType::SPEED_LIMIT_CAUTION) >
        0) {
      speed_limit_slt_obstacle_caches_.emplace_back(&obstacle_cache);
    }
  }

  ConsiderDecision();
}

void SpeedCache::CacheNudgeObstacles(
    const ReferenceLineInfo& reference_line_info, double time_from_last_frame) {
  const auto time_unit = config_.nudge_obstacle_time_unit();
  const auto time_count =
      static_cast<int>(
          round(reference_line_info.st_graph_data().total_time_by_conf() /
                time_unit)) +
      1;

  std::vector<NudgeObstacleCache> old_nudge_obstacle_caches;
  old_nudge_obstacle_caches.swap(nudge_obstacle_caches_);

  std::map<int32_t, const Obstacle*> nudge_obstacles;
  for (const auto& obstacle :
       reference_line_info.path_decision().obstacles().Items()) {
    if (obstacle == nullptr) {
      continue;
    }

    const auto& slt_boundary = obstacle->GetPathSLTBoundary();
    if (slt_boundary.IsEmpty() ||
        slt_boundary.GetBoundaryTypes().count(
            SLTBoundary::BoundaryType::NUDGE_CAUTION) <= 0) {
      continue;
    }

    const auto iter = nudge_obstacles.find(obstacle->PerceptionId());
    if (iter == nudge_obstacles.end() ||
        obstacle->path_st_boundary().max_t() >
            iter->second->path_st_boundary().max_t()) {
      nudge_obstacles.emplace(obstacle->PerceptionId(), obstacle);
    }
  }

  int index = 0;
  for (const auto& obstacle_pair : nudge_obstacles) {
    const auto* obstacle = obstacle_pair.second;
    const auto& perception_sl_boundary = obstacle->PerceptionSLBoundary();
    const auto& adc_sl_boundary = reference_line_info.AdcSlBoundary();

    // find old cache
    const auto iter = std::find_if(
        old_nudge_obstacle_caches.begin(), old_nudge_obstacle_caches.end(),
        [&](const auto& cache) {
          const auto* old_obstacle = cache.GetObstacle();
          return old_obstacle != nullptr &&
                 old_obstacle->PerceptionId() == obstacle->PerceptionId();
        });
    const auto* old_cache =
        (iter == old_nudge_obstacle_caches.end()) ? nullptr : &(*iter);

    // judge whether frozen state
    const auto frozen = (perception_sl_boundary.start_s() <
                         adc_sl_boundary.end_s() +
                             config_.front_nudge_obstacle_frozen_distance()) &&
                        (perception_sl_boundary.end_s() >
                         adc_sl_boundary.start_s() -
                             config_.back_nudge_obstacle_frozen_distance());

    // create cache
    nudge_obstacle_caches_.emplace_back(
        config_, reference_line_info, obstacle, time_unit, time_count, index++,
        old_cache, time_from_last_frame, frozen);
  }

  has_non_ignore_nudge_obstacle_ =
      std::any_of(nudge_obstacle_caches_.begin(), nudge_obstacle_caches_.end(),
                  [](const auto& nudge_obstacle_cache) {
                    return nudge_obstacle_cache.GetTargetNudgeState() !=
                           SpeedCacheConfig::IGNORE;
                  });
}

void SpeedCache::CacheGapObstacles(
    const std::shared_ptr<DependencyInjector>& injector,
    const ReferenceLineInfo& reference_line_info,
    const common::TrajectoryPoint& init_point) {
  gap_front_obstacle_info_ = nullptr;
  gap_rear_obstacle_info_ = nullptr;
  lane_change_prepare_target_.t = std::numeric_limits<double>::lowest();

  if (injector == nullptr || injector->planning_context() == nullptr ||
      !injector->planning_context()->planning_status().has_change_lane() ||
      !injector->planning_context()
           ->planning_status()
           .change_lane()
           .has_change_lane_safety_swap_info()) {
    return;
  }

  const auto& change_lane_safety_swap_info =
      injector->planning_context()
          ->planning_status()
          .change_lane()
          .change_lane_safety_swap_info();
  ADEBUG << change_lane_safety_swap_info.gap_front_obstacle_id() << ", "
         << change_lane_safety_swap_info.gap_rear_obstacle_id();
  for (const auto& slt_obstacle_cache : slt_obstacle_caches_) {
    const auto* obstacle = slt_obstacle_cache.GetObstacle();
    if (obstacle == nullptr) {
      continue;
    }

    if (obstacle->Id() ==
        change_lane_safety_swap_info.gap_front_obstacle_id()) {
      gap_front_obstacle_info_ = CacheGapObstacleInfo(slt_obstacle_cache);
    } else if (obstacle->Id() ==
               change_lane_safety_swap_info.gap_rear_obstacle_id()) {
      gap_rear_obstacle_info_ = CacheGapObstacleInfo(slt_obstacle_cache);
    }
  }

  if (change_lane_safety_swap_info.has_time() &&
      change_lane_safety_swap_info.has_ref_speed() &&
      change_lane_safety_swap_info.has_lane_change_ref_speed_position()) {
    const auto& position =
        change_lane_safety_swap_info.lane_change_ref_speed_position();
    common::SLPoint sl_point;
    reference_line_info.path_data().discretized_path().XYToSL(
        position.x(), position.y(), &sl_point);
    lane_change_prepare_target_.s = sl_point.s();
    lane_change_prepare_target_.v = change_lane_safety_swap_info.ref_speed();
    lane_change_prepare_target_.t = change_lane_safety_swap_info.time();

    lane_change_prepare_target_.t = fmax(
        lane_change_prepare_target_.t, config_.lane_change_prepare_time_unit());

    const auto max_speed =
        fmin(reference_line_info.GetCruiseSpeed() *
                 (1.0 + config_.lane_change_prepare_over_speed_ratio()),
             TL::common::math::ConvertDisplaySpdToReal(
                 config_.lane_change_prepare_max_speed()));
    lane_change_prepare_target_.v =
        fmin(lane_change_prepare_target_.v, max_speed);

    std::vector<std::vector<double>> decel_vec_vec_state;
    if (TL::planning::util::GetStateAtMinJerk(
            init_point.v(), init_point.a(), 0.0,
            std::numeric_limits<double>::max(),
            config_.lane_change_prepare_comfort_decel(),
            -fabs(config_.lane_change_prepare_comfort_jerk()),
            lane_change_prepare_target_.t,
            config_.lane_change_prepare_time_unit(), 0.0,
            &decel_vec_vec_state) &&
        decel_vec_vec_state.size() >= 4 && !decel_vec_vec_state.at(2).empty()) {
      lane_change_prepare_target_.v =
          fmax(lane_change_prepare_target_.v, decel_vec_vec_state.at(2).back());
    }

    std::vector<std::vector<double>> accel_vec_vec_state;
    if (TL::planning::util::GetStateAtMaxJerk(
            init_point.v(), init_point.a(), 0.0,
            std::numeric_limits<double>::max(),
            config_.lane_change_prepare_comfort_accel(),
            fabs(config_.lane_change_prepare_comfort_jerk()),
            lane_change_prepare_target_.t,
            config_.lane_change_prepare_time_unit(), 0, &accel_vec_vec_state) &&
        accel_vec_vec_state.size() >= 4 && !accel_vec_vec_state.at(2).empty()) {
      lane_change_prepare_target_.v =
          fmin(lane_change_prepare_target_.v, accel_vec_vec_state.at(2).back());
    }

    ADEBUG << FIXED << SETPRECISION(3)
           << "[lane_change_prepare_target]s:" << lane_change_prepare_target_.s
           << ", v:" << lane_change_prepare_target_.v
           << ", t:" << lane_change_prepare_target_.t << ", x:" << position.x()
           << ", y:" << position.y();
    SetNeedFollowCurve(false);
  }
}

void SpeedCache::CheckIfIgnoreCollision(
    const Frame& frame, const ReferenceLineInfo& reference_line_info) {
  if (!reference_line_info.IsChangeLanePath()) {
    return;
  }

  routing::ChangeLaneType change_lane_type =
      reference_line_info.reference_line().GetADCWaypoint().l > 0
          ? routing::ChangeLaneType::RIGHT
          : routing::ChangeLaneType::LEFT;

  const ReferenceLineInfo* adc_reference_line_info = nullptr;
  for (const auto& ref_line_info : frame.reference_line_info()) {
    if (!ref_line_info.IsChangeLanePath()) {
      adc_reference_line_info = &ref_line_info;
      break;
    }
  }
  if (adc_reference_line_info == nullptr) {
    return;
  }

  for (auto& slt_obstacle_cache : slt_obstacle_caches_) {
    const auto* obstacle = adc_reference_line_info->path_decision().Find(
        slt_obstacle_cache.GetId());
    if (obstacle == nullptr) {
      continue;
    }

    const auto obs_sl_boundary = obstacle->PerceptionSLBoundary();
    const auto obs_center_l =
        (obs_sl_boundary.start_l() + obs_sl_boundary.end_l()) * 0.5;
    double lane_left_width = 0.0;
    double lane_right_width = 0.0;
    adc_reference_line_info->reference_line().GetLaneWidth(
        (obs_sl_boundary.start_s() + obs_sl_boundary.end_s()) * 0.5,
        &lane_left_width, &lane_right_width);

    if ((change_lane_type == routing::ChangeLaneType::LEFT &&
         obs_center_l < -lane_right_width) ||
        (change_lane_type == routing::ChangeLaneType::RIGHT ||
         obs_center_l > lane_left_width)) {
      slt_obstacle_cache.SetIsIgnoreCollision(true);
    }
  }
}

const SLTObstacleCache::ObstacleInfo* SpeedCache::CacheGapObstacleInfo(
    const SLTObstacleCache& slt_obstacle_cache) {
  const auto half_width = vehicle_param_.width() * 0.5;
  for (const auto& obstacle_info : slt_obstacle_cache.GetObstacleInfos()) {
    if (obstacle_info.l_lower < half_width &&
        obstacle_info.l_upper > -half_width) {
      return &obstacle_info;
    }
  }
  return nullptr;
}

void SpeedCache::UpdateCurrentNudgeState(const SpeedData* speed_data) {
  if (speed_data == nullptr || speed_data->empty()) {
    return;
  }

  const auto& start_speed_point = speed_data->front();
  const auto& end_speed_point = speed_data->back();
  for (auto& nudge_obstacle_cache : nudge_obstacle_caches_) {
    if (nudge_obstacle_cache.GetTargetNudgeState() ==
        SpeedCacheConfig::IGNORE) {
      nudge_obstacle_cache.SetCurrentNudgeState(SpeedCacheConfig::IGNORE);
      continue;
    }

    const auto start_delta_s =
        start_speed_point.s() -
        nudge_obstacle_cache.GetObstacleInfoAtTime(start_speed_point.t())
            .s_lower;
    const auto end_delta_s =
        end_speed_point.s() -
        nudge_obstacle_cache.GetObstacleInfoAtTime(end_speed_point.t()).s_lower;
    if (start_delta_s + 2.0 < end_delta_s) {
      nudge_obstacle_cache.SetCurrentNudgeState(SpeedCacheConfig::OVERTAKE);
    } else {
      nudge_obstacle_cache.SetCurrentNudgeState(SpeedCacheConfig::FOLLOW);
    }
  }

  for (auto& nudge_obstacle_cache : nudge_obstacle_caches_) {
    const auto* obstacle = nudge_obstacle_cache.GetObstacle();
    if (obstacle == nullptr) {
      continue;
    }
    ADEBUG << "id:" << obstacle->Id() << ", current_nudge_state:"
           << SpeedCacheConfig::NudgeState_Name(
                  nudge_obstacle_cache.GetCurrentNudgeState());
  }
}

void SpeedCache::SetFollowObstacle(const Frame& frame,
                                   const ReferenceLineInfo& reference_line_info,
                                   const common::TrajectoryPoint& init_point,
                                   const std::string& obstacle_id) {
  follow_st_obstacle_cache_ = nullptr;
  for (auto& st_obstacle_cache : st_obstacle_caches_) {
    if (st_obstacle_cache.GetObstacle() != nullptr &&
        st_obstacle_cache.GetObstacle()->Id() == obstacle_id) {
      follow_st_obstacle_cache_ = &st_obstacle_cache;
      st_obstacle_cache.EstimateObstacleInfos(frame, jerk_interpolation_2d_);
    }
  }

  follow_slt_obstacle_cache_ = nullptr;
  for (auto& slt_obstacle_cache : slt_obstacle_caches_) {
    if (slt_obstacle_cache.GetObstacle() != nullptr &&
        slt_obstacle_cache.GetObstacle()->Id() == obstacle_id) {
      follow_slt_obstacle_cache_ = &slt_obstacle_cache;
      slt_obstacle_cache.EstimateObstacleInfos(frame, reference_line_info,
                                               jerk_interpolation_2d_);
    }
  }

  if (follow_slt_obstacle_cache_ == nullptr ||
      follow_slt_obstacle_cache_->GetObstacle() == nullptr) {
    old_follow_obstacle_id_ = -1;
    return;
  }

  const auto* follow_obstacle = follow_slt_obstacle_cache_->GetObstacle();
  double follow_time_change_speed = 0.1;

  constexpr auto time_unit = 1.0;
  const auto time_count =
      static_cast<int>(
          round(config_.obstacle_cache_time_length() / time_unit)) +
      1;
  follow_times_.resize(time_count);
  if (follow_times_.empty()) {
    return;
  }

  auto& follow_time = follow_times_.front();
  const auto set_follow_time = GetFollowTime();

  // calculate init follow time
  double obstacle_speed = 0.0;
  double obstacle_s_lower = 0.0;
  double obstacle_a = 0.0;
  if (follow_slt_obstacle_cache_->GetMinT() > 0.0) {
    const auto& obstacle_info =
        follow_slt_obstacle_cache_->GetObstacleInfoAtTimeCeil(
            follow_slt_obstacle_cache_->GetMinT());
    obstacle_s_lower =
        obstacle_info.s_lower - obstacle_info.ds * obstacle_info.t;
    obstacle_speed = obstacle_info.ds;
    obstacle_a = obstacle_info.dds;
  } else {
    obstacle_s_lower =
        follow_slt_obstacle_cache_->GetObstacleInfoAtTime(0.0).s_lower;
    obstacle_speed = follow_slt_obstacle_cache_->GetObstacleInfoAtTime(0.0).ds;
    obstacle_a = follow_slt_obstacle_cache_->GetObstacleInfoAtTime(0.0).dds;
  }
  auto real_distance =
      obstacle_s_lower - (0.0 + vehicle_param_.front_edge_to_center());
  auto init_follow_time =
      (real_distance - follow_slt_obstacle_cache_->GetMinFollowDistance()) /
      fmax(init_point.v(), 1.0);
  const auto distance_desired =
      init_point.v() + follow_slt_obstacle_cache_->GetMinFollowDistance();
  const double obstacle_dec_accel_threshold = -0.3;
  if (obstacle_a < obstacle_dec_accel_threshold) {
    if (distance_desired - real_distance > 1.0) {
      const auto gap =
          (distance_desired - real_distance) / fmax(init_point.v(), 1.0);
      const std::vector<double> distance_coef_vec = {0.1, 0.15, 0.3,
                                                     0.6, 0.7,  0.8};
      const std::vector<double> gap_vec = {0.1, 0.3, 0.5, 1.0, 1.8, 2.5};
      const auto distance_coef =
          common::math::InterpolationOne(gap, gap_vec, distance_coef_vec);
      const std::vector<double> speed_vec = {3.0, 8.3, 16.7};
      const std::vector<double> speed_coef_vec = {0.8, 0.9, 1.0};
      const double speed_coef = common::math::InterpolationOne(
          init_point.v(), speed_vec, speed_coef_vec);
      const std::vector<double> speed_relative_vec = {0, 1, 2, 3, 5};
      const std::vector<double> relative_speed_coef_vec = {1.0, 0.95, 0.9, 0.85,
                                                           0.7};
      const auto relative_speed_coef = common::math::InterpolationOne(
          obstacle_speed - init_point.v(), speed_relative_vec,
          relative_speed_coef_vec);
      const std::vector<double> obstacle_dec_vec = {-3.0, -2.0, -1.0, -0.3};
      const std::vector<double> obstacle_dec_coef_vec = {1, 0.95, 0.6, 0.3};
      const auto obstacle_dec_coef = common::math::InterpolationOne(
          obstacle_a, obstacle_dec_vec, obstacle_dec_coef_vec);
      follow_time_change_speed = std::max(
          speed_coef * distance_coef * relative_speed_coef * obstacle_dec_coef,
          0.1);
      ADEBUG << "distance_coef " << distance_coef << " speed_coef "
             << relative_speed_coef << " follow_time_change_speed "
             << follow_time_change_speed << " obstacle_dec_coef "
             << obstacle_dec_coef;
    }
  }
  ADEBUG << "follow_time_change_speed " << follow_time_change_speed;
  if (follow_obstacle->PerceptionId() == old_follow_obstacle_id_ &&
      (frame.vehicle_state().driving_mode() ==
           TL::soc::Chassis::COMPLETE_AUTO_DRIVE ||
       frame.vehicle_state().driving_mode() ==
           TL::soc::Chassis::AUTO_SPEED_ONLY)) {
    if (follow_time < set_follow_time) {
      follow_time = fmin(
          fmax(init_follow_time, follow_time + follow_time_change_speed * 0.1),
          set_follow_time);
    } else if (follow_time > set_follow_time) {
      follow_time = fmax(
          fmin(init_follow_time, follow_time - follow_time_change_speed * 0.1),
          set_follow_time);
    }
  } else {
    follow_time = init_follow_time;
  }

  for (int i = 1; i < time_count; ++i) {
    if (follow_time < set_follow_time) {
      follow_times_.at(i) =
          fmin(set_follow_time,
               follow_time + follow_time_change_speed * i * time_unit);
    } else if (follow_time > set_follow_time) {
      follow_times_.at(i) =
          fmax(set_follow_time,
               follow_time - follow_time_change_speed * i * time_unit);
    } else {
      follow_times_.at(i) = follow_time;
    }
  }

  if (GetLaneChangePrepareTarget().t > 0) {
    for (int i = 0; i < time_count; ++i) {
      follow_times_.at(i) = 0.05;
    }
  }

  old_follow_obstacle_id_ =
      follow_slt_obstacle_cache_->GetObstacle()->PerceptionId();
}

void SpeedCache::UpdateSTBoundary(ReferenceLineInfo* reference_line_info) {
  if (reference_line_info == nullptr || st_obstacle_caches_.empty()) {
    return;
  }
  for (const auto& st_obstacle_cache : st_obstacle_caches_) {
    auto* obstacle =
        reference_line_info->path_decision()->Find(st_obstacle_cache.GetId());
    if (obstacle == nullptr) {
      return;
    }

    constexpr double kSTBoundaryEpsilon = 1e-3;

    std::vector<STPoint> lower_points;
    std::vector<STPoint> upper_points;
    std::vector<STPoint> speed_points;
    const auto& obstacle_infos = st_obstacle_cache.GetObstacleInfos();
    lower_points.reserve(obstacle_infos.size());
    upper_points.reserve(obstacle_infos.size());
    const auto time_unit = st_obstacle_cache.GetTimeUnit();
    for (int i = 0; i < obstacle_infos.size(); ++i) {
      const auto t = i * time_unit;
      if (common::math::double_type::DefinitelyLess(
              t, st_obstacle_cache.GetMinT())) {
        continue;
      }
      if (common::math::double_type::DefinitelyGreater(
              t, st_obstacle_cache.GetMaxT())) {
        break;
      }
      lower_points.emplace_back(obstacle_infos.at(i).s_lower, t);
      upper_points.emplace_back(
          fmax(lower_points.back().s() + kSTBoundaryEpsilon,
               obstacle_infos.at(i).s_upper),
          t);
      speed_points.emplace_back(obstacle_infos.at(i).v, t);
    }

    auto st_boundary =
        STBoundary::CreateInstanceAccurate(lower_points, upper_points);
    if (st_boundary.lower_points().empty()) {
      continue;
    }
    st_boundary.set_speed_points(std::move(speed_points));
    obstacle->set_path_st_boundary(std::move(st_boundary));
  }
}

void SpeedCache::CalculateCalibrationTable() {
  if (!config_.has_jerk_value_calibration_2d_table() ||
      config_.jerk_value_calibration_2d_table().calibration_info_size() <= 1) {
    return;
  }

  Interpolation2D::DataType xyz{};
  ADEBUG << "+++++++++jerk 2d table++++++++++++";
  for (const auto& calibration :
       config_.jerk_value_calibration_2d_table().calibration_info()) {
    xyz.emplace_back(std::make_tuple(calibration.speed(), calibration.accel(),
                                     calibration.jerk()));
    ADEBUG << " speed:" << std::get<0>(xyz.back())
           << ", accel:" << std::get<1>(xyz.back())
           << ", jerk:" << std::get<2>(xyz.back());
  }
  // distance_interpolation_.reset(new Interpolation2D);
  jerk_interpolation_2d_ = std::make_unique<Interpolation2D>();
  if (!jerk_interpolation_2d_->Init(xyz)) {
    ADEBUG << " jerk interpolation init failed";
    jerk_interpolation_2d_.reset();
  }
}

void SpeedCache::ConsiderDecision() {
  for (const auto& slt_obstacle_cache : slt_obstacle_caches_) {
    if (slt_obstacle_cache.GetObstacle()->GetHasIntention()) {
      ADEBUG << "intention_obstacle_id:" << slt_obstacle_cache.GetId()
             << ", lat_intention:"
             << static_cast<int>(
                    slt_obstacle_cache.GetObstacle()->GetLateralIntention())
             << ", lon_intention:"
             << static_cast<int>(slt_obstacle_cache.GetObstacle()
                                     ->GetLongitudinalIntention());
    }
  }

  safe_st_obstacle_caches_.clear();
  for (const auto& st_obstacle_cache : st_obstacle_caches_) {
    const auto* obstacle = st_obstacle_cache.GetObstacle();
    if (obstacle != nullptr &&
        (intention_obstacle_ids_.count(obstacle->PerceptionId()) == 0 ||
         obstacle->GetHasIntention())) {
      safe_st_obstacle_caches_.emplace_back(&st_obstacle_cache);
    }
  }

  const auto get_origin_obstacle = [&](const std::string& id) {
    const auto iter =
        std::find_if(slt_obstacle_caches_.begin(), slt_obstacle_caches_.end(),
                     [&](const auto& slt_obstacle_cache) {
                       return absl::StrContains(id, slt_obstacle_cache.GetId());
                     });
    return iter == slt_obstacle_caches_.end() ? nullptr : &(*iter);
  };

  safe_slt_obstacle_caches_without_decision_.clear();
  safe_slt_obstacle_caches_with_decision_.clear();
  for (auto& slt_obstacle_cache : slt_obstacle_caches_) {
    const auto* obstacle = slt_obstacle_cache.GetObstacle();
    if (obstacle == nullptr ||
        obstacle->GetPathSLTBoundary().GetBoundaryTypes().count(
            SLTBoundary::BoundaryType::SAFE_CAUTION) == 0) {
      continue;
    }

    if (intention_obstacle_ids_.count(obstacle->PerceptionId()) == 0 ||
        obstacle->GetLongitudinalIntention() ==
            LongitudinalIntention::UNKNOWN ||
        (obstacle->GetLateralIntention() == LateralIntention::ALONGSIDE &&
         obstacle->GetLongitudinalIntention() ==
             LongitudinalIntention::YIELD)) {
      safe_slt_obstacle_caches_without_decision_.emplace_back(
          &slt_obstacle_cache);
      continue;
    }

    if (obstacle->GetLongitudinalIntention() == LongitudinalIntention::UNSET) {
      continue;
    }

    if (obstacle->GetLateralIntention() == LateralIntention::MERGE) {
      const auto* origin_obstacle =
          get_origin_obstacle(slt_obstacle_cache.GetId());
      if (origin_obstacle != nullptr) {
        *slt_obstacle_cache.GetMutableObstacleInfos() =
            origin_obstacle->GetObstacleInfos();
      }
    }
    safe_slt_obstacle_caches_with_decision_.emplace_back(&slt_obstacle_cache);
  }

  for (auto& safe_st_obstacle_cache : safe_st_obstacle_caches_) {
    ADEBUG << "[safe_st_obstacle_caches]id:"
           << safe_st_obstacle_cache->GetObstacle()->Id();
  }

  for (auto& slt_obstacle_cache : safe_slt_obstacle_caches_with_decision_) {
    ADEBUG << "[safe_slt_obstacle_caches_with_decision]id:"
           << slt_obstacle_cache->GetObstacle()->Id();
  }
  for (auto& slt_obstacle_cache : safe_slt_obstacle_caches_without_decision_) {
    ADEBUG << "[safe_slt_obstacle_caches_without_decision]id:"
           << slt_obstacle_cache->GetObstacle()->Id();
  }
}

void SpeedCache::IgnoreDecision() {
  safe_st_obstacle_caches_.clear();
  for (const auto& st_obstacle_cache : st_obstacle_caches_) {
    const auto* obstacle = st_obstacle_cache.GetObstacle();
    if (obstacle != nullptr && !obstacle->GetHasIntention()) {
      safe_st_obstacle_caches_.emplace_back(&st_obstacle_cache);
    }
  }

  safe_slt_obstacle_caches_without_decision_.clear();
  safe_slt_obstacle_caches_with_decision_.clear();
  for (auto& slt_obstacle_cache : slt_obstacle_caches_) {
    const auto* obstacle = slt_obstacle_cache.GetObstacle();
    if (obstacle == nullptr || obstacle->GetHasIntention()) {
      continue;
    }
    const auto& boundary_types =
        obstacle->GetPathSLTBoundary().GetBoundaryTypes();

    if (boundary_types.count(SLTBoundary::BoundaryType::SAFE_CAUTION) > 0) {
      safe_slt_obstacle_caches_without_decision_.emplace_back(
          &slt_obstacle_cache);
    }
  }
}

}  // namespace planning
}  // namespace TL
