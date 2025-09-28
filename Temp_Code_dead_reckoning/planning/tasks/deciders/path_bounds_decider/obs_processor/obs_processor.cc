/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  planning obstacle processor base class
 * Author: ROC
 */

#include "planning/tasks/deciders/path_bounds_decider/obs_processor/obs_processor.h"
#include <algorithm>
#include <memory>
#include <utility>
#include <vector>
#include "absl/strings/match.h"
#include "common/util/macros.h"
#include "planning/common/reference_line_info.h"

namespace TL {
namespace planning {

ObsProcessor::ObsProcessor(const TaskConfig& config) : config_(config) {}

ObsProcessor::ObsProcessor(const std::shared_ptr<DependencyInjector>& injector,
                           const TaskConfig& config)
    : injector_(injector),
      config_(config),
      process_bound_(new ProcessBound(injector, config)) {
  CHECK_NOTNULL(GetInjector());
}

bool ObsProcessor::Process(
    ReferenceLineInfo* const reference_line_info, Frame* const frame,
    std::vector<std::tuple<double, double, double>>* const path_boundaries,
    std::string* const blocking_obstacle_id,
    TowingPointsInfo* const towing_points,
    const bool is_enable_towing_process) {
  UNUSED(is_enable_towing_process);
  if (frame == nullptr || reference_line_info == nullptr ||
      path_boundaries == nullptr || blocking_obstacle_id == nullptr ||
      towing_points == nullptr) {
    AERROR << "Obs Processor init faild!";
    return false;
  }
  // bound process init.
  process_bound_->InitPathBounds(frame, reference_line_info);
  return true;
}

void ObsProcessor::SetTowingPoints(
    const std::pair<double, double>& towing_prepare_distance,
    const std::pair<double, double>& cur_obs_edges,
    const std::string& obstacle_id,
    ReferenceLineInfo* const reference_line_info,
    PathBound* const path_boundaries, TowingPointsInfo* const towing_points,
    const double expect_towing_l, const bool is_obstacle_left_nudge) {
  if (path_boundaries == nullptr || towing_points == nullptr) {
    AERROR << "SetTowingPoints nullptr check is failed!";
    return;
  }
  Obstacle* const curr_obs =
      process_bound_->GetMutableReferenceLineInfo()->path_decision()->Find(
          obstacle_id);
  curr_obs->SetMaxExpectTowingL(expect_towing_l);
  double short_distance_threshold = curr_obs->IsShortDistanceThresholdDone()
                                        ? curr_obs->GetShortDistanceThreshold()
                                        : ShortDistanceThreshold(curr_obs);
  ADEBUG << "short_distance_threshold = " << short_distance_threshold
         << ", id = " << obstacle_id;
  int process_front_iter =
      std::max(static_cast<int>(
                   (cur_obs_edges.first - std::get<0>(path_boundaries->at(0)) -
                    towing_prepare_distance.second) /
                   reference_line_info->PathBoundsDeciderResolution()),
               0);
  while (process_front_iter <
         std::min(static_cast<int>(
                      (cur_obs_edges.second + towing_prepare_distance.first -
                       std::get<0>(path_boundaries->at(0))) /
                      reference_line_info->PathBoundsDeciderResolution()),
                  static_cast<int>(path_boundaries->size()))) {
    ADEBUG << "process_front_iter = " << process_front_iter;
    if (is_obstacle_left_nudge) {
      const double towing_l_front =
          std::max(common::math::Clamp(
                       expect_towing_l, 0.0,
                       (std::get<2>(path_boundaries->at(process_front_iter)) -
                        process_bound_->GetObsTowingConf()
                            .obstacle_avoid_bound_buffer()) *
                           2),
                   std::get<1>(towing_points->at(process_front_iter)));

      if ((std::get<2>(path_boundaries->at(process_front_iter)) <
               short_distance_threshold ||
           towing_l_front < short_distance_threshold) &&
          !curr_obs->IsShortDistanceNudge()) {
        curr_obs->SetIsShortDistanceNudge(true);
      }
      ADEBUG << "obs id = " << obstacle_id
             << ", IsShortDistanceNudge = " << curr_obs->IsShortDistanceNudge();

      towing_points->at(process_front_iter) = std::make_tuple(
          std::get<0>(path_boundaries->at(process_front_iter)), towing_l_front,
          std::get<2>(towing_points->at(process_front_iter)), obstacle_id);
      ADEBUG << "obs in right path_boundaries_l_max: " << FIXED
             << SETPRECISION(9)
             << std::get<2>(path_boundaries->at(process_front_iter))
             << ", towing_l_front: " << towing_l_front << ", cur_s: "
             << std::get<0>(path_boundaries->at(process_front_iter))
             << ", process_front_iter: " << process_front_iter
             << ", cur obs id: " << obstacle_id;
    } else {
      const double towing_l_front =
          std::min(common::math::Clamp(
                       -expect_towing_l, 0.0,
                       (std::get<1>(path_boundaries->at(process_front_iter)) +
                        process_bound_->GetObsTowingConf()
                            .obstacle_avoid_bound_buffer()) *
                           2),
                   std::get<2>(towing_points->at(process_front_iter)));

      if ((std::get<1>(path_boundaries->at(process_front_iter)) >
               -short_distance_threshold ||
           towing_l_front > -short_distance_threshold) &&
          !curr_obs->IsShortDistanceNudge()) {
        curr_obs->SetIsShortDistanceNudge(true);
      }
      ADEBUG << "obs id = " << obstacle_id
             << ", IsShortDistanceNudge = " << curr_obs->IsShortDistanceNudge();

      towing_points->at(process_front_iter) =
          std::tuple<double, double, double, std::string>(
              std::get<0>(path_boundaries->at(process_front_iter)),
              std::get<1>(towing_points->at(process_front_iter)),
              towing_l_front, obstacle_id);
      ADEBUG << "obs in left path_boundaries_l_min: " << FIXED
             << SETPRECISION(9)
             << std::get<1>(path_boundaries->at(process_front_iter))
             << ", towing_l_front: " << towing_l_front << ", cur_s: "
             << std::get<0>(path_boundaries->at(process_front_iter))
             << ", process_front_iter: " << process_front_iter
             << ", cur obs id: " << obstacle_id;
    }
    ++process_front_iter;
  }
}

double ObsProcessor::ShortDistanceThreshold(Obstacle* const cur_obs) const {
  const double kTransToHighLightExpandBuffer = 0.2;
  const double kTransToShortDistanceExpandBuffer = 0.05;
  bool has_nudge_decision = false;
  // bool enable_short_distance_nudge = false;
  double short_towing_distance_threshold =
      process_bound_->GetObsTowingConf()
          .default_short_towing_distance_threshold();

  if (injector_ == nullptr || injector_->frame_history() == nullptr ||
      injector_->frame_history()->Latest() == nullptr ||
      injector_->frame_history()->Latest()->DriveReferenceLineInfo() ==
          nullptr) {
    return short_towing_distance_threshold;
  }

  std::vector<Obstacle*> same_obs;
  auto& obstacle_dict = GetProcessBound()
                            ->GetMutableReferenceLineInfo()
                            ->path_decision()
                            ->GetObstacles()
                            .MutableDict();

  for (const auto& obstacle : obstacle_dict) {
    if (obstacle.second.Perception().id() == cur_obs->Perception().id()) {
      Obstacle* ptr = &obstacle_dict.at(obstacle.first);
      same_obs.emplace_back(ptr);
    }
  }

  for (const auto& obstacle : same_obs) {
    const auto* last_obstacle = injector_->frame_history()
                                    ->Latest()
                                    ->DriveReferenceLineInfo()
                                    ->path_decision()
                                    .Find(obstacle->Id());
    // only find one of obstacle occurred last frame and calculate its threshold
    if (last_obstacle != nullptr) {
      for (const auto& decider_tag : last_obstacle->decider_tags()) {
        if (absl::StrContains(decider_tag, "left-nudge") ||
            absl::StrContains(decider_tag, "right-nudge")) {
          has_nudge_decision = true;
          break;
        }
      }

      // hard out both short distance and normal nudge
      short_towing_distance_threshold =
          has_nudge_decision
              ? last_obstacle->IsShortDistanceNudge()
                    ? (process_bound_->GetObsTowingConf()
                           .default_short_towing_distance_threshold() +
                       kTransToHighLightExpandBuffer)
                    : (process_bound_->GetObsTowingConf()
                           .default_short_towing_distance_threshold() -
                       kTransToShortDistanceExpandBuffer)
              : process_bound_->GetObsTowingConf()
                    .default_short_towing_distance_threshold();
      break;
    }
  }
  for (auto* each_obstacle : same_obs) {
    each_obstacle->SetShortDistanceThreshold(short_towing_distance_threshold);
    each_obstacle->SetIsShortDistanceThresholdDone(true);
  }
  return short_towing_distance_threshold;
}

void ObsProcessor::UseTtcCalculateTowingPrepareDistance(
    ReferenceLineInfo* const reference_line_info,
    std::pair<double, double>* const towing_prepare_distance,
    const bool is_static_obstacle) {
  double forward_prepare_time = 0.0;
  double backward_prepare_time = 0.0;
  if (is_static_obstacle) {
    forward_prepare_time = process_bound_->GetObsTowingConf()
                               .static_obstacle_towing_forward_prepare_time();
    backward_prepare_time = process_bound_->GetObsTowingConf()
                                .static_obstacle_towing_backward_prepare_time();
  } else {
    forward_prepare_time = process_bound_->GetObsTowingConf()
                               .big_car_towing_forward_prepare_time();
    backward_prepare_time = process_bound_->GetObsTowingConf()
                                .big_car_towing_backward_prepare_time();
  }
  towing_prepare_distance->first = std::max(
      hdmap::PncMap::LookForwardDistance(
          forward_prepare_time,
          std::fabs(reference_line_info->vehicle_state().linear_velocity()),
          reference_line_info->GetCruiseSpeed()),
      process_bound_->GetObsTowingConf()
          .static_obstacle_towing_prepare_distance_min_limit());
  towing_prepare_distance->second = std::max(
      hdmap::PncMap::LookForwardDistance(
          backward_prepare_time,
          std::fabs(reference_line_info->vehicle_state().linear_velocity()),
          reference_line_info->GetCruiseSpeed()),
      process_bound_->GetObsTowingConf()
          .static_obstacle_towing_prepare_distance_min_limit());
  ADEBUG << "towing_prepare_distance front = " << towing_prepare_distance->first
         << ", towing_prepare_distance behind = "
         << towing_prepare_distance->second;
}

double ObsProcessor::LookUpTowingLDistance(
    const double& input_x,
    const google::protobuf::RepeatedField<double>& input_y,
    const google::protobuf::RepeatedField<double>& output_y) {
  int input_y_size = input_y.size();
  int output_y_size = output_y.size();
  if (input_y_size < 1 || output_y_size < 1) {
    ADEBUG << "LookUpTowingLDistance: vector error 0";
    return process_bound_->GetObsTowingConf().default_obstacle_towing_l();
  }
  if (input_y_size != output_y_size) {
    ADEBUG << "LookUpTowingLDistance: vector error"
           << ", input_v_size: " << input_y_size
           << ", output_v_size: " << output_y_size;
    return process_bound_->GetObsTowingConf().default_obstacle_towing_l();
  }
  if (input_x >= input_y.Get(input_y_size - 1)) {
    return output_y.Get(input_y_size - 1);
  }
  if (input_x <= input_y.Get(0)) {
    return output_y.Get(0);
  }
  for (int a = 1; a < input_y_size; ++a) {
    if (input_x >= input_y.Get(a - 1) && input_x < input_y.Get(a)) {
      return output_y.Get(a);
    }
  }
  ADEBUG << "LookUpTowingLDistance protobuf error!";
  return process_bound_->GetObsTowingConf().default_obstacle_towing_l();
}

}  // namespace planning
}  // namespace TL
