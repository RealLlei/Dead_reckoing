/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

/**
 * @file
 **/

#include "planning/tasks/deciders/st_bounds_decider/st_obstacles_processor.h"

#include <algorithm>
#include <cstddef>
#include <limits>
#include <set>
#include <unordered_set>
#include <utility>
#include <vector>

#include "absl/strings/match.h"
#include "common/configs/vehicle_config_helper.h"
#include "common/file/log.h"
#include "common/math/double_type.h"
#include "common/math/line_segment2d.h"
#include "common/math/math_utils.h"
#include "common/math/vec2d.h"
#include "common/util/util.h"
#include "map/hdmap/hdmap_common.h"
#include "map/hdmap/hdmap_util.h"
#include "planning/common/path/path_data.h"
#include "planning/common/planning_gflags.h"
#include "planning/common/util/common.h"

#include "proto/common/pnc_point.pb.h"
#include "proto/fsm/function_manager.pb.h"
#include "proto/fsm/nnp_fct.pb.h"
#include "proto/perception/perception_obstacle.pb.h"
#include "proto/planning/decision.pb.h"
#include "proto/routing/routing.pb.h"

namespace TL {
namespace planning {

using TL::common::ErrorCode;
using TL::common::PathPoint;
using TL::common::Status;
using TL::common::math::Box2d;
using TL::common::math::Vec2d;

constexpr double kPathEnvelopeLBuffer = 3.0;

namespace {
// ObsTEdge contains: (is_starting_t, t, s_min, s_max, obs_id).
using ObsTEdge = std::tuple<int, double, double, double, std::string>;
}  // namespace

bool STObstaclesProcessor::Init(const double planning_distance,
                                const double planning_time,
                                ReferenceLineInfo* const reference_line_info,
                                const Frame* frame, History* const history,
                                const STBoundsDeciderConfig* st_bound_config,
                                const Frame* last_frame) {
  if (reference_line_info == nullptr || frame == nullptr ||
      st_bound_config == nullptr) {
    return false;
  }
  planning_time_ = planning_time;
  planning_distance_ = planning_distance;
  vehicle_param_ = common::VehicleConfigHelper::GetConfig().vehicle_param();

  const auto& path_data = reference_line_info->path_data();
  if (path_data.discretized_path().empty()) {
    AERROR << "discretized_path is empty";
    return false;
  }
  adc_path_init_s_ = path_data.discretized_path().front().s();
  history_ = history;
  reference_line_info_ = reference_line_info;
  obs_t_edges_.clear();
  obs_t_edges_idx_ = 0;

  obs_id_to_st_boundary_.clear();
  obs_id_to_decision_.clear();
  candidate_clear_zones_.clear();
  obs_id_to_alternative_st_boundary_.clear();

  frame_ = frame;
  last_frame_ = last_frame;
  st_bounds_config_ = st_bound_config;
  path_dir_line_segments_.clear();
  normal_line_segments_.clear();
  const auto& discretized_path = path_data.discretized_path();
  path_dir_line_segments_.reserve(discretized_path.size());
  normal_line_segments_.reserve(discretized_path.size());
  for (std::size_t i = 0; i + 1 < discretized_path.size(); ++i) {
    const auto& path_point = discretized_path.at(i);
    const auto& direction_point = discretized_path.at(i + 1);
    Vec2d path_pt(path_point.x(), path_point.y());
    Vec2d dir_pt(direction_point.x(), direction_point.y());
    path_dir_line_segments_.emplace_back(path_pt, dir_pt);
    normal_line_segments_.emplace_back(
        path_pt, path_dir_line_segments_.back().rotate(M_PI_2));
  }

  adc_reference_line_info_ = nullptr;
  for (const auto& ref_line_info : frame_->reference_line_info()) {
    if (!ref_line_info.IsChangeLanePath()) {
      adc_reference_line_info_ = &ref_line_info;
      break;
    }
  }

  CalculatePathRoadRight();
  CalculatePathFrenetInfos();
  CalculatePathCautionEnvelopes();

  if (frame_->local_view().HasFunctionManagerIn()) {
    is_avp_mode_ = functionmanager::AVP ==
                   frame_->local_view().GetFunctionManagerIn()->ta_pilot_mode();
  }

  return true;
}

Status STObstaclesProcessor::MapObstaclesToSTBoundaries(
    PathDecision* const path_decision) {
  // Sanity checks.
  if (path_decision == nullptr) {
    const std::string msg = "path_decision is nullptr";
    AERROR << msg;
    return Status(ErrorCode::PLANNER_CRUISING_STBOUNDS_ERROR, msg);
  }
  if (planning_time_ < 0.0) {
    const std::string msg = "Negative planning time.";
    AERROR << msg;
    return Status(ErrorCode::PLANNER_CRUISING_STBOUNDS_ERROR, msg);
  }
  if (planning_distance_ < 0.0) {
    const std::string msg = "Negative planning distance.";
    AERROR << msg;
    return Status(ErrorCode::PLANNER_CRUISING_STBOUNDS_ERROR, msg);
  }
  if (reference_line_info_->path_data().discretized_path().size() <= 1) {
    const std::string msg = "Number of path points is too few.";
    AERROR << msg;
    return Status(ErrorCode::PLANNER_CRUISING_STBOUNDS_ERROR, msg);
  }
  obs_id_to_st_boundary_.clear();

  // Some preprocessing to save the adc_low_road_right segments.
  bool is_adc_low_road_right_beginning = true;
  for (const auto& path_pt_info :
       reference_line_info_->path_data().path_point_decision_guide()) {
    double path_pt_s = 0.0;
    PathData::PathPointType path_pt_type = PathData::PathPointType::IN_LANE;
    std::tie(path_pt_s, path_pt_type, std::ignore) = path_pt_info;
    if (path_pt_type == PathData::PathPointType::OUT_ON_FORWARD_LANE ||
        path_pt_type == PathData::PathPointType::OUT_ON_REVERSE_LANE) {
      if (is_adc_low_road_right_beginning) {
        adc_low_road_right_segments_.emplace_back(path_pt_s, path_pt_s);
        is_adc_low_road_right_beginning = false;
      } else {
        adc_low_road_right_segments_.back().second = path_pt_s;
      }
    } else if (path_pt_type == PathData::PathPointType::IN_LANE) {
      if (!is_adc_low_road_right_beginning) {
        is_adc_low_road_right_beginning = true;
      }
    }
  }

  // Map obstacles into ST-graph.
  // Go through every obstacle and plot them in ST-graph.
  std::unordered_set<std::string> non_ignore_obstacles;
  std::tuple<std::string, STBoundary, Obstacle*> closest_stop_obstacle;
  std::get<0>(closest_stop_obstacle) = "NULL";
  ErrorCode code_obs = ErrorCode::OK;
  std::string error_msg;
  if (!FLAGS_use_multi_thread_obs_build_stbound) {
    for (const auto* obs_item_ptr : path_decision->obstacles().Items()) {
      // Sanity checks.
      Obstacle* obs_ptr = path_decision->Find(obs_item_ptr->Id());

      ADEBUG << "obs_id:" << obs_item_ptr->Id()
             << "   s_min:" << obs_item_ptr->PerceptionSLBoundary().start_s()
             << "   s_max:" << obs_item_ptr->PerceptionSLBoundary().end_s()
             << "   l_min:" << obs_item_ptr->PerceptionSLBoundary().start_l()
             << "   l_max:" << obs_item_ptr->PerceptionSLBoundary().end_l()
             << "   is_static:" << obs_item_ptr->IsStatic()
             << " is_traj_empty : "
             << obs_item_ptr->Trajectory().trajectory_point().empty()
             << "   front_to_edge:" << vehicle_param_.front_edge_to_center()
             << "   back_to_edge:" << vehicle_param_.back_edge_to_center();
      if (obs_ptr == nullptr) {
        const std::string msg = "Null obstacle pointer.";
        AERROR << msg;
        return Status(ErrorCode::PLANNER_CRUISING_STBOUNDS_ERROR, msg);
      }

      if (CheckIfIgnoreObstacle(obs_ptr)) {
        ObjectDecisionType ignore_decision;
        ignore_decision.mutable_ignore();
        obs_ptr->AddLongitudinalDecision("st_obstacle_processor",
                                         ignore_decision);
        continue;
      }

      // Draw the obstacle's st-boundary.
      std::vector<STPoint> lower_points;
      std::vector<STPoint> upper_points;
      bool is_caution_obstacle = false;
      double obs_caution_end_t = 0.0;
      if (!ComputeObstacleSTBoundary(*obs_ptr, &lower_points, &upper_points,
                                     &is_caution_obstacle,
                                     &obs_caution_end_t)) {
        // Obstacle doesn't appear on ST-Graph.
        continue;
      }
      auto boundary =
          STBoundary::CreateInstanceAccurate(lower_points, upper_points);
      ADEBUG << "boundary_obs_id:" << obs_item_ptr->Id()
             << "   s_min:" << boundary.min_s()
             << "   s_max:" << boundary.max_s()
             << "   t_min:" << boundary.min_t()
             << "   t_max:" << boundary.max_t();
      boundary.set_id(obs_ptr->Id());
      if (is_caution_obstacle) {
        boundary.set_obstacle_road_right_ending_t(obs_caution_end_t);
      }
      // Update the trimmed obstacle into alternative st-bound storage
      // for later uses.
      while (lower_points.size() > 2 &&
             lower_points.back().t() > obs_caution_end_t) {
        lower_points.pop_back();
      }
      while (upper_points.size() > 2 &&
             upper_points.back().t() > obs_caution_end_t) {
        upper_points.pop_back();
      }
      auto alternative_boundary =
          STBoundary::CreateInstanceAccurate(lower_points, upper_points);
      alternative_boundary.set_id(obs_ptr->Id());
      obs_id_to_alternative_st_boundary_[obs_ptr->Id()] = alternative_boundary;
      ADEBUG << "Obstacle " << obs_ptr->Id()
             << " has an alternative st-boundary with "
             << lower_points.size() + upper_points.size() << " points.";

      // Store all Keep-Clear zone together.
      if (absl::StrContains(obs_item_ptr->Id(), "KC")) {
        candidate_clear_zones_.emplace_back(obs_ptr->Id(), boundary, obs_ptr);
        continue;
      }

      // Process all other obstacles than Keep-Clear zone.
      if (obs_ptr->Trajectory().trajectory_point().empty()) {
        // Obstacle is static.
        if (std::get<0>(closest_stop_obstacle) == "NULL" ||
            std::get<1>(closest_stop_obstacle).bottom_left_point().s() >
                boundary.bottom_left_point().s()) {
          // If this static obstacle is closer for ADC to stop, record it.
          closest_stop_obstacle =
              std::make_tuple(obs_ptr->Id(), boundary, obs_ptr);
        }
      } else {
        // Obstacle is dynamic.
        if (boundary.bottom_left_point().s() - adc_path_init_s_ <
                kSIgnoreThreshold &&
            boundary.bottom_left_point().t() > kTIgnoreThreshold) {
          // Ignore obstacles that are behind.
          // TODO(jiacheng): don't ignore if ADC is in dangerous segments.
          // continue;
        }
        obs_id_to_st_boundary_[obs_ptr->Id()] = boundary;
        obs_ptr->set_path_st_boundary(std::move(boundary));
        non_ignore_obstacles.insert(obs_ptr->Id());
        ADEBUG << "Adding " << obs_ptr->Id() << " into the ST-graph.";
      }
    }
  } else {
    auto ComputerSTBoundaryEachObs =
        [&](auto& obs_item_ptr) {  // Sanity checks.
          common::sub_thread_name = "_planning";
          if (obs_item_ptr == nullptr) {
            std::lock_guard<std::mutex> guard(lock_0_);
            code_obs = ErrorCode::PLANNER_CRUISING_STBOUNDS_ERROR;
            return;
          }
          Obstacle* obs_ptr = path_decision->Find(obs_item_ptr->Id());
          ADEBUG << "obs_id:" << obs_item_ptr->Id() << "   s_min:"
                 << obs_item_ptr->PerceptionSLBoundary().start_s()
                 << "   s_max:" << obs_item_ptr->PerceptionSLBoundary().end_s()
                 << "   l_min:"
                 << obs_item_ptr->PerceptionSLBoundary().start_l()
                 << "   l_max:" << obs_item_ptr->PerceptionSLBoundary().end_l()
                 << "   is_static:" << obs_item_ptr->IsStatic()
                 << "   speed:" << obs_item_ptr->speed() << " is_traj_empty : "
                 << obs_item_ptr->Trajectory().trajectory_point().empty();
          if (obs_ptr == nullptr) {
            const std::string msg = "Null obstacle pointer.";
            AERROR << msg;
            {
              std::lock_guard<std::mutex> guard(lock_0_);
              error_msg = obs_item_ptr->Id();
              code_obs = ErrorCode::PLANNER_CRUISING_STBOUNDS_ERROR;
            }
            return;
          }

          if (CheckIfIgnoreObstacle(obs_ptr)) {
            ObjectDecisionType ignore_decision;
            ignore_decision.mutable_ignore();
            obs_ptr->AddLongitudinalDecision("st_obstacle_processor",
                                             ignore_decision);
            return;
          }

          // Draw the obstacle's st-boundary.
          std::vector<STPoint> lower_points;
          std::vector<STPoint> upper_points;
          bool is_caution_obstacle = false;
          double obs_caution_end_t = 0.0;
          if (!ComputeObstacleSTBoundary(*obs_ptr, &lower_points, &upper_points,
                                         &is_caution_obstacle,
                                         &obs_caution_end_t)) {
            // Obstacle doesn't appear on ST-Graph.
            return;
          }

          auto boundary =
              STBoundary::CreateInstanceAccurate(lower_points, upper_points);
          ADEBUG << "boundary_obs_id:" << obs_item_ptr->Id()
                 << "   s_min:" << boundary.min_s()
                 << "   s_max:" << boundary.max_s()
                 << "   t_min:" << boundary.min_t()
                 << "   t_max:" << boundary.max_t();
          boundary.set_id(obs_ptr->Id());
          if (is_caution_obstacle) {
            boundary.set_obstacle_road_right_ending_t(obs_caution_end_t);
          }
          // Update the trimmed obstacle into alternative st-bound storage
          // for later uses.
          while (lower_points.size() > 2 &&
                 lower_points.back().t() > obs_caution_end_t) {
            lower_points.pop_back();
          }
          while (upper_points.size() > 2 &&
                 upper_points.back().t() > obs_caution_end_t) {
            upper_points.pop_back();
          }
          auto alternative_boundary =
              STBoundary::CreateInstanceAccurate(lower_points, upper_points);
          alternative_boundary.set_id(obs_ptr->Id());

          {
            std::lock_guard<std::mutex> guard(lock_1_);
            obs_id_to_alternative_st_boundary_[obs_ptr->Id()] =
                alternative_boundary;
          }

          ADEBUG << "Obstacle " << obs_ptr->Id()
                 << " has an alternative st-boundary with "
                 << lower_points.size() + upper_points.size() << " points.";

          // Store all Keep-Clear zone together.
          if (absl::StrContains(obs_item_ptr->Id(), "KC")) {
            {
              std::lock_guard<std::mutex> guard(lock_2_);
              candidate_clear_zones_.emplace_back(obs_ptr->Id(), boundary,
                                                  obs_ptr);
            }
            return;
          }
          // Process all other obstacles than Keep-Clear zone.
          if (obs_ptr->Trajectory().trajectory_point().empty()) {
            {
              // Obstacle is static.
              std::lock_guard<std::mutex> guard(lock_4_);
              if (std::get<0>(closest_stop_obstacle) == "NULL" ||
                  std::get<1>(closest_stop_obstacle).bottom_left_point().s() >
                      boundary.bottom_left_point().s()) {
                // If this static obstacle is closer for ADC to stop, record it.
                closest_stop_obstacle =
                    std::make_tuple(obs_ptr->Id(), boundary, obs_ptr);
              }
            }
          } else {
            // Obstacle is dynamic.
            if (boundary.bottom_left_point().s() - adc_path_init_s_ <
                    kSIgnoreThreshold &&
                boundary.bottom_left_point().t() > kTIgnoreThreshold) {
              // Ignore obstacles that are behind.
              // TODO(jiacheng): don't ignore if ADC is in dangerous segments.
              // return;
            }
            {
              std::lock_guard<std::mutex> guard(lock_3_);
              obs_id_to_st_boundary_[obs_ptr->Id()] = boundary;
              non_ignore_obstacles.insert(obs_ptr->Id());
            }
            obs_ptr->set_path_st_boundary(std::move(boundary));
            ADEBUG << "Adding " << obs_ptr->Id() << " into the ST-graph.";
          }
          return;
        };
    TL::common::thread::ThreadPool::Instance()->ForEach(
        path_decision->obstacles().Items().begin(),
        path_decision->obstacles().Items().end(), ComputerSTBoundaryEachObs);
  }

  if (code_obs == ErrorCode::PLANNER_CRUISING_STBOUNDS_ERROR) {
    return Status(ErrorCode::PLANNER_CRUISING_STBOUNDS_ERROR,
                  error_msg + " Null obstacle pointer.");
  }
  // For static obstacles, only retain the closest one (also considers
  // Keep-Clear zone here).
  // Note: We only need to check the overlapping between the closest obstacle
  //       and all the Keep-Clear zones. Because if there is another obstacle
  //       overlapping with a Keep-Clear zone, which results in an even closer
  //       stop fence, then that very Keep-Clear zone must also overlap with
  //       the closest obstacle. (Proof omitted here)
  if (std::get<0>(closest_stop_obstacle) != "NULL") {
    auto closest_stop_obs_id = std::get<0>(closest_stop_obstacle);
    const auto* closest_stop_obs_boundary = &std::get<1>(closest_stop_obstacle);
    auto* closest_stop_obs_ptr = std::get<2>(closest_stop_obstacle);
    ADEBUG << "Closest obstacle ID = " << closest_stop_obs_id;
    // Go through all Keep-Clear zones, and see if there is an even closer
    // stop fence due to them.
    for (const auto& clear_zone : candidate_clear_zones_) {
      const auto& clear_zone_boundary = std::get<1>(clear_zone);
      if (closest_stop_obs_boundary->min_s() >= clear_zone_boundary.min_s() &&
          closest_stop_obs_boundary->min_s() <= clear_zone_boundary.max_s()) {
        closest_stop_obs_id = std::get<0>(clear_zone);
        closest_stop_obs_boundary = &std::get<1>(clear_zone);
        closest_stop_obs_ptr = std::get<2>(clear_zone);
        ADEBUG << "Clear zone " << closest_stop_obs_id << " is closer.";
        break;
      }
    }
    obs_id_to_st_boundary_[closest_stop_obs_id] = *closest_stop_obs_boundary;
    closest_stop_obs_ptr->set_path_st_boundary(*closest_stop_obs_boundary);
    non_ignore_obstacles.insert(closest_stop_obs_id);
    ADEBUG << "Adding " << closest_stop_obs_ptr->Id() << " into the ST-graph.";
    ADEBUG << "min_s = " << closest_stop_obs_boundary->min_s();
  }

  // // Set IGNORE decision for those that are not in ST-graph:
  // for (const auto* obs_item_ptr : path_decision->obstacles().Items()) {
  //   Obstacle* obs_ptr = path_decision->Find(obs_item_ptr->Id());
  //   if (non_ignore_obstacles.count(obs_ptr->Id()) == 0) {
  //     ObjectDecisionType ignore_decision;
  //     ignore_decision.mutable_ignore();
  //     if (!obs_ptr->HasLongitudinalDecision()) {
  //       obs_ptr->AddLongitudinalDecision("st_obstacle_processor",
  //                                        ignore_decision);
  //     }
  //     if (!obs_ptr->HasLateralDecision()) {
  //       obs_ptr->AddLateralDecision("st_obstacle_processor", ignore_decision);
  //     }
  //   }
  // }

  // Set set path_st_boundary for keep clear zones
  for (const auto& clear_zone : candidate_clear_zones_) {
    auto* obstacle = std::get<2>(clear_zone);
    if (obstacle == nullptr) {
      continue;
    }
    obstacle->set_path_st_boundary(std::get<1>(clear_zone));
    if (obstacle->reference_line_st_boundary().boundary_type() ==
        STBoundary::BoundaryType::KEEP_CLEAR) {
      obstacle->SetStBoundaryType(STBoundary::BoundaryType::KEEP_CLEAR);
    }
  }

  // Preprocess the obstacles for sweep-line algorithms.
  // Fetch every obstacle's beginning end ending t-edges only.
  for (const auto& it : obs_id_to_st_boundary_) {
    obs_t_edges_.emplace_back(true, it.second.min_t(),
                              it.second.bottom_left_point().s(),
                              it.second.upper_left_point().s(), it.first);
    obs_t_edges_.emplace_back(false, it.second.max_t(),
                              it.second.bottom_right_point().s(),
                              it.second.upper_right_point().s(), it.first);
  }
  // Sort the edges.
  std::sort(obs_t_edges_.begin(), obs_t_edges_.end(),
            [](const ObsTEdge& lhs, const ObsTEdge& rhs) {
              if (std::get<1>(lhs) != std::get<1>(rhs)) {
                return std::get<1>(lhs) < std::get<1>(rhs);
              }
              return std::get<0>(lhs) > std::get<0>(rhs);
            });

  for (auto& obs_t_edg : obs_t_edges_) {
    ADEBUG << "is start:" << std::get<0>(obs_t_edg)
           << " end_t:" << std::get<1>(obs_t_edg)
           << "  s_min:" << std::get<2>(obs_t_edg)
           << "   s_max:" << std::get<3>(obs_t_edg)
           << "   obs_id:" << std::get<4>(obs_t_edg);
  }

  return Status::OK();
}

bool STObstaclesProcessor::GetLimitingSpeedInfo(
    double t, std::pair<double, double>* const limiting_speed_info) {
  if (obs_id_to_decision_.empty()) {
    // If no obstacle, then no speed limits.
    return false;
  }

  double s_min = 0.0;
  double s_max = planning_distance_;
  for (const auto& it : obs_id_to_decision_) {
    auto obs_id = it.first;
    auto obs_decision = it.second;
    auto obs_st_boundary = obs_id_to_st_boundary_[obs_id];
    double obs_s_min = 0.0;
    double obs_s_max = 0.0;
    obs_st_boundary.GetBoundarySRange(t, &obs_s_max, &obs_s_min);
    double obs_ds_lower = 0.0;
    double obs_ds_upper = 0.0;
    obs_st_boundary.GetBoundarySlopes(t, &obs_ds_upper, &obs_ds_lower);
    if (obs_decision.has_yield() || obs_decision.has_stop()) {
      if (obs_s_min <= s_max) {
        s_max = obs_s_min;
        limiting_speed_info->second = obs_ds_lower;
      }
    } else if (it.second.has_overtake()) {
      if (obs_s_max >= s_min) {
        s_min = obs_s_max;
        limiting_speed_info->first = obs_ds_upper;
      }
    }
  }
  return s_min <= s_max;
}

// LCOV_EXCL_START
bool STObstaclesProcessor::GetSBoundsFromDecisions(
    double t, std::vector<std::pair<double, double>>* const available_s_bounds,
    std::vector<std::vector<std::pair<std::string, ObjectDecisionType>>>* const
        available_obs_decisions) {
  // Sanity checks.
  available_s_bounds->clear();
  available_obs_decisions->clear();

  // Gather any possible change in st-boundary situations.
  ADEBUG << "There are " << obs_t_edges_.size() << " t-edges.";
  std::vector<ObsTEdge> new_t_edges;
  while (obs_t_edges_idx_ < static_cast<int>(obs_t_edges_.size()) &&
         TL::common::math::double_type::DefinitelyLessEqual(
             std::get<1>(obs_t_edges_[obs_t_edges_idx_]), t)) {
    if (std::get<0>(obs_t_edges_[obs_t_edges_idx_]) == 0 &&
        TL::common::math::double_type::SeemsEqual(
            std::get<1>(obs_t_edges_[obs_t_edges_idx_]), t)) {
      break;
    }
    ADEBUG << "Seeing a new t-edge at t = "
           << std::get<1>(obs_t_edges_[obs_t_edges_idx_]);
    new_t_edges.push_back(obs_t_edges_[obs_t_edges_idx_]);
    ++obs_t_edges_idx_;
  }

  // For st-boundaries that disappeared before t, remove them.
  for (const auto& obs_t_edge : new_t_edges) {
    if (std::get<0>(obs_t_edge) == 0) {
      ADEBUG << "Obstacle id: " << std::get<4>(obs_t_edge)
             << " is leaving st-graph.";
      if (obs_id_to_decision_.count(std::get<4>(obs_t_edge)) != 0) {
        obs_id_to_decision_.erase(std::get<4>(obs_t_edge));
      }
    }
  }

  // For overtaken obstacles, remove them if we are after
  // their high right-of-road ending time (with a margin).
  std::vector<std::string> obs_id_to_remove;
  for (const auto& obs_id_to_decision_pair : obs_id_to_decision_) {
    auto obs_id = obs_id_to_decision_pair.first;
    auto obs_decision = obs_id_to_decision_pair.second;
    auto obs_st_boundary = obs_id_to_st_boundary_[obs_id];
    if (obs_decision.has_overtake() &&
        obs_st_boundary.min_t() <= t - kOvertakenObsCautionTime &&
        obs_st_boundary.obstacle_road_right_ending_t() <=
            t - kOvertakenObsCautionTime) {
      obs_id_to_remove.push_back(obs_id_to_decision_pair.first);
    }
  }
  for (const auto& obs_id : obs_id_to_remove) {
    obs_id_to_decision_.erase(obs_id);
    // Change the displayed st-boundary to the alternative one:
    if (obs_id_to_alternative_st_boundary_.count(obs_id) > 0) {
      Obstacle* obs_ptr = reference_line_info_->path_decision()->Find(obs_id);
      obs_id_to_st_boundary_[obs_id] =
          obs_id_to_alternative_st_boundary_[obs_id];
      obs_id_to_st_boundary_[obs_id].SetBoundaryType(
          STBoundary::BoundaryType::OVERTAKE);
      obs_ptr->set_path_st_boundary(obs_id_to_alternative_st_boundary_[obs_id]);
    }
  }

  // Based on existing decisions, get the s-boundary.
  double s_min = 0.0;
  double s_max = planning_distance_;
  for (const auto& it : obs_id_to_decision_) {
    auto obs_id = it.first;
    auto obs_decision = it.second;
    auto obs_st_boundary = obs_id_to_st_boundary_[obs_id];
    double obs_s_min = 0.0;
    double obs_s_max = 0.0;
    obs_st_boundary.GetBoundarySRange(t, &obs_s_max, &obs_s_min);
    if (obs_decision.has_yield() || obs_decision.has_stop()) {
      s_max = std::fmin(s_max, obs_s_min);
    } else if (it.second.has_overtake()) {
      s_min = std::fmax(s_min, obs_s_max);
    }
    ADEBUG << "time:" << t << "   obs_id:" << obs_id << "   s_max:" << obs_s_max
           << " s_min:" << obs_s_min;
  }
  if (s_min > s_max) {
    AERROR << "s_min is bigger than s_max.";
    return false;
  }
  ADEBUG << "S-boundary based on existing decisions = (" << s_min << ", "
         << s_max << ")";

  // For newly entering st_boundaries, determine possible new-boundaries.
  // For apparent ones, make decisions directly.
  std::vector<ObsTEdge> ambiguous_t_edges;
  for (auto obs_t_edge : new_t_edges) {
    ADEBUG << "For obstacle id: " << std::get<4>(obs_t_edge)
           << ", its s-range = [" << std::get<2>(obs_t_edge) << ", "
           << std::get<3>(obs_t_edge) << "]";
    if (std::get<0>(obs_t_edge) == 1) {
      if (std::get<2>(obs_t_edge) >= s_max) {
        ADEBUG << "  Apparently, it should be yielded.";
        obs_id_to_decision_[std::get<4>(obs_t_edge)] =
            DetermineObstacleDecision(std::get<2>(obs_t_edge),
                                      std::get<3>(obs_t_edge), s_max);
        obs_id_to_st_boundary_[std::get<4>(obs_t_edge)].SetBoundaryType(
            STBoundary::BoundaryType::YIELD);
      } else if (std::get<3>(obs_t_edge) <= s_min) {
        ADEBUG << "  Apparently, it should be overtaken.";
        obs_id_to_decision_[std::get<4>(obs_t_edge)] =
            DetermineObstacleDecision(std::get<2>(obs_t_edge),
                                      std::get<3>(obs_t_edge), s_min);
        obs_id_to_st_boundary_[std::get<4>(obs_t_edge)].SetBoundaryType(
            STBoundary::BoundaryType::OVERTAKE);
      } else {
        ADEBUG << "  It should be further analyzed.";
        ambiguous_t_edges.push_back(obs_t_edge);
      }
    }
  }
  // For ambiguous ones, enumerate all decisions and corresponding bounds.
  auto s_gaps = FindSGaps(ambiguous_t_edges, s_min, s_max);
  if (s_gaps.empty()) {
    AERROR << "s_gaps is empty.";
    return false;
  }
  for (auto s_gap : s_gaps) {
    available_s_bounds->push_back(s_gap);
    std::vector<std::pair<std::string, ObjectDecisionType>> obs_decisions;
    for (auto obs_t_edge : ambiguous_t_edges) {
      std::string obs_id = std::get<4>(obs_t_edge);
      double obs_s_min = std::get<2>(obs_t_edge);
      double obs_s_max = std::get<3>(obs_t_edge);
      obs_decisions.emplace_back(
          obs_id,
          DetermineObstacleDecision(obs_s_min, obs_s_max,
                                    (s_gap.first + s_gap.second) / 2.0));
    }
    available_obs_decisions->push_back(obs_decisions);
  }

  return true;
}

// LCOV_EXCL_STOP
void STObstaclesProcessor::SetObstacleDecision(
    const std::string& obs_id, const ObjectDecisionType& obs_decision) {
  obs_id_to_decision_[obs_id] = obs_decision;
  ObjectStatus object_status;
  object_status.mutable_motion_type()->mutable_dynamic();
  if (obs_decision.has_yield() || obs_decision.has_stop()) {
    obs_id_to_st_boundary_[obs_id].SetBoundaryType(
        STBoundary::BoundaryType::YIELD);
    object_status.mutable_decision_type()->mutable_yield();
  } else if (obs_decision.has_overtake()) {
    obs_id_to_st_boundary_[obs_id].SetBoundaryType(
        STBoundary::BoundaryType::OVERTAKE);
    object_status.mutable_decision_type()->mutable_overtake();
  }
  if (history_ != nullptr) {
    history_->mutable_history_status()->SetObjectStatus(obs_id, object_status);
  }
}

void STObstaclesProcessor::SetObstacleDecision(
    const std::vector<std::pair<std::string, ObjectDecisionType>>&
        obstacle_decisions) {
  for (const auto& obs_decision : obstacle_decisions) {
    SetObstacleDecision(obs_decision.first, obs_decision.second);
  }
}

///////////////////////////////////////////////////////////////////////////////
// Private helper functions.

bool STObstaclesProcessor::ComputeObstacleSTBoundary(
    const Obstacle& obstacle, std::vector<STPoint>* const lower_points,
    std::vector<STPoint>* const upper_points, bool* const is_caution_obstacle,
    double* const obs_caution_end_t) {
  if (frame_ != nullptr && frame_->local_view().HasFunctionManagerOut() &&
      frame_->local_view().GetFunctionManagerOut() != nullptr &&
      frame_->local_view().GetFunctionManagerOut()->fsm_state() ==
          functionmanager::MachineStateType::PERCEPTION_TYPE &&
      frame_->local_view().GetFunctionManagerOut()->perception_sub_state() ==
          functionmanager::PerceptionSubState::CRUISE_TYPE &&
      (!frame_->local_view().GetCruiseTargetId().empty() &&
       std::find(frame_->local_view().GetCruiseTargetId().begin(),
                 frame_->local_view().GetCruiseTargetId().end(),
                 obstacle.PerceptionId()) !=
           frame_->local_view().GetCruiseTargetId().end())) {
    return ComputeCruiseTargetSTBoundary(obstacle, lower_points, upper_points);
  }
  return obstacle.IsStatic() ? ComputeStaticObstacleSTBoundary(
                                   obstacle, lower_points, upper_points,
                                   is_caution_obstacle, obs_caution_end_t)
                             : ComputeDynamicObstacleSTBoundary(
                                   obstacle, lower_points, upper_points,
                                   is_caution_obstacle, obs_caution_end_t);
}

bool STObstaclesProcessor::ComputeStaticObstacleSTBoundary(
    const Obstacle& obstacle, std::vector<STPoint>* const lower_points,
    std::vector<STPoint>* const upper_points, bool* const is_caution_obstacle,
    double* const obs_caution_end_t) {
  lower_points->clear();
  upper_points->clear();
  *is_caution_obstacle = false;
  const auto& obs_trajectory = obstacle.Trajectory();

  // NOLINTBEGIN
  // const auto time_diff =
  //     frame_->local_view().GetPredictionObstacles()->header().data_stamp() -
  //     (frame_->vehicle_state().timestamp() +
  //      frame_->PlanningStartPoint().relative_time());

  if (!obs_trajectory.trajectory_point().empty() || !obstacle.IsStatic()) {
    AWARN << "Non-static obstacle[" << obstacle.Id()
          << "] has NO prediction trajectory."
          << obstacle.Perception().ShortDebugString();
  }

  // Get the overlapping s between ADC path and obstacle's perception box.
  double lateral_buffer =
      is_avp_mode_ ? FLAGS_adc_l_buffer_for_static_obstacle_avp_mode
                   : FLAGS_adc_l_buffer_for_static_obstacle_st_boundary;
  const Box2d& obs_box = obstacle.PerceptionBoundingBox();
  std::pair<double, double> overlapping_s;
  if (GetOverlappingS(reference_line_info_->path_data().discretized_path(),
                      obs_box, lateral_buffer, &overlapping_s)) {
    lower_points->emplace_back(overlapping_s.first, 0.0);
    lower_points->emplace_back(overlapping_s.first, planning_time_);
    upper_points->emplace_back(overlapping_s.second, 0.0);
    upper_points->emplace_back(overlapping_s.second, planning_time_);
  }
  *is_caution_obstacle = true;
  *obs_caution_end_t = planning_time_;
  return (!lower_points->empty() && !upper_points->empty());
}

bool STObstaclesProcessor::ComputeDynamicObstacleSTBoundary(
    const Obstacle& obstacle, std::vector<STPoint>* const lower_points,
    std::vector<STPoint>* const upper_points, bool* const is_caution_obstacle,
    double* const obs_caution_end_t) {
  lower_points->clear();
  upper_points->clear();
  *is_caution_obstacle = false;
  const auto& obs_trajectory = obstacle.Trajectory();

  const auto time_diff =
      frame_->local_view().GetPredictionObstacles()->header().data_stamp() -
      (frame_->vehicle_state().timestamp() +
       frame_->PlanningStartPoint().relative_time());

  if (obs_trajectory.trajectory_point().empty()) {
    return false;
  }

  // Processing a dynamic obstacle.
  // Go through every occurrence of the obstacle at all timesteps, and
  // figure out the overlapping s-max and s-min one by one.
  bool is_obs_first_traj_pt = true;
  const auto& trajectory_points = obs_trajectory.trajectory_point();

  ADEBUG << " Processing a dynamic obstacle.";

  for (const auto& obs_traj_pt : trajectory_points) {
    // TODO(jiacheng): Currently, if the obstacle overlaps with ADC at
    // disjoint segments (happens very rarely), we merge them into one.
    // In the future, this could be considered in greater details rather
    // than being approximated.
    const Box2d& obs_box = obstacle.GetBoundingBox(obs_traj_pt);
    std::pair<double, double> overlapping_s;
    if (GetOverlappingS(reference_line_info_->path_data().discretized_path(),
                        obs_box, kADCSafetyLBufferForDynamicObstacle,
                        &overlapping_s)) {
      ADEBUG << "Obstacle instance is overlapping with ADC path.";
      lower_points->emplace_back(overlapping_s.first,
                                 obs_traj_pt.relative_time() + time_diff);
      upper_points->emplace_back(overlapping_s.second,
                                 obs_traj_pt.relative_time() + time_diff);
      if (is_obs_first_traj_pt) {
        if (IsSWithinADCLowRoadRightSegment(overlapping_s.first) ||
            IsSWithinADCLowRoadRightSegment(overlapping_s.second)) {
          *is_caution_obstacle = true;
        }
      }
      if ((*is_caution_obstacle)) {
        if (IsSWithinADCLowRoadRightSegment(overlapping_s.first) ||
            IsSWithinADCLowRoadRightSegment(overlapping_s.second)) {
          *obs_caution_end_t = obs_traj_pt.relative_time();
        }
      }
    }
    is_obs_first_traj_pt = false;
  }
  if (lower_points->size() == 1) {
    lower_points->emplace_back(lower_points->front().s(),
                               lower_points->front().t() + 0.1);
    upper_points->emplace_back(upper_points->front().s(),
                               upper_points->front().t() + 0.1);
  }

  return (!lower_points->empty() && !upper_points->empty());
}

bool STObstaclesProcessor::ComputeCruiseTargetSTBoundary(
    const Obstacle& obstacle, std::vector<STPoint>* lower_points,
    std::vector<STPoint>* upper_points) {
  if (frame_ == nullptr) {
    return false;
  }

  lower_points->clear();
  upper_points->clear();

  const auto time_diff =
      frame_->local_view().GetPredictionObstacles()->header().data_stamp() -
      (frame_->vehicle_state().timestamp() +
       frame_->PlanningStartPoint().relative_time());

  const auto& trajectory_points = obstacle.Trajectory().trajectory_point();
  const auto& trajectory_bounding_boxs = obstacle.GetTrajectoryBoundingBox();
  if (trajectory_points.empty() || trajectory_bounding_boxs.empty()) {
    return false;
  }

  auto s_lower = std::numeric_limits<double>::max();
  auto s_upper = std::numeric_limits<double>::lowest();

  // process first point
  const auto& bounding_box = trajectory_bounding_boxs.at(0);
  const auto& discretized_path =
      reference_line_info_->path_data().discretized_path();
  double s = 0.0;
  double l = 0.0;
  int index_min = -1;
  double distance = 0.0;
  double radius = bounding_box.length() * 2.0;
  if (!discretized_path.GetProjection(bounding_box.center(), &s, &l, &distance,
                                      &index_min, radius, index_min)) {
    return false;
  }

  const auto cos_theta = cos(obstacle.PerceptionBoundingBox().heading() -
                             discretized_path.Evaluate(s).theta());
  for (const auto& corner : bounding_box.GetAllCorners()) {
    discretized_path.GetProjection(corner, &s, &l, &distance, &index_min,
                                   radius, index_min);
    s_lower = fmin(s_lower, s);
    s_upper = fmax(s_upper, s);
  }
  s_lower -= vehicle_param_.back_edge_to_center();
  s_upper += vehicle_param_.front_edge_to_center();

  const auto& first_trajectory_point = trajectory_points.at(0);
  lower_points->emplace_back(
      s_lower, first_trajectory_point.relative_time() + time_diff);
  upper_points->emplace_back(
      s_upper, first_trajectory_point.relative_time() + time_diff);
  constexpr auto kMaxTimeLength = 14.0;
  constexpr auto kEndTime = 7.0;
  constexpr auto kTimeInterval = 0.1;
  const auto total_time =
      fmin(kMaxTimeLength,
           kEndTime - (first_trajectory_point.relative_time() + time_diff));
  std::vector<std::vector<double>> vec_vec_state;
  if ((common::math::double_type::DefinitelyGreaterEqual(
           first_trajectory_point.a(), 0.0) &&
       !TL::planning::util::GetStateAtMinJerk(
           first_trajectory_point.v(), first_trajectory_point.a(), 0.0,
           std::numeric_limits<double>::max(), 0.0, -1.0, total_time,
           kTimeInterval, 0.0, &vec_vec_state)) ||
      (common::math::double_type::DefinitelyLess(first_trajectory_point.a(),
                                                 0.0) &&
       !TL::planning::util::GetStateAtMaxJerk(
           first_trajectory_point.v(), first_trajectory_point.a(), 0.0,
           std::numeric_limits<double>::max(), 0.0, 1.0, total_time,
           kTimeInterval, 0.0, &vec_vec_state)) ||
      vec_vec_state.size() < 4) {
    return false;
  }

  const auto& vec_t = vec_vec_state.at(0);
  const auto& vec_s = vec_vec_state.at(1);
  if (vec_t.size() != vec_s.size() || vec_t.empty()) {
    return false;
  }

  for (int i = 1; i < vec_t.size(); ++i) {
    const auto delta_s = vec_s.at(i) * cos_theta;
    lower_points->emplace_back(
        s_lower + delta_s,
        first_trajectory_point.relative_time() + vec_t.at(i) + time_diff);
    upper_points->emplace_back(
        s_upper + delta_s,
        first_trajectory_point.relative_time() + vec_t.at(i) + time_diff);
  }

  if (lower_points->size() == 1) {
    lower_points->emplace_back(lower_points->front().s(),
                               lower_points->front().t() + 0.1);
    upper_points->emplace_back(upper_points->front().s(),
                               upper_points->front().t() + 0.1);
  }

  return (!lower_points->empty() && !upper_points->empty());
}

bool STObstaclesProcessor::GetOverlappingS(
    const std::vector<PathPoint>& adc_path_points,
    const Box2d& obstacle_instance, const double adc_l_buffer,
    std::pair<double, double>* const overlapping_s) {
  // ADEBUG << "path end_s:" << adc_path_points.back().s();
  // Locate the possible range to search in details.
  int pt_before_idx = GetSBoundingPathPointIndex(
      adc_path_points, obstacle_instance, vehicle_param_.front_edge_to_center(),
      true, 0, static_cast<int>(adc_path_points.size()) - 2);
  ADEBUG << "The index before is " << pt_before_idx;
  int pt_after_idx = GetSBoundingPathPointIndex(
      adc_path_points, obstacle_instance, vehicle_param_.back_edge_to_center(),
      false, 0, static_cast<int>(adc_path_points.size()) - 2);
  ADEBUG << "The index after is " << pt_after_idx;
  if (pt_before_idx == static_cast<int>(adc_path_points.size()) - 2) {
    return false;
  }
  if (pt_after_idx == 0) {
    return false;
  }

  if (pt_before_idx == -1) {
    pt_before_idx = 0;
  }
  if (pt_after_idx == -1) {
    pt_after_idx = static_cast<int>(adc_path_points.size()) - 2;
  }
  if (pt_before_idx >= pt_after_idx) {
    return false;
  }

  // Detailed searching.
  bool has_overlapping = false;
  for (int i = pt_before_idx; i <= pt_after_idx; ++i) {
    if (IsADCOverlappingWithObstacle(adc_path_points[i], obstacle_instance,
                                     adc_l_buffer)) {
      ADEBUG << "have collision at trajectory front idx:" << i;
      overlapping_s->first = adc_path_points[std::max(i - 1, 0)].s();
      has_overlapping = true;
      ADEBUG << "There is overlapping.";
      break;
    }
  }
  if (!has_overlapping) {
    return false;
  }
  for (int i = pt_after_idx; i >= pt_before_idx; --i) {
    ADEBUG << "At ADC path index = " << i << " :";
    if (IsADCOverlappingWithObstacle(adc_path_points[i], obstacle_instance,
                                     adc_l_buffer)) {
      ADEBUG << "have collision at trajectory back idx:" << i;
      overlapping_s->second = adc_path_points[i + 1].s();
      ADEBUG << "There is overlapping.";
      break;
    }
  }
  return true;
}

int STObstaclesProcessor::GetSBoundingPathPointIndex(
    const std::vector<PathPoint>& adc_path_points,
    const Box2d& obstacle_instance, const double s_thresh, const bool is_before,
    const int start_idx, const int end_idx) {
  if (start_idx == end_idx) {
    if (IsPathPointAwayFromObstacle(start_idx, obstacle_instance, s_thresh,
                                    is_before)) {
      return start_idx;
    }
    return -1;
  }

  if (is_before) {
    int mid_idx = (start_idx + end_idx - 1) / 2 + 1;
    if (IsPathPointAwayFromObstacle(mid_idx, obstacle_instance, s_thresh,
                                    is_before)) {
      return GetSBoundingPathPointIndex(adc_path_points, obstacle_instance,
                                        s_thresh, is_before, mid_idx, end_idx);
    }
    return GetSBoundingPathPointIndex(adc_path_points, obstacle_instance,
                                      s_thresh, is_before, start_idx,
                                      mid_idx - 1);
  }

  int mid_idx = (start_idx + end_idx) / 2;
  if (IsPathPointAwayFromObstacle(mid_idx, obstacle_instance, s_thresh,
                                  is_before)) {
    return GetSBoundingPathPointIndex(adc_path_points, obstacle_instance,
                                      s_thresh, is_before, start_idx, mid_idx);
  }
  return GetSBoundingPathPointIndex(adc_path_points, obstacle_instance,
                                    s_thresh, is_before, mid_idx + 1, end_idx);
}

bool STObstaclesProcessor::IsPathPointAwayFromObstacle(
    const int path_point_index, const Box2d& obs_box, const double s_thresh,
    const bool is_before) {
  if (path_point_index < 0 ||
      path_point_index >= path_dir_line_segments_.size() ||
      path_point_index >= normal_line_segments_.size()) {
    return true;
  }
  const auto& path_dir_lineseg = path_dir_line_segments_.at(path_point_index);
  const auto& normal_line_seg = normal_line_segments_.at(path_point_index);

  Vec2d normal_line_ft_pt;
  for (const auto& corner_pt : obs_box.GetAllCorners()) {
    normal_line_seg.GetPerpendicularFoot(corner_pt, &normal_line_ft_pt);
    Vec2d perpendicular_vec = corner_pt - normal_line_ft_pt;
    double corner_pt_s_dist =
        path_dir_lineseg.unit_direction().InnerProd(perpendicular_vec);
    if (is_before && corner_pt_s_dist < s_thresh) {
      return false;
    }
    if (!is_before && corner_pt_s_dist > -s_thresh) {
      return false;
    }
  }
  return true;
}

bool STObstaclesProcessor::IsADCOverlappingWithObstacle(
    const PathPoint& adc_path_point, const Box2d& obs_box,
    const double l_buffer) const {
  // Convert reference point from center of rear axis to center of ADC.
  Vec2d ego_center_map_frame((vehicle_param_.front_edge_to_center() -
                              vehicle_param_.back_edge_to_center()) *
                                 0.5,
                             (vehicle_param_.left_edge_to_center() -
                              vehicle_param_.right_edge_to_center()) *
                                 0.5);
  ego_center_map_frame.SelfRotate(adc_path_point.theta());
  ego_center_map_frame.set_x(ego_center_map_frame.x() + adc_path_point.x());
  ego_center_map_frame.set_y(ego_center_map_frame.y() + adc_path_point.y());

  // Compute the ADC bounding box.
  Box2d adc_box(ego_center_map_frame, adc_path_point.theta(),
                vehicle_param_.length(), vehicle_param_.width() + l_buffer * 2);

  // Check whether ADC bounding box overlaps with obstacle bounding box.
  return obs_box.HasOverlap(adc_box);
}

// LCOV_EXCL_START
std::vector<std::pair<double, double>> STObstaclesProcessor::FindSGaps(
    const std::vector<ObsTEdge>& obstacle_t_edges, double s_min, double s_max) {
  std::vector<std::pair<double, int>> obs_s_edges;
  for (auto obs_t_edge : obstacle_t_edges) {
    obs_s_edges.emplace_back(std::get<2>(obs_t_edge), 1);
    obs_s_edges.emplace_back(std::get<3>(obs_t_edge), 0);
  }
  // obs_s_edges.emplace_back(std::numeric_limits<double>::lowest(), 1);
  obs_s_edges.emplace_back(s_min, 0);
  obs_s_edges.emplace_back(s_max, 1);
  // obs_s_edges.emplace_back(std::numeric_limits<double>::max(), 0);
  std::sort(
      obs_s_edges.begin(), obs_s_edges.end(),
      [](const std::pair<double, int>& lhs, const std::pair<double, int>& rhs) {
        if (lhs.first != rhs.first) {
          return lhs.first < rhs.first;
        }
        return lhs.second > rhs.second;
      });

  std::vector<std::pair<double, double>> s_gaps;
  int num_st_obs = 1;
  double prev_open_s = 0.0;
  for (auto obs_s_edge : obs_s_edges) {
    if (obs_s_edge.second == 1) {
      num_st_obs++;
      if (num_st_obs == 1) {
        s_gaps.emplace_back(prev_open_s, obs_s_edge.first);
      }
    } else {
      num_st_obs--;
      if (num_st_obs == 0) {
        prev_open_s = obs_s_edge.first;
      }
    }
  }
  return s_gaps;
}

// LCOV_EXCL_STOP
ObjectDecisionType STObstaclesProcessor::DetermineObstacleDecision(
    const double obs_s_min, const double obs_s_max, const double s) {
  ObjectDecisionType decision;
  if (s <= obs_s_min) {
    decision.mutable_yield()->set_distance_s(0.0);
  } else if (s >= obs_s_max) {
    decision.mutable_overtake()->set_distance_s(0.0);
  }
  return decision;
}

bool STObstaclesProcessor::IsSWithinADCLowRoadRightSegment(
    const double s) const {
  return std::any_of(
      adc_low_road_right_segments_.begin(), adc_low_road_right_segments_.end(),
      [&](const auto& seg) { return s >= seg.first && s <= seg.second; });
}

// LCOV_EXCL_START
bool STObstaclesProcessor::IsStaticObstaclePolygonPointInAdcBoundary(
    const std::vector<PathPoint>& adc_path_points, const double l_buffer,
    const Obstacle& obstacle) const {
  const double adc_speed =
      reference_line_info_->vehicle_state().linear_velocity();
  double adc_max_s =
      adc_speed * planning_time_ +
      0.5 * vehicle_param_.max_acceleration() * planning_time_ * planning_time_;
  double obs_nearest_l_dis = 1000;
  /// <找到obs polygon距离自车最近的点
  TL::common::SLPoint sl_point;
  common::math::Vec2d xy_point;
  common::math::Vec2d nearest_xy_point;
  /// <在sl坐标系下比较横向距离
  if (obstacle.Perception().polygon_point().empty()) {
    return false;
  }
  for (const auto& point : obstacle.Perception().polygon_point()) {
    xy_point.set_x(point.x());
    xy_point.set_y(point.y());
    reference_line_info_->reference_line().XYToSL(xy_point, &sl_point);
    if (fabs(sl_point.l()) <= obs_nearest_l_dis) {
      obs_nearest_l_dis = sl_point.l();
      nearest_xy_point = xy_point;
    }
  }

  for (const auto& adc_path_point : adc_path_points) {
    /// <超出范围的不考虑
    if (adc_path_point.s() > adc_max_s) {
      break;
    }
    /// <首先在轨迹点的位置生成自车的box
    Vec2d ego_center_map_frame((vehicle_param_.front_edge_to_center() -
                                vehicle_param_.back_edge_to_center()) *
                                   0.5,
                               (vehicle_param_.left_edge_to_center() -
                                vehicle_param_.right_edge_to_center()) *
                                   0.5);
    ego_center_map_frame.SelfRotate(adc_path_point.theta());
    ego_center_map_frame.set_x(ego_center_map_frame.x() + adc_path_point.x());
    ego_center_map_frame.set_y(ego_center_map_frame.y() + adc_path_point.y());

    // Compute the ADC bounding box.
    Box2d adc_box(ego_center_map_frame, adc_path_point.theta(),
                  vehicle_param_.length(),
                  vehicle_param_.width() + l_buffer * 2);
    if (adc_box.IsPointIn(nearest_xy_point)) {
      return true;
    }
  }
  return false;
}

// LCOV_EXCL_STOP

const std::vector<std::tuple<int, double, double, double, std::string>>&
STObstaclesProcessor::GetObsTEdges() const {
  return obs_t_edges_;
}

bool STObstaclesProcessor::CheckIfHasOverlapWithPathCaution(
    const Obstacle& obstacle, const PathData::PathCautionDirection& direction) {
  const auto& path_caution_envelopes =
      reference_line_info_->path_data().GetPathCautionEnvelopes();
  return std::any_of(path_caution_envelopes.begin(),
                     path_caution_envelopes.end(),
                     [&](const auto& path_caution_envelope) {
                       return path_caution_envelope.direction == direction &&
                              obstacle.GetTrajMinS() <
                                  path_caution_envelope.envelope.max_ref_s &&
                              obstacle.GetTrajMaxS() >
                                  path_caution_envelope.envelope.min_ref_s &&
                              obstacle.GetTrajMinL() <
                                  path_caution_envelope.envelope.max_ref_l +
                                      kPathEnvelopeLBuffer &&
                              obstacle.GetTrajMaxL() >
                                  path_caution_envelope.envelope.min_ref_l -
                                      kPathEnvelopeLBuffer;
                     });
}

bool STObstaclesProcessor::CheckIfHasOverlapWithPath(const Obstacle& obstacle) {
  const auto& path_envelope =
      reference_line_info_->path_data().GetPathEnvelope();
  return obstacle.GetTrajMinS() < path_envelope.max_ref_s &&
         obstacle.GetTrajMaxS() > path_envelope.min_ref_s &&
         obstacle.GetTrajMinL() <
             path_envelope.max_ref_l + kPathEnvelopeLBuffer &&
         obstacle.GetTrajMaxL() >
             path_envelope.min_ref_l - kPathEnvelopeLBuffer;
}

STObstaclesProcessor::ObstacleLongitudinalPosition
STObstaclesProcessor::CalculateObstacleLongitudinalPosition(
    const Obstacle& obstacle) {
  const auto& adc_boundary = reference_line_info_->AdcSlBoundary();
  const auto& obstacle_boundary = obstacle.PerceptionSLBoundary();

  if (obstacle_boundary.start_s() > adc_boundary.end_s()) {
    return ObstacleLongitudinalPosition::FRONT;
  }
  if (obstacle_boundary.end_s() < adc_boundary.start_s()) {
    return ObstacleLongitudinalPosition::BACK;
  }
  return ObstacleLongitudinalPosition::MIDDLE;
}

STObstaclesProcessor::ObstacleLateralPosition
STObstaclesProcessor::CalculateObstacleLateralPosition(
    const Obstacle& obstacle) {
  const auto* adc_obstacle =
      adc_reference_line_info_->path_decision().Find(obstacle.Id());
  if (adc_obstacle == nullptr) {
    return ObstacleLateralPosition::CENTER;
  }
  const auto& obs_boundary = adc_obstacle->PerceptionSLBoundary();
  const auto middle_s = (obs_boundary.start_s() + obs_boundary.end_s()) / 2.0;
  const auto middle_l = (obs_boundary.start_l() + obs_boundary.end_l()) / 2.0;
  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  adc_reference_line_info_->reference_line().map_path().GetLaneWidth(
      middle_s, &lane_left_width, &lane_right_width);
  if (middle_l > lane_left_width) {
    return ObstacleLateralPosition::LEFT;
  }
  if (middle_l < -lane_right_width) {
    return ObstacleLateralPosition::RIGHT;
  }
  return ObstacleLateralPosition::CENTER;
}

bool STObstaclesProcessor::CheckIfDirectlyBehind(
    const Obstacle& obstacle, const ObstacleLateralPosition& lateral_position,
    const ObstacleLongitudinalPosition& longitudinal_position) {
  if (longitudinal_position != ObstacleLongitudinalPosition::BACK) {
    return false;
  }

  if (lateral_position != ObstacleLateralPosition::CENTER) {
    const auto* adc_obstacle =
        adc_reference_line_info_->path_decision().Find(obstacle.Id());
    if (adc_obstacle == nullptr) {
      return false;
    }

    const auto& adc_adc_boundary = adc_reference_line_info_->AdcSlBoundary();
    auto end_s = std::numeric_limits<double>::max();
    for (const auto& envelope : adc_obstacle->GetTrajectoryEnvelope()) {
      if (envelope.center_p.l() > adc_adc_boundary.start_l() &&
          envelope.center_p.l() < adc_adc_boundary.end_l()) {
        end_s = fmax(envelope.low_left_p.s(), envelope.low_right_p.s());
        end_s = fmax(end_s, envelope.upper_left_p.s());
        end_s = fmax(end_s, envelope.upper_right_p.s());
        break;
      }
    }

    if (end_s > adc_adc_boundary.start_s()) {
      return false;
    }
  }

  if (frame_ == nullptr || reference_line_info_ == nullptr ||
      frame_->GetReferenceLineProvider() == nullptr ||
      frame_->GetReferenceLineProvider()->GetPncMap() == nullptr) {
    return false;
  }

  const auto& pnc_map = frame_->GetReferenceLineProvider()->GetPncMap();
  const auto& adc_passage_routing_info = pnc_map->GetAdcPassageRoutingInfo();
  if (adc_passage_routing_info == nullptr) {
    return false;
  }

  const auto& obstacle_trajectory_points =
      obstacle.Trajectory().trajectory_point();
  if (obstacle_trajectory_points.empty()) {
    return false;
  }

  const auto& last_trajectory_point =
      obstacle_trajectory_points.at(obstacle_trajectory_points.size() - 1);
  common::SLPoint sl_point_passage;
  common::SLPoint sl_point_lane;
  adc_passage_routing_info->GetProjection(
      common::math::Vec2d(last_trajectory_point.path_point().x(),
                          last_trajectory_point.path_point().y()),
      &sl_point_passage, &sl_point_lane);

  const auto& adc_waypoint =
      reference_line_info_->reference_line().GetADCWaypoint();
  if (adc_waypoint.lane == nullptr) {
    return false;
  }

  double left_width = 0.0;
  double right_width = 0.0;
  adc_waypoint.lane->GetWidth(adc_waypoint.s, &left_width, &right_width);
  return sl_point_lane.l() < left_width && sl_point_lane.l() > -right_width;
}

bool STObstaclesProcessor::CheckIfIgnoreObstacle(Obstacle* const obstacle) {
  // ignore obstacle which has ignore decision
  if (obstacle == nullptr || (obstacle->HasLongitudinalDecision() &&
                              obstacle->LongitudinalDecision().has_ignore())) {
    return true;
  }

  if (reference_line_info_ == nullptr || adc_reference_line_info_ == nullptr) {
    return false;
  }

  if (frame_ != nullptr && frame_->local_view().HasFunctionManagerOut() &&
      frame_->local_view().GetFunctionManagerOut() != nullptr &&
      frame_->local_view().GetFunctionManagerOut()->fsm_state() ==
          functionmanager::MachineStateType::PERCEPTION_TYPE &&
      frame_->local_view().GetFunctionManagerOut()->perception_sub_state() ==
          functionmanager::PerceptionSubState::CRUISE_TYPE &&
      (!frame_->local_view().GetCruiseTargetId().empty() &&
       std::find(frame_->local_view().GetCruiseTargetId().begin(),
                 frame_->local_view().GetCruiseTargetId().end(),
                 obstacle->PerceptionId()) !=
           frame_->local_view().GetCruiseTargetId().end())) {
    ObjectDecisionType caution_decision;
    caution_decision.mutable_caution();
    obstacle->AddLongitudinalDecision("st_obstacle_processor",
                                      caution_decision);
    return false;
  }

  if (absl::StrContains(obstacle->Id(), "_intention")) {
    ObjectDecisionType caution_decision;
    caution_decision.mutable_caution();
    obstacle->AddLongitudinalDecision("st_obstacle_processor",
                                      caution_decision);
  }

  // #ifndef FOR_BAIDU_SIMULATION
  // #ifdef ISMDC
  // ignore reverse move obstacle
  //   const auto is_nnp_mode =
  //       frame_->local_view().HasFunctionManagerIn() &&
  //       functionmanager::AVP !=
  //           frame_->local_view().GetFunctionManagerIn()->ta_pilot_mode();
  //   if (is_nnp_mode && obstacle->HasTrajectory() &&
  //       obstacle->Trajectory().trajectory_point_size() >= 2) {
  //     const auto size = obstacle->Trajectory().trajectory_point_size();
  //     const auto obstacle_start_moving_direction =
  //         obstacle->Trajectory().trajectory_point(1).path_point().theta();
  //     const auto obstacle_end_moving_direction =
  //         obstacle->Trajectory().trajectory_point(size - 1).path_point().theta();
  //     const auto& vehicle_state = reference_line_info_->vehicle_state();
  //     double vehicle_moving_direction = vehicle_state.heading();
  //     if (vehicle_state.gear() == soc::Chassis::GEAR_REVERSE) {
  //       vehicle_moving_direction =
  //           common::math::NormalizeAngle(vehicle_moving_direction + M_PI);
  //     }
  //     const auto start_heading_difference = std::abs(common::math::NormalizeAngle(
  //         obstacle_start_moving_direction - vehicle_moving_direction));
  //     const auto end_heading_difference = std::abs(common::math::NormalizeAngle(
  //         obstacle_end_moving_direction - vehicle_moving_direction));
  //     const auto& end_point =
  //         obstacle->Trajectory().trajectory_point(size - 1).path_point();
  //     const auto is_straight_reverse_traj =
  //         start_heading_difference > (M_PI * 0.67) &&
  //         end_heading_difference > (M_PI * 0.67) &&
  //         fabs(end_heading_difference - start_heading_difference) < 0.26;
  //     const auto reverse_traj_to_adc_lane =
  //         end_heading_difference > (M_PI * 0.89) &&
  //         reference_line_info_->reference_line().IsOnLane(
  //             Vec2d(end_point.x(), end_point.y())) &&
  //         obstacle->Perception().type() ==
  //             perception::PerceptionObstacle::VEHICLE;
  //     if (is_straight_reverse_traj || reverse_traj_to_adc_lane) {
  //       return true;
  //     }
  //   }
  // #endif
  // NOLINTEND

  // calculate obstacle longitudinal position
  const auto longitudinal_position =
      CalculateObstacleLongitudinalPosition(*obstacle);

  // calculate obstacle lateral position
  const auto lateral_position = CalculateObstacleLateralPosition(*obstacle);

  // ignore directly behind obstacle
  if (CheckIfDirectlyBehind(*obstacle, lateral_position,
                            longitudinal_position)) {
    return true;
  }

  // if front / behind obstacle, if has overlap with path caution envelope, not ignore
  if (((lateral_position == ObstacleLateralPosition::LEFT ||
        lateral_position == ObstacleLateralPosition::CENTER) &&
       CheckIfHasOverlapWithPathCaution(
           *obstacle, PathData::PathCautionDirection::LEFT)) ||
      ((lateral_position == ObstacleLateralPosition::RIGHT ||
        lateral_position == ObstacleLateralPosition::CENTER) &&
       CheckIfHasOverlapWithPathCaution(
           *obstacle, PathData::PathCautionDirection::RIGHT))) {
    ObjectDecisionType caution_decision;
    caution_decision.mutable_caution();
    obstacle->AddLongitudinalDecision("st_obstacle_processor",
                                      caution_decision);
    return false;
  }

  // for front obstacle, if has overlap with path envelope, not ignore
  return longitudinal_position == ObstacleLongitudinalPosition::BACK ||
         !CheckIfHasOverlapWithPath(*obstacle);
}

bool STObstaclesProcessor::CheckIfEnterNeighborLane(
    const Vec2d& xy_corner, const common::SLPoint& sl_corner,
    ReferenceLineInfo::LaneType lane_type, double threshold,
    const std::set<std::string>& exclude_lane_ids) const {
  const auto neighbor_lane =
      reference_line_info_->GetNeighborLaneInfo(lane_type, sl_corner.s());
  if (neighbor_lane == nullptr ||
      exclude_lane_ids.count(neighbor_lane->lane().id().id()) > 0) {
    return false;
  }

  double neighbor_s = 0.0;
  double neighbor_l = 0.0;
  if (!neighbor_lane->GetProjection(xy_corner, &neighbor_s, &neighbor_l)) {
    return false;
  }

  double neighbor_lane_left_width = 0.0;
  double neighbor_lane_right_width = 0.0;
  neighbor_lane->GetWidth(neighbor_s, &neighbor_lane_left_width,
                          &neighbor_lane_right_width);

  return ((lane_type == ReferenceLineInfo::LaneType::LeftForward ||
           lane_type == ReferenceLineInfo::LaneType::LeftReverse) &&
          (neighbor_l > -neighbor_lane_right_width + threshold)) ||
         ((lane_type == ReferenceLineInfo::LaneType::RightForward ||
           lane_type == ReferenceLineInfo::LaneType::RightReverse) &&
          (neighbor_l < neighbor_lane_left_width - threshold));
}

void STObstaclesProcessor::CheckIfKeepRoadRight() {
  if (frame_ == nullptr || reference_line_info_ == nullptr) {
    keep_left_road_right_ = false;
    keep_right_road_right_ = false;
    return;
  }

  const auto& reference_line = reference_line_info_->reference_line();
  if (last_frame_ != nullptr &&
      last_frame_->DriveReferenceLineInfo() != nullptr) {
    const auto current_l = reference_line.GetADCWaypoint().l;
    const auto last_l = last_frame_->DriveReferenceLineInfo()
                            ->reference_line()
                            .GetADCWaypoint()
                            .l;
    static constexpr double kLaneChangeFinshLThreshold = 1.0;
    if (last_l > kLaneChangeFinshLThreshold &&
        current_l < -kLaneChangeFinshLThreshold) {
      keep_right_road_right_ = true;
    } else if (last_l < -kLaneChangeFinshLThreshold &&
               current_l > kLaneChangeFinshLThreshold) {
      keep_left_road_right_ = true;
    }
  }

  if (!keep_left_road_right_ && !keep_right_road_right_) {
    return;
  }

  const auto& init_point = frame_->PlanningStartPoint();
  common::SLPoint sl_point;
  reference_line.XYToSL(common::math::Vec2d{init_point.path_point().x(),
                                            init_point.path_point().y()},
                        &sl_point);
  const auto ref_point = reference_line.map_path().GetSmoothPoint(sl_point.s());

  if (fabs(cos(common::math::AngleDiff(ref_point.heading(),
                                       init_point.path_point().theta())) *
           init_point.v()) < 0.2) {
    keep_left_road_right_ = false;
    keep_right_road_right_ = false;
    ADEBUG << "lat speed is small, not need keep road right";
    return;
  }

  Vec2d center((vehicle_param_.front_edge_to_center() -
                vehicle_param_.back_edge_to_center()) *
                   0.5,
               (vehicle_param_.left_edge_to_center() -
                vehicle_param_.right_edge_to_center()) *
                   0.5);
  center.SelfRotate(init_point.path_point().theta());
  center.set_x(center.x() + init_point.path_point().x());
  center.set_y(center.y() + init_point.path_point().y());
  Box2d adc_box(center, init_point.path_point().theta(),
                vehicle_param_.length(), vehicle_param_.width());
  const auto& xy_corners = adc_box.GetAllCorners();
  if (xy_corners.size() != 4) {
    return;
  }

  if (keep_left_road_right_) {
    const auto& left_front_xy_corner = xy_corners.at(1);
    const auto& left_rear_xy_corner = xy_corners.at(2);
    common::SLPoint left_front_sl_corner;
    common::SLPoint left_rear_sl_corner;

    int index_min = -1;
    double radius = adc_box.half_length() * 4.0;
    reference_line.XYToSL(left_front_xy_corner, &left_front_sl_corner,
                          &index_min, radius, index_min);
    reference_line.XYToSL(left_rear_xy_corner, &left_rear_sl_corner, &index_min,
                          radius, index_min);

    if (!CheckIfEnterNeighborLane(left_front_xy_corner, left_front_sl_corner,
                                  ReferenceLineInfo::LaneType::LeftForward, 0.0,
                                  {}) &&
        !CheckIfEnterNeighborLane(left_rear_xy_corner, left_rear_sl_corner,
                                  ReferenceLineInfo::LaneType::LeftForward, 0.0,
                                  {})) {
      keep_left_road_right_ = false;
      ADEBUG << "already get left road right";
    }
  }

  if (keep_right_road_right_) {
    const auto& right_front_xy_corner = xy_corners.at(0);
    const auto& right_rear_xy_corner = xy_corners.at(3);
    common::SLPoint right_front_sl_corner;
    common::SLPoint right_rear_sl_corner;

    int index_min = -1;
    double radius = adc_box.half_length() * 4.0;
    reference_line.XYToSL(right_front_xy_corner, &right_front_sl_corner,
                          &index_min, radius, index_min);
    reference_line.XYToSL(right_rear_xy_corner, &right_rear_sl_corner,
                          &index_min, radius, index_min);

    // neighbor lane check if adc enter right forward neighbor lane
    if (!CheckIfEnterNeighborLane(right_front_xy_corner, right_front_sl_corner,
                                  ReferenceLineInfo::LaneType::RightForward,
                                  0.0, {}) &&
        !CheckIfEnterNeighborLane(right_rear_xy_corner, right_rear_sl_corner,
                                  ReferenceLineInfo::LaneType::RightForward,
                                  0.0, {})) {
      keep_right_road_right_ = false;
      ADEBUG << "already get right road right";
    }
  }
}

void STObstaclesProcessor::CalculatePathRoadRight() {
  if (frame_ == nullptr || reference_line_info_ == nullptr ||
      frame_->GetReferenceLineProvider() == nullptr ||
      frame_->GetReferenceLineProvider()->GetPncMap() == nullptr) {
    return;
  }

  CheckIfKeepRoadRight();
  ADEBUG << "keep_left_road_right:" << keep_left_road_right_
         << ", keep_right_road_right:" << keep_right_road_right_;

  const auto& pnc_map = frame_->GetReferenceLineProvider()->GetPncMap();
  const auto lane_merge_info = pnc_map->GetFrontLaneMergeInfo();

  const auto& hdmap = hdmap::HDMapUtil::MapForPlanning();

  std::set<std::string> split_lane_ids;
  if (pnc_map->GetAdcPassageRoutingInfo() != nullptr) {
    for (const auto& lane_id :
         pnc_map->GetAdcPassageRoutingInfo()->GetLaneIds()) {
      const auto lane_info = hdmap.GetLaneById(hdmap::MakeMapId(lane_id));
      if (lane_info == nullptr ||
          lane_info->lane().predecessor_id_size() <= 0) {
        continue;
      }

      const auto predecessor_info =
          hdmap.GetLaneById(lane_info->lane().predecessor_id(0));
      if (predecessor_info == nullptr ||
          predecessor_info->lane().successor_id_size() <= 1) {
        continue;
      }

      for (const auto& successor_lane_id :
           predecessor_info->lane().successor_id()) {
        split_lane_ids.insert(successor_lane_id.id());
      }
    }
  }

  const auto& reference_line = reference_line_info_->reference_line();
  const auto& frenet_frame_path =
      reference_line_info_->path_data().frenet_frame_path();
  if (frenet_frame_path.empty()) {
    return;
  }
  const auto adc_frenet_l = frenet_frame_path.front().l();

  const auto& discretized_path =
      reference_line_info_->path_data().discretized_path();
  std::vector<PathData::RoadRightType> path_road_right(
      discretized_path.size(), PathData::RoadRightType::HIGH_ROAD_RIGHT);

  const auto is_borrow_path =
      absl::StrContains(reference_line_info_->path_data().path_label(),
                        "lane_keep/left") ||
      absl::StrContains(reference_line_info_->path_data().path_label(),
                        "lane_keep/right");

  std::size_t point_count_per_thread =
      discretized_path.size() / kPathRightThreadCount + 1;
  std::vector<int> thread_indexs;
  for (std::size_t i = 0; i < kPathRightThreadCount; ++i) {
    thread_indexs.emplace_back(i);
  }

  const auto calculate_path_point_road_right = [&](const size_t thread_index) {
    const auto start_index = thread_index * point_count_per_thread;
    const auto end_index = start_index + point_count_per_thread;
    for (std::size_t i = start_index;
         i < end_index && i < discretized_path.size(); ++i) {
      const auto& discretized_point = discretized_path.at(i);

      // check if adc is in lane turn
      if (discretized_point.has_lane_id()) {
        const auto lane_info =
            hdmap.GetLaneById(hdmap::MakeMapId(discretized_point.lane_id()));
        if (lane_info == nullptr) {
          continue;
        }
        if (lane_info->lane().turn() == hdmap::Lane_LaneTurn_LEFT_TURN ||
            lane_info->lane().turn() == hdmap::Lane_LaneTurn_RIGHT_TURN ||
            lane_info->lane().turn() == hdmap::Lane_LaneTurn_U_TURN) {
          path_road_right.at(i) =
              path_road_right.at(i) |
              PathData::RoadRightType::LANE_TURN_LEFT_LOW_ROAD_RIGHT |
              PathData::RoadRightType::LANE_TURN_RIGHT_LOW_ROAD_RIGHT;
        }
      }

      Vec2d ego_center_map_frame((vehicle_param_.front_edge_to_center() -
                                  vehicle_param_.back_edge_to_center()) *
                                     0.5,
                                 (vehicle_param_.left_edge_to_center() -
                                  vehicle_param_.right_edge_to_center()) *
                                     0.5);
      ego_center_map_frame.SelfRotate(discretized_point.theta());
      ego_center_map_frame.set_x(ego_center_map_frame.x() +
                                 discretized_point.x());
      ego_center_map_frame.set_y(ego_center_map_frame.y() +
                                 discretized_point.y());

      // compute adc bounding box
      Box2d adc_box(ego_center_map_frame, discretized_point.theta(),
                    vehicle_param_.length(), vehicle_param_.width());

      // compute adc bounding box projection
      std::vector<common::SLPoint> sl_corners(4);
      const auto& right_front_sl_corner = sl_corners.at(0);
      const auto& left_front_sl_corner = sl_corners.at(1);
      const auto& left_rear_sl_corner = sl_corners.at(2);
      const auto& right_rear_sl_corner = sl_corners.at(3);

      const auto& xy_corners = adc_box.GetAllCorners();
      if (xy_corners.size() != 4 || xy_corners.size() != sl_corners.size()) {
        continue;
      }
      double start_s = std::numeric_limits<double>::max();
      double end_s = std::numeric_limits<double>::lowest();
      double start_l = std::numeric_limits<double>::max();
      double end_l = std::numeric_limits<double>::lowest();

      // int index_min =
      //     static_cast<int>(frenet_frame_point.s() / reference_line_interval);
      int index_min = -1;
      double radius = adc_box.half_length() * 4.0;
      for (std::size_t j = 0; j < xy_corners.size(); ++j) {
        auto& sl_corner = sl_corners.at(j);
        reference_line.XYToSL(xy_corners.at(j), &sl_corner, &index_min, radius,
                              index_min);
        start_s = fmin(start_s, sl_corner.s());
        end_s = fmax(end_s, sl_corner.s());
        start_l = fmin(start_l, sl_corner.l());
        end_l = fmax(end_l, sl_corner.l());
      }

      const auto& right_front_xy_corner = xy_corners.at(0);
      const auto& left_front_xy_corner = xy_corners.at(1);
      const auto& left_rear_xy_corner = xy_corners.at(2);
      const auto& right_rear_xy_corner = xy_corners.at(3);

      double lane_left_width = 0.0;
      double lane_right_width = 0.0;
      double middle_s = (start_s + end_s) / 2.0;
      if (!reference_line_info_->reference_line().GetLaneWidth(
              middle_s, &lane_left_width, &lane_right_width)) {
        ADEBUG << "Unable to get SL-boundary of ego-vehicle.";
        continue;
      }

      if (reference_line_info_->IsChangeLanePath()) {
        // this means lane change, we must check if adc enter target lane
        if ((start_l >= lane_left_width || end_l <= -lane_right_width) ||
            (adc_frenet_l > 0.0 &&
             right_rear_sl_corner.l() <
                 lane_left_width - kFullyEnterNeighborLanethreshold) ||
            (adc_frenet_l < 0.0 &&
             left_rear_sl_corner.l() >
                 -lane_right_width + kFullyEnterNeighborLanethreshold)) {
          // 1. adc has not started lane change yet
          // 2. right lane change, adc has full enter target lane
          // 3. left lane change, adc has full enter target lane
          path_road_right.at(i) = PathData::RoadRightType::HIGH_ROAD_RIGHT;
        } else {
          // this means adc across two lanes
          path_road_right.at(i) =
              path_road_right.at(i) |
              (adc_frenet_l > 0.0
                   ? PathData::RoadRightType::LANE_CROSS_RIGHT_LOW_ROAD_RIGHT
                   : PathData::RoadRightType::LANE_CROSS_LEFT_LOW_ROAD_RIGHT);
        }
        continue;
      }

      if (is_borrow_path &&
          (start_l < -lane_right_width || end_l > lane_left_width)) {
        // this means lane borrow, adc has leave current lane
        path_road_right.at(i) =
            path_road_right.at(i) |
            ((start_l < -lane_right_width)
                 ? PathData::RoadRightType::LANE_CROSS_RIGHT_LOW_ROAD_RIGHT
                 : PathData::RoadRightType::LANE_CROSS_LEFT_LOW_ROAD_RIGHT);
        continue;
      }

      // this means left lane merge, we must check if adc enter left neighbor
      // lane check if adc enter left forward neighbor lane
      if (!keep_left_road_right_ &&
          CheckIfEnterNeighborLane(left_front_xy_corner, left_front_sl_corner,
                                   ReferenceLineInfo::LaneType::LeftForward,
                                   0.0, split_lane_ids) &&
          !CheckIfEnterNeighborLane(left_rear_xy_corner, left_rear_sl_corner,
                                    ReferenceLineInfo::LaneType::LeftForward,
                                    kFullyEnterNeighborLanethreshold,
                                    split_lane_ids)) {
        path_road_right.at(i) =
            path_road_right.at(i) |
            PathData::RoadRightType::LANE_CROSS_LEFT_LOW_ROAD_RIGHT;
        continue;
      }

      // check if adc enter left reverse neighbor lane
      if (CheckIfEnterNeighborLane(left_front_xy_corner, left_front_sl_corner,
                                   ReferenceLineInfo::LaneType::LeftReverse,
                                   0.0, split_lane_ids) &&
          !CheckIfEnterNeighborLane(left_rear_xy_corner, left_rear_sl_corner,
                                    ReferenceLineInfo::LaneType::LeftReverse,
                                    kFullyEnterNeighborLanethreshold,
                                    split_lane_ids)) {
        path_road_right.at(i) =
            path_road_right.at(i) |
            PathData::RoadRightType::LANE_CROSS_LEFT_LOW_ROAD_RIGHT;
      }

      // neighbor lane check if adc enter right forward neighbor lane
      if (!keep_right_road_right_ &&
          CheckIfEnterNeighborLane(right_front_xy_corner, right_front_sl_corner,
                                   ReferenceLineInfo::LaneType::RightForward,
                                   0.0, split_lane_ids) &&
          !CheckIfEnterNeighborLane(right_rear_xy_corner, right_rear_sl_corner,
                                    ReferenceLineInfo::LaneType::RightForward,
                                    kFullyEnterNeighborLanethreshold,
                                    split_lane_ids)) {
        path_road_right.at(i) =
            path_road_right.at(i) |
            PathData::RoadRightType::LANE_CROSS_RIGHT_LOW_ROAD_RIGHT;
        continue;
      }

      // check if adc enter right reverse neighbor lane
      if (CheckIfEnterNeighborLane(right_front_xy_corner, right_front_sl_corner,
                                   ReferenceLineInfo::LaneType::RightReverse,
                                   0.0, split_lane_ids) &&
          !CheckIfEnterNeighborLane(right_rear_xy_corner, right_rear_sl_corner,
                                    ReferenceLineInfo::LaneType::RightReverse,
                                    kFullyEnterNeighborLanethreshold,
                                    split_lane_ids)) {
        path_road_right.at(i) =
            path_road_right.at(i) |
            PathData::RoadRightType::LANE_CROSS_RIGHT_LOW_ROAD_RIGHT;
      }
    }
  };

  TL::common::thread::ThreadPool::Instance()->ForEach(
      thread_indexs.begin(), thread_indexs.end(),
      calculate_path_point_road_right);

  auto* path_data = reference_line_info_->mutable_path_data();
  if (path_data != nullptr) {
    path_data->SetPathRoadRight(path_road_right);
  }
}

void STObstaclesProcessor::CalculatePathFrenetInfos() {
  const auto& vehicle_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  const auto& frenet_path =
      reference_line_info_->path_data().frenet_frame_path();
  const auto& discretized_path =
      reference_line_info_->path_data().discretized_path();

  std::vector<PathData::PathFrenetInfo> path_frenet_infos;
  path_frenet_infos.resize(discretized_path.size());

  // calculate origin path frenet info
  PathData::PathEnvelope path_envelope;
  for (std::size_t i = 0; i < discretized_path.size(); ++i) {
    const auto& frenet_point = frenet_path.at(i);
    const auto& discretized_point = discretized_path.at(i);

    // compute the ADC bounding box.
    Vec2d ego_center_map_frame((vehicle_param.front_edge_to_center() -
                                vehicle_param.back_edge_to_center()) *
                                   0.5,
                               (vehicle_param.left_edge_to_center() -
                                vehicle_param.right_edge_to_center()) *
                                   0.5);
    ego_center_map_frame.SelfRotate(discretized_point.theta());
    ego_center_map_frame.set_x(ego_center_map_frame.x() +
                               discretized_point.x());
    ego_center_map_frame.set_y(ego_center_map_frame.y() +
                               discretized_point.y());
    common::math::Box2d adc_box(ego_center_map_frame, discretized_point.theta(),
                                vehicle_param.length(), vehicle_param.width());

    // project corner
    common::SLPoint sl_point;
    auto& path_frenet_info = path_frenet_infos.at(i);
    path_frenet_info.s_lower = std::numeric_limits<double>::max();
    path_frenet_info.s_upper = std::numeric_limits<double>::lowest();
    path_frenet_info.l_lower = std::numeric_limits<double>::max();
    path_frenet_info.l_upper = std::numeric_limits<double>::lowest();

    // auto index_min =
    //     static_cast<int>(frenet_point.s() / reference_line_interval);
    auto index_min = -1;
    auto radius = adc_box.half_length() * 4.0;
    for (const auto& corner : adc_box.GetAllCorners()) {
      reference_line_info_->reference_line().XYToSL(
          corner, &sl_point, &index_min, radius, index_min);
      path_frenet_info.s_lower = fmin(path_frenet_info.s_lower, sl_point.s());
      path_frenet_info.s_upper = fmax(path_frenet_info.s_upper, sl_point.s());
      path_frenet_info.l_lower = fmin(path_frenet_info.l_lower, sl_point.l());
      path_frenet_info.l_upper = fmax(path_frenet_info.l_upper, sl_point.l());
    }
    const auto delta_angle = path_frenet_info.cos_theta =
        discretized_point.theta() - reference_line_info_->reference_line()
                                        .GetReferencePoint(frenet_point.s())
                                        .heading();
    path_frenet_info.cos_theta = cos(delta_angle);
    path_frenet_info.sin_theta = sin(delta_angle);

    path_envelope.min_ref_s =
        fmin(path_envelope.min_ref_s, path_frenet_info.s_lower);
    path_envelope.max_ref_s =
        fmax(path_envelope.max_ref_s, path_frenet_info.s_upper);
    path_envelope.min_ref_l =
        fmin(path_envelope.min_ref_l, path_frenet_info.l_lower);
    path_envelope.max_ref_l =
        fmax(path_envelope.max_ref_l, path_frenet_info.l_upper);
  }

  auto* path_data = reference_line_info_->mutable_path_data();
  if (path_data != nullptr) {
    path_data->SetPathFrenetInfos(std::move(path_frenet_infos));
    path_data->SetPathEnvelope(path_envelope);
  }
}

void STObstaclesProcessor::CalculatePathCautionEnvelopes() {
  std::vector<PathData::PathCautionEnvelope> path_caution_envelopes;
  CalculateLaneTurnPathCautionEnvelopes(
      PathData::RoadRightType::LANE_TURN_LEFT_LOW_ROAD_RIGHT,
      &path_caution_envelopes);
  CalculateLaneTurnPathCautionEnvelopes(
      PathData::RoadRightType::LANE_TURN_RIGHT_LOW_ROAD_RIGHT,
      &path_caution_envelopes);
  CalculateLaneCrossPathCautionEnvelopes(
      PathData::RoadRightType::LANE_CROSS_LEFT_LOW_ROAD_RIGHT,
      &path_caution_envelopes);
  CalculateLaneCrossPathCautionEnvelopes(
      PathData::RoadRightType::LANE_CROSS_RIGHT_LOW_ROAD_RIGHT,
      &path_caution_envelopes);
  auto* path_data = reference_line_info_->mutable_path_data();
  if (path_data != nullptr) {
    path_data->SetPathCautionEnvelopes(std::move(path_caution_envelopes));
  }
}

void STObstaclesProcessor::CalculateLaneTurnPathCautionEnvelopes(
    const PathData::RoadRightType& low_road_right_type,
    std::vector<PathData::PathCautionEnvelope>* caution_path_envelopes) {
  const auto& path_road_right =
      reference_line_info_->path_data().GetPathRoadRight();
  const auto path_frenet_infos =
      reference_line_info_->path_data().GetPathFrenetInfos();
  std::size_t start_ref_index = 0;
  std::size_t end_ref_index = 0;
  bool low_road_right_range_start = false;
  bool low_road_right_range_end = false;
  for (std::size_t i = 0; i < path_road_right.size(); ++i) {
    if (!low_road_right_range_start &&
        (path_road_right.at(i) & low_road_right_type) == low_road_right_type) {
      start_ref_index = i;
      low_road_right_range_start = true;
      continue;
    }

    if (low_road_right_range_start) {
      if ((path_road_right.at(i) & low_road_right_type) !=
          low_road_right_type) {
        end_ref_index = i - 1;
        low_road_right_range_end = true;
      } else if (i + 1 == path_road_right.size()) {
        end_ref_index = i;
        low_road_right_range_end = true;
      }
    }

    if (low_road_right_range_start && low_road_right_range_end &&
        start_ref_index < path_road_right.size() &&
        end_ref_index < path_road_right.size()) {
      const auto& path_envelope =
          reference_line_info_->path_data().GetPathEnvelope();
      caution_path_envelopes->emplace_back(PathData::PathCautionEnvelope{
          low_road_right_type ==
                  PathData::RoadRightType::LANE_TURN_LEFT_LOW_ROAD_RIGHT
              ? PathData::PathCautionDirection::LEFT
              : PathData::PathCautionDirection::RIGHT,
          PathData::PathEnvelope{path_frenet_infos.at(start_ref_index).s_lower,
                                 path_frenet_infos.at(end_ref_index).s_upper,
                                 path_envelope.min_ref_l,
                                 path_envelope.max_ref_l}});
      low_road_right_range_start = false;
      low_road_right_range_end = false;
    }
  }
}

void STObstaclesProcessor::CalculateLaneCrossPathCautionEnvelopes(
    const PathData::RoadRightType& low_road_right_type,
    std::vector<PathData::PathCautionEnvelope>* caution_path_envelopes) {
  const auto& path_road_right =
      reference_line_info_->path_data().GetPathRoadRight();
  for (std::size_t i = path_road_right.size() - 1; i < path_road_right.size();
       --i) {
    if ((path_road_right.at(i) & low_road_right_type) == low_road_right_type) {
      const auto& path_envelope =
          reference_line_info_->path_data().GetPathEnvelope();
      const auto path_frenet_infos =
          reference_line_info_->path_data().GetPathFrenetInfos();
      caution_path_envelopes->emplace_back(PathData::PathCautionEnvelope{
          low_road_right_type ==
                  PathData::RoadRightType::LANE_CROSS_LEFT_LOW_ROAD_RIGHT
              ? PathData::PathCautionDirection::LEFT
              : PathData::PathCautionDirection::RIGHT,
          PathData::PathEnvelope{
              path_envelope.min_ref_s, path_frenet_infos.at(i).s_upper,
              path_envelope.min_ref_l, path_envelope.max_ref_l}});
      break;
    }
  }
}

}  // namespace planning
}  // namespace TL
