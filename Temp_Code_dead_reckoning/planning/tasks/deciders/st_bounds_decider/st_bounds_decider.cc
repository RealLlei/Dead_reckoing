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

#include "planning/tasks/deciders/st_bounds_decider/st_bounds_decider.h"

#include <algorithm>
#include <limits>
#include <memory>

#include "common/math/double_type.h"
#include "common/time/clock.h"
#include "planning/common/st_graph_data.h"
#include "planning/tasks/deciders/st_bounds_decider/st_obstacles_processor.h"

namespace TL {
namespace planning {

using TL::common::ErrorCode;
using TL::common::Status;
using TL::planning_internal::StGraphBoundaryDebug;
using TL::planning_internal::STGraphDebug;

namespace {
// STBoundPoint contains (t, s_min, s_max)
using STBoundPoint = std::tuple<double, double, double>;
// STBound is a vector of STBoundPoints
using STBound = std::vector<STBoundPoint>;
// ObsDecSet is a set of decision for new obstacles.
using ObsDecSet = std::vector<std::pair<std::string, ObjectDecisionType>>;
}  // namespace

STBoundsDecider::STBoundsDecider(
    const TaskConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : Decider(config, injector) {
  if (config.has_st_bounds_decider_config()) {
    st_bounds_config_.CopyFrom(config.st_bounds_decider_config());
  }
}

Status STBoundsDecider::Process(Frame* const frame,
                                ReferenceLineInfo* const reference_line_info) {
  // Initialize the related helper classes.
  if (!InitSTBoundsDecider(*frame, reference_line_info)) {
    const std::string msg = "st_bounds_decider init failed";
    AERROR << msg;
    return Status(ErrorCode::PLANNER_CRUISING_STBOUNDS_ERROR, msg);
  }
  obs_fillter_.Process(*frame, reference_line_info);
  // Sweep the t-axis, and determine the s-boundaries step by step.
  STBound regular_st_bound;
  STBound regular_vt_bound;
  std::vector<std::pair<double, double>> st_guide_line;
  Status ret = GenerateRegularSTBound(&regular_st_bound, &regular_vt_bound,
                                      &st_guide_line);
  // LCOV_EXCL_START
  bool debug_in_baidu_sim = false;
#ifdef FOR_BAIDU_SIMULATION
  debug_in_baidu_sim = true;
#endif
  if (FLAGS_st_boundary_debug_info || debug_in_baidu_sim) {
    DebugForValidStBoundary();
  }
  // LCOV_EXCL_STOP
  if (!ret.ok()) {
    ADEBUG << "Cannot generate a regular ST-boundary.";
    return Status(ErrorCode::PLANNER_CRUISING_STBOUNDS_ERROR,
                  ret.error_message());
  }
  if (regular_st_bound.empty()) {
    const std::string msg = "Generated regular ST-boundary is empty.";
    AERROR << msg;
    return Status(ErrorCode::PLANNER_CRUISING_STBOUNDS_ERROR, msg);
  }
  StGraphData* st_graph_data = reference_line_info_->mutable_st_graph_data();
  st_graph_data->SetSTDrivableBoundary(regular_st_bound, regular_vt_bound);

  // Record the ST-Graph for good visualization and easy debugging.
  const auto& all_st_boundaries = st_obstacles_processor_.GetAllSTBoundaries();
  std::vector<STBoundary> st_boundaries;
  st_boundaries.reserve(all_st_boundaries.size());
  for (const auto& st_boundary : all_st_boundaries) {
    st_boundaries.push_back(st_boundary.second);
  }
  ADEBUG << "Total ST boundaries = " << st_boundaries.size();
  STGraphDebug* st_graph_debug = reference_line_info->mutable_debug()
                                     ->mutable_planning_data()
                                     ->add_st_graph();
  RecordSTGraphDebug(st_boundaries, regular_st_bound, st_guide_line,
                     st_graph_debug);

  return Status::OK();
}

bool STBoundsDecider::InitSTBoundsDecider(
    const Frame& frame, ReferenceLineInfo* const reference_line_info) {
  const PathData& path_data = reference_line_info->path_data();
  // AERROR<<"path_label:"<<path_data.path_label();
  PathDecision* path_decision = reference_line_info->path_decision();

  // Map all related obstacles onto ST-Graph.
  auto time1 = common::Clock::NowInSeconds();

  obstacle_intention_processor_.Process(frame, reference_line_info);
  const auto* last_frame =
      (injector_ != nullptr && injector_->frame_history() != nullptr)
          ? injector_->frame_history()->Latest()
          : nullptr;
  if (!st_obstacles_processor_.Init(
          path_data.discretized_path().Length(), FLAGS_trajectory_time_length,
          reference_line_info, frame_, injector_->history(), &st_bounds_config_,
          last_frame)) {
    AERROR << "st_obstacles_processor init failed";
    return false;
  }
  st_obstacles_processor_.MapObstaclesToSTBoundaries(path_decision);
  auto time2 = common::Clock::NowInSeconds();
  const double diff = time2 - time1;
  ADEBUG << "Time for ST Obstacles Processing = " << diff * 1000 << " msec.";

  if (!slt_obstacles_processor_.Init(
          reference_line_info->path_data().discretized_path().Length(),
          FLAGS_trajectory_time_length, reference_line_info, frame_,
          injector_->frame_history()->Latest())) {
    AERROR << "slt_obstacles_processor init failed";
    return false;
  }
  slt_obstacles_processor_.MapObstaclesToSLTBoundaries(
      reference_line_info->path_decision());

  // Initialize Guide-Line and Driving-Limits.
  static constexpr double desired_speed = 10.0;
  // If the path_data optimization is guided from a reference path of a
  // reference trajectory, use its reference speed profile to select the st
  // bounds in LaneFollow Hybrid Mode
  // if (path_data.is_optimized_towards_trajectory_reference()) {
  //   st_guide_line_.Init(desired_speed,
  //                       injector_->learning_based_data()
  //                           ->learning_data_adc_future_trajectory_points());
  // } else {
  st_guide_line_.Init(desired_speed);
  // }
  TL::common::VehicleParam vehicle_param;
  vehicle_param.CopyFrom(
      TL::common::VehicleConfigHelper::GetConfig().vehicle_param());
  const double max_acc = vehicle_param.max_acceleration();
  const double max_dec = fabs(vehicle_param.max_deceleration());
  const double max_v = reference_line_info_->GetMaxSpeed();
  st_driving_limits_.Init(max_acc, max_dec, max_v,
                          std::fabs(frame.PlanningStartPoint().v()));
  return true;
}

// Status STBoundsDecider::GenerateFallbackSTBound(STBound* const st_bound,
//                                                STBound* const vt_bound) {
//  // Initialize st-boundary.
//  for (double curr_t = 0.0; curr_t <= FLAGS_trajectory_time_length;
//       curr_t += kSTBoundsDeciderResolution) {
//    st_bound->emplace_back(curr_t, std::numeric_limits<double>::lowest(),
//                           std::numeric_limits<double>::max());
//    vt_bound->emplace_back(curr_t, std::numeric_limits<double>::lowest(),
//                           std::numeric_limits<double>::max());
//  }
//
//  // Sweep-line to get detailed ST-boundary.
//  for (size_t i = 0; i < st_bound->size(); ++i) {
//    double t, s_lower, s_upper, lower_obs_v, upper_obs_v;
//    std::tie(t, s_lower, s_upper) = st_bound->at(i);
//    std::tie(t, lower_obs_v, upper_obs_v) = vt_bound->at(i);
//    ADEBUG << "Processing st-boundary at t = " << t;
//
//    // Get Boundary due to driving limits
//    auto driving_limits_bound = st_driving_limits_.GetVehicleDynamicsLimits(t);
//    s_lower = std::fmax(s_lower, driving_limits_bound.first);
//    s_upper = std::fmin(s_upper, driving_limits_bound.second);
//    ADEBUG << "Bounds for s due to driving limits are "
//           << "s_upper = " << s_upper << ", s_lower = " << s_lower;
//
//    // Get Boundary due to obstacles
//    std::vector<std::pair<double, double>> available_s_bounds;
//    std::vector<ObsDecSet> available_obs_decisions;
//    if (!st_obstacles_processor_.GetSBoundsFromDecisions(
//            t, &available_s_bounds, &available_obs_decisions)) {
//      const std::string msg =
//          "Failed to find a proper boundary due to obstacles.";
//      AERROR << msg;
//      return Status(ErrorCode::PLANNER_CRUISING_STBOUNDS_ERROR, msg);
//    }
//    std::vector<std::pair<STBoundPoint, ObsDecSet>> available_choices;
//    ADEBUG << "Available choices are:";
//    for (int j = 0; j < static_cast<int>(available_s_bounds.size()); ++j) {
//      ADEBUG << "  (" << available_s_bounds[j].first << ", "
//             << available_s_bounds[j].second << ")";
//      available_choices.emplace_back(
//          std::make_tuple(0.0, available_s_bounds[j].first,
//                          available_s_bounds[j].second),
//          available_obs_decisions[j]);
//    }
//    RemoveInvalidDecisions(driving_limits_bound, &available_choices);
//
//    // Always go for the most conservative option.
//    if (!available_choices.empty()) {
//      // Select the most conservative decision.
//      auto top_choice_s_range = available_choices.front().first;
//      auto top_choice_decision = available_choices.front().second;
//      for (size_t j = 1; j < available_choices.size(); ++j) {
//        if (std::get<1>(available_choices[j].first) <
//            std::get<1>(top_choice_s_range)) {
//          top_choice_s_range = available_choices[j].first;
//          top_choice_decision = available_choices[j].second;
//        }
//      }
//
//      // Set decision for obstacles without decisions.
//      bool is_limited_by_upper_obs = false;
//      bool is_limited_by_lower_obs = false;
//      if (s_lower < std::get<1>(top_choice_s_range)) {
//        s_lower = std::get<1>(top_choice_s_range);
//        is_limited_by_lower_obs = true;
//      }
//      if (s_upper > std::get<2>(top_choice_s_range)) {
//        s_upper = std::get<2>(top_choice_s_range);
//        is_limited_by_upper_obs = true;
//      }
//      st_obstacles_processor_.SetObstacleDecision(top_choice_decision);
//
//      // Update st-guide-line, st-driving-limit info, and v-limits.
//      std::pair<double, double> limiting_speed_info;
//      if (st_obstacles_processor_.GetLimitingSpeedInfo(t,
//                                                       &limiting_speed_info)) {
//        st_driving_limits_.UpdateBlockingInfo(
//            t, s_lower, limiting_speed_info.first, s_upper,
//            limiting_speed_info.second);
//        st_guide_line_.UpdateBlockingInfo(t, s_lower, true);
//        st_guide_line_.UpdateBlockingInfo(t, s_upper, false);
//        if (is_limited_by_lower_obs) {
//          lower_obs_v = limiting_speed_info.first;
//        }
//        if (is_limited_by_upper_obs) {
//          upper_obs_v = limiting_speed_info.second;
//        }
//      }
//    } else {
//      const std::string msg = "No valid st-boundary exists.";
//      AERROR << msg;
//      return Status(ErrorCode::PLANNER_CRUISING_STBOUNDS_ERROR, msg);
//    }
//
//    // Update into st_bound
//    st_bound->at(i) = std::make_tuple(t, s_lower, s_upper);
//    vt_bound->at(i) = std::make_tuple(t, lower_obs_v, upper_obs_v);
//  }
//
//  return Status::OK();
// }

Status STBoundsDecider::GenerateRegularSTBound(
    STBound* const st_bound, STBound* const vt_bound,
    std::vector<std::pair<double, double>>* const st_guide_line) {
  // Initialize st-boundary.
  const auto t_count = static_cast<int>(
      std::round(FLAGS_trajectory_time_length / kSTBoundsDeciderResolution));
  for (int i = 0; i <= t_count; ++i) {
    const auto curr_t = i * kSTBoundsDeciderResolution;
    st_bound->emplace_back(curr_t, std::numeric_limits<double>::lowest(),
                           std::numeric_limits<double>::max());
    vt_bound->emplace_back(curr_t, std::numeric_limits<double>::lowest(),
                           std::numeric_limits<double>::max());
  }

  // Sweep-line to get detailed ST-boundary.
  for (size_t i = 0; i < st_bound->size(); ++i) {
    double t = 0;
    double s_lower = 0;
    double s_upper = 0;
    double lower_obs_v = 0;
    double upper_obs_v = 0;
    std::tie(t, s_lower, s_upper) = st_bound->at(i);
    std::tie(t, lower_obs_v, upper_obs_v) = vt_bound->at(i);
    ADEBUG << "Processing st-boundary at t = " << t << "  s_lower:" << s_lower
           << "  s_upper:" << s_upper << "   lower_obs_v:" << lower_obs_v
           << "   upper_obs_v:" << upper_obs_v;

    // Get Boundary due to driving limits
    auto driving_limits_bound = st_driving_limits_.GetVehicleDynamicsLimits(t);
    ADEBUG << "driving_limits_bound  first:" << driving_limits_bound.first
           << "  second:" << driving_limits_bound.second;
    s_lower = std::fmax(s_lower, driving_limits_bound.first);
    s_upper = std::fmin(s_upper, driving_limits_bound.second);
    ADEBUG << "Bounds for s due to driving limits are "
           << "s_upper = " << s_upper << ", s_lower = " << s_lower;
    // LCOV_EXCL_START
    if (FLAGS_enable_st_valid_bound_check) {
      // Get Boundary due to obstacles
      std::vector<std::pair<double, double>> available_s_bounds;
      std::vector<ObsDecSet> available_obs_decisions;
      if (!st_obstacles_processor_.GetSBoundsFromDecisions(
              t, &available_s_bounds, &available_obs_decisions)) {
        const std::string msg =
            "Failed to find a proper boundary due to obstacles.";
        AERROR << msg;
        return Status(ErrorCode::PLANNER_CRUISING_STBOUNDS_ERROR, msg);
      }
      std::vector<std::pair<STBoundPoint, ObsDecSet>> available_choices;
      ADEBUG << "Available choices are:";
      for (int j = 0; j < static_cast<int>(available_s_bounds.size()); ++j) {
        ADEBUG << "  (" << available_s_bounds[j].first << ", "
               << available_s_bounds[j].second << ")";
        available_choices.emplace_back(
            std::make_tuple(0.0, available_s_bounds[j].first,
                            available_s_bounds[j].second),
            available_obs_decisions[j]);
      }

      // if can't get the available choice, release max deceleration space.
      bool no_valid_decision =
          !CheckValidDecisions(driving_limits_bound, available_choices);
      if (no_valid_decision) {
        ADEBUG << "release vehicle maximum deceleration threshold.";
        double delta_acce = FLAGS_delta_acceleration;
        double max_dece_threshold = FLAGS_max_deceleration_backup;
        while (no_valid_decision) {
          st_driving_limits_.ChangeMaxDece(delta_acce);
          if (-st_driving_limits_.GetMaxDece() < max_dece_threshold ||
              TL::common::math::double_type::SeemsEqual(t, 0.0)) {
            const std::string msg =
                "No valid st-boundary exists. Can't find solver even "
                "breaking "
                "max deceleration in vehicle parameter.";
            AERROR << msg;
            ADEBUG
                << "valid st_boundary_lp:0  "
                << "   vehicle_velocity:"
                << reference_line_info_->vehicle_state().linear_velocity()
                << "   vehicle_acceleration:"
                << reference_line_info_->vehicle_state().linear_acceleration();
            return Status(ErrorCode::PLANNER_CRUISING_STBOUNDS_ERROR, msg);
          }
          driving_limits_bound = st_driving_limits_.GetVehicleDynamicsLimits(t);
          no_valid_decision =
              !CheckValidDecisions(driving_limits_bound, available_choices);
        }
      }

      RemoveInvalidDecisions(driving_limits_bound, &available_choices);

      if (!available_choices.empty()) {
        ADEBUG << "One decision needs to be made among "
               << available_choices.size() << " choices.";
        double guide_line_s = st_guide_line_.GetGuideSFromT(t);
        st_guide_line->emplace_back(t, guide_line_s);
        RankDecisions(guide_line_s, driving_limits_bound, &available_choices);
        // Select the top decision.
        auto top_choice_s_range = available_choices.front().first;
        bool is_limited_by_upper_obs = false;
        bool is_limited_by_lower_obs = false;
        if (s_lower < std::get<1>(top_choice_s_range)) {
          s_lower = std::get<1>(top_choice_s_range);
          is_limited_by_lower_obs = true;
        }
        if (s_upper > std::get<2>(top_choice_s_range)) {
          s_upper = std::get<2>(top_choice_s_range);
          is_limited_by_upper_obs = true;
        }

        // Set decision for obstacles without decisions.
        auto top_choice_decision = available_choices.front().second;
        st_obstacles_processor_.SetObstacleDecision(top_choice_decision);

        // Update st-guide-line, st-driving-limit info, and v-limits.
        std::pair<double, double> limiting_speed_info;
        if (st_obstacles_processor_.GetLimitingSpeedInfo(
                t, &limiting_speed_info)) {
          st_driving_limits_.UpdateBlockingInfo(
              t, s_lower, limiting_speed_info.first, s_upper,
              limiting_speed_info.second);
          st_guide_line_.UpdateBlockingInfo(t, s_lower, true);
          st_guide_line_.UpdateBlockingInfo(t, s_upper, false);
          if (is_limited_by_lower_obs) {
            lower_obs_v = limiting_speed_info.first;
          }
          if (is_limited_by_upper_obs) {
            upper_obs_v = limiting_speed_info.second;
          }
        }
      } else {
        const std::string msg = "No valid st-boundary exists.";
        AERROR << msg;
        ADEBUG << "valid st_boundary_lp:0  "
               << "   vehicle_velocity:"
               << reference_line_info_->vehicle_state().linear_velocity()
               << "   vehicle_acceleration:"
               << reference_line_info_->vehicle_state().linear_acceleration();
        return Status(ErrorCode::PLANNER_CRUISING_STBOUNDS_ERROR, msg);
      }
    }
    // LCOV_EXCL_STOP

    // Update into st_bound
    st_bound->at(i) = std::make_tuple(t, s_lower, s_upper);
    vt_bound->at(i) = std::make_tuple(t, lower_obs_v, upper_obs_v);
    reference_line_info_->SetMaxDeceleration(st_driving_limits_.GetMaxDece());
    ADEBUG << "valid st_boundary_lp:1";
  }
  return Status::OK();
}

// LCOV_EXCL_START
void STBoundsDecider::RemoveInvalidDecisions(
    std::pair<double, double> driving_limit,
    std::vector<std::pair<STBoundPoint, ObsDecSet>>* available_choices) {
  // Remove those choices that don't even fall within driving-limits.
  size_t i = 0;
  while (i < available_choices->size()) {
    double s_lower = 0.0;
    double s_upper = 0.0;
    std::tie(std::ignore, s_lower, s_upper) = available_choices->at(i).first;
    if (s_lower > driving_limit.second || s_upper < driving_limit.first) {
      // Invalid bound, should be removed.
      if (i != available_choices->size() - 1) {
        swap(available_choices->at(i),
             available_choices->at(available_choices->size() - 1));
      }
      available_choices->pop_back();
    } else {
      // Valid bound, proceed to the next one.
      ++i;
    }
  }
}

// LCOV_EXCL_STOP

// LCOV_EXCL_START
bool STBoundsDecider::CheckValidDecisions(
    std::pair<double, double> driving_limit,
    const std::vector<
        std::pair<std::tuple<double, double, double>,
                  std::vector<std::pair<std::string, ObjectDecisionType>>>>&
        available_choices) {
  return std::any_of(available_choices.begin(), available_choices.end(),
                     [&](const auto& choices) {
                       return std::get<1>(choices.first) <=
                                  driving_limit.second &&
                              std::get<2>(choices.first) >= driving_limit.first;
                     });
}

// LCOV_EXCL_STOP

// LCOV_EXCL_START
void STBoundsDecider::RankDecisions(
    double s_guide_line, std::pair<double, double> driving_limit,
    std::vector<std::pair<STBoundPoint, ObsDecSet>>* available_choices) {
  // Perform sorting of the existing decisions.
  bool has_swaps = true;
  while (has_swaps) {
    has_swaps = false;
    for (int i = 0; i < static_cast<int>(available_choices->size()) - 1; ++i) {
      double A_s_lower = 0.0;
      double A_s_upper = 0.0;
      std::tie(std::ignore, A_s_lower, A_s_upper) =
          available_choices->at(i).first;
      double B_s_lower = 0.0;
      double B_s_upper = 0.0;
      std::tie(std::ignore, B_s_lower, B_s_upper) =
          available_choices->at(i + 1).first;

      ADEBUG << "    Range ranking: A has s_upper = " << A_s_upper
             << ", s_lower = " << A_s_lower;
      ADEBUG << "    Range ranking: B has s_upper = " << B_s_upper
             << ", s_lower = " << B_s_lower;

      // If not both are larger than passable-threshold, should select
      // the one with larger room.
      double A_room = std::fmin(driving_limit.second, A_s_upper) -
                      std::fmax(driving_limit.first, A_s_lower);
      double B_room = std::fmin(driving_limit.second, B_s_upper) -
                      std::fmax(driving_limit.first, B_s_lower);
      if (A_room < kSTPassableThreshold || B_room < kSTPassableThreshold) {
        if (A_room < B_room) {
          swap(available_choices->at(i + 1), available_choices->at(i));
          has_swaps = true;
          ADEBUG << "Swapping to favor larger room.";
        }
        continue;
      }

      // Should select the one with overlap to guide-line
      bool A_contains_guideline =
          A_s_upper >= s_guide_line && A_s_lower <= s_guide_line;
      bool B_contains_guideline =
          B_s_upper >= s_guide_line && B_s_lower <= s_guide_line;
      if (A_contains_guideline != B_contains_guideline) {
        if (!A_contains_guideline) {
          swap(available_choices->at(i + 1), available_choices->at(i));
          has_swaps = true;
          ADEBUG << "Swapping to favor overlapping with guide-line.";
        }
        continue;
      }
    }
  }
}

// LCOV_EXCL_STOP
void STBoundsDecider::RecordSTGraphDebug(
    const std::vector<STBoundary>& st_graph_data, const STBound& st_bound,
    const std::vector<std::pair<double, double>>& st_guide_line,
    planning_internal::STGraphDebug* const st_graph_debug) {
  if (!FLAGS_enable_record_debug || (st_graph_debug == nullptr)) {
    ADEBUG << "Skip record debug info";
    return;
  }

  st_graph_debug->set_name("ST Bounds Boundary");

  // Plot ST-obstacle boundaries.
  for (const auto& boundary : st_graph_data) {
    auto* boundary_debug = st_graph_debug->add_boundary();
    boundary_debug->set_name(boundary.id());
    if (boundary.boundary_type() == STBoundary::BoundaryType::YIELD) {
      boundary_debug->set_type(StGraphBoundaryDebug::ST_BOUNDARY_TYPE_YIELD);
      ADEBUG << "Obstacle ID = " << boundary.id() << ", decision = YIELD";
    } else if (boundary.boundary_type() == STBoundary::BoundaryType::OVERTAKE) {
      boundary_debug->set_type(StGraphBoundaryDebug::ST_BOUNDARY_TYPE_OVERTAKE);
      ADEBUG << "Obstacle ID = " << boundary.id() << ", decision = OVERTAKE";
    } else {
      boundary_debug->set_type(StGraphBoundaryDebug::ST_BOUNDARY_TYPE_UNKNOWN);
      ADEBUG << "Obstacle ID = " << boundary.id() << ", decision = UNKNOWN";
    }

    for (const auto& point : boundary.points()) {
      auto* point_debug = boundary_debug->add_point();
      point_debug->set_t(point.x());
      point_debug->set_s(point.y());
    }
  }

  // Plot the chosen ST boundary.
  auto* boundary_debug = st_graph_debug->add_boundary();
  boundary_debug->set_name("Generated ST-Boundary");
  boundary_debug->set_type(
      StGraphBoundaryDebug::ST_BOUNDARY_TYPE_DRIVABLE_REGION);
  for (const auto& st_bound_pt : st_bound) {
    auto* point_debug = boundary_debug->add_point();
    double t = 0.0;
    double s_lower = 0.0;
    double s_upper = 0.0;
    std::tie(t, s_lower, s_upper) = st_bound_pt;
    point_debug->set_t(t);
    point_debug->set_s(s_lower);
    s_lower = s_lower < -1000 ? -1000 : s_lower;
    s_upper = s_upper > 1000 ? 1000 : s_upper;
    ADEBUG << FIXED << SETPRECISION(3) << "st_drivable_region  time:" << t
           << ", lower:" << s_lower << "  upper:" << s_upper;
  }

  for (auto st_bound_pt = st_bound.rbegin(); st_bound_pt != st_bound.rend();
       ++st_bound_pt) {
    auto* point_debug = boundary_debug->add_point();
    if (point_debug != nullptr) {
      point_debug->set_t(std::get<0>(*st_bound_pt));
      point_debug->set_s(std::get<2>(*st_bound_pt));
    }
  }

  // Plot the used st_guide_line when generating the st_bounds
  for (const auto& st_points : st_guide_line) {
    auto* speed_point = st_graph_debug->add_speed_profile();
    speed_point->set_t(st_points.first);
    speed_point->set_s(st_points.second);
  }
}

// LCOV_EXCL_START
void STBoundsDecider::DebugForValidStBoundary() {
  auto vehicle_param_ =
      common::VehicleConfigHelper::GetConfig().vehicle_param();

  auto AdcBox = [&](double heading, double x, double y) {
    Vec2d veh_vec((vehicle_param_.front_edge_to_center() -
                   vehicle_param_.back_edge_to_center()) *
                      0.5,
                  (vehicle_param_.left_edge_to_center() -
                   vehicle_param_.right_edge_to_center()) *
                      0.5);
    veh_vec.SelfRotate(heading);
    veh_vec.set_x(veh_vec.x() + x);
    veh_vec.set_y(veh_vec.y() + y);

    TL::common::math::Box2d adc_box(
        veh_vec, heading, vehicle_param_.length(), vehicle_param_.width());

    return adc_box;
  };

  // planning
  for (const auto& traj :
       reference_line_info_->path_data().discretized_path()) {
    ADEBUG << FIXED << SETPRECISION(3)
           << "path_planning_st_trajectory_x_:" << traj.x()
           << "  path_planning_st_trajectory_y_:" << traj.y();

    auto adc_box_1 = AdcBox(traj.theta(), traj.x(), traj.y());
    for (auto point : adc_box_1.GetAllCorners()) {
      ADEBUG << FIXED << SETPRECISION(3) << "st_veh_x:" << point.x() << "  "
             << "st_veh_y:" << point.y();
    }
    for (auto point : adc_box_1.GetAllCorners()) {
      ADEBUG << FIXED << SETPRECISION(3) << "st_veh_x:" << point.x() << "  "
             << "st_veh_y:" << point.y();
      break;
    }
  }

  const auto& veh_state = reference_line_info_->vehicle_state();
  ADEBUG << FIXED << SETPRECISION(3)
         << "st_veh_speed:" << veh_state.linear_velocity()
         << "   st_veh_acce:" << veh_state.linear_acceleration()
         << "   st_veh_pos_x:" << veh_state.x()
         << "   st_veh_pos_y:" << veh_state.y()
         << "   st_veh_pos_heading:" << veh_state.heading()
         << "   length_:" << vehicle_param_.length()
         << "   front_axis_:" << vehicle_param_.front_edge_to_center()
         << "   back_axis_:" << vehicle_param_.back_edge_to_center();

  auto adc_box = AdcBox(veh_state.heading(), veh_state.x(), veh_state.y());
  for (const auto& point : adc_box.GetAllCorners()) {
    ADEBUG << FIXED << SETPRECISION(3) << "st_veh_x:" << point.x() << "  "
           << "st_veh_y:" << point.y();
  }
  for (auto point : adc_box.GetAllCorners()) {
    ADEBUG << FIXED << SETPRECISION(3) << "st_veh_x:" << point.x() << "  "
           << "st_veh_y:" << point.y();
    break;
  }

  // lp: log obstacle information
  for (const auto* obs :
       reference_line_info_->path_decision()->obstacles().Items()) {
    auto obs_id = "St_obs_" + obs->Id() + "_";
    TL::common::math::Box2d obs_box(
        {obs->PerceptionBoundingBox().center_x(),
         obs->PerceptionBoundingBox().center_y()},
        obs->PerceptionBoundingBox().heading(),
        obs->PerceptionBoundingBox().length(),
        obs->PerceptionBoundingBox().width());
    if (obs->IsStatic()) {
      for (auto point : obs_box.GetAllCorners()) {
        ADEBUG << FIXED << SETPRECISION(3) << "static "
               << obs_id + "x:" << point.x() << "  "
               << obs_id + "y:" << point.y();
      }
      for (auto point : obs_box.GetAllCorners()) {
        ADEBUG << FIXED << SETPRECISION(3) << "static "
               << obs_id + "x:" << point.x() << "  "
               << obs_id + "y:" << point.y();
        break;
      }
    } else {
      double time_start = -100.0;
      double time_end = -100.0;
      for (const auto& t_edge : st_obstacles_processor_.GetObsTEdges()) {
        if (std::get<4>(t_edge) == obs->Id() && std::get<0>(t_edge) == 1) {
          time_start = std::get<1>(t_edge);
        }
        if (std::get<4>(t_edge) == obs->Id() && std::get<0>(t_edge) == 0) {
          time_end = std::get<1>(t_edge);
        }
      }

      for (const auto& point : obs_box.GetAllCorners()) {
        ADEBUG << FIXED << SETPRECISION(3) << "dynamic "
               << obs_id + "x:" << point.x() << "  "
               << obs_id + "y:" << point.y();
      }
      for (const auto& point : obs_box.GetAllCorners()) {
        ADEBUG << FIXED << SETPRECISION(3) << "dynamic "
               << obs_id + "x:" << point.x() << "  "
               << obs_id + "y:" << point.y();
        break;
      }
      for (const auto& traj : obs->Trajectory().trajectory_point()) {
        if (!TL::common::math::double_type::AlmostEqual(time_start,
                                                           -100.0) &&
            TL::common::math::double_type::AlmostEqual(traj.relative_time(),
                                                          time_start)) {
          TL::common::math::Box2d obs_box_1(
              {traj.path_point().x(), traj.path_point().y()},
              traj.path_point().theta(), obs->PerceptionBoundingBox().length(),
              obs->PerceptionBoundingBox().width());
          for (const auto& point : obs_box_1.GetAllCorners()) {
            ADEBUG << FIXED << SETPRECISION(3) << "dynamic "
                   << obs_id + "x:" << point.x() << "  "
                   << obs_id + "y:" << point.y();
          }
          for (const auto& point : obs_box_1.GetAllCorners()) {
            ADEBUG << FIXED << SETPRECISION(3) << "dynamic "
                   << obs_id + "x:" << point.x() << "  "
                   << obs_id + "y:" << point.y();
            break;
          }
        }

        if (!TL::common::math::double_type::AlmostEqual(time_end, -100.0) &&
            TL::common::math::double_type::AlmostEqual(traj.relative_time(),
                                                          time_end)) {
          TL::common::math::Box2d obs_box_1(
              {traj.path_point().x(), traj.path_point().y()},
              traj.path_point().theta(), obs->PerceptionBoundingBox().length(),
              obs->PerceptionBoundingBox().width());
          for (auto point : obs_box_1.GetAllCorners()) {
            ADEBUG << FIXED << SETPRECISION(3) << "dynamic "
                   << obs_id + "x:" << point.x() << "  "
                   << obs_id + "y:" << point.y();
          }
          for (const auto& point : obs_box_1.GetAllCorners()) {
            ADEBUG << FIXED << SETPRECISION(3) << "dynamic "
                   << obs_id + "x:" << point.x() << "  "
                   << obs_id + "y:" << point.y();
            break;
          }
        }
      }
      for (const auto& traj : obs->Trajectory().trajectory_point()) {
        ADEBUG << FIXED << SETPRECISION(3)
               << obs_id + "x:" << traj.path_point().x() << "   "
               << obs_id + "y:" << traj.path_point().y() << "   " << obs_id
               << "heading:" << traj.path_point().theta();
      }
    }
  }
}

// LCOV_EXCL_STOP

}  // namespace planning
}  // namespace TL
