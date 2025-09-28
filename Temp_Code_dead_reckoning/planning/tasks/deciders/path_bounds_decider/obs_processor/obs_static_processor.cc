/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  planning path static obs processor
 * Author: ROC
 */

#include "planning/tasks/deciders/path_bounds_decider/obs_processor/obs_static_processor.h"
#include <cmath>
#include <complex>
#include <cstddef>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "common/file/log.h"
#include "common/math/double_type.h"
#include "common/math/vec2d.h"
#include "common/time/clock.h"
#include "planning/common/obstacle.h"
// #include "planning/reference_line/util/reference_line_debug.h"
#include "absl/strings/match.h"
#include "common/math/linear_interpolation.h"
#include "planning/tasks/deciders/path_bounds_decider/util/path_info.h"
#include "planning/tasks/deciders/utils/path_decider_obstacle_utils.h"
#include "proto/common/vehicle_config.pb.h"

namespace TL {
namespace planning {

using common::math::double_type::Compare;
using TL::common::VehicleConfigHelper;

ObsStaticProcessor::ObsStaticProcessor(
    const std::shared_ptr<DependencyInjector>& injector,
    const TaskConfig& config)
    : ObsProcessor(injector, config) {
  CHECK_NOTNULL(GetInjector());
}

// Currently, it processes each obstacle based on its frenet-frame
// projection. Therefore, it might be overly conservative when processing
// obstacles whose headings differ from road-headings a lot.
// TODO(all): (future work) this can be improved in the future.
bool ObsStaticProcessor::Process(ReferenceLineInfo* const reference_line_info,
                                 Frame* const frame,
                                 PathBound* const path_boundaries,
                                 std::string* const blocking_obstacle_id,
                                 TowingPointsInfo* const towing_points,
                                 const bool is_enable_towing_process) {
  if (reference_line_info == nullptr || frame == nullptr ||
      path_boundaries == nullptr || blocking_obstacle_id == nullptr ||
      towing_points == nullptr) {
    AERROR << "Obs static processor nullptr check is failed";
    return false;
  }
  // 1.bound process init.
  GetProcessBound()->InitPathBounds(frame, reference_line_info);
  double start_time = common::Clock::NowInMicroseconds();

  const auto& indexed_obstacles =
      reference_line_info->path_decision()->obstacles();
  auto& sorted_obstacles = obstacle_edges_;
  const auto& need_nudge_obstacles_edges = need_nudge_obstacle_edges_;

  // 3.prepare necessary variables
  double center_line = GetProcessBound()->GetAdcFrenetL();
  size_t obs_idx = 0;
  int path_blocked_idx = -1;
  std::multiset<std::pair<double, std::string>, std::greater<>> right_bounds;
  right_bounds.insert(
      std::make_pair(std::numeric_limits<double>::lowest(), " "));
  std::multiset<std::pair<double, std::string>> left_bounds;
  left_bounds.insert(std::make_pair(std::numeric_limits<double>::max(), " "));
  std::pair<double, double> default_static_obs_towing_prepare_distance = {0.0,
                                                                          0.0};
  // calculate towing prepare distance
  UseTtcCalculateTowingPrepareDistance(
      reference_line_info, &default_static_obs_towing_prepare_distance, true);

  // 4.Maps obstacle ID's to the decision of whether has kinematic corner.
  //         1. obs id; 2. kinematic_corner; 3. has_obs_corner
  std::unordered_map<std::string, std::pair<Vec2d, bool>> obs_kinematic_corner;
  bool is_calculate_kinematic_corner_failed = false;
  if (GetProcessBound()
          ->GetObsTowingConf()
          .use_kinematic_corner_calculate_static_obstacle_bound()) {
    if (!GetObstacleKinematicCorner(&sorted_obstacles, indexed_obstacles,
                                    &obs_kinematic_corner)) {
      AERROR << "Get obstacle kinematic corner failed!";
      is_calculate_kinematic_corner_failed = true;
    }
  }

  // 5.Step through every path point.
  std::vector<bool> is_obs_affected(path_boundaries->size(), false);
  for (size_t i = 0; i < path_boundaries->size(); ++i) {
    double curr_s = std::get<0>((*path_boundaries)[i]);
    obs_idx = 0;
    left_bounds.clear();
    left_bounds.insert(std::make_pair(std::numeric_limits<double>::max(), " "));
    right_bounds.clear();
    right_bounds.insert(
        std::make_pair(std::numeric_limits<double>::lowest(), " "));

    // filter the obstacles not in the curr_s scope.
    while (obs_idx < sorted_obstacles.size() &&
           Compare(sorted_obstacles[obs_idx].obstacle_edge_start_s, curr_s) <=
               0) {
      // A new obstacle enters into our scope:
      //   - Decide which direction for the ADC to pass.
      //   - Update the left/right bound accordingly.
      //   - If boundaries blocked, then decide whether can side-pass.
      //   - If yes, then borrow neighbor lane to side-pass.

      // if obstacle totally behind curr_s, do not process.
      if (Compare(sorted_obstacles[obs_idx].obstacle_edge_end_s, curr_s) < 0) {
        ++obs_idx;
        continue;
      }

      // only process the obstacle that curr_s between obs_start_s and obs_end_s
      const auto& curr_obstacle = sorted_obstacles[obs_idx];
      const double curr_obstacle_start_s = curr_obstacle.obstacle_edge_start_s;
      const double curr_obstacle_end_s = curr_obstacle.obstacle_edge_end_s;
      const double curr_obstacle_box_l_min = curr_obstacle.obstacle_edge_l_min;
      const double curr_obstacle_box_l_max = curr_obstacle.obstacle_edge_l_max;
      const std::string curr_obstacle_id = curr_obstacle.obstacle_id;
      std::pair<double, double> cross_points;
      double curr_obstacle_l_min = curr_obstacle.obstacle_edge_l_min;
      double curr_obstacle_l_max = curr_obstacle.obstacle_edge_l_max;

      // calculate accurate bound while successfully use kinematic corner
      if (GetProcessBound()
              ->GetObsTowingConf()
              .use_kinematic_corner_calculate_static_obstacle_bound() &&
          !is_calculate_kinematic_corner_failed) {
        is_obs_affected[i] =
            GetStaticObstacleBound(
                reference_line_info->reference_line().GetNearestReferencePoint(
                    curr_s),
                *(indexed_obstacles.Find(curr_obstacle_id)), &cross_points,
                obs_kinematic_corner[curr_obstacle_id]) ||
            is_obs_affected[i];

        curr_obstacle_l_min =
            fabs(cross_points.first - curr_obstacle.obstacle_edge_l_min) <
                    reference_line_info->PathBoundsDeciderResolution()
                ? curr_obstacle.obstacle_edge_l_min
                : cross_points.first;
        curr_obstacle_l_max =
            fabs(cross_points.second - curr_obstacle.obstacle_edge_l_max) <
                    reference_line_info->PathBoundsDeciderResolution()
                ? curr_obstacle.obstacle_edge_l_max
                : cross_points.second;
      }

      // set debug info
      SetStaticObstacleBoundDebugInfo(curr_obstacle);
      std::pair<double, double> static_obs_towing_prepare_distance = {
          default_static_obs_towing_prepare_distance.first,
          default_static_obs_towing_prepare_distance.second};
      std::pair<double, double> final_obs_edges = {curr_obstacle_start_s,
                                                   curr_obstacle_end_s};
      if (!CalculateStaticTowingPrepareDistance(
              sorted_obstacles, final_obs_edges,
              default_static_obs_towing_prepare_distance,
              &static_obs_towing_prepare_distance, obs_idx)) {
        ADEBUG << "Calculate static towing prepare distance failed, use "
                  "default prepare distance.";
      }
      double static_obs_expect_towing_l =
          GetProcessBound()->GetObsTowingConf().default_obstacle_towing_l();
      bool curr_obs_first_exist = false;
      const Obstacle* last_obs = nullptr;
      if (GetInjector() == nullptr ||
          GetInjector()->frame_history() == nullptr ||
          GetInjector()->frame_history()->Latest() == nullptr ||
          GetInjector()->frame_history()->Latest()->DriveReferenceLineInfo() ==
              nullptr) {
        curr_obs_first_exist = true;
      } else {
        last_obs = GetInjector()
                       ->frame_history()
                       ->Latest()
                       ->DriveReferenceLineInfo()
                       ->path_decision()
                       .obstacles()
                       .Find(curr_obstacle_id);
        if (last_obs == nullptr) {
          curr_obs_first_exist = true;
        }
      }
      if (Compare((curr_obstacle_box_l_min + curr_obstacle_box_l_max) / 2,
                  center_line) < 0) {
        // Obstacle is to the right of center-line, should pass from left.
        right_bounds.insert(
            std::make_pair(curr_obstacle_l_max, curr_obstacle_id));
        if (is_enable_towing_process &&
            GetProcessBound()
                ->GetObsTowingConf()
                .enable_static_obstacle_towing() &&
            !GetProcessBound()
                 ->GetObsTowingConf()
                 .use_decision_tags_set_towing_points()) {
          ADEBUG << "left: obs id = " << curr_obstacle_id
                 << " is prepared for static obstacle towing";
          // check history obs towing value

          double curr_obs_expect_towing_l =
              LookUpTowingLDistance(std::abs(curr_obstacle_l_max),
                                    GetProcessBound()
                                        ->GetObsTowingConf()
                                        .towing_l_distance_segment_conf()
                                        .obs_to_center_line_dis_segment(),
                                    GetProcessBound()
                                        ->GetObsTowingConf()
                                        .towing_l_distance_segment_conf()
                                        .static_obs_expect_towing_segment_l());
          static_obs_expect_towing_l =
              FLAGS_use_dynamic_adjust_towing_l
                  ? curr_obs_first_exist
                        ? curr_obs_expect_towing_l
                        : std::max(curr_obs_expect_towing_l,
                                   last_obs->GetMaxExpectTowingL())
                  : GetProcessBound()
                        ->GetObsTowingConf()
                        .default_obstacle_towing_l();
          ADEBUG << "switch: " << FLAGS_use_dynamic_adjust_towing_l
                 << ", curr_obstacle_l_max = " << curr_obstacle_l_max
                 << ", static_obs_expect_towing_l = "
                 << static_obs_expect_towing_l
                 << ", curr_obs_expect_towing_l = " << curr_obs_expect_towing_l;
          SetTowingPoints(static_obs_towing_prepare_distance, final_obs_edges,
                          curr_obstacle_id, reference_line_info,
                          path_boundaries, towing_points,
                          static_obs_expect_towing_l, true);
        }
      } else {
        // Obstacle is to the left of center-line, should pass from right.
        left_bounds.insert(
            std::make_pair(curr_obstacle_l_min, curr_obstacle_id));
        if (is_enable_towing_process &&
            GetProcessBound()
                ->GetObsTowingConf()
                .enable_static_obstacle_towing() &&
            !GetProcessBound()
                 ->GetObsTowingConf()
                 .use_decision_tags_set_towing_points()) {
          ADEBUG << "right: obs id = " << curr_obstacle_id
                 << " is prepared for static obstacle towing";

          double curr_obs_expect_towing_l =
              LookUpTowingLDistance(std::abs(curr_obstacle_l_min),
                                    GetProcessBound()
                                        ->GetObsTowingConf()
                                        .towing_l_distance_segment_conf()
                                        .obs_to_center_line_dis_segment(),
                                    GetProcessBound()
                                        ->GetObsTowingConf()
                                        .towing_l_distance_segment_conf()
                                        .static_obs_expect_towing_segment_l());
          static_obs_expect_towing_l =
              FLAGS_use_dynamic_adjust_towing_l
                  ? curr_obs_first_exist
                        ? curr_obs_expect_towing_l
                        : std::max(curr_obs_expect_towing_l,
                                   last_obs->GetMaxExpectTowingL())
                  : GetProcessBound()
                        ->GetObsTowingConf()
                        .default_obstacle_towing_l();
          ADEBUG << "switch: " << FLAGS_use_dynamic_adjust_towing_l
                 << ", curr_obstacle_l_min = " << curr_obstacle_l_min
                 << ", static_obs_expect_towing_l = "
                 << static_obs_expect_towing_l
                 << ", curr_obs_expect_towing_l = " << curr_obs_expect_towing_l;
          SetTowingPoints(static_obs_towing_prepare_distance, final_obs_edges,
                          curr_obstacle_id, reference_line_info,
                          path_boundaries, towing_points,
                          static_obs_expect_towing_l, false);
        }
      }

      // Update the bounds and center_line.
      if (!UpdatePathBoundaryAndCenterLineWithBuffer(
              i, left_bounds.begin()->first, right_bounds.begin()->first,
              path_boundaries, &center_line, *reference_line_info)) {
        path_blocked_idx = static_cast<int>(i);
        *blocking_obstacle_id = curr_obstacle_id;
        ADEBUG << "block id = " << *blocking_obstacle_id;
        break;
      }

      const auto right_bound =
          std::fmax(std::get<1>((*path_boundaries)[i]),
                    right_bounds.begin()->first +
                        ProcessBound::GetBufferBetweenADCCenterAndEdge());
      const auto left_bound =
          std::fmin(std::get<2>((*path_boundaries)[i]),
                    left_bounds.begin()->first -
                        ProcessBound::GetBufferBetweenADCCenterAndEdge());
      if (Compare(right_bound, left_bound) > 0) {
        ADEBUG << "Path is blocked at s = " << curr_s;
        path_blocked_idx = static_cast<int>(i);
        *blocking_obstacle_id = curr_obstacle_id;
        ADEBUG << "block id = " << *blocking_obstacle_id;
        break;
      }
      center_line = (std::get<1>((*path_boundaries)[i]) +
                     std::get<2>((*path_boundaries)[i])) /
                    2;
      ++obs_idx;
    }

    // If no obstacle influence curr_s, update center_line.
    center_line = (std::get<1>((*path_boundaries)[i]) +
                   std::get<2>((*path_boundaries)[i])) /
                  2;

    if (FLAGS_enable_path_bound_debug) {
      auto* const planning_data = GetProcessBound()
                                      ->GetMutableReferenceLineInfo()
                                      ->mutable_debug()
                                      ->mutable_planning_data();
      auto* const static_obstacles_path_bound = planning_data->mutable_path(0);
      TL::planning::PathBoundPointDebug* path_debug = nullptr;
      path_debug = static_obstacles_path_bound->mutable_path_bound_debug()
                       ->mutable_path_bound(static_cast<int>(i))
                       ->mutable_static_obstacles_path_boundaries();
      path_debug->set_s(std::get<0>((*path_boundaries)[i]));
      path_debug->set_l_min(std::get<1>((*path_boundaries)[i]));
      path_debug->set_l_max(std::get<2>((*path_boundaries)[i]));
    }

    // Early exit if path is blocked.
    if (path_blocked_idx != -1) {
      break;
    }
  }

  // 6.Towing process by decision
  if (is_enable_towing_process &&
      GetProcessBound()->GetObsTowingConf().enable_static_obstacle_towing() &&
      GetProcessBound()
          ->GetObsTowingConf()
          .use_decision_tags_set_towing_points()) {
    size_t curr_obs_idx = 0;
    for (const ObstacleEdge& curr_obs_edge : need_nudge_obstacles_edges) {
      std::pair<double, double> static_obs_towing_prepare_distance = {
          default_static_obs_towing_prepare_distance.first,
          default_static_obs_towing_prepare_distance.second};
      std::pair<double, double> final_obs_edges = {
          curr_obs_edge.obstacle_edge_start_s,
          curr_obs_edge.obstacle_edge_end_s};
      if (!CalculateStaticTowingPrepareDistance(
              need_nudge_obstacles_edges, final_obs_edges,
              default_static_obs_towing_prepare_distance,
              &static_obs_towing_prepare_distance, curr_obs_idx)) {
        AERROR << "Calculate static towing prepare distance failed, use "
                  "default prepare distance.";
      }
      const auto* const curr_obs = GetProcessBound()
                                       ->GetReferenceLineInfo()
                                       ->path_decision()
                                       .obstacles()
                                       .Find(curr_obs_edge.obstacle_id);
      if (curr_obs == nullptr) {
        ADEBUG << "Find obstacle failed!";
        continue;
      }
      // check history obs towing value
      bool curr_obs_first_exist = false;
      const Obstacle* last_obs = nullptr;
      if (GetInjector() == nullptr ||
          GetInjector()->frame_history() == nullptr ||
          GetInjector()->frame_history()->Latest() == nullptr ||
          GetInjector()->frame_history()->Latest()->DriveReferenceLineInfo() ==
              nullptr) {
        curr_obs_first_exist = true;
      } else {
        last_obs = GetInjector()
                       ->frame_history()
                       ->Latest()
                       ->DriveReferenceLineInfo()
                       ->path_decision()
                       .obstacles()
                       .Find(curr_obs_edge.obstacle_id);
        if (last_obs == nullptr) {
          curr_obs_first_exist = true;
        }
      }
      for (const auto& decider_tag : curr_obs->decider_tags()) {
        if (absl::StrContains(decider_tag, "static-left-nudge")) {
          ADEBUG << "left: obs id = " << curr_obs_edge.obstacle_id
                 << " is prepared for static obstacle towing";
          double curr_obs_expect_towing_l =
              LookUpTowingLDistance(std::abs(curr_obs_edge.obstacle_edge_l_max),
                                    GetProcessBound()
                                        ->GetObsTowingConf()
                                        .towing_l_distance_segment_conf()
                                        .obs_to_center_line_dis_segment(),
                                    GetProcessBound()
                                        ->GetObsTowingConf()
                                        .towing_l_distance_segment_conf()
                                        .static_obs_expect_towing_segment_l());
          const double static_obs_expect_towing_l =
              FLAGS_use_dynamic_adjust_towing_l
                  ? curr_obs_first_exist
                        ? curr_obs_expect_towing_l
                        : std::max(curr_obs_expect_towing_l,
                                   last_obs->GetMaxExpectTowingL())
                  : GetProcessBound()
                        ->GetObsTowingConf()
                        .default_obstacle_towing_l();
          ADEBUG << "switch: " << FLAGS_use_dynamic_adjust_towing_l
                 << ", curr_obstacle_l_max = "
                 << curr_obs_edge.obstacle_edge_l_max
                 << ", static_obs_expect_towing_l = "
                 << static_obs_expect_towing_l
                 << ", curr_obs_expect_towing_l = " << curr_obs_expect_towing_l;
          SetTowingPoints(static_obs_towing_prepare_distance, final_obs_edges,
                          curr_obs_edge.obstacle_id, reference_line_info,
                          path_boundaries, towing_points,
                          static_obs_expect_towing_l, true);
          break;
        }
        if (absl::StrContains(decider_tag, "static-right-nudge")) {
          ADEBUG << "right: obs id = " << curr_obs_edge.obstacle_id
                 << " is prepared for static obstacle towing";
          double curr_obs_expect_towing_l =
              LookUpTowingLDistance(std::abs(curr_obs_edge.obstacle_edge_l_min),
                                    GetProcessBound()
                                        ->GetObsTowingConf()
                                        .towing_l_distance_segment_conf()
                                        .obs_to_center_line_dis_segment(),
                                    GetProcessBound()
                                        ->GetObsTowingConf()
                                        .towing_l_distance_segment_conf()
                                        .static_obs_expect_towing_segment_l());
          const double static_obs_expect_towing_l =
              FLAGS_use_dynamic_adjust_towing_l
                  ? curr_obs_first_exist
                        ? curr_obs_expect_towing_l
                        : std::max(curr_obs_expect_towing_l,
                                   last_obs->GetMaxExpectTowingL())
                  : GetProcessBound()
                        ->GetObsTowingConf()
                        .default_obstacle_towing_l();
          ADEBUG << "switch: " << FLAGS_use_dynamic_adjust_towing_l
                 << ", curr_obstacle_l_min = "
                 << curr_obs_edge.obstacle_edge_l_min
                 << ", static_obs_expect_towing_l = "
                 << static_obs_expect_towing_l
                 << ", curr_obs_expect_towing_l = " << curr_obs_expect_towing_l;
          SetTowingPoints(static_obs_towing_prepare_distance, final_obs_edges,
                          curr_obs_edge.obstacle_id, reference_line_info,
                          path_boundaries, towing_points,
                          static_obs_expect_towing_l, false);
          break;
        }
      }
      ++curr_obs_idx;
    }
  }

  // 7.trim path bound and towing line while block
  int trim_path_bound_idx = -1;
  bool is_min_diff_bounds =
      IsMinDiffLeftAndRightBounds(path_boundaries, &trim_path_bound_idx);
  ADEBUG << "min diff left bound and right bound is [ " << std::boolalpha
         << is_min_diff_bounds
         << " ] less than K, min diff bound index: " << FIXED << SETPRECISION(3)
         << trim_path_bound_idx << " path_blocked_idx: " << path_blocked_idx;

  ProcessBound::TrimPathBounds(trim_path_bound_idx, path_boundaries);
  ProcessBound::TrimTowingPoints(trim_path_bound_idx, towing_points);

  if (FLAGS_enable_path_bound_debug) {
    GetProcessBound()
        ->GetMutableReferenceLineInfo()
        ->mutable_debug()
        ->mutable_planning_data()
        ->add_path()
        ->mutable_path_bound_debug()
        ->mutable_boundary_from_static_obstacles()
        ->set_obstacles_number(sorted_obstacles.size());
  }

  double end_time = common::Clock::NowInMicroseconds();
  ADEBUG << "static obstacle elapsed time = " << end_time - start_time;
  return true;
}

// The tuple contains (is_start_s, s, l_min, l_max, obstacle_id)
void ObsStaticProcessor::SortObstaclesForSweepLine(
    const ReferenceLineInfo& reference_line_info,
    const IndexedList<std::string, Obstacle>& indexed_obstacles) {
  obstacle_edges_.clear();
  need_nudge_obstacle_edges_.clear();
  const double kExpandStopStaticCarbuffer = 0.3;
  const double kCrossLineTolerance = 0.3;
  // Obs laterral buffer process
  double init_obs_lateral_buffer = 0.0;
  if (GetConfig()
          .path_bounds_decider_config()
          .obstacle_buffer_process_config()
          .enable_static_obs_dynamic_buffer_calculate()) {
    if (!GetInjector()->ego_info()->vehicle_state().has_linear_velocity()) {
      AERROR << "dynamic obs buffer err, can't has velocity.";
      init_obs_lateral_buffer = GetConfig()
                                    .path_bounds_decider_config()
                                    .obstacle_buffer_process_config()
                                    .obs_static_lat_buffer();
    } else {
      init_obs_lateral_buffer =
          static_obs_dynamic_buffer_calculate_.StaticObsDynamicBufferCalculate(
              GetInjector()->ego_info()->vehicle_state().linear_velocity());
    }
  } else {
    init_obs_lateral_buffer = GetConfig()
                                  .path_bounds_decider_config()
                                  .obstacle_buffer_process_config()
                                  .obs_static_lat_buffer();
  }

  // Go through every obstacle and preprocess it.
  for (const auto* const obstacle : indexed_obstacles.Items()) {
    bool is_towing_obstacle = false;

    // filter obstacle
    if (obstacle == nullptr || !IsConsiderStaticObstacle(obstacle)) {
      continue;
    }

    const auto& perception_sl_boundary = obstacle->PerceptionSLBoundary();
    double lane_left_width = 0.0;
    double lane_right_width = 0.0;
    reference_line_info.reference_line().GetLaneWidth(
        (perception_sl_boundary.start_s() + perception_sl_boundary.end_s()) *
            0.5,
        &lane_left_width, &lane_right_width);
    const auto is_not_on_lane =
        (perception_sl_boundary.start_l() - kCrossLineTolerance) >
            lane_left_width ||
        (perception_sl_boundary.end_l() + kCrossLineTolerance) <
            -lane_right_width;

    // cone expand less
    const auto obs_lateral_buffer = (obstacle->IsCone() || is_not_on_lane)
                                        ? init_obs_lateral_buffer * 0.5
                                        : init_obs_lateral_buffer;

    double block_obs_lat_buffer =
        obs_lateral_buffer + kExpandStopStaticCarbuffer;
    double curr_obs_lat_buffer = obs_lateral_buffer;
    bool is_stop_obstacle = false;
    if (GetInjector() == nullptr || GetInjector()->frame_history() == nullptr ||
        GetInjector()->frame_history()->Latest() == nullptr ||
        GetInjector()->frame_history()->Latest()->DriveReferenceLineInfo() ==
            nullptr) {
      is_stop_obstacle = false;
    } else {
      const auto* last_obstacle = GetInjector()
                                      ->frame_history()
                                      ->Latest()
                                      ->DriveReferenceLineInfo()
                                      ->path_decision()
                                      .Find(obstacle->Id());
      is_stop_obstacle = last_obstacle != nullptr &&
                         last_obstacle->LongitudinalDecision().has_stop();
    }
    curr_obs_lat_buffer =
        is_stop_obstacle ? block_obs_lat_buffer : obs_lateral_buffer;

    const auto& obstacle_sl = obstacle->PerceptionSLBoundary();
    ObstacleEdge curr_obstacle_edges = {
        obstacle_sl.start_s() - GetConfig()
                                    .path_bounds_decider_config()
                                    .obstacle_buffer_process_config()
                                    .obs_static_lon_start_buffer(),
        obstacle_sl.end_s() + GetConfig()
                                  .path_bounds_decider_config()
                                  .obstacle_buffer_process_config()
                                  .obs_static_lon_end_buffer(),
        obstacle_sl.start_l() - curr_obs_lat_buffer,
        obstacle_sl.end_l() + curr_obs_lat_buffer,
        obstacle->Id(),
        obstacle};
    obstacle_edges_.emplace_back(curr_obstacle_edges);
    // obstacles has decison
    for (const auto& decider_tag : obstacle->decider_tags()) {
      if (absl::StrContains(decider_tag, "static")) {
        need_nudge_obstacle_edges_.emplace_back(curr_obstacle_edges);
        is_towing_obstacle = true;
        if ((obstacle_sl.start_s() - GetProcessBound()->GetAdcFrenetS()) >
            GetProcessBound()
                ->GetObsTowingConf()
                .longitude_short_distance_static_obs_threshold()) {
          GetProcessBound()
              ->GetMutableReferenceLineInfo()
              ->path_decision()
              ->Find(obstacle->Id())
              ->SetForbidStaticObsNudgeDisplay(true);
        }
        break;
      }
    }
    if (!is_towing_obstacle) {
      GetProcessBound()
          ->GetMutableReferenceLineInfo()
          ->path_decision()
          ->Find(obstacle->Id())
          ->SetMaxExpectTowingL(0.0);
    }
  }

  // Sort for bound
  ForObstacleEdgesSort(&obstacle_edges_);
  // Sort for towing
  ForObstacleEdgesSort(&need_nudge_obstacle_edges_);
}

bool ObsStaticProcessor::IsConsiderStaticObstacle(
    const Obstacle* const obstacle) {
  if (obstacle == nullptr) {
    AERROR << "IsConsiderStaticObstacle nullptr check is failed!!!";
    return false;
  }
  // Only focus on those within-scope obstacles.
  if (!IsWithinPathDeciderScopeObstacle(*obstacle)) {
    return false;
  }
  // Ignore obstacle whose distance to reference line more than
  // obs_right_filter_distance or obs_left_filter_distance.
  if (common::math::double_type::DefinitelyLess(
          obstacle->PerceptionSLBoundary().end_l(),
          GetConfig()
              .path_bounds_decider_config()
              .dynamic_obs_process_config()
              .obs_right_filter_distance()) ||
      common::math::double_type::DefinitelyGreater(
          obstacle->PerceptionSLBoundary().start_l(),
          GetConfig()
              .path_bounds_decider_config()
              .dynamic_obs_process_config()
              .obs_left_filter_distance())) {
    ADEBUG << " Static Obs: " << obstacle->Id()
           << " isn't in lateral consider area";
    return false;
  }
  // Only focus on obstacles that are ahead of ADC.
  if ((common::math::double_type::DefinitelyLess(
           obstacle->PerceptionSLBoundary().end_s(),
           GetProcessBound()->GetAdcFrenetS()) &&
       !GetProcessBound()->GetReferenceLineInfo()->IsHistoryTrace()) ||
      (common::math::double_type::DefinitelyGreater(
           obstacle->PerceptionSLBoundary().start_s(),
           (GetProcessBound()->GetAdcFrenetS() +
            common::VehicleConfigHelper::GetConfig()
                .vehicle_param()
                .front_edge_to_center())) &&
       GetProcessBound()->GetReferenceLineInfo()->IsHistoryTrace())) {
    return false;
  }
  return true;
}

void ObsStaticProcessor::ForObstacleEdgesSort(
    std::vector<ObstacleEdge>* const prepare_to_sort_edges) {
  if (prepare_to_sort_edges == nullptr) {
    AERROR << "ForObsatcleEdgesSort nullptr check is failed!";
    return;
  }
  std::sort(
      prepare_to_sort_edges->begin(), prepare_to_sort_edges->end(),
      [](const ObstacleEdge& lhs, const ObstacleEdge& rhs) {
        if (Compare(lhs.obstacle_edge_start_s, rhs.obstacle_edge_start_s) !=
            0) {
          return Compare(lhs.obstacle_edge_start_s, rhs.obstacle_edge_start_s) <
                 0;
        }
        return Compare(lhs.obstacle_edge_end_s, rhs.obstacle_edge_end_s) < 0;
      });
}

void ObsStaticProcessor::SetStaticObstacleBoundDebugInfo(
    const ObstacleEdge& curr_obs_edge) {
  if (FLAGS_enable_path_bound_debug) {
    auto* const ref = GetProcessBound()
                          ->GetMutableReferenceLineInfo()
                          ->mutable_debug()
                          ->mutable_planning_data()
                          ->add_path()
                          ->mutable_path_bound_debug()
                          ->mutable_boundary_from_static_obstacles()
                          ->add_static_obstacle_info();
    ref->set_lower_s(curr_obs_edge.obstacle_edge_start_s);
    ref->set_l_min(curr_obs_edge.obstacle_edge_l_min);
    ref->set_l_max(curr_obs_edge.obstacle_edge_l_max);
    ref->set_upper_s(curr_obs_edge.obstacle_edge_end_s);
    ref->set_id(curr_obs_edge.obstacle_id);
  }
}

bool ObsStaticProcessor::CalculateStaticTowingPrepareDistance(
    const std::vector<ObstacleEdge>& sorted_obstacles,
    const std::pair<double, double>& curr_obs_edges,
    const std::pair<double, double>& default_towing_prepare_distance,
    std::pair<double, double>* const towing_prepare_distance,
    const size_t obs_idx) {

  if (towing_prepare_distance == nullptr) {
    AERROR << "CalculateStaticTowingPrepareDistance nullptr check is failed!!!";
    return false;
  }
  // only consider the case has multipul obstacles
  if (sorted_obstacles.size() > 1) {
    // the first obstacle
    if (obs_idx == 0) {
      if (common::math::double_type::DefinitelyGreaterEqual(
              curr_obs_edges.second,
              sorted_obstacles[obs_idx + 1].obstacle_edge_start_s)) {
        towing_prepare_distance->first = 0.0;
      } else {
        towing_prepare_distance->first =
            std::min(sorted_obstacles[obs_idx + 1].obstacle_edge_start_s -
                         curr_obs_edges.second,
                     default_towing_prepare_distance.first);
      }
    } else if (obs_idx == sorted_obstacles.size() - 1) {
      // the last obstacle
      if (common::math::double_type::DefinitelyLessEqual(
              curr_obs_edges.first,
              sorted_obstacles[obs_idx - 1].obstacle_edge_end_s)) {
        towing_prepare_distance->second = 0.0;
      } else {
        towing_prepare_distance->second =
            std::min(curr_obs_edges.first -
                         sorted_obstacles[obs_idx - 1].obstacle_edge_end_s,
                     default_towing_prepare_distance.second);
      }
    } else {
      // obs forward
      if (common::math::double_type::DefinitelyGreaterEqual(
              curr_obs_edges.second,
              sorted_obstacles[obs_idx + 1].obstacle_edge_start_s)) {
        towing_prepare_distance->first = 0.0;
      } else {
        towing_prepare_distance->first =
            std::min(sorted_obstacles[obs_idx + 1].obstacle_edge_start_s -
                         curr_obs_edges.second,
                     default_towing_prepare_distance.first);
      }
      // obs backward
      if (common::math::double_type::DefinitelyLessEqual(
              curr_obs_edges.first,
              sorted_obstacles[obs_idx - 1].obstacle_edge_end_s)) {
        towing_prepare_distance->second = 0.0;
      } else {
        towing_prepare_distance->second =
            std::min(curr_obs_edges.first -
                         sorted_obstacles[obs_idx - 1].obstacle_edge_end_s,
                     default_towing_prepare_distance.second);
      }
    }
  }
  ADEBUG << "prepare distance forward = " << towing_prepare_distance->first
         << ", prepare distance backward = " << towing_prepare_distance->second;
  return true;
}

bool ObsStaticProcessor::IsMinDiffLeftAndRightBounds(
    const PathBound* const path_boundaries, int* const min_diff_bounds_idx) {
  if ((path_boundaries == nullptr) || (min_diff_bounds_idx == nullptr)) {
    AERROR << "path_boundaries or min_diff_bounds_idx is nullptr!";
    return false;
  }

  int tmp_min_diff_bounds_idx = 0;
  double min_turn_radius =
      VehicleConfigHelper::GetConfig().vehicle_param().min_turn_radius();

  auto check_path_bound = [&](const PathBoundPoint& path_bound) {
    if (*min_diff_bounds_idx != -1 &&
        tmp_min_diff_bounds_idx > *min_diff_bounds_idx) {
      tmp_min_diff_bounds_idx = *min_diff_bounds_idx;
      return true;
    }
    if (tmp_min_diff_bounds_idx >
        static_cast<int>(round(FLAGS_max_stop_distance_obstacle /
                               GetProcessBound()
                                   ->GetReferenceLineInfo()
                                   ->PathBoundsDeciderResolution()))) {
      ++tmp_min_diff_bounds_idx;
      return false;
    }
    double adc_to_bound_min_lat_dis =
        std::fmin(std::get<2>(path_bound) - GetProcessBound()->GetAdcFrenetL(),
                  GetProcessBound()->GetAdcFrenetL() - std::get<1>(path_bound));

    adc_to_bound_min_lat_dis =
        std::fmin(std::fabs(adc_to_bound_min_lat_dis),
                  min_turn_radius - KAlmostZero1ENegtive5);
    double min_safe_distance =
        std::sqrt(std::fabs(min_turn_radius * min_turn_radius -
                            (min_turn_radius - adc_to_bound_min_lat_dis) *
                                (min_turn_radius - adc_to_bound_min_lat_dis))) +
        GetConfig()
            .path_bounds_decider_config()
            .obstacle_buffer_process_config()
            .stop_distance_buffer();
    min_safe_distance = TL::common::math::Clamp(
        min_safe_distance, FLAGS_max_stop_distance_obstacle,
        FLAGS_min_stop_distance_obstacle);
    double delta_s =
        GetProcessBound()->GetReferenceLineInfo()->IsHistoryTrace()
            ? -std::get<0>(path_bound) + GetProcessBound()->GetAdcFrenetS()
            : std::get<0>(path_bound) - GetProcessBound()->GetAdcFrenetS();

    if (std::get<2>(path_bound) - std::get<1>(path_bound) <
            GetConfig()
                .path_bounds_decider_config()
                .obstacle_buffer_process_config()
                .min_diff_left_and_right_bounds() &&
        delta_s < min_safe_distance &&
        adc_to_bound_min_lat_dis >
            std::fabs(GetProcessBound()->GetStartPointDl() * delta_s) +
                GetConfig()
                    .path_bounds_decider_config()
                    .obstacle_buffer_process_config()
                    .min_lat_dis_adc_and_bound_buffer()) {
      return true;
    }
    ++tmp_min_diff_bounds_idx;
    return false;
  };

  if (std::find_if(path_boundaries->begin(), path_boundaries->end(),
                   check_path_bound) != path_boundaries->end()) {
    *min_diff_bounds_idx = tmp_min_diff_bounds_idx;
    return true;
  }
  *min_diff_bounds_idx = static_cast<int>(path_boundaries->size());
  return false;
}

bool ObsStaticProcessor::UpdatePathBoundaryAndCenterLineWithBuffer(
    size_t idx, const double left_bound, const double right_bound,
    PathBound* const path_boundaries, double* const center_line,
    const ReferenceLineInfo& reference_line_info) {
  if (path_boundaries == nullptr || center_line == nullptr) {
    AERROR << "path_boundaries or center_line is nullptr!";
    return false;
  }

  if (!GetProcessBound()->UpdateObstaclePathBoundaryWithBuffer(
          idx, left_bound, right_bound, path_boundaries)) {
    *center_line = (std::get<1>((*path_boundaries)[idx]) +
                    std::get<2>((*path_boundaries)[idx])) /
                   2;
    ADEBUG << "Update Path Boundary With Buffer Failed";
    return false;
  }
  *center_line = (std::get<1>((*path_boundaries)[idx]) +
                  std::get<2>((*path_boundaries)[idx])) /
                 2;
  return true;
}

bool ObsStaticProcessor::GetStaticObstacleBound(
    const ReferencePoint& reference_point, const Obstacle& obs,
    std::pair<double, double>* const obs_path_bound,
    const std::pair<Vec2d, bool>& kinematic_corner) {
  if (obs_path_bound == nullptr) {
    AERROR << "GetStaticObstacleBound nullptr check is failed!!!";
    return false;
  }
  const auto& obs_corners = obs.PerceptionBoundingBox().GetAllCorners();
  std::vector<Vec2d> kinematic_corners;
  kinematic_corners.reserve(obs_corners.size() + 1);
  kinematic_corners = obs_corners;
  if (kinematic_corner.second) {
    kinematic_corners.emplace_back(kinematic_corner.first);
  }
  std::vector<double> cross_points;
  cross_points.reserve(kinematic_corners.size() * kinematic_corners.size());
  const double cosh = cos(reference_point.heading());
  const double sinh = sin(reference_point.heading());
  std::vector<Vec2d> corners_local_flu;
  corners_local_flu.reserve(kinematic_corners.size());
  for (auto corner : kinematic_corners) {
    const double x_i = (corner.x() - reference_point.x()) * cosh +
                       (corner.y() - reference_point.y()) * sinh;
    const double y_i = -(corner.x() - reference_point.x()) * sinh +
                       (corner.y() - reference_point.y()) * cosh;
    corners_local_flu.emplace_back(x_i, y_i);
  }
  for (int i = 0; i < kinematic_corners.size(); ++i) {
    for (int j = 0; j < kinematic_corners.size(); ++j) {
      if (i == j) {
        continue;
      }
      if ((common::math::double_type::DefinitelyGreater(
               corners_local_flu[i].x(), 0.0) &&
           common::math::double_type::DefinitelyLessEqual(
               corners_local_flu[j].x(), 0.0)) ||
          (common::math::double_type::DefinitelyLessEqual(
               corners_local_flu[i].x(), 0.0) &&
           common::math::double_type::DefinitelyGreater(
               corners_local_flu[j].x(), 0.0))) {
        if (common::math::double_type::SeemsEqual(corners_local_flu[i].x(),
                                                  corners_local_flu[j].x())) {
          cross_points.emplace_back(corners_local_flu[i].y());
          cross_points.emplace_back(corners_local_flu[j].y());
        } else {
          const double y =
              corners_local_flu[j].y() * (corners_local_flu[i].x()) /
                  (corners_local_flu[i].x() - corners_local_flu[j].x()) -
              corners_local_flu[i].y() * (corners_local_flu[j].x()) /
                  (corners_local_flu[i].x() - corners_local_flu[j].x());
          cross_points.emplace_back(y);
        }
      }
    }
  }
  if (cross_points.empty()) {
    *obs_path_bound = std::make_pair(std::numeric_limits<double>::max(),
                                     std::numeric_limits<double>::lowest());
    return false;
  }
  *obs_path_bound = std::make_pair(
      *std::min_element(cross_points.cbegin(), cross_points.cend()) -
          GetConfig()
              .path_bounds_decider_config()
              .obstacle_buffer_process_config()
              .obs_static_lat_buffer(),
      *std::max_element(cross_points.cbegin(), cross_points.cend()) +
          GetConfig()
              .path_bounds_decider_config()
              .obstacle_buffer_process_config()
              .obs_static_lat_buffer());
  return true;
}

bool ObsStaticProcessor::GetObstacleKinematicCorner(
    std::vector<ObstacleEdge>* const sorted_obstacles,
    const IndexedObstacles& indexed_obstacles,
    std::unordered_map<std::string, std::pair<Vec2d, bool>>* const
        obs_kinematic_corner) {
  if (sorted_obstacles == nullptr || obs_kinematic_corner == nullptr) {
    AERROR << "GetObstacleKinematicCorner nullptr check is failed!!!";
    return false;
  }
  for (auto& obs : *sorted_obstacles) {
    bool has_kinematic_corner = false;
    const auto* obs_ptr = indexed_obstacles.Find(obs.obstacle_id);

    // skip the no decision obstacles
    if (obs_ptr->decider_tags().empty()) {
      Vec2d kinematic_corner_global_none_point(0, 0);
      obs_kinematic_corner->insert(std::make_pair(
          obs.obstacle_id, std::make_pair(kinematic_corner_global_none_point,
                                          has_kinematic_corner)));
      continue;
    }

    // decide nudge side by decisions
    bool is_left_nudge = false;
    bool is_obs_need_nudge = false;
    for (const auto& decider_tag : obs_ptr->decider_tags()) {
      if (absl::StrContains(decider_tag, "static-left-nudge")) {
        is_left_nudge = true;
        is_obs_need_nudge = true;
      } else if (absl::StrContains(decider_tag, "static-right-nudge")) {
        is_left_nudge = false;
        is_obs_need_nudge = true;
      }
    }
    if (!is_obs_need_nudge) {
      Vec2d kinematic_corner_global_none_point(0, 0);
      obs_kinematic_corner->insert(std::make_pair(
          obs.obstacle_id, std::make_pair(kinematic_corner_global_none_point,
                                          has_kinematic_corner)));
      continue;
    }

    // calculate cross points
    auto obs_corners = obs_ptr->PerceptionBoundingBox().GetAllCorners();
    std::vector<std::tuple<double, Vec2d, Vec2d>> cross_points;
    cross_points.reserve(obs_corners.size());
    if (!CalculateCrossPoints(&obs_corners, &cross_points, is_left_nudge)) {
      AERROR << "Generate cross points failed!";
      continue;
    }

    if (cross_points.empty()) {
      Vec2d kinematic_corner_global_none_point(0, 0);
      obs_kinematic_corner->insert(std::make_pair(
          obs.obstacle_id, std::make_pair(kinematic_corner_global_none_point,
                                          has_kinematic_corner)));
      // AERROR << "Obs don't has kinematic corner, id: " << std::get<4>(obs)
      //        << ", vehicle heading: " << vehicle_state.heading();
      continue;
    }

    // determine the final kinematic corner point
    has_kinematic_corner = true;
    Vec2d kinematic_corner_global_point(0, 0);
    if (!SetKinematicCornerGlobalPoint(
            cross_points, &kinematic_corner_global_point, is_left_nudge)) {
      AERROR << "set kinematic corner global point failed!";
      continue;
    }
    obs_kinematic_corner->insert(std::make_pair(
        obs.obstacle_id,
        std::make_pair(kinematic_corner_global_point, has_kinematic_corner)));
    ADEBUG << "Obs has kinematic corner, id: " << obs.obstacle_id
           << ", x: " << obs_kinematic_corner->at(obs.obstacle_id).first.x()
           << ", y: " << obs_kinematic_corner->at(obs.obstacle_id).first.y();
    // the start(or end in TBA) s of the obstacle should be the projection of
    // kinematic corner(if has_kinematic_corner)
    if (has_kinematic_corner) {
      common::SLPoint kinematic_corner_sl;
      GetProcessBound()->GetReferenceLineInfo()->reference_line().XYToSL(
          kinematic_corner_global_point, &kinematic_corner_sl);
      if (GetProcessBound()->GetReferenceLineInfo()->IsHistoryTrace()) {
        obs.obstacle_edge_end_s =
            fmax(obs.obstacle_edge_end_s, kinematic_corner_sl.s());
      } else {
        obs.obstacle_edge_start_s =
            fmin(obs.obstacle_edge_start_s, kinematic_corner_sl.s());
      }
    }
  }
  return true;
}

bool ObsStaticProcessor::CalculateCrossPoints(
    std::vector<Vec2d>* const obs_corners,
    std::vector<std::tuple<double, Vec2d, Vec2d>>* const cross_points,
    const bool is_left_nudge) {
  if (obs_corners == nullptr || cross_points == nullptr) {
    AERROR << "CalculateCrossPoints nullptr check is failed!!!";
    return false;
  }
  std::vector<Vec2d> corners_local_flu;
  corners_local_flu.reserve(obs_corners->size());
  double half_width =
      VehicleConfigHelper::GetConfig().vehicle_param().width() / 2;
  double flu_translation = is_left_nudge ? half_width : -half_width;
  const auto& vehicle_state =
      GetProcessBound()->GetReferenceLineInfo()->vehicle_state();
  const double cosh = cos(vehicle_state.heading());
  const double sinh = sin(vehicle_state.heading());
  for (auto corner : *obs_corners) {
    const double x_i = (corner.x() - vehicle_state.x()) * cosh +
                       (corner.y() - vehicle_state.y()) * sinh;
    const double y_i = -(corner.x() - vehicle_state.x()) * sinh +
                       (corner.y() - vehicle_state.y()) * cosh;
    corners_local_flu.emplace_back(x_i, y_i + flu_translation);
  }

  for (int i = 0; i < obs_corners->size(); ++i) {
    for (int j = 0; j < obs_corners->size(); ++j) {
      if (i == j) {
        continue;
      }
      if ((TL::common::math::double_type::DefinitelyGreaterEqual(
               corners_local_flu[i].y(), 0.0) &&
           TL::common::math::double_type::DefinitelyLessEqual(
               corners_local_flu[j].y(), 0.0)) ||
          (TL::common::math::double_type::DefinitelyLessEqual(
               corners_local_flu[i].y(), 0.0) &&
           TL::common::math::double_type::DefinitelyGreaterEqual(
               corners_local_flu[j].y(), 0.0))) {
        if (common::math::double_type::SeemsEqual(corners_local_flu[i].y(),
                                                  corners_local_flu[j].y())) {
          cross_points->emplace_back(
              corners_local_flu[i].x(),
              Vec2d(corners_local_flu[i].x(), corners_local_flu[i].y()),
              Vec2d(corners_local_flu[j].x(), corners_local_flu[j].y()));
          cross_points->emplace_back(
              corners_local_flu[j].x(),
              Vec2d(corners_local_flu[i].x(), corners_local_flu[i].y()),
              Vec2d(corners_local_flu[j].x(), corners_local_flu[j].y()));
        } else {
          double cross_point_x = 0.0;
          double x_a = 0.0;
          double y_a = 0.0;
          double x_b = 0.0;
          double y_b = 0.0;
          if (TL::common::math::double_type::DefinitelyLessEqual(
                  std::fabs(corners_local_flu[i].x()),
                  std::fabs(corners_local_flu[j].x()))) {
            x_a = std::fabs(corners_local_flu[i].x());
            y_a = std::fabs(corners_local_flu[i].y());
            x_b = std::fabs(corners_local_flu[j].x());
            y_b = std::fabs(corners_local_flu[j].y());
          } else {
            x_a = std::fabs(corners_local_flu[j].x());
            y_a = std::fabs(corners_local_flu[j].y());
            x_b = std::fabs(corners_local_flu[i].x());
            y_b = std::fabs(corners_local_flu[i].y());
          }
          cross_point_x = y_a * (x_b - x_a) / (y_a + y_b) + x_a;

          cross_points->emplace_back(
              cross_point_x,
              Vec2d(corners_local_flu[i].x(), corners_local_flu[i].y()),
              Vec2d(corners_local_flu[j].x(), corners_local_flu[j].y()));
          // AERROR << "cross_point_x: " << cross_point_x
          //        << ", vehicle heading: " << vehicle_state.heading();
        }
      }
    }
  }
  return true;
}

bool ObsStaticProcessor::SetKinematicCornerGlobalPoint(
    const std::vector<std::tuple<double, Vec2d, Vec2d>>& cross_points,
    Vec2d* const kinematic_corner_global_point, const bool is_left_nudge) {
  if (kinematic_corner_global_point == nullptr) {
    AERROR << "SetKinematicCornerGlobalPoint nullptr check is failed!!!";
    return false;
  }
  const auto& vehicle_state =
      GetProcessBound()->GetReferenceLineInfo()->vehicle_state();
  const double cosh = cos(vehicle_state.heading());
  const double sinh = sin(vehicle_state.heading());
  double half_width =
      VehicleConfigHelper::GetConfig().vehicle_param().width() / 2;
  double flu_translation = is_left_nudge ? half_width : -half_width;
  std::tuple<double, Vec2d, Vec2d> min_distance_point;
  min_distance_point =
      *std::min_element(cross_points.cbegin(), cross_points.cend(), TupleComp);
  // +: left; -: right. after swap: 1: +; 2: - .
  if (std::get<1>(min_distance_point).y() <= 0 &&
      std::get<2>(min_distance_point).y() >= 0) {
    min_distance_point = {std::get<0>(min_distance_point),
                          std::get<2>(min_distance_point),
                          std::get<1>(min_distance_point)};
  }
  double min_distance_point_y = is_left_nudge
                                    ? std::get<1>(min_distance_point).y()
                                    : std::get<2>(min_distance_point).y();
  double min_distance_point_x = is_left_nudge
                                    ? std::get<1>(min_distance_point).x()
                                    : std::get<2>(min_distance_point).x();
  double safe_distance =
      std::sqrt(std::pow(VehicleConfigHelper::MinSafeTurnRadius(), 2) -
                std::pow((VehicleConfigHelper::MinSafeTurnRadius() -
                          fabs(min_distance_point_y)),
                         2));
  double kinematic_corner_flu_x =
      GetProcessBound()->GetReferenceLineInfo()->IsHistoryTrace()
          ? min_distance_point_x + safe_distance
          : min_distance_point_x - safe_distance;
  kinematic_corner_global_point->set_x(vehicle_state.x() +
                                       kinematic_corner_flu_x * cosh +
                                       flu_translation * sinh);
  kinematic_corner_global_point->set_y(vehicle_state.y() +
                                       kinematic_corner_flu_x * sinh -
                                       flu_translation * cosh);
  return true;
}

bool ObsStaticProcessor::TupleComp(const std::tuple<double, Vec2d, Vec2d>& a,
                                   const std::tuple<double, Vec2d, Vec2d>& b) {
  return std::get<0>(a) < std::get<0>(b);
}
}  // namespace planning
}  // namespace TL
