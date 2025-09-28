/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  planning obstacle dynamic processor
 */

#include "planning/tasks/deciders/path_bounds_decider/obs_processor/obs_dynamic_processor.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <functional>
#include <limits>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "absl/strings/match.h"
#include "common/file/log.h"
#include "common/math/double_type.h"
#include "common/math/linear_interpolation.h"
#include "common/time/clock.h"
#include "common/util/macros.h"
#include "glog/logging.h"
#include "planning/common/obstacle.h"
#include "planning/common/planning_gflags.h"
#include "planning/common/reference_line_info.h"
#include "planning/math/curve1d/quartic_polynomial_curve1d.h"
#include "planning/tasks/deciders/path_bounds_decider/util/path_info.h"

#define ENBLE_ACCURATE_SLBOUNDARY false
#define DYNAMIC_OBSTACLE_CONSTRAINT_TRIM false

namespace TL {
namespace planning {
using common::math::double_type::Compare;
using common::math::double_type::ComparedToZero;
using common::math::double_type::IsZero;
using TL::common::SLPoint;
using TL::common::VehicleConfigHelper;
using TL::common::math::Clamp;
using TL::common::math::Vec2d;

namespace {

bool MockLonTrajectory1d(const common::TrajectoryPoint* start_point,
                         const double max_speed, SpeedData* const speed_data) {
  if ((start_point == nullptr) || (speed_data == nullptr)) {
    return false;
  }

  std::array<double, 3> start_state = {0.0, start_point->v(), start_point->a()};
  std::array<double, 2> end_state = {0.0};
  end_state[0] = Clamp(start_point->v(), 5.0, max_speed);

  QuarticPolynomialCurve1d lon_trajectory1d(start_state, end_state,
                                            FLAGS_trajectory_time_length);
  common::SpeedPoint tmp_speed_point;
  double time = 0.0;
  while (time < FLAGS_trajectory_time_length) {
    tmp_speed_point.set_t(time);
    tmp_speed_point.set_s(lon_trajectory1d.Evaluate(0, time));
    tmp_speed_point.set_v(lon_trajectory1d.Evaluate(1, time));
    tmp_speed_point.set_a(lon_trajectory1d.Evaluate(2, time));
    tmp_speed_point.set_da(lon_trajectory1d.Evaluate(3, time));
    speed_data->emplace_back(tmp_speed_point);
    time += FLAGS_speed_planning_delta_time;
  }
  return true;
}

SLBoundary RoughObstacleSLBoundary(const ObsPointDescription& obs_point,
                                   const double obs_start_l,
                                   const double obs_end_l,
                                   const double lon_extend_length = 0.0) {
  double s_min = fmin(obs_point.low_left_p.s(), obs_point.low_right_p.s());
  s_min = fmin(obs_point.upper_left_p.s(), s_min);
  s_min = fmin(obs_point.upper_right_p.s(), s_min) - lon_extend_length;

  double s_max = fmax(obs_point.low_left_p.s(), obs_point.low_right_p.s());
  s_max = fmax(obs_point.upper_left_p.s(), s_max);
  s_max = fmax(obs_point.upper_right_p.s(), s_max) + lon_extend_length;

  SLBoundary sl_boundary;
  sl_boundary.set_end_l(obs_end_l);
  sl_boundary.set_start_l(obs_start_l);
  sl_boundary.set_end_s(s_max);
  sl_boundary.set_start_s(s_min);
  return sl_boundary;
}

bool BothHaveOverlap(const double obs_start_s, const double obs_end_s,
                     const double start_point_s, const double lower_bound,
                     const double upper_bound) {
  return lower_bound <= obs_end_s - start_point_s &&
         upper_bound >= obs_start_s - start_point_s;
}

}  // namespace

ObsDynamicProcessor::ObsDynamicProcessor(
    const std::shared_ptr<DependencyInjector>& injector,
    const TaskConfig& config)
    : ObsProcessor(injector, config) {
  CHECK_NOTNULL(GetInjector());
}

bool ObsDynamicProcessor::Process(ReferenceLineInfo* const reference_line_info,
                                  Frame* const frame,
                                  PathBound* const path_boundaries,
                                  std::string* const blocking_obstacle_id,
                                  TowingPointsInfo* const towing_points,
                                  const bool is_enable_towing_process) {
  if (reference_line_info == nullptr || frame == nullptr ||
      path_boundaries == nullptr || blocking_obstacle_id == nullptr ||
      towing_points == nullptr) {
    AERROR << "Obs dynamic processor init failed";
    return false;
  }

  // bound process init.
  GetProcessBound()->InitPathBounds(frame, reference_line_info);

  // PERF_BLOCK_START();
  if (!FLAGS_enable_big_car_dynamic_obstacle_process ||
      GetProcessBound()->GetReferenceLineInfo()->IsChangeLanePath() ||
      frame->PlanningStartPoint().v() < FLAGS_lane_borrow_max_speed) {
    if (frame->PlanningStartPoint().v() < FLAGS_lane_borrow_max_speed) {
      ADEBUG << "it is useless to construct dynamic obstacles' constraint if "
                "adc's "
                "speed is less than "
             << FLAGS_lane_borrow_max_speed << " m/s.";
    }
    return true;
  }

  if (!Init()) {
    AERROR << "dynamic obstacle Init failed!";
    return false;
  }
  double start_time = common::Clock::NowInMicroseconds();
  // 1. sort dynamic obstacles by using the obstacle sl boundary.
  std::vector<ObstacleEdge> dynamic_obs_edges;
  SortDynamicObstaclesFromSweepLine(
      GetProcessBound()->GetReferenceLineInfo()->path_decision().obstacles(),
      path_boundaries, &dynamic_obs_edges, towing_points);

  if (FLAGS_enable_path_bound_debug) {
    DebugInfo(&dynamic_obs_edges, blocking_obstacle_id, path_boundaries,
              path_boundaries);
  }

  if (!is_enable_towing_process) {
    towing_points->clear();
  }

  if (!is_avoid_big_car_) {
    return true;
  }

  std::string tmp_blocking_obstacle_id = *blocking_obstacle_id;

  // 2. construct boundary using dynamic obstacles constraint.

  if (!ConstructBoundaryFromDynamicObstacles(
          &dynamic_obs_edges, path_boundaries, blocking_obstacle_id)) {
    AERROR << "it is an error to construct boundary from dynamic obstacles. ";
    return false;
  }
  double end_time = common::Clock::NowInMicroseconds();
  ADEBUG << "dynamic obstacle elapsed time = " << end_time - start_time;
// now, it is not to consider dynamic obstacles block path
#if !DYNAMIC_OBSTACLE_CONSTRAINT_TRIM
  *blocking_obstacle_id = tmp_blocking_obstacle_id;
#endif
  return true;
}

void ObsDynamicProcessor::DynamicObsInit(const double ADC_buffer) {
  ADC_buffer_ = ADC_buffer;
}

bool ObsDynamicProcessor::Init() {
  if (GetProcessBound()->GetFrame() == nullptr || GetInjector() == nullptr) {
    AERROR << "Init nullptr check is failed!";
    return false;
  }

  is_avoid_big_car_ = false;
  big_car_ids_.clear();
  const auto& init_trajectory_point =
      GetProcessBound()->GetFrame()->PlanningStartPoint();
  double start_point_v = 0.0;
  if (init_trajectory_point.has_v()) {
    start_point_v = init_trajectory_point.v();
  }
  extend_length_ =
      Clamp(start_point_v * kExtendTime, kMinExtendLength, kMaxExtendLength);
  return true;
}

void ObsDynamicProcessor::SortDynamicObstaclesFromSweepLine(
    const IndexedList<std::string, Obstacle>& indexed_obstacle,
    PathBound* const path_boundaries,
    std::vector<ObstacleEdge>* const dynamic_obstacles_edges,
    TowingPointsInfo* const towing_points) {
  // PERF_FUNCTION_WITH_NAME("SortDynamicObstaclesFromSweepLine.");
  if (path_boundaries == nullptr || dynamic_obstacles_edges == nullptr ||
      towing_points == nullptr) {
    AERROR << "SortDynamicObstaclesFromSweepLine nullptr check is failed!";
    return;
  }

  if (indexed_obstacle.Items().empty()) {
    ADEBUG << "no obstacle to construct boundary.";
    return;
  }

  // 1. set up sample knots' info according to previous speed data.
  SpeedKnotInfo speed_knots_info;
  if (!GetSampleKnots(path_boundaries, &speed_knots_info)) {
    AWARN << "speed sample error. ";
    return;
  }

  // 2. sort considering obstacle.
  ADEBUG << " dynamic obstacles size: " << indexed_obstacle.Items().size();
  std::vector<ObstacleEdge> dynamic_obs_edge_process_towing_line;

  bool is_virtual_lane = GetProcessBound()->CheckIfInJunction();
  for (const auto* obstacle : indexed_obstacle.Items()) {
    // filter obstacle.
    if (obstacle != nullptr && CheckIfNudgeObstacle(*obstacle)) {
      is_avoid_big_car_ = true;
      // get obstacle single sl boundary.
      std::vector<SLBoundaryInfo> obs_sl_boundary;
      if (is_virtual_lane) {
        // 路口处取障碍物1s时刻的位置作bound处理
        const double kEstimationTime = 1.0;
        static constexpr double kDynamicObsExtendLonLength = 0.50;
        const auto& obstacle_trajectory_envelope =
            obstacle->GetTrajectoryEnvelope();
        auto obs_point_description_iter = obstacle_trajectory_envelope.begin();
        auto obstacle_sl_boundary_ptr_point = std::make_pair(
            obstacle_trajectory_envelope.end(), ObsPointDescription());
        GetObstacleSLBoundaryPointAtTime(
            kEstimationTime, obs_point_description_iter,
            obstacle_trajectory_envelope, &obstacle_sl_boundary_ptr_point);
        SLBoundary rough_sl_boundary =
            RoughObstacleSLBoundary(obstacle_sl_boundary_ptr_point.second,
                                    obstacle->PerceptionSLBoundary().start_l(),
                                    obstacle->PerceptionSLBoundary().end_l(),
                                    kDynamicObsExtendLonLength);
        obs_sl_boundary.emplace_back(rough_sl_boundary, true, obstacle->Id());
      } else {
        CalculateSLBoundary(speed_knots_info, *obstacle, &obs_sl_boundary);
      }
      // get obstacle edge
      CalculateObstacleEdges(obs_sl_boundary, *obstacle,
                             dynamic_obstacles_edges,
                             &dynamic_obs_edge_process_towing_line);
    }
  }

  ADEBUG << "check dynamic_obstacles_edges size = "
         << dynamic_obstacles_edges->size()
         << ", check dynamic_obs_edge_process_towing_line size = "
         << dynamic_obs_edge_process_towing_line.size();

  // 3. sort dynamic obstacles by distance of first frame to vehicle.
  std::sort(
      dynamic_obstacles_edges->begin(), dynamic_obstacles_edges->end(),
      [](const ObstacleEdge& lhs, const ObstacleEdge& rhs) {
        if (Compare(lhs.obstacle_edge_start_s, rhs.obstacle_edge_start_s) !=
            0) {
          return Compare(lhs.obstacle_edge_start_s, rhs.obstacle_edge_start_s) <
                 0;
        }
        return Compare(lhs.obstacle_edge_end_s, rhs.obstacle_edge_end_s) < 0;
      });

  // 4. Calculate towing line
  ProcessObstacleEdgesForSetTowingPoints(dynamic_obs_edge_process_towing_line,
                                         path_boundaries, towing_points);
}

bool ObsDynamicProcessor::GetSampleKnots(
    const PathBound* const path_boundaries,
    SpeedKnotInfo* const speed_knots_info) {
  if (path_boundaries == nullptr || speed_knots_info == nullptr) {
    AERROR << "GetSampleKnots nullptr check is failed!";
    return false;
  }

  // get speed data from the last frame history.
  SpeedData speed_data;
  if (!InitSpeedData(&speed_data)) {
    AINFO << "InitSpeedData failed!";
    return false;
  }
  if (speed_data.empty()) {
    AERROR << "No speed data to process!";
    return false;
  }

  const auto& init_path_point =
      GetProcessBound()->GetFrame()->PlanningStartPoint().path_point();
  Vec2d current_init_point_cartesian{init_path_point.x(), init_path_point.y()};
  if (!GetProcessBound()->GetReferenceLineInfo()->reference_line().XYToSL(
          current_init_point_cartesian, &planning_start_point_sl_)) {
    return false;
  }
  double optimize_length = std::min(
      GetProcessBound()->GetReferenceLineInfo()->reference_line().Length() -
          planning_start_point_sl_.s(),
      (static_cast<int>(path_boundaries->size()) - 1) *
          GetProcessBound()
              ->GetReferenceLineInfo()
              ->PathBoundsDeciderResolution());
  auto num_interval = static_cast<size_t>(speed_data.TotalTime() /
                                          FLAGS_speed_planning_delta_time);
  double time = 0.0;
  common::SpeedPoint path_matched_speed_point;
  for (size_t i = 0; i <= num_interval; ++i) {
    time = std::min(speed_data.TotalTime(), time);
    speed_data.EvaluateByTime(time, &path_matched_speed_point);
    if (path_matched_speed_point.s() > optimize_length && i > 0) {
      break;
    }
    speed_knots_info->emplace_back(time, path_matched_speed_point.s());
    time += FLAGS_speed_planning_delta_time;
  }

  return true;
}

bool ObsDynamicProcessor::InitSpeedData(SpeedData* const speed_data) {
  if (speed_data == nullptr || GetProcessBound()->GetFrame() == nullptr ||
      GetInjector() == nullptr ||
      GetProcessBound()->GetReferenceLineInfo() == nullptr ||
      GetInjector()->frame_history() == nullptr) {
    AERROR << "InitSpeedData nullptr check is failed!";
    return false;
  }

  const auto& planning_start_point =
      GetProcessBound()->GetFrame()->PlanningStartPoint();
  const auto& last_frame = GetInjector()->frame_history()->Latest();
  if ((last_frame == nullptr) ||
      !last_frame->PlanningStartPoint().has_path_point() ||
      !planning_start_point.has_path_point() ||
      (last_frame->DriveReferenceLineInfo() == nullptr)) {
    return false;
  }
  const auto& last_speed_data =
      last_frame->DriveReferenceLineInfo()->speed_data();
  // if last speed data is empty, mock the speed data.
  if (last_speed_data.empty()) {
    return MockLonTrajectory1d(
        &last_frame->PlanningStartPoint(),
        GetProcessBound()->GetReferenceLineInfo()->GetCruiseSpeed(),
        speed_data);
  }
  Vec2d last_init_point_cartesian = {
      last_frame->PlanningStartPoint().path_point().x(),
      last_frame->PlanningStartPoint().path_point().y()};
  Vec2d current_init_point_cartesian = {planning_start_point.path_point().x(),
                                        planning_start_point.path_point().y()};
  SLPoint last_init_point_sl;
  SLPoint current_init_point_sl;
  const auto& reference_line =
      last_frame->DriveReferenceLineInfo()->reference_line();
  if (!reference_line.XYToSL(last_init_point_cartesian, &last_init_point_sl) ||
      !reference_line.XYToSL(current_init_point_cartesian,
                             &current_init_point_sl)) {
    return MockLonTrajectory1d(
        &last_frame->PlanningStartPoint(),
        GetProcessBound()->GetReferenceLineInfo()->GetCruiseSpeed(),
        speed_data);
  }

  double delta_s = current_init_point_sl.s() - last_init_point_sl.s();
  for (const auto& speed_point : last_speed_data) {
    if (speed_point.s() < delta_s) {
      continue;
    }
    speed_data->emplace_back(speed_point);
  }

  if (speed_data->empty()) {
    return MockLonTrajectory1d(
        &last_frame->PlanningStartPoint(),
        GetProcessBound()->GetReferenceLineInfo()->GetCruiseSpeed(),
        speed_data);
  }

  for (std::size_t i = 0; i < speed_data->size(); ++i) {
    auto& start_speed_point = speed_data->at(0);
    auto& speed_point = speed_data->at(i);
    speed_point.set_s(speed_point.s() - start_speed_point.s());
    speed_point.set_t(speed_point.t() - start_speed_point.t());
  }
  return true;
}

bool ObsDynamicProcessor::IsConsideringDynamicObstacle(
    const Obstacle* const obstacle, const double considering_length) {
  if (obstacle == nullptr) {
    AERROR << "IsConsideringDynamicObstacle empty check is failed!";
    return false;
  }
  // Obstacle should be non-virtual.
  if (obstacle->IsVirtual()) {
    return false;
  }
  // Obstacle should not have ignore decision.
  if (obstacle->HasLongitudinalDecision() && obstacle->HasLateralDecision() &&
      obstacle->IsIgnore()) {
    ADEBUG << obstacle->Id() << " is Ignore";
    return false;
  }
  // Obstacle should not be moving obstacle.
  if (obstacle->IsStatic() ||
      obstacle->speed() <= GetConfig()
                               .path_bounds_decider_config()
                               .dynamic_obs_process_config()
                               .obs_speed_filter_velocity()) {
    ADEBUG << obstacle->Id() << "Dynamic Obs velocity: " << obstacle->speed()
           << " is behind: "
           << GetConfig()
                  .path_bounds_decider_config()
                  .dynamic_obs_process_config()
                  .obs_speed_filter_velocity();
    return false;
  }
  // Ignore obstacle whose distance to reference line more than
  // kIgnoreObsRightLimit or kIgnoreObsLeftLimit m.
  if (obstacle->PerceptionSLBoundary().end_l() <
          GetConfig()
              .path_bounds_decider_config()
              .dynamic_obs_process_config()
              .obs_right_filter_distance() ||
      obstacle->PerceptionSLBoundary().start_l() >
          GetConfig()
              .path_bounds_decider_config()
              .dynamic_obs_process_config()
              .obs_left_filter_distance()) {
    ADEBUG << " Dynamic Obs: " << obstacle->Id()
           << " isn't in lateral consider area";
    return false;
  }
  // Only focus on obstacles that are ahead of ADC.
  if (obstacle->PerceptionSLBoundary().end_s() +
          GetConfig()
              .path_bounds_decider_config()
              .dynamic_obs_process_config()
              .obs_backward_filter_distance() <
      planning_start_point_sl_.s()) {
    ADEBUG << " Dynamic Obs id: " << obstacle->Id()
           << " isn't in s_back consider area";
    return false;
  }
  // Only focus on obstacle's distance to ADC less than path boundary length.
  double distance =
      obstacle->PerceptionSLBoundary().start_s() - planning_start_point_sl_.s();
  // double in_length = std::min(40.0, considering_length);
  if (considering_length < distance) {
    ADEBUG << "Dynamic Obs: " << obstacle->Id()
           << " isn't in s_front consider area.";
    return false;
  }
  // Only focus on obstacle whose area more than 1e-4
  if (obstacle->PerceptionBoundingBox().area() < kMinObstacleArea) {
    ADEBUG << "Dynamic Obs: " << obstacle->Id() << " area isn't enough";
    return false;
  }

  // Ignore obstacle that are ahead of ADC and within ADC on lane.
  if (!ObstacleIsWithinLane(&obstacle->PerceptionSLBoundary())) {
    ADEBUG << "Dynamic Obs: " << obstacle->Id() << " is not within lane";
    return false;
  }
  // Ignore the reverse obstacle from ADC
  if (!IsSameDirectionObstacleFromADC(obstacle)) {
    ADEBUG << "Dynamic Obs: " << obstacle->Id()
           << " isn't same direction from ADC";
    return false;
  }
  // Ignoring dynamic obstacle avoidance of roads with large curvature
  if (!IsObstacleInReasonableCurvatureRoad(obstacle)) {
    ADEBUG << "Dynamic Obs: " << obstacle->Id()
           << " isn't in reasonable curvature road";
    return false;
  }
  // big cart must have enough lateral distance to adc.
  if (!IsObstacleDeadAheadADCWithBuffer(obstacle)) {
    ADEBUG << "obstacle [id: " << obstacle->PerceptionId()
           << "] has no enough lateral distance to adc.";
    return false;
  }
  // only avoid obs TRUCK and BUS type vehicle
  if (obstacle->IsOversizedVehicle()) {
    ADEBUG << "Big Car id: " << obstacle->PerceptionId()
           << ", type: " << obstacle->Perception().type()
           << ", width: " << obstacle->PerceptionBoundingBox().width()
           << ", length: " << obstacle->PerceptionBoundingBox().length();
    is_avoid_big_car_ = true;
    big_car_ids_.emplace_back(obstacle->Id());
    return true;
  }
  // When only avoid big vehicle, ignore vehicle whose sum of width and length
  // is less than 9 m
  if ((FLAGS_enable_dynamic_vehicle_total_consider_length &&
       (obstacle->PerceptionBoundingBox().width() +
            obstacle->PerceptionBoundingBox().length() >=
        GetConfig()
            .path_bounds_decider_config()
            .dynamic_obs_process_config()
            .obs_big_car_filter_length())) ||
      obstacle->PerceptionBoundingBox().length() >=
          GetConfig()
              .path_bounds_decider_config()
              .dynamic_obs_process_config()
              .obs_big_car_filter_length()) {
    ADEBUG << "Big Car id: " << obstacle->PerceptionId()
           << ", type: " << obstacle->Perception().type()
           << ", width: " << obstacle->PerceptionBoundingBox().width()
           << ", length: " << obstacle->PerceptionBoundingBox().length();
    is_avoid_big_car_ = true;
    big_car_ids_.emplace_back(obstacle->Id());
    return true;
  }
  ADEBUG << obstacle->Id() << "other dynamic obstacle";
  return false;
}

bool ObsDynamicProcessor::ObstacleIsWithinLane(
    const SLBoundary* const sl_boundary) {
  if ((sl_boundary == nullptr) || !sl_boundary->has_end_l() ||
      !sl_boundary->has_start_l() || !sl_boundary->has_start_s() ||
      !sl_boundary->has_end_s() ||
      GetProcessBound()->GetReferenceLineInfo() == nullptr) {
    AERROR << "ObstacleIsWithinLane empty check is failed!";
    return false;
  }

  const double middle_s = (sl_boundary->start_s() + sl_boundary->end_s()) / 2;
  double lane_left_width =
      GetProcessBound()->GetLaneBoundConf().path_default_lane_width() / 2;
  double lane_right_width =
      GetProcessBound()->GetLaneBoundConf().path_default_lane_width() / 2;
  GetProcessBound()->GetReferenceLineInfo()->reference_line().GetLaneWidth(
      middle_s, &lane_left_width, &lane_right_width);

  return sl_boundary->start_l() > lane_left_width ||
         sl_boundary->end_l() < -lane_right_width ||
         sl_boundary->end_l() > lane_left_width ||
         sl_boundary->start_l() < -lane_right_width;
}

bool ObsDynamicProcessor::IsSameDirectionObstacleFromADC(
    const Obstacle* const obstacle) const {
  if ((obstacle == nullptr) || !obstacle->HasTrajectory()) {
    AERROR << "IsSameDirectionObstacleFromADC empty check is failed!";
    return false;
  }
  double obstacle_moving_direction =
      obstacle->Trajectory().trajectory_point(0).path_point().theta();
  const auto& vehicle_state =
      GetProcessBound()->GetReferenceLineInfo()->vehicle_state();
  double vehicle_moving_direction = vehicle_state.heading();
  if (vehicle_state.gear() == soc::Chassis::GEAR_REVERSE) {
    vehicle_moving_direction =
        common::math::NormalizeAngle(vehicle_moving_direction + M_PI);
  }
  double heading_difference = std::abs(common::math::NormalizeAngle(
      obstacle_moving_direction - vehicle_moving_direction));
  return heading_difference < (M_PI_2);
}

bool ObsDynamicProcessor::IsObstacleInReasonableCurvatureRoad(
    const Obstacle* const obstacle) const {
  if (obstacle == nullptr) {
    AERROR << "IsObstacleInReasonableCurvatureRoad nullptr check is failed!";
    return false;
  }
  const auto& sl_boundary = obstacle->PerceptionSLBoundary();
  if (!sl_boundary.has_end_l() || !sl_boundary.has_start_l() ||
      !sl_boundary.has_start_s() || !sl_boundary.has_end_s()) {
    AERROR << "sl boundary has s or l check is failed!";
    return false;
  }
  const auto& obs_start_point = GetProcessBound()
                                    ->GetReferenceLineInfo()
                                    ->reference_line()
                                    .GetReferencePoint(sl_boundary.start_s());
  const auto& obs_end_point = GetProcessBound()
                                  ->GetReferenceLineInfo()
                                  ->reference_line()
                                  .GetReferencePoint(sl_boundary.end_s());

  return fabs(obs_start_point.kappa()) <= GetConfig()
                                              .path_bounds_decider_config()
                                              .dynamic_obs_process_config()
                                              .obs_consider_filter_kappa() &&
         fabs(obs_end_point.kappa()) <= GetConfig()
                                            .path_bounds_decider_config()
                                            .dynamic_obs_process_config()
                                            .obs_consider_filter_kappa();
}

bool ObsDynamicProcessor::IsObstacleDeadAheadADCWithBuffer(
    const Obstacle* const obstacle) const {
  if (obstacle == nullptr) {
    AERROR << "IsObstacleDeadAheadADCWithBuffer nullptr check is failed!";
    return false;
  }
  const auto& sl_boundary = obstacle->PerceptionSLBoundary();
  if (!sl_boundary.has_end_l() || !sl_boundary.has_start_l() ||
      !sl_boundary.has_start_s() || !sl_boundary.has_end_s()) {
    AERROR << "sl boundary has s or l check is failed!";
    return false;
  }

  const double obs_middle_s = (sl_boundary.start_s() + sl_boundary.end_s()) / 2;
  double lane_left_width =
      GetProcessBound()->GetLaneBoundConf().path_default_lane_width() / 2;
  double lane_right_width =
      GetProcessBound()->GetLaneBoundConf().path_default_lane_width() / 2;
  GetProcessBound()->GetReferenceLineInfo()->reference_line().GetLaneWidth(
      obs_middle_s, &lane_left_width, &lane_right_width);
  const double lane_width = lane_left_width + lane_right_width;
  const double veh_width =
      common::VehicleConfigHelper::GetConfig().vehicle_param().width();

  if (lane_width <= veh_width + ADC_buffer_ +
                        GetConfig()
                            .path_bounds_decider_config()
                            .dynamic_obs_process_config()
                            .dynamic_reserve_solution_space()) {
    return false;
  }

  if (sl_boundary.start_l() >
          lane_left_width - GetConfig()
                                .path_bounds_decider_config()
                                .dynamic_obs_process_config()
                                .obs_consider_filter_l_buffer() ||
      sl_boundary.end_l() < GetConfig()
                                    .path_bounds_decider_config()
                                    .dynamic_obs_process_config()
                                    .obs_consider_filter_l_buffer() -
                                lane_right_width) {
    return true;
  }
  return false;
}

double ObsDynamicProcessor::LookUpValue(const double a, const double b,
                                        const double ratio) {
  return (b - a) * ratio + a;
}

void ObsDynamicProcessor::InterpolatePointUsingLinearApproximation(
    const std::vector<ObsPointDescription>::const_iterator& tp0_iter,
    const std::vector<ObsPointDescription>::const_iterator& tp1_iter,
    std::pair<std::vector<ObsPointDescription>::const_iterator,
              ObsPointDescription>* const obstacle_sl_boundary_ptr_point,
    const double t) {
  if (obstacle_sl_boundary_ptr_point == nullptr) {
    AERROR << "InterpolatePointUsingLinearApproximation nullptr check is "
              "failed!";
    return;
  }
  const ObsPointDescription& tp0 = *tp0_iter;
  const ObsPointDescription& tp1 = *tp1_iter;
  if (Compare(t, tp0.time) <= 0 || Compare(tp1.time, tp0.time) == 0) {
    *obstacle_sl_boundary_ptr_point = std::make_pair(tp0_iter, tp0);
  }
  if (Compare(t, tp1.time) >= 0) {
    *obstacle_sl_boundary_ptr_point = std::make_pair(tp1_iter, tp1);
  }
  ObsPointDescription point;
  point.time = t;
  double ratio = (t - tp0.time) / (tp1.time - tp0.time);
  point.center_p.set_s(ObsDynamicProcessor::LookUpValue(
      tp0.center_p.s(), tp1.center_p.s(), ratio));
  point.center_p.set_l(ObsDynamicProcessor::LookUpValue(
      tp0.center_p.l(), tp1.center_p.l(), ratio));

  point.low_left_p.set_s(ObsDynamicProcessor::LookUpValue(
      tp0.low_left_p.s(), tp1.low_left_p.s(), ratio));
  point.low_left_p.set_l(ObsDynamicProcessor::LookUpValue(
      tp0.low_left_p.l(), tp1.low_left_p.l(), ratio));

  point.low_right_p.set_s(ObsDynamicProcessor::LookUpValue(
      tp0.low_right_p.s(), tp1.low_right_p.s(), ratio));
  point.low_right_p.set_l(ObsDynamicProcessor::LookUpValue(
      tp0.low_right_p.l(), tp1.low_right_p.l(), ratio));

  point.upper_left_p.set_s(ObsDynamicProcessor::LookUpValue(
      tp0.upper_left_p.s(), tp1.upper_left_p.s(), ratio));
  point.upper_left_p.set_l(ObsDynamicProcessor::LookUpValue(
      tp0.upper_left_p.l(), tp1.upper_left_p.l(), ratio));

  point.upper_right_p.set_s(ObsDynamicProcessor::LookUpValue(
      tp0.upper_right_p.s(), tp1.upper_right_p.s(), ratio));
  point.upper_right_p.set_l(ObsDynamicProcessor::LookUpValue(
      tp0.upper_right_p.l(), tp1.upper_right_p.l(), ratio));
  *obstacle_sl_boundary_ptr_point = std::make_pair(tp0_iter, point);
}

void ObsDynamicProcessor::GetObstacleSLBoundaryPointAtTime(
    const double estimation_time,
    const std::vector<ObsPointDescription>::const_iterator& start_iter,
    const std::vector<ObsPointDescription>& obstacle_trajectory_envelope,
    std::pair<std::vector<ObsPointDescription>::const_iterator,
              ObsPointDescription>* const obstacle_sl_boundary_ptr_point) {
  if (obstacle_sl_boundary_ptr_point == nullptr) {
    AERROR << "GetObstacleSLBoundaryPointAtTime nullptr check is failed!";
    return;
  }
  if (obstacle_trajectory_envelope.size() < 2) {
    ADEBUG << "obstacle_trajectory_envelope.size: "
           << obstacle_trajectory_envelope.size();
    *obstacle_sl_boundary_ptr_point = std::make_pair(
        obstacle_trajectory_envelope.end(), ObsPointDescription());
  }

  auto comp = [](const ObsPointDescription& obs_point_description,
                 const double time) {
    return obs_point_description.time < time;
  };

  auto it_lower = std::lower_bound(
      start_iter, obstacle_trajectory_envelope.end(), estimation_time, comp);

  if (it_lower == obstacle_trajectory_envelope.begin()) {
    *obstacle_sl_boundary_ptr_point =
        std::make_pair(obstacle_trajectory_envelope.begin(),
                       *obstacle_trajectory_envelope.begin());
  }
  if (it_lower == obstacle_trajectory_envelope.end()) {
    *obstacle_sl_boundary_ptr_point =
        std::make_pair(--obstacle_trajectory_envelope.end(),
                       *obstacle_trajectory_envelope.rbegin());
    it_lower = obstacle_trajectory_envelope.end() - 1;
  }

  auto start_lower = it_lower - 1;
  InterpolatePointUsingLinearApproximation(
      start_lower, it_lower, obstacle_sl_boundary_ptr_point, estimation_time);
}

void ObsDynamicProcessor::CalculateSLBoundary(
    const SpeedKnotInfo& speed_data, const Obstacle& obstacle,
    std::vector<SLBoundaryInfo>* const obs_sl_boundary) {
  // PERF_BLOCK_START();
  double veh_s_lb = 0.0;
  double veh_s_ub = 0.0;
  if (obs_sl_boundary == nullptr) {
    AERROR << "CalculateSLBoundary nullptr check is failed!";
    return;
  }
  if (BothHaveOverlap(obstacle.PerceptionSLBoundary().start_s(),
                      obstacle.PerceptionSLBoundary().end_s(),
                      planning_start_point_sl_.s(), veh_s_lb, veh_s_ub)) {
    obs_sl_boundary->emplace_back(obstacle.PerceptionSLBoundary(), true,
                                  obstacle.Id());
  } else {
    obs_sl_boundary->emplace_back(obstacle.PerceptionSLBoundary(), false,
                                  obstacle.Id());
  }

  if (!obstacle.HasTrajectory() || speed_data.empty()) {
    AERROR << "CalculateSLBoundary empty check is failed!";
    return;
  }

  double max_consider_time_length = 0.0;
  if (obstacle.Perception().type() == perception::PerceptionObstacle::VEHICLE) {
    max_consider_time_length = GetConfig()
                                   .path_bounds_decider_config()
                                   .dynamic_obs_process_config()
                                   .obs_boundary_evaluate_vehicle_time_length();
  } else {
    max_consider_time_length = GetConfig()
                                   .path_bounds_decider_config()
                                   .dynamic_obs_process_config()
                                   .obs_boundary_evaluate_other_time_length();
  }

  const auto& obstacle_trajectory_envelope = obstacle.GetTrajectoryEnvelope();
  const auto& obs_trajectory_end_pt = obstacle_trajectory_envelope.rbegin();
  auto speed_data_iter = speed_data.begin();
  auto obs_point_description_iter = obstacle_trajectory_envelope.begin();

  double estimation_time = -1.0;
  SLBoundary rough_sl_boundary;
  auto obstacle_sl_boundary_ptr_point =
      std::make_pair(obstacle_trajectory_envelope.end(), ObsPointDescription());
  std::string id;
  while (speed_data_iter != speed_data.end()) {
    estimation_time = speed_data_iter->first;
    if (IsZero(estimation_time)) {
      ++speed_data_iter;
      continue;
    }
    if (Compare(estimation_time, obs_trajectory_end_pt->time) > 0 ||
        Compare(estimation_time, max_consider_time_length) > 0) {
      break;
    }
    // double start_time_lambda_to_regular = common::Clock::NowInSeconds();
    GetObstacleSLBoundaryPointAtTime(
        estimation_time, obs_point_description_iter,
        obstacle_trajectory_envelope, &obstacle_sl_boundary_ptr_point);
    // double end_time_lambda_to_regular = common::Clock::NowInSeconds();
    // AERROR << "lambda_to_regular elapsed time = "
    //        << (end_time_lambda_to_regular - start_time_lambda_to_regular)*1000 <<" ms";
    obs_point_description_iter = obstacle_sl_boundary_ptr_point.first;

    if (obs_point_description_iter == obstacle_trajectory_envelope.end()) {
      ADEBUG << "at estimation_time: " << estimation_time
             << ", failed to get obstacle_sl_boundary.";
      break;
    }
    if (obstacle_sl_boundary_ptr_point.second.time < 0.0) {
      ADEBUG << "obstacle_sl_boundary.time: "
             << obstacle_sl_boundary_ptr_point.second.time;
      ++speed_data_iter;
      continue;
    }

    static constexpr double kDynamicObsExtendLonLength = 0.50;
    rough_sl_boundary = RoughObstacleSLBoundary(
        obstacle_sl_boundary_ptr_point.second,
        obstacle.PerceptionSLBoundary().start_l(),
        obstacle.PerceptionSLBoundary().end_l(), kDynamicObsExtendLonLength);
    const auto& veh_config =
        common::VehicleConfigHelper::GetConfig().vehicle_param();
    veh_s_lb = speed_data_iter->second - veh_config.back_edge_to_center();
    veh_s_ub = speed_data_iter->second + veh_config.front_edge_to_center();
    id = absl::StrCat(obstacle.Id(), "-", estimation_time);
    if (BothHaveOverlap(rough_sl_boundary.start_s(), rough_sl_boundary.end_s(),
                        planning_start_point_sl_.s(), veh_s_lb, veh_s_ub)) {
#if ENBLE_ACCURATE_SLBOUNDARY
      const auto trajectory_point = obstacle.GetPointAtTime(estimation_time);
      auto obstacle_Box2d = obstacle.GetBoundingBox(trajectory_point);
      obstacle_Box2d.LongitudinalExtend(kDynamicObsExtendLonLength);
      SLBoundary obs_sl_boundary;
      if (GetReferenceLineInfo()->reference_line().GetSLBoundary(
              obstacle_Box2d, &obs_sl_boundary)) {
        ADEBUG << "lon overlap dynamic obs id: " << id;
        sl_boundary.emplace_back(obs_sl_boundary, id);
      } else {
        AERROR << "lon overlap dynamic obs id: " << id
               << " and fail to get SLBoundary.";
      }
#else
      ADEBUG << "lon overlap dynamic obs id: " << id
             << " , using rough sl boundary.";
      obs_sl_boundary->emplace_back(rough_sl_boundary, true, id);
#endif
    } else {
      ADEBUG << "This obstacle is not overlaped with adc!";
      obs_sl_boundary->emplace_back(rough_sl_boundary, false, id);
    }
    ++speed_data_iter;
  }
}

bool ObsDynamicProcessor::CalculateObstacleEdges(
    const std::vector<SLBoundaryInfo>& sl_boundaries, const Obstacle& obstacle,
    std::vector<ObstacleEdge>* const obstacle_edges_process_bound,
    std::vector<ObstacleEdge>* const obstacle_edges_process_towing_line) {
  if (sl_boundaries.empty() || obstacle_edges_process_bound == nullptr ||
      obstacle_edges_process_towing_line == nullptr) {
    AERROR << "Calculate obstacle edges input error.";
    return false;
  }
  bool is_towing_obstacle = false;
  for (const auto& obstacle_sl : sl_boundaries) {
    double lat_buffer = GetDynamicObstacleBuffer(obstacle_sl, obstacle);
    // use all obstacles to process towing line
    ObstacleEdge curr_obstacle_edges = {
        std::get<0>(obstacle_sl).start_s() -
            GetConfig()
                .path_bounds_decider_config()
                .obstacle_buffer_process_config()
                .obs_dynamic_lon_start_buffer(),
        std::get<0>(obstacle_sl).end_s() + GetConfig()
                                               .path_bounds_decider_config()
                                               .obstacle_buffer_process_config()
                                               .obs_dynamic_lon_end_buffer(),
        std::get<0>(obstacle_sl).start_l() - lat_buffer,
        std::get<0>(obstacle_sl).end_l() + lat_buffer,
        std::get<2>(obstacle_sl)};

    // process nudge obsatcle
    for (const auto& decider_tag : obstacle.decider_tags()) {
      if (GetProcessBound()
                  ->GetObsTowingConf()
                  .use_obstacle_decider_process_dynamic_obstacle()
              ? (absl::StrContains(decider_tag, "dynamic-left-nudge") ||
                 absl::StrContains(decider_tag, "dynamic-right-nudge"))
              : absl::StrContains(decider_tag, "bigcar")) {
        is_towing_obstacle = true;
        obstacle_edges_process_towing_line->emplace_back(curr_obstacle_edges);
      }
    }

    // if the obstacle is overlaped with adc, use it to process bound
    if (std::get<1>(obstacle_sl)) {
      obstacle_edges_process_bound->emplace_back(curr_obstacle_edges);
    }
  }
  if (!is_towing_obstacle) {
    GetProcessBound()
        ->GetMutableReferenceLineInfo()
        ->path_decision()
        ->Find(obstacle.Id())
        ->SetMaxExpectTowingL(0.0);
  }
  return true;
}

double ObsDynamicProcessor::GetDynamicObstacleBuffer(
    const SLBoundaryInfo& sl_boundary, const Obstacle& obstacle) {
  UNUSED(obstacle);
  if (GetInjector() == nullptr) {
    AERROR << "GetDynamicObstacleBuffer nullptr check is failed!";
    return GetConfig()
        .path_bounds_decider_config()
        .obstacle_buffer_process_config()
        .obs_dynamic_lat_buffer();
  }
  if (GetConfig()
          .path_bounds_decider_config()
          .dynamic_obs_process_config()
          .enable_dynamic_obs_dynamic_buffer_calculate()) {
    ADEBUG << "----------Dynamic_Buffer_Calculate--------";
    if (!GetInjector()->ego_info()->vehicle_state().has_linear_velocity() ||
        !std::get<0>(sl_boundary).has_start_l() ||
        !std::get<0>(sl_boundary).has_start_s() ||
        !std::get<0>(sl_boundary).has_end_l()) {
      AERROR << "dynamic obs buffer is an err.";
      return GetConfig()
          .path_bounds_decider_config()
          .obstacle_buffer_process_config()
          .obs_dynamic_lat_buffer();
    }

    const double adc_speed =
        GetInjector()->ego_info()->vehicle_state().linear_velocity();

    return dynamic_obs_dynamic_buffer_calculate_
        .DynamicObsDynamicBufferCalculate(adc_speed);
  }

  return GetConfig()
      .path_bounds_decider_config()
      .obstacle_buffer_process_config()
      .obs_dynamic_lat_buffer();

  // switch (obstacle.Perception().type()) {
  //   case perception::PerceptionObstacle::VEHICLE:
  //     return GetConfig() .path_bounds_decider_config().obs_dynamic_lat_buffer();
  //     //    case perception::PerceptionObstacle::BIG_VEHICLE:
  //     //      return FLAGS_big_vehicle_obs_dynamic_lat_buffer;
  //   default:
  //     return GetConfig() .path_bounds_decider_config().obs_dynamic_lat_buffer();
  // }
}

void ObsDynamicProcessor::ProcessObstacleEdgesForSetTowingPoints(
    const std::vector<ObstacleEdge>& obstacle_edges_process_towing_line,
    PathBound* const path_boundaries, TowingPointsInfo* const towing_points) {
  if (path_boundaries == nullptr || towing_points == nullptr) {
    AERROR << "ProcessObstacleEdgesForSetTowingPoints nullptr check is failed!";
    return;
  }
  if (obstacle_edges_process_towing_line.empty()) {
    ADEBUG << "there is no edge need to be processed!";
    return;
  }
  int end_index = 0;
  int start_index = 0;
  std::string curr_obstacle_id =
      obstacle_edges_process_towing_line[end_index].obstacle_id;
  // process all the obstacle edges by consider with:
  // 1.every obstacle just call TowingPointsBigCarProcess once.
  // 2.use lower s of the init status obstacle edge as total lower s,
  //   use upper s of the last status obstacle edge as total upper s.
  while (end_index <
         static_cast<int>(obstacle_edges_process_towing_line.size())) {
    const std::string& curr_obstacle_edge_id =
        obstacle_edges_process_towing_line[end_index].obstacle_id;
    std::string naked_curr_obstacle_edge_id;
    std::pair<double, double> final_obs_l_edges = std::make_pair(
        obstacle_edges_process_towing_line[end_index].obstacle_edge_l_min,
        obstacle_edges_process_towing_line[end_index].obstacle_edge_l_max);
    int nPos = static_cast<int>(curr_obstacle_edge_id.find('-'));
    if (nPos != -1) {
      naked_curr_obstacle_edge_id = curr_obstacle_edge_id.substr(0, nPos);
    } else {
      naked_curr_obstacle_edge_id = curr_obstacle_edge_id;
    }
    ADEBUG << "end_index = " << end_index << ", start_index = " << start_index
           << ", obstacle id = " << curr_obstacle_id
           << ", obstacle edge id = " << curr_obstacle_edge_id;
    // skip the edge of same obstacle and move the end index to next edge,
    // but this condition not suitable for the last edge in the vector.
    if (naked_curr_obstacle_edge_id == curr_obstacle_id &&
        end_index !=
            static_cast<int>(obstacle_edges_process_towing_line.size()) - 1) {
      ++end_index;
      continue;
    }
    // whether is the last edge in the vector, means last obstacle,
    // use end_index and start index to process, then break.
    std::pair<double, double> final_obs_s_edges = {0.0, 0.0};
    if (end_index ==
        static_cast<int>(obstacle_edges_process_towing_line.size()) - 1) {
      // Save lower s and upper s after transforing dynamicobstacle to static obstacle .
      // first: lower s, second: upper s
      final_obs_s_edges.second =
          obstacle_edges_process_towing_line[end_index].obstacle_edge_end_s;
      final_obs_s_edges.first =
          obstacle_edges_process_towing_line[start_index].obstacle_edge_start_s;
      ADEBUG << "end_index = " << end_index << ", start_index = " << start_index
             << ", towing_start_edge_s = " << final_obs_s_edges.first
             << ", towing_end_edge_s = " << final_obs_s_edges.second
             << ", curr_obstacle_id = " << curr_obstacle_id
             << ", dynamic_obs_edge_process_towing_line[i] id = "
             << obstacle_edges_process_towing_line[end_index].obstacle_id
             << ", dynamic_obs_edge_process_towing_line[j] id = "
             << obstacle_edges_process_towing_line[start_index].obstacle_id;
      ToCallTowingProcess(final_obs_s_edges, final_obs_l_edges,
                          curr_obstacle_id, path_boundaries, towing_points);
      break;
    }
    // curr edge id is different form last edge means can process last obstacle,
    // use (end_index - 1) and start index to process,
    // then change the start index and current obstacle id and go on.
    int process_index = std::max(0, end_index - 1);
    // Save lower s and upper s after transforing dynamicobstacle to static obstacle .
    // first: start s, second: end s
    final_obs_s_edges.second =
        obstacle_edges_process_towing_line[process_index].obstacle_edge_end_s;
    final_obs_s_edges.first =
        obstacle_edges_process_towing_line[start_index].obstacle_edge_start_s;
    ADEBUG << "end_index = " << end_index << ", start_index = " << start_index
           << ", process_index = " << process_index
           << ", towing_start_edge_s = " << final_obs_s_edges.first
           << ", towing_end_edge_s = " << final_obs_s_edges.second
           << ", curr_obstacle_id = " << curr_obstacle_id
           << ", dynamic_obs_edge_process_towing_line[end_index] id = "
           << obstacle_edges_process_towing_line[end_index].obstacle_id
           << ", dynamic_obs_edge_process_towing_line[process_index] id = "
           << obstacle_edges_process_towing_line[process_index].obstacle_id
           << ", dynamic_obs_edge_process_towing_line[start_index] id = "
           << obstacle_edges_process_towing_line[start_index].obstacle_id;
    ToCallTowingProcess(final_obs_s_edges, final_obs_l_edges, curr_obstacle_id,
                        path_boundaries, towing_points);
    start_index = end_index;
    curr_obstacle_id = naked_curr_obstacle_edge_id;
    ++end_index;
  }
}

void ObsDynamicProcessor::ToCallTowingProcess(
    const std::pair<double, double>& cur_obs_s_edges,
    const std::pair<double, double>& cur_obs_l_edges,
    const std::string& obstacle_id, PathBound* const path_boundaries,
    TowingPointsInfo* const towing_points) {
  if (path_boundaries == nullptr || towing_points == nullptr) {
    AERROR << "ToCallTowingProcess nullpter check is failed!";
    return;
  }
  const auto* const curr_obs = GetProcessBound()
                                   ->GetMutableReferenceLineInfo()
                                   ->path_decision()
                                   ->obstacles()
                                   .Find(obstacle_id);
  if (curr_obs == nullptr) {
    ADEBUG << "Find obstacle failed!";
    return;
  }
  bool curr_obs_first_exist = false;
  const Obstacle* last_obs = nullptr;
  if (GetInjector() == nullptr || GetInjector()->frame_history() == nullptr ||
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
                   .Find(obstacle_id);
    if (last_obs == nullptr) {
      curr_obs_first_exist = true;
    }
  }
  const double delta_v = std::fabs(GetProcessBound()
                                       ->GetReferenceLineInfo()
                                       ->vehicle_state()
                                       .linear_velocity() -
                                   curr_obs->speed());
  for (const auto& decider_tag : curr_obs->decider_tags()) {
    bool is_nudge_from_obs_left =
        GetProcessBound()
                ->GetObsTowingConf()
                .use_obstacle_decider_process_dynamic_obstacle()
            ? absl::StrContains(decider_tag, "dynamic-left-nudge")
            : absl::StrContains(decider_tag, "bigcar-left-nudge");
    bool is_nudge_from_obs_right =
        GetProcessBound()
                ->GetObsTowingConf()
                .use_obstacle_decider_process_dynamic_obstacle()
            ? absl::StrContains(decider_tag, "dynamic-right-nudge")
            : absl::StrContains(decider_tag, "bigcar-right-nudge");
    if (is_nudge_from_obs_left) {
      ADEBUG << "left: obs id = " << obstacle_id
             << " is prepared for TowingPointsBigCarProcess";
      const double curr_obs_expect_towing_l =
          LookUpTowingLDistance(std::abs(cur_obs_l_edges.second),
                                GetProcessBound()
                                    ->GetObsTowingConf()
                                    .towing_l_distance_segment_conf()
                                    .obs_to_center_line_dis_segment(),
                                GetProcessBound()
                                    ->GetObsTowingConf()
                                    .towing_l_distance_segment_conf()
                                    .dynamic_obs_expect_towing_segment_l());
      const double dynamic_obs_expect_towing_l =
          FLAGS_use_dynamic_adjust_towing_l
              ? curr_obs_first_exist ? curr_obs_expect_towing_l
                                     : std::max(curr_obs_expect_towing_l,
                                                last_obs->GetMaxExpectTowingL())
              : GetProcessBound()
                    ->GetObsTowingConf()
                    .default_obstacle_towing_l();
      ADEBUG << "switch: " << FLAGS_use_dynamic_adjust_towing_l
             << ", cur_obs_l_edges.second = " << cur_obs_l_edges.second
             << ", dynamic_obs_expect_towing_l = "
             << dynamic_obs_expect_towing_l;
      TowingPointsBigCarProcess(cur_obs_s_edges, obstacle_id, path_boundaries,
                                towing_points, true, delta_v,
                                dynamic_obs_expect_towing_l);
      break;
    }
    if (is_nudge_from_obs_right) {
      ADEBUG << "right: obs id = " << obstacle_id
             << " is prepared for TowingPointsBigCarProcess";
      const double curr_obs_expect_towing_l =
          LookUpTowingLDistance(std::abs(cur_obs_l_edges.first),
                                GetProcessBound()
                                    ->GetObsTowingConf()
                                    .towing_l_distance_segment_conf()
                                    .obs_to_center_line_dis_segment(),
                                GetProcessBound()
                                    ->GetObsTowingConf()
                                    .towing_l_distance_segment_conf()
                                    .dynamic_obs_expect_towing_segment_l());
      const double dynamic_obs_expect_towing_l =
          FLAGS_use_dynamic_adjust_towing_l
              ? curr_obs_first_exist ? curr_obs_expect_towing_l
                                     : std::max(curr_obs_expect_towing_l,
                                                last_obs->GetMaxExpectTowingL())
              : GetProcessBound()
                    ->GetObsTowingConf()
                    .default_obstacle_towing_l();
      ADEBUG << "switch: " << FLAGS_use_dynamic_adjust_towing_l
             << ", cur_obs_l_edges.first = " << cur_obs_l_edges.first
             << ", dynamic_obs_expect_towing_l = "
             << dynamic_obs_expect_towing_l
             << ", curr_obs_expect_towing_l = " << curr_obs_expect_towing_l;
      TowingPointsBigCarProcess(cur_obs_s_edges, obstacle_id, path_boundaries,
                                towing_points, false, delta_v,
                                dynamic_obs_expect_towing_l);
      break;
    }
  }
}

bool ObsDynamicProcessor::ConstructBoundaryFromDynamicObstacles(
    std::vector<ObstacleEdge>* const obstacle_edges,
    PathBound* const path_boundaries, std::string* const blocking_obstacle_id) {
  // PERF_FUNCTION_WITH_NAME("ConstructBoundaryFromDynamicObstacles");
  if (blocking_obstacle_id == nullptr || obstacle_edges == nullptr) {
    AERROR << "ConstructBoundaryFromDynamicObstacles nullptr check failed!";
    return false;
  }
  if (path_boundaries->empty()) {
    AERROR << "path boundary size: " << path_boundaries->size();
    return false;
  }
  if (obstacle_edges->empty()) {
    ADEBUG << "dynamic obstacles size: " << obstacle_edges->size();
    return true;
  }
  double center_line = planning_start_point_sl_.l();
  std::multiset<std::pair<double, std::string>, std::greater<>> right_bounds;
  right_bounds.insert(
      std::make_pair(std::numeric_limits<double>::lowest(), " "));
  std::multiset<std::pair<double, std::string>> left_bounds;
  left_bounds.insert(std::make_pair(std::numeric_limits<double>::max(), " "));
  std::vector<size_t> processing_obs;
  int path_blocked_idx = -1;
  double curr_s = 0.0;
  std::size_t obs_idx = 0;
  /******************************************************************/

  // using obstacle edges constraint shrinks path bounds.
  for (size_t i = 0; i < path_boundaries->size(); ++i) {
    if (path_blocked_idx != -1 || obstacle_edges->size() < obs_idx) {
      break;
    }
    curr_s = std::get<0>(path_boundaries->at(i));
    GetProcessBound()->DeleteOutOfScopeObstacle(
        obstacle_edges, &processing_obs, &right_bounds, &left_bounds, curr_s);

    if (obs_idx < obstacle_edges->size() &&
        Compare(obstacle_edges->at(obs_idx).obstacle_edge_start_s, curr_s) <=
            0) {
      // new obstacle get into our considered scope
      while (obs_idx < obstacle_edges->size() &&
             Compare(obstacle_edges->at(obs_idx).obstacle_edge_start_s,
                     curr_s) <= 0) {
        // A new obstacle enters into our scope:
        //   - Decide which direction for the ADC to pass.
        //   - Update the left/right bound accordingly.
        //   - If boundaries blocked, then decide whether can side-pass.
        //   - If yes, then borrow neighbor lane to side-pass.
        auto& curr_obstacle = obstacle_edges->at(obs_idx);
        const double curr_obstacle_upper_s = curr_obstacle.obstacle_edge_end_s;
        const double curr_obstacle_l_min = curr_obstacle.obstacle_edge_l_min;
        const double curr_obstacle_l_max = curr_obstacle.obstacle_edge_l_max;
        const std::string curr_obstacle_id = curr_obstacle.obstacle_id;
        std::string naked_curr_obstacle_id;
        int nPos = static_cast<int>(curr_obstacle_id.find('-'));
        if (nPos != -1) {
          naked_curr_obstacle_id = curr_obstacle_id.substr(0, nPos);
        } else {
          naked_curr_obstacle_id = curr_obstacle_id;
        }
        if (Compare((curr_obstacle_l_min + curr_obstacle_l_max) / 2,
                    center_line) < 0) {
          // Obstacle is to the right of center-line, should pass from left.
          if (Compare(curr_obstacle_upper_s, curr_s) >= 0) {
            //   const auto extra_buffer = DynamicBuffer(
            //       *path_boundaries, *obstacle_edges, i, obs_idx, false);
            curr_obstacle.obstacle_edge_l_max = curr_obstacle_l_max;
            right_bounds.insert(std::make_pair(
                curr_obstacle.obstacle_edge_l_max, curr_obstacle_id));
          }
        } else {
          // Obstacle is to the left of center-line, should pass from right.
          if (Compare(curr_obstacle_upper_s, curr_s) >= 0) {
            // const auto extra_buffer = DynamicBuffer(
            //     *path_boundaries, *obstacle_edges, i, obs_idx, true);
            curr_obstacle.obstacle_edge_l_min = curr_obstacle_l_min;
            left_bounds.insert(std::make_pair(curr_obstacle.obstacle_edge_l_min,
                                              curr_obstacle_id));
          }
        }
        processing_obs.emplace_back(obs_idx);
        // every new obstacle which get into our considered scope need to do block check.
        if (!UpdatePathBoundaryAndCenterLineWithBuffer(
                i, left_bounds.begin()->first, right_bounds.begin()->first,
                path_boundaries, &center_line)) {
          ADEBUG << "Path is blocked at s = " << curr_s;
          path_blocked_idx = static_cast<int>(i);
          *blocking_obstacle_id = curr_obstacle_id;
          ADEBUG << "block id = " << *blocking_obstacle_id;
          break;
        }
        ++obs_idx;
      }
    } else {
      // no obstacle update, update boundary as before.
      if (!UpdatePathBoundaryAndCenterLineWithBuffer(
              i, left_bounds.begin()->first, right_bounds.begin()->first,
              path_boundaries, &center_line)) {
        ADEBUG << "Path is blocked at s = " << curr_s;
        *blocking_obstacle_id =
            Compare(std::fabs(left_bounds.begin()->first),
                    std::fabs(right_bounds.begin()->first)) < 0
                ? left_bounds.begin()->second
                : right_bounds.begin()->second;
        ADEBUG << "block id = " << *blocking_obstacle_id;
        break;
      }
    }
  }
  /******************************************************************/
#if DYNAMIC_OBSTACLE_CONSTRAINT_TRIM
  TrimPathBounds(path_blocked_idx, path_boundaries);
#endif
  return true;
}

double ObsDynamicProcessor::GetBufferBetweenADCCenterAndEdge() {
  return VehicleConfigHelper::GetConfig().vehicle_param().width() / 2;
}

bool ObsDynamicProcessor::UpdatePathBoundaryAndCenterLineWithBuffer(
    const size_t idx, double left_bound, double right_bound,
    PathBound* const path_boundaries, double* const center_line) {
  if (path_boundaries == nullptr || center_line == nullptr) {
    AERROR << "UpdatePathBoundaryAndCenterLineWithBuffer nullptr check is "
              "failed!";
    return false;
  }
  if (std::isgreaterequal(left_bound, std::numeric_limits<double>::max()) &&
      std::islessequal(right_bound, std::numeric_limits<double>::lowest())) {
    ADEBUG << "no dynamic obstacle causes path bounds [s: "
           << std::get<0>((*path_boundaries)[idx]) << "] to trim.";
    return true;
  }
  // Loose path bound constraint to avoid too tight.
  ProcessPathboundFromStartPoint(idx, path_boundaries, &left_bound,
                                 &right_bound);

  // Update the right bound (l_min):
  ADEBUG << "DY_Path_right " << idx << " /t"
         << std::get<1>((*path_boundaries)[idx]) << "/t"
         << (right_bound + GetBufferBetweenADCCenterAndEdge());
  ADEBUG << "DY_Path_left " << idx << " /t"
         << std::get<2>((*path_boundaries)[idx]) << "/t"
         << (left_bound - GetBufferBetweenADCCenterAndEdge());

  double new_l_min =
      std::fmax(std::get<1>((*path_boundaries)[idx]),
                right_bound + GetBufferBetweenADCCenterAndEdge());
  // Update the left bound (l_max):
  double new_l_max = std::fmin(std::get<2>((*path_boundaries)[idx]),
                               left_bound - GetBufferBetweenADCCenterAndEdge());

  // Check if ADC is blocked.
  // If blocked, don't update anything, return false.
  if (new_l_min > new_l_max) {
    ADEBUG << "Path is blocked at idx = " << idx;
    return false;
  }
  // Otherwise, update path_boundaries and center_line; then return true.
  std::get<1>((*path_boundaries)[idx]) = new_l_min;
  std::get<2>((*path_boundaries)[idx]) = new_l_max;

  *center_line = (std::get<1>((*path_boundaries)[idx]) +
                  std::get<2>((*path_boundaries)[idx])) /
                 2;
  return true;
}

void ObsDynamicProcessor::ProcessPathboundFromStartPoint(
    const size_t idx, const PathBound* const path_boundaries,
    double* const left_bound, double* const right_bound) {
  if (GetInjector() == nullptr || left_bound == nullptr ||
      right_bound == nullptr || path_boundaries == nullptr) {
    AERROR << "ProcessPathboundFromStartPoint nullptr check is failed!";
    return;
  }
  if (idx >= path_boundaries->size()) {
    ADEBUG << "idx beyond path boundaries's size.";
    return;
  }

  double start_path_bound_l_min = 0.0;
  double start_path_bound_l_max = 0.0;
  double start_path_bound_s = 0.0;
  std::tie(start_path_bound_s, start_path_bound_l_min, start_path_bound_l_max) =
      path_boundaries->at(0);

  if (GetConfig()
          .path_bounds_decider_config()
          .is_extend_lane_bounds_to_include_adc() &&
      idx == 0) {
    *left_bound = std::min(
        std::max(*left_bound, planning_start_point_sl_.l() + ADC_buffer_ +
                                  GetBufferBetweenADCCenterAndEdge()),
        start_path_bound_l_max + GetBufferBetweenADCCenterAndEdge());

    *right_bound = std::max(
        std::min(*right_bound, planning_start_point_sl_.l() - ADC_buffer_ -
                                   GetBufferBetweenADCCenterAndEdge()),
        start_path_bound_l_min - GetBufferBetweenADCCenterAndEdge());
    ADEBUG << FIXED << SETPRECISION(3) << "path bound index: " << idx
           << " start s: " << start_path_bound_s
           << " start left bound: " << *left_bound
           << " start right bound: " << *right_bound;
    return;
  }

  double path_bound_l_min = 0.0;
  double path_bound_l_max = 0.0;
  double path_bound_s = 0.0;
  std::tie(path_bound_s, path_bound_l_min, path_bound_l_max) =
      path_boundaries->at(idx);
  double delta_s = path_bound_s - planning_start_point_sl_.s();
  if (ComparedToZero(delta_s) < 1 || Compare(delta_s, extend_length_) > 0) {
    ADEBUG << "Not extend length considered area!";
    return;
  }

  const double new_path_bound_l =
      delta_s * GetProcessBound()->GetStartPointDl();
  if (GetConfig()
          .path_bounds_decider_config()
          .is_extend_lane_bounds_to_include_adc()) {
    *left_bound = std::max(
        std::max(*left_bound, start_path_bound_l_max + new_path_bound_l +
                                  GetBufferBetweenADCCenterAndEdge()),
        planning_start_point_sl_.l() + ADC_buffer_ +
            GetBufferBetweenADCCenterAndEdge());

    *right_bound = std::min(
        std::min(*right_bound, start_path_bound_l_min + new_path_bound_l -
                                   GetBufferBetweenADCCenterAndEdge()),
        planning_start_point_sl_.l() - ADC_buffer_ -
            GetBufferBetweenADCCenterAndEdge());
  } else {
    *left_bound =
        std::max(*left_bound, start_path_bound_l_max + new_path_bound_l +
                                  GetBufferBetweenADCCenterAndEdge());

    *right_bound =
        std::min(*right_bound, start_path_bound_l_min + new_path_bound_l -
                                   GetBufferBetweenADCCenterAndEdge());
  }
  if (std::isgreaterequal(*left_bound, std::numeric_limits<double>::max()) ||
      std::islessequal(*right_bound, std::numeric_limits<double>::lowest())) {
    ADEBUG << FIXED << SETPRECISION(3) << "path bound index: " << idx
           << " bound s: " << path_bound_s << " left bound: ["
           << std::isgreaterequal(*left_bound,
                                  std::numeric_limits<double>::max())
           << "] max"
           << " right bound: ["
           << std::islessequal(*right_bound,
                               std::numeric_limits<double>::lowest())
           << "] min";
    return;
  }
  ADEBUG << FIXED << SETPRECISION(3) << "path bound index: " << idx
         << " bound s: " << path_bound_s << " left bound: " << *left_bound
         << " right bound: " << *right_bound;
}

void ObsDynamicProcessor::TrimPathBounds(const int path_blocked_idx,
                                         PathBound* const path_boundaries) {
  if (path_boundaries == nullptr) {
    AERROR << "TrimPathBounds nullptr check is failed!";
    return;
  }
  if (path_blocked_idx == -1) {
    return;
  }
  if (path_blocked_idx == 0) {
    ADEBUG << "Completely blocked. Cannot move at all.";
  }
  int range = static_cast<int>(path_boundaries->size()) - path_blocked_idx;
  const auto erase_start_iter = std::prev(std::end(*path_boundaries), range);
  path_boundaries->erase(erase_start_iter, std::end(*path_boundaries));
}

double ObsDynamicProcessor::DynamicBuffer(
    const PathBound& path_boundary,
    const std::vector<ObstacleEdge>& obstacle_edges,
    const size_t path_boundary_index, const size_t obstacle_edge_index,
    const bool is_left_obstacle) const {
  return 0.0;
  if (GetProcessBound()->GetReferenceLineInfo() == nullptr) {
    AERROR << "DynamicBuffer nullptr check is failed!";
    return 0.0;
  }

  const double adc_speed = GetProcessBound()
                               ->GetReferenceLineInfo()
                               ->vehicle_state()
                               .linear_velocity();
  if (Compare(GetConfig()
                  .path_bounds_decider_config()
                  .dynamic_obs_process_config()
                  .obs_big_car_filter_adc_speed(),
              adc_speed) == 1 ||
      !is_avoid_big_car_) {
    return 0.0;
  }
  if (path_boundary.empty() || obstacle_edges.empty() ||
      path_boundary_index >= path_boundary.size() - 1 ||
      obstacle_edge_index >= obstacle_edges.size() - 1) {
    return 0.0;
  }
  // judge the type of obstacle.
  const auto pose_order =
      obstacle_edges[obstacle_edge_index].obstacle_id.find_last_of('-');
  const std::string obstacle_id =
      obstacle_edges[obstacle_edge_index].obstacle_id.substr(0, pose_order);
  if (obstacle_id.empty()) {
    ADEBUG << "obstacle id is empty.";
    return 0.0;
  }
  const auto* const obstacle_ptr =
      GetProcessBound()->GetReferenceLineInfo()->path_decision().Find(
          obstacle_id);
  if (obstacle_ptr == nullptr) {
    return 0.0;
  }
  const auto& obs_sl_boundary = obstacle_ptr->PerceptionSLBoundary();
  if (obstacle_ptr->Perception().type() !=
          perception::PerceptionObstacle::VEHICLE ||
      !obs_sl_boundary.has_start_s() || !obs_sl_boundary.has_end_s()) {
    return 0.0;
  }
  if (std::find(big_car_ids_.begin(), big_car_ids_.end(), obstacle_id) ==
      big_car_ids_.end()) {
    return 0.0;
  }
  // judge opposite path bounds' obstacle edge.
  double considering_length =
      (static_cast<int>(path_boundary.size()) - 1) *
      GetProcessBound()->GetReferenceLineInfo()->PathBoundsDeciderResolution();
  if (considering_length <
      obs_sl_boundary.start_s() - planning_start_point_sl_.s()) {
    return 0.0;
  }

  std::size_t aim_obstacle_edge_index = obstacle_edge_index + 1;
  std::size_t aim_path_boundary_index = path_boundary_index;
  bool find_aim_obstacle_edge = false;
  while (aim_path_boundary_index < path_boundary.size()) {
    double center_line = (std::get<1>(path_boundary[aim_path_boundary_index]) +
                          std::get<2>(path_boundary[aim_path_boundary_index])) /
                         2;
    while (aim_obstacle_edge_index < obstacle_edges.size()) {
      if (obstacle_edges[aim_obstacle_edge_index].obstacle_edge_start_s >
          std::get<0>(path_boundary[aim_path_boundary_index])) {
        break;
      }
      double aim_obstacle_edge_center_line =
          (obstacle_edges[aim_obstacle_edge_index].obstacle_edge_l_min +
           obstacle_edges[aim_obstacle_edge_index].obstacle_edge_l_max) /
          2;
      if ((!is_left_obstacle && aim_obstacle_edge_center_line >= center_line) ||
          (is_left_obstacle && aim_obstacle_edge_center_line < center_line)) {
        find_aim_obstacle_edge = true;
        break;
      }
      ++aim_obstacle_edge_index;
    }

    if (find_aim_obstacle_edge ||
        aim_obstacle_edge_index == obstacle_edges.size()) {
      break;
    }
    ++aim_path_boundary_index;
  }
  double buffer = 0.0;
  if (is_left_obstacle) {
    buffer = std::fmin(obstacle_edges[obstacle_edge_index].obstacle_edge_l_min -
                           GetBufferBetweenADCCenterAndEdge(),
                       std::get<2>(path_boundary[path_boundary_index])) -
             std::get<1>(path_boundary[path_boundary_index]);
  } else {
    buffer = std::get<2>(path_boundary[path_boundary_index]) -
             std::fmax(obstacle_edges[obstacle_edge_index].obstacle_edge_l_max +
                           GetBufferBetweenADCCenterAndEdge(),
                       std::get<1>(path_boundary[path_boundary_index]));
  }

  ADEBUG << std::boolalpha << "find_aim_obstacle_edge value is  "
         << find_aim_obstacle_edge;
  if (!find_aim_obstacle_edge) {
    return std::max(buffer - GetConfig()
                                 .path_bounds_decider_config()
                                 .dynamic_obs_process_config()
                                 .dynamic_reserve_solution_space(),
                    0.0);
  }
  ADEBUG << " find opposite path bounds' obstacle edge : "
         << obstacle_edges[aim_obstacle_edge_index].obstacle_id;

  // solve buffer.
  static constexpr double scale_buffer = 1.0;
  bool is_scale = false;  // if path bound is already occupied in aim
                          // obstacle, is_scale will be false.
  if (is_left_obstacle) {
    double new_aim_obstacle_edge =
        obstacle_edges[aim_obstacle_edge_index].obstacle_edge_l_max +
        GetBufferBetweenADCCenterAndEdge();
    is_scale = (new_aim_obstacle_edge + scale_buffer) <
               std::get<1>(path_boundary[aim_path_boundary_index]);
  } else {
    double new_aim_obstacle_edge =
        obstacle_edges[aim_obstacle_edge_index].obstacle_edge_l_min -
        GetBufferBetweenADCCenterAndEdge();
    is_scale = std::get<2>(path_boundary[aim_path_boundary_index]) <
               (new_aim_obstacle_edge - scale_buffer);
  }
  const double delta_s =
      obstacle_edges[aim_obstacle_edge_index].obstacle_edge_start_s -
      obstacle_edges[obstacle_edge_index].obstacle_edge_start_s;
  bool scale = IsScaleBufferByOppositeObstacles(
      obstacle_edge_index, aim_obstacle_edge_index, obstacle_edges);
  ADEBUG << "path bound is [ " << is_scale << " ] scale."
         << " both obstacles delta_s: " << delta_s << " scale: " << scale;

  static constexpr double kMaxS = 60.0;
  if (delta_s > kMaxS || is_scale || scale) {
    return std::max(buffer - GetConfig()
                                 .path_bounds_decider_config()
                                 .dynamic_obs_process_config()
                                 .dynamic_reserve_solution_space(),
                    0.0);
  }
  return 0.0;
}

bool ObsDynamicProcessor::IsScaleBufferByOppositeObstacles(
    const std::size_t current_obstacle_edge_index,
    const std::size_t opposite_obstacle_edge_index,
    const std::vector<ObstacleEdge>& obstacle_edges) const {
  if (obstacle_edges.empty() ||
      current_obstacle_edge_index > obstacle_edges.size() - 1 ||
      opposite_obstacle_edge_index > obstacle_edges.size() - 1) {
    AERROR << "obstacle_edges.empty() " << obstacle_edges.empty()
           << " current_obstacle_edge_index: " << current_obstacle_edge_index
           << " opposite_obstacle_edge_index: " << opposite_obstacle_edge_index;
    return true;
  }

  const std::string& current_obs_id =
      obstacle_edges[current_obstacle_edge_index].obstacle_id;
  const std::string& opposite_obs_id =
      obstacle_edges[opposite_obstacle_edge_index].obstacle_id;
  double current_obs_end_s = std::numeric_limits<double>::infinity();
  double opposite_obs_start_s = std::numeric_limits<double>::infinity();
  for (const auto& obstacle_edge : obstacle_edges) {
    if (obstacle_edge.obstacle_id == current_obs_id) {
      current_obs_end_s = obstacle_edge.obstacle_edge_end_s;
      continue;
    }

    if (obstacle_edge.obstacle_id == opposite_obs_id) {
      opposite_obs_start_s = obstacle_edge.obstacle_edge_start_s;
    }
  }
  ADEBUG << "current_obs_id: " << current_obs_id
         << " current_obs_end_s: " << current_obs_end_s
         << " opposite_obs_start_s: " << opposite_obs_start_s;
  return current_obs_end_s <
         opposite_obs_start_s -
             GetProcessBound()->GetReferenceLineInfo()->GetCruiseSpeed();
}

void ObsDynamicProcessor::DebugInfo(
    const std::vector<ObstacleEdge>* const obstacle_edges,
    const std::string* const obs_id,
    const PathBound* const origin_path_boundaries,
    const PathBound* const path_boundaries) {
  if ((obstacle_edges == nullptr) || (obs_id == nullptr) ||
      (path_boundaries == nullptr) || (origin_path_boundaries == nullptr)) {
    AERROR << "DebugInfo nullptr check is failed!";
    return;
  }
  if (obstacle_edges->empty()) {
    ADEBUG << "there is no edges to process.";
    return;
  }
  if (FLAGS_enable_path_bound_debug) {
    DynamicObstacleConstraintDebug dynamic_obstacle_constraint_debug;
    for (const auto& obse : *obstacle_edges) {
      auto* const obs_edges =
          dynamic_obstacle_constraint_debug.add_obstacle_edges();
      obs_edges->set_lower_s(obse.obstacle_edge_start_s);
      obs_edges->set_l_min(obse.obstacle_edge_l_min);
      obs_edges->set_l_max(obse.obstacle_edge_l_max);
      obs_edges->set_upper_s(obse.obstacle_edge_end_s);
      obs_edges->set_id(obse.obstacle_id);
    }
  }
  ADEBUG << "dynamic_obstacle_constraint_start_time: " << FIXED
         << SETPRECISION(3) << TL::common::Clock::NowInSeconds();
  ADEBUG << "dynamic obstacle edges " << obstacle_edges->size()
         << ", path blocked by dynamic obs id: " << *obs_id;
  const std::size_t max_num = 1000;
  for (std::size_t i = 0; i < std::min(origin_path_boundaries->size(), max_num);
       ++i) {
    ADEBUG << FIXED << SETPRECISION(3) << "origin_path_current_s: "
           << std::get<0>((*origin_path_boundaries)[i])
           << " origin_path_right_bound: "
           << std::get<1>((*origin_path_boundaries)[i])
           << " origin_path_left_bound: "
           << std::get<2>((*origin_path_boundaries)[i]);
  }

  for (std::size_t i = 0; i < std::min(path_boundaries->size(), max_num); ++i) {
    ADEBUG << FIXED << SETPRECISION(3)
           << "path current s: " << std::get<0>((*path_boundaries)[i])
           << " path right bound: " << std::get<1>((*path_boundaries)[i])
           << " path left bound: " << std::get<2>((*path_boundaries)[i]);
  }
  ADEBUG << "dynamic_obstacle_constraint_path_end";
  if (FLAGS_enable_path_bound_debug) {
    DynamicObstacleConstraintDebug dynamic_obstacle_constraint_debug;
    PathBoundDebugInfo(dynamic_obstacle_constraint_debug,
                       origin_path_boundaries, path_boundaries);
  }
}

void ObsDynamicProcessor::RecordElapseTimeInfo(
    ReferenceLineInfo* const reference_line_info, const std::string& name,
    const double time_diff_ms) {
  if (!FLAGS_enable_record_debug) {
    ADEBUG << "Skip record debug info";
    return;
  }
  if (reference_line_info == nullptr) {
    AERROR << "Reference line info is null.";
    return;
  }
#if 0
  auto ptr_latency_stats = reference_line_info->mutable_latency_stats();

  auto ptr_stats = ptr_latency_stats->add_task_stats();
  ptr_stats->set_name(name);
  ptr_stats->set_time_ms(time_diff_ms);
#endif
  ADEBUG << FIXED << SETPRECISION(3) << name
         << ", elapse_time: " << time_diff_ms;
}

void ObsDynamicProcessor::PathBoundDebugInfo(
    DynamicObstacleConstraintDebug dynamic_obstacle_constraint_debug,
    const PathBound* const origin_path_bound,
    const PathBound* const path_bound) {
  for (auto ori_path_bound_point : *origin_path_bound) {
    auto* const ori_bound =
        dynamic_obstacle_constraint_debug.add_origin_path_boundaries();
    ori_bound->set_s(std::get<0>(ori_path_bound_point));
    ori_bound->set_l_min(std::get<1>(ori_path_bound_point));
    ori_bound->set_l_max(std::get<2>(ori_path_bound_point));
  }
  for (auto path_bound_point : *path_bound) {
    auto* const bound_out =
        dynamic_obstacle_constraint_debug.add_path_boundaries();
    bound_out->set_s(std::get<0>(path_bound_point));
    bound_out->set_l_min(std::get<1>(path_bound_point));
    bound_out->set_l_max(std::get<2>(path_bound_point));
  }
}

void ObsDynamicProcessor::TowingPointsBigCarProcess(
    const std::pair<double, double>& cur_obs_edges,
    const std::string& obstacle_id, PathBound* const path_boundaries,
    TowingPointsInfo* const towing_points, bool is_bigcar_left_nudge,
    double delta_v, const double dynamic_obs_expect_towing_l) {
  if (path_boundaries == nullptr || towing_points == nullptr) {
    AERROR << "path_boundaries or towing_points is nullptr.";
    return;
  }
  if (path_boundaries->empty() || towing_points->empty()) {
    AERROR << "path_boundaries or towing_points is empty.";
    return;
  }

  // first:forward_towing_prepare_distance, second:backward_towing_prepare_distance
  std::pair<double, double> towing_prepare_distance = {0.0, 0.0};
  // double forward_towing_prepare_distance = 0.0;
  // double backward_towing_prepare_distance = 0.0;
  if (GetProcessBound()
          ->GetObsTowingConf()
          .use_ttc_calculate_towing_prepare_distance()) {
    towing_prepare_distance.first =
        std::max(GetProcessBound()
                         ->GetObsTowingConf()
                         .big_car_towing_forward_prepare_time() *
                     delta_v,
                 GetProcessBound()
                     ->GetObsTowingConf()
                     .big_car_towing_forward_prepare_distance());
    towing_prepare_distance.second =
        std::max(GetProcessBound()
                         ->GetObsTowingConf()
                         .big_car_towing_backward_prepare_time() *
                     delta_v,
                 GetProcessBound()
                     ->GetObsTowingConf()
                     .big_car_towing_backward_prepare_distance());
    // UseTtcCalculateTowingPrepareDistance(reference_line_info,
    //                                     &towing_prepare_distance, false);
  } else {
    towing_prepare_distance.first =
        GetProcessBound()
            ->GetObsTowingConf()
            .big_car_towing_forward_prepare_distance();
    towing_prepare_distance.second =
        cur_obs_edges.first - std::get<0>(path_boundaries->at(0));
  }
  ADEBUG << "towing_forward_prepare_distance = "
         << towing_prepare_distance.first
         << ", towing_backward_prepare_distance = "
         << towing_prepare_distance.second
         << ", path_boundaries size = " << path_boundaries->size()
         << ", path_boundaries start s = "
         << std::get<0>(path_boundaries->at(0))
         << ", cur_obstacle start s = " << cur_obs_edges.first
         << ", cur_obstacle_end_s = " << cur_obs_edges.second
         << ", delta_v = " << delta_v << ", PathBoundsDeciderResolution = "
         << GetProcessBound()
                ->GetReferenceLineInfo()
                ->PathBoundsDeciderResolution();

  SetTowingPoints(towing_prepare_distance, cur_obs_edges, obstacle_id,
                  GetProcessBound()->GetMutableReferenceLineInfo(),
                  path_boundaries, towing_points, dynamic_obs_expect_towing_l,
                  is_bigcar_left_nudge);
}

void ObsDynamicProcessor::NudgeBigCarFilterProcess(
    std::vector<ObstacleEdge>* const obstacle_edges, int big_car_idx) {
  if (obstacle_edges == nullptr || obstacle_edges->empty()) {
    AERROR << "obstacle_edges is nullptr or empty.";
    return;
  }
  // 1.longitude filter
  const double big_car_lower_s =
      obstacle_edges->at(big_car_idx).obstacle_edge_start_s;
  std::string big_car_id = obstacle_edges->at(big_car_idx).obstacle_id;
  const double forward_diff_s =
      std::abs(big_car_lower_s - planning_start_point_sl_.s());
  // big car is forward
  if (big_car_lower_s > planning_start_point_sl_.s()) {
    if (forward_diff_s <=
        GetConfig()
                .path_bounds_decider_config()
                .dynamic_obs_process_config()
                .nudge_big_car_s_filter_distance_to_adc() +
            common::VehicleConfigHelper::GetConfig().vehicle_param().length()) {
      // std::get<6>(obstacle_edges->at(big_car_idx)) = true;
    } else {
      ADEBUG << "big_car_id: " << big_car_id
             << "end_s is before adc and beyond considering area";
      return;
    }
  } else {
    if (forward_diff_s >= GetConfig()
                              .path_bounds_decider_config()
                              .dynamic_obs_process_config()
                              .nudge_big_car_s_filter_distance_to_adc()) {
      ADEBUG << "big_car_id: " << big_car_id
             << " end_s is before adc and beyond considering area";
      return;
    }
    // std::get<6>(obstacle_edges->at(big_car_idx)) = true;
  }
  const double big_car_upper_s =
      obstacle_edges->at(big_car_idx).obstacle_edge_end_s;
  const double diff_s =
      std::abs(big_car_upper_s - planning_start_point_sl_.s());
  // big car is backward
  if (big_car_upper_s <= planning_start_point_sl_.s()) {
    if (diff_s <= GetConfig()
                      .path_bounds_decider_config()
                      .dynamic_obs_process_config()
                      .nudge_big_car_s_filter_distance_to_adc()) {
      // std::get<6>(obstacle_edges->at(big_car_idx)) = true;
    } else {
      ADEBUG << "big_car_id: " << big_car_id
             << "is behind adc and beyond considering area";
      return;
    }
  } else {
    if (diff_s >
        GetConfig()
                .path_bounds_decider_config()
                .dynamic_obs_process_config()
                .nudge_big_car_s_filter_distance_to_adc() +
            common::VehicleConfigHelper::GetConfig().vehicle_param().length()) {
      ADEBUG << "big_car_id: " << big_car_id
             << "is behind adc and beyond considering area";
      return;
    }
    // std::get<6>(obstacle_edges->at(big_car_idx)) = true;
  }

  // 2.filter big car parallel on both sides
  const double base_big_car_s =
      obstacle_edges->at(big_car_idx).obstacle_edge_start_s;
  const double base_big_car_l_min =
      obstacle_edges->at(big_car_idx).obstacle_edge_l_min;
  const double base_big_car_l_max =
      obstacle_edges->at(big_car_idx).obstacle_edge_l_max;
  const std::string& base_big_car_id =
      obstacle_edges->at(big_car_idx).obstacle_id;
  double base_big_car_filter_length =
      CalculateFilterLength(obstacle_edges->at(big_car_idx));
  // left
  if (base_big_car_l_max > 0 && base_big_car_l_min > 0) {
    int j = 0;
    while (j < static_cast<int>(obstacle_edges->size()) &&
           obstacle_edges->at(j).obstacle_edge_l_min <= 0 &&
           obstacle_edges->at(j).obstacle_edge_l_max <= 0) {
      double another_side_s = obstacle_edges->at(j).obstacle_edge_start_s;
      double another_side_filter_length =
          CalculateFilterLength(obstacle_edges->at(j));
      if (((base_big_car_s - base_big_car_filter_length) >
           (another_side_s + another_side_filter_length)) ||
          ((base_big_car_s + base_big_car_filter_length) <
           (another_side_s - another_side_filter_length))) {
        ADEBUG << "base_big_car_id: " << base_big_car_id
               << "is not overlaped with " << obstacle_edges->at(j).obstacle_id;
        j++;
        continue;
      }
      ADEBUG << "base_big_car_id: " << base_big_car_id << "is overlaped with "
             << obstacle_edges->at(j).obstacle_id;
      // std::get<6>(obstacle_edges->at(big_car_idx)) = false;
      return;
    }
  } else {
    int j = 0;
    while (j < static_cast<int>(obstacle_edges->size()) &&
           obstacle_edges->at(j).obstacle_edge_l_min > 0 &&
           obstacle_edges->at(j).obstacle_edge_l_max > 0) {
      double another_side_s = obstacle_edges->at(j).obstacle_edge_start_s;
      double another_side_filter_length =
          CalculateFilterLength(obstacle_edges->at(j));
      if (((base_big_car_s - base_big_car_filter_length) >
           (another_side_s + another_side_filter_length)) ||
          ((base_big_car_s + base_big_car_filter_length) <
           (another_side_s - another_side_filter_length))) {
        ADEBUG << "base_big_car_id: " << base_big_car_id
               << "is not overlaped with " << obstacle_edges->at(j).obstacle_id;
        j++;
        continue;
      }
      ADEBUG << "base_big_car_id: " << base_big_car_id
             << "is not overlaped with " << obstacle_edges->at(j).obstacle_id;
      // std::get<6>(obstacle_edges->at(big_car_idx)) = false;
      return;
    }
  }
}

double ObsDynamicProcessor::CalculateFilterLength(
    const ObstacleEdge& obstacle_edge) {
  std::string start_obs_id = obstacle_edge.obstacle_id;
  std::string naked_start_obs_id = " ";
  auto nPos = start_obs_id.find('-');
  if (nPos != std::string::npos) {
    naked_start_obs_id = start_obs_id.substr(0, nPos);
  } else {
    naked_start_obs_id = start_obs_id;
  }

  const auto* curr_obstacle = GetProcessBound()
                                  ->GetReferenceLineInfo()
                                  ->path_decision()
                                  .obstacles()
                                  .Find(naked_start_obs_id);
  double curr_obstacle_v = curr_obstacle->speed();
  double curr_obstacle_length = curr_obstacle->PerceptionBoundingBox().length();
  double filter_t = 2;
  double filter_length =
      std::max(std::max(curr_obstacle_length,
                        (curr_obstacle_length + curr_obstacle_v * filter_t)),
               GetConfig()
                   .path_bounds_decider_config()
                   .dynamic_obs_process_config()
                   .both_sides_dynamic_obs_filter_distance());
  return filter_length;
}

bool ObsDynamicProcessor::CheckIfNudgeObstacle(const Obstacle& obstacle) {
  for (const auto& decider_tag : obstacle.decider_tags()) {
    if (GetProcessBound()
            ->GetObsTowingConf()
            .use_obstacle_decider_process_dynamic_obstacle()) {
      if (absl::StrContains(decider_tag, "dynamic-left-nudge") ||
          absl::StrContains(decider_tag, "dynamic-right-nudge")) {
        return true;
      }

      if (FLAGS_use_caution_dynamic_obs_nudge &&
          absl::StrContains(decider_tag, "caution")) {
        if (GetProcessBound() == nullptr ||
            GetProcessBound()->GetReferenceLineInfo() == nullptr) {
          continue;
        }

        const auto& sl_boundary = obstacle.PerceptionSLBoundary();
        // filter caution obstacle behind adc
        if (Compare(GetProcessBound()->adc_frenet_s_, sl_boundary.end_s()) >
            0) {
          return false;
        }

        // filter ego lane obstacle
        double lane_left_width = 0.0;
        double lane_right_width = 0.0;
        GetProcessBound()
            ->GetReferenceLineInfo()
            ->reference_line()
            .GetLaneWidth((sl_boundary.start_s() + sl_boundary.end_s()) * 0.5,
                          &lane_left_width, &lane_right_width);
        const auto center_l =
            (sl_boundary.start_l() + sl_boundary.end_l()) * 0.5;

        if (center_l > lane_left_width || center_l < -lane_right_width) {
          return true;
        }
      }
    } else if (absl::StrContains(decider_tag, "bigcar")) {
      return true;
    }
  }
  return false;
}

}  // namespace planning
}  // namespace TL
