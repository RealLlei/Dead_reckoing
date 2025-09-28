/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file obstacle_intention_processor.cc
 **/

#include "planning/tasks/deciders/st_bounds_decider/obstacle_intention_processor.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <unordered_set>
#include <utility>
#include <vector>

#include "absl/strings/match.h"
#include "common/file/log.h"
#include "common/math/double_type.h"
#include "common/math/math_utils.h"
#include "common/math/vec2d.h"
#include "common/util/util.h"
#include "planning/common/obstacle.h"
#include "planning/common/util/common.h"

#include "proto/common/pnc_point.pb.h"
#include "proto/fsm/function_manager.pb.h"
#include "proto/fsm/nnp_fct.pb.h"
#include "proto/planning/decision.pb.h"
#include "proto/routing/routing.pb.h"

namespace TL {
namespace planning {

using TL::common::math::double_type::DefinitelyGreater;
using TL::common::math::double_type::DefinitelyLess;

bool ObstacleIntentionProcessor::AddMergeIntentionObstacle(
    ReferenceLineInfo* reference_line_info, const Obstacle& origin_obstacle,
    const LateralIntention& lateral_intention,
    const LongitudinalIntention& longitudinal_intention) {
  const auto intention_obstacle_id = origin_obstacle.Id() + "_intention";
  Obstacle* intention_obstacle =
      reference_line_info->path_decision()->AddEmptyObstacleById(
          intention_obstacle_id);
  if (intention_obstacle == nullptr) {
    return false;
  }

  *intention_obstacle = origin_obstacle;
  intention_obstacle->SetId(intention_obstacle_id);
  intention_obstacle->SetLateralIntention(lateral_intention);
  intention_obstacle->SetLongitudinalIntention(longitudinal_intention);

  double accel = 0.0;
  double jerk = 0.0;
  if (longitudinal_intention == LongitudinalIntention::OVERTAKE) {
    accel = -2.0;
    jerk = -1.0;

  } else if (longitudinal_intention == LongitudinalIntention::YIELD) {
    accel = 1.0;
    jerk = 1.0;
  } else {
    return true;
  }

  auto* intention_trajectory = intention_obstacle->GetMutableTrajectory();
  if (intention_trajectory == nullptr) {
    return false;
  }

  const auto& origin_trajectory_points =
      origin_obstacle.Trajectory().trajectory_point();
  if (origin_trajectory_points.empty()) {
    return false;
  }

  std::vector<double> origin_accumulated_s;
  origin_accumulated_s.reserve(origin_trajectory_points.size());
  origin_accumulated_s.emplace_back(0.0);
  for (int i = 1; i < origin_trajectory_points.size(); ++i) {
    const auto& prev_point = origin_trajectory_points.at(i - 1).path_point();
    const auto& cur_point = origin_trajectory_points.at(i).path_point();
    origin_accumulated_s.emplace_back(
        origin_accumulated_s.back() +
        std::hypot(cur_point.x() - prev_point.x(),
                   cur_point.y() - prev_point.y()));
  }

  auto* intention_trajectory_points =
      intention_trajectory->mutable_trajectory_point();
  if (intention_trajectory_points == nullptr) {
    return false;
  }
  intention_trajectory_points->Clear();

  constexpr int time_count = 80;
  constexpr double time_unit = 0.1;
  std::vector<std::vector<double>> vec_vec_state;
  const auto& origin_start_point = origin_trajectory_points.at(0);
  if (DefinitelyGreater(origin_start_point.a(), accel)) {
    TL::planning::util::GetStateAtMinJerk(
        origin_start_point.v(), origin_start_point.a(), 0.0,
        std::numeric_limits<double>::max(), accel, -fabs(jerk),
        time_count * time_unit, time_unit, 0, &vec_vec_state);
  } else if (DefinitelyLess(origin_start_point.a(), accel)) {
    TL::planning::util::GetStateAtMaxJerk(
        origin_start_point.v(), origin_start_point.a(), 0.0,
        std::numeric_limits<double>::max(), accel, fabs(jerk),
        time_count * time_unit, time_unit, 0, &vec_vec_state);
  } else {
    TL::planning::util::GetStateAtMaxJerk(
        origin_start_point.v(), origin_start_point.a(), 0.0,
        std::numeric_limits<double>::max(), accel, 0.0, time_count * time_unit,
        time_unit, 0, &vec_vec_state);
  }

  if (vec_vec_state.size() < 4) {
    return false;
  }

  const auto& vec_s = vec_vec_state.at(1);
  const auto& vec_v = vec_vec_state.at(2);
  const auto& vec_a = vec_vec_state.at(3);
  const auto count =
      common::math::Clamp(static_cast<int>(vec_s.size()), 0, time_count);
  int next_point_index = 0;
  int i = 0;
  for (; i < count; ++i) {
    const auto t = i * time_unit;
    const auto s = vec_s.at(i);
    if (s < origin_accumulated_s.front()) {
      continue;
    }

    if (s > origin_accumulated_s.back()) {
      break;
    }
    while (next_point_index + 1 < origin_accumulated_s.size() &&
           origin_accumulated_s.at(next_point_index) < s) {
      ++next_point_index;
    }

    const auto& prev_point_index =
        next_point_index == 0 ? 0 : next_point_index - 1;
    const auto ds = origin_accumulated_s.at(next_point_index) -
                    origin_accumulated_s.at(prev_point_index);
    const auto ratio =
        common::math::double_type::IsZero(ds)
            ? 0.0
            : (s - origin_accumulated_s.at(prev_point_index)) / ds;
    const auto& origin_prev_trajectory_point =
        origin_trajectory_points.at(prev_point_index);
    const auto& origin_next_trajectory_point =
        origin_trajectory_points.at(next_point_index);

    auto* intention_trajectory_point = intention_trajectory_points->Add();
    auto* intention_path_point =
        intention_trajectory_point->mutable_path_point();
    intention_trajectory_point->set_relative_time(t);
    intention_trajectory_point->set_v(vec_v.at(i));
    intention_trajectory_point->set_a(vec_a.at(i));
    intention_path_point->set_s(s);
    intention_path_point->set_x(
        (1 - ratio) * origin_prev_trajectory_point.path_point().x() +
        ratio * origin_next_trajectory_point.path_point().x());
    intention_path_point->set_y(
        (1 - ratio) * origin_prev_trajectory_point.path_point().y() +
        ratio * origin_next_trajectory_point.path_point().y());
    intention_path_point->set_theta(
        (1 - ratio) * origin_prev_trajectory_point.path_point().theta() +
        ratio * origin_next_trajectory_point.path_point().theta());
  }

  if (i == 0) {
    return true;
  }

  const auto& origin_end_point =
      origin_trajectory_points.at(origin_trajectory_points.size() - 1);
  common::SLPoint origin_end_sl_point;
  reference_line_info->reference_line().XYToSL(
      {origin_end_point.path_point().x(), origin_end_point.path_point().y()},
      &origin_end_sl_point);
  common::math::Vec2d intention_xy_point;
  common::SLPoint intention_sl_point;
  for (; i < count; ++i) {
    const auto t = i * time_unit;
    const auto s = vec_s.at(i);

    intention_sl_point.set_s((s - origin_accumulated_s.back()) +
                             origin_end_sl_point.s());
    intention_sl_point.set_l(origin_end_sl_point.l());

    const auto reference_point =
        reference_line_info->reference_line().map_path().GetSmoothPoint(
            intention_sl_point.s());

    reference_line_info->reference_line().SLToXY(intention_sl_point,
                                                 &intention_xy_point);

    auto* intention_trajectory_point = intention_trajectory_points->Add();
    auto* intention_path_point =
        intention_trajectory_point->mutable_path_point();
    intention_trajectory_point->set_relative_time(t);
    intention_trajectory_point->set_v(vec_v.at(i));
    intention_trajectory_point->set_a(vec_a.at(i));
    intention_path_point->set_s(s);
    intention_path_point->set_x(intention_xy_point.x());
    intention_path_point->set_y(intention_xy_point.y());
    intention_path_point->set_theta(reference_point.heading());
  }

  intention_obstacle->InitTrajectoryBoundingBox();
  return true;
}

bool ObstacleIntentionProcessor::AddCutinIntentionObstacle(
    ReferenceLineInfo* const reference_line_info,
    const Obstacle& origin_obstacle, const LateralIntention& lateral_intention,
    const LongitudinalIntention& longitudinal_intention, double end_l) {
  if (reference_line_info == nullptr) {
    return false;
  }

  const auto intention_obstacle_id =
      std::to_string(origin_obstacle.PerceptionId()) + "_intention";
  Obstacle* intention_obstacle =
      reference_line_info->path_decision()->AddEmptyObstacleById(
          intention_obstacle_id);
  if (intention_obstacle == nullptr) {
    return false;
  }

  *intention_obstacle = origin_obstacle;
  intention_obstacle->SetId(intention_obstacle_id);
  intention_obstacle->SetLateralIntention(lateral_intention);
  intention_obstacle->SetLongitudinalIntention(longitudinal_intention);

  auto* intention_trajectory = intention_obstacle->GetMutableTrajectory();
  if (intention_trajectory == nullptr) {
    return false;
  }

  ADEBUG << "id:" << origin_obstacle.PerceptionId()
         << ", id:" << intention_obstacle->Id();

  const auto& perception_obstacle = origin_obstacle.Perception();
  common::math::Vec2d cutin_start_xy_point(perception_obstacle.position().x(),
                                           perception_obstacle.position().y());
  SLPoint cutin_start_sl_point;
  reference_line_info->reference_line().XYToSL(cutin_start_xy_point,
                                               &cutin_start_sl_point);
  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  reference_line_info->reference_line().GetLaneWidth(
      cutin_start_sl_point.s(), &lane_left_width, &lane_right_width);
  const auto lane_width = (lane_left_width + lane_right_width);

  auto* intention_trajectory_points =
      intention_trajectory->mutable_trajectory_point();
  if (intention_trajectory_points == nullptr) {
    return false;
  }
  intention_trajectory_points->Clear();

  const auto kCutinTotalTime = 5.0;
  const auto kTrajectoryPointInterval = 0.1;
  const auto kTrajectoryPointCount = 80;

  auto cutin_finish_time =
      kCutinTotalTime * fabs(end_l - cutin_start_sl_point.l()) / lane_width;

  ADEBUG << "lane_width:" << lane_width << ", l:" << cutin_start_sl_point.l();

  const auto v = std::hypot(perception_obstacle.velocity().x(),
                            perception_obstacle.velocity().y());
  SLPoint cutin_finish_sl_point;
  cutin_finish_sl_point.set_s(cutin_start_sl_point.s() +
                              fabs(v * cutin_finish_time));
  cutin_finish_sl_point.set_l(end_l);
  common::math::Vec2d cutin_finish_xy_point;
  reference_line_info->reference_line().SLToXYForGreaterThanRefMaxS(
      cutin_finish_sl_point, &cutin_finish_xy_point);
  const auto cutin_finish_distance =
      std::hypot(cutin_finish_xy_point.x() - cutin_start_xy_point.x(),
                 cutin_finish_xy_point.y() - cutin_start_xy_point.y());
  cutin_finish_time = cutin_finish_distance / fmax(v, 0.1);

  ADEBUG << "cutin_finish_distance:" << cutin_finish_distance
         << ", cutin_finish_time:" << cutin_finish_time;

  const auto cutin_angle =
      std::atan2(cutin_finish_xy_point.y() - cutin_start_xy_point.y(),
                 cutin_finish_xy_point.x() - cutin_start_xy_point.x());
  const auto forward_angle = reference_line_info->reference_line()
                                 .map_path()
                                 .GetSmoothPoint(cutin_finish_sl_point.s())
                                 .heading();

  int i = 0;
  for (; i < kTrajectoryPointCount; ++i) {
    const auto t = i * kTrajectoryPointInterval;
    if (t > cutin_finish_time) {
      break;
    }

    auto* intention_trajectory_point = intention_trajectory_points->Add();
    if (intention_trajectory_point == nullptr) {
      return false;
    }
    auto* intention_path_point =
        intention_trajectory_point->mutable_path_point();
    if (intention_path_point == nullptr) {
      return false;
    }

    const auto ratio = t / cutin_finish_time;
    intention_trajectory_point->mutable_path_point()->set_s(t * v);
    intention_trajectory_point->set_v(v);
    intention_trajectory_point->set_relative_time(t);
    intention_path_point->set_x((1 - ratio) * cutin_start_xy_point.x() +
                                ratio * cutin_finish_xy_point.x());
    intention_path_point->set_y((1 - ratio) * cutin_start_xy_point.y() +
                                ratio * cutin_finish_xy_point.y());
    intention_path_point->set_theta(cutin_angle);
  }

  common::math::Vec2d xy_point;
  SLPoint sl_point;
  for (; i < kTrajectoryPointCount; ++i) {
    const auto t = i * kTrajectoryPointInterval;
    sl_point.set_s(cutin_finish_sl_point.s() + v * (t - cutin_finish_time));
    sl_point.set_l(end_l);

    auto* intention_trajectory_point = intention_trajectory_points->Add();
    if (intention_trajectory_point == nullptr) {
      return false;
    }
    auto* intention_path_point =
        intention_trajectory_point->mutable_path_point();
    if (intention_path_point == nullptr) {
      return false;
    }

    reference_line_info->reference_line().SLToXY(sl_point, &xy_point);

    intention_trajectory_point->mutable_path_point()->set_s(t * v);
    intention_trajectory_point->set_v(v);
    intention_trajectory_point->set_relative_time(t);
    intention_path_point->set_x(xy_point.x());
    intention_path_point->set_y(xy_point.y());
    intention_path_point->set_theta(forward_angle);
  }

  intention_obstacle->InitTrajectoryBoundingBox();
  return true;
}

bool ObstacleIntentionProcessor::IsReverseObstacle(
    const ReferenceLineInfo& reference_line_info,
    const Obstacle& origin_obstacle) {
  const auto& origin_trajectory_points =
      origin_obstacle.Trajectory().trajectory_point();
  if (origin_trajectory_points.empty()) {
    return false;
  }

  // reverse obstacle don't need cut in intention
  const auto& ego_vehicle_state = reference_line_info.vehicle_state();
  const auto ego_moving_heading = common::math::NormalizeAngle(
      ego_vehicle_state.gear() == soc::Chassis::GEAR_REVERSE
          ? ego_vehicle_state.heading() + M_PI
          : ego_vehicle_state.heading());
  const auto obstacle_start_moving_heading =
      origin_trajectory_points.size() > 1
          ? origin_trajectory_points.at(1).path_point().theta()
          : origin_trajectory_points.at(0).path_point().theta();
  const auto start_heading_diff = fabs(common::math::NormalizeAngle(
      ego_moving_heading - obstacle_start_moving_heading));
  static constexpr double kHeadingDiffThreshold1st = 0.833 * M_PI;
  return (start_heading_diff > kHeadingDiffThreshold1st);
}

bool ObstacleIntentionProcessor::AddCutinPredictionProObstacle(
    ReferenceLineInfo* const reference_line_info,
    const Obstacle& origin_obstacle, const LateralIntention& lateral_intention,
    const LongitudinalIntention& longitudinal_intention, double end_l) {
  if (reference_line_info == nullptr ||
      origin_obstacle.Trajectory().trajectory_point().size() < 2) {
    return false;
  }
  const double pro = pow(origin_obstacle.Trajectory().probability(), 2);
  const auto& trajectory_points =
      origin_obstacle.Trajectory().trajectory_point();

  if (GetPrintDebug()) {
    ADEBUG << "obs_id" << origin_obstacle.Id();
    for (const auto& old_tra : trajectory_points) {
      common::math::Vec2d cutin_start_xy_point(old_tra.path_point().x(),
                                               old_tra.path_point().y());
      SLPoint cutin_start_sl_point;
      reference_line_info->reference_line().XYToSL(cutin_start_xy_point,
                                                   &cutin_start_sl_point);
      ADEBUG << "old_tra.s" << cutin_start_sl_point.s();
      ADEBUG << "old_tra.l" << cutin_start_sl_point.l();
      ADEBUG << "old_tra.theta" << old_tra.path_point().theta();
    }
  }

  const auto intention_obstacle_id =
      std::to_string(origin_obstacle.PerceptionId()) + "_intention";
  Obstacle* intention_obstacle =
      reference_line_info->path_decision()->AddEmptyObstacleById(
          intention_obstacle_id);
  if (intention_obstacle == nullptr) {
    return false;
  }

  *intention_obstacle = origin_obstacle;
  intention_obstacle->SetId(intention_obstacle_id);
  intention_obstacle->SetLateralIntention(lateral_intention);
  intention_obstacle->SetLongitudinalIntention(longitudinal_intention);

  auto* intention_trajectory = intention_obstacle->GetMutableTrajectory();
  if (intention_trajectory == nullptr) {
    return false;
  }
  const auto& frist_path_point = trajectory_points.at(1).path_point();
  const auto& cutin_v = trajectory_points.at(1).v();
  common::math::Vec2d cutin_start_xy_point(
      trajectory_points.at(0).path_point().x(),
      trajectory_points.at(0).path_point().y());
  SLPoint cutin_start_sl_point;
  reference_line_info->reference_line().XYToSL(cutin_start_xy_point,
                                               &cutin_start_sl_point);
  const auto obs_locate_ref_heading =
      reference_line_info->reference_line()
          .GetNearestReferencePoint(
              (origin_obstacle.PerceptionSLBoundary().start_s() +
               origin_obstacle.PerceptionSLBoundary().end_s()) /
              2.0)
          .heading();
  const auto theta_diff =
      fabs(frist_path_point.theta() - obs_locate_ref_heading);

  const auto initial_cutin_angle = M_PI / 2 - theta_diff * pro;

  const auto& cutin_pro_vx = fmax(cutin_v * cos(initial_cutin_angle), 0.00001);
  const auto& cutin_pro_vy = fmax(cutin_v * sin(initial_cutin_angle), 0.00001);
  // 根据切入概率更改切入角度
  const auto cutin_angle =
      common::math::NormalizeAngle(atan(cutin_pro_vy / cutin_pro_vx));

  const auto cutin_finish_distance =
      fabs(cutin_start_sl_point.l()) / cos(cutin_angle);

  const auto v = std::hypot(cutin_pro_vx, cutin_pro_vy);
  const auto cutin_finish_time = cutin_finish_distance / fmax(v, 0.1);

  auto* intention_trajectory_points =
      intention_trajectory->mutable_trajectory_point();
  if (intention_trajectory_points == nullptr) {
    return false;
  }
  intention_trajectory_points->Clear();

  const auto kTrajectoryPointInterval = 0.1;
  const auto kTrajectoryPointCount = 80;

  SLPoint cutin_finish_sl_point;
  cutin_finish_sl_point.set_s(cutin_start_sl_point.s() +
                              fabs(v * cutin_finish_time));
  cutin_finish_sl_point.set_l(end_l);
  common::math::Vec2d cutin_finish_xy_point;
  reference_line_info->reference_line().SLToXYForGreaterThanRefMaxS(
      cutin_finish_sl_point, &cutin_finish_xy_point);
  const auto theta =
      std::atan2(cutin_finish_xy_point.y() - cutin_start_xy_point.y(),
                 cutin_finish_xy_point.x() - cutin_start_xy_point.x());

  const auto forward_angle = reference_line_info->reference_line()
                                 .map_path()
                                 .GetSmoothPoint(cutin_finish_sl_point.s())
                                 .heading();

  int i = 0;
  for (; i < kTrajectoryPointCount; ++i) {
    const auto t = i * kTrajectoryPointInterval;
    if (t > cutin_finish_time) {
      break;
    }

    auto* intention_trajectory_point = intention_trajectory_points->Add();
    if (intention_trajectory_point == nullptr) {
      return false;
    }
    auto* intention_path_point =
        intention_trajectory_point->mutable_path_point();
    if (intention_path_point == nullptr) {
      return false;
    }

    const auto ratio = t / cutin_finish_time;
    intention_trajectory_point->mutable_path_point()->set_s(t * v);
    intention_trajectory_point->set_v(v);
    intention_trajectory_point->set_relative_time(t);
    intention_path_point->set_x((1 - ratio) * cutin_start_xy_point.x() +
                                ratio * cutin_finish_xy_point.x());
    intention_path_point->set_y((1 - ratio) * cutin_start_xy_point.y() +
                                ratio * cutin_finish_xy_point.y());
    intention_path_point->set_theta(theta);
  }

  common::math::Vec2d xy_point;
  SLPoint sl_point;
  for (; i < kTrajectoryPointCount; ++i) {
    const auto t = i * kTrajectoryPointInterval;
    sl_point.set_s(cutin_finish_sl_point.s() + v * (t - cutin_finish_time));
    sl_point.set_l(end_l);

    auto* intention_trajectory_point = intention_trajectory_points->Add();
    if (intention_trajectory_point == nullptr) {
      return false;
    }
    auto* intention_path_point =
        intention_trajectory_point->mutable_path_point();
    if (intention_path_point == nullptr) {
      return false;
    }

    reference_line_info->reference_line().SLToXY(sl_point, &xy_point);

    intention_trajectory_point->mutable_path_point()->set_s(t * v);
    intention_trajectory_point->set_v(v);
    intention_trajectory_point->set_relative_time(t);
    intention_path_point->set_x(xy_point.x());
    intention_path_point->set_y(xy_point.y());
    intention_path_point->set_theta(forward_angle);
  }
  if (GetPrintDebug()) {
    ADEBUG << "theta_diff" << theta_diff;
    ADEBUG << "cutin_angle" << initial_cutin_angle;
    ADEBUG << "change_cutin_angle" << cutin_angle;
    ADEBUG << " cutin_finish_distance" << cutin_finish_distance;
    ADEBUG << "cutin_finish_time" << cutin_finish_time;
    for (const auto& tra :
         intention_obstacle->Trajectory().trajectory_point()) {
      common::math::Vec2d cutin_start_xy_point(tra.path_point().x(),
                                               tra.path_point().y());
      SLPoint cutin_start_sl_point;
      reference_line_info->reference_line().XYToSL(cutin_start_xy_point,
                                                   &cutin_start_sl_point);
      ADEBUG << "tra.s" << cutin_start_sl_point.s();
      ADEBUG << "tra.l" << cutin_start_sl_point.l();
    }
  }

  intention_obstacle->InitTrajectoryBoundingBox();

  return true;
}

void ObstacleIntentionProcessor::ProcessMergeIntention(
    ReferenceLineInfo* const reference_line_info) {
  for (const auto& obs_behavior : reference_line_info->GetAdcObsBehavior()) {
    auto* origin_obstacle =
        reference_line_info->path_decision()->Find(obs_behavior.first);
    if (origin_obstacle == nullptr ||
        origin_obstacle->Trajectory().trajectory_point().empty()) {
      continue;
    }

    if (near_merge_point_obstacles_.count(origin_obstacle->PerceptionId()) <=
        0) {
      const auto& trajectory_envelope =
          origin_obstacle->GetTrajectoryEnvelope();
      if (trajectory_envelope.empty()) {
        continue;
      }
      const auto& last_envelope = trajectory_envelope.back();
      const auto max_s = fmax(last_envelope.low_left_p.s(),
                              fmax(last_envelope.low_right_p.s(),
                                   fmax(last_envelope.upper_left_p.s(),
                                        last_envelope.upper_right_p.s())));

      const auto& adc_map_common_info =
          reference_line_info->reference_line().GetAdcMapCommonInfo();
      if (max_s < adc_map_common_info.dis_first_merge_begin_point - 5.0) {
        continue;
      }
    }

    near_merge_point_obstacles_.insert(origin_obstacle->PerceptionId());

    if (obs_behavior.second.first.has_overtake()) {
      AddMergeIntentionObstacle(reference_line_info, *origin_obstacle,
                                LateralIntention::MERGE,
                                LongitudinalIntention::OVERTAKE);
    } else if (obs_behavior.second.first.has_yield()) {
      AddMergeIntentionObstacle(reference_line_info, *origin_obstacle,
                                LateralIntention::MERGE,
                                LongitudinalIntention::YIELD);
    } else {
      AddMergeIntentionObstacle(reference_line_info, *origin_obstacle,
                                LateralIntention::MERGE,
                                LongitudinalIntention::UNKNOWN);
    }
  }

  const auto& obstacles =
      reference_line_info->path_decision()->obstacles().Items();

  for (auto it = near_merge_point_obstacles_.begin();
       it != near_merge_point_obstacles_.end();) {
    bool flag = false;
    for (const auto* obstacle : obstacles) {
      if (obstacle != nullptr && obstacle->PerceptionId() == *it) {
        flag = true;
        break;
      }
    }

    if (flag) {
      ++it;
    } else {
      it = near_merge_point_obstacles_.erase(it);
    }
  }

  // near_merge_point_obstacles_.erase(iter, near_merge_point_obstacles_.end());
}

void ObstacleIntentionProcessor::ProcessCutinIntention(
    const Frame& frame, ReferenceLineInfo* const reference_line_info) {
  for (const auto& [id, intention] : frame.GetObsGameIntention()) {
    if (intention == GameObstacleIntention::Undefine) {
      continue;
    }

    const Obstacle* origin_obstacle = nullptr;
    for (const auto* obstacle :
         reference_line_info->path_decision()->obstacles().Items()) {
      if (obstacle->PerceptionId() == id) {
        origin_obstacle = obstacle;
        break;
      }
    }
    if (origin_obstacle == nullptr ||
        origin_obstacle->Trajectory().trajectory_point().empty() ||
        origin_obstacle->LongitudinalDecision().has_ignore()) {
      continue;
    }
    if (IsReverseObstacle(*reference_line_info, *origin_obstacle)) {
      continue;
    }

    const auto& origin_sl_boundary = origin_obstacle->PerceptionSLBoundary();
    if (intention == GameObstacleIntention::Cutin) {
      if (origin_obstacle->LongitudinalDecision().has_overtake()) {
        continue;
      }
      if (origin_obstacle->LongitudinalDecision().has_yield()) {
        AddCutinIntentionObstacle(reference_line_info, *origin_obstacle,
                                  LateralIntention::CUTIN,
                                  LongitudinalIntention::YIELD, 0.0);
      } else {
        ADEBUG << "id:" << origin_obstacle->Id()
               << ", cutin, but has no lon intention";
      }

    } else {
      const auto end_l =
          ((origin_sl_boundary.start_l() + origin_sl_boundary.end_l()) / 2.0 >
                   0.0
               ? 3.75
               : -3.75);
      if (origin_obstacle->LongitudinalDecision().has_overtake()) {
        AddCutinIntentionObstacle(reference_line_info, *origin_obstacle,
                                  LateralIntention::ALONGSIDE,
                                  LongitudinalIntention::OVERTAKE, end_l);
      } else if (origin_obstacle->LongitudinalDecision().has_yield()) {
        AddCutinPredictionProObstacle(reference_line_info, *origin_obstacle,
                                      LateralIntention::ALONGSIDE,
                                      LongitudinalIntention::YIELD, 0.0);
      } else {
        AddCutinPredictionProObstacle(reference_line_info, *origin_obstacle,
                                      LateralIntention::ALONGSIDE,
                                      LongitudinalIntention::YIELD, 0.0);
        ADEBUG << "id:" << origin_obstacle->Id()
               << ", alongside, but has no lon intention";
      }
    }
  }
}

void ObstacleIntentionProcessor::CheckIfCrossObstacles(
    const Frame& frame, ReferenceLineInfo* reference_line_info) {
  if (reference_line_info == nullptr ||
      (frame.local_view().HasFunctionManagerIn() &&
       frame.local_view().GetFunctionManagerIn()->ta_pilot_mode() ==
           functionmanager::AVP)) {
    return;
  }

  for (const auto* obstacle :
       reference_line_info->path_decision()->obstacles().Items()) {
    if (obstacle == nullptr) {
      continue;
    }
    const auto& trajectory_points = obstacle->Trajectory().trajectory_point();
    const auto& trajectory_envelope = obstacle->GetTrajectoryEnvelope();
    if (trajectory_points.empty() || trajectory_envelope.empty() ||
        trajectory_points.size() != trajectory_envelope.size()) {
      continue;
    }

    const auto iter = std::min_element(
        trajectory_envelope.begin(), trajectory_envelope.end(),
        [](const auto& point_description1, const auto& point_description2) {
          return fabs(point_description1.center_p.l()) <
                 fabs(point_description2.center_p.l());
        });
    if (iter == trajectory_envelope.end()) {
      continue;
    }

    const auto& trajectory_point =
        trajectory_points.at(std::distance(trajectory_envelope.begin(), iter));

    auto* mutable_obstacle =
        reference_line_info->path_decision()->Find(obstacle->Id());
    if (mutable_obstacle == nullptr) {
      continue;
    }

    const auto angle_diff =
        common::math::AngleDiff(reference_line_info->reference_line()
                                    .map_path()
                                    .GetSmoothPoint(iter->center_p.s())
                                    .heading(),
                                trajectory_point.path_point().theta());
    mutable_obstacle->SetIsCrossObstacle(fabs(angle_diff) > M_PI * 0.33 &&
                                         fabs(angle_diff) < M_PI * 0.9);
    if (mutable_obstacle->GetIsCrossObstacle()) {
      AERROR << "id:" << mutable_obstacle->Id() << ", cross";
    }
  }
}

void ObstacleIntentionProcessor::ProcessCrossIntention(
    const Frame& frame, ReferenceLineInfo* reference_line_info) {
  if (reference_line_info == nullptr) {
    return;
  }

  CheckIfCrossObstacles(frame, reference_line_info);

  const auto& adc_sl_boundary = reference_line_info->AdcSlBoundary();
  const auto& planning_start_point = frame.PlanningStartPoint();
  const auto v = planning_start_point.v();
  const auto a = planning_start_point.a();
  const auto time_diff =
      frame.local_view().GetPredictionObstacles()->header().data_stamp() -
      (frame.vehicle_state().timestamp() +
       frame.PlanningStartPoint().relative_time());
  auto stop_distance = std::numeric_limits<double>::max();
  auto s_buffer = 1.0;
  for (const auto* obstacle :
       reference_line_info->path_decision()->obstacles().Items()) {
    if (obstacle == nullptr || !obstacle->LongitudinalDecision().has_stop()) {
      continue;
    }
    if (obstacle->PerceptionSLBoundary().start_s() < stop_distance) {
      stop_distance =
          std::min(stop_distance, obstacle->PerceptionSLBoundary().start_s());
      s_buffer = obstacle->PerceptionSLBoundary().end_s() -
                 obstacle->PerceptionSLBoundary().start_s();
    }
  }
  for (const auto* obstacle :
       reference_line_info->path_decision()->obstacles().Items()) {
    if (obstacle == nullptr || !obstacle->GetIsCrossObstacle()) {
      continue;
    }

    LongitudinalIntention longitudinal_intention = LongitudinalIntention::YIELD;
    const auto& trajectory_envelope = obstacle->GetTrajectoryEnvelope();
    const auto& trajectory_points = obstacle->Trajectory().trajectory_point();
    if (trajectory_envelope.size() <= 1 ||
        trajectory_envelope.size() != trajectory_points.size()) {
      continue;
    }

    const auto cross_direction =
        trajectory_envelope.back().center_p.l() >
                trajectory_envelope.front().center_p.l()
            ? CrossDirection::FROM_RIGHT_TO_LEFT
            : CrossDirection::FROM_LEFT_TO_RIGHT;

    int index = 0;
    double prev_distance_to_lane = 0.0;
    for (int i = 0; i < trajectory_envelope.size(); ++i) {
      const auto& envelope = trajectory_envelope.at(i);

      double lane_left_width = 0.0;
      double lane_right_width = 0.0;
      reference_line_info->reference_line().GetLaneWidth(
          envelope.center_p.s(), &lane_left_width, &lane_right_width);

      const auto l_min =
          fmin(fmin(fmin(envelope.low_left_p.l(), envelope.low_right_p.l()),
                    envelope.upper_left_p.l()),
               envelope.upper_right_p.l());
      const auto l_max =
          fmax(fmax(fmin(envelope.low_left_p.l(), envelope.low_right_p.l()),
                    envelope.upper_left_p.l()),
               envelope.upper_right_p.l());

      CrossPosition cross_position = CrossPosition::UNKNOWN;
      double distance_to_lane = 0.0;
      if (l_min > lane_left_width) {
        cross_position = CrossPosition::ON_LEFT;
        distance_to_lane = l_min - lane_left_width;
      } else if (l_max < -lane_right_width) {
        cross_position = CrossPosition::ON_RIGHT;
        distance_to_lane = -lane_right_width - l_max;
      } else {
        cross_position = CrossPosition::ON_LANE;
        distance_to_lane = 0.0;
      }

      if ((cross_direction == CrossDirection::FROM_LEFT_TO_RIGHT &&
           cross_position != CrossPosition::ON_LEFT) ||
          (cross_direction == CrossDirection::FROM_RIGHT_TO_LEFT &&
           cross_position != CrossPosition::ON_RIGHT)) {
        index = i;
        break;
      }
      prev_distance_to_lane = distance_to_lane;
    }

    if (index == 0) {
      longitudinal_intention = LongitudinalIntention::YIELD;
      const auto& envelope = trajectory_envelope.at(index);

      auto current_s_min =
          fmin(fmin(fmin(envelope.low_left_p.s(), envelope.low_right_p.s()),
                    envelope.upper_left_p.s()),
               envelope.upper_right_p.s());
      if (current_s_min > stop_distance + s_buffer) {
        auto* mutable_obstacle =
            reference_line_info->path_decision()->Find(obstacle->Id());
        if (mutable_obstacle == nullptr) {
          continue;
        }

        ObjectDecisionType ignore_decision;
        ignore_decision.mutable_ignore();
        mutable_obstacle->AddLongitudinalDecision(
            "obstacle_intention_processor", ignore_decision);
        continue;
      }
      break;
    }

    const auto& prev_envelope = trajectory_envelope.at(index - 1);
    const auto& prev_trajectory_point = trajectory_points.at(index - 1);
    const auto dtheta = prev_trajectory_point.path_point().theta() -
                        reference_line_info->reference_line()
                            .map_path()
                            .GetSmoothPoint(prev_envelope.center_p.s())
                            .heading();
    const auto dt = prev_distance_to_lane /
                    fmax(fabs(prev_trajectory_point.v() * sin(dtheta)), 0.1);
    const auto ds = prev_trajectory_point.v() * cos(dtheta) * dt;
    auto current_s_min = fmin(fmin(fmin(prev_envelope.low_left_p.s(),
                                        prev_envelope.low_right_p.s()),
                                   prev_envelope.upper_left_p.s()),
                              prev_envelope.upper_right_p.s()) +
                         ds;
    if (current_s_min > stop_distance + s_buffer) {
      auto* mutable_obstacle =
          reference_line_info->path_decision()->Find(obstacle->Id());
      if (mutable_obstacle == nullptr) {
        continue;
      }

      ObjectDecisionType ignore_decision;
      ignore_decision.mutable_ignore();
      mutable_obstacle->AddLongitudinalDecision("obstacle_intention_processor",
                                                ignore_decision);
      continue;
    }
    double obs_reach_cross_point_time =
        prev_trajectory_point.relative_time() + dt + time_diff;

    double adc_reach_cross_point_time = 0.0;
    if (current_s_min > adc_sl_boundary.end_s()) {
      const auto s = current_s_min - adc_sl_boundary.end_s();
      const auto temp = v * v + 2.0 * a * s;
      if (temp > 0.0) {
        const auto temp_sqrt = sqrt(temp);
        adc_reach_cross_point_time = fmin(fmax((-v + temp_sqrt) / a, 0.0),
                                          fmax((-v - temp_sqrt) / a, 0.0));
      } else {
        adc_reach_cross_point_time = std::numeric_limits<double>::max();
      }
    }

    ADEBUG << "id:" << obstacle->Id()
           << ", obs_reach_cross_point_time:" << obs_reach_cross_point_time
           << ", adc_reach_cross_point_time:" << adc_reach_cross_point_time;

    if (adc_reach_cross_point_time < obs_reach_cross_point_time - 1.0 ||
        adc_reach_cross_point_time < obs_reach_cross_point_time * 0.8) {
      longitudinal_intention = LongitudinalIntention::OVERTAKE;
    }

    if (longitudinal_intention != LongitudinalIntention::OVERTAKE) {
      continue;
    }

    auto* mutable_obstacle =
        reference_line_info->path_decision()->Find(obstacle->Id());
    if (mutable_obstacle == nullptr) {
      continue;
    }

    ObjectDecisionType ignore_decision;
    ignore_decision.mutable_ignore();
    mutable_obstacle->AddLongitudinalDecision("obstacle_intention_processor",
                                              ignore_decision);
  }
}

void ObstacleIntentionProcessor::Process(
    const Frame& frame, ReferenceLineInfo* const reference_line_info) {
  ProcessMergeIntention(reference_line_info);
  ProcessCutinIntention(frame, reference_line_info);
  ProcessCrossIntention(frame, reference_line_info);
}

}  // namespace planning
}  // namespace TL
