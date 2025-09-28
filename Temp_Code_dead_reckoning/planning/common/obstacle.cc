/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "planning/common/obstacle.h"

#include <algorithm>
#include <cstddef>
#include <deque>
#include <functional>
#include <iostream>
#include <utility>
#include <vector>

#include "common/configs/vehicle_config_helper.h"
#include "common/file/log.h"
#include "common/math/double_type.h"
#include "common/math/linear_interpolation.h"
#include "common/math/vec2d.h"
#include "common/util/map_util.h"
#include "common/util/string_util.h"
#include "common/util/util.h"
#include "planning/common/planning_gflags.h"
#include "planning/common/speed/st_boundary.h"
#include "proto/perception/perception_obstacle.pb.h"
#include "proto/prediction/prediction_obstacle.pb.h"

namespace TL {
namespace planning {

using TL::common::VehicleConfigHelper;
using TL::common::util::FindOrDie;
using TL::perception::PerceptionObstacle;
using TL::prediction::ObstaclePriority;

namespace {
const double kStBoundaryDeltaS = 0.2;  // meters
const double kSampleEpsilon = 0.8;
constexpr double kNormalStopDistanceForSmallCar = 2.5;
constexpr double kNormalStopDistanceForBigCar = 3.5;
constexpr double kVelocityEpsilon = 0.1;

// constexpr double kObsMinSpeed = 0.01;
// constexpr double kStaticReverseObs = -2.0;
}  // namespace

void SparseTrajectory(const PerceptionObstacle& perception_obstacle,
                      const prediction::Trajectory& prediction_trajectory,
                      prediction::Trajectory* sparse_prediction_trajectory) {
  if (sparse_prediction_trajectory == nullptr) {
    return;
  }
  if (prediction_trajectory.trajectory_point_size() <= 2) {
    sparse_prediction_trajectory->CopyFrom(prediction_trajectory);
    return;
  }
  const auto length = perception_obstacle.length();
  const auto width = perception_obstacle.width();
  const auto theta = perception_obstacle.theta();
  // length along the direction of velocity
  auto sample_dis = abs(length * sin(theta)) + abs(width * cos(theta));
  sample_dis = sample_dis * kSampleEpsilon;

  double cut_time = FLAGS_cut_vehicle_trajectory_time_length;
  if (perception_obstacle.type() == PerceptionObstacle::BICYCLE ||
      perception_obstacle.type() == PerceptionObstacle::CYCLIST) {
    cut_time = FLAGS_cut_bicycle_trajectory_time_length;
  }

  double accumulated_s = 0.0;
  for (int i = 0; i < prediction_trajectory.trajectory_point_size(); ++i) {
    const auto& point = prediction_trajectory.trajectory_point(i);
    if (i == 0) {
      sparse_prediction_trajectory->add_trajectory_point()->CopyFrom(point);
      continue;
    }
    const auto& prev_point = prediction_trajectory.trajectory_point(i - 1);
    double dx = point.path_point().x() - prev_point.path_point().x();
    double dy = point.path_point().y() - prev_point.path_point().y();
    double ds = std::sqrt(dx * dx + dy * dy);
    accumulated_s += ds;
    double dt = point.relative_time() - prev_point.relative_time();
    if (dt < FLAGS_trajectory_time_resolution / 2) {
      continue;
    }
    if (FLAGS_enable_cut_trajectory && point.relative_time() > cut_time) {
      sparse_prediction_trajectory->add_trajectory_point()->CopyFrom(point);
      break;
    }
    if (accumulated_s >= sample_dis ||
        (prev_point.v() > kVelocityEpsilon && point.v() < kVelocityEpsilon) ||
        i == prediction_trajectory.trajectory_point_size() - 1) {
      sparse_prediction_trajectory->add_trajectory_point()->CopyFrom(point);
      accumulated_s = 0.0;
    }
  }
  if (prediction_trajectory.has_probability()) {
    sparse_prediction_trajectory->set_probability(
        prediction_trajectory.probability());
  }
}

// NOLINTBEGIN
const std::unordered_map<ObjectDecisionType::ObjectTagCase, int,
                         Obstacle::ObjectTagCaseHash>
    Obstacle::s_longitudinal_decision_safety_sorter_ = {
        {ObjectDecisionType::kIgnore, 0},
        {ObjectDecisionType::kCaution, 1},
        {ObjectDecisionType::kOvertake, 100},
        {ObjectDecisionType::kFollow, 300},
        {ObjectDecisionType::kYield, 400},
        {ObjectDecisionType::kLonNudge, 500},
        {ObjectDecisionType::kStop, 600}};

const std::unordered_map<ObjectDecisionType::ObjectTagCase, int,
                         Obstacle::ObjectTagCaseHash>
    Obstacle::s_lateral_decision_safety_sorter_ = {
        {ObjectDecisionType::kIgnore, 0},
        {ObjectDecisionType::kCaution, 1},
        {ObjectDecisionType::kNudge, 100}};

// NOLINTEND
Obstacle::Obstacle(
    const std::string& id,
    const perception::PerceptionObstacle& perception_obstacle,
    const prediction::ObstaclePriority::Priority& obstacle_priority,
    const prediction::ObstacleIntent::Type& obstacle_intent, bool is_static,
    prediction::TrackStatus tracking)
    : id_(id),
      perception_id_(perception_obstacle.id()),
      perception_obstacle_(perception_obstacle),
      perception_bounding_box_({perception_obstacle_.position().x(),
                                perception_obstacle_.position().y()},
                               perception_obstacle_.theta(),
                               perception_obstacle_.length(),
                               perception_obstacle_.width()),
      is_caution_level_obstacle_(obstacle_priority ==
                                 ObstaclePriority::CAUTION),
      priority_(obstacle_priority),
      intent_(obstacle_intent),
      track_status_(std::move(tracking)) {
  std::vector<common::math::Vec2d> polygon_points;
  if (perception_obstacle.polygon_point_size() <= 2) {
    perception_bounding_box_.GetAllCorners(&polygon_points);
  } else {
    ACHECK(perception_obstacle.polygon_point_size() > 2)
        << "object " << id << "has less than 3 polygon points";
    for (const auto& point : perception_obstacle.polygon_point()) {
      polygon_points.emplace_back(point.x(), point.y());
    }
  }
  ACHECK(common::math::Polygon2d::ComputeConvexHull(polygon_points,
                                                    &perception_polygon_))
      << "object[" << id << "] polygon is not a valid convex hull.\n"
      << perception_obstacle.DebugString();
  // deal with small static obstacles
#ifdef FOR_BAIDU_SIMULATION
  static constexpr double kUsePpolygonMinAreaFilterRatio = 0.1;
  if (perception_obstacle.type() ==
      TL::perception::PerceptionObstacle::UNKNOWN) {
    if (perception_polygon_.area() / perception_bounding_box_.area() <
        kUsePpolygonMinAreaFilterRatio) {
      perception_bounding_box_ = perception_polygon_.MinAreaBoundingBox();
    }
  }
#endif

  is_static_ = (is_static || (!FLAGS_enable_planning_self_simulator &&
                              obstacle_priority == ObstaclePriority::IGNORE));
  is_static_without_ignore_ = is_static;
  is_virtual_ = (perception_obstacle.id() < 0);
  is_low_height_ = IsObstacleWithLowHeight(perception_obstacle);
  is_vulnerable_alive_ = IsObstacleVulnerableAlive(perception_obstacle);
  speed_ = std::hypot(perception_obstacle.velocity().x(),
                      perception_obstacle.velocity().y());

  InitTrajectoryBoundingBox();
}

Obstacle::Obstacle(const std::string& id,
                   const PerceptionObstacle& perception_obstacle,
                   const prediction::Trajectory& trajectory,
                   const ObstaclePriority::Priority& obstacle_priority,
                   const prediction::ObstacleIntent::Type& obstacle_intent,
                   const bool is_static,
                   const prediction::TrackStatus& tracking)
    : Obstacle(id, perception_obstacle, obstacle_priority, obstacle_intent,
               is_static, tracking) {
  SparseTrajectory(perception_obstacle, trajectory, &trajectory_);
  auto& trajectory_points = *trajectory_.mutable_trajectory_point();
  double cumulative_s = 0.0;
  if (!trajectory_points.empty()) {
    trajectory_points[0].mutable_path_point()->set_s(0.0);
  }
  for (int i = 1; i < trajectory_points.size(); ++i) {
    const auto& prev = trajectory_points[i - 1];
    const auto& cur = trajectory_points[i];
    if (prev.relative_time() >= cur.relative_time()) {
      AERROR << "prediction time is not increasing."
             << "current point: " << cur.ShortDebugString()
             << "previous point: " << prev.ShortDebugString();
    }
    cumulative_s +=
        TL::common::util::DistanceXY(prev.path_point(), cur.path_point());
    trajectory_points[i].mutable_path_point()->set_s(cumulative_s);
  }

  InitTrajectoryBoundingBox();
}

common::TrajectoryPoint Obstacle::GetPointAtTime(
    const double relative_time) const {
  const auto& points = trajectory_.trajectory_point();
  if (points.size() < 2) {
    common::TrajectoryPoint point;
    point.mutable_path_point()->set_x(perception_obstacle_.position().x());
    point.mutable_path_point()->set_y(perception_obstacle_.position().y());
    point.mutable_path_point()->set_z(perception_obstacle_.position().z());
    point.mutable_path_point()->set_theta(perception_obstacle_.theta());
    point.mutable_path_point()->set_s(0.0);
    point.mutable_path_point()->set_kappa(0.0);
    point.mutable_path_point()->set_dkappa(0.0);
    point.mutable_path_point()->set_ddkappa(0.0);
    point.set_v(0.0);
    point.set_a(0.0);
    point.set_relative_time(0.0);
    return point;
  }
  auto comp = [](const common::TrajectoryPoint& p, const double time) {
    return p.relative_time() < time;
  };

  auto it_lower =
      std::lower_bound(points.begin(), points.end(), relative_time, comp);

  if (it_lower == points.begin()) {
    return *points.begin();
  }
  if (it_lower == points.end()) {
    return *points.rbegin();
  }
  return common::math::InterpolateUsingLinearApproximation(
      *(it_lower - 1), *it_lower, relative_time);
}

common::math::Box2d Obstacle::GetBoundingBox(
    const common::TrajectoryPoint& point) const {
  return {{point.path_point().x(), point.path_point().y()},
          point.path_point().theta(),
          perception_obstacle_.length(),
          perception_obstacle_.width()};
}

bool Obstacle::IsValidPerceptionObstacle(const PerceptionObstacle& obstacle) {
  if (obstacle.length() <= 0.0) {
    AERROR << "invalid obstacle length:" << obstacle.length();
    return false;
  }
  if (obstacle.width() <= 0.0) {
    AERROR << "invalid obstacle width:" << obstacle.width();
    return false;
  }
  if (obstacle.height() <= 0.0) {
    AERROR << "invalid obstacle height:" << obstacle.height();
    return false;
  }
  if (obstacle.has_velocity()) {
    if (std::isnan(obstacle.velocity().x()) ||
        std::isnan(obstacle.velocity().y())) {
      AERROR << "invalid obstacle velocity:"
             << obstacle.velocity().DebugString();
      return false;
    }
  }
  return std::all_of(obstacle.polygon_point().cbegin(),
                     obstacle.polygon_point().cend(),
                     [](const TL::common::Point3D& pt) {
                       return (!std::isnan(pt.x()) && !std::isnan(pt.y()));
                     });
  // for (const auto& pt : obstacle.polygon_point()) {
  //   if (std::isnan(pt.x()) || std::isnan(pt.y())) {
  //     AERROR << "invalid obstacle polygon point:" << pt.DebugString();
  //     return false;
  //   }
  // }
  return true;
}

std::list<std::shared_ptr<Obstacle>> Obstacle::CreateObstacles(
    const prediction::PredictionObstacles& predictions) {
  std::list<std::shared_ptr<Obstacle>> obstacles;
  std::mutex create_obstacle_mutex;
  auto lamCreateObstacle = [&](auto& prediction_obstacle) {
    if (prediction_obstacle.perception_obstacle().id() < 0) {
      return;
    }
    if (!IsValidPerceptionObstacle(prediction_obstacle.perception_obstacle())) {
      AERROR << "Invalid perception obstacle: "
             << prediction_obstacle.perception_obstacle().DebugString();
      return;
    }
    const auto perception_id =
        std::to_string(prediction_obstacle.perception_obstacle().id());
    // NOLINTBEGIN
    if (prediction_obstacle.trajectory().empty()) {
      auto obstacle_ptr = new Obstacle(
          perception_id, prediction_obstacle.perception_obstacle(),
          prediction_obstacle.priority().priority(),
          prediction_obstacle.intent().type(), prediction_obstacle.is_static(),
          prediction_obstacle.track_status());
      // NOLINTEND
      {
        std::lock_guard<std::mutex> lock(create_obstacle_mutex);
        obstacles.emplace_back(obstacle_ptr);
      }
      return;
    }
    // if (!prediction_obstacle.trajectory().empty() &&
    //     prediction_obstacle.perception_obstacle().has_velocity_flu() &&
    //     prediction_obstacle.perception_obstacle().velocity_flu().x() <
    //         kObsMinSpeed &&
    //     prediction_obstacle.perception_obstacle().velocity_flu().x() >
    //         kStaticReverseObs) {
    //   // trick 针对orin上出现的静止目标推逆向速度，特殊处理，后期要去掉
    //   // NOLINTBEGIN
    //   auto obstacle_ptr =
    //       new Obstacle(perception_id, prediction_obstacle.perception_obstacle(),
    //                    prediction_obstacle.priority().priority(), true,
    //                    prediction_obstacle.track_status());
    //   // NOLINTEND
    //   {
    //     std::lock_guard<std::mutex> lock(create_obstacle_mutex);
    //     obstacles.emplace_back(obstacle_ptr);
    //   }
    //   return;
    // }

    int trajectory_index = 0;
    for (const auto& trajectory : prediction_obstacle.trajectory()) {
      bool is_valid_trajectory = true;
      for (const auto& point : trajectory.trajectory_point()) {
        if (!IsValidTrajectoryPoint(point)) {
          AERROR << "obj:" << perception_id
                 << " TrajectoryPoint: " << trajectory.ShortDebugString()
                 << " is NOT valid.";
          is_valid_trajectory = false;
          break;
        }
      }
      if (!is_valid_trajectory) {
        return;
      }

      const std::string obstacle_id =
          absl::StrCat(perception_id, "_", trajectory_index);
      // NOLINTBEGIN
      auto obstacle_ptr = new Obstacle(
          obstacle_id, prediction_obstacle.perception_obstacle(), trajectory,
          prediction_obstacle.priority().priority(),
          prediction_obstacle.intent().type(), prediction_obstacle.is_static(),
          prediction_obstacle.track_status());
      // NOLINTEND
      {
        std::lock_guard<std::mutex> lock(create_obstacle_mutex);
        obstacles.emplace_back(obstacle_ptr);
      }
      ++trajectory_index;
    }
  };
  TL::common::thread::ThreadPool::Instance()->ForEach(
      predictions.prediction_obstacle().begin(),
      predictions.prediction_obstacle().end(), lamCreateObstacle);
  return obstacles;
}

std::shared_ptr<Obstacle> Obstacle::CreateStaticVirtualObstacles(
    const std::string& id, const common::math::Box2d& obstacle_box) {
  // create a "virtual" perception_obstacle
  perception::PerceptionObstacle perception_obstacle;
  // simulator needs a valid integer
  size_t negative_id = std::hash<std::string>{}(id);
  // set the first bit to 1 so negative_id became negative number
  negative_id |= (0x1U << 31U);
  perception_obstacle.set_id(static_cast<int32_t>(negative_id));
  perception_obstacle.mutable_position()->set_x(obstacle_box.center().x());
  perception_obstacle.mutable_position()->set_y(obstacle_box.center().y());
  perception_obstacle.set_theta(obstacle_box.heading());
  perception_obstacle.mutable_velocity()->set_x(0);
  perception_obstacle.mutable_velocity()->set_y(0);
  perception_obstacle.set_length(obstacle_box.length());
  perception_obstacle.set_width(obstacle_box.width());
  perception_obstacle.set_height(FLAGS_virtual_stop_wall_height);
  perception_obstacle.set_type(
      perception::PerceptionObstacle::UNKNOWN_UNMOVABLE);
  perception_obstacle.set_tracking_time(1.0);

  std::vector<common::math::Vec2d> corner_points;
  obstacle_box.GetAllCorners(&corner_points);
  for (const auto& corner_point : corner_points) {
    auto* point = perception_obstacle.add_polygon_point();
    point->set_x(corner_point.x());
    point->set_y(corner_point.y());
  }
  prediction::TrackStatus track_status;
  track_status.set_is_tracking(false);
  // NOLINTBEGIN
  auto* obstacle =
      new Obstacle(id, perception_obstacle, ObstaclePriority::NORMAL,
                   prediction::ObstacleIntent::UNKNOWN, true, track_status);
  // NOLINTEND
  obstacle->is_virtual_ = true;
  return std::shared_ptr<Obstacle>(obstacle);
}

bool Obstacle::IsValidTrajectoryPoint(const common::TrajectoryPoint& point) {
  return !((!point.has_path_point()) || std::isnan(point.path_point().x()) ||
           std::isnan(point.path_point().y()) ||
           std::isnan(point.path_point().z()) ||
           std::isnan(point.path_point().kappa()) ||
           std::isnan(point.path_point().s()) ||
           std::isnan(point.path_point().dkappa()) ||
           std::isnan(point.path_point().ddkappa()) || std::isnan(point.v()) ||
           std::isnan(point.a()) || std::isnan(point.relative_time()));
}

void Obstacle::SetPerceptionSlBoundary(const SLBoundary& sl_boundary) {
  sl_boundary_ = sl_boundary;
}

double Obstacle::MinRadiusStopDistance(
    const common::VehicleParam& vehicle_param) const {
  if (min_radius_stop_distance_ > 0) {
    return min_radius_stop_distance_;
  }
  static constexpr double stop_distance_buffer = 0.5;
  const double min_turn_radius = VehicleConfigHelper::MinSafeTurnRadius();
  double lateral_diff =
      vehicle_param.width() / 2.0 + std::max(std::fabs(sl_boundary_.start_l()),
                                             std::fabs(sl_boundary_.end_l()));
  const double kEpison = 1e-5;
  lateral_diff = std::min(lateral_diff, min_turn_radius - kEpison);
  double stop_distance =
      std::sqrt(std::fabs(min_turn_radius * min_turn_radius -
                          (min_turn_radius - lateral_diff) *
                              (min_turn_radius - lateral_diff))) +
      stop_distance_buffer;
  stop_distance -= vehicle_param.front_edge_to_center();
  stop_distance = std::min(stop_distance, FLAGS_max_stop_distance_obstacle);
  stop_distance = std::max(stop_distance, FLAGS_min_stop_distance_obstacle);
  return stop_distance;
}

double Obstacle::NormalStopDistance() const {
  return IsOversizedVehicle() ? kNormalStopDistanceForBigCar
                              : kNormalStopDistanceForSmallCar;
}

void Obstacle::BuildReferenceLineStBoundary(const ReferenceLine& reference_line,
                                            const double adc_start_s) {
  const auto& adc_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  const double adc_width = adc_param.width();
  if (is_static_ || trajectory_.trajectory_point().empty()) {
    std::vector<std::pair<STPoint, STPoint>> point_pairs;
    double start_s = sl_boundary_.start_s();
    double end_s = sl_boundary_.end_s();
    if (end_s - start_s < kStBoundaryDeltaS) {
      end_s = start_s + kStBoundaryDeltaS;
    }
    if (!reference_line.IsBlockRoad(perception_bounding_box_, adc_width)) {
      return;
    }
    point_pairs.emplace_back(STPoint(start_s - adc_start_s, 0.0),
                             STPoint(end_s - adc_start_s, 0.0));
    point_pairs.emplace_back(STPoint(start_s - adc_start_s, FLAGS_st_max_t),
                             STPoint(end_s - adc_start_s, FLAGS_st_max_t));
    reference_line_st_boundary_ = STBoundary(point_pairs);
  } else {
    if (BuildTrajectoryStBoundary(reference_line, adc_start_s,
                                  &reference_line_st_boundary_)) {
      ADEBUG << "Found st_boundary for obstacle " << id_;
      ADEBUG << "st_boundary: min_t = " << reference_line_st_boundary_.min_t()
             << ", max_t = " << reference_line_st_boundary_.max_t()
             << ", min_s = " << reference_line_st_boundary_.min_s()
             << ", max_s = " << reference_line_st_boundary_.max_s();
    } else {
      ADEBUG << "No st_boundary for obstacle " << id_;
    }
  }
}

void Obstacle::GetLineCrossPointS(const SLPoint& l_p, const SLPoint& r_p,
                                  double adc_width_threshold,
                                  std::vector<double>* const cross_s_ptr) {
  bool p1_f = TL::common::math::double_type::DefinitelyLessEqual(
      fabs(l_p.l()), adc_width_threshold);
  bool p2_f = TL::common::math::double_type::DefinitelyLessEqual(
      fabs(r_p.l()), adc_width_threshold);
  //   ADEBUG << "p1_f:" << p1_f << "  p2_f:" << p2_f << "  l_p_l:" << l_p.l()
  //          << "  r_p_l:" << r_p.l()
  //          << "   adc_wid_threshold:" << adc_width_threshold;
  double m = 0;
  if (p1_f && p2_f) {
    cross_s_ptr->push_back(l_p.s());
    cross_s_ptr->push_back(r_p.s());
  } else if (p1_f && !p2_f) {
    cross_s_ptr->push_back(l_p.s());
    if (r_p.l() > 0) {
      m = adc_width_threshold;
    } else {
      m = -adc_width_threshold;
    }
    cross_s_ptr->push_back(
        (l_p.s() - r_p.s()) * (m - l_p.l()) / (l_p.l() - r_p.l()) + l_p.s());
  } else if (!p1_f && p2_f) {
    cross_s_ptr->push_back(r_p.s());
    if (l_p.l() > 0) {
      m = adc_width_threshold;
    } else {
      m = -adc_width_threshold;
    }
    cross_s_ptr->push_back(
        (l_p.s() - r_p.s()) * (m - l_p.l()) / (l_p.l() - r_p.l()) + l_p.s());
  } else {
    if (TL::common::math::double_type::DefinitelyLess(l_p.l() * r_p.l(),
                                                         0.0)) {
      m = adc_width_threshold;
      cross_s_ptr->push_back(
          (l_p.s() - r_p.s()) * (m - l_p.l()) / (l_p.l() - r_p.l()) + l_p.s());
      m = -adc_width_threshold;
      cross_s_ptr->push_back(
          (l_p.s() - r_p.s()) * (m - l_p.l()) / (l_p.l() - r_p.l()) + l_p.s());
    }
  }
}

ObsPointDescription Obstacle::GetPointByRatio(const ObsPointDescription& p_1,
                                              const ObsPointDescription& p_2,
                                              double ra) {
  auto LookValue = [](double a, double b, double r) {
    return (b - a) * r + a;
  };
  ObsPointDescription point;
  point.time = LookValue(p_1.time, p_2.time, ra);
  point.center_p.set_s(LookValue(p_1.center_p.s(), p_2.center_p.s(), ra));
  point.center_p.set_l(LookValue(p_1.center_p.l(), p_2.center_p.l(), ra));

  point.low_left_p.set_s(LookValue(p_1.low_left_p.s(), p_2.low_left_p.s(), ra));
  point.low_left_p.set_l(LookValue(p_1.low_left_p.l(), p_2.low_left_p.l(), ra));

  point.low_right_p.set_s(
      LookValue(p_1.low_right_p.s(), p_2.low_right_p.s(), ra));
  point.low_right_p.set_l(
      LookValue(p_1.low_right_p.l(), p_2.low_right_p.l(), ra));

  point.upper_left_p.set_s(
      LookValue(p_1.upper_left_p.s(), p_2.upper_left_p.s(), ra));
  point.upper_left_p.set_l(
      LookValue(p_1.upper_left_p.l(), p_2.upper_left_p.l(), ra));

  point.upper_right_p.set_s(
      LookValue(p_1.upper_right_p.s(), p_2.upper_right_p.s(), ra));
  point.upper_right_p.set_l(
      LookValue(p_1.upper_right_p.l(), p_2.upper_right_p.l(), ra));

  return point;
}

bool Obstacle::FindTwoBoxCrossPoint(
    const ObsPointDescription& point_1, const ObsPointDescription& point_2,
    bool is_find_min_s, double adc_width_threshold,
    std::vector<double>* const cross_s_ptr,
    ObsPointDescription* const point_analys_ptr) {
  auto SetSpecS = [&](double* const change_vec_value, bool is_find_min_s) {
    if (cross_s_ptr->size() == 2) {
      if (is_find_min_s) {
        *change_vec_value = fmin(cross_s_ptr->at(0), cross_s_ptr->at(1));
      } else {
        *change_vec_value = fmax(cross_s_ptr->at(0), cross_s_ptr->at(1));
      }
    }
  };

  std::vector<double> specific_s;
  if (is_find_min_s) {
    specific_s.resize(4, 1000);
  } else {
    specific_s.resize(4, -1000);
  }

  cross_s_ptr->clear();
  cross_s_ptr->reserve(2);
  GetLineCrossPointS(point_1.low_left_p, point_2.low_left_p,
                     adc_width_threshold, cross_s_ptr);
  SetSpecS(&specific_s.at(0), is_find_min_s);

  cross_s_ptr->clear();
  cross_s_ptr->reserve(2);
  GetLineCrossPointS(point_1.low_right_p, point_2.low_right_p,
                     adc_width_threshold, cross_s_ptr);
  SetSpecS(&specific_s.at(1), is_find_min_s);

  cross_s_ptr->clear();
  cross_s_ptr->reserve(2);
  GetLineCrossPointS(point_1.upper_left_p, point_2.upper_left_p,
                     adc_width_threshold, cross_s_ptr);
  SetSpecS(&specific_s.at(2), is_find_min_s);

  cross_s_ptr->clear();
  cross_s_ptr->reserve(2);
  GetLineCrossPointS(point_1.upper_right_p, point_2.upper_right_p,
                     adc_width_threshold, cross_s_ptr);
  SetSpecS(&specific_s.at(3), is_find_min_s);

  int idx = -1;
  if (is_find_min_s) {
    idx = static_cast<int>(
        std::min_element(specific_s.begin(), specific_s.end()) -
        specific_s.begin());
  } else {
    idx = static_cast<int>(
        std::max_element(specific_s.begin(), specific_s.end()) -
        specific_s.begin());
  }
  double ratio = 0;
  if (idx == 0) {
    if (TL::common::math::double_type::SeemsEqual(point_1.low_left_p.s(),
                                                     point_2.low_left_p.s())) {
      return false;
    }
    ratio = (specific_s.at(idx) - point_2.low_left_p.s()) /
            (point_1.low_left_p.s() - point_2.low_left_p.s());

  } else if (idx == 1) {
    if (TL::common::math::double_type::SeemsEqual(point_1.low_right_p.s(),
                                                     point_2.low_right_p.s())) {
      return false;
    }
    ratio = (specific_s.at(idx) - point_2.low_right_p.s()) /
            (point_1.low_right_p.s() - point_2.low_right_p.s());

  } else if (idx == 2) {
    if (TL::common::math::double_type::SeemsEqual(
            point_1.upper_left_p.s(), point_2.upper_left_p.s())) {
      return false;
    }
    ratio = (specific_s.at(idx) - point_2.upper_left_p.s()) /
            (point_1.upper_left_p.s() - point_2.upper_left_p.s());

  } else if (idx == 3) {
    if (TL::common::math::double_type::SeemsEqual(
            point_1.upper_right_p.s(), point_2.upper_right_p.s())) {
      return false;
    }
    ratio = (specific_s.at(idx) - point_2.upper_right_p.s()) /
            (point_1.upper_right_p.s() - point_2.upper_right_p.s());

  } else {
    return false;
  }
  if (TL::common::math::double_type::DefinitelyLess(ratio, 0.3)) {
    return false;
  }
  *point_analys_ptr = GetPointByRatio(point_1, point_2, ratio);
  ADEBUG << "ratio:" << ratio
         << "   point_analys_ptr:" << point_analys_ptr->time;
  return true;
}

bool Obstacle::BuildTrajectoryStBoundary(const ReferenceLine& reference_line,
                                         const double adc_start_s,
                                         STBoundary* const st_boundary) {
  ADEBUG << "obs_" << id_ << " build st boundary.";
  if (!IsValidObstacle(perception_obstacle_)) {
    AERROR << "Fail to build trajectory st boundary because object is not "
              "valid. PerceptionObstacle: "
           << perception_obstacle_.DebugString();
    return false;
  }
  if (trajectory_.trajectory_point().empty()) {
    ADEBUG << "obs " << id_ << "  has no trajectory points.";
    return false;
  }
  if (trajectory_envelope_.empty()) {
    ADEBUG << "obs " << id_ << "  has no trajectory envelope.";
    return false;
  }
  const auto& adc_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  const double adc_length = adc_param.length();
  const double adc_width = adc_param.width();
  const double adc_width_threshold =
      adc_width / 2.0 + FLAGS_slack_add_obs_buff_in_ref;

  ADEBUG << "ref line tag [" << reference_line.Tag() << "]"
         << "obs[" << id_ << "] traj_min_l:" << traj_min_l_
         << "  traj_max_l:" << traj_max_l_
         << "   adc_width_threshold:" << adc_width_threshold
         << "  adc_start_s:" << adc_start_s << "  traj_max_s_:" << traj_max_s_;

  if (TL::common::math::double_type::DefinitelyGreater(
          traj_min_l_, adc_width_threshold) ||
      TL::common::math::double_type::DefinitelyLess(traj_max_l_,
                                                       -adc_width_threshold) ||
      traj_max_s_ < adc_start_s) {
    ADEBUG << "obs_" << id_ << " min l error.";
    return false;
  }

  //   ADEBUG << "obs_" << id_ << " build st boundary."
  //          << "  adc_width/2:" << adc_width / 2;
  const auto point_size = trajectory_envelope_.size();
  std::deque<bool> is_collision_array;
  is_collision_array.resize(point_size, false);

  auto IsCollisionWithVeh = [&](const ObsPointDescription& point) {
    double min_l = fmin(fmin(point.low_right_p.l(), point.low_left_p.l()),
                        fmin(point.upper_left_p.l(), point.upper_right_p.l()));
    double max_l = fmax(fmax(point.low_right_p.l(), point.low_left_p.l()),
                        fmax(point.upper_left_p.l(), point.upper_right_p.l()));
    return !(TL::common::math::double_type::DefinitelyGreater(
                 min_l, adc_width_threshold) ||
             TL::common::math::double_type::DefinitelyLess(
                 max_l, -adc_width_threshold));
  };
  for (int i = 0; i < point_size; ++i) {
    is_collision_array[i] = IsCollisionWithVeh(trajectory_envelope_.at(i));
    //     ADEBUG << "idx:" << i << "  is_collision:" << is_collision_array[i]
    //            << "  min_l:"
    //            << fmin(fmin(trajectory_envelope_.at(i).low_right_p.l(),
    //                         trajectory_envelope_.at(i).low_left_p.l()),
    //                    fmin(trajectory_envelope_.at(i).upper_left_p.l(),
    //                         trajectory_envelope_.at(i).upper_right_p.l()))
    //            << "  max_l:"
    //            << fmax(fmax(trajectory_envelope_.at(i).low_right_p.l(),
    //                         trajectory_envelope_.at(i).low_left_p.l()),
    //                    fmax(trajectory_envelope_.at(i).upper_left_p.l(),
    //                         trajectory_envelope_.at(i).upper_right_p.l()));
  }

  const auto& accu_s = reference_line.map_path().accumulated_s();
  STPoint obs_st_lower;
  STPoint obs_st_upper;
  std::vector<STPoint> st_lower_vev;
  std::vector<STPoint> st_upper_vev;
  double s_min = std::numeric_limits<double>::max();
  double s_max = std::numeric_limits<double>::lowest();
  std::vector<double> s_points_in_width;

  auto FindMinMaxSIndex = [&](const ObsPointDescription& obs_p) {
    s_points_in_width.clear();
    s_points_in_width.reserve(8);

    GetLineCrossPointS(obs_p.low_left_p, obs_p.low_right_p, adc_width_threshold,
                       &s_points_in_width);
    GetLineCrossPointS(obs_p.low_right_p, obs_p.upper_right_p,
                       adc_width_threshold, &s_points_in_width);
    GetLineCrossPointS(obs_p.upper_right_p, obs_p.upper_left_p,
                       adc_width_threshold, &s_points_in_width);
    GetLineCrossPointS(obs_p.upper_left_p, obs_p.low_left_p,
                       adc_width_threshold, &s_points_in_width);

    if (s_points_in_width.empty()) {
      return false;
    }
    std::sort(s_points_in_width.begin(), s_points_in_width.end());

    // std::string tms = "";
    // TL::common::util::vec2str(s_points_in_width, &tms);
    // ADEBUG << FIXED << SETPRECISION(4) << "vec_get:" << tms;
    // tms = "";

    s_min = s_points_in_width.front() - adc_length / 2.0;
    s_max = s_points_in_width.back() + adc_length / 2.0;

    if (TL::common::math::double_type::SeemsEqual(s_min, s_max) ||
        TL::common::math::double_type::DefinitelyGreaterEqual(
            s_min, accu_s.back())) {
      return false;
    }
    if (TL::common::math::double_type::DefinitelyGreater(s_max,
                                                            accu_s.back())) {
      s_max = accu_s.back();
    }
    obs_st_lower.set_t(obs_p.time);
    obs_st_lower.set_s(s_min);
    obs_st_upper.set_t(obs_p.time);
    obs_st_upper.set_s(s_max);
    return true;
  };

  st_lower_vev.reserve(point_size);
  st_upper_vev.reserve(point_size);
  ObsPointDescription point_analys;
  if (std::any_of(is_collision_array.begin(), is_collision_array.end(),
                  [](bool t) { return !t; }) &&
      std::any_of(is_collision_array.begin(), is_collision_array.end(),
                  [](bool t) { return t; })) {
    ADEBUG << "both true and false exit.";
  }

  for (int i = 0; i < point_size; ++i) {
    if (i != 0) {
      if (!is_collision_array[i - 1] && !is_collision_array[i]) {
        continue;
      }
      if (!is_collision_array[i - 1] && is_collision_array[i]) {
        if (!FindTwoBoxCrossPoint(
                trajectory_envelope_.at(i - 1), trajectory_envelope_.at(i),
                true, adc_width_threshold, &s_points_in_width, &point_analys)) {
          continue;
        }
      } else if (is_collision_array[i - 1] && !is_collision_array[i]) {
        if (!FindTwoBoxCrossPoint(trajectory_envelope_.at(i - 1),
                                  trajectory_envelope_.at(i), false,
                                  adc_width_threshold, &s_points_in_width,
                                  &point_analys)) {
          continue;
        }
      } else if (is_collision_array[i - 1] && is_collision_array[i]) {
        point_analys = trajectory_envelope_.at(i);
      }
    } else {
      if (is_collision_array[i]) {
        point_analys = trajectory_envelope_.at(i);
      } else {
        continue;
      }
    }
    // ADEBUG << "point_analys:" << point_analys.time;
    if (TL::common::math::double_type::DefinitelyGreaterEqual(
            point_analys.center_p.s(), 0.0) &&
        FindMinMaxSIndex(point_analys)) {
      st_lower_vev.push_back(obs_st_lower);
      st_upper_vev.push_back(obs_st_upper);
    }
  }
  ADEBUG << "obs_" << id_ << " st_lower_vev.size() " << st_lower_vev.size();
  //   for (int i = 0; i < st_lower_vev.size(); ++i) {
  //     ADEBUG << "lower time:" << st_lower_vev.at(i).t()
  //            << "  lower s:" << st_lower_vev.at(i).s()
  //            << "  upper time:" << st_upper_vev.at(i).t()
  //            << "  upper s:" << st_upper_vev.at(i).s();
  //   }
  if (st_lower_vev.empty()) {
    return false;
  }
  *st_boundary = STBoundary::CreateInstance(st_lower_vev, st_upper_vev);
  ADEBUG << "obs_" << id_ << " found st boundary";
  return true;
}

const STBoundary& Obstacle::reference_line_st_boundary() const {
  return reference_line_st_boundary_;
}

const std::vector<std::string>& Obstacle::decider_tags() const {
  return decider_tags_;
}

const std::vector<ObjectDecisionType>& Obstacle::decisions() const {
  return decisions_;
}

bool Obstacle::IsLateralDecision(const ObjectDecisionType& decision) {
  return decision.has_ignore() || decision.has_nudge() ||
         decision.has_caution();
}

bool Obstacle::IsLongitudinalDecision(const ObjectDecisionType& decision) {
  return decision.has_ignore() || decision.has_caution() ||
         decision.has_stop() || decision.has_yield() || decision.has_follow() ||
         decision.has_overtake() || decision.has_lon_nudge();
}

ObjectDecisionType Obstacle::MergeLongitudinalDecision(
    const ObjectDecisionType& lhs, const ObjectDecisionType& rhs) {
  if (lhs.object_tag_case() == ObjectDecisionType::OBJECT_TAG_NOT_SET) {
    return rhs;
  }
  if (rhs.object_tag_case() == ObjectDecisionType::OBJECT_TAG_NOT_SET) {
    return lhs;
  }
  const auto lhs_val =
      FindOrDie(s_longitudinal_decision_safety_sorter_, lhs.object_tag_case());
  const auto rhs_val =
      FindOrDie(s_longitudinal_decision_safety_sorter_, rhs.object_tag_case());
  if (lhs_val < rhs_val) {
    return rhs;
  }
  if (lhs_val > rhs_val) {
    return lhs;
  }
  if (lhs.has_ignore()) {
    return rhs;
  }
  if (lhs.has_stop()) {
    return lhs.stop().distance_s() < rhs.stop().distance_s() ? lhs : rhs;
  }
  if (lhs.has_yield()) {
    return lhs.yield().distance_s() < rhs.yield().distance_s() ? lhs : rhs;
  }
  if (lhs.has_follow()) {
    return lhs.follow().distance_s() < rhs.follow().distance_s() ? lhs : rhs;
  }
  if (lhs.has_overtake()) {
    return lhs.overtake().distance_s() > rhs.overtake().distance_s() ? lhs
                                                                     : rhs;
  }

  return lhs;  // stop compiler complaining
}

const ObjectDecisionType& Obstacle::LateralDecision() const {
  return lateral_decision_;
}

bool Obstacle::IsIgnore() const {
  return IsLongitudinalIgnore() && IsLateralIgnore();
}

bool Obstacle::IsLongitudinalIgnore() const {
  return longitudinal_decision_.has_ignore();
}

bool Obstacle::IsLateralIgnore() const {
  return lateral_decision_.has_ignore();
}

ObjectDecisionType Obstacle::MergeLateralDecision(
    const ObjectDecisionType& lhs, const ObjectDecisionType& rhs) {
  if (lhs.object_tag_case() == ObjectDecisionType::OBJECT_TAG_NOT_SET) {
    return rhs;
  }
  if (rhs.object_tag_case() == ObjectDecisionType::OBJECT_TAG_NOT_SET) {
    return lhs;
  }
  const auto lhs_val =
      FindOrDie(s_lateral_decision_safety_sorter_, lhs.object_tag_case());
  const auto rhs_val =
      FindOrDie(s_lateral_decision_safety_sorter_, rhs.object_tag_case());
  if (lhs_val < rhs_val) {
    return rhs;
  }
  if (lhs_val > rhs_val) {
    return lhs;
  }
  if (lhs.has_ignore()) {
    return rhs;
  }
  if (lhs.has_nudge()) {
    // The following DCHECK would misunderstand the reasonable difference
    // between PathBound and PathDecider, so comment out.
    // DCHECK(lhs.nudge().type() == rhs.nudge().type())
    //      << "could not merge left nudge and right nudge";
    if (lhs.nudge().type() == rhs.nudge().type()) {
      return std::fabs(lhs.nudge().distance_l()) >
                     std::fabs(rhs.nudge().distance_l())
                 ? lhs
                 : rhs;
    }
    AERROR << "lhs is covered by rhs";
    return rhs;
  }
  return lhs;
}

bool Obstacle::HasLateralDecision() const {
  return lateral_decision_.object_tag_case() !=
         ObjectDecisionType::OBJECT_TAG_NOT_SET;
}

bool Obstacle::HasLongitudinalDecision() const {
  return longitudinal_decision_.object_tag_case() !=
         ObjectDecisionType::OBJECT_TAG_NOT_SET;
}

bool Obstacle::HasNonIgnoreDecision() const {
  return (HasLateralDecision() && !IsLateralIgnore()) ||
         (HasLongitudinalDecision() && !IsLongitudinalIgnore());
}

void Obstacle::AddLongitudinalDecision(const std::string& decider_tag,
                                       const ObjectDecisionType& decision) {
  DCHECK(IsLongitudinalDecision(decision))
      << "Decision: " << decision.ShortDebugString()
      << " is not a longitudinal decision";
  longitudinal_decision_ =
      MergeLongitudinalDecision(longitudinal_decision_, decision);
  ADEBUG << decider_tag << " added obstacle " << Id()
         << " longitudinal decision: " << decision.ShortDebugString()
         << ". The merged decision is: "
         << longitudinal_decision_.ShortDebugString();
  decisions_.push_back(decision);
  decider_tags_.push_back(decider_tag);
}

void Obstacle::SetLongitudinalDecision(const std::string& decider_tag,
                                       const ObjectDecisionType& decision) {
  if (!IsLongitudinalDecision(decision)) {
    AERROR << "Decision: " << decision.ShortDebugString()
           << " is not a longitudinal decision";
    return;
  }

  longitudinal_decision_ = decision;
  ADEBUG << decider_tag << " added obstacle " << Id()
         << " longitudinal decision: " << decision.ShortDebugString()
         << ". The merged decision is: "
         << longitudinal_decision_.ShortDebugString();
  decisions_.push_back(decision);
  decider_tags_.push_back(decider_tag);
}

void Obstacle::AddLateralDecision(const std::string& decider_tag,
                                  const ObjectDecisionType& decision) {
  DCHECK(IsLateralDecision(decision))
      << "Decision: " << decision.ShortDebugString()
      << " is not a lateral decision";
  lateral_decision_ = MergeLateralDecision(lateral_decision_, decision);
  ADEBUG << decider_tag << " added obstacle " << Id()
         << " a lateral decision: " << decision.ShortDebugString()
         << ". The merged decision is: "
         << lateral_decision_.ShortDebugString();
  decisions_.push_back(decision);
  decider_tags_.push_back(decider_tag);
}

void Obstacle::ClearDecision() {
  lateral_decision_.Clear();
  longitudinal_decision_.Clear();
  decisions_.clear();
  decider_tags_.clear();
}

std::string Obstacle::DebugString() const {
  std::stringstream ss;
  ss << "Obstacle id: " << id_;
  for (size_t i = 0; i < decisions_.size(); ++i) {
    ss << " decision: " << decisions_[i].DebugString() << ", made by "
       << decider_tags_[i];
  }
  if (lateral_decision_.object_tag_case() !=
      ObjectDecisionType::OBJECT_TAG_NOT_SET) {
    ss << "lateral decision: " << lateral_decision_.ShortDebugString();
  }
  if (longitudinal_decision_.object_tag_case() !=
      ObjectDecisionType::OBJECT_TAG_NOT_SET) {
    ss << "longitudinal decision: "
       << longitudinal_decision_.ShortDebugString();
  }
  return ss.str();
}

void Obstacle::set_path_st_boundary(const STBoundary& boundary) {
  path_st_boundary_ = boundary;
  path_st_boundary_initialized_ = true;
}

void Obstacle::set_path_st_boundary(STBoundary&& boundary) {
  path_st_boundary_ = std::move(boundary);
  path_st_boundary_initialized_ = true;
}

void Obstacle::SetStBoundaryType(const STBoundary::BoundaryType& type) {
  path_st_boundary_.SetBoundaryType(type);
}

void Obstacle::EraseStBoundary() {
  path_st_boundary_ = STBoundary();
}

void Obstacle::SetReferenceLineStBoundary(const STBoundary& boundary) {
  reference_line_st_boundary_ = boundary;
}

void Obstacle::SetReferenceLineStBoundaryType(
    const STBoundary::BoundaryType& type) {
  reference_line_st_boundary_.SetBoundaryType(type);
}

void Obstacle::EraseReferenceLineStBoundary() {
  reference_line_st_boundary_ = STBoundary();
}

bool Obstacle::IsValidObstacle(
    const perception::PerceptionObstacle& perception_obstacle) {
  const double object_width = perception_obstacle.width();
  const double object_length = perception_obstacle.length();

  const double kMinObjectDimension = 1.0e-6;
  return !std::isnan(object_width) && !std::isnan(object_length) &&
         object_width > kMinObjectDimension &&
         object_length > kMinObjectDimension;
}

void Obstacle::CheckLaneBlocking(
    const std::shared_ptr<ReferenceLine>& reference_line) {
  if (!IsStatic()) {
    is_lane_blocking_ = false;
    return;
  }
  DCHECK(sl_boundary_.has_start_s());
  DCHECK(sl_boundary_.has_end_s());
  DCHECK(sl_boundary_.has_start_l());
  DCHECK(sl_boundary_.has_end_l());

  if (sl_boundary_.start_l() * sl_boundary_.end_l() < 0.0) {
    is_lane_blocking_ = true;
    return;
  }

  const double driving_width = reference_line->GetDrivingWidth(sl_boundary_);
  const auto& vehicle_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();

  if (reference_line->IsOnLane(sl_boundary_) &&
      driving_width <
          vehicle_param.width() + FLAGS_static_obstacle_nudge_l_buffer) {
    is_lane_blocking_ = true;
    return;
  }

  is_lane_blocking_ = false;
}

void Obstacle::SetLaneChangeBlocking(const bool is_distance_clear) {
  is_lane_change_blocking_ = is_distance_clear;
}

void Obstacle::SetIsShortDistanceThresholdDone(
    const bool is_short_distance_threshold_done) {
  is_short_distance_threshold_done_ = is_short_distance_threshold_done;
}

void Obstacle::SetShortDistanceThreshold(double short_distance_threshold) {
  short_distance_threshold_ = short_distance_threshold;
}

void Obstacle::SetIsShortDistanceNudge(const bool is_short_distance_nudge) {
  is_short_distance_nudge_ = is_short_distance_nudge;
}

void Obstacle::SetMaxExpectTowingL(const double max_expect_towing_l) {
  max_expect_towing_l_ = max_expect_towing_l;
}

void Obstacle::SetForbidStaticObsNudgeDisplay(
    bool forbid_static_obs_nudge_display) {
  forbid_static_obs_nudge_display_ = forbid_static_obs_nudge_display;
}

bool Obstacle::IsObstacleWithLowHeight(
    const perception::PerceptionObstacle& perception_obstacle) {
  switch (perception_obstacle.sub_type()) {
    case TL::perception::PerceptionObstacle::ST_BARRIER_SIGN:
    case TL::perception::PerceptionObstacle::ST_BARRIER_TRIANGLE:
    case TL::perception::PerceptionObstacle::ST_STOP:
    case TL::perception::PerceptionObstacle::ST_SLOWYIELD:
    case TL::perception::PerceptionObstacle::ST_NOPASS:
    case TL::perception::PerceptionObstacle::ST_NOENTRY:
    case TL::perception::PerceptionObstacle::ST_NOTURNINGLEFT:
    case TL::perception::PerceptionObstacle::ST_NOTURNINGRIGHT:
    case TL::perception::PerceptionObstacle::ST_NOGOINGSTRAIGHT:
    case TL::perception::PerceptionObstacle::ST_NOTURNINGAROUND:
    case TL::perception::PerceptionObstacle::ST_NOOVERTAKING:
    case TL::perception::PerceptionObstacle::ST_REMOVENOOVERTAKING:
    case TL::perception::PerceptionObstacle::ST_NOPARKING:
    case TL::perception::PerceptionObstacle::ST_NOHONKING:
    case TL::perception::PerceptionObstacle::ST_SPEEDLIMITLIFTED:
    case TL::perception::PerceptionObstacle::ST_SPEEDRELEASELIMITLIFTED:
    case TL::perception::PerceptionObstacle::ST_TRAFFICLIGHT:
    case TL::perception::PerceptionObstacle::ST_BAN:
    case TL::perception::PerceptionObstacle::ST_D_ARROW:
    case TL::perception::PerceptionObstacle::ST_L_ARROW:
    case TL::perception::PerceptionObstacle::ST_R_ARROW:
    case TL::perception::PerceptionObstacle::ST_A_ARROW:
    case TL::perception::PerceptionObstacle::ST_ZEBRA:
    case TL::perception::PerceptionObstacle::ST_SPEEDBUMP:
    case TL::perception::PerceptionObstacle::ST_STOP_LINE: {
      return true;
    }
    default:
      return false;
  }
}

bool Obstacle::IsObstacleVulnerableAlive(
    const perception::PerceptionObstacle& perception_obstacle) {
  switch (perception_obstacle.type()) {
    case TL::perception::PerceptionObstacle::PEDESTRIAN:
    case TL::perception::PerceptionObstacle::CYCLIST: {
      return true;
    }
    default:
      return false;
  }
}

void Obstacle::InitTrajectoryBoundingBox() {
  trajectory_bounding_box_.clear();
  double length = PerceptionBoundingBox().length();
  double width = PerceptionBoundingBox().width();
  for (const auto& point : trajectory_.trajectory_point()) {
    trajectory_bounding_box_.emplace_back(
        common::math::Vec2d{point.path_point().x(), point.path_point().y()},
        point.path_point().theta(), length, width);
  }
}

}  // namespace planning
}  // namespace TL
