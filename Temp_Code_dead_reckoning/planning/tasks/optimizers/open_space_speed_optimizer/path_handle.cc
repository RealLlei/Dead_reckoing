/******************************************************************************
 * Copyright 2019 The TL Authors. All Rights Reserved.
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
#include "planning/tasks/optimizers/open_space_speed_optimizer/path_handle.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <limits>
#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>
#include "common/configs/vehicle_config_helper.h"
#include "common/math/box2d.h"
#include "common/math/double_type.h"
#include "common/math/line_segment2d.h"
#include "common/math/math_utils.h"
#include "common/math/polygon2d.h"
#include "common/math/vec2d.h"
#include "common/time/clock.h"
#include "planning/common/open_space_info.h"
#include "planning/common/path/discretized_path.h"
#include "planning/common/util/common.h"
#include "planning/common/util/util.h"
#include "proto/common/pnc_point.pb.h"
#include "proto/common/vehicle_config.pb.h"
#include "proto/common/vehicle_state.pb.h"
#include "proto/perception/perception_freespace.pb.h"
#include "proto/perception/perception_obstacle.pb.h"
#include "proto/planning/planning_internal.pb.h"

namespace TL {
namespace planning {

using common::math::double_type::DefinitelyGreater;
using common::math::double_type::DefinitelyLess;
using perception::FreeSpaceOut;

namespace {
const double kSquareTwo = 1.414;
const double kAmplificationRatio = 1.2;
const double kVehiclLateralBuffer = 0.1;
}  // namespace

PathHandle::PathHandle(const OpenSpaceSpeedOptimizerConfig& config)
    : config_(config),
      predict_box_size_(static_cast<size_t>(
          config_.prediction_time_period() / predict_unit_t_ + 1)),
      speed_limit_unit_s_(config_.speed_limit_unit_s()) {
  moving_obs_ptrs_.reserve(max_moving_obs_size_);
  moving_obs_ptrs_.assign(max_moving_obs_size_, nullptr);
  static_obs_ptrs_.reserve(max_static_obs_size_);
  static_obs_ptrs_.assign(max_static_obs_size_, nullptr);
  uss_obs_ptrs_.reserve(max_uss_obs_size_);
  uss_obs_ptrs_.assign(max_uss_obs_size_, nullptr);
  wheelmask_obs_ptrs_.reserve(max_wheelmask_obs_size_);
  wheelmask_obs_ptrs_.assign(max_wheelmask_obs_size_, nullptr);
  moving_obs_boxs_.reserve(predict_box_size_);
  moving_obs_boxs_.assign(predict_box_size_, std::vector<common::math::Box2d>(
                                                 max_moving_obs_size_));
}

void PathHandle::Init(const planning::OpenSpaceEnvStructuredInfo&
                          open_space_env_structured_info) {
  moving_obs_size_ = 0;
  static_obs_size_ = 0;
  uss_obs_size_ = 0;
  wheelmask_obs_size_ = 0;
  not_lidar_not_vehicle_fs_segments_.clear();
  not_lidar_vehicle_fs_segments_.clear();
  lidar_not_vehicle_fs_segments_.clear();
  lidar_vehicle_fs_segments_.clear();
  low_height_fs_segments_.clear();
  high_height_curb_fs_segments_.clear();
  not_lidar_not_vehicle_fs_.clear();
  not_lidar_vehicle_fs_.clear();
  lidar_not_vehicle_fs_.clear();
  lidar_vehicle_fs_.clear();
  low_height_fs_.clear();
  high_height_curb_fs_.clear();
  speed_limits_.clear();
  all_freespace_segments_.clear();
  switch (open_space_env_structured_info.parking_scenario_type) {
    case ParkingScenarioType::LEFT_VERTICAL_PARKING_IN:
    case ParkingScenarioType::RIGHT_VERTICAL_PARKING_IN:
    case ParkingScenarioType::LEFT_OBLIQUE_PARKING_IN:
    case ParkingScenarioType::RIGHT_OBLIQUE_PARKING_IN:
      is_vertical_park_in_ = true;
      break;
    case ParkingScenarioType::LEFT_LATERAL_PARKING_IN:
    case ParkingScenarioType::RIGHT_LATERAL_PARKING_IN:
      is_lateral_park_in_ = true;
      break;
    case ParkingScenarioType::LEFT_LATERAL_PARKING_OUT:
    case ParkingScenarioType::RIGHT_LATERAL_PARKING_OUT:
      is_lateral_park_out_ = true;
      break;
    case ParkingScenarioType::FREESPACE_FORWARD_EXPLORATION:
      is_nns_adjust_ = true;
      break;
    default:
      is_lateral_park_in_ = false;
      is_lateral_park_out_ = false;
      is_vertical_park_in_ = false;
      is_nns_adjust_ = false;
      break;
  }
  is_narrow_spot_scenario_ =
      (open_space_env_structured_info.parking_scenario_diffculty_type &
       NARROW_SPOT_SCENARIO) != 0;
}

bool PathHandle::CutOffPathByWheelMask(const DiscretizedPath& path,
                                       const bool is_forward,
                                       const bool is_parking_inwards,
                                       const bool is_consider_wheel_mask,
                                       const Box2d& wheel_mask_box,
                                       DiscretizedPath* const new_path) {
  if (path.empty() || nullptr == new_path) {
    return false;
  }
  // parking-inwards but reversing OR vice-versa, no need run cut off path
  if ((!is_forward && is_parking_inwards) ||
      (is_forward && !is_parking_inwards) || !is_consider_wheel_mask ||
      !(wheel_mask_box.area() > 0)) {
    pre_wheel_mask_valid_ = false;
    *new_path = path;
    return true;
  }

  size_t wheel_mask_index = path.size();
  const double half_width =
      0.5 * common::VehicleConfigHelper::GetConfig().vehicle_param().width();
  auto wheel_mask_center = wheel_mask_box.center();
  if (is_forward && is_parking_inwards) {
    double slot_heading =
        common::math::NormalizeAngle(path.back().theta() + M_PI);
    const double wheel_base =
        common::VehicleConfigHelper::GetConfig().vehicle_param().wheel_base();
    wheel_mask_center =
        wheel_mask_center + wheel_base * Vec2d::CreateUnitVec2d(slot_heading);
  }

  common::SLPoint sl_point;
  if (path.XYToSL(wheel_mask_center.x(), wheel_mask_center.y(), &sl_point)) {
    sl_point.set_s(sl_point.s() - config_.wheel_mask_to_wheel_base_distance());
    if (DefinitelyLess(sl_point.s(), path.back().s()) &&
        DefinitelyLess(path.front().s(), sl_point.s()) &&
        DefinitelyLess(fabs(sl_point.l()), half_width)) {
      wheel_mask_index = std::distance(
          path.begin(), std::lower_bound(path.begin(), path.end(), sl_point.s(),
                                         [](const common::PathPoint& p,
                                            double s) { return p.s() < s; }));
    }
    if (pre_wheel_mask_valid_ && DefinitelyLess(sl_point.s(), 0) &&
        DefinitelyLess(fabs(sl_point.l()), half_width)) {
      wheel_mask_index = 0;
    }
  }

  pre_wheel_mask_valid_ = wheel_mask_index < path.size();
  *new_path = DiscretizedPath(std::vector<common::PathPoint>(
      path.begin(), path.begin() + static_cast<int>(wheel_mask_index)));
  if (wheel_mask_index > 0 && wheel_mask_index < path.size()) {
    new_path->emplace_back(path.Evaluate(sl_point.s()));
  }

  return !new_path->empty();
}

void PathHandle::UpdateStaticObstacleSegments() {
  static_vehicle_segments_.clear();
  static_vehicle_segment_count_.clear();
  static_pedestrian_segments_.clear();
  static_pedestrian_segment_count_.clear();
  static_other_segments_.clear();
  static_other_segment_count_.clear();
  size_t vehicle_segment_size = 0;
  size_t pedestrian_segment_size = 0;
  size_t other_segment_size = 0;

  for (size_t i = 0; i < static_obs_size_; i++) {
    auto obj_polygon2d =
        common::math::Polygon2d(static_obs_ptrs_[i]->PerceptionBoundingBox());
    auto type = static_obs_ptrs_[i]->Perception().type();
    ADEBUG << " static obs id: " << static_obs_ptrs_[i]->PerceptionId()
           << " type: " << type
           << " size: " << obj_polygon2d.line_segments().size();
    if (perception::PerceptionObstacle::PEDESTRIAN == type) {
      pedestrian_segment_size += obj_polygon2d.line_segments().size();
      static_pedestrian_segment_count_.emplace_back(pedestrian_segment_size, i);
      for (const auto& segment : obj_polygon2d.line_segments()) {
        static_pedestrian_segments_.emplace_back(segment, 0.0);
      }
    } else if (perception::PerceptionObstacle::VEHICLE == type) {
      vehicle_segment_size += obj_polygon2d.line_segments().size();
      static_vehicle_segment_count_.emplace_back(vehicle_segment_size, i);
      for (const auto& segment : obj_polygon2d.line_segments()) {
        static_vehicle_segments_.emplace_back(segment, 0.0);
      }
    } else {
      other_segment_size += obj_polygon2d.line_segments().size();
      static_other_segment_count_.emplace_back(other_segment_size, i);
      for (const auto& segment : obj_polygon2d.line_segments()) {
        static_other_segments_.emplace_back(segment, 0.0);
      }
    }
  }
}

void PathHandle::UpdateMovingObstacleBoxs() {
  for (size_t i = 0; i < predict_box_size_; i++) {
    for (size_t j = 0; j < moving_obs_size_; j++) {
      const auto* obs = moving_obs_ptrs_[j];
      if (perception::PerceptionObstacle::PEDESTRIAN ==
              obs->Perception().type() &&
          obs->HasTrajectory() &&
          obs->Trajectory().trajectory_point_size() > 0 &&
          obs->Trajectory().trajectory_point().at(0).has_gaussian_info()) {
        const auto& gaussian_info =
            obs->Trajectory().trajectory_point().at(0).gaussian_info();
        const auto center =
            Vec2d(obs->Trajectory().trajectory_point().at(0).path_point().x(),
                  obs->Trajectory().trajectory_point().at(0).path_point().y());
        moving_obs_boxs_[i][j] =
            Box2d(center, gaussian_info.theta_a(), gaussian_info.ellipse_a(),
                  gaussian_info.ellipse_b());
        ADEBUG << " pedestrian id:" << obs->Id() << " center x: " << center.x()
               << " center y: " << center.y()
               << " ellipse_a: " << gaussian_info.ellipse_a() << " ellipse_b: "
               << gaussian_info.ellipse_b();  // LCOV_EXCL_LINE
      } else {
        auto point =
            obs->GetPointAtTime(predict_unit_t_ * static_cast<double>(i));
        moving_obs_boxs_[i][j] = obs->GetBoundingBox(point);
      }
    }
  }
}

void PathHandle::UpdateFreeSpaceInfo(
    const DiscretizedPath& path,
    const std::shared_ptr<const FreeSpaceOutArray>& freespace_out_array,
    const std::vector<size_t>& under_spot_low_fs_idxs,
    const std::vector<size_t>& high_curb_fs_idxs,
    const std::vector<std::pair<size_t, std::vector<size_t>>>& ignore_fs_idxs) {
  if (path.empty() || nullptr == freespace_out_array) {
    return;
  }

  std::vector<FreeSpaceSegment> freespace_segments;
  util::UpdateFreeSpaceSegmentsByPath(path, freespace_out_array,
                                      &freespace_segments);
  ADEBUG << "freespace segments size: " << freespace_segments.size();
  auto is_lidar_fs = [](FreeSpaceOut::SensorType type) {
    return FreeSpaceOut::LIDAR == type ||
           FreeSpaceOut::LIDAR_CAM_FUSION == type ||
           FreeSpaceOut::USS_LIDAR_CAM_FUSION == type ||
           FreeSpaceOut::USS_LIDAR_FUSION == type;
  };

  for (const auto& freespace_segment : freespace_segments) {
    auto index_in_freespace_out = freespace_segment.index_in_freespace_out;
    auto index_in_keypoints = freespace_segment.index_in_keypoints;
    auto ignore_iter =
        std::find_if(ignore_fs_idxs.begin(), ignore_fs_idxs.end(),
                     [&index_in_freespace_out](const auto& p) {
                       return p.first == index_in_freespace_out;
                     });

    if (ignore_iter != ignore_fs_idxs.end() &&
        (std::find(ignore_iter->second.begin(), ignore_iter->second.end(),
                   index_in_keypoints) != ignore_iter->second.end() ||
         std::find(ignore_iter->second.begin(), ignore_iter->second.end(),
                   index_in_keypoints - 1) != ignore_iter->second.end())) {
      continue;
    }

    auto is_low_fs =
        std::find(under_spot_low_fs_idxs.begin(), under_spot_low_fs_idxs.end(),
                  index_in_freespace_out) != under_spot_low_fs_idxs.end();
    auto is_high_curb_fs =
        std::find(high_curb_fs_idxs.begin(), high_curb_fs_idxs.end(),
                  index_in_freespace_out) != high_curb_fs_idxs.end();
    auto is_lidar = is_lidar_fs(freespace_segment.sensor_type);
    auto is_vehicle = FreeSpaceOut::VEHICLE == freespace_segment.cls_type;
    std::pair<common::math::LineSegment2d, double> temp;
    temp.first = freespace_segment.segment;
    temp.second = 0.0;
    all_freespace_segments_.emplace_back(freespace_segment.segment,
                                         config_.path_collision_check_buffer());
    if (is_low_fs) {
      low_height_fs_segments_.emplace_back(freespace_segment);
      low_height_fs_.emplace_back(std::move(temp));
    } else if (!is_lidar && !is_vehicle) {
      not_lidar_not_vehicle_fs_segments_.emplace_back(freespace_segment);
      not_lidar_not_vehicle_fs_.emplace_back(std::move(temp));
    } else if (!is_lidar && is_vehicle) {
      not_lidar_vehicle_fs_segments_.emplace_back(freespace_segment);
      not_lidar_vehicle_fs_.emplace_back(std::move(temp));
    } else if (is_lidar && is_vehicle) {
      lidar_vehicle_fs_segments_.emplace_back(freespace_segment);
      lidar_vehicle_fs_.emplace_back(std::move(temp));
    } else if (is_lidar && !is_vehicle) {
      lidar_not_vehicle_fs_segments_.emplace_back(freespace_segment);
      lidar_not_vehicle_fs_.emplace_back(std::move(temp));
    }
    if (is_high_curb_fs) {
      high_height_curb_fs_segments_.emplace_back(freespace_segment);
      high_height_curb_fs_.emplace_back(std::move(temp));
    }
  }

  ADEBUG << " low_height_fs_segments size: " << low_height_fs_segments_.size()
         << " not_lidar_not_vehicle_fs_segments size: "
         << not_lidar_not_vehicle_fs_segments_.size()
         << " not_lidar_vehicle_fs_segments size:"
         << not_lidar_vehicle_fs_segments_.size()
         << " lidar_vehicle_fs_segments size: "
         << lidar_vehicle_fs_segments_.size()
         << " lidar_not_vehicle_fs_segment size: "
         << lidar_not_vehicle_fs_segments_.size();
  ADEBUG << "high_height_curb_fs_segments size: "
         << high_height_curb_fs_segments_.size();
}

void PathHandle::UpdateValidObstacleInfo(
    const DiscretizedPath& path,
    const std::vector<const std::shared_ptr<Obstacle>*>& obstacles,
    const std::shared_ptr<const FreeSpaceOutArray>& freespace_out_array,
    const std::vector<size_t>& under_spot_low_fs_idxs,
    const std::vector<size_t>& high_curb_fs_idxs,
    const std::vector<std::pair<size_t, std::vector<size_t>>>& ignore_fs_idxs,
    const bool is_forward, const bool is_rpa_direct_mode) {
  if (path.empty()) {
    AERROR << "input path is empty!";
    return;
  }

  SLBoundary sl_boundary;
  const auto& vehicle_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  double veh_front_edge = vehicle_param.front_edge_to_center();
  double veh_back_edge = vehicle_param.back_edge_to_center();
  double veh_width = vehicle_param.width();
  double max_s = path.back().s();
  double min_s = path.front().s();
  double max_l = veh_width * 0.5;
  double min_l = -veh_width * 0.5;
  max_l += config_.lateral_filter_distance();
  min_l -= config_.lateral_filter_distance();
  if (is_forward) {
    max_s += veh_front_edge + config_.longitudinal_filter_distance();
    min_s -= is_rpa_direct_mode
                 ? 0.0
                 : (veh_back_edge + config_.longitudinal_filter_distance());
  } else {
    max_s += veh_back_edge + config_.longitudinal_filter_distance();
    min_s -= is_rpa_direct_mode
                 ? 0.0
                 : (veh_front_edge + config_.longitudinal_filter_distance());
  }

  auto not_in_sl = [&]() {
    return DefinitelyLess(max_s, sl_boundary.start_s()) ||
           DefinitelyLess(sl_boundary.end_s(), min_s) ||
           DefinitelyLess(max_l, sl_boundary.start_l()) ||
           DefinitelyLess(sl_boundary.end_l(), min_l);
  };

  for (const auto* obs : obstacles) {
    if (nullptr == obs || (*obs)->IsLowHeight() || (*obs)->IsVirtual() ||
        !path.GetSLBoundary((*obs)->PerceptionBoundingBox(), &sl_boundary)) {
      continue;
    }
    if ((*obs)->Perception().sub_type() ==
        perception::PerceptionObstacle::ST_WHEELSTOP) {
      if (!not_in_sl() && wheelmask_obs_size_ < max_wheelmask_obs_size_) {
        wheelmask_obs_ptrs_[wheelmask_obs_size_++] = (*obs).get();
      }
      continue;
    }
    if ((*obs)->IsUssObs() && uss_obs_size_ < max_uss_obs_size_ &&
        !not_in_sl()) {
      uss_obs_ptrs_[uss_obs_size_++] = (*obs).get();
    } else if ((*obs)->IsStatic() && static_obs_size_ < max_static_obs_size_ &&
               !not_in_sl()) {
      static_obs_ptrs_[static_obs_size_++] = (*obs).get();
    } else if (!(*obs)->IsStatic() && moving_obs_size_ < max_moving_obs_size_) {
      moving_obs_ptrs_[moving_obs_size_++] = (*obs).get();
    }
  }
  UpdateStaticObstacleSegments();
  UpdateMovingObstacleBoxs();
  UpdateFreeSpaceInfo(path, freespace_out_array, under_spot_low_fs_idxs,
                      high_curb_fs_idxs, ignore_fs_idxs);
}

OpenSpaceSpeedOptimizerConfig::CollisionBufferInfo
PathHandle::UpdateCollisionBuffer(const bool is_forward) const {
  OpenSpaceSpeedOptimizerConfig::CollisionBufferInfo collision_buffer =
      is_forward ? config_.vertical_forward_collision_buffer()
                 : config_.vertical_backward_collision_buffer();
  if (is_lateral_park_in_ || is_lateral_park_out_) {
    collision_buffer = is_forward
                           ? config_.parallel_forward_collision_buffer()
                           : config_.parallel_backward_collision_buffer();
  }

  return collision_buffer;
}

void PathHandle::CalLateralBufferByControlDiff(
    const common::VehicleState& vehicle_state, const common::PathPoint& path_p,
    double* const left_lateral_buffer, double* const right_lateral_buffer) {
  Vec2d curr_to_ego = {vehicle_state.x() - path_p.x(),
                       vehicle_state.y() - path_p.y()};
  Vec2d ego_unit = Vec2d::CreateUnitVec2d(vehicle_state.heading());
  double lateral_diff = ego_unit.CrossProd(curr_to_ego);
  // + -> left diff ,- -> right diff
  if (lateral_diff > 0 && nullptr != left_lateral_buffer) {
    *left_lateral_buffer = lateral_diff;
  }

  if (lateral_diff < 0 && nullptr != right_lateral_buffer) {
    *right_lateral_buffer = fabs(lateral_diff);
  }
}

bool PathHandle::IsCollisionWithStaticObstacle(
    const DiscretizedPath& path, const bool is_forward,
    CollisionInfo::ObstacleCollisionInfo* const collision_info) {
  if (path.empty() || static_obs_size_ < 1 || nullptr == collision_info) {
    return false;
  }
  collision_info->Init(path.size());
  for (size_t i = 0; i < path.size(); ++i) {
    const auto& path_point = path.at(i);
    size_t collision_segment_index = 0;
    const common::VehicleParam& vehicle_param =
        common::VehicleConfigHelper::GetConfig().vehicle_param();
    double diff = (vehicle_param.front_edge_to_center() -
                   vehicle_param.back_edge_to_center()) /
                  2.0;
    common::math::Vec2d polygon2d_center(
        path_point.x() + diff * std::cos(path_point.theta()),
        path_point.y() + diff * std::sin(path_point.theta()));
    const auto unit_vec2d =
        common::math::Vec2d::CreateUnitVec2d(path_point.theta());
    const double euclidean_filter_distance =
        std::hypot(vehicle_param.length(), vehicle_param.width()) / 2.0;
    const double longitudinal_filter_distance = vehicle_param.length() / 2.0;
    const double lateral_filter_distance = vehicle_param.width() / 2.0;
    auto find_id_index = [](const std::pair<int, size_t>& pair, int index) {
      return pair.first < index;
    };

    // check if collision with pedestrian
    auto front_buffer =
        is_forward ? config_.collision_buffer_for_static_pedestrian() : 0.0;
    auto rear_buffer =
        is_forward ? 0.0 : config_.collision_buffer_for_static_pedestrian();
    auto polygon2d = common::VehicleConfigHelper::GetPolygon2dWithBuffer(
        path_point.x(), path_point.y(), path_point.theta(), front_buffer,
        rear_buffer, config_.collision_buffer_for_static_pedestrian(),
        config_.collision_buffer_for_static_pedestrian());
    if (common::math::CheckCollisionWithVehiclePolygon2d(
            polygon2d, polygon2d_center, unit_vec2d,
            static_pedestrian_segments_,
            euclidean_filter_distance +
                config_.collision_buffer_for_static_pedestrian() * kSquareTwo *
                    kAmplificationRatio,
            longitudinal_filter_distance +
                config_.collision_buffer_for_static_pedestrian() *
                    kAmplificationRatio,
            lateral_filter_distance +
                config_.collision_buffer_for_static_pedestrian() *
                    kAmplificationRatio,
            &collision_segment_index)) {
      collision_info->is_collision = true;
      collision_info->collision_index = i;
      auto iter = std::lower_bound(static_pedestrian_segment_count_.begin(),
                                   static_pedestrian_segment_count_.end(),
                                   collision_segment_index + 1, find_id_index);
      if (iter != static_pedestrian_segment_count_.end() &&
          iter->second < static_obs_size_) {
        collision_info->collision_obstacle_id =
            static_obs_ptrs_[iter->second]->PerceptionId();
        collision_info->collision_obstacle_type =
            static_obs_ptrs_[iter->second]->Perception().type();
      }

      return true;
    }

    // check if collision with vehicle
    front_buffer =
        is_forward ? config_.collision_buffer_for_static_vehicle() : 0.0;
    rear_buffer =
        is_forward ? 0.0 : config_.collision_buffer_for_static_vehicle();
    polygon2d = common::VehicleConfigHelper::GetPolygon2dWithBuffer(
        path_point.x(), path_point.y(), path_point.theta(), front_buffer,
        rear_buffer, kVehiclLateralBuffer, kVehiclLateralBuffer);
    collision_segment_index = 0;
    if (common::math::CheckCollisionWithVehiclePolygon2d(
            polygon2d, polygon2d_center, unit_vec2d, static_vehicle_segments_,
            euclidean_filter_distance +
                config_.collision_buffer_for_static_vehicle() * kSquareTwo *
                    kAmplificationRatio,
            longitudinal_filter_distance +
                config_.collision_buffer_for_static_vehicle() *
                    kAmplificationRatio,
            lateral_filter_distance +
                config_.collision_buffer_for_static_vehicle() *
                    kAmplificationRatio,
            &collision_segment_index)) {
      collision_info->is_collision = true;
      collision_info->collision_index = i;
      auto iter = std::lower_bound(static_vehicle_segment_count_.begin(),
                                   static_vehicle_segment_count_.end(),
                                   collision_segment_index + 1, find_id_index);
      if (iter != static_vehicle_segment_count_.end() &&
          iter->second < static_obs_size_) {
        collision_info->collision_obstacle_id =
            static_obs_ptrs_[iter->second]->PerceptionId();
        collision_info->collision_obstacle_type =
            static_obs_ptrs_[iter->second]->Perception().type();
      }

      return true;
    }

    // check if collision with others
    front_buffer = is_forward ? config_.collision_buffer_for_other() : 0.0;
    rear_buffer = is_forward ? 0.0 : config_.collision_buffer_for_other();
    polygon2d = common::VehicleConfigHelper::GetPolygon2dWithBuffer(
        path_point.x(), path_point.y(), path_point.theta(), front_buffer,
        rear_buffer, config_.collision_buffer_for_other(),
        config_.collision_buffer_for_other());
    collision_segment_index = 0;
    if (common::math::CheckCollisionWithVehiclePolygon2d(
            polygon2d, polygon2d_center, unit_vec2d, static_other_segments_,
            euclidean_filter_distance + config_.collision_buffer_for_other() *
                                            kSquareTwo * kAmplificationRatio,
            longitudinal_filter_distance +
                config_.collision_buffer_for_other() * kAmplificationRatio,
            lateral_filter_distance +
                config_.collision_buffer_for_other() * kAmplificationRatio,
            &collision_segment_index)) {
      collision_info->is_collision = true;
      collision_info->collision_index = i;
      auto iter = std::lower_bound(static_other_segment_count_.begin(),
                                   static_other_segment_count_.end(),
                                   collision_segment_index + 1, find_id_index);
      if (iter != static_other_segment_count_.end() &&
          iter->second < static_obs_size_) {
        collision_info->collision_obstacle_id =
            static_obs_ptrs_[iter->second]->PerceptionId();
        collision_info->collision_obstacle_type =
            static_obs_ptrs_[iter->second]->Perception().type();
      }
      return true;
    }
  }

  return false;
}

bool PathHandle::IsCollisionWithMovingObstacle(
    const DiscretizedPath& path, const bool is_forward,
    CollisionInfo::ObstacleCollisionInfo* const collision_info) {
  if (path.empty() || moving_obs_size_ < 1 || nullptr == collision_info) {
    return false;
  }
  const common::VehicleParam& vehicle_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  const auto front_buffer =
      is_forward ? config_.apa_moving_obstacle_front_buffer() : 0.0;
  const auto rear_buffer =
      is_forward ? 0.0 : config_.apa_moving_obstacle_rear_buffer();
  const double ego_center_to_big_buffer =
      (vehicle_param.length() + front_buffer + rear_buffer) / 2 -
      vehicle_param.back_edge_to_center() - rear_buffer;
  collision_info->Init(path.size());
  const auto box_length = vehicle_param.length() + front_buffer + rear_buffer;
  const auto box_width = vehicle_param.width() +
                         config_.apa_moving_obstacle_left_right_buffer() * 2;
  for (size_t i = 0; i < path.size(); ++i) {
    const double x_tans_dis =
        ego_center_to_big_buffer *
        std::cos(common::math::NormalizeAngle(path.at(i).theta()));
    const double y_tans_dis =
        ego_center_to_big_buffer *
        std::sin(common::math::NormalizeAngle(path.at(i).theta()));
    Vec2d ego_center_map_frame(path.at(i).x() + x_tans_dis,
                               path.at(i).y() + y_tans_dis);
    common::math::Box2d ego_box(ego_center_map_frame, path.at(i).theta(),
                                box_length, box_width);
    for (size_t m = 0; m < predict_box_size_; m++) {
      for (size_t n = 0; n < moving_obs_size_; n++) {
        if (ego_box.HasOverlap(moving_obs_boxs_[m][n])) {
          collision_info->is_collision = true;
          collision_info->collision_index = i;
          collision_info->collision_obstacle_id =
              moving_obs_ptrs_[n]->PerceptionId();
          collision_info->collision_obstacle_type =
              moving_obs_ptrs_[n]->Perception().type();
          return true;
        }
      }
    }
  }

  return false;
}

bool PathHandle::IsCollisionWithOutsideWheelMaskObstacle(
    const DiscretizedPath& path,
    const OpenSpaceSpeedOptimizerConfig::CollisionBufferInfo& collision_buffer,
    const double left_control_diff, const double right_control_diff,
    CollisionInfo::ObstacleCollisionInfo* const collision_info) {
  if (path.empty() || wheelmask_obs_size_ < 1 || nullptr == collision_info) {
    return false;
  }
  collision_info->Init(path.size());
  for (size_t i = 0; i < path.size(); i++) {
    const auto& path_point = path.at(i);
    auto polygon2d = common::VehicleConfigHelper::GetPolygon2dWithBuffer(
        path_point.x(), path_point.y(), path_point.theta(),
        collision_buffer.front_longitudinal_buffer_outside_wheelmask(),
        collision_buffer.rear_longitudinal_buffer_outside_wheelmask(),
        collision_buffer.left_lateral_buffer_outside_wheelmask() +
            left_control_diff,
        collision_buffer.right_lateral_buffer_outside_wheelmask() +
            right_control_diff);
    for (size_t n = 0; n < wheelmask_obs_size_; n++) {
      const auto* obs = wheelmask_obs_ptrs_.at(n);
      if (obs == nullptr || !obs->Perception().has_position() ||
          !obs->Perception().position().has_x() ||
          !obs->Perception().position().has_y()) {
        continue;
      }
      const Vec2d wheelmask_center(obs->Perception().position().x(),
                                   obs->Perception().position().y());
      if (polygon2d.IsPointIn(wheelmask_center)) {
        collision_info->collision_index = i;
        collision_info->is_collision = true;
        collision_info->collision_obstacle_id = obs->PerceptionId();
        collision_info->collision_obstacle_type = obs->Perception().type();
        return true;
      }
    }
  }
  return false;
}

bool PathHandle::IsCollisionWithFreeSpaceSegment(
    const DiscretizedPath& path,
    const OpenSpaceSpeedOptimizerConfig::CollisionBufferInfo& collision_buffer,
    const double left_control_diff, const double right_control_diff,
    const bool is_use_middle_buffer, const bool is_mirror_fold,
    const planning_internal::PathUpdateStatus::PathType& path_type,
    const size_t cur_path_idx,
    CollisionInfo::FreeSpaceCollisionInfo* const collision_info) {
  if (path.empty() || nullptr == collision_info) {
    return false;
  }

  auto update_collision_info =
      [&collision_info](const common::PathPoint& path_point,
                        const common::math::Polygon2d& polygon2d,
                        const size_t path_point_index,
                        const std::vector<FreeSpaceSegment>& fs_segment,
                        const size_t fs_segment_index) {
        auto path_vec2d = Vec2d(path_point.x(), path_point.y());
        collision_info->is_collision = true;
        collision_info->collision_index = path_point_index;
        collision_info->freespace_segment = fs_segment[fs_segment_index];
        Vec2d collision_point_1st =
            fs_segment[fs_segment_index].segment.start();
        Vec2d collision_point_2nd = fs_segment[fs_segment_index].segment.end();
        polygon2d.GetOverlap(fs_segment[fs_segment_index].segment,
                             &collision_point_1st, &collision_point_2nd);
        auto first_flu = common::math::ENUToFLU(
            collision_point_1st.x(), collision_point_1st.y(), path_point.x(),
            path_point.y(), path_point.theta());
        auto second_flu = common::math::ENUToFLU(
            collision_point_2nd.x(), collision_point_2nd.y(), path_point.x(),
            path_point.y(), path_point.theta());

        if (fabs(first_flu.first) < fabs(second_flu.first)) {
          collision_info->freespace_flu_point.set_x(first_flu.first);
          collision_info->freespace_flu_point.set_y(first_flu.second);
        } else {
          collision_info->freespace_flu_point.set_x(second_flu.first);
          collision_info->freespace_flu_point.set_y(second_flu.second);
        }
      };
  // pair.first means left buffer, second means right buffer
  std::pair<double, double> lateral_buffer_for_vehicle{0.0, 0.0};
  std::pair<double, double> lateral_buffer_for_not_vehicle{0.0, 0.0};
  std::pair<double, double> lateral_buffer_for_low_fs{0.0, 0.0};

  UpdateLateralBuffer(
      collision_buffer, is_use_middle_buffer, is_narrow_spot_scenario_,
      is_lateral_park_in_, is_lateral_park_out_, path_type, cur_path_idx,
      &lateral_buffer_for_vehicle, &lateral_buffer_for_not_vehicle,
      &lateral_buffer_for_low_fs);

  collision_info->Init(path.size());
  for (size_t i = 0; i < path.size(); ++i) {
    const auto& path_point = path.at(i);
    size_t collision_segment_index = 0;
    const common::VehicleParam& vehicle_param =
        common::VehicleConfigHelper::GetConfig().vehicle_param();
    double diff = (vehicle_param.front_edge_to_center() -
                   vehicle_param.back_edge_to_center()) /
                  2.0;
    common::math::Vec2d polygon2d_center(
        path_point.x() + diff * std::cos(path_point.theta()),
        path_point.y() + diff * std::sin(path_point.theta()));
    const auto unit_vec2d =
        common::math::Vec2d::CreateUnitVec2d(path_point.theta());
    const double euclidean_filter_distance =
        std::hypot(vehicle_param.length(), vehicle_param.width()) / 2.0;
    const double longitudinal_filter_distance = vehicle_param.length() / 2.0;
    const double lateral_filter_distance = vehicle_param.width() / 2.0;

    // not lidar, not vehicle
    auto used_polygon = common::VehicleConfigHelper::GetPolygon2dWithBuffer(
        path_point.x(), path_point.y(), path_point.theta(),
        collision_buffer.front_longitudinal_buffer(),
        collision_buffer.rear_longitudinal_buffer(),
        lateral_buffer_for_not_vehicle.first + left_control_diff,
        lateral_buffer_for_not_vehicle.second + right_control_diff);
    auto fliter_expand_buffer =
        std::max({collision_buffer.front_longitudinal_buffer(),
                  collision_buffer.rear_longitudinal_buffer(),
                  lateral_buffer_for_not_vehicle.first + left_control_diff,
                  lateral_buffer_for_not_vehicle.second + right_control_diff});
    if (common::math::CheckCollisionWithVehiclePolygon2d(
            used_polygon, polygon2d_center, unit_vec2d,
            not_lidar_not_vehicle_fs_,
            euclidean_filter_distance +
                fliter_expand_buffer * kSquareTwo * kAmplificationRatio,
            longitudinal_filter_distance +
                fliter_expand_buffer * kAmplificationRatio,
            lateral_filter_distance +
                fliter_expand_buffer * kAmplificationRatio,
            &collision_segment_index)) {
      update_collision_info(path_point, used_polygon, i,
                            not_lidar_not_vehicle_fs_segments_,
                            collision_segment_index);
      return true;
    }

    // not lidar, vehicle
    used_polygon = common::VehicleConfigHelper::GetPolygon2dWithBuffer(
        path_point.x(), path_point.y(), path_point.theta(),
        collision_buffer.front_longitudinal_buffer(),
        collision_buffer.rear_longitudinal_buffer(),
        lateral_buffer_for_vehicle.first + left_control_diff,
        lateral_buffer_for_vehicle.second + right_control_diff);
    fliter_expand_buffer =
        std::max({collision_buffer.front_longitudinal_buffer(),
                  collision_buffer.rear_longitudinal_buffer(),
                  lateral_buffer_for_vehicle.first + left_control_diff,
                  lateral_buffer_for_vehicle.second + right_control_diff});
    if (common::math::CheckCollisionWithVehiclePolygon2d(
            used_polygon, polygon2d_center, unit_vec2d, not_lidar_vehicle_fs_,
            euclidean_filter_distance +
                fliter_expand_buffer * kSquareTwo * kAmplificationRatio,
            longitudinal_filter_distance +
                fliter_expand_buffer * kAmplificationRatio,
            lateral_filter_distance +
                fliter_expand_buffer * kAmplificationRatio,
            &collision_segment_index)) {
      update_collision_info(path_point, used_polygon, i,
                            not_lidar_vehicle_fs_segments_,
                            collision_segment_index);
      return true;
    }

    // lidar, vehicle
    used_polygon = common::VehicleConfigHelper::GetPolygon2dWithBuffer(
        path_point.x(), path_point.y(), path_point.theta(),
        collision_buffer.front_longitudinal_buffer_lidar(),
        collision_buffer.rear_longitudinal_buffer_lidar(),
        lateral_buffer_for_vehicle.first + left_control_diff,
        lateral_buffer_for_vehicle.second + right_control_diff);
    fliter_expand_buffer =
        std::max({collision_buffer.front_longitudinal_buffer_lidar(),
                  collision_buffer.rear_longitudinal_buffer_lidar(),
                  lateral_buffer_for_vehicle.first + left_control_diff,
                  lateral_buffer_for_vehicle.second + right_control_diff});
    if (common::math::CheckCollisionWithVehiclePolygon2d(
            used_polygon, polygon2d_center, unit_vec2d, lidar_vehicle_fs_,
            euclidean_filter_distance +
                fliter_expand_buffer * kSquareTwo * kAmplificationRatio,
            longitudinal_filter_distance +
                fliter_expand_buffer * kAmplificationRatio,
            lateral_filter_distance +
                fliter_expand_buffer * kAmplificationRatio,
            &collision_segment_index)) {
      update_collision_info(path_point, used_polygon, i,
                            lidar_vehicle_fs_segments_,
                            collision_segment_index);
      return true;
    }

    // lidar, not vehicle
    used_polygon = common::VehicleConfigHelper::GetPolygon2dWithBuffer(
        path_point.x(), path_point.y(), path_point.theta(),
        collision_buffer.front_longitudinal_buffer_lidar(),
        collision_buffer.rear_longitudinal_buffer_lidar(),
        lateral_buffer_for_not_vehicle.first + left_control_diff,
        lateral_buffer_for_not_vehicle.second + right_control_diff);
    fliter_expand_buffer =
        std::max({collision_buffer.front_longitudinal_buffer_lidar(),
                  collision_buffer.rear_longitudinal_buffer_lidar(),
                  lateral_buffer_for_not_vehicle.first + left_control_diff,
                  lateral_buffer_for_not_vehicle.second + right_control_diff});
    if (common::math::CheckCollisionWithVehiclePolygon2d(
            used_polygon, polygon2d_center, unit_vec2d, lidar_not_vehicle_fs_,
            euclidean_filter_distance +
                fliter_expand_buffer * kSquareTwo * kAmplificationRatio,
            longitudinal_filter_distance +
                fliter_expand_buffer * kAmplificationRatio,
            lateral_filter_distance +
                fliter_expand_buffer * kAmplificationRatio,
            &collision_segment_index)) {
      update_collision_info(path_point, used_polygon, i,
                            lidar_not_vehicle_fs_segments_,
                            collision_segment_index);
      return true;
    }

    // low heigtht fs segment (Inluding low curb and low other class)
    used_polygon = common::VehicleConfigHelper::GetPolygon2dWithBuffer(
        path_point.x(), path_point.y(), path_point.theta(),
        collision_buffer.front_longitudinal_buffer_low_fs(),
        collision_buffer.rear_longitudinal_buffer_low_fs(),
        lateral_buffer_for_low_fs.first + left_control_diff,
        lateral_buffer_for_low_fs.second + right_control_diff);

    fliter_expand_buffer =
        std::max({collision_buffer.front_longitudinal_buffer_low_fs(),
                  collision_buffer.rear_longitudinal_buffer_low_fs(),
                  lateral_buffer_for_low_fs.first + left_control_diff,
                  lateral_buffer_for_low_fs.second + right_control_diff});
    if (common::math::CheckCollisionWithVehiclePolygon2d(
            used_polygon, polygon2d_center, unit_vec2d, low_height_fs_,
            euclidean_filter_distance +
                fliter_expand_buffer * kSquareTwo * kAmplificationRatio,
            longitudinal_filter_distance +
                fliter_expand_buffer * kAmplificationRatio,
            lateral_filter_distance +
                fliter_expand_buffer * kAmplificationRatio,
            &collision_segment_index)) {
      update_collision_info(path_point, used_polygon, i,
                            low_height_fs_segments_, collision_segment_index);
      return true;
    }

    // high height curb fs segment (Inluding wall and column)
    if (is_mirror_fold) {
      continue;
    }
    const double kDecreaseBufferForMirrorHighCurb = 0.05;
    double left_lateral_buffer_high_curb_fs =
        collision_buffer.left_lateral_buffer() -
        kDecreaseBufferForMirrorHighCurb;
    double right_lateral_buffer_high_curb_fs =
        collision_buffer.right_lateral_buffer() -
        kDecreaseBufferForMirrorHighCurb;
    const auto used_circle =
        common::VehicleConfigHelper::GetMirrorCirclesWithBuffer(
            path_point.x(), path_point.y(), path_point.theta(),
            left_lateral_buffer_high_curb_fs + left_control_diff,
            right_lateral_buffer_high_curb_fs + right_control_diff);
    if (used_circle.size() != 2) {
      AERROR << "used circle size is not 2";
      return false;
    }
    const auto& used_left_circle = used_circle.front();
    const auto& used_right_circle = used_circle.back();
    if (common::math::CheckCollisionWithCircle(
            used_left_circle, high_height_curb_fs_, &collision_segment_index) ||
        common::math::CheckCollisionWithCircle(used_right_circle,
                                               high_height_curb_fs_,
                                               &collision_segment_index)) {
      update_collision_info(path_point, used_polygon, i,
                            high_height_curb_fs_segments_,
                            collision_segment_index);
      return true;
    }
  }

  return false;
}

void PathHandle::CollisionInfoDecision(CollisionInfo* const collision_info) {
  if (nullptr == collision_info) {
    return;
  }

  collision_info->is_collision =
      collision_info->moving_obstacle_collision_info.is_collision ||
      collision_info->static_obstacle_collision_info.is_collision ||
      collision_info->freespace_collision_info.is_collision ||
      collision_info->outside_wheelmask_obstacle_collision_info.is_collision;
  collision_info->first_collision_index =
      std::min(std::initializer_list<size_t>{
          collision_info->moving_obstacle_collision_info.collision_index,
          collision_info->static_obstacle_collision_info.collision_index,
          collision_info->freespace_collision_info.collision_index,
          collision_info->outside_wheelmask_obstacle_collision_info
              .collision_index});
  if (collision_info->moving_obstacle_collision_info.is_collision) {
    collision_info->collision_type =
        AvpSpeedPlanCollisionInfo::MOVING_OBSTACLE_COLLISION;
  }
  if (collision_info->static_obstacle_collision_info.is_collision ||
      collision_info->outside_wheelmask_obstacle_collision_info.is_collision) {
    collision_info->collision_type =
        AvpSpeedPlanCollisionInfo::NO_COLLISION ==
                collision_info->collision_type
            ? AvpSpeedPlanCollisionInfo::STATIC_OBSTACLE_COLLISION
            : AvpSpeedPlanCollisionInfo::FUSION_COLLISION;
  }
  if (collision_info->freespace_collision_info.is_collision) {
    collision_info->collision_type =
        AvpSpeedPlanCollisionInfo::NO_COLLISION ==
                collision_info->collision_type
            ? AvpSpeedPlanCollisionInfo::FREESPACE_POINT_COLLISION
            : AvpSpeedPlanCollisionInfo::FUSION_COLLISION;
  }

  if (collision_info->freespace_collision_info.is_collision &&
      fabs(collision_info->freespace_collision_info.freespace_flu_point.y()) >
          0.5 * common::VehicleConfigHelper::GetConfig()
                    .vehicle_param()
                    .width()) {
    collision_info->stop_reserve_distance =
        config_.lateral_collision_stop_buffer();
  }
}

bool PathHandle::UpdateCollisionInfo(
    const DiscretizedPath& path,
    const std::vector<const std::shared_ptr<Obstacle>*>& obstacles,
    const std::shared_ptr<const FreeSpaceOutArray>& freespace_out_array,
    const std::vector<size_t>& under_spot_low_fs_idxs,
    const std::vector<size_t>& high_curb_fs_idxs,
    const std::vector<std::pair<size_t, std::vector<size_t>>>& ignore_fs_idxs,
    const common::VehicleState& vehicle_state, const bool is_forward,
    const bool is_rpa_direct_mode, const bool is_mirror_fold,
    const PartitionedPath& partitioned_paths,
    CollisionInfo* const collision_info) {
  if (nullptr == collision_info || path.empty()) {
    return false;
  }
  collision_info->Init(path.size());
  UpdateValidObstacleInfo(path, obstacles, freespace_out_array,
                          under_spot_low_fs_idxs, high_curb_fs_idxs,
                          ignore_fs_idxs, is_forward, is_rpa_direct_mode);
  auto collision_buffer = UpdateCollisionBuffer(is_forward);
  double left_control_diff = 0.0;
  double right_control_diff = 0.0;
  CalLateralBufferByControlDiff(vehicle_state, path.front(), &left_control_diff,
                                &right_control_diff);

  IsCollisionWithStaticObstacle(
      path, is_forward, &collision_info->static_obstacle_collision_info);
  IsCollisionWithMovingObstacle(
      path, is_forward, &collision_info->moving_obstacle_collision_info);
  IsCollisionWithOutsideWheelMaskObstacle(
      path, collision_buffer, left_control_diff, right_control_diff,
      &collision_info->outside_wheelmask_obstacle_collision_info);
  IsCollisionWithFreeSpaceSegment(
      path, collision_buffer, left_control_diff, right_control_diff,
      is_use_middle_buffer_, is_mirror_fold, partitioned_paths.path_type,
      partitioned_paths.path_idx, &collision_info->freespace_collision_info);
  CollisionInfoDecision(collision_info);
  UpdatePathCollisionRiskCount(path, partitioned_paths.path_type,
                               partitioned_paths.path_idx, is_mirror_fold,
                               is_forward);

  CollisionInfo::FreeSpaceCollisionInfo tmp;
  bool is_bigger_buffer_safe = true;
  if (is_use_middle_buffer_) {
    is_bigger_buffer_safe = !IsCollisionWithFreeSpaceSegment(
        path, collision_buffer, left_control_diff, right_control_diff, false,
        is_mirror_fold, partitioned_paths.path_type, partitioned_paths.path_idx,
        &tmp);
  }
  bigger_buffer_safe_count_ = is_use_middle_buffer_ && is_bigger_buffer_safe
                                  ? bigger_buffer_safe_count_ + 1
                                  : 0;
  is_use_middle_buffer_ =
      is_use_middle_buffer_ &&
      bigger_buffer_safe_count_ <= config_.bigger_buffer_safe_min_count();

  return true;
}

void PathHandle::UpdateInteractiveStage(
    const bool is_vehicle_still, const bool is_rpa_direct_mode,
    const CollisionInfo& collision_info,
    AvpSpeedPlanCollisionInfo::SpeedTaskInteractiveStage* interactive_stage) {
  static constexpr double kMinMoveDisatnce = 0.3;
  if (nullptr == interactive_stage) {
    return;
  }

  auto collision_type = collision_info.collision_type;
  auto is_restore_running = [&]() {
    return AvpSpeedPlanCollisionInfo::NO_COLLISION != collision_type
               ? DefinitelyLess(collision_info.stop_reserve_distance +
                                    config_.restore_dis_buffer(),
                                collision_info.curr_collision_distance)
               : true;
  };

  switch (*interactive_stage) {
    case AvpSpeedPlanCollisionInfo::INIT:
      if (is_vehicle_still &&
          AvpSpeedPlanCollisionInfo::NO_COLLISION != collision_type &&
          DefinitelyLess(
              collision_info.curr_collision_distance,
              collision_info.stop_reserve_distance + kMinMoveDisatnce)) {
        if (!config_.enable_wait_for_replan() ||
            AvpSpeedPlanCollisionInfo::MOVING_OBSTACLE_COLLISION ==
                collision_type) {
          *interactive_stage = AvpSpeedPlanCollisionInfo::WAITOBSTACLE;
        } else {
          *interactive_stage = AvpSpeedPlanCollisionInfo::WAITREPLAN;
          wait_replan_start_time_ = common::Clock::NowInSeconds();
          wait_replan_to_init_time_ = common::Clock::NowInSeconds();
        }
      }
      break;
    case AvpSpeedPlanCollisionInfo::WAITREPLAN:
      if (!is_restore_running()) {
        wait_replan_to_init_time_ = common::Clock::NowInSeconds();
      }
      if (DefinitelyGreater(
              common::Clock::NowInSeconds() - wait_replan_to_init_time_,
              config_.min_wait_replan_state_time())) {
        *interactive_stage = AvpSpeedPlanCollisionInfo::INIT;
      } else if (AvpSpeedPlanCollisionInfo::MOVING_OBSTACLE_COLLISION ==
                 collision_type) {
        *interactive_stage = AvpSpeedPlanCollisionInfo::WAITOBSTACLE;
        wait_obstacle_start_time_ = common::Clock::NowInSeconds();
      } else {
        double wait_replan_time =
            common::Clock::NowInSeconds() - wait_replan_start_time_;
        double threshold_time =
            is_rpa_direct_mode ? config_.max_wait_time_for_replan_rpa_direct()
                               : config_.max_wait_time_for_replan();
        if (DefinitelyGreater(wait_replan_time, threshold_time)) {
          *interactive_stage = AvpSpeedPlanCollisionInfo::WAITOBSTACLE;
          wait_obstacle_start_time_ = common::Clock::NowInSeconds();
        }
      }
      break;
    case AvpSpeedPlanCollisionInfo::WAITOBSTACLE:
      if (!is_restore_running()) {
        wait_obstacle_start_time_ = common::Clock::NowInSeconds();
      }
      if (DefinitelyGreater(
              common::Clock::NowInSeconds() - wait_obstacle_start_time_,
              config_.min_wait_obstacle_state_time())) {
        *interactive_stage = AvpSpeedPlanCollisionInfo::RUNNING;
      }
      break;
    case AvpSpeedPlanCollisionInfo::RUNNING:
#ifndef FOR_BAIDU_SIMULATION
      *interactive_stage = AvpSpeedPlanCollisionInfo::RUNNING;
#else
      *interactive_stage = AvpSpeedPlanCollisionInfo::INIT;
#endif
      break;
    default:
      *interactive_stage = AvpSpeedPlanCollisionInfo::INIT;
      break;
  }
}

void PathHandle::UpdateIsUseMiddleBuffer(
    const AvpSpeedPlanCollisionInfo::SpeedTaskInteractiveStage&
        interactive_stage,
    const bool curr_is_forward,
    const planning_internal::PathUpdateStatus::PathType& path_type) {

  if (FLAGS_enable_change_buffer_when_gear_changed &&
      path_type == planning_internal::PathUpdateStatus::SEARCH_EXTENSION_PATH &&
      !is_lateral_park_out_ &&
      (!last_is_forward_.first || last_is_forward_.second != curr_is_forward)) {
    is_use_middle_buffer_ = true;
  }
  wait_obstacle_count_ =
      AvpSpeedPlanCollisionInfo::WAITOBSTACLE == interactive_stage
          ? wait_obstacle_count_ + 1
          : 0;
  is_use_middle_buffer_ =
      is_use_middle_buffer_ ||
      wait_obstacle_count_ > config_.wait_obstacle_min_count();

  last_is_forward_.second = curr_is_forward;
  last_is_forward_.first = true;
}

bool PathHandle::CutOffPathByCollisionInfo(
    const DiscretizedPath& path, const CollisionInfo& collision_info,
    DiscretizedPath* const candidate_path) {
  if (nullptr == candidate_path || path.empty()) {
    return false;
  }

  candidate_path->clear();
  if (collision_info.is_collision &&
      collision_info.first_collision_index < path.size()) {
    double stop_distance =
        std::fmax(path.at(collision_info.first_collision_index).s() -
                      collision_info.stop_reserve_distance,
                  0.0);
    const auto index = std::distance(
        path.begin(), std::lower_bound(path.begin(), path.end(), stop_distance,
                                       [](const common::PathPoint& p,
                                          double s) { return p.s() < s; }));
    const auto stop_index = index > 0 ? index - 1 : index;
    *candidate_path = DiscretizedPath(std::vector<common::PathPoint>(
        path.begin(), path.begin() + stop_index + 1));
    if (index > 0) {
      auto proj_p = path.Evaluate(stop_distance);
      candidate_path->emplace_back(proj_p);
    }
  } else {
    *candidate_path = path;
  }

  return !candidate_path->empty();
}

double PathHandle::CalLimitSpeedByS(
    const double s,
    const OpenSpaceSpeedOptimizerConfig::SpeedBoundInfo& speed_bound_info) {
  static constexpr double kA = 6.2;
  static constexpr double kB = -23.5;
  static constexpr double kMinRation = 0.1;
  double ratio = std::max(
      speed_bound_info.max_sample_speed() - speed_bound_info.min_sample_speed(),
      kMinRation);

  return ratio * (1.0 / (1.0 + exp(kA + kB * s))) +
         speed_bound_info.min_sample_speed();
}

void PathHandle::UpdateSpeedLimits(
    const DiscretizedPath& path,
    const OpenSpaceSpeedOptimizerConfig::SpeedBoundInfo& speed_bound_info,
    const bool is_forward,
    const std::vector<common::PathPoint>& limit_speed_path_points) {
  if (path.empty()) {
    speed_limits_.clear();
    return;
  }

  const auto speed_limits_size =
      static_cast<size_t>(path.Length() / speed_limit_unit_s_ + 2);
  speed_limits_.resize(speed_limits_size);
  common::math::Box2d ego_box;
  common::PathPoint last_point;
  const auto& vehicle_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  const auto rear_fillter_distance =
      -vehicle_param.back_edge_to_center() - config_.rear_fillter_buffer();
  const auto front_fillter_distance =
      vehicle_param.front_edge_to_center() + config_.front_fillter_buffer();
  for (size_t i = 0; i < speed_limits_size; i++) {
    auto point = path.Evaluate(static_cast<double>(i) * speed_limit_unit_s_);
    ego_box = common::VehicleConfigHelper::GetBoundingBox(point);
    double nearest_obs_dist = std::numeric_limits<double>::max();

    for (size_t m = 0; m < static_obs_size_; m++) {
      const auto& obs_box = static_obs_ptrs_[m]->PerceptionBoundingBox();
      double curr_dis = ego_box.DistanceTo(obs_box);
      if (DefinitelyLess(curr_dis, nearest_obs_dist)) {
        nearest_obs_dist = curr_dis;
      }
    }

    for (size_t n = 0; n < uss_obs_size_; n++) {
      const auto& obs_box = uss_obs_ptrs_[n]->PerceptionBoundingBox();
      const auto flu_point =
          common::math::ENUToFLU(obs_box.center().x(), obs_box.center().y(),
                                 point.x(), point.y(), point.theta());
      if ((is_forward && flu_point.first < rear_fillter_distance) ||
          (!is_forward && flu_point.first > front_fillter_distance)) {
        continue;
      }
      double curr_dis = ego_box.DistanceTo(obs_box);
      if (DefinitelyLess(curr_dis, nearest_obs_dist)) {
        nearest_obs_dist = curr_dis;
      }
    }

    for (size_t k = 0; k < moving_obs_size_; k++) {
      const auto& obs_box = moving_obs_ptrs_[k]->PerceptionBoundingBox();
      double curr_dis = ego_box.DistanceTo(obs_box);
      if (DefinitelyLess(curr_dis, nearest_obs_dist)) {
        nearest_obs_dist = curr_dis;
      }
    }

    for (const auto& limit_p : limit_speed_path_points) {
      double curr_dis = ego_box.DistanceTo(Vec2d(limit_p.x(), limit_p.y()));
      if (DefinitelyLess(curr_dis, nearest_obs_dist)) {
        nearest_obs_dist = curr_dis;
      }
    }

    auto update_neareset_dist =
        [&](const std::pair<common::math::LineSegment2d, double>& segment) {
          const auto flu_point_start = common::math::ENUToFLU(
              segment.first.start().x(), segment.first.start().y(), point.x(),
              point.y(), point.theta());
          const auto flu_point_end = common::math::ENUToFLU(
              segment.first.end().x(), segment.first.end().y(), point.x(),
              point.y(), point.theta());
          if ((is_forward && flu_point_start.first < rear_fillter_distance &&
               flu_point_end.first < rear_fillter_distance) ||
              (!is_forward && flu_point_start.first > front_fillter_distance &&
               flu_point_end.first > front_fillter_distance)) {
            return;
          }
          double curr_dis = ego_box.DistanceTo(segment.first);
          if (DefinitelyLess(curr_dis, nearest_obs_dist)) {
            nearest_obs_dist = curr_dis;
          }
        };
    std::for_each(not_lidar_not_vehicle_fs_.begin(),
                  not_lidar_not_vehicle_fs_.end(), update_neareset_dist);
    std::for_each(not_lidar_vehicle_fs_.begin(), not_lidar_vehicle_fs_.end(),
                  update_neareset_dist);
    std::for_each(lidar_vehicle_fs_.begin(), lidar_vehicle_fs_.end(),
                  update_neareset_dist);
    std::for_each(lidar_not_vehicle_fs_.begin(), lidar_not_vehicle_fs_.end(),
                  update_neareset_dist);

    speed_limits_[i] = CalLimitSpeedByS(nearest_obs_dist, speed_bound_info);

    if (i > 0) {
      speed_limits_[i] = std::min(speed_limits_[i],
                                  util::GetDkappaSpeedLimit(last_point, point));
    }
    last_point = point;
  }
  SmoothSpeedLimits();
}

void PathHandle::SmoothSpeedLimits() {
  if (speed_limits_.empty()) {
    return;
  }

  int window_size = static_cast<int>(config_.speed_limit_smooth_window_size());
  const auto unsmoothed = speed_limits_;
  // [index.front, i] is a window, ascending order, index.front is the smallest in this windows
  std::deque<int> ascending_indexs;
  const auto smooth_end_index =
      static_cast<int>(unsmoothed.size()) + window_size;
  for (int i = 0; i < smooth_end_index; ++i) {
    if (i < unsmoothed.size()) {
      while (!ascending_indexs.empty() &&
             unsmoothed[i] <= unsmoothed[ascending_indexs.back()]) {
        ascending_indexs.pop_back();
      }
      ascending_indexs.push_back(i);
    }

    if (ascending_indexs.front() < i - 2 * window_size) {
      ascending_indexs.pop_front();
    }

    const auto start_index = i - window_size;
    if (0 <= start_index &&
        start_index < static_cast<int>(speed_limits_.size())) {
      speed_limits_[start_index] = unsmoothed[ascending_indexs.front()];
    }
  }
}

void PathHandle::UpdateDebugInfo(
    const CollisionInfo& collision_info,
    const AvpSpeedPlanCollisionInfo::SpeedTaskInteractiveStage&
        interactive_stage,
    const common::PathPoint& future_collision_point,
    const bool is_vehicle_still, const double wheel_mask_distance,
    OpenSpaceInfo* const mutable_open_space_info) {
  if (nullptr == mutable_open_space_info ||
      nullptr == mutable_open_space_info->mutable_speed_plan_collision_info() ||
      nullptr == mutable_open_space_info->mutable_future_collision_point()) {
    return;
  }

  if (AvpSpeedPlanCollisionInfo::WAITREPLAN == interactive_stage &&
      collision_info.collision_type !=
          AvpSpeedPlanCollisionInfo::NO_COLLISION) {
    mutable_open_space_info->set_replan_triggered_by_speed_plan(true);
  } else {
    mutable_open_space_info->set_replan_triggered_by_speed_plan(false);
  }

  auto* speed_plan_collision_info =
      mutable_open_space_info->mutable_speed_plan_collision_info();
  if (current_path_has_collision_count_ >
      config_.path_collision_risk_max_count()) {
    mutable_open_space_info->set_current_path_has_collision_risk(true);
    const auto& collision_segment =
        all_freespace_segments_[current_path_collision_index_].first;
    speed_plan_collision_info->mutable_risk_fs_segment_start()->set_x(
        collision_segment.start().x());
    speed_plan_collision_info->mutable_risk_fs_segment_start()->set_y(
        collision_segment.start().y());
    speed_plan_collision_info->mutable_risk_fs_segment_end()->set_x(
        collision_segment.end().x());
    speed_plan_collision_info->mutable_risk_fs_segment_end()->set_y(
        collision_segment.end().y());
  } else {
    mutable_open_space_info->set_current_path_has_collision_risk(false);
  }
  speed_plan_collision_info->set_is_wheel_mask_valid(pre_wheel_mask_valid_);
  speed_plan_collision_info->set_is_stop_near_wheel_mask(false);
  if (pre_wheel_mask_valid_ && is_vehicle_still &&
      DefinitelyLess(wheel_mask_distance, config_.wheel_mask_stop_accuracy())) {
    speed_plan_collision_info->set_is_stop_near_wheel_mask(true);
  }
  speed_plan_collision_info->set_speed_task_inter_stage(interactive_stage);
  speed_plan_collision_info->set_collision_type(collision_info.collision_type);
  if (collision_info.is_collision) {
    speed_plan_collision_info->set_collision_distance(
        collision_info.curr_collision_distance);
    *mutable_open_space_info->mutable_future_collision_point()
         ->mutable_path_point() = future_collision_point;
  }

  if (collision_info.static_obstacle_collision_info.is_collision) {
    speed_plan_collision_info->set_static_obstacle_id(
        collision_info.static_obstacle_collision_info.collision_obstacle_id);
    speed_plan_collision_info->set_static_obstacle_type(
        collision_info.static_obstacle_collision_info.collision_obstacle_type);
  }

  if (collision_info.moving_obstacle_collision_info.is_collision) {
    speed_plan_collision_info->set_moving_obstacle_id(
        collision_info.moving_obstacle_collision_info.collision_obstacle_id);
    speed_plan_collision_info->set_moving_obstacle_type(
        collision_info.moving_obstacle_collision_info.collision_obstacle_type);
  }

  if (collision_info.freespace_collision_info.is_collision) {
    speed_plan_collision_info->set_freespace_index_in_array(
        collision_info.freespace_collision_info.freespace_segment
            .index_in_freespace_out);
    speed_plan_collision_info->mutable_collision_fs_segment_start()->set_x(
        collision_info.freespace_collision_info.freespace_segment.segment
            .start()
            .x());
    speed_plan_collision_info->mutable_collision_fs_segment_start()->set_y(
        collision_info.freespace_collision_info.freespace_segment.segment
            .start()
            .y());
    speed_plan_collision_info->mutable_collision_fs_segment_end()->set_x(
        collision_info.freespace_collision_info.freespace_segment.segment.end()
            .x());
    speed_plan_collision_info->mutable_collision_fs_segment_end()->set_y(
        collision_info.freespace_collision_info.freespace_segment.segment.end()
            .y());
    *speed_plan_collision_info->mutable_collision_fs_point_flu() =
        collision_info.freespace_collision_info.freespace_flu_point;
    speed_plan_collision_info->set_freespace_type(
        collision_info.freespace_collision_info.freespace_segment.cls_type);
    speed_plan_collision_info->set_freespace_height_type(
        collision_info.freespace_collision_info.freespace_segment.height_type);
  }
  if (collision_info.outside_wheelmask_obstacle_collision_info.is_collision) {
    speed_plan_collision_info->set_static_obstacle_id(
        collision_info.outside_wheelmask_obstacle_collision_info
            .collision_obstacle_id);
    speed_plan_collision_info->set_static_obstacle_type(
        collision_info.outside_wheelmask_obstacle_collision_info
            .collision_obstacle_type);
  }
  speed_plan_collision_info->set_is_use_middle_buffer(is_use_middle_buffer_);
  speed_plan_collision_info->set_bigger_buffer_safe_count(
      bigger_buffer_safe_count_);
  speed_plan_collision_info->set_path_collision_risk_count(
      current_path_has_collision_count_);
}

std::string PathHandle::Process(  // NOLINT
    const DiscretizedPath& path,
    const std::vector<const std::shared_ptr<Obstacle>*>& obstacles,
    const std::shared_ptr<const FreeSpaceOutArray>& freespace_out_array,
    const common::VehicleState& vehicle_state, const bool is_vehicle_still,
    const bool is_forward, const bool is_rpa_direct_mode,
    const OpenSpaceSpeedOptimizerConfig::SpeedBoundInfo& speed_bound_info,
    const bool is_mirror_fold, const OpenSpaceInfo& open_space_info,
    DiscretizedPath* const candidate_path,
    AvpSpeedPlanCollisionInfo::SpeedTaskInteractiveStage* const
        interactive_stage,
    OpenSpaceInfo* const mutable_open_space_info) {
  std::string msg;
  if (nullptr == candidate_path || nullptr == interactive_stage ||
      nullptr == mutable_open_space_info) {
    msg = "path handle process input has nullptr";
    return msg;
  }
  Init(open_space_info.open_space_path_info().open_space_env_structured_info);
  DiscretizedPath collision_check_path;
  if (!CutOffPathByWheelMask(
          path, is_forward,
          open_space_info.open_space_path_info()
              .open_space_env_structured_info.is_parking_inwards,
          open_space_info.is_consider_wheel_mask(),
          open_space_info.open_space_wheel_mask_box(), &collision_check_path)) {
    mutable_open_space_info->mutable_speed_plan_collision_info()
        ->set_is_stop_near_wheel_mask(true);
    msg = "cut off path by wheel mask failed";
    return msg;
  }

  double wheel_mask_distance = collision_check_path.Length();
  CollisionInfo collision_info;
  const auto& under_spot_low_fs_idxs =
      mutable_open_space_info->under_spot_low_fs_idxs();
  const auto& high_curb_fs_idxs = mutable_open_space_info->high_curb_fs_idxs();
  const auto& ignore_fs_idxs = mutable_open_space_info->ignore_fs_idxs();
  if (!UpdateCollisionInfo(
          collision_check_path, obstacles, freespace_out_array,
          under_spot_low_fs_idxs, high_curb_fs_idxs, ignore_fs_idxs,
          vehicle_state, is_forward, is_rpa_direct_mode, is_mirror_fold,
          open_space_info.partitioned_paths(), &collision_info)) {
    msg = "update collision info failed";
    return msg;
  }

  if (!CutOffPathByCollisionInfo(collision_check_path, collision_info,
                                 candidate_path)) {
    msg = "cut off path by collision info failed";
    return msg;
  }
  collision_info.curr_collision_distance =
      collision_check_path.at(collision_info.first_collision_index).s();
  UpdateInteractiveStage(is_vehicle_still, is_rpa_direct_mode, collision_info,
                         interactive_stage);
  UpdateIsUseMiddleBuffer(*interactive_stage, is_forward,
                          open_space_info.partitioned_paths().path_type);
  UpdateSpeedLimits(*candidate_path, speed_bound_info, is_forward,
                    mutable_open_space_info->spd_limit_points());
  UpdateDebugInfo(collision_info, *interactive_stage,
                  collision_check_path.at(collision_info.first_collision_index),
                  is_vehicle_still, wheel_mask_distance,
                  mutable_open_space_info);

  return msg;
}

void PathHandle::UpdatePathCollisionRiskCount(
    const DiscretizedPath& path,
    const planning_internal::PathUpdateStatus::PathType& path_type,
    const size_t cur_path_idx, const bool is_mirror_fold,
    const bool is_forward) {
  if ((last_is_forward_.first && last_is_forward_.second != is_forward) ||
      path.empty()) {
    current_path_has_collision_count_ = 0;
  }

  if (path_type == planning_internal::PathUpdateStatus::SEARCH_EXTENSION_PATH &&
      cur_path_idx == 0) {
    current_path_has_collision_count_ = 0;
    return;
  }

  double path_collision_risk_max_distance = 0.0;
  if (is_vertical_park_in_ && is_narrow_spot_scenario_) {
    path_collision_risk_max_distance =
        config_.path_collision_risk_max_distance_for_narrow_spot();
  } else if (is_nns_adjust_) {
    path_collision_risk_max_distance =
        config_.path_collision_risk_max_distance_for_nns_adjust();
  } else {
    path_collision_risk_max_distance =
        config_.path_collision_risk_max_distance();
  }
  for (const auto& point : path) {
    if (point.s() >= path_collision_risk_max_distance) {
      break;
    }
    if (common::math::CheckCollisionWithVehiclePolygon2d(
            point.x(), point.y(), point.theta(), all_freespace_segments_,
            &current_path_collision_index_)) {
      current_path_has_collision_count_++;
      return;
    }
    if (is_mirror_fold) {
      continue;
    }
    const auto circles =
        common::VehicleConfigHelper::GetMirrorCirclesWithBuffer(
            point.x(), point.y(), point.theta());
    if (circles.size() != 2) {
      AERROR << "used circle size is not 2";
      return;
    }
    if (common::math::CheckCollisionWithCircle(
            circles.front(), high_height_curb_fs_,
            &current_path_collision_index_) ||
        common::math::CheckCollisionWithCircle(
            circles.back(), high_height_curb_fs_,
            &current_path_collision_index_)) {
      current_path_has_collision_count_++;
      return;
    }
  }

  current_path_has_collision_count_ = 0;
}

void PathHandle::UpdateLateralBuffer(
    const OpenSpaceSpeedOptimizerConfig::CollisionBufferInfo& collision_buffer,
    const bool is_use_middle_buffer, const bool is_narrow_spot_scenario,
    const bool is_lat_park_in, const bool is_lat_park_out,
    const planning_internal::PathUpdateStatus::PathType& path_type,
    const size_t cur_path_idx,
    std::pair<double, double>* const lateral_buffer_for_vehicle,
    std::pair<double, double>* const lateral_buffer_for_not_vehicle,
    std::pair<double, double>* const lateral_buffer_for_low_fs) {
  if (nullptr == lateral_buffer_for_vehicle ||
      nullptr == lateral_buffer_for_not_vehicle ||
      nullptr == lateral_buffer_for_low_fs) {
    ADEBUG << "update lateral buffer for speed failed";
    return;
  }
  static constexpr double kDecreaseBufferForSearchExtension = 0.05;
  static constexpr double kDecraeseBufferForParkout = 0.08;
  static constexpr double kDecraeseBufferForParkin = 0.05;
  const bool need_shrink_for_search_extension =
      is_use_middle_buffer &&
      path_type == planning_internal::PathUpdateStatus::SEARCH_EXTENSION_PATH &&
      cur_path_idx == 0;

  // previously init buffer by narrow and default
  if (is_use_middle_buffer && is_narrow_spot_scenario) {
    lateral_buffer_for_vehicle->first = collision_buffer.left_lateral_buffer();
    lateral_buffer_for_vehicle->second =
        collision_buffer.right_lateral_buffer();
  } else if (is_use_middle_buffer != is_narrow_spot_scenario) {
    lateral_buffer_for_vehicle->first =
        collision_buffer.left_lateral_buffer_middle();
    lateral_buffer_for_vehicle->second =
        collision_buffer.right_lateral_buffer_middle();
  } else if (!is_use_middle_buffer && !is_narrow_spot_scenario) {
    lateral_buffer_for_vehicle->first =
        collision_buffer.left_lateral_buffer_bigger();
    lateral_buffer_for_vehicle->second =
        collision_buffer.right_lateral_buffer_bigger();
  }

  lateral_buffer_for_not_vehicle->first =
      collision_buffer.left_lateral_buffer();
  lateral_buffer_for_not_vehicle->second =
      collision_buffer.right_lateral_buffer();

  lateral_buffer_for_low_fs->first =
      collision_buffer.left_lateral_buffer_low_fs();
  lateral_buffer_for_low_fs->second =
      collision_buffer.right_lateral_buffer_low_fs();

  if (need_shrink_for_search_extension) {
    lateral_buffer_for_vehicle->first -= kDecreaseBufferForSearchExtension;
    lateral_buffer_for_vehicle->second -= kDecreaseBufferForSearchExtension;
    lateral_buffer_for_not_vehicle->first -= kDecreaseBufferForSearchExtension;
    lateral_buffer_for_not_vehicle->second -= kDecreaseBufferForSearchExtension;
  }
  // independently shrink low_fs buffer for lateral parking
  if (is_lat_park_in && is_use_middle_buffer) {
    lateral_buffer_for_low_fs->first -= kDecraeseBufferForParkin;
    lateral_buffer_for_low_fs->second -= kDecraeseBufferForParkin;
  } else if (is_lat_park_out) {
    lateral_buffer_for_low_fs->first -= kDecraeseBufferForParkout;
    lateral_buffer_for_low_fs->second -= kDecraeseBufferForParkout;
  } else if (need_shrink_for_search_extension) {
    lateral_buffer_for_low_fs->first -= kDecreaseBufferForSearchExtension;
    lateral_buffer_for_low_fs->second -= kDecreaseBufferForSearchExtension;
  }
}

}  // namespace planning
}  // namespace TL
