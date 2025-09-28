/******************************************************************************
 * Copyright 2017 The TL Authors. All Rights Reserved.
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

#include "planning/prediction/container/obstacles/obstacle.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <tuple>
#include <utility>

#include "common/file/log.h"
#include "common/math/linear_interpolation.h"
#include "common/math/math_utils.h"
#include "common/math/vec2d.h"
#include "common/time/clock.h"
#include "common/util/util.h"
#include "planning/prediction/common/junction_analyzer.h"
#include "planning/prediction/common/prediction_constants.h"
#include "planning/prediction/common/prediction_gflags.h"
#include "planning/prediction/common/prediction_map.h"
#include "planning/prediction/common/prediction_system_gflags.h"
#include "planning/prediction/container/obstacles/obstacle_clusters.h"
#include "planning/prediction/scenario/perception_filter/perception_filter.h"
#include "proto/perception/perception_obstacle.pb.h"
#include "proto/prediction/feature.pb.h"
#include "proto/prediction/scenario.pb.h"

namespace TL {
namespace prediction {
namespace {

using TL::hdmap::JunctionInfo;
using TL::hdmap::LaneInfo;
using TL::perception::PerceptionObstacle;

bool IsClosed(const double x0, const double y0, const double theta0,
              const double x1, const double y1, const double theta1) {
  double angle_diff = std::fabs(common::math::AngleDiff(theta0, theta1));
  double distance = std::hypot(x0 - x1, y0 - y1);
  return distance < FLAGS_distance_threshold_to_junction_exit &&
         angle_diff < FLAGS_angle_threshold_to_junction_exit;
}

}  // namespace

PerceptionObstacle::Type Obstacle::type() const {
  return type_;
}

PerceptionObstacle::SubType Obstacle::Subtype() const {
  return sub_type_;
}

bool Obstacle::IsPedestrian() const {
  return type_ == PerceptionObstacle::PEDESTRIAN;
}

bool Obstacle::IsBicycle() const {
  return (type_ == PerceptionObstacle::BICYCLE ||
          type_ == PerceptionObstacle::CYCLIST);
}

int Obstacle::id() const {
  return id_;
}

double Obstacle::timestamp() const {
  if (feature_history_.empty()) {
    AERROR << "Obstacle [" << id_ << "] has no history";
    return 0.0;
  }
  return feature_history_.rbegin()->timestamp();
}

const Feature& Obstacle::feature(const int i) const {
  if (i >= feature_history_.size()) {
    AERROR << "Obstacle [" << id_ << "] feature i >= feature size";
    return *feature_history_.rbegin();
  }

  return feature_history_[i];
}

Feature* Obstacle::mutable_feature(const int i) {
  if (feature_history_.empty() || i >= feature_history_.size()) {
    AERROR << "Obstacle [" << id_ << "] has invalid feature history";
    return nullptr;
  }
  return &feature_history_[i];
}

const Feature& Obstacle::latest_feature() const {
  if (feature_history_.empty()) {
    AERROR << "Obstacle [" << id_ << "] has no history";
  }
  return *feature_history_.rbegin();
}

const Feature& Obstacle::earliest_feature() const {
  if (feature_history_.empty()) {
    AERROR << "Obstacle [" << id_ << "] has no history";
  }
  return *feature_history_.begin();
}

Feature* Obstacle::mutable_latest_feature() {
  if (feature_history_.empty()) {
    AERROR << "Obstacle [" << id_ << "] has no history";
  }
  return &(*feature_history_.rbegin());
}

int Obstacle::history_size() const {
  return feature_history_.size();
}

int Obstacle::tracking_frame() const {
  return tracking_frame_;
}

double Obstacle::OccludedProb() const {
  return occluded_prob_;
}

bool Obstacle::IsStill() {
  if (!feature_history_.empty()) {
    return feature_history_.rbegin()->is_still();
  }
  return true;
}

bool Obstacle::IsSlow() const {
  const Feature& feature = latest_feature();
  return feature.speed() < FLAGS_slow_obstacle_speed_threshold;
}

bool Obstacle::IsOnLane() const {
  if (feature_history_.empty() || !latest_feature().has_lane() ||
      latest_feature().lane().current_lane_feature().empty() ||
      !latest_feature().lane().has_lane_graph() ||
      latest_feature().lane().lane_graph().lane_sequence().empty()) {
    ADEBUG << "Obstacle [" << id_ << "] is not on lane.";
    // reverse lane
    TL::common::PointENU point_enu;
    point_enu.set_x(latest_feature().position().x());
    point_enu.set_y(latest_feature().position().y());
    double heading = common::math::NormalizeAngle(
        M_PI + latest_feature().velocity_heading());
    auto lane_info_ptr = PredictionMap::GetMostLikelyCurrentLane(
        point_enu, 3.0, heading, FLAGS_max_lane_angle_diff / 4);
    if (lane_info_ptr != nullptr) {
      if (lane_info_ptr->lane().has_left_boundary() &&
          lane_info_ptr->lane().left_boundary().virtual_() &&
          lane_info_ptr->lane().has_right_boundary() &&
          lane_info_ptr->lane().right_boundary().virtual_()) {
        ADEBUG << "Obstacle [" << id_
               << "] is on reverse lane, but reverse lane is virtual.";
        // return false;
      } else {
        ADEBUG << "Obstacle [" << id_ << "] is on reverse lane.";
        return true;
      }
    }
    return false;
  }

  const auto& cur_laneid =
      latest_feature().lane().current_lane_feature().begin()->lane_id();
  auto cur_laneinfo_ptr = PredictionMap::LaneById(cur_laneid);
  // OFF lane模式下采用QCNet predictor
  // 在perception mode下，virtual()是判断OFF lane的一个条件; 该模式下路口判断使用virtual()
  // 而远处补齐的车道也是virtual()，因此也采用了QC；临时改为远处不使用
  if (cur_laneinfo_ptr != nullptr &&
      cur_laneinfo_ptr->lane().has_left_boundary() &&
      cur_laneinfo_ptr->lane().left_boundary().virtual_() &&
      cur_laneinfo_ptr->lane().has_right_boundary() &&
      cur_laneinfo_ptr->lane().right_boundary().virtual_() &&
      latest_feature().has_position_flu() &&
      latest_feature().position_flu().has_x() &&
      std::isless(latest_feature().position_flu().x(), 100.0)) {
    ADEBUG << "Obstacle [" << id_ << "] is on lane, but cur lane is virtual.";
    return false;
  }

  ADEBUG << "Obstacle [" << id_ << "] is on lane.";
  return true;
}

bool Obstacle::ToIgnore() {
  if (feature_history_.empty()) {
    return true;
  }
  return latest_feature().priority().priority() == ObstaclePriority::IGNORE;
}

bool Obstacle::IsNearJunction() {
  if (feature_history_.empty()) {
    return false;
  }
  double pos_x = latest_feature().position().x();
  double pos_y = latest_feature().position().y();
  return PredictionMap::NearJunction({pos_x, pos_y},
                                     FLAGS_junction_search_radius);
}

void Obstacle::CheckHistory(const PerceptionObstacle& perception_obstacle,
                            double timestamp) {
  if (!feature_history_.empty() && tracking_frame_ != 0) {
    const auto& latest_history = latest_feature();
    double delta_t = timestamp - latest_history.timestamp();
    if (std::isless(delta_t, 0.3) && std::isgreater(delta_t, 0.0)) {
      double observed_speed_x =
          (perception_obstacle.position().x() - latest_history.position().x()) /
          delta_t;
      double observed_speed_y =
          (perception_obstacle.position().y() - latest_history.position().y()) /
          delta_t;
      double speed_vibration =
          std::max(std::fabs(latest_history.velocity().x() - observed_speed_x),
                   std::fabs(latest_history.velocity().y() - observed_speed_y));
      if (std::isless(speed_vibration / delta_t, FLAGS_obstacle_max_acc)) {
        return;
      }
    }
    ClearHistory();
  }
}

void Obstacle::FilterJunction(Feature* feature) const {
  // check junction
  if (feature == nullptr) {
    return;
  }

  if (feature->has_lane() &&
      PredictionMap::InJunction(feature->position().x(),
                                feature->position().y(),
                                FLAGS_junction_search_radius)) {
    if (!feature->lane().current_lane_feature().empty()) {
      // in junction, remove the lane if angle_diff too large
      double min_angle_diff = std::numeric_limits<double>::max();
      auto* mutable_current_lane_feature =
          feature->mutable_lane()->mutable_current_lane_feature();
      auto* mutable_nearby_lane_feature =
          feature->mutable_lane()->mutable_nearby_lane_feature();

      for (const auto& lane_feat : feature->lane().current_lane_feature()) {
        if (std::fabs(lane_feat.angle_diff()) < min_angle_diff) {
          min_angle_diff = std::fabs(lane_feat.angle_diff());
        }
      }
      ADEBUG << "id:" << id_ << " min_angle_diff:" << min_angle_diff;

      // about 10°
      static constexpr double angle_buffer = 0.17;

      auto it = mutable_current_lane_feature->begin();
      while (it != mutable_current_lane_feature->end()) {
        if (std::fabs(it->angle_diff()) > min_angle_diff + angle_buffer) {
          ADEBUG << "erase current lane : " << it->ShortDebugString();
          it = mutable_current_lane_feature->erase(it);
        } else {
          ++it;
        }
      }
      it = mutable_nearby_lane_feature->begin();
      while (it != mutable_nearby_lane_feature->end()) {
        if (std::fabs(it->angle_diff()) > min_angle_diff + angle_buffer) {
          ADEBUG << "erase nearby lane : " << it->ShortDebugString();
          it = mutable_nearby_lane_feature->erase(it);
        } else {
          ++it;
        }
      }
      if (mutable_current_lane_feature->empty()) {
        mutable_current_lane_feature->Swap(mutable_nearby_lane_feature);
      }
      ADEBUG << feature->lane().ShortDebugString();
    }
    if (feature->lane().current_lane_feature().empty() &&
        !feature->lane().nearby_lane_feature().empty()) {
      // set Junction Angle Diff = 30°
      static constexpr double nearby_angle_buffer = 0.524;
      auto* mutable_current_lane_feature =
          feature->mutable_lane()->mutable_current_lane_feature();
      auto* mutable_nearby_lane_feature =
          feature->mutable_lane()->mutable_nearby_lane_feature();
      auto it = mutable_nearby_lane_feature->begin();
      while (it != mutable_nearby_lane_feature->end()) {
        if (std::fabs(it->angle_diff()) > nearby_angle_buffer) {
          ADEBUG << "erase nearby lane : " << it->ShortDebugString();
          it = mutable_nearby_lane_feature->erase(it);
        } else {
          ++it;
        }
      }
      mutable_current_lane_feature->Swap(mutable_nearby_lane_feature);
    }
  }
}

void Obstacle::FilterUTurn(Feature* feature) {
  if (feature == nullptr) {
    return;
  }
  // do not consider U-turn lane if current_lane_feat > 1
  auto* mutable_current_lane_feature =
      feature->mutable_lane()->mutable_current_lane_feature();
  if (mutable_current_lane_feature->size() > 1) {
    auto it = mutable_current_lane_feature->begin();
    while (it != mutable_current_lane_feature->end()) {
      if (it->lane_turn_type() == hdmap::Lane::U_TURN) {
        it = mutable_current_lane_feature->erase(it);
      } else {
        ++it;
      }
    }
  }

  auto* mutable_nearby_lane_feature =
      feature->mutable_lane()->mutable_nearby_lane_feature();
  auto it = mutable_nearby_lane_feature->begin();
  while (it != mutable_nearby_lane_feature->end()) {
    if (it->lane_turn_type() == hdmap::Lane::U_TURN) {
      it = mutable_nearby_lane_feature->erase(it);
    } else {
      ++it;
    }
  }
}

bool Obstacle::Insert(const PerceptionObstacle& perception_obstacle,
                      const double timestamp,
                      const int prediction_obstacle_id) {
  if (!perception_obstacle.has_id() || !perception_obstacle.has_type()) {
    ADEBUG << "Perception obstacle has incomplete information; "
              "skip insertion";
    ADEBUG << perception_obstacle.DebugString();
    return false;
  }

  if (ReceivedOlderMessage(timestamp)) {
    AERROR << "Obstacle [" << id_ << "] received an older frame [" << FIXED
           << SETPRECISION(3) << timestamp
           << "] than the most recent timestamp [ " << this->timestamp()
           << "].";
    // return false;
    ClearHistory();
  }

  CheckHistory(perception_obstacle, timestamp);

  // Set ID, Type, and Status of the feature.
  Feature feature;
  if (!SetId(perception_obstacle, &feature, prediction_obstacle_id)) {
    return false;
  }

  SetType(perception_obstacle, &feature);

  SetStatus(perception_obstacle, timestamp, &feature);

  // Set obstacle lane features
  double timestamp_1 = common::Clock::NowInSeconds();

  if (!IsPedestrian()) {
    bool in_junction = PredictionMap::InJunction(feature.position().x(),
                                                 feature.position().y(),
                                                 FLAGS_junction_search_radius);
    if (!(IsBicycle() && in_junction)) {
      SetCurrentLanes(&feature);
      SetNearbyLanes(&feature);
      FilterUTurn(&feature);
      if (in_junction) {
        FilterJunction(&feature);
      }
    }
  }

  double timestamp_2 = common::Clock::NowInSeconds();
  ADEBUG << "set current lanes time " << (timestamp_2 - timestamp_1) * 1000;

  if (FLAGS_prediction_offline_mode ==
      PredictionConstants::kDumpDataForLearning) {
    SetSurroundingLaneIds(&feature, FLAGS_surrounding_lane_search_radius);
  }

  if (FLAGS_adjust_vehicle_heading_by_lane &&
      type_ == PerceptionObstacle::VEHICLE) {
    AdjustHeadingByLane(&feature);
  }

  ClearOldInformation();
  // Insert obstacle feature to history
  InsertFeatureToHistory(&feature);
  SetMotionStatus();
  // Trim historical features
  DiscardOutdatedHistory();
  return true;
}

bool Obstacle::InsertFeature(Feature* feature) {
  type_ = feature->type();
  sub_type_ = feature->sub_type();
  id_ = feature->id();
  InsertFeatureToHistory(feature);
  return true;
}

void Obstacle::ClearOldInformation() {
  if (feature_history_.empty()) {
    return;
  }
  feature_history_.rbegin()->clear_predicted_trajectory();
  Lane* lane = feature_history_.rbegin()->mutable_lane();
  lane->clear_current_lane_feature();
  lane->clear_nearby_lane_feature();
  lane->clear_lane_graph();
  lane->clear_lane_graph_ordered();
  obstacle_conf_.Clear();
}

void Obstacle::ClearHistory() {
  // feature_history_.Clear();
  tracking_frame_ = 0;
  kalman_motion_fusion_.DeInitFilter();
}

bool Obstacle::IsInJunction(const std::string& junction_id) const {
  // TODO(all) Consider if need to use vehicle front rather than position
  if (feature_history_.empty()) {
    AERROR << "Obstacle [" << id_ << "] has no history";
    return false;
  }
  if (junction_id.empty()) {
    return false;
  }
  std::shared_ptr<const JunctionInfo> junction_info_ptr =
      PredictionMap::JunctionById(junction_id);
  if (junction_info_ptr == nullptr) {
    return false;
  }
  const auto& position = latest_feature().position();
  return PredictionMap::IsPointInJunction(position.x(), position.y(),
                                          junction_info_ptr);
}

void Obstacle::BuildJunctionFeature() {
  // If obstacle has no history at all, then exit.
  if (feature_history_.empty()) {
    AERROR << "Obstacle [" << id_ << "] has no history";
    return;
  }
  Feature* latest_feature_ptr = mutable_latest_feature();
  if (nullptr == junction_analyzer_ || nullptr == latest_feature_ptr) {
    return;
  }

  if (history_size() > 1) {
    int index = history_size() - 2;
    feature_history_.at(index).clear_junction_feature();
  }

  junction_analyzer_->SetJunctionFeature(latest_feature_ptr);
}

bool Obstacle::IsCloseToJunctionExit() const {
  if (!HasJunctionFeatureWithExits()) {
    AERROR << "No junction feature found";
    return false;
  }
  const Feature& latest_feature = *feature_history_.rbegin();
  double position_x = latest_feature.position().x();
  double position_y = latest_feature.position().y();
  double raw_velocity_heading = std::atan2(latest_feature.raw_velocity().y(),
                                           latest_feature.raw_velocity().x());
  return std::any_of(latest_feature.junction_feature().junction_exit().begin(),
                     latest_feature.junction_feature().junction_exit().end(),
                     [&](const JunctionExit& junction_exit) {
                       double exit_x = junction_exit.exit_position().x();
                       double exit_y = junction_exit.exit_position().y();
                       double exit_heading = junction_exit.exit_heading();
                       return IsClosed(position_x, position_y,
                                       raw_velocity_heading, exit_x, exit_y,
                                       exit_heading);
                     });
}

void Obstacle::SetJunctionFeatureWithEnterLane(const std::string& enter_lane_id,
                                               Feature* const feature_ptr) {
  feature_ptr->mutable_junction_feature()->CopyFrom(
      junction_analyzer_->GetJunctionFeature(enter_lane_id));
}

void Obstacle::SetJunctionFeatureWithoutEnterLane(Feature* const feature_ptr) {
  // Sanity checks.
  if (!feature_ptr->has_lane()) {
    ADEBUG << "Obstacle [" << id_ << "] has no lane.";
    return;
  }

  // Get the possible lanes that the obstalce is on and their neighbor
  // lanes and treat them as the starting-lane-segments.
  std::vector<std::string> start_lane_ids;
  if (feature_ptr->lane().current_lane_feature_size() > 0) {
    for (const auto& lane_feature :
         feature_ptr->lane().current_lane_feature()) {
      start_lane_ids.emplace_back(lane_feature.lane_id());
    }
  }
  if (feature_ptr->lane().nearby_lane_feature_size() > 0) {
    for (const auto& lane_feature : feature_ptr->lane().nearby_lane_feature()) {
      start_lane_ids.emplace_back(lane_feature.lane_id());
    }
  }
  if (start_lane_ids.empty()) {
    ADEBUG << "Obstacle [" << id_ << "] has no lane in junction";
    return;
  }
  // TODO(kechxu) Maybe output all exits if no start lane found
  feature_ptr->mutable_junction_feature()->CopyFrom(
      junction_analyzer_->GetJunctionFeature(start_lane_ids));
}

void Obstacle::SetStatus(const PerceptionObstacle& perception_obstacle,
                         const double timestamp, Feature* feature) {
  SetTimestamp(perception_obstacle, timestamp, feature);
  SetPolygonPoints(perception_obstacle, feature);
  SetPosition(perception_obstacle, feature);
  SetVelocity(perception_obstacle, feature);
  SetAcceleration(perception_obstacle, feature);
  SetLengthWidthHeight(perception_obstacle, feature);
  SetTheta(perception_obstacle, feature);
  SetSensorType(perception_obstacle, feature);
  SetOccluded(perception_obstacle, feature);

  // SetIsNearJunction(perception_obstacle, feature);
}

void Obstacle::SetStatusWithKalmanFilter(
    const PerceptionObstacle& perception_obstacle, const double timestamp,
    Feature* feature) {
  kalman_motion_fusion_.UpdateWithMeasurement(perception_obstacle, timestamp,
                                              feature);
  SetTimestamp(perception_obstacle, timestamp, feature);
  SetPolygonPoints(perception_obstacle, feature);
  SetTheta(perception_obstacle, feature);
  SetLengthWidthHeight(perception_obstacle, feature);
  SetSensorType(perception_obstacle, feature);
}

bool Obstacle::SetId(const PerceptionObstacle& perception_obstacle,
                     Feature* feature, const int prediction_obstacle_id) {
  int id = prediction_obstacle_id > 0 ? prediction_obstacle_id
                                      : perception_obstacle.id();
  if (id_ < 0) {
    id_ = id;
    ADEBUG << "Obstacle has id [" << id_ << "].";
  } else {
    if (id_ != id) {
      AERROR << "Obstacle [" << id_ << "] has a mismatched ID [" << id
             << "] from perception obstacle.";
      return false;
    }
  }
  feature->set_id(id);
  return true;
}

void Obstacle::SetType(const PerceptionObstacle& perception_obstacle,
                       Feature* feature) {
  type_ = perception_obstacle.type();
  sub_type_ = perception_obstacle.sub_type();

  feature->set_type(perception_obstacle.type());
  feature->set_sub_type(perception_obstacle.sub_type());
}

void Obstacle::SetTimestamp(const PerceptionObstacle& perception_obstacle,
                            const double timestamp, Feature* feature) const {
  UNUSED(perception_obstacle);
  double ts = timestamp;
  feature->set_timestamp(ts);

  ADEBUG << "Obstacle [" << id_ << "] has timestamp [" << FIXED
         << SETPRECISION(3) << ts << "].";
}

void Obstacle::SetPolygonPoints(const PerceptionObstacle& perception_obstacle,
                                Feature* feature) const {
  for (const auto& polygon_point : perception_obstacle.polygon_point()) {
    *feature->add_polygon_point() = polygon_point;
    ADEBUG << "Obstacle [" << id_
           << "] has new corner point:" << polygon_point.DebugString();
  }
}

void Obstacle::SetPosition(const PerceptionObstacle& perception_obstacle,
                           Feature* feature) const {
  *feature->mutable_position() = perception_obstacle.position();
  *feature->mutable_raw_position() = perception_obstacle.position();
  if (perception_obstacle.has_position_flu()) {
    *feature->mutable_position_flu() = perception_obstacle.position_flu();
  }
  ADEBUG << "Obstacle [" << id_
         << "] has position:" << perception_obstacle.position().DebugString();
}

void Obstacle::SetVelocity(const PerceptionObstacle& perception_obstacle,
                           Feature* feature) {
  *feature->mutable_velocity_flu() = perception_obstacle.velocity_flu();
  double velocity_x = 0.0;
  double velocity_y = 0.0;
  double velocity_z = 0.0;

  if (perception_obstacle.has_velocity()) {
    if (perception_obstacle.velocity().has_x()) {
      velocity_x = perception_obstacle.velocity().x();
      if (std::isnan(velocity_x)) {
        AERROR << "Found nan velocity_x from perception obstacle";
        velocity_x = 0.0;
      } else if (velocity_x > 50.0 || velocity_x < -50.0) {
        // AERROR << "Found unreasonable velocity_x from perception obstacle["
        //        << perception_obstacle.id() << "] vx = :" << velocity_x;
      }
    }
    if (perception_obstacle.velocity().has_y()) {
      velocity_y = perception_obstacle.velocity().y();
      if (std::isnan(velocity_y)) {
        AERROR << "Found nan velocity_y from perception obstacle";
        velocity_y = 0.0;
      } else if (velocity_y > 50.0 || velocity_y < -50.0) {
        // AERROR << "Found unreasonable velocity_y from perception obstacle ["
        //        << perception_obstacle.id() << "] vy = :" << velocity_y;
      }
    }
    if (perception_obstacle.velocity().has_z()) {
      velocity_z = perception_obstacle.velocity().z();
      if (std::isnan(velocity_z)) {
        AERROR << "Found nan velocity z from perception obstacle";
        velocity_z = 0.0;
      } else if (velocity_z > 50.0 || velocity_z < -50.0) {
        AERROR << "Found unreasonable velocity_z from perception obstacle";
      }
    }
  }

  feature->mutable_raw_velocity()->set_x(velocity_x);
  feature->mutable_raw_velocity()->set_y(velocity_y);
  feature->mutable_raw_velocity()->set_z(velocity_z);

  double speed = std::hypot(velocity_x, velocity_y);
  double velocity_heading = std::atan2(velocity_y, velocity_x);
  if (FLAGS_adjust_velocity_by_obstacle_heading || speed < 0.1) {
    velocity_heading = perception_obstacle.theta();
  }

  if (FLAGS_adjust_velocity_by_position_shift && tracking_frame() > 0) {
    double diff_x =
        feature->position().x() - feature_history_.rbegin()->position().x();
    double diff_y =
        feature->position().y() - feature_history_.rbegin()->position().y();
    double prev_obstacle_size = std::fmax(feature_history_.rbegin()->length(),
                                          feature_history_.rbegin()->width());
    double obstacle_size =
        std::fmax(perception_obstacle.length(), perception_obstacle.width());
    double size_diff = std::abs(obstacle_size - prev_obstacle_size);
    double shift_thred =
        std::fmax(obstacle_size * FLAGS_valid_position_diff_rate_threshold,
                  FLAGS_valid_position_diff_threshold);
    double size_diff_thred =
        FLAGS_split_rate * std::min(obstacle_size, prev_obstacle_size);
    if (std::fabs(diff_x) > shift_thred && std::fabs(diff_y) > shift_thred &&
        size_diff < size_diff_thred) {
      double shift_heading = std::atan2(diff_y, diff_x);
      double angle_diff =
          common::math::NormalizeAngle(shift_heading - velocity_heading);
      if (std::fabs(angle_diff) > FLAGS_max_lane_angle_diff) {
        ADEBUG << "Shift velocity heading to be " << shift_heading;
        velocity_heading = shift_heading;
      }
    }
    velocity_x = speed * std::cos(velocity_heading);
    velocity_y = speed * std::sin(velocity_heading);
  }

  feature->mutable_velocity()->set_x(velocity_x);
  feature->mutable_velocity()->set_y(velocity_y);
  feature->mutable_velocity()->set_z(velocity_z);
  feature->set_velocity_heading(velocity_heading);
  feature->set_speed(speed);

  ADEBUG << "Obstacle [" << id_ << "] has velocity [" << FIXED
         << SETPRECISION(6) << velocity_x << ", " << velocity_y << ", "
         << velocity_z << "]"
         << "] has velocity heading [" << velocity_heading << "] "
         << "] has speed [" << speed << "].";
}

void Obstacle::AdjustHeadingByLane(Feature* feature) {
  if (!feature->has_lane() || !feature->lane().has_lane_feature()) {
    return;
  }
  double velocity_heading = feature->velocity_heading();
  double lane_heading = feature->lane().lane_feature().lane_heading();
  double angle_diff = feature->lane().lane_feature().angle_diff();
  ADEBUG << "raw_velocity_heading = " << velocity_heading
         << "lane_heading = " << lane_heading << "angle_diff = " << angle_diff;
  if (std::abs(angle_diff) < FLAGS_max_angle_diff_to_adjust_velocity) {
    velocity_heading = lane_heading;
    double speed = feature->speed();
    feature->mutable_velocity()->set_x(speed * std::cos(velocity_heading));
    feature->mutable_velocity()->set_y(speed * std::sin(velocity_heading));
    feature->set_velocity_heading(velocity_heading);
  }
}

void Obstacle::UpdateVelocity(const double theta, double* velocity_x,
                              double* velocity_y, double* velocity_heading,
                              double* speed) {
  *speed = std::hypot(*velocity_x, *velocity_y);
  double angle_diff = common::math::NormalizeAngle(*velocity_heading - theta);
  if (std::fabs(angle_diff) <= FLAGS_max_lane_angle_diff) {
    *velocity_heading = theta;
    *velocity_x = *speed * std::cos(*velocity_heading);
    *velocity_y = *speed * std::sin(*velocity_heading);
  }
}

void Obstacle::SetAcceleration(const PerceptionObstacle& perception_obstacle,
                               Feature* feature) const {
  if (perception_obstacle.has_acceleration_flu()) {
    *feature->mutable_acceleration_flu() =
        perception_obstacle.acceleration_flu();
  }

  if (!perception_obstacle.has_acceleration()) {
    feature->mutable_acceleration()->set_x(0.0);
    feature->mutable_acceleration()->set_y(0.0);
    feature->mutable_t_acceleration()->set_x(0.0);
    feature->mutable_t_acceleration()->set_y(0.0);
    return;
  }

  double velocity_heading = feature->velocity_heading();
  double acc_x = perception_obstacle.acceleration().x();
  double acc_y = perception_obstacle.acceleration().y();
  double acc = acc_x * cos(velocity_heading) + acc_y * sin(velocity_heading);
  // double upper_bound = FLAGS_vehicle_max_linear_acc;
  // double lower_bound = FLAGS_vehicle_min_linear_acc;
  // if (type_ == PerceptionObstacle::PEDESTRIAN) {
  //   upper_bound = FLAGS_pedestrian_max_acc;
  //   lower_bound = FLAGS_pedestrian_min_acc;
  // }

  double coefficient = 1.0;
  if (acc > FLAGS_vehicle_max_linear_acc) {
    coefficient = FLAGS_vehicle_max_linear_acc / acc;
    acc = FLAGS_vehicle_max_linear_acc;
  } else if (acc < FLAGS_vehicle_min_linear_acc) {
    coefficient = FLAGS_vehicle_min_linear_acc / acc;
    acc = FLAGS_vehicle_min_linear_acc;
  }
  acc_x = acc_x * coefficient;
  acc_y = acc_y * coefficient;

  feature->mutable_acceleration()->set_x(acc_x);
  feature->mutable_acceleration()->set_y(acc_y);
  feature->set_acc(acc);
  feature->mutable_t_acceleration()->set_x(
      perception_obstacle.acceleration().x());
  feature->mutable_t_acceleration()->set_y(
      perception_obstacle.acceleration().y());
  feature->mutable_t_acceleration()->set_z(
      perception_obstacle.acceleration().z());

  ADEBUG << "Obstacle [" << id_ << "] has acceleration [" << FIXED
         << SETPRECISION(6) << acc_x << ", " << acc_y << "]"
         << "] has acceleration value [" << acc << "].";
}

void Obstacle::SetTheta(const PerceptionObstacle& perception_obstacle,
                        Feature* feature) const {
  double theta = 0.0;
  double velocity_heading = 0.0;
  bool conflicting_heading = false;
  if (perception_obstacle.has_theta()) {
    theta = perception_obstacle.theta();
  }
  if (feature->has_velocity_heading()) {
    velocity_heading = feature->velocity_heading();
  }
  feature->set_theta(theta);
  feature->set_theta_flu(perception_obstacle.theta_flu());

  double angle_diff =
      std::abs(common::math::AngleDiff(theta, velocity_heading));
  if (feature->type() == PerceptionObstacle::VEHICLE && angle_diff > M_PI / 2) {
    conflicting_heading = true;
  }

  if (FLAGS_adjust_velocity_by_obstacle_heading ||
      feature->speed() < FLAGS_slow_obstacle_speed_threshold ||
      feature->length() > FLAGS_extra_long_veh_length_threshold) {
    if (!conflicting_heading) {
      feature->set_velocity_heading(theta);
    } else {
      feature->set_velocity_heading(theta + M_PI);
    }
    feature->mutable_velocity()->set_x(feature->speed() *
                                       std::cos(feature->velocity_heading()));
    feature->mutable_velocity()->set_y(feature->speed() *
                                       std::sin(feature->velocity_heading()));
  }

  ADEBUG << "Obstacle [" << id_ << "] has theta [" << FIXED << SETPRECISION(6)
         << theta << "].";
}

void Obstacle::SetLengthWidthHeight(
    const PerceptionObstacle& perception_obstacle, Feature* feature) const {
  double length = 0.0;
  double width = 0.0;
  double height = 0.0;

  if (perception_obstacle.has_length()) {
    length = perception_obstacle.length();
  }
  if (perception_obstacle.has_width()) {
    width = perception_obstacle.width();
  }
  if (perception_obstacle.has_height()) {
    height = perception_obstacle.height();
  }

  feature->set_length(length);
  feature->set_width(width);
  feature->set_height(height);

  ADEBUG << "Obstacle [" << id_ << "] has dimension [" << FIXED
         << SETPRECISION(6) << length << ", " << width << ", " << height
         << "].";
}

void Obstacle::SetSensorType(const PerceptionObstacle& perception_obstacle,
                             Feature* feature) {
  feature->mutable_sensor_feature()->mutable_curr_detect_sensor()->CopyFrom(
      perception_obstacle.current_detect_sensor());
}

void Obstacle::SetOccluded(const PerceptionObstacle& perception_obstacle,
                           Feature* feature) {
  if (perception_obstacle.has_occluded_prob()) {
    if (perception_obstacle.occluded_prob() > 2.0 ||
        perception_obstacle.occluded_prob() < 0.0) {
      AERROR << "Found unreasonable occluded_prob from perception obstacle ["
             << perception_obstacle.id()
             << "] occluded_prob = :" << perception_obstacle.occluded_prob();
      SetOccludedProb(-1.0);
    } else {
      double occluded_prob = perception_obstacle.occluded_prob();
      // clear acceleration for targets with a high occlusion ratio.
      static constexpr double kOccludedProbThreshold = 0.5;
      if (occluded_prob > kOccludedProbThreshold) {
        feature->mutable_acceleration()->set_x(0.0);
        feature->mutable_acceleration()->set_y(0.0);
      }
      SetOccludedProb(occluded_prob);
    }
  } else {
    SetOccludedProb(-1.0);
  }
}

void Obstacle::SetIsNearJunction(const PerceptionObstacle& perception_obstacle,
                                 Feature* feature) {
  if (!perception_obstacle.has_position()) {
    return;
  }
  if (!perception_obstacle.position().has_x() ||
      !perception_obstacle.position().has_y()) {
    return;
  }
  double x = perception_obstacle.position().x();
  double y = perception_obstacle.position().y();
  bool is_near_junction =
      PredictionMap::NearJunction({x, y}, FLAGS_junction_search_radius);
  feature->set_is_near_junction(is_near_junction);
}

bool Obstacle::HasJunctionFeatureWithExits() const {
  if (history_size() == 0) {
    return false;
  }
  return latest_feature().has_junction_feature() &&
         latest_feature().junction_feature().junction_exit_size() > 0;
}

// Margin for comparation, copy from hdmap_common.cc "kEpsilon"
constexpr double kExtras = 0.1;

void Obstacle::SetCurrentLanes(Feature* feature) {
  Eigen::Vector2d point(feature->position().x(), feature->position().y());
  double heading = feature->velocity_heading();
  int max_num_lane = FLAGS_max_num_current_lane;
  double max_angle_diff = FLAGS_max_lane_angle_diff;
  double lane_search_radius = FLAGS_lane_search_radius;
  if (PredictionMap::InJunction(point, FLAGS_junction_search_radius)) {
    max_num_lane = FLAGS_max_num_current_lane_in_junction;
    max_angle_diff = FLAGS_max_lane_angle_diff_in_junction;
    lane_search_radius = FLAGS_lane_search_radius_in_junction;
  }
  std::vector<std::shared_ptr<const LaneInfo>> current_lanes;

  current_lanes_id_.clear();

  // 暂时不使用角点判断onlane, 待后续有需要再打开；需要考虑：
  // 1）角点在车道范围，中心点不在；2）轨迹可能存在折线
  std::tuple<double, double, bool, int, double, double, double> params =
      std::make_tuple(heading, lane_search_radius, true, max_num_lane,
                      max_angle_diff, -1.0, -1.0);
  // std::tuple<double, double, bool, int, double, double, double> params =
  //     std::make_tuple(heading, lane_search_radius, true, max_num_lane,
  //                     max_angle_diff, feature->length(), feature->width());

  PredictionMap::OnLane(&lane_id_past_, point, params, &current_lanes);

  if (current_lanes.empty()) {
    ADEBUG << "Obstacle [" << id_ << "] has no current lanes.";
    return;
  }

  double obs_width = feature->width();
  double obs_length = feature->length();

  int curr_lanes_size = static_cast<int>(current_lanes.size());
  std::vector<bool> is_lane_valid(curr_lanes_size, true);
  std::vector<double> s_vec(curr_lanes_size, 0.0);
  std::vector<double> l_vec(curr_lanes_size, 0.0);
  std::vector<double> min_l_vec(curr_lanes_size, 0.0);
  std::vector<double> max_l_vec(curr_lanes_size, 0.0);
  // 考虑到车道重叠导致预测轨迹产生折线, 排除有前后继关系的前驱车道;
  // 考虑到车道前后继之间存在间隙，s方向上增加kExtras长度
  for (int i = 0; i < curr_lanes_size; ++i) {
    const std::shared_ptr<const LaneInfo>& current_lane = current_lanes.at(i);
    if (nullptr == current_lane) {
      is_lane_valid.at(i) = false;
      continue;
    }
    std::string lane_id = current_lane->id().id();
    current_lanes_id_.emplace_back(lane_id);

    ADEBUG << "####considered current lane id is " << lane_id;
    double s = 0.0;
    double l = 0.0;
    PredictionMap::GetProjection(point, current_lane, &s, &l);
    s_vec.at(i) = s;
    l_vec.at(i) = l;

    Eigen::Vector2d flu_raw_p3 = TL::common::math::RotateVector2d(
        {obs_length / 2, -obs_width / 2}, feature->velocity_heading());
    flu_raw_p3.x() += feature->position().x();
    flu_raw_p3.y() += feature->position().y();
    Eigen::Vector2d flu_raw_p4 = TL::common::math::RotateVector2d(
        {obs_length / 2, obs_width / 2}, feature->velocity_heading());
    flu_raw_p4.x() += feature->position().x();
    flu_raw_p4.y() += feature->position().y();
    double min_poly_l = 0.0;
    double max_poly_l = 0.0;
    double tmp_s = 0.0;
    PredictionMap::GetProjection(flu_raw_p3, current_lane, &tmp_s, &min_poly_l);
    PredictionMap::GetProjection(flu_raw_p4, current_lane, &tmp_s, &max_poly_l);
    if (std::isgreater(min_poly_l, max_poly_l)) {
      std::swap(min_poly_l, max_poly_l);
    }
    min_l_vec.at(i) = min_poly_l;
    max_l_vec.at(i) = max_poly_l;

    if (s < 0.0 || s >= current_lane->lane().length() + kExtras) {
      is_lane_valid.at(i) = false;
      continue;
    }
    for (int j = 0; j < i; ++j) {
      const std::shared_ptr<const LaneInfo>& tmp_lane = current_lanes.at(j);
      if (nullptr == tmp_lane) {
        continue;
      }
      if (!is_lane_valid.at(j)) {
        continue;
      }
      if (PredictionMap::IsSuccessorLane(tmp_lane, current_lane)) {
        is_lane_valid.at(i) = false;
        break;
      }
      if (PredictionMap::IsPredecessorLane(tmp_lane, current_lane)) {
        is_lane_valid.at(j) = false;
      }
    }
  }

  auto* mutable_lane = feature->mutable_lane();

  double min_heading_diff = std::numeric_limits<double>::infinity();
  int min_idx = -1;
  for (int i = 0; i < curr_lanes_size; ++i) {
    const std::shared_ptr<const LaneInfo>& current_lane = current_lanes.at(i);
    if (current_lane == nullptr) {
      continue;
    }

    if (!is_lane_valid.at(i)) {
      continue;
    }

    std::string lane_id = current_lane->id().id();
    int turn_type = PredictionMap::LaneTurnType(lane_id);
    ADEBUG << "####final current lane id is " << lane_id;
    common::math::Vec2d vec_point(point[0], point[1]);
    double distance = 0.0;
    common::PointENU nearest_point =
        current_lane->GetNearestPoint(vec_point, &distance);
    double nearest_point_heading =
        PredictionMap::PathHeading(current_lane, nearest_point);
    double angle_diff = common::math::AngleDiff(heading, nearest_point_heading);
    double left = 0.0;
    double right = 0.0;
    double s = s_vec.at(i);
    double l = l_vec.at(i);
    current_lane->GetWidth(s, &left, &right);
    LaneFeature* lane_feature = mutable_lane->add_current_lane_feature();
    lane_feature->set_lane_turn_type(turn_type);
    lane_feature->set_lane_id(lane_id);
    lane_feature->set_lane_s(s);
    lane_feature->set_lane_l(l);
    lane_feature->set_angle_diff(angle_diff);
    lane_feature->set_lane_heading(nearest_point_heading);
    lane_feature->set_dist_to_left_boundary(left - l);
    lane_feature->set_dist_to_right_boundary(right + l);
    lane_feature->set_lane_type(current_lane->lane().type());
    lane_feature->mutable_lane_point()->set_x(nearest_point.x());
    lane_feature->mutable_lane_point()->set_y(nearest_point.y());
    lane_feature->set_poly_min_l(min_l_vec.at(i));
    lane_feature->set_poly_max_l(max_l_vec.at(i));
    if (std::fabs(angle_diff) < min_heading_diff) {
      min_idx = mutable_lane->current_lane_feature_size() - 1;
      min_heading_diff = std::fabs(angle_diff);
    }
    clusters_ptr_->AddObstacle(id_, lane_id, s, l, feature->speed());
    ADEBUG << "Obstacle [" << id_ << "] has current lanes ["
           << lane_feature->ShortDebugString() << "].";
  }
  if (min_idx >= 0) {
    mutable_lane->mutable_lane_feature()->CopyFrom(
        mutable_lane->current_lane_feature(min_idx));
  }
}

void Obstacle::SetNearbyLanes(Feature* feature) {
  Eigen::Vector2d point(feature->position().x(), feature->position().y());
  int max_num_lane = FLAGS_max_num_nearby_lane;
  double lane_search_radius = FLAGS_lane_search_radius;
  if (PredictionMap::InJunction(point, FLAGS_junction_search_radius)) {
    max_num_lane = FLAGS_max_num_nearby_lane_in_junction;
    lane_search_radius = FLAGS_lane_search_radius_in_junction;
  }
  double theta = feature->velocity_heading();

  std::vector<std::shared_ptr<const LaneInfo>> current_lanes;
  for (const auto& current_lane_id : current_lanes_id_) {
    std::shared_ptr<const LaneInfo> lane =
        PredictionMap::LaneById(current_lane_id);
    if (lane != nullptr) {
      current_lanes.emplace_back(lane);
    }
  }

  std::vector<std::shared_ptr<const LaneInfo>> nearby_lanes;
  PredictionMap::NearbyLanesByCurrentLanes(point, theta, lane_search_radius,
                                           current_lanes, max_num_lane, kExtras,
                                           &nearby_lanes);
  if (nearby_lanes.empty()) {
    ADEBUG << "Obstacle [" << id_ << "] has no nearby lanes.";
    return;
  }

  for (const auto& nearby_lane : nearby_lanes) {
    if (nearby_lane == nullptr) {
      continue;
    }

    // Ignore bike and sidewalk lanes for vehicles
    if (type_ == PerceptionObstacle::VEHICLE) {
      if (nearby_lane->lane().has_type()) {
        if (!PredictionMap::IsDrivableLane(nearby_lane->lane().type()) ||
            nearby_lane->lane().type() == hdmap::Lane::EMERGENCY_LANE ||
            nearby_lane->lane().type() == hdmap::Lane::BIKING) {
          continue;
        }
      }
    }

    double s = -1.0;
    double l = 0.0;
    PredictionMap::GetProjection(point, nearby_lane, &s, &l);
    if (s < 0.0 || s >= nearby_lane->total_length() + kExtras) {
      continue;
    }
    int turn_type = PredictionMap::LaneTurnType(nearby_lane->id().id());
    double heading = feature->velocity_heading();
    double angle_diff = 0.0;
    hdmap::MapPathPoint nearest_point;
    if (PredictionMap::ProjectionFromLane(nearby_lane, s, &nearest_point)) {
      angle_diff = common::math::AngleDiff(nearest_point.heading(), heading);
    }

    double left = 0.0;
    double right = 0.0;
    nearby_lane->GetWidth(s, &left, &right);

    double obs_width = feature->width();
    double obs_length = feature->length();
    Eigen::Vector2d flu_raw_p3 = TL::common::math::RotateVector2d(
        {obs_length / 2, -obs_width / 2}, feature->velocity_heading());
    flu_raw_p3.x() += feature->position().x();
    flu_raw_p3.y() += feature->position().y();
    Eigen::Vector2d flu_raw_p4 = TL::common::math::RotateVector2d(
        {obs_length / 2, obs_width / 2}, feature->velocity_heading());
    flu_raw_p4.x() += feature->position().x();
    flu_raw_p4.y() += feature->position().y();
    double min_poly_l = 0.0;
    double max_poly_l = 0.0;
    double tmp_s = 0.0;
    PredictionMap::GetProjection(flu_raw_p3, nearby_lane, &tmp_s, &min_poly_l);
    PredictionMap::GetProjection(flu_raw_p4, nearby_lane, &tmp_s, &max_poly_l);
    if (std::isgreater(min_poly_l, max_poly_l)) {
      std::swap(min_poly_l, max_poly_l);
    }

    LaneFeature* lane_feature =
        feature->mutable_lane()->add_nearby_lane_feature();

    lane_feature->set_lane_turn_type(turn_type);
    lane_feature->set_lane_id(nearby_lane->id().id());
    lane_feature->set_lane_s(s);
    lane_feature->set_lane_l(l);
    lane_feature->set_angle_diff(angle_diff);
    lane_feature->set_lane_heading(nearest_point.heading());
    lane_feature->set_dist_to_left_boundary(left - l);
    lane_feature->set_dist_to_right_boundary(right + l);
    lane_feature->set_lane_type(nearby_lane->lane().type());
    lane_feature->mutable_lane_point()->set_x(nearest_point.x());
    lane_feature->mutable_lane_point()->set_y(nearest_point.y());
    lane_feature->set_poly_min_l(min_poly_l);
    lane_feature->set_poly_max_l(max_poly_l);
    ADEBUG << "Obstacle [" << id_ << "] has nearby lanes ["
           << lane_feature->ShortDebugString() << "]";
  }
}

void Obstacle::SetSurroundingLaneIds(Feature* feature, const double radius) {
  Eigen::Vector2d point(feature->position().x(), feature->position().y());
  std::vector<std::string> lane_ids =
      PredictionMap::NearbyLaneIds(point, radius);
  for (const auto& lane_id : lane_ids) {
    feature->add_surrounding_lane_id(lane_id);
    std::shared_ptr<const LaneInfo> lane_info =
        PredictionMap::LaneById(lane_id);
    if (lane_info == nullptr) {
      continue;
    }
    if (lane_info->IsOnLane(
            {feature->position().x(), feature->position().y()})) {
      feature->add_within_lane_id(lane_id);
    }
  }
}

bool Obstacle::HasJunctionExitLane(
    const LaneSequence& lane_sequence,
    const std::unordered_set<std::string>& exit_lane_id_set) const {
  const Feature& feature = latest_feature();
  if (!feature.has_junction_feature()) {
    AERROR << "Obstacle [" << id_ << "] has no junction feature.";
    return false;
  }
  return std::any_of(lane_sequence.lane_segment().begin(),
                     lane_sequence.lane_segment().end(),
                     [&](const LaneSegment& lane_segment) {
                       return exit_lane_id_set.find(lane_segment.lane_id()) !=
                              exit_lane_id_set.end();
                     });
}

void Obstacle::SetMergeInfo() {
  // Sanity checks.
  if (history_size() == 0) {
    AERROR << "No feature found.";
    return;
  }
  Feature* feature = mutable_latest_feature();
  if (nullptr == feature) {
    AERROR << "No mutable_latest_feature found.";
    return;
  }
  if (!feature->has_lane() || !feature->lane().has_lane_graph()) {
    return;
  }
  int num_lane_sequence = feature->lane().lane_graph().lane_sequence_size();
  for (int i = 0; i < num_lane_sequence; ++i) {
    auto* lane_seq_ptr =
        feature->mutable_lane()->mutable_lane_graph()->mutable_lane_sequence(i);
    lane_seq_ptr->clear_merge_lane_idx();
    if (lane_seq_ptr->lane_segment_size() < 2) {
      continue;
    }
    if (!(lane_seq_ptr->has_vehicle_on_lane() &&
          lane_seq_ptr->vehicle_on_lane())) {
      continue;
    }
    int merge_lane_idx = -1;
    double dist_to_merge_lane = 0.0;
    std::shared_ptr<const LaneInfo> curr_lane_ptr;
    for (int j = 1; j < lane_seq_ptr->lane_segment_size(); ++j) {
      curr_lane_ptr =
          PredictionMap::LaneById(lane_seq_ptr->lane_segment(j).lane_id());
      if (nullptr == curr_lane_ptr) {
        break;
      }
      if (curr_lane_ptr->lane().predecessor_id_size() > 1) {
        merge_lane_idx = j;
        break;
      }
      dist_to_merge_lane += curr_lane_ptr->lane().length();
    }
    if (merge_lane_idx < 1) {
      continue;
    }
    mutable_latest_feature()->mutable_intent()->set_type(
        ObstacleIntent::MERGING);

    lane_seq_ptr->set_merge_lane_idx(merge_lane_idx);
    dist_to_merge_lane -= lane_seq_ptr->lane_s();
    lane_seq_ptr->set_dist_to_merge_lane(dist_to_merge_lane);
    std::shared_ptr<const LaneInfo> prev_lane_ptr = PredictionMap::LaneById(
        lane_seq_ptr->lane_segment(merge_lane_idx - 1).lane_id());
    if (nullptr == prev_lane_ptr) {
      continue;
    }
    double accumulate_s = 0.0;
    double lateral = 0.0;
    curr_lane_ptr->GetProjection(
        common::util::PointFactory::ToVec2d(prev_lane_ptr->GetSmoothPoint(0.0)),
        &accumulate_s, &lateral);
    if (std::isless(lateral, 0.0)) {
      lane_seq_ptr->set_lane_seq_transition(LaneSequence::LEFT_MERGING);
    } else if (std::isgreater(lateral, 0.0)) {
      lane_seq_ptr->set_lane_seq_transition(LaneSequence::RIGHT_MERGING);
    } else {
      lane_seq_ptr->set_lane_seq_transition(LaneSequence::CONTINUE);
    }
  }
}

void Obstacle::BuildLaneGraph() {
  // Sanity checks.
  if (history_size() == 0) {
    AERROR << "No feature found.";
    return;
  }

  Feature* feature = mutable_latest_feature();
  // No need to BuildLaneGraph for those non-moving obstacles.
  if (feature->is_still() && id_ != FLAGS_ego_vehicle_id) {
    ADEBUG << "Not build lane graph for still obstacle";
    return;
  }
  if (feature->lane().lane_graph().lane_sequence_size() > 0) {
    ADEBUG << "Not build lane graph for an old obstacle";
    return;
  }
  double speed = feature->speed();
  double t_max = FLAGS_prediction_trajectory_time_length;
  auto estimated_move_distance = speed * t_max;

  double min_trajectory_spatial_length =
      FLAGS_min_prediction_trajectory_spatial_length;
  if (id_ == FLAGS_ego_vehicle_id) {
    min_trajectory_spatial_length = FLAGS_min_ego_trajectory_spatial_length;
  }
  double road_graph_search_distance =
      std::fmax(estimated_move_distance, min_trajectory_spatial_length);

  bool is_in_junction = HasJunctionFeatureWithExits();
  std::unordered_set<std::string> exit_lane_id_set;
  if (is_in_junction) {
    for (const auto& exit : feature->junction_feature().junction_exit()) {
      exit_lane_id_set.insert(exit.exit_lane_id());
    }
  }
  bool consider_lane_split = true;

  // BuildLaneGraph for current lanes:
  // Go through all the LaneSegments in current_lane,
  // construct up to max_num_current_lane of them.
  int seq_id = 0;
  int curr_lane_count = 0;
  for (const auto& lane : feature->lane().current_lane_feature()) {
    std::shared_ptr<const LaneInfo> lane_info =
        PredictionMap::LaneById(lane.lane_id());
    if (lane_info == nullptr || !lane.has_angle_diff()) {
      continue;
    }
    bool identical_heading = lane.angle_diff() < M_PI / 2;
    LaneGraph lane_graph = TL::prediction::ObstacleClusters::GetLaneGraph(
        lane.lane_s(), lane.lane_l(), road_graph_search_distance,
        consider_lane_split, identical_heading, lane_info);
    if (lane_graph.lane_sequence_size() > 0) {
      ++curr_lane_count;
    }
    for (const auto& lane_seq : lane_graph.lane_sequence()) {
      if (is_in_junction && !HasJunctionExitLane(lane_seq, exit_lane_id_set)) {
        continue;
      }
      LaneSequence* lane_seq_ptr =
          feature->mutable_lane()->mutable_lane_graph()->add_lane_sequence();
      lane_seq_ptr->CopyFrom(lane_seq);
      lane_seq_ptr->set_lane_sequence_id(seq_id++);
      lane_seq_ptr->set_lane_s(lane.lane_s());
      lane_seq_ptr->set_lane_l(lane.lane_l());
      lane_seq_ptr->set_vehicle_on_lane(true);
      lane_seq_ptr->set_lane_type(lane.lane_type());
      lane_seq_ptr->set_using_lane_heading(false);
      lane_seq_ptr->set_poly_max_l(lane.poly_max_l());
      lane_seq_ptr->set_poly_min_l(lane.poly_min_l());
      SetLaneSequenceStopSign(lane_seq_ptr);
      ADEBUG << "Obstacle [" << id_ << "],lane s = " << lane.lane_s()
             << "lane l = " << lane.lane_l() << " set a curr lane sequence ["
             << lane_seq.ShortDebugString() << "].";
    }
    if (curr_lane_count >= FLAGS_max_num_current_lane) {
      break;
    }
  }

  // int current_lane_seq_size = feature->lane().lane_graph().lane_sequence_size();
  // BuildLaneGraph for neighbor lanes.
  int nearby_lane_count = 0;
  for (const auto& lane : feature->lane().nearby_lane_feature()) {
    std::shared_ptr<const LaneInfo> lane_info =
        PredictionMap::LaneById(lane.lane_id());
    if (lane_info == nullptr || !lane.has_angle_diff()) {
      continue;
    }
    bool identical_heading = lane.angle_diff() < M_PI / 2;
    LaneGraph lane_graph = TL::prediction::ObstacleClusters::GetLaneGraph(
        lane.lane_s(), lane.lane_l(), road_graph_search_distance, false,
        identical_heading, lane_info);
    if (lane_graph.lane_sequence_size() > 0) {
      ++nearby_lane_count;
    }
    for (const auto& lane_seq : lane_graph.lane_sequence()) {
      if (is_in_junction && !HasJunctionExitLane(lane_seq, exit_lane_id_set)) {
        continue;
      }

      LaneSequence* lane_seq_ptr =
          feature->mutable_lane()->mutable_lane_graph()->add_lane_sequence();
      lane_seq_ptr->CopyFrom(lane_seq);
      lane_seq_ptr->set_lane_sequence_id(seq_id++);
      lane_seq_ptr->set_lane_s(lane.lane_s());
      lane_seq_ptr->set_lane_l(lane.lane_l());
      lane_seq_ptr->set_vehicle_on_lane(false);
      lane_seq_ptr->set_lane_type(lane.lane_type());
      lane_seq_ptr->set_using_lane_heading(false);
      lane_seq_ptr->set_poly_max_l(lane.poly_max_l());
      lane_seq_ptr->set_poly_min_l(lane.poly_min_l());
      SetLaneSequenceStopSign(lane_seq_ptr);
      ADEBUG << "Obstacle [" << id_ << "],lane s = " << lane.lane_s()
             << "lane l = " << lane.lane_l() << " set a near lane sequence ["
             << lane_seq.ShortDebugString() << "].";
    }
    if (nearby_lane_count >= FLAGS_max_num_nearby_lane) {
      break;
    }
  }

  if (feature->has_lane() && feature->lane().has_lane_graph()) {
    SetLanePoints(feature);
  }
  ADEBUG << "Obstacle [" << id_ << "] set lane graph features.";
}

void Obstacle::SetObsMergingInfoAccording2Ego(
    const std::shared_ptr<const LaneInfo>& lane_info_ptr, int level,
    double dist, const std::string& ignore_lane_id) {

  if (nullptr == lane_info_ptr) {
    return;
  }

  std::queue<std::pair<std::shared_ptr<const LaneInfo>, int>> lane_info_que;
  lane_info_que.emplace(lane_info_ptr, 1);
  std::queue<double> dist_que;
  dist_que.push(0.0);

  int h = 0;
  while (!lane_info_que.empty()) {
    auto tmp_lane_info = lane_info_que.front();
    lane_info_que.pop();
    double tmp_dist = dist_que.front();
    dist_que.pop();

    auto tmp_lane_id = tmp_lane_info.first->lane().id().id();
    if (tmp_lane_id != lane_info_ptr->id().id() &&
        (std::any_of(latest_feature().lane().current_lane_feature().begin(),
                     latest_feature().lane().current_lane_feature().end(),
                     [tmp_lane_id](const auto& lane) {
                       return tmp_lane_id == lane.lane_id();
                     }))) {
      mutable_latest_feature()->mutable_intent()->set_type(
          ObstacleIntent::MERGING);
      return;
    }

    h = std::max(h, tmp_lane_info.second + 1);
    if (h > level) {
      return;
    }
    if (std::isgreater(tmp_dist, dist)) {
      continue;
    }

    for (const auto& predecessor_lane :
         tmp_lane_info.first->lane().predecessor_id()) {
      const auto& predecessor_lane_ptr =
          PredictionMap::LaneById(predecessor_lane.id());
      if (nullptr == predecessor_lane_ptr ||
          predecessor_lane_ptr->id().id() == ignore_lane_id) {
        continue;
      }
      lane_info_que.emplace(predecessor_lane_ptr, tmp_lane_info.second + 1);
      dist_que.push(tmp_dist + predecessor_lane_ptr->total_length());
    }
  }
}

bool Obstacle::IsStop() const {
  // 临时方案，防止STOP意图覆盖MERGING，利于决策
  const Feature& feature = latest_feature();
  if (feature.type() != PerceptionObstacle::PEDESTRIAN) {
    if (feature.speed() <= FLAGS_suppose_to_stop_speed_threshold &&
        feature.acc() <= FLAGS_suppose_to_stop_acc_threshold &&
        feature.speed() / (-feature.acc()) <
            FLAGS_prediction_trajectory_time_length / 2.0) {
      // feature->mutable_intent()->set_type(ObstacleIntent::STOP);
      return true;
    }
  }
  return false;
}

void Obstacle::MakeDecision() {
  // Feature* feature = mutable_latest_feature();
  // // Make longitudinal decision
  // // stop: low speed + speed / deceleration < 4.0s
  // if (feature->type() != PerceptionObstacle::PEDESTRIAN) {
  //   if (feature->speed() <= FLAGS_suppose_to_stop_speed_threshold &&
  //       feature->acc() <= FLAGS_suppose_to_stop_acc_threshold &&
  //       feature->speed() / (-feature->acc()) <
  //           FLAGS_prediction_trajectory_time_length / 2.0) {
  //     feature->mutable_intent()->set_type(ObstacleIntent::STOP);
  //     return;
  //   }
  // }
  if (IsStop()) {
    return;
  }
  MakeHesitantLaneChangeDecision();
}

void Obstacle::MakeHesitantLaneChangeDecision() {
  // If a lane is invaded by more than 10% for 3 consecutive frames, set it to Hesitant State
  // In HesitantState, if no lane is invaded for 5 consecutive frames, cancel Hesitant State
  constexpr static int enter_count = 3;
  constexpr static int exit_count = 5;

  if (IsStill()) {
    hesitant_lane_change_state_ = false;
    return;
  }

  auto exit_hesitant =
      [](const Feature& feature,
         const std::shared_ptr<const LaneInfo>& lane_info) -> bool {
    const auto& position =
        Eigen::Vector2d(feature.position().x(), feature.position().y());
    double lane_s = 0.0;
    double lane_l = 0.0;
    if (!PredictionMap::GetProjection(position, lane_info, &lane_s, &lane_l)) {
      AERROR << "Failed in getting lane s and lane l";
      return false;
    }

    double lane_width = lane_info->GetWidth(lane_s);
    return std::abs(lane_l) - feature.width() * 0.5 > lane_width * 0.5;
  };

  auto enter_hesitant =
      [](const Feature& feature,
         const std::shared_ptr<const LaneInfo>& lane_info) -> bool {
    const auto& position =
        Eigen::Vector2d(feature.position().x(), feature.position().y());
    double lane_s = 0.0;
    double lane_l = 0.0;
    if (!PredictionMap::GetProjection(position, lane_info, &lane_s, &lane_l)) {
      AERROR << "Failed in getting lane s and lane l";
      return false;
    }
    double lane_width = lane_info->GetWidth(lane_s);
    return std::abs(lane_l) - feature.width() * 0.5 <
           lane_width * 0.5 - feature.width() * FLAGS_intrusion_width_ratio;
  };
  bool curr_hesitant = false;

  if (hesitant_lane_change_state_) {
    if (history_size() < exit_count) {
      hesitant_lane_change_state_ = false;
      return;
    }

    curr_hesitant = false;
    for (const auto& lane : latest_feature().lane().nearby_lane_feature()) {
      std::shared_ptr<const LaneInfo> lane_info =
          PredictionMap::LaneById(lane.lane_id());
      for (int i = 0; i < exit_count; ++i) {
        const auto& feature = feature_history_.at(history_size() - 1 - i);
        if (!exit_hesitant(feature, lane_info)) {
          curr_hesitant = true;
          break;
        }
      }
      if (curr_hesitant) {
        break;
      }
    }
    if (!curr_hesitant) {
      ADEBUG << "Obstacle [" << id_ << "] exit hesitant state.";
      hesitant_lane_change_state_ = curr_hesitant;
    }
  } else {
    if (history_size() < enter_count) {
      return;
    }
    curr_hesitant = true;
    for (const auto& lane : latest_feature().lane().nearby_lane_feature()) {
      std::shared_ptr<const LaneInfo> lane_info =
          PredictionMap::LaneById(lane.lane_id());
      for (int i = 0; i < enter_count; ++i) {
        const auto& feature = feature_history_.at(history_size() - 1 - i);
        if (!enter_hesitant(feature, lane_info)) {
          curr_hesitant = false;
          break;
        }
      }
      if (curr_hesitant) {
        break;
      }
    }
    if (curr_hesitant) {
      ADEBUG << "Obstacle [" << id_ << "] enter hesitant state.";
      hesitant_lane_change_state_ = curr_hesitant;
    }
  }
}

void Obstacle::BuildVectorNetGraph() {
  Feature* feature = mutable_latest_feature();
  common::PointENU center_point = common::util::PointFactory::ToPointENU(
      feature->position().x(), feature->position().y());

  VectorNet::Query(center_point, feature->mutable_vector_net_feature());

  auto* vector_net_pb = feature->mutable_vector_net_feature();
  vector_net_pb->mutable_center()->set_x(feature->position().x());
  vector_net_pb->mutable_center()->set_y(feature->position().y());
  vector_net_pb->set_theta(feature->velocity_heading());
}

void Obstacle::SetLaneSequenceStopSign(LaneSequence* lane_sequence_ptr) {
  // Set the nearest stop sign along the lane sequence
  if (lane_sequence_ptr->lane_segment().empty()) {
    return;
  }
  double accumulate_s = 0.0;
  for (const LaneSegment& lane_segment : lane_sequence_ptr->lane_segment()) {
    const StopSign& stop_sign =
        clusters_ptr_->QueryStopSignByLaneId(lane_segment.lane_id());
    if (stop_sign.has_stop_sign_id() &&
        stop_sign.lane_s() + accumulate_s > lane_sequence_ptr->lane_s()) {
      lane_sequence_ptr->mutable_stop_sign()->CopyFrom(stop_sign);
      lane_sequence_ptr->mutable_stop_sign()->set_lane_sequence_s(
          stop_sign.lane_s() + accumulate_s);
      ADEBUG << "Set StopSign for LaneSequence ["
             << lane_sequence_ptr->lane_sequence_id() << "].";
      break;
    }
    accumulate_s += lane_segment.total_length();
  }
}

// void Obstacle::GetNeighborLaneSegments(
//     const std::shared_ptr<const LaneInfo>& center_lane_info, bool is_left,
//     int recursion_depth, std::list<std::string>* const lane_ids_ordered,
//     std::unordered_set<std::string>* const existing_lane_ids) {
//   // Exit recursion if reached max num of allowed search depth.
//   if (recursion_depth <= 0) {
//     return;
//   }
//   if (is_left) {
//     std::vector<std::string> curr_left_lane_ids;
//     for (const auto& left_lane_id :
//          center_lane_info->lane().left_neighbor_forward_lane_id()) {
//       if (left_lane_id.has_id()) {
//         const std::string& lane_id = left_lane_id.id();
//         // If haven't seen this lane id before.
//         if (existing_lane_ids->count(lane_id) == 0) {
//           existing_lane_ids->insert(lane_id);
//           lane_ids_ordered->push_front(lane_id);
//           curr_left_lane_ids.emplace_back(lane_id);
//         }
//       }
//     }

//     for (const std::string& lane_id : curr_left_lane_ids) {
//       auto lane_info = PredictionMap::LaneById(lane_id);
//       if (lane_info == nullptr) {
//         continue;
//       }
//       GetNeighborLaneSegments(lane_info, true, recursion_depth - 1,
//                               lane_ids_ordered, existing_lane_ids);
//     }
//   } else {
//     std::vector<std::string> curr_right_lane_ids;
//     for (const auto& right_lane_id :
//          center_lane_info->lane().right_neighbor_forward_lane_id()) {
//       if (right_lane_id.has_id()) {
//         const std::string& lane_id = right_lane_id.id();
//         // If haven't seen this lane id before.
//         if (existing_lane_ids->count(lane_id) == 0) {
//           existing_lane_ids->insert(lane_id);
//           lane_ids_ordered->emplace_back(lane_id);
//           curr_right_lane_ids.emplace_back(lane_id);
//         }
//       }
//     }

//     for (const std::string& lane_id : curr_right_lane_ids) {
//       GetNeighborLaneSegments(PredictionMap::LaneById(lane_id), false,
//                               recursion_depth - 1, lane_ids_ordered,
//                               existing_lane_ids);
//     }
//   }
// }

// void Obstacle::BuildLaneGraphFromLeftToRight() {
//   // Sanity checks.
//   if (history_size() == 0) {
//     AERROR << "No feature found.";
//     return;
//   }

//   // No need to BuildLaneGraph for those non-moving obstacles.
//   Feature* feature = mutable_latest_feature();
//   if (feature->is_still()) {
//     ADEBUG << "Don't build lane graph for non-moving obstacle.";
//     return;
//   }
//   if (feature->lane().lane_graph_ordered().lane_sequence_size() > 0) {
//     ADEBUG << "Don't build lane graph for an old obstacle.";
//     return;
//   }
//   // double speed = feature->speed();
//   double road_graph_search_distance = 50.0 * 0.95;  // (45mph for 3sec)
//   // std::fmax(speed * FLAGS_prediction_trajectory_time_length +
//   //               0.5 * FLAGS_vehicle_max_linear_acc *
//   //               FLAGS_prediction_trajectory_time_length *
//   //               FLAGS_prediction_trajectory_time_length,
//   //           FLAGS_min_prediction_trajectory_spatial_length);

//   // Treat the most probable lane_segment as the center, put its left
//   // and right neighbors into a vector following the left-to-right order.
//   if (!feature->has_lane() || !feature->lane().has_lane_feature()) {
//     return;
//   }

//   bool is_in_junction = HasJunctionFeatureWithExits();
//   std::unordered_set<std::string> exit_lane_id_set;
//   if (is_in_junction) {
//     for (const auto& exit : feature->junction_feature().junction_exit()) {
//       exit_lane_id_set.insert(exit.exit_lane_id());
//     }
//   }

//   std::shared_ptr<const LaneInfo> center_lane_info =
//       PredictionMap::LaneById(feature->lane().lane_feature().lane_id());
//   if (center_lane_info == nullptr) {
//     return;
//   }
//   std::list<std::string> lane_ids_ordered_list;
//   std::unordered_set<std::string> existing_lane_ids;
//   GetNeighborLaneSegments(center_lane_info, true, 2, &lane_ids_ordered_list,
//                           &existing_lane_ids);
//   lane_ids_ordered_list.emplace_back(feature->lane().lane_feature().lane_id());
//   existing_lane_ids.insert(feature->lane().lane_feature().lane_id());
//   GetNeighborLaneSegments(center_lane_info, false, 2, &lane_ids_ordered_list,
//                           &existing_lane_ids);

//   const std::vector<std::string> lane_ids_ordered(lane_ids_ordered_list.begin(),
//                                                   lane_ids_ordered_list.end());
//   // TODO(all): sort the lane_segments from left to right (again)
//   //            to double-check and make sure it's well sorted.
//   // Build lane_graph for every lane_segment and update it into proto.
//   int seq_id = 0;
//   for (const std::string& lane_id : lane_ids_ordered) {
//     // Construct the local lane_graph based on the current lane_segment.
//     bool vehicle_is_on_lane = (lane_id == center_lane_info->lane().id().id());
//     std::shared_ptr<const LaneInfo> curr_lane_info =
//         PredictionMap::LaneById(lane_id);
//     if (curr_lane_info == nullptr) {
//       continue;
//     }
//     LaneGraph local_lane_graph = clusters_ptr_->GetLaneGraphWithoutMemorizing(
//         feature->lane().lane_feature().lane_s(),
//         feature->lane().lane_feature().lane_l(), road_graph_search_distance,
//         true, curr_lane_info);
//     // Update it into the Feature proto
//     for (const auto& lane_seq : local_lane_graph.lane_sequence()) {
//       if (is_in_junction && !HasJunctionExitLane(lane_seq, exit_lane_id_set)) {
//         continue;
//       }
//       LaneSequence* lane_seq_ptr = feature->mutable_lane()
//                                        ->mutable_lane_graph_ordered()
//                                        ->add_lane_sequence();
//       lane_seq_ptr->CopyFrom(lane_seq);
//       lane_seq_ptr->set_lane_sequence_id(seq_id++);
//       lane_seq_ptr->set_lane_s(feature->lane().lane_feature().lane_s());
//       lane_seq_ptr->set_lane_l(feature->lane().lane_feature().lane_l());
//       lane_seq_ptr->set_vehicle_on_lane(vehicle_is_on_lane);
//       lane_seq_ptr->set_using_lane_heading(false);
//       ADEBUG << "Obstacle [" << id_ << "] set a lane sequence ["
//              << lane_seq.ShortDebugString() << "].";
//     }
//   }

//   // Build lane_points.
//   if (feature->lane().has_lane_graph_ordered()) {
//     SetLanePoints(feature, 0.5, 100, true,
//                   feature->mutable_lane()->mutable_lane_graph_ordered());
//   }

//   ADEBUG << "Obstacle [" << id_ << "] set lane graph features.";
// }

// The default SetLanePoints applies to lane_graph with
// FLAGS_target_lane_gap.
void Obstacle::SetLanePoints(Feature* feature) {
  LaneGraph* lane_graph = feature->mutable_lane()->mutable_lane_graph();
  SetLanePoints(feature, FLAGS_target_lane_gap, FLAGS_max_num_lane_point, false,
                lane_graph);
}

// The generic SetLanePoints
void Obstacle::SetLanePoints(const Feature* feature,
                             const double lane_point_spacing,
                             const uint64_t max_num_lane_point,
                             const bool is_bidirection,
                             LaneGraph* const lane_graph) const {
  ADEBUG << "Spacing = " << lane_point_spacing;
  // Sanity checks.
  if (feature == nullptr || !feature->has_velocity_heading()) {
    AERROR << "Null feature or no velocity heading.";
    return;
  }
  double heading = feature->velocity_heading();
  double x = feature->position().x();
  double y = feature->position().y();
  Eigen::Vector2d position(x, y);

  // Go through every lane_sequence.
  for (int i = 0; i < lane_graph->lane_sequence_size(); ++i) {
    LaneSequence* lane_sequence = lane_graph->mutable_lane_sequence(i);
    if (lane_sequence->lane_segment().empty()) {
      continue;
    }
    // TODO(jiacheng): can refactor the following two parts into one to
    //                 make it more elegant.

    // If building bidirectionally, then build backward lane-points as well.
    if (is_bidirection) {
      int lane_index = 0;
      double lane_seg_s = lane_sequence->lane_segment(lane_index).start_s();
      while (lane_index < lane_sequence->lane_segment_size()) {
        // Go through lane_segment one by one sequentially.
        LaneSegment* lane_segment =
            lane_sequence->mutable_lane_segment(lane_index);

        // End-condition: reached the current ADC's location.
        if (lane_index == lane_sequence->adc_lane_segment_idx() &&
            lane_seg_s > lane_segment->adc_s()) {
          lane_segment->set_adc_lane_point_idx(lane_segment->lane_point_size());
          break;
        }

        if (lane_seg_s > lane_segment->end_s()) {
          // If already exceeds the current lane_segment, then go to the
          // next following one.
          // ADEBUG << "Move on to the next lane-segment.";
          lane_seg_s = lane_seg_s - lane_segment->end_s();
          ++lane_index;
        } else {
          // Otherwise, update lane_graph:
          // 1. Sanity checks.
          std::string lane_id = lane_segment->lane_id();

          // ADEBUG << "Currently on " << lane_id;
          auto lane_info = PredictionMap::LaneById(lane_id);
          if (lane_info == nullptr) {
            lane_segment->set_lane_turn_type(1);
            break;
          }
          lane_segment->set_lane_turn_type(
              static_cast<uint32_t>(lane_info->lane().turn()));
          // 2. Get the closeset lane_point
          // ADEBUG << "Lane-segment s = " << lane_seg_s;
          Eigen::Vector2d lane_point_pos =
              PredictionMap::PositionOnLane(lane_info, lane_seg_s);
          double lane_point_heading =
              PredictionMap::HeadingOnLane(lane_info, lane_seg_s);
          double lane_point_width =
              PredictionMap::LaneTotalWidth(lane_info, lane_seg_s);
          double lane_point_angle_diff =
              common::math::AngleDiff(lane_point_heading, heading);
          double lane_kappa = lane_info->Curvature(lane_seg_s);
          // 3. Update it into the lane_graph
          // ADEBUG << lane_point_pos[0] << "    " << lane_point_pos[1];
          LanePoint lane_point;
          lane_point.mutable_position()->set_x(lane_point_pos[0]);
          lane_point.mutable_position()->set_y(lane_point_pos[1]);
          lane_point.set_heading(lane_point_heading);
          lane_point.set_width(lane_point_width);
          lane_point.set_angle_diff(lane_point_angle_diff);
          lane_point.set_kappa(lane_kappa);
          // Update into lane_segment.
          lane_segment->add_lane_point()->CopyFrom(lane_point);
          lane_seg_s += lane_point_spacing;
        }
      }
    }

    // Build lane-point in the forward direction.
    int lane_index = lane_sequence->adc_lane_segment_idx();
    double total_s = 0.0;
    double lane_seg_s = lane_sequence->lane_segment(lane_index).adc_s();
    if (!is_bidirection) {
      lane_index = 0;
      lane_seg_s = lane_sequence->lane_segment(0).start_s();
    }
    std::size_t count_point = 0;
    while (lane_index < lane_sequence->lane_segment_size() &&
           count_point < max_num_lane_point) {
      // Go through lane_segment one by one sequentially.
      LaneSegment* lane_segment =
          lane_sequence->mutable_lane_segment(lane_index);

      if (lane_seg_s > lane_segment->end_s()) {
        // If already exceeds the current lane_segment, then go to the
        // next following one.
        // ADEBUG << "Move on to the next lane-segment.";
        lane_seg_s = lane_seg_s - lane_segment->end_s();
        ++lane_index;
      } else {
        // Otherwise, update lane_graph:
        // 1. Sanity checks.
        std::string lane_id = lane_segment->lane_id();
        // ADEBUG << "Currently on " << lane_id;
        auto lane_info = PredictionMap::LaneById(lane_id);
        if (lane_info == nullptr) {
          lane_segment->set_lane_turn_type(1);
          break;
        }
        lane_segment->set_lane_turn_type(
            static_cast<uint32_t>(lane_info->lane().turn()));

        // 2. Get the closeset lane_point
        // ADEBUG << "Lane-segment s = " << lane_seg_s;
        Eigen::Vector2d lane_point_pos =
            PredictionMap::PositionOnLane(lane_info, lane_seg_s);
        double lane_point_heading =
            PredictionMap::HeadingOnLane(lane_info, lane_seg_s);
        double lane_point_width =
            PredictionMap::LaneTotalWidth(lane_info, lane_seg_s);
        double lane_point_angle_diff =
            common::math::AngleDiff(lane_point_heading, heading);
        double lane_kappa = lane_info->Curvature(lane_seg_s);
        // 3. Update it into the lane_graph
        // ADEBUG << lane_point_pos[0] << "    " << lane_point_pos[1];
        LanePoint lane_point;
        // Update direct information.
        lane_point.mutable_position()->set_x(lane_point_pos[0]);
        lane_point.mutable_position()->set_y(lane_point_pos[1]);
        lane_point.set_heading(lane_point_heading);
        lane_point.set_width(lane_point_width);
        lane_point.set_angle_diff(lane_point_angle_diff);
        // Update deducted information.
        lane_point.set_relative_s(total_s);
        lane_point.set_relative_l(0.0);
        lane_point.set_kappa(lane_kappa);
        // Update into lane_segment.
        lane_segment->add_lane_point()->CopyFrom(lane_point);
        ++count_point;
        total_s += lane_point_spacing;
        lane_seg_s += lane_point_spacing;
      }
    }
  }
  ADEBUG << "Obstacle [" << id_ << "] has lane segments and points.";
}

void Obstacle::SetNearbyObstacles() {
  // This function runs after all basic features have been set up
  Feature* feature_ptr = mutable_latest_feature();

  LaneGraph* lane_graph = feature_ptr->mutable_lane()->mutable_lane_graph();
  for (int i = 0; i < lane_graph->lane_sequence_size(); ++i) {
    LaneSequence* lane_sequence = lane_graph->mutable_lane_sequence(i);
    if (lane_sequence->lane_segment_size() == 0) {
      AERROR << "Empty lane sequence found.";
      continue;
    }
    double obstacle_s = lane_sequence->lane_s();
    double obstacle_l = lane_sequence->lane_l();
    NearbyObstacle forward_obstacle;
    if (clusters_ptr_->ForwardNearbyObstacle(*lane_sequence, id_, obstacle_s,
                                             obstacle_l, &forward_obstacle)) {
      lane_sequence->add_nearby_obstacle()->CopyFrom(forward_obstacle);
      lane_sequence->mutable_front_nearest_obstacle()->Swap(&forward_obstacle);
      ADEBUG << "has front_nearest obstacle: "
             << lane_sequence->front_nearest_obstacle().ShortDebugString();
    }
    NearbyObstacle backward_obstacle;
    if (clusters_ptr_->BackwardNearbyObstacle(*lane_sequence, id_, obstacle_s,
                                              obstacle_l, &backward_obstacle)) {
      lane_sequence->add_nearby_obstacle()->CopyFrom(backward_obstacle);
    }
  }
}

void Obstacle::SetMotionStatus() {
  int history_size = static_cast<int>(feature_history_.size());
  if (history_size == 0) {
    AERROR << "Zero history found";
    return;
  }
  // double pos_std = FLAGS_still_obstacle_position_std;
  double speed_threshold = FLAGS_still_obstacle_speed_threshold;
  double speed_threshold_upper = FLAGS_still_obstacle_speed_threshold_upper;
  if (IsPedestrian()) {
    return;
    // speed_threshold = FLAGS_still_pedestrian_speed_threshold;
    // pos_std = FLAGS_still_pedestrian_position_std;
  }
  if (IsBicycle()) {
    speed_threshold = FLAGS_still_bicycle_speed_threshold;
  } else if (type_ == PerceptionObstacle::UNKNOWN ||
             type_ == PerceptionObstacle::UNKNOWN_MOVABLE ||
             type_ == PerceptionObstacle::UNKNOWN_UNMOVABLE) {
    speed_threshold = FLAGS_still_unknown_speed_threshold;
    // pos_std = FLAGS_still_unknown_position_std;
  }

  double speed = feature_history_.rbegin()->speed();

  if (feature_history_.size() == 1) {
    if (speed < speed_threshold) {
      ADEBUG << "Obstacle [" << id_ << "] has a small speed [" << speed
             << "] and is considered stationary in the first frame.";
      feature_history_.rbegin()->set_is_still(true);
    } else {
      feature_history_.rbegin()->set_is_still(false);
    }
  } else {
    const auto& prev_feat = feature_history_.at(history_size - 2);
    if (prev_feat.is_still()) {
      if (prev_feat.speed() > speed_threshold_upper &&
          speed > speed_threshold_upper) {
        feature_history_.rbegin()->set_is_still(false);
      } else {
        feature_history_.rbegin()->set_is_still(true);
      }
    } else {
      if (prev_feat.speed() < speed_threshold && speed < speed_threshold) {
        feature_history_.rbegin()->set_is_still(true);
      } else {
        feature_history_.rbegin()->set_is_still(false);
      }
    }
  }

  // if (!IsPedestrian()) {
  //   double start_x = 0.0;
  //   double start_y = 0.0;
  //   double avg_drift_x = 0.0;
  //   double avg_drift_y = 0.0;
  //   int len = std::min(history_size, FLAGS_max_still_obstacle_history_length);
  //   len = std::max(len, FLAGS_min_still_obstacle_history_length);
  //   CHECK_GT(len, 1);

  //   auto feature_riter = feature_history_.rbegin();
  //   start_x = feature_riter->position().x();
  //   start_y = feature_riter->position().y();
  //   ++feature_riter;
  //   while (feature_riter != feature_history_.rend()) {
  //     avg_drift_x += (feature_riter->position().x() - start_x) / (len - 1);
  //     avg_drift_y += (feature_riter->position().y() - start_y) / (len - 1);
  //     ++feature_riter;
  //   }

  //   double delta_ts = feature_history_.rbegin()->timestamp() -
  //                     feature_history_.begin()->timestamp();
  //   double speed_sensibility = std::sqrt(2 * history_size) * 4 * pos_std /
  //                              ((history_size + 1) * delta_ts);
  //   if (speed < speed_threshold) {
  //     AERROR << "Obstacle [" << id_ << "] has a small speed [" << speed
  //            << "] and is considered stationary.";
  //     feature_history_.rbegin()->set_is_still(true);
  //   } else if (speed_sensibility < speed_threshold) {
  //     AERROR << "Obstacle [" << id_ << "]"
  //            << "] considered moving [sensibility = " << speed_sensibility
  //            << "]";
  //     feature_history_.rbegin()->set_is_still(false);
  //   } else {
  //     double distance = std::hypot(avg_drift_x, avg_drift_y);
  //     double distance_std = std::sqrt(2.0 / len) * pos_std;
  //     if (distance > 2.0 * distance_std) {
  //       AERROR << "Obstacle [" << id_ << "] is moving.distance:" << distance
  //              << " is > 2.0 * distance_std:" << distance_std;
  //       feature_history_.rbegin()->set_is_still(false);
  //     } else {
  //       AERROR << "Obstacle [" << id_ << "] is stationary. distance" << distance
  //              << " is <= 2.0 * distance_std:" << distance_std;
  //       feature_history_.rbegin()->set_is_still(true);
  //     }
  //   }
  // }
}

void Obstacle::SetMotionStatusBySpeed() {
  feature_history_.rbegin()->set_is_still(false);
  return;

  auto history_size = feature_history_.size();
  if (history_size < 2) {
    ADEBUG << "Obstacle [" << id_ << "] has no history and "
           << "is considered moving.";
    if (history_size > 0) {
      feature_history_.rbegin()->set_is_still(false);
    }
    return;
  }
}

void Obstacle::InsertFeatureToHistory(Feature* feature) {
  feature_history_.Add()->Swap(feature);
  tracking_frame_++;
  ADEBUG << "Obstacle [" << id_ << "] inserted a frame into the history.";
}

std::unique_ptr<Obstacle> Obstacle::Create(ObstacleClusters* clusters_ptr) {
  std::unique_ptr<Obstacle> ptr_obstacle(new Obstacle());
  ptr_obstacle->SetClusters(clusters_ptr);

  return ptr_obstacle;
}

std::unique_ptr<Obstacle> Obstacle::Create(Feature* feature,
                                           ObstacleClusters* clusters_ptr) {
  std::unique_ptr<Obstacle> ptr_obstacle(new Obstacle());
  ptr_obstacle->SetClusters(clusters_ptr);
  ptr_obstacle->InsertFeatureToHistory(feature);
  return ptr_obstacle;
}

bool Obstacle::ReceivedOlderMessage(const double timestamp) const {
  if (feature_history_.empty()) {
    return false;
  }
  auto last_timestamp_received = feature_history_.rbegin()->timestamp();
  return timestamp <= last_timestamp_received;
}

void Obstacle::DiscardOutdatedHistory() {
  auto num_of_frames = feature_history_.size();
  const double latest_ts = feature_history_.rbegin()->timestamp();
  while (latest_ts - feature_history_.begin()->timestamp() >=
         FLAGS_max_history_time) {
    feature_history_.erase(feature_history_.begin());
  }
  auto num_of_discarded_frames = num_of_frames - feature_history_.size();
  if (num_of_discarded_frames > 0) {
    ADEBUG << "Obstacle [" << id_ << "] discards " << num_of_discarded_frames
           << " historical features";
  }
}

void Obstacle::SetCaution() {
  Feature* feature = mutable_latest_feature();
  feature->mutable_priority()->set_priority(ObstaclePriority::CAUTION);
}

bool Obstacle::IsCaution() const {
  if (feature_history_.empty()) {
    return false;
  }
  const Feature& feature = latest_feature();
  return feature.priority().priority() == ObstaclePriority::CAUTION;
}

bool Obstacle::IsOppositeHighway() const {
  // Opposite direction vehicles
  // 1.out of map , or on opposite direction road
  // 2.opposite speed direction

  auto is_carriagerway = [&](const hdmap::RoadSection::Type& type) -> bool {
    return (type == hdmap::RoadSection::MultipleCarriageWay ||
            type == hdmap::RoadSection::SingleCarriageWay);
  };

  static constexpr double kMaxAllowHeadingDiff = 0.25 * M_PI;
  static constexpr double kMaxAllowDistanceBuffer = 0.5;

  double obstacle_x = latest_feature().position().x();
  double obstacle_y = latest_feature().position().y();
  double velocity_heading = latest_feature().velocity_heading();
  bool is_opposite_direction = false;
  bool is_only_detected_from_radar = false;
  if (latest_feature().has_sensor_feature() &&
      latest_feature().sensor_feature().has_curr_detect_sensor()) {
    const auto& sensor = latest_feature().sensor_feature().curr_detect_sensor();
    is_only_detected_from_radar = sensor.from_radar_front_left() &&
                                  !sensor.from_radar_front() &&
                                  !sensor.from_lidar_front_left() &&
                                  !sensor.from_camera_front_long_range() &&
                                  !sensor.from_camera_front_wide_angle() &&
                                  !sensor.from_camera_left_forward_looking() &&
                                  !sensor.from_camera_left_backward_looking();
  }

  std::shared_ptr<const hdmap::LaneInfo> nearest_lane;
  double nearest_s = 0.0;
  double nearest_l = 0.0;
  PredictionMap::GetNearestLane(obstacle_x, obstacle_y, &nearest_lane,
                                &nearest_s, &nearest_l);

  if (nearest_lane != nullptr && nearest_s >= 0.0 &&
      nearest_s <= nearest_lane->total_length() &&
      is_carriagerway(nearest_lane->GetSectionType())) {
    // too far
    double left_lane_width = 0.0;
    double right_lane_width = 0.0;

    // Leftmost
    if (nearest_lane->lane().left_neighbor_forward_lane_id().empty()) {
      nearest_lane->GetWidth(nearest_s, &left_lane_width, &right_lane_width);
      double lane_heading = nearest_lane->Heading(nearest_s);
      double heading_diff =
          common::math::NormalizeAngle(lane_heading - velocity_heading);
      bool heading_opposite_direction =
          std::fabs(heading_diff) > kMaxAllowHeadingDiff;
      if (nearest_l > left_lane_width + kMaxAllowDistanceBuffer &&
          (heading_opposite_direction || is_only_detected_from_radar)) {
        ADEBUG << "Obstacle [" << latest_feature().id()
               << "],is on opposite direction road.left_lane_width: "
               << left_lane_width << ", lane_heading : " << lane_heading
               << ", velocity_heading :" << velocity_heading
               << ",heading_diff : " << heading_diff;
        is_opposite_direction = true;
      }
    }
  }
  return is_opposite_direction;
}

void Obstacle::AddPredictorType(
    const ObstacleConf::PredictorType& predictor_type) {
  obstacle_conf_.add_predictor_type(predictor_type);
}

ObstacleConf::ObstacleStatus Obstacle::GetObstacleStatus() {
  ObstacleConf::ObstacleStatus status = ObstacleConf::OFF_LANE;

  if (IsPedestrian()) {
    status = ObstacleConf::MOVING;
  } else if (IsStill()) {
    status = ObstacleConf::STATIONARY;
  } else if (latest_feature().has_junction_feature()) {
    status = ObstacleConf::IN_JUNCTION;
  } else if (IsOnLane()) {
    status = ObstacleConf::ON_LANE;
  }
  return status;
}

void Obstacle::SetClusters(ObstacleClusters* clusters_ptr) {
  clusters_ptr_ = clusters_ptr;
}

void Obstacle::SetUseKalmanFilter(bool use_kalman_filter) {
  if (use_kalman_filter && !use_kalman_filter_) {
    kalman_motion_fusion_.DeInitFilter();
  }
  use_kalman_filter_ = use_kalman_filter;
}

bool Obstacle::GetCurrentJunctionId(std::string* junction_id) {
  if (junction_id == nullptr) {
    return false;
  }
  common::math::Vec2d ego_vec(latest_feature().position().x(),
                              latest_feature().position().y());

  static constexpr double ego_junction_search_radius = 20.0;

  std::shared_ptr<const hdmap::JunctionInfo> junction_ptr = nullptr;

  if (!current_junction_id_.empty()) {
    junction_ptr = PredictionMap::JunctionById(current_junction_id_);
    if (junction_ptr != nullptr) {
      const auto& polygon = junction_ptr->polygon();
      if (polygon.DistanceTo(ego_vec) > ego_junction_search_radius) {
        junction_ptr = nullptr;
      }
    }
  }

  if (junction_ptr == nullptr) {
    current_junction_id_.clear();
    Eigen::Vector2d point(latest_feature().position().x(),
                          latest_feature().position().y());
    const auto& junction_ptr_vec = PredictionMap::GetJunctions(point, 50.0);
    if (junction_ptr_vec.empty()) {
      return false;
    }
    double min_dis = std::numeric_limits<double>::max();
    for (const auto& j_ptr : junction_ptr_vec) {
      const auto& polygon = j_ptr->polygon();
      double distance = polygon.DistanceTo(ego_vec);
      if (distance < min_dis) {
        min_dis = distance;
        *junction_id = j_ptr->id().id();
      }
    }
    current_junction_id_ = *junction_id;
  } else {
    *junction_id = junction_ptr->id().id();
  }

  return true;
}

bool Obstacle::IgnoreByOccludedProb(double occluded_ignore_distance) {
  if (OccludedProb() < 0.) {
    return false;
  }
  constexpr static double kOccludedProbThreshold = 0.5;

  // be occluded
  if (OccludedProb() > kOccludedProbThreshold) {
    // ingore obstacle if far away &&
    if (latest_feature().has_position_flu()) {
      double dis_to_ego = std::hypot(latest_feature().position_flu().x(),
                                     latest_feature().position_flu().y());
      if (dis_to_ego > occluded_ignore_distance) {
        ADEBUG << "dis_to_ego : " << dis_to_ego
               << " occ_distance : " << occluded_ignore_distance;
        return true;
      }
    }
    // ignore if Vehicle && not onlane  && large theta deviation
    if (type_ == PerceptionObstacle::VEHICLE ||
        type_ == PerceptionObstacle::UNKNOWN) {
      if (!latest_feature().has_lane() ||
          latest_feature().lane().current_lane_feature().empty()) {
        if (std::fabs(common::math::AngleDiff(
                latest_feature().theta(),
                latest_feature().velocity_heading())) > M_PI_4) {
          return true;
        }
      }
    }
  }

  return false;
}

}  // namespace prediction
}  // namespace TL
