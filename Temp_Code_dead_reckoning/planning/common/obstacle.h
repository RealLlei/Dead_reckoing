#ifndef PLANNING_COMMON_OBSTACLE_H
#define PLANNING_COMMON_OBSTACLE_H
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

#pragma once

#include <limits>
#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "common/math/box2d.h"
#include "common/math/vec2d.h"
#include "common/thread/thread_pool.h"
#include "planning/common/indexed_list.h"
#include "planning/common/speed/st_boundary.h"
#include "planning/common/speed/st_point.h"
#include "planning/reference_line/reference_line.h"

#include "proto/common/pnc_point.pb.h"
#include "proto/common/vehicle_config.pb.h"
#include "proto/perception/perception_obstacle.pb.h"
#include "proto/planning/decision.pb.h"
#include "proto/planning/sl_boundary.pb.h"
#include "proto/prediction/feature.pb.h"
#include "proto/prediction/prediction_obstacle.pb.h"

namespace TL {
namespace planning {
using TL::common::SLPoint;
using TL::common::math::Vec2d;
using TL::common::thread::ThreadPool;

enum GameObstacleIntention { Undefine = 0, AlongsideObs = 1, Cutin = 2 };

struct ObsPointDescription {
  ObsPointDescription() = default;
  double time = -1.0;
  SLPoint center_p;
  SLPoint low_left_p;
  SLPoint low_right_p;
  SLPoint upper_left_p;
  SLPoint upper_right_p;
};

/**
 * @brief lateral intention used for speed optimizer
 * 
 */
enum class LateralIntention {
  UNSET = 0,
  UNKNOWN = 1,
  MERGE = 2,
  ALONGSIDE = 3,
  CUTIN = 4,
};

/**
 * @brief longitudinal intention used for speed optimizer
 * 
 */
enum class LongitudinalIntention {
  UNSET = 0,
  UNKNOWN = 1,
  OVERTAKE = 2,
  YIELD = 3,
};

/**
 * @class Obstacle
 * @brief This is the class that associates an Obstacle with its path
 * properties. An obstacle's path properties relative to a path.
 * The `s` and `l` values are examples of path properties.
 * The decision of an obstacle is also associated with a path.
 *
 * The decisions have two categories: lateral decision and longitudinal
 * decision.
 * Lateral decision includes: nudge, ignore.
 * Lateral decision safety priority: nudge > ignore.
 * Longitudinal decision includes: stop, yield, follow, overtake, ignore.
 * Decision safety priorities order: stop > yield >= follow > overtake > ignore
 *
 * Ignore decision belongs to both lateral decision and longitudinal decision,
 * and it has the lowest priority.
 */
class Obstacle {
 public:
  Obstacle() = default;
  Obstacle(const std::string& id,
           const perception::PerceptionObstacle& perception_obstacle,
           const prediction::ObstaclePriority::Priority& obstacle_priority,
           const prediction::ObstacleIntent::Type& obstacle_intent,
           bool is_static, prediction::TrackStatus tracking);
  Obstacle(const std::string& id,
           const perception::PerceptionObstacle& perception_obstacle,
           const prediction::Trajectory& trajectory,
           const prediction::ObstaclePriority::Priority& obstacle_priority,
           const prediction::ObstacleIntent::Type& obstacle_intent,
           bool is_static, const prediction::TrackStatus& tracking);

  const std::string& Id() const { return id_; }

  void SetId(const std::string& id) { id_ = id; }

  double speed() const { return speed_; }

  const prediction::TrackStatus& TrackStatus() const { return track_status_; }

  int32_t PerceptionId() const { return perception_id_; }

  bool IsStatic() const { return is_static_; }

  bool IsStaticWithoutIgnore() const { return is_static_without_ignore_; }

  bool IsVirtual() const { return is_virtual_; }

  bool IsBicycle() const {
    return perception_obstacle_.type() ==
               perception::PerceptionObstacle::BICYCLE ||
           perception_obstacle_.type() ==
               perception::PerceptionObstacle::CYCLIST;
  }

  bool IsOversizedVehicle() const {
    return perception_obstacle_.type() ==
               perception::PerceptionObstacle::VEHICLE &&
           (perception_obstacle_.sub_type() ==
                perception::PerceptionObstacle::ST_TRUCK ||
            perception_obstacle_.sub_type() ==
                perception::PerceptionObstacle::ST_BIG_TRUCK ||
            perception_obstacle_.sub_type() ==
                perception::PerceptionObstacle::ST_BUS ||
            perception_obstacle_.sub_type() ==
                perception::PerceptionObstacle::ST_MIDDLE_BUS ||
            perception_obstacle_.sub_type() ==
                perception::PerceptionObstacle::ST_AMBULANCE);
  }

  /**
   * @brief Check if obstacle is cone
   * 
   * @return true 
   * @return false 
   */
  bool IsCone() const {
    return perception_obstacle_.sub_type() ==
               TL::perception::PerceptionObstacle::ST_TRAFFICCONE ||
           perception_obstacle_.sub_type() ==
               TL::perception::PerceptionObstacle::ST_WATERHORSE ||
           perception_obstacle_.sub_type() ==
               TL::perception::PerceptionObstacle::ST_CRASHBARRELS;
  }

  /**
   * @brief jude obstacle is low height, used to filter obs in  path or speed
   * optimizer
   *
   * @return true
   * @return false
   */
  bool IsLowHeight() const { return is_low_height_; }

  /**
   * @brief jude obstacle is alive and vulnerable, used to filter obs in  path
   *
   * @return true
   * @return false
   */
  bool IsVulnerableAlive() const { return is_vulnerable_alive_; }

  common::TrajectoryPoint GetPointAtTime(double time) const;

  common::math::Box2d GetBoundingBox(
      const common::TrajectoryPoint& point) const;

  const common::math::Box2d& PerceptionBoundingBox() const {
    return perception_bounding_box_;
  }

  const common::math::Polygon2d& PerceptionPolygon() const {
    return perception_polygon_;
  }

  const prediction::Trajectory& Trajectory() const { return trajectory_; }

  prediction::Trajectory* GetMutableTrajectory() { return &trajectory_; }

  common::TrajectoryPoint* AddTrajectoryPoint() {
    return trajectory_.add_trajectory_point();
  }

  bool HasTrajectory() const {
    return !(trajectory_.trajectory_point().empty());
  }

  const perception::PerceptionObstacle& Perception() const {
    return perception_obstacle_;
  }

  /**
   * @brief This is a helper function that can create obstacles from prediction
   * data.  The original prediction may have multiple trajectories for each
   * obstacle. But this function will create one obstacle for each trajectory.
   * @param predictions The prediction results
   * @return obstacles The output obstacles saved in a list of unique_ptr.
   */
  static std::list<std::shared_ptr<Obstacle>> CreateObstacles(
      const prediction::PredictionObstacles& predictions);

  static std::shared_ptr<Obstacle> CreateStaticVirtualObstacles(
      const std::string& id, const common::math::Box2d& obstacle_box);

  static bool IsValidPerceptionObstacle(
      const perception::PerceptionObstacle& obstacle);

  static bool IsValidTrajectoryPoint(const common::TrajectoryPoint& point);

  inline bool IsCautionLevelObstacle() const {
    return is_caution_level_obstacle_;
  }

  inline prediction::ObstaclePriority::Priority Priority() const {
    return priority_;
  }

  inline prediction::ObstacleIntent::Type Intent() const { return intent_; }

  // const Obstacle* obstacle() const;

  /**
   * return the merged lateral decision
   * Lateral decision is one of {Nudge, Ignore}
   **/
  const ObjectDecisionType& LateralDecision() const;

  /**
   * @brief return the merged longitudinal decision
   * Longitudinal decision is one of {Stop, Yield, Follow, Overtake, Ignore}
   **/
  const ObjectDecisionType& LongitudinalDecision() const {
    return longitudinal_decision_;
  }

  std::string DebugString() const;

  const SLBoundary& PerceptionSLBoundary() const { return sl_boundary_; }

  const STBoundary& reference_line_st_boundary() const;

  const STBoundary& path_st_boundary() const { return path_st_boundary_; }

  const SLTBoundary& GetPathSLTBoundary() const { return path_slt_boundary_; }

  const std::vector<std::string>& decider_tags() const;

  const std::vector<ObjectDecisionType>& decisions() const;

  void AddLongitudinalDecision(const std::string& decider_tag,
                               const ObjectDecisionType& decision);

  void SetLongitudinalDecision(const std::string& decider_tag,
                               const ObjectDecisionType& decision);

  void AddLateralDecision(const std::string& decider_tag,
                          const ObjectDecisionType& decision);

  void ClearDecision();

  bool HasLateralDecision() const;

  void set_path_st_boundary(const STBoundary& boundary);

  void set_path_st_boundary(STBoundary&& boundary);

  void SetPathSLTBoundary(SLTBoundary&& boundary) {
    path_slt_boundary_ = std::move(boundary);
  }

  bool is_path_st_boundary_initialized() const {
    return path_st_boundary_initialized_;
  }

  void SetStBoundaryType(const STBoundary::BoundaryType& type);

  void EraseStBoundary();

  void SetReferenceLineStBoundary(const STBoundary& boundary);

  void SetReferenceLineStBoundaryType(const STBoundary::BoundaryType& type);

  void EraseReferenceLineStBoundary();

  bool HasLongitudinalDecision() const;

  bool HasNonIgnoreDecision() const;

  /**
   * @brief Calculate stop distance with the obstacle using the ADC's minimum
   * turning radius
   */
  double MinRadiusStopDistance(const common::VehicleParam& vehicle_param) const;

  /**
   * @brief Calculate normal stop distance，do not consider lane borrow
   * turning radius
   */
  [[nodiscard]] double NormalStopDistance() const;

  /**
   * @brief Check if this object can be safely ignored.
   * The object will be ignored if the lateral decision is ignore and the
   * longitudinal decision is ignore
   *  return longitudinal_decision_ == ignore && lateral_decision == ignore.
   */
  bool IsIgnore() const;
  bool IsLongitudinalIgnore() const;
  bool IsLateralIgnore() const;

  void BuildReferenceLineStBoundary(const ReferenceLine& reference_line,
                                    double adc_start_s);

  void SetPerceptionSlBoundary(const SLBoundary& sl_boundary);

  /**
   * @brief check if an ObjectDecisionType is a longitudinal decision.
   **/
  static bool IsLongitudinalDecision(const ObjectDecisionType& decision);

  /**
   * @brief check if an ObjectDecisionType is a lateral decision.
   **/
  static bool IsLateralDecision(const ObjectDecisionType& decision);

  void SetBlockingObstacle(bool blocking) { is_blocking_obstacle_ = blocking; }

  bool IsBlockingObstacle() const { return is_blocking_obstacle_; }

  /*
   * @brief IsLaneBlocking is only meaningful when IsStatic() == true.
   */
  bool IsLaneBlocking() const { return is_lane_blocking_; }

  void CheckLaneBlocking(const std::shared_ptr<ReferenceLine>& reference_line);

  bool IsLaneChangeBlocking() const { return is_lane_change_blocking_; }

  void SetLaneChangeBlocking(bool is_distance_clear);

  bool IsShortDistanceNudge() const { return is_short_distance_nudge_; }

  void SetIsShortDistanceNudge(bool is_short_distance_nudge);

  bool IsShortDistanceThresholdDone() const {
    return is_short_distance_threshold_done_;
  }

  void SetIsShortDistanceThresholdDone(bool is_short_distance_threshold_done);

  double GetShortDistanceThreshold() const { return short_distance_threshold_; }

  void SetShortDistanceThreshold(double short_distance_threshold);

  double GetMaxExpectTowingL() const { return max_expect_towing_l_; }

  void SetMaxExpectTowingL(double max_expect_towing_l);

  void SetForbidStaticObsNudgeDisplay(bool forbid_static_obs_nudge_display);

  bool GetForbidStaticObsNudgeDisplay() const {
    return forbid_static_obs_nudge_display_;
  }

  /**
   * @brief Set trajectory envelope
   *
   * @param trajectoryEnvelope obstacle trajectory envelope
   */
  void SetTrajectoryEnvelope(
      std::vector<ObsPointDescription>&& trajectoryEnvelope) {
    // trajectory_envelope_ = trajectoryEnvelope;
    trajectory_envelope_ = std::move(trajectoryEnvelope);
    // determine if this obstacle trajectory crosses reference line
    // TODO(chenwei) in the future, if prediction give obstacle intention, we
    // should remote these code
    auto min_l = std::numeric_limits<double>::max();
    auto max_l = std::numeric_limits<double>::lowest();
    for (const auto& envelope : trajectory_envelope_) {
      min_l = fmin(min_l, envelope.center_p.l());
      max_l = fmax(max_l, envelope.center_p.l());
    }
    is_cross_reference_line_ = min_l < -1.0 && max_l > 1.0;
  }

  const std::vector<ObsPointDescription>& GetTrajectoryEnvelope() const {
    return trajectory_envelope_;
  }

  void InitTrajectoryBoundingBox();

  const std::vector<common::math::Box2d>& GetTrajectoryBoundingBox() const {
    return trajectory_bounding_box_;
  }

  void SetTrajectoryBoundingBox(
      std::vector<common::math::Box2d>&& trajectory_bounding_box) {
    trajectory_bounding_box_ = std::move(trajectory_bounding_box);
  }

  double GetTrajMinS() const { return traj_min_s_; }

  void SetTrajMinS(double traj_min_s) { traj_min_s_ = traj_min_s; }

  double GetTrajMaxS() const { return traj_max_s_; }

  void SetTrajMaxS(double traj_max_s) { traj_max_s_ = traj_max_s; }

  double GetTrajMinL() const { return traj_min_l_; }

  void SetTrajMinL(double traj_min_l) { traj_min_l_ = traj_min_l; }

  double GetTrajMaxL() const { return traj_max_l_; }

  void SetTrajMaxL(double traj_max_l) { traj_max_l_ = traj_max_l; }

  bool GetNudgeLimitOstacleInLeft() const {
    return nudge_limit_obstacle_inleft_;
  }

  void SetNudgeLimitOstacleInLeft(bool nudge_limit_obstacle_inleft) {
    nudge_limit_obstacle_inleft_ = nudge_limit_obstacle_inleft;
  }

  /**
   * @brief check whether obstacle trajectory cross reference line
   *
   * @return true obstacle trajectory cross reference line
   * @return false obstacle trajectory dose not cross reference line
   */
  bool IsCrossReferenceLine() const { return is_cross_reference_line_; }

  /**
   * @brief Set the Is Cross Reference Line object
   * 
   * @param is_cross_reference_line 
   */
  void SetIsCrossReferenceLine(bool is_cross_reference_line) {
    is_cross_reference_line_ = is_cross_reference_line;
  }

  /**
   * @brief check whether obstacle is cross obstacle
   *
   * @return true obstacle is cross obstacle
   * @return false obstacle is not cross obstacle
   */
  bool GetIsCrossObstacle() const { return is_cross_obstacle_; }

  /**
   * @brief Set obstacle is cross obstacle
   * 
   * @param is_cross_reference_line 
   */
  void SetIsCrossObstacle(bool is_cross_obstacle) {
    is_cross_obstacle_ = is_cross_obstacle;
  }

  void SetIsBelievable(bool is_believable) { is_believable_ = is_believable; }

  bool IsBelievable() const { return is_believable_; }

  /**
   * @brief is obs is uss obj type
   *
   * @return true
   * @return false
   */
  bool IsUssObs() const {
    return perception_obstacle_.sub_type() ==
           TL::perception::PerceptionObstacle::ST_USS;
  }

  [[nodiscard]] const LateralIntention& GetLateralIntention() const {
    return lateral_intention_;
  }

  void SetLateralIntention(const LateralIntention& lateral_intention) {
    lateral_intention_ = lateral_intention;
  }

  [[nodiscard]] const LongitudinalIntention& GetLongitudinalIntention() const {
    return longitudinal_intention_;
  }

  void SetLongitudinalIntention(
      const LongitudinalIntention& longitudinal_intention) {
    longitudinal_intention_ = longitudinal_intention;
  }

  bool GetHasIntention() const {
    return lateral_intention_ != LateralIntention::UNSET ||
           longitudinal_intention_ != LongitudinalIntention::UNSET;
  }

  /**
   * @brief Get the Is Occluded obstacle
   * 
   * @return  is_occluded_obstacle_
   */
  bool GetIsOccludedObstacle() const { return is_occluded_obstacle_; }

  /**
   * @brief Get the Is NudgeLimit obstacle
   * 
   * @return is_nudgelimit_obstacle_
   */
  bool GetIsNudgeLimitObstacle() const { return is_nudgelimit_obstacle_; }

  /**
   * @brief Set the Is Occluded obstacle
   * 
   * @param is_occluded_obstacle 
   */
  void SetIsOccludedObstacle(bool is_occluded_obstacle) {
    is_occluded_obstacle_ = is_occluded_obstacle;
  }

  /**
   * @brief Set the Is NudgeLimit Obstacle
   * 
   * @param is_nudgelimit_obstacle 
   */

  void SetIsNudgeLimitObstacle(bool is_nudgelimit_obstacle) {
    is_nudgelimit_obstacle_ = is_nudgelimit_obstacle;
  }

  void SetIsMergeObstacle(bool is_merge_obstacle) {
    is_merge_obstacle_ = is_merge_obstacle;
  }

  bool GetIsMergeObstacle() const { return is_merge_obstacle_; }

 private:
  // FRIEND_TEST(MergeLongitudinalDecision, AllDecisions);
  static ObjectDecisionType MergeLongitudinalDecision(
      const ObjectDecisionType& lhs, const ObjectDecisionType& rhs);
  // FRIEND_TEST(MergeLateralDecision, AllDecisions);
  static ObjectDecisionType MergeLateralDecision(const ObjectDecisionType& lhs,
                                                 const ObjectDecisionType& rhs);

  bool BuildTrajectoryStBoundary(const ReferenceLine& reference_line,
                                 double adc_start_s, STBoundary* st_boundary);
  static bool IsValidObstacle(
      const perception::PerceptionObstacle& perception_obstacle);

  static void GetLineCrossPointS(const SLPoint& l_p, const SLPoint& r_p,
                                 double adc_width_threshold,
                                 std::vector<double>* cross_s_ptr);

  static bool FindTwoBoxCrossPoint(const ObsPointDescription& point_1,
                                   const ObsPointDescription& point_2,
                                   bool is_find_min_s,
                                   double adc_width_threshold,
                                   std::vector<double>* cross_s_ptr,
                                   ObsPointDescription* point_analys_ptr);

  static ObsPointDescription GetPointByRatio(const ObsPointDescription& p_1,
                                             const ObsPointDescription& p_2,
                                             double ra);
  /**
   * @brief jugde obs is low height or not
   *
   * @param perception_obstacle  perception obstacle
   * @return true
   * @return false
   */
  static bool IsObstacleWithLowHeight(
      const perception::PerceptionObstacle& perception_obstacle);
  /**
   * @brief juge obstacle is vulnerable alive or not
   *
   * @param perception_obstacle
   * @return true
   * @return false
   */
  static bool IsObstacleVulnerableAlive(
      const perception::PerceptionObstacle& perception_obstacle);

 private:
  std::string id_;
  int32_t perception_id_ = 0;
  bool is_static_ = false;                 // static && ignore
  bool is_static_without_ignore_ = false;  // static
  bool is_virtual_ = false;
  bool is_low_height_ = false;
  bool is_vulnerable_alive_ = false;
  bool is_short_distance_nudge_ = false;
  bool is_occluded_obstacle_ = false;
  bool is_nudgelimit_obstacle_ = false;
  bool is_short_distance_threshold_done_ = false;
  double short_distance_threshold_ = 0.2;
  double max_expect_towing_l_ = 0.0;
  bool forbid_static_obs_nudge_display_ = false;

  double speed_ = 0.0;

  bool path_st_boundary_initialized_ = false;

  prediction::Trajectory trajectory_;
  std::vector<ObsPointDescription> trajectory_envelope_;
  double traj_min_s_ = std::numeric_limits<double>::max();
  double traj_max_s_ = std::numeric_limits<double>::lowest();
  double traj_min_l_ = std::numeric_limits<double>::max();
  double traj_max_l_ = std::numeric_limits<double>::lowest();
  std::vector<common::math::Box2d> trajectory_bounding_box_;

  perception::PerceptionObstacle perception_obstacle_;
  common::math::Box2d perception_bounding_box_;
  common::math::Polygon2d perception_polygon_;

  std::vector<ObjectDecisionType> decisions_;
  std::vector<std::string> decider_tags_;
  SLBoundary sl_boundary_;

  STBoundary reference_line_st_boundary_;
  STBoundary path_st_boundary_;
  SLTBoundary path_slt_boundary_;

  ObjectDecisionType lateral_decision_;
  ObjectDecisionType longitudinal_decision_;

  // for keep_clear usage only
  bool is_blocking_obstacle_ = false;

  bool is_lane_blocking_ = false;

  bool is_lane_change_blocking_ = false;

  bool is_caution_level_obstacle_ = false;

  double min_radius_stop_distance_ = -1.0;

  bool is_cross_reference_line_ = false;

  bool is_cross_obstacle_ = false;

  bool is_believable_ = true;

  bool nudge_limit_obstacle_inleft_ = false;
  prediction::ObstaclePriority::Priority priority_ =
      prediction::ObstaclePriority::NORMAL;

  prediction::ObstacleIntent::Type intent_ =
      prediction::ObstacleIntent::UNKNOWN;

  prediction::TrackStatus track_status_;

  struct ObjectTagCaseHash {
    size_t operator()(
        const planning::ObjectDecisionType::ObjectTagCase tag) const {
      return static_cast<size_t>(tag);
    }
  };

  static const std::unordered_map<ObjectDecisionType::ObjectTagCase, int,
                                  ObjectTagCaseHash>
      s_lateral_decision_safety_sorter_;
  static const std::unordered_map<ObjectDecisionType::ObjectTagCase, int,
                                  ObjectTagCaseHash>
      s_longitudinal_decision_safety_sorter_;

  LateralIntention lateral_intention_ = LateralIntention::UNSET;
  LongitudinalIntention longitudinal_intention_ = LongitudinalIntention::UNSET;

  bool is_merge_obstacle_ = false;
};

using IndexedObstacles = IndexedList<std::string, Obstacle>;
using ThreadSafeIndexedObstacles =
    ThreadSafeIndexedList<std::string, std::shared_ptr<Obstacle>>;

}  // namespace planning
}  // namespace TL

#endif  // PLANNING_COMMON_OBSTACLE_H
