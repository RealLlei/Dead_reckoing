/*
 * Copyright (c) 2022 TL Technologies Co., Ltd. All rights reserved.
 */

#pragma once
#include <cstdint>
#include <limits>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "planning/common/obstacle.h"
#include "planning/common/reference_line_info.h"
#include "proto/planning/decider_debug.pb.h"

namespace TL {
namespace planning {

struct ObstacleInfo {
  double start_s = 0.0;
  double end_s = 0.0;
  double start_l = 0.0;
  double end_l = 0.0;
  double speed = 0.0;
  ObstacleBehavior::InitialBehavior init_behavior = ObstacleBehavior::Unknown;
  int32_t perception_id;
};

class ObstaclesInfo {
 public:
  ObstaclesInfo() = default;
  // traverse all obstacles, and set initial behavior including follow, cutin, cross, alongside
  void GetObsInitialBehavior(const ReferenceLineInfo& reference_line_info);
  // obstacles in front of adc and is_blocking
  bool SetBlockingObs(const Obstacle* obs, double adc_s, bool is_forward,
                      bool is_blocking, bool is_going_to_follow_adc_lane);
  // cross_obs in front of adc
  bool SetCrossObs(const Obstacle* obs,
                   const ReferenceLineInfo& reference_line_info);
  // cut in obs in front or back of adc
  bool SetCutInObs(const Obstacle* obs, bool is_blocking,
                   bool is_going_to_follow_adc_lane);
  // obstacles in front of CIPV
  void GetObsFrontCIPV(std::vector<std::string>* ids,
                       const IndexedObstacles& obstacles);
  // judge whether obstacle is invalid
  static bool ObstacleHasInvalidBoudary(const Obstacle* obstacle);
  static bool ObstacleHasInvalidTrackingTime(const Obstacle* obs);
  // get heading difference
  static double ObsHeadingDifferenceFromAdc(
      const Obstacle* obstacle, const ReferenceLineInfo* reference_line_info);
  // interactive_obs_ may be front_cutin_obs, if adc is in_dying_road, may be back_cutin_obs
  void SelectInteractiveObs(double adc_end_s, double adc_speed);
  // TODO(All) to be improved, alongside_obs may be cut_in_obs
  void SetAlongsideObs(const Obstacle* obstacle, bool is_blocking,
                       double lane_left_width, double lane_right_width);
  // obs traj s < ref_line, return <s,l>
  static std::pair<double, double> GetTrajFarthestPoint(
      double ref_length,
      const std::vector<ObsPointDescription>& trajectory_envelope);
  // whether obs traj farthest point is in adc lane
  // TODO(All) it will be achieved in SetTrajectoryEnvelope
  static bool IsObsGoingToFollowAdcLane(
      const ReferenceLineInfo& reference_line_info,
      const std::vector<ObsPointDescription>& trajectory_envelope);

  // get nearest_front_onlane_id_
  const std::string& GetNearestBlockingObs() const {
    return nearest_blocking_obs_.first;
  }

  // get cipv_id_
  const std::string& GetCIPV() const { return cipv_.first; }

  // get alongside_obs_perception_ids_
  const std::unordered_set<int32_t>& GetAlongsideObsPerceptionIds() const {
    return alongside_obs_perception_ids_;
  }

  // get alongside_obs_perception_ids_
  const std::unordered_set<int32_t>& GetCutinObsPerceptionIds() const {
    return cutin_obs_perception_ids_;
  }

  // get nearest_front_cutin_id_
  const std::string& GetNearestFrontCutInObs() const {
    return nearest_front_cutin_obs_.first;
  }

  // get obstacles_behavior_info_
  const std::unordered_map<std::string, ObstacleInfo>& GetObstaclesInfo()
      const {
    return obstacles_behavior_info_;
  }

  // the key interactive to CalculateCost
  const std::array<std::pair<std::string, std::string>, 3>& GetInteractiveObs()
      const {
    return interactive_obs_;
  }

  const std::vector<std::pair<std::string, double>>& GetNeighborFrontMergeObs()
      const {
    return neighbor_front_merge_obs_;
  }

  const std::vector<std::pair<std::string, double>>& GetNeighborBackMergeObs()
      const {
    return neighbor_back_merge_obs_;
  }

  // the leading obs of interactive obs and adc
  const std::string& GetAdcLeadingObsId() const { return adc_leading_obs_id_; }

  std::pair<bool, double> GetAdcMergeInfo() const {
    return {adc_in_dying_road_, adc_distance_to_merging_end_s_};
  }

  void ObsInfoClear();

  void ObsInfoDebug();

  void ClearCache() { last_interactive_obs_perception_id_ = -2; }

 private:
  // adc_front_obstacles_ may influence adc, including is_blocking
  // or st_boundary is crossing adc_lane
  std::vector<std::string> adc_front_valid_obstacles_;
  // ignore_obs_before_cipv_
  std::vector<std::string> adc_front_obs_;
  // front cut-in obs including in front of cipv
  std::vector<std::string> front_cut_in_obs_;
  // heading direction is (1/4 pi, 3/4 pi)
  std::vector<std::string> cross_obs_;
  // is_going_to_follow_adc_lane
  std::vector<std::string> front_follow_adc_lane_obs_;
  // merge with adc
  std::vector<std::pair<std::string, double>> neighbor_front_merge_obs_;
  std::vector<std::pair<std::string, double>> neighbor_back_merge_obs_;

  std::unordered_set<int32_t> cutin_obs_perception_ids_;
  std::unordered_set<int32_t> alongside_obs_perception_ids_;

  // CIPV: Closest In Path Vehicle
  std::pair<std::string, double> cipv_ = {
      "", std::numeric_limits<double>::infinity()};
  std::pair<std::string, double> nearest_front_cutin_obs_ = {
      "", std::numeric_limits<double>::infinity()};
  std::pair<std::string, double> nearest_blocking_obs_ = {
      "", std::numeric_limits<double>::infinity()};

  // <interactive_obs_id, inter_leading_obs_id>
  // key interactive obs
  // 1st: nearest_obs_to_merge_point; 2nd: nearest_front_obs; 3rd: nearest_back_obs
  std::array<std::pair<std::string, std::string>, 3> interactive_obs_ = {};
  std::string adc_leading_obs_id_;

  int32_t last_interactive_obs_perception_id_ = -2;

  double adc_distance_to_merging_end_s_ =
      std::numeric_limits<double>::infinity();
  bool adc_in_dying_road_ = false;

  // <id, <start_s, end_s, start_l, end_l, speed, initial_behavior>>
  // initial_behavior: Follow,CutIn,Cross,Alongside_Left,Alongside_Right
  std::unordered_map<std::string, ObstacleInfo> obstacles_behavior_info_ = {};
};
}  // namespace planning
}  // namespace TL
