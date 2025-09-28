/*
 * Copyright (c) TL auto Co., Ltd. 2023-2024. All rights reserved.
 */

#pragma once

#include <deque>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "common/filters/digital_filter.h"
#include "common/filters/digital_filter_coefficients.h"
#include "planning/localview/lane_line_builder/obstacle_following_lane_line/missile_mode/vehicle_state_decider.h"
#include "planning/proto/navigation_hdmap_config.pb.h"
#include "proto/perception/perception_obstacle.pb.h"

namespace TL {
namespace planning {
namespace missilelane {

struct MissileObs {
  void ObsInit(const perception::PerceptionObstacle& obs);

  void ObsUpdate(const perception::PerceptionObstacle& obs);
  std::deque<perception::PerceptionObstacle> deq_obs;
  double average_heading;
  int count{0};
  TL::common::DigitalFilter heading_filter;
  bool is_valid;
  bool theta_flu_state{true};
};

class ObstaclesState {
 public:
  ObstaclesState() = delete;
  explicit ObstaclesState(
      const planning::PerceptionMapConfig& config,
      const std::shared_ptr<MissileVehicleState>& vehicle_state);
  ~ObstaclesState() = default;

  Status Init();

  void ClearObsInfo() { unmap_obs_.clear(); }

  bool Update(const std::shared_ptr<const perception::PerceptionObstacles>&
                  perception_obs_ptr,
              const std::vector<int32_t>& target_ids);

  const std::shared_ptr<MissileVehicleState>& vehicle_state() {
    return vehicle_state_;
  }

  const std::vector<int32_t>& target_ids() { return target_ids_; }

  bool has_no_target_obs() const { return has_no_target_obs_; }

  bool has_left_target_obs() const { return !left_target_ids_.empty(); }

  bool has_right_target_obs() const { return !right_target_ids_.empty(); }

  const std::unordered_map<int32_t, MissileObs>& unmap_obs() {
    return unmap_obs_;
  }

  const MissileObs& TargetObs(
      const TL::perception::LaneMarker& vehicle_lanemarker,
      const std::shared_ptr<LocalView>& local_view);
  static std::tuple<double, double, double> TjaGetTangent(double x, double y,
                                                          double heading);

  bool IsObsLanemarkerValid(
      const TL::perception::LaneMarker& vehicle_lanemarker);

  bool HasRightObs() { return !right_target_ids_.empty(); }

  bool HasLeftObs() { return !left_target_ids_.empty(); }

 private:
  void UpdateObsBoundaryPoints();
  const planning::PerceptionMapConfig& config_;
  std::shared_ptr<MissileVehicleState> vehicle_state_{nullptr};
  std::vector<int32_t> target_ids_;
  std::vector<int32_t> left_target_ids_;
  std::vector<int32_t> right_target_ids_;
  std::vector<TL::common::Point3D> left_boundary_points_;
  std::vector<TL::common::Point3D> right_boundary_points_;
  std::unordered_map<int32_t, MissileObs> unmap_obs_;
  bool has_no_target_obs_{true};
  MissileObs no_valid_obs_;
  int target_id_history_{0};
  int selected_target_id{0};
  bool is_cut_in_ = false;
  bool is_cip_in_lane_ = false;
  bool is_cip_cutout_ = false;
};
}  // namespace missilelane
}  // namespace planning
}  // namespace TL
