/*
 * Copyright (c) 2022 TL
 *
 * Author: Ling Peng  / Xing Kang
 */

#pragma once

#include <list>
#include <memory>
#include <string>

#include "map/hdmap/hdmap_util.h"
#include "planning/self_simulator/sim_dummy_obs/obstacle/perception_obs/perception_obstacle_base.h"
#include "proto/prediction/lane_graph.pb.h"

namespace TL {
namespace simdummy {
class LaneFollowObs : public PerceptionObstacleBase {
 public:
  explicit LaneFollowObs(int id, double x, double y, double heading, double v,
                         const std::string& lane_id) {
    SetState(id, 4.0, 2.0, 2.0, x, y, heading, v,
             TL::perception::PerceptionObstacle::VEHICLE);

    is_display_ = true;
    current_lane_id_ = lane_id;
  }

  void ConstructLaneSequence(
      const bool search_forward_direction, const double accumulated_s,
      const double curr_lane_seg_s,
      std::shared_ptr<const hdmap::LaneInfo> lane_info_ptr,
      const int graph_search_horizon, const bool consider_lane_split,
      std::list<prediction::LaneSegment>* const lane_segments,
      prediction::LaneGraph* const lane_graph_ptr) const;

  void BuildLaneGraph();

  bool StartDisplay(const DummyObsInputDataBase& data) override;

  bool StopDisplay(const DummyObsInputDataBase& data) override;

  void UpdateState(const DummyObsInputDataBase& data) override;

 private:
  prediction::LaneGraph lane_graph_;
  std::string current_lane_id_;
  double current_s_ = 0.0;
  double current_l_ = 0.0;
  // The total length to search for lane_graph.
  double length_ = 20.0;
  std::shared_ptr<hdmap::HDMap> map_ptr_ = nullptr;
};
}  // namespace simdummy
}  // namespace TL
