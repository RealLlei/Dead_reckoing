/*
 * Copyright (c) TL auto Co., Ltd. 2023-2024. All rights reserved.
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "planning/localview/lane_line_builder/lane_line_base.h"
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/decider_data.h"
#include "planning/localview/lane_line_builder/obstacle_following_lane_line/missile_mode/obstacle_decider.h"
#include "planning/localview/lane_line_builder/obstacle_following_lane_line/missile_mode/obstacles_state.h"
#include "planning/localview/lane_line_builder/obstacle_following_lane_line/missile_mode/points_filter.h"
#include "planning/localview/lane_line_builder/obstacle_following_lane_line/missile_mode/vehicle_state_decider.h"
#include "planning/localview/local_view.h"
#include "planning/proto/navigation_hdmap_config.pb.h"
#include "proto/common/vehicle_state.pb.h"

namespace TL {
namespace planning {
namespace missilelane {
class MissileMode : public LaneLineBase {
 public:
  MissileMode();

  /**
   * @brief module name
   */
  static std::string Name() { return "MissileMode"; }

  /**
   * @brief module initialization function
   * @return initialization status
   */
  TL::common::Status Init(
      const planning::PerceptionMapConfig& config,
      const std::shared_ptr<LocalViewData>& local_view_data);
  TL::common::Status Init() override;
  TL::common::Status Init(
      const std::shared_ptr<LocalViewData>& local_view_data);
  /**
   * @brief module start function
   * @return start status
   */
  TL::common::Status Start() override;

  /**
   * @brief module stop function
   */
  void Stop() override;

  /**
   * @brief main logic of the navigation_hdmap module, runs periodically
   * triggered by timer.
   */
  bool Process(const std::shared_ptr<LocalView>& local_view,
               functionmanager::FunctionManagerOut* to_fct) override;

  const std::shared_ptr<navigation_hdmap::MapMsg>& GetMapMsg(
      bool refresh) override {
    UNUSED(refresh);
    return current_map_msg_;
  };

  const std::shared_ptr<routing::RoutingResponse>& GetRoutingResponse()
      override {
    return current_routing_response_;
  };

  bool NoLaneStatus() { return nolane_rise_debounce_.LastStatus(); }

 private:
  bool GeneratePath(std::vector<Vec2d>* points, common::Path* path);
  bool GenerateMap(const common::Path& path, hdmap::Lane* lane,
                   TL::hdmap::Map* hdmap);
  bool GenerateOneLane(const common::Path& path, hdmap::Lane* lane,
                       double extension_one_lane_width);
  bool Addpoints(std::vector<Vec2d>* points);
  bool ConvertVec2Path(const std::vector<Vec2d>& points, common::Path* path);
  static bool SetRouting(TL::routing::RoutingResponse* inrouting,
                         TL::hdmap::Map* hd_map);
  bool GenerateLaneMarker(TL::perception::LaneMarker* lanemarker);
  static bool CheckerLanemarkers(const std::shared_ptr<LocalView>& local_view);
  static bool CheckerLanemarker(
      const TL::perception::LaneMarker& lanemarker);
  bool DebounceStatus(functionmanager::FunctionManagerOut* to_fct, bool status);
  planning::PerceptionMapConfig config_;
  std::shared_ptr<MissileVehicleState> missile_vehicle_state_{nullptr};
  std::shared_ptr<ObstacleDecider> obs_decider_{nullptr};
  std::shared_ptr<navigation_hdmap::MapMsg> current_map_msg_{nullptr};
  std::shared_ptr<routing::RoutingResponse> current_routing_response_{nullptr};
  std::shared_ptr<PointsFilter> points_filter_{nullptr};
  std::shared_ptr<ObstaclesState> obstacles_state_{nullptr};
  TL::planning::lanelineprocess::DebounceModule nolane_rise_debounce_;
};
}  // namespace missilelane
}  // namespace planning
}  // namespace TL
