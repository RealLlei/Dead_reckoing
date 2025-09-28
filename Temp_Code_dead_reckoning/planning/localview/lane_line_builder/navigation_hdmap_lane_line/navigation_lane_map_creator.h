/*
 * Copyright (c) TL auto Co., Ltd. 2021-2022. All rights reserved.
 */

#pragma once

#include <algorithm>
#include <iomanip>
#include <list>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include "common/vehicle_state/vehicle_state_provider.h"
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/navigation_lane_path_generator.h"
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/navigation_lanecentral_constructor.h"

#include "planning/proto/navigation_hdmap_config.pb.h"
#include "proto/common/vehicle_state.pb.h"
#include "proto/fsm/function_manager.pb.h"
#include "proto/map/navigation.pb.h"
#include "proto/planning/lanemarkers_lane_line.pb.h"

/**
 * @namespace TL::navigation_hdmap
 * @brief TL::navigation_hdmap
 */
namespace TL {
namespace planning {
using TL::planning::NaviPathTuple;
using TL::planning::lanelineprocess::DebounceModule;

class NavigationLaneMapCreator {
 public:
  NavigationLaneMapCreator() = default;
  explicit NavigationLaneMapCreator(
      const planning::PerceptionMapConfig& config);
  ~NavigationLaneMapCreator() = default;

  TL::common::Status Init();

  /**
   * @brief Generate a real-time relative map of approximately 250 m in length
   * based on several navigation line segments and map generation configuration
   * information.
   * @param map_config Map generation configuration information.
   * @param map_msg A pointer which outputs the real-time relative map.
   * @return True if the real-time relative map is created; false otherwise.
   */
  bool CreateMap(const std::list<NaviPathTuple>& navigation_path_list,
                 navigation_hdmap::MapMsg* map_msg);

  void SetLaneSpeedLimit(const double speed_limit) {
    hdmap_lane_speed_limit_ = speed_limit;
  }

  enum ChangeLaneTypeState { None, Left_Start, Right_Start, End };

  void SetChangeLaneType(
      const routing::PerceptionChangeLaneTypes& change_lane_type);

  void SetAdcIsInTunnel(bool is_in_tunnel) { adc_is_in_tunnel_ = is_in_tunnel; }

  void SetVehicleState(
      const std::shared_ptr<const common::VehicleState>& vehicle_state) {
    vehicle_state_ = vehicle_state;
  }

  void SetFctIn(
      const std::shared_ptr<const TL::functionmanager::FunctionManagerIn>&
          fct_in) {
    fct_in_ = fct_in;
  }

  const TL::planning::ChangeLaneTypeInfo& GetChangeLaneTypeInfo() {
    return change_lane_type_info_;
  }

  const TL::planning::CreatMapTime& GetCreatMapTime() {
    return creat_map_time_;
  }

 private:
  /**
   * @brief Create a single lane map.
   * @param navi_path_tuple A navigation path tuple.
   * @param map_config Map generation configuration information.
   * @param lane_marker The Perceived obstacle information and the lane
   * markings are used here.
   * @param hdmap The output single lane map in high-definition map format in
   * the relative map.
   * @param navigation_path The output navigation path map in the relative map.
   * @return True if the map is created; false otherwise.
   */
  bool CreateSingleLaneMap(
      const NaviPathTuple& navi_path_tuple,
      const perception::LaneMarkers& lane_marker, hdmap::Map* hdmap,
      google::protobuf::Map<std::string, navigation_hdmap::NavigationPath>*
          navigation_path);

  bool SetRoutingAndRoad(TL::routing::RoutingResponse* inrouting,
                         TL::hdmap::Map* hd_map);
  static void AddOtherPassage(hdmap::Map* hd_map,
                              routing::RoutingResponse* routing_response,
                              int index);
  void ChangeLaneTypeDecider(const perception::LaneMarkers& lane_marker);
  void DealStartState(bool input);
  // the configuration information required by the `NavigationLaneMapCreator`
  planning::PerceptionMapConfig config_;
  TL::planning::CreatMapTime creat_map_time_;
  routing::PerceptionChangeLaneTypes input_change_lane_types_;
  routing::PerceptionChangeLaneTypes current_change_lane_types_;
  std::shared_ptr<const common::VehicleState> vehicle_state_{nullptr};
  std::shared_ptr<const TL::functionmanager::FunctionManagerIn> fct_in_{
      nullptr};
  ChangeLaneTypeState change_lane_type_state_{ChangeLaneTypeState::None};
  int change_count_{0};
  DebounceModule change_type_diff_debounce_;
  TL::planning::ChangeLaneTypeInfo change_lane_type_info_;
  double hdmap_lane_speed_limit_ = 0.0;
  bool adc_is_in_tunnel_{false};
};

}  // namespace planning
}  // namespace TL
