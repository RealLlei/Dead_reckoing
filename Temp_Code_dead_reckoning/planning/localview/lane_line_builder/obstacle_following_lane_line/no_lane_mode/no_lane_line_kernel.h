/******************************************************************************
 * Copyright (c) TL Technologies Co., Ltd. 2019-2022. All rights reserved.
 * Author: LingPeng
 * Created Time: 2022/4/21
 *****************************************************************************/

#pragma once

#include <list>
#include <memory>
#include <string>
#include <vector>

#include "common/file/file.h"
#include "common/file/log.h"
#include "common/math/linear_interpolation.h"
#include "common/math/math_utils.h"
#include "common/time/clock.h"
#include "map/hdmap/hdmap_util.h"
#include "planning/common/planning_gflags.h"
#include "planning/localview/lane_line_builder/obstacle_following_lane_line/coordinate_system_convert.h"
#include "planning/localview/lane_line_builder/obstacle_following_lane_line/ego_vehicle_state.h"
#include "planning/localview/lane_line_builder/obstacle_following_lane_line/lane_line_list.h"
#include "planning/localview/lane_line_builder/obstacle_following_lane_line/obstacle_perception_manager.h"
#include "planning/localview/lane_line_builder/obstacle_following_lane_line/obstacle_perception_no_lane.h"

#include "proto/common/vehicle_config.pb.h"
#include "proto/common/vehicle_model_config.pb.h"
#include "proto/common/vehicle_signal.pb.h"
#include "proto/common/vehicle_state.pb.h"
#include "proto/perception/perception_obstacle.pb.h"

namespace TL {
namespace planning {
namespace nolane {

class NoLaneLineKernel {
 public:
  NoLaneLineKernel();

  NoLaneLineKernel(const NoLaneLineKernel& rhs) = delete;

  NoLaneLineKernel& operator=(const NoLaneLineKernel& rhs) = delete;

  ~NoLaneLineKernel() = default;

  enum class Mode { NormalObsWithoutLane, NoObsNoLane };

  void StartRecord();

  /**
   * @brief main function in without-lane-line mode.
   * @param obstacles_percep original perception obstacles
   * @return  true: successful to generate map;
   *          false: failed to generate map.
   */
  bool Process(const std::shared_ptr<LocalView>& local_view);

  /**
   * @brief try to cover scenario in which there is no any obstacle while
   * obstacles previous stored is sufficient for generate lane line in this
   * frame.if it's sufficient reset the value of mode to NoObsNoLane;
   * @return it is sufficient even if current frame has no obstacle return true,
   * otherwise return false.
   */
  bool IsSufficientWithObsesPrev();

  void SendToCyber(
      const TL::perception::PerceptionObstacles& perception_obstacles);

  /**-----------------------get function-----------------------------**/
  const obstacles_ptr_list& GetObstalcesPresent() const;
  const obstacles_ptr_list& GetObstalcesPrevious() const;
  const obstacles_ptr_list& GetObstaclesMerged() const;
  const EgoVehicleState& GetEgoState() const;
  const LaneLineList& GetLaneLineList() const;
  const CoordinateSystemConvert& GetCoordinateSystemConvert() const;
  Mode GetMode() const;
  const std::shared_ptr<TL::planning::WithoutLaneFollow>& GetPtrWithoutLane()
      const;

  /**-----------------------set function-----------------------------**/
  obstacles_ptr_list* const MutableObstalcesPresent();
  obstacles_ptr_list* const MutableGetObstalcesPrevious();
  obstacles_ptr_list* const MutableGetObstaclesMerged();
  EgoVehicleState* const MutableGetEgoState();
  LaneLineList* const MutableGetLaneLineList();
  CoordinateSystemConvert* const MutableCoordinateSystemConvert();

  bool CreatMapOut(const std::shared_ptr<LaneCenterLine>& output_lane_ptr);

  const std::shared_ptr<navigation_hdmap::MapMsg>& GetMapMsg() {
    return current_map_msg_;
  }

  const std::shared_ptr<routing::RoutingResponse>& GetRoutingResponse() {
    return current_routing_response_;
  }

 private:
  int counter_ = 0;
  ObstaclePerceptionManager obstacles_perception_manager_;
  obstacles_ptr_list obstacles_merged_;
  EgoVehicleState ego_state_;
  LaneLineList lane_line_list_;
  CoordinateSystemConvert coordinate_system_convert_;
  Mode mode_;
  std::shared_ptr<LocalView> local_view_ = nullptr;
  std::shared_ptr<TL::navigation_hdmap::MapMsg> current_map_msg_ = nullptr;
  std::shared_ptr<routing::RoutingResponse> current_routing_response_ = nullptr;
};

}  // namespace nolane
}  // namespace planning
}  // namespace TL
