/******************************************************************************
 * Copyright (c) TL Technologies Co., Ltd. 2019-2022. All rights reserved.
 * Author: LingPeng
 * Created Time: 2022/4/21
 *****************************************************************************/

#pragma once

#include <memory>
#include <tuple>
#include <utility>
#include <vector>

#include "planning/localview/lane_line_builder/obstacle_following_lane_line/ego_vehicle_state.h"
#include "planning/localview/lane_line_builder/obstacle_following_lane_line/fit_curve/fit_convoluted_helix.h"
#include "planning/localview/lane_line_builder/obstacle_following_lane_line/fit_curve/fit_osqp_curve.h"
#include "planning/localview/lane_line_builder/obstacle_following_lane_line/fit_curve/fit_polynomial_curve.h"
#include "planning/localview/lane_line_builder/obstacle_following_lane_line/lane_center_line.h"
#include "planning/reference_line/reference_line.h"

namespace TL {
namespace planning {
namespace nolane {
class LaneLineList {
 public:
  LaneLineList() = default;

  ~LaneLineList() = default;

  LaneLineList(const LaneLineList& rhs) = delete;

  LaneLineList& operator=(const LaneLineList& rhs) = delete;

  /**
   * @brief
   */
  void Clear();

  /**
   * @brief
   * @return true, at least has one lane. otherwise, return false.
   */
  bool SortLaneLines(
      std::shared_ptr<LaneCenterLine>* const optimal_lane_ptr = nullptr);

  /**
   * @brief
   * @param obstacle_followed_index
   * @return
   */
  bool ConstructLaneBetweenTrajectoryAndEgo(
      const StitchPointInfo& obstacle_followed_index,
      const EgoVehicleState& ego_state, const bool is_based_one_traj);

  /**
   * @brief construct lane-lines base stored obstacle.
   * @param obstacles
   * @param ego_state
   * @return numbers of lanes.
   */
  bool ConstructLaneLines(const obstacles_ptr_list& obstacles,
                          const EgoVehicleState& ego_state);

  void ConstructLaneForEachInDrivableList(bool based_on_traj_line = false);

  const std::shared_ptr<LaneCenterLine>& GetLaneLinePtrPrevious() const;
  const std::shared_ptr<LaneCenterLine>& GetLaneLinePtrOutput() const;

  /**
   * @brief Set the Lane Line Ptr Output object
   *
   * @param outputLaneLinePtr
   */
  void SetLaneLinePtrOutput(
      const std::shared_ptr<LaneCenterLine>& outputLaneLinePtr);

  void SetLaneLinePtrPrevious(
      const std::shared_ptr<LaneCenterLine>& laneLinePtrPrevious);

  bool ValidationCheckLaneLinePrevious(EgoVehicleState* const ego_state);

 private:
  std::shared_ptr<LaneCenterLine> lane_line_ptr_previous_;
  std::shared_ptr<LaneCenterLine> lane_line_ptr_output_;
  std::shared_ptr<LaneCenterLine> lane_line_ptr_previous_left_;
  std::shared_ptr<LaneCenterLine> lane_line_ptr_previous_right_;
  std::vector<std::shared_ptr<LaneCenterLine>> lane_line_drivable_;
};
}  // namespace nolane
}  // namespace planning
}  // namespace TL
