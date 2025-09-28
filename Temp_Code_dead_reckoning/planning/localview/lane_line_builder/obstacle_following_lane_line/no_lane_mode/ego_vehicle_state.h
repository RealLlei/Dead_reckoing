/******************************************************************************
 * Copyright (c) TL Technologies Co., Ltd. 2019-2022. All rights reserved.
 * Author: LingPeng
 * Created Time: 2022/04/21
 *****************************************************************************/

#pragma once

#include <limits>
#include <list>
#include <memory>

#include "common/math/vec2d.h"
#include "planning/localview/lane_line_builder/obstacle_following_lane_line/common_util/util.h"
#include "planning/reference_line/reference_line.h"

#include "proto/common/pnc_point.pb.h"
#include "proto/common/vehicle_config.pb.h"
#include "proto/common/vehicle_state.pb.h"

namespace TL {
namespace planning {
namespace nolane {
class EgoVehicleState {
 public:
  EgoVehicleState();
  void UpdateEgoState(const TL::common::VehicleState& vehicle_state);

  void SendToCyber();

  void SetVehicleStatePresent(const common::VehicleState& vehicleStatePresent);
  void SetVehicleParam(const common::VehicleParam& vehicleParam);
  void SetVehicleStateHistory(
      const std::list<TL::common::VehicleState>& vehicleStateHistory);
  void SetVehicleSl(const common::SLPoint& vehicleSl);

  void ProjectionEgoSL(
      const std::shared_ptr<ReferenceLine>& prev_reference_line_ptr);

  const common::VehicleState& GetVehicleStatePresent() const;
  const common::VehicleParam& GetVehicleParam() const;
  const std::list<TL::common::VehicleState>& GetVehicleStateHistory() const;
  const common::SLPoint& GetVehicleSl() const;

  /**
   * @brief calculate min turn radius in difference speed.
   */
  void CalculateMinTurnRadius();

  /**
   * @return min turn radius in different speed.
   */
  double GetMinTurnRadius() const { return min_turn_radius_; }

  /**
   * @return max curvature in different speed.
   */
  double GetMaxCurvature() const { return 1 / min_turn_radius_; }

 private:
  std::list<TL::common::VehicleState>
      vehicle_state_history_;  // lp: has been converted
  TL::common::SLPoint vehicle_sl_;
  TL::common::VehicleParam vehicle_param_;
  TL::common::VehicleState vehicle_state_present_;  // lp:has been converted
  const int max_size_ = 100;
  double min_turn_radius_ = std::numeric_limits<double>::infinity();
};
}  // namespace nolane
}  // namespace planning
}  // namespace TL
