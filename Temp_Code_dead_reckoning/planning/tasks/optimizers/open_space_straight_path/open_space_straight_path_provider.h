/*
 * Copyright (c) TL Technologies Co., Ltd. 2022. All rights reserved.
 * Description:  open_space_straight_path_provider.h
 */

#pragma once

#include <algorithm>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "common/math/vec2d.h"
#include "planning/tasks/optimizers/path_optimizer.h"
#include "planning/proto/map_parking.pb.h"
#include "proto/common/pnc_point.pb.h"
#include "proto/common/types.pb.h"
#include "proto/common/vehicle_config.pb.h"
#include "proto/common/vehicle_model_config.pb.h"
#include "proto/common/vehicle_state.pb.h"
#include "proto/perception/perception_freespace.pb.h"
#include "proto/perception/perception_obstacle.pb.h"
#include "proto/perception/perception_parking_lot.pb.h"

namespace TL {
namespace planning {

class OpenSpaceStraightPathProvider : public PathOptimizer {
 public:
  /**
  * @brief Construct a new Open Space Straight Path Provider object
  * 
  * @param config 
  * @param injector 
  */
  OpenSpaceStraightPathProvider(
      const TaskConfig& config,
      const std::shared_ptr<DependencyInjector>& injector);

 private:
  /**
   * @brief base class virtual function, need override
   *
   * @param path_data
   * @param init_point
   * @param speed_data
   * @return TL::common::Status
   */
  TL::common::Status Process(const SpeedData& /*speed_data*/,
                                const ReferenceLine& /*reference_line*/,
                                const common::TrajectoryPoint& /*init_point*/,
                                bool /*path_reusable*/,
                                PathData* /*path_data*/) override {
    return common::Status::OK();
  }

  /**
   * @brief 
   * 
   * @return TL::common::Status 
   */
  TL::common::Status Process() override;
  /**
 * @brief 
 * 
 * @param start_point planning start point
 * @param path_data path data
 * @param direction forward=1 ,backward=-1
 */
  void GenerateDirectMovingPath(const common::TrajectoryPoint& start_point,
                                double direction,
                                planning::DiscretizedPath* path_data) const;
  /**
 * @brief 
 * 
 * @param moving_direction forward=1 ,backward=-1
 * @param vehicle_state  
 * @return true set is stop trajectory
 * @return false 
 */
  bool NeedStopDecision(double moving_direction,
                        const common::VehicleState& vehicle_state);
  /**
 * @brief update openspace start point 
 * 
 * @param vehicle_state vehicle state information
 */
  void UpdateOpenSpaceStartPoint(const common::VehicleState& vehicle_state,
                                 common::TrajectoryPoint* start_point);

  planning::AVPStatus_ParkingType last_parking_type_ =
      planning::AVPStatus::NOSTATE;
  common::math::Vec2d original_direction_;
  common::math::Vec2d original_point_;
};

}  // namespace planning
}  // namespace TL
