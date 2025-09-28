/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include <memory>
#include <utility>
#include <vector>

#include "planning/common/frame.h"
#include "planning/common/st_graph_data.h"
#include "planning/tasks/deciders/decider.h"
#include "planning/tasks/deciders/speed_bounds_decider/speed_limit_decider.h"
#include "planning/proto/planning_config.pb.h"

namespace TL {
namespace planning {

class SpeedBoundsDecider : public Decider {
 public:
  SpeedBoundsDecider(const TaskConfig& config,
                     const std::shared_ptr<DependencyInjector>& injector);

 private:
  common::Status Process(Frame* frame,
                         ReferenceLineInfo* reference_line_info) override;

  static double SetSpeedFallbackDistance(PathDecision* path_decision);

  static void RecordSTGraphDebug(
      const StGraphData& st_graph_data,
      planning_internal::STGraphDebug* st_graph_debug);

  /**
   * @brief Load centric accel calibration table from file
   * 
   * @return true load successfully
   * @return false load failed
   */
  bool LoadCentricAccelCalibrationTable();

  static void SpeedLimitByCrossObstacle(Frame* frame,
                                        ReferenceLineInfo* reference_line_info);
  /**
   * @brief 
   * 
   * @param reference_line_info 
   */
  static void CacheFrontLaneInfo(const Frame* frame,
                                 ReferenceLineInfo* reference_line_info);

  SpeedBoundsDeciderConfig speed_bounds_config_;
  SpeedLimitDecider speed_limit_decider_;
  std::pair<std::vector<double>, std::vector<double>>
      hdmap_centric_accel_calibration_table_;
  std::pair<std::vector<double>, std::vector<double>>
      perception_centric_accel_calibration_table_;
};

}  // namespace planning
}  // namespace TL
