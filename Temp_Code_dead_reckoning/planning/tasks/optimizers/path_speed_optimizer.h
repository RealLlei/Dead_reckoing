/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file path_speed_optimizer.h
 **/

#pragma once

#include <memory>

#include "common/status/status.h"
#include "planning/reference_line/reference_line.h"
#include "planning/tasks/task.h"
#include "proto/common/pnc_point.pb.h"

namespace TL::planning {

class PathSpeedOptimizer : public Task {
 public:
  explicit PathSpeedOptimizer(const TaskConfig& config);
  PathSpeedOptimizer(const TaskConfig& config,
                     const std::shared_ptr<DependencyInjector>& injector);
  ~PathSpeedOptimizer() override = default;
  TL::common::Status Execute(
      Frame* frame, ReferenceLineInfo* reference_line_info) override;
  TL::common::Status Execute(Frame* frame) override;

 protected:
  /**
   * @brief Process
   * 
   * @param reference_line 
   * @param init_point 
   * @param path_reusable 
   * @param path_data 
   * @param speed_data 
   * @return TL::common::Status 
   */
  virtual TL::common::Status Process(
      const ReferenceLine& reference_line,
      const common::TrajectoryPoint& init_point, PathData* path_data,
      SpeedData* speed_data);

  virtual TL::common::Status Process(
      const ReferenceLine& reference_line,
      const common::TrajectoryPoint& init_point,
      DiscretizedTrajectory* discretized_trajectory);

  /**
     * @brief 
     * 
     * @return TL::common::Status 
     */
  virtual TL::common::Status Process(Frame* frame);

  /**
   * @brief Record debug info
   * 
   * @param path_data 
   * @param speed_data 
   */
  void RecordDebugInfo(const PathData& path_data, const SpeedData& speed_data);
};

}  // namespace TL::planning
