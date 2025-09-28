/******************************************************************************
 * Copyright 2022 The TL Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "planning/localview/local_view.h"

#include "proto/common/error_code.pb.h"
#include "proto/planning/planning.pb.h"
#include "proto/planning/trigger.pb.h"

namespace TL {
namespace planning {

class FaultCollect {
 public:
  FaultCollect();
  /**
   * @brief Collect input data error and planningAlg error for FM reporting
   *
   * @param local_view
   * @param ptr_trajectory_pb
   */
  void ProcessFaultCollect(const std::shared_ptr<LocalView>& local_view,
                           ADCTrajectory* ptr_trajectory_pb);

 private:
  /**
   * @brief Collect input data error
   *
   * @param local_view
   */
  void ProcessInputData(const std::shared_ptr<LocalView>& local_view);

  /**
   * @brief add fault
   *
   * @param error_code
   */
  void AddFaultDataType(const TL::common::ErrorCode& error_code);

 private:
  ADCTrajectory* ptr_trajectory_pb_ = nullptr;
  PlanningErrorCodeMap planning_error_code_map_;
};

}  // namespace planning
}  // namespace TL
