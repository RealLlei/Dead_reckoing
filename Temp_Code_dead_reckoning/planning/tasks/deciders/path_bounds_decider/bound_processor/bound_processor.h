/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  planning bound processor base class
 * Author: ROC
 */

#pragma once

#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include "common/status/status.h"
#include "planning/common/dependency_injector.h"
#include "planning/common/frame.h"
#include "planning/tasks/deciders/path_bounds_decider/util/path_info.h"

namespace TL {
namespace planning {

class BoundProcessor {
 public:
  explicit BoundProcessor(const TaskConfig& config);
  BoundProcessor(const std::shared_ptr<DependencyInjector>& injector,
                 const TaskConfig& config);
  virtual ~BoundProcessor() = default;

  /**
   * @brief Path bound process
   *
   * @param reference_line_info
   * @param path_bound
   * @param frame
   * @param lane_type_pool
   * @return TL::common::Status
   */
  virtual TL::common::Status Process(
      ReferenceLineInfo* reference_line_info,
      std::vector<std::tuple<double, double, double>>* path_bound, Frame* frame,
      std::vector<LaneType>* lane_type_pool) = 0;

  /**
   * @brief Path bound Init process
   *
   * @param lane_borrow_info
   * @param blocking_obstacle_id
   * @param borrow_lane_type
   */
  virtual bool BoundInit(const PathInfo::LaneBorrowInfo& lane_borrow_info,
                         std::string* blocking_obstacle_id,
                         std::string* borrow_lane_type);

  /**
   * @brief Blocking obstacle id init process
   *
   * @param blocking_obstacle_id
   */
  virtual bool BlockingIDInit(std::string* blocking_obstacle_id);

  const std::shared_ptr<DependencyInjector>& GetInjector() const {
    return injector_;
  }

  const TaskConfig& GetConfig() const { return config_; }

 private:
  std::shared_ptr<DependencyInjector> injector_;
  TaskConfig config_;
};
}  // namespace planning
}  // namespace TL
