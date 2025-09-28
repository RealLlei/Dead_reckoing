/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  planning path fallback bound processor
 * Author: ROC
 */

#pragma once

#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "planning/tasks/deciders/path_bounds_decider/bound_processor/bound_processor.h"
#include "planning/tasks/deciders/path_bounds_decider/bound_processor/process_bound.h"

#include "planning/proto/planning_config.pb.h"
#include "proto/common/pnc_point.pb.h"
#include "proto/planning/path_bound.pb.h"

namespace TL {
namespace planning {

class FallbackBoundProcessor : public BoundProcessor {
 public:
  FallbackBoundProcessor(const std::shared_ptr<DependencyInjector>& injector,
                         const TaskConfig& config);

  /**
  * @brief must be generated!
  *        generate fallback bound (just consider lanes and adc) 
  * 
  * @param reference_line_info 
  * @param path_bound: a vector for every point(s,l_min,l_max) in path boundary
  * @param frame 
  * @param lane_type_pool: a vector for reusable lane type 
  * @return Status : finish generate fallback bound return OK ,others return ErroeCode  
  */

  common::Status Process(ReferenceLineInfo* reference_line_info,
                         PathBound* path_bound, Frame* frame,
                         std::vector<LaneType>* lane_type_pool) override;

 private:
  std::shared_ptr<ProcessBound> process_bound_;
};
}  // namespace planning
}  // namespace TL
