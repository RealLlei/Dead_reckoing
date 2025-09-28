/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  planning path fallback bound processor
 * Author: ROC
 */

#include "planning/tasks/deciders/path_bounds_decider/bound_processor/fallback_bound_processor.h"

#include <algorithm>
#include <functional>
#include <limits>
#include <memory>
#include <set>

#include "planning/tasks/deciders/utils/path_decider_obstacle_utils.h"

namespace TL {
namespace planning {

// using TL::common::Clock;
using TL::common::ErrorCode;
using TL::common::Status;
using TL::planning::PathInfo;

FallbackBoundProcessor::FallbackBoundProcessor(
    const std::shared_ptr<DependencyInjector>& injector,
    const TaskConfig& config)
    : BoundProcessor(injector, config),
      process_bound_(new ProcessBound(injector, config)) {}

Status FallbackBoundProcessor::Process(
    ReferenceLineInfo* const reference_line_info, PathBound* const path_bound,
    Frame* const frame, std::vector<LaneType>* const lane_type_pool) {
  if (reference_line_info == nullptr || path_bound == nullptr ||
      frame == nullptr) {
    const std::string msg =
        "reference_line_info or path_bound or frame is nullptr.";
    AERROR << msg;
    return Status(ErrorCode::PLANNER_CRUISING_PATHBOUNDS_ERROR, msg);
  }

  // bound process init.
  process_bound_->InitPathBounds(frame, reference_line_info);

  // 1. Initialize the path boundaries to be an indefinitely large area.
  if (!process_bound_->InitPathBoundary(path_bound, GetInjector())) {
    const std::string msg = "Failed to initialize fallback path boundaries.";
    AERROR << msg;
    return Status(ErrorCode::PLANNER_CRUISING_PATHBOUNDS_ERROR, msg);
  }

  process_bound_->SetIsAllowLeftVirtualLaneBound(true);
  process_bound_->SetIsAllowRightVirtualLaneBound(true);
  process_bound_->SetIsAllowExpandLeftLaneBound(false);
  process_bound_->SetIsAllowExpandRightLaneBound(false);

  // 2. Decide a rough boundary based on lane info and ADC's position
  std::string dummy_borrow_lane_type;
  if (!process_bound_->GetBoundaryFromLanesAndADC(
          PathInfo::LaneBorrowInfo::NO_BORROW,
          PathBoundType::FALL_BACK_PATH_BOUND, path_bound,
          &dummy_borrow_lane_type,
          GetConfig().path_bounds_decider_config().adc_fallback_buffer(),
          lane_type_pool, true)) {
    const std::string msg =
        "Failed to decide a rough fallback boundary based on "
        "road information.";
    AERROR << msg;
    return Status(ErrorCode::PLANNER_CRUISING_PATHBOUNDS_ERROR, msg);
  }

  ADEBUG << "fallback path bound.";
  PathInfo::PathBoundsDebugString(*path_bound);

  ADEBUG << "Completed generating fallback path boundaries.";
  PathInfo::PathBoundDebugInfo(PathBoundType::FALL_BACK_PATH_BOUND, *path_bound,
                               reference_line_info);
  return Status::OK();
}

}  // namespace planning
}  // namespace TL
