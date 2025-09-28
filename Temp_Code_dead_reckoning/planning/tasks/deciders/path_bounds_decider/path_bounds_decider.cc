/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  planning path bounds decider
 */

#include "planning/tasks/deciders/path_bounds_decider/path_bounds_decider.h"

#include <algorithm>
#include <cstddef>
#include <functional>
#include <limits>
#include <memory>
#include <set>

#include "absl/strings/match.h"
#include "absl/strings/str_cat.h"
#include "common/status/status.h"
#include "planning/common/path_boundary.h"
#include "planning/common/planning_context.h"
#include "planning/common/planning_gflags.h"
#include "planning/tasks/deciders/path_bounds_decider/bound_processor/fallback_bound_processor.h"
#include "planning/tasks/deciders/path_bounds_decider/bound_processor/lane_change_bound_processor.h"
#include "planning/tasks/deciders/path_bounds_decider/bound_processor/lane_keep_bound_processor.h"
#include "planning/tasks/deciders/path_bounds_decider/bound_processor/pullover_bound_processor.h"
#include "planning/tasks/deciders/utils/path_decider_obstacle_utils.h"

namespace TL {
namespace planning {

// using TL::common::Clock;
using TL::common::ErrorCode;
using TL::common::Status;
using TL::planning::PathInfo;

PathBoundsDecider::PathBoundsDecider(
    const TaskConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : Decider(config, injector),
      lane_keep_bound_processor_(new LaneKeepBoundProcessor(injector, config)),
      lane_change_bound_processor_(
          new LaneChangeBoundProcessor(injector, config)),
      pull_over_bound_processor_(new PullOverBoundProcessor(injector, config)),
      fallback_bound_processor_(new FallbackBoundProcessor(injector, config)) {}

Status PathBoundsDecider::Process(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  // PERF_FUNCTION_WITH_NAME("path_bounds_decider");
  // PERF_BLOCK_START();
  if (frame == nullptr || reference_line_info == nullptr) {
    AERROR << "frame or reference_line_info is nullptr!";
    return Status(ErrorCode::PLANNER_CRUISING_PATHBOUNDS_ERROR);
  }

  // Skip the path boundary decision if reusing the path.
  if (FLAGS_enable_skip_path_tasks && reference_line_info->path_reusable()) {
    return Status::OK();
  }

  std::vector<PathBoundary> candidate_path_boundaries;
  std::vector<LaneType> lane_type_pool;

  // 1.fallback path boundary process
  if (!FallbackPathBoundProcess(frame, reference_line_info,
                                &candidate_path_boundaries, &lane_type_pool)) {
    AERROR << "Fallback path bound process failed.";
    return Status(ErrorCode::PLANNER_CRUISING_PATHBOUNDS_ERROR);
  }

  // 2.universal path boundary process
  if (!UniversalPathBoundProcess(frame, reference_line_info,
                                 &candidate_path_boundaries, &lane_type_pool)) {
    AERROR << "Universal path bound process failed.";
    return Status(ErrorCode::PLANNER_CRUISING_PATHBOUNDS_ERROR);
  }

  // Process success
  reference_line_info->SetCandidatePathBoundaries(
      std::move(candidate_path_boundaries));

  return Status::OK();
}

bool PathBoundsDecider::FallbackPathBoundProcess(
    Frame* const frame, ReferenceLineInfo* const reference_line_info,
    std::vector<PathBoundary>* const candidate_path_boundaries,
    std::vector<LaneType>* const lane_type_pool) {
  if (frame == nullptr || reference_line_info == nullptr ||
      candidate_path_boundaries == nullptr) {
    AERROR << "Fallback path process input has nullptr pointer.";
    return false;
  }

  PathBound fallback_path_bound;
  Status fallback_ret = fallback_bound_processor_->Process(
      reference_line_info, &fallback_path_bound, frame, lane_type_pool);

  if (!fallback_ret.ok()) {
    AERROR << "Cannot generate a fallback path bound.";
    return false;
  }
  if (fallback_path_bound.empty()) {
    AERROR << "Failed to get a valid fallback path boundary";
    return false;
  }
  std::vector<std::pair<double, double>> fallback_path_bound_pair;
  fallback_path_bound_pair.reserve(fallback_path_bound.size());
  for (const auto& fallback_bound : fallback_path_bound) {
    fallback_path_bound_pair.emplace_back(std::get<1>(fallback_bound),
                                          std::get<2>(fallback_bound));
  }

  candidate_path_boundaries->emplace_back(
      std::get<0>(fallback_path_bound[0]),
      reference_line_info->PathBoundsDeciderResolution(),
      fallback_path_bound_pair);
  candidate_path_boundaries->back().set_label("fallback");

  return true;
}

bool PathBoundsDecider::UniversalPathBoundProcess(
    Frame* const frame, ReferenceLineInfo* const reference_line_info,
    std::vector<PathBoundary>* const candidate_path_boundaries,
    std::vector<LaneType>* const lane_type_pool) {
  if (frame == nullptr || reference_line_info == nullptr ||
      candidate_path_boundaries == nullptr) {
    AERROR << "Universal path process input has nullptr pointer.";
    return false;
  }

  // 1.history trace lane keep path boundary process
  if (frame->GetMachineStateType() ==
      functionmanager::MachineStateType::HISTORY_TRACE_TYPE) {
    std::vector<PathInfo::LaneBorrowInfo> history_trace_borrow_info;
    history_trace_borrow_info.emplace_back(PathInfo::LaneBorrowInfo::NO_BORROW);
    if (!LaneKeepProcess(history_trace_borrow_info, frame, reference_line_info,
                         candidate_path_boundaries, lane_type_pool)) {
      AERROR << "Failed to generate lane keep bound.";
      return false;
    }
    return true;
  }

  // 2.pull-over path boundary process
  if (injector_->planning_context()
          ->mutable_planning_status()
          ->mutable_pull_over()
          ->plan_pull_over_path()) {
    if (!PullOverProcess(frame, reference_line_info, candidate_path_boundaries,
                         lane_type_pool)) {
      AERROR << "Failed generate pullover path bound.";
    } else {
      return true;
    }
  }

  // 3.lane change path boundary process
  if (FLAGS_enable_smarter_lane_change &&
      reference_line_info->IsChangeLanePath()) {
    if (!LaneChangeProcess(frame, reference_line_info,
                           candidate_path_boundaries, lane_type_pool)) {
      AERROR << "Failed generate lane change path bound.";
    } else {
      return true;
    }
  }
  // when the lane change process is completed,
  // update status of lane change.
  auto* const ego_info = injector_->ego_info();
  if (!ego_info->GetIsLaneChangeStart()) {
    ego_info->SetIsLaneChangeStart(true);
  }

  // 4.lane keep borrow info list switch
  std::vector<PathInfo::LaneBorrowInfo> lane_keep_borrow_info;
  if (FLAGS_enable_original_lane_borrow_process) {
    LaneBorrowSwitch(*reference_line_info, &lane_keep_borrow_info);
  } else {
    lane_keep_borrow_info.emplace_back(PathInfo::LaneBorrowInfo::NO_BORROW);
  }
  // 5.lane keep path boundary process
  if (!LaneKeepProcess(lane_keep_borrow_info, frame, reference_line_info,
                       candidate_path_boundaries, lane_type_pool)) {
    AERROR << "Failed to generate lane keep bound.";
    return false;
  }

  return true;
}

bool PathBoundsDecider::PullOverProcess(
    Frame* const frame, ReferenceLineInfo* const reference_line_info,
    std::vector<PathBoundary>* const candidate_path_boundaries,
    std::vector<LaneType>* const lane_type_pool) {
  if (frame == nullptr || reference_line_info == nullptr ||
      candidate_path_boundaries == nullptr) {
    AERROR << "Pull over process input pointer is nullptr!";
    return false;
  }

  auto* const pull_over_status = injector_->planning_context()
                                     ->mutable_planning_status()
                                     ->mutable_pull_over();
  PathBound pull_over_path_bound;
  Status pull_over_ret = pull_over_bound_processor_->Process(
      reference_line_info, &pull_over_path_bound, frame, lane_type_pool);

  if (!pull_over_ret.ok() || pull_over_path_bound.empty()) {
    AERROR << "Cannot generate a pullover path bound, do lane keep planning.";
    return false;
  }

  // Update the fallback path boundary into the reference_line_info.
  std::vector<std::pair<double, double>> pull_over_path_bound_pair;
  pull_over_path_bound_pair.reserve(pull_over_path_bound.size());
  for (const auto& pull_over_bound : pull_over_path_bound) {
    pull_over_path_bound_pair.emplace_back(std::get<1>(pull_over_bound),
                                           std::get<2>(pull_over_bound));
  }

  PathBoundary pull_over_path_boundary(
      std::get<0>(pull_over_path_bound[0]),
      reference_line_info->PathBoundsDeciderResolution(),
      pull_over_path_bound_pair);
  pull_over_path_boundary.set_label("regular/pullover");

  candidate_path_boundaries->emplace_back(pull_over_path_boundary);

  ADEBUG << "Completed pullover path boundaries generation.";

  // set debug info in planning_data
  auto* const pull_over_debug = reference_line_info->mutable_debug()
                                    ->mutable_planning_data()
                                    ->mutable_pull_over();
  pull_over_debug->mutable_position()->CopyFrom(pull_over_status->position());
  pull_over_debug->set_theta(pull_over_status->theta());
  pull_over_debug->set_length_front(pull_over_status->length_front());
  pull_over_debug->set_length_back(pull_over_status->length_back());
  pull_over_debug->set_width_left(pull_over_status->width_left());
  pull_over_debug->set_width_right(pull_over_status->width_right());

  return true;
}

bool PathBoundsDecider::LaneChangeProcess(
    Frame* const frame, ReferenceLineInfo* const reference_line_info,
    std::vector<PathBoundary>* const candidate_path_boundaries,
    std::vector<LaneType>* const lane_type_pool) {
  if (frame == nullptr || reference_line_info == nullptr ||
      candidate_path_boundaries == nullptr) {
    AERROR << "Pull over process input pointer is nullptr!";
    return false;
  }

  std::string blocking_obstacle_id;
  lane_change_bound_processor_->BlockingIDInit(&blocking_obstacle_id);

  PathBound lane_change_path_bound;
  Status lane_change_ret = lane_change_bound_processor_->Process(
      reference_line_info, &lane_change_path_bound, frame, lane_type_pool);

  if (!lane_change_ret.ok() || lane_change_path_bound.empty()) {
    AERROR << "Cannot generate lane change path bound.";
    return false;
  }

  std::vector<std::pair<double, double>> lane_change_path_bound_pair;
  lane_change_path_bound_pair.reserve(lane_change_path_bound.size());
  for (const auto& lane_change_bound : lane_change_path_bound) {
    lane_change_path_bound_pair.emplace_back(std::get<1>(lane_change_bound),
                                             std::get<2>(lane_change_bound));
  }

  PathBoundary lane_change_path_boundary(
      std::get<0>(lane_change_path_bound[0]),
      reference_line_info->PathBoundsDeciderResolution(),
      lane_change_path_bound_pair);
  lane_change_path_boundary.set_label("regular/lane_change");
  lane_change_path_boundary.set_blocking_obstacle_id(blocking_obstacle_id);

  candidate_path_boundaries->emplace_back(lane_change_path_boundary);
  if (FLAGS_enable_path_bound_record_debug) {
    PathInfo::RecordDebugInfo(lane_change_path_bound, "regular_lane_change",
                              reference_line_info);
  }

  return true;
}

bool PathBoundsDecider::LaneBorrowSwitch(
    const ReferenceLineInfo& reference_line_info,
    std::vector<PathInfo::LaneBorrowInfo>* const lane_borrow_info_list) {
  if (lane_borrow_info_list == nullptr) {
    AERROR << "lane_borrow_info_list is nullptr.";
    return false;
  }

  lane_borrow_info_list->clear();
  lane_borrow_info_list->emplace_back(PathInfo::LaneBorrowInfo::NO_BORROW);
  if (reference_line_info.is_path_lane_borrow()) {
    const auto& path_decider_status =
        injector_->planning_context()->planning_status().path_decider();
    for (const auto& lane_borrow_direction :
         path_decider_status.decided_side_pass_direction()) {
      if (lane_borrow_direction == PathDeciderStatus::LEFT_BORROW) {
        lane_borrow_info_list->emplace_back(
            PathInfo::LaneBorrowInfo::LEFT_BORROW);
      } else if (lane_borrow_direction == PathDeciderStatus::RIGHT_BORROW) {
        lane_borrow_info_list->emplace_back(
            PathInfo::LaneBorrowInfo::RIGHT_BORROW);
      }
    }
  }

  return true;
}

bool PathBoundsDecider::LaneKeepProcess(
    const std::vector<PathInfo::LaneBorrowInfo>& lane_borrow_info_list,
    Frame* const frame, ReferenceLineInfo* const reference_line_info,
    std::vector<PathBoundary>* const candidate_path_boundaries,
    std::vector<LaneType>* const lane_type_pool) {
  if (frame == nullptr || reference_line_info == nullptr ||
      candidate_path_boundaries == nullptr) {
    AERROR << "frame or reference_line_info or candidate_path_boundaries is "
              "nullptr!";
    return false;
  }
  // Try every possible lane-borrow option:
  // PathBound lane_keep_self_path_bound;
  // bool exist_self_path_bound = false;
  ADEBUG << "lane_borrow_info_list_size = " << lane_borrow_info_list.size();
  for (const auto& lane_borrow_info : lane_borrow_info_list) {
    PathBound lane_keep_path_bound;
    std::string blocking_obstacle_id;
    std::string borrow_lane_type;
    lane_keep_bound_processor_->BoundInit(
        lane_borrow_info, &blocking_obstacle_id, &borrow_lane_type);
    Status lane_keep_ret = lane_keep_bound_processor_->Process(
        reference_line_info, &lane_keep_path_bound, frame, lane_type_pool);
    if (!lane_keep_ret.ok()) {
      AERROR << "Cannot generate a lane keep path bound.";
      continue;
    }
    if (lane_keep_path_bound.empty()) {
      continue;
    }
    // Update the path boundary into the reference_line_info.
    std::vector<std::pair<double, double>> lane_keep_path_bound_pair;
    lane_keep_path_bound_pair.reserve(lane_keep_path_bound.size());
    for (const auto& lane_keep_bound : lane_keep_path_bound) {
      lane_keep_path_bound_pair.emplace_back(std::get<1>(lane_keep_bound),
                                             std::get<2>(lane_keep_bound));
    }

    std::string path_label = "self";
    if (FLAGS_enable_original_lane_borrow_process) {
      switch (lane_borrow_info) {
        case PathInfo::LaneBorrowInfo::LEFT_BORROW:
          path_label = "left";
          break;
        case PathInfo::LaneBorrowInfo::RIGHT_BORROW:
          path_label = "right";
          break;
        default:
          path_label = "self";
          // exist_self_path_bound = true;
          // lane_keep_self_path_bound = lane_keep_path_bound;
          break;
      }
    }

    if (FLAGS_enable_path_bound_record_debug) {
      PathInfo::RecordDebugInfo(lane_keep_path_bound, "regular_lane_keep",
                                reference_line_info);
    }

    PathBoundary lane_keep_path_boundary(
        std::get<0>(lane_keep_path_bound[0]),
        reference_line_info->PathBoundsDeciderResolution(),
        lane_keep_path_bound_pair);
    lane_keep_path_boundary.set_label(
        absl::StrCat("regular/lane_keep/", path_label, "/", borrow_lane_type));
    lane_keep_path_boundary.set_blocking_obstacle_id(blocking_obstacle_id);
    candidate_path_boundaries->emplace_back(lane_keep_path_boundary);
  }

  return true;
}

void PathBoundsDecider::RemoveRedundantPathBoundaries(
    std::vector<PathBoundary>* const candidate_path_boundaries) {
  if (candidate_path_boundaries == nullptr) {
    AERROR << "candidate_path_boundaries is empty.";
    return;
  }
  // 1. Check to see if both "left" and "right" exist.
  bool is_left_exist = false;
  std::vector<std::pair<double, double>> left_boundary;
  bool is_right_exist = false;
  std::vector<std::pair<double, double>> right_boundary;

  for (const auto& path_boundary : *candidate_path_boundaries) {
    if (absl::StrContains(path_boundary.label(), "left")) {
      is_left_exist = true;
      left_boundary = path_boundary.boundary();
    }
    if (absl::StrContains(path_boundary.label(), "right")) {
      is_right_exist = true;
      right_boundary = path_boundary.boundary();
    }
  }
  // 2. Check if "left" is contained by "right", and vice versa.
  if (!is_left_exist || !is_right_exist) {
    return;
  }
  bool is_left_redundant = false;
  bool is_right_redundant = false;
  if (IsContained(left_boundary, right_boundary)) {
    is_left_redundant = true;
  }
  if (IsContained(right_boundary, left_boundary)) {
    is_right_redundant = true;
  }

  // 3. If one contains the other, then remove the redundant one.
  for (size_t i = 0; i < candidate_path_boundaries->size(); ++i) {
    const auto& path_boundary = (*candidate_path_boundaries)[i];
    if (absl::StrContains(path_boundary.label(), "right") &&
        is_right_redundant) {
      (*candidate_path_boundaries)[i] = candidate_path_boundaries->back();
      candidate_path_boundaries->pop_back();
      break;
    }
    if (absl::StrContains(path_boundary.label(), "left") && is_left_redundant) {
      (*candidate_path_boundaries)[i] = candidate_path_boundaries->back();
      candidate_path_boundaries->pop_back();
      break;
    }
  }
}

bool PathBoundsDecider::IsContained(
    const std::vector<std::pair<double, double>>& lhs,
    const std::vector<std::pair<double, double>>& rhs) {
  if (lhs.size() > rhs.size()) {
    return false;
  }
  for (size_t i = 0; i < lhs.size(); ++i) {
    if (lhs[i].first < rhs[i].first) {
      return false;
    }
    if (lhs[i].second > rhs[i].second) {
      return false;
    }
  }

  return true;
}

}  // namespace planning
}  // namespace TL
