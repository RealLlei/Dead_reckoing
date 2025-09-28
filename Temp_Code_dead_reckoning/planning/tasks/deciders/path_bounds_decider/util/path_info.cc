/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  planning path info
 * Author: ROC
 */

#include "planning/tasks/deciders/path_bounds_decider/util/path_info.h"
#include <algorithm>
#include <cstddef>

#include "common/file/log.h"
#include "planning/common/planning_gflags.h"
#include "planning/tasks/deciders/path_bounds_decider/bound_processor/process_bound.h"

namespace TL {
namespace planning {

void PathInfo::LanesAndADCPathBoundDebugInfo(
    const LanesAndADCPathBoundDebugInfoInput& input) {
  if (input.path_bound_type == nullptr || input.lane_borrow_info == nullptr ||
      input.path_bound == nullptr || input.reference_line_info == nullptr ||
      input.curr_neighbor_lane_widths == nullptr ||
      input.ADC_speed_buffers == nullptr || input.ADC_buffers == nullptr ||
      input.curr_left_bound_lanes == nullptr ||
      input.curr_right_bound_lanes == nullptr ||
      input.curr_left_bounds == nullptr || input.curr_right_bounds == nullptr ||
      input.curr_lane_left_widths == nullptr ||
      input.curr_lane_right_widths == nullptr) {
    AERROR << "input data is nullptr";
    return;
  }
  const auto& path_bound_type = *input.path_bound_type;
  const auto& lane_borrow_info = *input.lane_borrow_info;
  const auto& path_bound = *input.path_bound;
  auto* reference_line_info = input.reference_line_info;
  const auto borrowing_reverse_lane = input.borrowing_reverse_lane;
  const auto& curr_neighbor_lane_widths = *input.curr_neighbor_lane_widths;
  const auto& ADC_speed_buffers = *input.ADC_speed_buffers;
  const auto& ADC_buffers = *input.ADC_buffers;
  const auto& curr_left_bound_lanes = *input.curr_left_bound_lanes;
  const auto& curr_right_bound_lanes = *input.curr_right_bound_lanes;
  const auto& curr_left_bounds = *input.curr_left_bounds;
  const auto& curr_right_bounds = *input.curr_right_bounds;
  const auto& curr_lane_left_widths = *input.curr_lane_left_widths;
  const auto& curr_lane_right_widths = *input.curr_lane_right_widths;

  if (!FLAGS_enable_path_bound_debug) {
    return;
  }
  if (reference_line_info == nullptr) {
    AERROR << "reference_line_info is empty.";
    return;
  }
  TL::planning::BoundaryFromLanesAndADC* adc_path_bound = nullptr;
  TL::common::Path* path = nullptr;
  if ((*reference_line_info).debug().planning_data().path().empty()) {
    path = reference_line_info->mutable_debug()
               ->mutable_planning_data()
               ->add_path();
  } else {
    path = reference_line_info->mutable_debug()
               ->mutable_planning_data()
               ->mutable_path(0);
  }
  TL::planning::PathBoundDebug* path_bound_debug =
      path->mutable_path_bound_debug();
  path_bound_debug->set_path_bound_type(path_bound_type);
  int i = 0;
  for (auto path_bound_point : path_bound) {
    switch (path_bound_type) {
      case PathBoundType::FALL_BACK_PATH_BOUND: {
        adc_path_bound = path_bound_debug->add_fall_back_path_bound();
        break;
      }
      default: {
        if ((*reference_line_info)
                .debug()
                .planning_data()
                .path(0)
                .path_bound_debug()
                .path_bound()
                .size() <= i) {
          adc_path_bound = path_bound_debug->add_path_bound();
        } else {
          adc_path_bound = path_bound_debug->mutable_path_bound(i);
        }
      }
    }
    if (FLAGS_enable_original_lane_borrow_process) {
      if (lane_borrow_info == LaneBorrowInfo::LEFT_BORROW) {
        if (borrowing_reverse_lane) {
          adc_path_bound->set_lane_borrow_info(
              PathBoundLaneBorrowInfo::LEFT_BACKWARD_BORROW);
        } else {
          adc_path_bound->set_lane_borrow_info(
              PathBoundLaneBorrowInfo::LEFT_FORWARD_BORROW);
        }
      } else {
        if (lane_borrow_info == LaneBorrowInfo::RIGHT_BORROW) {
          if (borrowing_reverse_lane) {
            adc_path_bound->set_lane_borrow_info(
                PathBoundLaneBorrowInfo::RIGHT_BACKWARD_BORROW);
          } else {
            adc_path_bound->set_lane_borrow_info(
                PathBoundLaneBorrowInfo::RIGHT_FORWARD_BORROW);
          }
        } else {
          adc_path_bound->set_lane_borrow_info(
              PathBoundLaneBorrowInfo::NO_BORROW);
        }
      }
    } else {
      adc_path_bound->set_lane_borrow_info(
              PathBoundLaneBorrowInfo::NO_BORROW);
    }
    adc_path_bound->set_neighbor_lane_width(curr_neighbor_lane_widths[i]);
    adc_path_bound->set_adc_speed_buffer(ADC_speed_buffers[i]);
    adc_path_bound->set_adc_buffer(ADC_buffers[i]);
    adc_path_bound->set_left_bound_lane(curr_left_bound_lanes[i]);
    adc_path_bound->set_right_bound_lane(curr_right_bound_lanes[i]);
    adc_path_bound->set_left_bound_adc(curr_left_bounds[i]);
    adc_path_bound->set_right_bound_adc(curr_right_bounds[i]);
    adc_path_bound->set_lane_left_width(curr_lane_left_widths[i]);
    adc_path_bound->set_lane_right_width(curr_lane_right_widths[i]);
    adc_path_bound->mutable_path_boundaries_lanes_and_adc()->set_s(
        std::get<0>(path_bound_point));
    adc_path_bound->mutable_path_boundaries_lanes_and_adc()->set_l_min(
        std::get<1>(path_bound_point));
    adc_path_bound->mutable_path_boundaries_lanes_and_adc()->set_l_max(
        std::get<2>(path_bound_point));
    ++i;
  }
}

void PathInfo::PathBoundDebugInfo(
    const PathBoundType& path_bound_type, const PathBound& path_bound,
    ReferenceLineInfo* const reference_line_info) {
  if (!FLAGS_enable_path_bound_debug) {
    return;
  }
  if (reference_line_info == nullptr) {
    AERROR << "Reference_line_info is a nullptr.";
    return;
  }

  TL::planning::BoundaryFromLanesAndADC* path_bound_out = nullptr;
  int i = 0;
  for (auto path_bound_point : path_bound) {
    if (path_bound_type == FALL_BACK_PATH_BOUND) {
      path_bound_out = reference_line_info->mutable_debug()
                           ->mutable_planning_data()
                           ->mutable_path(0)
                           ->mutable_path_bound_debug()
                           ->mutable_fall_back_path_bound(i);
    } else {
      path_bound_out = reference_line_info->mutable_debug()
                           ->mutable_planning_data()
                           ->mutable_path(0)
                           ->mutable_path_bound_debug()
                           ->mutable_path_bound(i);
    }
    path_bound_out->set_path_bound_type(path_bound_type);
    path_bound_out->mutable_path_boundaries()->set_s(
        std::get<0>(path_bound_point));
    path_bound_out->mutable_path_boundaries()->set_l_min(
        std::get<1>(path_bound_point));
    path_bound_out->mutable_path_boundaries()->set_l_max(
        std::get<2>(path_bound_point));
    ++i;
  }
}

void PathInfo::RecordDebugInfo(const PathBound& path_boundaries,
                               const std::string& debug_name,
                               ReferenceLineInfo* const reference_line_info) {
  // Sanity checks.
  if (path_boundaries.empty()) {
    AERROR << "path_boundaries is empty!";
    return;
  }
  if (reference_line_info == nullptr) {
    AERROR << "reference_line_info is nullptr!";
    return;
  }
  auto* mutable_path =
      reference_line_info->mutable_debug()->mutable_planning_data()->add_path();
  mutable_path->set_name(debug_name);
  const auto size = static_cast<int>(path_boundaries.size());
  mutable_path->mutable_frenet_frame_point()->Reserve(size);
  for (const PathBoundPoint& path_bound_point : path_boundaries) {
    auto* mutable_frenet_point = mutable_path->add_frenet_frame_point();
    mutable_frenet_point->set_s(std::get<0>(path_bound_point));
    mutable_frenet_point->set_dl(0.0);
    mutable_frenet_point->set_ddl(0.0);
    mutable_frenet_point->set_l(std::get<2>(path_bound_point));
    mutable_frenet_point->set_l1(std::get<1>(path_bound_point));
  }
}

void PathInfo::PathBoundsDebugString(const PathBound& path_boundaries) {
  if (!FLAGS_enable_path_bound_debug) {
    return;
  }
  for (size_t i = 0; i < path_boundaries.size(); ++i) {
    ADEBUG << "idx " << i
           << "; path_boundary_s:" << std::get<0>(path_boundaries[i])
           << " l_min:" << std::get<1>(path_boundaries[i])
           << " l_max:" << std::get<2>(path_boundaries[i]);
  }
}

void PathInfo::PathBoundsDebugString(
    const PathBound& path_boundaries,
    const ReferenceLineInfo& reference_line_info, const double adc_frenet_l) {
  if (!FLAGS_enable_path_bound_debug) {
    return;
  }
  routing::ChangeLaneType lane_change_type =
      ProcessBound::JudgeLaneChangeType(adc_frenet_l);
  double path_boundary_s = 0.0;
  bool is_solid_lane = false;
  for (size_t i = 0; i < path_boundaries.size(); ++i) {
    path_boundary_s = std::get<0>(path_boundaries[i]);
    is_solid_lane = !ProcessBound::CheckLaneBoundaryType(
        reference_line_info, path_boundary_s, lane_change_type);
    ADEBUG << "idx " << i
           << " path_boundary_s:" << std::get<0>(path_boundaries[i])
           << " is_solid_lane:" << is_solid_lane
           << " path_boundary_l_min:" << std::get<1>(path_boundaries[i])
           << " path_boundary_l_max:" << std::get<2>(path_boundaries[i]);
  }
}

void PathInfo::TowingLineDebugInfo(
    const PathBound& path_boundaries, const std::vector<double>& towing_line,
    ReferenceLineInfo* const reference_line_info) {
  // Sanity checks.
  if (path_boundaries.empty() || towing_line.empty()) {
    AERROR << "path_boundaries or towing_line is empty!";
    return;
  }
  if (reference_line_info == nullptr) {
    AERROR << "reference_line_info is nullptr!";
    return;
  }
  auto* mutable_path =
      reference_line_info->mutable_debug()->mutable_planning_data()->add_path();
  mutable_path->set_name("towing_line");
  const auto size = static_cast<int>(towing_line.size());
  mutable_path->mutable_frenet_frame_point()->Reserve(size);
  for (size_t i = 0; i < towing_line.size(); ++i) {
    auto* mutable_frenet_point = mutable_path->add_frenet_frame_point();
    mutable_frenet_point->set_s(std::get<0>(path_boundaries[i]));
    mutable_frenet_point->set_dl(0.0);
    mutable_frenet_point->set_ddl(0.0);
    mutable_frenet_point->set_l(towing_line[i]);
  }
}
}  // namespace planning
}  // namespace TL
