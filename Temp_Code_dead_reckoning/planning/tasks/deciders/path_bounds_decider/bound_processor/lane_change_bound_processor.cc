/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  planning path lane change bound processor
 * Author: ROC
 */

#include "planning/tasks/deciders/path_bounds_decider/bound_processor/lane_change_bound_processor.h"

#include <algorithm>
#include <cstddef>
#include <functional>
#include <limits>
#include <memory>
#include <set>
#include <utility>

#include "common/math/double_type.h"
#include "common/math/math_utils.h"
// #include "common/time/clock.h"
// #include "common/math/linear_interpolation.h"
#include "planning/common/planning_context.h"
#include "planning/common/planning_gflags.h"
#include "planning/tasks/deciders/path_bounds_decider/util/path_info.h"
#include "planning/tasks/deciders/utils/path_decider_obstacle_utils.h"
#include "proto/fsm/function_manager.pb.h"

namespace TL {
namespace planning {

// using TL::common::Clock;
using TL::common::ErrorCode;
using TL::common::Status;
using TL::common::math::double_type::Compare;
using TL::common::math::double_type::ComparedToZero;
using TL::common::math::double_type::DefinitelyGreater;
using TL::common::math::double_type::DefinitelyLess;
using TL::hdmap::LaneBoundaryType;
using TL::planning::PathInfo;

LaneChangeBoundProcessor::LaneChangeBoundProcessor(
    const std::shared_ptr<DependencyInjector>& injector,
    const TaskConfig& config)
    : BoundProcessor(injector, config),
      process_bound_(new ProcessBound(injector, config)),
      obs_static_process_(new ObsStaticProcessor(injector, config)) {}

Status LaneChangeBoundProcessor::Process(
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
    const std::string msg = "Failed to initialize path boundaries.";
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
          PathBoundType::LANE_CHANGE_PATH_BOUND, path_bound,
          &dummy_borrow_lane_type,
          GetConfig().path_bounds_decider_config().adc_lane_change_buffer(),
          lane_type_pool, true)) {
    const std::string msg =
        "Failed to decide a rough boundary based on "
        "road information.";
    AERROR << msg;
    return Status(ErrorCode::PLANNER_CRUISING_PATHBOUNDS_ERROR, msg);
  }
  ADEBUG << "Get boundary from lanes and ADC for lane change path bound.";

  // 3. Get lane change start point status
  std::pair<double, double> lane_change_start_sl(
      process_bound_->GetAdcFrenetS(), process_bound_->GetAdcFrenetL());
  // GetLaneChangeStartStatus(*reference_line_info, &lane_change_start_sl);

  // 4. Remove the S-length of target lane out of the path-bound.
  // GetBoundaryFromLaneChangeForbiddenZone(*reference_line_info, path_bound,
  //                                        &lane_change_start_sl);

  // 5. Calculate lane change after forbidden zone lane boundary
  GetBoundaryForLaneChangeConstraint(*frame, *reference_line_info, path_bound,
                                     lane_change_start_sl);

  // 6. Decide a rough boundary based on lane boundary type.
  // GetBoundaryFromLaneBoundaryType(reference_line_info, path_bound);

  // 7. Static obstacle proccess
  PathBound temp_path_bound = *path_bound;
  TowingPointsInfo towing_points;
  towing_points.resize(
      path_bound->size(),
      std::tuple<double, double, double, std::string>(0.0, 0.0, 0.0, "0"));
  obs_static_process_->GetProcessBound()->InitPathBounds(frame,
                                                         reference_line_info);

  const auto& indexed_obstacles =
      reference_line_info->path_decision()->obstacles();
  obs_static_process_->SortObstaclesForSweepLine(*reference_line_info,
                                                 indexed_obstacles);
  if (!obs_static_process_->Process(reference_line_info, frame, path_bound,
                                    blocking_obstacle_id_, &towing_points,
                                    false)) {
    const std::string msg =
        "Failed to decide fine tune the boundaries after "
        "taking into consideration all static obstacles.";
    AERROR << msg;
    return Status(ErrorCode::PLANNER_CRUISING_PATHBOUNDS_ERROR, msg);
  }

  ADEBUG << "lane change path bound.";
  PathInfo::PathBoundsDebugString(*path_bound, *reference_line_info,
                                  process_bound_->GetAdcFrenetL());
  // Append some extra path bound points to avoid zero-length path data.
  int counter = 0;
  while (!blocking_obstacle_id_->empty() &&
         path_bound->size() < temp_path_bound.size() &&
         counter < kNumExtraTailBoundPoint) {
    path_bound->push_back(temp_path_bound[path_bound->size()]);
    counter++;
  }

  ADEBUG << "Completed generating lane change path boundaries.";
  PathInfo::PathBoundDebugInfo(PathBoundType::LANE_CHANGE_PATH_BOUND,
                               *path_bound, reference_line_info);
  return Status::OK();
}

bool LaneChangeBoundProcessor::BlockingIDInit(
    std::string* const blocking_obstacle_id) {
  if (blocking_obstacle_id == nullptr) {
    AERROR << "blocking_obstacle_id is nullptr";
    return false;
  }

  blocking_obstacle_id_ = blocking_obstacle_id;
  return true;
}

void LaneChangeBoundProcessor::GetLaneChangeStartStatus(
    const ReferenceLineInfo& reference_line_info,
    std::pair<double, double>* const lane_change_start_sl) {
  if (lane_change_start_sl == nullptr) {
    AERROR << "lane_change_start_sl ptr is nullptr!";
    return;
  }
  const ReferenceLine& reference_line = reference_line_info.reference_line();
  // If there is a pre-determined lane-change starting position, then use it;
  // otherwise, decide one.
  auto* const lane_change_status = GetInjector()
                                       ->planning_context()
                                       ->mutable_planning_status()
                                       ->mutable_change_lane();

  if (lane_change_status->change_lane_safety_swap_info()
          .has_lane_change_start_position()) {
    common::SLPoint start_point_sl;
    reference_line.XYToSL(lane_change_status->change_lane_safety_swap_info()
                              .lane_change_start_position(),
                          &start_point_sl);

    lane_change_start_sl->first =
        std::max(start_point_sl.s(), process_bound_->GetAdcFrenetS());
  } else {
    AERROR << "lane change gap error :no gap start position!";
    return;
  }
}

void LaneChangeBoundProcessor::GetBoundaryFromLaneChangeForbiddenZone(
    const ReferenceLineInfo& reference_line_info, PathBound* const path_bound,
    std::pair<double, double>* const lane_change_start_sl) {
  if (path_bound == nullptr || lane_change_start_sl == nullptr) {
    AERROR << "lane change path_bound or lane_change_start_sl ptr is nullptr!";
    return;
  }
  // Remove the target lane out of the path-boundary, up to the decided S.
  if (GetInjector()
          ->planning_context()
          ->mutable_planning_status()
          ->mutable_change_lane()
          ->change_lane_safety_swap_info()
          .has_lane_change_end_position()) {
    // If already passed the decided S, then return.
    // lane_change_status->set_exist_lane_change_start_position(false);
    return;
  }

  const ReferenceLine& reference_line = reference_line_info.reference_line();
  for (size_t i = 0; i < path_bound->size(); ++i) {
    double curr_s = std::get<0>((*path_bound)[i]);
    if (curr_s > lane_change_start_sl->first) {
      // set lane change prepare end point l
      lane_change_start_sl->second =
          i > 1 ? (process_bound_->GetAdcFrenetL() > KAlmostZero1ENegtive3
                       ? std::get<1>((*path_bound)[i - 1]) +
                             GetConfig()
                                 .path_bounds_decider_config()
                                 .lane_change_process_config()
                                 .lane_change_heuristic_line_buffer()
                       : std::get<2>((*path_bound)[i - 1]) -
                             GetConfig()
                                 .path_bounds_decider_config()
                                 .lane_change_process_config()
                                 .lane_change_heuristic_line_buffer())
                : process_bound_->GetAdcFrenetL();

      break;
    }
    double curr_lane_left_width = 0.0;
    double curr_lane_right_width = 0.0;
    // double offset_to_map = 0.0;
    // reference_line.GetOffsetToMap(curr_s, &offset_to_map);
    if (reference_line.GetLaneWidth(curr_s, &curr_lane_left_width,
                                    &curr_lane_right_width)) {
      // double offset_to_lane_center = 0.0;
      // reference_line.GetOffsetToMap(curr_s, &offset_to_lane_center);
      // curr_lane_left_width -= offset_to_lane_center;
      // curr_lane_right_width += offset_to_lane_center;
    }
    // curr_lane_left_width -= offset_to_map;
    // curr_lane_right_width += offset_to_map;

    double lane_change_prepare_line_buffer =
        GetConfig()
            .path_bounds_decider_config()
            .lane_change_process_config()
            .lane_change_prepare_line_buffer();
    double lane_change_prepare_opposite_line_buffer =
        GetConfig()
            .path_bounds_decider_config()
            .lane_change_process_config()
            .lane_change_prepare_opposite_line_buffer();
    // 1.(?) left to right, right bound; 2.(:) right to left, right bound
    std::get<1>((*path_bound)[i]) =
        process_bound_->GetAdcFrenetL() > KAlmostZero1ENegtive3
            ? curr_lane_left_width +
                  ProcessBound::GetBufferBetweenADCCenterAndEdge() +
                  lane_change_prepare_line_buffer
            : process_bound_->GetAdcFrenetL() -
                  lane_change_prepare_opposite_line_buffer;
    // 1.(?) right to left, left bound; 2.(:) left to right, left bound
    std::get<2>((*path_bound)[i]) =
        process_bound_->GetAdcFrenetL() < -KAlmostZero1ENegtive3
            ? -curr_lane_right_width -
                  ProcessBound::GetBufferBetweenADCCenterAndEdge() -
                  lane_change_prepare_line_buffer
            : process_bound_->GetAdcFrenetL() +
                  lane_change_prepare_opposite_line_buffer;
    // prepare opposite line buffer right bound calculate
    std::get<1>((*path_bound)[i]) =
        ((process_bound_->GetAdcFrenetL() < -KAlmostZero1ENegtive3) &&
         (std::get<2>((*path_bound)[i]) - std::get<1>((*path_bound)[i]) <
          GetConfig()
              .path_bounds_decider_config()
              .lane_change_process_config()
              .lane_change_prepare_opposite_line_protect_diff()))
            ? std::get<2>((*path_bound)[i]) -
                  GetConfig()
                      .path_bounds_decider_config()
                      .lane_change_process_config()
                      .lane_change_prepare_opposite_line_protect_buffer()
            : std::get<1>((*path_bound)[i]);
    // prepare opposite line buffer left bound calculate
    std::get<2>((*path_bound)[i]) =
        ((process_bound_->GetAdcFrenetL() > KAlmostZero1ENegtive3) &&
         (std::get<2>((*path_bound)[i]) - std::get<1>((*path_bound)[i]) <
          GetConfig()
              .path_bounds_decider_config()
              .lane_change_process_config()
              .lane_change_prepare_opposite_line_protect_diff()))
            ? std::get<1>((*path_bound)[i]) +
                  GetConfig()
                      .path_bounds_decider_config()
                      .lane_change_process_config()
                      .lane_change_prepare_opposite_line_protect_buffer()
            : std::get<2>((*path_bound)[i]);
    // if (i < 10) {
    //   // left to right front protect process
    //   if (process_bound_->GetAdcFrenetL() >= curr_lane_left_width) {
    //     if (process_bound_->GetAdcFrenetL() - std::get<1>((*path_bound)[i]) <
    //         5e-2) {
    //       std::get<1>((*path_bound)[i]) = process_bound_->GetAdcFrenetL() - 0.05;
    //       AERROR << "i: " << i
    //              << ", right bound: " << std::get<1>((*path_bound)[i]);
    //     }
    //   }
    //   // right to left front protect process
    //   if (process_bound_->GetAdcFrenetL() <= -curr_lane_right_width) {
    //     if (process_bound_->GetAdcFrenetL() - std::get<2>((*path_bound)[i]) >
    //         5e-2) {
    //       std::get<2>((*path_bound)[i]) = process_bound_->GetAdcFrenetL() + 0.05;
    //       AERROR << "i: " << i
    //              << ", left bound: " << std::get<2>((*path_bound)[i]);
    //     }
    //   }
    // }
  }
}

void LaneChangeBoundProcessor::GetBoundaryForLaneChangeConstraint(
    const Frame& frame, const ReferenceLineInfo& reference_line_info,
    PathBound* const path_bound,
    const std::pair<double, double>& lane_change_start_sl) {
  if (path_bound == nullptr) {
    AERROR << "lane change path bound ptr is nullptr!";
    return;
  }
  const ReferenceLine& reference_line = reference_line_info.reference_line();

  // If there is a pre-determined lane-change starting position, then use it;
  // otherwise, decide one.
  const auto* const lane_change_status = GetInjector()
                                             ->planning_context()
                                             ->mutable_planning_status()
                                             ->mutable_change_lane();

  const auto& lc_info = lane_change_status->change_lane_safety_swap_info();
  if (lane_change_status->status() != ChangeLaneStatus::IN_CHANGE_LANE &&
      !lc_info.has_lane_change_start_position()) {
    AERROR << "lane change status: " << lane_change_status->status()
           << "exist lane change start position: "
           << lc_info.has_lane_change_start_position();
    return;
  }
  double decision_lane_change_length = 0.0;
  if (lc_info.has_lane_change_start_position() &&
      lc_info.has_lane_change_end_position() &&
      std::hypot(lc_info.lane_change_end_position().x() -
                     lc_info.lane_change_start_position().x(),
                 lc_info.lane_change_end_position().y() -
                     lc_info.lane_change_start_position().y()) > 1e-5) {
    ADEBUG << "has_end position: " << lc_info.has_lane_change_end_position();
    common::SLPoint end_point_sl;
    reference_line.XYToSL(lc_info.lane_change_end_position(), &end_point_sl);

    auto* const ego_info = GetInjector()->ego_info();

    if (ego_info->GetIsLaneChangeStart()) {
      ADEBUG << "init last lane change position";
      decision_lane_change_length =
          end_point_sl.s() - lane_change_start_sl.first;
      cross_solid_line_checked_ = false;
      ego_info->SetIsLaneChangeStart(false);
    } else {
      ADEBUG << "calculate lc delta length!";
      decision_lane_change_length = lane_change_length_;
      const bool last_cur_frame_check =
          GetInjector()->frame_history() != nullptr &&
          GetInjector()->frame_history()->Latest() != nullptr &&
          GetInjector()
              ->frame_history()
              ->Latest()
              ->local_view()
              .HasFunctionManagerOut() &&
          frame.local_view().HasFunctionManagerOut();
      if (last_cur_frame_check) {
        const auto& last_fct_out = GetInjector()
                                       ->frame_history()
                                       ->Latest()
                                       ->local_view()
                                       .GetFunctionManagerOut();
        const auto& cur_fct_out = frame.local_view().GetFunctionManagerOut();
        const bool fsm_state_check =
            cur_fct_out->has_fsm_state() && last_fct_out->has_fsm_state() &&
            cur_fct_out->fsm_state() == last_fct_out->fsm_state();
        const bool hdmap_type_check =
            cur_fct_out->has_hdmap_sub_state() &&
            last_fct_out->has_hdmap_sub_state() &&
            cur_fct_out->hdmap_sub_state() == last_fct_out->hdmap_sub_state() &&
            cur_fct_out->has_localization_maptype() &&
            last_fct_out->has_localization_maptype() &&
            cur_fct_out->localization_maptype() ==
                last_fct_out->localization_maptype();
        const bool percep_type_check =
            cur_fct_out->has_perception_sub_state() &&
            last_fct_out->has_perception_sub_state() &&
            cur_fct_out->perception_sub_state() ==
                last_fct_out->perception_sub_state();
        if (fsm_state_check &&
            ((cur_fct_out->fsm_state() ==
                  functionmanager::MachineStateType::HDMAP_TYPE &&
              hdmap_type_check) ||
             (cur_fct_out->fsm_state() ==
                  functionmanager::MachineStateType::PERCEPTION_TYPE &&
              percep_type_check))) {
          ADEBUG << "lc fct state check passed!";
          common::SLPoint last_end_point_sl;
          reference_line.XYToSL(last_lane_change_end_position_,
                                &last_end_point_sl);
          const double delta_length = end_point_sl.s() - last_end_point_sl.s();
          constexpr static double AbnormalThreshod = 100;
          if (std::abs(delta_length) < AbnormalThreshod) {
            decision_lane_change_length = lane_change_length_ + delta_length;
          } else {
            AERROR << "The current lc position differ too much from "
                      "last frame, reuse last lc length!!!!";
          }
        }
      }
    }
    ADEBUG << "lc start position x: "
           << lc_info.lane_change_start_position().x()
           << ", y: " << lc_info.lane_change_start_position().y()
           << ", lc start position s: " << lane_change_start_sl.first
           << ", adc position: " << process_bound_->GetAdcFrenetS() << "\n"
           << "lc end position x: " << lc_info.lane_change_end_position().x()
           << ", y: " << lc_info.lane_change_end_position().y()
           << ", lc end position s: " << end_point_sl.s()
           << ", cur lc length: " << decision_lane_change_length
           << ", delta s lc length: "
           << decision_lane_change_length - lane_change_length_;

    lane_change_length_ = decision_lane_change_length;
    last_lane_change_end_position_ = lc_info.lane_change_end_position();
  } else {
    AERROR << "lane change gap error :no gap end position!";
  }

  // decide lane change length
  const double lane_change_distance_preview_time =
      GetConfig()
          .path_bounds_decider_config()
          .lane_change_process_config()
          .lane_change_distance_preview_time();
  const double lane_change_preview_distance = common::math::Clamp(
      std::fabs(reference_line_info.vehicle_state().linear_velocity()) *
          lane_change_distance_preview_time,
      GetConfig()
          .path_bounds_decider_config()
          .lane_change_process_config()
          .lane_change_distance_lower(),
      std::min(GetConfig()
                   .path_bounds_decider_config()
                   .lane_change_process_config()
                   .lane_change_distance_upper(),
               reference_line_info.reference_line()
                   .GetAdcMapCommonInfo()
                   .adc_distance_to_merging_end));
  const double lane_change_length =
      std::max(lane_change_preview_distance, decision_lane_change_length);

  const double lane_change_init_dl =
      process_bound_->GetAdcFrenetL() > KAlmostZero1ENegtive3
          ? std::fmin(process_bound_->GetStartPointDl(),
                      GetConfig()
                          .path_bounds_decider_config()
                          .lane_change_process_config()
                          .lane_change_heuristic_protect_dl())
          : std::fmax(process_bound_->GetStartPointDl(),
                      -GetConfig()
                           .path_bounds_decider_config()
                           .lane_change_process_config()
                           .lane_change_heuristic_protect_dl());

  if (GetConfig()
          .path_bounds_decider_config()
          .lane_change_process_config()
          .enable_heuristic_prepare_lane_change()) {
    double lane_change_prepare_length = common::math::Clamp(
        hdmap::PncMap::LookForwardDistance(
            GetConfig()
                .path_bounds_decider_config()
                .lane_change_process_config()
                .lane_change_heuristic_prepare_preview_time(),
            std::fabs(reference_line_info.vehicle_state().linear_velocity()),
            reference_line_info.vehicle_state().linear_velocity()),
        GetConfig()
            .path_bounds_decider_config()
            .lane_change_process_config()
            .lane_change_heuristic_prepare_distance_lower(),
        GetConfig()
            .path_bounds_decider_config()
            .lane_change_process_config()
            .lane_change_heuristic_prepare_distance_upper());

    lc_heuristic_generator_.Init(
        lane_change_start_sl.first, lane_change_start_sl.second,
        lane_change_init_dl, lane_change_length, lane_change_prepare_length,
        GetConfig()
            .path_bounds_decider_config()
            .lane_change_process_config()
            .lane_change_heuristic_prepare_l_limit(),
        GetConfig()
            .path_bounds_decider_config()
            .lane_change_process_config()
            .lane_change_heuristic_prepare_dl_limit(),
        GetConfig()
            .path_bounds_decider_config()
            .lane_change_process_config()
            .enable_heuristic_prepare_dl_limit());

    ADEBUG << "lane_change_init_dl: " << lane_change_init_dl
           << ", lane_change_length: " << lane_change_length
           << ", s0: " << lane_change_start_sl.first
           << ", l0: " << lane_change_start_sl.second
           << ", lane_change_prepare_length: " << lane_change_prepare_length;
  } else {
    if (!cross_solid_line_checked_) {
      lc_heuristic_generator_.Init(lane_change_start_sl.first,
                                   lane_change_start_sl.second,
                                   lane_change_init_dl, lane_change_length);
      // check if adc is crossing solid line
      is_cross_solid_line_ =
          CheckCrossSolidLine(lane_change_length, reference_line, *path_bound);
      cross_solid_line_checked_ = true;
    }

    if (is_cross_solid_line_) {
      const double min_lane_change_preview_time =
          GetConfig()
              .path_bounds_decider_config()
              .lane_change_process_config()
              .min_lane_change_preview_time();
      const double min_lane_change_length = common::math::Clamp(
          hdmap::PncMap::LookForwardDistance(
              min_lane_change_preview_time,
              std::fabs(reference_line_info.vehicle_state().linear_velocity()),
              reference_line_info.GetCruiseSpeed()),
          GetConfig()
              .path_bounds_decider_config()
              .lane_change_process_config()
              .lane_change_distance_lower(),
          std::min(GetConfig()
                       .path_bounds_decider_config()
                       .lane_change_process_config()
                       .lane_change_distance_upper(),
                   reference_line_info.reference_line()
                       .GetAdcMapCommonInfo()
                       .adc_distance_to_merging_end));
      adjusted_lane_change_length_ =
          std::max(adjusted_lane_change_length_, min_lane_change_length);
      lc_heuristic_generator_.Init(
          lane_change_start_sl.first, lane_change_start_sl.second,
          lane_change_init_dl, adjusted_lane_change_length_);

      ADEBUG << "The vehicle will cross solid line, lane "
                "change length need to be adjusted.  "
             << "adjusted lane change length: " << adjusted_lane_change_length_
             << ", min lane change length: " << min_lane_change_length;
    } else {
      lc_heuristic_generator_.Init(lane_change_start_sl.first,
                                   lane_change_start_sl.second,
                                   lane_change_init_dl, lane_change_length);
    }
  }

  for (auto& i : *path_bound) {
    double curr_s = std::get<0>(i);
    double curr_lane_left_width = 0.0;
    double curr_lane_right_width = 0.0;
    // double offset_to_map = 0.0;
    // reference_line.GetOffsetToMap(curr_s, &offset_to_map);
    if (reference_line.GetLaneWidth(curr_s, &curr_lane_left_width,
                                    &curr_lane_right_width)) {
      // double offset_to_lane_center = 0.0;
      // reference_line.GetOffsetToMap(curr_s, &offset_to_lane_center);
      // curr_lane_left_width -= offset_to_lane_center;
      // curr_lane_right_width += offset_to_lane_center;
    }
    // curr_lane_left_width -= offset_to_map;
    // curr_lane_right_width += offset_to_map;

    if (GetConfig()
            .path_bounds_decider_config()
            .lane_change_process_config()
            .use_heuristic_lane_change_path() &&
        lc_heuristic_generator_.InitSuccess()) {
      double heuristic_l = lc_heuristic_generator_.GetLWithS(std::get<0>(i));
      // left to right lane change
      // left bound
      std::get<2>(i) =
          (DefinitelyGreater(process_bound_->GetAdcFrenetL(),
                             process_bound_->GetAdcLaneLeftWidth()))
              ? heuristic_l + GetConfig()
                                  .path_bounds_decider_config()
                                  .lane_change_process_config()
                                  .lane_change_heuristic_opposite_line_buffer()
              : std::get<2>(i);
      // right bound
      std::get<1>(i) =
          (DefinitelyGreater(process_bound_->GetAdcFrenetL(),
                             process_bound_->GetAdcLaneLeftWidth()))
              ? heuristic_l - GetConfig()
                                  .path_bounds_decider_config()
                                  .lane_change_process_config()
                                  .lane_change_heuristic_line_buffer()
              : std::get<1>(i);
      // right to left lane change
      // right bound
      std::get<1>(i) =
          (DefinitelyLess(process_bound_->GetAdcFrenetL(),
                          -process_bound_->GetAdcLaneRightWidth()))
              ? heuristic_l - GetConfig()
                                  .path_bounds_decider_config()
                                  .lane_change_process_config()
                                  .lane_change_heuristic_opposite_line_buffer()
              : std::get<1>(i);
      // left bound
      std::get<2>(i) =
          (DefinitelyLess(process_bound_->GetAdcFrenetL(),
                          -process_bound_->GetAdcLaneRightWidth()))
              ? heuristic_l + GetConfig()
                                  .path_bounds_decider_config()
                                  .lane_change_process_config()
                                  .lane_change_heuristic_line_buffer()
              : std::get<2>(i);
    } else {
      double lanechange_t = FLAGS_path_lane_change_distance_preview_time;
      lanechange_t =
          lanechange_t +
          (process_bound_->GetAdcFrenetL() > curr_lane_left_width
               ? (process_bound_->GetAdcFrenetL() - curr_lane_left_width) *
                     GetConfig()
                         .path_bounds_decider_config()
                         .lane_change_process_config()
                         .lane_change_distance_preview_extend_time_ratio()
               : 0);
      lanechange_t =
          lanechange_t +
          (process_bound_->GetAdcFrenetL() < -curr_lane_right_width
               ? std::fabs(process_bound_->GetAdcFrenetL() +
                           curr_lane_right_width) *
                     GetConfig()
                         .path_bounds_decider_config()
                         .lane_change_process_config()
                         .lane_change_distance_preview_extend_time_ratio()
               : 0);

      const double normal_lane_change_length = common::math::Clamp(
          hdmap::PncMap::LookForwardDistance(
              lanechange_t,
              std::fabs(reference_line_info.vehicle_state().linear_velocity()),
              reference_line_info.GetCruiseSpeed()),
          GetConfig()
              .path_bounds_decider_config()
              .lane_change_process_config()
              .lane_change_distance_lower(),
          GetConfig()
              .path_bounds_decider_config()
              .lane_change_process_config()
              .lane_change_distance_upper());

      if (curr_s >= lane_change_start_sl.first + normal_lane_change_length) {
        // right bound
        std::get<1>(i) =
            (process_bound_->GetAdcFrenetL() > curr_lane_left_width) ||
                    (process_bound_->GetAdcFrenetL() >= -curr_lane_right_width)
                ? std::get<1>(i)
                : -curr_lane_right_width +
                      ProcessBound::GetBufferBetweenADCCenterAndEdge();
        // left bound
        std::get<2>(i) =
            (process_bound_->GetAdcFrenetL() < -curr_lane_right_width) ||
                    (process_bound_->GetAdcFrenetL() <= curr_lane_left_width)
                ? std::get<2>(i)
                : curr_lane_left_width -
                      ProcessBound::GetBufferBetweenADCCenterAndEdge();
      }
    }
  }
}

bool LaneChangeBoundProcessor::CheckCrossSolidLine(
    const double lane_change_length, const ReferenceLine& reference_line,
    const PathBound& path_bound) {
  const bool left_to_right = DefinitelyGreater(
      process_bound_->GetAdcFrenetL(), process_bound_->GetAdcLaneLeftWidth());
  const bool right_to_left = DefinitelyLess(
      process_bound_->GetAdcFrenetL(), -process_bound_->GetAdcLaneRightWidth());
  // lane change end position s
  const double check_lane_type_max_s =
      lane_change_length + process_bound_->GetAdcFrenetS();

  for (const auto& i : path_bound) {
    const double curr_s = std::get<0>(i);
    // check only when curr_s is less than lane change end position
    if (curr_s <= check_lane_type_max_s) {
      const double heuristic_l = lc_heuristic_generator_.GetLWithS(curr_s);
      const double solid_forward_s =
          std::min(curr_s + process_bound_->GetLaneBoundConf()
                                .solid_line_forward_extend_length(),
                   reference_line.Length());
      const auto& curr_waypoint =
          reference_line.GetNearestReferencePoint(curr_s)
              .lane_waypoints()
              .front();
      const double solid_line_check_buffer =
          GetConfig()
              .path_bounds_decider_config()
              .lane_change_process_config()
              .lane_change_solid_line_check_buffer();

      if (left_to_right) {
        LaneBoundaryType::Type cur_left_lane_type =
            hdmap::LeftBoundaryType(curr_waypoint);
        const auto left_forward_lane_type = hdmap::LeftBoundaryType(
            reference_line.GetNearestReferencePoint(solid_forward_s)
                .lane_waypoints()
                .front());
        const bool is_left_solid =
            ((cur_left_lane_type != LaneBoundaryType::DOTTED_YELLOW &&
              cur_left_lane_type != LaneBoundaryType::DOTTED_WHITE) ||
             (left_forward_lane_type != LaneBoundaryType::DOTTED_YELLOW &&
              left_forward_lane_type != LaneBoundaryType::DOTTED_WHITE));

        // check only when the lane line on the side close to adc is solid
        if (is_left_solid) {
          // if the lane type of s too close to adc is solid,
          // reducing the length of lane change will only accelerate adc to cross line.
          if (curr_s >= GetConfig()
                                .path_bounds_decider_config()
                                .lane_change_process_config()
                                .lane_change_distance_lower() +
                            process_bound_->GetAdcFrenetS() &&
              process_bound_->GetAdcLaneLeftWidth() - heuristic_l <
                  solid_line_check_buffer +
                      ProcessBound::GetBufferBetweenADCCenterAndEdge()) {
            const double adjusted_heuristic_l =
                process_bound_->GetAdcLaneLeftWidth() -
                solid_line_check_buffer -
                ProcessBound::GetBufferBetweenADCCenterAndEdge();
            adjusted_lane_change_length_ =
                fabs(process_bound_->GetAdcFrenetL()) /
                fabs(adjusted_heuristic_l - process_bound_->GetAdcFrenetL()) *
                fabs(curr_s - process_bound_->GetAdcFrenetS());
            return true;
          }
          break;
        }
      } else if (right_to_left) {
        LaneBoundaryType::Type cur_right_lane_type =
            hdmap::RightBoundaryType(curr_waypoint);
        const auto right_forward_lane_type = hdmap::RightBoundaryType(
            reference_line.GetNearestReferencePoint(solid_forward_s)
                .lane_waypoints()
                .front());
        const bool is_right_solid =
            ((cur_right_lane_type != LaneBoundaryType::DOTTED_YELLOW &&
              cur_right_lane_type != LaneBoundaryType::DOTTED_WHITE) ||
             (right_forward_lane_type != LaneBoundaryType::DOTTED_YELLOW &&
              right_forward_lane_type != LaneBoundaryType::DOTTED_WHITE));

        // check only when the lane line on the side close to adc is solid
        if (is_right_solid) {
          if (curr_s >= GetConfig()
                                .path_bounds_decider_config()
                                .lane_change_process_config()
                                .lane_change_distance_lower() +
                            process_bound_->GetAdcFrenetS() &&
              process_bound_->GetAdcLaneRightWidth() + heuristic_l <
                  solid_line_check_buffer +
                      ProcessBound::GetBufferBetweenADCCenterAndEdge()) {
            const double adjusted_heuristic_l =
                -process_bound_->GetAdcLaneRightWidth() +
                solid_line_check_buffer +
                ProcessBound::GetBufferBetweenADCCenterAndEdge();
            adjusted_lane_change_length_ =
                fabs(process_bound_->GetAdcFrenetL()) /
                fabs(adjusted_heuristic_l - process_bound_->GetAdcFrenetL()) *
                fabs(curr_s - process_bound_->GetAdcFrenetS());
            return true;
          }
          break;
        }
      }
    } else {
      break;
    }
  }
  return false;
}

void LaneChangeBoundProcessor::GetBoundaryFromLaneBoundaryType(
    const ReferenceLineInfo& reference_line_info,
    PathBound* const path_boundaries) {
  if (path_boundaries == nullptr || process_bound_->GetFrame() == nullptr) {
    AERROR << "path_boundaries or process_bound_->GetFrame()  is nullptr.";
    return;
  }
  if (process_bound_->GetFrame()->reference_line_info().size() < 2 ||
      path_boundaries->empty()) {
    ADEBUG << "BoundaryFromLaneBoundaryType return";
    return;
  }
  const ReferenceLine& reference_line = reference_line_info.reference_line();
  const auto& lane_change_status =
      GetInjector()->planning_context()->planning_status().change_lane();
  double lane_change_start_s = 0.0;
  if (lane_change_status.exist_lane_change_start_position()) {
    common::SLPoint point_sl;
    reference_line.XYToSL(lane_change_status.lane_change_start_position(),
                          &point_sl);
    lane_change_start_s = point_sl.s();
  }

  // PATH Evaluate lane change distance

  const double lanechange_t = FLAGS_path_lane_change_distance_preview_time;
  double lane_change_distance = hdmap::PncMap::LookForwardDistance(
      lanechange_t,
      std::fabs(reference_line_info.vehicle_state().linear_velocity()),
      reference_line_info.GetCruiseSpeed());
  lane_change_distance = common::math::Clamp(lane_change_distance,
                                             GetConfig()
                                                 .path_bounds_decider_config()
                                                 .lane_change_process_config()
                                                 .lane_change_distance_lower(),
                                             GetConfig()
                                                 .path_bounds_decider_config()
                                                 .lane_change_process_config()
                                                 .lane_change_distance_upper());

  double finish_lane_change_s =
      std::fmin(lane_change_distance + process_bound_->GetAdcFrenetS(),
                reference_line.Length());
  ADEBUG << "lane change start s: " << lane_change_start_s;
  if (lane_change_start_s > finish_lane_change_s) {
    return;
  }

  routing::ChangeLaneType lane_change_type =
      ProcessBound::JudgeLaneChangeType(process_bound_->GetAdcFrenetL());
  size_t forbid_lane_change_index = std::numeric_limits<size_t>::max();
  for (size_t index = 0; index < path_boundaries->size(); ++index) {
    double curr_s = std::get<0>((*path_boundaries)[index]);
    if (curr_s > finish_lane_change_s &&
        std::isgreaterequal(forbid_lane_change_index,
                            std::numeric_limits<size_t>::max())) {
      break;
    }
    if (!ProcessBound::CheckLaneBoundaryType(reference_line_info, curr_s,
                                             lane_change_type)) {
      forbid_lane_change_index = index;
    }
  }
  ADEBUG << "forbid_to_change_lane_index:" << forbid_lane_change_index;

  if (std::isgreaterequal(forbid_lane_change_index,
                          std::numeric_limits<size_t>::max())) {
    return;
  }

  static constexpr double num_percentage = 0.80;
  static constexpr size_t extend_cnt = 10;
  double percentage = static_cast<double>(forbid_lane_change_index) /
                      static_cast<double>(path_boundaries->size());
  if (percentage > num_percentage) {
    forbid_lane_change_index = path_boundaries->size();
  } else if (percentage < 1 - num_percentage) {
    forbid_lane_change_index = static_cast<size_t>(
        fmin(path_boundaries->size(), forbid_lane_change_index + extend_cnt));
  }

  for (size_t i = 0; i < path_boundaries->size(); ++i) {
    if (i > forbid_lane_change_index) {
      break;
    }
    double curr_s = std::get<0>((*path_boundaries)[i]);
    // Remove the target lane out of the path-boundary, up to the decided S.
    double curr_lane_left_width = 0.0;
    double curr_lane_right_width = 0.0;
    // double offset_to_map = 0.0;
    // reference_line.GetOffsetToMap(curr_s, &offset_to_map);
    if (reference_line.GetLaneWidth(curr_s, &curr_lane_left_width,
                                    &curr_lane_right_width)) {
      // double offset_to_lane_center = 0.0;
      // reference_line.GetOffsetToMap(curr_s, &offset_to_lane_center);
      // curr_lane_left_width -= offset_to_lane_center;
      // curr_lane_right_width += offset_to_lane_center;
    }
    // curr_lane_left_width -= offset_to_map;
    // curr_lane_right_width += offset_to_map;
    static constexpr double kBoundaryBuf = 0.50;
    static constexpr double kMinBoundarySpace = 0.05;
    std::get<1>((*path_boundaries)[i]) =
        process_bound_->GetAdcFrenetL() > curr_lane_left_width
            ? std::fmin(curr_lane_left_width +
                            ProcessBound::GetBufferBetweenADCCenterAndEdge() +
                            kBoundaryBuf,
                        std::get<2>((*path_boundaries)[i]) - kMinBoundarySpace)
            : std::get<1>((*path_boundaries)[i]);
    std::get<2>((*path_boundaries)[i]) =
        process_bound_->GetAdcFrenetL() < -curr_lane_right_width
            ? std::fmax(-curr_lane_right_width -
                            ProcessBound::GetBufferBetweenADCCenterAndEdge() -
                            kBoundaryBuf,
                        std::get<1>((*path_boundaries)[i]) + kMinBoundarySpace)
            : std::get<2>((*path_boundaries)[i]);
  }
  ProcessPathboundFromStartPoint(path_boundaries);
}

void LaneChangeBoundProcessor::ProcessPathboundFromStartPoint(
    PathBound* const path_boundaries) {
  if (!GetConfig()
           .path_bounds_decider_config()
           .is_extend_lane_bounds_to_include_adc()) {
    return;
  }
  if (path_boundaries == nullptr || path_boundaries->empty()) {
    AERROR << "path_boundaries is nullptr or empty.";
    return;
  }
  double start_path_bound_l_min = 0.0;
  double start_path_bound_l_max = 0.0;
  double start_path_bound_s = 0.0;
  std::tie(start_path_bound_s, start_path_bound_l_min, start_path_bound_l_max) =
      path_boundaries->at(0);
  if (Compare(start_path_bound_l_max, start_path_bound_l_min) < 1) {
    AERROR
        << "err: start_path_bound_l_max is less than start_path_bound_l_min!";
  }
  bool is_out_right =
      Compare(process_bound_->GetAdcFrenetL(), start_path_bound_l_min) < 0;
  bool is_out_left =
      Compare(process_bound_->GetAdcFrenetL(), start_path_bound_l_max) > 0;
  if (!is_out_left && !is_out_right) {
    ADEBUG << "GetAdcFrenetL() is not out of boundary.";
    return;
  }

  std::get<2>(path_boundaries->at(0)) =
      std::fmax(process_bound_->GetAdcFrenetL(), start_path_bound_l_max);
  std::get<1>(path_boundaries->at(0)) =
      std::fmin(process_bound_->GetAdcFrenetL(), start_path_bound_l_min);

  double path_bound_l_min = 0.0;
  double path_bound_l_max = 0.0;
  double path_bound_s = 0.0;
  static constexpr double kPathBoundBuf = 0.0;
  for (std ::size_t idx = 1; idx < path_boundaries->size(); ++idx) {
    std::tie(path_bound_s, path_bound_l_min, path_bound_l_max) =
        path_boundaries->at(idx);
    double delta_s =
        std::get<0>(path_boundaries->at(idx)) - process_bound_->GetAdcFrenetS();
    if (ComparedToZero(delta_s) < 1) {
      AERROR << "path bound s is equal.";
      return;
    }
    if (Compare(delta_s, FLAGS_constraint_length_from_start_point) > 0) {
      if (is_out_left) {
        std::get<2>(path_boundaries->at(idx)) =
            path_bound_l_max - kPathBoundBuf;
        std::get<1>(path_boundaries->at(idx)) =
            path_bound_l_min - kPathBoundBuf;
      }
      if (is_out_right) {
        std::get<2>(path_boundaries->at(idx)) =
            path_bound_l_max + kPathBoundBuf;
        std::get<1>(path_boundaries->at(idx)) =
            path_bound_l_min + kPathBoundBuf;
      }
      return;
    }
    const double new_path_bound_l = delta_s * process_bound_->GetStartPointDl();
    std::get<2>(path_boundaries->at(idx)) =
        std::fmax(path_bound_l_max, start_path_bound_l_max + new_path_bound_l);
    std::get<1>(path_boundaries->at(idx)) =
        std::fmin(path_bound_l_min, start_path_bound_l_min + new_path_bound_l);
  }
}
}  // namespace planning
}  // namespace TL
