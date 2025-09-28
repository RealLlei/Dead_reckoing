/******************************************************************************
 * Copyright (c) TL Technologies Co., Ltd. 2019-2022. All rights reserved.
 * Author: LingPeng
 * Created Time: 2022/4/21
 *****************************************************************************/

#include "planning/localview/lane_line_builder/obstacle_following_lane_line/lane_line_list.h"

#include <tuple>
#include <utility>
#include <vector>

#include "common/math/vec2d.h"

namespace TL {
namespace planning {
namespace nolane {

using TL::common::math::Vec2d;

bool LaneLineList::SortLaneLines(
    std::shared_ptr<LaneCenterLine>* const optimal_lane_ptr) {
  LogProcess::LogProc(__func__);
  if (lane_line_drivable_.empty()) {
    AERROR << "lane_line_drivable is empty.";
    return false;
  } else if (lane_line_drivable_.size() == 1) {
    ADEBUG << "lane_line_drivable_ only one lane.";
    lane_line_ptr_previous_ = lane_line_drivable_.front();
    lane_line_ptr_previous_->ExtendLaneLength();
    *optimal_lane_ptr = lane_line_ptr_previous_;
  } else {
    ACHECK(false) << "3 targets.";
    // lp: TODO choose one lane center line to follow.
  }
  return true;
}

const std::shared_ptr<LaneCenterLine>& LaneLineList::GetLaneLinePtrPrevious()
    const {
  return lane_line_ptr_previous_;
}

bool LaneLineList::ConstructLaneBetweenTrajectoryAndEgo(
    const StitchPointInfo& obstacle_followed_index,
    const EgoVehicleState& ego_state, const bool is_based_one_traj) {
  LogProcess::LogProc(__func__);
  lane_line_drivable_.clear();
  auto obs_ptr = obstacle_followed_index.obstacle_ptr;
  auto& state_v = obs_ptr->GetStateHistoryExtended();
  std::vector<Vec2d> ego_lane_points;
  if (is_based_one_traj) {
    // lp: construct lane line based on one obstacle's trajectory without
    // connected curve.
    ego_lane_points.reserve(state_v.size());
    // for (auto& pos_sl : state_v) {
    //   ego_lane_points.emplace_back(pos_sl.position.x, pos_sl.position.y);
    // }
    double delta_x = 0.0;
    double delta_y = 0.0;
    for (int i = obstacle_followed_index.index; i < state_v.size(); ++i) {
      delta_x = GetExtendStateIndex(state_v.at(i)).x;
      // state_v.at(i).position_fit.x - ego_state.GetVehicleStatePresent().x();
      delta_y = GetExtendStateIndex(state_v.at(i)).y;
      ego_lane_points.emplace_back(delta_x, delta_y);
    }
    lane_line_drivable_.emplace_back(std::make_shared<LaneCenterLine>(
        std::move(ego_lane_points), ego_state));
  } else {
    // lp: construct lane line based on one obstacle's trajectory with
    // connected curve.
    lane_line_drivable_.emplace_back(
        std::make_shared<LaneCenterLine>(obstacle_followed_index, ego_state));
  }
  ConstructLaneForEachInDrivableList(false);
  std::shared_ptr<LaneCenterLine> lane_ptr;
  if (!lane_line_drivable_.empty()) {
    lane_ptr = lane_line_drivable_.front();
  } else {
    AERROR << "lane_line_drivable is empty.";
    return false;
  }
  if (!lane_ptr) {
    AERROR << "lane_ptr is nullptr.";
    return false;
  }
  lane_ptr->ExtendLaneLength();
  if (debug_flag[9]) {
    AERROR << "construct lane done.";
  }
  lane_line_ptr_previous_ = lane_ptr;
  return true;
}

bool LaneLineList::ConstructLaneLines(const obstacles_ptr_list& obstacles,
                                      const EgoVehicleState& ego_state) {
  LogProcess::LogProc(__func__);
  lane_line_drivable_.clear();
  std::vector<std::pair<common::SLPoint, Vec2d>> ego_lane_points;
  std::vector<Vec2d> left_lane_points;
  std::vector<Vec2d> right_lane_points;
  obstacles_ptr_list obs_qualified;

  for (auto& obs_ptr : obstacles) {
    if (obs_ptr->GetMovingBehavious() ==
            ObstaclePerceptionNoLane::MovingBehavior::Cruise &&
        (obs_ptr->GetLaneType() ==
             ObstaclePerceptionNoLane::LaneType::EgoLane ||
         obs_ptr->GetLaneType() ==
             ObstaclePerceptionNoLane::LaneType::TooFarUnDefined) &&
        obs_ptr->GetStateSl().size() > 3) {
      obs_qualified.push_back(obs_ptr);
    }
  }
  if (obs_qualified.size() > 1) {
    FilterObstacleFromMultiTrajectory(&obs_qualified);
  }

  for (auto& obs_ptr : obs_qualified) {
    if (true) {
      if (debug_flag[19]) {
        AERROR << "obs_ptr_id_perc_unique:[" << obs_ptr->GetIdPerception()
               << "," << obs_ptr->GetIdUnique() << "]";
        for (auto& pos_sl : obs_ptr->GetStateSl()) {
          AERROR << PRECISION(3) << "input_pos_S:" << std::get<0>(pos_sl).s()
                 << "  input_pos_L:" << std::get<0>(pos_sl).l()
                 << "  lane_type:" << static_cast<int>(std::get<1>(pos_sl));
          if (std::get<0>(pos_sl).s() > 3000) {
            ACHECK(false);
            return false;
          }
        }
        AERROR << "";
      }
      if (ego_lane_points.capacity() <
          obs_ptr->GetStateSl().size() + ego_lane_points.size()) {
        ego_lane_points.reserve(obs_ptr->GetStateSl().size() +
                                ego_lane_points.size() + 50);
      }
      common::SLPoint proj_sl;
      int counter_s = 0;
      auto& pos_xy_ext = obs_ptr->GetStateHistoryExtended();
      for (auto& pos_sl : obs_ptr->GetStateSl()) {
        proj_sl.set_s(std::get<0>(pos_sl).s());
        proj_sl.set_l(std::get<0>(pos_sl).l());
        ego_lane_points.push_back(
            {proj_sl,
             {GetExtendStateIndex(pos_xy_ext.at(counter_s)).x,
              GetExtendStateIndex(pos_xy_ext.at(counter_s)).y}});
        ++counter_s;
      }
    } else if (false) {
      // lp: extract ego lane points when obs cut in from other lane or cut out
      // from ego lane.
      // lp: attention : how to handle cross road
      for (const std::tuple<SLPoint, ObstaclePerceptionNoLane::LaneType>&
               pos_sl : obs_ptr->GetStateSl()) {
        if (std::get<1>(pos_sl) ==
            ObstaclePerceptionNoLane::LaneType::EgoLane) {
          // ego_lane_points.emplace_back(std::get<0>(pos_sl).s(),
          //                              std::get<0>(pos_sl).l());
        }
      }
    }
  }
  const double minimal_drive_time = 0.5;
  double point_length = 0.0;
  double max_s = 0.0;
  double min_s = 0.0;
  if (!ego_lane_points.empty()) {
    auto minmax_p = std::minmax_element(
        ego_lane_points.cbegin(), ego_lane_points.cend(),
        [](auto& lhs, auto& rhs) {
          return DefinitelyLess(lhs.first.s(), rhs.first.s());
        });
    point_length = minmax_p.second->first.s() - minmax_p.first->first.s();
    max_s = minmax_p.second->first.s();
    min_s = minmax_p.first->first.s();
  }

  if (!ego_lane_points.empty() &&
      point_length > ego_state.GetVehicleStatePresent().linear_velocity() *
                         minimal_drive_time &&
      DefinitelyGreater(max_s, ego_state.GetVehicleSl().s())) {
    lane_line_drivable_.emplace_back(std::make_shared<LaneCenterLine>(
        std::move(ego_lane_points), ego_state, lane_line_ptr_previous_));
  } else {
    AERROR << "ego lane size:" << ego_lane_points.size()
           << "  ego_s:" << ego_state.GetVehicleSl().s() << "  max_s:" << max_s
           << "  min_s:" << min_s << "  ego_lane_length:" << point_length
           << "  speed:" << ego_state.GetVehicleStatePresent().linear_velocity()
           << "(m/s)  drive_time:" << minimal_drive_time << "(s).";
    return false;
  }
  // lp: TODO cut_left lane and cut_right lane
  ConstructLaneForEachInDrivableList(true);
  return true;
}

void LaneLineList::ConstructLaneForEachInDrivableList(bool based_on_traj_line) {
  LogProcess::LogProc(__func__);
  std::for_each(lane_line_drivable_.begin(), lane_line_drivable_.end(),
                [&](const std::shared_ptr<LaneCenterLine>& lane_line) {
                  lane_line->ConstructLaneLine(based_on_traj_line);
                });
  if (debug_flag[40]) {
    for (const auto& lane : lane_line_drivable_) {
      AERROR << "lane id:" << lane->GetId()
             << "  lane_is_success:" << lane->IsConstructLaneSuccess();
    }
  }
  lane_line_drivable_.erase(
      std::remove_if(lane_line_drivable_.begin(), lane_line_drivable_.end(),
                     [](const std::shared_ptr<LaneCenterLine>& lane_line) {
                       return !lane_line->IsConstructLaneSuccess();
                     }),
      lane_line_drivable_.end());
  double current_time = TL::common::Clock::NowInSeconds();
  std::for_each(lane_line_drivable_.begin(), lane_line_drivable_.end(),
                [current_time](auto& lane_line) {
                  lane_line->SetTimeUpdateLatest(current_time);
                });
  if (debug_flag[40]) {
    AERROR << "lane_line_drivable_size:" << lane_line_drivable_.size();
  }
}

void LaneLineList::Clear() {
  lane_line_ptr_output_ = nullptr;
}

const std::shared_ptr<LaneCenterLine>& LaneLineList::GetLaneLinePtrOutput()
    const {
  return lane_line_ptr_output_;
}

void LaneLineList::SetLaneLinePtrPrevious(
    const std::shared_ptr<LaneCenterLine>& laneLinePtrPrevious) {
  lane_line_ptr_previous_ = laneLinePtrPrevious;
}

bool LaneLineList::ValidationCheckLaneLinePrevious(
    EgoVehicleState* const ego_state) {
  LogProcess::LogProc(__func__);
  ego_state->ProjectionEgoSL(lane_line_ptr_previous_->GetReferenceLinePtr());
  if (fabs(ego_state->GetVehicleSl().l()) > FLAGS_nolane_ego_traj_max_l) {
    if (debug_flag[46]) {
      AERROR << "the ego-l value when ego project trajectory is too big. sl:["
             << ego_state->GetVehicleSl().s() << ","
             << ego_state->GetVehicleSl().l()
             << "]  previous_lane:" << GetPtr(lane_line_ptr_previous_)
             << "  previous_ref_ptr:"
             << GetPtr(lane_line_ptr_previous_->GetReferenceLinePtr())
             << " will reset nullptr.";
    }
    SetLaneLinePtrPrevious(nullptr);
    return false;
  }
  double time_current = TL::common::Clock::NowInSeconds();
  double time_max_diff = 3;
  if (time_current - lane_line_ptr_previous_->GetTimeUpdateLatest() >
      time_max_diff) {
    if (debug_flag[46]) {
      AERROR << "time diff is too big. reset lane line.time_diff(s):"
             << time_current - lane_line_ptr_previous_->GetTimeUpdateLatest()
             << " current_time:" << time_current << "  previous_lane_time:"
             << lane_line_ptr_previous_->GetTimeUpdateLatest()
             << "  prev_lane_ptr:" << GetPtr(lane_line_ptr_previous_)
             << "  prev_ref_line:"
             << GetPtr(lane_line_ptr_previous_->GetReferenceLinePtr());
    }
    SetLaneLinePtrPrevious(nullptr);
    return false;
  }
  return true;
}

void LaneLineList::SetLaneLinePtrOutput(
    const std::shared_ptr<LaneCenterLine>& outputLaneLinePtr) {
  return;
}
}  // namespace nolane
}  // namespace planning
}  // namespace TL
