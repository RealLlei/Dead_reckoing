/******************************************************************************
 * Copyright (c) TL Technologies Co., Ltd. 2019-2022. All rights reserved.
 * Author: LingPeng
 * Created Time: 2022/4/21
 *****************************************************************************/

#include "planning/localview/lane_line_builder/obstacle_following_lane_line/no_lane_line_kernel.h"

#include <string>

namespace TL {
namespace planning {
namespace nolane {

NoLaneLineKernel::NoLaneLineKernel() : mode_(Mode::NormalObsWithoutLane) {}

void NoLaneLineKernel::StartRecord() {
  double header_time = TL::common::Clock::NowInSeconds();
  if (debug_flag[4]) {
    AERROR << "-----------------no_lane_period------------\n";
    AERROR << PRECISION(3) << "nolane_time_stamp:" << header_time
           << " no_lane_counter:" << counter_++;
  }
  auto counter_cyber = ptr_without_lane->mutable_debug()->add_temp_value();
  counter_cyber->set_name("header_time_counter");
  counter_cyber->set_d1(header_time);
  counter_cyber->set_i1(counter_);
}

bool NoLaneLineKernel::Process(const std::shared_ptr<LocalView>& local_view) {
  NoLaneToCyber no_lane_to_cyber(local_view);
  local_view_ = local_view;
  StartRecord();

  const TL::common::VehicleState& vehicle_state =
      *local_view->GetVehicleState();
  const TL::perception::PerceptionObstacles& perception_obstacles =
      *local_view->GetPerceptionObstacles();
  coordinate_system_convert_.Process(vehicle_state);
  ego_state_.UpdateEgoState(vehicle_state);

  if (!perception_obstacles.perception_obstacle_size() &&
      !IsSufficientWithObsesPrev()) {
    ADEBUG << "there is no obstacle.";
    return false;
  }
  SendToCyber(perception_obstacles);

  if (!obstacles_perception_manager_.UpdateObstaclePresent(
          local_view->GetPerceptionObstacles(), coordinate_system_convert_,
          ego_state_)) {
    ADEBUG << "update obstacle state failed.";
    return false;
  }

  obstacles_perception_manager_.MergeObstacles(&obstacles_merged_);
  obstacles_perception_manager_.DeleteUnqualifiedObstacle(&obstacles_merged_);
  obstacles_perception_manager_.ExtendObstacleTrajectory(&obstacles_merged_);
  obstacles_perception_manager_.TrajectoryFitForEachObstacle(
      &obstacles_merged_);

  // lp: generate lane line base on obstacles front.
  auto previous_lane = lane_line_list_.GetLaneLinePtrPrevious();
  auto previous_lane_raw = previous_lane;
  if (previous_lane && previous_lane->GetReferenceLinePtr()) {
    if (lane_line_list_.ValidationCheckLaneLinePrevious(&ego_state_)) {
      obstacles_perception_manager_.EgoIsCloseReferenceLine(
          ego_state_, previous_lane->GetStartS());
    } else {
      previous_lane = lane_line_list_.GetLaneLinePtrPrevious();
    }
  }
  if (debug_flag[41]) {
    std::string prev_ref =
        previous_lane_raw ? GetPtr(previous_lane_raw->GetReferenceLinePtr())
                          : "nullptr";
    std::string is_connect =
        previous_lane_raw
            ? std::to_string(previous_lane_raw->IsCurveConnectEgoTrajectory())
            : "0";
    AERROR << "has previous_lane_raw:" << GetPtr(previous_lane_raw)
           << "  prev_ref_line_ptr:" << prev_ref
           << "  prev_lane:" << GetPtr(lane_line_list_.GetLaneLinePtrPrevious())
           << "  is_connect_ego_obs:" << is_connect << " is_close_enough:"
           << obstacles_perception_manager_.IsCloseEnough() << PRECISION(3)
           << "  ego_sl:[" << ego_state_.GetVehicleSl().s() << ","
           << ego_state_.GetVehicleSl().l() << "].";
  }
  if (!previous_lane || !obstacles_perception_manager_.IsCloseEnough()) {
    // lp: generate lane when function activated.
    if (obstacles_perception_manager_.IsEgoInInAllObstacleTrajectory(
            obstacles_merged_, ego_state_, perception_obstacles) ||
        previous_lane) {
      // lp: choose one trajectory in cross-trajectories to follow.
      const std::vector<int>& v_index =
          obstacles_perception_manager_.ChooseObstaclesToFollowed();
      if (cyber_flag[2]) {
        auto ptr_v = ptr_without_lane->mutable_debug()->add_temp_value();
        ptr_v->set_name("obs_followed_index");
        for (auto val : v_index) {
          ptr_v->add_ri(val);
        }
      }
      const auto& obstacle_followed =
          obstacles_perception_manager_.DecideFollowType(v_index, ego_state_);
      if (previous_lane) {
        if (previous_lane->EgoIsStillInPreviousLane(ego_state_)) {
          if (!obstacles_perception_manager_.ProjectTrajectory(
                  lane_line_list_.GetLaneLinePtrPrevious(), &obstacles_merged_,
                  &ego_state_)) {
            return false;
          }
          obstacles_perception_manager_.DeleteObsBackEgoAndTrimTraj(
              lane_line_list_.GetLaneLinePtrPrevious(), ego_state_,
              &obstacles_merged_);
          return CreatMapOut(previous_lane);
        } else {
          AERROR << "ego is not in previous lane, "
                 << "preparing construct a new lane.";
        }
      }

      if (cyber_flag[3]) {
        // lp: debug to cyber
        auto ptr = ptr_without_lane->mutable_debug()->add_temp_value();
        if (obstacle_followed.obstacle_ptr) {
          ptr->set_name("followed_obs.");
          ptr->set_i1(obstacle_followed.obstacle_ptr->GetIdUnique());
          ptr->set_i2(obstacle_followed.index);
          ptr->set_d1(obstacle_followed.cost);
        } else {
          ptr->set_name("no followed obs.");
        }
      }
      if (debug_flag[44]) {
        if (obstacle_followed.obstacle_ptr) {
          AERROR << "followed_obstacle_id_perception:"
                 << obstacle_followed.obstacle_ptr->GetIdPerception()
                 << "  index:" << obstacle_followed.index
                 << "  cost:" << obstacle_followed.cost;
        } else {
          AERROR << "there is no obstacle to be followed.";
        }
      }
      if (obstacle_followed.obstacle_ptr &&
          lane_line_list_.ConstructLaneBetweenTrajectoryAndEgo(
              obstacle_followed, ego_state_,
              obstacles_perception_manager_.IsCloseEnough())) {
        return CreatMapOut(lane_line_list_.GetLaneLinePtrPrevious());
      } else {
        AERROR << "No obstacle can be followed.";
        return false;
      }
    } else {
      ADEBUG << "Obstacles is too far,"
                "can't generate lane line base on stored obstacles.";
      return false;
    }
  }
  if (!obstacles_perception_manager_.ProjectTrajectory(
          lane_line_list_.GetLaneLinePtrPrevious(), &obstacles_merged_,
          &ego_state_)) {
    return false;
  }

  obstacles_perception_manager_.DeleteObsBackEgoAndTrimTraj(
      lane_line_list_.GetLaneLinePtrPrevious(), ego_state_, &obstacles_merged_);

  if (!lane_line_list_.ConstructLaneLines(obstacles_merged_, ego_state_)) {
    return false;
  }

  std::shared_ptr<LaneCenterLine> output_lane_ptr;
  if (!lane_line_list_.SortLaneLines(&output_lane_ptr)) {
    return false;
  }

  return CreatMapOut(output_lane_ptr);
}

bool NoLaneLineKernel::CreatMapOut(
    const std::shared_ptr<LaneCenterLine>& output_lane_ptr) {
  LogProcess::LogProc(__func__);
  if (!output_lane_ptr) {
    AERROR << "output_lane_ptr is empty.";
    return false;
  }
  if (output_lane_ptr->CreatNoLaneMapAndRouting() &&
      output_lane_ptr->GetMapMsgPtr()->hdmap().lane_size() > 0) {
    current_map_msg_ = output_lane_ptr->GetMapMsgPtr();
    current_routing_response_ = std::make_shared<routing::RoutingResponse>(
        output_lane_ptr->GetMapMsgPtr()->routing());
    if (debug_flag[25]) {
      AERROR << "Creat no_lane map success!!!";
    }
    return true;
  } else {
    AERROR << "Creat no_lane map failed!!!";
    return false;
  }
}

const obstacles_ptr_list& NoLaneLineKernel::GetObstaclesMerged() const {
  return obstacles_merged_;
}

const EgoVehicleState& NoLaneLineKernel::GetEgoState() const {
  return ego_state_;
}

const LaneLineList& NoLaneLineKernel::GetLaneLineList() const {
  return lane_line_list_;
}

obstacles_ptr_list* const NoLaneLineKernel::MutableGetObstaclesMerged() {
  return &obstacles_merged_;
}

EgoVehicleState* const NoLaneLineKernel::MutableGetEgoState() {
  return &ego_state_;
}

LaneLineList* const NoLaneLineKernel::MutableGetLaneLineList() {
  return &lane_line_list_;
}

bool NoLaneLineKernel::IsSufficientWithObsesPrev() {
  // lp: check the preceding-stored obstacles is sufficient navigate ego
  // vehicle and change the mode.
  if (!lane_line_list_.GetLaneLinePtrPrevious()) {
    if (debug_flag[47]) {
      AERROR << "has no previous lane.";
    }
    return false;
  }
  if (obstacles_perception_manager_.GetPerceptionIdObsPrevious().empty()) {
    if (debug_flag[47]) {
      AERROR << "previous obs is empty.";
    }
    return false;
  }
  return true;
}

const CoordinateSystemConvert& NoLaneLineKernel::GetCoordinateSystemConvert()
    const {
  return coordinate_system_convert_;
}

NoLaneLineKernel::Mode NoLaneLineKernel::GetMode() const {
  return mode_;
}

CoordinateSystemConvert* const
NoLaneLineKernel::MutableCoordinateSystemConvert() {
  return &coordinate_system_convert_;
}

void NoLaneLineKernel::SendToCyber(
    const TL::perception::PerceptionObstacles& perception_obstacles) {
  if (cyber_flag[4]) {
    for (auto& obs : perception_obstacles.perception_obstacle()) {
      ptr_without_lane->mutable_perception_obstacles()
          ->add_perception_obstacles()
          ->CopyFrom(obs);
    }
  }
  if (debug_flag[22]) {
    std::string obs_id = "[";
    for (auto& obs : perception_obstacles.perception_obstacle()) {
      obs_id += std::to_string(obs.id()) + ",";
    }
    obs_id.back() = ']';
    AERROR << "obstacles description  " + obs_id;
  }
}

}  // namespace nolane
}  // namespace planning
}  // namespace TL
