/******************************************************************************
 * Copyright (c) TL Technologies Co., Ltd. 2019-2022. All rights reserved.
 * Author: LingPeng
 * Created Time: 2022/4/21
 *****************************************************************************/

#include "planning/localview/lane_line_builder/obstacle_following_lane_line/obstacle_perception_manager.h"

#include <limits>
#include <list>
#include <map>
#include <memory>
#include <queue>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace TL {
namespace planning {
namespace nolane {
using TL::common::SLPoint;

ObstaclePerceptionManager::ObstaclePerceptionManager()
    : is_close_enough_(false), max_obstacle_nums_(500) {
  if (!id_unique_pool_.get()) {
    id_unique_pool_ = std::make_shared<std::queue<int>>();
  }
  for (int i = 1; i <= max_obstacle_nums_; ++i) {
    id_unique_pool_->push(i);
  }
  timestamp_previous_ = 0.0;
  sequence_num_previous_ = 0;
}

bool ObstaclePerceptionManager::UpdateObstaclePresent(
    const std::shared_ptr<const perception::PerceptionObstacles>& obstacles_raw,
    const CoordinateSystemConvert& coordinate_system_convert,
    const EgoVehicleState& ego_state) {
  double timestamp_present = 0.0;
  if (obstacles_raw->has_header() && obstacles_raw->header().has_data_stamp() &&
      obstacles_raw->header().has_sequence_num()) {
    timestamp_present = obstacles_raw->header().data_stamp();
    auto sequence_num_present = obstacles_raw->header().seq();
    if (SeemsNotEqual(timestamp_previous_, timestamp_present) &&
        sequence_num_previous_ != sequence_num_present) {
      timestamp_previous_ = timestamp_present;
      sequence_num_previous_ = sequence_num_present;
    } else {
      AERROR << "obsolete obstacles   timestamp_previous_:"
             << timestamp_previous_
             << "  timestamp_present:" << timestamp_present
             << "  sequence_num_previous_:" << sequence_num_previous_
             << "   sequence_num_present:" << sequence_num_present;
      return false;
    }
  } else {
    AERROR << "obstacles state abnormal: header state("
           << obstacles_raw->has_header() << ") timestamp state("
           << obstacles_raw->header().has_data_stamp()
           << ") sequence num state("
           << obstacles_raw->header().has_sequence_num() << ").";
    return false;
  }

  obstacles_present_.clear();
  ObstacleRelativePosition(obstacles_raw, ego_state);

  auto SetObsState = [](const ::TL::perception::PerceptionObstacle& obs,
                        StateType* const state) {
    // lp: TODO consider coordinate system conversion.
    if (obs.has_position()) {
      state->position.x = obs.position().x();
      state->position.y = obs.position().y();
    } else {
      AERROR << "obs id:" << obs.id() << " has no position value.";
      return false;
    }
    if (obs.has_velocity()) {
      state->velocity.x = obs.velocity().x();
      state->velocity.y = obs.velocity().y();
    } else {
      AERROR << "obs id:" << obs.id() << " has no velocity.";
    }
    if (obs.has_acceleration()) {
      state->acceleration.x = obs.acceleration().x();
      state->acceleration.y = obs.acceleration().y();
    } else {
      ADEBUG << "obs id:" << obs.id() << " has no acceleration.";
    }
    if (obs.has_theta()) {
      state->theta = obs.theta();
    } else {
      AERROR << "obs id:" << obs.id() << "has no theta.";
    }
    return true;
  };

  double delta_x = 0.0;
  double delta_y = 0.0;
  for (auto& obs : obstacles_raw->perception_obstacle()) {
    delta_x = fabs(obs.position().x() - ego_state.GetVehicleStatePresent().x());
    delta_y = fabs(obs.position().y() - ego_state.GetVehicleStatePresent().y());
    if (sqrt(delta_x * delta_x + delta_y * delta_y) >
            fmax(ego_state.GetVehicleStatePresent().linear_velocity() *
                     FLAGS_trajectory_time_length,
                 FLAGS_nolane_obstacle_min_distance) ||
        (false && sqrt(obs.velocity().x() * obs.velocity().x() +
                       obs.velocity().y() * obs.velocity().y()) <
                      FLAGS_nolane_obs_min_velocity) ||
        (obs.has_type() &&
         !(obs.type() == TL::perception::PerceptionObstacle::TRUCK ||
           obs.type() == TL::perception::PerceptionObstacle::VEHICLE ||
           obs.type() == TL::perception::PerceptionObstacle::BUS))) {
      if (debug_flag[43]) {
        AERROR << "unqualified_obs id(percep):" << obs.id() << PRECISION(3)
               << "  delta_xy:[" << delta_x << "," << delta_y << "]"
               << "  vel_ego:"
               << ego_state.GetVehicleStatePresent().linear_velocity()
               << "   vel_obs_xy:[" << obs.velocity().x() << ","
               << obs.velocity().y() << "].";
      }
      continue;
    }
    int obs_id = obs.id();
    if (id_unique_pool_->empty()) {
      id_unique_pool_->push(++max_obstacle_nums_);
    }
    int obs_id_unique = id_unique_pool_->front();
    id_unique_pool_->pop();
    StateType state{};
    // lp: TODO calculate obstacle state.
    // lp: this is temporary in arbitrary coordinate.
    std::pair<double, double> position;
    coordinate_system_convert.SystemEgo2Earth(obs.position().x(),
                                              obs.position().y(), &position);
    state.time_stamp = timestamp_present;
    if (!SetObsState(obs, &state)) {
      return false;
    }

    auto obs_ptr = std::make_shared<ObstaclePerceptionNoLane>(
        obs_id_unique, obs_id, state, id_unique_pool_, obs);
    perception_id_obs_previous_[obs_id].push_back(obs_ptr);
    obstacles_present_.emplace_back(obs_ptr);
  }
  if (cyber_flag[8]) {
    ObstaclesPresentToCyber();
  }
  if (debug_flag[43]) {
    AERROR << "obstacles_present_size:" << obstacles_present_.size();
    std::string id_unique_perception = "[";
    for (const auto& obs : obstacles_present_) {
      id_unique_perception += std::to_string(obs->GetIdPerception()) + "," +
                              std::to_string(obs->GetIdUnique()) + ";";
    }
    if (id_unique_perception.size() > 1) {
      id_unique_perception.back() = ']';
    } else {
      id_unique_perception = "empty.";
    }
    AERROR << PRECISION(3) << "current_obs_header_time_stamp:"
           << obstacles_raw->header().data_stamp()
           << "  current_cycle_obs_map_id_percep_unique:"
           << id_unique_perception;
    DebugIdMap(__FILE__, __LINE__);
  }
  return true;
}

void ObstaclePerceptionManager::DeleteUnqualifiedObstacle(
    obstacles_ptr_list* const obstacle_merged) {
  for (auto& obs_ptr : *obstacle_merged) {
    obs_ptr->StateIsStable();
    obs_ptr->IsTooFar();
  }

  auto ObsIsNeedRemove =
      [](const std::shared_ptr<ObstaclePerceptionNoLane>& obs) {
        return obs->IsTooFar() || !obs->IsStable();
      };
  obstacles_present_.remove_if(ObsIsNeedRemove);
}

void ObstaclePerceptionManager::MergeObstacles(
    obstacles_ptr_list* const obstacle_merged) {
  ADEBUG << "start merge obstacles.";

  for (auto& obs : obstacles_present_) {
    int obs_id = obs->GetIdPerception();
    auto obs_it = perception_id_obs_previous_.find(obs_id);
    if (obs_it == perception_id_obs_previous_.end()) {
      AERROR << "can't find obs id:" << obs_id;
      continue;
    }

    for (auto& obs_other : perception_id_obs_previous_[obs_id]) {
      if (obs.get() == obs_other.get()) {
        continue;
      }
      if (obs_other->UpdateState(*obs)) {
        obs->SetIsNeedRemove(true);
        break;
      }
    }
  }
  static constexpr double time_max_tracking = 5;
  for (auto& id_obs : perception_id_obs_previous_) {
    std::for_each(id_obs.second.begin(), id_obs.second.end(), [&](auto& obs) {
      if (obs->GetStateHistory().empty()) {
        obs->SetIsNeedRemove(true);
      } else if (timestamp_previous_ -
                     obs->GetStateHistory().back().time_stamp >
                 time_max_tracking) {
        obs->SetIsNeedRemove(true);
      }
    });
  }

  // lp: update obstacle_merged list.

  obstacle_merged->clear();
  std::vector<int> empty_id_perception_;
  std::string delete_id = "deleted_id_per_unique:[";
  for (auto& key_value : perception_id_obs_previous_) {
    if (debug_flag[12]) {
      for (auto& obs : key_value.second) {
        if (obs->IsNeedRemove()) {
          delete_id += std::to_string(obs->GetIdPerception()) + "," +
                       std::to_string(obs->GetIdUnique()) + ";";
        }
      }
    }
    if (cyber_flag[7]) {
      for (auto& obs : key_value.second) {
        // lp: send to cyber.
        for (auto& obs_p : *(ptr_without_lane->mutable_obstacle_manager()
                                 ->mutable_obstacle_present())) {
          if (obs_p.id_unique() == obs->GetIdUnique()) {
            obs_p.set_is_need_remove(true);
            break;
          }
        }
      }
    }
    key_value.second.erase(
        std::remove_if(key_value.second.begin(), key_value.second.end(),
                       [](auto& obs_ptr) { return obs_ptr->IsNeedRemove(); }),
        key_value.second.end());
    if (key_value.second.empty()) {
      empty_id_perception_.push_back(key_value.first);
      continue;
    }
    for (auto& obs : key_value.second) {
      obstacle_merged->emplace_back(obs);
    }
  }
  for (auto id : empty_id_perception_) {
    perception_id_obs_previous_.erase(id);
  }
  if (cyber_flag[7]) {
    ObstaclesPreviousToCyber();
  }
  // ObstaclesMergeToCyber(*obstacle_merged);
  if (debug_flag[12]) {
    delete_id.back() = ']';
    AERROR << delete_id
           << "   obstacles_merged_size:" << obstacle_merged->size();
    DebugString(*obstacle_merged);
  }
}

bool ObstaclePerceptionManager::IsEgoInInAllObstacleTrajectory(
    const obstacles_ptr_list& obstacle_merge, const EgoVehicleState& ego_state,
    const TL::perception::PerceptionObstacles& obstacles_raw) {
  LogProcess::LogProc(__func__);
  obstacle_trajectory_cross_car_.clear();
  is_close_enough_ = false;
  bool allowed_lane_change_followed = LateralLimitCheck(ego_state);
  for (auto& obs : obstacle_merge) {
    auto res = obs->IsEgoInTrajectory(ego_state, obstacles_raw,
                                      allowed_lane_change_followed);
    if (std::get<0>(res)) {
      obstacle_trajectory_cross_car_.push_back(
          {obs, std::get<1>(res), std::get<2>(res), std::get<3>(res)});
    }
  }
  if (debug_flag[45]) {
    AERROR << "obstacle_trajectory_cross_car_ info:";
    for (const auto& val : obstacle_trajectory_cross_car_) {
      AERROR << "obs_id_perp_unique:[" << val.obstacle_ptr->GetIdPerception()
             << "," << val.obstacle_ptr->GetIdUnique()
             << "]  index:" << val.index << "  cost:" << val.cost
             << "  is_closed:" << val.is_closed;
    }
  }
  return !obstacle_trajectory_cross_car_.empty();
}

const std::list<StitchPointInfo>&
ObstaclePerceptionManager::GetObstacleTrajectoryCrossCar() const {
  return obstacle_trajectory_cross_car_;
}

void ObstaclePerceptionManager::ExtendObstacleTrajectory(
    obstacles_ptr_list* const obstacle_ptr) {
  for (auto& obs_ptr : *obstacle_ptr) {
    obs_ptr->TrajectoryExtendFuture();
    obs_ptr->TrajectoryExtendPast();
  }
}

void ObstaclePerceptionManager::DeleteObsBackEgoAndTrimTraj(
    const std::shared_ptr<LaneCenterLine>& lane_previous,
    const EgoVehicleState& ego_state, obstacles_ptr_list* const obstacle_ptr) {
  LogProcess::LogProc(__func__);
  if (debug_flag[14]) {
    AERROR << "previous_lane_reference_line_pointer:"
           << GetPtr(lane_previous->GetReferenceLinePtr());
  }
  for (auto& obs_ptr : *obstacle_ptr) {
    obs_ptr->TrajectoryTrim(ego_state);
  }
}

bool ObstaclePerceptionManager::ProjectTrajectory(
    const std::shared_ptr<LaneCenterLine>& prev_lane_ptr,
    obstacles_ptr_list* const obstacle_ptr, EgoVehicleState* const ego_state) {
  LogProcess::LogProc(__func__);
  auto prev_reference_line_ptr = prev_lane_ptr->GetReferenceLinePtr();
  if (!prev_reference_line_ptr.get()) {
    AERROR << "reference line is nullptr.";
    ObstaclesMergeToCyber(*obstacle_ptr);
    return false;
  }
  if (debug_flag[18]) {
    int counter = 0;
    auto& acc_s = prev_reference_line_ptr->GetMapPath().accumulated_s();
    const auto& ext_path_points =
        prev_lane_ptr->GetReferenceLinePtrExtend()->GetMapPath().path_points();
    AERROR << "Previous line information prev_line_ptr:"
           << GetPtr(prev_reference_line_ptr);
    for (const auto& val :
         prev_reference_line_ptr->GetMapPath().path_points()) {
      AERROR << PRECISION(3) << "previous_ref_x:" << val.x()
             << "  previous_ref_y:" << val.y() << "  heading:" << val.heading()
             << "  ext_ref_x:" << ext_path_points.at(counter).x()
             << "  ext_ref_y:" << ext_path_points.at(counter).y()
             << "  ext_ref_acc_s:" << acc_s.at(counter);
      ++counter;
    }
  }

  for (auto& obs_ptr : *obstacle_ptr) {
    obs_ptr->ProjectOnReferenceLine(
        *prev_reference_line_ptr,
        fmax(ego_state->GetVehicleStatePresent().linear_velocity() *
                 FLAGS_trajectory_time_length,
             FLAGS_nolane_trajectory_minimal_length) +
            ego_state->GetVehicleSl().s());
  }
  ObstaclesMergeToCyber(*obstacle_ptr);
  return true;
}

double ObstaclePerceptionManager::GetTimestampPrevious() const {
  return timestamp_previous_;
}

unsigned int ObstaclePerceptionManager::GetSequenceNumPrevious() const {
  return sequence_num_previous_;
}

const obstacles_ptr_list& ObstaclePerceptionManager::GetObstaclesPresent()
    const {
  return obstacles_present_;
}

const std::map<int, std::vector<std::shared_ptr<ObstaclePerceptionNoLane>>>&
ObstaclePerceptionManager::GetPerceptionIdObsPrevious() const {
  return perception_id_obs_previous_;
}

int ObstaclePerceptionManager::GetMaxObstacleNums() const {
  return max_obstacle_nums_;
}

void ObstaclePerceptionManager::SetObstacleTrajectoryCrossCar(
    const std::list<StitchPointInfo>& obstacleTrajectoryCrossCar) {
  obstacle_trajectory_cross_car_ = obstacleTrajectoryCrossCar;
}

void ObstaclePerceptionManager::SetTimestampPrevious(double timestampPrevious) {
  timestamp_previous_ = timestampPrevious;
}

void ObstaclePerceptionManager::SetSequenceNumPrevious(
    unsigned int sequenceNumPrevious) {
  sequence_num_previous_ = sequenceNumPrevious;
}

void ObstaclePerceptionManager::SetObstaclesPresent(
    const obstacles_ptr_list& obstaclesPresent) {
  obstacles_present_ = obstaclesPresent;
}

void ObstaclePerceptionManager::SetPerceptionIdObsPrevious(
    const std::map<int, std::vector<std::shared_ptr<ObstaclePerceptionNoLane>>>&
        perceptionIdObsPrevious) {
  perception_id_obs_previous_ = perceptionIdObsPrevious;
}

void ObstaclePerceptionManager::SetMaxObstacleNums(int maxObstacleNums) {
  max_obstacle_nums_ = maxObstacleNums;
}

void ObstaclePerceptionManager::DebugString(
    const obstacles_ptr_list& obstacle_ptr) const {
  for (auto& obs : obstacle_ptr) {
    obs->DebugObstacle(__FILE__, __LINE__);
  }
}

void ObstaclePerceptionManager::DebugIdMap(const std::string& file, int line,
                                           const std::string& custom) const {
  for (auto& key_value : perception_id_obs_previous_) {
    std::string id_u = "[";
    for (auto& obs : key_value.second) {
      id_u += std::to_string(obs->GetIdUnique());
      id_u += ",";
    }
    id_u.back() = ']';
    AERROR << "previous_key_map_id:" << key_value.first
           << "  id_unique:" << id_u << "  Call_Func "
           << DebugHelp(file, line, custom);
  }
}

const std::shared_ptr<std::queue<int>>&
ObstaclePerceptionManager::GetIdUniquePool() const {
  return id_unique_pool_;
}

void ObstaclePerceptionManager::SetIdUniquePool(
    const std::shared_ptr<std::queue<int>>& idUniquePool) {
  id_unique_pool_ = idUniquePool;
}

std::vector<int> ObstaclePerceptionManager::ChooseObstaclesToFollowed() {
  LogProcess::LogProc(__func__);
  if (obstacle_trajectory_cross_car_.empty()) {
    AERROR << "obstacle_trajectory_cross_car_ is empty.";
    return {};
  }

  size_t index = 0;
  size_t min_index = 0;
  int min_cost = 0;
  auto begin_it = obstacle_trajectory_cross_car_.begin();
  std::vector<int> v_index;

  for (auto it = begin_it; it != obstacle_trajectory_cross_car_.end(); ++it) {
    index = std::distance(begin_it, it);
    if (!index) {
      // lp: process the first point.
      min_cost = it->cost;
      min_index = index;
    } else if (min_cost > it->cost) {
      min_cost = it->cost;
      min_index = index;
    }
    if (it->cost == 0) {
      v_index.push_back(index);
    }
  }

  if (v_index.empty()) {
    v_index.push_back(min_index);
  }
  if (debug_flag[45]) {
    std::string index_str = "";
    TL::common::util::vec2str(v_index, &index_str);
    AERROR << "alternative_index_in_cross_car:" << index_str;
  }

  return v_index;
}

void ObstaclePerceptionManager::TrajectoryFitForEachObstacle(
    obstacles_ptr_list* const obstacle_ptr) {
  LogProcess::LogProc(__func__);
  for (auto& obs_ptr : *obstacle_ptr) {
    obs_ptr->TrajectoryFit();
  }
}

StitchPointInfo ObstaclePerceptionManager::DecideFollowType(
    const std::vector<int>& v_index, const EgoVehicleState& ego_state) {
  if (v_index.empty()) {
    AERROR << "v_index is empty.";
    is_close_enough_ = false;
    return {};
  }
  const auto& it = obstacle_trajectory_cross_car_.cbegin();
  auto it_f = it;
  int min_index = 0;
  if (v_index.size() == 1) {
    std::advance(it_f, v_index.front());
    if (it_f->cost == 0 || it_f->is_closed) {
      is_close_enough_ = true;
    } else {
      is_close_enough_ = false;
    }
    min_index = v_index.front();
  } else {
    bool all_zero_cost =
        std::all_of(v_index.begin(), v_index.end(), [&](int idx) {
          it_f = it;
          std::advance(it_f, idx);
          return !it_f->cost;
        });
    if (!all_zero_cost) {
      AERROR << "all cost should be zero if size > 1 after filtering the "
                "minimal cost(ChooseObstaclesToFollowed).";
      return {nullptr, 0, 0};
    }
    double dis_min = std::numeric_limits<double>::infinity();
    double dis_t = 0;
    double dis_delta_x = 0;
    double dis_delta_y = 0;
    for (int i = 0; i < v_index.size(); ++i) {
      it_f = it;
      std::advance(it_f, v_index.at(i));
      dis_delta_x = ego_state.GetVehicleStatePresent().x() -
                    it_f->obstacle_ptr->GetStatePresent().position.x;
      dis_delta_y = ego_state.GetVehicleStatePresent().y() -
                    it_f->obstacle_ptr->GetStatePresent().position.y;
      dis_t = sqrt(dis_delta_x * dis_delta_x + dis_delta_y * dis_delta_y);
      if (dis_min > dis_t) {
        is_close_enough_ = true;
        min_index = v_index.at(i);
        dis_min = dis_t;
      }
    }
  }
  it_f = it;
  std::advance(it_f, min_index);
  return *it_f;
}

bool ObstaclePerceptionManager::IsCloseEnough() const {
  return is_close_enough_;
}

void ObstaclePerceptionManager::SetIsCloseEnough(bool isCloseEnough) {
  is_close_enough_ = isCloseEnough;
}

void ObstaclePerceptionManager::ObstacleAssignToProto(
    const std::shared_ptr<ObstaclePerceptionNoLane>& obs_source,
    ObstacleWithoutLane* const obs_destination) const {
  obs_destination->set_id_unique(obs_source->GetIdUnique());
  obs_destination->set_perception_id(obs_source->GetIdPerception());

  auto StateTypeToProto =
      [](const StateType& state_type_raw,
         TL::planning::ObsStateType* const obs_state_type_pb) {
        obs_state_type_pb->set_time(state_type_raw.time_stamp);
        obs_state_type_pb->mutable_position()->set_x(state_type_raw.position.x);
        obs_state_type_pb->mutable_position()->set_y(state_type_raw.position.y);
        obs_state_type_pb->mutable_velocity()->set_x(state_type_raw.velocity.x);
        obs_state_type_pb->mutable_velocity()->set_y(state_type_raw.velocity.y);
        obs_state_type_pb->mutable_acceleration()->set_x(
            state_type_raw.acceleration.x);
        obs_state_type_pb->mutable_acceleration()->set_y(
            state_type_raw.acceleration.y);
        obs_state_type_pb->set_theta(state_type_raw.theta);
      };

  StateTypeToProto(obs_source->GetStatePresent(),
                   obs_destination->mutable_state_present());
  // lp: how to deal with the history state not copy all the state.
  for (auto& state : obs_source->GetStateHistory()) {
    auto obs_des_state = obs_destination->add_state_history();
    StateTypeToProto(state, obs_des_state);
  }
  obs_destination->set_is_need_remove(obs_source->IsNeedRemove());
  obs_destination->set_is_stable(obs_source->IsStable());
  obs_destination->set_is_too_far(obs_source->IsTooFar());
  obs_destination->set_is_update_consistence(obs_source->IsUpdateConsistent());
  obs_destination->set_moving_behavior(
      static_cast<TL::planning::ObstacleWithoutLane::MovingBehavior>(
          obs_source->GetMovingBehavious()));

  int lane_value = static_cast<int>(obs_source->GetLaneType());
  obs_destination->set_lane_type(
      static_cast<TL::planning::ObstacleWithoutLane::LaneType>(lane_value));

  for (auto& sl_s : obs_source->GetStateSl()) {
    auto sl_des = obs_destination->add_sl_position_lane_type();
    sl_des->mutable_sl_position()->set_s(std::get<0>(sl_s).s());
    sl_des->mutable_sl_position()->set_l(std::get<0>(sl_s).l());
    lane_value = static_cast<int>(std::get<1>(sl_s));
    sl_des->set_lane_type(
        static_cast<TL::planning::ObstacleWithoutLane::LaneType>(
            lane_value));
  }
}

void ObstaclePerceptionManager::ObstaclesPresentToCyber() {
  for (const auto& obs : obstacles_present_) {
    auto obs_p =
        ptr_without_lane->mutable_obstacle_manager()->add_obstacle_present();
    ObstacleAssignToProto(obs, obs_p);
  }
}

void ObstaclePerceptionManager::ObstaclesPreviousToCyber() {
  for (auto& obs_id : perception_id_obs_previous_) {
    auto id_map = ptr_without_lane->mutable_obstacle_manager()->add_id_map();
    id_map->set_perception_id(obs_id.first);
    for (auto& obs : obs_id.second) {
      id_map->add_id_unique(obs->GetIdUnique());
    }
  }
}

void ObstaclePerceptionManager::ObstaclesMergeToCyber(
    const obstacles_ptr_list& obstacle_ptr) {
  for (auto& obs : obstacle_ptr) {
    auto obs_merged =
        ptr_without_lane->mutable_obstacle_manager()->add_obstacle_merged();
    ObstacleAssignToProto(obs, obs_merged);
  }
  ptr_without_lane->mutable_obstacle_manager()->set_sequence_num(
      obstacle_ptr.size());
  ptr_without_lane->mutable_obstacle_manager()->set_is_close_enough(
      is_close_enough_);
}

void ObstaclePerceptionManager::EgoIsCloseReferenceLine(
    const EgoVehicleState& ego_state, double start_s) {
  LogProcess::LogProc(__func__);
  if (!std::isinf(start_s) && ego_state.GetVehicleSl().s() > start_s &&
      fabs(ego_state.GetVehicleSl().l()) < FLAGS_nolane_ego_is_close_ref) {
    is_close_enough_ = true;
  } else {
    if (debug_flag[45]) {
      AERROR << "ego is far away refer_line ego_sl:["
             << ego_state.GetVehicleSl().s() << ","
             << ego_state.GetVehicleSl().l() << "]  lane_start_s:" << start_s;
    }
  }
}

void ObstaclePerceptionManager::ObstacleRelativePosition(
    const std::shared_ptr<const TL::perception::PerceptionObstacles>&
        obs_ptr_raw,
    const EgoVehicleState& ego_state) {
  obstacle_relative_state_.clear();
  obstacle_relative_state_.reserve(obs_ptr_raw->perception_obstacle_size());
  CoordinateTransform<2, double> ego_system;
  ego_system.SetNewBaseVector(ego_state.GetVehicleStatePresent().heading());
  std::vector<double> obs_points_translation(2);
  std::vector<double> obs_points_translation_rotate(2);
  StateType state_rel_trans_rotate{};
  for (auto& obs : obs_ptr_raw->perception_obstacle()) {
    obs_points_translation[0] =
        obs.position().x() - ego_state.GetVehicleStatePresent().x();
    obs_points_translation[1] =
        obs.position().y() - ego_state.GetVehicleStatePresent().y();
    ego_system.Origin2NewBaseRotate(obs_points_translation,
                                    &obs_points_translation_rotate);
    state_rel_trans_rotate.position.x = obs_points_translation_rotate[0];
    state_rel_trans_rotate.position.y = obs_points_translation_rotate[1];
    state_rel_trans_rotate.theta =
        obs.theta() - ego_state.GetVehicleStatePresent().heading();
    obstacle_relative_state_.push_back({&obs, state_rel_trans_rotate});
  }
  if (debug_flag[49]) {
    for (auto& obs : obstacle_relative_state_) {
      AERROR << PRECISION(3)
             << "obstacle_rel_state  id_percep:" << obs.per_obs_ptr->id()
             << "  obs_xy_theta:[" << obs.per_obs_ptr->position().x() << ","
             << obs.per_obs_ptr->position().y() << ","
             << obs.per_obs_ptr->theta() << "  v_xy:["
             << obs.per_obs_ptr->velocity().x() << ","
             << obs.per_obs_ptr->velocity().y() << "]   rel_un_rotate:["
             << obs.per_obs_ptr->position().x() -
                    ego_state.GetVehicleStatePresent().x()
             << ","
             << obs.per_obs_ptr->position().y() -
                    ego_state.GetVehicleStatePresent().y()
             << "]"
             << "   rel_xy_theta:[" << obs.pos_rel.position.x << ","
             << obs.pos_rel.position.y << "," << obs.pos_rel.theta << "]";
    }
  }
}

bool ObstaclePerceptionManager::LateralLimitCheck(
    const EgoVehicleState& ego_state) {
  double dis_min = std::numeric_limits<double>::infinity();
  double dis_temp = 0;
  const PercepObsRelPos* per_obs_rel = nullptr;

  std::for_each(
      obstacle_relative_state_.begin(), obstacle_relative_state_.end(),
      [&](const PercepObsRelPos& obs_rel_state) {
        if ((fabs(obs_rel_state.pos_rel.position.y) +
             obs_rel_state.per_obs_ptr->width() / 2) < LaneWidth / 2.0 &&
            obs_rel_state.pos_rel.theta < M_PI / 2) {
          dis_temp = obs_rel_state.pos_rel.position.x;
          if (dis_min > dis_temp) {
            dis_min = dis_temp;
            per_obs_rel = &obs_rel_state;
          }
        }
      });
  if (debug_flag[45]) {
    if (per_obs_rel) {
      AERROR << "dis_min:" << dis_min
             << "  obs_id:" << per_obs_rel->per_obs_ptr->id() << "  rel_xy:["
             << per_obs_rel->pos_rel.position.x << ","
             << per_obs_rel->pos_rel.position.y << "]";
    }
  }
  if (dis_min >
      fmax(FLAGS_nolane_obs_change_lane_min_dis,
           FLAGS_nolane_obs_change_lane_ego_time *
               ego_state.GetVehicleStatePresent().linear_velocity())) {
    return true;
  }
  return false;
}

}  // namespace nolane
}  // namespace planning
}  // namespace TL
