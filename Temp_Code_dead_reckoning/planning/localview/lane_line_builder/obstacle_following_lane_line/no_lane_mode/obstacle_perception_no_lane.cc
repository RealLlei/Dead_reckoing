/******************************************************************************
 * Copyright (c) TL Technologies Co., Ltd. 2019-2022. All rights reserved.
 * Author: LingPeng
 * Created Time: 2022/4/21
 *****************************************************************************/

#include "planning/localview/lane_line_builder/obstacle_following_lane_line/obstacle_perception_no_lane.h"

#include <algorithm>
#include <limits>
#include <numeric>
#include <string>
#include <tuple>

#include "common/math/vec2d.h"
#include "planning/localview/lane_line_builder/obstacle_following_lane_line/common_util/coordinate_transform.h"

#include "proto/common/pnc_point.pb.h"
#include "proto/common/vehicle_config.pb.h"

namespace TL {
namespace planning {
namespace nolane {

using common::SLPoint;
using TL::common::math::Vec2d;

ObstaclePerceptionNoLane::ObstaclePerceptionNoLane(
    int id_unique, int id, StateType state,
    std::shared_ptr<std::queue<int>> id_pool,
    const TL::perception::PerceptionObstacle& perception_obstacle,
    FitType fit_type, signed int order)
    : id_unique_(id_unique),
      id_perception_(id),
      state_present_(state),
      is_stable_(true),
      is_too_far_(false),
      is_update_consistent_(false),
      moving_behavior_(MovingBehavior::Unknown),
      is_need_remove_(false),
      id_unique_pool_ptr_(id_pool),
      lane_type_(LaneType::Unknown),
      polynomial_order_(order),
      default_cost_(-999),
      counter_(0) {
  perception_obstacle_.CopyFrom(perception_obstacle);
  state_history_.push_back(state_present_);
  fit_curve_ = new FitManager(fit_type);
  if (debug_cost_flag_) {
    map_name_list_.emplace_back("cost_all");
    map_name_list_.emplace_back("arc_length");
    map_name_list_.emplace_back("angle_diff");
    map_name_list_.emplace_back("rho");
    map_name_list_.emplace_back("inner");
    map_name_list_.emplace_back("inter_angle_diff");
    map_name_list_.emplace_back("distance");
    map_name_list_.emplace_back("speed");
    for (auto& key : map_name_list_) {
      cost_debug_.insert({key, std::vector<double>()});
    }
  }
}

ObstaclePerceptionNoLane::~ObstaclePerceptionNoLane() {
  if (auto id_pool = id_unique_pool_ptr_.lock()) {
    id_pool->push(id_unique_);
  } else {
    AERROR << "id_unique:" << id_unique_
           << "   id_perception:" << id_perception_ << "   id_pool is empty!";
  }
  delete fit_curve_;
  fit_curve_ = nullptr;
}

bool ObstaclePerceptionNoLane::IsUpdateConsistent() const {
  return is_update_consistent_;
}

void ObstaclePerceptionNoLane::SetIsUpdateConsistent(bool isUpdateConsistent) {
  is_update_consistent_ = isUpdateConsistent;
}

bool ObstaclePerceptionNoLane::UpdateState(
    const ObstaclePerceptionNoLane& obstacle) {
  if (state_history_.empty()) {
    ADEBUG << "id_uni_percep:[" << id_unique_ << "," << id_perception_
           << "  state history is empty.";
    return false;
  }
  const auto& state = obstacle.GetStatePresent();
  auto& state_prev = state_history_.back();
  double delta_time = state.time_stamp - state_prev.time_stamp;
  if (delta_time > FLAGS_nolane_obs_tracking_max_time) {
    if (debug_flag[21]) {
      AERROR << PRECISION(3) << "  id_uni_percep:[" << id_unique_ << ","
             << id_perception_
             << "]  delta_time too big. time_state_prev_pres:["
             << state.time_stamp << "," << state_prev.time_stamp << "]"
             << "  delta_time:" << delta_time;
    }
    return false;
  }
  double bench_x = state_prev.velocity.x * delta_time +
                   state_prev.acceleration.x * delta_time * delta_time / 2 +
                   state_prev.position.x;
  double bench_y = state_prev.velocity.y * delta_time +
                   state_prev.acceleration.y * delta_time * delta_time / 2 +
                   state_prev.position.y;
  double x_min = bench_x - FLAGS_nolane_obs_x_threshold;
  double x_max = bench_x + FLAGS_nolane_obs_x_threshold;
  double y_min = bench_y - FLAGS_nolane_obs_y_threshold;
  double y_max = bench_y + FLAGS_nolane_obs_y_threshold;
  // lp: TODO can be more precision.
  if (cyber_flag[5]) {
    ptr_without_lane->mutable_debug()->add_pure_value(id_unique_);
    ptr_without_lane->mutable_debug()->add_pure_value(obstacle.GetIdUnique());
    ptr_without_lane->mutable_debug()->add_pure_value(state.position.x);
    ptr_without_lane->mutable_debug()->add_pure_value(bench_x);
    ptr_without_lane->mutable_debug()->add_pure_value(x_min);
    ptr_without_lane->mutable_debug()->add_pure_value(x_max);
    ptr_without_lane->mutable_debug()->add_pure_value(state_prev.position.x);
    ptr_without_lane->mutable_debug()->add_pure_value(state_prev.velocity.x);
    ptr_without_lane->mutable_debug()->add_pure_value(
        state_prev.acceleration.x);
    ptr_without_lane->mutable_debug()->add_pure_value(state.position.y);
    ptr_without_lane->mutable_debug()->add_pure_value(bench_y);
    ptr_without_lane->mutable_debug()->add_pure_value(y_min);
    ptr_without_lane->mutable_debug()->add_pure_value(y_max);
    ptr_without_lane->mutable_debug()->add_pure_value(state_prev.position.y);
    ptr_without_lane->mutable_debug()->add_pure_value(state_prev.velocity.y);
    ptr_without_lane->mutable_debug()->add_pure_value(
        state_prev.acceleration.y);
    ptr_without_lane->mutable_debug()->add_pure_value(delta_time);
  }
  bool com_x_min = DefinitelyGreaterEqual(state.position.x, x_min);
  bool com_x_max = DefinitelyLessEqual(state.position.x, x_max);
  bool com_y_min = DefinitelyGreaterEqual(state.position.y, y_min);
  bool com_y_max = DefinitelyLessEqual(state.position.y, y_max);
  if (debug_flag[21]) {
    AERROR << PRECISION(3) << "consistency:"
           << (com_x_min && com_x_max && com_y_min && com_y_max)
           << "  id_uni_percep:[" << id_unique_ << "," << id_perception_ << ":"
           << obstacle.GetIdUnique() << "," << obstacle.GetIdPerception()
           << "]  bench_xy:[" << bench_x << "," << bench_y << "]  xy_min_max:["
           << x_min << "," << x_max << "," << y_min << "," << y_max
           << "]  state_pos_xy:[" << state.position.x << "," << state.position.y
           << "]  prev_sva_xy:[" << state_prev.position.x << ","
           << state_prev.position.y << "," << state_prev.velocity.x << ","
           << state_prev.velocity.y << "," << state_prev.acceleration.x << ","
           << state_prev.acceleration.y << "]  delta_time:" << delta_time
           << "  com_xy_min_max:" << com_x_min << com_x_max << com_y_min
           << com_y_max;
  }
  if (com_x_min && com_x_max && com_y_min && com_y_max) {
    if (sqrt(state.velocity.x * state.velocity.x +
             state.velocity.y * state.velocity.y) <
            FLAGS_nolane_obs_min_velocity &&
        !state_history_.empty() &&
        sqrt(state_history_.back().velocity.x *
                 state_history_.back().velocity.x +
             state_history_.back().velocity.y *
                 state_history_.back().velocity.y) <
            FLAGS_nolane_obs_min_velocity) {
      state_history_.pop_back();
    }
    state_history_.push_back(state);
    state_present_ = state;
    return true;
  } else {
    return false;
  }
}

void ObstaclePerceptionNoLane::TrajectoryExtendFuture() {
  // lp: TODO calculate dynamic state  ....
  state_history_extended_ = state_history_;
  std::for_each(state_history_extended_.begin(), state_history_extended_.end(),
                [](StateType& state) { state.position_fit = state.position; });
  trajectory_type_ = TrajectoryType::Unknown;
  trajectory_need_project_ = false;
  ACHECK(!state_history_extended_.empty())
      << "id_perception_unique:[" << id_perception_ << "  " << id_unique_ << "]"
      << "   trajectory is empty!";
  double delta_x = fabs(state_history_extended_.back().position.x -
                        state_history_extended_.front().position.x);
  double delta_y = fabs(state_history_extended_.back().position.y -
                        state_history_extended_.front().position.y);
  trajectory_distance_length_ = sqrt(delta_x * delta_x + delta_y * delta_y);
  trajectory_time_length_ = state_history_extended_.back().time_stamp -
                            state_history_extended_.front().time_stamp;
  IsTrajectoryProject();
  ProjectionTrajectoryPoint();
}

void ObstaclePerceptionNoLane::TrajectoryExtendPast() {
  // lp: TODO 2 different aspect...
  // lp: state_history_extended_.push_back(...);
}

void ObstaclePerceptionNoLane::TrajectoryTrim(
    const EgoVehicleState& ego_state) {
  auto it =
      std::find_if(state_sl_.begin(), state_sl_.end(), [&](const auto& sl_p) {
        return DefinitelyGreater(
                   std::get<0>(sl_p).s(),
                   ego_state.GetVehicleSl().s() -
                       1.2 * FLAGS_nolane_trajectory_fit_radius) &&
               DefinitelyGreater(std::get<0>(sl_p).s(), 0.0);
      });

  if (debug_flag[14]) {
    AERROR << PRECISION(3) << "obs_id_unique:" << id_unique_
           << "  id_perception:" << id_perception_ << "  width:["
           << perception_obstacle_.width() << "," << GetWidth() << "]"
           << "  lane_type:" << static_cast<int>(lane_type_)
           << "  moving_behavior:" << static_cast<int>(moving_behavior_)
           << " ego_s:" << ego_state.GetVehicleSl().s()
           << "  ego_l:" << ego_state.GetVehicleSl().l()
           << "  delete_obs_until_index:"
           << std::distance(state_sl_.begin(), it)
           << "   traj_length:" << trajectory_distance_length_
           << "  traj_time:" << trajectory_time_length_;
    if (state_sl_.size() == state_history_extended_.size()) {
      size_t counter = 0;
      std::string ref_sl = "";
      bool size_enough = state_history_extended_.size() > 3;
      std::string obs_x = "obs_x=[";
      std::string obs_y = "obs_y=[";
      std::string obs_x_f = "obs_x_f=[";
      std::string obs_y_f = "obs_y_f=[";
      std::string obs_x_l = "obs_x_l=[";
      std::string obs_y_l = "obs_y_l=[";
      std::string obs_time = "obs_time=[";
      for (int i = 0; i < state_history_extended_.size(); ++i) {
        if (i < state_sl_.size()) {
          ref_sl =
              "  in_prev_ref_s_l:[" +
              std::to_string(std::get<0>(state_sl_.at(i)).s()) + "," +
              std::to_string(std::get<0>(state_sl_.at(i)).l()) +
              "]  lane_type:" +
              std::to_string(static_cast<int>(std::get<1>(state_sl_.at(i))));
        } else {
          ref_sl = "in_prev_ref_s_l is too far.";
        }
        AERROR << "counter:" << counter << PRECISION(3) << "   traj_extend_x:"
               << state_history_extended_.at(counter).position.x
               << "   traj_extend_y:"
               << state_history_extended_.at(counter).position.y
               << "   traj_extend_theta:"
               << state_history_extended_.at(counter).theta << "   rel_xy:["
               << state_history_extended_.at(counter).position_projection.x
               << ","
               << state_history_extended_.at(counter).position_projection.y
               << "]  fit_xy:["
               << state_history_extended_.at(counter).position_fit.x << ","
               << state_history_extended_.at(counter).position_fit.y << "]  "
               << ref_sl;
        if (size_enough) {
          obs_x +=
              std::to_string(state_history_extended_.at(counter).position.x) +
              ",";
          obs_y +=
              std::to_string(state_history_extended_.at(counter).position.y) +
              ",";
          obs_x_f += std::to_string(
                         state_history_extended_.at(counter).position_fit.x) +
                     ",";
          obs_y_f += std::to_string(
                         state_history_extended_.at(counter).position_fit.y) +
                     ",";
          obs_x_l +=
              std::to_string(
                  state_history_extended_.at(counter).position_projection.x) +
              ",";
          obs_y_l +=
              std::to_string(
                  state_history_extended_.at(counter).position_projection.y) +
              ",";
          obs_time +=
              std::to_string(state_history_extended_.at(counter).time_stamp) +
              ",";
        }
        ++counter;
      }
      if (size_enough) {
        obs_x.back() = ']';
        obs_y.back() = ']';
        obs_x_f.back() = ']';
        obs_y_f.back() = ']';
        obs_x_l.back() = ']';
        obs_y_l.back() = ']';
        obs_time.back() = ']';
        AERROR << obs_x;
        AERROR << obs_y;
        AERROR << obs_x_f;
        AERROR << obs_y_f;
        AERROR << obs_x_l;
        AERROR << obs_y_l;
        AERROR << obs_time;
      }
    } else {
      AERROR << "state_sl size:" << state_sl_.size()
             << ",  traj_size:" << state_history_extended_.size() << ".";
    }
  }
  state_sl_.erase(state_sl_.begin(), it);
  auto it_state = state_history_extended_.begin();
  size_t dis_extend = std::distance(state_sl_.begin(), it);
  std::advance(it_state, dis_extend);
  TrimTrajectoryHistory(dis_extend);
  state_history_extended_.erase(state_history_extended_.begin(), it_state);
  if (debug_flag[14]) {
    AERROR << "state_sl size:" << state_sl_.size()
           << "  his_ext_size:" << state_history_extended_.size()
           << "  his_size:" << state_history_.size();
    AERROR << "";
  }
}

void ObstaclePerceptionNoLane::ProjectOnReferenceLine(
    const ReferenceLine& reference_line, const double max_proj_s) {
  Vec2d position_xy;
  SLPoint position_sl;
  state_sl_.clear();
  for (auto& state : state_history_extended_) {
    position_xy.set_x(GetExtendStateIndex(state).x);
    position_xy.set_y(GetExtendStateIndex(state).y);
    reference_line.XYToSL(position_xy, &position_sl);
    if (position_sl.s() > max_proj_s) {
      // lp: TODO hypothesis that traj'sl is monotone increasing, but for
      // some trajectory the conclusion is wrong.
      if (debug_flag[38]) {
        AERROR << "position_s:" << position_sl.s()
               << "  max_proj_s:" << max_proj_s;
      }
      // lp: here will lead to the size of state_sl_ and state_history_extend
      // are not-equal.
      break;
    }
    state_sl_.push_back(
        {position_sl, ObstaclePerceptionNoLane::LaneType::Unknown});
  }
  IdentifyLaneBehavior(reference_line.GetMapPath().length());
}

void ObstaclePerceptionNoLane::IdentifyLaneBehavior(const double ref_length) {
  std::for_each(state_sl_.begin(), state_sl_.end(), [&](auto& pos_sl) {
    if (DefinitelyGreaterEqual(std::get<0>(pos_sl).s(), ref_length)) {
      std::get<1>(pos_sl) = LaneType::TooFarUnDefined;
      return;
    }
    if (DefinitelyGreater(std::get<0>(pos_sl).l(),
                          LaneWidth * 0.5 - GetWidth()) &&
        DefinitelyLessEqual(std::get<0>(pos_sl).l(),
                            LaneWidth * 1.5 - GetWidth())) {
      std::get<1>(pos_sl) = LaneType::LeftLane;
    } else if (DefinitelyGreater(std::get<0>(pos_sl).l(),
                                 LaneWidth * 1.5 - GetWidth())) {
      std::get<1>(pos_sl) = LaneType::LLeftLane;
    } else if (DefinitelyLess(std::get<0>(pos_sl).l(),
                              -LaneWidth * 0.5 + GetWidth()) &&
               DefinitelyGreaterEqual(std::get<0>(pos_sl).l(),
                                      -LaneWidth * 1.5 + GetWidth())) {
      std::get<1>(pos_sl) = LaneType::RightLane;
    } else if (DefinitelyLess(std::get<0>(pos_sl).l(),
                              -LaneWidth * 1.5 + GetWidth())) {
      std::get<1>(pos_sl) = LaneType::RRightLane;
    } else {
      std::get<1>(pos_sl) = LaneType::EgoLane;
    }
  });
  if (!state_sl_.empty()) {
    lane_type_ = std::get<1>(state_sl_.at(0));
  }
  // lp: TODO consider left lleft right rright lane,
  // lp: here just calc ego_lane.
  bool is_not_ego_lane =
      std::any_of(state_sl_.begin(), state_sl_.end(), [](auto& pos_sl) {
        return std::get<1>(pos_sl) != LaneType::EgoLane &&
               std::get<1>(pos_sl) != LaneType::TooFarUnDefined;
      });
  bool is_not_all_too_far =
      std::any_of(state_sl_.begin(), state_sl_.end(), [](auto& pos_sl) {
        return std::get<1>(pos_sl) != LaneType::TooFarUnDefined;
      });
  if (is_not_ego_lane) {
    // lp: lane_type_ should be more precision, here is same as pre-defined.
    lane_type_ = std::get<1>(state_sl_.at(0));
    moving_behavior_ = MovingBehavior::Unknown;
  } else if (!is_not_all_too_far) {
    lane_type_ = LaneType::Unknown;
    moving_behavior_ = MovingBehavior::Unknown;
  } else {
    moving_behavior_ = MovingBehavior::Cruise;
  }
  if (debug_flag[15]) {
    std::string lane_type = "";
    std::string moving_behavior = "";
    switch (lane_type_) {
      case LaneType::EgoLane:
        lane_type = "ego_lane";
        break;
      case LaneType::LeftLane:
        lane_type = "left_lane";
        break;
      case LaneType::LLeftLane:
        lane_type = "lleft_lane";
        break;
      case LaneType::RRightLane:
        lane_type = "rright_lane";
        break;
      case LaneType::RightLane:
        lane_type = "right_lane";
        break;
      case LaneType::Unknown:
        lane_type = "unknown";
        break;
      case LaneType::TooFarUnDefined:
        lane_type = "too_far";
        break;
      default:
        lane_type = "import undefined!!!";
        break;
    }
    switch (moving_behavior_) {
      case MovingBehavior::Cruise:
        moving_behavior = "moving_cruise";
        break;
      default:
        moving_behavior = "unknown.";
        break;
    }
    std::string traj_type = "";
    switch (trajectory_type_) {
      case TrajectoryType::Straight:
        traj_type = "trajectory_straight";
        break;
      case TrajectoryType::Curve:
        traj_type = "trajectory_curve";
        break;
      default:
        traj_type = "trajectory_unknown";
        break;
    }
    AERROR << "id_unique_percep:[" << id_unique_ << "," << id_perception_
           << "]  is_in_ego_lane:" << !is_not_ego_lane
           << "  lane_type:" << lane_type
           << "  moving_behavior:" << moving_behavior << PRECISION(2)
           << "  traj_type:" << traj_type
           << "  traj_statistic [mean:" << trajectory_static_feature_.mean
           << "  var:" << trajectory_static_feature_.variance
           << "  std_e:" << trajectory_static_feature_.std_error
           << "  skw:" << trajectory_static_feature_.skewness
           << "  kur:" << trajectory_static_feature_.kurtosis
           << "  init:" << trajectory_static_feature_.init
           << "  complete:" << trajectory_static_feature_.calc_complete
           << "  size:" << trajectory_static_feature_.size << "]";
  }
}

const std::deque<StateType>& ObstaclePerceptionNoLane::GetStateHistoryExtended()
    const {
  return state_history_extended_;
}

int ObstaclePerceptionNoLane::CostByTwoState(
    double distance, const common::VehicleState& veh_state, const Vec2d& obs_v,
    const Vec2d& delta_pos, const Vec2d& obs_rel) {
  Vec2d ego_v{veh_state.linear_velocity() * cos(veh_state.heading()),
              veh_state.linear_velocity() * sin(veh_state.heading())};
  // lp: TODO calculate ego trajectory in future time.
  double angle_diff = fabs(obs_v.Angle() - ego_v.Angle());
  double speed_diff = fabs(veh_state.linear_velocity() - obs_v.Length());
  if (fabs(angle_diff) < 0.1 && obs_rel.y() < 0.4 &&
      distance < FLAGS_nolane_obs_ego_too_closed_ignore_cost) {
    return 0;
  }
  /*  fml:
   *以自车位置为坐标原点，自车速度方向为x轴方向，可以知道障碍物点的坐标(dx,dy)和方向(diff_angle),
   *然后建立经过这两点且圆心在y轴的圆，分别求取该圆的曲率角度差和弧长作为cost.
   * The following code calculates firstly obstacle's coordinate points(dx,dy)
   * and moving direction(diff_angle) in ego coordinate system in which ego
   * linear velocity direction is the x-axis and the driver's left-90deg
   * direction is the y-axis. Then it constructs a circle that passes
   *the ego-point and the obstacle points. In the end, it gets the obstacle
   *cost based on the circle's central angle and arc length.
   */
  double rho = fabs(cos(delta_pos.Angle()) / (distance / 2));
  double arc_length = fabs(2 * delta_pos.Angle()) / rho;
  double inner_prod_cost = 1.0 / fabs(ego_v.InnerProd(obs_v));
  if (debug_cost_flag_) {
    cost_debug_["arc_length"].at(counter_) = arc_length;
    cost_debug_["angle_diff"].at(counter_) = angle_diff;
    cost_debug_["inter_angle_diff"].at(counter_) = 0;
    cost_debug_["inner"].at(counter_) = inner_prod_cost;
    cost_debug_["rho"].at(counter_) = rho;
    cost_debug_["distance"].at(counter_) = distance;
    cost_debug_["speed"].at(counter_) = speed_diff;
  }
  return static_cast<int>(FLAGS_nolane_obs_ego_cost_dis * distance +
                          FLAGS_nolane_obs_ego_cost_lateral * obs_rel.y() +
                          FLAGS_nolane_obs_ego_cost_theta * angle_diff +
                          FLAGS_nolane_obs_ego_cost_speed * speed_diff);
}

std::tuple<bool, int, int, bool> ObstaclePerceptionNoLane::IsEgoInTrajectory(
    const EgoVehicleState& ego_state,
    const TL::perception::PerceptionObstacles& obstacles_raw,
    bool lane_change_followed) {
  std::tuple<bool, int, int, bool> optimal_point{false, 0, 0, false};
  const auto& veh_state = ego_state.GetVehicleStatePresent();

  int min_size = 10;
  if (state_history_extended_.size() < min_size) {
    if (debug_flag[6]) {
      AERROR << PRECISION(3) << "id_unique_:" << id_unique_
             << "  id_perception:" << id_perception_
             << "  trajectory_size:" << state_history_extended_.size();
    }
    return optimal_point;
  }

  counter_ = 0;
  double dis = 0.0;
  int cost_min = std::numeric_limits<int32_t>::max();
  int cost_current = 0.0;

  Vec2d obs_v{0, 0};
  double obs_abs_v = 0;
  Vec2d delta_pos;
  CoordinateTransform<2, double> ego_system;
  ego_system.SetNewBaseVector(veh_state.heading());

  if (debug_cost_flag_) {
    for (auto& val : cost_debug_) {
      val.second.resize(state_history_extended_.size(), default_cost_);
    }
  }

  const double max_time = 5.0;
  std::vector<double> pos_rel_raw(2);
  std::vector<double> pos_rel_rotate(2);  // lp: flu system
  const double max_speed_offset = 15;
  for (auto& state : state_history_extended_) {
    delta_pos.set_x(state.position.x - veh_state.x());
    delta_pos.set_y(state.position.y - veh_state.y());
    obs_abs_v = sqrt(state.velocity.x * state.velocity.x +
                     state.velocity.y * state.velocity.y);
    dis = delta_pos.Length();
    obs_v.set_x(obs_abs_v * cos(state.theta));
    obs_v.set_y(obs_abs_v * sin(state.theta));

    pos_rel_raw.at(0) = delta_pos.x();
    pos_rel_raw.at(1) = delta_pos.y();
    ego_system.Origin2NewBaseRotate(pos_rel_raw, &pos_rel_rotate);

    std::string state_str = "";
    if (debug_flag[6]) {
      state_str =
          "lane_change_followed:" + std::to_string(lane_change_followed) +
          "   obs_xy_theta:[" + std::to_string(state.position.x) + "," +
          std::to_string(state.position.y) + "," + std::to_string(state.theta) +
          "]  xy_rel:[" + std::to_string(pos_rel_rotate.at(0)) + "," +
          std::to_string(pos_rel_rotate.at(1)) + "]   xy_raw:[" +
          std::to_string(pos_rel_raw.at(0)) + "," +
          std::to_string(pos_rel_raw.at(1)) + "]   v_obs_ego:[" +
          std::to_string(obs_abs_v) + "," +
          std::to_string(veh_state.linear_velocity()) + "]";
    }

    bool flag_dis = dis > fmax(FLAGS_nolane_obs_ego_too_far_ignore_cost,
                               max_time * veh_state.linear_acceleration());
    const double theta_diff = veh_state.heading() - state.theta;
    bool flag_theta = fabs(theta_diff) > M_PI / 2;
    bool flag_rel_x = pos_rel_rotate.at(0) < 0;
    bool flag_rel_y = lane_change_followed
                          ? fabs(pos_rel_rotate.at(1)) > 0.6 * LaneWidth
                          : fabs(pos_rel_rotate.at(1)) > 0.5 * LaneWidth;
    bool flag_rel_v =
        obs_abs_v - veh_state.linear_velocity() > max_speed_offset;

    if (flag_dis || flag_theta || flag_rel_x || flag_rel_y || flag_rel_v) {
      if (debug_flag[6]) {
        AERROR << "continue   counter:" << counter_ << PRECISION(3)
               << "  dis:" << dis
               << "  relative_angle:" << veh_state.heading() - state.theta
               << "   " << state_str << "  dis_theta_x_y_v:" << flag_dis
               << flag_theta << flag_rel_x << flag_rel_y << flag_rel_v;
      }
      ++counter_;
      continue;
    }

    Vec2d pos_rel_v{pos_rel_rotate[0], pos_rel_rotate[1]};
    cost_current = CostByTwoState(dis, veh_state, obs_v, delta_pos, pos_rel_v);
    if (debug_cost_flag_) {
      cost_debug_["cost_all"].at(counter_) = cost_current;
    }

    if (debug_flag[6]) {
      state_str += "  min_cost:" + std::to_string(cost_min) +
                   "  current_cost:" + std::to_string(cost_current);
    }
    if (fabs(theta_diff) < FLAGS_nolane_ego_follow_obs_theta &&
        fabs(pos_rel_rotate.at(1)) < FLAGS_nolane_ego_follow_obs_rel_y &&
        fabs(pos_rel_rotate.at(0)) < FLAGS_nolane_ego_follow_obs_rel_x) {
      if (debug_flag[6]) {
        AERROR << state_str << "   theta_diff:" << theta_diff;
      }
      std::get<0>(optimal_point) = true;
      std::get<1>(optimal_point) = 0;
      std::get<2>(optimal_point) = 0;
      std::get<3>(optimal_point) = true;
      break;
      //   return {true, 0, 0, true};
    } else if (cost_min > cost_current &&
               pos_rel_rotate.at(0) > FLAGS_nolane_ego_follow_obs_rel_x / 2.0) {
      const auto& param =
          common::VehicleConfigHelper::GetConfig().vehicle_param();
      std::array<double, 3> ego_state_rel{0, 0, tan(veh_state.heading())};
      std::array<double, 3> obs_state_rel{state.position.x - veh_state.x(),
                                          state.position.y - veh_state.y(),
                                          tan(state.theta)};
      auto coef = PolynomialConnectTwoPoint(ego_state_rel, obs_state_rel);
      std::vector<double> v_coef;
      double dis_delta = 0.5;
      std::vector<PointDirection> points(
          static_cast<int>(fabs(obs_state_rel[0]) / dis_delta));
      for (int i = 0; i < coef.size(); ++i) {
        v_coef.emplace_back(coef(i, 0));
      }
      double x_t = 0;
      dis_delta = obs_state_rel[0] > 0 ? 0.5 : -0.5;
      for (int i = 0; i < points.size(); ++i) {
        x_t = i * dis_delta;
        if (fabs(x_t) > fabs(obs_state_rel[0])) {
          break;
        }
        points.at(i) = {
            x_t + veh_state.x(),
            v_coef[0] * x_t * x_t * x_t + v_coef[1] * x_t * x_t +
                v_coef[2] * x_t + v_coef[3] + veh_state.y(),
            3 * v_coef[0] * x_t * x_t + 2 * v_coef[1] * x_t + v_coef[2]};
      }
      if (MaxCurvatureInPolynomial(v_coef, ego_state_rel[0], obs_state_rel[0]) <
              ego_state.GetMaxCurvature() &&
          !CollisionCheckWithAllObstacles(points, obstacles_raw, param)) {
        cost_min = cost_current;
        std::get<0>(optimal_point) = true;
        if (debug_flag[6]) {
          if (!cost_min) {
            AERROR << state_str
                   << "  minimal cost update and  loop will break because "
                      "cost_min is 0.";
          } else {
            AERROR << state_str << "   minimal cost update.";
          }
        }
        std::get<1>(optimal_point) = counter_;
        std::get<2>(optimal_point) = cost_min;
        if (!cost_min) {
          break;
        }
      }
    } else {
      if (debug_flag[6]) {
        AERROR << state_str << "  maintain previous cost.";
      }
    }
    ++counter_;
  }

  if (debug_flag[6]) {
    AERROR << PRECISION(3) << "id_unique_:" << id_unique_
           << "  id_perception:" << id_perception_;
    std::string cost_string = "";
    for (int i = 0; i < state_history_extended_.size(); ++i) {
      cost_string = "";
      if (debug_cost_flag_) {
        for (auto& key : map_name_list_) {
          cost_string +=
              "  cost_" + key + ":" + std::to_string(cost_debug_[key].at(i));
        }
      }
      AERROR << PRECISION(3) << "counter:" << i << "  obs_raw_traj_x:"
             << state_history_extended_.at(i).position.x  //- veh_state.x()
             << "  obs_raw_traj_y:"
             << state_history_extended_.at(i).position.y  //- veh_state.y()
             << "  v_xy:[" << state_history_.at(i).velocity.x << ","
             << state_history_.at(i).velocity.y
             << "]  obs_traj_theta:" << state_history_extended_.at(i).theta
             << cost_string;
    }
    AERROR << "";

    auto debug = ptr_without_lane->mutable_debug()->add_temp_value();
    debug->set_name("flag_cost_num");
    debug->set_flag(std::get<0>(optimal_point));
    debug->set_i1(std::get<1>(optimal_point));
    debug->set_i2(std::get<2>(optimal_point));
  }

  return optimal_point;
}  // namespace nolane

auto ObstaclePerceptionNoLane::GetStateSl() const
    -> const std::vector<std::tuple<SLPoint, LaneType>>& {
  return state_sl_;
}

void ObstaclePerceptionNoLane::SetStateSl(
    const std::vector<std::tuple<SLPoint, LaneType>>& stateSl) {
  state_sl_ = stateSl;
}

int ObstaclePerceptionNoLane::GetIdUnique() const {
  return id_unique_;
}

int ObstaclePerceptionNoLane::GetIdPerception() const {
  return id_perception_;
}

const StateType& ObstaclePerceptionNoLane::GetStatePresent() const {
  return state_present_;
}

ObstaclePerceptionNoLane::MovingBehavior
ObstaclePerceptionNoLane::GetMovingBehavious() const {
  return moving_behavior_;
}

const std::deque<StateType>& ObstaclePerceptionNoLane::GetStateHistory() const {
  return state_history_;
}

ObstaclePerceptionNoLane::LaneType ObstaclePerceptionNoLane::GetLaneType()
    const {
  return lane_type_;
}

void ObstaclePerceptionNoLane::SetIdUnique(int idUnique) {
  id_unique_ = idUnique;
}

void ObstaclePerceptionNoLane::SetIdPerception(int idPerception) {
  id_perception_ = idPerception;
}

void ObstaclePerceptionNoLane::SetStatePresent(const StateType& statePresent) {
  state_present_ = statePresent;
}

void ObstaclePerceptionNoLane::SetMovingBehavious(
    ObstaclePerceptionNoLane::MovingBehavior movingBehavious) {
  moving_behavior_ = movingBehavious;
}

void ObstaclePerceptionNoLane::SetStateHistory(
    const std::deque<StateType>& stateHistory) {
  state_history_ = stateHistory;
}

void ObstaclePerceptionNoLane::SetStateHistoryExtended(
    const std::deque<StateType>& stateHistoryExtended) {
  state_history_extended_ = stateHistoryExtended;
}

void ObstaclePerceptionNoLane::SetLaneType(
    ObstaclePerceptionNoLane::LaneType laneType) {
  lane_type_ = laneType;
}

bool ObstaclePerceptionNoLane::IsNeedRemove() const {
  return is_need_remove_;
}

void ObstaclePerceptionNoLane::SetIsNeedRemove(bool isNeedRemove) {
  is_need_remove_ = isNeedRemove;
}

bool ObstaclePerceptionNoLane::IsStable() const {
  return is_stable_;
}

void ObstaclePerceptionNoLane::StateIsStable() {
  // lp: TODO obstacle is in large std-var need be delete.
  return;
}

void ObstaclePerceptionNoLane::DebugObstacle(const std::string& file, int line,
                                             const std::string& custom) const {
  std::string obs_raw_info;
  obs_raw_info = "xy_theta_vxy:[" +
                 std::to_string(perception_obstacle_.position().x()) + "," +
                 std::to_string(perception_obstacle_.position().y()) + "," +
                 std::to_string(perception_obstacle_.theta()) + "," +
                 std::to_string(perception_obstacle_.velocity().x()) + "," +
                 std::to_string(perception_obstacle_.velocity().y()) +
                 "]  CallFunc " + DebugHelp(file, line, custom);
  AERROR << "obs_id_uni_per:[" << id_unique_ << "," << perception_obstacle_.id()
         << PRECISION(3) << "]  present_time:" << state_present_.time_stamp
         << "  " << obs_raw_info;
}

void ObstaclePerceptionNoLane::ObsIsTooFarFromEgo(
    const EgoVehicleState& ego_state) {
  // lp: TODO obstacle which is too far from ego car need be deleted.
  is_too_far_ = false;
}

bool ObstaclePerceptionNoLane::IsTooFar() const {
  return is_too_far_;
}

void ObstaclePerceptionNoLane::TrajectoryFit() {
  if (!trajectory_need_project_) {
    return;
  }
  CheckTrajectoryIsStraight();
  if (trajectory_type_ == TrajectoryType::Straight) {
    fit_curve_->SetPolynomialOrder(1);
    std::vector<Vec2d> raw_points;
    raw_points.reserve(state_history_extended_.size());
    for (auto& points : state_history_extended_) {
      raw_points.emplace_back(points.position_projection.x,
                              points.position_projection.y);
    }
    fit_curve_->Fit(raw_points);
    coordinate_sys_.SetNewBaseVector(-coordinate_sys_.Angle());
    std::vector<double> input_p(2);
    std::vector<double> output_p(2);
    for (int i = 0; i < raw_points.size(); ++i) {
      raw_points.at(i).set_y(
          fit_curve_->GenerateSinglePoint(raw_points.at(i).x()));
      input_p[0] = raw_points.at(i).x();
      input_p[1] = raw_points.at(i).y();
      coordinate_sys_.Origin2NewBaseRotatePan(input_p, &output_p);
      state_history_extended_.at(i).position_fit.x = output_p[0];
      state_history_extended_.at(i).position_fit.y = output_p[1];
    }
  }
}

void ObstaclePerceptionNoLane::TrimTrajectoryHistory(size_t index_traj_extend) {
  if (index_traj_extend == state_history_extended_.size()) {
    state_history_.erase(state_history_.begin(), state_history_.end());
    return;
  }
  auto it_e = state_history_extended_.begin();
  std::advance(it_e, index_traj_extend);
  for (auto it = state_history_.begin(); it != state_history_.end(); ++it) {
    if (SeemsEqual((*it).position.x, (*it_e).position.x)) {
      state_history_.erase(state_history_.begin(), it);
      return;
    }
  }
}

double ObstaclePerceptionNoLane::GetWidth() const {
  static constexpr double factor = 0.8;
  static constexpr double width_min_valid = 0.5;
  return fmin(fmax(perception_obstacle_.width() / 2.0, 1.0) * factor,
              LaneWidth / 2.0 - width_min_valid);
}

bool ObstaclePerceptionNoLane::CollisionCheckWithEgo(
    const std::vector<PointDirection>& lane_line_points,
    const perception::PerceptionObstacle& obstacle,
    const TL::common::VehicleParam& veh_param) {
  double dis_t = 0.0;
  double dis_min = std::numeric_limits<double>::max();
  double index_min = -1;
  double delta_x = 0.0;
  double delta_y = 0.0;
  int point_size = lane_line_points.size();
  auto GetDis = [&](int i) {
    delta_x = obstacle.position().x() - lane_line_points.at(i).x;
    delta_y = obstacle.position().y() - lane_line_points.at(i).y;
    return sqrt(delta_x * delta_x + delta_y * delta_y);
  };
  int i = 0;
  bool is_increase_direction = true;
  if (lane_line_points.size() > 8) {
    double dis_t1 = GetDis(0);
    double dis_t2 = GetDis(point_size - 1);
    if (dis_t1 > dis_t2) {
      dis_min = dis_t2;
      i = point_size - 1;
      index_min = i--;
      is_increase_direction = false;
    } else {
      dis_min = dis_t1;
      index_min = i++;
    }
  }
  for (; i >= 0 && i < point_size; is_increase_direction ? ++i : --i) {
    dis_t = GetDis(i);
    // lp: assumption is the dis is monotonous.
    if (dis_min > dis_t) {
      index_min = i;
      dis_min = dis_t;
    } else {
      break;
    }
  }

  if (index_min >= 0) {
    // check if collision will happen?
    TL::common::math::Box2d box_ego{
        {lane_line_points.at(index_min).x, lane_line_points.at(index_min).y},
        atan(lane_line_points.at(index_min).slope),
        veh_param.length(),
        veh_param.width()};
    TL::common::math::Box2d box_obs{
        {obstacle.position().x(), obstacle.position().y()},
        obstacle.theta(),
        obstacle.length(),
        obstacle.width()};
    return box_ego.HasOverlap(box_obs);
  } else {
    return false;
  }
}

bool ObstaclePerceptionNoLane::CollisionCheckWithAllObstacles(
    const std::vector<PointDirection>& lane_line_points,
    const perception::PerceptionObstacles& obstacles_raw,
    const TL::common::VehicleParam& veh_param) {
  return std::any_of(obstacles_raw.perception_obstacle().begin(),
                     obstacles_raw.perception_obstacle().end(),
                     [&](const perception::PerceptionObstacle& obs) {
                       return obs.id() != this->id_perception_ &&
                              CollisionCheckWithEgo(lane_line_points, obs,
                                                    veh_param);
                     });
}

void ObstaclePerceptionNoLane::ProjectionTrajectoryPoint() {
  if (!trajectory_need_project_) {
    return;
  }
  std::vector<StateType> state_position(state_history_extended_.size());
  // lp: consider to return lhs instead, but need to check the implementation of
  // std::partial_sum in the platform of vehicle.
  std::partial_sum(state_history_extended_.begin(),
                   state_history_extended_.end(), state_position.begin(),
                   [](StateType& lhs, StateType& rhs) {
                     StateType rev;
                     rev.position.x = lhs.position.x + rhs.position.x;
                     rev.position.y = lhs.position.y + rhs.position.y;
                     rev.time_stamp = lhs.time_stamp;
                     return rev;
                   });
  int part_size = state_history_extended_.size() / 3;
  StateType front{state_position.at(part_size - 1) / part_size};
  StateType end{state_position.at(2 * part_size - 1) / (2 * part_size)};

  const StateType delta = end - front;
  Vec2d delta_dis{delta.position.x, delta.position.y};
  coordinate_sys_.SetNewBaseVectorPan(delta_dis.Angle(),
                                      {front.position.x, front.position.y});
  std::vector<double> output_state(2);
  for (StateType& state : state_history_extended_) {
    coordinate_sys_.Origin2NewBasePanRotate(
        {state.position.x, state.position.y}, &output_state);
    state.position_projection.x = output_state.at(0);
    state.position_projection.y = output_state.at(1);
  }
}

void ObstaclePerceptionNoLane::IsTrajectoryProject() {
  trajectory_need_project_ = trajectory_distance_length_ > 2 &&
                             trajectory_time_length_ > 0.3 &&
                             state_history_extended_.size() > 3;
}

void ObstaclePerceptionNoLane::CheckTrajectoryIsStraight() {
  std::vector<double> rel_y;
  rel_y.reserve(state_history_extended_.size());
  std::transform(state_history_extended_.begin(), state_history_extended_.end(),
                 std::back_inserter(rel_y), [](const StateType& state) {
                   return state.position_projection.y;
                 });
  // lp: strictly random access iterator.
  bool is_up = rel_y.at(1) > rel_y.at(0);
  bool monotonous = true;
  for (int i = 2; i < rel_y.size(); ++i) {
    if (is_up) {
      if (rel_y.at(i) < rel_y.at(i - 1)) {
        monotonous = false;
        break;
      }
    } else {
      if (rel_y.at(i) > rel_y.at(i - 1)) {
        monotonous = false;
        break;
      }
    }
  }
  if (monotonous) {
    trajectory_type_ = TrajectoryType::Curve;
    return;
  }

  std::sort(rel_y.begin(), rel_y.end());
  double drive_straight_buffer = 0.6;
  trajectory_static_feature_ = StaticFeature::CalculateStaticFeature(rel_y);

  if (*(++rel_y.rbegin()) - *(++rel_y.begin()) < drive_straight_buffer ||
      (trajectory_static_feature_.std_error <
           FLAGS_nolane_obs_max_lateral_offset_in_straight &&
       trajectory_static_feature_.kurtosis < 0.0 &&
       trajectory_static_feature_.skewness < 0.9)) {
    trajectory_type_ = TrajectoryType::Straight;
  } else {
    trajectory_type_ = TrajectoryType::Curve;
  }
}

void FilterObstacleFromMultiTrajectory(obstacles_ptr_list* const obstacles) {
  LogProcess::LogProc(__func__);
  obstacles->sort([](auto& lhs, auto& rhs) {
    return std::get<0>(lhs->GetStateSl().back()).s() <
           std::get<0>(rhs->GetStateSl().back()).s();
  });

  std::vector<std::shared_ptr<ObstaclePerceptionNoLane>> rev_list;
  rev_list.push_back(*obstacles->begin());
  SLPoint prev_traj_back_sl;
  SLPoint temp_traj_back_sl;
  for (auto it = ++obstacles->begin(); it != obstacles->end(); ++it) {
    if (DefinitelyLessEqual(
            fabs(std::get<0>(rev_list.back()->GetStateSl().back()).l() -
                 std::get<0>((*it)->GetStateSl().back()).l()),
            LaneWidth * 0.8)) {
      rev_list.push_back(*it);
    }
  }

  // TODO(lp) multi trajectory how to choose in ego lane???
  // lp: precision???

  obstacles->assign(rev_list.begin(), rev_list.end());
}

}  // namespace nolane
}  // namespace planning
}  // namespace TL
