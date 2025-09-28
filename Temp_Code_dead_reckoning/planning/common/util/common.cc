/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "planning/common/util/common.h"

#include <cstddef>
#include <limits>
#include <memory>
#include "common/util/string_util.h"
#include "common/util/util.h"

namespace TL {
namespace planning {
namespace util {

using TL::hdmap::RoadSection;

/*
 * @brief: build virtual obstacle of stop wall, and add STOP decision
 */
int BuildStopDecision(const std::string& stop_wall_id, const double stop_line_s,
                      const double stop_distance,
                      const StopReasonCode& stop_reason_code,
                      const std::vector<std::string>& wait_for_obstacles,
                      const std::string& decision_tag, Frame* const frame,
                      ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  // check
  const auto& reference_line = reference_line_info->reference_line();
  if (stop_line_s <= 0.0) {
    ADEBUG << "stop_line_s[" << stop_line_s << "] is not on reference line";
    return 0;
  }

  // create virtual stop wall
  double obj_start_s = stop_line_s;
  if (!reference_line_info->path_data().frenet_frame_path().is_forward_path()) {
    obj_start_s = stop_line_s - FLAGS_virtual_stop_wall_length;
  }
  const auto obstacle =
      frame->CreateStopObstacle(reference_line_info, stop_wall_id, obj_start_s);
  if (obstacle == nullptr) {
    ADEBUG << "Failed to create obstacle [" << stop_wall_id << "]";
    return -1;
  }
  const Obstacle* stop_wall = reference_line_info->AddObstacle(obstacle);
  if (stop_wall == nullptr) {
    ADEBUG << "Failed to add obstacle[" << stop_wall_id << "]";
    return -1;
  }

  // build stop decision
  double stop_s = stop_line_s - stop_distance;
  if (!reference_line_info->path_data().frenet_frame_path().is_forward_path()) {
    stop_s = stop_line_s + stop_distance;
  }
  const auto& stop_point =
      reference_line.GetReferencePointForGreaterThanRefMaxS(stop_s);
  const double stop_heading =
      reference_line.GetReferencePoint(stop_s).heading();

  ObjectDecisionType stop;
  auto* stop_decision = stop.mutable_stop();
  stop_decision->set_reason_code(stop_reason_code);
  stop_decision->set_distance_s(-stop_distance);
  stop_decision->set_stop_heading(stop_heading);
  stop_decision->mutable_stop_point()->set_x(stop_point.x());
  stop_decision->mutable_stop_point()->set_y(stop_point.y());
  stop_decision->mutable_stop_point()->set_z(0.0);

  for (const auto& wait_for_obstacle : wait_for_obstacles) {
    stop_decision->add_wait_for_obstacle(wait_for_obstacle);
  }

  auto* path_decision = reference_line_info->path_decision();
  path_decision->AddLongitudinalDecision(decision_tag, stop_wall->Id(), stop);

  return 0;
}

int BuildStopDecision(const std::string& stop_wall_id,
                      const std::string& lane_id, const double lane_s,
                      const double stop_distance,
                      const StopReasonCode& stop_reason_code,
                      const std::vector<std::string>& wait_for_obstacles,
                      const std::string& decision_tag, Frame* const frame,
                      ReferenceLineInfo* const reference_line_info) {
  UNUSED(wait_for_obstacles);
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);
  UNUSED(stop_reason_code);
  const auto& reference_line = reference_line_info->reference_line();

  // create virtual stop wall
  const auto obstacle =
      frame->CreateStopObstacle(stop_wall_id, lane_id, lane_s);
  if (obstacle == nullptr) {
    ADEBUG << "Failed to create obstacle [" << stop_wall_id << "]";
    return -1;
  }

  const Obstacle* stop_wall = reference_line_info->AddObstacle(obstacle);
  if (stop_wall == nullptr) {
    ADEBUG << "Failed to create obstacle for: " << stop_wall_id;
    return -1;
  }

  const auto& stop_wall_box = stop_wall->PerceptionBoundingBox();
  if (!reference_line.IsOnLane(stop_wall_box.center())) {
    ADEBUG << "stop point is not on lane. SKIP STOP decision";
    return 0;
  }

  // build stop decision
  auto stop_point = reference_line.GetReferencePoint(
      stop_wall->PerceptionSLBoundary().start_s() - stop_distance);

  ObjectDecisionType stop;
  auto* stop_decision = stop.mutable_stop();
  stop_decision->set_reason_code(stop_reason_code);
  stop_decision->set_distance_s(-stop_distance);
  stop_decision->set_stop_heading(stop_point.heading());
  stop_decision->mutable_stop_point()->set_x(stop_point.x());
  stop_decision->mutable_stop_point()->set_y(stop_point.y());
  stop_decision->mutable_stop_point()->set_z(0.0);

  auto* path_decision = reference_line_info->path_decision();
  path_decision->AddLongitudinalDecision(decision_tag, stop_wall->Id(), stop);

  return 0;
}

bool GetStateAtMinJerk(const double init_v, const double init_a,
                       const double min_v, const double max_v,
                       const double max_dece, const double jerk_min,
                       const double max_size, const double delta_size,
                       const double start_s,
                       std::vector<std::vector<double>>* const vec_vec_state) {
  if (vec_vec_state == nullptr) {
    ADEBUG << "vec_vec_states is null!";
    return false;
  }

  bool init_point_right = true;
  double last_s = start_s;
  double last_v = init_v;
  double last_a = init_a;
  const auto const_jerk_t = (init_a > max_dece)
                                ? (max_dece - init_a) / jerk_min
                                : std::numeric_limits<double>::lowest();

  std::tuple<double, double, double, double> state_t;
  const auto delta_step = delta_size;
  const auto step = static_cast<int>(std::round(delta_size / delta_step));
  const auto max_step = static_cast<int>(std::round(max_size / delta_step));
  const int vector_size =
      static_cast<int>(std::floor((max_step - step) / step)) + 2;

  std::vector<double> vec_t;
  vec_t.reserve(vector_size);
  vec_t.emplace_back(0.0);
  std::vector<double> vec_s;
  vec_s.reserve(vector_size);
  vec_s.emplace_back(0.0);
  std::vector<double> vec_v;
  vec_v.reserve(vector_size);
  vec_v.emplace_back(init_v);
  std::vector<double> vec_a;
  vec_a.reserve(vector_size);
  vec_a.emplace_back(init_a);

  for (int i = step; i <= max_step; i += step) {
    const auto t = i * delta_step;
    double acce = delta_size * jerk_min + last_a;
    std::get<3>(state_t) = std::fmax(acce, max_dece);
    const auto t1 =
        common::math::Clamp(const_jerk_t - t + delta_size, 0.0, delta_size);
    const auto t2 = common::math::Clamp(t - const_jerk_t, 0.0, delta_size);
    std::get<2>(state_t) = std::fmax(
        last_v + last_a * t1 + 0.5 * jerk_min * t1 * t1 + max_dece * t2, min_v);
    std::get<1>(state_t) =
        last_s + last_v * delta_size +
        1.0 / 3.0 * last_a * delta_size * delta_size +
        1.0 / 6.0 * std::get<3>(state_t) * delta_size * delta_size;
    std::get<0>(state_t) = t;

    last_a = std::get<3>(state_t);
    last_v = std::get<2>(state_t);
    last_s = std::get<1>(state_t);

    if (std::get<2>(state_t) <= min_v + TL::common::math::kMathEpsilon &&
        min_v < TL::common::math::kMathEpsilon) {
      last_v = min_v;
      last_s = vec_s.back();
    }

    vec_t.emplace_back(t);
    vec_s.emplace_back(last_s);
    vec_v.emplace_back(last_v);
    vec_a.emplace_back(last_a);

    if (std::get<2>(state_t) > max_v) {
      ADEBUG << "init point error at min jerk.  init_v:" << init_v
             << "   init_a:" << init_a << "   min_v:" << min_v
             << "   max_v:" << max_v << "   max_dece:" << max_dece
             << "   jerk_min:" << jerk_min;
      std::string msg;
      TL::common::util::vec2str(vec_t, &msg);
      AERROR << "t_min =" << msg;
      TL::common::util::vec2str(vec_s, &msg);
      AERROR << "s_min =" << msg;
      TL::common::util::vec2str(vec_v, &msg);
      AERROR << "v_min =" << msg;
      TL::common::util::vec2str(vec_a, &msg);
      AERROR << "a_min =" << msg;
      init_point_right = false;
      break;
    }
  }
  vec_vec_state->clear();
  if (vec_t.size() == vec_s.size() && vec_s.size() == vec_v.size() &&
      vec_v.size() == vec_a.size() && !vec_t.empty()) {
    vec_vec_state->emplace_back(std::move(vec_t));
    vec_vec_state->emplace_back(std::move(vec_s));
    vec_vec_state->emplace_back(std::move(vec_v));
    vec_vec_state->emplace_back(std::move(vec_a));
  } else {
    ADEBUG << "vec_t vec_s vec_v vec_a size are not equal or are empty."
           << "  vec_t size:" << vec_t.size() << "  vec_s size:" << vec_s.size()
           << "  vec_v size:" << vec_v.size()
           << "  vec_a size:" << vec_a.size();
  }
  return init_point_right;
}

bool GetStateAtMaxJerk(const double init_v, const double init_a,
                       const double min_v, const double max_v,
                       const double max_acce, const double jerk_max,
                       const double max_size, const double delta_size,
                       const double start_s,
                       std::vector<std::vector<double>>* const vec_vec_state) {
  if (vec_vec_state == nullptr) {
    ADEBUG << "vec_vec_states is null!";
    return false;
  }

  bool init_point_right = true;
  double last_s = start_s;
  double last_v = init_v;
  double last_a = init_a;

  const auto const_jerk_t = (init_a < max_acce)
                                ? (max_acce - init_a) / jerk_max
                                : std::numeric_limits<double>::lowest();

  std::tuple<double, double, double, double> state_t;
  const auto delta_step = delta_size;
  const auto step = static_cast<int>(std::round(delta_size / delta_step));
  const auto max_step = static_cast<int>(std::round(max_size / delta_step));
  const int vector_size =
      static_cast<int>(std::floor((max_step - step) / step)) + 2;

  std::vector<double> vec_t;
  vec_t.reserve(vector_size);
  vec_t.emplace_back(0.0);
  std::vector<double> vec_s;
  vec_s.reserve(vector_size);
  vec_s.emplace_back(0.0);
  std::vector<double> vec_v;
  vec_v.reserve(vector_size);
  vec_v.emplace_back(init_v);
  std::vector<double> vec_a;
  vec_a.reserve(vector_size);
  vec_a.emplace_back(init_a);

  for (int i = step; i <= max_step; i += step) {
    const auto t = i * delta_step;
    double acce = delta_size * jerk_max + last_a;
    std::get<3>(state_t) = std::fmin(acce, max_acce);
    const auto t1 =
        common::math::Clamp(const_jerk_t - t + delta_size, 0.0, delta_size);
    const auto t2 = common::math::Clamp(t - const_jerk_t, 0.0, delta_size);
    std::get<2>(state_t) = std::fmax(
        last_v + last_a * t1 + 0.5 * jerk_max * t1 * t1 + max_acce * t2, min_v);
    std::get<1>(state_t) =
        last_s + last_v * delta_size +
        1.0 / 3.0 * last_a * delta_size * delta_size +
        1.0 / 6.0 * std::get<3>(state_t) * delta_size * delta_size;
    std::get<0>(state_t) = t;

    last_a = std::get<3>(state_t);
    last_v = std::get<2>(state_t);
    last_s = std::get<1>(state_t);

    if (std::get<2>(state_t) <= min_v + TL::common::math::kMathEpsilon &&
        min_v < TL::common::math::kMathEpsilon) {
      last_v = min_v;
      last_s = vec_s.back();
    }

    vec_t.emplace_back(t);
    vec_s.emplace_back(last_s);
    vec_v.emplace_back(last_v);
    vec_a.emplace_back(last_a);

    if (std::get<2>(state_t) < min_v) {
      ADEBUG << "init point error at min jerk.  init_v:" << init_v
             << "   init_a:" << init_a << "   min_v:" << min_v
             << "   max_v:" << max_v << "   max_acce:" << max_acce
             << "   jerk_max:" << jerk_max;
      std::string msg;
      TL::common::util::vec2str(vec_t, &msg);
      AERROR << "t_max =" << msg;
      TL::common::util::vec2str(vec_s, &msg);
      AERROR << "s_max =" << msg;
      TL::common::util::vec2str(vec_v, &msg);
      AERROR << "v_max =" << msg;
      TL::common::util::vec2str(vec_a, &msg);
      AERROR << "a_max =" << msg;
      init_point_right = false;
      break;
    }
  }
  vec_vec_state->clear();
  if (vec_t.size() == vec_s.size() && vec_s.size() == vec_v.size() &&
      vec_v.size() == vec_a.size() && !vec_t.empty()) {
    vec_vec_state->emplace_back(std::move(vec_t));
    vec_vec_state->emplace_back(std::move(vec_s));
    vec_vec_state->emplace_back(std::move(vec_v));
    vec_vec_state->emplace_back(std::move(vec_a));
  } else {
    ADEBUG << "vec_t vec_s vec_v vec_a size are not equal or are empty."
           << "  vec_t size:" << vec_t.size() << "  vec_s size:" << vec_s.size()
           << "  vec_v size:" << vec_v.size()
           << "  vec_a size:" << vec_a.size();
  }
  return init_point_right;
}

bool IsSameReferenceLine(const Frame& frame_curr,
                         const ReferenceLineInfo& reference_line_info,
                         const ReferenceLine& ref_prev) {
  if (frame_curr.reference_line_info().size() != 2) {
    ADEBUG << "current reference line is not 2 in reference_line_info.";
    return false;
  }
  const auto& ref_line_other =
      &frame_curr.reference_line_info().front() == &reference_line_info
          ? frame_curr.reference_line_info().back().reference_line()
          : frame_curr.reference_line_info().front().reference_line();
  const auto& ref_line_cur = reference_line_info.reference_line();

  const auto& accu_s = ref_line_other.map_path().accumulated_s();
  const size_t accu_s_size = accu_s.size();
  const auto& path_points = ref_line_other.map_path().path_points();
  if (accu_s_size != path_points.size()) {
    ADEBUG << "accu_s_size is not equal to path_points.size().";
    return false;
  }
  const double interval_dis = 2;
  const int interval_size =
      static_cast<int>(interval_dis / (accu_s.at(1) - accu_s.at(0)));

  std::vector<double> l_rep(accu_s_size, -100.0);

  bool is_find_max_point = false;
  SLPoint sl_pos;
  Vec2d map_pos_xy;
  for (size_t i = 0; i < accu_s_size; i += interval_size) {
    map_pos_xy.set_x(path_points.at(i).x());
    map_pos_xy.set_y(path_points.at(i).y());
    ref_line_cur.XYToSL(map_pos_xy, &sl_pos);
    if (sl_pos.l() >= 2.0) {
      is_find_max_point = true;
      break;
    }
    l_rep.at(i) = fabs(sl_pos.l());
  }
  if (!is_find_max_point) {
    int max_l = static_cast<int>(std::max_element(l_rep.begin(), l_rep.end()) -
                                 l_rep.begin());
    l_rep.clear();
    l_rep.resize(accu_s_size, -100.0);
    size_t start_index = max_l - interval_size + 1;
    start_index = start_index >= 0 ? start_index : 0;
    size_t end_index = max_l + interval_size - 1;
    end_index = end_index < accu_s_size ? end_index : accu_s_size - 1;
    for (size_t i = start_index; i <= end_index; ++i) {
      map_pos_xy.set_x(path_points.at(i).x());
      map_pos_xy.set_y(path_points.at(i).y());
      ref_line_cur.XYToSL(map_pos_xy, &sl_pos);
      l_rep.at(i) = fabs(sl_pos.l());
    }
    max_l = static_cast<int>(std::max_element(l_rep.begin(), l_rep.end()) -
                             l_rep.begin());
    if (TL::common::math::double_type::SeemsEqual(l_rep.at(max_l), 0.0)) {
      return true;
    }
    map_pos_xy.set_x(path_points.at(max_l).x());
    map_pos_xy.set_y(path_points.at(max_l).y());
    ref_line_cur.XYToSL(map_pos_xy, &sl_pos);
  }
  SLPoint sl_in_prev_ref;
  ref_prev.XYToSL(map_pos_xy, &sl_in_prev_ref);
  return sl_in_prev_ref.l() > sl_pos.l() / 2.0;
}

bool IsRoadCurvedSection(const hdmap::LaneWaypoint& curr_waypoint) {
  return curr_waypoint.lane->GetSectionType() == RoadSection::Ramp ||
         curr_waypoint.lane->GetSectionType() ==
             RoadSection::RoundaboutCircle ||
         curr_waypoint.lane->GetSectionType() == RoadSection::SlipRoad ||
         curr_waypoint.lane->GetSectionType() == RoadSection::Service ||
         curr_waypoint.lane->GetSectionType() == RoadSection::JCT;
}

bool IsNormalTurn(const hdmap::LaneWaypoint& curr_waypoint) {
  if (curr_waypoint.lane == nullptr) {
    ADEBUG << "Input pointer is nullptr.";
    return false;
  }

  auto lane_turn_type = curr_waypoint.lane->lane().has_turn()
                            ? curr_waypoint.lane->lane().turn()
                            : hdmap::Lane::NO_TURN;
  return lane_turn_type == hdmap::Lane::LEFT_TURN ||
         lane_turn_type == hdmap::Lane::LEFT_FRONT ||
         lane_turn_type == hdmap::Lane::RIGHT_TURN ||
         lane_turn_type == hdmap::Lane::RIRHT_FRONT;
}

bool IsNormalTurn(const hdmap::LaneWaypoint& curr_waypoint,
                  bool* const is_left_normal_turn,
                  bool* const is_right_normal_turn) {
  if (is_left_normal_turn == nullptr || is_right_normal_turn == nullptr ||
      curr_waypoint.lane == nullptr) {
    ADEBUG << "Input pointer is nullptr.";
    return false;
  }

  auto lane_turn_type = curr_waypoint.lane->lane().has_turn()
                            ? curr_waypoint.lane->lane().turn()
                            : hdmap::Lane::NO_TURN;
  *is_left_normal_turn = lane_turn_type == hdmap::Lane::LEFT_TURN ||
                         lane_turn_type == hdmap::Lane::LEFT_FRONT;
  *is_right_normal_turn = lane_turn_type == hdmap::Lane::RIGHT_TURN ||
                          lane_turn_type == hdmap::Lane::RIRHT_FRONT;

  return *is_left_normal_turn || *is_right_normal_turn;
}

bool IsOutermostLane(const TL::hdmap::LaneInfoConstPtr& lane,
                     const std::shared_ptr<hdmap::PncMap>& pnc_map) {
  if (lane == nullptr || pnc_map == nullptr) {
    return false;
  }
  const auto is_left = lane->lane().left_neighbor_forward_lane_id().empty() &&
                       lane->lane().left_neighbor_reverse_lane_id().empty();
  auto is_right = lane->lane().right_neighbor_forward_lane_id().empty() &&
                  lane->lane().right_neighbor_reverse_lane_id().empty();
  if (!is_right && !lane->lane().right_neighbor_forward_lane_id().empty()) {
    const auto right_neighbor_lane = pnc_map->GetLaneType(
        lane->lane().right_neighbor_forward_lane_id().at(0).id());
    is_right = right_neighbor_lane != nullptr &&
               right_neighbor_lane->lane().type() == hdmap::Lane::BIKING;
  }
  return (is_left || is_right);
}

double GetDkappaSpeedLimit(const common::PathPoint& pre_point,
                           const common::PathPoint& point) {
  static constexpr double kEpsilon = 0.0001;
  const auto& vehicle_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  const auto delta_steer_angle =
      fabs(atan(vehicle_param.wheel_base() * point.kappa()) -
           atan(vehicle_param.wheel_base() * pre_point.kappa())) *
      vehicle_param.steer_ratio() * 180.0 / M_PI;
  const auto delta_s = fabs(point.s() - pre_point.s());
  if (!vehicle_param.has_steer_wheel_speed_segment() ||
      vehicle_param.steer_wheel_speed_segment()
          .vehicle_speed_segment()
          .empty() ||
      vehicle_param.steer_wheel_speed_segment().vehicle_speed_segment_size() !=
          vehicle_param.steer_wheel_speed_segment()
              .steering_wheel_speed_limit_segment_size()) {
    return FLAGS_speed_upper_bound;
  }
  const auto& speed_segments =
      vehicle_param.steer_wheel_speed_segment().vehicle_speed_segment();
  const auto& steer_speed_segments = vehicle_param.steer_wheel_speed_segment()
                                         .steering_wheel_speed_limit_segment();
  for (int i = speed_segments.size() - 1; i >= 0; --i) {
    double t = delta_s / (speed_segments[i] + kEpsilon);
    double min_steer_speed = delta_steer_angle / (t + kEpsilon);
    if (steer_speed_segments[i] > min_steer_speed) {
      return speed_segments[i];
    }
  }

  return FLAGS_speed_upper_bound;
}

}  // namespace util
}  // namespace planning
}  // namespace TL
