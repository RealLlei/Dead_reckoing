/*
 * Copyright (c) TL auto Co., Ltd. 2023-2024. All rights reserved.
 */

#include "planning/localview/lane_line_builder/obstacle_following_lane_line/missile_mode/obstacles_state.h"
#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <utility>

namespace TL {
namespace planning {
namespace missilelane {
constexpr double k_TJA_L_TrailOffsetThreshold = 0.5;
constexpr double k_TJA_D_MinCutInTargetSeparationSquared = 16;
constexpr double k_TJA_An_TrailAngleThreshold = 0.2;

double CalculateLanemarkerY(const double distance,
                            const TL::perception::LaneMarker& lane_marker) {
  double x2 = distance * distance;
  double x3 = x2 * distance;
  return lane_marker.c0_position() + lane_marker.c1_heading_angle() * distance +
         lane_marker.c2_curvature() * x2 +
         lane_marker.c3_curvature_derivative() * x3;
}

void MissileObs::ObsInit(const perception::PerceptionObstacle& obs) {
  std::vector<double> den(3, 0.0);
  std::vector<double> num(3, 0.0);
  TL::common::LpfCoefficients(0.03, 10, &den, &num);  // ts_ , cutoff_freq;
  heading_filter.set_coefficients(den, num);
  deq_obs.push_front(obs);
  count++;
  is_valid = false;
  double heading =
      obs.velocity_flu().x() > 3.0
          ? std::atan2(obs.velocity_flu().y(), obs.velocity_flu().x())
          : obs.theta_flu();
  average_heading = heading_filter.Filter(heading);
}

void MissileObs::ObsUpdate(const perception::PerceptionObstacle& obs) {
  if (deq_obs.size() >= 3) {
    deq_obs.pop_back();
  }
  deq_obs.push_front(obs);
  count++;
  if (count > 30) {
    is_valid = true;
  }
  double heading =
      obs.velocity_flu().x() > 3.0
          ? std::atan2(obs.velocity_flu().y(), obs.velocity_flu().x())
          : obs.theta_flu();
  average_heading = heading_filter.Filter(heading);
}

ObstaclesState::ObstaclesState(
    const planning::PerceptionMapConfig& config,
    const std::shared_ptr<MissileVehicleState>& vehicle_state)
    : config_(config), vehicle_state_(vehicle_state) {}

Status ObstaclesState::Init() {
  UNUSED(config_);
  perception::PerceptionObstacle obs;
  obs.set_id(-1);
  no_valid_obs_.is_valid = false;
  no_valid_obs_.deq_obs.push_front(obs);
  return Status::OK();
}

bool ObstaclesState::Update(
    const std::shared_ptr<const perception::PerceptionObstacles>&
        perception_obs_ptr,
    const std::vector<int32_t>& target_ids) {
  has_no_target_obs_ = target_ids.empty() || target_ids.front() == -1 ||
                       perception_obs_ptr->perception_obstacle().empty();
  target_ids_ = target_ids;
  left_target_ids_.clear();
  right_target_ids_.clear();
  for (size_t i = 0; i < target_ids.size(); ++i) {
    perception::PerceptionObstacle target_obs;
    bool has_obs = false;
    for (const auto& obs : perception_obs_ptr->perception_obstacle()) {
      if (obs.id() == target_ids[i]) {
        if (obs.type() != perception::PerceptionObstacle::Type::
                              PerceptionObstacle_Type_VEHICLE) {
          target_ids_[i] = -1;
        } else {
          target_obs = obs;
          has_obs = true;
        }
        break;
      }
    }
    // cruise_target_id_ = {
    //     [0]:前方目标1,      [1]: 前方目标2,   [2]:左前方目标1,    [3]:左前方目标2,
    //     [4]:右前方目标1,    [5]:右前方目标2,   [6]:cut_in,       [7]:左侧方目标1,
    //     [8]:左侧方目标2,    [9]:右侧方目标1,   [10]:右侧方目标2,   [11]:左侧后方目标1,
    //     [12]:左侧后方目标2, [13]:右侧后方目标1, [14]:右侧后方目标2, [15]:后方目标};
    if (target_ids[i] != -1 && has_obs) {
      // 右侧ids
      if (i == 4 || i == 5 || i == 9 || i == 10 || i == 13 || i == 14) {
        right_target_ids_.emplace_back(target_ids[i]);
      }
      // 左侧ids
      if (i == 2 || i == 3 || i == 7 || i == 8 || i == 11 || i == 12) {
        left_target_ids_.emplace_back(target_ids[i]);
      }
      if (unmap_obs_.find(target_ids[i]) != unmap_obs_.end()) {
        unmap_obs_.find(target_ids[i])->second.ObsUpdate(target_obs);
      } else {
        MissileObs missile_obs;
        missile_obs.ObsInit(target_obs);
        unmap_obs_[target_ids[i]] = missile_obs;
      }
    }
  }
  // 删除map中非target_ids的数据
  for (auto it = unmap_obs_.begin(); it != unmap_obs_.end();) {
    bool has_obs{false};
    for (const auto& id : target_ids) {
      if (it->first == id || it->first == target_id_history_) {
        has_obs = true;
        break;
      }
    }
    if (!has_obs) {
      it = unmap_obs_.erase(it);
    } else {
      ++it;
    }
  }
  return true;
}

// cruise_target_id_ = {cip, sip, left1, left2, right1, right2, cut_in}
const MissileObs& ObstaclesState::TargetObs(
    const TL::perception::LaneMarker& vehicle_lanemarker,
    const std::shared_ptr<LocalView>& local_view) {
  if (target_ids_.size() < 2 ||
      unmap_obs_.find(target_ids_[0]) == unmap_obs_.end() ||
      !local_view->HasChassis()) {
    has_no_target_obs_ = true;
    ADEBUG << " no_TargetObs!!";
    return no_valid_obs_;
  }

  double vehicle_speed = vehicle_state_->spd();
  auto fusion_kappa = std::fabs(local_view->GetSubjectKappa());
  double original_steering_percentage =
      local_view->GetChassis()->has_steering_percentage()
          ? local_view->GetChassis()->steering_percentage()
          : 0.0;
  double steer_angle =
      std::fabs(original_steering_percentage / 100 * (8.0345 / M_PI * 180));
  const bool is_check_bl =
      fusion_kappa > 1.0 / 300.0 && steer_angle > 20.0 && (vehicle_speed < 1.5);
  ADEBUG << ", fusion_kappa: " << fusion_kappa
         << " ,steer_angle: " << steer_angle
         << " ,is_check_bl: " << is_check_bl;

  // 是否有前前车,若有则计算CIP和SIP的切线方程,获取CIP和SIP之间的偏移和角度
  // 再运用直线法向量夹角余弦求得直线夹角
  if (target_ids_[1] != -1 && (!is_check_bl)) {
    auto cip_it = unmap_obs_.find(target_ids_[0]);
    auto sip_it = unmap_obs_.find(target_ids_[1]);
    // 检查cip_it和sip_it是否有效
    if (cip_it == unmap_obs_.end() || sip_it == unmap_obs_.end()) {
      ADEBUG << "Target ID not found in unmap_obs_";
      return no_valid_obs_;
    }
    const auto& targets_cip = cip_it->second.deq_obs;
    const auto& targets_sip = sip_it->second.deq_obs;
    if (targets_cip.empty() || targets_sip.empty()) {
      has_no_target_obs_ = true;
      ADEBUG << " no_TargetObs!!";
      return no_valid_obs_;
    }
    const auto& target_cip = targets_cip.front();
    const auto& target_sip = targets_sip.front();
    double obs_x1 = target_cip.position_flu().x();
    double obs_y1 = target_cip.position_flu().y();
    double cip_heading = target_cip.velocity_flu().x() > 3.0
                             ? std::atan2(target_cip.velocity_flu().y(),
                                          target_cip.velocity_flu().x())
                             : target_cip.theta_flu();
    auto [cip_a, cip_b, cip_c] = TjaGetTangent(obs_x1, obs_y1, cip_heading);
    ADEBUG << "cip_heading" << cip_heading;
    double obs_x2 = target_sip.position_flu().x();
    double obs_y2 = target_sip.position_flu().y();
    double sip_heading = target_sip.velocity_flu().x() > 3.0
                             ? std::atan2(target_sip.velocity_flu().y(),
                                          target_sip.velocity_flu().x())
                             : target_sip.theta_flu();
    ADEBUG << "SIP_HEADING" << sip_heading;
    auto [sip_a, sip_b, sip_c] = TjaGetTangent(obs_x2, obs_y2, sip_heading);
    double offset = sip_a * obs_x1 + sip_b * obs_y1 + sip_c;
    double dotProduct = sip_a * cip_a + sip_b * cip_b;
    // double magnitudeProduct = std::sqrt(vec1.x * vec1.x + vec1.y * vec1.y) *
    //                           std::sqrt(vec2.x * vec2.x + vec2.y * vec2.y);
    double angle = acos(fmin(fmax((dotProduct), -1), 1));
    ADEBUG << "sip_id: " << target_ids_[1] << ",obs_y2:" << obs_y2
           << " Sip_A: " << sip_a << ",Sip_B: " << sip_b << ",sip_C: " << sip_c
           << "cip_id: " << target_ids_[0] << ",obs_y1: " << obs_y1
           << " cip_A: " << cip_a << " cip_B: " << cip_b << " cip_c: " << cip_c;
    ADEBUG << "dotProduct" << dotProduct;
    // 判断当前巡航cip目标是否在自车道内
    is_cip_in_lane_ = (abs(offset) < k_TJA_L_TrailOffsetThreshold) &&
                      (abs(angle) < k_TJA_An_TrailAngleThreshold);
    // 判断目标分离度：计算CIP和SIP目标之间的水平距离（deltaX）和垂直距离（deltaY），然后计算它们之间的平方距离
    double deltaX = obs_x1 - obs_x2;
    double deltaY = obs_y1 - obs_y2;
    double targetSeparationSquared = deltaX * deltaX + deltaY * deltaY;
    if (targetSeparationSquared > k_TJA_D_MinCutInTargetSeparationSquared) {
      is_cut_in_ = true;
      ADEBUG << "CutIn!!";
    }
    is_cip_cutout_ = (offset > 1) && (cip_heading > 0.15) && (angle > 0.15);

    ADEBUG << " offset: " << offset << ",angle:" << angle << ",cipinlane"
           << is_cip_in_lane_ << ",targetSeparationSquared"
           << targetSeparationSquared << " ,is_cip_cutout: " << is_cip_cutout_;
  }

  // selectedTargetId 当CIP在车道内时，选择CIP；当CIP不存在或者CUT OUT时，选择SIP
  //  && (target_ids_[1] == target_id_history_)
  if (target_ids_[1] != -1 &&
      ((!is_cip_in_lane_ && target_ids_[1] == target_id_history_) ||
       (is_cip_cutout_ && target_ids_.front() == target_id_history_))) {
    selected_target_id = target_ids_[1];
  } else {
    selected_target_id = target_ids_[0];
  }
  ADEBUG << "selected_target_id: " << selected_target_id
         << " ,target_id_history_: " << target_id_history_;
  if (selected_target_id != target_id_history_ &&
      vehicle_state_->is_missile_mode_active() &&
      unmap_obs_.find(selected_target_id) != unmap_obs_.end() &&
      unmap_obs_.find(target_id_history_) != unmap_obs_.end() &&
      (!is_cip_cutout_)) {
    const auto& targets_obs =
        unmap_obs_.find(selected_target_id)->second.deq_obs;
    if (targets_obs.empty()) {
      has_no_target_obs_ = true;
      ADEBUG << " no_TargetObs!!";
      return no_valid_obs_;
    }
    const auto& target_obs = targets_obs.front();
    auto y =
        target_obs.position_flu().y() -
        CalculateLanemarkerY(target_obs.position_flu().x(), vehicle_lanemarker);
    const auto& targets_obs_his =
        unmap_obs_.find(target_id_history_)->second.deq_obs;
    if (targets_obs_his.empty()) {
      has_no_target_obs_ = true;
      ADEBUG << " no_TargetObs!!";
      return no_valid_obs_;
    }
    const auto& target_obs_his = targets_obs_his.front();
    double y_history = target_obs_his.position_flu().y() -
                       CalculateLanemarkerY(target_obs_his.position_flu().x(),
                                            vehicle_lanemarker);

    bool is_good_target_history = std::fabs(y_history) < 1.5;
    bool is_good_target =
        std::fabs(y) < 0.8 ||
        (is_good_target_history && std::fabs(y_history - y) < 0.5) ||
        (is_cip_in_lane_ && std::fabs(target_obs.position_flu().y()) < 1);
    // AERROR << "is_good_target" << is_good_target << " y:" << y
    //        << " ,CalculateLanemarkerY: "
    //        << CalculateLanemarkerY(target_obs.position_flu().x(),
    //                                vehicle_lanemarker)
    //        << ",is_good_target_history" << is_good_target_history
    //        << " ,his_calc_Y: "
    //        << CalculateLanemarkerY(target_obs_his.position_flu().x(),
    //                                vehicle_lanemarker)
    //        << ", is_cip_in_lane_:" << is_cip_in_lane_;

    if (!is_good_target && is_good_target_history) {
      selected_target_id = target_id_history_;
    }
    if (!is_good_target && !is_good_target_history) {
      has_no_target_obs_ = true;
      return no_valid_obs_;
    }
  }
  target_id_history_ = selected_target_id;
  ADEBUG << " selected_target_id: " << selected_target_id
         << " , target_ids_.front: " << target_ids_.front();
  return unmap_obs_.find(selected_target_id)->second;
}

void ObstaclesState::UpdateObsBoundaryPoints() {
  left_boundary_points_.clear();
  right_boundary_points_.clear();
  // 修复巡航目标跳遍成3/4目标导致的跟车目标切换，进而边界检查没通过
  auto is_change_to_right = std::find(
      right_target_ids_.begin(), right_target_ids_.end(), target_id_history_);
  auto is_change_to_left = std::find(
      left_target_ids_.begin(), left_target_ids_.end(), target_id_history_);

  if (has_no_target_obs_ ||
      unmap_obs_.find(target_id_history_) == unmap_obs_.end()) {
    return;
  }

  const double start_x = -10.0;
  const double end_x = unmap_obs_.find(target_id_history_)
                           ->second.deq_obs.front()
                           .position_flu()
                           .x();
  // 收集左侧点
  TL::common::Point3D point_min_left{};
  double min_left_y{100.0};
  for (const auto& id : left_target_ids_) {
    if (unmap_obs_.find(id) == unmap_obs_.end() ||
        is_change_to_right != right_target_ids_.end() ||
        is_change_to_left != left_target_ids_.end()) {
      ADEBUG << "is curise_id change cause ";
      continue;
    }
    // 车辆未在范围内，不考虑
    const auto& obs = unmap_obs_.find(id)->second.deq_obs.front();
    if (obs.position_flu().x() > end_x || obs.position_flu().x() < start_x) {
      continue;
    }
    // TTC
    constexpr double kAdcwithObsMinTTC = 0.0001;
    constexpr double kAdcwithObsMaxTTC = 1000.0;
    constexpr double kAdcObsDefaultTTC = 1000.0;
    constexpr double kAdcObsMinDeltaSpeed = 0.0001;
    double delta_s = obs.position_flu().x();
    double delta_v = obs.velocity_flu().x() - vehicle_state_->spd();
    double front_obs_ttc = kAdcObsDefaultTTC;
    if (delta_v < kAdcObsMinDeltaSpeed) {
      front_obs_ttc = delta_s / abs(delta_v);
    }
    if (front_obs_ttc < kAdcwithObsMinTTC ||
        front_obs_ttc > kAdcwithObsMaxTTC) {
      front_obs_ttc = kAdcObsDefaultTTC;
    }
    // 车辆在自车后方，且比自车速度慢，不考虑
    if (obs.position_flu().x() < 5.0 &&
        (obs.velocity_flu().x() < 0.5 ||
         (vehicle_state_->spd() - obs.velocity_flu().x()) > 1.0)) {
      continue;
    }
    // TODO(TL) 后面可做距离插值表
    // 车辆在自车前方，且比自车速度快，不考虑
    if (obs.position_flu().x() > 15.0 ||
        (obs.position_flu().x() > 10.0 &&
         (vehicle_state_->spd() - obs.velocity_flu().x() < 1.0)) ||
        (is_cut_in_ && front_obs_ttc > 3)) {
      ADEBUG << "skip,front_obs_ttc: " << front_obs_ttc;
      continue;
    }
    // 横向允许有0.5m的重合阈值
    TL::common::Point3D point{obs.position_flu()};
    point.set_y(obs.position_flu().y() - obs.width() / 2 + 0.5);
    if (point.y() < min_left_y) {
      min_left_y = point.y();
    }
    left_boundary_points_.emplace_back(point);
  }
  point_min_left.set_x(end_x);
  point_min_left.set_y(min_left_y);
  left_boundary_points_.emplace_back(point_min_left);

  // 收集右侧点
  TL::common::Point3D point_max_right{};
  double max_right_y{-100.0};
  for (const auto& id : right_target_ids_) {
    ADEBUG << "right_target_ids_: " << id;
    if (unmap_obs_.find(id) == unmap_obs_.end() ||
        is_change_to_right != right_target_ids_.end() ||
        is_change_to_left != left_target_ids_.end()) {
      ADEBUG << "is curise_id change cause ";
      continue;
    }
    const auto& obs = unmap_obs_.find(id)->second.deq_obs.front();
    if (obs.position_flu().x() > end_x || obs.position_flu().x() < start_x) {
      continue;
    }
    // 横向允许有0.5m的重合阈值
    TL::common::Point3D point{obs.position_flu()};
    ADEBUG << "id: " << obs.id()
           << "obs.position_flu().x(): " << obs.position_flu().x()
           << " , obs.position_flu().y(): " << obs.position_flu().y()
           << " , obs.width(): " << obs.width();
    point.set_y(obs.position_flu().y() + obs.width() / 2 - 0.5);
    if (point.y() > max_right_y) {
      max_right_y = point.y();
    }
  }
  point_max_right.set_x(end_x);
  point_max_right.set_y(max_right_y);
  right_boundary_points_.emplace_back(point_max_right);
}

bool ObstaclesState::IsObsLanemarkerValid(
    const TL::perception::LaneMarker& vehicle_lanemarker) {
  constexpr double half_lane_width = 1.6;
  UpdateObsBoundaryPoints();
  // return std::all_of(left_boundary_points_.begin(), left_boundary_points_.end(),
  //                    [&vehicle_lanemarker](
  //                        const TL::common::Point3D& point) -> bool {
  //                      return point.y() > CalculateLanemarkerY(
  //                                             point.x(), vehicle_lanemarker) +
  //                                             half_lane_width;
  //                    }) &&
  //        std::all_of(right_boundary_points_.begin(),
  //                    right_boundary_points_.end(),
  //                    [&vehicle_lanemarker](
  //                        const TL::common::Point3D& point) -> bool {
  //                      return point.y() < CalculateLanemarkerY(
  //                                             point.x(), vehicle_lanemarker) -
  //                                             half_lane_width;
  //                    });

  for (const auto& point : left_boundary_points_) {
    double lane_y = CalculateLanemarkerY(point.x(), vehicle_lanemarker);
    if (point.y() < lane_y + half_lane_width) {
      ADEBUG << " point.x: " << point.x() << " , point.y: " << point.y()
             << " , lane_y: " << lane_y;
      return false;
    }
  }

  for (const auto& point : right_boundary_points_) {
    double lane_y = CalculateLanemarkerY(point.x(), vehicle_lanemarker);
    if (point.y() > lane_y - half_lane_width) {
      ADEBUG << " point.x: " << point.x() << " , point.y: " << point.y()
             << " , lane_y: " << lane_y;
      return false;
    }
  }
  return true;
}

std::tuple<double, double, double> ObstaclesState::TjaGetTangent(
    double x, double y, double heading) {
  // 计算切线上的点
  double pointX = x + std::cos(heading);
  double pointY = y + std::sin(heading);

  // 计算直线的系数Ax + By + C = 0
  double A = y - pointY;
  double B = pointX - x;
  double C = (x * pointY - y * pointX);

  // 计算法线向量的长度
  double norm = std::sqrt(A * A + B * B);

  // 归一化系数
  std::tuple<double, double, double> coefficients =
      std::make_tuple(A / norm, B / norm, C / norm);

  return coefficients;
}

}  // namespace missilelane
}  // namespace planning
}  // namespace TL
