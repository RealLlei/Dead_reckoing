/******************************************************************************
 * Copyright (c) TL Technologies Co., Ltd. 2019-2022. All rights reserved.
 * Author: LingPeng
 * Created Time: 2022/04/21
 *****************************************************************************/

#include "planning/localview/lane_line_builder/obstacle_following_lane_line/lane_center_line.h"

#include <deque>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "map/hdmap/path.h"
using TL::common::util::operator+;

namespace TL {
namespace planning {
namespace nolane {
using TL::common::SLPoint;

LaneCenterLine::LaneCenterLine(
    std::vector<Vec2d> points, const EgoVehicleState& ego_state,
    const std::shared_ptr<LaneCenterLine>& previous_lane)
    : ego_position_({ego_state.GetVehicleStatePresent().x(),
                     ego_state.GetVehicleStatePresent().y()}),
      trajectory_points_raw_(std::move(points)),
      previous_lane_center_line_(previous_lane),
      no_lane_map_msg_(nullptr),
      obs_followed_index_({}) {
  ego_sl_pos_.set_s(ego_state.GetVehicleSl().s());
  ego_sl_pos_.set_l(ego_state.GetVehicleSl().l());
  fit_curve_ = std::make_shared<FitManager>(
      FitType::PolyNomial,
      ego_state.GetVehicleStatePresent().linear_velocity());
}

LaneCenterLine::LaneCenterLine(
    std::vector<std::pair<common::SLPoint, Vec2d>> points,
    const EgoVehicleState& ego_state,
    const std::shared_ptr<LaneCenterLine>& previous_lane)
    : ego_position_({ego_state.GetVehicleStatePresent().x(),
                     ego_state.GetVehicleStatePresent().y()}),
      previous_lane_center_line_(previous_lane),
      fit_raw_points_(std::move(points)),
      no_lane_map_msg_(nullptr),
      obs_followed_index_({}) {
  Vec2d pos_sl;
  for (auto& val : fit_raw_points_) {
    pos_sl.set_x(val.first.s());
    pos_sl.set_y(val.first.l());
    trajectory_points_raw_.push_back(pos_sl);
  }
  ego_sl_pos_.set_s(ego_state.GetVehicleSl().s());
  ego_sl_pos_.set_l(ego_state.GetVehicleSl().l());
  fit_curve_ = std::make_shared<FitManager>(
      FitType::PolyNomial,
      ego_state.GetVehicleStatePresent().linear_velocity());
}

LaneCenterLine::LaneCenterLine(const StitchPointInfo& obs_followed_index,
                               const EgoVehicleState& ego_state)
    : obs_followed_index_(obs_followed_index) {
  const std::shared_ptr<ObstaclePerceptionNoLane>& obs_ptr =
      obs_followed_index.obstacle_ptr;
  const std::deque<StateType>& state_history_extend =
      obs_ptr->GetStateHistoryExtended();
  int num_trajectory_ext = state_history_extend.size();
  int num_start = obs_followed_index.index;
  trajectory_points_raw_.clear();
  if (num_trajectory_ext > num_start) {
    trajectory_points_raw_.reserve(num_trajectory_ext - num_start);
  } else {
    AERROR << "points start number error  num_traj_start:["
           << num_trajectory_ext << "," << num_start << "].";
  }
  // lp: 3 order polynomial to connect ego and obstacle trajectory from specific
  // points.
  const double ego_x_raw = ego_state.GetVehicleStatePresent().x();
  const double ego_y_raw = ego_state.GetVehicleStatePresent().y();
  const double ego_theta_raw = ego_state.GetVehicleStatePresent().heading();
  double obs_x = GetExtendStateIndex(state_history_extend.at(num_start)).x;
  double obs_y = GetExtendStateIndex(state_history_extend.at(num_start)).y;
  double obs_theta = state_history_extend.at(num_start).theta;
  if (debug_flag[0]) {
    AERROR << PRECISION(3) << "connect_ego_and_obs  ego_raw_x:" << ego_x_raw
           << "  ego_raw_y:" << ego_y_raw << "  ego_theta_raw:" << ego_theta_raw
           << "  obs_x:" << obs_x << "  obs_y:" << obs_y
           << "  obs_theta:" << obs_theta;
  }

  double ego_x_rel = 0.0;
  double ego_y_rel = 0.0;
  double ego_theta_rel = 0.0;
  CoordinateTransform<2, double> base_system;
  base_system.SetNewBaseVectorPan(ego_theta_raw, {ego_x_raw, ego_y_raw});
  std::vector<double> pos_rel;
  base_system.Origin2NewBasePanRotate({obs_x, obs_y}, &pos_rel);
  const double obs_x_rel = pos_rel.at(0);
  const double obs_y_rel = pos_rel.at(1);
  const double obs_theta_rel = tan(obs_theta - ego_theta_raw);

  std::array<double, 3> ego_rel{ego_x_rel, ego_y_rel, ego_theta_rel};
  std::array<double, 3> obs_rel{obs_x_rel, obs_y_rel, obs_theta_rel};
  auto coef = PolynomialConnectTwoPoint(ego_rel, obs_rel);
  double s_interval = 1.0;
  double euclidean_dis_ego_obs =
      sqrt(pow(ego_x_rel - obs_x_rel, 2.0) + pow(ego_y_rel - obs_y_rel, 2.0));
  int num = static_cast<int>(euclidean_dis_ego_obs / s_interval);
  s_interval = obs_x_rel / num;
  if (num == 0) {
    s_interval = obs_x_rel;
  }
  double x_t = ego_x_rel;
  double y_t = ego_y_rel;
  std::vector<double> pos_rel_to_abs;
  for (int i = 0; i < num; ++i) {
    x_t += s_interval;
    y_t = coef(0, 0) * pow(x_t, 3.0) + coef(1, 0) * pow(x_t, 2.0) +
          coef(2, 0) * x_t + coef(3, 0);
    base_system.NewBase2OriginRotatePan({x_t, y_t}, &pos_rel_to_abs);
    if (debug_flag[20]) {
      AERROR << PRECISION(4) << "x_t:" << x_t << "  y_t:" << y_t << "  pos_xy:["
             << pos_rel_to_abs.at(0) << "," << pos_rel_to_abs.at(1) << "]";
    }
    trajectory_points_raw_.emplace_back(pos_rel_to_abs.at(0),
                                        pos_rel_to_abs.at(1));
  }

  // for (int i = num_start + 1; i < num_trajectory_ext; ++i) {
  //   trajectory_points_raw_.emplace_back(
  //       GetExtendStateIndex(state_history_extend.at(i)).x - ego_x_raw,
  //       GetExtendStateIndex(state_history_extend.at(i)).y - ego_y_raw);
  // }
  if (debug_flag[50]) {
    std::string raw_x = "obs_x_raw=[";
    std::string raw_y = "obs_y_raw=[";
    for (auto& p : trajectory_points_raw_) {
      raw_x += std::to_string(p.x()) + ",";
      raw_y += std::to_string(p.y()) + ",";
    }
    raw_x.back() = ']';
    raw_y.back() = ']';
    AERROR << raw_x;
    AERROR << raw_y;
  }
  double x_s = GetExtendStateIndex(state_history_extend.at(num_start)).x;
  double y_s = GetExtendStateIndex(state_history_extend.at(num_start)).y;
  double theta_s = state_history_extend.at(num_start).theta;
  double x_e = GetExtendStateIndex(state_history_extend.back()).x;
  double y_e = GetExtendStateIndex(state_history_extend.back()).y;
  double theta_e = tan(state_history_extend.back().theta - theta_s);
  base_system.SetNewBaseVectorPan(theta_s, {x_s, y_s});
  std::vector<double> pos_rel_end;
  base_system.Origin2NewBasePanRotate({x_e, y_e}, &pos_rel_end);
  std::array<double, 3> s_state{0.0, 0.0, 0.0};
  std::array<double, 3> e_state{pos_rel_end.at(0), pos_rel_end.at(1), theta_e};
  coef = PolynomialConnectTwoPoint(s_state, e_state);
  std::vector<double> x_vec;
  std::vector<double> y_vec;
  x_vec.reserve(50);
  y_vec.reserve(50);
  double dis_x = e_state[0];
  double delta_x = copysign(0.3, dis_x);
  double temp_x = delta_x;
  double temp_y = 0.0;
  if (debug_flag[20]) {
    AERROR << PRECISION(3) << "start_xy:[" << x_s << "," << y_s << "]  end_xy:["
           << x_e << "," << y_e << "]"
           << "  dis_x:" << dis_x << "  "
           << "  delta_x:" << delta_x;
  }
  while (fabs(temp_x) < fabs(dis_x)) {
    temp_y = coef(0, 0) * temp_x * temp_x * temp_x +
             coef(1, 0) * temp_x * temp_x + coef(2, 0) * temp_x + coef(3, 0);
    base_system.NewBase2OriginRotatePan({temp_x, temp_y}, &pos_rel_to_abs);
    if (debug_flag[20]) {
      AERROR << PRECISION(4) << "x_t:" << temp_x << "  y_t:" << temp_y
             << "  pos_xy:[" << pos_rel_to_abs.at(0) << ","
             << pos_rel_to_abs.at(1) << "]";
    }
    x_vec.push_back(pos_rel_to_abs.at(0));
    y_vec.push_back(pos_rel_to_abs.at(1));
    trajectory_points_raw_.emplace_back(pos_rel_to_abs.at(0),
                                        pos_rel_to_abs.at(1));
    temp_x += delta_x;
  }

  if (debug_flag[50]) {
    AERROR << PRECISION(3) << "start_state:[" << x_s << "," << y_s << ","
           << theta_s << "] state_end:[" << x_e << "," << y_e << "," << theta_e
           << "]"
           << "  delta_x:" << delta_x << "  dis_x:" << dis_x;
    std::string x_v = "x_vec=[";
    std::string y_v = "y_vec=[";
    for (int i = 0; i < x_vec.size(); ++i) {
      x_v += std::to_string(x_vec[i]) + ",";
      y_v += std::to_string(y_vec[i]) + ",";
    }
    x_v.back() = ']';
    y_v.back() = ']';
    AERROR << x_v;
    AERROR << y_v;
  }
  fit_curve_ = std::make_shared<FitManager>(
      FitType::PolyNomial,
      ego_state.GetVehicleStatePresent().linear_velocity());
  ego_position_.set_x(ego_x_raw);
  ego_position_.set_y(ego_y_raw);
  if (cyber_flag[1]) {
    auto ptr_e = ptr_without_lane->mutable_debug()->add_temp_value();
    ptr_e->set_name("coefficient");
    ptr_e->add_rd(coef(0, 0));
    ptr_e->add_rd(coef(1, 0));
    ptr_e->add_rd(coef(2, 0));
    ptr_e->add_rd(coef(3, 0));
  }
  if (debug_flag[0]) {
    AERROR << PRECISION(4) << "euclidean_dis_ego_obs:" << euclidean_dis_ego_obs
           << "  num:" << num << "   ego_rel_x:" << ego_x_rel
           << "  obs_x:" << obs_x << "   s_interval:" << s_interval
           << "  start_num:" << num_start
           << "  point_num:" << num_trajectory_ext;
    for (int i = 0; i < trajectory_points_raw_.size(); ++i) {
      AERROR << PRECISION(4) << "i:" << i
             << "  fit_3order_x:" << trajectory_points_raw_.at(i).x()
             << "  fit_3order_y:" << trajectory_points_raw_.at(i).y();
    }
  }
  if (debug_flag[5]) {
    for (int i = 0; i < state_history_extend.size(); ++i) {
      AERROR << PRECISION(4) << "i:" << i << "   obs_rel_traj_x:"
             << GetExtendStateIndex(state_history_extend.at(i)).x -
                    ego_position_.x()
             << "   obs_rel_traj_y:"
             << GetExtendStateIndex(state_history_extend.at(i)).y -
                    ego_position_.y();
    }
  }
}

LaneCenterLine::LaneLocation LaneCenterLine::IsTrajectoryCrossLane(
    StateType obs_state) {
  return LaneCenterLine::LaneLocation::EgoLane;
}

bool LaneCenterLine::TrajectoryIsInLane(StateType obs_state) {
  return false;
}

void LaneCenterLine::SetLength(double length) {
  length_ = length;
}

void LaneCenterLine::SetWidth(double width) {
  width_ = width;
}

void LaneCenterLine::SetLaneLocation(
    LaneCenterLine::LaneLocation laneLocation) {
  lane_location_ = laneLocation;
}

const std::shared_ptr<FitManager>& LaneCenterLine::GetFitCurve() const {
  return fit_curve_;
}

double LaneCenterLine::GetLength() const {
  return length_;
}

double LaneCenterLine::GetWidth() const {
  return width_;
}

LaneCenterLine::LaneLocation LaneCenterLine::GetLaneLocation() const {
  return lane_location_;
}

unsigned int LaneCenterLine::GetId() const {
  return id_;
}

void LaneCenterLine::SetId(unsigned int id) {
  id_ = id;
}

const obstacles_ptr_list& LaneCenterLine::GetObstaclesInLane() const {
  return obstacles_in_lane_;
}

void LaneCenterLine::SetObstaclesInLane(
    const obstacles_ptr_list& obstaclesInLane) {
  obstacles_in_lane_ = obstaclesInLane;
}

const std::vector<std::pair<double, double>>&
LaneCenterLine::GetTrajectoriesInLane() const {
  return trajectories_in_lane_;
}

const std::vector<std::pair<double, double>>&
LaneCenterLine::GetTrajectoriesCrossed() const {
  return trajectories_crossed_;
}

void LaneCenterLine::SetTrajectoriesInLane(
    const std::vector<std::pair<double, double>>& trajectoriesInLane) {
  trajectories_in_lane_ = trajectoriesInLane;
}

void LaneCenterLine::SetTrajectoriesCrossed(
    const std::vector<std::pair<double, double>>& trajectoriesCrossed) {
  trajectories_crossed_ = trajectoriesCrossed;
}

void LaneCenterLine::ConstructLaneLine(bool based_on_traj_line) {
  LogProcess::LogProc(__func__);
  if (!fit_curve_) {
    AERROR << "fit_curve_pimpl_ is nullptr.";
    return;
  }
  fit_curve_->Fit(trajectory_points_raw_);
  bool is_ok = fit_curve_->GeneratePointsByFormula(&trajectory_points_fit_);
  const unsigned int min_size_fit_points = 2;
  if (trajectory_points_fit_.size() < min_size_fit_points) {
    AERROR << "fit size is too small.";
    return;
  }

  /***************** fit piecewise start ************************/

  std::vector<Vec2d> fit_points;

  if (based_on_traj_line &&
      !fit_curve_->FitPiecewise(
          &fit_raw_points_, &fit_points,
          previous_lane_center_line_->GetReferenceLinePtrExtend())) {
    if (debug_flag[17]) {
      AERROR << "fit_piecewise is not ok.";
    }
    is_ok = false;
  }

  /***************** fit piecewise end   ************************/

  if (!is_ok || trajectory_points_fit_.empty()) {
    AERROR << "fit curve failed.  is_ok:" << is_ok
           << "  fit_size:" << trajectory_points_fit_.size();
    construct_lane_success_ = false;
    return;
  }
  CheckPointsIsInLane();

  int min_num = 5;
  if (trajectory_points_fit_.size() < min_num) {
    construct_lane_success_ = false;
    return;
  }

  // lp: construct reference line
  const size_t points_num = trajectory_points_fit_.size();
  trajectory_points_output_xy_.clear();
  trajectory_points_output_xy_.resize(points_num);
  trajectory_points_heading_.clear();
  trajectory_points_heading_.resize(points_num);

  if (debug_flag[17]) {
    AERROR << PRECISION(3) << "ego_s:" << ego_sl_pos_.s()
           << "  ego_l:" << ego_sl_pos_.l() << "  Ego_Position:["
           << ego_position_.x() << "," << ego_position_.y() << "]";
  }
  if (based_on_traj_line) {
    // lp: process first point
    if (!previous_lane_center_line_->GetReferenceLinePtrExtend()) {
      AERROR << "previous reference_ptr is nullptr.";
      // construct_lane_success_ = false;
    }
    SLPoint sl_p;
    for (int i = 0; i < points_num; ++i) {
      sl_p.set_s(trajectory_points_fit_.at(i).x());
      sl_p.set_l(trajectory_points_fit_.at(i).y());
      previous_lane_center_line_->GetReferenceLinePtrExtend()->SLToXY(
          sl_p, &(trajectory_points_output_xy_.at(i)));
      if (debug_flag[17]) {
        AERROR << "i:" << i << PRECISION(3) << "  fit_out_sl_p__s:" << sl_p.s()
               << "  fit_out_sl_p__l:" << sl_p.l()
               << "   proj_x:" << trajectory_points_output_xy_.at(i).x()
               << "   proj_y:" << trajectory_points_output_xy_.at(i).y();
      }
    }
    trajectory_points_output_xy_.clear();
    trajectory_points_output_xy_ = fit_points;
    curve_connect_ego_trajectory_ = false;
  } else {
    // lp: connect lane between ego and obstacle

    // lp: TODO fit curvature is too big!!! Solve it!
    // trajectory_points_output_xy_ = trajectory_points_fit_;

    // lp: temporary solution
    const size_t raw_size = trajectory_points_raw_.size();
    trajectory_points_output_xy_.resize(raw_size);
    trajectory_points_output_xy_ = trajectory_points_raw_;

    if (debug_flag[17]) {
      AERROR << "based_on_traj_line:" << based_on_traj_line;
      for (const auto& pos : trajectory_points_output_xy_) {
        AERROR << PRECISION(3) << "before_restore_traj_x:" << pos.x()
               << "  before_restore_traj_y:" << pos.y();
      }
    }
    // RestorePointPosition();
    curve_connect_ego_trajectory_ = true;
  }

  const size_t num_xy = trajectory_points_output_xy_.size();
  if (num_xy < 2) {
    AERROR << "is_based_on_traj:" << based_on_traj_line
           << "   num_xy:" << num_xy;
    return;
  }
  ref_path_point_.clear();
  ref_path_point_.reserve(num_xy);
  trajectory_points_heading_.resize(num_xy);
  double heading_t = 0;
  for (int i = 1; i < num_xy; ++i) {
    heading_t = atan((trajectory_points_output_xy_.at(i).y() -
                      trajectory_points_output_xy_.at(i - 1).y()) /
                     (trajectory_points_output_xy_.at(i).x() -
                      trajectory_points_output_xy_.at(i - 1).x()));
    if (common::math::double_type::DefinitelyGreater(
            trajectory_points_output_xy_.at(i).y(),
            trajectory_points_output_xy_.at(i - 1).y())) {
      if (common::math::double_type::DefinitelyLess(heading_t, 0.0)) {
        heading_t += M_PI;
      }
    } else if (common::math::double_type::DefinitelyLess(
                   trajectory_points_output_xy_.at(i).y(),
                   trajectory_points_output_xy_.at(i - 1).y())) {
      if (common::math::double_type::DefinitelyGreater(heading_t, 0.0)) {
        heading_t -= M_PI;
      }
    } else {
      heading_t = 0.0;
    }
    trajectory_points_heading_.at(i) = heading_t;
  }
  trajectory_points_heading_.at(0) = trajectory_points_heading_.at(1);

  for (int i = 0; i < num_xy; ++i) {
    if (debug_flag[7]) {
      AERROR << PRECISION(3) << "i:" << i
             << " current_ref_x:" << trajectory_points_output_xy_.at(i).x()
             << "  current_ref_y:" << trajectory_points_output_xy_.at(i).y()
             << "  current_ref_theta:" << trajectory_points_heading_.at(i);
    }
    // lp: Here I set kappa,dkappa as 0, values can be calc precisely by
    // fit_curve.
    ref_path_point_.push_back({{{trajectory_points_output_xy_.at(i).x(),
                                 trajectory_points_output_xy_.at(i).y()},
                                trajectory_points_heading_.at(i)},
                               0,
                               0});
  }
  // reference_line_ptr_ =
  //     std::make_shared<ReferenceLine>(std::move(ref_path_point_));
  if (debug_flag[7]) {
    AERROR << "Generated reference line:" << GetPtr(reference_line_ptr_)
           << "  start_s:" << start_s_;
  }
  if (!based_on_traj_line && obs_followed_index_.obstacle_ptr) {
    const auto& trajectory_extend =
        obs_followed_index_.obstacle_ptr->GetStateHistoryExtended();
    const size_t num_start = obs_followed_index_.index;
    if (trajectory_extend.size() > num_start) {
      Vec2d start_xy(GetExtendStateIndex(trajectory_extend.at(num_start)).x,
                     GetExtendStateIndex(trajectory_extend.at(num_start)).y);
      SLPoint start_sl;
      reference_line_ptr_->XYToSL(start_xy, &start_sl);
      start_s_ = start_sl.s();
    }
  } else if (based_on_traj_line) {
    start_s_ = -0.001;
  }
  construct_lane_success_ = true;
}

bool LaneCenterLine::CheckPointsIsInLane() {
  // lp: TODO Are all the points  in lane?
  return true;
}

bool LaneCenterLine::IsConstructLaneSuccess() const {
  return construct_lane_success_;
}

void LaneCenterLine::SetConstructLaneSuccess(bool constructLaneSuccess) {
  construct_lane_success_ = constructLaneSuccess;
}

const std::shared_ptr<ReferenceLine>& LaneCenterLine::GetReferenceLinePtr()
    const {
  return reference_line_ptr_;
}

void LaneCenterLine::SetReferenceLinePtr(
    const std::shared_ptr<ReferenceLine>& referenceLinePtr) {
  reference_line_ptr_ = referenceLinePtr;
}

const std::vector<Vec2d>& LaneCenterLine::GetTrajectoryPointsRaw() const {
  return trajectory_points_raw_;
}

const std::vector<Vec2d>& LaneCenterLine::GetTrajectoryPointsFit() const {
  return trajectory_points_fit_;
}

const std::vector<Vec2d>& LaneCenterLine::GetTrajectoryPointsXy() const {
  return trajectory_points_output_xy_;
}

const std::vector<double>& LaneCenterLine::GetTrajectoryPointsHeading() const {
  return trajectory_points_heading_;
}

void LaneCenterLine::SetTrajectoryPointsRaw(
    const std::vector<Vec2d>& trajectoryPointsRaw) {
  trajectory_points_raw_ = trajectoryPointsRaw;
}

void LaneCenterLine::SetTrajectoryPointsFit(
    const std::vector<Vec2d>& trajectoryPointsFit) {
  trajectory_points_fit_ = trajectoryPointsFit;
}

void LaneCenterLine::SetTrajectoryPointsXy(
    const std::vector<Vec2d>& trajectoryPointsXy) {
  trajectory_points_output_xy_ = trajectoryPointsXy;
}

void LaneCenterLine::SetTrajectoryPointsHeading(
    const std::vector<double>& trajectoryPointsHeading) {
  trajectory_points_heading_ = trajectoryPointsHeading;
}

void LaneCenterLine::ExtendLaneLength() {
  size_t num = ref_path_point_.size();
  double length = reference_line_ptr_->GetMapPath().length();
  double extend_length = length_ - length;
  if (debug_flag[27]) {
    AERROR << "calc_traj_heading:" << trajectory_points_heading_.at(num - 1)
           << "   ref_heading:" << ref_path_point_.at(num - 1).heading();
  }
  SLPoint ego_sl;
  reference_line_ptr_->XYToSL({ego_position_.x(), ego_position_.y()}, &ego_sl);
  if (ego_sl.s() < 0) {
    if (debug_flag[28]) {
      AERROR << PRECISION(3) << "ref_point_0_x:" << ref_path_point_.front().x()
             << "  ref_point_0_y:" << ref_path_point_.front().y()
             << "  ref_0_theta:" << trajectory_points_heading_.front()
             << "  ref_point_1_x:" << ref_path_point_.at(1).x()
             << "  ref_point_1_y:" << ref_path_point_.at(1).y()
             << "  ref_1_theta:" << trajectory_points_heading_.at(1)
             << "  ego_sl:" << ego_sl.s();
    }
    double theta = trajectory_points_heading_.front() > 0
                       ? trajectory_points_heading_.front() - M_PI
                       : M_PI + trajectory_points_heading_.front();
    ref_path_point_.at(0).set_x((fabs(ego_sl.s()) + back_ego_length_) *
                                    cos(theta) +
                                ref_path_point_.front().x());
    ref_path_point_.at(0).set_y((fabs(ego_sl.s()) + back_ego_length_) *
                                    sin(theta) +
                                ref_path_point_.front().y());
    if (debug_flag[28]) {
      AERROR << PRECISION(3) << "ref_point_0_x:" << ref_path_point_.front().x()
             << "  ref_point_0_y:" << ref_path_point_.front().y()
             << "  ref_0_theta:" << trajectory_points_heading_.front()
             << "  ref_point_1_x:" << ref_path_point_.at(1).x()
             << "  ref_point_1_y:" << ref_path_point_.at(1).y()
             << "  ref_1_theta:" << trajectory_points_heading_.at(1)
             << "   theta:" << theta;
    }
  }

  ref_path_point_.at(num - 1).set_x(
      extend_length * cos(trajectory_points_heading_.at(num - 1)) +
      ref_path_point_.at(num - 2).x());
  ref_path_point_.at(num - 1).set_y(
      extend_length * sin(trajectory_points_heading_.at(num - 1)) +
      ref_path_point_.at(num - 2).y());
  // reference_line_ptr_extend_ = std::make_shared<ReferenceLine>(ref_path_point_);
}

const std::shared_ptr<ReferenceLine>&
LaneCenterLine::GetReferenceLinePtrExtend() const {
  return reference_line_ptr_extend_;
}

const std::shared_ptr<navigation_hdmap::MapMsg>& LaneCenterLine::GetMapMsgPtr()
    const {
  return no_lane_map_msg_;
}

bool LaneCenterLine::CreatNoLaneMapAndRouting() {
  if (!reference_line_ptr_extend_) {
    AERROR << "ref_extend is empty.";
    return false;
  }
  auto map_path = reference_line_ptr_extend_->GetMapPath();
  auto path = GeneratePathFromPoints(map_path);
  no_lane_map_msg_ = std::make_shared<navigation_hdmap::MapMsg>();
  common::util::FillHeader("nolane_hdmap", no_lane_map_msg_.get());
  auto hd_map = no_lane_map_msg_->mutable_hdmap();
  common::util::FillHeader("from_nolane_hdmap", hd_map->mutable_header());
  auto lane = hd_map->add_lane();
  GenerateOneLane(path, lane);
  if (debug_flag[29]) {
    for (auto& poi :
         lane->central_curve().segment().at(0).line_segment().point()) {
      AERROR << PRECISION(3) << "Lane_Point_x:" << poi.x()
             << "  Lane_Point_y:" << poi.y();
    }
  }
  if (cyber_flag[0]) {
    if (lane->has_central_curve()) {
      ptr_without_lane->add_central_curve()->CopyFrom(lane->central_curve());
    }
    if (lane->has_left_boundary()) {
      ptr_without_lane->mutable_left_boundary()->CopyFrom(
          lane->left_boundary());
    }
    if (lane->has_right_boundary()) {
      ptr_without_lane->mutable_right_boundary()->CopyFrom(
          lane->right_boundary());
    }
  }
  auto routing = no_lane_map_msg_->mutable_routing();
  SetNoLaneRouting(routing, hd_map);
  if (false && debug_flag[24]) {
    AERROR << "Routing Info:" << routing->DebugString();
  }
  return true;
}

common::Path LaneCenterLine::GeneratePathFromPoints(
    const hdmap::Path& map_path) {
  common::Path path;
  path.set_name("0_from_nolane");
  auto path_points = map_path.path_points();
  TL::hdmap::MapPathPoint prev_point;
  TL::hdmap::MapPathPoint cur_point;
  double delta_s;
  common::PathPoint path_point;
  for (int i = 0; i < path_points.size(); i++) {
    if (i == 0) {
      path_point.set_s(0.0);
      path_point.set_x(path_points.at(i).x());
      path_point.set_y(path_points.at(i).y());
      path_point.set_theta(path_points.at(i).heading());
    } else {
      prev_point = path_points.at(i - 1);
      cur_point = path_points.at(i);
      delta_s = Vec2d(prev_point.x(), prev_point.y())
                    .DistanceTo(Vec2d(cur_point.x(), cur_point.y()));
      path_point.set_s(delta_s + (*path.path_point().rbegin()).s());
      path_point.set_x(path_points.at(i).x());
      path_point.set_y(path_points.at(i).y());
      path_point.set_theta(path_points.at(i).heading());
    }
    path.add_path_point()->CopyFrom(path_point);
  }
  return path;
}

bool LaneCenterLine::GenerateOneLane(const common::Path& path,
                                     hdmap::Lane* lane) {
  if (path.path_point_size() < 2) {
    AERROR << "The path length of line index is invalid";
    return false;
  }
  lane->mutable_id()->set_id(path.name());
  // lane types
  lane->set_type(hdmap::Lane::CITY_DRIVING);
  lane->set_turn(hdmap::Lane::NO_TURN);

  // speed limit
  lane->set_speed_limit(20.0);

  // center line
  auto* curve_segment = lane->mutable_central_curve()->add_segment();
  curve_segment->set_heading(path.path_point(0).theta());
  curve_segment->set_length(path.path_point(path.path_point_size() - 1).s());
  lane->set_length(path.path_point(path.path_point_size() - 1).s());
  auto* line_segment = curve_segment->mutable_line_segment();

  // left boundary
  auto* left_boundary = lane->mutable_left_boundary();
  auto* left_boundary_type = left_boundary->add_boundary_type();
  left_boundary->set_virtual_(false);
  left_boundary_type->set_s(0.0);
  left_boundary_type->add_types(hdmap::LaneBoundaryType::DOTTED_WHITE);
  auto* left_segment =
      left_boundary->mutable_curve()->add_segment()->mutable_line_segment();

  // right boundary
  auto* right_boundary = lane->mutable_right_boundary();
  auto* right_boundary_type = right_boundary->add_boundary_type();
  right_boundary->set_virtual_(false);
  right_boundary_type->set_s(0.0);
  right_boundary_type->add_types(hdmap::LaneBoundaryType::DOTTED_WHITE);
  auto* right_segment =
      right_boundary->mutable_curve()->add_segment()->mutable_line_segment();

  const double half_lane_width = 3.8 * 0.5;
  double index = 0;

  for (size_t i = 0; i < path.path_point().size(); i++) {
    auto* point = line_segment->add_point();
    point->set_x(path.path_point().at(i).x());
    point->set_y(path.path_point().at(i).y());
    point->set_z(path.path_point().at(i).z());
    index = index + 1;
    auto* left_sample = lane->add_left_sample();
    left_sample->set_s(path.path_point().at(i).s());
    left_sample->set_width(half_lane_width);
    left_segment->add_point()->CopyFrom(
        *point +
        half_lane_width *
            Vec2d::CreateUnitVec2d(path.path_point().at(i).theta() + M_PI_2));

    auto* right_sample = lane->add_right_sample();
    right_sample->set_s(path.path_point().at(i).s());
    right_sample->set_width(half_lane_width);
    right_segment->add_point()->CopyFrom(
        *point +
        half_lane_width *
            Vec2d::CreateUnitVec2d(path.path_point().at(i).theta() - M_PI_2));
  }
  return true;
}

bool LaneCenterLine::SetNoLaneRouting(
    TL::routing::RoutingResponse* inrouting, TL::hdmap::Map* hd_map) {
  // Set road boundary
  auto* road = hd_map->add_road();
  road->mutable_id()->set_id("road_nolane");
  auto* section = road->add_section();
  section->mutable_id()->set_id("0");
  section->add_lane_id()->CopyFrom(hd_map->lane(0).id());

  auto* outer_polygon = section->mutable_boundary()->mutable_outer_polygon();
  auto* left_edge = outer_polygon->add_edge();
  left_edge->set_type(TL::hdmap::BoundaryEdge::LEFT_BOUNDARY);
  left_edge->mutable_curve()->CopyFrom(hd_map->lane(0).left_boundary().curve());

  auto* right_edge = outer_polygon->add_edge();
  right_edge->set_type(TL::hdmap::BoundaryEdge::RIGHT_BOUNDARY);
  right_edge->mutable_curve()->CopyFrom(
      hd_map->lane(0).right_boundary().curve());
  // Set routing info
  auto* routing_road = inrouting->add_road();
  routing_road->set_id(road->id().id());
  // set passage and routing
  auto* passage = routing_road->add_passage();
  passage->set_can_exit(false);
  passage->set_change_lane_type(routing::ChangeLaneType::FORWARD);
  auto* segment = passage->add_segment();
  segment->set_id(hd_map->lane(0).id().id());
  auto adc_lane_segment_points =
      hd_map->lane(0).central_curve().segment().at(0).line_segment().point();

  common::PointENU start_point = adc_lane_segment_points.at(0);
  int max_index = adc_lane_segment_points.size() - 1;
  common::PointENU end_point = adc_lane_segment_points.at(max_index);
  auto* routing_request = inrouting->mutable_routing_request();
  common::util::FillHeader("from_nolane_routingrequest", routing_request);

  segment->set_start_s(0.0);
  segment->set_end_s(hd_map->lane(0).length());

  routing::LaneWaypoint waypoint;

  waypoint.set_id(hd_map->lane(0).id().id());
  waypoint.mutable_pose()->set_x(start_point.x());
  waypoint.mutable_pose()->set_y(start_point.y());
  waypoint.set_s(0.0);
  routing_request->add_waypoint()->CopyFrom(waypoint);

  waypoint.set_s(hd_map->lane(0).length());
  waypoint.mutable_pose()->set_x(end_point.x());
  waypoint.mutable_pose()->set_y(end_point.y());
  routing_request->add_waypoint()->CopyFrom(waypoint);

  common::util::FillHeader("from_nolane_routing", inrouting);
  return true;
}

const std::shared_ptr<navigation_hdmap::MapMsg>&
LaneCenterLine::GetNoLaneMapMsg() const {
  return no_lane_map_msg_;
}

void LaneCenterLine::RestorePointPosition() {
  std::for_each(trajectory_points_output_xy_.begin(),
                trajectory_points_output_xy_.end(), [&](auto& point) {
                  point.set_x(point.x() + ego_position_.x());
                  point.set_y(point.y() + ego_position_.y());
                });
}

bool LaneCenterLine::EgoIsStillInPreviousLane(
    const EgoVehicleState& veh_state) {
  LogProcess::LogProc(__func__);
  if (!reference_line_ptr_) {
    AERROR << "previous reference line is nullptr.";
  }
  ego_position_.set_x(veh_state.GetVehicleStatePresent().x());
  ego_position_.set_y(veh_state.GetVehicleStatePresent().y());
  SLPoint ego_sl;
  reference_line_ptr_->XYToSL(ego_position_, &ego_sl);
  if (debug_flag[10]) {
    AERROR << "ego_s:" << ego_sl.s() << "  max_s:"
           << reference_line_ptr_->map_path().accumulated_s().back()
           << "  min_s:"
           << reference_line_ptr_->map_path().accumulated_s().front();
  }
  return ego_sl.s() < reference_line_ptr_->map_path().accumulated_s().back() -
                          FLAGS_nolane_min_s_before_ego_in_ref &&
         ego_sl.s() > reference_line_ptr_->map_path().accumulated_s().front();
}

void LaneCenterLine::SetLaneCenterLinePtr(
    const std::shared_ptr<LaneCenterLine>& laneCenterLinePtr) {
  previous_lane_center_line_ = laneCenterLinePtr;
}

double LaneCenterLine::GetStartS() const {
  return start_s_;
}

bool LaneCenterLine::IsCurveConnectEgoTrajectory() const {
  return curve_connect_ego_trajectory_;
}

void LaneCenterLine::SetCurveConnectEgoTrajectory(
    bool curveConnectEgoTrajectory) {
  curve_connect_ego_trajectory_ = curveConnectEgoTrajectory;
}

double LaneCenterLine::GetTimeUpdateLatest() const {
  return time_update_latest_;
}

void LaneCenterLine::SetTimeUpdateLatest(double timeUpdateLatest) {
  time_update_latest_ = timeUpdateLatest;
}
}  // namespace nolane
}  // namespace planning
}  // namespace TL
