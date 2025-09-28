/*
 * Copyright (c) TL Technologies Co., Ltd. 2023. All rights reserved.
 * Description:  nlp_path_smoother.cc
 */

#include "planning/open_space/nlp_path_smoother/nlp_path_smoother.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "coin/IpIpoptApplication.hpp"
#include "coin/IpSmartPtr.hpp"
#include "coin/IpSolveStatistics.hpp"
#include "common/math/box2d.h"
#include "common/math/double_type.h"
#include "common/math/line_segment2d.h"
#include "common/math/math_utils.h"
#include "common/math/polygon2d.h"
#include "common/math/vec2d.h"
#include "common/status/status.h"
#include "common/util/util.h"
#include "common/vehicle_state/vehicle_state_provider.h"
#include "glog/logging.h"
#include "planning/common/open_space_info.h"
#include "planning/common/path/discretized_path.h"
#include "planning/open_space/nlp_path_smoother/nlp_math_model.h"
#include "planning/open_space/nlp_path_smoother/nlp_math_model_cppad.h"
#include "proto/common/error_code.pb.h"
#include "proto/common/pnc_point.pb.h"
#include "proto/common/vehicle_state.pb.h"

namespace TL {
namespace planning {

using TL::common::ErrorCode;
using TL::common::Status;

namespace {
constexpr double kSteerAngleBuffer = 30.0 / 180.0 * M_PI;
constexpr double kLatExtendBuffer = 1.0;
}  // namespace

bool NlpPathSmoother::NlpSolver(
    const std::vector<PathGearPair>& partition_paths,
    const std::vector<std::vector<
        std::pair<common::math::LineSegment2d, common::math::LineSegment2d>>>&
        xy_road_bounds,
    const bool enable_fix_start_kappa,
    const std::tuple<std::pair<double, double>, bool, bool>&
        dest_lat_region_constrain,
    std::vector<PathGearPair>* const smooth_path,
    bool* const force_enable_dest_lat_constrain) const {
  DCHECK(nullptr != smooth_path);
  DCHECK(nullptr != force_enable_dest_lat_constrain);
  DCHECK_EQ(partition_paths.size(), xy_road_bounds.size());
  *force_enable_dest_lat_constrain = false;
  if (!config_.enable_smoother()) {
    AERROR << "NlpPathSmoother is disabled.";
    return false;
  }
  for (size_t i = 0; i < partition_paths.size(); ++i) {
    if (partition_paths[i].first.size() != xy_road_bounds[i].size() ||
        partition_paths[i].first.size() < 2) {
      AERROR << "NlpPathSmoother i: " << i
             << ", partition_paths.size: " << partition_paths[i].first.size()
             << ", xy_road_bounds[i].size(): " << xy_road_bounds[i].size();
      return false;
    }
  }

  smooth_path->clear();

  NlpInputParam nlp_input_param;
  CalculateNlpInputInfo(partition_paths, xy_road_bounds, enable_fix_start_kappa,
                        dest_lat_region_constrain, &nlp_input_param);
  std::vector<double> optimum;
  bool success = SolveProblem(nlp_input_param, &optimum);
  // 0: left/right buffer; 1: has region; 2: use region
  if (!success && std::get<1>(dest_lat_region_constrain) &&
      (!std::get<2>(dest_lat_region_constrain))) {
    *force_enable_dest_lat_constrain = true;
    nlp_input_param.enable_dest_lat_region_constrain = true;
    optimum.clear();
    success = SolveProblem(nlp_input_param, &optimum);
  }
  const double sx = nlp_input_param.origin_point.x();
  const double sy = nlp_input_param.origin_point.y();
  std::vector<PathGearPair> optimized_paths;
  if (success) {
    uint32_t cur_index = 0;
    for (size_t i = 0; i < nlp_input_param.path_point_size.size(); ++i) {
      DiscretizedPath optimized_path;
      optimized_path.reserve(nlp_input_param.path_point_size[i]);
      for (uint32_t j = 0; j < nlp_input_param.path_point_size[i]; ++j) {
        common::PathPoint tmp;
        tmp.set_x(optimum.at(cur_index) + sx);
        tmp.set_y(optimum.at(nlp_input_param.total_point_size + cur_index) +
                  sy);
        tmp.set_theta(common::math::NormalizeAngle(
            optimum.at(2 * nlp_input_param.total_point_size + cur_index)));
        tmp.set_kappa(
            optimum.at(3 * nlp_input_param.total_point_size + cur_index));
        if (j == 0) {
          tmp.set_s(0.0);
        } else {
          double dx = tmp.x() - optimized_path.back().x();
          double dy = tmp.y() - optimized_path.back().y();
          tmp.set_s(optimized_path.back().s() + std::sqrt(dx * dx + dy * dy));
        }
        optimized_path.push_back(std::move(tmp));
        cur_index++;
      }

      optimized_paths.emplace_back(optimized_path, partition_paths[i].second);
    }
  }

  InterpolationByS(optimized_paths, smooth_path);
  for (auto& path : *smooth_path) {
    const double sign = (path.second == soc::Chassis::GEAR_DRIVE) ? 1.0 : -1.0;
    for (auto& p : path.first) {
      p.set_kappa(sign * p.kappa());
    }
  }

  return (!smooth_path->empty());
}

Status NlpPathSmoother::XYRoadPreprocessor(
    const std::vector<PathGearPair>& raw_partition_path_pairs,
    const std::vector<std::pair<common::math::LineSegment2d, double>>&
        obj_segments,
    const std::pair<double, bool>& init_kappa_constrain,
    std::vector<PathGearPair>* const partition_paths,
    std::vector<std::vector<std::pair<common::math::LineSegment2d,
                                      common::math::LineSegment2d>>>* const
        xy_road_bounds) const {
  if (raw_partition_path_pairs.empty() || partition_paths == nullptr ||
      xy_road_bounds == nullptr) {
    const std::string msg =
        "raw_partition_path_pairs.size: " +
        std::to_string(raw_partition_path_pairs.size()) +
        ", partition_paths is nullptr: " +
        std::to_string(static_cast<uint32_t>(partition_paths == nullptr)) +
        ", xy_road_bounds is nullptr: " +
        std::to_string(static_cast<uint32_t>(xy_road_bounds == nullptr));
    AERROR << msg;
    return Status(ErrorCode::PLANNER_PARKING_PATHSMOOTHER_ERROR, msg);
  }

  partition_paths->clear();
  xy_road_bounds->clear();
  partition_paths->reserve(raw_partition_path_pairs.size());
  xy_road_bounds->reserve(raw_partition_path_pairs.size());
  for (size_t i = 0; i < raw_partition_path_pairs.size(); ++i) {
    bool is_forward = true;
    std::vector<common::PathPoint> modifed_rough_point;
    auto status = RoughPathProcessor(
        raw_partition_path_pairs[i].first, init_kappa_constrain,
        raw_partition_path_pairs.size(), i, &is_forward, &modifed_rough_point);
    if (!status.ok()) {
      AERROR << "RoughPathProcessor failed.";
      return status;
    }

    std::vector<common::PathPoint> interpolation_point;
    if (!InterpolationByS(modifed_rough_point, &interpolation_point)) {
      const std::string msg =
          "InterpolationByS failed, path_index: " + std::to_string(i);
      AERROR << msg;
      return Status(ErrorCode::PLANNER_PARKING_PATHSMOOTHER_ERROR, msg);
    }
    const auto gear =
        is_forward ? soc::Chassis::GEAR_DRIVE : soc::Chassis::GEAR_REVERSE;
    partition_paths->emplace_back(interpolation_point, gear);
    ADEBUG << "path index: " << i;
    std::vector<
        std::pair<common::math::LineSegment2d, common::math::LineSegment2d>>
        xy_road_bound;
    CalculateXYRoadBound(partition_paths->back().first, obj_segments,
                         &xy_road_bound);
    xy_road_bounds->push_back(std::move(xy_road_bound));
  }

  return Status::OK();
}

Status NlpPathSmoother::RoughPathProcessor(
    const std::vector<common::PathPoint>& raw_rough_point,
    const std::pair<double, bool>& init_kappa_constrain,
    const size_t /*total_path_size*/, const size_t path_index,
    bool* const is_forward,
    std::vector<common::PathPoint>* const modifed_rough_point) {
  if (is_forward == nullptr || modifed_rough_point == nullptr ||
      raw_rough_point.size() < 2) {
    const std::string msg =
        "is_forward is nullptr: " +
        std::to_string(static_cast<uint32_t>(is_forward == nullptr)) +
        ", rough_path is nullptr: " +
        std::to_string(static_cast<uint32_t>(modifed_rough_point == nullptr)) +
        ", raw_rough_point.size: " + std::to_string(raw_rough_point.size()) +
        ", path_index: " + std::to_string(path_index);
    AERROR << msg;
    return Status(ErrorCode::PLANNER_PARKING_PATHSMOOTHER_ERROR, msg);
  }

  modifed_rough_point->clear();
  modifed_rough_point->reserve(raw_rough_point.size());
  const double first_heading = raw_rough_point[0].theta();
  const double first_to_second_heading =
      std::atan2(raw_rough_point[1].y() - raw_rough_point[0].y(),
                 raw_rough_point[1].x() - raw_rough_point[0].x());
  *is_forward = std::fabs(common::math::NormalizeAngle(
                    first_heading - first_to_second_heading)) < M_PI_2;
  for (size_t j = 0; j < raw_rough_point.size(); j++) {
    common::PathPoint tmp;
    tmp.set_x(raw_rough_point[j].x());
    tmp.set_y(raw_rough_point[j].y());
    tmp.set_theta(raw_rough_point[j].theta());
    if (j == 0) {
      tmp.set_kappa(0.0);
      tmp.set_s(0.0);
    } else {
      double ds =
          std::hypot(std::fabs(tmp.x() - modifed_rough_point->back().x()),
                     std::fabs(tmp.y() - modifed_rough_point->back().y()));
      double dtheta = common::math::NormalizeAngle(
          tmp.theta() - modifed_rough_point->back().theta());
      tmp.set_kappa(dtheta / ds);
      tmp.set_s(modifed_rough_point->back().s() + ds);
    }
    modifed_rough_point->push_back(std::move(tmp));
  }
  if (path_index == 0 && init_kappa_constrain.second) {
    const double sign = (*is_forward) ? 1.0 : -1.0;
    modifed_rough_point->at(0).set_kappa(sign * init_kappa_constrain.first);
  } else {
    double dtheta =
        common::math::NormalizeAngle(modifed_rough_point->at(1).theta() -
                                     modifed_rough_point->at(0).theta());
    modifed_rough_point->at(0).set_kappa(dtheta /
                                         modifed_rough_point->at(1).s());
  }

  return Status::OK();
}

bool NlpPathSmoother::InterpolationByS(
    const std::vector<common::PathPoint>& raw_path_point,
    std::vector<common::PathPoint>* const interpolation_point) const {
  if (raw_path_point.size() < 2 || interpolation_point == nullptr) {
    AERROR << "InterpolationByS interpolation_point==nullptr or "
              "raw_path_point.size: "
           << raw_path_point.size() << " is less than 2.";
    return false;
  }

  interpolation_point->clear();
  const double path_length =
      raw_path_point.back().s() - raw_path_point.front().s();
  const double ds = std::min(config_.interpolation_s(),
                             path_length / config_.min_point_num());
  ADEBUG << "InterpolationByS ds: " << ds << ", length: " << path_length;
  DiscretizedPath discretized_path(raw_path_point);
  interpolation_point->push_back(discretized_path.front());
  for (double s = ds; s < path_length; s += ds) {
    auto evaluate_point =
        discretized_path.Evaluate(discretized_path.front().s() + s);
    interpolation_point->push_back(std::move(evaluate_point));
  }
  constexpr double kTwo = 2.0;
  if (discretized_path.back().s() - interpolation_point->back().s() >
      ds / kTwo) {
    interpolation_point->push_back(discretized_path.back());
  } else {
    interpolation_point->back() = discretized_path.back();
  }

  return interpolation_point->size() >
         static_cast<size_t>(config_.min_point_num());
}

void NlpPathSmoother::CalculateXYRoadBound(
    const std::vector<common::PathPoint>& interpolation_point,
    const std::vector<std::pair<common::math::LineSegment2d, double>>&
        obj_segments,
    std::vector<std::pair<common::math::LineSegment2d,
                          common::math::LineSegment2d>>* const xy_road_bound)
    const {
  DCHECK(nullptr != xy_road_bound);
  xy_road_bound->clear();
  xy_road_bound->reserve(interpolation_point.size());

  const double front_center_length =
      vehicle_param_.ultrasonic_position().ultrasonic_s1().point().x() +
      config_.lon_collision_buffer();
  const double back_center_length =
      std::fabs(
          vehicle_param_.ultrasonic_position().ultrasonic_s7().point().x()) +
      config_.lon_collision_buffer();
  const double lat_width = vehicle_param_.width() / 2.0 + kLatExtendBuffer;
  for (const auto& p : interpolation_point) {
    const common::math::Vec2d back_center_point(
        p.x() - back_center_length * std::cos(p.theta()),
        p.y() - back_center_length * std::sin(p.theta()));
    const common::math::Vec2d front_center_point(
        p.x() + front_center_length * std::cos(p.theta()),
        p.y() + front_center_length * std::sin(p.theta()));
    common::math::LineSegment2d back_to_front(back_center_point,
                                              front_center_point);
    const auto adc_box =
        common::math::Box2d(back_to_front, vehicle_param_.width());
    double filter_distance = 0.0;
    common::math::Polygon2d left_roi;
    common::math::Polygon2d right_roi;
    CalculateROI(back_to_front, lat_width, &filter_distance, &left_roi,
                 &right_roi);
    double left_min_distance = lat_width;
    double right_min_distance = lat_width;
    for (const auto& seg : obj_segments) {
      if (common::math::double_type::DefinitelyLessEqual(
              seg.second, std::numeric_limits<double>::epsilon())) {
        continue;
      }
      if (seg.first.DistanceTo(
              {back_to_front.center().x(), back_to_front.center().y()}) >
          filter_distance) {
        continue;
      }
      if (left_roi.HasOverlap(seg.first)) {
        left_min_distance =
            std::min(left_min_distance,
                     std::max(0.0, adc_box.DistanceTo(seg.first) - seg.second));
      } else if (right_roi.HasOverlap(seg.first)) {
        right_min_distance =
            std::min(right_min_distance,
                     std::max(0.0, adc_box.DistanceTo(seg.first) - seg.second));
      }
    }
    const double left_shift_distance =
        vehicle_param_.width() / 2.0 + left_min_distance;
    const double right_shift_distance =
        vehicle_param_.width() / 2.0 + right_min_distance;
    std::pair<common::math::LineSegment2d, common::math::LineSegment2d>
        xy_bound_pair;
    back_to_front.Translate(right_shift_distance,
                            back_to_front.heading() - M_PI_2);
    xy_bound_pair.first = back_to_front;
    back_to_front.Translate(right_shift_distance + left_shift_distance,
                            back_to_front.heading() + M_PI_2);
    xy_bound_pair.second = back_to_front;
    xy_road_bound->push_back(std::move(xy_bound_pair));
  }
}

void NlpPathSmoother::CalculateNlpInputInfo(
    const std::vector<PathGearPair>& partition_paths,
    const std::vector<std::vector<
        std::pair<common::math::LineSegment2d, common::math::LineSegment2d>>>&
        xy_road_bounds,
    const bool enable_fix_start_kappa,
    const std::tuple<std::pair<double, double>, bool, bool>&
        dest_lat_region_constrain,
    NlpInputParam* const nlp_input_param) const {
  DCHECK(nullptr != nlp_input_param);
  nlp_input_param->config = config_;
  nlp_input_param->path_point_size.clear();
  for (const auto& partition_path : partition_paths) {
    nlp_input_param->path_point_size.push_back(partition_path.first.size());
    nlp_input_param->total_point_size += partition_path.first.size();
  }
  nlp_input_param->enable_fix_start_kappa = enable_fix_start_kappa;
  nlp_input_param->enable_dest_lat_region_constrain =
      std::get<2>(dest_lat_region_constrain);

  const double max_beta_angle = common::VehicleStateProvider::EstimateBetaAngle(
      vehicle_param_.max_steer_angle() - kSteerAngleBuffer, vehicle_param_);
  nlp_input_param->max_kappa =
      fabs(std::tan(max_beta_angle) / vehicle_param_.wheel_base());
  nlp_input_param->front_collision_length =
      vehicle_param_.ultrasonic_position().ultrasonic_s1().point().x() +
      config_.lon_collision_buffer();
  nlp_input_param->back_collision_length =
      std::fabs(
          vehicle_param_.ultrasonic_position().ultrasonic_s7().point().x()) +
      config_.lon_collision_buffer();
  nlp_input_param->half_collision_width =
      vehicle_param_.ultrasonic_position().ultrasonic_s6().point().y();

  nlp_input_param->kappa_weight = config_.kappa_weight();
  nlp_input_param->dkappa_weight = config_.dkappa_weight();
  nlp_input_param->bias_weight = config_.bias_weight();
  nlp_input_param->origin_point =
      common::math::Vec2d(partition_paths.front().first.front().x(),
                          partition_paths.front().first.front().y());
  nlp_input_param->refine_paths.clear();
  nlp_input_param->refine_paths.reserve(partition_paths.size());
  double refine_theta = 0.0;
  for (size_t i = 0; i < partition_paths.size(); ++i) {
    std::vector<std::tuple<double, double, double, double>> refine_path;
    refine_path.reserve(partition_paths[i].first.size());
    for (size_t j = 0; j < partition_paths[i].first.size(); ++j) {
      const double refine_x =
          partition_paths[i].first[j].x() - nlp_input_param->origin_point.x();
      const double refine_y =
          partition_paths[i].first[j].y() - nlp_input_param->origin_point.y();
      if (i == 0 && j == 0) {
        refine_theta = partition_paths[i].first[j].theta();
      } else {
        double pre_point_theta = 0.0;
        if (j == 0) {
          pre_point_theta = partition_paths[i - 1].first.back().theta();
        } else {
          pre_point_theta = partition_paths[i].first[j - 1].theta();
        }
        const double dtheta = common::math::NormalizeAngle(
            partition_paths[i].first[j].theta() - pre_point_theta);
        refine_theta += dtheta;
      }
      const double refine_kappa = common::util::BoundedValue(
          -nlp_input_param->max_kappa, nlp_input_param->max_kappa,
          partition_paths[i].first[j].kappa());
      refine_path.emplace_back(refine_x, refine_y, refine_theta, refine_kappa);
    }

    nlp_input_param->refine_paths.push_back(std::move(refine_path));
  }

  nlp_input_param->constraint_lines.clear();
  nlp_input_param->constraint_lines.reserve(
      nlp_input_param->refine_paths.size());
  for (size_t i = 0; i < nlp_input_param->refine_paths.size(); ++i) {
    std::vector<NlpInputParam::ConstraintLine> lines;
    lines.reserve(nlp_input_param->refine_paths[i].size());
    for (size_t j = 0; j < nlp_input_param->refine_paths[i].size(); ++j) {
      const auto& p = nlp_input_param->refine_paths[i][j];
      const auto center_vec =
          common::math::Vec2d(std::get<0>(p), std::get<1>(p));
      const auto unit_vec =
          common::math::Vec2d::CreateUnitVec2d(std::get<2>(p) + M_PI_2);
      double left_moving_buffer = config_.lat_moving_buffer();
      double right_moving_buffer = config_.lat_moving_buffer();
      if (std::get<1>(dest_lat_region_constrain) &&
          i == nlp_input_param->refine_paths.size() - 1 &&
          j == nlp_input_param->refine_paths[i].size() - 1) {
        left_moving_buffer = std::min(
            left_moving_buffer, std::get<0>(dest_lat_region_constrain).first);
        right_moving_buffer = std::min(
            right_moving_buffer, std::get<0>(dest_lat_region_constrain).second);
      }
      auto start = center_vec + left_moving_buffer * unit_vec;
      auto end = center_vec - right_moving_buffer * unit_vec;
      NlpInputParam::ConstraintLine line;
      line.equation_a = end.y() - start.y();
      line.equation_b = start.x() - end.x();
      line.equation_c = end.x() * start.y() - start.x() * end.y();
      line.min_x = std::min(start.x(), end.x());
      line.max_x = std::max(start.x(), end.x());
      line.min_y = std::min(start.y(), end.y());
      line.max_y = std::max(start.y(), end.y());
      lines.push_back(line);
    }

    nlp_input_param->constraint_lines.push_back(std::move(lines));
  }

  nlp_input_param->refine_road_bounds.clear();
  nlp_input_param->refine_road_bounds.reserve(xy_road_bounds.size());
  for (const auto& road_bound : xy_road_bounds) {
    std::vector<
        std::pair<common::math::LineSegment2d, common::math::LineSegment2d>>
        refine_bound;
    refine_bound.reserve(road_bound.size());
    for (const auto& bound : road_bound) {
      const common::math::LineSegment2d lower_seg(
          bound.first.start() - nlp_input_param->origin_point,
          bound.first.end() - nlp_input_param->origin_point);
      const common::math::LineSegment2d upper_seg(
          bound.second.start() - nlp_input_param->origin_point,
          bound.second.end() - nlp_input_param->origin_point);
      refine_bound.emplace_back(lower_seg, upper_seg);
    }
    nlp_input_param->refine_road_bounds.push_back(std::move(refine_bound));
  }
}

void NlpPathSmoother::InterpolationByS(
    const std::vector<PathGearPair>& optimized_paths,
    std::vector<PathGearPair>* const final_path) const {
  DCHECK(nullptr != final_path);
  if (optimized_paths.empty()) {
    return;
  }

  final_path->clear();
  final_path->reserve(optimized_paths.size());
  for (const auto& path : optimized_paths) {
    const double ds =
        std::min(config_.interpolation_s(),
                 (path.first.back().s() - path.first.front().s()) /
                     config_.min_point_num());
    if (ds < config_.output_path_ds()) {
      final_path->emplace_back(path.first, path.second);
      continue;
    }

    DiscretizedPath interpolation_path;
    DiscretizedPath discretized_path(path.first);
    interpolation_path.push_back(discretized_path.front());
    for (double s = config_.output_path_ds(); s < discretized_path.Length();
         s += config_.output_path_ds()) {
      auto evaluate_point = discretized_path.Evaluate(s);
      interpolation_path.push_back(std::move(evaluate_point));
    }
    if (discretized_path.back().s() - interpolation_path.back().s() >
        config_.output_path_ds() / 3.0) {
      interpolation_path.push_back(discretized_path.back());
    } else {
      interpolation_path.back() = discretized_path.back();
    }

    final_path->emplace_back(interpolation_path, path.second);
  }
}

void NlpPathSmoother::CalculateROI(
    const common::math::LineSegment2d& back_to_front, const double lat_width,
    double* const filter_distance, common::math::Polygon2d* const left_roi,
    common::math::Polygon2d* const right_roi) {
  DCHECK(nullptr != filter_distance);
  DCHECK(nullptr != left_roi);
  DCHECK(nullptr != right_roi);
  const common::math::Vec2d center_point = back_to_front.center();
  auto start_to_end = back_to_front;
  start_to_end.Translate(lat_width, back_to_front.heading() + M_PI_2);

  *filter_distance = center_point.DistanceTo(start_to_end.end());
  *left_roi =
      common::math::Polygon2d({back_to_front.start(), back_to_front.end(),
                               start_to_end.end(), start_to_end.start()});
  start_to_end.Translate(2.0 * lat_width, back_to_front.heading() - M_PI_2);
  *right_roi =
      common::math::Polygon2d({back_to_front.start(), back_to_front.end(),
                               start_to_end.end(), start_to_end.start()});
}

bool NlpPathSmoother::SolveProblem(const NlpInputParam& nlp_input_param,
                                   std::vector<double>* const optimum) {
  CHECK_NOTNULL(optimum);
  optimum->clear();

  bool success = false;
  if (!nlp_input_param.config.enable_automatic_differentiation()) {
    auto* smoother = new NlpMathModel(nlp_input_param);  // NOLINT
    Ipopt::SmartPtr<Ipopt::TNLP> problem = smoother;
    Ipopt::SmartPtr<Ipopt::IpoptApplication> app(IpoptApplicationFactory());
    app->Options()->SetIntegerValue("print_level",
                                    nlp_input_param.config.print_level());
    app->Options()->SetNumericValue("max_cpu_time",
                                    nlp_input_param.config.max_cpu_time());
    app->Options()->SetIntegerValue("max_iter",
                                    nlp_input_param.config.max_iter_num());
    app->Options()->SetNumericValue("tol", nlp_input_param.config.tol());
    app->Options()->SetNumericValue("acceptable_tol",
                                    nlp_input_param.config.acceptable_tol());
    // app->Options()->SetStringValue("mu_strategy", "adaptive");
    Ipopt::ApplicationReturnStatus status = app->Initialize();
    if (status != Ipopt::Solve_Succeeded) {
      AWARN << "Ipopt failed during initialization.";
      return false;
    }
    status = app->OptimizeTNLP(problem);
    success = (status == Ipopt::Solve_Succeeded ||
               status == Ipopt::Solved_To_Acceptable_Level);
    Ipopt::Index iter_count = app->Statistics()->IterationCount();
    ADEBUG << "*** The problem solved in " << iter_count << " iterations!"
           << ", status: " << status;
    if (success) {
      smoother->optimized_result(optimum);
    }
  } else {
    success = NlpMathModelCppAD::IpoptSolveWithCppAD(nlp_input_param, optimum);
  }

  ADEBUG << "SolveProblemWithIpopt success: " << success;
  return success;
}

}  // namespace planning
}  // namespace TL
