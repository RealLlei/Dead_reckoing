/*
 * Copyright (c) TL Technologies Co., Ltd. 2023. All rights reserved.
 * Description:  ipopt_pos_optimize_smoother.cc
 */

#include "planning/math/discretized_points_smoothing/ipopt_pos_optimize_smoother.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "coin/IpIpoptApplication.hpp"
#include "coin/IpSolveStatistics.hpp"
#include "common/math/math_utils.h"
#include "common/math/vec2d.h"
#include "common/util/util.h"
#include "common/vehicle_state/vehicle_state_provider.h"
#include "glog/logging.h"
#include "planning/math/discretized_points_smoothing/ipopt_pos_optimize_math_model.h"
#include "proto/common/error_code.pb.h"
#include "proto/common/pnc_point.pb.h"

namespace TL::planning {

namespace {
constexpr double kExtendSafetyBuffer = 0.0;
constexpr double kTwo = 2.0;
constexpr double kHalfOne = 0.5;
}  // namespace

IpoptPosOptimizeSmoother::IpoptPosOptimizeSmoother(
    IpoptPosOptimizeSmootherConfig config)
    : config_(std::move(config)),
      vehicle_param_(common::VehicleConfigHelper::GetConfig().vehicle_param()),
      half_width_(vehicle_param_.width() / kTwo) {}

bool IpoptPosOptimizeSmoother::NlpSolver(
    const std::vector<common::PathPoint>& raw_path_point,
    const std::pair<std::vector<common::math::Vec2d>,
                    std::vector<common::math::Vec2d>>& xy_lower_upper_bounds,
    const bool is_forward_path, const bool enable_fix_start_kappa,
    const bool enable_fix_end_state, const double frenet_ds,
    DiscretizedPath* const smooth_path) const {
  DCHECK(nullptr != smooth_path);
  if (raw_path_point.size() < 2) {
    AERROR << "IpoptPosOptimizeSmoother raw_path_point size: "
           << raw_path_point.size() << " less than 2.";
    return false;
  }
  if ((!xy_lower_upper_bounds.first.empty() ||
       !xy_lower_upper_bounds.second.empty()) &&
      (xy_lower_upper_bounds.first.size() !=
           xy_lower_upper_bounds.second.size() ||
       raw_path_point.size() != xy_lower_upper_bounds.first.size())) {
    AERROR << "IpoptPosOptimizeSmoother raw_path_point.size: "
           << raw_path_point.size()
           << ", xy_lower_bound.size: " << xy_lower_upper_bounds.first.size()
           << ", xy_upper_bound.size: " << xy_lower_upper_bounds.second.size();
    return false;
  }
  IpoptPosOptimizeMathModelParam nlp_input_param;
  if (xy_lower_upper_bounds.first.empty()) {
    double ds = 0.0;
    std::vector<common::PathPoint> interpolation_point;
    if (!PathPointsInterpolationByS(raw_path_point, &ds,
                                    &interpolation_point)) {
      AERROR << "IpoptPosOptimizeSmoother PathPointsInterpolationByS "
                "failed.";
      return false;
    }
    CalculateNlpInputInfo(interpolation_point, xy_lower_upper_bounds,
                          is_forward_path, enable_fix_start_kappa,
                          enable_fix_end_state, ds, &nlp_input_param);
  } else {
    CalculateNlpInputInfo(raw_path_point, xy_lower_upper_bounds,
                          is_forward_path, enable_fix_start_kappa,
                          enable_fix_end_state, frenet_ds, &nlp_input_param);
  }

  auto* smoother = new IpoptPosOptimizeMathModel(nlp_input_param);  // NOLINT
  Ipopt::SmartPtr<Ipopt::TNLP> problem = smoother;
  Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();
  app->Options()->SetIntegerValue("max_iter", config_.max_iter_num());
  app->Options()->SetIntegerValue("print_level", config_.print_level());
  app->Options()->SetNumericValue("max_cpu_time", config_.max_cpu_time());
  app->Options()->SetNumericValue("tol", config_.tol());
  app->Options()->SetNumericValue("acceptable_tol", config_.acceptable_tol());
  Ipopt::ApplicationReturnStatus status = app->Initialize();
  if (status != Ipopt::Solve_Succeeded) {
    AWARN << "Ipopt failed during initialization.";
    return false;
  }
  status = app->OptimizeTNLP(problem);
  bool success = (status == Ipopt::Solve_Succeeded ||
                  status == Ipopt::Solved_To_Acceptable_Level);
  Ipopt::Index iter_count = app->Statistics()->IterationCount();
  ADEBUG << "*** The problem solved in " << iter_count << " iterations!";
  if (success) {
    std::vector<double> optimum;
    smoother->optimized_result(&optimum);

    const double sign = is_forward_path ? 1.0 : -1.0;
    smooth_path->clear();
    smooth_path->reserve(nlp_input_param.path_point_size);
    for (int32_t i = 0; i < nlp_input_param.path_point_size; ++i) {
      common::PathPoint tmp;
      tmp.set_x(optimum.at(i) + nlp_input_param.origin_point.x());
      tmp.set_y(optimum.at(nlp_input_param.path_point_size + i) +
                nlp_input_param.origin_point.y());
      tmp.set_theta(common::math::NormalizeAngle(
          optimum.at(2 * nlp_input_param.path_point_size + i)));
      tmp.set_kappa(sign * optimum.at(3 * nlp_input_param.path_point_size + i));
      if (i == 0) {
        tmp.set_s(0.0);
      } else {
        const double dx = tmp.x() - smooth_path->back().x();
        const double dy = tmp.y() - smooth_path->back().y();
        tmp.set_s(smooth_path->back().s() + std::sqrt(dx * dx + dy * dy));
      }

      smooth_path->emplace_back(std::move(tmp));
    }
  }

  return success && smooth_path->size() >= 2;
}

bool IpoptPosOptimizeSmoother::RoughPathProcessor(
    const std::pair<double, bool>& init_kappa_constrain,
    const std::vector<std::tuple<double, double, double>>& xytheta_vector,
    const bool is_forward_path,
    std::vector<common::PathPoint>* const rough_path) {
  if (rough_path == nullptr || xytheta_vector.size() < 2) {
    const std::string msg =
        "rough_path is nullptr: " +
        std::to_string(static_cast<int>(rough_path == nullptr)) +
        ", xytheta_vector.size(): " + std::to_string(xytheta_vector.size());
    AERROR << msg;
    return false;
  }

  rough_path->clear();
  rough_path->reserve(xytheta_vector.size());
  for (size_t i = 0; i < xytheta_vector.size(); i++) {
    common::PathPoint tmp;
    tmp.set_x(std::get<0>(xytheta_vector[i]));
    tmp.set_y(std::get<1>(xytheta_vector[i]));
    tmp.set_theta(std::get<2>(xytheta_vector[i]));
    if (i == 0) {
      tmp.set_kappa(0.0);
      tmp.set_s(0.0);
    } else {
      double ds = std::hypot(std::fabs(tmp.x() - rough_path->back().x()),
                             std::fabs(tmp.y() - rough_path->back().y()));
      constexpr double kMinDistance = 0.05;
      if (ds < kMinDistance) {
        continue;
      }
      double dtheta = common::math::NormalizeAngle(tmp.theta() -
                                                   rough_path->back().theta());
      tmp.set_kappa(dtheta / ds);
      tmp.set_s(rough_path->back().s() + ds);
    }
    rough_path->push_back(tmp);
  }
  if (rough_path->size() < 2) {
    const std::string msg =
        "rough_path->size(): " + std::to_string(rough_path->size());
    AERROR << msg;
    return false;
  }
  if (init_kappa_constrain.second) {
    const double sign = is_forward_path ? 1.0 : -1.0;
    rough_path->at(0).set_kappa(sign * init_kappa_constrain.first);
  } else {
    double dtheta = common::math::NormalizeAngle(rough_path->at(1).theta() -
                                                 rough_path->at(0).theta());
    rough_path->at(0).set_kappa(dtheta / rough_path->at(1).s());
  }

  return true;
}

bool IpoptPosOptimizeSmoother::PathPointsInterpolationByS(
    const std::vector<common::PathPoint>& raw_path_point,
    double* const sample_ds,
    std::vector<common::PathPoint>* const interpolation_point) const {
  if (raw_path_point.size() < 2 || sample_ds == nullptr ||
      interpolation_point == nullptr) {
    AERROR << "PathPointsInterpolationByS interpolation_point==nullptr or "
              "raw_path_point.size: "
           << raw_path_point.size() << " is less than 2.";
    return false;
  }

  interpolation_point->clear();
  const double path_length =
      raw_path_point.back().s() - raw_path_point.front().s();
  *sample_ds = std::min(config_.interpolation_s(),
                        path_length / config_.min_point_num());
  ADEBUG << "PathPointsInterpolationByS sample_ds: " << sample_ds
         << ", length: " << path_length;
  DiscretizedPath discretized_path(raw_path_point);
  interpolation_point->push_back(discretized_path.front());
  for (double s = *sample_ds; s < path_length; s += *sample_ds) {
    auto evaluate_point =
        discretized_path.Evaluate(discretized_path.front().s() + s);
    interpolation_point->push_back(std::move(evaluate_point));
  }
  if (discretized_path.back().s() - interpolation_point->back().s() >
      *sample_ds / kTwo) {
    interpolation_point->push_back(discretized_path.back());
  } else {
    interpolation_point->back() = discretized_path.back();
  }

  return interpolation_point->size() >
         static_cast<size_t>(config_.min_point_num());
}

void IpoptPosOptimizeSmoother::CalculateNlpInputInfo(
    const std::vector<common::PathPoint>& refer_points,
    const std::pair<std::vector<common::math::Vec2d>,
                    std::vector<common::math::Vec2d>>& xy_lower_upper_bounds,
    const bool is_forward_path, const bool enable_fix_start_kappa,
    const bool enable_fix_end_state, const double ref_ds,
    IpoptPosOptimizeMathModelParam* const nlp_input_param) const {
  DCHECK(nullptr != nlp_input_param);
  DCHECK_GE(refer_points.size(), 2U);
  DCHECK_EQ(xy_lower_upper_bounds.first.size(),
            xy_lower_upper_bounds.second.size());
  if (!xy_lower_upper_bounds.first.empty()) {
    DCHECK_EQ(refer_points.size(), xy_lower_upper_bounds.first.size());
  }
  nlp_input_param->path_point_size = static_cast<int32_t>(refer_points.size());
  nlp_input_param->enable_fix_start_kappa = enable_fix_start_kappa;
  nlp_input_param->enable_fix_end_state = enable_fix_end_state;
  nlp_input_param->is_forward_path = is_forward_path;
  const double max_beta_angle = common::VehicleStateProvider::EstimateBetaAngle(
      vehicle_param_.max_steer_angle() - config_.steer_angle_buffer(),
      vehicle_param_);
  nlp_input_param->max_kappa =
      std::tan(max_beta_angle) / vehicle_param_.wheel_base();
  ADEBUG << "nlp_input_param->max_kappa: " << nlp_input_param->max_kappa;
  nlp_input_param->bias_weight = config_.bias_weight();
  nlp_input_param->kappa_weight = config_.kappa_weight();
  nlp_input_param->dkappa_weight = config_.dkappa_weight();

  nlp_input_param->origin_point =
      common::math::Vec2d(refer_points.front().x(), refer_points.front().y());
  nlp_input_param->refine_path_point.clear();
  nlp_input_param->refine_path_point.reserve(refer_points.size());
  double refine_x = 0.0;
  double refine_y = 0.0;
  double refine_theta = 0.0;
  double refine_kappa = 0.0;
  for (int32_t i = 0; i < nlp_input_param->path_point_size; i++) {
    refine_x = refer_points[i].x() - nlp_input_param->origin_point.x();
    refine_y = refer_points[i].y() - nlp_input_param->origin_point.y();
    if (i == 0) {
      refine_theta = refer_points[0].theta();
    } else {
      double dtheta = common::math::NormalizeAngle(refer_points[i].theta() -
                                                   refer_points[i - 1].theta());
      refine_theta += dtheta;
    }
    refine_kappa = common::util::BoundedValue(-nlp_input_param->max_kappa,
                                              nlp_input_param->max_kappa,
                                              refer_points[i].kappa());
    nlp_input_param->refine_path_point.emplace_back(refine_x, refine_y,
                                                    refine_theta, refine_kappa);
  }
  nlp_input_param->is_collision_free = xy_lower_upper_bounds.first.empty();
  nlp_input_param->min_distance_threshold = half_width_ + kExtendSafetyBuffer;
  nlp_input_param->front_delta_index = static_cast<int32_t>(
      lround(vehicle_param_.wheel_base() / ref_ds + kHalfOne));
  if (!nlp_input_param->is_collision_free) {
    nlp_input_param->xy_lower_upper_bounds.first.clear();
    nlp_input_param->xy_lower_upper_bounds.second.clear();
    nlp_input_param->xy_lower_upper_bounds.first.reserve(refer_points.size());
    nlp_input_param->xy_lower_upper_bounds.second.reserve(refer_points.size());
    for (int32_t i = 0; i < nlp_input_param->path_point_size; i++) {
      refine_x = xy_lower_upper_bounds.first[i].x() -
                 nlp_input_param->origin_point.x();
      refine_y = xy_lower_upper_bounds.first[i].y() -
                 nlp_input_param->origin_point.y();
      nlp_input_param->xy_lower_upper_bounds.first.emplace_back(refine_x,
                                                                refine_y);

      refine_x = xy_lower_upper_bounds.second[i].x() -
                 nlp_input_param->origin_point.x();
      refine_y = xy_lower_upper_bounds.second[i].y() -
                 nlp_input_param->origin_point.y();
      nlp_input_param->xy_lower_upper_bounds.second.emplace_back(refine_x,
                                                                 refine_y);
    }
  }

  nlp_input_param->constraint_line.clear();
  nlp_input_param->constraint_line.reserve(refer_points.size());
  for (int32_t i = 0; i < nlp_input_param->path_point_size; ++i) {
    const auto& p = nlp_input_param->refine_path_point[i];
    const auto center_vec = common::math::Vec2d(std::get<0>(p), std::get<1>(p));
    const auto unit_vec =
        common::math::Vec2d::CreateUnitVec2d(std::get<2>(p) + M_PI_2);
    double left_moving_buffer = config_.lat_moving_buffer();
    double right_moving_buffer = config_.lat_moving_buffer();
    if (!nlp_input_param->is_collision_free) {
      const auto& lp = nlp_input_param->xy_lower_upper_bounds.second[i];
      const auto& rp = nlp_input_param->xy_lower_upper_bounds.first[i];
      left_moving_buffer =
          std::max(lp.DistanceTo(center_vec) - half_width_, 0.0);
      right_moving_buffer =
          std::max(rp.DistanceTo(center_vec) - half_width_, 0.0);
    }
    const auto start = center_vec + left_moving_buffer * unit_vec;
    const auto end = center_vec - right_moving_buffer * unit_vec;
    IpoptPosOptimizeMathModelParam::ConstraintLine constraint_line;
    constraint_line.equation_a = end.y() - start.y();
    constraint_line.equation_b = start.x() - end.x();
    constraint_line.equation_c = end.x() * start.y() - start.x() * end.y();
    constraint_line.min_x = std::min(start.x(), end.x());
    constraint_line.max_x = std::max(start.x(), end.x());
    constraint_line.min_y = std::min(start.y(), end.y());
    constraint_line.max_y = std::max(start.y(), end.y());
    nlp_input_param->constraint_line.push_back(constraint_line);
  }
}

}  // namespace TL::planning
