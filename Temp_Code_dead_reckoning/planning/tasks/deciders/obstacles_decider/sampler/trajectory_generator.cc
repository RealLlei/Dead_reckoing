/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2023. All rights reserved.
 * Description:  trajectory_generator.cc
 */

#include "planning/tasks/deciders/obstacles_decider/sampler/trajectory_generator.h"

#include <algorithm>
#include <cstddef>
#include <memory>
#include <set>
#include <string>

#include "common/file/log.h"
#include "common/math/cartesian_frenet_conversion.h"
#include "common/math/path_matcher.h"
#include "planning/common/planning_gflags.h"
#include "planning/tasks/deciders/obstacles_decider/sampler/at_sampler.h"
#include "planning/tasks/deciders/obstacles_decider/sampler/sl_curve.h"
#include "planning/tasks/deciders/obstacles_decider/sampler/sl_sampler.h"

namespace TL {
namespace planning {
namespace {
constexpr double kEpsilon = 1e-5;
}

using TL::common::PathPoint;
using TL::common::TrajectoryPoint;
using TL::common::math::CartesianFrenetConverter;

TrajectoryGenerator::TrajectoryGenerator(const ObstaclesDeciderConfig& config)
    : config_(config) {
  if (config.has_sampling_interval()) {
    sampling_interval_ = config.sampling_interval();
  }
  if (config.has_trajectory_time_resolution()) {
    trajectory_time_resolution_ = config.trajectory_time_resolution();
  }
  if (config.has_at_sampler_config()) {
    at_sampler_ = std::make_unique<AtSampler>(config.at_sampler_config());
  }
  if (config.has_sl_sampler_config()) {
    sl_sampler_ = std::make_unique<SlSampler>(config.sl_sampler_config());
  }
}

void TrajectoryGenerator::Init(const std::array<double, 3>& init_lon_state,
                               const std::array<double, 3>& init_lat_state,
                               const ReferenceLineInfo& reference_line_info) {
  at_sampler_->Init(init_lon_state);
  sl_sampler_->Init(init_lat_state);
  discretized_trajectories_.clear();
  init_lon_state_ = init_lon_state;
  init_lat_state_ = init_lat_state;
  Sample(reference_line_info.reference_line(), sampling_interval_);
}

void TrajectoryGenerator::Sample(const ReferenceLine& reference_line,
                                 const double sampling_interval) {
  if (at_sampler_ == nullptr || sl_sampler_ == nullptr) {
    AERROR << "Sample failed! check at_sampler_ == nullptr || sl_sampler_ == "
              "nullptr";
    return;
  }

  std::vector<std::shared_ptr<SlCurve>> sl_curves;
  std::vector<std::shared_ptr<QuadraticVTCurve>> vt_curves;
  at_sampler_->GenerateLongitudinalTrajectoryBundle(&vt_curves);
  ADEBUG << "avoid alongside decider vt_curves size:" << vt_curves.size();
  sl_sampler_->GenerateLateralTrajectoryBundle(&sl_curves);
  ADEBUG << "avoid alongside decider sl_curves size:" << sl_curves.size();
  const size_t lon_trajectory_size = vt_curves.size();
  const size_t lat_trajectory_size = sl_curves.size();
  discretized_trajectories_.reserve(lon_trajectory_size * lat_trajectory_size);
  for (const auto& lon_trajectory : vt_curves) {
    if (lon_trajectory == nullptr) {
      continue;
    }
    for (const auto& lat_trajectory : sl_curves) {
      if (lat_trajectory == nullptr) {
        continue;
      }
      discretized_trajectories_.emplace_back();
      Combine(reference_line, *lon_trajectory, *lat_trajectory,
              sampling_interval, &discretized_trajectories_.back());
    }
  }
}

void TrajectoryGenerator::Combine(
    const ReferenceLine& reference_line, const QuadraticVTCurve& lon_trajectory,
    const SlCurve& lat_trajectory, const double init_relative_time,
    DiscretizedTrajectory* const combined_trajectory) const {
  if (combined_trajectory == nullptr) {
    return;
  }

  const double s0 = lon_trajectory.GetPoint(0.0).s() + init_lon_state_.at(0);
  // const double s_ref_max = reference_line.back().s();
  const double s_ref_max = reference_line.Length();
  double accumulated_trajectory_s = init_lon_state_.at(0);
  PathPoint prev_trajectory_point;

  double last_s = -FLAGS_numerical_epsilon;
  constexpr double kTrajectoryTimeLength = 7.0;
  int trajectory_point_size = static_cast<int>(
      kTrajectoryTimeLength / std::max(trajectory_time_resolution_, kEpsilon));

  for (int i = 0; i < trajectory_point_size; ++i) {
    double t_param = i * trajectory_time_resolution_;
    double s = lon_trajectory.GetPoint(t_param).s() + init_lon_state_.at(0);
    if (last_s > 0.0) {
      s = std::max(last_s, s);
    }
    last_s = s;

    const double s_dot =
        std::max(FLAGS_numerical_epsilon, lon_trajectory.GetPoint(t_param).v());
    const double s_ddot = lon_trajectory.GetPoint(t_param).a();
    if (s > s_ref_max) {
      break;
    }

    const double relative_s = s - s0;
    const double d = lat_trajectory.Evaluate(0, relative_s);
    const double d_prime = lat_trajectory.Evaluate(1, relative_s);
    const double d_pprime = lat_trajectory.Evaluate(2, relative_s);

    const PathPoint& matched_ref_point =
        reference_line.GetNearestReferencePoint(s).ToPathPoint(s);

    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    double kappa = 0.0;
    double v = 0.0;
    double a = 0.0;

    const double rs = matched_ref_point.s();
    const double rx = matched_ref_point.x();
    const double ry = matched_ref_point.y();
    const double rtheta = matched_ref_point.theta();
    const double rkappa = matched_ref_point.kappa();
    const double rdkappa = matched_ref_point.dkappa();

    std::array<double, 3> s_conditions = {rs, s_dot, s_ddot};
    std::array<double, 3> d_conditions = {d, d_prime, d_pprime};
    CartesianFrenetConverter::frenet_to_cartesian(
        rs, rx, ry, rtheta, rkappa, rdkappa, s_conditions, d_conditions, &x, &y,
        &theta, &kappa, &v, &a);

    if (prev_trajectory_point.has_x() && prev_trajectory_point.has_y()) {
      double delta_x = x - prev_trajectory_point.x();
      double delta_y = y - prev_trajectory_point.y();
      double delta_s = std::hypot(delta_x, delta_y);
      accumulated_trajectory_s += delta_s;
    }

    TrajectoryPoint* trajectory_point = combined_trajectory->add();
    trajectory_point->mutable_path_point()->set_x(x);
    trajectory_point->mutable_path_point()->set_y(y);
    trajectory_point->mutable_path_point()->set_s(accumulated_trajectory_s);
    trajectory_point->mutable_path_point()->set_l(d);
    trajectory_point->mutable_path_point()->set_theta(theta);
    trajectory_point->mutable_path_point()->set_kappa(kappa);
    trajectory_point->set_v(v);
    trajectory_point->set_a(a);
    trajectory_point->set_relative_time(t_param + init_relative_time);
    if (combined_trajectory->size() == 1) {
      trajectory_point->set_da(0);
    } else if (combined_trajectory->size() > 1) {
      trajectory_point->set_da(
          (trajectory_point->a() - combined_trajectory->back().a()) /
          std::max(trajectory_time_resolution_, kEpsilon));
    }

    prev_trajectory_point = trajectory_point->path_point();
  }
}

}  // namespace planning
}  // namespace TL
