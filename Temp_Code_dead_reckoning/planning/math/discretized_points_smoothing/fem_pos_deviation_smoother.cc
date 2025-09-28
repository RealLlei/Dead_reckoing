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

/**
 * @file
 **/

#include "planning/math/discretized_points_smoothing/fem_pos_deviation_smoother.h"

#include <cstddef>
#include <memory>

#include "common/file/log.h"
#include "planning/common/planning_gflags.h"
#include "planning/reference_line/util/reference_line_debug.h"

#include "proto/common/pnc_point.pb.h"

namespace TL {
namespace planning {
FemPosDeviationSmoother::FemPosDeviationSmoother(
    const FemPosDeviationSmootherConfig& config)
    : config_(config) {}

bool FemPosDeviationSmoother::Solve(
    const std::vector<std::pair<double, double>>& raw_point2d,
    const std::vector<double>& bounds, std::vector<double>* const opt_x,
    std::vector<double>* const opt_y) {
  if (config_.apply_curvature_constraint()) {
    if (config_.use_sqp()) {
      return SqpWithOsqp(raw_point2d, bounds, opt_x, opt_y);
    } else {
      return QpWithOsqp(raw_point2d, bounds, opt_x, opt_y);
    }
  } else {
    return QpWithOsqp(raw_point2d, bounds, opt_x, opt_y);
  }
  return true;
}

bool FemPosDeviationSmoother::QpWithOsqp(
    const std::vector<std::pair<double, double>>& raw_point2d,
    const std::vector<double>& bounds, std::vector<double>* const opt_x,
    std::vector<double>* const opt_y) {
  if (opt_x == nullptr || opt_y == nullptr) {
    AERROR << "opt_x or opt_y is nullptr";
    return false;
  }

  FemPosDeviationOsqpInterface solver;

  solver.SetWeightFemPosDeviation(config_.weight_fem_pos_deviation());
  solver.SetWeightPathLength(config_.weight_path_length());
  solver.SetWeightRefDeviation(config_.weight_ref_deviation());

  solver.SetMaxIter(config_.max_iter());
  solver.SetTimeLimit(config_.time_limit());
  solver.SetVerbose(config_.verbose());
  solver.SetScaledTermination(config_.scaled_termination());
  solver.SetWarmStart(config_.warm_start());

  solver.SetRefPoints(raw_point2d);
  solver.SetBoundsAroundRefs(bounds);

  if (FLAGS_enable_reference_line_debug) {
    std::vector<c_float> eval_x;
    std::vector<c_float> eval_y;
    std::vector<c_float> eval_heading;
    std::vector<c_float> accumulated_s;
    std::vector<c_float> kappas;
    std::vector<c_float> dkappas;

    eval_x.reserve(raw_point2d.size());
    eval_y.reserve(raw_point2d.size());
    eval_heading.reserve(raw_point2d.size());
    accumulated_s.reserve(raw_point2d.size());
    kappas.reserve(raw_point2d.size());
    dkappas.reserve(raw_point2d.size());

    auto reference_line_debug = std::make_unique<common::ReferenceLine>();

    reference_line_debug->set_name("OpenSpacePath/0");
    DiscretePointsMath::ComputeDiscretePointsProfile(
        raw_point2d, &eval_x, &eval_y, &eval_heading, &accumulated_s, &kappas,
        &dkappas);
    AnchorpointsInformation anchorpoints_information(
        bounds.size(), {0.0, 0.0, 0.0, false, false});

    std::vector<std::pair<c_float, c_float>> lat_bounds;
    lat_bounds.reserve(bounds.size());
    for (const auto& bound : bounds) {
      lat_bounds.emplace_back(bound, bound);
    }

    ReferenceLineDebug::RawPointDebug(
        raw_point2d, anchorpoints_information, lat_bounds, bounds, eval_heading,
        accumulated_s, kappas, dkappas, reference_line_debug.get(), false);
  }

  if (!solver.Solve()) {
    return false;
  }

  *opt_x = solver.OptX();
  *opt_y = solver.OptY();

  // Debug: calculate solution profile

  if (FLAGS_enable_reference_line_debug) {
    std::vector<c_float> eval_x;
    std::vector<c_float> eval_y;
    std::vector<c_float> eval_heading;
    std::vector<c_float> accumulated_s;
    std::vector<c_float> kappas;
    std::vector<c_float> dkappas;
    std::vector<std::pair<double, double>> opt_xy;

    eval_x.reserve(raw_point2d.size());
    eval_y.reserve(raw_point2d.size());
    eval_heading.reserve(raw_point2d.size());
    accumulated_s.reserve(raw_point2d.size());
    kappas.reserve(raw_point2d.size());
    dkappas.reserve(raw_point2d.size());
    opt_xy.reserve(raw_point2d.size());

    auto reference_line_debug = std::make_unique<common::ReferenceLine>();

    reference_line_debug->set_name("OpenSpacePath/0");
    for (size_t i = 0; i < (*opt_x).size(); ++i) {
      opt_xy.emplace_back((*opt_x).at(i), (*opt_y).at(i));
    }

    DiscretePointsMath::ComputeDiscretePointsProfile(
        opt_xy, &eval_x, &eval_y, &eval_heading, &accumulated_s, &kappas,
        &dkappas);
    AnchorpointsInformation anchorpoints_information(
        bounds.size(), {0.0, 0.0, 0.0, false, false});
    ReferenceLineDebug::SmoothPointDebug(opt_xy, anchorpoints_information,
                                         eval_heading, accumulated_s, kappas,
                                         dkappas, reference_line_debug.get());
  }

  // Debug Log: calculate solution profile
  // for (int i = 0; i < opt_xy.size(); ++i) {
  // ADEBUG << "solution: "
  //        << "opt_x" << i << ": " << opt_xy[i].first << " opt_y" << i << ": "
  //        << opt_xy[i].second << " heading" << i << ": " << eval_heading[i]
  //        << " accumulated_s" << i << ": " << accumulated_s[i] << " kappa" <<
  //        i
  //        << ": " << kappas[i] << " dkappas" << i << ": " << dkappas[i];
  // }

  return true;
}

bool FemPosDeviationSmoother::SqpWithOsqp(
    const std::vector<std::pair<double, double>>& raw_point2d,
    const std::vector<double>& bounds, std::vector<double>* const opt_x,
    std::vector<double>* const opt_y) {
  if (opt_x == nullptr || opt_y == nullptr) {
    AERROR << "opt_x or opt_y is nullptr";
    return false;
  }

  FemPosDeviationSqpOsqpInterface solver;
  std::vector<c_float> eval_x;
  std::vector<c_float> eval_y;
  std::vector<c_float> eval_heading;
  std::vector<c_float> accumulated_s;
  std::vector<c_float> kappas;
  std::vector<c_float> dkappas;
  std::vector<std::pair<double, double>> opt_xy;

  solver.SetWeightFemPosDeviation(config_.weight_fem_pos_deviation());
  solver.SetWeightPathLength(config_.weight_path_length());
  solver.SetWeightRefDeviation(config_.weight_ref_deviation());
  solver.SetWeightCurvatureConstraintSlackVar(
      config_.weight_curvature_constraint_slack_var());

  solver.SetCurvatureConstraint(config_.curvature_constraint());

  solver.SetSqpSubMaxIter(config_.sqp_sub_max_iter());
  solver.SetSqpFtol(config_.sqp_ftol());
  solver.SetSqpPenMaxIter(config_.sqp_pen_max_iter());
  solver.SetSqpCtol(config_.sqp_ctol());

  solver.SetMaxIter(config_.max_iter());
  solver.SetTimeLimit(config_.time_limit());
  solver.SetVerbose(config_.verbose());
  solver.SetScaledTermination(config_.scaled_termination());
  solver.SetWarmStart(config_.warm_start());

  solver.SetRefPoints(raw_point2d);
  solver.SetBoundsAroundRefs(bounds);

  // Debug: calculate raw_point2d profile
  // smoother_calculate.ComputeDiscretePointsProfile(raw_point2d, &eval_x,
  // &eval_y,
  //                                         &eval_heading, &accumulated_s,
  //                                         &kappas, &dkappas);
  // for (int i = 0; i < raw_point2d.size(); ++i) {
  //   // Debug Log: calculate raw_point2d profile
  //   ADEBUG << "raw_point2d: "
  //          << "x" << i << ": " << raw_point2d[i].first << " y" << i << ": "
  //          << raw_point2d[i].second << " heading" << i << ": "
  //          << eval_heading[i] << " accumulated_s" << i << ": "
  //          << accumulated_s[i] << " kappa" << i << ": " << kappas[i]
  //          << " dkappas" << i << ": " << dkappas[i];
  // }

  if (!solver.Solve()) {
    return false;
  }

  opt_xy = solver.OptXY();
  // TODO(Jinyun): unify output data container
  opt_x->resize(opt_xy.size());
  opt_y->resize(opt_xy.size());
  for (size_t i = 0; i < opt_xy.size(); ++i) {
    (*opt_x)[i] = opt_xy[i].first;
    (*opt_y)[i] = opt_xy[i].second;
  }

  // Debug: calculate solution profile
  // smoother_calculate.ComputeDiscretePointsProfile(opt_xy, &eval_x, &eval_y,
  //                                         &eval_heading, &accumulated_s,
  //                                         &kappas, &dkappas);
  // Debug Log: calculate solution profile
  // for (int i = 0; i < opt_xy.size(); ++i) {
  //   ADEBUG << "solution: "
  //          << "opt_x" << i << ": " << opt_xy[i].first << " opt_y" << i << ":
  //          "
  //          << opt_xy[i].second << " heading" << i << ": " << eval_heading[i]
  //          << " accumulated_s" << i << ": " << accumulated_s[i] << " kappa"
  //          << i
  //          << ": " << kappas[i] << " dkappas" << i << ": " << dkappas[i];
  // }
  return true;
}

// bool FemPosDeviationSmoother::NlpWithIpopt(
//     const std::vector<std::pair<double, double>>& raw_point2d,
//     const std::vector<double>& bounds, std::vector<double>* opt_x,
//     std::vector<double>* opt_y) {
//   if (opt_x == nullptr || opt_y == nullptr) {
//     AERROR << "opt_x or opt_y is nullptr";
//     return false;
//   }

//   FemPosDeviationIpoptInterface* smoother =
//       new FemPosDeviationIpoptInterface(raw_point2d, bounds);

//   smoother->SetWeightFemPosDeviation(config_.weight_fem_pos_deviation());
//   smoother->SetWeightPathLength(config_.weight_path_length());
//   smoother->SetWeightRefDeviation(config_.weight_ref_deviation());
//   smoother->SetWeightCurvatureConstraintSlackVar(
//       config_.weight_curvature_constraint_slack_var());
//   smoother->SetCurvatureConstraint(config_.curvature_constraint());

//   Ipopt::SmartPtr<Ipopt::TNLP> problem = smoother;

//   // Create an instance of the IpoptApplication
//   Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();

//   app->Options()->SetIntegerValue("print_level",
//                                   static_cast<int>(config_.print_level()));
//   app->Options()->SetIntegerValue(
//       "max_iter", static_cast<int>(config_.max_num_of_iterations()));
//   app->Options()->SetIntegerValue(
//       "acceptable_iter",
//       static_cast<int>(config_.acceptable_num_of_iterations()));
//   app->Options()->SetNumericValue("tol", config_.tol());
//   app->Options()->SetNumericValue("acceptable_tol",
//   config_.acceptable_tol());

//   Ipopt::ApplicationReturnStatus status = app->Initialize();
//   if (status != Ipopt::Solve_Succeeded) {
//     AERROR << "*** Error during initialization!";
//     return false;
//   }

//   status = app->OptimizeTNLP(problem);

//   if (status == Ipopt::Solve_Succeeded ||
//       status == Ipopt::Solved_To_Acceptable_Level) {
//     // Retrieve some statistics about the solve
//     Ipopt::Index iter_count = app->Statistics()->IterationCount();
//     ADEBUG << "*** The problem solved in " << iter_count << " iterations!";
//   } else {
//     AERROR << "Solver fails with return code: " << static_cast<int>(status);
//     return false;
//   }
//   smoother->get_optimization_results(opt_x, opt_y);
//   return true;
// }

}  // namespace planning
}  // namespace TL
