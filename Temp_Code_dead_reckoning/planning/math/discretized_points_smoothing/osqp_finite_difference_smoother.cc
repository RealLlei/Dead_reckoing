/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  planning osqp finite differfence interface smoother
 * Author: ROC
 */

#include "planning/math/discretized_points_smoothing/osqp_finite_difference_smoother.h"
#include <cstddef>
#include <utility>

#include "planning/common/planning_gflags.h"
#include "planning/math/discrete_points_math.h"
#include "planning/reference_line/util/reference_line_debug.h"

namespace TL {
namespace planning {
OsqpFiniteDifferenceSmoother::OsqpFiniteDifferenceSmoother(OsqpConfig config)
    : config_(std::move(config)) {}

bool OsqpFiniteDifferenceSmoother::Solve(
    const std::vector<std::pair<double, double>>& raw_point2d,
    const AnchorpointsInformation& anchorpoints_information,
    std::vector<double>* const opt_x, std::vector<double>* const opt_y,
    common::ReferenceLine* const reference_line_debug, bool openspace_status) {
  return QpWithOsqp(raw_point2d, anchorpoints_information, opt_x, opt_y,
                    reference_line_debug, openspace_status);
}

bool OsqpFiniteDifferenceSmoother::QpWithOsqp(
    const std::vector<std::pair<double, double>>& raw_point2d,
    // first lat, second lon, third is_stitch, forth is_back_unsmooth
    const AnchorpointsInformation& anchorpoints_information,
    std::vector<double>* const opt_x, std::vector<double>* const opt_y,
    common::ReferenceLine* const reference_line_debug, bool openspace_status) {
  if (opt_x == nullptr || opt_y == nullptr) {
    AERROR << "opt_x or opt_y is nullptr";
    return false;
  }

  OsqpFiniteDifferenceInterface solver;
  std::vector<c_float> eval_x;
  std::vector<c_float> eval_y;
  std::vector<c_float> eval_heading;
  // first left, second right
  std::vector<std::pair<c_float, c_float>> lat;
  std::vector<c_float> lon;
  std::vector<c_float> accumulated_s;
  std::vector<c_float> kappas;
  std::vector<c_float> dkappas;
  std::vector<c_float> solution;
  std::vector<std::pair<double, double>> solution_pair;

  // Init Vector size
  eval_x.reserve(raw_point2d.size());
  eval_y.reserve(raw_point2d.size());
  eval_heading.reserve(raw_point2d.size());
  lat.reserve(raw_point2d.size());
  lon.reserve(raw_point2d.size());
  accumulated_s.reserve(raw_point2d.size());
  kappas.reserve(raw_point2d.size());
  dkappas.reserve(raw_point2d.size());
  solution.reserve(2 * raw_point2d.size());
  solution_pair.reserve(raw_point2d.size());
  // Settings of osqp
  solver.SetOsqpVerbose(config_.verbose());
  solver.SetOsqpEpsAbs(config_.eps_abs());
  solver.SetOsqpEpsRel(config_.eps_rel());
  solver.SetOsqpEpsPrimInf(config_.eps_prim_inf());
  solver.SetOsqpEpsDualInf(config_.eps_dual_inf());
  solver.SetOsqpScaledTermination(config_.scaled_termination());
  solver.SetOsqpEnableAdaptiveRho(config_.adaptive_rho());
  solver.SetOsqpPolish(config_.polish());
  solver.SetOsqpPolishRefineIter(config_.polish_refine_iter());
  solver.SetOsqpMaxIter(config_.max_iter());
  solver.SetOsqpAlpha(config_.alpha());
  solver.SetOsqpAdaptiveRhoInterval(config_.adaptive_rho_interval());
  solver.SetOsqpTimeLimit(config_.time_limit());
  // Weights in optimization cost function
  solver.SetWeightFirstOrder1st(config_.first_order_1st());
  solver.SetWeightFirstOrder2nd(config_.first_order_2nd());
  solver.SetWeightFirstOrder4th(config_.first_order_4th());
  solver.SetWeightSecondOrder2nd(config_.second_order_2nd());
  solver.SetWeightSecondOrder4th(config_.second_order_4th());
  solver.SetWeightThirdOrder1st(config_.third_order_1st());
  solver.SetWeightThirdOrder2nd(config_.third_order_2nd());
  solver.SetWeightFourthOrder2nd(config_.fourth_order_2nd());
  solver.SetWeightLatDeviation(config_.lat_deviation());
  solver.SetWeightLonDeviation(config_.lon_deviation());
  solver.SetWeightEulerDeviation(config_.euler_deviation());
  solver.SetWeightCtWeight(config_.ct_weight());
  solver.SetWeightCurvature(config_.curvature());
  solver.SetWeightsQpPenMaxIter(config_.sqp_pen_max_iter());
  solver.SetWeightsQpSubMaxIter(config_.sqp_sub_max_iter());
  solver.SetWeightsSqpFtol(config_.sqp_ftol());

  // Debug: calculate raw_point2d profile
  if (FLAGS_enable_reference_line_debug) {
    DiscretePointsMath::ComputeDiscretePointsProfile(
        raw_point2d, &eval_x, &eval_y, &eval_heading, &accumulated_s, &kappas,
        &dkappas);
  } else {
    DiscretePointsMath::ComputeDiscretePointsProfile(raw_point2d, &eval_x,
                                                     &eval_y, &eval_heading);
  }

  ACHECK(raw_point2d.size() == raw_point2d.size());

  if (FLAGS_enable_reference_line_cost_segment || openspace_status) {
    for (size_t i = 0; i < raw_point2d.size(); ++i) {
      lat.emplace_back(std::get<0>(anchorpoints_information[i]),
                       std::get<1>(anchorpoints_information[i]));
      lon.emplace_back(std::get<2>(anchorpoints_information[i]));
    }
  } else {
    for (size_t i = 0; i < raw_point2d.size(); ++i) {
      lat.emplace_back(config_.l_boundary(), config_.l_boundary());
      lon.emplace_back(config_.s_boundary());
    }
    lat[0] =
        std::make_pair(config_.l_first_boundary(), config_.l_first_boundary());
    lat[raw_point2d.size() - 1] =
        std::make_pair(config_.l_end_boundary(), config_.l_end_boundary());
    lon[0] = config_.s_first_boundary();
    lon[raw_point2d.size() - 1] = config_.s_end_boundary();
  }

  if (FLAGS_enable_reference_line_debug) {
    ReferenceLineDebug::RawPointDebug(raw_point2d, anchorpoints_information,
                                      lat, lon, eval_heading, accumulated_s,
                                      kappas, dkappas, reference_line_debug);
  }
  if (eval_x.size() < 5) {
    AERROR << "Raw reference point size < 5!, size: " << eval_x.size();
    return false;
  }
  const auto sqp =
      solver.Smooth(eval_x, eval_y, eval_heading, lon, lat, &solution);

  switch (sqp) {
    case -1:
      AERROR << "Smooth initial_res calculate error!";
      return false;
    case 0:
      ADEBUG << "No SQP was used or SQP was failed at the very first time";
      break;
    default:
      ADEBUG << "SQP was successful for: " << sqp << " times";
      break;
  }

  DiscretePointsMath::SolutionToOptXY(solution, opt_x, opt_y, &solution_pair);

  // Debug: calculate solution profile
  if (FLAGS_enable_reference_line_debug) {
    DiscretePointsMath::ComputeDiscretePointsProfile(
        solution_pair, &eval_x, &eval_y, &eval_heading, &accumulated_s, &kappas,
        &dkappas);
    ReferenceLineDebug::SmoothPointDebug(
        solution_pair, anchorpoints_information, eval_heading, accumulated_s,
        kappas, dkappas, reference_line_debug);
  }
  return true;
}
}  // namespace planning
}  // namespace TL
