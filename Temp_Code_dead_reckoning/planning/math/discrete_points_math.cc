/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  planning discrete points math calculate
 * Author: ROC
 */

#include "planning/math/discrete_points_math.h"
#include <cstddef>

#include "common/file/log.h"

namespace TL {
namespace planning {
namespace DiscretePointsMath {

void SolutionToOptXY(const std::vector<c_float>& solution,
                     std::vector<c_float>* const opt_x,
                     std::vector<c_float>* const opt_y,
                     std::vector<std::pair<double, double>>* const xy_points) {
  size_t i = 0;
  size_t j = 0;
  (*opt_x).resize(solution.size() / 2);
  (*opt_y).resize(solution.size() / 2);
  for (i = 0; i < solution.size(); i += 2) {
    (*opt_x)[j] = solution[i];
    (*opt_y)[j] = solution[i + 1];
    xy_points->emplace_back((*opt_x)[j], (*opt_y)[j]);
    ++j;
  }
}

bool ComputeDiscretePointsProfile(
    const std::vector<std::pair<double, double>>& xy_points,
    std::vector<c_float>* const eval_x, std::vector<c_float>* const eval_y,
    std::vector<c_float>* const eval_heading,
    std::vector<c_float>* const accumulated_s,
    std::vector<c_float>* const kappas, std::vector<c_float>* const dkappas) {
  eval_x->clear();
  eval_y->clear();
  eval_heading->clear();
  accumulated_s->clear();
  kappas->clear();
  dkappas->clear();

  if (xy_points.size() < 2) {
    AERROR << "xy_points size < 2!";
    return false;
  }
  std::vector<c_float> dxs;
  std::vector<c_float> dys;
  std::vector<c_float> y_over_s_first_derivatives;
  std::vector<c_float> x_over_s_first_derivatives;
  std::vector<c_float> y_over_s_second_derivatives;
  std::vector<c_float> x_over_s_second_derivatives;

  dxs.reserve(xy_points.size());
  dys.reserve(xy_points.size());
  y_over_s_first_derivatives.reserve(xy_points.size());
  x_over_s_first_derivatives.reserve(xy_points.size());
  y_over_s_second_derivatives.reserve(xy_points.size());
  x_over_s_second_derivatives.reserve(xy_points.size());

  // Get finite difference approximated dx and dy for heading and kappa
  // calculation
  std::size_t points_size = xy_points.size();
  for (std::size_t i = 0; i < points_size; ++i) {
    c_float x_delta = 0.0;
    c_float y_delta = 0.0;
    if (i == 0) {
      x_delta = (xy_points[i + 1].first - xy_points[i].first);
      y_delta = (xy_points[i + 1].second - xy_points[i].second);
    } else if (i == points_size - 1) {
      x_delta = (xy_points[i].first - xy_points[i - 1].first);
      y_delta = (xy_points[i].second - xy_points[i - 1].second);
    } else {
      x_delta = 0.5 * (xy_points[i + 1].first - xy_points[i - 1].first);
      y_delta = 0.5 * (xy_points[i + 1].second - xy_points[i - 1].second);
    }

    dxs.emplace_back(x_delta);
    dys.emplace_back(y_delta);
    eval_x->emplace_back(xy_points[i].first);
    eval_y->emplace_back(xy_points[i].second);
  }

  // Heading calculation
  for (std::size_t i = 0; i < points_size; ++i) {
    eval_heading->emplace_back(std::atan2(dys[i], dxs[i]));
    // ADEBUG << "compute_path_profile_heading: "
    //        << " eval_heading" << i << ": " << eval_heading->at(i);
  }

  // Get linear interpolated s for dkappa calculation
  c_float distance = 0.0;
  accumulated_s->emplace_back(distance);
  c_float fx = xy_points[0].first;
  c_float fy = xy_points[0].second;
  c_float nx = 0.0;
  c_float ny = 0.0;
  for (std::size_t i = 1; i < points_size; ++i) {
    nx = xy_points[i].first;
    ny = xy_points[i].second;
    c_float end_segment_s =
        std::sqrt((fx - nx) * (fx - nx) + (fy - ny) * (fy - ny));
    accumulated_s->emplace_back(end_segment_s + distance);
    distance += end_segment_s;
    fx = nx;
    fy = ny;
  }

  // Get finite difference approximated first derivative of y and x respective
  // to s for kappa calculation
  for (std::size_t i = 0; i < points_size; ++i) {
    c_float xds = 0.0;
    c_float yds = 0.0;
    if (i == 0) {
      xds = (xy_points[i + 1].first - xy_points[i].first) /
            (accumulated_s->at(i + 1) - accumulated_s->at(i));
      yds = (xy_points[i + 1].second - xy_points[i].second) /
            (accumulated_s->at(i + 1) - accumulated_s->at(i));
    } else if (i == points_size - 1) {
      xds = (xy_points[i].first - xy_points[i - 1].first) /
            (accumulated_s->at(i) - accumulated_s->at(i - 1));
      yds = (xy_points[i].second - xy_points[i - 1].second) /
            (accumulated_s->at(i) - accumulated_s->at(i - 1));
    } else {
      xds = (xy_points[i + 1].first - xy_points[i - 1].first) /
            (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
      yds = (xy_points[i + 1].second - xy_points[i - 1].second) /
            (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
    }
    x_over_s_first_derivatives.emplace_back(xds);
    y_over_s_first_derivatives.emplace_back(yds);
  }

  // Get finite difference approximated second derivative of y and x respective
  // to s for kappa calculation
  for (std::size_t i = 0; i < points_size; ++i) {
    c_float xdds = 0.0;
    c_float ydds = 0.0;
    if (i == 0) {
      xdds =
          (x_over_s_first_derivatives[i + 1] - x_over_s_first_derivatives[i]) /
          (accumulated_s->at(i + 1) - accumulated_s->at(i));
      ydds =
          (y_over_s_first_derivatives[i + 1] - y_over_s_first_derivatives[i]) /
          (accumulated_s->at(i + 1) - accumulated_s->at(i));
    } else if (i == points_size - 1) {
      xdds =
          (x_over_s_first_derivatives[i] - x_over_s_first_derivatives[i - 1]) /
          (accumulated_s->at(i) - accumulated_s->at(i - 1));
      ydds =
          (y_over_s_first_derivatives[i] - y_over_s_first_derivatives[i - 1]) /
          (accumulated_s->at(i) - accumulated_s->at(i - 1));
    } else {
      xdds = (x_over_s_first_derivatives[i + 1] -
              x_over_s_first_derivatives[i - 1]) /
             (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
      ydds = (y_over_s_first_derivatives[i + 1] -
              y_over_s_first_derivatives[i - 1]) /
             (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
    }
    x_over_s_second_derivatives.emplace_back(xdds);
    y_over_s_second_derivatives.emplace_back(ydds);
  }
  for (std::size_t i = 0; i < points_size; ++i) {
    // kappa = (dx * d2y - dy * d2x) / [(dx * dx + dy * dy)^(3/2)]
    // kappa = (xds * ydds - yds * xdds) / (std::sqrt(xds * xds + yds * yds) *
    // (xds * xds + yds * yds) + 1e-6);
    c_float xds = x_over_s_first_derivatives[i];
    c_float yds = y_over_s_first_derivatives[i];
    const c_float xdds = x_over_s_second_derivatives[i];
    const c_float ydds = y_over_s_second_derivatives[i];
    auto norm_square = xds * xds + yds * yds;
    auto norm = std::sqrt(norm_square);
    const c_float a = xds * ydds - yds * xdds;
    const c_float b = norm * norm_square;
    const c_float kappa = a / b;
    kappas->emplace_back(kappa);
  }

  // Dkappa calculation
  for (std::size_t i = 0; i < points_size; ++i) {
    c_float dkappa = 0.0;
    if (i == 0) {
      dkappa = (kappas->at(i + 1) - kappas->at(i)) /
               (accumulated_s->at(i + 1) - accumulated_s->at(i));
    } else if (i == points_size - 1) {
      dkappa = (kappas->at(i) - kappas->at(i - 1)) /
               (accumulated_s->at(i) - accumulated_s->at(i - 1));
    } else {
      dkappa = (kappas->at(i + 1) - kappas->at(i - 1)) /
               (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
    }
    dkappas->emplace_back(dkappa);
  }
  return true;
}

bool ComputeDiscretePointsProfile(
    const std::vector<std::pair<double, double>>& xy_points,
    std::vector<c_float>* const eval_x, std::vector<c_float>* const eval_y,
    std::vector<c_float>* const eval_heading) {
  eval_x->clear();
  eval_y->clear();
  eval_heading->clear();

  if (xy_points.size() < 2) {
    AERROR << "xy_points size < 2!";
    return false;
  }
  std::vector<c_float> dxs;
  std::vector<c_float> dys;

  dxs.reserve(xy_points.size());
  dys.reserve(xy_points.size());

  // Get finite difference approximated dx and dy for heading and kappa
  // calculation
  std::size_t points_size = xy_points.size();
  for (std::size_t i = 0; i < points_size; ++i) {
    c_float x_delta = 0.0;
    c_float y_delta = 0.0;
    if (i == 0) {
      x_delta = (xy_points[i + 1].first - xy_points[i].first);
      y_delta = (xy_points[i + 1].second - xy_points[i].second);
    } else if (i == points_size - 1) {
      x_delta = (xy_points[i].first - xy_points[i - 1].first);
      y_delta = (xy_points[i].second - xy_points[i - 1].second);
    } else {
      x_delta = 0.5 * (xy_points[i + 1].first - xy_points[i - 1].first);
      y_delta = 0.5 * (xy_points[i + 1].second - xy_points[i - 1].second);
    }

    dxs.emplace_back(x_delta);
    dys.emplace_back(y_delta);
    eval_x->emplace_back(xy_points[i].first);
    eval_y->emplace_back(xy_points[i].second);
  }

  // Heading calculation
  for (std::size_t i = 0; i < points_size; ++i) {
    eval_heading->emplace_back(std::atan2(dys[i], dxs[i]));
    // ADEBUG << "compute_path_profile_heading: "
    //        << " eval_heading" << i << ": " << eval_heading->at(i);
  }
  return true;
}

bool ComputeDiscretePointsProfile(
    const std::vector<std::pair<double, double>>& xy_points,
    std::vector<double>* headings, std::vector<double>* accumulated_s,
    std::vector<double>* kappas, std::vector<double>* dkappas) {
  CHECK_NOTNULL(headings);
  CHECK_NOTNULL(kappas);
  CHECK_NOTNULL(dkappas);
  CHECK_NOTNULL(accumulated_s);
  headings->clear();
  kappas->clear();
  dkappas->clear();
  accumulated_s->clear();

  if (xy_points.size() < 2) {
    AERROR << "xy_points size < 2!";
    return false;
  }
  std::vector<double> dxs;
  std::vector<double> dys;
  std::vector<double> y_over_s_first_derivatives;
  std::vector<double> x_over_s_first_derivatives;
  std::vector<double> y_over_s_second_derivatives;
  std::vector<double> x_over_s_second_derivatives;

  dxs.reserve(xy_points.size());
  dys.reserve(xy_points.size());
  y_over_s_first_derivatives.reserve(xy_points.size());
  x_over_s_first_derivatives.reserve(xy_points.size());
  y_over_s_second_derivatives.reserve(xy_points.size());
  x_over_s_second_derivatives.reserve(xy_points.size());

  // Get finite difference approximated dx and dy for heading and kappa
  // calculation
  std::size_t points_size = xy_points.size();
  for (std::size_t i = 0; i < points_size; ++i) {
    double x_delta = 0.0;
    double y_delta = 0.0;
    if (i == 0) {
      x_delta = (xy_points[i + 1].first - xy_points[i].first);
      y_delta = (xy_points[i + 1].second - xy_points[i].second);
    } else if (i == points_size - 1) {
      x_delta = (xy_points[i].first - xy_points[i - 1].first);
      y_delta = (xy_points[i].second - xy_points[i - 1].second);
    } else {
      x_delta = 0.5 * (xy_points[i + 1].first - xy_points[i - 1].first);
      y_delta = 0.5 * (xy_points[i + 1].second - xy_points[i - 1].second);
    }
    dxs.push_back(x_delta);
    dys.push_back(y_delta);
  }

  // Heading calculation
  for (std::size_t i = 0; i < points_size; ++i) {
    headings->push_back(std::atan2(dys[i], dxs[i]));
  }

  // Get linear interpolated s for dkappa calculation
  double distance = 0.0;
  accumulated_s->push_back(distance);
  double fx = xy_points[0].first;
  double fy = xy_points[0].second;
  double nx = 0.0;
  double ny = 0.0;
  for (std::size_t i = 1; i < points_size; ++i) {
    nx = xy_points[i].first;
    ny = xy_points[i].second;
    double end_segment_s =
        std::sqrt((fx - nx) * (fx - nx) + (fy - ny) * (fy - ny));
    accumulated_s->push_back(end_segment_s + distance);
    distance += end_segment_s;
    fx = nx;
    fy = ny;
  }

  // Get finite difference approximated first derivative of y and x respective
  // to s for kappa calculation
  for (std::size_t i = 0; i < points_size; ++i) {
    double xds = 0.0;
    double yds = 0.0;
    if (i == 0) {
      xds = (xy_points[i + 1].first - xy_points[i].first) /
            (accumulated_s->at(i + 1) - accumulated_s->at(i));
      yds = (xy_points[i + 1].second - xy_points[i].second) /
            (accumulated_s->at(i + 1) - accumulated_s->at(i));
    } else if (i == points_size - 1) {
      xds = (xy_points[i].first - xy_points[i - 1].first) /
            (accumulated_s->at(i) - accumulated_s->at(i - 1));
      yds = (xy_points[i].second - xy_points[i - 1].second) /
            (accumulated_s->at(i) - accumulated_s->at(i - 1));
    } else {
      xds = (xy_points[i + 1].first - xy_points[i - 1].first) /
            (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
      yds = (xy_points[i + 1].second - xy_points[i - 1].second) /
            (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
    }
    x_over_s_first_derivatives.push_back(xds);
    y_over_s_first_derivatives.push_back(yds);
  }

  // Get finite difference approximated second derivative of y and x respective
  // to s for kappa calculation
  for (std::size_t i = 0; i < points_size; ++i) {
    double xdds = 0.0;
    double ydds = 0.0;
    if (i == 0) {
      xdds =
          (x_over_s_first_derivatives[i + 1] - x_over_s_first_derivatives[i]) /
          (accumulated_s->at(i + 1) - accumulated_s->at(i));
      ydds =
          (y_over_s_first_derivatives[i + 1] - y_over_s_first_derivatives[i]) /
          (accumulated_s->at(i + 1) - accumulated_s->at(i));
    } else if (i == points_size - 1) {
      xdds =
          (x_over_s_first_derivatives[i] - x_over_s_first_derivatives[i - 1]) /
          (accumulated_s->at(i) - accumulated_s->at(i - 1));
      ydds =
          (y_over_s_first_derivatives[i] - y_over_s_first_derivatives[i - 1]) /
          (accumulated_s->at(i) - accumulated_s->at(i - 1));
    } else {
      xdds = (x_over_s_first_derivatives[i + 1] -
              x_over_s_first_derivatives[i - 1]) /
             (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
      ydds = (y_over_s_first_derivatives[i + 1] -
              y_over_s_first_derivatives[i - 1]) /
             (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
    }
    x_over_s_second_derivatives.push_back(xdds);
    y_over_s_second_derivatives.push_back(ydds);
  }

  for (std::size_t i = 0; i < points_size; ++i) {
    double xds = x_over_s_first_derivatives[i];
    double yds = y_over_s_first_derivatives[i];
    double xdds = x_over_s_second_derivatives[i];
    double ydds = y_over_s_second_derivatives[i];
    double kappa =
        (xds * ydds - yds * xdds) /
        (std::sqrt(xds * xds + yds * yds) * (xds * xds + yds * yds) + 1e-6);
    kappas->push_back(kappa);
  }

  // Dkappa calculation
  for (std::size_t i = 0; i < points_size; ++i) {
    double dkappa = 0.0;
    if (i == 0) {
      dkappa = (kappas->at(i + 1) - kappas->at(i)) /
               (accumulated_s->at(i + 1) - accumulated_s->at(i));
    } else if (i == points_size - 1) {
      dkappa = (kappas->at(i) - kappas->at(i - 1)) /
               (accumulated_s->at(i) - accumulated_s->at(i - 1));
    } else {
      dkappa = (kappas->at(i + 1) - kappas->at(i - 1)) /
               (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
    }
    dkappas->push_back(dkappa);
  }
  return true;
}

double ComputeCosineTheoremTheta(const double s_a, const double s_b,
                                 const double s_c) {
  return acos((s_a * s_a + s_b * s_b - s_c * s_c) / (2.0 * s_a * s_b)) / M_PI *
         180.0;
}

double ComputeTwoPointDistance(const double point_a_x, const double point_a_y,
                               const double point_b_x, const double point_b_y) {
  return std::pow(std::pow(point_a_x - point_b_x, 2.0) +
                      std::pow(point_a_y - point_b_y, 2.0),
                  0.5);
}
}  // namespace DiscretePointsMath
}  // namespace planning
}  // namespace TL
