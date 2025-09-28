/*
 * Copyright (c) TL auto Co., Ltd. 2021-2022. All rights reserved.
 */
#include "planning/localview/lane_line_builder/adaptive_cruise_lane_line/target_overlap.h"
#include <unistd.h>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>
#include "common/math/box2d.h"
#include "common/math/vec2d.h"
#include "common/status/status.h"
#include "planning/reference_line/reference_line.h"
#include "proto/common/pnc_point.pb.h"
#include "proto/common/vehicle_config.pb.h"
#include "proto/perception/perception_obstacle.pb.h"

namespace TL {
namespace planning {
TargetOverlap::TargetOverlap(const PerceptionMapConfig& config)
    : navigation_hdmap_config_(config) {}

Status TargetOverlap::Init() {
  vehicle_param_ = common::VehicleConfigHelper::GetConfig().vehicle_param();
  return Status::OK();
}

Status TargetOverlap::Start() {
  return Status::OK();
}

void TargetOverlap::Stop() {}

bool TargetOverlap::Process(const TargetInfos& target_infos,
                            const SelectFunnel& select_funnel,
                            OverlapInfos* overlap_infos) {
  if (overlap_infos == nullptr) {
    overlap_ = false;
    return false;
  }
  SelectTrajectoryInfos trajectory_infos{};
  SetTrajectoryInfos(select_funnel.outer_left, select_funnel.outer_right, 0.0,
                     0.0, &trajectory_infos);
  TargetOverlapInfos target_overlap_infos{};

  target_overlap_infos = Overlap(trajectory_infos, target_infos);
  overlap_infos->current_path_overlap_infos = target_overlap_infos;
  if (target_overlap_infos.obstalce_occupancy > 0.0001) {
    overlap_infos->adjacent_path_overlap_infos.overlap = 0;
    overlap_infos->adjacent_path_overlap_infos.overlap_var = 0;
    overlap_infos->adjacent_path_overlap_infos.trajectory_occupancy =
        1 - overlap_infos->current_path_overlap_infos.trajectory_occupancy;
    overlap_infos->adjacent_path_overlap_infos.trajectory_occupancy_var = 0;
    overlap_infos->adjacent_path_overlap_infos.obstalce_occupancy =
        1 - overlap_infos->current_path_overlap_infos.obstalce_occupancy;
    overlap_infos->adjacent_path_overlap_infos.obstalce_occupancy_var = 0;
  } else {
    InitTargetOverlapInfos(&overlap_infos->current_path_overlap_infos);
    InitTargetOverlapInfos(&target_overlap_infos);
    SetTrajectoryInfos(select_funnel.wide_outer_left,
                       select_funnel.wide_outer_right, 0.0, 0.0,
                       &trajectory_infos);

    target_overlap_infos = Overlap(trajectory_infos, target_infos);
    if (target_overlap_infos.obstalce_occupancy < 0.0001) {
      InitTargetOverlapInfos(&overlap_infos->adjacent_path_overlap_infos);
    } else {
      overlap_infos->adjacent_path_overlap_infos = target_overlap_infos;
    }
  }
  overlap_ = true;
  return true;
}

void TargetOverlap::InitTargetOverlapInfos(
    TargetOverlapInfos* target_overlap_infos) {
  target_overlap_infos->overlap = 0;
  target_overlap_infos->overlap_var = 0;
  target_overlap_infos->trajectory_occupancy = 0;
  target_overlap_infos->trajectory_occupancy_var = 0;
  target_overlap_infos->obstalce_occupancy = 0;
  target_overlap_infos->obstalce_occupancy_var = 0;
}

void TargetOverlap::SetTrajectoryInfos(
    const double left_width, const double right_width,
    const double left_width_variance, const double right_width_variance,
    SelectTrajectoryInfos* trajectory_infos) {
  trajectory_infos->trajectory_width =
      std::abs(left_width) + std::abs(right_width);
  trajectory_infos->trajectory_width_var =
      left_width_variance + right_width_variance;
}

TargetOverlapInfos TargetOverlap::Overlap(
    const SelectTrajectoryInfos& trajectory_infos,
    const TargetInfos& target_infos) {
  TargetOverlapInfos target_overlap_infos{};
  const int k_corner_point_mode = 1;
  const int k_fix_width_mode = 0;
  const double k_delta = 0.0001;
  const int k_obj_width_mode = k_corner_point_mode;
  target_overlap_infos.overlap = 0.0;
  target_overlap_infos.overlap_var = 0.0;
  target_overlap_infos.obstalce_occupancy = 0.0;
  target_overlap_infos.obstalce_occupancy_var = 0.0;
  target_overlap_infos.trajectory_occupancy = 0.0;
  target_overlap_infos.trajectory_occupancy_var = 0.0;

  double a = 0.0;
  double a_half = 0.0;
  double b = 0.0;
  double b_half = 0.0;
  double c = 0.0;
  double a_variance = 0.0;
  double b_variance = 0.0;
  double c_variance = 0.0;
  double abc_variance = 0.0;
  double al_bl_n = 0.0;
  double al_bl_y = 0.0;
  double ar_br_n = 0.0;
  double ar_br_y = 0.0;
  double al_bl_n_x_ar_br_n = 0.0;
  double al_bl_y_x_ar_br_n = 0.0;
  double al_bl_n_x_ar_br_y = 0.0;
  double al_bl_y_x_ar_br_y = 0.0;

  const double trajectory_width_abs =
      std::abs(trajectory_infos.trajectory_width);
  double obstacle_width_abs = std::abs(target_infos.obstacle_corridor_width);
  if (k_obj_width_mode == k_fix_width_mode) {
    obstacle_width_abs = std::abs(target_infos.obstacle_width);
  }

  if ((trajectory_width_abs < k_delta) || (obstacle_width_abs < k_delta)) {
    target_overlap_infos.overlap = 0;
    target_overlap_infos.overlap_var = 0;
    target_overlap_infos.trajectory_occupancy = 0;
    target_overlap_infos.trajectory_occupancy_var = 0;
    target_overlap_infos.obstalce_occupancy = 0;
    target_overlap_infos.obstalce_occupancy_var = 0;
    return target_overlap_infos;
  }
  a = trajectory_infos.trajectory_width;
  a_half = 0.5 * a;
  a_variance = trajectory_infos.trajectory_width_var;
  b = target_infos.obstacle_corridor_width;
  b_half = 0.5 * b;
  b_variance = target_infos.obstacle_corridor_width_var;
  c = target_infos.lat_distance;
  c_variance = target_infos.lat_distance_var;

  abc_variance = (0.25 * a_variance) + (0.25 * b_variance) + c_variance;
  al_bl_n = CalculateStdGaussianCumulateDistance(0.0, (a_half - b_half) + c,
                                                 abc_variance);
  al_bl_y = 1.0 - al_bl_n;
  ar_br_n = CalculateStdGaussianCumulateDistance(0.0, -a_half + b_half + c,
                                                 abc_variance);
  ar_br_y = 1.0 - ar_br_n;

  al_bl_n_x_ar_br_n = al_bl_n * ar_br_n;
  al_bl_y_x_ar_br_n = al_bl_y * ar_br_n;
  al_bl_n_x_ar_br_y = al_bl_n * ar_br_y;
  al_bl_y_x_ar_br_y = al_bl_y * ar_br_y;

  target_overlap_infos.overlap =
      (al_bl_n_x_ar_br_n * (a_half + b_half + c)) + (al_bl_n_x_ar_br_y * a) +
      (al_bl_y_x_ar_br_n * b) + (al_bl_y_x_ar_br_y * ((a_half + b_half) - c));
  target_overlap_infos.overlap_var =
      (al_bl_n_x_ar_br_n * abc_variance) + (al_bl_n_x_ar_br_y * a_variance) +
      (al_bl_y_x_ar_br_n * b_variance) + (al_bl_y_x_ar_br_y * abc_variance);

  target_overlap_infos.trajectory_occupancy =
      (target_overlap_infos.overlap / a);
  target_overlap_infos.trajectory_occupancy_var =
      (target_overlap_infos.overlap_var / std::pow(a, 2));

  target_overlap_infos.obstalce_occupancy = (target_overlap_infos.overlap / b);
  target_overlap_infos.obstalce_occupancy_var =
      (target_overlap_infos.overlap_var / std::pow(b, 2));

  return target_overlap_infos;
}

double TargetOverlap::CalculateStdGaussianCumulateDistance(const double value,
                                                           const double average,
                                                           const double sigma) {
  double temp = 0.0;
  const double k_gaussian_cumulate_distanc_min_sigma = 0.000001;
  if (std::abs(sigma) < k_gaussian_cumulate_distanc_min_sigma) {
    if (value < average) {
      temp = 0.0;
    } else {
      temp = 1.0;
    }
  } else {
    temp = (value - average) / (sigma * std::sqrt(2));
    /* check for negative values */
    if (temp < 0.0) {
      temp = -CalculateGaussErrorFunction(-temp);
    } else {
      temp = CalculateGaussErrorFunction(temp);
    }
    temp = MultAdd(0.5, temp, 0.5);
  }
  return temp;
}

double TargetOverlap::MultAdd(const double a, const double b, const double d) {
  return a * b + d;
}

double TargetOverlap::CalculateGaussErrorFunction(const double value) {
  const double k_gauss_error_function_Max_x = 1.99;
  const double k_gauss_error_function_const4 = 0.08869;
  const double k_gauss_errorfunction_const3 = 0.2841;
  const double k_gauss_error_function_const2 = 0.01092;
  const double k_gauss_error_function_const1 = 1.146;
  const double k_gauss_error_function_const0 = 0.002289;

  double temp = 0.0;
  double temp2 = 0.0;
  double temp3 = 0.0;
  double temp4 = 0.0;

  if (value >= k_gauss_error_function_Max_x) {
    temp = 1.0;
  } else {
    temp2 = value * value;
    /* x^2 */
    temp3 = temp2 * value;
    /* x^3 */
    temp4 = temp2 * temp2;
    /* x^4 */
    temp = ((((k_gauss_error_function_const4 * temp4) -
              (k_gauss_errorfunction_const3 * temp3)) -
             (k_gauss_error_function_const2 * temp2)) +
            (k_gauss_error_function_const1 * value)) +
           k_gauss_error_function_const0;

    temp = std::min(temp, 1.0);
  }
  return temp;
}

}  // namespace planning
}  // namespace TL
