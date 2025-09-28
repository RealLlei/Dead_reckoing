/*
 * Copyright (c) TL Technologies Co., Ltd. 2023. All rights reserved.
 * Description:  ipopt_pos_optimize_smoother.h
 */

#pragma once

#include <tuple>
#include <utility>
#include <vector>

#include "planning/common/path/discretized_path.h"
#include "planning/math/discretized_points_smoothing/ipopt_pos_optimize_math_model_param.h"
#include "planning/proto/ipopt_pos_optimize_smoother_config.pb.h"
#include "proto/common/pnc_point.pb.h"
#include "proto/common/vehicle_config.pb.h"

namespace TL::planning {

class IpoptPosOptimizeSmoother final {
 public:
  /**
   * @brief Construct a new Nlp Path Smoother object
   * 
   * @param config 
   */
  explicit IpoptPosOptimizeSmoother(IpoptPosOptimizeSmootherConfig config);

  /**
   * @brief Destroy the Nlp Path Smoother object
   * 
   */
  virtual ~IpoptPosOptimizeSmoother() = default;

  /**
   * @brief NlpSolver
   * 
   * @param raw_path_point 
   * @param xy_lower_upper_bounds 
   * @param is_forward_path 
   * @param enable_fix_start_kappa 
   * @param enable_fix_end_state 
   * @param frenet_ds 
   * @param smooth_path 
   * @return true 
   * @return false 
   */
  bool NlpSolver(
      const std::vector<common::PathPoint>& raw_path_point,
      const std::pair<std::vector<common::math::Vec2d>,
                      std::vector<common::math::Vec2d>>& xy_lower_upper_bounds,
      bool is_forward_path, bool enable_fix_start_kappa,
      bool enable_fix_end_state, double frenet_ds,
      DiscretizedPath* smooth_path) const;

  /**
   * @brief RoughPathProcessor
   * 
   * @param init_kappa_constrain 
   * @param xytheta_vector 
   * @param is_forward_path 
   * @param rough_path 
   * @return true 
   * @return false 
   */
  static bool RoughPathProcessor(
      const std::pair<double, bool>& init_kappa_constrain,
      const std::vector<std::tuple<double, double, double>>& xytheta_vector,
      bool is_forward_path, std::vector<common::PathPoint>* rough_path);

 private:
  /**
   * @brief PathPointsInterpolationByS
   * 
   * @param raw_path_point 
   * @param sample_ds 
   * @param interpolation_point 
   * @return true 
   * @return false 
   */
  bool PathPointsInterpolationByS(
      const std::vector<common::PathPoint>& raw_path_point, double* sample_ds,
      std::vector<common::PathPoint>* interpolation_point) const;

  /**
   * @brief CalculateNlpInputInfo
   * 
   * @param refer_points 
   * @param xy_lower_upper_bounds 
   * @param is_forward_path 
   * @param enable_fix_start_kappa 
   * @param enable_fix_end_state 
   * @param ref_ds 
   * @param nlp_input_param 
   */
  void CalculateNlpInputInfo(
      const std::vector<common::PathPoint>& refer_points,
      const std::pair<std::vector<common::math::Vec2d>,
                      std::vector<common::math::Vec2d>>& xy_lower_upper_bounds,
      bool is_forward_path, bool enable_fix_start_kappa,
      bool enable_fix_end_state, double ref_ds,
      IpoptPosOptimizeMathModelParam* nlp_input_param) const;

  const IpoptPosOptimizeSmootherConfig config_;
  const common::VehicleParam vehicle_param_;
  const double half_width_;
};

}  // namespace TL::planning
