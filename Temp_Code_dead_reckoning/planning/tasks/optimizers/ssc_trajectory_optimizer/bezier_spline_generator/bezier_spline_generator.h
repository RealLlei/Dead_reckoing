/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file bezier_spline_generator.h
 **/

#include <array>
#include <vector>

#include "osqp/glob_opts.h"
#include "osqp/osqp.h"
#include "planning/tasks/optimizers/ssc_trajectory_optimizer/bezier_spline_generator/bezier_spline.h"
#include "planning/tasks/optimizers/ssc_trajectory_optimizer/bezier_spline_generator/bezier_utils.h"
#include "planning/tasks/optimizers/ssc_trajectory_optimizer/ssc_map/semantics.h"

namespace TL::planning {

template <int N_DEG, int N_DIM>
class BezierSplineGenerator {
 public:
  using BezierSplineType = BezierSpline<N_DEG, N_DIM>;

  /**
   * @brief Return the optimal bezier curve in corridor
   * @note typical usage parameterization is 2D-t, the corridor is 3D (2D+t)
   * @param start_constraints, pos, vel, acc, etc constraints at start point
   * @param end_constraints, pos, vel, acc, etc constrainst at end point
   */
  bool GetBezierSplineUsingCorridor(
      const std::vector<SpatioTemporalSemanticCubeNd<N_DIM>>& cubes,
      const std::vector<std::array<double, N_DIM>>& start_constraints,
      const std::vector<std::array<double, N_DIM>>& end_constraints,
      const std::vector<double>& ref_stamps,
      const std::vector<std::array<double, N_DIM>>& ref_points,
      BezierSplineType* bezier_spline);

  OSQPSettings* SolverDefaultSettings();

  bool CalculateKernel(
      const std::vector<SpatioTemporalSemanticCubeNd<N_DIM>>& cubes,
      const std::vector<double>& ref_stamps,
      const std::vector<std::array<double, N_DIM>>& ref_points,
      std::vector<c_float>* p_data, std::vector<c_int>* p_indices,
      std::vector<c_int>* p_row_indices, std::vector<c_float>* q);

  bool CalculateConstraint(
      const std::vector<SpatioTemporalSemanticCubeNd<N_DIM>>& cubes,
      const std::vector<std::array<double, N_DIM>>& start_constraints,
      const std::vector<std::array<double, N_DIM>>& end_constraints,
      std::vector<c_float>* a_data, std::vector<c_int>* a_indices,
      std::vector<c_int>* a_row_indices, std::vector<c_float>* lower_bounds,
      std::vector<c_float>* upper_bounds);
};

template <typename T>
T* CopyData(const std::vector<T>& vec) {
  T* data = new T[vec.size()];  //NOLINT
  memcpy(data, vec.data(), sizeof(T) * vec.size());
  return data;
}

void FreeOsqpData(OSQPData* data, OSQPSettings* settings,
                  OSQPWorkspace* osqp_work);

}  // namespace TL::planning
