/*
 * Copyright (c) TL Technologies Co., Ltd. 2022. All rights reserved.
 * Description:  nlp_path_smoother.h
 */

#pragma once

#include <tuple>
#include <utility>
#include <vector>

#include "common/configs/vehicle_config_helper.h"
#include "common/status/status.h"
#include "planning/common/open_space_info.h"
#include "planning/open_space/nlp_path_smoother/nlp_input_param.h"
#include "planning/proto/planner_open_space_config.pb.h"
#include "proto/common/pnc_point.pb.h"
#include "proto/common/vehicle_config.pb.h"

namespace TL {
namespace planning {

class NlpPathSmoother final {
 public:
  /**
   * @brief Construct a new Nlp Path Smoother object
   * 
   * @param config 
   */
  explicit NlpPathSmoother(NlpPathSmootherConfig config)
      : config_(std::move(config)),
        vehicle_param_(
            common::VehicleConfigHelper::GetConfig().vehicle_param()) {}

  /**
   * @brief Destroy the Nlp Path Smoother object
   * 
   */
  ~NlpPathSmoother() = default;

  /**
   * @brief NlpSolver
   * 
   * @param partition_paths 
   * @param xy_road_bounds 
   * @param enable_fix_start_kappa 
   * @param dest_lat_region_constrain 
   * @param smooth_path 
   * @param force_enable_dest_lat_constrain 
   * @return true 
   * @return false 
   */
  bool NlpSolver(
      const std::vector<PathGearPair>& partition_paths,
      const std::vector<std::vector<
          std::pair<common::math::LineSegment2d, common::math::LineSegment2d>>>&
          xy_road_bounds,
      bool enable_fix_start_kappa,
      const std::tuple<std::pair<double, double>, bool, bool>&
          dest_lat_region_constrain,
      std::vector<PathGearPair>* smooth_path,
      bool* force_enable_dest_lat_constrain) const;
  /**
   * @brief XYRoadPreprocessor
   * 
   * @param raw_partition_path_pairs 
   * @param obj_segments 
   * @param init_kappa_constrain 
   * @param partition_paths 
   * @param xy_road_bounds 
   * @return common::Status 
   */
  common::Status XYRoadPreprocessor(
      const std::vector<PathGearPair>& raw_partition_path_pairs,
      const std::vector<std::pair<common::math::LineSegment2d, double>>&
          obj_segments,
      const std::pair<double, bool>& init_kappa_constrain,
      std::vector<PathGearPair>* partition_paths,
      std::vector<std::vector<
          std::pair<common::math::LineSegment2d, common::math::LineSegment2d>>>*
          xy_road_bounds) const;

 private:
  /**
   * @brief RoughPathProcessor
   * 
   * @param raw_rough_point 
   * @param init_kappa_constrain 
   * @param path_index 
   * @param is_forward 
   * @param modifed_rough_point 
   * @return common::Status 
   */
  static common::Status RoughPathProcessor(
      const std::vector<common::PathPoint>& raw_rough_point,
      const std::pair<double, bool>& init_kappa_constrain,
      size_t total_path_size, size_t path_index, bool* is_forward,
      std::vector<common::PathPoint>* modifed_rough_point);
  /**
   * @brief InterpolationByS
   * 
   * @param raw_path_point 
   * @param interpolation_point 
   * @return true 
   * @return false 
   */
  bool InterpolationByS(
      const std::vector<common::PathPoint>& raw_path_point,
      std::vector<common::PathPoint>* interpolation_point) const;
  /**
   * @brief CalculateXYRoadBound
   * 
   * @param interpolation_point 
   * @param obj_segments 
   * @param xy_road_bound 
   */
  void CalculateXYRoadBound(
      const std::vector<common::PathPoint>& interpolation_point,
      const std::vector<std::pair<common::math::LineSegment2d, double>>&
          obj_segments,
      std::vector<std::pair<common::math::LineSegment2d,
                            common::math::LineSegment2d>>* xy_road_bound) const;
  /**
   * @brief CalculateNlpInputInfo
   * 
   * @param partition_paths 
   * @param xy_road_bounds 
   * @param enable_fix_start_kappa 
   * @param dest_lat_region_constrain 
   * @param nlp_input_param 
   */
  void CalculateNlpInputInfo(
      const std::vector<PathGearPair>& partition_paths,
      const std::vector<std::vector<
          std::pair<common::math::LineSegment2d, common::math::LineSegment2d>>>&
          xy_road_bounds,
      bool enable_fix_start_kappa,
      const std::tuple<std::pair<double, double>, bool, bool>&
          dest_lat_region_constrain,
      NlpInputParam* nlp_input_param) const;
  /**
   * @brief InterpolationByS
   * 
   * @param optimized_paths 
   * @param final_path 
   */
  void InterpolationByS(const std::vector<PathGearPair>& optimized_paths,
                        std::vector<PathGearPair>* final_path) const;
  /**
   * @brief CalculateROI
   * 
   * @param back_to_front 
   * @param lat_width 
   * @param filter_distance 
   * @param left_roi 
   * @param right_roi 
   */
  static void CalculateROI(const common::math::LineSegment2d& back_to_front,
                           double lat_width, double* filter_distance,
                           common::math::Polygon2d* left_roi,
                           common::math::Polygon2d* right_roi);
  /**
   * @brief SolveProblem
   * 
   * @param nlp_input_param 
   * @param optimum 
   * @return true 
   * @return false 
   */
  static bool SolveProblem(const NlpInputParam& nlp_input_param,
                           std::vector<double>* optimum);

  const NlpPathSmootherConfig config_;
  const common::VehicleParam vehicle_param_;
};

}  // namespace planning
}  // namespace TL
