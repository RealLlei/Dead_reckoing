/*
 * Copyright (c) TL Technologies Co., Ltd. 2023. All rights reserved.
 * Description:  open_space_path_smoother.h
 */

#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "planning/open_space/nlp_path_smoother/nlp_path_smoother.h"
#include "planning/proto/open_space_task_config.pb.h"
#include "proto/planning/planning_internal.pb.h"

namespace TL {
namespace planning {

class OpenSpacePathSmoother {
 public:
  explicit OpenSpacePathSmoother(const NlpPathSmootherConfig& config);

  ~OpenSpacePathSmoother() = default;

  /**
   * @brief Smooth
   * 
   * @param input 
   * @param output 
   */
  void Smooth(const OpenSpacePathInput& input, OpenSpacePathOutput* output);

  /**
   * @brief PathPointNormalizing
   * 
   * @param rotate_angle 
   * @param translate_origin 
   * @param path_point_ptr 
   */
  static void PathPointNormalizing(double rotate_angle,
                                   const Vec2d& translate_origin,
                                   common::PathPoint* path_point_ptr);

  /**
   * @brief UpdateDebugInfo
   * 
   * @param debug 
   */
  void UpdateDebugInfo(planning_internal::OpenSpaceDebug* debug);

 private:
  const NlpPathSmootherConfig config_;
  std::unique_ptr<NlpPathSmoother> nlp_path_smoother_;
  planning_internal::OpenSpaceDebug debug_;
};
}  // namespace planning
}  // namespace TL
