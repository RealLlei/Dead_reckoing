#ifndef PLANNING_COMMON_OPEN_SPACE_CHART_H
#define PLANNING_COMMON_OPEN_SPACE_CHART_H

//  Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.

#pragma once
#include <memory>
#include <string>
#include <vector>
#include "planning/common/dependency_injector.h"
#include "planning/common/frame.h"
#include "proto/planning/planning.pb.h"
#include "proto/planning/planning_internal.pb.h"

namespace TL {
namespace planning {
class OpenSpaceChart {
 public:
  OpenSpaceChart(Frame* frame,
                 const std::shared_ptr<DependencyInjector>& injector);
  void ExportOpenSpaceChart(const planning_internal::Debug& debug_info,
                            const planning::ADCTrajectory& trajectory_pb,
                            planning_internal::Debug* debug_chart);

 private:
  void UpdateRadarPos(dreamview::Line* radar_dot,
                      const std::vector<Vec2d>& points);
  void AddOpenSpaceOptimizerResult(const planning_internal::Debug& debug_info,
                                   planning_internal::Debug* debug_chart);
  void AddOpenSpaceMultiSearchResult(const planning_internal::Debug& debug_info,
                                     planning_internal::Debug* debug_chart);
  /**
   * @brief 画泊车chart
   *
   * @param debug_info 输入planning debug信息
   * @param debug_chart 输出chart信息
   */
  void AddPartitionedPath(const planning_internal::Debug& debug_info,
                          planning_internal::Debug* debug_chart);

  /**
   * @brief draw published speed
   *
   * @param trajectory_pb
   * @param debug_chart
   */
  static void AddPublishedSpeed(const planning::ADCTrajectory& trajectory_pb,
                                planning_internal::Debug* debug_chart);

  /**
   * @brief draw published acc
   *
   * @param trajectory_pb
   * @param debug
   */
  static void AddPublishedAcceleration(
      const planning::ADCTrajectory& trajectory_pb,
      planning_internal::Debug* debug);
  /**
   * @brief draw speed limits info in sv graph
   *
   * @param trajectory_pb
   * @param debug_chart
   */
  static void AddSpeedPlanDebugInfo(
      const planning::ADCTrajectory& trajectory_pb,
      planning_internal::Debug* debug_chart);

  static void PopulateChartOptions(double x_min, double x_max,
                                   const std::string& x_label, double y_min,
                                   double y_max, const std::string& y_label,
                                   bool display,
                                   TL::dreamview::Chart* chart);

  /**
   * @brief Update obstacle in chart real time
   *
   * @param chart
   */
  void UpdateObstacleRealTime(dreamview::Chart* chart);

  /**
   * @brief Update obstacle in chart when replan
   *
   * @param debug_info
   * @param chart
   */
  static void UpdateObstacle(const google::protobuf::RepeatedPtrField<
                                 planning_internal::ObstacleDebug>& obstacles,
                             dreamview::Chart* chart);

 private:
  Frame* frame_;
  std::shared_ptr<DependencyInjector> injector_;
};

}  // namespace planning
}  // namespace TL

#endif  // PLANNING_COMMON_OPEN_SPACE_CHART_H
