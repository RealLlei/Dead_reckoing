/*
 * Copyright (c) TL auto Co., Ltd. 2021-2022. All rights reserved.
 */
#include <algorithm>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>
#include "common/configs/vehicle_config_helper.h"
#include "common/filters/digital_filter.h"
#include "common/filters/digital_filter_coefficients.h"
#include "common/status/status.h"
#include "planning/common/frame.h"
#include "planning/common/obstacle.h"
#include "planning/localview/hdmap_avp_state/hdmap_avp_state.h"
#include "planning/localview/lane_line_builder/lane_line_base.h"
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_logical_decider.h"

namespace TL {
namespace planning {
struct SelectFunnel {
  double outer_left;
  double outer_right;
  double inner_left;
  double inner_right;
  double wide_inner_left;
  double wide_inner_right;
  double wide_outer_left;
  double wide_outer_right;
};

struct TargetOverlapInfos {
  double overlap;
  double overlap_var;
  double obstalce_occupancy;
  double obstalce_occupancy_var;
  double trajectory_occupancy;
  double trajectory_occupancy_var;
};

struct OverlapInfos {
  TargetOverlapInfos current_path_overlap_infos;
  TargetOverlapInfos adjacent_path_overlap_infos;
};

struct SelectTrajectoryInfos {
  double trajectory_width;
  double trajectory_width_var;
};

struct TargetInfos {
  double lat_distance;
  double lat_distance_var;
  double obstacle_width;
  double obstacle_width_var;
  double obstacle_corridor_width;
  double obstacle_corridor_width_var;
};

class TargetOverlap {
 public:
  TargetOverlap() = default;
  explicit TargetOverlap(const PerceptionMapConfig& config);
  ~TargetOverlap() = default;

  static std::string Name() { return "TargetOverlap"; }

  Status Init();
  static Status Start();
  void Stop();

  bool Process(const TargetInfos& target_infos,
               const SelectFunnel& select_funnel, OverlapInfos* overlap_infos);

 private:
  static double CalculateGaussErrorFunction(double value);
  static double MultAdd(double a, double b, double d);
  static double CalculateStdGaussianCumulateDistance(double value,
                                                     double average,
                                                     double sigma);
  static TargetOverlapInfos Overlap(
      const SelectTrajectoryInfos& trajectory_infos,
      const TargetInfos& target_infos);
  static void SetTrajectoryInfos(double left_width, double right_width,
                                 double left_width_variance,
                                 double right_width_variance,
                                 SelectTrajectoryInfos* trajectory_infos);
  static void InitTargetOverlapInfos(TargetOverlapInfos* target_overlap_infos);

 private:
  PerceptionMapConfig navigation_hdmap_config_;
  common::VehicleParam vehicle_param_;
  bool overlap_ = true;
};
}  // namespace planning
}  // namespace TL
