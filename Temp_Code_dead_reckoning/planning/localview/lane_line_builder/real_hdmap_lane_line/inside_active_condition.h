/******************************************************************************
 * Copyright 2022 The TL Authors. All Rights Reserved.
 *****************************************************************************/
#pragma once
#include <memory>
#include <tuple>
#include <utility>
#include <vector>

#include "planning/common/planning_gflags.h"
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/decider_data.h"
#include "planning/localview/local_view.h"
#include "planning/localview/localview_comdata_manager.h"
#include "planning/pnc_map/pnc_map.h"

#include "planning/proto/hmi_config.pb.h"
#include "proto/fsm/function_manager.pb.h"
#include "proto/routing/routing.pb.h"

namespace TL {
namespace planning {

using TL::planning::lanelineprocess::DebounceModule;

/**
 * @brief deal inside active condition
 */
class InsideActiveCondition {
 public:
  using HmiDataType = std::vector<std::tuple<double, double>>;

  explicit InsideActiveCondition(
      const std::shared_ptr<LocalViewData>& local_view_data);
  InsideActiveCondition() = default;
  ~InsideActiveCondition() = default;
  /**
   * @brief update inside active condition
   * @param is_hd_map 
   * @param pnc_map 
   * @param to_fct 
   */
  void Process(const std::shared_ptr<LocalView>& local_view,
               const std::shared_ptr<hdmap::PncMap>& pnc_map,
               functionmanager::FunctionManagerOut* to_fct,
               bool is_hd_map = false);

  void MapFusionProcess(const std::shared_ptr<LocalView>& local_view,
                        const std::shared_ptr<hdmap::PncMap>& pnc_map,
                        const std::shared_ptr<hdmap::HDMap>& hd_map,
                        functionmanager::FunctionManagerOut* to_fct,
                        bool is_percep_map = false);

 private:
  void OddDeal(const std::shared_ptr<LocalView>& local_view,
               const std::shared_ptr<hdmap::PncMap>& pnc_map,
               functionmanager::FunctionManagerOut* to_fct) const;
  std::pair<bool, bool> CheckVehicleHeadingAndReverseLane(
      const std::shared_ptr<LocalView>& local_view,
      const hdmap::LaneWaypoint& adc_waypoint);
  bool CheckCurvature(const std::shared_ptr<LocalView>& local_view,
                      const hdmap::LaneWaypoint& adc_waypoint,
                      functionmanager::FunctionManagerOut* to_fct);
  bool CheckCurvatureAndVelocity(
      const std::shared_ptr<LocalView>& local_view,
      const std::pair<double, double>& curvature_velocity,
      functionmanager::FunctionManagerOut* to_fct);
  bool CheckCruiseCurvature(const std::shared_ptr<LocalView>& local_view,
                            const std::shared_ptr<hdmap::HDMap>& hd_map,
                            const hdmap::LaneWaypoint& adc_waypoint);
  static void AddPoints(std::vector<Vec2d>* map_points,
                        const TL::hdmap::LaneInfoConstPtr& lane,
                        const common::Pose& pose);
  static Vec2d PointEarth2Bus(const Vec2d& point, const common::Pose& pose);
  HmiDataType hmi_curvature_velocity_{};
  functionmanager::HmiConfig hmi_config_{};
  DebounceModule not_in_forbidlane_debounce_{2.0, 0.5, 0.1};
  DebounceModule current_lane_curve_debounce_{2.0, 0.5, 0.1};
  DebounceModule current_lane_headingerr_debounce_{2.0, 0.5, 0.1};
  DebounceModule current_lane_c0_debounce_{0.3, 0.0, 0.1};
  DebounceModule cruise_lane_c0_debounce_{0.3, 0.2, 0.1};
  DebounceModule headingerr_fit_debounce_{0.5, 0.0, 0.1};
  std::shared_ptr<LocalViewData> local_view_data_ = nullptr;
};

}  // namespace planning
}  // namespace TL
