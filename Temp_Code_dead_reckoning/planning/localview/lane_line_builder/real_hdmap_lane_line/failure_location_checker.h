/*
 * Copyright (c) TL auto Co., Ltd. 2022-2023. All rights reserved.
 */
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/decider_data.h"
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/lane_change_observer.h"
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/lane_line_delay.h"
#include "planning/localview/lane_line_builder/real_hdmap_lane_line/double_rise_decider.h"
#include "planning/localview/local_view.h"
#include "planning/pnc_map/pnc_map.h"
#include "proto/common/types.pb.h"

namespace TL {
namespace planning {
using TL::common::math::Vec2d;
using TL::planning::lanelineprocess::DebounceModule;
using TL::planning::lanelineprocess::Delay;
constexpr double kDfaultLaneWidth = 1.875;
constexpr int kLaneMarkerDelaySize = 5;

class FailureLocationChecker {
 public:
  FailureLocationChecker() = default;
  ~FailureLocationChecker() = default;
  void Init();
  void Process(const std::shared_ptr<LocalView>& local_view,
               const std::shared_ptr<TL::hdmap::PncMap>& pnc_map,
               const std::shared_ptr<TL::hdmap::HDMap>& hdmap,
               functionmanager::FunctionManagerOut* to_fct);
  void Reset();

  enum LocErrState { Loc_Init, Shift, Err };

 private:
  bool DeciderLaneMarkerAndWidth(const TL::common::Pose& pose);
  int LocationErrDecider(const std::shared_ptr<LocalView>& local_view,
                         functionmanager::FunctionManagerOut* to_fct);
  std::pair<double, double> CalculateValidLocErrHasMap(
      const TL::hdmap::LaneInfoConstPtr& nearestlane);
  void DealLocInitState(std::pair<double, double> width_diff,
                        bool is_bad_lanemarkers, bool has_no_width_diff,
                        bool not_in_main_road, bool is_match);
  void DealLocShiftState(std::pair<double, double> width_diff,
                         bool is_bad_lanemarkers, bool has_no_width_diff,
                         bool not_in_main_road, bool is_match);
  TL::perception::LaneMarkers GenerateVirtualMapLanemarkers() const;
  void LanechangeRiseDecider(std::pair<bool, bool> perception,
                             std::pair<bool, bool> map);
  static double CalculateLanemarkerY(
      double distance, const TL::perception::LaneMarker& lane_marker);
  bool MatchingLaneAndMap(const std::shared_ptr<TL::hdmap::PncMap>& pnc_map,
                          const common::Pose& pose,
                          const std::shared_ptr<TL::hdmap::HDMap>& hdmap,
                          functionmanager::FunctionManagerOut* to_fct,
                          bool match_prerequisite);
  static std::tuple<double, double, double> CheckerDistance(
      const std::vector<Vec2d>& map_bound_points,
      const TL::perception::LaneMarker& lane_marker);
  void AddMapPoints(std::vector<Vec2d>* left_map_points,
                    std::vector<Vec2d>* right_map_points,
                    const TL::hdmap::LaneInfoConstPtr& lane,
                    const common::Pose& pose);
  Vec2d PointEarth2Bus(const Vec2d& point, const common::Pose& pose);
  std::vector<Vec2d> InterPolateVec2dPoints(
      const std::vector<Vec2d>& map_points, double start_x, double end_x,
      double delta);
  Vec2d InterPolateVec2dPoint(const Vec2d& point_start, const Vec2d& point_end,
                              double dis);
  void ResetMatchState();
  bool MapLaneChecker(const std::shared_ptr<TL::hdmap::PncMap>& pnc_map,
                      const std::shared_ptr<TL::hdmap::HDMap>& hdmap,
                      functionmanager::FunctionManagerOut* to_fct);
  void DealPerceptionLocErr(const std::pair<double, double>& width_diff,
                            bool is_bad_lanemarkers, bool has_no_width_diff,
                            bool not_in_main_road, bool is_match);
  static bool GetErrStatebl(
      const std::shared_ptr<TL::hdmap::PncMap>& pnc_map,
      const std::shared_ptr<TL::hdmap::HDMap>& hdmap);
  bool DeciderObsBeforeVehicle(const std::shared_ptr<LocalView>& local_view);
  static double CalculateObsY(const TL::perception::LaneMarker& lane_marker,
                              double obs_x, double obs_y);
  TL::planning::lanelineprocess::LaneChangeObserver per_lanechange_observer_;
  TL::planning::lanelineprocess::LaneChangeObserver map_lanechange_observer_;
  TL::perception::LaneMarkers lanemarkers_;
  double left_map_c0_ = kDfaultLaneWidth;
  double right_map_c0_ = -kDfaultLaneWidth;
  int width_diff_count_ = 0;
  int width_same_count_ = 0;
  int err_state_count_ = 0;
  bool is_left_lane_change_ = false;
  bool is_right_lane_change_ = false;
  bool is_lane_change_ = false;
  DoubleRiseDecider double_left_lane_change_rise_decider_;
  DoubleRiseDecider double_right_lane_change_rise_decider_;
  Delay<double> left_lanemarker_a0_delay_{kLaneMarkerDelaySize};
  Delay<double> left_lanemarker_a1_delay_{kLaneMarkerDelaySize};
  Delay<double> right_lanemarker_a0_delay_{kLaneMarkerDelaySize};
  Delay<double> right_lanemarker_a1_delay_{kLaneMarkerDelaySize};

  // err state
  LocErrState per_loc_err_state_ = LocErrState::Loc_Init;
  int loc_err_state_{0};
  double history_pose_time_ = 0.0;
  TL::common::Point2D history_location_point_;
  DebounceModule left_lanemarker_quality_debounce_;
  DebounceModule right_lanemarker_quality_debounce_;
  DebounceModule left_lanemarker_viewrange_debounce_;
  DebounceModule right_lanemarker_viewrange_debounce_;
  DebounceModule is_in_main_road_debounce_;
  DebounceModule is_good_lanemarker_debounce_;
  DebounceModule is_good_init_lanemarker_debounce_;
  DebounceModule is_in_overlaplane_debounce_;
  DebounceModule original_location_err_debounce_{0.0, 2.0, 0.1};
  std::pair<double, double> width_diff_history_{0.0, 0.0};
  double left_lanemarker_c1_delta_{0.0};
  double right_lanemarker_c1_delta_{0.0};
  uint localization_zone_id_ = 0;
  std::shared_ptr<const common::VehicleState> vehicle_state_{nullptr};
  // 0: init_state 1: start_no_match_state 2:no_match_state
  //   int match_state_{0};
  int left_match_state_{0};
  int right_match_state_{0};
  int left_match_state_count_{0};
  int right_match_state_count_{0};
  int left_check_dis_count_{0};
  int right_check_dis_count_{0};
  //   int perception_loc_err_status_{0};
  bool is_location_init_{false};
  bool is_loc_pose_jump_{false};
  bool not_change_lane_{false};
  bool err_not_main_road_{false};
  bool is_change_mode_by_odd_type_{false};
};

}  // namespace planning
}  // namespace TL
