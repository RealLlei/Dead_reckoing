/*
 * Copyright (c) TL auto Co., Ltd. 2021-2022. All rights reserved.
 */

#pragma once
#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>
#include "common/interpolation/interpolation_1d.h"
#include "common/status/status.h"
#include "planning/localview/lane_line_builder/ldp_ldw_creater/ldp_ldw_base.h"
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/decider_data.h"
#include "planning/localview/local_view.h"
#include "planning/proto/navigation_hdmap_config.pb.h"
#include "proto/common/vehicle_state.pb.h"

namespace TL {
namespace planning {
using TL::common::Status;

class LdpCreater {
 public:
  LdpCreater();
  explicit LdpCreater(const PerceptionMapConfig& config);
  ~LdpCreater() = default;

  /**
   * @brief module name
   */
  static std::string Name() { return "LdpCreater"; }

  /**
   * @brief module initialization function
   * @return initialization status
   */
  Status Init();

  /**
   * @brief module start function
   * @return start status
   */
  static Status Start();

  /**
   * @brief module stop function
   */
  void Stop();
  void SetDealMg(const std::shared_ptr<LocalView>& local_view);
  /**
   * @brief main logic of the navigation_hdmap module, runs periodically
   * triggered by timer.
   */

  bool LdpDeal(functionmanager::FunctionManagerOut* to_fct,
               LdpLdwData* ldp_ldw_data,
               const std::shared_ptr<LocalView>& local_view,
               const std::shared_ptr<LanemarkersLaneLine>& lanemarker_debug);

  bool LeftWarningLogic(double a1, double vehspd, double tire_distance_2_line,
                        bool good_path, double warning_inzone,
                        double warning_outzone) const;
  bool RightWarningLogic(double a1, double vehspd, double tire_distance_2_line,
                         bool good_path, double warning_inzone,
                         double warning_outzone) const;
  void LineUpdate(const ::TL::perception::LaneMarkers& lanemarkers);
  double LeftLineMarkerEUation(double x) const;
  double RightLineMarkerEUation(double x) const;
  std::tuple<double, double, double, double> WarningZoneCalculator(
      double ego_lane_width, double speed, bool left_trigger_last,
      bool right_trigger_last, LdpLdwData* ldp_ldw_data);
  static bool IsGoodPath(double line_quality, double quality_vaule);
  static double SaturationDynamicLimit(double x, double low, double high);
  double WarningZoneLimit(double ldwzone, double slow_low, double slow_high,
                          double sstandard_low, double sstandard_high,
                          double shigh_low, double shigh_high);
  double InnerZoneCalculatorVyShift(double angletheta, double speed,
                                    int symbol);
  double InnerZoneCalculatorAyShift(double aheadcurvature, double speed,
                                    int symbol);
  double InnerZoneCalculatorPositionn(double ego_lane_width);
  std::pair<double, double> GetZoneHysteresis(bool trigger_last);
  std::pair<double, double> GetRightZoneHysteresis();
  static double GetEgoLaneWidth(bool left_goodpath, bool right_goodpath,
                                LdpLdwData* ldp_ldw_data);
  void InnerZoneTableCreater();
  double LookAheadCurvature(double speed, double a3, double a2);
  double GetLeftTireDistance2Line();
  double GetRightTireDistance2Line();
  static bool ODDindxCondDeal(uint16_t indx_cond, uint16_t indx_select_cond);
  int LdpRoadCurveJudgement(double left_a2, double left_a3, double right_a2,
                            double right_a3, int ldp_boundary_case,
                            double speed);
  bool LdpPreventionDone(LdpLdwData* ldp_ldw_data, int ldp_boundary_case,
                         bool trigger_condition, bool ldp_active, double speed,
                         bool is_hand_off);
  bool LdpPositionPrevDone(int boundary_case, double left_a0,
                           double right_a0) const;
  bool LdpHeaderPrevDone(int boundary_case, double left_a1, double right_a1,
                         double speed, bool trigger_condition);
  static int LdpLineBoundaryCase(bool left_quality, bool right_quality);
  void LdpDoneState(int* last_state, bool ldp_active, double* counter,
                    bool ldp_done) const;
  static bool LdpDynamicRelay(double x, double relay_on, double relay_off,
                              bool* result_last);
  void LdpMessageInfo(
      const std::shared_ptr<LocalView>& local_view,
      std::tuple<double, double, double, double> warning_zone,
      LdpLdwData* ldp_ldw_data,
      const std::shared_ptr<LanemarkersLaneLine>& lanemarker_debug) const;

 private:
  common::VehicleParam vehicle_param_;
  const PerceptionMapConfig& navi_hdmap_config_;
  std::shared_ptr<const common::VehicleState> vehicle_state_{nullptr};
  std::shared_ptr<const perception::PerceptionObstacles> perception_obstacles_{
      nullptr};
  bool ldp_right_warning_trigger_ = false;
  bool ldp_left_warning_trigger_ = false;
  bool ldp_right_warning_trigger_last_ = false;
  bool ldp_left_warning_trigger_last_ = false;
  bool ldp_left_lane_line_condition_ = false;
  bool ldp_right_lane_line_condition_ = false;
  bool ldp_left_warning_done_trigger_ = false;
  bool ldp_right_warning_done_trigger_ = false;
  int ldp_isin_pos_zone_last_ = 0;
  double pos_counter_ = 0;
  int ldp_isin_pos_header_zone_last_ = 0;
  double pos_header_counter_ = 0;
  int ldp_isin_header_zone_last_ = 0;
  double header_counter_ = 0;
  bool ldp_latacc_to_left_last_ = false;
  bool ldp_latacc_to_right_last_ = false;
  std::array<double, 8> vehicle_speed_x = {0, 20, 40, 60, 80, 100, 120, 140};
  std::array<double, 8> ldp_curve_ref_point_y_ = {20, 20, 25, 30,
                                                 35, 40, 40, 40};
  std::array<double, 8> ldp_latacc_threshold_y_ = {0.4, 0.4, 0.4,  0.4,
                                                  0.5, 0.6, 0.65, 0.65};
  std::array<double, 8> ldp_heading_threshold_y_ = {0.01,  0.01,  0.01,  0.01,
                                                   0.009, 0.007, 0.006, 0.005};
  common::Interpolation1D ldp_heading_threshold_;
  common::Interpolation1D::DataType ldp_heading_threshold_xy_{};
  common::Interpolation1D ldp_curve_ref_point_dist_;  // 车速插值表
  common::Interpolation1D::DataType ldp_curve_ref_point_dist_xy_{};
  common::Interpolation1D ldp_latacc_threshold_;
  common::Interpolation1D::DataType ldp_latacc_threshold_xy_{};
  Sensitivity ldwsensitivity_ = StvlStandard;
  common::Interpolation1D in_zone_position_table_low_;
  common::Interpolation1D in_zone_position_table_standard_;
  common::Interpolation1D in_zone_position_table_high_;
  common::Interpolation1D::DataType xy1_{};  // {2.5, 2.75, 3, 3.25, 3.5, 3.75},
      // {-0.1, -0.1, -0.05, 0, 0.05, 0.05}
  common::Interpolation1D::DataType xy2_{};  // [-0.1 -0.05 0 0.1 0.2 0.2]
  common::Interpolation1D::DataType xy3_{};  // [-0.05 0 0.1 0.25 0.4 0.45]
  std::array<double, 6> inner_zone_table_arr_x_ = {2.5,  2.75, 3,
                                                   3.25, 3.5,  3.75};
  std::array<double, 6> inner_zone_table_arr_y1_ = {-0.1, -0.1, -0.05,
                                                    0,    0.05, 0.05};
  std::array<double, 6> inner_zone_table_arr_y2_ = {-0.1, -0.05, 0,
                                                    0.25, 0.3,   0.35};
  std::array<double, 6> inner_zone_table_arr_y3_ = {-0.05, 0,   0.1,
                                                    0.25,  0.4, 0.45};
  TL::planning::lanelineprocess::LowPassFilter ldp_lowpass_filter_;
  TL::planning::lanelineprocess::DebounceModule
      ldp_road_classification_debounce_;
  // TL::planning::lanelineprocess::DebounceModule LdwDeboucemodule;
  // double LdwODDDebounceFallTime_ = 1;
  // double LdwODDDebounceRiseTime_ = 0.1;
};

}  // namespace planning
}  // namespace TL
