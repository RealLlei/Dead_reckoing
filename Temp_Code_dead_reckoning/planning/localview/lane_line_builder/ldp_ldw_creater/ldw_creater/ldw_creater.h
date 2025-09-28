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

class LdwCreater {
 public:
  LdwCreater();
  explicit LdwCreater(const PerceptionMapConfig& config);
  ~LdwCreater() = default;

  /**
   * @brief module name
   */
  static std::string Name() { return "LdwCreater"; }

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
  bool LdwDeal(functionmanager::FunctionManagerOut* to_fct,
               LdpLdwData* ldp_ldw_data,
               const std::shared_ptr<LocalView>& local_view);

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
  void LdwMessageInfo(
      const std::shared_ptr<LocalView>& local_view,
      const std::shared_ptr<LanemarkersLaneLine>& lanemarker_debug) const;

 private:
  common::VehicleParam vehicle_param_;
  const PerceptionMapConfig& navi_hdmap_config_;
  std::shared_ptr<const common::VehicleState> vehicle_state_{nullptr};
  std::shared_ptr<const perception::PerceptionObstacles> perception_obstacles_{
      nullptr};
  bool ldw_right_warning_trigger_ = false;
  bool ldw_left_warning_trigger_ = false;
  bool ldw_right_warning_trigger_last_ = false;
  bool ldw_left_warning_trigger_last_ = false;
  bool ldw_left_lane_line_condition_ = false;
  bool ldw_right_lane_line_condition_ = false;
  //   double left_line_markers_c0_ = 0;
  //   double left_line_markers_c1_ = 0;
  //   double left_line_markers_c2_ = 0;
  //   double left_line_markers_c3_ = 0;
  //   double left_line_quality_ = 0;
  //   double right_line_markers_c0_ = 0;
  //   double right_line_markers_c1_ = 0;
  //   double right_line_markers_c2_ = 0;
  //   double right_line_markers_c3_ = 0;
  //   double right_line_quality_ = 0;
  //   double left_line_markers_offset_last_ = 0;
  //   double right_line_markers_offset_last_ = 0;
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
                                                    0.1,  0.2,   0.2};
  std::array<double, 6> inner_zone_table_arr_y3_ = {-0.05, 0,   0.1,
                                                    0.25,  0.4, 0.45};
  TL::planning::lanelineprocess::LowPassFilter ldw_lowpass_filter_;
};

}  // namespace planning
}  // namespace TL
