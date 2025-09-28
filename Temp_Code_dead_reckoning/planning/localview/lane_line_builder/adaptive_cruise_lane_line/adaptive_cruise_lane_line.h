/*
 * Copyright (c) TL auto Co., Ltd. 2021-2022. All rights reserved.
 */

#pragma once

#include <algorithm>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "common/configs/vehicle_config_helper.h"
#include "common/filters/digital_filter.h"
#include "common/filters/digital_filter_coefficients.h"
#include "planning/localview/lane_line_builder/adaptive_cruise_lane_line/cruise_target_select.h"
#include "planning/localview/lane_line_builder/lane_line_base.h"
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_logical_decider.h"
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/decider_data.h"
#include "planning/localview/local_view.h"
#include "planning/localview/localview_comdata_manager.h"

#include "planning/proto/navigation_hdmap_config.pb.h"
#include "proto/common/vehicle_state.pb.h"

namespace TL {
namespace planning {
class AdaptiveCruise : public LaneLineBase {
 public:
  AdaptiveCruise() = default;

  /**
   * @brief module name
   */
  static std::string Name() { return "AdaptiveCruise"; }

  /**
   * @brief module initialization function
   * @return initialization status
   */
  TL::common::Status Init(
      const planning::PerceptionMapConfig& config,
      const std::shared_ptr<LocalViewData>& local_view_data);
  TL::common::Status Init() override;

  /**
   * @brief module start function
   * @return start status
   */
  TL::common::Status Start() override;

  /**
   * @brief module stop function
   */
  void Stop() override;

  /**
   * @brief main logic of the navigation_hdmap module, runs periodically
   * triggered by timer.
   */
  bool Process(const std::shared_ptr<LocalView>& local_view,
               functionmanager::FunctionManagerOut* to_fct) override;

  void KappaPreprocessing(const std::shared_ptr<LocalView>& local_view);
  const std::shared_ptr<navigation_hdmap::MapMsg>& GetMapMsg(
      bool refresh) override {
    UNUSED(refresh);
    return current_map_msg_;
  };

  const std::shared_ptr<routing::RoutingResponse>& GetRoutingResponse()
      override {
    return current_routing_response_;
  };

  // cruise_target_id_ = {
  //     前方目标1,     前方目标2,     左前方目标1,   左前方目标2,
  //     右前方目标1,   右前方目标2,   cut_in,        左侧方目标1,
  //     左侧方目标2,   右侧方目标1,   右侧方目标2,   左侧后方目标1,
  //     左侧后方目标2, 右侧后方目标1, 右侧后方目标2, 后方目标};

 private:
  void KappaBySteerAngle(double angle, double speed, double lf, double lr,
                         double Cf, double Cr, double mass);

  void KappaByYawrate(double yaw_rate, double speed);

  void SubjTrajKappaFusion(double kappa_yaw_rate, double kappa_by_steer_angle,
                           double speed);
  static std::tuple<bool, double, bool, double> AdjacentLaneExtensionSign(
      double left_c0, double left_quality, double right_c0,
      double right_quality);
  static std::tuple<double, double, double, double, double> ExtensionLaneWidth(
      std::tuple<bool, double, bool, double> adjacent_lane_estension_tuple);
  static double SafeDivide(double nom, double denom, double threshold);

  /*线性插值*/
  static double InterplLinear(double x, double x1, double y1, double x2,
                              double y2) {
    double y = 0.0;
    if (x < x1) {
      y = y1;
    } else if (x > x2) {
      y = y2;
    } else {
      y = (y2 - y1) * (x - x1) / (x2 - x1);
    }
    return y;
  }

  double YawrateFilter(double yaw_rate);
  /**
   * @description: 计算稳定性因数
   * @return {K_delta, K} 
   */
  static std::pair<double, double> StabilityFactor(double delta, double a,
                                                   double b, double k1,
                                                   double k2, double m);
  /**
   * @description: 基于二自由度动力学计算omega 
   * @return {omega}
   */
  static double YawRateByWheelAngle(double K_delta, double delta, double v,
                                    double a, double b);
  /**
   * @description: 基于二自由度通过稳定性因数修正自车轨迹半径
   * @return {Radius, kappa}
   */
  static std::pair<double, double> KappaAndRadiusBySteerStabilityFactor(
      double delta, double a, double b, double k1, double k2, double m,
      double v);

  double SteerPct2Wheel(double steer_percentage);
  double WheelAngleLimit(double speed);
  double CalculateWheelAngle(double steer_angle, double speed);
  double CalculateRadius(double wheel_angle);
  bool GeneratePathPoints(
      double radius, double angle, const common::VehicleState& vehicle_state,
      std::tuple<double, double, double, double, double> extension_lane_width,
      std::tuple<common::Path, common::Path, common::Path>* center_path);

  void GenerateInitPoints(
      double radius, double angle, common::Path* init_points,
      common::Path* init_left_points, common::Path* init_right_points,
      double max_arc_length,
      std::tuple<double, double, double, double, double> extension_lane_width);
  /**
   * @description: 生成自车轨迹中心线
   * @return {init_points}
   */
  void GenerateCurrentLaneTrajectoryPoints(double radius, double angle,
                                           common::Path* init_points,
                                           double max_arc_length,
                                           double extension_lane_width);
  /**
   * @description: 生成左侧轨迹中心线
   * @return {*}
   */
  void GenerateLeftLaneTrajectoryPoints(double radius, double angle,
                                        common::Path* init_points,
                                        double max_arc_length,
                                        double extension_lane_width);
  /**
   * @description: 生成右侧轨迹中心线
   * @return {*}
   */
  void GenerateRightLaneTrajectoryPoints(double radius, double angle,
                                         common::Path* init_points,
                                         double max_arc_length,
                                         double extension_lane_width);
  /**
   * @description: 根据自车轨迹虚拟临车道中心线, 由于自车轨迹线采样点较密, 为降低耗时,对临车道采样点稀疏
   * discrete_step 离散补偿不能设置太大,不然在大转弯时会与自车道轨迹边界交叉
   * @return {adjacent_init_points}
   */
  static void GenerateAdjacentLaneTrajectoryPoints(
      const common::Path& init_points, common::Path* adjacent_init_points,
      std::tuple<double, double, double, double, double> extension_lane_width,
      int flag, double radius, double angle, int discrete_step);
  /**
   * @brief generate one lane by points
   * 
   * @param path points
   * @param lane lane in map
   * @return true 
   * @return false 
   */
  bool GenerateOneLane(const common::Path& path, hdmap::Lane* lane,
                       double extension_one_lane_width, int lane_id,
                       int discrete_step);
  static bool SetRouting(TL::routing::RoutingResponse* inrouting,
                         TL::hdmap::Map* hd_map);
  static void TrajectoryPointTransforENU(
      common::Path* path_point, const common::VehicleState& vehicle_state);
  static void ResetCruiseTargetId(std::vector<int32_t>* cruise_target_id);
  common::VehicleParam vehicle_param_;
  TL::common::DigitalFilter steer_angle_filter_;
  planning::PerceptionMapConfig navi_hdmap_config_;
  std::shared_ptr<navigation_hdmap::MapMsg> current_map_msg_ = nullptr;
  std::shared_ptr<routing::RoutingResponse> current_routing_response_ = nullptr;
  CruiseTargetSelect cruise_target_select_;
  lanelineprocess::FirstOrderLowerPassFilter kappa_by_yaw_rate_filter_;
  lanelineprocess::FirstOrderLowerPassFilter kappa_by_steer_angle_filter_;
  lanelineprocess::FirstOrderLowerPassFilter radius_after_fusion_filter_;
  double kappa_by_yaw_rate_ = 0.0;
  double radius_by_yaw_rate_ = 0.0;
  double kappa_by_steer_angle_ = 0.0;
  double radius_by_steer_angle_ = 0.0;
  double kappa_fusion_ = 0.0;
  double radius_Fusion_ = 0.0;
  double yaw_rate_filter_fast_last_ = 0.0;
  double filte_yaw_rate_last_ = 0.0;
  double wheel_angle_{0.0};
  std::vector<int32_t> cruise_target_id_{};
  bool acc_function_active_ = false;
  bool acc_function_active_last_ = false;
};

}  // namespace planning
}  // namespace TL
