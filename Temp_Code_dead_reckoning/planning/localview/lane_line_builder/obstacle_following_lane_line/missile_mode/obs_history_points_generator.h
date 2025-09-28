/*
 * Copyright (c) TL auto Co., Ltd. 2023-2024. All rights reserved.
 */

#pragma once

#include <array>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "common/interpolation/interpolation_1d.h"
#include "common/math/curve_fitting.h"
#include "common/math/vec2d.h"
#include "common/status/status.h"
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/decider_data.h"
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/lane_line_delay.h"
#include "planning/localview/lane_line_builder/obstacle_following_lane_line/missile_mode/obstacles_state.h"
#include "planning/localview/lane_line_builder/obstacle_following_lane_line/missile_mode/vehicle_state_decider.h"
#include "planning/localview/local_view.h"
#include "planning/proto/navigation_hdmap_config.pb.h"
#include "proto/fsm/function_manager.pb.h"
#include "proto/perception/perception_obstacle.pb.h"

namespace TL {
namespace planning {
namespace missilelane {
using TL::common::Status;
using TL::common::math::Vec2d;
using Matrix = Eigen::MatrixXd;
using TL::perception::LaneMarker;
using TL::perception::LaneMarkers;

// using TL::common::math::FitPolynomial;

class ObsHisPointsGenerator {
 public:
  struct PreLanes {
    bool valid{false};
    double pre_c0{0.0};
    double view_range{0.0};
    double lane_width{0.0};
    int left_track_id{0};
    int right_track_id{0};
    perception::LaneMarker central_lanemarker{};
  };

  ObsHisPointsGenerator() = default;
  ~ObsHisPointsGenerator() = default;
  Status Init(const std::shared_ptr<ObstaclesState>& obstacles_state_ptr);
  bool Process(const std::shared_ptr<LocalView>& local_view);

  const std::vector<Vec2d>& GetObsPoints() { return obs_points_; }

  const TL::perception::LaneMarker& GetObsLanemarker() {
    return obs_lanemarker_;
  }

  void SetPerceptionObsPtr(
      const std::shared_ptr<const perception::PerceptionObstacles>&
          perception_obs_ptr,
      const std::shared_ptr<const functionmanager::FunctionManagerIn>& fct_in,
      functionmanager::FunctionManagerOut* fct_out) {
    perception_obstacles_ = perception_obs_ptr;
    fct_in_ = fct_in;
    history_perception_sub_state_ = fct_out->perception_sub_state();
  }

  double GetObsHeading() const { return average_heading_; }

  double obs_x() const { return obs_x_; }

  double obs_y() const { return obs_y_; }

  double half_lane_width() const { return half_lane_width_; }

 private:
  bool DealObs(const std::shared_ptr<LocalView>& local_view);
  void GenerateObsBackPoints(const TL::perception::PerceptionObstacle& obs);
  void ObsInStateAction(std::deque<double>* init_delta, double* dx,
                        double* dphi, double* dis, bool is_target_in);

  void GenerateObsHeadingAndCurve(
      const TL::perception::PerceptionObstacle& obs);
  bool DoVehicleTrajectoryPredict(double dx, double dphi, bool reset_flag,
                                  bool is_target_in);
  void DataPreprocess(const TL::perception::PerceptionObstacle& obs);
  void TJAdistEndPointTableCreater();
  static bool JudgeTrgtVehIN(const TL::perception::PerceptionObstacle& obs);
  static double SaturationDynamicLimit(double x, double low, double high);
  void EnableDynamicRateLimiter(int in, int ic, double up, double low,
                                double mainLoopTime, double* out);
  bool DoVehHistPntsBuffUpdate(double dx, double dy, double dphi,
                               bool rest_flag, bool is_target_in);

  static void BufferReset(std::deque<Vec2d>* hist_points_buffer, double obs_x,
                          double obs_y, int num_history_buff);
  static void BufferReset(std::deque<Vec2d>* hist_points_buffer, double obs_x,
                          double obs_y, int num_history_buff, double heading);
  void DealBackPoints();
  /**
   * @brief 坐标转换
   *
   * @param points
   * @param dtheta
   * @param dx
   * @param dy
   * @param is_movefirst
   */
  template <typename T>
  static void CoordTranslate2d(T* output_points, const T& input_points,
                               double theta, double dx, double dy,
                               bool is_movefirst);
  /**
   * @brief 最小二乘法计算平均heading
   *
   * @param hist_points_x 
   * @param hist_points_y 
   * @return double 
   */
  double CalculateAverageHeading(const Eigen::MatrixXd* hist_points_x,
                                 const Eigen::MatrixXd* hist_points_y);
  double CalculateAvgMoveCurvature(const Eigen::MatrixXd& hist_points_x,
                                   const Eigen::MatrixXd& hist_points_y);
  static double CalculateStandDeviation(const Eigen::MatrixXd& hist_points_x,
                                        const Eigen::MatrixXd& hist_points_y);
  bool GenerateTrgtVehPreTrajPnts(Eigen::MatrixXd* pre_points_x,
                                  Eigen::MatrixXd* pre_points_y);
  static void LinSpace(Eigen::MatrixXd* LinearVector, double xDistObj,
                       int NumHistBuff);
  std::vector<Vec2d> buffHistPoints_RstVal{20, Vec2d(0, 0)};

  bool TrgtVehMvTrdStsReliable(double obs_lon_spd,
                               const Eigen::MatrixXd& hist_points_x,
                               const Eigen::MatrixXd& hist_points_y);

  void TrgtVehTrajPntsMergeFix(const Eigen::MatrixXd& pre_points_x,
                               const Eigen::MatrixXd& pre_points_y);
  static void CreatInitLaneMarkerPoints(
      const TL::perception::LaneMarker& input_lanemarker,
      std::vector<Vec2d>* out_points);
  std::tuple<double, double, bool, bool> Timer_Scheduler();

  bool DealLaneLine(const std::shared_ptr<LocalView>& local_view);
  bool DealNoObsLaneLine(const std::shared_ptr<LocalView>& local_view);
  void GenerateAllLanemarker(
      const TL::perception::LaneMarker& left_lane_marker,
      const TL::perception::LaneMarker& right_lane_marker);
  void GenerateNoObsLanemarker(const std::shared_ptr<LocalView>& local_view);
  void GenerateOneLanemarker(const TL::perception::LaneMarker& lanemarker,
                             int is_left_lanemarker);
  bool ObsVaildDecider(const MissileObs& missile_obs);
  bool NeedUsingHisPoints(const MissileObs& missile_obs);
  void TranslateHisPoints(TL::perception::LaneMarker* lanemarker,
                          double* totle_length, double* totle_time);
  static double CalculateLanemarkerY(
      double distance, const TL::perception::LaneMarker& lane_marker);
  static double RoadCurvatureCalculate(
      const TL::perception::LaneMarker& lane_marker, double length);
  std::shared_ptr<const functionmanager::FunctionManagerIn> fct_in_{nullptr};
  std::pair<bool, bool> CheckerLanemarker(
      const std::shared_ptr<LocalView>& local_view,
      TL::perception::LaneMarker* left_lanemarker,
      TL::perception::LaneMarker* right_lanemarker);
  static double CalculateObsY(const TL::perception::LaneMarker& lane_marker,
                              double obs_x, double obs_y);
  std::pair<double, bool> CalcObsInWhichLane(
      const LaneMarker& left_lanemarker,
      const LaneMarker& right_lanemarker) const;

  void DealLocalMap(const std::shared_ptr<LocalView>& local_view);
  void GetTargetLanes(const std::shared_ptr<LocalView>& local_view);

  std::shared_ptr<MissileVehicleState> vehicle_state_{nullptr};
  std::shared_ptr<ObstaclesState> obstacles_state_{nullptr};
  std::shared_ptr<const perception::PerceptionObstacles> perception_obstacles_{
      nullptr};
  std::shared_ptr<const perception::LaneMarkers> lanemarkers_;
  std::vector<Vec2d> obs_points_{};
  std::deque<Vec2d> hist_points_buffer_;
  std::deque<Vec2d> hist_points_buffer_update_;
  std::deque<Vec2d> hist_points_buffer_for_heading_;
  //   double history_heading_{0.0};
  int missile_mode_state_{0};
  int lane_line_state_{0};
  int no_obs_lane_line_state_{0};
  double no_obs_total_length_{0.0};
  double no_obs_total_time_{0.0};

  lanelineprocess::Delay<double> Ramp_Resettable_delay_;
  lanelineprocess::Delay<double> c1_heading_delay_;
  // 频繁使用的obs和车辆信息
  double obs_x_{0.0};
  // after filter obs_y_
  double obs_y_{0.0};
  double obs_vx_{0.0};
  double obs_heading_{0.0};
  double ego_vehspd_{0.0};
  double ego_yawtate_{0.0};
  double average_heading_{0.0};
  double average_curve_{0.0};
  double half_lane_width_{1.6};
  int laneline_state_one_count_{0};
  int32_t obs_id_{0};
  int32_t obs_id_last_{0};
  bool is_obj_id_changed_bl_{false};
  bool is_prepoint_generate_{false};
  bool is_target_in_{false};
  //   bool need_using_history_points_{false};
  double using_history_points_time_{0.0};
  double using_history_points_length_{0.0};
  double max_using_history_points_length_{0.0};

  // const navigation_hdmap::NavigationHdmapConfig& config_;
  // Obs_In_LowSpd: obs_v < 1.0, Obs_In_HighSpd: obs_v > 1.5
  enum State { Obs_Out, Obs_In_LowSpd, Obs_In_HighSpd };

  State current_points_state_ = Obs_Out;

  bool is_avgtrnd_buff_full_bl_ = false;
  bool is_stand_deviation_nok_ = false;
  functionmanager::PerceptionSubState history_perception_sub_state_{
      functionmanager::SUB_INITIAL_TYPE};

  TL::perception::LaneMarker obs_lanemarker_{};
  TL::perception::LaneMarker laneline_lanemarker_{};
  TL::perception::LaneMarker no_obs_lanemarker_{};
  common::Interpolation1D TJA_ODPR_dist_EndPoint_;
  common::Interpolation1D::DataType xy1_{};
  std::array<double, 13> k_ODPR_V_VehSpdForVfEndPnt_Xaxis = {
      0,  10, 20, 30,  40,  50, 60,
      70, 80, 90, 100, 110, 120};  // Y axis End dist in vehicle follow mode.

  std::array<double, 13> k_ODPR_d_VfEndPntDist_Yaxis = {
      30, 31, 32, 33, 34, 35, 36,
      37, 38, 39, 40, 41, 42};  // X axis vehicle speed for End distance in vehicle following mode
  lanelineprocess::DebounceModule est_trgmovetrend_not_relible_debounce_;
  lanelineprocess::DebounceModule has_lanemarker_debounce_{0.0, 0.0, 0.1};
  lanelineprocess::LowPassFilter k_standard_deviation_filter_;
  lanelineprocess::LowPassFilter avg_trgveh_move_curvature_filter_;
  lanelineprocess::LowPassFilter avg_trgveh_move_heading_filter_;
  lanelineprocess::LowPassFilter trgveh_curve_ddy_filter_;
  lanelineprocess::FirstOrderLowerPassFilter obs_lat_y_filter_;
  double his_left_c0_{0.0};
  double his_right_c0_{0.0};
  double his_viewrange_{0.0};
  std::vector<PreLanes> pre_lanes_{};
  PreLanes his_pre_lane_{};
};

}  // namespace missilelane
}  // namespace planning
}  // namespace TL
