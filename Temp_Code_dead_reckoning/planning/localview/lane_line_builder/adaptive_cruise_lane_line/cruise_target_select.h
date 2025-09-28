/*
 * Copyright (c) TL auto Co., Ltd. 2021-2022. All rights reserved.
 */
#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>
#include "planning/localview/hdmap_avp_state/hdmap_avp_state.h"
#include "planning/localview/lane_line_builder/adaptive_cruise_lane_line/target_overlap.h"
#include "planning/localview/local_view.h"
#include "planning/proto/navigation_hdmap_config.pb.h"
#include "proto/common/vehicle_state.pb.h"

namespace TL {
namespace planning {
class CruiseTargetSelect {
 public:
  struct Point3D {
    double x;
    double y;
    double z;
  };

  struct OneTargetCalculateinfos {
    int32_t id;
    int fusn_src;
    bool maturity_ok;
    bool jump_ok;
    bool coasting_ok;
    bool use_yaw_plus_lane_change;
    bool target_valid;
    double offset_deriv;
    double offset;
    double center_offset;
    SelectFunnel select_funnel;
    bool use_prediction;
    double predicted_offset;
    bool is_cut_in;
    bool with_in_wide_funnel;
    bool with_in_funnel;
    Point3D position;
    Point3D velocity;
    Point3D acceleration;
    bool has_same_id_last_infos;
    OverlapInfos overlap_infos;
  };

  struct SelectResultInfos {
    int32_t first_front_id;         // 前方目标1
    int32_t second_front_id;        // 前方目标2
    int32_t first_left_front_id;    // 左前方目标1
    int32_t first_right_front_id;   // 右前方目标1
    int32_t second_left_front_id;   // 左前方目标2
    int32_t second_right_front_id;  // 右前方目标2
    int32_t cut_in_id;              // 切入目标
    int32_t first_left_side_id;     // 左侧方目标1
    int32_t second_left_side_id;    // 左侧方目标2
    int32_t first_right_side_id;    // 右侧方目标1
    int32_t second_right_side_id;   // 右侧方目标2
    int32_t first_left_rear_id;     // 左侧后方目标1
    int32_t second_left_rear_id;    // 左侧后方目标2
    int32_t first_right_rear_id;    // 右侧后方目标1
    int32_t second_right_rear_id;   // 右侧后方目标2
    int32_t first_rear_id;          // 后方目标
  };

  struct TargetValidatorInfos {
    bool target_quality_valid;
    bool target_moving_valid;
    bool on_coming;
    bool target_stationary_valid;
    bool target_motor_cycle_valid;
    bool target_car_valid;
    bool target_valid;
  };

  CruiseTargetSelect() = default;
  explicit CruiseTargetSelect(const PerceptionMapConfig& config);
  ~CruiseTargetSelect() = default;

  static std::string Name() { return "CruiseTargetSelect"; }

  Status Init();
  static Status Start();
  void Stop();

  bool Process(const std::shared_ptr<LocalView>& local_view,
               double subject_kappa, double steer_angle,
               std::vector<int32_t>* cruise_target_id);

 private:
  static bool UpdateMaturity(int fusn_src, double target_position_longitudinal,
                             bool snsr_data_valid);
  static bool UpdateJumpMaturity(double target_position_longitudinal,
                                 double last_target_position_longitudinal,
                                 double target_position_lateral,
                                 double last_target_position_lateral,
                                 bool has_same_id_last_infos);
  static bool UpdateCoastedTimer(int snsr_data_sts);
  static bool UpdateUseYawPlusInLChange(bool left_dicator, bool right_dicator,
                                        bool lane_change);
  static TargetValidatorInfos TargetValidator(
      int fusn_src, int snsr_data_sts, double target_position_longitudinal,
      int target_type, double target_speed, bool maturity_ok, bool jump_ok,
      bool coasting_ok);
  static bool QualityValid(int fusn_src, int snsr_data_sts, bool maturity_ok,
                           bool jump_ok, bool coasting_ok);
  static std::pair<bool, bool> MovingValid(int target_type,
                                           double target_speed);
  static bool StationaryValid();
  static bool ValidMotorcycle();
  static bool CarValid();
  static std::tuple<double, double, double, double> CalculateTargetOffset(
      double target_position_longitudinal, double target_position_lateral,
      double target_orientation, double target_speed_lateral,
      double target_width, double target_length, double c0, bool target_valid,
      double steer_angle, double subject_speed_longitudinal);
  static double CalculateC1Factor();
  static double CalculateYawPlusOffset(double target_orientation,
                                       double target_position_longitudinal,
                                       double target_position_lateral,
                                       double c0, double C1, double c1_factor);
  SelectFunnel CalculateFunnel(double lane_width,
                               double target_position_longitudinal,
                               double subject_speed_longitudinal,
                               bool left_indicator, bool right_indicator,
                               double lane_position, bool on_coming);
  bool CheckInWidePath(const SelectFunnel& select_funnel,
                       const OverlapInfos& overlap_infos,
                       const perception::PerceptionObstacle& obstacle,
                       double offset, int32_t id, bool has_same_id_last_infos);
  bool CheckInPath(const SelectFunnel& select_funnel,
                   const OverlapInfos& overlap_infos,
                   const perception::PerceptionObstacle& obstacle,
                   double offset, int32_t id, bool has_same_id_last_infos);

  static double SafeDivide(double nom, double denom, double threshold);
  template <class T, std::size_t N>
  std::pair<int, double> BinarySearch(double u, std::array<T, N> bp);
  template <class T, std::size_t N>
  double InterpolationBinarySearch(int index, double fraction,
                                   std::array<T, N> ydat);

  static double CartY(double x, double a0_ref, double a1_ref, double a2_ref,
                      double a3_ref);
  static double CartDotProduct(std::vector<double> A, std::vector<double> B);
  static int CartSign(double val);
  static std::vector<double> CartProjectPoint(double x_in, double y_in,
                                              double a0_ref, double a1_ref,
                                              double a2_ref, double a3_ref,
                                              double error);
  static double MinLatDistance(double x_in, double y_in, double x_r,
                               double y_r);
  static double MinDistanceL(double x_in, double y_in, double a0_ref,
                             double a1_ref, double a2_ref, double a3_ref,
                             double error);
  static bool FiltByObstacleInfos(
      const TL::perception::PerceptionObstacle& obstacle);
  static void ResetSelectResultInfos(SelectResultInfos* select_result_infos);
  static void OutputSelectResultInfos(
      const SelectResultInfos& select_result_infos,
      std::vector<int32_t>* cruise_target_id);
  static void InitTargetOverlapInfos(TargetOverlapInfos* target_overlap_infos);
  static void SetTargetInfos(
      const std::tuple<double, double, double, double>& target_offset_tuple,
      const perception::PerceptionObstacle& obstacle,
      TargetInfos* target_infos);
  static std::tuple<double, double, double> Offset(const std::vector<double>& x,
                                                   const std::vector<double>& y,
                                                   double orientation,
                                                   double c0, double c1,
                                                   double c1_factor);
  double SafeStopDistance(double speed, double accel, double kappa,
                          double safe_min_accel, double safe_max_accel,
                          double safe_min_jerk, double safe_max_jerk);
  void ModifyStopSelectResultInfos(
      const SelectResultInfos& select_result_infos,
      std::unordered_map<int32_t, OneTargetCalculateinfos>&
          target_calculate_infos);
  static double ConstDecelerateToStop(double current_v, double current_a,
                                      double const_dec_a, double const_dec_jerk,
                                      double const_inc_jerk);

 private:
  class TargetOverlap target_overlap_;
  PerceptionMapConfig navigation_hdmap_config_;
  common::VehicleParam vehicle_param_;
  std::shared_ptr<const perception::PerceptionObstacles> perception_obstacles_{
      nullptr};
  std::unordered_map<int32_t, OneTargetCalculateinfos>
      last_target_calculate_infos_{};
  std::shared_ptr<const common::VehicleState> vehicle_state_{nullptr};
  SelectResultInfos stop_result_infos_{};
  SelectResultInfos last_stop_result_infos_{};
  bool acc_state_is_stand_ = false;
  int subject_kappa_counter_ = 0;
  double safe_stop_distance_ = 150.0;
};

}  // namespace planning
}  // namespace TL
