/******************************************************************************
 * Copyright 2023 The TL Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <cstdint>
#include <limits>
#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <tuple>
#include <unordered_set>
#include <utility>
#include <vector>

#include "gflags/gflags_declare.h"
#include "nlohmann/json.hpp"
#include "planning/common/path/discretized_path.h"
#include "planning/common/real_jerk/real_jerk.h"
#include "planning/hmi/lon_hmi/spd_adapt/spd_adapt.h"
#include "planning/localview/local_view.h"
#include "proto/fsm/avp_fct.pb.h"
#include "proto/fsm/metric.pb.h"

namespace TL {
namespace planning {

using Json = nlohmann::json;

enum class ScenarioEnum {
  OTHER = 0,
  TUNNEL = 1,
  RAMP = 2,
  MAINROAD = 3,
  CAR_AVOID = 4,
  LARGE_CURV = 5,
  MERGE_SCENARIO = 6,
  DRIVING_DRAGON = 7,
};

// NOLINTBEGIN
struct NnpMetricS {
  uint8_t driver_mode = 2;
  double start_time = -1.0;
  double end_time = -1.0;
  double active_mile = 0.0;
  double hdmap_mile = 0.0;
  std::vector<double> lat_jerk = {};
  std::vector<double> lon_jerk = {};
  u_int32_t lcc_downgrade = 0;
  u_int32_t location_downgrade = 0;
  u_int32_t internal_downgrade = 0;
  u_int32_t acc_downgrade = 0;
  u_int32_t lat_override = 0;
  u_int32_t lon_override = 0;
  u_int32_t key_take_over = 0;
  u_int32_t take_over = 0;
  u_int32_t fault_take_over = 0;
  double prediction_timeout = 0.0;
  double total_timeout = 0.0;
  double run_once_time = 0.0;
  double total_regular_time = 0.0;
  double planning_cpu_err = 0.0;
  double all_cpu_err = 0.0;
  u_int32_t in_mainroad_success = 0;
  u_int32_t in_mainroad_fail = 0;
  u_int32_t in_ramp_success = 0;
  u_int32_t in_ramp_fail = 0;
  u_int32_t navi_lane_change_success = 0;
  u_int32_t navi_lane_change_fail = 0;
  u_int32_t effi_lane_change_success = 0;
  u_int32_t effi_lane_change_fail = 0;
  u_int32_t switch_lane_change_success = 0;
  u_int32_t switch_lane_change_fail = 0;
  u_int32_t audio_lane_change_success = 0;
  u_int32_t audio_lane_change_fail = 0;
  u_int32_t emerger_braking = 0;
  std::pair<u_int32_t, double> fault_obj = {0, 0.0};
  std::pair<std::vector<ScenarioEnum>, double> scenario_obj = {{}, 0.0};
  std::pair<double, double> coordinate = {0.0, 0.0};
  double heading_angle = 0.0;
  u_int32_t static_obs_avoid = 0;
  u_int32_t dynamic_obs_avoid = 0;
  double height = 0.0;
};

struct AvpRunningStatus {
  bool has_collision_risk = false;
  bool is_running = false;
  bool wait_obs = false;
  bool onceflag = false;

  void Reset() {
    has_collision_risk = false;
    is_running = false;
    wait_obs = false;
    onceflag = false;
  }
};

struct CPUInfo {
  double planning_cpu = 0.0;
  double planning_mem = 0.0;
  double all_cpu_used = 0.0;
  double mem_free = 0.0;
  double total_mem = 0.0;
  u_int32_t cnt = 0;

  void Reset() {
    planning_cpu = 0.0;
    planning_mem = 0.0;
    all_cpu_used = 0.0;
    mem_free = 0.0;
    total_mem = 0.0;
    cnt = 0;
  }
};

struct AvpMetric {
  double start_time = -1.0;
  double end_time = -1.0;
  double parking_start_time = -1.0;
  double parking_end_time = -1.0;
  double crusing_start_distance = -1.0;
  double crusing_end_distance = -1.0;
  u_int32_t sys_mode = 0;
  u_int32_t parking_type = 0;
  u_int32_t finish_status = 0;
  double parking_time = 0;
  u_int32_t parking_shift_count = 0;
  double min_collison_point_x = std::numeric_limits<double>::infinity();
  double min_collison_point_y = std::numeric_limits<double>::infinity();
  double final_v = 0.0;
  double init_coarse_path_time = 0.0;
  double init_smooth_path_time = 0.0;
  CPUInfo average_cpu_info;
  CPUInfo top_cpu_info;
  double crusing_start_time = -1.0;
  double crusing_end_time = -1.0;
  u_int32_t emergency_brake_counter = 0;
  double cruise_mileage = 0.0;
  double cruise_time = 0.0;
  std::vector<double> cruise_abnormal_latjerk;
  std::vector<double> cruise_abnormal_lonjerk;
  u_int32_t cruise_nudge_pedestrian_counter = 0;
  u_int32_t cruise_avoidance_pedestrian_counter = 0;
  u_int32_t cruise_nudge_vehicle_counter = 0;
  u_int32_t cruise_avoidance_vehicle_counter = 0;
  u_int32_t cruise_nudge_cone_or_barricade_counter = 0;
  u_int32_t cruise_pass_speed_bump_counter = 0;
  bool cruise2parking_smooth_switch = false;
  bool cruise2parking_fixed_parklot = false;
  std::vector<std::pair<int, int>> cruise_planning_dec_over_threshold_frame_set;
  double crusing_max_latictrl_tors_pure_yawerr = 0.0;
  double crusing_max_ctrldec_lat_sys_poserr = 0.0;
  double crusing_max_ctrldec_lon_sys_poserr = 0.0;
  double crusing_max_lonctrl_vel_vel_err = 0.0;
  double parking_max_latictrl_tors_pure_yawerr = 0.0;
  double parking_max_ctrldec_lat_sys_poserr = 0.0;
  double parking_max_ctrldec_lon_sys_poserr = 0.0;
  double parking_max_lonctrl_vel_vel_err = 0.0;
  u_int32_t force_enable_dest_lat_constrian_counter = 0;

  // not send to cloud
  AvpRunningStatus avp_running_status;

  void ClearData() {
    start_time = -1.0;
    end_time = -1.0;
    parking_start_time = -1.0;
    parking_end_time = -1.0;
    crusing_start_time = -1.0;
    crusing_end_time = -1.0;
    cruise_time = -1.0;
    sys_mode = 0;
    parking_type = 0;
    finish_status = 0;
    parking_time = 0;
    parking_shift_count = 0;
    min_collison_point_x = std::numeric_limits<double>::infinity();
    min_collison_point_y = std::numeric_limits<double>::infinity();
    final_v = 0.0;
    init_coarse_path_time = 0.0;
    init_smooth_path_time = 0.0;
    crusing_start_distance = -1.0;
    crusing_end_distance = -1.0;
    emergency_brake_counter = 0;
    cruise_mileage = 0.0;
    cruise_abnormal_latjerk.clear();
    cruise_abnormal_lonjerk.clear();
    cruise_nudge_pedestrian_counter = 0;
    cruise_avoidance_pedestrian_counter = 0;
    cruise_nudge_vehicle_counter = 0;
    cruise_avoidance_vehicle_counter = 0;
    cruise_nudge_cone_or_barricade_counter = 0;
    cruise_pass_speed_bump_counter = 0;
    cruise2parking_smooth_switch = false;
    cruise2parking_fixed_parklot = false;
    cruise_planning_dec_over_threshold_frame_set.clear();
    crusing_max_latictrl_tors_pure_yawerr = 0.0;
    crusing_max_ctrldec_lat_sys_poserr = 0.0;
    crusing_max_ctrldec_lon_sys_poserr = 0.0;
    crusing_max_lonctrl_vel_vel_err = 0.0;
    parking_max_latictrl_tors_pure_yawerr = 0.0;
    parking_max_ctrldec_lat_sys_poserr = 0.0;
    parking_max_ctrldec_lon_sys_poserr = 0.0;
    parking_max_lonctrl_vel_vel_err = 0.0;
    force_enable_dest_lat_constrian_counter = 0;
    avp_running_status.Reset();
    average_cpu_info.Reset();
    top_cpu_info.Reset();
  }
};

struct PilotMetric {
  double start_time = -1.0;
  double end_time = -1.0;
  double active_mile = 0.0;
  double total_mile = 0.0;
  std::vector<double> lat_jerk = {};
  std::vector<double> lon_jerk = {};
  u_int32_t downgrade = 0;
  u_int32_t lat_override = 0;
  u_int32_t lon_override = 0;
  u_int32_t take_over = 0;
  double prediction_timeout = 0.0;
  double total_timeout = 0.0;
  double run_once_time = 0.0;
  double total_regular_time = 0.0;
  double planning_cpu_err = 0.0;
  double all_cpu_err = 0.0;
  u_int32_t switch_lane_change_success = 0;
  u_int32_t switch_lane_change_fail = 0;
  u_int32_t audio_lane_change_success = 0;
  u_int32_t audio_lane_change_fail = 0;
  u_int32_t emerger_braking = 0;
  std::pair<double, double> coordinate = {0.0, 0.0};
  double heading_angle = 0.0;
  u_int32_t static_obs_avoid = 0;
  u_int32_t dynamic_obs_avoid = 0;
  double height = 0.0;
};

// NOLINTEND

class MetricCollect {
 public:
  MetricCollect();
  ~MetricCollect() = default;
  /**
   * @brief update input data
   * @param local_view
   */
  void UpdateMetricData(const std::shared_ptr<LocalView>& local_view);

  /**
   * @brief is adc has collision risk
   *
   * @param local_view
   * @param collision_point_x obs collision point x
   * @param collision_point_y obs collision point y
   * @return true
   * @return false
   */
  static bool IsAdcHasCollisionRisk(
      const std::shared_ptr<LocalView>& local_view, double* collision_point_x,
      double* collision_point_y);

 private:
  /**
  * @brief update stop and nudge metric data
  * @param local_view
  */
  void UpdateStopNudgeMetricData(const std::shared_ptr<LocalView>& local_view);

  /**
  * @brief get the vehicle alias.
  * @param local_view
  */
  void GetCarAlias(const std::shared_ptr<LocalView>& local_view);

  /**
  * @brief get the software of sw package TL version.
  * @param local_view
  */
  void GetSoftwareVersion(const std::shared_ptr<LocalView>& local_view);

  /**
  * @brief get the stop obstacle sl and the nudge obstacle xy
  * @param local_view
  */
  static void GetStopNudgeObstacle(
      const std::shared_ptr<LocalView>& local_view,
      std::vector<std::tuple<std::string, int32_t, common::SLPoint>>*
          stop_obstacle_sl,
      std::vector<std::tuple<std::string, int32_t, common::math::Vec2d>>*
          nudge_obstacle_xy);
  /**
   * @brief calculate the stop metric
   * @param stop_obstacle_sl
  */
  void CalculateStopMetric(
      const std::vector<std::tuple<std::string, int32_t, common::SLPoint>>&
          stop_obstacle_sl);
  /**
   * @brief calculate the nudge metric
   * @param nudge_obstacle_sl
  */
  void CalculateNudgeMetric(
      const std::vector<std::tuple<std::string, int32_t, common::math::Vec2d>>&
          nudge_obstacle_xy);
  /**
   * @brief update nnp and pilot metric input data
   * @param local_view
   */
  void UpdateNnpPilotMetricData(const std::shared_ptr<LocalView>& local_view);
  /**
   * @brief update avp metric input data
   * @param local_view
   */
  void UpdateAvpMetricData(const std::shared_ptr<LocalView>& local_view);

  /**
   * @brief 更新接管指标
   * @param local_view
   */
  void UpdateTakeoverMetric(const std::shared_ptr<LocalView>& local_view,
                            metric::NnpMetric* mutable_nnp_metric);

  /**
   * @brief update avp metric input data in parking stage
   * @param local_view
   */
  void UpdateAvpParkingMetricData(const std::shared_ptr<LocalView>& local_view);
  /**
   * @brief update avp metric input data in cruing stage
   * @param local_view
   */
  void UpdateAvpCrusingMetricData(const std::shared_ptr<LocalView>& local_view);
  /**
   * @brief update mileage metric
   * @param odo_meter
   * @param fct_in
   * @param fct_out
   * @param mutable_nnp_metric
   */
  void UpdateMileageMetric(double odo_meter,
                           const functionmanager::FunctionManagerIn& fct_in,
                           const functionmanager::FunctionManagerOut& fct_out,
                           metric::NnpMetric* mutable_nnp_metric);
  /**
   * @brief update fault and scenario metric
   * @param location
   * @param fct_in
   * @param fct_out
   * @param adas
   */
#ifdef ISMDC
  void UpdateFaultMetric(const localization::Localization& location,
                         const functionmanager::FunctionManagerIn& fct_in,
                         const functionmanager::FunctionManagerOut& fct_out,
                         const common::mcu_to_soc_DebugData& adas);
#else
  void UpdateFaultMetric(const localization::Localization& location,
                         const functionmanager::FunctionManagerIn& fct_in,
                         const functionmanager::FunctionManagerOut& fct_out,
                         const control::McuToSocPnc& adas, double hand_torque);
#endif
  /**
   * @brief update scenario metric
   * @param control
   * @param fct_out
   */

#ifdef ISMDC
  void UpdateScenarioMetric(const control::MbdDebugFromMCU& control,
                            const functionmanager::FunctionManagerOut& fct_out);
#else
  void UpdateScenarioMetric(const control::McuToSocPnc& control,
                            const functionmanager::FunctionManagerOut& fct_out);
#endif
  /**
   * @brief update enter ramp and mainroad metric
   * @param fct_in
   * @param fct_out
   */
  void UpdateInOutRampMetric(const functionmanager::FunctionManagerIn& fct_in,
                             const functionmanager::FunctionManagerOut& fct_out,
                             metric::NnpMetric* mutable_nnp_metric);
  /**
   * @brief update nnp function metric
   * @param fct_in
   * @param adc_trajectory
   */
  void UpdateNnpFunctionMetric(const functionmanager::FunctionManagerIn& fct_in,
                               const ADCTrajectory& adc_trajectory,
                               metric::NnpMetric* mutable_nnp_metric);
  /**
   * @brief update pilot function metric
   * @param fct_in
   * @param fct_out
   */
  void UpdatePilotFunctionMetric(
      const functionmanager::FunctionManagerIn& fct_in,
      const functionmanager::FunctionManagerOut& fct_out);
  /**
   * @brief update lane change metric
   * @param fct_out
   */
  void UpdateLaneChangeMetric(
      const functionmanager::FunctionManagerOut& fct_out,
      metric::NnpMetric* mutable_nnp_metric);
  /**
   * @brief update task metric
   * @param adc_trajectory
   */
  void UpdateTaskMetric(const ADCTrajectory& adc_trajectory);
  /**
   * @brief update lat and lon jerk metric
   * @param fct_out
   */
  void UpdateJerkMetric(const functionmanager::FunctionManagerOut& fct_out);

  /**
   * @brief get avp parking type
   * @param parking_lot_info
   * @param sys_command
   * @return uint32_t
   */
  uint32_t GetAVPParkingType(
      const perception::ParkingLotOutArray& parking_lot_info,
      const functionmanager::AvpFctIn::SysCmdType& sys_command) const;

  /**
   * @brief update avp finish status based on fct
   *
   * @param fct_in
   */
  void UpdateFinishStatusBaseFct(const functionmanager::AvpFctIn& fct_in);

  /**
   * @brief update open space debug info
   *
   * @param local_view
   */
  void UpdateOpenSpaceDebugInfo(const std::shared_ptr<LocalView>& local_view);
  /**
   * @brief check save status
   */
  void CheckSaveStatus();
  /**
   * @brief clear history metric
   */
  void ClearMetric();
  /**
   * @brief init metric
   */
  void InitMetric(const std::shared_ptr<LocalView>& local_view);
  /**
   * @brief insert nnp metric
   */
  void InsertNnpMetric(double lat, double lon, double heading, double height);
  /**
   * @brief inssert pilot metric
   */
  void InsertPilotMetric(double lat, double lon, double heading, double height);
  /**
   * @brief metric data to json
   */
  void MetricDataToJson();
  /**
   * @brief save json data to file
   * @param metric_json
   */
  void SaveJsonToFile(const Json::array_t& metric_json);
  /**
   * @brief  init avpmetric class member
   */
  void AvpInit();

  bool is_ok_to_save_ = false;
  std::vector<std::shared_ptr<NnpMetricS>> nnp_metric_ = {};
  std::vector<std::shared_ptr<PilotMetric>> pilot_metric_ = {};
  std::unique_ptr<AvpMetric> avp_metric_ = nullptr;
  std::unique_ptr<RealJerk> avp_real_jerk_ = nullptr;
  std::list<double> planning_cpus_{};
  std::list<double> all_cpu_used_{};
  bool is_nnp_active_ = false;
  bool is_pilot_active_ = false;
  bool is_insert_nnp_metric_ = false;
  bool is_insert_pilot_metric_ = false;
  metric::MetricConf avp_metric_conf_;
  bool is_avp_mode_ = false;
  functionmanager::MachineStateType last_fsm_state_ =
      functionmanager::MachineStateType::INITIAL_TYPE;
  bool is_avp_parking_ = false;
  bool is_avp_crusing_ = false;
  bool last_trajacc_over_flag_ = false;
  bool is_all_acc_over_threshd = true;
  google::uint32 start_id_ = 0;
  google::uint32 end_id_ = 0;
  soc::Chassis::GearPosition parking_gear_ = soc::Chassis::GEAR_NEUTRAL;
  bool is_just_now_crusing_ = false;
  int32_t stop_id_ = -1;
  std::unordered_set<int32_t> nudge_ids_ = {};
  bool last_force_enable_dest_lat_constrian_flag_ = false;
  std::string car_alias_;
  std::string sw_ver_ = "unkown";
};

}  // namespace planning
}  // namespace TL
