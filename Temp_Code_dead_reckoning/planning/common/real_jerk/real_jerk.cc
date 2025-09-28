/******************************************************************************
 * Copyright 2023 The TL Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/common/real_jerk/real_jerk.h"

#include "common/file/file.h"
#include "common/math/double_type.h"
#include "planning/common/planning_gflags.h"
#include "proto/fsm/nnp_fct.pb.h"

namespace TL {
namespace planning {

namespace {
constexpr double kEpsilon = 1e-6;
}  // namespace

// using TL::common::Clock;

RealJerk::RealJerk() {
  if (!TL::common::GetProtoFromFile(FLAGS_metric_config_file,
                                       &metric_conf_)) {
    AERROR << "Failed to load obs follow time config file "
           << FLAGS_metric_config_file;
  }
  lon_acc_filter_.Init(metric_conf_.metric_filter().filter_ts(),
                       metric_conf_.metric_filter().filter_cutoff_freq());
  lat_acc_filter_.Init(metric_conf_.metric_filter().filter_ts(),
                       metric_conf_.metric_filter().filter_cutoff_freq());
  steer_rate_filter_.Init(metric_conf_.metric_filter().filter_ts(),
                          metric_conf_.metric_filter().filter_cutoff_freq());
}

void RealJerk::UpdateAccData(
    const std::shared_ptr<LocalView>& local_view,
    const std::shared_ptr<ADCTrajectory>& ptr_trajectory_pb) {
  if (!local_view->HasFunctionManagerIn() || !local_view->HasChassis()) {
    AERROR << "update acc data ptr is nullptr";
    return;
  }
  const auto& fct_in = local_view->GetFunctionManagerIn();
  const auto& driver_mode = fct_in->driver_mode();
  if (fct_in->has_fct_nnp_in()) {
    const auto& nnp_sys_state = fct_in->fct_nnp_in().nnp_sysstate();
    is_lat_active_ =
        (nnp_sys_state == functionmanager::NNPSysState::NNPS_ACTIVE ||
         nnp_sys_state == functionmanager::NNPSysState::NNPS_LON_OVERRIDE);
    is_lon_active_ =
        (nnp_sys_state == functionmanager::NNPSysState::NNPS_ACTIVE ||
         nnp_sys_state == functionmanager::NNPSysState::NNPS_LAT_OVERRIDE);
  }
  // only deal lat or lon active
  is_lat_active_ =
      (is_lat_active_ ||
       driver_mode ==
           functionmanager::DriveMode::ADAS_LAT_ACTIVE_LGT_OVERRIDE ||
       driver_mode == functionmanager::DriveMode::ADAS_LAT_LGT_ACTIVE ||
       driver_mode == functionmanager::DriveMode::AVP_LAT_ACTIVE_LGT_OVERRIDE ||
       driver_mode == functionmanager::DriveMode::AVP_LAT_LGT_ACTIVE ||
       driver_mode == functionmanager::DriveMode::AVP_CRUSING);
  is_lon_active_ =
      (is_lon_active_ ||
       driver_mode == functionmanager::DriveMode::ADAS_LAT_LGT_ACTIVE ||
       driver_mode ==
           functionmanager::DriveMode::ADAS_LGT_ACTIVE_LAT_OVERRIDE ||
       driver_mode == functionmanager::DriveMode::AVP_LAT_LGT_ACTIVE ||
       driver_mode == functionmanager::DriveMode::AVP_LGT_ACTIVE_LAT_OVERRIDE ||
       driver_mode == functionmanager::DriveMode::AVP_CRUSING);
  const auto& chassis = local_view->GetChassis();
  if (ptr_trajectory_pb != nullptr && chassis->has_header() &&
      chassis->has_imu_acc() && chassis->has_steering_rate()) {
    double time_stamp = chassis->header().data_stamp();
    const auto& lat_acc = chassis->imu_acc().x();
    const auto& lon_acc = chassis->imu_acc().y();
    CalcLatJerk(lat_acc, time_stamp, ptr_trajectory_pb);
    CalcLonJerk(lon_acc, time_stamp, ptr_trajectory_pb);
    FilterSteerRate(chassis->steering_rate(), ptr_trajectory_pb);
  }
}

void RealJerk::FilterSteerRate(
    double steer_rate,
    const std::shared_ptr<ADCTrajectory>& ptr_trajectory_pb) {
  if (std::isnan(steer_rate)) {
    return;
  }
  auto* nnp_metric =
      ptr_trajectory_pb->mutable_function_manager_out()->mutable_nnp_metric();
  double output_steer_rate = steer_rate_filter_.OutputFilter(steer_rate);
  nnp_metric->mutable_filter_acc_info()->set_steer_rate(output_steer_rate);
}

void RealJerk::CalcLatJerk(
    const double lat_acc, const double timestamp,
    const std::shared_ptr<ADCTrajectory>& ptr_trajectory_pb) {
  static double last_lat_acc = -1.0;
  static double last_timestamp = -1.0;
  if (std::isnan(lat_acc)) {
    last_timestamp = -1.0;
    return;
  }
  auto* nnp_metric =
      ptr_trajectory_pb->mutable_function_manager_out()->mutable_nnp_metric();
  double output_lat_acc = lat_acc_filter_.OutputFilter(lat_acc);
  nnp_metric->mutable_filter_acc_info()->set_lat_acc(output_lat_acc);
  if (is_lat_active_ && last_timestamp > 0.0 &&
      common::math::double_type::DefinitelyGreater((timestamp - last_timestamp),
                                                   kEpsilon)) {
    double lat_jerk =
        (output_lat_acc - last_lat_acc) / (timestamp - last_timestamp);
    if (std::fabs(lat_jerk) >
        metric_conf_.metric_lat_lon().lat_jerk_threshold()) {
      auto* jerk_err = nnp_metric->mutable_jerk_err_info();
      jerk_err->set_lat(lat_jerk);
      jerk_err->set_timestamp(timestamp);
    }
  } else {
    last_timestamp = -1.0;
  }
  last_timestamp = timestamp;
  last_lat_acc = output_lat_acc;
}

void RealJerk::CalcLonJerk(
    const double lon_acc, const double timestamp,
    const std::shared_ptr<ADCTrajectory>& ptr_trajectory_pb) {
  static double last_lon_acc = -1.0;
  static double last_timestamp = -1.0;
  if (std::isnan(lon_acc)) {
    last_timestamp = -1.0;
    return;
  }
  auto* nnp_metric =
      ptr_trajectory_pb->mutable_function_manager_out()->mutable_nnp_metric();
  double output_lon_acc = lon_acc_filter_.OutputFilter(lon_acc);
  nnp_metric->mutable_filter_acc_info()->set_lon_acc(output_lon_acc);
  if (is_lon_active_ && last_timestamp > 0.0 &&
      common::math::double_type::DefinitelyGreater((timestamp - last_timestamp),
                                                   kEpsilon)) {
    double lon_jerk =
        (output_lon_acc - last_lon_acc) / (timestamp - last_timestamp);
    if (std::fabs(lon_jerk) >
        metric_conf_.metric_lat_lon().lon_jerk_threshold()) {
      auto* jerk_err = nnp_metric->mutable_jerk_err_info();
      jerk_err->set_lon(lon_jerk);
      jerk_err->set_timestamp(timestamp);
    }
  } else {
    last_timestamp = -1.0;
  }
  last_timestamp = timestamp;
  last_lon_acc = output_lon_acc;
}

bool RealJerk::CalcNtpLatJerk(const std::shared_ptr<LocalView>& local_view,
                              double* const lat_acc_out) {
  static double last_lat_acc = -1.0;
  static double last_timestamp = -1.0;
  bool flag = false;
  if (!local_view->HasFunctionManagerIn() || !local_view->HasChassis() ||
      !local_view->HasFunctionManagerOut()) {
    return flag;
  }
  bool is_ntp_active = false;
  const auto fsm_state = local_view->GetFunctionManagerOut()->fsm_state();
  const auto& fct_in = local_view->GetFunctionManagerIn()->fct_avp_in();
  is_ntp_active =
      (fsm_state == functionmanager::MachineStateType::HDMAP_AVP_TYPE &&
       fct_in.sys_run_state() != functionmanager::AvpFctIn::PAUSE);
  const auto& chassis = local_view->GetChassis();
  double timestamp = chassis->header().data_stamp();
  const auto& lat_acc = chassis->imu_acc().x();
  if (std::isnan(lat_acc)) {
    last_timestamp = -1.0;
    return flag;
  }
  double output_lat_acc = lat_acc_filter_.OutputFilter(lat_acc);
  if (is_ntp_active && last_timestamp > 0.0 &&
      common::math::double_type::DefinitelyGreater((timestamp - last_timestamp),
                                                   kEpsilon)) {
    double lat_jerk =
        (output_lat_acc - last_lat_acc) / (timestamp - last_timestamp);
    if (std::fabs(lat_jerk) >
        metric_conf_.ntp_metric_lat_lon().ntp_lat_jerk_threshold()) {
      *lat_acc_out = lat_jerk;
      flag = true;
    }
  }
  last_timestamp = timestamp;
  last_lat_acc = output_lat_acc;
  return flag;
}

bool RealJerk::CalcNtpLonJerk(const std::shared_ptr<LocalView>& local_view,
                              double* const lon_acc_out) {
  static double last_lon_acc = -1.0;
  static double last_timestamp = -1.0;
  bool flag = false;
  if (!local_view->HasFunctionManagerIn() || !local_view->HasChassis() ||
      !local_view->HasFunctionManagerOut()) {
    return flag;
  }
  const auto fsm_state = local_view->GetFunctionManagerOut()->fsm_state();
  const auto& fct_in = local_view->GetFunctionManagerIn()->fct_avp_in();
  bool is_ntp_active = false;
  is_ntp_active =
      (fsm_state == functionmanager::MachineStateType::HDMAP_AVP_TYPE &&
       fct_in.sys_run_state() != functionmanager::AvpFctIn::PAUSE);
  const auto& chassis = local_view->GetChassis();
  double timestamp = chassis->header().data_stamp();
  const auto& lon_acc = chassis->imu_acc().y();
  if (std::isnan(lon_acc)) {
    last_timestamp = -1.0;
    return flag;
  }
  double output_lon_acc = lon_acc_filter_.OutputFilter(lon_acc);
  if (is_ntp_active && last_timestamp > 0.0 &&
      common::math::double_type::DefinitelyGreater((timestamp - last_timestamp),
                                                   kEpsilon)) {
    double lon_jerk =
        (output_lon_acc - last_lon_acc) / (timestamp - last_timestamp);
    if (std::fabs(lon_jerk) >
        metric_conf_.ntp_metric_lat_lon().ntp_lon_jerk_threshold()) {
      *lon_acc_out = lon_jerk;
      flag = true;
    }
  }
  last_timestamp = timestamp;
  last_lon_acc = output_lon_acc;
  return flag;
}

}  // namespace planning
}  // namespace TL
