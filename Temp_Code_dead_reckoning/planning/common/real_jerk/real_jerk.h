#ifndef PLANNING_COMMON_REAL_JERK_REAL_JERK_H
#define PLANNING_COMMON_REAL_JERK_REAL_JERK_H

/******************************************************************************
 * Copyright 2023 The TL Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <sys/types.h>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "planning/common/real_jerk/acc_filter.h"
#include "planning/localview/local_view.h"

#include "proto/fsm/metric_conf.pb.h"

namespace TL {
namespace planning {

struct JerkErr {
  bool has_data = false;
  double timestamp = 0.0;
  double jerk = 0.0;
};

class RealJerk {
 public:
  RealJerk();
  /**
   * @brief update acc data
   * @param local_view 
   * @param ptr_trajectory_pb 
   */
  void UpdateAccData(const std::shared_ptr<LocalView>& local_view,
                     const std::shared_ptr<ADCTrajectory>& ptr_trajectory_pb);
  /**
   * @brief calc NTPlat jerk
   * @param local_view 
   * @param lat_acc_out
   */
  bool CalcNtpLatJerk(const std::shared_ptr<LocalView>& local_view,
                      double* lat_acc_out);
  /**
   * @brief calc NTPlon jerk
   * @param local_view
   * @param lon_acc_out
   */
  bool CalcNtpLonJerk(const std::shared_ptr<LocalView>& local_view,
                      double* lon_acc_out);

 private:
  /**
   * @brief calc lat jerk
   * @param lat_acc 
   * @param timestamp 
   * @param ptr_trajectory_pb 
   */
  void CalcLatJerk(double lat_acc, double timestamp,
                   const std::shared_ptr<ADCTrajectory>& ptr_trajectory_pb);

  void FilterSteerRate(double steer_rate,
                       const std::shared_ptr<ADCTrajectory>& ptr_trajectory_pb);

  /**
   * @brief calc lon jerk
   * @param lon_acc 
   * @param timestamp 
   * @param ptr_trajectory_pb 
   */
  void CalcLonJerk(double lon_acc, double timestamp,
                   const std::shared_ptr<ADCTrajectory>& ptr_trajectory_pb);
  AccFilter lon_acc_filter_;
  AccFilter lat_acc_filter_;
  AccFilter steer_rate_filter_;
  bool is_lat_active_ = false;
  bool is_lon_active_ = false;
  metric::MetricConf metric_conf_;
};
}  // namespace planning
}  // namespace TL

#endif  // PLANNING_COMMON_REAL_JERK_REAL_JERK_H
