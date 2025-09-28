/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file speed_curve.h
 **/

#pragma once

#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "common/math/math_utils.h"
#include "common/util/macros.h"
#include "planning/common/speed/speed_data.h"

#include "planning/proto/speed_evaluator_config.pb.h"
#include "planning/proto/task_config.pb.h"

namespace TL {
namespace planning {

struct alignas(CACHELINE_SIZE) SpeedCurveTarget {
  enum class Mode {
    UNKNOWN = 0,
    CRUISE = 1,
    FOLLOW = 2,
    STOP = 3,
    STOP_TO_STANDSTILL = 4,
  };

  enum class Type { GUIDE = 0, SAFE = 1, NORMAL = 2 };

  Mode mode = Mode::CRUISE;
  Type type = Type::NORMAL;
  double speed = 0.0;
  std::string obstacle_id;
};

class SpeedCurvePoint {
 public:
  /**
   * @brief Get t
   *
   * @return double
   */
  [[nodiscard]] double t() const { return t_; }

  /**
   * @brief Get s
   *
   * @return double
   */
  [[nodiscard]] double s() const { return s_; }

  /**
   * @brief Get v
   *
   * @return double
   */
  [[nodiscard]] double v() const { return v_; }

  /**
   * @brief Get a
   *
   * @return double
   */
  [[nodiscard]] double a() const { return a_; }

  /**
   * @brief Get j
   *
   * @return double
   */
  [[nodiscard]] double j() const { return j_; }

  /**
   * @brief Set t
   *
   * @param t
   */
  void set_t(double t) { t_ = t; }

  /**
   * @brief Set s
   *
   * @param s
   */
  void set_s(double s) { s_ = s; }

  /**
   * @brief Set v
   *
   * @param v
   */
  void set_v(double v) { v_ = v; }

  /**
   * @brief Set a
   *
   * @param a
   */
  void set_a(double a) { a_ = a; }

  /**
   * @brief Set j
   *
   * @param j
   */
  void set_j(double j) { j_ = j; }

 private:
  double t_ = 0.0;
  double s_ = 0.0;
  double v_ = 0.0;
  double a_ = 0.0;
  double j_ = 0.0;
};

/**
 * @class SpeedCurve
 * @brief speed curve
 */
class alignas(CACHELINE_SIZE) SpeedCurve {
 public:
  explicit SpeedCurve(const SpeedCurveConfig& config);
  virtual ~SpeedCurve() = default;

  /**
   * @brief Discretize the curve to vt graph points
   *
   * @param total_t total time length
   * @param unit_t sample interval
   * @param process_peak_value true: process peak value, false ignore peak value
   * @param vt_graph_points
   * @return int
   */
  virtual int Discretize(
      double total_t, double unit_t, bool process_peak_value,
      std::vector<SpeedCurvePoint>* vt_graph_points) const = 0;
  /**
   * @brief Discretize the curve to speed data
   * 
   * @param total_t total time length
   * @param unit_t sample interval
   * @param speed_data 
   */
  void Discretize(double total_t, double unit_t, SpeedData* speed_data) const;

  int DiscretizeSparsePoint();

  int DiscretizeDensePoint();

  /**
   * @brief Get start time
   *
   * @return double
   */
  [[nodiscard]] double GetStartTime() const { return start_time_; }

  /**
   * @brief Get end time
   *
   * @return double
   */
  [[nodiscard]] double GetEndTime() const { return end_time_; }

  /**
   * @brief Get start s
   *
   * @return double
   */
  [[nodiscard]] double GetStartS() const { return start_s_; }

  /**
   * @brief Get end s
   *
   * @return double
   */
  [[nodiscard]] double GetEndS() const { return end_s_; }

  /**
   * @brief Get start v
   *
   * @return double
   */
  [[nodiscard]] double GetStartV() const { return start_v_; }

  /**
   * @brief Get end v
   *
   * @return double
   */
  [[nodiscard]] double GetEndV() const { return end_v_; }

  /**
   * @brief Get min v
   *
   * @return double
   */
  [[nodiscard]] double GetMinV() const { return min_v_; }

  /**
   * @brief Get max v
   *
   * @return double
   */
  [[nodiscard]] double GetMaxV() const { return max_v_; }

  /**
   * @brief Get start accel
   *
   * @return double
   */
  [[nodiscard]] double GetStartAccel() const { return start_accel_; }

  /**
   * @brief Get max accel
   *
   * @return double
   */
  [[nodiscard]] double GetMaxAccel() const { return max_accel_; }

  /**
   * @brief Get min accel
   *
   * @return double
   */
  [[nodiscard]] double GetMinAccel() const { return min_accel_; }

  /**
   * @brief Get start jerk
   *
   * @return double
   */
  [[nodiscard]] double GetStartJerk() const { return start_jerk_; }

  /**
   * @brief Get end jerk
   *
   * @return double
   */
  [[nodiscard]] double GetEndJerk() const { return end_jerk_; }

  /**
   * @brief Get the Max Jerk object
   *
   * @return double
   */
  [[nodiscard]] double GetMaxJerk() const { return max_jerk_; }

  /**
   * @brief Get the Min Jerk object
   *
   * @return double
   */
  [[nodiscard]] double GetMinJerk() const { return min_jerk_; }

  /**
   * @brief get s length
   *
   * @return double
   */
  [[nodiscard]] virtual double GetSLength() const { return end_s_ - start_s_; }

  /**
   * @brief Get time length
   *
   * @return double
   */
  [[nodiscard]] virtual double GetTimeLength() {
    return end_time_ - start_time_;
  }

  /**
   * @brief Get target
   *
   * @return const SpeedCurveTarget&
   */
  [[nodiscard]] const SpeedCurveTarget& GetTarget() const { return target_; }

  // sparse_points_ is sampled in a long time interval
  [[nodiscard]] const std::vector<SpeedCurvePoint>& GetSparsePoints() const {
    return sparse_points_;
  }

  // dense_points_ is sample in a short time interval
  [[nodiscard]] const std::vector<SpeedCurvePoint>& GetDensePoints() const {
    return dense_points_;
  }

  // min_sparse_point_count_ means from 0s to 7s
  [[nodiscard]] int GetMinSparsePointCount() const {
    return min_sparse_point_count_;
  }

  [[nodiscard]] int GetMaxSparsePointCount() const {
    return max_sparse_point_count_;
  }

  [[nodiscard]] int GetCurveSparsePointCount() const {
    return curve_sparse_point_count_;
  }

  [[nodiscard]] int GetMinDensePointCount() const {
    return min_dense_point_count_;
  }

  [[nodiscard]] int GetMaxDensePointCount() const {
    return max_dense_point_count_;
  }

  [[nodiscard]] int GetCurveDensePointCount() const {
    return curve_dense_point_count_;
  }

  [[nodiscard]] double GetSparsePointInterval() const {
    return sparse_point_interval_;
  }

  [[nodiscard]] double GetDensePointInterval() const {
    return dense_point_interval_;
  }

  /**
   * @brief Check whether this vt sample curve is used in last frame
   *
   * @return true
   * @return false
   */
  [[nodiscard]] bool LastSelected() const { return start_time_ > 5e-2; }

  /**
   * @brief Get vt graph point
   *
   * @param t
   * @return StGraphPoint
   */
  [[nodiscard]] virtual SpeedCurvePoint GetPoint(double t) const = 0;

  [[nodiscard]] virtual SpeedCurvePoint& GetNearestPoint(double t) {
    const auto index =
        common::math::Clamp(static_cast<int>(round(t / dense_point_interval_)),
                            0, GetMaxDensePointCount() - 1);
    return dense_points_[index];
  }

  /**
   * @brief Set start point
   *
   * @param estimated_start_time
   * @return true
   * @return false
   */
  virtual bool SetStartPoint(double estimated_start_time) = 0;

  const std::vector<std::pair<bool, double>>& GetPeekV() const {
    return peek_v_;
  }

  /**
  * @brief Clone curve
  * 
  * @return std::shared_ptr<VtSampleCurve> 
  */
  [[nodiscard]] virtual std::shared_ptr<SpeedCurve> Clone() const = 0;

  /**
   * @brief Get debug string
   * 
   * @return std::string 
   */
  [[nodiscard]] virtual std::string DebugString() const = 0;

 protected:
  // curve target info
  SpeedCurveTarget target_;

  // curve time info
  double start_time_ = 0.0;
  double end_time_ = 0.0;

  // curve s info
  double start_s_ = 0.0;
  double end_s_ = 0.0;

  // curve speed info
  double start_v_ = 0.0;
  double end_v_ = 0.0;
  double max_v_ = std::numeric_limits<double>::lowest();
  double min_v_ = std::numeric_limits<double>::max();

  // curve accel info
  double start_accel_ = 0.0;
  double end_accel_ = 0.0;
  double max_accel_ = std::numeric_limits<double>::lowest();
  double min_accel_ = std::numeric_limits<double>::max();

  // curve jerk info
  double start_jerk_ = 0.0;
  double end_jerk_ = 0.0;
  double max_jerk_ = std::numeric_limits<double>::lowest();
  double min_jerk_ = std::numeric_limits<double>::max();

  // sparse_points_ is sampled in a long time interval
  std::vector<SpeedCurvePoint> sparse_points_;
  // dense_points_ is sample in a short time interval
  std::vector<SpeedCurvePoint> dense_points_;
  // min_sparse_point_count_ means from 0s to 7s
  int min_sparse_point_count_ = 0;
  // max_sparse_point_count_ means from 0s to vt graph time length
  int max_sparse_point_count_ = 0;
  // min_dense_point_count_ means from 0s to 7s
  int min_dense_point_count_ = 0;
  // max_sparse_point_count_ means from 0s to vt graph time length
  int max_dense_point_count_ = 0;
  // curve_sparse_point_count_ means from 0s to curve time length
  int curve_sparse_point_count_ = 0;
  // curve_dense_point_count_ means from 0s to curve time length
  int curve_dense_point_count_ = 0;
  // sparse point interval_
  double sparse_point_interval_ = 1.0;
  // dense point interval_
  double dense_point_interval_ = 0.2;
  std::vector<std::pair<bool, double>> peek_v_;
};

}  // namespace planning
}  // namespace TL
