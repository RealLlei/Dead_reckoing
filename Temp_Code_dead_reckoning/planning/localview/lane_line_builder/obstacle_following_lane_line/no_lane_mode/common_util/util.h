/******************************************************************************
 * Copyright (c) TL Technologies Co., Ltd. 2019-2022. All rights reserved.
 * Author: Lingpeng
 * Created time: 2022/05/25
 *****************************************************************************/

#pragma once

#include <Eigen/Dense>

#include <array>
#include <bitset>
#include <chrono>
#include <cmath>
#include <list>
#include <map>
#include <memory>
#include <numeric>
#include <queue>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "common/file/log.h"
#include "common/math/double_type.h"
#include "common/time/time.h"
#include "common/util/message_util.h"
#include "common/util/string_util.h"
#include "common/util/util.h"
#include "planning/common/util/util.h"
#include "planning/localview/local_view.h"

namespace TL {
namespace planning {
namespace nolane {
#ifdef NOLANE_UTIL
#define EXTERN
#else
#define EXTERN extern
#endif

EXTERN std::bitset<600> debug_flag;
EXTERN std::bitset<600> cyber_flag;

#define PRECISION(N) FIXED << SETPRECISION(N)

constexpr double LaneWidth = 3.75;

struct StateType {
  struct v2d {
    double x;
    double y;
  };

  double time_stamp;
  v2d position;
  v2d velocity;
  v2d acceleration;
  v2d position_projection;
  v2d position_fit;
  double theta;

  StateType operator/(double m) const {
    StateType rev{};
    rev.position.x = this->position.x / m;
    rev.position.y = this->position.y / m;
    rev.time_stamp = this->time_stamp;
    return rev;
  }

  // StateType& operator=(const StateType& rhs) {
  //   this->time_stamp = rhs.time_stamp;
  //   this->position = rhs.position;
  //   this->velocity = rhs.velocity;
  //   this->acceleration = rhs.acceleration;
  //   this->position_projection = rhs.position_projection;
  //   this->position_fit = rhs.position_fit;
  //   this->theta = rhs.theta;
  // }
};

/**
 * @brief operator - function
 *
 * @param lhs
 * @param rhs
 * @return StateType
 */
StateType operator-(const StateType& lhs, const StateType& rhs);

/**
 * @brief return specific state
 * @param index
 * @return
 */
inline const StateType::v2d& GetExtendStateIndex(const StateType& state) {
  return state.position_fit;
  // return state.position;
}

struct PercepObsRelPos {
  const TL::perception::PerceptionObstacle* per_obs_ptr;
  StateType pos_rel;
};

inline std::string DebugHelp(const std::string& file, int line,
                             const std::string& custom) {
  return file.substr(file.find_last_of('/') + 1, file.size()) + ":" +
         std::to_string(line) + "  " + custom;
}

using TL::common::math::double_type::DefinitelyGreater;
using TL::common::math::double_type::DefinitelyGreaterEqual;
using TL::common::math::double_type::DefinitelyLess;
using TL::common::math::double_type::DefinitelyLessEqual;
using TL::common::math::double_type::SeemsEqual;
using TL::common::math::double_type::SeemsNotEqual;

class NoLaneToCyber {
 public:
  explicit NoLaneToCyber(const std::shared_ptr<LocalView>& local_view);

  NoLaneToCyber(const NoLaneToCyber& rhs) = delete;

  NoLaneToCyber& operator=(const NoLaneToCyber& rhs) = delete;

  ~NoLaneToCyber();

  static const std::shared_ptr<TL::planning::WithoutLaneFollow>&
  GetPtrWithoutLane();

 private:
  double time_start_;
  double time_end_;
  std::shared_ptr<LocalView> local_view_;
  static std::shared_ptr<TL::planning::WithoutLaneFollow> ptr_without_lane_;
};

extern const std::shared_ptr<TL::planning::WithoutLaneFollow>&
    ptr_without_lane;

/**
 * @brief y = coef[0]*x^3 + coef[1]*x^2 + coef[2]*x + coef[3]
 * @param start_point (x,y,slope:tan(theta))
 * @param end_point (x,y,slope:tan(theta))
 * @return 3order coefficient
 */
Eigen::VectorXd PolynomialConnectTwoPoint(
    const std::array<double, 3>& start_point,
    const std::array<double, 3>& end_point);
double GetPolyValue(const std::vector<double>& coef_i, double x);

inline double GetCurvature(const std::vector<double>& coef_1,
                           const std::vector<double>& coef_2, double x);

double MaxCurvatureInPolynomial(const std::vector<double>& coef, double x_min,
                                double x_max, int interval_size = 10,
                                const double precision = 0.01);

namespace StaticFeature {
struct StaticRes {
  double mean{0.0};
  double variance{0.0};
  double std_error{0.0};
  double skewness{0.0};
  double kurtosis{0.0};
  int size{0};
  bool calc_complete{false};
  bool init{false};
};

/**
 * @brief DataValidationCheck
 *
 * @param data
 * @return true
 * @return false
 */
inline bool DataValidationCheck(const std::vector<double>& data) {
  return !data.empty();
}

/**
 * @brief Calculate the mean
 *
 * @param data
 * @return double
 */
inline double CalculateMeanValue(const std::vector<double>& data) {
  if (!DataValidationCheck(data))
    return 0;
  return std::accumulate(data.begin(), data.end(), 0.0) / data.size();
}

/**
 * @brief Calculate the variance
 *
 * @param data
 * @return double
 */
double CalculateVariance(const std::vector<double>& data);

/**
 * @brief Calculate the standard deviation
 *
 * @param data
 * @return double
 */
inline double CalculateStandardError(const std::vector<double>& data) {
  if (!DataValidationCheck(data))
    return 0;
  return data.size() == 1 ? sqrt(CalculateVariance(data) / data.size())
                          : sqrt(CalculateVariance(data) / (data.size() - 1));
}

/**
 * @brief Calculate the skewness
 *
 * @param data
 * @return double
 */
double CalculateSkewness(const std::vector<double>& data);

/**
 * @brief Calculate the kurtosis
 *
 * @param data
 * @return double
 */
double CalculateKurtosis(const std::vector<double>& data);

/**
 * @brief Calculate all the statistical feature
 *
 * @param data
 * @return StaticRes
 */
StaticRes CalculateStaticFeature(const std::vector<double>& data);
};  // namespace StaticFeature

class LogProcess {
 public:
  /**
   * @brief Log all the functions called.
   *
   * @param func
   * @return std::string&
   */
  static inline std::string& LogProc(const std::string& func = "") {
    static std::string log_process_ = "";
    if (debug_flag[50]) {
      log_process_ += func + ", ";
    }
    return log_process_;
  }

  /**
   * @brief Clear log_process_
   *
   */
  static void Clear() {
    std::string& log = LogProc();
    log.clear();
  }
};
}  // namespace nolane
}  // namespace planning
}  // namespace TL
