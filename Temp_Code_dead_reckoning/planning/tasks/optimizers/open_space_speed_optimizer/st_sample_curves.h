/******************************************************************************
 * Copyright 2019 The TL Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once

#include <array>
#include <limits>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "planning/common/speed/speed_data.h"
#include "planning/proto/open_space_task_config.pb.h"

namespace TL {
namespace planning {

static constexpr size_t kMinTShapeParamsSize = 1;
static constexpr size_t kMidTShapeParamsSize = 2;
static constexpr size_t kMaxTShapeParamsSize = 3;
static constexpr double kMinShapeTime = 0.3;
static constexpr double kMinSampleS = 0.01;
static constexpr double kMinCruiseTime = 2.0;

class StCurve final {
 public:
  struct UniformSpeedOrigin {
    double t = 0.0;
    double s = 0.0;
    double v = 0.0;
    double a = 0.0;
  };

  /**
   * @brief Construct a new St Curve object
   *
   */
  explicit StCurve() = default;

  /**
   * @brief init st sample essential params
   *
   * @param start_s
   * @param start_v
   * @param end_s
   * @param end_v
   * @param at_params
   * @return true
   * @return false
   */
  bool Init(double start_s, double start_v, double end_s, double end_v,
            const std::vector<std::array<double, 2>>& at_params);

  /**
   * @brief update origin_ by diff time
   *
   * @param time
   * @return void
   */
  void UpdateOriginByDiffTime(double time);

  /**
   * @brief evaluate tsva by time
   *
   * @param order
   * @param t
   * @param tsva
   * @return double
   */

  static double EvaluateFast(std::uint32_t order, double t,
                             const UniformSpeedOrigin& tsva);

  /**
   * @brief discrete tsva by unit_t
   *
   * @param unit_t
   * @param tsva tsva set
   * @param tsva_size valid tsva size
   * @return true
   * @return false
   */
  bool Discrete(double unit_t, std::vector<std::array<double, 4>>* tsva,
                size_t* tsva_size) const;

  /**
   * @brief dicreate tsva by unit_t
   *
   * @param unit_t
   * @param tsva
   * @return true
   * @return false
   */
  bool Discrete(double unit_t, std::vector<std::array<double, 4>>* tsva) const;

  /**
   * @brief discrete speed data by unit
   *
   * @param unit_t
   * @param speed_data_ptr
   * @return true
   * @return false
   */
  bool Discrete(double unit_t, SpeedData* speed_data_ptr) const;

  /**
   * @brief Get the Tsva Pieces object
   *
   * @return const std::vector<UniformSpeedOrigin>&
   */
  const std::vector<UniformSpeedOrigin>& GetTsvaPieces() const {
    return origins_;
  }

  /**
   * @brief Get the Total Time object
   *
   * @return double
   */
  double GetTotalTime() const { return total_t_; }

  void SetCostInfo(double effi_cost, double acc_cost, double jerk_cost,
                   double over_cost, double diff_cost) {
    efficiency_cost_ = effi_cost;
    acc_cost_ = acc_cost;
    jerk_cost_ = jerk_cost;
    over_speed_cost_ = over_cost;
    diff_cost_ = diff_cost;
    total_cost_ = efficiency_cost_ + acc_cost_ + jerk_cost_ + over_speed_cost_ +
                  diff_cost_;
    cost_valid_flag_ = true;
  }

  void ResetCostInfo() {
    efficiency_cost_ = std::numeric_limits<double>::max();
    acc_cost_ = std::numeric_limits<double>::max();
    jerk_cost_ = std::numeric_limits<double>::max();
    over_speed_cost_ = std::numeric_limits<double>::max();
    diff_cost_ = std::numeric_limits<double>::max();
    total_cost_ = std::numeric_limits<double>::max();
    cost_valid_flag_ = false;
  }

  std::shared_ptr<StCurve> Clone() const {
    return std::make_shared<StCurve>(*this);
  }

  /**
   * @brief
   *
   * @return * std::string
   */
  std::string DebugInfo() const;

 private:
  std::vector<UniformSpeedOrigin> origins_{UniformSpeedOrigin{0, 0, 0, 0},
                                           UniformSpeedOrigin{0, 0, 0, 0},
                                           UniformSpeedOrigin{0, 0, 0, 0}};
  double total_t_ = 0.0;
  double start_s_ = 0.0;
  double start_v_ = 0.0;
  double end_s_ = 0.0;
  double end_v_ = 0.0;

  double efficiency_cost_ = std::numeric_limits<double>::max();
  double acc_cost_ = std::numeric_limits<double>::max();
  double jerk_cost_ = std::numeric_limits<double>::max();
  double over_speed_cost_ = std::numeric_limits<double>::max();
  double diff_cost_ = std::numeric_limits<double>::max();
  double total_cost_ = std::numeric_limits<double>::max();
  bool cost_valid_flag_ = false;
};

struct StSampleParams {
  double start_s = 0.0;
  double start_v = 0.0;
  double start_acc = 0.0;
  double end_s = 0.0;
  double end_v = 0.0;
  double unit_acc = 0.02;
  double unit_max_v = 0.1;
  double efficiency_cost = 10;
  double acc_cost = 1;
  double jerk_cost = 10;
  double over_speed_cost_max = 100;
  double over_speed_cost_min = 10;
  double diff_cost = 10;
  OpenSpaceSpeedOptimizerConfig::SpeedBoundInfo speed_bound_info;
};

class StSampleCurves final {
 public:
  /**
   * @brief Construct a new St Sample Curves object
   *
   */
  StSampleCurves();

  /**
   * @brief init sample essential params
   *
   * @param sample_params
   */
  bool Init(const StSampleParams& sample_params);

  /**
   * @brief sample process
   *
   * @param sample_params
   * @return size_t
   */
  size_t SampleProcess(const StSampleParams& sample_params);

  /**
   * @brief Get the St Sample Curves object
   *
   * @return const std::vector<StCurve>&
   */
  const std::vector<StCurve>& GetStSampleCurves() const {
    return t_shape_curves_;
  }

  std::vector<StCurve>* GetMutableStSampleCurves() { return &t_shape_curves_; }

  /**
   * @brief Get the St Sample Curve Size object
   *
   * @return size_t
   */
  size_t GetStSampleCurveSize() const { return t_shape_curve_size_; }

  /**
   * @brief Get the Min Samplt T object
   *
   * @return double
   */
  double GetMinSampleT() const { return min_sample_t_; }

  /**
   * @brief Get the Max Sample T object
   *
   * @return double
   */
  double GetMaxSampleT() const { return max_sample_t_; }

 private:
  /**
 * @brief judge if at params valid
 *
 * @param at_params [[acc, t]...]
 * @return true
 * @return false
 */
  bool IsTShapeParamsValid(
      const std::vector<std::array<double, 2>>& at_params) const;

  /**
   * @brief
   *
   * @param at_params
   */
  static void PrintTShapeParams(
      const std::vector<std::array<double, 2>>& at_params);

  /**
   * @brief
   *
   */
  void SampleAcc();

  /**
   * @brief
   *
   */
  void SampleMaxVel();

  /**
   * @brief
   *
   * @param max_acc
   * @param min_acc
   * @param max_v
   */
  bool CalStParamsDirectly(double max_acc, double min_acc, double max_v,
                           std::vector<std::array<double, 2>>* at_params) const;

  /**
   * @brief
   *
   * @param t
   * @return size_t
   */
  size_t SampleCurvesUseMaxV(double max_v);

 private:
  size_t t_shape_curve_size_ = 0;
  std::vector<StCurve> t_shape_curves_;
  double min_sample_t_ = std::numeric_limits<double>::max();
  double max_sample_t_ = 0.0;

  StSampleParams sample_params_;

  std::vector<double> candidate_max_accs_;
  std::vector<double> candidate_min_accs_;
  std::vector<double> candidate_max_v_;
};

}  // namespace planning
}  // namespace TL
