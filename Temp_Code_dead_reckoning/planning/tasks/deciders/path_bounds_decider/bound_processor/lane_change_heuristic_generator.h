/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2022. All rights reserved.
 * Description:  planning path lane change heuristic path generator
 * Author: ROC
 */

namespace TL {
namespace planning {
class LaneChangeHeuristicGenerator {
 public:
  /**
   * @brief Compute Lane Change Length
   * 
   * @param v current adc speed(m/s)
   * @return double 
   */
  static double ComputeLaneChangeLength(double v);

  /**
   * @brief Init Success
   * 
   * @return true 
   * @return false 
   */
  [[nodiscard]] bool InitSuccess() const;

  /**
   * @brief Init
   * cos function to generate a heuristic lane change reference curve
   * Init vehicle state（s0、l0、dl0）
   * f(s) = A * (1+cos(B*s+C)), parameters(A，B，C)
   * [1]f(s0) = l0
   * [2]f’(s0) = dl0
   * [3]B = 2 * PI/T
   * [4]T = 2 * g(v)
   * [5]g(v) = 5.5 * v + 0.2 * v * v, g(v): heuristic lane change finish distance
   * @param s0 
   * @param l0 
   * @param lane_change_length 
   * @param dl0 
   */
  void Init(double s0, double l0, double dl0, double lane_change_length);

  /**
   * @brief Init
   * cos function to generate a heuristic lane change reference curve
   * Init vehicle state（s0、l0、dl0）
   * f(s) = A * (1+cos(B*s+C)), parameters(A，B，C)
   * [1]f(s0) = l0
   * [2]f’(s0) = dl0
   * [3]B = 2 * PI/T
   * [4]T = 2 * g(v)
   * [5]g(v) = 5.5 * v + 0.2 * v * v, g(v): heuristic lane change finish distance
   * @param s0 
   * @param l0 
   * @param lane_change_length 
   * @param dl0  
   * @param s_prepare 
   * @param l_limit 
   * @param dl_limit 
   */
  void Init(double s0, double l0, double dl0, double lane_change_length,
            double s_prepare, double l_limit, double dl_limit,
            bool enable_dl_limit);

  /**
   * @brief Get L With S
   * 
   * @param s 
   * @return double 
   */
  [[nodiscard]] double GetLWithS(double s) const;

  /**
   * @brief Get the Dl With S
   * 
   * @param s 
   * @return double 
   */
  [[nodiscard]] double GetDlWithS(double s) const;

 private:
  double s0_ = 0.0;
  double l0_ = 0.0;
  double s1_ = 0.0;
  double l1_ = 0.0;
  double l0_1_ = 0.0;
  double s0_1_ = 0.0;
  double dl0_ = 0.0;
  // double dl1_ = 0.0;
  double s_prepare_ = 0.0;
  double a_ = 0.0;
  double b_ = 0.0;
  double c_ = 0.0;
  double s_zero_ = 0.0;

  bool init_success_ = false;
};
}  // namespace planning
}  // namespace TL
