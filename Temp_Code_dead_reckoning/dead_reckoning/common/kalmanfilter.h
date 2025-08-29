/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include <limits>
#include <vector>
#include "dead_reckoning/common/structs.h"
namespace apollo {
namespace dead_reckoning {

class KalmanFilterCorrector {
 public:
  unsigned int _len_sw = 0;  // sliding window length of history residual error
  double _chi_thr = 1.0;     // chi-square threshold
  Eigen::VectorXd _gain_coef;  // gain coef

  void AdaptiveCorrect(Eigen::VectorXd* x, Eigen::MatrixXd* P,
                       const Eigen::VectorXd& res_error,
                       const Eigen::MatrixXd& H, const Eigen::MatrixXd& R);

  void BasicCorrect(Eigen::VectorXd* x, Eigen::MatrixXd* P,
                    const Eigen::VectorXd& res_error, const Eigen::MatrixXd& H,
                    const Eigen::MatrixXd& R);

 private:
  std::vector<Eigen::VectorXd> _hist_res_error;  // history residual error
  Eigen::MatrixXd _est_sum_res_var;  // estimated sum of residual variance
  unsigned int _cur_idx;             // current index of residual error
};

void KalmanFilterCorrect(
    Eigen::VectorXd* x, Eigen::MatrixXd* P, const Eigen::VectorXd& res_error,
    const Eigen::MatrixXd& H, const Eigen::MatrixXd& R,
    const Eigen::MatrixXd& proj_mat = Eigen::MatrixXd(),
    double chi_square_threshold = std::numeric_limits<double>::max());

}  // namespace dead_reckoning
}  // namespace apollo
