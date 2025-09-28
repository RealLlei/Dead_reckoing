/******************************************************************************
 * Copyright 2023 The TL Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/common/real_jerk/acc_filter.h"

#include "common/filters/digital_filter_coefficients.h"

namespace TL {
namespace planning {
void AccFilter::Init(const double ts, const double cut_off_freq) {
  // Low pass filter
  std::vector<double> den(3, 0.0);
  std::vector<double> num(3, 0.0);
  TL::common::LpfCoefficients(ts, cut_off_freq, &den, &num);
  acc_digital_filter_.set_coefficients(den, num);
}

double AccFilter::OutputFilter(double input_val) {
  return acc_digital_filter_.Filter(input_val);
}

}  // namespace planning
}  // namespace TL
