/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 * Shengfa Zhu <zhushengfa@sensetime.com>
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

namespace TL {
namespace planning {

enum class InterpolationType { OUTSIDE_LINEAR = 1, END_CUTOFF = 2 };

/**
 * @brief two dimensional linear interpolation
 */
template <typename T>
class LinearInterpolator {  // NOLINT
 public:
  /**
   * @brief LinearInterpolator default constructor
   */
  LinearInterpolator() = default;

  /**
   * @brief ~LinearInterpolator default deconstructor
   */
  ~LinearInterpolator() = default;

  LinearInterpolator& operator=(const LinearInterpolator& rhs) {
    if (&rhs != this) {
      x_ = rhs.x_;
      y_ = rhs.y_;
      x_min_ = rhs.x_min_;
      x_max_ = rhs.x_max_;
      type_ = rhs.type_;
    }
    return *this;
  }

  /**
   * @brief Update independent variable and dependent variable
   *
   * @param x is vector of independent variable
   * @param y is vector of dependent variable
   *
   * @return true if input x in valid, otherwise return false
   */
  bool Update(
      const std::vector<double>& x, const std::vector<T>& y,
      const double eps = 0.1,
      const InterpolationType type = InterpolationType::OUTSIDE_LINEAR) {
    if ((x.size() != y.size()) || x.size() < 2) {
      return false;
    }

    x_.clear(), y_.clear();
    for (uint32_t i = 0; i < static_cast<uint32_t>(x.size()); ++i) {
      if (i == 0) {
        x_.emplace_back(x[i]), y_.emplace_back(y[i]);
      } else {
        if (x[i] - x_[i - 1] < eps) {
          x_.emplace_back(x_[i - 1] + eps);
        } else {
          x_.emplace_back(x[i]);
        }
        y_.emplace_back(y[i]);
      }
    }

    type_ = type;

    x_min_ = std::min(x_.front(), x_.back());
    x_max_ = std::max(x_.front(), x_.back());

    return true;
  }

  /**
   * @brief at get dependent variable according to enquried
   * independent variable
   *
   * @param x is enquried independent variable
   *
   * @return dependent variable
   */
  T at(const double x) const {
    if (type_ == InterpolationType::END_CUTOFF) {
      if (x < x_min_) {
        return y_.front();
      }
      if (x > x_max_) {
        return y_.back();
      }
    }

    // linear interpolate
    auto lower = std::lower_bound(x_.begin(), x_.end(), x);
    uint32_t index = std::distance(x_.begin(), lower);
    uint32_t index_begin = 0;
    uint32_t index_end = 0;
    if (index == 0) {
      index_begin = 0, index_end = 1;
    } else if (index == x_.size()) {
      index_begin = x_.size() - 2, index_end = x_.size() - 1;
    } else {
      index_begin = index - 1, index_end = index;
    }
    double ratio = (x - x_[index_begin]) / (x_[index_end] - x_[index_begin]);
    return y_[index_begin] + (y_[index_end] - y_[index_begin]) * ratio;
  }

  /**
   * @brief operator() get dependent variable according to enquried
   * independent variable
   *
   * @param x is enquried independent variable
   *
   * @return dependent variable
   */
  T operator()(const double x) const { return at(x); }

  /**
     * @brief Reverse reverse the y sequence
     */
  void Reverse() { std::reverse(y_.begin(), y_.end()); }

  /**
     * @brief GetMinY get min value of x seq
     *
     * @return min value of x
     */
  double GetMinX() const { return x_min_; }

  /**
   * @brief GetMaxY get Max value of x seq
   *
   * @return max value of x
   */
  double GetMaxX() const { return x_max_; }

  /**
   * @brief GetXSeq get x seq
   *
   * @return x seq
   */
  const std::vector<double>& GetXSeq() const { return x_; }

  /**
   * @brief GetYSeq get y seq
   *
   * @return y seq
   */
  const std::vector<T>& GetYSeq() { return y_; }

  /**
   * @brief IsValid
   *
   * @return true
   * @return false
   */
  bool IsValid() const { return (!x_.empty() && !y_.empty()); }

 private:
  std::vector<double> x_;
  std::vector<T> y_;
  double x_min_;
  double x_max_;
  InterpolationType type_ = InterpolationType::END_CUTOFF;
};

template <typename T>
using LinearInterpolatorPtr = std::shared_ptr<LinearInterpolator<T>>;

}  // namespace planning
}  // namespace TL
