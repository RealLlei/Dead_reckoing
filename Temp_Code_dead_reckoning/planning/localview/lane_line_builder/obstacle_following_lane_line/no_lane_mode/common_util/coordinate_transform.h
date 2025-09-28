/******************************************************************************
 * Copyright (c) TL Technologies Co., Ltd. 2019-2022. All rights reserved.
 * Author: LingPeng
 * Created Time: 2022/06/29
 *****************************************************************************/

#pragma once

#include <Eigen/Dense>

#include <vector>

#include "common/file/log.h"

namespace TL::planning::nolane {

template <size_t N, typename T>
class CoordinateTransform {
 public:
  CoordinateTransform() = default;

  /**
   * @brief Construct a new Coordinate Transform object
   *
   * @param base_vector
   */
  explicit CoordinateTransform(Eigen::Matrix<T, N, N> base_vector)
      : rotate_vector_(base_vector) {}

  ~CoordinateTransform() = default;
  CoordinateTransform(const CoordinateTransform<N, T>& rhs) = delete;
  CoordinateTransform<N, T>& operator=(const CoordinateTransform<N, T>& rhs) =
      delete;

  /**
   * @brief Get In Origin
   *
   * @param lhs
   * @param rhs
   * @return true
   * @return false
   */
  bool InOrigin(const std::vector<T>& lhs, std::vector<T>* const rhs) {
    if (lhs.size() != N) {
      return false;
    }
    for (int i = 0; i < N; ++i) {
      input_(i, 0) = lhs.at(i);
    }
    output_ = rotate_vector_ * input_;
    rhs->clear();
    for (int i = 0; i < N; ++i) {
      rhs->push_back(output_(i, 0));
    }
    return true;
  }

  /**
   * @brief Origin2NewBaseRotate
   *
   * @param lhs
   * @param rhs
   * @return true
   * @return false
   */
  bool Origin2NewBaseRotate(const std::vector<T>& lhs,
                            std::vector<T>* const rhs) {
    return InOrigin(lhs, rhs);
  }

  /**
   * @brief NewBase2OriginRotate
   *
   * @param lhs
   * @param rhs
   * @return true
   * @return false
   */
  bool NewBase2OriginRotate(const std::vector<T>& lhs,
                            std::vector<T>* const rhs) {
    if (lhs.size() != N) {
      return false;
    }
    for (int i = 0; i < N; ++i) {
      input_(i, 0) = lhs.at(i);
    }
    output_ = rotate_vector_.inverse() * input_;
    rhs->clear();
    for (int i = 0; i < N; ++i) {
      rhs->push_back(output_(i, 0));
    }
    return true;
  }

  /**
   * @brief Set the New Base Vector object
   *
   * @param newBaseVector
   */
  void SetNewBaseVector(const Eigen::Matrix<T, N, N>& newBaseVector) {
    rotate_vector_ = newBaseVector;
  }

  const Eigen::Matrix<T, N, N>& GetNewBaseVector() const {
    return rotate_vector_;
  }

 private:
  Eigen::Matrix<T, N, N> rotate_vector_;
  Eigen::Vector<T, N> input_;
  Eigen::Vector<T, N> output_;
};

template <typename T>
class CoordinateTransform<2, T> {
 public:
  /**
   * @brief Construct a new Coordinate Transform object
   *
   */
  CoordinateTransform() {
    angle_ = M_PI_2;
    rotate_vector_(0, 0) = 1;
    rotate_vector_(0, 1) = 0;
    rotate_vector_(1, 0) = 0;
    rotate_vector_(1, 1) = 1;
    pan_(0, 0) = 0;
    pan_(1, 0) = 0;
  }

  explicit CoordinateTransform(Eigen::Matrix<T, 2, 2> base_vector)
      : rotate_vector_(base_vector) {}

  ~CoordinateTransform() = default;

  /**
   * @brief InOrigin
   *
   * @param lhs
   * @param rhs
   * @return true
   * @return false
   */
  bool InOrigin(const std::vector<T>& lhs, std::vector<T>* const rhs) {
    if (lhs.size() != 2) {
      return false;
    }
    input_(0, 0) = lhs.front();
    input_(1, 0) = lhs.back();
    output_ = rotate_vector_ * input_;
    rhs->clear();
    rhs->push_back(output_(0, 0));
    rhs->push_back(output_(1, 0));
    return true;
  }

  /**
   * @brief NewBase2OriginRotate
   *
   * @param lhs
   * @param rhs
   * @return true
   * @return false
   */
  inline bool NewBase2OriginRotate(const std::vector<T>& lhs,
                                   std::vector<T>* const rhs) {
    return InOrigin(lhs, rhs);
  }

  /**
   * @brief NewBase2OriginRotatePan
   *
   * @param lhs
   * @param rhs
   * @return true
   * @return false
   */
  bool NewBase2OriginRotatePan(const std::vector<T>& lhs,
                               std::vector<T>* const rhs) {
    if (!NewBase2OriginRotate(lhs, rhs)) {
      return false;
    }
    rhs->at(0) = rhs->at(0) + pan_(0, 0);
    rhs->at(1) = rhs->at(1) + pan_(1, 0);
    return true;
  }

  /**
   * @brief NewBase2OriginRotate
   *
   * @param lhs
   * @param rhs
   * @return true
   * @return false
   */
  bool NewBase2OriginPanRotate(const std::vector<T>& lhs,
                               std::vector<T>* const rhs) {
    std::vector<T> lhs_t = lhs;
    lhs_t.at(0) = lhs.at(0) - pan_(0, 0);
    lhs_t.at(1) = lhs.at(1) - pan_(1, 0);
    return NewBase2OriginRotate(lhs_t, rhs);
  }

  /**
   * @brief Origin2NewBaseRotate
   *
   * @param lhs
   * @param rhs
   * @return true
   * @return false
   */
  bool Origin2NewBaseRotate(const std::vector<T>& lhs,
                            std::vector<T>* const rhs) {
    if (lhs.size() != 2) {
      return false;
    }

    input_(0, 0) = lhs.front();
    input_(1, 0) = lhs.back();
    output_ = rotate_vector_.inverse() * input_;
    rhs->clear();
    rhs->push_back(output_(0, 0));
    rhs->push_back(output_(1, 0));
    return true;
  }

  /**
   * @brief Origin2NewBaseRotatePan
   *
   * @param lhs
   * @param rhs
   * @return true
   * @return false
   */
  bool Origin2NewBaseRotatePan(const std::vector<T>& lhs,
                               std::vector<T>* const rhs) {
    if (!Origin2NewBaseRotate(lhs, rhs)) {
      return false;
    }
    rhs->at(0) = rhs->at(0) + pan_(0, 0);
    rhs->at(1) = rhs->at(1) + pan_(1, 0);
    return true;
  }

  /**
   * @brief Origin2NewBaseRotate
   *
   * @param lhs
   * @param rhs
   * @return true
   * @return false
   */
  bool Origin2NewBasePanRotate(const std::vector<T>& lhs,
                               std::vector<T>* const rhs) {
    std::vector<T> lhs_t = lhs;
    lhs_t.at(0) = lhs.at(0) - pan_(0, 0);
    lhs_t.at(1) = lhs.at(1) - pan_(1, 0);
    return Origin2NewBaseRotate(lhs_t, rhs);
  }

  /**
   * @brief Set the New Base Vector object
   *
   * @param newBaseVector
   */
  void SetNewBaseVector(const Eigen::Matrix<T, 2, 2>& newBaseVector) {
    rotate_vector_ = newBaseVector;
  }

  /**
   * @brief Set the New Base Vector object
   *
   * @param theta
   */
  inline void SetNewBaseVector(const double theta) {
    angle_ = theta;
    rotate_vector_(0, 0) = cos(theta);
    rotate_vector_(1, 0) = sin(theta);
    rotate_vector_(0, 1) = -sin(theta);
    rotate_vector_(1, 1) = cos(theta);
  }

  /**
   * @brief Get angle_
   *
   * @return double
   */
  inline double Angle() { return angle_; }

  /**
   * @brief Set the Pan object
   *
   * @param pan_vector
   */
  inline void SetPan(const std::vector<double>& pan_vector) {
    ACHECK(pan_vector.size() == 2) << "pan_vector size must be 2.";
    pan_(0, 0) = pan_vector.at(0);
    pan_(1, 0) = pan_vector.at(1);
  }

  /**
   * @brief Set the New Base Vector Pan object
   *
   * @param theta
   * @param pan_vector
   */
  inline void SetNewBaseVectorPan(const double theta,
                                  const std::vector<double>& pan_vector) {
    SetNewBaseVector(theta);
    SetPan(pan_vector);
  }

  void DebugClass() { AERROR << "\n" << rotate_vector_; }

  const Eigen::Matrix<T, 2, 2>& GetNewBaseVector() const {
    return rotate_vector_;
  }

 private:
  Eigen::Matrix<T, 2, 2> rotate_vector_;
  Eigen::Vector<T, 2> input_;
  Eigen::Vector<T, 2> output_;
  Eigen::Vector<T, 2> pan_;
  double angle_;
};
}  // namespace TL::planning::nolane

::nolane
