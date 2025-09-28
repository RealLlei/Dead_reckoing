/*
 * Copyright (c) TL Technologies Co., Ltd. 2022. All rights reserved.
 * Description:  al_ilqr_interface.h
 */

#pragma once

#include <memory>
#include <utility>
#include <vector>
#include "common/math/line_segment2d.h"
#include "common/math/math_utils.h"
#include "common/math/vec2d.h"
#include "planning/open_space/ilqr_smoother/al_ilqr.h"
#include "planning/open_space/ilqr_smoother/kinematic_model.h"

class ALILQR_INTERFACE {
 public:
  explicit ALILQR_INTERFACE(const Param& param)
      : param_(param), al_ilqr_(std::make_unique<ALILQR>(param)) {}

  /**
   * @brief 
   * 
   * @param x1 
   * @param y1 
   * @param theta1 
   * @param x2 
   * @param y2 
   * @param theta2 
   * @param has_no_shift 
   * @param is_lat_spot 
   * @param direction 
   * @param obstacles_segments_vec 
   */
  void Init(
      double x1, double y1, double theta1, double x2, double y2, double theta2,
      bool has_no_shift, bool is_lat_spot, int direction,
      const std::vector<std::pair<TL::common::math::LineSegment2d, double>>&
          obstacles_segments_vec);

  /**
   * @brief Get the Optimal object
   * 
   * @param x 
   * @param y 
   * @param theta 
   * @return true 
   * @return false 
   */
  bool GetOptimal(std::vector<double>* x, std::vector<double>* y,
                  std::vector<double>* theta);

 private:
  /**
   * @brief 
   * 
   * @param step_size 
   * @param x_op 
   * @param u_op 
   * @param x 
   * @param y 
   * @param theta 
   */
  static void Interpolate(double step_size,
                          const Eigen::Ref<const Eigen::MatrixXd>& x_op,
                          const Eigen::Ref<const Eigen::MatrixXd>& u_op,
                          std::vector<double>* x, std::vector<double>* y,
                          std::vector<double>* theta);

  /**
   * @brief 
   * 
   * @param obs_segs 
   * @param obs_vecs 
   */
  static void TransObsSegToVec(
      const std::vector<std::pair<TL::common::math::LineSegment2d, double>>&
          obs_segs,
      std::vector<std::pair<TL::common::math::Vec2d, double>>* obs_vecs);

  /**
   * @brief 
   * 
   * @param has_no_shift 
   * @param x1 
   * @param y1 
   * @param theta1 
   * @param x2 
   * @param y2 
   * @param theta2 
   * @param obstacles_segments_vec 
   */
  static std::pair<double, double> CalculatePoseRelax(
      bool has_no_shift, double x1, double y1, double theta1, double x2,
      double y2, double theta2,
      const std::vector<std::pair<TL::common::math::LineSegment2d, double>>&
          obstacles_segments_vec);

  Param param_;
  Eigen::MatrixXd xu_ref_;
  Eigen::MatrixXd xu_initial_;
  std::unique_ptr<ALILQR> al_ilqr_;
  std::vector<std::pair<TL::common::math::Vec2d, double>> obs_vec_;
  bool has_no_shift_ = false;
  bool is_lat_spot_ = false;
  int direction_ = -1;
  std::pair<double, double> pose_relax_;
};
