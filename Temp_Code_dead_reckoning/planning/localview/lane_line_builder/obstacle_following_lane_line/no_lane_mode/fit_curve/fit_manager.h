/******************************************************************************
 * Copyright (c) TL Technologies Co., Ltd. 2019-2022. All rights reserved.
 * Author: Lingpeng
 * Created time: 2022/06/29
 *****************************************************************************/

#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "planning/localview/lane_line_builder/obstacle_following_lane_line/common_util/util.h"
#include "planning/localview/lane_line_builder/obstacle_following_lane_line/fit_curve/fit_convoluted_helix.h"
#include "planning/localview/lane_line_builder/obstacle_following_lane_line/fit_curve/fit_curve.h"
#include "planning/localview/lane_line_builder/obstacle_following_lane_line/fit_curve/fit_osqp_curve.h"
#include "planning/localview/lane_line_builder/obstacle_following_lane_line/fit_curve/fit_polynomial_curve.h"

namespace TL {
namespace planning {
namespace nolane {

enum class FitType { PolyNomial = 0, Osqp, Convoluted_Helix };
enum class TransCoordSys { NoTranSystem = 0, Slope, Error };

class FitManager {
 public:
  explicit FitManager(FitType type, double vehicle_speed = 0, int order = 3);

  ~FitManager() = default;

  /**
   * @brief Fit the points
   *
   * @param raw_points
   * @return true
   * @return false
   */
  bool Fit(const std::vector<Vec2d>& raw_points) {
    if (!fit_curve_pimpl_) {
      return false;
    }
    fit_curve_pimpl_->Fit(raw_points);

    return true;
  }

  /**
   * @brief GeneratePointsByFormula
   *
   * @param points_fit
   * @return true
   * @return false
   */
  bool GeneratePointsByFormula(std::vector<Vec2d>* const points_fit) {
    if (!fit_curve_pimpl_) {
      return false;
    }
    return fit_curve_pimpl_->GeneratePointsByFormula(points_fit);
  }

  bool FitPiecewise(
      std::vector<std::pair<common::SLPoint, Vec2d>>* const points,
      std::vector<Vec2d>* const fit_points,
      const std::shared_ptr<ReferenceLine>& reference_line_ptr);

  /**
   * @brief CalibratedByTrueValue
   *
   * @param lane_line_hdmap
   */
  void CalibratedByTrueValue(
      const std::vector<std::pair<double, double>>& lane_line_hdmap) {
    fit_curve_pimpl_->CalibratedByTrueValue(lane_line_hdmap);
  }

  const std::vector<double>& GetParameter() const {
    return fit_curve_pimpl_->GetParameter();
  }

  /**
   * @brief GenerateSinglePoint in x
   *
   * @param x
   * @return double
   */
  double GenerateSinglePoint(double x) {
    return fit_curve_pimpl_->GenerateSinglePoint(x);
  }

  /**
   * @brief reset fit order
   * @param order
   * @return
   */
  void SetPolynomialOrder(int order) {
    fit_curve_pimpl_.reset(new FitPolynomialCurve(order));
  }

 private:
  /**
   * @brief
   * 1. get point_t which is the ratote point after coordinate_transform_
   * rotate.
   * 2. calculate the fit point in point_t.
   * 3. rerotate the fit point in coordinate_transform_ system.
   * @param origin_point_in
   * @param origin_point_out
   */
  void RestoreVector(const Vec2d& origin_point_in,
                     Vec2d* const origin_point_out);

  void RestoreVector(Vec2d* const point);

  void AdjustDistancePiecewise();

  void CoordinateFit(const std::vector<Vec2d>& points);

 private:
  CoordinateTransform<2, double> coordinate_transform_;
  std::unique_ptr<FitCurve> fit_curve_pimpl_;
  double distance_piecewise_;
  TransCoordSys coordinate_fit_;
};
}  // namespace nolane
}  // namespace planning
}  // namespace TL
