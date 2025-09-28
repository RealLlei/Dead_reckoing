/******************************************************************************
 * Copyright
 *
*/
#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "common/math/box2d.h"
#include "proto/common/pnc_point.pb.h"

namespace TL {
namespace prediction {

static constexpr double kDisThreshold = 1e-2;
static constexpr double kAngThreshold = 0.02;

class DiscretizedPath : public std::vector<common::PathPoint> {
 public:
  DiscretizedPath() = default;

  explicit DiscretizedPath(std::vector<common::PathPoint> path_points);

  double Length() const;

  common::PathPoint Evaluate(double path_s) const;

  common::PathPoint EvaluateReverse(double path_s) const;

  /**
   * @brief x y to s l
   *
   * @param x
   * @param y
   * @param sl_point
   * @return true
   * @return false
   */
  bool XYToSL(double x, double y, common::SLPoint* sl_point) const;

  /**
   * @brief check is point in path
   * 
   * @param point 
   * @return int 
   */
  bool IsPointIn(const common::PathPoint& point,
                 double dis_threshold = kDisThreshold,
                 double ang_threshold = kAngThreshold) const;

  /**
   * @brief check is same point or not
   * 
   * @param p_a 
   * @param p_b 
   * @return true 
   * @return false 
   */
  static bool IsSamePoint(const common::PathPoint& p_a,
                          const common::PathPoint& p_b,
                          double dis_threshold = kDisThreshold,
                          double ang_threshold = kAngThreshold);

  bool GetProjection(const common::math::Vec2d& point, double* accumulate_s,
                     double* lateral, double* min_distance, int* index_min,
                     double radius1d, int index_center) const;

 protected:
  std::vector<common::PathPoint>::const_iterator QueryLowerBound(
      double path_s) const;
  std::vector<common::PathPoint>::const_iterator QueryUpperBound(
      double path_s) const;
  std::vector<double> accumulated_s_{};
  std::vector<common::math::LineSegment2d> segments_{};
};

}  // namespace prediction
}  // namespace TL
