/*
 * Copyright (c) TL Technologies Co., Ltd. 2022. All rights reserved.
 * Description: scs_shape_path.cc
 */

#include "planning/open_space/coarse_path_generator/shape/scs_shape_path.h"
#include <array>
#include <iterator>
#include <utility>
#include "common/math/math_utils.h"
#include "common/math/vec2d.h"
#include "planning/common/open_space_info.h"
#include "planning/open_space/coarse_path_generator/utils/penalty_function_method.h"
#include "proto/common/pnc_point.pb.h"

namespace TL {
namespace planning {
using TL::common::math::Vec2d;

namespace {
constexpr double kEpsilon = 1.0e-3;
constexpr double kOnLineThrehold = 3e-2;
constexpr double kAngleSameThrehold = 1e-2;
constexpr double kLargeAngleThreshold = 0.9 * M_PI;
}  // namespace

bool SCSShapePath::GenerateSCSShapePath(
    const common::PathPoint& start_node, const common::PathPoint& end_node,
    const DestRegionWithAng& dest_region_with_angle, const double min_radius,
    const bool is_park_out, PathGeneratorResult* const path_result) {
  if (path_result == nullptr) {
    AERROR << "GenerateGeometryPath input check fails";
    return false;
  }

  path_result->reset();
  const double angle_diff_abs =
      fabs(common::math::AngleDiff(start_node.theta(), end_node.theta()));
  if (angle_diff_abs > kLargeAngleThreshold) {
    AERROR << "angle diff is too large, can not support now";
    return false;
  }
  const Vec2d start_end_vec(end_node.x() - start_node.x(),
                            end_node.y() - start_node.y());
  const double from_end_node_lat_distance =
      fabs(cos(start_node.theta()) * start_end_vec.y() -
           sin(start_node.theta()) * start_end_vec.x());
  const bool is_collinear = angle_diff_abs <= kAngleSameThrehold;
  if (is_collinear && from_end_node_lat_distance >
                          fmax(kOnLineThrehold,
                               start_end_vec.Length() * fabs(angle_diff_abs))) {
    AERROR << "from and to points have same angle,  but not on same line";
    return false;
  }
  common::PathPoint end_node_local = end_node;
  DestRegionWithAng dest_region_with_angle_local(dest_region_with_angle);
  int quadrant = 1;
  Trans2LocalCoor(start_node, &end_node_local, &dest_region_with_angle_local,
                  &quadrant);
  const auto& polygon = std::get<0>(dest_region_with_angle_local);
  if (polygon.num_points() > 2) {
    ADEBUG << "dest region \n" << polygon.DebugString();
  }
  ADEBUG << "end_node_local " << end_node_local.DebugString();
  ADEBUG << "quadrant " << quadrant;
  double radius = start_node.has_kappa() &&
                          !common::math::double_type::IsZero(start_node.kappa())
                      ? 1 / fabs(start_node.kappa())
                      : min_radius;
  if (!is_collinear && !GetSLSInfo(dest_region_with_angle_local, min_radius,
                                   is_park_out, &end_node_local, &radius)) {
    AERROR << "get sls info failed";
    return false;
  }

  if (!GenerateSLSPath(is_collinear, end_node_local, radius, path_result)) {
    AERROR << "GenerateSLSPath failed";
    return false;
  }
  // Transform to world space
  Vec2d point;
  Vec2d start_point(start_node.x(), start_node.y());
  for (int i = 0; i < path_result->x.size(); i++) {
    point.set_x(path_result->x.at(i));
    point.set_y(path_result->y.at(i));
    TransPointBasedOnQuadrantint(quadrant, &point);
    point.SelfRotate(start_node.theta());
    point += start_point;
    path_result->x.at(i) = point.x();
    path_result->y.at(i) = point.y();
    path_result->phi.at(i) =
        TransAngleBasedOnQuadrantint(quadrant, path_result->phi.at(i));
    path_result->phi.at(i) += start_node.theta();
    ADEBUG << "x " << path_result->x.at(i) << ", y " << path_result->y.at(i)
           << ", phi" << path_result->phi.at(i) << ",";
  }
  return true;
}

void SCSShapePath::Trans2LocalCoor(
    const common::PathPoint& origin, common::PathPoint* const end_node_ptr,
    DestRegionWithAng* const dest_region_with_angle_ptr, int* const quadrant) {
  if (nullptr == quadrant || nullptr == dest_region_with_angle_ptr ||
      nullptr == end_node_ptr) {
    return;
  }
  ADEBUG << "origin " << origin.DebugString();
  ADEBUG << "end_node_ptr " << end_node_ptr->DebugString();
  Vec2d relative_point(end_node_ptr->x() - origin.x(),
                       end_node_ptr->y() - origin.y());
  const Vec2d origin_point(origin.x(), origin.y());
  relative_point.SelfRotate(-origin.theta());
  ADEBUG << "relative_point " << relative_point.DebugString();
  double relative_angle =
      common::math::NormalizeAngle(end_node_ptr->theta() - origin.theta());
  *quadrant = 1;
  if (relative_point.x() > 0.0) {
    *quadrant = relative_point.y() > 0.0 ? 1 : 4;
  } else {
    *quadrant = relative_point.y() > 0.0 ? 2 : 3;
  }
  relative_angle = TransAngleBasedOnQuadrantint(*quadrant, relative_angle);
  TransPointBasedOnQuadrantint(*quadrant, &relative_point);
  end_node_ptr->set_x(relative_point.x());
  end_node_ptr->set_y(relative_point.y());
  end_node_ptr->set_theta(relative_angle);
  const auto& origin_polygon2d = std::get<0>(*dest_region_with_angle_ptr);
  if (origin_polygon2d.num_points() > 2) {
    auto points = origin_polygon2d.GetAllVertices();
    for (auto& point : points) {
      point -= origin_point;
      point.SelfRotate(-origin.theta());
      TransPointBasedOnQuadrantint(*quadrant, &point);
    }
    // TODO(jyw): angle should be transformed
    std::get<0>(*dest_region_with_angle_ptr) = common::math::Polygon2d(points);
  }
}

bool SCSShapePath::GetSLSInfo(const DestRegionWithAng& dest_region_with_angle,
                              const double min_radius, const bool is_park_out,
                              common::PathPoint* const target_pose_ptr,
                              double* const radius_ptr) {
  if (nullptr == target_pose_ptr || nullptr == radius_ptr) {
    return false;
  }
  if (*radius_ptr < 0) {
    AERROR << "sls check failed with radius_ptr " << *radius_ptr;
    return false;
  }
  if (target_pose_ptr->x() < 0 || target_pose_ptr->y() < 0 ||
      target_pose_ptr->theta() < 0) {
    AERROR << "sls check failed with x " << target_pose_ptr->x() << " y "
           << target_pose_ptr->y() << " theta " << target_pose_ptr->theta();
    return false;
  }
  const double sin_theta = sin(target_pose_ptr->theta());
  const double cos_theta = cos(target_pose_ptr->theta());
  const auto& polygon2d = std::get<0>(dest_region_with_angle);
  const bool is_polygon_valid = polygon2d.num_points() > 2;
  int edge_num =
      is_polygon_valid ? static_cast<int>(polygon2d.line_segments().size()) : 4;
  int constrain_dim = edge_num + 3;
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(constrain_dim, 3);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(constrain_dim);
  if (is_polygon_valid) {
    TransPolygon2HyperPlan(polygon2d, A.block(0, 0, edge_num, 3),
                           b.head(edge_num));
  } else {
    A(0, 0) = 1;
    b(0) = target_pose_ptr->x() + kEpsilon;
    A(1, 0) = -1;
    b(1) = -target_pose_ptr->x() + kEpsilon;
    A(2, 1) = 1;
    b(2) = target_pose_ptr->y() + kEpsilon;
    A(3, 1) = -1;
    b(3) = -target_pose_ptr->y() + kEpsilon;
  }
  // r_min constrain
  A.row(edge_num) << 0, 0, -1;
  b(edge_num) = -1 * min_radius;
  // -s1 <= 0
  A.row(edge_num + 1) << -1, cos_theta / sin_theta,
      sin_theta + cos_theta * (cos_theta - 1) / sin_theta;
  b(edge_num + 1) = 0.0;
  // -s2 <= 0
  A.row(edge_num + 2) << 0, -1 / sin_theta, (1 - cos_theta) / sin_theta;
  b(edge_num + 2) = 0.0;
  Eigen::Vector3d x_ref;
  Eigen::Vector3d x;
  x_ref << target_pose_ptr->x(), target_pose_ptr->y(), *radius_ptr;
  x << target_pose_ptr->x(), target_pose_ptr->y(), *radius_ptr;
  PenaltyFunctionMethod quadratic_penalty_function_method(x_ref);
  quadratic_penalty_function_method.AddAffineConstrains(A, b);
  quadratic_penalty_function_method.CalculaterKernal(is_park_out);
  if (!quadratic_penalty_function_method.Optimize(x)) {
    AERROR << "optimize failed";
    return false;
  }
  target_pose_ptr->set_x(x[0]);
  target_pose_ptr->set_y(x[1]);
  *radius_ptr = x[2];
  ADEBUG << "relative_point " << target_pose_ptr->DebugString();
  ADEBUG << "rad " << *radius_ptr;
  return true;
}

bool SCSShapePath::GenerateSLSPath(const bool is_collinear,
                                   const common::PathPoint& end_node,
                                   const double radius,
                                   PathGeneratorResult* const sls_path_ptr) {
  if (nullptr == sls_path_ptr) {
    AERROR << "sls_path_ptr is nullptr";
    return false;
  }
  double s1 = end_node.x();
  double s2 = 0.0;
  if (!is_collinear) {
    double sin_theta = sin(end_node.theta());
    double cos_theta = cos(end_node.theta());
    s2 = (end_node.y() - radius * (1 - cos_theta)) / sin_theta;
    s1 = end_node.x() - radius * sin_theta - s2 * cos_theta;
  }
  ADEBUG << "s1 " << s1 << " s2 " << s2 << " radius " << radius;
  if (is_collinear && s1 < kEpsilon && s2 < kEpsilon) {
    AERROR << "s1 and s2 is zero";
    return false;
  }
  double accum_s = 0.0;
  constexpr double kDefaultInterpolatedDeltaS = 0.1;
  while (accum_s < s1 + kDefaultInterpolatedDeltaS - kEpsilon) {
    accum_s = std::min(accum_s, s1);
    sls_path_ptr->x.emplace_back(accum_s);
    sls_path_ptr->y.emplace_back(0.0);
    sls_path_ptr->phi.emplace_back(0.0);
    accum_s += kDefaultInterpolatedDeltaS;
  }
  if (is_collinear) {
    if (!sls_path_ptr->phi.empty()) {
      sls_path_ptr->phi.back() = end_node.theta();
    }
    return true;
  }
  const double delta_angle = kDefaultInterpolatedDeltaS / fabs(radius);
  double accum_angle = delta_angle;
  Vec2d radius_center(s1, radius);
  while (accum_angle < end_node.theta() - kAngleSameThrehold) {
    sls_path_ptr->x.emplace_back(radius_center.x() + radius * sin(accum_angle));
    sls_path_ptr->y.emplace_back(radius_center.y() - radius * cos(accum_angle));
    sls_path_ptr->phi.emplace_back(accum_angle);
    accum_angle += delta_angle;
  }
  accum_s = 0.0;
  double x1 = radius_center.x() + radius * sin(end_node.theta());
  double y1 = radius_center.y() - radius * cos(end_node.theta());
  while (accum_s < s2 + kDefaultInterpolatedDeltaS - kEpsilon) {
    accum_s = std::min(accum_s, s2);
    sls_path_ptr->x.emplace_back(x1 + accum_s * cos(end_node.theta()));
    sls_path_ptr->y.emplace_back(y1 + accum_s * sin(end_node.theta()));
    sls_path_ptr->phi.emplace_back(end_node.theta());
    accum_s += kDefaultInterpolatedDeltaS;
  }
  return true;
}

}  // namespace planning
}  // namespace TL
