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

#include "common/util/util.h"

#include <cmath>
#include <vector>

#include "common/math/math_utils.h"
#include "common/math/quaternion.h"

namespace TL {
namespace common {
namespace util {

PointENU operator+(const PointENU& enu, const math::Vec2d& xy) {
  PointENU point;
  point.set_x(enu.x() + xy.x());
  point.set_y(enu.y() + xy.y());
  point.set_z(enu.z());
  return point;
}

PathPoint GetWeightedAverageOfTwoPathPoints(const PathPoint& p1,
                                            const PathPoint& p2,
                                            const double w1, const double w2) {
  PathPoint p;
  p.set_x(p1.x() * w1 + p2.x() * w2);
  p.set_y(p1.y() * w1 + p2.y() * w2);
  p.set_z(p1.z() * w1 + p2.z() * w2);
  p.set_theta(p1.theta() * w1 + p2.theta() * w2);
  p.set_kappa(p1.kappa() * w1 + p2.kappa() * w2);
  p.set_dkappa(p1.dkappa() * w1 + p2.dkappa() * w2);
  p.set_ddkappa(p1.ddkappa() * w1 + p2.ddkappa() * w2);
  p.set_s(p1.s() * w1 + p2.s() * w2);
  return p;
}

void TransformToMRF(const TL::common::Point3D& point_vrf,
                    const TL::common::Quaternion& orientation,
                    TL::common::Point3D* point_mrf) {
  Eigen::Vector3d v_vrf(point_vrf.x(), point_vrf.y(), point_vrf.z());
  auto m_vrf = TL::common::math::QuaternionRotate(orientation, v_vrf);
  point_mrf->set_x(m_vrf.x());
  point_mrf->set_y(m_vrf.y());
  point_mrf->set_z(m_vrf.z());
}

void TransformToVRF(const TL::common::Point3D& point_mrf,
                    const TL::common::Quaternion& orientation,
                    TL::common::Point3D* point_vrf) {
  Eigen::Vector3d v_mrf(point_mrf.x(), point_mrf.y(), point_mrf.z());
  auto v_vrf =
      TL::common::math::InverseQuaternionRotate(orientation, v_mrf);
  point_vrf->set_x(v_vrf.x());
  point_vrf->set_y(v_vrf.y());
  point_vrf->set_z(v_vrf.z());
}

math::Vec2d ComputeCOMPosition(const double rear_to_com_distance,
                               const math::Vec2d& rear_point,
                               const common::Quaternion& orientation) {
  // set length as distance between rear wheel and center of mass.
  Eigen::Vector3d v;
  v << 0.0, rear_to_com_distance, 0.0;
  Eigen::Vector3d pos_vec(rear_point.x(), rear_point.y(), 0.0);
  // Initialize the COM position without rotation
  Eigen::Vector3d com_pos_3d = v + pos_vec;

  // If we have rotation information, take it into consideration.
  Eigen::Quaternion<double> quaternion(orientation.w(), orientation.x(),
                                       orientation.y(), orientation.z());
  // Update the COM position with rotation
  com_pos_3d = quaternion.toRotationMatrix() * v + pos_vec;
  math::Vec2d vec_2d(com_pos_3d[0], com_pos_3d[1]);
  return vec_2d;
}  // namespace common

bool DoubleTriangleConstructor(const double x, const double y,
                               const double heading,
                               const double triangle_height,
                               const double bottom_angle,
                               common::math::Polygon2d* const up_triangle,
                               common::math::Polygon2d* const down_triangle) {
  static constexpr double kEpsilon = 1e-5;
  if (up_triangle == nullptr || down_triangle == nullptr ||
      triangle_height < kEpsilon || bottom_angle < 0.0 ||
      bottom_angle > M_PI / 2.0) {
    return false;
  }

  const double edge_length =
      std::fabs(triangle_height / std::fmax(std::sin(bottom_angle), kEpsilon));

  // 计算上三角形点
  const common::math::Vec2d left_up_edge_unit =
      common::math::Vec2d::CreateUnitVec2d(heading + M_PI - bottom_angle);
  const common::math::Vec2d left_up_edge_vec = left_up_edge_unit * edge_length;
  const common::math::Vec2d left_up_point = {x + left_up_edge_vec.x(),
                                             y + left_up_edge_vec.y()};

  const common::math::Vec2d right_up_edge_unit =
      common::math::Vec2d::CreateUnitVec2d(heading + bottom_angle);
  const common::math::Vec2d right_up_edge_vec =
      right_up_edge_unit * edge_length;
  const common::math::Vec2d right_up_point = {x + right_up_edge_vec.x(),
                                              y + right_up_edge_vec.y()};

  // 计算下三角形角点
  const common::math::Vec2d left_down_edge_unit =
      common::math::Vec2d::CreateUnitVec2d(heading - M_PI + bottom_angle);
  const common::math::Vec2d left_down_edge_vec =
      left_down_edge_unit * edge_length;
  const common::math::Vec2d left_down_point = {x + left_down_edge_vec.x(),
                                               y + left_down_edge_vec.y()};

  const common::math::Vec2d right_down_edge_unit =
      common::math::Vec2d::CreateUnitVec2d(heading - bottom_angle);
  const common::math::Vec2d right_down_edge_vec =
      right_down_edge_unit * edge_length;
  const common::math::Vec2d right_down_point = {x + right_down_edge_vec.x(),
                                                y + right_down_edge_vec.y()};

  *up_triangle =
      common::math::Polygon2d({{x, y}, right_up_point, left_up_point});
  *down_triangle =
      common::math::Polygon2d({{x, y}, right_down_point, left_down_point});

  return true;
}

}  // namespace util
}  // namespace common
}  // namespace TL
