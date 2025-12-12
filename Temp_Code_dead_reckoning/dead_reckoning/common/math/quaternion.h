// Minimal quaternion helper stubs
#pragma once

#include <Eigen/Dense>
#include <cmath>

namespace Magna {
namespace common {
namespace math {

inline Eigen::Quaterniond HeadingToQuaternion(double heading) {
  // heading is yaw around Z
  return Eigen::Quaterniond(Eigen::AngleAxisd(heading, Eigen::Vector3d::UnitZ()));
}

inline double QuaternionToHeading(const Eigen::Quaterniond& q) {
  // extract yaw
  double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
  double cosy_cosp = 1.0 - 2.0 * (q.y()*q.y() + q.z()*q.z());
  return std::atan2(siny_cosp, cosy_cosp);
}

inline Eigen::Vector3d Quaternion2EulerZyx(const Eigen::Quaterniond& q) {
  // returns [yaw, pitch, roll] approximately
  double ysqr = q.y() * q.y();
  double t0 = +2.0 * (q.w() * q.x() + q.y() * q.z());
  double t1 = +1.0 - 2.0 * (q.x() * q.x() + ysqr);
  double roll = std::atan2(t0, t1);

  double t2 = +2.0 * (q.w() * q.y() - q.z() * q.x());
  t2 = t2 > 1.0 ? 1.0 : t2;
  t2 = t2 < -1.0 ? -1.0 : t2;
  double pitch = std::asin(t2);

  double t3 = +2.0 * (q.w() * q.z() + q.x() * q.y());
  double t4 = +1.0 - 2.0 * (ysqr + q.z() * q.z());
  double yaw = std::atan2(t3, t4);
  return Eigen::Vector3d(roll, pitch, yaw);
}

} // namespace math
} // namespace common
} // namespace Magna
