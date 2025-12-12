// Minimal quaternion -> euler helpers in Magna::dead_reckoning namespace
#include <Eigen/Geometry>
#include "src/dead_reckoning_core.h"

namespace Magna {
namespace dead_reckoning {

Eigen::Vector3d Quaternion2EulerZyx(const Eigen::Quaterniond& q) {
  Eigen::Vector3d euler;
  euler(0) = atan2(2.0 * (q.y() * q.z() + q.w() * q.x()),
                   1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y()));
  euler(1) = asin(-2.0 * (q.x() * q.z() - q.w() * q.y()));
  euler(2) = atan2(2.0 * (q.x() * q.y() + q.w() * q.z()),
                   1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
  return euler;
}

Eigen::Vector3d Quaternion2EulerZxy(const Eigen::Quaterniond& q) {
  Eigen::Vector3d euler;
  euler(0) = asin(2.0 * (q.y() * q.z() + q.w() * q.x()));
  euler(1) = atan2(2.0 * (q.x() * q.z() - q.w() * q.y()),
                   1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y()));
  euler(2) = atan2(-2.0 * (q.x() * q.y() - q.w() * q.z()),
                   1.0 - 2.0 * (q.x() * q.x() + q.z() * q.z()));
  return euler;
}

}  // namespace dead_reckoning
}  // namespace Magna
