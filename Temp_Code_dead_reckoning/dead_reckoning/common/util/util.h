// Minimal util.h stub
#pragma once

#include <Eigen/Dense>

namespace Magna {
namespace common {
namespace util {

inline void TransformToMRF(const Eigen::Vector3d& in, Eigen::Vector3d* out) {
  if (out) *out = in;
}
inline void TransformToVRF(const Eigen::Vector3d& in, Eigen::Vector3d* out) {
  if (out) *out = in;
}

} // namespace util
} // namespace common
} // namespace Magna
