// Minimal message util stubs used by dead_reckoning
#pragma once

#include <string>
#include <Eigen/Dense>

namespace Magna {
namespace common {
namespace util {

struct Header { double timestamp = 0.0; };

inline void FillHeader(Header* h) { if (h) h->timestamp = 0.0; }

// declare transforms; implementations provided in util.h to avoid duplicate symbols
inline void TransformToMRF(const Eigen::Vector3d&, Eigen::Vector3d*);
inline void TransformToVRF(const Eigen::Vector3d&, Eigen::Vector3d*);

} // namespace util
} // namespace common
} // namespace Magna
