// Minimal math utilities stub required by dead_reckoning_core.cc
#pragma once

#include <cmath>

namespace Magna {
namespace common {
namespace math {

inline double NormalizeAngle(double a) {
  while (a > M_PI) a -= 2.0*M_PI;
  while (a <= -M_PI) a += 2.0*M_PI;
  return a;
}

inline double DegToRad(double d) { return d * M_PI / 180.0; }
inline double RadToDeg(double r) { return r * 180.0 / M_PI; }

} // namespace math
} // namespace common
} // namespace Magna
