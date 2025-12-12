// Minimal stub for linear interpolation
#pragma once

namespace Magna {
namespace common {
namespace math {

inline double LinearInterpolate(double x0, double x1, double t) {
  return x0 + (x1 - x0) * t;
}

// compatibility function name used in core
inline double InterpolateUsingLinearApproximation(const double x0, const double y0,
                                                   const double x1, const double y1,
                                                   const double x) {
  if (x1 == x0) return y0;
  double t = (x - x0) / (x1 - x0);
  return y0 + t * (y1 - y0);
}

}  // namespace math
}  // namespace common
}  // namespace Magna
