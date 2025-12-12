// Minimal compatibility shim for Magna::soc types used in dead_reckoning
#pragma once

#include <cstdint>
#include <Eigen/Core>

namespace Magna {
namespace soc {

struct WheelSpeed {
  enum Type { FORWARD = 0, BACKWARD = 1, UNKNOWN = 2 };
  double fl_speed = 0.0;
  double fr_speed = 0.0;
  double rl_speed = 0.0;
  double rr_speed = 0.0;
  int rl_dir = UNKNOWN;
  int rr_dir = UNKNOWN;
  int fl_dir = UNKNOWN;
  int fr_dir = UNKNOWN;
};

using Chassis = void; // placeholder; real Chassis provided by dead_reckoning_core.h alias

}  // namespace soc
}  // namespace Magna
