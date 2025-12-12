// Minimal clock stub to replace Magna::common::Clock
#pragma once

#include <chrono>

namespace Magna {
namespace common {
struct Clock {
  // return seconds as double
  static double NowInSeconds() {
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
  }
};
} // namespace common
} // namespace Magna
