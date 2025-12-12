// Minimal compatibility stubs to replace Apollo/Magna utility calls and logging
#pragma once

#include <iostream>
#include <iomanip>

#include <ostream>

// Simple logging helpers that can be used like: AINFO << "msg" << var;
inline std::ostream& LogInfo() { return std::cout; }
inline std::ostream& LogDebug() { return std::cout; }
inline std::ostream& LogError() { return std::cerr; }

#define AINFO LogInfo()
#define ADEBUG LogDebug()
#define AERROR LogError()

// Minimal replacement for gflags DEFINE macros used in repository.
// They create simple variables named FLAGS_<name> with the provided default value.
#define DEFINE_double(name, default_val, desc) \
  double FLAGS_##name = (default_val)

#define DEFINE_int32(name, default_val, desc) \
  int FLAGS_##name = (default_val)

#define DEFINE_bool(name, default_val, desc) \
  bool FLAGS_##name = (default_val)

#define DEFINE_string(name, default_val, desc) \
  const char* FLAGS_##name = (default_val)

#define FIXED std::fixed
#define SETPRECISION(x) std::setprecision(x)

namespace Magna {
namespace common {
namespace util {
// Minimal stubs for utilities used by the code. These are intentionally
// no-ops or simple templates so the src files can be built without
// depending on the full Apollo/Magna stack.
inline void FillHeader(const char* /*name*/, void* /*msg*/) {}

template <typename A, typename B, typename C>
inline void TransformToMRF(const A& /*in*/, const B& /*q*/, C* /*out*/) {
  // no-op stub
}

template <typename A, typename B, typename C>
inline void TransformToVRF(const A& /*in*/, const B& /*q*/, C* /*out*/) {
  // no-op stub
}

}  // namespace util
}  // namespace common
}  // namespace Magna
