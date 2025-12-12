// Minimal stub for vehicle_state_provider to allow isolated compilation.
#ifndef COMMON_VEHICLE_STATE_VEHICLE_STATE_PROVIDER_H_
#define COMMON_VEHICLE_STATE_VEHICLE_STATE_PROVIDER_H_

#include <memory>
#include <string>

namespace Magna {
namespace common {
namespace vehicle_state {

// Minimal VehicleStateProvider stub. Real implementation lives in upstream.
class VehicleStateProvider {
 public:
  VehicleStateProvider() = default;
  ~VehicleStateProvider() = default;

  bool Init(const std::string& /*conf_path*/) { return true; }
  // Add any methods referenced by dead_reckoning_core.cc as no-ops or stubs
};

}  // namespace vehicle_state
}  // namespace common
}  // namespace Magna

#endif  // COMMON_VEHICLE_STATE_VEHICLE_STATE_PROVIDER_H_
