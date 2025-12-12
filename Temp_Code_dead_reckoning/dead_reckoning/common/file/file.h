// Minimal stub for common/file/file.h
#pragma once

#include <string>

namespace common {
namespace file {

inline bool ReadFileToString(const std::string& path, std::string* out) {
  (void)path;
  if (out) *out = std::string();
  return true;
}

}  // namespace file
}  // namespace common
