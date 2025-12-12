// Minimal gflags replacement: registry + simple parser for DEFINE_/DECLARE_
#pragma once

#include <string>
#include <unordered_map>
#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <sstream>

namespace gflags_stub {

enum FlagType { FT_BOOL, FT_DOUBLE, FT_STRING };

struct FlagReg { void* ptr; FlagType type; };

inline std::unordered_map<std::string, FlagReg>& registry() {
  static std::unordered_map<std::string, FlagReg> m;
  return m;
}

inline void gflags_register(const std::string& name, void* ptr, FlagType t) {
  registry()[name] = {ptr, t};
}

inline void ParseCommandLineFlags(int* argc, char*** argv, bool remove_flags = true) {
  if (!argc || !argv) return;
  int write = 1; // keep argv[0]
  for (int i = 1; i < *argc; ++i) {
    char* s = (*argv)[i];
    if (!s) continue;
    if (std::strncmp(s, "--", 2) != 0) {
      (*argv)[write++] = s;
      continue;
    }
    std::string flag(s + 2);
    std::string name;
    std::string val;
    bool is_no = false;
    auto eq = flag.find('=');
    if (eq != std::string::npos) {
      name = flag.substr(0, eq);
      val = flag.substr(eq + 1);
    } else {
      if (flag.rfind("no-", 0) == 0) {
        name = flag.substr(3);
        is_no = true;
      } else {
        name = flag;
      }
    }
    auto it = registry().find(name);
    if (it == registry().end()) {
      // unknown flag: drop it (or keep?) -- drop to mimic produced flags being consumed
      continue;
    }
    FlagReg reg = it->second;
    if (reg.type == FT_BOOL) {
      bool v = false;
      if (eq != std::string::npos) {
        std::string low = val;
        std::transform(low.begin(), low.end(), low.begin(), ::tolower);
        v = (low == "1" || low == "true" || low == "yes");
      } else {
        v = !is_no;
      }
      *static_cast<bool*>(reg.ptr) = v;
    } else if (reg.type == FT_DOUBLE) {
      double dv = 0.0;
      if (eq != std::string::npos) dv = std::atof(val.c_str());
      *static_cast<double*>(reg.ptr) = dv;
    } else if (reg.type == FT_STRING) {
      if (eq != std::string::npos)
        *static_cast<std::string*>(reg.ptr) = val;
      else
        *static_cast<std::string*>(reg.ptr) = std::string();
    }
  }
  if (remove_flags) {
    (*argc) = write;
    (*argv)[write] = nullptr;
  }
}

inline void SetUsageMessage(const char* /*msg*/) {}
inline void HandleCommandLineHelpFlags() {}

} // namespace gflags_stub

// Minimal macros to emulate gflags interface
#define DECLARE_bool(name) extern bool FLAGS_##name
#define DECLARE_double(name) extern double FLAGS_##name
#define DECLARE_string(name) extern std::string FLAGS_##name

#define DEFINE_bool(name, val, desc) \
  bool FLAGS_##name = (val); \
  namespace { struct _gflags_reg_##name { _gflags_reg_##name() { gflags_stub::gflags_register(#name, &FLAGS_##name, gflags_stub::FT_BOOL); } } _gflags_reg_instance_##name; }

#define DEFINE_double(name, val, desc) \
  double FLAGS_##name = (val); \
  namespace { struct _gflags_reg_##name { _gflags_reg_##name() { gflags_stub::gflags_register(#name, &FLAGS_##name, gflags_stub::FT_DOUBLE); } } _gflags_reg_instance_##name; }

#define DEFINE_string(name, val, desc) \
  std::string FLAGS_##name = (val); \
  namespace { struct _gflags_reg_##name { _gflags_reg_##name() { gflags_stub::gflags_register(#name, &FLAGS_##name, gflags_stub::FT_STRING); } } _gflags_reg_instance_##name; }

// Provide ParseCommandLineFlags symbol in global namespace for compatibility
inline void ParseCommandLineFlags(int* argc, char*** argv, bool remove_flags = true) {
  gflags_stub::ParseCommandLineFlags(argc, argv, remove_flags);
}

