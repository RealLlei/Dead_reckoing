load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "safety_guard_planner",
    srcs = ["safety_guard_planner.cc"],
    hdrs = ["safety_guard_planner.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//cyber/common:log",
        "//modules/common/proto:pnc_point_cc_proto",
        "//modules/common/status",
        "//modules/common/util",
        "//modules/common/util:factory",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/planning/common:planning_common",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/planner",
        "//modules/planning/proto:planning_cc_proto",
        "//modules/planning/tasks:task",
        "@com_github_gflags_gflags//:gflags",
        "@eigen",
    ],
)

cpplint()
