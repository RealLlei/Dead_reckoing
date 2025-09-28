load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])
CONTROL_COPTS = ['-DMODULE_NAME=\\"control\\"']

cc_library(
    name = "control_lib",
    srcs = [],
    copts = CONTROL_COPTS,
    deps = [
        "//cyber",
        "//cyber/time:clock",
        "//canbus/proto:chassis_cc_proto",
        "//common/adapters:adapter_gflags",
        "//common/latency_recorder",
        "//common/monitor_log",
        "//common/util",
        "//control/common",
        "//control/controller",
        "//control/proto:control_cmd_cc_proto",
        "//control/proto:control_conf_cc_proto",
        "//control/proto:pad_msg_cc_proto",
        "//control/proto:preprocessor_cc_proto",
        "//control/submodules:lat_lon_controller_submodule_lib",
        "//control/submodules:mpc_controller_submodule_lib",
        "//control/submodules:postprocessor_submodule_lib",
        "//control/submodules:preprocessor_submodule_lib",
        "//localization/proto:localization_cc_proto",
        "//planning/proto:planning_cc_proto",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "control_component_lib",
    srcs = ["control_component.cc"],
    hdrs = ["control_component.h"],
    copts = CONTROL_COPTS,
    deps = [
        ":control_lib",
        "//control/common:dependency_injector",
    ],
)

cc_binary(
    name = "libcontrol_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":control_component_lib"],
)

cc_test(
    name = "control_component_test",
    size = "small",
    srcs = ["control_component_test.cc"],
    data = ["//control:control_testdata"],
    deps = [
        ":control_component_lib",
        "@com_google_googletest//:gtest_main",
    ],
)

filegroup(
    name = "control_testdata",
    srcs = glob([
        "testdata/**",
    ]),
)

cpplint()
