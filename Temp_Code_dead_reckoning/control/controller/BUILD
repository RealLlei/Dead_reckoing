load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])
CONTROL_COPTS = ['-DMODULE_NAME=\\"control\\"']

cc_library(
    name = "controller_interface",
    srcs = [],
    hdrs = ["controller.h"],
    copts = CONTROL_COPTS,
    deps = [
        "//control/common:dependency_injector",
    ],
)

cc_library(
    name = "lat_controller",
    srcs = ["lat_controller.cc"],
    hdrs = ["lat_controller.h"],
    copts = CONTROL_COPTS,
    deps = [
        ":controller_interface",
        "//cyber",
        "//cyber/time:clock",
        "//common/configs:config_gflags",
        "//common/configs:vehicle_config_helper",
        "//common/filters:digital_filter",
        "//common/filters:digital_filter_coefficients",
        "//common/filters:mean_filter",
        "//common/math:euler_angles_zxy",
        "//common/math:geometry",
        "//common/math:lqr",
        "//common/math:mpc_osqp",
        "//common/status",
        "//control/common:control_gflags",
        "//control/common:interpolation_1d",
        "//control/common:leadlag_controller",
        "//control/common:mrac_controller",
        "//control/common:trajectory_analyzer",
        "//control/proto:calibration_table_cc_proto",
        "//control/proto:control_cmd_cc_proto",
        "//control/proto:control_conf_cc_proto",
        "//control/proto:gain_scheduler_conf_cc_proto",
        "//control/proto:input_debug_cc_proto",
        "//control/proto:lat_controller_conf_cc_proto",
        "//control/proto:lon_controller_conf_cc_proto",
        "//control/proto:mpc_controller_conf_cc_proto",
        "//control/proto:mrac_conf_cc_proto",
        "//control/proto:pad_msg_cc_proto",
        "@com_google_absl//absl/strings",
        "@eigen",
    ],
)

cc_library(
    name = "lon_controller",
    srcs = ["lon_controller.cc"],
    hdrs = ["lon_controller.h"],
    copts = CONTROL_COPTS,
    deps = [
        ":controller_interface",
        "//cyber",
        "//common/configs:vehicle_config_helper",
        "//common/filters:digital_filter",
        "//common/filters:digital_filter_coefficients",
        "//common/status",
        "//control/common:control_gflags",
        "//control/common:interpolation_2d",
        "//control/common:leadlag_controller",
        "//control/common:pid_controller",
        "//control/common:trajectory_analyzer",
        "//control/proto:calibration_table_cc_proto",
        "//control/proto:control_cmd_cc_proto",
        "//control/proto:control_conf_cc_proto",
        "//control/proto:gain_scheduler_conf_cc_proto",
        "//control/proto:input_debug_cc_proto",
        "//control/proto:lat_controller_conf_cc_proto",
        "//control/proto:lon_controller_conf_cc_proto",
        "//control/proto:mpc_controller_conf_cc_proto",
        "//control/proto:mrac_conf_cc_proto",
        "//control/proto:pad_msg_cc_proto",
        "//localization/common:localization_common",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "mpc_controller",
    srcs = ["mpc_controller.cc"],
    hdrs = ["mpc_controller.h"],
    copts = CONTROL_COPTS,
    deps = [
        ":controller_interface",
        "//cyber",
        "//cyber/time:clock",
        "//common/configs:vehicle_config_helper",
        "//common/filters:digital_filter",
        "//common/filters:digital_filter_coefficients",
        "//common/filters:mean_filter",
        "//common/math:euler_angles_zxy",
        "//common/math:geometry",
        "//common/math:lqr",
        "//common/math:mpc_osqp",
        "//common/proto:geometry_cc_proto",
        "//common/status",
        "//control/common:control_gflags",
        "//control/common:interpolation_1d",
        "//control/common:interpolation_2d",
        "//control/common:trajectory_analyzer",
        "//control/proto:calibration_table_cc_proto",
        "//control/proto:control_cmd_cc_proto",
        "//control/proto:control_conf_cc_proto",
        "//control/proto:gain_scheduler_conf_cc_proto",
        "//control/proto:input_debug_cc_proto",
        "//control/proto:lat_controller_conf_cc_proto",
        "//control/proto:lon_controller_conf_cc_proto",
        "//control/proto:mpc_controller_conf_cc_proto",
        "//control/proto:mrac_conf_cc_proto",
        "//control/proto:pad_msg_cc_proto",
        "@com_google_absl//absl/strings",
        "@eigen",
    ],
)

cc_library(
    name = "controller",
    copts = CONTROL_COPTS,
    deps = [
        ":controller_agent",
        ":controller_interface",
        ":lat_controller",
        ":lon_controller",
        ":mpc_controller",
        "@eigen",
    ],
)

cc_library(
    name = "controller_agent",
    srcs = ["controller_agent.cc"],
    hdrs = ["controller_agent.h"],
    copts = CONTROL_COPTS,
    deps = [
        ":controller_interface",
        ":lat_controller",
        ":lon_controller",
        ":mpc_controller",
        "//cyber",
        "//cyber/time:clock",
        "//common/util:factory",
        "//control/common:dependency_injector",
        "//control/proto:control_cmd_cc_proto",
        "//planning/proto:planning_cc_proto",
        "@osqp",
    ],
)

cc_test(
    name = "lon_controller_test",
    size = "small",
    srcs = ["lon_controller_test.cc"],
    copts = ["-fno-access-control"],
    data = ["//control/testdata:control_testdata"],
    deps = [
        ":lon_controller",
        "//cyber",
        "//cyber/time:clock",
        "//common/util",
        "//common/vehicle_state:vehicle_state_provider",
        "//control/common:control_gflags",
        "//control/proto:control_conf_cc_proto",
        "//planning/proto:planning_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "lat_controller_test",
    size = "small",
    srcs = ["lat_controller_test.cc"],
    copts = ["-fno-access-control"],
    data = ["//control/testdata:control_testdata"],
    deps = [
        ":lat_controller",
        "//cyber",
        "//cyber/time:clock",
        "//common/util",
        "//common/vehicle_state:vehicle_state_provider",
        "//control/common:control_gflags",
        "//planning/proto:planning_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "mpc_controller_test",
    size = "small",
    srcs = ["mpc_controller_test.cc"],
    copts = ["-fno-access-control"],
    data = ["//control/testdata:control_testdata"],
    deps = [
        ":mpc_controller",
        "//cyber",
        "//cyber/time:clock",
        "//common/util",
        "//common/vehicle_state:vehicle_state_provider",
        "//control/common:control_gflags",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
