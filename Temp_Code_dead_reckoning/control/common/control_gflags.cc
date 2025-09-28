/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "control/common/control_gflags.h"

DEFINE_string(control_conf_file, "conf/control/control_conf.pb.txt",
              "default control conf data file");

DEFINE_string(control_common_conf_file,
              "conf/control/control_common_conf.pb.txt",
              "common control conf data file");

DEFINE_string(mpc_controller_conf_file,
              "conf/control/mpc_controller_conf.pb.txt",
              "mpc controller conf data file");

DEFINE_string(lateral_controller_conf_file,
              "conf/control/lateral_controller_conf.pb.txt",
              "lateral controller conf data file");

DEFINE_string(longitudinal_controller_conf_file,
              "conf/control/longitudinal_controller_conf.pb.txt",
              "longitudinal controller conf data file");

DEFINE_string(matrix_k_map_file, "conf/control/matrix_k_map.pb.txt",
              "matrix k map file");

DEFINE_string(calibration_table_file, "conf/control/calibration_table.pb.txt",
              "calibration table file");

DEFINE_string(mbd_ctrl_conf_file, "conf/control/mbd_control_conf.pb.txt",
              "default mbd control conf data file");

DEFINE_bool(enable_csv_debug, false, "True to write out csv debug file.");
DEFINE_bool(enable_speed_station_preview, true, "enable speed/station preview");

DEFINE_string(control_node_name, "control", "The control node name in proto");

DEFINE_string(mpc_controller_submodule_name, "MPCControllerSubmodule",
              "MPC controller node name in proto");

DEFINE_string(lat_lon_controller_submodule_name, "LatLonControllerSubmodule",
              "lateral+longitudinal controller node name in proto");

DEFINE_string(preprocessor_submodule_name, "PreprocessorSubmodule",
              "preprocessor submodule name in proto");

DEFINE_string(postprocessor_submodule_name, "PostprocessorSubmodule",
              "postprocessor submodule name in proto");

DEFINE_bool(is_control_test_mode, false, "True to run control in test mode");
DEFINE_bool(use_preview_speed_for_table, false,
            "True to use preview speed for table lookup");

DEFINE_double(steer_angle_rate, 100.0,
              "Steer angle change rate in percentage.");
DEFINE_bool(enable_gain_scheduler, false,
            "Enable gain scheduler for higher vehicle speed");
DEFINE_bool(set_steer_limit, false, "Set steer limit");

DEFINE_bool(enable_slope_offset, false, "Enable slope offset compensation");

DEFINE_double(lock_steer_speed, 0.081,
              "Minimum speed to lock the steer, in m/s");

DEFINE_bool(enable_navigation_mode_error_filter, false,
            "Enable error_filter for navigation mode");

DEFINE_bool(enable_navigation_mode_position_update, true,
            "Enable position update for navigation mode");

DEFINE_int32(chassis_pending_queue_size, 10, "Max chassis pending queue size");
DEFINE_int32(planning_pending_queue_size, 10,
             "Max planning pending queue size");
DEFINE_int32(localization_pending_queue_size, 10,
             "Max localization pending queue size");
DEFINE_int32(pad_msg_pending_queue_size, 10,
             "Max pad message pending queue size");

DEFINE_bool(reverse_heading_control, false, "test vehicle reverse control");

DEFINE_bool(
    trajectory_transform_to_com_reverse, false,
    "Enable planning trajectory coordinate transformation from center of "
    "rear-axis to center of mass, during reverse driving");
DEFINE_bool(
    trajectory_transform_to_com_drive, false,
    "Enable planning trajectory coordinate transformation from center of "
    "rear-axis to center of mass, during forward driving");

DEFINE_bool(enable_maximum_steer_rate_limit, false,
            "Enable steer rate limit obtained from vehicle_param.pb.txt");

DEFINE_bool(query_time_nearest_point_only, false,
            "only use the trajectory point at nearest time as target point");

DEFINE_bool(query_forward_time_point_only, false,
            "only use the trajectory point in future");

DEFINE_bool(enable_feedback_augment_on_high_speed, false,
            "Enable augmented control on lateral error on high speed");

DEFINE_bool(
    enable_gear_drive_negative_speed_protection, false,
    "Enable estop to prevent following negative speed during gear drive");

DEFINE_double(steer_offset, 2.0, "steer offset");

DEFINE_double(control_delay_compensation, 0.2,
              "Compensation time from perception to control");

DEFINE_double(steer_angle_limit, 500.0, "steer angle limit angle");
DEFINE_double(steer_angle_limit_first, 500.0, "steer angle limit angle");

DEFINE_int32(control_delay_time, 10,
             "(ms)delay control cmd in vehicle execute");

DEFINE_bool(enable_mpc_calibration, false, "enable mpc calibration");

DEFINE_bool(enable_pid_speedsegment_control, true,
            "enable pid speedsegment control");

DEFINE_bool(enable_steer_2_torque, false, "enable steer 2 torque");

// teleop
DEFINE_bool(enable_teleop, false, "enable teleop mode");
DEFINE_bool(enable_teleop_pid_calculate, false, "enable teleop pid calculate");
DEFINE_bool(enable_teleop_steer_torque, false, "enable teleop torque control");

DEFINE_double(throttle_inc_delta, 2.0,
              "throttle pedal command delta percentage.");
DEFINE_double(brake_inc_delta, 2.0, "brake pedal delta percentage");
DEFINE_double(steer_inc_delta, 2.0, "steer delta percentage");
// TODO(ALL) : switch the acceleration cmd or pedal cmd
// default : use pedal cmd
DEFINE_bool(
    use_acceleration, true,
    "switch to use acceleration instead of throttle pedal and brake pedal");
DEFINE_double(steer_2_torque_right, 1.0, "steer 2 torque right");
DEFINE_double(steer_2_torque_left, 1.0, "steer 2 torque left");
DEFINE_bool(enable_steer_segment_feedforward, false,
            "switch to use steer segment feedforward or main feedforward");
DEFINE_bool(enable_torque_gradient_protection, false,
            "enable torque gradient protection");
DEFINE_bool(enable_mbd_control, false, "enable mbd control");
DEFINE_bool(use_localization_timestamp_for_control, true,
            "use localization timestamp for control");
DEFINE_double(set_localization_control_time_gap, 0.02,
              "set localization control time gap");
