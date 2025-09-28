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
// NOLINTBEGIN
#include "planning/common/planning_gflags.h"

#include <limits>

#include "gflags/gflags.h"
DEFINE_bool(planning_test_mode, false, "Enable planning test mode.");

DEFINE_int32(planning_loop_rate, 10, "Loop rate for planning node");

DEFINE_int32(history_max_record_num, 5,
             "the number of planning history frame to keep");
DEFINE_int32(max_frame_history_num, 1, "The maximum history frame number");

DEFINE_string(planning_default_config_file,
              "conf/planning/planning_config.pb.txt",
              "default planning config file");
DEFINE_bool(enable_st_valid_bound_check, false, "st regular bound valid check");

DEFINE_double(nolane_obs_x_threshold, 1,
              "buffer when calculate next position.");
DEFINE_double(nolane_obs_y_threshold, 0.3,
              "buffer when calculate next position.");
DEFINE_double(nolane_obs_ego_cost_kdistance, 1,
              "coefficient value in x error.");
DEFINE_double(nolane_obs_ego_cost_kv_theta, 1, "coefficient value in v error.");
DEFINE_double(nolane_obs_ego_cost_rho, 100, "coefficient value in rho.");
DEFINE_double(nolane_obs_ego_cost_dis, 1, "coefficient value in rho.");
DEFINE_double(nolane_obs_ego_cost_lateral, 2, "coefficient value in rho.");
DEFINE_double(nolane_obs_ego_cost_theta, 10, "coefficient value in rho.");
DEFINE_double(nolane_obs_ego_cost_speed, 1, "coefficient value in rho.");
DEFINE_double(nolane_trajectory_fit_radius, 10.0, "coefficient value in rho.");
DEFINE_double(nolane_trajectory_fit_factor, 3.6, "coefficient value in rho.");
DEFINE_double(nolane_obs_ego_cost_intersection_angle_diff, 1,
              "coefficient value in intersection diff angle.");
DEFINE_double(nolane_obs_ego_too_far_ignore_cost, 100.0,
              "ignore cost in calc cost.");
DEFINE_double(nolane_obs_ego_too_closed_ignore_cost, 1,
              "ignore cost in calc cost.");
DEFINE_bool(nolane_log_all, false, "log all the information in no lane.");
DEFINE_bool(nolane_log_none, true, "not log any information at all.");
DEFINE_double(nolane_trajectory_minimal_length, 30,
              "minimal trajectory distance in no-lane");
DEFINE_double(nolane_obstacle_min_distance, 30,
              "minimal trajectory distance in no-lane");
DEFINE_double(nolane_obs_min_velocity, 3, "minimal speed in no-lane");
DEFINE_double(nolane_ego_traj_max_l, 2, "minimal speed in no-lane");
DEFINE_double(nolane_min_s_before_ego_in_ref, 3, "minimal speed in no-lane");
DEFINE_double(nolane_ego_is_close_ref, 1.5, "minimal speed in no-lane");
DEFINE_double(nolane_obs_tracking_max_time, 0.8, "max time in obs tracking");
DEFINE_double(nolane_obs_change_lane_min_dis, 10, "max time in obs tracking");
DEFINE_double(nolane_obs_change_lane_ego_time, 2, "max time in obs tracking");
DEFINE_double(nolane_obs_max_lateral_offset_in_straight, 0.3,
              "max offset in straight line when obstacle drive.");
DEFINE_double(nolane_ego_follow_obs_theta, 0.1, "max time in obs tracking");
DEFINE_double(nolane_ego_follow_obs_rel_x, 2, "max time in obs tracking");
DEFINE_double(nolane_ego_follow_obs_rel_y, 0.6, "max time in obs tracking");

DEFINE_int32(
    least_size_in_reuse_speed_plan, 5,
    "trajectory points' size at least been reused in use prev speed plan");

DEFINE_double(
    slack_add_obs_buff_in_ref, 0.5,
    "buffer value of add obstacle st boundary value in reference line.");

// scenario related
DEFINE_string(scenario_bare_intersection_unprotected_config_file,
              "conf/planning/"
              "scenario/bare_intersection_unprotected_config.pb.txt",
              "The bare_intersection_unprotected scenario configuration file");
DEFINE_string(nnp_scenario_lane_follow_config_file,
              "conf/planning/"
              "scenario/nnp_lane_follow_config.pb.txt",
              "The nnp lane_follow scenario configuration file");
DEFINE_string(ncp_scenario_lane_follow_config_file,
              "conf/planning/"
              "scenario/ncp_lane_follow_config.pb.txt",
              "The ncp lane_follow scenario configuration file");
DEFINE_string(scenario_lane_follow_hybrid_config_file,
              "conf/planning/"
              "scenario/lane_follow_hybrid_config.pb.txt",
              "The lane_follow scenario configuration file for HYBRID");
DEFINE_string(scenario_learning_model_sample_config_file,
              "conf/planning/"
              "scenario/learning_model_sample_config.pb.txt",
              "learning_model_sample scenario config file");
DEFINE_string(scenario_narrow_street_u_turn_config_file,
              "conf/planning/"
              "scenario/narrow_street_u_turn_config.pb.txt",
              "narrow_street_u_turn scenario config file");
DEFINE_string(scenario_park_and_go_config_file,
              "conf/planning/"
              "scenario/park_and_go_config.pb.txt",
              "park_and_go scenario config file");
DEFINE_string(scenario_pull_over_config_file,
              "conf/planning/"
              "scenario/pull_over_config.pb.txt",
              "The pull_over scenario configuration file");
DEFINE_string(scenario_emergency_pull_over_config_file,
              "conf/planning/"
              "scenario/emergency_pull_over_config.pb.txt",
              "The emergency_pull_over scenario configuration file");
DEFINE_string(scenario_emergency_stop_config_file,
              "conf/planning/"
              "scenario/emergency_stop_config.pb.txt",
              "The emergency_stop scenario configuration file");
DEFINE_string(scenario_stop_sign_unprotected_config_file,
              "conf/planning/"
              "scenario/stop_sign_unprotected_config.pb.txt",
              "stop_sign_unprotected scenario configuration file");
DEFINE_string(scenario_traffic_light_protected_config_file,
              "conf/planning/"
              "scenario/traffic_light_protected_config.pb.txt",
              "traffic_light_protected scenario config file");
DEFINE_string(scenario_traffic_light_unprotected_left_turn_config_file,
              "conf/planning/"
              "scenario/traffic_light_unprotected_left_turn_config.pb.txt",
              "traffic_light_unprotected_left_turn scenario config file");
DEFINE_string(scenario_traffic_light_unprotected_right_turn_config_file,
              "conf/planning/"
              "scenario/traffic_light_unprotected_right_turn_config.pb.txt",
              "traffic_light_unprotected_right_turn scenario config file");
DEFINE_string(scenario_valet_parking_config_file,
              "conf/planning/"
              "scenario/valet_parking_config.pb.txt",
              "valet_parking scenario config file");
DEFINE_string(scenario_yield_sign_config_file,
              "conf/planning/"
              "scenario/yield_sign_config.pb.txt",
              "yield_sign scenario config file");

DEFINE_string(dynamic_buffer_calculate_config_file,
              "conf/planning/dynamic_buffer_calculate_config.pb.txt",
              "dynamic_buffer_calculate config file");
DEFINE_string(hmi_config_file, "conf/planning/hmi_config.pb.txt",
              "hmi config file");
DEFINE_string(default_trigger_config_file,
              "conf/planning/trigger_config.pb.txt",
              "default trigger config file");
DEFINE_string(trigger_config_file,
              "/opt/usr/col/planning/trigger_config.pb.txt",
              "trigger config file");
DEFINE_string(orin_trigger_config_file,
              "/opt/usr/col/planning/conf/trigger_config.pb.txt",
              "trigger config file");
DEFINE_string(metric_config_file, "conf/planning/metric_conf.pb.txt",
              "metric config file");
DEFINE_string(valet_parking_history_trace_config_file,
              "conf/planning/parking/parking_lot_out_array_0.pb.txt",
              "valet parking history trace config file");

DEFINE_bool(st_boundary_debug_info, false, "add st boundary debug");

DEFINE_bool(enable_scenario_bare_intersection, true,
            "enable bare_intersection scenarios in planning");

DEFINE_bool(enable_scenario_park_and_go, true,
            "enable park-and-go scenario in planning");

DEFINE_bool(enable_scenario_pull_over, false,
            "enable pull-over scenario in planning");

DEFINE_bool(enable_scenario_emergency_pull_over, true,
            "enable emergency-pull-over scenario in planning");

DEFINE_bool(enable_scenario_emergency_stop, true,
            "enable emergency-stop scenario in planning");

DEFINE_bool(enable_scenario_side_pass_multiple_parked_obstacles, true,
            "enable ADC to side-pass multiple parked obstacles without"
            "worrying if the obstacles are blocked by others.");

DEFINE_bool(enable_scenario_stop_sign, false,
            "enable stop_sign scenarios in planning");

DEFINE_bool(enable_scenario_traffic_light, true,
            "enable traffic_light scenarios in planning");

DEFINE_bool(enable_scenario_yield_sign, true,
            "enable yield_sign scenarios in planning");

DEFINE_bool(enable_force_pull_over_open_space_parking_test, false,
            "enable force_pull_over_open_space_parking_test");

DEFINE_string(traffic_rule_config_filename,
              "conf/planning/traffic_rule_config.pb.txt",
              "Traffic rule config filename");

DEFINE_string(
    reference_line_smoother_config_filename,
    "conf/planning/smoother/fdm_discrete_points_smoother_config.pb.txt",
    "The configuration file for qp_spline smoother");

DEFINE_string(
    openspace_path_smoother_config_filename,
    "conf/planning/smoother/"
    "openspace_path_discrete_points_smoother_config.pb.txt",
    "The openspace parking path configuration file for qp_spline smoother");

DEFINE_string(rtk_trajectory_filename, "modules/planning/data/garage.csv",
              "Loop rate for planning node");

DEFINE_uint64(rtk_trajectory_forward, 800,
              "The number of points to be included in RTK trajectory "
              "after the matched point");

DEFINE_double(rtk_trajectory_resolution, 0.01,
              "The time resolution of output trajectory for rtk planner.");

DEFINE_bool(enable_trajectory_stitcher, true, "enable stitching trajectory");

DEFINE_bool(prioritize_change_lane, false,
            "change lane strategy has higher priority, always use a valid "
            "change lane path if such path exists");
DEFINE_double(change_lane_min_length, 30.0,
              "meters. If the change lane target has longer length than this "
              "threshold, it can shortcut the default lane.");

DEFINE_double(replan_lateral_distance_threshold, 0.5,
              "The lateral distance threshold of replan");
DEFINE_double(replan_heading_threshold, 0.21,
              "The heading threshold of replan");

DEFINE_double(replan_longitudinal_distance_threshold, 2.5,
              "The longitudinal distance threshold of replan");
DEFINE_double(replan_longitudinal_time_threshold, 0.2,
              "The longitudinal time threshold of replan");
DEFINE_bool(replan_every_period, false, "replan every period");
DEFINE_double(longitudinal_replan_speed_threshold, 2.0,
              "longitudinal replan threshold");
DEFINE_double(valet_parking_replan_longitudinal_distance_threshold, 1.0,
              "The valet parking longitudinal distance threshold of replan");
DEFINE_double(open_space_replan_longitudinal_distance_threshold, 0.3,
              "open_sapce_replan_longitudinal_distance_threshold");
DEFINE_double(open_space_replan_velocity_threshold, 0.05,
              "open_space_replan_velocity_threshold");

// planning parameter for reference line
DEFINE_bool(enable_reference_line_stitching, true,
            "Enable stitching reference line, which can reducing computing "
            "time and improve stability");
DEFINE_bool(enable_reference_line_interval_segment, true,
            "Enable reference line constraint interval segment");
DEFINE_bool(enable_reference_line_cost_segment, true,
            "Enable reference line cost segment");
DEFINE_bool(enable_smooth_reference_line, true,
            "enable smooth the map reference line");
DEFINE_bool(enable_smooth_select_reference_line, false,
            "enable smooth the select reference line");
DEFINE_bool(enable_reference_line_provider_thread, true,
            "Enable reference line provider thread.");
DEFINE_bool(enable_reference_line_history, false,
            "Enable reference line history.");
DEFINE_double(default_reference_line_width, 4.0,
              "Default reference line width");
DEFINE_bool(enable_reference_line_backward_anchor_point_unsmooth, false,
            "Enable reference line backward unsmooth use anchor points.");

DEFINE_bool(
    enable_reference_line_backward_anchor_point_unsmooth_avp, true,
    "Enable reference line backward unsmooth use anchor points for avp.");

DEFINE_bool(enable_path_bound_debug, true,
            "Enable path bound debug information.");

DEFINE_bool(enable_reference_line_debug, false,
            "Enable reference line debug information.");

DEFINE_double(planning_upper_speed_limit, 31.3,
              "Maximum speed (m/s) in planning.");
DEFINE_double(follow_time_desired, 3,
              "desired_time_speed_control_when_follow_obs");
DEFINE_double(follow_tanh_factor, 8,
              "desired_time_speed_control_when_follow_obs");

DEFINE_double(path_boundary_min_length, 40.0, "path_boundary_min_length");

// planning trajectory output time density control
DEFINE_double(
    trajectory_time_min_interval, 0.01,
    "(seconds) Trajectory time interval when publish. The is the min value.");
DEFINE_double(
    trajectory_time_max_interval, 0.1,
    "(seconds) Trajectory time interval when publish. The is the max value.");
DEFINE_double(
    trajectory_time_high_density_period, 1.5,
    "(seconds) Keep high density in the next this amount of seconds. ");

DEFINE_bool(enable_trajectory_check, false,
            "Enable sanity check for planning trajectory.");

DEFINE_double(speed_lower_bound, -0.1, "The lowest speed allowed.");
DEFINE_double(speed_upper_bound, 40.0, "The highest speed allowed.");

DEFINE_double(longitudinal_acceleration_lower_bound, -6.0,
              "The lowest longitudinal acceleration allowed.");
DEFINE_double(longitudinal_acceleration_upper_bound, 4.0,
              "The highest longitudinal acceleration allowed.");
DEFINE_double(lateral_acceleration_bound, 4.0,
              "Bound of lateral acceleration; symmetric for left and right");

DEFINE_double(longitudinal_jerk_lower_bound, -4.0,
              "The lower bound of longitudinal jerk.");

DEFINE_double(longitudinal_jerk_upper_bound, 2.0,
              "The upper bound of longitudinal jerk.");

DEFINE_double(
    longitudinal_jerk_upper_bound_slack, 0.92,
    "the slack value of longitudinal jerk bound in case of osqp solve failed.");
DEFINE_double(
    longitudinal_jerk_lower_bound_slack, 0.92,
    "the slack value of longitudinal jerk bound in case of osqp solve failed.");
DEFINE_double(lateral_jerk_bound, 4.0,
              "Bound of lateral jerk; symmetric for left and right");
DEFINE_double(
    longitudinal_acce_upper_bound_slack, 0.98,
    "the slack value of longitudinal acce bound in case of osqp solve failed.");
DEFINE_double(
    longitudinal_acce_lower_bound_slack, 0.98,
    "the slack value of longitudinal acce bound in case of osqp solve failed.");
DEFINE_double(longitudinal_speed_buffer, 0.1,
              "the slack value of longitudinal speed buffer.");
DEFINE_double(longitudinal_dis_buffer, 0.2,
              "the slack value of longitudinal distance buffer.");

DEFINE_double(kappa_bound, 0.1979, "The bound for trajectory curvature");

// speed limit
DEFINE_double(delta_time_speed_limit, 0.02,
              "time interval when calc speed limit at max dece");

// ST Boundary
DEFINE_double(st_max_s, 100, "the maximum s of st boundary");
DEFINE_double(st_max_t, 8, "the maximum t of st boundary");

DEFINE_bool(debug_cost_hexagon_big_car, false,
            "open print flag when debug coat hexagon");
DEFINE_bool(enable_avoid_big_cart, false, "enable avoid the big cart");

DEFINE_double(speed_planning_delta_time, 0.1, "delta time in speed planning");

DEFINE_int32(speed_freeze_frame, 3, "speed plan use previous frames.");
DEFINE_bool(enable_cut_trajectory, false, "enable cut trajectory");
DEFINE_double(cut_vehicle_trajectory_time_length, 4.0,
              "cut vehicle trajectory length");
DEFINE_double(cut_bicycle_trajectory_time_length, 2.0,
              "cut bicycle trajectory length");

// Decision Parvalidt
DEFINE_bool(enable_nudge_slowdown, true,
            "True to slow down when nudge obstacles.");

DEFINE_double(static_obstacle_nudge_l_buffer, 0.3,
              "minimum l-distance to nudge a static obstacle (meters)");
DEFINE_double(nonstatic_obstacle_nudge_l_buffer, 0.4,
              "minimum l-distance to nudge a non-static obstacle (meters)");
DEFINE_double(lane_change_obstacle_nudge_l_buffer, 0.3,
              "minimum l-distance to nudge when changing lane (meters)");
DEFINE_double(lateral_ignore_buffer, 3.0,
              "If an obstacle's lateral distance is further away than this "
              "distance, ignore it");
DEFINE_double(max_stop_distance_obstacle, 10.0,
              "max stop distance from in-lane obstacle (meters)");
DEFINE_double(min_stop_distance_obstacle, 6.0,
              "min stop distance from in-lane obstacle (meters)");
DEFINE_double(follow_min_distance, 3.0,
              "min follow distance for vehicles/bicycles/moving objects");
DEFINE_double(follow_min_obs_lateral_distance, 2.5,
              "obstacle min lateral distance to follow");
DEFINE_double(yield_distance, 5.0,
              "min yield distance for vehicles/moving objects "
              "other than pedestrians/bicycles");
DEFINE_double(follow_time_buffer, 2.5,
              "time buffer in second to calculate the following distance.");
DEFINE_double(follow_min_time_sec, 2.0,
              "min follow time in st region before considering a valid follow,"
              " this is to differentiate a moving obstacle cross adc's"
              " current lane and move to a different direction");
DEFINE_double(signal_expire_time_sec, 5.0,
              "traffic light signal info read expire time in sec");
DEFINE_string(destination_obstacle_id, "DEST",
              "obstacle id for converting destination to an obstacle");
DEFINE_double(destination_check_distance, 5.0,
              "if the distance between destination and ADC is less than this,"
              " it is considered to reach destination");

DEFINE_double(virtual_stop_wall_length, 0.1,
              "virtual stop wall length (meters)");
DEFINE_double(virtual_stop_wall_height, 2.0,
              "virtual stop wall height (meters)");

DEFINE_bool(enable_lane_change_by_obstacle, false,
            "enable lane change by obstacle");
DEFINE_bool(enable_lane_change_by_turn_signal, false,
            "enable lane change by turn signal");

DEFINE_bool(enable_lane_change_by_audio_signal, false,
            "enable lane change by audio signal");

DEFINE_bool(enable_no_lane_change_when_no_auto_drive, true,
            "no lane change when drive mode is no auto drive");
DEFINE_bool(
    only_enable_efficient_lane_change, false,
    "only enable efficient lane change. when only_enable_efficient_lane_change "
    "is true, navigation result is ignored");
DEFINE_bool(enable_lane_change_test_by_pad_msg, false,
            "enable to test lane change by pad message");

DEFINE_double(reference_line_priority_cost, 200000.0,
              "when reference line is not drivable, set reference line "
              "priority cost 200000.0.");
DEFINE_string(reference_line_info_decider_config_file,
              "conf/planning/reference_line_info_decider_config.pb.txt",
              "reference_line_info_decider config file");

DEFINE_double(path_lane_change_distance_preview_time, 6.5,
              "path lane change distance preview time.");
DEFINE_double(
    straight_forward_line_cost, 10.0,
    "when reference line is not lane change, set reference line cost 10.0");

DEFINE_bool(lane_change_planning_error_send_hmi, false,
            "enable to send hmi lane change planning error");
// Path Deciders
DEFINE_bool(enable_skip_path_tasks, false,
            "skip all path tasks and use trimmed previous path");

DEFINE_bool(enable_original_lane_borrow_process, false,
            "enable lane borrow process");
DEFINE_bool(enable_big_car_dynamic_obstacle_process, true,
            "use big car dynamic obstacle process.");
DEFINE_bool(enable_dynamic_vehicle_total_consider_length, true,
            "enable dynamic vehicle total consider length whose sum of length "
            "and width is less than a certain data.");
DEFINE_double(obstacle_lat_buffer, 0.4,
              "obstacle lateral buffer (meters) for deciding path boundaries");
DEFINE_double(obstacle_lon_start_buffer, 3.0,
              "obstacle longitudinal start buffer (meters) for deciding "
              "path boundaries");
DEFINE_double(obstacle_lon_end_buffer, 2.0,
              "obstacle longitudinal end buffer (meters) for deciding "
              "path boundaries");
DEFINE_double(static_obstacle_speed_threshold, 0.5,
              "The speed threshold to decide whether an obstacle is static "
              "or not.");
DEFINE_double(lane_borrow_max_speed, 5.0,
              "The speed threshold for lane-borrow");
DEFINE_int32(long_term_blocking_obstacle_cycle_threshold, 3,
             "The cycle threshold for long-term blocking obstacle.");
DEFINE_bool(enable_path_integral_calculate, false,
            "enable path intergral path calculate");
DEFINE_bool(enable_path_bound_record_debug, false,
            "enable_path_bound_record_debug");
DEFINE_bool(enable_towing_line_debug, false, "enable_towing_line_debug");
DEFINE_bool(enable_judge_whether_valid_path, false,
            "enable_judge_whether_valid_path");
DEFINE_bool(enable_plot_speed_l_buffer, false, "enable_plot_speed_l_buffer");

// Prediction Part
DEFINE_double(prediction_total_time, 5.0, "Total prediction time");
DEFINE_bool(align_prediction_time, false,
            "enable align prediction data based planning time");

// Trajectory

// according to DMV's rule, turn signal should be on within 200 ft from
// intersection.
DEFINE_double(
    turn_signal_distance, 100.00,
    "In meters. If there is a turn within this distance, use turn signal");

DEFINE_double(adc_l_buffer_for_static_obstacle_st_boundary, 0.2,
              "In meters. path decider and path st boundary use it to check "
              "path overlap with obstacle box.");
DEFINE_double(
    adc_l_buffer_for_static_obstacle_avp_mode, 0.4,
    "In meters, path decider use it to check path overlap with obstacle box.");

DEFINE_int32(trajectory_point_num_for_debug, 10,
             "number of output trajectory points for debugging");

DEFINE_double(lane_change_prepare_length, 80.0,
              "The distance of lane-change preparation on current lane.");

DEFINE_double(min_lane_change_prepare_length, 10.0,
              "The minimal distance needed of lane-change on current lane.");

DEFINE_double(allowed_lane_change_failure_time, 2.0,
              "The time allowed for lane-change failure before updating"
              "preparation distance.");

DEFINE_bool(enable_smarter_lane_change, false,
            "enable smarter lane change with longer preparation distance.");

// QpSt optimizer
DEFINE_double(slowdown_profile_deceleration, -4.0,
              "The deceleration to generate slowdown profile. unit: m/s^2.");
DEFINE_double(delta_acceleration, 0.1,
              "delta acceleration in each st bound test.");
DEFINE_double(max_deceleration_backup, -4.0,
              "max acceleration in st bound test.");

// SQP solver
DEFINE_bool(enable_sqp_solver, true, "True to enable SQP solver.");

// piecewise jeck path problem set ddx and dddx weight (v*low_spd_weight +
// min_longitudinal_spd)
DEFINE_double(min_longitudinal_spd, 3.0, "min_longitudinal_spd");
DEFINE_double(low_spd_weight, 0.85, "low_spd_weight");
DEFINE_double(max_lateral_acc, 10.0, "max lateral acc");
DEFINE_double(max_lateral_jerk, 100.0, "max lateral jerk");
DEFINE_double(path_bound_relaxation_param, 0.1, "path_bound_relaxation_param");
/// thread pool

DEFINE_uint32(load_nnp_ncp_map_interval, 800, "time interval of load nnp map");
DEFINE_uint32(load_perception_map_interval, 100,
              "time interval of load perception map");

DEFINE_bool(use_multi_thread_to_add_obstacles, true,
            "use multiple thread to add obstacles.");
DEFINE_bool(enable_multi_thread_in_dp_st_graph, false,
            "Enable multiple thread to calculation curve cost in dp_st_graph.");
DEFINE_bool(use_multi_thread_in_reference_line, false,
            "use multiple thread to reference line smooth.");
DEFINE_bool(use_multi_thread_obs_build_stbound, true,
            "use multiple thread to obstacle build st-boundaries.");
DEFINE_bool(use_multi_thread_path_optimaize, true,
            "use multiple thread to path optimization.");

/// Lattice Planner
DEFINE_double(numerical_epsilon, 1e-6, "Epsilon in lattice planner.");
DEFINE_double(default_cruise_speed, 5.0, "default cruise speed");
DEFINE_double(trajectory_time_resolution, 0.1,
              "Trajectory time resolution in planning");
DEFINE_double(trajectory_space_resolution, 1.0,
              "Trajectory space resolution in planning");
DEFINE_int32(min_path_data_point_count, 100, "min point count in path data");
DEFINE_double(speed_lon_decision_horizon, 500.0,
              "Longitudinal horizon for speed decision making (meter)");
DEFINE_uint64(num_velocity_sample, 6,
              "The number of velocity samples in end condition sampler.");
DEFINE_bool(enable_backup_trajectory, true,
            "If generate backup trajectory when planning fail");
DEFINE_double(backup_trajectory_cost, 1000.0,
              "Default cost of backup trajectory");
DEFINE_double(min_velocity_sample_gap, 1.0,
              "Minimal sampling gap for velocity");
DEFINE_double(lon_collision_buffer, 2.0,
              "The longitudinal buffer to keep distance to other vehicles");
DEFINE_double(lat_collision_buffer, 0.1,
              "The lateral buffer to keep distance to other vehicles");
DEFINE_uint64(num_sample_follow_per_timestamp, 3,
              "The number of sample points for each timestamp to follow");

// Lattice Evaluate Parameters
DEFINE_double(weight_lon_objective, 10.0, "Weight of longitudinal travel cost");
DEFINE_double(weight_lon_jerk, 1.0, "Weight of longitudinal jerk cost");
DEFINE_double(weight_lon_collision, 5.0,
              "Weight of longitudinal collision cost");
DEFINE_double(weight_lat_offset, 2.0, "Weight of lateral offset cost");
DEFINE_double(weight_lat_comfort, 10.0, "Weight of lateral comfort cost");
DEFINE_double(weight_centripetal_acceleration, 1.5,
              "Weight of centripetal acceleration");
DEFINE_double(cost_non_priority_reference_line, 5.0,
              "The cost of planning on non-priority reference line.");
DEFINE_double(weight_same_side_offset, 1.0,
              "Weight of same side lateral offset cost");
DEFINE_double(weight_opposite_side_offset, 10.0,
              "Weight of opposite side lateral offset cost");
DEFINE_double(weight_dist_travelled, 10.0, "Weight of travelled distance cost");
DEFINE_double(weight_target_speed, 1.0, "Weight of target speed cost");
DEFINE_double(lat_offset_bound, 3.0, "The bound of lateral offset");
DEFINE_double(lon_collision_yield_buffer, 1.0,
              "Longitudinal collision buffer for yield");
DEFINE_double(lon_collision_overtake_buffer, 5.0,
              "Longitudinal collision buffer for overtake");
DEFINE_double(lon_collision_cost_std, 0.5,
              "The standard deviation of longitudinal collision cost function");
DEFINE_double(default_lon_buffer, 5.0,
              "Default longitudinal buffer to sample path-time points.");
DEFINE_double(time_min_density, 1.0,
              "Minimal time density to search sample points.");
DEFINE_double(comfort_acceleration_factor, 0.5,
              "Factor for comfort acceleration.");
DEFINE_double(polynomial_minimal_param, 0.01,
              "Minimal time parameter in polynomials.");
DEFINE_double(lattice_stop_buffer, 0.02,
              "The buffer before the stop s to check trajectories.");

DEFINE_bool(lateral_optimization, true,
            "whether using optimization for lateral trajectory generation");
DEFINE_double(weight_lateral_offset, 1.0,
              "weight for lateral offset "
              "in lateral trajectory optimization");
DEFINE_double(weight_lateral_derivative, 500.0,
              "weight for lateral derivative "
              "in lateral trajectory optimization");
DEFINE_double(weight_lateral_second_order_derivative, 1000.0,
              "weight for lateral second order derivative "
              "in lateral trajectory optimization");
DEFINE_double(weight_lateral_third_order_derivative, 1000.0,
              "weight for lateral third order derivative "
              "in lateral trajectory optimization");
DEFINE_double(
    weight_lateral_obstacle_distance, 0.0,
    "weight for lateral obstacle distance in lateral trajectory optimization");
DEFINE_double(lateral_third_order_derivative_max, 0.1,
              "the maximal allowance for lateral third order derivative");
DEFINE_double(lateral_derivative_bound_default, 2.0,
              "the default value for lateral derivative bound.");
DEFINE_double(max_s_lateral_optimization, 60.0,
              "The maximal s for lateral optimization.");
DEFINE_double(default_delta_s_lateral_optimization, 1.0,
              "The default delta s for lateral optimization.");
DEFINE_double(bound_buffer, 0.1, "buffer to boundary for lateral optimization");
DEFINE_double(nudge_buffer, 0.3, "buffer to nudge for lateral optimization");

DEFINE_double(fallback_total_time, 3.0, "total fallback trajectory time");
DEFINE_double(fallback_time_unit, 0.1,
              "fallback trajectory unit time in seconds");

DEFINE_double(speed_bump_speed_limit, 4.4704,
              "the speed limit when passing a speed bump, m/s. The default "
              "speed limit is 10 mph.");
DEFINE_double(default_city_road_speed_limit, 15.67,
              "default speed limit (m/s) for city road. 35 mph.");
DEFINE_double(default_highway_speed_limit, 29.06,
              "default speed limit (m/s) for highway. 65 mph.");

// navigation mode
DEFINE_bool(enable_planning_pad_msg, false,
            "To control whether to enable planning pad message.");
DEFINE_bool(enable_turn_signal_before_lane_change, true,
            "enable turn on the turn signal before auto lane change.");
// TODO(all): open space planner, merge with planning conf
DEFINE_string(planner_open_space_config_filename,
              "conf/planning/planner_open_space_config.pb.txt",
              "The open space planner configuration file");

DEFINE_bool(use_dual_variable_warm_start, true,
            "whether or not enable dual variable warm start ");

DEFINE_bool(use_s_curve_speed_smooth, false,
            "Whether use s-curve (piecewise_jerk) for smoothing Hybrid Astar "
            "speed/acceleration.");

DEFINE_bool(
    enable_parallel_path_smoothing, false,
    "Whether to partition the trajectory first and do smoothing in parallel");

DEFINE_bool(enable_osqp_debug, false,
            "True to turn on OSQP verbose debug output in log.");

DEFINE_bool(export_chart, false, "export chart in planning");
DEFINE_bool(enable_record_debug, true,
            "True to enable record debug info in chart format");
DEFINE_bool(enable_record_openspace_debug, true,
            "True to enable record openspace debug info in chart format");
DEFINE_bool(enable_vt_sample_debug, false,
            "True to enable vt_sample record debug info in file");

DEFINE_bool(enable_one_shoot_log, false, "enable_one_shoot_log");
DEFINE_string(one_shoot_log_root_dir, "/data/log", "one_shoot_log_root_dir");

DEFINE_double(
    default_front_clear_distance, 300.0,
    "default front clear distance value in case there is no obstacle around.");

DEFINE_double(max_trajectory_len, 1000.0,
              "(unit: meter) max possible trajectory length.");
DEFINE_bool(enable_rss_fallback, false, "trigger rss fallback");
DEFINE_bool(enable_rss_info, true, "enable rss_info in trajectory_pb");
DEFINE_double(rss_max_front_obstacle_distance, 3000.0,
              "(unit: meter) for max front obstacle distance.");

DEFINE_bool(
    enable_planning_smoother, false,
    "True to enable planning smoother among different planning cycles.");
DEFINE_double(smoother_stop_distance, 10.0,
              "(unit: meter) for ADC stop, if it is close to the stop point "
              "within this threshold, current planning will be smoothed.");

DEFINE_bool(enable_parallel_hybrid_a, false,
            "True to enable hybrid a* parallel implementation.");

DEFINE_double(open_space_standstill_acceleration, 0.0,
              "(unit: meter/sec^2) for open space stand still at destination");
DEFINE_double(pause_brake_acceleration, 0.0,
              "(unit: meter/sec^2) for open space pause condition");

DEFINE_bool(enable_dp_reference_speed, true,
            "True to penalize dp result towards default cruise speed");
DEFINE_double(speed_threshold_big_car, 15.0, "speed threshold for big car");
DEFINE_double(dp_acceleration_buffer, 1.0, "acceleration buffer in dp search");
DEFINE_double(length_threshold_big_car, 10.0,
              "length threshold for big car (length + width)");
DEFINE_double(area_threshold_big_car, 15.0, "area threshold for big car");
DEFINE_double(init_point_lmax_threshold_big_car, 6.5,
              "lmax for big car in initial point");
DEFINE_double(path_point_lmin_threshold_big_car, 1.5,
              "lmin for big car in all point");
DEFINE_double(path_point_lmax_threshold_big_car, 7.0,
              "lmin for big car in all point");

DEFINE_double(message_latency_threshold, 0.3, "Threshold for message delay");
DEFINE_bool(enable_lane_change_urgency_checking, false,
            "True to check the urgency of lane changing");
DEFINE_bool(enable_avoid_stop_to_lane_change, false,
            "True to avoid to stop to change lane.");
DEFINE_double(
    min_speed_for_lane_change, 16.6666,
    "The minimum speed of the car is reduced to 60k/h when changing lane.");
DEFINE_double(short_path_length_threshold, 15.0,
              "Threshold for too short path length");
DEFINE_double(constraint_length_from_start_point, 20.0,
              "constraint path bound length by planning init point dl.");
DEFINE_uint64(trajectory_stitching_preserved_length, 20,
              "preserved points number in trajectory stitching");

DEFINE_double(trajectory_stitching_time, 0.20,
              "relative time in trajectory stitching");

DEFINE_double(side_pass_driving_width_l_buffer, 0.1,
              "(unit: meter) for side pass driving width l buffer");

DEFINE_bool(use_st_drivable_boundary, false,
            "True to use st_drivable boundary in speed planning");

DEFINE_bool(enable_reuse_path_in_lane_follow, false,
            "True to enable reuse path in lane follow");
DEFINE_bool(
    use_smoothed_dp_guide_line, false,
    "True to penalize speed optimization result to be close to dp guide line");

DEFINE_bool(use_soft_bound_in_nonlinear_speed_opt, true,
            "False to disallow soft bound in nonlinear speed opt");

DEFINE_bool(use_front_axe_center_in_path_planning, false,
            "If using front axe center in path planning, the path can be "
            "more agile.");

DEFINE_bool(use_road_boundary_from_map, false, "get road boundary from HD map");

DEFINE_uint32(apa_gear_shift_limit, 7, "get road boundary from HD map");

DEFINE_bool(planning_offline_learning, false,
            "offline learning. read record files and dump learning_data");
DEFINE_string(planning_data_dir, "data/",
              "Prefix of files to store feature data");
DEFINE_string(planning_offline_bags, "",
              "a list of source files or directories for offline mode. "
              "The items need to be separated by colon ':'. ");
DEFINE_int32(learning_data_obstacle_history_time_sec, 3.0,
             "time sec (second) of history trajectory points for a obstacle");
DEFINE_int32(learning_data_frame_num_per_file, 100,
             "number of learning_data_frame to write out in one data file.");
DEFINE_string(planning_birdview_img_feature_renderer_config_file,
              "conf/planning/planning_semantic_map_config.pb.txt",
              "config file for renderer singleton");

DEFINE_bool(
    skip_path_reference_in_side_pass, false,
    "skipping using learning model output as path reference in side pass");
DEFINE_bool(
    skip_path_reference_in_change_lane, true,
    "skipping using learning model output as path reference in change lane");

DEFINE_int32(min_past_history_points_len, 0,
             "minimun past history points length for trainsition from "
             "rule-based planning to learning-based planning");

// navigation_hdmap
DEFINE_string(navigation_hdmap_config_filename,
              "conf/planning/navigation_hdmap_config.pb.txt",
              "Relative map configuration file");

DEFINE_int32(navigation_hdmap_loop_rate, 10,
             "Loop rate for navigation_hdmap node");

DEFINE_bool(navigation_hdmap_generate_left_boundray, true,
            "Generate left boundary for detected lanes.");

DEFINE_bool(navigation_hdmap_no_lane_marker_c3, false,
            "Not used c3 of lane marker.");
DEFINE_bool(navigation_hdmap_no_lane_marker_c2, false,
            "Not used c2 of lane marker.");
DEFINE_bool(using_filter_of_c2, false, "using_filter_of_c2 in lane marker.");
DEFINE_bool(using_filter_of_c3, false, "using_filter_of_c3 in lane marker.");
DEFINE_double(buffer_gainst_lookford_distance, 20.0,
              "buffer against lookford distance when creating central line.");
DEFINE_double(common_center_line_resolution, 1.0,
              " the resolution of central line.");
DEFINE_int32(right_navi_path_priority, 12,
             "the Priority of right navigation path for mobileye "
             "navigation_hdmap node");
DEFINE_int32(
    left_navi_path_priority, 11,
    "the Priority of right navigation path for mobileye navigation_hdmap node");
DEFINE_int32(current_navi_path_priority, 0,
             "the Priority of current navigation path for mobileye "
             "navigation_hdmap node");
DEFINE_double(has_no_lanemarker_time, 3,
              "if not received mobileye's lanemarker more than a certain "
              "period of time will return false");
DEFINE_bool(using_original_lanemarker_boundary, true,
            "using the boundary path from original lanemarker equation.");
DEFINE_bool(generate_three_centralines_alltime, false,
            "generate three centerlines.");
DEFINE_double(central_lane_min_width, 3.0, " the central lane min width.");
DEFINE_double(central_lane_min_length_after_cut, 5.0,
              " the central lane min length after cut.");
DEFINE_double(lane_max_back_length, 10.0, " the lane max back length.");
DEFINE_bool(enable_online_hdmap_generator, false,
            "enable online map generator");
DEFINE_int32(online_hdmap_geberatepath_num, 3,
             "1,only central path;2,central and left path;3,central left right "
             "path;4,central and right path");
DEFINE_double(online_hdmap_geberatepath_lane_width, 3.8,
              "online_hdmap_geberatepath_lane_width");
DEFINE_double(adaptive_cruise_lane_width, 3.5, "adaptive_cruise_lane_width");
DEFINE_double(steering_wheel_reduction_factor, 0.8,
              "steering_wheel_reduction_factor");
DEFINE_bool(enable_esitimate_position_when_stitching, false,
            " esitimate position when stitching");
DEFINE_bool(enable_prediction_output, false, "enable prediction");
DEFINE_bool(enable_skip_check_input, false, "enable enable_skip_check_input");

DEFINE_double(yield_time_buffer, 3, "yield time buffer");

DEFINE_bool(enable_planning_warning, true, "enable_planning_warning");
/// AVP
DEFINE_double(park_out_distance, 6.0,
              "s_diff between start point and end point");
DEFINE_bool(lgsvl_sim_switch, true, "switch sim : false off ; true on ");
DEFINE_double(parkingout_roi_length, 15.0, "parking out scene ROI length");

DEFINE_double(direct_move_roi_length, 10,
              "direct_move scene: direct_move_roi_forward_or_backward");
DEFINE_double(direct_move_roi_width, 3,
              "direct_move_scene: direct_move_roi_lateral");
DEFINE_double(direct_move_length, 5,
              "direct_move_scene: direct_move_planning_length");
DEFINE_double(nns_stuck_roi_min_x_margin, -2, "nns roi mininum x margin");
DEFINE_double(nns_stuck_roi_max_x_margin, 2, "nns roi maximum x margin");
DEFINE_double(nns_stuck_roi_min_y_margin, -2, "nns roi minium y margin");
DEFINE_double(nns_stuck_roi_max_y_margin, 2, "nns roi maxmimum y margin");
DEFINE_double(default_cruise_low_speed, 5,
              "parking_cruise_scene: default cruise speed");
DEFINE_double(park_out_early_stop_angle_limit, 0.2618,
              "park_out_early_stop_angle_limit: early stop parking out to "
              "avolid execute tough path");
DEFINE_int32(lgsvl_sim_parking_direction, 1,
             "parking type and parking direction: 1: verticle "
             "left;2: verticle "
             "right;3: level left;4: level right");
DEFINE_double(open_space_lane_width, 5, "open_space_lane_width");

DEFINE_bool(forbit_replan_switch, true,
            "start replan distance replan openspace");

DEFINE_int32(perception_loss_count, 6, "perction loss parking slot count");

DEFINE_int32(publish_trajectory_points_number, 60,
             "control need path point number");
DEFINE_string(test_parking_file,
              "conf/planning/parking/perfect_slots_verticle.txt",
              "The praking info test file");

DEFINE_double(adjust_roi_shape_longitudinal_buffer, 0.2,
              " adjust roi shape longitudinal length.");

DEFINE_double(adjust_wheel_mask_buffer, 0.4,
              "wheel and wheel mask 's distance");

DEFINE_bool(enable_consider_wheel_mask, false, "enable consider wheel mask");

DEFINE_double(each_rs_length, 0.3, "choose appropriate Rs curve length");

// hpp cruise parameter
DEFINE_int32(hpp_cruise_finish_count_threhold, 20,
             "hpp_cruise_finish_count_threhold");
DEFINE_int32(hpp_cruise_prefinish_count_threhold, 25,
             "hpp_cruise_prefinish_count_threhold");

DEFINE_bool(enable_osqp_finite_smoother, true, "use new smooth method");

// APA:set end_pose_state' v
DEFINE_bool(is_enable_end_pose_v, false,
            "enable set APA's scene end pose velocity");
DEFINE_double(Openspace_end_pose_v, 0.1, "Openspace end pose velocity");

// AVP:set control calibration endpose in vrf
DEFINE_double(ctl_calibr_x, 2, "control calibration endpose x");
DEFINE_double(ctl_calibr_y, 2, "control calibration endpose y");
DEFINE_double(ctl_calibr_theta, 0.0, "control calibration endpose heading");

// AVP: set later slot special domain conf
DEFINE_double(park_in_lateral_ignore_bottom_dist_threhold, 2.0,
              "park_in_lateral_ignore_bottom_dist_threhold");
DEFINE_double(park_in_lateral_leave_parking_domain_bottom_dist_threhold, 2.5,
              "park_in_lateral_ignore_bottom_dist_threhold");
// vt_sample_optimizer
DEFINE_bool(enable_vt_sample_urgent_curve, false,
            "enable vt sample optimizer to sample urgent curve");

DEFINE_double(avp_ego_inflated_buffer_for_checking_collision, 0.05,
              "avp ego inflated buffer for checking collision");

DEFINE_double(
    extra_collision_buffer_for_nns_adjust, 0.25,
    "additional buffer in nns adjust scenario for checking collision");

DEFINE_double(avp_max_guild_line_display, 3.0,
              "avp max  guild line display value");

DEFINE_bool(force_mirror_fold, false, "force_mirror_fold");
DEFINE_bool(use_fake_perception, false, "use fake perception");
DEFINE_double(history_trace_path_extend_buffer, 0.5,
              "history trace path extend buffer");
DEFINE_double(openspace_short_path_limit, 0.5, "openspace short path limit");
DEFINE_double(initial_trans_to_park_speed_threshold, 0.05,
              "initial trans to park speed threshold");

// AVP:set parking inwards swtich conf
DEFINE_bool(avp_enable_parking_inwards, false, "avp enable parking inwards");

DEFINE_double(lat_spot_enable_trace_replan_dist_threshold, 5.0,
              "lat spot enable trace replan dist threshold");
DEFINE_double(non_lat_spot_enable_trace_replan_dist_threshold, 5.0,
              "non lat spot enable trace replan dist threshold");

// perception lane line state config
DEFINE_double(lane_line_rise_limit_time, 2.0,
              "lane_line_rise_limit_time, the status will switch after a "
              "period of success");
DEFINE_bool(enable_odd_area_internal_to_perception, true,
            "enable_odd_area_internal_to_perception ");
DEFINE_bool(enable_layer_odd_area_reduce, false,
            "enable_layer_odd_area_internal_reduce_to_perception ");
DEFINE_bool(enable_failure_location_checker, false,
            "enable_failure_location_checker ");
DEFINE_double(min_map_lane_diff, 0.32, "min_map_lane_diff");
DEFINE_bool(only_using_locationself_err, false, "only_using_locationself_err");
DEFINE_bool(using_hdmap_lnaechange_type, true, "using_hdmap_lnaechange_type");
DEFINE_bool(using_camera_lanemarker_points, false,
            "using_camera_lanemarker_points");
DEFINE_bool(using_ldp_stvl_high, false, "using_ldp_stvl_high");
DEFINE_bool(not_using_nolane_func_gain, true, "flag_not_using_func_gain");
DEFINE_uint32(using_missile_mode_level, 3, "using_missile_mode_level");

// fault management
DEFINE_string(planning_fm_error_code_map_file,
              "conf/planning/fm/planning_fm_error_code_map.pb.txt",
              "planning fm error code map file");
DEFINE_string(orin_planning_fm_error_code_map_file,
              "conf/planning/fm/orin/planning_fm_error_code_map.pb.txt",
              "planning fm error code map file");
DEFINE_uint32(local_view_queue_size, 1, "local view queue size");

DEFINE_uint32(local_view_queue_timeout, 50,
              "local view queue timeout, unit:ms");

DEFINE_bool(export_local_view_to_file, false, "output_local_view_to_file ");
DEFINE_bool(export_input_data_to_file, false, "input_data_to_file ");

DEFINE_bool(enable_lon_nudge_parallel_obstacle, true,
            "enable lon nudge parellel obstacle");

DEFINE_bool(enable_change_buffer_when_gear_changed, false,
            "enable change buffer when gear changed");

DEFINE_string(ipopt_pos_optimize_smoother_config_file,
              "conf/planning/smoother/"
              "ipopt_pos_optimize_smoother_config.pb.txt",
              "The TBA ipopt_pos_optimize_smoother_config configuration file");

DEFINE_double(rcw_double_flashing_time, 5.0, "rcw_double_flashing_time");
DEFINE_double(layer_function_quit_distance, 100.0,
              "layer_function_quit_distance");
DEFINE_double(function_quit_distance, 50.0, "function_quit_distance");
DEFINE_bool(switch_layer_function_quit_distance, true,
            "switch_layer_function_quit_distance");

DEFINE_double(input_data_publish_time_out_period_num, 0.3,
              "publish_time time out");
DEFINE_double(input_data_data_time_out_period_num, 0.5, "data_stamp time out");

DEFINE_bool(use_TL_lane, false, "use TL lane");

DEFINE_string(default_speed_cost_config_file,
              "conf/planning/speed/default_speed_cost_config.pb.txt",
              "default speed cost config file");
DEFINE_string(default_speed_scenario_config_dir, "conf/planning/speed/scenario",
              "default speed scenario config dir");

DEFINE_string(default_joint_scenario_config_dir, "conf/planning/joint/scenario",
              "default speed scenario config dir");

DEFINE_double(map_radius, 200.0, "map radius");

DEFINE_double(open_space_thread_manager_max_wait_time, 40.0,
              " open space thread manager max wait time, default 40.0 s ");
DEFINE_bool(use_dkappa_speed_limit, false, "use dkappa speed limit");
DEFINE_bool(pnc_map_valid_log, false, "debug pnc_map routing");
DEFINE_double(pnc_curve_base_length, 20,
              " open space thread manager max wait time, default 40.0 s ");
DEFINE_double(pnc_curve_derived_length, 40,
              " open space thread manager max wait time, default 40.0 s ");
DEFINE_double(pnc_vehicle_pose_predict_time, 0.2,
              "vehicle pose predcition_time in pnc_map");

DEFINE_bool(use_freespace_process_bound, false, "use_freespace_process_bound");

DEFINE_bool(use_caution_dynamic_obs_nudge, true,
            "use caution dynamic obs nudge");

DEFINE_bool(use_dynamic_adjust_towing_l, true, "use_dynamic_adjust_towing_l");
// dynamic time gap
DEFINE_bool(use_dynamic_time_gap, true, "use dynamic time gap");
DEFINE_double(time_gap_relative_velocity_coef, 0.012,
              "time gap relative velocity coef");
DEFINE_double(time_gap_front_accel_coef, 0.015, "time gap front accel coef");

DEFINE_bool(enable_lane_change_safety_log, true,
            "Enable lange change safety log.");

DEFINE_uint32(pb_mem_pool_block_count, 40, "Protobuf memory pool block count");
DEFINE_uint32(pb_mem_pool_block_size, 2 * 1024 * 1024,
              "Protobuf memory pool block size, Default: 2M");
DEFINE_bool(use_original_fct_in, true, "use original fct_in");
// NOLINTEND
