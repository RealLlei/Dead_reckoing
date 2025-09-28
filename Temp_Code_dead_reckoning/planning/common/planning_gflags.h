#ifndef PLANNING_COMMON_PLANNING_GFLAGS_H
#define PLANNING_COMMON_PLANNING_GFLAGS_H

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

#pragma once
// NOLINTBEGIN
#include "gflags/gflags.h"
#include "gflags/gflags_declare.h"

DECLARE_bool(planning_test_mode);

DECLARE_int32(history_max_record_num);
DECLARE_int32(max_frame_history_num);

DECLARE_string(planning_default_config_file);

// scenarios related
DECLARE_string(scenario_bare_intersection_unprotected_config_file);
DECLARE_string(scenario_emergency_pull_over_config_file);
DECLARE_string(scenario_emergency_stop_config_file);
DECLARE_string(nnp_scenario_lane_follow_config_file);
DECLARE_string(ncp_scenario_lane_follow_config_file);
DECLARE_string(scenario_lane_follow_hybrid_config_file);
DECLARE_string(scenario_learning_model_sample_config_file);
DECLARE_string(scenario_narrow_street_u_turn_config_file);
DECLARE_string(scenario_park_and_go_config_file);
DECLARE_string(scenario_pull_over_config_file);
DECLARE_string(scenario_stop_sign_unprotected_config_file);
DECLARE_string(scenario_traffic_light_protected_config_file);
DECLARE_string(scenario_traffic_light_unprotected_left_turn_config_file);
DECLARE_string(scenario_traffic_light_unprotected_right_turn_config_file);
DECLARE_string(scenario_valet_parking_config_file);
DECLARE_string(scenario_yield_sign_config_file);
DECLARE_string(dynamic_buffer_calculate_config_file);
DECLARE_string(hmi_config_file);
DECLARE_string(default_trigger_config_file);
DECLARE_string(trigger_config_file);
DECLARE_string(orin_trigger_config_file);
DECLARE_string(metric_config_file);
DECLARE_string(valet_parking_history_trace_config_file);

DECLARE_bool(enable_scenario_bare_intersection);
DECLARE_bool(enable_scenario_emergency_pull_over);
DECLARE_bool(enable_scenario_emergency_stop);
DECLARE_bool(enable_scenario_park_and_go);
DECLARE_bool(enable_scenario_pull_over);
DECLARE_bool(enable_scenario_stop_sign);
DECLARE_bool(enable_scenario_traffic_light);
DECLARE_bool(enable_scenario_yield_sign);

DECLARE_double(slack_add_obs_buff_in_ref);
DECLARE_bool(enable_scenario_side_pass_multiple_parked_obstacles);
DECLARE_bool(enable_force_pull_over_open_space_parking_test);

DECLARE_string(traffic_rule_config_filename);
DECLARE_string(reference_line_smoother_config_filename);
DECLARE_string(openspace_path_smoother_config_filename);
DECLARE_int32(planning_loop_rate);
DECLARE_string(rtk_trajectory_filename);
DECLARE_uint64(rtk_trajectory_forward);
DECLARE_double(rtk_trajectory_resolution);

DECLARE_bool(prioritize_change_lane);
DECLARE_double(change_lane_min_length);

DECLARE_bool(enable_trajectory_stitcher);
DECLARE_bool(enable_st_valid_bound_check);

DECLARE_double(nolane_obs_x_threshold);
DECLARE_double(nolane_obs_y_threshold);
DECLARE_double(nolane_obs_ego_cost_kdistance);
DECLARE_double(nolane_obs_ego_cost_kv_theta);
DECLARE_double(nolane_obs_ego_cost_rho);
DECLARE_double(nolane_obs_ego_cost_dis);
DECLARE_double(nolane_obs_ego_cost_lateral);
DECLARE_double(nolane_obs_ego_cost_theta);
DECLARE_double(nolane_obs_ego_cost_speed);
DECLARE_double(nolane_trajectory_fit_radius);
DECLARE_double(nolane_trajectory_fit_factor);
DECLARE_double(nolane_obs_ego_cost_intersection_angle_diff);
DECLARE_double(nolane_obs_ego_too_far_ignore_cost);
DECLARE_double(nolane_obs_ego_too_closed_ignore_cost);
DECLARE_bool(nolane_log_all);
DECLARE_bool(nolane_log_none);
DECLARE_double(nolane_trajectory_minimal_length);
DECLARE_double(nolane_obstacle_min_distance);
DECLARE_double(nolane_obs_min_velocity);
DECLARE_double(nolane_ego_traj_max_l);
DECLARE_double(nolane_min_s_before_ego_in_ref);
DECLARE_double(nolane_ego_is_close_ref);
DECLARE_double(nolane_obs_tracking_max_time);
DECLARE_double(nolane_obs_change_lane_min_dis);
DECLARE_double(nolane_obs_change_lane_ego_time);
DECLARE_double(nolane_obs_max_lateral_offset_in_straight);
DECLARE_double(nolane_ego_follow_obs_theta);
DECLARE_double(nolane_ego_follow_obs_rel_x);
DECLARE_double(nolane_ego_follow_obs_rel_y);

DECLARE_int32(least_size_in_reuse_speed_plan);

// parameters for trajectory stitching and reinit planning starting point.
DECLARE_double(replan_lateral_distance_threshold);
DECLARE_double(replan_heading_threshold);
DECLARE_double(replan_longitudinal_distance_threshold);
DECLARE_double(replan_longitudinal_time_threshold);
DECLARE_bool(replan_every_period);
DECLARE_double(longitudinal_replan_speed_threshold);
DECLARE_double(valet_parking_replan_longitudinal_distance_threshold);
DECLARE_double(open_space_replan_longitudinal_distance_threshold);
DECLARE_double(open_space_replan_velocity_threshold);

// parameter for reference line
DECLARE_bool(enable_smooth_reference_line);
DECLARE_bool(enable_smooth_select_reference_line);
DECLARE_bool(enable_reference_line_stitching);
DECLARE_bool(enable_reference_line_interval_segment);
DECLARE_bool(enable_reference_line_cost_segment);
DECLARE_bool(enable_reference_line_provider_thread);
DECLARE_bool(enable_reference_line_history);
DECLARE_double(default_reference_line_width);
DECLARE_bool(enable_reference_line_backward_anchor_point_unsmooth);
DECLARE_bool(enable_reference_line_backward_anchor_point_unsmooth_avp);

// parameters for trajectory planning
DECLARE_double(planning_upper_speed_limit);
DECLARE_double(path_boundary_min_length);
DECLARE_double(trajectory_time_min_interval);
DECLARE_double(trajectory_time_max_interval);
DECLARE_double(trajectory_time_high_density_period);
DECLARE_double(follow_time_desired);
DECLARE_double(follow_tanh_factor);

// parameters for trajectory sanity check
DECLARE_bool(enable_trajectory_check);
DECLARE_double(speed_lower_bound);
DECLARE_double(speed_upper_bound);

DECLARE_double(longitudinal_acceleration_lower_bound);
DECLARE_double(longitudinal_acceleration_upper_bound);

DECLARE_double(longitudinal_jerk_lower_bound);
DECLARE_double(longitudinal_jerk_upper_bound);
DECLARE_double(longitudinal_jerk_upper_bound_slack);
DECLARE_double(longitudinal_jerk_lower_bound_slack);
DECLARE_double(lateral_jerk_bound);
DECLARE_double(longitudinal_acce_upper_bound_slack);
DECLARE_double(longitudinal_acce_lower_bound_slack);
DECLARE_double(longitudinal_speed_buffer);
DECLARE_double(longitudinal_dis_buffer);

DECLARE_double(kappa_bound);

// speed limit
DECLARE_double(delta_time_speed_limit);

// STBoundary
DECLARE_double(st_max_s);
DECLARE_double(st_max_t);
DECLARE_bool(debug_cost_hexagon_big_car);
DECLARE_bool(enable_avoid_big_cart);
DECLARE_double(speed_planning_delta_time);
DECLARE_int32(speed_freeze_frame);
DECLARE_bool(enable_cut_trajectory);
DECLARE_double(cut_vehicle_trajectory_time_length);
DECLARE_double(cut_bicycle_trajectory_time_length);

// Decision Part
DECLARE_bool(enable_nudge_slowdown);
DECLARE_double(static_obstacle_nudge_l_buffer);
DECLARE_double(nonstatic_obstacle_nudge_l_buffer);
DECLARE_double(lane_change_obstacle_nudge_l_buffer);
DECLARE_double(lateral_ignore_buffer);
DECLARE_double(min_stop_distance_obstacle);
DECLARE_double(max_stop_distance_obstacle);
DECLARE_double(follow_min_distance);
DECLARE_double(follow_min_obs_lateral_distance);
DECLARE_double(yield_distance);
DECLARE_double(follow_time_buffer);
DECLARE_double(follow_min_time_sec);
DECLARE_double(signal_expire_time_sec);
DECLARE_bool(enable_lane_change_by_turn_signal);
DECLARE_bool(enable_lane_change_by_audio_signal);
DECLARE_bool(only_enable_efficient_lane_change);
DECLARE_bool(enable_lane_change_by_obstacle);
DECLARE_bool(enable_lane_change_test_by_pad_msg);
DECLARE_double(reference_line_priority_cost);
DECLARE_double(straight_forward_line_cost);
DECLARE_bool(lane_change_planning_error_send_hmi);
DECLARE_string(reference_line_info_decider_config_file);
DECLARE_double(path_lane_change_distance_preview_time);

// Path Deciders
DECLARE_bool(enable_skip_path_tasks);
DECLARE_bool(enable_reuse_path_in_lane_follow);

DECLARE_bool(enable_original_lane_borrow_process);
DECLARE_bool(enable_big_car_dynamic_obstacle_process);
DECLARE_bool(enable_dynamic_vehicle_total_consider_length);
DECLARE_bool(enable_path_bound_debug);
DECLARE_bool(enable_reference_line_debug);
DECLARE_double(obstacle_lat_buffer);
DECLARE_double(obstacle_lon_start_buffer);
DECLARE_double(obstacle_lon_end_buffer);
DECLARE_double(static_obstacle_speed_threshold);
DECLARE_double(lane_borrow_max_speed);
DECLARE_int32(long_term_blocking_obstacle_cycle_threshold);
DECLARE_bool(enable_path_integral_calculate);
DECLARE_bool(enable_path_bound_record_debug);
DECLARE_bool(enable_towing_line_debug);
DECLARE_bool(enable_judge_whether_valid_path);

DECLARE_string(destination_obstacle_id);
DECLARE_double(destination_check_distance);

DECLARE_double(virtual_stop_wall_length);
DECLARE_double(virtual_stop_wall_height);

DECLARE_double(prediction_total_time);
DECLARE_bool(align_prediction_time);
DECLARE_int32(trajectory_point_num_for_debug);
DECLARE_double(lane_change_prepare_length);
DECLARE_double(min_lane_change_prepare_length);
DECLARE_double(allowed_lane_change_failure_time);
DECLARE_bool(enable_smarter_lane_change);

DECLARE_double(turn_signal_distance);
DECLARE_double(adc_l_buffer_for_static_obstacle_st_boundary);
DECLARE_double(adc_l_buffer_for_static_obstacle_avp_mode);
DECLARE_bool(enable_plot_speed_l_buffer);

// QpSt optimizer
DECLARE_double(slowdown_profile_deceleration);
DECLARE_double(delta_acceleration);
DECLARE_double(max_deceleration_backup);

DECLARE_bool(enable_sqp_solver);

// piecewise jeck path problem set ddx and dddx weight (v*low_spd_weight +
// min_longitudinal_spd)
DECLARE_double(min_longitudinal_spd);
DECLARE_double(low_spd_weight);
DECLARE_double(max_lateral_acc);
DECLARE_double(max_lateral_jerk);
DECLARE_double(path_bound_relaxation_param);

/// thread pool
DECLARE_bool(use_multi_thread_to_add_obstacles);
DECLARE_bool(enable_multi_thread_in_dp_st_graph);
DECLARE_bool(use_multi_thread_in_reference_line);
DECLARE_bool(use_multi_thread_obs_build_stbound);
DECLARE_bool(use_multi_thread_path_optimaize);

DECLARE_double(numerical_epsilon);
DECLARE_double(default_cruise_speed);

DECLARE_double(trajectory_time_resolution);
DECLARE_double(trajectory_space_resolution);
DECLARE_int32(min_path_data_point_count);
DECLARE_double(lateral_acceleration_bound);
DECLARE_double(speed_lon_decision_horizon);
DECLARE_uint64(num_velocity_sample);
DECLARE_bool(enable_backup_trajectory);
DECLARE_double(backup_trajectory_cost);
DECLARE_double(min_velocity_sample_gap);
DECLARE_double(lon_collision_buffer);
DECLARE_double(lat_collision_buffer);
DECLARE_uint64(num_sample_follow_per_timestamp);

DECLARE_bool(lateral_optimization);
DECLARE_double(weight_lateral_offset);
DECLARE_double(weight_lateral_derivative);
DECLARE_double(weight_lateral_second_order_derivative);
DECLARE_double(weight_lateral_third_order_derivative);
DECLARE_double(weight_lateral_obstacle_distance);
DECLARE_double(lateral_third_order_derivative_max);
DECLARE_double(lateral_derivative_bound_default);

// Lattice Evaluate Parameters
DECLARE_double(weight_lon_objective);
DECLARE_double(weight_lon_jerk);
DECLARE_double(weight_lon_collision);
DECLARE_double(weight_lat_offset);
DECLARE_double(weight_lat_comfort);
DECLARE_double(weight_centripetal_acceleration);
DECLARE_double(cost_non_priority_reference_line);
DECLARE_double(weight_same_side_offset);
DECLARE_double(weight_opposite_side_offset);
DECLARE_double(weight_dist_travelled);
DECLARE_double(weight_target_speed);
DECLARE_double(lat_offset_bound);
DECLARE_double(lon_collision_yield_buffer);
DECLARE_double(lon_collision_overtake_buffer);
DECLARE_double(lon_collision_cost_std);
DECLARE_double(default_lon_buffer);
DECLARE_double(time_min_density);
DECLARE_double(comfort_acceleration_factor);
DECLARE_double(polynomial_minimal_param);
DECLARE_double(lattice_stop_buffer);
DECLARE_double(max_s_lateral_optimization);
DECLARE_double(default_delta_s_lateral_optimization);
DECLARE_double(bound_buffer);
DECLARE_double(nudge_buffer);

DECLARE_double(fallback_total_time);
DECLARE_double(fallback_time_unit);

DECLARE_double(speed_bump_speed_limit);
DECLARE_double(default_city_road_speed_limit);
DECLARE_double(default_highway_speed_limit);

// navigation mode
DECLARE_bool(enable_planning_pad_msg);
DECLARE_bool(enable_turn_signal_before_lane_change);
DECLARE_bool(enable_no_lane_change_when_no_auto_drive);

// open space planner
DECLARE_string(planner_open_space_config_filename);
DECLARE_bool(use_dual_variable_warm_start);
DECLARE_bool(use_s_curve_speed_smooth);
DECLARE_bool(enable_parallel_path_smoothing);

DECLARE_bool(enable_osqp_debug);
DECLARE_bool(export_chart);
DECLARE_bool(enable_record_debug);
DECLARE_bool(enable_record_openspace_debug);
DECLARE_bool(enable_vt_sample_debug);
DECLARE_bool(enable_one_shoot_log);
DECLARE_string(one_shoot_log_root_dir);

DECLARE_double(default_front_clear_distance);

DECLARE_double(max_trajectory_len);
DECLARE_bool(enable_rss_fallback);
DECLARE_bool(enable_rss_info);
DECLARE_double(rss_max_front_obstacle_distance);

DECLARE_bool(enable_planning_smoother);
DECLARE_double(smoother_stop_distance);

DECLARE_double(side_pass_driving_width_l_buffer);

DECLARE_bool(enable_parallel_hybrid_a);

DECLARE_double(open_space_standstill_acceleration);
DECLARE_double(pause_brake_acceleration);

DECLARE_bool(enable_dp_reference_speed);
DECLARE_double(dp_acceleration_buffer);
DECLARE_double(speed_threshold_big_car);
DECLARE_double(length_threshold_big_car);
DECLARE_double(area_threshold_big_car);
DECLARE_double(init_point_lmax_threshold_big_car);
DECLARE_double(path_point_lmin_threshold_big_car);
DECLARE_double(path_point_lmax_threshold_big_car);

DECLARE_double(message_latency_threshold);
DECLARE_bool(enable_lane_change_urgency_checking);
DECLARE_bool(enable_avoid_stop_to_lane_change);
DECLARE_double(min_speed_for_lane_change);
DECLARE_double(short_path_length_threshold);
DECLARE_double(constraint_length_from_start_point);
DECLARE_bool(st_boundary_debug_info);

DECLARE_uint64(trajectory_stitching_preserved_length);
DECLARE_double(trajectory_stitching_time);

DECLARE_bool(use_st_drivable_boundary);

DECLARE_bool(use_smoothed_dp_guide_line);

DECLARE_bool(use_soft_bound_in_nonlinear_speed_opt);

DECLARE_bool(use_front_axe_center_in_path_planning);

DECLARE_bool(use_road_boundary_from_map);

DECLARE_uint32(apa_gear_shift_limit);

// learning related
DECLARE_bool(planning_offline_learning);
DECLARE_string(planning_data_dir);
DECLARE_string(planning_offline_bags);
DECLARE_int32(learning_data_obstacle_history_time_sec);
DECLARE_int32(learning_data_frame_num_per_file);
DECLARE_string(planning_birdview_img_feature_renderer_config_file);
DECLARE_int32(min_past_history_points_len);

// hybrid model
DECLARE_bool(skip_path_reference_in_side_pass);
DECLARE_bool(skip_path_reference_in_change_lane);

// navigation_hdmap
DECLARE_string(navigation_hdmap_config_filename);
DECLARE_int32(navigation_hdmap_loop_rate);
DECLARE_bool(navigation_hdmap_generate_left_boundray);
DECLARE_bool(navigation_hdmap_no_lane_marker_c3);
DECLARE_bool(navigation_hdmap_no_lane_marker_c2);
DECLARE_bool(using_filter_of_c2);
DECLARE_bool(using_filter_of_c3);
DECLARE_double(buffer_gainst_lookford_distance);
DECLARE_double(common_center_line_resolution);
DECLARE_int32(right_navi_path_priority);
DECLARE_int32(left_navi_path_priority);
DECLARE_int32(current_navi_path_priority);
DECLARE_double(has_no_lanemarker_time);
DECLARE_bool(using_original_lanemarker_boundary);
DECLARE_bool(generate_three_centralines_alltime);
DECLARE_double(central_lane_min_width);
DECLARE_double(central_lane_min_length_after_cut);
DECLARE_double(lane_max_back_length);

DECLARE_bool(enable_online_hdmap_generator);
DECLARE_int32(online_hdmap_geberatepath_num);
DECLARE_double(online_hdmap_geberatepath_lane_width);
DECLARE_double(adaptive_cruise_lane_width);
DECLARE_double(steering_wheel_reduction_factor);
DECLARE_bool(enable_esitimate_position_when_stitching);
DECLARE_bool(enable_prediction_output);
DECLARE_bool(enable_skip_check_input);
DECLARE_double(yield_time_buffer);

DECLARE_bool(enable_planning_warning);

/// AVP
DECLARE_double(park_out_distance);
DECLARE_bool(lgsvl_sim_switch);
DECLARE_double(parkingout_roi_length);

DECLARE_double(direct_move_roi_length);
DECLARE_double(direct_move_roi_width);
DECLARE_double(direct_move_length);
DECLARE_double(nns_stuck_roi_min_x_margin);
DECLARE_double(nns_stuck_roi_max_x_margin);
DECLARE_double(nns_stuck_roi_min_y_margin);
DECLARE_double(nns_stuck_roi_max_y_margin);
DECLARE_double(default_cruise_low_speed);
DECLARE_double(park_out_early_stop_angle_limit);

DECLARE_int32(lgsvl_sim_parking_direction);
DECLARE_bool(forbit_replan_switch);

DECLARE_int32(perception_loss_count);

DECLARE_int32(publish_trajectory_points_number);
DECLARE_string(test_parking_file);
DECLARE_double(adjust_roi_shape_longitudinal_buffer);

DECLARE_double(open_space_lane_width);

// consider wheel mask
DECLARE_double(adjust_wheel_mask_buffer);
DECLARE_bool(enable_consider_wheel_mask);

// AVP RS
DECLARE_double(each_rs_length);

// hpp cruise parameter
DECLARE_int32(hpp_cruise_finish_count_threhold);
DECLARE_int32(hpp_cruise_prefinish_count_threhold);

DECLARE_bool(enable_osqp_finite_smoother);

// APA:set end_pose_state' v
DECLARE_bool(is_enable_end_pose_v);
DECLARE_double(Openspace_end_pose_v);

// AVP:set control calibration endpose in vrf
DECLARE_double(ctl_calibr_x);
DECLARE_double(ctl_calibr_y);
DECLARE_double(ctl_calibr_theta);

// AVP: set later slot special domain conf
DECLARE_double(park_in_lateral_ignore_bottom_dist_threhold);
DECLARE_double(park_in_lateral_leave_parking_domain_bottom_dist_threhold);

// AVP:set parking inwards swtich conf
DECLARE_bool(avp_enable_parking_inwards);

DECLARE_double(lat_spot_enable_trace_replan_dist_threshold);
DECLARE_double(non_lat_spot_enable_trace_replan_dist_threshold);

// vt_sample_optimizer
DECLARE_bool(enable_vt_sample_urgent_curve);

DECLARE_double(avp_ego_inflated_buffer_for_checking_collision);
DECLARE_double(extra_collision_buffer_for_nns_adjust);

DECLARE_double(avp_max_guild_line_display);

DECLARE_bool(force_mirror_fold);
DECLARE_bool(use_fake_perception);

DECLARE_double(history_trace_path_extend_buffer);
DECLARE_double(initial_trans_to_park_speed_threshold);

DECLARE_double(openspace_short_path_limit);

// perception lane line state config
DECLARE_double(lane_line_rise_limit_time);
DECLARE_bool(enable_odd_area_internal_to_perception);
DECLARE_bool(enable_layer_odd_area_reduce);
DECLARE_bool(enable_failure_location_checker);
DECLARE_double(min_map_lane_diff);
DECLARE_bool(only_using_locationself_err);
DECLARE_bool(using_hdmap_lnaechange_type);
DECLARE_bool(using_camera_lanemarker_points);
DECLARE_bool(using_ldp_stvl_high);
DECLARE_bool(not_using_nolane_func_gain);
DECLARE_uint32(using_missile_mode_level);

// fault management
DECLARE_string(planning_fm_error_code_map_file);
DECLARE_string(orin_planning_fm_error_code_map_file);
DECLARE_uint32(local_view_queue_size);

DECLARE_uint32(local_view_queue_timeout);
DECLARE_uint32(load_nnp_ncp_map_interval);
DECLARE_uint32(load_perception_map_interval);
DECLARE_bool(export_local_view_to_file);
DECLARE_bool(export_input_data_to_file);

DECLARE_bool(enable_lon_nudge_parallel_obstacle);

DECLARE_bool(enable_change_buffer_when_gear_changed);

DECLARE_string(ipopt_pos_optimize_smoother_config_file);

DECLARE_double(rcw_double_flashing_time);
DECLARE_double(layer_function_quit_distance);
DECLARE_double(function_quit_distance);
DECLARE_bool(switch_layer_function_quit_distance);
DECLARE_bool(use_TL_lane);

DECLARE_string(default_speed_cost_config_file);
DECLARE_string(default_speed_scenario_config_dir);

DECLARE_string(default_joint_scenario_config_dir);

DECLARE_double(map_radius);

DECLARE_double(open_space_thread_manager_max_wait_time);

DECLARE_bool(use_dkappa_speed_limit);

DECLARE_bool(pnc_map_valid_log);
DECLARE_double(pnc_curve_base_length);
DECLARE_double(pnc_curve_derived_length);
DECLARE_double(pnc_vehicle_pose_predict_time);

DECLARE_bool(use_freespace_process_bound);
DECLARE_bool(use_caution_dynamic_obs_nudge);
DECLARE_bool(use_dynamic_adjust_towing_l);
DECLARE_double(input_data_publish_time_out_period_num);
DECLARE_double(input_data_data_time_out_period_num);
DECLARE_bool(use_dynamic_time_gap);
DECLARE_double(time_gap_relative_velocity_coef);
DECLARE_double(time_gap_front_accel_coef);

DECLARE_bool(enable_lane_change_safety_log);

DECLARE_uint32(pb_mem_pool_block_count);
DECLARE_uint32(pb_mem_pool_block_size);

DECLARE_bool(use_original_fct_in);
// NOLINTEND
#endif  // PLANNING_COMMON_PLANNING_GFLAGS_H
