cmake_minimum_required(VERSION 3.12.1)
project(europa_planning)

if((PLATFORM MATCHES "x86") AND (COMPILE_ALL))
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ftest-coverage -fprofile-arcs")
endif()

file(
  GLOB
  PROTOS
  "proto/*.proto"
  "proto/math/*.proto"
  "routing/proto/*.proto"
  "proto/path/*.proto"
  "prediction/proto/*.proto"
  )
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTOS})

SET(CMAKE_CXX_FLAGS "-Wall ${CMAKE_CXX_FLAGS}")
message("Planning Total Compiler Options: " ${CMAKE_CXX_FLAGS})

include_directories(${MapManage_INCLUDE_DIRS})
link_directories(${MapManage_LIBRARY_DIRS})


add_library(
  ${PROJECT_NAME} SHARED
  ${PROTO_SRCS}
  common/smoothers/smoother.cc
  common/path_decision.cc
  common/history.cc
  common/obstacle_blocking_analyzer.cc
  common/speed/speed_data.cc
  common/speed/st_point.cc
  common/speed/st_boundary.cc
  common/real_jerk/acc_filter.cc
  common/real_jerk/real_jerk.cc
  common/dependency_injector.h
  common/trajectory_stitcher.cc
  common/frame.cc
  common/indexed_list.h
  common/indexed_queue.h

  # common/trajectory1d/standing_still_trajectory1d.cc
  common/trajectory1d/piecewise_acceleration_trajectory1d.cc

  # common/trajectory1d/piecewise_trajectory1d.cc
  common/trajectory1d/piecewise_jerk_trajectory1d.cc

  # common/trajectory1d/constant_deceleration_trajectory1d.cc
  common/trajectory1d/constant_jerk_trajectory1d.cc
  common/path_boundary.cc
  common/obstacle.cc

  common/ego_info.cc
  common/speed_limit.cc
  common/planning_context.cc
  common/planning_chart_debug.cc

  common/util/math_util.cc
  common/util/common.cc
  common/util/util.cc
  common/reference_line_info.cc
  common/st_graph_data.cc
  common/trajectory/discretized_trajectory.cc
  common/trajectory/publishable_trajectory.cc
  common/planning_gflags.cc
  common/open_space_info.cc
  common/open_space_chart.cc
  common/path/discretized_path.cc
  common/path/fallback_path.cc
  common/path/frenet_frame_path.cc
  common/path/path_data.cc
  common/speed_profile_generator.cc
  core/on_lane_planning.cc
  core/planning_base.cc
  core/safety_guard_planning.cc

  common/game/vehicle_model/idm_model.cc
  common/game/vehicle_model/ideal_steer_model.cc
  common/game/vehicle_model/ctx_idm_model.cc
  common/game/game_common/rss/responsibility_sensitive_safety.cc
  common/game/game_common/idm/idm.cc
  common/game/forward_simulator/onlane_forward_simulation.cc
  common/game/controllers/pure_pursuit_controller.cc
  common/game/controllers/intelligent_velocity_controller.cc
  common/game/controllers/ctx_idm_velocity_controller.cc

  math/polynomial_xd.cc
  math/discretized_points_smoothing/fem_pos_deviation_osqp_interface.cc
  math/discretized_points_smoothing/fem_pos_deviation_sqp_osqp_interface.cc
  math/discretized_points_smoothing/fem_pos_deviation_smoother.cc
  math/discretized_points_smoothing/osqp_finite_difference_smoother.cc
  math/discretized_points_smoothing/osqp_finite_difference_interface.cc
  math/discretized_points_smoothing/ipopt_pos_optimize_math_model.cc
  math/discretized_points_smoothing/ipopt_pos_optimize_smoother.cc
  math/piecewise_jerk/piecewise_jerk_problem.cc
  math/piecewise_jerk/piecewise_jerk_path_problem.cc
  math/piecewise_jerk/piecewise_jerk_speed_problem.cc
  math/smoothing_spline/spline_2d_constraint.cc
  math/smoothing_spline/affine_constraint.cc
  math/smoothing_spline/spline_1d.cc
  math/smoothing_spline/spline_seg_kernel.cc
  math/smoothing_spline/osqp_spline_2d_solver.cc
  math/smoothing_spline/spline_1d_constraint.cc
  math/smoothing_spline/spline_1d_kernel.cc
  math/smoothing_spline/osqp_spline_1d_solver.cc
  math/smoothing_spline/spline_2d.cc
  math/smoothing_spline/spline_1d_seg.cc
  math/smoothing_spline/spline_2d_seg.cc
  math/smoothing_spline/spline_2d_kernel.cc
  math/smoothing_spline/spline_1d_solver.cc
  math/curve1d/quintic_spiral_path.cc
  math/curve1d/quadratic_polynomial_curve1d.cc
  math/curve1d/cubic_polynomial_curve1d.cc
  math/curve1d/quartic_polynomial_curve1d.cc
  math/curve1d/linear_polynomial_curve1d.cc
  math/curve1d/piecewise_quintic_spiral_path.cc
  math/curve1d/quintic_polynomial_curve1d.cc
  math/quadratic_function_math.cc
  math/curve_math.cc
  math/discrete_points_math.cc
  online_hdmap_generator/online_hdmap_generator.cc
  open_space/coarse_path_generator/hybrid_a_star.cc
  open_space/coarse_path_generator/path_generator.cc
  open_space/coarse_path_generator/geometry_path_generator.cc
  open_space/coarse_path_generator/ilqr_path.cc
  open_space/coarse_path_generator/node3d.cc
  open_space/coarse_path_generator/shape/reeds_shepp_path.cc
  open_space/coarse_path_generator/shape/geometry_path.cc
  open_space/coarse_path_generator/shape/scs_shape_path.cc
  open_space/coarse_path_generator/utils/penalty_function_method.cc
  open_space/coarse_path_generator/geometric_path.cc
  # open_space/trajectory_smoother/iterative_anchoring_smoother.cc
  open_space/ilqr_smoother/al_cost.cc
  open_space/ilqr_smoother/al_ilqr_interface.cc
  open_space/ilqr_smoother/al_ilqr.cc
  open_space/ilqr_smoother/kinematic_model.cc
  open_space/nlp_path_smoother/nlp_path_smoother.cc
  open_space/nlp_path_smoother/nlp_math_model_cppad.cc
  open_space/nlp_path_smoother/nlp_math_model.cpp

  planner/planner_dispatcher.cc
  planner/on_lane_planner_dispatcher.cc
  planner/public_road/public_road_planner.cc
  planner/planner.h
  planner/safety_guard_planner_dispatcher.cc
  planner/safety_guard/safety_guard_planner.cc
  pnc_map/route_segments.cc
  pnc_map/passage_routing_info.cc
  pnc_map/new_framework/optimal_pass.cc
  pnc_map/new_framework/downstream_interface.cc
  pnc_map/new_framework/routing_construct_pass.cc
  pnc_map/new_framework/help_aux.cc
  pnc_map/new_framework/lane_info.cc
  pnc_map/new_framework/pnc_map_b.cc
  pnc_map/new_framework/road_info.cc
  pnc_map/new_framework/passage_pnc_b.cc
  pnc_map/new_framework/pnc_validation.cc
  pnc_map/new_framework/way_info.cc
  reference_line/reference_point.cc

  reference_line/util/reference_line_debug.cc
  reference_line/smoother/qp_spline_reference_line_smoother.cc
  reference_line/smoother/discrete_points_reference_line_smoother.cc

  reference_line/reference_line.cc
  reference_line/reference_line_provider.cc

  reference_line_info_decider/junction_reference_line_info_decider/junction_reference_line_info_decider.cc
  reference_line_info_decider/common/reference_line_info_decider_helper.cc
  reference_line_info_decider/common/reference_line_info_decider_common.cc
  reference_line_info_decider/lane_change_intent_decider/lane_change_intent_decider.cc
  reference_line_info_decider/lane_change_intent_decider/audio_lane_change.cc
  reference_line_info_decider/lane_change_intent_decider/turn_signal_switch_lane_change.cc
  reference_line_info_decider/lane_change_intent_decider/navigator_lane_change.cc
  reference_line_info_decider/lane_change_intent_decider/efficiency_lane_change.cc
  reference_line_info_decider/reference_line_info_decider.cc

  lane_change_safety_decider/carrier/carrier.cc
  lane_change_safety_decider/gap/gap.cc
  lane_change_safety_decider/gap/gap_common.cc
  lane_change_safety_decider/gap/cruise_model.cc
  lane_change_safety_decider/gap/ttc_checker.cc
  lane_change_safety_decider/gap/gap_manager.cc
  lane_change_safety_decider/lane_change_safety_decider.cc


  scenarios/scenario.cc
  scenarios/common/stage_intersection_cruise_impl.cc
  scenarios/traffic_light/unprotected_left_turn/stage_creep.cc
  scenarios/traffic_light/unprotected_left_turn/stage_intersection_cruise.cc
  scenarios/traffic_light/unprotected_left_turn/traffic_light_unprotected_left_turn_scenario.cc
  scenarios/traffic_light/unprotected_left_turn/stage_approach.cc
  scenarios/traffic_light/protected/stage_intersection_cruise.cc
  scenarios/traffic_light/protected/traffic_light_protected_scenario.cc
  scenarios/traffic_light/protected/stage_approach.cc
  scenarios/traffic_light/unprotected_right_turn/stage_creep.cc
  scenarios/traffic_light/unprotected_right_turn/stage_intersection_cruise.cc
  scenarios/traffic_light/unprotected_right_turn/stage_stop.cc
  scenarios/traffic_light/unprotected_right_turn/traffic_light_unprotected_right_turn_scenario.cc
  scenarios/stop_sign/unprotected/stage_creep.cc
  scenarios/stop_sign/unprotected/stage_intersection_cruise.cc
  scenarios/stop_sign/unprotected/stop_sign_unprotected_scenario.cc
  scenarios/stop_sign/unprotected/stage_stop.cc
  scenarios/stop_sign/unprotected/stage_pre_stop.cc
  scenarios/park/pull_over/stage_retry_parking.cc
  scenarios/park/pull_over/stage_retry_approach_parking.cc
  scenarios/park/pull_over/pull_over_scenario.cc
  scenarios/park/pull_over/stage_approach.cc
  scenarios/yield_sign/stage_creep.cc
  scenarios/yield_sign/yield_sign_scenario.cc
  scenarios/yield_sign/stage_approach.cc
  scenarios/valet_parking/valet_parking_scenario.cc
  scenarios/valet_parking/stage_valet_parking_cruise.cc
  scenarios/valet_parking/stage_valet_parking_parking.cc

  scenarios/stage.cc
  scenarios/bare_intersection/unprotected/stage_intersection_cruise.cc
  scenarios/bare_intersection/unprotected/bare_intersection_unprotected_scenario.cc
  scenarios/bare_intersection/unprotected/stage_approach.cc
  scenarios/lane_follow/nnp_lane_follow_scenario.cc
  scenarios/lane_follow/nnp_lane_follow_stage.cc
  scenarios/lane_follow/ncp_lane_follow_scenario.cc
  scenarios/lane_follow/ncp_lane_follow_stage.cc
  scenarios/scenario_manager.cc
  scenarios/emergency/emergency_stop/stage_standby.cc
  scenarios/emergency/emergency_stop/emergency_stop_scenario.cc
  scenarios/emergency/emergency_stop/stage_approach.cc
  scenarios/emergency/emergency_pull_over/stage_standby.cc
  scenarios/emergency/emergency_pull_over/stage_slow_down.cc
  scenarios/emergency/emergency_pull_over/emergency_pull_over_scenario.cc
  scenarios/emergency/emergency_pull_over/stage_approach.cc
  scenarios/util/util.cc
  scenarios/narrow_street_u_turn/narrow_street_u_turn_scenario.cc
  tasks/task_factory.cc
  tasks/optimizers/trajectory_optimizer.cc

  tasks/optimizers/path_optimizer.cc

  tasks/optimizers/speed_optimizer.cc

  tasks/optimizers/path_speed_optimizer.cc

  tasks/optimizers/piecewise_jerk_path/piecewise_jerk_path_optimizer.cc

  tasks/optimizers/speed_data_optimizer/caches/basic_cache.cc
  tasks/optimizers/speed_data_optimizer/caches/st_obstacle_cache.cc
  tasks/optimizers/speed_data_optimizer/caches/nudge_obstacle_cache.cc
  tasks/optimizers/speed_data_optimizer/caches/speed_limit_cache.cc
  tasks/optimizers/speed_data_optimizer/caches/slt_obstacle_cache.cc
  tasks/optimizers/speed_data_optimizer/caches/speed_cache.cc

  tasks/optimizers/speed_data_optimizer/costs/speed_curve.cc
  tasks/optimizers/speed_data_optimizer/costs/speed_cost.cc
  tasks/optimizers/speed_data_optimizer/costs/accel_cost.cc
  tasks/optimizers/speed_data_optimizer/costs/jerk_cost.cc
  tasks/optimizers/speed_data_optimizer/costs/curve_priority_cost.cc
  tasks/optimizers/speed_data_optimizer/costs/gap_cost.cc
  tasks/optimizers/speed_data_optimizer/costs/low_speed_cost.cc
  tasks/optimizers/speed_data_optimizer/costs/over_speed_cost.cc
  tasks/optimizers/speed_data_optimizer/costs/nudge_cost.cc
  tasks/optimizers/speed_data_optimizer/costs/obstacle_safe_distance_cost.cc
  tasks/optimizers/speed_data_optimizer/costs/obstacle_expected_distance_cost.cc
  tasks/optimizers/speed_data_optimizer/costs/stop_distance_cost.cc
  tasks/optimizers/speed_data_optimizer/costs/collision_cost.cc
  tasks/optimizers/speed_data_optimizer/costs/speed_cost_factory.cc

  tasks/optimizers/speed_data_optimizer/evaluators/speed_evaluator.cc
  tasks/optimizers/speed_data_optimizer/evaluators/lane_keep_evaluator.cc
  tasks/optimizers/speed_data_optimizer/evaluators/lane_change_ongoing_evaluator.cc
  tasks/optimizers/speed_data_optimizer/evaluators/lane_change_prefinish_evaluator.cc
  tasks/optimizers/speed_data_optimizer/evaluators/lane_merge_cruise_evaluator.cc
  tasks/optimizers/speed_data_optimizer/evaluators/lane_merge_stop_evaluator.cc
  tasks/optimizers/speed_data_optimizer/evaluators/lane_continue_evaluator.cc
  tasks/optimizers/speed_data_optimizer/evaluators/reverse_gear_evaluator.cc
  tasks/optimizers/speed_data_optimizer/evaluators/speed_evaluator_factory.cc

  tasks/optimizers/speed_data_optimizer/scenarios/speed_stage.cc
  tasks/optimizers/speed_data_optimizer/scenarios/forward_gear/lane_keep/lane_keep_stage.cc
  tasks/optimizers/speed_data_optimizer/scenarios/forward_gear/lane_change/lane_change_ongoing_stage.cc
  tasks/optimizers/speed_data_optimizer/scenarios/forward_gear/lane_change/lane_change_prefinish_stage.cc
  tasks/optimizers/speed_data_optimizer/scenarios/forward_gear/lane_merge/lane_merge_cruise_stage.cc
  tasks/optimizers/speed_data_optimizer/scenarios/forward_gear/lane_merge/lane_merge_stop_stage.cc
  tasks/optimizers/speed_data_optimizer/scenarios/forward_gear/lane_continue/lane_continue_stage.cc
  tasks/optimizers/speed_data_optimizer/scenarios/reverse_gear/reverse_gear_stage.cc
  tasks/optimizers/speed_data_optimizer/scenarios/speed_stage_factory.cc

  tasks/optimizers/speed_data_optimizer/scenarios/speed_scenario.cc
  tasks/optimizers/speed_data_optimizer/scenarios/forward_gear/forward_gear_scenario.cc
  tasks/optimizers/speed_data_optimizer/scenarios/forward_gear/lane_keep/lane_keep_scenario.cc
  tasks/optimizers/speed_data_optimizer/scenarios/forward_gear/lane_change/lane_change_scenario.cc
  tasks/optimizers/speed_data_optimizer/scenarios/forward_gear/lane_merge/lane_merge_scenario.cc
  tasks/optimizers/speed_data_optimizer/scenarios/forward_gear/lane_continue/lane_continue_scenario.cc
  tasks/optimizers/speed_data_optimizer/scenarios/reverse_gear/reverse_gear_scenario.cc
  tasks/optimizers/speed_data_optimizer/scenarios/speed_scenario_factory.cc

  tasks/optimizers/speed_data_optimizer/scenarios/speed_scenario_manager.cc

  tasks/optimizers/speed_data_optimizer/generators/speed_data_generator.cc
  tasks/optimizers/speed_data_optimizer/generators/vt_sample/curves/quadratic_vt_curve.cc
  tasks/optimizers/speed_data_optimizer/generators/vt_sample/curves/cubic_vt_curve.cc
  tasks/optimizers/speed_data_optimizer/generators/vt_sample/curves/quartic_vt_curve.cc
  tasks/optimizers/speed_data_optimizer/generators/vt_sample/samplers/vt_sampler.cc
  tasks/optimizers/speed_data_optimizer/generators/vt_sample/samplers/forward_vt_sampler.cc
  tasks/optimizers/speed_data_optimizer/generators/vt_sample/samplers/reverse_vt_sampler.cc
  tasks/optimizers/speed_data_optimizer/generators/vt_sample/rules/vt_sample_rule.cc
  tasks/optimizers/speed_data_optimizer/generators/vt_sample/rules/base_limit_rule.cc
  tasks/optimizers/speed_data_optimizer/generators/vt_sample/rules/stop_comfort_rule.cc
  tasks/optimizers/speed_data_optimizer/generators/vt_sample/forward_vt_sample_generator.cc
  tasks/optimizers/speed_data_optimizer/generators/vt_sample/reverse_vt_sample_generator.cc
  tasks/optimizers/speed_data_optimizer/generators/forward_gear_speed_data_generator.cc
  tasks/optimizers/speed_data_optimizer/generators/reverse_gear_speed_data_generator.cc
  tasks/optimizers/speed_data_optimizer/generators/speed_data_generator_factory.cc
  tasks/optimizers/speed_data_optimizer/speed_data_optimizer.cc

  tasks/optimizers/ssc_trajectory_optimizer/ssc_trajectory_optimizer.cc
  tasks/optimizers/ssc_trajectory_optimizer/ssc_map/semantics.cc
  tasks/optimizers/ssc_trajectory_optimizer/ssc_map/ssc_map.cc
  tasks/optimizers/ssc_trajectory_optimizer/ssc_map/ssc_map.cc
  tasks/optimizers/ssc_trajectory_optimizer/ssc_trajectory_optimizer.cc
  tasks/optimizers/ssc_trajectory_optimizer/bezier_spline_generator/bezier_spline_generator.cc

  tasks/optimizers/joint_optimizer/common/polyline_boundary.cc

  tasks/optimizers/joint_optimizer/cilqr_solver/al_cost.cc
  tasks/optimizers/joint_optimizer/cilqr_solver/ilqr_problem.cc
  tasks/optimizers/joint_optimizer/cilqr_solver/cilqr_problem.cc

  tasks/optimizers/joint_optimizer/joint_limits/speed_limit.cc
  tasks/optimizers/joint_optimizer/joint_limits/accel_limit.cc
  tasks/optimizers/joint_optimizer/joint_limits/jerk_limit.cc
  tasks/optimizers/joint_optimizer/joint_limits/steering_wheel_limit.cc
  tasks/optimizers/joint_optimizer/joint_limits/limit_manager.cc

  tasks/optimizers/joint_optimizer/joint_constraints/cone_constraints/bound_collision_constraint.cc
  tasks/optimizers/joint_optimizer/joint_constraints/cone_constraints/obstacle_collision_constraint.cc
  tasks/optimizers/joint_optimizer/joint_constraints/cone_constraints/state_constraint.cc
  tasks/optimizers/joint_optimizer/joint_constraints/cone_constraints/control_constraint.cc

  tasks/optimizers/joint_optimizer/joint_constraints/indicator_constraints/speed_constraint.cc
  tasks/optimizers/joint_optimizer/joint_constraints/indicator_constraints/kappa_constraint.cc
  tasks/optimizers/joint_optimizer/joint_constraints/indicator_constraints/acc_constraint.cc
  tasks/optimizers/joint_optimizer/joint_constraints/indicator_constraints/dkappa_constraint.cc
  tasks/optimizers/joint_optimizer/joint_constraints/indicator_constraints/jerk_constraint.cc
  tasks/optimizers/joint_optimizer/joint_constraints/indicator_constraints/bound_collision_constraint.cc

  tasks/optimizers/joint_optimizer/joint_constraints/constraint_manager.cc

  tasks/optimizers/joint_optimizer/joint_costs/state_control_cost.cc
  tasks/optimizers/joint_optimizer/joint_costs/bound_ttc_cost.cc
  tasks/optimizers/joint_optimizer/joint_costs/obstacle_ttc_cost.cc
  tasks/optimizers/joint_optimizer/joint_costs/cost_manager.cc
  tasks/optimizers/joint_optimizer/vehicle_models/linear_bicycle_dynamic.cc
  tasks/optimizers/joint_optimizer/vehicle_models/nonlinear_bicycle_dynamic.cc

  tasks/optimizers/joint_optimizer/joint_scenarios/joint_stage.cc
  tasks/optimizers/joint_optimizer/joint_scenarios/forward_gear/lane_keep/lane_keep_joint_stage.cc
  tasks/optimizers/joint_optimizer/joint_scenarios/forward_gear/lane_change/lane_change_ongoing_joint_stage.cc
  tasks/optimizers/joint_optimizer/joint_scenarios/forward_gear/lane_change/lane_change_prefinish_joint_stage.cc
  tasks/optimizers/joint_optimizer/joint_scenarios/forward_gear/lane_merge/lane_merge_cruise_joint_stage.cc
  tasks/optimizers/joint_optimizer/joint_scenarios/forward_gear/lane_merge/lane_merge_stop_joint_stage.cc
  tasks/optimizers/joint_optimizer/joint_scenarios/forward_gear/lane_continue/lane_continue_joint_stage.cc
  tasks/optimizers/joint_optimizer/joint_scenarios/reverse_gear/reverse_gear_joint_stage.cc
  tasks/optimizers/joint_optimizer/joint_scenarios/joint_stage_factory.cc

  tasks/optimizers/joint_optimizer/joint_scenarios/joint_scenario.cc
  tasks/optimizers/joint_optimizer/joint_scenarios/forward_gear/forward_gear_joint_scenario.cc
  tasks/optimizers/joint_optimizer/joint_scenarios/forward_gear/lane_keep/lane_keep_joint_scenario.cc
  tasks/optimizers/joint_optimizer/joint_scenarios/forward_gear/lane_change/lane_change_joint_scenario.cc
  tasks/optimizers/joint_optimizer/joint_scenarios/forward_gear/lane_merge/lane_merge_joint_scenario.cc
  tasks/optimizers/joint_optimizer/joint_scenarios/forward_gear/lane_continue/lane_continue_joint_scenario.cc
  tasks/optimizers/joint_optimizer/joint_scenarios/reverse_gear/reverse_gear_joint_scenario.cc
  tasks/optimizers/joint_optimizer/joint_scenarios/joint_scenario_factory.cc

  tasks/optimizers/joint_optimizer/joint_scenarios/joint_scenario_manager.cc
  tasks/optimizers/joint_optimizer/rough_planner/rough_path_planner/pure_pursuit_planner.cc
  tasks/optimizers/joint_optimizer/rough_planner/rough_path_planner/rough_path_planner.cc
  tasks/optimizers/joint_optimizer/rough_planner/rough_speed_planner/rough_speed_planner.cc
  tasks/optimizers/joint_optimizer/rough_planner/rough_speed_planner/follow_model.cc
  tasks/optimizers/joint_optimizer/rough_planner/rough_planner.cc
  tasks/optimizers/joint_optimizer/joint_optimizer.cc

  tasks/deciders/joint_post_decider/joint_post_decider.cc

  tasks/optimizers/open_space_path_generation/open_space_path_generator.cc
  tasks/optimizers/open_space_path_generation/open_space_path_smoother.cc
  tasks/optimizers/open_space_path_generation/open_space_path_provider.cc
  tasks/optimizers/open_space_path_partition/open_space_path_partition.cc
  tasks/optimizers/open_space_speed_optimizer/open_space_speed_optimizer.cc
  tasks/optimizers/open_space_speed_optimizer/st_sample_curves.cc
  tasks/optimizers/open_space_speed_optimizer/st_sample_cost.cc
  tasks/optimizers/open_space_speed_optimizer/path_handle.cc
  tasks/optimizers/open_space_straight_path/open_space_straight_path_provider.cc
  tasks/optimizers/guide_line_path_optimizer/guide_line_path_optimizer.cc
  tasks/optimizers/guide_line_path_optimizer/cilqr_path_smoother.cc
  tasks/optimizers/guide_line_path_optimizer/path_dynamics.cc
  tasks/deciders/guide_line_bounds_decider/guide_line_bounds_decider.cc
  tasks/deciders/obstacle_filter_decider/obstacle_filter_decider.cc
  tasks/deciders/precise_path_bounds_decider/precise_path_bounds_decider.cc
  tasks/deciders/astar_path_decider/astar_path_decider.cc
  tasks/deciders/astar_path_decider/astar_grid_search.cc
  tasks/deciders/path_reuse_decider/path_reuse_decider.cc
  tasks/deciders/avoid_alongside_decider/avoid_alongside_decider.cc
  tasks/deciders/path_lane_borrow_decider/path_lane_borrow_decider.cc
  tasks/deciders/path_assessment_decider/path_assessment_decider.cc
  tasks/deciders/path_bounds_decider/path_bounds_decider.cc
  tasks/deciders/path_bounds_decider/util/path_info.cc
  tasks/deciders/path_bounds_decider/obs_processor/dynamic_buffer_processor.cc
  tasks/deciders/path_bounds_decider/obs_processor/obs_static_processor.cc
  tasks/deciders/path_bounds_decider/obs_processor/obs_dynamic_processor.cc
  tasks/deciders/path_bounds_decider/obs_processor/obs_processor.cc
  tasks/deciders/path_bounds_decider/bound_processor/process_bound.cc
  tasks/deciders/path_bounds_decider/bound_processor/lane_keep_bound_processor.cc
  tasks/deciders/path_bounds_decider/bound_processor/lane_change_bound_processor.cc
  tasks/deciders/path_bounds_decider/bound_processor/fallback_bound_processor.cc
  tasks/deciders/path_bounds_decider/bound_processor/pullover_bound_processor.cc
  tasks/deciders/path_bounds_decider/bound_processor/bound_processor.cc
  tasks/deciders/path_bounds_decider/bound_processor/lane_change_heuristic_generator.cc
  tasks/deciders/lane_change_decider/lane_change_decider.cc
  tasks/deciders/obstacles_decider/conflict_resolution/conflict_resolution.cc
  tasks/deciders/obstacles_decider/conflict_resolution/behavior_conflict_resolution.cc
  tasks/deciders/obstacles_decider/evaluator/trajectory_evaluator.cc
  tasks/deciders/obstacles_decider/evaluator/behavior_evaluator.cc
  tasks/deciders/obstacles_decider/obstacle_selector/obstacles_info_updater.cc
  tasks/deciders/obstacles_decider/obstacle_selector/obstacle_selector.cc
  tasks/deciders/obstacles_decider/obstacle_selector/cutin_intention_estimation.cc
  tasks/deciders/obstacles_decider/sampler/at_sampler.cc
  tasks/deciders/obstacles_decider/sampler/sl_curve.cc
  tasks/deciders/obstacles_decider/sampler/sl_sampler.cc
  tasks/deciders/obstacles_decider/sampler/trajectory_generator.cc
  tasks/deciders/obstacles_decider/obstacles_decider.cc
  tasks/deciders/speed_bounds_decider/speed_bounds_decider.cc
  tasks/deciders/speed_bounds_decider/speed_limit_decider.cc
  tasks/deciders/speed_bounds_decider/st_boundary_mapper.cc
  tasks/deciders/obstacle_game_decider/obstacle_game_decider.cc
  
  tasks/deciders/speed_decider/speed_decider.cc

  tasks/deciders/rule_based_stop_decider/lane_change_urgency.cc
  tasks/deciders/rule_based_stop_decider/rule_based_stop_decider.cc
  tasks/deciders/st_bounds_decider/st_bounds_decider.cc
  tasks/deciders/st_bounds_decider/st_guide_line.cc
  tasks/deciders/st_bounds_decider/st_driving_limits.cc
  tasks/deciders/st_bounds_decider/st_obstacles_processor.cc
  tasks/deciders/st_bounds_decider/obstacle_intention_processor.cc
  tasks/deciders/st_bounds_decider/slt_obstacles_processor.cc
  tasks/deciders/st_bounds_decider/obstacle_filter.cc
  tasks/deciders/decider.cc
  tasks/deciders/path_decider/path_decider.cc
  tasks/deciders/utils/path_decider_obstacle_utils.cc
  tasks/deciders/creep_decider/creep_decider.cc
  tasks/deciders/open_space_decider/open_space_roi_decider.cc
  tasks/deciders/open_space_decider/open_space_obstacle.cc
  tasks/deciders/open_space_decider/open_space_fine_tuning.cc
  tasks/task.cc
  tasks/utils/st_gap_estimator.cc
  traffic_rules/traffic_light.cc
  traffic_rules/yield_sign.cc
  traffic_rules/destination.cc
  traffic_rules/reference_line_end.cc
  traffic_rules/backside_vehicle.cc
  traffic_rules/rerouting.cc
  traffic_rules/stop_sign.cc
  traffic_rules/traffic_decider.cc
  traffic_rules/keep_clear.cc
  traffic_rules/pnc_junction.cc
  traffic_rules/crosswalk.cc
  traffic_rules/odd.cc
  traffic_rules/map_split.cc
  localview/lane_line_builder/navigation_hdmap_lane_line/navigation_hdmap_lane_line.cc
  localview/lane_line_builder/navigation_hdmap_lane_line/navigation_lane_map_creator.cc
  localview/lane_line_builder/navigation_hdmap_lane_line/navigation_lane_path_generator.cc
  localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/lanemarker_new_decider.cc
  localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/coortrans_and_copy.cc
  localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/lanemarker_change_decider.cc
  localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/lane_width_predictor.cc
  localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/lanemarker_filter.cc
  localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/reset_and_quality_monitor.cc
  localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/decider_data.cc
  localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/lane_detm_reset.cc
  localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/lane_width_and_quality_monitor.cc
  localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/lane_change_observer.cc
  localview/lane_line_builder/navigation_hdmap_lane_line/navigation_lanecentral_constructor.cc
  localview/lane_line_builder/real_hdmap_lane_line/real_hdmap_lane_line.cc
  localview/lane_line_builder/real_hdmap_lane_line/inside_active_condition.cc
  localview/lane_line_builder/real_hdmap_lane_line/failure_location_checker.cc
  localview/lane_line_builder/real_hdmap_lane_line/double_rise_decider.cc
  localview/lane_line_builder/local_hdmap_lane_line/local_hdmap_lane_line.cc
  localview/lane_line_builder/map_fusion_lane_line/map_fusion_lane_line.cc
  localview/lane_line_builder/lane_line_base.cc
  localview/lane_line_builder/adaptive_cruise_lane_line/adaptive_cruise_lane_line.cc
  localview/lane_line_builder/ldp_ldw_creater/ldp_ldw_core.cc
  localview/lane_line_builder/ldp_ldw_creater/ldw_creater/ldw_creater.cc
  localview/lane_line_builder/ldp_ldw_creater/ldp_creater/ldp_creater.cc
  localview/lane_line_builder/ldp_ldw_creater/lane_line_detection/lane_line_creater.cc
  localview/lane_line_builder/obstacle_following_lane_line/missile_mode/missile_mode_lane_line.cc
  localview/lane_line_builder/obstacle_following_lane_line/missile_mode/vehicle_state_decider.cc
  localview/lane_line_builder/obstacle_following_lane_line/missile_mode/obstacle_decider.cc
  localview/lane_line_builder/obstacle_following_lane_line/missile_mode/points_filter.cc
  localview/lane_line_builder/obstacle_following_lane_line/missile_mode/obs_history_points_generator.cc
  localview/lane_line_builder/obstacle_following_lane_line/missile_mode/obstacles_state.cc
  localview/perception_lane_line_state/perception_lane_line_state.cc
  localview/parking_lane_line_state/parking_lane_line_state.cc
  localview/hdmap_lane_line_state/hdmap_lane_line_state.cc
  localview/hdmap_perception_fused_state/hdmap_perception_fused_state.cc
  localview/history_trace_lane_line_state/history_trace_lane_line_state.cc
  localview/hdmap_avp_state/hdmap_avp_state.cc
  localview/state_machine/initial_state.cc
  localview/state_machine/local_view_state_machine.cc
  localview/localview_comdata_manager.cc
  localview/lane_line_builder/adaptive_cruise_lane_line/cruise_target_select.cc
  localview/lane_line_builder/adaptive_cruise_lane_line/target_overlap.cc
  visitors/visitor.cc
  visitors/visitor_helper.cc
  visitors/stitcher_visitor.cc
  visitors/lane_change_intent_visitor.cc
  trigger/event_collect.cc
  trigger/fault_collect.cc
  trigger/metric_collect.cc
  hmi/can_hmi.cc
  hmi/eth_hmi.cc
  hmi/function_manager.cc
  hmi/lon_hmi/spd_adapt/spd_adapt.cc
  hmi/lon_hmi/speed_convertor/speed_convertor.cc
  hmi/lon_hmi/lon_hmi.cc
  hmi/lon_hmi/overtaking_assistance/overtaking_assistance.cc
  zmq2receiver/zmq2receiver.cc
  tasks/deciders/path_bounds_decider/util/py_plot.cpp
)


target_link_libraries(
  ${PROJECT_NAME}
  globalproto
  europa_prediction
  europa_warning
  europa_routing
  europa_self_simulator
  ${MapManage_LIBRARIES}
  ${YAMLCPP_LIBRARIES}
  ${ABSL_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  ${BOOST_LIBRARIES}
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OSQP_LIBRARIES}
  ${IPOPT_LIBRARIES}
  ${CppAD_LIBRARIES}
  ${OPENCV_LIBRARIES})
if(PNC_SELF_LIB)
  target_link_libraries(
      ${PROJECT_NAME}
      europa_self_hdmap
      europa_self_common)
else()
  target_link_libraries(
    ${PROJECT_NAME}
    europa_hdmap
    europa_common)
endif(PNC_SELF_LIB)
set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "-Wl,--disable-new-dtags")
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION .)
install(DIRECTORY conf DESTINATION .)

if(PLATFORM MATCHES "mdc")
  include(middleware/cmake/mdc.cmake)
endif()

if(PLATFORM MATCHES "orin")
  include(middleware/cmake/orin.cmake)
endif()

if(PLATFORM MATCHES "x86")
  include(middleware/cmake/cyber.cmake)
  if((FOR_BAIDU_SIMULATION MATCHES 0) AND (COMPILE_ALL MATCHES 0) )
    include(middleware/cmake/orin.cmake)
  endif()
endif(PLATFORM MATCHES "x86")

#add_subdirectory(proto)
add_subdirectory(routing)
add_subdirectory(warning)
add_subdirectory(self_simulator)
add_subdirectory(prediction)