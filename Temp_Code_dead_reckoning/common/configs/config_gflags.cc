
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
#include "common/configs/config_gflags.h"

#include "gflags/gflags.h"

DEFINE_string(map_dir, "data/map/demo",
              "Directory which contains a group of related maps.");
DEFINE_int32(local_utm_zone_id, 100, "UTM zone id.");

DEFINE_string(base_map_filename, "base_map.bin|base_map.xml|base_map.txt",
              "Base map files in the map_dir, search in order.");
DEFINE_string(sim_map_filename, "sim_map.bin|sim_map.txt",
              "Simulation map files in the map_dir, search in order.");
DEFINE_string(routing_map_filename, "routing_map.bin|routing_map.txt",
              "Routing map files in the map_dir, search in order.");
DEFINE_string(end_way_point_filename, "default_end_way_point.txt",
              "End way point of the map, will be sent in RoutingRequest.");
DEFINE_string(default_routing_filename, "default_cycle_routing.txt",
              "Default cycle routing of the map, will be sent in Task to Task "
              "Manager Module.");

DEFINE_string(vehicle_config_path, "conf/vehicle_param.pb.txt",
              "the file path of vehicle config file");

DEFINE_string(vehicle_config_path_EP41_CTC, "conf/vehicle_param_EP41CTC.pb.txt",
              "the file path of vehicle config file for EP41 CTC project");

DEFINE_string(vehicle_model_config_filename, "conf/vehicle_model_config.pb.txt",
              "the file path of vehicle model config file");

// DEFINE_bool(use_navigation_mode, false,
//             "Use relative position in navigation mode");
DEFINE_bool(use_hdmap_mode, true, "Use map in hdmap mode");
// fct_mode,only in hdmap mode whether hdmap success or faild
DEFINE_bool(use_new_lanemarker_decider, true, "use new lanemarker decider");
DEFINE_bool(not_using_nolane, true, "not_using_nolane");
DEFINE_bool(not_using_cruise, true, "not_suing_cruise");
DEFINE_bool(force_NNP_mode, false, "force nnp mode");
DEFINE_bool(global_enable_nnp, false, "global_enable_nnp");
DEFINE_bool(enable_ncp, false, "enable_ncp");
DEFINE_int32(lane_line_decider_debug_flag, 1, "lane_line_decider_debug_flag");
DEFINE_bool(nolane_mode_first_for_simulation, false,
            "in perception mode,nolane mode first");
DEFINE_bool(use_fct_mode, false, "Use map in fct mode");

DEFINE_bool(enable_hdmap_nnp_mode, true, "enable_hdmap_nnp_mode");
DEFINE_bool(enable_hdmap_ncp_mode, true, "enable_hdmap_ncp_mode");
DEFINE_bool(enable_laneline_mode, true, "enable_laneline_mode");
DEFINE_bool(enable_cruise_mode, true, "enable_cruise_mode");
DEFINE_bool(enable_mapfusion_mode, true, "enable_mapfusion_mode");
DEFINE_double(half_vehicle_width, 1.05, "half vehicle width");

DEFINE_double(trajectory_time_length, 7.0,
              "look forward time times adc speed to calculate this distance "
              "when creating reference line from routing");

DEFINE_double(reference_line_extend_time, 0.25, "reference line extend time");

DEFINE_bool(use_sim_time, false, "Use bag time in mock time mode.");

DEFINE_bool(reverse_heading_vehicle_state, false,
            "test flag for reverse driving.");

DEFINE_bool(state_transform_to_com_reverse, false,
            "Enable vehicle states coordinate transformation from center of "
            "rear-axis to center of mass, during reverse driving");
DEFINE_bool(state_transform_to_com_drive, true,
            "Enable vehicle states coordinate transformation from center of "
            "rear-axis to center of mass, during forward driving");
DEFINE_bool(multithread_run, false,
            "multi-thread run flag mainly used by simulation");

// Thread Pool
DEFINE_int32(threadpool_future_wait_timeout, 5,
             "threadpool future wait timeout");
DEFINE_int32(threadpool_level0_thread_num, 10,
             "threadpool level0 thread number");
DEFINE_int32(threadpool_level1_thread_num, 10,
             "threadpool level1 thread number");
DEFINE_int32(threadpool_level2_thread_num, 10,
             "threadpool level2 thread number");
DEFINE_int32(threadpool_queue_multiplier, 2, "threadpool queue multiplier");
DEFINE_int32(threadpool_queue_num, 20, "threadpool queue number");

// NETA steer offset
DEFINE_bool(is_nnp_mode, true, " is_nnp_mode or avp ");
DEFINE_bool(enable_viz, true, " enable_viz");

DEFINE_bool(enable_dreamview_to_planning_zmq, true,
            " enable_dreamview_to_planning_viz");
DEFINE_bool(enable_dreamview_send_fct, true, "enable_dreamview_send_fct");

DEFINE_bool(enable_recv_local_zmq, false, "use when parse_bag play bag");

// zmq config
DEFINE_int32(zmq_data_port, 10000,
             "port of zmq data for hmi, dreamview and parse_rtf_bag");
DEFINE_int32(zmq_perception_data_port, 9800,
             "port of zmq data for perception, dreamview and parse_rtf_bag");
DEFINE_int32(zmq_data_localization_port, 10003,
             "port of zmq receive localization data alone");
DEFINE_string(zmq_publisher_ip, "tcp://127.0.0.1", "ip of publisher");

DEFINE_int32(zmq_data_dreamview_port, 10002,
             "port of zmq data for hmi, dreamview and parse_rtf_bag");
DEFINE_string(zmq_publisher_dreamview_ip, "tcp://127.0.0.1", "ip of publisher");

DEFINE_bool(use_ehp, false, "use ehp");
DEFINE_bool(use_ehp_odd_to_stop, true, "use odd to stop");

DEFINE_bool(export_map_in_planning, false, "export map in planning");
DEFINE_bool(is_record_replay, false, "is  record play mode");
DEFINE_bool(is_mcap_replay, false, "is  mcap replayy mode");

DEFINE_bool(enable_from_file_response, false, "from file get response");
DEFINE_bool(enable_random_response, false, "routing random");

DEFINE_bool(show_cam_data, true, "show cam data");
DEFINE_bool(export_secret, false, "export map and routing");
DEFINE_bool(zmq_replay_data, false, "only replay mdc data");

// map config
// look backward distance
DEFINE_double(look_backward_distance, 130,
              "look backward this distance when creating reference line from "
              "routing");
DEFINE_double(
    look_backward_distance_perception, 30,
    "look backward perception distance when creating reference line from "
    "routing");
DEFINE_double(look_backward_distance_avp, 10,
              "look backward this distance when creating reference line from "
              "routing for avp");
DEFINE_double(look_backward_distance_avp_history, 60,
              "look backward this distance when creating reference line from "
              "routing for avp history");
DEFINE_double(look_backward_smooth_distance, 20,
              "look backward smooth distance when smoothing reference line");
DEFINE_double(
    look_backward_smooth_distance_perception, 15,
    "look backward smooth perception distance when smoothing reference line");
DEFINE_double(look_backward_smooth_distance_avp, 10,
              "look backward smooth distance when smoothing reference line");
DEFINE_double(look_backward_smooth_distance_avp_history, 60,
              "look backward smooth distance when smoothing reference line");
// look forward distance
DEFINE_double(look_forward_long_distance, 350,  // 180 km/h  view 7s
              "look forward this distance when creating reference line from "
              "routing when ADC is quick");
DEFINE_double(look_forward_short_distance, 100,  // 80km/h  view 7s
              "short look forward this distance when creating reference line "
              "from routing when ADC is slow");
DEFINE_double(look_forward_distance_avp, 60,
              "look forward distance when creating reference line from routing "
              "for avp");
DEFINE_double(look_forward_distance_avp_history, 60,
              "look forward distance when creating reference line from routing "
              "for avp history");
DEFINE_bool(forbid_lane_change_in_eye_mode, true,
            " forbid lane change in eye mode");

DEFINE_bool(forbid_efficiency_lane_change_in_eye_mode, true,
            " forbid efficiency lane change in eye mode");
DEFINE_bool(enable_control_zmq, true, " enable_viz");
DEFINE_bool(enable_struct_pb_debug, true,
            " enable to print struct2pb debug log");

DEFINE_bool(enable_planning_injector_obs, false,
            "enable_planning_injector_obs");
DEFINE_bool(enable_planning_self_simulator, false,
            "enable_planning_self_simulator");

DEFINE_string(test_avp_hdmap_data_file,
              "data/map/avp_map/test_data/map_data.bin",
              "path of avp hdmap data file");
DEFINE_bool(enable_hdmap_avp_state_sim, false, "enable hdmap_avp_state sim.");
DEFINE_bool(enable_history_trace_state_sim, false,
            "enable history_trace_state sim.");
DEFINE_bool(enable_parking_state_sim, false, "enable parking_state sim.");
DEFINE_bool(enable_ignore_slot_status, false, "enable ignore slot status.");
DEFINE_bool(enable_park_with_one_slot, false, "enable parking with only slot.");

DEFINE_bool(is_baidu_log2world_sim, false, "baidu log2word sim");
DEFINE_bool(publish_estop, false, "publish estop decision in planning");

// ehp monitor
DEFINE_int32(ehp_monitor, 0, "0: all input; 1: only loc; 2: loc+planning");

// map layer
DEFINE_bool(map_layer_enable, false, "enable map_layer function");
DEFINE_int32(distance_boundary_maximum, 400000,
             "(cm)construct quad tree max distance");
DEFINE_int32(distance_boundary_start_build, 300000,
             "(cm)construct quad tree max distance");
DEFINE_int32(distance_boundary_use_new, 200000,
             "(cm)construct quad tree max distance");
DEFINE_string(map_layer_direction, "data/map/map_layer", "map_layer_direction");

// ehp/ehr routing
DEFINE_bool(enable_ehp_routing, false, "using sdk navigation");

DEFINE_string(ehp_log_root_dir, "/opt/usr/log/hd_log", "ehp log path");

DEFINE_bool(use_config_cruise_speed, false, "use config cruise speed");

DEFINE_bool(use_config_change_nnp_or_avp, false,
            "use config dynamic change nnp or avp");

DEFINE_int32(input_data_time_out_period_num, 3,
             "input data time out period num");

DEFINE_bool(global_enable_warning_mem, false, "global enable warning mem");
DEFINE_bool(record_realtime_update_ehr_map_date, true,
            "record_realtime_update_ehr_map_date");
DEFINE_double(log_v3map_data_interval_time, 10.0,
              "log_v3map_data_interval_time.");
DEFINE_bool(using_record_ehp_data, false, "using_record_ehp_data");
DEFINE_double(cruise_map_speed_limit, 5.55, "NTP cruise map speed limit");

// fault and health management
DEFINE_bool(enable_planning_trigger, true, "enable event and fault trigger");
DEFINE_bool(enable_warning_fault_process, true, "enable warning fault process");
DEFINE_bool(enable_event_collect, true, "enable event collect");
DEFINE_bool(enable_fault_collect, true, "enable fault collect");
DEFINE_string(warning_inhibition_fault_map_file,
              "conf/planning/fm/warning_inhibition_fault_map.pb.txt",
              "warning functions inhibition fault map file");
DEFINE_string(planning_fm_register_config_file,
              "/opt/app/1/conf/planning/fm/planning_fm_config.yaml",
              "fm fault register config file");
DEFINE_string(orin_warning_inhibition_fault_map_file,
              "conf/planning/fm/orin/warning_inhibition_fault_map.pb.txt",
              "warning functions inhibition fault map file");

// localization
DEFINE_string(localization_main_module, "fc",
              "localization trajectory for car to move");
DEFINE_bool(localization_data_independent, false,
            "send map and trajectory independently without other process");
DEFINE_bool(decode_loc_jpeg_front, false,
            "decode jpeg format front camera image sent by localization");

DEFINE_bool(use_carla_read_map_simulation, false,
            "Enable carla simulation for apa.");

// perception
DEFINE_double(perception_compensation, 0.2, " perception latency compensation");

DEFINE_string(avp_map_path_file, "/mnt/simulation/records/map/",
              "avp map file diretory.");
DEFINE_double(hdmap_avp_path_extend_buffer, 20, "hdmap avp path extend buffer");
DEFINE_bool(use_lapa_read_map_simulation, false, "enable lapa simulation");
DEFINE_double(hdmap_avp_boundary_klimit, 1, "hdmap_avp_boundary_klimit");
DEFINE_bool(skip_static_box_simulation, false,
            "takeout static obstacle box in simulation");
DEFINE_bool(skip_freespace_simulation, false,
            "takeout freespace info in simulation");

// amap simulator
DEFINE_bool(use_amap_simulation, false, "enable amap simulation");
DEFINE_bool(map_extract_by_file, true, "add layer points by the loc file");
DEFINE_bool(local_simulation_when_add_layer, false,
            "enable local simulation when add layer points");
DEFINE_bool(enable_lapa_simulation, false, "switch on lapa simulation");
DEFINE_bool(use_dv_traffic_light, false, "use dv traffci light");
DEFINE_bool(use_dv_routing, false, "use dv traffci routing");
DEFINE_bool(map_data_to_hmi, false, "send map data to hmi");

// mapservice mode
DEFINE_int32(map_service_mode, 0, "0: map.bin; 1: ehp; 2: map api");

DEFINE_int32(location_vector_count, 5, "定位队列个数");
// NOLINTEND
