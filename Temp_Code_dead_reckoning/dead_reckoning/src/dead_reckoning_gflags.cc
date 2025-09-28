/*
 * @LastEditors: Liu Bei
 */
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

#include "dead_reckoning/src/dead_reckoning_gflags.h"

DEFINE_double(yaw_rate_senesor_offset, 0.086, "yaw rate sensor offset");

DEFINE_string(dead_reckoning_conf_file,
              "conf/dead_reckoning/dead_reckoning_conf.pb.txt",
              "default dead reckoning conf data file");
DEFINE_bool(is_sim_mode, true, "sim mode no publish chassis");
DEFINE_bool(enable_auto_angle_rate_bias, true, "enable auto angle rate bias");
DEFINE_double(auto_angle_rate_bias_time, 300.0, "auto angle rate bias time");
DEFINE_double(default_angle_rate_bias_value, -0.0007689,
              "default angle rate bias value");
DEFINE_double(default_standstill_value, 0.0001, "default_standstill_value");
DEFINE_double(dr_start_point_x, 0.0, "start point x in dead reckoning");
DEFINE_double(dr_start_point_y, 0.0, "start point y in dead reckoning");
DEFINE_double(dr_start_point_heading, 0.0,
              "start point theta in dead reckoning");
