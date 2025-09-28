
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

#include "planning/self_simulator/sim_dummy_gflags.h"
DEFINE_bool(enable_sim_dummy_obs, true, "enable sim dummy obs");

DEFINE_double(left_lane_near_obstacle_speed_km_h, 50, "speed km/h");
DEFINE_double(left_lane_near_obstacle_display_dis, 40, "dis");

DEFINE_double(left_lane_far_obstacle_display_dis, 40, "dis");
DEFINE_double(left_lane_far_obstacle_speed_km_h, 50, "speed km/h");

DEFINE_double(front_obstacle_speed_km_h, 50, "speed km/h");
DEFINE_double(front_obstacle_display_dis, 40, "dis");

DEFINE_double(right_lane_far_obstacle_display_dis, 40, "dis");
DEFINE_double(right_lane_far_obstacle_speed_km_h, 50, "speed km/h");

DEFINE_double(right_lane_near_obstacle_display_dis, 40, "dis");
DEFINE_double(right_lane_near_obstacle_speed_km_h, 50, "speed km/h");

DEFINE_double(random_obstacle_min_speed_km_h, 70,
              "random_obstacle_min_speed_km_h");
DEFINE_double(random_obstacle_max_speed_km_h, 120,
              "random_obstacle_max_speed_km_h");
DEFINE_int32(obstacle_number, 30, "obstacle number");

DEFINE_bool(only_enable_sim_adc_side_dummy_obs, false,
            "enable sim adc side dummy obs");
