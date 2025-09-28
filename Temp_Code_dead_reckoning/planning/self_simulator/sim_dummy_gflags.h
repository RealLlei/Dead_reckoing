
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

#include "gflags/gflags.h"
DECLARE_bool(enable_sim_dummy_obs);

DECLARE_double(left_lane_near_obstacle_speed_km_h);
DECLARE_double(left_lane_near_obstacle_display_dis);

DECLARE_double(left_lane_far_obstacle_display_dis);
DECLARE_double(left_lane_far_obstacle_speed_km_h);

DECLARE_double(front_obstacle_speed_km_h);
DECLARE_double(front_obstacle_display_dis);

DECLARE_double(right_lane_far_obstacle_display_dis);
DECLARE_double(right_lane_far_obstacle_speed_km_h);

DECLARE_double(right_lane_near_obstacle_display_dis);
DECLARE_double(right_lane_near_obstacle_speed_km_h);
DECLARE_double(random_obstacle_min_speed_km_h);
DECLARE_double(random_obstacle_max_speed_km_h);

DECLARE_int32(obstacle_number);
DECLARE_bool(only_enable_sim_adc_side_dummy_obs);
