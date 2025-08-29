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

#pragma once

#include "gflags/gflags.h"

DECLARE_bool(enable_publish_chassis);
DECLARE_string(dead_reckoning_conf_file);
DECLARE_double(yaw_rate_senesor_offset);
DECLARE_bool(enable_auto_angle_rate_bias);
DECLARE_double(auto_angle_rate_bias_time);
DECLARE_double(default_angle_rate_bias_value);
DECLARE_double(default_standstill_value);

DECLARE_double(dr_start_point_x);
DECLARE_double(dr_start_point_y);
DECLARE_double(dr_start_point_heading);
