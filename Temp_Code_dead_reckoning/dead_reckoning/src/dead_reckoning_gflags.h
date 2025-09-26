/*
 * @LastEditors: Liu Bei
 */
/******************************************************************************
 * Copyright 2017 The Magna Authors. All Rights Reserved.
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

/*
1. dead_reckoning_gflags.h：声明参数（DECLARE_xxx）
头文件中通过 DECLARE_xxx 系列宏（如 DECLARE_bool、DECLARE_string、DECLARE_double）声明了航位推算模块所需的参数。
这些声明的作用是：让其他包含该头文件的代码（如模块核心逻辑、组件初始化代码等）能够 “看到” 这些参数，从而在编译时知道参数的存在，
允许在代码中通过 FLAGS_参数名 的形式使用这些参数。
例如：在 dead_reckoning_core.cc 的 Init() 函数中，通过 FLAGS_dead_reckoning_conf_file 加载航位推算的配置文件（dead_reckoning_conf.pb.txt）
*/


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
