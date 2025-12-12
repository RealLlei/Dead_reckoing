// Minimal gflags/constants placeholders used by the cleaned core
#pragma once

namespace Magna {
namespace dead_reckoning {
// nothing here; keep for include compatibility
}
}

// commonly used constants
// Avoid redefining DEG_2_RAD if already defined in other headers
#ifndef DEG_2_RAD
#define DEG_2_RAD 0.017453292519943295
#endif
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

// gflags is not available in this trimmed build; provide extern declarations
// that match the simple DEFINE_* expansions in compat.h
extern bool FLAGS_enable_publish_chassis;
extern const char* FLAGS_dead_reckoning_conf_file;
extern double FLAGS_yaw_rate_senesor_offset;
extern bool FLAGS_enable_auto_angle_rate_bias;
// The core header may define constexpr defaults; only declare externs if not already defined.
#ifndef FLAGS_auto_angle_rate_bias_time
extern double FLAGS_auto_angle_rate_bias_time;
#endif
#ifndef FLAGS_default_angle_rate_bias_value
extern double FLAGS_default_angle_rate_bias_value;
#endif
extern double FLAGS_default_standstill_value;

extern double FLAGS_dr_start_point_x;
extern double FLAGS_dr_start_point_y;
extern double FLAGS_dr_start_point_heading;
