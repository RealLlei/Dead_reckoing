/*
 * Copyright (c) TL auto Co., Ltd. 2021-2022. All rights reserved.
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "common/configs/vehicle_config_helper.h"
#include "common/math/vec2d.h"
#include "common/status/status.h"
#include "common/util/message_util.h"
#include "common/util/util.h"
#include "map/hdmap/hdmap_util.h"
#include "planning/common/planning_gflags.h"
#include "planning/localview/local_view.h"
#include "planning/proto/navigation_hdmap_config.pb.h"

namespace TL {
namespace planning {

enum Sensitivity { StvlUnavaiable, StvlLow, StvlStandard, StvlHigh };

struct LdpLdwData {
  uint16_t odd_indx_line_cond{0};
  double left_line_markers_c0{0.0};
  double left_line_markers_c1{0.0};
  double left_line_markers_c2{0.0};
  double left_line_markers_c3{0.0};
  double left_line_markers_quality{0.0};
  double right_line_markers_c0{0.0};
  double right_line_markers_c1{0.0};
  double right_line_markers_c2{0.0};
  double right_line_markers_c3{0.0};
  double right_line_markers_quality{0.0};
  double left_tire_distance_2_line{0.0};
  double right_tire_distance_2_line{0.0};
  double left_line_markers_offset_last{0.0};
  double right_line_markers_offset_last{0.0};
};

}  // namespace planning
}  // namespace TL
