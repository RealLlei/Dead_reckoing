
//   Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.
#pragma once

#include "planning/prediction/common/prediction_gflags.h"
#include "planning/prediction/container/obstacles/obstacle.h"
#include "planning/prediction/scenario/perception_filter/sensor_filter.h"

namespace TL {
namespace prediction {

class PerceptionFilter {
 public:
  PerceptionFilter() = delete;

  static bool Filter(Obstacle* obstacle, const Scenario* current_scenario,
                     double occluded_ignore_distance) {
    if (current_scenario->type() != Scenario::CRUISE_HIGHWAY ||
        !FLAGS_enable_percpetion_filter || obstacle == nullptr ||
        !obstacle->latest_feature().has_position_flu()) {
      return false;
    }

    if (FLAGS_enable_perception_sensor_filter && IgnoreBySensor(obstacle)) {
      ADEBUG << "Obstacle [ " << obstacle->id() << "] ,ignore by sensor";
      return true;
    }
    if (FLAGS_enable_perception_outofmap_filter && OppositeHighway(obstacle)) {
      ADEBUG << "Obstacle [ " << obstacle->id()
             << "] ,ignore by opposite highway";
      return true;
    }
    if (obstacle->IgnoreByOccludedProb(occluded_ignore_distance)) {
      ADEBUG << "Obstacle [ " << obstacle->id() << "] ,ignore by occluded prob";
      return true;
    }

    return false;
  }

  static bool IgnoreBySensor(Obstacle* obstacle) {
    return SensorFilter::IgnoreBySensor(obstacle);
  }

  static bool OppositeHighway(Obstacle* obstacle) {
    return obstacle->IsOppositeHighway();
  }
};
}  // namespace prediction
}  // namespace TL
