
//   Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.
#pragma once

#include <string>
#include "planning/prediction/common/prediction_gflags.h"
#include "planning/prediction/container/obstacles/obstacle.h"
#include "proto/perception/perception_obstacle.pb.h"

namespace TL {
namespace prediction {

class SensorFilter {
 public:
  SensorFilter() = delete;

  static bool IgnoreBySensor(Obstacle* obstacle) {
    if (FLAGS_enable_perception_sensor_radar_filter &&
        IgnoreByOnlyFrontRadar(obstacle)) {
      return true;
    }
    if ((FLAGS_enable_perception_sensor_lidar_filter) &&
        IgnoreByOnlyLidar(obstacle)) {
      return true;
    }
    return false;
  }

  static bool IgnoreByOnlyFrontRadar(Obstacle* obstacle) {
    auto* latest_feature = obstacle->mutable_latest_feature();
    // 24.2.4
    // 临时修复：目前感知没有sensor信息，velocity_std度量中，使用较为保守的距离阈值+较少的过滤帧数减少E4阶段顿挫
    // 可能造成刹车晚3帧
    // const auto& detect_sensor =
    //     latest_feature->sensor_feature().curr_detect_sensor();
    const auto flu_x = latest_feature->position_flu().x();

    // if (IsOnlyDetectByFrontRadar(detect_sensor) &&
    //     (flu_x > FLAGS_only_front_radar_range) &&
    if ((flu_x > FLAGS_only_front_radar_range) &&
        obstacle->history_size() <= FLAGS_only_front_radar_frame_num) {
      std::string ignore_reason =
          "ignore front radar , flu_x: " + std::to_string(flu_x) +
          ", history_size: " + std::to_string(obstacle->history_size());
      latest_feature->mutable_sensor_feature()->mutable_ignore_reason()->append(
          ignore_reason);
      latest_feature->mutable_sensor_feature()->set_is_ignore_by_sensor(true);
      return true;
    }
    return false;
  }

  static bool IgnoreByOnlyLidar(Obstacle* obstacle) {
    auto* latest_feature = obstacle->mutable_latest_feature();
    const auto& detect_sensor =
        latest_feature->sensor_feature().curr_detect_sensor();
    const auto flu_x = latest_feature->position_flu().x();

    if (IsOnlyDetectByFrontLidar(detect_sensor) &&
        flu_x >= FLAGS_only_front_lidar_range &&
        obstacle->history_size() <= FLAGS_only_front_lidar_frame_num) {
      std::string ignore_reason =
          "ignore front lidar , flu x: " + std::to_string(flu_x) +
          ", history length: " + std::to_string(obstacle->history_size());
      latest_feature->mutable_sensor_feature()->mutable_ignore_reason()->append(
          ignore_reason);
      latest_feature->mutable_sensor_feature()->set_is_ignore_by_sensor(true);
      return true;
    }
    return false;
  }

  static bool IsIgnoreByOnlyFrontRadar(
      const perception::DetectSensor& detect_sensor, double flu_x) {
    return IsOnlyDetectByFrontRadar(detect_sensor) &&
           (flu_x > FLAGS_only_front_radar_range);
  }

  static bool IsOnlyDetectByFrontRadar(
      const perception::DetectSensor& detect_sensor) {
    return IsDetectByFrontRadar(detect_sensor) &&
           !IsDetectByAnyFrontCornerRadar(detect_sensor) &&
           !IsDetectByAnyFrontCamera(detect_sensor) &&
           !IsDetectByAnyFrontLidar(detect_sensor);
  }

  static bool IsDetectByFrontRadar(
      const perception::DetectSensor& detect_sensor) {
    return detect_sensor.from_radar_front();
  }

  static bool IsDetectByAnyFrontCornerRadar(
      const perception::DetectSensor& detect_sensor) {
    return detect_sensor.from_radar_front_left() ||
           detect_sensor.from_radar_front_right();
  }

  static bool IsDetectByAnyFrontCamera(
      const perception::DetectSensor& detect_sensor) {
    return detect_sensor.from_camera_front_long_range() ||
           detect_sensor.from_camera_front_wide_angle() ||
           detect_sensor.from_camera_right_forward_looking() ||
           detect_sensor.from_camera_left_forward_looking();
  }

  static bool IsDetectByAnyFrontLidar(
      const perception::DetectSensor& detect_sensor) {
    return detect_sensor.from_lidar_front_left() ||
           detect_sensor.from_lidar_front_right();
  }

  static bool IsOnlyDetectByFrontLidar(
      const perception::DetectSensor& detect_sensor) {
    return (detect_sensor.from_lidar_front_left() &&
            !detect_sensor.from_lidar_front_right()) ||
           (!detect_sensor.from_lidar_front_left() &&
            detect_sensor.from_lidar_front_right());
  }
};

}  // namespace prediction
}  // namespace TL
