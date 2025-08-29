/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2023. All rights reserved.
 * Description:  planning
 */

#pragma once
#include "common/utm_projection/utm_zone.h"

#include "common/math/math_utils.h"
#include "common/utm_projection/coordinate_convertor.h"

namespace TL {
namespace common {
namespace utm_zone {

bool SetLocalizationAndPerceptionByZoneID(
    u_int32_t zone_id,
    const std::shared_ptr<localization::Localization>& localization,
    const std::shared_ptr<perception::PerceptionObstacles>& obstacles) {
  if (!localization->pose().has_utm_zone_01() &&
      !localization->pose().has_utm_zone_02()) {
    TransPerceptionFromFLU2ENU(localization, obstacles);
    return true;
  }
#ifdef FOR_BAIDU_SIMULATION
  if (!FLAGS_enable_planning_self_simulator) {
    localization->mutable_pose()->set_using_utm_zone(zone_id);
    return true;
  }
#endif
  if (zone_id == localization->pose().using_utm_zone()) {
    TransPerceptionFromFLU2ENU(localization, obstacles);
    return true;
  }
  if (zone_id == localization->pose().utm_zone_01()) {
    localization->mutable_pose()->mutable_position()->CopyFrom(
        localization->pose().pos_utm_01());
    localization->mutable_pose()->set_using_utm_zone(
        localization->pose().utm_zone_01());
  } else if (zone_id == localization->pose().utm_zone_02()) {
    localization->mutable_pose()->mutable_position()->CopyFrom(
        localization->pose().pos_utm_02());
    localization->mutable_pose()->set_using_utm_zone(
        localization->pose().utm_zone_02());
  } else if (zone_id != -1 && zone_id != 0) {
    AERROR << "utm zone error " << zone_id << " utm1 "
           << localization->pose().utm_zone_01() << " utm2 "
           << localization->pose().utm_zone_02();
    TransPerceptionFromFLU2ENU(localization, obstacles);
    return false;
  } else {
    ADEBUG << "utm zone error " << zone_id << " utm1 "
           << localization->pose().utm_zone_01() << " utm2 "
           << localization->pose().utm_zone_02();
    TransPerceptionFromFLU2ENU(localization, obstacles);
    return true;
  }
  // TODO(all):如果是slam定位,heading不需要补偿,需要屏蔽以下
  localization->mutable_pose()->set_heading(
      localization->pose().heading_gcs() +
      common::coordinate_convertor::HeadingError(
          static_cast<int>(localization->pose().using_utm_zone()),
          localization->pose().gcj02().x(), localization->pose().gcj02().y()));

  TransPerceptionFromFLU2ENU(localization, obstacles);

  return true;
}

bool SelectNewZoneID(
    int previous_zone_id,  // NOLINT
    const std::shared_ptr<const localization::Localization>&
        localization,
    u_int32_t* new_zone_id) {
#ifdef FOR_BAIDU_SIMULATION
  if (!FLAGS_enable_planning_self_simulator) {
    *new_zone_id = FLAGS_local_utm_zone_id;
    return true;
  }
#endif
  // 如果读取地图进行仿真，上游已经置默认值using_zone_id，则强制utm带号不再动态变化
  if (localization->pose().has_using_utm_zone()) {
    *new_zone_id = localization->pose().using_utm_zone();
    return true;
  }
  *new_zone_id =
      static_cast<u_int32_t>(localization->pose().gcj02().x() / 6 + 31);
  if (*new_zone_id != localization->pose().utm_zone_01() &&
      *new_zone_id != localization->pose().utm_zone_02()) {
    *new_zone_id = localization->pose().utm_zone_01();
  }
  return true;
}

void TransPerceptionFromFLU2ENU(
    const std::shared_ptr<localization::Localization>& localization,
    const std::shared_ptr<perception::PerceptionObstacles>& obstacles) {
  const auto& pose = localization->pose();
  for (auto& obs : *obstacles->mutable_perception_obstacle()) {
    if (!obs.has_position_flu()) {
      continue;
    }
    double flu_raw_x = obs.position_flu().x();
    double flu_raw_y = obs.position_flu().y();
    double flu_raw_vx = obs.velocity_flu().x();
    double flu_raw_vy = obs.velocity_flu().y();
    double flu_raw_ax = obs.acceleration_flu().x();
    double flu_raw_ay = obs.acceleration_flu().y();
    double obs_theta_flu = obs.theta_flu();
    double length = obs.length();
    double width = obs.width();
    // double height = obs.height();
    // FLU -> ENU
    double adc_heading = pose.heading();
    double obs_heading = adc_heading + obs_theta_flu;

    // convert to pose ENU with localization
    Eigen::Vector2d enu_pos = TL::common::math::RotateVector2d(
        {flu_raw_x, flu_raw_y}, adc_heading);
    enu_pos.x() += pose.position().x();
    enu_pos.y() += pose.position().y();

    // speed
    Eigen::Vector2d enu_velocity = TL::common::math::RotateVector2d(
        {flu_raw_vx, flu_raw_vy}, adc_heading);
    // acce
    Eigen::Vector2d enu_acceleration = TL::common::math::RotateVector2d(
        {flu_raw_ax, flu_raw_ay}, adc_heading);

    // flu polygon
    Eigen::Vector2d flu_raw_p1 = TL::common::math::RotateVector2d(
        {-length / 2, width / 2}, obs_theta_flu);
    Eigen::Vector2d flu_raw_p2 = TL::common::math::RotateVector2d(
        {-length / 2, -width / 2}, obs_theta_flu);
    Eigen::Vector2d flu_raw_p3 = TL::common::math::RotateVector2d(
        {length / 2, -width / 2}, obs_theta_flu);
    Eigen::Vector2d flu_raw_p4 = TL::common::math::RotateVector2d(
        {length / 2, width / 2}, obs_theta_flu);
    flu_raw_p1.x() += flu_raw_x;
    flu_raw_p1.y() += flu_raw_y;
    flu_raw_p2.x() += flu_raw_x;
    flu_raw_p2.y() += flu_raw_y;
    flu_raw_p3.x() += flu_raw_x;
    flu_raw_p3.y() += flu_raw_y;
    flu_raw_p4.x() += flu_raw_x;
    flu_raw_p4.y() += flu_raw_y;
    // enu polygon
    Eigen::Vector2d enu_p1 =
        TL::common::math::RotateVector2d(flu_raw_p1, adc_heading);
    enu_p1.x() += pose.position().x();
    enu_p1.y() += pose.position().y();
    Eigen::Vector2d enu_p2 =
        TL::common::math::RotateVector2d(flu_raw_p2, adc_heading);
    enu_p2.x() += pose.position().x();
    enu_p2.y() += pose.position().y();
    Eigen::Vector2d enu_p3 =
        TL::common::math::RotateVector2d(flu_raw_p3, adc_heading);
    enu_p3.x() += pose.position().x();
    enu_p3.y() += pose.position().y();
    Eigen::Vector2d enu_p4 =
        TL::common::math::RotateVector2d(flu_raw_p4, adc_heading);
    enu_p4.x() += pose.position().x();
    enu_p4.y() += pose.position().y();

    // set enu info
    obs.mutable_position()->set_x(enu_pos.x());
    obs.mutable_position()->set_y(enu_pos.y());
    obs.set_theta(obs_heading);
    obs.mutable_velocity()->set_x(enu_velocity.x());
    obs.mutable_velocity()->set_y(enu_velocity.y());
    obs.mutable_acceleration()->set_x(enu_acceleration.x());
    obs.mutable_acceleration()->set_y(enu_acceleration.y());

    obs.mutable_polygon_point()->Clear();
    auto* polygon_point = obs.add_polygon_point();
    polygon_point->set_x(enu_p1.x());
    polygon_point->set_y(enu_p1.y());
    polygon_point = obs.add_polygon_point();
    polygon_point->set_x(enu_p2.x());
    polygon_point->set_y(enu_p2.y());
    polygon_point = obs.add_polygon_point();
    polygon_point->set_x(enu_p3.x());
    polygon_point->set_y(enu_p3.y());
    polygon_point = obs.add_polygon_point();
    polygon_point->set_x(enu_p4.x());
    polygon_point->set_y(enu_p4.y());
  }
}

}  // namespace utm_zone
}  // namespace common
}  // namespace TL
