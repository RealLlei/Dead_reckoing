/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "planning/prediction/mobileye_adapter/mobileye.h"

#include "common/file/file.h"
#include "common/math/math_utils.h"
#include "common/math/quaternion.h"
#include "common/math/vec2d.h"
#include "common/util/util.h"
#include "cyber/time/clock.h"

namespace TL {
namespace mobileye {

using TL::localization::Localization;
using TL::perception::PerceptionObstacles;
using TL::soc::ChassisDetail;

#define MULTI_OBJECT

DEFINE_double(mobileye_obs_keep_time, 501.0, "obstacle keep time , ms");

Mobileye::Mobileye() {}

Status Mobileye::Init() {
  obs_type_parser_[soc::Vis_obs_msg_1_675::VIS_OBS_CLASSIFICATION_01_CAR] =
      perception::PerceptionObstacle::VEHICLE;
  obs_type_parser_
      [soc::Vis_obs_msg_1_675::VIS_OBS_CLASSIFICATION_01_MOTORCYCLE] =
          perception::PerceptionObstacle::BICYCLE;
  obs_type_parser_
      [soc::Vis_obs_msg_1_675::VIS_OBS_CLASSIFICATION_01_PEDESTRIAN] =
          perception::PerceptionObstacle::PEDESTRIAN;
  obs_type_parser_[soc::Vis_obs_msg_1_675::VIS_OBS_CLASSIFICATION_01_TRUCK] =
      perception::PerceptionObstacle::VEHICLE;
  obs_type_parser_[soc::Vis_obs_msg_1_675::VIS_OBS_CLASSIFICATION_01_BICYCLE] =
      perception::PerceptionObstacle::BICYCLE;
  return Status::OK();
}

TL::common::Status Mobileye::Start() {
  return Status::OK();
}

bool Mobileye::Process(std::shared_ptr<PerceptionObstacles> perception_obs) {
  // remove old obstacle
  // and insert current obs to send

  perception_obs->Clear();
  double timestamp = TL::common::Clock::NowInSeconds();

  std::lock_guard<std::mutex> lock(mobileye_mutex_);

#ifdef MULTI_OBJECT
  auto iter = obs_map_.begin();
  while (iter != obs_map_.end()) {
    double timediff = timestamp - iter->second.timestamp();
    if (abs(timediff) * 1000 > FLAGS_mobileye_obs_keep_time) {
      ADEBUG << "remove obs[" << iter->second.id()
             << "],timediff = " << timediff;
      obs_map_.erase(iter++);
    } else {
      ADEBUG << "obs[" << iter->second.id()
             << "]still alive ,timediff = " << timediff;
      perception_obs->add_perception_obstacle()->CopyFrom(iter->second);
      ++iter;
    }
  }
#else
  double timediff = timestamp - raw_obstacle_.timestamp();
  if (abs(timediff) < Kmobileye_obs_keep_time) {
    ADEBUG << "obs[" << raw_obstacle_.id()
           << "]still alive ,timediff = " << timediff;
    perception_obs->add_perception_obstacle()->CopyFrom(raw_obstacle_);
  }
#endif

  return true;
}

void Mobileye::OnChassisDetail(const ChassisDetail& chassis_detail) {
  auto mobileye_all = chassis_detail.neta().mobileye_all();
  if (mobileye_all.empty()) {
    return;
  }
  auto it = mobileye_all.cbegin();
  // for (auto it = mobileye_all.cbegin(); it != mobileye_all.cend(); ++it) {
  int id = it->first;
  auto& mobileye_obs = it->second;
  double received_time = mobileye_obs.received_time();
  auto msg_675 = mobileye_obs.vis_obs_msg_1_675();
  auto msg_676 = mobileye_obs.vis_obs_msg_2_676();
  auto msg_677 = mobileye_obs.vis_obs_msg_3_677();
  // }

  // if (!chassis_detail.neta().has_vis_obs_msg_1_675() ||
  //     !chassis_detail.neta().has_vis_obs_msg_2_676() ||
  //     !chassis_detail.neta().has_vis_obs_msg_3_677()) {
  //   return;s
  // }

  constexpr static double Kepsinon = 0.0000001;

  // int id = msg_675.vis_obs_id_01();
  double raw_x = msg_677.vis_obs_lat_pos_01();
  double raw_y = msg_677.vis_obs_long_pos_01();
  if (abs(raw_x) <= Kepsinon && abs(raw_y) <= Kepsinon) {
    return;
  }
  double timestamp = TL::common::Clock::NowInSeconds();

  double raw_vx = msg_677.vis_obs_lat_vel_01();
  double raw_vy = msg_677.vis_obs_long_vel_01();
  double width = msg_677.vis_obs_width_01();
  double length = width;
  double theta = 0;
  auto mobileye_type = msg_675.vis_obs_classification_01();
  perception::PerceptionObstacle::Type type =
      (obs_type_parser_.find(mobileye_type) != obs_type_parser_.end())
          ? obs_type_parser_[mobileye_type]
          : perception::PerceptionObstacle::UNKNOWN;
  // unused
  auto obs_turn_indicator = msg_675.vis_obs_turn_indicator_01();
  auto obs_ped_waist_up = msg_675.vis_obs_ped_waist_up_01();
  auto raw_height = msg_675.vis_obs_height_01();
  auto brake_light = msg_675.vis_obs_brake_light_indic_01();
  auto obs_ttc = msg_676.vis_obs_ttc_const_acc_model_01();
  auto obs_long_acc = msg_676.vis_obs_long_accel_01();
  auto obs_cut_in_out = msg_676.vis_obs_cut_in_out_01();

  // FLU -> ENU
  double ENU_raw_x = raw_y;
  double ENU_raw_y = -raw_x;
  double ENU_raw_vx = raw_vy;
  double ENU_raw_vy = -raw_vx;
  // set position at
  Eigen::Vector2d ENU_raw_position(ENU_raw_x + (length / 2), ENU_raw_y);

  // polygon
  Eigen::Vector2d ENU_raw_p1(ENU_raw_x, ENU_raw_y + width / 2);
  Eigen::Vector2d ENU_raw_p2(ENU_raw_x, ENU_raw_y - width / 2);
  Eigen::Vector2d ENU_raw_p3(ENU_raw_x + length, ENU_raw_y - width / 2);
  Eigen::Vector2d ENU_raw_p4(ENU_raw_x + length, ENU_raw_y + width / 2);

  // convert to ENU with localization
  double heading = localization_.pose().heading();
  Eigen::Vector2d ENU_pos =
      common::math::RotateVector2d({ENU_raw_position.x(), ENU_raw_position.y()},
                                   localization_.pose().heading());

  ENU_pos.x() += localization_.pose().position().x();
  ENU_pos.y() += localization_.pose().position().y();

  Eigen::Vector2d ENU_velocity = common::math::RotateVector2d(
      {ENU_raw_vx, ENU_raw_vy}, localization_.pose().heading());

  Eigen::Vector2d ENU_p1 =
      common::math::RotateVector2d(ENU_raw_p1, localization_.pose().heading());
  ENU_p1.x() += localization_.pose().position().x();
  ENU_p1.y() += localization_.pose().position().y();
  Eigen::Vector2d ENU_p2 =
      common::math::RotateVector2d(ENU_raw_p2, localization_.pose().heading());
  ENU_p2.x() += localization_.pose().position().x();
  ENU_p2.y() += localization_.pose().position().y();
  Eigen::Vector2d ENU_p3 =
      common::math::RotateVector2d(ENU_raw_p3, localization_.pose().heading());
  ENU_p3.x() += localization_.pose().position().x();
  ENU_p3.y() += localization_.pose().position().y();
  Eigen::Vector2d ENU_p4 =
      common::math::RotateVector2d(ENU_raw_p4, localization_.pose().heading());
  ENU_p4.x() += localization_.pose().position().x();
  ENU_p4.y() += localization_.pose().position().y();

  // set pb
  TL::perception::PerceptionObstacle obstacle;
  obstacle.set_id(id);
  obstacle.mutable_position()->set_x(ENU_pos.x());
  obstacle.mutable_position()->set_y(ENU_pos.y());
  obstacle.set_theta(heading);
  obstacle.set_length(length);
  obstacle.set_width(width);
  obstacle.set_height(width);
  obstacle.mutable_velocity()->set_x(ENU_velocity.x());
  obstacle.mutable_velocity()->set_y(ENU_velocity.y());
  obstacle.set_type(type);

  auto polygon_point = obstacle.add_polygon_point();
  polygon_point->set_x(ENU_p1.x());
  polygon_point->set_y(ENU_p1.y());
  polygon_point = obstacle.add_polygon_point();
  polygon_point->set_x(ENU_p2.x());
  polygon_point->set_y(ENU_p2.y());
  polygon_point = obstacle.add_polygon_point();
  polygon_point->set_x(ENU_p3.x());
  polygon_point->set_y(ENU_p3.y());
  polygon_point = obstacle.add_polygon_point();
  polygon_point->set_x(ENU_p4.x());
  polygon_point->set_y(ENU_p4.y());

  obstacle.set_timestamp(timestamp);
  ADEBUG << "obs[" << id << "]recive ,at  = " << timestamp;
  {
    std::lock_guard<std::mutex> lock(mobileye_mutex_);
#ifdef MULTI_OBJECT
    obs_map_[id] = obstacle;
#else
    raw_obstacle_.CopyFrom(obstacle);
#endif
  }
}

void Mobileye::OnLocalization(
    const localization::Localization& localization_estimate) {
  {
    std::lock_guard<std::mutex> lock(mobileye_mutex_);
    localization_.CopyFrom(localization_estimate);
  }
  return;
}

void Mobileye::Stop() {}

}  // namespace mobileye
}  // namespace TL
