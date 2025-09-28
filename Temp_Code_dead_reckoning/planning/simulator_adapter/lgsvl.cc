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

#include "planning/simulator_adapter/lgsvl.h"

#include "common/file/file.h"
#include "common/math/math_utils.h"
#include "common/math/quaternion.h"
#include "common/math/vec2d.h"
#include "common/util/util.h"

using TL::common::math::HeadingToQuaternion;

namespace TL {
namespace lgsvl {

using TL::common::ErrorCode;
using TL::common::Status;
using TL::localization::Localization;
using TL::perception::PerceptionObstacles;
using TL::soc::ChassisDetail;

Lgsvl::Lgsvl() {}

Status Lgsvl::Init() {
  return Status::OK();
}

TL::common::Status Lgsvl::Start() {
  return Status::OK();
}

bool Lgsvl::Process(std::shared_ptr<ChassisDetail> chassis_detail,
                    std::shared_ptr<Localization> localization) {
  std::lock_guard<std::mutex> lock(lgsvl_mutex_);
  chassis_detail->CopyFrom(sim_chassis_detail_);
  localization->CopyFrom(sim_localization_msg_);
  return true;
}

void Lgsvl::OnLgsvlPerceptionLines(
    const perception::PerceptionLanes& line_msg) {
  soc::ChassisDetail chassis_detail;
  {
    if (line_msg.camera_laneline_size() < 2) {
      return;
    }

    auto neta = chassis_detail.mutable_neta();
    auto vis_lane_near_left_individual_6ac =
        neta->mutable_vis_lane_near_left_individual_6ac();
    auto input_line = line_msg.camera_laneline().at(0).curve_camera_coord();
    vis_lane_near_left_individual_6ac->set_vis_lane_left_individ_a0(
        input_line.a());
    vis_lane_near_left_individual_6ac->set_vis_lane_left_individ_a1(
        input_line.b());
    vis_lane_near_left_individual_6ac->set_vis_lane_left_individ_a2(
        input_line.c());
    vis_lane_near_left_individual_6ac->set_vis_lane_left_individ_a3(
        input_line.d());
    vis_lane_near_left_individual_6ac->set_vis_lane_left_individ_range(360);
    auto vis_lane_near_left_individual_6a7 =
        neta->mutable_vis_lane_near_left_parallel_6a7();
    vis_lane_near_left_individual_6a7->set_vis_lane_left_parall_a0(
        input_line.a());
    vis_lane_near_left_individual_6a7->set_vis_lane_left_parall_a1(
        input_line.b());
    vis_lane_near_left_individual_6a7->set_vis_lane_left_parall_a2(
        input_line.c());
    vis_lane_near_left_individual_6a7->set_vis_lane_left_parall_a3(
        input_line.d());
    vis_lane_near_left_individual_6a7->set_vis_lane_left_parall_range(360);

    auto vis_lane_near_right_individual_6ab =
        neta->mutable_vis_lane_near_right_individual_6ab();
    input_line = line_msg.camera_laneline().at(1).curve_camera_coord();
    vis_lane_near_right_individual_6ab->set_vis_lane_right_individ_a0(
        input_line.a());
    vis_lane_near_right_individual_6ab->set_vis_lane_right_individ_a1(
        input_line.b());
    vis_lane_near_right_individual_6ab->set_vis_lane_right_individ_a2(
        input_line.c());
    vis_lane_near_right_individual_6ab->set_vis_lane_right_individ_a3(
        input_line.d());
    vis_lane_near_right_individual_6ab->set_vis_lane_right_individ_range(360);
    auto vis_lane_near_right_individual_6a6 =
        neta->mutable_vis_lane_near_right_parallel_6a6();
    vis_lane_near_right_individual_6a6->set_vis_lane_right_parall_a0(
        input_line.a());
    vis_lane_near_right_individual_6a6->set_vis_lane_right_parall_a1(
        input_line.b());
    vis_lane_near_right_individual_6a6->set_vis_lane_right_parall_a2(
        input_line.c());
    vis_lane_near_right_individual_6a6->set_vis_lane_right_parall_a3(
        input_line.d());
    vis_lane_near_right_individual_6a6->set_vis_lane_right_parall_range(360);
  }
  {
    std::lock_guard<std::mutex> lock(lgsvl_mutex_);
    sim_chassis_detail_.CopyFrom(chassis_detail);
  }
}

void Lgsvl::OnLgsvlGps(const localization::Gps& gps_msg) {
  Localization localization;
  {
    auto sim_pose = localization.mutable_pose();
    sim_pose->mutable_position()->set_x(gps_msg.localization().position().x());
    sim_pose->mutable_position()->set_y(gps_msg.localization().position().y());
    // double heading = M_PI / 2 - (gps_msg.localization().heading() / 180);
    // double heading = (gps_msg.localization().heading() + 90.0) / 180.0 *
    // M_PI;
    // if (heading > M_PI) {
    //   heading = heading - 2 * M_PI;
    // }

    localization.mutable_header()->set_data_stamp(
        gps_msg.header().data_stamp());

    // sim_pose->mutable_linear_velocity()->set_x(
    //     gps_msg.localization().linear_velocity().y());
    // sim_pose->mutable_linear_velocity()->set_y(
    //     -gps_msg.localization().linear_velocity().x());
    // sim_pose->mutable_linear_velocity()->set_z(
    //     gps_msg.localization().linear_velocity().z());

    sim_pose->mutable_angular_velocity()->set_x(0.0);
    sim_pose->mutable_angular_velocity()->set_y(0.0);
    sim_pose->mutable_angular_velocity()->set_z(0.0);

    sim_pose->mutable_angular_velocity_vrf()->set_x(0.0);
    sim_pose->mutable_angular_velocity_vrf()->set_y(0.0);
    sim_pose->mutable_angular_velocity_vrf()->set_z(0.0);

    sim_pose->mutable_orientation()->CopyFrom(
        gps_msg.localization().orientation());

    double heading = TL::common::math::QuaternionToHeading(
        sim_pose->orientation().qw(), sim_pose->orientation().qx(),
        sim_pose->orientation().qy(), sim_pose->orientation().qz());

    sim_pose->set_heading(heading);

    auto linearvxvy = common::math::ENUToRFU(
        gps_msg.localization().linear_velocity().y(),
        -gps_msg.localization().linear_velocity().x(), 0, 0, heading);
    sim_pose->mutable_linear_velocity()->set_x(
        gps_msg.localization().linear_velocity().y());
    sim_pose->mutable_linear_velocity()->set_y(
        -gps_msg.localization().linear_velocity().x());
    sim_pose->mutable_linear_velocity()->set_z(
        gps_msg.localization().linear_velocity().z());

    sim_pose->mutable_linear_velocity_vrf()->set_x(linearvxvy.first);
    sim_pose->mutable_linear_velocity_vrf()->set_y(linearvxvy.second);
    sim_pose->mutable_linear_velocity_vrf()->set_z(
        gps_msg.localization().linear_velocity().z());
  }
  {
    std::lock_guard<std::mutex> lock(lgsvl_mutex_);
    sim_localization_msg_.MergeFrom(localization);
  }

  return;
}

void Lgsvl::OnLgsvlImu(const drivers::gnss::Imu& imu_msg) {
  Localization localization;
  {
    std::lock_guard<std::mutex> lock(lgsvl_mutex_);
    auto sim_pose = localization.mutable_pose();
    if (sim_localization_msg_.pose().has_quaternion()) {
      //   Eigen::Vector3d orig(imu_msg.linear_acceleration().x(),
      //                        imu_msg.linear_acceleration().y(),
      //                        imu_msg.linear_acceleration().z());
      //   Eigen::Vector3d vec = common::math::QuaternionRotate(
      //       sim_localization_msg_.pose().quaternion(), orig);
      //   sim_pose->mutable_linear_acceleration()->set_x(vec[0]);
      //   sim_pose->mutable_linear_acceleration()->set_y(vec[1]);
      //   sim_pose->mutable_linear_acceleration()->set_z(vec[2]);

      // linear_acceleration_vfr
      sim_pose->mutable_linear_acceleration_vrf()->set_x(
          -imu_msg.linear_acceleration().y());
      sim_pose->mutable_linear_acceleration_vrf()->set_y(
          imu_msg.linear_acceleration().x());
      sim_pose->mutable_linear_acceleration_vrf()->set_z(
          imu_msg.linear_acceleration().z());
      // linear_acceleration
      TL::common::util::TransformToMRF(
          sim_pose->linear_acceleration_vrf(), sim_pose->orientation(),
          sim_pose->mutable_linear_acceleration());

      // angular_velocity_vrf
      sim_pose->mutable_angular_velocity_vrf()->set_x(
          imu_msg.angular_velocity().x());
      sim_pose->mutable_angular_velocity_vrf()->set_y(
          imu_msg.angular_velocity().y());
      sim_pose->mutable_angular_velocity_vrf()->set_z(
          imu_msg.angular_velocity().z());
      sim_localization_msg_.MergeFrom(localization);
      // angular_velocity
      TL::common::util::TransformToMRF(sim_pose->angular_velocity_vrf(),
                                          sim_pose->orientation(),
                                          sim_pose->mutable_angular_velocity());
    }
  }
}

void Lgsvl::Stop() {}

}  // namespace lgsvl
}  // namespace TL
