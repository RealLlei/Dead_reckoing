/*
 * Copyright (c) TL auto Co., Ltd. 2021-2022. All rights reserved.
 */
#include "planning/localview/lane_line_builder/adaptive_cruise_lane_line/cruise_target_select.h"
#include <unistd.h>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>
#include "common/file/log.h"
#include "common/math/box2d.h"
#include "common/math/vec2d.h"
#include "common/status/status.h"
#include "planning/common/planning_gflags.h"
#include "proto/common/vehicle_config.pb.h"
#include "proto/perception/perception_obstacle.pb.h"
#include "proto/soc/chassis.pb.h"

namespace TL {
namespace planning {
namespace {
constexpr double k_ACCTS_MaxSelectionRange = 150;
constexpr double k_ACCTS_MinStrAngleForNonline = 5.0;
constexpr double k_ACCTS_MaxRangeHighTsQly = 200.0;
const std::array<double, 8> k_ACCTS_FunnelOffsetSpeed{0,  2,  5,  10,
                                                      20, 30, 40, 55};
const std::array<double, 8> k_ACCTS_FunnelOffsetNormal{0, 0, 0, 0, 0, 0, 0, 0};
const std::array<double, 8> k_ACCTS_FunnelOffsetOvertaking{0.4, 0.4, 0.5, 1.2,
                                                           1.5, 1.5, 1.5, 1.5};
const std::array<double, 8> k_ACCTS_InnerFunnelOffsetOpposite{
    -0.2, -0.2, -0.2, -0.2, -0.2, -0.2, -0.2, -0.2};
const std::array<double, 8> k_ACCTS_InnerFunnelOffsetOvertaking{
    0.2, 0.2, 0.25, 0.6, 0.75, 0.75, 0.75, 0.75};
const std::array<double, 19> k_ACCTS_IP_FunnelIP_acc{
    -200, -150, -100, -80, -60, -40, -20, -12, -4, 0,
    4,    12,   20,   40,  60,  80,  100, 150, 200};
const std::array<double, 19> k_ACCTS_IP_OuterFunnelWidths{
    1.2,         1.2,  1.1,  0.94, 0.900000036, 0.900000036, 0.900000036,
    0.900000036, 0.85, 0.85, 0.85, 0.900000036, 0.900000036, 0.900000036,
    0.900000036, 0.94, 1.0,  1.0,  1.0};
const std::array<double, 19> k_ACCTS_IP_InnerFunnelWidths{
    0.71, 0.68, 0.66, 0.7,  0.7,  0.7,  0.7, 0.55, 0.45, 0.45,
    0.55, 0.6,  0.65, 0.75, 0.75, 0.75, 0.7, 0.7,  0.7};
const double k_ACCTS_AdjacentLaneMultFactorInner = 3.6;
const double k_ACCTS_RightKeepFunnelScalingLowTsQlyRegion = 1;
const double k_ACCTS_AdjacentLaneMultFactorOuter = 0.85;
const double k_DynCalPrmMdlAxleDistFrnt = 3.6;
const double k_ACCTS_vehicleLengthOffset = 1.5;
const double k_ACCTS_vehicleLength = 4.98;
const double k_oncoming_target_speed_threshold = -2.8;
}  // namespace

CruiseTargetSelect::CruiseTargetSelect(const PerceptionMapConfig& config)
    : navigation_hdmap_config_(config) {}

Status CruiseTargetSelect::Init() {
  vehicle_param_ = common::VehicleConfigHelper::GetConfig().vehicle_param();
  target_overlap_.Init();
  ResetSelectResultInfos(&stop_result_infos_);
  ResetSelectResultInfos(&last_stop_result_infos_);
  return Status::OK();
}

Status CruiseTargetSelect::Start() {
  return Status::OK();
}

void CruiseTargetSelect::Stop() {}

bool CruiseTargetSelect::Process(const std::shared_ptr<LocalView>& local_view,
                                 const double subject_kappa,
                                 const double steer_angle,
                                 std::vector<int32_t>* cruise_target_id) {
  if (cruise_target_id == nullptr || local_view == nullptr) {
    return false;
  }
  perception_obstacles_ = local_view->GetPerceptionObstacles();
  vehicle_state_ = local_view->GetVehicleState();
  const double subject_speed_longitudinal = vehicle_state_->linear_velocity();
  if (std::abs(subject_kappa) < 0.0005) {
    subject_kappa_counter_ = subject_kappa_counter_ + 1;
  } else {
    subject_kappa_counter_ = subject_kappa_counter_ - 1;
  }
  if (subject_kappa_counter_ >= 100) {
    subject_kappa_counter_ = 100;
  }
  if (subject_kappa_counter_ <= -100) {
    subject_kappa_counter_ = -100;
  }
  acc_state_is_stand_ =
      local_view->HasFunctionManagerIn() &&
      local_view->GetFunctionManagerIn() != nullptr &&
      local_view->GetFunctionManagerIn()->fct_nnp_in().has_acc_state() &&
      (local_view->GetFunctionManagerIn()->fct_nnp_in().acc_state() ==
           functionmanager::FctToNnpInput::ACC_STANDBY ||
       local_view->GetFunctionManagerIn()->fct_nnp_in().acc_state() ==
           functionmanager::FctToNnpInput::ACC_ACTIVE ||
       local_view->GetFunctionManagerIn()->fct_nnp_in().acc_state() ==
           functionmanager::FctToNnpInput::ACC_STANDSTILL_ACTIVE ||
       local_view->GetFunctionManagerIn()->fct_nnp_in().acc_state() ==
           functionmanager::FctToNnpInput::ACC_STANDSTILL_WAIT);
  ADEBUG << "acc_state "
         << static_cast<int>(local_view->GetFunctionManagerIn()->fct_nnp_in().acc_state());
  // ADEBUG << "subject_speed_longitudinal " << subject_speed_longitudinal;
  std::unordered_map<int32_t, OneTargetCalculateinfos> target_calculate_infos;
  SelectResultInfos select_result_infos{};
  ResetSelectResultInfos(&select_result_infos);
  if (perception_obstacles_->perception_obstacle_size() > 0) {
    for (const auto& obstacle : perception_obstacles_->perception_obstacle()) {
      if (FiltByObstacleInfos(obstacle)) {
        continue;
      }
      const auto id = obstacle.id();
      const int fusn_src = 2;
      const int snsr_data_sts = 5;
      const bool snsr_data_valid = true;
      const double target_position_longitudinal = obstacle.position_flu().x();
      const double target_position_lateral = obstacle.position_flu().y();
      const int target_type = static_cast<int>(obstacle.type());
      const double target_speed = obstacle.velocity_flu().x();
      const double target_orientation = obstacle.orientation();
      // int target_quality = 4;
      // double distance_to_center_lane = 0;
      const double target_speed_lateral = obstacle.velocity_flu().y();
      const double target_width = obstacle.width();
      const double target_length = obstacle.length();
      double last_target_position_longitudinal = 0;
      double last_target_position_lateral = 0;
      bool has_same_id_last_infos = false;

      if (!last_target_calculate_infos_.empty()) {
        auto it = last_target_calculate_infos_.find(obstacle.id());
        if (it != last_target_calculate_infos_.end()) {
          has_same_id_last_infos = true;
        }
        if (has_same_id_last_infos) {
          last_target_position_longitudinal =
              last_target_calculate_infos_[obstacle.id()].position.x;
          last_target_position_lateral =
              last_target_calculate_infos_[obstacle.id()].position.y;
        }
      }

      ADEBUG << " id " << obstacle.id() << " target_type " << target_type
             << " target_width " << target_width << " x "
             << target_position_longitudinal << " y " << target_position_lateral
             << " xK1 " << last_target_position_longitudinal << " yK1 "
             << last_target_position_lateral << " vx " << target_speed << " vy "
             << target_speed_lateral;

      const bool maturity_ok = UpdateMaturity(
          fusn_src, target_position_longitudinal, snsr_data_valid);

      const bool jump_ok = UpdateJumpMaturity(
          target_position_longitudinal, last_target_position_longitudinal,
          target_position_lateral, last_target_position_lateral,
          has_same_id_last_infos);

      const bool coasting_ok = UpdateCoastedTimer(snsr_data_sts);
      const bool use_yaw_plus_lane_change =
          UpdateUseYawPlusInLChange(false, false, false);
      TargetValidatorInfos target_validator_infos = TargetValidator(
          fusn_src, snsr_data_sts, target_position_longitudinal, target_type,
          target_speed, maturity_ok, jump_ok, coasting_ok);
      std::tuple<double, double, double, double> target_offset_tuple =
          CalculateTargetOffset(target_position_longitudinal,
                                target_position_lateral, target_orientation,
                                target_speed_lateral, target_width,
                                target_length, 0.5 * subject_kappa,
                                target_validator_infos.target_valid,
                                steer_angle, subject_speed_longitudinal);

      ADEBUG << "target_valid " << target_validator_infos.target_valid
             << " offset " << std::get<1>(target_offset_tuple)
             << " center_offset " << std::get<2>(target_offset_tuple);
      SelectFunnel select_funnel = CalculateFunnel(
          FLAGS_adaptive_cruise_lane_width, target_position_longitudinal,
          subject_speed_longitudinal, false, false, 0,
          target_validator_infos.on_coming);

      const bool use_prediction = false;
      const double predicted_offset = std::get<1>(target_offset_tuple) +
                                      std::get<0>(target_offset_tuple) * 1;
      const bool is_cut_in = false;
      // bool target_id_change = target_id != last_target_id;
      OverlapInfos overlap_infos{};
      InitTargetOverlapInfos(&overlap_infos.current_path_overlap_infos);
      InitTargetOverlapInfos(&overlap_infos.adjacent_path_overlap_infos);
      TargetInfos target_infos{};
      SetTargetInfos(target_offset_tuple, obstacle, &target_infos);
      if (!target_overlap_.Process(target_infos, select_funnel,
                                   &overlap_infos)) {
        InitTargetOverlapInfos(&overlap_infos.current_path_overlap_infos);
        InitTargetOverlapInfos(&overlap_infos.adjacent_path_overlap_infos);
      }

      const bool with_in_wide_funnel = CheckInWidePath(
          select_funnel, overlap_infos, obstacle,
          std::get<1>(target_offset_tuple), id, has_same_id_last_infos);
      const bool with_in_funnel = CheckInPath(
          select_funnel, overlap_infos, obstacle,
          std::get<1>(target_offset_tuple), id, has_same_id_last_infos);
      ADEBUG << "outer_left " << select_funnel.outer_left << " outer_right "
             << select_funnel.outer_right << " wide_outer_left "
             << select_funnel.wide_outer_left << " wide_outer_right "
             << select_funnel.wide_outer_right;
      ADEBUG << " with_in_wide_funnel " << with_in_wide_funnel
             << " with_in_funnel " << with_in_funnel
             << " current_obstalce_occupancy "
             << overlap_infos.current_path_overlap_infos.obstalce_occupancy
             << " current_trajectory_occupancy "
             << overlap_infos.current_path_overlap_infos.trajectory_occupancy
             << " adjacent_obstalce_occupancy "
             << overlap_infos.adjacent_path_overlap_infos.obstalce_occupancy
             << " adjacent_trajectory_occupancy "
             << overlap_infos.adjacent_path_overlap_infos.trajectory_occupancy;

      target_calculate_infos[obstacle.id()].id = obstacle.id();
      target_calculate_infos[obstacle.id()].fusn_src = fusn_src;
      target_calculate_infos[obstacle.id()].maturity_ok = maturity_ok;
      target_calculate_infos[obstacle.id()].jump_ok = jump_ok;
      target_calculate_infos[obstacle.id()].coasting_ok = coasting_ok;
      target_calculate_infos[obstacle.id()].use_yaw_plus_lane_change =
          use_yaw_plus_lane_change;
      target_calculate_infos[obstacle.id()].target_valid =
          target_validator_infos.target_valid;
      target_calculate_infos[obstacle.id()].offset_deriv =
          std::get<0>(target_offset_tuple);
      target_calculate_infos[obstacle.id()].offset =
          std::get<1>(target_offset_tuple);
      target_calculate_infos[obstacle.id()].center_offset =
          std::get<2>(target_offset_tuple);
      target_calculate_infos[obstacle.id()].select_funnel = select_funnel;
      target_calculate_infos[obstacle.id()].use_prediction = use_prediction;
      target_calculate_infos[obstacle.id()].predicted_offset = predicted_offset;
      target_calculate_infos[obstacle.id()].is_cut_in = is_cut_in;
      target_calculate_infos[obstacle.id()].with_in_wide_funnel =
          with_in_wide_funnel;
      target_calculate_infos[obstacle.id()].with_in_funnel = with_in_funnel;
      target_calculate_infos[obstacle.id()].position.x =
          obstacle.position_flu().x();
      target_calculate_infos[obstacle.id()].position.y =
          obstacle.position_flu().y();
      target_calculate_infos[obstacle.id()].position.z =
          obstacle.position_flu().z();
      target_calculate_infos[obstacle.id()].velocity.x =
          obstacle.velocity_flu().x();
      target_calculate_infos[obstacle.id()].velocity.y =
          obstacle.velocity_flu().y();
      target_calculate_infos[obstacle.id()].velocity.z =
          obstacle.velocity_flu().z();
      target_calculate_infos[obstacle.id()].acceleration.x =
          obstacle.acceleration_flu().x();
      target_calculate_infos[obstacle.id()].acceleration.y =
          obstacle.acceleration_flu().y();
      target_calculate_infos[obstacle.id()].acceleration.z =
          obstacle.acceleration_flu().z();
      target_calculate_infos[obstacle.id()].has_same_id_last_infos =
          has_same_id_last_infos;
      target_calculate_infos[obstacle.id()].overlap_infos = overlap_infos;
    }
    safe_stop_distance_ = SafeStopDistance(
        subject_speed_longitudinal, vehicle_state_->linear_acceleration(),
        subject_kappa, -2.5, -1.0, -2.5, -1.0);
    ADEBUG << "safe_stop_distance_ " << safe_stop_distance_;
    for (const auto& obstacle : perception_obstacles_->perception_obstacle()) {
      if (FiltByObstacleInfos(obstacle)) {
        continue;
      }
      if (!target_calculate_infos[obstacle.id()].target_valid) {
        continue;
      }
      const int32_t id = obstacle.id();
      const double position_longitudinal = obstacle.position_flu().x();
      const int fusn_src = 2;
      ADEBUG << " id = " << obstacle.id() << " with_in_wide_funnel1 "
             << target_calculate_infos[obstacle.id()].with_in_wide_funnel
             << " with_in_funnel1 "
             << target_calculate_infos[obstacle.id()].with_in_funnel;
      if (target_calculate_infos[id].with_in_funnel) {
        bool invalid_oncoming_target =
            (obstacle.velocity_flu().x() < k_oncoming_target_speed_threshold &&
             (target_calculate_infos[obstacle.id()]
                      .overlap_infos.current_path_overlap_infos
                      .obstalce_occupancy < 0.3 ||
              subject_speed_longitudinal > 5.0)) ||
            obstacle.velocity_flu().x() < -15;
        if (invalid_oncoming_target) {
          continue;
        }
        if (position_longitudinal > 0) {
          if (position_longitudinal >
              (safe_stop_distance_ + 0.5 * obstacle.length())) {
            continue;
          }
          if (select_result_infos.first_front_id == -1) {
            select_result_infos.first_front_id = id;
          } else {
            if (position_longitudinal <
                target_calculate_infos[select_result_infos.first_front_id]
                    .position.x) {
              if (target_calculate_infos[select_result_infos.first_front_id]
                      .has_same_id_last_infos) {
                if ((target_calculate_infos[select_result_infos.first_front_id]
                             .position.x -
                         position_longitudinal >
                     1.0) &&
                    (last_target_calculate_infos_[select_result_infos
                                                      .first_front_id]
                             .position.x -
                         position_longitudinal >
                     1.0)) {
                  select_result_infos.second_front_id =
                      select_result_infos.first_front_id;
                  select_result_infos.first_front_id = id;
                } else {
                  select_result_infos.second_front_id =
                      select_result_infos.first_front_id;
                }
              } else {
                if (target_calculate_infos[select_result_infos.first_front_id]
                            .position.x -
                        position_longitudinal <
                    1.0) {
                  if (fusn_src == 1) {
                    continue;
                  }
                  select_result_infos.second_front_id =
                      select_result_infos.first_front_id;
                } else {
                  select_result_infos.second_front_id =
                      select_result_infos.first_front_id;
                  select_result_infos.first_front_id = id;
                }
              }
            } else {
              if (position_longitudinal -
                      target_calculate_infos[select_result_infos.first_front_id]
                          .position.x <
                  1.0) {
                if (fusn_src == 2 ||
                    target_calculate_infos[select_result_infos.first_front_id]
                            .fusn_src == 2) {
                  select_result_infos.second_front_id =
                      select_result_infos.first_front_id;
                }
              } else {
                if (select_result_infos.second_front_id == -1) {
                  select_result_infos.second_front_id = id;
                } else {
                  if (position_longitudinal <
                      target_calculate_infos[select_result_infos
                                                 .second_front_id]
                              .position.x -
                          1.0) {
                    select_result_infos.second_front_id = id;
                  }
                }
              }
            }
          }
        } else {
          // rear target
          if (position_longitudinal <
              -1 * (k_ACCTS_vehicleLength + k_ACCTS_vehicleLengthOffset)) {
            if (select_result_infos.first_rear_id == -1) {
              select_result_infos.first_rear_id = id;
            } else {
              if (position_longitudinal >
                  target_calculate_infos[select_result_infos.first_rear_id]
                      .position.x) {
                select_result_infos.first_rear_id = id;
              }
            }
          }
        }
      } else if (target_calculate_infos[id].with_in_wide_funnel) {
        if (position_longitudinal >
            k_DynCalPrmMdlAxleDistFrnt + k_ACCTS_vehicleLengthOffset) {
          if (target_calculate_infos[id].offset > 0) {
            if (select_result_infos.first_left_front_id == -1) {
              select_result_infos.first_left_front_id = id;
            } else {
              if (position_longitudinal <
                  target_calculate_infos[select_result_infos
                                             .first_left_front_id]
                      .position.x) {
                select_result_infos.second_left_front_id =
                    select_result_infos.first_left_front_id;
                select_result_infos.first_left_front_id = id;
              } else {
                if (select_result_infos.second_left_front_id == -1) {
                  select_result_infos.second_left_front_id = id;
                } else {
                  if (select_result_infos.second_left_front_id != -1 &&
                      position_longitudinal <
                          target_calculate_infos[select_result_infos
                                                     .second_left_front_id]
                              .position.x) {
                    select_result_infos.second_left_front_id = id;
                  }
                }
              }
            }
          } else {
            if (select_result_infos.first_right_front_id == -1) {
              select_result_infos.first_right_front_id = id;
            } else {
              if (position_longitudinal <
                  target_calculate_infos[select_result_infos
                                             .first_right_front_id]
                      .position.x) {
                select_result_infos.second_right_front_id =
                    select_result_infos.first_right_front_id;
                select_result_infos.first_right_front_id = id;
              } else {
                if (select_result_infos.second_right_front_id == -1) {
                  select_result_infos.second_right_front_id = id;
                } else {
                  if (position_longitudinal <
                      target_calculate_infos[select_result_infos
                                                 .second_right_front_id]
                          .position.x) {
                    select_result_infos.second_right_front_id = id;
                  }
                }
              }
            }
          }
        } else {
          // rear target
          if (target_calculate_infos[id].offset > 0) {
            // 纵向距离在负前半车长内
            if (position_longitudinal >= -1 * (k_ACCTS_vehicleLength / 2)) {
              if (select_result_infos.first_left_side_id == -1) {
                select_result_infos.first_left_side_id = id;
              } else {
                if (position_longitudinal >
                    target_calculate_infos[select_result_infos
                                               .first_left_side_id]
                        .position.x) {
                  select_result_infos.first_left_side_id = id;
                }
              }
            } else if (position_longitudinal <
                           -1 * (k_ACCTS_vehicleLength / 2) &&
                       position_longitudinal >=
                           -1 * (k_ACCTS_vehicleLength +
                                 k_ACCTS_vehicleLengthOffset)) {
              if (select_result_infos.second_left_side_id == -1) {
                select_result_infos.second_left_side_id = id;
              } else {
                if (position_longitudinal >
                    target_calculate_infos[select_result_infos
                                               .second_left_side_id]
                        .position.x) {
                  select_result_infos.second_left_side_id = id;
                }
              }
            } else {
              if (select_result_infos.first_left_rear_id == -1) {
                select_result_infos.first_left_rear_id = id;
              } else {
                if (position_longitudinal >
                    target_calculate_infos[select_result_infos
                                               .first_left_rear_id]
                        .position.x) {
                  select_result_infos.second_left_rear_id =
                      select_result_infos.first_left_rear_id;
                  select_result_infos.first_left_rear_id = id;
                } else {
                  if (select_result_infos.second_left_rear_id == -1) {
                    select_result_infos.second_left_rear_id = id;
                  } else {
                    if (position_longitudinal >
                        target_calculate_infos[select_result_infos
                                                   .second_left_rear_id]
                            .position.x) {
                      select_result_infos.second_left_rear_id = id;
                    }
                  }
                }
              }
            }
          } else {
            if (position_longitudinal >= -1 * (k_ACCTS_vehicleLength / 2)) {
              if (select_result_infos.first_right_side_id == -1) {
                select_result_infos.first_right_side_id = id;
              } else {
                if (position_longitudinal >
                    target_calculate_infos[select_result_infos
                                               .first_right_side_id]
                        .position.x) {
                  select_result_infos.first_right_side_id = id;
                }
              }
            } else if (position_longitudinal <
                           -1 * (k_ACCTS_vehicleLength / 2) &&
                       position_longitudinal >=
                           -1 * (k_ACCTS_vehicleLength +
                                 k_ACCTS_vehicleLengthOffset)) {
              if (select_result_infos.second_right_side_id == -1) {
                select_result_infos.second_right_side_id = id;
              } else {
                if (position_longitudinal >
                    target_calculate_infos[select_result_infos
                                               .second_right_side_id]
                        .position.x) {
                  select_result_infos.second_right_side_id = id;
                }
              }
            } else {
              if (select_result_infos.first_right_rear_id == -1) {
                select_result_infos.first_right_rear_id = id;
              } else {
                if (position_longitudinal >
                    target_calculate_infos[select_result_infos
                                               .first_right_rear_id]
                        .position.x) {
                  select_result_infos.second_right_rear_id =
                      select_result_infos.first_right_rear_id;
                  select_result_infos.first_right_rear_id = id;
                } else {
                  if (select_result_infos.second_right_rear_id == -1) {
                    select_result_infos.second_right_rear_id = id;
                  } else {
                    if (position_longitudinal >
                        target_calculate_infos[select_result_infos
                                                   .second_right_rear_id]
                            .position.x) {
                      select_result_infos.second_right_rear_id = id;
                    }
                  }
                }
              }
            }
          }
        }
        // cut in
      }
    }
  }
  ModifyStopSelectResultInfos(select_result_infos, target_calculate_infos);
  OutputSelectResultInfos(stop_result_infos_, cruise_target_id);
  last_target_calculate_infos_ = target_calculate_infos;
  return true;
}

void CruiseTargetSelect::ModifyStopSelectResultInfos(
    const SelectResultInfos& select_result_infos,
    std::unordered_map<int32_t, OneTargetCalculateinfos>&
        target_calculate_infos) {
  stop_result_infos_ = select_result_infos;
  ADEBUG << "last_stop_result_infos_.first_front_id "
         << last_stop_result_infos_.first_front_id;
  if (last_stop_result_infos_.first_front_id != -1 &&
      last_stop_result_infos_.first_front_id !=
          stop_result_infos_.first_front_id &&
      ((stop_result_infos_.first_front_id != -1 &&
        std::abs(target_calculate_infos[stop_result_infos_.first_front_id]
                     .position.y) >
            std::abs(
                target_calculate_infos[last_stop_result_infos_.first_front_id]
                    .position.y)) ||
       stop_result_infos_.first_front_id == -1) &&
      acc_state_is_stand_ &&
      (vehicle_state_->has_linear_velocity() &&
       vehicle_state_->linear_velocity() < 1.0)) {
    stop_result_infos_ = last_stop_result_infos_;
  }
  last_stop_result_infos_ = stop_result_infos_;
}

double CruiseTargetSelect::SafeStopDistance(
    const double speed, const double accel, const double kappa,
    const double safe_min_accel, const double safe_max_accel,
    const double safe_min_jerk, const double safe_max_jerk) {
  const std::vector<double> coefs_limit{0.0, 0.3, 0.6, 1.0};
  const std::vector<double> radius_limit{6.0, 50.0, 100.0, 200.0};
  const double radius = SafeDivide(1.0, std::abs(kappa), 0.0001);
  const double coef =
      common::math::InterpolationOne(radius, radius_limit, coefs_limit);
  double min_time_accel_to_zero = 0.0;
  double max_time_accel_to_zero = 0.0;
  ADEBUG << "speed " << speed << " accel " << accel << " kappa " << kappa
         << " R " << radius << " coef " << coef;
  if (accel > 0.1) {
    min_time_accel_to_zero =
        std::abs(0.0 - accel) / std::max(std::abs(safe_min_jerk), 0.001);
    max_time_accel_to_zero =
        std::abs(0.0 - accel) / std::max(std::abs(safe_max_jerk), 0.001);
  }
  ADEBUG << "min_time_accel_to_zero " << min_time_accel_to_zero
         << " max_time_accel_to_zero " << max_time_accel_to_zero;
  const double min_speed_accel_to_zero =
      speed + 0.5 * accel * min_time_accel_to_zero;
  const double max_speed_accel_to_zero =
      speed + 0.5 * accel * max_time_accel_to_zero;
  ADEBUG << "min_speed_accel_to_zero " << min_speed_accel_to_zero
         << " max_speed_accel_to_zero " << max_speed_accel_to_zero;
  const double min_distance_accel_to_zero =
      speed * min_time_accel_to_zero +
      0.5 * (accel * 0.5) * std::pow(min_time_accel_to_zero, 2);
  const double max_distance_accel_to_zero =
      speed * max_time_accel_to_zero +
      0.5 * (accel * 0.5) * std::pow(max_time_accel_to_zero, 2);
  ADEBUG << "min_distance_accel_to_zero " << min_distance_accel_to_zero
         << " max_distance_accel_to_zero " << max_distance_accel_to_zero;
  const double safe_min_distance =
      std::pow(min_speed_accel_to_zero, 2) /
          std::max((2.0 * std::abs(safe_min_accel)), 0.01) +
      min_distance_accel_to_zero;
  const double safe_max_distance =
      std::pow(max_speed_accel_to_zero, 2) /
          std::max((2.0 * std::abs(safe_max_accel)), 0.01) +
      max_distance_accel_to_zero;
  ADEBUG << "safe_min_distance " << safe_min_distance << " safe_max_distance "
         << safe_max_distance;
  double distance = safe_min_distance +
                    (safe_max_distance - safe_min_distance) * coef +
                    vehicle_param_.front_edge_to_center();
  if (radius < 200.0) {
    distance = std::min(
        std::max(distance, 10.0 + vehicle_param_.front_edge_to_center()),
        150.0);
  } else {
    distance = std::max(
        50.0,
        std::min(150.0, speed * 8 + vehicle_param_.front_edge_to_center()));
  }
  return distance;
}

double CruiseTargetSelect::ConstDecelerateToStop(const double current_v,
                                                 const double current_a,
                                                 const double const_dec_a,
                                                 const double const_dec_jerk,
                                                 const double const_inc_jerk) {
  double t1 = 0.0;
  double t2 = 0.0;
  double v1 = 0.0;
  double v2 = 0.0;
  double s1 = 0.0;
  double s2 = 0.0;
  double s3 = 0.0;
  if (current_a > const_dec_a) {
    t1 = (current_a - const_dec_a) / std::max(std::abs(const_dec_jerk), 0.01);
  } else {
    t1 = (const_dec_a - current_a) / std::max(std::abs(const_inc_jerk), 0.01);
  }
  v1 = current_v + 0.5 * (current_a + const_dec_a) * t1;
  v1 = std::max(v1, 0.0);
  s1 = 0.5 * (current_v + v1) * t1;
  t2 = std::abs(0 - const_dec_a) / std::max(std::abs(const_inc_jerk), 0.01);
  s2 = 0.5 * std::abs(0 - const_dec_a) * std::pow(t2, 2);
  if (v1 > 0.0) {
    v2 = 0.5 * std::abs(0 - const_dec_a) * t2;
  }
  s3 = std::abs(std::pow(v2, 2) - std::pow(v1, 2)) /
       std::max(std::abs(2 * const_dec_a), 0.01);
  return s1 + s2 + s3;
}

void CruiseTargetSelect::SetTargetInfos(
    const std::tuple<double, double, double, double>& target_offset_tuple,
    const perception::PerceptionObstacle& obstacle, TargetInfos* target_infos) {
  target_infos->lat_distance = std::get<1>(target_offset_tuple);
  target_infos->lat_distance_var = 0;
  target_infos->obstacle_width = obstacle.width();
  target_infos->obstacle_width_var = 0;
  target_infos->obstacle_corridor_width = std::abs(
      std::get<1>(target_offset_tuple) - std::get<3>(target_offset_tuple));
  target_infos->obstacle_corridor_width_var = 0.0;
}

void CruiseTargetSelect::InitTargetOverlapInfos(
    TargetOverlapInfos* target_overlap_infos) {
  target_overlap_infos->overlap = 0;
  target_overlap_infos->overlap_var = 0;
  target_overlap_infos->trajectory_occupancy = 0;
  target_overlap_infos->trajectory_occupancy_var = 0;
  target_overlap_infos->obstalce_occupancy = 0;
  target_overlap_infos->obstalce_occupancy_var = 0;
}

void CruiseTargetSelect::OutputSelectResultInfos(
    const SelectResultInfos& select_result_infos,
    std::vector<int32_t>* cruise_target_id) {
  cruise_target_id->clear();
  *cruise_target_id = {select_result_infos.first_front_id,
                       select_result_infos.second_front_id,
                       select_result_infos.first_left_front_id,
                       select_result_infos.second_left_front_id,
                       select_result_infos.first_right_front_id,
                       select_result_infos.second_right_front_id,
                       select_result_infos.cut_in_id,
                       select_result_infos.first_left_side_id,
                       select_result_infos.second_left_side_id,
                       select_result_infos.first_right_side_id,
                       select_result_infos.second_right_side_id,
                       select_result_infos.first_left_rear_id,
                       select_result_infos.second_left_rear_id,
                       select_result_infos.first_right_rear_id,
                       select_result_infos.second_right_rear_id,
                       select_result_infos.first_rear_id};
}

void CruiseTargetSelect::ResetSelectResultInfos(
    SelectResultInfos* select_result_infos) {
  select_result_infos->first_front_id = -1;
  select_result_infos->second_front_id = -1;
  select_result_infos->first_left_front_id = -1;
  select_result_infos->first_right_front_id = -1;
  select_result_infos->second_left_front_id = -1;
  select_result_infos->second_right_front_id = -1;
  select_result_infos->cut_in_id = -1;
  select_result_infos->first_left_side_id = -1;
  select_result_infos->second_left_side_id = -1;
  select_result_infos->first_right_side_id = -1;
  select_result_infos->second_right_side_id = -1;
  select_result_infos->first_left_rear_id = -1;
  select_result_infos->second_left_rear_id = -1;
  select_result_infos->first_right_rear_id = -1;
  select_result_infos->second_right_rear_id = -1;
  select_result_infos->first_rear_id = -1;
}

bool CruiseTargetSelect::UpdateMaturity(
    const int fusn_src, const double target_position_longitudinal,
    const bool snsr_data_valid) {
  bool maturity_ok = true;
  UNUSED(fusn_src);
  UNUSED(target_position_longitudinal);
  UNUSED(snsr_data_valid);
  // if (maturity_counters_.Counter(snsr_data_valid) >= 2) {
  //   maturity_ok = true;
  // }
  return maturity_ok;
}

bool CruiseTargetSelect::UpdateJumpMaturity(
    const double target_position_longitudinal,
    const double last_target_position_longitudinal,
    const double target_position_lateral,
    const double last_target_position_lateral,
    const bool has_same_id_last_infos) {
  bool jump_ok = false;
  if (has_same_id_last_infos) {
    if ((target_position_longitudinal - last_target_position_longitudinal) <
            3.0 &&
        std::abs(target_position_lateral - last_target_position_lateral) <
            3.0) {
      jump_ok = true;
    }
  }
  return jump_ok;
}

bool CruiseTargetSelect::UpdateCoastedTimer(const int snsr_data_sts) {
  UNUSED(snsr_data_sts);
  bool coasting_ok = false;
  return coasting_ok;
}

bool CruiseTargetSelect::UpdateUseYawPlusInLChange(const bool left_dicator,
                                                   const bool right_dicator,
                                                   const bool lane_change) {
  UNUSED(left_dicator);
  UNUSED(right_dicator);
  UNUSED(lane_change);
  bool use_yaw_plus_lane_change = false;
  return use_yaw_plus_lane_change;
}

CruiseTargetSelect::TargetValidatorInfos CruiseTargetSelect::TargetValidator(
    const int fusn_src, const int snsr_data_sts,
    const double target_position_longitudinal, const int target_type,
    const double target_speed, const bool maturity_ok, const bool jump_ok,
    const bool coasting_ok) {
  TargetValidatorInfos target_validator_infos{};
  target_validator_infos.target_quality_valid =
      QualityValid(fusn_src, snsr_data_sts, maturity_ok, jump_ok, coasting_ok);
  std::pair<bool, bool> moving_valid = MovingValid(target_type, target_speed);
  target_validator_infos.target_moving_valid = std::get<0>(moving_valid);
  target_validator_infos.on_coming = std::get<1>(moving_valid);
  target_validator_infos.target_stationary_valid = StationaryValid();
  target_validator_infos.target_motor_cycle_valid = ValidMotorcycle();
  target_validator_infos.target_car_valid = CarValid();
  target_validator_infos.target_valid =
      target_validator_infos.target_quality_valid &&
      target_position_longitudinal < k_ACCTS_MaxSelectionRange &&
      (target_validator_infos.target_moving_valid ||
       target_validator_infos.target_stationary_valid);

  return target_validator_infos;
}

bool CruiseTargetSelect::QualityValid(const int fusn_src,
                                      const int snsr_data_sts,
                                      const bool maturity_ok,
                                      const bool jump_ok,
                                      const bool coasting_ok) {
  bool target_quality_valid = maturity_ok && jump_ok &&
                              (fusn_src == 2 || fusn_src == 1) &&
                              (coasting_ok || snsr_data_sts == 5);
  return target_quality_valid;
}

std::pair<bool, bool> CruiseTargetSelect::MovingValid(
    const int target_type, const double target_speed) {
  bool target_moving_valid = target_type != 0;
  // if (subject_kappa_counter_ > 10) {
  //   target_moving_valid = target_speed > -20.0 && target_type != 0;
  // }
  const bool on_coming =
      target_moving_valid && target_speed < k_oncoming_target_speed_threshold;
  return std::make_pair(target_moving_valid, on_coming);
}

bool CruiseTargetSelect::StationaryValid() {
  const bool target_stationary_valid = false;
  return target_stationary_valid;
}

bool CruiseTargetSelect::ValidMotorcycle() {
  const bool target_motor_cycle_valid = false;
  return target_motor_cycle_valid;
}

bool CruiseTargetSelect::CarValid() {
  const bool target_car_valid = false;
  return target_car_valid;
}

std::tuple<double, double, double, double>
CruiseTargetSelect::CalculateTargetOffset(
    const double target_position_longitudinal,
    const double target_position_lateral, const double target_orientation,
    const double target_speed_lateral, const double target_width,
    const double target_length, const double c0, const bool target_valid,
    const double steer_angle, const double subject_speed_longitudinal) {
  common::math::Vec2d target_center(target_position_longitudinal,
                                    target_position_lateral);
  common::math::Box2d target_box(target_center, target_orientation,
                                 target_length, target_width);
  std::vector<double> x{};
  std::vector<double> y{};
  if (!target_box.GetAllCorners().empty()) {
    for (auto i : target_box.GetAllCorners()) {
      x.emplace_back(i.x());
      y.emplace_back(i.y());
    }
    x.emplace_back(target_position_longitudinal);
    y.emplace_back(target_position_lateral);
  } else {
    x = {1000, 1000, 1000, 1000, target_position_longitudinal};
    y = {1000, 1000, 1000, 1000, target_position_lateral};
  }

  UNUSED(target_length);
  double offset_deriv = 0;
  double kappa_half = c0;
  double c1 = 0;
  double c1_factor = CalculateC1Factor();
  std::tuple<double, double, double> offset{100, 100, 100};
  if (!target_valid) {
    return std::make_tuple(offset_deriv, 100.0, 100.0, 100.0);
  }
  const bool use_kappa_all = true;
  if (!use_kappa_all) {
    if (std::abs(steer_angle * 57.3) >= k_ACCTS_MinStrAngleForNonline ||
        subject_speed_longitudinal * 3.6 < 6.0 ||
        std::abs(2 * kappa_half) < 0.000333333) {
      kappa_half = 0;
      c1 = 0;
      c1_factor = 0;
    }
  } else {
    if (std::abs(2 * kappa_half) < 0.000333333) {
      kappa_half = 0;
      c1 = 0;
      c1_factor = 0;
    }
  }
  offset = Offset(x, y, target_orientation, kappa_half, c1, c1_factor);
  ADEBUG << "steer_angle " << steer_angle * 57.3 << " min_offset "
         << std::get<0>(offset) << " max_offset " << std::get<2>(offset)
         << " center_offset " << std::get<1>(offset);
  UNUSED(target_speed_lateral);
  return std::make_tuple(offset_deriv, std::get<0>(offset), std::get<1>(offset),
                         std::get<2>(offset));
}

std::tuple<double, double, double> CruiseTargetSelect::Offset(
    const std::vector<double>& x, const std::vector<double>& y,
    const double orientation, const double c0, const double c1,
    const double c1_factor) {
  double min_offset = 100.0;
  double max_offset = 100.0;
  double center_offset = 100.0;
  min_offset =
      CalculateYawPlusOffset(orientation, x[0], y[0], c0, c1, c1_factor);
  max_offset =
      CalculateYawPlusOffset(orientation, x[0], y[0], c0, c1, c1_factor);
  ADEBUG << "i " << 0 << " tmp_offset " << min_offset;
  for (size_t i = 1; i < x.size(); i++) {
    double tmp_offset =
        CalculateYawPlusOffset(orientation, x[i], y[i], c0, c1, c1_factor);
    if (std::abs(tmp_offset) < std::abs(min_offset)) {
      min_offset = tmp_offset;
    }
    if (std::abs(tmp_offset) > std::abs(max_offset)) {
      max_offset = tmp_offset;
    }
    ADEBUG << "i " << i << " tmp_offset " << tmp_offset;
  }
  center_offset = CalculateYawPlusOffset(orientation, x.back(), y.back(), c0,
                                         c1, c1_factor);
  return std::make_tuple(min_offset, center_offset, max_offset);
}

double CruiseTargetSelect::CalculateC1Factor() {
  double c1_factor = 0;
  return c1_factor;
}

double CruiseTargetSelect::CalculateYawPlusOffset(
    const double target_orientation, const double target_position_longitudinal,
    const double target_position_lateral, const double c0, const double c1,
    const double c1_factor) {
  UNUSED(c1);
  const double c1_tmp =
      SafeDivide(c1_factor * (std::tan(target_orientation) -
                              2 * c0 * target_position_longitudinal),
                 3 * std::pow(target_position_longitudinal, 2), 0.0001);
  double offset = target_position_lateral -
                  c0 * std::pow(target_position_longitudinal, 2) -
                  c1_tmp * std::pow(target_position_longitudinal, 3);
  return offset;
}

SelectFunnel CruiseTargetSelect::CalculateFunnel(
    const double lane_width, const double target_position_longitudinal,
    const double subject_speed_longitudinal, const bool left_indicator,
    const bool right_indicator, const double lane_position,
    const bool on_coming) {
  SelectFunnel select_funnel{};
  select_funnel.outer_left = 0;
  select_funnel.outer_right = 0;
  select_funnel.inner_left = 0;
  select_funnel.inner_right = 0;
  select_funnel.wide_inner_left = 0;
  select_funnel.wide_inner_right = 0;
  select_funnel.wide_outer_left = 0;
  select_funnel.wide_outer_right = 0;
  const bool low_conf_funnels =
      target_position_longitudinal > k_ACCTS_MaxRangeHighTsQly;

  const std::pair<int, double> subject_speed_binary_search_pair =
      BinarySearch(subject_speed_longitudinal, k_ACCTS_FunnelOffsetSpeed);
  const double normal = InterpolationBinarySearch(
      subject_speed_binary_search_pair.first,
      subject_speed_binary_search_pair.second, k_ACCTS_FunnelOffsetNormal);
  const double bigger = InterpolationBinarySearch(
      subject_speed_binary_search_pair.first,
      subject_speed_binary_search_pair.second, k_ACCTS_FunnelOffsetOvertaking);
  const double smaller =
      InterpolationBinarySearch(subject_speed_binary_search_pair.first,
                                subject_speed_binary_search_pair.second,
                                k_ACCTS_InnerFunnelOffsetOpposite);
  const double smaller_inner =
      InterpolationBinarySearch(subject_speed_binary_search_pair.first,
                                subject_speed_binary_search_pair.second,
                                k_ACCTS_InnerFunnelOffsetOpposite);
  const double bigger_inner =
      InterpolationBinarySearch(subject_speed_binary_search_pair.first,
                                subject_speed_binary_search_pair.second,
                                k_ACCTS_InnerFunnelOffsetOvertaking);
  double outer_left_funnel_offset = 0;
  double outer_right_funnel_offset = 0;
  double inner_left_funnel_offset = 0;
  double inner_right_funnel_offset = 0;
  if (left_indicator) {
    outer_left_funnel_offset = bigger;
    inner_left_funnel_offset = bigger_inner;
  } else if (right_indicator) {
    outer_left_funnel_offset = smaller;
    inner_left_funnel_offset = smaller_inner;
  } else {
    outer_left_funnel_offset = normal;
    inner_left_funnel_offset = 0;
  }
  if (right_indicator) {
    outer_right_funnel_offset = bigger;
    inner_right_funnel_offset = bigger_inner;
  } else if (left_indicator) {
    outer_right_funnel_offset = smaller;
    inner_right_funnel_offset = smaller_inner;
  } else {
    outer_right_funnel_offset = normal;
    inner_right_funnel_offset = 0;
  }
  if (on_coming && !left_indicator && !right_indicator) {
    outer_left_funnel_offset = smaller - 0.1;
    inner_left_funnel_offset = smaller_inner - 0.05;
    outer_right_funnel_offset = smaller - 0.1;
    inner_right_funnel_offset = smaller_inner - 0.05;
  }

  // const double outer_left_funnel_offset = outer_left_funnel_offset_.RateLimit(
  //     outer_left_raw, k_ACCTS_FunnelGrowthRate, k_ACCTS_FunnelShrinkRate, false,
  //     0.1);
  // const double outer_right_funnel_offset = outer_right_funnel_offset_.RateLimit(
  //     outer_right_raw, k_ACCTS_FunnelGrowthRate, k_ACCTS_FunnelShrinkRate,
  //     false, 0.1);
  // const double inner_left_funnel_offset = inner_left_funnel_offset_.RateLimit(
  //     inner_left_raw, k_ACCTS_FunnelGrowthRate, k_ACCTS_FunnelShrinkRate, false,
  //     0.1);
  // const double inner_right_funnel_offset = inner_right_funnel_offset_.RateLimit(
  //     inner_right_raw, k_ACCTS_FunnelGrowthRate, k_ACCTS_FunnelShrinkRate,
  //     false, 0.1);

  const std::pair<int, double> target_position_longitudinal_binary_search_pair =
      BinarySearch(target_position_longitudinal, k_ACCTS_IP_FunnelIP_acc);
  const double outer_funnel =
      InterpolationBinarySearch(
          target_position_longitudinal_binary_search_pair.first,
          target_position_longitudinal_binary_search_pair.second,
          k_ACCTS_IP_OuterFunnelWidths) *
      lane_width;
  const double inner_funnel =
      InterpolationBinarySearch(
          target_position_longitudinal_binary_search_pair.first,
          target_position_longitudinal_binary_search_pair.second,
          k_ACCTS_IP_InnerFunnelWidths) *
      lane_width;

  select_funnel.inner_left =
      inner_left_funnel_offset + 0.5 * inner_funnel - lane_position;
  select_funnel.inner_right =
      -1 * (inner_right_funnel_offset + 0.5 * inner_funnel) - lane_position;
  select_funnel.outer_left =
      outer_left_funnel_offset + 0.5 * outer_funnel - lane_position;
  select_funnel.outer_right =
      -1 * (outer_right_funnel_offset + 0.5 * outer_funnel) - lane_position;
  if (low_conf_funnels) {
    select_funnel.inner_left =
        0.5 * inner_funnel * k_ACCTS_AdjacentLaneMultFactorInner * 0.5;
    select_funnel.inner_right =
        (-1 * (inner_right_funnel_offset + 0.5 * inner_funnel) -
         lane_position) *
        k_ACCTS_RightKeepFunnelScalingLowTsQlyRegion;
    select_funnel.outer_left = 0.5 * outer_funnel *
                               k_ACCTS_AdjacentLaneMultFactorInner *
                               k_ACCTS_AdjacentLaneMultFactorOuter * 0.5;
    select_funnel.outer_right =
        -1 * (outer_right_funnel_offset + 0.5 * outer_funnel) - lane_position;
  }
  select_funnel.wide_inner_left =
      inner_funnel * 0.5 * k_ACCTS_AdjacentLaneMultFactorInner * 0.8;
  select_funnel.wide_inner_right = -1 * select_funnel.wide_inner_left;
  select_funnel.wide_outer_left =
      outer_funnel * 0.5 * k_ACCTS_AdjacentLaneMultFactorInner * 0.85;
  select_funnel.wide_outer_right = -1 * select_funnel.wide_outer_left;
  ADEBUG << "outer_left " << select_funnel.outer_left << " outer_right "
         << select_funnel.outer_right << " inner_left "
         << select_funnel.inner_left << " inner_right "
         << select_funnel.inner_right << " wide_inner_left "
         << select_funnel.wide_inner_left << " wide_inner_right "
         << select_funnel.wide_inner_right << " wide_outer_left "
         << select_funnel.wide_outer_left << " wide_outer_right "
         << select_funnel.wide_outer_right;
  return select_funnel;
}

bool CruiseTargetSelect::CheckInWidePath(
    const SelectFunnel& select_funnel, const OverlapInfos& overlap_infos,
    const perception::PerceptionObstacle& obstacle, const double offset,
    const int32_t id, const bool has_same_id_last_infos) {
  bool with_in_wide_funnel = false;
  const std::vector<double> k_reference_distance_adjacent_xaxis{40, 70, 100,
                                                                130};
  const std::vector<double> k_obstalce_occupancy_adjacent_yaxis{0.1, 0.15, 0.2,
                                                                0.25};
  const double obstalce_occupancy = common::math::InterpolationOne(
      obstacle.position_flu().x(), k_reference_distance_adjacent_xaxis,
      k_obstalce_occupancy_adjacent_yaxis);
  if (has_same_id_last_infos) {
    with_in_wide_funnel =
        overlap_infos.adjacent_path_overlap_infos.obstalce_occupancy >
            obstalce_occupancy &&
        (offset >= select_funnel.wide_outer_right &&
         offset <= select_funnel.wide_outer_left) &&
        (last_target_calculate_infos_[id].offset >=
             last_target_calculate_infos_[id].select_funnel.wide_outer_right &&
         last_target_calculate_infos_[id].offset <=
             last_target_calculate_infos_[id].select_funnel.wide_outer_left);
  }
  return with_in_wide_funnel;
}

bool CruiseTargetSelect::CheckInPath(
    const SelectFunnel& select_funnel, const OverlapInfos& overlap_infos,
    const perception::PerceptionObstacle& obstacle, const double offset,
    const int32_t id, const bool has_same_id_last_infos) {
  bool with_in_funnel = false;
  bool using_inner_funnel = false;
  const std::vector<double> k_reference_distance_current_xaxis{40, 70, 100,
                                                               130};
  const std::vector<double> k_obstalce_occupancy_current_yaxis{0.1, 0.15, 0.2,
                                                               0.25};
  const double obstalce_occupancy = common::math::InterpolationOne(
      obstacle.position_flu().x(), k_reference_distance_current_xaxis,
      k_obstalce_occupancy_current_yaxis);
  if (has_same_id_last_infos) {
    if (using_inner_funnel) {
      with_in_funnel =
          (offset >= select_funnel.inner_right &&
           offset <= select_funnel.inner_left) &&
          (last_target_calculate_infos_[id].offset >=
               last_target_calculate_infos_[id].select_funnel.inner_right &&
           last_target_calculate_infos_[id].offset <=
               last_target_calculate_infos_[id].select_funnel.inner_left);
    } else {
      with_in_funnel =
          overlap_infos.current_path_overlap_infos.obstalce_occupancy >
              obstalce_occupancy &&
          (offset >= select_funnel.outer_right &&
           offset <= select_funnel.outer_left) &&
          (last_target_calculate_infos_[id].offset >=
               last_target_calculate_infos_[id].select_funnel.outer_right &&
           last_target_calculate_infos_[id].offset <=
               last_target_calculate_infos_[id].select_funnel.outer_left);
    }
  }
  return with_in_funnel;
}

// NOLINTBEGIN
template <class T, std::size_t N>
double CruiseTargetSelect::InterpolationBinarySearch(const int index,
                                                     const double fraction,
                                                     std::array<T, N> ydat) {
  const int ydat_size = static_cast<int>(ydat.size());
  double y = 0;
  if (index <= 0) {
    y = ydat[0];
  } else if (index >= (ydat_size - 1)) {
    y = ydat[ydat_size - 1];
  } else {
    y = ydat[index] + (ydat[index + 1] - ydat[index]) * fraction;
  }
  return y;
}

template <class T, std::size_t N>
std::pair<int, double> CruiseTargetSelect::BinarySearch(const double u,
                                                        std::array<T, N> bp) {
  int index = 0;
  double fraction = 0;
  const int bp_size = static_cast<int>(bp.size());
  if (u <= bp[0]) {
    index = 0;
    fraction = 0;
  } else if (u >= bp[bp_size - 1]) {
    index = bp_size - 2;
    fraction = 1;
  } else {
    int index_left = 0;
    int index_right = bp_size - 1;
    int index_tmp = bp_size - 1;
    while (index_right - index_left > 1) {
      if (u < bp[index_tmp]) {
        index_right = index_tmp;
      } else {
        index_left = index_tmp;
      }
      index_tmp = static_cast<int>(std::ceil((index_right + index_left) / 2));
    }
    fraction = (u - bp[index_left]) / (bp[index_left + 1] - bp[index_left]);
    index = index_left;
  }
  return std::make_pair(index, fraction);
}

// NOLINTEND
double CruiseTargetSelect::SafeDivide(const double nom, const double denom,
                                      const double threshold) {
  double denominator = 0.0;
  if (fabs(denom) > threshold) {
    denominator = denom;
  } else if (denom > 0) {
    denominator = threshold;
  } else {
    denominator = -threshold;
  }
  return nom / denominator;
}

double CruiseTargetSelect::MinLatDistance(const double x_in, const double y_in,
                                          const double x_r, const double y_r) {
  std::vector<double> v_r_point(2, 0);
  v_r_point[0] = x_r - x_in;
  v_r_point[1] = y_r - y_in;
  return CartSign(y_in - y_r) * std::sqrt(CartDotProduct(v_r_point, v_r_point));
}

std::vector<double> CruiseTargetSelect::CartProjectPoint(
    const double x_in, const double y_in, const double a0_ref,
    const double a1_ref, const double a2_ref, const double a3_ref,
    const double error) {
  std::vector<double> project_point(3, 0);
  std::vector<double> xy_project(2, 0);
  double x_start = -100;
  double x_end = 150;
  double y_start = CartY(x_start, a0_ref, a1_ref, a2_ref, a3_ref);
  double y_end = CartY(x_end, a0_ref, a1_ref, a2_ref, a3_ref);

  std::vector<double> v_project_point_start{x_start - x_in, y_start - y_in};
  std::vector<double> v_project_point_end{x_end - x_in, y_end - y_in};
  std::vector<double> v_project_point_start_end{x_end - x_start,
                                                y_end - y_start};
  double dp_pps_pspe =
      CartDotProduct(v_project_point_start, v_project_point_start_end);
  double dp_ppe_pspe =
      CartDotProduct(v_project_point_end, v_project_point_start_end);

  double x_mid = (0);
  double y_mid = (0);
  std::vector<double> v_project_point_mid(2, 0);
  std::vector<double> v_project_point_mid_start(2, 0);
  std::vector<double> v_project_point_end_mid(2, 0);

  double dp_pps_pmps = 0;
  double dp_ppm_pmps = 0;
  double dp_ppm_pepm = 0;
  double dp_ppe_pepm = 0;
  int i = 0;
  if (CartSign(dp_pps_pspe * dp_ppe_pspe) < 0) {
    for (i = 1; i < 100; i++) {
      x_mid = (x_start + x_end) / 2;
      y_mid = CartY(x_mid, a0_ref, a1_ref, a2_ref, a3_ref);

      v_project_point_mid[0] = x_mid - x_in;
      v_project_point_mid[1] = y_mid - y_in;

      v_project_point_mid_start[0] = x_start - x_mid;
      v_project_point_mid_start[1] = y_start - y_mid;

      v_project_point_end_mid[0] = x_mid - x_end;
      v_project_point_end_mid[1] = y_mid - y_end;

      dp_pps_pmps =
          CartDotProduct(v_project_point_start, v_project_point_mid_start);
      dp_ppm_pmps =
          CartDotProduct(v_project_point_mid, v_project_point_mid_start);

      dp_ppe_pepm =
          CartDotProduct(v_project_point_end, v_project_point_end_mid);
      dp_ppm_pepm =
          CartDotProduct(v_project_point_mid, v_project_point_end_mid);
      if (CartSign(dp_pps_pmps * dp_ppm_pmps) < 0 &&
          CartSign(dp_ppm_pepm * dp_ppe_pepm) > 0) {
        x_end = x_mid;
        y_end = y_mid;
        v_project_point_end[0] = v_project_point_mid[0];
        v_project_point_end[1] = v_project_point_mid[1];
      } else if (CartSign(dp_pps_pmps * dp_ppm_pmps) > 0 &&
                 CartSign(dp_ppm_pepm * dp_ppe_pepm) < 0) {
        x_start = x_mid;
        y_start = y_mid;
        v_project_point_start[0] = v_project_point_mid[0];
        v_project_point_start[1] = v_project_point_mid[1];
      } else if (dp_pps_pmps == 0) {
        xy_project[0] = x_start;
        xy_project[1] = y_start;
      } else if (dp_ppe_pepm == 0) {
        xy_project[0] = x_end;
        xy_project[1] = y_end;
      } else {
        xy_project[0] = x_mid;
        xy_project[1] = y_mid;
        break;
      }
      if (sqrt(CartDotProduct(v_project_point_mid_start,
                              v_project_point_mid_start)) < error) {
        xy_project[0] = x_mid;
        xy_project[1] = y_mid;
        break;
      }
    }
    project_point[0] = xy_project[0];
    project_point[1] = xy_project[1];
    project_point[2] = i;

  } else {
    xy_project[0] = x_in;
    xy_project[1] = CartY(x_in, a0_ref, a1_ref, a2_ref, a3_ref);
    project_point[0] = xy_project[0];
    project_point[1] = xy_project[1];
    project_point[2] = 0;
  }
  return project_point;
}

int CruiseTargetSelect::CartSign(const double val) {
  if (val >= 0.000001) {
    return 1;
  }
  if (val <= -0.000001) {
    return -1;
  }
  return 0;
}

double CruiseTargetSelect::CartY(const double x, const double a0_ref,
                                 const double a1_ref, const double a2_ref,
                                 const double a3_ref) {
  return (a0_ref + a1_ref * x + a2_ref * x * x + a3_ref * x * x * x);
}

double CruiseTargetSelect::CartDotProduct(std::vector<double> A,
                                          std::vector<double> B) {
  return (A[0] * B[0] + A[1] * B[1]);
}

double CruiseTargetSelect::MinDistanceL(const double x_in, const double y_in,
                                        const double a0_ref,
                                        const double a1_ref,
                                        const double a2_ref,
                                        const double a3_ref,
                                        const double error) {
  std::vector<double> project_point =
      CartProjectPoint(x_in, y_in, a0_ref, a1_ref, a2_ref, a3_ref, error);
  double l = MinLatDistance(x_in, y_in, project_point[0], project_point[1]);
  return l;
}

bool CruiseTargetSelect::FiltByObstacleInfos(
    const TL::perception::PerceptionObstacle& obstacle) {
  bool filt_obstacle_flag = false;
  if (!obstacle.has_id()) {
    filt_obstacle_flag = true;
  }
  // if (!obstacle.has_type()) {
  //   filt_obstacle_flag = true;
  // }
  if (obstacle.position_flu().x() < -80 || obstacle.position_flu().x() > 150) {
    filt_obstacle_flag = true;
  }
  if (obstacle.type() != perception::PerceptionObstacle::Type::
                             PerceptionObstacle_Type_PEDESTRIAN &&
      obstacle.type() != perception::PerceptionObstacle::Type::
                             PerceptionObstacle_Type_BICYCLE &&
      obstacle.type() != perception::PerceptionObstacle::Type::
                             PerceptionObstacle_Type_VEHICLE &&
      obstacle.type() != perception::PerceptionObstacle::Type::
                             PerceptionObstacle_Type_CYCLIST) {
    filt_obstacle_flag = true;
  }
  return filt_obstacle_flag;
}

}  // namespace planning
}  // namespace TL
