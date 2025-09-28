/*
 * Copyright (c) TL Technologies Co., Ltd. 2022. All rights reserved.
 * Description:  open_space_fine_tuning.cc
 */
#include <algorithm>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "common/math/linear_interpolation.h"
#include "common/math/math_utils.h"
#include "common/math/vec2d.h"
#include "common/status/status.h"
#include "common/time/clock.h"
#include "planning/localview/hdmap_avp_state/hdmap_avp_state.h"
#include "planning/tasks/deciders/open_space_decider/open_space_fine_tuning.h"

using TL::common::Status;
using TL::common::math::Vec2d;

namespace {
constexpr double kEpsilon = 1.0e-3;
}  // namespace

namespace TL {
namespace planning {

OpenSpaceFineTuning::OpenSpaceFineTuning(
    const TaskConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : injector_(injector),
      config_(config),
      vehicle_params_(
          TL::common::VehicleConfigHelper::GetConfig().vehicle_param()) {}

Status OpenSpaceFineTuning::Process(
    const bool is_entered_lateral_slot_domain,
    const bool is_consider_wheel_mask,
    const AVPStatus::ParkingType& parking_type,
    const planning::ParkLotInfo& park_lot,
    const OpenSpacePathInfo& open_space_path_info,
    const std::vector<std::pair<common::math::LineSegment2d, double>>&
        linked_obstacles_segments_vec,
    const std::vector<std::pair<common::math::LineSegment2d, double>>&
        high_curb_obstacles_segments_vec,
    const common::PathPoint& veh_point,
    const common::math::LineSegment2d& reference_curb,
    common::PathPoint* const end_pose_enu_ptr) {
  if (nullptr == end_pose_enu_ptr) {
    const std::string msg = "Fine tuning failed, end_pose_enu_ptr is nullptr";
    AERROR << msg;
    return Status(ErrorCode::PLANNER_PARKING_ROIDECIDER_ERROR, msg);
  }
  is_entered_lateral_slot_domain_ = is_entered_lateral_slot_domain;
  raw_end_pose_enu_ = *end_pose_enu_ptr;
  target_adc_coord_heading_ = raw_end_pose_enu_.theta() - M_PI_2;
  const bool is_lat_slot =
      (perception::ParkingLotOut::LATERAL == park_lot.park_type);
  if (parking_type != TL::planning::AVPStatus::PARKING_IN) {
    target_adc_coord_heading_ =
        (park_lot.vertices.at(3) - park_lot.vertices.at(0)).Angle();
  } else if (is_lat_slot) {
    target_adc_coord_heading_ = park_lot.is_right_side
                                    ? raw_end_pose_enu_.theta()
                                    : raw_end_pose_enu_.theta() + M_PI;
  }
  const auto& obstacles_segments_vec =
      open_space_path_info.obstacles_segments_vec;
  const auto& origin = open_space_path_info.origin;
  const auto& rotate_angle = open_space_path_info.rotate_angle;
  const auto lat_space =
      GetTopHalfLatSpace(is_lat_slot, obstacles_segments_vec);
  const double last_lat_fine_tune_dis = lat_fine_tune_dis_;
  const double last_lon_fine_tune_dis = lon_fine_tune_dis_;
  const double last_yaw_fine_tune_rad = yaw_fine_tune_rad_;
  is_fine_tune_update_ = false;
  if (FineTuneParkOutTargetBasedOnObs(parking_type, park_lot.sensor_type,
                                      park_lot.park_type, park_lot.vertices,
                                      veh_point, reference_curb)) {
    const auto fine_tune_target = GetFineTuneTarget();
    end_pose_enu_ptr->set_x(fine_tune_target.x());
    end_pose_enu_ptr->set_y(fine_tune_target.y());
    end_pose_enu_ptr->set_theta(raw_end_pose_enu_.theta() + yaw_fine_tune_rad_);
    if (std::fabs(last_lat_fine_tune_dis - lat_fine_tune_dis_) > kEpsilon ||
        std::fabs(last_lon_fine_tune_dis - lon_fine_tune_dis_) > kEpsilon ||
        std::fabs(common::math::AngleDiff(yaw_fine_tune_rad_,
                                          last_yaw_fine_tune_rad)) > kEpsilon) {
      is_fine_tune_update_ = true;
    }
    return Status::OK();
  }
  if (parking_type != TL::planning::AVPStatus::PARKING_IN) {
    return Status::OK();
  }
  FineTuneParkInTargetBasedOnObs(veh_point, park_lot, obstacles_segments_vec,
                                 linked_obstacles_segments_vec,
                                 high_curb_obstacles_segments_vec, origin,
                                 rotate_angle);
  FineTuneTargetBasedOnNarrowLatSpot(is_lat_slot, obstacles_segments_vec,
                                     lat_space);
  FineTuneTargetBasedOnEndReplanPose(
      veh_point, park_lot.is_right_side, park_lot.park_type, origin,
      target_adc_coord_heading_, lat_space, is_consider_wheel_mask,
      open_space_path_info.open_space_env_structured_info.is_parking_inwards);
  if (std::fabs(last_lat_fine_tune_dis - lat_fine_tune_dis_) > kEpsilon ||
      std::fabs(last_lon_fine_tune_dis - lon_fine_tune_dis_) > kEpsilon ||
      std::fabs(common::math::AngleDiff(yaw_fine_tune_rad_,
                                        last_yaw_fine_tune_rad)) > kEpsilon) {
    is_fine_tune_update_ = true;
  }
  const auto fine_tune_target = GetFineTuneTarget();
  end_pose_enu_ptr->set_x(fine_tune_target.x());
  end_pose_enu_ptr->set_y(fine_tune_target.y());
  is_init_park_in_ = false;
  return Status::OK();
}

Status OpenSpaceFineTuning::Reset() {
  is_entered_lateral_slot_domain_ = false;
  lon_fine_tune_dis_ = 0.0;
  lat_fine_tune_dis_ = 0.0;
  yaw_fine_tune_rad_ = 0.0;
  target_adc_coord_heading_ = 0.0;
  need_lat_fine_tune_ = false;
  has_end_replaned_ = false;
  has_narrow_lon_tune_ = false;
  pre_narrow_lon_fine_tune_dis_ = 0.0;
  is_init_park_in_ = true;
  ADEBUG << "fine tuning is reseted";
  return Status::OK();
}

common::math::Vec2d OpenSpaceFineTuning::GetFineTuneTarget() {
  Vec2d fine_tune_target{raw_end_pose_enu_.x(), raw_end_pose_enu_.y()};
  return fine_tune_target +
         lat_fine_tune_dis_ *
             common::math::Vec2d::CreateUnitVec2d(target_adc_coord_heading_) +
         lon_fine_tune_dis_ * common::math::Vec2d::CreateUnitVec2d(
                                  target_adc_coord_heading_ + M_PI_2);
}

common::math::Vec2d OpenSpaceFineTuning::GetFineTuneTarget(
    const double lat_fine_tune_dis, const double lon_fine_tune_dis) {
  Vec2d fine_tune_target{raw_end_pose_enu_.x(), raw_end_pose_enu_.y()};
  return fine_tune_target +
         lat_fine_tune_dis *
             common::math::Vec2d::CreateUnitVec2d(target_adc_coord_heading_) +
         lon_fine_tune_dis * common::math::Vec2d::CreateUnitVec2d(
                                 target_adc_coord_heading_ + M_PI_2);
}

void OpenSpaceFineTuning::FineTuneParkInTargetBasedOnObs(
    const common::PathPoint& veh_point,
    const planning::ParkLotInfo& park_lot_info,
    const std::vector<std::pair<common::math::LineSegment2d, double>>&
        original_obs_segments_pair,
    const std::vector<std::pair<common::math::LineSegment2d, double>>&
        linked_obs_segments_pair,
    const std::vector<std::pair<common::math::LineSegment2d, double>>&
        high_curb_obs_segments_pair,
    const common::math::Vec2d& origin, const double rotate_angle) {
  if (park_lot_info.sensor_type == perception::ParkingLotOut::USS) {
    return;
  }
  const bool is_lat_slot =
      (perception::ParkingLotOut::LATERAL == park_lot_info.park_type);
  const bool is_parking_lot_update = park_lot_info.is_parking_lot_update;
  const bool is_right_side = park_lot_info.is_right_side;
  const auto curr_need_lat_fine_tune = [&]() {
    if (IsBlockByInnerFs(is_lat_slot, origin, rotate_angle)) {
      return true;
    }
    constexpr double kTriggerFineTuneThetaThreshold = 0.35;
    const double angle_diff =
        common::math::AngleDiff(veh_point.theta(), raw_end_pose_enu_.theta());
    if (std::fabs(angle_diff) > kTriggerFineTuneThetaThreshold) {
      return false;
    }
    return IsTargetCollisionWithLinkedObs(is_lat_slot, is_parking_lot_update,
                                          linked_obs_segments_pair) ||
           IsTargetCollisionWithHighCurbObs(is_lat_slot, is_parking_lot_update,
                                            high_curb_obs_segments_pair);
  };
  const bool is_enable_lat_fine_tune =
      curr_need_lat_fine_tune() ||
      (need_lat_fine_tune_ && is_parking_lot_update);
  const bool is_enable_lon_fine_tune =
      is_lat_slot && (is_init_park_in_ ||
                      (park_lot_info.is_high_quality_triggered &&
                       IsBlockByInnerFs(is_lat_slot, origin, rotate_angle)));
  if (!is_enable_lat_fine_tune && !is_enable_lon_fine_tune) {
    return;
  }

  InitConfig(is_lat_slot, is_enable_lat_fine_tune, is_enable_lon_fine_tune);
  const double shift_distance =
      0.5 * vehicle_params_.length() - vehicle_params_.back_edge_to_center();
  Vec2d target_adc_center =
      Vec2d(raw_end_pose_enu_.x(), raw_end_pose_enu_.y()) +
      shift_distance * Vec2d::CreateUnitVec2d(raw_end_pose_enu_.theta());
  const auto fine_tune_obs = std::make_shared<OpenSpaceFineTuningObs>();
  for (const auto& obs_seg_pair : original_obs_segments_pair) {
    if (obs_seg_pair.second < kEpsilon) {
      continue;
    }
    fine_tune_obs->original_buffer.emplace_back(obs_seg_pair.second);
    fine_tune_obs->obs_segments_pair.emplace_back(obs_seg_pair);
    fine_tune_obs->obs_segments_pair.back().first.Transform(
        target_adc_center, target_adc_coord_heading_);
  }
  for (const auto& obs_seg_pair : high_curb_obs_segments_pair) {
    if (obs_seg_pair.second < kEpsilon) {
      continue;
    }
    fine_tune_obs->original_buffer.emplace_back(obs_seg_pair.second);
    fine_tune_obs->obs_segments_pair.emplace_back(obs_seg_pair);
    fine_tune_obs->obs_segments_pair.back().first.Transform(
        target_adc_center, target_adc_coord_heading_);
  }
  if (fine_tune_obs->obs_segments_pair.size() !=
      fine_tune_obs->original_buffer.size()) {
    AERROR << "obs_segments_pair size is not equal to original_buffer size!";
    return;
  }

  if (ValidityCheck(is_lat_slot, is_right_side, lat_fine_tune_dis_,
                    lon_fine_tune_dis_, fine_tune_obs)) {
    ADEBUG << "target pose is collision free using previous lon_fine_tune_dis "
           << lon_fine_tune_dis_ << " and previous lat_fine_tune_dis "
           << lat_fine_tune_dis_;
    return;
  }

  auto start_time = common::Clock::NowInSeconds();
  std::unordered_map<std::string, std::shared_ptr<OpenSpaceFineTuningGrid>>
      search_grids_set;
  std::priority_queue<std::pair<std::string, double>,
                      std::vector<std::pair<std::string, double>>, cmp>
      search_grids_pq;
  std::unordered_set<std::string> visited_grids;
  const auto start_grid = std::make_shared<OpenSpaceFineTuningGrid>(
      0.0, 0.0, search_config_.x_grid_resolution,
      search_config_.y_grid_resolution, search_config_.xy_bounds);
  search_grids_set.emplace(start_grid->index(), start_grid);
  search_grids_pq.emplace(start_grid->index(), 0.0);
  while (!search_grids_pq.empty()) {
    const auto grid_index = search_grids_pq.top().first;
    search_grids_pq.pop();
    const auto& grid = search_grids_set.at(grid_index);
    if (visited_grids.find(grid_index) != visited_grids.end() ||
        grid->x() < search_config_.xy_bounds[0] ||
        grid->x() > search_config_.xy_bounds[1] ||
        grid->y() < search_config_.xy_bounds[2] ||
        grid->y() > search_config_.xy_bounds[3]) {
      continue;
    }
    visited_grids.insert(grid->index());
    if (ValidityCheck(is_lat_slot, is_right_side, grid->x(), grid->y(),
                      fine_tune_obs)) {
      lat_fine_tune_dis_ = grid->x();
      lon_fine_tune_dis_ = grid->y();
      ADEBUG << "target pose is collision free after fine tune with both "
                "lon_fine_tune_dis "
             << lon_fine_tune_dis_ << " and lat_fine_tune_dis "
             << lat_fine_tune_dis_;
      AINFO << "fine tune time is "
            << common::Clock::NowInSeconds() - start_time;
      return;
    }
    AddCutOffSearchGrids(grid, is_lat_slot, fine_tune_obs->obs_segments_pair,
                         &visited_grids);
    const auto next_grids = grid->next();
    for (const auto& next_grid : next_grids) {
      if (visited_grids.find(next_grid->index()) != visited_grids.end()) {
        continue;
      }
      const double cost = search_config_.w_x * next_grid->x() * next_grid->x() +
                          search_config_.w_y * next_grid->y() * next_grid->y();
      search_grids_set.emplace(next_grid->index(), next_grid);
      search_grids_pq.emplace(next_grid->index(), cost);
    }
  }
  ADEBUG << "search collision-free grid failed";
  AINFO << "fine tune time is " << common::Clock::NowInSeconds() - start_time;
}

bool OpenSpaceFineTuning::IsBlockByInnerFs(const bool is_lat_slot,
                                           const common::math::Vec2d& origin,
                                           const double rotate_angle) {
  const auto& previous_frame = injector_->frame_history()->Latest();
  if (nullptr == previous_frame) {
    return false;
  }
  if (!previous_frame->IsVehicleStandStill() ||
      (injector_->planning_context()->planning_status().open_space().replan() &
       static_cast<uint32_t>(
           TL::planning::OpenSpaceStatus::BLOCK_BY_STATIC_OBSTACLE)) == 0U) {
    return false;
  }
  const auto& spd_collision_info =
      previous_frame->open_space_info().speed_plan_collision_info();
  auto collision_fs_segment_start =
      Vec2d(spd_collision_info.collision_fs_segment_start().x(),
            spd_collision_info.collision_fs_segment_start().y());
  auto collision_fs_segment_end =
      Vec2d(spd_collision_info.collision_fs_segment_end().x(),
            spd_collision_info.collision_fs_segment_end().y());
  collision_fs_segment_start -= origin;
  collision_fs_segment_start.SelfRotate(-rotate_angle);
  collision_fs_segment_end -= origin;
  collision_fs_segment_end.SelfRotate(-rotate_angle);
  const double slot_inner_dist_thres = is_lat_slot ? -1.0 : 1.5;
  return collision_fs_segment_start.y() < slot_inner_dist_thres ||
         collision_fs_segment_end.y() < slot_inner_dist_thres;
}

bool OpenSpaceFineTuning::IsTargetCollisionWithLinkedObs(
    const bool is_lat_slot, const bool is_parking_lot_update,
    const std::vector<std::pair<common::math::LineSegment2d, double>>&
        linked_obs_segments_pair) {
  if (!is_lat_slot || !is_parking_lot_update) {
    return false;
  }
  const auto fine_tune_target = GetFineTuneTarget();
  return common::math::CheckCollisionWithRoughVehiclePolygon2d(
      fine_tune_target.x(), fine_tune_target.y(), raw_end_pose_enu_.theta(),
      linked_obs_segments_pair);
}

bool OpenSpaceFineTuning::IsTargetCollisionWithHighCurbObs(
    const bool is_lat_slot, const bool is_parking_lot_update,
    const std::vector<std::pair<common::math::LineSegment2d, double>>&
        high_curb_obs_segments_pair) {
  if (is_lat_slot || !is_parking_lot_update) {
    return false;
  }
  const auto fine_tune_target = GetFineTuneTarget();
  return common::math::CheckCollisionWithRoughVehiclePolygon2d(
      fine_tune_target.x(), fine_tune_target.y(), raw_end_pose_enu_.theta(),
      high_curb_obs_segments_pair);
}

void OpenSpaceFineTuning::UpdateAdjustBuffer(
    const bool is_lat_slot, const double lat_fine_tune_dis,
    const double lon_fine_tune_dis, const int idx_obs,
    const std::shared_ptr<OpenSpaceFineTuningObs>& fine_tune_obs) {
  if (nullptr == fine_tune_obs) {
    AERROR << "fine_tune_obs is null";
    return;
  }
  if (idx_obs < 0 || idx_obs >= fine_tune_obs->obs_segments_pair.size()) {
    AERROR << "index is invalid";
    return;
  }
  const auto& obs_inflat_buffer = fine_tune_obs->original_buffer.at(idx_obs);
  auto& obs_seg_pair = fine_tune_obs->obs_segments_pair.at(idx_obs);
  obs_seg_pair.second = obs_inflat_buffer;
  static constexpr double extra_lat_spot_bottom_adjust_buffer = 0.1;
  static constexpr double extra_non_lat_spot_bottom_adjust_buffer = 0.05;
  // obs coordinate is already transformed
  if (is_lat_slot) {
    const double lat_dis_to_center = 0.5 * vehicle_params_.length();
    const double lon_dis_to_center = 0.5 * vehicle_params_.width();
    const double bottom_boundary = -1 * lon_dis_to_center + lon_fine_tune_dis;
    const double left_boundary =
        -1 * lat_dis_to_center - obs_inflat_buffer + lat_fine_tune_dis;
    const double right_boundary =
        lat_dis_to_center + obs_inflat_buffer + lat_fine_tune_dis;
    if (obs_seg_pair.first.start().x() < left_boundary &&
        obs_seg_pair.first.end().x() < left_boundary) {
      return;
    }
    if (obs_seg_pair.first.start().x() > right_boundary &&
        obs_seg_pair.first.end().x() > right_boundary) {
      return;
    }
    if (obs_seg_pair.first.start().y() > bottom_boundary &&
        obs_seg_pair.first.end().y() > bottom_boundary) {
      return;
    }
    obs_seg_pair.second =
        fabs(obs_seg_pair.second - kBtmLowFsBuffer) <= kEpsilon
            ? config_.open_space_roi_decider_config()
                  .lat_spot_bottom_low_fs_adjust_buffer()
            : config_.open_space_roi_decider_config()
                  .lat_spot_bottom_adjust_buffer();
    obs_seg_pair.second +=
        (is_entered_lateral_slot_domain_ ? extra_lat_spot_bottom_adjust_buffer
                                         : 0);
  } else {
    const double lon_dis_to_center = 0.5 * vehicle_params_.length();
    const double bottom_boundary = -1 * lon_dis_to_center +
                                   vehicle_params_.back_edge_to_center() -
                                   obs_inflat_buffer + lon_fine_tune_dis;
    const double up_boundary =
        lon_dis_to_center + obs_inflat_buffer + lon_fine_tune_dis;
    if (obs_seg_pair.first.start().y() > up_boundary &&
        obs_seg_pair.first.end().y() > up_boundary) {
      return;
    }
    if (obs_seg_pair.first.start().y() < bottom_boundary &&
        obs_seg_pair.first.end().y() < bottom_boundary) {
      return;
    }
    obs_seg_pair.second =
        fabs(obs_seg_pair.second - kBtmLowFsBuffer) <= kEpsilon
            ? config_.open_space_speed_optimizer_config()
                  .vertical_backward_collision_buffer()
                  .left_lateral_buffer_low_fs()
            : config_.open_space_speed_optimizer_config()
                  .vertical_backward_collision_buffer()
                  .left_lateral_buffer_bigger();
    obs_seg_pair.second += extra_non_lat_spot_bottom_adjust_buffer;
  }
}

void OpenSpaceFineTuning::LonFineTuneTargetBasedOnObs(
    const bool is_lat_slot,
    const std::vector<std::pair<common::math::LineSegment2d, double>>&
        obs_segments_pair,
    double* const lon_fine_tune_dis_ptr) {
  if (nullptr == lon_fine_tune_dis_ptr) {
    AERROR << "LonFineTuneTargetBasedOnObs failed, lon_fine_tune_dis_ptr is "
              "nullptr";
    return;
  }
  double max_fine_tune_lon_dis =
      config_.open_space_roi_decider_config()
          .park_in_vertical_target_lon_adjust_threshold();
  double lat_dis_to_center = 0.5 * vehicle_params_.width();
  double lon_dis_to_center = 0.5 * vehicle_params_.length();
  if (is_lat_slot) {
    // lat parking pot
    max_fine_tune_lon_dis = config_.open_space_roi_decider_config()
                                .park_in_lateral_target_lon_adjust_threshold();
    lat_dis_to_center = 0.5 * vehicle_params_.length();
    lon_dis_to_center = 0.5 * vehicle_params_.width();
  }
  double obs_min_lon_dis_to_center = -1 * lon_dis_to_center;
  // obstacle is under the adc
  for (const auto& obs_seg_pair : obs_segments_pair) {
    const auto& obs_seg = obs_seg_pair.first;
    const double obs_inflat_buffer = obs_seg_pair.second + kEpsilon;
    const double lon_distance = LonDistanceWithLatBoundary(
        obs_seg, -lat_dis_to_center - obs_inflat_buffer,
        lat_dis_to_center + obs_inflat_buffer,
        -1 * lon_dis_to_center - obs_inflat_buffer);
    obs_min_lon_dis_to_center =
        std::max(obs_min_lon_dis_to_center, lon_distance + obs_inflat_buffer);
  }

  *lon_fine_tune_dis_ptr =
      common::math::Clamp(lon_dis_to_center + obs_min_lon_dis_to_center, 0.0,
                          max_fine_tune_lon_dis);
}

void OpenSpaceFineTuning::LatFineTuneTargetBasedOnObs(
    const bool is_lat_slot,
    const std::vector<std::pair<common::math::LineSegment2d, double>>&
        obs_segments_pair,
    double* const lat_fine_tune_dis_ptr) {
  if (nullptr == lat_fine_tune_dis_ptr) {
    AERROR << "LatFineTuneTargetBasedOnObs failed, lat_fine_tune_dis_ptr is "
              "nullptr";
    return;
  }
  double max_fine_tune_lat_dis =
      config_.open_space_roi_decider_config()
          .park_in_vertical_target_lat_adjust_threshold();
  double lat_dis_to_center = 0.5 * vehicle_params_.width();
  double lon_dis_to_center = 0.5 * vehicle_params_.length();
  if (is_lat_slot) {
    // lat parking pot
    lat_dis_to_center = 0.5 * vehicle_params_.length();
    lon_dis_to_center = 0.5 * vehicle_params_.width();
    max_fine_tune_lat_dis = config_.open_space_roi_decider_config()
                                .park_in_lateral_target_lat_adjust_threshold();
  }
  const double min_width =
      is_lat_slot ? vehicle_params_.length() : vehicle_params_.width();

  double obs_min_left_lat_dis_to_center =
      -lat_dis_to_center - max_fine_tune_lat_dis;
  double obs_min_right_lat_dis_to_center =
      lat_dis_to_center + max_fine_tune_lat_dis;
  for (const auto& obs_seg_pair : obs_segments_pair) {
    const auto& obs_seg = obs_seg_pair.first;
    const double obs_inflat_buffer = obs_seg_pair.second + kEpsilon;

    const double bottom_boundary = -1 * lon_dis_to_center - obs_inflat_buffer;
    const double up_boundary = lon_dis_to_center + obs_inflat_buffer;
    const double max_y = std::max(obs_seg.start().y(), obs_seg.end().y());
    const double min_y = std::min(obs_seg.start().y(), obs_seg.end().y());
    if (max_y < bottom_boundary || min_y > up_boundary) {
      continue;
    }
    const double min_x = std::min(obs_seg.start().x(), obs_seg.end().x());
    const double max_x = std::max(obs_seg.start().x(), obs_seg.end().x());
    if (min_x < 0) {
      obs_min_left_lat_dis_to_center =
          std::max(max_x + obs_inflat_buffer, obs_min_left_lat_dis_to_center);
    }
    if (max_x > 0) {
      obs_min_right_lat_dis_to_center =
          std::min(min_x - obs_inflat_buffer, obs_min_right_lat_dis_to_center);
    }
    if (obs_min_right_lat_dis_to_center - obs_min_left_lat_dis_to_center <
        min_width) {
      *lat_fine_tune_dis_ptr = 0.0;
      ADEBUG << "freespace points do not allow valid lateral tuning with left "
                "obs dist "
             << obs_min_left_lat_dis_to_center << " and right obs dist "
             << obs_min_right_lat_dis_to_center;
      return;
    }
  }

  const double lat_left_fine_tune_dis =
      common::math::Clamp(obs_min_left_lat_dis_to_center + lat_dis_to_center,
                          0.0, max_fine_tune_lat_dis);
  const double lat_right_fine_tune_dis =
      common::math::Clamp(obs_min_right_lat_dis_to_center - lat_dis_to_center,
                          -1 * max_fine_tune_lat_dis, 0.0);
  *lat_fine_tune_dis_ptr = fabs(lat_right_fine_tune_dis) < kEpsilon
                               ? lat_left_fine_tune_dis
                               : lat_right_fine_tune_dis;
}

double OpenSpaceFineTuning::LonDistanceWithLatBoundary(
    const common::math::LineSegment2d& obs_seg, const double left_boundary,
    const double right_boundary, const double lon_dis_threshold) {
  double lon_dis = lon_dis_threshold;
  double min_x = std::min(obs_seg.start().x(), obs_seg.end().x());
  double max_x = std::max(obs_seg.start().x(), obs_seg.end().x());
  double min_y = std::min(obs_seg.start().y(), obs_seg.end().y());
  if (min_x > right_boundary || max_x < left_boundary || min_y > 0.0) {
    // out of the boundary
    return lon_dis;
  }
  lon_dis = std::max(obs_seg.start().y(), obs_seg.end().y());
  if (common::math::double_type::SeemsEqual(min_x, max_x)) {
    // is vertical
    return lon_dis;
  }
  // check larger 'y' is out of the boundary
  // if so, cut off by the boundary
  int quadrant = static_cast<int>(
      common::math::NormalizeAngle2(obs_seg.heading()) / M_PI_2);
  switch (quadrant) {
    case 0: {
      if (obs_seg.end().x() > right_boundary) {
        lon_dis = obs_seg.start().y() + (right_boundary - obs_seg.start().x()) *
                                            std::tan(obs_seg.heading());
      }
      break;
    }
    case 1: {
      if (obs_seg.end().x() < left_boundary) {
        lon_dis = obs_seg.start().y() + (obs_seg.start().x() - left_boundary) *
                                            std::tan(M_PI - obs_seg.heading());
      }
      break;
    }
    case 2: {
      if (obs_seg.start().x() > right_boundary) {
        lon_dis = obs_seg.end().y() + (right_boundary - obs_seg.end().x()) *
                                          std::tan(obs_seg.heading() - M_PI);
      }
      break;
    }
    case 3: {
      if (obs_seg.start().x() < left_boundary) {
        lon_dis =
            obs_seg.end().y() + (obs_seg.end().x() - left_boundary) *
                                    std::tan(2 * M_PI - obs_seg.heading());
      }
      break;
    }
    default:
      break;
  }
  return lon_dis;
}

void OpenSpaceFineTuning::FineTuneTargetBasedOnEndReplanPose(
    const common::PathPoint& veh_point, const bool is_right_side,
    const perception::ParkingLotOut::ParkType& park_lot_type,
    const Vec2d& origin_point, const double target_heading,
    const std::pair<double, double>& lat_space,
    const bool is_consider_wheel_mask, const bool is_parking_inwards) {
  if ((injector_->planning_context()->planning_status().open_space().replan() &
       static_cast<uint32_t>(
           TL::planning::OpenSpaceStatus::END_ANGLE_UNREACHABLE)) != 0U) {
    real_end_replan_point_ = veh_point;
    has_end_replaned_ = true;
  }
  if (!has_end_replaned_) {
    return;
  }
  const bool is_lat_slot =
      (perception::ParkingLotOut::LATERAL == park_lot_type);

  auto replan_point =
      Vec2d(real_end_replan_point_.x(), real_end_replan_point_.y()) -
      origin_point;
  replan_point.SelfRotate(-target_heading);
  const auto end_replan_adc_box =
      common::VehicleConfigHelper::GetBoundingBox(real_end_replan_point_);
  double point_dist = (is_lat_slot && !is_right_side) ? -100.0 : 100.0;
  for (const auto& point : end_replan_adc_box.GetAllCorners()) {
    auto point_origin = point - origin_point;
    point_origin.SelfRotate(-target_heading);
    if (is_lat_slot && is_right_side && point_dist > point_origin.x()) {
      point_dist = point_origin.x();
      replan_point.set_x(point_dist + vehicle_params_.back_edge_to_center());
    } else if (is_lat_slot && !is_right_side && point_dist < point_origin.x()) {
      point_dist = point_origin.x();
      replan_point.set_x(point_dist - vehicle_params_.back_edge_to_center());
    } else if (!is_lat_slot && point_dist > point_origin.y()) {
      point_dist = point_origin.y();
      replan_point.set_y(point_dist + vehicle_params_.back_edge_to_center());
    }
  }
  auto end_point =
      Vec2d(raw_end_pose_enu_.x(), raw_end_pose_enu_.y()) - origin_point;
  end_point.SelfRotate(-target_heading);

  double lat_fine_tune_dis = 0.0;
  double lon_fine_tune_dis = 0.0;
  LatFineTuneTargetBasedOnEndReplanPose(
      is_lat_slot, is_right_side, replan_point, end_point, &lat_fine_tune_dis);
  LonFineTuneTargetBasedOnEndReplanPose(
      is_lat_slot, is_right_side, is_consider_wheel_mask, is_parking_inwards,
      lat_space, replan_point, end_point, &lon_fine_tune_dis);
  if (lat_fine_tune_dis_ * lat_fine_tune_dis < 0 ||
      fabs(lat_fine_tune_dis_) < fabs(lat_fine_tune_dis)) {
    lat_fine_tune_dis_ = lat_fine_tune_dis;
    ADEBUG << "lat_fine_tune_dis due to end replan " << lat_fine_tune_dis_;
  }
  if (lon_fine_tune_dis_ * lon_fine_tune_dis < 0 ||
      fabs(lon_fine_tune_dis_) < fabs(lon_fine_tune_dis)) {
    lon_fine_tune_dis_ = lon_fine_tune_dis;
    ADEBUG << "lon_fine_tune_dis due to end replan " << lon_fine_tune_dis_;
  }
}

void OpenSpaceFineTuning::LatFineTuneTargetBasedOnEndReplanPose(
    const bool is_lat_slot, const bool is_right_side,
    const common::math::Vec2d& real_end_replan_point,
    const common::math::Vec2d& end_point, double* const lat_fine_tune_dis_ptr) {
  if (nullptr == lat_fine_tune_dis_ptr || !is_lat_slot) {
    return;
  }
  const double tune_direction = is_right_side ? 1 : -1;
  double lat_fine_tune_dis = real_end_replan_point.x() - end_point.x();
  if (tune_direction * lat_fine_tune_dis < kEpsilon) {
    ADEBUG << "target pose is not cut off, no need to lat adjust target";
    return;
  }
  *lat_fine_tune_dis_ptr = lat_fine_tune_dis;
}

void OpenSpaceFineTuning::LonFineTuneTargetBasedOnEndReplanPose(
    const bool is_lat_slot, const bool is_right_side,
    const bool is_consider_wheel_mask, const bool is_parking_inwards,
    const std::pair<double, double>& lat_space,
    const common::math::Vec2d& real_end_replan_point,
    const common::math::Vec2d& end_point, double* const lon_fine_tune_dis_ptr) {
  if (nullptr == lon_fine_tune_dis_ptr) {
    return;
  }
  constexpr double narrow_dis_thres = 1.0;
  const double extra_space_dis =
      (is_right_side ? lat_space.second : lat_space.first) -
      0.5 * vehicle_params_.length();
  if (is_lat_slot &&
      (extra_space_dis > narrow_dis_thres || !is_consider_wheel_mask)) {
    return;
  }
  double lon_fine_tune_dis = real_end_replan_point.y() - end_point.y();
  if ((lon_fine_tune_dis < kEpsilon && !is_parking_inwards) ||
      (lon_fine_tune_dis > -kEpsilon && is_parking_inwards)) {
    AINFO << "fine tune dis for end replan is invalid, target pose is not cut "
             "off";
    return;
  }
  double max_fine_tune_lon_dis =
      is_lat_slot ? config_.open_space_roi_decider_config()
                        .park_in_lateral_target_lon_adjust_threshold()
                  : config_.open_space_roi_decider_config()
                        .park_in_vertical_target_lon_adjust_threshold();
  *lon_fine_tune_dis_ptr =
      is_parking_inwards ? std::max(lon_fine_tune_dis, -max_fine_tune_lon_dis)
                         : std::min(lon_fine_tune_dis, max_fine_tune_lon_dis);
}

void OpenSpaceFineTuning::FineTuneTargetBasedOnNarrowLatSpot(
    const bool is_lat_slot,
    const std::vector<std::pair<common::math::LineSegment2d, double>>&
        obstacles_segments_vec,
    const std::pair<double, double>& lat_space) {
  const auto last_fine_tune_target = GetFineTuneTarget();
  if (common::math::CheckCollisionWithRoughVehiclePolygon2d(
          last_fine_tune_target.x(), last_fine_tune_target.y(),
          raw_end_pose_enu_.theta(), obstacles_segments_vec)) {
    ADEBUG << "target pose is now in collision , do not "
              "narrow tuning under current condition";
    return;
  }
  const bool is_enable_narrow_lon_fine_tune =
      !has_narrow_lon_tune_ && is_entered_lateral_slot_domain_ && is_lat_slot;
  if (!is_enable_narrow_lon_fine_tune) {
    lon_fine_tune_dis_ =
        std::max(pre_narrow_lon_fine_tune_dis_, lon_fine_tune_dis_);
    return;
  }
  has_narrow_lon_tune_ = true;
  static constexpr double kLatSpotLengthUpperBound = 1.5;
  static constexpr double kLatSpotLengthLowerBound = 0.8;
  const double lat_spot_length =
      std::max(lat_space.second - lat_space.first, 0.0);
  const double extra_lat_spot_length =
      lat_spot_length - vehicle_params_.length();
  const std::vector<double> input_v = {kLatSpotLengthLowerBound,
                                       kLatSpotLengthUpperBound};
  const std::vector<double> output_v = {
      config_.open_space_roi_decider_config()
          .park_in_lateral_target_lon_adjust_threshold(),
      0.0};
  const double lat_fine_tune_dis = lat_fine_tune_dis_;
  const double lon_fine_tune_dis =
      common::math::InterpolationOne(extra_lat_spot_length, input_v, output_v);
  ADEBUG << "Lon tune dis due to narrow_lat_spot : " << lon_fine_tune_dis;
  pre_narrow_lon_fine_tune_dis_ = lon_fine_tune_dis;
  if (lon_fine_tune_dis < lon_fine_tune_dis_) {
    return;
  }
  auto fine_tune_target =
      GetFineTuneTarget(lat_fine_tune_dis, lon_fine_tune_dis);
  if (!common::math::CheckCollisionWithVehiclePolygon2d(
          fine_tune_target.x(), fine_tune_target.y(), raw_end_pose_enu_.theta(),
          obstacles_segments_vec)) {
    lon_fine_tune_dis_ = lon_fine_tune_dis;
    return;
  }
  ADEBUG << "target pose is in collision after narrow lat spot fine tune";
}

void OpenSpaceFineTuning::AddCutOffSearchGrids(
    const std::shared_ptr<OpenSpaceFineTuningGrid>& grid,
    const bool is_lat_slot,
    const std::vector<std::pair<common::math::LineSegment2d, double>>&
        obstacles_segments_vec,
    std::unordered_set<std::string>* const visited_grids) {
  if (nullptr == visited_grids) {
    return;
  }
  const double lat_fine_tune_dis = grid->x();
  const double lon_fine_tune_dis = grid->y();
  if (is_lat_slot) {
    const auto lat_space =
        GetSpotLatSpace(is_lat_slot, lon_fine_tune_dis, obstacles_segments_vec);
    double lat_dis_to_center = 0.5 * vehicle_params_.length();
    const double max_lat_fine_tune_dis = lat_space.second - lat_dis_to_center;
    const double min_lat_fine_tune_dis = lat_space.first + lat_dis_to_center;
    double x = search_config_.xy_bounds[0];
    double y = lon_fine_tune_dis;
    while (x <= search_config_.xy_bounds[1]) {
      if (x < min_lat_fine_tune_dis || x > max_lat_fine_tune_dis) {
        const OpenSpaceFineTuningGrid cut_off_grid(
            x, y, search_config_.x_grid_resolution,
            search_config_.y_grid_resolution, search_config_.xy_bounds);
        visited_grids->insert(cut_off_grid.index());
      }
      x += search_config_.x_grid_resolution;
    }
    if (grid->x() < min_lat_fine_tune_dis) {
      const double next_x =
          std::ceil(min_lat_fine_tune_dis / search_config_.x_grid_resolution) *
          search_config_.x_grid_resolution;
      const auto extra_next_grid = std::make_shared<OpenSpaceFineTuningGrid>(
          next_x, lon_fine_tune_dis, search_config_.x_grid_resolution,
          search_config_.y_grid_resolution, search_config_.xy_bounds);
      grid->add_next_grid(extra_next_grid);
    }
    if (grid->x() > max_lat_fine_tune_dis) {
      const double next_x =
          std::floor(max_lat_fine_tune_dis / search_config_.x_grid_resolution) *
          search_config_.x_grid_resolution;
      const auto extra_next_grid = std::make_shared<OpenSpaceFineTuningGrid>(
          next_x, lon_fine_tune_dis, search_config_.x_grid_resolution,
          search_config_.y_grid_resolution, search_config_.xy_bounds);
      grid->add_next_grid(extra_next_grid);
    }
  } else {
    const auto lon_space =
        GetSpotLonSpace(is_lat_slot, lat_fine_tune_dis, obstacles_segments_vec);
    double lon_dis_to_center = 0.5 * vehicle_params_.length();
    const double min_lon_fine_tune_dis = lon_space + lon_dis_to_center;
    double x = lat_fine_tune_dis;
    double y = search_config_.xy_bounds[2];
    while (y <= search_config_.xy_bounds[3]) {
      if (y < min_lon_fine_tune_dis) {
        OpenSpaceFineTuningGrid cut_off_grid(
            x, y, search_config_.x_grid_resolution,
            search_config_.y_grid_resolution, search_config_.xy_bounds);
        visited_grids->insert(cut_off_grid.index());
      }
      y += search_config_.y_grid_resolution;
    }
    if (grid->y() < min_lon_fine_tune_dis) {
      const double next_y =
          std::ceil(min_lon_fine_tune_dis / search_config_.y_grid_resolution) *
          search_config_.y_grid_resolution;
      const auto extra_next_grid = std::make_shared<OpenSpaceFineTuningGrid>(
          next_y, lon_fine_tune_dis, search_config_.x_grid_resolution,
          search_config_.y_grid_resolution, search_config_.xy_bounds);
      grid->add_next_grid(extra_next_grid);
    }
  }
}

bool OpenSpaceFineTuning::ValidityCheck(
    const bool is_lat_slot, const bool is_right_side,
    const double lat_fine_tune_dis, const double lon_fine_tune_dis,
    const std::shared_ptr<OpenSpaceFineTuningObs>& fine_tune_obs) {
  for (int i = 0; i < fine_tune_obs->obs_segments_pair.size(); ++i) {
    UpdateAdjustBuffer(is_lat_slot, lat_fine_tune_dis, lon_fine_tune_dis, i,
                       fine_tune_obs);
  }
  double adc_heading = M_PI_2;
  if (is_lat_slot) {
    adc_heading = is_right_side ? 0 : M_PI;
  }
  const double shift_distance =
      0.5 * vehicle_params_.length() - vehicle_params_.back_edge_to_center();
  const auto fine_tune_target =
      Vec2d(lat_fine_tune_dis, lon_fine_tune_dis) +
      shift_distance * Vec2d::CreateUnitVec2d(adc_heading + M_PI);
  return !common::math::CheckCollisionWithRoughVehiclePolygon2d(
      fine_tune_target.x(), fine_tune_target.y(), adc_heading,
      fine_tune_obs->obs_segments_pair);
}

std::pair<double, double> OpenSpaceFineTuning::LatDistanceWithLonBoundary(
    const common::math::LineSegment2d& obs_seg, const double bottom_boundary,
    const double up_boundary, const double left_lat_dis_threshold,
    const double right_lat_dis_threshold) {
  std::pair<double, double> nearest_obs_to_center = {left_lat_dis_threshold,
                                                     right_lat_dis_threshold};
  const double max_y = std::max(obs_seg.start().y(), obs_seg.end().y());
  const double min_y = std::min(obs_seg.start().y(), obs_seg.end().y());
  if (max_y < bottom_boundary || min_y > up_boundary) {
    return nearest_obs_to_center;
  }
  const double min_x = std::min(obs_seg.start().x(), obs_seg.end().x());
  const double max_x = std::max(obs_seg.start().x(), obs_seg.end().x());
  if (min_x < 0) {
    nearest_obs_to_center.first = max_x;
  }
  if (max_x > 0) {
    nearest_obs_to_center.second = min_x;
  }
  return nearest_obs_to_center;
}

std::pair<double, double> OpenSpaceFineTuning::GetSpotLatSpace(
    const bool is_lat_slot, const double lon_fine_tune_dis,
    const std::vector<std::pair<common::math::LineSegment2d, double>>&
        transformed_obs_seg,
    const bool is_consider_buffer, const bool is_consider_whole_spot) {
  double lon_dis_to_center = is_lat_slot ? 0.5 * vehicle_params_.width()
                                         : 0.5 * vehicle_params_.length();
  double obs_min_left_lat_dis_to_center = -10.0;
  double obs_min_right_lat_dis_to_center = 10.0;
  for (const auto& obs_seg_pair : transformed_obs_seg) {
    const double obs_inflat_buffer =
        is_consider_buffer ? obs_seg_pair.second + kEpsilon : 0.0;
    const double bottom_boundary =
        is_consider_whole_spot
            ? -1 * lon_dis_to_center - obs_inflat_buffer + lon_fine_tune_dis
            : 0.0;
    const double up_boundary =
        lon_dis_to_center + obs_inflat_buffer + lon_fine_tune_dis;
    const auto nearest_obs_to_center = LatDistanceWithLonBoundary(
        obs_seg_pair.first, bottom_boundary, up_boundary, -10.0, 10.0);
    obs_min_left_lat_dis_to_center =
        std::max(nearest_obs_to_center.first + obs_inflat_buffer,
                 obs_min_left_lat_dis_to_center);
    obs_min_right_lat_dis_to_center =
        std::min(nearest_obs_to_center.second - obs_inflat_buffer,
                 obs_min_right_lat_dis_to_center);
  }
  return {obs_min_left_lat_dis_to_center, obs_min_right_lat_dis_to_center};
}

double OpenSpaceFineTuning::GetSpotLonSpace(
    const bool is_lat_slot, const double lat_fine_tune_dis,
    const std::vector<std::pair<common::math::LineSegment2d, double>>&
        obstacles_segments_vec) {
  double lat_dis_to_center = is_lat_slot ? 0.5 * vehicle_params_.length()
                                         : 0.5 * vehicle_params_.width();
  double obs_min_lon_dis_to_center = -10.0;
  for (const auto& obs_seg_pair : obstacles_segments_vec) {
    const auto& obs_seg = obs_seg_pair.first;
    const double obs_inflat_buffer = obs_seg_pair.second + kEpsilon;
    const double lon_distance = LonDistanceWithLatBoundary(
        obs_seg, -lat_dis_to_center - obs_inflat_buffer + lat_fine_tune_dis,
        lat_dis_to_center + obs_inflat_buffer + lat_fine_tune_dis, -10.0);
    obs_min_lon_dis_to_center =
        std::max(obs_min_lon_dis_to_center, lon_distance + obs_inflat_buffer);
  }
  return obs_min_lon_dis_to_center;
}

std::pair<double, double> OpenSpaceFineTuning::GetTopHalfLatSpace(
    const bool is_lat_slot,
    const std::vector<std::pair<common::math::LineSegment2d, double>>&
        obstacles_segments_vec) {
  std::vector<std::pair<common::math::LineSegment2d, double>>
      transformed_obs_seg;
  const double shift_distance =
      0.5 * vehicle_params_.length() - vehicle_params_.back_edge_to_center();
  Vec2d target_adc_center =
      Vec2d(raw_end_pose_enu_.x(), raw_end_pose_enu_.y()) +
      shift_distance * Vec2d::CreateUnitVec2d(raw_end_pose_enu_.theta());
  for (const auto& obs_seg_pair : obstacles_segments_vec) {
    auto obs = obs_seg_pair;
    obs.first.Transform(target_adc_center, target_adc_coord_heading_);
    transformed_obs_seg.emplace_back(obs);
  }
  const auto lat_space = GetSpotLatSpace(is_lat_slot, lon_fine_tune_dis_,
                                         transformed_obs_seg, false, false);
  ADEBUG << "space " << lat_space.first << "," << lat_space.second;
  return lat_space;
}

bool OpenSpaceFineTuning::FineTuneParkOutTargetBasedOnObs(
    const AVPStatus::ParkingType& parking_type,
    const perception::ParkingLotOut::SenType& sensor_type,
    const perception::ParkingLotOut::ParkType& park_lot_type,
    const ParkingLotVertexType& parking_spot_enu,
    const common::PathPoint& veh_point,
    const common::math::LineSegment2d& reference_curb) {
  if (sensor_type == perception::ParkingLotOut::USS ||
      (parking_type != planning::AVPStatus::PARKING_OUT_LEFT &&
       parking_type != planning::AVPStatus::PARKING_OUT_RIGHT) ||
      reference_curb.length() < kEpsilon ||
      std::fabs(yaw_fine_tune_rad_) > kEpsilon) {
    return false;
  }
  const auto lt2rt = common::math::LineSegment2d(parking_spot_enu.at(0),
                                                 parking_spot_enu.at(3));
  constexpr double kCurbHeightThreshold = 1.5;
  if (lt2rt.ProductOntoUnit(reference_curb.start()) < kCurbHeightThreshold &&
      lt2rt.ProductOntoUnit(reference_curb.end()) < kCurbHeightThreshold) {
    return false;
  }
  if (lt2rt.ProductOntoUnit(Vec2d(veh_point.x(), veh_point.y())) < kEpsilon) {
    return false;
  }
  const bool is_park_out_right_side =
      (parking_type == planning::AVPStatus::PARKING_OUT_RIGHT &&
       park_lot_type != perception::ParkingLotOut::LATERAL) ||
      (parking_type == planning::AVPStatus::PARKING_OUT_LEFT &&
       park_lot_type == perception::ParkingLotOut::LATERAL);
  const double target_heading = is_park_out_right_side
                                    ? reference_curb.heading()
                                    : reference_curb.heading() + M_PI;
  constexpr double kAngleDiffUpperThreshold = M_PI_2;
  constexpr double kAngleDiffLowerThreshold = 0.35;
  const double angle_diff =
      common::math::AngleDiff(raw_end_pose_enu_.theta(), target_heading);
  if (std::fabs(angle_diff) > kAngleDiffUpperThreshold ||
      std::fabs(angle_diff) < kAngleDiffLowerThreshold) {
    return false;
  }
  Vec2d raw_target_point = Vec2d(raw_end_pose_enu_.x(), raw_end_pose_enu_.y());
  const double park_out_height =
      park_lot_type == TL::perception::ParkingLotOut::LATERAL
          ? config_.open_space_roi_decider_config().lateral_park_out_height()
          : config_.open_space_roi_decider_config().vertical_park_out_height();
  const auto fine_tune_target =
      raw_target_point +
      (park_out_height - reference_curb.ProductOntoUnit(raw_target_point)) *
          Vec2d::CreateUnitVec2d(reference_curb.heading() + M_PI_2);
  const auto raw_to_target = fine_tune_target - raw_target_point;
  const auto unit_vec = lt2rt.unit_direction();
  lon_fine_tune_dis_ = unit_vec.CrossProd(raw_to_target);
  lat_fine_tune_dis_ = unit_vec.InnerProd(raw_to_target);
  yaw_fine_tune_rad_ = angle_diff;
  return true;
}

}  // namespace planning
}  // namespace TL
