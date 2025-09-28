/*
 * Copyright (c) TL Technologies Co., Ltd. 2022. All rights reserved.
 * Description:  open_space_obstacle.cc
 */

#include "planning/tasks/deciders/open_space_decider/open_space_obstacle.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "common/math/box2d.h"
#include "common/math/line_segment2d.h"
#include "common/math/linear_interpolation.h"
#include "common/math/polygon2d.h"
#include "common/math/vec2d.h"
#include "common/status/status.h"
#include "planning/common/obstacle.h"
#include "planning/common/open_space_info.h"
#include "planning/common/planning_gflags.h"
#include "planning/common/trajectory1d/constant_jerk_trajectory1d.h"

#include "proto/common/pnc_point.pb.h"
#include "proto/common/types.pb.h"
#include "proto/perception/perception_freespace.pb.h"
#include "proto/perception/perception_obstacle.pb.h"
#include "proto/perception/perception_parking_lot.pb.h"
#include "proto/planning/decision.pb.h"
#include "proto/planning/planning_internal.pb.h"
#include "proto/planning/planning_status.pb.h"

namespace TL {
namespace planning {

using TL::common::ErrorCode;
using TL::common::Status;

namespace {
constexpr double kDefaultDistToBoundary = 100;
}  // namespace

std::string ParkLotStatusToString(const ParkLotStatus& status) {
  std::string ret = "UNDEFINED";

  // Determine the string representation based on the enum value
  switch (status) {
    case NORMAL: {
      ret = "NORMAL";
      break;
    }
    case INCOMPLETE: {
      ret = "INCOMPLETE";
      break;
    }
    case POSITION_ERROR: {
      ret = "POSITION_ERROR";
      break;
    }
    case SMALL: {
      ret = "SMALL";
      break;
    }
    case UNFREE: {
      ret = "UNFREE";
      break;
    }
    case NONCONVEX: {
      ret = "NONCONVEX";
      break;
    }
  }

  // Return the string representation
  return ret;
}

OpenSpaceObstacle::OpenSpaceObstacle(const TaskConfig& config)
    : config_(config),
      vehicle_params_(
          common::VehicleConfigHelper::GetConfig().vehicle_param()) {}

TL::common::Status OpenSpaceObstacle::Init(
    const std::shared_ptr<const perception::FreeSpaceOutArray>&
        free_space_array_ptr,
    const ThreadSafeIndexedObstacles* const obstacles,
    const common::PathPoint& cur_adc_pose) {
  uss_obs_.clear();
  fs_obs_.clear();
  low_fs_obs_.clear();
  box_obs_.clear();
  virtual_obs_.clear();
  wheel_mask_obs_.clear();
  if (nullptr != obstacles) {
    const auto adc_polygon2d =
        common::VehicleConfigHelper::GetPolygon2dWithBuffer(
            cur_adc_pose.x(), cur_adc_pose.y(), cur_adc_pose.theta());
    for (const auto& obstacle : obstacles->Items()) {
      if (nullptr == obstacle || (*obstacle)->IsVirtual() ||
          !(*obstacle)->IsStatic() || (*obstacle)->IsVulnerableAlive() ||
          (*obstacle)->IsLowHeight()) {
        continue;
      }
      if ((*obstacle)->Perception().sub_type() ==
          perception::PerceptionObstacle::ST_USS) {
        InitUssObs(adc_polygon2d, **obstacle);
      } else {
        InitBBoxObs(**obstacle);
      }
      if ((*obstacle)->Perception().sub_type() ==
          perception::PerceptionObstacle::ST_WHEELSTOP) {
        InitWheelMaskObs(**obstacle);
      }
    }
  }
  InitFsObs(free_space_array_ptr);
  return Status::OK();
}

TL::common::Status OpenSpaceObstacle::LoadObs(
    const ParkingLotVertexType& parking_spot_enu,
    const ObsFilterMap& obs_filter_map,
    const AvpSpeedPlanCollisionInfo& spd_collision_info,
    const bool is_lateral_park_out, const bool replan_triggered_by_speed_plan,
    const bool is_nns_adjust_scenario,
    const ThreadSafeIndexedObstacles* const obstacles,
    std::vector<std::pair<common::math::LineSegment2d, double>>* const obs_ptr,
    std::vector<std::pair<common::math::LineSegment2d, double>>* const
        linked_obs_ptr,
    std::vector<std::pair<common::math::LineSegment2d, double>>*
        high_curb_fs_obs_ptr,
    std::vector<std::pair<common::math::LineSegment2d, double>>* const
        low_fs_obs_ptr) {
  if (nullptr == obs_ptr || nullptr == linked_obs_ptr ||
      nullptr == high_curb_fs_obs_ptr || nullptr == low_fs_obs_ptr) {
    const std::string msg = "fail at Load Obstacle";
    AERROR << msg;
    return Status(ErrorCode::PLANNER_PARKING_ROIDECIDER_ERROR, msg);
  }
  // insert obs
  LoadUssObs(obs_filter_map, obs_ptr);
  LoadBBoxObs(obs_filter_map, obs_ptr);
  LoadFsObs(obs_filter_map, obs_ptr, linked_obs_ptr, high_curb_fs_obs_ptr);
  LoadLowFsObs(obs_filter_map, parking_spot_enu, is_lateral_park_out, obs_ptr,
               low_fs_obs_ptr);
  LoadWheelMaskObs(obs_filter_map, obs_ptr);
  // obs filter
  FilterObs(obs_filter_map, obs_ptr);
  AdjustObsBuffer(parking_spot_enu, spd_collision_info,
                  replan_triggered_by_speed_plan, obstacles, obs_ptr);
  // Adjust obs buffer if we are searching path in crusing scenario(side effects possible)
  for (auto& obs : *(obs_ptr)) {
    if (is_nns_adjust_scenario && obs.second > kEps) {
      obs.second += FLAGS_extra_collision_buffer_for_nns_adjust;
    }
  }
  return Status::OK();
}

void OpenSpaceObstacle::LoadSpdCollisionObsSegments(
    const ThreadSafeIndexedObstacles* const obstacles,
    const planning_internal::AvpSpeedPlanCollisionInfo& spd_collision_info,
    const common::PathPoint& adc_pose) {
  ADEBUG << "load speed collision obstacle segements ";
  const auto adc_box = common::VehicleConfigHelper::GetBoundingBox(
      adc_pose, kADCBoxEps, kADCBoxEps);
  // Add spd_warning obs at first
  ADEBUG << "path_collision_risk from spd_collision_info: "
         << spd_collision_info.has_risk_fs_segment_start();
  if (spd_collision_info.has_risk_fs_segment_start()) {
    ADEBUG << "add speed warning fs_segments for path replan ";
    ADEBUG << "risk start point: "
           << spd_collision_info.risk_fs_segment_start().DebugString();
    ADEBUG << "risk end point: "
           << spd_collision_info.risk_fs_segment_end().DebugString();
    const Vec2d start{spd_collision_info.risk_fs_segment_start().x(),
                      spd_collision_info.risk_fs_segment_start().y()};
    const Vec2d end(spd_collision_info.risk_fs_segment_end().x(),
                    spd_collision_info.risk_fs_segment_end().y());
    const auto fs_seg = common::math::LineSegment2d(start, end);
    if (!adc_box.HasOverlap(fs_seg)) {
      virtual_obs_.emplace_back(
          fs_seg, FLAGS_avp_ego_inflated_buffer_for_checking_collision);
    }
  }

  if (!spd_collision_info.has_collision_type()) {
    ADEBUG << "speed collision info has no other collision type";
    return;
  }
  bool collision_by_polygon = false;
  bool collision_by_fs = false;
  switch (spd_collision_info.collision_type()) {
    case AvpSpeedPlanCollisionInfo::NO_COLLISION:
    case AvpSpeedPlanCollisionInfo::MOVING_OBSTACLE_COLLISION:
      break;
    case AvpSpeedPlanCollisionInfo::STATIC_OBSTACLE_COLLISION: {
      collision_by_polygon = spd_collision_info.has_static_obstacle_id();
      break;
    }
    case AvpSpeedPlanCollisionInfo::FREESPACE_POINT_COLLISION: {
      collision_by_fs = spd_collision_info.has_collision_fs_segment_start();
      break;
    }
    case AvpSpeedPlanCollisionInfo::FUSION_COLLISION: {
      collision_by_polygon = spd_collision_info.has_static_obstacle_id();
      collision_by_fs = spd_collision_info.has_collision_fs_segment_start();
      break;
    }
    default:
      break;
  }
  ADEBUG << "collision_by_polygon " << collision_by_polygon
         << " collision_by_fs " << collision_by_fs;

  if (collision_by_polygon && nullptr != obstacles) {
    for (const auto* item : obstacles->Items()) {
      if (nullptr != item &&
          (*item)->PerceptionId() == spd_collision_info.static_obstacle_id()) {
        const auto& polygon = (*item)->PerceptionPolygon();
        if (polygon.DistanceTo(adc_box) > kUseBBoxDistThreshold) {
          continue;
        }
        for (const auto& segment : polygon.line_segments()) {
          if (!adc_box.HasOverlap(segment)) {
            virtual_obs_.emplace_back(segment,
                                      std::numeric_limits<double>::epsilon());
          }
        }
        break;
      }
    }
  }

  if (collision_by_fs) {
    const Vec2d start{spd_collision_info.collision_fs_segment_start().x(),
                      spd_collision_info.collision_fs_segment_start().y()};
    const Vec2d end(spd_collision_info.collision_fs_segment_end().x(),
                    spd_collision_info.collision_fs_segment_end().y());
    const auto fs_seg = common::math::LineSegment2d(start, end);
    if (!adc_box.HasOverlap(fs_seg)) {
      const bool is_low_fs =
          IsLowFs(spd_collision_info.freespace_type(),
                  spd_collision_info.freespace_height_type());
      const double inflat_buffer =
          is_low_fs ? kBtmLowFsBuffer : std::numeric_limits<double>::epsilon();
      virtual_obs_.emplace_back(fs_seg, inflat_buffer);
    }
  }
}

bool OpenSpaceObstacle::GetNearestAdcBoundary(
    const common::math::Polygon2d& adc_polygon, const Vec2d& point,
    common::math::LineSegment2d* const nearest_seg) {
  if (nullptr == nearest_seg) {
    return false;
  }
  double distance_square = INFINITY;
  int closeIdx = -1;
  for (size_t i = 0; i < adc_polygon.line_segments().size(); ++i) {
    double tmp = adc_polygon.line_segments()[i].DistanceSquareTo(point);
    if (tmp < distance_square) {
      distance_square = tmp;
      closeIdx = static_cast<int>(i);
    }
  }
  if (closeIdx > -1) {
    *nearest_seg = adc_polygon.line_segments()[closeIdx];
    return true;
  }
  return false;
}

common::math::LineSegment2d OpenSpaceObstacle::ConstructUssWall(
    const common::math::Polygon2d& adc_polygon, const Vec2d& uss_point_enu) {
  const double half_uss_wall_width =
      0.5 * config_.open_space_roi_decider_config().uss_wall_width();
  common::math::LineSegment2d close_segment;
  if (GetNearestAdcBoundary(adc_polygon, uss_point_enu, &close_segment)) {
    const Vec2d unit_vec = Vec2d::CreateUnitVec2d(close_segment.heading());
    return common::math::LineSegment2d(
        {uss_point_enu - half_uss_wall_width * unit_vec,
         uss_point_enu + half_uss_wall_width * unit_vec});
  }
  return common::math::LineSegment2d({uss_point_enu, uss_point_enu});
}

void OpenSpaceObstacle::GetVerticalSpotParkInVirtualObsLength(
    const bool is_right_slot, const bool is_parking_inwards,
    const bool is_high_quality_triggered,
    const ParkingLotVertexType& parking_spot_enu,
    const common::PathPoint& end_pose_enu,
    const std::shared_ptr<const perception::FreeSpaceOutArray>&
        free_space_array_ptr,
    const bool is_narrow_spot_scenario,
    double* const left_virtual_obstacle_length,
    double* const right_virtual_obstacle_length) {
  if (nullptr == left_virtual_obstacle_length ||
      nullptr == right_virtual_obstacle_length ||
      nullptr == free_space_array_ptr) {
    return;
  }
  if (is_narrow_spot_scenario) {
    *left_virtual_obstacle_length = kNarrowObsLength;
    *right_virtual_obstacle_length = kNarrowObsLength;
    return;
  }
  *left_virtual_obstacle_length =
      config_.open_space_roi_decider_config()
          .vertical_park_in_virtual_obstacle_lon_dis();
  *right_virtual_obstacle_length =
      config_.open_space_roi_decider_config()
          .vertical_park_in_virtual_obstacle_lon_dis();
  if (is_high_quality_triggered) {
    return;
  }
  const bool is_right_turn_side =
      is_parking_inwards ? !is_right_slot : is_right_slot;
  common::math::Polygon2d left_corridor_filter;
  common::math::Polygon2d right_corridor_filter;
  if (!BuildVerticalSpotParkInVirtualObsFSFilterArea(
          parking_spot_enu, &left_corridor_filter, &right_corridor_filter)) {
    ADEBUG << "Build Vertical Spot Park In Virtual Obs FS Filter Area fail";
    return;
  }
  const auto& corridor_filter =
      is_right_turn_side ? right_corridor_filter : left_corridor_filter;
  Vec2d point_vec;
  double virtual_obs_length = config_.open_space_roi_decider_config()
                                  .vertical_park_in_virtual_obstacle_lon_dis();
  for (const auto& free_space_item : free_space_array_ptr->freespace_out()) {
    if (free_space_item.has_cls() &&
        free_space_item.cls() == perception::FreeSpaceOut::PEDESTRAIN) {
      ADEBUG << "ignore pedestrians";
      continue;
    }
    for (const auto& point : free_space_item.freespace_keypoint()) {
      point_vec.set_x(point.x());
      point_vec.set_y(point.y());
      if (!corridor_filter.IsPointIn(point_vec)) {
        continue;
      }
      virtual_obs_length = std::max(
          virtual_obs_length, GetMinVirtualObsLengthBasedOnFS(
                                  is_parking_inwards, end_pose_enu, point_vec));
    }
  }
  if (is_right_turn_side) {
    *right_virtual_obstacle_length = virtual_obs_length;
  } else {
    *left_virtual_obstacle_length = virtual_obs_length;
  }
}

double OpenSpaceObstacle::GetMinVirtualObsLengthBasedOnFS(
    const bool is_parking_inwards, const common::PathPoint& end_pose_enu,
    const Vec2d& fs_point) {
  constexpr double kTurnSideFSBuffer = 0.3;
  constexpr double KRadiusMargin = 0.2;
  const double min_radius = vehicle_params_.min_turn_radius() + KRadiusMargin;
  constexpr double kDefaultMinVirtualObsLength = 3.5;
  const auto base_line =
      is_parking_inwards ? Vec2d::CreateUnitVec2d(end_pose_enu.theta() + M_PI)
                         : Vec2d::CreateUnitVec2d(end_pose_enu.theta());
  const auto end_point = is_parking_inwards
                             ? Vec2d(end_pose_enu.x(), end_pose_enu.y()) -
                                   vehicle_params_.wheel_base() * base_line
                             : Vec2d(end_pose_enu.x(), end_pose_enu.y());
  const double fs_height = (fs_point - end_point).InnerProd(base_line);
  const double fs_width =
      std::fabs((fs_point - end_point).CrossProd(base_line));
  // r is distance between turning center and fs point that keeps safe
  // buffer with arc path.
  // so turning center is on the circle with fs point as center and r as radius
  const double r =
      min_radius - 0.5 * vehicle_params_.width() - kTurnSideFSBuffer;
  // also, turning center is on the line parallel to end pose between which
  // distance is min_radius.
  const double d = min_radius - fs_width;
  double min_virtual_obs_length =
      r > d ? fs_height - std::sqrt(r * r - d * d) : fs_height;
  min_virtual_obs_length -= is_parking_inwards
                                ? (vehicle_params_.front_edge_to_center() -
                                   vehicle_params_.wheel_base())
                                : vehicle_params_.back_edge_to_center();
  min_virtual_obs_length =
      std::min(min_virtual_obs_length, kDefaultMinVirtualObsLength);
  return min_virtual_obs_length;
}

void OpenSpaceObstacle::AddVerticalSpotParkInVirtualObs(
    const bool is_right_slot, const bool is_parking_inwards,
    const double lateral_nearest_dist_to_boundary,
    const common::math::Polygon2d& adc_polygon,
    const common::PathPoint& end_pose_enu,
    const ParkingLotVertexType& parking_spot_enu,
    const std::shared_ptr<const perception::FreeSpaceOutArray>&
        free_space_array_ptr,
    const bool is_narrow_spot_scenario, const bool is_high_quality_triggered,
    std::vector<common::math::LineSegment2d>* const virtual_obstacles_ptr) {
  if (nullptr == virtual_obstacles_ptr) {
    return;
  }
  UNUSED(lateral_nearest_dist_to_boundary);
  double left_virtual_obstacle_lon_dis = 0.0;
  double right_virtual_obstacle_lon_dis = 0.0;
  GetVerticalSpotParkInVirtualObsLength(
      is_right_slot, is_parking_inwards, is_high_quality_triggered,
      parking_spot_enu, end_pose_enu, free_space_array_ptr,
      is_narrow_spot_scenario, &left_virtual_obstacle_lon_dis,
      &right_virtual_obstacle_lon_dis);
  ADEBUG << "left_virtual_obstacle_lon_dis: " << left_virtual_obstacle_lon_dis;
  ADEBUG << "right_virtual_obstacle_lon_dis: "
         << right_virtual_obstacle_lon_dis;

  const double virtual_obstacle_lat_dis =
      config_.open_space_roi_decider_config()
          .vertical_park_in_virtual_obstacle_lat_dis();
  if (common::math::double_type::ComparedToZero(virtual_obstacle_lat_dis) < 1) {
    ADEBUG << "virtual obstacle is not positive, drop it";
    return;
  }
  common::math::LineSegment2d left_virtual_obs;
  common::math::LineSegment2d right_virtual_obs;
  Vec2d end_pose = Vec2d(end_pose_enu.x(), end_pose_enu.y());
  if (is_parking_inwards) {
    Vec2d front_axis_center =
        end_pose + vehicle_params_.wheel_base() *
                       Vec2d::CreateUnitVec2d(end_pose_enu.theta());
    left_virtual_obs = common::math::LineSegment2d(
        front_axis_center,
        front_axis_center +
            left_virtual_obstacle_lon_dis *
                Vec2d::CreateUnitVec2d(end_pose_enu.theta() + M_PI));
    right_virtual_obs = common::math::LineSegment2d(
        front_axis_center,
        front_axis_center +
            right_virtual_obstacle_lon_dis *
                Vec2d::CreateUnitVec2d(end_pose_enu.theta() + M_PI));
  } else {
    left_virtual_obs = common::math::LineSegment2d(
        end_pose, end_pose + left_virtual_obstacle_lon_dis *
                                 Vec2d::CreateUnitVec2d(end_pose_enu.theta()));
    right_virtual_obs = common::math::LineSegment2d(
        end_pose, end_pose + right_virtual_obstacle_lon_dis *
                                 Vec2d::CreateUnitVec2d(end_pose_enu.theta()));
  }
  auto adjust_virtual_obs_length =
      [](const common::math::Polygon2d& adc_polygon,
         common::math::LineSegment2d* const virtual_obs_ptr) {
        double cutoff_dis = 0.0;
        for (const auto& point : adc_polygon.points()) {
          double lon_dis_to_start = virtual_obs_ptr->ProjectOntoUnit(point);
          if (lon_dis_to_start < 0) {
            cutoff_dis = virtual_obs_ptr->length();
            break;
          }
          cutoff_dis = std::max(virtual_obs_ptr->length() - lon_dis_to_start,
                                cutoff_dis);
        }
        if (cutoff_dis > kEps) {
          virtual_obs_ptr->Extend(-1 * cutoff_dis);
        }
      };
  left_virtual_obs.Translate(
      vehicle_params_.left_edge_to_center() + virtual_obstacle_lat_dis,
      left_virtual_obs.heading() + M_PI_2);
  right_virtual_obs.Translate(
      vehicle_params_.right_edge_to_center() + virtual_obstacle_lat_dis,
      left_virtual_obs.heading() - M_PI_2);
  adjust_virtual_obs_length(adc_polygon, &left_virtual_obs);
  adjust_virtual_obs_length(adc_polygon, &right_virtual_obs);
  if (left_virtual_obs.length() > kMinVirtualObsLength &&
      common::math::double_type::ComparedToZero(left_virtual_obstacle_lon_dis) >
          0) {
    virtual_obstacles_ptr->emplace_back(left_virtual_obs);
  }
  if (right_virtual_obs.length() > kMinVirtualObsLength &&
      common::math::double_type::ComparedToZero(
          right_virtual_obstacle_lon_dis) > 0) {
    virtual_obstacles_ptr->emplace_back(right_virtual_obs);
  }
}

void OpenSpaceObstacle::AddLateralSpotParkInVirtualObs(
    const common::PathPoint& cur_adc_pose,
    const common::PathPoint& end_pose_enu, const bool is_consider_wheel_mask,
    const common::math::Box2d& wheel_mask_box,
    std::vector<common::math::LineSegment2d>* const virtual_obstacles_ptr) {
  if (nullptr == virtual_obstacles_ptr || !is_consider_wheel_mask) {
    return;
  }
  const auto end_point = Vec2d(end_pose_enu.x(), end_pose_enu.y());
  const auto wheel_mask_seg = common::math::LineSegment2d(
      wheel_mask_box.center() +
          wheel_mask_box.half_length() *
              Vec2d::CreateUnitVec2d(wheel_mask_box.heading() + M_PI),
      wheel_mask_box.center() +
          wheel_mask_box.half_length() *
              Vec2d::CreateUnitVec2d(wheel_mask_box.heading()));
  if (wheel_mask_seg.DistanceTo(end_point) >
      0.5 * vehicle_params_.wheel_base()) {
    AERROR << "wheel mask is opposite to end pose";
    return;
  }
  const double dist_virtual_obs_to_end_point =
      vehicle_params_.back_edge_to_center() + kEps +
      std::max(0.0, wheel_mask_seg.ProductOntoUnit(end_point) -
                        config_.open_space_speed_optimizer_config()
                            .wheel_mask_to_wheel_base_distance());
  const auto end_point_bottom =
      end_point + dist_virtual_obs_to_end_point *
                      Vec2d::CreateUnitVec2d(end_pose_enu.theta() + M_PI);
  const auto bottom_virtual_obs = common::math::LineSegment2d(
      end_point_bottom +
          wheel_mask_box.half_length() *
              Vec2d::CreateUnitVec2d(end_pose_enu.theta() + M_PI_2),
      end_point_bottom +
          wheel_mask_box.half_length() *
              Vec2d::CreateUnitVec2d(end_pose_enu.theta() - M_PI_2));
  const auto cur_adc_polygon =
      common::math::Polygon2d(common::VehicleConfigHelper::GetBoundingBox(
          cur_adc_pose,
          2 * FLAGS_avp_ego_inflated_buffer_for_checking_collision,
          2 * FLAGS_avp_ego_inflated_buffer_for_checking_collision));
  if (!cur_adc_polygon.HasOverlap(bottom_virtual_obs)) {
    virtual_obstacles_ptr->emplace_back(bottom_virtual_obs);
  }
}

void OpenSpaceObstacle::AddVerticalSpotParkOutVirtualObs(
    const common::PathPoint& init_adc_pose, const bool is_narrow_spot_scenario,
    std::vector<common::math::LineSegment2d>* const virtual_obstacles_ptr) {
  if (nullptr == virtual_obstacles_ptr) {
    return;
  }
  // add bottom obs
  Vec2d init_adc_pose_vec = Vec2d(init_adc_pose.x(), init_adc_pose.y());
  auto adc_bottom_virtual_obs = common::math::LineSegment2d(
      init_adc_pose_vec,
      init_adc_pose_vec +
          vehicle_params_.width() *
              Vec2d::CreateUnitVec2d(init_adc_pose.theta() + M_PI_2));
  adc_bottom_virtual_obs.Translate(vehicle_params_.left_edge_to_center(),
                                   init_adc_pose.theta() - M_PI_2);
  adc_bottom_virtual_obs.Translate(
      vehicle_params_.back_edge_to_center() +
          config_.open_space_roi_decider_config()
              .vertical_park_out_virtual_obstacle_bottom_dis(),
      init_adc_pose.theta() + M_PI);
  if (adc_bottom_virtual_obs.length() > kMinVirtualObsLength) {
    virtual_obstacles_ptr->emplace_back(adc_bottom_virtual_obs);
  }
  if (!is_narrow_spot_scenario) {
    return;
  }
  auto left_virtual_obs = common::math::LineSegment2d(
      init_adc_pose_vec,
      init_adc_pose_vec +
          kNarrowObsLength * Vec2d::CreateUnitVec2d(init_adc_pose.theta()));
  auto right_virtual_obs = left_virtual_obs;
  const double virtual_obstacle_lat_dis =
      config_.open_space_roi_decider_config()
          .vertical_park_in_virtual_obstacle_lat_dis();
  left_virtual_obs.Translate(
      vehicle_params_.left_edge_to_center() + virtual_obstacle_lat_dis,
      left_virtual_obs.heading() + M_PI_2);
  right_virtual_obs.Translate(
      vehicle_params_.right_edge_to_center() + virtual_obstacle_lat_dis,
      left_virtual_obs.heading() - M_PI_2);
  if (left_virtual_obs.length() > kMinVirtualObsLength) {
    virtual_obstacles_ptr->emplace_back(left_virtual_obs);
  }
  if (right_virtual_obs.length() > kMinVirtualObsLength) {
    virtual_obstacles_ptr->emplace_back(right_virtual_obs);
  }
}

void OpenSpaceObstacle::AddVirtualObs(
    const AVPStatus::ParkingType& parking_type,
    const ParkLotInfo& park_lot_info,
    const OpenSpacePathInfo& open_space_path_info,
    const std::shared_ptr<const perception::FreeSpaceOutArray>&
        free_space_array_ptr,
    const ThreadSafeIndexedObstacles* const obstacles,
    const std::vector<common::math::LineSegment2d>& roi_boundary,
    const common::PathPoint& init_adc_pose,
    const common::PathPoint& cur_adc_pose, const bool is_consider_wheel_mask,
    const common::math::Box2d& wheel_mask_box,
    const planning_internal::AvpSpeedPlanCollisionInfo& spd_collision_info,
    std::vector<std::pair<common::math::LineSegment2d, double>>* const
        obs_ptr) {
  virtual_obs_.clear();
  const auto& park_lot_type = park_lot_info.park_type;
  const auto& parking_spot_enu = park_lot_info.vertices;
  const auto& is_right_slot = park_lot_info.is_right_side;
  const auto& is_parking_inwards =
      open_space_path_info.open_space_env_structured_info.is_parking_inwards;
  const auto& end_pose_enu = open_space_path_info.end_point;
  const auto& is_narrow_spot_scenario =
      (open_space_path_info.open_space_env_structured_info
           .parking_scenario_diffculty_type &
       NARROW_SPOT_SCENARIO) != 0;
  const auto adc_polygon =
      common::math::Polygon2d(common::VehicleConfigHelper::GetBoundingBox(
          cur_adc_pose, kADCBoxEps, kADCBoxEps));
  std::vector<common::math::LineSegment2d> virtual_obstacles(roi_boundary);
  double lateral_nearest_dist_to_boundary = kDefaultDistToBoundary;
  // add turn side virtual obs
  const bool need_add_turn_side_obs =
      (parking_type != TL::planning::AVPStatus::PARKING_OUT_FRONT &&
       parking_type != TL::planning::AVPStatus::PARKING_OUT_NNS &&
       parking_type != TL::planning::AVPStatus::NNS_ADJUST);
  if (need_add_turn_side_obs) {
    bool is_right_side = is_right_slot;
    if (parking_type == TL::planning::AVPStatus::PARKING_OUT_LEFT) {
      is_right_side = perception::ParkingLotOut::LATERAL == park_lot_type;
    } else if (parking_type == TL::planning::AVPStatus::PARKING_OUT_RIGHT) {
      is_right_side = perception::ParkingLotOut::LATERAL != park_lot_type;
    } else if (is_parking_inwards) {
      is_right_side = !is_right_slot;
    }
    // add turn side virtual obstacle, improve robustness due to
    // perception error for park in  and perception lost obstacle for park out
    AddTurnSideVirtualObs(parking_type, park_lot_type, is_right_side,
                          free_space_array_ptr, cur_adc_pose, end_pose_enu,
                          parking_spot_enu, &lateral_nearest_dist_to_boundary);
  }

  switch (parking_type) {
    case TL::planning::AVPStatus::PARKING_IN: {
      // adjust virtual obstacle to ensure path near target is straight
      if (perception::ParkingLotOut::VERTICAL == park_lot_type ||
          perception::ParkingLotOut::OBLIQUE == park_lot_type) {
        // currently including oblique parking in
        AddVerticalSpotParkInVirtualObs(
            is_right_slot, is_parking_inwards, lateral_nearest_dist_to_boundary,
            adc_polygon, end_pose_enu, parking_spot_enu, free_space_array_ptr,
            is_narrow_spot_scenario, park_lot_info.is_high_quality_triggered,
            &virtual_obstacles);
      } else {
        AddLateralSpotParkInVirtualObs(cur_adc_pose, end_pose_enu,
                                       is_consider_wheel_mask, wheel_mask_box,
                                       &virtual_obstacles);
      }
      break;
    }
    case TL::planning::AVPStatus::PARKING_OUT_LEFT:
    case TL::planning::AVPStatus::PARKING_OUT_RIGHT:
    case TL::planning::AVPStatus::PARKING_OUT_FRONT:
    case TL::planning::AVPStatus::PARKING_OUT_NNS: {
      if (perception::ParkingLotOut::VERTICAL == park_lot_type ||
          perception::ParkingLotOut::OBLIQUE == park_lot_type) {
        AddVerticalSpotParkOutVirtualObs(init_adc_pose, is_narrow_spot_scenario,
                                         &virtual_obstacles);
      }
      break;
    }
    default:
      break;
  }
  for (auto& obs : virtual_obstacles) {
    if (obs.length() > kMinVirtualObsLength && !adc_polygon.HasOverlap(obs)) {
      virtual_obs_.emplace_back(obs, std::numeric_limits<double>::epsilon());
    }
  }
  LoadSpdCollisionObsSegments(obstacles, spd_collision_info, cur_adc_pose);
  obs_ptr->insert(obs_ptr->end(), virtual_obs_.begin(), virtual_obs_.end());
}

void OpenSpaceObstacle::FilterObs(
    const ObsFilterMap& obs_filter_map,
    std::vector<std::pair<common::math::LineSegment2d, double>>* const
        obs_ptr) {
  if (nullptr == obs_ptr || obs_ptr->empty()) {
    return;
  }
  if (obs_filter_map.find(COMMON_OBS) == obs_filter_map.end()) {
    return;
  }
  const auto& common_obs_filter = obs_filter_map.at(COMMON_OBS);
  ADEBUG << common_obs_filter.ShortDebugString();
  if (!common_obs_filter.use_obstacle ||
      (common_obs_filter.filter_areas.empty() &&
       common_obs_filter.filter_planes.empty())) {
    return;
  }
  auto iter = obs_ptr->begin();
  while (iter != obs_ptr->end()) {
    if (FilterSegtBaseArea(common_obs_filter.filter_areas, iter->first) ||
        FilterSegBasePlane(common_obs_filter.filter_planes, iter->first)) {
      iter = obs_ptr->erase(iter);
    } else {
      ++iter;
    }
  }
}

void OpenSpaceObstacle::AdjustObsBuffer(
    const ParkingLotVertexType& parking_spot_enu,
    const AvpSpeedPlanCollisionInfo& spd_collision_info,
    const bool replan_triggered_by_speed_plan,
    const ThreadSafeIndexedObstacles* const obstacles,
    std::vector<std::pair<common::math::LineSegment2d, double>>* const
        obs_ptr) {
  if (nullptr == obs_ptr || obs_ptr->empty()) {
    return;
  }
  // hyper plane: obs which higher than parking spot
  // certain distance has more sensor uncertainty, should has larger buffer
  static constexpr double kHighDis = 4.0;
  static constexpr double kExtraBuffer = 0.2;
  common::math::LineSegment2d plane{parking_spot_enu[0], parking_spot_enu[3]};
  plane.Translate(kHighDis, plane.heading() + M_PI_2);
  if (IsBlockByOutsideObs(plane, spd_collision_info,
                          replan_triggered_by_speed_plan, obstacles)) {
    AINFO << "has been blocked by outside obs, remove extra buffer for search";
    return;
  }
  auto is_above_plan = [&](const common::math::LineSegment2d& obs_seg) {
    return plane.ProductOntoUnit(obs_seg.start()) > 0.0 &&
           plane.ProductOntoUnit(obs_seg.end()) > 0.0;
  };
  for (auto& obs : *obs_ptr) {
    if (obs.second < kEps) {
      continue;
    }
    if (is_above_plan(obs.first)) {
      obs.second += kExtraBuffer;
    }
  }
}

bool OpenSpaceObstacle::IsBlockByOutsideObs(
    const common::math::LineSegment2d& plane,
    const AvpSpeedPlanCollisionInfo& spd_collision_info,
    const bool replan_triggered_by_speed_plan,
    const ThreadSafeIndexedObstacles* const obstacles) {
  if (!replan_triggered_by_speed_plan) {
    return false;
  }
  if (spd_collision_info.has_collision_fs_segment_start() &&
      spd_collision_info.has_collision_fs_segment_end()) {
    const auto fs_seg_start =
        Vec2d(spd_collision_info.collision_fs_segment_start().x(),
              spd_collision_info.collision_fs_segment_start().y());
    const auto fs_seg_end =
        Vec2d(spd_collision_info.collision_fs_segment_end().x(),
              spd_collision_info.collision_fs_segment_end().y());
    if (plane.ProductOntoUnit(fs_seg_start) > 0.0 ||
        plane.ProductOntoUnit(fs_seg_end) > 0.0) {
      return true;
    }
  }
  if (!spd_collision_info.has_static_obstacle_id() || nullptr == obstacles) {
    return false;
  }
  for (const auto* item : obstacles->Items()) {
    if (nullptr != item &&
        (*item)->PerceptionId() == spd_collision_info.static_obstacle_id()) {
      const auto& obs_center = (*item)->PerceptionBoundingBox().center();
      return plane.ProductOntoUnit(obs_center) > 0.0;
    }
  }
  return false;
}

void OpenSpaceObstacle::AddTurnSideVirtualObs(
    const AVPStatus::ParkingType& parking_type,
    const perception::ParkingLotOut::ParkType& park_lot_type,
    const bool is_right_turn_side,
    const std::shared_ptr<const perception::FreeSpaceOutArray>&
        free_space_array_ptr,
    const common::PathPoint& cur_adc_pose,
    const common::PathPoint& end_pose_enu,
    const ParkingLotVertexType& parking_spot_enu,
    double* const lateral_nearest_dist_to_boundary) {
  if (nullptr == free_space_array_ptr ||
      free_space_array_ptr->freespace_out().empty() ||
      nullptr == lateral_nearest_dist_to_boundary) {
    ADEBUG << "No freespace info to use, do not add virtual obstacle";
    return;
  }
  common::math::Polygon2d longtidual_filter;
  common::math::Polygon2d lateral_filter;
  const double longitudall_filter_ength =
      config_.open_space_roi_decider_config().turn_side_filter_length();

  if (!BuildTurnSideFSFilterArea(parking_type, park_lot_type, parking_spot_enu,
                                 is_right_turn_side, longitudall_filter_ength,
                                 &lateral_filter)) {
    AERROR << "build park in fs filter fails";
    return;
  }
  // first lateral side fs
  const auto lateral_boundary = is_right_turn_side
                                    ? lateral_filter.line_segments().at(0)
                                    : lateral_filter.line_segments().at(2);
  Vec2d lateral_nearest_fs_point;
  FindNearestFSPoint(free_space_array_ptr, lateral_filter, lateral_boundary,
                     lateral_nearest_dist_to_boundary,
                     &lateral_nearest_fs_point);
  if (*lateral_nearest_dist_to_boundary >= kDefaultDistToBoundary - 1) {
    ADEBUG << "no fs is found";
    return;
  }
  // construct longitudal filter
  auto left_right = parking_spot_enu[3] - parking_spot_enu[0];
  left_right.Normalize();
  auto down_top = parking_spot_enu[0] - parking_spot_enu[1];
  down_top.Normalize();
  left_right.Normalize();
  if (is_right_turn_side) {
    const auto longtidual_filter_zero =
        lateral_nearest_fs_point +
        (lateral_filter.points().at(0) - lateral_nearest_fs_point)
                .InnerProd(down_top) *
            down_top;
    const auto longtidual_filter_one =
        longtidual_filter_zero -
        lateral_filter.line_segments().at(0).length() * down_top;
    longtidual_filter = common::math::Polygon2d(std::vector<Vec2d>{
        longtidual_filter_zero, longtidual_filter_one,
        longtidual_filter_one + left_right * longitudall_filter_ength,
        longtidual_filter_zero + left_right * longitudall_filter_ength});
  } else {
    const auto longtidual_filter_three =
        lateral_nearest_fs_point +
        (lateral_filter.points().at(3) - lateral_nearest_fs_point)
                .InnerProd(down_top) *
            down_top;
    const auto longtidual_filter_two =
        longtidual_filter_three -
        lateral_filter.line_segments().at(0).length() * down_top;
    longtidual_filter = common::math::Polygon2d(std::vector<Vec2d>{
        longtidual_filter_three - left_right * longitudall_filter_ength,
        longtidual_filter_two - left_right * longitudall_filter_ength,
        longtidual_filter_two, longtidual_filter_three});
  }
  // find hightest fs point
  double longtidual_nearest_dist_to_boundary = kDefaultDistToBoundary;
  Vec2d longitudal_nearest_fs_point;
  FindNearestFSPoint(free_space_array_ptr, longtidual_filter,
                     longtidual_filter.line_segments().at(3),
                     &longtidual_nearest_dist_to_boundary,
                     &longitudal_nearest_fs_point);

  // add virtual obs
  static constexpr double kExcludeVirtualObstcaleBuffer = 0.3;
  const auto cur_adc_polygon =
      parking_type == TL::planning::AVPStatus::PARKING_IN
          ? common::math::Polygon2d(common::VehicleConfigHelper::GetBoundingBox(
                cur_adc_pose, 2 * kExcludeVirtualObstcaleBuffer,
                2 * kExcludeVirtualObstcaleBuffer))
          : common::math::Polygon2d(common::VehicleConfigHelper::GetBoundingBox(
                cur_adc_pose, 2 * kExcludeVirtualObstcaleBuffer,
                kExcludeVirtualObstcaleBuffer));

  const auto endpose_adc_polygon =
      parking_type == TL::planning::AVPStatus::PARKING_IN
          ? common::math::Polygon2d(common::VehicleConfigHelper::GetBoundingBox(
                end_pose_enu, 2 * kExcludeVirtualObstcaleBuffer,
                kExcludeVirtualObstcaleBuffer))
          : common::math::Polygon2d(common::VehicleConfigHelper::GetBoundingBox(
                end_pose_enu, 2 * kExcludeVirtualObstcaleBuffer,
                2 * kExcludeVirtualObstcaleBuffer));
  // add lateral virtual obstacle
  InsertTurnSideVirtualObs(cur_adc_polygon, endpose_adc_polygon,
                           lateral_nearest_fs_point, parking_type,
                           park_lot_type, parking_spot_enu);
  // add longitudal virtual obstacle
  if (longtidual_nearest_dist_to_boundary < kDefaultDistToBoundary) {
    InsertTurnSideVirtualObs(cur_adc_polygon, endpose_adc_polygon,
                             longitudal_nearest_fs_point, parking_type,
                             park_lot_type, parking_spot_enu);
  }
}

void OpenSpaceObstacle::InsertTurnSideVirtualObs(
    const common::math::Polygon2d& cur_adc_polygon,
    const common::math::Polygon2d& endpose_adc_polygon,
    const common::math::Vec2d& fs_point,
    const AVPStatus::ParkingType& parking_type,
    const perception::ParkingLotOut::ParkType& park_lot_type,
    const ParkingLotVertexType& parking_spot_enu) {
  static constexpr double kMinObsLength = 1.0e-2;
  auto down_top = parking_spot_enu[0] - parking_spot_enu[1];
  down_top.Normalize();
  const double fs_above_slot_dis =
      (fs_point - parking_spot_enu.at(0)).InnerProd(down_top);
  double default_virtual_obs_length =
      (perception::ParkingLotOut::LATERAL == park_lot_type)
          ? config_.open_space_roi_decider_config()
                .lateral_park_in_turn_side_filter_length()
          : config_.open_space_roi_decider_config()
                .non_lateral_park_in_turn_side_filter_length();
  if (parking_type != TL::planning::AVPStatus::PARKING_IN) {
    default_virtual_obs_length =
        (perception::ParkingLotOut::LATERAL == park_lot_type)
            ? config_.open_space_roi_decider_config()
                  .lateral_park_out_turn_side_filter_length()
            : config_.open_space_roi_decider_config()
                  .non_lateral_park_out_turn_side_filter_length();
  }
  const double virtual_obs_length =
      fmin(default_virtual_obs_length,
           fmax(config_.open_space_roi_decider_config()
                        .turn_side_above_slot_height_limit() -
                    fs_above_slot_dis,
                0));
  if (virtual_obs_length > kMinObsLength) {
    const auto virtual_obs = common::math::LineSegment2d(
        fs_point, fs_point + down_top * virtual_obs_length);
    if (!cur_adc_polygon.HasOverlap(virtual_obs) &&
        !endpose_adc_polygon.HasOverlap(virtual_obs)) {
      ADEBUG << "add virtual obstacle, from " << virtual_obs.start().x() << ", "
             << virtual_obs.start().y() << " to " << virtual_obs.end().x()
             << ", " << virtual_obs.end().y();
      virtual_obs_.emplace_back(
          virtual_obs, FLAGS_avp_ego_inflated_buffer_for_checking_collision);
    }
  }
}

void OpenSpaceObstacle::InitUssObs(const common::math::Polygon2d& adc_polygon2d,
                                   const TL::planning::Obstacle& obstacle) {
  const Vec2d uss_point(obstacle.Perception().position().x(),
                        obstacle.Perception().position().y());
  uss_obs_.emplace_back(
      ConstructUssWall(adc_polygon2d, uss_point),
      config_.open_space_roi_decider_config().uss_inflat_buffer());
}

void OpenSpaceObstacle::InitWheelMaskObs(
    const TL::planning::Obstacle& obstacle) {
  static constexpr double kWheelMaskCircleInflatedBuffer = 0.3;
  const auto& wheel_mask_center = obstacle.PerceptionBoundingBox().center();
  const common::math::LineSegment2d wheel_mask_seg(wheel_mask_center,
                                                   wheel_mask_center);
  wheel_mask_obs_.emplace_back(wheel_mask_seg, kWheelMaskCircleInflatedBuffer);
}

void OpenSpaceObstacle::InitBBoxObs(const TL::planning::Obstacle& obstacle) {
  const auto& obstacle_box_corners =
      obstacle.PerceptionBoundingBox().GetAllCorners();
  const size_t cornerSize = obstacle_box_corners.size();
  for (size_t i = 0; i < cornerSize; ++i) {
    const common::math::LineSegment2d seg(
        obstacle_box_corners[i], obstacle_box_corners[(i + 1) % cornerSize]);
    box_obs_.emplace_back(seg,
                          FLAGS_avp_ego_inflated_buffer_for_checking_collision);
  }
}

void OpenSpaceObstacle::InitFsObs(
    const std::shared_ptr<const perception::FreeSpaceOutArray>&
        free_space_array_ptr) {
  if (nullptr == free_space_array_ptr ||
      free_space_array_ptr->freespace_out().empty()) {
    ADEBUG << "No freespace info to use";
    return;
  }
  const uint64_t ignore_points_num_threhold =
      config_.open_space_roi_decider_config().ignore_unknown_point_num();
  linked_fs_obs_idx_set_.clear();
  high_curb_fs_obs_idx_set_.clear();
  for (const auto& free_space_item : free_space_array_ptr->freespace_out()) {
    if (free_space_item.has_cls() &&
        free_space_item.cls() == perception::FreeSpaceOut::PEDESTRAIN) {
      ADEBUG << "ignore pedestrians";
      continue;
    }
    int points_num = free_space_item.freespace_keypoint_size();
    if (points_num < ignore_points_num_threhold) {
      ADEBUG << "the num of point is less than threhold, "
                " ignore it";
      continue;
    }
    const bool is_linked_fs =
        free_space_item.has_islinkobjfusion() &&
        free_space_item.islinkobjfusion() && free_space_item.has_obstacleid() &&
        (free_space_item.cls() == perception::FreeSpaceOut::CONE_POLE ||
         free_space_item.cls() == perception::FreeSpaceOut::VEHICLE);
    const bool is_high_curb_fs =
        free_space_item.cls() == perception::FreeSpaceOut::CURBSTONE &&
        free_space_item.height_type() == perception::FreeSpaceOut::OVERDRIVABLE;
    const bool is_low_fs =
        IsLowFs(free_space_item.cls(), free_space_item.height_type());
    for (int i = 0; i + 1 < points_num; i++) {
      Vec2d start_point{free_space_item.freespace_keypoint().at(i).x(),
                        free_space_item.freespace_keypoint().at(i).y()};
      Vec2d end_point{free_space_item.freespace_keypoint().at(i + 1).x(),
                      free_space_item.freespace_keypoint().at(i + 1).y()};
      // To avoid unusual link ignore too long segment
      if (start_point.DistanceTo(end_point) >
          config_.open_space_roi_decider_config().ignore_length_threhold()) {
        continue;
      }
      if (is_low_fs) {
        // Update smaller buffer in LoadObs
        low_fs_obs_.emplace_back(
            common::math::LineSegment2d(start_point, end_point),
            FLAGS_avp_ego_inflated_buffer_for_checking_collision);
      } else {
        if (is_linked_fs) {
          linked_fs_obs_idx_set_.emplace(fs_obs_.size());
        }
        if (is_high_curb_fs) {
          high_curb_fs_obs_idx_set_.emplace(fs_obs_.size());
        }
        fs_obs_.emplace_back(
            common::math::LineSegment2d(start_point, end_point),
            FLAGS_avp_ego_inflated_buffer_for_checking_collision);
      }
    }
  }
}

void OpenSpaceObstacle::LoadUssObs(
    const ObsFilterMap& obs_filter_map,
    std::vector<std::pair<common::math::LineSegment2d, double>>* const
        obs_ptr) {
  if (nullptr == obs_ptr) {
    return;
  }
  const bool has_uss_filter =
      obs_filter_map.find(USS_OBS) != obs_filter_map.end();
  if (has_uss_filter && !obs_filter_map.at(USS_OBS).use_obstacle) {
    ADEBUG << "ignore uss type obs";
    return;
  }
  std::vector<std::pair<common::math::Polygon2d, bool>> filter_areas;
  std::vector<std::pair<common::math::LineSegment2d, bool>> filter_planes;
  if (has_uss_filter) {
    filter_areas = obs_filter_map.at(USS_OBS).filter_areas;
    filter_planes = obs_filter_map.at(USS_OBS).filter_planes;
  }
  for (const auto& obs_seg : uss_obs_) {
    if (FilterPointBaseArea(filter_areas, obs_seg.first.center()) ||
        FilterPointBasePlane(filter_planes, obs_seg.first.center())) {
      continue;
    }
    obs_ptr->emplace_back(obs_seg);
  }
}

void OpenSpaceObstacle::LoadBBoxObs(
    const ObsFilterMap& obs_filter_map,
    std::vector<std::pair<common::math::LineSegment2d, double>>* const
        obs_ptr) {
  if (nullptr == obs_ptr) {
    return;
  }
  const bool has_box_filter =
      obs_filter_map.find(BOX_OBS) != obs_filter_map.end();
  if (has_box_filter && !obs_filter_map.at(BOX_OBS).use_obstacle) {
    ADEBUG << "ignore bounding box";
    return;
  }
  std::vector<std::pair<common::math::Polygon2d, bool>> filter_areas;
  std::vector<std::pair<common::math::LineSegment2d, bool>> filter_planes;
  if (has_box_filter) {
    filter_areas = obs_filter_map.at(BOX_OBS).filter_areas;
    filter_planes = obs_filter_map.at(BOX_OBS).filter_planes;
  }
  for (const auto& obs_seg : box_obs_) {
    if (FilterSegtBaseArea(filter_areas, obs_seg.first) ||
        FilterSegBasePlane(filter_planes, obs_seg.first)) {
      continue;
    }
    obs_ptr->emplace_back(obs_seg);
  }
}

void OpenSpaceObstacle::LoadFsObs(
    const ObsFilterMap& obs_filter_map,
    std::vector<std::pair<common::math::LineSegment2d, double>>* const obs_ptr,
    std::vector<std::pair<common::math::LineSegment2d, double>>* const
        linked_obs_ptr,
    std::vector<std::pair<common::math::LineSegment2d, double>>*
        high_curb_fs_obs_ptr) {
  // filter fs obs
  // modify buffer
  const bool has_free_space_filter =
      obs_filter_map.find(FREE_SPACE_OBS) != obs_filter_map.end();
  const bool has_high_curb_fs_filter =
      obs_filter_map.find(HIGH_CURB_FREE_SPACE_OBS) != obs_filter_map.end();
  if (has_free_space_filter &&
      !obs_filter_map.at(FREE_SPACE_OBS).use_obstacle) {
    // ignore free space
    return;
  }
  std::vector<std::pair<common::math::Polygon2d, bool>> filter_areas;
  std::vector<std::pair<common::math::LineSegment2d, bool>> filter_planes;
  std ::vector<std::pair<common::math::LineSegment2d, bool>>
      high_curb_fs_filter_planes;
  if (has_free_space_filter) {
    filter_areas = obs_filter_map.at(FREE_SPACE_OBS).filter_areas;
    filter_planes = obs_filter_map.at(FREE_SPACE_OBS).filter_planes;
    ADEBUG << "obs_filter_map.at(FREE_SPACE_OBS) "
           << obs_filter_map.at(FREE_SPACE_OBS).ShortDebugString();
  }
  if (has_high_curb_fs_filter) {
    high_curb_fs_filter_planes =
        obs_filter_map.at(HIGH_CURB_FREE_SPACE_OBS).filter_planes;
  }
  for (int i = 0; i < fs_obs_.size(); ++i) {
    const auto& obs_seg = fs_obs_.at(i);
    if (FilterSegtBaseArea(filter_areas, obs_seg.first)) {
      continue;
    }
    if (FilterPointBasePlane(filter_planes, obs_seg.first.start()) ||
        FilterPointBasePlane(filter_planes, obs_seg.first.end())) {
      continue;
    }
    obs_ptr->emplace_back(obs_seg);
    if (linked_fs_obs_idx_set_.find(i) != linked_fs_obs_idx_set_.end()) {
      linked_obs_ptr->emplace_back(obs_seg);
    }
    if (high_curb_fs_obs_idx_set_.find(i) != high_curb_fs_obs_idx_set_.end()) {
      if (!high_curb_fs_filter_planes.empty() &&
          (FilterPointBasePlane(high_curb_fs_filter_planes,
                                obs_seg.first.start()) ||
           FilterPointBasePlane(high_curb_fs_filter_planes,
                                obs_seg.first.end()))) {
        ADEBUG << "ignore bottom high curb fs for fine tune";
        continue;
      }
      high_curb_fs_obs_ptr->emplace_back(obs_seg.first, kHighCurbBuffer);
    }
  }
}

void OpenSpaceObstacle::LoadLowFsObs(
    const ObsFilterMap& obs_filter_map,
    const ParkingLotVertexType& parking_spot_enu,
    const bool is_lateral_park_out,
    std::vector<std::pair<common::math::LineSegment2d, double>>* const obs_ptr,
    std::vector<std::pair<common::math::LineSegment2d, double>>* const
        low_fs_obs_ptr) {
  // filter fs obs
  // modify buffer
  const bool has_free_space_filter =
      obs_filter_map.find(LOW_FREE_SPACE_OBS) != obs_filter_map.end();
  if (has_free_space_filter &&
      !obs_filter_map.at(LOW_FREE_SPACE_OBS).use_obstacle) {
    // ignore free space
    return;
  }
  common::math::LineSegment2d lmid_to_rmid(
      (parking_spot_enu[0] + parking_spot_enu[1]) / 2.0,
      (parking_spot_enu[2] + parking_spot_enu[3]) / 2.0);
  std::vector<std::pair<common::math::Polygon2d, bool>> filter_areas;
  std::vector<std::pair<common::math::LineSegment2d, bool>> filter_planes;
  double inflate_buff = FLAGS_avp_ego_inflated_buffer_for_checking_collision;
  if (has_free_space_filter) {
    filter_areas = obs_filter_map.at(LOW_FREE_SPACE_OBS).filter_areas;
    filter_planes = obs_filter_map.at(LOW_FREE_SPACE_OBS).filter_planes;
    ADEBUG << "obs_filter_map.at(LOW_FREE_SPACE_OBS) "
           << obs_filter_map.at(LOW_FREE_SPACE_OBS).ShortDebugString();
  }
  for (const auto& obs_seg : low_fs_obs_) {
    const auto& start = obs_seg.first.start();
    const auto& end = obs_seg.first.end();

    // Independently storing low_fs_obs_segement with small buffer
    low_fs_obs_ptr->emplace_back(obs_seg.first, kBtmLowFsBuffer);

    if (FilterSegtBaseArea(filter_areas, obs_seg.first)) {
      continue;
    }
    if (FilterPointBasePlane(filter_planes, start) ||
        FilterPointBasePlane(filter_planes, end)) {
      continue;
    }
    // Load low fs for all ParkType
    if (lmid_to_rmid.ProductOntoUnit(start) < 0.0 &&
        lmid_to_rmid.ProductOntoUnit(end) < 0.0) {
      inflate_buff = (is_lateral_park_out) ? kBTMLowFsBufferForLatParkOut
                                           : kBtmLowFsBuffer;
    }
    obs_ptr->emplace_back(obs_seg.first, inflate_buff);
  }
}

void OpenSpaceObstacle::LoadWheelMaskObs(
    const ObsFilterMap& obs_filter_map,
    std::vector<std::pair<common::math::LineSegment2d, double>>* const
        obs_ptr) {
  if (nullptr == obs_ptr) {
    return;
  }
  const bool has_wheel_mask_filter =
      obs_filter_map.find(WHEEL_MASK_OBS) != obs_filter_map.end();
  if (!has_wheel_mask_filter ||
      !obs_filter_map.at(WHEEL_MASK_OBS).use_obstacle) {
    ADEBUG << "there is no wheel_mask_obs under considering";
    return;
  }
  for (const auto& obs_seg : wheel_mask_obs_) {
    if (FilterSegtBaseArea(obs_filter_map.at(WHEEL_MASK_OBS).filter_areas,
                           obs_seg.first) ||
        FilterSegBasePlane(obs_filter_map.at(WHEEL_MASK_OBS).filter_planes,
                           obs_seg.first)) {
      continue;
    }
    obs_ptr->emplace_back(obs_seg);
  }
}

void OpenSpaceObstacle::UpdateOpenSpaceInfoForSpd(
    const AVPStatus::ParkingType& parking_type,
    const perception::ParkingLotOut::ParkType& park_lot_type,
    const ParkingLotVertexType& parking_spot_enu,
    const std::shared_ptr<const perception::FreeSpaceOutArray>&
        free_space_array_ptr,
    const common::PathPoint& init_adc_pose,
    OpenSpaceInfo* const open_space_info_ptr) {
  if (nullptr == open_space_info_ptr) {
    AERROR << "open_space_info_ptr is nullptr";
    return;
  }
  const bool is_lat_spot =
      (park_lot_type == perception::ParkingLotOut::LATERAL);
  common::math::LineSegment2d lmid_to_rmid(
      (parking_spot_enu[0] + parking_spot_enu[1]) / 2.0,
      (parking_spot_enu[2] + parking_spot_enu[3]) / 2.0);
  common::math::LineSegment2d rear_center_to_right_wheel(
      {init_adc_pose.x(), init_adc_pose.y()},
      {init_adc_pose.x() + sin(init_adc_pose.theta()),
       init_adc_pose.y() - cos(init_adc_pose.theta())});
  for (int fs_idx = 0; fs_idx < free_space_array_ptr->freespace_out().size();
       ++fs_idx) {
    const auto& free_space_item =
        free_space_array_ptr->freespace_out().at(fs_idx);
    const bool is_low_fs =
        IsLowFs(free_space_item.cls(), free_space_item.height_type());
    const bool is_high_curb_fs =
        IsHighCurbFs(free_space_item.cls(), free_space_item.height_type());
    // When fs low curb & fs low others are under the parking slot
    // decrease inflate buffer for approaching
    int points_num = free_space_item.freespace_keypoint_size();
    if (is_low_fs && points_num > 0 &&
        IsFsLowerThanSpot(free_space_item.freespace_keypoint(), lmid_to_rmid)) {
      // filter low fs in Lateral Parking_in
      open_space_info_ptr->mutable_under_spot_low_fs_idxs()->emplace_back(
          fs_idx);
    }
    if (is_high_curb_fs && points_num > 0) {
      open_space_info_ptr->mutable_high_curb_fs_idxs()->emplace_back(fs_idx);
    }
    Vec2d start_point;
    Vec2d end_point;
    for (int i = 0; i + 1 < points_num; i++) {
      start_point = {free_space_item.freespace_keypoint().at(i).x(),
                     free_space_item.freespace_keypoint().at(i).y()};
      end_point = {free_space_item.freespace_keypoint().at(i + 1).x(),
                   free_space_item.freespace_keypoint().at(i + 1).y()};
      if (is_low_fs && !is_lat_spot && parking_type != AVPStatus::PARKING_IN) {
        // When is not lateral parking_in, ignore some of the low obs
        if (rear_center_to_right_wheel.ProductOntoUnit(start_point) < 0.0 &&
            rear_center_to_right_wheel.ProductOntoUnit(end_point) < 0.0) {
          ADEBUG << "ignore low fs under init adc in vertical park out scene";
          auto& ignore_fs_pair =
              (open_space_info_ptr->ignore_fs_idxs().empty() ||
               open_space_info_ptr->ignore_fs_idxs().back().first != fs_idx)
                  ? open_space_info_ptr->mutable_ignore_fs_idxs()
                        ->emplace_back()
                  : open_space_info_ptr->mutable_ignore_fs_idxs()->back();
          ignore_fs_pair.first = fs_idx;
          ignore_fs_pair.second.emplace_back(i);
          if (i + 2 == points_num) {
            ignore_fs_pair.second.emplace_back(i + 1);
          }
          continue;
        }
      }
    }
  }
}

// longtidual--y direction, lateral--x direction
// filter points ordered by
/* ***********zero********three******
   **********************************
   ***********one**********two*******
*/
bool OpenSpaceObstacle::BuildTurnSideFSFilterArea(
    const AVPStatus::ParkingType& parking_type,
    const perception::ParkingLotOut::ParkType& park_lot_type,
    const ParkingLotVertexType& parking_spot_enu, const bool is_right_turn_side,
    const double longitudal_filter_length,
    common::math::Polygon2d* lateral_filter) {
  if (lateral_filter == nullptr) {
    AERROR << " BuildTurnSideFSFilterArea input check fails";
    return false;
  }
  static constexpr double kFilterAboveSlotHeight = 1.0;
  auto down_top = parking_spot_enu[0] - parking_spot_enu[1];
  auto left_right = parking_spot_enu[3] - parking_spot_enu[0];
  // default vertical/oblique park in
  double lateral_filter_length =
      config_.open_space_roi_decider_config().vertical_expand_buffer();
  if (park_lot_type == perception::ParkingLotOut::LATERAL) {
    lateral_filter_length =
        config_.open_space_roi_decider_config().lateral_expand_buffer();
  } else if (parking_type != TL::planning::AVPStatus::PARKING_IN) {
    lateral_filter_length = config_.open_space_roi_decider_config()
                                .park_out_vertical_expand_buffer();
  }
  lateral_filter_length = lateral_filter_length / 2 + longitudal_filter_length;
  down_top.Normalize();
  left_right.Normalize();
  // construct lateral_filter
  Vec2d lateral_filter_zero;
  Vec2d lateral_filter_one;
  Vec2d lateral_filter_two;
  Vec2d lateral_filter_three;
  if (is_right_turn_side) {
    lateral_filter_zero =
        parking_spot_enu.at(3) + down_top * kFilterAboveSlotHeight;
    lateral_filter_one =
        parking_spot_enu.at(3) - down_top * kFilterAboveSlotHeight;
    lateral_filter_two =
        lateral_filter_one + left_right * lateral_filter_length;
    lateral_filter_three =
        lateral_filter_zero + left_right * lateral_filter_length;
  } else {
    lateral_filter_three =
        parking_spot_enu.at(0) + down_top * kFilterAboveSlotHeight;
    lateral_filter_two =
        parking_spot_enu.at(0) - down_top * kFilterAboveSlotHeight;
    lateral_filter_one =
        lateral_filter_two - left_right * lateral_filter_length;
    lateral_filter_zero =
        lateral_filter_three - left_right * lateral_filter_length;
  }
  *lateral_filter = common::math::Polygon2d(
      std::vector<Vec2d>{lateral_filter_zero, lateral_filter_one,
                         lateral_filter_two, lateral_filter_three});
  return lateral_filter->GetValid();
}

void OpenSpaceObstacle::FindNearestFSPoint(
    const std::shared_ptr<const perception::FreeSpaceOutArray>&
        free_space_array_ptr,
    const common::math::Polygon2d& filter_area,
    const common::math::LineSegment2d& boundary,
    double* const nearest_dist_to_boundary, Vec2d* const nearest_fs_point) {
  if (nullptr == free_space_array_ptr || nearest_dist_to_boundary == nullptr ||
      nearest_fs_point == nullptr) {
    AERROR << "FindNearestFSPoint input check fails";
    return;
  }
  Vec2d point_vec;
  double dist_to_bounadry = 0.0;
  for (const auto& free_space_item : free_space_array_ptr->freespace_out()) {
    if (free_space_item.has_cls() &&
        free_space_item.cls() == perception::FreeSpaceOut::PEDESTRAIN) {
      ADEBUG << "ignore pedestrians";
      continue;
    }
    for (const auto& point : free_space_item.freespace_keypoint()) {
      point_vec.set_x(point.x());
      point_vec.set_y(point.y());
      if (filter_area.IsPointIn(point_vec)) {
        dist_to_bounadry = boundary.DistanceTo(point_vec);
        if (dist_to_bounadry < *nearest_dist_to_boundary) {
          *nearest_fs_point = point_vec;
          *nearest_dist_to_boundary = dist_to_bounadry;
        }
      }
    }
  }
}

void OpenSpaceObstacle::FindNearestFSPointInItem(
    const perception::FreeSpaceOut& free_space_item,
    const common::math::Polygon2d& filter_area,
    const common::math::LineSegment2d& boundary,
    double* const nearest_dist_to_boundary, Vec2d* const nearest_fs_point) {
  if (nullptr == nearest_dist_to_boundary || nearest_fs_point == nullptr) {
    AERROR << "FindNearestFSPointInItem input check fails";
    return;
  }
  Vec2d point_vec;
  double dist_to_bounadry = 0.0;
  for (const auto& point : free_space_item.freespace_keypoint()) {
    point_vec.set_x(point.x());
    point_vec.set_y(point.y());
    if (filter_area.IsPointIn(point_vec)) {
      dist_to_bounadry = boundary.DistanceTo(point_vec);
      if (dist_to_bounadry < *nearest_dist_to_boundary) {
        *nearest_fs_point = point_vec;
        *nearest_dist_to_boundary = dist_to_bounadry;
      }
    }
  }
}

bool OpenSpaceObstacle::FilterPointBaseArea(
    const std::vector<std::pair<common::math::Polygon2d, bool>>& filter_areas,
    const common::math::Vec2d& obs) {
  return std::any_of(
      filter_areas.begin(), filter_areas.end(),
      [&](const std::pair<common::math::Polygon2d, bool>& filter) {
        const bool is_obs_in_area = filter.first.IsPointIn(obs);
        return (filter.second && is_obs_in_area) ||
               (!filter.second && !is_obs_in_area);
      });
}

bool OpenSpaceObstacle::FilterPointBasePlane(
    const std::vector<std::pair<common::math::LineSegment2d, bool>>&
        filter_planes,
    const common::math::Vec2d& obs) {
  return std::any_of(
      filter_planes.begin(), filter_planes.end(),
      [&](const std::pair<common::math::LineSegment2d, bool>& filter) {
        const bool is_obs_above_plane = filter.first.ProductOntoUnit(obs) > 0.0;
        return (filter.second && is_obs_above_plane) ||
               (!filter.second && !is_obs_above_plane);
      });
}

bool OpenSpaceObstacle::FilterSegtBaseArea(
    const std::vector<std::pair<common::math::Polygon2d, bool>>& filter_areas,
    const common::math::LineSegment2d& obs_seg) {
  return std::any_of(
      filter_areas.begin(), filter_areas.end(),
      [&](const std::pair<common::math::Polygon2d, bool>& filter) {
        return (filter.second && filter.first.HasOverlap(obs_seg)) ||
               (!filter.second && !filter.first.HasOverlap(obs_seg));
      });
}

bool OpenSpaceObstacle::FilterSegBasePlane(
    const std::vector<std::pair<common::math::LineSegment2d, bool>>&
        filter_planes,
    const common::math::LineSegment2d& obs_seg) {
  return std::any_of(
      filter_planes.begin(), filter_planes.end(),
      [&](const std::pair<common::math::LineSegment2d, bool>& filter) {
        double cross_prod_1 = filter.first.ProductOntoUnit(obs_seg.start());
        double cross_prod_2 = filter.first.ProductOntoUnit(obs_seg.end());
        return (filter.second && (cross_prod_1 > 0.0 || cross_prod_2 > 0.0)) ||
               (!filter.second && (cross_prod_1 < 0.0 || cross_prod_2 < 0.0));
      });
}

double OpenSpaceObstacle::ParkInObsLengthLateralConstraint(
    const bool is_narrow_spot_scenario,
    const double lateral_nearest_dist_to_boundary,
    const ParkingLotVertexType& parking_spot_enu) {
  constexpr double kNarrowObsLength = 3.5;
  if (is_narrow_spot_scenario) {
    return kNarrowObsLength;
  }
  // calculate acceptable obs length due to lateral fs
  const double lateral_freespace_width =
      lateral_nearest_dist_to_boundary +
      (parking_spot_enu.at(3) - parking_spot_enu.at(0)).Length() / 2 -
      vehicle_params_.width() / 2;
  return common::math::InterpolationOne(
      lateral_freespace_width,
      config_.open_space_roi_decider_config()
          .vertical_park_in_virtual_obs_lat_constraint()
          .lat_dist(),
      config_.open_space_roi_decider_config()
          .vertical_park_in_virtual_obs_lat_constraint()
          .obs_length());
}

double OpenSpaceObstacle::ParkInObsLengthLongitudinalConstraint(
    const bool is_right_turn_side,
    const std::shared_ptr<const perception::FreeSpaceOutArray>&
        free_space_array_ptr,
    const ParkingLotVertexType& parking_spot_enu) {
  double obs_length = config_.open_space_roi_decider_config()
                          .vertical_park_in_virtual_obstacle_lon_dis();
  if (nullptr == free_space_array_ptr ||
      free_space_array_ptr->freespace_out().empty()) {
    ADEBUG << "No freespace info to use";
    return obs_length;
  }
  common::math::Polygon2d corridor_filter;
  if (!BuildParkInCorridorFSFilterArea(is_right_turn_side, parking_spot_enu,
                                       &corridor_filter)) {
    ADEBUG << "build park in corridor fs filter area fails";
    return obs_length;
  }
  double nearest_fs_dist = kDefaultDistToBoundary;
  Vec2d nearest_fs_point;
  FindNearestFSPoint(free_space_array_ptr, corridor_filter,
                     corridor_filter.line_segments().at(1), &nearest_fs_dist,
                     &nearest_fs_point);
  ADEBUG << "nearest_fs_dist: " << nearest_fs_dist;
  ADEBUG << "nearest_fs_point: " << nearest_fs_point.x() << ", "
         << nearest_fs_point.y();
  return common::math::InterpolationOne(
      nearest_fs_dist +
          config_.open_space_roi_decider_config()
              .vertical_park_in_virtual_obs_lon_filter_height_min(),
      config_.open_space_roi_decider_config()
          .vertical_park_in_virtual_obs_lon_constraint()
          .height_dist(),
      config_.open_space_roi_decider_config()
          .vertical_park_in_virtual_obs_lon_constraint()
          .obs_length());
}

bool OpenSpaceObstacle::BuildParkInCorridorFSFilterArea(
    const bool is_right_turn_side, const ParkingLotVertexType& parking_spot_enu,
    common::math::Polygon2d* const corridor_filter) {
  if (nullptr == corridor_filter) {
    AERROR << "BuildParkInCorridorFSFilterArea input check fails";
    return false;
  }
  const double filter_lower_height =
      config_.open_space_roi_decider_config()
          .vertical_park_in_virtual_obs_lon_filter_height_min();
  const double filter_upper_height =
      config_.open_space_roi_decider_config()
          .vertical_park_in_virtual_obs_lon_filter_height_max();
  const double filter_width =
      config_.open_space_roi_decider_config()
          .vertical_park_in_virtual_obs_lon_filter_width();
  auto down_top = parking_spot_enu.at(0) - parking_spot_enu.at(1);
  down_top.Normalize();
  auto left_right = parking_spot_enu.at(3) - parking_spot_enu.at(0);
  left_right.Normalize();
  const auto slot_center =
      (parking_spot_enu.at(0) + parking_spot_enu.at(3)) / 2;
  const auto center_lower_point = slot_center + filter_lower_height * down_top;
  const auto center_upper_point = slot_center + filter_upper_height * down_top;
  *corridor_filter = is_right_turn_side
                         ? common::math::Polygon2d(std::vector<Vec2d>{
                               center_upper_point, center_lower_point,
                               center_lower_point + filter_width * left_right,
                               center_upper_point + filter_width * left_right})
                         : common::math::Polygon2d(std::vector<Vec2d>{
                               center_upper_point - filter_width * left_right,
                               center_lower_point - filter_width * left_right,
                               center_lower_point, center_upper_point});
  return corridor_filter->GetValid();
}

uint32_t OpenSpaceObstacle::ScenarioDiffcultyDecison(
    const AVPStatus::ParkingType& parking_type,
    const ParkLotInfo& park_lot_info,
    const std::shared_ptr<const perception::FreeSpaceOutArray>&
        free_space_array_ptr) {
  uint32_t ret = ScenarioDiffcultyType::NORMAL_SCENARIO;
  if (IsDeadendScenario(parking_type, park_lot_info.park_type,
                        park_lot_info.is_right_side, park_lot_info.vertices,
                        free_space_array_ptr)) {
    ret += ScenarioDiffcultyType::DEADEND_SCENARIO;
  }
  if (IsNarrowSpotScenario(park_lot_info)) {
    ret += ScenarioDiffcultyType::NARROW_SPOT_SCENARIO;
  }
  if (IsNarrowPassageScenario(parking_type, park_lot_info.park_type,
                              park_lot_info.vertices, free_space_array_ptr)) {
    ret += ScenarioDiffcultyType::NARROW_PASSAGE_SCENARIO;
  }

  return ret;
}

bool OpenSpaceObstacle::IsDeadendScenario(
    const AVPStatus::ParkingType& parking_type,
    const perception::ParkingLotOut::ParkType& park_lot_type,
    const bool is_right_slot, const ParkingLotVertexType& parking_spot_enu,
    const std::shared_ptr<const perception::FreeSpaceOutArray>&
        free_space_array_ptr) {
  if (nullptr == free_space_array_ptr) {
    return false;
  }
  if (parking_type != TL::planning::AVPStatus::PARKING_IN ||
      park_lot_type == perception::ParkingLotOut::LATERAL ||
      !config_.open_space_roi_decider_config()
           .enable_recognize_deadend_scenario()) {
    return false;
  }
  common::math::Polygon2d corrior_filter;
  if (!BuildDeadendScenarioDeciderFSFilterArea(is_right_slot, parking_spot_enu,
                                               &corrior_filter)) {
    ADEBUG << "Build Deadend Scenario Decider FS Filter Area fail";
    return false;
  }
  std::vector<std::pair<Vec2d, double>> filtered_fs;
  Vec2d point_vec;
  for (const auto& free_space_item : free_space_array_ptr->freespace_out()) {
    if (free_space_item.has_cls() &&
        free_space_item.cls() == perception::FreeSpaceOut::PEDESTRAIN) {
      ADEBUG << "ignore pedestrians";
      continue;
    }
    for (const auto& point : free_space_item.freespace_keypoint()) {
      point_vec.set_x(point.x());
      point_vec.set_y(point.y());
      if (corrior_filter.IsPointIn(point_vec)) {
        const double dist =
            corrior_filter.line_segments().at(1).DistanceTo(point_vec);
        filtered_fs.emplace_back(point_vec, dist);
      }
    }
  }
  std::sort(
      filtered_fs.begin(), filtered_fs.end(),
      [](const std::pair<Vec2d, double>& p1,
         const std::pair<Vec2d, double>& p2) { return p1.second < p2.second; });
  auto lower_iter = filtered_fs.begin();
  auto upper_iter = filtered_fs.begin();
  const double filter_height = 2.0;
  static constexpr double kDownUpDistThrehold = 1.2;
  while (lower_iter != filtered_fs.end()) {
    while (upper_iter != filtered_fs.end() &&
           upper_iter->second - lower_iter->second < filter_height) {
      ++upper_iter;
    }
    --upper_iter;
    if (upper_iter->first.DistanceTo(lower_iter->first) >=
        kDownUpDistThrehold) {
      return true;
    }
    ++lower_iter;
  }
  return false;
}

bool OpenSpaceObstacle::IsNarrowSpotScenario(const ParkLotInfo& park_lot_info) {
  return (park_lot_info.park_type == perception::ParkingLotOut::VERTICAL ||
          park_lot_info.park_type == perception::ParkingLotOut::OBLIQUE) &&
         park_lot_info.is_narrow_spot;
}

bool OpenSpaceObstacle::IsNarrowPassageScenario(
    const AVPStatus::ParkingType& parking_type,
    const perception::ParkingLotOut::ParkType& park_lot_type,
    const ParkingLotVertexType& parking_spot_enu,
    const std::shared_ptr<const perception::FreeSpaceOutArray>&
        free_space_array_ptr) {
  if (nullptr == free_space_array_ptr) {
    return false;
  }
  // only for vertical park in
  if (parking_type != TL::planning::AVPStatus::PARKING_IN ||
      park_lot_type != perception::ParkingLotOut::VERTICAL) {
    return false;
  }
  common::math::Polygon2d passage_corridor_filter;
  common::math::LineSegment2d filter_left_edge;
  if (!BuildNarrowPassageScenarioDeciderFSFilterArea(
          parking_spot_enu, &passage_corridor_filter, &filter_left_edge)) {
    return false;
  }
  static constexpr size_t kCheckGridNum = 10;
  static constexpr double kResolution = 2.0;
  static constexpr size_t kCountThreshold = 6;
  static constexpr size_t kLeftMidIndex = 4;
  static constexpr size_t kRightMidIndex = 5;
  std::vector<bool> narrow_passage_check_vec(kCheckGridNum, false);

  size_t narrow_passage_check_count = 0;
  Vec2d fs_point_vec;
  for (const auto& free_space_item : free_space_array_ptr->freespace_out()) {
    if (free_space_item.has_cls() &&
        free_space_item.cls() == perception::FreeSpaceOut::PEDESTRAIN) {
      ADEBUG << "ignore pedestrians";
      continue;
    }
    for (const auto& point : free_space_item.freespace_keypoint()) {
      fs_point_vec.set_x(point.x());
      fs_point_vec.set_y(point.y());
      if (passage_corridor_filter.IsPointIn(fs_point_vec)) {
        double dist_to_left_edge = filter_left_edge.DistanceTo(fs_point_vec);
        auto check_idx = static_cast<size_t>(dist_to_left_edge / kResolution);
        if (check_idx < narrow_passage_check_vec.size() &&
            !narrow_passage_check_vec.at(check_idx)) {
          narrow_passage_check_vec.at(check_idx) = true;
          narrow_passage_check_count++;
        }
      }
    }
  }
  return (narrow_passage_check_count >= kCountThreshold) &&
         narrow_passage_check_vec.at(kLeftMidIndex) &&
         narrow_passage_check_vec.at(kRightMidIndex);
}

bool OpenSpaceObstacle::GetParkOutCurbSeg(
    const AVPStatus::ParkingType& parking_type,
    const perception::ParkingLotOut::ParkType& park_lot_type,
    const ParkingLotVertexType& parking_spot_enu,
    const std::shared_ptr<const perception::FreeSpaceOutArray>&
        free_space_array_ptr,
    common::math::LineSegment2d* const reference_curb) {
  if (nullptr == free_space_array_ptr || nullptr == reference_curb) {
    return false;
  }
  common::math::Polygon2d corrior_filter;
  if (!BuildParkOutCurbFSFilterArea(parking_type, park_lot_type,
                                    parking_spot_enu, &corrior_filter)) {
    ADEBUG << "Build Park Out Curb FS Filter Area fail";
    return false;
  }
  double nearest_fs_dist = kDefaultDistToBoundary;
  perception::FreeSpaceOut nearest_fs_item;
  Vec2d point_vec;
  double dist_to_bounadry = 0.0;
  for (const auto& free_space_item : free_space_array_ptr->freespace_out()) {
    if (free_space_item.has_cls() &&
        free_space_item.cls() != perception::FreeSpaceOut::CURBSTONE) {
      ADEBUG << "ignore not curbs";
      continue;
    }
    for (const auto& point : free_space_item.freespace_keypoint()) {
      point_vec.set_x(point.x());
      point_vec.set_y(point.y());
      if (corrior_filter.IsPointIn(point_vec)) {
        dist_to_bounadry =
            corrior_filter.line_segments().at(3).DistanceTo(point_vec);
        if (dist_to_bounadry < nearest_fs_dist) {
          nearest_fs_item = free_space_item;
          nearest_fs_dist = dist_to_bounadry;
        }
      }
    }
  }
  if (nearest_fs_dist >= kDefaultDistToBoundary / 2) {
    ADEBUG << "fs item is not found";
    return false;
  }
  Vec2d nearest_left_fs_point;
  double nearest_left_fs_dist = kDefaultDistToBoundary;
  Vec2d nearest_right_fs_point;
  double nearest_right_fs_dist = kDefaultDistToBoundary;
  FindNearestFSPointInItem(nearest_fs_item, corrior_filter,
                           corrior_filter.line_segments().at(0),
                           &nearest_left_fs_dist, &nearest_left_fs_point);
  FindNearestFSPointInItem(nearest_fs_item, corrior_filter,
                           corrior_filter.line_segments().at(2),
                           &nearest_right_fs_dist, &nearest_right_fs_point);
  constexpr double kLeftRightThrehold = 1.2;
  if (nearest_left_fs_point.DistanceTo(nearest_right_fs_point) <
      kLeftRightThrehold) {
    return false;
  }
  *reference_curb = common::math::LineSegment2d(nearest_left_fs_point,
                                                nearest_right_fs_point);
  return true;
}

bool OpenSpaceObstacle::BuildDeadendScenarioDeciderFSFilterArea(
    const bool is_right_slot, const ParkingLotVertexType& parking_spot_enu,
    common::math::Polygon2d* const corridor_filter) {
  if (nullptr == corridor_filter) {
    AERROR << "BuildDeadendScenarioDeciderFSFilterArea input check fails";
    return false;
  }

  const double filter_width = 3.5;
  const double filter_upper_height = 5.5;
  const double filter_lower_height = 1.5;
  auto down_top = parking_spot_enu.at(0) - parking_spot_enu.at(1);
  down_top.Normalize();
  auto left_right = parking_spot_enu.at(3) - parking_spot_enu.at(0);
  left_right.Normalize();
  const auto slot_end =
      is_right_slot ? parking_spot_enu.at(3) : parking_spot_enu.at(0);
  const auto filter_lower_point = slot_end + filter_lower_height * down_top;
  const auto filter_upper_point = slot_end + filter_upper_height * down_top;
  *corridor_filter = is_right_slot
                         ? common::math::Polygon2d(std::vector<Vec2d>{
                               filter_upper_point, filter_lower_point,
                               filter_lower_point + filter_width * left_right,
                               filter_upper_point + filter_width * left_right})
                         : common::math::Polygon2d(std::vector<Vec2d>{
                               filter_upper_point - filter_width * left_right,
                               filter_lower_point - filter_width * left_right,
                               filter_lower_point, filter_upper_point});
  return corridor_filter->GetValid();
}

bool OpenSpaceObstacle::BuildNarrowSpotScenarioDeciderFSFilterArea(
    const ParkingLotVertexType& parking_spot_enu,
    common::math::Polygon2d* left_corridor_filter,
    common::math::Polygon2d* right_corridor_filter) {
  if (nullptr == left_corridor_filter || nullptr == right_corridor_filter) {
    AERROR << "BuildNarrowSpotScenarioDeciderFSFilterArea input check fails";
    return false;
  }
  const auto top_edge_mid =
      (parking_spot_enu.at(0) + parking_spot_enu.at(3)) * 0.5;
  auto down_top = parking_spot_enu.at(0) - parking_spot_enu.at(1);
  down_top.Normalize();
  auto left_right = parking_spot_enu.at(3) - parking_spot_enu.at(0);
  left_right.Normalize();
  const double filter_width = 3.0;
  const double filter_length =
      0.5 * (parking_spot_enu.at(0) - parking_spot_enu.at(1)).Length();

  std::vector<Vec2d> left_filter_points(4);
  left_filter_points.at(3) = top_edge_mid;
  left_filter_points.at(2) =
      left_filter_points.at(3) - filter_length * down_top;
  left_filter_points.at(1) =
      left_filter_points.at(2) - filter_width * left_right;
  left_filter_points.at(0) =
      left_filter_points.at(1) + filter_length * down_top;
  *left_corridor_filter = common::math::Polygon2d(left_filter_points);

  std::vector<Vec2d> right_filter_points(4);
  right_filter_points.at(0) = top_edge_mid;
  right_filter_points.at(1) =
      right_filter_points.at(0) - filter_length * down_top;
  right_filter_points.at(2) =
      right_filter_points.at(1) + filter_width * left_right;
  right_filter_points.at(3) =
      right_filter_points.at(2) + filter_length * down_top;
  *right_corridor_filter = common::math::Polygon2d(right_filter_points);

  return left_corridor_filter->GetValid() && right_corridor_filter->GetValid();
}

bool OpenSpaceObstacle::BuildNarrowPassageScenarioDeciderFSFilterArea(
    const ParkingLotVertexType& parking_spot_enu,
    common::math::Polygon2d* passage_corridor_filter,
    common::math::LineSegment2d* filter_left_edge) {
  if (passage_corridor_filter == nullptr || filter_left_edge == nullptr) {
    AERROR << "BuildNarrowPassageScenarioDeciderFSFilterArea inputs check "
              "fails";
    return false;
  }
  auto down_top = parking_spot_enu.at(0) - parking_spot_enu.at(1);
  down_top.Normalize();
  auto left_right = parking_spot_enu.at(3) - parking_spot_enu.at(0);
  left_right.Normalize();
  const auto top_edge_mid =
      (parking_spot_enu.at(0) + parking_spot_enu.at(3)) * 0.5;
  const double upper_filter_height = 4.5;
  const double lower_filter_height = 3.5;
  const double filter_width = 10.0;
  std::vector<Vec2d> outer_filter_points(4);
  /*
    * filter points ordered by
    * *******0-----------------3******
    * *******|                 |****
    * *******1-----------------2******
  */
  outer_filter_points.at(0) =
      top_edge_mid + down_top * upper_filter_height - left_right * filter_width;
  outer_filter_points.at(1) =
      top_edge_mid + down_top * lower_filter_height - left_right * filter_width;
  outer_filter_points.at(2) =
      top_edge_mid + down_top * lower_filter_height + left_right * filter_width;
  outer_filter_points.at(3) =
      top_edge_mid + down_top * upper_filter_height + left_right * filter_width;

  *filter_left_edge = common::math::LineSegment2d(outer_filter_points.at(1),
                                                  outer_filter_points.at(0));

  *passage_corridor_filter = common::math::Polygon2d(outer_filter_points);
  return passage_corridor_filter->GetValid();
}

bool OpenSpaceObstacle::BuildParkOutCurbFSFilterArea(
    const AVPStatus::ParkingType& parking_type,
    const perception::ParkingLotOut::ParkType& park_lot_type,
    const ParkingLotVertexType& parking_spot_enu,
    common::math::Polygon2d* corridor_filter) {
  if (nullptr == corridor_filter) {
    AERROR << "BuildParkOutCurbFSFilterArea input check fails";
    return false;
  }
  const bool is_park_out_right_side =
      (parking_type == planning::AVPStatus::PARKING_OUT_RIGHT &&
       park_lot_type != perception::ParkingLotOut::LATERAL) ||
      (parking_type == planning::AVPStatus::PARKING_OUT_LEFT &&
       park_lot_type == perception::ParkingLotOut::LATERAL);
  auto down_top = parking_spot_enu.at(0) - parking_spot_enu.at(1);
  down_top.Normalize();
  auto left_right = parking_spot_enu.at(3) - parking_spot_enu.at(0);
  left_right.Normalize();
  std::vector<Vec2d> filter_points(4);
  const double filter_gap_width = 1.0;
  const double filter_width = 1.5;
  const double filter_length = 6.0;
  if (is_park_out_right_side) {
    filter_points.at(0) = parking_spot_enu.at(3) +
                          filter_gap_width * left_right +
                          0.5 * filter_length * down_top;
    filter_points.at(1) = parking_spot_enu.at(3) +
                          filter_gap_width * left_right -
                          0.5 * filter_length * down_top;
    filter_points.at(2) = parking_spot_enu.at(3) +
                          (filter_gap_width + filter_width) * left_right -
                          0.5 * filter_length * down_top;
    filter_points.at(3) = parking_spot_enu.at(3) +
                          (filter_gap_width + filter_width) * left_right +
                          0.5 * filter_length * down_top;
  } else {
    filter_points.at(0) = parking_spot_enu.at(0) -
                          (filter_gap_width + filter_width) * left_right +
                          0.5 * filter_length * down_top;
    filter_points.at(1) = parking_spot_enu.at(0) -
                          (filter_gap_width + filter_width) * left_right -
                          0.5 * filter_length * down_top;
    filter_points.at(2) = parking_spot_enu.at(0) -
                          filter_gap_width * left_right -
                          0.5 * filter_length * down_top;
    filter_points.at(3) = parking_spot_enu.at(0) -
                          filter_gap_width * left_right +
                          0.5 * filter_length * down_top;
  }
  *corridor_filter = common::math::Polygon2d(filter_points);
  return corridor_filter->GetValid();
}

bool OpenSpaceObstacle::BuildVerticalSpotParkInVirtualObsFSFilterArea(
    const ParkingLotVertexType& parking_spot_enu,
    common::math::Polygon2d* left_corridor_filter,
    common::math::Polygon2d* right_corridor_filter) {
  if (nullptr == left_corridor_filter || nullptr == right_corridor_filter) {
    AERROR << "BuildNarrowScenarioDeciderFSFilterArea input check fails";
    return false;
  }
  const auto top_edge_mid =
      (parking_spot_enu.at(0) + parking_spot_enu.at(3)) * 0.5;
  auto down_top = parking_spot_enu.at(0) - parking_spot_enu.at(1);
  down_top.Normalize();
  auto left_right = parking_spot_enu.at(3) - parking_spot_enu.at(0);
  left_right.Normalize();
  const double filter_width = 3.0;
  const double filter_upper_length = 0.5;
  const double filter_lower_length =
      0.5 * (parking_spot_enu.at(0) - parking_spot_enu.at(1)).Length();

  std::vector<Vec2d> left_filter_points(4);
  left_filter_points.at(3) = top_edge_mid + filter_upper_length * down_top;
  left_filter_points.at(2) = top_edge_mid - filter_lower_length * down_top;
  left_filter_points.at(1) =
      left_filter_points.at(2) - filter_width * left_right;
  left_filter_points.at(0) =
      left_filter_points.at(3) - filter_width * left_right;
  *left_corridor_filter = common::math::Polygon2d(left_filter_points);

  std::vector<Vec2d> right_filter_points(4);
  right_filter_points.at(0) = top_edge_mid + filter_upper_length * down_top;
  right_filter_points.at(1) = top_edge_mid - filter_lower_length * down_top;
  right_filter_points.at(2) =
      right_filter_points.at(1) + filter_width * left_right;
  right_filter_points.at(3) =
      right_filter_points.at(0) + filter_width * left_right;
  *right_corridor_filter = common::math::Polygon2d(right_filter_points);

  return left_corridor_filter->GetValid() && right_corridor_filter->GetValid();
}

}  // namespace planning
}  // namespace TL
