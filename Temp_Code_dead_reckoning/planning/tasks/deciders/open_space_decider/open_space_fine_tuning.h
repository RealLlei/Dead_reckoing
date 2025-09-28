#pragma once

/*
 * Copyright (c) TL Technologies Co., Ltd. 2022. All rights reserved.
 * Description:  open_space_fine_tuning.h
 */

#include <complex>
#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>
#include "common/math/math_utils.h"
#include "planning/tasks/deciders/open_space_decider/open_space_obstacle.h"
#include "proto/common/pnc_point.pb.h"

namespace TL {
namespace planning {

class OpenSpaceFineTuningGrid {
 public:
  explicit OpenSpaceFineTuningGrid(double x, double y, double x_grid_resolution,
                                   double y_grid_resolution,
                                   const std::vector<double>& xy_bounds)
      : x_(x),
        y_(y),
        x_grid_resolution_(x_grid_resolution),
        y_grid_resolution_(y_grid_resolution),
        xy_bounds_(xy_bounds) {
    if (xy_bounds_.size() != 4) {
      CHECK_EQ(xy_bounds_.size(), 4U)
          << "xy_bounds size is not 4, but" << xy_bounds_.size();
    }
    x = common::math::Clamp(x, xy_bounds[0], xy_bounds[1]);
    y = common::math::Clamp(y, xy_bounds[2], xy_bounds[3]);
    int x_grid = static_cast<int>(round(x / x_grid_resolution));
    int y_grid = static_cast<int>(round(y / y_grid_resolution));
    index_ = absl::StrCat(x_grid, "_", y_grid);
  }

  ~OpenSpaceFineTuningGrid() = default;

  std::vector<std::shared_ptr<OpenSpaceFineTuningGrid>> next() {
    std::vector<std::shared_ptr<OpenSpaceFineTuningGrid>> next_grids;
    const std::vector<double> x_offsets = {-x_grid_resolution_,
                                           0.0,
                                           x_grid_resolution_,
                                           -x_grid_resolution_,
                                           x_grid_resolution_,
                                           -x_grid_resolution_,
                                           0.0,
                                           x_grid_resolution_};
    const std::vector<double> y_offsets = {
        -y_grid_resolution_, -y_grid_resolution_, -y_grid_resolution_, 0.0, 0.0,
        y_grid_resolution_,  y_grid_resolution_,  y_grid_resolution_};
    const auto offsets_num = x_offsets.size();
    for (int i = 0; i < offsets_num; ++i) {
      if (x_ + x_offsets.at(i) < xy_bounds_[0] ||
          x_ + x_offsets.at(i) > xy_bounds_[1]) {
        continue;
      }
      if (y_ + y_offsets.at(i) < xy_bounds_[2] ||
          y_ + y_offsets.at(i) > xy_bounds_[3]) {
        continue;
      }
      next_grids.emplace_back(std::make_shared<OpenSpaceFineTuningGrid>(
          x_ + x_offsets.at(i), y_ + y_offsets.at(i), x_grid_resolution_,
          y_grid_resolution_, xy_bounds_));
    }
    next_grids.insert(next_grids.end(), extra_next_grids_.begin(),
                      extra_next_grids_.end());
    return next_grids;
  }

  void add_next_grid(
      const std::shared_ptr<OpenSpaceFineTuningGrid>& extra_next_grid) {
    extra_next_grids_.emplace_back(extra_next_grid);
  }

  std::string index() const { return index_; }

  double x() const { return x_; }

  double y() const { return y_; }

 private:
  double x_, y_, x_grid_resolution_, y_grid_resolution_;
  std::vector<double> xy_bounds_;
  std::string index_;
  std::vector<std::shared_ptr<OpenSpaceFineTuningGrid>> extra_next_grids_;
};

struct OpenSpaceFineTuningObs {
  std::vector<std::pair<common::math::LineSegment2d, double>> obs_segments_pair;
  std::vector<double> original_buffer;
};

struct OpenSpaceFineTuningConfig {
  double x_grid_resolution = 0.01;
  double y_grid_resolution = 0.01;
  std::vector<double> xy_bounds = {0.0, 0.0, 0.0, 0.0};
  double w_x = 1.0;
  double w_y = 1.0;
};

struct cmp {
  bool operator()(const std::pair<std::string, double>& left,
                  const std::pair<std::string, double>& right) const {
    return left.second >= right.second;
  }
};

class OpenSpaceFineTuning {
 public:
  explicit OpenSpaceFineTuning(
      const TaskConfig& config,
      const std::shared_ptr<DependencyInjector>& injector);
  ~OpenSpaceFineTuning() = default;

  TL::common::Status Process(
      bool is_entered_lateral_slot_domain, bool is_consider_wheel_mask,
      const AVPStatus::ParkingType& parking_type,
      const planning::ParkLotInfo& park_lot,
      const OpenSpacePathInfo& open_space_path_info,
      const std::vector<std::pair<common::math::LineSegment2d, double>>&
          linked_obstacles_segments_vec,
      const std::vector<std::pair<common::math::LineSegment2d, double>>&
          high_curb_obstacles_segments_vec,
      const common::PathPoint& veh_point,
      const common::math::LineSegment2d& reference_curb,
      common::PathPoint* end_pose_enu_ptr);

  TL::common::Status Reset();

  double lon_fine_tune_dis() const { return lon_fine_tune_dis_; }

  double lat_fine_tune_dis() const { return lat_fine_tune_dis_; }

  double yaw_fine_tune_rad() const { return yaw_fine_tune_rad_; }

  bool is_fine_tune_update() const { return is_fine_tune_update_; }

 private:
  void InitConfig(bool const is_lat_slot, bool const is_enable_lat_fine_tune,
                  bool const is_enable_lon_fine_tune) {
    search_config_.x_grid_resolution =
        config_.open_space_roi_decider_config().park_in_lat_adjust_resolution();
    search_config_.y_grid_resolution =
        config_.open_space_roi_decider_config().park_in_lon_adjust_resolution();
    search_config_.xy_bounds.resize(4);
    if (is_enable_lat_fine_tune) {
      search_config_.xy_bounds[0] =
          is_lat_slot
              ? -1 * config_.open_space_roi_decider_config()
                         .park_in_lateral_target_lat_adjust_threshold()
              : -1 * config_.open_space_roi_decider_config()
                         .park_in_vertical_target_lat_adjust_threshold();
      search_config_.xy_bounds[1] =
          is_lat_slot ? config_.open_space_roi_decider_config()
                            .park_in_lateral_target_lat_adjust_threshold()
                      : config_.open_space_roi_decider_config()
                            .park_in_vertical_target_lat_adjust_threshold();
    } else {
      search_config_.xy_bounds[0] = 0.0;
      search_config_.xy_bounds[1] = 0.0;
    }
    if (is_enable_lon_fine_tune) {
      search_config_.xy_bounds[2] = 0.0;
      search_config_.xy_bounds[3] =
          is_lat_slot ? config_.open_space_roi_decider_config()
                            .park_in_lateral_target_lon_adjust_threshold()
                      : config_.open_space_roi_decider_config()
                            .park_in_vertical_target_lon_adjust_threshold();
    } else {
      search_config_.xy_bounds[2] = 0.0;
      search_config_.xy_bounds[3] = 0.0;
    }
  }

  /**
   * @brief 
   * 
   * @param veh_point 
   * @param park_lot_info 
   * @param original_obs_segments_pair 
   * @param linked_obs_segments_pair 
   * @param high_curb_obs_segments_pair 
   * @param origin 
   * @param rotate_angle 
   */
  void FineTuneParkInTargetBasedOnObs(
      const common::PathPoint& veh_point,
      const planning::ParkLotInfo& park_lot_info,
      const std::vector<std::pair<common::math::LineSegment2d, double>>&
          original_obs_segments_pair,
      const std::vector<std::pair<common::math::LineSegment2d, double>>&
          linked_obs_segments_pair,
      const std::vector<std::pair<common::math::LineSegment2d, double>>&
          high_curb_obs_segments_pair,
      const common::math::Vec2d& origin, double rotate_angle);

  /**
    * @brief 
    * 
    * @param parking_type 
    * @param sensor_type 
    * @param park_lot_type 
    * @param parking_spot_enu 
    * @param veh_point 
    * @param reference_curb 
    * @return true 
    * @return false 
    */
  bool FineTuneParkOutTargetBasedOnObs(
      const AVPStatus::ParkingType& parking_type,
      const perception::ParkingLotOut::SenType& sensor_type,
      const perception::ParkingLotOut::ParkType& park_lot_type,
      const ParkingLotVertexType& parking_spot_enu,
      const common::PathPoint& veh_point,
      const common::math::LineSegment2d& reference_curb);

  /**
   * @brief 
   * 
   * @param is_lat_slot 
   * @param origin 
   * @param rotate_angle 
   * @return true 
   * @return false 
   */
  bool IsBlockByInnerFs(bool is_lat_slot, const common::math::Vec2d& origin,
                        double rotate_angle);

  /**
   * @brief 
   * 
   * @param is_lat_slot 
   * @param is_parking_lot_update 
   * @param linked_obs_segments_pair 
   * @return true 
   * @return false 
   */
  bool IsTargetCollisionWithLinkedObs(
      bool is_lat_slot, bool is_parking_lot_update,
      const std::vector<std::pair<common::math::LineSegment2d, double>>&
          linked_obs_segments_pair);

  /**
   * @brief 
   * 
   * @param is_lat_slot 
   * @param is_parking_lot_update 
   * @param high_curb_obs_segments_pair 
   * @return true 
   * @return false 
   */
  bool IsTargetCollisionWithHighCurbObs(
      bool is_lat_slot, bool is_parking_lot_update,
      const std::vector<std::pair<common::math::LineSegment2d, double>>&
          high_curb_obs_segments_pair);

  /**
   * @brief 
   * 
   * @param is_lat_slot 
   * @param lat_fine_tune_dis 
   * @param lon_fine_tune_dis 
   * @param idx_obs 
   * @param fine_tune_obs 
   */
  void UpdateAdjustBuffer(
      bool is_lat_slot, double lat_fine_tune_dis, double lon_fine_tune_dis,
      int idx_obs,
      const std::shared_ptr<OpenSpaceFineTuningObs>& fine_tune_obs);

  /**
   * @brief 
   * 
   * @param is_lat_slot
   * @param obs_segments_pair 
   * @param lon_fine_tune_dis_ptr
   */
  void LonFineTuneTargetBasedOnObs(
      bool is_lat_slot,
      const std::vector<std::pair<common::math::LineSegment2d, double>>&
          obs_segments_pair,
      double* lon_fine_tune_dis_ptr);

  /**
   * @brief 
   * 
   * @param is_lat_slot
   * @param obs_segments_pair 
   * @param lat_fine_tune_dis_ptr
   */
  void LatFineTuneTargetBasedOnObs(
      bool is_lat_slot,
      const std::vector<std::pair<common::math::LineSegment2d, double>>&
          obs_segments_pair,
      double* lat_fine_tune_dis_ptr);

  /**
   * @brief 
   * 
   * @param obs_seg 
   * @param left_boundary 
   * @param right_boundary 
   * @param lon_dis_threshold 
   * @return double 
   */
  static double LonDistanceWithLatBoundary(
      const common::math::LineSegment2d& obs_seg, double left_boundary,
      double right_boundary, double lon_dis_threshold);

  /**
   * @brief 
   * 
   * @param veh_point 
   * @param is_right_side 
   * @param park_lot_type 
   * @param origin_point 
   * @param target_heading 
   * @param lat_space 
   * @param is_consider_wheel_mask
   * @param is_parking_inwards
   */
  void FineTuneTargetBasedOnEndReplanPose(
      const common::PathPoint& veh_point, bool is_right_side,
      const perception::ParkingLotOut::ParkType& park_lot_type,
      const Vec2d& origin_point, double target_heading,
      const std::pair<double, double>& lat_space, bool is_consider_wheel_mask,
      bool is_parking_inwards);

  /**
   * @brief 
   * 
   * @param is_lat_slot 
   * @param is_right_side 
   * @param real_end_replan_point 
   * @param end_point 
   * @param lat_fine_tune_dis_ptr 
   */
  static void LatFineTuneTargetBasedOnEndReplanPose(
      bool is_lat_slot, bool is_right_side,
      const common::math::Vec2d& real_end_replan_point,
      const common::math::Vec2d& end_point, double* lat_fine_tune_dis_ptr);

  /**
   * @brief 
   * 
   * @param is_lat_slot 
   * @param is_right_side 
   * @param is_consider_wheel_mask
   * @param is_parking_inwards
   * @param lat_space 
   * @param real_end_replan_point 
   * @param end_point 
   * @param lon_fine_tune_dis_ptr 
   */
  void LonFineTuneTargetBasedOnEndReplanPose(
      bool is_lat_slot, bool is_right_side, bool is_consider_wheel_mask,
      bool is_parking_inwards, const std::pair<double, double>& lat_space,
      const common::math::Vec2d& real_end_replan_point,
      const common::math::Vec2d& end_point, double* lon_fine_tune_dis_ptr);

  /**
   * @brief
   *
   * @param is_lat_slot
   * @param obstacles_segments_vec
   * @param lat_space
   */
  void FineTuneTargetBasedOnNarrowLatSpot(
      bool is_lat_slot,
      const std::vector<std::pair<common::math::LineSegment2d, double>>&
          obstacles_segments_vec,
      const std::pair<double, double>& lat_space);

  /**
   * @brief
   *
   * @param grid
   * @param is_lat_slot
   * @param obstacles_segments_vec
   * @param visited_grids
   */
  void AddCutOffSearchGrids(
      const std::shared_ptr<OpenSpaceFineTuningGrid>& grid, bool is_lat_slot,
      const std::vector<std::pair<common::math::LineSegment2d, double>>&
          obstacles_segments_vec,
      std::unordered_set<std::string>* visited_grids);

  /**
   * @brief
   *
   * @param is_lat_slot
   * @param is_right_side
   * @param lat_fine_tune_dis
   * @param lon_fine_tune_dis
   * @param fine_tune_obs
   * @return true
   * @return false
   */
  bool ValidityCheck(
      bool is_lat_slot, bool is_right_side, double lat_fine_tune_dis,
      double lon_fine_tune_dis,
      const std::shared_ptr<OpenSpaceFineTuningObs>& fine_tune_obs);

  /**
   * @brief
   *
   * @param obs_seg
   * @param bottom_boundary
   * @param up_boundary
   * @param left_lat_dis_threshold
   * @param right_lat_dis_threshold
   */
  static std::pair<double, double> LatDistanceWithLonBoundary(
      const common::math::LineSegment2d& obs_seg, double bottom_boundary,
      double up_boundary, double left_lat_dis_threshold,
      double right_lat_dis_threshold);

  /**
   * @brief
   *
   * @param is_lat_slot
   * @param lon_fine_tune_dis
   * @param obstacles_segments_vec
   * @param is_consider_buffer
   * @param is_consider_whole_spot
   */
  std::pair<double, double> GetSpotLatSpace(
      bool is_lat_slot, double lon_fine_tune_dis,
      const std::vector<std::pair<common::math::LineSegment2d, double>>&
          obstacles_segments_vec,
      bool is_consider_buffer = true, bool is_consider_whole_spot = true);

  /**
   * @brief
   *
   * @param is_lat_slot
   * @param lat_fine_tune_dis
   * @param obstacles_segments_vec
   */
  double GetSpotLonSpace(
      bool is_lat_slot, double lat_fine_tune_dis,
      const std::vector<std::pair<common::math::LineSegment2d, double>>&
          obstacles_segments_vec);

  /**
   * @brief 
   * 
   * @param is_lat_slot 
   * @param obstacles_segments_vec 
   * @return std::pair<double, double>
   */
  std::pair<double, double> GetTopHalfLatSpace(
      bool is_lat_slot,
      const std::vector<std::pair<common::math::LineSegment2d, double>>&
          obstacles_segments_vec);

  common::math::Vec2d GetFineTuneTarget();
  common::math::Vec2d GetFineTuneTarget(double lat_fine_tune_dis,
                                        double lon_fine_tune_dis);

  std::shared_ptr<DependencyInjector> injector_;
  TaskConfig config_;
  OpenSpaceFineTuningConfig search_config_;
  TL::common::VehicleParam vehicle_params_;
  bool is_entered_lateral_slot_domain_ = false;
  double lon_fine_tune_dis_ = 0.0;
  double lat_fine_tune_dis_ = 0.0;
  double yaw_fine_tune_rad_ = 0.0;
  double target_adc_coord_heading_ = 0.0;
  common::PathPoint raw_end_pose_enu_;
  bool need_lat_fine_tune_ = false;
  common::PathPoint real_end_replan_point_;
  bool has_end_replaned_ = false;
  bool has_narrow_lon_tune_ = false;
  double pre_narrow_lon_fine_tune_dis_ = 0.0;
  bool is_fine_tune_update_ = false;
  std::vector<std::pair<common::math::LineSegment2d, double>> linked_fs_obs_;
  bool is_init_park_in_ = true;
};
}  // namespace planning
}  // namespace TL
