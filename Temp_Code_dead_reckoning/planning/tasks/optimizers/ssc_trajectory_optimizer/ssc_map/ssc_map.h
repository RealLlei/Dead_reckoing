/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file ssc_map.h
 **/

#pragma once

#include <algorithm>
#include <array>
#include <iostream>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "planning/tasks/optimizers/ssc_trajectory_optimizer/ssc_map/semantics.h"
#include "planning/proto/task_config.pb.h"

namespace TL::planning {

using ObstacleMapType = uint8_t;
using SscMapDataType = uint8_t;

class SscMap {
 public:
  using GridMap3D = GridMapND<ObstacleMapType, 3>;
  using Config = SscTrajectoryOptimizerConfig::SscMapConfig;

  // struct Config {
  //   std::array<int, 3> map_size = {{1000, 100, 81}};            // s, d, t
  //   std::array<double, 3> map_resolution = {{0.25, 0.2, 0.1}};  // m, m, s
  //   std::array<std::string, 3> axis_name = {{"s", "d", "t"}};
  //   double s_back_len = 0.0;
  //   double kMaxLongitudinalVel = 50.0;
  //   double kMinLongitudinalVel = 0.0;
  //   double kMaxLongitudinalAcc = 3.0;
  //   double kMaxLongitudinalDecel = -8.0;  // Avg. driver max
  //   double kMaxLateralVel = 3.0;
  //   double kMaxLateralAcc = 2.5;

  //   int kMaxNumOfGridAlongTime = 2;

  //   std::array<int, 6> inflate_steps = {{20, 5, 10, 10, 1, 1}};
  // };

  explicit SscMap(const Config& config);

  ~SscMap() = default;

  const std::shared_ptr<GridMap3D>& p_3d_grid() const { return p_3d_grid_; }

  const std::shared_ptr<GridMap3D>& p_3d_inflated_grid() const {
    return p_3d_inflated_grid_;
  }

  Config config() const { return config_; }

  std::vector<DrivingCorridor> driving_corridor_vec() const {
    return driving_corridor_vec_;
  }

  std::vector<std::vector<SpatioTemporalSemanticCubeNd<2>>> final_corridor_vec()
      const {
    return final_corridor_vec_;
  }

  std::vector<int> if_corridor_valid() const { return if_corridor_valid_; }

  void set_initial_fs(const game_common::FrenetState& fs) { initial_fs_ = fs; }

  void UpdateMapOrigin(const game_common::FrenetState& ori_fs);

  void ConstructSscMap(
      const std::unordered_map<int, std::vector<FsVehicle>>&
          sur_vehicle_trajs_fs,
      const std::vector<std::array<double, 3>>& obstacle_grids);

  void InflateObstacleGrid(const game_common::VehicleParam& param);

  bool ConstructCorridorUsingInitialTrajectory(
      const std::shared_ptr<GridMap3D>& p_grid,
      const std::vector<FsVehicle>& trajs);

  void ClearGridMap();

  void ClearDrivingCorridor();

  bool GetFinalGlobalMetricCubesList();

  void ResetSscMap(const game_common::FrenetState& ini_fs);

 private:
  static bool CheckIfCubeIsFree(const std::shared_ptr<GridMap3D>& p_grid,
                                const AxisAlignedCubeNd<int, 3>& cube);

  static bool CheckIfPlaneIsFreeOnXAxis(
      const std::shared_ptr<GridMap3D>& p_grid,
      const AxisAlignedCubeNd<int, 3>& cube, const int& x);

  static bool CheckIfPlaneIsFreeOnYAxis(
      const std::shared_ptr<GridMap3D>& p_grid,
      const AxisAlignedCubeNd<int, 3>& cube, const int& y);

  static bool CheckIfPlaneIsFreeOnZAxis(
      const std::shared_ptr<GridMap3D>& p_grid,
      const AxisAlignedCubeNd<int, 3>& cube, const int& z);

  static bool CheckIfCubeContainsSeed(const AxisAlignedCubeNd<int, 3>& cube_a,
                                      const std::array<int, 3>& seed);

  static void GetInitialCubeUsingSeed(const std::array<int, 3>& seed_0,
                                      const std::array<int, 3>& seed_1,
                                      AxisAlignedCubeNd<int, 3>* cube);

  static void GetTimeCoveredCubeIndices(const DrivingCorridor* p_corridor,
                                        const int& start_id, const int& dir,
                                        const int& t_trans,
                                        std::vector<int>* idx_list);

  static void CorridorRelaxation(const std::shared_ptr<GridMap3D>& p_grid,
                                 DrivingCorridor* p_corridor);

  void InflateCubeIn3dGrid(
      const std::shared_ptr<GridMap3D>& p_grid,
      const std::array<bool, 6>& dir_disabled,
      const SscTrajectoryOptimizerConfig::InflateSteps& inflate_steps,
      AxisAlignedCubeNd<int, 3>* cube);

  static void GetInflationDirections(const bool& if_first_cube,
                                     std::array<bool, 6>* dirs_disabled);

  static bool InflateCubeOnXPosAxis(const std::shared_ptr<GridMap3D>& p_grid,
                                    const int& n_step,
                                    AxisAlignedCubeNd<int, 3>* cube);
  static bool InflateCubeOnXNegAxis(const std::shared_ptr<GridMap3D>& p_grid,
                                    const int& n_step,
                                    AxisAlignedCubeNd<int, 3>* cube);
  static bool InflateCubeOnYPosAxis(const std::shared_ptr<GridMap3D>& p_grid,
                                    const int& n_step,
                                    AxisAlignedCubeNd<int, 3>* cube);
  static bool InflateCubeOnYNegAxis(const std::shared_ptr<GridMap3D>& p_grid,
                                    const int& n_step,
                                    AxisAlignedCubeNd<int, 3>* cube);
  static bool InflateCubeOnZPosAxis(const std::shared_ptr<GridMap3D>& p_grid,
                                    const int& n_step,
                                    AxisAlignedCubeNd<int, 3>* cube);
  static bool InflateCubeOnZNegAxis(const std::shared_ptr<GridMap3D>& p_grid,
                                    const int& n_step,
                                    AxisAlignedCubeNd<int, 3>* cube);

  void FillStaticPart(const std::vector<std::array<double, 3>>& obs_grid_fs);

  void FillDynamicPart(const std::unordered_map<int, std::vector<FsVehicle>>&
                           sur_vehicle_trajs_fs);

  bool FillMapWithFsVehicleTraj(const std::vector<FsVehicle>& traj);

  Config config_;
  std::shared_ptr<GridMapND<SscMapDataType, 3>> p_3d_grid_;
  std::shared_ptr<GridMapND<SscMapDataType, 3>> p_3d_inflated_grid_;
  game_common::FrenetState initial_fs_;
  std::vector<DrivingCorridor> driving_corridor_vec_;
  std::vector<int> if_corridor_valid_;
  std::vector<std::vector<SpatioTemporalSemanticCubeNd<2>>> final_corridor_vec_;
};

}  // namespace TL::planning
