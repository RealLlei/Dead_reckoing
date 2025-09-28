/*
 * Copyright (c) TL Technologies Co., Ltd. 2024. All rights reserved.
 * Description:  ssc_map.cc
 */

#include "planning/tasks/optimizers/ssc_trajectory_optimizer/ssc_map/ssc_map.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "common/file/file.h"

namespace TL::planning {

SscMap::SscMap(const SscMap::Config& config) : config_(config) {
  std::array<int, 3> dims_size{
      {config_.map_size_x(), config_.map_size_y(), config_.map_size_z()}};
  std::array<double, 3> dims_resolution{
      {config_.map_resl_x(), config_.map_resl_y(), config_.map_resl_z()}};
  std::array<std::string, 3> dims_name{{"s", "l", "t"}};
  p_3d_grid_ = std::make_shared<GridMapND<SscMapDataType, 3>>(
      dims_size, dims_resolution, dims_name);
  p_3d_inflated_grid_ = std::make_shared<GridMapND<SscMapDataType, 3>>(
      dims_size, dims_resolution, dims_name);
}

void SscMap::ResetSscMap(const game_common::FrenetState& ini_fs) {
  ClearDrivingCorridor();
  ClearGridMap();
  UpdateMapOrigin(ini_fs);
}

void SscMap::UpdateMapOrigin(const game_common::FrenetState& ori_fs) {
  initial_fs_ = ori_fs;

  std::array<double, 3> map_origin{};
  map_origin[0] = ori_fs.s_state[0] - config_.s_back_len();
  map_origin[1] =
      -1 * (config_.map_size_y() - 1) * config_.map_resl_y() / 2.0;  // d
  map_origin[2] = ori_fs.time_stamp;                                 // t

  p_3d_grid_->set_origin(map_origin);
  p_3d_inflated_grid_->set_origin(map_origin);
}

void SscMap::GetInitialCubeUsingSeed(const std::array<int, 3>& seed_0,
                                     const std::array<int, 3>& seed_1,
                                     AxisAlignedCubeNd<int, 3>* const cube) {
  std::array<int, 3> lb{};
  std::array<int, 3> ub{};
  lb[0] = std::min(seed_0[0], seed_1[0]);
  lb[1] = std::min(seed_0[1], seed_1[1]);
  lb[2] = std::min(seed_0[2], seed_1[2]);
  ub[0] = std::max(seed_0[0], seed_1[0]);
  ub[1] = std::max(seed_0[1], seed_1[1]);
  ub[2] = std::max(seed_0[2], seed_1[2]);

  *cube = AxisAlignedCubeNd<int, 3>(ub, lb);
}

void SscMap::ConstructSscMap(
    const std::unordered_map<int, std::vector<FsVehicle>>& sur_vehicle_trajs_fs,
    const std::vector<std::array<double, 3>>& obstacle_grids) {
  p_3d_grid_->clear_data();
  p_3d_inflated_grid_->clear_data();
  FillStaticPart(obstacle_grids);
  FillDynamicPart(sur_vehicle_trajs_fs);
}

void SscMap::GetInflationDirections(const bool& if_first_cube,
                                    std::array<bool, 6>* const dirs_disabled) {
  (*dirs_disabled)[0] = false;
  (*dirs_disabled)[1] = false;
  (*dirs_disabled)[2] = false;
  (*dirs_disabled)[3] = false;
  (*dirs_disabled)[4] = false;
  (*dirs_disabled)[5] = !if_first_cube;
}

void SscMap::ClearGridMap() {
  p_3d_grid_->clear_data();
  p_3d_inflated_grid_->clear_data();
}

void SscMap::ClearDrivingCorridor() {
  driving_corridor_vec_.clear();
}

bool SscMap::ConstructCorridorUsingInitialTrajectory(
    const std::shared_ptr<GridMap3D>& p_grid,
    const std::vector<FsVehicle>& trajs) {
  // ~ Stage I: Get seeds
  std::vector<std::array<int, 3>> traj_seeds;
  int num_states = static_cast<int>(trajs.size());
  if (num_states > 1) {
    bool first_seed_determined = false;
    for (int k = 0; k < num_states; ++k) {
      std::array<double, 3> p_w = {};
      if (!first_seed_determined) {
        double s_0 = initial_fs_.s_state[0];
        double d_0 = initial_fs_.l_state[0];
        double t_0 = initial_fs_.time_stamp;
        std::array<double, 3> p_w_0 = {s_0, d_0, t_0};
        auto coord_0 = p_grid->GetCoordUsingGlobalPosition(p_w_0);

        double s_1 = trajs[k].frenet_state.s_state[0];
        double d_1 = trajs[k].frenet_state.l_state[0];
        double t_1 = trajs[k].frenet_state.time_stamp;
        std::array<double, 3> p_w_1 = {s_1, d_1, t_1};
        auto coord_1 = p_grid->GetCoordUsingGlobalPosition(p_w_1);
        // * remove the states out of range
        if (!p_grid->CheckCoordInRange(coord_1)) {
          continue;
        }
        // earlier than start time
        if (coord_1[2] <= 0) {
          continue;
        }

        first_seed_determined = true;
        traj_seeds.emplace_back(coord_0);
        traj_seeds.emplace_back(coord_1);
      } else {
        double s = trajs[k].frenet_state.s_state[0];
        double d = trajs[k].frenet_state.l_state[0];
        double t = trajs[k].frenet_state.time_stamp;
        p_w = {s, d, t};
        auto coord = p_grid->GetCoordUsingGlobalPosition(p_w);
        // * remove the states out of range
        if (!p_grid->CheckCoordInRange(coord)) {
          continue;
        }
        traj_seeds.emplace_back(coord);
      }
    }
  }

  // ~ Stage II: Inflate cubes
  DrivingCorridor driving_corridor;
  bool is_valid = true;
  auto seed_num = static_cast<int>(traj_seeds.size());
  if (seed_num < 2) {
    driving_corridor.is_valid = false;
    driving_corridor_vec_.push_back(driving_corridor);
    is_valid = false;
    return false;
  }
  for (int i = 0; i < seed_num; ++i) {
    if (i == 0) {
      AxisAlignedCubeNd<int, 3> cube{};
      GetInitialCubeUsingSeed(traj_seeds[i], traj_seeds[i + 1], &cube);
      if (!CheckIfCubeIsFree(p_grid, cube)) {
        AERROR << "[Ssc] SccMap - Initial cube is not free, seed id: " << i;

        DrivingCube driving_cube;
        driving_cube.cube = cube;
        driving_cube.seeds.push_back(traj_seeds[i]);
        driving_cube.seeds.push_back(traj_seeds[i + 1]);
        driving_corridor.cubes.push_back(driving_cube);

        driving_corridor.is_valid = false;
        driving_corridor_vec_.push_back(driving_corridor);
        is_valid = false;
        break;
      }

      std::array<bool, 6> dirs_disabled = {false, false, false,
                                           false, false, false};
      InflateCubeIn3dGrid(p_grid, dirs_disabled, config_.inflate_steps(),
                          &cube);

      DrivingCube driving_cube;
      driving_cube.cube = cube;
      driving_cube.seeds.push_back(traj_seeds[i]);
      driving_corridor.cubes.push_back(driving_cube);
    } else {
      if (CheckIfCubeContainsSeed(driving_corridor.cubes.back().cube,
                                  traj_seeds[i])) {
        driving_corridor.cubes.back().seeds.push_back(traj_seeds[i]);
        continue;
      }
      // ~ Get the last seed in cube
      const auto& seed_r = driving_corridor.cubes.back().seeds.back();
      driving_corridor.cubes.back().seeds.pop_back();
      // ~ Cut cube on time axis
      driving_corridor.cubes.back().cube.upper_bound[2] = seed_r[2];
      i = i - 1;

      AxisAlignedCubeNd<int, 3> cube{};
      GetInitialCubeUsingSeed(traj_seeds[i], traj_seeds[i + 1], &cube);

      if (!CheckIfCubeIsFree(p_grid, cube)) {
        AERROR << "[Ssc] SccMap - Initial cube is not free, seed id: " << i;
        DrivingCube driving_cube;
        driving_cube.cube = cube;
        driving_cube.seeds.push_back(traj_seeds[i]);
        driving_cube.seeds.push_back(traj_seeds[i + 1]);
        driving_corridor.cubes.push_back(driving_cube);

        driving_corridor.is_valid = false;
        driving_corridor_vec_.push_back(driving_corridor);
        is_valid = false;
        break;
      }

      std::array<bool, 6> dirs_disabled = {false, false, false,
                                           false, false, false};
      InflateCubeIn3dGrid(p_grid, dirs_disabled, config_.inflate_steps(),
                          &cube);
      DrivingCube driving_cube;
      driving_cube.cube = cube;
      driving_cube.seeds.push_back(traj_seeds[i]);
      driving_corridor.cubes.push_back(driving_cube);
    }
  }
  if (is_valid) {
    // CorridorRelaxation(p_grid, &driving_corridor);
    // ~ Cut cube on time axis
    driving_corridor.cubes.back().cube.upper_bound[2] = traj_seeds.back()[2];
    driving_corridor.is_valid = true;
    driving_corridor_vec_.push_back(driving_corridor);
  }

  return true;
}

void SscMap::GetTimeCoveredCubeIndices(const DrivingCorridor* p_corridor,
                                       const int& start_idx, const int& dir,
                                       const int& t_trans,
                                       std::vector<int>* idx_list) {
  int dt = 0;
  int num_cube = static_cast<int>(p_corridor->cubes.size());
  int idx = start_idx;
  while (idx < num_cube && idx >= 0) {
    dt += p_corridor->cubes[idx].cube.upper_bound[2] -
          p_corridor->cubes[idx].cube.lower_bound[2];
    idx_list->push_back(idx);
    if (dir == 1) {
      ++idx;
    } else {
      --idx;
    }
    if (dt >= t_trans) {
      break;
    }
  }
}

void SscMap::CorridorRelaxation(const std::shared_ptr<GridMap3D>& p_grid,
                                DrivingCorridor* p_corridor) {
  std::array<int, 2> margin = {{50, 10}};
  int t_trans = 7;
  int num_cube = static_cast<int>(p_corridor->cubes.size());
  for (int i = 0; i < num_cube - 1; ++i) {
    {
      // ~ Enable s direction
      int cube_0_lb = p_corridor->cubes[i].cube.lower_bound[0];
      int cube_0_ub = p_corridor->cubes[i].cube.upper_bound[0];

      int cube_1_lb = p_corridor->cubes[i + 1].cube.lower_bound[0];
      int cube_1_ub = p_corridor->cubes[i + 1].cube.upper_bound[0];

      if (abs(cube_0_ub - cube_1_lb) < margin[0]) {
        int room = margin[0] - abs(cube_0_ub - cube_1_lb);
        // ~ Upward
        std::vector<int> up_idx_list;
        GetTimeCoveredCubeIndices(p_corridor, i + 1, 1, t_trans, &up_idx_list);
        // ~ Downward
        std::vector<int> down_idx_list;
        GetTimeCoveredCubeIndices(p_corridor, i, 0, t_trans, &down_idx_list);
        for (const auto& idx : up_idx_list) {
          InflateCubeOnXNegAxis(p_grid, room, &(p_corridor->cubes[idx].cube));
        }
        for (const auto& idx : down_idx_list) {
          InflateCubeOnXPosAxis(p_grid, room, &(p_corridor->cubes[idx].cube));
        }
      }
      if (abs(cube_0_lb - cube_1_ub) < margin[0]) {
        int room = margin[0] - abs(cube_0_lb - cube_1_ub);
        // ~ Upward
        std::vector<int> up_idx_list;
        GetTimeCoveredCubeIndices(p_corridor, i + 1, 1, t_trans, &up_idx_list);
        // ~ Downward
        std::vector<int> down_idx_list;
        GetTimeCoveredCubeIndices(p_corridor, i, 0, t_trans, &down_idx_list);
        for (const auto& idx : up_idx_list) {
          InflateCubeOnXPosAxis(p_grid, room, &(p_corridor->cubes[idx].cube));
        }
        for (const auto& idx : down_idx_list) {
          InflateCubeOnXNegAxis(p_grid, room, &(p_corridor->cubes[idx].cube));
        }
      }
    }
    {
      // ~ Enable d direction
      int cube_0_lb = p_corridor->cubes[i].cube.lower_bound[1];
      int cube_0_ub = p_corridor->cubes[i].cube.upper_bound[1];

      int cube_1_lb = p_corridor->cubes[i + 1].cube.lower_bound[1];
      int cube_1_ub = p_corridor->cubes[i + 1].cube.upper_bound[1];

      if (abs(cube_0_ub - cube_1_lb) < margin[1]) {
        int room = margin[1] - abs(cube_0_ub - cube_1_lb);
        // ~ Upward
        std::vector<int> up_idx_list;
        GetTimeCoveredCubeIndices(p_corridor, i + 1, 1, t_trans, &up_idx_list);
        // ~ Downward
        std::vector<int> down_idx_list;
        GetTimeCoveredCubeIndices(p_corridor, i, 0, t_trans, &down_idx_list);
        for (const auto& idx : up_idx_list) {
          InflateCubeOnYNegAxis(p_grid, room, &(p_corridor->cubes[idx].cube));
        }
        for (const auto& idx : down_idx_list) {
          InflateCubeOnYPosAxis(p_grid, room, &(p_corridor->cubes[idx].cube));
        }
      }
      if (abs(cube_0_lb - cube_1_ub) < margin[1]) {
        int room = margin[1] - abs(cube_0_lb - cube_1_ub);
        // ~ Upward
        std::vector<int> up_idx_list;
        GetTimeCoveredCubeIndices(p_corridor, i + 1, 1, t_trans, &up_idx_list);
        // ~ Downward
        std::vector<int> down_idx_list;
        GetTimeCoveredCubeIndices(p_corridor, i, 0, t_trans, &down_idx_list);
        for (const auto idx : up_idx_list) {
          InflateCubeOnYPosAxis(p_grid, room, &(p_corridor->cubes[idx].cube));
        }
        for (const auto idx : down_idx_list) {
          InflateCubeOnYNegAxis(p_grid, room, &(p_corridor->cubes[idx].cube));
        }
      }
    }
  }
}

void SscMap::InflateObstacleGrid(const game_common::VehicleParam& param) {
  double s_p_inflate_len = param.length() / 2.0 - param.d_cr();
  double s_n_inflate_len = param.length() - s_p_inflate_len;
  int num_s_p_inflate_grids =
      std::floor(s_p_inflate_len / config_.map_resl_x());
  int num_s_n_inflate_grids =
      std::floor(s_n_inflate_len / config_.map_resl_x());
  int num_d_inflate_grids =
      std::floor((param.width() - 0.5) / 2.0 / config_.map_resl_y());
  bool is_free = false;

  for (int i = 0; i < config_.map_size_x(); ++i) {
    for (int j = 0; j < config_.map_size_y(); ++j) {
      for (int k = 0; k < config_.map_size_z(); ++k) {
        std::array<int, 3> coord = {i, j, k};
        p_3d_grid_->CheckIfEqualUsingCoordinate(coord, 0, &is_free);
        if (!is_free) {
          for (int s = -num_s_n_inflate_grids; s < num_s_p_inflate_grids; s++) {
            for (int d = -num_d_inflate_grids; d < num_d_inflate_grids; d++) {
              coord = {i + s, j + d, k};
              p_3d_inflated_grid_->SetValueUsingCoordinate(
                  coord, GridMap3D::ValType::OCCUPIED);
            }
          }
        }
      }
    }
  }
}

void SscMap::InflateCubeIn3dGrid(
    const std::shared_ptr<GridMap3D>& p_grid,
    const std::array<bool, 6>& dir_disabled,
    const SscTrajectoryOptimizerConfig::InflateSteps& inflate_steps,
    AxisAlignedCubeNd<int, 3>* cube) {
  bool x_p_finish = dir_disabled[0];
  bool x_n_finish = dir_disabled[1];
  bool y_p_finish = dir_disabled[2];
  bool y_n_finish = dir_disabled[3];
  bool z_p_finish = dir_disabled[4];

  int x_p_step = inflate_steps.x_p();
  int x_n_step = inflate_steps.x_n();
  int y_p_step = inflate_steps.y_p();
  int y_n_step = inflate_steps.y_n();
  int z_p_step = inflate_steps.z_p();

  int t_max_grids = cube->lower_bound[2] + config_.max_grids_along_time();

  double t = t_max_grids * p_grid->dims_resolution(2);
  double a_max = config_.dyn_bounds().max_lon_acc();
  double a_min = config_.dyn_bounds().max_lon_dec();
  double d_comp = initial_fs_.s_state[1] * 1;

  double s_u = initial_fs_.s_state[0] + initial_fs_.s_state[1] * t +
               0.5 * a_max * t * t + d_comp;
  double s_l = initial_fs_.s_state[0] + initial_fs_.s_state[1] * t +
               0.5 * a_min * t * t - d_comp;

  int s_idx_u = 0;
  int s_idx_l = 0;
  p_grid->GetCoordUsingGlobalMetricOnSingleDim(s_u, 0, &s_idx_u);
  p_grid->GetCoordUsingGlobalMetricOnSingleDim(s_l, 0, &s_idx_l);
  s_idx_l = std::max(s_idx_l, static_cast<int>((config_.s_back_len() / 2.0) /
                                               config_.map_resl_x()));

  while (!(x_p_finish && x_n_finish && y_p_finish && y_n_finish)) {
    if (!x_p_finish) {
      x_p_finish = InflateCubeOnXPosAxis(p_grid, x_p_step, cube);
    }
    if (!x_n_finish) {
      x_n_finish = InflateCubeOnXNegAxis(p_grid, x_n_step, cube);
    }
    if (!y_p_finish) {
      y_p_finish = InflateCubeOnYPosAxis(p_grid, y_p_step, cube);
    }
    if (!y_n_finish) {
      y_n_finish = InflateCubeOnYNegAxis(p_grid, y_n_step, cube);
    }
    if (cube->upper_bound[0] >= s_idx_u) {
      x_p_finish = true;
    }
    if (cube->lower_bound[0] <= s_idx_l) {
      x_n_finish = true;
    }
  }

  // ~ No need to inflate along z-neg
  while (!z_p_finish) {
    if (!z_p_finish) {
      z_p_finish = InflateCubeOnZPosAxis(p_grid, z_p_step, cube);
    }
    if (cube->upper_bound[2] - cube->lower_bound[2] >=
        config_.max_grids_along_time()) {
      z_p_finish = true;
    }
  }
}

bool SscMap::InflateCubeOnXPosAxis(const std::shared_ptr<GridMap3D>& p_grid,
                                   const int& n_step,
                                   AxisAlignedCubeNd<int, 3>* cube) {
  for (int i = 0; i < n_step; ++i) {
    int x = cube->upper_bound[0] + 1;
    if (!p_grid->CheckCoordInRangeOnSingleDim(x, 0)) {
      return true;
    }
    if (CheckIfPlaneIsFreeOnXAxis(p_grid, *cube, x)) {
      // The plane in 3D obstacle grid is free
      cube->upper_bound[0] = x;
    } else {
      // The plane in 3D obstacle grid is not free, finish
      return true;
    }
  }
  return false;
}

bool SscMap::InflateCubeOnXNegAxis(const std::shared_ptr<GridMap3D>& p_grid,
                                   const int& n_step,
                                   AxisAlignedCubeNd<int, 3>* cube) {
  for (int i = 0; i < n_step; ++i) {
    int x = cube->lower_bound[0] - 1;
    if (!p_grid->CheckCoordInRangeOnSingleDim(x, 0)) {
      return true;
    }
    if (CheckIfPlaneIsFreeOnXAxis(p_grid, *cube, x)) {
      // The plane in 3D obstacle grid is free
      cube->lower_bound[0] = x;
    } else {
      return true;
    }
  }
  return false;
}

bool SscMap::InflateCubeOnYPosAxis(const std::shared_ptr<GridMap3D>& p_grid,
                                   const int& n_step,
                                   AxisAlignedCubeNd<int, 3>* cube) {
  for (int i = 0; i < n_step; ++i) {
    int y = cube->upper_bound[1] + 1;
    if (!p_grid->CheckCoordInRangeOnSingleDim(y, 1)) {
      return true;
    }
    if (CheckIfPlaneIsFreeOnYAxis(p_grid, *cube, y)) {
      // The plane in 3D obstacle grid is free
      cube->upper_bound[1] = y;
    } else {
      return true;
    }
  }
  return false;
}

bool SscMap::InflateCubeOnYNegAxis(const std::shared_ptr<GridMap3D>& p_grid,
                                   const int& n_step,
                                   AxisAlignedCubeNd<int, 3>* cube) {
  for (int i = 0; i < n_step; ++i) {
    int y = cube->lower_bound[1] - 1;
    if (!p_grid->CheckCoordInRangeOnSingleDim(y, 1)) {
      return true;
    }
    if (CheckIfPlaneIsFreeOnYAxis(p_grid, *cube, y)) {
      // The plane in 3D obstacle grid is free
      cube->lower_bound[1] = y;
    } else {
      return true;
    }
  }
  return false;
}

bool SscMap::InflateCubeOnZPosAxis(const std::shared_ptr<GridMap3D>& p_grid,
                                   const int& n_step,
                                   AxisAlignedCubeNd<int, 3>* cube) {
  for (int i = 0; i < n_step; ++i) {
    int z = cube->upper_bound[2] + 1;
    if (!p_grid->CheckCoordInRangeOnSingleDim(z, 2)) {
      return true;
    }
    if (CheckIfPlaneIsFreeOnZAxis(p_grid, *cube, z)) {
      // The plane in 3D obstacle grid is free
      cube->upper_bound[2] = z;
    } else {
      return true;
    }
  }
  return false;
}

bool SscMap::InflateCubeOnZNegAxis(const std::shared_ptr<GridMap3D>& p_grid,
                                   const int& n_step,
                                   AxisAlignedCubeNd<int, 3>* cube) {
  for (int i = 0; i < n_step; ++i) {
    int z = cube->lower_bound[2] - 1;
    if (!p_grid->CheckCoordInRangeOnSingleDim(z, 2)) {
      return true;
    }
    if (CheckIfPlaneIsFreeOnZAxis(p_grid, *cube, z)) {
      // The plane in 3D obstacle grid is free
      cube->lower_bound[2] = z;
    } else {
      return true;
    }
  }
  return false;
}

bool SscMap::CheckIfCubeIsFree(const std::shared_ptr<GridMap3D>& p_grid,
                               const AxisAlignedCubeNd<int, 3>& cube) {
  int f0_min = cube.lower_bound[0];
  int f0_max = cube.upper_bound[0];
  int f1_min = cube.lower_bound[1];
  int f1_max = cube.upper_bound[1];
  int f2_min = cube.lower_bound[2];
  int f2_max = cube.upper_bound[2];

  int i = 0;
  int j = 0;
  int k = 0;
  std::array<int, 3> coord{};
  bool is_free = false;
  for (i = f0_min; i <= f0_max; ++i) {
    for (j = f1_min; j <= f1_max; ++j) {
      for (k = f2_min; k <= f2_max; ++k) {
        coord = {i, j, k};
        p_grid->CheckIfEqualUsingCoordinate(coord, 0, &is_free);
        if (!is_free) {
          return false;
        }
      }
    }
  }
  return true;
}

bool SscMap::CheckIfPlaneIsFreeOnXAxis(const std::shared_ptr<GridMap3D>& p_grid,
                                       const AxisAlignedCubeNd<int, 3>& cube,
                                       const int& x) {
  int f0_min = cube.lower_bound[1];
  int f0_max = cube.upper_bound[1];
  int f1_min = cube.lower_bound[2];
  int f1_max = cube.upper_bound[2];
  std::array<int, 3> coord{};
  bool is_free = false;
  for (int i = f0_min; i <= f0_max; ++i) {
    for (int j = f1_min; j <= f1_max; ++j) {
      coord = {x, i, j};
      p_grid->CheckIfEqualUsingCoordinate(coord, 0, &is_free);
      if (!is_free) {
        return false;
      }
    }
  }
  return true;
}

bool SscMap::CheckIfPlaneIsFreeOnYAxis(const std::shared_ptr<GridMap3D>& p_grid,
                                       const AxisAlignedCubeNd<int, 3>& cube,
                                       const int& y) {
  int f0_min = cube.lower_bound[0];
  int f0_max = cube.upper_bound[0];
  int f1_min = cube.lower_bound[2];
  int f1_max = cube.upper_bound[2];
  std::array<int, 3> coord{};
  bool is_free = false;
  for (int i = f0_min; i <= f0_max; ++i) {
    for (int j = f1_min; j <= f1_max; ++j) {
      coord = {i, y, j};
      p_grid->CheckIfEqualUsingCoordinate(coord, 0, &is_free);
      if (!is_free) {
        return false;
      }
    }
  }
  return true;
}

bool SscMap::CheckIfPlaneIsFreeOnZAxis(const std::shared_ptr<GridMap3D>& p_grid,
                                       const AxisAlignedCubeNd<int, 3>& cube,
                                       const int& z) {
  int f0_min = cube.lower_bound[0];
  int f0_max = cube.upper_bound[0];
  int f1_min = cube.lower_bound[1];
  int f1_max = cube.upper_bound[1];
  std::array<int, 3> coord{};
  bool is_free = false;
  for (int i = f0_min; i <= f0_max; ++i) {
    for (int j = f1_min; j <= f1_max; ++j) {
      coord = {i, j, z};
      p_grid->CheckIfEqualUsingCoordinate(coord, 0, &is_free);
      if (!is_free) {
        return false;
      }
    }
  }
  return true;
}

bool SscMap::CheckIfCubeContainsSeed(const AxisAlignedCubeNd<int, 3>& cube_a,
                                     const std::array<int, 3>& seed) {
  for (int i = 0; i < 3; ++i) {
    if (cube_a.lower_bound.at(i) > seed.at(i) ||
        cube_a.upper_bound.at(i) < seed.at(i)) {
      return false;
    }
  }
  return true;
}

bool SscMap::GetFinalGlobalMetricCubesList() {
  final_corridor_vec_.clear();
  if_corridor_valid_.clear();
  for (const auto& corridor : driving_corridor_vec_) {
    std::vector<SpatioTemporalSemanticCubeNd<2>> cubes;
    if (!corridor.is_valid) {
      if_corridor_valid_.push_back(0);
    } else {
      if_corridor_valid_.push_back(1);
      for (int k = 0; k < static_cast<int>(corridor.cubes.size()); ++k) {
        SpatioTemporalSemanticCubeNd<2> cube;
        double x_lb = 0.0;
        double x_ub = 0.0;
        double y_lb = 0.0;
        double y_ub = 0.0;
        double z_lb = 0.0;
        double z_ub = 0.0;

        p_3d_grid_->GetGlobalMetricUsingCoordOnSingleDim(
            corridor.cubes[k].cube.lower_bound[0], 0, &x_lb);
        p_3d_grid_->GetGlobalMetricUsingCoordOnSingleDim(
            corridor.cubes[k].cube.upper_bound[0], 0, &x_ub);
        p_3d_grid_->GetGlobalMetricUsingCoordOnSingleDim(
            corridor.cubes[k].cube.lower_bound[1], 1, &y_lb);
        p_3d_grid_->GetGlobalMetricUsingCoordOnSingleDim(
            corridor.cubes[k].cube.upper_bound[1], 1, &y_ub);
        p_3d_grid_->GetGlobalMetricUsingCoordOnSingleDim(
            corridor.cubes[k].cube.lower_bound[2], 2, &z_lb);
        p_3d_grid_->GetGlobalMetricUsingCoordOnSingleDim(
            corridor.cubes[k].cube.upper_bound[2], 2, &z_ub);

        cube.t_lb = z_lb;
        cube.t_ub = z_ub;

        cube.p_lb[0] = x_lb;
        cube.p_ub[0] = x_ub;
        cube.v_lb[0] = config_.dyn_bounds().min_lon_vel();
        cube.v_ub[0] = config_.dyn_bounds().max_lon_vel();
        cube.a_lb[0] = config_.dyn_bounds().max_lon_dec();
        cube.a_ub[0] = config_.dyn_bounds().max_lon_acc();

        cube.p_lb[1] = y_lb;
        cube.p_ub[1] = y_ub;
        cube.v_lb[1] = -config_.dyn_bounds().max_lat_vel();
        cube.v_ub[1] = config_.dyn_bounds().max_lat_vel();
        cube.a_lb[1] = -config_.dyn_bounds().max_lat_acc();
        cube.a_ub[1] = config_.dyn_bounds().max_lat_acc();

        if (k == 0) {
          if (y_lb > initial_fs_.l_state[0] || y_ub < initial_fs_.l_state[0]) {
            AERROR << "[Ssc] SscMap - Initial state out of bound d: "
                   << initial_fs_.l_state[0] << ", lb: " << y_lb
                   << ", ub: " << y_ub;
            // assert(false);
            return false;
          }
        }

        cubes.push_back(cube);
      }
    }
    final_corridor_vec_.push_back(cubes);
  }

  return true;
}

void SscMap::FillStaticPart(
    const std::vector<std::array<double, 3>>& obs_grid_fs) {
  for (const auto& obs_grid_f : obs_grid_fs) {
    if (obs_grid_f[0] <= 0) {
      continue;
    }
    for (int k = 0; k < config_.map_size_z(); ++k) {
      std::array<double, 3> pt = {
          {obs_grid_f[0], obs_grid_f[1], k * config_.map_resl_z()}};
      auto coord = p_3d_grid_->GetCoordUsingGlobalPosition(pt);
      if (p_3d_grid_->CheckCoordInRange(coord)) {
        p_3d_grid_->SetValueUsingCoordinate(coord,
                                            GridMap3D::ValType::OCCUPIED);
      }
    }
  }
}

void SscMap::FillDynamicPart(
    const std::unordered_map<int, std::vector<FsVehicle>>&
        sur_vehicle_trajs_fs) {
  for (const auto& sur_vehicle_trajs_f : sur_vehicle_trajs_fs) {
    FillMapWithFsVehicleTraj(sur_vehicle_trajs_f.second);
  }
}

bool SscMap::FillMapWithFsVehicleTraj(const std::vector<FsVehicle>& traj) {
  if (traj.empty()) {
    AERROR << "[Ssc] SscMap - Trajectory is empty.";
    return false;
  }
  for (const auto& pt : traj) {
    bool is_valid = true;
    for (const auto v : pt.vertices) {
      if (v.x() <= 0) {
        is_valid = false;
        break;
      }
    }
    if (!is_valid) {
      continue;
    }
    double z = pt.frenet_state.time_stamp;
    int t_idx = 0;
    std::vector<cv::Point2i> v_coord_cv;
    std::array<double, 3> p_w{};
    for (const auto v : pt.vertices) {
      p_w = {v.x(), v.y(), z};
      auto coord = p_3d_grid_->GetCoordUsingGlobalPosition(p_w);
      t_idx = coord[2];
      if (!p_3d_grid_->CheckCoordInRange(coord)) {
        is_valid = false;
        break;
      }
      v_coord_cv.emplace_back(coord[0], coord[1]);
    }
    if (!is_valid) {
      continue;
    }

    std::vector<std::vector<cv::Point2i>> vv_coord_cv;
    vv_coord_cv.push_back(v_coord_cv);
    int w = p_3d_grid_->dims_size()[0];
    int h = p_3d_grid_->dims_size()[1];
    int layer_offset = t_idx * w * h;
    auto layer_mat =
        cv::Mat(h, w, CV_8UC1, p_3d_grid_->get_data_ptr() + layer_offset);
    cv::fillPoly(layer_mat, vv_coord_cv, GridMap3D::ValType::OCCUPIED);
  }
  return true;
}

}  // namespace TL::planning
