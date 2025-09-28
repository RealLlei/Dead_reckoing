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

/*
 * @file
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "common/math/math_utils.h"
#include "planning/open_space/coarse_path_generator/node3d.h"

#include "planning/proto/planner_open_space_config.pb.h"
#include "proto/common/vehicle_config.pb.h"

namespace TL {
namespace planning {
// NOLINTBEGIN
struct ReedSheppPath {
  std::vector<double> segs_lengths;
  std::vector<char> segs_types;
  double total_length = 0.0;
  double shortest_path_segment_length = 0.0;
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> phi;
  // true for driving forward and false for driving backward
  std::vector<bool> gear;
};

struct RSPParam {
  bool flag = false;
  double t = 0.0;
  double u = 0.0;
  double v = 0.0;
};

class ReedShepp {
 public:
  ReedShepp(const common::VehicleParam& vehicle_param,
            const WarmStartConfig& warm_start_config);
  virtual ~ReedShepp() = default;

  /** 
   * @brief  Pick the shortest path from all possible combination 
   *        of movement primitives by Reed Shepp
   *
   * @param start_node 
   * @param end_node 
   * @param optimal_path 
   * @return true 
   * @return false 
   */
  bool ShortestRSP(const std::shared_ptr<Node3d>& start_node,
                   const std::shared_ptr<Node3d>& end_node,
                   std::shared_ptr<ReedSheppPath> optimal_path);

  /**
   * @brief update reeds sheep config
   * 
   * @param max_beta_angle
   */
  void UpdateConfig(double max_beta_angle) {
    max_kappa_ = fabs(std::tan(max_beta_angle) / vehicle_param_.wheel_base());
  }

 protected:
  /**
   * @brief Generate all possible combination of movement primitives
   *        by Reed Shepp and interpolate them
   *
   * @param start_node 
   * @param end_node 
   * @param all_possible_paths 
   * @return true 
   * @return false 
   */
  bool GenerateRSPs(const std::shared_ptr<Node3d>& start_node,
                    const std::shared_ptr<Node3d>& end_node,
                    std::vector<ReedSheppPath>* all_possible_paths);

  /**
   * @brief Set the general profile of the movement primitives
   * 
   * @param start_node 
   * @param end_node 
   * @param all_possible_paths 
   * @return true 
   * @return false 
   */
  bool GenerateRSP(const std::shared_ptr<Node3d>& start_node,
                   const std::shared_ptr<Node3d>& end_node,
                   std::vector<ReedSheppPath>* all_possible_paths);
  /**
   * @brief Set the general profile of the movement primitives, parallel implementation
   * 
   * @param start_node 
   * @param end_node 
   * @param all_possible_paths 
   * @return true 
   * @return false 
   */
  bool GenerateRSPPar(const std::shared_ptr<Node3d>& start_node,
                      const std::shared_ptr<Node3d>& end_node,
                      std::vector<ReedSheppPath>* all_possible_paths);

  /**
   * @brief Set local exact configurations profile of each movement primitive
   * 
   * @param start_node 
   * @param end_node 
   * @param shortest_path 
   * @return true 
   * @return false 
   */
  bool GenerateLocalConfigurations(const std::shared_ptr<Node3d>& start_node,
                                   const std::shared_ptr<Node3d>& end_node,
                                   ReedSheppPath* shortest_path);

  /**
   * @brief Interpolation usde in GenetateLocalConfiguration
   * 
   * @param index path point index
   * @param pd path segment length
   * @param m path segment type
   * @param ox origin path point x 
   * @param oy origin path point y
   * @param ophi origin path point phi
   * @param px path point x
   * @param py path point y
   * @param pphi path point phi
   * @param pgear path point gear
   */
  void Interpolation(int index, double pd, char m, double ox, double oy,
                     double ophi, std::vector<double>* px,
                     std::vector<double>* py, std::vector<double>* pphi,
                     std::vector<bool>* pgear);

  /**
   * @brief motion primitives combination setup function
   * 
   * @param size 
   * @param lengths 
   * @param types 
   * @param all_possible_paths 
   * @return true 
   * @return false 
   */
  bool SetRSP(int size, const double* lengths, const char* types,
              std::vector<ReedSheppPath>* all_possible_paths);

  /**
   * @brief setRSP parallel version
   * 
   * @param size 
   * @param lengths 
   * @param types 
   * @param all_possible_paths 
   * @param idx 
   * @return true 
   * @return false 
   */
  bool SetRSPPar(int size, const std::array<double, 5>& lengths,
                 const std::string& types,
                 std::vector<ReedSheppPath>* all_possible_paths, int idx);
  // Six different combination of motion primitive in Reed Shepp path used in
  // GenerateRSP()
  bool SCS(double x, double y, double phi,
           std::vector<ReedSheppPath>* all_possible_paths);
  bool CSC(double x, double y, double phi,
           std::vector<ReedSheppPath>* all_possible_paths);
  bool CCC(double x, double y, double phi,
           std::vector<ReedSheppPath>* all_possible_paths);
  bool CCCC(double x, double y, double phi,
            std::vector<ReedSheppPath>* all_possible_paths);
  bool CCSC(double x, double y, double phi,
            std::vector<ReedSheppPath>* all_possible_paths);
  bool CCSCC(double x, double y, double phi,
             std::vector<ReedSheppPath>* all_possible_paths);
  // different options for different combination of motion primitives
  void LSL(double x, double y, double phi, RSPParam* param);
  void LSR(double x, double y, double phi, RSPParam* param);
  void LRL(double x, double y, double phi, RSPParam* param);
  void SLS(double x, double y, double phi, RSPParam* param);
  void LRLRn(double x, double y, double phi, RSPParam* param);
  void LRLRp(double x, double y, double phi, RSPParam* param);
  void LRSR(double x, double y, double phi, RSPParam* param);
  void LRSL(double x, double y, double phi, RSPParam* param);
  void LRSLR(double x, double y, double phi, RSPParam* param);
  std::pair<double, double> calc_tau_omega(double u, double v, double xi,
                                           double eta, double phi);

 private:
  common::VehicleParam vehicle_param_;
  const WarmStartConfig warm_start_config_;
  double max_kappa_ = 0.0;
};

// NOLINTEND
}  // namespace planning
}  // namespace TL
