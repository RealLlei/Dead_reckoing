/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include <iostream>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "absl/strings/str_cat.h"
#include "common/math/line_segment2d.h"

#include "planning/proto/planner_open_space_config.pb.h"

namespace TL {
namespace planning {

class Node2d {
 public:
  Node2d(double x, double y, double xy_resolution,
         const std::vector<double>& xy_bounds)
      : x_(x),
        y_(y),
        grid_x_(static_cast<int>((x - xy_bounds[0]) / xy_resolution)),
        grid_y_(static_cast<int>((y - xy_bounds[2]) / xy_resolution)) {
    // xy_bounds with xmin, xmax, ymin, ymax
    index_ = ComputeStringIndex(grid_x_, grid_y_);
  }

  void SetPathCost(const double path_cost) {
    path_cost_ = path_cost;
    cost_ = path_cost_ + heuristic_;
  }

  void SetHeuristic(const double heuristic) {
    heuristic_ = heuristic;
    cost_ = path_cost_ + heuristic_;
  }

  void SetCost(const double cost) { cost_ = cost; }

  void SetPreNode(const std::shared_ptr<Node2d>& pre_node) {
    pre_node_ = pre_node;
  }

  double GetX() const { return x_; }

  double GetY() const { return y_; }

  double GetGridX() const { return grid_x_; }

  double GetGridY() const { return grid_y_; }

  double GetPathCost() const { return path_cost_; }

  double GetHeuCost() const { return heuristic_; }

  double GetCost() const { return cost_; }

  const std::string& GetIndex() const { return index_; }

  std::shared_ptr<Node2d> GetPreNode() const { return pre_node_; }

  /**
   * @brief cacuate coordinate index in grid map
   * 
   * @param x 
   * @param y 
   * @param xy_resolution 
   * @param xy_bounds 
   * @return std::string 
   */
  static std::string CalcIndex(double x, double y, double xy_resolution,
                               const std::vector<double>& xy_bounds) {
    // xy_bounds with xmin, xmax, ymin, ymax
    int grid_x = static_cast<int>((x - xy_bounds[0]) / xy_resolution);
    int grid_y = static_cast<int>((y - xy_bounds[2]) / xy_resolution);
    return ComputeStringIndex(grid_x, grid_y);
  }

  bool operator==(const Node2d& right) const {
    return right.GetIndex() == index_;
  }

 private:
  static std::string ComputeStringIndex(int x_grid, int y_grid) {
    return absl::StrCat(x_grid, "_", y_grid);
  }

  double x_ = 0.0;
  double y_ = 0.0;
  int grid_x_ = 0;
  int grid_y_ = 0;
  double path_cost_ = 0.0;
  double heuristic_ = 0.0;
  double cost_ = 0.0;
  std::string index_;
  std::shared_ptr<Node2d> pre_node_ = nullptr;
};

struct GridAStartResult {
  std::vector<double> x;
  std::vector<double> y;
  double path_cost = 0.0;
};

class GridSearch {
 public:
  explicit GridSearch(const WarmStartConfig& config);
  virtual ~GridSearch() = default;
  /**
   * @brief Generate A star path in grid
   * 
   * @param sx start x coordinate
   * @param sy start y coordinate
   * @param ex end x coordiante
   * @param ey end y coordinate
   * @param xy_bounds xy bound limit
   * @param obstacles_segments_vec obstacle which represent as segments
   * @param result a star path result
   * @return true 
   * @return false 
   */
  bool GenerateAStarPath(
      double sx, double sy, double ex, double ey,
      const std::vector<double>& xy_bounds,
      const std::vector<std::pair<common::math::LineSegment2d, double>>&
          obstacles_segments_vec,
      GridAStartResult* result);

  /**
   * @brief Generate node cost to end node, which acclerate heuristic caculate
   * 
   * @param ex 
   * @param ey 
   * @param xy_bounds 
   * @param obstacles_segments_vec 
   * @return true 
   * @return false 
   */
  bool GenerateDpMap(
      double ex, double ey, const std::vector<double>& xy_bounds,
      const std::vector<std::pair<common::math::LineSegment2d, double>>&
          obstacles_segments_vec);

  /**
   * @brief get node cost in Dp map
   * 
   * @param sx 
   * @param sy 
   * @return double 
   */
  double CheckDpMap(double sx, double sy);

  /**
   * @brief set grid map xy resolution
   * 
   * @param xy_grid_resolution 
   */
  void SetXYGridResolution(double xy_grid_resolution);

 private:
  /**
   * @brief caculate euclid distance
   * 
   * @param x1 
   * @param y1 
   * @param x2 
   * @param y2 
   * @return double 
   */
  double EuclidDistance(double x1, double y1, double x2, double y2);

  /**
   * @brief Genearate next node 
   * 
   * @param node currnet node
   * @return std::vector<std::shared_ptr<Node2d>> 
   */
  std::vector<std::shared_ptr<Node2d>> GenerateNextNodes(
      const std::shared_ptr<Node2d>& node);

  /**
   * @brief Check node safety contrain
   * 
   * @param node 
   * @param obstacles_segments_vec 
   * @return true 
   * @return false 
   */
  bool CheckConstraints(
      const std::shared_ptr<Node2d>& node,
      const std::vector<std::pair<common::math::LineSegment2d, double>>&
          obstacles_segments_vec) const;

  /**
   * @brief Load A star result in grid map
   * 
   * @param result 
   */
  void LoadGridAStarResult(GridAStartResult* result);

  double xy_grid_resolution_ = 0.0;
  double node_radius_ = 0.0;
  std::vector<double> xy_bounds_;
  double max_grid_x_ = 0.0;
  double max_grid_y_ = 0.0;
  std::shared_ptr<Node2d> start_node_;
  std::shared_ptr<Node2d> end_node_;
  std::shared_ptr<Node2d> final_node_;

  struct cmp {
    bool operator()(const std::pair<std::string, double>& left,
                    const std::pair<std::string, double>& right) const {
      return left.second >= right.second;
    }
  };

  std::unordered_map<std::string, std::shared_ptr<Node2d>> dp_map_;
};
}  // namespace planning
}  // namespace TL
