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

#include "planning/open_space/coarse_path_generator/shape/grid_search.h"
#include "common/file/log.h"

namespace TL {
namespace planning {

GridSearch::GridSearch(const WarmStartConfig& config)
    : xy_grid_resolution_(config.grid_a_star_xy_resolution()),
      node_radius_(config.node_radius()) {}

double GridSearch::EuclidDistance(const double x1, const double y1,
                                  const double x2, const double y2) {
  return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

bool GridSearch::CheckConstraints(
    const std::shared_ptr<Node2d>& node,
    const std::vector<std::pair<common::math::LineSegment2d, double>>&
        obstacles_segments_vec) const {
  const double node_grid_x = node->GetGridX();
  const double node_grid_y = node->GetGridY();
  if (node_grid_x > max_grid_x_ || node_grid_x < 0 ||
      node_grid_y > max_grid_y_ || node_grid_y < 0) {
    return false;
  }
  if (obstacles_segments_vec.empty()) {
    return true;
  }
  for (const auto& obstacle_linesegment : obstacles_segments_vec) {
    if (obstacle_linesegment.first.DistanceTo({node->GetX(), node->GetY()}) <
        node_radius_) {
      return false;
    }
  }
  return true;
}

std::vector<std::shared_ptr<Node2d>> GridSearch::GenerateNextNodes(
    const std::shared_ptr<Node2d>& current_node) {
  double current_node_x = current_node->GetX();
  double current_node_y = current_node->GetY();
  double current_node_path_cost = current_node->GetPathCost();
  static constexpr double kTwo = 2.0;
  double diagonal_distance = std::sqrt(kTwo);  // /需要更改对角线代价值
  std::vector<std::shared_ptr<Node2d>> next_nodes;
  std::shared_ptr<Node2d> up = std::make_shared<Node2d>(
      current_node_x, current_node_y + xy_grid_resolution_, xy_grid_resolution_,
      xy_bounds_);
  up->SetPathCost(current_node_path_cost + 1.0);
  std::shared_ptr<Node2d> up_right = std::make_shared<Node2d>(
      current_node_x + xy_grid_resolution_,
      current_node_y + xy_grid_resolution_, xy_grid_resolution_, xy_bounds_);
  up_right->SetPathCost(current_node_path_cost + diagonal_distance);
  std::shared_ptr<Node2d> right =
      std::make_shared<Node2d>(current_node_x + xy_grid_resolution_,
                               current_node_y, xy_grid_resolution_, xy_bounds_);
  right->SetPathCost(current_node_path_cost + 1.0);
  std::shared_ptr<Node2d> down_right = std::make_shared<Node2d>(
      current_node_x + xy_grid_resolution_,
      current_node_y - xy_grid_resolution_, xy_grid_resolution_, xy_bounds_);
  down_right->SetPathCost(current_node_path_cost + diagonal_distance);
  std::shared_ptr<Node2d> down = std::make_shared<Node2d>(
      current_node_x, current_node_y - xy_grid_resolution_, xy_grid_resolution_,
      xy_bounds_);
  down->SetPathCost(current_node_path_cost + 1.0);
  std::shared_ptr<Node2d> down_left = std::make_shared<Node2d>(
      current_node_x - xy_grid_resolution_,
      current_node_y - xy_grid_resolution_, xy_grid_resolution_, xy_bounds_);
  down_left->SetPathCost(current_node_path_cost + diagonal_distance);
  std::shared_ptr<Node2d> left =
      std::make_shared<Node2d>(current_node_x - xy_grid_resolution_,
                               current_node_y, xy_grid_resolution_, xy_bounds_);
  left->SetPathCost(current_node_path_cost + 1.0);
  std::shared_ptr<Node2d> up_left = std::make_shared<Node2d>(
      current_node_x - xy_grid_resolution_,
      current_node_y + xy_grid_resolution_, xy_grid_resolution_, xy_bounds_);
  up_left->SetPathCost(current_node_path_cost + diagonal_distance);

  next_nodes.emplace_back(up);
  next_nodes.emplace_back(up_right);
  next_nodes.emplace_back(right);
  next_nodes.emplace_back(down_right);
  next_nodes.emplace_back(down);
  next_nodes.emplace_back(down_left);
  next_nodes.emplace_back(left);
  next_nodes.emplace_back(up_left);
  return next_nodes;
}

bool GridSearch::GenerateAStarPath(
    const double sx, const double sy, const double ex, const double ey,
    const std::vector<double>& xy_bounds,
    const std::vector<std::pair<common::math::LineSegment2d, double>>&
        obstacles_segments_vec,
    GridAStartResult* result) {
  std::priority_queue<std::pair<std::string, double>,
                      std::vector<std::pair<std::string, double>>, cmp>
      open_pq;
  std::unordered_map<std::string, std::shared_ptr<Node2d>> open_set;
  std::unordered_map<std::string, std::shared_ptr<Node2d>> close_set;
  xy_bounds_ = xy_bounds;
  std::shared_ptr<Node2d> start_node =
      std::make_shared<Node2d>(sx, sy, xy_grid_resolution_, xy_bounds_);
  std::shared_ptr<Node2d> end_node =
      std::make_shared<Node2d>(ex, ey, xy_grid_resolution_, xy_bounds_);
  final_node_ = nullptr;
  open_set.emplace(start_node->GetIndex(), start_node);
  open_pq.emplace(start_node->GetIndex(), start_node->GetCost());

  // Grid a star begins
  size_t explored_node_num = 0;
  while (!open_pq.empty()) {
    std::string current_id = open_pq.top().first;
    open_pq.pop();
    std::shared_ptr<Node2d> current_node = open_set[current_id];
    // Check destination
    if (*(current_node) == *(end_node)) {
      final_node_ = current_node;
      break;
    }
    close_set.emplace(current_node->GetIndex(), current_node);
    std::vector<std::shared_ptr<Node2d>> next_nodes =
        GenerateNextNodes(current_node);
    for (auto& next_node : next_nodes) {
      if (!CheckConstraints(next_node, obstacles_segments_vec)) {
        continue;
      }
      if (close_set.find(next_node->GetIndex()) != close_set.end()) {
        continue;
      }
      if (open_set.find(next_node->GetIndex()) == open_set.end()) {
        ++explored_node_num;
        next_node->SetHeuristic(
            EuclidDistance(next_node->GetGridX(), next_node->GetGridY(),
                           end_node->GetGridX(), end_node->GetGridY()));
        next_node->SetPreNode(current_node);
        open_set.emplace(next_node->GetIndex(), next_node);
        open_pq.emplace(next_node->GetIndex(), next_node->GetCost());
      }
    }
  }

  if (final_node_ == nullptr) {
    AERROR << "Grid A searching return null ptr(open_set ran out)";
    return false;
  }
  LoadGridAStarResult(result);
  ADEBUG << "explored node num is " << explored_node_num;
  return true;
}

bool GridSearch::GenerateDpMap(
    const double ex, const double ey, const std::vector<double>& xy_bounds,
    const std::vector<std::pair<common::math::LineSegment2d, double>>&
        obstacles_segments_vec) {
  std::priority_queue<std::pair<std::string, double>,
                      std::vector<std::pair<std::string, double>>, cmp>
      open_pq;
  std::unordered_map<std::string, std::shared_ptr<Node2d>> open_set;
  dp_map_ = decltype(dp_map_)();
  xy_bounds_ = xy_bounds;

  // xy_bounds with xmin, xmax, ymin, ymax
  max_grid_y_ =
      std::round((xy_bounds_[3] - xy_bounds_[2]) / xy_grid_resolution_);
  max_grid_x_ =
      std::round((xy_bounds_[1] - xy_bounds_[0]) / xy_grid_resolution_);
  static constexpr int kNum = 3;
  static constexpr int kTimes = 10;
  static constexpr double kHalf = 0.5;
  auto grid_round = [](const double x) {
    return floor(x * pow(kTimes, kNum) + kHalf) / pow(kTimes, kNum) +
           pow(kTimes, -1 - kNum);
  };
  std::shared_ptr<Node2d> end_node = std::make_shared<Node2d>(
      grid_round(ex), grid_round(ey), xy_grid_resolution_, xy_bounds_);
  open_set.emplace(end_node->GetIndex(), end_node);
  open_pq.emplace(end_node->GetIndex(), end_node->GetCost());

  // Grid a star begins
  size_t explored_node_num = 0;
  while (!open_pq.empty()) {
    const std::string current_id = open_pq.top().first;
    open_pq.pop();
    std::shared_ptr<Node2d> current_node = open_set[current_id];

    dp_map_.emplace(current_node->GetIndex(), current_node);
    std::vector<std::shared_ptr<Node2d>> next_nodes =
        GenerateNextNodes(current_node);
    for (auto& next_node : next_nodes) {
      if (!CheckConstraints(next_node, obstacles_segments_vec)) {
        continue;
      }

      if (dp_map_.find(next_node->GetIndex()) != dp_map_.end()) {
        continue;
      }
      if (open_set.find(next_node->GetIndex()) == open_set.end()) {
        ++explored_node_num;
        next_node->SetPreNode(current_node);
        open_set.emplace(next_node->GetIndex(), next_node);
        open_pq.emplace(next_node->GetIndex(), next_node->GetCost());
      } else {
        if (open_set[next_node->GetIndex()]->GetCost() > next_node->GetCost()) {
          open_set[next_node->GetIndex()]->SetCost(next_node->GetCost());
          open_set[next_node->GetIndex()]->SetPreNode(current_node);
        }
      }
    }
  }
  ADEBUG << "explored node num is " << explored_node_num;
  return true;
}

double GridSearch::CheckDpMap(const double sx, const double sy) {
  std::string index =
      Node2d::CalcIndex(sx, sy, xy_grid_resolution_, xy_bounds_);
  if (dp_map_.find(index) != dp_map_.end()) {
    return dp_map_[index]->GetCost() * xy_grid_resolution_;
  }
  return std::numeric_limits<double>::infinity();
}

void GridSearch::LoadGridAStarResult(GridAStartResult* result) {
  (*result).path_cost = final_node_->GetPathCost() * xy_grid_resolution_;
  std::shared_ptr<Node2d> current_node = final_node_;
  std::vector<double> grid_a_x;
  std::vector<double> grid_a_y;
  while (current_node->GetPreNode() != nullptr) {
    grid_a_x.push_back(current_node->GetGridX() * xy_grid_resolution_ +
                       xy_bounds_[0]);
    grid_a_y.push_back(current_node->GetGridY() * xy_grid_resolution_ +
                       xy_bounds_[2]);
    current_node = current_node->GetPreNode();
  }
  std::reverse(grid_a_x.begin(), grid_a_x.end());
  std::reverse(grid_a_y.begin(), grid_a_y.end());
  (*result).x = std::move(grid_a_x);
  (*result).y = std::move(grid_a_y);
}

void GridSearch::SetXYGridResolution(const double xy_grid_resolution) {
  xy_grid_resolution_ = xy_grid_resolution;
}

}  // namespace planning
}  // namespace TL
