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

#include "planning/open_space/coarse_path_generator/node3d.h"

#include "absl/strings/str_cat.h"
#include "common/math/math_utils.h"
#include "glog/logging.h"

namespace TL {
namespace planning {

using TL::common::math::Box2d;

Node3d::Node3d(const double x, const double y, const double phi)
    : x_(x), y_(y), phi_(phi) {
  traversed_x_.push_back(x);
  traversed_y_.push_back(y);
  traversed_phi_.push_back(phi);
}

Node3d::Node3d(double x, double y, double phi,
               const std::vector<double>& xy_bounds,
               const double xy_grid_resolution,
               const double phi_grid_resolution) {
  CHECK_EQ(xy_bounds.size(), 4U)
      << "xy_bounds size is not 4, but" << xy_bounds.size();

  x_ = x;
  y_ = y;
  phi_ = phi;

  x_grid_ = static_cast<int>((x_ - xy_bounds[0]) / xy_grid_resolution);
  y_grid_ = static_cast<int>((y_ - xy_bounds[2]) / xy_grid_resolution);
  phi_grid_ = static_cast<int>(common::math::WrapAngle(phi_ - (-M_PI)) /
                               phi_grid_resolution);

  traversed_x_.push_back(x);
  traversed_y_.push_back(y);
  traversed_phi_.push_back(phi);

  index_ = ComputeStringIndex(x_grid_, y_grid_, phi_grid_);
}

Node3d::Node3d(const std::vector<double>& traversed_x,
               const std::vector<double>& traversed_y,
               const std::vector<double>& traversed_phi,
               const std::vector<double>& xy_bounds,
               const double xy_grid_resolution,
               const double phi_grid_resolution) {
  CHECK_EQ(xy_bounds.size(), 4U)
      << "xy_bounds size is not 4, but" << xy_bounds.size();
  CHECK_EQ(traversed_x.size(), traversed_y.size());
  CHECK_EQ(traversed_x.size(), traversed_phi.size());

  x_ = traversed_x.back();
  y_ = traversed_y.back();
  phi_ = traversed_phi.back();

  x_grid_ = static_cast<int>((x_ - xy_bounds[0]) / xy_grid_resolution);
  y_grid_ = static_cast<int>((y_ - xy_bounds[2]) / xy_grid_resolution);
  phi_grid_ = static_cast<int>(common::math::WrapAngle(phi_ - (-M_PI)) /
                               phi_grid_resolution);
  traversed_x_ = traversed_x;
  traversed_y_ = traversed_y;
  traversed_phi_ = traversed_phi;

  index_ = ComputeStringIndex(x_grid_, y_grid_, phi_grid_);
  step_size_ = traversed_x.size();
}

bool Node3d::operator==(const Node3d& right) const {
  return right.GetIndex() == index_;
}

std::string Node3d::ComputeStringIndex(int x_grid, int y_grid, int phi_grid) {
  return absl::StrCat(x_grid, "_", y_grid, "_", phi_grid);
}

}  // namespace planning
}  // namespace TL
