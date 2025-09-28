/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

/**
 * @file path_data.cc
 **/

#include "planning/common/path/path_data.h"

#include <algorithm>

#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "common/math/cartesian_frenet_conversion.h"
#include "common/math/math_utils.h"
#include "common/util/point_factory.h"
#include "common/util/string_util.h"
#include "planning/common/path/discretized_path.h"
#include "planning/common/planning_gflags.h"
#include "proto/common/pnc_point.pb.h"

namespace TL {
namespace planning {

using TL::common::PathPoint;
using TL::common::SLPoint;
using TL::common::math::CartesianFrenetConverter;
using TL::common::util::PointFactory;

bool PathData::SetDiscretizedPath(DiscretizedPath path) {
  if (reference_line_ == nullptr) {
    AERROR << "Should NOT set discretized path when reference line is nullptr. "
              "Please set reference line first.";
    return false;
  }
  discretized_path_ = std::move(path);
  if (!XYToSL(discretized_path_, &frenet_path_)) {
    AERROR << "Fail to transfer discretized path to frenet path.";
    return false;
  }
  DCHECK_EQ(discretized_path_.size(), frenet_path_.size());
  return true;
}

bool PathData::SetFrenetPath(FrenetFramePath frenet_path) {
  if (reference_line_ == nullptr) {
    AERROR << "Should NOT set frenet path when reference line is nullptr. "
              "Please set reference line first.";
    return false;
  }
  frenet_path_ = std::move(frenet_path);
  if (!SLToXY(frenet_path_, &discretized_path_)) {
    AERROR << "Fail to transfer frenet path to discretized path.";
    return false;
  }
  DCHECK_EQ(discretized_path_.size(), frenet_path_.size());
  return true;
}

bool PathData::SetPathPointDecisionGuide(
    std::vector<std::tuple<double, PathPointType, double>>
        path_point_decision_guide) {
  if (reference_line_ == nullptr) {
    AERROR << "Should NOT set path_point_decision_guide when reference line is "
              "nullptr. ";
    return false;
  }
  if (frenet_path_.empty() || discretized_path_.empty()) {
    AERROR << "Should NOT set path_point_decision_guide when frenet_path or "
              "world frame trajectory is empty. ";
    return false;
  }
  path_point_decision_guide_ = std::move(path_point_decision_guide);
  return true;
}

const DiscretizedPath& PathData::discretized_path() const {
  return discretized_path_;
}

const FrenetFramePath& PathData::frenet_frame_path() const {
  return frenet_path_;
}

const std::vector<std::tuple<double, PathData::PathPointType, double>>&
PathData::path_point_decision_guide() const {
  return path_point_decision_guide_;
}

bool PathData::Empty() const {
  return discretized_path_.empty() && frenet_path_.empty();
}

void PathData::SetReferenceLine(const ReferenceLine* reference_line) {
  Clear();
  reference_line_ = reference_line;
}

common::PathPoint PathData::GetPathPointWithPathS(const double s) const {
  return discretized_path_.Evaluate(s);
}

bool PathData::GetPathPointWithRefS(const double ref_s,
                                    common::PathPoint* const path_point) const {
  ACHECK(reference_line_);
  DCHECK_EQ(discretized_path_.size(), frenet_path_.size());
  if (frenet_path_.is_forward_path()) {
    if (ref_s < 0) {
      AERROR << "ref_s[" << ref_s << "] should be > 0";
      return false;
    }
    if (ref_s > frenet_path_.back().s()) {
      AERROR << "ref_s is larger than the length of frenet_path_ length ["
             << frenet_path_.back().s() << "].";
      return false;
    }
  } else {
    if (ref_s > reference_line_->Length()) {
      AERROR << "ref_s[" << ref_s << "] should be < length of reference["
             << reference_line_->Length() << "]";
      return false;
    }
    if (ref_s < frenet_path_.back().s()) {
      AERROR << "ref_s is less than the length of frenet_path_ length ["
             << frenet_path_.back().s() << "].";
      return false;
    }
  }
  uint32_t index = 0;
  const double kDistanceEpsilon = 1e-3;
  for (uint32_t i = 0; i + 1 < frenet_path_.size(); ++i) {
    if (fabs(ref_s - frenet_path_.at(i).s()) < kDistanceEpsilon) {
      path_point->CopyFrom(discretized_path_.at(i));
      return true;
    }

    if (frenet_path_.is_forward_path() && frenet_path_.at(i).s() < ref_s &&
        ref_s <= frenet_path_.at(i + 1).s()) {
      index = i;
      break;
    }
    if (!frenet_path_.is_forward_path() && frenet_path_.at(i).s() > ref_s &&
        ref_s >= frenet_path_.at(i + 1).s()) {
      index = i;
      break;
    }
  }
  double r = (ref_s - frenet_path_.at(index).s()) /
             (frenet_path_.at(index + 1).s() - frenet_path_.at(index).s());

  const double discretized_path_s = discretized_path_.at(index).s() +
                                    r * (discretized_path_.at(index + 1).s() -
                                         discretized_path_.at(index).s());
  path_point->CopyFrom(discretized_path_.Evaluate(discretized_path_s));

  return true;
}

void PathData::Clear() {
  discretized_path_.clear();
  frenet_path_.clear();
  path_point_decision_guide_.clear();
  path_reference_.clear();
  reference_line_ = nullptr;
}

std::string PathData::DebugString() const {
  const auto limit = static_cast<uint>(
      std::min(discretized_path_.size(),
               static_cast<size_t>(FLAGS_trajectory_point_num_for_debug)));

  return absl::StrCat(
      "[\n",
      absl::StrJoin(discretized_path_.begin(),
                    discretized_path_.begin() + limit, ",\n",
                    TL::common::util::DebugStringFormatter()),
      "]\n");
}

bool PathData::SLToXY(const FrenetFramePath& frenet_path,
                      DiscretizedPath* const discretized_path) {
  std::vector<hdmap::LaneInfoConstPtr> lane_infos;
  std::vector<common::PathPoint> path_points;
  path_points.reserve(frenet_path.size());
  for (const common::FrenetFramePoint& frenet_point : frenet_path) {
    const common::SLPoint sl_point =
        PointFactory::ToSLPoint(frenet_point.s(), frenet_point.l());
    common::math::Vec2d cartesian_point;
    if (!reference_line_->SLToXY(sl_point, &cartesian_point)) {
      AERROR << "Fail to convert sl point to xy point";
      return false;
    }
    const ReferencePoint ref_point =
        reference_line_->GetReferencePoint(frenet_point.s());
    const double theta = CartesianFrenetConverter::CalculateTheta(
        ref_point.heading(), ref_point.kappa(), frenet_point.l(),
        frenet_point.dl());
    const double kappa = CartesianFrenetConverter::CalculateKappa(
        ref_point.kappa(), ref_point.dkappa(), frenet_point.l(),
        frenet_point.dl(), frenet_point.ddl());

    double s = 0.0;
    double dkappa = 0.0;
    if (!path_points.empty()) {
      common::math::Vec2d last = PointFactory::ToVec2d(path_points.back());
      const double distance = (last - cartesian_point).Length();
      s = path_points.back().s() + distance;
      dkappa = (kappa - path_points.back().kappa()) / distance;
    }
    path_points.emplace_back(
        PointFactory::ToPathPoint(cartesian_point.x(), cartesian_point.y(), 0.0,
                                  s, theta, kappa, dkappa));
    lane_infos.clear();
    reference_line_->GetLaneFromS(frenet_point.s(), &lane_infos);
    if (!lane_infos.empty() && lane_infos.front() != nullptr) {
      path_points.back().set_lane_id(lane_infos.front()->id().id());
    }
  }
  *discretized_path = DiscretizedPath(std::move(path_points));

  return true;
}

bool PathData::XYToSL(const DiscretizedPath& discretized_path,
                      FrenetFramePath* const frenet_path) {
  ACHECK(reference_line_);
  std::vector<common::FrenetFramePoint> frenet_frame_points;
  frenet_frame_points.reserve(discretized_path.size());
  const double max_len = reference_line_->Length();
  for (const auto& path_point : discretized_path) {
    common::FrenetFramePoint frenet_point =
        reference_line_->GetFrenetPoint(path_point);
    if (!frenet_point.has_s()) {
      SLPoint sl_point;
      if (!reference_line_->XYToSL(path_point, &sl_point)) {
        AERROR << "Fail to transfer cartesian point to frenet point.";
        return false;
      }
      common::FrenetFramePoint frenet_point;
      // NOTICE: does not set dl and ddl here. Add if needed.
      frenet_point.set_s(std::max(0.0, std::min(sl_point.s(), max_len)));
      frenet_point.set_l(sl_point.l());
      frenet_frame_points.emplace_back(frenet_point);
      continue;
    }
    frenet_point.set_s(std::max(0.0, std::min(frenet_point.s(), max_len)));
    frenet_frame_points.emplace_back(frenet_point);
  }
  *frenet_path = FrenetFramePath(std::move(frenet_frame_points));
  return true;
}

bool PathData::LeftTrimWithRefS(const common::FrenetFramePoint& frenet_point) {
  ACHECK(reference_line_);
  std::vector<common::FrenetFramePoint> frenet_frame_points;
  frenet_frame_points.emplace_back(frenet_point);

  for (const common::FrenetFramePoint& fp : frenet_path_) {
    if (std::fabs(fp.s() - frenet_point.s()) < 1e-6) {
      continue;
    }
    if (frenet_path_.is_forward_path() && fp.s() > frenet_point.s()) {
      frenet_frame_points.push_back(fp);
    }
    if (!frenet_path_.is_forward_path() && fp.s() < frenet_point.s()) {
      frenet_frame_points.push_back(fp);
    }
  }
  SetFrenetPath(FrenetFramePath(std::move(frenet_frame_points)));
  return true;
}

bool PathData::UpdateFrenetFramePath(const ReferenceLine* reference_line) {
  reference_line_ = reference_line;
  return SetDiscretizedPath(discretized_path_);
}

void PathData::set_path_label(const std::string& label) {
  path_label_ = label;
}

const std::string& PathData::path_label() const {
  return path_label_;
}

const std::vector<PathPoint>& PathData::path_reference() const {
  return path_reference_;
}

void PathData::set_path_reference(
    const std::vector<PathPoint>& path_reference) {
  path_reference_ = path_reference;
}

bool PathData::GetRefSWithPathS(const double path_s,
                                double* const ref_s) const {
  if (discretized_path_.empty()) {
    return false;
  }
  const auto discretized_path_point_index =
      path_s / frenet_path_.GetSpaceResolution();
  const auto prev_index = common::math::Clamp(
      static_cast<int>(std::floor(discretized_path_point_index)), 0,
      static_cast<int>(discretized_path_.size() - 1));
  const auto next_index = common::math::Clamp(
      static_cast<int>(std::ceil(discretized_path_point_index)), 0,
      static_cast<int>(discretized_path_.size() - 1));
  if (prev_index == next_index) {
    *ref_s = frenet_path_[prev_index].s();
  } else {
    const auto ratio =
        (path_s - discretized_path_[prev_index].s()) /
        (discretized_path_[next_index].s() - discretized_path_[prev_index].s());
    *ref_s =
        frenet_path_[prev_index].s() +
        ratio * (frenet_path_[next_index].s() - frenet_path_[prev_index].s());
  }
  return true;
}

void PathData::SetOffsetCompensation(const PathPoint& shift_point) {
  if (discretized_path_.empty()) {
    return;
  }
  const double shift_x = shift_point.x() - discretized_path_.front().x();
  const double shift_y = shift_point.y() - discretized_path_.front().y();
  const double shift_theta =
      shift_point.theta() - discretized_path_.front().theta();
  for (auto& point : discretized_path_) {
    point.set_x(point.x() + shift_x);
    point.set_y(point.y() + shift_y);
    point.set_theta(common::math::NormalizeAngle(point.theta() + shift_theta));
  }
}

}  // namespace planning
}  // namespace TL
