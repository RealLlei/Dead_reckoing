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

#include "common/math/line_segment2d.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "absl/strings/str_cat.h"
#include "common/file/log.h"

#include "common/math/double_type.h"
#include "common/math/math_utils.h"
namespace TL {
namespace common {
namespace math {

void GenerateCenterPoint(const std::vector<Vec2d>& left_point,
                         const std::vector<Vec2d>& right_point,
                         std::vector<Vec2d>* cent_point) {
  if (left_point.size() >= 2 && right_point.size() >= 2) {
    if (left_point.size() >= right_point.size()) {
      CenterPoint(left_point, right_point, cent_point);
    } else {
      CenterPoint(right_point, left_point, cent_point);
    }
  } else {
    AERROR << "point size error";
  }
}

void CenterPoint(const std::vector<Vec2d>& project_points,
                 const std::vector<Vec2d>& points,
                 std::vector<Vec2d>* cent_point) {
  cent_point->emplace_back(
      (project_points.front().x() + points.front().x()) / 2,
      (project_points.front().y() + points.front().y()) / 2);
  for (int i = 1; i < project_points.size() - 1; ++i) {
    bool found(false);
    int point_count(0);
    double lambda(0.0);
    while (!found) {
      if (point_count + 1 < points.size()) {
        if (CalculateLambda(project_points[i], project_points[i + 1],
                            points[point_count], points[point_count + 1],
                            &lambda)) {
          found = true;
          break;
        }
      } else {
        break;
      }
      ++point_count;
    }
    if (found) {
      Vec2d sub_point(lambda * points[point_count].x() +
                          (1 - lambda) * points[point_count + 1].x(),
                      lambda * points[point_count].y() +
                          (1 - lambda) * points[point_count + 1].y());

      double l1(sub_point.DistanceTo(project_points[i]));
      double l2(points[point_count].DistanceTo(points[point_count + 1]));
      double theta =
          acos((project_points[i] - sub_point)
                   .InnerProd(points[point_count + 1] - points[point_count]) /
               (l1 * l2));
      double alpha = sin(theta) / (1 + sin(theta));
      cent_point->emplace_back((1 - alpha) * project_points[i] +
                               alpha * sub_point);
    } else {
      cent_point->clear();
      AERROR << "inital point error";
      return;
    }
  }
  cent_point->emplace_back((project_points.back().x() + points.back().x()) / 2,
                           (project_points.back().y() + points.back().y()) / 2);
}

bool CalculateLambda(const Vec2d& p1, const Vec2d& p2, const Vec2d& p3,
                     const Vec2d& p4, double* lambda) {
  double delta = (p3.x() - p4.x()) * (p2.x() - p1.x()) -
                 (p3.y() - p4.y()) * (p1.y() - p2.y());
  if (!TL::common::math::double_type::IsZero(delta)) {
    *lambda = ((p2.x() - p1.x()) * (p1.x() - p4.x()) +
               (p1.y() - p2.y()) * (p4.y() - p1.y())) /
              delta;
    return *lambda >= 0 && *lambda <= 1;
  }
  return false;
}

namespace {

bool IsWithin(double val, double bound1, double bound2) {
  if (bound1 > bound2) {
    std::swap(bound1, bound2);
  }
  return val >= bound1 - kMathEpsilon && val <= bound2 + kMathEpsilon;
}

}  // namespace

LineSegment2d::LineSegment2d() {
  unit_direction_ = Vec2d(1, 0);
}

LineSegment2d::LineSegment2d(const Vec2d& start, const Vec2d& end)
    : start_(start), end_(end) {
  Init(start, end);
}

void LineSegment2d::Init(const Vec2d& lhs, const Vec2d& rhs) {
  start_ = lhs;
  end_ = rhs;
  const double dx = end_.x() - start_.x();
  const double dy = end_.y() - start_.y();
  length_ = hypot(dx, dy);
  auto unit_direction =
      (length_ <= kMathEpsilon ? Vec2d(0, 0)
                               : Vec2d(dx / length_, dy / length_));
  unit_direction_ = unit_direction;
  heading_ = unit_direction.Angle();
}

Vec2d LineSegment2d::rotate(const double angle) const {
  Vec2d diff_vec = end_ - start_;
  diff_vec.SelfRotate(angle);
  return start_ + diff_vec;
}

double LineSegment2d::length() const {
  return length_;
}

double LineSegment2d::length_sqr() const {
  return length_ * length_;
}

double LineSegment2d::DistanceTo(const Vec2d& point) const {
  if (length_ <= kMathEpsilon) {
    return point.DistanceTo(start_);
  }
  const double x0 = point.x() - start_.x();
  const double y0 = point.y() - start_.y();
  const double proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
  if (proj <= 0.0) {
    return hypot(x0, y0);
  }
  if (proj >= length_) {
    return point.DistanceTo(end_);
  }
  return std::abs(x0 * unit_direction_.y() - y0 * unit_direction_.x());
}

double LineSegment2d::DistanceTo(const Vec2d& point,
                                 Vec2d* const nearest_pt) const {
  CHECK_NOTNULL(nearest_pt);
  if (length_ <= kMathEpsilon) {
    *nearest_pt = start_;
    return point.DistanceTo(start_);
  }
  const double x0 = point.x() - start_.x();
  const double y0 = point.y() - start_.y();
  const double proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
  if (proj < 0.0) {
    *nearest_pt = start_;
    return hypot(x0, y0);
  }
  if (proj > length_) {
    *nearest_pt = end_;
    return point.DistanceTo(end_);
  }
  *nearest_pt = start_ + unit_direction_ * proj;
  return std::abs(x0 * unit_direction_.y() - y0 * unit_direction_.x());
}

double LineSegment2d::DistanceSquareTo(const Vec2d& point,
                                       std::string* const str_info) const {
  if (length_ <= kMathEpsilon) {
    return point.DistanceSquareTo(start_);
  }
  const double x0 = point.x() - start_.x();
  const double y0 = point.y() - start_.y();
  const double proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
  if (str_info != nullptr) {
    *str_info += "  p_x:" + std::to_string(point.x()) +
                 "  p_y:" + std::to_string(point.y()) +
                 "  start_x:" + std::to_string(start_.x()) +
                 "  start_y:" + std::to_string(start_.y()) +
                 "  end_x:" + std::to_string(end_.x()) +
                 "  end_y:" + std::to_string(end_.y()) +
                 "  x0:" + std::to_string(x0) + "  y0:" + std::to_string(y0) +
                 "  proj:" + std::to_string(proj) +
                 "  length:" + std::to_string(length_);
  }
  if (proj <= 0.0) {
    return Square(x0) + Square(y0);
  }
  if (proj >= length_) {
    return point.DistanceSquareTo(end_);
  }
  return Square(x0 * unit_direction_.y() - y0 * unit_direction_.x());
}

double LineSegment2d::DistanceSquareTo(const Vec2d& point,
                                       Vec2d* const nearest_pt) const {
  CHECK_NOTNULL(nearest_pt);
  if (length_ <= kMathEpsilon) {
    *nearest_pt = start_;
    return point.DistanceSquareTo(start_);
  }
  const double x0 = point.x() - start_.x();
  const double y0 = point.y() - start_.y();
  const double proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
  if (proj <= 0.0) {
    *nearest_pt = start_;
    return Square(x0) + Square(y0);
  }
  if (proj >= length_) {
    *nearest_pt = end_;
    return point.DistanceSquareTo(end_);
  }
  *nearest_pt = start_ + unit_direction_ * proj;
  return Square(x0 * unit_direction_.y() - y0 * unit_direction_.x());
}

bool LineSegment2d::IsPointIn(const Vec2d& point) const {
  if (length_ <= kMathEpsilon) {
    return std::abs(point.x() - start_.x()) <= kMathEpsilon &&
           std::abs(point.y() - start_.y()) <= kMathEpsilon;
  }
  const double prod = CrossProd(point, start_, end_);
  if (std::abs(prod) > kMathEpsilon) {
    return false;
  }
  return IsWithin(point.x(), start_.x(), end_.x()) &&
         IsWithin(point.y(), start_.y(), end_.y());
}

double LineSegment2d::ProjectOntoUnit(const Vec2d& point) const {
  return unit_direction_.InnerProd(point - start_);
}

double LineSegment2d::ProductOntoUnit(const Vec2d& point) const {
  return unit_direction_.CrossProd(point - start_);
}

bool LineSegment2d::HasIntersect(const LineSegment2d& other_segment) const {
  Vec2d point;
  return GetIntersect(other_segment, &point);
}

bool LineSegment2d::GetIntersect(const LineSegment2d& other_segment,
                                 Vec2d* const point) const {
  CHECK_NOTNULL(point);
  if (IsPointIn(other_segment.start())) {
    *point = other_segment.start();
    return true;
  }
  if (IsPointIn(other_segment.end())) {
    *point = other_segment.end();
    return true;
  }
  if (other_segment.IsPointIn(start_)) {
    *point = start_;
    return true;
  }
  if (other_segment.IsPointIn(end_)) {
    *point = end_;
    return true;
  }
  if (length_ <= kMathEpsilon || other_segment.length() <= kMathEpsilon) {
    return false;
  }
  const double cc1 = CrossProd(start_, end_, other_segment.start());
  const double cc2 = CrossProd(start_, end_, other_segment.end());
  if (cc1 * cc2 >= -kMathEpsilon) {
    return false;
  }
  const double cc3 =
      CrossProd(other_segment.start(), other_segment.end(), start_);
  const double cc4 =
      CrossProd(other_segment.start(), other_segment.end(), end_);
  if (cc3 * cc4 >= -kMathEpsilon) {
    return false;
  }
  const double ratio = cc4 / (cc4 - cc3);
  *point = Vec2d(start_.x() * ratio + end_.x() * (1.0 - ratio),
                 start_.y() * ratio + end_.y() * (1.0 - ratio));
  return true;
}

// return distance with perpendicular foot point.
double LineSegment2d::GetPerpendicularFoot(const Vec2d& point,
                                           Vec2d* const foot_point) const {
  CHECK_NOTNULL(foot_point);
  if (length_ <= kMathEpsilon) {
    *foot_point = start_;
    return point.DistanceTo(start_);
  }
  const double x0 = point.x() - start_.x();
  const double y0 = point.y() - start_.y();
  const double proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
  *foot_point = start_ + unit_direction_ * proj;
  return std::abs(x0 * unit_direction_.y() - y0 * unit_direction_.x());
}

void LineSegment2d::GetProjectPoint(const Vec2d& point,
                                    Vec2d* foot_point) const {
  const double proj = (point.x() - start_.x()) * unit_direction_.x() +
                      (point.y() - start_.y()) * unit_direction_.y();
  *foot_point = start_ + unit_direction_ * proj;
}

void LineSegment2d::Translate(double distance, double direction) {
  Vec2d tanslate_vec = Vec2d::CreateUnitVec2d(direction) * distance;
  start_ += tanslate_vec;
  end_ += tanslate_vec;
}

void LineSegment2d::Extend(double distance) {
  end_ += Vec2d::CreateUnitVec2d(heading()) * distance;
  const double dx = end_.x() - start_.x();
  const double dy = end_.y() - start_.y();
  length_ = hypot(dx, dy);
}

void LineSegment2d::Transform(const Vec2d& origin, double direction) {
  start_ -= origin;
  start_.SelfRotate(-direction);
  end_ -= origin;
  end_.SelfRotate(-direction);
  const double dx = end_.x() - start_.x();
  const double dy = end_.y() - start_.y();
  unit_direction_ =
      (length_ <= kMathEpsilon ? Vec2d(0, 0)
                               : Vec2d(dx / length_, dy / length_));
  heading_ = unit_direction_.Angle();
}

std::string LineSegment2d::DebugString() const {
  return absl::StrCat("segment2d ( start = ", start_.DebugString(),
                      "  end = ", end_.DebugString(), " )");
}

void MultiLineSegment::Init(const std::vector<Vec2d>& points) {
  path_points_.assign(points.begin(), points.end());
  num_points_ = static_cast<int>(path_points_.size());
  CHECK_GE(num_points_, 2);

  accumulated_s_.clear();
  accumulated_s_.reserve(num_points_);
  segments_.clear();
  segments_.reserve(num_points_ - 1);
  double s = 0.0;
  for (int i = 0; i < num_points_; ++i) {
    accumulated_s_.emplace_back(s);
    Vec2d heading;
    if (i + 1 >= num_points_) {
      heading = path_points_[i] - path_points_[i - 1];
    } else {
      segments_.emplace_back(path_points_[i], path_points_[i + 1]);
      heading = path_points_[i + 1] - path_points_[i];
      // TODO(All): use heading.length when all adjacent lanes are guarantee to
      // be connected.
      s += heading.Length();
    }
  }
  num_segments_ = num_points_ - 1;
  length_ = s;
  CHECK_EQ(accumulated_s_.size(), static_cast<size_t>(num_points_));
  CHECK_EQ(segments_.size(), static_cast<size_t>(num_segments_));
}

bool MultiLineSegment::GetProjection(const Vec2d& point,  // NOLINT
                                     double* accumulate_s, double* lateral,
                                     double* min_distance, int* index_min,
                                     double radius1d, int index_center,
                                     std::string* const str_info) const {
  if (segments_.empty()) {
    return false;
  }
  if (accumulate_s == nullptr || lateral == nullptr ||
      min_distance == nullptr) {
    return false;
  }
  if (str_info != nullptr) {
    *str_info = "";
  }

  CHECK_GE(num_points_, 2);
  int min_index = 0;
  if (num_points_ == 2) {
    common::math::LineSegment2d segment_min_1 = segments_[0];
    *min_distance = sqrt(segment_min_1.DistanceSquareTo(point, str_info));
    if (str_info != nullptr) {
      *str_info += "  min_dis:" + std::to_string(*min_distance);
    }
  } else {
    *min_distance = std::numeric_limits<double>::infinity();

    double distance_temp_min = segments_[0].DistanceSquareTo(point, str_info);
    if (str_info != nullptr) {
      *str_info += "  distance_temp_min:" + std::to_string(distance_temp_min);
    }
    // distance_temp_min = std::numeric_limits<double>::infinity();
    common::math::LineSegment2d segment_min_1 = segments_[0];
    common::math::LineSegment2d segment_min_2 = segments_[1];

    double distance = 0;
    size_t start_index = 1;
    size_t end_index = std::numeric_limits<int>::max();
    if (radius1d >= 0 && index_center >= 0 &&
        index_center < accumulated_s_.size()) {
      double center_s = accumulated_s_.at(index_center);
      double start_s = fmax(center_s - radius1d, accumulated_s_.front());
      double end_s = fmin(center_s + radius1d, accumulated_s_.back());
      start_index =
          std::lower_bound(accumulated_s_.begin(),
                           accumulated_s_.begin() + index_center, start_s) -
          accumulated_s_.begin();
      end_index = std::upper_bound(accumulated_s_.begin() + index_center,
                                   accumulated_s_.end(), end_s) -
                  accumulated_s_.begin();
      start_index = TL::common::math::Clamp(
          start_index, static_cast<size_t>(0), segments_.size() - 1);
      end_index = TL::common::math::Clamp(end_index, static_cast<size_t>(0),
                                             segments_.size() - 1);
    }
    // bool flag = false;
    for (size_t i = start_index; i < num_segments_ && i <= end_index; ++i) {
      distance = segments_[i].DistanceSquareTo(point);
      if (distance < distance_temp_min) {
        // flag = true;
        min_index = static_cast<int>(i);
        distance_temp_min = distance;
      }
    }
    if (str_info != nullptr) {
      *str_info += "  start_index:" + std::to_string(start_index) +
                   "  end_index:" + std::to_string(end_index);
    }

    *min_distance = std::sqrt(distance_temp_min);
  }
  if (index_min != nullptr) {
    *index_min = min_index;
  }
  const auto& nearest_seg = segments_[min_index];
  const auto prod = nearest_seg.ProductOntoUnit(point);
  const auto proj = nearest_seg.ProjectOntoUnit(point);
  if (str_info != nullptr) {
    *str_info += "  num_segment:" + std::to_string(num_segments_) +
                 "  min_index:" + std::to_string(min_index) +
                 "  prod:" + std::to_string(prod) +
                 "  proj:" + std::to_string(proj);
  }
  if (min_index == 0) {
    *accumulate_s = std::min(proj, nearest_seg.length());
    if (proj < 0) {
      *lateral = prod;
    } else {
      *lateral = (prod > 0.0 ? 1 : -1) * *min_distance;
    }
  } else if (min_index == num_segments_ - 1) {
    *accumulate_s = accumulated_s_[min_index] + std::max(0.0, proj);
    if (proj > 0) {
      *lateral = prod;
    } else {
      *lateral = (prod > 0.0 ? 1 : -1) * *min_distance;
    }
  } else {
    *accumulate_s = accumulated_s_[min_index] +
                    std::max(0.0, std::min(proj, nearest_seg.length()));
    *lateral = (prod > 0.0 ? 1 : -1) * *min_distance;
  }
  return true;
}

}  // namespace math
}  // namespace common
}  // namespace TL
