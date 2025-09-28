/******************************************************************************
 * Copyright
 *
*/

#include "planning/prediction/common/discretized_trajectory.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "common/math/linear_interpolation.h"
#include "common/math/math_utils.h"
#include "common/math/path_matcher.h"

#include "common/math/vec2d.h"
#include "planning/common/planning_gflags.h"
#include "proto/common/pnc_point.pb.h"

namespace TL {
namespace prediction {

using TL::common::PathPoint;

DiscretizedPath::DiscretizedPath(std::vector<PathPoint> path_points)
    : std::vector<PathPoint>(std::move(path_points)) {

  if (size() < 2) {
    return;
  }

  accumulated_s_.reserve(size());
  segments_.reserve(size());
  double s = 0.0;
  accumulated_s_.emplace_back(s);
  for (int i = 0; i + 1 < size(); ++i) {
    const auto current_point = at(i);
    const auto next_point = at(i + 1);
    s += std::hypot(next_point.x() - current_point.x(),
                    next_point.y() - current_point.y());
    accumulated_s_.emplace_back(s);
    segments_.emplace_back(
        common::math::Vec2d{current_point.x(), current_point.y()},
        common::math::Vec2d{next_point.x(), next_point.y()});
  }
}

double DiscretizedPath::Length() const {
  if (empty()) {
    return 0.0;
  }
  return back().s() - front().s();
}

PathPoint DiscretizedPath::Evaluate(const double path_s) const {
  ACHECK(!empty());
  auto it_lower = QueryLowerBound(path_s);
  if (it_lower == begin()) {
    return front();
  }
  if (it_lower == end()) {
    return back();
  }
  return common::math::InterpolateUsingLinearApproximation(*(it_lower - 1),
                                                           *it_lower, path_s);
}

std::vector<PathPoint>::const_iterator DiscretizedPath::QueryLowerBound(
    const double path_s) const {
  auto func = [](const PathPoint& tp, const double path_s) {
    return tp.s() < path_s;
  };
  return std::lower_bound(begin(), end(), path_s, func);
}

PathPoint DiscretizedPath::EvaluateReverse(const double path_s) const {
  ACHECK(!empty());
  auto it_upper = QueryUpperBound(path_s);
  if (it_upper == begin()) {
    return front();
  }
  if (it_upper == end()) {
    return back();
  }
  return common::math::InterpolateUsingLinearApproximation(*(it_upper - 1),
                                                           *it_upper, path_s);
}

std::vector<PathPoint>::const_iterator DiscretizedPath::QueryUpperBound(
    const double path_s) const {
  auto func = [](const double path_s, const PathPoint& tp) {
    return tp.s() < path_s;
  };
  return std::upper_bound(begin(), end(), path_s, func);
}

bool DiscretizedPath::XYToSL(const double x, const double y,
                             common::SLPoint* const sl_point) const {
  if (sl_point == nullptr || empty()) {
    return false;
  }

  auto sl = common::math::PathMatcher::GetPathFrenetCoordinate(*this, x, y);
  sl_point->set_s(sl.first);
  sl_point->set_l(sl.second);

  return true;
}

bool DiscretizedPath::IsPointIn(const common::PathPoint& point,
                                double dis_threshold,
                                double ang_threshold) const {
  bool ret = false;
  if (empty()) {
    return ret;
  }
  common::SLPoint point_sl;
  if (1 == size()) {
    ret = IsSamePoint(front(), point, dis_threshold, ang_threshold);
  } else if (XYToSL(point.x(), point.y(), &point_sl)) {
    const bool s_in_limit = point_sl.s() > front().s() - dis_threshold &&
                            point_sl.s() < back().s() + dis_threshold;
    const bool l_in_limit = fabs(point_sl.l()) < dis_threshold;
    ret = s_in_limit && l_in_limit;
  }
  return ret;
}

bool DiscretizedPath::IsSamePoint(const common::PathPoint& p_a,
                                  const common::PathPoint& p_b,
                                  double dis_threshold, double ang_threshold) {
  return fabs(p_a.x() - p_b.x()) < dis_threshold &&
         fabs(p_a.y() - p_b.y()) < dis_threshold &&
         fabs(common::math::NormalizeAngle(p_a.theta() - p_b.theta())) <
             ang_threshold;
}

bool DiscretizedPath::GetProjection(const common::math::Vec2d& point,
                                    double* accumulate_s, double* lateral,
                                    double* min_distance, int* index_min,
                                    double radius1d, int index_center) const {
  if (segments_.empty() || accumulate_s == nullptr || lateral == nullptr ||
      min_distance == nullptr) {
    return false;
  }

  const auto segments_count = segments_.size();

  int min_index = 0;
  if (segments_.size() != 1) {
    *min_distance = std::numeric_limits<double>::infinity();

    double distance_temp_min =
        fmin(segments_[0].start().DistanceSquareTo(point),
             segments_[0].end().DistanceSquareTo(point));
    const auto* segment_min_1 = segments_.data();
    const auto* segment_min_2 = &segments_[1];

    double distance = 0;
    size_t start_index = 1;
    size_t end_index = std::numeric_limits<size_t>::max();
    if (radius1d >= 0 && index_center >= 0 &&
        index_center < static_cast<int>(accumulated_s_.size())) {
      const auto center_s = accumulated_s_.at(index_center);
      const auto start_s = fmax(center_s - radius1d, accumulated_s_.front());
      const auto end_s = fmin(center_s + radius1d, accumulated_s_.back());
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

    for (size_t i = start_index; i + 1 < segments_count && i <= end_index;
         ++i) {
      distance = segments_[i].end().DistanceSquareTo(point);
      if (distance < distance_temp_min) {
        min_index = i;  //NOLINT
        distance_temp_min = distance;
        segment_min_1 = &segments_[i];
        segment_min_2 = &segments_[i + 1];
      }
    }

    if (start_index == end_index) {
      min_index = static_cast<int>(start_index);
      *min_distance = segments_[start_index].DistanceSquareTo(point);
    } else {
      distance_temp_min = segment_min_1->DistanceSquareTo(point);
      distance = segment_min_2->DistanceSquareTo(point);
      if (distance_temp_min > distance) {
        *min_distance = distance;
        min_index++;
      } else {
        *min_distance = distance_temp_min;
      }
      *min_distance = sqrt(*min_distance);
    }
  } else {
    *min_distance = sqrt(segments_[0].DistanceSquareTo(point));
  }
  if (index_min != nullptr) {
    *index_min = min_index;
  }
  const auto& nearest_seg = segments_[min_index];
  const auto prod = nearest_seg.ProductOntoUnit(point);
  const auto proj = nearest_seg.ProjectOntoUnit(point);
  if (min_index == 0) {
    *accumulate_s = fmin(proj, nearest_seg.length());
    if (proj < 0) {
      *lateral = prod;
    } else {
      *lateral = (prod > 0.0 ? 1 : -1) * *min_distance;
    }
  } else if (min_index + 1 == segments_count) {
    *accumulate_s = accumulated_s_[min_index] + fmax(0.0, proj);
    if (proj > 0) {
      *lateral = prod;
    } else {
      *lateral = (prod > 0.0 ? 1 : -1) * *min_distance;
    }
  } else {
    *accumulate_s =
        accumulated_s_[min_index] + fmax(0.0, fmin(proj, nearest_seg.length()));
    *lateral = (prod > 0.0 ? 1 : -1) * *min_distance;
  }
  return true;
}

}  // namespace prediction
}  // namespace TL
