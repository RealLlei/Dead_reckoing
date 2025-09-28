/*
 * Copyright (c) TL auto Co., Ltd. 2021-2022. All rights reserved.
 */
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_old_decider/navigation_lanecentral_constructor.h"

#include <algorithm>
#include <limits>
#include <vector>

#include "common/file/log.h"
#include "common/math/double_type.h"
#include "common/math/integral.h"
#include "planning/common/planning_gflags.h"

namespace TL {
namespace planning {
// NOLINTBEGIN
using ::google::protobuf::RepeatedField;
using ::google::protobuf::RepeatedPtrField;

using ::TL::common::Point3D;
// using ::TL::common::math::DoubleType;
using ::TL::common::math::IntegrateByGaussLegendre;
using ::TL::common::math::Vec2d;
using std::vector;

const double kProjectionEpsilon = 1e-08;
const double CONFIDENCE = 0.8;

bool NaviLaneCentralConstructorOld::BuildReferenceLine(
    const double central_length, const double lane_width,
    const perception::LaneMarker& right_lane_marker,
    const perception::LaneMarker& left_lane_marker,
    vector<Vec2d>* const central_line_pionts, vector<Vec2d>* left_boundary,
    vector<Vec2d>* right_boundary, Vec2d* delta_point) {
  CHECK_NOTNULL(central_line_pionts);
  vector<Vec2d> left_vec{};
  vector<Vec2d> right_vec{};
  // 未使用默认道路宽度lane_width，而是使用感知得到的道路宽度perceives_lane_width来求中心线。
  double perceives_lane_width = std::fabs(right_lane_marker.c0_position() -
                                          left_lane_marker.c0_position());
  ADEBUG << " default_lane_width = " << lane_width
         << "; perceives_lane_width = " << perceives_lane_width;
  // 求解左右边界线方程，转化为左右边界点信息，存储到left_vec和right_vec中
  if (!EvaulateBoundaries(right_lane_marker, left_lane_marker, central_length,
                          &left_vec, &right_vec, delta_point)) {
    AERROR << "Lane has no boundary";
    return false;
  }

  // ADEBUG << "FLAGS_common_lane_width " << width;
  double weight_left_boundary = left_lane_marker.quality();
  double weight_right_boundary = right_lane_marker.quality();
  auto weights_left_boundary = weight_left_boundary > weight_right_boundary
                                   ? weight_left_boundary
                                   : weight_right_boundary;
  (*central_line_pionts).clear();
  // 根据左右边界点，求解平均后的中心线点信息。存储到_central_pts中，
  if (!BuildCentralLine(left_vec, right_vec, 0.5, perceives_lane_width,
                        central_line_pionts)) {
    AERROR << "Lane cannot build central line";
    return false;
  }

  // (*central_line_pionts) = right_vec;
  left_boundary->swap(left_vec);
  right_boundary->swap(right_vec);
  return true;
}

bool NaviLaneCentralConstructorOld::BuildCentralLine(
    const std::vector<Vec2d>& left_line, const std::vector<Vec2d>& right_line,
    const double& weight_left_boundary, const double& width,
    std::vector<Vec2d>* const central_pts) const {
  double left_to_central = 0.0;
  double right_to_central = 0.0;
  left_to_central = width * 0.5f;
  right_to_central = -left_to_central;

  // 由左边界点求出的中心线
  std::vector<Vec2d> central_from_left;
  bool left_is_projected = false;
  // 与零相等返回零
  if (common::math::double_type::Compare(weight_left_boundary, 0.0) != 0) {
    left_is_projected =
        DoProjection(left_line, left_to_central, &central_from_left);
    if (!left_is_projected) {
      // "NO Central Line Found From Left Boundary";
      central_from_left.clear();
    }
  }

  // 由右边界点求出的中心线
  std::vector<Vec2d> central_from_right;
  bool right_is_projected = false;
  if (common::math::double_type::Compare(1 - weight_left_boundary, 0.0f) != 0) {
    right_is_projected =
        DoProjection(right_line, right_to_central, &central_from_right);
    if (!right_is_projected) {
      // "NO Central Line Found From Right Boundary";
      central_from_right.clear();
    }
  }

  const size_t num_left_pts = central_from_left.size();
  const size_t num_right_pts = central_from_right.size();
  if (num_left_pts <= 2 || num_right_pts <= 2) {
    AERROR << "central line by left or right too short:"
           << "num_left_pts = " << num_left_pts
           << "; num_right_pts = " << num_right_pts;
    return false;
  }
  const size_t num_pts =
      num_left_pts < num_right_pts ? num_left_pts : num_right_pts;
  Vec2d pts(0.0f, 0.0f);
  for (size_t i = 0; i < num_pts; i++) {
    pts = weight_left_boundary * central_from_left[i] +
          (1 - weight_left_boundary) * central_from_right[i];
    // 根据左右权重平均后的中心线
    central_pts->push_back(pts);
  }
  ADEBUG << "central line size by left = " << num_left_pts
         << "central line size by right = " << num_right_pts
         << "central line size = " << central_pts->size();

  return left_is_projected && right_is_projected;
}

bool NaviLaneCentralConstructorOld::DoProjection(
    const std::vector<Vec2d>& ref_v, const double& width,
    std::vector<Vec2d>* const results) const {
  int max_length = ref_v.size();
  if (ref_v.empty() || (max_length < 2)) {
    AERROR << "Input ref_v is empty";
    return false;
  }
  const std::vector<Vec2d>::const_iterator refv_begin = ref_v.begin();
  const std::vector<Vec2d>::const_iterator refv_end = ref_v.end();

  // Initialize the projection of the first line segment
  // 找出的垂向量都是朝向中间的。
  std::vector<Vec2d>::const_iterator p0 = refv_begin;
  std::vector<Vec2d>::const_iterator p1 = (p0 + 1);
  Vec2d project_0;
  bool is_perpendicular = FindPerpendicular(*p0, *p1, width, &project_0);
  if (!is_perpendicular) {
    AERROR << "Cannot Find Perpendicular Line";
    return false;
  }

  Vec2d a = project_0 + *p0;
  Vec2d b = project_0 + *p1;
  results->push_back(a);
  results->push_back(b);
  int index = 0;
  for (std::vector<Vec2d>::const_iterator i = refv_begin + 1;
       (i + 1) != refv_end; i++) {
    index++;
    // AERROR << "Before p2 " << index;
    // AERROR << "max_length" << max_length;
    const std::vector<Vec2d>::const_iterator p2 = (i + 1);
    p1 = p2 - 1;
    p0 = p1 - 1;
    // AERROR << "DoProjection step two p2-p1";
    // AERROR << p2->x() << "," << p2->y() << "."<< p1->x() << "," <<
    // p1->y();
    const Vec2d v2_p2p1 = *p2 - *p1;
    const Vec2d v1_p1p0 = *p1 - *p0;
    const double dot_v2v1 = v2_p2p1.InnerProd(v1_p1p0);

    const double cross_v2v1 = v2_p2p1.CrossProd(v1_p1p0);
    // 外积很小，说明两个向量平行，则可以直接使用上一个点的垂向量。
    if (cross_v2v1 <= kProjectionEpsilon) {
      // these two line segments are colinear, sharing the same projection
      b = project_0 + *p2;
      results->push_back(b);
      // AERROR << "DoProjection step continue";
      continue;
    }
    Vec2d project_1;
    // 找出中间点与下一个点的垂向量
    is_perpendicular = FindPerpendicular(*p1, *p2, width, &project_1);
    if (!is_perpendicular) {
      AERROR << "Cannot Find Perpendicular Line";
      return false;
    }
    const Vec2d project_2 = Vec2d(0, 0) - project_1;
    const double dot_pro2pro1 = project_2.InnerProd(project_0);
    const double dot_pro1pro0 = project_1.InnerProd(project_0);
    Vec2d project(0, 0);
    // 内积为零，两向量垂直
    if (abs(dot_v2v1) <= kProjectionEpsilon) {
      // two line segments are perpendicular
      project = project_1.InnerProd(v1_p1p0) > 0 ? project_1 : project_2;
      project =
          project_0.InnerProd(v2_p2p1) < 0 ? project : Vec2d(0, 0) - project;
    } else {
      // general siutation: two line segements are neither colinear, nor
      // perpendicular
      project = dot_pro1pro0 * dot_v2v1 > 0 ? project_1 : project_2;
    }
    a = project + *p1;
    b = project + *p2;
    // found projection vector, start looking for intersection
    const Vec2d& res_back = results->back();
    const double x_a = res_back.x();
    const double y_a = res_back.y();
    const double x_b = a.x();
    const double y_b = a.y();
    const double x_c = (*p1).x();
    const double y_c = (*p1).y();

    const Vec2d a_b = (a - res_back);
    if (a_b.Length() <= kProjectionEpsilon) {
      // two line segments are colinear, sharing the same projection
      b = project_0 + (*p2);
      results->push_back(b);
      continue;
    }
    const double d = width;
    const double m =
        (d * d + (x_a * x_a + y_a * y_a) - (x_c * x_c + y_c * y_c));
    const double n = ((x_b * x_b + y_b * y_b) - (x_a * x_a + y_a * y_a));
    const double a =
        2 * (x_a - x_c) * (y_a - y_b) - 2 * (x_b - x_a) * (y_c - y_a);

    const double y = (m * (x_b - x_a) - n * (x_a - x_c)) / a;
    const double x = (m * (y_a - y_b) - n * (y_c - y_a)) / a;

    // the projection does not make sense if the intersection falls on the
    // reversed extended line
    std::vector<Vec2d>::iterator i_pro_now = results->end() - 1;
    const std::vector<Vec2d>::const_iterator p0_1 = (i_pro_now - 1);
    const Vec2d p1_reset(x, y);
    const Vec2d v_p1reset_p0 = p1_reset - *p0_1;
    const double direction_p0p1 = v_p1reset_p0.InnerProd(v1_p1p0);
    if (direction_p0p1 < 0) {
      // Cannot Projection
      // Reduce width
      // Or Resize Polygon
      AERROR << "Cannot sweep, adjust width or check points";
      return false;
    }
    (*i_pro_now).set_x(x);
    (*i_pro_now).set_y(y);
    results->push_back(b);
    project_0 = project;
  }

  return true;
}

bool NaviLaneCentralConstructorOld::EvaulateBoundaries(
    const perception::LaneMarker& right_lanemarker,
    const perception::LaneMarker& left_lanemarker, const double& length,
    std::vector<Vec2d>* const left_vecs, std::vector<Vec2d>* const right_vecs,
    Vec2d* point) const {
  double left_lanemarker_length = 0.0;
  double lanemarker_longitude_end =
      fmin(left_lanemarker.longitude_end(), right_lanemarker.longitude_end());
  auto left_func = [&](const double& x) {
    double df = 3 * left_lanemarker.c3_curvature_derivative() * x * x +
                2 * left_lanemarker.c2_curvature() * x +
                left_lanemarker.c1_heading_angle();
    return sqrt(1 + df * df);
  };
  left_lanemarker_length += IntegrateByGaussLegendre<5>(
      left_func, left_lanemarker.longitude_start(), lanemarker_longitude_end);

  double right_lanemark_length = 0.0;
  auto right_func = [&](const double& x) {
    double df = 3 * right_lanemarker.c3_curvature_derivative() * x * x +
                2 * right_lanemarker.c2_curvature() * x +
                right_lanemarker.c1_heading_angle();
    return sqrt(1 + df * df);
  };
  right_lanemark_length += IntegrateByGaussLegendre<5>(
      right_func, right_lanemarker.longitude_start(), lanemarker_longitude_end);
  double left_stepwise_factor = FLAGS_common_center_line_resolution;
  double right_stepwise_factor = FLAGS_common_center_line_resolution;

  if (left_lanemarker_length < right_lanemark_length) {
    left_stepwise_factor *= left_lanemarker_length / right_lanemark_length;
  } else {
    right_stepwise_factor *= right_lanemark_length / left_lanemarker_length;
  }
  if (common::math::double_type::Compare(left_stepwise_factor, 0.0f) <= 0 ||
      common::math::double_type::Compare(right_stepwise_factor, 0.0f) <= 0) {
    AERROR << "lane line stepwise <= 0";
    return false;
  }

  double left_lanemarker_length_end =
      std::max(left_lanemarker_length > 50.0
                   ? std::max(left_lanemarker_length, 100.0)
                   : (left_lanemarker_length > 20.0
                          ? 50.0
                          : std::max(left_lanemarker_length, 20.0)),
               length);
  double right_lanemark_length_end =
      std::max(right_lanemark_length > 50.0
                   ? std::max(right_lanemark_length, 100.0)
                   : (right_lanemark_length > 20.0
                          ? 50.0
                          : std::max(right_lanemark_length, 20.0)),
               length);
  ADEBUG << " lanemarker_longitude_end = " << lanemarker_longitude_end
         << "; left_lanemarker_length = " << left_lanemarker_length_end
         << "; right_lanemark_length =" << right_lanemark_length_end;
  double cur_left_length = 0.0;
  double cur_right_length = 0.0;
  cur_left_length = EvaulateBoundary(left_lanemarker, left_stepwise_factor,
                                     left_lanemarker_length_end, left_vecs);
  cur_right_length = EvaulateBoundary(right_lanemarker, right_stepwise_factor,
                                      right_lanemark_length_end, right_vecs);
  ADEBUG << " before extend:cur_left_length = " << cur_left_length
         << "; size of left_vecs = " << left_vecs->size();
  ADEBUG << " cur_right_length = " << cur_right_length
         << "; size of right_vecs =" << right_vecs->size();
  // Determine the direction of the extension line
  if (left_vecs->size() < 2 || right_vecs->size() < 2) {
    AERROR << "error vecs_size:"
           << "left_vecs_size = " << left_vecs->size()
           << "; right_vecs_size = " << right_vecs->size();
    return false;
  }
  Vec2d left_delta_point(left_vecs->at(left_vecs->size() - 1).x() -
                             left_vecs->at(left_vecs->size() - 2).x(),
                         left_vecs->at(left_vecs->size() - 1).y() -
                             left_vecs->at(left_vecs->size() - 2).y());
  Vec2d right_delta_point(right_vecs->at(right_vecs->size() - 1).x() -
                              right_vecs->at(right_vecs->size() - 2).x(),
                          right_vecs->at(right_vecs->size() - 1).y() -
                              right_vecs->at(right_vecs->size() - 2).y());
  if (point->Length() <= 0) {
    *point = right_lanemark_length <= left_lanemarker_length ? right_delta_point
                                                             : left_delta_point;
    point->set_x(0.6 * history_point_.x() + 0.4 * point->x());
    point->set_y(0.6 * history_point_.y() + 0.4 * point->y());
    history_point_.set_x(point->x());
    history_point_.set_y(point->y());
  }
  // Extend the left boundary to a fixed scale and direction
  double left_delta_x = point->x();
  double left_delta_y = point->y();
  double left_x = left_vecs->at(left_vecs->size() - 1).x();
  double left_y = left_vecs->at(left_vecs->size() - 1).y();
  while (cur_left_length < left_lanemarker_length_end) {
    left_x += left_delta_x;
    left_y += left_delta_y;
    left_vecs->emplace_back(left_x, left_y);
    cur_left_length +=
        sqrt(left_delta_x * left_delta_x + left_delta_y * left_delta_y);
  }
  // Extend the right boundary to a fixed scale and direction
  double rigth_delta_x = point->x();
  double rigth_delta_y = point->y();
  double right_x = right_vecs->at(right_vecs->size() - 1).x();
  double right_y = right_vecs->at(right_vecs->size() - 1).y();
  while (cur_right_length < right_lanemark_length_end) {
    right_x += rigth_delta_x;
    right_y += rigth_delta_y;
    right_vecs->emplace_back(right_x, right_y);
    cur_right_length +=
        sqrt(rigth_delta_x * rigth_delta_x + rigth_delta_y * rigth_delta_y);
  }
  ADEBUG << " After extend: cur_left_length = " << cur_left_length
         << "; size of left_vecs = " << left_vecs->size();
  ADEBUG << " after cur_right_length = " << cur_right_length
         << "; size of right_vecs =" << right_vecs->size();

  return true;
}

double NaviLaneCentralConstructorOld::EvaulateBoundary(
    const perception::LaneMarker& lanmarker, const double& stepwise_factor,
    const double& rest_of_length, std::vector<Vec2d>* const line_pts) const {
  const double& a = lanmarker.c3_curvature_derivative();
  const double& b = lanmarker.c2_curvature();
  const double& c = lanmarker.c1_heading_angle();
  const double& d = lanmarker.c0_position();
  const double& x0 = lanmarker.longitude_start();
  const double& x1 = lanmarker.longitude_end();

  const double& start_x = x0 < x1 ? x0 : x1;
  const double& end_x = x0 > x1 ? x0 : x1;

  double x_2(0.0f);
  double x_3(0.0f);
  double y(0.0f);
  double length(0.0f);
  double prev_x(start_x);
  double prev_y(a * prev_x * prev_x * prev_x + b * prev_x * prev_x +
                c * prev_x + d);
  for (double x = start_x; x <= end_x;) {
    if (length >= rest_of_length) {
      return length;
    }
    x_2 = x * x;
    x_3 = x * x_2;
    y = a * x_3 + b * x_2 + c * x + d;

    length += sqrt((x - prev_x) * (x - prev_x) + (y - prev_y) * (y - prev_y));

    prev_x = x;
    prev_y = y;
    line_pts->emplace_back(x, y);

    double slope = 3 * a * x * x + 2 * b * x + c;
    double stepwise = std::max(1e-7, stepwise_factor / sqrt(1 + slope * slope));
    // ADEBUG << "x = " << x << " y = " << y << " slope = " << slope
    //        << " stepwise = " << stepwise;
    x += stepwise;
  }
  return length;
}

bool NaviLaneCentralConstructorOld::FindPerpendicular(const Vec2d& p0,
                                                      const Vec2d& p1,
                                                      const double& distance,
                                                      Vec2d* const res) const {
  const double x0 = p0.x();
  const double y0 = p0.y();
  const double x1 = p1.x();
  const double y1 = p1.y();
  const double d = distance;

  double x(0.0);
  x = d * d * (y0 - y1) * (y0 - y1);
  x /= ((y0 - y1) * (y0 - y1) + (x1 - x0) * (x1 - x0));
  x = sqrt(x);

  double y(0.0);
  y = d * d * (x1 - x0) * (x1 - x0);
  y /= ((y0 - y1) * (y0 - y1) + (x1 - x0) * (x1 - x0));
  if ((y0 - y1) * (x1 - x0) > 0) {
    y = sqrt(y);
  } else {
    y = -sqrt(y);
  }

  Vec2d p(x, y);
  const Vec2d p1_p0 = p1 - p0;

  double threshold = abs(p.InnerProd(p1_p0));
  if (threshold > kProjectionEpsilon) {
    AERROR << "Projecting Fatal Error (No Perpendicular Bisector)";
    AERROR << "Dot Product in Finding Perpendicular = " << threshold;
    return false;
  }

  if (p.CrossProd(p1_p0) * d < 0.0f) {
    p.set_x(-p.x());
    p.set_y(-p.y());
  }

  res->set_x(p.x());
  res->set_y(p.y());

  return true;
}  // namespace TL

// NOLINTEND
}  // namespace planning
}  // namespace TL
