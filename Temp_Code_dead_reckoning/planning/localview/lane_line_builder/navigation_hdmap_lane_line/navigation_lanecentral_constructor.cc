/*
 * Copyright (c) TL auto Co., Ltd. 2021-2022. All rights reserved.
 */
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/navigation_lanecentral_constructor.h"

#include <algorithm>
#include <cmath>
#include <complex>
#include <limits>
#include <string>
#include <tuple>
#include <utility>

#include "common/file/log.h"
#include "common/filters/digital_filter_coefficients.h"
#include "common/math/double_type.h"
#include "common/math/integral.h"
#include "common/status/status.h"
#include "common/time/clock.h"
#include "planning/common/planning_gflags.h"
#include "planning/math/discrete_points_math.h"

namespace TL {
namespace planning {
// using ::TL::common::math::DoubleType;
using TL::common::Status;
using TL::common::math::InterpolateVec2dPoints;
using TL::common::math::double_type::Compare;
using TL::common::math::double_type::ComparedToZero;
using std::vector;

const double kProjectionEpsilon = 1e-08;

Status NaviLaneCentralConstructor::Init() {
  lane_width_debounce_.ResetTime(0.3, 0.1, 0.1);
  return Status::OK();
}

Status NaviLaneCentralConstructor::BuildReferenceLine(  // NOLINT
    const double central_length, const double lane_width,
    const perception::LaneMarker& right_lane_marker,
    const perception::LaneMarker& left_lane_marker, vector<Vec2d>* left_points,
    vector<Vec2d>* right_points, vector<Vec2d>* const central_line_pionts,
    vector<Vec2d>* left_boundary, vector<Vec2d>* right_boundary,
    EndPonitTuple* end_point_tuple, Vec2d* delta_front_point,
    const bool is_central_lane) {
  CHECK_NOTNULL(central_line_pionts);
  if (left_points->empty() || right_points->empty()) {
    ADEBUG << " has no points, "
           << "left_points_size: " << left_points->size()
           << ", right_points_size: " << right_points->size();
    return Status(common::ErrorCode::LOCALVIEW_MAP_LANE_ERROR, "points empty!");
  }
  perception::LaneMarker lanemarker_average;
  lanemarker_average.set_c0_position(
      (right_lane_marker.c0_position() + left_lane_marker.c0_position()) / 2);
  lanemarker_average.set_c1_heading_angle(
      (right_lane_marker.c1_heading_angle() +
       left_lane_marker.c1_heading_angle()) /
      2);

  vector<Vec2d> left_vec{};
  vector<Vec2d> right_vec{};
  Status out_status;
  // 将原始的离散点根据左右长度进行插值处理，使得左右对称，然后存储到left_vec和right_vec中
  out_status =
      EvaulateBoundaries(central_length, left_points, right_points, &left_vec,
                         &right_vec, end_point_tuple, delta_front_point,
                         lanemarker_average, is_central_lane, lane_width);
  if (!out_status.ok()) {
    ADEBUG << "Lane has no boundary";
    return out_status;
  }

  // ADEBUG << "FLAGS_common_lane_width " << width;
  (*central_line_pionts).clear();
  // 根据左右边界点，求解平均后的中心线点信息。存储到_central_pts中，
  out_status = BuildCentralLine(left_vec, right_vec, 0.5, lane_width,
                                central_line_pionts);
  if (!out_status.ok()) {
    AERROR << "Lane cannot build central line";
    return out_status;
  }

  // (*central_line_pionts) = right_vec;
  left_boundary->swap(left_vec);
  right_boundary->swap(right_vec);
  return Status::OK();
}

bool NaviLaneCentralConstructor::ExtendNextReferenceLine(  // NOLINT
    const std::vector<Vec2d>& ref_boundary_points, double lane_width,
    std::vector<Vec2d>* const central_pionts,
    std::vector<Vec2d>* const extend_boundary_points) {
  return (DoProjection(ref_boundary_points, lane_width * 0.5, central_pionts)
              .ok() &&
          DoProjection(ref_boundary_points, lane_width, extend_boundary_points)
              .ok());
}

Status NaviLaneCentralConstructor::BuildCentralLine(  // NOLINT
    const std::vector<Vec2d>& left_line, const std::vector<Vec2d>& right_line,
    const double& weight_left_boundary, const double& width,
    std::vector<Vec2d>* const central_pts) const {
  double left_to_central = 0.0;
  double right_to_central = 0.0;
  left_to_central = width * 0.5;
  right_to_central = -left_to_central;

  // 由左边界点求出的中心线
  std::vector<Vec2d> central_from_left;
  Status left_is_projected{common::ErrorCode::LOCALVIEW_MAP_LANE_ERROR, " "};
  // 与零相等返回零
  if (Compare(weight_left_boundary, 0.0) != 0) {
    left_is_projected =
        DoProjection(left_line, left_to_central, &central_from_left);
    if (!left_is_projected.ok()) {
      // "NO Central Line Found From Left Boundary";
      central_from_left.clear();
    }
  }

  // 由右边界点求出的中心线
  std::vector<Vec2d> central_from_right;
  Status right_is_projected{common::ErrorCode::LOCALVIEW_MAP_LANE_ERROR, " "};
  if (Compare(1 - weight_left_boundary, 0.0) != 0) {
    right_is_projected =
        DoProjection(right_line, right_to_central, &central_from_right);
    if (!right_is_projected.ok()) {
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
    return Status(common::ErrorCode::LOCALVIEW_MAP_LANE_ERROR, "num_pts <= 2!");
  }
  const size_t num_pts =
      num_left_pts < num_right_pts ? num_left_pts : num_right_pts;
  Vec2d pts(0.0, 0.0);
  for (size_t i = 0; i < num_pts; i++) {
    pts = weight_left_boundary * central_from_left[i] +
          (1 - weight_left_boundary) * central_from_right[i];
    // 根据左右权重平均后的中心线
    central_pts->push_back(pts);
  }
  ADEBUG << "central line size by left = " << num_left_pts
         << "central line size by right = " << num_right_pts
         << "central line size = " << central_pts->size();
  return (left_is_projected.ok() && right_is_projected.ok())
             ? Status::OK()
             : Status(common::ErrorCode::LOCALVIEW_MAP_LANE_ERROR,
                      "left: " + left_is_projected.error_message() +
                          ", right: " + right_is_projected.error_message());
}

Status NaviLaneCentralConstructor::DoProjection(
    const std::vector<Vec2d>& ref_v, const double& width,
    std::vector<Vec2d>* const results) {
  int max_length = static_cast<int>(ref_v.size());
  if (ref_v.empty() || (max_length < 2)) {
    AERROR << "Input ref_v is empty";
    return Status(common::ErrorCode::LOCALVIEW_MAP_LANE_ERROR,
                  "Input ref_v is empty!");
  }
  const auto refv_begin = ref_v.begin();
  const auto refv_end = ref_v.end();

  // Initialize the projection of the first line segment
  // 找出的垂向量都是朝向中间的。
  std::vector<Vec2d>::const_iterator p0 = refv_begin;
  auto p1 = (p0 + 1);
  Vec2d project_0;
  bool is_perpendicular = FindPerpendicular(*p0, *p1, width, &project_0);
  if (!is_perpendicular) {
    AERROR << "Cannot Find Perpendicular Line";
    return Status(common::ErrorCode::LOCALVIEW_MAP_LANE_ERROR,
                  "Cannot Find Perpendicular Line!");
  }

  Vec2d a = project_0 + *p0;
  Vec2d b = project_0 + *p1;
  results->push_back(a);
  results->push_back(b);
  for (auto i = refv_begin + 1; (i + 1) != refv_end; i++) {
    // AERROR << "Before p2 " << index;
    // AERROR << "max_length" << max_length;
    const auto p2 = (i + 1);
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
      return Status(common::ErrorCode::LOCALVIEW_MAP_LANE_ERROR,
                    "Cannot Find Perpendicular Line!");
    }
    const Vec2d project_2 = Vec2d(0, 0) - project_1;
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
    auto i_pro_now = results->end() - 1;
    const auto p0_1 = (i_pro_now - 1);
    const Vec2d p1_reset(x, y);
    const Vec2d v_p1reset_p0 = p1_reset - *p0_1;
    const double direction_p0p1 = v_p1reset_p0.InnerProd(v1_p1p0);
    if (direction_p0p1 < 0) {
      // Cannot Projection
      // Reduce width
      // Or Resize Polygon
      AERROR << "Cannot sweep, adjust width or check points";
      return Status(common::ErrorCode::LOCALVIEW_MAP_LANE_ERROR,
                    "Cannot sweep, adjust width or check points!");
    }
    (*i_pro_now).set_x(x);
    (*i_pro_now).set_y(y);
    results->push_back(b);
    project_0 = project;
  }

  return Status::OK();
}

Status NaviLaneCentralConstructor::EvaulateBoundaries(
    const double& length, std::vector<Vec2d>* const left_input_points,
    std::vector<Vec2d>* const right_input_points,
    std::vector<Vec2d>* const left_vecs, std::vector<Vec2d>* const right_vecs,
    EndPonitTuple* end_point_tuple, Vec2d* delta_front_point,
    const perception::LaneMarker& average_lane_marker,
    const bool is_central_lane, const double lane_width) {
  UNUSED(average_lane_marker);
  const double back_length = FLAGS_lane_max_back_length;
  const double init_back_length = FLAGS_lane_max_back_length;
  double min_longitude_end =
      fmin(left_input_points->back().x(), right_input_points->back().x());
  Status out_status;
  // 按照纵向固定距离（最小的距离）对左右边界线进行裁剪，添加起始点和终点
  if (!AddStartAndEndPoints(left_input_points, -init_back_length,
                            min_longitude_end, 1.0) ||
      !AddStartAndEndPoints(right_input_points, -init_back_length,
                            min_longitude_end, 1.0)) {
    return Status(common::ErrorCode::LOCALVIEW_MAP_LANE_ERROR,
                  "AddStartAndEndPoints faild!");
  }
  // 计算散点长度，确定左右插值的距离
  double left_length = CalculateVec2dLength(*left_input_points);
  double right_length = CalculateVec2dLength(*right_input_points);
  double left_stepwise_factor = FLAGS_common_center_line_resolution;
  double right_stepwise_factor = FLAGS_common_center_line_resolution;
  ADEBUG << "CalculateVec2dLength, left length: " << left_length
         << ", right_length: " << right_length;
  if (left_length < right_length) {
    left_stepwise_factor *= left_length / right_length;
  } else {
    right_stepwise_factor *= right_length / left_length;
  }
  if (Compare(left_stepwise_factor, 0.0) <= 0 ||
      Compare(right_stepwise_factor, 0.0) <= 0) {
    AERROR << "lane line stepwise <= 0";
    return Status(common::ErrorCode::LOCALVIEW_MAP_LANE_ERROR,
                  "lane line stepwise <= 0");
  }
  ADEBUG << "CalculateVec2dLength, left_stepwise_factor: "
         << left_stepwise_factor
         << ", right_stepwise_factor: " << right_stepwise_factor;
  *left_vecs = InterpolateVec2dPoints(*left_input_points, left_stepwise_factor);
  *right_vecs =
      InterpolateVec2dPoints(*right_input_points, right_stepwise_factor);
  RemoveRedundantPoint(left_vecs, right_vecs);
  double min_lane_width = is_central_lane ? lane_width : 2.5;
  // s = v*min_t
  double min_lane_front_length = FLAGS_central_lane_min_length_after_cut;
  out_status =
      LaneWidthCheck(left_vecs, right_vecs, min_lane_width, left_input_points,
                     right_input_points, min_lane_front_length);
  if (!out_status.ok()) {
    return out_status;
  }
  ADEBUG << "after check lane width in zero: "
         << (left_vecs->front().y() - right_vecs->front().y())
         << "min lane width: "
         << (left_vecs->back().y() - right_vecs->back().y());
  double left_total_length = std::max(left_length, length);
  double right_total_length = std::max(right_length, length);
  double cur_left_length = CalculateVec2dLength(*left_vecs);
  double cur_right_length = CalculateVec2dLength(*right_vecs);
  // Determine the direction of the extension line
  if (left_vecs->size() < 2 || right_vecs->size() < 2) {
    ADEBUG << "error vecs_size:"
           << "left_vecs_size = " << left_vecs->size()
           << "; right_vecs_size = " << right_vecs->size();
    return Status(common::ErrorCode::LOCALVIEW_MAP_LANE_ERROR,
                  "vecs->size() < 2");
  }

  // 小于零说明是空的，第一次求point
  if (std::get<0>(*end_point_tuple).Length() <= 0) {
    std::vector<std::pair<double, double>> left_points;
    std::vector<std::pair<double, double>> right_points;
    left_points.reserve(left_vecs->size());
    right_points.reserve(right_vecs->size());
    for (const auto& point : *left_vecs) {
      left_points.emplace_back(point.x(), point.y());
    }
    // Compute path profile
    std::vector<double> left_headings;
    std::vector<double> left_kappas;
    std::vector<double> left_dkappas;
    std::vector<double> left_accumulated_s;
    DiscretePointsMath::ComputeDiscretePointsProfile(
        left_points, &left_headings, &left_accumulated_s, &left_kappas,
        &left_dkappas);
    for (const auto& point : *right_vecs) {
      right_points.emplace_back(point.x(), point.y());
    }
    // Compute path profile
    std::vector<double> right_headings;
    std::vector<double> right_kappas;
    std::vector<double> right_dkappas;
    std::vector<double> right_accumulated_s;
    DiscretePointsMath::ComputeDiscretePointsProfile(
        right_points, &right_headings, &right_accumulated_s, &right_kappas,
        &right_dkappas);
    double heading{0.0};
    double kappa{0.0};
    double dkappa{0.0};
    const int select_back_index = 5;
    if (left_vecs->size() >= 10) {
      if (std::fabs(left_kappas[left_kappas.size() - select_back_index]) <=
          std::fabs(right_kappas[right_kappas.size() - select_back_index])) {
        heading = left_headings.back();
        kappa = (left_kappas[select_back_index - 1] +
                 left_kappas[select_back_index] +
                 left_kappas[select_back_index + 1]) /
                3;
        dkappa = (left_dkappas[select_back_index] +
                  left_dkappas[select_back_index + 1] +
                  left_dkappas[select_back_index + 2]) /
                 3;
      } else {
        heading = right_headings.back();
        kappa = (right_kappas[select_back_index - 1] +
                 right_kappas[select_back_index] +
                 right_kappas[select_back_index + 1]) /
                3;
        dkappa = (right_dkappas[select_back_index] +
                  right_dkappas[select_back_index + 1] +
                  right_dkappas[select_back_index + 2]) /
                 3;
      }
    } else {
      heading = left_headings.back();
    }
    if (DoubleHasSameSign(kappa, dkappa) ||
        Compare(std::fabs(dkappa), std::fabs(kappa)) > 0) {
      dkappa = 0.0;
    }
    EndPonitTuple now_end_point_tuple = std::make_tuple(
        Vec2d(std::cos(heading), std::sin(heading)), heading, kappa, dkappa);
    if (std::get<0>(history_end_point_tuple_).Length() <= 0.0) {
      *end_point_tuple = now_end_point_tuple;
    } else {
      *end_point_tuple = GetEndPointTupleByCoefficient(
          now_end_point_tuple, history_end_point_tuple_, 0.2);
    }
    history_end_point_tuple_ = *end_point_tuple;
    ADEBUG << "end_point_time: " << FIXED << SETPRECISION(3)
           << TL::common::Clock::NowInSeconds();
    ADEBUG << "end_point_tuple_navi";
    ADEBUG << "navigation lane heading: " << heading << " , kappa: " << kappa
           << " , dkappa: " << dkappa;
  }

  ADEBUG << "right_end_point_tuple dx: " << std::get<0>(*end_point_tuple).x()
         << " , ddx: " << std::get<1>(*end_point_tuple)
         << " , dddx: " << std::get<2>(*end_point_tuple);
  ADEBUG << "end_point_tuple dy: " << std::get<0>(*end_point_tuple).y()
         << " , ddy: " << std::get<3>(*end_point_tuple)
         << " , dddy: " << std::get<3>(*end_point_tuple);
  // Extend the left boundary end to a fixed scale and direction
  double left_kappa = std::get<2>(*end_point_tuple);
  double left_heading = std::get<1>(*end_point_tuple);
  double left_x = left_vecs->at(left_vecs->size() - 1).x();
  double left_y = left_vecs->at(left_vecs->size() - 1).y();
  double new_kappa = 0.0;
  while (cur_left_length < left_total_length) {
    new_kappa = left_kappa + std::get<3>(*end_point_tuple);
    if (DoubleHasSameSign(new_kappa, std::get<2>(*end_point_tuple))) {
      left_kappa = new_kappa;
    }
    left_heading += left_kappa;
    left_x += std::cos(left_heading);
    left_y += std::sin(left_heading);
    left_vecs->emplace_back(left_x, left_y);
    cur_left_length += 1.0;
  }
  // Extend the right boundary end to a fixed scale and direction
  double right_kappa = std::get<2>(*end_point_tuple);
  double right_heading = std::get<1>(*end_point_tuple);
  double right_x = right_vecs->at(right_vecs->size() - 1).x();
  double right_y = right_vecs->at(right_vecs->size() - 1).y();

  while (cur_right_length < right_total_length) {
    new_kappa = right_kappa + std::get<3>(*end_point_tuple);
    if (DoubleHasSameSign(right_kappa, std::get<2>(*end_point_tuple))) {
      right_kappa = new_kappa;
    }
    right_heading += right_kappa;
    right_x += std::cos(right_heading);
    right_y += std::sin(right_heading);
    right_vecs->emplace_back(right_x, right_y);
    cur_right_length += 1.0;
  }
  // Extend the left/right boundary front by lanemarker
  if (back_length > left_vecs->front().x()) {
    std::vector<Vec2d> left_front_extend;
    std::vector<Vec2d> right_front_extend;
    Vec2d front_left_delta_point(left_vecs->at(1).x() - left_vecs->at(0).x(),
                                 left_vecs->at(1).y() - left_vecs->at(0).y());
    Vec2d front_right_delta_point(
        right_vecs->at(1).x() - right_vecs->at(0).x(),
        right_vecs->at(1).y() - right_vecs->at(0).y());
    if (delta_front_point->Length() <= 0) {
      *delta_front_point = right_length <= left_length ? front_right_delta_point
                                                       : front_left_delta_point;
      // delta_front_point->set_x(0.8 * history_front_point_.x() +
      //                          0.2 * delta_front_point->x());
      // delta_front_point->set_y(0.8 * history_front_point_.y() +
      //                          0.2 * delta_front_point->y());
      // history_front_point_.set_x(delta_front_point->x());
      // history_front_point_.set_y(delta_front_point->y());
      delta_front_point->Normalize();
    }
    double left_delta_x = delta_front_point->x();
    double left_delta_y = delta_front_point->y();
    left_x = left_vecs->front().x();
    left_y = left_vecs->front().y();
    double now_back_length = init_back_length;
    while (now_back_length < back_length) {
      left_x -= left_delta_x;
      left_y -= left_delta_y;
      left_front_extend.emplace_back(left_x, left_y);
      now_back_length +=
          sqrt(left_delta_x * left_delta_x + left_delta_y * left_delta_y);
    }
    double right_delta_x = delta_front_point->x();
    double right_delta_y = delta_front_point->y();
    right_x = right_vecs->front().x();
    right_y = right_vecs->front().y();
    now_back_length = init_back_length;
    while (now_back_length < back_length) {
      right_x -= right_delta_x;
      right_y -= right_delta_y;
      right_front_extend.emplace_back(right_x, right_y);
      now_back_length +=
          sqrt(right_delta_x * right_delta_x + right_delta_y * right_delta_y);
    }
    std::reverse(std::begin(left_front_extend), std::end(left_front_extend));
    std::reverse(std::begin(right_front_extend), std::end(right_front_extend));
    left_vecs->insert(left_vecs->begin(), std::begin(left_front_extend),
                      std::end(left_front_extend));
    right_vecs->insert(right_vecs->begin(), std::begin(right_front_extend),
                       std::end(right_front_extend));
  }

  ADEBUG << " After extend: cur_left_length = " << cur_left_length
         << "; size of left_vecs = " << left_vecs->size();
  ADEBUG << " after cur_right_length = " << cur_right_length
         << "; size of right_vecs =" << right_vecs->size();
  ADEBUG << "vecs end lane width: "
         << (left_vecs->back().y() - right_vecs->back().y());
  const double end_timestamp = common::Clock::NowInSeconds();
  ADEBUG << "lane_end_time:" << FIXED << SETPRECISION(3) << end_timestamp;
  return Status::OK();
}

EndPonitTuple NaviLaneCentralConstructor::GetEndPointTupleByCoefficient(
    const EndPonitTuple& tuple_first, const EndPonitTuple& tuple_second,
    double k) {
  auto heading =
      k * std::get<1>(tuple_first) + (1 - k) * std::get<1>(tuple_second);
  return std::make_tuple(Vec2d(std::cos(heading), std::sin(heading)), heading,
                         std::get<2>(tuple_first), std::get<3>(tuple_first));
}

double NaviLaneCentralConstructor::EvaulateBoundary(
    const perception::LaneMarker& lanmarker, const double& stepwise_factor,
    const double& rest_of_length, std::vector<Vec2d>* const line_pts) {
  const double& a = lanmarker.c3_curvature_derivative();
  const double& b = lanmarker.c2_curvature();
  const double& c = lanmarker.c1_heading_angle();
  const double& d = lanmarker.c0_position();
  const double& x0 = lanmarker.longitude_start();
  const double& x1 = lanmarker.longitude_end();

  const double& start_x = x0 < x1 ? x0 : x1;
  const double& end_x = x0 > x1 ? x0 : x1;

  double x_2(0.0);
  double x_3(0.0);
  double y(0.0);
  double length(0.0);
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

bool NaviLaneCentralConstructor::FindPerpendicular(const Vec2d& p0,
                                                   const Vec2d& p1,
                                                   const double& distance,
                                                   Vec2d* const res) {
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

  if (p.CrossProd(p1_p0) * d < 0.0) {
    p.set_x(-p.x());
    p.set_y(-p.y());
  }

  res->set_x(p.x());
  res->set_y(p.y());

  return true;
}

bool NaviLaneCentralConstructor::AddStartAndEndPoints(
    std::vector<Vec2d>* const boundary_points, const double start_x,
    const double end_x, const double step_length) {
  if (boundary_points->size() < 2 ||
      start_x - boundary_points->front().x() < (-step_length - 14) ||
      end_x - boundary_points->back().x() > step_length) {
    AERROR << "points size less than two or the start and end points in "
              "buundary_points not match!";
    return false;
  }
  Vec2d start_point_f{0.0, 0.0};
  Vec2d start_point_b{0.0, 0.0};
  Vec2d end_point_f{0.0, 0.0};
  Vec2d end_point_b{0.0, 0.0};
  size_t start_index = boundary_points->size() - 1;
  size_t end_index = 0;
  for (size_t i = 0; i + 1 < boundary_points->size(); i++) {
    if (boundary_points->at(i).x() > start_x) {
      start_index = i;
      break;
    }
  }
  for (size_t i = boundary_points->size() - 1;; i--) {
    if (i == 0 || boundary_points->at(i).x() < end_x) {
      end_index = i;
      break;
    }
  }
  ADEBUG << "start_index: " << start_index << ", end_index: " << end_index;
  if (start_index >= end_index) {
    AERROR << "start_index or end_index error! start_index: " << start_index
           << ", end_index: " << end_index;
    return false;
  }

  if (start_index == 0) {
    start_point_f = boundary_points->front();
    start_point_b = boundary_points->at(1);
  } else {
    start_point_f = boundary_points->at(start_index);
    start_point_b = boundary_points->at(start_index - 1);
  }

  if (end_index == boundary_points->size() - 1) {
    end_point_f = boundary_points->back();
    end_point_b = boundary_points->at(boundary_points->size() - 2);
  } else {
    end_point_f = boundary_points->at(end_index);
    end_point_b = boundary_points->at(end_index - 1);
  }
  const double start_y =
      start_point_b.y() - ((start_point_b.y() - start_point_f.y()) /
                           (start_point_b.x() - start_point_f.x())) *
                              (start_point_b.x() - start_x);
  const double end_y = end_point_b.y() - ((end_point_b.y() - end_point_f.y()) /
                                          (end_point_b.x() - end_point_f.x())) *
                                             (end_point_b.x() - end_x);
  ADEBUG << "start_y: " << start_y << ", end_y: " << end_y;
  boundary_points->erase(
      boundary_points->end() - (boundary_points->size() - 1 -  // NOLINT
                                end_index),                    // NOLINT
      boundary_points->end());
  boundary_points->push_back(Vec2d(end_x, end_y));
  boundary_points->erase(boundary_points->begin(),
                         boundary_points->begin() + start_index);  // NOLINT
  boundary_points->insert(boundary_points->begin(), Vec2d(start_x, start_y));
  ADEBUG << "boundary_size: " << boundary_points->size()
         << ",start_point, x: " << boundary_points->front().x()
         << ", y: " << boundary_points->front().y()
         << ", end_point, x: " << boundary_points->back().x()
         << ", y: " << boundary_points->back().y();
  return true;
}

double NaviLaneCentralConstructor::CalculateVec2dLength(
    const std::vector<Vec2d>& input_points) {
  if (input_points.size() < 2) {
    AERROR << "the size of input_points less than 2!";
    return 0.0;
  }
  double length = 0;
  for (size_t i = 0; i < input_points.size() - 2; i++) {
    length += input_points.at(i).DistanceTo(input_points.at(i + 1));
  }
  ADEBUG << "input_points length: " << length;
  return length;
}

Status NaviLaneCentralConstructor::LaneWidthCheck(
    std::vector<Vec2d>* left_points, std::vector<Vec2d>* right_points,
    const double min_lane_width, std::vector<Vec2d>* ori_left_points,
    std::vector<Vec2d>* ori_right_points, const double min_length) {
  ADEBUG << "befor check left_points size: " << left_points->size()
         << ", right_points size: " << right_points->size();
  ADEBUG << "ori left_points size: " << ori_left_points->size()
         << ", ori right_points size: " << ori_right_points->size();
  std::string msg = "LaneWidthCheck: ";
  double lane_width_average_points =
      left_points->begin()->y() - right_points->begin()->y();
  bool lane_width_state = true;
  if (lane_width_average_points > 5 || lane_width_average_points < 2.3) {
    lane_width_state = false;
  } else if (lane_width_average_points < 4.8 &&
             lane_width_average_points > 2.5) {
    lane_width_state = true;
  } else {
    lane_width_state = lane_width_state_last_;
  }
  lane_width_state_last_ = lane_width_debounce_.DealDebounce(lane_width_state);
  if (!lane_width_state_last_) {
    ADEBUG << "lane_width_is_not_ok";
    msg += "lane_width_max_or_min";
    return Status(common::ErrorCode::LOCALVIEW_MAP_LANE_ERROR, msg);
  }
  int zero_index = std::max(
      static_cast<int>(
          std::find_if(left_points->begin(), left_points->end(),
                       [](const Vec2d& point) { return point.x() >= 0; }) -
          left_points->begin()),
      0);

  double lane_width = 0;
  double end_x =
      std::min(ori_left_points->back().x(), ori_right_points->back().x());
  auto cut_index = left_points->size();
  for (size_t i = zero_index; i < left_points->size(); ++i) {
    lane_width = (*left_points)[i].y() - (*right_points)[i].y();
    // AERROR << "lane width[" << i << "]: " << lane_width;
    if (std::abs(lane_width - min_lane_width) > 0.2 * min_lane_width) {
      cut_index = i;
      end_x = std::min((*left_points)[i].x(), (*right_points)[i].x());
      ADEBUG << "zero_index:" << zero_index << ", cut_index:" << cut_index
             << ", end_x: " << end_x << ", lane_width:" << lane_width
             << ", min_lane_width: " << min_lane_width;
      msg += " lane_width jump; ";
      break;
    }
  }

  if (end_x < min_length) {
    ADEBUG << "after cut lane with min_lane_width,lane length is too low!!! "
              "end_x: "
           << end_x;
    msg += "end_x err;";
    return Status(common::ErrorCode::LOCALVIEW_MAP_LANE_ERROR, msg);
  }

  for (size_t i = ori_left_points->size() - 1; ori_left_points->back().x() > 0;
       --i) {
    if ((*ori_left_points)[i].x() > end_x) {
      ori_left_points->pop_back();
    } else {
      break;
    }
  }
  for (size_t i = ori_right_points->size() - 1;
       ori_right_points->back().x() > 0; --i) {
    if ((*ori_right_points)[i].x() > end_x) {
      ori_right_points->pop_back();
    } else {
      break;
    }
  }
  left_points->resize(cut_index);
  right_points->resize(cut_index);
  ADEBUG << "after check left_points size: " << left_points->size()
         << ", right_points size: " << right_points->size();
  ADEBUG << "ori left_points size: " << ori_left_points->size()
         << ", ori right_points size: " << ori_right_points->size();
  return Status::OK();
}

void NaviLaneCentralConstructor::RemoveRedundantPoint(
    std::vector<Vec2d>* left_points, std::vector<Vec2d>* right_points) {
  auto point_size = left_points->size() >= right_points->size()
                        ? right_points->size()
                        : left_points->size();
  left_points->resize(point_size);
  right_points->resize(point_size);
}

bool NaviLaneCentralConstructor::DoubleHasSameSign(double first,
                                                   double second) {
  return (ComparedToZero(first) >= 0 && ComparedToZero(second) >= 0) ||
         (ComparedToZero(first) < 0 && ComparedToZero(second) < 0);
}
}  // namespace planning
}  // namespace TL
