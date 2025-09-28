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

#include "planning/common/util/util.h"

#include <algorithm>
#include <cstddef>
#include <limits>
#include <memory>
#include <set>
#include <vector>
#include "common/configs/vehicle_config_helper.h"
#include "common/math/line_segment2d.h"
#include "common/math/polygon2d.h"
#include "common/math/vec2d.h"
#include "common/util/util.h"
#include "map/hdmap/path.h"
#include "planning/common/open_space_info.h"
#include "proto/perception/perception_freespace.pb.h"
#include "proto/perception/perception_parking_lot.pb.h"

namespace TL {
namespace planning {
namespace util {

using TL::common::VehicleState;
using TL::hdmap::PathOverlap;

bool IsVehicleStateValid(const VehicleState& vehicle_state) {
  return !(std::isnan(vehicle_state.x()) || std::isnan(vehicle_state.y()) ||
           std::isnan(vehicle_state.z()) ||
           std::isnan(vehicle_state.heading()) ||
           std::isnan(vehicle_state.kappa()) ||
           std::isnan(vehicle_state.linear_velocity()) ||
           std::isnan(vehicle_state.linear_acceleration()));
}

double GetADCStopDeceleration(TL::common::VehicleState* vehicle_state,
                              const double adc_front_edge_s,
                              const double stop_line_s) {
  if (vehicle_state == nullptr) {
    return 0.0;
  }
  double adc_speed = vehicle_state->linear_velocity();
  const double max_adc_stop_speed = common::VehicleConfigHelper::GetConfig()
                                        .vehicle_param()
                                        .max_abs_speed_when_stopped();
  if (adc_speed < max_adc_stop_speed) {
    return 0.0;
  }

  double stop_distance = 0;

  if (stop_line_s > adc_front_edge_s) {
    stop_distance = stop_line_s - adc_front_edge_s;
  }
  if (stop_distance < 1e-5) {
    return std::numeric_limits<double>::max();
  }
  return (adc_speed * adc_speed) / (2 * stop_distance);
}

/*
 * @brief: check if a stop_sign_overlap is still along reference_line
 */
bool CheckStopSignOnReferenceLine(const ReferenceLineInfo& reference_line_info,
                                  const std::string& stop_sign_overlap_id) {
  const std::vector<PathOverlap>& stop_sign_overlaps =
      reference_line_info.reference_line().map_path().stop_sign_overlaps();
  auto stop_sign_overlap_it =
      std::find_if(stop_sign_overlaps.begin(), stop_sign_overlaps.end(),
                   [&stop_sign_overlap_id](const PathOverlap& overlap) {
                     return overlap.object_id == stop_sign_overlap_id;
                   });
  return (stop_sign_overlap_it != stop_sign_overlaps.end());
}

/*
 * @brief: check if a traffic_light_overlap is still along reference_line
 */
bool CheckTrafficLightOnReferenceLine(
    const ReferenceLineInfo& reference_line_info,
    const std::string& traffic_light_overlap_id) {
  const std::vector<PathOverlap>& traffic_light_overlaps =
      reference_line_info.reference_line().map_path().signal_overlaps();
  auto traffic_light_overlap_it =
      std::find_if(traffic_light_overlaps.begin(), traffic_light_overlaps.end(),
                   [&traffic_light_overlap_id](const PathOverlap& overlap) {
                     return overlap.object_id == traffic_light_overlap_id;
                   });
  return (traffic_light_overlap_it != traffic_light_overlaps.end());
}

/*
 * @brief: check if ADC is till inside a pnc-junction
 */
bool CheckInsidePnCJunction(const ReferenceLineInfo& reference_line_info) {
  const auto adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const auto adc_back_edge_s = reference_line_info.AdcSlBoundary().start_s();

  hdmap::PathOverlap pnc_junction_overlap;
  reference_line_info.GetPnCJunction(adc_front_edge_s, &pnc_junction_overlap);
  if (pnc_junction_overlap.object_id.empty()) {
    return false;
  }

  static constexpr double kIntersectionPassDist = 2.0;  // unit: m
  const auto distance_adc_pass_intersection =
      adc_back_edge_s - pnc_junction_overlap.end_s;
  ADEBUG << "distance_adc_pass_intersection[" << distance_adc_pass_intersection
         << "] pnc_junction_overlap[" << pnc_junction_overlap.object_id
         << "] start_s[" << pnc_junction_overlap.start_s << "]";

  return distance_adc_pass_intersection < kIntersectionPassDist;
}

/*
 * @brief: get files at a path
 */
void GetFilesByPath(const boost::filesystem::path& path,  //NOLINT
                    std::vector<std::string>* files) {
  ACHECK(files);
  if (!boost::filesystem::exists(path)) {
    return;
  }
  if (boost::filesystem::is_regular_file(path)) {
    AINFO << "Found record file: " << path.c_str();
    files->push_back(path.c_str());
    return;
  }
  if (boost::filesystem::is_directory(path)) {
    for (auto& entry : boost::make_iterator_range(
             boost::filesystem::directory_iterator(path), {})) {
      GetFilesByPath(entry.path(), files);
    }
  }
}

/*
 * @brief: check hdmap and routing matching, only checking module_name of the
 * header. module_name of hdmap and routing must ending by "_hdmap or
 * _routing",and only compare the character string before
 * them.Example:The module names of "from_file_hdmap" and "from_file_routing"
 * are equal and will return true.
 */

bool IsMatchedHdmapAndRouting(const TL::common::Header& routing_header,
                              const TL::common::Header& hdmap_header) {
  std::string routing_name = routing_header.frame_id();
  std::string hdmap_name = hdmap_header.frame_id();
  if (routing_name.empty() || hdmap_name.empty()) {
    AERROR << "Routing_name or hdmap_name are empty,"
           << " routing_name: " << routing_name
           << ", hdmap_name: " << hdmap_name;
    return false;
  }
  std::size_t routing_position = routing_name.find_last_of('_');
  std::size_t hdmap_position = hdmap_name.find_last_of('_');
  if (routing_position == std::string::npos ||
      hdmap_position == std::string::npos) {
    AERROR << "Routing_name or hdmap_name can ont find _ "
           << ", routing_name: " << routing_name
           << ", hdmap_name: " << hdmap_name;
    return false;
  }
  std::string& routing_name_after_cut = routing_name.erase(routing_position);
  std::string& hdmap_name_after_cut = hdmap_name.erase(hdmap_position);
  const auto ret = routing_name_after_cut == hdmap_name_after_cut;
  if (!ret) {
    AERROR << "routing_name: " << routing_name << ", hdmap_name: " << hdmap_name
           << ", routing_name_after_cut: " << routing_name_after_cut
           << ", hdmap_name_after_cut: " << hdmap_name_after_cut;
  }
  return ret;
}

bool GetFreeSpaceSegments(
    const std::shared_ptr<const perception::FreeSpaceOutArray>&
        freespace_out_array,
    std::vector<FreeSpaceSegment>* const freespace_segments) {
  if (nullptr == freespace_out_array || nullptr == freespace_segments) {
    return false;
  }
  const auto& freespace_outs = freespace_out_array->freespace_out();
  for (int i = 0; i < freespace_outs.size(); ++i) {
    const auto& freespace_keypoints = freespace_outs[i].freespace_keypoint();
    const int freespace_keypoints_size = freespace_keypoints.size();
    if (freespace_keypoints_size < 2) {
      continue;
    }

    FreeSpaceSegment freespace_segment;
    for (int j = 1; j < freespace_keypoints_size; ++j) {
      freespace_segment.index_in_freespace_out = i;
      freespace_segment.index_in_keypoints = j;
      freespace_segment.cls_type = freespace_outs[i].cls();
      freespace_segment.height_type = freespace_outs[i].height_type();
      freespace_segment.sensor_type = freespace_outs[i].sensor_type();
      freespace_segment.isLinkObjFusion = freespace_outs[i].islinkobjfusion();
      freespace_segment.obstacleId = freespace_outs[i].obstacleid();
      freespace_segment.segment = common::math::LineSegment2d(
          {freespace_keypoints[j - 1].x(), freespace_keypoints[j - 1].y()},
          {freespace_keypoints[j].x(), freespace_keypoints[j].y()});
      freespace_segments->emplace_back(freespace_segment);
    }
  }

  return !freespace_segments->empty();
}

void UpdateFreeSpaceSegmentsByPath(
    const DiscretizedPath& discretized_path,
    const std::shared_ptr<const perception::FreeSpaceOutArray>&
        freespace_out_array,
    std::vector<FreeSpaceSegment>* const freespace_segments) {
  if (nullptr == freespace_segments) {
    return;
  }

  // init
  freespace_segments->clear();

  if (discretized_path.empty() || nullptr == freespace_out_array ||
      freespace_out_array->freespace_out_size() == 0) {
    return;
  }

  std::vector<FreeSpaceSegment> all_freespace_segments;
  if (!GetFreeSpaceSegments(freespace_out_array, &all_freespace_segments)) {
    return;
  }

  common::math::Polygon2d up_triangle;
  common::math::Polygon2d down_triangle;
  static constexpr double kTriangleHeight = 5.0;
  static constexpr double kTriangleBottomAngle = 80.0 / 180.0 * M_PI;
  std::set<size_t> valid_segment_index;
  for (const auto& path_point : discretized_path) {
    if (!TL::common::util::DoubleTriangleConstructor(
            path_point.x(), path_point.y(), path_point.theta(), kTriangleHeight,
            kTriangleBottomAngle, &up_triangle, &down_triangle)) {
      continue;
    }
    auto path_vec2d = common::math::Vec2d(path_point.x(), path_point.y());

    const common::math::Vec2d triangle_height_uint =
        common::math::Vec2d::CreateUnitVec2d(path_point.theta() + M_PI_2);
    const double half_bottom_edge_width =
        std::fabs(triangle_height_uint.CrossProd(up_triangle.points().at(1) -
                                                 path_vec2d));

    for (size_t i = 0; i < all_freespace_segments.size(); ++i) {
      const auto vec_a = all_freespace_segments[i].segment.start() - path_vec2d;
      const auto vec_b = all_freespace_segments[i].segment.end() - path_vec2d;
      const double cross_prod_a = triangle_height_uint.CrossProd(vec_a);
      const double cross_prod_b = triangle_height_uint.CrossProd(vec_b);
      const bool is_left_or_right_line_segment =
          std::fabs(cross_prod_a) > half_bottom_edge_width &&
          std::fabs(cross_prod_b) > half_bottom_edge_width &&
          cross_prod_a * cross_prod_b > 0.0;

      const double inner_prod_a = triangle_height_uint.InnerProd(vec_a);
      const double inner_prod_b = triangle_height_uint.InnerProd(vec_b);
      const bool is_up_or_down_line_segment =
          std::fabs(inner_prod_a) > kTriangleHeight &&
          std::fabs(inner_prod_b) > kTriangleHeight &&
          inner_prod_a * inner_prod_b > 0.0;

      if (is_left_or_right_line_segment || is_up_or_down_line_segment) {
        continue;
      }
      valid_segment_index.emplace(i);
    }
  }

  // start and end point, use polygon check
  static constexpr double kFilterDistance = 2.0;
  const auto ego_box_front =
      common::VehicleConfigHelper::GetBoundingBox(discretized_path.front());
  const auto ego_box_end =
      common::VehicleConfigHelper::GetBoundingBox(discretized_path.back());
  for (size_t i = 0; i < all_freespace_segments.size(); i++) {
    if (ego_box_front.DistanceTo(all_freespace_segments[i].segment) <
            kFilterDistance ||
        ego_box_end.DistanceTo(all_freespace_segments[i].segment) <
            kFilterDistance) {
      valid_segment_index.emplace(i);
    }
  }

  for (const auto index : valid_segment_index) {
    freespace_segments->emplace_back(all_freespace_segments[index]);
  }
}

bool IsHasOptParkLot(
    const perception::ParkingLotOutArray& parking_lot_out_array) {
  if (!parking_lot_out_array.has_opt_parking_seq() ||
      parking_lot_out_array.parking_lots().empty()) {
    return false;
  }

  const auto& parking_lots = parking_lot_out_array.parking_lots();
  const auto& opt_seq = parking_lot_out_array.opt_parking_seq();
  return std::find_if(parking_lots.begin(), parking_lots.end(),
                      [&](const auto& lot) {
                        return lot.parking_seq() == opt_seq;
                      }) != parking_lots.end();
}

bool IsEgoInParkLot(const perception::ParkingLotOutArray& parking_lot_out_array,
                    const common::math::Vec2d& ego_point) {
  if (parking_lot_out_array.parking_lots().empty()) {
    return false;
  }
  static constexpr int kParkingLotVertexNum = 4;

  auto is_ego_in_park_lot = [&ego_point](const perception::ParkingLotOut& lot) {
    auto point_size = lot.pts_enu_size();
    if (point_size < kParkingLotVertexNum) {
      return false;
    }

    std::vector<common::math::Vec2d> points(kParkingLotVertexNum);
    for (const auto& parking_lot_point : lot.pts_enu()) {
      int idx = 0;
      switch (parking_lot_point.position()) {
        case TL::perception::PSPoint_Position_TOP_LEFT: {
          idx = 0;
          break;
        }
        case TL::perception::PSPoint_Position_BOTTOM_LEFT: {
          idx = 1;
          break;
        }
        case TL::perception::PSPoint_Position_BOTTOM_RIGHT: {
          idx = 2;
          break;
        }
        case TL::perception::PSPoint_Position_TOP_RIGHT: {
          idx = 3;
          break;
        }
        case TL::perception::PSPoint_Position_STOP_LEFT: {
          idx = 4;
          break;
        }
        case TL::perception::PSPoint_Position_STOP_RIGHT: {
          idx = 5;
          break;
        }
        default: {
          break;
        }
      }

      if (idx < kParkingLotVertexNum) {
        points[idx] =
            Vec2d(parking_lot_point.point().x(), parking_lot_point.point().y());
      }
    }
    if (!common::math::Polygon2d::IsConvexPolygon(points)) {
      return false;
    }
    common::math::Polygon2d polygon(points);
    return polygon.IsPointIn(ego_point);
  };

  return std::any_of(parking_lot_out_array.parking_lots().begin(),
                     parking_lot_out_array.parking_lots().end(),
                     is_ego_in_park_lot);
}
}  // namespace util
}  // namespace planning
}  // namespace TL
