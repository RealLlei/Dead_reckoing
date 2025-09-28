#ifndef PLANNING_COMMON_UTIL_UTIL_H
#define PLANNING_COMMON_UTIL_UTIL_H

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

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>

#include "planning/common/open_space_info.h"
#include "planning/common/path/discretized_path.h"
#include "planning/common/reference_line_info.h"
#include "proto/common/vehicle_state.pb.h"
#include "proto/perception/perception_freespace.pb.h"
#include "proto/routing/routing.pb.h"

namespace TL {
namespace planning {
namespace util {

bool IsVehicleStateValid(const TL::common::VehicleState& vehicle_state);

double GetADCStopDeceleration(TL::common::VehicleState* vehicle_state,
                              double adc_front_edge_s, double stop_line_s);

bool CheckStopSignOnReferenceLine(const ReferenceLineInfo& reference_line_info,
                                  const std::string& stop_sign_overlap_id);

bool CheckTrafficLightOnReferenceLine(
    const ReferenceLineInfo& reference_line_info,
    const std::string& traffic_light_overlap_id);

bool CheckInsidePnCJunction(const ReferenceLineInfo& reference_line_info);

void GetFilesByPath(const boost::filesystem::path& path,
                    std::vector<std::string>* files);
bool IsMatchedHdmapAndRouting(const TL::common::Header& routing_header,
                              const TL::common::Header& hdmap_header);

/**
 * Retrieves the free space segments from the given `freespace_out_array`
 * and populates the `freespace_segments` vector with the results.
 *
 * @param freespace_out_array A shared pointer to a constant `FreeSpaceOutArray` object
 *                           that contains the free space segments.
 * @param freespace_segments A pointer to a `std::vector` of `FreeSpaceSegment` objects
 *                           that will be populated with the retrieved segments.
 *
 * @return `true` if the free space segments were successfully retrieved and populated
 *         into the `freespace_segments` vector, `false` otherwise.
*/
bool GetFreeSpaceSegments(
    const std::shared_ptr<const perception::FreeSpaceOutArray>&
        freespace_out_array,
    std::vector<FreeSpaceSegment>* freespace_segments);

/**
 * Updates the free space segments by path.
 *
 * @param discretized_path the discretized path
 * @param freespace_out_array the shared pointer to the FreeSpaceOutArray
 * @param freespace_segments the vector of FreeSpaceSegment
 *
 * @throws ErrorType description of error
 */
void UpdateFreeSpaceSegmentsByPath(
    const DiscretizedPath& discretized_path,
    const std::shared_ptr<const perception::FreeSpaceOutArray>&
        freespace_out_array,
    std::vector<FreeSpaceSegment>* freespace_segments);
/**
 * Checks if the given parking lot array has an optional parking lot.
 *
 * @param parking_lot_out_array The array of parking lots to check.
 *
 * @return True if the array has an optional parking lot, false otherwise.
 */
bool IsHasOptParkLot(
    const perception::ParkingLotOutArray& parking_lot_out_array);

/**
 * Checks if the ego vehicle is in a parking lot.
 *
 * @param parking_lot_out_array the array of parking lot outputs
 * @param ego_point the position of the ego vehicle
 *
 * @return true if the ego vehicle is in a parking lot, false otherwise
 */
bool IsEgoInParkLot(const perception::ParkingLotOutArray& parking_lot_out_array,
                    const common::math::Vec2d& ego_point);

}  // namespace util
}  // namespace planning
}  // namespace TL

#endif  // PLANNING_COMMON_UTIL_UTIL_H
