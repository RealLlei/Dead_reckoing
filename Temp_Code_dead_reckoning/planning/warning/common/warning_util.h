/******************************************************************************
 * Copyright 2022 The TL Authors. All Rights Reserved.
 *****************************************************************************/
#include "planning/localview/local_view.h"
#include "planning/warning/cta/cta.h"
#include "planning/warning/lbs/common/tue_common_libs.h"
#include "planning/warning/lbs/src/bsd/lbs_bsd_main.h"
#include "planning/warning/lbs/src/bsd/lbs_bsd_par.h"
#include "planning/warning/lbs/src/lbs_calculation.h"
#include "planning/warning/lbs/src/lbs_main.h"
#include "planning/warning/lbs/src/lbs_par.h"
#include "planning/warning/lbs/src/lbs_wrapper.h"
#include "planning/warning/lbs/src/rcw/lbs_rcw_calculation.h"
#include "planning/warning/lbs/src/rcw/lbs_rcw_main.h"

namespace TL {
namespace planning {
namespace warning {
namespace WarningUtil {

/**
 * @brief calculate road curve radius by means of linear interpolation
 *
 * @param right_lane_position the distance between right lane and ego vehicle
 * @param left_lane_position the distance between left lane and ego vehicle
 * @param right_lane_curve_radius the local curve radius of right lane
 * @param left_lane_curve_radius the local curve radius of left lane
 * @return road curve radius
 */
double RoadCurveRadius(double right_lane_position, double left_lane_position,
                       double right_lane_curve_radius,
                       double left_lane_curve_radius);
/**
 * @brief Get the Nearest Lane Warning object
 *
 * @param local_view the input of planning
 * @param point thepoint whose nearest lane is required
 * @param nearest_lane_w the nearest lane pointer
 * @param nearest_s the accumulated curve length corresponding to the point on
 * the nearest lane
 * @param nearest_l the lateral distance corresponding to the point on the
 * nearest lane
 * @return uint8_t lane type
 */
uint8_t GetNearestLaneWarning(const std::shared_ptr<LocalView>& local_view,
                              const TL::common::PointENU& point,
                              hdmap::LaneInfoConstPtr nearest_lane_w,
                              double* nearest_s, double* nearest_l);
/**
 * @brief  convert the obstacle type information to the code in the C program
 *
 * @param fusion_obj_motion_type obstacle motion type in fusion protocol
 * @return uint8_t obstacle motion type code
 */
uint8_t ConvertObjMotionType(uint8 fusion_obj_motion_type);

/**
 * @brief determine the number of adjacent lanes
 *
 * @param lane_count the lane count of the nearest lane
 * @param left_lane_line_seq the sequence number of the left lane
 * @param right_lane_line_seq the sequence number of the right lane
 * @return sint8 the number of adjacent lanes
 */
sint8 CalculateAdjacentLaneCount(const uint32_t lane_count,
                                 const int32_t left_lane_line_seq,
                                 const int32_t right_lane_line_seq);
/**
 * @brief calculate lane width when ego vehicle is not parallel to the lane
 *
 * @param left_lane_position the distance between left lane and ego vehicle
 * @param left_heading the angle between local left lane and ego vehicle
 * @param right_lane_position the distance between right lane and ego vehicle
 * @param right_heading the angle between local right lane and ego vehicle
 * @return float32 local lane width
 */
float32 CalculateLaneWidth(double left_lane_position, double left_heading,
                           double right_lane_position, double right_heading);
/**
 * @brief calculate road curve radius by means of linear interpolation
 *
 * @param right_lane_position the distance between right lane and ego vehicle
 * @param left_lane_position the distance between left lane and ego vehicle
 * @param right_lane_kappa the local curvature of right lane
 * @param left_lane_kappa the local curvature of left lane
 * @param right_lane_kappa_dev the derivative of the local curvature of right lane
 * @param left_lane_kappa_dev the derivative of the local curvature of left lane
 * @param right_lane_heading the local heading of right lane
 * @param left_lane_heading the local heading of left lane
 * @param ego_lane_c0 the 0 coefficient of ego lane
 * @param ego_lane_c1 the 1 coefficient of ego lane
 * @param ego_lane_heading the local heading of ego lane
 */
void CalculateEgoLaneCurvature(
    const double right_lane_position, const double left_lane_position,
    const double right_lane_kappa, const double left_lane_kappa,
    const double right_lane_kappa_dev, const double left_lane_kappa_dev,
    const double right_lane_heading, const double left_lane_heading,
    double* ego_lane_kappa, double* ego_lane_kappa_dev,
    double* ego_lane_heading);
/**
 * @brief convert the obstacle type information to the code in the C program
 * 
 * @param fusion_obj_type obstacle type in fusion protocol
 * @return uint32_t obstacle type code
 */
uint32_t ConvertObjType(uint32_t fusion_obj_type);

}  // namespace WarningUtil
}  // namespace warning
}  // namespace planning
}  // namespace TL
