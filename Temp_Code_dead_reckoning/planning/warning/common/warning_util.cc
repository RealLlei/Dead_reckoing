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

// fusion object type -->warning object type
uint32_t ConvertObjType(uint32_t fusion_obj_type) {
  uint32_t waring_obj_type = LBS_EM_GEN_OBJECT_CLASS_RESERVED;
  switch (fusion_obj_type) {
    case (perception::PerceptionObstacle::ST_PEDESTRIAN):
      waring_obj_type = LBS_EM_GEN_OBJECT_CLASS_PED;
      break;
    case (perception::PerceptionObstacle::ST_BICYCLE):
      waring_obj_type = LBS_EM_GEN_OBJECT_CLASS_BICYCLE;
      break;
    case (perception::PerceptionObstacle::ST_CAR):
    case (perception::PerceptionObstacle::ST_VAN):
    case (perception::PerceptionObstacle::ST_MINIBUS):
    case (perception::PerceptionObstacle::ST_PICKUP):
    case (perception::PerceptionObstacle::ST_AMBULANCE):
    case (perception::PerceptionObstacle::ST_POLICECAR):
      waring_obj_type = LBS_EM_GEN_OBJECT_CLASS_CAR;
      break;
    case (perception::PerceptionObstacle::ST_TRUCK):
    case (perception::PerceptionObstacle::ST_BIG_TRUCK):
      waring_obj_type = LBS_EM_GEN_OBJECT_CLASS_TRUCK;
      break;
    case (perception::PerceptionObstacle::ST_BUS):
    case (perception::PerceptionObstacle::ST_MIDDLE_BUS):
      waring_obj_type = LBS_EM_GEN_OBJECT_CLASS_WIDE;
      break;
    case (perception::PerceptionObstacle::ST_MOTORCYCLE):
    case (perception::PerceptionObstacle::ST_ELETRICBICYCLE):
      waring_obj_type = LBS_EM_GEN_OBJECT_CLASS_MOTOCYCLE;
      break;
    // case (perception::PerceptionObstacle::TRICYCLE):
    //   reqPorts.GenObjList.aObject[i].Attributes.eClassification_nu =
    //       LBS_EM_GEN_OBJECT_CLASS_CAR;
    // case (perception::PerceptionObstacle::CONE):
    //   reqPorts.GenObjList.aObject[i].Attributes.eClassification_nu =
    //       LBS_EM_GEN_OBJECT_CLASS_CAR;
    // case (perception::PerceptionObstacle::BAN):
    //   reqPorts.GenObjList.aObject[i].Attributes.eClassification_nu =
    //       LBS_EM_GEN_OBJECT_CLASS_CAR;
    // case (perception::PerceptionObstacle::STOPBAR):
    //   reqPorts.GenObjList.aObject[i].Attributes.eClassification_nu =
    //       LBS_EM_GEN_OBJECT_CLASS_CAR;
    // case (perception::PerceptionObstacle::SPEEDHUMP):
    //   reqPorts.GenObjList.aObject[i].Attributes.eClassification_nu =
    //       LBS_EM_GEN_OBJECT_CLASS_CAR;
    // case (perception::PerceptionObstacle::SPECIAL):
    //   reqPorts.GenObjList.aObject[i].Attributes.eClassification_nu =
    //       LBS_EM_GEN_OBJECT_CLASS_CAR;
    // case (perception::PerceptionObstacle::Minibus):
    //   reqPorts.GenObjList.aObject[i].Attributes.eClassification_nu =
    //       LBS_EM_GEN_OBJECT_CLASS_CAR;
    // case (perception::PerceptionObstacle::SMALLANI):
    //   reqPorts.GenObjList.aObject[i].Attributes.eClassification_nu =
    //       LBS_EM_GEN_OBJECT_CLASS_TRUCK;
    // case (perception::PerceptionObstacle::SHUIMA):
    //   reqPorts.GenObjList.aObject[i].Attributes.eClassification_nu =
    //       LBS_EM_GEN_OBJECT_CLASS_PED;
    // case (perception::PerceptionObstacle::SANJIAOBAN):
    //   reqPorts.GenObjList.aObject[i].Attributes.eClassification_nu =
    //       LBS_EM_GEN_OBJECT_CLASS_MOTOCYCLE;
    // case (perception::PerceptionObstacle::LONGMENJIA):
    //   reqPorts.GenObjList.aObject[i].Attributes.eClassification_nu =
    //       LBS_EM_GEN_OBJECT_CLASS_MOTOCYCLE;
    // case (perception::PerceptionObstacle::ZHUZI):
    //   reqPorts.GenObjList.aObject[i].Attributes.eClassification_nu =
    //       LBS_EM_GEN_OBJECT_CLASS_MOTOCYCLE;
    // case (perception::PerceptionObstacle::GOUWUCHE):
    //   reqPorts.GenObjList.aObject[i].Attributes.eClassification_nu =
    //       LBS_EM_GEN_OBJECT_CLASS_MOTOCYCLE;
    // case (perception::PerceptionObstacle::YINGERTUICHE):
    //   reqPorts.GenObjList.aObject[i].Attributes.eClassification_nu =
    //       LBS_EM_GEN_OBJECT_CLASS_MOTOCYCLE;
    // case (perception::PerceptionObstacle::FANGUANGLIZHU):
    //   reqPorts.GenObjList.aObject[i].Attributes.eClassification_nu =
    //       LBS_EM_GEN_OBJECT_CLASS_MOTOCYCLE;
    // case (perception::PerceptionObstacle::LUNDANG):
    //   reqPorts.GenObjList.aObject[i].Attributes.eClassification_nu =
    //       LBS_EM_GEN_OBJECT_CLASS_MOTOCYCLE;
    // case (perception::PerceptionObstacle::DISUO):
    //   reqPorts.GenObjList.aObject[i].Attributes.eClassification_nu =
    //       LBS_EM_GEN_OBJECT_CLASS_MOTOCYCLE;
    default:
      waring_obj_type = LBS_EM_GEN_OBJECT_CLASS_RESERVED;
      break;
  }
  return waring_obj_type;
}

uint8_t ConvertObjMotionType(uint8 fusion_obj_motion_type) {
  uint8_t motion_type = EM_GEN_OBJECT_DYN_PROPERTY_UNKNOWN;
  switch (fusion_obj_motion_type) {
    case 0:  // UNKNOWN
      motion_type = EM_GEN_OBJECT_DYN_PROPERTY_UNKNOWN;
      break;
    case 1:  // MOVING_EGODIRECTION_DRIVING
      motion_type = EM_GEN_OBJECT_DYN_PROPERTY_MOVING;
      break;

    case 2:  // MOVING_EGODIRECTION_STOPPED
      motion_type = EM_GEN_OBJECT_DYN_PROPERTY_STOPPED;
      break;

    case 3:  // MOVING_EGODIRECTION_REVERSING
      motion_type = EM_GEN_OBJECT_DYN_PROPERTY_UNKNOWN;
      break;

    case 4:  // MOVING_ONCOMING
      motion_type = EM_GEN_OBJECT_DYN_PROPERTY_ONCOMING;
      break;

    case 5:  // MOVING_CROSSING
      motion_type = EM_GEN_OBJECT_DYN_PROPERTY_CROSSING_MOVING;
      break;

    case 6:  // STATIONARY
      motion_type = EM_GEN_OBJECT_DYN_PROPERTY_STATIONARY;
      break;

    default:
      motion_type = EM_GEN_OBJECT_DYN_PROPERTY_UNKNOWN;
      break;
  }

  return motion_type;
};

sint8 CalculateAdjacentLaneCount(const uint32_t lane_count,
                                 const int32_t left_lane_line_seq,
                                 const int32_t right_lane_line_seq) {
  uint8_t adjacent_lane_count = 0;
  if (lane_count < 2) {
    adjacent_lane_count = 0;
  }
  // else if (((left_lane_line_seq == -1) || (right_lane_line_seq == 2)) &&
  //           (lane_count == 2))
  else if (lane_count == 2) {
    adjacent_lane_count = 1;
  } else {
    adjacent_lane_count = 2;
  }
  return adjacent_lane_count;
}

float32 CalculateLaneWidth(double left_lane_position, double left_heading,
                           double right_lane_position, double right_heading) {
  float32 lane_width = 0;
  double cos_left = 1 / SafeDiv(1 + left_heading * left_heading);
  double cos_right = 1 / SafeDiv(1 + right_heading * right_heading);

  lane_width =
      abs((left_lane_position * cos_left) - (right_lane_position * cos_right));
  lane_width = TUE_CML_MinMax(LBS_INPUT_LANE_WIDTH_MIN,
                              LBS_INPUT_LANE_WIDTH_MAX, lane_width);
  return lane_width;
}

double RoadCurveRadius(double right_lane_position, double left_lane_position,
                       double right_lane_curve_radius,
                       double left_lane_curve_radius) {
  double rat =
      -left_lane_position / SafeDiv(right_lane_position - left_lane_position);
  rat = fmin(rat, 1);
  rat = fmax(rat, 0);
  double radius =
      right_lane_curve_radius * rat + left_lane_curve_radius * (1 - rat);
  if (right_lane_curve_radius * left_lane_curve_radius < 0.0) {
    return 1 / TUE_C_F32_DELTA;
  }
  return radius;
}

void CalculateEgoLaneCurvature(
    const double right_lane_position, const double left_lane_position,
    const double right_lane_kappa, const double left_lane_kappa,
    const double right_lane_kappa_dev, const double left_lane_kappa_dev,
    const double right_lane_heading, const double left_lane_heading,
    double* ego_lane_kappa, double* ego_lane_kappa_dev,
    double* ego_lane_heading) {
  double rat =
      -left_lane_position / SafeDiv(right_lane_position - left_lane_position);
  rat = fmin(rat, 1);
  rat = fmax(rat, 0);
  *ego_lane_kappa = right_lane_kappa * rat + left_lane_kappa * (1 - rat);
  *ego_lane_kappa =
      TUE_CML_MinMax(right_lane_kappa, left_lane_kappa, *ego_lane_kappa);
  *ego_lane_kappa_dev =
      right_lane_kappa_dev * rat + left_lane_kappa_dev * (1 - rat);
  *ego_lane_kappa_dev = TUE_CML_MinMax(
      right_lane_kappa_dev, left_lane_kappa_dev, *ego_lane_kappa_dev);
  *ego_lane_heading = right_lane_heading * rat + left_lane_heading * (1 - rat);
  *ego_lane_heading =
      TUE_CML_MinMax(right_lane_heading, left_lane_heading, *ego_lane_heading);
}

uint8_t GetNearestLaneWarning(const std::shared_ptr<LocalView>& localview,
                              const TL::common::PointENU& point,
                              hdmap::LaneInfoConstPtr nearest_lane_w,
                              double* nearest_s, double* nearest_l) {
  // LaneType: {NONE = 1;CITY_DRIVING = 2;BIKING = 3;SIDEWALK = 4;
  // PARKING = 5;SHOULDER = 6; HIGHWAY_DRIVING = 7; LEFT_TURN_WAITING_ZONE
  // = 8; EMERGENCY_LANE = 9; ROUNDABOUT = 10;}
  uint8 type_w = 0;
  localview->GetHDMapPtr()->GetNearestLane(point, &nearest_lane_w, nearest_s,
                                           nearest_l);

  if (nearest_lane_w != nullptr) {
    type_w = nearest_lane_w->lane().type();
    // nearest_lane_w->GetWidth(nearest_s, &left_width, &right_width);
    // road_width_w = left_width + right_width;
    // reqPorts.Road.fLaneWidth_met = road_width_w;
  }

  return type_w;
}

}  // namespace WarningUtil
}  // namespace warning
}  // namespace planning
}  // namespace TL
