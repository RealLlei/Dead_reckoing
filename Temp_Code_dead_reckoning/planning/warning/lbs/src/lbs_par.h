#pragma once
#include <math.h>
#include "planning/warning/lbs/common/tue_common_libs.h"
#ifndef LBS_PAR_H
#define LBS_PAR_H

#define WARNING_OBS_MAX_NUMBERS 80
#define LBS_INPUT_OBJECT_U16ID_MAX (600u)

#define LBS_INPUT_ROADTYPE_CONFIDENCE (0.9999f)
#define LBS_INPUT_ROADTYPE_UKNOWN_0_UINT (0u)

#define LBS_INPUT_LANE_WIDTH_MAX (6.0f)
#define LBS_INPUT_LANE_WIDTH_MIN (3.50f)

// constant value; Believe the information from HDMap -JO
#define LBS_INPUT_DEFUALT_0_FLOAT \
  (0.0F)  // constant value 0 for no use FLOAT value -JO
#define LBS_LCA_TTC_BaseVel (1U);
#define LBS_LCA_WarnEnbale_MinVel (20.0 / 3.6f)
#define LBS_LCA_WarnDisenbale_MinVel (15.0 / 3.6f)

#define LBS_LCA_DIS_TTCsTHRESHOLD (7.5f)     // *product max_velocity
#define LBS_LCA_TTCsTHRESHOLD_LOWSPD (2.5f)  // *product max_velocity = 10m/s
#define LBS_LCA_TTCsTHRESHOLD_MIDSPD (3.0f)  // --JO  max_velocity = 15m/s
#define LBS_LCA_TTCsTHRESHOLD_HIGSPD (3.5f)  // --JO  max_velocity = 20m/s
#define LBS_LCA_TTCsTHRESHOLD_HYS (1.0f)     // --JO

#define LBS_INPUT_DEFUALT_0_UINT \
  (0u)  // constant value for no use uintxx value -JO
#define LBS_INPUT_NO_USE_UINT \
  (6u)  // constant value for no use uintxx value -JO

#define LBS_INPUT_NO_USE_FLOAT \
  (0.0000000001f)  // constant value for no use FLOAT value -JO
#define LBS_INPUT_DEFUALT_1000_FLOAT \
  (1000.0f)  // constant value for no use FLOAT value -JO

// constant value of vehicle length / width / wheel base -JO
// vehicle length: 4.933   width: 2.11   wheel_base: 2.8448
#define LBS_INPUT_VEHICLE_LENGTH (4.952f)
#define LBS_INPUT_VEHICLE_WIDTH (1.98f)
#define LBS_INPUT_VEHICLE_WHEEL_BASE (2.96f)
#define LBS_INPUT_VEHICLE_STEER_RATIO (15.17f)
#define LBS_INPUT_VEHICLE_MAX_DRIVE_CURVE_RADIUS (10000.0f)
#define LBS_INPUT_VEHICLE_OVERHANG_FRONT (0.934f)

// SOT delay threshold:1.5
#define LBS_INPUT_SOT_DELAY_THRESHOLD (1.5f)
// SOT OBJ  cut off speed: -4m/s
#define LBS_INPUT_SOT_CUTOFF_SPEED (-5.0f)  // *product -4.0f --> -5.0f
// SOT Min warning duration:2 HZ
#define LBS_INPUT_SOT_MIN_WARN_DURATION (2.0f)  // *product 0.2f -->2f

//  Min velocity enable/ disable BSD function :5.5
#define LBS_INPUT_BSD_EN_DISABLE_VELOCITY (15 / 3.6)  // *product

//  Min velocity enable/ enable BSD function :5.5
#define LBS_INPUT_BSD_EN_ENABLE_VELOCITY (20 / 3.6)  //  *product

//  BSD warning zone X direction min position : -9m //  -9->7  JO
#define LBS_INPUT_BSD_ZONE_MIN_X                                      \
  (-LBS_INPUT_VEHICLE_WHEEL_BASE / 2 - LBS_INPUT_VEHICLE_LENGTH / 2 - \
   3)  // *product

//  BSD warning zone X direction max position : -2m//  -2->-0.961  Yi
#define LBS_INPUT_BSD_ZONE_MAX_X -0.961  // *product

//  BSD warning zone Y direction min position : 0.5m
#define LBS_INPUT_BSD_ZONE_MIN_Y \
  (0.5 + LBS_INPUT_VEHICLE_WIDTH / 2)  // *product1.49

//  BSD warning zone Y direction max position : 3.5m
#define LBS_INPUT_BSD_ZONE_MAX_Y \
  (3.75 + LBS_INPUT_VEHICLE_WIDTH / 2)  // *product 4.74

//  BSD warning zone hysteresis X/Y direction: 1m
#define LBS_INPUT_BSD_ZONE_HYST (1.0f)

// constant value for Obstacle length ahead of the track position,uint:m  -JO
#define LBS_INPUT_LENGTH_FRONT_5 (5u)
// constant value for Obstacle length behind of the track position,uint:m  -JO
#define LBS_INPUT_LENGTH_REAR_3 (3u)

// constant value for Obstacle is measured (0) or predicted(1)  -JO
#define LBS_INPUT_OBJ_IS_MEASURED (0u)
#define LBS_INPUT_OBJ_IS_PREDICTED (1u)

// constant value for Obstacle dynamic property: moving 0, oncoming 1,
// stationary 2.   -JO
#define LBS_INPUT_OBJ_DYNAMIC_PROPERTY (0u)
// constant value for Obstacle dynamic property and classic confidence.   -JO
#define LBS_INPUT_OBJ_CONFIDENCE (100u)

// LCA Zone Parameter
#define LBS_INPUT_LCA_Zone_MID_X (-40);
#define LBS_INPUT_LCA_Zone_MIN_X (-60);
#define LBS_INPUT_LCA_Zone_MIN_NEAR_Y (0.6);
#define LBS_INPUT_LCA_Zone_MAX_NEAR_Y (4.74);
#define LBS_INPUT_LCA_Zone_MIN_FAR_Y (1.5);
#define LBS_INPUT_LCA_Zone_MAX_FAR_Y (4.74);

/*****************************************************************************
  EM RADAR
*****************************************************************************/
#ifndef EM_GEN_OBJ_MT_STATE_NEW
#define EM_GEN_OBJ_MT_STATE_NEW (1u)
#endif

#ifndef EM_GEN_OBJ_MT_STATE_PREDICTED
#define EM_GEN_OBJ_MT_STATE_PREDICTED (3u)
#endif
/* class unclassified minimum, maximum and default dimensions length and width
 * treated same */

#define SIG_STATUS_OK 1

#define LBS_EM_RADAR_ENUM_INVALID 999
#define LBS_EM_RADAR_ENUM_ARS308 0
#define LBS_EM_RADAR_ENUM_YILAIDA 1
#define LBS_EM_RADAR_ENUM_MUNIU 2

#define LBS_SRR_RADAR_TYPE LBS_EM_RADAR_ENUM_INVALID

// Sensor Mounting Parameter
#define LBS_SensorLeft_LatPos 0.0f;         //  0.5->0.0  JO
#define LBS_SensorLeft_LongPos 0.0f;        // 1.2 -> 0.0  JO
#define LBS_SensorLeft_VertPos 0.0f;        // 0.2 -> 0.0  JO
#define LBS_SensorLeft_LongPosToCoG 0.0f;   // 1.0 -> 0.0 JO
#define LBS_SensorLeft_PitchAngle 0.0f;     // 0.1 -> 0.0 JO
#define LBS_SensorLeft_Orientation 0.0f;    // 0.1 -> 0.0 JO
#define LBS_SensorLeft_RollAngle_rad 0.0f;  // 0.1 -> 0.0 JO
#define LBS_SensorLeft_YawAngle 0.0f;       // 0.1 -> 0.0 JO

#define LBS_SensorRight_LatPos 0.0f;         //  0.5->0.0  JO
#define LBS_SensorRight_LongPos 0.0f;        // 1.2 -> 0.0  JO
#define LBS_SensorRight_VertPos 0.0f;        // 0.2 -> 0.0  JO
#define LBS_SensorRight_LongPosToCoG 0.0f;   // 1.0 -> 0.0 JO
#define LBS_SensorRight_PitchAngle 0.0f;     // 0.1 -> 0.0 JO
#define LBS_SensorRight_Orientation 0.0f;    // 0.1 -> 0.0 JO
#define LBS_SensorRight_RollAngle_rad 0.0f;  // 0.1 -> 0.0 JO
#define LBS_SensorRight_YawAngle 0.0f;       // 0.1 -> 0.0 JO

/*****************************************************************************
  CONSTS
*****************************************************************************/
#define LBS_TTC_INVALID TUE_C_F32_VALUE_INVALID

// LowPassFilter alpha defines
#define LBS_LPF_TTCFILTERED_ALPHA (0.25f)
#define LBS_LPF_VRELXY_ALPHA (0.1f)

#define LBS_UPDATERATE_FILTER_UP (0.05f)
#define LBS_UPDATERATE_FILTER_DOWN (0.025f)
#define LBS_ASSOCPROB_FILTER_UP (0.05f)
#define LBS_ASSOCPROB_FILTER_DOWN (0.015f)

#define LBS_MIN_UPDATERATE_TTC_ACCEL \
  (0.8f)  // 0.75 ---> 0.8 the min update rate threshold for TTC calculate

#define LBS_OBJSEL_TARGETRANGE_MAX (71.0f)
#define LBS_OBJSEL_VEGO_MIN (-100.0f)
#define LBS_OBJSEL_VEGO_MAX (100.0f)
#define LBS_OBJSEL_VTARGETMIN (0.83f)
#define LBS_OBJSEL_XMAX_BREAKTHROUGH (7.0f)
#define LBS_OBJSEL_XMIN_BREAKTHROUGH (0.0f)

#define LBS_MAXSPD_OVERGND_LIFETIME_THRESH (2u)  // 100->2
#define LBS_MAXSPD_OVERGND_FASTILTER_MIN_FMRTE (1.0f)
#define LBS_MAXSPD_OVERGND_FASTFILTER \
  (0.01f)  // max over ground speed filter up alpha
#define LBS_MAXSPD_OVERGND_SLOWFILTER \
  (0.00001f)  // max over ground speed filter down alpha
#define LBS_MAXSPD_OVERGND_ROADTYPE_CONF_MIN (0.7f)

#define LBS_FIRSTDYNPROPERTY_POE_THRESH (0.9f)
#define LBS_FIRSTDYNPROPERTY_LIFETIME_THRESH (20u)

#define LBS_FIRSTDETECTDIST_LIFETIME_THRESH (1u)

#define EM_GEN_OBJ_MT_STATE_DELETED (0u)
#define EM_GEN_OBJ_MT_STATE_MEASURED (2u)

#define EM_GEN_OBJECT_DYN_PROPERTY_MOVING (0u)
#define EM_GEN_OBJECT_DYN_PROPERTY_STATIONARY (1u)
#define EM_GEN_OBJECT_DYN_PROPERTY_ONCOMING (2u)
#define EM_GEN_OBJECT_DYN_PROPERTY_STATIONARY_CANDIDATE (3u)  // no use
#define EM_GEN_OBJECT_DYN_PROPERTY_UNKNOWN (4u)
#define EM_GEN_OBJECT_DYN_PROPERTY_CROSSING_STATIONARY (5u)  // no use
#define EM_GEN_OBJECT_DYN_PROPERTY_CROSSING_MOVING (6u)
#define EM_GEN_OBJECT_DYN_PROPERTY_STOPPED (7u)

#define LBS_EM_GEN_OBJECT_CLASS_POINT (0u)
#define LBS_EM_GEN_OBJECT_CLASS_CAR (1u)
#define LBS_EM_GEN_OBJECT_CLASS_TRUCK (2u)
#define LBS_EM_GEN_OBJECT_CLASS_PED (3u)
#define LBS_EM_GEN_OBJECT_CLASS_MOTOCYCLE (4u)
#define LBS_EM_GEN_OBJECT_CLASS_BICYCLE (5u)
#define LBS_EM_GEN_OBJECT_CLASS_WIDE (6u)
#define LBS_EM_GEN_OBJECT_CLASS_RESERVED (7u)

/* definition of classification dependent minimum, maximum and default
 * dimensions */
/* class car minimum, maximum and default dimensions */
#define LBS_EM_GEN_CLASS_CAR_MIN_LENGTH 3.0f
#define LBS_EM_GEN_CLASS_CAR_MAX_LENGTH 7.0f
#define LBS_EM_GEN_CLASS_CAR_DEFAULT_LENGTH 5.0f
#define LBS_EM_GEN_CLASS_CAR_MIN_WIDTH 1.8f
#define LBS_EM_GEN_CLASS_CAR_MAX_WIDTH 2.2f
#define LBS_EM_GEN_CLASS_CAR_DEFAULT_WIDTH 1.8f
/* class truck minimum, maximum and default dimensions */
#define LBS_EM_GEN_CLASS_TRUCK_MIN_LENGTH 5.0f
#define LBS_EM_GEN_CLASS_TRUCK_MAX_LENGTH 25.0f
#define LBS_EM_GEN_CLASS_TRUCK_DEFAULT_LENGTH 18.0f
#define LBS_EM_GEN_CLASS_TRUCK_MIN_WIDTH 2.0f
#define LBS_EM_GEN_CLASS_TRUCK_MAX_WIDTH 2.8f
#define LBS_EM_GEN_CLASS_TRUCK_DEFAULT_WIDTH 2.5f
/* class pedestrian minimum, maximum and default dimensions length and width
 * treated same */
#define LBS_EM_GEN_CLASS_PED_MIN_DIMENSION 0.4f
#define LBS_EM_GEN_CLASS_PED_MAX_DIMENSION 0.8f
#define LBS_EM_GEN_CLASS_PED_DEFAULT_DIMENSION 0.6f
/* class motorcycle minimum, maximum and default dimensions */
#define LBS_EM_GEN_CLASS_MOTORCYCLE_MIN_LENGTH 2.0f
#define LBS_EM_GEN_CLASS_MOTORCYCLE_MAX_LENGTH 4.0f
#define LBS_EM_GEN_CLASS_MOTORCYCLE_DEFAULT_LENGTH 2.5f
#define LBS_EM_GEN_CLASS_MOTORCYCLE_MIN_WIDTH 0.5f
#define LBS_EM_GEN_CLASS_MOTORCYCLE_MAX_WIDTH 1.2f
#define LBS_EM_GEN_CLASS_MOTORCYCLE_DEFAULT_WIDTH 1.0f
/* class bicile minimum, maximum and default dimensions */
#define LBS_EM_GEN_CLASS_BICICLE_MIN_LENGTH \
  LBS_EM_GEN_CLASS_MOTORCYCLE_MIN_LENGTH
#define LBS_EM_GEN_CLASS_BICICLE_MAX_LENGTH \
  LBS_EM_GEN_CLASS_MOTORCYCLE_MAX_LENGTH
#define LBS_EM_GEN_CLASS_BICICLE_DEFAULT_LENGTH \
  LBS_EM_GEN_CLASS_MOTORCYCLE_DEFAULT_LENGTH
#define LBS_EM_GEN_CLASS_BICICLE_MIN_WIDTH LBS_EM_GEN_CLASS_MOTORCYCLE_MIN_WIDTH
#define LBS_EM_GEN_CLASS_BICICLE_MAX_WIDTH LBS_EM_GEN_CLASS_MOTORCYCLE_MAX_WIDTH
#define LBS_EM_GEN_CLASS_BICICLE_DEFAULT_WIDTH \
  LBS_EM_GEN_CLASS_MOTORCYCLE_DEFAULT_WIDTH
/* class point minimum, maximum and default dimensions length and width treated
 * same */
#define LBS_EM_GEN_CLASS_POINT_MIN_DIMENSION 0.1f
#define LBS_EM_GEN_CLASS_POINT_MAX_DIMENSION 0.5f
#define LBS_EM_GEN_CLASS_POINT_DEFAULT_DIMENSION 0.4f
/* class unclassified minimum, maximum and default dimensions length and width
 * treated same */
#define LBS_EM_GEN_CLASS_UNCLASSIFIED_MIN_DIMENSION 0.1f
#define LBS_EM_GEN_CLASS_UNCLASSIFIED_MAX_DIMENSION 50.0f
#define LBS_EM_GEN_CLASS_UNCLASSIFIED_DEFAULT_DIMENSION 50.0f

#define LBS_OSE_NUM_OF_BREAK_LINES 2u
#define LBS_OSE_QUALITY_POE_MIN (0.6f)
#define LBS_OSE_MIN_LIFETIME_RELEVANT (2u)
#endif
