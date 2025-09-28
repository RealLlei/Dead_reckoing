#pragma once
#ifndef LBS_SI_PAR_H
#define LBS_SI_PAR_H
#ifdef __cplusplus
extern "C" {
#endif
#define SI_DYNAMIC_LANE_BRACKET_EXTENSION 0
#define SI_LCA_ZONE_BORDER_EXTENSION 1
#define SI_EM_GEN_OBJ_MT_STATE_DELETED (0U)
/*****************************************************************************
  CONSTS
*****************************************************************************/
#ifndef C_MS_KMH
#define C_MS_KMH (0.277778F)
#endif

#define INITVALUE_BRACKETPOSITION (999.0F)
/*****************************************************************************
  ACC corridor Parameters
*****************************************************************************/
#define SI_ACC_TRAJECTORY_WIDTH_STAT (2.5F)
#define SI_ACC_TRAJECTORY_NARROWWIDTH_STAT (2.0F)

#define SI_PAR_MAX_DISTX_STAT_POINT_SMALL_SEEK (-50.0F)

/*****************************************************************************
  Bracket
*****************************************************************************/
//#define SI_BRARST_TTC_MIN_DISTTOCRS (5.0F)

#define SI_BRARST_MAX_RB_Y_OFFSET (10.0F)
#define SI_BRARST_MAX_RB_Y_CONF (0.3F)
#define SI_BRARST_MIN_LANES (1)
#define SI_BRARST_MIN_LANE_CONF (0.7F)

#define SI_BRARST_LANE_WIDTH_FAC (1.5F)
/*****************************************************************************
  SI defines for bracket extension/restriction
*****************************************************************************/

// Bracket restriction far distance
#define SI_BRACKET_RESTR_FAR_DIST_MIN (0.0F)
#define SI_BRACKET_RESTR_FAR_DIST_MAX (0.25F)
#define SI_BRACKET_RESTR_FAR_DIST_X_MIN (-80.0F)
#define SI_BRACKET_RESTR_FAR_DIST_X_MID (-60.0F)
#define SI_BRACKET_RESTR_FAR_DIST_X_MAX (-40.0F)
// Bracket restriction near distance
#define SI_BRACKET_RESTR_NEAR_DIST_X_MIN (-20.0F)
#define SI_BRACKET_RESTR_NEAR_DIST_X_MAX (-5.0F)
// Bracket restriction for curve scenarios
#define SI_MIN_CURVE_RADIUS_BRACKET_REST (25.0F)
#define SI_MIN_CR_BRARST_REAR (40.0F)
#define SI_MAX_CURVE_RADIUS_BRACKET_REST (125.0F)
#define SI_BRARST_CR_MIN_REST (0.75F)
#define SI_BRARST_CR_MIN_REST_REAR (0.5F)
#define SI_BRARST_CR_MAX_REST (0.0F)
#define SI_FRONT_MAX_CURVE_RADIUS_BRACKET_REST (500.0F)

#define SI_IN2OUTLANE_MAX_TRANSITIONTIME (42U)

#define SI_MIN_OBJECT_WIDTH (0.8F)
#define SI_MAX_OBJECT_WIDTH (2.55F)
#define SI_MIN_OBJECT_WIDTH_HALF \
  (0.4F)  // Assume minimum object width to be 0.80m
#define SI_MAX_OBJECT_WIDTH_HALF \
  (1.275F)  // Assume maximum object width to be 2.55m

#define SI_VREL_TO_TRAJ_FILTER_CONST (0.1F)
/*****************************************************************************
  Constants for lane widths used in corridor criteria
*****************************************************************************/
// The tracked objects maximum additional lane width
#define SI_HIGHWAYLANEWIDTH_TRACK_OFFSET_MAX (1.1F)
// The tracked objects minimum additional lane width
#define SI_HIGHWAYLANEWIDTH_TRACK_OFFSET_MIN (0.7F)

// The seek lane width corresponding to the ego of SI_MAX_SPEED_HIGHWAYLANEWIDTH
#define SI_HIGHWAYLANEWIDTH_SEEK_MAX (3.75F)
// The seek lane width corresponding to the ego of SI_MIN_SPEED_HIGHWAYLANEWIDTH
#define SI_HIGHWAYLANEWIDTH_SEEK_MIN (3.75F)

// The ego speed for maximum seek lane width on highways
#define SI_MAX_SPEED_HIGHWAYLANEWIDTH (160.0F * C_MS_KMH)// 44.4
// The ego speed for minimum seek lane width on highways
#define SI_MIN_SPEED_HIGHWAYLANEWIDTH (130.0F * C_MS_KMH)//36.1

/*****************************************************************************
  Constants for SIExtensionRoadBorder
*****************************************************************************/
#define SI_BRACKET_OFFSET_DEFAULT (50.0F)

#define SI_BRAEXT_RB_MIN_EGOSPEED (10.0F)
#define SI_BRAEXT_RB_MAX_YOFFSET (10.0F)
#define SI_BRAEXT_RB_LANE_CONF (0.9F)
#define SI_BRAEXT_RB_LANE_CONF_MIN (0.7F)
#define SI_BRAEXT_RB_FUSED_RB_CONF (0.5F)

#define SI_BRAEXT_RB_LANE_WIDTH_FAC (2.5F)

#define SI_BRAEXT_TTC_MIN_DISTTOCRS (5.0F)
#define SI_BRAEXT_TTC_MAX_DISTX (-1.0F)

#define SI_BRAEXT_MAX_RB_Y_OFFSET (10.0F)
#define SI_BRAEXT_NAX_RB_Y_CONF (0.3F)
#define SI_BRAEXT_MIN_LANES (1)
#define SI_BRAEXT_MIN_LANE_CONF (0.7F)

#define SI_BRAEXT_LANE_WIDTH_FAC (1.5F)

// Minimum 2nd lane occupancy threshold function of ego speed
#define SI_BRAEXT_OCC_TRSH_MIN_EGOSPEED (10.0F)
#define SI_BRAEXT_OCC_TRSH_MAX_EGOSPEED (25.0F)
#define SI_BRAEXT_OCC_TRSH_MIN_OCC (0.3F)
#define SI_BRAEXT_OCC_TRSH_MAX_OCC (0.15F)

// Bracket restriction function of dist X
#define SI_BRARST_MIN_DISTX (40.0F)
#define SI_BRARST_MAX_DISTX (80.0F)
#define SI_BRARST_MIN_RESTRICT (0.0F)
#define SI_BRARST_MAX_RESTRICT (0.5F)

/*****************************************************************************
  Constants for SIExtensionOwnlaneCriticalTTC
*****************************************************************************/
#define SI_DECELERATION_OWNLANE_EXT (3.0F)

/*****************************************************************************
  CONSTS
*****************************************************************************/

/*****************************************************************************
  New cut-in potential parameters
*****************************************************************************/
// Minimal prediction time used for cut-in potential calculation
#define SI_MIN_CUT_IN_PRED_TIME (0.1F)
// Maximal prediction time used for cut-in potential calculation
#define SI_MAX_CUT_IN_PRED_TIME (1.0F)
// Minimal prediction time used for cut-out potential calculation
#define SI_MIN_CUT_OUT_PRED_TIME (0.1F)
// Maximal prediction time used for cut-out potential calculation
#define SI_MAX_CUT_OUT_PRED_TIME (0.5F)

#define SI_MIN_PROB_OF_EXIST_PREDICTION (0.98F)

/*****************************************************************************
  SI lane association params
*****************************************************************************/

// typedef struct SIOccupancyParams
//{
//	float32 ObjOccPickupThresh;
//	float32 ObjOccDropThresh;
//	float32 LaneOccPickupThresh;
//	float32 LaneOccDropThresh;
//	float32 ObjLaneOverlapPickupThresh;
//	float32 ObjLaneOverlapDropThresh;
//}SIOccupancyParams_t;

/* Pickup and drop thresholds for lane selection of moving objects */

#define ObjectOccupancyPickupThresh \
  (0.50F)  // SIOccupancyParams.ObjOccPickupThresh
#define ObjectOccupancyDropThresh (0.3F)  // SIOccupancyParams.ObjOccDropThresh
#define LaneOccupancyPickupThresh \
  (0.50F)  // SIOccupancyParams.LaneOccPickupThresh
#define LaneOccupancyDropThresh (0.30F)  // SIOccupancyParams.LaneOccDropThresh
#define ObjectLaneOverlapPickupThresh \
  (1.00F)  // SIOccupancyParams.ObjLaneOverlapPickupThresh
#define ObjectLaneOverlapDropThresh \
  (0.3F)  // SIOccupancyParams.ObjLaneOverlapDropThresh

#define SI_OCC_PICKUP_TRSH_CR_MIN (50.0F)
#define SI_OCC_PICKUP_TRSH_CR_MAX (300.0F)
#define SI_OCC_PICKUP_TRSH_MIN (0.85F)

#define SI_OCC_PICKUP_ADD_TRSH (0.15F)

#define SI_DROP_TRSH_GRD_Y_OFFSET (10.0F)
#define SI_DROP_TRSH_GRD_Y_CONF (0.5F)

/* Max. Overlap variance for normal object occupancy drop border */
#define ObjectOccupancyDropMaxOverlapVariance 0.35f
#define OverlapDropMaxOverlapVariance 0.35f

/* object occupancy drop border for insecure overlap values */
#define ObjectOccupancyDropThreshInsecureOverlap 0.30
#define OverlapDropThreshInsecureOverlap 0.30

/* Pickup and drop threshold for lane selection of stationary and oncoming
 * objects */
#define ObjectOccupancyPickupThreshStat (0.60F)
#define ObjectOccupancyDropThreshStat (0.45F)
#define LaneOccupancyPickupThreshStat (0.30F)
#define LaneOccupancyDropThreshStat (0.20F)
#define LaneOverlapPickupThreshStat (1.00F)
#define LaneOverlapDropThreshStat (0.40F)
/* Pickup and drop threshold for lane selection of stationary and oncoming
 * objects in Road works or unlearned sensor */
#define ObjectOccupancyPickupThreshStatConservative (0.80F)
#define ObjectOccupancyDropThreshStatConservative (0.45F)
#define LaneOccupancyPickupThreshStatConservative (0.50F)
#define LaneOccupancyDropThreshStatConservative (0.45F)

/* times for stationary objects for being accepted inlane */
#define SI_STATOBJ_INLANE_HIGHSPEED (60.0F * C_MS_KMH)  //[m/s]
#define SI_STATOBJ_INLANE_LOWSPEED (15.0F * C_MS_KMH)   //[m/s]
#define SI_STATOBJ_INLANE_PARKSPEED (5.0F * C_MS_KMH)   //[m/s]
#define SI_STATOBJ_INLANE_NOSPEED (TUE_C_F32_DELTA)     //[m/s]

#define SI_STATOBJ_INLANE_DISTMAX (20.0F)
#define SI_STATOBJ_INLANE_DISTMIN (5.0F)
#define SI_STATOBJ_INLANE_DISTSLOPE                          \
  ((SI_STATOBJ_INLANE_DISTMAX - SI_STATOBJ_INLANE_DISTMIN) * \
   (1.0F / (SI_STATOBJ_INLANE_HIGHSPEED - SI_STATOBJ_INLANE_LOWSPEED)))
#define SI_STATOBJ_INLANE_DISTOFFSET \
  (SI_STATOBJ_INLANE_DISTMAX -       \
   (SI_STATOBJ_INLANE_DISTSLOPE * SI_STATOBJ_INLANE_HIGHSPEED))

#define SI_STATOBJ_INLANE_TIME_PARKSPEED \
  (SI_STATOBJ_INLANE_DISTMIN * (1.0F / SI_STATOBJ_INLANE_PARKSPEED))
#define SI_STATOBJ_INLANE_TIME_NOSPEED (0.72F)

/* times for stationary vehicle(Truck, Car, Motorcycle) for being accepted
 * inlane */
#define SI_STATVEH_INLANE_HIGHSPEED (100.0F * C_MS_KMH)  //[m/s]
#define SI_STATVEH_INLANE_LOWSPEED (25.0F * C_MS_KMH)    //[m/s]
#define SI_STATVEH_INLANE_PARKSPEED (5.0F * C_MS_KMH)    //[m/s]
#define SI_STATVEH_INLANE_NOSPEED (TUE_C_F32_DELTA)      //[m/s]

#define SI_STATVEH_INLANE_DISTMAX (20.0F)
#define SI_STATVEH_INLANE_DISTMIN (5.0F)
#define SI_STATVEH_INLANE_DISTSLOPE                          \
  ((SI_STATVEH_INLANE_DISTMAX - SI_STATVEH_INLANE_DISTMIN) * \
   (1.0F / (SI_STATVEH_INLANE_HIGHSPEED - SI_STATVEH_INLANE_LOWSPEED)))
#define SI_STATVEH_INLANE_DISTOFFSET \
  (SI_STATVEH_INLANE_DISTMAX -       \
   (SI_STATVEH_INLANE_DISTSLOPE)*SI_STATVEH_INLANE_HIGHSPEED)

#define SI_STATVEH_INLANE_TIME_PARKSPEED \
  (SI_STATVEH_INLANE_DISTMIN * (1.0F / SI_STATVEH_INLANE_PARKSPEED))
#define SI_STATVEH_INLANE_TIME_NOSPEED (0.72F)

/* times for moving objects for being accepted in lane */
/* time is a function of distance */
#define SI_DISTX_MOVE_OBJ_IN_LANE_MIN (0.1F)
#define SI_DISTX_MOVE_OBJ_IN_LANE_MAX (-100.0F)
#define SI_MINTIME_MOVE_OBJ_IN_LANE_MIN (0.0F)
#define SI_MINTIME_MOVE_OBJ_IN_LANE_MAX (0.5F)

#define SI_INLANE_TIME_MIN_EGOSPEED (0.0F)
#define SI_INLANE_TIME_MAX_EGOSPEED (7.0F)
#define SI_INLANE_TIME_SPE_MIN_TRSH (0.5F)
#define SI_INLANE_TIME_SPE_MAX_TRSH (0.0F)

#define SI_INLANE_TIME_MIN_UPDRATE (1.0F)
#define SI_INLANE_TIME_MAX_UPDRATE (0.85F)
#define SI_INLANE_TIME_UPRTE_MIN_TRSH (-0.15F)
#define SI_INLANE_TIME_UPRTE_MAX_TRSH (0.0F)

#define SI_INLANE_TIME_MIN_CURVE (25.0F)
#define SI_INLANE_TIME_MAX_CURVE (250.0F)
#define SI_INLANE_TIME_CRV_MIN_TRSH (0.5F)
#define SI_INLANE_TIME_CRV_MAX_TRSH (0.0F)

#define SI_INLANE_TIME_TRSH_MIN (0.1F)
#define SI_INLANE_TIME_TRSH_MAX (0.5F)

/*****************************************************************************
  Pre selection Parameters
*****************************************************************************/
// maximum registered accumulated driven distance where an object was relevant
#define SI_INLANE_DIST_MAXVALUE (200.0F)
#define SI_TRACK_SAFATY (-70.0F)
#define SI_WHEELTRACK_MIN (1.0F)
#define SI_WHEELTRACK_MAX (4.0F)
#define SI_TRACK_ABSTANDDIFF_MAX (30.0F)
#define SI_TRACK_ABSTANDDIFF_MIN (10.0F)
#define SI_WHEELTRACK_DEC (0.66F)

#define SI_TIMEEXT_VRELX_MIN (10.0F)
#define SI_TIMEEXT_VRELX_MAX (20.0F)
#define SI_TIMEEXT_TIME_MIN (1.0F)
#define SI_TIMEEXT_TIME_MAX (2.0F)
#define SI_FAR_RANGE (112.0F)
/*****************************************************************************
  (SYMBOLIC) CONSTANTS
*****************************************************************************/
#define SI_PRED_LANE_ASSOC_X_OBJ_MIN (-80)
#define SI_PRED_LANE_ASSOC_X_OBJ_MAX (-10)
#define SI_PRED_LANE_ASSOC_VX_OBJ_MAX (25.0F)
#define SI_PRED_LANE_ASSOC_VEGO_MIN (10.0F)
#define SI_PRED_LANE_ASSOC_CURVE_RAD_MIN (500.0F)
#define SI_PRED_LANE_ASSOC_MIN_Y_OPP_BORDER (6.5F)
#define SI_PRED_LANE_ASSOC_OPP_BORDER_INVLD (100.0F)
#define SI_PRED_LANE_ASSOC_MIN_OBJ_LIFETIME (2U)  // 10->2
#define SI_PRED_LANE_ASSOC_TIME_MAX (10U)
#define SI_PRED_LANE_ASSOC_TIME_IN_ENABLE (5U)
#define SI_PRED_LANE_ASSOC_TIME_OUT_ENABLE (5U)

/*****************************************************************************
  ASSOCIATIN LANE ENUMERATION CONSTANTS
*****************************************************************************/
#define ASSOC_LANE_UNKNOWN (0U)
#define ASSOC_LANE_FAR_LEFT (1U)
#define ASSOC_LANE_LEFT (2U)
#define ASSOC_LANE_EGO (3U)
#define ASSOC_LANE_RIGHT (4U)
#define ASSOC_LANE_FAR_RIGHT (5U)

/*****************************************************************************
  STATIC CONST PARAM
*****************************************************************************/
typedef struct {
  float dAmin;  // Minimum output value

  float dAmax;  // Maximum ouput value

  float dM;  // Slope of the line (Amax-Amin)/(Emax-Emin)

  float dB;  // Intercept value of the line (Amax-Amin)/(Emax-Emin) * Emin
} BML_t_LinFunctionArgs;

#define GDBLFunction_t BML_t_LinFunctionArgs

static const GDBLFunction_t TRCKSeekLaneWidthSpeedAdapted = {
    SI_HIGHWAYLANEWIDTH_SEEK_MIN,  // A1
    SI_HIGHWAYLANEWIDTH_SEEK_MAX,  // A2
    (SI_HIGHWAYLANEWIDTH_SEEK_MAX - SI_HIGHWAYLANEWIDTH_SEEK_MIN) *
        (1.0F /
         (SI_MAX_SPEED_HIGHWAYLANEWIDTH - SI_MIN_SPEED_HIGHWAYLANEWIDTH)),
    // A1 - (A2-A1)/(E2-E1)*E1
    SI_HIGHWAYLANEWIDTH_SEEK_MIN -
        ((SI_HIGHWAYLANEWIDTH_SEEK_MAX - SI_HIGHWAYLANEWIDTH_SEEK_MIN) *
         (1.0F /
          (SI_MAX_SPEED_HIGHWAYLANEWIDTH - SI_MIN_SPEED_HIGHWAYLANEWIDTH)) *
         SI_MIN_SPEED_HIGHWAYLANEWIDTH)};

static const GDBLFunction_t TRCKTrackOffsetSpeedAdapted = {
    SI_HIGHWAYLANEWIDTH_TRACK_OFFSET_MAX,  // A1
    SI_HIGHWAYLANEWIDTH_TRACK_OFFSET_MIN,  // A2
    (SI_HIGHWAYLANEWIDTH_TRACK_OFFSET_MIN -
     SI_HIGHWAYLANEWIDTH_TRACK_OFFSET_MAX) *
        (1.0F /
         (SI_MAX_SPEED_HIGHWAYLANEWIDTH - SI_MIN_SPEED_HIGHWAYLANEWIDTH)),
    // A1 - (A2-A1)/(E2-E1)*E1
    SI_HIGHWAYLANEWIDTH_TRACK_OFFSET_MAX -
        ((SI_HIGHWAYLANEWIDTH_TRACK_OFFSET_MIN -
          SI_HIGHWAYLANEWIDTH_TRACK_OFFSET_MAX) *
         (1.0F /
          (SI_MAX_SPEED_HIGHWAYLANEWIDTH - SI_MIN_SPEED_HIGHWAYLANEWIDTH)) *
         SI_MIN_SPEED_HIGHWAYLANEWIDTH)};

static const GDBLFunction_t aSILinearBracketRestrictionFarDist = {
    SI_BRACKET_RESTR_FAR_DIST_MAX,  // A1
    SI_BRACKET_RESTR_FAR_DIST_MIN,  // A2
    (SI_BRACKET_RESTR_FAR_DIST_MIN - SI_BRACKET_RESTR_FAR_DIST_MAX) *
        (1.0F /
         (SI_BRACKET_RESTR_FAR_DIST_X_MAX - SI_BRACKET_RESTR_FAR_DIST_X_MIN)),
    // A1 - (A2-A1)/(E2-E1)*E1
    SI_BRACKET_RESTR_FAR_DIST_MAX -
        (((SI_BRACKET_RESTR_FAR_DIST_MIN - SI_BRACKET_RESTR_FAR_DIST_MAX) *
          (1.0F / (SI_BRACKET_RESTR_FAR_DIST_X_MAX -
                   SI_BRACKET_RESTR_FAR_DIST_X_MIN))) *
         (SI_BRACKET_RESTR_FAR_DIST_X_MIN))};
#ifdef __cplusplus
}
#endif  // __cplusplus
#endif