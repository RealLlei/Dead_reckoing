#pragma once
#ifndef LBS_LCA_PAR_H
#define LBS_LCA_PAR_H

#define LCA_CFG_USE_TTCFILT_ON_WARN_START FALSE  // TRUE
/*****************************************************************************
  CONSTS
*****************************************************************************/
#define EM_N_OBJECTS 80

#ifndef F32_VALUE_INVALID
#define F32_VALUE_INVALID (1000.f)
#endif
#ifndef C_F32_DELTA
#define C_F32_DELTA (0.0001f)
#endif

#ifndef UI_255_TO_BINARY
#define UI_255_TO_BINARY (255u)  // 1111 1111
#endif
#ifndef UI_192_TO_BINARY
#define UI_192_TO_BINARY (192u)  // 1100 0000
#endif
#ifndef UI_128_TO_BINARY
#define UI_128_TO_BINARY (128u)  // 1000 0000
#endif
#ifndef UI_64_TO_BINARY
#define UI_64_TO_BINARY (64u)  // 0100 0000
#endif

// Definitions for mirror check
#define LCA_MAX_UPDRATE_MIRROR_CHECK (0.95f)
#define LCA_MIN_DIST2BORDER_MIRROR_CHECK (-0.5f)

// Definition for lane condition check
#define LCA_LANE_COND_NROF_THRESHOLDS (3)
#define LCA_LANE_COND_LOW_SPEED (13.89f)   // 50km/h
#define LCA_LANE_COND_MED_SPEED (25.f)     // 90km/h
#define LCA_LANE_COND_HIGH_SPEED (33.33f)  // 120km/h
#define LCA_LANE_COND_LANE_WIDTH_MIN (1.8f)
#define LCA_LANE_COND_LANE_WIDTH_MED (2.5f)
#define LCA_LANE_COND_LANE_WIDTH_MAX (3.2f)

// Definitions for mirror detection
#define LCA_MIN_EGOSPEED_FM_DETECTION (5.0f)
#define LCA_MIRROR_PROB_RECLASSIFY (0.5f)
#define LCA_MIRROR_PROB_LOW (0.7f)
#define LCA_MIRROR_PROB_HIGH (0.95f)

// Definition for front mirror detection
#define LCA_MIN_TARGETSPEED_FM (5.0f)

// Parameter for LCA range limitation

#define LCA_RANGE_MIN_ADDTHRESH (0.1f)

// InLane Time parameters
#define LCA_INLANETIME_MIN_VREL_TO_TRAJ (0.1f)
#define LCA_INLANETIME_MID_VREL_TO_TRAJ (0.5f)
#define LCA_INLANETIME_MIN_OVERLAP_OFFSET (0.3f)
#define LCA_INLANETIME_MAX_OVERLAP_OFFSET (0.6f)
#define LCA_INLANETIME_MIN_X_DIST_OFFSET (-80.f)
#define LCA_INLANETIME_MAX_X_DIST_OFFSET (-50.f)
#define LCA_INLANETIME_BRACKET_OVERLAP_MIN (0.8f)

#define LCA_MIN_UPDATERATE_RELEVANT (0.6f)
#define LCA_MIN_UPDATERATE (0.75f)
#define LCA_MIN_POE_RELEVANT (0.8f)
#define LCA_MIN_POE (0.9f)
#define LCA_MIN_POE_INITIAL (0.99f)
#define LCA_MAX_X_WARN_ACTIVATION (-2.0f)
#define LCA_MAX_X_TTC_ACCEL_EVAL (-20.f)
#define LCA_MAX_VX_TTC_ACCEL_EVAL (5.0f)
#define LCA_MIN_ASSOC_PROB_WARNING (70u)
#define LCA_MIN_X_TTC_FILTERED (-10.0f)

// Definitions for object start property check
#define LCA_MAX_X_DIST_ADJ_STABLE_OBJ (2.5f)
#define LCA_MAX_Y_DIST_ADJ_STABLE_OBJ (2.5f)
#define LCA_MIN_UPDRATE_ADJ_STABLE_OBJ (0.9f)
#define LCA_MIN_X_LOW_TTC_START_CHECK (-50.f)
#define LCA_MAX_X_LOW_TTC_START_CHECK (-5.f)

// Definitions for mirror check
#define LCA_BEHIND_GRD_MIN_X_THRESH (-50.f)
#define LCA_BEHIND_GRD_MAX_X_THRESH (-10.f)
#define LCA_BEHIND_GRD_MIN_DIST2BORDER (0.f)
#define LCA_BEHIND_GRD_MAX_DIST2BORDER (-0.5f)

// check for relevance
#define LCA_MIN_INLANE_TIME (0.3f)
#define LCA_MIN_X_MOVEMENT_RELEVANT (5.0f)
#define LCA_MIN_LIFETIME_RELEVANT (2u)  //50 ->2
#define LCA_MAX_X_DIST_RELEVANT (-5.f)

// LOWPASS FILTER defines

#define LCA_BEHINDGRD_PROB_FILTER_DOWN (0.0025f)
#define LCA_BEHINDGRD_PROB_FILTER_UP (0.05f)

// LCA path blocked, border conf function of ego speed
#define LCA_LI_PATHB_EGOSPEED_MIN (5.f)
#define LCA_LI_PATHB_EGOSPEED_MAX (15.f)
#define LCA_LI_PATHB_BORDERCONF_MIN (0.6f)
#define LCA_LI_PATHB_BORDERCONF_MAX (0.4f)

// LCA Obj behind guardrail, border conf function of ego speed
#define LCA_LI_BEHINDGRD_MIN_EGOSPEED (0.f)
#define LCA_LI_BEHINDGRD_MAX_EGOSPEED (25.f)
#define LCA_LI_BEHINDGRD_MIN_BORDERCONF (0.4f)
#define LCA_LI_BEHINDGRD_MAX_BORDERCONF (0.8f)

#define LCA_PATHB_COUNTER_MAX (10u)
#define LCA_PATHB_ACTIVE_COUNTER_THRESH (2u)
#define LCA_PATHB_INACTIVE_COUNTER_THRESH (6u)

#define LCA_QUAL_LIFETIME_MIN (2u)  // 15 ---> 2 JO
#define LCA_QUAL_VRELX_MIN (6.f)    // 12 --->6  JO
#define LCA_QUAL_ACTIVE_UPDRATE_MIN (0.7f)
#define LCA_QUAL_INACTIVE_UPDRATE_MIN (0.9f)
#define LCA_QUAL_LOWTTC_ADJACENT_UPDATE_MIN (0.95f)
#define LCA_QUAL_HIGH_UPDATE_DISTX_MIN (-40.f)

// ST Parameter
#define LCA_PATHINV_INACTIVE_SIDEOFFSET (0.5f)
#define LCA_PATHINV_INACTIVE_BORDER_CONF (0.6f)
#define LCA_PATHINV_ACTIVE_SIDEOFFSET (1.0f)
#define LCA_PATHINV_ACTIVE_BORDER_CONF (0.4f)

#define LCA_UPDATED_UPDATED_MIN (0.8f)

#define LCA_BEHINDGRD_DIST2BORD_ADDTHRESH (2.f)

#define LCA_WARN_ACTIVE_TTC_FACTOR (1.5f)
#define LCA_WARN_BSDZONE_XDIFF_MAX (1.f)

#define LCA_WARNING_BEHINDGRD_THRESH (0.7f)

#define LCA_FM_ADDVXTHRESH_FMRATE_MIN (1.5f)
#define LCA_FM_ADDVXTHRESH_UPDRATE_FACTOR (2.f)

#define LCA_FM_ADDVXTHRESH_LOWRCS (0.5f)

#define LCA_FM_COUNTER_MAX (20u)
#define LCA_FM_ACTIVE_CNT_THRESH (2u)
#define LCA_FM_INACTIVE_CNT_THRESH (10u)
/*LCA WARN CFG*/
#define LCA_WARN_CFG_OFF (0)
#define LCA_WARN_CFG_CUSTOM (10)

// Defines which have the same first 32 characters create a clash
// max interpreted length 32
/*default TTC parameter*/
#define LCA_TTC_THRESH_DEFAULT (4.0f)
#define LCA_TTC_VREL_LOW_MAX (10.0f)
#define LCA_TTC_VREL_MID_MAX (15.0f)

/*Parameter for LCA range limitation*/
#define LCA_RANGE_MAX_DEFAULT (60.0f)
#define LCA_RANGE_MIN (7.0f)
#define LCA_RANGE_MIN_ADDTHRESH (0.1f)
#define LCA_CURVE_RAD_MAX_DEFAULT (250.0f)
#define LCA_CURVE_RAD_MIN (25.0f)

/**/
#define LCA_RANGE_LIM_EGOSPEED_LOW (30.0 / 3.6f)  //

/*Definitions for front mirror detection*/
#define LCA_MIN_EGOSPEED_FM_DETECTION (5.0f)
#define LCA_MIN_TARGETSPEED_FM (5.0f)
#define LCA_MIN_UPDATE_MIRRORING_OBJ (0.95)
#define LCA_MAX_X_MIRRORING_OBJ (-12.0f)
#define LCA_MIN_X_MIRRORING_OBJ (-80.0f)
#define LCA_MAX_Y_MIRRORING_OBJ (5.0f)
#define LCA_MAX_VX_MIRRORING_OBJ (5.0f)
#define LCA_FRONT_MIRROR_VX_THRESH_OFFSET (0.5f)
#define LCA_FRONT_MIRROR_RCS_INVALID (-20.0f)
#define LCA_FRONT_MIRROR_RCS_FILT_UP (0.1f)
#define LCA_FRONT_MIRROR_RCS_FILT_DN (0.05f)

/*LCA Range function of ego speed*/
#define LCA_LI_MIN_RANGETMPSPEED (10.0f)  //10
#define LCA_LI_MAX_RANGETMPSPEED (70.0f)  // 70

/*Low pass filter alpha defines*/
#define LCA_RANGE_FILTER_DOWN (0.1f)
#define LCA_RANGE_FILTER_UP (0.05f)

#define LCA_ADDVXTHRESH_FILTER_DOWN (0.01f)
#define LCA_ADDVXTHRESH_FILTER_UP (0.05f)

#define LCA_FMOBJRATE_FILTER_DOWN (0.0025f)
#define LCA_FMOBJRATE_FILTER_UP (0.05f)

/*LCA FM additional thresh function of ego speed,function of curve radius*/
#define LCA_LI_FM_ADDVXSPEED_MIN_EGOSPEED (0.0f)
#define LCA_LI_FM_ADDVXSPEED_MAX_EGOSPEED (40.0f)
#define LCA_LI_FM_ADDVXSPEED_MIN_THRESH (-0.5f)
#define LCA_LI_FM_ADDVXSPEED_MAX_THRESH (0.5f)

#define LCA_LI_FM_ADDVXCURVE_MIN_CURVE (50.0f)
#define LCA_LI_FM_ADDVXCURVE_MAX_CURVE (100.0f)
#define LCA_LI_FM_ADDVXCURVE_MIN_THRESH (1.5f)
#define LCA_LI_FM_ADDVXCURVE_MAX_THRESH (0.0f)

#define LCA_LI_FM_ADDVXACCEL_MIN_ACCEL (0.0f)
#define LCA_LI_FM_ADDVXACCEL_MAX_ACCEL (2.0f)
#define LCA_LI_FM_ADDVXACCEL_MIN_THRESH (0.0f)
#define LCA_LI_FM_ADDVXACCEL_MAX_THRESH (0.6f)

#define LCA_LI_FM_ADDVXFMRATE_MIN_FMRATE (1.5f)
#define LCA_LI_FM_ADDVXFMRATE_MAX_FMRATE (5.0f)
#define LCA_LI_FM_ADDVXFMRATE_MIN_THRESH (1.0f)
#define LCA_LI_FM_ADDVXFMRATE_MAX_THRESH (2.0f)

/*LCA FM,additional VrelX thresh function of VrelX*/
#define LCA_LI_FM_VXTHRESHADD_MIN_VX (0.0f)
#define LCA_LI_FM_VXTHRESHADD_MAX_VX (20.0f)
#define LCA_LI_FM_VXTHRESHADD_MIN_THRESH (0.5f)
#define LCA_LI_FM_VXTHRESHADD_MAX_THRESH (2.0f)

#define LCA_FM_ADD_VXACCEL_EGOSPEED_MIN (9.0f)
#define LCA_FM_ADD_VXFMRATE_FMRATE_MIN (1.5f)
#define LCA_FM_LIMITVATHRESH_EGOSPEED_FACTOR (0.25f)

#define LCA_FM_STABLEOBJ_RCS_ADDTHRESH (10.0f)
#define LCA_FM_STABLEOBJ_UPDRT_ADDTHRESH (0.1f)
#define LCA_FM_STABLEOBJ_XDIFF_THRESH (20.0f)
#define LCA_FM_STBLOBJ_MAX_FACTOR_VRELXABS (0.6f)
#define LCA_FM_STBLOBJ_MAX_VRELXABS (10.0f)
#define LCA_FM_STBLOBJ_DISTX_MAX (-5.0f)
#define LCA_FM_STBLOBJ_ACTIVE_DISTX_MAX (-1.0f)
#define LCA_FM_STBLOBJ_DISTX_MIN (-80.0f)
#define LCA_FM_STBLOBJ_UPDRT_MIN (0.9f)
#define LCA_FM_STBLOBJ_BEHINDGRD_MAX (0.3f)
#define LCA_CLOSE_STBLOBJ_DISTX_ADDTHRESH (3.0f)
#define LCA_CLOSE_STBLOBJ_VRELX_ADDTHRESH (3.0f)
#define LCA_CLOSE_STBLOBJ_UPDRT_ADDTHRESH (0.15f)
#define LCA_FM_ADDVXTHRESH_FMRATE_MIN (1.5f)
#define LCA_FM_ADDVXTHRESH_UPDRTE_FACTOR (2.0f)
#define LCA_FM_ADDVXTHRESH_VXOVERGROUND_FACTOR (1.5f)
#define LCA_FM_LOW_RCS_THRESH (15.0f)
#define LCA_FM_ADDVXTHRESH_LOWRCS (0.5f)
#define LCA_FM_OBJ_BEHIND_DISTX_THRESH (10.0f)
#define LCA_FM_OBJ_BEHIND_DISTY_THRESH (2.0f)
#define LCA_FM_COUNTER_MAX (20u)
#define LCA_FM_ACTIVE_CNT_THRESH (2u)
#define LCA_FM_INACTIVE_CNT_THRESH (10u)
/*****************************************************************************
  CONSTS
*****************************************************************************/

// InLane Time parameters

#define LCA_MAX_X_WARN_ACTIVATION (-2.0f)
#define LCA_MAX_X_TTC_ACCEL_EVAL (-20.f)
#define LCA_MAX_VX_TTC_ACCEL_EVAL (5.0f)
#define LCA_MIN_ASSOC_PROB_WARNING (70u)
#define LCA_MIN_X_TTC_FILTERED (-10.0f)

/*****************************************************************************
  ASSOCIATIN LANE ENUMERATION CONSTANTS
*****************************************************************************/
#define LCA_SI_ASSOC_LANE_UNKNOWN (0u)
#define LCA_SI_ASSOC_LANE_FAR_LEFT (1u)
#define LCA_SI_ASSOC_LANE_LEFT (2u)
#define LCA_SI_ASSOC_LANE_EGO (3u)
#define LCA_SI_ASSOC_LANE_RIGHT (4u)
#define LCA_SI_ASSOC_LANE_FAR_RIGHT (5u)

#endif