#pragma once
#ifndef LBS_BSD_PAR_H
#define LBS_BSD_PAR_H

/*****************************************************************************
  CONSTS
*****************************************************************************/
#define BSD_SOT_ZONE_YMAX_MAX (10.f)  // limit SOT Zone YMAX
#define BSD_MIN_ZONE_OVERLAP_MULTILANE (0.3f)
#define BSD_OBJOVERLAP_ADJLANES_MAX (2)
#define BSD_BEHIND_GRD_COUNT_WARN_OFF_HYST (15u)
#define BSD_BEHIND_GRD_COUNT_LTIME_WARN_OFF_HYST (5u)
#define BSD_LI_OWNL_TTC_FILT_MIN (0.f)
#define BSD_LI_OWNL_TTC_FILT_MAX (1.5f)
#define BSD_LI_OWNL_TTC_INC_MIN (-10.f)
#define BSD_LI_OWNL_TTC_INC_MAX (0.f)
#define BSD_OWNLANE_COUNTER_MAX (20u)
#define BSD_OWNLANE_CNT_THRESH (3u)
#define CML_f_Delta \
  (0.0001f)  // Delta threshold value for 32 bit floating point equality tests
#define BSD_OWNLANE_COUNT_WARN_OFF_HYST (15u)
#define BSD_OWNLANE_CNT_FACTOR (0.5f)
#define BSD_WRAPPEDOBJ_ASSOC_PROB_MIN (0.7f)
/* Min update rate function of object speed over ground */
#define BSD_LI_UPDRATE_MIN_VXOVERGROUND (0.f)
#define BSD_LI_UPDRATE_MAX_VXOVERGROUND (3.f)
#define BSD_LI_UPDRATE_MIN (0.9f)
#define BSD_LI_UPDRATE_MAX (0.7f)
#define BSD_QUALITY_POE_MIN (0.6f)

#define BSD_OBJ_LIVED_ENOUGH_LIFETIME_MIN (0.25f)  // 0.3 --> 0.025 * 10
#define UI_192_TO_BINARY (192u)
#define UI_128_TO_BINARY (128u)
#define UI_64_TO_BINARY (64u)

//#define EM_GEN_OBJECT_DYN_PROPERTY_MOVING                          (0u)
//#define LBS_BSD_CLASS_STATIC_GRDHITCOUNTER                       (150u)
//#define LBS_BSD_CLASS_VEH_FRONT                                  (10u)
//#define LBS_BSD_CLASS_VEH_LAST                                   (99u)
#define BSD_OBJREL_DISTX_MAX (-1.f)
#define BSD_OBJREL_ARELX_MAX (4.f)
#define BSD_SOT_ABS_DISTX_MAX (1.f)
#define BSD_MAX_X_SOT (0.f)
#define BSD_MAX_CYCLETIME_SOT_CLOSE (0.3f)
#define BSD_MAX_X_SOT_CLOSE (-2.f)
#define BSD_MAX_Y_SOT_CLOSE (3.f)
#define BSD_MIN_VX_NONSOT (-4.0f)  // -3.5 --> -4
#define BSD_SOT_VXCUT_HYST (1.f)
#define BSD_FSOT_VXCUT_HYST_ACTIVEDELAY (0.5f)
#define BSD_FSOT_VXCUT_HYST_INACTIVEDELAY (1.f)
#define BSD_FASTSOT_ARELX_LIMIT_MIN (-5.f)
#define BSD_FASTSOT_ARELX_LIMIT_MAX (5.f)
#define BSD_SOT_VX_PRED_CYCLES (4.f)
#define BSD_SOT_OBSERVER_LIFETIME_MIN (10u)
#define BSD_PLAUSIBILITY_FIRSTX_MIN (-1.f)
#define BSD_PLAUSIBILITY_LIFECYCLES_MIN (2u)  //  100 -> 2
#define BSD_SOT_PLA_VRELX_MAX (5.0f)          // 0.5 -> 5
#define BSD_SOT_PLA_FIRSTX_MIN (-5.f)
#define BSD_SOT_PLA_XMOVED_MIN (1.f)
#define BSD_SOT_PLA_XMOVED_MIN_WARNING (0.5f)
#define BSD_PLAUSIBILITY_XMOVED_MIN (3.f)
#define BSD_PLAUSIBILITY_XMOVED_MIN_WARNING (1.5f)
#define BSD_GHOST_PLA_EGOSPEED_MIN (15.f)
#define BSD_GHOST_PLA_FISRTX_MIN (-7.f)
#define BSD_GHOST_PLA_XMOVED_MAX (2.5f)

/*****************************************************************************
  CONSTS
*****************************************************************************/

// Objects are not classified if they are at least that old
#define BSD_CLASSIFY_MIN_LIFETIME (0.3f)
// Below this speed,no reclassification should be done
#define BSD_CLASSIFY_VEH_CONFIRMED_REAR_VOWN_MIN (2.7f)
// Minimum object lifetime for static front reclassification
#define BSD_RECLASS_STATIC_FRONT_MIN_LIFETIME (2u)  // 50->2
// Minimum confidence for rear object reclassification
#define BSD_RECLASS_MIN_REAR_CONF (0.7f)

// Defines to for the determination of the appearance area
#define BSD_APPEAR_FRONT_RATIO_CUT (0.6f)
#define BSD_APPEAR_REAR_RATIO_CUT (0.6f)
#define BSD_APPEAR_FRONT_SECTOR_CUT (76.0f)
#define BSD_APPEAR_REAR_SECTOR_CUT (104.0f)
#define BSD_APPEARANCE_CHECK_MIN_LIFETIME (0.2f)
#define BSD_APPEARANCE_CHECK_MAX_LIFETIME (0.3f)

// Parameters to shorter the BSD Zone using the curve
#define F32_BSDZONE_INNER_SPEED_THRESHOLD (30.0f * C_MS_KMH)
#define F32_BSDZONE_INNER_HIGHSPEED_MIN_RADIUS (30.0f)
#define F32_BSDZONE_INNER_HIGHSPEED_EDGE_RADIUS (55.0f)
#define F32_BSDZONE_INNER_HIGHSPEED_XMIN (-4.0f)
#define F32_BSDZONE_INNER_LOWSPEED_MIN_RADIUS (5.0f)
#define F32_BSDZONE_INNER_LOWSPEED_EDGE_RADIUS (30.0f)
#define F32_BSDZONE_INNER_LOWSPEED_XMIN (-1.0f)
#define F32_BSDZONE_OUTER_SPEED_THRESHOLD (30.0f * C_MS_KMH)
#define F32_BSDZONE_OUTER_HIGHSPEED_MIN_RADIUS (30.0f)
#define F32_BSDZONE_OUTER_HIGHSPEED_EDGE_RADIUS (55.0f)
#define F32_BSDZONE_OUTER_HIGHSPEED_XMIN (-4.0f)
#define F32_BSDZONE_OUTER_LOWSPEED_MIN_RADIUS (5.0f)
#define F32_BSDZONE_OUTER_LOWSPEED_EDGE_RADIUS (30.0f)
#define F32_BSDZONE_OUTER_LOWSPEED_XMIN (-1.0f)

#define F32_BSDZONE_CURVEADAPTION_RADIUS (200.0f)

// Counter defines
#define BSD_GRD_HIT_COUNTER_MAX (30u)  // max guardrail hit counter number
#define BSD_GRD_COUNTER_MIN_INV_GRDTRACK (12u)

#define BSD_BEHIND_GRD_COUNTER_MAX \
  (30u)  // max behind guardrail hit counter number

// Minimum border confidence function of ego speed
#define BSD_LI_BORDER_MIN_EGOSPEED (2.0f)
#define BSD_LI_BORDER_MAX_EGOSPEED (15.0f)
#define BSD_LI_BORDER_MINBORDER_MIN_CONF (0.2f)
#define BSD_LI_BORDER_MINBORDER_MAX_CONF (0.7f)

#define BSD_GRD_DIST_OBJ_MIN_DISTX (-2.0f)
#define BSD_GRD_DIST_OBJ_FIRSTDETECT_X (-10.0f)

// GRD counter ,TTC increment function of TTC
#define BSD_LI_GRD_TTC_MIN (0.0f)
#define BSD_LI_GRD_TTC_MAX (2.0f)
#define BSD_LI_GRD_TTC_INC_MIN (-10.0f)
#define BSD_LI_GRD_TTC_INC_MAX (0.0f)

// Adapted BSD zone:BSD Zone length,LinearInterpolationFunction alpha function
// of ego speed
#define BSD_LI_ZONE_EGO_SPEED_MIN (0.0f)
#define BSD_LI_ZONE_EGO_SPEED_MAX (8.0f)
#define BSD_LI_ZONE_LPF_APLHA_MIN (0.0f)
#define BSD_LI_ZONE_LPF_APLHA_MAX_SHORTEN (0.1f)
#define BSD_LI_ZONE_LPF_APLHA_MAX_LENGTHEN (0.025f)

// Moving to stationary VX threshold function of ego speed
#define BSD_LI_MOVSTAT_EGOSPEED_MIN (0.0f)
#define BSD_LI_MOVSTAT_EGOSPEED_MAX (15.0f)
#define BSD_LI_MOVSTAT_VXTHRESH_MIN (1.5f)
#define BSD_LI_MOVSTAT_VXTHRESH_MAX (5.0f)

// Side object reclassification,XminThresh function of curve radius
#define BSD_LI_SIDE_RECLASS_MIN_CURVERAD (100.0f)
#define BSD_LI_SIDE_RECLASS_MAX_CURVERAD (50.0f)
#define BSD_LI_SIDE_RECLASS_MIN_XMIN_THRESH (-2.5f)
#define BSD_LI_SIDE_RECLASS_MAX_XMIN_THRESH (-3.5f)

// Side object reclassification,rear object confidence,LPF alpha function of
// XMinObj
#define BSD_LI_SIDE_RECLASS_MIN_XMIN (-1.5f)
#define BSD_LI_SIDE_RECLASS_MAX_XMIN (-4.0f)
#define BSD_LI_SIDE_RECLASS_MIN_LPF_ALPHA (0.05f)
#define BSD_LI_SIDE_RECLASS_MAX_LPF_ALPHA (0.1f)

// Position threshold to classify Object as guardrail object
#define BSD_INNER_SIDE_X1 (-2.0f)
#define BSD_INNER_SIDE_X2 (-1.0f)
#define BSD_INNER_SIDE_X3 (1.0f)
#define BSD_INNER_SIDE_X4 (2.0f)
#define BSD_INNER_SIDE_Y1 (1.0f)
#define BSD_INNER_SIDE_Y2 (1.5f)
#define BSD_INNER_SIDE_Y3 (1.5f)
#define BSD_INNER_SIDE_Y4 (2.0f)

#define BSD_OUTER_SIDE_X1 (-2.0f)
#define BSD_OUTER_SIDE_X2 (-1.0f)
#define BSD_OUTER_SIDE_X3 (1.0f)
#define BSD_OUTER_SIDE_X4 (2.0f)
#define BSD_OUTER_SIDE_Y1 (-1.0f)
#define BSD_OUTER_SIDE_Y2 (-0.5f)
#define BSD_OUTER_SIDE_Y3 (-0.5f)
#define BSD_OUTER_SIDE_Y4 (-1.0f)

#define BSD_GRD_REL_NROF_THRESHOLDS (4)

// GRD preconditions defines
#define BSD_GRD_DETECT_BORDER_CONF_MIN (0.7f)
#define BSD_GRD_DETECT_FIRSTDETECTX_MIN (-20.0f)
#define BSD_GRD_DETECT_VRELX_MAX (5.0f)

// GRD Counter defines
#define BSD_GRD_INCREMENT_LCA_WARNING (-1)
#define BSD_GRD_INCREMENT_TTC_DISTX_MAX (-1.0f)

// Own Lane Counter Defines
#define BSD_OWNL_INC_LCA_WARNING (-2)
#define BSD_OWNL_INC_OVERLAP (1)
#define BSD_OWNL_OVERLAP_METERS (1.50f)  // 1 -> 1.5
#define BSD_SOTZONE_FIRSTDETECTX_MIN (-1.0f)
#define BSD_SOT_SPEED_MINTHRESHOLD (-5.0f)

#define BSD_ZONE_DIST_THRESHOLD_LENGTHEN (0.02f)

// Sector cut calculation defines
#define BSD_SECTORCUT_EGOSPEED_MIN (1.0f)
#define BSD_SECTORCUT_ABSCURVE_MIN (5.0f)
#define BSD_SECTORCUT_ABSCURVE_MAX (200.0f)
#define BSD_AVERAGE_WHEELBASE_FACTOR (29.6f)

// Object reclassification  threshold
// Front Object
#define BSD_FRONT_RECLASS_BORDER_CONF_MIN (0.6f)
#define BSD_FRONT_RECLASS_OBJ_XMIN_MAX (-1.0f)
#define BSD_FRONT_RECLASS_DIST2BORD_MAX (-1.0f)
// Side object
#define BSD_SIDE_RECLASS_LAT2LONG_MOV_THRESH (2.0f)
#define BSD_SIDE_RECLASS_VRELX_MAX (4.0f)
#define BSD_SIDE_RECLASS_LPF_DOWN_ALPHA (0.05f)
#define BSD_SIDE_RECLASS_LOW_CURVE_THRESH (100.0f)
#define BSD_SIDE_RECLASS_XMIN (-1.5f)
#define BSD_SIDE_RECLASS_XMIN_HIGH (-2.0f)
// Rear object
#define BSD_RECLASS_AT_REAR_POE_MIN (0.98f)
#define BSD_RECLASS_AT_REAR_UPDATE_MIN (0.9f)
#define BSD_RECLASS_AT_REAR_LIFETIME_MIN (0.3f)  //  1 --> 0.3
#define BSD_RECLASS_AT_REAR_DISTY_MIN (1.2f)
#define BSD_RECLASS_AT_REAR_XMIN_MAX (-3.0f)
#define BSD_RECLASS_AT_REAR_XMAX_MAX (-1.5f)

// Classify GRD object
#define BSD_CLASSIFY_GRD_COUNTER_MIN (20u)
#define BSD_CLASSIFY_GRD_XMIN_MAX (-1.5f)
#define BSD_CLASSIFY_GRD_BORDER_CONF_MIN (0.8f)
#define BSD_CLASSIFY_GRD_DIST2BORD_MAX (-1.0f)

// Minimum Association probability function of mounting angle
#define BSD_LI_ASSOC_MIN_MOUNTINGANGLE (20.0f)
#define BSD_LI_ASSOC_MAX_MOUNTINGANGLE (50.0f)
#define BSD_LI_ASSOC_MIN_MINPROB_FRONT (0.6f)
#define BSD_LI_ASSOC_MAX_MINPROB_FRONT (0.5f)
#define BSD_LI_ASSOC_MIN_MINPROB_SIDEREAR (0.65f)
#define BSD_LI_ASSOC_MAX_MINPROB_SIDEREAR (0.5f)
#define BSD_LI_ASSOC_MIN_MINXMOVED (1.5f)
#define BSD_LI_ASSOC_MAX_MINXMOVED (0.0f)

#define BSD_WRAPPEROBJ_LIFETIME_MIN (0.5f)
#define BSD_WRAPPEROBJ_FIRSTDETX_MIN (-5.0f)
#define BSD_WRAPPEROBJ_FIRSTDETX_MAX (-20.0f)
#define BSD_WRAPPEROBJ_ANGLE_MIN (170.0f)
#define BSD_NEW_WRAPPEROBJ_MIN_FIRSTDETX (0.0f)
#define BSD_NEW_WRAPPEROBJ_MAX_FIRSTDETX (-2.5f)
#define BSD_NEW_WRAPPEROBJ_ANGLE_MIN (180.0f)

#define BSD_ASSOCPROB_INIT (0.5f)

#endif