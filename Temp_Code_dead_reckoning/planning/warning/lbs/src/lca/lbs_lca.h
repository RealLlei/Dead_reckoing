#pragma once
#ifndef LBS_LCA_H
#define LBS_LCA_H
/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "planning/warning/lbs/common/TM_Global_Types.h"

#define LCA_DEBUG
#define LCA_INPUT_OBJECT_NUMBER 80
#define LCA_EM_GEN_OBJECT_MT_STATE_DELETED 0U

#ifdef LCA_DEBUG
#include <assert.h>
#define LCA_ASSERT(cond) assert(cond)
#else
#define LCA_ASSERT(const) ((void)0)
#endif
/*****************************************************************************
  INPUT
*****************************************************************************/
typedef struct {
  uint32 uiTimeStamp_ms;
  uint16 uiMeasurementCounter_nu;
  uint16 uiCycleCounter_nu;
  uint8 eSigStatus_nu;
} LCASignalHeader_t;

typedef struct {
  boolean
      bRightSensor; /*Flag whether the object is detected by the right sensor*/
  float32 fDistX_met;       /*Object's longitudinal relative distance*/
  float32 fDistY_met;       /*Object's lateral relative distance*/
  float32 fVrelX_mps;       /*Object's longitudinal relative velocity*/
  float32 fVabsX_mps;       /*Object's longitudinal absolute velocity*/
  float32 fWidthLeft_met;   /*Object's width left of the track position(left
                               sensor view)*/
  float32 fWidthRight_met;  /*Object's width right of the track position(left
                               sensor view)*/
  float32 fLengthFront_met; /*Object's length ahead of the track position(left
                               sensor view)*/
  uint16 uiLifeCycles_nu;   /*Object lifetime in cycles,unit: null*/
  uint8
      uiMaintenanceState_nu; /*Maintenance state if object(measured,predicted)*/
  float32
      fProbabilityOfExistence_per;    /* Probability that the object represents
                                          a real object,unit:0.0-1.0f*/
  uint8 uiHighestAssocProb_per;       /*Highest association probability of all
                                          associated clusters in this cycle*/
  uint8 uiMeasuredTargetFrequency_nu; /*Bitfield to indicate if the object was
                                          measured in the last 8 cycles*/
  boolean bObjStable;                 /*Flag that object is stable*/
  float32 fDist2Border_met;  /* Object distance to a road border, unit:m*/
  boolean bDist2BorderValid; /* Valid flag if the value in Dist2Border can be
                                used */
  // SensorSpecific
  float32 fMirrorProb_per;  // The probability of mirror object
  float32 fRCS;             /*Filter RCS of the object*/
} LCAGenObjInfo_t;

typedef struct {
  float32 fTTC_s;         /*The object TTC time to ego vehicle,unit:s*/
  float32 fTTCAccel_mps2; /*TTC including acceleration*/
  float32 fTTCFiltered_s; /*The object filter TTC time to ego vehicle,unit:s*/
  float32 fUpdateRate_nu; /*The object measurement update rate,unit:NULL*/
  float32 fXMovement_met; /*The object total moving distance in the x
                             direction,unit:m*/
  boolean bLowTTCAtStart;
  boolean bCreateAdjStableObj;
} LCALBSObjInfo_t;

typedef LCALBSObjInfo_t LCALBSObjInfo_Array[LCA_INPUT_OBJECT_NUMBER];

typedef struct {
  uint8 eAssociatedLane;           // Lane Association
  float32 fVrelToTraj_mps;         // Object's relative velocity to trajectory
  float32 fDistToTraj_met;         // Distance to trajectory
  float32 fTraceBracketLeft_met;   // The trace bracket left side coordinate
  float32 fTraceBracketRight_met;  // The trace bracket right side coordinate
  float32 fObjBracketOverlap_met;  // Overlap between object and trace bracket

} LCASIObjInfo_t;

typedef LCASIObjInfo_t LCASIObjInfo_Array[LCA_INPUT_OBJECT_NUMBER];

typedef struct {
  LCAGenObjInfo_t GenObjInfo;

} LCAGenObject_st;

typedef LCAGenObject_st LCAGenObjectArray[LCA_INPUT_OBJECT_NUMBER];

typedef struct {
  uint32 uiVersionNumber;
  LCASignalHeader_t sSigHeader;
  LCAGenObjectArray aObject;
} LCAGenObjList_st;

typedef struct {
  LCASignalHeader_t sSigHeader;
  float32 fegoVelocity_mps; /*the ego vehicle longitudinal velocity ,unit:m/s*/
  float32 fegoAcceleration_mps2; /*the ego vehicle longitudinal acceleration
                                    ,unit:m/s^2*/
  uint8 uLCAGearPosition;
} LCAVehicleInfo_t;

typedef struct {
  /*CurveInfo_t */
  float32 fCurveRadius_met; /*The curvature radius of the current driver road
                               ,unit:m*/
  float32
      fDrivenCurveRadius_met; /*The curvature radius of the ego driver curve*/

  /*FusedRoadBorder_t */
  float32 fYOffsetFused_met; /*Distance between ego vehicle and left boundary*/
  float32
      fConfYOffset_per; /*Confidence percentage of offset in y-axis positive
                               direction */
  float32 fYOffsetFusedOppBorder_met; /*Distance between ego vehicle and right
                                         boundary */
  float32 fConfYOppOffset_per; /*Confidence percentage of offset in y-axis
                                  negative direction */

  /*LaneEstimation_t */
  float32 fConfAdjacentLanes_per; /*Confidence percentage of adjacent lane
                                     information */
  float32 fConfOppositeLanes_per; /*undetermined */
  sint8 iNumOfAdjacentLanes_nu;   /*The Number of adjacent lane */
  sint8 iNumOfOppositeLanes_nu;   /*undetermined */
} LCARoad_t;

typedef struct {
  boolean bBSDWarningLastCycle; /*The BSD Warning active signal */
  boolean bLCAWarningLastCycle; /*The LCA Warning active signal */
} LCALBSWarningInfo_t;

typedef struct {
  float32 fBSDZoneXMin_met;  // BSD zone xmin from BSD global
} LCABSDObjInfo_t;

typedef LCABSDObjInfo_t LCABSDObjInfo_Array[LCA_INPUT_OBJECT_NUMBER];

typedef struct {
  LCASIObjInfo_Array SIObjInfoList;
  LCABSDObjInfo_Array BSDObjInfoList;
  LCALBSObjInfo_Array LBSObjInfoList;
  LCALBSWarningInfo_t LBSWarningLastCycle;
} LCALBSInputInfo_st;

typedef struct {
  boolean bLCAFunctionActive;       /* LCA function active flag */
  boolean bLCAFunctionOutputActive; /* LCA function output flag */
  float32 fCycletime_s;             /* Current task cycle time */
} LCASystemParam_t;

typedef struct {
  float32 fMaxSpeedOverGround_mps;
} LCALBSGlobals_t;

typedef struct {
  LCAGenObjList_st GenObjList;       /* General Radar Object information */
  LCAVehicleInfo_t EgoVehInfo;       /* Vehicle dynamic information */
  LCARoad_t Road;                    /* Road information */
  LCALBSInputInfo_st LBSInputInfo;   /* LBS information */
  LCALBSGlobals_t LCALBSGlobalInput; /* LBS Global variables to LCA Input*/
  LCASystemParam_t LCASystemParam;   /* LCA System Parameter */
} LCAInReq_st;

/*****************************************************************************
  OUTPUT
*****************************************************************************/

typedef struct {
  uint32 uLBSBSDStartTime; /*the BSD function start time*/
  uint32 uLBSBSDEndTime;   /*the BSD function end time*/
  uint32 uLBSBSDRunTime;   /*the BSD function run time*/
} LCARunTime_t;

typedef struct {
  float32 fCriticalTTC_s;
  float32 fXObjectWarning_met;  // Object's longitudinal relative distance of
                                // LCA warning condition
  sint32 uLCAWarningID_nu;      // LCA warning object ID
  boolean bLCAWarnActive;       // LCA warning flag for LCA function, on if any
  // object is warning
  boolean bLCAWarnActiveLeft;
  boolean bLCAWarnActiveRight;
  boolean bLCAWarningLastCycle;  // LCA warning status from last cycle
} LCAWarnInfo_t;

typedef struct {
  float32 fTTCThreshold;       // TTC threshold
  float32 fBehindGrdProb_per;  // the object behind the guardrail
                               // probability,range:0.0 - 1.0,unit:percent
  uint8 uFrontMirrorCnt;
  boolean bUpdateRecently;  // Flag whether the current object is sufficiently
                            // updated(measured) in the last two cycle
  boolean bInLCARange;      //
  boolean bLCAMirrorObject;
  boolean bLCAMirrorFrontObject;
  boolean bLCAObjPathInvalid;  // Flag whether path is accessible
  boolean bLCAQuality;         // Flag whether the object quality is enough
  boolean bLCALaneConditions;
  boolean bLCARelevant;
  boolean bLCAWarningConditions;  // Flag of meeting all LCA Warning conditions
  boolean bLCAWarning;            // Flag whether current object is warning

  boolean bLowTTCAtStart;
  boolean bCreateAdjStableObj;
} LCAObjInfo_t;

typedef LCAObjInfo_t LCAObjInfo_Array[LCA_INPUT_OBJECT_NUMBER];

typedef struct {
  uint32 uLCAStartTime;  /*the LCA function start time*/
  uint32 uLBSBSDEndTime; /*the LCA function end time*/
  uint32 uLBSBSDRunTime; /*the LCA function run time*/
  float32 fCriticalTTC_s;
  float32 fXObjectWarning_met;  // Object's longitudinal relative distance of
                                // LCA warning condition
  sint32 uLCAWarningID_nu;      // LCA warning object ID
  boolean bLCAWarnActive;       // LCA warning flag for LCA function, on if any
                                // object is warning
  boolean bLCAWarnActiveLeft;
  boolean bLCAWarnActiveRight;
  boolean bLCAWarningLastCycle;  // LCA warning status from last cycle
  LCAObjInfo_Array LCAObjOutputList;
} LCAOutPro_st;

/*****************************************************************************
  PARAMTER
*****************************************************************************/

typedef struct {
  float32 fLCAZoneXMid_met; /*the LCA Warning Zone X direction middle position
                               unit:m*/
  float32
      fLCAZoneXMin_met; /*the LCA Warning Zone X direction max position unit:m*/
  float32 fLCAZoneYMinNear_met; /*the LCA Warning Zone Y direction min near
                                   position unit:m*/
  float32 fLCAZoneYMinFar_met;  /*the LCA Warning Zone Y direction min far
                                   position unit:m*/
  float32 fLCAZoneYMaxNear_met; /*the LCA Warning Zone Y direction max near
                                   position unit:m*/
  float32 fLCAZoneYMaxFar_met;  /*the LCA Warning Zone Y direction max far
                                   position unit:m*/
} LCAZone_t;

typedef struct {
  float32 fVehicleWidth_met; /*the vehicle body width,unit:m*/
  // float32 fDefaultLaneWidth_met;
  float32 fVoWnMinWarnEnable_mps;  /*the min enable velocity of LCA warning*/
  float32 fVownMinWarnDisable_mps; /*the max enable velocity of LCA warning*/
  float32 fTTCThreshold_s; /*the single TTC Threshold is applied when the
                              warning configuration is set to custom*/
  float32 fTTCThreshLowRelSpeed_s; /*the low TTC threshold is applied when the
                                       warning configuration is set to default*/
  float32 fTTCThreshMidRelSpeed_s; /*the mid TTC threshold is applied when the
                                       warning configuration is set to default*/
  float32
      fTTCThreshHighRelSpeed_s; /*the high TTC threshold is applied when the
                                       warning configuration is set to default*/
  // float32 fSwithOffDelay;
  float32 fMinTTCHysteresis_s; /*the compensation TTC threshold is applied when
                                   the LCA warning was activated in last cycle*/
  float32 fBridgeWarningTime_s;   /*add the additional BridgeWarningTime which
                                   bridges the warning for consecutive objects*/
  float32 fMaxLCARange_met;       /*the max LCA range threshold*/
  float32 fMaxLCACurveRadius_met; /*the max LCA curve radius threshold*/
  uint8 uLCAWarningDuration; /*the LCA warning configuration: off, custom and
                                default*/
  // boolean bUseDynamicLaneWidth;
  // boolean bActive;
  LCAZone_t LCAZone; /*LCA zone*/
} LCAParam_st;

/*****************************************************************************
  Calculation
*****************************************************************************/

typedef struct {
  float32 fTTCThreshVrelLow_s;
  float32 fTTCThreshVrelMid_s;
  float32 fTTCThreshVrelHigh_s;
  float32 fTTCHysteresis_s;
  float32 fLCARangeMax_met;     // Maximum LCA range parameter
  float32 fLCACurveRadMax_met;  // Maximum LCA curve radius parameter
  uint8 uLCAWarningDurationCfg;
} LCAConfig_t;

typedef struct {
  float32 fFMObjRate;
  float32 LCA_Vf_VxThreshAdd_mps;
  float32 LCA_Vf_VxThreshOwnLaneMin_mps;
  float32 LCA_Vf_VxThreshOwnLaneMax_mps;
  float32 LCA_Vf_VxThreshAdjLaneMin_mps;
  float32 LCA_Vf_VxThreshAdjLaneMax_mps;
  float32 fRCSStableObjOwnLane;  // RCS of the stable object in own lane
  float32 fRCSStableObjAdjLane;  // RCS of the stable object in adjacent lane
  sint32 uClosetStableObjIDOwnLane;  // the closet stable object ID in own lane
  sint32 uClosetStableObjIDAdjLane;  // the closet stable object ID in adjacent
                                     // lane
  uint8 uNofFMObjects;               // the number of front mirror objects
} LCAFrontMirror_t;

typedef struct {
  LCAWarnInfo_t LCAWarnInfo;
  LCAConfig_t LCAConfig;
  LCAFrontMirror_t LCAFrontMirror;
  float32 fLCARange;
  uint8 uCntLCAPathBlockedLeft;   // Counter of left LCA path blocked
  uint8 uCntLCAPathBlockedRight;  // Counter of right LCA path blocked
  boolean bLCAPathBlockedLeft;    // Flag whether left adjacent lane is no more
  boolean bLCAPathBlockedRight;   // Flag whether Right adjacent lane is no more
} LCAGlobals_st;

typedef enum { LCA_INIT, LCA_OK } LCAState_t;

typedef struct {
  boolean bLCAFunctionActionLastCycle;
  LCAState_t eLCAState;
} LCARunState_t;

typedef struct {
  LCAObjInfo_Array LCAObjInfoList;
  LCAGlobals_st LCAGlobals;
  LCARunState_t LCARunState;
} LCACalculate_st;

/*Temp define for other module*/
typedef struct {
  LCAWarnInfo_t LCAWarnInfo;
  LCAConfig_t LCAConfig;
  LCAFrontMirror_t LCAFrontMirror;
  float32 fLCARange;
  uint8 uCntLCAPathBlocked;
  boolean bLCAPathBlocked;
} LBS_LCA_Globals_st;

/*****************************************************************************
  DEBUG
*****************************************************************************/
typedef struct {
  uint32 uiVersionNumber;
  uint32 uLCAWarningID_nu;
  // LCAGlobals
  LCAWarnInfo_t LCAWarnInfo;
  LCAConfig_t LCAConfig;
  LCAFrontMirror_t LCAFrontMirror;
  float32 fLCARange;
  uint8 uCntLCAPathBlockedLeft;   // Counter of left LCA path blocked
  uint8 uCntLCAPathBlockedRight;  // Counter of right LCA path blocked
  boolean bLCAPathBlockedLeft;    // Flag whether left adjacent lane is no more
  boolean bLCAPathBlockedRight;   // Flag whether Right adjacent lane is no more
  LCAObjInfo_Array LCAObjOutputList;
} LCADebug_t;

/*****************************************************************************
  DATA_LOG
*****************************************************************************/
typedef struct {
  LCAInReq_st LCAReqPorts;
  LCAParam_st LCAParams;
  LCAOutPro_st LCAProPorts;
  LCADebug_t LCADebugInfo;
} LCADatalog_st;

#endif