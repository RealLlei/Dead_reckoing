#pragma once
#ifndef LBS_RCW_EXT_H
#define LBS_RCW_EXT_H
/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "planning/warning/lbs/common/TM_Global_Types.h"
#include "string.h"

/*****************************************************************************
  CONSTS
*****************************************************************************/
#define RCW_INPUT_OBJECT_NUMBER (80u)  // re-define
#define RCW_MAX_NOF_CORR_OBJS (4u)

#define RCW_EM_GEN_OBJECT_MT_STATE_DELETED (0U)
#define RCW_EM_GEN_OBJECT_MIN_VELOCITY_X (1.94f)  // 7km/h

#define C_PI ((float32)3.14159265359f)

// #define LBS_RCW_BLK_CORR_MIN_SDISIX 		(-50.f)
// #define LBS_RCW_BLK_CORR_MAX_SDISIX 		(0.f)

#define LBS_RCW_MIN_UPDATERATE_REVELENT (0.60f)
#define LBS_RCW_MIN_UPDATERATE (0.75f)
#define LBS_RCW_MIN_POE_REVEVANT (0.80f)
#define LBS_RCW_MIN_POE (0.90f)
#define LBS_RCW_MIN_POE_INITAL (0.99f)

// relevent check
#define LBS_RCW_MIN_X_MOVEMENT_RELEVENT (5.0f)
#define LBS_RCW_MIN_DISTX_RELEVENT (-40.f)
#define LBS_RCW_MIN_X_OPP_SIDE_RELEVENT (-15.f)
#define LBS_RCW_MAX_X_OPP_SIDE_RELEVENT (-5.f)

#define LBS_RCW_QUAL_ACTIVE_UPDATE_MIN (0.7f)
#define LBS_RCW_QUAL_INACTIVE_UPDATE_MIN (0.9f)
#define LBS_RCW_QUAL_ADJOBJ_UPDATE_MIN (0.98f)
#define LBS_RCW_QUAL_LOWTTC_UPDATE_MIN (0.95f)
#define LBS_RCW_RELEVANCE_LIFECYCLE_MIN (2U)

// multipath object
#define LBS_RCW_X_MIN_MULTIPATH_OBJ (-50.f)
#define LBS_RCW_VX_MIN_MULTIPATH_OBJ (10.f * TUE_C_MS_KMH)
#define LBS_RCW_X_MARGIN_MULTIPATH_OBJ (5.f)
#define LBS_RCW_VX_MARGIN_MULTIPATH_OBJ (0.5f)

#define LBS_RCW_MULTIP_VXMARGIN_FACTOR (0.2f)
#define LBS_RCW_MULTIP_DISTX_MIN (-10.f)
#define LBS_RCW_MULTIP_DIST2CRS_DIFF (3.f)
#define LBS_RCW_MULTIP_CNP_MAX (20u)
#define LBS_RCW_MULTIP_ACTIVE_CNT_THRESH (5u)
#define LBS_RCW_MULTIP_INACTIVE_CNT_THRESH (10u)

#define LBS_RCW_LI_MULTIP_MIN_DISTX (-15.f)
#define LBS_RCW_LI_MULTIP_MAX_DISTX (-8.f)
#define LBS_RCW_LI_MULTIP_CUR_MIN_DISTX (2.f)
#define LBS_RCW_LI_MULTIP_CUR_MAX_DISTX (0.5f)

// update status
#define LBS_UI_240_TO_BINARY (240u)
#define LBS_UI_224_TO_BINARY (224u)

#define LBS_RCW_UPDATED_MIN_UPDATE (0.8f)

// corridor calculate
#define LBS_RCW_STRAIGHT_CORRIDOR_SPEED_THRESH (1.f)

#define LBS_RCW_MIN_OBJECT_WIDTH (0.f)
#define LBS_RCW_MAX_OBJECT_WIDTH (2.55f)

// #define LBS_RCW_CORRHITCNT_SMALL_OBJ_WIDTH	(2.5f)//1.5origin todo

/* #define LBS_RCW_CORRIDOR_HIT_CNT_THRESH		15u	move the
 * parameter input*/
#define LBS_RCW_CORRIDOR_HIT_CNT_MAX (30u)

#define LBS_RCW_LI_TIMEIN_OCC_MAX (0.6f)

// corridor blocked
// #define LBS_RCW_CORRB_ACTIVE_OCC_MIN		(0.15f)
// #define	LBS_RCW_CORRB_INACTIVE_OCC_MIN		(0.30f)

// choose occupancy
// #define LBS_RCW_OCC_ADJUST					(0.35f) move to
// para.

// Condition check
#define ASSOC_LANE_EGO (3u)

// Stationary blocked
#define EM_GEN_OBJECT_DYN_PROPERTY_MOVING (0u)
#define EM_GEN_OBJECT_DYN_PROPERTY_STATIONARY (1u)
#define EM_GEN_OBJECT_DYN_PROPERTY_ONCOMING (2u)
#define EM_GEN_OBJECT_DYN_PROPERTY_STATIONARY_CANDIDATE (3u)
#define EM_GEN_OBJECT_DYN_PROPERTY_UNKNOWN (4u)
#define EM_GEN_OBJECT_DYN_PROPERTY_CROSSING_STATIONARY (5u)
#define EM_GEN_OBJECT_DYN_PROPERTY_CROSSING_MOVING (6u)
#define EM_GEN_OBJECT_DYN_PROPERTY_STOPPED (7u)

/*****************************************************************************
  INPUT
*****************************************************************************/
typedef struct {
  uint32 uiTimeStamp_ms;
  uint16 uiMeasurementCounter_nu;
  uint16 uiCycleCounter_nu;
  uint8 eSigStatus_nu;
} RCWSignalHeader_t;

typedef struct {
  float32 fDistX_met; /*Object's longitudinal relative distance*/
  float32 fDistY_met; /*Object's lateral relative distance*/
  float32 fVrelX_mps; /*Object's longitudinal relative velocity*/
  float32 fVrelY_mps; /*Object's lateral relative velocity*/
  float32 fArelX_mpss;
  float32 fArelY_mpss;

  float32 fVabsX_mps;
  float32 fVabsY_mps;
} RCW_GenObjKinematics_t;

typedef struct {
  float32 fWidth_met;       /*Object's overall width*/
  float32 fWidthLeft_met;   /*Object's width left of the track position(left
                               sensor view)*/
  float32 fWidthRight_met;  /*Object's width right of the track position(left
                               sensor view)*/
  float32 fLength_met;      /*Object's overall length*/
  float32 fLengthFront_met; /*Object's length ahead of the track position(left
                               sensor view)*/
  float32 fLengthRear_met;  /*Object's length behind the track position(left
                               sensor view)*/
  float32 fAbsOrientation_rad; /*Object moving direction,based on VX and VY in
                                  AUTOSAR(left sensor view)*/
} RCW_GenObjGeometry_t;

typedef struct {
  float32 fLifeTime_s;         /*Object lifetime in second,unit: s*/
  uint16 uiLifeCycles_nu;      /*Object lifetime in cycles,unit: null*/
  uint8 uiMaintenanceState_nu; /*Maintenance state if
                                  object(measured,predicted)*/
  sint32 uiID_nu; /*Object detected ID ,unit: number,range:0-65536*/
} RCW_GenObjGenerals_t;

typedef struct {
  uint8 eDynamicProperty_nu;   /*Object dynamic property,stationary,moving or
                                  oncoming*/
  uint8 uiDynConfidence_per;   /*General confidence of dynamic
                                  property(moving,crossing,oncoming)*/
  uint32 eClassification_nu;   /*Object classification*/
  uint8 uiClassConfidence_per; /*Confidences for all classification*/
} RCW_GenObjAttributes_t;

typedef struct {
  float32 fProbabilityOfExistence_per; /* Probability that the object represents
                                          a real object,unit:0.0-1.0f*/
  uint8 uiHighestAssocProb_per;        /*Highest association probability of all
                                          associated clusters in this cycle*/
  uint8 uiMeasuredTargetFrequency_nu;  /*Bitfield to indicate if the object was
                                          measured in the last 8 cycles*/
  boolean bObjStable;                  /*Flag that object is stable*/
} RCW_GenObjQualifiers_t;

typedef struct {
  float32 fDist2Course_met; /* Object distance to course, unit:m*/
} RCW_GenObjRoadRelation_t;

typedef struct {
  boolean bRightSensor;
  RCW_GenObjKinematics_t Kinemactic;
  RCW_GenObjGeometry_t Geometry;
  RCW_GenObjGenerals_t General;
  RCW_GenObjAttributes_t Attributes;
  RCW_GenObjQualifiers_t Qualifiers;
  RCW_GenObjRoadRelation_t RoadRelation;
} RCW_GenObject_st;

typedef RCW_GenObject_st RCW_GenObjectArray[RCW_INPUT_OBJECT_NUMBER];

typedef struct {
  RCW_GenObjectArray aObject;
} RCWGenObjList_st;

typedef struct {
  float32 fegoVelocity_mps; /*the ego vehicle longitudinal velocity ,unit:m/s*/
  float32 fegoAcceleration_mps2; /*the ego vehicle longitudinal acceleration
                                    ,unit:m/s^2*/
  float32 fLatAccel_mps2;   /*the ego vehicle Lateral Acceleration,unit:m/s^2*/
  float32 fLatVelocity_mps; /*the ego vehicle Lateral velocity ,unit:m/s*/

  uint8 uRCWGearPosition;

  float32 fRCWEgo_steer_angle_rad;
} RCWVehicleInfo_t;

typedef struct {
  boolean bRCWFunctionActive;    /* RCW function active flag */
  boolean bRCWFunctionNVMActive; /* Last cycle RCW function NVM active flag */
  boolean bRCWFunctionOutputActive; /* RCW function output flag */
  // float32 fCycletime_s;            MOVE TO parameter
} RCWSystemSwitch_t;

typedef struct {
  float32 fSensorOffsetToRear_met;
  float32 fSensorOffsetToSide_met;
} RCW_LBSGlobalInfo_t;

typedef struct {
  float32 fXmin_met; /*The object x min position,unit:m*/
  float32 fXmax_met; /*The object x max position,unit:m*/
  float32 fYmin_met; /*The object y min position,unit:m*/
  float32 fYmax_met; /*The object y max position,unit:m*/
} RCWObjBorders_t;

typedef struct {
  RCWObjBorders_t ObjBorders; /*The object border information*/
  float32 fTTC_s;             /*The object TTC time to ego vehicle,unit:s*/
  float32 fTTCAccel_mps2;
  float32 fCycletimeSum_s; /*The object exist life time,unit:s*/
  float32 fUpdateRate_nu;  /*The object measurement update rate,unit:NULL*/
  float32 fXMovement_met;  /*The object total moving distance in the x
                              direction,unit:m*/
  float32 fYMovement_met;  /*The object total moving distance in the y
                              direction,unit:m*/
  float32 fAngle_deg; /*The object current position angle(0-360),unit:degree*/
  float32 fAssocProbFiltered; /*Highest cluster association probability of the
                                 object filter result*/
  boolean bLowTTCAtStart;
  boolean bCreateAdjStableObj;
  uint8 eAssociatedLane;    // Lane Association
  float32 fDistToTraj_met;  // Distance to trajectory
  float32 fVrelToTraj_mps;  // Object's relative velocity to trajectory
} RCW_LBSObjInfo_st;

typedef RCW_LBSObjInfo_st RCW_LBSObjInfo_Array[RCW_INPUT_OBJECT_NUMBER];

typedef struct {
  boolean bLCAMirrorObject;
  boolean bLCAMirrorFrontObject;
  boolean bLCAWarning;  // Flag whether current object is warning
} RCW_LCAObjInfo_t;

typedef RCW_LCAObjInfo_t RCW_LCAObjInfo_Array[RCW_INPUT_OBJECT_NUMBER];

typedef struct {
  boolean bBSDWarning;  // Flag whether current object is warning
} RCW_BSDObjInfo_t;

typedef RCW_BSDObjInfo_t RCW_BSDObjInfo_Array[RCW_INPUT_OBJECT_NUMBER];

typedef struct {
  RCW_LBSGlobalInfo_t LBSGlobalInfo;
  RCW_LBSObjInfo_Array LBSObjInfoList;
  RCW_LCAObjInfo_Array LCAObjInfoList;
  RCW_BSDObjInfo_Array BSDObjInfoList;
} RCW_LBSInputInfo_st;

typedef struct {
  // OnOff
  boolean RCWHmiOpen;
  // failure
  boolean RCWFailure;
  // passive
  boolean LeftTurnLightOpen;
  boolean RightTurnLightOpen;
} RCWPreProcessInput_t;

typedef struct {
  RCWGenObjList_st GenObjList;             /* General EM Object information */
  RCWVehicleInfo_t EgoVehInfo;             /* Vehicle dynamic information */
  RCWSystemSwitch_t RCWSystemSwitch;       /* RCW System Parameter */
  RCW_LBSInputInfo_st LBSInputInfo;        /* LBS information */
  RCWPreProcessInput_t RCWPreProcessInput; /* RCW Hmi Input */
} RCWInReq_st;

/*****************************************************************************
  PARAMTER
*****************************************************************************/
typedef struct {
  float32 fVehicleWidth_met;        /*the vehicle body width,unit:m*/
  float32 fVehicleLength_met;       /*the vehicle body length,unit:m*/
  float32 fVehCenter2FrontAxis_met; /*the vehicle center to front axis center
                                       distance*/
} RCWVehParameter_t;

typedef struct {
  // stationary obj checking threshold
  float32 fEgoVelStationaryCheck;  // 0.1f
  // RCW Warning Level1 / Level2 TTCThreshold
  float32 fTTCThresholdLevel1;  // 4.4f
  float32 fTTCThresholdLevel2;  // 1.4f
  // suppression condition
  float32 fSuppressEgoLongVelMin;    // 0 m/s
  float32 fSuppressEgoLongVelMax;    // 38.88 m/s
  float32 fSuppressEgoLatVelMax;     // - m/s
  float32 fSuppressEgoLatAcceMax;    // - m/s2
  float32 fSuppressRCWBlockingTime;  // 2000 ms
  // corridor blocked
  float32 fCorrBlockedActiveOccMin;    // 0.15f
  float32 fCorrBlockedInActiveOccMin;  // 0.30f
  // choose occupancy
  float32 fOppoOccuAddAdjust;  // 0.35f
  // object warning decision
  float32 fWarnDecisionVrel2TrajMax;          // 1.f
  float32 fWarnDecisionAssocProbMin;          // 0.8f
  float32 fWarnDecisionTTCAdjustThreshold;    // 1.4f
  float32 fWarnDecisionArelXMaxTTCAdjust;     // -0.5f
  float32 fWarnDecisionArelXMinTTCAdjust;     // -1.5f
  float32 fWarnDecisionArelXTTCReduceFactor;  // 0.1f
  // object relevent check para
  float32 fReleventDeactCorrOccuThreshold;  // 0.2f
  float32 fReleventDist2CourseMax;          // -0.1f
  float32 fReleventDist2CourseMin;          // -0.5f
  float32 fReleventMaxXOppSideRelevent;     // -5.f
  float32 fReleventMinXOppSideRelevent;     // -15.f
  uint16 fReleventLifeCnt;                  // 50u
  float32 fReleventMaxDist;                 // -5.f
  float32 fReleventMinXMovement;            // 5.f
  // Corridor check para
  float32 fCorridorMinTimeInCorridorThresh;  // 0.5f
  uint8 uCorridorHitCntThresh;               // 15U
  float32 fCycletime_s;                      // Current task cycle time
  float32 fMinTimeInCorridorThresh;          // 0.5f
  float32 fCorrHitCntSmallObjWidth;          // 2.5f
  // RCW warning range
  float32 fRCWCorrMinSDISIX;  // -50.f
  float32 fRCWCorrMaxSDISIX;  // 0.f
  float32 fRCWCorridorXMid;   // -20.f;
  float32 fRCWCorridorXMin;   // -40.f;
  // General
  float32 fVrelTTCMin;           // 10 * 0.27777 for cyclist change to 10kph
  float32 fVrelTTCMax;           // 30 * 0.27777
  float32 fTTCThreshVrelMin;     // 0.f // 0->1.4
  float32 fTTCThreshVrelMax;     // 1.4f //1.4 ->4.4
  float32 fMinHeadingAngle;      // 2.5f
  float32 fMaxHeadingAngle;      // 6.0f
  float32 fCorrOccWarnEnable;    // 0.3f
  float32 fCorrOccDropThresh;    // 0.1f
  float32 fCorrOccPickupThresh;  // 0.28f
  float32 fCorrOccHitThresh;     // 0.2f
} RCWWarningParameter_t;

typedef struct {
  float32 fLatPos_met;       /*the radar sensor mounting Y position ,unit:m*/
  float32 fLongPos_met;      /*the radar sensor mounting X position ,unit:m*/
  float32 fVertPos_met;      /*the radar sensor mounting Z position ,unit:m*/
  float32 fLongPosToCoG_met; /*the radar sensor mounting position to COG
                                longitudinal distance ,unit:m*/
  float32 fPitchAngle_rad;   /*the radar sensor mounting Pitch angle ,unit:rad*/
  float32 fOrientation_rad;
  float32 fRollAngle_rad; /*the radar sensor mounting Roll angle ,unit:degree*/
  float32 fYawAngle_rad;  /*the radar sensor mounting Yaw angle ,unit:degree*/
} RCWSensorMounting_t;

typedef struct {
  RCWSensorMounting_t SensorLeft;
  RCWSensorMounting_t SensorRight;
} RCWSensorMounting_st;

typedef struct {
  RCWSensorMounting_st SensorMounting;
  RCWVehParameter_t RCWVehParameter;
  RCWWarningParameter_t RCWWarningParameter;
} RCWParam_st;

/*****************************************************************************
  OUTPUT
*****************************************************************************/
typedef struct {
  float32 fHmiXObjectWarning;
  sint32 uHmiRCWWarningID;
  sint32 uHmiRCWWarningActive;
  boolean bHmiRCWHmiOn;
  boolean bHmiRCWFailure;
  float32 rcw_fTTC_s;
} RCWOutPro_st;  // Check with CAN Matrix signal

/*****************************************************************************
  Calculation
*****************************************************************************/
typedef enum {
  RCW_INIT,
  RCW_OK,
} RCWState_t;

typedef struct {
  boolean bCorridorBlocked;
  boolean bCorridorBlockedLastCycle;
  float32 fBlockXPosition;
} RCWStationaryBlocked_t;

typedef struct {
  float32 fTTC;
  float32 fXObjectWarning;
  sint32 uRCWWarningID;
  boolean bRCWWarningActive;
  boolean bRCWWarningActiveLastCycle;
} RCWWarningInfo_t;

typedef struct {
  float32 fXDist;
  float32 fCorridorOccupancy;
  float32 fXMin;
  float32 fXMax;
  float32 fInCorridorTime;
  sint32 uObjID;
} RCWCorridorObserver_t;

typedef RCWCorridorObserver_t RCWCorridorObserver_Arry[RCW_MAX_NOF_CORR_OBJS];

typedef struct {
  boolean bRCWFunctionActionLastCycle;
  RCWState_t eRCWState;
} RCWRunState_t;

typedef struct {
  float32 fTTCThreshold;
  float32 fCorridorOverlap;
  float32 fCorridorOccupancy;
  float32 fObjectOccupancy;
  float32 fCorridorOccThreshold;
  float32 fInCOrridorTime;
  float32 fYBreakThrough;
  float32 fHeadingFiltered;
  uint8 uCorridorHitCnt;
  uint8 uMultiPathCnt;
  boolean bRCWQuality;
  boolean bUpdateRecently;
  boolean bRCWRelevant;
  boolean bInRCWCorridor;
  boolean bHeadingAngleInRange;
  boolean bObjCorridorBlocked;
  boolean bMultiPathObj;
  boolean bRCWWarningConditions;
  boolean bRCWWarning;
  boolean bOppositeOverlap;
} RCW_Info_t;

typedef RCW_Info_t RCW_Info_Array[RCW_INPUT_OBJECT_NUMBER];

typedef enum {
  RCWState_Init = 0U,
  RCWState_passive,
  RCWState_StandBy,
  RCWState_Active,
  RCWState_Failure,
  RCWState_Off
} RCWStateMachine_t;

typedef struct {
  boolean bRCWPassiveCondition;
  boolean bRCWStandByCondition;
  boolean bRCWActiveConditon;
  boolean bRCWFailureCondition;
  boolean bRCWHmiOpen;
} RCWStatusCondition_t;

typedef struct {
  RCWStationaryBlocked_t RCWStationaryBlocker;
  RCWWarningInfo_t RCWWarningInfo;
  RCWCorridorObserver_Arry RCWCorridorObjs;
  RCWRunState_t RCWRunState;
  RCW_Info_Array RCWObjInfoList;
  RCWStateMachine_t RCWstatemachine;           // state machine
  RCWStateMachine_t RCWstatemachineLastCycle;  // state machine last cycle
  RCWStatusCondition_t RCWStatusCondition;     // state condition
  boolean RCWBlockingTimeActive;
} RCWCalculate_st;

typedef struct {
  float32 fCorridorBracketLeft;
  float32 fCorridorBracketRight;
} LBSRCWCorridor_t;

typedef struct {
  float32 fOverlap_met;         /*the object with trace bracket overlap,unit:m*/
  float32 fOverlapVar_met;      /*the object with trace bracket overlap
                                   variance,unit:m*/
  float32 fObjectOccupancy_per; /*the object with trace bracket overlap as a
                                   percentage of the object width,unit:percent*/
  float32 fObjectOccupancyVar_per;  /*the object with trace bracket overlap as
                                       a percentage variance of the object
                                       width,unit:percent*/
  float32 fTrajectoryOccupancy_per; /*the object with trace bracket overlap as
                                       a percentage of the trace bracket
                                       width,unit:percent*/
  float32 fTrajectoryOccupancyVar_per; /*the object with trace bracket overlap
                                          as a percentage variance of the
                                          trace bracket width,unit:percent*/
  boolean bOppositeOverlap_met;
} RCWSITrajOccupancy_t;

/*****************************************************************************
  DEBUG
*****************************************************************************/
typedef struct {
  boolean debugActive_bRCWWarningActive;
  boolean debugActive_bCorridorBlocked;
  boolean debugPassive_VelocityOutOfRange;
  boolean debugPassive_LatAcceloutofRange;
  boolean debugPassive_LeftTurnLight;
  boolean debugPassive_RightTurnLight;
  boolean debugPassive_GearPosition;
  boolean debugPassive_BlockingtimeActive;
  boolean debugHmiOff_bRCWHmiOpen;
  boolean debugFailure_bRCWFailureCondition;
} RCWStatusSubcondition;

typedef struct {
  RCWStateMachine_t Debug_RCWstatemachine;  // state machine
  RCWStatusSubcondition Debug_RCWDebugSubConditions;
  RCWWarningInfo_t Debug_RCWWarningInfo;
  RCW_Info_Array Debug_RCWObjInfo;
  RCWCorridorObserver_Arry Debug_RCWCorridorObjs;
} RCWDebug_t;

/*****************************************************************************
  VARIABLES
*****************************************************************************/
// input port
extern RCWInReq_st RCWReqPorts;
extern RCWParam_st RCWParams;

// Output port
extern RCWOutPro_st RCWProPorts;
extern RCWDebug_t RCWDebugInfo;

/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/
void LBS_RCWExec(const RCWInReq_st* reqPorts, const RCWParam_st* params,
                 RCWOutPro_st* proPorts, RCWDebug_t* debugInfo);
void LBS_RCW_Init_Reset();

#endif