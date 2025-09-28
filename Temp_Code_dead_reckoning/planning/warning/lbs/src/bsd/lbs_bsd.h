#pragma once
#ifndef LBS_BSD_H
#define LBS_BSD_H
/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "../lbs_par.h"
#include "planning/warning/lbs/common/TM_Global_Types.h"

/*****************************************************************************
  CONSTS
*****************************************************************************/
#define LBS_BSD_VERSION_NUMBER (20210117)

#define BSD_EM_GEN_OBJECT_MT_STATE_DELETED 0U

#define BSD_INPUT_OBJECT_NUMBER 80
#define BSD_INPUT_OBJECT_U16ID_MAX LBS_INPUT_OBJECT_U16ID_MAX

#define EM_GEN_OBJECT_DYN_PROPERTY_MOVING (0u)
#define EM_GEN_OBJECT_DYN_PROPERTY_STATIONARY (1u)
#define EM_GEN_OBJECT_DYN_PROPERTY_ONCOMING (2u)
#define EM_GEN_OBJECT_DYN_PROPERTY_STATIONARY_CANDIDATE (3u)
#define EM_GEN_OBJECT_DYN_PROPERTY_UNKNOWN (4u)
#define EM_GEN_OBJECT_DYN_PROPERTY_CROSSING_STATIONARY (5u)
#define EM_GEN_OBJECT_DYN_PROPERTY_CROSSING_MOVING (6u)
#define EM_GEN_OBJECT_DYN_PROPERTY_STOPPED (7u)

#define F32_VALUE_INVALID (1000.0f)
#define C_F32_DELTA (0.0001f)

/*****************************************************************************
  BSD CLASSIFY
*****************************************************************************/
#define BSD_APPEAR_INVALID (0u)
#define BSD_APPEAR_FRONT (1u)
#define BSD_APPEAR_REAR (2u)
#define BSD_APPEAR_SIDE (3u)

#define BSD_CLASS_UNDEFINED (0u)
#define BSD_CLASS_VEH_FRONT (10u)
#define BSD_CLASS_VEH_REAR (20u)
#define BSD_CLASS_VEH_SIDE (30u)
#define BSD_CLASS_VEH_SIDE_TRUCK (31u)
#define BSD_CLASS_VEH_MERGE (50u)
#define BSD_CLASS_VEH_CONFIRMED_REAR (60u)
#define BSD_CLASS_VEH_LAST (99u)
#define BSD_CLASS_STATIC_FRONT (100u)
#define BSD_CLASS_STATIC_REAR (120u)
#define BSD_CLASS_STATIC_SIDE (130u)
#define BSD_CLASS_STATIC_GRDHITCOUNTER (150u)
#define BSD_CLASS_STATIC_GRD (155u)  // no use
#define BSD_CLASS_STATIC_LAST_RECOVERABLE (180u)

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/
typedef struct {
  uint32 uiTimeStamp_ms;
  uint16 uiMeasurementCounter_nu;
  uint16 uiCycleCounter_nu;
  uint8 eSigStatus_nu;
  uint8 a_reserve;
} BSD_SignalHeader_t;

/*****************************************************************************
  INPUT
*****************************************************************************/

typedef struct {
  float32 fDistX_met; /*Object's longitudinal relative distance*/
  float32 fDistY_met; /*Object's lateral relative distance*/
  float32 fVrelX_mps; /*Object's longitudinal relative velocity*/
  float32 fVrelY_mps; /*Object's lateral relative velocity*/
  float32 fArelX_mpss;
  float32 fArelY_mpss;
} BSD_GenObjKinematics_t;

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
} BSD_GenObjGeometry_t;

typedef struct {
  float32
      fFirstDetectX_met; /* X position where the object was created, unit:m*/
  float32
      fFirstDetectY_met; /* Y position where the object was created, unit:m*/
} BSD_SRRObjHistory_t;

typedef struct {
  float32
      fProbabilityOfExistence_per;    /* Probability that the object represents
                                          a real object,unit:0.0-1.0f*/
  uint8 uiHighestAssocProb_per;       /*Highest association probability of all
                                          associated clusters in this cycle*/
  uint8 uiMeasuredTargetFrequency_nu; /*Bitfield to indicate if the object was
                                          measured in the last 8 cycles*/
  boolean bObjStable;                 /*Flag that object is stable*/
} BSD_SRRObjQualifiers_t;

typedef struct {
  float32 fDist2Course_met;  /* Object distance to course, unit:m*/
  float32 fDist2Border_met;  /* Object distance to border, unit:m*/
  boolean bDist2BorderValid; /* Valid flag if the value in Dist2Border can be
                                used */
} BSD_SRRObjRoadRelation_t;

typedef struct {
  float32 fLifeTime_s;    /*Object lifetime in second,unit: s*/
  uint16 uiLifeCycles_nu; /*Object lifetime in cycles,unit: null*/
  uint8
      uiMaintenanceState_nu; /*Maintenance state if object(measured,predicted)*/
  sint32 uiID_nu;            /*Object detected ID ,unit: number,range:0-65536*/
} BSD_GenObjGenerals_t;

typedef struct {
  uint8 eDynamicProperty_nu;   /*Object dynamic property,stationary,moving or
                                  oncoming*/
  uint8 uiDynConfidence_per;   /*General confidence of dynamic
                                  property(moving,crossing,oncoming)*/
  uint32 eClassification_nu;   /*Object classification*/
  uint8 uiClassConfidence_per; /*Confidences for all classification*/
} BSD_GenObjAttributes_t;

typedef struct {
  BSD_GenObjKinematics_t Kinemactic;
  BSD_GenObjGeometry_t Geometry;
  BSD_GenObjGenerals_t General;
  BSD_GenObjAttributes_t Attributes;
  boolean bRightSensor;
} BSD_GenObject_st;

typedef BSD_GenObject_st BSD_GenObjectArray[BSD_INPUT_OBJECT_NUMBER];

typedef struct {
  BSD_GenObjectArray aObject;
} BSDGenObjList_st;

typedef struct {
  BSD_SRRObjHistory_t History;
  BSD_SRRObjQualifiers_t Qualifiers;
  BSD_SRRObjRoadRelation_t RoadRelation;
} BSD_SRRObject_st;

typedef BSD_SRRObject_st BSD_SRRObjectArray[BSD_INPUT_OBJECT_NUMBER];

typedef struct {
  BSD_SRRObjectArray aObject;
} BSDSRRObjList_st;

typedef struct {
  float32 fSensorOffsetToRear_met;
  float32 fSensorOffsetToSide_met;
  boolean bInnerSensorDriven;   /*the ego driver course direction flag*/
  boolean bInnerSensorSteering; /*the ego driver steering direction flag*/

} BSD_LBSGlobalInfo_t;

typedef struct {
  float32 fXmin_met; /*The object x min position,unit:m*/
  float32 fXmax_met; /*The object x max position,unit:m*/
  float32 fYmin_met; /*The object y min position,unit:m*/
  float32 fYmax_met; /*The object y max position,unit:m*/
} BSDObjBorders_t;

typedef struct {
  BSDObjBorders_t ObjBorders; /*The object border information*/
  float32 fTTC_s;             /*The object TTC time to ego vehicle,unit:s*/
  float32 fTTCFiltered_s;  /*The object filter TTC time to ego vehicle,unit:s*/
  float32 fCycletimeSum_s; /*The object exist life time,unit:s*/
  float32 fUpdateRate_nu;  /*The object measurement update rate,unit:NULL*/
  float32 fXMovement_met;  /*The object total moving distance in the x
                              direction,unit:m*/
  float32 fYMovement_met;  /*The object total moving distance in the y
                              direction,unit:m*/
  float32 fAngle_deg; /*The object current position angle(0-360),unit:degree*/
  float32 fAssocProbFiltered; /*Highest cluster association probability of the
                                 object filter result*/
} BSD_LBSObjInfo_st;

typedef BSD_LBSObjInfo_st BSD_LBSObjInfo_Array[BSD_INPUT_OBJECT_NUMBER];

typedef struct {
  boolean bLCAMirrorObject;
  boolean bLCAMirrorFrontObject;
  boolean bLCAWarning;  // Flag whether current object is warning
} BSD_LCAObjInfo_t;

typedef BSD_LCAObjInfo_t BSD_LCAObjInfo_Array[BSD_INPUT_OBJECT_NUMBER];

typedef struct {
  BSD_LBSGlobalInfo_t LBSGlobalInfo;
  BSD_LBSObjInfo_Array LBSObjInfoList;
  BSD_LCAObjInfo_Array LCAObjInfoList;
} BSD_LBSInputInfo_st;

typedef struct {
  boolean bBSDFunctionActive;       /* BSD function active flag */
  boolean bBSDFunctionOutputActive; /* BSD function output flag */
  float32 fCycletime_s;             /* Current task cycle time */
} BSDSystemParam_t;

typedef struct {
  /*CurveInfo_t */
  float32 fCurveRadius_met; /*The curvature radius of the current driver road
                               ,unit:m*/
  float32
      fDrivenCurveRadius_met; /*The curvature radius of the ego driver curve*/

  // Scale coefficient
  // f = (1/2) * C0 * x^2 + (1/6) * C1 * x^3 + Offset
  /*FusedRoadBorder_t */
  float32 fYOffsetFused_met; /*Polynomial coefficient that index is zero -> the
                                offset of the Y Coordinate */
  float32 fConfYOffset_per;  /*The confidence of polynomial coefficient that
                                index is zero*/
  float32 fYOffsetFusedOppBorder_met; /*Opposite polynomial coefficient that
                                         index is zero -> the Opposite offset of
                                         the Y Coordinate*/
  float32 fConfYOppOffset_per;        /*The confidence of opposite polynomial
                                         coefficient that index is zero*/

  /*LaneEstimation_t */
  float32 fLaneWidth_met;       /*The ego lane width ,unit:m*/
  sint8 iNumOfAdjacentLanes_nu; /*undetermined */

  /*GridBorderData_t */
  float32 BorderEstmGridData_fConf_per; /*undetermined */

} BSDRoad_t;

typedef struct {
  float32 fegoVelocity_mps; /*the ego vehicle longitudinal velocity ,unit:m/s*/
  uint8 uBSDGearPosition;
} BSDVehicleInfo_t;

typedef struct {
  BSDGenObjList_st GenObjList;      /* General Radar Object information */
  BSDSRRObjList_st SRRObjList;      /* SRR Radar Object information */
  BSDVehicleInfo_t EgoVehInfo;      /* Vehicle dynamic information */
  BSDRoad_t Road;                   /* Road information */
  BSD_LBSInputInfo_st LBSInputInfo; /* LBS information */
  BSDSystemParam_t BSDSystemParam;  /* BSD System Parameter */
} BSDInReq_st;

/*****************************************************************************
  OUTPUT
*****************************************************************************/

typedef struct {
  uint8 uNumberSoTObjs_nu;          /*current cycle SoT Object*/
  uint8 uNumberSoTObjsLastCycle_nu; /*last cycle SoT Object*/
} BSDScenarioObserver_t;

typedef struct {
  float32 fBSDZoneXmin_met;         /* the BSD Warning Zone X direction min
                                       position,unit:m*/
  float32 fBSDZoneXmax_met;         /* the BSD Warning Zone X direction max
                                       position,unit:m*/
  float32 fBSDZoneYmin_met;         /* the BSD Warning Zone Y direction min
                                       position,unit:m*/
  float32 fBSDZoneYmax_met;         /* the BSD Warning Zone Y direction max
                                       position,unit:m*/
  float32 fBSDZoneXminWithHyst_met; /* the BSD Warning Zone X direction min
                                       position with hysteresis,unit:m*/
  float32 fBSDZoneXmaxWithHyst_met; /* the BSD Warning Zone X direction max
                                       position with hysteresis,unit:m*/
  float32 fBSDZoneYminWithHyst_met; /* the BSD Warning Zone Y direction min
                                       position with hysteresis,unit:m*/
  float32 fBSDZoneYmaxWithHyst_met; /* the BSD Warning Zone Y direction max
                                       position with hysteresis,unit:m*/
} BSDZoneParameter_t;

typedef struct {
  BSDZoneParameter_t BSDZoneParameterLeft;
  BSDZoneParameter_t BSDZoneParameterRight;
} BSDGlobal_BSDZone_st;

typedef struct {
  BSDScenarioObserver_t ScenarioObserver;
  float32 fVxThreshMovStat_mps;  /* real Vx speed threshold for SoTs,unit:m/s*/
  float32 fAngleFrontSector_deg; /* Angle of Front Sector Cut which is adjusted
                                     by turning angle,unit:degree*/
  float32 fBSDZoneXminStatic_met;   /* Actual Xmin (bottom edge) of the BSD
                                     Zone,unit:m*/
  float32 fBSDZoneXmin_met;         /* the BSD Warning Zone X direction min
                                     position,unit:m*/
  float32 fBSDZoneXmax_met;         /* the BSD Warning Zone X direction max
                                     position,unit:m*/
  float32 fBSDZoneYmin_met;         /* the BSD Warning Zone Y direction min
                                     position,unit:m*/
  float32 fBSDZoneYmax_met;         /* the BSD Warning Zone Y direction max
                                     position,unit:m*/
  float32 fBSDZoneXminWithHyst_met; /* the BSD Warning Zone X direction min
                                       position with hysteresis,unit:m*/
  float32 fBSDZoneXmaxWithHyst_met; /* the BSD Warning Zone X direction max
                                       position with hysteresis,unit:m*/
  float32 fBSDZoneYminWithHyst_met; /* the BSD Warning Zone Y direction min
                                       position with hysteresis,unit:m*/
  float32 fBSDZoneYmaxWithHyst_met; /* the BSD Warning Zone Y direction max
                                       position with hysteresis,unit:m*/
  float32 fMinAssocProbFront_nu;    /* Quality check association threshold of
                                       object appear from front*/
  float32 fMinAssocProbSideRear_nu; /* Quality check association threshold of
                                       object appear from side and rear*/
  float32 fMinXmoved_met; /* the object movement distance min threshold  of
                             Plausibility check ,unit:m*/
  float32
      fBSDWarnActiveLeftDistX_met; /* the left side BSD warning most closest
                                          object distance x,unit:m  */
  float32 fBSDWarnActiveRightDistX_met; /* the right side BSD warning most
                                           closest object distance x,unit:m */
  boolean bBSDWarnActive;     /* boolean value to distinguish whether BSD side
                                  warning is active*/
  boolean bBSDWarnActiveLeft; /* boolean value to distinguish whether BSD left
                                  side warning is active*/
  boolean
      bBSDWarnActiveRight; /* boolean value to distinguish whether BSD right
                                  side warning is active*/
  boolean bBSDWarnActiveLeftLastCycle;  /* boolean value to distinguish whether
                                           BSD left side warning is active in
                                           last cycle*/
  boolean bBSDWarnActiveRightLastCycle; /* boolean value to distinguish whether
                                           BSD right side warning is active in
                                           last cycle*/
  sint32 uBSDWarnActiveLeftID_nu;  /* the left side BSD warning most closest
                                     object id */
  sint32 uBSDWarnActiveRightID_nu; /* the right side BSD warning object id */
  boolean bInnerSensorDriven; /* boolean value to distinguish between inner and
                                 outer sensor*/
  boolean bInnerSensorSteering; /* boolean value to distinguish between inner
                                   and outer sensor*/
} BSD_Globals_t;

typedef enum {
  LBS_BSD_LeftSensorObj = 1,
  LBS_BSD_RightSensorObj = 2
} LBSBSDObj_Direction;

typedef struct {
  float32 fSoTDelayTime_s; /* current SOT object delay time,unit:s*/
  float32 fRearConf_nu; /* The object classify to rear confidence,unit:percent*/
  float32
      fBSDZoneObjXmin_met;  /* BSD Zone length per object using NHTSA formula*/
  uint8 ubAppearance_nu;    /* Zone appearance of current object :front ,side,
                               rear,unit:NULL*/
  uint8 ubHitsInFront_nu;   /* The ratio of object appearance from the front
                               direction,unit:NULL*/
  uint8 ubHitsInSide_nu;    /* The ratio of object appearance from the side
                               direction,unit:NULL*/
  uint8 ubHitsInRear_nu;    /* The ratio of object appearance from the rear
                               direction,unit:NULL*/
  uint8 ubGrdHitCounter_nu; /* Counter which increase if the object hits the
                               guardrail next to the vehicle,unit:NULL*/
  uint8
      ubBehindGrdCounter_nu; /* Counter which increase if the object is behind
                                  the guardrail,unit:NULL*/
  uint8 ubClass_nu; /* The object classification:vehicle, static...,unit:NULL*/
  uint8 ubOwnLaneCounter_nu; /* Counter which is increased when an object is
                                assumed to be on the own lane,unit:NULL*/
  boolean bInBSDZone;        /*The object in BSD zone indicator flag */
  boolean bInSOTZone;        /*The object in SOT zone indicator flag */
  boolean
      bInSOTZonePrevious; /*The last cycle object in SOT zone indicator flag */
  boolean bObjectAndZoneOverlap; /*whether Overlap ratio of object and BSD zone
                                    is enough flag*/
  boolean bBSDRelevant;          /*The object relevance flag of BSD function */
  boolean bBSDWarning;           /*The object warning flag */
  boolean bUpdatedRecently;      /*Whether the current object is sufficiently
                                    updated(measured) flag in the last two cycle*/
  boolean bUpdatedRecentlyWeak;  /*Whether the current object is sufficiently
                                    updated(measured) flag at least one of last
                                    two cycle*/
  boolean
      bLivedLongEnough; /*Flag which shows that this object lived long enough */
  boolean bQualityEnough; /*Flag which shows that this object quality enough */
  boolean bObjectOnOwnlane; /*Object is on own lane flag*/
  boolean bCreateBehindGRD; /*if the object is relevance with the guardrail in
                               the first cycle time*/
  boolean bObjectBehindGRD; /* Flag whether object is behind guardrail*/
  boolean bSoTDelayActive;  /*SoT delay flag of the object*/
  boolean bShortWarn;       /*ShortWarning flag of the object*/
  boolean bIsSoT;           /*SoT flag of the object*/
  boolean bFastSoT;         /*Fast SoT flag of the object*/
  boolean bPlausibility;    /*Plausibility check flag of the object*/
  boolean bPossibleWrappedObj; /*Possible wrapped flag of the object*/
  LBSBSDObj_Direction
      eObjDirection; /*The object on the right side or left side
                                        flag by sensor */
} BSD_Info_t;

typedef BSD_Info_t BSD_Info_Array[BSD_INPUT_OBJECT_NUMBER];

typedef struct {
  uint32 uLBSBSDStartTime; /*the BSD function start time*/
  uint32 uLBSBSDEndTime;   /*the BSD function end time*/
  uint32 uLBSBSDRunTime;   /*the BSD function run time*/
} BSD_RunTime_t;

typedef struct {
  float32 fZoneXmin_met; /* The current object BSDZone X min position,unit:m*/
  float32 fZoneXmax_met; /* The current object BSDZone X min position,unit:m*/
  float32 fZoneYmin_met; /* The current object BSDZone Y max position,unit:m*/
  float32 fZoneYmax_met; /* The current object BSDZone Y max position,unit:m*/
} BSDZone_ObjPar;

typedef BSDZone_ObjPar BSDZone_ObjPar_Array[BSD_INPUT_OBJECT_NUMBER];

typedef struct {
  BSD_RunTime_t BsdRuntime;
  BSD_Globals_t BSD_Globals;
  BSDZone_ObjPar_Array BSDZoneObjParList;
} BSDOutPro_st;

/*****************************************************************************
  PARAMTER
*****************************************************************************/

typedef struct {
  float32 fXmin_met; /*the BSD Warning Zone X direction min position unit:m*/
  float32 fXmax_met; /*the BSD Warning Zone X direction max position unit:m*/
  float32 fYmin_met; /*the BSD Warning Zone Y direction min position unit:m*/
  float32 fYmax_met; /*the BSD Warning Zone Y direction max position unit:m*/
  float32 fHysteresisX_met; /*the BSD Warning Zone X direction hysteresis value
                               unit:m*/
  float32 fHysteresisYmin_met; /*the BSD Warning Zone Y direction hysteresis
                                  value unit:m*/
  float32 fHysteresisYmax_met; /*the BSD Warning Zone Y direction hysteresis
                                  value unit:m*/
} BsdZone_t;

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
} BSDSensorMounting_t;

typedef struct {
  BSDSensorMounting_t SensorLeft;
  BSDSensorMounting_t SensorRight;

} BSDSensorMounting_st;

typedef struct {
  float32 fWheelBase_met; /*The distance between the center of the front wheel
                             and the center of the rear wheel,unit:m*/
  float32 fVehicleWidth_met;        /*the vehicle body width,unit:m*/
  float32 fVehicleLength_met;       /*the vehicle body length,unit:m*/
  float32 fVehCenter2FrontAxis_met; /*the vehicle center to front axis center
                                       distance*/
  float32 fVehRear2FrontAxis_met;   /*the vehicle rear position to front axis
                                       center distance*/
} BSDVehParameter_t;

typedef struct {
  boolean bNCAPActive;
  float32 fSoTDelayThresh_s; /*the SOT object delay time max threshold,unit:s */
  float32 fSoTMinWarnDuration_s; /*the short warning duration time
                                    threshold,unit:s */
  float32 fSoTCutoffSpeed_mps; /*the object cutoff speed threshold of SOT check
                                  ,unit:m/s */

  float32 fVelMinWarnDisable_mps; /*BSD function min disable velocity
                                     threshold,unit:m/s*/
  float32 fVelMinWarnEnable_mps;  /*BSD function min enable velocity
                                     threshold,unit:m/s*/
} BSDWarningParameter_t;

typedef struct {
  BsdZone_t BsdZone;
  BSDSensorMounting_st SensorMounting;
  BSDVehParameter_t BSDVehParameter;
  BSDWarningParameter_t BSDWarningParameter;

} BSDParam_st;

/*****************************************************************************
  DEBUG
*****************************************************************************/

typedef struct {
  uint32 uiVersionNumber;
  BSD_Info_Array BSDObjInfo;
} BSDDebug_t;

/*****************************************************************************
  Calculation
*****************************************************************************/
typedef enum {
  BSD_INIT,
  BSD_OK,
} BSDState_t;

typedef struct {
  boolean bBSDFunctionActionLastCycle;
  BSDState_t eBSDState;
} BSDRunState_t;

typedef sint32 LastCycleObjID_Array[BSD_INPUT_OBJECT_NUMBER];

typedef struct {
  BSDZone_ObjPar_Array BSDZoneObjParList;
  BSD_Info_Array BSDObjInfoList;
  LastCycleObjID_Array LastObjIDList;
  BSD_Globals_t BSD_Globals;
  BSDGlobal_BSDZone_st BSDGlobalsZones;
  BSDRunState_t BSDRunState;
} BSDCalculate_st;

#endif