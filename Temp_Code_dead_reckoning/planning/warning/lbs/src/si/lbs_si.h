#pragma once
#ifndef LBS_SI_H
#define LBS_SI_H
#ifdef __cplusplus
extern "C" {
#endif
/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "planning/warning/lbs/common/TM_Global_Types.h"
#include "planning/warning/lbs/src/lca/lbs_lca.h"

#define SI_INPUT_OBJECT_NUMBER (80u)

/*****************************************************************************
  ENUM
*****************************************************************************/
typedef enum {
  DEFAULT_MODE,     /*Default mode:set to given value*/
  RESTRICTION_MODE, /*Trace bracket restriction mode:default init to left:-max
                       right:+max when updating,take smaller of bracket
                       values(restrict)*/
  EXTENSION_MODE    /*Trace bracket restriction mode:default init to left:+max
                       right:-max    when updating,take greater of bracket
                       values(restrict)*/
} SITraceBracketMode_t;

typedef enum {
  SI_INIT,
  SI_OK,
} SIState_t;

/*****************************************************************************
  INPUT
*****************************************************************************/
typedef struct {
  uint32 uiTimeStamp_ms;
  uint16 uiMeasurementCounter_nu;
  uint16 uiCycleCounter_nu;
  uint8 eSigStatus_nu;
} SISignalHeader_t;

typedef struct {
  float32 fCycletime_s; /* Current task cycle time,unit: s*/
} SISystemParam_t;

typedef struct {
  boolean
      bRightSensor; /*Flag whether the object is detected by the right sensor*/
  float32 fDistX_met;      /*Object's longitudinal relative distance*/
  float32 fDistY_met;      /*Object's lateral relative distance*/
  float32 fVrelX_mps;      /*Object's longitudinal relative velocity*/
  float32 fWidthLeft_met;  /*Object's width left of the track position(left
                              sensor view)*/
  float32 fWidthRight_met; /*Object's width right of the track position(left
                              sensor view)*/
  uint16 uiLifeCycles_nu;  /*Object lifetime in cycles,unit: null*/
  uint8
      uiMaintenanceState_nu; /*Maintenance state if object(measured,predicted)*/
  uint8 eDynamicProperty_nu; /*Object dynamic property,stationary,moving or
                                oncoming*/
  uint32 eClassification_nu; /*Object classification*/
  float32
      fFirstDetectX_met; /* X position where the object was created, unit:m*/
  float32 fProbabilityOfExistence_per; /* Probability that the object represents
                                          a real object,unit:0.0-1.0f*/
  float32 fDist2Course_met; /* Object distance to the driven ego trace, unit:m*/
} SIGenObjInfo_t;

typedef struct {
  SIGenObjInfo_t GenObjInfo;
} SIGenObject_st;

typedef SIGenObject_st SIGenObjectArray[LCA_INPUT_OBJECT_NUMBER];

typedef struct {
  uint32 uiVersionNumber;
  SISignalHeader_t sSigHeader;
  SIGenObjectArray aObject;
} SIGenObjList_st;

typedef struct {
  SISignalHeader_t sSigHeader;
  float32 fegoVelocity_mps; /*the ego vehicle longitudinal velocity ,unit:m/s*/
  float32 fegoAcceleration_mps2; /*the ego vehicle longitudinal acceleration
                                    ,unit:m/s^2*/
} SIVehicleInfo_t;

typedef struct {
  /*CurveInfo_t */
  float32 fCurveRadius_met; /*The curvature radius of the current driver road
                               ,unit:m*/
  float32
      fDrivenCurveRadius_met; /*The curvature radius of the ego driver curve*/

  // Scale cofficients
  // f = (1/2) * C0 * x^2 + (1/6) * C1 * x^3 + Offset
  /*FusedRoadBorder_t */
  float32 fC0Fused_1pm;       /*Polynomial coefficient that index is two  */
  float32 fC1Fused_1pm2;      /*Polynomial coefficient that index is three */
  float32 fYOffsetFused_met;  /*Polynomial coefficient that index is zero -> the
                                 offset of the Y Coordinate */
  float32 fYawAngleFused_rad; /*undetermined */
  // float32 fConfCurvature_per;                  /*undetermined */
  float32 fConfYOffset_per; /*The confidence of polynomial coefficient that
                               index is zero*/
  float32 fYOffsetFusedOppBorder_met; /*Opposite polynomial coefficient that
                                         index is zero -> the Opposite offset of
                                         the Y Coordinate*/
  float32 fConfYOppOffset_per;        /*The confidence of opposite polynomial
                                         coefficient that index is zero*/

  /*LaneEstimation_t */
  float32 fConfAdjacentLanes_per; /*undetermined */
  float32 fConfOppositeLanes_per; /*undetermined */
  float32 fLaneWidth_met;         /*The ego lane width ,unit:m*/
  sint8 iNumOfAdjacentLanes_nu;   /*undetermined */
  sint8 iNumOfOppositeLanes_nu;   /*undetermined */
} SIRoad_t;

typedef struct {
  boolean bLCAWarning;  // Flag whether current object is warning
} SILCAObjInfo_t;

typedef SILCAObjInfo_t SILCAObjInfo_Array[LCA_INPUT_OBJECT_NUMBER];

typedef struct {
  float32 fUpdateRate_nu; /*The object measurement update rate,unit:NULL*/
} SILBSObjInfo_st;

typedef SILBSObjInfo_st SILBSObjInfo_Array[SI_INPUT_OBJECT_NUMBER];

typedef struct {
  SIGenObjList_st GenObjList;        /* General Radar Object information */
  SIVehicleInfo_t EgoVehInfo;        /* Vehicle dynamic information */
  SIRoad_t Road;                     /* Road information */
  SILCAObjInfo_Array LCAObjInfoList; /*LCA information*/
  SILBSObjInfo_Array LBSObjInfoList; /*LBS information*/
  SISystemParam_t SISysParam;        /*LBS System parameter*/
} SIInPut_st;

typedef struct {
  SIGenObjList_st* pGenObjList; /* General Radar Object information */
  SIVehicleInfo_t* pEgoVehInfo; /* Vehicle dynamic information */
  SIRoad_t* pRoad;              /* Road information */
  float32 fCycletime_s;
} SLACInReq_t;

/*****************************************************************************
  OUTPUT
*****************************************************************************/

typedef struct {
  float32 fSIseekLaneWidth_met;  /*Seek mode lane width,unit:m*/
  float32 fSITrackLaneWidth_met; /*Track mode lane width,unit:m*/
  float32 fLaneWidth_met;        /*Base lane width,estimated by EM Road,unit:m*/
  float32 fCurveRadiusMinFiltered_met; /*The minimum filter curve radius */
  boolean bLaneChange;                 /*The lane change flag*/
  float32 fCycletime_s;                /* Current task cycle time,unit: s*/
} SI_Globals_t;

typedef struct {
  float32 fOverlap_met; /*the object with trace bracket overlap,unit:m*/
  float32
      fOverlapVar_met; /*the object with trace bracket overlap variance,unit:m*/
  float32 fObjectOccupancy_per;     /*the object with trace bracket overlap as a
                                       percentage of the object width,unit:percent*/
  float32 fObjectOccupancyVar_per;  /*the object with trace bracket overlap as a
                                       percentage variance of the object
                                       width,unit:percent*/
  float32 fTrajectoryOccupancy_per; /*the object with trace bracket overlap as a
                                       percentage of the trace bracket
                                       width,unit:percent*/
  float32 fTrajectoryOccupancyVar_per; /*the object with trace bracket overlap
                                          as a percentage variance of the trace
                                          bracket width,unit:percent*/
  boolean bOppositeOverlap_met;
} SITrajOccupancy_t;

typedef struct {
  float32 fTrajC0_1pm;    /*The ego vehicle trajectory curvature,unit:1/m*/
  float32 fTrajC1_1pmm;   /*The ego vehicle trajectory curvature
                             derivative,unit:1/m^2*/
  float32 fTrajAngle_rad; /*The ego vehicle trajectory heading angle(yaw
                             angle),unit:rad*/
} SITrajectoryData_t;

typedef struct {
  float32
      fBracketPositionLeft_met; /*right side trace bracket position,unit:m */
  float32
      fBracketPositionRight_met; /*left side trace bracket position,unit:m */
} SIBracketOutput_t;

typedef struct {
  SIBracketOutput_t TrackWidthSeek;
  SIBracketOutput_t TrackWidthTrck;
  SIBracketOutput_t RestrictionDistDependent;
  SIBracketOutput_t RestrictionCurve;
  SIBracketOutput_t ExtensionRoadBorder;
  SIBracketOutput_t ExtensionCriticalTTC;
} SICriteriaMatrix_t;

typedef enum SILaneStateTag {
  OBJ_STATE_INLANE = 0, /*Object is in lane*/
  OBJ_STATE_OUTLANE = 1 /*Object is not in adjacent lane*/
} SILaneState_t;

typedef struct {
  SILaneState_t SIInlaneState; /*Lane assignment (confirmed with all criteria)*/
  SILaneState_t SIActLaneState;    /*Lane assignment without timer and distance
                                      criteria(unconfirmed)*/
  uint8 uIn2OutlaneTransition_nu;  /*Object change from in lane to out lane
                                      timers,unit:nu*/
  uint8 uInlaneCycleCounter_nu;    /*Object in lane cycle counter,if object in
                                      lane,the counter will be increase,unit:nu*/
  float32 fCorridorRelevantTime_s; /*Time the object has been inside
                                      corridor,unit:s*/
  float32 fCorridorRelevantDist_met; /*Distance the object has been inside
                                        corridor,unit:met*/
} SIObjLaneState_t;

typedef struct {
  float32
      fRelTraceExtensionFactor_nu; /*Factor for widening seek lane width to
                                      relevant obj track lane width depending on
                                      time and distance,range:0-1*/
  float32 fRelTraceDistExtensionFactor_nu; /*Factor for widening seek lane width
                                              to relevant obj track lane width
                                              depending on distance
                                              difference,range:0-1*/
  float32 fTraceBracketOffsetLeft_met;  /*Left trace bracket offset,unit:met*/
  float32 fTraceBracketOffsetRight_met; /*Right trace bracket offset,unit:met*/
} SIObjCorridor_t;

typedef struct {
  float32 fX_met;          /*Trajectory reference point X coordinate,unit:m*/
  float32 fY_met;          /*Trajectory reference point Y coordinate,unit:m*/
  float32 fDistToTraj_met; /*Distance to trajectory,unit:m*/
  float32 fDistOnTraj_met; /*Distance from vehicle center of gravity to object
                              on trajectory,unit:m*/
} SITrajRefPoint_t;

typedef struct {
  float32
      fTraceBracketLeft_met; /*The trace bracket left side coordinate,unit:m*/
  float32
      fTraceBracketRight_met; /*The trace bracket right side coordinate,unit:m*/
} SITraceBracket_t;

typedef struct {
  boolean bInLOccValue;    /*Occupancy Inlane check flag*/
  boolean bInLCustomValue; /*Custom Inlane check flag*/
  boolean bInLQualityValue;
  boolean bInLObjOccValue;  /*Inlane overlap as a percentage of object occupancy
                               check flag*/
  boolean bInLLaneOccValue; /*Inlane overlap as a percentage of trace bracket
                               occupancy check flag*/
  boolean bInLLaneOverlapValue; /*Inlane overlap check flag*/
  boolean bInLTimeValue;        /*Inlane time criteria check flag*/
  boolean bOutLOccValue;        /*Occupancy outlane check flag*/
  boolean bOutLCustomValue;     /*Custom outlane check flag*/
  boolean bOutLObjOccValue;     /*Outlane overlap as a percentage of object
                                   occupancy check flag*/
  boolean bOutLLaneOccValue; /*Outlane overlap as a percentage of trace bracket
                                occupancy check flag*/
  boolean bOutLLaneOverlapValue; /*Outlane overlap check flag*/

} SIBoolTag;

typedef struct {
  SIBoolTag SIBool;
  SIObjLaneState_t ObjLaneLCAStatus;
  SIObjCorridor_t ObjCor;
  SITrajRefPoint_t ObjTrajRefPoint;
  SITraceBracket_t ObjTraceBracket; /*Distance from trace bracket left and right
                                       side to trajectory*/
  float32 fTraceBracket_met;
  float32 fObjBracketOverlap_met;   /*Overlap between object and trace bracket*/
  float32 fVrelToTraj_mps;          /*Object's relative velocity to trajectory*/
  uint8 uInlanePredictionTimer_nu;  /*Predict the number of time inlane*/
  uint8 uOutlanePredictionTimer_nu; /*Predict the number of time outlane*/
} SI_Info_st;

typedef uint8 eAssociatedLane_t;

typedef SI_Info_st SI_Info_Array[SI_INPUT_OBJECT_NUMBER];
typedef eAssociatedLane_t eAssociatedLane_Array[SI_INPUT_OBJECT_NUMBER];

/* Predicted distance structure with variances */
typedef struct {
  float32 pdist;              /*The predicted distance*/
  float32 pdist_var;          /*The predicted distance variance*/
  float32 pdist_var_fullpred; /*A variance corrected predicted distance*/
} SIPredictedDistance_t;

typedef struct {
  uint8 eAssociatedLane;           // Lane Association
  float32 fVrelToTraj_mps;         // Object's relative velocity to trajectory
  float32 fDistToTraj_met;         // Distance to trajectory
  float32 fTraceBracketLeft_met;   // The trace bracket left side coordinate
  float32 fTraceBracketRight_met;  // The trace bracket right side coordinate
  float32 fObjBracketOverlap_met;  // Overlap between object and trace bracket
} SIObjInfo_st;

typedef SIObjInfo_st SIObjInfo_Array[SI_INPUT_OBJECT_NUMBER];

typedef struct {
  SI_Globals_t SIGlobal;
  // eAssociatedLane_Array SILaneInfoList; /*Associated lane for each object*/
  SIObjInfo_Array SIObjInfoList;
} SIOutPut_st;

/*****************************************************************************
  PARAMTER
*****************************************************************************/

typedef struct {
  float32 fLCAZoneXMid_met;     /* LCA zone Xmid */
  float32 fLCAZoneXMin_met;     /* LCA zone Ymin */
  float32 fLCAZoneYMinNear_met; /* LCA near zone Ymin */
  float32 fLCAZoneYMinFar_met;  /* LCA far zone Ymin */
  float32 fLCAZoneYMaxNear_met; /* LCA near zone Ymax */
  float32 fLCAZoneYMaxFar_met;  /* LCA far zone Ymax */
} SI_LCAZone_t;

typedef struct {
  float32 fDefaultLaneWidth;
  SI_LCAZone_t LCAZone;
} SI_LCAParameter_st;

typedef struct {
  float32 fWheelBase_met; /*The distance between the center of the front wheel
                             and the center of the rear wheel,unit:m*/
  float32 fVehicleWidth_met;        /*the vehicle body width,unit:m*/
  float32 fVehicleLength_met;       /*the vehicle body length,unit:m*/
  float32 fVehCenter2FrontAxis_met; /*the vehicle center to front axis center
                                       distance*/
} SIVehParameter_t;

typedef struct {
  float32 fLatPos_met;       /*the radar sensor mounting Y position ,unit:m*/
  float32 fLongPos_met;      /*the radar sensor mounting X position ,unit:m*/
  float32 fVertPos_met;      /*the radar sensor mounting Z position ,unit:m*/
  float32 fLongPosToCoG_met; /*the radar sensor mounting position to COG
                                longitudinal distance ,unit:m*/
  float32 fPitchAngle_rad;   /*the radar sensor mounting Pitch angle ,unit:rad*/
  float32 fOrientation_rad;
  float32 fRollAngle_rad; /*the radar sensor mounting Roll angle ,unit:rad*/
  float32 fYawAngle_rad;  /*the radar sensor mounting Yaw angle
                             ,unit:rad,range[-pi,+pi]*/
} SISensorMounting_t;

typedef struct {
  // LBS_SensorMounting_t SensorFrontLeft;
  // LBS_SensorMounting_t SensorFrontRight;
  SISensorMounting_t SensorLeft;
  SISensorMounting_t SensorRight;
} SISensorMounting_st;

typedef struct {
  SISensorMounting_st SensorMounting;
  SIVehParameter_t VehParameter;
  SI_LCAParameter_st LCAParamter;
} SIParam_st;

/*****************************************************************************
  DEBUG
*****************************************************************************/
typedef struct {
  SI_Info_Array SIObjInfoList;
} SIDebug_st;

/*****************************************************************************
  Calculation
*****************************************************************************/

typedef struct {
  SI_Globals_t SIGlobals;
  SI_Info_Array SIObjInfoList;
  eAssociatedLane_Array eAssociatedLaneList; /*Lane Association*/
  SIPredictedDistance_t SIPredictedDist;
} SICalculate_st;

#ifdef __cplusplus
}
#endif  // __cplusplus
#endif