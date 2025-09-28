#pragma once
#ifndef LBS_CALCULATE_H
#define LBS_CALCULATE_H

/*****************************************************************************
  INCLUDES
*****************************************************************************/
//#include "lbs.h"
#include "lbs_external.h"
#include "lbs_par.h"
#include "planning/warning/lbs/common/TM_Global_Types.h"

/*****************************************************************************
  VARIABLES
*****************************************************************************/

/*****************************************************************************
  Calculation
*****************************************************************************/

typedef struct {
  boolean bLBSFunctionActionLastCycle;
  LBSState_t eLBState;
} LBSRunState_t;

typedef sint32 LBSLastCycleObjID_Array[LBS_INPUT_OBJECT_NUMBER];

typedef LBS_SRRObjHistory_t LBSObjHistory_Array[LBS_INPUT_OBJECT_NUMBER];

/*****************************************************************************
  LBS COMMOM INPUT
*****************************************************************************/
typedef struct {
  float32 fXmin_met; /*The object x min position,unit:m*/
  float32 fXmax_met; /*The object x max position,unit:m*/
  float32 fYmin_met; /*The object y min position,unit:m*/
  float32 fYmax_met; /*The object y max position,unit:m*/
} LBSObjBorders_t;

typedef struct {
  boolean bObjectFastEnough;
  boolean bBreakthroughHit;
  boolean bObjInRange;
} LBSObjectSelect_t;

typedef struct {
  LBSObjBorders_t ObjBorders;         /*The object border information*/
  LBSObjBorders_t ObjMovementBorders; /**/
  LBSObjectSelect_t ObjSel;
  float32 fTTC_s;         /*The object TTC time to ego vehicle,unit:s*/
  float32 fTTCAccel_mps2; /**/
  float32 fTTCFiltered_s; /*The object filter TTC time to ego vehicle,unit:s*/
  float32 fTTCRadial_s;
  float32 fVabs_mpss;
  float32 fRangeRadial;
  float32 fXLastCycle_met;
  float32 fYLastCycle_met;
  float32 fVxPosBased;
  float32 fVyPosBased;
  float32 fSpeedFiltered_mpss;
  float32 fCycletimeSum_s; /*The object exist life time,unit:s*/
  float32 fUpdateRate_nu;  /*The object measurement update rate,unit:NULL*/
  float32 fXMovement_met;  /*The object total moving distance in the x
                              direction,unit:m*/
  float32 fYMovement_met;  /*The object total moving distance in the y
                              direction,unit:m*/
  float32 fAngle_deg; /*The object current position angle(0-360),unit:degree*/
  float32 fAssocProbFiltered; /*Highest cluster association probability of the
                                 object filter result*/
  float32 fObjWidthMax;
  float32 fObjLengthMax;
  sint32 uUniqueID;
  sint32 uLastMergedObjID;
  boolean bLowTTCAtStart;
  boolean bCreateAdjStableObj;
  uint8 firstStableDynProp;
} LBSObjInfo_st;

typedef LBSObjInfo_st LBSObjInfo_Array[LBS_INPUT_OBJECT_NUMBER];

typedef struct {
  uint32 uiVersionNum_nu;  // uint32 value example
  //...
} LBSSICalculate_t;

typedef struct {
  uint32 uiVersionNum_nu;     // uint32 value example
  boolean bBSDWarnActiveLeft; /* boolean value to distinguish whether BSD left
                                  side warning is active*/
  boolean
      bBSDWarnActiveRight; /* boolean value to distinguish whether BSD right
                                  side warning is active*/
                           //...
} LBSBSDCalculate_t;

typedef struct {
  uint32 uiVersionNum_nu;  // uint32 value example
  boolean bLCAWarnActive;
  boolean bLCAWarnActiveLeft; 
                                  
  boolean
      bLCAWarnActiveRight; 
  sint32 uLCAWarningID_nu;
  float32 fXObjectWarning_met;
  float32 fCriticalTTC_s;
  float32 fFMObjRate;

} LBSLCACalculate_t;

typedef enum {
  OSE_STATE_PASSIVE,
  OSE_STATE_ACTIVE,
  OSE_STATE_LOCKED,
  OSE_NUM_WARN_STATES
} LBSOSEWarnState_t;

typedef struct {
  boolean bOSEWarnActive[LBS_OSE_NUM_OF_WARN_LEVELS];  // Flag whether OSE
                                                       // warning is actived
  boolean bWarningInterrupt;  // Flag whether ose warning is interruped
  LBSOSEWarnState_t OSEWarnState[LBS_OSE_NUM_OF_WARN_LEVELS];
  float32 WarningTimer[LBS_OSE_NUM_OF_WARN_LEVELS];
  sint32 uCriticalObjID;  // Critical object ID
  float32 fCriticalTTC;   // Critical object's TTC
  boolean bCriticalObsIsRight;
} LBSOSECalculate_t;

typedef struct {
  uint32 uiVersionNum_nu;        // uint32 value example
  boolean bBSDWarningLastCycle;  // Flag whether BSD is warning in last cycle
  boolean bBSDWarningLeftLastCycle;  // Flag whether BSD is warning in last
                                     // cycle
  boolean
      bBSDWarningRightLastCycle;  // Flag whether BSD is warning in last cycle
  boolean bLCAWarningLastCycle;   // Flag whether LCA is warning in last
  boolean bLCAWarningLeftLastCycle;
  boolean bLCAWarningRightLastCycle;
  boolean bOSEWarningLastCycle[LBS_OSE_NUM_OF_WARN_LEVELS];  // Flag whether OSE
                                                             // is warning in
                                                             // last cycle
  //...
} LBSWarningLastCycle_t;

typedef struct {
  float32 fBSDZoneXMin_met;  // BSD zone xmin from BSD global
} LBSBSDObjInfo_t;

typedef LBSBSDObjInfo_t LBSBSDObjInfo_Array[LBS_INPUT_OBJECT_NUMBER];

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
} LBSLCAObjInfo_t;

typedef LBSLCAObjInfo_t LBSLCAObjInfo_Array[LBS_INPUT_OBJECT_NUMBER];

typedef struct {
  uint8 eAssociatedLane;           // Lane Association
  float32 fVrelToTraj_mps;         // Object's relative velocity to trajectory
  float32 fDistToTraj_met;         // Distance to trajectory
  float32 fTraceBracketLeft_met;   // The trace bracket left side coordinate
  float32 fTraceBracketRight_met;  // The trace bracket right side coordinate
  float32 fObjBracketOverlap_met;  // Overlap between object and trace bracket

} LBSSIObjInfo_t;

typedef LBSSIObjInfo_t LBSSIObjInfo_Array[LBS_INPUT_OBJECT_NUMBER];

typedef struct {
  float32 fBTHitHystTimer;
  uint8 uBreakthroughHitConfi[LBS_OSE_NUM_OF_BREAK_LINES];  // Breakthrough hit
                                                            // confidence
  boolean
      bBreakthroughHit[LBS_OSE_NUM_OF_BREAK_LINES];  // Flag whether the object
                                                     // hits breakthrough
  boolean bWarning;           // Flag whether the OSE warning is actived
  boolean bWarningLastCycle;  // Flag whether the OSE warning was actived in the
                              // last cycle
  boolean bObjectInRange;  // Flag whether the object is in the range of three
                           // different level
  boolean bTTCBelowThresh[LBS_OSE_NUM_OF_BREAK_LINES];  // Flag whether the
                                                        // object's TTC is below
                                                        // TTC Threshold
  boolean bBTHitHystActive;  // Flag whether the warning hysteresis
} LBSOSEObjInfoLevelOutPro_t;

typedef struct {
  uint16 uCounters[3];
  float32 fValue_met;  // The object width in the algorithm
} LBSOSEWidthEstimOutPro_t;

typedef struct {
  LBSOSEObjInfoLevelOutPro_t
      InfoLevel[LBS_OSE_NUM_OF_WARN_LEVELS];  // DOW warning information
                                              // structure
  float32
      fYBreakthrough[LBS_OSE_NUM_OF_BREAK_LINES];  // The y-axis breakthrough of
                                                   // the object
  float32
      fYBreakthroughStd[LBS_OSE_NUM_OF_BREAK_LINES];  // The y-axis breakthrough
                                                      // standard deviation of
                                                      // the object
  float32 fTTC_s[LBS_OSE_NUM_OF_BREAK_LINES];         // TTC of the object
  float32
      fTTCFiltered_s[LBS_OSE_NUM_OF_BREAK_LINES];  // Filtered TTC of the object
  float32 fDistToCrossingLine_met
      [LBS_OSE_NUM_OF_BREAK_LINES];  // Distance between object front edge and
                                     // ego vehicle rear edge
  float32 fSideTrackProb;            // The probability of the side object
  // float32 fObjBreakthroughMargin[OSE_NUM_OF_BREAK_LINES];
  boolean bRelevant;  // Flag whether the object is relevant for a OSE warning
  boolean bMirror;    // Flag whether the object is mirror
  boolean bSideTrack;
  boolean bObjectFromRear;  // Flag whether the object is approaching from the
                            // rear in the current cycle or first time
  boolean bValidApproachAngle;  // Flag whether the object heading angle is
                                // valid
  boolean bObjectAtEdgeFoV;     // Flag whether the object is at the edge of FOV
  boolean bShortTTC;
  boolean bUpdatedRecently;  // Flag whether the object is updated recently
  float32 fQuality;  // The quality probability of the DOW warning object
  LBSOSEWidthEstimOutPro_t fEstWidth;
} LBSOSEObjInfo_t;

typedef LBSOSEObjInfo_t LBSOSEObjInfo_Array[LBS_INPUT_OBJECT_NUMBER];

typedef LBS_SRRObjRoadRelation_t
    LBS_SRRObjRoadRelation_Array[LBS_INPUT_OBJECT_NUMBER];

typedef struct {
  // LBS_LCA_Globals_st LCA_Globals;
  LBS_SRRObjRoadRelation_Array RoadRelation;
  LBSLastCycleObjID_Array LastObjIDList;
  LBSObjHistory_Array LBSObjHistoryList;
  LBSRunState_t LBSRunState;

  LBS_Globals_t LBS_Globals;
  LBSObjInfo_Array LBSObjInfoList;
  LBSSICalculate_t LBSSICalc;
  LBSSIObjInfo_Array SIObjInfoList;
  LBSBSDCalculate_t LBSBSDCalc;
  LBSBSDObjInfo_Array BSDObjInfoList;
  LBSLCACalculate_t LBSLCACalc;
  LBSLCAObjInfo_Array LCAObjInfoList;
  LBSOSECalculate_t LBSOSECalc;
  LBSOSEObjInfo_Array OSEObjInfoList;
  LBSWarningLastCycle_t LBSWarnLastCycle;

} LBSCalculate_st;

extern LBSCalculate_st LBSCalculate;
/*****************************************************************************
  TYPEDEF
*****************************************************************************/

/*****************************************************************************
  CALCULATION FUNCTION PROTOTYPES
*****************************************************************************/

void LBSInitObjCalArray(const EMGenObjList_st* pGenObjList);
void LBSFCTObjSelSetParameter();
void LBSCalculateGlobalProperties(const LBSInReq_st* reqPorts,
                                  const LBSParam_st* params,
                                  LBSOutPro_t* proPorts, LBSDebug_t* debugInfo);
void LBSCalculateObjectProperties(const LBSInReq_st* reqPorts,
                                  const LBSParam_st* params,
                                  LBSOutPro_t* proPorts, LBSDebug_t* debugInfo);
void LBSCalculateCheckInnerSensor(const EMRoad_t* pRoad);

void LBSCalculateCycleTimeSum(uint8 uObjectIndex,
                              const LBSSystemParam_t* pLBSSystemPar);
void LBSCalculateObjectBorder(uint8 uObjectIndex,
                              const LBS_GenObject_st* pGenObj);
void LBSCalculateObjectMovementBorders(uint8 uObjectIndex,
                                       const LBS_GenObject_st* pGenObj);
void LBSCalculateObjectQualifiers(uint8 uObjectIndex,
                                  const LBS_GenObject_st* pGenObj,
                                  const LBS_SRRObject_st* pSRRObj);
void LBSCalculateTTC(uint8 uObjectIndex, const LBS_GenObject_st* pGenObj,
                     const LBS_SRRObject_st* pSRRObj,
                     const LBSParam_st* params);
void LBSCalculateRadianCoords(uint8 uObjectIndex,
                              const LBS_GenObject_st* pGenObj,
                              const LBSParam_st* params);
void LBSCalculatePosBasedVxVy(uint8 uObjectIndex,
                              const LBS_GenObject_st* pGenObj,
                              const LBSSystemParam_t* pLBSSystemPar);
void LBSCalculateMaxDimensions(uint8 uObjectIndex,
                               const LBS_GenObject_st* pGenObj);
void LBSCalculateAbsoluteObjectVelocity(uint8 uObjectIndex,
                                        const LBS_GenObject_st* pGenObj,
                                        const EgoVehicleInfo_t* pEgoInfo,
                                        const LBSParam_st* params);
void LBSCalculateRadialObejctRange(uint8 uObjectIndex,
                                   const LBS_GenObject_st* pGenObj);
void LBSCalculateFirstDynProp(uint8 uObjectIndex,
                              const LBS_GenObject_st* pGenObj,
                              const LBS_SRRObject_st* pSRRObj);
void LBSCalculateFirstDetectDist(uint8 uObjectIndex,
                                 const LBS_GenObject_st* pGenObj,
                                 const LBS_SRRObject_st* pSRRObj);
void LBSCalculateDist2Course(uint8 uObjectIndex,
                             const LBS_GenObject_st* pGenObj,
                             const LBS_SRRObject_st* pSRRObj,
                             const EMRoad_t* pRoad);
void LBSCalculateSensorOffset(const LBSParam_st* params);

float32 LBSCalculateTTCRaw(const float32 fX, const float32 fVx,
                           const float32 fAx, boolean bUseAcceleration);
void LBSCalculateMaxSpeedOverGround(const EMGenObjList_st* pGenObjList,
                                    const EgoVehicleInfo_t* pEgoInfo,
                                    const EMRoad_t* pRoad);

/*****************************************************************************
  CALCULATION FUNCTION PROTOTYPES
*****************************************************************************/
LBSCalculate_st* pGetLBSCalculatePointer();
const LBS_GenObject_st* pGetGenObjListPointer_Object(
    uint8 uObj, const EMGenObjList_st* pGenObjList);
const LBS_SRRObject_st* pGetSRRObjListPointer_Object(
    uint8 uObj, const EMSRRObjList_st* pSRRObjList);

LBSObjInfo_st* pGetLBSObjInfoPointer(uint8 uObj);

LBSLCAObjInfo_t* pGetLCAObjInfoPointer(uint8 uObj);
// LCAObjInfo_t* pGetLCAObjInfoPointer              (uint8 uObj);

LBS_Globals_t* pGeLBSCalculatePointer_LBSGlobals();
LBSLCACalculate_t* pGeLBSCalculatePointer_LCAGlobals();
boolean bGetGenObjIsDeleted(uint8 uObj, const EMGenObjList_st* pGenObjList);

float32 LBSCalculateDistToCurve(float32 fXpos, float32 fR);

#endif