#pragma once
#ifndef LBS_LCA_CALCULATE_H
#define LBS_LCA_CALCULATE_H
#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "lbs_lca.h"
/*****************************************************************************
  TYPEDEF
*****************************************************************************/

/*****************************************************************************
  VARIABLES
*****************************************************************************/

/*****************************************************************************
  TOOLS FUNCTION PROTOTYPES
*****************************************************************************/
LCACalculate_st* pGetLCACalculatePointer();
LCARunState_t* pGetLCACalculatePointer_LCARunState();
LCAGlobals_st* pGetLCACalculatePointer_LCAGlobals();
LCAObjInfo_t* pGetLCACalculatePointer_LCAObjInfo(uint8 uObj);
const LCAGenObject_st* pGetLCAGenObject(uint8 uObj,
                                        const LCAGenObjList_st* pGenObjList);
// const LCALBSObjInfo_t* pGetLCAGenObject_LBSObj(uint8 uObj, const
// LCAGenObjList_st* pGenObjList); const LCASIObjInfo_t*
// pGetLCAGenObject_SIObj(uint8 uObj, const LCAGenObjList_st* pGenObjList);
const LCAGenObjInfo_t* pGetLCAGenObject_GenObj(
    uint8 uObj, const LCAGenObjList_st* pGenObjList);
boolean bGetLCAGenObject_GenObjIsDeleted(uint8 uObj,
                                         const LCAGenObjList_st* pGenObjList);
/*****************************************************************************
  CALCULATION FUNCTION PROTOTYPES
*****************************************************************************/
void LCASetGlobals(LCAGlobals_st* pLCAGlobals);
void LCASetParamter(const LCAParam_st* params, LCAGlobals_st* pLCAGlobals);
void LCASetTTCParamter(const LCAParam_st* params,
                       const LCALBSInputInfo_st* pLBSInfo,
                       LCAGlobals_st* pLCAGlobals);
void LCACalculateRangeLimitation(const LCARoad_t* pRoad,
                                 const LCAVehicleInfo_t* pEgoInfo,
                                 LCAGlobals_st* pLCAGlobals);

void LCACheckStableMirringObject(const LCAGenObjList_st* pGenObjList,
                                 const LCALBSObjInfo_Array* pLBSObj,
                                 const LCASIObjInfo_Array* pSIObjList,
                                 const LCALBSInputInfo_st* pLBSInfo,
                                 const LCARoad_t* pRoad,
                                 const LCAVehicleInfo_t* pEgoInfo,
                                 LCACalculate_st* pLCACal);
void LCACalculateAddVxThresh(const LCARoad_t* pRoad,
                             const LCAVehicleInfo_t* pEgoInfo,
                             LCAGlobals_st* pLCAGlobals);
void LCASearchClosestStableObjects(
    const LCAGenObjList_st* pGenObjList, const LCALBSInputInfo_st* pLBSInfo,
    const LCAVehicleInfo_t* pEgoInfo, LCACalculate_st* pLCACal,
    sint32* puObjIdxOwnLane1, sint32* puObjIdxOwnLane2,
    sint32* puObjIdxAdjLane1, sint32* puObjIdxAdjLane2);

void LCASelectClosestStableObjects(const LCAGenObjList_st* pGenObjList,
                                   const LCALBSInputInfo_st* pLBSInfo,
                                   sint32 uObjIdxLane1, sint32 uObjIdxLane2,
                                   sint32* puObjIdxClosestStableObj);
void LCACompareClosestStableObjects(const LCAGenObjList_st* pGenObjList,
                                    const LCALBSObjInfo_Array* pLBSObj,
                                    LCAGlobals_st* pLCAGlobals,
                                    uint8* puObjIdxClosestStableObjOwnLane,
                                    uint8* puObjIdxClosestStableObjAdjLane);
void LCAUpdateClosestStableObjectsInfo(const LCAGenObjList_st* pGenObjList,
                                       const LCAVehicleInfo_t* pEgoInfo,
                                       LCAGlobals_st* pLCAGlobals,
                                       uint8 uObjIdxClosestStableObjOwnLane,
                                       uint8 uObjIdxClosestStableObjAdjLane);

void LCACheckLCAPathBlocked(const LCARoad_t* pRoad,
                            const LCAVehicleInfo_t* pEgoInfo,
                            float32 fVehicleWidth, LCAGlobals_st* LCAGlobals);
void PathBlockedDecision(boolean bZeroAdjLanes, boolean bGRDTooClose,
                         uint8* uCntLCAPathBlocked, boolean* bLCAPathBlocked);

void LCASetObjDependTTCThreshold(boolean bLCAWarningLastCycle,
                                 LCACalculate_st* pLCACalc,
                                 const LCAGenObjInfo_t* pGenObj,
                                 float32* fTTCThreshold);
void LCACheckObjectStartProperties(uint8 uObjIndex,
                                   const LCAGenObjList_st* pGenObjList,
                                   const LCALBSObjInfo_t* pLBSObj,
                                   float32 fTTCThreshold,
                                   boolean* bLowTTCAtStart,
                                   boolean* bCreateAdjStableObj);
boolean LCACheckObjectMirrorStatus(const LCAGenObjInfo_t* pGenObj,
                                   float32 fUpdateRate_nu,
                                   boolean bLCAMirrorObjectLastCycle);

boolean LCACheckObjectFrontMirror(uint8 uObjIndex,
                                  const LCAGenObjList_st* pGenObjList,
                                  const LCALBSObjInfo_t* pLBSObj,
                                  const LCAVehicleInfo_t* pEgoInfo,
                                  LCAObjInfo_t* pLCAObj,
                                  LCACalculate_st* pLCACalc,
                                  float32 fMaxSpeedOverGround_mps);
void LCAVxThreshAddCalc(float32 fUpdateRate, float32 fegoVelocity,
                        float32 fVrelX, LCAFrontMirror_t* pLCAFrontMirror,
                        float32 fMaxSpeedOverGround,
                        float32* fVxThreshAddOwnLane,
                        float32* fVxThreshAddAdjLane);

boolean LCAUpdateMirrorFrontObjectState(uint8* uFrontMirrorCnt,
                                        boolean bPossibleMirrorFrontObject,
                                        boolean bLCAMirrorFrontObjectLastCycle,
                                        uint8* uNofFMObjects);
void LCACheckStableOwnLaneObject(uint8 uObjIndex,
                                 const LCAGenObjList_st* pGenObjList,
                                 LCAFrontMirror_t* pLCAFrontMirror,
                                 boolean* bCheckForStableOwnLaneObj,
                                 boolean* bCheckForStableAdjLaneObj);
void LCAGetXYMirrorSurface(sint32 uObjID, const LCAGenObjList_st* pGenObjList,
                           float32* pfX, float32* pfY);

boolean LCACheckObjectQuality(const LCAGenObjInfo_t* pGenObj,
                              const LCALBSObjInfo_t* pLBSObj,
                              boolean bLowTTCAtStart,
                              boolean bCreateAdjStableObj,
                              boolean bLCAQualityLastCycle,
                              boolean bLCAWarningLastCycle);
boolean LCACheckUpdateStatus(uint8 uiMeasuredTargetFrequency,
                             float32 fUpdateRate);
boolean LCACheckObjectRelevance(const LCAGenObjInfo_t* pGenObj,
                                const LCASIObjInfo_t* pSIObj,
                                float32 fXMovement,
                                boolean bLCARelevantLastCycle,
                                boolean bLCAWarningLastCycle);
float32 LCACalculateObjInLaneTime(const LCAGenObjInfo_t* pGenObj,
                                  const LCASIObjInfo_t* pSIObj);

boolean LCACheckObjectInLCARange(const LCAGenObjInfo_t* pGenObj,
                                 float32 fLCARange);
float32 LCACheckObjectBehindGRD(const LCAGenObjInfo_t* pGenObj,
                                const LCARoad_t* pRoad,
                                const LCAVehicleInfo_t* pEgoInfo,
                                boolean bLCAMirrorFrontObject,
                                float32 fBehindGrdProb);
boolean CheckObjBehindGrd(const LCAGenObjInfo_t* pGenObj, float32 fConfYOffset,
                          float32 fDist2Border, float32 fegoVelocity,
                          boolean bLCAMirrorFrontObject);
boolean LCACheckLaneConditions(const LCAGenObjInfo_t* pGenObj,
                               const LCASIObjInfo_t* pSIObj,
                               boolean bLCAPathBlockedLeft,
                               boolean bLCAPathBlockedRight);
boolean LCACheckObjPath(const LCAGenObjInfo_t* pGenObj, const LCARoad_t* pRoad,
                        boolean bLCAObjPathInvalidLastCycle,
                        float32 fVehicleWidth);
boolean LCACheckWarningConditions(const LCAGenObjInfo_t* pGenObj,
                                  LCAObjInfo_t* pLCAObj,
                                  uint8 uLCAGearPosition);
boolean LCAFinalWarningDecision(const LCAGenObjInfo_t* pGenObj,
                                const LCALBSObjInfo_t* pLBSObj,
                                LCAObjInfo_t* pLCAObj, boolean bInLCARange,
                                boolean bLCAWarningConditions,
                                boolean bLCAWarningLastCycle,
                                float32 fBSDZoneXMin,
                                LCAWarnInfo_t* pLCAWarnInfo);
void LCAStoreWarningObjInfo(uint8 uObjIndex, const LCAGenObjInfo_t* pGenObj,
                            LCAObjInfo_t* pLCAObj,
                            const LCALBSObjInfo_t* pLBSObj,
                            LCAWarnInfo_t* pLCAWarnInfo);

#ifdef __cplusplus
}
#endif
#endif