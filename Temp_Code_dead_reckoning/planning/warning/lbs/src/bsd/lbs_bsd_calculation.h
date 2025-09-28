#pragma once
#ifndef LBS_BSD_CALCULATE_H
#define LBS_BSD_CALCULATE_H
#ifdef __cplusplus
extern "C" {
#endif
/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "lbs_bsd.h"
#include "planning/warning/lbs/common/tue_common_libs.h"

/*****************************************************************************
  TYPEDEF
*****************************************************************************/

/*****************************************************************************
  VARIABLES
*****************************************************************************/
BSDCalculate_st BSDCalculate;

/*****************************************************************************
  CALCULATION FUNCTION PROTOTYPES
*****************************************************************************/
// void GetBSDObjPointer                       (uint8 uObj);
boolean bGetObjIsDeleted(uint8 uObj, const BSDGenObjList_st* pGenObjList);
boolean bGetObjRightSensorFlag(uint8 uObj, const BSD_GenObject_st* pGenObjInfo);

void BSDUpdateBSDWarningActiveFlag(uint8 uObj, const BSD_GenObject_st* pGenObj);
void BSDUpdateBSDZoneWithHyst(uint8 uObj, const BSD_GenObject_st* pGenObj,
                              BSD_Globals_t* pBSDGlobal);
void BSDUpdateGlobalZoneParameters(uint8 uObj, const BSD_GenObject_st* pGenObj);
float32 BSDCalculateAdaptedBSDZoneLength(
    const BSD_LBSGlobalInfo_t* pLBSGlobalInput,
    const BSDVehicleInfo_t* pEgoInfo, const BSDRoad_t* pRoad,
    const BSDSensorMounting_t* pSenserMounting, float32 fBSDZoneXmin,
    float32 fCenter2Axis);
void BSDGetZoneParametersForCurrentObject(uint8 uObj,
                                          const BSD_GenObject_st* pGenObj,
                                          const BSDRoad_t* pRoad,
                                          const BsdZone_t* pBsdZonePar);
void BSDCalculateSectorCuts(uint8 uObj, const BSD_GenObject_st* pGenObj,
                            const BSD_LBSGlobalInfo_t* pLBSGlobalInput,
                            const BSDVehicleInfo_t* pEgoInfo,
                            const BSDRoad_t* pRoad,
                            const BSDVehParameter_t* pBSDVehParameter);
uint8 BSDClassifyAppreance(uint8 uObj, const BSD_GenObject_st* pGenObj,
                           const BSD_SRRObject_st* pSRRObj,
                           const BSD_LBSInputInfo_st* pLBSInputInfo,
                           const BSDVehParameter_t* pVehParameter);
boolean BSDCheckObjectInBSDZone(uint8 uObj,
                                const BSD_LBSInputInfo_st* pLBSInputInfo,
                                const BSDVehicleInfo_t* pEgoInfo,
                                const BSDRoad_t* pRoad);
boolean BSDCheckObjectInSOTZone(uint8 uObj, const BSD_GenObject_st* pGenObj,
                                const BSD_LBSInputInfo_st* pLBSInputInfo,
                                const BSDRoad_t* pRoad);
boolean BSDCheckObjectZoneOverlap(uint8 uObj, const BSD_GenObject_st* pGenObj,
                                  const BSD_LBSInputInfo_st* pLBSInputInfo,
                                  const BSDRoad_t* pRoad);
void BSDCalculateUpdateGrdCounter(uint8 uObj, const BSD_GenObject_st* pGenObj,
                                  const BSD_SRRObject_st* pSRRObj,
                                  const BSD_LBSInputInfo_st* pLBSInputInfo,
                                  const BSDVehicleInfo_t* pEgoInfo,
                                  const BSDRoad_t* pRoad,
                                  const BSDSensorMounting_t* pSensorMounting);
boolean BSDCheckObjectBehindGRD(uint8 uObj, const BSD_GenObject_st* pGenObj,
                                const BSD_LBSInputInfo_st* pLBSInputInfo);
boolean BSDCheckObjectBehindGRD_hz(uint8 uObj, const BSD_GenObject_st* pGenObj,
                                   const BSD_LBSInputInfo_st* pLBSInputInfo);
void BSDCalculateUpdateOwnlaneCounter(uint8 uObj,
                                      const BSD_GenObject_st* pGenObj,
                                      const BSD_SRRObject_st* pSRRObj,
                                      const BSD_LBSInputInfo_st* pLBSInputInfo,
                                      const BSDRoad_t* pRoad);
boolean BSDCheckObjectOnOwnlane(uint8 uObj, const BSD_GenObject_st* pGenObj);
boolean BSDCheckObjectQuality(uint8 uObj, const BSD_GenObject_st* pGenObj,
                              const BSD_SRRObject_st* pSRRObj,
                              const BSD_LBSInputInfo_st* pLBSInputInfo,
                              const BSDVehicleInfo_t* pEgoInfo);
boolean BSDCheckObjectLivedLongEnough(uint8 uObj,
                                      const BSD_LBSInputInfo_st* pLBSInputInfo);
boolean BSDCheckObjectLivedLongEnough_hz(
    uint8 uObj, const BSD_LBSInputInfo_st* pLBSInputInfo);
void BSDCheckObjectUpdatedRecently(uint8 uObj, const BSD_SRRObject_st* pSRRObj);
void BSDClassifyObject(uint8 uObj, const BSD_GenObject_st* pGenObj,
                       const BSD_SRRObject_st* pSRRObj,
                       const BSD_LBSInputInfo_st* pLBSInputInfo,
                       const BSDVehicleInfo_t* pEgoInfo, const BSDRoad_t* pRoad,
                       const BSDVehParameter_t* pVehPar);
void BSDClassifyGRD(uint8 uObj, const BSD_GenObject_st* pGenObj,
                    const BSD_SRRObject_st* pSRRObj,
                    const BSD_LBSInputInfo_st* pLBSInputInfo,
                    const BSDVehicleInfo_t* pEgoInfo, const BSDRoad_t* pRoad);
boolean BSDCheckObjectIsRelevant(uint8 uObj, const BSD_GenObject_st* pGenObj,
                                 const BSDVehParameter_t* pVehPar);
boolean BSDCheckObjectSoT(uint8 uObj, const BSD_GenObject_st* pGenObj,
                          const BSD_LBSInputInfo_st* pLBSInputInfo,
                          const BSDWarningParameter_t* pBSDWarnParameter,
                          const BSDVehParameter_t* pVehPar);
boolean BSDCheckObjectFastSoT(uint8 uObj, const BSD_GenObject_st* pGenObj,
                              const BSDWarningParameter_t* pBSDWarnParameter,
                              const BSDSystemParam_t* pBSDSystemParam);
boolean BSDCheckObjectSoTDelay(uint8 uObj,
                               const BSDWarningParameter_t* pBSDWarnParameter);
void BSDCalculateUpdateObjectSoTDelay(uint8 uObj,
                                      const BSD_GenObject_st* pGenObj,
                                      const BSD_SRRObject_st* pSRRObj,
                                      const BSD_LBSInputInfo_st* pLBSInputInfo,
                                      const BSDSystemParam_t* pBSDSystemParam,
                                      const BSDVehParameter_t* pVehPar);
void BSDUpdateScenarioObserver(uint8 uObj, const BSD_GenObject_st* pGenObjt);
boolean BSDCheckObjectPlausibility(uint8 uObj, const BSD_GenObject_st* pGenObj,
                                   const BSD_SRRObject_st* pSRRObj,
                                   const BSD_LBSInputInfo_st* pLBSInputInfo,
                                   const BSDVehicleInfo_t* pEgoInfo,
                                   const BSDVehParameter_t* pVehPar);
boolean BSDCheckObjectShortWarning(
    uint8 uObj, const BSD_GenObject_st* pGenObj,
    const BSD_LBSInputInfo_st* pLBSInputInfo, const BSDRoad_t* pRoad,
    const BSDWarningParameter_t* pBSDWarnParameter);
boolean BSDCheckObjectWarningConditions(
    uint8 uObj, const BSD_SRRObject_st* pSRRObj,
    const BSD_LBSInputInfo_st* pLBSInputInfo, uint8 uBSDGearPosition);
float32 BSDCalculateObjectRelDist(uint8 uObj, const BSD_GenObject_st* pGenObj,
                                  const BSD_SRRObject_st* pSRRObj,
                                  const BSD_LBSInputInfo_st* pLBSInputInfo);
void BSDSetGlobalWarning(uint8 uObj, BSD_Info_t* pBSDObjInfo,
                         float32 fObjDistX);

float32 BSDCalculateDistToDrivenCurve(float32 fXpos, float32 fR);
void BSDCalculateUpdateAndLimitCounter(uint8* uCounter, sint8 iInc, uint8 uMin,
                                       uint8 uMax);

void BSDCalculateObjectGrdrailRelation(uint8 uObj,
                                       const BSD_GenObject_st* pGenObj,
                                       const BSD_SRRObject_st* pSRRObj,
                                       float32 fGrdDist2Course,
                                       boolean bGrdDistFound,
                                       boolean* pbBehindGrd, boolean* pbGrdHit);

boolean BSDCalculateObjectInRectZone(uint8 uObj, float32 fZoneXMin,
                                     float32 fZoneXMax, float32 fZoneYMin,
                                     float32 fZoneYMax,
                                     const BSD_LBSInputInfo_st* pLBSInputInfo);
boolean BSDCalculateObjectInCurvedZone(uint8 uObj, float32 fZoneXMin,
                                       float32 fZoneXMax, float32 fZoneYMin,
                                       float32 fZoneYMax,
                                       const BSD_LBSInputInfo_st* pLBSInputInfo,
                                       const BSDRoad_t* pRoad);
void BSDCalculateGrdDist2Course(uint8 uObj, const BSD_GenObject_st* pGenObj,
                                const BSD_SRRObject_st* pSRRObj,
                                const BSDVehicleInfo_t* pEgoInfo,
                                const BSDRoad_t* pRoad,
                                const BSDSensorMounting_t* pSensorMounting,
                                float32* pfGrdDist2Course,
                                boolean* pbGrdDistFound);
boolean BSDCalculateCheckCornersInCurvedZone(
    uint8 uObj, float32 fZoneXMin, float32 fZoneXMax, float32 fZoneYMin,
    float32 fZoneYMax, float32 fCurveRadius,
    const BSD_LBSInputInfo_st* pLBSInputInfo);
boolean BSDCalculateCheckPointInCurvedZone(float32 fObjBorderX,
                                           float32 fObjBorderY,
                                           float32 fZoneXMin, float32 fZoneXMax,
                                           float32 fZoneYMin, float32 fZoneYMax,
                                           float32 fDist2Course);
boolean BSDCalculateCheckGrdPreCondition(uint8 Obj,
                                         const BSD_GenObject_st* pGenObj,
                                         const BSD_SRRObject_st* pSRRObj,
                                         const BSDRoad_t* pRoad);

void BSDClassifyFrontObject(uint8 uObj, const BSD_GenObject_st* pGenObj,
                            const BSD_SRRObject_st* pSRRObj,
                            const BSD_LBSInputInfo_st* pLBSInputInfo,
                            const BSDVehicleInfo_t* pEgoInfo,
                            const BSDRoad_t* pRoad,
                            const BSDVehParameter_t* pVehPar);
void BSDClassifyRearObject(uint8 uObj);
void BSDClassifySideObject(uint8 uObj, const BSD_GenObject_st* pGenObj,
                           const BSD_LBSInputInfo_st* pLBSInputInfo,
                           const BSDVehicleInfo_t* pEgoInfo,
                           const BSDRoad_t* pRoad,
                           const BSDVehParameter_t* pVehPar);
void BSDClassifyConfirmedObjectAtRear(uint8 uObj,
                                      const BSD_GenObject_st* pGenObj,
                                      const BSD_SRRObject_st* pSRRObj,
                                      const BSD_LBSInputInfo_st* pLBSInputInfo,
                                      const BSDVehicleInfo_t* pEgoInfo,
                                      const BSDVehParameter_t* pVehPar);
float32 BSDCalculateTimeToExit(uint8 uObj, const BSD_GenObject_st* pGenObj,
                               const BSD_LBSInputInfo_st* pLBSInputInfo,
                               const BSDWarningParameter_t* pBSDWarnParameter,
                               const BSDRoad_t* pRoad);

// float32 SafeDiv                                      (float32 fDivisor);
float32 BSDCalculate_TTBreakthroughLine(float32 fZoneBoundary,
                                        float32 fObjectBoundary,
                                        float32 fObjectSpeed);
float32 BSDCalculate_TTBreakthroughCurve(
    uint8 uObj, float32 fObjXmin, float32 fObjXmax, float32 fObjY,
    float32 fZoneY, float32 fObjectSpeed,
    const BSDWarningParameter_t* pBSDWarnParameter, const BSDRoad_t* pRoad);

/*****************************************************************************
  CALCULATION FUNCTION PROTOTYPES
*****************************************************************************/
void InitBSDCalculate();
BSDCalculate_st* pGetBSDCalculatePointer();
BSD_Globals_t* pGetBSDCalculatePointer_BSDGlobals();
BSDGlobal_BSDZone_st* pGetBSDCalculatePointer_BSDGlobaZonePar();
BSDZone_ObjPar* pGetBSDCalculatePointer_BSDZoneObjPar(uint8 uObj);
BSD_Info_t* pGetBSDCalculatePointer_ObjInfo(uint8 uObj);

const BSD_LBSObjInfo_st* pGetBSD_LBSObjInfoPointer_LBSObjInfo(
    uint8 uObj, const BSD_LBSInputInfo_st* pLBSInputInfo);
const BSD_LCAObjInfo_t* pGetBSD_LBSObjInfoPointer_LCAObjInfo(
    uint8 uObj, const BSD_LBSInputInfo_st* pLBSInputInfo);
const BSD_GenObject_st* pGetBSDGenObjListPointer_Object(
    uint8 uObj, const BSDGenObjList_st* pGenObjList);
const BSD_SRRObject_st* pGetBSDSRRObjListPointer_Object(
    uint8 uObj, const BSDSRRObjList_st* pSRRObjList);

#ifdef __cplusplus
}
#endif
#endif
