#pragma once
#ifndef LBS_RCW_CALCULATE_H
#define LBS_RCW_CALCULATE_H
#ifdef __cplusplus
extern "C" {
#endif
/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "lbs_rcw_ext.h"

#include "planning/warning/lbs/common/tue_common_libs.h"
/*****************************************************************************
  TYPEDEF
*****************************************************************************/

/*****************************************************************************
  VARIABLES
*****************************************************************************/
extern RCWCalculate_st RCWCalculate;

/*****************************************************************************
  CALCULATION FUNCTION PROTOTYPES
*****************************************************************************/

/*****************************************************************************
  CALCULATION FUNCTION PROTOTYPES
*****************************************************************************/
RCWCalculate_st* pGetRCWCalculatePointer();
RCW_Info_Array* pGetRCWCalculatePointer_RCWAllObjInfo();
RCW_Info_t* pGetRCWCalculatePointer_RCWObjInfoList(uint8 uObj);
RCWCorridorObserver_t* pGetRCWCalculatePointer_RCWCorridorObjs(uint8 uidx);
RCWWarningInfo_t* pGetRCWCalculatePointer_RCWWarningInfo();
RCWStationaryBlocked_t* pGetRCWCalculatePointer_RCWStationaryBlocker();
RCWWarningInfo_t* pGetRCWCalculatePointer_RCWWarningInfo_t();
RCWStatusCondition_t* pGetRCWCalculatePointer_RCWStatusCondition();
RCWStateMachine_t* pGetRCWCalculatePointer_RCWstatemachine();

boolean RCWbGetObjIsDeleted(uint8 uObj, const RCWGenObjList_st* pGenObjList);
float32 pGetObjLongDisplacement(uint8 uObj,
                                const RCWGenObjList_st* pGenObjList);
float32 pGetObjLatDisplacement(uint8 uObj, const RCWGenObjList_st* pGenObjList);
float32 pGetObjLongVrel(uint8 uObj, const RCWGenObjList_st* pGenObjList);
float32 pGetObjDist2Course(uint8 uObj, const RCWGenObjList_st* pGenObjList);
// boolean RCWbGetObjRightSensorFlag(uint8 uObj, const RCW_GenObject_st*
// pGenObjInfo);

void LBSRCWCustomCalculateTTCThreshold(const RCWParam_st* RCWparams,
                                       const RCWGenObjList_st* pGenObj,
                                       uint8 uidx, RCW_Info_t* pRCWObjInfo);
void LBSRCWCheckObjectQuality(const RCWGenObjList_st* pGenObj, uint8 uidx,
                              RCW_Info_t* pRCWObjInfo,
                              const RCW_LBSInputInfo_st* pGetLBSInputInfo);
// void    LBSRCWCheckMultipathObject(const RCWGenObjList_st* pGenObj, uint8
// uidx, RCW_Info_Array* pRCWAllObjInfo, RCW_Info_t* pRCWObjInfo);
void LBSRCWCheckUpdateStatus(const RCWGenObjList_st* pGenObj, uint8 uidx,
                             RCW_Info_t* pRCWObjInfo,
                             const RCW_LBSInputInfo_st* pGetLBSInputInfo);
void LBSRCWCalculateHeadingFilter(const RCWGenObjList_st* pGenObj, uint8 uidx,
                                  RCW_Info_t* pRCWObjInfo,
                                  const RCW_LBSInputInfo_st* pGetLBSInputInfo);
void LBSRCWCheckHeadingInRange(const RCWGenObjList_st* pGenObj, uint8 uidx,
                               RCW_Info_t* pRCWObjInfo,
                               const RCWParam_st* RCWparams);
void LBSRCWProcessCorridor(const RCWGenObjList_st* pGenObj, uint8 uidx,
                           RCW_Info_t* pRCWObjInfo,
                           const RCW_LBSInputInfo_st* pGetLBSInputInfo,
                           const RCWParam_st* RCWparams,
                           const RCWVehicleInfo_t* pGetEgoInfo);
void LBSRCWCalculateCorridor(const RCWGenObjList_st* pGenObj, uint8 uidx,
                             LBSRCWCorridor_t* pRCWCorridor,
                             const RCWParam_st* RCWparams);
void LBSRCWCorridorAssociation(const RCWGenObjList_st* pGenObj, uint8 uidx,
                               LBSRCWCorridor_t* pRCWCorridor,
                               const RCWParam_st* RCWparams,
                               const RCWVehicleInfo_t* pGetEgoInfo,
                               RCW_Info_t* pRCWObjInfo,
                               const RCW_LBSInputInfo_st* pGetLBSInputInfo);
void LBSRCWCalculateInCorridorOverlap(const RCWGenObjList_st* pGenObj,
                                      uint8 uobj, const float32 fObjRefDist,
                                      const float32 fTraceBracketLeft,
                                      const float32 fTraceBracketRight,
                                      const float32 fMinObjWidth,
                                      const float32 fMaxObjWidth,
                                      RCWSITrajOccupancy_t* pOccupancy);
void LBSRCWCheckObjectRelevence(const RCWGenObjList_st* pGenObj, uint8 uidx,
                                RCW_Info_t* pRCWObjInfo,
                                const RCWParam_st* RCWparams,
                                const RCW_LBSInputInfo_st* pGetLBSInputInfo);
void LBSRCWCheckBlockedCorridor(const RCWGenObjList_st* pGenObj, uint8 uidx,
                                RCW_Info_t* pRCWObjInfo,
                                const RCWParam_st* RCWparams,
                                const RCW_LBSInputInfo_st* pGetLBSInputInfo);
void LBSRCWChooseOccupancyThreshold(const RCWGenObjList_st* pGenObj, uint8 uidx,
                                    RCW_Info_t* pRCWObjInfo,
                                    const RCWParam_st* RCWparams);
void LBSRCWCheckWarningConditions(const RCWGenObjList_st* pGenObj, uint8 uidx,
                                  RCW_Info_t* pRCWObjInfo,
                                  const RCW_LBSInputInfo_st* pGetLBSInputInfo);
void LBSRCWFinalWarningDecision(const RCWGenObjList_st* pGenObj, uint8 uidx,
                                RCW_Info_t* pRCWObjInfo,
                                const RCW_LBSInputInfo_st* pGetLBSInputInfo,
                                const RCWParam_st* RCWparams,
                                RCWWarningInfo_t* pRCWCalcWarningInfo);
void LBSRCWStoreWarningObjInfor(const RCWGenObjList_st* pGenObj, uint8 uidx,
                                RCW_Info_t* pRCWObjInfo,
                                const RCW_LBSInputInfo_st* pGetLBSInputInfo,
                                RCWWarningInfo_t* pRCWCalcWarningInfo);
void LBSRCWCorrIsStationaryBlocked(
    const RCWVehicleInfo_t* pGetEgoInfo, const RCWGenObjList_st* pGenObj,
    const RCWParam_st* RCWparams,
    RCWStationaryBlocked_t* pRCWStationaryBlocker);
#ifdef ST_PERCEPTION

void LBSRCWStateConditionProcess(
    RCWStatusCondition_t* pGetRCWStateConditions,
    const RCWVehicleInfo_t* pGetEgoInfo,
    const RCWPreProcessInput_t* pGetPreProcessInput, const RCWParam_st* params,
    RCWWarningInfo_t* pRCWCalcWarningInfo,
    const RCWStationaryBlocked_t* pRCWStationaryBlocker, RCWDebug_t* debugInfo);

#endif  // ST_PERCEPTION

void LBSRCWStateConditionProcess(
    RCWStatusCondition_t* pGetRCWStateConditions,
    const RCWVehicleInfo_t* pGetEgoInfo,
    const RCWPreProcessInput_t* pGetPreProcessInput, const RCWParam_st* params,
    RCWWarningInfo_t* pRCWCalcWarningInfo,
    const RCWStationaryBlocked_t* pRCWStationaryBlocker, RCWDebug_t* debugInfo);

void LBSRCWStateMachineProcess(
    RCWStateMachine_t* pGetRCWstatemachine,
    const RCWStatusCondition_t* pGetRCWStateConditions, RCWDebug_t* debugInfo);
void LBSRCWHmiOutput(const RCWStateMachine_t* pGetRCWstatemachine,
                     RCWOutPro_st* proPorts,
                     const RCWWarningInfo_t* pRCWCalcWarningInfo,
                     const RCWParam_st* params);
void LBSRCWBlockingTimeProcess(const RCWStateMachine_t* pGetRCWstatemachine,
                               const RCWParam_st* params);

#ifdef __cplusplus
}
#endif
#endif
