#pragma once
#ifndef LBS_BSD_MAIN_H
#define LBS_BSD_MAIN_H
#ifdef __cplusplus
extern "C" {
#endif
/*****************************************************************************
  INCLUDES
*****************************************************************************/

#include "planning/warning/lbs/common/tue_common_libs.h"
#include "planning/warning/lbs/src/bsd/lbs_bsd.h"
#include "planning/warning/lbs/src/bsd/lbs_bsd_ext.h"

/*****************************************************************************
  MAIN FUNCTION PROTOTYPES
*****************************************************************************/
void BSD_InitObject();
void BSD_InitOneObject(uint8 uObjNumber);
void BSD_InitGlobal();

void BSDGetBSDZoneParameters(const BSD_LBSGlobalInfo_t* pLBSGlobalInput,
                             const BSDVehicleInfo_t* pEgoInfo,
                             const BSDRoad_t* pRoad, const BSDParam_st* params);
void BSDCycleGlobalUpdate(const BSDSensorMounting_t* pSensorMounting);
void BSDCalculateVxThreshMovStatClassification(
    const BSDVehicleInfo_t* pEgoInfo);
void BSDInitObjCalArray(const BSDGenObjList_st* pGenObjList);

void BSD_PreProecss(const BSDInReq_st* reqPorts, const BSDParam_st* params,
                    BSDOutPro_st* proPorts, BSDDebug_t* debugInfo);
void BSD_MainProcess(const BSDInReq_st* reqPorts, const BSDParam_st* params,
                     BSDOutPro_st* proPorts, BSDDebug_t* debugInfo);
void BSD_PostProcess(const BSDInReq_st* reqPorts, const BSDParam_st* params,
                     BSDOutPro_st* proPorts, BSDDebug_t* debugInfo);
#ifdef __cplusplus
}
#endif
#endif