#pragma once
#ifndef LBS_RCW_MAIN_H
#define LBS_RCW_MAIN_H
#ifdef __cplusplus
extern "C" {
#endif
/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "lbs_rcw_ext.h"
#include "planning/warning/lbs/common/tue_common_libs.h"
/*****************************************************************************
  MAIN FUNCTION PROTOTYPES
*****************************************************************************/
void RCW_InitObject();
void RCW_InitOneObject(uint8 uObjNumber);
void RCW_InitGlobal();

void RCW_PreProecss(const RCWInReq_st* reqPorts, const RCWParam_st* params);
void RCW_MainProcess(const RCWInReq_st* reqPorts, const RCWParam_st* params,
                     RCWOutPro_st* proPorts, RCWDebug_t* debugInfo);
void RCW_PostProcess(const RCWInReq_st* reqPorts, const RCWParam_st* params,
                     RCWOutPro_st* proPorts, RCWDebug_t* debugInfo);

void LBSRCWCyclicGlobalUpdate();
void LBSRCWBlockedCorridorAnalysis(const RCWInReq_st* reqPorts,
                                   const RCWParam_st* params);

#ifdef __cplusplus
}
#endif
#endif