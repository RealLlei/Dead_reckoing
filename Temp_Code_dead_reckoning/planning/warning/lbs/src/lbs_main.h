#pragma once
#ifndef LBS_MAIN_H
#define LBS_MAIN_H
#ifdef __cplusplus
extern "C" {
#endif

#include "planning/warning/lbs/src/lbs_external.h"

#include "planning/warning/lbs/src/lbs_calculation.h"

#define HZ_PERCEPTION TRUE
#define ST_PERCEPTION FALSE

#define LBS_DEVELOPMENT_DEBUG FALSE
#if LBS_DEVELOPMENT_DEBUG
void LBSInputProcess(LBSInReq_st* reqPorts, LBSParam_st* params);
#endif
extern SICalculate_st SICalculate;

void LBSProcess(const LBSInReq_st* reqPorts, const LBSParam_st* params,
                LBSOutPro_t* proPorts, LBSDebug_t* debugInfo);
void LBSProcessLCAWarnings(const LBSInReq_st* reqPorts,
                           const LBSParam_st* params, LBSOutPro_t* proPorts,
                           LBSDebug_t* debugInfo, LBSCalculate_st* pLBSCalc);
boolean LBSSetFunctionOutput(boolean bFunctionActive,
                             boolean bEgoSpeedCondition);
void LBSProcessBSDWarnings(const LBSInReq_st* reqPorts,
                           const LBSParam_st* params, LBSOutPro_t* proPorts,
                           LBSDebug_t* debugInfo, LBSCalculate_st* pLBSCalc);
void LBSProcessOSEWarnings(const LBSInReq_st* reqPorts,
                           const LBSParam_st* params, LBSOutPro_t* proPorts,
                           LBSDebug_t* debugInfo, LBSCalculate_st* pLBSCalc);

boolean LBSOSEWarnCalcNextState(uint8 uWarnLevel, boolean bWarnThisCycle,
                                const float32 fCycletime,
                                const LBSParam_st* params,
                                LBSCalculate_st* pLBSCalc);
void LBSPostProcess(const LBSInReq_st* reqPorts, const LBSParam_st* params,
                    LBSOutPro_t* proPorts, LBSDebug_t* debugInfo,
                    LBSCalculate_st* pLBSCalc);
boolean LBSCheckEgoSpeedActThreshold(boolean bLastWarningState,
                                     float32 fEgoSpeed, float32 fDisableThresh,
                                     float32 fEnableThresh,
                                     boolean* bEgoSpeedConLastCycle);
void LBSInitObjInfo(uint8 uObjectIndex);
void LBS_Exec(const LBSInReq_st* reqPorts, const LBSParam_st* params,
              LBSOutPro_t* proPorts, LBSDebug_t* debugInfo);
void LBS_Reset();
#ifdef __cplusplus
}
#endif
#endif