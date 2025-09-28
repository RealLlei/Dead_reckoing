#pragma once
//#include "lbs.h"
#include "bsd/lbs_bsd_ext.h"
#include "planning/warning/lbs/src/lbs_calculation.h"
#include "planning/warning/lbs/src/lbs_external.h"
#include "planning/warning/lbs/src/lca/lbs_lca_ext.h"
#include "planning/warning/lbs/src/ose/lbs_ose_ext.h"
#include "planning/warning/lbs/src/rcw/lbs_rcw_ext.h"
#include "planning/warning/lbs/src/si/lbs_si_ext.h"

/*****************************************************************************
  SI FUNCTION PROTOTYPES
*****************************************************************************/
//#include "lbs_si_ext.h"

extern SICalculate_st SICalculate;

void LBSSIInputWrapper(const LBSInReq_st* extReqPorts,
                       const LBSParam_st* extReqParam, SIInPut_st* input,
                       SIParam_st* paramInput, LBSCalculate_st* pLBSCalc);
//void LBSSIOutputWrapper(SIOutPut_st* pOutput, LBSSIDebug_t* pDebug,
//                       LBSCalculate_st* pLBSCalc, LBSDebug_t* extDebugPorts);
void LBSSIOutputWrapper(LBSCalculate_st* pLBSCalc, LBSDebug_t* extDebugPorts,
                        SIOutPut_st* pOutput, LBSSIDebug_t* pDebug,
                        SICalculate_st* pSICalc);
/*****************************************************************************
  BSD FUNCTION PROTOTYPES
*****************************************************************************/
//#include "lbs_bsd_ext.h"
void BSDInputWrapper(const LBSInReq_st* extReqPorts, BSDInReq_st* input,
                     const LBSParam_st* extReqParam, BSDParam_st* paramInput);
void BSDOutputWrapper(LBSOutPro_t* extProPorts, LBSDebug_t* extDebugPorts,
                      BSDOutPro_st* pOutput, BSDDebug_t* pDebug,
                      LBSCalculate_st* pLBSCalc);
/*****************************************************************************
  LCA FUNCTION PROTOTYPES
*****************************************************************************/
void LBSLCAInputWrapper(const LBSInReq_st* extReqPorts,
                        const LBSParam_st* extReqParam, LCAInReq_st* input,
                        LCAParam_st* paramInput, LBSCalculate_st* pLBSCalc);
void LBSLCAOutputWrapper(LCAOutPro_st* pOutput, LCADebug_t* pDebug,
                         LBSCalculate_st* pLBSCalc, LBSDebug_t* extDebugPorts);
/*****************************************************************************
  OSE FUNCTION PROTOTYPES
*****************************************************************************/
void LBSOSEInputWrapper(const LBSInReq_st* extReqPorts,
                        const LBSParam_st* extReqParam, OSEInReq_t* input,
                        OSEParam_t* paramInput, LBSCalculate_st* pLBSCalc);
void LBSOSEOutputWrapper(OSEOutPro_t* pOutput, OSEDebug_t* OSEDebugInfo,
                         LBSCalculate_st* pLBSCalc, LBSDebug_t* debugInfo);

/*****************************************************************************
  RCW FUNCTION PROTOTYPES
*****************************************************************************/
void RCWInputWrapper(const LBSInReq_st* extReqPorts, RCWInReq_st* input,
                     const LBSParam_st* extReqParam, RCWParam_st* paramInput,
                     LBSCalculate_st* pLBSCalc);

void RCWOutputWrapper(LBSOutPro_t* extProPorts, LBSDebug_t* extDebugPorts,
                      RCWOutPro_st* pOutput, RCWDebug_t* pDebug,
                      LBSCalculate_st* pLBSCalc);