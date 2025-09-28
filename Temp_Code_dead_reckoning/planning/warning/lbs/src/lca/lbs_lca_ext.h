#pragma once
#ifndef LBS_LCA_EXT_H
#define LBS_LCA_EXT_H
#ifdef __cplusplus
extern "C" {
#endif
/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "lbs_lca.h"

/*****************************************************************************
  VARIABLES
*****************************************************************************/
extern LCAInReq_st LCAReqPorts;
extern LCAOutPro_st LCAProPorts;
extern LCAParam_st LCAParams;
extern LCADebug_t LCADebugInfo;

/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/
void LBS_LCA_Exec(const LCAInReq_st* reqPorts, const LCAParam_st* params,
                  LCAOutPro_st* proPorts, LCADebug_t* debugInfo);
void LBS_LCA_Init_Reset();

#ifdef __cplusplus
}
#endif
#endif