
#pragma once
#ifndef LBS_SI_EXT_H
#define LBS_SI_EXT_H
/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "lbs_si.h"

//#define LBS_SI_DEVELOPMENT_STATE_OK TRUE
/*****************************************************************************
  VARIABLES
*****************************************************************************/

extern SIInPut_st SIReqPorts;
extern SIOutPut_st SIProPorts;
extern SIParam_st SIParams;
extern SIDebug_st SIDebugInfo;

/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/

void LBS_SIExec(const SIInPut_st* reqPorts, const SIParam_st* params,
                SIOutPut_st* proPorts, SIDebug_st* debugInfo);
void LBS_SI_Init_Reset();

#endif