#pragma once
#ifndef LBS_BSD_EXT_H
#define LBS_BSD_EXT_H
/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "planning/warning/lbs/src/bsd/lbs_bsd.h"

/*****************************************************************************
  VARIABLES
*****************************************************************************/
extern BSDInReq_st BsdReqPorts;
extern BSDOutPro_st BsdProPorts;
extern BSDParam_st BsdParams;
extern BSDDebug_t BsdDebugInfo;

/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/
void LBS_BSDExec(const BSDInReq_st* reqPorts, const BSDParam_st* params,
                 BSDOutPro_st* proPorts, BSDDebug_t* debugInfo);
void LBS_BSD_Init_Reset();

#endif