#pragma once
#ifndef LBS_SI_MAIN_H
#define LBS_SI_MAIN_H
#ifdef __cplusplus
extern "C" {
#endif  //
/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "lbs_si.h"
#include "lbs_si_ext.h"

/*****************************************************************************
  MAIN FUNCTION PROTOTYPES
*****************************************************************************/
void SIUpdateGlobal(const SIInPut_st* reqPorts, const SIParam_st* params);
void SICalculateBaseLaneWidths(const SIInPut_st* reqPorts,
                               const SIParam_st* params);
void SICalculateLaneCorridor(const SIInPut_st* reqPorts,
                             const SIParam_st* params);
void SI_LaneAssociation(const SIInPut_st* reqPorts, const SIParam_st* params,
                        SIOutPut_st* proPorts, SIDebug_st* debugInfo);
#ifdef __cplusplus
}
#endif  // __cplusplus
#endif