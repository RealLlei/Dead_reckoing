#pragma once
#ifndef LBS_LCA_MAIN_H
#define LBS_LCA_MAIN_H

/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "lbs_lca.h"
#include "lbs_lca_ext.h"

/*****************************************************************************
  VARIABLES
*****************************************************************************/

/*****************************************************************************
  MAIN FUNCTION PROTOTYPES
*****************************************************************************/
void LCA_InitObjects();
void LCA_InitObject(uint8 uObjNumber);
void LCA_InitGlobal();

void LCA_PreProcess(const LCAInReq_st* reqPorts, const LCAParam_st* params,
                    LCAOutPro_st* proPorts, LCADebug_t* debugInfo);
void LCA_MainProcess(const LCAInReq_st* reqPorts, const LCAParam_st* params,
                     LCAOutPro_st* proPorts, LCADebug_t* debugInfo);
void LCA_PostProcess(LCAOutPro_st* proPorts, LCADebug_t* debugInfo);
#endif