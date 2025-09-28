#include "planning/warning/cta/cta_extern.h"
#include "planning/warning/lbs/src/bsd/lbs_bsd_ext.h"
#include "planning/warning/lbs/src/lbs_calculation.h"
#include "planning/warning/lbs/src/lbs_external.h"
#include "planning/warning/lbs/src/ose/lbs_ose_ext.h"
#include "planning/warning/lbs/src/rcw/lbs_rcw_ext.h"
#include "planning/warning/lbs/src/si/lbs_si_ext.h"

LBSInReq_st reqPorts;
LBSParam_st params;
LBSOutPro_t proPorts;
LBSDebug_t debugInfo;

// CTA interface
CTAInReq_t CTA_reqPorts;
CTAParam_t CTA_params;
CTAOutPro_t CTA_proPorts;
CTADebug_t CTA_debugInfo;

SIInPut_st SIReqPorts;
SIOutPut_st SIProPorts;
SIParam_st SIParams;
SIDebug_st SIDebugInfo;

// input port
RCWInReq_st RCWReqPorts;
RCWParam_st RCWParams;

// Output port
RCWOutPro_st RCWProPorts;
RCWDebug_t RCWDebugInfo;

OSEInReq_t OSEReqPorts;
OSEParam_t OSEParams;
OSEOutPro_t OSEProPorts;
OSEDebug_t OSEDebugInfo;

LCAInReq_st LCAReqPorts;
LCAOutPro_st LCAProPorts;
LCAParam_st LCAParams;
LCADebug_t LCADebugInfo;

BSDInReq_st BsdReqPorts;
BSDParam_st BsdParams;
BSDOutPro_st BsdProPorts;
BSDDebug_t BsdDebugInfo;

LBSCalculate_st LBSCalculate;
RCWCalculate_st RCWCalculate;
SICalculate_st SICalculate;

CTA_Switch CTA_function_switch;
LBS_Switch LBS_function_switch;

FCTAInReq_t FCTAreqPorts;
RCTAInReq_t RCTAreqPorts;
FCTAObjCycle_t obs_move_cycle_arr;