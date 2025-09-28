/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "lbs_main.h"

#include "bsd/lbs_bsd_ext.h"
#include "lbs_external.h"
#include "lbs_wrapper.h"
#include "lca/lbs_lca_ext.h"
#include "ose/lbs_ose_ext.h"
#include "rcw/lbs_rcw_ext.h"
#include "si/lbs_si_ext.h"

#ifdef UBUNTU_SYSTEM
#include "BSW_DataLog_Server.h"
#endif

/*****************************************************************************
  VARIABLES
*****************************************************************************/
LBSState_t LBSState = LBS_INIT;

#if LBS_DEVELOPMENT_DEBUG
/*****************************************************************************
  Functionname: LBSInputProcess                                  */ /*!

  @brief:

  @description:

  @param[in]:reqPorts

  @return:void
*****************************************************************************/
void LBSInputProcess(LBSInReq_st* reqPorts, LBSParam_st* params) {
  uint16 i;
  uint8* pObjDynProp;
  uint8 uObjDynProp;
  LBS_GenObject_st* pGenObj = NULL;

  for (i = 0; i < LBS_INPUT_OBJECT_NUMBER; i++) {
    pGenObj = &(reqPorts->GenObjList.aObject[i]);
    pObjDynProp =
        &(reqPorts->GenObjList.aObject[i].Attributes.eDynamicProperty_nu);

    /*****************************************************************************
     *Updata Object width and length by object classify                   */ /*!
		****************************************************************************/

    switch (pGenObj->Attributes.eClassification_nu) {
      case LBS_EM_GEN_OBJECT_CLASS_CAR:
        pGenObj->Geometry.fWidth_met = LBS_EM_GEN_CLASS_CAR_DEFAULT_WIDTH;
        pGenObj->Geometry.fLength_met = LBS_EM_GEN_CLASS_CAR_DEFAULT_LENGTH;

        pGenObj->Geometry.fWidthLeft_met =
            LBS_EM_GEN_CLASS_CAR_DEFAULT_WIDTH * 0.5F;
        pGenObj->Geometry.fWidthRight_met =
            LBS_EM_GEN_CLASS_CAR_DEFAULT_WIDTH * 0.5F;
        pGenObj->Geometry.fLengthFront_met =
            LBS_EM_GEN_CLASS_CAR_DEFAULT_LENGTH * 0.5F;
        pGenObj->Geometry.fLengthRear_met =
            LBS_EM_GEN_CLASS_CAR_DEFAULT_LENGTH * 0.5F;

        break;
      case LBS_EM_GEN_OBJECT_CLASS_TRUCK:
        pGenObj->Geometry.fWidth_met = TUE_CML_MinMax(
            pGenObj->Geometry.fWidth_met, LBS_EM_GEN_CLASS_TRUCK_MIN_WIDTH,
            LBS_EM_GEN_CLASS_TRUCK_MAX_WIDTH);
        pGenObj->Geometry.fLength_met = TUE_CML_MinMax(
            pGenObj->Geometry.fLength_met, LBS_EM_GEN_CLASS_TRUCK_MIN_LENGTH,
            LBS_EM_GEN_CLASS_TRUCK_MIN_LENGTH);

        pGenObj->Geometry.fWidthLeft_met = pGenObj->Geometry.fWidth_met * 0.5F;
        pGenObj->Geometry.fWidthRight_met = pGenObj->Geometry.fWidth_met * 0.5F;

        pGenObj->Geometry.fLengthFront_met =
            pGenObj->Geometry.fLength_met * 0.5F;
        pGenObj->Geometry.fLengthRear_met =
            pGenObj->Geometry.fLength_met * 0.5F;

        break;
      default:
        // nothing to do
        break;
    }
  }
}
#endif

/*****************************************************************************
  Functionname: LBS_Exec                                  */ /*!

  @brief:LBS Main Exec function

  @description:the LBS main logical function

  @param[in]:reqPorts,params,proPorts,debugInfo

  @return:void
*****************************************************************************/
void LBS_Exec(const LBSInReq_st* reqPorts, const LBSParam_st* params,
              LBSOutPro_t* proPorts, LBSDebug_t* debugInfo) {
  LBSCalculate_st* pLBSCalc = &LBSCalculate;

  if (LBSState == LBS_OK) {
#if LBS_DEVELOPMENT_DEBUG

    LBSInputProcess(reqPorts, params);

    /**************************************************
     * delete the function , implemented in t interface
     ***************************************************/
#endif
    // SI MAIN FUNCTION
    /**********************************************/
    /* Run the SI function module                */
    /**********************************************/
    // get SI input from LBS layer
    LBSSIInputWrapper(reqPorts, params, &SIReqPorts, &SIParams, pLBSCalc);
    // SI main function
    LBS_SIExec(&SIReqPorts, &SIParams, &SIProPorts, &SIDebugInfo);
    // set LCA output to LBS layer
    //LBSSIOutputWrapper(&SIProPorts, &debugInfo->SIDebug, pLBSCalc, debugInfo);
    LBSSIOutputWrapper(pLBSCalc, debugInfo, &SIProPorts,
                       (LBSSIDebug_t*)&SIDebugInfo, &SICalculate);

    // LBS process...
    LBSProcess(reqPorts, params, proPorts, debugInfo);
    // main process...
    // BSD MAIN FUNCTION outer wrapper
    // get input from extern data
    BSDInputWrapper(reqPorts, &BsdReqPorts, params, &BsdParams);
    // BSD MAIN FUNCTION
    LBS_BSDExec(&BsdReqPorts, &BsdParams, &BsdProPorts, &BsdDebugInfo);

    // set output to extern data
    BSDOutputWrapper(proPorts, debugInfo, &BsdProPorts, &BsdDebugInfo,
                     pLBSCalc);

    /**********************************************/
    /* Run the LCA function module                */
    /**********************************************/
    // get LCA input from LBS layer
    LBSLCAInputWrapper(reqPorts, params, &LCAReqPorts, &LCAParams, pLBSCalc);
    // LCA main function
    LBS_LCA_Exec(&LCAReqPorts, &LCAParams, &LCAProPorts, &LCADebugInfo);

    // set LCA output to LBS layer
    LBSLCAOutputWrapper(&LCAProPorts, &LCADebugInfo, pLBSCalc, debugInfo);

    /**********************************************/
    /* Run the OSE function module                */
    /**********************************************/
    // get OSE input from LBS layer
    LBSOSEInputWrapper(reqPorts, params, &OSEReqPorts, &OSEParams, pLBSCalc);
    // OSE main function
    LBS_OSE_Exec(&OSEReqPorts, &OSEParams, &OSEProPorts, &OSEDebugInfo);
    // set OSE output to LBS layer
    LBSOSEOutputWrapper(&OSEProPorts, &OSEDebugInfo, pLBSCalc, debugInfo);

    /**********************************************/
    /* Run the RCW function module                */
    /**********************************************/
    // get input from extern data
    RCWInputWrapper(reqPorts, &RCWReqPorts, params, &RCWParams, pLBSCalc);
    // RCW MAIN FUNCTION
    LBS_RCWExec(&RCWReqPorts, &RCWParams, &RCWProPorts, &RCWDebugInfo);
    // set output to extern data
    RCWOutputWrapper(proPorts, debugInfo, &RCWProPorts, &RCWDebugInfo,
                     pLBSCalc);

    LBSPostProcess(reqPorts, params, proPorts, debugInfo, pLBSCalc);
    proPorts->LBSFunState.fTTC_s = RCWProPorts.rcw_fTTC_s;

  } else {
    LBS_Reset();
    LBSState = LBS_OK;
  }
  // DEBUG_Print("LBS PROCESS END\n");

#ifdef UBUNTU_SYSTEM
  // datalogger logic
  DATALOGInfo_t Record1;
  static LCADatalog_st LCADatalog;
  memcpy(&LCADatalog.LCAReqPorts, &LCAReqPorts, sizeof(LCAInReq_st));
  memcpy(&LCADatalog.LCAParams, &LCAParams, sizeof(LCAParam_st));
  memcpy(&LCADatalog.LCAProPorts, &LCAProPorts, sizeof(LCAOutPro_st));
  memcpy(&LCADatalog.LCADebugInfo, &LCADebugInfo, sizeof(LCADebug_t));

  Record1.StructID = Data_LCADatalog_t_type;
  Record1.Length = sizeof(LCADatalog_st);
  Record1.SocBuf = (uint8*)&LCADatalog;

  BSW_DataLog_FreezeData(Record1);
#endif
}

/*****************************************************************************
  Functionname: LBSProcess                                  */ /*!

  @brief:The LBS General property calculate function

  @description:The LBS General property calculate function

  @param[in]:reqPorts,params,proPorts,debugInfo

  @return:void
*****************************************************************************/
void LBSProcess(const LBSInReq_st* reqPorts, const LBSParam_st* params,
                LBSOutPro_t* proPorts, LBSDebug_t* debugInfo) {
  // LBS Init...
  const EMGenObjList_st* pGenObjList = &reqPorts->GenObjList;

  // Clear object LBS calculate array if the object id changed this cycle
  LBSInitObjCalArray(pGenObjList);

  // Calculated global LBS properties
  LBSCalculateGlobalProperties(reqPorts, params, proPorts, debugInfo);

  // Calculates object properties used in LBS
  LBSCalculateObjectProperties(reqPorts, params, proPorts, debugInfo);
}

/*****************************************************************************
  Functionname: LBSInitObjCalArray                                  */ /*!

  @brief: Clear LBS calculate array

  @description: Clear LBS calculate array if the object id changed
				and save the new object for next cycle to judge if the object id change

  @param[in]:pGenObjList: the LBS object list input pointer

  @return:void
*****************************************************************************/
void LBSInitObjCalArray(const EMGenObjList_st* pGenObjList) {
  uint8 uObj = 0;
  const LBS_GenObject_st* pGenObjInfo = NULL;
  LBSCalculate_st* pLBSCalculate = pGetLBSCalculatePointer();

  for (uObj = 0U; uObj < LBS_INPUT_OBJECT_NUMBER; uObj++) {
    pGenObjInfo = pGetGenObjListPointer_Object(uObj, pGenObjList);

    if (pLBSCalculate->LastObjIDList[uObj] != pGenObjInfo->General.uiID_nu) {
      // object id have been change,init the calculate array
      LBSInitObjInfo(uObj);
      // save current object id
      pLBSCalculate->LastObjIDList[uObj] = pGenObjInfo->General.uiID_nu;
    }
  }
}

/*****************************************************************************
  Functionname: LBSPostProcess                                  */ /*!

  @brief:the LBS post output function

  @description:output the LBS result to proPorts

  @param[in]:reqPorts,params,proPorts,debugInfo

  @return:void
*****************************************************************************/
void LBSPostProcess(const LBSInReq_st* reqPorts, const LBSParam_st* params,
                    LBSOutPro_t* proPorts, LBSDebug_t* debugInfo,
                    LBSCalculate_st* pLBSCalc) {
  //const LBSSystemParam_t* pLBSParam = &reqPorts->LBSSystemParam;

  LBSProcessBSDWarnings(reqPorts, params, proPorts, debugInfo, pLBSCalc);
  LBSProcessLCAWarnings(reqPorts, params, proPorts, debugInfo, pLBSCalc);
  LBSProcessOSEWarnings(reqPorts, params, proPorts, debugInfo, pLBSCalc);
}

/*****************************************************************************
  Functionname: LBSProcessBSDWarnings                                  */ /*!

  @brief:check if the BSD warning in enable conditions

  @description:check if the BSD warning in enable conditions,ego speed or switch

  @param[in]:reqPorts,params,proPorts,debugInfo

  @return:void
*****************************************************************************/
void LBSProcessBSDWarnings(const LBSInReq_st* reqPorts,
                           const LBSParam_st* params, LBSOutPro_t* proPorts,
                           LBSDebug_t* debugInfo, LBSCalculate_st* pLBSCalc) {
  const LBSSystemParam_t* pLBSParam = &reqPorts->LBSSystemParam;
  const LBSBSDWarningParameter_t* pBSDParam =
      &params->LBS_Ks_BSDParameter_nu.LBS_Ks_BSDWarnParameter_nu;
  const LBSBSDCalculate_t* pBSDCal = &pLBSCalc->LBSBSDCalc;
  //const float32 fEgoSpeedX = reqPorts->EgoVehInfo.fegoVelocity_mps;
  // LBSFunState_t* pFunState = &proPorts->LBSFunState;
  LBS_Globals_t* pLBSGlobal = &proPorts->LBSGlobals;

  boolean bBSDFunctionOutputActive = FALSE;
  boolean bBSDEgoSpeedCondition_left = FALSE;
  boolean bBSDEgoSpeedCondition_right = FALSE;
  boolean bBSDFunctionOutput_left = FALSE;
  boolean bBSDFunctionOutput_right = FALSE;

  if ((pLBSParam->bBSDFunctionActive == TRUE) &&
      (pLBSParam->bBSDFunctionOutputActive == TRUE)) {
    // set output active if function and function output both active
    bBSDFunctionOutputActive = TRUE;
  }

  /*****************************************************************************/
  /* BSD */
  /*****************************************************************************/

  bBSDEgoSpeedCondition_left = LBSCheckEgoSpeedActThreshold(
      pLBSCalc->LBSWarnLastCycle.bBSDWarningLeftLastCycle,
      reqPorts->EgoVehInfo.fegoVelocity_mps,
      pBSDParam->LBS_Kf_BSDVelMinWarnDisable_mps,
      pBSDParam->LBS_Kf_BSDVelMinWarnEnable_mps,
      &pLBSCalc->LBS_Globals.LastCycleStates.bEgoSpeedConditionBSD);
  bBSDEgoSpeedCondition_right = LBSCheckEgoSpeedActThreshold(
      pLBSCalc->LBSWarnLastCycle.bBSDWarningRightLastCycle,
      reqPorts->EgoVehInfo.fegoVelocity_mps,
      pBSDParam->LBS_Kf_BSDVelMinWarnDisable_mps,
      pBSDParam->LBS_Kf_BSDVelMinWarnEnable_mps,
      &pLBSCalc->LBS_Globals.LastCycleStates.bEgoSpeedConditionBSD);

  bBSDFunctionOutput_left = LBSSetFunctionOutput(bBSDFunctionOutputActive,
                                                 bBSDEgoSpeedCondition_left);
  bBSDFunctionOutput_right = LBSSetFunctionOutput(bBSDFunctionOutputActive,
                                                  bBSDEgoSpeedCondition_right);

  pLBSGlobal->bBSDFunctionOutput =
      bBSDFunctionOutput_left || bBSDFunctionOutput_right;
  // LBSSetFunctionOutput(bBSDFunctionOutputActive, bEgoSpeedCondition);

  pLBSCalc->LBSWarnLastCycle.bBSDWarningLastCycle =
      (pLBSCalc->LBSBSDCalc.bBSDWarnActiveLeft ||
       pLBSCalc->LBSBSDCalc.bBSDWarnActiveRight);

  pLBSCalc->LBSWarnLastCycle.bBSDWarningLeftLastCycle =
      pLBSCalc->LBSBSDCalc.bBSDWarnActiveLeft;
  pLBSCalc->LBSWarnLastCycle.bBSDWarningRightLastCycle =
      pLBSCalc->LBSBSDCalc.bBSDWarnActiveRight;
  // Check warning flag if function active and output
  if (bBSDFunctionOutput_left && pLBSCalc->LBSBSDCalc.bBSDWarnActiveLeft) {
    proPorts->LBSFunState.bBSDWarning = TRUE;
    proPorts->LBSFunState.bBSDWarningLeft = TRUE;
  } else if (bBSDFunctionOutput_right &&
             pLBSCalc->LBSBSDCalc.bBSDWarnActiveRight) {
    proPorts->LBSFunState.bBSDWarning = TRUE;
    proPorts->LBSFunState.bBSDWarningRight = TRUE;
  } else {
    proPorts->LBSFunState.bBSDWarning = FALSE;
    proPorts->LBSFunState.bBSDWarningLeft = FALSE;
    proPorts->LBSFunState.bBSDWarningRight = FALSE;
  }
}

/*****************************************************************************
  Functionname: LBSProcessLCAWarnings                                  */ /*!

  @brief:check if the LCA warning in enable conditions

  @description:check if the LCA warning in enable conditions,ego speed or switch

  @param[in]:reqPorts,params,proPorts,debugInfo

  @return:void
*****************************************************************************/
void LBSProcessLCAWarnings(const LBSInReq_st* reqPorts,
                           const LBSParam_st* params, LBSOutPro_t* proPorts,
                           LBSDebug_t* debugInfo, LBSCalculate_st* pLBSCalc) {

  boolean bLCAEgoSpeedCondition_left = FALSE;
  boolean bLCAEgoSpeedCondition_right = FALSE;
  boolean bLCAFunctionOutput_left = FALSE;
  boolean bLCAFunctionOutput_right = FALSE;
  boolean bLCAFunctionOutputActive = FALSE;
  sint32 uLCAWaningID = pLBSCalc->LBSLCACalc.uLCAWarningID_nu;

  if (reqPorts->LBSSystemParam.bLCAFunctionActive &&
      reqPorts->LBSSystemParam.bLCAFunctionOutputActive) {
    // set output active if function and function output both active
    bLCAFunctionOutputActive = TRUE;
  }
  // Check if the ego speed in the range of function activation
  bLCAEgoSpeedCondition_left = LBSCheckEgoSpeedActThreshold(
      pLBSCalc->LBSWarnLastCycle.bLCAWarningLeftLastCycle,
      reqPorts->EgoVehInfo.fegoVelocity_mps,
      params->LBS_Ks_LCAParameter_nu.LBS_Kf_LCAVownMinWarnDisable_mps,
      params->LBS_Ks_LCAParameter_nu.LBS_Kf_LCAVoWnMinWarnEnable_mps,  //20/3.6
      &pLBSCalc->LBS_Globals.LastCycleStates.bEgoSpeedConditionLCA);
  bLCAEgoSpeedCondition_right = LBSCheckEgoSpeedActThreshold(
      pLBSCalc->LBSWarnLastCycle.bBSDWarningRightLastCycle,
      reqPorts->EgoVehInfo.fegoVelocity_mps,
      params->LBS_Ks_LCAParameter_nu.LBS_Kf_LCAVownMinWarnDisable_mps,
      params->LBS_Ks_LCAParameter_nu.LBS_Kf_LCAVoWnMinWarnEnable_mps,  //20/3.6
      &pLBSCalc->LBS_Globals.LastCycleStates.bEgoSpeedConditionLCA);

  // Check if all conditions are met to activate function output
  bLCAFunctionOutput_left = LBSSetFunctionOutput(bLCAFunctionOutputActive,
                                                 bLCAEgoSpeedCondition_left);

  bLCAFunctionOutput_right = LBSSetFunctionOutput(bLCAFunctionOutputActive,
                                                  bLCAEgoSpeedCondition_right);

  pLBSCalc->LBS_Globals.bLCAFunctionOutput =
      bLCAFunctionOutput_left || bLCAFunctionOutput_right;

  pLBSCalc->LBSWarnLastCycle.bLCAWarningLastCycle =
      pLBSCalc->LBSLCACalc.bLCAWarnActive;
  pLBSCalc->LBSWarnLastCycle.bLCAWarningLeftLastCycle =
      pLBSCalc->LBSLCACalc.bLCAWarnActiveLeft;
  pLBSCalc->LBSWarnLastCycle.bLCAWarningRightLastCycle =
      pLBSCalc->LBSLCACalc.bLCAWarnActiveRight;

  // Check warning flag if function active and output

  if (pLBSCalc->LBS_Globals.bLCAFunctionOutput &&
      pLBSCalc->LBSLCACalc.bLCAWarnActive) {
    proPorts->LBSFunState.bLCAWarning = TRUE;
    proPorts->LBSFunState.fTTC_s = pLBSCalc->LBSLCACalc.fCriticalTTC_s;
    proPorts->LBSFunState.bLCAWarningLeft =
        pLBSCalc->LBSLCACalc.bLCAWarnActiveLeft;
    proPorts->LBSFunState.bLCAWarningRight =
        pLBSCalc->LBSLCACalc.bLCAWarnActiveRight;
  } else {

    proPorts->LBSFunState.bLCAWarning = FALSE;
    proPorts->LBSFunState.bLCAWarningLeft = FALSE;
    proPorts->LBSFunState.bLCAWarningRight = FALSE;
  }
  // debug
  // debugInfo->LCADebug.uLCAWarningID_nu =
  // pLBSCalc->LBSLCACalc.uLCAWarningID_nu;
}

/*****************************************************************************
  Functionname: LBSProcessOSEWarnings                                  */ /*!

  @brief: check if the OSE warning in enable conditions

  @description: check if the OSE warning in enable conditions,ego speed or switch

  @param[in]: reqPorts,params,proPorts,debugInfo

  @return:void
*****************************************************************************/
void LBSProcessOSEWarnings(const LBSInReq_st* reqPorts,
                           const LBSParam_st* params, LBSOutPro_t* proPorts,
                           LBSDebug_t* debugInfo, LBSCalculate_st* pLBSCalc) {
  if (FALSE == reqPorts->LBSSystemParam.bOSEFunctionActive) {
    proPorts->LBSFunState.bOSEWarning.bPreWarnActive = FALSE;
    proPorts->LBSFunState.bOSEWarning.bAcuteWarnActive = FALSE;
    proPorts->LBSFunState.bOSEWarning.bDoorLockingActive = FALSE;
    proPorts->LBSFunState.bOSEWarningLeft.bPreWarnActive = FALSE;
    proPorts->LBSFunState.bOSEWarningLeft.bAcuteWarnActive = FALSE;
    proPorts->LBSFunState.bOSEWarningLeft.bDoorLockingActive = FALSE;
    proPorts->LBSFunState.bOSEWarningRight.bPreWarnActive = FALSE;
    proPorts->LBSFunState.bOSEWarningRight.bAcuteWarnActive = FALSE;
    proPorts->LBSFunState.bOSEWarningRight.bDoorLockingActive = FALSE;
    pLBSCalc->LBSOSECalc.bOSEWarnActive[0] = FALSE;
    pLBSCalc->LBSOSECalc.bOSEWarnActive[1] = FALSE;
    pLBSCalc->LBSOSECalc.bOSEWarnActive[2] = FALSE;
    proPorts->LBSFunState.fTTC_s = TUE_C_F32_VALUE_INVALID;
    return;
  }
  boolean bFunctionEnabled = FALSE;
  boolean bOSEFunctionOutputActive = FALSE;
  boolean bWarnThisCycle[OSE_LBS_NUM_OF_WARN_LEVELS] = {0};
  sint32 uOSEWaningID = pLBSCalc->LBSOSECalc.uCriticalObjID;
  boolean bEgoLeftDoorOpen = reqPorts->EgoVehInfo.bLeftDoorOpen;
  boolean bEgoRightDoorOpen = reqPorts->EgoVehInfo.bRightDoorOpen;

  proPorts->LBSFunState.bOSEWarningLeft.bPreWarnActive = FALSE;
  proPorts->LBSFunState.bOSEWarningLeft.bAcuteWarnActive = FALSE;
  proPorts->LBSFunState.bOSEWarningLeft.bDoorLockingActive = FALSE;
  proPorts->LBSFunState.bOSEWarningRight.bPreWarnActive = FALSE;
  proPorts->LBSFunState.bOSEWarningRight.bAcuteWarnActive = FALSE;
  proPorts->LBSFunState.bOSEWarningRight.bDoorLockingActive = FALSE;

  // minimum activation speed condition is common for all warnings
  if (reqPorts->EgoVehInfo.fegoVelocity_mps <
          params->LBS_Ks_OSEParameter_nu.LBS_Kf_OSEVEgoMax_mps &&
      reqPorts->EgoVehInfo.fegoVelocity_mps >
          params->LBS_Ks_OSEParameter_nu.LBS_Kf_OSEVEgoMin_mps) {
    bFunctionEnabled = TRUE;
  } else {
    bFunctionEnabled = FALSE;
  }

  if (reqPorts->LBSSystemParam.bOSEFunctionActive &&
      reqPorts->LBSSystemParam.bOSEFunctionOutputActive) {
    // set output active if function and function output both active
    bOSEFunctionOutputActive = TRUE;
  }
  // Checks OSE activation via BSW parameters(driver), gear shift position and
  // ego speed condition
  bFunctionEnabled =
      LBSSetFunctionOutput(bOSEFunctionOutputActive, bFunctionEnabled);
  pLBSCalc->LBS_Globals.LastCycleStates.bEgoSpeedConditionOSE =
      bFunctionEnabled;
  for (uint8 uWarnLevel = 0U; uWarnLevel < 2; uWarnLevel++) {
    pLBSCalc->LBS_Globals.bOSEFunctionOutput[uWarnLevel] = bFunctionEnabled;
    bWarnThisCycle[uWarnLevel] =
        pLBSCalc->LBS_Globals.bOSEFunctionOutput[uWarnLevel] &&
        pLBSCalc->LBSOSECalc.bOSEWarnActive[uWarnLevel];

    if (uWarnLevel < 1U && pLBSCalc->LBSOSECalc.bWarningInterrupt) {
      // First level of warning can be interrupted
      bWarnThisCycle[uWarnLevel] = FALSE;
    }  // else: rest of levels can't be interrupted or first level is not
       // interrupted
    bWarnThisCycle[uWarnLevel] = LBSOSEWarnCalcNextState(
        uWarnLevel, bWarnThisCycle[uWarnLevel],
        reqPorts->LBSSystemParam.fCycletime_s, params, pLBSCalc);

    pLBSCalc->LBSWarnLastCycle.bOSEWarningLastCycle[uWarnLevel] =
        pLBSCalc->LBSOSECalc.bOSEWarnActive[uWarnLevel];
  }

  // for OSE_l3_warning
  bWarnThisCycle[2] = (bWarnThisCycle[0] || bWarnThisCycle[1]);
  pLBSCalc->LBSWarnLastCycle.bOSEWarningLastCycle[2] = bWarnThisCycle[2];

  proPorts->LBSFunState.bOSEWarning.bPreWarnActive = bWarnThisCycle[0];
  proPorts->LBSFunState.bOSEWarning.bAcuteWarnActive = bWarnThisCycle[1];
  proPorts->LBSFunState.bOSEWarning.bDoorLockingActive = bWarnThisCycle[2];

  if (uOSEWaningID != TUE_C_UI8_VALUE_INVALID) {
    if (reqPorts->GenObjList.aObject[uOSEWaningID].bRightSensor) {
      pLBSCalc->LBSOSECalc.bCriticalObsIsRight = TRUE;
      proPorts->LBSFunState.bOSEWarningRight.bPreWarnActive = bWarnThisCycle[0];
      proPorts->LBSFunState.bOSEWarningRight.bAcuteWarnActive =
          bWarnThisCycle[1];
      proPorts->LBSFunState.bOSEWarningRight.bDoorLockingActive =
          (bWarnThisCycle[2] && bEgoRightDoorOpen);

      proPorts->LBSFunState.bOSEWarningLeft.bPreWarnActive = FALSE;
      proPorts->LBSFunState.bOSEWarningLeft.bAcuteWarnActive = FALSE;
      proPorts->LBSFunState.bOSEWarningLeft.bDoorLockingActive = FALSE;

    } else {
      pLBSCalc->LBSOSECalc.bCriticalObsIsRight = FALSE;
      proPorts->LBSFunState.bOSEWarningLeft.bPreWarnActive = bWarnThisCycle[0];
      proPorts->LBSFunState.bOSEWarningLeft.bAcuteWarnActive =
          bWarnThisCycle[1];
      proPorts->LBSFunState.bOSEWarningLeft.bDoorLockingActive =
          (bWarnThisCycle[2] && bEgoLeftDoorOpen);

      proPorts->LBSFunState.bOSEWarningRight.bPreWarnActive = FALSE;
      proPorts->LBSFunState.bOSEWarningRight.bAcuteWarnActive = FALSE;
      proPorts->LBSFunState.bOSEWarningRight.bDoorLockingActive = FALSE;
    }
  } else {

    proPorts->LBSFunState.bOSEWarningLeft.bPreWarnActive = FALSE;
    proPorts->LBSFunState.bOSEWarningLeft.bAcuteWarnActive = FALSE;
    proPorts->LBSFunState.bOSEWarningLeft.bDoorLockingActive = FALSE;
    proPorts->LBSFunState.bOSEWarningRight.bPreWarnActive = FALSE;
    proPorts->LBSFunState.bOSEWarningRight.bAcuteWarnActive = FALSE;
    proPorts->LBSFunState.bOSEWarningRight.bDoorLockingActive = FALSE;
  }

  if (proPorts->LBSFunState.bOSEWarning.bPreWarnActive) {
    proPorts->LBSFunState.fTTC_s = pLBSCalc->LBSOSECalc.fCriticalTTC;
  } else {
    proPorts->LBSFunState.fTTC_s = TUE_C_F32_VALUE_INVALID;
  }
}

/*****************************************************************************
  Functionname: LBSOSEWarnCalcNextState                                  */ /*!

  @brief: Process the warning state for the given warning

  @description: Process the warning state for the given warning

  @param[in]: uWarnLevel     The warning 

  @return: retValue          Flag whether OSE warning is output by the state machine
*****************************************************************************/
boolean LBSOSEWarnCalcNextState(uint8 uWarnLevel, boolean bWarnThisCycle,
                                const float32 fCycletime,
                                const LBSParam_st* params,
                                LBSCalculate_st* pLBSCalc) {
  boolean retValue = FALSE;

  switch (pLBSCalc->LBSOSECalc.OSEWarnState[uWarnLevel]) {
    case OSE_STATE_PASSIVE: {
      if (bWarnThisCycle) {
        // currently in passive state -> transition to active state
        pLBSCalc->LBSOSECalc.OSEWarnState[uWarnLevel] = OSE_STATE_ACTIVE;
      }  // else remain in passive state
      // in passive state always reset timer and set output to false
      pLBSCalc->LBSOSECalc.WarningTimer[uWarnLevel] = 0.0F;
      retValue = FALSE;
    } break;
    case OSE_STATE_ACTIVE: {
      if (bWarnThisCycle) {
        // currently in active state -> check maximum timer

        retValue = TRUE;
        if (pLBSCalc->LBSOSECalc.WarningTimer[uWarnLevel] <
            params->LBS_Ks_OSEParameter_nu.LBS_Ka_OSEMinTime_s[uWarnLevel]) {
          // minimum time has not yet elapsed -> set output to true
          retValue = TRUE;
        }
      }
    } break;
    case OSE_STATE_LOCKED: {
      if (pLBSCalc->LBSOSECalc.WarningTimer[uWarnLevel] >
          OSE_LBS_COMMON_WARN_LOCK_TIME)  // 5
      {
        // Lock time expired -> allowed to go inti passive state and reset time
        pLBSCalc->LBSOSECalc.OSEWarnState[uWarnLevel] = OSE_STATE_PASSIVE;
        pLBSCalc->LBSOSECalc.WarningTimer[uWarnLevel] = 0.0F;
      }  // else keep locked state
      // in locked state always set output to false
      retValue = FALSE;
    } break;
    default: {
      // always set output to false and reset timer
      retValue = FALSE;
      pLBSCalc->LBSOSECalc.WarningTimer[uWarnLevel] = 0.0F;
    } break;
  }
  // incream timer for current state
  pLBSCalc->LBSOSECalc.WarningTimer[uWarnLevel] += fCycletime;
  return retValue;
}

/*****************************************************************************
  Functionname: LBSCheckEgoSpeedActThreshold                                  */ /*!

  @brief:Check if the ego speed in the range of function activation

  @description:Check if the ego speed in the range of function activation

  @param[in]:fEgoSpeed,fDisableThresh,fEnableThresh,bEgoSpeedConLastCycle

  @return:void
*****************************************************************************/
boolean LBSCheckEgoSpeedActThreshold(boolean bLastWarningState,
                                     float32 fEgoSpeed, float32 fDisableThresh,
                                     float32 fEnableThresh,
                                     boolean* bEgoSpeedConLastCycle) {
  boolean bEgoSpeedCondition = FALSE;
  if ((fEgoSpeed > fEnableThresh) ||
      ((fEgoSpeed > fDisableThresh) && (*bEgoSpeedConLastCycle == TRUE) &&
       bLastWarningState)) {
    bEgoSpeedCondition = TRUE;
  }

  // Update speed condition last cycle
  *bEgoSpeedConLastCycle = bEgoSpeedCondition;

  return bEgoSpeedCondition;
}

/*****************************************************************************
  Functionname: LBSSetFunctionOutput                                  */ /*!

  @brief: Check if all parameters and conditions are met to activate function output

  @description: Check if all parameters and conditions are met to activate function output

  @param[in]:
			  bFunctionActive               Flag whether function has been activated by algorithm
			  bEgoSpeedCondition            Flag whether ego speed is sufficient to activate the function

  @return:    bFunctionOutput               Flag whether all conditions are met
*****************************************************************************/
boolean LBSSetFunctionOutput(boolean bFunctionActive,
                             boolean bEgoSpeedCondition) {
  boolean bFunctionOutput = FALSE;

  // All conditions have to be fulfilled to set the function output
  if ((bFunctionActive == TRUE) && (bEgoSpeedCondition == TRUE)) {
    // function switch enable and in the enable speed range
    bFunctionOutput = TRUE;
  } else {
    bFunctionOutput = FALSE;
  }
  return bFunctionOutput;
}

/*****************************************************************************
  Functionname: LBS_Reset                                  */ /*!

  @brief: LBS function init reset

  @description:the function first exec reset process

  @param[in]:void

  @return:void
*****************************************************************************/
void LBS_Reset() {
  // Reset LBS Global
  //LBSState = LBS_INIT;
  // init LBSGlobal struct
  uint8 uObjectIndex = 0;
  LBSCalculate_st* pLBSCalculate = pGetLBSCalculatePointer();

  // init LBS Global parameter
  memset(pLBSCalculate, 0U, sizeof(LBSCalculate_st));

  // clear all LBS related information
  for (uObjectIndex = 0U; uObjectIndex < LBS_INPUT_OBJECT_NUMBER;
       uObjectIndex++) {
    LBSInitObjInfo(uObjectIndex);
  }

  // Init last cycle id to invalid
  for (uObjectIndex = 0U; uObjectIndex < LBS_INPUT_OBJECT_NUMBER;
       uObjectIndex++) {
    pLBSCalculate->LastObjIDList[uObjectIndex] = TUE_C_UI16_VALUE_INVALID;
    pLBSCalculate->LBSObjHistoryList[uObjectIndex].fFirstDetectX_met =
        TUE_C_F32_VALUE_INVALID;
    pLBSCalculate->LBSObjHistoryList[uObjectIndex].fFirstDetectY_met =
        TUE_C_F32_VALUE_INVALID;
    pLBSCalculate->LBSObjHistoryList[uObjectIndex].fMaxRange_met =
        TUE_C_F32_VALUE_INVALID;
  }

  // Set object selection algo parameters
  LBSFCTObjSelSetParameter();
}

/*****************************************************************************
  Functionname: LBSInitObjInfo                                  */ /*!

  @brief: LBS function for init calculate object array

  @description:the function first exec reset process for object

  @param[in]:uObjectIndex,the object array index

  @return:void
*****************************************************************************/
void LBSInitObjInfo(uint8 uObjectIndex) {
  LBSObjInfo_st* pLBSObj = pGetLBSObjInfoPointer(uObjectIndex);

  // clear all LBS general object related information
  pLBSObj->fTTCAccel_mps2 = TUE_C_F32_VALUE_INVALID;
  pLBSObj->fTTC_s = TUE_C_F32_VALUE_INVALID;
  pLBSObj->fTTCFiltered_s = TUE_C_F32_VALUE_INVALID;
  pLBSObj->fTTCRadial_s = TUE_C_F32_VALUE_INVALID;
  pLBSObj->fCycletimeSum_s = 0.0F;
  pLBSObj->fUpdateRate_nu = 0.74999F;     // 0.5 -> 0.74999  --- JO
  pLBSObj->fAssocProbFiltered = 0.9999F;  // 0.5 -> 0.999999999  --- JO
  pLBSObj->fAngle_deg = 0.0F;
  pLBSObj->fObjLengthMax = 0.0F;
  pLBSObj->fObjWidthMax = 0.0F;
  pLBSObj->fRangeRadial = TUE_C_F32_VALUE_INVALID;
  pLBSObj->fVabs_mpss = 0.0F;
  pLBSObj->ObjBorders.fXmin_met = 0.0F;
  pLBSObj->ObjBorders.fXmax_met = 0.0F;
  pLBSObj->ObjBorders.fYmin_met = 0.0F;
  pLBSObj->ObjBorders.fYmax_met = 0.0F;
  pLBSObj->fXLastCycle_met = TUE_C_F32_VALUE_INVALID;
  pLBSObj->fYLastCycle_met = TUE_C_F32_VALUE_INVALID;
  pLBSObj->fVxPosBased = TUE_C_F32_VALUE_INVALID;
  pLBSObj->fVyPosBased = TUE_C_F32_VALUE_INVALID;
  pLBSObj->ObjMovementBorders.fXmin_met = TUE_C_F32_VALUE_INVALID;
  pLBSObj->ObjMovementBorders.fXmax_met = TUE_C_F32_VALUE_INVALID;
  pLBSObj->ObjMovementBorders.fYmin_met = TUE_C_F32_VALUE_INVALID;
  pLBSObj->ObjMovementBorders.fYmax_met = TUE_C_F32_VALUE_INVALID;
  pLBSObj->fXMovement_met = 0.0F;
  pLBSObj->fYMovement_met = 0.0F;
  pLBSObj->uUniqueID = TUE_C_UI16_VALUE_INVALID;
  pLBSObj->uLastMergedObjID = TUE_C_UI8_VALUE_INVALID;
  pLBSObj->bLowTTCAtStart = FALSE;
  pLBSObj->bCreateAdjStableObj = FALSE;
  pLBSObj->fSpeedFiltered_mpss = 0.0F;
  pLBSObj->ObjSel.bBreakthroughHit = FALSE;
  pLBSObj->ObjSel.bObjectFastEnough = FALSE;
  pLBSObj->ObjSel.bObjInRange = FALSE;
  pLBSObj->firstStableDynProp = EM_GEN_OBJECT_DYN_PROPERTY_UNKNOWN;
}
