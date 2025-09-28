/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "lbs_rcw_main.h"
#include "lbs_rcw_calculation.h"

/*****************************************************************************
  VARIABLES
*****************************************************************************/

/*****************************************************************************
  FUNCTION
*****************************************************************************/

/*****************************************************************************
  Functionname: RCWExec                                  */ /*!

  @brief: RCW Exec main function 

  @description: RCW main function

  @param[in]:RCW input,params

  @return:void
*****************************************************************************/
void LBS_RCWExec(const RCWInReq_st* reqPorts, const RCWParam_st* params,
                 RCWOutPro_st* proPorts, RCWDebug_t* debugInfo) {
  boolean bRCWFunctionActive = reqPorts->RCWSystemSwitch.bRCWFunctionActive;
  boolean bRCWFunctionActiveLast =
      pGetRCWCalculatePointer()->RCWRunState.bRCWFunctionActionLastCycle;
  RCWState_t eRCWState = RCWCalculate.RCWRunState.eRCWState;
  // printf("eRCWState = %d\t", eRCWState);

  switch (eRCWState) {
    case RCW_OK:
      /*Function switch enable and function has been init*/
      if (bRCWFunctionActive == TRUE) {
        RCW_PreProecss(reqPorts, params);
        RCW_MainProcess(reqPorts, params, proPorts, debugInfo);
      } else {
        /*Function exit reset*/
        if (bRCWFunctionActiveLast == TRUE) {
          LBS_RCW_Init_Reset();
          pGetRCWCalculatePointer()->RCWRunState.eRCWState = RCW_OK;
        }
      }
      break;
    case RCW_INIT:
      /*First enter reset*/
      LBS_RCW_Init_Reset();
      pGetRCWCalculatePointer()->RCWRunState.eRCWState = RCW_OK;
      break;
    default:
      /*First enter reset*/
      LBS_RCW_Init_Reset();
      pGetRCWCalculatePointer()->RCWRunState.eRCWState = RCW_OK;
      break;
  }
  RCW_PostProcess(reqPorts, params, proPorts, debugInfo);
  pGetRCWCalculatePointer()->RCWRunState.bRCWFunctionActionLastCycle =
      bRCWFunctionActive;
}

/*****************************************************************************
  Functionname: LBS_RCW_Init_Reset                                  */ /*!

  @brief: RCW function init reset

  @description:the function first exec reset process

  @param[in]:void

  @return: void
*****************************************************************************/
void LBS_RCW_Init_Reset() {
  // Init internal object data
  RCW_InitObject();
  // Init internal global data
  RCW_InitGlobal();
  // Init RCW run flag
  pGetRCWCalculatePointer()->RCWRunState.eRCWState = RCW_INIT;
  // Init state machine
  pGetRCWCalculatePointer()->RCWstatemachine = RCWState_Init;
  pGetRCWCalculatePointer()->RCWstatemachineLastCycle = RCWState_Init;
  pGetRCWCalculatePointer()->RCWBlockingTimeActive = FALSE;
}

/*****************************************************************************
  Functionname: RCW_InitObject                                  */ /*!

  @brief:this function initializes all LBS internal objects related RCW-data

  @description: include zone parameter and object property init

  @param[in]:void

  @return:void
*****************************************************************************/
void RCW_InitObject() {
  // init pro port object array
  // proPorts->RCWObjInfo
  uint8 uObjNumber = 0;

  for (uObjNumber = 0u; uObjNumber < RCW_INPUT_OBJECT_NUMBER; uObjNumber++) {
    RCW_InitOneObject(uObjNumber);
  }
}

/*****************************************************************************
  Functionname: RCW_InitOneObject                                  */ /*!

  @brief:this function initializes one LBS internal objects related RCW-data

  @description: include the object property init

  @param[in]:uObjNumber:the object array index number

  @return:void
*****************************************************************************/
void RCW_InitOneObject(uint8 uObjNumber) {
  RCW_Info_t* pRCWObjInfo = pGetRCWCalculatePointer_RCWObjInfoList(uObjNumber);

  // Init LBS RCW Attributions
  pRCWObjInfo->fTTCThreshold = 0.F;
  pRCWObjInfo->fCorridorOverlap = 0.F;
  pRCWObjInfo->fCorridorOccupancy = 0.F;
  pRCWObjInfo->fObjectOccupancy = 0.F;
  pRCWObjInfo->fCorridorOccThreshold = 0.F;
  pRCWObjInfo->fInCOrridorTime = 0.F;
  pRCWObjInfo->fYBreakThrough = 0.F;
  pRCWObjInfo->fHeadingFiltered = 0.F;
  pRCWObjInfo->uCorridorHitCnt = 0u;
  pRCWObjInfo->uMultiPathCnt = 0u;
  pRCWObjInfo->bRCWQuality = FALSE;
  pRCWObjInfo->bUpdateRecently = FALSE;
  pRCWObjInfo->bRCWRelevant = FALSE;
  pRCWObjInfo->bInRCWCorridor = FALSE;
  pRCWObjInfo->bObjCorridorBlocked = FALSE;
  pRCWObjInfo->bMultiPathObj = FALSE;
  pRCWObjInfo->bRCWWarningConditions = FALSE;
  pRCWObjInfo->bRCWWarning = FALSE;
  pRCWObjInfo->bOppositeOverlap = FALSE;
  pRCWObjInfo->bHeadingAngleInRange = FALSE;
}

/*****************************************************************************
  Functionname: RCW_InitGlobal                                  */ /*!

  @brief:this function initializes the internal global RCW data

  @description:initializes the RCW global data

  @param[in]:void

  @return:void
*****************************************************************************/
void RCW_InitGlobal() {
  RCWWarningInfo_t* tRCWWarningInfo = pGetRCWCalculatePointer_RCWWarningInfo();
  RCWStationaryBlocked_t* tRCWStationaryBlocker =
      pGetRCWCalculatePointer_RCWStationaryBlocker();
  RCWStatusCondition_t* pRCWStatusCondition =
      pGetRCWCalculatePointer_RCWStatusCondition();

  for (uint8 uIdx = 0u; uIdx < RCW_MAX_NOF_CORR_OBJS; uIdx++) {
    RCWCorridorObserver_t* tRCWCorridorObject =
        pGetRCWCalculatePointer_RCWCorridorObjs(uIdx);

    tRCWCorridorObject->fXDist = -TUE_C_F32_VALUE_INVALID;
    tRCWCorridorObject->fCorridorOccupancy = 0.F;
    tRCWCorridorObject->fXMin = -TUE_C_F32_VALUE_INVALID;
    tRCWCorridorObject->fXMax = -TUE_C_F32_VALUE_INVALID;
    tRCWCorridorObject->fInCorridorTime = 0.F;
    tRCWCorridorObject->uObjID = TUE_C_UI8_VALUE_INVALID;
  }

  tRCWWarningInfo->bRCWWarningActive = FALSE;
  tRCWWarningInfo->bRCWWarningActiveLastCycle = FALSE;
  tRCWWarningInfo->fTTC = TUE_C_F32_VALUE_INVALID;
  tRCWWarningInfo->fXObjectWarning = -TUE_C_F32_VALUE_INVALID;
  tRCWWarningInfo->uRCWWarningID = TUE_C_UI8_VALUE_INVALID;

  tRCWStationaryBlocker->bCorridorBlocked = FALSE;
  tRCWStationaryBlocker->bCorridorBlockedLastCycle = FALSE;
  tRCWStationaryBlocker->fBlockXPosition = -TUE_C_F32_VALUE_INVALID;

  // init state condition
  pRCWStatusCondition->bRCWActiveConditon = FALSE;
  pRCWStatusCondition->bRCWFailureCondition = FALSE;
  pRCWStatusCondition->bRCWHmiOpen = FALSE;
  pRCWStatusCondition->bRCWPassiveCondition = FALSE;
  pRCWStatusCondition->bRCWStandByCondition = FALSE;
}

/*****************************************************************************
  Functionname: RCW_PrePorecss                                  */ /*!

  @brief: RCW PreProcess function 

  @description: RCW PreProcess function to update BSDZone parameter and threshold

  @param[in]:LBS Global input,Road,Params

  @return:void
*****************************************************************************/
void RCW_PreProecss(const RCWInReq_st* reqPorts, const RCWParam_st* params) {
  // Cyclic update for RCW global variables
  LBSRCWCyclicGlobalUpdate();

  // sort all object in corrisor by range
  LBSRCWBlockedCorridorAnalysis(reqPorts, params);
}

/*****************************************************************************
  Functionname: LBSRCWCyclicGlobalUpdate                                  */ /*!

  @brief: Cyclic update of global RCW information

  @description: Cyclic update of global RCW information

  @param[in]:RCWCalculate

  @return:void
*****************************************************************************/
void LBSRCWCyclicGlobalUpdate() {
  // store internal corridor blocked status from last cycle
  RCWCalculate.RCWStationaryBlocker.bCorridorBlockedLastCycle =
      RCWCalculate.RCWStationaryBlocker.bCorridorBlocked;
  RCWCalculate.RCWStationaryBlocker.bCorridorBlocked = FALSE;

  // store internal global warning status from last cycle
  RCWCalculate.RCWWarningInfo.bRCWWarningActiveLastCycle =
      RCWCalculate.RCWWarningInfo.bRCWWarningActive;
  RCWCalculate.RCWWarningInfo.bRCWWarningActive = FALSE;
  RCWCalculate.RCWWarningInfo.fTTC = TUE_C_F32_VALUE_INVALID;
  RCWCalculate.RCWWarningInfo.fXObjectWarning = -TUE_C_F32_VALUE_INVALID;
  RCWCalculate.RCWWarningInfo.uRCWWarningID = TUE_C_UI8_VALUE_INVALID;

  RCWCalculate.RCWStatusCondition.bRCWActiveConditon = FALSE;
  RCWCalculate.RCWStatusCondition.bRCWFailureCondition = FALSE;
  RCWCalculate.RCWStatusCondition.bRCWHmiOpen = FALSE;
  RCWCalculate.RCWStatusCondition.bRCWPassiveCondition = FALSE;
  RCWCalculate.RCWStatusCondition.bRCWStandByCondition = FALSE;
}

/*****************************************************************************
  Functionname: LBSRCWBlockedCorridorAnalysis */ /*!

  @brief: sort all object in corrisor by range

  @description: sort all object in corrisor by range

  @param[in]:

  @return:void
*****************************************************************************/
void LBSRCWBlockedCorridorAnalysis(const RCWInReq_st* reqPorts,
                                   const RCWParam_st* params) {
  RCWCorridorObserver_Arry tRCWCorridorObj;
  const RCWGenObjList_st* pGenObjList = &reqPorts->GenObjList;
  const RCW_LBSInputInfo_st* pGetLBSObjInfo = &reqPorts->LBSInputInfo;
  uint8 uidx = 0;
  uint8 uIndex = 0;
  uint8 uObjID = 0;
  boolean bFound = FALSE;

  for (uidx = 0u; uidx < RCW_MAX_NOF_CORR_OBJS; uidx++) {
    tRCWCorridorObj[uidx].fXDist = -TUE_C_F32_VALUE_INVALID;
    tRCWCorridorObj[uidx].fXMax = -TUE_C_F32_VALUE_INVALID;
    tRCWCorridorObj[uidx].fXMin = -TUE_C_F32_VALUE_INVALID;
    tRCWCorridorObj[uidx].uObjID = TUE_C_UI8_VALUE_INVALID;
    tRCWCorridorObj[uidx].fInCorridorTime = 0.F;
    tRCWCorridorObj[uidx].fCorridorOccupancy = 0.F;
  }

  for (uObjID = 0U; uObjID < RCW_INPUT_OBJECT_NUMBER; uObjID++) {
    if (!RCWbGetObjIsDeleted(uObjID, pGenObjList)) {
      RCW_Info_t* pRCWObjInfo = pGetRCWCalculatePointer_RCWObjInfoList(uObjID);
      float32 fXObj = pGetObjLongDisplacement(uObjID, pGenObjList);

      if ((fXObj > params->RCWWarningParameter.fRCWCorrMinSDISIX) &&
          (fXObj < params->RCWWarningParameter.fRCWCorrMaxSDISIX) &&
          (pRCWObjInfo->bInRCWCorridor == TRUE))  // use last cycle
      {
        bFound = FALSE;

        // Object 1 to 4
        for (uidx = 0u; (uidx < RCW_MAX_NOF_CORR_OBJS) && (bFound == FALSE);
             uidx++) {
          // check whether the range of object is higher

          if (fXObj > tRCWCorridorObj[uidx].fXDist) {
            for (uIndex = (RCW_MAX_NOF_CORR_OBJS - 1u); (uIndex >= (uidx + 1u));
                 uIndex--) {
              tRCWCorridorObj[uIndex].fXDist =
                  tRCWCorridorObj[uIndex - 1u].fXDist;
              tRCWCorridorObj[uIndex].fXMax =
                  tRCWCorridorObj[uIndex - 1u].fXMax;
              tRCWCorridorObj[uIndex].fXMin =
                  tRCWCorridorObj[uIndex - 1u].fXMin;
              tRCWCorridorObj[uIndex].uObjID =
                  tRCWCorridorObj[uIndex - 1u].uObjID;
              tRCWCorridorObj[uIndex].fInCorridorTime =
                  tRCWCorridorObj[uIndex - 1u].fInCorridorTime;
              tRCWCorridorObj[uIndex].fCorridorOccupancy =
                  tRCWCorridorObj[uIndex - 1u].fCorridorOccupancy;
            }

            bFound = TRUE;
            tRCWCorridorObj[uidx].fXDist = fXObj;
            tRCWCorridorObj[uidx].fXMax =
                pGetLBSObjInfo->LBSObjInfoList[uObjID].ObjBorders.fXmax_met;
            tRCWCorridorObj[uidx].fXMin =
                pGetLBSObjInfo->LBSObjInfoList[uObjID].ObjBorders.fXmin_met;
            tRCWCorridorObj[uidx].uObjID = uObjID;
            tRCWCorridorObj[uidx].fInCorridorTime =
                pRCWObjInfo->fInCOrridorTime;
            tRCWCorridorObj[uidx].fCorridorOccupancy =
                pRCWObjInfo->fCorridorOccupancy;
          }
        }
      }
    }
  }

  // copy local structure to the global structure
  for (uidx = 0u; uidx < RCW_MAX_NOF_CORR_OBJS; uidx++) {
    RCWCorridorObserver_t* tRCWCorridorObject =
        pGetRCWCalculatePointer_RCWCorridorObjs(uidx);

    tRCWCorridorObject->fXDist = tRCWCorridorObj[uidx].fXDist;
    tRCWCorridorObject->fCorridorOccupancy =
        tRCWCorridorObj[uidx].fCorridorOccupancy;
    tRCWCorridorObject->fXMin = tRCWCorridorObj[uidx].fXMin;
    tRCWCorridorObject->fXMax = tRCWCorridorObj[uidx].fXMax;
    tRCWCorridorObject->fInCorridorTime = tRCWCorridorObj[uidx].fInCorridorTime;
    tRCWCorridorObject->uObjID = tRCWCorridorObj[uidx].uObjID;
  }
}

/*****************************************************************************
  Functionname: RCW_MainProcess                                  */ /*!

  @brief:the RCW Warning main logical function

  @description:the RCW Warning main logical function

  @param[in]:reqPorts,params,proPorts,debugInfo

  @return:void
*****************************************************************************/
void RCW_MainProcess(const RCWInReq_st* reqPorts, const RCWParam_st* params,
                     RCWOutPro_st* proPorts, RCWDebug_t* debugInfo) {
  const RCWGenObjList_st* pGenObjList = &reqPorts->GenObjList;
  const RCW_LBSInputInfo_st* pGetLBSInputInfo = &reqPorts->LBSInputInfo;
  const RCWVehicleInfo_t* pGetEgoInfo = &reqPorts->EgoVehInfo;
  RCWStationaryBlocked_t* pRCWStationaryBlocker =
      pGetRCWCalculatePointer_RCWStationaryBlocker();
  // RCW_Info_Array* pRCWCalcAllObjInfo =
  //     pGetRCWCalculatePointer_RCWAllObjInfo();
  RCWWarningInfo_t* pRCWCalcWarningInfo =
      pGetRCWCalculatePointer_RCWWarningInfo_t();
  //RCWStatusCondition_t* pGetRCWStateConditions =
  // pGetRCWCalculatePointer_RCWStatusCondition();
  // RCWStateMachine_t* pGetRCWstatemachine =
  // pGetRCWCalculatePointer_RCWstatemachine();

  for (uint8 uObjectIndex = 0u; uObjectIndex < RCW_INPUT_OBJECT_NUMBER;
       uObjectIndex++) {
    RCW_Info_t* pRCWCalcObjInfo =
        pGetRCWCalculatePointer_RCWObjInfoList(uObjectIndex);

    if (!RCWbGetObjIsDeleted(uObjectIndex, pGenObjList) &&
        (pGenObjList->aObject[uObjectIndex].Kinemactic.fVabsX_mps >
         RCW_EM_GEN_OBJECT_MIN_VELOCITY_X)) {
      // Set Vrel dependent TTC threshold
      LBSRCWCustomCalculateTTCThreshold(params, pGenObjList, uObjectIndex,
                                        pRCWCalcObjInfo);

      // check if the object quality is dufficient
      LBSRCWCheckObjectQuality(pGenObjList, uObjectIndex, pRCWCalcObjInfo,
                               pGetLBSInputInfo);

      // check if the object is a multipath object
      // LBSRCWCheckMultipathObject(pGenObjList, uObjectIndex,
      // pRCWCalcAllObjInfo, pRCWCalcObjInfo);

      // check the update status of the object
      LBSRCWCheckUpdateStatus(pGenObjList, uObjectIndex, pRCWCalcObjInfo,
                              pGetLBSInputInfo);

      // calculate the heading based lateral breakthrough
      LBSRCWCalculateHeadingFilter(pGenObjList, uObjectIndex, pRCWCalcObjInfo,
                                   pGetLBSInputInfo);

      // Check the heading angle is in the specified range
      LBSRCWCheckHeadingInRange(pGenObjList, uObjectIndex, pRCWCalcObjInfo,
                                params);

      // perform RCW Corridor Calculation and association
      LBSRCWProcessCorridor(pGenObjList, uObjectIndex, pRCWCalcObjInfo,
                            pGetLBSInputInfo, params, pGetEgoInfo);

      // check if the object is relevant for RCW
      LBSRCWCheckObjectRelevence(pGenObjList, uObjectIndex, pRCWCalcObjInfo,
                                 params, pGetLBSInputInfo);

      // check if the RCW corridor is blocked by another object
      LBSRCWCheckBlockedCorridor(pGenObjList, uObjectIndex, pRCWCalcObjInfo,
                                 params, pGetLBSInputInfo);

      // choose occupancy threshold
      LBSRCWChooseOccupancyThreshold(pGenObjList, uObjectIndex, pRCWCalcObjInfo,
                                     params);

      // check if the object fulfill the warning conditions
      LBSRCWCheckWarningConditions(pGenObjList, uObjectIndex, pRCWCalcObjInfo,
                                   pGetLBSInputInfo);

      // perform the final warning decision
      LBSRCWFinalWarningDecision(pGenObjList, uObjectIndex, pRCWCalcObjInfo,
                                 pGetLBSInputInfo, params, pRCWCalcWarningInfo);

      // store warning information of the closet warning object
      LBSRCWStoreWarningObjInfor(pGenObjList, uObjectIndex, pRCWCalcObjInfo,
                                 pGetLBSInputInfo, pRCWCalcWarningInfo);
    }
  }

  // determine if the corridor is blocked by stationary target at standstill
  LBSRCWCorrIsStationaryBlocked(pGetEgoInfo, pGenObjList, params,
                                pRCWStationaryBlocker);

  // store debug signal
  memcpy(&debugInfo->Debug_RCWWarningInfo, pRCWCalcWarningInfo,  //NOLINT
         sizeof(RCWWarningInfo_t));
  memcpy(&debugInfo->Debug_RCWObjInfo,  //NOLINT
         &pGetRCWCalculatePointer()->RCWObjInfoList,
         (sizeof(RCW_Info_t) * RCW_INPUT_OBJECT_NUMBER));
  memcpy(&debugInfo->Debug_RCWCorridorObjs,  //NOLINT
         &pGetRCWCalculatePointer()->RCWCorridorObjs,
         (sizeof(RCWCorridorObserver_t) * RCW_MAX_NOF_CORR_OBJS));
}

/*****************************************************************************
  Functionname: RCW_PostProcess                                  */ /*!

  @brief:the RCW post output function

  @description:output the RCW calculate result to proPorts 

  @param[in]:reqPorts,params,proPorts,debugInfo

  @return:void
*****************************************************************************/
void RCW_PostProcess(const RCWInReq_st* reqPorts, const RCWParam_st* params,
                     RCWOutPro_st* proPorts, RCWDebug_t* debugInfo) {
  const RCWVehicleInfo_t* pGetEgoInfo = &reqPorts->EgoVehInfo;
  const RCWPreProcessInput_t* pGetPreProcessInput =
      &reqPorts->RCWPreProcessInput;
  RCWStationaryBlocked_t* pRCWStationaryBlocker =
      pGetRCWCalculatePointer_RCWStationaryBlocker();
  RCWWarningInfo_t* pRCWCalcWarningInfo =
      pGetRCWCalculatePointer_RCWWarningInfo_t();
  RCWStatusCondition_t* pGetRCWStateConditions =
      pGetRCWCalculatePointer_RCWStatusCondition();
  RCWStateMachine_t* pGetRCWstatemachine =
      pGetRCWCalculatePointer_RCWstatemachine();
  // RCWStateMachine_t* pGetRCWstatemachineLastCycle =
  //     pGetRCWCalculatePointer_RCWstatemachineLastCycle();

  // RCW state condition process
  LBSRCWStateConditionProcess(pGetRCWStateConditions, pGetEgoInfo,
                              pGetPreProcessInput, params, pRCWCalcWarningInfo,
                              pRCWStationaryBlocker, debugInfo);

  // RCW Status
  LBSRCWStateMachineProcess(pGetRCWstatemachine, pGetRCWStateConditions,
                            debugInfo);

  // RCW preprocess
  LBSRCWBlockingTimeProcess(pGetRCWstatemachine, params);

  // RCW hmi output
  LBSRCWHmiOutput(pGetRCWstatemachine, proPorts, pRCWCalcWarningInfo, params);

  // printf("RCW state is %d\t", *pGetRCWstatemachine);
}
