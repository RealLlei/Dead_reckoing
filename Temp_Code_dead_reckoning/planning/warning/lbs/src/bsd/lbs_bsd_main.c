/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "lbs_bsd_main.h"
#include "lbs_bsd_calculation.h"
#include "lbs_bsd_par.h"

/*****************************************************************************
  VARIABLES
*****************************************************************************/

/*****************************************************************************
  FUNCTION
*****************************************************************************/

/*****************************************************************************
  Functionname: BSDExec                                  */ /*!

  @brief: BSD Exec main function 

  @description: BSD main function

  @param[in]:LBS input,params

  @return:void
*****************************************************************************/
void LBS_BSDExec(const BSDInReq_st* reqPorts, const BSDParam_st* params,
                 BSDOutPro_st* proPorts, BSDDebug_t* debugInfo) {
  boolean bBSDFunctionActive = reqPorts->BSDSystemParam.bBSDFunctionActive;
  boolean bBSDFunctionActiveLast =
      pGetBSDCalculatePointer()->BSDRunState.bBSDFunctionActionLastCycle;
  BSDState_t eBSDState = pGetBSDCalculatePointer()->BSDRunState.eBSDState;
  // BSDState_t eBSDState = BSDCalculate.BSDRunState.eBSDState;

  switch (eBSDState) {
    case BSD_OK:
      /*Function switch enable and function has been init*/
      if (bBSDFunctionActive == TRUE) {
        BSD_PreProecss(reqPorts, params, proPorts, debugInfo);
        BSD_MainProcess(reqPorts, params, proPorts, debugInfo);
        BSD_PostProcess(reqPorts, params, proPorts, debugInfo);
      } else {
        /*Function exit reset*/
        if (bBSDFunctionActiveLast == TRUE) {
          LBS_BSD_Init_Reset();
          pGetBSDCalculatePointer()->BSDRunState.eBSDState = BSD_OK;
        }
      }
      break;
    case BSD_INIT:
      /*First enter reset*/
      LBS_BSD_Init_Reset();
      pGetBSDCalculatePointer()->BSDRunState.eBSDState = BSD_OK;
      break;
    default:
      LBS_BSD_Init_Reset();
      pGetBSDCalculatePointer()->BSDRunState.eBSDState = BSD_OK;
      break;
  }
  pGetBSDCalculatePointer()->BSDRunState.bBSDFunctionActionLastCycle =
      bBSDFunctionActive;
}

/*****************************************************************************
  Functionname: LBS_BSD_Init_Reset                                  */ /*!

  @brief: BSD function init reset

  @description:the function first exec reset process

  @param[in]:void

  @return: void
*****************************************************************************/
void LBS_BSD_Init_Reset() {
  // Init internal object data
  InitBSDCalculate();
  BSD_InitObject();
  // Init internal global data
  BSD_InitGlobal();
  // Init BSD run flag
  pGetBSDCalculatePointer()->BSDRunState.eBSDState = BSD_INIT;
}

/*****************************************************************************
  Functionname: BSD_InitObject                                  */ /*!

  @brief:this function initializes all LBS internal objects related BSD-data

  @description: include zone parameter and object property init

  @param[in]:void

  @return:void
*****************************************************************************/
void BSD_InitObject() {
  // init pro port object array
  // proPorts->BSDObjInfo
  uint8 uObjNumber = 0;

  for (uObjNumber = 0U; uObjNumber < BSD_INPUT_OBJECT_NUMBER; uObjNumber++) {
    BSD_InitOneObject(uObjNumber);
  }
}

/*****************************************************************************
  Functionname: BSD_InitOneObject                                  */ /*!

  @brief:this function initializes one LBS internal objects related BSD-data

  @description: include zone parameter and object property init

  @param[in]:uObjNumber:the object array index number

  @return:void
*****************************************************************************/
void BSD_InitOneObject(uint8 uObjNumber) {
  BSD_Info_t* pBSDObjInfo = NULL;
  BSDZone_ObjPar* pBSDObjZonePar = NULL;

  // Get pointer of Calculate data
  pBSDObjInfo = pGetBSDCalculatePointer_ObjInfo(uObjNumber);
  pBSDObjZonePar = pGetBSDCalculatePointer_BSDZoneObjPar(uObjNumber);

  // init check data like delay time,appearance
  pBSDObjInfo->fSoTDelayTime_s = 0.F;
  pBSDObjInfo->fRearConf_nu = 0.F;
  pBSDObjInfo->fBSDZoneObjXmin_met = 0.F;
  pBSDObjInfo->ubAppearance_nu = BSD_APPEAR_INVALID;
  pBSDObjInfo->ubHitsInFront_nu = 0U;
  pBSDObjInfo->ubHitsInSide_nu = 0U;
  pBSDObjInfo->ubHitsInRear_nu = 0U;
  pBSDObjInfo->ubGrdHitCounter_nu = 0U;
  pBSDObjInfo->ubBehindGrdCounter_nu = 0U;
  pBSDObjInfo->ubClass_nu = BSD_CLASS_UNDEFINED;
  pBSDObjInfo->ubOwnLaneCounter_nu = 0U;

  // init check boolean result
  pBSDObjInfo->bBSDRelevant = FALSE;
  pBSDObjInfo->bBSDWarning = FALSE;
  pBSDObjInfo->bCreateBehindGRD = FALSE;
  pBSDObjInfo->bFastSoT = FALSE;
  pBSDObjInfo->bInBSDZone = FALSE;
  pBSDObjInfo->bInSOTZone = FALSE;
  pBSDObjInfo->bInSOTZonePrevious = FALSE;
  pBSDObjInfo->bIsSoT = FALSE;
  pBSDObjInfo->bLivedLongEnough = FALSE;
  pBSDObjInfo->bObjectAndZoneOverlap = FALSE;
  pBSDObjInfo->bObjectBehindGRD = FALSE;
  pBSDObjInfo->bObjectOnOwnlane = FALSE;
  pBSDObjInfo->bPlausibility = FALSE;
  pBSDObjInfo->bPossibleWrappedObj = FALSE;
  pBSDObjInfo->bQualityEnough = FALSE;
  pBSDObjInfo->bShortWarn = FALSE;
  pBSDObjInfo->bSoTDelayActive = FALSE;
  pBSDObjInfo->bUpdatedRecently = FALSE;
  pBSDObjInfo->bUpdatedRecentlyWeak = FALSE;

  // init each object warning zone parameter
  pBSDObjZonePar->fZoneXmax_met = 0.0F;
  pBSDObjZonePar->fZoneXmin_met = 0.0F;
  pBSDObjZonePar->fZoneYmax_met = 0.0F;
  pBSDObjZonePar->fZoneYmin_met = 0.0F;
}

/*****************************************************************************
  Functionname: BSD_InitGlobal                                  */ /*!

  @brief:this function initializes the internal global BSD data

  @description:initializes the BSD global data and Zone parameter

  @param[in]:void

  @return:void
*****************************************************************************/
void BSD_InitGlobal() {
  // init BsdGlobal struct
  BSDCalculate_st* pBSDCal = pGetBSDCalculatePointer();
  BSD_Globals_t* pBSDGlobal = pGetBSDCalculatePointer_BSDGlobals();
  BSDGlobal_BSDZone_st* pBSDGlobalZonePar =
      pGetBSDCalculatePointer_BSDGlobaZonePar();

  // Init BSD Global parameter
  memset(pBSDGlobal, 0U, sizeof(BSD_Globals_t));
  pBSDGlobal->fMinAssocProbFront_nu = BSD_ASSOCPROB_INIT;
  pBSDGlobal->fMinAssocProbSideRear_nu = BSD_ASSOCPROB_INIT;
  pBSDGlobal->fMinXmoved_met = 0.0F;
  pBSDGlobal->fBSDZoneXmin_met = TUE_C_F32_VALUE_INVALID;

  // Init left and right side warning zone parameter
  memset(pBSDGlobalZonePar, 0U, sizeof(BSDGlobal_BSDZone_st));

  // Init the first cycle object id array
  for (uint8 uObj = 0; uObj < BSD_INPUT_OBJECT_NUMBER; uObj++) {
    pBSDCal->LastObjIDList[uObj] = TUE_C_UI16_VALUE_INVALID;
  }
}

/*****************************************************************************
  Functionname: BSD_PrePorecss                                  */ /*!

  @brief: BSD PreProcess function 

  @description: BSD PreProcess function to update BSDZone parameter and threshold

  @param[in]:LBS Global input,Road,Params

  @return:void
*****************************************************************************/
void BSD_PreProecss(const BSDInReq_st* reqPorts, const BSDParam_st* params,
                    BSDOutPro_st* proPorts, BSDDebug_t* debugInfo) {
  const BSDVehicleInfo_t* pEgoInfo = &reqPorts->EgoVehInfo;
  const BSD_LBSGlobalInfo_t* pLBSGlobalInput =
      &reqPorts->LBSInputInfo.LBSGlobalInfo;
  const BSDRoad_t* pRoad = &reqPorts->Road;
  const BSDSensorMounting_t* pSensorMounting =
      &params->SensorMounting.SensorLeft;  // Temp to use left side
  //const BSDVehParameter_t* pBSDVehParameter = &params->BSDVehParameter;

  /*update left and right side basic BSD Zone parameter at the same time*/
  BSDGetBSDZoneParameters(pLBSGlobalInput, pEgoInfo, pRoad, params);

  /*Calculate object association  threshold*/
  BSDCycleGlobalUpdate(pSensorMounting);

  /*Calculate check object moving or stationary threshold of velocity */
  BSDCalculateVxThreshMovStatClassification(pEgoInfo);

  /*Clear object BSD calculate array if the object id changed this cycle*/
  BSDInitObjCalArray(&reqPorts->GenObjList);

  /*Store last cycle warning flag*/
  pGetBSDCalculatePointer_BSDGlobals()->bBSDWarnActiveLeftLastCycle =
      pGetBSDCalculatePointer_BSDGlobals()->bBSDWarnActiveLeft;
  pGetBSDCalculatePointer_BSDGlobals()->bBSDWarnActiveRightLastCycle =
      pGetBSDCalculatePointer_BSDGlobals()->bBSDWarnActiveRight;
}

/*****************************************************************************
  Functionname: BSDInitObjCalArray                                  */ /*!

  @brief: Clear BSD calculate array

  @description: Clear BSD calculate array if the object id changed
                and save the new object for next cycle to judge if the object id change

  @param[in]:pGenObjList: the LBS object list input pointer

  @return:void
*****************************************************************************/
void BSDInitObjCalArray(const BSDGenObjList_st* pGenObjList) {
  uint8 uObj = 0;

  const BSD_GenObject_st* pGenObjInfo = NULL;
  BSDCalculate_st* pBSDCalculate = pGetBSDCalculatePointer();

  for (uObj = 0U; uObj < BSD_INPUT_OBJECT_NUMBER; uObj++) {
    pGenObjInfo = pGetBSDGenObjListPointer_Object(uObj, pGenObjList);

    if (pBSDCalculate->LastObjIDList[uObj] != pGenObjInfo->General.uiID_nu) {
      // object id have been change,init the calculate array
      BSD_InitOneObject(uObj);
      // save current object id
      pBSDCalculate->LastObjIDList[uObj] = pGenObjInfo->General.uiID_nu;
    }
  }
}

/*****************************************************************************
  Functionname: BSDGetBSDZoneParameters                                  */ /*!

  @brief:Update current both side BSDZone Parameter 

  @description:update both side BSDZone Parameter

  @param[in]:LBSGlobalsInfo,EgoInfo,RoadInfo,params

  @return:void
*****************************************************************************/

void BSDGetBSDZoneParameters(const BSD_LBSGlobalInfo_t* pLBSGlobalInput,
                             const BSDVehicleInfo_t* pEgoInfo,
                             const BSDRoad_t* pRoad,
                             const BSDParam_st* params) {
  BSD_Globals_t* pBSDGlobal = pGetBSDCalculatePointer_BSDGlobals();
  BSDGlobal_BSDZone_st* pBSDGlobalZonePar =
      pGetBSDCalculatePointer_BSDGlobaZonePar();
  BSDZoneParameter_t* pLeftZonePar = &pBSDGlobalZonePar->BSDZoneParameterLeft;
  BSDZoneParameter_t* pRightZonePar = &pBSDGlobalZonePar->BSDZoneParameterRight;
  const BsdZone_t* pBSDZoneParams = &params->BsdZone;
  //const BSDSensorMounting_t* pSenserMountLeft =
  //&params->SensorMounting.SensorLeft;
  const BSDSensorMounting_t* pSenserMountRight =
      &params->SensorMounting.SensorRight;

  const float32 fVehicleWidth = params->BSDVehParameter.fVehicleWidth_met;
  const float32 fCenter2Axis = params->BSDVehParameter.fVehCenter2FrontAxis_met;
  const float32 fXmin = pBSDZoneParams->fXmin_met;
  const float32 fXmax = pBSDZoneParams->fXmax_met;
  const float32 fYmin = pBSDZoneParams->fYmin_met;
  const float32 fYmax = pBSDZoneParams->fYmax_met;
  const float32 fX_hys = pBSDZoneParams->fHysteresisX_met;
  const float32 fYmin_hys = pBSDZoneParams->fHysteresisYmin_met;
  const float32 fYmax_hys = pBSDZoneParams->fHysteresisYmax_met;
  float32 fXminAdapt = 0.F;

  if (pBSDGlobal->fBSDZoneXmin_met == TUE_C_F32_VALUE_INVALID) {
    // Init BSD Zone X Min value
    // Trivial case: not in a narrow curve,check rectangular boundaries

    pBSDGlobal->fBSDZoneXmin_met =
        pBSDZoneParams->fXmin_met -
        params->BSDVehParameter.fVehCenter2FrontAxis_met;
  }
  const float32 fZoneXminLastCycle = pBSDGlobal->fBSDZoneXmin_met;

  /**************************************************************
   *update X min adapter range
   **************************************************************/
  fXminAdapt = BSDCalculateAdaptedBSDZoneLength(
      pLBSGlobalInput, pEgoInfo, pRoad, pSenserMountRight, fZoneXminLastCycle,
      fCenter2Axis);
  pBSDGlobal->fBSDZoneXminStatic_met = fXmin - fCenter2Axis;
  // add vehicle width (consider sensor mounting) to the Zone for transform
  // coordinator from zone to ego
  pLeftZonePar->fBSDZoneYmin_met = fYmin;
  pLeftZonePar->fBSDZoneYmax_met = fYmax;
  pLeftZonePar->fBSDZoneYminWithHyst_met =
      pLeftZonePar->fBSDZoneYmin_met - fYmin_hys;
  pLeftZonePar->fBSDZoneYmaxWithHyst_met =
      pLeftZonePar->fBSDZoneYmax_met + fYmax_hys;

  /**************************************************************
   *update left side BSD Zone parameter
   **************************************************************/
  // Object coordinate has been translate to AUTOSAR,so not need to
  // compensation sensor mount distance
  pLeftZonePar->fBSDZoneXmin_met = fXminAdapt;
  pLeftZonePar->fBSDZoneXminWithHyst_met = fXminAdapt - fX_hys;
  pLeftZonePar->fBSDZoneXmax_met = fXmax - fCenter2Axis;
  pLeftZonePar->fBSDZoneXmaxWithHyst_met =
      pLeftZonePar->fBSDZoneXmax_met + fX_hys;
  /**************************************************************
   *update right side BSD Zone parameter                         *
   **************************************************************/
  pRightZonePar->fBSDZoneXmin_met = fXminAdapt;
  pRightZonePar->fBSDZoneXminWithHyst_met = fXminAdapt - fX_hys;
  pRightZonePar->fBSDZoneXmax_met = fXmax - fCenter2Axis;
  pRightZonePar->fBSDZoneXmaxWithHyst_met =
      pRightZonePar->fBSDZoneXmax_met + fX_hys;

  pRightZonePar->fBSDZoneYmin_met = -fYmax;
  pRightZonePar->fBSDZoneYmax_met = -fYmin;
  pRightZonePar->fBSDZoneYminWithHyst_met =
      pRightZonePar->fBSDZoneYmin_met - fYmin_hys;
  pRightZonePar->fBSDZoneYmaxWithHyst_met =
      pRightZonePar->fBSDZoneYmax_met + fYmax_hys;
}

/*****************************************************************************
  Functionname: BSDCycleGlobalUpdate                                  */ /*!

  @brief:Calculate BSD Global threshold and store

  @description:according to sensor mounting angle to get the BSD threshold

  @param[in]:sensorMountingInfo

  @return:void
*****************************************************************************/
void BSDCycleGlobalUpdate(const BSDSensorMounting_t* pSensorMounting) {
  // update global information
  BSD_Globals_t* pBSDGlobal = pGetBSDCalculatePointer_BSDGlobals();
  const float32 fMountingAngleDeg =
      RAD2DEG(fABS(pSensorMounting->fRollAngle_rad)) - 90.0F;

  // Calculate front association  threshold
  pBSDGlobal->fMinAssocProbFront_nu = GDBmathLinFuncLimBounded(
      fMountingAngleDeg, BSD_LI_ASSOC_MIN_MOUNTINGANGLE,
      BSD_LI_ASSOC_MAX_MOUNTINGANGLE, BSD_LI_ASSOC_MIN_MINPROB_FRONT,
      BSD_LI_ASSOC_MAX_MINPROB_FRONT);

  // Calculate side association  threshold
  pBSDGlobal->fMinAssocProbSideRear_nu = GDBmathLinFuncLimBounded(
      fMountingAngleDeg, BSD_LI_ASSOC_MIN_MOUNTINGANGLE,
      BSD_LI_ASSOC_MAX_MOUNTINGANGLE, BSD_LI_ASSOC_MIN_MINPROB_SIDEREAR,
      BSD_LI_ASSOC_MAX_MINPROB_SIDEREAR);

  // Calculate x moved association  threshold
  pBSDGlobal->fMinXmoved_met = GDBmathLinFuncLimBounded(
      fMountingAngleDeg, BSD_LI_ASSOC_MIN_MOUNTINGANGLE,
      BSD_LI_ASSOC_MAX_MOUNTINGANGLE, BSD_LI_ASSOC_MIN_MINXMOVED,
      BSD_LI_ASSOC_MAX_MINXMOVED);

  // init SOT object number counter
  pBSDGlobal->ScenarioObserver.uNumberSoTObjsLastCycle_nu =
      pBSDGlobal->ScenarioObserver.uNumberSoTObjs_nu;
  pBSDGlobal->ScenarioObserver.uNumberSoTObjs_nu = 0U;
}

/*****************************************************************************
  Functionname: BSDCalculateVxThreshMovStatClassification */ /*!

  @brief:Update object moving state velocity threshold

  @description:according ego speed to get the move state velocity threshold 

  @param[in]:EgoInfo;

  @return:void
*****************************************************************************/
void BSDCalculateVxThreshMovStatClassification(
    const BSDVehicleInfo_t* pEgoInfo) {
  BSD_Globals_t* pBSDGlobal = pGetBSDCalculatePointer_BSDGlobals();

  // base ego speed to interpolation fVxThreshMovStat_mps value
  pBSDGlobal->fVxThreshMovStat_mps = GDBmathLinFuncLimBounded(
      pEgoInfo->fegoVelocity_mps, BSD_LI_MOVSTAT_EGOSPEED_MIN,
      BSD_LI_MOVSTAT_EGOSPEED_MAX, BSD_LI_MOVSTAT_VXTHRESH_MIN,
      BSD_LI_MOVSTAT_VXTHRESH_MAX);
}

/*****************************************************************************
  Functionname: BSD_MainProcess                                  */ /*!

  @brief:the BSD Warning main logical function

  @description:the BSD Warning main logical function

  @param[in]:reqPorts,params,proPorts,debugInfo

  @return:void
*****************************************************************************/
void BSD_MainProcess(const BSDInReq_st* reqPorts, const BSDParam_st* params,
                     BSDOutPro_st* proPorts, BSDDebug_t* debugInfo) {
  uint16 uObj = 0;
  const BSDVehicleInfo_t* pEgoInfo = &reqPorts->EgoVehInfo;
  const BSDGenObjList_st* pGenObjList = &reqPorts->GenObjList;
  const BSDSRRObjList_st* pSRRObjList = &reqPorts->SRRObjList;
  const BSDRoad_t* pRoad = &reqPorts->Road;
  const BSD_LBSInputInfo_st* pLBSInputInfo = &reqPorts->LBSInputInfo;
  const BSDSystemParam_t* pBSDSystemParam = &reqPorts->BSDSystemParam;

  const BSD_GenObject_st* pGenObjInfo = NULL;
  const BSD_SRRObject_st* pSRRObjInfo = NULL;

  const BsdZone_t* pBsdZonePar = &params->BsdZone;
  //const BSDSensorMounting_t* pSensorMounting =
  // &params->SensorMounting.SensorLeft;  // Temp to use left side parameter
  const BSDWarningParameter_t* pBSDWarnParameter = &params->BSDWarningParameter;
  const BSDVehParameter_t* pVehParameter = &params->BSDVehParameter;

  float32 fBSDGlobalWarningDistX = 0.F;

  BSD_Globals_t* pBSDGlobal = pGetBSDCalculatePointer_BSDGlobals();
  BSD_Info_t* pBSDObjInfo = NULL;

  // Rest Warning flag from last cycle
  pBSDGlobal->bBSDWarnActive = FALSE;
  pBSDGlobal->bBSDWarnActiveLeft = FALSE;
  pBSDGlobal->bBSDWarnActiveRight = FALSE;
  pBSDGlobal->fBSDWarnActiveLeftDistX_met = -F32_VALUE_INVALID;
  pBSDGlobal->fBSDWarnActiveRightDistX_met = -F32_VALUE_INVALID;

  for (uObj = 0U; uObj < BSD_INPUT_OBJECT_NUMBER; uObj++) {
    pBSDObjInfo = pGetBSDCalculatePointer_ObjInfo(uObj);
    pGenObjInfo = pGetBSDGenObjListPointer_Object(uObj, pGenObjList);
    pSRRObjInfo = pGetBSDSRRObjListPointer_Object(uObj, pSRRObjList);

    if (!bGetObjIsDeleted(uObj, pGenObjList)) {
      // CHEKED: update BSD Warning active flag by current object direction flag
      BSDUpdateBSDWarningActiveFlag(uObj, pGenObjInfo);

      // CHEKED: update BSD zone with hysteresis by current object direction
      // flag
      BSDUpdateBSDZoneWithHyst(uObj, pGenObjInfo, pBSDGlobal);

      // CHEKED: judge sensor side to update current global BSD Zone parameter
      BSDUpdateGlobalZoneParameters(uObj, pGenObjInfo);

      // CHEKED: base on current object warning state to adjust object BSD zone
      // area
      BSDGetZoneParametersForCurrentObject(uObj, pGenObjInfo, pRoad,
                                           pBsdZonePar);

      // CHEKED: Calculate object compensation angle for SectorCuts
      BSDCalculateSectorCuts(uObj, pGenObjInfo, &pLBSInputInfo->LBSGlobalInfo,
                             pEgoInfo, pRoad, pVehParameter);

      // CHEKED: check object appearance to the BSD zone from front,side or rear
      pBSDObjInfo->ubAppearance_nu = BSDClassifyAppreance(
          uObj, pGenObjInfo, pSRRObjInfo, pLBSInputInfo, pVehParameter);

      // CHEKED: check whether the object is in the BSD Zone area
      pBSDObjInfo->bInBSDZone =
          BSDCheckObjectInBSDZone(uObj, pLBSInputInfo, pEgoInfo, pRoad);

      // CHEKED: check whether the object is in the SOT Zone area
      pBSDObjInfo->bInSOTZone =
          BSDCheckObjectInSOTZone(uObj, pGenObjInfo, pLBSInputInfo, pRoad);

      // CHEKED: check whether the object has enough overlap with the zero to
      // warn
      pBSDObjInfo->bObjectAndZoneOverlap =
          BSDCheckObjectZoneOverlap(uObj, pGenObjInfo, pLBSInputInfo, pRoad);
#if ST_PERCEPTION
      // Think HZ Perception have delete the car behind the grid
      // CHEKED: update possibly guardrail object counter
      BSDCalculateUpdateGrdCounter(uObj, pGenObjInfo, pSRRObjInfo,
                                   pLBSInputInfo, pEgoInfo, pRoad,
                                   pSensorMounting);
      // check whether the object is behind guardrail
      pBSDObjInfo->bObjectBehindGRD =
          BSDCheckObjectBehindGRD(uObj, pGenObjInfo, pLBSInputInfo);
#endif
      // HZ: check whether the object is behind guardrail,return false
      pBSDObjInfo->bObjectBehindGRD =
          BSDCheckObjectBehindGRD_hz(uObj, pGenObjInfo, pLBSInputInfo);
      // CHEKED:
      BSDCalculateUpdateOwnlaneCounter(uObj, pGenObjInfo, pSRRObjInfo,
                                       pLBSInputInfo, pRoad);
      // CHEKED: count up if object is on the own lane
      pBSDObjInfo->bObjectOnOwnlane =
          BSDCheckObjectOnOwnlane(uObj, pGenObjInfo);

      // check whether the object quality is high enough
      // CHEKED AND MODIFY: Ues the condition uiMaintenanceState_nu == 2;
      pBSDObjInfo->bQualityEnough = BSDCheckObjectQuality(
          uObj, pGenObjInfo, pSRRObjInfo, pLBSInputInfo, pEgoInfo);

#if ST_PERCEPTION
      // check whether the object lived enough
      pBSDObjInfo->bLivedLongEnough =
          BSDCheckObjectLivedLongEnough(uObj, pLBSInputInfo);
#endif
      // CHEKED AND MODIFY: check whether the object lived enough
      // ignore the grid
      pBSDObjInfo->bLivedLongEnough =
          BSDCheckObjectLivedLongEnough_hz(uObj, pLBSInputInfo);

      // CHEKED: check whether the object was updated in last few cycle
      // USED ubMeasuredTargetFrequency and updaterate
      BSDCheckObjectUpdatedRecently(uObj, pSRRObjInfo);

      // CHEKED: classify object
      BSDClassifyObject(uObj, pGenObjInfo, pSRRObjInfo, pLBSInputInfo, pEgoInfo,
                        pRoad, pVehParameter);

      // classify objects which have a high enough guardrail hit counter
      // CHEKED: Because BSDCalculateUpdateGrdCounter()never be called,so
      // pBSDObj->ubClass_nu = BSD_CLASS_STATIC_GRDHITCOUNTER,never true

#if ST_PERCEPTION
      BSDClassifyGRD(uObj, pGenObjInfo, pSRRObjInfo, pLBSInputInfo, pEgoInfo,
                     pRoad);
#endif
      // CHEKED: check object is BSD relevant
      pBSDObjInfo->bBSDRelevant =
          BSDCheckObjectIsRelevant(uObj, pGenObjInfo, pVehParameter);

      // CHEKED: check whether the object is SOT target
      pBSDObjInfo->bIsSoT = BSDCheckObjectSoT(uObj, pGenObjInfo, pLBSInputInfo,
                                              pBSDWarnParameter, pVehParameter);

      // CHEKED:  check object for Fast SoT condition
      pBSDObjInfo->bFastSoT = BSDCheckObjectFastSoT(
          uObj, pGenObjInfo, pBSDWarnParameter, pBSDSystemParam);

      // CHEKED: check SOT delay
      pBSDObjInfo->bSoTDelayActive =
          BSDCheckObjectSoTDelay(uObj, pBSDWarnParameter);

      // CHEKED: calculate the new SOT object delay time
      BSDCalculateUpdateObjectSoTDelay(uObj, pGenObjInfo, pSRRObjInfo,
                                       pLBSInputInfo, pBSDSystemParam,
                                       pVehParameter);

      // CHEKED: update the scenario observer
      BSDUpdateScenarioObserver(uObj, pGenObjInfo);

      // CHEKED: check whether the object movement is plausible
      pBSDObjInfo->bPlausibility =
          BSDCheckObjectPlausibility(uObj, pGenObjInfo, pSRRObjInfo,
                                     pLBSInputInfo, pEgoInfo, pVehParameter);

      // CHEKED: check for short warning,to inhibition short warn
      pBSDObjInfo->bShortWarn = BSDCheckObjectShortWarning(
          uObj, pGenObjInfo, pLBSInputInfo, pRoad, pBSDWarnParameter);

      // CHEKED: check whether all warning conditions are fulfilled
      pBSDObjInfo->bBSDWarning = BSDCheckObjectWarningConditions(
          uObj, pSRRObjInfo, pLBSInputInfo, pEgoInfo->uBSDGearPosition);

      // CHEKED: Calculate real distance if object in warning state
      fBSDGlobalWarningDistX = BSDCalculateObjectRelDist(
          uObj, pGenObjInfo, pSRRObjInfo, pLBSInputInfo);

      // CHEKED: check the object warning state,and switch on the
      // globapSRRObjInfol

      BSDSetGlobalWarning(uObj, pBSDObjInfo, fBSDGlobalWarningDistX);
    }
  }
}

/*****************************************************************************
  Functionname: BSD_PostProcess                                  */ /*!

  @brief:the BSD post output function

  @description:output the BSD calculate result to proPorts 

  @param[in]:reqPorts,params,proPorts,debugInfo

  @return:void
*****************************************************************************/
void BSD_PostProcess(const BSDInReq_st* reqPorts, const BSDParam_st* params,
                     BSDOutPro_st* proPorts, BSDDebug_t* debugInfo) {
  // set output to BSD global
  BSD_Globals_t* pBSDOutputGlobals = &proPorts->BSD_Globals;

  const BSD_Globals_t* const pBSDGlobals = pGetBSDCalculatePointer_BSDGlobals();
  const BSDCalculate_st* const pCal_BSDObj = pGetBSDCalculatePointer();
  // const BSD_Info_Array* const pBSDObjList =
  //     &pGetBSDCalculatePointer()->BSDObjInfoList;
  const BSDZone_ObjPar_Array* const pBSDObjZoneArr =
      &pGetBSDCalculatePointer()->BSDZoneObjParList;

  // set global data and obj data to output
  memcpy(pBSDOutputGlobals, pBSDGlobals,  //NOLINT
         sizeof(BSD_Globals_t));

  pBSDOutputGlobals->bBSDWarnActive =
      (pBSDGlobals->bBSDWarnActiveLeft || pBSDGlobals->bBSDWarnActiveRight);
  memcpy(proPorts->BSDZoneObjParList,  //NOLINT
         pBSDObjZoneArr, sizeof(BSDZone_ObjPar_Array));
  // set Debug output

  memcpy(&debugInfo->BSDObjInfo,  //NOLINT
         &pCal_BSDObj->BSDObjInfoList, sizeof(BSD_Info_Array));
  debugInfo->uiVersionNumber = LBS_BSD_VERSION_NUMBER;
}
