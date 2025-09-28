/*****************************************************************************
        INCLUDES
*****************************************************************************/
#include "lbs_ose.h"

/*****************************************************************************
        VARIABLES
*****************************************************************************/
OSEGlobal_t OSEGlobal;

/*****************************************************************************
  Functionname:                                          */ /*!

  @brief:

  @description:

  @param[in] void
  @param[out] void

  @return
*****************************************************************************/
void LBS_OSE_Exec(const OSEInReq_t* reqPorts, const OSEParam_t* params,
                  OSEOutPro_t* proPorts, OSEDebug_t* debug) {
  if (FALSE == reqPorts->OSEFunctionSwitch.bOSEFunctionActive) {
    proPorts->bOSEWarnActive[0] = FALSE;
    proPorts->bOSEWarnActive[1] = FALSE;
    proPorts->bOSEWarnActive[2] = FALSE;
    proPorts->bWarningInterrupt = TRUE;
    proPorts->bCriticalObsIsRight = FALSE;
    proPorts->fCriticalTTC = 1000;
    proPorts->uCriticalObjID = 0;
    proPorts->fCriticalObjDistX = 1000;
    for (uint8 uObj = 0U; uObj < OSE_LBS_NUM_OBJECTS; uObj++) {
      proPorts->OSEObjInfoArray[uObj].InfoLevel[0].bWarning = FALSE;
      proPorts->OSEObjInfoArray[uObj].InfoLevel[1].bWarning = FALSE;
      proPorts->OSEObjInfoArray[uObj].InfoLevel[2].bWarning = FALSE;
    }
    return;
  }
  OSERunStateGlobal_t eOSEState = OSEGlobal.eOSEState;

  switch (eOSEState) {
    case OSE_OK:
      if (TRUE == reqPorts->OSEFunctionSwitch.bOSEFunctionActive) {
        OSE_PreProcess(reqPorts, params, &OSEGlobal);
        OSE_MainProcess(reqPorts, params, &OSEGlobal);
        OSE_PostProcess(proPorts, debug, &OSEGlobal);
      } else {
        LBS_OSE_Reset();
      }
      break;
    case OSE_Init:
      LBS_OSE_Reset();
      OSEGlobal.eOSEState = OSE_OK;
      break;
    default:
      LBS_OSE_Reset();
      OSEGlobal.eOSEState = OSE_OK;
      break;
  }
}

/*****************************************************************************
  Functionname: LBS_OSE_Reset                                         */ /*!

  @brief: Init of OSE signals

  @description: Init of OSE signals

  @param[in] void
  @param[out] void

  @return
*****************************************************************************/
void LBS_OSE_Reset() {
  OSEInitGlobals();
  OSEInitObjects();
  OSEGlobal.eOSEState = OSE_Init;
  OSEProPorts.bOSEWarnActive[0] = FALSE;
  OSEProPorts.bOSEWarnActive[1] = FALSE;
  OSEProPorts.bOSEWarnActive[2] = FALSE;
  OSEProPorts.fCriticalTTC = 1000;
  OSEProPorts.uCriticalObjID = 0;
}

/*****************************************************************************
  Functionname: LBS_OSE_Reset                                         */ /*!

  @brief: Init of OSE global data

  @description: Init of OSE global data

  @param[in] void
  @param[out] void

  @return
*****************************************************************************/
void OSEInitGlobals() {
  OSEGlobal.MultiObj.fCriticalTTC = TUE_C_F32_VALUE_INVALID;
  OSEGlobal.MultiObj.fCriticalObjDistX = TUE_C_F32_VALUE_INVALID;
  OSEGlobal.MultiObj.fCriticalObjDistXLastCycle = TUE_C_F32_VALUE_INVALID;
  OSEGlobal.MultiObj.uCriticalObjID = TUE_C_UI8_VALUE_INVALID;
  OSEGlobal.MultiObj.uCriticalObjIDLastCycle = TUE_C_UI8_VALUE_INVALID;
  OSEGlobal.MultiObj.bWarningInterrupt = FALSE;
  OSEGlobal.MultiObj.uInterruptCycleCount = 0;
  // OSEGlobal.SelDrvHyp = LBS_OSE_HYP_UNKNOWN;

  for (uint8 uWarnLevel = 0U; uWarnLevel < OSE_NUM_OF_WARN_LEVELS;
       uWarnLevel++) {
    OSEGlobal.bOSEWarnActive[uWarnLevel] = FALSE;
    OSEGlobal.ParameterLevel[uWarnLevel].fYMaxBreakthrough[0U] =
        TUE_C_F32_VALUE_INVALID;
    OSEGlobal.ParameterLevel[uWarnLevel].fYMaxBreakthrough[1U] =
        TUE_C_F32_VALUE_INVALID;
    OSEGlobal.ParameterLevel[uWarnLevel].fYMinBreakthrough[0U] =
        TUE_C_F32_VALUE_INVALID;
    OSEGlobal.ParameterLevel[uWarnLevel].fYMinBreakthrough[1U] =
        TUE_C_F32_VALUE_INVALID;
    // OSEGlobal.WarningTimer[uWarnLevel] = 0.0F;
  }
}

/*****************************************************************************
  Functionname: LBS_OSE_Reset                                         */ /*!

  @brief: Init of object data

  @description: Init of object data

  @param[in] void
  @param[out] void

  @return
*****************************************************************************/
void OSEInitObjects() {
  for (uint8 uObj = 0U; uObj < OSE_LBS_NUM_OBJECTS; uObj++) {
    OSEGlobal.OSEObjInfoArray[uObj].fYBreakthrough[0U] =
        TUE_C_F32_VALUE_INVALID;
    OSEGlobal.OSEObjInfoArray[uObj].fYBreakthrough[1U] =
        TUE_C_F32_VALUE_INVALID;
    OSEGlobal.OSEObjInfoArray[uObj].fYBreakthroughStd[0U] =
        TUE_C_F32_VALUE_INVALID;
    OSEGlobal.OSEObjInfoArray[uObj].fYBreakthroughStd[1U] =
        TUE_C_F32_VALUE_INVALID;
    OSEGlobal.OSEObjInfoArray[uObj].fTTC_s[0U] = TUE_C_F32_VALUE_INVALID;
    OSEGlobal.OSEObjInfoArray[uObj].fTTC_s[1U] = TUE_C_F32_VALUE_INVALID;
    OSEGlobal.OSEObjInfoArray[uObj].fTTCFiltered_s[0U] =
        TUE_C_F32_VALUE_INVALID;
    OSEGlobal.OSEObjInfoArray[uObj].fTTCFiltered_s[1U] =
        TUE_C_F32_VALUE_INVALID;
    OSEGlobal.OSEObjInfoArray[uObj].fDistToCrossingLine_met[0U] =
        TUE_C_F32_VALUE_INVALID;
    OSEGlobal.OSEObjInfoArray[uObj].fDistToCrossingLine_met[1U] =
        TUE_C_F32_VALUE_INVALID;
    OSEGlobal.OSEObjInfoArray[uObj].fSideTrackProb = OSE_FRONTOBJ_INIT_PROB;
    OSEGlobal.OSEObjInfoArray[uObj].bRelevant = FALSE;
    OSEGlobal.OSEObjInfoArray[uObj].bMirror = FALSE;
    OSEGlobal.OSEObjInfoArray[uObj].bSideTrack = FALSE;
    OSEGlobal.OSEObjInfoArray[uObj].bObjectFromRear = FALSE;
    OSEGlobal.OSEObjInfoArray[uObj].bValidApproachAngle = FALSE;
    OSEGlobal.OSEObjInfoArray[uObj].bObjectAtEdgeFoV = FALSE;
    OSEGlobal.OSEObjInfoArray[uObj].bShortTTC = FALSE;
    OSEGlobal.OSEObjInfoArray[uObj].fQuality = 0.0F;
    OSEGlobal.OSEObjInfoArray[uObj].bUpdatedRecently = FALSE;
    OSEGlobal.OSEObjInfoArray[uObj].fEstWidth.uCounters[0] = 0U;
    OSEGlobal.OSEObjInfoArray[uObj].fEstWidth.uCounters[1] = 0U;
    OSEGlobal.OSEObjInfoArray[uObj].fEstWidth.uCounters[2] = 0U;
    OSEGlobal.OSEObjInfoArray[uObj].fEstWidth.fValue_met = 0.0F;

    for (uint8 uWarnLevel = 0U; uWarnLevel < OSE_NUM_OF_WARN_LEVELS;
         uWarnLevel++) {
      // Init warning level values
      OSEGlobal.OSEObjInfoArray[uObj].InfoLevel[uWarnLevel].fBTHitHystTimer =
          0.0F;
      OSEGlobal.OSEObjInfoArray[uObj]
          .InfoLevel[uWarnLevel]
          .uBreakthroughHitConfi[0U] = 0U;
      OSEGlobal.OSEObjInfoArray[uObj]
          .InfoLevel[uWarnLevel]
          .uBreakthroughHitConfi[1U] = 0U;
      OSEGlobal.OSEObjInfoArray[uObj].InfoLevel[uWarnLevel].bWarning = FALSE;
      OSEGlobal.OSEObjInfoArray[uObj].InfoLevel[uWarnLevel].bWarningLastCycle =
          FALSE;
      OSEGlobal.OSEObjInfoArray[uObj].InfoLevel[uWarnLevel].bObjectInRange =
          FALSE;
      OSEGlobal.OSEObjInfoArray[uObj]
          .InfoLevel[uWarnLevel]
          .bTTCBelowThresh[0U] = FALSE;
      OSEGlobal.OSEObjInfoArray[uObj]
          .InfoLevel[uWarnLevel]
          .bTTCBelowThresh[1U] = FALSE;
      OSEGlobal.OSEObjInfoArray[uObj].InfoLevel[uWarnLevel].bBTHitHystActive =
          FALSE;
      OSEGlobal.OSEObjInfoArray[uObj]
          .InfoLevel[uWarnLevel]
          .bBreakthroughHit[0U] = FALSE;
      OSEGlobal.OSEObjInfoArray[uObj]
          .InfoLevel[uWarnLevel]
          .bBreakthroughHit[1U] = FALSE;
    }
  }
}

/*****************************************************************************
  Functionname: OSE_PreProcess                                          */ /*!

  @brief: The PreProcess of OSE function

  @description: The PreProcess of OSE function

  @param[in ] reqPorts       the inputs structure of OSE function
              params         the parameter structure of OSE function
			  pOSEGlobal     OSE global variable structure
  @param[out] void

  @return
*****************************************************************************/
void OSE_PreProcess(const OSEInReq_t* reqPorts, const OSEParam_t* params,
                    OSEGlobal_t* pOSEGlobal) {
  // Calculate left and right sensor offset to side
  LBSOSESetParameters(params, &pOSEGlobal->fLeftSensorOffsetToSide_met,
                      &pOSEGlobal->fRightSensorOffsetToSide_met,
                      &pOSEGlobal->fLeftSensorOffsetToRear_met,
                      &pOSEGlobal->fRightSensorOffsetToRear_met);
  // Init all cyclic variables
  LBSOSEInitCyclic(pOSEGlobal);
  // Check current driving condition: unknown   parking   driving
  // LBSOSECheckDrivingConditions();
}

/*****************************************************************************
  Functionname: OSE_MainProcess                                        */ /*!

  @brief: OSE main function 

  @description: OSE condition check and warning decision 

  @param[in]  reqPorts        Input structure of OSE function
              params          Parameter structure of OSE function
  @param[out] pOSEGlobal      OSE global variable structure

  @return
*****************************************************************************/
void OSE_MainProcess(const OSEInReq_t* reqPorts, const OSEParam_t* params,
                     OSEGlobal_t* pOSEGlobal) {
  // Perform same operations on all existing objects
  for (uint8 uObj = 0U; uObj < OSE_LBS_NUM_OBJECTS; uObj++) {
    const OSEGenObjectInReq_t* pEMGenObjInReq =
        &reqPorts->EMGenObjList.aObject[uObj];
    OSEObjInfoArrayGlobal_t* pOSEObjGlobal = &pOSEGlobal->OSEObjInfoArray[uObj];
    const OSELBSObjInfoArrayInReq_t* pOSELBSObjInReq =
        &reqPorts->OSELBSGlobalInReq.OSELBSObjInfoArray[uObj];

    if (reqPorts->EMGenObjList.aObject[uObj].eMaintenanceState !=
        EM_GEN_OBJ_MT_STATE_DELETED) {
      // Calculate the limit value of YBreakthrough for OSE function
      LBSOSECalculateYBreakthroughLimit(
          reqPorts->EMGenObjList.aObject[uObj].bRightSensor,
          pOSEGlobal->fLeftSensorOffsetToSide_met,
          pOSEGlobal->fRightSensorOffsetToSide_met, params,
          pOSEGlobal->ParameterLevel);
      // Calculate distance to crossing line
      LBSOSECalculateDistToCrossingLine(
          pEMGenObjInReq->bRightSensor, pEMGenObjInReq->fDistX_met,
          pOSEGlobal->fLeftSensorOffsetToRear_met,
          pOSEGlobal->fRightSensorOffsetToRear_met,
          params->fXBreakthroughLine_met[0U], pEMGenObjInReq->fLengthFront_met,
          &pOSEObjGlobal->fDistToCrossingLine_met[0U]);
      LBSOSECalculateDistToCrossingLine(
          pEMGenObjInReq->bRightSensor, pEMGenObjInReq->fDistX_met,
          pOSEGlobal->fLeftSensorOffsetToRear_met,
          pOSEGlobal->fRightSensorOffsetToRear_met,
          params->fXBreakthroughLine_met[1U], pEMGenObjInReq->fLengthFront_met,
          &pOSEObjGlobal->fDistToCrossingLine_met[1U]);

// Calculate the additional margin depending on the object heading angle
// LBSOSECalculateObjBreakthroughMargin();
// LBSOSECalculateObjBreakthroughMargin();
// Estimate the width of the object based in RCS
#ifdef ST_PERCEPTION
      LBSOSEEstimate_Width(pEMGenObjInReq->fWidth_met, pEMGenObjInReq->fRCS,
                           pOSEObjGlobal->fEstWidth.uCounters,
                           &pOSEObjGlobal->fEstWidth.fValue_met);
#endif
      // Calculate x-axis breakthrough
      LBSOSECalculateYBreakthrough(pEMGenObjInReq, pOSEObjGlobal, 0U);
      LBSOSECalculateYBreakthrough(pEMGenObjInReq, pOSEObjGlobal, 1U);
      // Calculate time to crossing
#ifdef ST_PERCEPTION

      LBSOSECalculateTTC(reqPorts->fCycletime_s, pEMGenObjInReq->fVrelX_mps,
                         pOSEObjGlobal->fDistToCrossingLine_met[0U],
                         &pOSEObjGlobal->fTTC_s[0U],
                         &pOSEObjGlobal->fTTCFiltered_s[0U]);
      LBSOSECalculateTTC(reqPorts->fCycletime_s, pEMGenObjInReq->fVrelX_mps,
                         pOSEObjGlobal->fDistToCrossingLine_met[1U],
                         &pOSEObjGlobal->fTTC_s[1U],
                         &pOSEObjGlobal->fTTCFiltered_s[1U]);
#endif

      LBSOSECalculateTTC(reqPorts->fCycletime_s, pEMGenObjInReq->fDistX_met,
                         pEMGenObjInReq->fVrelX_mps,
                         pOSEObjGlobal->fDistToCrossingLine_met[0U],
                         &pOSEObjGlobal->fTTC_s[0U],
                         &pOSEObjGlobal->fTTCFiltered_s[0U]);
      LBSOSECalculateTTC(reqPorts->fCycletime_s, pEMGenObjInReq->fDistX_met,
                         pEMGenObjInReq->fVrelX_mps,
                         pOSEObjGlobal->fDistToCrossingLine_met[1U],
                         &pOSEObjGlobal->fTTC_s[1U],
                         &pOSEObjGlobal->fTTCFiltered_s[1U]);
      // Calculate the side object probability
      LBSOSECalculateFrontObjectProbability(uObj, reqPorts, pOSEGlobal,
                                            &pOSEObjGlobal->fSideTrackProb);
      // Check whether this object is a side object
      LBSOSECheckSideObject(pOSEObjGlobal->fSideTrackProb,
                            &pOSEObjGlobal->bSideTrack);
      // Check whether the object was updated in the last cycles
      LBSOSECheckObjectUpdateRecently(
          pEMGenObjInReq->uiMeasuredTargetFrequency_nu,
          &pOSEObjGlobal->bUpdatedRecently);
      // Check if object is in range
      LBSOSECheckObjectInRange(
          pEMGenObjInReq->fDistX_met, pEMGenObjInReq->fDistY_met,
          params->fXMaxTargetRange_met, params->fXMinTargetRange_met,
          pOSEObjGlobal->InfoLevel);  // *product
      // Check if target angle of approach is in range
      LBSOSECheckRearApproach(pEMGenObjInReq->fDistX_met,
                              pEMGenObjInReq->fFirstDetectX_met,
                              &pOSEObjGlobal->bObjectFromRear);
      // Check angle of approach is in range
      LBSOSECheckApproachAngle(
          pEMGenObjInReq->fAbsOrientation_rad, pOSEObjGlobal->bObjectFromRear,
          params->fMaxHeadingAngle, params->fMinHeadingAngle,
          &pOSEObjGlobal->bValidApproachAngle);
      // Check if an object which enables a warning is a possible mirror
      LBSOSECheckForMirror(pEMGenObjInReq->fMirrorProb_per,
                           &pOSEObjGlobal->bMirror);
      // Check whether object is relevant
      LBSOSECheckObjectRelevance(
          pOSELBSObjInReq->fXMovement_met, pOSELBSObjInReq->fYMovement_met,
          pEMGenObjInReq->fVabsX_mps, pEMGenObjInReq->fVabsY_mps,
          pEMGenObjInReq->uiLifeCycles_nu, params->fVTargetMin_mps,
          &pOSEObjGlobal->bRelevant);
      // Calculate whether the object has the necessary quality
      LBSOSECheckObjectQuality(pEMGenObjInReq, pOSELBSObjInReq->fUpdateRate_nu,
                               pOSELBSObjInReq->fAssocProbFiltered,
                               &pOSEObjGlobal->fQuality);
      // Check whether the object hits the breakthrough line
      LBSOSECheckBreakthroughHit(0U, pOSEObjGlobal->fDistToCrossingLine_met,
                                 pOSEObjGlobal->fYBreakthrough,
                                 pOSEGlobal->ParameterLevel,
                                 pOSEObjGlobal->InfoLevel);
      LBSOSECheckBreakthroughHit(1U, pOSEObjGlobal->fDistToCrossingLine_met,
                                 pOSEObjGlobal->fYBreakthrough,
                                 pOSEGlobal->ParameterLevel,
                                 pOSEObjGlobal->InfoLevel);
      // Update the confidence of the breakthrough hit
      LBSOSECalculateBreakthroughHitConfidence(
          0U, pOSEObjGlobal->fYBreakthrough, pOSEObjGlobal->fYBreakthroughStd,
          pOSEObjGlobal->bObjectAtEdgeFoV, pOSEGlobal->ParameterLevel,
          pOSEObjGlobal->InfoLevel);
      LBSOSECalculateBreakthroughHitConfidence(
          1U, pOSEObjGlobal->fYBreakthrough, pOSEObjGlobal->fYBreakthroughStd,
          pOSEObjGlobal->bObjectAtEdgeFoV, pOSEGlobal->ParameterLevel,
          pOSEObjGlobal->InfoLevel);
      // Check the hysteresis timer(smoothing of warning ON/OFF)
      LBSOSEUpdateBTHitHysteresisTimer(
          reqPorts->fCycletime_s, pOSEObjGlobal->fTTC_s[0U],
          pOSEObjGlobal->fTTCFiltered_s[0U], pOSEObjGlobal->InfoLevel);
      // Check the hysteresis timer condition (unstable object trajectories do
      // not result in unstable warnings)
      LBSOSECheckBTHitHysteresis(pOSEObjGlobal->InfoLevel);
      // Check the TTC condition
      LBSOSECheckTTC(0U, pOSEObjGlobal->fTTC_s[0U],
                     pOSEObjGlobal->fTTCFiltered_s[0U],
                     pOSEObjGlobal->bObjectAtEdgeFoV, params->fTTCThreshold_s,
                     pOSEObjGlobal->InfoLevel);
      LBSOSECheckTTC(1U, pOSEObjGlobal->fTTC_s[1U],
                     pOSEObjGlobal->fTTCFiltered_s[1U],
                     pOSEObjGlobal->bObjectAtEdgeFoV, params->fTTCThreshold_s,
                     pOSEObjGlobal->InfoLevel);
      // Check the short warning condition
      LBSOSECheckShortWarning(pOSEObjGlobal->fTTC_s[0U],
                              pOSEObjGlobal->fTTCFiltered_s[0U],
                              &pOSEObjGlobal->bShortTTC);
#ifdef ST_PERCEPTION
      // Decide whether the object shall warn
      LBSOSEWarningDecision(pEMGenObjInReq->fRCS, pOSEObjGlobal,
                            pOSEObjGlobal->InfoLevel);
#endif
      // Decide whether the object shall warn
      LBSOSEWarningDecision(pOSEObjGlobal, pOSEObjGlobal->InfoLevel);
      // Set the global warning
      LBSOSESetGlobalWarning(uObj, pEMGenObjInReq->fDistX_met,
                             pEMGenObjInReq->fLengthRear_met,
                             pOSEObjGlobal->fTTCFiltered_s[0U], pOSEObjGlobal,
                             &pOSEGlobal->MultiObj, pOSEGlobal->bOSEWarnActive);
    } else {
      OSEGlobal.OSEObjInfoArray[uObj].fYBreakthrough[0U] =
          TUE_C_F32_VALUE_INVALID;
      OSEGlobal.OSEObjInfoArray[uObj].fYBreakthrough[1U] =
          TUE_C_F32_VALUE_INVALID;
      OSEGlobal.OSEObjInfoArray[uObj].fYBreakthroughStd[0U] =
          TUE_C_F32_VALUE_INVALID;
      OSEGlobal.OSEObjInfoArray[uObj].fYBreakthroughStd[1U] =
          TUE_C_F32_VALUE_INVALID;
      OSEGlobal.OSEObjInfoArray[uObj].fTTC_s[0U] = TUE_C_F32_VALUE_INVALID;
      OSEGlobal.OSEObjInfoArray[uObj].fTTC_s[1U] = TUE_C_F32_VALUE_INVALID;
      OSEGlobal.OSEObjInfoArray[uObj].fTTCFiltered_s[0U] =
          TUE_C_F32_VALUE_INVALID;
      OSEGlobal.OSEObjInfoArray[uObj].fTTCFiltered_s[1U] =
          TUE_C_F32_VALUE_INVALID;
      OSEGlobal.OSEObjInfoArray[uObj].fDistToCrossingLine_met[0U] =
          TUE_C_F32_VALUE_INVALID;
      OSEGlobal.OSEObjInfoArray[uObj].fDistToCrossingLine_met[1U] =
          TUE_C_F32_VALUE_INVALID;
      OSEGlobal.OSEObjInfoArray[uObj].fSideTrackProb = OSE_FRONTOBJ_INIT_PROB;
      OSEGlobal.OSEObjInfoArray[uObj].bRelevant = FALSE;
      OSEGlobal.OSEObjInfoArray[uObj].bMirror = FALSE;
      OSEGlobal.OSEObjInfoArray[uObj].bSideTrack = FALSE;
      OSEGlobal.OSEObjInfoArray[uObj].bObjectFromRear = FALSE;
      OSEGlobal.OSEObjInfoArray[uObj].bValidApproachAngle = FALSE;
      OSEGlobal.OSEObjInfoArray[uObj].bObjectAtEdgeFoV = FALSE;
      OSEGlobal.OSEObjInfoArray[uObj].bShortTTC = FALSE;
      OSEGlobal.OSEObjInfoArray[uObj].fQuality = 0.0F;
      OSEGlobal.OSEObjInfoArray[uObj].bUpdatedRecently = FALSE;
      OSEGlobal.OSEObjInfoArray[uObj].fEstWidth.uCounters[0] = 0U;
      OSEGlobal.OSEObjInfoArray[uObj].fEstWidth.uCounters[1] = 0U;
      OSEGlobal.OSEObjInfoArray[uObj].fEstWidth.uCounters[2] = 0U;
      OSEGlobal.OSEObjInfoArray[uObj].fEstWidth.fValue_met = 0.0F;

      for (uint8 uWarnLevel = 0U; uWarnLevel < OSE_NUM_OF_WARN_LEVELS;
           uWarnLevel++) {
        // Init warning level values
        OSEGlobal.OSEObjInfoArray[uObj].InfoLevel[uWarnLevel].fBTHitHystTimer =
            0.0F;
        OSEGlobal.OSEObjInfoArray[uObj]
            .InfoLevel[uWarnLevel]
            .uBreakthroughHitConfi[0U] = 0U;
        OSEGlobal.OSEObjInfoArray[uObj]
            .InfoLevel[uWarnLevel]
            .uBreakthroughHitConfi[1U] = 0U;
        OSEGlobal.OSEObjInfoArray[uObj].InfoLevel[uWarnLevel].bWarning = FALSE;
        OSEGlobal.OSEObjInfoArray[uObj]
            .InfoLevel[uWarnLevel]
            .bWarningLastCycle = FALSE;
        OSEGlobal.OSEObjInfoArray[uObj].InfoLevel[uWarnLevel].bObjectInRange =
            FALSE;
        OSEGlobal.OSEObjInfoArray[uObj]
            .InfoLevel[uWarnLevel]
            .bTTCBelowThresh[0U] = FALSE;
        OSEGlobal.OSEObjInfoArray[uObj]
            .InfoLevel[uWarnLevel]
            .bTTCBelowThresh[1U] = FALSE;
        OSEGlobal.OSEObjInfoArray[uObj].InfoLevel[uWarnLevel].bBTHitHystActive =
            FALSE;
        OSEGlobal.OSEObjInfoArray[uObj]
            .InfoLevel[uWarnLevel]
            .bBreakthroughHit[0U] = FALSE;
        OSEGlobal.OSEObjInfoArray[uObj]
            .InfoLevel[uWarnLevel]
            .bBreakthroughHit[1U] = FALSE;
      }
    }
  }
  // Handle multi targets
  LBSOSECheckMultiObjectInterrupt(&reqPorts->EMGenObjList.aObject,
                                  &pOSEGlobal->MultiObj);
}

/*****************************************************************************
  Functionname: OSE_PostProcess                                        */ /*!

  @brief: Process the warnings for OSE function

  @description: Process the warnings for OSE function

  @param[in] 
  @param[out] 

  @return
*****************************************************************************/
void OSE_PostProcess(OSEOutPro_t* proPorts, OSEDebug_t* debug,
                     OSEGlobal_t* pOSEGlobal) {
  if (sizeof(OSEObjInfoArrayOutPro_t) != sizeof(OSEObjInfoArrayGlobal_t)) {
    DEBUG_Print("OSE OSE_PostProcess Error!");
    return;
  }
  memcpy(proPorts->OSEObjInfoArray, pOSEGlobal->OSEObjInfoArray,  //NOLINT
         sizeof(OSEObjInfoArrayGlobal_t) * OSE_LBS_NUM_OBJECTS);
  for (uint8 uWarnLevel = 0U; uWarnLevel < OSE_NUM_OF_WARN_LEVELS;
       uWarnLevel++) {
    proPorts->bOSEWarnActive[uWarnLevel] =
        pOSEGlobal->bOSEWarnActive[uWarnLevel];
  }
  proPorts->bWarningInterrupt = pOSEGlobal->MultiObj.bWarningInterrupt;
  proPorts->uCriticalObjID = pOSEGlobal->MultiObj.uCriticalObjID;
  proPorts->fCriticalObjDistX = pOSEGlobal->MultiObj.fCriticalObjDistX;
  proPorts->fCriticalTTC = pOSEGlobal->MultiObj.fCriticalTTC;
}
