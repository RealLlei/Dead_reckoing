#include "cta_rcta.h"

#include "planning/warning/lbs/common/tue_common_libs.h"

RCTAGlobal_t RCTAGlobal;

/*****************************************************************************
  Functionname:RCTAReset                                     */ /*!

  @brief Init RCTA data

  @description Init of RCTA data

  @param[in]

  @param[out]

  @return
*****************************************************************************/
void RCTAReset() {
  // init of global data
  for (uint8 uWarnLevel = 0U; uWarnLevel < CTA_RCTA_CFG_NUM_OF_WARN_LEVELS;
       uWarnLevel++) {
    RCTAGlobal.bRCTAWarnActive[uWarnLevel] = FALSE;
    RCTAGlobal.fMaxObjRange_met[uWarnLevel] = TUE_C_F32_VALUE_INVALID;
    RCTAGlobal.fTTCThreshold_s[uWarnLevel] = TUE_C_F32_VALUE_INVALID;
    RCTAGlobal.fXMaxBreakthrough_met[uWarnLevel] = TUE_C_F32_VALUE_INVALID;
    RCTAGlobal.fXMinBreakthrough_met[uWarnLevel] = TUE_C_F32_VALUE_INVALID;
  }
  RCTAGlobal.fMaxHeadingAngle_deg = TUE_C_F32_VALUE_INVALID;
  RCTAGlobal.fMinHeadingAngle_deg = TUE_C_F32_VALUE_INVALID;
  RCTAGlobal.fCriticalTTC_s = TUE_C_F32_VALUE_INVALID;
  RCTAGlobal.fCriticalObjDistY_met = TUE_C_F32_VALUE_INVALID;
  RCTAGlobal.fCriticalObjDistYLastCycle_met = TUE_C_F32_VALUE_INVALID;
  RCTAGlobal.uCriticalObjID_nu = TUE_C_UI8_VALUE_INVALID;
  RCTAGlobal.uCriticalObjIDLastCycle_nu = TUE_C_UI8_VALUE_INVALID;
  RCTAGlobal.bWarningInterrupt = FALSE;
  RCTAGlobal.uInterruptCycleCount_nu = 0U;

  // init all objects data
  for (uint8 uObj = 0U; uObj < CTA_RCTA_CFG_NUM_OF_WARN_LEVELS; uObj++) {
    RCTAGlobal.RCTA_Va_ObjectListGlobal[uObj].bRelevant = FALSE;
    RCTAGlobal.RCTA_Va_ObjectListGlobal[uObj].bMirror = FALSE;
    RCTAGlobal.RCTA_Va_ObjectListGlobal[uObj].bObjectFromSide = FALSE;
    RCTAGlobal.RCTA_Va_ObjectListGlobal[uObj].bValidApproachAngle = FALSE;
    RCTAGlobal.RCTA_Va_ObjectListGlobal[uObj].bShortTTC = FALSE;
    RCTAGlobal.RCTA_Va_ObjectListGlobal[uObj].bQuality = FALSE;
    RCTAGlobal.RCTA_Va_ObjectListGlobal[uObj].bUpdatedRecently = FALSE;

    // CT global initialize
  }
}

/*****************************************************************************
  Functionname:                                     */ /*!

  @brief

  @description

  @param[in]

  @param[out]

  @return
*****************************************************************************/
void RCTAExec(const RCTAInReq_t* RCTAreqPorts, const RCTAParam_t* RCTAparams,
              RCTAOutPro_t* RCTAproPorts, RCTADebug_t* RCTAdebugInfo) {
  if (RCTAreqPorts->bRCTAFunctionActive) {
    RCTAPreProcess(RCTAreqPorts, RCTAparams, &RCTAGlobal);
    RCTAMainProcess(RCTAreqPorts, RCTAparams, &RCTAGlobal);
    RCTAProProcess(&RCTAGlobal, RCTAproPorts, RCTAdebugInfo);
  } else {
    if (RCTAreqPorts->LastCycleStates.bRCTAFunctionActive) {
      RCTAReset();
    }
  }
}

/*****************************************************************************
  Functionname:RCTAPreProcess                                     */ /*!

  @brief

  @description

  @param[in]

  @param[out]

  @return
*****************************************************************************/
void RCTAPreProcess(const RCTAInReq_t* RCTAreqPorts,
                    const RCTAParam_t* RCTAparams, RCTAGlobal_t* RCTAGlobal) {
  // set all parameters for RCTA function depending on ego speed and BSW
  // parameters
  CTARCTASetParameters(&RCTAreqPorts->RCTAVehicleSig,
                       &RCTAparams->RCTAAlgoParam, RCTAGlobal);
  // Init all cyclic variables
  CTARCTAInitCyclic(RCTAGlobal->bRCTAWarnActive);
}

/*****************************************************************************
  Functionname:CTARCTASetParameters                                     */ /*!

  @brief set necessary algorithm parameters for RCTA decision

  @description set necessary algorithm parameters for RCTA decision, differentiate between stationary (ego vehicle not moving) and
		       dynamic (ego vehicle moving backwards). Use BSW algorithm parameters by default, if not available use hard coded defines,
		       so keep defines up to date

  @param[in] RCTAVehicleSig: RCTA vehicle signals
             RCTAAlgoParam: RCTA algorithm parameters

  @param[out] RCTAGlobal: RCTA global variable structure

  @return
*****************************************************************************/
void CTARCTASetParameters(const RCTAVehicleSignal_t* RCTAVehicleSig,
                          const RCTAAlgoParam_t* RCTAAlgoParam,
                          RCTAGlobal_t* RCTAGlobal) {
  float32 fTTCThreshold = 0.F;
  float32 fTargetRangeMax = 0.F;
  float32 fXMaxBreakthrough = 0.F;
  float32 fXMinBreakthrough = 0.F;

  // set level dependent parameters
  for (uint8 uWarnLevel = 0; uWarnLevel < CTA_RCTA_CFG_NUM_OF_WARN_LEVELS;
       uWarnLevel++) {
    switch (uWarnLevel) {
      case RCTA_WARN_LEVEL_ONE:
        fTTCThreshold = RCTAAlgoParam->fTTCThreshold_s +
                        RCTAAlgoParam->fTTCThresholdMargin_s;
        fTargetRangeMax = RCTAAlgoParam->fTargetRangeMax_met;
        fXMaxBreakthrough = RCTAAlgoParam->fXMaxBreakthrough_met;
        if (RCTAAlgoParam->SteeringAngleCutOff.bEnableSteeringAngleCutOff_nu) {
          // Adjust minimal breakthrough threshold based on steer angle
          fXMinBreakthrough = RCTACalculateBreakthroughLength(
              RCTAVehicleSig->StWheelAngle_rad, RCTAAlgoParam);
        } else {
          // Use the breakthrough value from algorithm parameter
          fXMinBreakthrough = RCTAAlgoParam->fXMinBreakthrough_met;
        }
        break;
      case RCTA_WARN_LEVEL_TWO:
        fTTCThreshold = RCTAAlgoParam->fTTCThresholdL2_s +
                        RCTAAlgoParam->fTTCThresholdMargin_s;
        fTargetRangeMax = RCTAAlgoParam->fTargetRangeMaxL2_met;
        fXMaxBreakthrough = RCTAAlgoParam->fXMaxBreakthrough_met;
        fXMinBreakthrough = RCTAAlgoParam->fXMinBreakthroughL2_met;
        break;
      default:
        break;
    }
    // set TTC threshold
    RCTAGlobal->fTTCThreshold_s[uWarnLevel] = fTTCThreshold;
    // set max object range
    RCTAGlobal->fMaxObjRange_met[uWarnLevel] = fTargetRangeMax;
    // set min and max XBreakthrough
    RCTAGlobal->fXMaxBreakthrough_met[uWarnLevel] = fXMaxBreakthrough;
    RCTAGlobal->fXMinBreakthrough_met[uWarnLevel] = fXMinBreakthrough;
  }

  // set common parameter
  RCTAGlobal->fMaxHeadingAngle_deg =
      RCTAAlgoParam->fMaxHeadingAngle + RCTA_APPROACH_ANGLE_MARGIN;  //-30 + 5
  RCTAGlobal->fMinHeadingAngle_deg =
      RCTAAlgoParam->fMinHeadingAngle - RCTA_APPROACH_ANGLE_MARGIN;  //-160 - 5
}

/*****************************************************************************
  Functionname:RCTACalculateBreakthroughLength */ /*!

  @brief Adapt the warning length parameter based on steering wheel angle

  @description Adapt the warning length parameter based on steering wheel angle

  @param[in]  StWheelAngle_rad: wheel steering angle
              RCTAAlgoParam: RCTA algorithm parameter structure

  @param[out] 

  @return fMinBreakthrough: min breakthrough value of RCTA level1 warning 
*****************************************************************************/
float32 RCTACalculateBreakthroughLength(float32 StWheelAngle_rad,
                                        const RCTAAlgoParam_t* RCTAAlgoParam) {
  float32 fMinBreakthrough = 0.F;
  const float32 fSteerAngleAbs = fABS(RAD2DEG(StWheelAngle_rad));

  // Check whether the steering angle is smaller than the lower threshold
  if (fSteerAngleAbs <
      RCTAAlgoParam->SteeringAngleCutOff.fSteerAngleCutOffMin_deg) {
    // In this case use the value from algorithm parameter
    fMinBreakthrough = RCTAAlgoParam->fXMinBreakthrough_met;
  } else if (fSteerAngleAbs <
             RCTAAlgoParam->SteeringAngleCutOff.fSteerAngleCutOffMid_deg) {
    // Linear interpolate between the min and mid value
    fMinBreakthrough = TUE_CML_BoundedLinInterpol2(
        fSteerAngleAbs,
        RCTAAlgoParam->SteeringAngleCutOff.fSteerAngleCutOffMin_deg,
        RCTAAlgoParam->SteeringAngleCutOff.fSteerAngleCutOffMid_deg,
        RCTAAlgoParam->SteeringAngleCutOff.fXMinBreakthroughSWAMin_met,
        RCTAAlgoParam->SteeringAngleCutOff.fXMinBreakthroughSWAMid_met);
  } else {
    // Linear interpolate between the mid and max value
    fMinBreakthrough = TUE_CML_BoundedLinInterpol2(
        fSteerAngleAbs,
        RCTAAlgoParam->SteeringAngleCutOff.fSteerAngleCutOffMid_deg,
        RCTAAlgoParam->SteeringAngleCutOff.fSteerAngleCutOffMax_deg,
        RCTAAlgoParam->SteeringAngleCutOff.fXMinBreakthroughSWAMid_met,
        RCTAAlgoParam->SteeringAngleCutOff.fXMinBreakthroughSWAMax_met);
  }
  return fMinBreakthrough;
}

/*****************************************************************************
  Functionname:CTARCTAInitCyclic                                    */ /*!

  @brief RCTA cyclic initialization

  @description Initialize variables which are not stored across cycles

  @param[in]  pbRCTAWarnActive�� Flag whether three waning levels are activated

  @param[out] pbRCTAWarnActive�� Flag whether three waning levels are activated

  @return
*****************************************************************************/
void CTARCTAInitCyclic(boolean* pbRCTAWarnActive) {
  for (uint8 uWarnLevel = 0; uWarnLevel < CTA_RCTA_CFG_NUM_OF_WARN_LEVELS;
       uWarnLevel++) {
    pbRCTAWarnActive[uWarnLevel] = FALSE;
  }
  RCTAGlobal.uCriticalObjIDLastCycle_nu = RCTAGlobal.uCriticalObjID_nu;
  RCTAGlobal.uCriticalObjID_nu = TUE_C_UI8_VALUE_INVALID;
  RCTAGlobal.fCriticalObjDistYLastCycle_met = RCTAGlobal.fCriticalObjDistY_met;
  RCTAGlobal.fCriticalObjDistY_met = TUE_C_F32_VALUE_INVALID;
  RCTAGlobal.fCriticalTTC_s = TUE_C_F32_VALUE_INVALID;
}

/*****************************************************************************
  Functionname:                                     */ /*!

  @brief

  @description

  @param[in]

  @param[out]

  @return
*****************************************************************************/
void RCTAMainProcess(const RCTAInReq_t* RCTAreqPorts,
                     const RCTAParam_t* RCTAparams, RCTAGlobal_t* RCTAGlobal) {
  for (uint8 uObj = 0U; uObj < RCTA_MAX_NUM_OBJECTS; uObj++) {
    const RCTAEMSRRObjInReq_t* pEMSRRObjInput =
        &RCTAreqPorts->EMSRRObjListInput[uObj];
    RCTAObjGlobal_t* pRCTAObjGlobal =
        &RCTAGlobal->RCTA_Va_ObjectListGlobal[uObj];
    const RCTACTObjListInReq_t* pRCTACTObjInput =
        &RCTAreqPorts->CTAGlobleInput.RCTACTObjListInput[uObj];
    const RCTACTAObjListInReq_t* pRCTACTAObjInput =
        &RCTAreqPorts->CTAGlobleInput.RCTACTAObjListInput[uObj];

    if (pEMSRRObjInput->uiMaintenanceState_nu !=
        CTA_EM_GEN_OBJECT_MT_STATE_DELETED) {
      // Check whether the object was updated in the last cycles
      CTARCTACheckObjectUpdateRecently(
          pEMSRRObjInput->uiMeasuredTargetFrequency_nu,
          &pRCTAObjGlobal->bUpdatedRecently);
      // Check if object is in range
      CTARCTACheckObjectInRange(
          pEMSRRObjInput->fDistX_met, pEMSRRObjInput->fDistY_met,
          RCTAGlobal->fMaxObjRange_met, pRCTAObjGlobal->bObjectInRange);
      // Check if target angle of approach is in range
      CTARCTACheckSideApproach(
          pEMSRRObjInput->bRightSensor, pEMSRRObjInput->fFirstDetectY_met,
          pEMSRRObjInput->fDistY_met, &pRCTAObjGlobal->bObjectFromSide);
      // Check angle of approach is in range
      CTARCTACheckApproachAngle(
          pEMSRRObjInput->bRightSensor, pEMSRRObjInput->fRelHeading_rad,
          pEMSRRObjInput->fRelHeadingStd_rad, RCTAGlobal->fMaxHeadingAngle_deg,
          RCTAGlobal->fMinHeadingAngle_deg, pRCTAObjGlobal->bObjectFromSide,
          &pRCTAObjGlobal->bValidApproachAngle);
      // Check if an object which enables a warning is a possible mirror
      CTARCTACheckForMirror(pEMSRRObjInput->fMirrorProb_per,
                            &pRCTAObjGlobal->bMirror);
      // Check whether object is relevant
      CTARCTACheckObjectRelevance(
          pEMSRRObjInput->fVabsX_mps, pEMSRRObjInput->fVabsY_mps,
          RCTAparams->RCTAAlgoParam.fVTargetMin,
          pEMSRRObjInput->uiLifeCycles_nu, pRCTACTAObjInput->fXMovement_met,
          pRCTACTAObjInput->fYMovement_met, &pRCTAObjGlobal->bRelevant);
      // Calculate whether the object has the necessary quality
      CTARCTACheckObjectQuality(
          pEMSRRObjInput, pRCTACTAObjInput->fUpdateRate_nu,
          pRCTACTAObjInput->fAssocProbFiltered_nu, &pRCTAObjGlobal->bQuality);
      // Check whether the object hits the breakthrough line
      CTARCTACheckBreakthroughHit(
          RCTAGlobal->fXMaxBreakthrough_met, RCTAGlobal->fXMinBreakthrough_met,
          pRCTACTObjInput->fDistToCrossingLine_met,
          pRCTACTObjInput->fXBreakthrough_met, pRCTAObjGlobal->bWarning,
          pRCTAObjGlobal->bBreakthroughHit);
      // Update the confidence of the breakthrough hit
      CTARCTACalculateBreakthroughHitConfidence(
          pRCTACTObjInput->fXBreakthrough_met,
          pRCTACTObjInput->fXBreakthroughStd_met,
          RCTAGlobal->fXMaxBreakthrough_met, RCTAGlobal->fXMinBreakthrough_met,
          pRCTAObjGlobal->bBreakthroughHit,
          pRCTAObjGlobal->uBreakthroughHitConfi);
      // Check the hysteresis timer(smoothing of warning ON/OFF)
      CTARCTAUpdateBTHitHysteresisTimer(
          RCTAreqPorts->fCycleTime_s, pRCTACTObjInput->fTTC_s,
          pRCTACTObjInput->fTTCFiltered_s, pRCTAObjGlobal->bWarning,
          pRCTAObjGlobal->bBreakthroughHit, pRCTAObjGlobal->bWarningLastCycle,
          pRCTAObjGlobal->fBTHitHystTimer_s);
      // Check the hysteresis timer condition (unstable object trajectories do
      // not result in unstable warnings)
      CTARCTACheckBTHitHysteresisTimer(pRCTAObjGlobal->fBTHitHystTimer_s,
                                       pRCTAObjGlobal->bBTHitHystActive);
      // Check the TTC condition
      CTARCTACheckTTC(pRCTACTObjInput->fTTC_s, pRCTACTObjInput->fTTCFiltered_s,
                      RCTAGlobal->fTTCThreshold_s,
                      pRCTAObjGlobal->bTTCBelowThresh);
      // Check the short warning condition
      CTARCTACheckShortWarning(pRCTACTObjInput->fTTC_s,
                               pRCTACTObjInput->fTTCFiltered_s,
                               &pRCTAObjGlobal->bShortTTC);
      // Decide whether the object shall warn
#ifdef ST_PERCEPTION
      CTARCTAWarningDecision(
          pEMSRRObjInput->fRCS, pRCTAObjGlobal, pRCTACTObjInput->bRearTrack_nu,
          pRCTAObjGlobal->bWarningLastCycle, pRCTAObjGlobal->bWarning);
#endif  // ST_PERCEPTION

      CTARCTAWarningDecision(
          RCTAreqPorts->RCTAVehicleSig.uRCTAGearPosition, pRCTAObjGlobal,
          pRCTACTObjInput->bRearTrack_nu,
          RCTAreqPorts->RCTAVehicleSig.bRCTAEgoSixDoorsClosed,
          pRCTAObjGlobal->bWarning);
      // Set the global warning
      CTARCTASetGlobalWarning(uObj, pEMSRRObjInput->fDistY_met,
                              pEMSRRObjInput->fWidthLeft_met,
                              pRCTAObjGlobal->bWarning,
                              pRCTACTObjInput->fTTCFiltered_s, RCTAGlobal);
    } else {
      pRCTAObjGlobal->fBTHitHystTimer_s[0] = 0;
      pRCTAObjGlobal->fBTHitHystTimer_s[1] = 0;
      pRCTAObjGlobal->uBreakthroughHitConfi[0] = 0;
      pRCTAObjGlobal->uBreakthroughHitConfi[1] = 0;
      pRCTAObjGlobal->bBreakthroughHit[0] = 0;
      pRCTAObjGlobal->bBreakthroughHit[1] = 0;

      pRCTAObjGlobal->bBreakthroughHit[0] = 0;
      pRCTAObjGlobal->bBreakthroughHit[1] = 0;

      pRCTAObjGlobal->bWarning[0] = 0;
      pRCTAObjGlobal->bWarning[1] = 0;

      pRCTAObjGlobal->bWarningLastCycle[0] = 0;
      pRCTAObjGlobal->bWarningLastCycle[1] = 0;

      pRCTAObjGlobal->bObjectInRange[0] = 0;
      pRCTAObjGlobal->bObjectInRange[1] = 0;

      pRCTAObjGlobal->bRelevant = 0;
      pRCTAObjGlobal->bMirror = 0;
      pRCTAObjGlobal->bObjectFromSide = 0;
      pRCTAObjGlobal->bValidApproachAngle = 0;
      pRCTAObjGlobal->bUpdatedRecently = 0;
      pRCTAObjGlobal->bRelevant = 0;
      pRCTAObjGlobal->bQuality = 0;
    }
  }
  CTARCTACheckMultiObjectInterrupt(RCTAreqPorts, RCTAGlobal);
}

/*****************************************************************************
  Functionname:CTARCTACheckObjectUpdateRecently */ /*!

  @brief Check object update status

  @description Check object update status

  @param[in]  uiMeasuredTargetFrequency_nu       Bitfield to indicate if the object was measured in the last 8 cycles

  @param[out] pbUpdatedRecently                  Flag whether the object is updated recently 

  @return
*****************************************************************************/
void CTARCTACheckObjectUpdateRecently(const uint8 uiMeasuredTargetFrequency_nu,
                                      boolean* pbUpdatedRecently) {
  boolean bUpdateRecently = FALSE;
  // Update in this cycle
  if ((uiMeasuredTargetFrequency_nu & 128U) == 128U)  // 1000 0000
  {
    bUpdateRecently = TRUE;
  }
  *pbUpdatedRecently = bUpdateRecently;
}

/*****************************************************************************
  Functionname:CTARCTACheckObjectInRange                                    */ /*!

  @brief Check for valid RCTA range

  @description The range check prevents target which may turn away from warning

  @param[in]  fDistX_met                                 Object's longitudinal relative distance
			  fDistY_met                                 Object's lateral relative distance
			  fMaxObjRange_met                           The max range thresholds of three levels RCTA warning
  @param[out] pbObjectInRange                            Flag whether the object is in the range of three different level                

  @return
*****************************************************************************/
void CTARCTACheckObjectInRange(const float32 fDistX_met,
                               const float32 fDistY_met,
                               float32* fMaxObjRange_met,
                               boolean* pbObjectInRange) {
  float32 fRangeSqr = 0.F;
  boolean bInRange = FALSE;
  for (uint8 uWarnLevel = 0U; uWarnLevel < CTA_RCTA_CFG_NUM_OF_WARN_LEVELS;
       uWarnLevel++) {
    bInRange = FALSE;
    // Calculate the distance of the object
    fRangeSqr = SQR(fDistX_met) + SQR(fDistY_met);
    // Check whether the object is in range
    if (fRangeSqr < SQR(fMaxObjRange_met[uWarnLevel])) {
      bInRange = TRUE;
    }
    pbObjectInRange[uWarnLevel] = bInRange;
  }
}

/*****************************************************************************
  Functionname:CTARCTACheckSideApproach                                    */ /*!

  @brief Check the object side approach

  @description Check the object side approach

  @param[in]  bRightSensor: Flag whether the object is detected by the right sensor
              fFirstDetectY_met: Y position where the object was created, unit:m
			  fDistY_met: Object's lateral relative distance

  @param[out] pbObjectFromSide: Flag whether the object approach from side

  @return
*****************************************************************************/
void CTARCTACheckSideApproach(const boolean bRightSensor,
                              const float32 fFirstDetectY_met,
                              const float32 fDistY_met,
                              boolean* pbObjectFromSide) {
  boolean bSideObject = *pbObjectFromSide;
  float32 fDistY = fDistY_met;

  // Check if on right sensor, revert the sign of the Y distance for using the
  // old coordinate system
  if (bRightSensor) {
    fDistY = fDistY * -1.F;
  }
  // If the object is not yet detected as side objects
  if (!bSideObject) {
    // Check for the first detection position and the current position
    if (fFirstDetectY_met > RCTA_SIDE_APPROACH_FIRSTY_MIN ||  // 4
        fDistY > RCTA_SIDE_APPROACH_DISTY_MIN)                // 8
    {
      bSideObject = TRUE;
    }
  }
  *pbObjectFromSide = bSideObject;
}

/*****************************************************************************
  Functionname:CTARCTACheckApproachAngle                                     */ /*!

  @brief Check for valid RCTA approach angle

  @description Check for valid RCTA approach angle

  @param[in]  bRightSensor:Flag whether the object is detected by the right sensor
              fRelHeading_rad:Heading Angle
			  fRelHeadingStd_rad:standard deviation heading Angle
			  fMaxHeadingAngle_deg:Calculate the max value of heading angle
			  fMinHeadingAngle_deg:Calculate the min value of heading angle
			  bObjectFromSide: Flag whether the object approach from side 
			  bValidApproachAngle:Flag whether the object heading angle is valid in the last cycle
  @param[out] bValidApproachAngle:Flag whether the object heading angle is valid in the current cycle

  @return
*****************************************************************************/
void CTARCTACheckApproachAngle(const boolean bRightSensor,
                               const float32 fRelHeading_rad,
                               const float32 fRelHeadingStd_rad,
                               float32 fMaxHeadingAngle_deg,
                               float32 fMinHeadingAngle_deg,
                               boolean bObjectFromSide,
                               boolean* bValidApproachAngle) {
  float32 fMaxHeadingAngle = fMaxHeadingAngle_deg;
  float32 fMinHeadingAngle = fMinHeadingAngle_deg;
  float32 fMaxApprochAngleStd = RCTA_APPROACH_ANGLE_MAX_STD;  // 25
  float32 fRelHeading = fRelHeading_rad;
  boolean bValidAngle = FALSE;
  // Check whether hysteresis shall be applied
  if (*bValidApproachAngle) {
    // add hysteresis
    fMaxHeadingAngle += RCTA_APPROACH_ANGLE_HYST;  // 10
    fMinHeadingAngle -= RCTA_APPROACH_ANGLE_HYST;
    fMaxApprochAngleStd *= 2.F;
  }
  // Check if on right sensor, revert the sign of the heading for using the old
  // coordinate system
  if (bRightSensor) {
    fRelHeading = fRelHeading * -1.F;
  }
  // side targets use the customer defined parameters for max rear crossing
  // angles
  if (bObjectFromSide) {
    if (fRelHeading < DEG2RAD(fMaxHeadingAngle) &&
        fRelHeading > DEG2RAD(fMinHeadingAngle) &&
        fRelHeadingStd_rad < DEG2RAD(fMaxApprochAngleStd)) {
      bValidAngle = TRUE;
    }
  }
  // to inhibit false alerts, targets which appear/approach from behind use
  // tighter angle requirements
  else {
    if (fRelHeading < DEG2RAD(fMaxHeadingAngle) &&
        fRelHeading > DEG2RAD(fMinHeadingAngle + RCTA_APPROACH_ANGLE_MARGIN) &&
        fRelHeadingStd_rad < DEG2RAD(fMaxApprochAngleStd)) {
      bValidAngle = TRUE;
    }
  }
  *bValidApproachAngle = bValidAngle;
}

/*****************************************************************************
  Functionname:CTARCTACheckForMirror                                     */ /*!

  @brief Check if object has sufficient mirror probability

  @description Check if object has sufficient mirror probability

  @param[in]  fMirrorProb_per: The probability that the object is mirror
              pbMirror: Flag whether the object is mirror in the last cycle
  @param[out] pbMirror: Flag whether the object is mirror in the current cycle

  @return
*****************************************************************************/
void CTARCTACheckForMirror(const float32 fMirrorProb_per, boolean* pbMirror) {
  boolean bMirror = FALSE;
  if (!(*pbMirror)) {
    if (fMirrorProb_per > RCTA_MIRROR_ACTIVATION_THRESH)  // 0.8
    {
      // Object has sufficient mirror probability, set mirror flag
      bMirror = TRUE;
    }
  } else {
    if (fMirrorProb_per > RCTA_MIRROR_DEACTIVATION_THRESH)  // 0.5
    {
      // Object has sufficient mirror probability, keep mirror flag
      bMirror = TRUE;
    }
  }
  *pbMirror = bMirror;
}

/*****************************************************************************
  Functionname: CTARCTACheckObjectRelevance */ /*!

  @brief Check for object relevance

  @description Check if the tracked object is relevant (exists) for a warning

  @param[in]   fXMovement_met          The object total moving distance in the x direction
			   fYMovement_met          The object total moving distance in the y direction
			   fVabsX_mps              Object's longitudinal velocity over ground
			   fVabsY_mps              Object's lateral velocity over ground
			   uiLifeCycles_nu         Object lifetime in cycles
			   fVTargetMin             The min target vehicle speed of RCTA is actived; Value: 1.399f m/s
			   pbRelevant              Flag whether the object is relevant for a RCTA warning in the last cycle
  @param[out]  pbRelevant              Flag whether the object is relevant for a RCTA warning in the current cycle

  @return
*****************************************************************************/
void CTARCTACheckObjectRelevance(const float32 fVabsX_mps,
                                 const float32 fVabsY_mps, float32 fVTargetMin,
                                 const uint16 uiLifeCycles_nu,
                                 float32 fXMovement_met, float32 fYMovement_met,
                                 boolean* pbRelevant) {
  boolean bRelevance = FALSE;
  float32 fSpeedOverGroundSqr = 0.F;
  float32 fMovementAbsSqr = 0.F;
  // Compute object's overall velocity over ground
  fSpeedOverGroundSqr = SQR(fVabsX_mps) + SQR(fVabsY_mps);
  // Compute object's overall movement over ground
  fMovementAbsSqr = SQR(fXMovement_met) + SQR(fYMovement_met);

  if (*pbRelevant) {
    if (fSpeedOverGroundSqr > SQR(0.5 * fVTargetMin)) {
      bRelevance = TRUE;
    }
  } else {
    // Threshold for movement over ground decreased to 2.0m (from 4.0m), reason:
    // use case with obscured FoV
    if (fSpeedOverGroundSqr > SQR(fVTargetMin) && fMovementAbsSqr > SQR(2.0F) &&
        uiLifeCycles_nu > RCTA_RELEVANCE_LIFECYCLE_MIN)  // 2
    {
      bRelevance = TRUE;
    }
  }
  *pbRelevant = bRelevance;
}

/*****************************************************************************
  Functionname:CTARCTACheckObjectQuality                                     */ /*!

  @brief Check object quality

  @description Check object quality

  @param[in]  pEMSRRObjInput:EM side radar object information structure
              fUpdateRate_nu:The object measurement update rate,unit:NULL
			  fAssocProbFiltered_nu:Highest cluster association probability of the object filter result
  @param[out] pbQuality:Flag whether the object's quality meet warning conditions

  @return
*****************************************************************************/
void CTARCTACheckObjectQuality(const RCTAEMSRRObjInReq_t* pEMSRRObjInput,
                               float32 fUpdateRate_nu,
                               float32 fAssocProbFiltered_nu,
                               boolean* pbQuality) {
  boolean bObjQuality = FALSE;
  float32 fPoEThresh = 0.F;
  float32 fPoEThreshLifetime = 0.F;
  float32 fUpdateRateThresh = 0.F;
  float32 fUpdateRateThreshLifetime = 0.F;
  float32 fAssocProbThresh = 0.F;
  float32 fAssocProbThreshRange = 0.F;
  float32 fRangeSqr = 0.F;
  float32 fSpeedOverGroundSqr = 0.F;

  // Calculate object's speed over ground square
  fSpeedOverGroundSqr =
      SQR(pEMSRRObjInput->fVabsX_mps) + SQR(pEMSRRObjInput->fVabsY_mps);
  // Calculate PoE threshold
  fPoEThresh = TUE_CML_BoundedLinInterpol2(
      fSpeedOverGroundSqr, SQR(RCTA_LI_POE_MIN_SPEED),
      SQR(RCTA_LI_POE_MAX_SPEED), RCTA_LI_POE_MIN_POE,
      RCTA_LI_POE_MAX_POE);  // 30/3.6  60/3.6  0.99  0.93
  fPoEThreshLifetime = TUE_CML_BoundedLinInterpol2(
      (float32)pEMSRRObjInput->uiLifeCycles_nu, RCTA_LI_POE_LFT_MIN_LIFETIME,
      RCTA_LI_POE_LFT_MAX_LIFETIME, RCTA_LI_POE_LFT_MIN_POE,
      RCTA_LI_POE_LFT_MAX_POE);  // 15  25  0.93  0.99
  fPoEThresh = MAX(fPoEThresh, fPoEThreshLifetime);
  // Calculate update rate threshold
  fUpdateRateThresh = TUE_CML_BoundedLinInterpol2(
      fSpeedOverGroundSqr, SQR(RCTA_LI_UPDATE_MIN_SPEED),
      SQR(RCTA_LI_UPDATE_MAX_SPEED), RCTA_LI_UPDATE_MIN_POE,
      RCTA_LI_UPDATE_MAX_POE);  // 30/3.6  60/3.6  0.85  0.75
  fUpdateRateThreshLifetime = TUE_CML_BoundedLinInterpol2(
      (float32)pEMSRRObjInput->uiLifeCycles_nu, RCTA_LI_UPDATE_LFT_MIN_LIFETIME,
      RCTA_LI_UPDATE_LFT_MAX_LIFETIME, RCTA_LI_UPDATE_LFT_MIN_UPDATE,
      RCTA_LI_UPDATE_LFT_MAX_UPDATE);  // 15  25  0.75  0.85
  fUpdateRateThresh = MAX(fUpdateRateThresh, fUpdateRateThreshLifetime);
  // Calculate the association probability threshold
  // The association probability threshold should be speed and distance
  // dependent This makes sure that fast objects are allowed to warn early
  // enough, but mostly close ghost objects do not reach the threshold
  fAssocProbThresh = TUE_CML_BoundedLinInterpol2(
      fSpeedOverGroundSqr, SQR(RCTA_LI_ASSOCP_MIN_SPEED),
      SQR(RCTA_LI_ASSOCP_MAX_SPEED), RCTA_LI_ASSOCP_MIN_ASSOCP,
      RCTA_LI_ASSOCP_MAX_ASSOCP);
  fRangeSqr = SQR(pEMSRRObjInput->fDistX_met) + SQR(pEMSRRObjInput->fDistY_met);
  fAssocProbThreshRange = TUE_CML_BoundedLinInterpol2(
      fRangeSqr, SQR(RCTA_LI_ASSOCP_RNG_MIN_RANGE),
      SQR(RCTA_LI_ASSOCP_RNG_MAX_RANGE), RCTA_LI_ASSOCP_RNG_MIN_ASSOCP,
      RCTA_LI_ASSOCP_RNG_MAX_ASSOCP);
  fAssocProbThresh = MIN(fAssocProbThresh, fAssocProbThreshRange);
  // Apply hysteresis
  if (*pbQuality) {
    fPoEThresh -= RCTA_POE_HYST;            // 0.02
    fUpdateRateThresh -= RCTA_UPDRTE_HYST;  // 0.1
    fAssocProbThresh -= RCTA_ASSOCP_HYST;   // 0.1
  }
  // Check all quality criteria
  if (pEMSRRObjInput->fProbabilityOfExistence_per > fPoEThresh &&
      fUpdateRate_nu > fUpdateRateThresh &&
      fAssocProbFiltered_nu > fAssocProbThresh && pEMSRRObjInput->bObjStable) {
    bObjQuality = TRUE;
  }

  *pbQuality = bObjQuality;
}

/*****************************************************************************
  Functionname:CTARCTACheckBreakthroughHit                                    */ /*!

  @brief Check for object breakthrough hit

  @description Check for object trajectory intersection with rear crossing line

  @param[in]  fXMaxBreakthrough_met:x-axis min edge of breakthrough
              fXMinBreakthrough_met:x-axis max edge of breakthrough
			  fDistToCrossingLine_met
			  fXBreakthrough_met:The x-axis breakthrough of the object
			  pbWarning:Flag whether the RCTA warning is activated
  @param[out] pbBreakthroughHit:Flag whether the object hits breakthrough

  @return
*****************************************************************************/
void CTARCTACheckBreakthroughHit(float32* fXMaxBreakthrough_met,
                                 float32* fXMinBreakthrough_met,
                                 float32 fDistToCrossingLine_met,
                                 float32 fXBreakthrough_met, boolean* pbWarning,
                                 boolean* pbBreakthroughHit) {
  boolean bBreakthroughHit = FALSE;
  float32 fMaxBreakthrough = 0.F;
  float32 fMinBreakthrough = 0.F;

  for (uint8 uWarnLevel = 0U; uWarnLevel < CTA_RCTA_CFG_NUM_OF_WARN_LEVELS;
       uWarnLevel++) {
    bBreakthroughHit = FALSE;
    fMaxBreakthrough = fXMaxBreakthrough_met[uWarnLevel];
    fMinBreakthrough = fXMinBreakthrough_met[uWarnLevel];
    // The object already passed the crossing line

    // 0-->-1,make sure check breakthrough normally
    if (fDistToCrossingLine_met > -1.0F) {
      // Switch on hysteresis
      if (pbWarning[uWarnLevel]) {
        fMaxBreakthrough += RCTA_BREAKTHROUGH_HYSTERESIS;  // 2
        fMinBreakthrough -= RCTA_BREAKTHROUGH_HYSTERESIS;
      }
      // Test breakthrough in range
      if (fXBreakthrough_met < fMaxBreakthrough &&
          fXBreakthrough_met > fMinBreakthrough) {
        bBreakthroughHit = TRUE;
      }
    }
    pbBreakthroughHit[uWarnLevel] = bBreakthroughHit;
  }
}

/*****************************************************************************
  Functionname:CTARCTACalculateBreakthroughHitConfidence */ /*!

  @brief Calculate breakthrough hit confidence for an object

  @description Calculate breakthrough hit confidence for an object

  @param[in]  fXBreakthrough_met:The y-axis breakthrough of the object
              fXBreakthroughStd_met:The y-axis breakthrough standard deviation of the object
			  fXMinBreakthrough_met:x-axis min edge of breakthrough
			  fXMaxBreakthrough_met:x-axis max edge of breakthrough
			  pbBreakthroughHit:Flag whether the object hits breakthrough
  @param[out] uBreakthroughHitConfi:Breakthrough hit confidence

  @return
*****************************************************************************/
void CTARCTACalculateBreakthroughHitConfidence(float32 fXBreakthrough_met,
                                               float32 fXBreakthroughStd_met,
                                               float32* fXMaxBreakthrough_met,
                                               float32* fXMinBreakthrough_met,
                                               boolean* pbBreakthroughHit,
                                               uint8* puBreakthroughHitConfi) {
  float32 fHitConfidence = 0.F;
  float32 fHitConfidenceUpdate = 0.F;
  float32 fBreakthroughHitRatio = 0.F;
  float32 fMaxHitLimit = 0.F;
  float32 fMinHitLimit = 0.F;
  float32 fMaxBreakthroughLimit = 0.F;
  float32 fMinBreakthroughLimit = 0.F;
  // float32 fMaxConfUpdate = OSE_CONFI_UPDATE_MAX; //20
  // float32 fMinConfUpdate = OSE_CONFI_UPDATE_MIN; //5

  for (uint8 uWarnLevel = 0U; uWarnLevel < CTA_RCTA_CFG_NUM_OF_WARN_LEVELS;
       uWarnLevel++) {
    fHitConfidence = puBreakthroughHitConfi[uWarnLevel];
    // Get the min and max point where the object hits the breakthrough line
    fMaxBreakthroughLimit = fXBreakthrough_met + fXBreakthroughStd_met;
    fMinBreakthroughLimit = fXBreakthrough_met - fXBreakthroughStd_met;
    fMaxHitLimit = CTARCTACalculateHitLimit(
        fMaxBreakthroughLimit, fXMaxBreakthrough_met[uWarnLevel],
        fXMinBreakthrough_met[uWarnLevel], pbBreakthroughHit[uWarnLevel]);
    fMinHitLimit = CTARCTACalculateHitLimit(
        fMinBreakthroughLimit, fXMaxBreakthrough_met[uWarnLevel],
        fXMinBreakthrough_met[uWarnLevel], pbBreakthroughHit[uWarnLevel]);
    // Calculate the ratio of the length where the object will probably hit the
    // breakthrough and the uncertainty of the hit estimation
    fBreakthroughHitRatio =
        (fMaxHitLimit - fMinHitLimit) / SafeDiv(2.0F * fXBreakthroughStd_met);
    // Breakthrough was not hit
    if (!pbBreakthroughHit[uWarnLevel]) {
      // The confidence should be decreased. The less the object hits the
      // breakthrough the faster the confidence shall decrease
      fHitConfidenceUpdate = TUE_CML_BoundedLinInterpol2(
          fBreakthroughHitRatio, 0.0F, 1.0F, -RCTA_LI_BRKACTIVE_MIN_HITCONF,
          -RCTA_LI_BRKACTIVE_MAX_HITCONF);  // 5   20
    } else {
      // The confidence should be increased. The more the object hits the
      // breakthrough the faster the confidence shall increase
      fHitConfidenceUpdate = TUE_CML_BoundedLinInterpol2(
          fBreakthroughHitRatio, 0.0F, 1.0F, RCTA_LI_BRKACTIVE_MIN_HITCONF,
          RCTA_LI_BRKACTIVE_MAX_HITCONF);
    }
    fHitConfidence += fHitConfidenceUpdate;
    fHitConfidence = TUE_CML_MinMax(0.0F, 100.0F, fHitConfidence);

    puBreakthroughHitConfi[uWarnLevel] = (uint8)fHitConfidence;
  }
}

/*****************************************************************************
  Functionname: CTARCTACalculateHitLimit */ /*!

  @brief: Limit the expected breakthrough between the upper and the lower

  @description: Limit the expected breakthrough between the upper and the lower

  @param[in]  fBreakthroughLimit          pfYBreakthrough[uLine] + pfYBreakthroughStd[uLine]
			  fYMaxBreakthrough           y-axis max edge of breakthrough
			  fYMinBreakthrough           y-axis min edge of breakthrough
			  bBreakthroughHit            Flag whether the object hits breakthrough in the current cycle
  @param[out] void

  @return     fRet                        Limited value of breakthrough
*****************************************************************************/
float32 CTARCTACalculateHitLimit(float32 fBreakthroughLimit,
                                 float32 fXMaxBreakthrough,
                                 float32 fXMinBreakthrough,
                                 boolean bBreakthroughHit) {
  float32 fBreakthroughHyst = 0.F;
  float32 fRet = 0.F;

  // Add hysteresis of breakthrough
  if (bBreakthroughHit) {
    fBreakthroughHyst = RCTA_BREAKTHROUGH_HYSTERESIS;  // 2
  }

  if (fBreakthroughLimit > (fXMaxBreakthrough + fBreakthroughHyst)) {
    // The limit of the expected breakthrough is above the upper breakthrough
    // limit
    fRet = fXMaxBreakthrough;
  } else if (fBreakthroughLimit < (fXMinBreakthrough - fBreakthroughHyst)) {
    // The limit of the expected breakthrough is below the lower breakthrough
    // limit
    fRet = fXMinBreakthrough;
  } else {
    // The limit of the expected breakthrough is within the defined breakthrough
    fRet = fBreakthroughLimit;
  }
  return fRet;
}

/*****************************************************************************
  Functionname: CTARCTAUpdateBTHitHysteresisTimer */ /*!

  @brief Update the warning hysteresis timer

  @description Update the warning hysteresis timer to prevent unstable per object warnings
               and to dead reckon to exact warning termination time

  @param[in]  fCycleTime_s:Current task cycle time from EMGlobalOutput
              fTTC_s:TTC of the object
			  fTTCFiltered_s:Filtered TTC of the object
			  bWarning:Flag whether the RCTA warning is activated
			  bBreakthroughHit:Flag whether the object hits breakthrough in the current cycle
			  bWarningLastCycle:Flag whether the RCTA warning is activated in the last cycle
  @param[out] fBTHitHystTimer_s:the warning hysteresis timer

  @return
*****************************************************************************/
void CTARCTAUpdateBTHitHysteresisTimer(float32 fCycleTime_s, float32 fTTC_s,
                                       float32 fTTCFiltered_s,
                                       boolean* bWarning,
                                       boolean* bBreakthroughHit,
                                       boolean* bWarningLastCycle,
                                       float32* pfBTHitHystTimer_s) {
  for (uint8 uWarnLevel = 0U; uWarnLevel < CTA_RCTA_CFG_NUM_OF_WARN_LEVELS;
       uWarnLevel++) {
    // When a warning starts or when the breakthrough was hit during a warning
    if (bWarning[uWarnLevel] &&
        (bBreakthroughHit[uWarnLevel] || !bWarningLastCycle[uWarnLevel])) {
      // Set the dead reckon timer to the calculated TTC
      pfBTHitHystTimer_s[uWarnLevel] = MIN(fTTC_s, fTTCFiltered_s);
    } else {
      // Run the timer
      pfBTHitHystTimer_s[uWarnLevel] -= fCycleTime_s;
    }
    // Limit it to 1s
    pfBTHitHystTimer_s[uWarnLevel] =
        TUE_CML_MinMax(0.0F, 1.0F, pfBTHitHystTimer_s[uWarnLevel]);
  }
}

/*****************************************************************************
  Functionname:CTARCTACheckBTHitHysteresisTimer */ /*!

  @brief Check warning against hysteresis timer

  @description Check warning against hysteresis timer

  @param[in]  fBTHitHystTimer_s:the warning hysteresis timer

  @param[out] bBTHitHystActive:Flag whether the warning hysteresis

  @return
*****************************************************************************/
void CTARCTACheckBTHitHysteresisTimer(float32* fBTHitHystTimer_s,
                                      boolean* pbBTHitHystActive) {
  boolean bBTHitHystActive = FALSE;

  for (uint8 uWarnLevel = 0U; uWarnLevel < CTA_RCTA_CFG_NUM_OF_WARN_LEVELS;
       uWarnLevel++) {
    bBTHitHystActive = FALSE;
    // As long as the time is not elapsed set hysteresis to active
    if (fBTHitHystTimer_s[uWarnLevel] > TUE_C_F32_DELTA) {
      bBTHitHystActive = TRUE;
    }
    pbBTHitHystActive[uWarnLevel] = bBTHitHystActive;
  }
}

/*****************************************************************************
  Functionname:CTARCTACheckTTC                                    */ /*!

  @brief Check the object's TTC

  @description Check the object's TTC for possible OSE warning start
   
  @param[in]  fTTC_s:TTC of the object     
              fTTCFiltered_s:Filtered TTC of the object
			  fTTCThreshold_s:TTC thresholds of three levels
  @param[out] bTTCBelowThresh:Flag whether the object's TTC is below TTC Threshold

  @return
*****************************************************************************/
void CTARCTACheckTTC(float32 fTTC_s, float32 fTTCFiltered_s,
                     float32* pfTTCThreshold_s, boolean* pbTTCBelowThresh) {
  float32 fTTCThreshold = 0.F;
  boolean bTTCBelowThresh = FALSE;

  for (uint8 uWarnLevel = 0U; uWarnLevel < CTA_RCTA_CFG_NUM_OF_WARN_LEVELS;
       uWarnLevel++) {
    bTTCBelowThresh = FALSE;
    fTTCThreshold = pfTTCThreshold_s[uWarnLevel];
    // The TTC was already below the threshold. Use hysteresis
    if (pbTTCBelowThresh[uWarnLevel]) {
      if (fTTC_s < (2.0F * fTTCThreshold) ||
          fTTCFiltered_s < (2.0 * fTTCThreshold)) {
        bTTCBelowThresh = TRUE;
      }
    } else {
      if (fTTC_s < fTTCThreshold || fTTCFiltered_s < fTTCThreshold) {
        bTTCBelowThresh = TRUE;
      }
    }
    pbTTCBelowThresh[uWarnLevel] = bTTCBelowThresh;
  }
}

/*****************************************************************************
  Functionname:CTARCTACheckShortWarning                                     */ /*!

  @brief Check the object's TTC

  @description Check the object's TTC for possible OSE warning start

  @param[in]  fTTC_s:TTC of the object
			  fTTCFiltered_s:Filtered TTC of the object
  @param[out] bShortTTC:Flag whether object's TTC is below 0

  @return 
*****************************************************************************/
void CTARCTACheckShortWarning(float32 fTTC_s, float32 fTTCFiltered_s,
                              boolean* pbShortTTC) {
  boolean bShortTTC = FALSE;

  if (fTTC_s < RCTA_SHORT_WARNING_THRESH ||
      fTTCFiltered_s < RCTA_SHORT_WARNING_THRESH)  // 0
  {
    bShortTTC = TRUE;
  }
  *pbShortTTC = bShortTTC;
}

/*****************************************************************************
  Functionname:CTARCTAWarningDecision                                     */ /*!

  @brief Decision if this object causes a RCTA warning

  @description Decision if this track causes a RCTA warning by checking breakthrough and TTC,
               consider hysteresis and detect cluster hopper here and save ID of track

  @param[in]  fRCS:RCS
              pRCTAObjGlobal:RCTA object information global structure
			  bRearTrack_nu
			  bWarningLastCycle:Flag whether the RCTA warning was activated in the last cycle
  @param[out] pbWarning:Flag whether the RCTA warning was activated in the current cycle

  @return
*****************************************************************************/
#ifdef ST_PERCEPION

void CTARCTAWarningDecision(float32 fRCS, RCTAObjGlobal_t* pRCTAObjGlobal,
                            boolean bRearTrack_nu, boolean* bWarningLastCycle,
                            boolean* pbWarning) {
  boolean bWarning;

  for (uint8 uWarnLevel = 0U; uWarnLevel < CTA_RCTA_CFG_NUM_OF_WARN_LEVELS;
       uWarnLevel++) {
    bWarning = FALSE;
    // Check warning start conditions
    if (!pbWarning[uWarnLevel]) {
      if (pRCTAObjGlobal->uBreakthroughHitConfi[uWarnLevel] >=
              RCTA_HIT_CONFI_THRESH &&  // 90
          pRCTAObjGlobal->bObjectInRange[uWarnLevel] &&
          pRCTAObjGlobal->bRelevant &&
          pRCTAObjGlobal->bTTCBelowThresh[uWarnLevel] &&
          !pRCTAObjGlobal->bMirror && !pRCTAObjGlobal->bShortTTC &&
          !bRearTrack_nu && pRCTAObjGlobal->bValidApproachAngle &&
          pRCTAObjGlobal->bUpdatedRecently && pRCTAObjGlobal->bQuality &&
          fRCS > RCTA_RCS_THRESH)  //-20
      {
        bWarning = TRUE;
      }
    }
    // Check continued warning condition
    else {
      if (pRCTAObjGlobal->bRelevant &&
          pRCTAObjGlobal->bTTCBelowThresh[uWarnLevel] &&
          pRCTAObjGlobal->bBTHitHystActive[uWarnLevel] &&
          !pRCTAObjGlobal->bMirror && !bRearTrack_nu) {
        bWarning = TRUE;
      }
    }
    // Store current warning info to detect position edge
    pRCTAObjGlobal->bWarningLastCycle[uWarnLevel] = pbWarning[uWarnLevel];
    pbWarning[uWarnLevel] = bWarning;
  }
}
#endif  // ST_PERCEPION

void CTARCTAWarningDecision(uint8 uGearReverse, RCTAObjGlobal_t* pRCTAObjGlobal,
                            boolean bRearTrack_nu, boolean bRCTASixDoorsClosed,
                            boolean* pbWarning) {
  boolean bWarning = FALSE;

  for (uint8 uWarnLevel = 0U; uWarnLevel < CTA_RCTA_CFG_NUM_OF_WARN_LEVELS;
       uWarnLevel++) {
    bWarning = FALSE;
    if (bRCTASixDoorsClosed) {
      // Check warning start conditions
      if (!pbWarning[uWarnLevel]) {
        if (pRCTAObjGlobal->uBreakthroughHitConfi[uWarnLevel] >=
                RCTA_HIT_CONFI_THRESH &&  // 90
            pRCTAObjGlobal->bObjectInRange[uWarnLevel] &&
            pRCTAObjGlobal->bRelevant &&
            pRCTAObjGlobal->bTTCBelowThresh[uWarnLevel] &&
            !pRCTAObjGlobal->bMirror && !pRCTAObjGlobal->bShortTTC &&
            !bRearTrack_nu && pRCTAObjGlobal->bValidApproachAngle &&
            pRCTAObjGlobal->bUpdatedRecently && pRCTAObjGlobal->bQuality &&
            (1 == uGearReverse))  // R Gear
        {
          bWarning = TRUE;
        }
      }
      // Check continued warning condition
      else {
        if (pRCTAObjGlobal->bRelevant &&
            pRCTAObjGlobal->bTTCBelowThresh[uWarnLevel] &&
            pRCTAObjGlobal->bBTHitHystActive[uWarnLevel] &&
            !pRCTAObjGlobal->bMirror && !bRearTrack_nu) {
          bWarning = TRUE;
        }
      }
    }
    // Store current warning info to detect position edge
    pRCTAObjGlobal->bWarningLastCycle[uWarnLevel] = pbWarning[uWarnLevel];
    pbWarning[uWarnLevel] = bWarning;
  }
}

/*****************************************************************************
  Functionname:CTARCTASetGlobalWarning                                     */ /*!

  @brief Global warning decision 

  @description Decision if this track causes a RCTA warning by checking breakthrough and TTC,
               consider hysteresis and detect cluster hopper here and save ID of track.

  @param[in]  uObj:The serial number of number   
              fDistY_met:Object's lateral relative distance
			  fWidthLeft_met:Object's width left of the track position(left sensor view)
			  pbWarning:Flag whether the RCTA warning was activated in the current cycle
			  fTTCFiltered_s:Filtered TTC of the object
  @param[out] RCTAGlobal:RCTA global structure

  @return
*****************************************************************************/
void CTARCTASetGlobalWarning(uint8 uObj, const float32 fDistY_met,
                             const float32 fWidthLeft_met, boolean* pbWarning,
                             float32 fTTCFiltered_s, RCTAGlobal_t* RCTAGlobal) {
  boolean rcta_warning = FALSE;
  for (uint8 uWarnLevel = 0U; uWarnLevel < CTA_RCTA_CFG_NUM_OF_WARN_LEVELS;
       uWarnLevel++) {
    // The current object is warning

    // if (*pbWarning) {
    if (pbWarning[uWarnLevel]) {
      // Set the global warning
      RCTAGlobal->bRCTAWarnActive[uWarnLevel] = TRUE;
      rcta_warning = TRUE;
      // If the TTC of the current object is below another warning object
      if (fTTCFiltered_s < RCTAGlobal->fCriticalTTC_s) {
        RCTAGlobal->fCriticalTTC_s = fTTCFiltered_s;
        RCTAGlobal->uCriticalObjID_nu = uObj;
        // Save rear edge position
        RCTAGlobal->fCriticalObjDistY_met = fDistY_met + fWidthLeft_met;
      }
      // Set additional info according to the warning state
      // switch (uWarnLevel) {
      //   case (uint8)RCTA_WARN_LEVEL_ONE:
      //     // If the TTC of the current object is below another warning
      //     object if (fTTCFiltered_s < RCTAGlobal->fCriticalTTC_s) {
      //       RCTAGlobal->fCriticalTTC_s = fTTCFiltered_s;
      //       RCTAGlobal->uCriticalObjID_nu = uObj;
      //       // Save rear edge position
      //       RCTAGlobal->fCriticalObjDistY_met = fDistY_met +
      //       fWidthLeft_met;
      //     }
      //     break;
      //   case (uint8)RCTA_WARN_LEVEL_TWO:
      //     break;
      //   case (uint8)RCTA_WARN_LEVEL_THREE:
      //     break;
      //   default:
      //     break;
      // }
    }
  }
}

/*****************************************************************************
  Functionname:CTARCTACheckMultiObjectInterrupt */ /*!

  @brief Handle the warning interruption for multiple following targets

  @description Handle the warning interruption for multiple following targets

  @param[in]  RCTAreqPorts:RCTA input information structure 
              RCTAGlobal:RCTA global variable structure
  @param[out] RCTAGlobal:RCTA global variable structure

  @return
*****************************************************************************/
void CTARCTACheckMultiObjectInterrupt(const RCTAInReq_t* RCTAreqPorts,
                                      RCTAGlobal_t* RCTAGlobal) {
  float32 fSafetyMargin = 0.F;
  float32 fCriticalObjFrontDistY = 0.F;
  boolean bInterrupt = FALSE;
  float32 fDistY = 0.F;
  float32 fVrelY = 0.F;

  // Check whether the critical object has changed
  if (RCTAGlobal->uCriticalObjID_nu != TUE_C_UI8_VALUE_INVALID &&
      RCTAGlobal->uCriticalObjIDLastCycle_nu != TUE_C_UI8_VALUE_INVALID &&
      RCTAGlobal->uCriticalObjID_nu != RCTAGlobal->uCriticalObjIDLastCycle_nu) {
    fDistY = RCTAreqPorts->EMSRRObjListInput[RCTAGlobal->uCriticalObjID_nu]
                 .fDistY_met;
    fVrelY = RCTAreqPorts->EMSRRObjListInput[RCTAGlobal->uCriticalObjID_nu]
                 .fVrelY_mps;
    // Check if on right sensor, revert the sign of the Y distance and Vy for
    // using the old coordinate system
    if (RCTAreqPorts->EMSRRObjListInput[RCTAGlobal->uCriticalObjID_nu]
            .bRightSensor) {
      fVrelY = fVrelY * -1.F;
      fDistY = fDistY * -1.F;
    }
    fCriticalObjFrontDistY =
        fDistY - RCTAreqPorts->EMSRRObjListInput[RCTAGlobal->uCriticalObjID_nu]
                     .fWidthLeft_met;
    // Calculate the safety margin between the critical object of the last
    // cycle and the current cycle
    fSafetyMargin =
        (fCriticalObjFrontDistY - RCTAGlobal->fCriticalObjDistYLastCycle_met) /
        SafeDiv(-fVrelY);

    if (fSafetyMargin > RCTA_MULTIOBJ_SAFE_MARGIN)  // 0.7
    {
      // set the interruption counter
      RCTAGlobal->uInterruptCycleCount_nu =
          RCTA_MULTIOBJ_INTERRUPT_CYCLES;  // 1
    }
  }

  // Check whether the interruption counter is still not 0
  if (RCTAGlobal->uInterruptCycleCount_nu > 0U) {
    bInterrupt = TRUE;
    RCTAGlobal->uInterruptCycleCount_nu--;
  }
  RCTAGlobal->bWarningInterrupt = bInterrupt;
}

/*****************************************************************************
  Functionname:RCTAProProcess                                     */ /*!

  @brief Process the warnings for RCTA function

  @description Process the warnings for RCTA function

  @param[in]

  @param[out]

  @return
*****************************************************************************/
void RCTAProProcess(RCTAGlobal_t* RCTAGlobal, RCTAOutPro_t* RCTAproPorts,
                    RCTADebug_t* RCTAdebugInfo) {

  if (sizeof(RCTAObjGlobal_t) != sizeof(RCTAObjOutput_t)) {
    DEBUG_Print("RCTA RCTAProProcess Error!");
    return;
  }
  memcpy(RCTAproPorts->RCTA_Va_ObjectListGlobal,  //NOLINT
         RCTAGlobal->RCTA_Va_ObjectListGlobal,
         sizeof(RCTAObjGlobal_t) * RCTA_MAX_NUM_OBJECTS);
  for (uint8 uWarnLevel = 0U; uWarnLevel < CTA_RCTA_CFG_NUM_OF_WARN_LEVELS;
       uWarnLevel++) {
    RCTAproPorts->bRCTAWarnActive[uWarnLevel] =
        RCTAGlobal->bRCTAWarnActive[uWarnLevel];
    RCTAproPorts->fTTCThreshold_s[uWarnLevel] =
        RCTAGlobal->fTTCThreshold_s[uWarnLevel];
    RCTAproPorts->fXMinBreakthrough_met[uWarnLevel] =
        RCTAGlobal->fXMinBreakthrough_met[uWarnLevel];
    RCTAproPorts->fXMaxBreakthrough_met[uWarnLevel] =
        RCTAGlobal->fXMaxBreakthrough_met[uWarnLevel];
    RCTAproPorts->fXMaxBreakthrough_met[uWarnLevel] =
        RCTAGlobal->fXMaxBreakthrough_met[uWarnLevel];
    RCTAproPorts->fMaxObjRange_met[uWarnLevel] =
        RCTAGlobal->fMaxObjRange_met[uWarnLevel];
  }
  RCTAproPorts->bWarningInterrupt = RCTAGlobal->bWarningInterrupt;
  RCTAproPorts->fCriticalObjDistYLastCycle_met =
      RCTAGlobal->fCriticalObjDistYLastCycle_met;
  RCTAproPorts->fCriticalObjDistY_met = RCTAGlobal->fCriticalObjDistY_met;
  RCTAproPorts->fCriticalTTC_s = RCTAGlobal->fCriticalTTC_s;
  RCTAproPorts->uCriticalObjIDLastCycle_nu =
      RCTAGlobal->uCriticalObjIDLastCycle_nu;
  RCTAproPorts->uCriticalObjID_nu = RCTAGlobal->uCriticalObjID_nu;
  RCTAproPorts->uInterruptCycleCount_nu = RCTAGlobal->uInterruptCycleCount_nu;
  RCTAproPorts->fMaxHeadingAngle_deg = RCTAGlobal->fMaxHeadingAngle_deg;
  RCTAproPorts->fMinHeadingAngle_deg = RCTAGlobal->fMinHeadingAngle_deg;
}