#include "planning/warning/cta/cta.h"

#include "planning/warning/cta/cta_extern.h"
#include "planning/warning/lbs/common/tue_common_libs.h"

CTAGlobal_t CTAGlobal;
// static FCTAInReq_t FCTAreqPorts;
static FCTAParam_t FCTAparams;
static FCTAOutPro_t FCTAproPorts;
static FCTADebug_t FCTAdebugInfo;
// static RCTAInReq_t RCTAreqPorts;
static RCTAParam_t RCTAparams;
static RCTAOutPro_t RCTAproPorts;
static RCTADebug_t RCTAdebugInfo;
extern RCTAInReq_t RCTAreqPorts;
extern FCTAInReq_t FCTAreqPorts;

/*****************************************************************************
  Functionname:                                       */ /*!

  @brief

  @description

  @param[in]

  @param[out]

  @return
*****************************************************************************/
void CTA_Reset() {
  CTAGlobal.eCTAState = CTA_INIT;
  // Init all CTA globals

  memset(&FCTAreqPorts, 0, sizeof(FCTAInReq_t));
  memset(&FCTAparams, 0, sizeof(FCTAParam_t));
  memset(&FCTAproPorts, 0, sizeof(FCTAOutPro_t));
  memset(&FCTAdebugInfo, 0, sizeof(FCTADebug_t));
  memset(&RCTAreqPorts, 0, sizeof(RCTAInReq_t));
  memset(&RCTAparams, 0, sizeof(RCTAParam_t));
  memset(&RCTAproPorts, 0, sizeof(RCTAOutPro_t));
  memset(&RCTAdebugInfo, 0, sizeof(RCTADebug_t));

  for (uint8 uWarnLevel = 0U; uWarnLevel < CTA_FCTA_CFG_NUM_OF_WARN_LEVELS;
       uWarnLevel++) {
    CTAGlobal.bFCTAFunctionOutput = FALSE;
  }
  for (uint8 uWarnLevel = 0U; uWarnLevel < CTA_RCTA_CFG_NUM_OF_WARN_LEVELS;
       uWarnLevel++) {
    CTAGlobal.bRCTAFunctionOutput = FALSE;
  }
  // CTAGlobal.fSensorOffsetToRear_met
  // CTAGlobal.fSensorOffsetToSide_met
  CTAGlobal.fMaxSpeedOverGround = 0.F;
  // CTAGlobal.LastCycleStates.eRoadType = CTA_ROAD_TYPE_UNKNOWN;
  // No use -->JO
  CTAGlobal.LastCycleStates.bFCTAFunctionActive = FALSE;
  CTAGlobal.LastCycleStates.bRCTAFunctionActive = FALSE;
  CTAGlobal.LastCycleStates.bEgoSpeedConditionFCTA = FALSE;
  CTAGlobal.LastCycleStates.CTAState = CTA_INIT;
  // Initialize CTA related information
  CTAInitObjects();

  RCTAReset();
  FCTAReset();
}

/*****************************************************************************
  Functionname:                                     */ /*!

  @brief

  @description

  @param[in]

  @param[out]

  @return
*****************************************************************************/
void CTAInitObjects() {
  for (uint8 uObj = 0U; uObj < CTA_MAX_NUM_OBJECTS; uObj++) {
    // Clear all CTA general object related information
    CTAGlobal.CTAObjectList[uObj].fTTCAccel_s = TUE_C_F32_VALUE_INVALID;
    CTAGlobal.CTAObjectList[uObj].fTTC_s = TUE_C_F32_VALUE_INVALID;
    CTAGlobal.CTAObjectList[uObj].fTTCFiltered_s = TUE_C_F32_VALUE_INVALID;
    CTAGlobal.CTAObjectList[uObj].fTTCRadial_s = TUE_C_F32_VALUE_INVALID;
    CTAGlobal.CTAObjectList[uObj].fCycleTimeSum_s = 0.F;
    CTAGlobal.CTAObjectList[uObj].fUpdateRate_nu = 0.74999F;
    // 0.5 ----> 0.74999
    CTAGlobal.CTAObjectList[uObj].fAssocProbFiltered_nu = 0.999999F;
    // 0.5 --> 0.99999; no data input from perception,and git it a value that
    // makes the condition true;
    CTAGlobal.CTAObjectList[uObj].fAngle_deg = 0.F;
    CTAGlobal.CTAObjectList[uObj].fObjLengthMax_met = 0.F;
    CTAGlobal.CTAObjectList[uObj].fObjWidthMax_met = 0.F;
    // CTAGlobal.CTAObjectList[uObj].fRangeRadial_met = TUE_C_F32_VALUE_INVALID;
    CTAGlobal.CTAObjectList[uObj].fVabs = 0.F;
    CTAGlobal.CTAObjectList[uObj].ObjectBorder.fXMin_met = 0.F;
    CTAGlobal.CTAObjectList[uObj].ObjectBorder.fXMax_met = 0.F;
    CTAGlobal.CTAObjectList[uObj].ObjectBorder.fYMin_met = 0.F;
    CTAGlobal.CTAObjectList[uObj].ObjectBorder.fYMax_met = 0.F;
    CTAGlobal.CTAObjectList[uObj].fXLastCycle_met = TUE_C_F32_VALUE_INVALID;
    CTAGlobal.CTAObjectList[uObj].fYLastCycle_met = TUE_C_F32_VALUE_INVALID;
    CTAGlobal.CTAObjectList[uObj].fVxPosBased_mps = TUE_C_F32_VALUE_INVALID;
    CTAGlobal.CTAObjectList[uObj].fVyPosBased_mps = TUE_C_F32_VALUE_INVALID;
    CTAGlobal.CTAObjectList[uObj].ObjectMovementBorder.fXMin_met =
        TUE_C_F32_VALUE_INVALID;
    CTAGlobal.CTAObjectList[uObj].ObjectMovementBorder.fXMax_met =
        TUE_C_F32_VALUE_INVALID;
    CTAGlobal.CTAObjectList[uObj].ObjectMovementBorder.fYMin_met =
        TUE_C_F32_VALUE_INVALID;
    CTAGlobal.CTAObjectList[uObj].ObjectMovementBorder.fYMax_met =
        TUE_C_F32_VALUE_INVALID;
    CTAGlobal.CTAObjectList[uObj].ObjectRotated.fDistX =
        TUE_C_F32_VALUE_INVALID;
    CTAGlobal.CTAObjectList[uObj].ObjectRotated.fDistY =
        TUE_C_F32_VALUE_INVALID;
    CTAGlobal.CTAObjectList[uObj].ObjectRotated.fLength =
        TUE_C_F32_VALUE_INVALID;
    CTAGlobal.CTAObjectList[uObj].ObjectRotated.fWidth =
        TUE_C_F32_VALUE_INVALID;
    CTAGlobal.CTAObjectList[uObj].fXMovement_met = 0.F;
    CTAGlobal.CTAObjectList[uObj].fYMovement_met = 0.F;
    // CTAGlobal.CTAObjectList[uObj].uUniqueID = TUE_C_UI16_VALUE_INVALID;
    // CTAGlobal.CTAObjectList[uObj].uLastMergedObjID = TUE_C_UI8_VALUE_INVALID;
    // CTAGlobal.CTAObjectList[uObj].bLowTTCAtState = FALSE;
    // CTAGlobal.CTAObjectList[uObj].bCreatedAdjStableObj = FALSE;
    // CTAGlobal.CTAObjectList[uObj].bObjValidForSelection = FALSE;
    // CTAGlobal.CTAObjectList[uObj].bPriolObject = FALSE;
    // CTAGlobal.CTAObjectList[uObj].fSpeedFiltered = 0.F;

    // Initialize CTA CT object
    CTAGlobal.CTObjectList[uObj].fXBreakthrough_met = TUE_C_F32_VALUE_INVALID;
    CTAGlobal.CTObjectList[uObj].fXBreakthroughFiltered_met =
        TUE_C_F32_VALUE_INVALID;
    CTAGlobal.CTObjectList[uObj].fXBreakthroughStd_met =
        TUE_C_F32_VALUE_INVALID;
    CTAGlobal.CTObjectList[uObj].fTTC_s = TUE_C_F32_VALUE_INVALID;
    CTAGlobal.CTObjectList[uObj].fTTCFiltered_s = TUE_C_F32_VALUE_INVALID;
    CTAGlobal.CTObjectList[uObj].fDistToCrossingLine_met =
        TUE_C_F32_VALUE_INVALID;
    CTAGlobal.CTObjectList[uObj].fDistToCrossingLineFiltered_met =
        TUE_C_F32_VALUE_INVALID;
    // CTAGlobal.CTObjectList[uObj].fObjBreakthroughMargin_met =
    // TUE_C_F32_VALUE_INVALID;
    CTAGlobal.CTObjectList[uObj].fRearTrackProb_per =
        CTA_FCTA_REARTRACK_INIT_PROB;  // 0.3
    CTAGlobal.CTObjectList[uObj].bRearTrack_nu = FALSE;
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
void CTA_Exec(const CTAInReq_t* reqPorts, const CTAParam_t* params,
              CTAOutPro_t* proPorts, CTADebug_t* CTAdebugInfo) {
  if (CTAGlobal.eCTAState == CTA_OK) {
    memset(&FCTAproPorts, 0, sizeof(FCTAOutPro_t));  //NOLINT
    memset(&RCTAproPorts, 0, sizeof(RCTAOutPro_t));  //NOLINT
    // PreProcess
    CTAPreProcess(reqPorts, params, &CTAGlobal);

    // FCTA: Front Cross Traffic Alert
    CTAToFCTAInputWrapper(reqPorts, params, &CTAGlobal, &FCTAreqPorts,
                          &FCTAparams);
    FCTAExec(&FCTAreqPorts, &FCTAparams, &FCTAproPorts, &FCTAdebugInfo);
    FCTAToCTAOutputWrapper(&FCTAproPorts, &FCTAdebugInfo, &CTAGlobal,
                           CTAdebugInfo);

    // RCTA: Rear Cross Traffic Alert
    CTAToRCTAInputWrapper(reqPorts, params, &CTAGlobal, &RCTAreqPorts,
                          &RCTAparams);
    RCTAExec(&RCTAreqPorts, &RCTAparams, &RCTAproPorts, &RCTAdebugInfo);
    RCTAToCTAOutputWrapper(&RCTAproPorts, &RCTAdebugInfo, &CTAGlobal,
                           CTAdebugInfo);

    // ProProcess
    CTAProProcess(reqPorts, params, &CTAGlobal, proPorts);

  } else {
    CTA_Reset();
    CTAGlobal.eCTAState = CTA_OK;
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
void CTAPreProcess(const CTAInReq_t* reqPorts, const CTAParam_t* params,
                   CTAGlobal_t* pCTAGlobal) {
  // gather global properties needed for CTA

  CTACalculateGlobalProperties(&params->CTA_Ks_VehicleParameter_nu,
                               &pCTAGlobal->fSensorOffsetToSide_met,
                               &pCTAGlobal->fSensorOffsetToRear_met);
  // gather general object properties needed for CTA
  CTACalculateObjectProperties(reqPorts, params, pCTAGlobal);
  // gather general object properties needed for CTA CT
  CTACalculateCTObjectProperties(reqPorts, params, pCTAGlobal);
}

/*****************************************************************************
  Functionname:CTACalculateGlobalProperties */ /*!

  @brief Calculate global CTA properties

  @description Calculate global CTA properties

  @param[in]  pVehicleParameter: Vehicle parameter structure

  @param[out] pfSensorOffsetToSide_met: the offset from sensor mounting position to the side edge of vehicle
			  pfSensorOffsetToRear_met: the offset from sensor mounting position to the front or rear edge of vehicle

  @return
*****************************************************************************/
void CTACalculateGlobalProperties(
    const CTAVehicleParam_t* pVehicleParameter,
    SensorMountingPosGlobal_t* pfSensorOffsetToSide_met,
    SensorMountingPosGlobal_t* pfSensorOffsetToRear_met) {
  // Process vehicle parameters from RTE. This is additional to init processing,
  // to make sure that SW signal delays do not cause issues with setting the
  // values correctly calculate the Offset between sensor and side
  pfSensorOffsetToSide_met->fLeftFrontPos_met =
      0.5F * pVehicleParameter->CTA_Kf_VehicleWidth_met -
      pVehicleParameter->CTA_Ks_LeftFrontSensorMounting.CTA_Kf_LatPos_met;

  pfSensorOffsetToSide_met->fRightFrontPos_met =
      -0.5F * pVehicleParameter->CTA_Kf_VehicleWidth_met -
      pVehicleParameter->CTA_Ks_RightFrontSensorMounting.CTA_Kf_LatPos_met;

  pfSensorOffsetToSide_met->fLeftRearPos_met =
      0.5F * pVehicleParameter->CTA_Kf_VehicleWidth_met -
      pVehicleParameter->CTA_Ks_LeftRearSensorMounting_nu.CTA_Kf_LatPos_met;

  pfSensorOffsetToSide_met->fRightRearPos_met =
      -0.5F * pVehicleParameter->CTA_Kf_VehicleWidth_met -
      pVehicleParameter->CTA_Ks_RightRearSensorMounting_nu.CTA_Kf_LatPos_met;

  // calculate the Offset between sensor and front/rear side
  pfSensorOffsetToRear_met->fLeftFrontPos_met =
      pVehicleParameter->CTA_Ks_LeftFrontSensorMounting.CTA_Kf_LongPos_met -
      pVehicleParameter->CTA_Kf_OverhangFront_met;

  // pfSensorOffsetToRear_met->fLeftFrontPos_met =
  //     TUE_CML_MinMax(-0.5F, -0.1F,
  //     pfSensorOffsetToRear_met->fLeftFrontPos_met);

  pfSensorOffsetToRear_met->fRightFrontPos_met =
      pVehicleParameter->CTA_Ks_RightFrontSensorMounting.CTA_Kf_LongPos_met -
      pVehicleParameter->CTA_Kf_OverhangFront_met;

  // pfSensorOffsetToRear_met->fRightFrontPos_met = TUE_CML_MinMax(
  //     -0.5F, -0.1F, pfSensorOffsetToRear_met->fRightFrontPos_met);

  pfSensorOffsetToRear_met->fLeftRearPos_met =
      pVehicleParameter->CTA_Kf_VehicleLength_met -
      pVehicleParameter->CTA_Kf_OverhangFront_met +
      pVehicleParameter->CTA_Ks_LeftRearSensorMounting_nu.CTA_Kf_LongPos_met;

  // pfSensorOffsetToRear_met->fLeftRearPos_met =
  //     TUE_CML_MinMax(0.05F, 0.35F,
  //     pfSensorOffsetToRear_met->fLeftRearPos_met);

  pfSensorOffsetToRear_met->fRightRearPos_met =
      pVehicleParameter->CTA_Kf_VehicleLength_met -
      pVehicleParameter->CTA_Kf_OverhangFront_met +
      pVehicleParameter->CTA_Ks_RightRearSensorMounting_nu.CTA_Kf_LongPos_met;

  // pfSensorOffsetToRear_met->fRightRearPos_met =
  //     TUE_CML_MinMax(0.05F, 0.35F,
  //     pfSensorOffsetToRear_met->fRightRearPos_met);
}

/*****************************************************************************
  Functionname:                                     */ /*!

  @brief

  @description

  @param[in]

  @param[out]

  @return
*****************************************************************************/
void CTACalculateObjectProperties(const CTAInReq_t* reqPorts,
                                  const CTAParam_t* params,
                                  CTAGlobal_t* pCTAGlobal) {
  for (uint8 uObj = 0U; uObj < CTA_MAX_NUM_OBJECTS; uObj++) {
    const EMSRRObjectInReq_t* pEMSRRObjInput = &reqPorts->CTAEMSRRObjList[uObj];
    CTAObjectInfoGlobal_t* pCTAObjGlobal = &pCTAGlobal->CTAObjectList[uObj];
    const EgoVehicleInReq_t* pEgoVehicleInput = &reqPorts->EgoVehicleInfo;
    unsigned temp = pEMSRRObjInput->uiMaintenanceState_nu;
    if (temp) {
      // calculate the borders in which an object has moved during its lifetime
      CTACalculateObjectMovementBorders(
          pEMSRRObjInput->fDistX_met, pEMSRRObjInput->fDistY_met,
          &pCTAObjGlobal->ObjectMovementBorder, &pCTAObjGlobal->fXMovement_met,
          &pCTAObjGlobal->fYMovement_met);
      // calculate object quality related qualifies
      CTACalculateObjectQualifiers(pEMSRRObjInput->uiHighestAssocProb_per,
                                   pEMSRRObjInput->uiMaintenanceState_nu,
                                   &pCTAObjGlobal->fUpdateRate_nu,
                                   &pCTAObjGlobal->fAssocProbFiltered_nu);
      // calculate vx and vy based on the change in position of the current
      // object
      CTACalculatePosBasedVxVy(
          reqPorts->fCycleTime_s, pEMSRRObjInput,
          pCTAObjGlobal->fXLastCycle_met, pCTAObjGlobal->fYLastCycle_met,
          &pCTAObjGlobal->fVxPosBased_mps, &pCTAObjGlobal->fVyPosBased_mps);
      // calculate the absolute object velocity
      CTACalculateAbsoluteObjectVelocity(
          pEMSRRObjInput, params, pEgoVehicleInput, &pCTAObjGlobal->fVabs);
    }
  }
}

/*****************************************************************************
  Functionname:CTACalculateObjectMovementBorders */ /*!

  @brief:Calculates the x and y borders in which an object has moved during its lifetime

  @description:Calculates the x and y borders in which an object has moved during its lifetime

  @param[in]  fDistX_met:Object's longitudinal relative distance
              fDistY_met:Object's lateral relative distance

  @param[out] pObjectMovementBorder:The object movement border information
              fXMovement_met:The object total moving distance in the x direction,unit:m
			  fYMovement_met:The object total moving distance in the y direction,unit:m
  @return
*****************************************************************************/
void CTACalculateObjectMovementBorders(float32 fDistX_met, float32 fDistY_met,
                                       CTAObjectBorder_t* pObjectMovementBorder,
                                       float32* fXMovement_met,
                                       float32* fYMovement_met) {
  if (pObjectMovementBorder->fXMin_met > TUE_C_F32_VALUE_INVALID - 1.0F) {
    pObjectMovementBorder->fXMin_met = fDistX_met;
    pObjectMovementBorder->fXMax_met = fDistX_met;
    pObjectMovementBorder->fYMin_met = fDistY_met;
    pObjectMovementBorder->fYMax_met = fDistY_met;
  } else {
    // Calculate the bounds of the longitudinal motion
    if (fDistX_met > pObjectMovementBorder->fXMax_met) {
      pObjectMovementBorder->fXMax_met = fDistX_met;
      *fXMovement_met =
          pObjectMovementBorder->fXMax_met - pObjectMovementBorder->fXMin_met;
    } else {
      if (fDistX_met < pObjectMovementBorder->fXMin_met) {
        pObjectMovementBorder->fXMin_met = fDistX_met;
        *fXMovement_met =
            pObjectMovementBorder->fXMax_met - pObjectMovementBorder->fXMin_met;
      }
    }

    // Calculate the bounds of the lateral motion
    if (fDistY_met > pObjectMovementBorder->fYMax_met) {
      pObjectMovementBorder->fYMax_met = fDistY_met;
      *fYMovement_met =
          pObjectMovementBorder->fYMax_met - pObjectMovementBorder->fYMin_met;
    } else {
      if (fDistY_met < pObjectMovementBorder->fYMin_met) {
        pObjectMovementBorder->fYMin_met = fDistY_met;
        *fYMovement_met =
            pObjectMovementBorder->fYMax_met - pObjectMovementBorder->fYMin_met;
      }
    }
  }
}

/*****************************************************************************
  Functionname:CTACalculateObjectQualifiers */ /*!

  @brief:Calculates object quality related qualifiers

  @description:Calculates object quality related qualifiers,update rate,association probability filtered

  @param[in]  uiHighestAssocProb_per:Highest cluster association probability of the object filter result
              uiMaintenanceState_nu:the maintenance(measured,predicted) state whether the object is deleted
  @param[out] pfUpdateRate_nu:The object measurement update rate,unit:NULL
              pfAssocProbFiltered_nu:Filtered highest cluster association probability of the object filter result
  @return
*****************************************************************************/
void CTACalculateObjectQualifiers(uint8 uiHighestAssocProb_per,
                                  uint8 uiMaintenanceState_nu,
                                  float32* pfUpdateRate_nu,
                                  float32* pfAssocProbFiltered_nu) {
  float32 fObjMeadured = 0.F;
  float32 fFilterConst = 0.F;
  const float32 fHighestAssocProb = (float32)uiHighestAssocProb_per * 0.01F;
  /**********************************************************************
   *Filter update rate
   **********************************************************************/
  if (uiMaintenanceState_nu == CTA_EM_GEN_OBJECT_MT_STATE_MEASURED) {
    fObjMeadured = 1.0F;
    fFilterConst = CTA_UPDATERATE_FILTER_UP;  // 0.05
  } else {
    fObjMeadured = 0.0F;
    fFilterConst = CTA_UPDATERATE_FILTER_DOWN;  // 0.025
  }

  GDB_Math_LowPassFilter(pfUpdateRate_nu, fObjMeadured, fFilterConst);

  /**********************************************************************
   *Filter highest association probability
   **********************************************************************/
  // Now not to use uiHighestAssocProb_per because miss the AssocProb data
  if (*pfAssocProbFiltered_nu < fHighestAssocProb) {
    fFilterConst = CTA_ASSOCPROB_FILTER_UP;  // 0.05
  } else {
    fFilterConst = CTA_ASSOCPROB_FILTER_DOWN;  // 0.05
  }
  GDB_Math_LowPassFilter(pfAssocProbFiltered_nu, fHighestAssocProb,
                         fFilterConst);
}

/*****************************************************************************
  Functionname:CTACalculatePosBasedVxVy                                     */ /*!

  @brief Calculate a filtered VrelX and VrelY based on the change in position of the object

  @description Calculate a filtered VrelX and VrelY based on the change in position of the object

  @param[in]  fCycleTime_s:Current task cycle time from EMGlobalOutput
              pEMSRRObjInput:EM Side radar objects information
			  fXLastCycle_met:DistX of last cycle
			  fYLastCycle_met:DistY of last cycle
  @param[out] pfVxPosBased_mps:the VrelX based on the position of current and last cycle
              pfVyPosBased_mps:the VrelY based on the position of current and last cycle
  @return
*****************************************************************************/
void CTACalculatePosBasedVxVy(float32 fCycleTime_s,
                              const EMSRRObjectInReq_t* pEMSRRObjInput,
                              float32 fXLastCycle_met, float32 fYLastCycle_met,
                              float32* pfVxPosBased_mps,
                              float32* pfVyPosBased_mps) {
  // Check if the current cycle time is valid and if a value for the position of
  // the last cycle has already been set
  if (fXLastCycle_met > (TUE_C_F32_VALUE_INVALID - TUE_C_F32_DELTA) ||
      fCycleTime_s < TUE_C_F32_DELTA) {
    *pfVxPosBased_mps = pEMSRRObjInput->fVrelX_mps;
    *pfVyPosBased_mps = pEMSRRObjInput->fVrelY_mps;
  } else {
    // Calculate a filtered vrelx and vrely based on the change in position of
    // the object
    GDB_Math_LowPassFilter(
        pfVxPosBased_mps,
        (pEMSRRObjInput->fDistX_met - fXLastCycle_met) / fCycleTime_s,
        CTA_LPF_VRELXY_ALPHA);
    GDB_Math_LowPassFilter(
        pfVyPosBased_mps,
        (pEMSRRObjInput->fDistY_met - fYLastCycle_met) / fCycleTime_s,
        CTA_LPF_VRELXY_ALPHA);
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
void CTACalculateAbsoluteObjectVelocity(
    const EMSRRObjectInReq_t* pEMSRRObjInput, const CTAParam_t* params,
    const EgoVehicleInReq_t* pEgoVehicleInput, float32* fVabs) {
  // float32 fSensorLongPos;
  // float32 fSensorLatPos;
  // float32 fSensorLongOffset;
  // const float32 fOverhangFront = 1.F;  // Todo: platform dependent, to be
  //                                      // provided by VehPar or algorithm
  //                                      param
  // const float32 fOverhangRear = 0.9F;  // Todo: platform dependent, to be
  //                                      // provided by VehPar or algorithm
  //                                      param
  float32 fVxAbs = 0.F;
  float32 fVyAbs = 0.F;
  float32 fVabsObj = 0.F;
  // float32 fSensorOffsetToRearAxle;

  // // calculate Vabs
  // if (pEMSRRObjInput->eSensorMountingPos == LeftFrontPos ||
  //     pEMSRRObjInput->eSensorMountingPos == RightFrontPos) {
  //   if (pEMSRRObjInput->eSensorMountingPos == LeftFrontPos) {
  //     fSensorLongOffset =
  //         params->CTA_Ks_VehicleParameter_nu.CTA_Ks_LeftFrontSensorMounting
  //             .CTA_Kf_LongPos_met;
  //     fSensorLatPos = params->CTA_Ks_VehicleParameter_nu
  //                         .CTA_Ks_LeftFrontSensorMounting.CTA_Kf_LatPos_met;
  //   } else {
  //     fSensorLongOffset =
  //         params->CTA_Ks_VehicleParameter_nu.CTA_Ks_RightFrontSensorMounting
  //             .CTA_Kf_LongPos_met;
  //     fSensorLatPos = params->CTA_Ks_VehicleParameter_nu
  //                         .CTA_Ks_RightFrontSensorMounting.CTA_Kf_LatPos_met;
  //   }
  //   fSensorLongPos =
  //       fABS(fOverhangFront +
  //            params->CTA_Ks_VehicleParameter_nu.CTA_Kf_WheelBase_met +
  //            fSensorLongOffset);

  //   // fVxAbs = pEgoVehicleInput->fegoVelocity_mps +
  //   pEMSRRObjInput->fVrelX_mps
  //   // -
  //   //          pEgoVehicleInput->fYawRate_radps *
  //   //              (pEMSRRObjInput->fDistY_met + fSensorLatPos);
  //   //----SENTIME

  //   fVxAbs = pEMSRRObjInput->fVrelX_mps;  // JO

  //   fVyAbs = pEMSRRObjInput->fVrelY_mps;  // JO
  //   // fVyAbs = pEMSRRObjInput->fVrelY_mps +
  //   //          pEgoVehicleInput->fYawRate_radps *
  //   //              (pEMSRRObjInput->fDistX_met + fSensorLongPos);
  //   //----SENTIME
  // } else {
  //   if (pEMSRRObjInput->eSensorMountingPos == LeftRearPos) {
  //     fSensorLongOffset =
  //         params->CTA_Ks_VehicleParameter_nu.CTA_Ks_LeftRearSensorMounting_nu
  //             .CTA_Kf_LongPos_met;
  //     fSensorLatPos = params->CTA_Ks_VehicleParameter_nu
  //                         .CTA_Ks_LeftRearSensorMounting_nu.CTA_Kf_LatPos_met;
  //   } else {
  //     fSensorLongOffset =
  //         params->CTA_Ks_VehicleParameter_nu.CTA_Ks_RightRearSensorMounting_nu
  //             .CTA_Kf_LongPos_met;
  //     fSensorLatPos = params->CTA_Ks_VehicleParameter_nu
  //                         .CTA_Ks_RightRearSensorMounting_nu.CTA_Kf_LatPos_met;
  //   }
  //   fSensorOffsetToRearAxle = fOverhangRear - fSensorLongOffset;

  //   fVxAbs = pEgoVehicleInput->fegoVelocity_mps + pEMSRRObjInput->fVrelX_mps
  //   -
  //            pEgoVehicleInput->fYawRate_radps *
  //                (pEMSRRObjInput->fDistY_met + fSensorLatPos);
  //   fVyAbs = pEMSRRObjInput->fVrelY_mps +
  //            pEgoVehicleInput->fYawRate_radps *
  //                (pEMSRRObjInput->fDistX_met - fSensorOffsetToRearAxle);
  // }
  if (pEMSRRObjInput->eDynamicProperty_nu ==
          CTA_EM_GEN_OBJECT_DYN_PROPERTY_STATIONARY ||
      pEMSRRObjInput->eDynamicProperty_nu ==
          CTA_EM_GEN_OBJECT_DYN_PROPERTY_STOPPED) {
    *fVabs = 0.F;
  } else {
    fVxAbs = pEMSRRObjInput->fVabsX_mps;         // JO
    fVyAbs = pEMSRRObjInput->fVabsY_mps;         // JO
    fVabsObj = sqrt(SQR(fVxAbs) + SQR(fVyAbs));  /////  SENTIME  WHY?
    *fVabs = fVabsObj;
  }
}

/*****************************************************************************
  Functionname:CTACalculateCTObjectProperties */ /*!

  @brief

  @description

  @param[in]

  @param[out]

  @return
*****************************************************************************/
void CTACalculateCTObjectProperties(const CTAInReq_t* reqPorts,
                                    const CTAParam_t* params,
                                    CTAGlobal_t* pCTAGlobal) {
  // init all cyclic variables
  CTA_CTInitCyclic(pCTAGlobal);
  // calculate the maximum lateral sensor range
  CTA_CTCalculateMaxLatSensorRange(reqPorts->CTAEMSRRObjList,
                                   reqPorts->EgoVehicleInfo.fegoVelocity_mps,
                                   &pCTAGlobal->CTGlobals.fMaxLatSensorRange);

  for (uint8 uObj = 0U; uObj < CTA_MAX_NUM_OBJECTS; uObj++) {
    CTObjectInfoGlobal_t* pCTObjGlobal = &pCTAGlobal->CTObjectList[uObj];
    const EMSRRObjectInReq_t* pEMSRRObjInput = &reqPorts->CTAEMSRRObjList[uObj];

    if (reqPorts->CTAEMSRRObjList[uObj].uiMaintenanceState_nu) {
      switch (reqPorts->CTAEMSRRObjList[uObj].uiMeasuredTargetFrequency_nu) {
        case 255:
          CTAGlobal.CTAObjectList[uObj].fUpdateRate_nu = 0.999F;
          break;
        case 128:
          CTAGlobal.CTAObjectList[uObj].fUpdateRate_nu = 0.96F;
          break;
        case 64:
          CTAGlobal.CTAObjectList[uObj].fUpdateRate_nu = 0.91F;
          break;

        default:
          CTAGlobal.CTAObjectList[uObj].fUpdateRate_nu = 0.74999F;
          break;
      }

      // Calculate distance to crossing line
      CTA_CTCalculateDistToCrossingLine(
          reqPorts->fCycleTime_s, pEMSRRObjInput,
          &pCTAGlobal->fSensorOffsetToSide_met,
          &pCTObjGlobal->fDistToCrossingLine_met,
          &pCTObjGlobal->fDistToCrossingLineFiltered_met);
      // Calculate x-axis breakthrough
      CTA_CTCalculateXBreakthrough(pEMSRRObjInput,
                                   &params->CTA_Ks_VehicleParameter_nu,
                                   pCTObjGlobal->fDistToCrossingLine_met,
                                   &pCTObjGlobal->fXBreakthrough_met,
                                   &pCTObjGlobal->fXBreakthroughStd_met,
                                   &pCTObjGlobal->fXBreakthroughFiltered_met);
      // Calculate Time to crossing
      CTA_CTCalculateTTC(
          reqPorts->fCycleTime_s, pEMSRRObjInput->eSensorMountingPos,
          pEMSRRObjInput->fVrelY_mps, pCTObjGlobal->fDistToCrossingLine_met,
          &pCTObjGlobal->fTTC_s, &pCTObjGlobal->fTTCFiltered_s);
      // Calculate the rear object probability
      CTA_CTCalculateRearObjectProbability(uObj, pEMSRRObjInput, reqPorts,
                                           pCTAGlobal,
                                           &pCTObjGlobal->fRearTrackProb_per);
      // Check the rear object
      CTA_CTCheckRearObject(pEMSRRObjInput->uClassification_nu,
                            pCTObjGlobal->fRearTrackProb_per,
                            &pCTObjGlobal->bRearTrack_nu);
    }
  }
}

/*****************************************************************************
  Functionname:CTA_CTInitCyclic                                     */ /*!

  @brief:Initialize variables which are not stored across cycles

  @description:Initialize variables which are not stored across cycles

  @param[in]

  @param[out]

  @return
*****************************************************************************/
void CTA_CTInitCyclic(CTAGlobal_t* pCTAGlobal) {
  // TODO(wenqing): use same properties for RCTA and FCTA
  // init multi object handling
  pCTAGlobal->CTGlobals.uCriticalObjIDLastCycle =
      pCTAGlobal->CTGlobals.uCriticalObjID;
  pCTAGlobal->CTGlobals.uCriticalObjID = TUE_C_UI8_VALUE_INVALID;
  pCTAGlobal->CTGlobals.fCriticalObjDistYLastCycle =
      pCTAGlobal->CTGlobals.fCriticalObjDistY;
  pCTAGlobal->CTGlobals.fCriticalObjDistY = TUE_C_F32_VALUE_INVALID;
  pCTAGlobal->CTGlobals.fCriticalTTC = TUE_C_F32_VALUE_INVALID;
}

/*****************************************************************************
  Functionname:CTA_CTCalculateMaxLatSensorRange */ /*!

  @brief: calculate the current maximum lateral sensor range based on crossing object information

  @description: calculate the current maximum lateral sensor range based on crossing object information

  @param[in]

  @param[out]

  @return
*****************************************************************************/
void CTA_CTCalculateMaxLatSensorRange(
    const EMSRRObjectInReq_t* pCTAEMSRRObjList, float32 fegoVelocity_mps,
    float32* pfMaxLatSensorRange) {
  float32 fDistY = 0.F;
  float32 fVrelY = 0.F;
  float32 fMaxLatSensorRange = *pfMaxLatSensorRange;
  if (fegoVelocity_mps < CTA_CT_MAXLATERALRANGE_MIN_EGOSPEED) {
    if (*pfMaxLatSensorRange > (TUE_C_F32_VALUE_INVALID - TUE_C_F32_DELTA)) {
      *pfMaxLatSensorRange = 0.F;
    }
    // Loop over all objects
    for (uint8 uObj = 0U; uObj < CTA_MAX_NUM_OBJECTS; uObj++) {
      if ((!pCTAEMSRRObjList[uObj].uiMaintenanceState_nu) ==
          CTA_EM_GEN_OBJECT_MT_STATE_DELETED) {
        fDistY = pCTAEMSRRObjList[uObj].fDistY_met;
        fVrelY = pCTAEMSRRObjList[uObj].fVrelY_mps;
        // check if on right sensor, revert the sign of the Y distance for using
        // the old coordinate system
        if (pCTAEMSRRObjList[uObj].eSensorMountingPos == RightFrontPos ||
            pCTAEMSRRObjList[uObj].eSensorMountingPos == RightRearPos) {
          fDistY = fDistY * -1.F;
          fVrelY = fVrelY * -1.F;
        }
        if (fVrelY < CTA_CT_MAXLATRANGE_MAX_VRELX &&
            pCTAEMSRRObjList[uObj].fDistX_met < CTA_CT_MAXLATRANGE_MAX_DISTX &&
            fDistY < CTA_CT_MAXLATRANGE_MAX_YDIST &&
            fDistY > CTA_CT_MAXLATRANGE_MIN_YDIST &&
            pCTAEMSRRObjList[uObj].fProbabilityOfExistence_per >
                CTA_CT_MAXLATRANGE_MIN_POE) {
          if (pCTAEMSRRObjList[uObj].fFirstDetectY_met > *pfMaxLatSensorRange) {
            fMaxLatSensorRange = pCTAEMSRRObjList[uObj].fFirstDetectY_met;
          }
        }
      }
    }
  } else {
    fMaxLatSensorRange = TUE_C_F32_VALUE_INVALID;
  }

  *pfMaxLatSensorRange = fMaxLatSensorRange;
}

/*****************************************************************************
  Functionname:CTA_CTCalculateDistToCrossingLine */ /*!

  @brief:Calculate the distance of the object to the crossing line

  @description:Calculate the distance of the object to the crossing line

  @param[in]  pEMSRRObjInput:EM side radar object information structure

  @param[out]

  @return
*****************************************************************************/
void CTA_CTCalculateDistToCrossingLine(
    float32 fCycleTime_s, const EMSRRObjectInReq_t* pEMSRRObjInput,
    SensorMountingPosGlobal_t* pfSensorOffsetToSide_met,
    float32* pfDistToCrossingLine_met,
    float32* pfDistToCrossingLineFiltered_met) {
  float32 fDistToCrossingLine = 0.F;
  float32 fClosestDistY = 0.F;
  float32 fDistYFLCorner = 0.F;
  float32 fDistYFRCorner = 0.F;
  float32 fSensorOffsetToSide = 0.F;

  // Reference point of object to calculate distance to crossing line is the
  // closest of the two front corners to crossing
  // line because objects are now rotated
  fDistYFLCorner = pEMSRRObjInput->fDistY_met +
                   pEMSRRObjInput->fLengthFront_met *
                       SIN_(pEMSRRObjInput->fAbsOrientation_rad) +
                   pEMSRRObjInput->fWidthLeft_met *
                       COS_(pEMSRRObjInput->fAbsOrientation_rad);
  fDistYFRCorner = pEMSRRObjInput->fDistY_met +
                   pEMSRRObjInput->fLengthFront_met *
                       SIN_(pEMSRRObjInput->fAbsOrientation_rad) -
                   pEMSRRObjInput->fWidthRight_met *
                       COS_(pEMSRRObjInput->fAbsOrientation_rad);

  if (pEMSRRObjInput->eSensorMountingPos == LeftFrontPos ||
      pEMSRRObjInput->eSensorMountingPos == LeftRearPos) {
    // On the left sensor the closest corner always has smallest Y distance
    // between the two front corners
    fClosestDistY = MIN(fDistYFLCorner, fDistYFRCorner);
  } else {
    // On the left sensor the closest corner always has smallest Y distance
    // between the two front corners
    fClosestDistY = MAX(fDistYFLCorner, fDistYFRCorner);
    // Revert the sign of the Y distance for using the old coordinate system
    fClosestDistY = fClosestDistY * -1.F;
  }

  switch (pEMSRRObjInput->eSensorMountingPos) {
    case LeftFrontPos:
      fSensorOffsetToSide = pfSensorOffsetToSide_met->fLeftFrontPos_met;
      break;
    case RightFrontPos:
      fSensorOffsetToSide = pfSensorOffsetToSide_met->fRightFrontPos_met;
      break;
    case LeftRearPos:
      fSensorOffsetToSide = pfSensorOffsetToSide_met->fLeftRearPos_met;
      break;
    case RightRearPos:
      fSensorOffsetToSide = pfSensorOffsetToSide_met->fRightRearPos_met;
      break;
    default:
      fSensorOffsetToSide = 0.F;
      break;
  }
  fDistToCrossingLine = fClosestDistY - fABS(fSensorOffsetToSide);
  *pfDistToCrossingLine_met = fDistToCrossingLine;
  *pfDistToCrossingLineFiltered_met = fDistToCrossingLine;

  if (pEMSRRObjInput->eSensorMountingPos == LeftFrontPos ||
      pEMSRRObjInput->eSensorMountingPos == RightFrontPos) {
    if (*pfDistToCrossingLineFiltered_met <
        TUE_C_F32_VALUE_INVALID - TUE_C_F32_DELTA) {
      if (pEMSRRObjInput->fVrelY_mps * pEMSRRObjInput->fDistY_met > 0.0) {
        *pfDistToCrossingLineFiltered_met +=
            fABS(pEMSRRObjInput->fVrelY_mps * fCycleTime_s);
      } else {
        *pfDistToCrossingLineFiltered_met -=
            fABS(pEMSRRObjInput->fVrelY_mps * fCycleTime_s);
      }
      TUE_CML_LowPassFilter(pfDistToCrossingLineFiltered_met,
                            *pfDistToCrossingLine_met,
                            CTA_CT_LPF_DIST2CROSS_FILTER);
    } else {
      *pfDistToCrossingLineFiltered_met = *pfDistToCrossingLine_met;
    }
  }
}

/*****************************************************************************
  Functionname:CTA_CTCalculateXBreakthrough */ /*!

  @brief:Calculate y-axis breakthrough of this object

  @description:Calculate y-axis breakthrough of this object, also check its start position

  @param[in]  pEMSRRObjInput:EM side radar object information structure
			  fDistToCrossingLine_met:Distance to crossing line

  @param[out] pfXBreakthrough_met
              pfXBreakthroughStd_met
              pfXBreakthroughFiltered_met
  @return
*****************************************************************************/
void CTA_CTCalculateXBreakthrough(const EMSRRObjectInReq_t* pEMSRRObjInput,
                                  const CTAVehicleParam_t* pVehicleParameter,
                                  float32 fDistToCrossingLine_met,
                                  float32* pfXBreakthrough_met,
                                  float32* pfXBreakthroughStd_met,
                                  float32* pfXBreakthroughFiltered_met) {
  float32 fHeading = pEMSRRObjInput->fRelHeading_rad;
  float32 fHeadingStd = pEMSRRObjInput->fRelHeadingStd_rad;
  float32 fTanHeading = 0.F;
  float32 fXBreakthrough = TUE_C_F32_VALUE_INVALID;
  float32 fXBreakthroughStd = TUE_C_F32_VALUE_INVALID;
  float32 fXBreakthrough_fDistX = 0.F;
  float32 fXBreakthrough_fHeading = 0.F;
  float32 fXBreakthrough_fDistToCrossingLine = 0.F;

  // Check if on right sensor, revert the sign of the heading for using the old
  // coordinate system
  if (!(pEMSRRObjInput->eSensorMountingPos == RightFrontPos ||
        pEMSRRObjInput->eSensorMountingPos == RightRearPos)) {
    fHeading = -fHeading;  // *product
  }
  // For object which move towards the X axis calculate the breakthrough
  if (fHeading > TUE_C_F32_DELTA && fHeading < (TUE_CML_Pi - TUE_C_F32_DELTA)) {
    fTanHeading = TAN_(TUE_CML_Pi / 2.F - fHeading);  // cot
    // Take the object position
    fXBreakthrough = pEMSRRObjInput->fDistX_met;
    // Add the heading angle dependent part
    fXBreakthrough += fTanHeading * (fDistToCrossingLine_met);

    if (fTanHeading * fXBreakthrough < 0.F) {
      fXBreakthrough +=
          fTanHeading * pVehicleParameter->CTA_Kf_VehicleWidth_met;
    }

    if (fXBreakthrough > 0.F) {
      fXBreakthrough -= 0.5F * (pEMSRRObjInput->fWidthLeft_met +
                                pEMSRRObjInput->fWidthRight_met);
    } else {
      fXBreakthrough += 0.5F * (pEMSRRObjInput->fWidthRight_met +
                                pEMSRRObjInput->fWidthLeft_met);
    }

    // Calculate the Std
    /*The error propagation is calculated using following formula:
      q = f(x1,...,xn)
      the uncertainty is calculated:
      q_std = sqrt((dq/dx1 * x1_std)^2 + .... + (dq/dxn * xn_std)^2)*/
    // Calculate the partial derivatives of the breakthrough calculation formula
    fXBreakthrough_fDistX = 1.0F;
    fXBreakthrough_fHeading =
        (2.0F * fDistToCrossingLine_met) /
        SafeDiv(COS_(2.0F * (fHeading + TUE_CML_Pi)) + 1.0F);
    fXBreakthrough_fDistToCrossingLine = fTanHeading;
    // Calculate the square sum of the partial derivatives times the
    // corresponding standard deviation
    fXBreakthroughStd =
        SQR(fXBreakthrough_fDistToCrossingLine * pEMSRRObjInput->fDistYStd_met);
    // Calculate the root
    fXBreakthroughStd = sqrt(fXBreakthroughStd);
    // Limit the value to reasonable limits
    fXBreakthroughStd = MIN(fXBreakthroughStd, TUE_C_F32_VALUE_INVALID);
  }
  *pfXBreakthrough_met = fXBreakthrough;
  *pfXBreakthroughStd_met = fXBreakthroughStd;

  if (pEMSRRObjInput->eSensorMountingPos == LeftFrontPos ||
      pEMSRRObjInput->eSensorMountingPos == RightFrontPos) {
    // if the calculated breakthrough is not invalid
    if (*pfXBreakthrough_met < (TUE_C_F32_VALUE_INVALID - TUE_C_F32_DELTA)) {
      // In the first cycle the filtered breakthrough is set to the current
      // breakthrough
      if (*pfXBreakthroughFiltered_met > TUE_C_F32_VALUE_INVALID - 1.F) {
        *pfXBreakthroughFiltered_met = *pfXBreakthrough_met;
      } else {
        // Afterwards filter the value. The filter speed depends on the lifetime
        // of the object. => Young objects -> fast filter
        *pfXBreakthroughFiltered_met = TUE_CML_MacroLowPassFilter(
            *pfXBreakthrough_met, *pfXBreakthroughFiltered_met,
            TUE_CML_Min(pEMSRRObjInput->uiLifeCycles_nu,
                        CTA_CT_XBREAKTHROUGH_MIN_FILTER));  // 10
      }
    }
  }
}

/*****************************************************************************
  Functionname:CTA_CTCalculateTTC                                     */ /*!

  @brief:Calculate TTC to computed breakthrough

  @description:Calculate TTC to computed breakthrough using constant speed assumption

  @param[in]  fCycleTime_s:Current task cycle time from EMGlobalOutput
              eSensorMountingPos
			  fVrelY_mps
			  fDistToCrossingLine_met
  @param[out] pfTTC_s
              pfTTCFiltered_s

  @return
*****************************************************************************/
void CTA_CTCalculateTTC(float32 fCycleTime_s,
                        const eSensorMountingPos_t eSensorMountingPos,
                        const float32 fVrelY_mps,
                        float32 fDistToCrossingLine_met, float32* pfTTC_s,
                        float32* pfTTCFiltered_s) {
  float32 fVrelY = fVrelY_mps;
  float32 fTTC = 0.F;
  float32 fTTCFiltered = *pfTTCFiltered_s;
  float32 fFilterSpeed = 0.F;

  // Check if on right sensor, revert the sign of the velocity for using the old
  // coordinate system
  if (eSensorMountingPos == LeftFrontPos || eSensorMountingPos == LeftRearPos) {
    fVrelY = fVrelY * -1.F;
  }
  // Calculate the TTC
  fTTC = fDistToCrossingLine_met / SafeDiv(fVrelY);
  // if the TTC is negative or when the object already passed the crossing line
  if (fTTC < 0.F || fDistToCrossingLine_met < 0.F) {
    fTTC = TUE_C_F32_VALUE_INVALID;
  }
  // For large TTCs set TTC filtered to TTC
  if (fTTC > CTA_CT_LARGE_TTC || fTTCFiltered > CTA_CT_LARGE_TTC)  // 30
  {
    fTTCFiltered = fTTC;
  } else {
    // Predicted the TTC
    fTTCFiltered -= fCycleTime_s;
    // calculate the filter speed for increasing an for decreasing TTC
    if (fTTCFiltered > fTTC) {
      // For close objects set slow filter constant
      fFilterSpeed = GDBmathLinFuncLimBounded(
          fDistToCrossingLine_met, 0.F, CTA_CT_TTCFILTER_MAX_DIST2CROS,
          CTA_CT_TTCFILTER_DOWN_MIN_ALPHA,
          CTA_CT_TTCFILTER_DOWN_MAX_ALPHA);  // 0  5  0.1  0.5
    } else {
      // For close objects set slow filter constant
      fFilterSpeed = GDBmathLinFuncLimBounded(
          fDistToCrossingLine_met, 0.F, CTA_CT_TTCFILTER_MAX_DIST2CROS,
          CTA_CT_TTCFILTER_UP_MIN_ALPHA,
          CTA_CT_TTCFILTER_UP_MAX_ALPHA);  // 0  5  0.05  0.2
    }
    // Filter TTC
    TUE_CML_LowPassFilter(&fTTCFiltered, fTTC, fFilterSpeed);
  }
  *pfTTCFiltered_s = fTTCFiltered;
  *pfTTC_s = fTTC;
}

/*****************************************************************************
  Functionname:CTA_CTCalculateRearObjectProbability */ /*!

  @brief:Calculate a probability value for this object to be a rear object, by looking for objects in front of it

  @description:Calculate a probability value for this object to be a rear object, by looking for objects in front of it

  @param[in]

  @param[out]

  @return
*****************************************************************************/
void CTA_CTCalculateRearObjectProbability(
    uint8 uObj, const EMSRRObjectInReq_t* pCurrObjectEMInput,
    const CTAInReq_t* reqPorts, CTAGlobal_t* pCTAGlobal,
    float32* fRearTrackProb_per) {
  float32 fDistYCurrObj = 0.F;
  float32 fDistYCandObj = 0.F;
  float32 bAbortLoop = FALSE;
  CTARearObjState_t eRearObjState = CTA_REARTRACK_NO;
  float32 fProbChange = 0.F;
  uint16 uLifeCycleThresh = 0;
  float32 fTemp = 0.F;

  fDistYCurrObj = pCurrObjectEMInput->fDistY_met;
  // Check if on right sensor, revert the sign of the Y distance for using the
  // old coordinate system
  if (pCurrObjectEMInput->eSensorMountingPos == RightFrontPos ||
      pCurrObjectEMInput->eSensorMountingPos == RightRearPos) {
    fDistYCurrObj = fDistYCurrObj * -1.F;
  }
  // loop over all object
  for (uint8 uIndex = 0U; (uIndex < CTA_MAX_NUM_OBJECTS) && (!bAbortLoop) &&
                          (eRearObjState == CTA_REARTRACK_NO);
       uIndex++) {
    const EMSRRObjectInReq_t* pObjCandEMInfo =
        &reqPorts->CTAEMSRRObjList[uIndex];
    CTObjectInfoGlobal_t* pCTObjCandGlobal = &pCTAGlobal->CTObjectList[uIndex];
    CTAObjectInfoGlobal_t* pCTAObjCandGlobal =
        &pCTAGlobal->CTAObjectList[uIndex];
    CTAObjectInfoGlobal_t* pCTAObjCurrGlobal = &pCTAGlobal->CTAObjectList[uObj];
    CTAFCTAObjectInfoGlobal_t* pCTAFCTAObjGlobal =
        &pCTAGlobal->CTAFCTAObjectList[uIndex];

    if ((!pObjCandEMInfo->uiMaintenanceState_nu) ==
            CTA_EM_GEN_OBJECT_MT_STATE_DELETED &&
        uObj != uIndex) {
      fDistYCandObj = pObjCandEMInfo->fDistY_met;
      if (pObjCandEMInfo->eSensorMountingPos == RightFrontPos ||
          pObjCandEMInfo->eSensorMountingPos == RightRearPos) {
        fDistYCandObj = fDistYCandObj * -1.F;
      }
      // If the object is in the area of interest or the object is behind the
      // search object
      if ((pCurrObjectEMInput->fDistX_met - pObjCandEMInfo->fDistX_met) >
              CTA_CT_REAROBJPROB_DISTX_THRESH ||    // 10
          (fDistYCurrObj - fDistYCandObj) < 0.F) {  //NOLINT
        // This object is not in the area of interest check the next one
      } else if ((pCurrObjectEMInput->fDistX_met - pObjCandEMInfo->fDistX_met) <
                 -CTA_CT_REAROBJPROB_DISTX_THRESH) {
        // This and all following objects are in front of the area of interest
        // we can abort the loop
        bAbortLoop = TRUE;
      } else if (pCTAFCTAObjGlobal->bRelevant &&
                 !pCTObjCandGlobal->bRearTrack_nu) {
        eRearObjState = CTA_CTCalculateSearchFrontObject(
            pCurrObjectEMInput, pObjCandEMInfo, pCTAObjCurrGlobal,
            pCTAObjCandGlobal,
            pCTAGlobal->CTObjectList[uObj].fRearTrackProb_per);
      } else {
        // MISRA happiness
      }
    }
  }

  // Adapt the rear object probability
  switch (eRearObjState) {
    case CTA_REARTRACK_POS_SPEED:
      // The condition is completely fulfilled: increase the probability
      if (*fRearTrackProb_per > CTA_FCTA_REARTRACK_IMPLAUSIBLE_THRESH)  // 0.05
      {
        if (pCurrObjectEMInput->eSensorMountingPos == LeftFrontPos ||
            pCurrObjectEMInput->eSensorMountingPos == RightFrontPos) {
          uLifeCycleThresh = (uint16)TUE_CML_BoundedLinInterpol2(
              pCurrObjectEMInput->fFirstDetectY_met,
              CTA_CT_LI_REARPROB_MIN_FIRSTY, CTA_CT_LI_REARPROB_MAX_FIRSTY,
              CTA_CT_LI_REARPROB_MAX_LIFECYCLE,
              CTA_CT_LI_REARPROB_MIN_LIFECYCLE);  // 20  40  75  50
        } else {
          uLifeCycleThresh = (uint16)CTA_CT_LI_REARPROB_MIN_LIFECYCLE;
        }
        // Reduce the max probability change for old objects
        fTemp = TUE_CML_BoundedLinInterpol2(
            (float32)pCurrObjectEMInput->uiLifeCycles_nu,
            (float32)uLifeCycleThresh, 0.F, 0.F, CTA_FCTA_REARTRACK_INC_PROB);
        // Reduce the max probability change for objects near the y-axes
        fProbChange = TUE_CML_BoundedLinInterpol2(
            fDistYCurrObj, 0.F, CTA_CT_LI_MAXREARPROB_MAX_DISTY, 0.F,
            CTA_FCTA_REARTRACK_INC_PROB);
        // Use the minimum of all limitations
        fProbChange = MIN(fProbChange, fTemp);
      }
      break;
    case CTA_REARTRACK_POS:
      // The condition is partly fulfilled: keep the probability
      break;
    case CTA_REARTRACK_NO:
      // The condition is not fulfilled: decrease the probability
      fProbChange = -CTA_FCTA_REARTRACK_DEC_PROB;  // 0.02
      break;
    default:
      // Do nothing
      break;
  }
  // Update the probability
  *fRearTrackProb_per += fProbChange;
  // Limit the rear object probability
  *fRearTrackProb_per = TUE_CML_MinMax(0.F, 1.F, *fRearTrackProb_per);
}

/*****************************************************************************
  Functionname:CTA_CTCalculateSearchFrontObject */ /*!

  @brief Search for objects in front of the current object

  @description Search for objects in front of the current object

  @param[in]

  @param[out]

  @return
*****************************************************************************/
CTARearObjState_t CTA_CTCalculateSearchFrontObject(
    const EMSRRObjectInReq_t* pCurrObjectEMInput,
    const EMSRRObjectInReq_t* pObjCandEMInfo,
    CTAObjectInfoGlobal_t* pCTAObjCurrGlobal,
    CTAObjectInfoGlobal_t* pCTAObjCandGlobal, float32 fRearTrackProb_per) {
  float32 fXDiff = 0.F;
  float32 fYDiff = 0.F;
  float32 fVxDiff = 0.F;
  float32 fVyDiff = 0.F;
  float32 fVyDiffPosBased = 0.F;
  float32 fMaxRangeDiff = 0.F;
  float32 fMaxDistXDiff = 0.F;
  float32 fMaxDistYDiff = 0.F;
  float32 fMaxVxDiff = 0.F;
  float32 fMaxVyDiff = 0.F;
  CTARearObjState_t eObjRearTrackState = CTA_REARTRACK_NO;
  // Use the heading values because it represents the relative heading of the
  // target
  float32 fHeading = pObjCandEMInfo->fRelHeading_rad;

  // check if on right sensor, revert the sign of the heading for using the old
  // coordinate system
  if (pObjCandEMInfo->eSensorMountingPos == RightFrontPos ||
      pObjCandEMInfo->eSensorMountingPos == RightRearPos) {
    fHeading = fHeading * -1.F;
  }
  // Calculate object position difference
  fXDiff =
      CTA_CTCalculateObjectDistance(pCTAObjCurrGlobal->ObjectBorder.fXMin_met,
                                    pCTAObjCurrGlobal->ObjectBorder.fXMax_met,
                                    pCTAObjCandGlobal->ObjectBorder.fXMin_met,
                                    pCTAObjCandGlobal->ObjectBorder.fXMax_met);
  fYDiff =
      CTA_CTCalculateObjectDistance(pCTAObjCurrGlobal->ObjectBorder.fYMin_met,
                                    pCTAObjCurrGlobal->ObjectBorder.fYMax_met,
                                    pCTAObjCandGlobal->ObjectBorder.fYMin_met,
                                    pCTAObjCandGlobal->ObjectBorder.fYMax_met);
  fVxDiff = fABS(pObjCandEMInfo->fVrelX_mps - pCurrObjectEMInput->fVrelX_mps);
  fVyDiff = fABS(pObjCandEMInfo->fVrelY_mps - pCurrObjectEMInput->fVrelY_mps);
  fVyDiffPosBased = fABS(pCTAObjCandGlobal->fVyPosBased_mps -
                         pCTAObjCurrGlobal->fVyPosBased_mps);
  // Calculate max differences. Assume that two following vehicles have at least
  // CTA_CT_MULTIOBJ_SAFE_MARGIN security distance Multiply with the assumed
  // security distance S = V*t
  fMaxRangeDiff = sqrt(SQR(pCurrObjectEMInput->fVabsX_mps) +
                       SQR(pCurrObjectEMInput->fVabsY_mps)) *
                  CTA_CT_MULTIOBJ_SAFE_MARGIN;
  // Calculate the limits depending on the heading angle
  fMaxDistXDiff = fABS(COS_(fHeading) * fMaxRangeDiff);
  fMaxDistYDiff = fABS(SIN_(fHeading) * fMaxRangeDiff);
  // Limit the thresholds
  fMaxDistXDiff = TUE_CML_MinMax(CTA_CT_FRONTOBJ_MIN_XYDIFF,
                                 CTA_CT_FRONTOBJ_MAX_XYDIFF, fMaxDistXDiff);
  fMaxDistYDiff = TUE_CML_MinMax(CTA_CT_FRONTOBJ_MIN_XYDIFF,
                                 CTA_CT_FRONTOBJ_MAX_XYDIFF, fMaxDistYDiff);
  // Check rear object conditions
  if (fXDiff < fMaxDistXDiff && fYDiff < fMaxDistYDiff) {
    // Position criteria is fulfilled
    eObjRearTrackState = CTA_REARTRACK_POS;
    // Set velocity thresholds
    fMaxVxDiff = TUE_CML_BoundedLinInterpol2(fRearTrackProb_per, 0.F, 1.F,
                                             CTA_CT_LI_FRONTOBJ_MIN_VXYDIFF,
                                             CTA_CT_LI_FRONTOBJ_MAX_VXYDIFF);
    fMaxVyDiff = TUE_CML_BoundedLinInterpol2(fRearTrackProb_per, 0.F, 1.F,
                                             CTA_CT_LI_FRONTOBJ_MIN_VXYDIFF,
                                             CTA_CT_LI_FRONTOBJ_MAX_VXYDIFF);
    // Check speed criteria
    if (fVxDiff < fMaxVxDiff &&
        (fVyDiff < fMaxVyDiff || fVyDiffPosBased < fMaxVyDiff)) {
      eObjRearTrackState = CTA_REARTRACK_POS_SPEED;
    }
  }
  // return rear track bool
  return eObjRearTrackState;
}

/*****************************************************************************
  Functionname: CTA_CTCalculateObjectDistance */ /*!

  @brief Calculate the distance of two objects in X or Y direction, using their dimensions

  @description Calculate the distance of two objects in X or Y direction, using their dimensions

  @param[in]  fObjCurrMin_met
              fObjCurrMax_met
			  fObjCandMin_met
			  fObjCandMax_met
  @param[out]

  @return fDistance
*****************************************************************************/
float32 CTA_CTCalculateObjectDistance(float32 fObjCurrMin_met,
                                      float32 fObjCurrMax_met,
                                      float32 fObjCandMin_met,
                                      float32 fObjCandMax_met) {
  float32 fDistance = 0.F;

  const float32 fDiffBA = fABS(fObjCandMax_met - fObjCurrMin_met);
  const float32 fDiffAB = fABS(fObjCurrMax_met - fObjCandMin_met);

  // Check the condition for no overlap
  if (fObjCandMax_met < fObjCurrMin_met || fObjCurrMax_met < fObjCandMin_met) {
    fDistance = MIN(fDiffBA, fDiffAB);
  }
  return fDistance;
}

/*****************************************************************************
  Functionname:CTA_CTCheckRearObject                                     */ /*!

  @brief check the rear object

  @description Search for rear objects which are the end of a moving vehicle such objects would also warn and keep the warning active too long or
		       trigger new warnings on the same object

  @param[in]  uClassification_nu
              fRearTrackProb_per
  @param[out] bRearTrack_nu

  @return
****************************** get error result  WHY ***********************************************/

void CTA_CTCheckRearObject(uint32 uClassification_nu,
                           float32 fRearTrackProb_per,
                           boolean* pbRearTrack_nu) {
  boolean bRearTrack = FALSE;
  // Use hysteresis
  if (*pbRearTrack_nu) {
    if (fRearTrackProb_per > CTA_FCTA_REARTRACK_DEACTIVATION_THRESH ||  // 0.7
        uClassification_nu == CTA_EM_GEN_OBJECT_CLASS_MULTIPLE)         // 8
    {
      bRearTrack = TRUE;
    }
  } else {
    if (fRearTrackProb_per > CTA_FCTA_REARTRACK_ACTIVATION_THRESH ||  // 0.9
        uClassification_nu == CTA_EM_GEN_OBJECT_CLASS_MULTIPLE)       // 8
    {
      bRearTrack = TRUE;
    }
  }
  *pbRearTrack_nu = bRearTrack;
}

/*****************************************************************************
  Functionname:                                     */ /*!

  @brief

  @description

  @param[in]

  @param[out]

  @return
*****************************************************************************/
void CTAToFCTAInputWrapper(const CTAInReq_t* reqPorts, const CTAParam_t* params,
                           CTAGlobal_t* pCTAGlobal, FCTAInReq_t* FCTAreqPorts,
                           FCTAParam_t* FCTAparams) {
  memset(&(FCTAreqPorts->EMSRRObjListInput), 0, sizeof(FCTAEMSRRObjInReq_t));

  FCTAreqPorts->FCTAVehicleSig.uFCTAGearPosition =
      reqPorts->EgoVehicleInfo.uCTAGearPosition;
  FCTAreqPorts->FCTAVehicleSig.bFCTAEgoSixDoorsClosed =
      reqPorts->EgoVehicleInfo.bEgoSixDoorsClosed;
  FCTAreqPorts->bFCTAFunctionActive = reqPorts->bFCTAFunctionActive;
  FCTAreqPorts->bFCTAFunctionOutputActive = reqPorts->bFCTAFunctionOutputActive;
  FCTAreqPorts->fCycleTime_s = reqPorts->fCycleTime_s;
  FCTAreqPorts->FCTAVehicleSig.fegoVelocity_mps =
      reqPorts->EgoVehicleInfo.fegoVelocity_mps;
  FCTAreqPorts->FCTARoadInformation.fCurveRadius_met =
      reqPorts->CTARoadInformation.fCurveRadius_met;

  FCTAreqPorts->FCTARoadInformation.fCurveRadius_met =
      reqPorts->CTARoadInformation.fCurveRadius_met;
  FCTAparams->bActive = params->CTA_Ks_FCTAAlgoParameter_nu.CTA_Kb_Active_nu;
  FCTAparams->fBreakthroughMargin =
      params->CTA_Ks_FCTAAlgoParameter_nu.CTA_Kf_BreakthroughMargin_met;
  FCTAparams->fMaxHeadingAngle =
      params->CTA_Ks_FCTAAlgoParameter_nu.CTA_Kf_MaxHeadingAngle_deg;
  FCTAparams->fMinHeadingAngle =
      params->CTA_Ks_FCTAAlgoParameter_nu.CTA_Kf_MinHeadingAngle_deg;
  FCTAparams->fTargetRangeMax =
      params->CTA_Ks_FCTAAlgoParameter_nu.CTA_Kf_TargetRangeMax_met;
  FCTAparams->fTargetRangeMaxL2 =
      params->CTA_Ks_FCTAAlgoParameter_nu.CTA_Kf_TargetRangeMaxL2_met;
  FCTAparams->fTTCThreshold =
      params->CTA_Ks_FCTAAlgoParameter_nu.CTA_Kf_TTCThreshold_s;
  FCTAparams->fTTCThresholdL2 =
      params->CTA_Ks_FCTAAlgoParameter_nu.CTA_Kf_TTCThresholdL2_s;
  FCTAparams->fTTCThresholdMargin =
      params->CTA_Ks_FCTAAlgoParameter_nu.CTA_Kf_TTCThresholdMargin_s;
  FCTAparams->fVEgoMax = params->CTA_Ks_FCTAAlgoParameter_nu.CTA_Kf_VEgoMax_mps;
  FCTAparams->fVEgoMin = params->CTA_Ks_FCTAAlgoParameter_nu.CTA_Kf_VEgoMin_mps;
  FCTAparams->fVTargetMax =
      params->CTA_Ks_FCTAAlgoParameter_nu.CTA_Kf_VTargetMax_mps;
  FCTAparams->fVTargetMin =
      params->CTA_Ks_FCTAAlgoParameter_nu.CTA_Kf_VTargetMin_mps;
  FCTAparams->fXMaxBreakthrough =
      params->CTA_Ks_FCTAAlgoParameter_nu.CTA_Kf_XMaxBreakthrough_met;
  FCTAparams->fXMinBreakthrough =
      params->CTA_Ks_FCTAAlgoParameter_nu.CTA_Kf_XMinBreakthrough_met;
  FCTAparams->fXMinBreakthroughL2 =
      params->CTA_Ks_FCTAAlgoParameter_nu.CTA_Kf_XMinBreakthroughL2_met;

  FCTAreqPorts->CTAGlobleInput.fSensorOffsetToRear_met =
      (pCTAGlobal->fSensorOffsetToRear_met.fLeftFrontPos_met +
       pCTAGlobal->fSensorOffsetToRear_met.fRightFrontPos_met) *
      0.5F;
  FCTAreqPorts->CTGlobalInput.fMaxLatSensorRange =
      pCTAGlobal->CTGlobals.fMaxLatSensorRange;

  for (uint8 uObj = 0; uObj < CTA_MAX_NUM_OBJECTS; uObj++) {
    FCTAreqPorts->EMSRRObjListInput[uObj].bObjStable =
        reqPorts->CTAEMSRRObjList[uObj].bObjStable;
    FCTAreqPorts->EMSRRObjListInput[uObj].fAbsOrientation_rad =
        reqPorts->CTAEMSRRObjList[uObj].fAbsOrientation_rad;
    FCTAreqPorts->EMSRRObjListInput[uObj].fArelX_mpss =
        reqPorts->CTAEMSRRObjList[uObj].fArelX_mpss;
    FCTAreqPorts->EMSRRObjListInput[uObj].fArelY_mpss =
        reqPorts->CTAEMSRRObjList[uObj].fArelY_mpss;
    FCTAreqPorts->EMSRRObjListInput[uObj].fDistX_met =
        reqPorts->CTAEMSRRObjList[uObj].fDistX_met;
    FCTAreqPorts->EMSRRObjListInput[uObj].fDistY_met =
        reqPorts->CTAEMSRRObjList[uObj].fDistY_met;
    FCTAreqPorts->EMSRRObjListInput[uObj].fFirstDetectX_met =
        reqPorts->CTAEMSRRObjList[uObj].fFirstDetectX_met;
    FCTAreqPorts->EMSRRObjListInput[uObj].fFirstDetectY_met =
        reqPorts->CTAEMSRRObjList[uObj].fFirstDetectY_met;
    FCTAreqPorts->EMSRRObjListInput[uObj].fProbabilityOfExistence_per =
        reqPorts->CTAEMSRRObjList[uObj].fProbabilityOfExistence_per;
    FCTAreqPorts->EMSRRObjListInput[uObj].fMirrorProb_per =
        1 - reqPorts->CTAEMSRRObjList[uObj].fProbabilityOfExistence_per;
    // reqPorts->CTAEMSRRObjList[uObj].fMirrorProb_per;
    FCTAreqPorts->EMSRRObjListInput[uObj].fRCS =
        reqPorts->CTAEMSRRObjList[uObj].fRCS;
    FCTAreqPorts->EMSRRObjListInput[uObj].fVabsX_mps =
        reqPorts->CTAEMSRRObjList[uObj].fVabsX_mps;
    FCTAreqPorts->EMSRRObjListInput[uObj].fVabsY_mps =
        reqPorts->CTAEMSRRObjList[uObj].fVabsY_mps;
    FCTAreqPorts->EMSRRObjListInput[uObj].fVrelX_mps =
        reqPorts->CTAEMSRRObjList[uObj].fVrelX_mps;
    FCTAreqPorts->EMSRRObjListInput[uObj].fVrelY_mps =
        reqPorts->CTAEMSRRObjList[uObj].fVrelY_mps;
    FCTAreqPorts->EMSRRObjListInput[uObj].fWidthLeft_met =
        reqPorts->CTAEMSRRObjList[uObj].fWidthLeft_met;
    FCTAreqPorts->EMSRRObjListInput[uObj].uiLifeCycles_nu =
        reqPorts->CTAEMSRRObjList[uObj].uiLifeCycles_nu;
    FCTAreqPorts->EMSRRObjListInput[uObj].uiMaintenanceState_nu =
        reqPorts->CTAEMSRRObjList[uObj].uiMaintenanceState_nu;
    FCTAreqPorts->EMSRRObjListInput[uObj].eDynamicProperty_nu =
        reqPorts->CTAEMSRRObjList[uObj].eDynamicProperty_nu;
    FCTAreqPorts->EMSRRObjListInput[uObj].uiMeasuredTargetFrequency_nu =
        reqPorts->CTAEMSRRObjList[uObj].uiMeasuredTargetFrequency_nu;
    if (reqPorts->CTAEMSRRObjList[uObj].eSensorMountingPos == RightFrontPos ||
        reqPorts->CTAEMSRRObjList[uObj].eSensorMountingPos == RightRearPos) {
      FCTAreqPorts->EMSRRObjListInput[uObj].bRightSensor = TRUE;
    } else {
      FCTAreqPorts->EMSRRObjListInput[uObj].bRightSensor = FALSE;
    }

    FCTAreqPorts->CTAGlobleInput.FCTACTAObjListInput[uObj]
        .fAssocProbFiltered_nu =
        pCTAGlobal->CTAObjectList[uObj].fAssocProbFiltered_nu;
    FCTAreqPorts->CTAGlobleInput.FCTACTAObjListInput[uObj].fUpdateRate_nu =
        pCTAGlobal->CTAObjectList[uObj].fUpdateRate_nu;
    FCTAreqPorts->CTAGlobleInput.FCTACTAObjListInput[uObj].fVabs =
        pCTAGlobal->CTAObjectList[uObj].fVabs;
    FCTAreqPorts->CTAGlobleInput.FCTACTAObjListInput[uObj].fVxPosBased =
        pCTAGlobal->CTAObjectList[uObj].fVxPosBased_mps;
    FCTAreqPorts->CTAGlobleInput.FCTACTAObjListInput[uObj].fVyPosBased =
        pCTAGlobal->CTAObjectList[uObj].fVyPosBased_mps;
    FCTAreqPorts->CTAGlobleInput.FCTACTAObjListInput[uObj].fXMovement_met =
        pCTAGlobal->CTAObjectList[uObj].fXMovement_met;
    FCTAreqPorts->CTAGlobleInput.FCTACTAObjListInput[uObj].fYMovement_met =
        pCTAGlobal->CTAObjectList[uObj].fYMovement_met;

    FCTAreqPorts->CTGlobalInput.CTObjectListGlobalInput[uObj].bRearTrack_nu =
        pCTAGlobal->CTObjectList[uObj].bRearTrack_nu;
    FCTAreqPorts->CTGlobalInput.CTObjectListGlobalInput[uObj]
        .fDistToCrossingLine_met =
        pCTAGlobal->CTObjectList[uObj].fDistToCrossingLine_met;
    FCTAreqPorts->CTGlobalInput.CTObjectListGlobalInput[uObj]
        .fDistToCrossingLineFiltered_met =
        pCTAGlobal->CTObjectList[uObj].fDistToCrossingLineFiltered_met;
    FCTAreqPorts->CTGlobalInput.CTObjectListGlobalInput[uObj].fTTCFiltered_s =
        pCTAGlobal->CTObjectList[uObj].fTTCFiltered_s;
    FCTAreqPorts->CTGlobalInput.CTObjectListGlobalInput[uObj].fTTC_s =
        pCTAGlobal->CTObjectList[uObj].fTTC_s;
    FCTAreqPorts->CTGlobalInput.CTObjectListGlobalInput[uObj]
        .fXBreakthrough_met = pCTAGlobal->CTObjectList[uObj].fXBreakthrough_met;
    FCTAreqPorts->CTGlobalInput.CTObjectListGlobalInput[uObj]
        .fXBreakthroughStd_met =
        pCTAGlobal->CTObjectList[uObj].fXBreakthroughStd_met;
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
void FCTAToCTAOutputWrapper(FCTAOutPro_t* FCTAproPorts,
                            FCTADebug_t* FCTAdebugInfo, CTAGlobal_t* pCTAGlobal,
                            CTADebug_t* debugInfo) {
  if (sizeof(FCTAObjGlobal_t) != sizeof(FCTAObjOutput_t)) {
    DEBUG_Print("FCTA FCTAToCTAOutputWrapper Error!");
    return;
  }
  memcpy(debugInfo->FCTAObjGlobal,  //NOLINT
         FCTAproPorts->FCTA_Va_ObjectListGlobal,
         sizeof(FCTAObjOutput_t) * FCTA_MAX_NUM_OBJECTS);

  for (uint8 uWarnLevel = 0U; uWarnLevel < CTA_FCTA_CFG_NUM_OF_WARN_LEVELS;
       uWarnLevel++) {
    pCTAGlobal->CTAFCTAOutput.bFCTAWarnActive[uWarnLevel] =
        FCTAproPorts->bFCTAWarnActive[uWarnLevel];
    //  debug for assigning value
    debugInfo->FCTAGlobal.fTTCThreshold_s[uWarnLevel] =
        FCTAproPorts->fTTCThreshold_s[uWarnLevel];
    debugInfo->FCTAGlobal.fXMinBreakthrough_met[uWarnLevel] =
        FCTAproPorts->fXMinBreakthrough_met[uWarnLevel];
    debugInfo->FCTAGlobal.fXMaxBreakthrough_met[uWarnLevel] =
        FCTAproPorts->fXMaxBreakthrough_met[uWarnLevel];
    debugInfo->FCTAGlobal.fMaxObjRange_met[uWarnLevel] =
        FCTAproPorts->fMaxObjRange_met[uWarnLevel];
    debugInfo->FCTAGlobal.bFCTAWarnActive[uWarnLevel] =
        FCTAproPorts->bFCTAWarnActive[uWarnLevel];
  }
  pCTAGlobal->CTAFCTAOutput.fCriticalTTC_s = FCTAproPorts->fCriticalTTC_s;

  //  debug for assigning value
  debugInfo->FCTAGlobal.fMaxHeadingAngle_deg =
      FCTAproPorts->fMaxHeadingAngle_deg;
  debugInfo->FCTAGlobal.fMinHeadingAngle_deg =
      FCTAproPorts->fMinHeadingAngle_deg;
  debugInfo->FCTAGlobal.fCriticalTTC_s = FCTAproPorts->fCriticalTTC_s;
  debugInfo->FCTAGlobal.fCriticalObjDistY_met =
      FCTAproPorts->fCriticalObjDistY_met;

  debugInfo->FCTAGlobal.fCriticalObjDistYLastCycle_met =
      FCTAproPorts->fCriticalObjDistYLastCycle_met;

  debugInfo->FCTAGlobal.uCriticalObjID_nu = FCTAproPorts->uCriticalObjID_nu;
  debugInfo->FCTAGlobal.uCriticalObjIDLastCycle_nu =
      FCTAproPorts->uCriticalObjIDLastCycle_nu;
  debugInfo->FCTAGlobal.uInterruptCycleCount_nu =
      FCTAproPorts->uInterruptCycleCount_nu;
  debugInfo->FCTAGlobal.bWarningInterrupt = FCTAproPorts->bWarningInterrupt;
}

/*****************************************************************************
   RCTA Input Wrapper
*****************************************************************************/
void CTAToRCTAInputWrapper(const CTAInReq_t* reqPorts, const CTAParam_t* params,
                           CTAGlobal_t* pCTAGlobal, RCTAInReq_t* RCTAreqPorts,
                           RCTAParam_t* RCTAparams) {
  memset(&(RCTAreqPorts->EMSRRObjListInput), 0, sizeof(RCTAEMSRRObjInReq_t));

  RCTAreqPorts->RCTAVehicleSig.uRCTAGearPosition =
      reqPorts->EgoVehicleInfo.uCTAGearPosition;
  RCTAreqPorts->RCTAVehicleSig.bRCTAEgoSixDoorsClosed =
      reqPorts->EgoVehicleInfo.bEgoSixDoorsClosed;
  RCTAreqPorts->bRCTAFunctionActive = reqPorts->bRCTAFunctionActive;
  RCTAreqPorts->bRCTAFunctionOutputActive = reqPorts->bRCTAFunctionOutputActive;
  RCTAreqPorts->fCycleTime_s = reqPorts->fCycleTime_s;
  RCTAreqPorts->RCTAVehicleSig.StWheelAngle_rad =
      reqPorts->EgoVehicleInfo.fSelfSteering_rad;
  RCTAparams->RCTAAlgoParam.fMaxHeadingAngle =
      params->CTA_Ks_RCTAAlgoParameter_nu.CTA_Kf_MaxHeadingAngle_deg;
  RCTAparams->RCTAAlgoParam.fMinHeadingAngle =
      params->CTA_Ks_RCTAAlgoParameter_nu.CTA_Kf_MinHeadingAngle_deg;
  RCTAparams->RCTAAlgoParam.fTargetRangeMax_met =
      params->CTA_Ks_RCTAAlgoParameter_nu.CTA_Kf_TargetRangeMax_met;
  RCTAparams->RCTAAlgoParam.fTargetRangeMaxL2_met =
      params->CTA_Ks_RCTAAlgoParameter_nu.CTA_Kf_TargetRangeMaxL2_met;
  RCTAparams->RCTAAlgoParam.fTTCThreshold_s =
      params->CTA_Ks_RCTAAlgoParameter_nu.CTA_Kf_TTCThreshold_s;
  RCTAparams->RCTAAlgoParam.fTTCThresholdL2_s =
      params->CTA_Ks_RCTAAlgoParameter_nu.CTA_Kf_TTCThresholdL2_s;
  RCTAparams->RCTAAlgoParam.fTTCThresholdMargin_s =
      params->CTA_Ks_RCTAAlgoParameter_nu.CTA_Kf_TTCThresholdMargin_s;
  RCTAparams->RCTAAlgoParam.fVTargetMin =
      params->CTA_Ks_RCTAAlgoParameter_nu.CTA_Kf_VTargetMin_mps;
  RCTAparams->RCTAAlgoParam.fXMaxBreakthrough_met =
      params->CTA_Ks_RCTAAlgoParameter_nu.CTA_Kf_XMaxBreakthrough_met;
  RCTAparams->RCTAAlgoParam.fXMinBreakthrough_met =
      params->CTA_Ks_RCTAAlgoParameter_nu.CTA_Kf_XMinBreakthrough_met;
  RCTAparams->RCTAAlgoParam.fXMinBreakthroughL2_met =
      params->CTA_Ks_RCTAAlgoParameter_nu.CTA_Kf_XMinBreakthroughL2_met;
  memcpy(&RCTAparams->RCTAAlgoParam.SteeringAngleCutOff,  //NOLINT
         &params->CTA_Ks_RCTAAlgoParameter_nu.CTA_Ks_SteeringAngleCutOff_nu,
         sizeof(RCTASteeringAngleCutOffParam_t));

  for (uint8 uObj = 0; uObj < CTA_MAX_NUM_OBJECTS; uObj++) {
    RCTAreqPorts->EMSRRObjListInput[uObj].bObjStable =
        reqPorts->CTAEMSRRObjList[uObj].bObjStable;
    RCTAreqPorts->EMSRRObjListInput[uObj].fDistX_met =
        reqPorts->CTAEMSRRObjList[uObj].fDistX_met;
    RCTAreqPorts->EMSRRObjListInput[uObj].fDistY_met =
        reqPorts->CTAEMSRRObjList[uObj].fDistY_met;
    RCTAreqPorts->EMSRRObjListInput[uObj].fFirstDetectY_met =
        reqPorts->CTAEMSRRObjList[uObj].fFirstDetectY_met;
    RCTAreqPorts->EMSRRObjListInput[uObj].fLengthFront_met =
        reqPorts->CTAEMSRRObjList[uObj].fLengthFront_met;
    RCTAreqPorts->EMSRRObjListInput[uObj].fProbabilityOfExistence_per =
        reqPorts->CTAEMSRRObjList[uObj].fProbabilityOfExistence_per;
    RCTAreqPorts->EMSRRObjListInput[uObj].fMirrorProb_per =
        1 - reqPorts->CTAEMSRRObjList[uObj].fProbabilityOfExistence_per;
    // reqPorts->CTAEMSRRObjList[uObj].fMirrorProb_per;
    RCTAreqPorts->EMSRRObjListInput[uObj].fRCS =
        reqPorts->CTAEMSRRObjList[uObj].fRCS;
    RCTAreqPorts->EMSRRObjListInput[uObj].fRelHeading_rad =
        reqPorts->CTAEMSRRObjList[uObj].fRelHeading_rad;
    RCTAreqPorts->EMSRRObjListInput[uObj].fRelHeadingStd_rad =
        reqPorts->CTAEMSRRObjList[uObj].fRelHeadingStd_rad;
    RCTAreqPorts->EMSRRObjListInput[uObj].fVabsX_mps =
        reqPorts->CTAEMSRRObjList[uObj].fVabsX_mps;
    RCTAreqPorts->EMSRRObjListInput[uObj].fVabsY_mps =
        reqPorts->CTAEMSRRObjList[uObj].fVabsY_mps;
    RCTAreqPorts->EMSRRObjListInput[uObj].fVrelY_mps =
        reqPorts->CTAEMSRRObjList[uObj].fVrelY_mps;
    RCTAreqPorts->EMSRRObjListInput[uObj].fWidthLeft_met =
        reqPorts->CTAEMSRRObjList[uObj].fWidthLeft_met;
    RCTAreqPorts->EMSRRObjListInput[uObj].uiLifeCycles_nu =
        reqPorts->CTAEMSRRObjList[uObj].uiLifeCycles_nu;
    RCTAreqPorts->EMSRRObjListInput[uObj].uiMaintenanceState_nu =
        reqPorts->CTAEMSRRObjList[uObj].uiMaintenanceState_nu;
    RCTAreqPorts->EMSRRObjListInput[uObj].uiMeasuredTargetFrequency_nu =
        reqPorts->CTAEMSRRObjList[uObj].uiMeasuredTargetFrequency_nu;
    if (reqPorts->CTAEMSRRObjList[uObj].eSensorMountingPos == RightFrontPos ||
        reqPorts->CTAEMSRRObjList[uObj].eSensorMountingPos == RightRearPos) {
      RCTAreqPorts->EMSRRObjListInput[uObj].bRightSensor = TRUE;
    } else {
      RCTAreqPorts->EMSRRObjListInput[uObj].bRightSensor = FALSE;
    }

    RCTAreqPorts->CTAGlobleInput.RCTACTAObjListInput[uObj]
        .fAssocProbFiltered_nu =
        pCTAGlobal->CTAObjectList[uObj].fAssocProbFiltered_nu;
    RCTAreqPorts->CTAGlobleInput.RCTACTAObjListInput[uObj].fUpdateRate_nu =
        pCTAGlobal->CTAObjectList[uObj].fUpdateRate_nu;
    RCTAreqPorts->CTAGlobleInput.RCTACTAObjListInput[uObj].fXMovement_met =
        pCTAGlobal->CTAObjectList[uObj].fXMovement_met;
    RCTAreqPorts->CTAGlobleInput.RCTACTAObjListInput[uObj].fYMovement_met =
        pCTAGlobal->CTAObjectList[uObj].fYMovement_met;

    // CT object
    RCTAreqPorts->CTAGlobleInput.RCTACTObjListInput[uObj].bRearTrack_nu =
        pCTAGlobal->CTObjectList[uObj].bRearTrack_nu;
    RCTAreqPorts->CTAGlobleInput.RCTACTObjListInput[uObj]
        .fDistToCrossingLine_met =
        pCTAGlobal->CTObjectList[uObj].fDistToCrossingLine_met;
    RCTAreqPorts->CTAGlobleInput.RCTACTObjListInput[uObj].fTTC_s =
        pCTAGlobal->CTObjectList[uObj].fTTC_s;
    RCTAreqPorts->CTAGlobleInput.RCTACTObjListInput[uObj].fTTCFiltered_s =
        pCTAGlobal->CTObjectList[uObj].fTTCFiltered_s;
    RCTAreqPorts->CTAGlobleInput.RCTACTObjListInput[uObj].fXBreakthrough_met =
        pCTAGlobal->CTObjectList[uObj].fXBreakthrough_met;
    RCTAreqPorts->CTAGlobleInput.RCTACTObjListInput[uObj]
        .fXBreakthroughStd_met =
        pCTAGlobal->CTObjectList[uObj].fXBreakthroughStd_met;
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
void RCTAToCTAOutputWrapper(RCTAOutPro_t* RCTAproPorts,
                            RCTADebug_t* RCTAdebugInfo, CTAGlobal_t* pCTAGlobal,
                            CTADebug_t* debugInfo) {
  //  debug for assigning value
  if (sizeof(RCTAObjGlobal_t) != sizeof(RCTAObjOutput_t)) {
    DEBUG_Print("RCTA RCTAToCTAOutputWrapper Error!");
    return;
  }
  memcpy(debugInfo->RCTAObjGlobal,  //NOLINT
         RCTAproPorts->RCTA_Va_ObjectListGlobal,
         sizeof(RCTAObjOutput_t) * RCTA_MAX_NUM_OBJECTS);

  for (uint8 uWarnLevel = 0U; uWarnLevel < CTA_RCTA_CFG_NUM_OF_WARN_LEVELS;
       uWarnLevel++) {
    pCTAGlobal->CTARCTAOutput.bRCTAWarnActive[uWarnLevel] =
        RCTAproPorts->bRCTAWarnActive[uWarnLevel];
    //  debug for assigning value
    debugInfo->RCTAGlobal.fTTCThreshold_s[uWarnLevel] =
        RCTAproPorts->fTTCThreshold_s[uWarnLevel];
    debugInfo->RCTAGlobal.fXMinBreakthrough_met[uWarnLevel] =
        RCTAproPorts->fXMinBreakthrough_met[uWarnLevel];
    debugInfo->RCTAGlobal.fXMaxBreakthrough_met[uWarnLevel] =
        RCTAproPorts->fXMaxBreakthrough_met[uWarnLevel];
    debugInfo->RCTAGlobal.fMaxObjRange_met[uWarnLevel] =
        RCTAproPorts->fMaxObjRange_met[uWarnLevel];
    debugInfo->RCTAGlobal.bRCTAWarnActive[uWarnLevel] =
        RCTAproPorts->bRCTAWarnActive[uWarnLevel];
  }
  pCTAGlobal->CTARCTAOutput.bWarningInterrupt = RCTAproPorts->bWarningInterrupt;
  pCTAGlobal->CTARCTAOutput.fCriticalObjDistYLastCycle_met =
      RCTAproPorts->fCriticalObjDistYLastCycle_met;
  pCTAGlobal->CTARCTAOutput.fCriticalObjDistY_met =
      RCTAproPorts->fCriticalObjDistY_met;
  pCTAGlobal->CTARCTAOutput.fCriticalTTC_s = RCTAproPorts->fCriticalTTC_s;
  pCTAGlobal->CTARCTAOutput.uCriticalObjIDLastCycle_nu =
      RCTAproPorts->uCriticalObjIDLastCycle_nu;
  pCTAGlobal->CTARCTAOutput.uCriticalObjID_nu = RCTAproPorts->uCriticalObjID_nu;
  pCTAGlobal->CTARCTAOutput.uInterruptCycleCount_nu =
      RCTAproPorts->uInterruptCycleCount_nu;

  debugInfo->RCTAGlobal.fMaxHeadingAngle_deg =
      RCTAproPorts->fMaxHeadingAngle_deg;
  debugInfo->RCTAGlobal.fMinHeadingAngle_deg =
      RCTAproPorts->fMinHeadingAngle_deg;
  debugInfo->RCTAGlobal.fCriticalTTC_s = RCTAproPorts->fCriticalTTC_s;
  debugInfo->RCTAGlobal.fCriticalObjDistY_met =
      RCTAproPorts->fCriticalObjDistY_met;
  debugInfo->RCTAGlobal.fCriticalObjDistYLastCycle_met =
      RCTAproPorts->fCriticalObjDistYLastCycle_met;
  debugInfo->RCTAGlobal.uCriticalObjID_nu = RCTAproPorts->uCriticalObjID_nu;
  debugInfo->RCTAGlobal.uCriticalObjIDLastCycle_nu =
      RCTAproPorts->uCriticalObjIDLastCycle_nu;
  debugInfo->RCTAGlobal.uInterruptCycleCount_nu =
      RCTAproPorts->uInterruptCycleCount_nu;
  debugInfo->RCTAGlobal.bWarningInterrupt = RCTAproPorts->bWarningInterrupt;
}

/*****************************************************************************
  Functionname:                                     */ /*!

  @brief

  @description

  @param[in]

  @param[out]

  @return
*****************************************************************************/
void CTAProProcess(const CTAInReq_t* reqPorts, const CTAParam_t* params,
                   CTAGlobal_t* pCTAGlobal, CTAOutPro_t* proPorts) {
  boolean bFCTAFunctionOutputActive = FALSE;
  boolean bRCTAFunctionOutputActive = FALSE;
  boolean bEgoSpeedCondition = FALSE;
  boolean bTempWarning = FALSE;
  proPorts->bFCTAWarning = FALSE;
  proPorts->bFCTAWarning2 = FALSE;
  proPorts->bRCTAWarning = FALSE;
  proPorts->bRCTAWarning = FALSE;

  proPorts->fFCTAfTTC_s = TUE_C_F32_VALUE_INVALID;
  proPorts->fRCTAfTTC_s = TUE_C_F32_VALUE_INVALID;

  // RCTA
  if (reqPorts->bRCTAFunctionActive && reqPorts->bRCTAFunctionOutputActive) {
    bRCTAFunctionOutputActive = TRUE;
  }
  bEgoSpeedCondition = CTAProcessCheckEgoSpeedRange(
      reqPorts->EgoVehicleInfo.fegoVelocity_mps,
      params->CTA_Ks_RCTAAlgoParameter_nu.CTA_Kf_VEgoMax_mps,
      params->CTA_Ks_RCTAAlgoParameter_nu.CTA_Kf_VEgoMin_mps);
  pCTAGlobal->bRCTAFunctionOutput = CTAProcessSetFuncionOutput(
      params->CTA_Ks_RCTAAlgoParameter_nu.CTA_Kb_Active_nu,
      bRCTAFunctionOutputActive, bEgoSpeedCondition);
  for (uint8 uWarnLevel = 0; uWarnLevel < CTA_RCTA_CFG_NUM_OF_WARN_LEVELS;
       uWarnLevel++) {
    switch (uWarnLevel) {
      case (uint8)CTA_RCTA_WARN_LEVEL_ONE:
        // check whether the warning is active and the whether it's not
        // interrupted
        if (pCTAGlobal->CTARCTAOutput.bRCTAWarnActive[uWarnLevel] &&
            !pCTAGlobal->CTARCTAOutput.bWarningInterrupt) {
          bTempWarning = TRUE;
        } else {
          bTempWarning = FALSE;
        }
        bTempWarning = CTAProcessSetWarningOutput(
            pCTAGlobal->bRCTAFunctionOutput, bTempWarning);
        // Set CTAState values for RCTA warning
        if (bTempWarning) {
          proPorts->bRCTAWarning = TRUE;
          proPorts->fRCTAfTTC_s = pCTAGlobal->CTARCTAOutput.fCriticalTTC_s;
        } else {
          proPorts->bRCTAWarning = FALSE;
          proPorts->fRCTAfTTC_s = TUE_C_F32_VALUE_INVALID;
        }
        break;
      case (uint8)CTA_RCTA_WARN_LEVEL_TWO:
        // set CTAState value for L2 RCTA warning
        proPorts->bRCTAWarningL2 = CTAProcessSetWarningOutput(
            pCTAGlobal->bRCTAFunctionOutput,
            pCTAGlobal->CTARCTAOutput.bRCTAWarnActive[uWarnLevel]);
        // Set CTAState values for RCTA warning
        if (proPorts->bRCTAWarningL2) {
          proPorts->fRCTAfTTC_s = pCTAGlobal->CTARCTAOutput.fCriticalTTC_s;
        } else {
          proPorts->fRCTAfTTC_s = TUE_C_F32_VALUE_INVALID;
        }
        break;
      // case (uint8)CTA_RCTA_WARN_LEVEL_THREE:
      //   proPorts->bRCTAWarningL3 = CTAProcessSetWarningOutput(
      //       pCTAGlobal->bRCTAFunctionOutput,
      //       pCTAGlobal->CTARCTAOutput.bRCTAWarnActive[uWarnLevel]);
      //   break;
      default:
        break;
    }
  }

  // FCTA
  if (reqPorts->bFCTAFunctionActive && reqPorts->bFCTAFunctionOutputActive) {
    bFCTAFunctionOutputActive = TRUE;
  }
  bEgoSpeedCondition = CTAProcessCheckEgoSpeedRange(
      reqPorts->EgoVehicleInfo.fegoVelocity_mps,
      params->CTA_Ks_FCTAAlgoParameter_nu.CTA_Kf_VEgoMax_mps,
      params->CTA_Ks_FCTAAlgoParameter_nu.CTA_Kf_VEgoMin_mps);
  pCTAGlobal->bFCTAFunctionOutput = CTAProcessSetFuncionOutput(
      params->CTA_Ks_FCTAAlgoParameter_nu.CTA_Kb_Active_nu,
      bFCTAFunctionOutputActive, bEgoSpeedCondition);
  for (uint8 uWarnLevel = 0; uWarnLevel < CTA_FCTA_CFG_NUM_OF_WARN_LEVELS;
       uWarnLevel++) {
    switch (uWarnLevel) {
      case 0:
        // check whether the warning is active and the whether it's not
        // interrupted
        if (pCTAGlobal->CTAFCTAOutput.bFCTAWarnActive[uWarnLevel]) {
          bTempWarning = TRUE;
        } else {
          bTempWarning = FALSE;
        }
        bTempWarning = CTAProcessSetWarningOutput(
            pCTAGlobal->bFCTAFunctionOutput, bTempWarning);
        // Set CTAState values for RCTA warning
        if (bTempWarning) {
          proPorts->bFCTAWarning = TRUE;
          proPorts->fFCTAfTTC_s = pCTAGlobal->CTAFCTAOutput.fCriticalTTC_s;
        } else {
          proPorts->bFCTAWarning = FALSE;
          proPorts->fFCTAfTTC_s = TUE_C_F32_VALUE_INVALID;
        }
        break;

      case 1:
        // set CTAState value for L2 RCTA warning
        proPorts->bFCTAWarning2 = CTAProcessSetWarningOutput(
            pCTAGlobal->bFCTAFunctionOutput,
            pCTAGlobal->CTAFCTAOutput.bFCTAWarnActive[uWarnLevel]);
        // Set CTAState values for RCTA warning
        if (proPorts->bFCTAWarning2) {
          proPorts->fFCTAfTTC_s = pCTAGlobal->CTAFCTAOutput.fCriticalTTC_s;
        } else {
          proPorts->fFCTAfTTC_s = TUE_C_F32_VALUE_INVALID;
        }
        break;
      default:
        break;
    }
  }

  // CTA
  for (uint8 uObj = 0U; uObj < CTA_MAX_NUM_OBJECTS; uObj++) {
    if ((!reqPorts->CTAEMSRRObjList[uObj].uiMaintenanceState_nu) ==
        CTA_EM_GEN_OBJECT_MT_STATE_DELETED) {
      pCTAGlobal->CTAObjectList[uObj].fXLastCycle_met =
          reqPorts->CTAEMSRRObjList[uObj].fDistX_met;
      pCTAGlobal->CTAObjectList[uObj].fYLastCycle_met =
          reqPorts->CTAEMSRRObjList[uObj].fDistY_met;
    }
  }
}

/*****************************************************************************
  Functionname: CTAProcessCheckEgoSpeedRange */ /*!

  @brief Check whether ego vehicle speed is sufficient

  @description Check whether ego vehicle speed is sufficient

  @param[in]  fEgoSpeed
              fMinEgoSpeed
			  fMaxEgoSpeed
  @param[out]

  @return bEgoSpeedCondition
*****************************************************************************/
boolean CTAProcessCheckEgoSpeedRange(float32 fEgoSpeed, float32 fMaxEgoSpeed,
                                     float32 fMinEgoSpeed) {
  boolean bEgoSpeedCondition = FALSE;
  if (fEgoSpeed > fMinEgoSpeed && fEgoSpeed < fMaxEgoSpeed) {
    bEgoSpeedCondition = TRUE;
  }
  return bEgoSpeedCondition;
}

/*****************************************************************************
  Functionname: CTAProcessSetFuncionOutput                                    */ /*!

  @brief Check whether ego vehicle speed is sufficient

  @description Check whether ego vehicle speed is sufficient

  @param[in]  bFunctionEnabled: flag of algorithm parameters function output
              bFunctionActive: flag of function output switch
			  bEgoSpeedCondition: flag of speed enable
  @param[out]

  @return bFunctionOutput: flag of function output
*****************************************************************************/
boolean CTAProcessSetFuncionOutput(boolean bFunctionEnabled,
                                   boolean bFunctionActive,
                                   boolean bEgoSpeedCondition) {
  boolean bFunctionOutput = FALSE;
  // All conditions have to be fulfilled to set the function output
  if (bFunctionActive && bFunctionEnabled && bEgoSpeedCondition) {
    bFunctionOutput = TRUE;
  }
  return bFunctionOutput;
}

/*****************************************************************************
  Functionname: CTAProcessSetWarningOutput                                    */ /*!

  @brief Check whether ego vehicle speed is sufficient

  @description Check whether ego vehicle speed is sufficient

  @param[in]  bFunctionOutputEnabled: flag of function output
              bInternalWarning: RCTA algorithm warning
  @param[out] 

  @return bWarningoutput: CTA process warning
*****************************************************************************/
boolean CTAProcessSetWarningOutput(boolean bFunctionOutputEnabled,
                                   boolean bInternalWarning) {
  boolean bWarningoutput = FALSE;
  if (bFunctionOutputEnabled && bInternalWarning) {
    bWarningoutput = TRUE;
  }
  return bWarningoutput;
}