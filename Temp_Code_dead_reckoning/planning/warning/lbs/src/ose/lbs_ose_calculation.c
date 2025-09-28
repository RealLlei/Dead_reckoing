/*****************************************************************************
        INCLUDES
*****************************************************************************/
#include "planning/warning/lbs/src/ose/lbs_ose_calculation.h"

#include "planning/warning/lbs/src/lbs_par.h"

//#include "lbs_ose.h"

/*****************************************************************************
        VARIABLES
*****************************************************************************/

/*****************************************************************************
  Functionname: LBSOSESetParameters                                         */ /*!

  @brief: Calculate sensor offset to side and rear edge

  @description: Calculate sensor offset to side and rear edge

  @param[in]  params                                 Algo parameters of OSE function
              pfLeftSensorOffsetToSide_met           Distance between mounting position of the left sensor and vehicle left side
			  pfRightSensorOffsetToSide_met          Distance between mounting position of the right sensor and vehicle right side
  @param[out] void

  @return
*****************************************************************************/
void LBSOSESetParameters(const OSEParam_t* params,
                         float32* pfLeftSensorOffsetToSide_met,
                         float32* pfRightSensorOffsetToSide_met,
                         float32* fLeftSensorOffsetToRear_met,
                         float32* fRightSensorOffsetToRear_met) {
  *pfLeftSensorOffsetToSide_met = 0.5F * params->VehParAdd.fVehicleWidth_met -
                                  params->SensorMounting.SensorLeft.fLatPos_met;
  *pfRightSensorOffsetToSide_met =
      -0.5F * params->VehParAdd.fVehicleWidth_met -
      params->SensorMounting.SensorRight.fLatPos_met;

  *fLeftSensorOffsetToRear_met = params->VehParAdd.fVehicleLength_met -
                                 params->VehParAdd.fOverhangFront_met +
                                 params->SensorMounting.SensorLeft.fLatPos_met;
  *fRightSensorOffsetToRear_met =
      params->VehParAdd.fVehicleLength_met -
      params->VehParAdd.fOverhangFront_met +
      params->SensorMounting.SensorRight.fLatPos_met;
}

/*****************************************************************************
  Functionname: LBSOSEInitCyclic                                         */ /*!

  @brief: OSE cyclic initialization

  @description: Initialize global variables which are not stored across cycles

  @param[in]  pOSEGlobal       OSE global variable structure
  @param[out] void

  @return
*****************************************************************************/
void LBSOSEInitCyclic(OSEGlobal_t* pOSEGlobal) {
  pOSEGlobal->MultiObj.fCriticalObjDistXLastCycle =
      pOSEGlobal->MultiObj.uCriticalObjID;
  pOSEGlobal->MultiObj.uCriticalObjID = TUE_C_UI8_VALUE_INVALID;
  pOSEGlobal->MultiObj.fCriticalObjDistXLastCycle =
      pOSEGlobal->MultiObj.fCriticalObjDistX;
  pOSEGlobal->MultiObj.fCriticalObjDistX = TUE_C_F32_VALUE_INVALID;
  pOSEGlobal->MultiObj.fCriticalTTC = TUE_C_F32_VALUE_INVALID;

  for (uint8 uWarnLevel = 0U; uWarnLevel < OSE_NUM_OF_WARN_LEVELS;
       uWarnLevel++) {
    pOSEGlobal->bOSEWarnActive[uWarnLevel] = FALSE;
  }
}

/*****************************************************************************
  Functionname: LBSOSECalculateYBreakthroughLimit */ /*!

  @brief: Set necessary algo parameters for OSE decision (3 warn level and 2 crossing line)

  @description: Set necessary algo parameters for OSE decision (3 warn level and 2 crossing line)

  @param[in]  bRightSensor                           Flag whether the object is detected by the right sensor
              fLeftSensorOffsetToSide_met            Distance between mounting position of the left sensor and vehicle left side
			  fRightSensorOffsetToSide_met           Distance between mounting position of the right sensor and vehicle right side
			  params                                 Algo parameters of OSE function

  @param[out] fYMinBreakthrough[0]                   The min YBreakthrough at the position of crossing line 0
              fYMaxBreakthrough[0]                   The max YBreakthrough at the position of crossing line 0
			  fYMinBreakthrough[1]					 The min YBreakthrough at the position of crossing line 1
			  fYMaxBreakthrough[1]					 The max YBreakthrough at the position of crossing line 1
  @return
*****************************************************************************/
void LBSOSECalculateYBreakthroughLimit(
    boolean bRightSensor, float32 fLeftSensorOffsetToSide_met,
    float32 fRightSensorOffsetToSide_met, const OSEParam_t* params,
    OSE_ParameterLevelGlobal_t* pParameterLevel) {
#ifdef ST_PERCEPTION
  for (uint8 uWarnLevel = 0; uWarnLevel < OSE_NUM_OF_WARN_LEVELS;
       uWarnLevel++) {
    if (!bRightSensor) {
      // Calculate the left limit value of YBreakthrough, including crossing
      // line 0 and crossing line 1

      // line 0:
      // pParameterLevel[uWarnLevel].fYMinBreakthrough[0u] = 0 + 0 - 1 = -1;
      // pParameterLevel[uWarnLevel].fYMaxBreakthrough[0u] = 0 + 0 + 0.5 = 0.5;
      pParameterLevel[uWarnLevel].fYMinBreakthrough[0u] =
          params->fYMinBreakthrough_met[0u] + fLeftSensorOffsetToSide_met -
          params->fYMinBreakthroughMargin_met[0u];
      pParameterLevel[uWarnLevel].fYMaxBreakthrough[0u] =
          params->fYMaxBreakthrough_met[0u] + fLeftSensorOffsetToSide_met +
          params->fYMaxBreakthroughMargin_met[0u];

      // line 1:
      // pParameterLevel[uWarnLevel].fYMinBreakthrough[1u] = 0 + 0 - 1 = -1;
      // pParameterLevel[uWarnLevel].fYMaxBreakthrough[1u] = 0 + 0 + 0.5 = 0.5;
      pParameterLevel[uWarnLevel].fYMinBreakthrough[1u] =
          params->fYMinBreakthrough_met[1u] + fLeftSensorOffsetToSide_met -
          params->fYMinBreakthroughMargin_met[1u];
      pParameterLevel[uWarnLevel].fYMaxBreakthrough[1u] =
          params->fYMaxBreakthrough_met[1u] + fLeftSensorOffsetToSide_met +
          params->fYMaxBreakthroughMargin_met[1u];
    } else {
      // Calculate the right limit value of YBreakthrough, including crossing
      // line 0 and crossing line 1 Mirror right YBreakthrough limit value to
      // left
      pParameterLevel[uWarnLevel].fYMinBreakthrough[0u] =
          params->fYMinBreakthrough_met[0u] - fRightSensorOffsetToSide_met -
          params->fYMinBreakthroughMargin_met[0u];
      pParameterLevel[uWarnLevel].fYMaxBreakthrough[0u] =
          params->fYMaxBreakthrough_met[0u] - fRightSensorOffsetToSide_met +
          params->fYMaxBreakthroughMargin_met[0u];
      pParameterLevel[uWarnLevel].fYMinBreakthrough[1u] =
          params->fYMinBreakthrough_met[1u] - fRightSensorOffsetToSide_met -
          params->fYMinBreakthroughMargin_met[1u];
      pParameterLevel[uWarnLevel].fYMaxBreakthrough[1u] =
          params->fYMaxBreakthrough_met[1u] - fRightSensorOffsetToSide_met +
          params->fYMaxBreakthroughMargin_met[1u];
    }
  }
#endif  // ST_PERCEPTION
  for (uint8 uWarnLevel = 0; uWarnLevel < OSE_NUM_OF_WARN_LEVELS;
       uWarnLevel++) {
    if (!bRightSensor) {
      // Calculate the left limit value of YBreakthrough, including crossing
      // line 0 and crossing line 1
      pParameterLevel[uWarnLevel].fYMinBreakthrough[0U] =
          params->fYMinBreakthrough_met[0U];  // 0.5
      pParameterLevel[uWarnLevel].fYMaxBreakthrough[0U] =
          params->fYMaxBreakthrough_met[0U];  // 2.5

      // line 1:

      pParameterLevel[uWarnLevel].fYMinBreakthrough[1U] =
          params->fYMinBreakthrough_met[1U];  // 0.5
      pParameterLevel[uWarnLevel].fYMaxBreakthrough[1U] =
          params->fYMaxBreakthrough_met[1U];  // 2.5
    } else {
      // Calculate the right limit value of YBreakthrough, including crossing
      // line 0 and crossing line 1 Mirror right YBreakthrough limit value to
      // left
      pParameterLevel[uWarnLevel].fYMinBreakthrough[0U] =
          -params->fYMaxBreakthrough_met[0U];  //-2.5
      pParameterLevel[uWarnLevel].fYMaxBreakthrough[0U] =
          -params->fYMinBreakthrough_met[0U];  //-0.5
      pParameterLevel[uWarnLevel].fYMinBreakthrough[1U] =
          -params->fYMaxBreakthrough_met[1U];  //-2.5
      pParameterLevel[uWarnLevel].fYMaxBreakthrough[1U] =
          -params->fYMinBreakthrough_met[1U];  //-0.5
    }
  }
}

/*****************************************************************************
  Functionname: LBSOSECalculateDistToCrossingLine */ /*!

  @brief: Calculate OSE object front distance to crossing line

  @description: Calculate OSE object distance to crossing line

  @param[in]  bRightSensor                            Flag whether the object is detected by the right sensor
              fDistX_met                              Object's longitudinal relative distance
			  fLeftSensorOffsetToRear_met             Distance between mounting position of the left sensor and vehicle rear side
			  fRightSensorOffsetToRear_met            Distance between mounting position of the right sensor and vehicle rear side
			  fXBreakthroughLine_met                  Distance between vehicle rear edge and two X-axis breakthrough lines
			  fObjLengthFront_met                     Object's length ahead of the track position               

  @param[out] fDistToCrossingLine_met                 Distance between object front edge and ego vehicle rear edge

  @return
*****************************************************************************/
void LBSOSECalculateDistToCrossingLine(boolean bRightSensor, float32 fDistX_met,
                                       float32 fLeftSensorOffsetToRear_met,
                                       float32 fRightSensorOffsetToRear_met,
                                       float32 fXBreakthroughLine_met,
                                       float32 fObjLengthFront_met,
                                       float32* fDistToCrossingLine_met) {
  float32 fDistToCrossingLine = 0.F;

  if (fDistX_met > 0.0) {
    fDistToCrossingLine =
        fDistX_met - fXBreakthroughLine_met - fObjLengthFront_met;
  } else {
    fDistToCrossingLine =
        fXBreakthroughLine_met - fDistX_met - fObjLengthFront_met;
  }

  *fDistToCrossingLine_met = fDistToCrossingLine;
}

/*****************************************************************************
  Functionname: LBSOSEEstimate_Width                                         */ /*!

  @brief: Estimates the object width based on RCS if the RCS is within reasonable limits

  @description: Estimates the object width based on RCS if the RCS is within reasonable limits

  @param[in] fGenObjWidth_met            Object's overall width
             fRCS                        Object's RCS value
  @param[out] uCounters                  
			  fValue_met                 The object width in the algorithm

  @return
*****************************************************************************/
void LBSOSEEstimate_Width(float32 fGenObjWidth_met, float32 fRCS,
                          uint16* uCounters, float32* fValue_met) {
  const float32 RCSLimitValue[2][2] = {{-15.0F, -2.0F}, {10.0F, 25.0F}};
  const float32 WidthValue[2][2] = {{0.7F, 0.7F}, {1.6F, 1.85F}};
  float32 fEstWidth = fGenObjWidth_met;
  uint8 uObjClassificationIdx = 0;
  uint16 uMaxCounter = 0;

  // Determine whether the object is pedestrian, cyclist and vehicle based on
  // RCS
  if (fRCS > RCSLimitValue[0][0] && fRCS < RCSLimitValue[0][1]) {
    // Check RCS range for pedestrian and cyclist,RCS in [-15,-2]
    fEstWidth = TUE_CML_BoundedLinInterpol2(fRCS, RCSLimitValue[0][0],
                                            RCSLimitValue[0][1],
                                            WidthValue[0][0], WidthValue[0][1]);
    uObjClassificationIdx = OSE_PC_IDX;
  } else {
    if (fRCS > RCSLimitValue[1][0] && fRCS < RCSLimitValue[1][1]) {
      // Check RCS range for vehicle [10,25]
      fEstWidth = TUE_CML_BoundedLinInterpol2(
          fRCS, RCSLimitValue[1][0], RCSLimitValue[1][1], WidthValue[1][0],
          WidthValue[1][1]);
      uObjClassificationIdx = OSE_VEH_IDX;
    } else {
      // no estimation of width is possible
      uObjClassificationIdx = OSE_NOESTIM_IDX;
    }
  }

  // Determine maximum counter
  uMaxCounter = uCounters[0];
  for (uint8 uIdx = 1; uIdx < (OSE_NOESTIM_IDX + 1U); uIdx++) {
    if (uCounters[uIdx] > uMaxCounter) {
      uMaxCounter = uCounters[uIdx];
    }
  }

  // Increment the counter and limit it to avoid overflows the drawback of this
  // method is that counters are never decremented It is assumed that the object
  // does not stay in FoV more than 60000 * 0.04 = 2400 sec
  uCounters[uObjClassificationIdx] = TUE_CML_Min(
      OSE_MAX_UPDATE_COUNTER, (1U + uCounters[uObjClassificationIdx]));
  // The max counter can never be 0 because at least one of the counters is
  // incremented in the code above
  if (((float32)uCounters[uObjClassificationIdx] / (float32)uMaxCounter) >=
      0.8F) {
    *fValue_met = fEstWidth;
  }  // else there is no change in the estimated width of the object

  // The estimated width should never be smaller than the EM width for
  // non-pedestrian objects
  if (fEstWidth < fGenObjWidth_met) {
    *fValue_met = fGenObjWidth_met;
  }  // else there is no change in the estimated width of the object
}

/*****************************************************************************
  Functionname: LBSOSECalculateYBreakthrough */ /*!

  @brief: Calculate y-axis breakthrough of this object

  @description: Calculate y-axis breakthrough of this object, also check its start position

  @param[in]  pEMGenObjInReq                                     EM information of object
              pOSEObjGlobal                                      OSE global variable structure pointer
			  uline                                              Index of crossing line
  @param[out] pOSEObjGlobal->fYBreakthrough[uline]               Object's YBreakthrough in the corresponding crossing line
              pOSEObjGlobal->fYBreakthroughStd[uline]            Standard deviation YBreakthrough in the corresponding crossing line

  @return
*****************************************************************************/
void LBSOSECalculateYBreakthrough(const OSEGenObjectInReq_t* pEMGenObjInReq,
                                  OSEObjInfoArrayGlobal_t* pOSEObjGlobal,
                                  const uint8 uline) {
  float32 fOneSideWidth = 0.F;
  float32 fHeading = 0.F;
  float32 fYBreakthrough = TUE_C_F32_VALUE_INVALID;
  float32 fTanHeading = 0.F;
  float32 fYBreakthrough_fDistY = 0.F;
  float32 fYBreakthrough_fHeading = 0.F;
  float32 fYBreakthrough_fDistToCrossingLine = 0.F;
  float32 fYBreakthroughStd = TUE_C_F32_VALUE_INVALID;

  // For objects which move towards the Y axis calculate the breakthrough
  // fAbsOrientation_rad = 2.8 in [-3.14,3.14] ringt rear -138 degree
  // -----JOOOOOOOO
  if (pEMGenObjInReq->fAbsOrientation_rad < (TUE_CML_Pi + TUE_C_F32_DELTA) &&
      pEMGenObjInReq->fAbsOrientation_rad > -(TUE_CML_Pi + TUE_C_F32_DELTA)) {
    // if (!pEMGenObjInReq->bRightSensor) {
    fOneSideWidth = pEMGenObjInReq->fWidthRight_met;
    // Use the orientation values because it represents the absolute heading
    // of the target
    fHeading = pEMGenObjInReq->fAbsOrientation_rad;
    fYBreakthrough = pEMGenObjInReq->fDistY_met;
    // } else {
    //   fOneSideWidth = pEMGenObjInReq->fWidthLeft_met;
    //   fHeading = -pEMGenObjInReq->fAbsOrientation_rad;
    //   fYBreakthrough = -pEMGenObjInReq->fDistY_met;
    // }

#ifdef ST_PERCEPTION
    // if the estimated width is significantly higher than the current width use
    // it instead
    if (pOSEObjGlobal->fEstWidth.fValue_met - pEMGenObjInReq->fWidth_met >
        0.5F) {
      fOneSideWidth = pOSEObjGlobal->fEstWidth.fValue_met * fOneSideWidth /
                      SafeDiv(pEMGenObjInReq->fWidth_met);
    }
#endif

    if (!pEMGenObjInReq->bRightSensor) {
      fYBreakthrough -= fOneSideWidth;
    } else {
      fYBreakthrough += fOneSideWidth;
    }

    if (TUE_CML_Abs(fHeading) > OSE_MIN_HEADING_PRED &&  // 0.0872
        TUE_CML_Abs(fHeading) < TUE_CML_Pi - OSE_MIN_HEADING_PRED &&
        pEMGenObjInReq->fAbsOrientationStd_rad < 20.0F) {
      fTanHeading = TAN_(fHeading);
      if (pEMGenObjInReq->fDistX_met > 0) {
        fTanHeading = -fTanHeading;
      }
      fYBreakthrough +=
          fTanHeading * pOSEObjGlobal->fDistToCrossingLine_met[uline];

      // Calculate the Std
      /*The error propagation is calculated using following formula:
        q = f(x1,...,xn)
        the uncertainty is calculated:
        q_std = sqrt((dq/dx1 * x1_std)^2 + .... + (dq/dxn * xn_std)^2)*/
      // Calculate the partial derivatives of the breakthrough calculation
      // formula
      fYBreakthrough_fDistY = 1.0F;
      fYBreakthrough_fHeading =
          (2.0F * pOSEObjGlobal->fDistToCrossingLine_met[uline]) /
          SafeDiv(COS_(2.0F * fHeading) + 1.0F);
      fYBreakthrough_fDistToCrossingLine = fTanHeading;
      // Calculate the square sum of the partial derivatives times the
      // corresponding standard deviation
      fYBreakthroughStd =
          SQR(fYBreakthrough_fDistY * pEMGenObjInReq->fDistYStd_met);
      fYBreakthroughStd +=
          SQR(fYBreakthrough_fHeading * pEMGenObjInReq->fAbsOrientationStd_rad);
      fYBreakthroughStd += SQR(fYBreakthrough_fDistToCrossingLine *
                               pEMGenObjInReq->fDistXStd_met);
      // Calculate the root
      fYBreakthroughStd = sqrt(fYBreakthroughStd);
    } else {
      fYBreakthroughStd = pEMGenObjInReq->fDistYStd_met;
    }
    // Limit the value to reasonable limits
    fYBreakthroughStd = MIN(fYBreakthroughStd, TUE_C_F32_VALUE_INVALID);
  }
  pOSEObjGlobal->fYBreakthrough[uline] = fYBreakthrough;
  pOSEObjGlobal->fYBreakthroughStd[uline] = fYBreakthroughStd;
}

/*****************************************************************************
  Functionname:  LBSOSECalculateTTC                                        */ /*!

  @brief: Calculate TTC to computed breakthrough

  @description: Calculate TTC to computed breakthrough using constant speed assumption

  @param[in] void
  @param[out] void

  @return
*****************************************************************************/
void LBSOSECalculateTTC(float32 fCycletime_s, float32 fDistX_met,
                        float32 fVrelX_mps, float32 fDistToCrossingLine_met,
                        float32* pfTTC_s, float32* pfTTCFiltered_s) {
  float32 fTTC = 0.F;
  float32 fTTCFiltered = *pfTTCFiltered_s;
  float32 fFilterSpeed = 0.F;
  // Calculate the TTC
  if (fDistX_met > 0) {
    fTTC = fDistToCrossingLine_met / SafeDiv(-fVrelX_mps);
  } else {
    fTTC = fDistToCrossingLine_met / SafeDiv(fVrelX_mps);
  }

  // If the TTC is negative or when the object already passed the crossing line
  if (fTTC < 0.0F ||
      fDistToCrossingLine_met < 0.0F) {  // !For front obstacle shall DOW work?
    fTTC = TUE_C_F32_VALUE_INVALID;
  }
  // For large TTCs set TTC filtered to TTC
  if (fTTC > 30.0F || fTTCFiltered > 30.0F) {
    fTTCFiltered = fTTC;
  } else {
    // Predicted the TTC and limit it to be positive
    //float32 a = fTTCFiltered - fCycletime_s;
    fTTCFiltered = MAX(0.0F, fTTCFiltered - fCycletime_s);
    fFilterSpeed = GDBmathLinFuncLimBounded(fDistToCrossingLine_met, 0.0F,
                                            15.0F, 0.1F, 0.3F);
    // Filter the TTC
    GDB_Math_LowPassFilter(&fTTCFiltered, fTTC, fFilterSpeed);
  }
  *pfTTC_s = fTTC;
  *pfTTCFiltered_s = fTTCFiltered;
}

#ifdef ST_PERCEPTION
void LBSOSECalculateTTC(float32 fCycletime_s, float32 fVrelX_mps,
                        float32 fDistToCrossingLine_met, float32* pfTTC_s,
                        float32* pfTTCFiltered_s) {
  float32 fTTC;
  float32 fTTCFiltered = *pfTTCFiltered_s;
  float32 fFilterSpeed;
  // Calculate the TTC
  fTTC = fDistToCrossingLine_met / SafeDiv(fVrelX_mps);
  // If the TTC is negative or when the object already passed the crossing line
  if (fTTC < 0.0f ||
      fDistToCrossingLine_met < 0.0f) {  // !For front obstacle shall DOW work?
    fTTC = TUE_C_F32_VALUE_INVALID;
  }
  // For large TTCs set TTC filtered to TTC
  if (fTTC > 30.0f || fTTCFiltered > 30.0f) {
    fTTCFiltered = fTTC;
  } else {
    // Predicted the TTC and limit it to be positive
    float32 a = fTTCFiltered - fCycletime_s;
    fTTCFiltered = MAX(0.0F, fTTCFiltered - fCycletime_s);
    fFilterSpeed = GDBmathLinFuncLimBounded(fDistToCrossingLine_met, 0.0F,
                                            15.0F, 0.1F, 0.3f);
    // Filter the TTC
    GDB_Math_LowPassFilter(&fTTCFiltered, fTTC, fFilterSpeed);
  }
  *pfTTC_s = fTTC;
  *pfTTCFiltered_s = fTTCFiltered;
}
#endif
/*****************************************************************************
  Functionname: LBSOSECalculateFrontObjectProbability */ /*!

  @brief: Calculate a probability value for this object to be a front object,by looking for objects 
          in front of it.

  @description: Calculate a probability value for this object to be a front object,by looking for objects
		  in front of it.

  @param[in]  uObj                    The serial number            
              reqPorts                Inputs structure of OSE
			  pOSEGlobal              OSE global variables pointer
			  pfSideTrackProb         The probability of the side object in the last cycle
  @param[out] pfSideTrackProb         The probability of the side object in the current cycle

  @return
*****************************************************************************/
void LBSOSECalculateFrontObjectProbability(uint8 uObj,
                                           const OSEInReq_t* reqPorts,
                                           OSEGlobal_t* pOSEGlobal,
                                           float32* pfSideTrackProb) {
  float32 bAbortLoop = FALSE;
  float32 fXTemp = 0.F;
  float32 fYTemp = 0.F;
  float32 fTemp = 0.F;
  const OSEGenObjectInReq_t* pCurrObject =
      &reqPorts->EMGenObjList.aObject[uObj];
  const OSEGenObjectInReq_t* pFrontObjCand = NULL;
  OSEFrontObjState_t eSideObjState = OSE_FRONTOBJ_NO;
  OSEObjInfoArrayGlobal_t* pOSEFrontObjCandInfo = NULL;
  float32 fProbChange = 0.0F;

  // Loop over all objects
  for (uint8 uIndex = 0U; (uIndex < OSE_LBS_NUM_OBJECTS) && (!bAbortLoop);
       uIndex++) {
    if ((reqPorts->EMGenObjList.aObject[uIndex].eMaintenanceState !=
         EM_GEN_OBJ_MT_STATE_DELETED) &&
        uObj != uIndex) {
      pFrontObjCand = &reqPorts->EMGenObjList.aObject[uIndex];
      pOSEFrontObjCandInfo = &pOSEGlobal->OSEObjInfoArray[uIndex];
      fXTemp = pCurrObject->fDistX_met - pFrontObjCand->fDistX_met;
      if (!pCurrObject->bRightSensor) {
        fYTemp = pCurrObject->fDistY_met - pFrontObjCand->fDistY_met;
      } else {
        fYTemp = -(pCurrObject->fDistY_met - pFrontObjCand->fDistY_met);
      }

      // If the object is further away or behind the search object
      if (fXTemp > 10.0f || fYTemp < 0.0F) {
        // This object is not in the area of interest check the next one
      } else {
        if (fXTemp < -10.0F) {
          // This and all following objects are in front of the area of interest
          // we can abort the loop
          bAbortLoop = TRUE;
        } else {
          if (pOSEFrontObjCandInfo->bRelevant &&
              !pOSEFrontObjCandInfo->bSideTrack && fXTemp > 0.0F) {
            eSideObjState = LBSOSECheckFrontOject(
                &reqPorts->EMGenObjList.aObject[uObj],
                &reqPorts->EMGenObjList.aObject[uIndex],
                &reqPorts->OSELBSGlobalInReq.OSELBSObjInfoArray[uObj]
                     .ObjBorders,
                &reqPorts->OSELBSGlobalInReq.OSELBSObjInfoArray[uIndex]
                     .ObjBorders,
                *pfSideTrackProb);
          }
        }
      }
    }
  }
  // Adapt the front object probability
  switch (eSideObjState) {
    case OSE_FRONTOBJ_POS_SPEED:
      // The condition is completely fulfilled: Increase the probability
      if (*pfSideTrackProb > OSE_FRONTOBJ_IMPLAUSIBLE_THRESH)  // 0.05
      {
        // Reduce the max probability change for old objects
        fTemp = TUE_CML_BoundedLinInterpol2(
            (float32)pCurrObject->uiLifeCycles_nu, 50.0F, 0.0F, 0.0F,
            OSE_FRONTOBJ_INC_PROB);  // 0.1
        // Reduce the max probability change for objects near the x-axis
        fProbChange = TUE_CML_BoundedLinInterpol2(
            pCurrObject->fDistX_met, 0.0F, 5.0F, 0.0F, OSE_FRONTOBJ_INC_PROB);
        // Use the minimum of all limitations
        fProbChange = MIN(fProbChange, fTemp);
      }
      break;
    case OSE_FRONTOBJ_POS:
      // The condition is partly fulfilled: keep the probability
      break;
    case OSE_FRONTOBJ_NO:
      // The condition is not fulfilled: Decrease the probability
      fProbChange = -OSE_FRONTOBJ_DEC_PROB;  // 0.02
      break;
    default:
      // Do nothing
      break;
  }
  // Update the probability
  *pfSideTrackProb += fProbChange;
  *pfSideTrackProb = TUE_CML_MinMax(0.0F, 1.0F, *pfSideTrackProb);
}

/*****************************************************************************
  Functionname: LBSOSECheckFrontOject                                         */ /*!

  @brief: Checks if two objects are close to each other with one of the objects in front of the other

  @description: Checks if two objects are close to each other with one of the objects in front of the other

  @param[in]  pEMObjInfo                 Object's EM information pointer
              pEMFrontCandObjInfo        Front candidate object's EM information pointer
			  pObjBorder                 Object's border information pointer
			  pFrontCandObjBorder        Front candidate object's border information pointer
			  fSideTrackProbability      The probability of the side object 
  @param[out] void

  @return     eObjFrontState             The check state of the front candidate object position and speed 
*****************************************************************************/
OSEFrontObjState_t LBSOSECheckFrontOject(
    const OSEGenObjectInReq_t* pEMObjInfo,
    const OSEGenObjectInReq_t* pEMFrontCandObjInfo,
    const OSELBSObjBorders_t* pObjBorder,
    const OSELBSObjBorders_t* pFrontCandObjBorder,
    float32 fSideTrackProbability) {
  float32 fXDiff = 0.F;
  float32 fYDiff = 0.F;
  float32 fVxDiff = 0.F;
  float32 fVyDiff = 0.F;
  float32 fMaxDistXDiff = 0.F;
  float32 fMaxDistYDiff = 0.F;
  float32 fMaxRangeDiff = 0.F;
  OSEFrontObjState_t eObjFrontState = OSE_FRONTOBJ_NO;
  float32 fMaxVxDiff = 0.F;
  float32 fMaxVyDiff = 0.F;

  // Calculate object position difference
  fXDiff = LBSOSECalculateObjectDistance(
      pObjBorder->fXmin_met, pObjBorder->fXmax_met,
      pFrontCandObjBorder->fXmin_met, pFrontCandObjBorder->fXmax_met);
  fYDiff = LBSOSECalculateObjectDistance(
      pObjBorder->fYmin_met, pObjBorder->fYmax_met,
      pFrontCandObjBorder->fYmin_met, pFrontCandObjBorder->fYmax_met);
  // Calculate object velocity difference
  fVxDiff = fABS(pEMFrontCandObjInfo->fVrelX_mps - pEMObjInfo->fVrelX_mps);
  fVyDiff = fABS(pEMFrontCandObjInfo->fVrelY_mps - pEMObjInfo->fVrelY_mps);
  // Calculate max differences. Assume that two following vehicles have at least
  // LBS_OSE_MULTIOBJ_SAFE_MARGIN security distance
  fMaxRangeDiff = sqrt(SQR(pEMFrontCandObjInfo->fVabsX_mps) +
                       SQR(pEMFrontCandObjInfo->fVabsY_mps));
  // Multiply with the assumed security distance s = v * t
  fMaxRangeDiff *= OSE_MULTIOBJ_SAFE_MARGIN;  // 0.7
  // Calculate the limits depending on the heading angle and limit the
  // thresholds
  fMaxDistXDiff =
      fABS(COS_(pEMFrontCandObjInfo->fAbsOrientation_rad) *
           fMaxRangeDiff);  // fMaxDistXDiff = fABS(COS_(0.5 * TUE_CML_Pi -
                            // pEMFrontCandObjInfo->fAbsOrientation_rad) *
                            // fMaxRangeDiff);
  fMaxDistXDiff = TUE_CML_MinMax(2.0F, 10.0F, fMaxDistXDiff);
  fMaxDistYDiff =
      fABS(SIN_(pEMFrontCandObjInfo->fAbsOrientation_rad) * fMaxRangeDiff);
  fMaxDistYDiff = TUE_CML_MinMax(2.0F, 10.0F, fMaxDistYDiff);
  // Check front object condition
  if (fXDiff < fMaxDistXDiff && fYDiff < fMaxDistYDiff) {
    // Position criteria is fulfilled
    eObjFrontState = OSE_FRONTOBJ_POS;
    // Set velocity threshold
    fMaxVxDiff = TUE_CML_BoundedLinInterpol2(fSideTrackProbability, 0.0F, 1.0F,
                                             1.0F, 15.0F);
    fMaxVyDiff = TUE_CML_BoundedLinInterpol2(fSideTrackProbability, 0.0F, 1.0F,
                                             1.0F, 15.0F);
    // Check speed criteria
    if (fVxDiff < fMaxVxDiff && fVyDiff < fMaxVyDiff) {
      eObjFrontState = OSE_FRONTOBJ_POS_SPEED;
    }
  }
  // return rear track bool
  return eObjFrontState;
}

/*****************************************************************************
  Functionname: LBSOSECalculateObjectDistance */ /*!

  @brief: Calculate the distance of two objects

  @description: Calculate the distance of two objects in X or Y direction, using their dimensions

  @param[in]   fMinObj               The object x or y min position
               fMaxObj               The object x or y max position
			   fMinFrontCandObj      The front candidate object x or y min position
			   fMaxFrontCandObj      The front candidate object x or y max position
  @param[out]  void

  @return      fDistance             Min distance between object and front candidate
*****************************************************************************/
float32 LBSOSECalculateObjectDistance(float32 fMinObj, float32 fMaxObj,
                                      float32 fMinFrontCandObj,
                                      float32 fMaxFrontCandObj) {
  float32 fDistance = 0.0F;
  const float32 fDiffBA = fABS(fMaxFrontCandObj - fMinObj);
  const float32 fDiffAB = fABS(fMinFrontCandObj - fMaxObj);
  // Check the condition for no overlap
  if (fMaxFrontCandObj < fMinObj || fMinFrontCandObj > fMaxObj) {
    fDistance = MIN(fDiffBA, fDiffAB);
  }
  return fDistance;
}

/*****************************************************************************
  Functionname: LBSOSECheckSideObject                                         */ /*!

  @brief: Search for side objects

  @description: Search for objects that are around a main object that

  @param[in]  fSideTrackProb       The probability of the side object
              pbSideTrack          flag whether the object is the side object in the last cycle
  @param[out] pbSideTrack          flag whether the object is the side object in the current cycle 

  @return
*****************************************************************************/
void LBSOSECheckSideObject(float32 fSideTrackProb, boolean* pbSideTrack) {
  boolean bSideTrack = FALSE;
  // use hysteresis
  if (*pbSideTrack) {
    if (fSideTrackProb > OSE_FRONTOBJ_DEACTIVETION_THRESH)  // 0.7
    {
      bSideTrack = TRUE;
    }
  } else {
    if (fSideTrackProb > OSE_FRONTOBJ_ACTIVETION_THRESH)  // 0.9
    {
      bSideTrack = TRUE;
    }
  }
  *pbSideTrack = bSideTrack;
}

/*****************************************************************************
  Functionname: LBSOSECheckObjectUpdateRecently */ /*!

  @brief: Check object update status

  @description: Check object update status

  @param[in]  uiMeasuredTargetFrequency_nu       Bitfield to indicate if the object was measured in the last 8 cycles
  @param[out] pbUpdatedRecently                  Flag whether the object is updated recently 

  @return
*****************************************************************************/
void LBSOSECheckObjectUpdateRecently(uint8 uiMeasuredTargetFrequency_nu,
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
  Functionname: LBSOSECheckObjectInRange */ /*!

  @brief: Check for valid OSE range

  @description: The range check prevents target which may turn away from warning

  @param[in]  fDistX_met                                 Object's longitudinal relative distance
              fDistY_met                                 Object's lateral relative distance
			  fTargetRangeMax                            The max range thresholds of three levels OSE warning
  @param[out] pInfoLevel[uWarnLevel].bObjectInRange      Flag whether the object is in the range of three different level

  @return
*****************************************************************************/
// void LBSOSECheckObjectInRange(
//     float32 fDistX_met, float32 fDistY_met,
//     fTargetRangeMax_arrayParam_t fTargetRangeMax,
//     OSEObjInfoLevel_t* pInfoLevel) {
//   float32 fRangeSqr;
//   float32 bInRange;
//   for (uint8 uWarnLevel = 0u; uWarnLevel < OSE_NUM_OF_WARN_LEVELS;
//        uWarnLevel++) {
//     bInRange = FALSE;
//     // Calculate the distance of the object
//     fRangeSqr = SQR(fDistX_met) + SQR(fDistY_met);
//     // Check whether the object is in range
//     if (fRangeSqr < SQR(fTargetRangeMax[uWarnLevel])) {
//       bInRange = TRUE;
//     }
//     pInfoLevel[uWarnLevel].bObjectInRange = bInRange;
//   }
// }
void LBSOSECheckObjectInRange(float32 fDistX_met, float32 fDistY_met,
                              float32 fXMaxTargetRange_met,
                              float32 fXMinTargetRange_met,
                              OSEObjInfoLevel_t* pInfoLevel) {
  boolean bInRange = FALSE;
  for (uint8 uWarnLevel = 0U; uWarnLevel < OSE_NUM_OF_WARN_LEVELS;
       uWarnLevel++) {
    bInRange = ((fDistX_met < fXMaxTargetRange_met) &&
                (fDistX_met > fXMinTargetRange_met));
    pInfoLevel[uWarnLevel].bObjectInRange = bInRange;
  }
}

/*****************************************************************************
  Functionname: LBSOSECheckRearApproach */ /*!

  @brief: check for valid OSE range

  @description: The range check prevents targets which may turn away

  @param[in]  fDistX_met            Object's longitudinal relative distance
              fFirstDetectX_met     X position where the object was created
			  pbObjectFromRear      Flag whether the object is approaching from the rear in the last cycle or first time
  @param[out] pbObjectFromRear      Flag whether the object is approaching from the rear in the current cycle or first time

  @return
*****************************************************************************/
void LBSOSECheckRearApproach(float32 fDistX_met, float32 fFirstDetectX_met,
                             boolean* pbObjectFromRear) {
  // boolean bRearObject = *pbObjectFromRear; ST
  boolean bRearObject = FALSE;  // HZ

  // If the object is not yet detected as rear objects
  // if (!bRearObject) {
  // Check for the first detection position and the current position
  // if (fFirstDetectX_met < -4.0f || fDistX_met < -8.0F) {
  if (fDistX_met < 0.0F) {
    bRearObject = TRUE;
  }
  // }
  *pbObjectFromRear = bRearObject;
}

/*****************************************************************************
  Functionname: LBSOSECheckApproachAngle */ /*!

  @brief: Check for valid OSE approach angle

  @description: Check for valid OSE approach angle

  @param[in]  fAbsOrientation_rad       Object moving direction,based on VX and VY in AUTOSAR
              bObjectFromRear           Flag whether the object is approaching from the rear in the current cycle or first time
			  fMaxHeadingAngle          The max heading threshold; Value: 60.0f
			  fMinHeadingAngle          The min heading threshold; Value: -70.0f
			  pbValidApproachAngle      Flag whether the object heading angle is valid in the last cycle
  @param[out] pbValidApproachAngle      Flag whether the object heading angle is valid in the current cycle

  @return
*****************************************************************************/
void LBSOSECheckApproachAngle(float32 fAbsOrientation_rad,
                              boolean bObjectFromRear, float32 fMaxHeadingAngle,
                              float32 fMinHeadingAngle,
                              boolean* pbValidApproachAngle) {
  float32 fMAxApproachAngle = fMaxHeadingAngle;  // 60deg
  float32 fMinApproachAngle = fMinHeadingAngle;  //-70deg
  boolean bValidApproachAngle = FALSE;
  float32 fM_PI = 3.14159F;
  if (*pbValidApproachAngle) {
    // Add hysteresis
    fMAxApproachAngle += OSE_APPROACH_ANGLE_HYST;  // 10deg
    fMinApproachAngle -= OSE_APPROACH_ANGLE_HYST;  // 10deg
  }
  // Rear targets use the customer defined parameters for max rear crossing
  // angles
  if (bObjectFromRear) {
    if (fAbsOrientation_rad < DEG2RAD(fMAxApproachAngle) &&
        fAbsOrientation_rad > DEG2RAD(fMinApproachAngle)) {
      bValidApproachAngle = TRUE;
    }
  } else  // to inhibit false alerts, targets which appear/approach from side
          // use tighter angle requirements
  {
    // float32 a = DEG2RAD(fMAxApproachAngle);
    // float32 b = DEG2RAD(fMinApproachAngle + OSE_APPROACH_ANGLE_MARGIN);
    if (fAbsOrientation_rad < (DEG2RAD(fMAxApproachAngle) - fM_PI) ||
        fAbsOrientation_rad >
            (fM_PI + DEG2RAD(fMinApproachAngle + OSE_APPROACH_ANGLE_MARGIN))) {
      bValidApproachAngle = TRUE;
    }
  }
  *pbValidApproachAngle = bValidApproachAngle;
}

/*****************************************************************************
  Functionname: LBSOSECheckForMirror                                        */ /*!

  @brief: Check if object is a mirror

  @description: Check if object is a mirror

  @param[in]  fMirrorProb_per        The probability that the object is mirror
              pbMirror               Flag whether the object is mirror in the last cycle
  @param[out] pbMirror               Flag whether the object is mirror in the current cycle

  @return
*****************************************************************************/
void LBSOSECheckForMirror(float32 fMirrorProb_per, boolean* pbMirror) {
  boolean bMirror = FALSE;
#if ST_PERCEPTION
  if (!(*pbMirror)) {
    if (fMirrorProb_per > OSE_MIRROR_ACTIVATION_THRESH)  // 0.8
    {
      // Object has sufficient mirror probability, set mirror flag
      bMirror = TRUE;
    }
  } else {
    if (fMirrorProb_per > OSE_MIRROR_DEACTIVATION_THRESH)  // 0.5
    {
      // Object has sufficient mirror probability, keep mirror flag
      bMirror = TRUE;
    }
  }
#endif
  *pbMirror = bMirror;
}

/*****************************************************************************
  Functionname: LBSOSECheckObjectRelevance */ /*!

  @brief: Check for object relevance

  @description: Check if the tracked object is relevant (exists) for a warning

  @param[in]  fXMovement_met          The object total moving distance in the x direction
              fYMovement_met          The object total moving distance in the y direction
              fVabsX_mps              Object's longitudinal velocity over ground
			  fVabsY_mps              Object's lateral velocity over ground
			  uiLifeCycles_nu         Object lifetime in cycles
			  fVTargetMin             The min target vehicle speed of OSE is actived; Value: 3.0f m/s
			  pbRelevant              Flag whether the object is relevant for a OSE warning in the last cycle
  @param[out] pbRelevant              Flag whether the object is relevant for a OSE warning in the current cycle

  @return
*****************************************************************************/
void LBSOSECheckObjectRelevance(float32 fXMovement_met, float32 fYMovement_met,
                                float32 fVabsX_mps, float32 fVabsY_mps,
                                uint16 uiLifeCycles_nu, float32 fVTargetMin,
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
    // Threshold for movement over ground decreased to 2.0m (from 4.0m),
    // reason: use case with obscured FoV
    if (fSpeedOverGroundSqr > SQR(fVTargetMin) && fMovementAbsSqr > SQR(2.0F) &&
        uiLifeCycles_nu > LBS_OSE_MIN_LIFETIME_RELEVANT) {  // 2
      bRelevance = TRUE;
    }
  }
  *pbRelevant = bRelevance;
}

/*****************************************************************************
  Functionname: LBSOSECheckObjectQuality */ /*!

  @brief: Check object quality

  @description: Check object quality

  @param[in]  pEMGenObjInReq       Object EM information structure pointer
              fUpdateRate_nu       The object measurement update rate
			  fAssocProbFiltered   Highest cluster association probability of the object filter result
  @param[out] pfQuality            The quality probability of the DOW warning object

  @return
*****************************************************************************/
void LBSOSECheckObjectQuality(const OSEGenObjectInReq_t* pEMGenObjInReq,
                              float32 fUpdateRate_nu,
                              float32 fAssocProbFiltered, float32* pfQuality) {
  float32 fObjQuality = 0.F;
  float32 fPoEThresh = 0.F;
  float32 fPoEThreshLifetime = 0.F;
  float32 fUpdateRateThresh = 0.F;
  float32 fUpdateRateThreshLifetime = 0.F;
  float32 fAssocProbThresh = 0.F;
  float32 fAssocProbThreshRange = 0.F;
  float32 fRangeSqr = 0.F;
  float32 fSpeedOverGroundSqr = 0.F;
  boolean bMaiten = FALSE;

  if (pEMGenObjInReq->eMaintenanceState == EM_GEN_OBJ_MT_STATE_MEASURED) {
    bMaiten = TRUE;
  }
#if ST_PERCEPTION
  // Calculate object's speed over ground square
  fSpeedOverGroundSqr =
      SQR(pEMGenObjInReq->fVabsX_mps) + SQR(pEMGenObjInReq->fVabsY_mps);
  // Calculate PoE threshold
  fPoEThresh = TUE_CML_BoundedLinInterpol2(
      fSpeedOverGroundSqr, SQR(30.0f * TUE_C_MS_KMH), SQR(60.0f * TUE_C_MS_KMH),
      0.99F, 0.93f);
  fPoEThreshLifetime = TUE_CML_BoundedLinInterpol2(
      (float32)pEMGenObjInReq->uiLifeCycles_nu, 15.0F, 25.0F, 0.93F, 0.99f);
  fPoEThresh = MAX(fPoEThresh, fPoEThreshLifetime);
  // Calculate update rate threshold
  fUpdateRateThresh = TUE_CML_BoundedLinInterpol2(
      fSpeedOverGroundSqr, SQR(30.0f * TUE_C_MS_KMH), SQR(60.0f * TUE_C_MS_KMH),
      0.85F, 0.75f);
  fUpdateRateThreshLifetime = TUE_CML_BoundedLinInterpol2(
      (float32)pEMGenObjInReq->uiLifeCycles_nu, 15.0F, 25.0F, 0.75F, 0.85f);
  fUpdateRateThresh = MAX(fUpdateRateThresh, fUpdateRateThreshLifetime);
  // Calculate the association probability threshold
  // The association probability threshold should be speed and distance
  // dependent This makes sure that fast objects are allowed to warn early
  // enough, but mostly close ghost objects do not reach the threshold
  fAssocProbThresh = TUE_CML_BoundedLinInterpol2(
      fSpeedOverGroundSqr, SQR(10.0f * TUE_C_MS_KMH), SQR(50.0f * TUE_C_MS_KMH),
      0.8F, 0.3f);
  fRangeSqr = SQR(pEMGenObjInReq->fDistX_met) + SQR(pEMGenObjInReq->fDistY_met);
  fAssocProbThreshRange = TUE_CML_BoundedLinInterpol2(fRangeSqr, SQR(40.0f),
                                                      SQR(30.0f), 0.40F, 1.00f);
  fAssocProbThresh = MIN(fAssocProbThresh, fAssocProbThreshRange);
  // Apply hysteresis
  if (*pfQuality >= 1.0f) {
    fPoEThresh -= 0.02F;
    fUpdateRateThresh -= 0.1F;
    fAssocProbThresh -= 0.1F;
  }
  // Check all quality criteria
  if (pEMGenObjInReq->fProbabilityOfExistence_per > fPoEThresh &&
      fUpdateRate_nu > fUpdateRateThresh &&
      fAssocProbFiltered > fAssocProbThresh) {
    fObjQuality = 1.0F;
  } else {
    fObjQuality = (pEMGenObjInReq->fProbabilityOfExistence_per +
                   fUpdateRate_nu + fAssocProbFiltered) /
                  (fPoEThresh + fUpdateRateThresh + fAssocProbThresh);
  }
#endif
  //#if HZ_PERCEPTION
  if ((TRUE == bMaiten) &&
      (pEMGenObjInReq->fProbabilityOfExistence_per > LBS_OSE_QUALITY_POE_MIN)) {
    fObjQuality = 1.0F;
  }
  //#endif
  *pfQuality = fObjQuality;
}

/*****************************************************************************
  Functionname: LBSOSECheckBreakthroughHit */ /*!

  @brief: Check for object breakthrough

  @description: Check for object trajectory intersection with rear crossing line

  @param[in]  uLine                            The number of crossing line 
              pfDistToCrossingLine_met         Distance between object front edge and ego vehicle rear edge
			  pfYBreakthrough                  The y-axis breakthrough of the object
			  pParameterLevel                  The max and min edges of breakthrough parameter
  @param[out] pInfoLevel                       DOW warning information structure

  @return
*****************************************************************************/
void LBSOSECheckBreakthroughHit(const uint8 uLine,
                                float32* pfDistToCrossingLine_met,
                                float32* pfYBreakthrough,
                                OSE_ParameterLevelGlobal_t* pParameterLevel,
                                OSEObjInfoLevel_t* pInfoLevel) {
  boolean bBreakthroughHit = FALSE;
  float32 fMaxBreakthrough = 0.F;
  float32 fMinBreakthrough = 0.F;

  for (uint8 uWarnLevel = 0U; uWarnLevel < OSE_NUM_OF_WARN_LEVELS;
       uWarnLevel++) {
    bBreakthroughHit = FALSE;
    fMaxBreakthrough = pParameterLevel[uWarnLevel].fYMaxBreakthrough[uLine];
    fMinBreakthrough = pParameterLevel[uWarnLevel].fYMinBreakthrough[uLine];
    // The object did not pass the crossing line yet
    if (pfDistToCrossingLine_met[uLine] > 0.0F) {
      // Switch on hysteresis
      if (pInfoLevel[uWarnLevel].bWarning) {
        fMaxBreakthrough += OSE_BREAKTHROUGH_HYSTERESIS;  // 0.3
        fMinBreakthrough -= OSE_BREAKTHROUGH_HYSTERESIS;
      }
      // Test breakthrough in range
      if (pfYBreakthrough[uLine] < fMaxBreakthrough &&
          pfYBreakthrough[uLine] > fMinBreakthrough) {
        bBreakthroughHit = TRUE;
      }
    }
    pInfoLevel[uWarnLevel].bBreakthroughHit[uLine] = bBreakthroughHit;
  }
}

/*****************************************************************************
  Functionname: LBSOSECalculateBreakthroughHitConfidence */ /*!

  @brief: Update the confidence of the breakthrough hit confidence

  @description: Update the confidence of the breakthrough hit

  @param[in]  uLine                                                  number of crossing line 
              pInfoLevel[uWarnLevel].uBreakthroughHitConfi[uLine]    Breakthrough hit confidence in the last cycle
			  pfYBreakthrough                                        The y-axis breakthrough of the object
			  pfYBreakthroughStd                                     The y-axis breakthrough standard deviation of the object
			  bObjectAtEdgeFoV                                       Flag whether the object is at the edge of FOV
			  pParameterLevel                                        The max and min edges of breakthrough parameter
			  pInfoLevel                                             DOW warning information structure
  @param[out] pInfoLevel[uWarnLevel].uBreakthroughHitConfi[uLine]    Breakthrough hit confidence in the current cycle

  @return
*****************************************************************************/
void LBSOSECalculateBreakthroughHitConfidence(
    const uint8 uLine, float32* pfYBreakthrough, float32* pfYBreakthroughStd,
    boolean bObjectAtEdgeFoV, OSE_ParameterLevelGlobal_t* pParameterLevel,
    OSEObjInfoLevel_t* pInfoLevel) {
  float32 fHitConfidence = 0.F;
  float32 fHitConfidenceUpdate = 0.F;
  float32 fBreakthroughHitRatio = 0.F;
  float32 fMaxHitLimit = 0.F;
  float32 fMinHitLimit = 0.F;
  float32 fMaxBreakthroughLimit = 0.F;
  float32 fMinBreakthroughLimit = 0.F;
  float32 fMaxConfUpdate = OSE_CONFI_UPDATE_MAX;  // 20 --->30
  float32 fMinConfUpdate = OSE_CONFI_UPDATE_MIN;  // 5  --->25

  for (uint8 uWarnLevel = 0U; uWarnLevel < OSE_NUM_OF_WARN_LEVELS;
       uWarnLevel++) {
    fHitConfidence = pInfoLevel[uWarnLevel].uBreakthroughHitConfi[uLine];
    // Get the min and max point where the object hits the breakthrough line
    fMaxBreakthroughLimit = pfYBreakthrough[uLine] + pfYBreakthroughStd[uLine];
    fMinBreakthroughLimit = pfYBreakthrough[uLine] - pfYBreakthroughStd[uLine];
    fMaxHitLimit = LBSOSECalculateHitLimit(
        fMaxBreakthroughLimit,
        pParameterLevel[uWarnLevel].fYMaxBreakthrough[uLine],
        pParameterLevel[uWarnLevel].fYMinBreakthrough[uLine],
        pInfoLevel[uWarnLevel].bBreakthroughHit[uLine]);
    fMinHitLimit = LBSOSECalculateHitLimit(
        fMinBreakthroughLimit,
        pParameterLevel[uWarnLevel].fYMaxBreakthrough[uLine],
        pParameterLevel[uWarnLevel].fYMinBreakthrough[uLine],
        pInfoLevel[uWarnLevel].bBreakthroughHit[uLine]);
    // Calculate the ratio of the length where the object will probably hit
    // the breakthrough and the uncertainty of the hit estimation
    fBreakthroughHitRatio = (fMaxHitLimit - fMinHitLimit) /
                            SafeDiv(2.0F * pfYBreakthroughStd[uLine]);
    // Calculate min and max confidance update depending on object at the edge
    // of FoV
    if (bObjectAtEdgeFoV) {
      fMaxConfUpdate = OSE_CONFI_UPDATE_MAX_EFOV;  // 10
      fMinConfUpdate = OSE_CONFI_UPDATE_MIN_EFOV;  // 2
    }
    // Breakthrough was not hit
    if (!pInfoLevel->bBreakthroughHit[uLine]) {
      // The confidence should be decreased. The less the object hits the
      // breakthrough the faster the confidence shall decrease
      fHitConfidenceUpdate = TUE_CML_BoundedLinInterpol2(
          fBreakthroughHitRatio, 0.0F, 1.0F, -fMaxConfUpdate, -fMinConfUpdate);
    } else {
      // The confidence should be increased. The more the object hits the
      // breakthrough the faster the confidence shall increase
      fHitConfidenceUpdate = TUE_CML_BoundedLinInterpol2(
          fBreakthroughHitRatio, 0.0F, 1.0F, fMinConfUpdate, fMaxConfUpdate);
    }
    fHitConfidence += fHitConfidenceUpdate;
    fHitConfidence = TUE_CML_MinMax(0.0F, 100.0F, fHitConfidence);
    // fHitConfidence = TUE_CML_MinMax(0.0F, 100.0F, 120);

    pInfoLevel[uWarnLevel].uBreakthroughHitConfi[uLine] = (uint8)fHitConfidence;
  }
}

/*****************************************************************************
  Functionname: LBSOSECalculateHitLimit */ /*!

  @brief: Limit the expected breakthrough between the upper and the lower

  @description: Limit the expected breakthrough between the upper and the lower

  @param[in]  fBreakthroughLimit          pfYBreakthrough[uLine] + pfYBreakthroughStd[uLine]
              fYMaxBreakthrough           y-axis max edge of breakthrough
			  fYMinBreakthrough           y-axis min edge of breakthrough
			  bBreakthroughHit            Flag whether the object hits breakthrough in the current cycle
  @param[out] void

  @return     fRet                        Limited value of breakthrough
*****************************************************************************/
float32 LBSOSECalculateHitLimit(float32 fBreakthroughLimit,
                                float32 fYMaxBreakthrough,
                                float32 fYMinBreakthrough,
                                boolean bBreakthroughHit) {
  float32 fBreakthroughHyst = 0.F;
  float32 fRet = 0.F;

  // Add hysteresis of breakthrough
  if (bBreakthroughHit) {
    fBreakthroughHyst = OSE_BREAKTHROUGH_HYSTERESIS;  // 0.3
  }

  if (fBreakthroughLimit > (fYMaxBreakthrough + fBreakthroughHyst)) {
    // The limit of the expected breakthrough is above the upper breakthrough
    // limit
    fRet = fYMaxBreakthrough;
  } else if (fBreakthroughLimit < (fYMinBreakthrough - fBreakthroughHyst)) {
    // The limit of the expected breakthrough is below the lower breakthrough
    // limit
    fRet = fYMinBreakthrough;
  } else {
    // The limit of the expected breakthrough is within the defined
    // breakthrough
    fRet = fBreakthroughLimit;
  }
  return fRet;
}

/*****************************************************************************
  Functionname: LBSOSEUpdateBTHitHysteresisTimer */ /*!

  @brief: Update the warning hysteresis timer

  @description: Update the warning hysteresis timer to prevent unstable per object warnings

  @param[in]  fCycletime          Current task cycle time from EMGlobalOutput
              fTTC0_s             TTC of the object to crossing line 0
			  fTTCFiltered0_s     Filtered TTC of the object to crossing line 0
			  pInfoLevel          DOW warning information structure
  @param[out] pInfoLevel[uWarnLevel].fBTHitHystTimer      the warning hysteresis timer

  @return
*****************************************************************************/
void LBSOSEUpdateBTHitHysteresisTimer(float32 fCycletime, float32 fTTC0_s,
                                      float32 fTTCFiltered0_s,
                                      OSEObjInfoLevel_t* pInfoLevel) {
  for (uint8 uWarnLevel = 0U; uWarnLevel < OSE_NUM_OF_WARN_LEVELS;
       uWarnLevel++) {
    // When a warning starts or when the breakthrough was hit during a warning
    if (pInfoLevel[uWarnLevel].bWarning &&
        (pInfoLevel[uWarnLevel].bBreakthroughHit[0U] ||
         !pInfoLevel[uWarnLevel].bWarningLastCycle)) {
      // Set the dead reckon timer to the calculated TTC
      pInfoLevel[uWarnLevel].fBTHitHystTimer = MIN(fTTC0_s, fTTCFiltered0_s);
    } else {
      // Run the timer
      pInfoLevel[uWarnLevel].fBTHitHystTimer -= fCycletime;
    }
    // Limit it to 1s
    pInfoLevel[uWarnLevel].fBTHitHystTimer =
        TUE_CML_MinMax(0.0F, 1.0F, pInfoLevel[uWarnLevel].fBTHitHystTimer);
  }
}

/*****************************************************************************
  Functionname: LBSOSECheckBTHitHysteresis */ /*!

  @brief: Check warning against hysteresis timer

  @description: Check warning against hysteresis timer

  @param[in]  pInfoLevel                                    DOW warning information structure
  @param[out] pInfoLevel[uWarnLevel].bBTHitHystActive       Flag whether the warning hysteresis

  @return
*****************************************************************************/
void LBSOSECheckBTHitHysteresis(OSEObjInfoLevel_t* pInfoLevel) {
  boolean bBTHitHystActive = FALSE;

  for (uint8 uWarnLevel = 0U; uWarnLevel < OSE_NUM_OF_WARN_LEVELS;
       uWarnLevel++) {
    bBTHitHystActive = FALSE;
    // As long as the time is not elapsed set hysteresis to active
    if (pInfoLevel[uWarnLevel].fBTHitHystTimer > TUE_C_F32_DELTA) {
      bBTHitHystActive = TRUE;
    }
    pInfoLevel[uWarnLevel].bBTHitHystActive = bBTHitHystActive;
  }
}

/*****************************************************************************
  Functionname: LBSOSECheckTTC                                          */ /*!

  @brief: Check the object's TTC

  @description: Check the object's TTC for possible OSE warning start

  @param[in]  uLine                                               The number of crossing line
              fTTC_s                                              TTC of the object                              
			  fTTCFiltered_s                                      Filtered TTC of the object
			  bObjectAtEdgeFoV                                    Flag whether the object is at the edge of FOV
			  pfTTCThreshold_s                                    TTC thresholds of three levels; Value: {2.5F, 1.5F, 1.2F} 
			  pInfoLevel[uWarnLevel].bTTCBelowThresh[uLine]       Flag whether the object's TTC is below TTC Threshold in the last cycle
  @param[out] pInfoLevel[uWarnLevel].bTTCBelowThresh[uLine]       Flag whether the object's TTC is below TTC Threshold in the current cycle

  @return
*****************************************************************************/
void LBSOSECheckTTC(const uint8 uLine, float32 fTTC_s, float32 fTTCFiltered_s,
                    boolean bObjectAtEdgeFoV, const float32* pfTTCThreshold_s,
                    OSEObjInfoLevel_t* pInfoLevel) {
  float32 fTTCThreshold = 0.F;
  boolean bTTCBelowThresh = FALSE;

  for (uint8 uWarnLevel = 0U; uWarnLevel < OSE_NUM_OF_WARN_LEVELS;
       uWarnLevel++) {
    bTTCBelowThresh = FALSE;
    fTTCThreshold = pfTTCThreshold_s[uWarnLevel];
    // The TTC was already below the threshold. Use hysteresis
    if (pInfoLevel[uWarnLevel].bTTCBelowThresh[uLine]) {
      if (fTTC_s < (2.0F * fTTCThreshold) ||
          fTTCFiltered_s < (2.0 * fTTCThreshold)) {
        bTTCBelowThresh = TRUE;
      }
    } else {
      // Limit the threshold if the object is at the edge of FoV
      if (bObjectAtEdgeFoV) {
        fTTCThreshold = MIN(fTTCThreshold, OSE_TTC_EFOV_THRESH);  // 1
      }

      if (fTTC_s < fTTCThreshold || fTTCFiltered_s < fTTCThreshold) {
        bTTCBelowThresh = TRUE;
      }
    }
    pInfoLevel[uWarnLevel].bTTCBelowThresh[uLine] = bTTCBelowThresh;
  }
}

/*****************************************************************************
  Functionname: LBSOSECheckShortWarning */ /*!

  @brief: Check the object's TTC

  @description: Check the object's TTC for possible OSE warning start

  @param[in]  fTTC_s              TTC of the object to crossing line 0         
              fTTCFiltered_s	  Filtered TTC of the object to crossing line 0 
  @param[out] pbShortTTC          Flag whether object's TTC is below 0

  @return
*****************************************************************************/
void LBSOSECheckShortWarning(float32 fTTC_s, float32 fTTCFiltered_s,
                             boolean* pbShortTTC) {
  boolean bShortTTC = FALSE;

  if (fTTC_s < OSE_SHORT_WARNING_THRESH ||
      fTTCFiltered_s < OSE_SHORT_WARNING_THRESH)  // 0
  {
    bShortTTC = TRUE;
  }
  *pbShortTTC = bShortTTC;
}

/*****************************************************************************
  Functionname: LBSOSEWarningDecision */ /*!

  @brief: Decision if this object causes an OSE warning

  @description: Decision if this object should warn based on warning conditions 

  @param[in]  fRCS                                             Object's RCS
              pOSEObjGlobal                                    OSE global variables pointer
			  pInfoLevel                                       DOW warning information structure
  @param[out] pInfoLevel[uWarnLevel].bWarningLastCycle         Flag whether the OSE warning was actived in the last cycle
			  pInfoLevel[uWarnLevel].bWarning                  Flag whether the OSE warning is actived
  @return
*****************************************************************************/
#ifdef ST_PERCEPTION

void LBSOSEWarningDecision(float32 fRCS, OSEObjInfoArrayGlobal_t* pOSEObjGlobal,
                           OSEObjInfoLevel_t* pInfoLevel) {
  boolean bWarning;
  for (uint8 uWarnLevel = 0u; uWarnLevel < OSE_NUM_OF_WARN_LEVELS;
       uWarnLevel++) {
    bWarning = FALSE;
    // Check warning start conditions
    if (!pInfoLevel[uWarnLevel].bWarning) {
      if (pInfoLevel[uWarnLevel].uBreakthroughHitConfi[0u] >=
              OSE_HIT_CONFI_THRESH &&  // 90
          pInfoLevel[uWarnLevel].uBreakthroughHitConfi[1u] >=
              OSE_HIT_CONFI_THRESH &&
          pInfoLevel[uWarnLevel].bObjectInRange &&
          pInfoLevel[uWarnLevel].bTTCBelowThresh[0u] &&
          pOSEObjGlobal->bRelevant && !pOSEObjGlobal->bMirror &&
          !pOSEObjGlobal->bShortTTC && !pOSEObjGlobal->bSideTrack &&
          pOSEObjGlobal->bValidApproachAngle &&
          pOSEObjGlobal->bUpdatedRecently && pOSEObjGlobal->fQuality >= 1.0f &&
          fRCS > OSE_RCS_THRESH)  //-20
      {
        bWarning = TRUE;
      }
    }
    // Check continued warning conditions
    else {
      if (pOSEObjGlobal->bRelevant && pInfoLevel[uWarnLevel].bBTHitHystActive &&
          (pInfoLevel[uWarnLevel].bTTCBelowThresh[0u] ||
           pInfoLevel[uWarnLevel].bTTCBelowThresh[1u])) {
        bWarning = TRUE;
      }
    }
    // Store current warning info to detect positive edge
    pInfoLevel[uWarnLevel].bWarningLastCycle = pInfoLevel[uWarnLevel].bWarning;
    pInfoLevel[uWarnLevel].bWarning = bWarning;
  }
}
#endif

void LBSOSEWarningDecision(OSEObjInfoArrayGlobal_t* pOSEObjGlobal,
                           OSEObjInfoLevel_t* pInfoLevel) {
  boolean bWarning = FALSE;

  // for OSE_l1_warning / OSE_l2_warning
  for (uint8 uWarnLevel = 0U; uWarnLevel < 2; uWarnLevel++) {
    bWarning = FALSE;
    // Check warning start conditions
    if (!pInfoLevel[uWarnLevel].bWarning) {
      if (pInfoLevel[uWarnLevel].uBreakthroughHitConfi[0U] >=
              OSE_HIT_CONFI_THRESH &&  // 90
          pInfoLevel[uWarnLevel].uBreakthroughHitConfi[1U] >=
              OSE_HIT_CONFI_THRESH &&
          pInfoLevel[uWarnLevel].bObjectInRange &&
          pInfoLevel[uWarnLevel].bTTCBelowThresh[0U] &&
          pOSEObjGlobal->bRelevant && !pOSEObjGlobal->bShortTTC &&
          !pOSEObjGlobal->bSideTrack && pOSEObjGlobal->bValidApproachAngle &&
          pOSEObjGlobal->bUpdatedRecently && pOSEObjGlobal->fQuality >= 1.0F) {
        bWarning = TRUE;
      }
    }
    // Check continued warning conditions
    else {
      if (pOSEObjGlobal->bRelevant && pInfoLevel[uWarnLevel].bBTHitHystActive &&
          (pInfoLevel[uWarnLevel].bTTCBelowThresh[0U] ||
           pInfoLevel[uWarnLevel].bTTCBelowThresh[1U])) {
        bWarning = TRUE;
      }
    }
    // Store current warning info to detect positive edge
    pInfoLevel[uWarnLevel].bWarningLastCycle = pInfoLevel[uWarnLevel].bWarning;
    pInfoLevel[uWarnLevel].bWarning = bWarning;
  }

  // // for OSE_l3_warning

  // boolean bWarning_l3 = FALSE;

  // if (pInfoLevel[0].bWarning || pInfoLevel[1].bWarning) {  // L2_WARNING is
  // true
  //   if ((!bRightSensor && bEgoLeftDoorOpen) ||
  //       (bRightSensor && bEgoRightDoorOpen)) {
  //     bWarning_l3 = TRUE;
  //   }
  // }
  // // Store current l3 warning info to detect positive edge
  // pInfoLevel[2].bWarningLastCycle = pInfoLevel[2].bWarning;
  // pInfoLevel[2].bWarning = bWarning_l3;
}

/*****************************************************************************
  Functionname: LBSOSESetGlobalWarning */ /*!

  @brief: Global warning decision    

  @description: Global warning decision    

  @param[in]  uObj                 The serial number of number   
              fDistX_met           Object's longitudinal relative distance
			  fLengthRear          Object's length behind the track position
			  fTTCFiltered_s       Filtered TTC of the object
			  pOSEObjGlobal        OSE global variables pointer
  @param[out] pMultiObj            Critical object's information structure pointer
              pbOSEWarnActive      Flag whether OSE warning is activated

  @return
*****************************************************************************/
void LBSOSESetGlobalWarning(uint8 uObj, float32 fDistX_met, float32 fLengthRear,
                            float32 fTTCFiltered_s,
                            OSEObjInfoArrayGlobal_t* pOSEObjGlobal,
                            OSE_MultiObjGlobal_t* pMultiObj,
                            boolean* pbOSEWarnActive) {
  float32 fTTC_MIN = TUE_CML_Min(pOSEObjGlobal->fTTC_s[0], fTTCFiltered_s);
  for (uint8 uWarnLevel = 0U; uWarnLevel < OSE_NUM_OF_WARN_LEVELS;
       uWarnLevel++) {
    // The current object is warning
    if (pOSEObjGlobal->InfoLevel[uWarnLevel].bWarning) {
      // Set the global warning
      pbOSEWarnActive[uWarnLevel] = TRUE;
      if (fTTC_MIN < pMultiObj->fCriticalTTC) {
        pMultiObj->fCriticalTTC = fTTC_MIN;
        pMultiObj->uCriticalObjID = uObj;
        // Save rear edge position
        pMultiObj->fCriticalObjDistX = -fDistX_met + fLengthRear;
      }
      // Set additional info according to the warning state
      // switch ((OSEWarningLevel_t)uWarnLevel) {
      //   case OSE_WARN_LEVEL_ONE:
      //     // If the TTC of the current object is below another warning object
      //     if (fTTCFiltered_s < pMultiObj->fCriticalTTC) {
      //       pMultiObj->fCriticalTTC = fTTCFiltered_s;
      //       pMultiObj->uCriticalObjID = uObj;
      //       // Save rear edge position
      //       pMultiObj->fCriticalObjDistX = -fDistX_met + fLengthRear;
      //     }
      //     break;
      //   case OSE_WARN_LEVEL_TWO:
      //     break;
      //   case OSE_WARN_LEVEL_THREE:
      //     break;
      //   default:
      //     break;
      // }
    }
  }
}

/*****************************************************************************
  Functionname: LBSOSECheckMultiObjectInterrupt */ /*!

  @brief: Handle the warning interruption for multiple following targets

  @description: Handle the warning interruption for multiple following targets

  @param[in]   pEMObjArray          EM object array information
  @param[out]  pMultiObj            Critical object's information

  @return
*****************************************************************************/
void LBSOSECheckMultiObjectInterrupt(const OSEGenObjArrayInReq* pEMObjArray,
                                     OSE_MultiObjGlobal_t* pMultiObj) {
  float32 fSafetyMargin = 0.F;
  float32 fCriticalObjFrontDistX = 0.F;
  boolean bInterrupt = FALSE;

  // Check whether the critical object has changed
  if (pMultiObj->uCriticalObjID != TUE_C_UI8_VALUE_INVALID &&
      pMultiObj->uCriticalObjIDLastCycle != TUE_C_UI8_VALUE_INVALID &&
      pMultiObj->uCriticalObjID != pMultiObj->uCriticalObjIDLastCycle) {
    fCriticalObjFrontDistX =
        -pEMObjArray[pMultiObj->uCriticalObjID]->fDistX_met -
        pEMObjArray[pMultiObj->uCriticalObjID]->fLengthFront_met;
    // Calculate the safety margin between the critical object of the last
    // cycle and the current cycle
    fSafetyMargin =
        (fCriticalObjFrontDistX - pMultiObj->fCriticalObjDistXLastCycle) /
        SafeDiv(-pEMObjArray[pMultiObj->uCriticalObjID]->fVrelX_mps);

    if (fSafetyMargin > OSE_MULTIOBJ_SAFE_MARGIN)  // 0.7
    {
      pMultiObj->uInterruptCycleCount = OSE_MULTIOBJ_INTERRUPT_CYCLES;  // 1
    }
  }

  // Check whether the interruption counter is still not 0
  if (pMultiObj->uInterruptCycleCount > 0U) {
    bInterrupt = TRUE;
    pMultiObj->uInterruptCycleCount--;
  }
  pMultiObj->bWarningInterrupt = bInterrupt;
}
