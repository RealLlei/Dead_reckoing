/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "lbs_calculation.h"

/*****************************************************************************
  VARIABLES
*****************************************************************************/

/*****************************************************************************
  MAIN FUNCTION
*****************************************************************************/
LBSLCAObjInfo_t* pGetLCAObjInfoPointer(uint8 uObj) {
  //  Check id number
  return &LBSCalculate.LCAObjInfoList[uObj];
}

LBSObjInfo_st* pGetLBSObjInfoPointer(uint8 uObj) {
  //  Check id number
  return &LBSCalculate.LBSObjInfoList[uObj];
}

const LBS_GenObject_st* pGetGenObjListPointer_Object(
    uint8 uObj, const EMGenObjList_st* pGenObjList) {
  //  check obj number range
  return &pGenObjList->aObject[uObj];
}

const LBS_SRRObject_st* pGetSRRObjListPointer_Object(
    uint8 uObj, const EMSRRObjList_st* pSRRObjList) {
  return &pSRRObjList->aObject[uObj];
}

boolean bGetGenObjIsDeleted(uint8 uObj, const EMGenObjList_st* pGenObjList) {
  //  obj number check
  boolean bRet = FALSE;
  if (pGenObjList->aObject[uObj].General.uiMaintenanceState_nu ==
      EM_GEN_OBJ_MT_STATE_DELETED) {
    bRet = TRUE;
  } else {
    bRet = FALSE;
  }
  return bRet;
}

LBS_Globals_t* pGeLBSCalculatePointer_LBSGlobals() {
  return &LBSCalculate.LBS_Globals;
}

LBSLCACalculate_t* pGeLBSCalculatePointer_LCAGlobals() {
  return &LBSCalculate.LBSLCACalc;
}

LBSCalculate_st* pGetLBSCalculatePointer() {
  return &LBSCalculate;
}

/*****************************************************************************
  MAIN FUNCTION
*****************************************************************************/

/*****************************************************************************
  Functionname: LBSFCTObjSelSetParameter                                  */ /*!

  @brief:Set object selection algo parameters

  @description:Set object selection algo parameters

  @param[in]:void

  @return:void
*****************************************************************************/
void LBSFCTObjSelSetParameter() {
  LBS_Globals_t* pLBSGlobals = pGeLBSCalculatePointer_LBSGlobals();
  pLBSGlobals->ParameterObjSel.fTargetRangeMax = LBS_OBJSEL_TARGETRANGE_MAX;
  pLBSGlobals->ParameterObjSel.fVegoMin = LBS_OBJSEL_VEGO_MIN;
  pLBSGlobals->ParameterObjSel.fVegoMax = LBS_OBJSEL_VEGO_MAX;
  pLBSGlobals->ParameterObjSel.fVTargetMin = LBS_OBJSEL_VTARGETMIN;
  pLBSGlobals->ParameterObjSel.fXMaxBreakthrough = LBS_OBJSEL_XMAX_BREAKTHROUGH;
  pLBSGlobals->ParameterObjSel.fXMinBreakThrough = LBS_OBJSEL_XMIN_BREAKTHROUGH;
}

/*****************************************************************************
  Functionname: LBSCalculateGlobalProperties                                  */ /*!

  @brief:Calculated global LBS properties

  @description:Call other functions to calculate global LBS properties

  @param[in]:reqPorts,params,proPorts,debugInfo

  @return:void
*****************************************************************************/
void LBSCalculateGlobalProperties(const LBSInReq_st* reqPorts,
                                  const LBSParam_st* params,
                                  LBSOutPro_t* proPorts,
                                  LBSDebug_t* debugInfo) {
  const EMRoad_t* pRoad = &reqPorts->Road;
  const EgoVehicleInfo_t* pEgoInfo = &reqPorts->EgoVehInfo;
  const EMGenObjList_st* pGenObjList = &reqPorts->GenObjList;
  //const EMSRRObjList_st* pSRRObjList = &reqPorts->SRRObjList;
  //const LBSSystemParam_t* pLBSSystemPar = &reqPorts->LBSSystemParam;

  LBSCalculateCheckInnerSensor(pRoad);
  LBSCalculateMaxSpeedOverGround(pGenObjList, pEgoInfo, pRoad);
  LBSCalculateSensorOffset(params);
}

/*****************************************************************************
  Functionname: LBSCalculateSensorOffset                                  */ /*!

  @brief:Calculate sensor offset by sensor mounting parameter

  @description:Calculate sensor offset by sensor mounting parameter

  @param[in]:params

  @return:void
*****************************************************************************/
void LBSCalculateSensorOffset(const LBSParam_st* params) {
  //  consider to delete this function because radar input has been
  // translate to AUTOSAR coordination

  LBS_Globals_t* pLBSGlobals = pGeLBSCalculatePointer_LBSGlobals();
  const float32 fVehWidth =
      params->LBS_Ks_VehParameter_nu.LBS_Kf_VehicleWidth_met;
  float32 fSensorLatPos = 0.F;
  boolean bSensorRight = FALSE;

  if (bSensorRight == FALSE) {
    fSensorLatPos =
        params->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu.LBS_Kf_LatPos_met;
    pLBSGlobals->fSensorOffetToSide_met = 0.5F * fVehWidth - fSensorLatPos;
    pLBSGlobals->fSensorOffsetToRear_met =
        params->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu
            .LBS_Kf_LongPos_met;
  } else {
    fSensorLatPos = params->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu
                        .LBS_Kf_LatPos_met;
    pLBSGlobals->fSensorOffetToSide_met = -0.5F * fVehWidth - fSensorLatPos;
    pLBSGlobals->fSensorOffsetToRear_met =
        params->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu
            .LBS_Kf_LongPos_met;
  }
  /*pLBSGlobals->fLeftSensorOffetToSide_met = 0.5F * fVehWidth -
  params->SensorMounting.SensorLeft.fLatPos_met;
  pLBSGlobals->fRightSensorOffetToSide_met = -0.5F * fVehWidth -
  params->SensorMounting.SensorRight.fLatPos_met;*/
}

/*****************************************************************************
  Functionname: LBSCalculateMaxSpeedOverGround */ /*!

  @brief:Calculate the maximum speed over ground of all present objects including
         the speed of the subject vehicle

  @description:Calculate the maximum speed over ground of all present objects including
		       the speed of the subject vehicle

  @param[in]:pGenObjList,pEgoInfo,pRoad

  @return:void
*****************************************************************************/
void LBSCalculateMaxSpeedOverGround(const EMGenObjList_st* pGenObjList,
                                    const EgoVehicleInfo_t* pEgoInfo,
                                    const EMRoad_t* pRoad) {
  const LBSLCAObjInfo_t* pLCAObjInfo = NULL;
  const LBS_GenObject_st* pGenObjInfo = NULL;
  const LBSLCACalculate_t* pLCAGlobals = pGeLBSCalculatePointer_LCAGlobals();
  LBS_Globals_t* pLBSGlobals = pGeLBSCalculatePointer_LBSGlobals();

  const float32 fEgoSpeedX = pEgoInfo->fegoVelocity_mps;
  float32 fMaxSpeedOverGround = fEgoSpeedX;
  float32 fFilterConst = LBS_MAXSPD_OVERGND_SLOWFILTER;
  uint8 uObjectIndex = 0;

  for (uObjectIndex = 0U; uObjectIndex < LBS_INPUT_OBJECT_NUMBER;
       uObjectIndex++) {
    pGenObjInfo = pGetGenObjListPointer_Object(uObjectIndex, pGenObjList);
    pLCAObjInfo = pGetLCAObjInfoPointer(uObjectIndex);
    if ((!bGetGenObjIsDeleted(uObjectIndex, pGenObjList)) &&
        (pGenObjInfo->General.uiLifeCycles_nu >
         LBS_MAXSPD_OVERGND_LIFETIME_THRESH) &&
        (pGenObjInfo->Kinemactic.fVrelX_mps > 0.0F) &&
        (pLCAObjInfo->bLCAMirrorFrontObject == FALSE)) {
      const float32 fVxObjOverGround =
          pGenObjInfo->Kinemactic.fVrelX_mps + fEgoSpeedX;
      if (fVxObjOverGround > fMaxSpeedOverGround) {
        fMaxSpeedOverGround = fVxObjOverGround;
      }
    }
  }

  // If the calculated maximum speed over ground is larger than the previous
  // stored value use a fast filtering in case of present front mirror objects
  // the filter rate shall not be increased as it leads to wrong values
  if ((fMaxSpeedOverGround > pLBSGlobals->fMaxSpeedOverGround_mps) &&
      (pLCAGlobals->fFMObjRate < LBS_MAXSPD_OVERGND_FASTILTER_MIN_FMRTE)) {
    fFilterConst = LBS_MAXSPD_OVERGND_FASTFILTER;
  } else {
    fFilterConst = LBS_MAXSPD_OVERGND_SLOWFILTER;
  }

  GDB_Math_LowPassFilter(&pLBSGlobals->fMaxSpeedOverGround_mps,
                         fMaxSpeedOverGround, fFilterConst);

  // Check if the road type changed compared to last cycle and the confidence is
  // sufficient Also set it to the current estimated maximum value if the
  // filtered value is smaller than the ego speed
  if ((pLBSGlobals->fMaxSpeedOverGround_mps < fEgoSpeedX) ||
      ((pLBSGlobals->LastCycleStates.uRoadType != pRoad->RoadType.uiRoadType) &&
       (pRoad->RoadType.fRoadTypeConf >
        LBS_MAXSPD_OVERGND_ROADTYPE_CONF_MIN))) {
    // Reset filtered max speed over ground
    pLBSGlobals->fMaxSpeedOverGround_mps = fMaxSpeedOverGround;
  }
  pLBSGlobals->LastCycleStates.uRoadType = pRoad->RoadType.uiRoadType;
}

/*****************************************************************************
  Functionname: LBSCalculateCheckInnerSensor                                  */ /*!

  @brief:Check if current sensor is inner or outer sensor

  @description:Check if current sensor is inner or outer sensor using the curve radius

  @param[in]:pRoad

  @return:void
*****************************************************************************/
void LBSCalculateCheckInnerSensor(const EMRoad_t* pRoad) {
  LBS_Globals_t* pLBSGlobals = pGeLBSCalculatePointer_LBSGlobals();
  float32 fCurveRadiusSteering = pRoad->fCurveRadius_met;
  float32 fCurveRadiusDriven = pRoad->fDrivenCurveRadius_met;

  pLBSGlobals->bInnerSensorDriven = FALSE;
  pLBSGlobals->bInnerSensorSteering = FALSE;

  // only calculate left side sensor ,the inner direction is positive turn to
  // right for right side to reverse inner flag
  if (fCurveRadiusSteering > 0.0F) {
    pLBSGlobals->bInnerSensorSteering = TRUE;
  }
  if (fCurveRadiusDriven > 0.0F) {
    pLBSGlobals->bInnerSensorDriven = TRUE;
  }
}

/*****************************************************************************
  Functionname: LBSCalculateObjectProperties                                  */ /*!

  @brief:Calculates object properties used in LBS

  @description:Calls other functions to calculate object properties used in LBS

  @param[in]:reqPorts,params, proPorts, debugInfo)

  @return:void
*****************************************************************************/
void LBSCalculateObjectProperties(const LBSInReq_st* reqPorts,
                                  const LBSParam_st* params,
                                  LBSOutPro_t* proPorts,
                                  LBSDebug_t* debugInfo) {
  const LBS_GenObject_st* pGenObjInfo = NULL;
  const LBS_SRRObject_st* pSRRObjInfo = NULL;

  const EMRoad_t* pRoad = &reqPorts->Road;
  const EgoVehicleInfo_t* pEgoInfo = &reqPorts->EgoVehInfo;
  const EMGenObjList_st* pGenObjList = &reqPorts->GenObjList;
  const EMSRRObjList_st* pSRRObjList = &reqPorts->SRRObjList;
  const LBSSystemParam_t* pLBSSystemPar = &reqPorts->LBSSystemParam;
  uint8 uObjectIndex = 0;

  for (uObjectIndex = 0U; uObjectIndex < LBS_INPUT_OBJECT_NUMBER;
       uObjectIndex++) {
    pGenObjInfo = pGetGenObjListPointer_Object(uObjectIndex, pGenObjList);
    pSRRObjInfo = pGetSRRObjListPointer_Object(uObjectIndex, pSRRObjList);

    /* calculate first detected distance */
    LBSCalculateFirstDetectDist(uObjectIndex, pGenObjInfo, pSRRObjInfo);

    if (!bGetGenObjIsDeleted(uObjectIndex, pGenObjList)) {
      if (pGenObjList->aObject[uObjectIndex].General.uiMaintenanceState_nu !=
          EM_GEN_OBJ_MT_STATE_DELETED) {
        switch (pSRRObjInfo->Qualifiers.uiMeasuredTargetFrequency_nu) {
          case 255:
            LBSCalculate.LBSObjInfoList[uObjectIndex].fUpdateRate_nu = 0.999F;
            break;
          case 128:
            LBSCalculate.LBSObjInfoList[uObjectIndex].fUpdateRate_nu = 0.96F;
            break;
          case 64:
            LBSCalculate.LBSObjInfoList[uObjectIndex].fUpdateRate_nu = 0.91F;
            break;

          default:
            LBSCalculate.LBSObjInfoList[uObjectIndex].fUpdateRate_nu = 0.74999F;
            break;
        }
      }
      /* increase the cycletime_sum of the object */
      LBSCalculateCycleTimeSum(uObjectIndex, pLBSSystemPar);
      /* Calculate xmin, xmax, ymin, ymax of the current object */
      LBSCalculateObjectBorder(uObjectIndex, pGenObjInfo);
      /* calculate the borders in which an object has moved during its lifetime
       */
      LBSCalculateObjectMovementBorders(uObjectIndex, pGenObjInfo);
      /* calculate object quality related qualifies */
      LBSCalculateObjectQualifiers(uObjectIndex, pGenObjInfo, pSRRObjInfo);
      /* calculate the TTC of the current object */
      LBSCalculateTTC(uObjectIndex, pGenObjInfo, pSRRObjInfo, params);
      /* calculate the radian coordinates of the current object */
      LBSCalculateRadianCoords(uObjectIndex, pGenObjInfo, params);
      /* calculate the position based vx and vy of the current object */
      LBSCalculatePosBasedVxVy(uObjectIndex, pGenObjInfo, pLBSSystemPar);
      /* calculate the maximum dimensions (length and width) of the current
       * object */
      LBSCalculateMaxDimensions(uObjectIndex, pGenObjInfo);
      /* calculate the absolute object velocity */
      LBSCalculateAbsoluteObjectVelocity(uObjectIndex, pGenObjInfo, pEgoInfo,
                                         params);
      /* calculate the absolute object range */
      LBSCalculateRadialObejctRange(uObjectIndex, pGenObjInfo);
      /* calculate first stabel dynamic property */
      LBSCalculateFirstDynProp(uObjectIndex, pGenObjInfo, pSRRObjInfo);
      /*Calculate object distance to ego course*/
      LBSCalculateDist2Course(uObjectIndex, pGenObjInfo, pSRRObjInfo, pRoad);
    }
  }
}

/*****************************************************************************
  Functionname: LBSCalculateCycleTimeSum                                  */ /*!

  @brief:Calculates the cycle time sum

  @description:Calculate the duration the object was alive(in seconds)

  @param[in]:uObjectIndex,pLBSSystemPar

  @return:void
*****************************************************************************/
void LBSCalculateCycleTimeSum(uint8 uObjectIndex,
                              const LBSSystemParam_t* pLBSSystemPar) {
  LBSObjInfo_st* pLBSObjInfo = pGetLBSObjInfoPointer(uObjectIndex);
  float32 fCycleTime = pLBSSystemPar->fCycletime_s;

  // Increase the duration th object war alive(in seconds)
  pLBSObjInfo->fCycletimeSum_s = pLBSObjInfo->fCycletimeSum_s + fCycleTime;
}

/*****************************************************************************
  Functionname: LBSCalculateObjectBorder                                  */ /*!

  @brief:Calculates the borders of an object

  @description:Calculates the borders of an object

  @param[in]:uObjectIndex,pGenObj

  @return:void
*****************************************************************************/
void LBSCalculateObjectBorder(uint8 uObjectIndex,
                              const LBS_GenObject_st* pGenObj) {
  LBSObjInfo_st* pLBSObjInfo = pGetLBSObjInfoPointer(uObjectIndex);

  pLBSObjInfo->ObjBorders.fXmin_met =
      pGenObj->Kinemactic.fDistX_met - pGenObj->Geometry.fLengthRear_met;
  pLBSObjInfo->ObjBorders.fXmax_met =
      pGenObj->Kinemactic.fDistX_met + pGenObj->Geometry.fLengthFront_met;

  pLBSObjInfo->ObjBorders.fYmin_met =
      pGenObj->Kinemactic.fDistY_met - pGenObj->Geometry.fWidthRight_met;
  pLBSObjInfo->ObjBorders.fYmax_met =
      pGenObj->Kinemactic.fDistY_met + pGenObj->Geometry.fWidthLeft_met;
}

/*****************************************************************************
  Functionname: LBSCalculateObjectMovementBorders */ /*!

  @brief:Calculates the x and y borders in which an object has moved during its lifetime

  @description:Calculates the x and y borders in which an object has moved during its lifetime

  @param[in]:uObjectIndex,pGenObj

  @return:void
*****************************************************************************/
void LBSCalculateObjectMovementBorders(uint8 uObjectIndex,
                                       const LBS_GenObject_st* pGenObj) {
  LBSObjInfo_st* pLBSObjInfo = pGetLBSObjInfoPointer(uObjectIndex);

  const float32 fObjDistX = pGenObj->Kinemactic.fDistX_met;
  const float32 fObjDistY = pGenObj->Kinemactic.fDistY_met;

  if (pLBSObjInfo->ObjMovementBorders.fXmin_met >
      TUE_C_F32_VALUE_INVALID - 1.0F) {
    pLBSObjInfo->ObjMovementBorders.fXmin_met = fObjDistX;
    pLBSObjInfo->ObjMovementBorders.fXmax_met = fObjDistX;
    pLBSObjInfo->ObjMovementBorders.fYmin_met = fObjDistY;
    pLBSObjInfo->ObjMovementBorders.fYmax_met = fObjDistY;
  } else {
    // Calculate the bounds of the longitudinal motion
    if (fObjDistX > pLBSObjInfo->ObjMovementBorders.fXmax_met) {
      // the object move forward
      pLBSObjInfo->ObjMovementBorders.fXmax_met = fObjDistX;
      pLBSObjInfo->fXMovement_met = pLBSObjInfo->ObjMovementBorders.fXmax_met -
                                    pLBSObjInfo->ObjMovementBorders.fXmin_met;

    } else if (fObjDistX < pLBSObjInfo->ObjMovementBorders.fXmin_met) {
      // the object move backward
      pLBSObjInfo->ObjMovementBorders.fXmin_met = fObjDistX;
      pLBSObjInfo->fXMovement_met = pLBSObjInfo->ObjMovementBorders.fXmax_met -
                                    pLBSObjInfo->ObjMovementBorders.fXmin_met;
    }

    if (fObjDistY > pLBSObjInfo->ObjMovementBorders.fYmax_met) {
      // the object move left side
      pLBSObjInfo->ObjMovementBorders.fYmax_met = fObjDistY;
      pLBSObjInfo->fYMovement_met = pLBSObjInfo->ObjMovementBorders.fYmax_met -
                                    pLBSObjInfo->ObjMovementBorders.fYmin_met;
    } else if (fObjDistY < pLBSObjInfo->ObjMovementBorders.fYmin_met) {
      // the object move right side
      pLBSObjInfo->ObjMovementBorders.fYmin_met = fObjDistY;
      pLBSObjInfo->fYMovement_met = pLBSObjInfo->ObjMovementBorders.fYmax_met -
                                    pLBSObjInfo->ObjMovementBorders.fYmin_met;
    }
  }
}

/*****************************************************************************
  Functionname: LBSCalculateObjectQualifiers                                  */ /*!

  @brief:Calculates object quality related qualifiers

  @description:Calculates object quality related qualifiers,update rate,association probability filtered

  @param[in]:uObjectIndex,pGenObj, pSRRObj

  @return:void
*****************************************************************************/
void LBSCalculateObjectQualifiers(uint8 uObjectIndex,
                                  const LBS_GenObject_st* pGenObj,
                                  const LBS_SRRObject_st* pSRRObj) {
  float32 fObjMeadured = 0.F;
  float32 fFilterConst = 0.F;
  const uint8 uHighestAssocProb = pSRRObj->Qualifiers.uiHighestAssocProb_per;
  const float32 fHighestAssocProb = ((float32)uHighestAssocProb * 0.01F);
  const uint8 uiObjMeasuredState = pGenObj->General.uiMaintenanceState_nu;
  LBSObjInfo_st* pLBSObjInfo = pGetLBSObjInfoPointer(uObjectIndex);

  /**********************************************************************
   *Filter update rate
   **********************************************************************/
  if (uiObjMeasuredState == EM_GEN_OBJ_MT_STATE_MEASURED) {
    fObjMeadured = 1.0F;
    fFilterConst = LBS_UPDATERATE_FILTER_UP;
  } else {
    fObjMeadured = 0.0F;
    fFilterConst = LBS_UPDATERATE_FILTER_DOWN;
  }

  GDB_Math_LowPassFilter(&pLBSObjInfo->fUpdateRate_nu, fObjMeadured,
                         fFilterConst);

  /**********************************************************************
   *Filter highest association probability
   **********************************************************************/
  // Now not to use uiHighestAssocProb_per because miss the AssocProb data
  if (pLBSObjInfo->fAssocProbFiltered < fHighestAssocProb) {
    fFilterConst = LBS_ASSOCPROB_FILTER_UP;
  } else {
    fFilterConst = LBS_ASSOCPROB_FILTER_DOWN;
  }
  GDB_Math_LowPassFilter(&pLBSObjInfo->fAssocProbFiltered, fHighestAssocProb,
                         fFilterConst);
}

/*****************************************************************************
  Functionname: LBSCalculateTTC                                     */ /*!

  @brief:Calculated the TTC of an object

  @description:Calculates the time to collision(TTC) of an object
               The following is assumed: 
			   a)the object and the host keep their current longitudinal acceleration
			   b)the host will hit the vehicle(the object is inside the driving corridor
			   or will enter the driving corridor)

  @param[in]:uObjectIndex, pGenObj, pSRRObj,params

  @return:void
*****************************************************************************/
void LBSCalculateTTC(uint8 uObjectIndex, const LBS_GenObject_st* pGenObj,
                     const LBS_SRRObject_st* pSRRObj,
                     const LBSParam_st* params) {
  LBSObjInfo_st* pLBSObjInfo = pGetLBSObjInfoPointer(uObjectIndex);
  const LBS_SensorMounting_t* pSensorMounting =
      &params->LBS_Ks_SensorMounting_nu
           .LBS_Kf_SensorLeft_nu;  // Temp to use left side sensor

  const float32 fObjLengthFront = pGenObj->Geometry.fLengthFront_met;
  const uint8 uiDynamicProperty = pGenObj->Attributes.eDynamicProperty_nu;
  float32 fTTCTmp1 = LBS_TTC_INVALID;
  float32 fTTCTmp2 = LBS_TTC_INVALID;
  float32 fTTCTmp3 = LBS_TTC_INVALID;

  boolean bUseAcceleration = FALSE;
  float32 fVrelRadSqr = 0.F;
  float32 fDistRadSqr = 0.F;

  boolean bIsMovingObj = FALSE;
  boolean bIsRelevantObj = FALSE;

  // Relative movement = object movement - ego movement if tracked in absolute
  // values
  float32 fX = pGenObj->Kinemactic.fDistX_met + fObjLengthFront +
               params->LBS_Ks_VehParameter_nu.LBS_Kf_VehicleLength_met -
               params->LBS_Ks_VehParameter_nu.LBS_Kf_OverhangFront_met +
               pSensorMounting->LBS_Kf_LatPos_met;
  float32 fVx = pGenObj->Kinemactic.fVrelX_mps;
  float32 fAx = pGenObj->Kinemactic.fArelX_mpss;
  // Calculate radial TTC
  if ((uiDynamicProperty == EM_GEN_OBJECT_DYN_PROPERTY_MOVING) ||
      (uiDynamicProperty == EM_GEN_OBJECT_DYN_PROPERTY_ONCOMING)) {
    bIsMovingObj = TRUE;
  }

  if (((pGenObj->Kinemactic.fDistX_met < 0.0F) &&
       (pGenObj->Kinemactic.fVrelX_mps > 0.0F)) ||
      ((pGenObj->Kinemactic.fDistX_met > 0.0F) &&
       (pGenObj->Kinemactic.fVrelX_mps < 0.0F))) {
    bIsRelevantObj = TRUE;
  }
  // Only use acceleration for objects that are well update
  if (pLBSObjInfo->fUpdateRate_nu > LBS_MIN_UPDATERATE_TTC_ACCEL) {  // 0.8
    bUseAcceleration = TRUE;
  }

  // Calculation depends on object movement
  if (pGenObj->General.uiMaintenanceState_nu == 0) {
    fTTCTmp1 = LBS_TTC_INVALID;
    fTTCTmp2 = LBS_TTC_INVALID;
  } else if ((uiDynamicProperty == EM_GEN_OBJECT_DYN_PROPERTY_MOVING ||
              (uiDynamicProperty == EM_GEN_OBJECT_DYN_PROPERTY_ONCOMING)) &&
             bIsRelevantObj) {
    // Moving in same direction
    fTTCTmp1 = LBSCalculateTTCRaw(fX, fVx, fAx, bUseAcceleration);
    fTTCTmp2 = LBSCalculateTTCRaw(fX, fVx, fAx, FALSE);
  } else {
    // Object is oncoming or stationary
    fTTCTmp1 = LBS_TTC_INVALID;
    fTTCTmp2 = LBS_TTC_INVALID;
  }

  if (bIsMovingObj && bIsRelevantObj) {
    fVrelRadSqr =
        (pGenObj->Kinemactic.fVrelX_mps) * (pGenObj->Kinemactic.fVrelX_mps) +
        (pGenObj->Kinemactic.fVrelY_mps) * (pGenObj->Kinemactic.fVrelY_mps);
    fDistRadSqr =
        (pGenObj->Kinemactic.fDistX_met) * (pGenObj->Kinemactic.fDistX_met) +
        (pGenObj->Kinemactic.fDistY_met) * (pGenObj->Kinemactic.fDistY_met);

    if (fVrelRadSqr > TUE_C_F32_DELTA) {
      fTTCTmp3 = sqrt(fDistRadSqr / fVrelRadSqr);
    } else {
      fTTCTmp3 = LBS_TTC_INVALID;
    }
  } else {
    fTTCTmp3 = LBS_TTC_INVALID;
  }

  // Store result in LBS object properties
  pLBSObjInfo->fTTCAccel_mps2 = fTTCTmp1;
  pLBSObjInfo->fTTC_s = fTTCTmp2;
  pLBSObjInfo->fTTCRadial_s = fTTCTmp3;

  GDB_Math_LowPassFilter(&pLBSObjInfo->fTTCFiltered_s, fTTCTmp2,
                         LBS_LPF_TTCFILTERED_ALPHA);
}

/*****************************************************************************
  Functionname: LBSCalculateTTCRaw                                     */ /*!

  @brief:Calculate the raw TTC

  @description:Calculate the time to collision(TTC),function does not check if
               Object/or host comes to stop before TTC

  @param[in]:fX,fVx,fAx,bUseAcceleration

  @return:fTTCTmp,the time to collision
*****************************************************************************/
float32 LBSCalculateTTCRaw(const float32 fX, const float32 fVx,
                           const float32 fAx, boolean bUseAcceleration) {
  float32 fTTCTmp = LBS_TTC_INVALID;
  float32 fRadical = 0.F;
  boolean bAxCond = FALSE;

  // The following equation is used
  // Distance = 0.5 * acceleration * t^2 + velocity * t

  // if acceleration is zero or the object is poorly updated, use simplified
  // calculation(to avoid a division by zero)
  if ((fABS(fAx) < TUE_C_F32_DELTA) || (bUseAcceleration == FALSE)) {
    if (fABS(fVx) > TUE_C_F32_DELTA) {
      fTTCTmp = fX / (-fVx);
    } else {
      fTTCTmp = LBS_TTC_INVALID;
    }
  } else {
    //             Calculation for a forward looking system
    //      -velocity +/- sqrt(velocity^2 - 2 * acceleration * distance)
    // TTC = -------------------------------------------------------------
    //                         - acceleration

    // radical
    fRadical = ((SQR(fVx)) / (SQR(fAx))) - (2.0F * (fX / fAx));

    if (fRadical < TUE_C_F32_DELTA)  // a:radical is negative
    {
      fTTCTmp = LBS_TTC_INVALID;
    } else {
      // check the Ax direction is relevant with sensor angle,front or rear
      if (fAx > -TUE_C_F32_DELTA) {
        bAxCond = TRUE;
      }
      if (bAxCond == TRUE) {
        fTTCTmp = -(fVx / fAx) + sqrt(fRadical);
      } else {
        fTTCTmp = -(fVx / fAx) - sqrt(fRadical);
      }
    }
  }

  // Limit result
  // collision in the past or some when in the far future -> no collision will
  // happen
  if ((fTTCTmp < TUE_C_F32_DELTA) || (fTTCTmp > LBS_TTC_INVALID)) {
    fTTCTmp = LBS_TTC_INVALID;
  }

  return fTTCTmp;
}

/*****************************************************************************
  Functionname: LBSCalculateRadianCoords                                     */ /*!

  @brief:Calculates the radian coordinates

  @description:Calculate the angle of the object based on X,Y distance

  @param[in]:uObjectIndex,pGenObj

  @return:void
*****************************************************************************/
void LBSCalculateRadianCoords(uint8 uObjectIndex,
                              const LBS_GenObject_st* pGenObj,
                              const LBSParam_st* params) {
  const LBS_SensorMounting_t* pSensor = NULL;
  LBSObjInfo_st* pLBSObjInfo = pGetLBSObjInfoPointer(uObjectIndex);
  float32 fSensorObjX = pGenObj->Kinemactic.fDistX_met;
  float32 fSensorObjY = pGenObj->Kinemactic.fDistY_met;

#ifdef ST_PERCEPTION
  if (pGenObj->bRightSensor == FALSE) {
    // translate coordinate and base left side sensor coordinate to calculate
    // Angle
    pSensor = &params->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu;
    fSensorObjX = fSensorObjX - pSensor->LBS_Kf_LatPos_met;
    fSensorObjY = fSensorObjY - pSensor->LBS_Kf_LatPos_met;
  } else {
    // translate coordinate and base right side sensor coordinate to calculate
    // Angle,
    pSensor = &params->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu;
    fSensorObjX = fSensorObjX - pSensor->LBS_Kf_LatPos_met;
    fSensorObjY = fSensorObjY - pSensor->LBS_Kf_LatPos_met;
  }
#endif

  // Calculate the angle
  pLBSObjInfo->fAngle_deg = RAD2DEG(ATAN2_(fSensorObjY, SafeDiv(fSensorObjX)));

  // Results of atan2 are in the range [-180,180],transform it to [0,360]
  if (pLBSObjInfo->fAngle_deg < 0.0F) {
    pLBSObjInfo->fAngle_deg += 360.0F;
  }
}

/*****************************************************************************
  Functionname: LBSCalculatePosBasedVxVy                                     */ /*!

  @brief:Calculate relative Vx and Vy based on position of object and cycle time

  @description:Calculate relative Vx and Vy based on position of object and cycle time

  @param[in]:uObjectIndex, pGenObj,pLBSSystemPar

  @return:void
*****************************************************************************/
void LBSCalculatePosBasedVxVy(uint8 uObjectIndex,
                              const LBS_GenObject_st* pGenObj,
                              const LBSSystemParam_t* pLBSSystemPar) {
  LBSObjInfo_st* pLBSObjInfo = pGetLBSObjInfoPointer(uObjectIndex);
  const float32 fObjDistX = pGenObj->Kinemactic.fDistX_met;
  const float32 fObjDistY = pGenObj->Kinemactic.fDistY_met;
  const float32 fTaskCycleTime = pLBSSystemPar->fCycletime_s;

  // Check if the current cycle time is valid and if a value for the position of
  // the last cycle has already been set
  if ((pLBSObjInfo->fXLastCycle_met >
       ((TUE_C_F32_VALUE_INVALID - TUE_C_F32_DELTA))) ||
      (fTaskCycleTime < TUE_C_F32_DELTA)) {
    pLBSObjInfo->fVxPosBased = pGenObj->Kinemactic.fVrelX_mps;
    pLBSObjInfo->fVyPosBased = pGenObj->Kinemactic.fVrelY_mps;
  } else {
    // Calculate a filtered VX and VY based on the change in position of the
    // object
    GDB_Math_LowPassFilter(
        &pLBSObjInfo->fVxPosBased,
        (fObjDistX - pLBSObjInfo->fXLastCycle_met) / fTaskCycleTime,
        LBS_LPF_VRELXY_ALPHA);
    GDB_Math_LowPassFilter(
        &pLBSObjInfo->fVyPosBased,
        (fObjDistY - pLBSObjInfo->fYLastCycle_met) / fTaskCycleTime,
        LBS_LPF_VRELXY_ALPHA);
  }

  // Storing of fXLastCycle and fYLastCycle is done in the PostProcessing step
}

/*****************************************************************************
  Functionname: LBSCalculateMaxDimensions                                     */ /*!

  @brief:Calculate the maximum dimensions of the given object

  @description:Calculate the maximum dimensions(length and width) of the given object

  @param[in]:uObjectIndex, pGenObj

  @return:void
*****************************************************************************/
void LBSCalculateMaxDimensions(uint8 uObjectIndex,
                               const LBS_GenObject_st* pGenObj) {
  LBSObjInfo_st* pLBSObjInfo = pGetLBSObjInfoPointer(uObjectIndex);
  const float32 fObjWidth =
      pGenObj->Geometry.fWidthLeft_met + pGenObj->Geometry.fWidthRight_met;
  const float32 fObjLength =
      pGenObj->Geometry.fLengthFront_met + pGenObj->Geometry.fLengthRear_met;

  // Calculate maximum length of an object over it is lifetime
  pLBSObjInfo->fObjLengthMax =
      TUE_CML_Max(fObjLength, pLBSObjInfo->fObjLengthMax);
  pLBSObjInfo->fObjWidthMax = TUE_CML_Max(fObjWidth, pLBSObjInfo->fObjWidthMax);
}

/*****************************************************************************
  Functionname: LBSCalculateAbsoluteObjectVelocity                      */ /*!

  @brief:Calculate absolute velocity of the object

  @description:Calculate absolute velocity of the object

  @param[in]:uObjectIndex,pGenObj,pEgoInfo,params

  @return:void
*****************************************************************************/
void LBSCalculateAbsoluteObjectVelocity(uint8 uObjectIndex,
                                        const LBS_GenObject_st* pGenObj,
                                        const EgoVehicleInfo_t* pEgoInfo,
                                        const LBSParam_st* params) {
  LBSObjInfo_st* pLBSObjInfo = pGetLBSObjInfoPointer(uObjectIndex);
  //const float32 fSensorLatPos =
  //params->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu
  //.LBS_Kf_LatPos_met;  // Temp to use left side
  //const float32 fSensorLongPos =
  // params->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu.LBS_Kf_LatPos_met;
  const float32 fEgoSpeedX = pEgoInfo->fegoVelocity_mps;
  const float32 fYawRate = pEgoInfo->fYawRate_radps;

  //const float32 fWheelBase =
  // params->LBS_Ks_VehParameter_nu.LBS_Kf_WheelBase_met;
  float32 fVxAbs = 0.F;
  float32 fVyAbs = 0.F;
  float32 fVabsObj = 0.F;

  // Compensation longitude velocity value by yawrate, V = W*R = YawRate *
  // Distance
  fVxAbs = (fEgoSpeedX + pGenObj->Kinemactic.fVrelX_mps) -
           (fYawRate * pGenObj->Kinemactic.fDistY_met);
  fVyAbs = (pGenObj->Kinemactic.fVrelY_mps) +
           (fYawRate * pGenObj->Kinemactic.fDistX_met);

  fVabsObj = sqrt(SQR(fVxAbs) + SQR(fVyAbs));

  pLBSObjInfo->fVabs_mpss = fVabsObj;
}

/*****************************************************************************
  Functionname: LBSCalculateRadialObejctRange                            */ /*!

  @brief:Calculate radial range of the object to sensor

  @description:Calculate radial range of the object to sensor

  @param[in]:uObjectIndex,pGenObj

  @return:void
*****************************************************************************/
void LBSCalculateRadialObejctRange(uint8 uObjectIndex,
                                   const LBS_GenObject_st* pGenObj) {
  LBSObjInfo_st* pLBSObjInfo = pGetLBSObjInfoPointer(uObjectIndex);
  float32 fRange = 0.F;

  // Calculate radial range
  fRange = sqrt(SQR(pGenObj->Kinemactic.fDistX_met) +
                SQR(pGenObj->Kinemactic.fDistY_met));
  pLBSObjInfo->fRangeRadial = fRange;
}

/*****************************************************************************
  Functionname: LBSCalculateFirstDynProp                            */ /*!

  @brief:Stores the dynamic property of the object at the first cycle when
         the object meets the stable object criteria

  @description:Stores the dynamic property of the object at the first cycle when
		       the object meets the stable object criteria

  @param[in]:uObjectIndex,pGenObj,pSRRObj

  @return:void
*****************************************************************************/
void LBSCalculateFirstDynProp(uint8 uObjectIndex,
                              const LBS_GenObject_st* pGenObj,
                              const LBS_SRRObject_st* pSRRObj) {
  LBSObjInfo_st* pLBSObjInfo = pGetLBSObjInfoPointer(uObjectIndex);
  const float32 fObjPOE = pSRRObj->Qualifiers.fProbabilityOfExistence_per;
  const uint16 uiLifeCycle = pGenObj->General.uiLifeCycles_nu;

  if ((fObjPOE > LBS_FIRSTDYNPROPERTY_POE_THRESH) &&
      (uiLifeCycle > LBS_FIRSTDYNPROPERTY_LIFETIME_THRESH) &&
      (pLBSObjInfo->firstStableDynProp == EM_GEN_OBJECT_DYN_PROPERTY_UNKNOWN)) {
    // use dynamic property until not classify to unknown property
    pLBSObjInfo->firstStableDynProp = pGenObj->Attributes.eDynamicProperty_nu;
  }
}

/*****************************************************************************
  Functionname: LBSCalculateFirstDetectDist                            */ /*!

  @brief:Calculate the object first detect distance information

  @description:Calculate the object first detect distance information by measurement state

  @param[in]:uObjectIndex,pGenObj,pSRRObj

  @return:void
*****************************************************************************/
void LBSCalculateFirstDetectDist(uint8 uObjectIndex,
                                 const LBS_GenObject_st* pGenObj,
                                 const LBS_SRRObject_st* pSRRObj) {
  LBSCalculate_st* pLBSCal = pGetLBSCalculatePointer();
  LBS_SRRObjHistory_t* pSRRObjHistory = (LBS_SRRObjHistory_t*)&pSRRObj->History;

  uint16 uiLifeCycles = pGenObj->General.uiLifeCycles_nu;
  uint8 uiObjMeasureStat = pGenObj->General.uiMaintenanceState_nu;

  // set the first detect distance of first lifeCycle

  // Sensetime
  if ((uiObjMeasureStat != EM_GEN_OBJ_MT_STATE_DELETED) &&
      (uiLifeCycles == LBS_FIRSTDETECTDIST_LIFETIME_THRESH)) {
    // if ((uiObjMeasureStat != EM_GEN_OBJ_MT_STATE_DELETED)) {
    // pSRRObjHistory->fMaxRange_met = 0.0F;//check the max range means
    pLBSCal->LBSObjHistoryList[uObjectIndex].fFirstDetectX_met =
        pGenObj->Kinemactic.fDistX_met;
    pLBSCal->LBSObjHistoryList[uObjectIndex].fFirstDetectY_met =
        pGenObj->Kinemactic.fDistY_met;
  } else if (uiObjMeasureStat == EM_GEN_OBJ_MT_STATE_DELETED) {
    // Reset first detect distance if Obj deleted this cycle
    pLBSCal->LBSObjHistoryList[uObjectIndex].fFirstDetectX_met =
        TUE_C_F32_VALUE_INVALID;
    pLBSCal->LBSObjHistoryList[uObjectIndex].fFirstDetectY_met =
        TUE_C_F32_VALUE_INVALID;
  }

  // use inner store value to update object first detect value
  pSRRObjHistory->fFirstDetectX_met =
      pLBSCal->LBSObjHistoryList[uObjectIndex].fFirstDetectX_met;
  pSRRObjHistory->fFirstDetectY_met =
      pLBSCal->LBSObjHistoryList[uObjectIndex].fFirstDetectY_met;
}

/*****************************************************************************
  Functionname: LBSCalculateDist2Course                            */ /*!

  @brief:Calculate object distance to ego trace

  @description:Calculate object distance to ego trace

  @param[in]:uObjectIndex,pGenObj,pSRRObj,pRoad

  @return:void
*****************************************************************************/
void LBSCalculateDist2Course(uint8 uObjectIndex,
                             const LBS_GenObject_st* pGenObj,
                             const LBS_SRRObject_st* pSRRObj,
                             const EMRoad_t* pRoad) {
  float32 fDist2Course = 0.F;
  LBS_SRRObjRoadRelation_t* pSRRRoad =
      (LBS_SRRObjRoadRelation_t*)&pSRRObj->RoadRelation;

  // compensation Y distance by curve radius offset
  fDist2Course = LBSCalculateDistToCurve(pGenObj->Kinemactic.fDistX_met,
                                         pRoad->fDrivenCurveRadius_met);
  fDist2Course += pGenObj->Kinemactic.fDistY_met;

  pSRRRoad->fDist2Course_met = fDist2Course;

  // temp not to use border input because consider miss camera sensor border
  // input situation
  pSRRRoad->fDist2Border_met = TUE_C_F32_VALUE_INVALID;
  pSRRRoad->bDist2BorderValid = FALSE;
}

/*****************************************************************************
  Functionname: LBSCalculateDistToDrivenCurve */ /*!

  @brief :Compute for each distance the y-position of the given half-circle radius

  @description

  @param[in]:fXpos x-distance
			 fR    Radius of circle

  @return : fRet y-distance
*****************************************************************************/
float32 LBSCalculateDistToCurve(float32 fXpos, float32 fR) {
  float32 fY1 = 0.0F;
  float32 fY2 = 0.0F;
  float32 fRet = TUE_C_F32_VALUE_INVALID;
  float32 fRadix = SQR(fR) - SQR(fXpos);

  if (fRadix > TUE_C_F32_DELTA) {
    fY1 = fR + sqrt(fRadix);
    fY2 = fR - sqrt(fRadix);
  }
  if (fABS(fY1) < fABS(fY2)) {
    fRet = fY1;
  } else {
    fRet = fY2;
  }
  return fRet;
}
