/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "lbs_rcw_calculation.h"

/*****************************************************************************
  VARIABLES
*****************************************************************************/

/*****************************************************************************
  MAIN FUNCTION
*****************************************************************************/

/*****************************************************************************
  Functionname: pGetRCWCalculatePointer                                  */ /*!

  @brief

  @description

  @param[in]

  @return
*****************************************************************************/
RCWCalculate_st* pGetRCWCalculatePointer() {
  return &RCWCalculate;
}

RCW_Info_Array* pGetRCWCalculatePointer_RCWAllObjInfo() {
  return &RCWCalculate.RCWObjInfoList;
}

RCW_Info_t* pGetRCWCalculatePointer_RCWObjInfoList(uint8 uObj) {
  return &RCWCalculate.RCWObjInfoList[uObj];
}

RCWCorridorObserver_t* pGetRCWCalculatePointer_RCWCorridorObjs(uint8 uidx) {
  return &RCWCalculate.RCWCorridorObjs[uidx];
}

RCWWarningInfo_t* pGetRCWCalculatePointer_RCWWarningInfo() {
  return &RCWCalculate.RCWWarningInfo;
}

RCWStationaryBlocked_t* pGetRCWCalculatePointer_RCWStationaryBlocker() {
  return &RCWCalculate.RCWStationaryBlocker;
}

// boolean RCWbGetObjRightSensorFlag(uint8 uObj, RCW_GenObject_st*
// pGenObjInfo)
// {
// 	return pGenObjInfo->bRightSensor;
// }

RCWWarningInfo_t* pGetRCWCalculatePointer_RCWWarningInfo_t() {
  return &RCWCalculate.RCWWarningInfo;
}

RCWStatusCondition_t* pGetRCWCalculatePointer_RCWStatusCondition() {
  return &RCWCalculate.RCWStatusCondition;
}

RCWStateMachine_t* pGetRCWCalculatePointer_RCWstatemachine() {
  return &RCWCalculate.RCWstatemachine;
}

RCWStateMachine_t* pGetRCWCalculatePointer_RCWstatemachineLastCycle() {
  return &RCWCalculate.RCWstatemachineLastCycle;
}

/*****************************************************************************
  CALCULATION FUNCTION
*****************************************************************************/
/*****************************************************************************
  Functionname: RCWbGetObjIsDeleted                                  */ /*!

  @brief

  @description

  @param[in]

  @return
  *****************************************************************************/
/*****************************************************************************
@startuml
start
if (uiMaintenanceState_nu == 0)then(yes)
:bRet = TRUE;
else(no)
:bRet = FALSE;
endif
:return bRet;
stop
@enduml
*****************************************************************************/
boolean RCWbGetObjIsDeleted(uint8 uObj, const RCWGenObjList_st* pGenObjList) {
  // Obj number check
  boolean bRet = FALSE;
  if (pGenObjList->aObject[uObj].General.uiMaintenanceState_nu ==
      RCW_EM_GEN_OBJECT_MT_STATE_DELETED) {
    bRet = TRUE;
  } else {
    bRet = FALSE;
  }
  return bRet;
}

float32 pGetObjLongDisplacement(uint8 uObj,
                                const RCWGenObjList_st* pGenObjList) {
  return pGenObjList->aObject[uObj].Kinemactic.fDistX_met;
}

float32 pGetObjLatDisplacement(uint8 uObj,
                               const RCWGenObjList_st* pGenObjList) {
  return pGenObjList->aObject[uObj].Kinemactic.fDistY_met;
}

float32 pGetObjLongVrel(uint8 uObj, const RCWGenObjList_st* pGenObjList) {
  return pGenObjList->aObject[uObj].Kinemactic.fVrelX_mps;
}

float32 pGetObjDist2Course(uint8 uObj, const RCWGenObjList_st* pGenObjList) {
  return pGenObjList->aObject[uObj].RoadRelation.fDist2Course_met;
}

/*****************************************************************************
  Functionname: LBSRCWCustomCalculateTTCThreshold */ /*!

  @brief:Calculate the vrel dependent TTC threshold

  @description:Calculate the vrel dependent TTC threshold

  @param[in]:uidx pGenObj RCWparams

  @return:void
*****************************************************************************/
void LBSRCWCustomCalculateTTCThreshold(const RCWParam_st* RCWparams,
                                       const RCWGenObjList_st* pGenObj,
                                       uint8 uidx, RCW_Info_t* pRCWObjInfo) {
  float32 fTTCThreshold = 0.F;
  const float32 fVxObj = pGetObjLongVrel(uidx, pGenObj);
  const boolean bRCWWarningLastCycle = pRCWObjInfo->bRCWWarning;

  if (fVxObj >= RCWparams->RCWWarningParameter.fVrelTTCMax) {          // 22.22
    fTTCThreshold = RCWparams->RCWWarningParameter.fTTCThreshVrelMax;  // 4.4
  } else {
    if (fVxObj > RCWparams->RCWWarningParameter.fVrelTTCMin) {  // 8.33
      // fTTCThreshold = RCWparams->RCWWarningParameter.fTTCThreshVrelMax
      // * ((fVxObj * (1.f / RCWparams->RCWWarningParameter.fVrelTTCMin))
      // - 1.F);

      fTTCThreshold = TUE_CML_BoundedLinInterpol2(
          fVxObj,
          RCWparams->RCWWarningParameter.fVrelTTCMin,         // 30kph
          RCWparams->RCWWarningParameter.fVrelTTCMax,         // 80kph
          RCWparams->RCWWarningParameter.fTTCThreshVrelMin,   // 2.2f
          RCWparams->RCWWarningParameter.fTTCThreshVrelMax);  // 5.2f

      if (bRCWWarningLastCycle == TRUE) {
        fTTCThreshold += 0.1F;
      }
    } else {
      fTTCThreshold = RCWparams->RCWWarningParameter.fTTCThreshVrelMin;  // 2.2f
    }
  }

  pRCWObjInfo->fTTCThreshold = fTTCThreshold;
}

/*****************************************************************************
  Functionname: LBSRCWCheckObjectQuality                                  */ /*!

  @brief:check if the current object quality is high enough for RCW

  @description:check if the current object quality is high enough for RCW

  @param[in]:uidx pGenObj

  @return:void
*****************************************************************************/
void LBSRCWCheckObjectQuality(const RCWGenObjList_st* pGenObj, uint8 uidx,
                              RCW_Info_t* pRCWObjInfo,
                              const RCW_LBSInputInfo_st* pGetLBSInputInfo) {
  const float32 fProbabilityOfExistence =
      pGenObj->aObject[uidx].Qualifiers.fProbabilityOfExistence_per;
  float32 fMinRCWUpdateRate = LBS_RCW_MIN_UPDATERATE;
  float32 fMinPoE = LBS_RCW_MIN_POE;
  boolean bRCWQuality = FALSE;

  // if the object is already marked as warning or relevent for RCW
  // lower quality threshold are use
  if (pRCWObjInfo->bRCWWarning == TRUE) {
    fMinRCWUpdateRate = LBS_RCW_MIN_UPDATERATE_REVELENT;
    fMinPoE = LBS_RCW_MIN_POE_REVEVANT;
  } else {
    if (pRCWObjInfo->bRCWQuality == TRUE) {
      fMinRCWUpdateRate = LBS_RCW_MIN_UPDATERATE_REVELENT;
      fMinPoE = LBS_RCW_MIN_POE;
    } else {
      fMinPoE = LBS_RCW_MIN_POE_INITAL;
      // check for a high update rate for object
      // at close to mid distance with low movement in x direction since
      // start
      if ((pGetLBSInputInfo->LBSObjInfoList[uidx].fXMovement_met <
           LBS_RCW_MIN_X_MOVEMENT_RELEVENT) &&
          (pGetObjLongDisplacement(uidx, pGenObj) >
           LBS_RCW_MIN_DISTX_RELEVENT)) {
        fMinRCWUpdateRate = LBS_RCW_QUAL_INACTIVE_UPDATE_MIN;
      } else {
        // check if a high update rate for objects that have a low TTC
        // directly after creation
        if (pGetLBSInputInfo->LBSObjInfoList[uidx].bLowTTCAtStart == TRUE) {
          // check for an even higher update rate if these object are
          // created adjacent to a stable object beacuse it is very
          // likely that these object are non-relevent distortions
          if (pGetLBSInputInfo->LBSObjInfoList[uidx].bCreateAdjStableObj ==
              TRUE) {
            fMinRCWUpdateRate = LBS_RCW_QUAL_ADJOBJ_UPDATE_MIN;
          } else {
            fMinRCWUpdateRate = LBS_RCW_QUAL_LOWTTC_UPDATE_MIN;
          }
        } else {
          // check for a high update rate if the object is created
          // adjancet to a stable object
          if (pGetLBSInputInfo->LBSObjInfoList[uidx].bCreateAdjStableObj ==
              TRUE) {
            fMinRCWUpdateRate = LBS_RCW_QUAL_INACTIVE_UPDATE_MIN;
          }
        }
      }
    }
  }

  // for already warning objects check if either the updaterate or
  // the probability of existence is larger than the defined threshold
  if (pRCWObjInfo->bRCWWarning == TRUE) {
    if ((pGetLBSInputInfo->LBSObjInfoList[uidx].fUpdateRate_nu >
         fMinRCWUpdateRate) ||
        (fProbabilityOfExistence > fMinPoE)) {
      bRCWQuality = TRUE;
    }
  } else {
    // for non-warning ojects check if update rate and the probability
    // of existence are larger than the defined threshold
    if ((pGetLBSInputInfo->LBSObjInfoList[uidx].fUpdateRate_nu >
         fMinRCWUpdateRate) &&
        (fProbabilityOfExistence > fMinPoE)) {
      bRCWQuality = TRUE;
    }
  }

  pRCWObjInfo->bRCWQuality = bRCWQuality;
}

// /*****************************************************************************
//   Functionname: LBSRCWCheckMultipathOnject */ /*!

//   @brief:check if the current object quality is a multipath object

//   @description:check if the current object quality is a multipath object

//   @param[in]:pGenObj,uidx,pRCWAllObjInfo,pRCWObjInfo

//   @return:void
// *****************************************************************************/
/*void LBSRCWCheckMultipathObject(const RCWGenObjList_st* pGenObj, uint8 uidx,
RCW_Info_Array* pRCWAllObjInfo, RCW_Info_t* pRCWObjInfo)
{
  // uint8 uObjSearchIdx;
  uint8 uMultiPathThreshold = 5U;
  boolean bMultiPathCond = FALSE;
 const float32 fXObj = pGetObjLongDisplacement(uidx, pGenObj) +
pGenObj->aObject[uidx].Geometry.fLengthFront_met; float32 fVxObj =
pGetObjLongVrel(uidx, pGenObj);const float32 fObjDist2Course =
pGetObjDist2Course(uidx, pGenObj);const float32 fVxMargin =
LBS_RCW_MULTIP_VXMARGIN_FACTOR * fVxObj;

  if(   (fVxObj > LBS_RCW_VX_MIN_MULTIPATH_OBJ)
      &&(fXObj > LBS_RCW_X_MIN_MULTIPATH_OBJ) )
  {
    for(uint8 uObjSearchIdx = 0U; (uObjSearchIdx < RCW_INPUT_OBJECT_NUMBER) &&
(bMultiPathCond == FALSE); uObjSearchIdx++)
    {
      if(     (!RCWbGetObjIsDeleted(uObjSearchIdx, pGenObj)         )
          &&  (pRCWAllObjInfo[uObjSearchIdx]->bRCWQuality == TRUE)
          &&  (uObjSearchIdx != uidx) )
      {
        const float32 fXCurrObj = pGetObjLongDisplacement(uObjSearchIdx,
pGenObj); const float32 fVxCurrObj = pGetObjLongVrel(uObjSearchIdx, pGenObj);
        const float32 fCurrObjDist2Crs = pGetObjDist2Course(uObjSearchIdx,
pGenObj); const float32 fDiffObjDist2Course = fABS(fObjDist2Course -
fCurrObjDist2Crs); const float32 fXMaxMultiPath = fXCurrObj -
GDBmathLinFuncLimBounded(fXObj, LBS_RCW_LI_MULTIP_MIN_DISTX,
                                                                            LBS_RCW_LI_MULTIP_MAX_DISTX,
                                                                            LBS_RCW_LI_MULTIP_CUR_MIN_DISTX,
                                                                            LBS_RCW_LI_MULTIP_CUR_MAX_DISTX);

        if(   (   fXObj       <     fXMaxMultiPath )
            &&(     (fXObj      >   LBS_RCW_MULTIP_DISTX_MIN)
                  ||(fXObj    <     ((2.f * fXCurrObj)  +
LBS_RCW_X_MARGIN_MULTIPATH_OBJ) )     )
            &&(   fXObj         >   ((2.f * fXCurrObj)  -
LBS_RCW_X_MARGIN_MULTIPATH_OBJ)       )
            &&(   fVxObj      <     ((2.f * fVxCurrObj) + fVxMargin )       )
            &&(   fVxObj        >   ((2.f * fVxCurrObj) - fVxMargin )       )
            &&(   fDiffObjDist2Course < LBS_RCW_MULTIP_DIST2CRS_DIFF ) )
        {
          bMultiPathCond = TRUE;
        }
      }
    }
  }

  if(bMultiPathCond == TRUE)
  {
    pRCWObjInfo->uMultiPathCnt = TUE_CML_Min(pRCWObjInfo->uMultiPathCnt + 1U,
LBS_RCW_MULTIP_CNP_MAX);
  }
  else
  {
    if(pRCWObjInfo->uMultiPathCnt > 0U)
    {
      pRCWObjInfo->uMultiPathCnt--;
    }
  }

  //choose the multipath threshold based on the previous assignment of the
object if(pRCWObjInfo->bMultiPathObj == TRUE)
  {
    uMultiPathThreshold = LBS_RCW_MULTIP_ACTIVE_CNT_THRESH;
  }
  else
  {
    uMultiPathThreshold = LBS_RCW_MULTIP_INACTIVE_CNT_THRESH;
  }

  if(pRCWObjInfo->uMultiPathCnt > uMultiPathThreshold)
  {
    pRCWObjInfo->bMultiPathObj = TRUE;
  }
  else
  {
    pRCWObjInfo->bMultiPathObj = FALSE;
  }
}*/

/*****************************************************************************
  Functionname: LBSRCWCheckUpdateStatus                                  */ /*!

  @brief:check if the current update status of the object

  @description:check if the current update status of the object

  @param[in]:pGenObj,uidx,pGetLBSInputInfo,pRCWObjInfo

  @return:void
*****************************************************************************/
void LBSRCWCheckUpdateStatus(const RCWGenObjList_st* pGenObj, uint8 uidx,
                             RCW_Info_t* pRCWObjInfo,
                             const RCW_LBSInputInfo_st* pGetLBSInputInfo) {
  uint8 uMeasuredTargetFrequency =
      pGenObj->aObject[uidx].Qualifiers.uiMeasuredTargetFrequency_nu;
  boolean bUpdateRecentlyTmp = FALSE;

  if ((uMeasuredTargetFrequency & LBS_UI_240_TO_BINARY) ==
      LBS_UI_240_TO_BINARY) {
    bUpdateRecentlyTmp = TRUE;
  } else {
    if ((pGetLBSInputInfo->LBSObjInfoList[uidx].fUpdateRate_nu >
         LBS_RCW_UPDATED_MIN_UPDATE) &&
        ((uMeasuredTargetFrequency & LBS_UI_224_TO_BINARY) ==
         LBS_UI_224_TO_BINARY)) {
      bUpdateRecentlyTmp = TRUE;
    }
  }

  pRCWObjInfo->bUpdateRecently = bUpdateRecentlyTmp;
}

/*****************************************************************************
  Functionname: LBSRCWCalculateYBreakThrough                                  */ /*!

  @brief:Calculate YBreakThrough      

  @description:Calculate YBreakThrough  

  @param[in]:pGenObj,uidx,pGetLBSInputInfo,pRCWObjInfo

  @return:void
*****************************************************************************/
void LBSRCWCalculateHeadingFilter(const RCWGenObjList_st* pGenObj, uint8 uidx,
                                  RCW_Info_t* pRCWObjInfo,
                                  const RCW_LBSInputInfo_st* pGetLBSInputInfo) {
  // float32 fYBreakThrough = TUE_C_F32_VALUE_INVALID;
  const RCW_GenObjGeometry_t* pEMGeometry = &pGenObj->aObject[uidx].Geometry;
  // const float32 fXObj = pGetObjLongDisplacement(uidx, pGenObj) +
  // pGetLBSInputInfo->LBSGlobalInfo.fSensorOffsetToRear_met +
  // pEMGeometry->fLengthFront_met; const float32 fYObj =
  // pGetObjLatDisplacement(uidx, pGenObj); const float32 fMaxHeadingInput =
  // (C_PI * 0.5F) - TUE_C_F32_DELTA;

  // if(fABS(pEMGeometry->fAbsOrientation_rad) < fMaxHeadingInput)
  // {
  //   fYBreakThrough = fYObj;
  //   fYBreakThrough += TAN_HD_(pEMGeometry->fAbsOrientation_rad) *
  //   fABS(fXObj);
  // }
  // else
  // {
  //   fYBreakThrough = TUE_C_F32_VALUE_INVALID;
  // }

  pRCWObjInfo->fHeadingFiltered = pEMGeometry->fAbsOrientation_rad;
  // pRCWObjInfo->fYBreakThrough = fYBreakThrough;
}

/*****************************************************************************
  Functionname: LBSRCWCheckHeadingInRange                                  */ /*!

  @brief:check if the heading angle is in special range

  @description:check if the heading angle is in special range

  @param[in]:pGenObj,uidx,RCWparams,pRCWObjInfo

  @return:void
*****************************************************************************/
void LBSRCWCheckHeadingInRange(const RCWGenObjList_st* pGenObj, uint8 uidx,
                               RCW_Info_t* pRCWObjInfo,
                               const RCWParam_st* RCWparams) {
  float32 fHeadingAngleThreshold = 0.F;
  boolean bHeadingAngleInRange = FALSE;
  float32 fHeadingAngle_rad = pRCWObjInfo->fHeadingFiltered;  // JO
  // float32 fHeadingAngle_rad = TUE_RAD2DEG(pRCWObjInfo->fHeadingFiltered);

  // choose the angle threshold based on the whether the object carries a
  // warning or not
  if (pRCWObjInfo->bRCWWarning == TRUE) {
    fHeadingAngleThreshold = RCWparams->RCWWarningParameter.fMaxHeadingAngle;
  } else {
    fHeadingAngleThreshold = RCWparams->RCWWarningParameter.fMinHeadingAngle;
  }

  if (fABS(fHeadingAngle_rad) < fHeadingAngleThreshold) {
    bHeadingAngleInRange = TRUE;
  }

  pRCWObjInfo->bHeadingAngleInRange = bHeadingAngleInRange;
}

/*****************************************************************************
  Functionname: LBSRCWProcessCorridor                                  */ /*!

  @brief:Corridor processing function for an object

  @description:Corridor processing function for an object

  @param[in]:pGenObj,uidx,RCWparams,pRCWObjInfo

  @return:void
*****************************************************************************/
void LBSRCWProcessCorridor(const RCWGenObjList_st* pGenObj, uint8 uidx,
                           RCW_Info_t* pRCWObjInfo,
                           const RCW_LBSInputInfo_st* pGetLBSInputInfo,
                           const RCWParam_st* RCWparams,
                           const RCWVehicleInfo_t* pGetEgoInfo) {
  LBSRCWCorridor_t sRCWCorridor = {0.F, 0.F};

  // Calculate the RCW corridor width based on the distance of the object
  LBSRCWCalculateCorridor(pGenObj, uidx, &sRCWCorridor, RCWparams);

  // perform the RCW corridor association
  LBSRCWCorridorAssociation(pGenObj, uidx, &sRCWCorridor, RCWparams,
                            pGetEgoInfo, pRCWObjInfo, pGetLBSInputInfo);
}

/*****************************************************************************
  Functionname: LBSRCWCalculateCorridor                                  */ /*!

  @brief:calculate the RCW corridor

  @description:calculate the RCW corridor

  @param[in]:pGenObj,uidx,pRCWCorridor,RCWparams

  @return:void
*****************************************************************************/
void LBSRCWCalculateCorridor(const RCWGenObjList_st* pGenObj, uint8 uidx,
                             LBSRCWCorridor_t* pRCWCorridor,
                             const RCWParam_st* RCWparams) {
  const float32 fXObj = pGetObjLongDisplacement(uidx, pGenObj);
  const float32 fRCWCorridorXMid =
      RCWparams->RCWWarningParameter.fRCWCorridorXMid;
  const float32 fRCWCorridorXMin =
      RCWparams->RCWWarningParameter.fRCWCorridorXMin;
  const float32 fRCWCorridorNarrow =
      RCWparams->RCWVehParameter.fVehicleWidth_met;
  const float32 fRCWCorridorWidth = fRCWCorridorNarrow + 0.5F;

  // calculate the RCW corridor width based on the distance of the object
  float32 fWidthHalf = GDBmathLinFuncLimBounded(
      fXObj, fRCWCorridorXMin, fRCWCorridorXMid, (fRCWCorridorWidth * 0.5F),
      (fRCWCorridorNarrow * 0.5F));

  pRCWCorridor->fCorridorBracketLeft = fWidthHalf;
  pRCWCorridor->fCorridorBracketRight = -fWidthHalf;
}

/*****************************************************************************
  Functionname: LBSRCWCalculateCorridor                                  */ /*!

  @brief:perform the corrdior association

  @description:perform the corrdior association

  @param[in]:pGenObj,uidx,pRCWCorridor,RCWparams

  @return:void
*****************************************************************************/
void LBSRCWCorridorAssociation(const RCWGenObjList_st* pGenObj, uint8 uidx,
                               LBSRCWCorridor_t* pRCWCorridor,
                               const RCWParam_st* RCWparams,
                               const RCWVehicleInfo_t* pGetEgoInfo,
                               RCW_Info_t* pRCWObjInfo,
                               const RCW_LBSInputInfo_st* pGetLBSInputInfo) {
  RCWSITrajOccupancy_t sOccupancy = {0.F, 0.F, 0.F, 0.F, 0.F, 0.F, FALSE};
  float32 fTrajOccupancyThresh = 0.F;
  float32 fTimeInFactor = 0.F;
  float32 fDistToXAxix = pGetObjLatDisplacement(uidx, pGenObj);
  boolean bInRCWCorridorTmp = FALSE;
  float32 fOwnSpeed = fABS(pGetEgoInfo->fegoVelocity_mps);
  float32 fObjWidth = pGenObj->aObject[uidx].Geometry.fWidthLeft_met +
                      pGenObj->aObject[uidx].Geometry.fWidthRight_met;
  float32 fTaskCycleTime = RCWparams->RCWWarningParameter.fCycletime_s;
  float32 fInCOrridorTime_tmp = 0.0F;  // JO
  // if(pGenObj->aObject->bRightSensor == TRUE)
  // {
  //   fDistToXAxix = pGetObjLongDisplacement(uidx, pGenObj) +
  //   RCWparams->SensorMounting.SensorRight.fLatPos_met;
  // }
  // else
  // {
  //   fDistToXAxix = pGetObjLongDisplacement(uidx, pGenObj) +
  //   RCWparams->SensorMounting.SensorLeft.fLatPos_met;
  // }

  // calculate the corridor overlap and occupancy
  // corridor is straight when ego is at standstill
  if (fOwnSpeed <= LBS_RCW_STRAIGHT_CORRIDOR_SPEED_THRESH) {
    LBSRCWCalculateInCorridorOverlap(
        pGenObj, uidx, fDistToXAxix, pRCWCorridor->fCorridorBracketLeft,
        pRCWCorridor->fCorridorBracketRight, LBS_RCW_MIN_OBJECT_WIDTH,
        LBS_RCW_MAX_OBJECT_WIDTH, &sOccupancy);
  }
  // otherwise, corridor follow ego trace
  else {
    LBSRCWCalculateInCorridorOverlap(
        pGenObj, uidx, pGetLBSInputInfo->LBSObjInfoList[uidx].fDistToTraj_met,
        pRCWCorridor->fCorridorBracketLeft, pRCWCorridor->fCorridorBracketRight,
        LBS_RCW_MIN_OBJECT_WIDTH, LBS_RCW_MAX_OBJECT_WIDTH, &sOccupancy);
  }

  pRCWObjInfo->fCorridorOverlap = sOccupancy.fOverlap_met;
  pRCWObjInfo->fCorridorOccupancy = sOccupancy.fTrajectoryOccupancy_per;
  pRCWObjInfo->bOppositeOverlap = sOccupancy.bOppositeOverlap_met;
  pRCWObjInfo->fObjectOccupancy = sOccupancy.fObjectOccupancy_per;

  // choose the occupancy threshold based on the previous corrdior assignment
  // of the object
  if (pRCWObjInfo->bInRCWCorridor == TRUE) {
    fTrajOccupancyThresh = RCWparams->RCWWarningParameter.fCorrOccDropThresh;
  } else {
    fTrajOccupancyThresh = RCWparams->RCWWarningParameter.fCorrOccPickupThresh;
  }

  // for small objects a smaller occupancy threshold is used with a hit
  // counter for corridor association
  if ((sOccupancy.fTrajectoryOccupancy_per >
       RCWparams->RCWWarningParameter.fCorrOccHitThresh) &&
      (fObjWidth <
       RCWparams->RCWWarningParameter.fCorrHitCntSmallObjWidth /*2.5*/)) {
    pRCWObjInfo->uCorridorHitCnt = TUE_CML_Min(
        pRCWObjInfo->uCorridorHitCnt + 1U, LBS_RCW_CORRIDOR_HIT_CNT_MAX);
  } else {
    if (pRCWObjInfo->uCorridorHitCnt > 1U) {
      pRCWObjInfo->uCorridorHitCnt = pRCWObjInfo->uCorridorHitCnt - 2U;
    }
  }

  // if the corridor occupancy of the object is larger than the threshold or
  // the corridor hit counter is larger enough the in-corridor-time is
  // increased, otherwise is is reset to zero
  if ((sOccupancy.fTrajectoryOccupancy_per > fTrajOccupancyThresh) &&
      (pRCWObjInfo->uCorridorHitCnt >
       RCWparams->RCWWarningParameter.uCorridorHitCntThresh)) {
    fTimeInFactor = GDBmathLinFuncLimBounded(
        sOccupancy.fTrajectoryOccupancy_per,
        RCWparams->RCWWarningParameter.fCorrOccHitThresh,
        LBS_RCW_LI_TIMEIN_OCC_MAX, fTaskCycleTime, (fTaskCycleTime * 2.F));

    // pRCWObjInfo->fInCOrridorTime =
    //     MIN((pRCWObjInfo->fInCOrridorTime + fTimeInFactor),
    //         TUE_C_F32_VALUE_INVALID);
    fInCOrridorTime_tmp = pRCWObjInfo->fInCOrridorTime + fTimeInFactor;
    pRCWObjInfo->fInCOrridorTime =
        MIN(fInCOrridorTime_tmp, TUE_C_F32_VALUE_INVALID);
  } else {
    pRCWObjInfo->fInCOrridorTime = 0.F;
  }

  if (pRCWObjInfo->fInCOrridorTime >
      RCWparams->RCWWarningParameter.fMinTimeInCorridorThresh) {
    bInRCWCorridorTmp = TRUE;
  } else {
    bInRCWCorridorTmp = FALSE;
  }

  pRCWObjInfo->bInRCWCorridor = bInRCWCorridorTmp;
}

/*****************************************************************************
  Functionname: LBSRCWCalculateInCorridorOverlap */ /*!

  @brief:choose the occupancy threshold based on the position of the target to the sensor

  @description:choose the occupancy threshold based on the position of the target to the sensor

  @param[in]:pGenObj,uobj,fObjRefDist,fTraceBracket,fObjWidth,pOccupancy

  @return:void
*****************************************************************************/
void LBSRCWCalculateInCorridorOverlap(const RCWGenObjList_st* pGenObj,
                                      uint8 uobj, const float32 fObjRefDist,
                                      const float32 fTraceBracketLeft,
                                      const float32 fTraceBracketRight,
                                      const float32 fMinObjWidth,
                                      const float32 fMaxObjWidth,
                                      RCWSITrajOccupancy_t* pOccupancy) {
  const RCW_GenObjGeometry_t* pObjGeometry = &pGenObj->aObject[uobj].Geometry;
  float32 fObjRefDistInternal = fObjRefDist;
  float32 fBracketWidth = fTraceBracketLeft - fTraceBracketRight;
  float32 fOverlap = 0.F;
  float32 fObjOccupancy = 0.F;
  float32 fTrajOccupancy = 0.F;
  float32 fObjectWidthLeft = 0.F;
  float32 fObjectWidthRight = 0.F;
  float32 fObjectWIdth = 0.F;
  float32 fDistToTrajMax = 0.F;
  float32 fDistToTrajMin = 0.F;
  boolean bOppositeOverlap = FALSE;

  // limit the object width to be vetween 0.m and 2.55m
  fObjectWidthLeft = MAX(pObjGeometry->fWidthLeft_met, (fMinObjWidth * 0.5F));
  fObjectWidthLeft = MIN(fObjectWidthLeft, (fMaxObjWidth * 0.5F));

  fObjectWidthRight = MAX(pObjGeometry->fWidthRight_met, (fMinObjWidth * 0.5F));
  fObjectWidthRight = MIN(fObjectWidthRight, (fMaxObjWidth * 0.5F));

  // assure that the object width is larger than TUE_C_F32_DELTA for divisions
  fObjectWIdth = MAX((fObjectWidthLeft + fObjectWidthRight), TUE_C_F32_DELTA);

  // assure that the bracket is larger than TUE_C_F32_DELTA for divisions
  fBracketWidth = MAX(fBracketWidth, TUE_C_F32_DELTA);

  fDistToTrajMax = fObjRefDistInternal + fObjectWidthLeft;
  fDistToTrajMin = fObjRefDistInternal - fObjectWidthRight;

  // cover unusal case that the object width is larger than the bracket width
  if ((fObjectWIdth >= fBracketWidth) &&
      (fDistToTrajMin <= fTraceBracketRight) &&
      (fDistToTrajMax >= fTraceBracketLeft)) {
    fOverlap = fBracketWidth;
    fObjOccupancy = 1.F;
    fTrajOccupancy = 1.F;
  } else {
    // no overlap
    if ((fDistToTrajMax < fTraceBracketRight) ||
        (fDistToTrajMin > fTraceBracketLeft)) {
      fOverlap = 0.F;
      fObjOccupancy = 0.F;
      fTrajOccupancy = 0.F;
    } else {
      // object full overlap in bracket
      if ((fDistToTrajMin > fTraceBracketRight) &&
          (fDistToTrajMax < fTraceBracketLeft)) {
        fOverlap = fObjectWIdth;
        fObjOccupancy = 1.F;
        fTrajOccupancy = fOverlap / fBracketWidth;
      } else {
        /* partial overlap with the left trace bracket  */
        /*          |                 |                 */
        /*                                              */
        /*          |                 |                 */
        /*  +----------------+                          */
        /*  |       |        |        |                 */
        /*  |                |                          */
        /*  |       |        |        |                 */
        /*  |                |                          */
        /*  |       |        |        |                 */
        /*  +----------------+                          */
        /*          |                 |                 */
        /*                                              */
        /*          |                 |                 */
        /*                                              */
        /*          |                 |                 */
        /*    Bracket Left      Bracket Right           */
        if (fDistToTrajMax > fTraceBracketLeft) {
          fOverlap = (fTraceBracketLeft - fDistToTrajMin);
          fObjOccupancy = fOverlap / fObjectWIdth;
          fTrajOccupancy = fOverlap / fBracketWidth;

          if (pGenObj->aObject[uobj].bRightSensor == TRUE) {
            bOppositeOverlap = TRUE;
          }
        }
        /* partial overlap with the left trace bracket  */
        /*          |                 |                 */
        /*                                              */
        /*          |                 |                 */
        /*                      +----------------+      */
        /*          |           |     |          |      */
        /*                      |                |      */
        /*          |           |     |          |      */
        /*                      |                |      */
        /*          |           |     |          |      */
        /*                      +----------------+      */
        /*          |                 |                 */
        /*                                              */
        /*          |                 |                 */
        /*                                              */
        /*          |                 |                 */
        /*    Bracket Left      Bracket Right           */
        else {
          if (fDistToTrajMin < fTraceBracketRight) {
            fOverlap = (fDistToTrajMax - fTraceBracketRight);
            fObjOccupancy = fOverlap / fObjectWIdth;
            fTrajOccupancy = fOverlap / fBracketWidth;

            if (pGenObj->aObject[uobj].bRightSensor != TRUE) {
              bOppositeOverlap = TRUE;
            }
          }
        }
      }
    }
  }

  pOccupancy->fOverlap_met = fOverlap;
  pOccupancy->fOverlapVar_met = 0.F;
  pOccupancy->fObjectOccupancy_per = fObjOccupancy;
  pOccupancy->fObjectOccupancyVar_per = 0.F;
  pOccupancy->fTrajectoryOccupancy_per = fTrajOccupancy;
  pOccupancy->fTrajectoryOccupancyVar_per = 0.F;
  pOccupancy->bOppositeOverlap_met = bOppositeOverlap;
}

/*****************************************************************************
  Functionname: LBSRCWCheckObjectRelevence                                  */ /*!

  @brief:check if the current object is relevent for RCW

  @description:check if the current object is relevent for RCW

  @param[in]:pGenObj,uidx,pRCWCorridor,RCWparams

  @return:void
*****************************************************************************/
void LBSRCWCheckObjectRelevence(const RCWGenObjList_st* pGenObj, uint8 uidx,
                                RCW_Info_t* pRCWObjInfo,
                                const RCWParam_st* RCWparams,
                                const RCW_LBSInputInfo_st* pGetLBSInputInfo) {
  boolean bRCWRelevent = FALSE;

  if ((pRCWObjInfo->bRCWWarning == TRUE) ||
      (pRCWObjInfo->bInRCWCorridor == TRUE)) {
    if (pRCWObjInfo->bRCWRelevant == FALSE) {
      // if ((pGetLBSInputInfo->LCAObjInfoList->bLCAMirrorFrontObject == FALSE)
      // &&
      if (((pGetLBSInputInfo->LBSObjInfoList[uidx].fXMovement_met >
            RCWparams->RCWWarningParameter
                .fReleventMinXMovement)  // xmovement > 5.f
           || (pGetObjLongDisplacement(uidx, pGenObj) <
               RCWparams->RCWWarningParameter.fReleventMaxDist)  // DX < -25.f
           || (pGenObj->aObject[uidx].General.uiLifeCycles_nu >
               RCWparams->RCWWarningParameter
                   .fReleventLifeCnt)) &&  // lifecycle > 2u
          (pGetObjLongDisplacement(uidx, pGenObj) <
           LBS_RCW_MAX_X_OPP_SIDE_RELEVENT)) {
        bRCWRelevent = TRUE;
      }
    } else {
      // object is already relevent, therefore keep this status

      bRCWRelevent = TRUE;
    }
  }

  // check for close object at the edge of the field of view
  // that should not enale an RCW warning anymore
  if (bRCWRelevent == TRUE) {
    const float32 fXObj = pGetObjLongDisplacement(uidx, pGenObj);
    const float32 fDist2Course = pGetObjDist2Course(uidx, pGenObj);
    const float32 fMaxDist2Course = GDBmathLinFuncLimBounded(
        fXObj, RCWparams->RCWWarningParameter.fReleventMinXOppSideRelevent,
        RCWparams->RCWWarningParameter.fReleventMaxXOppSideRelevent,
        RCWparams->RCWWarningParameter.fReleventDist2CourseMin,
        RCWparams->RCWWarningParameter.fReleventDist2CourseMax);

    // reset to deactivate
    if ((fXObj > RCWparams->RCWWarningParameter.fReleventMinXOppSideRelevent) &&
        (fDist2Course < fABS(fMaxDist2Course)) &&
        (pRCWObjInfo->fCorridorOccupancy <
         RCWparams->RCWWarningParameter.fReleventDeactCorrOccuThreshold) &&
        (pRCWObjInfo->bRCWWarning == FALSE)) {
      bRCWRelevent = FALSE;
    }
  }

  pRCWObjInfo->bRCWRelevant = bRCWRelevent;
}

/*****************************************************************************
  Functionname: LBSRCWCheckBlockedCorridor                                  */ /*!

  @brief:check if the corridor is blocked or not

  @description:check if the corridor is blocked or not

  @param[in]:pGenObj,uidx,pRCWCorridor,RCWparams

  @return:void
*****************************************************************************/
void LBSRCWCheckBlockedCorridor(const RCWGenObjList_st* pGenObj, uint8 uidx,
                                RCW_Info_t* pRCWObjInfo,
                                const RCWParam_st* RCWparams,
                                const RCW_LBSInputInfo_st* pGetLBSInputInfo) {
  uint8 uCorridorObjectIndex = 0U;
  float32 fCorrBlockedOccThreshold = 0.F;
  boolean bCorridorBlocked = FALSE;
  float32 fXObj = pGetObjLongDisplacement(uidx, pGenObj);

  if (pRCWObjInfo->bInRCWCorridor == TRUE) {
    // choose the blocked corridor occupancy threshold based on the precious
    // assignment of the corridor
    if (pRCWObjInfo->bObjCorridorBlocked == TRUE) {
      fCorrBlockedOccThreshold =
          RCWparams->RCWWarningParameter
              .fCorrBlockedActiveOccMin;  // low threshod easy lead to
                                          // blocked
    } else {
      fCorrBlockedOccThreshold =
          RCWparams->RCWWarningParameter.fCorrBlockedInActiveOccMin;
    }

    // for object 1 - 4, check if the corridor is blocked
    for (uCorridorObjectIndex = 0U;
         (uCorridorObjectIndex < RCW_MAX_NOF_CORR_OBJS) &&
         (bCorridorBlocked == FALSE);
         uCorridorObjectIndex++) {
      RCWCorridorObserver_t* tRCWCorridorObject =
          pGetRCWCalculatePointer_RCWCorridorObjs(uCorridorObjectIndex);
      // printf("uObjID = %d\t", tRCWCorridorObject->uObjID);
      if (tRCWCorridorObject->uObjID != uidx) {
        // Even RCWCorridorObject don't have selected,
        // it's default value could still lead bCorridorBlocked to False
        if ((fXObj > tRCWCorridorObject->fXDist) &&
            (pGetLBSInputInfo->LBSObjInfoList[uidx].ObjBorders.fXmin_met >
             tRCWCorridorObject->fXMax)) {
          bCorridorBlocked = FALSE;
        } else {
          if (pRCWObjInfo->fCorridorOccupancy > fCorrBlockedOccThreshold) {
            bCorridorBlocked = TRUE;
          }
        }
      }
    }
  }

  pRCWObjInfo->bObjCorridorBlocked = bCorridorBlocked;
  // printf("bObjCorridorBlocked(%d),uidx(%d)\t", bCorridorBlocked, uidx);
}

/*****************************************************************************
  Functionname: LBSRCWChooseOccupancyThreshold */ /*!

  @brief:choose the occupancy threshold based on the position of the target to the sensor

  @description:choose the occupancy threshold based on the position of the target to the sensor

  @param[in]:pGenObj,uidx,pRCWObjInfo,RCWparams

  @return:void
*****************************************************************************/
void LBSRCWChooseOccupancyThreshold(const RCWGenObjList_st* pGenObj, uint8 uidx,
                                    RCW_Info_t* pRCWObjInfo,
                                    const RCWParam_st* RCWparams) {
  float32 fCorridorOccThreshold = 0.F;

  if (pRCWObjInfo->bOppositeOverlap == TRUE) {
    fCorridorOccThreshold =
        RCWparams->RCWWarningParameter.fCorrOccWarnEnable +  // 0.3
        RCWparams->RCWWarningParameter.fOppoOccuAddAdjust;
  } else {
    fCorridorOccThreshold =
        RCWparams->RCWWarningParameter.fCorrOccWarnEnable;  // 0.35
  }

  pRCWObjInfo->fCorridorOccThreshold = fCorridorOccThreshold;
}

/*****************************************************************************
  Functionname: LBSRCWCheckWarningConditions                                  */ /*!

  @brief:check if the current object meets the RCW warning conditions

  @description:check if the current object meets the RCW warning conditions

  @param[in]:pGenObj,uidx,pRCWObjInfo,pGetLBSInputInfo

  @return:void
*****************************************************************************/
void LBSRCWCheckWarningConditions(const RCWGenObjList_st* pGenObj, uint8 uidx,
                                  RCW_Info_t* pRCWObjInfo,
                                  const RCW_LBSInputInfo_st* pGetLBSInputInfo) {
  boolean bRCWWarningConditions = FALSE;

  // Check if all warning condition are fulfilled
  // - object has enough quality
  // - object is relevent
  // - object is in RCW corridor
  // - object heading is in range
  // - object is not mirror
  // - object is onego lane

  if ((pRCWObjInfo->bRCWQuality == TRUE) &&
      (pRCWObjInfo->bInRCWCorridor == TRUE) &&
      (pRCWObjInfo->bRCWRelevant == TRUE) &&
      (pRCWObjInfo->bHeadingAngleInRange == TRUE) &&
      //(pGetLBSInputInfo->LCAObjInfoList->bLCAMirrorFrontObject == FALSE) &&
      // (pGetLBSInputInfo->LBSObjInfoList[uidx].eAssociatedLane ==
      //  ASSOC_LANE_EGO)  && // 3
      (pGetLBSInputInfo->BSDObjInfoList[uidx].bBSDWarning == FALSE)) {
    bRCWWarningConditions = TRUE;
  }

  pRCWObjInfo->bRCWWarningConditions = bRCWWarningConditions;
}

/*****************************************************************************
  Functionname: LBSRCWFinalWarningDecision                                  */ /*!

  @brief:check if the current object shall enable RCW function

  @description:check if the current object shall enable RCW function

  @param[in]:pGenObj,uidx,pRCWObjInfo,RCWparams

  @return:void
*****************************************************************************/
void LBSRCWFinalWarningDecision(const RCWGenObjList_st* pGenObj, uint8 uidx,
                                RCW_Info_t* pRCWObjInfo,
                                const RCW_LBSInputInfo_st* pGetLBSInputInfo,
                                const RCWParam_st* RCWparams,
                                RCWWarningInfo_t* pRCWCalcWarningInfo) {
  float32 fTTCThresholdAx = 0.F;
  boolean bEnableRCWWarning = FALSE;
  const float32 fAxObj = pGenObj->aObject[uidx].Kinemactic.fArelX_mpss;
  const float32 fTTCRelevant =
      pGetLBSInputInfo->LBSObjInfoList[uidx].fTTCAccel_mps2;
  const float32 fTTCThreshold = pRCWObjInfo->fTTCThreshold;
  const boolean bRCWWarningLastCycle = pRCWObjInfo->bRCWWarning;

  // if(uidx == 0U)
  // {
  //   printf("ObjTTc is %f\t",
  //   pGetLBSInputInfo->LBSObjInfoList[uidx].fTTCAccel_mps2);
  // }

  // reset warning from last cycle
  pRCWObjInfo->bRCWWarning = FALSE;

  if (pRCWObjInfo->bRCWWarningConditions == TRUE) {
    if (fTTCRelevant < fTTCThreshold) {
      if (bRCWWarningLastCycle == TRUE) {
        bEnableRCWWarning = TRUE;
      } else {
        // object does not warn yet
        /********************************************************/
        /*  check:                                              */
        /*  - object was updated recently                       */
        /*  - the corridor occupancy is larged enough           */
        /*  - the object is not classified as multipath object  */
        /*  - corridor is not blocked                           */
        /********************************************************/
        if ((pRCWObjInfo->bUpdateRecently == TRUE) &&
            (pRCWObjInfo->bObjCorridorBlocked == FALSE)
            // &&  (pRCWObjInfo->bMultiPathObj == FALSE)
            && (pRCWObjInfo->fCorridorOccupancy >
                pRCWObjInfo->fCorridorOccThreshold) &&
            (fABS(pGetLBSInputInfo
                      ->LBSObjInfoList[uidx]  // 0.9
                      .fVrelToTraj_mps) <
             RCWparams->RCWWarningParameter.fWarnDecisionVrel2TrajMax)  // 1.0
            &&
            (pGetLBSInputInfo->LBSObjInfoList[uidx].fAssocProbFiltered >  // 0.9
             RCWparams->RCWWarningParameter
                 .fWarnDecisionAssocProbMin)) {  // 0.8
          if ((fTTCThreshold > (RCWparams->RCWWarningParameter
                                    .fWarnDecisionTTCAdjustThreshold -  // 1.4
                                TUE_C_F32_DELTA)) &&
              (fAxObj <  //(-1.1 < -0.5)
               RCWparams->RCWWarningParameter.fWarnDecisionArelXMaxTTCAdjust)) {
            // for object with a large deceleration reduce the TTC
            // threshold to avoid FA on braking targets therefore
            // slightly reduce TTC  threshold based on the currently
            // estimated deceleration

            // TTC prediction based on accelaration
            fTTCThresholdAx = GDBmathLinFuncLimBounded(
                fAxObj,
                RCWparams->RCWWarningParameter.fWarnDecisionArelXMaxTTCAdjust,
                RCWparams->RCWWarningParameter.fWarnDecisionArelXMinTTCAdjust,
                RCWparams->RCWWarningParameter.fTTCThreshVrelMax,
                (RCWparams->RCWWarningParameter.fTTCThreshVrelMax -
                 RCWparams->RCWWarningParameter
                     .fWarnDecisionArelXTTCReduceFactor));
            if (fTTCRelevant < fTTCThresholdAx) {
              bEnableRCWWarning = TRUE;
            }
          } else {
            bEnableRCWWarning = TRUE;
          }
        }
      }
    }
  } else {
    // do nothing
  }

  if (bEnableRCWWarning == TRUE) {
    pRCWObjInfo->bRCWWarning = TRUE;
    pRCWCalcWarningInfo->bRCWWarningActive = TRUE;
  }
}

/*****************************************************************************
  Functionname: LBSRCWStoreWarningObjInfor                                  */ /*!

  @brief:store the warning information of the closet warning object

  @description:store the warning information of the closet warning object

  @param[in]:pGenObj,uidx,pRCWObjInfo,RCWparams

  @return:void
*****************************************************************************/
void LBSRCWStoreWarningObjInfor(const RCWGenObjList_st* pGenObj, uint8 uidx,
                                RCW_Info_t* pRCWObjInfo,
                                const RCW_LBSInputInfo_st* pGetLBSInputInfo,
                                RCWWarningInfo_t* pRCWCalcWarningInfo) {
  const float32 fObjLengthFront =
      pGenObj->aObject[uidx].Geometry.fLengthFront_met;
  const float32 fXObj =
      pGetObjLongDisplacement(uidx, pGenObj) +
      (fObjLengthFront +
       pGetLBSInputInfo->LBSGlobalInfo.fSensorOffsetToRear_met);
  const float32 fTTC_t = pGetLBSInputInfo->LBSObjInfoList[uidx].fTTCAccel_mps2;
  if (pRCWObjInfo->bRCWWarning == TRUE) {
    if (fXObj > pRCWCalcWarningInfo->fXObjectWarning) {
      pRCWCalcWarningInfo->fXObjectWarning = fXObj;
      pRCWCalcWarningInfo->uRCWWarningID = uidx;
      pRCWCalcWarningInfo->fTTC = fTTC_t;
    }
  }
}

/*****************************************************************************
  Functionname: LBSRCWCorrIsStationaryBlocked */ /*!

  @brief:determine if the corridor is blocked by stationary target at standstill

  @description:determine if the corridor is blocked by stationary target at standstill

  @param[in]:pGetEgoInfo,pGenObj,RCWparams,tRCWCorridorObject

  @return:void
*****************************************************************************/
void LBSRCWCorrIsStationaryBlocked(
    const RCWVehicleInfo_t* pGetEgoInfo, const RCWGenObjList_st* pGenObj,
    const RCWParam_st* RCWparams,
    RCWStationaryBlocked_t* pRCWStationaryBlocker) {
  uint8 uFirstRCWRelevantObj =
      pGetRCWCalculatePointer_RCWCorridorObjs(0)->uObjID;
  float32 fEgoStationaryThreshold =
      RCWparams->RCWWarningParameter.fEgoVelStationaryCheck;  // 0.1f
  float32 fOwnSpeed = fABS(pGetEgoInfo->fegoVelocity_mps);

  if (fOwnSpeed < fEgoStationaryThreshold) {
    // target is stationary, continue with the logic
    // if the corridor was blocked in the previous cycle, then we maintain
    // it blocked in this cycle
    if (pRCWStationaryBlocker->bCorridorBlockedLastCycle == TRUE &&
        uFirstRCWRelevantObj != TUE_C_UI8_VALUE_INVALID &&
        fABS(pGenObj->aObject[uFirstRCWRelevantObj].Kinemactic.fDistX_met) <
            15.0) {
      pRCWStationaryBlocker->bCorridorBlocked = TRUE;
    } else {
      // check if the first object in the RCW corridor local has the
      // classification STOPPED
      if (uFirstRCWRelevantObj == TUE_C_UI8_VALUE_INVALID) {
        // no relevant object in the corridor, no reason to blocked it
        pRCWStationaryBlocker->bCorridorBlocked = FALSE;
        pRCWStationaryBlocker->fBlockXPosition = TUE_C_F32_VALUE_INVALID;
      } else {
        // check the dynamic condition of the object
        if (pGenObj->aObject[uFirstRCWRelevantObj]
                .Attributes.eDynamicProperty_nu ==
            EM_GEN_OBJECT_DYN_PROPERTY_STOPPED) {
          // object is stopped in the corridor, blocked the corridor
          pRCWStationaryBlocker->bCorridorBlocked = TRUE;
          pRCWStationaryBlocker->fBlockXPosition =
              pGenObj->aObject[uFirstRCWRelevantObj].Kinemactic.fDistX_met;
        }
      }
    }
  } else {
    // ego car started to move, the flag should be reseted
    pRCWStationaryBlocker->bCorridorBlocked = FALSE;
    pRCWStationaryBlocker->fBlockXPosition = -TUE_C_F32_VALUE_INVALID;
  }
}

/*****************************************************************************
  Functionname: LBSRCWStateConditionProcess                                  */ /*!

  @brief:pre process the status conditions

  @description:pre process the status conditions

  @param[in]:pGetRCWStateConditions, pGetEgoInfo, pGetPreProcessInput, params, pRCWCalcWarningInfo, pRCWStationaryBlocker, debugInfo

  @return:void
*****************************************************************************/
#ifdef ST_PERCEPTION

void LBSRCWStateConditionProcess(
    RCWStatusCondition_t* pGetRCWStateConditions,
    const RCWVehicleInfo_t* pGetEgoInfo,
    const RCWPreProcessInput_t* pGetPreProcessInput, const RCWParam_st* params,
    RCWWarningInfo_t* pRCWCalcWarningInfo,
    const RCWStationaryBlocked_t* pRCWStationaryBlocker,
    RCWDebug_t* debugInfo) {
  const boolean tBlockTimerActive =
      pGetRCWCalculatePointer()->RCWBlockingTimeActive;

  // hmi open
  if (pGetPreProcessInput->RCWHmiOpen == TRUE) {
    pGetRCWStateConditions->bRCWHmiOpen = TRUE;
  } else {
    pGetRCWStateConditions->bRCWHmiOpen = FALSE;
  }

  // failure
  if (pGetPreProcessInput->RCWFailure == TRUE) {
    pGetRCWStateConditions->bRCWFailureCondition = TRUE;
  } else {
    pGetRCWStateConditions->bRCWFailureCondition = FALSE;
  }

  // suppression
  if ((pGetEgoInfo->fegoVelocity_mps >
       params->RCWWarningParameter.fSuppressEgoLongVelMax)  // vx > 140kph
      || (pGetEgoInfo->fegoVelocity_mps <
          params->RCWWarningParameter.fSuppressEgoLongVelMin)  // vx < 0kph
      || (fABS(pGetEgoInfo->fLatAccel_mps2) >
          params->RCWWarningParameter.fSuppressEgoLatAcceMax)  // ay > ---
      || (fABS(pGetEgoInfo->fLatVelocity_mps) >
          params->RCWWarningParameter.fSuppressEgoLatVelMax)  // vy > ---
      || (pGetPreProcessInput->LeftTurnLightOpen == TRUE)     // left turn light
      || (pGetPreProcessInput->RightTurnLightOpen == TRUE)  // right turn light
      || (1 == pGetPreProcessInput->uRCWGearPosition)       // R gear Passive
      || (tBlockTimerActive == TRUE)                        // blocking timer
  ) {
    pGetRCWStateConditions->bRCWPassiveCondition = TRUE;
  } else {
    pGetRCWStateConditions->bRCWPassiveCondition = FALSE;
  }

  // active condition
  if ((pRCWCalcWarningInfo->bRCWWarningActive == TRUE) &&
      (pRCWStationaryBlocker->bCorridorBlocked == FALSE)) {
    pGetRCWStateConditions->bRCWActiveConditon = TRUE;
  }

  debugInfo->Debug_RCWDebugSubConditions.debugActive_bRCWWarningActive =
      pRCWCalcWarningInfo->bRCWWarningActive;
  debugInfo->Debug_RCWDebugSubConditions.debugActive_bCorridorBlocked =
      pRCWStationaryBlocker->bCorridorBlocked;
  debugInfo->Debug_RCWDebugSubConditions.debugPassive_BlockingtimeActive =
      tBlockTimerActive;
  debugInfo->Debug_RCWDebugSubConditions.debugPassive_GearPosition =
      (1 == pGetEgoInfo->uRCWGearPosition);
  debugInfo->Debug_RCWDebugSubConditions.debugPassive_LatAcceloutofRange =
      (fABS(pGetEgoInfo->fLatAccel_mps2) >
       params->RCWWarningParameter.fSuppressEgoLatVelMax);
  debugInfo->Debug_RCWDebugSubConditions.debugPassive_LeftTurnLight =
      (pGetPreProcessInput->LeftTurnLightOpen == TRUE);
  debugInfo->Debug_RCWDebugSubConditions.debugPassive_RightTurnLight =
      (pGetPreProcessInput->RightTurnLightOpen == TRUE);
  debugInfo->Debug_RCWDebugSubConditions.debugPassive_VelocityOutOfRange =
      ((pGetEgoInfo->fegoVelocity_mps >
        params->RCWWarningParameter.fSuppressEgoLongVelMax) ||
       (pGetEgoInfo->fegoVelocity_mps <
        params->RCWWarningParameter.fSuppressEgoLongVelMin));
  debugInfo->Debug_RCWDebugSubConditions.debugHmiOff_bRCWHmiOpen =
      pGetRCWStateConditions->bRCWHmiOpen;
  debugInfo->Debug_RCWDebugSubConditions.debugFailure_bRCWFailureCondition =
      pGetRCWStateConditions->bRCWFailureCondition;
}

#endif  // ST_PERCEPTION

void LBSRCWStateConditionProcess(
    RCWStatusCondition_t* pGetRCWStateConditions,
    const RCWVehicleInfo_t* pGetEgoInfo,
    const RCWPreProcessInput_t* pGetPreProcessInput, const RCWParam_st* params,
    RCWWarningInfo_t* pRCWCalcWarningInfo,
    const RCWStationaryBlocked_t* pRCWStationaryBlocker,
    RCWDebug_t* debugInfo) {
  const boolean tBlockTimerActive =
      pGetRCWCalculatePointer()->RCWBlockingTimeActive;

  // hmi open
  if (pGetPreProcessInput->RCWHmiOpen == TRUE) {
    pGetRCWStateConditions->bRCWHmiOpen = TRUE;
  } else {
    pGetRCWStateConditions->bRCWHmiOpen = FALSE;
  }

  // failure
  if (pGetPreProcessInput->RCWFailure == TRUE) {
    pGetRCWStateConditions->bRCWFailureCondition = TRUE;
  } else {
    pGetRCWStateConditions->bRCWFailureCondition = FALSE;
  }

  // suppression
  if ((pGetEgoInfo->fegoVelocity_mps >
       params->RCWWarningParameter.fSuppressEgoLongVelMax)  // vx > 140kph
      || (pGetEgoInfo->fegoVelocity_mps <
          params->RCWWarningParameter.fSuppressEgoLongVelMin)  // vx < 0kph
      || (fABS(pGetEgoInfo->fLatAccel_mps2) >
          params->RCWWarningParameter.fSuppressEgoLatAcceMax)  // ay > ---
      || (fABS(pGetEgoInfo->fLatVelocity_mps) >
          params->RCWWarningParameter.fSuppressEgoLatVelMax)  // vy > ---
      || (pGetPreProcessInput->LeftTurnLightOpen == TRUE)     // left turn light
      || (pGetPreProcessInput->RightTurnLightOpen == TRUE)  // right turn light
      || (1 == pGetEgoInfo->uRCWGearPosition)  // R gear suppression RCW
      || (DEG2RAD(20.0) <=
          fABS(pGetEgoInfo->fRCWEgo_steer_angle_rad))  // R gear suppression RCW
      || (tBlockTimerActive == TRUE)                   // blocking timer
  ) {
    pGetRCWStateConditions->bRCWPassiveCondition = TRUE;
  } else {
    pGetRCWStateConditions->bRCWPassiveCondition = FALSE;
  }

  // active condition
  if ((pRCWCalcWarningInfo->bRCWWarningActive == TRUE) &&
      (pRCWStationaryBlocker->bCorridorBlocked == FALSE)) {
    pGetRCWStateConditions->bRCWActiveConditon = TRUE;
  }

  debugInfo->Debug_RCWDebugSubConditions.debugActive_bRCWWarningActive =
      pRCWCalcWarningInfo->bRCWWarningActive;
  debugInfo->Debug_RCWDebugSubConditions.debugActive_bCorridorBlocked =
      pRCWStationaryBlocker->bCorridorBlocked;
  debugInfo->Debug_RCWDebugSubConditions.debugPassive_BlockingtimeActive =
      tBlockTimerActive;
  debugInfo->Debug_RCWDebugSubConditions.debugPassive_GearPosition =
      (1 == pGetEgoInfo->uRCWGearPosition);
  debugInfo->Debug_RCWDebugSubConditions.debugPassive_LatAcceloutofRange =
      (fABS(pGetEgoInfo->fLatAccel_mps2) >
       params->RCWWarningParameter.fSuppressEgoLatAcceMax);
  debugInfo->Debug_RCWDebugSubConditions.debugPassive_LeftTurnLight =
      (pGetPreProcessInput->LeftTurnLightOpen == TRUE);
  debugInfo->Debug_RCWDebugSubConditions.debugPassive_RightTurnLight =
      (pGetPreProcessInput->RightTurnLightOpen == TRUE);
  debugInfo->Debug_RCWDebugSubConditions.debugPassive_VelocityOutOfRange =
      ((pGetEgoInfo->fegoVelocity_mps >
        params->RCWWarningParameter.fSuppressEgoLongVelMax) ||
       (pGetEgoInfo->fegoVelocity_mps <
        params->RCWWarningParameter.fSuppressEgoLongVelMin));
  debugInfo->Debug_RCWDebugSubConditions.debugHmiOff_bRCWHmiOpen =
      pGetRCWStateConditions->bRCWHmiOpen;
  debugInfo->Debug_RCWDebugSubConditions.debugFailure_bRCWFailureCondition =
      pGetRCWStateConditions->bRCWFailureCondition;
}

/*****************************************************************************
  Functionname: LBSRCWStateMachineProcess                                  */ /*!

  @brief:state machine process

  @description:state machine process

  @param[in]:pGetRCWstatemachine, pGetRCWstatemachineLastCycle, pGetRCWStateConditions, debugInfo

  @return:void
*****************************************************************************/
void LBSRCWStateMachineProcess(
    RCWStateMachine_t* pGetRCWstatemachine,
    const RCWStatusCondition_t* pGetRCWStateConditions, RCWDebug_t* debugInfo) {
  RCWStateMachine_t* pGetRCWstatemachineLastCycle =
      pGetRCWCalculatePointer_RCWstatemachineLastCycle();

  // store last cycle status
  *pGetRCWstatemachineLastCycle = *pGetRCWstatemachine;

  switch (*pGetRCWstatemachine) {
    case RCWState_Init:
    case RCWState_Failure:
    case RCWState_Off:
      if (pGetRCWStateConditions->bRCWHmiOpen == FALSE)  // priority 1
      {
        *pGetRCWstatemachine = RCWState_Off;
      } else if (pGetRCWStateConditions->bRCWFailureCondition == TRUE) {
        *pGetRCWstatemachine = RCWState_Failure;
      } else {
        *pGetRCWstatemachine = RCWState_StandBy;
      }
      break;

    case RCWState_StandBy:
    case RCWState_Active:
      if (pGetRCWStateConditions->bRCWHmiOpen == FALSE) {
        *pGetRCWstatemachine = RCWState_Off;
      } else if (pGetRCWStateConditions->bRCWFailureCondition == TRUE) {
        *pGetRCWstatemachine = RCWState_Failure;
      } else if (pGetRCWStateConditions->bRCWPassiveCondition == TRUE) {
        *pGetRCWstatemachine = RCWState_passive;
      } else if (pGetRCWStateConditions->bRCWActiveConditon == TRUE) {
        *pGetRCWstatemachine = RCWState_Active;
      } else {
        *pGetRCWstatemachine = RCWState_StandBy;
      }
      break;

    case RCWState_passive:
      if (pGetRCWStateConditions->bRCWHmiOpen == FALSE) {
        *pGetRCWstatemachine = RCWState_Off;
      } else if (pGetRCWStateConditions->bRCWFailureCondition == TRUE) {
        *pGetRCWstatemachine = RCWState_Failure;
      } else if (pGetRCWStateConditions->bRCWPassiveCondition == TRUE) {
        *pGetRCWstatemachine = RCWState_passive;
      } else {
        *pGetRCWstatemachine = RCWState_StandBy;
      }
      break;

    default:
      *pGetRCWstatemachine = RCWState_Init;
      break;
  }

  // store debug signal
  debugInfo->Debug_RCWstatemachine = *pGetRCWstatemachine;
}

/*****************************************************************************
  Functionname: LBSRCWHmiOutput                                  */ /*!

  @brief:HMI output

  @description:HMI output

  @param[in]:pGetRCWstatemachine, proPorts, pRCWCalcWarningInfo, params

  @return:void
*****************************************************************************/
void LBSRCWHmiOutput(const RCWStateMachine_t* pGetRCWstatemachine,
                     RCWOutPro_st* proPorts,
                     const RCWWarningInfo_t* pRCWCalcWarningInfo,
                     const RCWParam_st* params) {
  proPorts->rcw_fTTC_s = pRCWCalcWarningInfo->fTTC;
  const double kMinTTCThreshold = 1e-5;
  if ((*pGetRCWstatemachine != RCWState_Off)
      /*&&  (*pGetRCWstatemachine != RCWState_Failure)*/) {
    proPorts->bHmiRCWHmiOn = TRUE;
  } else {
    proPorts->bHmiRCWHmiOn = FALSE;
  }

  proPorts->uHmiRCWWarningActive = 0U;
  proPorts->uHmiRCWWarningID = TUE_C_UI8_VALUE_INVALID;
  proPorts->fHmiXObjectWarning = -TUE_C_F32_VALUE_INVALID;
  if (*pGetRCWstatemachine == RCWState_Active &&
      kMinTTCThreshold < pRCWCalcWarningInfo->fTTC) {
    proPorts->uHmiRCWWarningID = pRCWCalcWarningInfo->uRCWWarningID;
    proPorts->fHmiXObjectWarning = pRCWCalcWarningInfo->fXObjectWarning;

    if (pRCWCalcWarningInfo->fTTC <
        params->RCWWarningParameter.fTTCThresholdLevel2) {
      proPorts->uHmiRCWWarningActive = 2U;
    }
  }

  if (*pGetRCWstatemachine == RCWState_Failure) {
    proPorts->bHmiRCWFailure = TRUE;
  } else {
    proPorts->bHmiRCWFailure = FALSE;
  }
}

/*****************************************************************************
  Functionname: LBSRCWBlockingTimeProcess                                  */ /*!

  @brief:blocking timer process

  @description:blocking timer process

  @param[in]:pGetRCWstatemachine, pGetRCWstatemachineLastCycle, pGetPreProcessInput, params

  @return:void
*****************************************************************************/
void LBSRCWBlockingTimeProcess(const RCWStateMachine_t* pGetRCWstatemachine,
                               const RCWParam_st* params) {
  boolean RCWBlocktimerActive = FALSE;
  static float32 fBlockTimerRemainValue = 0.F;
  RCWStateMachine_t* pGetRCWstatemachineLastCycle =
      pGetRCWCalculatePointer_RCWstatemachineLastCycle();

  // if(     (pRCWCalcWarningInfo->bRCWWarningActive == FALSE)
  //     &&  (pRCWCalcWarningInfo->bRCWWarningActiveLastCycle == TRUE) )
  if ((*pGetRCWstatemachineLastCycle == RCWState_Active) &&
      ((*pGetRCWstatemachine == RCWState_StandBy) ||
       (*pGetRCWstatemachine == RCWState_Off) ||
       (*pGetRCWstatemachine == RCWState_passive) ||
       (*pGetRCWstatemachine == RCWState_Failure))) {
    RCWBlocktimerActive = TRUE;
  }

  pGetRCWCalculatePointer()->RCWBlockingTimeActive = TUE_CML_TimerRetrigger(
      params->RCWWarningParameter.fCycletime_s, RCWBlocktimerActive,
      params->RCWWarningParameter.fSuppressRCWBlockingTime,
      &fBlockTimerRemainValue);
  // printf("RCWBlocktimerActive is %d\tfBlockTimerRemainValue is %f\t",
  //        pGetPreProcessInput->BlockingTimeActive, fBlockTimerRemainValue);
}
