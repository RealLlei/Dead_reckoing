/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "lbs_bsd_calculation.h"
#include "lbs_bsd_par.h"

/*****************************************************************************
  VARIABLES
*****************************************************************************/

/*****************************************************************************
  MAIN FUNCTION
*****************************************************************************/

/*****************************************************************************
  Functionname: pGetBSDCalculatePointer                                  */ /*!

  @brief

  @description

  @param[in]

  @return
*****************************************************************************/
BSDCalculate_st* pGetBSDCalculatePointer() {
  return &BSDCalculate;
}

void InitBSDCalculate() {
  memset(&(BSDCalculate), 0, sizeof(BSDCalculate_st));

  // return &BSDCalculate;
}

BSD_Globals_t* pGetBSDCalculatePointer_BSDGlobals() {
  return &BSDCalculate.BSD_Globals;
}

BSDGlobal_BSDZone_st* pGetBSDCalculatePointer_BSDGlobaZonePar() {
  // todo check obj number range
  return &BSDCalculate.BSDGlobalsZones;
}

BSDZone_ObjPar* pGetBSDCalculatePointer_BSDZoneObjPar(uint8 uObj) {
  // todo check obj number range
  return &BSDCalculate.BSDZoneObjParList[uObj];
}

BSD_Info_t* pGetBSDCalculatePointer_ObjInfo(uint8 uObj) {
  // todo check obj number range
  return &BSDCalculate.BSDObjInfoList[uObj];
}

const BSD_LBSObjInfo_st* pGetBSD_LBSObjInfoPointer_LBSObjInfo(
    uint8 uObj, const BSD_LBSInputInfo_st* pLBSInputInfo) {
  return &pLBSInputInfo->LBSObjInfoList[uObj];
}

const BSD_LCAObjInfo_t* pGetBSD_LBSObjInfoPointer_LCAObjInfo(
    uint8 uObj, const BSD_LBSInputInfo_st* pLBSInputInfo) {
  return &pLBSInputInfo->LCAObjInfoList[uObj];
}

const BSD_GenObject_st* pGetBSDGenObjListPointer_Object(
    uint8 uObj, const BSDGenObjList_st* pGenObjList) {
  // todo check obj number range
  return &pGenObjList->aObject[uObj];
}

const BSD_SRRObject_st* pGetBSDSRRObjListPointer_Object(
    uint8 uObj, const BSDSRRObjList_st* pSRRObjList) {
  return &pSRRObjList->aObject[uObj];
}

/*****************************************************************************
  CALCULATION FUNCTION
*****************************************************************************/
/*****************************************************************************
  Functionname: bGetObjIsDeleted                                  */ /*!

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
boolean bGetObjIsDeleted(uint8 uObj, const BSDGenObjList_st* pGenObjList) {
  // obj number check
  boolean bRet = FALSE;
  if (pGenObjList->aObject[uObj].General.uiMaintenanceState_nu ==
      BSD_EM_GEN_OBJECT_MT_STATE_DELETED) {
    bRet = TRUE;
  } else {
    bRet = FALSE;
  }
  return bRet;
}

/*****************************************************************************
  Functionname: bGetObjRightSensorFlag                                  */ /*!

  @brief

  @description

  @param[in]

  @return
*****************************************************************************/
boolean bGetObjRightSensorFlag(uint8 uObj,
                               const BSD_GenObject_st* pGenObjInfo) {
  // Obj number check

  return pGenObjInfo->bRightSensor;
}

/*****************************************************************************
  Functionname: BSDUpdateBSDWarningActiveFlag */ /*!

  @brief :Update current side BSD warning flag 

  @description :Update current side BSD warning flag by the object current side

  @param[in]:uObj,pGenObj

  @return:void
*****************************************************************************/
void BSDUpdateBSDWarningActiveFlag(uint8 uObj,
                                   const BSD_GenObject_st* pGenObj) {
  boolean bSensorRightFlag = bGetObjRightSensorFlag(uObj, pGenObj);
  BSD_Globals_t* pBSDGlobal = pGetBSDCalculatePointer_BSDGlobals();
  BSD_Info_t* pBSDObjInfo = pGetBSDCalculatePointer_ObjInfo(uObj);

  if (bSensorRightFlag == FALSE) {
    pBSDGlobal->bBSDWarnActive = pBSDGlobal->bBSDWarnActiveLeft;
    pBSDObjInfo->eObjDirection = LBS_BSD_LeftSensorObj;
  } else {
    pBSDGlobal->bBSDWarnActive = pBSDGlobal->bBSDWarnActiveRight;
    pBSDObjInfo->eObjDirection = LBS_BSD_RightSensorObj;
  }
}

/*****************************************************************************
  Functionname: BSDUpdateBSDZoneWithHyst                                  */ /*!

  @brief:Update BSDZone hysteresis parameter 

  @description:Update BSDZone hysteresis parameter by the object current side

  @param[in]:uObj,pGenObj,pBSDGlobal

  @return:void
*****************************************************************************/
void BSDUpdateBSDZoneWithHyst(uint8 uObj, const BSD_GenObject_st* pGenObj,
                              BSD_Globals_t* pBSDGlobal) {
  boolean bSensorRightFlag = bGetObjRightSensorFlag(uObj, pGenObj);
  BSDZoneParameter_t* pBSDZoneParameterLeft =
      &pGetBSDCalculatePointer_BSDGlobaZonePar()->BSDZoneParameterLeft;
  BSDZoneParameter_t* pBSDZoneParameterRight =
      &pGetBSDCalculatePointer_BSDGlobaZonePar()->BSDZoneParameterRight;

  if (bSensorRightFlag == FALSE) {
    pBSDGlobal->fBSDZoneXminWithHyst_met =
        pBSDZoneParameterLeft->fBSDZoneXminWithHyst_met;
    pBSDGlobal->fBSDZoneXmaxWithHyst_met =
        pBSDZoneParameterLeft->fBSDZoneXmaxWithHyst_met;
    pBSDGlobal->fBSDZoneYminWithHyst_met =
        pBSDZoneParameterLeft->fBSDZoneYminWithHyst_met;
    pBSDGlobal->fBSDZoneYmaxWithHyst_met =
        pBSDZoneParameterLeft->fBSDZoneYmaxWithHyst_met;
  } else {
    pBSDGlobal->fBSDZoneXminWithHyst_met =
        pBSDZoneParameterRight->fBSDZoneXminWithHyst_met;
    pBSDGlobal->fBSDZoneXmaxWithHyst_met =
        pBSDZoneParameterRight->fBSDZoneXmaxWithHyst_met;
    pBSDGlobal->fBSDZoneYminWithHyst_met =
        pBSDZoneParameterRight->fBSDZoneYminWithHyst_met;
    pBSDGlobal->fBSDZoneYmaxWithHyst_met =
        pBSDZoneParameterRight->fBSDZoneYmaxWithHyst_met;
  }
}

/*****************************************************************************
  Functionname: BSDUpdateGlobalZoneParameters */ /*!

  @brief :Update current BSD Zone parameter by object detect side

  @description :base on object detect side use difference Zone parameter

  @param[in]:Object Array Index: uObj,General Radar Object:pGenObj

  @return :void
*****************************************************************************/
void BSDUpdateGlobalZoneParameters(uint8 uObj,
                                   const BSD_GenObject_st* pGenObj) {
  BSD_Globals_t* pBSDGlobal = pGetBSDCalculatePointer_BSDGlobals();
  BSDZoneParameter_t* pBSDZonePar = NULL;
  boolean bSensorRightFlag = bGetObjRightSensorFlag(uObj, pGenObj);

  if (bSensorRightFlag == FALSE) {
    // Object in the left side,use the left side zone parameter
    pBSDZonePar =
        &(pGetBSDCalculatePointer_BSDGlobaZonePar()->BSDZoneParameterLeft);
  } else {
    // Object in the right side
    pBSDZonePar =
        &(pGetBSDCalculatePointer_BSDGlobaZonePar()->BSDZoneParameterRight);
  }

  // writer current side zone parameter to global
  pBSDGlobal->fBSDZoneXmin_met = pBSDZonePar->fBSDZoneXmin_met;
  pBSDGlobal->fBSDZoneXmax_met = pBSDZonePar->fBSDZoneXmax_met;
  pBSDGlobal->fBSDZoneYmin_met = pBSDZonePar->fBSDZoneYmin_met;
  pBSDGlobal->fBSDZoneYmax_met = pBSDZonePar->fBSDZoneYmax_met;
}

/*****************************************************************************
  Functionname: BSDCalculateAdaptedBSDZoneLength */ /*!

  @brief : Adapts the length of the BSD Zone according to the current curve radius

  @description : Adapts the length of the BSD Zone according to the current curve radius

  @param[in]: GenObj,EgoInfo,RoadInfo,SensorMounting

  @return: BSDZone min X coordinate ,fBSDZoneXmin
*****************************************************************************/
float32 BSDCalculateAdaptedBSDZoneLength(
    const BSD_LBSGlobalInfo_t* pLBSGlobalInput,
    const BSDVehicleInfo_t* pEgoInfo, const BSDRoad_t* pRoad,
    const BSDSensorMounting_t* pSenserMounting, float32 fBSDZoneXmin,
    float32 fCenter2Axis) {
  BSD_Globals_t* pBSDGlobal = pGetBSDCalculatePointer_BSDGlobals();
  float32 fEgoSpeedX = pEgoInfo->fegoVelocity_mps;
  const float32 fDrivenCurveRadius = fABS(pRoad->fDrivenCurveRadius_met);
  const float32 fCurveRadius = fABS(pRoad->fCurveRadius_met);
  float32 fABSRadius = TUE_CML_Min(fDrivenCurveRadius, fCurveRadius);
  float32 fReqLength = pBSDGlobal->fBSDZoneXminStatic_met;
  float32 fBSDZoneXminStatic = pBSDGlobal->fBSDZoneXminStatic_met;
  float32 fFilterSpeed = 0.F;
  float32 fRmax = 0.F;
  float32 fRmin = 0.F;
  float32 fXShort = 0.F;
  float32 fXLong = 0.F;

  // check we are in a curve,and calculate the X min of the zone
  // consider road curve to calculate new X min,curveRadius down and zone length
  // shorter
  if (fABSRadius < F32_BSDZONE_INNER_HIGHSPEED_EDGE_RADIUS) {
    // if driver curve direction is left
    if (pLBSGlobalInput->bInnerSensorDriven == TRUE) {
      // check if in the low speed curve situation
      if (fABSRadius < F32_BSDZONE_INNER_LOWSPEED_EDGE_RADIUS) {
        // low speed and inner driver curve situation
        fRmax = F32_BSDZONE_INNER_LOWSPEED_EDGE_RADIUS;
        fRmin = F32_BSDZONE_INNER_LOWSPEED_MIN_RADIUS;
        // fXShort = F32_BSDZONE_INNER_LOWSPEED_XMIN -
        // pSenserMounting->fLatPos_met; fXLong  =
        // F32_BSDZONE_INNER_HIGHSPEED_XMIN - pSenserMounting->fLatPos_met;
        fXShort = F32_BSDZONE_INNER_LOWSPEED_XMIN - fCenter2Axis;
        fXLong = F32_BSDZONE_INNER_HIGHSPEED_XMIN - fCenter2Axis;
      } else {
        // high speed and inner driver curve situation
        fRmax = F32_BSDZONE_INNER_HIGHSPEED_EDGE_RADIUS;
        fRmin = F32_BSDZONE_INNER_HIGHSPEED_MIN_RADIUS;
        // fXShort = F32_BSDZONE_INNER_HIGHSPEED_XMIN -
        // pSenserMounting->fLatPos_met;
        fXShort = F32_BSDZONE_INNER_HIGHSPEED_XMIN - fCenter2Axis;
        fXLong = fBSDZoneXminStatic;
      }
    } else  // if driver curve direction is right
    {
      // check if in the low speed curve situation
      if (fABSRadius < F32_BSDZONE_OUTER_LOWSPEED_EDGE_RADIUS) {
        // low speed and outer driver curve situation
        fRmax = F32_BSDZONE_OUTER_LOWSPEED_EDGE_RADIUS;
        fRmin = F32_BSDZONE_OUTER_LOWSPEED_MIN_RADIUS;
        // fXShort = F32_BSDZONE_OUTER_LOWSPEED_XMIN -
        // pSenserMounting->fLatPos_met; fXLong =
        // F32_BSDZONE_OUTER_HIGHSPEED_XMIN - pSenserMounting->fLatPos_met;
        fXShort = F32_BSDZONE_OUTER_LOWSPEED_XMIN - fCenter2Axis;
        fXLong = F32_BSDZONE_OUTER_HIGHSPEED_XMIN - fCenter2Axis;

      } else {
        // high speed and outer driver curve situation
        fRmax = F32_BSDZONE_OUTER_HIGHSPEED_EDGE_RADIUS;
        fRmin = F32_BSDZONE_OUTER_HIGHSPEED_MIN_RADIUS;
        // fXShort = F32_BSDZONE_OUTER_HIGHSPEED_XMIN -
        // pSenserMounting->fLatPos_met;
        fXShort = F32_BSDZONE_OUTER_HIGHSPEED_XMIN - fCenter2Axis;
        fXLong = fBSDZoneXminStatic;
      }
    }

    // base speed and road curve situation to interpolation  CurveRadius and
    // limit ReqLength (range fXShort -> fXLong)
    fReqLength =
        GDBmathLinFuncLimBounded(fABSRadius, fRmin, fRmax, fXShort, fXLong);
    //
  }

  // check request zone length increase or decrease from last cycle
  // request length is greater than the current length,means the BSD ZONE
  // shorter
  if (fReqLength > fBSDZoneXmin) {
    // base on egoSpeed to get the LowPassFilter shorter Alpha value
    fFilterSpeed = GDBmathLinFuncLimBounded(
        fEgoSpeedX, BSD_LI_ZONE_EGO_SPEED_MIN, BSD_LI_ZONE_EGO_SPEED_MAX,
        BSD_LI_ZONE_LPF_APLHA_MIN, BSD_LI_ZONE_LPF_APLHA_MAX_SHORTEN);

    // filter the BSD Zone length
    GDB_Math_LowPassFilter(&fBSDZoneXmin, fReqLength, fFilterSpeed);
  } else  // means the BSD zone is lengthen
  {
    // if the requested value and is sufficiently different to the
    // parameterized,lengthen is using a filter
    if ((fBSDZoneXmin - fBSDZoneXminStatic) >
        BSD_ZONE_DIST_THRESHOLD_LENGTHEN) {
      // base on egoSpeed to get the LowPassFilter lengthen Alpha value
      fFilterSpeed = GDBmathLinFuncLimBounded(
          fEgoSpeedX, BSD_LI_ZONE_EGO_SPEED_MIN, BSD_LI_ZONE_EGO_SPEED_MAX,
          BSD_LI_ZONE_LPF_APLHA_MIN, BSD_LI_ZONE_LPF_APLHA_MAX_LENGTHEN);

      // filter the BSD Zone length
      GDB_Math_LowPassFilter(&fBSDZoneXmin, fReqLength, fFilterSpeed);
    } else {
      // otherwise set it, to avoid filtering till eternity
      fBSDZoneXmin = fBSDZoneXminStatic;
    }
  }

  return fBSDZoneXmin;
}

/*****************************************************************************
  Functionname: BSDGetZoneParametersForCurrentObject */ /*!

  @brief: Get the correct warning zone parameter for each object

  @description: Get the correct warning zone parameter for each object

  @param[in]:pGenObj,pRoad,pBsdZonePar

  @return:void
*****************************************************************************/
void BSDGetZoneParametersForCurrentObject(uint8 uObj,
                                          const BSD_GenObject_st* pGenObj,
                                          const BSDRoad_t* pRoad,
                                          const BsdZone_t* pBsdZonePar) {
  BSD_Globals_t* pBSDGlobal = pGetBSDCalculatePointer_BSDGlobals();
  BSD_Info_t* pBSDObj = pGetBSDCalculatePointer_ObjInfo(uObj);
  BSDZone_ObjPar* pBSDObjZone = pGetBSDCalculatePointer_BSDZoneObjPar(uObj);

  // check whether use NCAP flag to update obj info Xmin
  // float32 fABSRadius = fABS(pRoad->fDrivenCurveRadius_met);
  // float32 fObjLongVrel = pGenObj->Kinemactic.fVrelX_mps;
  // pBSDObj->fBSDZoneObjXmin_met = -((0.11F * SQR(fObjLongVrel)) + (1.08F *
  // fObjLongVrel) + 2.97); pBSDObj->fBSDZoneObjXmin_met = -MIN((fABSRadius +
  // C_F32_DELTA), fABS(pBSDObj->fBSDZoneObjXmin_met));

  if (pBSDObj->bBSDWarning == TRUE) {
    // use hysteresis value if in the BSDWarning state(lengthen and widen than
    // base Zone parameter)
    pBSDObjZone->fZoneXmin_met = pBSDGlobal->fBSDZoneXminWithHyst_met;
    pBSDObjZone->fZoneXmax_met = pBSDGlobal->fBSDZoneXmaxWithHyst_met;
    pBSDObjZone->fZoneYmin_met = pBSDGlobal->fBSDZoneYminWithHyst_met;
    pBSDObjZone->fZoneYmax_met = pBSDGlobal->fBSDZoneYmaxWithHyst_met;

  } else {
    // use normal value if in the BSDWarning state
    pBSDObjZone->fZoneXmin_met = pBSDGlobal->fBSDZoneXmin_met;
    pBSDObjZone->fZoneXmax_met = pBSDGlobal->fBSDZoneXmax_met;
    pBSDObjZone->fZoneYmin_met = pBSDGlobal->fBSDZoneYmin_met;
    pBSDObjZone->fZoneYmax_met = pBSDGlobal->fBSDZoneYmax_met;
  }
}

/*****************************************************************************
  Functionname: BSDCalculateSectorCuts                                  */ /*!

  @brief:Calculate object angle compensation angle when ego turns

  @description:Calculate object angle compensation angle when ego turns

  @param[in]:EgoInfo,RoadInfo,VehParameter

  @return:void
*****************************************************************************/
void BSDCalculateSectorCuts(uint8 uObj, const BSD_GenObject_st* pGenObj,
                            const BSD_LBSGlobalInfo_t* pLBSGlobalInput,
                            const BSDVehicleInfo_t* pEgoInfo,
                            const BSDRoad_t* pRoad,
                            const BSDVehParameter_t* pBSDVehParameter) {
  BSD_Globals_t* pBSDGlobal = pGetBSDCalculatePointer_BSDGlobals();
  const float32 fDrivenCurveRadius = fABS(pRoad->fDrivenCurveRadius_met);
  const float32 fCurveRadius = fABS(pRoad->fCurveRadius_met);
  const float32 fAbsCurveRadius = TUE_CML_Min(fDrivenCurveRadius, fCurveRadius);
  const float32 fEgoSpeed = pEgoInfo->fegoVelocity_mps;
  const float32 fWheelBaseDistance = pBSDVehParameter->fWheelBase_met;
  float32 fAngleShift = 0.F;

  /***********************************************************************************************/
  /* Calculate object angle compensation angle when ego turns inner sensor */
  /***********************************************************************************************/
  boolean bInnerSensorSteering = pLBSGlobalInput->bInnerSensorSteering;
  boolean bInnerSensorDriven = pLBSGlobalInput->bInnerSensorDriven;
  if (bGetObjRightSensorFlag(uObj, pGenObj)) {
    bInnerSensorSteering = !bInnerSensorSteering;
    bInnerSensorDriven = !bInnerSensorDriven;
  }
  pBSDGlobal->bInnerSensorSteering = bInnerSensorSteering;
  pBSDGlobal->bInnerSensorDriven = bInnerSensorDriven;

  /*When the vehicle turns,it is facing the 90 clusters at a different angle
  The relationship should be roughly tan(shiftAngle) = dx / WheelBaseDistance
  where dx is the shift of the front wheels over the rear wheels
  It was calculated as dx = CurveRadius - SquareRoot(CurveRadius - WheelBase)
  Calculating this for several different WheelBase distances led to this
  approximated formula shiftAngle = 29.6 * WheelBase*/

  if ((pBSDGlobal->bInnerSensorSteering == TRUE) &&
      (fEgoSpeed > BSD_SECTORCUT_EGOSPEED_MIN) &&
      (fAbsCurveRadius > BSD_SECTORCUT_ABSCURVE_MIN) &&
      (fAbsCurveRadius < BSD_SECTORCUT_ABSCURVE_MAX)) {
    fAngleShift =
        (BSD_AVERAGE_WHEELBASE_FACTOR * fWheelBaseDistance) / fAbsCurveRadius;
  }
  pBSDGlobal->fAngleFrontSector_deg =
      BSD_APPEAR_FRONT_SECTOR_CUT - fAngleShift;  // 76
}

/*****************************************************************************
  Functionname: BSDClassifyAppreance                                  */ /*!

  @brief:Check whether the object appeared in Front,Rear or Side Region

  @description: According to object appeared angle to judge appear region direction

  @param[in]:uObj,pGenObj,pSRRObj,pLBSInputInfo

  @return:uiRetApperance -> Object appear direction
*****************************************************************************/
uint8 BSDClassifyAppreance(uint8 uObj, const BSD_GenObject_st* pGenObj,
                           const BSD_SRRObject_st* pSRRObj,
                           const BSD_LBSInputInfo_st* pLBSInputInfo,
                           const BSDVehParameter_t* pVehParameter) {
  BSD_Globals_t* pBSDGlobal = pGetBSDCalculatePointer_BSDGlobals();
  BSD_Info_t* pBSDObj = pGetBSDCalculatePointer_ObjInfo(uObj);

  const BSD_LBSObjInfo_st* pLBSObjInfo =
      pGetBSD_LBSObjInfoPointer_LBSObjInfo(uObj, pLBSInputInfo);

  float32 fAbsAngle = fABS(pLBSObjInfo->fAngle_deg);
  uint8 uiRetApperance = BSD_APPEAR_INVALID;
  float32 fHitSum = 0.F;
  float32 fHitRatioFront = 0.F;
  float32 fHitRatioRear = 0.F;
  uint8 HitGain = 1U;

  // compensation BSD parameter with AUTOSAR coordinate(add vehicle rear
  // position to front axis distance)
  const float32 fVehRear2FrontAxis_met = pVehParameter->fVehRear2FrontAxis_met;

  /***********************************************************************************************/
  /* Transform right side object degree to left side range */
  /***********************************************************************************************/
  if (bGetObjRightSensorFlag(uObj, pGenObj) == TRUE) {
    // right side degree range(+180 ~ +360) -> (+180 ~ 0)
    fAbsAngle = 360.0F - fAbsAngle;
  }
  /***********************************************************************************************/
  /* Object lifetime less than 0.3s */
  /***********************************************************************************************/

  if (pLBSObjInfo->fCycletimeSum_s <
      BSD_APPEARANCE_CHECK_MAX_LIFETIME) {  // 0.3
    // if object appearance angle less than 76 degree(-> 0~76),means object
    // appear from front when the vehicle turn this Front sector need to shift
    // small angle
    if (fAbsAngle < pBSDGlobal->fAngleFrontSector_deg) {
      // count up hits if object appear from front

      uiRetApperance = BSD_APPEAR_FRONT;  // HE_PERCEPERTION
      pBSDObj->ubHitsInFront_nu = pBSDObj->ubHitsInFront_nu + HitGain;
    } else if (fAbsAngle < BSD_APPEAR_REAR_SECTOR_CUT) {  // 104
      // appear from 76 ~ 104 degree
      uiRetApperance = BSD_APPEAR_SIDE;  // HE_PERCEPERTION
      pBSDObj->ubHitsInSide_nu = pBSDObj->ubHitsInSide_nu + HitGain;
    } else {
      // appear from 104 ~ 180 degree
      uiRetApperance = BSD_APPEAR_REAR;  // HE_PERCEPERTION
      pBSDObj->ubHitsInRear_nu = pBSDObj->ubHitsInRear_nu + HitGain;
    }

  } else {
    // // ST_PERCEPERTION
    // // check for possibly wrapped objects in the rear that may cause FA
    // if ((pLBSObjInfo->fCycletimeSum_s < BSD_WRAPPEROBJ_LIFETIME_MIN) &&  //
    // 0.5
    //     (pBSDObj->bPossibleWrappedObj == FALSE)) {
    //   if ((pSRRObj->History.fFirstDetectX_met <
    //        (BSD_WRAPPEROBJ_FIRSTDETX_MIN - fVehRear2FrontAxis_met)) &&
    //       (pSRRObj->History.fFirstDetectX_met >
    //        (BSD_WRAPPEROBJ_FIRSTDETX_MAX - fVehRear2FrontAxis_met)) &&
    //       (fAbsAngle > BSD_WRAPPEROBJ_ANGLE_MIN)) {
    //   pBSDObj->bPossibleWrappedObj = TRUE;
    // } else if ((pSRRObj->History.fFirstDetectX_met <
    //             (BSD_NEW_WRAPPEROBJ_MIN_FIRSTDETX -
    //              fVehRear2FrontAxis_met)) &&
    //            (pSRRObj->History.fFirstDetectX_met >
    //             (BSD_NEW_WRAPPEROBJ_MAX_FIRSTDETX -
    //              fVehRear2FrontAxis_met)) &&
    //            (fAbsAngle > BSD_NEW_WRAPPEROBJ_ANGLE_MIN))

    // ----  HZ_PERCEPERTION
    // check for possibly wrapped objects in the rear that may cause FA
    if ((pLBSObjInfo->fCycletimeSum_s < BSD_WRAPPEROBJ_LIFETIME_MIN) &&  // 0.5
        (pBSDObj->bPossibleWrappedObj == FALSE)) {
      if ((pSRRObj->History.fFirstDetectX_met < BSD_WRAPPEROBJ_FIRSTDETX_MIN) &&
          (pSRRObj->History.fFirstDetectX_met > BSD_WRAPPEROBJ_FIRSTDETX_MAX) &&
          (fAbsAngle > BSD_WRAPPEROBJ_ANGLE_MIN)) {
        pBSDObj->bPossibleWrappedObj = TRUE;
      } else if ((pSRRObj->History.fFirstDetectX_met <
                  BSD_NEW_WRAPPEROBJ_MIN_FIRSTDETX) &&
                 (pSRRObj->History.fFirstDetectX_met >
                  BSD_NEW_WRAPPEROBJ_MAX_FIRSTDETX) &&
                 (fAbsAngle > BSD_NEW_WRAPPEROBJ_ANGLE_MIN)) {
        // check for possibly wrapped objects that are created behind the
        // subject vehicle on the functional non relevant side
        pBSDObj->bPossibleWrappedObj = TRUE;
      } else {
        // nothing to do
      }
    }
  }

  /***********************************************************************************************/
  /* Object lifetime between 0.2 - 0.3s */
  /* track is old enough to calculate the appearance based on the ratio of hits
   * in front and rear*/
  /***********************************************************************************************/

  // ----  ST_PERCEPERTION
  // if ((pLBSObjInfo->fCycletimeSum_s > BSD_APPEARANCE_CHECK_MIN_LIFETIME) &&
  //     (pLBSObjInfo->fCycletimeSum_s < BSD_APPEARANCE_CHECK_MAX_LIFETIME)) {
  //   // calculate common denominator for the ratio
  //   fHitSum = (float32)pBSDObj->ubHitsInFront_nu +
  //             (float32)pBSDObj->ubHitsInSide_nu +
  //             (float32)pBSDObj->ubHitsInRear_nu;

  //   // be careful for divisions by zeros
  //   fHitSum = MAX(1.0F, fHitSum);

  //   // calculate front and rear sector ratio
  //   fHitRatioFront = (float32)pBSDObj->ubHitsInFront_nu / fHitSum;
  //   fHitRatioRear = (float32)pBSDObj->ubHitsInRear_nu / fHitSum;

  //   if (fHitRatioFront > BSD_APPEAR_FRONT_RATIO_CUT) {
  //     // set appearance front
  //     uiRetApperance = BSD_APPEAR_FRONT;
  //   } else if (fHitRatioRear > BSD_APPEAR_REAR_RATIO_CUT) {
  //     // set appearance rear
  //     uiRetApperance = BSD_APPEAR_REAR;
  //   } else {
  //     // set appearance side
  //     uiRetApperance = BSD_APPEAR_SIDE;
  //   }
  // }

  /***********************************************************************************************/
  /* Object lifetime more than 0.3s */
  /***********************************************************************************************/
  if (pLBSObjInfo->fCycletimeSum_s >= BSD_APPEARANCE_CHECK_MAX_LIFETIME) {
    uiRetApperance = pBSDObj->ubAppearance_nu;
  }

  return uiRetApperance;
}

// --------------------  2022 02 11
/*****************************************************************************
  Functionname: BSDCheckObjectInBSDZone                                  */ /*!

  @brief:Check whether the object in the BSDZone 

  @description:Check whether the object in the BSDZone,consider straight and curve road

  @param[in]:

  @return:void
*****************************************************************************/
boolean BSDCheckObjectInBSDZone(uint8 uObj,
                                const BSD_LBSInputInfo_st* pLBSInputInfo,
                                const BSDVehicleInfo_t* pEgoInfo,
                                const BSDRoad_t* pRoad) {
  BSD_Globals_t* pBSDGlobal = pGetBSDCalculatePointer_BSDGlobals();
  BSD_Info_t* pBSDObj = pGetBSDCalculatePointer_ObjInfo(uObj);
  BSDZone_ObjPar* pBSDObjZone = pGetBSDCalculatePointer_BSDZoneObjPar(uObj);

  float32 fZoneXMin = pBSDObjZone->fZoneXmin_met;
  float32 fZoneXMax = pBSDObjZone->fZoneXmax_met;
  float32 fZoneYMin = pBSDObjZone->fZoneYmin_met;
  float32 fZoneYMax = pBSDObjZone->fZoneYmax_met;

  float32 fDrivenCurveRadius = fABS(pRoad->fDrivenCurveRadius_met);
  float32 fCurveRadius = fABS(pRoad->fCurveRadius_met);
  float32 fABSCurveRadius = TUE_CML_Min(fDrivenCurveRadius, fCurveRadius);
  boolean bObjInZone = FALSE;

  // Trivial case: not in a narrow curve,check rectangular boundaries
  if (fABSCurveRadius > F32_BSDZONE_CURVEADAPTION_RADIUS) {
    bObjInZone = BSDCalculateObjectInRectZone(
        uObj, fZoneXMin, fZoneXMax, fZoneYMin, fZoneYMax, pLBSInputInfo);
  } else {
    // Non trivial case, we are in a narrow curve,means we have to check if the
    // object is in the curved BSD Zone
    bObjInZone = BSDCalculateObjectInCurvedZone(
        uObj, fZoneXMin, fZoneXMax, fZoneYMin, fZoneYMax, pLBSInputInfo, pRoad);
  }
  return bObjInZone;
}

/*****************************************************************************
  Functionname: BSDCheckObjectInSOTZone                                  */ /*!

  @brief: Check of Object is with its boundaries inside the SOT-Zone

  @description: Check of Object is with its boundaries inside the SOT-Zone

  @param[in]: 

  @return:void
*****************************************************************************/
boolean BSDCheckObjectInSOTZone(uint8 uObj, const BSD_GenObject_st* pGenObj,
                                const BSD_LBSInputInfo_st* pLBSInputInfo,
                                const BSDRoad_t* pRoad) {
  boolean bInSOTZone = FALSE;
  BSD_Globals_t* pBSDGlobal = pGetBSDCalculatePointer_BSDGlobals();
  BSD_Info_t* pBSDObj = pGetBSDCalculatePointer_ObjInfo(uObj);
  float32 fZoneXMin = 0.F;
  float32 fZoneXMax = 0.F;
  float32 fZoneYMin = 0.F;
  float32 fZoneYMax = 0.F;

  /* Expand the SOT Zone to the guardrail */
  if (bGetObjRightSensorFlag(uObj, pGenObj) == FALSE) {
    // If left sensor find object, Ymax of SOTZone is EMRoad.fYOffsetFused_met
    // which is limited [3.5 10] and Ymin of SOTZone is 0.5.
    fZoneYMax = TUE_CML_MinMax(pBSDGlobal->fBSDZoneYmax_met,
                               BSD_SOT_ZONE_YMAX_MAX, pRoad->fYOffsetFused_met);
    fZoneYMin = pBSDGlobal->fBSDZoneYmin_met;
  } else {
    // If right sensor find object, Ymax of SOTZone is -0.5 and Ymin of SOTZone
    // is EMRoad.fYOffsetFused_met which is limited [-3.5 -10]
    fZoneYMax = pBSDGlobal->fBSDZoneYmax_met;
    fZoneYMin =
        TUE_CML_MinMax(-BSD_SOT_ZONE_YMAX_MAX, pBSDGlobal->fBSDZoneYmin_met,
                       -1 * pRoad->fYOffsetFused_met);
  }
  // Xmax of SOTZone is 1 and Xmin of SOTZone is -7.--ST
  // HZ: 95% ~ -3m ;behind the host car
  fZoneXMax = pBSDGlobal->fBSDZoneXmax_met;
  fZoneXMin = pBSDGlobal->fBSDZoneXmin_met;

  // Determine whether the object is in SOTZone.
  bInSOTZone = BSDCalculateObjectInRectZone(
      uObj, fZoneXMin, fZoneXMax, fZoneYMin, fZoneYMax, pLBSInputInfo);

  /* Copy the values into the BSD_Info struct */
  pBSDObj->bInSOTZonePrevious = pBSDObj->bInSOTZone;
  // pBSDObj->bInSOTZone = bInSOTZone;
  return bInSOTZone;
}

/*****************************************************************************
  Functionname: BSDCheckObjectZoneOverlap                                  */ /*!

  @brief:Calculate the overlap of an object and the zone(works only on the rectangular zone case)

  @description:Calculate the overlap of an object and the zone(works only on the rectangular zone case)

  @param[in]:uObj,GenObj,LBSInputInfo,RoadInfo

  @return:bRet
*****************************************************************************/
boolean BSDCheckObjectZoneOverlap(uint8 uObj, const BSD_GenObject_st* pGenObj,
                                  const BSD_LBSInputInfo_st* pLBSInputInfo,
                                  const BSDRoad_t* pRoad) {
  BSD_Info_t* pBSDObj = pGetBSDCalculatePointer_ObjInfo(uObj);
  const BSDObjBorders_t* pObjBorders =
      &pGetBSD_LBSObjInfoPointer_LBSObjInfo(uObj, pLBSInputInfo)->ObjBorders;
  BSDZone_ObjPar* pBSDZonePar = pGetBSDCalculatePointer_BSDZoneObjPar(uObj);
  const BSD_LCAObjInfo_t* pLCAObjInfo =
      pGetBSD_LBSObjInfoPointer_LCAObjInfo(uObj, pLBSInputInfo);
  boolean bRightSensorFlag = bGetObjRightSensorFlag(uObj, pGenObj);
  boolean bRet = FALSE;

  if (pBSDObj->bInBSDZone == TRUE) {
    /* If the outer border of the object is in the zone
       or the overlap is sufficient
       or there are less than 2 lanes
       or the LCA warning is active */

    if (bRightSensorFlag == FALSE) {
      if ((pObjBorders->fYmax_met < pBSDZonePar->fZoneYmax_met) ||
          (pBSDZonePar->fZoneYmax_met - pObjBorders->fYmin_met >
           BSD_MIN_ZONE_OVERLAP_MULTILANE) ||
          (pRoad->iNumOfAdjacentLanes_nu < BSD_OBJOVERLAP_ADJLANES_MAX) ||
          (pLCAObjInfo->bLCAWarning == TRUE)) {
        /* set overlap to true */
        bRet = TRUE;
      }

    } else {
      if ((pObjBorders->fYmin_met > pBSDZonePar->fZoneYmin_met) ||
          (-pBSDZonePar->fZoneYmin_met + pObjBorders->fYmax_met >
           BSD_MIN_ZONE_OVERLAP_MULTILANE) ||
          (pRoad->iNumOfAdjacentLanes_nu < BSD_OBJOVERLAP_ADJLANES_MAX) ||
          (pLCAObjInfo->bLCAWarning == TRUE)) {
        /* set overlap to true */
        bRet = TRUE;
      }
    }
  }
  // pBSDObj->bObjectAndZoneOverlap = bRet;
  return bRet;
}

/*****************************************************************************
  Functionname: BSDCalculateUpdateGrdCounter                                  */ /*!

  @brief:Update counter if Object is inside or behind the estimated guardrail

  @description:Update counter if Object is inside or behind the estimated guardrail

  @param[in]:uObj,pGenObj,pSRRObj,pLBSInputInfo,pEgoInfo,pRoad,pSensorMounting

  @return:void
*****************************************************************************/
void BSDCalculateUpdateGrdCounter(uint8 uObj, const BSD_GenObject_st* pGenObj,
                                  const BSD_SRRObject_st* pSRRObj,
                                  const BSD_LBSInputInfo_st* pLBSInputInfo,
                                  const BSDVehicleInfo_t* pEgoInfo,
                                  const BSDRoad_t* pRoad,
                                  const BSDSensorMounting_t* pSensorMounting) {
  const BSD_LBSObjInfo_st* pLBSObj =
      pGetBSD_LBSObjInfoPointer_LBSObjInfo(uObj, pLBSInputInfo);
  const BSD_LCAObjInfo_t* pLCAObj =
      pGetBSD_LBSObjInfoPointer_LCAObjInfo(uObj, pLBSInputInfo);
  BSD_Globals_t* pBSDGlobal = pGetBSDCalculatePointer_BSDGlobals();
  BSD_Info_t* pBSDObj = pGetBSDCalculatePointer_ObjInfo(uObj);

  float32 fGrdDist2Course = F32_VALUE_INVALID;
  boolean bGrdDistFound = FALSE;
  boolean bBehindGrd = FALSE;
  boolean bGrdHit = FALSE;
  boolean bGrdCondFulfilled = FALSE;

  const float32 fTTC = pLBSObj->fTTC_s;
  const float32 fObjDistX = pGenObj->Kinemactic.fDistX_met;
  sint8 iIncLCA = 0;
  sint8 iIncrement = 0;
  float32 fIncTTC = 0.F;

  /**************************************************************************************
   *Check whether guardrail condition,and update  bGrdDistFound, bBehindGrd
   *,bGrdHit flags
   **************************************************************************************/
  bGrdCondFulfilled =
      BSDCalculateCheckGrdPreCondition(uObj, pGenObj, pSRRObj, pRoad);

  if (bGrdCondFulfilled == TRUE) {
    // Calculate the distance of the object and the guardrail
    BSDCalculateGrdDist2Course(uObj, pGenObj, pSRRObj, pEgoInfo, pRoad,
                               pSensorMounting, &fGrdDist2Course,
                               &bGrdDistFound);
    // Check whether the object position is on or behind the guardrail
    BSDCalculateObjectGrdrailRelation(uObj, pGenObj, pSRRObj, fGrdDist2Course,
                                      bGrdDistFound, &bBehindGrd, &bGrdHit);
  }

  /***************************************************************************************
   *Check if the counters have been set, if not set them to decrease now *
   ****************************************************************************************/
  // Condition for counting down faster
  if (pLCAObj->bLCAWarning == TRUE) {
    iIncLCA = BSD_GRD_INCREMENT_LCA_WARNING;
  }

  // Calculate TTC based decrement
  // But only use the TTC if the object is really behind the subject
  if ((fObjDistX > BSD_GRD_INCREMENT_TTC_DISTX_MAX) || (bBehindGrd == TRUE) ||
      (bGrdHit == TRUE)) {
    fIncTTC = 0.0F;
  } else {
    fIncTTC = GDBmathLinFuncLimBounded(
        fTTC, BSD_LI_GRD_TTC_MIN, BSD_LI_GRD_TTC_MAX, BSD_LI_GRD_TTC_INC_MIN,
        BSD_LI_GRD_TTC_INC_MAX);
    fIncTTC = ROUND(fIncTTC);
  }

  // Update the GRD hit counter
  if (bGrdHit == FALSE) {
    // base to decrease hit counter,consider LCA warning and TTC
    iIncrement = -1 + iIncLCA + (sint8)fIncTTC;
  } else {
    // base to increase hit counter,consider LCA warning and TTC
    iIncrement = 1 + iIncLCA + (sint8)fIncTTC;
  }

  // Update guardrail hit counter base on LCA,TTC and current GrdHit
  BSDCalculateUpdateAndLimitCounter(&pBSDObj->ubGrdHitCounter_nu, iIncrement,
                                    0U, BSD_GRD_HIT_COUNTER_MAX);

  if (bBehindGrd == FALSE) {
    // This is counter was not increased -> decrease it now,same as GrdHit
    // counter update
    iIncrement = -1 + iIncLCA + (sint8)fIncTTC;
  } else {
    // base to increase behindGrd counter,consider LCA warning and TTC
    iIncrement = 1 + iIncLCA + (sint8)fIncTTC;
  }
  // Update behind guardrail hit counter base on LCA,TTC and current GrdHit
  BSDCalculateUpdateAndLimitCounter(&pBSDObj->ubBehindGrdCounter_nu, iIncrement,
                                    0U, BSD_BEHIND_GRD_COUNTER_MAX);

  // check reset ID
  // if (iGrdTrkResetID == (sint8)uObj)
  //{
  //}
}

/*****************************************************************************
  Functionname: BSDCheckObjectBehindGRD                                  */ /*!

  @brief:Check if Object is behind a guardrail

  @description:Check if Object is behind a guardrail

  @param[in]:uObj,pGenObj,pLBSInputInfo

  @return:bBehindGRD,the object behind guardrail flag
*****************************************************************************/
boolean BSDCheckObjectBehindGRD(uint8 uObj, const BSD_GenObject_st* pGenObj,
                                const BSD_LBSInputInfo_st* pLBSInputInfo) {
  BSD_Info_t* pBSDObj = pGetBSDCalculatePointer_ObjInfo(uObj);
  uint16 uiLifeCycles = pGenObj->General.uiLifeCycles_nu;
  boolean bBehindGRD = FALSE;

  if (pBSDObj->bObjectBehindGRD == TRUE) {
    /* Behind guardrail already detected, check keep active condition */
    if (pBSDObj->ubBehindGrdCounter_nu > 0U) {
      bBehindGRD = TRUE;
    }
  } else {
    /* Behind guardrail not detected, check activation condition */
    if ((pBSDObj->ubBehindGrdCounter_nu > BSD_BEHIND_GRD_COUNT_WARN_OFF_HYST) ||
        ((pBSDObj->ubBehindGrdCounter_nu >
          BSD_BEHIND_GRD_COUNT_LTIME_WARN_OFF_HYST) &&
         ((uint16)pBSDObj->ubBehindGrdCounter_nu == uiLifeCycles))) {
      bBehindGRD = TRUE;
    }
  }
  // pBSDObj->bObjectBehindGRD = bBehindGRD;
  return bBehindGRD;
}

/*****************************************************************************
  Functionname: BSDCheckObjectBehindGRD                                  */ /*!

  @brief:Think Object is not behind a guardrail after the filter of perception

  @description:Check if Object is behind a guardrail

  @param[in]:uObj,pGenObj,pLBSInputInfo

  @return:bBehindGRD,the object behind guardrail flag
*****************************************************************************/
boolean BSDCheckObjectBehindGRD_hz(uint8 uObj, const BSD_GenObject_st* pGenObj,
                                   const BSD_LBSInputInfo_st* pLBSInputInfo) {
  BSD_Info_t* pBSDObj = pGetBSDCalculatePointer_ObjInfo(uObj);
  uint16 uiLifeCycles = pGenObj->General.uiLifeCycles_nu;
  boolean bBehindGRD = FALSE;

  return bBehindGRD;
}

/*****************************************************************************
  Functionname: BSDCalculateUpdateOwnlaneCounter */ /*!

  @brief: Calculates the ownlane counter of an object

  @description: Calculates the ownlane counter of an object

  @param[in]:uObj,pGenObj,pSRRObj,pLBSInputInfo,pRoad

  @return:void
*****************************************************************************/
void BSDCalculateUpdateOwnlaneCounter(uint8 uObj,
                                      const BSD_GenObject_st* pGenObj,
                                      const BSD_SRRObject_st* pSRRObj,
                                      const BSD_LBSInputInfo_st* pLBSInputInfo,
                                      const BSDRoad_t* pRoad) {
  const BSD_LCAObjInfo_t* pLCAObj =
      pGetBSD_LBSObjInfoPointer_LCAObjInfo(uObj, pLBSInputInfo);
  const BSD_LBSObjInfo_st* pLBSObj =
      pGetBSD_LBSObjInfoPointer_LBSObjInfo(uObj, pLBSInputInfo);
  BSD_Info_t* pBSDObj = pGetBSDCalculatePointer_ObjInfo(uObj);
  float32 fInnerEdgeDist2Course = 0.F;
  float32 fOuterEdgeDist2Course = 0.F;
  float32 fOwnLanePart = 0.F;
  float32 fAdjLanePart = 0.F;
  float32 fLaneWidth = pRoad->fLaneWidth_met;
  sint8 iPosInc = -1;
  sint8 iLCAInc = 0;
  sint8 iTTCInc = 0;
  float32 fIncTemp = 0.F;

  if (bGetObjRightSensorFlag(uObj, pGenObj) == FALSE) {
    /*If left sensor find object, the distance between the inner edge of the
      object and the trajectory of the own vehicle = the distance between the
      object and the trajectory of the own vehicle - right width of the
      object.*/
    // fDist2Course_met left  side sign: +
    // fDist2Course_met right side sign: -
    fInnerEdgeDist2Course = pSRRObj->RoadRelation.fDist2Course_met -
                            pGenObj->Geometry.fWidthRight_met;
    fOuterEdgeDist2Course = pSRRObj->RoadRelation.fDist2Course_met +
                            pGenObj->Geometry.fWidthLeft_met;
  } else {
    // Similarly.
    // fDist2Course_met is always positive value. And fWidthRight_met is always
    // positive value.
    fInnerEdgeDist2Course = pSRRObj->RoadRelation.fDist2Course_met +
                            pGenObj->Geometry.fWidthLeft_met;
    fOuterEdgeDist2Course = pSRRObj->RoadRelation.fDist2Course_met -
                            pGenObj->Geometry.fWidthRight_met;
  }
  /****************************************************************************/
  /* Calculate position based increment                                       */
  /****************************************************************************/
  if (bGetObjRightSensorFlag(uObj, pGenObj) == FALSE) {
    /* Calculate the part of the object which overlaps the own lane */
    fOwnLanePart = TUE_CML_Abs(
        TUE_CML_Min(0.F, fInnerEdgeDist2Course - (0.5F * fLaneWidth)));
    /* Calculate the part of the object which overlaps the adjacent lane */
    fAdjLanePart = TUE_CML_Abs(
        TUE_CML_Max(0.F, fOuterEdgeDist2Course - (0.5F * fLaneWidth)));
  } else {
    fOwnLanePart = TUE_CML_Abs(
        TUE_CML_Min(0.F, -(0.5F * fLaneWidth) - fInnerEdgeDist2Course));
    fAdjLanePart = TUE_CML_Abs(
        TUE_CML_Max(0.F, -(0.5F * fLaneWidth) - fOuterEdgeDist2Course));
  }
  /* The object has to be at least 1 m on the own lane.
     Or the main part of the object has to be on the own lane */
  if ((fOwnLanePart > BSD_OWNL_OVERLAP_METERS) ||  //1 -> 1.5
      (fOwnLanePart > fAdjLanePart)) {
    iPosInc = BSD_OWNL_INC_OVERLAP;
  }
  /****************************************************************************/
  /* Calculate LCA-warning based increment                                    */
  /****************************************************************************/
  if (pLCAObj->bLCAWarning == TRUE) {
    iLCAInc = BSD_OWNL_INC_LCA_WARNING;
  }
  /****************************************************************************/
  /* Calculate TTC-based increment                                            */
  /****************************************************************************/
  fIncTemp = GDBmathLinFuncLimBounded(
      pLBSObj->fTTCFiltered_s, BSD_LI_OWNL_TTC_FILT_MIN,
      BSD_LI_OWNL_TTC_FILT_MAX, BSD_LI_OWNL_TTC_INC_MIN,
      BSD_LI_OWNL_TTC_INC_MAX);

  iTTCInc = (sint8)ROUND_TO_INT(fIncTemp);
  /****************************************************************************/
  /* Calculate the new value and limit it                                     */
  /****************************************************************************/
  BSDCalculateUpdateAndLimitCounter(&pBSDObj->ubOwnLaneCounter_nu,
                                    (iPosInc + iLCAInc + iTTCInc), 0U,
                                    BSD_OWNLANE_COUNTER_MAX);
}

/*****************************************************************************
  Functionname: BSDCheckObjectOnOwnlane                                  */ /*!

  @brief

  @description

  @param[in]

  @return
*****************************************************************************/
boolean BSDCheckObjectOnOwnlane(uint8 uObj, const BSD_GenObject_st* pGenObj) {
  BSD_Info_t* pBSDObj = pGetBSDCalculatePointer_ObjInfo(uObj);
  uint16 uLifeCycle = pGenObj->General.uiLifeCycles_nu;
  boolean bObjOnOwnLane = FALSE;
  float32 ftemp = 0.F;

  if (pBSDObj->bObjectOnOwnlane == TRUE) {
    /* Own lane flag is set, check keep active condition */
    if (pBSDObj->ubOwnLaneCounter_nu > 0U) {
      bObjOnOwnLane = TRUE;
    }
  } else {
    /* Own lane not detected, check activation condition */
    if (pBSDObj->ubOwnLaneCounter_nu >= BSD_OWNLANE_CNT_THRESH) {
      ftemp = (float32)pBSDObj->ubOwnLaneCounter_nu / SafeDiv(uLifeCycle);
    }
    if ((pBSDObj->ubOwnLaneCounter_nu > BSD_OWNLANE_COUNT_WARN_OFF_HYST) ||
        ftemp >= BSD_OWNLANE_CNT_FACTOR) {
      bObjOnOwnLane = TRUE;
    }
  }
  // pBSDObj->bObjectOnOwnlane = bObjOnOwnLane;
  return bObjOnOwnLane;
}

/*****************************************************************************
  Functionname: BSDCheckObjectQuality                                  */ /*!

  @brief:Check if object quality is high enough

  @description:Check if object quality is high enough

  @param[in]:uObj,pGenObj,pSRRObj,pLBSInputInfo,pEgoInfo

  @return:bObjQuality,the object is high enough flag
*****************************************************************************/
boolean BSDCheckObjectQuality(uint8 uObj, const BSD_GenObject_st* pGenObj,
                              const BSD_SRRObject_st* pSRRObj,
                              const BSD_LBSInputInfo_st* pLBSInputInfo,
                              const BSDVehicleInfo_t* pEgoInfo) {
  BSD_Info_t* pBSDObj = pGetBSDCalculatePointer_ObjInfo(uObj);
  BSD_Globals_t* pBSDGlobal = pGetBSDCalculatePointer_BSDGlobals();
  float32 fMinAssocProb = 0.F;
  float32 fMinUpdateRate = 0.F;
  boolean bMaiten = FALSE;

  if (pGenObj->General.uiMaintenanceState_nu == EM_GEN_OBJ_MT_STATE_MEASURED) {
    bMaiten = TRUE;
  }

  const float32 fVxOverGround =
      pGenObj->Kinemactic.fVrelX_mps + pEgoInfo->fegoVelocity_mps;
  const BSD_LBSObjInfo_st* pLBSObj =
      pGetBSD_LBSObjInfoPointer_LBSObjInfo(uObj, pLBSInputInfo);

  boolean bObjQuality = FALSE;

  if (pBSDObj->ubAppearance_nu == BSD_APPEAR_FRONT) {
    fMinAssocProb = pBSDGlobal->fMinAssocProbFront_nu;
  } else {
    fMinAssocProb = pBSDGlobal->fMinAssocProbSideRear_nu;
  }
  /* For possibly wrapped objects check for a larger filtered association
   * probability */
  if (pBSDObj->bPossibleWrappedObj == TRUE) {
    fMinAssocProb = BSD_WRAPPEDOBJ_ASSOC_PROB_MIN;
  }

  /* Determine the minimum requested update rate by the object speed over ground
   */
  fMinUpdateRate = GDBmathLinFuncLimBounded(
      fVxOverGround, BSD_LI_UPDRATE_MIN_VXOVERGROUND,
      BSD_LI_UPDRATE_MAX_VXOVERGROUND, BSD_LI_UPDRATE_MIN, BSD_LI_UPDRATE_MAX);

#if ST_PERCEPTION

  if ((pLBSObj->fUpdateRate_nu > fMinUpdateRate) &&
      (pLBSObj->fAssocProbFiltered > fMinAssocProb) &&
      pSRRObj->Qualifiers.fProbabilityOfExistence_per > BSD_QUALITY_POE_MIN) {
    bObjQuality = TRUE;
  }
#endif

  //#if HZ_PERCEPTION

  if ((TRUE == bMaiten) &&
      (pSRRObj->Qualifiers.fProbabilityOfExistence_per > BSD_QUALITY_POE_MIN)) {
    bObjQuality = TRUE;
  }
  //#endif
  // pBSDObj->bQualityEnough = bObjQuality;
  return bObjQuality;
}

/*****************************************************************************
  Functionname: BSDCheckObjectLivedLongEnough */ /*!

  @brief: Check if Object lived long enough

  @description:Check if Object lived long enough

  @param[in]:uObj,pLBSInputInfo

  @return:bLiveLong,the Object lived long enough flag
*****************************************************************************/
boolean BSDCheckObjectLivedLongEnough(
    uint8 uObj, const BSD_LBSInputInfo_st* pLBSInputInfo) {
  BSD_Info_t* pBSDObj = pGetBSDCalculatePointer_ObjInfo(uObj);
  const BSD_LBSObjInfo_st* pLBSObj =
      pGetBSD_LBSObjInfoPointer_LBSObjInfo(uObj, pLBSInputInfo);
  boolean bLiveLong = FALSE;

  if ((pBSDObj->bLivedLongEnough == FALSE) &&
      (pLBSObj->fCycletimeSum_s > BSD_OBJ_LIVED_ENOUGH_LIFETIME_MIN)) {
    if (pBSDObj->bCreateBehindGRD == FALSE) {
      // pBSDObj->bLivedLongEnough = TRUE;
      bLiveLong = TRUE;
    } else {
      /* If the object has been created behind a confirmed guardrail the minimum
       * lifetime of the object must exceed the */
      /* the counter for deciding if it is classified as a non relevant behind
       * the guardrail object */
      if (pLBSObj->fCycletimeSum_s >
          (2.0F * BSD_OBJ_LIVED_ENOUGH_LIFETIME_MIN)) {
        // pBSDObj->bLivedLongEnough = TRUE;
        bLiveLong = TRUE;
      }
    }
  }

  if (pBSDObj->bLivedLongEnough == TRUE) {
    bLiveLong = pBSDObj->bLivedLongEnough;
  }
  return bLiveLong;
}

/*****************************************************************************
  Functionname: BSDCheckObjectLivedLongEnough */ /*!

  @brief: Check if Object lived long enough

  @description:Check if Object lived long enough

  @param[in]:uObj,pLBSInputInfo

  @return:bLiveLong,the Object lived long enough flag
*****************************************************************************/
boolean BSDCheckObjectLivedLongEnough_hz(
    uint8 uObj, const BSD_LBSInputInfo_st* pLBSInputInfo) {
  BSD_Info_t* pBSDObj = pGetBSDCalculatePointer_ObjInfo(uObj);
  const BSD_LBSObjInfo_st* pLBSObj =
      pGetBSD_LBSObjInfoPointer_LBSObjInfo(uObj, pLBSInputInfo);
  boolean bLiveLong = FALSE;

  if (pBSDObj->bLivedLongEnough == TRUE) {
    bLiveLong = pBSDObj->bLivedLongEnough;
  }

  if ((pBSDObj->bLivedLongEnough == FALSE) &&
      (pLBSObj->fCycletimeSum_s > BSD_OBJ_LIVED_ENOUGH_LIFETIME_MIN)) {
    bLiveLong = TRUE;
  }
  return bLiveLong;
}

/*****************************************************************************
  Functionname: BSDCheckObjectUpdatedRecently */ /*!

  @brief:Check if the object was updated in last few(2) cycles
   
  @description:Check if the object was updated in last few(2) cycles

  @param[in]:uObj,pSRRObj

  @return:void
*****************************************************************************/
void BSDCheckObjectUpdatedRecently(uint8 uObj,
                                   const BSD_SRRObject_st* pSRRObj) {
  uint8 ubMeasuredTargetFrequency =
      pSRRObj->Qualifiers.uiMeasuredTargetFrequency_nu;
  BSD_Info_t* pBSDObj = pGetBSDCalculatePointer_ObjInfo(uObj);
  boolean bUpdatedRecently = FALSE;
  boolean bUpdatedRecentlyWeak = FALSE;

  if ((ubMeasuredTargetFrequency & UI_192_TO_BINARY) == UI_192_TO_BINARY) {
    bUpdatedRecently = TRUE;
    bUpdatedRecentlyWeak = TRUE;
  }
  /* Check the weak update condition (1 out of the two last cycle has to have an
   * update) */
  if (((ubMeasuredTargetFrequency & UI_128_TO_BINARY) == UI_128_TO_BINARY) ||
      ((ubMeasuredTargetFrequency & UI_64_TO_BINARY) == UI_64_TO_BINARY)) {
    bUpdatedRecentlyWeak = TRUE;
  }
  pBSDObj->bUpdatedRecently = bUpdatedRecently;
  pBSDObj->bUpdatedRecentlyWeak = bUpdatedRecentlyWeak;
}

/*****************************************************************************
  Functionname: BSDClassifyObject                                  */ /*!

  @brief:Classify Object based on appearance and kinematic

  @description:Classify Object based on appearance and kinematic

  @param[in]:uObj,pGenObj,pSRRObj,pLBSInputInfo,pEgoInfo,pRoad

  @return:void
*****************************************************************************/
void BSDClassifyObject(uint8 uObj, const BSD_GenObject_st* pGenObj,
                       const BSD_SRRObject_st* pSRRObj,
                       const BSD_LBSInputInfo_st* pLBSInputInfo,
                       const BSDVehicleInfo_t* pEgoInfo, const BSDRoad_t* pRoad,
                       const BSDVehParameter_t* pVehPar) {
  const BSD_LBSObjInfo_st* pLBSObj =
      pGetBSD_LBSObjInfoPointer_LBSObjInfo(uObj, pLBSInputInfo);
  BSD_Info_t* pBSDObj = pGetBSDCalculatePointer_ObjInfo(uObj);

  if ((pLBSObj->fCycletimeSum_s >= BSD_CLASSIFY_MIN_LIFETIME) &&     // 0.3
      (pBSDObj->ubClass_nu <= BSD_CLASS_STATIC_LAST_RECOVERABLE) &&  // 180
      (pBSDObj->ubAppearance_nu != BSD_APPEAR_INVALID)) {            // 0
    if (pBSDObj->ubAppearance_nu == BSD_APPEAR_FRONT) {              // 1
      if (pBSDObj->bInBSDZone == TRUE) {
        BSDClassifyFrontObject(uObj, pGenObj, pSRRObj, pLBSInputInfo, pEgoInfo,
                               pRoad, pVehPar);
      }
    } else {
      if (pBSDObj->ubAppearance_nu == BSD_APPEAR_REAR) {  // 2
        BSDClassifyRearObject(uObj);
      } else {
        BSDClassifySideObject(uObj, pGenObj, pLBSInputInfo, pEgoInfo, pRoad,
                              pVehPar);
      }
    }
  }

  // Reclassify objects which are observed in the rear
  BSDClassifyConfirmedObjectAtRear(uObj, pGenObj, pSRRObj, pLBSInputInfo,
                                   pEgoInfo, pVehPar);
}

/*****************************************************************************
  Functionname: BSDClassifyGRD                                  */ /*!

  @brief :Update counter if Object is inside the estimated Guardrail

  @description:Update counter if Object is inside the estimated Guardrail

  @param[in]: uObj,pGenObj, pSRRObj, pLBSInputInfo,pEgoInfo,pRoad

  @return:void
*****************************************************************************/
void BSDClassifyGRD(uint8 uObj, const BSD_GenObject_st* pGenObj,
                    const BSD_SRRObject_st* pSRRObj,
                    const BSD_LBSInputInfo_st* pLBSInputInfo,
                    const BSDVehicleInfo_t* pEgoInfo, const BSDRoad_t* pRoad) {
  const BSD_LBSObjInfo_st* pLBSObj =
      pGetBSD_LBSObjInfoPointer_LBSObjInfo(uObj, pLBSInputInfo);
  BSD_Globals_t* pBSDGlobal = pGetBSDCalculatePointer_BSDGlobals();
  BSD_Info_t* pBSDObj = pGetBSDCalculatePointer_ObjInfo(uObj);

  float32 fObjAbsSpeedX =
      fABS(pEgoInfo->fegoVelocity_mps + pGenObj->Kinemactic.fVrelX_mps);
  float32 fYOffsetBorder = pRoad->fYOffsetFused_met;
  float32 fConYOffsetBorder = pRoad->fConfYOffset_per;
  float32 fObjDist2Border = pSRRObj->RoadRelation.fDist2Border_met;
  boolean bObjDist2BorderValid = pSRRObj->RoadRelation.bDist2BorderValid;
  boolean bSensorRightFlag = pGenObj->bRightSensor;
  float32 fYOuterBSDZone = 0.F;

  if ((pBSDObj->ubClass_nu != BSD_CLASS_STATIC_GRDHITCOUNTER) &&
      (pBSDObj->ubGrdHitCounter_nu >=
       BSD_CLASSIFY_GRD_COUNTER_MIN)) {  // alway 0
    pBSDObj->ubClass_nu = BSD_CLASS_STATIC_GRDHITCOUNTER;
  } else {
    // reverse Y Offset sign for right sensor
    if (bSensorRightFlag == TRUE) {
      fYOffsetBorder = -fYOffsetBorder;
      fObjDist2Border = -fObjDist2Border;
      fYOuterBSDZone = pBSDGlobal->fBSDZoneYmin_met;
    } else {
      fYOuterBSDZone = pBSDGlobal->fBSDZoneYmax_met;
    }

    // Verify the static classification by checking the current movement of the
    // object and it is position with respect to the confirmed road border If
    // these check indicate a real moving object the class is set to undefined
    if ((fObjAbsSpeedX > pBSDGlobal->fVxThreshMovStat_mps) &&
        (fYOffsetBorder > fYOuterBSDZone) &&
        (pLBSObj->ObjBorders.fXmin_met <
         BSD_CLASSIFY_GRD_XMIN_MAX))  //&&  // -1.5
    //(fConYOffsetBorder > BSD_CLASSIFY_GRD_BORDER_CONF_MIN) &&       // 0.8
    //(bObjDist2BorderValid == TRUE) &&
    //(fObjDist2Border < BSD_CLASSIFY_GRD_DIST2BORD_MAX)) // -1  ----
    // HZ_PERCEPTION
    {
      pBSDObj->ubClass_nu = BSD_CLASS_UNDEFINED;
    }
  }
}

/*****************************************************************************
  Functionname: BSDCheckObjectIsRelevant                                  */ /*!

  @brief:Mark relevant objects

  @description:check object if relevant objects

  @param[in]:uObj,pGenObj

  @return:bRelevance:relevance flag of the current object
*****************************************************************************/
boolean BSDCheckObjectIsRelevant(uint8 uObj, const BSD_GenObject_st* pGenObj,
                                 const BSDVehParameter_t* pVehPar) {
  BSD_Info_t* pBSDObject = pGetBSDCalculatePointer_ObjInfo(uObj);
  const float32 fXObj = pGenObj->Kinemactic.fDistX_met;
  const float32 fAxObj = pGenObj->Kinemactic.fArelX_mpss;
  boolean bRelevance = FALSE;

  // compensation BSD parameter with AUTOSAR coordinate(add vehicle rear
  // position to front axis distance)
  const float32 fBSD_OBJREL_DISTX_MAX =
      BSD_OBJREL_DISTX_MAX - pVehPar->fVehRear2FrontAxis_met;

  /* Only moving object should be relevant.
     The guardrail track should never be relevant */
  if (pGenObj->Attributes.eDynamicProperty_nu ==
      EM_GEN_OBJECT_DYN_PROPERTY_MOVING) {
    /* If the object is already warning (in this case the object was already
     * relevant) or the object has a relevant class.. */
    if (((pBSDObject->bBSDWarning == TRUE) &&
         (pBSDObject->ubClass_nu != BSD_CLASS_STATIC_GRDHITCOUNTER)) ||
        ((pBSDObject->ubClass_nu >= BSD_CLASS_VEH_FRONT) &&
         (pBSDObject->ubClass_nu <= BSD_CLASS_VEH_LAST))) {
      /* ..set relevance */
      if ((fXObj < fBSD_OBJREL_DISTX_MAX) || (fAxObj < BSD_OBJREL_ARELX_MAX) ||
          (pBSDObject->bBSDWarning == TRUE)) {
        bRelevance = TRUE;
      }
    }
  }
  return bRelevance;
}

/*****************************************************************************
  Functionname: BSDCheckObjectSoT                                  */ /*!

  @brief:Check Object if SOT Object

  @description:Find Objects which are SOTs and mark them

  @param[in]:uObj,pGenObj,pLBSInputInfo,pBSDWarnParameter

  @return:bIsSOT,the SOT flag of the current object
*****************************************************************************/
boolean BSDCheckObjectSoT(uint8 uObj, const BSD_GenObject_st* pGenObj,
                          const BSD_LBSInputInfo_st* pLBSInputInfo,
                          const BSDWarningParameter_t* pBSDWarnParameter,
                          const BSDVehParameter_t* pVehPar) {
  BSD_Info_t* pBSDObject = pGetBSDCalculatePointer_ObjInfo(uObj);
  // compensation distance X to sensor coordinate
  const float32 fDistXAbs =
      fABS(pGenObj->Kinemactic.fDistX_met + pVehPar->fVehRear2FrontAxis_met);
  const float32 fVxObj = pGenObj->Kinemactic.fVrelX_mps;
  boolean bIsSOT = FALSE;
  const BSD_LBSObjInfo_st* pLBSObj =
      pGetBSD_LBSObjInfoPointer_LBSObjInfo(uObj, pLBSInputInfo);
  float32 fSOTCutOffSpeed = -fABS(pBSDWarnParameter->fSoTCutoffSpeed_mps);
  boolean bSensorRightFlag = bGetObjRightSensorFlag(uObj, pGenObj);
  boolean bIsSOTInBSDZone = FALSE;

#ifdef ST_PECEPERTION
  // compensation BSD parameter with AUTOSAR coordinate(add vehicle rear
  // position to front axis distance)
  const float32 fBSD_MAX_X_SOT_CLOSE =
      BSD_MAX_X_SOT_CLOSE - pVehPar->fVehRear2FrontAxis_met;
  const float32 fBSD_MAX_X_SOT =
      BSD_MAX_X_SOT - pVehPar->fVehRear2FrontAxis_met;
#endif

  // HE_PERCEPERTION no need compensation BSD parameter with AUTOSAR
  // coordinate(add vehicle rear
  // position to front axis distance)
  float32 fBSD_MAX_X_SOT_CLOSE = BSD_MAX_X_SOT_CLOSE;
  float32 fBSD_MAX_X_SOT = BSD_MAX_X_SOT;

  if (pBSDObject->bIsSoT == TRUE) {
    // If we have once decided that this object is a SOT, we keep this as long
    // as the object is near 90 or while Vx is negative;
    if ((fDistXAbs < BSD_SOT_ABS_DISTX_MAX) || (fVxObj < -4.0F)) {
      bIsSOT = TRUE;
    }
  } else {
    // For objects which entered the zone in this cycle */

    // Check all objects in zone for SOT properties, but use stricter conditions
    // as for object which just entered the zone

    if ((fVxObj < (BSD_MIN_VX_NONSOT - BSD_SOT_VXCUT_HYST)) ||  // -5
        (fVxObj < (fSOTCutOffSpeed - BSD_SOT_VXCUT_HYST))) {    // -6
      bIsSOT = TRUE;
    } else {
      // if the object enters anywhere with a large negative Vx(-5m/s) it is an
      // SOT
      if ((fVxObj < LBS_INPUT_SOT_CUTOFF_SPEED)) {  //-5 m/s
        bIsSOT = TRUE;
      } else {
        if ((pBSDObject->bInSOTZonePrevious == FALSE) &&
            (pBSDObject->bInSOTZone == TRUE)) {
          // reverse check direction for right sensor
          if (bSensorRightFlag == FALSE) {
            if ((pLBSObj->ObjBorders.fXmax_met > fBSD_MAX_X_SOT_CLOSE) &&
                (pLBSObj->ObjBorders.fYmin_met < BSD_MAX_Y_SOT_CLOSE)) {
              bIsSOTInBSDZone = TRUE;
            }
          } else {
            if ((pLBSObj->ObjBorders.fXmax_met > fBSD_MAX_X_SOT_CLOSE) &&
                (pLBSObj->ObjBorders.fYmax_met > -BSD_MAX_Y_SOT_CLOSE)) {
              bIsSOTInBSDZone = TRUE;
            }
          }

          /* if the object overlaps the front part of zone it is an SOT and Vx
           * is directed backwards */
          if (pLBSObj->ObjBorders.fXmax_met > fBSD_MAX_X_SOT) {
            // bIsSOT = TRUE;
          } else {
            /* Object which appear in the zone, should be handled as SOT */
            if ((pLBSObj->fCycletimeSum_s < BSD_MAX_CYCLETIME_SOT_CLOSE) &&
                (bIsSOTInBSDZone == TRUE)) {
              bIsSOT = TRUE;
            }
          }
        }
      }
    }
  }
  return bIsSOT;
}

/*****************************************************************************
  Functionname: BSDCheckObjectFastSoT                                  */ /*!

  @brief: Check object for Fast SOT condition
    
  @description:Find objects which are FastSOTs and mark them

  @param[in]:uObj,pGenObj,pBSDWarnParameter,pBSDSystemParam

  @return:bIsFastSOT,FastSoT flag of the current object
*****************************************************************************/
boolean BSDCheckObjectFastSoT(uint8 uObj, const BSD_GenObject_st* pGenObj,
                              const BSDWarningParameter_t* pBSDWarnParameter,
                              const BSDSystemParam_t* pBSDSystemParam) {
  const float32 fSOTDelay = pBSDWarnParameter->fSoTDelayThresh_s;
  BSD_Info_t* pBSDObj = pGetBSDCalculatePointer_ObjInfo(uObj);
  BSD_Globals_t* pBSDGlobals = pGetBSDCalculatePointer_BSDGlobals();
  float32 fSOTCutOffSpeed = -fABS(pBSDWarnParameter->fSoTCutoffSpeed_mps);
  const BSD_GenObjKinematics_t* pEMObjKin = &pGenObj->Kinemactic;
  float32 fAx = 0.F;
  float32 fVxPredicted = 0.F;
  const float32 fVxObj = pGenObj->Kinemactic.fVrelX_mps;
  boolean bIsFastSOT = FALSE;

  /* Check initial conditions for a FastSOT classification.
     - SOT-Delay is active. Only in this case a FastSOT handling is possible
     - The object is a SOT
     - If a warning is currently active so it's not possible that there is a
     FastSOT */
  if ((fSOTDelay > C_F32_DELTA) && (pBSDObj->bIsSoT == TRUE) &&
      ((pBSDGlobals->bBSDWarnActive == FALSE))) {
    if (pBSDObj->bFastSoT == TRUE) {
      /* The object is already detected as fast SOT. Check the keep flag active
       * conditions */
      /* Add a hysteresis on the cut off speed. During a running SOT delay the
       * condition should be more tight. */
      if (pBSDObj->bSoTDelayActive == TRUE) {
        fSOTCutOffSpeed += BSD_FSOT_VXCUT_HYST_ACTIVEDELAY;
      } else {
        fSOTCutOffSpeed += BSD_FSOT_VXCUT_HYST_INACTIVEDELAY;
      }
    }
    /* Get x_acceleration of the object and limit it reasonable */
    fAx = TUE_CML_MinMax(BSD_FASTSOT_ARELX_LIMIT_MIN,
                         BSD_FASTSOT_ARELX_LIMIT_MAX, pEMObjKin->fArelX_mpss);
    /* Calculate the predicted Vx */
    fVxPredicted = pEMObjKin->fVrelX_mps + (BSD_SOT_VX_PRED_CYCLES * fAx *
                                            pBSDSystemParam->fCycletime_s);
    /* Check whether the object matches the Vx condition.
       Check whether there exists a rear object. If we found one only set this
       object as FastSOT if the rear object is not warning */
    if ((fVxObj < fSOTCutOffSpeed) || (fVxPredicted < fSOTCutOffSpeed)) {
      bIsFastSOT = TRUE;
    }
  }
  return bIsFastSOT;
}

/*****************************************************************************
  Functionname: BSDCheckObjectSoTDelay                                  */ /*!

  @brief:Check SOT delay

  @description:Check conditions for a SOT delay of the current object

  @param[in]:uObj,pBSDWarnParameter

  @return:bSOTDelay,SOT delay flag of the current object
*****************************************************************************/
boolean BSDCheckObjectSoTDelay(uint8 uObj,
                               const BSDWarningParameter_t* pBSDWarnParameter) {
  BSD_Info_t* pBSDObj = pGetBSDCalculatePointer_ObjInfo(uObj);
  boolean bSOTDelay = FALSE;
  /* Check condition for SOT delay:
     - SOTDelay has to be switched on
     - The SOTDelay should not be exceeded
     - The object should be detected as SOT
     - The warning has to be inactive ( To avoid toggeling of the warning)
     - The object has to be in the SOT zone or the delay is already active */
  if ((C_F32_DELTA < pBSDWarnParameter->fSoTDelayThresh_s) &&
      (pBSDObj->fSoTDelayTime_s < pBSDWarnParameter->fSoTDelayThresh_s) &&
      (pBSDObj->bIsSoT == TRUE) && (pBSDObj->bBSDWarning == FALSE) &&
      ((pBSDObj->bInSOTZone == TRUE) || (pBSDObj->bSoTDelayActive == TRUE))) {
    bSOTDelay = TRUE;
  }
  return bSOTDelay;
}

/*****************************************************************************
  Functionname: BSDCalculateUpdateObjectSoTDelay */ /*!

  @brief:Calculate the new SOTDelay

  @description:Update the SOT delay for the current object

  @param[in]:uObj,pGenObj,pSRRObj,pLBSInputInfo,pBSDSystemParam

  @return:void
*****************************************************************************/
void BSDCalculateUpdateObjectSoTDelay(uint8 uObj,
                                      const BSD_GenObject_st* pGenObj,
                                      const BSD_SRRObject_st* pSRRObj,
                                      const BSD_LBSInputInfo_st* pLBSInputInfo,
                                      const BSDSystemParam_t* pBSDSystemParam,
                                      const BSDVehParameter_t* pVehPar) {
  BSD_Info_t* pBSDObj = pGetBSDCalculatePointer_ObjInfo(uObj);
  BSD_Globals_t* pBSDGlobals = pGetBSDCalculatePointer_BSDGlobals();

  // compensation BSD parameter with AUTOSAR coordinate(add vehicle rear
  // position to front axis distance)
  const float32 fBSD_SOTZONE_FIRSTDETECTX_MIN =
      BSD_SOTZONE_FIRSTDETECTX_MIN - pVehPar->fVehRear2FrontAxis_met;

  if (pBSDObj->bSoTDelayActive == TRUE) {
    /* Add the cycle time */
    pBSDObj->fSoTDelayTime_s += pBSDSystemParam->fCycletime_s;

    /* For objects which appear in the zone, calculator the theoretic time since
     * it entered the SOT_Zone */
    if ((pGenObj->General.uiLifeCycles_nu == 1U) &&
        (pSRRObj->History.fFirstDetectX_met < fBSD_SOTZONE_FIRSTDETECTX_MIN) &&
        (pGenObj->Kinemactic.fVrelX_mps < -C_F32_DELTA)) {
      pBSDObj->fSoTDelayTime_s =
          TUE_CML_Min(((pSRRObj->History.fFirstDetectX_met -
                        pBSDGlobals->fBSDZoneXmax_met) /
                       pGenObj->Kinemactic.fVrelX_mps),
                      1.F);
    }
  } else {
    /* Check whether the object entered the SOT Zone this cycle. If yes reset
     * the SOTDelay. */
    if ((pBSDObj->bInSOTZone == TRUE) &&
        (pBSDObj->bInSOTZonePrevious == FALSE)) {
      pBSDObj->fSoTDelayTime_s = 0.F;
    }
  }
}

/*****************************************************************************
  Functionname: BSDUpdateScenarioObserver                                  */ /*!

  @brief:Update the scenario observer

  @description:Update SOT scenario observer

  @param[in]:uObj,pGenObj

  @return:void
*****************************************************************************/
void BSDUpdateScenarioObserver(uint8 uObj, const BSD_GenObject_st* pGenObj) {
  BSD_Info_t* pBSDObjInfo = pGetBSDCalculatePointer_ObjInfo(uObj);
  uint16 uiLifeCycles = pGenObj->General.uiLifeCycles_nu;
  BSD_Globals_t* pBSDGlobals = pGetBSDCalculatePointer_BSDGlobals();

  if ((pBSDObjInfo->bIsSoT == TRUE) && (pBSDObjInfo->bInBSDZone == TRUE) &&
      (uiLifeCycles > BSD_SOT_OBSERVER_LIFETIME_MIN)) {
    pBSDGlobals->ScenarioObserver.uNumberSoTObjs_nu += 1;
  }
}

/*****************************************************************************
  Functionname: BSDCheckObjectPlausibility                                  */ /*!

  @brief:Check whether the object movement is plausibility

  @description:Check whether the object movement indicates a valid object

  @param[in]:uObj, pGenObj,pSRRObj,pLBSInputInfo, pEgoInfo

  @return:bObjPlausibility,plausibility quality enough flag of the object
*****************************************************************************/
boolean BSDCheckObjectPlausibility(uint8 uObj, const BSD_GenObject_st* pGenObj,
                                   const BSD_SRRObject_st* pSRRObj,
                                   const BSD_LBSInputInfo_st* pLBSInputInfo,
                                   const BSDVehicleInfo_t* pEgoInfo,
                                   const BSDVehParameter_t* pVehPar) {
  BSD_Globals_t* pBSDGlobals = pGetBSDCalculatePointer_BSDGlobals();
  const BSD_LBSObjInfo_st* pLBSObj =
      pGetBSD_LBSObjInfoPointer_LBSObjInfo(uObj, pLBSInputInfo);
  const float32 fXMoved = pLBSObj->fXMovement_met;
  BSD_Info_t* pBSDObj = pGetBSDCalculatePointer_ObjInfo(uObj);
  boolean bObjPlausibility = FALSE;
  const float32 fEgoSpeed = pEgoInfo->fegoVelocity_mps;

  // compensation BSD parameter with AUTOSAR coordinate(add vehicle rear
  // position to front axis distance)
  // float32 fBSD_PLAUSIBILITY_FIRSTX_MIN =
  //     BSD_PLAUSIBILITY_FIRSTX_MIN - pVehPar->fVehRear2FrontAxis_met;
  // float32 fBSD_SOT_PLA_FIRSTX_MIN =
  //     BSD_SOT_PLA_FIRSTX_MIN - pVehPar->fVehRear2FrontAxis_met;
  // float32 fBSD_SOT_PLA_XMOVED_MIN =
  //     BSD_SOT_PLA_XMOVED_MIN - pVehPar->fVehRear2FrontAxis_met;
  // float32 fBSD_SOT_PLA_XMOVED_MIN_WARNING =
  //     BSD_SOT_PLA_XMOVED_MIN_WARNING - pVehPar->fVehRear2FrontAxis_met;
  // float32 fBSD_PLAUSIBILITY_XMOVED_MIN =
  //     BSD_PLAUSIBILITY_XMOVED_MIN - pVehPar->fVehRear2FrontAxis_met;
  // float32 fBSD_PLAUSIBILITY_XMOVED_MIN_WARNING =
  //     BSD_PLAUSIBILITY_XMOVED_MIN_WARNING - pVehPar->fVehRear2FrontAxis_met;
  // float32 fBSD_GHOST_PLA_FISRTX_MIN =
  //     BSD_GHOST_PLA_FISRTX_MIN - pVehPar->fVehRear2FrontAxis_met;
  // float32 fBSD_GHOST_PLA_XMOVED_MAX =
  //     BSD_GHOST_PLA_XMOVED_MAX - pVehPar->fVehRear2FrontAxis_met;

  // HE_PERCEPTION:  NO need use pVehPar->fVehRear2FrontAxis_met to
  // compensation BSD parameter with AUTOSAR coordinate(add vehicle rear
  // position to front axis distance)
  float32 fBSD_PLAUSIBILITY_FIRSTX_MIN =
      BSD_PLAUSIBILITY_FIRSTX_MIN;  // pVehPar->fVehRear2FrontAxis_met;
  float32 fBSD_SOT_PLA_FIRSTX_MIN =
      BSD_SOT_PLA_FIRSTX_MIN;  // pVehPar->fVehRear2FrontAxis_met;
  float32 fBSD_SOT_PLA_XMOVED_MIN =
      BSD_SOT_PLA_XMOVED_MIN;  // pVehPar->fVehRear2FrontAxis_met;
  float32 fBSD_SOT_PLA_XMOVED_MIN_WARNING =
      BSD_SOT_PLA_XMOVED_MIN_WARNING;  // pVehPar->fVehRear2FrontAxis_met;
  float32 fBSD_PLAUSIBILITY_XMOVED_MIN =
      BSD_PLAUSIBILITY_XMOVED_MIN;  // pVehPar->fVehRear2FrontAxis_met;
  float32 fBSD_PLAUSIBILITY_XMOVED_MIN_WARNING =
      BSD_PLAUSIBILITY_XMOVED_MIN_WARNING;  // pVehPar->fVehRear2FrontAxis_met;
  float32 fBSD_GHOST_PLA_FISRTX_MIN =
      BSD_GHOST_PLA_FISRTX_MIN;  // pVehPar->fVehRear2FrontAxis_met;
  float32 fBSD_GHOST_PLA_XMOVED_MAX =
      BSD_GHOST_PLA_XMOVED_MAX;  // pVehPar->fVehRear2FrontAxis_met;

  /* If an object is once plausible keep it.
     Object plausibility check is done only for objects behind the 90line(to
     avoid influence on SOT performance) Very old object should also be
     plausible */
  if (fXMoved > pBSDGlobals->fMinXmoved_met) {
    if ((pBSDObj->bPlausibility == TRUE) ||
        (pSRRObj->History.fFirstDetectX_met > fBSD_PLAUSIBILITY_FIRSTX_MIN) ||
        (pGenObj->General.uiLifeCycles_nu >
         BSD_PLAUSIBILITY_LIFECYCLES_MIN)) {  // 2
      bObjPlausibility = TRUE;
    } else {
      /* For object which seems to be a SOT the threshold should be low to
       * ensure long vehicle performance */
      if ((pGenObj->Kinemactic.fVrelX_mps < BSD_SOT_PLA_VRELX_MAX) &&
          (pSRRObj->History.fFirstDetectX_met > fBSD_SOT_PLA_FIRSTX_MIN)) {
        if ((fXMoved > fBSD_SOT_PLA_XMOVED_MIN) ||
            ((pBSDGlobals->bBSDWarnActive == TRUE) &&
             (fXMoved > fBSD_SOT_PLA_XMOVED_MIN_WARNING))) {
          bObjPlausibility = TRUE;
        }
      } else {
        if ((fXMoved > fBSD_PLAUSIBILITY_XMOVED_MIN) ||
            ((pBSDGlobals->bBSDWarnActive == TRUE) &&
             (fXMoved > fBSD_PLAUSIBILITY_XMOVED_MIN_WARNING))) {
          bObjPlausibility = TRUE;
        }
      }
    }
  }
  /* During SOT ghost objects may appear behind the subject that should not be
   * marked as plausible */
  if ((bObjPlausibility == TRUE) && (fEgoSpeed > BSD_GHOST_PLA_EGOSPEED_MIN) &&
      (pBSDGlobals->ScenarioObserver.uNumberSoTObjsLastCycle_nu > 0U) &&
      (pSRRObj->History.fFirstDetectX_met > fBSD_GHOST_PLA_FISRTX_MIN) &&
      (fXMoved < fBSD_GHOST_PLA_XMOVED_MAX) &&
      (pBSDObj->bBSDWarning == FALSE) &&
      (pBSDObj->ubClass_nu == BSD_CLASS_VEH_REAR)) {
    bObjPlausibility = FALSE;
  }
  return bObjPlausibility;
}

/*****************************************************************************
  Functionname: BSDCheckObjectShortWarning                                  */ /*!

  @brief:Check for short warnings

  @description:Calculate predicted time for object to exit and compare to defined threshold

  @param[in]:uObj, pGenObj,pLBSInputInfo,pRoad, pBSDWarnParameter

  @return:bShortWarning,short warning flag for current object
*****************************************************************************/
boolean BSDCheckObjectShortWarning(
    uint8 uObj, const BSD_GenObject_st* pGenObj,
    const BSD_LBSInputInfo_st* pLBSInputInfo, const BSDRoad_t* pRoad,
    const BSDWarningParameter_t* pBSDWarnParameter) {
  const float32 fSOTWarnDuration = pBSDWarnParameter->fSoTMinWarnDuration_s;
  BSD_Info_t* pBSDObj = pGetBSDCalculatePointer_ObjInfo(uObj);
  BSD_Globals_t* pBSDGlobals = pGetBSDCalculatePointer_BSDGlobals();
  float32 fTimeToExit = 0.F;
  boolean bShortWarning = FALSE;

  /* Check short warnings only for SOTs and when a MinWarn Duration is defined.
     No short warning shall be calculated if there is currently a active
     warning. */
  if ((fSOTWarnDuration > C_F32_DELTA) && (pBSDObj->bIsSoT == TRUE) &&
      ((pBSDGlobals->bBSDWarnActive == FALSE))) {
    fTimeToExit = BSDCalculateTimeToExit(uObj, pGenObj, pLBSInputInfo,
                                         pBSDWarnParameter, pRoad);
    if (fTimeToExit < fSOTWarnDuration) {
      bShortWarning = TRUE;
    }
  }
  return bShortWarning;
}

/*****************************************************************************
  Functionname: BSDCheckObjectWarningConditions */ /*!

  @brief:Check whether all warning conditions are fulfilled

  @description:check warning conditions function

  @param[in]:uObj, pSRRObj,pLBSInputInfo

  @return:bWarning,the warning flag of current object
*****************************************************************************/
boolean BSDCheckObjectWarningConditions(
    uint8 uObj, const BSD_SRRObject_st* pSRRObj,
    const BSD_LBSInputInfo_st* pLBSInputInfo, uint8 uBSDGearPosition) {
  const BSD_LCAObjInfo_t* pLCAObj =
      pGetBSD_LBSObjInfoPointer_LCAObjInfo(uObj, pLBSInputInfo);
  BSD_Globals_t* pBSDGlobal = pGetBSDCalculatePointer_BSDGlobals();
  BSD_Info_t* pBSDObj = pGetBSDCalculatePointer_ObjInfo(uObj);
  boolean bWarning = FALSE;

  // Check if all the relevant flags are set to keep a existing waring
  if ((1 != uBSDGearPosition) && (pSRRObj->Qualifiers.bObjStable == TRUE) &&
      //(pLCAObj->bLCAMirrorObject == FALSE) &&
      //(pLCAObj->bLCAMirrorFrontObject == FALSE) &&
      (pBSDObj->bInBSDZone == TRUE) && (pBSDObj->bBSDRelevant == TRUE) &&
      (pBSDObj->bLivedLongEnough == TRUE) &&
      (pBSDObj->bObjectOnOwnlane == FALSE) &&
      (pBSDObj->bObjectBehindGRD == FALSE) &&
      (pBSDObj->bPlausibility == TRUE)) {
    // The object already has a warning,keep it warning flag
    if (pBSDObj->bBSDWarning == TRUE) {
      bWarning = TRUE;
    } else {
      // If no warning is active:check some more flag which are necessary to
      // start the warning
      if ((pBSDObj->bUpdatedRecently == TRUE) &&
          (pBSDObj->ubGrdHitCounter_nu == 0U) &&
          (pBSDObj->bQualityEnough == TRUE) &&
          (pBSDObj->bSoTDelayActive == FALSE) &&
          (pBSDObj->bShortWarn == FALSE) && (pBSDObj->bFastSoT == FALSE) &&
          (pBSDObj->bObjectAndZoneOverlap == TRUE)) {
        bWarning = TRUE;
      }
    }
  }

  return bWarning;
}

/*****************************************************************************
  Functionname: BSDCalculateObjectRelDist                                  */ /*!

  @brief:Calculate Object distance to ego

  @description:according to object longitudinal distance to calculate current object real distance

  @param[in]:uObj,pGenObj, pSRRObj,pLBSInputInfo)

  @return:fDistX,the compensation real object distance
*****************************************************************************/
float32 BSDCalculateObjectRelDist(uint8 uObj, const BSD_GenObject_st* pGenObj,
                                  const BSD_SRRObject_st* pSRRObj,
                                  const BSD_LBSInputInfo_st* pLBSInputInfo) {
  float32 fDistX = 0.F;
  float32 fObjLengthFront = 0.F;
  BSD_Info_t* pBSDObjInfo = pGetBSDCalculatePointer_ObjInfo(uObj);

  // Calculate real object distance
  fObjLengthFront = pGenObj->Geometry.fLengthFront_met;
  // fDistX = fObjLengthFront + pGenObj->Kinemactic.fDistX_met; //ST_PERCEPTION
  fDistX = pGenObj->Kinemactic.fDistX_met;  // HZ_PERCEPTION
  // fDistX = fDistX + pLBSInputInfo->LBSGlobalInfo.fSensorOffsetToRear_met;

  return fDistX;
}

/*****************************************************************************
  Functionname: BSDSetGlobalWarning                                  */ /*!

  @brief:Set global BSD warning flag

  @description:check the object warning state,and switch on the global pSRRObjInfol warning

  @param[in]:uObj,pBSDObjInfo,fObjDistX)

  @return:void
*****************************************************************************/
void BSDSetGlobalWarning(uint8 uObj, BSD_Info_t* pBSDObjInfo,
                         float32 fObjDistX) {
  BSD_Globals_t* pBSDGlobal = pGetBSDCalculatePointer_BSDGlobals();

  if (pBSDObjInfo->bBSDWarning == TRUE) {
    switch (pBSDObjInfo->eObjDirection) {
      case LBS_BSD_LeftSensorObj:
        pBSDGlobal->bBSDWarnActiveLeft = TRUE;
        if (fObjDistX > pBSDGlobal->fBSDWarnActiveLeftDistX_met) {
          pBSDGlobal->uBSDWarnActiveLeftID_nu = uObj;
          pBSDGlobal->fBSDWarnActiveLeftDistX_met = fObjDistX;
        }
        break;
      case LBS_BSD_RightSensorObj:
        pBSDGlobal->bBSDWarnActiveRight = TRUE;
        if (fObjDistX > pBSDGlobal->fBSDWarnActiveRightDistX_met) {
          pBSDGlobal->uBSDWarnActiveRightID_nu = uObj;
          pBSDGlobal->fBSDWarnActiveRightDistX_met = fObjDistX;
        }
        break;
      default:
        break;
    }

  } else {
    // nothing to do
  }
}

/*****************************************************************************
  Functionname: BSDCalculateDistToDrivenCurve */ /*!

  @brief :Compute for each distance the y-position of the given half-circle radius

  @description

  @param[in]:fXpos x-distance
             fR    Radius of circle

  @return : fRet y-distance
*****************************************************************************/
float32 BSDCalculateDistToDrivenCurve(float32 fXpos, float32 fR) {
  float32 fY1 = 0.F;
  float32 fY2 = 0.F;
  float32 fRet = F32_VALUE_INVALID;
  float32 fRadix = SQR(fR) - SQR(fXpos);

  if (fRadix > C_F32_DELTA) {
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

/*****************************************************************************
  Functionname: BSDCalculateGrdDist2Course                                  */ /*!

  @brief:Compute for each distance the y-position of the given half-Circle radius

  @description:Compute for each distance the y-position of the given half-Circle radius

  @param[in]:uObj,pGenObj,pSRRObj,pEgoInfo,pRoad, pSensorMounting, pbGrdDistFound)

  @return:void
*****************************************************************************/
void BSDCalculateGrdDist2Course(uint8 uObj, const BSD_GenObject_st* pGenObj,
                                const BSD_SRRObject_st* pSRRObj,
                                const BSDVehicleInfo_t* pEgoInfo,
                                const BSDRoad_t* pRoad,
                                const BSDSensorMounting_t* pSensorMounting,
                                float32* pfGrdDist2Course,
                                boolean* pbGrdDistFound) {
  float32 fConfYOffset = pRoad->fConfYOffset_per;
  float32 fGrdDist2Course = pRoad->fYOffsetFused_met;
  float32 fConfRoadBorderEstim = pRoad->BorderEstmGridData_fConf_per;
  float32 fFirstDetectX = pSRRObj->History.fFirstDetectX_met;
  float32 fSensorYPos = pSensorMounting->fLatPos_met;
  float32 fMinBorderConf = 0.F;
  // float32 fLatDistBorder2Course;

  // Calculate the minimum confidence necessary to trust the road estimation
  // If we are driving slowly we can not expect a very high confidence.In this
  // case we reduce the minimum threshold
  fMinBorderConf = GDBmathLinFuncLimBounded(
      pEgoInfo->fegoVelocity_mps, BSD_LI_BORDER_MIN_EGOSPEED,
      BSD_LI_BORDER_MAX_EGOSPEED, BSD_LI_BORDER_MINBORDER_MIN_CONF,
      BSD_LI_BORDER_MINBORDER_MAX_CONF);
  if (pGenObj->bRightSensor) {
    // right side,set road offset and confidence
    fConfYOffset = pRoad->fConfYOppOffset_per;
    fGrdDist2Course = pRoad->fYOffsetFusedOppBorder_met;
  } else {
    // left side,set road offset and confidence
    fConfYOffset = pRoad->fConfYOffset_per;
    fGrdDist2Course = pRoad->fYOffsetFused_met;
  }

  // The fused road border has enough confidence
  if (fConfYOffset > fMinBorderConf) {
    //*pfGrdDist2Course = pRoad->fYOffsetFused_met + fSensorYPos; //do not
    // compensention sensor
    *pfGrdDist2Course = fGrdDist2Course;
    *pbGrdDistFound = TRUE;
  }

  // Now missing border estimation data,temp no to use this function
  // The fused road border and the guardrail track can not detect guardrails or
  // walls which appear next to us very fast Therefor we should use the road
  // estimation for objects in the front part of the FOV also,when it is
  // confidence is sufficient if (  (pGenObj->Kinemactic.fDistX_met >
  // BSD_GRD_DIST_OBJ_MIN_DISTX)
  //	&&(fFirstDetectX > BSD_GRD_DIST_OBJ_FIRSTDETECT_X)
  //	&&(fConfRoadBorderEstim > fMinBorderConf))
  //{
  //	fLatDistBorder2Course = pRoad->BorderEstmGridData_fYDist_met +
  // fSensorYPos; 	*pfGrdDist2Course = MIN(fLatDistBorder2Course,
  //*pfGrdDist2Course); 	*pbGrdDistFound = TRUE;
  //}
}

/*****************************************************************************
  Functionname: BSDCalculateUpdateAndLimitCounter */ /*!

  @brief:Update an observer within specified limits

  @description:Update an observer within specified limits

  @param[in]:uCounter,limits value
             iInc,Value used to updated the counter
			 uMin,Minimum value allowed for the counter
			 uMax,Maximum value allowed for the counter

  @return:void
*****************************************************************************/
void BSDCalculateUpdateAndLimitCounter(uint8* uCounter, sint8 iInc, uint8 uMin,
                                       uint8 uMax) {
  sint8 iTemp = (sint8)*uCounter;

  iTemp = iTemp + iInc;
  iTemp = TUE_CML_MinMax((sint8)uMin, (sint8)uMax, iTemp);
  *uCounter = (uint8)iTemp;
}

/*****************************************************************************
  Functionname: BSDCalculateObjectGrdrailRelation */ /*!

  @brief:Calculate the relation of the object and guardrail

  @description:Calculate the relation of the object and guardrail

  @param[in]:uObj,pGenObj,pSRRObj,fGrdDist2Course, bGrdDistFound,pbBehindGrd,pbGrdHit

  @return:void
*****************************************************************************/
void BSDCalculateObjectGrdrailRelation(
    uint8 uObj, const BSD_GenObject_st* pGenObj,
    const BSD_SRRObject_st* pSRRObj, float32 fGrdDist2Course,
    boolean bGrdDistFound, boolean* pbBehindGrd, boolean* pbGrdHit) {
  static GDBVector2_t InnerSideGrdThresh[BSD_GRD_REL_NROF_THRESHOLDS] = {
      {BSD_INNER_SIDE_X1, BSD_INNER_SIDE_Y1},
      {BSD_INNER_SIDE_X2, BSD_INNER_SIDE_Y2},
      {BSD_INNER_SIDE_X3, BSD_INNER_SIDE_Y3},
      {BSD_INNER_SIDE_X4, BSD_INNER_SIDE_Y4}};

  static GDBVector2_t OuterSideGrdThresh[BSD_GRD_REL_NROF_THRESHOLDS] = {
      {BSD_OUTER_SIDE_X1, BSD_OUTER_SIDE_Y1},
      {BSD_OUTER_SIDE_X2, BSD_OUTER_SIDE_Y2},
      {BSD_OUTER_SIDE_X3, BSD_OUTER_SIDE_Y3},
      {BSD_OUTER_SIDE_X4, BSD_OUTER_SIDE_Y4}};

  const boolean bRightSensorFlag = pGenObj->bRightSensor;
  const float32 fObjDistX = pGenObj->Kinemactic.fDistX_met;
  const float32 fObjDist2Course = pSRRObj->RoadRelation.fDist2Course_met;
  float32 fObjDist2Grd = F32_VALUE_INVALID;
  float32 fInnerObj2GrdDistThresh = 0.0F;
  float32 fOuterObj2GrdDistThresh = 0.0F;

  if (bGrdDistFound == TRUE) {
    /*if (bRightSensorFlag == TRUE)
    {
            OuterSideGrdThresh[0].f1 = -OuterSideGrdThresh[0].f1;
            OuterSideGrdThresh[1].f1 = -OuterSideGrdThresh[1].f1;
            OuterSideGrdThresh[2].f1 = -OuterSideGrdThresh[2].f1;
            OuterSideGrdThresh[3].f1 = -OuterSideGrdThresh[3].f1;

            InnerSideGrdThresh[0].f1 = -InnerSideGrdThresh[0].f1;
            InnerSideGrdThresh[1].f1 = -InnerSideGrdThresh[1].f1;
            InnerSideGrdThresh[2].f1 = -InnerSideGrdThresh[2].f1;
            InnerSideGrdThresh[3].f1 = -InnerSideGrdThresh[3].f1;
    }*/

    // Calculate the lateral distance from object to guardrail
    fObjDist2Grd = fObjDist2Course - fGrdDist2Course;

    // Check if the Object is behind the guardrail,and increase counter for that
    //   left guardrail                        right guardrail       "*" means
    //   objects
    //       |                                       |
    //       |  *  object in the right side   	     |  *  object in the right
    //       side |     fObjDist2Grd < 0				     |
    //       behind guardrail
    //       | | fObjDist2Grd
    //       <
    //       0
    //    *  |  object in the left side			     |
    //       |  behind guardrail				 *   |  object
    //       in the left side |  fObjDist2Grd > 0 | fObjDist2Grd > 0

    if ((bRightSensorFlag == FALSE) && (fObjDist2Grd > 0.0F)) {
      *pbBehindGrd = TRUE;
    } else if ((bRightSensorFlag == TRUE) && (fObjDist2Grd < 0.0F)) {
      *pbBehindGrd = TRUE;
    } else {
      // Nothing to do
    }

    fOuterObj2GrdDistThresh = CML_f_CalculatePolyValue(
        BSD_GRD_REL_NROF_THRESHOLDS, OuterSideGrdThresh, fObjDistX);
    fInnerObj2GrdDistThresh = CML_f_CalculatePolyValue(
        BSD_GRD_REL_NROF_THRESHOLDS, InnerSideGrdThresh, fObjDistX);

    if (bRightSensorFlag == TRUE) {
      fOuterObj2GrdDistThresh = -fOuterObj2GrdDistThresh;
      fInnerObj2GrdDistThresh = -fInnerObj2GrdDistThresh;
    }

    // Check whether the object is located between the calculate boundaries
    if ((bRightSensorFlag == FALSE) &&
        (fObjDist2Grd < fOuterObj2GrdDistThresh) &&
        (fObjDist2Grd > fInnerObj2GrdDistThresh)) {
      *pbGrdHit = TRUE;
    } else if ((bRightSensorFlag == TRUE) &&
               (fObjDist2Grd < fInnerObj2GrdDistThresh) &&
               (fObjDist2Grd > fOuterObj2GrdDistThresh)) {
      *pbGrdHit = TRUE;
    } else {
      // Nothing to do
    }
  }
}

/*****************************************************************************
  Functionname: BSDCalculateObjectInRectZone                                  */ /*!

  @brief:Check of object is with its boundaries inside the BSD Zone

  @description:Check of object is with its boundaries inside the BSD Zone

  @param[in]:uObj, fZoneXMin, fZoneXMax,fZoneYMin,fZoneYMax,pLBSInputInfo

  @return:bInZone,flag whether object is inside the BSD Zone
*****************************************************************************/
boolean BSDCalculateObjectInRectZone(uint8 uObj, float32 fZoneXMin,
                                     float32 fZoneXMax, float32 fZoneYMin,
                                     float32 fZoneYMax,
                                     const BSD_LBSInputInfo_st* pLBSInputInfo) {
  const BSD_LBSObjInfo_st* pLBSObj =
      pGetBSD_LBSObjInfoPointer_LBSObjInfo(uObj, pLBSInputInfo);
  boolean bInZone = FALSE;

  // If any side of object border is in the SOTZone, bInZone is true.
  if ((pLBSObj->ObjBorders.fXmin_met < fZoneXMax) &&
      (pLBSObj->ObjBorders.fXmax_met > fZoneXMin) &&
      (pLBSObj->ObjBorders.fYmin_met < fZoneYMax) &&
      (pLBSObj->ObjBorders.fYmax_met > fZoneYMin)) {
    bInZone = TRUE;
  }
  return bInZone;
}

/*****************************************************************************
  Functionname: BSDCalculateObjectInCurvedZone */ /*!

  @brief:Check of object is with its boundaries inside the BSD Zone(Curve condition)

  @description:Check of object is with its boundaries inside the BSD Zone

  @param[in]:uObj,fZoneXMin,fZoneXMax, fZoneYMin,fZoneYMax,pLBSInputInfo,pRoad

  @return:bInZone,flag whether object is inside the BSD Zone
*****************************************************************************/
boolean BSDCalculateObjectInCurvedZone(uint8 uObj, float32 fZoneXMin,
                                       float32 fZoneXMax, float32 fZoneYMin,
                                       float32 fZoneYMax,
                                       const BSD_LBSInputInfo_st* pLBSInputInfo,
                                       const BSDRoad_t* pRoad) {
  const BSD_LBSObjInfo_st* pLBSObj =
      pGetBSD_LBSObjInfoPointer_LBSObjInfo(uObj, pLBSInputInfo);

  float32 fDrivenCurveRadius = fABS(pRoad->fDrivenCurveRadius_met);
  float32 fRoadCurveRadius = fABS(pRoad->fCurveRadius_met);
  float32 fCurveRadius = TUE_CML_Min(fDrivenCurveRadius, fRoadCurveRadius);

  boolean bInZone = FALSE;

  float32 fObjWidth = 0.F;
  float32 fObjLength = 0.F;
  float32 fTmpZoneYmin = 0.F;
  float32 fTmpZoneYmax = 0.F;
  float32 fTmpZoneWidth = 0.F;
  float32 fTmpZoneLength = 0.F;
  float32 fDist2CourseXmax = 0.F;
  float32 fDist2CourseXmin = 0.F;

  // Step1:Check whether the object is inside the inner rectangle of curved zone
  // if this if TRUE we know that the object is in zone
  fDist2CourseXmax = BSDCalculateDistToDrivenCurve(fZoneXMax, fCurveRadius);
  fDist2CourseXmin = BSDCalculateDistToDrivenCurve(fZoneXMin, fCurveRadius);

  // check inferior zone
  fTmpZoneYmin =
      MAX((fDist2CourseXmax + fZoneYMin), (fDist2CourseXmin + fZoneYMin));
  fTmpZoneYmin = MAX(fTmpZoneYmin, fZoneYMin);
  fTmpZoneYmax =
      MIN((fDist2CourseXmax + fZoneYMax), (fDist2CourseXmin + fZoneYMax));
  fTmpZoneYmax = MIN(fTmpZoneYmax, fZoneYMax);

  bInZone = BSDCalculateObjectInRectZone(
      uObj, fZoneXMin, fZoneXMax, fTmpZoneYmin, fTmpZoneYmax, pLBSInputInfo);

  if (bInZone == FALSE) {
    // STEP2:the object is not in the inner rectangle
    // now we make help check whether the object is in the overlaying zone
    // if this is not TRUE we know that the object is outside the zone
    // otherwise some detailed checks are necessary

    fTmpZoneYmin =
        MIN((fDist2CourseXmax + fZoneYMin), (fDist2CourseXmin + fZoneYMin));
    fTmpZoneYmin = MIN(fTmpZoneYmin, fZoneYMin);
    fTmpZoneYmax =
        MAX((fDist2CourseXmax + fZoneYMax), (fDist2CourseXmin + fZoneYMax));
    fTmpZoneYmax = MAX(fTmpZoneYmax, fZoneYMax);
    bInZone = BSDCalculateObjectInRectZone(
        uObj, fZoneXMin, fZoneXMax, fTmpZoneYmin, fTmpZoneYmax, pLBSInputInfo);

    if (bInZone == TRUE) {
      // STEP3:At least one object corner is in superior zone ,so make
      // additional checks Reinitialize bInzone flag,because the outer rect
      // check is not sufficient
      bInZone = FALSE;

      fObjLength =
          pLBSObj->ObjBorders.fXmax_met - pLBSObj->ObjBorders.fXmin_met;
      fObjWidth = pLBSObj->ObjBorders.fYmax_met - pLBSObj->ObjBorders.fYmin_met;
      fTmpZoneLength = fZoneXMax - fZoneXMin;
      fTmpZoneWidth = fTmpZoneYmax - fTmpZoneYmin;

      if ((fObjLength > fTmpZoneLength) || (fObjWidth > fTmpZoneWidth)) {
        // STEP3.0:check whether the object is longer or wider than the superior
        // zone
        // In this case the object is definitely in the zone
        // but the object corner is not in the zone
        bInZone = TRUE;
      } else if (BSDCalculateCheckCornersInCurvedZone(
                     uObj, fZoneXMin, fZoneXMax, fZoneYMin, fZoneYMax,
                     fCurveRadius, pLBSInputInfo) == TRUE) {
        // STEP3.1:Make exact check for all corners
        // if object not longer or wider than the zone,need to check whether
        // object have corner in the zone area
        bInZone = TRUE;
      } else {
        // STEP3.2:No corner is in the zone,so we finally have to check whether
        // the object is in the zone with an edge
        // but not with corner,In this case the corner of the zone lays in the
        // object boundaries
        // We check this only for the rear corners of the zone

        fDist2CourseXmin =
            BSDCalculateDistToDrivenCurve(fZoneXMin, fCurveRadius);

        if ((fZoneXMin > pLBSObj->ObjBorders.fXmin_met) &&
            (fZoneXMin < pLBSObj->ObjBorders.fXmax_met) &&
            (fDist2CourseXmin + fZoneYMin > pLBSObj->ObjBorders.fYmin_met) &&
            (fDist2CourseXmin + fZoneYMin < pLBSObj->ObjBorders.fYmax_met)) {
          // Check inner rear zone corner
          bInZone = TRUE;
        } else if ((fZoneXMin > pLBSObj->ObjBorders.fXmin_met) &&
                   (fZoneXMin < pLBSObj->ObjBorders.fXmax_met) &&
                   (fDist2CourseXmin + fZoneYMax >
                    pLBSObj->ObjBorders.fYmin_met) &&
                   (fDist2CourseXmin + fZoneYMax <
                    pLBSObj->ObjBorders.fYmax_met)) {
          // check outer rear zone corner
          bInZone = TRUE;
        } else {
          // noting to do
        }
      }
    }
  }

  return bInZone;
}

/*****************************************************************************
  Functionname: BSDCalculateCheckCornersInCurvedZone */ /*!

  @brief:Check of object is with its boundaries inside the BSD Zone

  @description:Check of object is with its boundaries inside the BSD Zone

  @param[in]:uObj,fZoneXMin, fZoneXMax,fZoneYMin,fZoneYMax,fCurveRadius,pLBSInputInfo

  @return:bCornerInZone,the flag whether an object has corner inside the BSDZone
*****************************************************************************/
boolean BSDCalculateCheckCornersInCurvedZone(
    uint8 uObj, float32 fZoneXMin, float32 fZoneXMax, float32 fZoneYMin,
    float32 fZoneYMax, float32 fCurveRadius,
    const BSD_LBSInputInfo_st* pLBSInputInfo) {
  const BSD_LBSObjInfo_st* pLBSObj =
      pGetBSD_LBSObjInfoPointer_LBSObjInfo(uObj, pLBSInputInfo);
  float32 fObjBorderXMin = pLBSObj->ObjBorders.fXmin_met;
  float32 fObjBorderXMax = pLBSObj->ObjBorders.fXmax_met;
  float32 fObjBorderYMin = pLBSObj->ObjBorders.fYmin_met;
  float32 fObjBorderYMax = pLBSObj->ObjBorders.fYmax_met;
  float32 fDist2Course = 0.F;
  boolean bCornerInZone = FALSE;

  // let start with th bottom edge calculate the distance to the ego trace(half
  // circle) for the lower edge According to the distance from the object to the
  // self-driving curve,to compensation zone area offset when vehicle in curve
  fDist2Course = BSDCalculateDistToDrivenCurve(fObjBorderXMin, fCurveRadius);

  // STEP1:check bottom inner corner of object whether in the zone area
  bCornerInZone = BSDCalculateCheckPointInCurvedZone(
      fObjBorderXMin, fObjBorderYMin, fZoneXMin, fZoneXMax, fZoneYMin,
      fZoneYMax, fDist2Course);

  if (bCornerInZone == FALSE) {
    // STEP2: check bottom outer corner of object whether in the zone area
    bCornerInZone = BSDCalculateCheckPointInCurvedZone(
        fObjBorderXMin, fObjBorderYMax, fZoneXMin, fZoneXMax, fZoneYMin,
        fZoneYMax, fDist2Course);
  }

  if (bCornerInZone == FALSE) {
    // Calculate the distance to the ego trace (half circle) for the upper edge
    fDist2Course = BSDCalculateDistToDrivenCurve(fObjBorderXMax, fCurveRadius);

    // do the same for the upper edge
    // STEP3:check upper inner corner of object whether in the zone area
    bCornerInZone = BSDCalculateCheckPointInCurvedZone(
        fObjBorderXMax, fObjBorderYMin, fZoneXMin, fZoneXMax, fZoneYMin,
        fZoneYMax, fDist2Course);

    if (bCornerInZone == FALSE) {
      // STEP4:check upper outer corner of object whether in the zone area
      bCornerInZone = BSDCalculateCheckPointInCurvedZone(
          fObjBorderXMax, fObjBorderYMax, fZoneXMin, fZoneXMax, fZoneYMin,
          fZoneYMax, fDist2Course);
    }
  }

  return bCornerInZone;
}

/*****************************************************************************
  Functionname: BSDCalculateCheckPointInCurvedZone */ /*!

  @brief:Checks if given point is inside a zone which is curved around the Ego trace

  @description:Checks if given point is inside a zone which is curved around the Ego trace

  @param[in]: fObjBorderX,fObjBorderY, fZoneXMin, fZoneXMax, fZoneYMin,fZoneYMax, fDist2Course

  @return:bRet,flag if point is in zone or not
*****************************************************************************/
boolean BSDCalculateCheckPointInCurvedZone(float32 fObjBorderX,
                                           float32 fObjBorderY,
                                           float32 fZoneXMin, float32 fZoneXMax,
                                           float32 fZoneYMin, float32 fZoneYMax,
                                           float32 fDist2Course) {
  boolean bRet = FALSE;

  if ((fObjBorderY > (fDist2Course + fZoneYMin)) &&
      (fObjBorderY < (fDist2Course + fZoneYMax)) && (fObjBorderX > fZoneXMin) &&
      (fObjBorderX < fZoneXMax)) {
    bRet = TRUE;
  }

  return bRet;
}

/*****************************************************************************
  Functionname: BSDCalculateCheckGrdPreCondition */ /*!

  @brief:Check basic condition for a guardrail,velocity,history,etc...

  @description:Check whether the object movement allows guardrail classification

  @param[in]:Obj, pGenObj,pSRRObj, pRoad)

  @return:bPreCondFufilled,decision flag whether the object could be a guardrail object or not
*****************************************************************************/
boolean BSDCalculateCheckGrdPreCondition(uint8 Obj,
                                         const BSD_GenObject_st* pGenObj,
                                         const BSD_SRRObject_st* pSRRObj,
                                         const BSDRoad_t* pRoad) {
  const float32 fFirsetDetectX = pSRRObj->History.fFirstDetectX_met;
  const float32 fVrelX = pGenObj->Kinemactic.fVrelX_mps;
  const float32 fFusedBorederConf = pRoad->fConfYOffset_per;
  boolean bPreCondFufilled = FALSE;

  // Only objects which was not created far behind us and are not moving clearly
  // TOS should be considered as guardrail
  if ((fFusedBorederConf > BSD_GRD_DETECT_BORDER_CONF_MIN) ||
      ((fFirsetDetectX >
        BSD_GRD_DETECT_FIRSTDETECTX_MIN)  // compensation sensor position
       && (fVrelX < BSD_GRD_DETECT_VRELX_MAX))) {
    bPreCondFufilled = TRUE;
  }
  return bPreCondFufilled;
}

/*****************************************************************************
  Functionname: BSDClassifyFrontObject                                  */ /*!

  @brief:Classify object based which appear in front sector

  @description:Classify object based which appear in front sector

  @param[in]:uObj,pGenObj,pSRRObj,pLBSInputInfo, pEgoInfo,pRoad

  @return:void
*****************************************************************************/
void BSDClassifyFrontObject(uint8 uObj, const BSD_GenObject_st* pGenObj,
                            const BSD_SRRObject_st* pSRRObj,
                            const BSD_LBSInputInfo_st* pLBSInputInfo,
                            const BSDVehicleInfo_t* pEgoInfo,
                            const BSDRoad_t* pRoad,
                            const BSDVehParameter_t* pVehPar) {
  const BSD_LBSObjInfo_st* pLBSObj =
      pGetBSD_LBSObjInfoPointer_LBSObjInfo(uObj, pLBSInputInfo);
  BSD_Globals_t* pBSDGlobal = pGetBSDCalculatePointer_BSDGlobals();
  BSD_Info_t* pBSDObj = pGetBSDCalculatePointer_ObjInfo(uObj);

  float32 fYOffsetBorder = pRoad->fYOffsetFused_met;
  float32 fConfYOffsetBorder = pRoad->fConfYOffset_per;
  float32 fObjDist2Border = pSRRObj->RoadRelation.fDist2Border_met;
  float32 fObjAbsSpeedX =
      fABS(pEgoInfo->fegoVelocity_mps + pGenObj->Kinemactic.fVrelX_mps);
  boolean bDist2BorderValid = pSRRObj->RoadRelation.bDist2BorderValid;
  float32 fYOuterBSDZone = 0.F;

  // compensation BSD parameter with AUTOSAR coordinate(add vehicle rear
  // position to front axis distance)
  const float32 fBSD_FRONT_RECLASS_OBJ_XMIN_MAX =
      BSD_FRONT_RECLASS_OBJ_XMIN_MAX - pVehPar->fVehRear2FrontAxis_met;

  // If GRD counter is not zero wait with the classification
  if ((pBSDObj->ubGrdHitCounter_nu == 0U) &&
      (pBSDObj->ubClass_nu == BSD_CLASS_UNDEFINED)) {
    // check the minimum velocity threshold
    if (fObjAbsSpeedX > pBSDGlobal->fVxThreshMovStat_mps) {
      pBSDObj->ubClass_nu = BSD_CLASS_VEH_FRONT;
    } else {
      pBSDObj->ubClass_nu = BSD_CLASS_STATIC_FRONT;
    }
  } else {
    // Enable a reclassification for long living objects that are classified as
    // static front
    if ((pGenObj->General.uiLifeCycles_nu >
         BSD_RECLASS_STATIC_FRONT_MIN_LIFETIME) &&
        (pBSDObj->ubClass_nu == BSD_CLASS_STATIC_FRONT)) {
      // base object side
      if (pGenObj->bRightSensor == TRUE) {
        // reverse Y offset sign for right sensor for legible comparison on the
        // following if cond
        fYOffsetBorder = -fYOffsetBorder;
        fObjDist2Border = -fObjDist2Border;
        fYOuterBSDZone = pBSDGlobal->fBSDZoneYmin_met;
      } else {
        fYOuterBSDZone = pBSDGlobal->fBSDZoneYmax_met;
      }

      // The conditions for reclassification are very strict
      // check minimum x position of the object
      // check VX of the object for significant difference from stationary
      // targets check that the object is in front of a confirmed guardrail
      if ((fObjAbsSpeedX > pBSDGlobal->fVxThreshMovStat_mps) &&
          (fYOffsetBorder > fYOuterBSDZone) &&
          (pLBSObj->ObjBorders.fXmin_met < fBSD_FRONT_RECLASS_OBJ_XMIN_MAX) &&
          (fConfYOffsetBorder > BSD_FRONT_RECLASS_BORDER_CONF_MIN) &&
          (fObjDist2Border < BSD_FRONT_RECLASS_DIST2BORD_MAX) &&
          (bDist2BorderValid == TRUE)) {
        pBSDObj->ubClass_nu = BSD_CLASS_VEH_FRONT;
      }
    }
  }
}

/*****************************************************************************
  Functionname: BSDClassifyRearObject                                  */ /*!

  @brief Classify object based which appear in Rear Sector

  @description Classify object based which appear in Rear Sector( Angle > 104 degree)

  @param[in] uObj Array position of the object

  @return
*****************************************************************************/
void BSDClassifyRearObject(uint8 uObj) {
  BSD_Info_t* pBSDObj = pGetBSDCalculatePointer_ObjInfo(uObj);

  // If GRD counter is not zero wait with the classification
  if (pBSDObj->ubGrdHitCounter_nu == 0U) {
    pBSDObj->ubClass_nu = BSD_CLASS_VEH_REAR;
  }
}

/*****************************************************************************
  Functionname: BSDClassifySideObject                                  */ /*!

  @brief:Classify Object based which appear in side sector

  @description:Classify Object based which appear in side sector

  @param[in]uObj,pGenObj,pLBSInputInfo, pEgoInfo, pRoad

  @return:void
*****************************************************************************/
void BSDClassifySideObject(uint8 uObj, const BSD_GenObject_st* pGenObj,
                           const BSD_LBSInputInfo_st* pLBSInputInfo,
                           const BSDVehicleInfo_t* pEgoInfo,
                           const BSDRoad_t* pRoad,
                           const BSDVehParameter_t* pVehPar) {
  const BSD_LBSObjInfo_st* pLBSObj =
      pGetBSD_LBSObjInfoPointer_LBSObjInfo(uObj, pLBSInputInfo);
  BSD_Globals_t* pBSDGlobal = pGetBSDCalculatePointer_BSDGlobals();
  BSD_Info_t* pBSDObj = pGetBSDCalculatePointer_ObjInfo(uObj);

  float32 fXMinObj = pLBSObj->ObjBorders.fXmin_met;
  float32 fEgoSpeed = pEgoInfo->fegoVelocity_mps;
  float32 fCurveRadiusMin =
      MIN(fABS(pRoad->fCurveRadius_met), fABS(pRoad->fDrivenCurveRadius_met));
  float32 fXMinThresh = 0.0F;

  // Standard case: objects appearing at the side sector first are usually
  // static
  if (pBSDObj->ubClass_nu == BSD_CLASS_UNDEFINED) {
    pBSDObj->ubClass_nu = BSD_CLASS_STATIC_SIDE;
  }

  // if classify side static object,check more condition
  // STEP1:base on ego situation to calculate fXMinThresh value
  if (pBSDObj->ubClass_nu == BSD_CLASS_STATIC_SIDE) {
    // Take the minimum value from the driven curve radius and the forward curve
    // radius for the threshold limitation
    if (fCurveRadiusMin < BSD_SIDE_RECLASS_LOW_CURVE_THRESH) {
      // Decrease XMinThresh if subject is in a curve
      fXMinThresh = GDBmathLinFuncLimBounded(
          fCurveRadiusMin, BSD_LI_SIDE_RECLASS_MIN_CURVERAD,
          BSD_LI_SIDE_RECLASS_MAX_CURVERAD, BSD_LI_SIDE_RECLASS_MIN_XMIN_THRESH,
          BSD_LI_SIDE_RECLASS_MAX_XMIN_THRESH);

    } else {
      // Decrease XMinThreshold if lateral movement is too high compared to
      // longitudinal movement
      if (((pLBSObj->fYMovement_met - pLBSObj->fXMovement_met) >
           BSD_SIDE_RECLASS_LAT2LONG_MOV_THRESH) &&
          (pBSDObj->bInBSDZone == FALSE)) {
        fXMinThresh = BSD_SIDE_RECLASS_XMIN_HIGH;
      } else {
        fXMinThresh = BSD_SIDE_RECLASS_XMIN;
      }
    }
    // compensation BSD parameter with AUTOSAR coordinate(add vehicle rear
    // position to front axis distance)
    fXMinThresh -= pVehPar->fVehRear2FrontAxis_met;

    // STEP2:check kinematics suggest its moving
    if ((fABS(fEgoSpeed + pGenObj->Kinemactic.fVrelX_mps) >
         pBSDGlobal->fVxThreshMovStat_mps) &&
        (fXMinObj < fXMinThresh) &&
        (fABS(pGenObj->Kinemactic.fVrelY_mps) < BSD_SIDE_RECLASS_VRELX_MAX)) {
      // Determine filter constant by minimum x position of the target
      const float32 fFillterConst = GDBmathLinFuncLimBounded(
          fXMinObj, BSD_LI_SIDE_RECLASS_MIN_XMIN, BSD_LI_SIDE_RECLASS_MAX_XMIN,
          BSD_LI_SIDE_RECLASS_MIN_LPF_ALPHA, BSD_LI_SIDE_RECLASS_MAX_LPF_ALPHA);

      // Filter up rear object confidence depending on the minimum x position
      GDB_Math_LowPassFilter(&pBSDObj->fRearConf_nu, 1.0F, fFillterConst);

    } else {
      // Filter down rear object confidence if conditions are not fulfilled
      GDB_Math_LowPassFilter(&pBSDObj->fRearConf_nu, 0.0F,
                             BSD_SIDE_RECLASS_LPF_DOWN_ALPHA);
    }

    // STEP3:check object confidence to classify
    // If the rear object confidence is large enough update the object class
    if (pBSDObj->fRearConf_nu > BSD_RECLASS_MIN_REAR_CONF) {
      pBSDObj->ubClass_nu = BSD_CLASS_VEH_CONFIRMED_REAR;
    }
  }
}

/*****************************************************************************
  Functionname: BSDClassifyConfirmedObjectAtRear */ /*!

  @brief:Reclassify object which are observed in the rear

  @description:Reclassify object which has a strong reflection at the rear of BSDZone

  @param[in]:uObj,pGenObj,pSRRObj, pLBSInputInfo,pEgoInfo

  @return:void
*****************************************************************************/
void BSDClassifyConfirmedObjectAtRear(uint8 uObj,
                                      const BSD_GenObject_st* pGenObj,
                                      const BSD_SRRObject_st* pSRRObj,
                                      const BSD_LBSInputInfo_st* pLBSInputInfo,
                                      const BSDVehicleInfo_t* pEgoInfo,
                                      const BSDVehParameter_t* pVehPar) {
  const BSD_LBSObjInfo_st* pLBSObj =
      pGetBSD_LBSObjInfoPointer_LBSObjInfo(uObj, pLBSInputInfo);
  BSD_Globals_t* pBSDGlobal = pGetBSDCalculatePointer_BSDGlobals();
  BSD_Info_t* pBSDObj = pGetBSDCalculatePointer_ObjInfo(uObj);

  float32 fEgoSpeedX = pEgoInfo->fegoVelocity_mps;
  float32 fObjPOE = pSRRObj->Qualifiers.fProbabilityOfExistence_per;
  float32 fObjAbsVx = fABS(fEgoSpeedX + pGenObj->Kinemactic.fVrelX_mps);
  float32 fObjAbsDistY = fABS(pGenObj->Kinemactic.fDistY_met);

  // ST: compensation BSD parameter with AUTOSAR coordinate(add vehicle rear
  // position to front axis distance)

  // HZ: No need compensation BSD parameter with AUTOSAR coordinate(add vehicle
  // rear
  // position to front axis distance)
  const float32 fBSD_RECLASS_AT_REAR_XMIN_MAX =
      BSD_RECLASS_AT_REAR_XMIN_MAX;  // - pVehPar->fVehRear2FrontAxis_met;
  const float32 fBSD_RECLASS_AT_REAR_XMAX_MAX =
      BSD_RECLASS_AT_REAR_XMAX_MAX;  // - pVehPar->fVehRear2FrontAxis_met;

  // Check some conditions:
  // the object should have a minimal age
  // the object shouldn't be on or behind guardrail object
  // the probability of existence should be sufficient
  // the object is measured regularly
  // the ego speed is sufficient
  // the object is moving faster to the rear than the calculated VxThresh
  // the object should not be too close,because then it is probably not a
  // vehicle the object should be measured in the last cycles the object has to
  // be far enough in the rear: Rear position is smaller than -4m,or front
  // position is smaller than -1.5m
  if ((pLBSObj->fCycletimeSum_s > BSD_RECLASS_AT_REAR_LIFETIME_MIN) &&  // 0.3
      (pLBSObj->fUpdateRate_nu > BSD_RECLASS_AT_REAR_UPDATE_MIN) &&     // 0.9
      (pBSDObj->ubGrdHitCounter_nu == 0U) &&
      (pBSDObj->ubBehindGrdCounter_nu == 0U) &&
      (pBSDObj->bUpdatedRecentlyWeak == TRUE) &&
      (fObjPOE > BSD_RECLASS_AT_REAR_POE_MIN) &&  // 0.98
      (fEgoSpeedX > BSD_CLASSIFY_VEH_CONFIRMED_REAR_VOWN_MIN) &&
      (fObjAbsVx > pBSDGlobal->fVxThreshMovStat_mps) &&
      (fObjAbsDistY > BSD_RECLASS_AT_REAR_DISTY_MIN) &&
      ((pLBSObj->ObjBorders.fXmin_met < fBSD_RECLASS_AT_REAR_XMIN_MAX) ||
       (pLBSObj->ObjBorders.fXmax_met < fBSD_RECLASS_AT_REAR_XMAX_MAX))) {
    // if there is no class yet or it is classified as static
    if ((pBSDObj->ubClass_nu == BSD_CLASS_UNDEFINED) ||
        ((pBSDObj->ubClass_nu > BSD_CLASS_VEH_LAST) &&
         (pBSDObj->ubClass_nu < BSD_CLASS_STATIC_LAST_RECOVERABLE))) {
      // reclassify object to rear confirmed vehicle
      pBSDObj->ubClass_nu = BSD_CLASS_VEH_CONFIRMED_REAR;
    }
  }
}

/*****************************************************************************
  Functionname: BSDCalculateTimeToExit                                  */ /*!

  @brief:Calculate predicted time for object to exit

  @description:Calculate predicted time for object to exit

  @param[in]:uObj,pGenObj,pLBSInputInfo,pBSDWarnParameter,pRoad

  @return:Predicted time until the object leaves the zone
*****************************************************************************/
float32 BSDCalculateTimeToExit(uint8 uObj, const BSD_GenObject_st* pGenObj,
                               const BSD_LBSInputInfo_st* pLBSInputInfo,
                               const BSDWarningParameter_t* pBSDWarnParameter,
                               const BSDRoad_t* pRoad) {
  float32 fTimeToExitRear = 0.F;
  float32 fTimeToExitSide = F32_VALUE_INVALID;
  BSDZone_ObjPar* pBSDZoneObjPar = pGetBSDCalculatePointer_BSDZoneObjPar(uObj);
  float32 fZoneXmin = pBSDZoneObjPar->fZoneXmin_met;
  float32 fZoneYmax = pBSDZoneObjPar->fZoneYmax_met;
  float32 fZoneYmin = pBSDZoneObjPar->fZoneYmin_met;
  const BSD_LBSObjInfo_st* pLBSObj =
      pGetBSD_LBSObjInfoPointer_LBSObjInfo(uObj, pLBSInputInfo);
  float32 fObjBordersXmax = pLBSObj->ObjBorders.fXmax_met;
  float32 fObjBordersXmin = pLBSObj->ObjBorders.fXmin_met;
  float32 fObjBordersYmax = pLBSObj->ObjBorders.fYmax_met;
  float32 fObjBordersYmin = pLBSObj->ObjBorders.fYmin_met;

  float32 fVxObj = pGenObj->Kinemactic.fVrelX_mps;
  float32 fVyObj = pGenObj->Kinemactic.fVrelY_mps;
  float32 fRelevantYBorderZone = 0.F;
  float32 fRelevantYBorderObj = 0.F;
  float32 fAbsCurveRadius = fABS(pRoad->fDrivenCurveRadius_met);

  /* Bottom of zone exit */
  fTimeToExitRear =
      BSDCalculate_TTBreakthroughLine(fZoneXmin, fObjBordersXmax, fVxObj);

  /* Side of zone exit */
  if (fABS(fVyObj) < C_F32_DELTA) {
    fTimeToExitSide = F32_VALUE_INVALID;
  } else {
    /* Get the relevant border of the zone and the object depending on the side
     * where it will leave */
    if (fVyObj < 0.F) {
      fRelevantYBorderZone = fZoneYmin;
      fRelevantYBorderObj = fObjBordersYmax;
    } else {
      fRelevantYBorderZone = fZoneYmax;
      fRelevantYBorderObj = fObjBordersYmin;
    }

    if (fAbsCurveRadius > F32_BSDZONE_CURVEADAPTION_RADIUS) {
      fTimeToExitSide = BSDCalculate_TTBreakthroughLine(
          fRelevantYBorderZone, fRelevantYBorderObj, fVyObj);
    } else {
      fTimeToExitSide = BSDCalculate_TTBreakthroughCurve(
          uObj, fObjBordersXmin, fObjBordersXmax, fRelevantYBorderObj,
          fRelevantYBorderZone, fVyObj, pBSDWarnParameter, pRoad);
    }
  }
  return TUE_CML_Min(fTimeToExitSide, fTimeToExitRear);
}

///*****************************************************************************
//  Functionname: SafeDiv                                  */ /*!
//
//  @brief: Return a value which is verified not zero regardless of sign
//
//  @description: Return a value which is verified not zero or close to zero
//  regardless of sign
//
//  @param[in]
//
//  @return
//*****************************************************************************/
// float32 SafeDiv(float32 fDivisor)
//{
//	if (TUE_CML_Abs(fDivisor) < CML_f_Delta)
//	{
//		if (fDivisor < 0.F)
//		{
//			fDivisor = -CML_f_Delta;
//		}
//		else
//		{
//			fDivisor = CML_f_Delta;
//		}
//	}
//	return fDivisor;
//}

/*****************************************************************************
  Functionname: BSDCalculate_TTBreakthroughLine */ /*!

  @brief: Calculate the time to zone boundary crossing the zone boundaries are lines

  @description: Calculate the time to zone boundary crossing the zone boundaries are lines

  @param[in]:fZoneBoundary,zone boundary coordinate
             fObjectBoundary,object boundary coordinate
			 fObjectSpeed,object speed

  @return:fTime,current object TTB time
*****************************************************************************/
float32 BSDCalculate_TTBreakthroughLine(float32 fZoneBoundary,
                                        float32 fObjectBoundary,
                                        float32 fObjectSpeed) {
  float32 fTime = 0.F;
  fTime = (fZoneBoundary - fObjectBoundary) / SafeDiv(fObjectSpeed);
  /* Negative times are actually "very large" */
  if (fTime < 0.F) {
    fTime = F32_VALUE_INVALID;
  }
  return fTime;
}

/*****************************************************************************
  Functionname: BSDCalculate_TTBreakthroughCurve */ /*!

  @brief: Calculate the time zone boundary crossing if the zone boundaries are a curve

  @description:The time to exit for an object from a particular boundary pair,
               Negative times are reported as large positive

  @param[in]:uObj,object array index
             fObjXmin,Maximal object boundary in x-axis
			 fObjXmax,Minimal object boundary in x-axis
			 fObjY,Object boundary in y-axis which will leave the zone at latest(depend on sign of object speed)
			 fZoneY,Offset of zone in y-axis on the side on which the object will leave(depend on sign of object speed)
			 fObjectSpeed,Object speed in y direction
			 pBSDWarnParameter, pRoad

  @return:fTime,Time in seconds until the object leaves the zone over specified boundary
*****************************************************************************/
float32 BSDCalculate_TTBreakthroughCurve(
    uint8 uObj, float32 fObjXmin, float32 fObjXmax, float32 fObjY,
    float32 fZoneY, float32 fObjectSpeed,
    const BSDWarningParameter_t* pBSDWarnParameter, const BSDRoad_t* pRoad) {
  float32 fXTopEdge = 0.0F;
  float32 fXBottomEdge = 0.0F;
  BSD_Globals_t* pBSDGlobals = pGetBSDCalculatePointer_BSDGlobals();

  const boolean bBSDNHTSAActive = pBSDWarnParameter->bNCAPActive;
  BSD_Info_t* pBSDObj = pGetBSDCalculatePointer_ObjInfo(uObj);
  float32 fYBorderTopEdge = 0.0F;
  float32 fYBorderBottomEdge = 0.0F;
  float32 fRadius = pRoad->fDrivenCurveRadius_met;
  float32 fDist2LeaveTopEdge = 0.0F;
  float32 fDist2LeaveBottomEdge = 0.0F;
  float32 fDist2Leave = 0.0F;
  float32 fTime = 0.0F;
  /* Check upper edge */

  /* If the object is longer than the warning zone the x value should be limited
   * to the warning zone */
  fXTopEdge = TUE_CML_Min(pBSDGlobals->fBSDZoneXmax_met, fObjXmax);
  if (bBSDNHTSAActive == FALSE) {
    fXBottomEdge = TUE_CML_Max(pBSDGlobals->fBSDZoneXmin_met, fObjXmin);
  } else {
    fXBottomEdge = TUE_CML_Max(pBSDObj->fBSDZoneObjXmin_met, fObjXmin);
  }
  /* Calculate Y value of zone border */
  // Compensation curve y offset value
  fYBorderTopEdge = BSDCalculateDistToDrivenCurve(fXTopEdge, fRadius) + fZoneY;
  fYBorderBottomEdge =
      BSDCalculateDistToDrivenCurve(fXBottomEdge, fRadius) + fZoneY;

  // reduce obj relevant y distance to get the leave distance
  fDist2LeaveTopEdge = fYBorderTopEdge - fObjY;
  fDist2LeaveBottomEdge = fYBorderBottomEdge - fObjY;

  /* calculate overall distance */
  if (fObjectSpeed < 0.F) {
    /* if Vy < 0 the object will leave over inner boundary.-> negetive
     * Dist2Leave is corret */
    fDist2Leave = TUE_CML_Min(fDist2LeaveTopEdge, fDist2LeaveBottomEdge);
  } else {
    /* if Vy > 0 the object will leave over outer boundary.-> positive
     * Dist2Leave is correct */
    fDist2Leave = TUE_CML_Max(fDist2LeaveTopEdge, fDist2LeaveBottomEdge);
  }
  /* calculate time (check for safe division is done in calling function) */
  fTime = fDist2Leave / fObjectSpeed;

  /* negative times are actually "very large"  */
  if (fTime < 0.F) {
    fTime = F32_VALUE_INVALID;
  }
  return fTime;
}
