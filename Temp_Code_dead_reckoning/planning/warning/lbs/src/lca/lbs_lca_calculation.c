/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "lbs_lca_calculation.h"
#include "lbs_lca_par.h"
#include "planning/warning/lbs/common/tue_common_libs.h"

/*****************************************************************************
  VARIABLES
*****************************************************************************/

/*****************************************************************************
  MAIN FUNCTION
*****************************************************************************/

/*****************************************************************************
  Functionname: LCASetGlobals                                  */ /*!

  @brief:Set global warning parameter

  @description:Set global warning parameter

  @param[in]: pLCAGlobals -> The LCA internal global data pointer

  @param[out]:bLCAWarningLastCycle  the last cycle LCA warning status
  @param[out]:bLCAWarnActive        Reset global LCA warning stored in LCA globals
  @param[out]:uLCAWarningID_nu      the current cycle warning object id
  @param[out]:fXObjectWarning_met   the current cycle warning object distance x
  @param[out]:uNofFMObjects         the counter for the front mirror objects

  @return:void
*****************************************************************************/
void LCASetGlobals(LCAGlobals_st* pLCAGlobals) {
  /*Store internal global warning status from last cycle*/
  pLCAGlobals->LCAWarnInfo.bLCAWarningLastCycle =
      pLCAGlobals->LCAWarnInfo.bLCAWarnActive;

  /*Reset global LCA warning stored in LCA globals*/
  pLCAGlobals->LCAWarnInfo.bLCAWarnActive = FALSE;

  /*Reset global information about the warning object*/
  pLCAGlobals->LCAWarnInfo.uLCAWarningID_nu = TUE_C_UI8_VALUE_INVALID;
  pLCAGlobals->LCAWarnInfo.fXObjectWarning_met = -TUE_C_F32_VALUE_INVALID;

  /*Reset counter for the front mirror objects*/
  pLCAGlobals->LCAFrontMirror.uNofFMObjects = 0U;
}

/*****************************************************************************
  Functionname: LCASetParamter                                  */ /*!

  @brief:Set the LCA parameter based on the parameter input

  @description:Set the LCA parameter based on the parameter input

  @param[in]: params              -> the LCA parameter input pointer
  @param[in]: pLCAGlobals         -> the LCA Global data pointer

  @param[out]:fLCARangeMax_met    ->  Maximum LCA range parameter
  @param[out]:fLCACurveRadMax_met ->  Maximum LCA curve radius parameter
  @return:void
*****************************************************************************/
void LCASetParamter(const LCAParam_st* params, LCAGlobals_st* pLCAGlobals) {
  /*Check if the range and curve parameter are plausible*/
  /*fMaxLCARange is larger than LCA_RANGE_MIN           */
  /*fMaxLCACurveRadius is larger than LCA_CURVE_RAD_MIN */
  /*fMaxLCARange > 7.0m and fMaxLCACurveRadius_met > 25.0m*/
  if ((params->fMaxLCARange_met > LCA_RANGE_MIN)                // 7
      && (params->fMaxLCACurveRadius_met > LCA_CURVE_RAD_MIN))  // 25
  {
    pLCAGlobals->LCAConfig.fLCARangeMax_met = params->fMaxLCARange_met;
    pLCAGlobals->LCAConfig.fLCACurveRadMax_met = params->fMaxLCACurveRadius_met;
  } else {
    pLCAGlobals->LCAConfig.fLCARangeMax_met = LCA_RANGE_MAX_DEFAULT; /* 60m */
    pLCAGlobals->LCAConfig.fLCACurveRadMax_met =
        LCA_CURVE_RAD_MAX_DEFAULT; /* 250m */
  }
}

/*****************************************************************************
  Functionname: LCASetTTCParamter                                  */ /*!

  @brief:Sets the LCA TTC Matrix Parameter

  @description:Sets the LCA TTC Matrix Parameter

  @param[in]: params                  input parameters pointer for LCA function
  @param[in]: uLCAWarningDurationCfg  the LCA Warning TTC configuration flag
  @param[in]: bBSDWarning             the BSD warning flag
  @param[in]: bLCAWarnActive          the LCA warning flag

  @param[out]: fTTCThreshVrelLow_s   LCA TTC threshold for low relative speed
  @param[out]: fTTCThreshVrelMid_s   LCA TTC threshold for mid relative speed
  @param[out]: fTTCThreshVrelHigh_s  LCA TTC threshold for high relative speed

  @return:void
*****************************************************************************/
void LCASetTTCParamter(const LCAParam_st* params,
                       const LCALBSInputInfo_st* pLBSInfo,
                       LCAGlobals_st* pLCAGlobals) {
  pLCAGlobals->LCAConfig.uLCAWarningDurationCfg = params->uLCAWarningDuration;
  pLCAGlobals->LCAConfig.fTTCHysteresis_s = params->fMinTTCHysteresis_s;

  /*Check which TTC setting should be applied*/
  if (params->uLCAWarningDuration == (uint8)LCA_WARN_CFG_OFF) {
    /*if the warning configuration is set to off,the TTC value are set to -1*/
    pLCAGlobals->LCAConfig.fTTCThreshVrelLow_s = -1.0F;
    pLCAGlobals->LCAConfig.fTTCThreshVrelMid_s = -1.0F;
    pLCAGlobals->LCAConfig.fTTCThreshVrelHigh_s = -1.0F;
  } else {
    if (params->uLCAWarningDuration == (uint8)LCA_WARN_CFG_CUSTOM) {
      /*if the warning configuration is set to custom,the single TTC Threshold
       * is applied*/
      pLCAGlobals->LCAConfig.fTTCThreshVrelLow_s = params->fTTCThreshold_s;
      pLCAGlobals->LCAConfig.fTTCThreshVrelMid_s = params->fTTCThreshold_s;
      pLCAGlobals->LCAConfig.fTTCThreshVrelHigh_s = params->fTTCThreshold_s;
    } else {
      /*If another warning configuration setting is set,the relative speed
       * dependent TTC threshold value are applied*/
      pLCAGlobals->LCAConfig.fTTCThreshVrelLow_s =
          params->fTTCThreshLowRelSpeed_s;
      pLCAGlobals->LCAConfig.fTTCThreshVrelMid_s =
          params->fTTCThreshMidRelSpeed_s;
      pLCAGlobals->LCAConfig.fTTCThreshVrelHigh_s =
          params->fTTCThreshHighRelSpeed_s;
    }
  }

  /*Check if the TTC threshold is set to a valid value greater than zero*/
  if (pLCAGlobals->LCAConfig.fTTCThreshVrelLow_s > TUE_C_F32_DELTA) {
    /*Check if either the LCA or BSD warning where switched on in the last
     * cycle*/
    if ((pLBSInfo->LBSWarningLastCycle.bLCAWarningLastCycle) ||
        (pLBSInfo->LBSWarningLastCycle.bBSDWarningLastCycle)) {
      /*Add the additional BridgeWarningTime which bridges the warning for
       * consecutive objects*/
      pLCAGlobals->LCAConfig.fTTCThreshVrelLow_s +=
          params->fBridgeWarningTime_s;
      pLCAGlobals->LCAConfig.fTTCThreshVrelMid_s +=
          params->fBridgeWarningTime_s;
      pLCAGlobals->LCAConfig.fTTCThreshVrelHigh_s +=
          params->fBridgeWarningTime_s;
    }
  }
}

/*****************************************************************************
  Functionname: LCACalculateRangeLimitation                                  */ /*!

  @brief: Calculates the range limitation applied to LCA

  @description: Calculates the range limitation applied to LCA

  @param[in]: fEgoSpeed           Longitudinal ego velocity
  @param[in]: fCurveRadius        Radius of curves
  @param[in]: fDriverCurveRadius  Radius of driven curve

  @param[out]: fLCARange          Range of LCA function in meters

  @return:void
*****************************************************************************/
void LCACalculateRangeLimitation(const LCARoad_t* pRoad,
                                 const LCAVehicleInfo_t* pEgoInfo,
                                 LCAGlobals_st* pLCAGlobals) {
  const float32 fDrivenCurveRadius = fABS(pRoad->fDrivenCurveRadius_met);
  const float32 fCurveRadius = fABS(pRoad->fCurveRadius_met);
  const float32 fEgoSpeed = pEgoInfo->fegoVelocity_mps;
  float32 fCurveRadiusMin = 0.001F;
  float32 fLCARangeByCurves = 0.F;
  float32 fLCARangeByEgoSpeed = 0.F;
  float32 fLCARangeTmp = 0.F;

  /*The the minimum value from the driven curve radius and the forward curve
   * radius for the LCA range limitation*/
  fCurveRadiusMin = TUE_CML_Min(fDrivenCurveRadius, fCurveRadius);

  /*Calculate the range limitation based on the current minimum curve radius*/
  /*CurveRadius 25-250m(default) -> LCARange 7-60m(default)*/
  /*It means the LCA range will shorter when ego or road curve radius smaller*/
  fLCARangeByCurves = TUE_CML_BoundedLinInterpol2(
      fCurveRadiusMin,
      LCA_CURVE_RAD_MIN,                           // 25
      pLCAGlobals->LCAConfig.fLCACurveRadMax_met,  //250
      LCA_RANGE_MIN,                               // 7
      pLCAGlobals->LCAConfig.fLCARangeMax_met);    //71

  /*if ego car in low speed(<10.8 km/h) condition, the LCA range will shorter
   * faster when egospeed slow*/
  /* ego speed 0 - 3m/s ->LCARange 10 - 20m(default)*/
  if (fEgoSpeed < LCA_RANGE_LIM_EGOSPEED_LOW)  // 30/3.6
  {
    fLCARangeByEgoSpeed =
        TUE_CML_BoundedLinInterpol2(fEgoSpeed, 0.0F,
                                    LCA_RANGE_LIM_EGOSPEED_LOW,  //  30/3.6
                                    LCA_LI_MIN_RANGETMPSPEED,    // 5
                                    LCA_LI_MAX_RANGETMPSPEED);   // 10
  } else {
    fLCARangeByEgoSpeed = pLCAGlobals->LCAConfig.fLCARangeMax_met;
  }

  fLCARangeTmp = TUE_CML_Min(fLCARangeByCurves, fLCARangeByEgoSpeed);

  /*filter LCA range,slower up and faster down to avoid false warning*/
  if (fLCARangeTmp <= pLCAGlobals->fLCARange) {
    TUE_CML_LowPassFilter(&pLCAGlobals->fLCARange, fLCARangeTmp,
                          LCA_RANGE_FILTER_DOWN);  // 0.1
  } else {
    TUE_CML_LowPassFilter(&pLCAGlobals->fLCARange, fLCARangeTmp,
                          LCA_RANGE_FILTER_UP);  // 0.05
  }
}

/*****************************************************************************
  Functionname: LCACheckStableMirringObject                                  */ /*!

  @brief:Check if mirroring object exists and establish Front Mirror Vx interval

  @description:Check if there is a stable mirroring object on own lane and adjacent
               lane,Pick closest ones.Establish the Front Mirror relative speed
			   interval for potential mirrors that are stationary(traffic signs).
			   This interval is around FMVx = EgoSpeed + Absolute speed of mirroring
			   target,or FMVx = (EgpSpeed*2) + VRelX of mirroring target,The interval
			   will be [FMVx - hysteresis Vx,FMVx + hysteresis Vx]
			   Multiple sub functions are called.

  @param[in]:pGenObjList   The GenObjList input data pointer
  @param[in]:pLCAGlobals   The LCA internal global data pointer
  @param[in]:pRoad         The EM Road information  data pointer
  @param[in]:pEgoInfo      The LCA ego velocity information input data pointer
  @param[in]:pLCACal       The LCA internal Calculate data pointer
  
  @param[out]: uObjIdxClosestStableObjOwnLane  Array index of closest stable object on own lane
  @param[out]: uObjIdxClosestStableObjAdjLane  Array index of closest stable object on adjacent lane
  @param[out]: pLCAGlobals->LCAFrontMirror  Global structure containing Front Mirror relative speed interval

  @return:void
*****************************************************************************/
void LCACheckStableMirringObject(const LCAGenObjList_st* pGenObjList,
                                 const LCALBSObjInfo_Array* pLBSObj,
                                 const LCASIObjInfo_Array* pSIObjList,
                                 const LCALBSInputInfo_st* pLBSInfo,
                                 const LCARoad_t* pRoad,
                                 const LCAVehicleInfo_t* pEgoInfo,
                                 LCACalculate_st* pLCACal) {
  float32 fVxCloseStableObjAdjLane = TUE_C_F32_VALUE_INVALID;
  float32 fVxCloseStableObjOwnLane = TUE_C_F32_VALUE_INVALID;
  sint32 uObjIdxClosestStableObjAdjLane = TUE_C_UI8_VALUE_INVALID;
  sint32 uObjIdxClosestStableObjOwnLane = TUE_C_UI8_VALUE_INVALID;
  sint32 uObjIdxAdjLane1 = TUE_C_UI8_VALUE_INVALID;
  sint32 uObjIdxAdjLane2 = TUE_C_UI8_VALUE_INVALID;
  sint32 uObjIdxOwnLane1 = TUE_C_UI8_VALUE_INVALID;
  sint32 uObjIdxOwnLane2 = TUE_C_UI8_VALUE_INVALID;
  const LCAGenObjInfo_t* pGenObj1 = NULL;
  const LCAGenObjInfo_t* pGenObj2 = NULL;
  const LCALBSObjInfo_t* pLBSObj1 = NULL;
  const LCALBSObjInfo_t* pLBSObj2 = NULL;

  LCAGlobals_st* pLCAGlobals = &pLCACal->LCAGlobals;

  /*Calculate the additional Vx threshold which is depending on ego speed
   * ,acceleration and curve radius*/
  LCACalculateAddVxThresh(pRoad, pEgoInfo, pLCAGlobals);

  /*Get the IDs of the stable objects behind the subject vehicle*/
  /*Two objects on adjacent lane and two objects on own lane*/
  LCASearchClosestStableObjects(pGenObjList, pLBSInfo, pEgoInfo, pLCACal,
                                &uObjIdxOwnLane1, &uObjIdxOwnLane2,
                                &uObjIdxAdjLane1, &uObjIdxAdjLane2);

  /*Check if an object is available on the adjacent lane and update closest
   * object id*/
  LCASelectClosestStableObjects(pGenObjList, pLBSInfo, uObjIdxAdjLane1,
                                uObjIdxAdjLane2,
                                &uObjIdxClosestStableObjAdjLane);
  /*Check if an object is available on the own lane and update closest object
   * id*/
  LCASelectClosestStableObjects(pGenObjList, pLBSInfo, uObjIdxOwnLane1,
                                uObjIdxOwnLane2,
                                &uObjIdxClosestStableObjOwnLane);

  /*Compares the closest stable objects found in the current cycle */
  /* against the ones found in the last cycle*/
  // LCACompareClosestStableObjects(pGenObjList, pLBSObj, pLCAGlobals,
  //                                &uObjIdxClosestStableObjOwnLane,
  //                                &uObjIdxClosestStableObjAdjLane);

  /*Update the global closest object id and velocity range in the own
   * lane,adjacent lane*/
  LCAUpdateClosestStableObjectsInfo(pGenObjList, pEgoInfo, pLCAGlobals,
                                    uObjIdxClosestStableObjOwnLane,
                                    uObjIdxClosestStableObjAdjLane);
}

/*****************************************************************************
  Functionname: LCACalculateAddVxThresh                                  */ /*!

  @brief:Calculates the additional vx threshold for front mirror detection

  @description: Calculates the additional vx threshold for front mirror detection

  @param[in]:fEgoSpeed    Longitudinal ego velocity
  @param[in]:fEgoAccel    Longitudinal ego acceleration
  @param[in]:fCurveRadius Radius of curve
  @param[in]:fFMObjRate   the object FM update rate

  @param[out]:fVxThreshAdd Additional VX threshold to be used for Front mirror detection

  @return:void
*********************** ******************************************************/
void LCACalculateAddVxThresh(const LCARoad_t* pRoad,
                             const LCAVehicleInfo_t* pEgoInfo,
                             LCAGlobals_st* pLCAGlobals) {
  float32 fCurveRadiusAbs = TUE_CML_Min(fABS(pRoad->fDrivenCurveRadius_met),
                                        fABS(pRoad->fCurveRadius_met));
  float32 fEgoSpeed = fABS(pEgoInfo->fegoVelocity_mps);
  float32 fEgoAccel = fABS(pEgoInfo->fegoAcceleration_mps2);
  float32 fFMObjRate = pLCAGlobals->LCAFrontMirror.fFMObjRate;

  float32 fVXThreshAddOwnSpeed = 0.F;
  float32 fVXThreshAddCurve = 0.F;
  float32 fVXThreshAddEgoAccel = 0.F;
  float32 fVXThreshAddFMObjRate = 0.F;
  float32 fVXThreshAdd = 0.F;

  /*Additional VX threshold offset depending on ego speed*/
  /*EgoSpeed 0 ~ 40 m/s  ->  AddOwnSpeed -0.5 ~ 0.5 m/s */
  fVXThreshAddOwnSpeed =
      TUE_CML_BoundedLinInterpol2(fEgoSpeed,
                                  LCA_LI_FM_ADDVXSPEED_MIN_EGOSPEED,  // 0
                                  LCA_LI_FM_ADDVXSPEED_MAX_EGOSPEED,  // 40
                                  LCA_LI_FM_ADDVXSPEED_MIN_THRESH,    //-0.5
                                  LCA_LI_FM_ADDVXSPEED_MAX_THRESH);   // 0.5

  /*Additional VX threshold offset depending on the curve radius*/
  /*CurveRadius 50 ~ 100 m  ->  AddCurve 1.5 ~ 0 m/s */
  fVXThreshAddCurve = TUE_CML_BoundedLinInterpol2(
      fCurveRadiusAbs, LCA_LI_FM_ADDVXCURVE_MIN_CURVE,
      LCA_LI_FM_ADDVXCURVE_MAX_CURVE, LCA_LI_FM_ADDVXCURVE_MIN_THRESH,
      LCA_LI_FM_ADDVXCURVE_MAX_THRESH);

  /*Only ego speed > 32.4 km/h use the accel add */
  if (fEgoSpeed > LCA_FM_ADD_VXACCEL_EGOSPEED_MIN) {
    /*ego Acceleration 0 ~ 2 m/s^2  ->  AddAceel 0 ~ 0.6 m/s */
    fVXThreshAddEgoAccel = TUE_CML_BoundedLinInterpol2(
        fEgoAccel, LCA_LI_FM_ADDVXACCEL_MIN_ACCEL,
        LCA_LI_FM_ADDVXACCEL_MAX_ACCEL, LCA_LI_FM_ADDVXACCEL_MIN_THRESH,
        LCA_LI_FM_ADDVXACCEL_MAX_THRESH);
  } else {
    fVXThreshAddEgoAccel = 0.0F;
  }

  /*Additional VX threshold offset based on the current amount of front mirror
   * objects*/
  if (fFMObjRate > LCA_FM_ADD_VXFMRATE_FMRATE_MIN)  // 1.5
  {
    /*Obj rate 1.5 ~ 5   ->  Add 1.0 ~ 2.0 m/s */
    fVXThreshAddFMObjRate = TUE_CML_BoundedLinInterpol2(
        fFMObjRate, LCA_LI_FM_ADDVXFMRATE_MIN_FMRATE,
        LCA_LI_FM_ADDVXFMRATE_MAX_FMRATE, LCA_LI_FM_ADDVXFMRATE_MIN_THRESH,
        LCA_LI_FM_ADDVXFMRATE_MAX_THRESH);

    /*Factor-in ego speed for additional VX threshold based on FM object rate*/
    /*ego speed 5.0 ~ 10.0 m/s   ->  factor 0.0 ~ 1.0 */
    fVXThreshAddFMObjRate *= TUE_CML_BoundedLinInterpol2(
        fEgoSpeed, LCA_MIN_EGOSPEED_FM_DETECTION,
        (2.0F * LCA_MIN_EGOSPEED_FM_DETECTION), 0.0F, 1.0F);
  } else {
    fVXThreshAddFMObjRate = 0.0F;
  }

  /*Sum up the predefined VX threshold offset with the ones calculated based on
   * ego speed and curve radius*/
  fVXThreshAdd = LCA_FRONT_MIRROR_VX_THRESH_OFFSET + fVXThreshAddOwnSpeed +
                 fVXThreshAddCurve + fVXThreshAddEgoAccel +
                 fVXThreshAddFMObjRate;  // 0.5

  /* Low pass filter the calculated VX threshold offset*/
  if (fVXThreshAdd > pLCAGlobals->LCAFrontMirror.LCA_Vf_VxThreshAdd_mps) {
    TUE_CML_LowPassFilter(&pLCAGlobals->LCAFrontMirror.LCA_Vf_VxThreshAdd_mps,
                          fVXThreshAdd, LCA_ADDVXTHRESH_FILTER_UP);  // 0.05
  } else {
    TUE_CML_LowPassFilter(&pLCAGlobals->LCAFrontMirror.LCA_Vf_VxThreshAdd_mps,
                          fVXThreshAdd, LCA_ADDVXTHRESH_FILTER_DOWN);  // 0.01
  }

  /*Limit the additional VX threshold offset to 0.25 * egospeed */
  pLCAGlobals->LCAFrontMirror.LCA_Vf_VxThreshAdd_mps =
      TUE_CML_Min(pLCAGlobals->LCAFrontMirror.LCA_Vf_VxThreshAdd_mps,
                  (fEgoSpeed * LCA_FM_LIMITVATHRESH_EGOSPEED_FACTOR));  // 0.25
}

/*****************************************************************************
  Functionname: LCASearchClosestStableObjects */ /*!

  @brief:Search for the closest stable objects on own lane and adjacent lane

  @description:Search for the closest stable objects on own lane and adjacent
			   lane for calculation of the VX thresholds for front mirror
			   detection

  @param[in]:fegoVelocity_mps                the ego vehicle longitudinal speed
  @param[in]:fXObj                           the object longitudinal distance
  @param[in]:fVxAbsObj                       the object longitudinal abs velocity
  @param[in]:pSIObj->eAssociatedLane         the object lane association status from SI
  @param[in]:pLBSObj->fUpdateRate_nu         the object update rate from LBS
  @param[in]:pLCAObj->fBehindGrdProb_per     the object behind grd probability(miss data)
  @param[in]:pLCAObj->bLCAMirrorFrontObject  the object mirrior flag last cycle

  @param[out]:puObjIdxOwnLane1  Object index of the nearest stable object on the own lane
  @param[out]:puObjIdxOwnLane2  Object index of the second nearest stable object on the own lane
  @param[out]:puObjIdxAdjLane1  Object index of the nearest stable object on the adjacent lane
  @param[out]:puObjIdxAdjLane2  Object index of the second nearest stable object on the adjacent lane

  @return:void
*****************************************************************************/
void LCASearchClosestStableObjects(
    const LCAGenObjList_st* pGenObjList, const LCALBSInputInfo_st* pLBSInfo,
    const LCAVehicleInfo_t* pEgoInfo, LCACalculate_st* pLCACal,
    sint32* puObjIdxOwnLane1, sint32* puObjIdxOwnLane2,
    sint32* puObjIdxAdjLane1, sint32* puObjIdxAdjLane2) {
  sint32 uObj = 0;
  const LCAGenObjInfo_t* pGenObj = NULL;
  const LCALBSObjInfo_t* pLBSObj = NULL;
  const LCAObjInfo_t* pLCAObj = NULL;
  const LCASIObjInfo_t* pSIObj = NULL;
  const LCAGlobals_st* pLCAGlobals = &pLCACal->LCAGlobals;

  float32 fXObjAdjLane1 = -TUE_C_F32_VALUE_INVALID;
  float32 fXObjAdjLane2 = -TUE_C_F32_VALUE_INVALID;
  float32 fXObjOwnLane1 = -TUE_C_F32_VALUE_INVALID;
  float32 fXObjOwnLane2 = -TUE_C_F32_VALUE_INVALID;
  boolean bStableObjPreviosCycle = FALSE;
  boolean bKinematicConditions = FALSE;

  /*Search for stable objects behind us and globally store their ID such that
  distance and velocity parameters of this object can be accessed later on*/
  for (uObj = 0; uObj < LCA_INPUT_OBJECT_NUMBER; uObj++) {
    if (!bGetLCAGenObject_GenObjIsDeleted(uObj, pGenObjList)) {
      pGenObj = pGetLCAGenObject_GenObj(uObj, pGenObjList);
      // pLBSObj = pGetLCAGenObject_LBSObj(uObj, pGenObjList);
      pLBSObj = &pLBSInfo->LBSObjInfoList[uObj];
      // pSIObj = pGetLCAGenObject_SIObj(uObj, pGenObjList);
      pLCAObj = pGetLCACalculatePointer_LCAObjInfo(uObj);
      // pSIObj = &pSIObjList[uObj];
      pSIObj = &pLBSInfo->SIObjInfoList[uObj];
      float32 fXObj = pGenObj->fDistX_met;
      float32 fVxAbsObj = fABS(pGenObj->fVrelX_mps);

      bStableObjPreviosCycle = FALSE;
      if ((uObj == pLCAGlobals->LCAFrontMirror.uClosetStableObjIDOwnLane) ||
          (uObj == pLCAGlobals->LCAFrontMirror.uClosetStableObjIDAdjLane)) {
        bStableObjPreviosCycle = TRUE;
      }

      bKinematicConditions = FALSE;
      if ((fVxAbsObj < (LCA_FM_STBLOBJ_MAX_FACTOR_VRELXABS *
                        pEgoInfo->fegoVelocity_mps))    // 0.6
          && (fVxAbsObj < LCA_FM_STBLOBJ_MAX_VRELXABS)  // 10
          && (fXObj > LCA_FM_STBLOBJ_DISTX_MIN)         //-80
          && ((fXObj < LCA_FM_STBLOBJ_DISTX_MAX) ||
              (bStableObjPreviosCycle &&
               (fXObj < LCA_FM_STBLOBJ_ACTIVE_DISTX_MAX))))  //-5    -1
      {
        bKinematicConditions = TRUE;
      }

      if ((pLCAObj->bLCAMirrorFrontObject == FALSE) &&
          (bKinematicConditions == TRUE) &&
          (pLBSObj->fUpdateRate_nu > LCA_FM_STBLOBJ_UPDRT_MIN)  // 0.9
          &&
          (pLCAObj->fBehindGrdProb_per < LCA_FM_STBLOBJ_BEHINDGRD_MAX))  // 0.3
      {
        /*Store the first and second nearest object that fulfill the quality and
         * stability conditions*/
        /*Treat own lane and adjacent lane objects separately*/
        switch (pSIObj->eAssociatedLane) {
          case LCA_SI_ASSOC_LANE_LEFT:
            /*Check for stable objects on the left lane*/
            if (fXObj > fXObjAdjLane1) {
              /*Store the nearest object on the adjacent lane*/
              *puObjIdxAdjLane1 = uObj;
              fXObjAdjLane1 = fXObj;
            } else {
              if (fXObj > fXObjAdjLane2) {
                /*Store the second nearest object on the adjacent lane*/
                *puObjIdxAdjLane2 = uObj;
                fXObjAdjLane2 = fXObj;
              }
            }
            break;
          case LCA_SI_ASSOC_LANE_EGO:
            /*Check for stable objects on the own lane*/
            if (fXObj > fXObjOwnLane1) {
              /*Store the nearest object on the own lane lane*/
              *puObjIdxOwnLane1 = uObj;
              fXObjOwnLane1 = fXObj;
            } else {
              if (fXObj > fXObjOwnLane2) {
                /*Store the second nearest object on the own lane lane*/
                *puObjIdxOwnLane2 = uObj;
                fXObjOwnLane2 = fXObj;
              }
            }
            break;
          default:
            break;
        }
      }
    }
  }
}

/*****************************************************************************
  Functionname: LCASelectClosestStableObjects */ /*!

  @brief:Selection one of the stable object

  @description:Selection one of the stable object depend on distance,RCS and updateRate

  @param[in]:uObjIdxLane1       the closest object id
  @param[in]:uObjIdxLane2       the second closest object id
  @param[in]:fRCS               the object radar RCS property
  @param[in]:fUpdateRate_nu     the object measurement update rate
  @param[in]:fDistX_met         the object longitudinal distance

  @param[out]:puObjIdxClosestStableObj   the available and most closest object id

  @return:void
*****************************************************************************/
void LCASelectClosestStableObjects(const LCAGenObjList_st* pGenObjList,
                                   const LCALBSInputInfo_st* pLBSInfo,
                                   sint32 uObjIdxLane1, sint32 uObjIdxLane2,
                                   sint32* puObjIdxClosestStableObj) {
  const LCAGenObjInfo_t* pGenObj1 = NULL;
  const LCAGenObjInfo_t* pGenObj2 = NULL;
  const LCALBSObjInfo_t* pLBSObj1 = NULL;
  const LCALBSObjInfo_t* pLBSObj2 = NULL;
  float32 fDistXDiff = 0.F;

  if (uObjIdxLane1 < LCA_INPUT_OBJECT_NUMBER) {
    /*Check if a second object was observed on the adjacent lane */
    /*and compare both objects in terms of update quality and RCS*/
    if (uObjIdxLane2 < LCA_INPUT_OBJECT_NUMBER) {
      pGenObj1 = pGetLCAGenObject_GenObj(uObjIdxLane1, pGenObjList);
      pGenObj2 = pGetLCAGenObject_GenObj(uObjIdxLane2, pGenObjList);
      // pLBSObj1 = pGetLCAGenObject_LBSObj(uObjIdxLane1, pGenObjList);
      // pLBSObj2 = pGetLCAGenObject_LBSObj(uObjIdxLane2, pGenObjList);
      pLBSObj1 = &pLBSInfo->LBSObjInfoList[uObjIdxLane1];
      pLBSObj2 = &pLBSInfo->LBSObjInfoList[uObjIdxLane2];

      const float32 fRCSObjAdj1 = pGenObj1->fRCS;
      const float32 fRCSObjAdj2 = pGenObj2->fRCS;
      fDistXDiff = pGenObj1->fDistX_met - pGenObj2->fDistX_met;
      if ((fRCSObjAdj2 > (fRCSObjAdj1 + LCA_FM_STABLEOBJ_RCS_ADDTHRESH))  // 10
          && (pLBSObj2->fUpdateRate_nu > pLBSObj1->fUpdateRate_nu) &&
          (fDistXDiff < LCA_FM_STABLEOBJ_XDIFF_THRESH))  // 20
      {
        /*If the object at larger distance has better quality and RCS then store
         * that one as the stable object*/
        *puObjIdxClosestStableObj = uObjIdxLane2;
      } else {
        /*Store the closest observed object as the stable object*/
        *puObjIdxClosestStableObj = uObjIdxLane1;
      }
    } else {
      /*Store the closest observed object as the stable object*/
      *puObjIdxClosestStableObj = uObjIdxLane1;
    }
  }
}

/*****************************************************************************
  Functionname: LCACompareClosestStableObjects */ /*!

  @brief:Compares the closest stable objects found in the current cycle against
		 the ones found in the last cycle from the last cycle

  @description:Compares the closest stable objects found in the current cycle against
			   the ones found in the last cycle from the last cycle

  @param[in]:uObjAdjLaneLastCycle  index of the nearest object adjacent lane last cycle
  @param[in]:uObjOwnLaneLastCycle  index of the nearest object own lane last cycle
  @param[in]:uObjAdjLaneCurrCycle  index of the nearest object adjacent lane current cycle
  @param[in]:uObjOwnLaneCurrCycle  index of the nearest object own lane current cycle
  @param[in]:fXObjLastCycle        nearest object distance x last cycle
  @param[in]:fVxObjLastCycle       nearest object velocity x last cycle
  @param[in]:fUpdateObjLastCycle   nearest object measurement update rate last cycle
  @param[in]:fXObjCurrCycle        nearest object distance x current cycle
  @param[in]:fVxObjCurrCycle       nearest object velocity x current cycle
  @param[in]:fUpdateObjCurrCycle   nearest object measurement update rate current cycle

  @param[out]:puObjIdxClosestStableObjOwnLane  Object index of the nearest stable object on the own lane
  @param[out]:puObjIdxClosestStableObjAdjLane  Object index of the nearest stable object on the adjacent lane

  @return:void
*****************************************************************************/
void LCACompareClosestStableObjects(const LCAGenObjList_st* pGenObjList,
                                    const LCALBSObjInfo_Array* pLBSObj,
                                    LCAGlobals_st* pLCAGlobals,
                                    uint8* puObjIdxClosestStableObjOwnLane,
                                    uint8* puObjIdxClosestStableObjAdjLane) {
  float32 fXObjLastCycle = 0.F;
  float32 fVxObjLastCycle = 0.F;
  float32 fUpdateObjLastCycle = 0.F;
  float32 fXObjCurrCycle = 0.F;
  float32 fVxObjCurrCycle = 0.F;
  float32 fUpdateObjCurrCycle = 0.F;
  uint8 uObjAdjLaneLastCycle =
      pLCAGlobals->LCAFrontMirror.uClosetStableObjIDAdjLane;
  uint8 uObjOwnLaneLastCycle =
      pLCAGlobals->LCAFrontMirror.uClosetStableObjIDOwnLane;
  uint8 uObjAdjLaneCurrCycle = *puObjIdxClosestStableObjAdjLane;
  uint8 uObjOwnLaneCurrCycle = *puObjIdxClosestStableObjOwnLane;

  const LCAGenObjInfo_t* pGenObjLastCycle = NULL;
  const LCAGenObjInfo_t* pGenObjCurrCycle = NULL;
  const LCALBSObjInfo_t* pLBSObjLastCycle = NULL;
  const LCALBSObjInfo_t* pLBSObjCurrCycle = NULL;

  /*Compare the stable object on the own lane found in the current cycle*/
  /*against the one found in the last cycle in case both are available*/
  if ((uObjOwnLaneLastCycle < LCA_INPUT_OBJECT_NUMBER) &&
      (uObjOwnLaneCurrCycle < LCA_INPUT_OBJECT_NUMBER)) {
    pGenObjLastCycle =
        pGetLCAGenObject_GenObj(uObjOwnLaneLastCycle, pGenObjList);
    pGenObjCurrCycle =
        pGetLCAGenObject_GenObj(uObjOwnLaneCurrCycle, pGenObjList);
    // pLBSObjLastCycle = pGetLCAGenObject_LBSObj(uObjOwnLaneLastCycle,
    // pGenObjList); pLBSObjCurrCycle =
    // pGetLCAGenObject_LBSObj(uObjOwnLaneCurrCycle, pGenObjList);
    pLBSObjLastCycle = pLBSObj[uObjOwnLaneLastCycle];
    pLBSObjCurrCycle = pLBSObj[uObjOwnLaneCurrCycle];
    if ((!bGetLCAGenObject_GenObjIsDeleted(uObjOwnLaneLastCycle,
                                           pGenObjList)) &&
        (uObjOwnLaneLastCycle != uObjOwnLaneCurrCycle)) {
      fXObjLastCycle = pGenObjLastCycle->fDistX_met;
      fVxObjLastCycle = pGenObjLastCycle->fVrelX_mps;
      fUpdateObjLastCycle = pLBSObjLastCycle->fUpdateRate_nu;
      fXObjCurrCycle = pGenObjCurrCycle->fDistX_met;
      fVxObjCurrCycle = pGenObjCurrCycle->fVrelX_mps;
      // fUpdateObjCurrCycle = pLBSObjCurrCycle->fUpdateRate_nu;

      /*Comparison based on*/
      /* -x position difference*/
      /* -Velocity x difference*/
      /* -update rate difference*/
      if ((fXObjLastCycle >
           (fXObjCurrCycle - LCA_CLOSE_STBLOBJ_DISTX_ADDTHRESH))  // 3
          && (fVxObjLastCycle <
              (fVxObjCurrCycle - LCA_CLOSE_STBLOBJ_VRELX_ADDTHRESH)) &&  // 3
          (fUpdateObjLastCycle >
           (fUpdateObjLastCycle -
            LCA_CLOSE_STBLOBJ_UPDRT_ADDTHRESH))) {  // 0.15
        *puObjIdxClosestStableObjOwnLane = uObjOwnLaneLastCycle;
      }
    }
  }

  /*Compare the stable object on the adjacent lane found in the current cycle*/
  /*against the one found in the last cycle in case both are available*/
  if ((uObjAdjLaneLastCycle < LCA_INPUT_OBJECT_NUMBER) &&
      (uObjAdjLaneCurrCycle < LCA_INPUT_OBJECT_NUMBER)) {
    pGenObjLastCycle =
        pGetLCAGenObject_GenObj(uObjAdjLaneLastCycle, pGenObjList);
    pGenObjCurrCycle =
        pGetLCAGenObject_GenObj(uObjAdjLaneCurrCycle, pGenObjList);
    // pLBSObjLastCycle = pGetLCAGenObject_LBSObj(uObjAdjLaneLastCycle,
    // pGenObjList); pLBSObjCurrCycle =
    // pGetLCAGenObject_LBSObj(uObjAdjLaneCurrCycle, pGenObjList);
    pLBSObjLastCycle = pLBSObj[uObjAdjLaneLastCycle];
    pLBSObjCurrCycle = pLBSObj[uObjAdjLaneCurrCycle];
    if ((!bGetLCAGenObject_GenObjIsDeleted(uObjAdjLaneLastCycle,
                                           pGenObjList)) &&
        (uObjAdjLaneLastCycle != uObjAdjLaneCurrCycle)) {
      fXObjLastCycle = pGenObjLastCycle->fDistX_met;
      fVxObjLastCycle = pGenObjLastCycle->fVrelX_mps;
      fUpdateObjLastCycle = pLBSObjLastCycle->fUpdateRate_nu;
      fXObjCurrCycle = pGenObjCurrCycle->fDistX_met;
      fVxObjCurrCycle = pGenObjCurrCycle->fVrelX_mps;
      // fUpdateObjCurrCycle = pLBSObjCurrCycle->fUpdateRate_nu;

      /*Comparison based on*/
      /* -x position difference*/
      /* -Velocity x difference*/
      /* -update rate difference*/
      if ((fXObjLastCycle >
           (fXObjCurrCycle - LCA_CLOSE_STBLOBJ_DISTX_ADDTHRESH)) &&
          (fVxObjLastCycle <
           (fVxObjCurrCycle - LCA_CLOSE_STBLOBJ_VRELX_ADDTHRESH)) &&
          (fUpdateObjLastCycle >
           (fUpdateObjLastCycle - LCA_CLOSE_STBLOBJ_UPDRT_ADDTHRESH))) {
        *puObjIdxClosestStableObjAdjLane = uObjAdjLaneLastCycle;
      }
    }
  }
}

/*****************************************************************************
  Functionname: LCAUpdateClosestStableObjectsInfo */ /*!

  @brief:Calculate mirror object velocity range to filter and store

  @description:Calculate mirror object velocity range to filter and store

  @param[in]:

  @return:void
*****************************************************************************/
void LCAUpdateClosestStableObjectsInfo(const LCAGenObjList_st* pGenObjList,
                                       const LCAVehicleInfo_t* pEgoInfo,
                                       LCAGlobals_st* pLCAGlobals,
                                       uint8 uObjIdxClosestStableObjOwnLane,
                                       uint8 uObjIdxClosestStableObjAdjLane) {
  const LCAGenObjInfo_t* pGenObj = NULL;
  const float32 fVxAddThresh =
      pLCAGlobals->LCAFrontMirror.LCA_Vf_VxThreshAdd_mps;
  float32 fVxCloseStableObjAdjLane = TUE_C_F32_VALUE_INVALID;
  float32 fVxCloseStableObjOwnLane = TUE_C_F32_VALUE_INVALID;
  float32 fVxThreshFrontMirror = 0.0F;

  if (uObjIdxClosestStableObjOwnLane < LCA_INPUT_OBJECT_NUMBER) {
    pGenObj =
        pGetLCAGenObject_GenObj(uObjIdxClosestStableObjOwnLane, pGenObjList);
    fVxCloseStableObjOwnLane = pGenObj->fVrelX_mps;

    /*Calculate the Vx threshold for front mirror detection based on the Vx
    of the stable object on the own lane and the ego speed*/
    fVxThreshFrontMirror =
        (2.0F * fVxCloseStableObjOwnLane) + pEgoInfo->fegoVelocity_mps;
    pLCAGlobals->LCAFrontMirror.LCA_Vf_VxThreshOwnLaneMin_mps =
        fVxThreshFrontMirror - fVxAddThresh;
    pLCAGlobals->LCAFrontMirror.LCA_Vf_VxThreshOwnLaneMax_mps =
        fVxThreshFrontMirror + fVxAddThresh;

    if (pGenObj->fRCS > pLCAGlobals->LCAFrontMirror.fRCSStableObjOwnLane) {
      TUE_CML_LowPassFilter(&pLCAGlobals->LCAFrontMirror.fRCSStableObjOwnLane,
                            pGenObj->fRCS, LCA_FRONT_MIRROR_RCS_FILT_UP);
    } else {
      TUE_CML_LowPassFilter(&pLCAGlobals->LCAFrontMirror.fRCSStableObjOwnLane,
                            pGenObj->fRCS, LCA_FRONT_MIRROR_RCS_FILT_DN);
    }
  } else {
    /*Set the Vx threshold and RCS value to invalid*/
    pLCAGlobals->LCAFrontMirror.LCA_Vf_VxThreshOwnLaneMin_mps =
        TUE_C_F32_VALUE_INVALID;
    pLCAGlobals->LCAFrontMirror.LCA_Vf_VxThreshOwnLaneMax_mps =
        TUE_C_F32_VALUE_INVALID;
    pLCAGlobals->LCAFrontMirror.fRCSStableObjOwnLane =
        LCA_FRONT_MIRROR_RCS_INVALID;
  }

  if (uObjIdxClosestStableObjAdjLane < LCA_INPUT_OBJECT_NUMBER) {  // 80
    pGenObj =
        pGetLCAGenObject_GenObj(uObjIdxClosestStableObjAdjLane, pGenObjList);
    fVxCloseStableObjAdjLane = pGenObj->fVrelX_mps;

    /*Calculate the Vx threshold for front mirror detection based on the Vx
    of the stable object on the own lane and the ego speed*/
    fVxThreshFrontMirror =
        (2.0F * fVxCloseStableObjAdjLane) + pEgoInfo->fegoVelocity_mps;
    pLCAGlobals->LCAFrontMirror.LCA_Vf_VxThreshAdjLaneMin_mps =
        fVxThreshFrontMirror - fVxAddThresh;
    pLCAGlobals->LCAFrontMirror.LCA_Vf_VxThreshAdjLaneMax_mps =
        fVxThreshFrontMirror + fVxAddThresh;

    if (pGenObj->fRCS > pLCAGlobals->LCAFrontMirror.fRCSStableObjAdjLane) {
      TUE_CML_LowPassFilter(&pLCAGlobals->LCAFrontMirror.fRCSStableObjAdjLane,
                            pGenObj->fRCS, LCA_FRONT_MIRROR_RCS_FILT_UP);
    } else {
      TUE_CML_LowPassFilter(&pLCAGlobals->LCAFrontMirror.fRCSStableObjAdjLane,
                            pGenObj->fRCS, LCA_FRONT_MIRROR_RCS_FILT_DN);
    }
  } else {
    /*Set the Vx threshold and RCS value to invalid*/
    pLCAGlobals->LCAFrontMirror.LCA_Vf_VxThreshAdjLaneMin_mps =
        TUE_C_F32_VALUE_INVALID;
    pLCAGlobals->LCAFrontMirror.LCA_Vf_VxThreshAdjLaneMax_mps =
        TUE_C_F32_VALUE_INVALID;
    pLCAGlobals->LCAFrontMirror.fRCSStableObjAdjLane =
        LCA_FRONT_MIRROR_RCS_INVALID;
  }

  /*Store the IDs of the stable objects in the LCA globals struct*/
  pLCAGlobals->LCAFrontMirror.uClosetStableObjIDOwnLane =
      uObjIdxClosestStableObjOwnLane;
  pLCAGlobals->LCAFrontMirror.uClosetStableObjIDAdjLane =
      uObjIdxClosestStableObjAdjLane;
}

/*****************************************************************************
  Functionname: LCACheckLCAPathBlocked                                  */ /*!

  @brief: Check if the adjacent lane path is blocked such that no LCA warning shall be initiated

  @description:  Check if the adjacent lane path is blocked such that no LCA warning shall be initiated.
                 The check is based on the estimated number of adjacent lanes and the distance to the 
				 road border.

  @param[in]:
              pRoad->fYOffsetFused_met                    Distance between ego vehicle and boundary
			  pRoad->fYOffsetFusedOppBorder_met           Distance between ego vehicle and right boundary
			  pRoad->fConfAdjacentLanes_per               Confidence percentage of adjacent lane information
			  pRoad->iNumOfAdjacentLanes_nu               The Number of adjacent lane
			  pRoad->fConfYOffset_per                     Confidence percentage of offset in y-axis positive direction
			  pRoad->fConfYOppOffset_per                  Confidence percentage of offset in y-axis negative direction
			  fVehicleWidth                               Vehicle width
			  pEgoInfo->fegoVelocity_mps                  The ego vehicle longitudinal velocity
			  LCAGlobals->uCntLCAPathBlockedLeft          Counter of left LCA path blocked
			  LCAGlobals->uCntLCAPathBlockedRight		  Counter of right LCA path blocked
			  LCAGlobals->bLCAPathBlockedLeft			  Flag whether left adjacent lane is no more
			  LCAGlobals->bLCAPathBlockedRight			  Flag whether right adjacent lane is no more
  @return:void
*****************************************************************************/
void LCACheckLCAPathBlocked(const LCARoad_t* pRoad,
                            const LCAVehicleInfo_t* pEgoInfo,
                            float32 fVehicleWidth, LCAGlobals_st* LCAGlobals) {
  float32 fMinConfRoadEstimation = 0.F;
  float32 fMinRoadBorderDistance = 0.F;
  boolean bZeroAdjLanes = FALSE;
  float32 fYOffsetBorder = pRoad->fYOffsetFused_met - 0.5 * fVehicleWidth;
  float32 fYOffsetFusedOppBorder =
      fABS(pRoad->fYOffsetFusedOppBorder_met) - 0.5 * fVehicleWidth;  //+
  boolean bGRDTooCloseLeft = FALSE;
  boolean bGRDTooCloseRight = FALSE;
  static const GDBVector2_t
      aLCALinearLaneSpeedCondition[LCA_LANE_COND_NROF_THRESHOLDS] =
          {{LCA_LANE_COND_LOW_SPEED, LCA_LANE_COND_LANE_WIDTH_MIN},
           {LCA_LANE_COND_MED_SPEED, LCA_LANE_COND_LANE_WIDTH_MED},
           {LCA_LANE_COND_HIGH_SPEED,
            LCA_LANE_COND_LANE_WIDTH_MAX}};  // LCA calculation of min road
                                             // border distance related to ego
                                             // speed

  // Calculate the minium confidence required for the road estimation check
  // based on the ego speed
  fMinConfRoadEstimation = GDBmathLinFuncLimBounded(
      pEgoInfo->fegoVelocity_mps, LCA_LI_PATHB_EGOSPEED_MIN,
      LCA_LI_PATHB_EGOSPEED_MAX, LCA_LI_PATHB_BORDERCONF_MIN,
      LCA_LI_PATHB_BORDERCONF_MAX);  //[5 15 0.6 0.4]
  // Calculate the minimum fused road border distance allowed for the given ego
  // speed
  fMinRoadBorderDistance = GDB_Math_CalculatePolygonValue(
      LCA_LANE_COND_NROF_THRESHOLDS, aLCALinearLaneSpeedCondition,
      pEgoInfo->fegoVelocity_mps);
  // Check if the lane number estimation reports at least one adjacent lane with
  // a reasonable confidence
  if (pRoad->fConfAdjacentLanes_per > fMinConfRoadEstimation &&
      pRoad->iNumOfAdjacentLanes_nu == 0) {
    bZeroAdjLanes = TRUE;
  }
  // Check, based on the estimated border offset, if a vehicle may overtake the
  // subject left path
  if (pRoad->fConfYOffset_per > fMinConfRoadEstimation &&
      fYOffsetBorder < fMinRoadBorderDistance) {
    bGRDTooCloseLeft = TRUE;
  }
  PathBlockedDecision(bZeroAdjLanes, bGRDTooCloseLeft,
                      &LCAGlobals->uCntLCAPathBlockedLeft,
                      &LCAGlobals->bLCAPathBlockedLeft);
  // right path
  if (pRoad->fConfYOppOffset_per > fMinConfRoadEstimation &&
      fYOffsetFusedOppBorder < fMinRoadBorderDistance) {
    bGRDTooCloseRight = TRUE;
  }
  PathBlockedDecision(bZeroAdjLanes, bGRDTooCloseRight,
                      &LCAGlobals->uCntLCAPathBlockedRight,
                      &LCAGlobals->bLCAPathBlockedRight);
}

/*****************************************************************************
  Functionname: PathBlockedDecision                                  */ /*!

  @brief: Decision whether the LCA path is blocked

  @description: Basing on the count, we decide whether the LCA path is blocked

  @param[in]: 
              bZeroAdjLanes                   Flag whether adjacent lane is zero 
			  bGRDTooClose                    Flag whether the LCA path width is not enough
			  uCntLCAPathBlocked              Count of LCA path blocked
			  bLCAPathBlocked                 Flag whether the LCA path is blocked
  @param[out]: 
              uCntLCAPathBlocked              Count of LCA path blocked
              bLCAPathBlocked                 Flag whether the LCA path is blocked
  @return:void
*****************************************************************************/
void PathBlockedDecision(boolean bZeroAdjLanes, boolean bGRDTooClose,
                         uint8* uCntLCAPathBlocked, boolean* bLCAPathBlocked) {
  // if both condition are true, count up the counter that indicates that the
  // adjacent lane path is free. Otherwise, count down the respective counter
  if (bZeroAdjLanes && bGRDTooClose) {
    if (*uCntLCAPathBlocked < LCA_PATHB_COUNTER_MAX) {
      *uCntLCAPathBlocked += 1U;
    }
  } else {
    if (*uCntLCAPathBlocked > 0U) {
      *uCntLCAPathBlocked -= 1U;
    }
  }
  // Use different counter thresholds to set the adjacent lane path to free
  // based on the observed state of the adjacent lane path This introduces a
  // hysteresis for this check
  if (*bLCAPathBlocked) {
    if (*uCntLCAPathBlocked > LCA_PATHB_ACTIVE_COUNTER_THRESH)  // 2
    {
      *bLCAPathBlocked = TRUE;
    } else {
      *bLCAPathBlocked = FALSE;
    }
  } else {
    if (*uCntLCAPathBlocked > LCA_PATHB_INACTIVE_COUNTER_THRESH)  // 6
    {
      *bLCAPathBlocked = TRUE;
    } else {
      *bLCAPathBlocked = FALSE;
    }
  }
}

/*****************************************************************************
  Functionname: LCASetObjDependTTCThreshold                                  */ /*!

  @brief: Sets the LCA TTC threshold for each object separately

  @description: Sets the LCA TTC threshold for each object separately

  @param[in]:
              bLCAWarningLastCycle             Flag whether current object is warning in last cycle
			  pGenObj->fVrelX_mps              Object's longitudinal relative velocity
			  pLCACalc->LCAGlobals.LCAConfig.fTTCThreshVrelHigh
			  pLCACalc->LCAGlobals.LCAConfig.fTTCThreshVrelMid
			  pLCACalc->LCAGlobals.LCAConfig.fTTCThreshVrelLow
			  pLCACalc->LCAGlobals.LCAConfig.fTTCHysteresis
  @param[out]:
			  fTTCThreshold                    TTC threshold
  @return:void
*****************************************************************************/
void LCASetObjDependTTCThreshold(boolean bLCAWarningLastCycle,
                                 LCACalculate_st* pLCACalc,
                                 const LCAGenObjInfo_t* pGenObj,
                                 float32* fTTCThreshold) {
  // Check absolute speed
  if (pGenObj->fVabsX_mps > LCA_TTC_VREL_LOW_MAX)  // 10
  {
    if (pGenObj->fVabsX_mps > LCA_TTC_VREL_MID_MAX)  // 15
    {
      *fTTCThreshold = pLCACalc->LCAGlobals.LCAConfig.fTTCThreshVrelHigh_s;
    } else {
      *fTTCThreshold = pLCACalc->LCAGlobals.LCAConfig.fTTCThreshVrelMid_s;
    }
  } else {
    *fTTCThreshold = pLCACalc->LCAGlobals.LCAConfig.fTTCThreshVrelLow_s;
  }
  // Check if the object already carries an LCA warning
  if (bLCAWarningLastCycle) {
    // Add the LCA TTC hysteresis value to the TTC threshold
    *fTTCThreshold += pLCACalc->LCAGlobals.LCAConfig.fTTCHysteresis_s;
  }
}

/*****************************************************************************
  Functionname: LCACheckObjectStartProperties */ /*!

  @brief: Gather relevant information about the start of an object

  @description: Gather relevant information about the start of an object

  @param[in]:
              pGenObj->uiLifeCycles_nu
			  pLBSObj->fTTC_s
			  pGenObj->fDistX_met

  @param[out]:
			  bLowTTCAtStart                  Flag whether current object has low TTC at start
			  bCreateAdjStableObj             Flag whether current object was created next to another stable object
  @return:void
*****************************************************************************/
void LCACheckObjectStartProperties(uint8 uObjIndex,
                                   const LCAGenObjList_st* pGenObjList,
                                   const LCALBSObjInfo_t* pLBSObj,
                                   float32 fTTCThreshold,
                                   boolean* bLowTTCAtStart,
                                   boolean* bCreateAdjStableObj) {
  boolean bAdjObjFound = FALSE;
  uint8 uObjIdxCur = 0;
  const LCAGenObjInfo_t* pGenObj = &pGenObjList->aObject[uObjIndex].GenObjInfo;
  // const LCALBSObjInfo_t* pLBSObj =
  // &pGenObjList->aObject[uObjIndex].LBSObjInfo;

  if (pGenObj->uiLifeCycles_nu == 1U) {
    // Check if the object has a very low TTC directly after creation
    if (pLBSObj->fTTC_s < fTTCThreshold &&
        pGenObj->fDistX_met < LCA_MAX_X_LOW_TTC_START_CHECK &&  //-5
        pGenObj->fDistX_met > LCA_MIN_X_LOW_TTC_START_CHECK)    //-50
    {
      *bLowTTCAtStart = TRUE;
    }
    // Check if the object is created in the direct neighborhood of a stable
    // object
    uObjIdxCur = 0U;
    while (bAdjObjFound == FALSE && uObjIdxCur < (uint8)EM_N_OBJECTS) {
      if (!bGetLCAGenObject_GenObjIsDeleted(uObjIdxCur, pGenObjList) &&
          uObjIdxCur != uObjIndex &&
          pLBSObj->fUpdateRate_nu > LCA_MIN_UPDRATE_ADJ_STABLE_OBJ)  // 0.9
      {
        const float32 fXObjCur =
            pGenObjList->aObject[uObjIdxCur].GenObjInfo.fDistX_met;
        const float32 fYObjCur =
            pGenObjList->aObject[uObjIdxCur].GenObjInfo.fDistY_met;
        const float32 fXDiffAbs = fABS(pGenObj->fDistX_met - fXObjCur);
        const float32 fYDiffAbs = fABS(pGenObj->fDistY_met - fYObjCur);
        if (fXDiffAbs < LCA_MAX_X_DIST_ADJ_STABLE_OBJ &&  // 2.5
            fYDiffAbs < LCA_MAX_Y_DIST_ADJ_STABLE_OBJ)    // 2.5
        {
          *bCreateAdjStableObj = TRUE;
          bAdjObjFound = TRUE;
        }
      }
      uObjIdxCur = uObjIdxCur + 1U;
    }
  }
}

/*****************************************************************************
  Functionname: LCACheckObjectMirrorStatus                                  */ /*!

  @brief: Check if the current object is a mirror object

  @description: Check if the current object is a mirror object based on probability provided by EM

  @param[in]:

  @return:void
*****************************************************************************/
boolean LCACheckObjectMirrorStatus(const LCAGenObjInfo_t* pGenObj,
                                   float32 fUpdateRate_nu,
                                   boolean bLCAMirrorObjectLastCycle) {
  boolean bLCAMirrorObject = bLCAMirrorObjectLastCycle;
  float32 fObjDist2Border = pGenObj->fDist2Border_met;

  if (!bLCAMirrorObjectLastCycle) {
    if ((pGenObj->fMirrorProb_per > LCA_MIRROR_PROB_LOW &&  // 0.7
         fUpdateRate_nu < LCA_MAX_UPDRATE_MIRROR_CHECK) ||  // 0.95
        pGenObj->fMirrorProb_per > LCA_MIRROR_PROB_HIGH)    // 0.95
    {
      bLCAMirrorObject = TRUE;
    }
  } else {
    if (pGenObj->bRightSensor) {
      fObjDist2Border = -fObjDist2Border;
    }

    if (pGenObj->fMirrorProb_per < LCA_MIRROR_PROB_RECLASSIFY &&  // 0.5
        (fObjDist2Border < LCA_MIN_DIST2BORDER_MIRROR_CHECK ||    //-0.5
         !pGenObj->bDist2BorderValid)) {
      bLCAMirrorObject = FALSE;
    }
  }
  return bLCAMirrorObject;
}

#if ST_PERCEPTION
/*****************************************************************************
  Functionname: LCACheckObjectFrontMirror                                  */ /*!

  @brief: Check if the current object is a front mirror target

  @description: Check if the current object is a static object in front of our subject
                vehicle that gets mirrored by a stable object behind the subject vehicle

  @param[in]:

  @return:void
*****************************************************************************/
boolean LCACheckObjectFrontMirror(uint8 uObjIndex,
                                  const LCAGenObjList_st* pGenObjList,
                                  const LCALBSObjInfo_t* pLBSObj,
                                  const LCAVehicleInfo_t* pEgoInfo,
                                  LCAObjInfo_t* pLCAObj,
                                  LCACalculate_st* pLCACalc,
                                  float32 fMaxSpeedOverGround_mps) {
  boolean bPossibleMirrorFrontObject = FALSE;
  boolean bLCAMirrorFrontObject = pLCAObj->bLCAMirrorFrontObject;
  const LCAGenObjInfo_t* pGenObj = &pGenObjList->aObject[uObjIndex].GenObjInfo;
  // const LCALBSObjInfo_t* pLBSObj =
  // &pGenObjList->aObject[uObjIndex].LBSObjInfo;
  LCAFrontMirror_t* pLCAFrontMirror = &pLCACalc->LCAGlobals.LCAFrontMirror;
  float32 fVxThreshAddOwnLane;
  float32 fVxThreshAddAdjLane;
  boolean bCheckForStableOwnLaneObj = FALSE;
  boolean bCheckForStableAdjLaneObj = FALSE;

  if ((pEgoInfo->fegoVelocity_mps > LCA_MIN_EGOSPEED_FM_DETECTION ||  // 5
       (pEgoInfo->fegoVelocity_mps >
            (0.5F * LCA_MIN_EGOSPEED_FM_DETECTION) &&                     // 5
        pLCAFrontMirror->fFMObjRate > LCA_FM_ADDVXTHRESH_FMRATE_MIN)) &&  // 1.5
      !pLCAObj->bLCAWarning &&
      pGenObj->fVrelX_mps > LCA_MIN_TARGETSPEED_FM)  // 5
  {
    // Calculate vx add thresholds of own lane and adjacent lane
    LCAVxThreshAddCalc(pLBSObj->fUpdateRate_nu, pEgoInfo->fegoVelocity_mps,
                       pGenObj->fVrelX_mps, pLCAFrontMirror,
                       fMaxSpeedOverGround_mps, &fVxThreshAddOwnLane,
                       &fVxThreshAddAdjLane);
    // Check if there is an stable object on the own lane, and on the adjacent
    // lane
    LCACheckStableOwnLaneObject(uObjIndex, pGenObjList, pLCAFrontMirror,
                                &bCheckForStableOwnLaneObj,
                                &bCheckForStableAdjLaneObj);
    // Check for stable ownlane object vx threshold
    if (bCheckForStableOwnLaneObj) {
      if (pGenObj->fVrelX_mps >
              (pLCAFrontMirror->LCA_Vf_VxThreshOwnLaneMin_mps -
               fVxThreshAddOwnLane) &&
          pGenObj->fVrelX_mps <
              (pLCAFrontMirror->LCA_Vf_VxThreshOwnLaneMax_mps +
               fVxThreshAddOwnLane)) {
        bPossibleMirrorFrontObject = TRUE;
      }
    }
    // Check for stable adjacent lane object vx threshold
    if (!bPossibleMirrorFrontObject && bCheckForStableAdjLaneObj) {
      if (pGenObj->fVrelX_mps >
              (pLCAFrontMirror->LCA_Vf_VxThreshAdjLaneMin_mps -
               fVxThreshAddAdjLane) &&
          pGenObj->fVrelX_mps <
              (pLCAFrontMirror->LCA_Vf_VxThreshAdjLaneMax_mps +
               fVxThreshAddAdjLane)) {
        bPossibleMirrorFrontObject = TRUE;
      }
    }
  }

  // Update the MirrorFrontObject counters and the associated flag
  bLCAMirrorFrontObject = LCAUpdateMirrorFrontObjectState(
      &pLCAObj->uFrontMirrorCnt, bPossibleMirrorFrontObject,
      bLCAMirrorFrontObject,
      &pLCACalc->LCAGlobals.LCAFrontMirror.uNofFMObjects);

  return bLCAMirrorFrontObject;
}
#endif

/*****************************************************************************
  Functionname: LCAVxThreshAddCalc                                  */ /*!

  @brief:

  @description:

  @param[in]:

  @return:void
*****************************************************************************/
void LCAVxThreshAddCalc(float32 fUpdateRate, float32 fegoVelocity,
                        float32 fVrelX, LCAFrontMirror_t* pLCAFrontMirror,
                        float32 fMaxSpeedOverGround,
                        float32* fVxThreshAddOwnLane,
                        float32* fVxThreshAddAdjLane) {
  float32 fVxThreshAdd = 0.F;

  // Additional vx threshold offset based on the object quality
  fVxThreshAdd = LCA_FM_ADDVXTHRESH_UPDRATE_FACTOR * (1.F - fUpdateRate);  // 2
  if (fegoVelocity > (2.F * LCA_MIN_EGOSPEED_FM_DETECTION) &&              // 5
      (fVrelX + fegoVelocity) >
          LCA_FM_ADDVXTHRESH_UPDRATE_FACTOR * fMaxSpeedOverGround) {
    fVxThreshAdd += GDBmathLinFuncLimBounded(
        fVrelX, LCA_LI_FM_VXTHRESHADD_MIN_VX, LCA_LI_FM_VXTHRESHADD_MAX_VX,
        LCA_LI_FM_VXTHRESHADD_MIN_THRESH,
        LCA_LI_FM_VXTHRESHADD_MAX_THRESH);  // 0  20  0.5  2
  }
  *fVxThreshAddOwnLane = fVxThreshAdd;
  *fVxThreshAddAdjLane = fVxThreshAdd;

  if (fegoVelocity < (2.F * LCA_MIN_EGOSPEED_FM_DETECTION))  // 5
  {
    if (pLCAFrontMirror->fRCSStableObjOwnLane < LCA_FM_LOW_RCS_THRESH)  // 15
    {
      *fVxThreshAddOwnLane -= LCA_FM_ADDVXTHRESH_LOWRCS;  // 0.5
    }
    if (pLCAFrontMirror->fRCSStableObjAdjLane < LCA_FM_LOW_RCS_THRESH) {
      *fVxThreshAddAdjLane -= LCA_FM_ADDVXTHRESH_LOWRCS;
    }
  }

  *fVxThreshAddOwnLane = MAX(*fVxThreshAddOwnLane, 0.F);
  *fVxThreshAddAdjLane = MAX(*fVxThreshAddAdjLane, 0.F);
}

/*****************************************************************************
  Functionname: LCACheckStableOwnLaneObject                                  */ /*!

  @brief: Check if there is an stable object on the own lane, and on the adjacent lane

  @description: Check if there is an stable object on the own lane, and on the adjacent lane

  @param[in]:

  @return:void
*****************************************************************************/
void LCACheckStableOwnLaneObject(uint8 uObjIndex,
                                 const LCAGenObjList_st* pGenObjList,
                                 LCAFrontMirror_t* pLCAFrontMirror,
                                 boolean* bCheckForStableOwnLaneObj,
                                 boolean* bCheckForStableAdjLaneObj) {
  float32 fXMirroringObjOwnLane = -100.F;
  float32 fYMirroringObjOwnLane = -100.F;
  float32 fXMirroringObjAdjLane = 0.F;
  float32 fYMirroringObjAdjLane = 0.F;
  const sint32 uCloseStableObjIDOwnLane =
      pLCAFrontMirror->uClosetStableObjIDOwnLane;
  const sint32 uCloseStableObjIDAdjLane =
      pLCAFrontMirror->uClosetStableObjIDAdjLane;
  const LCAGenObjInfo_t* pGenObj = &pGenObjList->aObject[uObjIndex].GenObjInfo;

  // Either get X/Y from the stable own lane object, or in case a trailer is
  // attached get a standard trailer mirror surface coordinate-set
  LCAGetXYMirrorSurface(uCloseStableObjIDOwnLane, pGenObjList,
                        &fXMirroringObjOwnLane, &fYMirroringObjOwnLane);

  if (uCloseStableObjIDOwnLane != uObjIndex) {
    if (pGenObj->fDistX_met < fXMirroringObjOwnLane) {
      *bCheckForStableOwnLaneObj = TRUE;
    }
  }

  if (uCloseStableObjIDAdjLane < EM_N_OBJECTS &&
      uCloseStableObjIDAdjLane != uObjIndex) {
    fXMirroringObjAdjLane =
        pGenObjList->aObject[uCloseStableObjIDAdjLane].GenObjInfo.fDistX_met;
    if (pGenObj->fDistX_met < fXMirroringObjAdjLane) {
      *bCheckForStableAdjLaneObj = TRUE;
    }
  }

  if (*bCheckForStableOwnLaneObj) {
    if (fABS(fYMirroringObjOwnLane - pGenObj->fDistY_met) <
            LCA_FM_OBJ_BEHIND_DISTY_THRESH &&  // 2
        (fXMirroringObjOwnLane - pGenObj->fDistX_met) <
            LCA_FM_OBJ_BEHIND_DISTX_THRESH)  // 10
    {
      // Object is directly behind the own lane stable object, therefore do not
      // check for the adjacent lane object
      *bCheckForStableAdjLaneObj = FALSE;
    }
  }

  if (*bCheckForStableAdjLaneObj && uCloseStableObjIDAdjLane < EM_N_OBJECTS) {
    fXMirroringObjAdjLane =
        pGenObjList->aObject[uCloseStableObjIDAdjLane].GenObjInfo.fDistX_met;
    fYMirroringObjAdjLane =
        pGenObjList->aObject[uCloseStableObjIDAdjLane].GenObjInfo.fDistY_met;
    if (fABS(fYMirroringObjAdjLane - pGenObj->fDistY_met) <
            LCA_FM_OBJ_BEHIND_DISTY_THRESH &&  // 2
        (fXMirroringObjAdjLane - pGenObj->fDistX_met) <
            LCA_FM_OBJ_BEHIND_DISTX_THRESH)  // 10
    {
      // Object is directly behind the adjacent lane stable object, therefore do
      // not check for the own lane object
      *bCheckForStableOwnLaneObj = FALSE;
    }
  }
}

/*****************************************************************************
  Functionname: LCAGetXYMirrorSurface                                  */ /*!

  @brief: Returns x-y coordinate of a mirror surface

  @description: Returns x-y coordinate of a mirror surface

  @param[in]: 
              uObjID               Array index of object whose coordinates should be reported as mirror surface
			  pGenObjList          All objects list general information
  @param[out]:
              pfX                  Pointer to x-coordinate
			  pfY                  Pointer to y-coordinate
  @return:void
*****************************************************************************/
void LCAGetXYMirrorSurface(sint32 uObjID, const LCAGenObjList_st* pGenObjList,
                           float32* pfX, float32* pfY) {
  *pfX = -100.F;
  *pfY = -100.F;

  if (uObjID < EM_N_OBJECTS) {
    *pfX = pGenObjList->aObject[uObjID].GenObjInfo.fDistX_met;
    *pfY = pGenObjList->aObject[uObjID].GenObjInfo.fDistY_met;
  }
}

/*****************************************************************************
  Functionname: LCAUpdateMirrorFrontObjectState */ /*!

  @brief: Update the MirrorFrontObject counters and the associated flag

  @description: Update the MirrorFrontObject counters and the associated flag

  @param[in]:

  @return:void
*****************************************************************************/
boolean LCAUpdateMirrorFrontObjectState(uint8* uFrontMirrorCnt,
                                        boolean bPossibleMirrorFrontObject,
                                        boolean bLCAMirrorFrontObjectLastCycle,
                                        uint8* uNofFMObjects) {
  boolean bLCAMirrorFrontObject = bLCAMirrorFrontObjectLastCycle;

  if (bPossibleMirrorFrontObject) {
    if (*uFrontMirrorCnt < LCA_FM_COUNTER_MAX)  // 20
    {
      *uFrontMirrorCnt += 1U;
    }
  } else {
    if (*uFrontMirrorCnt > 0U) {
      *uFrontMirrorCnt -= 1U;
    }
  }

  if (bLCAMirrorFrontObjectLastCycle) {
    if (*uFrontMirrorCnt > LCA_FM_ACTIVE_CNT_THRESH)  // 2
    {
      bLCAMirrorFrontObject = TRUE;
    } else {
      bLCAMirrorFrontObject = FALSE;
    }
  } else {
    if (*uFrontMirrorCnt > LCA_FM_INACTIVE_CNT_THRESH)  // 10
    {
      bLCAMirrorFrontObject = TRUE;
    } else {
      bLCAMirrorFrontObject = FALSE;
    }
  }

  // Count the number of front mirror objects and filter up the front mirror
  // rate
  if (bLCAMirrorFrontObject) {
    *uNofFMObjects += 1U;
  }
  return bLCAMirrorFrontObject;
}

/*****************************************************************************
  Functionname: LCACheckObjectQuality                                  */ /*!

  @brief: Check if the currents object quality is high enough for LCA

  @description: Check if the currents object quality is high enough for LCA
                Check is made by based on update rate and POE

  @param[in]: 
			  bLCAWarningLastCycle                             Flag whether current object is warning
			  bLCAQualityLastCycle                             Flag whether the object quality is enough
			  pGenObj->uiLifeCycles_nu                         Object lifetime in cycles
			  pGenObj->uiMeasuredTargetFrequency_nu            Bitfield to indicate if the object was measured in the last 8 cycles
			  pGenObj->fVrelX_mps                              Object's longitudinal relative velocity
			  pGenObj->fDistX_met                              Object's longitudinal relative distance
			  pGenObj->fProbabilityOfExistence_per             Probability that the object represents a real object
			  pLBSObj->fXMovement_met                          The object total moving distance in the x direction
			  pLBSObj->bLowTTCAtStart
			  pLBSObj->bCreateAdjStableObj
			  pLBSObj->fUpdateRate_nu                          The object measurement update rate
  @return: 
              bLCAQuality                                      Flag whether the currents object quality is high enough for LCA
*****************************************************************************/
boolean LCACheckObjectQuality(const LCAGenObjInfo_t* pGenObj,
                              const LCALBSObjInfo_t* pLBSObj,
                              boolean bLowTTCAtStart,
                              boolean bCreateAdjStableObj,
                              boolean bLCAQualityLastCycle,
                              boolean bLCAWarningLastCycle) {
  boolean bLCAQuality = FALSE;
  float32 fMinPOE = LCA_MIN_POE;                   // 0.9
  float32 fMinLCAUpdateRate = LCA_MIN_UPDATERATE;  // 0.75

  // If the object is already marked as warning or relevant for LCA lower
  // quality thresholds are used
  if (bLCAWarningLastCycle) {
    fMinPOE = LCA_MIN_POE_RELEVANT;                   // 0.8
    fMinLCAUpdateRate = LCA_MIN_UPDATERATE_RELEVANT;  // 0.6
  } else {
    if (bLCAQualityLastCycle ||
        (pGenObj->uiLifeCycles_nu > LCA_QUAL_LIFETIME_MIN &&  // 2
         (pGenObj->uiMeasuredTargetFrequency_nu & UI_255_TO_BINARY) ==
             UI_255_TO_BINARY &&                     // 1111 1111
         pGenObj->fVrelX_mps > LCA_QUAL_VRELX_MIN))  // 12
    {
      fMinPOE = LCA_MIN_POE;                            // 0.9
      fMinLCAUpdateRate = LCA_QUAL_ACTIVE_UPDRATE_MIN;  // 0.7
    } else {
      fMinPOE = LCA_MIN_POE_INITIAL;  // 0.99
      // Check for a high update rate for objects at close to mid distance with
      // low movement in x direction since start
      if (pLBSObj->fXMovement_met < LCA_MIN_X_MOVEMENT_RELEVANT &&  // 5
          pGenObj->fDistX_met > LCA_QUAL_HIGH_UPDATE_DISTX_MIN)     //-40
      {
        fMinLCAUpdateRate = LCA_QUAL_INACTIVE_UPDRATE_MIN;  // 0.9
      } else {
        // Check for a high update rate for objects that have a low TTC directly
        // after creation
        if (bLowTTCAtStart) {
          // Check for an even higher update rate if these objects are created
          // adjacent to a stable object because it is very likely that these
          // objects are non-relevant distortions
          if (bCreateAdjStableObj) {
            fMinLCAUpdateRate = LCA_QUAL_LOWTTC_ADJACENT_UPDATE_MIN;  // 0.95
          } else {
            fMinLCAUpdateRate = LCA_QUAL_INACTIVE_UPDRATE_MIN;  // 0.9
          }
        } else {
          if (bCreateAdjStableObj) {
            fMinLCAUpdateRate = LCA_QUAL_INACTIVE_UPDRATE_MIN;  // 0.9
          }
        }
      }
    }
  }
  // For already warning objects check if either the update rate or the
  // probability of existence is larger than the defined threshold
  if (bLCAWarningLastCycle) {
    if (pLBSObj->fUpdateRate_nu > fMinLCAUpdateRate ||
        pGenObj->fProbabilityOfExistence_per > fMinPOE) {
      bLCAQuality = TRUE;
    }
  } else {
    // For non-warning objects check if update rate and probability of existence
    // is larger than the defined threshold
    if (pLBSObj->fUpdateRate_nu > fMinLCAUpdateRate &&
        pGenObj->fProbabilityOfExistence_per > fMinPOE) {
      bLCAQuality = TRUE;
    }
  }
  return bLCAQuality;
}

/*****************************************************************************
  Functionname: LCASetObjDependTTCThreshold                                  */ /*!

  @brief: Check the current update status of the object

  @description: Check the current update status of the object

  @param[in]:
              uiMeasuredTargetFrequency           Bitfield to indicate if the object was measured in the last 8 cycles
			  fUpdateRate                         The object measurement update rate
  @return:
              bUpdateRecently                     Flag whether the current status is updated
*****************************************************************************/
boolean LCACheckUpdateStatus(uint8 uiMeasuredTargetFrequency,
                             float32 fUpdateRate) {
  boolean bUpdateRecently = FALSE;

  if ((uiMeasuredTargetFrequency & UI_192_TO_BINARY) ==
      UI_192_TO_BINARY)  // 1100 0000
  {
    bUpdateRecently = TRUE;
  } else {
    if (fUpdateRate > LCA_UPDATED_UPDATED_MIN &&
        ((uiMeasuredTargetFrequency & UI_128_TO_BINARY) ==
             UI_128_TO_BINARY ||  // Updated in this cycle but not in the last
                                  // cycle
         (uiMeasuredTargetFrequency & UI_64_TO_BINARY) ==
             UI_64_TO_BINARY))  // Updated in last cycle but not in the this
                                // cycle
    {
      bUpdateRecently = TRUE;
    }
  }
  return bUpdateRecently;
}

/*****************************************************************************
  Functionname: LCACheckObjectRelevancev                                  */ /*!

  @brief: Check if the current object is relevant for LCA

  @description: Check if the current object is relevant for LCA.
                Check is done based on InLaneTime, Xmovement, LiftCycle and X distance

  @param[in]:
              pSIObj->eAssociatedLane          Lane Association
			  bLCAWarningLastCycle             Flag of LCA warning last cycle
			  bLCARelevantLastCycle            Flag of LCA relevant last cycle
			  fXMovement                       The object total moving distance in the x direction
			  pGenObj->fDistX_met              Object's longitudinal relative distance
			  pGenObj->uiLifeCycles_nu         Object lifetime in cycles

  @return:    bLCARelevant                      Flag whether the object is relevant for LCA
*****************************************************************************/
boolean LCACheckObjectRelevance(const LCAGenObjInfo_t* pGenObj,
                                const LCASIObjInfo_t* pSIObj,
                                float32 fXMovement,
                                boolean bLCARelevantLastCycle,
                                boolean bLCAWarningLastCycle) {
  boolean bLCAInlaneTimeCondition = TRUE;
  boolean bLCARelevant = FALSE;
  float32 fInlaneTime = 0.F;

  if (pSIObj->eAssociatedLane == LCA_SI_ASSOC_LANE_LEFT ||
      pSIObj->eAssociatedLane == LCA_SI_ASSOC_LANE_RIGHT) {
    // Calculate the estimated time of the object being inside the relevant lane
    fInlaneTime = LCACalculateObjInLaneTime(pGenObj, pSIObj);
    // Check if the object is preselected for LCA and the object is not a
    // guardrail object
    if (fInlaneTime >= LCA_MIN_INLANE_TIME ||  // 0.3
        bLCAWarningLastCycle) {
      if (!bLCARelevantLastCycle) {
        if (fXMovement > LCA_MIN_X_MOVEMENT_RELEVANT ||            // 5
            pGenObj->fDistX_met < LCA_MAX_X_DIST_RELEVANT ||       //-5
            pGenObj->uiLifeCycles_nu > LCA_MIN_LIFETIME_RELEVANT)  // 50
        {
          bLCARelevant = TRUE;
        }
      } else {
        // Object is already relevant, therefore keep this state
        bLCARelevant = TRUE;
      }
    }
  }
  return bLCARelevant;
}

/*****************************************************************************
  Functionname: LCACalculateObjInLaneTime                                  */ /*!

  @brief: Calculate predicted time in which the object will exit the adjacent lane

  @description: Calculate predicted time in which the object will exit the adjacent lane

  @param[in]:
              pSIObj->fVrelToTraj_mps                Object's relative velocity to trajectory
			  pSIObj->fObjBracketOverlap_met         Overlap between object and trace bracket
			  pSIObj->fDistToTraj_met                Distance to trajectory
			  pSIObj->fTraceBracketLeft_met          The trace bracket left side coordinate
			  pSIObj->fTraceBracketRight_met         The trace bracket right side coordinate
			  pGenObj->fDistX_met                    Object's longitudinal relative distance
			  pGenObj->fWidthLeft_met                Object's width left of the track position
			  pGenObj->fWidthRight_met               Object's width right of the track position
			  pGenObj->bRightSensor                  Flag whether the object is detected by the right sensor
  @return:
              fInlaneTime                            time of the object within the adjacent lane
*****************************************************************************/
float32 LCACalculateObjInLaneTime(const LCAGenObjInfo_t* pGenObj,
                                  const LCASIObjInfo_t* pSIObj) {
  float32 fInlaneTime = 0.F;
  float32 fObjAbsPosLeft = 0.F;
  float32 fObjAbsPosRight = 0.F;
  float32 fObjBracketDist = 0.F;
  float32 fOverlapOffset = 0.F;

  // Check if the object overlaps with the trace brackets and that it has a
  // relevant relative velocity to the trajectory
  if (fABS(pSIObj->fVrelToTraj_mps) > LCA_INLANETIME_MIN_VREL_TO_TRAJ &&  // 0.1
      pSIObj->fObjBracketOverlap_met > C_F32_DELTA) {
    fOverlapOffset = GDBmathLinFuncLimBounded(
        pGenObj->fDistX_met, LCA_INLANETIME_MIN_X_DIST_OFFSET,
        LCA_INLANETIME_MAX_X_DIST_OFFSET, LCA_INLANETIME_MAX_OVERLAP_OFFSET,
        LCA_INLANETIME_MIN_OVERLAP_OFFSET);  //[-80 -50 0.6 0.3]//0.3
    // Calculate the position of the object that includes the dimension
    fObjAbsPosLeft = pSIObj->fDistToTraj_met +
                     pGenObj->fWidthLeft_met;  // 0 + 1.683463573 /2 = 0.8417
    fObjAbsPosRight = pSIObj->fDistToTraj_met - pGenObj->fWidthRight_met;
    // Calculate the inlane time that is needed until the object is outside the
    // trace bracket
    if (pSIObj->fVrelToTraj_mps < 0.F) {
      if (pGenObj->bRightSensor) {
        fObjBracketDist = pSIObj->fTraceBracketLeft_met - fObjAbsPosRight;
      } else {
        fObjBracketDist =
            fObjAbsPosLeft -
            pSIObj->fTraceBracketRight_met;  //0.8417 -( -4.989999771) = 5.8417;
      }
      fInlaneTime =
          (fObjBracketDist - fOverlapOffset) /
          (-pSIObj->fVrelToTraj_mps);  //(5.8417 -0.3)/(-0.099977322) = -55
    } else {
      if (pGenObj->bRightSensor) {
        fObjBracketDist = fObjAbsPosLeft - pSIObj->fTraceBracketRight_met;
      } else {
        fObjBracketDist = pSIObj->fTraceBracketLeft_met - fObjAbsPosRight;
      }
      fInlaneTime =
          (fObjBracketDist - fOverlapOffset) / SafeDiv(pSIObj->fVrelToTraj_mps);
    }

    if (fABS(pSIObj->fVrelToTraj_mps) <
            LCA_INLANETIME_MID_VREL_TO_TRAJ &&  // 0.5
        pSIObj->fObjBracketOverlap_met >
            LCA_INLANETIME_BRACKET_OVERLAP_MIN)  // 0.8
    {
      fInlaneTime = F32_VALUE_INVALID;
    }
    fInlaneTime = MAX(fInlaneTime, 0.F);
  } else {
    fInlaneTime = F32_VALUE_INVALID;
  }
  return fInlaneTime;
}

/*****************************************************************************
  Functionname: LCACheckObjectInLCARange                                  */ /*!

  @brief: Check if the current object is within the current LCA range

  @description: Check if the current object is within the current LCA range

  @param[in]:
              pGenObj->fDistX_met          Object's longitudinal relative distance
			  fLCARange

  @return:    bInLCARange                  Flag whether the object is within LCA range
*****************************************************************************/
boolean LCACheckObjectInLCARange(const LCAGenObjInfo_t* pGenObj,
                                 float32 fLCARange) {
  boolean bInLCARange = FALSE;

  // Check if the object is within the calculated LCA range
  // Do not set the range limitation flag to true if the range limitation is at
  // its smallest value because of possible mathematical inconsistencies in the
  // egotrace curve calculation
  if (pGenObj->fDistX_met >= -fLCARange &&
      fLCARange > (LCA_RANGE_MIN + LCA_RANGE_MIN_ADDTHRESH))  // 7 + 0.1
  {
    bInLCARange = TRUE;
  } else {
    bInLCARange = FALSE;
  }
  return bInLCARange;
}

/*****************************************************************************
  Functionname: LCACheckObjectBehindGRD                                  */ /*!

  @brief: Check if the current object is behind a guardrail

  @description: Check if the current object is behind a guardrail

  @param[in]:
		      pGenObj
  @return:void
*****************************************************************************/
float32 LCACheckObjectBehindGRD(const LCAGenObjInfo_t* pGenObj,
                                const LCARoad_t* pRoad,
                                const LCAVehicleInfo_t* pEgoInfo,
                                boolean bLCAMirrorFrontObject,
                                float32 fBehindGrdProb) {
  boolean bObjBehindGrd = FALSE;
  float32 fDist2Border = pGenObj->fDist2Border_met;

  if (!pGenObj->bRightSensor) {
    bObjBehindGrd =
        CheckObjBehindGrd(pGenObj, pRoad->fConfYOffset_per, fDist2Border,
                          pEgoInfo->fegoVelocity_mps, bLCAMirrorFrontObject);
  } else {
    fDist2Border = -fDist2Border;
    bObjBehindGrd =
        CheckObjBehindGrd(pGenObj, pRoad->fConfYOppOffset_per, fDist2Border,
                          pEgoInfo->fegoVelocity_mps, bLCAMirrorFrontObject);
  }

  // Filter behind GRD probability depending on the current state
  if (bObjBehindGrd) {
    GDB_Math_LowPassFilter(&fBehindGrdProb, 1.F,
                           LCA_BEHINDGRD_PROB_FILTER_UP);  // 0.05
  } else {
    GDB_Math_LowPassFilter(&fBehindGrdProb, 0.F,
                           LCA_BEHINDGRD_PROB_FILTER_DOWN);  // 0.0025
  }
  return fBehindGrdProb;
}

/*****************************************************************************
  Functionname: CheckObjBehindGrd                                  */ /*!

  @brief:

  @description:

  @param[in]:
			 
  @return:
*****************************************************************************/
boolean CheckObjBehindGrd(const LCAGenObjInfo_t* pGenObj, float32 fConfYOffset,
                          float32 fDist2Border, float32 fegoVelocity,
                          boolean bLCAMirrorFrontObject) {
  float32 fConfYOffsetThresh = 0.F;
  float32 fDist2BorderThresh = 0.F;
  boolean bObjBehindGrd = FALSE;

  // Calculate confidence percentage threshold of the offset in y-axis direction
  fConfYOffsetThresh = GDBmathLinFuncLimBounded(
      fegoVelocity, LCA_LI_BEHINDGRD_MIN_EGOSPEED,
      LCA_LI_BEHINDGRD_MAX_EGOSPEED, LCA_LI_BEHINDGRD_MIN_BORDERCONF,
      LCA_LI_BEHINDGRD_MAX_BORDERCONF);  //[0 25 0.4 0.8]
  // Calculate the distance threshold between object to border
  fDist2BorderThresh = GDBmathLinFuncLimBounded(
      pGenObj->fDistX_met, LCA_BEHIND_GRD_MIN_X_THRESH,
      LCA_BEHIND_GRD_MAX_X_THRESH, LCA_BEHIND_GRD_MAX_DIST2BORDER,
      LCA_BEHIND_GRD_MIN_DIST2BORDER);  //[-50 -10 -0.5 0]
  // Check if a valid road border estimation is available
  if (fConfYOffset > fConfYOffsetThresh) {
    if (!bLCAMirrorFrontObject) {
      if (pGenObj->bDist2BorderValid && fDist2Border > fDist2BorderThresh) {
        bObjBehindGrd = TRUE;
      }
    } else {
      if (pGenObj->bDist2BorderValid &&
          fDist2Border >
              (fDist2BorderThresh - LCA_BEHINDGRD_DIST2BORD_ADDTHRESH))  // 2
      {
        bObjBehindGrd = TRUE;
      }
    }
  }
  return bObjBehindGrd;
}

/*****************************************************************************
  Functionname: LCACheckLaneConditions                                  */ /*!

  @brief: Check if the current object meets the lane conditions for LCA

  @description: Check if the current object meets the lane conditions for LCA

  @param[in]:
			  pSIObj->eAssociatedLane             Lane Association
			  bLCAPathBlocked                     Flag whether adjacent lane is no more
  @return:
			  bLCALaneConditions                  
*****************************************************************************/
boolean LCACheckLaneConditions(const LCAGenObjInfo_t* pGenObj,
                               const LCASIObjInfo_t* pSIObj,
                               boolean bLCAPathBlockedLeft,
                               boolean bLCAPathBlockedRight) {
  boolean bLCALaneConditions = FALSE;
  boolean bLCAPathBlocked = FALSE;

  if (!pGenObj->bRightSensor) {
    bLCAPathBlocked = bLCAPathBlockedLeft;
  } else {
    bLCAPathBlocked = bLCAPathBlockedRight;
  }
  // Check the lane association and encironmental condition
  if ((pSIObj->eAssociatedLane == LCA_SI_ASSOC_LANE_LEFT ||
       pSIObj->eAssociatedLane == LCA_SI_ASSOC_LANE_RIGHT) &&
      bLCAPathBlocked == FALSE) {
    bLCALaneConditions = TRUE;
  } else {
    bLCALaneConditions = FALSE;
  }
  return bLCALaneConditions;
}

/*****************************************************************************
  Functionname: LCACheckObjPath                                  */ /*!

  @brief: Check if the object's width will fit between ego and road border

  @description: Check if the object's width will fit between ego and road border
				Additional offset is added to object's width

  @param[in]:
			 pGenObj->fWidthLeft_met           Object's width left of the track position
			 pGenObj->fWidthRight_met          Object's width right of the track position
			 pGenObj->bRightSensor             Flag whether the object is detected by the right sensor
			 fVehicleWidth                     the vehicle body width
			 pRoad->fYOffsetFused_met          Distance between ego vehicle and boundary
			 pLCAObj->bLCAObjPathInvalid       Flag whether path is accessible
			 pRoad->fConfYOffset_per
  @return:
			 bLCAObjPathInvalid                Flag whether path is accessible current cycle
*****************************************************************************/
boolean LCACheckObjPath(const LCAGenObjInfo_t* pGenObj, const LCARoad_t* pRoad,
                        boolean bLCAObjPathInvalidLastCycle,
                        float32 fVehicleWidth) {
  float32 fSideOffset = 0.F;
  float32 fMinConfBorder = 0.F;
  float32 fObjWidth = pGenObj->fWidthLeft_met + pGenObj->fWidthRight_met;
  float32 fAbsYOffsetBorder = 0.F;
  boolean bLCAObjPathInvalid = FALSE;

  if (!bLCAObjPathInvalidLastCycle) {
    fSideOffset = LCA_PATHINV_INACTIVE_SIDEOFFSET;      // 0.5
    fMinConfBorder = LCA_PATHINV_INACTIVE_BORDER_CONF;  // 0.6
  } else {
    fSideOffset = LCA_PATHINV_ACTIVE_SIDEOFFSET;      // 1
    fMinConfBorder = LCA_PATHINV_ACTIVE_BORDER_CONF;  // 0.4
  }
  // Reverse Y offset sign for right sensor for legible comparison on the
  // following if cond
  if (!pGenObj->bRightSensor) {
    fAbsYOffsetBorder = pRoad->fYOffsetFused_met - 0.5 * fVehicleWidth;
  } else {
    fAbsYOffsetBorder =
        (-pRoad->fYOffsetFusedOppBorder_met) - 0.5 * fVehicleWidth;
  }

  if (pRoad->fConfYOffset_per > fMinConfBorder &&
      fAbsYOffsetBorder < (fObjWidth + fSideOffset)) {
    bLCAObjPathInvalid = TRUE;
  }
  return bLCAObjPathInvalid;
}

/*****************************************************************************
  Functionname: LCACheckWarningConditions                                  */ /*!

  @brief: Check if the current object meets LCA warning conditions

  @description: Check if the current object meets LCA warning conditions

  @param[in]:
			  pGenObj->bObjStable                   Flag that object is stable
			  pLCAObj->bLCAQuality
			  pLCAObj->bLCARelevant
			  pLCAObj->bLCAMirrorObject
			  pLCAObj->bLCAMirrorFrontObject
			  pLCAObj->bLCALaneConditions
			  pLCAObj->bLCAObjPathInvalid
			  pLCAObj->fBehindGrdProb
  @param[out]:
			  void
  @return: bLCAWarningConditions
*****************************************************************************/
boolean LCACheckWarningConditions(const LCAGenObjInfo_t* pGenObj,
                                  LCAObjInfo_t* pLCAObj,
                                  uint8 uLCAGearPosition) {
  boolean bLCAWarningConditions = FALSE;

  if (1 != uLCAGearPosition && pGenObj->bObjStable == TRUE &&
      pLCAObj->bLCAQuality == TRUE && pLCAObj->bLCARelevant == TRUE &&
      // pLCAObj->bLCAMirrorObject == FALSE &&
      // pLCAObj->bLCAMirrorFrontObject == FALSE &&
      pLCAObj->bLCALaneConditions == TRUE &&
      pLCAObj->bLCAObjPathInvalid == FALSE &&
      pLCAObj->fBehindGrdProb_per < LCA_WARNING_BEHINDGRD_THRESH)  // 0.7
  {
    bLCAWarningConditions = TRUE;
  }
  return bLCAWarningConditions;
}

/*****************************************************************************
  Functionname: LCAFinalWarningDecision                                  */ /*!

  @brief: Checks if the current object shall enable a LCA warning

  @description: Checks if the current object shall enable a LCA warning

  @param[in]
			pLCAObj->bLCAWarning                LCA warning flag last cycle
			pLCAObj->bLCAWarningConditions      Flag of meeting all LCA Warning conditions
			pLCAObj->fTTCThreshold              TTC threshold
			pLCAObj->bInLCARange
			pLCAObj->bUpdateRecently            
			pLBSObj->fTTC_s                     The object TTC time to ego vehicle
			pLBSObj->fTTCAccel_mps2             TTC including acceleration
			pLBSObj->fTTCFiltered_s
			pGenObj->fDistX_met                 Object's longitudinal relative distance
			pGenObj->fVrelX_mps                 Object's longitudinal relative velocity
			pGenObj->uiHighestAssocProb_per
			fBSDZoneXMin
  @param[out]
			pLCAWarnInfo->bLCAWarnActive        LCA warning active flag
			pLCAObj->bLCAWarning                LCA warning flag
  @return:void
*****************************************************************************/
boolean LCAFinalWarningDecision(const LCAGenObjInfo_t* pGenObj,
                                const LCALBSObjInfo_t* pLBSObj,
                                LCAObjInfo_t* pLCAObj, boolean bInLCARange,
                                boolean bLCAWarningConditions,
                                boolean bLCAWarningLastCycle,
                                float32 fBSDZoneXMin,
                                LCAWarnInfo_t* pLCAWarnInfo) {
  boolean bEnableLCAWarning = FALSE;
  //const float32 fXDiffBSDZone =
  //fBSDZoneXMin - (pGenObj->fDistX_met + pGenObj->fLengthFront_met);
  boolean bLCAWarning = FALSE;
  // boolean bLCAWarningLastCycle = pLCAObj->bLCAWarning;

  // Use TTC without acceleration for now
  if (bLCAWarningConditions == TRUE) {
    if (pLBSObj->fTTC_s < pLCAObj->fTTCThreshold) {
      if (bLCAWarningLastCycle == TRUE) {
        bEnableLCAWarning = TRUE;
      } else {
        // object does not warn yet
        // Check:
        // - x position is behind LCA_MAX_X_WARN_ACTIVATION
        // - TTC including acceleration is not invalid
        // - Object is within the currently calculated LCA range
        // - Object was updated in the last two cycles
        // - highest association probability is large enough
        if (pGenObj->fDistX_met < LCA_MAX_X_WARN_ACTIVATION &&  //-2
            (pLBSObj->fTTCAccel_mps2 < (TUE_C_F32_VALUE_INVALID - 1.0F) ||
             pGenObj->fDistX_met < LCA_MAX_X_TTC_ACCEL_EVAL          //-20
             || pGenObj->fVrelX_mps > LCA_MAX_VX_TTC_ACCEL_EVAL) &&  // 5
            pLCAObj->bInLCARange == TRUE &&
            pLCAObj->bUpdateRecently == TRUE &&
            pGenObj->uiHighestAssocProb_per > LCA_MIN_ASSOC_PROB_WARNING)  // 70
        {
          // Check if the object is closer to the subject than
          // LCA_MIN_X_TTC_FILTERED If this is the case, perform an additional
          // check for the filtered TTC
#if (LCA_CFG_USE_TTCFILT_ON_WARN_START)
          if (pGenObj->fDistX_met > LCA_MIN_X_TTC_FILTERED) {
            if (pLBSObj->fTTCFiltered_s < pLCAObj->fTTCThreshold) {
              bEnableLCAWarning = TRUE;
            }
          } else {
#endif
            bEnableLCAWarning = TRUE;
#if (LCA_CFG_USE_TTCFILT_ON_WARN_START)
          }
#endif
        }
      }
    } else {
      // For close already warning objects check also for the fast filtered TTC
      // to stabilize the LCA warning For objects that are about to enter the
      // BSD zone and currently warning use a larger TTC threshold
      if (bLCAWarningLastCycle == TRUE) {
        if ((pLBSObj->fTTCFiltered_s < pLCAObj->fTTCThreshold &&
             pGenObj->fDistX_met > LCA_MIN_X_TTC_FILTERED) ||  //-10
            ((fBSDZoneXMin - pGenObj->fDistX_met) <
                 LCA_WARN_BSDZONE_XDIFF_MAX &&  // 1
             pLBSObj->fTTC_s <
                 (LCA_WARN_ACTIVE_TTC_FACTOR * pLCAObj->fTTCThreshold)))  // 1.5
        {
          bEnableLCAWarning = TRUE;
        }
      }
    }
  }

  if (bEnableLCAWarning) {
    pLCAWarnInfo->bLCAWarnActive = TRUE;
    if (pGenObj->bRightSensor) {
      pLCAWarnInfo->bLCAWarnActiveRight = TRUE;
      pLCAWarnInfo->bLCAWarnActiveLeft = FALSE;
    }
    if (!pGenObj->bRightSensor) {
      pLCAWarnInfo->bLCAWarnActiveLeft = TRUE;
      pLCAWarnInfo->bLCAWarnActiveRight = FALSE;
    }

    bLCAWarning = TRUE;
  }
  return bLCAWarning;
}

/*****************************************************************************
  Functionname: LCAStoreWarningObjInfo                                  */ /*!

  @brief: Store the warning information of the closest warning object

  @description: Store the warning information of the closest warning object

  @param[in]:
			  uObjIndex                Object ID
			  bLCAWarning              LCA warning flag
			  fDistX_met               Object's longitudinal relative distance
  @param[out]:
			  fXObjectWarning_met      Object's longitudinal relative distance of LCA warning condition
			  uLCAWarningID_nu         LCA warning object ID
  @return:void
*****************************************************************************/
void LCAStoreWarningObjInfo(uint8 uObjIndex, const LCAGenObjInfo_t* pGenObj,
                            LCAObjInfo_t* pLCAObj,
                            const LCALBSObjInfo_t* pLBSObj,
                            LCAWarnInfo_t* pLCAWarnInfo) {
  pLCAWarnInfo->bLCAWarningLastCycle = pLCAWarnInfo->bLCAWarnActive;
  if (pLCAObj->bLCAWarning == TRUE) {
    if (pGenObj->fDistX_met > pLCAWarnInfo->fXObjectWarning_met) {
      pLCAWarnInfo->fXObjectWarning_met =
          pGenObj->fDistX_met + pGenObj->fLengthFront_met;
      pLCAWarnInfo->uLCAWarningID_nu = uObjIndex;
      pLCAWarnInfo->fCriticalTTC_s = pLBSObj->fTTC_s;
    }
  }

  // Write LCA warning information to LCA globals
  if (!(pLCAWarnInfo->bLCAWarnActive &&
        pLCAWarnInfo->uLCAWarningID_nu < EM_N_OBJECTS)) {
    pLCAWarnInfo->uLCAWarningID_nu = TUE_C_UI8_VALUE_INVALID;
    pLCAWarnInfo->fCriticalTTC_s = TUE_C_F32_VALUE_INVALID;
  }
}