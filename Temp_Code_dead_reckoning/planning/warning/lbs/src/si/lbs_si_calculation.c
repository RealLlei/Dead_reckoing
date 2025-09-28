/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "lbs_si_calculation.h"
#include "lbs_si.h"
#include "lbs_si_par.h"
#include "planning/warning/lbs/common/tue_common_libs.h"

/*****************************************************************************
  TOOLS FUNCTION
*****************************************************************************/
SICalculate_st* GetSICalculatePointer() {
  return &SICalculate;
}

SI_Globals_t* GetSICalculatePointer_SIGlobals() {
  return &SICalculate.SIGlobals;
}

SI_Info_st* GetSICalculatePointer_SIObjInfo(uint8 uObj) {
  return &SICalculate.SIObjInfoList[uObj];
}

eAssociatedLane_t* GetSICalculatePointer_AssociatedLane(uint8 uObj) {
  return &SICalculate.eAssociatedLaneList[uObj];
}

SIPredictedDistance_t* GetSICalculatePointer_SIPredictedDist() {
  return &SICalculate.SIPredictedDist;
}

const SILCAObjInfo_t* pSIGetLCAObjInfoPointer(
    uint8 uObj, const SILCAObjInfo_Array* pLCAObjInfoList) {
  return pLCAObjInfoList[uObj];
}

// SILBSObjInfo_st* pSIGetLBSObjInfoPointer(uint8 uObj, LBSObjInfo_Array*
// pLBSObjInfoList)
//{
//	return pLBSObjInfoList[uObj];
//}

SIGenObject_st* pSIGetGenObjListPointer_Object(uint8 uObj,
                                               SIGenObjList_st* pGenObjList) {
  // todo check obj number range
  return &pGenObjList->aObject[uObj];
}

// SISRRObject_st* pSIGetSRRObjListPointer_Object(uint8 uObj, EMSRRObjList_st*
// pSRRObjList)
//{
//	return &pSRRObjList->aObject[uObj];
//}

/*****************************************************************************
  BASE FUNCTION
*****************************************************************************/
/*****************************************************************************
  Functionname: SIApproximateRefpoint                                  */ /*!

  @brief:Calculate x value of the normal crossing from object to clothoid
		 tangent at last approximated X value

  @description:Calculate x value of the normal crossing from object to clothoid
			   tangent at last approximated X value
					x
					^
					|
					|    AUTOSAR coordinate system
			   y<----
  @param[in]:fX :X coordinate of given point
			 fY :Y coordinate of given point
			 fC0:C0 of 3 degree clothoid polynom
			 fC1: C1 of 3 degree clothoid polynom
  @param[out]: pfRefX :last approximated X value Reference pointer

  @return: void
*****************************************************************************/
void SIApproximateRefpoint(float32 fX, float32 fY, float32 fC0, float32 fC1,
                           float32 fAngle, float32* pfRefX) {
  float32 fXc = *pfRefX;
  float32 fXXc = SQR(fXc);
  float32 fC1XXc = fC1 * fXXc;
  float32 fC0Xc = fC0 * fXc;
  float32 fYc =
      (TUE_C_SIXTH * fC1XXc * fXc) + (0.5F * fC0Xc * fXc) + (fAngle * fXc);
  float32 fm = ((0.5F * fC1XXc) + fC0Xc + fAngle);
  float32 fm_inv = 0.F;

  if (fABS(fm) < TUE_C_F32_DELTA) {
    *pfRefX = fX;
  } else {
    fm_inv = 1.0F / fm;
    *pfRefX = (((fY - fYc) + (fm * fXc)) + (fm_inv * fX)) / (fm + fm_inv);
  }
}

/*****************************************************************************
  Functionname: SICalculateDistancePoint2Clothoid */ /*!

  @brief:Calculate lateral distance between(X,Y)point and clothoid

  @description:Calculate lateral distance between(X,Y)point and clothoid
					x
					^
					|
					|    AUTOSAR coordinate system
			   y<----
  @param[in]:fX :X coordinate of given point
			 fY :Y coordinate of given point
			 fC0: Beginning of curvature
			 fC1: Constant curvature change rate
			 fAngle:

  @return: ReferencePoint Distance to clothoid
*****************************************************************************/
SITrajRefPoint_t SICalculateDistancePoint2Clothoid(float32 fX, float32 fY,
                                                   float32 fC0, float32 fC1,
                                                   float32 fAngle) {
  float32 fTemp = 0.F;
  float32 fYDiff = 0.F;
  SITrajRefPoint_t ReferencePoint;

  ReferencePoint.fX_met = fX;
  SIApproximateRefpoint(fX, fY, fC0, fC1, fAngle, &ReferencePoint.fX_met);

  fTemp = SQR(ReferencePoint.fX_met);
  ReferencePoint.fY_met = (fAngle * ReferencePoint.fX_met) +
                          (0.5F * fC0 * fTemp) +
                          ((fC1 * fTemp * ReferencePoint.fX_met) * TUE_C_SIXTH);

  fYDiff = fY - ReferencePoint.fY_met;

  if (fABS(fYDiff) < TUE_C_F32_VALUE_INVALID) {
    ReferencePoint.fDistToTraj_met =
        SQRT(SQR(fX - ReferencePoint.fX_met) + SQR(fYDiff));
  } else {
    ReferencePoint.fDistToTraj_met = TUE_C_F32_VALUE_INVALID;
  }

  if (fYDiff < 0.0F) {
    ReferencePoint.fDistToTraj_met *= -1.0F;
  }

  ReferencePoint.fDistOnTraj_met = ReferencePoint.fX_met;

  return ReferencePoint;
}

/*****************************************************************************
  CALCULATION FUNCTION
*****************************************************************************/
/*****************************************************************************
  Functionname: SICalcRelTraTrackWidthSeek                                  */ /*!

  @brief:Get seek lane width to use

  @description:Get the seek lane width to use(the width when not following the object)
		 This width depend on the road type and the ego velocity

  @param[in]:pEgoInfo: Ego Vehicle information pointer

  @return: fret:The seek lane width
*****************************************************************************/
float32 SICalcRelTraTrackWidthSeek(const SIVehicleInfo_t* pEgoInfo,
                                   const SIParam_st* params) {
  float32 fret = 0.F;

  // Use estimated lane width as seek lane width
#if SI_DYNAMIC_LANE_BRACKET_EXTENSION
  SI_Globals_t* pSIGlobal = GetSICalculatePointer_SIGlobals();
  fret = pSIGlobal->fLaneWidth_met;
#else
  const SI_LCAZone_t* pLCAZone = &params->LCAParamter.LCAZone;
  const float32 fLCAZoneYMaxNear_met = pLCAZone->fLCAZoneYMaxNear_met;
  const float32 fLCAZoneYMinNear_met = pLCAZone->fLCAZoneYMinNear_met;
  fret = (fLCAZoneYMaxNear_met - fLCAZoneYMinNear_met);
#endif

  return fret;
}

/*****************************************************************************
  Functionname: SICalcRelTraTrackWidthTrack                                  */ /*!

  @brief:Get track lane width to use

  @description:Get the track lane width to use(when already following object)
			   This width depends on the road type and the ego velocity

  @param[in]: pEgoInfo: Ego Vehicle information pointer

  @return:fret The track lane width
*****************************************************************************/
float32 SICalcRelTraTrackWidthTrack(const SIVehicleInfo_t* pEgoInfo,
                                    const SIParam_st* params) {
  float32 fret = 0.F;
  const float32 fEgoSpeed = pEgoInfo->fegoVelocity_mps;

  // Assuming RoadType as Highway because no other information is available
  // fret = TUE_CML_BoundedLinInterpol(&TRCKSeekLaneWidthSpeedAdapted,
  //                                   fEgoSpeed);  // 3.75
  fret = TUE_CML_BoundedLinInterpol2(  //3.75
      fEgoSpeed, SI_MIN_SPEED_HIGHWAYLANEWIDTH, SI_MAX_SPEED_HIGHWAYLANEWIDTH,
      SI_HIGHWAYLANEWIDTH_SEEK_MIN, SI_HIGHWAYLANEWIDTH_SEEK_MAX);
  //(ego_velocity,26.1,44.4,3.75,3.75)
  // fret += TUE_CML_BoundedLinInterpol(
  //     &TRCKTrackOffsetSpeedAdapted,
  //     fEgoSpeed);  //[fEgoSpeed, 130/3.6, 160/3.6, 0.7, 1.1]
  fret += TUE_CML_BoundedLinInterpol2(fEgoSpeed, SI_MIN_SPEED_HIGHWAYLANEWIDTH,
                                      SI_MAX_SPEED_HIGHWAYLANEWIDTH,
                                      SI_HIGHWAYLANEWIDTH_TRACK_OFFSET_MAX,
                                      SI_HIGHWAYLANEWIDTH_TRACK_OFFSET_MIN);
  //(ego_velocity,26.1,44.4,1.1,0.7)
  //3.75 +[1.876,-2.94] = [5.626,0.81]
#if (!SI_DYNAMIC_LANE_BRACKET_EXTENSION)
  const SI_LCAZone_t* pLCAZone = &params->LCAParamter.LCAZone;
  float32 fLCAZoneYMinNear_met = pLCAZone->fLCAZoneYMinNear_met;  // 0.6
  fret -= fLCAZoneYMinNear_met;                                   //[5.026,0.21]
#endif
  return fret;
}

/*****************************************************************************
  Functionname: SIAdvantageRelevantObject                                  */ /*!

  @brief:Update object corridor information for given object

  @description:Update object corridor information for given object

  @param[in]: uObj: Array index of the object to update times for
			  pGenObj: the EM General object pointer
			  pSRRObj: the EM SRR object pointer
			  pSIObj:  the SI inner calculation object struct pointer
  @return:void
*****************************************************************************/
void SIAdvantageRelevantObject(
    uint8 uObj, const SIGenObject_st* pGenObj, const SILCAObjInfo_t* pLCAObj,
    const float32 fCycletime_s)  // SISRRObject_st* pSRRObj,
{
  /*Update relevant object lane extension factor time base */
  SIUpdateTimeTrackLaneExtFactor(uObj, pGenObj, pLCAObj, fCycletime_s);
  /*Update relevant object lane extension factor distance base */
  SIUpdateDistTrackLaneExtFactor(uObj, pGenObj, pLCAObj);  // pSRRObj,
}

/*****************************************************************************
  Functionname: SIUpdateTimeTrackLaneExtFactor */ /*!

  @brief:Update relevant object lane extension factor time based

  @description:This function updates the relevant object lane extension factor.
			   If an object becomes relevant then the trace brackets get extended
			   (wider) depending on the distance the given object is in.The
			   Closer the object the faster the lane extension factor goes up.
			   This extension is done for Objects that are closer than SI_TRACK_SAFATY.
			   When an object goes farther than SI_TRACK_SAFATY,then the factor gets
			   decreased again,For objects that are no longer relevant the factor gets
			   decreased quickly.

  @param[in]: uObj: Array index of the object to update times for
			  pGenObj: the EM General object pointer
					pGenObj->GenObjInfo.fDistX_met
					pGenObj->GenObjInfo.fVrelX_mps
			  pSIObj:  the SI inner calculation object struct pointer
					pSIObj->ObjCor.fRelTraceExtensionFactor_nu
					pSIObj->ObjLaneLCAStatus.uInlaneCycleCounter_nu

			  fCycleTime        Current task cycle time
  @return:void
*****************************************************************************/
void SIUpdateTimeTrackLaneExtFactor(uint8 uObj, const SIGenObject_st* pGenObj,
                                    const SILCAObjInfo_t* pLCAObj,
                                    float32 fCycleTime) {
  SI_Info_st* pSIObj = GetSICalculatePointer_SIObjInfo(uObj);
  float32 fObjExtensionFactor = pSIObj->ObjCor.fRelTraceExtensionFactor_nu;
  const float32 fXObj = pGenObj->GenObjInfo.fDistX_met;
  const float32 fVxObj = pGenObj->GenObjInfo.fVrelX_mps;
  float32 fIncDecFactor = 0.F;
  float32 fAbsFactor = 0.F;

  if (pLCAObj->bLCAWarning == TRUE) {
    // Object within track width for own track
    if (fXObj > SI_TRACK_SAFATY)  //-70
    {
      // Object within the distance for track safety
      // the closer, the smaller the fAbsFactor factor
      fAbsFactor = fXObj * (1.0F / SI_TRACK_SAFATY);
      fIncDecFactor =
          fCycleTime / (SI_WHEELTRACK_MIN +
                        ((SI_WHEELTRACK_MAX - SI_WHEELTRACK_MIN) * fAbsFactor));
      // Increase time based extension factor faster for object with high
      // relative velocity
      if (fObjExtensionFactor > 0.1F) {
        fIncDecFactor *= GDBmathLinFuncLimBounded(
            fVxObj, SI_TIMEEXT_VRELX_MIN, SI_TIMEEXT_VRELX_MAX,
            SI_TIMEEXT_TIME_MIN, SI_TIMEEXT_TIME_MAX);
      }
    } else {
      // Object out of distance for track safety the further away, the
      // smaller the distance factor
      fAbsFactor = 1.0F - ((fXObj - SI_TRACK_SAFATY) *
                           (1.0F / (SI_FAR_RANGE - SI_TRACK_SAFATY)));
      fIncDecFactor =
          -(fCycleTime /
            (SI_WHEELTRACK_MIN +
             ((SI_WHEELTRACK_MAX - SI_WHEELTRACK_MIN) * fAbsFactor)));
    }
  } else {
    // object outside the track width for own track
    fIncDecFactor = -(fCycleTime * (1.0F / SI_WHEELTRACK_DEC));  //-0.07575
  }
  fObjExtensionFactor += fIncDecFactor;
  fObjExtensionFactor = TUE_CML_MinMax(0.0F, 1.0F, fObjExtensionFactor);
  pSIObj->ObjCor.fRelTraceExtensionFactor_nu = fObjExtensionFactor;
}

/*****************************************************************************
  Functionname: SIUpdateDistTrackLaneExtFactor */ /*!

  @brief:Update relevant object lane extension factor distance based

  @description:If an object becomes relevant,then wider trace brackets are
			   used then in the seek case.Depending on the object longitudinal
			   distance change the trace brackets can be extended.If the object
			   is not relevant,this extension factor is reset to zero.
			   The calculated factor itself is a value between[0,1]

  @param[in]: uObj: Array index of the object to update
			  pGenObj: the EM General object pointer
					pGenObj->GenObjInfo.fDistX_met
					pGenObj->GenObjInfo.fVrelX_mps
			  pSRRObj: the EM SRR object pointer
					pSRRObj->History.fFirstDetectX_met
			  pSIObj:  the SI inner calculation object struct pointer
					pSIObj->ObjCor.fRelTraceDistExtensionFactor_nu
			  pLCAObj->bLCAWarning


  @return:void
*****************************************************************************/
void SIUpdateDistTrackLaneExtFactor(
    uint8 uObj, const SIGenObject_st* pGenObj,
    const SILCAObjInfo_t* pLCAObj)  // SISRRObject_st* pSRRObj,
{
  SI_Info_st* pSIObj = GetSICalculatePointer_SIObjInfo(uObj);

  float32 fObjExtensionFactor =  //NOLINT
      pSIObj->ObjCor.fRelTraceDistExtensionFactor_nu;
  const float32 fObjDistX = pGenObj->GenObjInfo.fDistX_met;
  const float32 fObjVx = pGenObj->GenObjInfo.fVrelX_mps;
  float32 dAbstandDiff = 0.0F;

  if ((pLCAObj->bLCAWarning == TRUE) && (fObjVx < 10.0F * C_MS_KMH)) {
    // Object within trace brackets (ego lane) and we are approaching it
    dAbstandDiff = (fObjDistX - pGenObj->GenObjInfo.fFirstDetectX_met);

    // 10m - 30m ->fRelTraceDistExtensionFactor range 0-1
    fObjExtensionFactor =
        (dAbstandDiff - SI_TRACK_ABSTANDDIFF_MIN) *
        (1.0F / (SI_TRACK_ABSTANDDIFF_MAX - SI_TRACK_ABSTANDDIFF_MIN));
    fObjExtensionFactor = TUE_CML_MinMax(0.0F, 1.0F, fObjExtensionFactor);
  } else {
    // Object not relevant or we are not approaching it with a sufficiently
    // large speed difference,reset extension factor
    fObjExtensionFactor = 0.0F;
  }

  pSIObj->ObjCor.fRelTraceDistExtensionFactor_nu = fObjExtensionFactor;
}

/*****************************************************************************
  Functionname: SICalcCriteriaMatrix                                  */ /*!

  @brief:Calculate the lane assignment (trace bracket) matrix

  @description:Calculate the lane assignment (trace bracket) matrix

  @param[in]:uObj: Array index of the object to update

  @return:void
*****************************************************************************/
void SICalcCriteriaMatrix(uint8 uObj, const SIInPut_st* reqPorts,
                          const SIParam_st* params) {
  const SIGenObject_st* pGenObj = &reqPorts->GenObjList.aObject[uObj];
  // SISRRObject_st* pSRRObj = &reqPorts->SRRObjList.aObject[uObj];
  const SILCAObjInfo_t* pLCAObjInfo = &reqPorts->LCAObjInfoList[uObj];
  const SIVehicleInfo_t* pEgoVehInfo = &reqPorts->EgoVehInfo;
  const SI_LCAParameter_st* pLCAParamter = &params->LCAParamter;

  const SIRoad_t* pRoad = &reqPorts->Road;
  const SISensorMounting_t* pSensorMount = &params->SensorMounting.SensorLeft;
  const SIVehParameter_t* pVehPar = &params->VehParameter;
  SI_Info_st* pSIObj = GetSICalculatePointer_SIObjInfo(uObj);

  SICriteriaMatrix_t CriteraMaxtrix;
  SITrajRefPoint_t TrajRefPoint;
  SITrajectoryData_t TrajectoryData;
  float32 fDistToTrajLastCycle = 0.F;
  const float32 fObjX =
      pGenObj->GenObjInfo.fDistX_met - fABS(pSensorMount->fLongPosToCoG_met);
  const float32 fObjY = pGenObj->GenObjInfo.fDistY_met;

  /*Init trace bracket matrix*/
  SIInitCriteriaMatrix(&CriteraMaxtrix);

  /*Init the Trajectory data,object have been translate AUTOSAR coordinate*/
  TrajectoryData.fTrajC0_1pm = pRoad->fC0Fused_1pm;
  TrajectoryData.fTrajC1_1pmm = pRoad->fC1Fused_1pm2;
  TrajectoryData.fTrajAngle_rad = pRoad->fYawAngleFused_rad;

  /*Store the distance to the trajectory from the last cycle*/
  fDistToTrajLastCycle = pSIObj->ObjTrajRefPoint.fDistToTraj_met;

  /*Calculate the distance to the trajectory from the current cycle*/
  SICalculateDistance2Traj(fObjX, fObjY, &TrajectoryData, &TrajRefPoint);

  /*Copy the trajRefPoint to the SI object*/
  pSIObj->ObjTrajRefPoint.fX_met = TrajRefPoint.fX_met;
  pSIObj->ObjTrajRefPoint.fY_met = TrajRefPoint.fY_met;
  pSIObj->ObjTrajRefPoint.fDistOnTraj_met = TrajRefPoint.fDistOnTraj_met;
  pSIObj->ObjTrajRefPoint.fDistToTraj_met = TrajRefPoint.fDistToTraj_met;

  /*Calculate the vrel to the trajectory based on last and current distance to
   * the trajectory*/
  SICalculateVrel2Traj(uObj, pGenObj, fDistToTrajLastCycle,
                       TrajRefPoint.fDistToTraj_met,
                       reqPorts->SISysParam.fCycletime_s);

  /*Determine seek lane width to use for the object*/
  SISetBaseSeekWidth(pGenObj, &CriteraMaxtrix);

  if (pGenObj->GenObjInfo.eDynamicProperty_nu ==
      EM_GEN_OBJECT_DYN_PROPERTY_MOVING) {
    /*Check lane association for track case*/
    SICheckTrackCriteria(uObj, pGenObj, pRoad, pVehPar, &CriteraMaxtrix,
                         pEgoVehInfo, pLCAParamter, pLCAObjInfo);
  }

  /*Set the trace bracket position based on the extensions and restrictions*/
  SISetBracketPosition(pGenObj, pSIObj, &CriteraMaxtrix);
}

/*****************************************************************************
  Functionname: SIInitCriteriaMatrix                                  */ /*!

  @brief:Initialize lane assignment criteria matrix

  @description:Initialize lane assignment criteria matrix

  @param[in]:pCriteriaMatrix The criteria matrix to initialize

  @return:void
*****************************************************************************/
void SIInitCriteriaMatrix(SICriteriaMatrix_t* pCriteraMaxtrix) {
  // Define criteria for trace bracket width
  SIInitCriteriaOutput(&(pCriteraMaxtrix->TrackWidthSeek), DEFAULT_MODE);
  SIInitCriteriaOutput(&(pCriteraMaxtrix->TrackWidthTrck), DEFAULT_MODE);
  // Criteria define new trace bracket width
  SIInitCriteriaOutput(&(pCriteraMaxtrix->RestrictionDistDependent),
                       RESTRICTION_MODE);
  SIInitCriteriaOutput(&(pCriteraMaxtrix->RestrictionCurve), RESTRICTION_MODE);
  SIInitCriteriaOutput(&(pCriteraMaxtrix->ExtensionRoadBorder), EXTENSION_MODE);
  SIInitCriteriaOutput(&(pCriteraMaxtrix->ExtensionCriticalTTC),
                       EXTENSION_MODE);
}

/*****************************************************************************
  Functionname: SIInitCriteriaOutput                                  */ /*!

  @brief:Initialize one criteria result bracket

  @description:Initialize one criteria result bracket

  @param[in]:eMode mode of initialization of the bracket

  @return:void
*****************************************************************************/
void SIInitCriteriaOutput(SIBracketOutput_t* pOutput,
                          SITraceBracketMode_t eMode) {
  switch (eMode) {
    case RESTRICTION_MODE:
      pOutput->fBracketPositionLeft_met = INITVALUE_BRACKETPOSITION;
      pOutput->fBracketPositionRight_met = -INITVALUE_BRACKETPOSITION;
      break;
    case EXTENSION_MODE:
      pOutput->fBracketPositionLeft_met = -INITVALUE_BRACKETPOSITION;
      pOutput->fBracketPositionRight_met = INITVALUE_BRACKETPOSITION;
      break;
    case DEFAULT_MODE:
    default:
      pOutput->fBracketPositionLeft_met = 0.0F;
      pOutput->fBracketPositionRight_met = 0.0F;
      break;
  }
}

/*****************************************************************************
  Functionname: SICalculateDistance2Traj                                  */ /*!

  @brief:Get trajectory reference point

  @description:Get the trajectory reference point information for given(fX,fY)
			   point using the passed trajectory
					x
					^
					|
					|    AUTOSAR coordinate system
			   y<----

  @param[in]:fX : X coordinate to get trajectory distance of
			 fY : Y coordinate to get trajectory distance of
			 pTrajData: The trajectory object to use

  @param[out]: pDist2Traj:The calculated distance to trajectory (result)

  @return:void
*****************************************************************************/
void SICalculateDistance2Traj(float32 fX, float32 fY,
                              const SITrajectoryData_t* pTrajData,
                              SITrajRefPoint_t* pDist2Traj) {
  *pDist2Traj = SICalculateDistancePoint2Clothoid(
      fX, fY, pTrajData->fTrajC0_1pm, pTrajData->fTrajC1_1pmm,
      pTrajData->fTrajAngle_rad);
}

/*****************************************************************************
  Functionname: SICalculateVrel2Traj                                  */ /*!

  @brief:Calculates the vrel to the trajectory

  @description:Calculates the vrel to the trajectory

  @param[in]:uObj Array index of the object to update
			 pGenObj the EM General object pointer
			 fDistToTrajLastCycle Distance to trajectory of the object in the last cycle
			 fDistToTrajCurrentCycle Distance to trajectory of the object in current cycle

  @param[out]:pSIObj->fVrelToTraj_mps:Object's relative speed to trajectory
  @return:void
*****************************************************************************/
void SICalculateVrel2Traj(uint8 uObj, const SIGenObject_st* pGenObj,
                          float32 fDistToTrajLastCycle,
                          float32 fDistToTrajCurrentCycle,
                          float32 fTaskCycleTime) {
  SI_Info_st* pSIObj = GetSICalculatePointer_SIObjInfo(uObj);
  float32 fVrelToTraj = 0.0F;

  if ((pGenObj->GenObjInfo.uiLifeCycles_nu > 1U) &&
      (fTaskCycleTime > TUE_C_F32_DELTA)) {
    fVrelToTraj =
        (fDistToTrajCurrentCycle - fDistToTrajLastCycle) / fTaskCycleTime;
  } else {
    fVrelToTraj = 0.0F;
    pSIObj->fVrelToTraj_mps = 0.0F;
  }

  GDB_Math_LowPassFilter(&pSIObj->fVrelToTraj_mps, fVrelToTraj,
                         SI_VREL_TO_TRAJ_FILTER_CONST);
}

/*****************************************************************************
  Functionname: SISetBaseSeekWidth                                  */ /*!

  @brief:Set the basic seek lane width

  @description:Set the basic seek lane width

  @param[in]: pGenObj: the EM General object pointer
			  pCriteraMaxtrix: the Criteria matrix containing seek and track brackets

  @return:void
*****************************************************************************/
void SISetBaseSeekWidth(const SIGenObject_st* pGenObj,
                        SICriteriaMatrix_t* pCriteraMaxtrix) {
  float32 fObjX = pGenObj->GenObjInfo.fDistX_met;
  SI_Globals_t* pSIGlobal = GetSICalculatePointer_SIGlobals();
  float32 fHalfLaneWidth = 0.F;

  /*Determine seek lane width to use for the object*/
  if (pGenObj->GenObjInfo.eDynamicProperty_nu !=
      EM_GEN_OBJECT_DYN_PROPERTY_STATIONARY) {
    fHalfLaneWidth = pSIGlobal->fSIseekLaneWidth_met * 0.5F;
  } else {
    /*For stationary objects limit the seek lane width,as most roadside
    stationaries are seen as 'point' class objects in large distances,limit
    lane width more for far away points. Note:in tight turns valid in-lane
    objects also tend to be points.thus do not limit near
    range objects that much*/
    if ((pGenObj->GenObjInfo.eClassification_nu !=
         LBS_EM_GEN_OBJECT_CLASS_POINT) ||
        (fObjX > SI_PAR_MAX_DISTX_STAT_POINT_SMALL_SEEK)) {
      /*Limit the seek lane width to 'SI_ACC_TRAJECTORY_WIDTH_STAT' at
       * most*/
      fHalfLaneWidth =
          MIN(pSIGlobal->fSIseekLaneWidth_met, SI_ACC_TRAJECTORY_WIDTH_STAT) *
          0.5F;
    } else {
      /*Limit the seek lane width to 'SI_ACC_NARROWWIDTH_STAT' at most*/
      fHalfLaneWidth = MIN(pSIGlobal->fSIseekLaneWidth_met,
                           SI_ACC_TRAJECTORY_NARROWWIDTH_STAT) *
                       0.5F;
    }
  }
  /*according to object dynamic property choose difference base width*/
  pCriteraMaxtrix->TrackWidthSeek.fBracketPositionRight_met = -fHalfLaneWidth;
  pCriteraMaxtrix->TrackWidthSeek.fBracketPositionLeft_met = fHalfLaneWidth;
}

/*****************************************************************************
  Functionname: SISetBracketPosition                                  */ /*!

  @brief: Set trace bracket for the object

  @description: Set trace bracket for the object

  @param[in]:

  @return:void
*****************************************************************************/
void SISetBracketPosition(const SIGenObject_st* pGenObj, SI_Info_st* pSIObj,
                          SICriteriaMatrix_t* pCriteraMaxtrix) {
  SIObjCorridor_t* pObjCor = &pSIObj->ObjCor;
  float32 TrackWidthRight = 0.F;
  float32 TrackWidthLeft = 0.F;
  float32 TrackWidthExtRight = 0.F;
  float32 TrackWidthExtLeft = 0.F;
  float32 TrackWidthResRight = 0.F;
  float32 TrackWidthResLeft = 0.F;

  /*Calculation of the base lane width left and right*/
  SISetBaseTrackWidth(&TrackWidthLeft, &TrackWidthRight, pCriteraMaxtrix);
  /*Set extended trace brackets*/
  SISetTrackWidthExtension(&TrackWidthExtLeft, &TrackWidthExtRight,
                           pCriteraMaxtrix);
  // Set the lane width,use the wider bracket because extension mode
  TrackWidthRight = TUE_CML_Min(TrackWidthRight, TrackWidthExtRight);
  TrackWidthLeft = TUE_CML_Max(TrackWidthLeft, TrackWidthExtLeft);
  /*Set restricted trace brackets*/
  SISetTrackWidthRestriction(&TrackWidthResRight, &TrackWidthResLeft,
                             pCriteraMaxtrix);
  // Set the lanewidth
  TrackWidthRight = MAX(TrackWidthRight, TrackWidthResRight);
  TrackWidthLeft = MIN(TrackWidthLeft, TrackWidthResLeft);
  /*Write trace brackets to output*/
  SISetRightBracket(&pObjCor->fTraceBracketOffsetRight_met, TrackWidthRight,
                    DEFAULT_MODE);
  SISetLeftBracket(&pObjCor->fTraceBracketOffsetLeft_met, TrackWidthLeft,
                   DEFAULT_MODE);
  // Final check of the trace bracket position
  if (pObjCor->fTraceBracketOffsetLeft_met <
      pObjCor->fTraceBracketOffsetRight_met) {
    SISetLeftBracket(&pObjCor->fTraceBracketOffsetLeft_met,
                     (float32)(pObjCor->fTraceBracketOffsetRight_met + 0.1F),
                     DEFAULT_MODE);
  }
}

/*****************************************************************************
  Functionname: SISetBaseTrackWidth                                  */ /*!

  @brief:Set base trace brackets based on largest interval between seek and track mode

  @description:Set base trace brackets based on largest interval between seek and track mode

  @param[in]:

  @return:void
*****************************************************************************/
void SISetBaseTrackWidth(float32* pTrackWidthLeft, float32* pTrackWidthRight,
                         SICriteriaMatrix_t* pCriteraMaxtrix) {
  // Set bracket width for seek case
  *pTrackWidthLeft = pCriteraMaxtrix->TrackWidthSeek.fBracketPositionLeft_met;
  *pTrackWidthRight = pCriteraMaxtrix->TrackWidthSeek.fBracketPositionRight_met;

  // Set bracket width for track case
  // left side use the maximum position
  if (fABS(pCriteraMaxtrix->TrackWidthTrck.fBracketPositionLeft_met) >
      TUE_C_F32_DELTA) {
    *pTrackWidthLeft =
        TUE_CML_Max(*pTrackWidthLeft,
                    pCriteraMaxtrix->TrackWidthTrck.fBracketPositionLeft_met);
  }
  // ride side use the minimum position,means narrowing the bracket
  if (fABS(pCriteraMaxtrix->TrackWidthTrck.fBracketPositionRight_met) >
      TUE_C_F32_DELTA) {
    *pTrackWidthRight =
        TUE_CML_Min(*pTrackWidthRight,
                    pCriteraMaxtrix->TrackWidthTrck.fBracketPositionRight_met);
  }
}

/*****************************************************************************
  Functionname: SISetTrackWidthExtension                                  */ /*!

  @brief:Set extended trace brackets

  @description:Set extended trace brackets

  @param[in]:

  @return:void
*****************************************************************************/
void SISetTrackWidthExtension(float32* pTrackWidthExtLeft,
                              float32* pTrackWidthExtRight,
                              SICriteriaMatrix_t* pCriteraMaxtrix) {
  float32 fMaxAbsoluteExtensionRight = INITVALUE_BRACKETPOSITION;
  float32 fMaxAbsoluteExtensionLeft = -INITVALUE_BRACKETPOSITION;

  // Calculation of the absolute extension of the brackets
  // Right hand side
  fMaxAbsoluteExtensionRight =
      MIN(fMaxAbsoluteExtensionRight,
          pCriteraMaxtrix->ExtensionRoadBorder.fBracketPositionRight_met);
  fMaxAbsoluteExtensionRight =
      MIN(fMaxAbsoluteExtensionRight,
          pCriteraMaxtrix->ExtensionCriticalTTC.fBracketPositionRight_met);
  // Left hand side
  fMaxAbsoluteExtensionLeft =
      MAX(fMaxAbsoluteExtensionLeft,
          pCriteraMaxtrix->ExtensionRoadBorder.fBracketPositionLeft_met);
  fMaxAbsoluteExtensionLeft =
      MAX(fMaxAbsoluteExtensionLeft,
          pCriteraMaxtrix->ExtensionCriticalTTC.fBracketPositionLeft_met);
  // set the bracket width
  *pTrackWidthExtRight = fMaxAbsoluteExtensionRight;
  *pTrackWidthExtLeft = fMaxAbsoluteExtensionLeft;
}

/*****************************************************************************
  Functionname: SISetTrackWidthRestriction                                  */ /*!

  @brief: Set restricted trace brackets

  @description: Set restricted trace brackets

  @param[in]:

  @return:void
*****************************************************************************/
void SISetTrackWidthRestriction(float32* TrackWidthResRight,
                                float32* TrackWidthResLeft,
                                SICriteriaMatrix_t* pCriteraMaxtrix) {
  float32 fMaxAbsoluteRestrictionRight = -INITVALUE_BRACKETPOSITION;
  float32 fMaxAbsoluteRestrictionLeft = INITVALUE_BRACKETPOSITION;

  // Calculate of the absolute restriction of the brackets
  // Right hand side
  fMaxAbsoluteRestrictionRight =
      MAX(fMaxAbsoluteRestrictionRight,
          pCriteraMaxtrix->RestrictionDistDependent.fBracketPositionRight_met);
  fMaxAbsoluteRestrictionRight =
      MAX(fMaxAbsoluteRestrictionRight,
          pCriteraMaxtrix->RestrictionCurve.fBracketPositionRight_met);
  // Left hand side
  fMaxAbsoluteRestrictionLeft =
      MIN(fMaxAbsoluteRestrictionLeft,
          pCriteraMaxtrix->RestrictionDistDependent.fBracketPositionLeft_met);
  fMaxAbsoluteRestrictionLeft =
      MIN(fMaxAbsoluteRestrictionLeft,
          pCriteraMaxtrix->RestrictionCurve.fBracketPositionLeft_met);
  // Set the bracket width
  *TrackWidthResRight = fMaxAbsoluteRestrictionRight;
  *TrackWidthResLeft = fMaxAbsoluteRestrictionLeft;
}

/*****************************************************************************
  Functionname: SISetRightBracket                                  */ /*!

  @brief: Set right bracket position

  @description: Set the right trace bracket position depending on mode:
				For mode RESTRICTION_MODE take the greater of the currently set and new value.
				For mode EXTENSION_MODE take the smaller of the currently set and new value.
				Otherwise at default just set to new value.

  @param[in]:

  @return:void
*****************************************************************************/
void SISetRightBracket(float32* pfTrackBorderRight, float32 fNewValueRight,
                       SITraceBracketMode_t eBracketMode) {
  switch (eBracketMode) {
    case RESTRICTION_MODE:
      *pfTrackBorderRight = MAX(*pfTrackBorderRight, fNewValueRight);
      break;
    case EXTENSION_MODE:
      *pfTrackBorderRight = MIN(*pfTrackBorderRight, fNewValueRight);
      break;
    case DEFAULT_MODE:
      *pfTrackBorderRight = fNewValueRight;
      break;
  }
}

/*****************************************************************************
  Functionname: SISetLeftBracket                                  */ /*!

  @brief: Set left bracket position

  @description: Set the left trace bracket position depending on mode:
				For mode RESTRICTION_MODE take the greater of the currently set and new value.
				For mode EXTENSION_MODE take the smaller of the currently set and new value.
				Otherwise at default just set to new value.
  @param[in]:

  @return:void
*****************************************************************************/
void SISetLeftBracket(float32* pfTrackBorderLeft, float32 fNewValueLeft,
                      SITraceBracketMode_t eBracketMode) {
  switch (eBracketMode) {
    case RESTRICTION_MODE:
      *pfTrackBorderLeft = MIN(*pfTrackBorderLeft, fNewValueLeft);
      break;
    case EXTENSION_MODE:
      *pfTrackBorderLeft = MAX(*pfTrackBorderLeft, fNewValueLeft);
      break;
    case DEFAULT_MODE:
      *pfTrackBorderLeft = fNewValueLeft;
      break;
  }
}

/*****************************************************************************
  Functionname: SICheckTrackCriteria                                  */ /*!

  @brief: Calculate extensions and restructions for track mode

  @description: Calculate extensions and restrictions for track mode based on distance, curve, TTC

  @param[in]:

  @return:void
*****************************************************************************/
void SICheckTrackCriteria(
    uint8 uObj, const SIGenObject_st* pGenObj, const SIRoad_t* pRoad,
    const SIVehParameter_t* pVehPar, SICriteriaMatrix_t* pCriteraMaxtrix,
    const SIVehicleInfo_t* pEgoInfo, const SI_LCAParameter_st* pLCAParamter,
    const SILCAObjInfo_t* pLCAObjInfo)  // SISRRObject_st* pSRRObj,
{
  /*Determination of right and left trace bracket in track mode*/
  SITrackWidthTrck(uObj, pCriteraMaxtrix);

#if SI_DYNAMIC_LANE_BRACKET_EXTENSION
  /*Calculates the extension factor to the left side of the left lane*/
  SIExtensionRoadBorder(uObj, pCriteraMaxtrix, pGenObj, pRoad, pEgoInfo);
  /*Calculate the extension factor to the inner side of the lane*/
  SIExtensionOwnlaneCriticalTTC(uObj, pCriteraMaxtrix, pGenObj, pSRRObj);
  /*Calculates the restriction factor which is depending on the objects
   * distance*/
  SIRestrictionDistanceDependent(uObj, pGenObj, pRoad, pVehPar, pCriteraMaxtrix,
                                 pLCAObjInfo, pLCAParamter);
  /*Calculates the restriction factor which is depending on the curve radius*/
  SIRestrictionCurve(uObj, pCriteraMaxtrix, pGenObj);
#else
#if (SI_LCA_ZONE_BORDER_EXTENSION)
  SIExtensionRoadBorder(uObj, pCriteraMaxtrix, pGenObj, pRoad, pEgoInfo,
                        pVehPar, pLCAParamter);
#endif
  SIRestrictionDistanceDependent(uObj, pGenObj, pRoad, pVehPar, pCriteraMaxtrix,
                                 pLCAParamter);
#endif
}

/*****************************************************************************
  Functionname: SITrackWidthTrck                                  */ /*!

  @brief:Determination of right and left trace bracket in track mode

  @description:Calculate the right and left trace brackets in track mode
			   Basically the trace brackets are extended to the left and
			   right based on the greater of the two factors
			   "fRelTraceExtensionFactor" and "fRelTraceDistExtensionFactor"
			   of the pObjInput structure

  @param[in] uObj            :Array index of the object
			 pCriteraMaxtrix :where the seek and trace bracket is written

  @return:void
*****************************************************************************/
void SITrackWidthTrck(uint8 uObj, SICriteriaMatrix_t* pCriteraMaxtrix) {
  SIObjCorridor_t* pObjCor = &GetSICalculatePointer_SIObjInfo(uObj)->ObjCor;
  SI_Globals_t* pSIGlobal = GetSICalculatePointer_SIGlobals();
  float32 fHalfWidth = 0.F;
  float32 fExtensionFactor = 0.F;

  // Use the larger extension factor of the two
  fExtensionFactor = MAX(pObjCor->fRelTraceExtensionFactor_nu,
                         pObjCor->fRelTraceDistExtensionFactor_nu);

  if (fExtensionFactor > TUE_C_F32_DELTA) {
    fHalfWidth =
        ((pSIGlobal->fSITrackLaneWidth_met - pSIGlobal->fSIseekLaneWidth_met) *
         fExtensionFactor) +
        pSIGlobal->fSIseekLaneWidth_met;
    fHalfWidth *= 0.5F;

    // set left bracket position
    pCriteraMaxtrix->TrackWidthTrck.fBracketPositionLeft_met = fHalfWidth;

    // set right bracket position
    pCriteraMaxtrix->TrackWidthTrck.fBracketPositionRight_met = -fHalfWidth;
  } else {
    /*Object is not relevant*/
  }
}
#if (SI_DYNAMIC_LANE_BRACKET_EXTENSION)
/*****************************************************************************
  Functionname: SIExtensionRoadBorder                                  */ /*!

  @brief:Calculates the extension factor to the left side of the left lane

  @description:Calculates the extension factor to the left side of the left lane
			   if a road border is present and the probability is high that only
			   one lane to the left of the ego vehicle exists.

  @param[in]: uObj Array index of the object

  @return:void
*****************************************************************************/
void SIExtensionRoadBorder(uint8 uObj, SICriteriaMatrix_t* pCriteraMaxtrix,
                           SIGenObject_st* pGenObj, SIRoad_t* pRoad,
                           SIVehicleInfo_t* pEgoInfo) {
  SI_Globals_t* pSIGlobal = GetSICalculatePointer_SIGlobals();
  const float32 fYOffsetFusedRoadBorder = fABS(pRoad->fYOffsetFused_met);
  const float32 fConfFusedRoadBorder = pRoad->fConfYOffset_per;
  const float32 fObjDistX = pGenObj->GenObjInfo.fDistX_met;
  const float32 fEgoSpeed = pEgoInfo->fegoVelocity_mps;
  const float32 fLaneWidthRoad = pSIGlobal->fLaneWidth_met;

  float32 fExtensionToBorder;
  float32 fBorder2ndLaneOcc;
  float32 fThresh2ndLaneOcc;
  float32 fBracketLeft;
  float32 fBracketRight;
  boolean bConfitionsExtensionRoadBorder = FALSE;

  if (fABS(pCriteraMaxtrix->TrackWidthTrck.fBracketPositionLeft_met) >
      TUE_C_F32_DELTA) {
    // We are in track mode,so use these brackets for further calculation
    fBracketLeft = pCriteraMaxtrix->TrackWidthTrck.fBracketPositionLeft_met;
    fBracketRight = pCriteraMaxtrix->TrackWidthTrck.fBracketPositionRight_met;
  } else {
    // We are in seek mode, so use these brackets for further calculation
    fBracketLeft = pCriteraMaxtrix->TrackWidthSeek.fBracketPositionLeft_met;
    fBracketRight = pCriteraMaxtrix->TrackWidthSeek.fBracketPositionRight_met;
  }

  if (pGenObj->GenObjInfo.bRightSensor == FALSE) {
    fExtensionToBorder =
        (fYOffsetFusedRoadBorder) - (fBracketLeft + fLaneWidthRoad);
  } else {
    fExtensionToBorder =
        (fYOffsetFusedRoadBorder) - (fABS(fBracketRight) + fLaneWidthRoad);
  }

  if ((pRoad->iNumOfAdjacentLanes_nu == 1) &&
      (fEgoSpeed > SI_BRAEXT_RB_MIN_EGOSPEED) &&
      (fYOffsetFusedRoadBorder < SI_BRAEXT_RB_MAX_YOFFSET) &&
      ((pRoad->fConfAdjacentLanes_per > SI_BRAEXT_RB_LANE_CONF) ||
       ((pRoad->fConfAdjacentLanes_per > SI_BRAEXT_RB_LANE_CONF_MIN) &&
        (fConfFusedRoadBorder > SI_BRAEXT_RB_FUSED_RB_CONF)))) {
    bConfitionsExtensionRoadBorder = TRUE;
  }

  if ((fObjDistX < 0.0f) && (bConfitionsExtensionRoadBorder == TRUE)) {
    // Check the position of the road border regarding the second lane
    // Only if less than 85% of a potential second lane is not covered by
    // the guardrail range the single lane hypothesis shall be used and the
    // extension to the road border is applied
    if (fLaneWidthRoad > TUE_C_F32_DELTA) {
      fBorder2ndLaneOcc = ((fYOffsetFusedRoadBorder) -
                           (SI_BRAEXT_RB_LANE_WIDTH_FAC * fLaneWidthRoad)) /
                          fLaneWidthRoad;
    } else {
      fBorder2ndLaneOcc = TUE_C_F32_VALUE_INVALID;
    }

    fThresh2ndLaneOcc = GDBmathLinFuncLimBounded(
        fEgoSpeed, SI_BRAEXT_OCC_TRSH_MIN_EGOSPEED,
        SI_BRAEXT_OCC_TRSH_MAX_EGOSPEED, SI_BRAEXT_OCC_TRSH_MIN_OCC,
        SI_BRAEXT_OCC_TRSH_MAX_OCC);

    if (fABS(fBorder2ndLaneOcc) > fThresh2ndLaneOcc) {
      fExtensionToBorder = TUE_CML_Max(0.0f, fExtensionToBorder);
      fBracketLeft = fBracketLeft + fExtensionToBorder;
      fBracketRight = fBracketRight - fExtensionToBorder;
    }
  }

  if (pGenObj->GenObjInfo.bRightSensor == FALSE) {
    pCriteraMaxtrix->ExtensionRoadBorder.fBracketPositionLeft_met =
        fBracketLeft;
  } else {
    pCriteraMaxtrix->ExtensionRoadBorder.fBracketPositionRight_met =
        fBracketRight;
  }
}
#else
#if (SI_LCA_ZONE_BORDER_EXTENSION)
/*****************************************************************************
  Functionname: SIExtensionRoadBorder                                  */ /*!

  @brief: Calculates the extension factor to the left side of the left lane

  @description: Calculates the extension factor to the left side of the left lane
				if a road border is present and the probability is high that only
				one lane to the left of the ego vehicle exists.

  @param[in]: uObj Array index of the object
			  pLCAParamter->LCAZone.fLCAZoneYMaxNear_met
			  pLCAParamter->LCAZone.fLCAZoneYMinNear_met
			  pLCAParamter->LCAZone.fLCAZoneYMaxFar_met

			  pCriteraMaxtrix->TrackWidthTrck.fBracketPositionLeft_met
			  pCriteraMaxtrix->TrackWidthTrck.fBracketPositionRight_met
			  pCriteraMaxtrix->TrackWidthSeek.fBracketPositionLeft_met
			  pCriteraMaxtrix->TrackWidthSeek.fBracketPositionRight_met

			  pRoad->fYOffsetFused_met
			  pRoad->iNumOfAdjacentLanes_nu
			  pRoad->fConfAdjacentLanes_per
			  pRoad->fConfYOffset_per

			  pVehParam->fVehicleWidth_met
			  pEgoInfo->fegoVelocity_mps
			  pGenObj->GenObjInfo.fDistX_met
			  pGenObj->GenObjInfo.bRightSensor

  @return:void
*****************************************************************************/
void SIExtensionRoadBorder(uint8 uObj, SICriteriaMatrix_t* pCriteraMaxtrix,
                           const SIGenObject_st* pGenObj, const SIRoad_t* pRoad,
                           const SIVehicleInfo_t* pEgoInfo,
                           const SIVehParameter_t* pVehParam,
                           const SI_LCAParameter_st* pLCAParamter) {
  float32 fBracketLeft = 0.F;
  float32 fBracketRight = 0.F;
  float32 fExtensionToBorder = 0.F;
  boolean bConditionExtensionRoadBorder = FALSE;
  float32 fYOffset2Lanes = 0.F;
  float32 fBorder2ndLaneOcc = 0.F;

  const float32 fLCAZoneYMaxNear_met =
      pLCAParamter->LCAZone.fLCAZoneYMaxNear_met;
  const float32 fLCAZoneYMinNear_met =
      pLCAParamter->LCAZone.fLCAZoneYMinNear_met;

  if (fABS(pCriteraMaxtrix->TrackWidthTrck.fBracketPositionLeft_met) >
      TUE_C_F32_DELTA) {
    // We are in track mode, so use these brackets for further calculation
    fBracketLeft = pCriteraMaxtrix->TrackWidthTrck.fBracketPositionLeft_met;
    fBracketRight = pCriteraMaxtrix->TrackWidthTrck.fBracketPositionRight_met;
  } else {
    // We are in seek mode, so use these brackets for further calculation
    fBracketLeft = pCriteraMaxtrix->TrackWidthSeek.fBracketPositionLeft_met;
    fBracketRight = pCriteraMaxtrix->TrackWidthSeek.fBracketPositionRight_met;
  }

  if (!pGenObj->GenObjInfo.bRightSensor) {
    fExtensionToBorder =
        pRoad->fYOffsetFused_met - ((pVehParam->fVehicleWidth_met * 0.5F) +
                                    pLCAParamter->LCAZone.fLCAZoneYMaxFar_met);
  } else {
    fExtensionToBorder = -pRoad->fYOffsetFusedOppBorder_met -
                         ((pVehParam->fVehicleWidth_met * 0.5F) +
                          pLCAParamter->LCAZone.fLCAZoneYMaxFar_met);
  }

  if ((pRoad->iNumOfAdjacentLanes_nu == 1) &&
      (pEgoInfo->fegoVelocity_mps > SI_BRAEXT_RB_MIN_EGOSPEED) &&        // 10
      (pRoad->fYOffsetFused_met < SI_BRAEXT_RB_MAX_YOFFSET) &&           //10
      (((pRoad->fConfAdjacentLanes_per > SI_BRAEXT_RB_LANE_CONF) &&      //0.9
        (pRoad->fConfYOffset_per > SI_BRAEXT_RB_FUSED_RB_CONF)) ||       //0.5
       ((pRoad->fConfAdjacentLanes_per > SI_BRAEXT_RB_LANE_CONF_MIN) &&  //0.7
        (pRoad->fConfYOffset_per > 0.7F)))) {
    bConditionExtensionRoadBorder = TRUE;
  }

  if ((pGenObj->GenObjInfo.fDistX_met < 0.F) &&
      (bConditionExtensionRoadBorder == TRUE)) {
    fYOffset2Lanes = (2.F * (fLCAZoneYMaxNear_met - fLCAZoneYMinNear_met)) +
                     fLCAZoneYMinNear_met +
                     (pVehParam->fVehicleWidth_met * 0.5F);
    fBorder2ndLaneOcc = fYOffset2Lanes - pRoad->fYOffsetFused_met;

    if ((fBorder2ndLaneOcc > 1.0F) && (fExtensionToBorder > TUE_C_F32_DELTA)) {
      fBracketLeft = fBracketLeft + fExtensionToBorder;
      fBracketRight = fBracketRight - fExtensionToBorder;
    }
  }

  if (pGenObj->GenObjInfo.bRightSensor == FALSE) {
    pCriteraMaxtrix->ExtensionRoadBorder.fBracketPositionLeft_met =
        fBracketLeft;
  } else {
    pCriteraMaxtrix->ExtensionRoadBorder.fBracketPositionRight_met =
        fBracketRight;
  }
}
#endif
#endif

#if (SI_DYNAMIC_LANE_BRACKET_EXTENSION)
/*****************************************************************************
  Functionname: SIExtensionOwnlaneCriticalTTC */ /*!

  @brief:Calculate the extension factor to the inner side of the lane

  @description:Calculate the extension factor to the inner side of the lane
			   if an object with a critical TTC,that will not allow for
			   braking in time,approaches us

  @param[in]:

  @return:void
*****************************************************************************/
void SIExtensionOwnlaneCriticalTTC(
    uint8 uObj, SICriteriaMatrix_t* pCriteraMaxtrix,
    SIGenObject_st* pGenObj)  //, SISRRObject_st* pSRRObj
{
  float32 fObjDistCourse = pGenObj->GenObjInfo.fDist2Course_met;
  const float32 fObjDistX = pGenObj->GenObjInfo.fDistX_met;
  const float32 fObjVX = pGenObj->GenObjInfo.fVrelX_mps;
  float32 fXEstimatedStop;
  float32 fWayToStandstill;
  float32 fBracketLeft;
  float32 fBracketRight;

  if (pGenObj->GenObjInfo.bRightSensor == TRUE) {
    // transform right sensor sign for legible comparison
    fObjDistCourse = -fObjDistCourse;
  }

  if ((fObjDistX < SI_BRAEXT_TTC_MAX_DISTX) && (fObjVX > 0.0f) &&
      (fObjDistCourse < SI_BRAEXT_TTC_MIN_DISTTOCRS)) {
    // In reality this value has a negative sign but further on it is only
    // used to calculate the estimated way fro deceleration until standstill
    // v = v0 + 2as with v the target velocity equal to zero,this leads to
    // s = v0/(-2a) with s equal to the distance the target will need to
    // reach equal velocity as the subject
    if (fABS(pCriteraMaxtrix->TrackWidthTrck.fBracketPositionLeft_met) >
        TUE_C_F32_DELTA) {
      // We are in track mode,so use these brackets for further
      // calculation
      fBracketLeft = pCriteraMaxtrix->TrackWidthTrck.fBracketPositionLeft_met;
      fBracketRight = pCriteraMaxtrix->TrackWidthTrck.fBracketPositionRight_met;
    } else {
      // We are in seek mode, so use these brackets for further
      // calculation
      fBracketLeft = pCriteraMaxtrix->TrackWidthSeek.fBracketPositionLeft_met;
      fBracketRight = pCriteraMaxtrix->TrackWidthSeek.fBracketPositionRight_met;
    }

    fWayToStandstill =
        SQR(fObjVX) * (1.0f / (2.0f * SI_DECELERATION_OWNLANE_EXT));

    fXEstimatedStop = fObjDistX + fWayToStandstill;

    if (fXEstimatedStop > -1.0f) {
      if (pGenObj->GenObjInfo.bRightSensor == FALSE) {
        pCriteraMaxtrix->ExtensionCriticalTTC.fBracketPositionRight_met =
            fBracketRight - 0.5f;
      } else {
        pCriteraMaxtrix->ExtensionCriticalTTC.fBracketPositionLeft_met =
            fBracketLeft + 0.5f;
      }
    }
  }
}
#endif

#if (SI_DYNAMIC_LANE_BRACKET_EXTENSION)
/*****************************************************************************
  Functionname: SIRestrictionDistanceDependent */ /*!

  @brief:Calculates the restriction factor which is depending on the objects distance

  @description:Calculates the restriction factor which is depending on the objects distance

  @param[in] uObj           :Array index of the object
			 pCriteraMaxtrix:where the seek and trace bracket is written

  @return:void
*****************************************************************************/
void SIRestrictionDistanceDependent(uint8 uObj, SIGenObject_st* pGenObj,
                                    SIRoad_t* pRoad,
                                    SIVehParameter_t* pVehParam,
                                    SICriteriaMatrix_t* pCriteraMaxtrix,
                                    SILCAObjInfo_t* pLCAObjInfo,
                                    SI_LCAParameter_st* pLCAParamter) {
  const float32 fObjX = pGenObj->GenObjInfo.fDistX_met;
  const float32 fSensorOffsetToSide = (0.5f * pVehParam->fVehicleWidth_met);
  const float32 fLaneWidthDefault = pLCAParamter->fDefaultLaneWidth;
  float32 fRestrictionOuter = -TUE_C_F32_VALUE_INVALID;
  float32 fRestrictionInner = -TUE_C_F32_VALUE_INVALID;
  float32 fRestriction;
  float32 fMaxRestriction;
  float32 fBracketLeft;
  float32 fBracketRight;
  const boolean bLCAWarning = pLCAObjInfo->bLCAWarning;

  if (fABS(pCriteraMaxtrix->TrackWidthTrck.fBracketPositionLeft_met) >
      TUE_C_F32_DELTA) {
    // We are in track mode,so use these brackets for further calculation
    fBracketLeft = pCriteraMaxtrix->TrackWidthTrck.fBracketPositionLeft_met;
    fBracketRight = pCriteraMaxtrix->TrackWidthTrck.fBracketPositionRight_met;
  } else {
    // We are in seek mode, so use these brackets for further calculation
    fBracketLeft = pCriteraMaxtrix->TrackWidthSeek.fBracketPositionLeft_met;
    fBracketRight = pCriteraMaxtrix->TrackWidthSeek.fBracketPositionRight_met;
  }

  // Sensor is mounted rear
  // Bracket restriction at far distance
  if (fObjX < SI_BRACKET_RESTR_FAR_DIST_X_MAX) {
    // Calculate linear restriction based on x position
    if (pRoad->iNumOfAdjacentLanes_nu > 1) {
      // fRestrictionOuter =
      // TUE_CML_BoundedLinInterpol(&aSILinearBracketRestrictionFarDist,
      // fObjX);
      fRestrictionOuter = GDBmathLinFuncLimBounded(
          fObjX, SI_BRACKET_RESTR_FAR_DIST_X_MIN,
          SI_BRACKET_RESTR_FAR_DIST_X_MAX, SI_BRACKET_RESTR_FAR_DIST_MAX,
          SI_BRACKET_RESTR_FAR_DIST_MIN);
    }

    fRestrictionInner =
        GDBmathLinFuncLimBounded(fObjX, SI_BRACKET_RESTR_FAR_DIST_X_MIN,
                                 SI_BRACKET_RESTR_FAR_DIST_X_MAX, 0.8f, 0.0f);
  } else {
    // Bracket restriction at close distances for non warning objects to a
    // maximum of 3.5m/3.8m next to the subject vehicle
    if ((fObjX > SI_BRACKET_RESTR_NEAR_DIST_X_MIN) &&
        (((fABS(pRoad->fYOffsetFused_met) > SI_BRARST_MAX_RB_Y_OFFSET) &&
          (pRoad->fConfYOffset_per > SI_BRARST_MIN_LANE_CONF)) ||
         ((pRoad->iNumOfAdjacentLanes_nu > SI_BRARST_MIN_LANES) &&
          (pRoad->fConfAdjacentLanes_per > SI_BRARST_MIN_LANE_CONF)))) {
      // Calculate maximum restriction
      fMaxRestriction =
          (SI_BRARST_LANE_WIDTH_FAC * fLaneWidthDefault) -
          (fLaneWidthDefault + 0.5f * pVehParam->fVehicleWidth_met);
      // Consider a smaller lane width than the default one in the
      // restriction value
      fMaxRestriction =
          (fMaxRestriction - (fLaneWidthDefault - pRoad->fLaneWidth_met));

      /*If an object carries an LCA warning the maximum restriction is
      reduced by 0.5m else we reduce it by 0.3m due to the differences in
      concepts between BSD and
      LCA(LCA requires more overlap between lane and object than BSD)*/
      if (bLCAWarning == TRUE) {
        fMaxRestriction = fMaxRestriction - 0.5f;
      } else {
        fMaxRestriction = fMaxRestriction - 0.3f;
      }

      // Apply the restriction if it is greater than zero and below 2m
      if ((fMaxRestriction > TUE_C_F32_DELTA) && (fMaxRestriction < 2.0f)) {
        // Calculate linear restriction based on x position
        fRestrictionOuter = GDBmathLinFuncLimBounded(
            fObjX, SI_BRACKET_RESTR_NEAR_DIST_X_MIN,
            SI_BRACKET_RESTR_NEAR_DIST_X_MAX, 0.0f, fMaxRestriction);
      }
    }
  }

  ////Check if a restriction value has been calculated for the object
  // if (fRestrictionInner > TUE_C_F32_DELTA)
  //{
  //	//Apply restriction to inner trace bracket considering sensor side
  //	pCriteraMaxtrix->RestrictionDistDependent.fBracketPositionLeft_met =
  // fBracketLeft - fRestrictionOuter;
  //}

  // if (fRestrictionOuter > TUE_C_F32_DELTA)
  //{
  //	pCriteraMaxtrix->RestrictionDistDependent.fBracketPositionRight_met =
  // fBracketLeft + fRestrictionInner;
  //}

  // Check if a inner restriction value has been calculated for the object
  if (fRestrictionInner > TUE_C_F32_DELTA) {
    // Apply restriction to inner trace bracket considering sensor side
    if (pGenObj->GenObjInfo.bRightSensor == FALSE) {
      pCriteraMaxtrix->RestrictionDistDependent.fBracketPositionRight_met =
          fBracketRight + fRestrictionInner;
    } else {
      pCriteraMaxtrix->RestrictionDistDependent.fBracketPositionLeft_met =
          fBracketLeft - fRestrictionInner;
    }
  }
  // Check if a outer restriction value has been calculated for the object
  if (fRestrictionOuter > TUE_C_F32_DELTA) {
    // Apply restriction to outer trace bracket considering sensor side
    if (pGenObj->GenObjInfo.bRightSensor == FALSE) {
      pCriteraMaxtrix->RestrictionDistDependent.fBracketPositionLeft_met =
          fBracketLeft - fRestrictionOuter;
    } else {
      pCriteraMaxtrix->RestrictionDistDependent.fBracketPositionRight_met =
          fBracketRight + fRestrictionOuter;
    }
  }
}
#else  // #if (SI_DYNAMIC_LANE_BRACKET_EXTENSION)
/*****************************************************************************
  Functionname: SIRestrictionDistanceDependent */ /*!

  @brief: Calculates the restriction factor which is depending on the objects distance

  @description: Calculates the restriction factor which is depending on the objects distance

  @param[in] uObj           :Array index of the object
			 pCriteraMaxtrix:where the seek and trace bracket is written

  @return:void
*****************************************************************************/
void SIRestrictionDistanceDependent(uint8 uObj, const SIGenObject_st* pGenObj,
                                    const SIRoad_t* pRoad,
                                    const SIVehParameter_t* pVehParam,
                                    SICriteriaMatrix_t* pCriteraMaxtrix,
                                    const SI_LCAParameter_st* pLCAParamter) {
  float32 fBracketLeft = 0.F;
  float32 fBracketRight = 0.F;
  float32 fDistX = pGenObj->GenObjInfo.fDistX_met;
  float32 fLCAZoneXMid_met = pLCAParamter->LCAZone.fLCAZoneXMid_met;
  float32 fLCAZoneXMin_met = pLCAParamter->LCAZone.fLCAZoneXMin_met;
  float32 fLCAZoneYMinDiff = pLCAParamter->LCAZone.fLCAZoneYMinFar_met -
                             pLCAParamter->LCAZone.fLCAZoneYMinNear_met;
  float32 fLCAZoneYMaxDiff = pLCAParamter->LCAZone.fLCAZoneYMaxNear_met -
                             pLCAParamter->LCAZone.fLCAZoneYMaxFar_met;
  float32 fRestrictionInner = 0.F;
  float32 fRestrictionOuter = 0.F;

  if (fABS(pCriteraMaxtrix->TrackWidthTrck.fBracketPositionLeft_met) >
      TUE_C_F32_DELTA) {
    // We are in track mode,so use these brackets for further calculation
    fBracketLeft = pCriteraMaxtrix->TrackWidthTrck.fBracketPositionLeft_met;
    fBracketRight = pCriteraMaxtrix->TrackWidthTrck.fBracketPositionRight_met;
  } else {
    // We are in seek mode, so use these brackets for further calculation
    fBracketLeft = pCriteraMaxtrix->TrackWidthSeek.fBracketPositionLeft_met;
    fBracketRight = pCriteraMaxtrix->TrackWidthSeek.fBracketPositionRight_met;
  }

  // If the x distance is less than fLCAZoneXMid_met inner and outer
  // restrictions have to be considered
  if (fDistX < fLCAZoneXMid_met) {
    if (pGenObj->GenObjInfo.bRightSensor == FALSE) {
      // In all cases the inner restriction is calculated between
      // fLCAZoneXMin_met and fLCAZoneXMid_met
      fRestrictionInner = GDBmathLinFuncLimBounded(
          fDistX, fLCAZoneXMid_met, fLCAZoneXMin_met, 0.F, fLCAZoneYMinDiff);
      // If fLCAZoneYMaxNear_met is different from fLCAZoneYMaxFar_met,
      // the outer restriction is calculated between fLCAZoneXMin_met and
      // fLCAZoneXMid_met
      fRestrictionOuter = GDBmathLinFuncLimBounded(
          fDistX, fLCAZoneXMid_met, fLCAZoneXMin_met, 0.F, fLCAZoneYMaxDiff);
    } else {
      // For Right sensor, left bracket is inner and right bracket is
      // outer
      fRestrictionInner = GDBmathLinFuncLimBounded(
          fDistX, fLCAZoneXMid_met, fLCAZoneXMin_met, 0.F, fLCAZoneYMaxDiff);
      fRestrictionOuter = GDBmathLinFuncLimBounded(
          fDistX, fLCAZoneXMid_met, fLCAZoneXMin_met, 0.F, fLCAZoneYMinDiff);
    }
  }

#if (SI_LCA_ZONE_BORDER_EXTENSION)

  if (pGenObj->GenObjInfo.bRightSensor == FALSE) {
    if (pCriteraMaxtrix->ExtensionRoadBorder.fBracketPositionLeft_met <
        TUE_C_F32_DELTA) {
      pCriteraMaxtrix->RestrictionDistDependent.fBracketPositionLeft_met =
          fBracketLeft - fRestrictionOuter;
    }
    pCriteraMaxtrix->RestrictionDistDependent.fBracketPositionRight_met =
        fBracketRight + fRestrictionInner;
  } else {
    if (pCriteraMaxtrix->ExtensionRoadBorder.fBracketPositionRight_met >
        -TUE_C_F32_DELTA) {
      pCriteraMaxtrix->RestrictionDistDependent.fBracketPositionRight_met =
          fBracketRight + fRestrictionInner;
    }
    pCriteraMaxtrix->RestrictionDistDependent.fBracketPositionLeft_met =
        fBracketLeft - fRestrictionOuter;
  }
#else
  pCriteraMaxtrix->RestrictionDistDependent.fBracketPositionLeft_met =
      fBracketLeft - fRestrictionOuter;
  pCriteraMaxtrix->RestrictionDistDependent.fBracketPositionRight_met =
      fBracketRight + fRestrictionInner;
#endif
}
#endif

#if (SI_DYNAMIC_LANE_BRACKET_EXTENSION)
/*****************************************************************************
  Functionname: SIRestrictionCurve                                  */ /*!

  @brief:Calculates the restriction factor which is depending on the curve radius

  @description:Calculates the restriction factor which is depending on the curve radius

  @param[in]:

  @return:void
*****************************************************************************/
void SIRestrictionCurve(uint8 uObj, SICriteriaMatrix_t* pCriteraMaxtrix,
                        SIGenObject_st* pGenObj) {
  float32 fFilterCurveRaius =
      GetSICalculatePointer_SIGlobals()->fCurveRadiusMinFiltered_met;
  float32 fRestriction = 0.0f;
  float32 fBracketLeft = 0.0f;
  float32 fBracketRight;
  boolean bLowCurvedRad = FALSE;

  if (fFilterCurveRaius < SI_MAX_CURVE_RADIUS_BRACKET_REST) {
    bLowCurvedRad = TRUE;
  }

  // Check if the curve radius is below the maximum curve radius for enabling
  // the bracket restriction
  if (bLowCurvedRad == TRUE) {
    if (fABS(pCriteraMaxtrix->TrackWidthTrck.fBracketPositionLeft_met) >
        TUE_C_F32_DELTA) {
      // We are in track mode,so use these brackets for further
      // calculation
      fBracketLeft = pCriteraMaxtrix->TrackWidthTrck.fBracketPositionLeft_met;
      fBracketRight = pCriteraMaxtrix->TrackWidthTrck.fBracketPositionRight_met;
    } else {
      // We are in seek mode, so use these brackets for further
      // calculation
      fBracketLeft = pCriteraMaxtrix->TrackWidthSeek.fBracketPositionLeft_met;
      fBracketRight = pCriteraMaxtrix->TrackWidthSeek.fBracketPositionRight_met;
    }

    // Calculate the liner bracket restriction based on the current curve
    // radius
    fRestriction = GDBmathLinFuncLimBounded(
        fFilterCurveRaius, SI_MIN_CR_BRARST_REAR,
        SI_MAX_CURVE_RADIUS_BRACKET_REST, SI_BRARST_CR_MIN_REST_REAR,
        SI_BRARST_CR_MAX_REST);

    // Check if the restriction is greater than zero
    if (fRestriction > TUE_C_F32_DELTA) {
      // Apply the restriction to both trace brackets
      if (pGenObj->GenObjInfo.bRightSensor == FALSE) {
        pCriteraMaxtrix->RestrictionCurve.fBracketPositionRight_met =
            fBracketRight + fRestriction;
      } else {
        pCriteraMaxtrix->RestrictionCurve.fBracketPositionLeft_met =
            fBracketLeft - fRestriction;
      }
    }
  }
}
#endif
/*****************************************************************************
  Functionname: SISetTraceBracketOutput                                  */ /*!

  @brief:

  @description:

  @param[in]: pLCAParamter->LCAZone.fLCAZoneYMinNear_met
			  pLCAParamter->LCAZone.fLCAZoneYMaxNear_met

			  pSIObj->ObjCor.fTraceBracketOffsetLeft_met
			  pSIObj->ObjCor.fTraceBracketOffsetRight_met

  @return:void
*****************************************************************************/
void SISetTraceBracketOutput(uint8 uObj, const SIGenObject_st* pGenObj,
                             SI_Info_st* pSIObj, const SIParam_st* params) {
#if (SI_DYNAMIC_LANE_BRACKET_EXTENSION)
  float32 fLaneWidth = GetSICalculatePointer_SIGlobals()->fLaneWidth_met;
#else
  const SI_LCAParameter_st* pLCAParamter = &params->LCAParamter;
  const SIVehParameter_t* pVehPar = &params->VehParameter;
  const float32 fLCAZoneYMinNear_met =
      pLCAParamter->LCAZone.fLCAZoneYMinNear_met;
  const float32 fLCAZoneYMaxNear_met =
      pLCAParamter->LCAZone.fLCAZoneYMaxNear_met;
  const float32 fLCAZoneYOffset =
      (pVehPar->fVehicleWidth_met * 0.5F) + fLCAZoneYMinNear_met +
      ((fLCAZoneYMaxNear_met - fLCAZoneYMinNear_met) * 0.5F);
#endif

#if (SI_DYNAMIC_LANE_BRACKET_EXTENSION)
  if (pGenObj->GenObjInfo.bRightSensor == FALSE) {
    pSIObj->ObjTraceBracket.fTraceBracketLeft_met =
        fLaneWidth + pSIObj->ObjCor.fTraceBracketOffsetLeft_met;
    pSIObj->ObjTraceBracket.fTraceBracketRight_met =
        fLaneWidth + pSIObj->ObjCor.fTraceBracketOffsetRight_met;
  } else {
    pSIObj->ObjTraceBracket.fTraceBracketLeft_met =
        -(fLaneWidth - pSIObj->ObjCor.fTraceBracketOffsetLeft_met);
    pSIObj->ObjTraceBracket.fTraceBracketRight_met =
        -(fLaneWidth - pSIObj->ObjCor.fTraceBracketOffsetRight_met);
  }
#else
  if (pGenObj->GenObjInfo.bRightSensor == FALSE) {
    pSIObj->ObjTraceBracket.fTraceBracketLeft_met =
        fLCAZoneYOffset + pSIObj->ObjCor.fTraceBracketOffsetLeft_met;
    pSIObj->ObjTraceBracket.fTraceBracketRight_met =
        fLCAZoneYOffset + pSIObj->ObjCor.fTraceBracketOffsetRight_met;
  } else {
    pSIObj->ObjTraceBracket.fTraceBracketLeft_met =
        -(fLCAZoneYOffset - pSIObj->ObjCor.fTraceBracketOffsetLeft_met);
    pSIObj->ObjTraceBracket.fTraceBracketRight_met =
        -(fLCAZoneYOffset - pSIObj->ObjCor.fTraceBracketOffsetRight_met);
  }
#endif
}

/*****************************************************************************
  Functionname: SICalculateInlaneOverlap                                  */ /*!

  @brief: Calculate overlap and occupancy between object and trace brackets

  @description: Calculate overlap and occupancy between object and trace brackets

  @param[in]: uObj                                                  Array index of object for which occupancies are checked
  @param[in]: pSIObj->ObjTraceBracket.fTraceBracketLeft_met         Left trace bracket position
  @param[in]: pSIObj->ObjTraceBracket.fTraceBracketRight_met        Right trace bracket position
  @param[in]: fMinObjWidth                                          Minimum object's width
  @param[in]: fMaxObjWidth                                          Maximum object's width
  @param[in]: pSIObj->ObjTrajRefPoint.fDistToTraj_met               Object's lateral distance to trajectory
  @param[in]: pGenObj->GenObjInfo.fWidthLeft_met                      Object's width left of the track position
  @param[in]: pGenObj->GenObjInfo.fWidthRight_met                     Object's width Right of the track position

  @param[out]: pOccupancy      Pointer to occupancy information of the object: overlap, ObjectOccupancy and TrajectoryOccupancy

  @return: void
*****************************************************************************/
void SICalculateInlaneOverlap(uint8 uObj, const SIGenObject_st* pGenObj,
                              SI_Info_st* pSIObj, const float32 fMinObjWidth,
                              const float32 fMaxObjWidth,
                              SITrajOccupancy_t* pOccupancy) {
  float32 fObjectWidthLeft = 0.F;
  float32 fObjectWidthRight = 0.F;
  float32 fObjectWidth = 0.F;
  float32 fTraceBracketLeft = pSIObj->ObjTraceBracket.fTraceBracketLeft_met;
  float32 fTraceBracketRight = pSIObj->ObjTraceBracket.fTraceBracketRight_met;
  float32 fBracketWidth = 0.F;
  float32 fObjRefDist = pSIObj->ObjTrajRefPoint.fDistToTraj_met;
  float32 fDistToTrajMax = 0.F;
  float32 fDistToTrajMin = 0.F;
  float32 fOverlap = 0.F;
  float32 fObjOccupancy = 0.F;
  float32 fTrajOccupancy = 0.F;

  // Limit the object width to be between 0.8m and 2.55m
  fObjectWidthLeft =  //0.84
      MAX(pGenObj->GenObjInfo.fWidthLeft_met, (fMinObjWidth * 0.5F));
  fObjectWidthLeft = MIN(fObjectWidthLeft, (fMaxObjWidth * 0.5F));  // 0.84
  fObjectWidthRight =
      MAX(pGenObj->GenObjInfo.fWidthRight_met, (fMinObjWidth * 0.5F));
  fObjectWidthRight = MIN(fObjectWidthRight, (fMaxObjWidth * 0.5F));  //0.84
  fObjectWidth =
      MAX((fObjectWidthLeft + fObjectWidthRight), TUE_C_F32_DELTA);  //1.68

  // Assure that the bracket width is larger than 0 for divisions
  fBracketWidth =
      MAX((fTraceBracketLeft - fTraceBracketRight), TUE_C_F32_DELTA);  //3.4
  //(-1.6 - (-5) = 3.4)
  // Calculate distance between object sides and trajectory
  fDistToTrajMax = fObjRefDist + fObjectWidthLeft;
  fDistToTrajMin = fObjRefDist - fObjectWidthRight;

  // Calculate fOverlap is the overlap width between object and trace bracket
  //                              fOverlap fOverlap
  // Calculate fObjOccupancy = -------------------      and fTrajOccupancy =
  // ------------------
  //                             fObjectWidth fBracketWidth

  if ((fObjectWidth > fBracketWidth) && (fDistToTrajMin < fTraceBracketRight) &&
      (fDistToTrajMax > fTraceBracketLeft)) {
    // Cover unusual case that the object width is larger than the bracket
    // width
    fOverlap = fBracketWidth;
    fObjOccupancy = 1.F;
    fTrajOccupancy = 1.F;
  } else {
    if (((fTraceBracketLeft - fDistToTrajMax) > fBracketWidth) ||
        ((fDistToTrajMin - fTraceBracketRight) > fBracketWidth)) {
      // No Overlap
      fOverlap = 0.F;
      fObjOccupancy = 0.F;
      fTrajOccupancy = 0.F;
    } else {
      if ((fDistToTrajMax < fTraceBracketLeft) &&
          (fDistToTrajMin > fTraceBracketRight)) {
        // Full overlap
        fOverlap = MIN((fTraceBracketLeft - fDistToTrajMin),
                       (fDistToTrajMax - fTraceBracketRight));
        fObjOccupancy = 1.F;
        fTrajOccupancy = fOverlap / fBracketWidth;
      } else {
        if (fDistToTrajMax > fTraceBracketLeft) {
          // Partial overlap with left trace bracket
          fOverlap = fTraceBracketLeft - fDistToTrajMin;
          fObjOccupancy = fOverlap / fObjectWidth;
          fTrajOccupancy = fOverlap / fBracketWidth;
        } else {
          if (fDistToTrajMin < fTraceBracketRight) {
            // Partial overlap with right trace bracket
            fOverlap = fDistToTrajMax - fTraceBracketRight;
            fObjOccupancy = fOverlap / fObjectWidth;
            fTrajOccupancy = fOverlap / fBracketWidth;
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

  pSIObj->fObjBracketOverlap_met = pOccupancy->fOverlap_met;
}

/*****************************************************************************
  Functionname: SILaneAsscoiationChange                                  */ /*!

  @brief: Check lane association change state flow for inlane to outlane or out to inlane

  @description: Check lane association change state flow for inlane to outlane or out to inlane

  @param[in]:  pGenObj->GenObjInfo.uiMaintenanceState_nu          Maintenance state if object(measured,predicted)
			   pObjLCAState->SIActLaneState
			   pSIObj->uOutlanePredictionTimer_nu              Predict the number of time outlane
  @param[out]: pObjLCAState->uInlaneCycleCounter_nu            Object in lane cycle counter
			   pObjLCAState->fCorridorRelevantTime_s           Time the object has been inside corridor
			   pObjLCAState->SIInlaneState                     Lane assignment
			   pObjLCAState->uIn2OutlaneTransition_nu          Object change from in lane to out lane timer
			   pObjLCAState->SIInlaneState                     Lane assignment
			   pObjLCAState->SIActLaneState

  @return:void
*****************************************************************************/
void SILaneAsscoiationChange(uint8 uObj, const SIGenObject_st* pGenObj,
                             SI_Info_st* pSIObj,
                             const SILBSObjInfo_st* pLBSObjInfo,
                             SITrajOccupancy_t* pOccupancy,
                             SLACInReq_t reqPorts1, const SIParam_st* params) {
  boolean bStateFlow = 0;

  if (pGenObj->GenObjInfo.uiMaintenanceState_nu !=
      EM_GEN_OBJ_MT_STATE_DELETED) {
    if (pSIObj->ObjLaneLCAStatus.uIn2OutlaneTransition_nu > 0U) {
      pSIObj->ObjLaneLCAStatus.uIn2OutlaneTransition_nu =
          pSIObj->ObjLaneLCAStatus.uIn2OutlaneTransition_nu - 1U;
    } else {
      pSIObj->ObjLaneLCAStatus.uIn2OutlaneTransition_nu = 0U;
    }

    switch (pSIObj->ObjLaneLCAStatus.SIInlaneState) {
      case OBJ_STATE_OUTLANE:
        bStateFlow = SICheckStateFlowOutlaneToInlane(
            uObj, pGenObj, pSIObj, pLBSObjInfo, pOccupancy, reqPorts1, params);

        if ((bStateFlow == TRUE) &&
            (pSIObj->uOutlanePredictionTimer_nu == 0U)) {
          pSIObj->ObjLaneLCAStatus.SIInlaneState = OBJ_STATE_INLANE;
          pSIObj->ObjLaneLCAStatus.SIActLaneState = OBJ_STATE_INLANE;
        } else if (pSIObj->uOutlanePredictionTimer_nu > 0U) {
          pSIObj->uOutlanePredictionTimer_nu =
              pSIObj->uOutlanePredictionTimer_nu - 1U;
        }
        break;
      case OBJ_STATE_INLANE:
        bStateFlow = SICheckStateFlowInlaneToOutlane(
            uObj, pGenObj, pSIObj, pLBSObjInfo, pOccupancy, reqPorts1, params);

        if ((bStateFlow == TRUE) &&
            (pSIObj->uOutlanePredictionTimer_nu == 0U)) {
          pSIObj->ObjLaneLCAStatus.SIInlaneState = OBJ_STATE_OUTLANE;
          pSIObj->ObjLaneLCAStatus.SIActLaneState = OBJ_STATE_OUTLANE;

          // Object actually changes from inlane to outlane -> set
          // timer
          pSIObj->ObjLaneLCAStatus.uIn2OutlaneTransition_nu =
              SI_IN2OUTLANE_MAX_TRANSITIONTIME;
        } else if (pSIObj->uOutlanePredictionTimer_nu > 0U) {
          pSIObj->uOutlanePredictionTimer_nu =
              pSIObj->uOutlanePredictionTimer_nu - 1U;
        }
        break;
      default:
        break;
    }
  } else {
    // init the state flow counter and Transition if object measurement
    // deleted
    pSIObj->ObjLaneLCAStatus.uInlaneCycleCounter_nu = 0U;
    pSIObj->ObjLaneLCAStatus.fCorridorRelevantTime_s = 0.0F;
    pSIObj->ObjLaneLCAStatus.SIInlaneState = OBJ_STATE_OUTLANE;
    pSIObj->ObjLaneLCAStatus.uIn2OutlaneTransition_nu = 0U;
  }
}

/*****************************************************************************
  Functionname: SICheckStateFlowOutlaneToInlane */ /*!

  @brief: check the conditions for state change from out of lane to in lane.

  @description: check object flow from out lane to in ego lane

  @param[in]:  uObj             array index of object for which the lane flow is checked
			   pGenObj, pSRRObj, pLBSObjInfo, pSIObj, pEgoInfo, pRoadInfo, pOccupancy, params, fCycletime
  @param[out]: pSIObj->ObjLaneLCAStatus.SIActLaneState
			   pSIObj->SIBool.bInLTimeValue                Inlane time criteria check flag

  @return:     bRet            TRUE if object determined to be in ego lane
*****************************************************************************/
boolean SICheckStateFlowOutlaneToInlane(
    uint8 uObj, const SIGenObject_st* pGenObj, SI_Info_st* pSIObj,
    const SILBSObjInfo_st* pLBSObjInfo, SITrajOccupancy_t* pOccupancy,
    SLACInReq_t reqPorts1, const SIParam_st* params) {
  boolean bRet = FALSE;
  boolean bOccValue = 0;
  boolean bCustomValue = 0;
  boolean bInlaneTimeValue = 0;
  boolean bInlaneDistValue = 0;
  float32 fCycletime = reqPorts1.fCycletime_s;
  SIVehicleInfo_t* pEgoVehInfo = reqPorts1.pEgoVehInfo;
  /*Check the occupancy values for InLane decision of object*/
  bOccValue = SICheckInlaneOccupancies(pGenObj, pSIObj, pOccupancy);
  /*Check predicted criterias for object InLane*/
  bCustomValue =
      SI_CheckCustomInlaneCriteria(uObj, pGenObj, pSIObj, reqPorts1, params);

  if ((bOccValue == TRUE) || (bCustomValue)) {
    pSIObj->ObjLaneLCAStatus.SIActLaneState = OBJ_STATE_INLANE;
    /*Check time condition for in lane decision of objects*/
    bInlaneTimeValue = SICheckInlaneTimer(pGenObj, pLBSObjInfo, pSIObj,
                                          pEgoVehInfo, fCycletime);
    /*Check distance condition for in lane decision of objects*/
    bInlaneDistValue =
        SICheckInlaneDistance(pGenObj, pSIObj, pEgoVehInfo, fCycletime);
  } else {
    pSIObj->ObjLaneLCAStatus.SIActLaneState = OBJ_STATE_OUTLANE;
    bInlaneTimeValue = FALSE;
    bInlaneDistValue = FALSE;
    /*Reset the In lane Timer if timing conditions not fullfilled*/
    SIResetInlaneTimer(pGenObj, pSIObj, fCycletime);
    /*Reset the In lane distance*/
    SIResetInlaneDistance(pSIObj);
  }
  pSIObj->SIBool.bInLTimeValue = bInlaneTimeValue;

  /*State flow: object is inlane of occupancies satisfied together with at
  least one of inlane timer ,inlane distance or camera inlane, or the custom
  inlane criteria met (usually predicted pickup) */
  if (((bOccValue == TRUE) &&
       ((bInlaneTimeValue == TRUE) || (bInlaneDistValue == TRUE))) ||
      (bCustomValue == TRUE)) {
    bRet = TRUE;
  } else {
    bRet = FALSE;
  }
  return bRet;
}

/*****************************************************************************
  Functionname: SICheckInlaneOccupancies                                  */ /*!

  @brief: Check the occupancy values for inlane decision of object

  @description: This check is called for objects which were assumed to be out lane
				in previous evaluation(s),thus uses higher occupancy values to avoid
				false pick-ups

  @param[in]: pGenObj->GenObjInfo.fWidthLeft_met             Object's width left of the track position
			  pGenObj->GenObjInfo.fWidthRight_met            Object's width right of the track position
			  pGenObj->GenObjInfo.uiLifeCycles_nu             Object's life cycle
			  pOccupancy->fObjectOccupancy_per             the object with trace bracket overlap as a percentage of the object width
			  pOccupancy->fObjectOccupancyVar_per          the object with trace bracket overlap as a percentage variance of the object width
			  pOccupancy->fTrajectoryOccupancy_per         the object with trace bracket overlap as a percentage of the trace bracket width
			  pOccupancy->fTrajectoryOccupancyVar_per      the object with trace bracket overlap as a percentage variance of the trace bracket width
			  pOccupancy->fOverlap_met                     the object with trace bracket overlap
			  pOccupancy->fOverlapVar_met                  the object with trace bracket overlap variance
  @param[out]: pSIObj->SIBool.bInLObjOccValue               Inlane overlap as a percentage of object occupancy check flag
			  pSIObj->SIBool.bInLLaneOccValue              Inlane overlap as a percentage of trace bracket occupancy check flag
			  pSIObj->SIBool.bInLLaneOverlapValue          Inlane overlap check flag
			  pSIObj->SIBool.bInLOccValue                  Inlane check flag

  @return:    bRet                                         TRUE if inlane occupancy is sufficient
*****************************************************************************/
boolean SICheckInlaneOccupancies(const SIGenObject_st* pGenObj,
                                 SI_Info_st* pSIObj,
                                 SITrajOccupancy_t* pOccupancy) {
  const float32 fObjWidth =
      pGenObj->GenObjInfo.fWidthLeft_met + pGenObj->GenObjInfo.fWidthRight_met;
  boolean bObjOccValue = FALSE;
  boolean bLaneOccValue = FALSE;
  boolean bObjOverlapValue = FALSE;
  const float32 fObjOccupancy = pOccupancy->fObjectOccupancy_per -
                                SQRT(pOccupancy->fObjectOccupancyVar_per);
  const float32 fObjOccThresh =
      SIGetObjOccPickupThreshold(pGenObj, pSIObj, fObjWidth);
  const float32 fLaneOccpancy = pOccupancy->fTrajectoryOccupancy_per -
                                SQRT(pOccupancy->fTrajectoryOccupancyVar_per);
  const float32 fLaneOccThresh = SIGetLaneOccPickupThreshold(pGenObj, pSIObj);
  const float32 fOverlap =
      pOccupancy->fOverlap_met - SQRT(pOccupancy->fOverlapVar_met);
  const float32 fOverlapThresh = SIGetOverlapPickupThreshold(pGenObj, pSIObj);
  boolean bRet = FALSE;
  // Check the object occupancy value for inlane decision of object
  if (fObjOccupancy > fObjOccThresh) {
    bObjOccValue = TRUE;
  }
  // Check the lane occupancy value for inlane decision of object
  if (fLaneOccpancy > fLaneOccThresh) {
    bLaneOccValue = TRUE;
  }
  // Check the object trajectory overlap value for inlane decision of object
  if (fOverlap > fOverlapThresh) {
    bObjOverlapValue = TRUE;
  }

  if ((pSIObj->SIBool.bInLObjOccValue == FALSE) &&
      (pSIObj->SIBool.bInLLaneOccValue == FALSE) &&
      (pGenObj->GenObjInfo.uiLifeCycles_nu < 3U) &&
      (fObjWidth < SI_MIN_OBJECT_WIDTH)) {
    bObjOccValue = FALSE;
    bLaneOccValue = FALSE;
  }

  pSIObj->SIBool.bInLObjOccValue = bObjOccValue;
  pSIObj->SIBool.bInLLaneOccValue = bLaneOccValue;
  pSIObj->SIBool.bInLLaneOverlapValue = bObjOverlapValue;

  if ((bObjOccValue == TRUE) || (bLaneOccValue == TRUE) ||
      (bObjOverlapValue == TRUE)) {
    bRet = TRUE;
  } else {
    bRet = FALSE;
  }

  pSIObj->SIBool.bInLOccValue = bRet;
  return bRet;
}

/*****************************************************************************
  Functionname: SIGetObjOccPickupThreshold                                  */ /*!

  @brief: Get threshold for the object occupancy value for inlane decision of object

  @description: Return the threshold for object occupancy value for inlane decision of the specified object

  @param[in]: pGenObj->GenObjInfo.eDynamicProperty_nu                 Object dynamic property
			  pSIObj->ObjLaneLCAStatus.SIActLaneState                 Lane assignment without timer and distance criteria
			  SICalculate.SIGlobals.fCurveRadiusMinFiltered_met       The minimum filter curve radius
			  fObjWidth                                               Object's width

  @return:    PickupThresholdObj                                      Occupancy threshold for inlane association of object
*****************************************************************************/
float32 SIGetObjOccPickupThreshold(const SIGenObject_st* pGenObj,
                                   SI_Info_st* pSIObj, float32 fObjWidth) {
  float32 PickupThresholdObj = 0.F;
  // Select threshold due to dynamic property
  if ((pGenObj->GenObjInfo.eDynamicProperty_nu ==
       EM_GEN_OBJECT_DYN_PROPERTY_STATIONARY) ||
      (pGenObj->GenObjInfo.eDynamicProperty_nu ==
       EM_GEN_OBJECT_DYN_PROPERTY_ONCOMING)) {
    if (pSIObj->ObjLaneLCAStatus.SIActLaneState == OBJ_STATE_INLANE) {
      PickupThresholdObj = ObjectOccupancyDropThreshStat;  // 0.45
    } else {
      PickupThresholdObj = ObjectOccupancyPickupThreshStat;  // 0.6
    }
  } else {
    // In narrow curves the occupancy pickup threshold is changed to larger
    // values
    PickupThresholdObj = GDBmathLinFuncLimBounded(
        SICalculate.SIGlobals.fCurveRadiusMinFiltered_met,
        SI_OCC_PICKUP_TRSH_CR_MIN, SI_OCC_PICKUP_TRSH_CR_MAX,
        SI_OCC_PICKUP_TRSH_MIN,        // 0.85
        ObjectOccupancyPickupThresh);  // 0.5
    // For very small objects the occupancy threshold is increased to large
    // values to prevent false pickups of unreasonable objects
    if (fObjWidth < SI_MIN_OBJECT_WIDTH) {
      PickupThresholdObj =
          PickupThresholdObj + SI_OCC_PICKUP_ADD_TRSH;  //+ 0.15
    }
  }
  return PickupThresholdObj;
}

/*****************************************************************************
  Functionname: SIGetLaneOccPickupThreshold                                  */ /*!

  @brief: Get threshold for the lane occupancy value for inlane decision of object

  @description: Get the threshold for lane occupancy value for inlane decision of the specified object

  @param[in]: pGenObj->GenObjInfo.eDynamicProperty_nu        Object dynamic property
			  pSIObj->ObjLaneLCAStatus.SIActLaneState        Lane assignment without timer and distance criteria

  @return:    PickupThresholdLane                            Lane occupancy pickup threshold for given object
*****************************************************************************/
float32 SIGetLaneOccPickupThreshold(const SIGenObject_st* pGenObj,
                                    SI_Info_st* pSIObj) {
  float32 PickupThresholdLane = 0.F;
  // Select thresholds due to dynamic property
  if ((pGenObj->GenObjInfo.eDynamicProperty_nu ==
       EM_GEN_OBJECT_DYN_PROPERTY_STATIONARY) ||
      (pGenObj->GenObjInfo.eDynamicProperty_nu ==
       EM_GEN_OBJECT_DYN_PROPERTY_ONCOMING)) {
    if (pSIObj->ObjLaneLCAStatus.SIActLaneState == OBJ_STATE_INLANE) {
      PickupThresholdLane = LaneOccupancyDropThreshStat;  // 0.2
    } else {
      PickupThresholdLane = LaneOccupancyPickupThreshStat;  // 0.3
    }
  } else {
    PickupThresholdLane = LaneOccupancyPickupThresh;
  }
  return PickupThresholdLane;
}

/*****************************************************************************
  Functionname: SIGetOverlapPickupThreshold                                  */ /*!

  @brief: Get threshold for the overlap value between object and lane for inlane decision of object

  @description: Get threshold for the overlap value between object and lane for inlane decision of object

  @param[in]: pGenObj->GenObjInfo.eDynamicProperty_nu        Object dynamic property
			  pSIObj->ObjLaneLCAStatus.SIActLaneState        Lane assignment without timer and distance criteria

  @return:    PickupThresholdOverlap                            Lane occupancy pickup threshold for given object
*****************************************************************************/
float32 SIGetOverlapPickupThreshold(const SIGenObject_st* pGenObj,
                                    SI_Info_st* pSIObj) {
  float32 PickupThresholdOverlap = 0.F;

  // Select thresholds due to dynamic property
  if ((pGenObj->GenObjInfo.eDynamicProperty_nu ==
       EM_GEN_OBJECT_DYN_PROPERTY_STATIONARY) ||
      (pGenObj->GenObjInfo.eDynamicProperty_nu ==
       EM_GEN_OBJECT_DYN_PROPERTY_ONCOMING)) {
    if (pSIObj->ObjLaneLCAStatus.SIActLaneState == OBJ_STATE_INLANE) {
      PickupThresholdOverlap = LaneOverlapDropThreshStat;  // 0.4
    } else {
      PickupThresholdOverlap = LaneOverlapPickupThreshStat;  // 1
    }
  } else {
    PickupThresholdOverlap = ObjectLaneOverlapPickupThresh;  // 1
  }
  return PickupThresholdOverlap;
}

/*****************************************************************************
  Functionname: SI_CheckCustomInlaneCriteria                                  */ /*!

  @brief: Check customer specific Inlane criteria

  @description: Check customer specific Inlane criteria

  @param[in]: pGenObj->GenObjInfo.eDynamicProperty_nu         Object dynamic property
  @param[out]: pSIObj->SIBool.bInLCustomValue                 Custom Inlane check flag
  @return:    bret                                            TRUE  if object is inlane based on custom criteria
															  FALSE if object is outlane based on custom criteria
*****************************************************************************/
boolean SI_CheckCustomInlaneCriteria(uint8 uObj, const SIGenObject_st* pGenObj,
                                     SI_Info_st* pSIObj, SLACInReq_t reqPorts1,
                                     const SIParam_st* params) {
  boolean bRet = FALSE;

  // Predicted lane association not for stationary objects
  if (pGenObj->GenObjInfo.eDynamicProperty_nu !=
      EM_GEN_OBJECT_DYN_PROPERTY_STATIONARY) {
    bRet = SI_CheckPredictedInlaneCriteria(uObj, pGenObj, pSIObj, reqPorts1,
                                           params);
  } else {
    bRet = FALSE;
  }
  pSIObj->SIBool.bInLCustomValue = bRet;
  return bRet;
}

/*****************************************************************************
  Functionname: SI_CheckPredictedInlaneCriteria */ /*!

  @brief: Check predicted criterias for object inlane

  @description: Check predicted criterias for object inlane

  @param[in]: pGenObj->GenObjInfo.fDistX_met        Object's longitudinal relative distance
			  pGenObj->GenObjInfo.fDistY_met        Object's lateral relative distance
			  pGenObj->GenObjInfo.fVrelX_mps        Object's longitudinal relative velocity
			  pGenObj->GenObjInfo.fWidthLeft_met      Object's width left of the track position
			  pGenObj->GenObjInfo.fWidthRight_met     Object's width Right of the track position
			  pGenObj->GenObjInfo.uiLifeCycles_nu      Object lifetime in cycles

			  params->pSensorMounting->SensorLeft.fLatPos_met      the radar sensor mounting Y position

			  pEgoInfo->fegoVelocity_mps            Ego velocity

			  SI_Globals_t fCurveRadiusMinFiltered_met
			  pRoadInfo->fCurveRadius_met           The curvature radius of the current driver road

			  pSIObj->ObjTraceBracket.fTraceBracketLeft_met    The trace bracket left side coordinate
			  pSIObj->ObjTraceBracket.fTraceBracketRight_met   The trace bracket Right side coordinate
			  pSIObj->uInlanePredictionTimer_nu                The timer of inlane prediction
			  pSIObj->ObjTrajRefPoint.fDistToTraj_met          Distance to trajectory
			  pSIObj->fVrelToTraj_mps

			  eAssociatedLane                       Association lane enumeration
			  pSIPredictedDist                      The predicted position

  @return:    bRet                                  TRUE if object is predicted to be inlane
*****************************************************************************/
boolean SI_CheckPredictedInlaneCriteria(uint8 uObj,
                                        const SIGenObject_st* pGenObj,
                                        SI_Info_st* pSIObj,
                                        SLACInReq_t reqPorts1,
                                        const SIParam_st* params) {
  float32 prediction_time = 0.F;
  float32 fPredDistLeftEdge = 0.F;
  float32 fPredDistRightEdge = 0.F;
  float32 fThreshLeftBracket = 0.F;
  float32 fThreshRightBracket = 0.F;
  const float32 fYObjMax = pGenObj->GenObjInfo.fDistY_met +
                           params->SensorMounting.SensorLeft.fLatPos_met +
                           pGenObj->GenObjInfo.fWidthLeft_met;
  const float32 fYObjMin = pGenObj->GenObjInfo.fDistY_met +
                           params->SensorMounting.SensorLeft.fLatPos_met -
                           pGenObj->GenObjInfo.fWidthRight_met;
  boolean bRet = FALSE;
  eAssociatedLane_t eAssociatedLane =
      GetSICalculatePointer()->eAssociatedLaneList[uObj];
  SIPredictedDistance_t* pSIPredictedDist =
      GetSICalculatePointer_SIPredictedDist();
  float32 fCurveRadiusMinFiltered_met =
      GetSICalculatePointer_SIGlobals()->fCurveRadiusMinFiltered_met;

  // SISRRObject_st* pSRRObj = &reqPorts1.SRRObjList.aObject[uObj];
  SIVehicleInfo_t* pEgoInfo = reqPorts1.pEgoVehInfo;
  SIRoad_t* pRoadInfo = reqPorts1.pRoad;
  // Get longitudinal 1/ttc as predition time
  prediction_time = SIGetPredictionTime(pGenObj, SI_MIN_CUT_IN_PRED_TIME,
                                        SI_MAX_CUT_IN_PRED_TIME);  // pSRRObj,

  if ((prediction_time > TUE_C_F32_DELTA) &&
      (pGenObj->GenObjInfo.uiLifeCycles_nu >
       SI_PRED_LANE_ASSOC_MIN_OBJ_LIFETIME) &&  // 2
      (pGenObj->GenObjInfo.fDistX_met < SI_PRED_LANE_ASSOC_X_OBJ_MAX) &&
      (pGenObj->GenObjInfo.fDistX_met > SI_PRED_LANE_ASSOC_X_OBJ_MIN) &&
      (pGenObj->GenObjInfo.fVrelX_mps < SI_PRED_LANE_ASSOC_VX_OBJ_MAX) &&
      (pEgoInfo->fegoVelocity_mps > SI_PRED_LANE_ASSOC_VEGO_MIN) &&
      (((-(fCurveRadiusMinFiltered_met)) >
        SI_PRED_LANE_ASSOC_MIN_Y_OPP_BORDER) ||
       (eAssociatedLane == ASSOC_LANE_EGO)) &&
      (fABS(pRoadInfo->fCurveRadius_met) > SI_PRED_LANE_ASSOC_CURVE_RAD_MIN)) {
    // Calculate displacement now
    // The predicted displacement is the actual position plus the product of
    // gradient and prediction time
    pSIPredictedDist->pdist = (pSIObj->ObjTrajRefPoint.fDistToTraj_met) +
                              (prediction_time * pSIObj->fVrelToTraj_mps);
    pSIPredictedDist->pdist_var =
        0.F;  // DistanceToTrajVar + (SQR(prediction_time) * VRelToTrajVar *
              // fVRelToTrajVarFac)
    pSIPredictedDist->pdist_var_fullpred =
        0.F;  // DistanceToTrajVar + (SQR(SiLaneAssPredTime) * VRelToTrajVar
              // * fVRelToTrajVarFac)

    // Calculate sides of predicted object and trace bracket
    fPredDistLeftEdge =
        pSIPredictedDist->pdist + pGenObj->GenObjInfo.fWidthLeft_met;
    fPredDistRightEdge =
        pSIPredictedDist->pdist - pGenObj->GenObjInfo.fWidthRight_met;
    fThreshLeftBracket = pSIObj->ObjTraceBracket.fTraceBracketLeft_met - 1.0F;
    fThreshRightBracket = pSIObj->ObjTraceBracket.fTraceBracketRight_met + 1.0F;

    // check :
    // -- if the predicted right edge of the object is to the right of the
    // left bracket minus an additional offset
    // -- if the predicted left edge of the object is to the left of the
    // right bracket plus an additional offset
    if ((fPredDistRightEdge < fThreshLeftBracket) &&
        (fPredDistLeftEdge > fThreshRightBracket)) {
      if (pSIObj->uInlanePredictionTimer_nu < SI_PRED_LANE_ASSOC_TIME_MAX) {
        pSIObj->uInlanePredictionTimer_nu =
            pSIObj->uInlanePredictionTimer_nu + 1U;
      }
    } else {
      if (pSIObj->uInlanePredictionTimer_nu > 0U) {
        pSIObj->uInlanePredictionTimer_nu =
            pSIObj->uInlanePredictionTimer_nu - 1U;
      }
    }

    // Check if the inlane prediction timer is above the require threshold
    // and the object is at least partially inside the trace brackets
    if ((pSIObj->uInlanePredictionTimer_nu >
         SI_PRED_LANE_ASSOC_TIME_IN_ENABLE) &&
        (fYObjMax > pSIObj->ObjTraceBracket.fTraceBracketRight_met) &&
        (fYObjMin < pSIObj->ObjTraceBracket.fTraceBracketLeft_met)) {
      bRet = TRUE;
    } else {
      bRet = FALSE;
    }
  } else {
    if (pSIObj->uInlanePredictionTimer_nu > 0U) {
      pSIObj->uInlanePredictionTimer_nu =
          pSIObj->uInlanePredictionTimer_nu - 1U;
    }
  }
  return bRet;
}

/*****************************************************************************
  Functionname: SIGetPredictionTime                                  */ /*!

  @brief: Get prediction time to use for inlane/outlane prediction

  @description: This function returns the prediction time to use for a given object. This is a value between
				[SI_MIN_CUT_IN_PRED_TIME, SI_MAX_CUT_IN_PRED_TIME], basically limited by the object distance
				(since long time predictions for near object are too error prone)

  @param[in]: pGenObj->GenObjInfo.fDistX_met                        Object's longitudinal relative distance
			  pGenObj->GenObjInfo.fVrelX_mps                        Object's longitudinal relative velocity
			  pSRRObj->Qualifiers.fProbabilityOfExistence_per       Probability that the object represents a real object

  @return:    fPredTime                                             Prediction time [SI_MIN_CUT_IN_PRED_TIME, SI_MAX_CUT_IN_PRED_TIME]
*****************************************************************************/
float32 SIGetPredictionTime(
    const SIGenObject_st* pGenObj, const float32 fMinPredTime,
    const float32 fMaxPredTime)  // SISRRObject_st* pSRRObj,
{
  // Set default (minimal) prediction time
  float32 fPredTime = fMinPredTime;

  if (pGenObj->GenObjInfo.fProbabilityOfExistence_per >
      SI_MIN_PROB_OF_EXIST_PREDICTION) {
    if (((-(pGenObj->GenObjInfo.fDistX_met)) > TUE_C_F32_DELTA) &&
        (pGenObj->GenObjInfo.fVrelX_mps > TUE_C_F32_DELTA)) {
      fPredTime =
          pGenObj->GenObjInfo.fVrelX_mps / (-(pGenObj->GenObjInfo.fDistX_met));
    }
  }
  fPredTime = TUE_CML_MinMax(fMinPredTime, fMaxPredTime, fPredTime);
  return fPredTime;
}

/*****************************************************************************
  Functionname: SICheckInlaneTimer                                  */ /*!

  @brief: Check time condition for in lane decision of objects

  @description: Check time condition for in lane decision of objects

  @param[in]: pSIObj->ObjLaneLCAStatus.fCorridorRelevantTime_s     Time the object has been inside corridor
			  pGenObj->GenObjInfo.eDynamicProperty_nu              Object dynamic property

  @return:    bRet                                                 TRUE if object has satisfied inlane time criteria
*****************************************************************************/
boolean SICheckInlaneTimer(const SIGenObject_st* pGenObj,
                           const SILBSObjInfo_st* pLBSObjInfo,
                           SI_Info_st* pSIObj, SIVehicleInfo_t* pEgoInfo,
                           float32 fCycletime) {
  boolean bRet = FALSE;

  // Get inlane time threshold for considering DistX,egoVelocity,UpdateRate
  // and CurveRadiusMinFiltered. [0.1, 0.5]
  float32 fTimeThresh =
      SIGetInlaneTimeThreshold(pGenObj, pLBSObjInfo, pEgoInfo);
  if (pSIObj->ObjLaneLCAStatus.fCorridorRelevantTime_s > fTimeThresh) {
    if (pGenObj->GenObjInfo.eDynamicProperty_nu !=
        EM_GEN_OBJECT_DYN_PROPERTY_STATIONARY) {
      pSIObj->ObjLaneLCAStatus.fCorridorRelevantTime_s = 1.0F;
    }
    bRet = TRUE;
  } else {
    pSIObj->ObjLaneLCAStatus.fCorridorRelevantTime_s += fCycletime;
  }
  return bRet;
}

/*****************************************************************************
  Functionname: SIGetInlaneTimeThreshold                                  */ /*!

  @brief: Get threshold for time condition for in lane decision of objects

  @description: Return required time for object to be classified as inlane prior to getting the real inlane flag
				(e.g.: being selected for the 6 object interface)

  @param[in]: pGenObj->GenObjInfo.eDynamicProperty_nu       Object dynamic property,stationary,moving or oncoming
			  pGenObj->GenObjInfo.fDistX_met                Object's longitudinal relative distance
			  pEgoInfo->fegoVelocity_mps                    the ego vehicle longitudinal velocity
			  pLBSObjInfo->fUpdateRate_nu                   The object measurement update rate
			  pSIGlobals->fCurveRadiusMinFiltered_met       The minimum filter curve radius

  @return:    fTimeThreshInlane                             The object inlane time threshold
*****************************************************************************/
float32 SIGetInlaneTimeThreshold(const SIGenObject_st* pGenObj,
                                 const SILBSObjInfo_st* pLBSObjInfo,
                                 const SIVehicleInfo_t* pEgoInfo) {
  SI_Globals_t* pSIGlobals = GetSICalculatePointer_SIGlobals();
  float32 fTimeThreshInlane = 0.F;
  if (pGenObj->GenObjInfo.eDynamicProperty_nu !=
      EM_GEN_OBJECT_DYN_PROPERTY_STATIONARY) {
    // Set the inlane time threshold based on the distance of the object.
    // [InputMin,InputMax,TimeMin,TimeMax] Larger distance results in higher
    // uncertainty. [0.1, -100, 0, 0.5]
    fTimeThreshInlane = GDBmathLinFuncLimBounded(
        pGenObj->GenObjInfo.fDistX_met, SI_DISTX_MOVE_OBJ_IN_LANE_MIN,
        SI_DISTX_MOVE_OBJ_IN_LANE_MAX, SI_MINTIME_MOVE_OBJ_IN_LANE_MIN,
        SI_MINTIME_MOVE_OBJ_IN_LANE_MAX);
    // Increase the inlane time threshold for very low ownspeeds. [0, 7,
    // 0.5, 0]
    fTimeThreshInlane = fTimeThreshInlane +
                        GDBmathLinFuncLimBounded(pEgoInfo->fegoVelocity_mps,
                                                 SI_INLANE_TIME_MIN_EGOSPEED,
                                                 SI_INLANE_TIME_MAX_EGOSPEED,
                                                 SI_INLANE_TIME_SPE_MIN_TRSH,
                                                 SI_INLANE_TIME_SPE_MAX_TRSH);
    // Consider the update quality of the object. [1, 0.85, -0.15, 0]
    fTimeThreshInlane = fTimeThreshInlane +
                        GDBmathLinFuncLimBounded(pLBSObjInfo->fUpdateRate_nu,
                                                 SI_INLANE_TIME_MIN_UPDRATE,
                                                 SI_INLANE_TIME_MAX_UPDRATE,
                                                 SI_INLANE_TIME_UPRTE_MIN_TRSH,
                                                 SI_INLANE_TIME_UPRTE_MAX_TRSH);
    // Consider the currently driven curve radius as it adds more dynamics
    // to the scenario.[25, 250, 0.5, 0]
    fTimeThreshInlane =
        fTimeThreshInlane +
        GDBmathLinFuncLimBounded(
            pSIGlobals->fCurveRadiusMinFiltered_met, SI_INLANE_TIME_MIN_CURVE,
            SI_INLANE_TIME_MAX_CURVE, SI_INLANE_TIME_CRV_MIN_TRSH,
            SI_INLANE_TIME_CRV_MAX_TRSH);
    // Limit the inlane time threshold to a minumum of 0.1s and a maximum of
    // 0.5s
    fTimeThreshInlane = TUE_CML_MinMax(
        SI_INLANE_TIME_TRSH_MIN, SI_INLANE_TIME_TRSH_MAX, fTimeThreshInlane);
  } else  // stationary objects or vehicles
  {
    fTimeThreshInlane = TUE_C_F32_VALUE_INVALID;
  }
  return fTimeThreshInlane;
}

/*****************************************************************************
  Functionname: SICheckInlaneDistance                                  */ /*!

  @brief: Check distance condition for in lane decision of objects

  @description: Update given object's pSIObj->ObjLaneLCAStatus.fCorridorRelevantDist_met to get the ego
				driven distance during which object was in ego lane

  @param[in]: pEgoInfo->fegoVelocity_mps                            The ego vehicle longitudinal velocity
			  fCycletime                                            Cycle time
			  pEgoInfo->fegoAcceleration_mps2                       The ego vehicle longitudinal acceleration
			  pSIObj->ObjLaneLCAStatus.fCorridorRelevantDist_met    Distance the object has been inside corridor

  @return:    bRet             TRUE if ego vehicle traveled relevant distance to make an association for objects
*****************************************************************************/
boolean SICheckInlaneDistance(const SIGenObject_st* pGenObj, SI_Info_st* pSIObj,
                              SIVehicleInfo_t* pEgoInfo, float32 fCycletime) {
  boolean bRet = FALSE;
  float32 fDistThresh = 0.F;

  pSIObj->ObjLaneLCAStatus.fCorridorRelevantDist_met +=
      (pEgoInfo->fegoVelocity_mps * fCycletime) +
      (0.5F * pEgoInfo->fegoAcceleration_mps2 * fCycletime * fCycletime);
  pSIObj->ObjLaneLCAStatus.fCorridorRelevantDist_met =
      MIN(SI_INLANE_DIST_MAXVALUE,
          pSIObj->ObjLaneLCAStatus.fCorridorRelevantDist_met);

  fDistThresh = SIGetInlaneDistanceThreshold(pGenObj, pEgoInfo);
  if (pSIObj->ObjLaneLCAStatus.fCorridorRelevantDist_met > fDistThresh) {
    bRet = TRUE;
  } else {
    bRet = FALSE;
  }
  return bRet;
}

/*****************************************************************************
  Functionname: SIGetInlaneDistanceThreshold                                  */ /*!

  @brief: Get distance threshold for time condition for in lane decision of objects

  @description: Get distance threshold for time condition for in lane decision of objects

  @param[in]: pGenObj->GenObjInfo.eDynamicProperty_nu          Object dynamic property,stationary,moving or oncoming
			  pEgoInfo->fegoVelocity_mps                       the ego vehicle longitudinal velocity
			  pGenObj->GenObjInfo.eClassification_nu           Object classification

  @return:    fDistThreshInlane                                The distance threshold for given object
*****************************************************************************/
float32 SIGetInlaneDistanceThreshold(const SIGenObject_st* pGenObj,
                                     const SIVehicleInfo_t* pEgoInfo) {
  float32 fDistThreshInlane = 0.F;
  float32 fABSEgoVel = fABS(pEgoInfo->fegoVelocity_mps);

  if (pGenObj->GenObjInfo.eDynamicProperty_nu !=
      EM_GEN_OBJECT_DYN_PROPERTY_STATIONARY) {
    // Only use the time in lane for moving and stopped objects ->
    // deactivate the distance inlane
    fDistThreshInlane =
        (SI_INLANE_DIST_MAXVALUE + TUE_C_F32_DELTA);  // 200 + 0.0001
  } else  // Stationary object or vehicle
  {
    if ((pGenObj->GenObjInfo.eClassification_nu ==
         LBS_EM_GEN_OBJECT_CLASS_CAR) ||
        (pGenObj->GenObjInfo.eClassification_nu ==
         LBS_EM_GEN_OBJECT_CLASS_TRUCK) ||
        (pGenObj->GenObjInfo.eClassification_nu ==
         LBS_EM_GEN_OBJECT_CLASS_MOTOCYCLE)) {
      if (fABSEgoVel > SI_STATVEH_INLANE_HIGHSPEED)  //[100/3.6 m/s]
      {
        fDistThreshInlane = SI_STATVEH_INLANE_DISTMAX;      // 20m
      } else if (fABSEgoVel < SI_STATVEH_INLANE_PARKSPEED)  //[5/3.6 m/s]
      {
        fDistThreshInlane =
            MAX(SI_STATVEH_INLANE_DISTMIN,
                (fABSEgoVel * SI_STATVEH_INLANE_TIME_PARKSPEED));  // max[5,
        // fABSEgoVel*3.6]
      } else if (fABSEgoVel < SI_STATVEH_INLANE_LOWSPEED)  //[25/3.6 m/s]
      {
        fDistThreshInlane = SI_STATVEH_INLANE_DISTMIN;  // 5m
      } else {
        // only use the time inlane -> deactivate the distance inlane
        fDistThreshInlane =
            (SI_INLANE_DIST_MAXVALUE + TUE_C_F32_DELTA);  // 200 + 0.0001
      }
    } else {
      if (fABSEgoVel > SI_STATOBJ_INLANE_HIGHSPEED)  //[60/3.6 m/s]
      {
        fDistThreshInlane = SI_STATOBJ_INLANE_DISTMAX;      // 20m
      } else if (fABSEgoVel < SI_STATOBJ_INLANE_PARKSPEED)  //[5/3.6 m/s]
      {
        fDistThreshInlane =
            MAX(SI_STATOBJ_INLANE_DISTMIN,
                (fABSEgoVel * SI_STATOBJ_INLANE_TIME_PARKSPEED));  // max[5,
        // fABSEgoVel*3.6]
      } else if (fABSEgoVel <
                 SI_STATOBJ_INLANE_LOWSPEED)  //[5/3.6 m/s, 15/3.6 m/s]
      {
        fDistThreshInlane = SI_STATOBJ_INLANE_DISTMIN;  // 5m
      } else                                            //[15/3.6m/s, 60/3.6m/s]
      {
        // only use the time inlane -> deactivate the distance inlane
        fDistThreshInlane =
            (SI_INLANE_DIST_MAXVALUE + TUE_C_F32_DELTA);  // 200 + 0.0001
      }
    }
  }
  return fDistThreshInlane;
}

/*****************************************************************************
  Functionname: SIResetInlaneTimer                                  */ /*!

  @brief: Reset the inlane timer if timing conditions not fullfilled

  @description: Update (reset) object's pSIObj->ObjLaneLCAStatus.fCorridorRelevantTime_s field for hysteriesis

  @param[in]: pGenObj->GenObjInfo.eDynamicProperty_nu                Object dynamic property,stationary,moving or oncoming
			  pSIObj->ObjLaneLCAStatus.fCorridorRelevantTime_s       Time the object has been inside corridor
			  fCycletime                                             Cycle time
  @return:    void
*****************************************************************************/
void SIResetInlaneTimer(const SIGenObject_st* pGenObj, SI_Info_st* pSIObj,
                        float32 fCycletime) {
  if (pGenObj->GenObjInfo.eDynamicProperty_nu ==
      EM_GEN_OBJECT_DYN_PROPERTY_STATIONARY) {
    pSIObj->ObjLaneLCAStatus.fCorridorRelevantTime_s =
        pSIObj->ObjLaneLCAStatus.fCorridorRelevantTime_s - (2.0F * fCycletime);
    pSIObj->ObjLaneLCAStatus.fCorridorRelevantTime_s =
        MAX(pSIObj->ObjLaneLCAStatus.fCorridorRelevantTime_s, 0.F);
  } else {
    pSIObj->ObjLaneLCAStatus.fCorridorRelevantTime_s = 0.F;
  }
}

/*****************************************************************************
  Functionname: SIResetInlaneDistance                                  */ /*!

  @brief: Reset the inlane distance

  @description: Reset the inlane distance

  @param[in]: pSIObj->ObjLaneLCAStatus.fCorridorRelevantTime_s        Time the object has been inside corridor
			  pSIObj->ObjLaneLCAStatus.fCorridorRelevantDist_met      Distance the object has been inside corridor

  @return:void
*****************************************************************************/
void SIResetInlaneDistance(SI_Info_st* pSIObj) {
  // Only delete the distance inlane when the time inlane is 0
  if (pSIObj->ObjLaneLCAStatus.fCorridorRelevantTime_s < TUE_C_F32_DELTA) {
    pSIObj->ObjLaneLCAStatus.fCorridorRelevantDist_met = 0.0F;
  }
}

/*****************************************************************************
  Functionname: SICheckStateFlowInlaneToOutlane */ /*!

  @brief: Check the condition for state change from in lane to out of lane

  @description: Check the condition for state change from in lane to out of lane

  @param[in]: uObj, pGenObj, pSIObj, pLBSObjInfo, pOccupancy, reqPorts1, params

  @return:    bRet             TRUE if object is transitioning from inlane to outlane
*****************************************************************************/
boolean SICheckStateFlowInlaneToOutlane(
    uint8 uObj, const SIGenObject_st* pGenObj, SI_Info_st* pSIObj,
    const SILBSObjInfo_st* pLBSObjInfo, SITrajOccupancy_t* pOccupancy,
    SLACInReq_t reqPorts1, const SIParam_st* params) {
  boolean bRet = FALSE;
  boolean bOutOccValue = FALSE;
  boolean bOutCustomValue = FALSE;
  boolean bInOccValue = FALSE;
  boolean bInCustomValue = FALSE;
  float32 fCycletime = reqPorts1.fCycletime_s;

  SIRoad_t* pRoadInfo = reqPorts1.pRoad;

  // Check the occupancy values for out Lane decision of object
  bOutOccValue = SICheckOutlaneOccupancies(pGenObj, pSIObj, pOccupancy,
                                           pRoadInfo, fCycletime);
  // Check predicted criterias for object OutLane
  bOutCustomValue =
      SI_CheckCustomOutlaneCriteria(uObj, pGenObj, pSIObj, reqPorts1);

  // Update timer and distance information if outlane occupancies set
  if (bOutOccValue == TRUE) {
    SIResetInlaneTimer(pGenObj, pSIObj, fCycletime);
    SIResetInlaneDistance(pSIObj);
  }

  if ((bOutOccValue == TRUE) || (bOutCustomValue == TRUE)) {
    // At least one of the outlane criteria satisfied : object may be making
    // transition from inlane to outlane state. Next we need to check inlane
    // criteria, since if those aren't met, then the transition is OK.
    bInOccValue = SICheckInlaneOccupancies(pGenObj, pSIObj, pOccupancy);
    bInCustomValue =
        SI_CheckCustomInlaneCriteria(uObj, pGenObj, pSIObj, reqPorts1, params);

    if ((bInOccValue == TRUE) || (bInCustomValue == TRUE)) {
      // Since inlane criteria satisfied as well, these overule the
      // outlane criteria
      bRet = FALSE;
    } else {
      // Inlane criteria not satisfied : object really making transition
      // from inlane to outlane
      bRet = TRUE;
    }
  } else {
    // Neither outlane criteria satisfied : return FALSE (no state
    // transition)
    bRet = FALSE;
  }
  return bRet;
}

/*****************************************************************************
  Functionname: SICheckOutlaneOccupancies                                  */ /*!

  @brief: Check the occupancy values for outlane decision of object

  @description: Check the occupancy values for outlane decision of object

  @param[in]: pGenObj->GenObjInfo.fWidthLeft_met         Object's width left of the track position
			  pGenObj->GenObjInfo.fWidthRight_met        Object's width right of the track position
			  pOccupancy->fObjectOccupancy_per         the object with trace bracket overlap as a percentage of the object width
			  pOccupancy->fTrajectoryOccupancy_per     the object with trace bracket overlap as a percentage of the trace bracket width
			  pOccupancy->fOverlap_met                 the object with trace bracket overlap
			  pOccupancy->fOverlapVar_met              the object with trace bracket overlap variance
 @param[out]: pSIObj->SIBool.bOutLObjOccValue          Outlane overlap as a percentage of object occupancy check flag
			  pSIObj->SIBool.bOutLLaneOccValue         Outlane overlap as a percentage of trace bracket occupancy check flag
			  pSIObj->SIBool.bOutLLaneOverlapValue     Outlane overlap check flag

  @return:    bRet                                     TRUE if outlane occupancy is sufficient
*****************************************************************************/
boolean SICheckOutlaneOccupancies(const SIGenObject_st* pGenObj,
                                  SI_Info_st* pSIObj,
                                  SITrajOccupancy_t* pOccupancy,
                                  SIRoad_t* pRoadInfo, float32 fCycleTime) {
  boolean bRet = FALSE;
  boolean bObjOccValue = FALSE;
  boolean bLaneOccValue = FALSE;
  boolean bObjOverlapValue = FALSE;

  // Calculate object's width
  const float32 fObjWidth =
      pGenObj->GenObjInfo.fWidthLeft_met + pGenObj->GenObjInfo.fWidthRight_met;
  // Get threshold for the object occupancy value for inlane decision of
  // object
  const float32 fObjOccDropThresh =
      SIGetObjOccDropThreshold(pGenObj, pSIObj, pRoadInfo, fCycleTime);
  // Get threshold for the lane occupancy value for inlane decision of object
  const float32 fLaneOccDropThresh = SIGetLaneOccDropThreshold(pGenObj);
  // Check the object overlap value for inlane decision of object
  const float32 fOverlapDropThresh =
      SIGetOverlapDropThreshold(pGenObj, pSIObj, pRoadInfo, fCycleTime);

  // Check the object occupancy value for inlane decision of object
  if (pOccupancy->fObjectOccupancy_per < fObjOccDropThresh) {
    if (pOccupancy->fOverlapVar_met < ObjectOccupancyDropMaxOverlapVariance) {
      bObjOccValue = TRUE;
    } else {
      if (pOccupancy->fObjectOccupancy_per <=
          ObjectOccupancyDropThreshInsecureOverlap) {
        bObjOccValue = TRUE;
      }
    }
  }
  // Check the Lane occupancy value for inlane decision of object
  if (pOccupancy->fTrajectoryOccupancy_per < fLaneOccDropThresh) {
    bLaneOccValue = TRUE;
  }
  // Check the object overlap value for inlane decision of object
  if (pOccupancy->fOverlap_met < fOverlapDropThresh) {
    if (pOccupancy->fOverlapVar_met < OverlapDropMaxOverlapVariance) {
      bObjOverlapValue = TRUE;
    } else {
      if (pOccupancy->fOverlap_met <= OverlapDropThreshInsecureOverlap) {
        bObjOverlapValue = TRUE;
      }
    }
  }

  pSIObj->SIBool.bOutLObjOccValue = bObjOccValue;
  pSIObj->SIBool.bOutLLaneOccValue = bLaneOccValue;
  pSIObj->SIBool.bOutLLaneOverlapValue = bObjOverlapValue;

  // Check the occupancy values for outlane decision of object
  if (((bObjOccValue == TRUE) || (fObjWidth < SI_MIN_OBJECT_WIDTH)) &&
      (bLaneOccValue == TRUE) && (bObjOverlapValue == TRUE)) {
    bRet = TRUE;
  } else {
    bRet = FALSE;
  }

  pSIObj->SIBool.bOutLOccValue = bRet;
  return bRet;
}

/*****************************************************************************
  Functionname: SIGetObjOccDropThreshold                                  */ /*!

  @brief: Get threshold for the object occupancy value for inlane decision of object

  @description: Get threshold for the object occupancy value for inlane decision of object

  @param[in]: pGenObj->GenObjInfo.fWidthLeft_met                         Object's width left of the track position
			  pGenObj->GenObjInfo.fWidthRight_met                        Object's width right of the track position
			  pGenObj->GenObjInfo.eDynamicProperty_nu                  Object dynamic property,stationary,moving or oncoming
			  pSIObj->ObjLaneLCAStatus.fCorridorRelevantTime_s         Time the object has been inside corridor
			 fCycletime_s          Cycle time
			  pRoadInfo->iNumOfAdjacentLanes_nu
			  pRoadInfo->fYOffsetFused_met
			  pRoadInfo->fConfYOffset_per

  @return:    fObjOccDropThresh                                        Object occupancy drop threshold
*****************************************************************************/
float32 SIGetObjOccDropThreshold(const SIGenObject_st* pGenObj,
                                 SI_Info_st* pSIObj, SIRoad_t* pRoadInfo,
                                 float32 fCycleTime) {
  float32 fObjOccDropThresh = 0.F;
  float32 fObjWidth =
      pGenObj->GenObjInfo.fWidthLeft_met + pGenObj->GenObjInfo.fWidthRight_met;

  // Select threshold due to dynamic property
  if ((pGenObj->GenObjInfo.eDynamicProperty_nu ==
       EM_GEN_OBJECT_DYN_PROPERTY_STATIONARY) ||
      (pGenObj->GenObjInfo.eDynamicProperty_nu ==
       EM_GEN_OBJECT_DYN_PROPERTY_ONCOMING)) {
    fObjOccDropThresh = ObjectOccupancyDropThreshStat;  // 0.45
  } else {
    if (pSIObj->ObjLaneLCAStatus.fCorridorRelevantTime_s < 1.0F) {
      fObjOccDropThresh = ((ObjectOccupancyDropThresh - 0.1F) *
                           pSIObj->ObjLaneLCAStatus.fCorridorRelevantTime_s) +
                          0.1F;
      fObjOccDropThresh = MIN(ObjectOccupancyDropThresh, fObjOccDropThresh);
      pSIObj->ObjLaneLCAStatus.fCorridorRelevantTime_s += fCycleTime;
    } else {
      // For very wide objects (> 2m) the drop threshold is increased such
      // that these objects are dropped earlier
      fObjOccDropThresh = GDBmathLinFuncLimBounded(
          fObjWidth, 2.F, (2.F * SI_MAX_OBJECT_WIDTH_HALF), 0.F, 0.2F);
      fObjOccDropThresh = ObjectOccupancyDropThresh + fObjOccDropThresh;
      // Check if the guardrail is far away or there is no proper
      // guardrail information. In such a case the object measurement can
      // be assumed to be more precise. Hence the occupancy drop threshold
      // can be increased releasing objects in the relevent lane earlier.
      if ((pRoadInfo->iNumOfAdjacentLanes_nu > 1) &&
          ((pRoadInfo->fYOffsetFused_met > SI_DROP_TRSH_GRD_Y_OFFSET)    // 10
           || (pRoadInfo->fConfYOffset_per < SI_DROP_TRSH_GRD_Y_CONF)))  // 0.5
      {
        fObjOccDropThresh = fObjOccDropThresh + 0.2F;
      } else {
        fObjOccDropThresh = fObjOccDropThresh;
      }
    }
  }
  return fObjOccDropThresh;
}

/*****************************************************************************
  Functionname: SIGetLaneOccDropThreshold                                  */ /*!

  @brief: Get threshold for the lane occupancy value for inlane decision of object

  @description: Get threshold for the lane occupancy value for inlane decision of object

  @param[in]: pGenObj->GenObjInfo.eDynamicProperty_nu       Object dynamic property,stationary,moving or oncoming

  @return:    fLaneOccDropThresh                            Lane occupancy drop threshold
*****************************************************************************/
float32 SIGetLaneOccDropThreshold(const SIGenObject_st* pGenObj) {
  float32 fLaneOccDropThresh = 0.F;

  // Select threshold due to dynamic property
  if ((pGenObj->GenObjInfo.eDynamicProperty_nu ==
       EM_GEN_OBJECT_DYN_PROPERTY_STATIONARY) ||
      (pGenObj->GenObjInfo.eDynamicProperty_nu ==
       EM_GEN_OBJECT_DYN_PROPERTY_ONCOMING)) {
    fLaneOccDropThresh = LaneOccupancyDropThreshStat;  // 0.2
  } else {
    fLaneOccDropThresh = LaneOccupancyDropThresh;  // 0.3
  }
  return fLaneOccDropThresh;
}

/*****************************************************************************
  Functionname: SIGetOverlapDropThreshold                                  */ /*!

  @brief: Get threshold for the object overlap value for inlane decision of object

  @description: Get threshold for the object overlap value for inlane decision of object

  @param[in]: pGenObj->GenObjInfo.fWidthLeft_met                        Object's width left of the track position
			  pGenObj->GenObjInfo.fWidthRight_met                       Object's width right of the track position
			  pGenObj->GenObjInfo.eDynamicProperty_nu                 Object dynamic property,stationary,moving or oncoming
			  pSIObj->ObjLaneLCAStatus.fCorridorRelevantTime_s        Time the object has been inside corridor
			  fCycletime_s         Cycle time
			  pRoadInfo->iNumOfAdjacentLanes_nu
			  pRoadInfo->fYOffsetFused_met
			  pRoadInfo->fConfYOffset_per

  @return:    fOverlapDropThresh                                      Lane overlap drop threshold for given object
*****************************************************************************/
float32 SIGetOverlapDropThreshold(const SIGenObject_st* pGenObj,
                                  SI_Info_st* pSIObj, SIRoad_t* pRoadInfo,
                                  float32 fCycleTime) {
  float32 fOverlapDropThresh = 0.F;
  float32 fObjWidth =
      pGenObj->GenObjInfo.fWidthLeft_met + pGenObj->GenObjInfo.fWidthRight_met;

  // Select threshold due to dynamic property
  if ((pGenObj->GenObjInfo.eDynamicProperty_nu ==
       EM_GEN_OBJECT_DYN_PROPERTY_STATIONARY) ||
      (pGenObj->GenObjInfo.eDynamicProperty_nu ==
       EM_GEN_OBJECT_DYN_PROPERTY_ONCOMING)) {
    fOverlapDropThresh = LaneOverlapDropThreshStat;
  } else {
    // FCT_SEN_MOUNTED_FRONT
    // Increase inlane cycle counter
    // if (FCTMountingState == FCT_SEN_MOUNTED_FRONT)
    //{
    //	pSIObj->ObjLaneLCAStatus.uInlaneCycleCounter_nu += 1;
    //	pSIObj->ObjLaneLCAStatus.uInlaneCycleCounter_nu =
    // MIN(pSIObj->ObjLaneLCAStatus.uInlaneCycleCounter_nu,
    // TUE_C_UI8_VALUE_INVALID);
    //}

    if (pSIObj->ObjLaneLCAStatus.fCorridorRelevantTime_s < 1.0F) {
      fOverlapDropThresh = ((ObjectLaneOverlapDropThresh - 0.1F) *
                            pSIObj->ObjLaneLCAStatus.fCorridorRelevantTime_s) +
                           0.1F;
      fOverlapDropThresh = MIN(ObjectOccupancyDropThresh, fOverlapDropThresh);
      pSIObj->ObjLaneLCAStatus.fCorridorRelevantTime_s += fCycleTime;
    } else {
      // Check if the guardrail is far away or there is no proper
      // guardrail information. In such a case the object measurement can
      // be assumed to be more precise. Hence the occupancy drop threshold
      // can be increased releasing objects in the relevant lane earlier.
      if ((pRoadInfo->iNumOfAdjacentLanes_nu > 1) &&
          ((pRoadInfo->fYOffsetFused_met > SI_DROP_TRSH_GRD_Y_OFFSET) ||
           (pRoadInfo->fConfYOffset_per < SI_DROP_TRSH_GRD_Y_CONF)) &&
          (fObjWidth > (2.F * SI_MIN_OBJECT_WIDTH))) {
        fOverlapDropThresh = ObjectLaneOverlapDropThresh + 0.1F;
      } else {
        fOverlapDropThresh = ObjectLaneOverlapDropThresh;
      }
    }
  }
  return fOverlapDropThresh;
}

/*****************************************************************************
  Functionname: SI_CheckCustomOutlaneCriteria */ /*!

  @brief: Check customer specific outlane criteria

  @description: Custom outlane criteria check function

  @param[in]: pGenObj->GenObjInfo.eDynamicProperty_nu         Object dynamic property,stationary,moving or oncoming
  @param[out]:pSIObj->SIBool.bOutLCustomValue                 Custom outlane check flag

  @return:    bRet                                            TRUE if object is inlane based on custom criteria
															  FALSE if object is outlane based on custom criteria
*****************************************************************************/
boolean SI_CheckCustomOutlaneCriteria(uint8 uObj, const SIGenObject_st* pGenObj,
                                      SI_Info_st* pSIObj,
                                      SLACInReq_t reqPorts1) {
  boolean bRet = FALSE;

  // Predicted lane association not for stationary objects
  if (pGenObj->GenObjInfo.eDynamicProperty_nu !=
      EM_GEN_OBJECT_DYN_PROPERTY_STATIONARY) {
    bRet = SI_CheckPredictedOutlaneCriteria(uObj, pGenObj, pSIObj, reqPorts1);
  } else {
    bRet = FALSE;
  }
  pSIObj->SIBool.bOutLCustomValue = bRet;
  return bRet;
}

/*****************************************************************************
  Functionname: SI_CheckPredictedOutlaneCriteria */ /*!

  @brief: Check predicted criterias for object inlane

  @description: Check predicted criterias for object inlane

  @param[in]: pGenObj->GenObjInfo.uiLifeCycles_nu                    Object lifetime in cycles
			  pGenObj->GenObjInfo.fDistX_met                      Object's longitudinal relative distance
			  pGenObj->GenObjInfo.fVrelX_mps                      Object's longitudinal relative velocity
			  pGenObj->GenObjInfo.fWidthLeft_met                    Object's width left of the track position
			  pGenObj->GenObjInfo.fWidthRight_met                   Object's width right of the track position

			  pEgoInfo->fegoVelocity_mps                          The ego vehicle longitudinal velocity
			  pRoad->fCurveRadiusMinFiltered_met
			  pRoad->fCurveRadius_met                             The curvature radius of the current driver road

			  pSIObj->ObjTrajRefPoint.fDistToTraj_met             Distance to trajectory
			  pSIObj->fVrelToTraj_mps
			  pSIObj->ObjTraceBracket.fTraceBracketLeft_met       The trace bracket left side coordinate
			  pSIObj->ObjTraceBracket.fTraceBracketRight_met      The trace bracket right side coordinate
			  pSIObj->uOutlanePredictionTimer_nu                  Predict the number of time outlane
  @return:void
*****************************************************************************/
boolean SI_CheckPredictedOutlaneCriteria(uint8 uObj,
                                         const SIGenObject_st* pGenObj,
                                         SI_Info_st* pSIObj,
                                         SLACInReq_t reqPorts1) {
  boolean bRet = FALSE;
  SIPredictedDistance_t* pSIPredictedDist =
      GetSICalculatePointer_SIPredictedDist();
  float32 fPredDistLeftEdge = 0.0F;
  float32 fPredDistRightEdge = 0.0F;
  float32 fThreshLeftBracket = 0.0F;
  float32 fThreshRightBracket = 0.0F;
  float32 fCurveRadiusMinFiltered_met =
      GetSICalculatePointer_SIGlobals()->fCurveRadiusMinFiltered_met;

  SIVehicleInfo_t* pEgoInfo = reqPorts1.pEgoVehInfo;
  SIRoad_t* pRoad = reqPorts1.pRoad;
  // SISRRObject_st* pSRRObj = &reqPorts1.SRRObjList.aObject[uObj];
  // Get prediction time to use for outlane prediction
  float32 fPredictionTime =
      SIGetPredictionTime(pGenObj, SI_MIN_CUT_OUT_PRED_TIME,
                          SI_MAX_CUT_OUT_PRED_TIME);  // pSRRObj,
  if ((fPredictionTime > TUE_C_F32_DELTA) &&
      (pGenObj->GenObjInfo.uiLifeCycles_nu >
       SI_PRED_LANE_ASSOC_MIN_OBJ_LIFETIME) &&  //2
      (pGenObj->GenObjInfo.fDistX_met < SI_PRED_LANE_ASSOC_X_OBJ_MAX) &&
      (pGenObj->GenObjInfo.fDistX_met > SI_PRED_LANE_ASSOC_X_OBJ_MIN) &&
      (pGenObj->GenObjInfo.fVrelX_mps < SI_PRED_LANE_ASSOC_VX_OBJ_MAX) &&
      (pEgoInfo->fegoVelocity_mps > SI_PRED_LANE_ASSOC_VEGO_MIN) &&
      ((-(fCurveRadiusMinFiltered_met)) >
       SI_PRED_LANE_ASSOC_MIN_Y_OPP_BORDER) &&
      (TUE_CML_Min(fABS(pRoad->fDrivenCurveRadius_met),
                   fABS(pRoad->fCurveRadius_met)) >
       SI_PRED_LANE_ASSOC_CURVE_RAD_MIN)) {
    // Calculate displacement now
    // The predicted displacement is the actual position plus the product of
    // gradient and prediction time
    pSIPredictedDist->pdist = (pSIObj->ObjTrajRefPoint.fDistToTraj_met) +
                              (fPredictionTime * pSIObj->fVrelToTraj_mps);
    pSIPredictedDist->pdist_var =
        0.F;  // DistanceToTrajVar + (SQR(prediction_time) * VRelToTrajVar *
              // fVRelToTrajVarFac)
    pSIPredictedDist->pdist_var_fullpred =
        0.F;  // DistanceToTrajVar + (SQR(SiLaneAssPredTime) * VRelToTrajVar
              // * fVRelToTrajVarFac)

    // Calculate sides of predicted object and trace bracket
    fPredDistLeftEdge =
        pSIPredictedDist->pdist + pGenObj->GenObjInfo.fWidthLeft_met;
    fPredDistRightEdge =
        pSIPredictedDist->pdist - pGenObj->GenObjInfo.fWidthRight_met;
    fThreshLeftBracket = pSIObj->ObjTraceBracket.fTraceBracketLeft_met + 0.5F;
    fThreshRightBracket = pSIObj->ObjTraceBracket.fTraceBracketRight_met - 0.5F;

    // Check:
    // -- if the predicted left edge
    if ((fPredDistLeftEdge < fThreshRightBracket) ||
        (fPredDistRightEdge > fThreshLeftBracket)) {
      if (pSIObj->uOutlanePredictionTimer_nu < SI_PRED_LANE_ASSOC_TIME_MAX) {
        pSIObj->uOutlanePredictionTimer_nu =
            pSIObj->uOutlanePredictionTimer_nu + 1U;
      }
    } else {
      if (pSIObj->uOutlanePredictionTimer_nu > 0U) {
        pSIObj->uOutlanePredictionTimer_nu =
            pSIObj->uOutlanePredictionTimer_nu - 1U;
      }
    }

    if (pSIObj->uOutlanePredictionTimer_nu >
        SI_PRED_LANE_ASSOC_TIME_OUT_ENABLE) {
      bRet = TRUE;
    } else {
      bRet = FALSE;
    }
  } else {
    if (pSIObj->uOutlanePredictionTimer_nu > 0U) {
      pSIObj->uOutlanePredictionTimer_nu =
          pSIObj->uOutlanePredictionTimer_nu - 1U;
    }
  }
  return bRet;
}

/*****************************************************************************
  Functionname: SIGetAssociateLane                                  */ /*!

  @brief: Associates an object with one of the road lanes

  @description: Associates an object with one of the road lanes

  @param[in]: pGenObj->GenObjInfo.bRightSensor
			  GetSICalculatePointer_SIGlobals()->fLaneWidth_met          Base lane width,estimated by EM Road
			  pSIObj->ObjTrajRefPoint.fDistToTraj_met                    Distance to trajectory
			  pSIObj->ObjLaneLCAStatus.SIInlaneState                     Lane assignment
  @param[out]:GetSICalculatePointer()->eAssociatedLaneList[uObj]         Lane Association
  @return:void
*****************************************************************************/
void SIGetAssociateLane(uint8 uObj, const SIGenObject_st* pGenObj,
                        SI_Info_st* pSIObj) {
  eAssociatedLane_t uObjectLane = 0;

  // if (pGenObj->GenObjInfo.bRightSensor == TRUE)
  //{
  //	pSIObj->ObjTrajRefPoint.fDistToTraj_met =
  //-(pSIObj->ObjTrajRefPoint.fDistToTraj_met);
  //}

  // Check if object is associated to inlane
  if (pSIObj->ObjLaneLCAStatus.SIInlaneState == OBJ_STATE_INLANE) {
    if (!pGenObj->GenObjInfo.bRightSensor) {
      uObjectLane = ASSOC_LANE_LEFT;
    } else {
      uObjectLane = ASSOC_LANE_RIGHT;
    }
  }
  // Check if object is on right side of lane
  else if (pSIObj->ObjTrajRefPoint.fDistToTraj_met <
               (0.75F * GetSICalculatePointer_SIGlobals()->fLaneWidth_met) &&
           pSIObj->ObjTrajRefPoint.fDistToTraj_met >
               -(0.75F * GetSICalculatePointer_SIGlobals()->fLaneWidth_met)) {
    uObjectLane = ASSOC_LANE_EGO;
  }
  // object is on ego lane or far left side
  else {
    if (!pGenObj->GenObjInfo.bRightSensor) {
      uObjectLane = ASSOC_LANE_FAR_LEFT;
    } else {
      uObjectLane = ASSOC_LANE_FAR_RIGHT;
    }
  }
  GetSICalculatePointer()->eAssociatedLaneList[uObj] = uObjectLane;
}

/*****************************************************************************
  Functionname: SIPostProcess                                  */ /*!

  @brief: Output the lane association information

  @description: Output the lane association information

  @param[in]: 
              uObj          object array number
              proPorts      the output structure
			  debugInfo     the debug structure
  @param[out]:
              eAssociatedLane              the result of lane association
			  fDistToTraj_met              distance from object position to trajectory
			  fVrelToTraj_mps              vrel from object position to trajectory
			  fTraceBracketLeft_met        The trace bracket left side coordinate
			  fTraceBracketRight_met       The trace bracket right side coordinate
			  fObjBracketOverlap_met       Overlap between object and trace bracket
  @return:void 
*****************************************************************************/
void SIPostProcess(uint8 uObj, SIOutPut_st* proPorts, SIDebug_st* debugInfo) {
  proPorts->SIObjInfoList[uObj].eAssociatedLane =
      SICalculate.eAssociatedLaneList[uObj];
  proPorts->SIObjInfoList[uObj].fDistToTraj_met =
      SICalculate.SIObjInfoList[uObj].ObjTrajRefPoint.fDistToTraj_met;
  proPorts->SIObjInfoList[uObj].fVrelToTraj_mps =
      SICalculate.SIObjInfoList[uObj].fVrelToTraj_mps;
  proPorts->SIObjInfoList[uObj].fTraceBracketLeft_met =
      SICalculate.SIObjInfoList[uObj].ObjTraceBracket.fTraceBracketLeft_met;
  proPorts->SIObjInfoList[uObj].fTraceBracketRight_met =
      SICalculate.SIObjInfoList[uObj].ObjTraceBracket.fTraceBracketRight_met;
  proPorts->SIObjInfoList[uObj].fObjBracketOverlap_met =
      SICalculate.SIObjInfoList[uObj].fObjBracketOverlap_met;
}