#include "lbs_wrapper.h"

/*****************************************************************************
  SI FUNCTION PROTOTYPES
*****************************************************************************/
void LBSSIInputWrapper(const LBSInReq_st* extReqPorts,
                       const LBSParam_st* extReqParam, SIInPut_st* input,
                       SIParam_st* paramInput, LBSCalculate_st* pLBSCalc) {
  // memcpy LBSParam to SIParam
  // memcpy(&paramInput->pSensorMounting, &extReqParam->SensorMounting,
  // sizeof(LBS_SensorMounting_st)); memcpy(&paramInput->pVehParameter,
  // &extReqParam->VehParameter, sizeof(LBS_VehParameter_t));

  // Ego vehicle information
  input->EgoVehInfo.fegoVelocity_mps = extReqPorts->EgoVehInfo.fegoVelocity_mps;
  input->EgoVehInfo.fegoAcceleration_mps2 =
      extReqPorts->EgoVehInfo.fegoAcceleration_mps2;
  // Road information
  input->Road.fC0Fused_1pm = extReqPorts->Road.fC0Fused_1pm;
  input->Road.fC1Fused_1pm2 = extReqPorts->Road.fC1Fused_1pm2;
  input->Road.fConfAdjacentLanes_per = extReqPorts->Road.fConfAdjacentLanes_per;
  input->Road.fConfOppositeLanes_per = extReqPorts->Road.fConfOppositeLanes_per;
  input->Road.fConfYOffset_per = extReqPorts->Road.fConfYOffset_per;
  input->Road.fConfYOppOffset_per = extReqPorts->Road.fConfYOppOffset_per;
  input->Road.fCurveRadius_met = extReqPorts->Road.fCurveRadius_met;
  input->Road.fDrivenCurveRadius_met = extReqPorts->Road.fDrivenCurveRadius_met;
  input->Road.fLaneWidth_met = extReqPorts->Road.fLaneWidth_met;
  input->Road.fYawAngleFused_rad = extReqPorts->Road.fYawAngleFused_rad;
  input->Road.fYOffsetFused_met = extReqPorts->Road.fYOffsetFused_met;
  input->Road.fYOffsetFusedOppBorder_met =
      extReqPorts->Road.fYOffsetFusedOppBorder_met;
  input->Road.iNumOfAdjacentLanes_nu = extReqPorts->Road.iNumOfAdjacentLanes_nu;
  input->Road.iNumOfOppositeLanes_nu = extReqPorts->Road.iNumOfOppositeLanes_nu;

  input->SISysParam.fCycletime_s = extReqPorts->LBSSystemParam.fCycletime_s;

  input->GenObjList.sSigHeader.eSigStatus_nu = 1U;
  for (uint8 uObjIndex = 0U; uObjIndex < LBS_INPUT_OBJECT_NUMBER; uObjIndex++) {
    // EM general object list information
    input->GenObjList.aObject[uObjIndex].GenObjInfo.bRightSensor =
        extReqPorts->GenObjList.aObject[uObjIndex].bRightSensor;
    input->GenObjList.aObject[uObjIndex].GenObjInfo.eClassification_nu =
        extReqPorts->GenObjList.aObject[uObjIndex]
            .Attributes.eClassification_nu;
    input->GenObjList.aObject[uObjIndex].GenObjInfo.eDynamicProperty_nu =
        extReqPorts->GenObjList.aObject[uObjIndex]
            .Attributes.eDynamicProperty_nu;
    input->GenObjList.aObject[uObjIndex].GenObjInfo.fDistX_met =
        extReqPorts->GenObjList.aObject[uObjIndex].Kinemactic.fDistX_met;
    input->GenObjList.aObject[uObjIndex].GenObjInfo.fDistY_met =
        extReqPorts->GenObjList.aObject[uObjIndex].Kinemactic.fDistY_met;
    input->GenObjList.aObject[uObjIndex].GenObjInfo.fVrelX_mps =
        extReqPorts->GenObjList.aObject[uObjIndex].Kinemactic.fVrelX_mps;
    input->GenObjList.aObject[uObjIndex].GenObjInfo.fWidthLeft_met =
        extReqPorts->GenObjList.aObject[uObjIndex].Geometry.fWidthLeft_met;
    input->GenObjList.aObject[uObjIndex].GenObjInfo.fWidthRight_met =
        extReqPorts->GenObjList.aObject[uObjIndex].Geometry.fWidthRight_met;
    input->GenObjList.aObject[uObjIndex].GenObjInfo.uiLifeCycles_nu =
        extReqPorts->GenObjList.aObject[uObjIndex].General.uiLifeCycles_nu;
    input->GenObjList.aObject[uObjIndex].GenObjInfo.uiMaintenanceState_nu =
        extReqPorts->GenObjList.aObject[uObjIndex]
            .General.uiMaintenanceState_nu;
    input->GenObjList.aObject[uObjIndex]
        .GenObjInfo.fProbabilityOfExistence_per =
        extReqPorts->SRRObjList.aObject[uObjIndex]
            .Qualifiers.fProbabilityOfExistence_per;
    input->GenObjList.aObject[uObjIndex].GenObjInfo.fDist2Course_met =
        extReqPorts->SRRObjList.aObject[uObjIndex]
            .RoadRelation.fDist2Course_met;
    input->GenObjList.aObject[uObjIndex].GenObjInfo.fFirstDetectX_met =
        extReqPorts->SRRObjList.aObject[uObjIndex].History.fFirstDetectX_met;

    // LBS object information
    input->LBSObjInfoList[uObjIndex].fUpdateRate_nu =
        pLBSCalc->LBSObjInfoList[uObjIndex].fUpdateRate_nu;
    // LCA object information
    input->LCAObjInfoList[uObjIndex].bLCAWarning =
        pLBSCalc->LCAObjInfoList[uObjIndex].bLCAWarning;
  }

  paramInput->VehParameter.fVehicleWidth_met =
      extReqParam->LBS_Ks_VehParameter_nu.LBS_Kf_VehicleWidth_met;
  // paramInput->LCAParamter.fDefaultLaneWidth =
  // extReqParam->LBSLCAParameter.fDefaultLaneWidth_met;
  paramInput->LCAParamter.LCAZone.fLCAZoneXMid_met =
      extReqParam->LBS_Ks_LCAParameter_nu.LBS_Ks_LCAZone_nu
          .LBS_Kf_LCAZoneXMid_met;
  paramInput->LCAParamter.LCAZone.fLCAZoneXMin_met =
      extReqParam->LBS_Ks_LCAParameter_nu.LBS_Ks_LCAZone_nu
          .LBS_Kf_LCAZoneXMin_met;
  paramInput->LCAParamter.LCAZone.fLCAZoneYMaxFar_met =
      extReqParam->LBS_Ks_LCAParameter_nu.LBS_Ks_LCAZone_nu
          .LBS_Kf_LCAZoneYMaxFar_met;
  paramInput->LCAParamter.LCAZone.fLCAZoneYMaxNear_met =
      extReqParam->LBS_Ks_LCAParameter_nu.LBS_Ks_LCAZone_nu
          .LBS_Kf_LCAZoneYMaxNear_met;
  paramInput->LCAParamter.LCAZone.fLCAZoneYMinFar_met =
      extReqParam->LBS_Ks_LCAParameter_nu.LBS_Ks_LCAZone_nu
          .LBS_Kf_LCAZoneYMinFar_met;
  paramInput->LCAParamter.LCAZone.fLCAZoneYMinNear_met =
      extReqParam->LBS_Ks_LCAParameter_nu.LBS_Ks_LCAZone_nu
          .LBS_Kf_LCAZoneYMinNear_met;
  // Sensor mounting parameter
  paramInput->SensorMounting.SensorLeft.fLatPos_met =
      extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu
          .LBS_Kf_LatPos_met;
  paramInput->SensorMounting.SensorLeft.fLongPos_met =
      extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu
          .LBS_Kf_LongPos_met;
  paramInput->SensorMounting.SensorLeft.fLongPosToCoG_met =
      extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu
          .LBS_Kf_LongPosToCoG_met;
  paramInput->SensorMounting.SensorLeft.fOrientation_rad =
      extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu
          .LBS_Kf_Orientation_rad;
  paramInput->SensorMounting.SensorLeft.fPitchAngle_rad =
      extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu
          .LBS_Kf_PitchAngle_rad;
  paramInput->SensorMounting.SensorLeft.fRollAngle_rad =
      extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu
          .LBS_Kf_RollAngle_rad;
  paramInput->SensorMounting.SensorLeft.fVertPos_met =
      extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu
          .LBS_Kf_VertPos_met;
  paramInput->SensorMounting.SensorLeft.fYawAngle_rad =
      extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu
          .LBS_Kf_YawAngle_rad;

  paramInput->SensorMounting.SensorRight.fLatPos_met =
      extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu
          .LBS_Kf_LatPos_met;
  paramInput->SensorMounting.SensorRight.fLongPos_met =
      extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu
          .LBS_Kf_LongPos_met;
  paramInput->SensorMounting.SensorRight.fLongPosToCoG_met =
      extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu
          .LBS_Kf_LongPosToCoG_met;
  paramInput->SensorMounting.SensorRight.fOrientation_rad =
      extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu
          .LBS_Kf_Orientation_rad;
  paramInput->SensorMounting.SensorRight.fPitchAngle_rad =
      extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu
          .LBS_Kf_PitchAngle_rad;
  paramInput->SensorMounting.SensorRight.fRollAngle_rad =
      extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu
          .LBS_Kf_RollAngle_rad;
  paramInput->SensorMounting.SensorRight.fVertPos_met =
      extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu
          .LBS_Kf_VertPos_met;
  paramInput->SensorMounting.SensorRight.fYawAngle_rad =
      extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu
          .LBS_Kf_YawAngle_rad;
}

void LBSSIOutputWrapper(LBSCalculate_st* pLBSCalc, LBSDebug_t* extDebugPorts,
                        SIOutPut_st* pOutput, LBSSIDebug_t* pDebug,
                        SICalculate_st* pSICalc) {
  memcpy(&extDebugPorts->SIDebug.SI_Info_Debug,  //NOLINT
         &pSICalc->SIObjInfoList, sizeof(SI_Info_st) * SI_INPUT_OBJECT_NUMBER);

  memcpy(&extDebugPorts->SIDebug.SI_Global_Debug,  //NOLINT
         &pSICalc->SIGlobals, sizeof(SI_Globals_t));

  for (uint8 uObjIndex = 0U; uObjIndex < LBS_INPUT_OBJECT_NUMBER; uObjIndex++) {

    pLBSCalc->SIObjInfoList[uObjIndex].eAssociatedLane =
        pOutput->SIObjInfoList[uObjIndex].eAssociatedLane;

    pLBSCalc->SIObjInfoList[uObjIndex].fDistToTraj_met =
        pOutput->SIObjInfoList[uObjIndex].fDistToTraj_met;

    pLBSCalc->SIObjInfoList[uObjIndex].fObjBracketOverlap_met =
        pOutput->SIObjInfoList[uObjIndex].fObjBracketOverlap_met;
    pLBSCalc->SIObjInfoList[uObjIndex].fTraceBracketLeft_met =
        pOutput->SIObjInfoList[uObjIndex].fTraceBracketLeft_met;
    pLBSCalc->SIObjInfoList[uObjIndex].fTraceBracketRight_met =
        pOutput->SIObjInfoList[uObjIndex].fTraceBracketRight_met;
    pLBSCalc->SIObjInfoList[uObjIndex].fVrelToTraj_mps =
        pOutput->SIObjInfoList[uObjIndex].fVrelToTraj_mps;
  }
}

/*****************************************************************************
  BSD FUNCTION PROTOTYPES
*****************************************************************************/
void BSDInputWrapper(const LBSInReq_st* extReqPorts, BSDInReq_st* input,
                     const LBSParam_st* extReqParam, BSDParam_st* paramInput) {
  const LBSCalculate_st* pLBSCalculate = pGetLBSCalculatePointer();
  const LBS_Globals_t* pLBSGlobals = pGeLBSCalculatePointer_LBSGlobals();
  const LBSBSDParameter_t* pBSDParam = &extReqParam->LBS_Ks_BSDParameter_nu;
  uint8 uObj = 0;
  // DEBUG_Print("BSDInputWrapper\n");
  if (sizeof(BsdZone_t) != sizeof(LBSBsdZone_t)) {
    DEBUG_Print("BSD BSDInputWrapper Error!");
    return;
  }
  memcpy(&paramInput->BsdZone, &pBSDParam->LBS_Ks_BSDZoneParameter_nu,  //NOLINT
         sizeof(LBSBsdZone_t));
  // set Object List input
  for (uObj = 0; uObj < LBS_INPUT_OBJECT_NUMBER; uObj++) {
    // set General Object List input
    input->GenObjList.aObject[uObj].Attributes.eClassification_nu =
        extReqPorts->GenObjList.aObject[uObj].Attributes.eClassification_nu;
    input->GenObjList.aObject[uObj].Attributes.eDynamicProperty_nu =
        extReqPorts->GenObjList.aObject[uObj].Attributes.eDynamicProperty_nu;
    input->GenObjList.aObject[uObj].Attributes.uiClassConfidence_per =
        extReqPorts->GenObjList.aObject[uObj].Attributes.uiClassConfidence_per;
    input->GenObjList.aObject[uObj].Attributes.uiDynConfidence_per =
        extReqPorts->GenObjList.aObject[uObj].Attributes.uiDynConfidence_per;

    input->GenObjList.aObject[uObj].General.fLifeTime_s =
        extReqPorts->GenObjList.aObject[uObj].General.fLifeTime_s;
    input->GenObjList.aObject[uObj].General.uiID_nu =
        extReqPorts->GenObjList.aObject[uObj].General.uiID_nu;
    input->GenObjList.aObject[uObj].General.uiLifeCycles_nu =
        extReqPorts->GenObjList.aObject[uObj].General.uiLifeCycles_nu;
    input->GenObjList.aObject[uObj].General.uiMaintenanceState_nu =
        extReqPorts->GenObjList.aObject[uObj].General.uiMaintenanceState_nu;

    input->GenObjList.aObject[uObj].Geometry.fLength_met =
        extReqPorts->GenObjList.aObject[uObj].Geometry.fLength_met;
    input->GenObjList.aObject[uObj].Geometry.fLengthFront_met =
        extReqPorts->GenObjList.aObject[uObj].Geometry.fLengthFront_met;
    input->GenObjList.aObject[uObj].Geometry.fLengthRear_met =
        extReqPorts->GenObjList.aObject[uObj].Geometry.fLengthRear_met;
    input->GenObjList.aObject[uObj].Geometry.fWidth_met =
        extReqPorts->GenObjList.aObject[uObj].Geometry.fWidth_met;
    input->GenObjList.aObject[uObj].Geometry.fWidthLeft_met =
        extReqPorts->GenObjList.aObject[uObj].Geometry.fWidthLeft_met;
    input->GenObjList.aObject[uObj].Geometry.fWidthRight_met =
        extReqPorts->GenObjList.aObject[uObj].Geometry.fWidthRight_met;

    input->GenObjList.aObject[uObj].Kinemactic.fArelX_mpss =
        extReqPorts->GenObjList.aObject[uObj].Kinemactic.fArelX_mpss;
    input->GenObjList.aObject[uObj].Kinemactic.fArelY_mpss =
        extReqPorts->GenObjList.aObject[uObj].Kinemactic.fArelY_mpss;
    input->GenObjList.aObject[uObj].Kinemactic.fDistX_met =
        extReqPorts->GenObjList.aObject[uObj].Kinemactic.fDistX_met;
    input->GenObjList.aObject[uObj].Kinemactic.fDistY_met =
        extReqPorts->GenObjList.aObject[uObj].Kinemactic.fDistY_met;
    input->GenObjList.aObject[uObj].Kinemactic.fVrelX_mps =
        extReqPorts->GenObjList.aObject[uObj].Kinemactic.fVrelX_mps;
    input->GenObjList.aObject[uObj].Kinemactic.fVrelX_mps =
        extReqPorts->GenObjList.aObject[uObj].Kinemactic.fVrelX_mps;

    input->GenObjList.aObject[uObj].bRightSensor =
        extReqPorts->GenObjList.aObject[uObj].bRightSensor;

    // set SRR Object List input
    input->SRRObjList.aObject[uObj].History.fFirstDetectX_met =
        extReqPorts->SRRObjList.aObject[uObj].History.fFirstDetectX_met;
    input->SRRObjList.aObject[uObj].History.fFirstDetectY_met =
        extReqPorts->SRRObjList.aObject[uObj].History.fFirstDetectY_met;

    input->SRRObjList.aObject[uObj].Qualifiers.bObjStable =
        extReqPorts->SRRObjList.aObject[uObj].Qualifiers.bObjStable;
    input->SRRObjList.aObject[uObj].Qualifiers.fProbabilityOfExistence_per =
        extReqPorts->SRRObjList.aObject[uObj]
            .Qualifiers.fProbabilityOfExistence_per;
    input->SRRObjList.aObject[uObj].Qualifiers.uiHighestAssocProb_per =
        extReqPorts->SRRObjList.aObject[uObj].Qualifiers.uiHighestAssocProb_per;
    input->SRRObjList.aObject[uObj].Qualifiers.uiMeasuredTargetFrequency_nu =
        extReqPorts->SRRObjList.aObject[uObj]
            .Qualifiers.uiMeasuredTargetFrequency_nu;

    input->SRRObjList.aObject[uObj].RoadRelation.bDist2BorderValid =
        extReqPorts->SRRObjList.aObject[uObj].RoadRelation.bDist2BorderValid;
    input->SRRObjList.aObject[uObj].RoadRelation.fDist2Border_met =
        extReqPorts->SRRObjList.aObject[uObj].RoadRelation.fDist2Border_met;
    input->SRRObjList.aObject[uObj].RoadRelation.fDist2Course_met =
        extReqPorts->SRRObjList.aObject[uObj].RoadRelation.fDist2Course_met;
  }

  // set Vehicle information input
  input->EgoVehInfo.fegoVelocity_mps = extReqPorts->EgoVehInfo.fegoVelocity_mps;
  input->EgoVehInfo.uBSDGearPosition = extReqPorts->EgoVehInfo.uLBSGearPosition;

  // set Road information input
  input->Road.BorderEstmGridData_fConf_per =
      extReqPorts->Road.BorderEstmGridData_fConf_per;
  input->Road.fConfYOffset_per = extReqPorts->Road.fConfYOffset_per;
  input->Road.fCurveRadius_met = extReqPorts->Road.fCurveRadius_met;
  input->Road.fDrivenCurveRadius_met = extReqPorts->Road.fDrivenCurveRadius_met;
  input->Road.fLaneWidth_met = extReqPorts->Road.fLaneWidth_met;
  input->Road.fYOffsetFused_met = extReqPorts->Road.fYOffsetFused_met;
  input->Road.iNumOfAdjacentLanes_nu = extReqPorts->Road.iNumOfAdjacentLanes_nu;
  input->Road.fConfYOppOffset_per = extReqPorts->Road.fConfYOppOffset_per;
  input->Road.fYOffsetFusedOppBorder_met =
      extReqPorts->Road.fYOffsetFusedOppBorder_met;

  // set LBS and LCA object information input
  for (uint8 uObj = 0; uObj < LBS_INPUT_OBJECT_NUMBER; uObj++) {
    input->LBSInputInfo.LBSObjInfoList[uObj].fAngle_deg =
        pLBSCalculate->LBSObjInfoList[uObj].fAngle_deg;
    input->LBSInputInfo.LBSObjInfoList[uObj].fAssocProbFiltered =
        pLBSCalculate->LBSObjInfoList[uObj].fAssocProbFiltered;
    input->LBSInputInfo.LBSObjInfoList[uObj].fCycletimeSum_s =
        pLBSCalculate->LBSObjInfoList[uObj].fCycletimeSum_s;
    input->LBSInputInfo.LBSObjInfoList[uObj].fTTC_s =
        pLBSCalculate->LBSObjInfoList[uObj].fTTC_s;
    input->LBSInputInfo.LBSObjInfoList[uObj].fTTCFiltered_s =
        pLBSCalculate->LBSObjInfoList[uObj].fTTCFiltered_s;
    input->LBSInputInfo.LBSObjInfoList[uObj].fUpdateRate_nu =
        pLBSCalculate->LBSObjInfoList[uObj].fUpdateRate_nu;
    input->LBSInputInfo.LBSObjInfoList[uObj].fXMovement_met =
        pLBSCalculate->LBSObjInfoList[uObj].fXMovement_met;
    input->LBSInputInfo.LBSObjInfoList[uObj].fYMovement_met =
        pLBSCalculate->LBSObjInfoList[uObj].fYMovement_met;
    input->LBSInputInfo.LBSObjInfoList[uObj].ObjBorders.fXmax_met =
        pLBSCalculate->LBSObjInfoList[uObj].ObjBorders.fXmax_met;
    input->LBSInputInfo.LBSObjInfoList[uObj].ObjBorders.fXmin_met =
        pLBSCalculate->LBSObjInfoList[uObj].ObjBorders.fXmin_met;
    input->LBSInputInfo.LBSObjInfoList[uObj].ObjBorders.fYmax_met =
        pLBSCalculate->LBSObjInfoList[uObj].ObjBorders.fYmax_met;
    input->LBSInputInfo.LBSObjInfoList[uObj].ObjBorders.fYmin_met =
        pLBSCalculate->LBSObjInfoList[uObj].ObjBorders.fYmin_met;

    input->LBSInputInfo.LCAObjInfoList[uObj].bLCAMirrorFrontObject =
        pLBSCalculate->LCAObjInfoList[uObj].bLCAMirrorFrontObject;
    input->LBSInputInfo.LCAObjInfoList[uObj].bLCAMirrorObject =
        pLBSCalculate->LCAObjInfoList[uObj].bLCAMirrorObject;
    input->LBSInputInfo.LCAObjInfoList[uObj].bLCAWarning =
        pLBSCalculate->LCAObjInfoList[uObj].bLCAWarning;
  }

  // set LBS Global input
  input->LBSInputInfo.LBSGlobalInfo.bInnerSensorDriven =
      pLBSGlobals->bInnerSensorDriven;
  input->LBSInputInfo.LBSGlobalInfo.bInnerSensorSteering =
      pLBSGlobals->bInnerSensorSteering;
  input->LBSInputInfo.LBSGlobalInfo.fSensorOffsetToRear_met =
      pLBSGlobals->fSensorOffsetToRear_met;
  // input->LBSInputInfo.LBSGlobalInfo.fSensorOffsetToSide_met =
  // pLBSGlobals->fSensorOffetToSide_met;

  // set function enable switch input
  input->BSDSystemParam.bBSDFunctionActive =
      extReqPorts->LBSSystemParam.bBSDFunctionActive;
  input->BSDSystemParam.bBSDFunctionOutputActive =
      extReqPorts->LBSSystemParam.bBSDFunctionOutputActive;
  input->BSDSystemParam.fCycletime_s = extReqPorts->LBSSystemParam.fCycletime_s;

  // set parameter input
  // vehicle parameter
  paramInput->BSDVehParameter.fVehicleLength_met =
      extReqParam->LBS_Ks_VehParameter_nu.LBS_Kf_VehicleLength_met;
  paramInput->BSDVehParameter.fVehicleWidth_met =
      extReqParam->LBS_Ks_VehParameter_nu.LBS_Kf_VehicleWidth_met;
  paramInput->BSDVehParameter.fWheelBase_met =
      extReqParam->LBS_Ks_VehParameter_nu.LBS_Kf_WheelBase_met;
  paramInput->BSDVehParameter.fVehCenter2FrontAxis_met =
      extReqParam->LBS_Ks_VehParameter_nu.LBS_Kf_VehCenter2FrontAxis_met;
  paramInput->BSDVehParameter.fVehRear2FrontAxis_met =
      extReqParam->LBS_Ks_VehParameter_nu.LBS_Kf_VehCenter2FrontAxis_met +
      extReqParam->LBS_Ks_VehParameter_nu.LBS_Kf_VehicleLength_met * 0.5F;

  // BSD warning parameter and zone area parameter
  memcpy(&paramInput->BSDWarningParameter,  //NOLINT
         &pBSDParam->LBS_Ks_BSDWarnParameter_nu, sizeof(BSDWarningParameter_t));

  // Sensor mounting parameter
  paramInput->SensorMounting.SensorLeft.fLatPos_met =
      extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu
          .LBS_Kf_LatPos_met;
  paramInput->SensorMounting.SensorLeft.fLongPos_met =
      extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu
          .LBS_Kf_LongPos_met;
  paramInput->SensorMounting.SensorLeft.fLongPosToCoG_met =
      extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu
          .LBS_Kf_LongPosToCoG_met;
  paramInput->SensorMounting.SensorLeft.fOrientation_rad =
      extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu
          .LBS_Kf_Orientation_rad;
  paramInput->SensorMounting.SensorLeft.fPitchAngle_rad =
      extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu
          .LBS_Kf_PitchAngle_rad;
  paramInput->SensorMounting.SensorLeft.fRollAngle_rad =
      extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu
          .LBS_Kf_RollAngle_rad;
  paramInput->SensorMounting.SensorLeft.fVertPos_met =
      extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu
          .LBS_Kf_VertPos_met;
  paramInput->SensorMounting.SensorLeft.fYawAngle_rad =
      extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu
          .LBS_Kf_YawAngle_rad;

  paramInput->SensorMounting.SensorRight.fLatPos_met =
      extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu
          .LBS_Kf_LatPos_met;
  paramInput->SensorMounting.SensorRight.fLongPos_met =
      extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu
          .LBS_Kf_LongPos_met;
  paramInput->SensorMounting.SensorRight.fLongPosToCoG_met =
      extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu
          .LBS_Kf_LongPosToCoG_met;
  paramInput->SensorMounting.SensorRight.fOrientation_rad =
      extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu
          .LBS_Kf_Orientation_rad;
  paramInput->SensorMounting.SensorRight.fPitchAngle_rad =
      extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu
          .LBS_Kf_PitchAngle_rad;
  paramInput->SensorMounting.SensorRight.fRollAngle_rad =
      extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu
          .LBS_Kf_RollAngle_rad;
  paramInput->SensorMounting.SensorRight.fVertPos_met =
      extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu
          .LBS_Kf_VertPos_met;
  paramInput->SensorMounting.SensorRight.fYawAngle_rad =
      extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu
          .LBS_Kf_YawAngle_rad;
}

void BSDOutputWrapper(LBSOutPro_t* extProPorts, LBSDebug_t* extDebugPorts,
                      BSDOutPro_st* pOutput, BSDDebug_t* pDebug,
                      LBSCalculate_st* pLBSCalc) {
  // DEBUG_Print("BSDOutputWrapper\n");
  // BSD debug output
  // memcpy(&extDebugPorts->BSDOutput, pOutput, sizeof(BSDOutPro_st));
  if (sizeof(LBSBSDInfoDebug_t) != sizeof(BSD_Info_t)) {
    DEBUG_Print("BSD BSDOutputWrapper Error!");
    return;
  }
  memcpy(&extDebugPorts->BSDDebug.BSDObjInfo,  //NOLINT
         &pDebug->BSDObjInfo, sizeof(BSD_Info_t) * LBS_INPUT_OBJECT_NUMBER);

  for (uint8 uObj = 0; uObj < LBS_INPUT_OBJECT_NUMBER; uObj++) {
    pLBSCalc->BSDObjInfoList[uObj].fBSDZoneXMin_met =
        pOutput->BSDZoneObjParList[uObj].fZoneXmin_met;
  }
  pLBSCalc->LBSBSDCalc.bBSDWarnActiveLeft =
      pOutput->BSD_Globals.bBSDWarnActiveLeft;
  pLBSCalc->LBSBSDCalc.bBSDWarnActiveRight =
      pOutput->BSD_Globals.bBSDWarnActiveRight;
}

/*****************************************************************************
  LCA FUNCTION PROTOTYPES
*****************************************************************************/
void LBSLCAInputWrapper(const LBSInReq_st* extReqPorts,
                        const LBSParam_st* extReqParam, LCAInReq_st* input,
                        LCAParam_st* paramInput, LBSCalculate_st* pLBSCalc) {
  float32 fDistX_met = 0.F;
  float32 fDistY_met = 0.F;
  // Ego vehicle information
  input->EgoVehInfo.uLCAGearPosition = extReqPorts->EgoVehInfo.uLBSGearPosition;
  input->EgoVehInfo.fegoVelocity_mps = extReqPorts->EgoVehInfo.fegoVelocity_mps;
  input->EgoVehInfo.fegoAcceleration_mps2 =
      extReqPorts->EgoVehInfo.fegoAcceleration_mps2;
  // Road information
  input->Road.fConfAdjacentLanes_per = extReqPorts->Road.fConfAdjacentLanes_per;
  input->Road.fConfOppositeLanes_per = extReqPorts->Road.fConfOppositeLanes_per;
  input->Road.fConfYOffset_per = extReqPorts->Road.fConfYOffset_per;
  input->Road.fConfYOppOffset_per = extReqPorts->Road.fConfYOppOffset_per;
  input->Road.fCurveRadius_met = extReqPorts->Road.fCurveRadius_met;
  input->Road.fDrivenCurveRadius_met = extReqPorts->Road.fDrivenCurveRadius_met;
  input->Road.fYOffsetFused_met = extReqPorts->Road.fYOffsetFused_met;
  input->Road.fYOffsetFusedOppBorder_met =
      extReqPorts->Road.fYOffsetFusedOppBorder_met;
  input->Road.iNumOfAdjacentLanes_nu = extReqPorts->Road.iNumOfAdjacentLanes_nu;
  input->Road.iNumOfOppositeLanes_nu = extReqPorts->Road.iNumOfOppositeLanes_nu;
  // LBS system parameters
  input->LCASystemParam.bLCAFunctionActive =
      extReqPorts->LBSSystemParam.bLCAFunctionActive;
  input->LCASystemParam.bLCAFunctionOutputActive =
      extReqPorts->LBSSystemParam.bLCAFunctionOutputActive;
  input->LCASystemParam.fCycletime_s = extReqPorts->LBSSystemParam.fCycletime_s;

  // Calculate information from LBS layer
  if (pLBSCalc->LBSWarnLastCycle.bBSDWarningLeftLastCycle ||
      pLBSCalc->LBSWarnLastCycle.bBSDWarningRightLastCycle) {
    input->LBSInputInfo.LBSWarningLastCycle.bBSDWarningLastCycle = TRUE;
  }
  input->LBSInputInfo.LBSWarningLastCycle.bLCAWarningLastCycle =
      pLBSCalc->LBSWarnLastCycle.bLCAWarningLastCycle;
  input->LCALBSGlobalInput.fMaxSpeedOverGround_mps =
      pLBSCalc->LBS_Globals.fMaxSpeedOverGround_mps;

  for (uint8 uObjIndex = 0U; uObjIndex < LBS_INPUT_OBJECT_NUMBER; uObjIndex++) {
    // EM general object list information
    input->GenObjList.aObject[uObjIndex].GenObjInfo.bRightSensor =
        extReqPorts->GenObjList.aObject[uObjIndex].bRightSensor;
    if (!input->GenObjList.aObject[uObjIndex].GenObjInfo.bRightSensor) {
      fDistX_met =
          extReqPorts->GenObjList.aObject[uObjIndex].Kinemactic.fDistX_met +
          extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu
              .LBS_Kf_LatPos_met;
      fDistY_met =
          extReqPorts->GenObjList.aObject[uObjIndex].Kinemactic.fDistY_met +
          extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu
              .LBS_Kf_LongPos_met;
    } else {
      fDistX_met =
          extReqPorts->GenObjList.aObject[uObjIndex].Kinemactic.fDistX_met +
          extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu
              .LBS_Kf_LatPos_met;
      fDistY_met =
          extReqPorts->GenObjList.aObject[uObjIndex].Kinemactic.fDistY_met +
          extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu
              .LBS_Kf_LongPos_met;
    }
    input->GenObjList.aObject[uObjIndex].GenObjInfo.fDistX_met = fDistX_met;
    input->GenObjList.aObject[uObjIndex].GenObjInfo.fDistY_met = fDistY_met;
    input->GenObjList.aObject[uObjIndex].GenObjInfo.fVabsX_mps =
        extReqPorts->GenObjList.aObject[uObjIndex].Kinemactic.fVabsX_mps;
    input->GenObjList.aObject[uObjIndex].GenObjInfo.fVrelX_mps =
        extReqPorts->GenObjList.aObject[uObjIndex].Kinemactic.fVrelX_mps;
    input->GenObjList.aObject[uObjIndex].GenObjInfo.fLengthFront_met =
        extReqPorts->GenObjList.aObject[uObjIndex].Geometry.fLengthFront_met;
    input->GenObjList.aObject[uObjIndex].GenObjInfo.fWidthLeft_met =
        extReqPorts->GenObjList.aObject[uObjIndex].Geometry.fWidthLeft_met;
    input->GenObjList.aObject[uObjIndex].GenObjInfo.fWidthRight_met =
        extReqPorts->GenObjList.aObject[uObjIndex].Geometry.fWidthRight_met;
    input->GenObjList.aObject[uObjIndex].GenObjInfo.uiLifeCycles_nu =
        extReqPorts->GenObjList.aObject[uObjIndex].General.uiLifeCycles_nu;
    input->GenObjList.aObject[uObjIndex].GenObjInfo.uiMaintenanceState_nu =
        extReqPorts->GenObjList.aObject[uObjIndex]
            .General.uiMaintenanceState_nu;
    input->GenObjList.aObject[uObjIndex].GenObjInfo.fDist2Border_met =
        extReqPorts->SRRObjList.aObject[uObjIndex]
            .RoadRelation.fDist2Border_met;
    input->GenObjList.aObject[uObjIndex].GenObjInfo.bDist2BorderValid =
        extReqPorts->SRRObjList.aObject[uObjIndex]
            .RoadRelation.bDist2BorderValid;
    input->GenObjList.aObject[uObjIndex].GenObjInfo.fMirrorProb_per =
        1 - extReqPorts->SRRObjList.aObject[uObjIndex]
                .Qualifiers.fProbabilityOfExistence_per;
    // extReqPorts->SRRObjList.aObject[uObjIndex]
    //     .SensorSpecific.fMirrorProb_per;
    input->GenObjList.aObject[uObjIndex].GenObjInfo.fRCS =
        extReqPorts->SRRObjList.aObject[uObjIndex].SensorSpecific.fRCS;
    input->GenObjList.aObject[uObjIndex].GenObjInfo.bObjStable =
        extReqPorts->SRRObjList.aObject[uObjIndex].Qualifiers.bObjStable;
    input->GenObjList.aObject[uObjIndex]
        .GenObjInfo.fProbabilityOfExistence_per =
        extReqPorts->SRRObjList.aObject[uObjIndex]
            .Qualifiers.fProbabilityOfExistence_per;
    input->GenObjList.aObject[uObjIndex].GenObjInfo.uiHighestAssocProb_per =
        extReqPorts->SRRObjList.aObject[uObjIndex]
            .Qualifiers.uiHighestAssocProb_per;
    input->GenObjList.aObject[uObjIndex]
        .GenObjInfo.uiMeasuredTargetFrequency_nu =
        extReqPorts->SRRObjList.aObject[uObjIndex]
            .Qualifiers.uiMeasuredTargetFrequency_nu;
    // LBS SI object information
    input->LBSInputInfo.SIObjInfoList[uObjIndex].eAssociatedLane =
        pLBSCalc->SIObjInfoList[uObjIndex].eAssociatedLane;
    input->LBSInputInfo.SIObjInfoList[uObjIndex].fDistToTraj_met =
        pLBSCalc->SIObjInfoList[uObjIndex].fDistToTraj_met;
    input->LBSInputInfo.SIObjInfoList[uObjIndex].fVrelToTraj_mps =
        pLBSCalc->SIObjInfoList[uObjIndex].fVrelToTraj_mps;
    input->LBSInputInfo.SIObjInfoList[uObjIndex].fObjBracketOverlap_met =
        pLBSCalc->SIObjInfoList[uObjIndex].fObjBracketOverlap_met;
    input->LBSInputInfo.SIObjInfoList[uObjIndex].fTraceBracketLeft_met =
        pLBSCalc->SIObjInfoList[uObjIndex].fTraceBracketLeft_met;
    input->LBSInputInfo.SIObjInfoList[uObjIndex].fTraceBracketRight_met =
        pLBSCalc->SIObjInfoList[uObjIndex].fTraceBracketRight_met;
    // LBS BSD object information
    input->LBSInputInfo.BSDObjInfoList[uObjIndex].fBSDZoneXMin_met =
        pLBSCalc->BSDObjInfoList[uObjIndex].fBSDZoneXMin_met;
    input->LBSInputInfo.LBSObjInfoList[uObjIndex].fUpdateRate_nu =
        pLBSCalc->LBSObjInfoList[uObjIndex].fUpdateRate_nu;
    input->LBSInputInfo.LBSObjInfoList[uObjIndex].fXMovement_met =
        pLBSCalc->LBSObjInfoList[uObjIndex].fXMovement_met;
    input->LBSInputInfo.LBSObjInfoList[uObjIndex].bCreateAdjStableObj =
        pLBSCalc->LBSObjInfoList[uObjIndex].bCreateAdjStableObj;
    input->LBSInputInfo.LBSObjInfoList[uObjIndex].bLowTTCAtStart =
        pLBSCalc->LBSObjInfoList[uObjIndex].bLowTTCAtStart;
    input->LBSInputInfo.LBSObjInfoList[uObjIndex].fTTC_s =
        pLBSCalc->LBSObjInfoList[uObjIndex].fTTC_s;
    input->LBSInputInfo.LBSObjInfoList[uObjIndex].fTTCAccel_mps2 =
        pLBSCalc->LBSObjInfoList[uObjIndex].fTTCAccel_mps2;
    input->LBSInputInfo.LBSObjInfoList[uObjIndex].fTTCFiltered_s =
        pLBSCalc->LBSObjInfoList[uObjIndex].fTTCFiltered_s;
  }

  paramInput->fBridgeWarningTime_s =
      extReqParam->LBS_Ks_LCAParameter_nu.LBS_Kf_LCABridgeWarningTime_s;
  paramInput->fMaxLCACurveRadius_met =
      extReqParam->LBS_Ks_LCAParameter_nu.LBS_Kf_LCAMaxLCACurveRadius_met;
  paramInput->fMaxLCARange_met =
      extReqParam->LBS_Ks_LCAParameter_nu.LBS_Kf_LCAMaxLCARange_met;
  paramInput->fMinTTCHysteresis_s =
      extReqParam->LBS_Ks_LCAParameter_nu.LBS_Kf_LCAMinTTCHysteresis_s;
  paramInput->fTTCThreshHighRelSpeed_s =
      extReqParam->LBS_Ks_LCAParameter_nu.LBS_Kf_LCATTCThreshHighRelSpeed_s;
  paramInput->fTTCThreshLowRelSpeed_s =
      extReqParam->LBS_Ks_LCAParameter_nu.LBS_Kf_LCATTCThreshLowRelSpeed_s;
  paramInput->fTTCThreshMidRelSpeed_s =
      extReqParam->LBS_Ks_LCAParameter_nu.LBS_Kf_LCATTCThreshMidRelSpeed_s;
  paramInput->fTTCThreshold_s =
      extReqParam->LBS_Ks_LCAParameter_nu.LBS_Kf_LCATTCThreshold_s;
  paramInput->fVehicleWidth_met =
      extReqParam->LBS_Ks_LCAParameter_nu.LBS_Kf_LCAVehicleWidth_met;
  paramInput->uLCAWarningDuration =
      extReqParam->LBS_Ks_LCAParameter_nu.LBS_Ku_LCAWarningDuration_nu;
  paramInput->LCAZone.fLCAZoneXMid_met =
      extReqParam->LBS_Ks_LCAParameter_nu.LBS_Ks_LCAZone_nu
          .LBS_Kf_LCAZoneXMid_met;
  paramInput->LCAZone.fLCAZoneXMin_met =
      extReqParam->LBS_Ks_LCAParameter_nu.LBS_Ks_LCAZone_nu
          .LBS_Kf_LCAZoneXMin_met;
  paramInput->LCAZone.fLCAZoneYMaxFar_met =
      extReqParam->LBS_Ks_LCAParameter_nu.LBS_Ks_LCAZone_nu
          .LBS_Kf_LCAZoneYMaxFar_met;
  paramInput->LCAZone.fLCAZoneYMaxNear_met =
      extReqParam->LBS_Ks_LCAParameter_nu.LBS_Ks_LCAZone_nu
          .LBS_Kf_LCAZoneYMaxNear_met;
  paramInput->LCAZone.fLCAZoneYMinFar_met =
      extReqParam->LBS_Ks_LCAParameter_nu.LBS_Ks_LCAZone_nu
          .LBS_Kf_LCAZoneYMinFar_met;
  paramInput->LCAZone.fLCAZoneYMinNear_met =
      extReqParam->LBS_Ks_LCAParameter_nu.LBS_Ks_LCAZone_nu
          .LBS_Kf_LCAZoneYMinNear_met;
}

void LBSLCAOutputWrapper(LCAOutPro_st* pOutput, LCADebug_t* pDebug,
                         LBSCalculate_st* pLBSCalc, LBSDebug_t* extDebugPorts) {
  pLBSCalc->LBSLCACalc.bLCAWarnActive = pOutput->bLCAWarnActive;
  pLBSCalc->LBSLCACalc.bLCAWarnActiveLeft = pOutput->bLCAWarnActiveLeft;
  pLBSCalc->LBSLCACalc.bLCAWarnActiveRight = pOutput->bLCAWarnActiveRight;

  pLBSCalc->LBSLCACalc.uLCAWarningID_nu = pOutput->uLCAWarningID_nu;
  pLBSCalc->LBSLCACalc.fXObjectWarning_met = pOutput->fXObjectWarning_met;
  pLBSCalc->LBSLCACalc.fCriticalTTC_s = pOutput->fCriticalTTC_s;
  pLBSCalc->LBSWarnLastCycle.bLCAWarningLastCycle =
      pOutput->bLCAWarningLastCycle;

  for (uint8 uObj = 0U; uObj < LBS_INPUT_OBJECT_NUMBER; uObj++) {
    pLBSCalc->LCAObjInfoList[uObj].bLCAMirrorFrontObject =
        pOutput->LCAObjOutputList[uObj].bLCAMirrorFrontObject;
    pLBSCalc->LCAObjInfoList[uObj].bLCAMirrorObject =
        pOutput->LCAObjOutputList[uObj].bLCAMirrorObject;
    pLBSCalc->LCAObjInfoList[uObj].bLCAWarning =
        pOutput->LCAObjOutputList[uObj].bLCAWarning;
  }
  // debug for assigning value
  memcpy(extDebugPorts->LCADebug.LCAObjOutputList,  //NOLINT
         pOutput->LCAObjOutputList, sizeof(LBSLCAObjInfoArrayDebug));
  memcpy(&extDebugPorts->LCADebug.LCAWarnInfo, &pDebug->LCAWarnInfo,  //NOLINT
         sizeof(LCAWarnInfo_t));
  memcpy(&extDebugPorts->LCADebug.LCAConfig, &pDebug->LCAConfig,  //NOLINT
         sizeof(LCAConfig_t));
  memcpy(&extDebugPorts->LCADebug.LCAFrontMirror,  //NOLINT
         &pDebug->LCAFrontMirror, sizeof(LCAFrontMirror_t));
  extDebugPorts->LCADebug.LCAGlobal.fLCARange = pDebug->fLCARange;
  extDebugPorts->LCADebug.LCAGlobal.uCntLCAPathBlockedLeft =
      pDebug->uCntLCAPathBlockedLeft;
  extDebugPorts->LCADebug.LCAGlobal.uCntLCAPathBlockedRight =
      pDebug->uCntLCAPathBlockedRight;
  extDebugPorts->LCADebug.LCAGlobal.bLCAPathBlockedLeft =
      pDebug->bLCAPathBlockedLeft;
  extDebugPorts->LCADebug.LCAGlobal.bLCAPathBlockedRight =
      pDebug->bLCAPathBlockedRight;
}

// typedef struct {
//   uint32 uiVersionNumber;
//   uint32 uLCAWarningID_nu;
//   // LCAGlobals
//   float32 fLCARange;
//   uint8 uCntLCAPathBlockedLeft;   // Counter of left LCA path blocked
//   uint8 uCntLCAPathBlockedRight;  // Counter of right LCA path blocked
//   boolean bLCAPathBlockedLeft;    // Flag whether left adjacent lane is no
//   more boolean bLCAPathBlockedRight;   // Flag whether Right adjacent lane is
//   no more LCAObjInfo_Array LCAObjOutputList;
// } LCADebug_t;

/*****************************************************************************
  OSE FUNCTION PROTOTYPES
*****************************************************************************/
void LBSOSEInputWrapper(const LBSInReq_st* extReqPorts,
                        const LBSParam_st* extReqParam, OSEInReq_t* input,
                        OSEParam_t* paramInput, LBSCalculate_st* pLBSCalc) {
  // extReqPorts->input

  input->bDoorOpenLeft = extReqPorts->EgoVehInfo.bLeftDoorOpen;
  input->bDoorOpenRight = extReqPorts->EgoVehInfo.bRightDoorOpen;
  for (uint8 uObj = 0U; uObj < LBS_INPUT_OBJECT_NUMBER; uObj++) {
    // EM input information
    input->EMGenObjList.aObject[uObj].bRightSensor =
        extReqPorts->GenObjList.aObject[uObj].bRightSensor;
    input->EMGenObjList.aObject[uObj].eMaintenanceState =
        extReqPorts->GenObjList.aObject[uObj].General.uiMaintenanceState_nu;
    input->EMGenObjList.aObject[uObj].fAbsOrientationStd_rad =
        extReqPorts->GenObjList.aObject[uObj].Geometry.fAbsOrientationStd_rad;
    input->EMGenObjList.aObject[uObj].fAbsOrientation_rad =
        extReqPorts->GenObjList.aObject[uObj].Geometry.fAbsOrientation_rad;
    input->EMGenObjList.aObject[uObj].fDistXStd_met =
        extReqPorts->GenObjList.aObject[uObj].Kinemactic.fDistXStd_met;
    input->EMGenObjList.aObject[uObj].fDistX_met =
        extReqPorts->GenObjList.aObject[uObj].Kinemactic.fDistX_met;
    input->EMGenObjList.aObject[uObj].fDistYStd_met =
        extReqPorts->GenObjList.aObject[uObj].Kinemactic.fDistYStd_met;
    input->EMGenObjList.aObject[uObj].fDistY_met =
        extReqPorts->GenObjList.aObject[uObj].Kinemactic.fDistY_met;
    input->EMGenObjList.aObject[uObj].fFirstDetectX_met =
        extReqPorts->SRRObjList.aObject[uObj].History.fFirstDetectX_met;
    input->EMGenObjList.aObject[uObj].fLengthFront_met =
        extReqPorts->GenObjList.aObject[uObj].Geometry.fLengthFront_met;
    input->EMGenObjList.aObject[uObj].fLengthRear_met =
        extReqPorts->GenObjList.aObject[uObj].Geometry.fLengthRear_met;
    input->EMGenObjList.aObject[uObj].fMirrorProb_per =
        1 - extReqPorts->SRRObjList.aObject[uObj]
                .Qualifiers.fProbabilityOfExistence_per;
    // extReqPorts->SRRObjList.aObject[uObj].SensorSpecific.fMirrorProb_per;
    input->EMGenObjList.aObject[uObj].fProbabilityOfExistence_per =
        extReqPorts->SRRObjList.aObject[uObj]
            .Qualifiers.fProbabilityOfExistence_per;
    input->EMGenObjList.aObject[uObj].fRCS =
        extReqPorts->SRRObjList.aObject[uObj].SensorSpecific.fRCS;
    input->EMGenObjList.aObject[uObj].fVabsX_mps =
        extReqPorts->GenObjList.aObject[uObj].Kinemactic.fVabsX_mps;
    input->EMGenObjList.aObject[uObj].fVabsY_mps =
        extReqPorts->GenObjList.aObject[uObj].Kinemactic.fVabsY_mps;
    input->EMGenObjList.aObject[uObj].fVrelX_mps =
        extReqPorts->GenObjList.aObject[uObj].Kinemactic.fVrelX_mps;
    input->EMGenObjList.aObject[uObj].fVrelY_mps =
        extReqPorts->GenObjList.aObject[uObj].Kinemactic.fVrelY_mps;
    input->EMGenObjList.aObject[uObj].fWidthLeft_met =
        extReqPorts->GenObjList.aObject[uObj].Geometry.fWidthLeft_met;
    input->EMGenObjList.aObject[uObj].fWidthRight_met =
        extReqPorts->GenObjList.aObject[uObj].Geometry.fWidthRight_met;
    input->EMGenObjList.aObject[uObj].fWidth_met =
        extReqPorts->GenObjList.aObject[uObj].Geometry.fWidth_met;
    input->EMGenObjList.aObject[uObj].uiLifeCycles_nu =
        extReqPorts->GenObjList.aObject[uObj].General.uiLifeCycles_nu;
    input->EMGenObjList.aObject[uObj].uiMeasuredTargetFrequency_nu =
        extReqPorts->SRRObjList.aObject[uObj]
            .Qualifiers.uiMeasuredTargetFrequency_nu;
    // LBS object information
    input->OSELBSGlobalInReq.OSELBSObjInfoArray[uObj].fAssocProbFiltered =
        pLBSCalc->LBSObjInfoList[uObj].fAssocProbFiltered;
    input->OSELBSGlobalInReq.OSELBSObjInfoArray[uObj].fUpdateRate_nu =
        pLBSCalc->LBSObjInfoList[uObj].fUpdateRate_nu;
    input->OSELBSGlobalInReq.OSELBSObjInfoArray[uObj].fXMovement_met =
        pLBSCalc->LBSObjInfoList[uObj].fXMovement_met;
    input->OSELBSGlobalInReq.OSELBSObjInfoArray[uObj].fYMovement_met =
        pLBSCalc->LBSObjInfoList[uObj].fYMovement_met;
    input->OSELBSGlobalInReq.OSELBSObjInfoArray[uObj].ObjBorders.fXmax_met =
        pLBSCalc->LBSObjInfoList[uObj].ObjBorders.fXmax_met;
    input->OSELBSGlobalInReq.OSELBSObjInfoArray[uObj].ObjBorders.fXmin_met =
        pLBSCalc->LBSObjInfoList[uObj].ObjBorders.fXmin_met;
    input->OSELBSGlobalInReq.OSELBSObjInfoArray[uObj].ObjBorders.fYmax_met =
        pLBSCalc->LBSObjInfoList[uObj].ObjBorders.fYmax_met;
    input->OSELBSGlobalInReq.OSELBSObjInfoArray[uObj].ObjBorders.fYmin_met =
        pLBSCalc->LBSObjInfoList[uObj].ObjBorders.fYmin_met;
  }
  input->fCycletime_s = extReqPorts->LBSSystemParam.fCycletime_s;
  input->OSEFunctionSwitch.bOSEFunctionActive =
      extReqPorts->LBSSystemParam.bOSEFunctionActive;

  // extReqParam->paramInput
  paramInput->bActive = extReqParam->LBS_Ks_OSEParameter_nu.LBS_Kb_OSEActive_nu;
  // paramInput->bEnableObjAdaptiveBreakthrough =
  // extReqParam->LBSOSEParameter.bEnableObjAdaptiveBreakthrough;
  paramInput->fMaxHeadingAngle =
      extReqParam->LBS_Ks_OSEParameter_nu.LBS_Kf_OSEMaxHeadingAngle_deg;
  paramInput->fMaxTime_s[0] =
      extReqParam->LBS_Ks_OSEParameter_nu.LBS_Ka_OSEMaxTime_s[0];
  paramInput->fMaxTime_s[1] =
      extReqParam->LBS_Ks_OSEParameter_nu.LBS_Ka_OSEMaxTime_s[1];
  paramInput->fMaxTime_s[2] =
      extReqParam->LBS_Ks_OSEParameter_nu.LBS_Ka_OSEMaxTime_s[2];
  paramInput->fMinHeadingAngle =
      extReqParam->LBS_Ks_OSEParameter_nu.LBS_Kf_OSEMinHeadingAngle_deg;
  paramInput->fMinTime_s[0] =
      extReqParam->LBS_Ks_OSEParameter_nu.LBS_Ka_OSEMinTime_s[0];
  paramInput->fMinTime_s[1] =
      extReqParam->LBS_Ks_OSEParameter_nu.LBS_Ka_OSEMinTime_s[1];
  paramInput->fMinTime_s[2] =
      extReqParam->LBS_Ks_OSEParameter_nu.LBS_Ka_OSEMinTime_s[2];
  //   paramInput->fTargetRangeMax_met[0] =
  //       extReqParam->LBS_Ks_OSEParameter_nu.LBS_Ka_OSETargetRangeMax_met[0];
  //   paramInput->fTargetRangeMax_met[1] =
  //       extReqParam->LBS_Ks_OSEParameter_nu.LBS_Ka_OSETargetRangeMax_met[1];
  //   paramInput->fTargetRangeMax_met[2] =
  //       extReqParam->LBS_Ks_OSEParameter_nu.LBS_Ka_OSETargetRangeMax_met[2];
  paramInput->fXMaxTargetRange_met =
      extReqParam->LBS_Ks_OSEParameter_nu.LBS_Ka_OSEXMaxTargetRange_met;
  paramInput->fXMinTargetRange_met =
      extReqParam->LBS_Ks_OSEParameter_nu.LBS_Ka_OSEXMinTargetRange_met;
  paramInput->fTTCThreshold_s[0] =
      extReqParam->LBS_Ks_OSEParameter_nu.LBS_Ka_OSETTCThreshold_s[0];
  paramInput->fTTCThreshold_s[1] =
      extReqParam->LBS_Ks_OSEParameter_nu.LBS_Ka_OSETTCThreshold_s[1];
  paramInput->fTTCThreshold_s[2] =
      extReqParam->LBS_Ks_OSEParameter_nu.LBS_Ka_OSETTCThreshold_s[2];
  paramInput->fVEgoMax_mps =
      extReqParam->LBS_Ks_OSEParameter_nu.LBS_Kf_OSEVEgoMax_mps;
  paramInput->fVEgoMin_mps =
      extReqParam->LBS_Ks_OSEParameter_nu.LBS_Kf_OSEVEgoMin_mps;
  paramInput->fVTargetMax_mps =
      extReqParam->LBS_Ks_OSEParameter_nu.LBS_Kf_OSEVTargetMax_mps;
  paramInput->fVTargetMin_mps =
      extReqParam->LBS_Ks_OSEParameter_nu.LBS_Kf_OSEVTargetMin_mps;
  paramInput->fXBreakthroughLine_met[0] =
      extReqParam->LBS_Ks_OSEParameter_nu.LBS_Ka_OSEXBreakthroughLine_met[0];
  paramInput->fXBreakthroughLine_met[1] =
      extReqParam->LBS_Ks_OSEParameter_nu.LBS_Ka_OSEXBreakthroughLine_met[1];
  paramInput->fYMaxBreakthroughMargin_met[0] =
      extReqParam->LBS_Ks_OSEParameter_nu
          .LBS_Ka_OSEYMaxBreakthroughMargin_met[0];
  paramInput->fYMaxBreakthroughMargin_met[1] =
      extReqParam->LBS_Ks_OSEParameter_nu
          .LBS_Ka_OSEYMaxBreakthroughMargin_met[1];
  paramInput->fYMaxBreakthrough_met[0] =
      extReqParam->LBS_Ks_OSEParameter_nu.LBS_Ka_OSEYMaxBreakthrough_met[0];
  paramInput->fYMaxBreakthrough_met[1] =
      extReqParam->LBS_Ks_OSEParameter_nu.LBS_Ka_OSEYMaxBreakthrough_met[1];
  paramInput->fYMinBreakthroughMargin_met[0] =
      extReqParam->LBS_Ks_OSEParameter_nu
          .LBS_Ka_OSEYMinBreakthroughMargin_met[0];
  paramInput->fYMinBreakthroughMargin_met[1] =
      extReqParam->LBS_Ks_OSEParameter_nu
          .LBS_Ka_OSEYMinBreakthroughMargin_met[1];
  paramInput->fYMinBreakthrough_met[0] =
      extReqParam->LBS_Ks_OSEParameter_nu.LBS_Ka_OSEYMinBreakthrough_met[0];
  paramInput->fYMinBreakthrough_met[1] =
      extReqParam->LBS_Ks_OSEParameter_nu.LBS_Ka_OSEYMinBreakthrough_met[1];
  paramInput->SensorMounting.SensorLeft.fLatPos_met =
      extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu
          .LBS_Kf_LatPos_met;
  paramInput->SensorMounting.SensorLeft.fLongPos_met =
      extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu
          .LBS_Kf_LongPos_met;
  paramInput->SensorMounting.SensorRight.fLatPos_met =
      extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu
          .LBS_Kf_LatPos_met;
  paramInput->SensorMounting.SensorRight.fLongPos_met =
      extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu
          .LBS_Kf_LongPos_met;
  paramInput->VehParAdd.fOverhangFront_met =
      extReqParam->LBS_Ks_VehParameter_nu.LBS_Kf_OverhangFront_met;
  paramInput->VehParAdd.fVehicleLength_met =
      extReqParam->LBS_Ks_VehParameter_nu.LBS_Kf_VehicleLength_met;
  paramInput->VehParAdd.fVehicleWidth_met =
      extReqParam->LBS_Ks_VehParameter_nu.LBS_Kf_VehicleWidth_met;
}

void LBSOSEOutputWrapper(OSEOutPro_t* pOutput, OSEDebug_t* OSEDebugInfo,
                         LBSCalculate_st* pLBSCalc, LBSDebug_t* debugInfo) {
  // pOutput -> pLBSCalc
  for (uint8 uWarnLevel = 0U; uWarnLevel < LBS_OSE_NUM_OF_WARN_LEVELS;
       uWarnLevel++) {
    pLBSCalc->LBSOSECalc.bOSEWarnActive[uWarnLevel] =
        pOutput->bOSEWarnActive[uWarnLevel];
  }
  pLBSCalc->LBSOSECalc.bWarningInterrupt = pOutput->bWarningInterrupt;
  pLBSCalc->LBSOSECalc.fCriticalTTC = pOutput->fCriticalTTC;
  pLBSCalc->LBSOSECalc.uCriticalObjID = pOutput->uCriticalObjID;

  if (sizeof(LBSOSEObjInfo_t) != sizeof(OSEObjInfoArrayOutPro_t)) {
    DEBUG_Print("OSE LBSOSEOutputWrapper Error 1!");
    return;
  }

  if (sizeof(LBSOSEObjInfoArrayDebug_t) != sizeof(OSEObjInfoArrayOutPro_t)) {
    DEBUG_Print("OSE LBSOSEOutputWrapper Error 2!");
    return;
  }
  memcpy(pLBSCalc->OSEObjInfoList, pOutput->OSEObjInfoArray,  //NOLINT
         sizeof(OSEObjInfoArrayOutPro_t) * LBS_INPUT_OBJECT_NUMBER);

  // pOutput->OSEDebug
  memcpy(debugInfo->OSEDebug.OSEObjInfoArray,  //NOLINT
         pOutput->OSEObjInfoArray,
         sizeof(OSEObjInfoArrayOutPro_t) * LBS_INPUT_OBJECT_NUMBER);
}

/*****************************************************************************
  RCW FUNCTION PROTOTYPES
*****************************************************************************/
void RCWInputWrapper(const LBSInReq_st* extReqPorts, RCWInReq_st* input,
                     const LBSParam_st* extReqParam, RCWParam_st* paramInput,
                     LBSCalculate_st* pLBSCalc) {
  // RCWInReq_st
  input->EgoVehInfo.fRCWEgo_steer_angle_rad =
      extReqPorts->EgoVehInfo.fSelfSteering_rad;
  input->EgoVehInfo.uRCWGearPosition = extReqPorts->EgoVehInfo.uLBSGearPosition;
  const LBS_Globals_t* pLBSGlobals = pGeLBSCalculatePointer_LBSGlobals();
  for (uint8 uObj = 0U; uObj < LBS_INPUT_OBJECT_NUMBER; uObj++) {
    input->GenObjList.aObject[uObj].bRightSensor =
        extReqPorts->GenObjList.aObject[uObj].bRightSensor;
    input->GenObjList.aObject[uObj].Kinemactic.fDistX_met =
        extReqPorts->GenObjList.aObject[uObj].Kinemactic.fDistX_met;
    input->GenObjList.aObject[uObj].Kinemactic.fDistY_met =
        extReqPorts->GenObjList.aObject[uObj].Kinemactic.fDistY_met;
    input->GenObjList.aObject[uObj].Kinemactic.fVrelX_mps =
        extReqPorts->GenObjList.aObject[uObj].Kinemactic.fVrelX_mps;
    input->GenObjList.aObject[uObj].Kinemactic.fVrelY_mps =
        extReqPorts->GenObjList.aObject[uObj].Kinemactic.fVrelY_mps;
    input->GenObjList.aObject[uObj].Kinemactic.fArelX_mpss =
        extReqPorts->GenObjList.aObject[uObj].Kinemactic.fArelX_mpss;
    input->GenObjList.aObject[uObj].Kinemactic.fArelY_mpss =
        extReqPorts->GenObjList.aObject[uObj].Kinemactic.fArelY_mpss;
    input->GenObjList.aObject[uObj].Kinemactic.fVabsX_mps =
        extReqPorts->GenObjList.aObject[uObj].Kinemactic.fVabsX_mps;
    input->GenObjList.aObject[uObj].Kinemactic.fVabsY_mps =
        extReqPorts->GenObjList.aObject[uObj].Kinemactic.fVabsY_mps;
    input->GenObjList.aObject[uObj].Geometry.fWidth_met =
        extReqPorts->GenObjList.aObject[uObj].Geometry.fWidth_met;
    input->GenObjList.aObject[uObj].Geometry.fWidthLeft_met =
        extReqPorts->GenObjList.aObject[uObj].Geometry.fWidthLeft_met;
    input->GenObjList.aObject[uObj].Geometry.fWidthRight_met =
        extReqPorts->GenObjList.aObject[uObj].Geometry.fWidthRight_met;
    input->GenObjList.aObject[uObj].Geometry.fLength_met =
        extReqPorts->GenObjList.aObject[uObj].Geometry.fLength_met;
    input->GenObjList.aObject[uObj].Geometry.fLengthFront_met =
        extReqPorts->GenObjList.aObject[uObj].Geometry.fLengthFront_met;
    input->GenObjList.aObject[uObj].Geometry.fLengthRear_met =
        extReqPorts->GenObjList.aObject[uObj].Geometry.fLengthRear_met;
    input->GenObjList.aObject[uObj].Geometry.fAbsOrientation_rad =
        extReqPorts->GenObjList.aObject[uObj].Geometry.fAbsOrientation_rad;
    input->GenObjList.aObject[uObj].General.fLifeTime_s =
        extReqPorts->GenObjList.aObject[uObj].General.fLifeTime_s;
    input->GenObjList.aObject[uObj].General.uiLifeCycles_nu =
        extReqPorts->GenObjList.aObject[uObj].General.uiLifeCycles_nu;
    input->GenObjList.aObject[uObj].General.uiMaintenanceState_nu =
        extReqPorts->GenObjList.aObject[uObj].General.uiMaintenanceState_nu;
    input->GenObjList.aObject[uObj].General.uiID_nu =
        extReqPorts->GenObjList.aObject[uObj].General.uiID_nu;
    input->GenObjList.aObject[uObj].Attributes.eDynamicProperty_nu =
        extReqPorts->GenObjList.aObject[uObj].Attributes.eDynamicProperty_nu;
    input->GenObjList.aObject[uObj].Attributes.uiDynConfidence_per =
        extReqPorts->GenObjList.aObject[uObj].Attributes.uiDynConfidence_per;
    input->GenObjList.aObject[uObj].Attributes.eClassification_nu =
        extReqPorts->GenObjList.aObject[uObj].Attributes.eClassification_nu;
    input->GenObjList.aObject[uObj].Attributes.uiClassConfidence_per =
        extReqPorts->GenObjList.aObject[uObj].Attributes.uiClassConfidence_per;
    input->GenObjList.aObject[uObj].Qualifiers.fProbabilityOfExistence_per =
        extReqPorts->SRRObjList.aObject[uObj]
            .Qualifiers.fProbabilityOfExistence_per;
    input->GenObjList.aObject[uObj].Qualifiers.uiHighestAssocProb_per =
        extReqPorts->SRRObjList.aObject[uObj].Qualifiers.uiHighestAssocProb_per;
    input->GenObjList.aObject[uObj].Qualifiers.uiMeasuredTargetFrequency_nu =
        extReqPorts->SRRObjList.aObject[uObj]
            .Qualifiers.uiMeasuredTargetFrequency_nu;
    input->GenObjList.aObject[uObj].Qualifiers.bObjStable =
        extReqPorts->SRRObjList.aObject[uObj].Qualifiers.bObjStable;
    input->GenObjList.aObject[uObj].RoadRelation.fDist2Course_met =
        pLBSCalc->RoadRelation[uObj].fDist2Course_met;
    // extReqPorts->SRRObjList.aObject[uObj].RoadRelation.fDist2Course_met;

    input->LBSInputInfo.LBSObjInfoList[uObj].ObjBorders.fXmin_met =
        pLBSCalc->LBSObjInfoList[uObj].ObjBorders.fXmin_met;
    input->LBSInputInfo.LBSObjInfoList[uObj].ObjBorders.fXmax_met =
        pLBSCalc->LBSObjInfoList[uObj].ObjBorders.fXmax_met;
    input->LBSInputInfo.LBSObjInfoList[uObj].ObjBorders.fYmin_met =
        pLBSCalc->LBSObjInfoList[uObj].ObjBorders.fYmin_met;
    input->LBSInputInfo.LBSObjInfoList[uObj].ObjBorders.fYmax_met =
        pLBSCalc->LBSObjInfoList[uObj].ObjBorders.fYmax_met;
    input->LBSInputInfo.LBSObjInfoList[uObj].fTTC_s =
        pLBSCalc->LBSObjInfoList[uObj].fTTC_s;
    input->LBSInputInfo.LBSObjInfoList[uObj].fTTCAccel_mps2 =
        pLBSCalc->LBSObjInfoList[uObj].fTTCAccel_mps2;
    input->LBSInputInfo.LBSObjInfoList[uObj].fCycletimeSum_s =
        pLBSCalc->LBSObjInfoList[uObj].fCycletimeSum_s;
    input->LBSInputInfo.LBSObjInfoList[uObj].fUpdateRate_nu =
        pLBSCalc->LBSObjInfoList[uObj].fUpdateRate_nu;
    input->LBSInputInfo.LBSObjInfoList[uObj].fXMovement_met =
        pLBSCalc->LBSObjInfoList[uObj].fXMovement_met;
    input->LBSInputInfo.LBSObjInfoList[uObj].fYMovement_met =
        pLBSCalc->LBSObjInfoList[uObj].fYMovement_met;
    input->LBSInputInfo.LBSObjInfoList[uObj].fAngle_deg =
        pLBSCalc->LBSObjInfoList[uObj].fAngle_deg;
    input->LBSInputInfo.LBSObjInfoList[uObj].fAssocProbFiltered =
        pLBSCalc->LBSObjInfoList[uObj].fAssocProbFiltered;
    input->LBSInputInfo.LBSObjInfoList[uObj].bLowTTCAtStart =
        pLBSCalc->LBSObjInfoList[uObj].bLowTTCAtStart;
    input->LBSInputInfo.LBSObjInfoList[uObj].bCreateAdjStableObj =
        pLBSCalc->LBSObjInfoList[uObj].bCreateAdjStableObj;
    input->LBSInputInfo.LBSObjInfoList[uObj].eAssociatedLane =
        pLBSCalc->SIObjInfoList[uObj].eAssociatedLane;
    input->LBSInputInfo.LBSObjInfoList[uObj].fDistToTraj_met =
        pLBSCalc->SIObjInfoList[uObj].fDistToTraj_met;
    input->LBSInputInfo.LBSObjInfoList[uObj].fVrelToTraj_mps =
        pLBSCalc->SIObjInfoList[uObj].fVrelToTraj_mps;
    input->LBSInputInfo.LCAObjInfoList[uObj].bLCAMirrorObject =
        pLBSCalc->LCAObjInfoList[uObj].bLCAMirrorObject;
    input->LBSInputInfo.LCAObjInfoList[uObj].bLCAMirrorFrontObject =
        pLBSCalc->LCAObjInfoList[uObj].bLCAMirrorFrontObject;
    input->LBSInputInfo.LCAObjInfoList[uObj].bLCAWarning =
        pLBSCalc->LCAObjInfoList[uObj].bLCAWarning;
    input->LBSInputInfo.BSDObjInfoList[uObj].bBSDWarning =
        FALSE;  // pLBSCalc->BSDObjInfoList[uObj].bBSDWarning; todo
  }
  input->EgoVehInfo.fegoAcceleration_mps2 =
      extReqPorts->EgoVehInfo.fegoAcceleration_mps2;
  input->EgoVehInfo.fegoVelocity_mps = extReqPorts->EgoVehInfo.fegoVelocity_mps;
  input->EgoVehInfo.fLatAccel_mps2 = extReqPorts->EgoVehInfo.fLatAccel_mps2;
  input->EgoVehInfo.fLatVelocity_mps = 0.F;

  input->RCWSystemSwitch.bRCWFunctionActive =
      extReqPorts->LBSSystemParam.bRCWFunctionActive;
  input->RCWSystemSwitch.bRCWFunctionOutputActive =
      extReqPorts->LBSSystemParam.bRCWFunctionOutputActive;
  input->RCWSystemSwitch.bRCWFunctionNVMActive =
      extReqPorts->LBS_Ns_NVRAM_nu.LBS_Nb_RCWPowerOffSwitchState_nu;

  input->LBSInputInfo.LBSGlobalInfo.fSensorOffsetToRear_met =
      pLBSGlobals->fSensorOffsetToRear_met;
  input->LBSInputInfo.LBSGlobalInfo.fSensorOffsetToSide_met =
      pLBSGlobals->fSensorOffetToSide_met;
  input->RCWPreProcessInput.RCWHmiOpen =
      extReqPorts->LBSSystemParam.bRCWFunctionActive;
  input->RCWPreProcessInput.RCWFailure =
      extReqPorts->LBSSupportInfo.LBSRCWFailure;
  input->RCWPreProcessInput.LeftTurnLightOpen =
      extReqPorts->EgoVehInfo.LBSLeftTurnLightOpen;
  input->RCWPreProcessInput.RightTurnLightOpen =
      extReqPorts->EgoVehInfo.LBSRightTurnLightOpen;
  // input->RCWPreProcessInput.BlockingTimeActive = FALSE; inplement in the
  // postproces

  // RCWParam_st
  paramInput->RCWVehParameter.fVehicleWidth_met =
      extReqParam->LBS_Ks_VehParameter_nu.LBS_Kf_VehicleWidth_met;
  paramInput->RCWVehParameter.fVehicleLength_met =
      extReqParam->LBS_Ks_VehParameter_nu.LBS_Kf_VehicleLength_met;
  paramInput->RCWVehParameter.fVehCenter2FrontAxis_met =
      extReqParam->LBS_Ks_VehParameter_nu.LBS_Kf_VehCenter2FrontAxis_met;

  paramInput->SensorMounting.SensorLeft.fLatPos_met =
      extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu
          .LBS_Kf_LatPos_met;
  paramInput->SensorMounting.SensorLeft.fLongPos_met =
      extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu
          .LBS_Kf_LongPos_met;
  paramInput->SensorMounting.SensorLeft.fVertPos_met =
      extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu
          .LBS_Kf_VertPos_met;
  paramInput->SensorMounting.SensorLeft.fLongPosToCoG_met =
      extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu
          .LBS_Kf_LongPosToCoG_met;
  paramInput->SensorMounting.SensorLeft.fPitchAngle_rad =
      extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu
          .LBS_Kf_PitchAngle_rad;
  paramInput->SensorMounting.SensorLeft.fOrientation_rad =
      extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu
          .LBS_Kf_Orientation_rad;
  paramInput->SensorMounting.SensorLeft.fYawAngle_rad =
      extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu
          .LBS_Kf_YawAngle_rad;
  paramInput->SensorMounting.SensorRight.fLatPos_met =
      extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu
          .LBS_Kf_LatPos_met;
  paramInput->SensorMounting.SensorRight.fLongPos_met =
      extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu
          .LBS_Kf_LongPos_met;
  paramInput->SensorMounting.SensorRight.fVertPos_met =
      extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu
          .LBS_Kf_VertPos_met;
  paramInput->SensorMounting.SensorRight.fLongPosToCoG_met =
      extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu
          .LBS_Kf_LongPosToCoG_met;
  paramInput->SensorMounting.SensorRight.fPitchAngle_rad =
      extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu
          .LBS_Kf_PitchAngle_rad;
  paramInput->SensorMounting.SensorRight.fOrientation_rad =
      extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu
          .LBS_Kf_Orientation_rad;
  paramInput->SensorMounting.SensorRight.fYawAngle_rad =
      extReqParam->LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu
          .LBS_Kf_YawAngle_rad;
  memcpy(&paramInput->RCWWarningParameter,  //NOLINT
         &extReqParam->LBS_Ks_RCWParameter_nu, sizeof(RCWWarningParameter_t));

  //   SenseTime_Memcpy(&extReqParam->LBS_Ks_RCWParameter_nu,
  //                    &paramInput->RCWWarningParameter,
  //                    sizeof(RCWWarningParameter_t));
}

void RCWOutputWrapper(LBSOutPro_t* extProPorts, LBSDebug_t* extDebugPorts,
                      RCWOutPro_st* pOutput, RCWDebug_t* pDebug,
                      LBSCalculate_st* pLBSCalc) {
  extProPorts->LBSFunState.uRCWWarning = pOutput->uHmiRCWWarningActive;
  extProPorts->LBSFunState.fTTC_s = pOutput->rcw_fTTC_s;
  extProPorts->LBS_Ns_NVRAM_nu.LBS_Nb_RCWPowerOffSwitchState_nu =
      pOutput->bHmiRCWHmiOn;

  if (sizeof(RCWDebug_t) != sizeof(LBSRCWDebug_t)) {
    DEBUG_Print("RCW RCWOutputWrapper Error!");
    return;
  }
  memcpy(&extDebugPorts->RCWDebug, pDebug,  //NOLINT
         sizeof(RCWDebug_t));
}