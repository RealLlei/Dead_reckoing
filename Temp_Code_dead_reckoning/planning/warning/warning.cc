/******************************************************************************
 * Copyright 2022 The TL Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/warning/warning.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>

#include "common/math/math_utils.h"
#include "common/util/message_util.h"
#include "common/util/util.h"
#include "planning/warning/common/warning_debug_info.h"
#include "planning/warning/common/warning_util.h"
#include "planning/warning/lbs/common/tue_common_libs.h"

namespace TL {
namespace planning {
namespace warning {

using TL::common::ErrorCode;
using TL::common::Status;
using TL::localization::Localization;
using TL::perception::PerceptionObstacles;
using TL::soc::DoorStatus;

// #def
Status Warning::Init() {
  obs_extended_info_.resize(WARNING_OBS_MAX_NUMBERS);
  for (int i = 0; i < WARNING_OBS_MAX_NUMBERS; ++i) {
    obs_extended_info_[i].id = -1;
  }
  InitLBSInput();
  SigHeaderInfo();
  InitCTAInput();
  LBS_Reset();
  return Status::OK();
}

Status Warning::InitLBSInput() {
  // LBS memset
  memset(&(reqPorts), 0, sizeof(LBSInReq_st));
  memset(&(params), 0, sizeof(LBSParam_st));
  memset(&(proPorts), 0, sizeof(LBSOutPro_t));
  memset(&(debugInfo), 0, sizeof(LBSDebug_t));
  memset(&(obs_move_cycle_arr), 0, sizeof(FCTAObjCycle_t));

  // SI memset
  memset(&(SIReqPorts), 0, sizeof(SIInPut_st));
  memset(&(SIProPorts), 0, sizeof(SIOutPut_st));
  memset(&(SIParams), 0, sizeof(SIParam_st));
  memset(&(SIDebugInfo), 0, sizeof(SIDebug_st));

  memset(&(LBSCalculate), 0, sizeof(LBSCalculate_st));
  memset(&(RCWCalculate), 0, sizeof(RCWCalculate_st));
  // LBS system parameter

  // sensor mounting parameter all set as 0.0
  params.LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu.LBS_Kf_LatPos_met =
      LBS_SensorLeft_LatPos;
  params.LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu.LBS_Kf_LongPos_met =
      LBS_SensorLeft_LongPos;
  params.LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu.LBS_Kf_VertPos_met =
      LBS_SensorLeft_VertPos;
  params.LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu.LBS_Kf_LongPosToCoG_met =
      LBS_SensorLeft_LongPosToCoG;
  params.LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu.LBS_Kf_PitchAngle_rad =
      LBS_SensorLeft_PitchAngle;
  params.LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu.LBS_Kf_Orientation_rad =
      LBS_SensorLeft_Orientation;
  params.LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu.LBS_Kf_RollAngle_rad =
      LBS_SensorLeft_RollAngle_rad;
  params.LBS_Ks_SensorMounting_nu.LBS_Kf_SensorLeft_nu.LBS_Kf_YawAngle_rad =
      LBS_SensorLeft_YawAngle;

  params.LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu.LBS_Kf_LatPos_met =
      LBS_SensorRight_LatPos;
  params.LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu.LBS_Kf_LongPos_met =
      LBS_SensorRight_LongPos;
  params.LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu.LBS_Kf_VertPos_met =
      LBS_SensorRight_VertPos;
  params.LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu
      .LBS_Kf_LongPosToCoG_met = LBS_SensorRight_LongPosToCoG;
  params.LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu.LBS_Kf_PitchAngle_rad =
      LBS_SensorRight_PitchAngle;
  params.LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu.LBS_Kf_Orientation_rad =
      LBS_SensorRight_Orientation;
  params.LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu.LBS_Kf_RollAngle_rad =
      LBS_SensorRight_RollAngle_rad;
  params.LBS_Ks_SensorMounting_nu.LBS_Kf_SensorRight_nu.LBS_Kf_YawAngle_rad =
      LBS_SensorRight_YawAngle;

  // RCW parameter
  // LBS_Ks_RCWParameter_nu:use sensetime recommended value
  params.LBS_Ks_RCWParameter_nu.fEgoVelStationaryCheck = 0.1;

  params.LBS_Ks_RCWParameter_nu.fTTCThresholdLevel1 = 2.5;  // no LEVEL_1
  params.LBS_Ks_RCWParameter_nu.fTTCThresholdLevel2 = 1.5;
  params.LBS_Ks_RCWParameter_nu.fSuppressEgoLongVelMin = 4.17;  // 15km/h
  params.LBS_Ks_RCWParameter_nu.fSuppressEgoLongVelMax = 38.88;
  params.LBS_Ks_RCWParameter_nu.fSuppressEgoLatVelMax = 0.1;
  // no suggested value
  params.LBS_Ks_RCWParameter_nu.fSuppressEgoLatAcceMax = 0.1;
  // no suggested value
  params.LBS_Ks_RCWParameter_nu.fSuppressRCWBlockingTime =
      2.0;  // 2000.0 ->2 0803
  params.LBS_Ks_RCWParameter_nu.fCorrBlockedActiveOccMin = 0.15;
  params.LBS_Ks_RCWParameter_nu.fCorrBlockedInActiveOccMin = 0.30;
  params.LBS_Ks_RCWParameter_nu.fOppoOccuAddAdjust = 0.35;
  params.LBS_Ks_RCWParameter_nu.fWarnDecisionVrel2TrajMax = 1.0;
  params.LBS_Ks_RCWParameter_nu.fWarnDecisionAssocProbMin = 0.8;

  params.LBS_Ks_RCWParameter_nu.fWarnDecisionTTCAdjustThreshold =
      1.4;  // 1.4-> 4.4
  params.LBS_Ks_RCWParameter_nu.fWarnDecisionArelXMaxTTCAdjust = -0.5;
  params.LBS_Ks_RCWParameter_nu.fWarnDecisionArelXMinTTCAdjust = -1.5;
  params.LBS_Ks_RCWParameter_nu.fWarnDecisionArelXTTCReduceFactor = 0.1;

  params.LBS_Ks_RCWParameter_nu.fReleventDist2CourseMin = -0.5;
  params.LBS_Ks_RCWParameter_nu.fReleventMaxXOppSideRelevent = -5.0;
  params.LBS_Ks_RCWParameter_nu.fReleventMinXOppSideRelevent = -15.0;
  params.LBS_Ks_RCWParameter_nu.fReleventLifeCnt =
      LBS_RCW_RELEVANCE_LIFECYCLE_MIN;                     // 50 -> 2
  params.LBS_Ks_RCWParameter_nu.fReleventMaxDist = -25.0;  // production
  params.LBS_Ks_RCWParameter_nu.fReleventMinXMovement = 5.0;

  params.LBS_Ks_RCWParameter_nu.fCorridorMinTimeInCorridorThresh = 0.5;
  params.LBS_Ks_RCWParameter_nu.uCorridorHitCntThresh = 0U;  // 15-> 5 JO
  params.LBS_Ks_RCWParameter_nu.fCycletime_s = 0.1;
  params.LBS_Ks_RCWParameter_nu.fMinTimeInCorridorThresh = 0.05;
  // 0.5 -> 0.05
  params.LBS_Ks_RCWParameter_nu.fCorrHitCntSmallObjWidth = 2.5;
  params.LBS_Ks_RCWParameter_nu.fRCWCorrMinSDISIX = -50.0;
  params.LBS_Ks_RCWParameter_nu.fRCWCorrMaxSDISIX = 0.0;
  params.LBS_Ks_RCWParameter_nu.fRCWCorridorXMid = -20.0;
  params.LBS_Ks_RCWParameter_nu.fRCWCorridorXMin = -40.0;
  params.LBS_Ks_RCWParameter_nu.fVrelTTCMin = 0;  // m/s 10km/h -> 0 production
  params.LBS_Ks_RCWParameter_nu.fVrelTTCMax = 8.333;      // m/s 30km/h
  params.LBS_Ks_RCWParameter_nu.fTTCThreshVrelMin = 0;    // 2 -> 0
  params.LBS_Ks_RCWParameter_nu.fTTCThreshVrelMax = 1.5;  // 2.5 -> 1.5
  params.LBS_Ks_RCWParameter_nu.fMinHeadingAngle = DEG2RAD(50.0);
  params.LBS_Ks_RCWParameter_nu.fMaxHeadingAngle = DEG2RAD(60.0);
  params.LBS_Ks_RCWParameter_nu.fCorrOccWarnEnable = 0.3;
  params.LBS_Ks_RCWParameter_nu.fCorrOccDropThresh = 0.1;
  params.LBS_Ks_RCWParameter_nu.fCorrOccPickupThresh = 0.28;
  params.LBS_Ks_RCWParameter_nu.fCorrOccHitThresh = 0.2;

  // Vehicle parameter
  params.LBS_Ks_VehParameter_nu.LBS_Kf_WheelBase_met =
      LBS_INPUT_VEHICLE_WHEEL_BASE;
  params.LBS_Ks_VehParameter_nu.LBS_Kf_VehicleLength_met =
      LBS_INPUT_VEHICLE_LENGTH;
  params.LBS_Ks_VehParameter_nu.LBS_Kf_VehicleWidth_met =
      LBS_INPUT_VEHICLE_WIDTH;
  params.LBS_Ks_VehParameter_nu.LBS_Kf_VehCenter2FrontAxis_met =
      0.5 * LBS_INPUT_VEHICLE_WHEEL_BASE;
  params.LBS_Ks_VehParameter_nu.LBS_Kf_OverhangFront_met =
      LBS_INPUT_VEHICLE_OVERHANG_FRONT;

  // LBS Parameter
  params.LBS_Ks_LBSParameter_nu.LBS_Kb_NCAPActive_nu = TRUE;
  params.LBS_Ks_LBSParameter_nu.LBS_Kf_SoTDelayThresh_s =
      LBS_INPUT_SOT_DELAY_THRESHOLD;  // no use
  params.LBS_Ks_LBSParameter_nu.LBS_Kf_SoTMinWarnDuration_s =
      LBS_INPUT_SOT_MIN_WARN_DURATION;
  params.LBS_Ks_LBSParameter_nu.LBS_Kf_SoTCutoffSpeed_mps =
      LBS_INPUT_SOT_CUTOFF_SPEED;

  // BSD Param

  params.LBS_Ks_BSDParameter_nu.LBS_Ks_BSDWarnParameter_nu
      .LBS_Kb_BSDNCAPActive_nu = FALSE;
  params.LBS_Ks_BSDParameter_nu.LBS_Ks_BSDWarnParameter_nu
      .LBS_Kf_BSDSoTDelayThresh_s = 2.0F;
  params.LBS_Ks_BSDParameter_nu.LBS_Ks_BSDWarnParameter_nu
      .LBS_Kf_BSDSoTMinWarnDuration_s =
      LBS_INPUT_SOT_MIN_WARN_DURATION;  // 2.0f s;
  params.LBS_Ks_BSDParameter_nu.LBS_Ks_BSDWarnParameter_nu
      .LBS_Kf_BSDSoTCutoffSpeed_mps = LBS_INPUT_SOT_CUTOFF_SPEED;
  params.LBS_Ks_BSDParameter_nu.LBS_Ks_BSDWarnParameter_nu
      .LBS_Kf_BSDVelMinWarnDisable_mps = LBS_INPUT_BSD_EN_DISABLE_VELOCITY;
  params.LBS_Ks_BSDParameter_nu.LBS_Ks_BSDWarnParameter_nu
      .LBS_Kf_BSDVelMinWarnEnable_mps = LBS_INPUT_BSD_EN_ENABLE_VELOCITY;

  params.LBS_Ks_BSDParameter_nu.LBS_Ks_BSDZoneParameter_nu.LBS_Kf_BSDXmin_met =
      LBS_INPUT_BSD_ZONE_MIN_X;  // -5.925
  params.LBS_Ks_BSDParameter_nu.LBS_Ks_BSDZoneParameter_nu.LBS_Kf_BSDXmax_met =
      LBS_INPUT_BSD_ZONE_MAX_X;  // -0.961
  params.LBS_Ks_BSDParameter_nu.LBS_Ks_BSDZoneParameter_nu.LBS_Kf_BSDYmin_met =
      LBS_INPUT_BSD_ZONE_MIN_Y;  // 1.555
  params.LBS_Ks_BSDParameter_nu.LBS_Ks_BSDZoneParameter_nu.LBS_Kf_BSDYmax_met =
      LBS_INPUT_BSD_ZONE_MAX_Y;  // 4.74
  params.LBS_Ks_BSDParameter_nu.LBS_Ks_BSDZoneParameter_nu
      .LBS_Kf_BSDHysteresisX_met = LBS_INPUT_BSD_ZONE_HYST;
  params.LBS_Ks_BSDParameter_nu.LBS_Ks_BSDZoneParameter_nu
      .LBS_Kf_BSDHysteresisYmin_met = LBS_INPUT_BSD_ZONE_HYST;
  params.LBS_Ks_BSDParameter_nu.LBS_Ks_BSDZoneParameter_nu
      .LBS_Kf_BSDHysteresisYmax_met = LBS_INPUT_BSD_ZONE_HYST;

  // LCA Parameter

  params.LBS_Ks_LCAParameter_nu.LBS_Kf_LCAVehicleWidth_met =
      params.LBS_Ks_VehParameter_nu.LBS_Kf_VehicleWidth_met;
  params.LBS_Ks_LCAParameter_nu.LBS_Kf_LCAVoWnMinWarnEnable_mps =
      LBS_LCA_WarnEnbale_MinVel;
  params.LBS_Ks_LCAParameter_nu.LBS_Kf_LCAVownMinWarnDisable_mps =
      LBS_LCA_WarnDisenbale_MinVel;
  params.LBS_Ks_LCAParameter_nu.LBS_Kf_LCATTCThreshold_s =
      LBS_LCA_DIS_TTCsTHRESHOLD;
  params.LBS_Ks_LCAParameter_nu.LBS_Kf_LCATTCThreshLowRelSpeed_s =
      LBS_LCA_TTCsTHRESHOLD_LOWSPD;
  params.LBS_Ks_LCAParameter_nu.LBS_Kf_LCATTCThreshMidRelSpeed_s =
      LBS_LCA_TTCsTHRESHOLD_MIDSPD;
  params.LBS_Ks_LCAParameter_nu.LBS_Kf_LCATTCThreshHighRelSpeed_s =
      LBS_LCA_TTCsTHRESHOLD_HIGSPD;
  params.LBS_Ks_LCAParameter_nu.LBS_Kf_LCAMinTTCHysteresis_s =
      LBS_LCA_TTCsTHRESHOLD_HYS;
  params.LBS_Ks_LCAParameter_nu.LBS_Kf_LCABridgeWarningTime_s = 0.1F;
  params.LBS_Ks_LCAParameter_nu.LBS_Kf_LCAMaxLCARange_met =
      120.0;  // SET 0  NO USE
  params.LBS_Ks_LCAParameter_nu.LBS_Kf_LCAMaxLCACurveRadius_met =
      250.0;  // SET 0  NO USE;
  params.LBS_Ks_LCAParameter_nu.LBS_Ku_LCAWarningDuration_nu =
      LBS_LCA_TTC_BaseVel;
  params.LBS_Ks_LCAParameter_nu.LBS_Ks_LCAZone_nu.LBS_Kf_LCAZoneXMid_met =
      LBS_INPUT_LCA_Zone_MID_X;  //-40
  params.LBS_Ks_LCAParameter_nu.LBS_Ks_LCAZone_nu.LBS_Kf_LCAZoneXMin_met =
      LBS_INPUT_LCA_Zone_MIN_X;  //-60
  params.LBS_Ks_LCAParameter_nu.LBS_Ks_LCAZone_nu.LBS_Kf_LCAZoneYMinNear_met =
      LBS_INPUT_LCA_Zone_MIN_NEAR_Y;  //0.6
  params.LBS_Ks_LCAParameter_nu.LBS_Ks_LCAZone_nu.LBS_Kf_LCAZoneYMinFar_met =
      LBS_INPUT_LCA_Zone_MIN_FAR_Y;  //1.5
  params.LBS_Ks_LCAParameter_nu.LBS_Ks_LCAZone_nu.LBS_Kf_LCAZoneYMaxNear_met =
      LBS_INPUT_LCA_Zone_MAX_NEAR_Y;  //4.74
  params.LBS_Ks_LCAParameter_nu.LBS_Ks_LCAZone_nu.LBS_Kf_LCAZoneYMaxFar_met =
      LBS_INPUT_LCA_Zone_MAX_FAR_Y;  //4.74

  params.LBS_Ks_OSEParameter_nu.LBS_Ka_OSETTCThreshold_s[0] = 3.0;
  params.LBS_Ks_OSEParameter_nu.LBS_Ka_OSETTCThreshold_s[1] = 3.0;
  params.LBS_Ks_OSEParameter_nu.LBS_Ka_OSEYMinBreakthrough_met[0] = 0.5;
  params.LBS_Ks_OSEParameter_nu.LBS_Ka_OSEYMinBreakthrough_met[1] = 0.5;
  params.LBS_Ks_OSEParameter_nu.LBS_Ka_OSEYMaxBreakthrough_met[0] = 3.5;
  params.LBS_Ks_OSEParameter_nu.LBS_Ka_OSEYMaxBreakthrough_met[1] = 3.5;
  params.LBS_Ks_OSEParameter_nu.LBS_Ka_OSEYMinBreakthroughMargin_met[0] = 0;
  params.LBS_Ks_OSEParameter_nu.LBS_Ka_OSEYMinBreakthroughMargin_met[1] = 0;
  params.LBS_Ks_OSEParameter_nu.LBS_Ka_OSEYMaxBreakthroughMargin_met[0] = 0;
  params.LBS_Ks_OSEParameter_nu.LBS_Ka_OSEYMaxBreakthroughMargin_met[1] = 0;
  params.LBS_Ks_OSEParameter_nu.LBS_Ka_OSEXBreakthroughLine_met[0] =
      0;  // production
  params.LBS_Ks_OSEParameter_nu.LBS_Ka_OSEXBreakthroughLine_met[1] =
      0;  // production
  params.LBS_Ks_OSEParameter_nu.LBS_Kf_OSEVEgoMax_mps = 0.5;
  params.LBS_Ks_OSEParameter_nu.LBS_Kf_OSEVEgoMin_mps = -0.1;  // production
  params.LBS_Ks_OSEParameter_nu.LBS_Kf_OSEVTargetMin_mps = 3 / 3.6;
  params.LBS_Ks_OSEParameter_nu.LBS_Kf_OSEVTargetMax_mps = 40.0;

  params.LBS_Ks_OSEParameter_nu.LBS_Ka_OSEMinTime_s[0] =
      1.0;  //730 production 3--> 1
  params.LBS_Ks_OSEParameter_nu.LBS_Ka_OSEMinTime_s[1] =
      1.0;  //730 production 3--> 1
  params.LBS_Ks_OSEParameter_nu.LBS_Ka_OSEMinTime_s[2] =
      1.0;  //730 production 3--> 1
  params.LBS_Ks_OSEParameter_nu.LBS_Ka_OSEMaxTime_s[0] = 10.0;
  params.LBS_Ks_OSEParameter_nu.LBS_Ka_OSEMaxTime_s[1] = 10.0;
  params.LBS_Ks_OSEParameter_nu.LBS_Ka_OSEMaxTime_s[2] = 10.0;
  params.LBS_Ks_OSEParameter_nu.LBS_Kf_OSEMaxHeadingAngle_deg = 45.0;
  params.LBS_Ks_OSEParameter_nu.LBS_Kf_OSEMinHeadingAngle_deg = -45.0;

  //   params.LBS_Ks_OSEParameter_nu.LBS_Ka_OSETargetRangeMax_met[0] =
  //       42.0;
  //   params.LBS_Ks_OSEParameter_nu.LBS_Ka_OSETargetRangeMax_met[1] = 30.0;
  //   params.LBS_Ks_OSEParameter_nu.LBS_Ka_OSETargetRangeMax_met[2] = 20.0;
  params.LBS_Ks_OSEParameter_nu.LBS_Ka_OSEXMaxTargetRange_met =
      60 + LBS_INPUT_VEHICLE_OVERHANG_FRONT;  // *product
  params.LBS_Ks_OSEParameter_nu.LBS_Ka_OSEXMinTargetRange_met =
      -60 - (LBS_INPUT_VEHICLE_LENGTH - LBS_INPUT_VEHICLE_OVERHANG_FRONT);
  params.LBS_Ks_OSEParameter_nu.LBS_Kb_OSEEnableObjAdaptiveBreakthrough_nu =
      TRUE;
  params.LBS_Ks_OSEParameter_nu.LBS_Kb_OSEActive_nu = TRUE;

  // LBS System Parameter
  reqPorts.LBSSystemParam.bBSDFunctionActive = FALSE;
  reqPorts.LBSSystemParam.bBSDFunctionOutputActive = FALSE;
  reqPorts.LBSSystemParam.bLCAFunctionActive = FALSE;
  reqPorts.LBSSystemParam.bLCAFunctionOutputActive = FALSE;
  reqPorts.LBSSystemParam.bOSEFunctionActive = FALSE;
  reqPorts.LBSSystemParam.bOSEFunctionOutputActive = FALSE;
  reqPorts.LBSSystemParam.fCycletime_s = 0.1;

  proPorts.LBSFunState.bBSDWarning = false;
  proPorts.LBSFunState.bBSDWarningLeft = false;
  proPorts.LBSFunState.bBSDWarningRight = false;

  proPorts.LBSFunState.bLCAWarning = false;
  proPorts.LBSFunState.bLCAWarningLeft = false;
  proPorts.LBSFunState.bLCAWarningRight = false;

  proPorts.LBSFunState.bOSEWarning.bPreWarnActive = false;
  proPorts.LBSFunState.bOSEWarning.bAcuteWarnActive = false;
  proPorts.LBSFunState.bOSEWarning.bDoorLockingActive = false;

  proPorts.LBSFunState.bOSEWarningLeft.bPreWarnActive = false;
  proPorts.LBSFunState.bOSEWarningLeft.bAcuteWarnActive = false;
  proPorts.LBSFunState.bOSEWarningLeft.bDoorLockingActive = false;

  proPorts.LBSFunState.bOSEWarningRight.bPreWarnActive = false;
  proPorts.LBSFunState.bOSEWarningRight.bAcuteWarnActive = false;
  proPorts.LBSFunState.bOSEWarningRight.bDoorLockingActive = false;

  proPorts.LBSFunState.fTTC_s = TUE_C_F32_VALUE_INVALID;

  reqPorts.LBSSystemParam.bRCWFunctionActive = FALSE;
  reqPorts.LBSSystemParam.bRCWFunctionOutputActive = FALSE;

  return Status::OK();
}

Status Warning::InitCTAInput() {
  // CTA interface memset  // FCTA --JO
  memset(&(CTA_reqPorts), 0, sizeof(CTAInReq_t));
  memset(&(CTA_params), 0, sizeof(CTAParam_t));
  memset(&(CTA_proPorts), 0, sizeof(CTAOutPro_t));
  memset(&(CTA_debugInfo), 0, sizeof(CTADebug_t));

  // CTA CTA_reqPorts
  CTA_reqPorts.fCycleTime_s = 0.1;
  CTA_reqPorts.bFCTAFunctionActive = FALSE;
  CTA_reqPorts.bFCTAFunctionOutputActive = FALSE;
  CTA_reqPorts.bRCTAFunctionActive = FALSE;
  CTA_reqPorts.bRCTAFunctionOutputActive = FALSE;
  CTA_reqPorts.CTA_Ns_NVRAM_nu.CTA_Nb_FCTAPowerOffSwitchState_nu =
      true;  // no use
  CTA_reqPorts.CTA_Ns_NVRAM_nu.CTA_Nb_RCTAPowerOffSwitchState_nu =
      true;  // no use

  // CTA FCTA_params
  CTA_params.CTA_Ks_FCTAAlgoParameter_nu.CTA_Kf_TTCThreshold_s =
      1.5;  // NO Level 1
  CTA_params.CTA_Ks_FCTAAlgoParameter_nu.CTA_Kf_TTCThresholdL2_s = 1.5;
  CTA_params.CTA_Ks_FCTAAlgoParameter_nu.CTA_Kf_XMinBreakthrough_met =
      LBS_INPUT_VEHICLE_OVERHANG_FRONT + -1.0;  // 0-> 3
  CTA_params.CTA_Ks_FCTAAlgoParameter_nu.CTA_Kf_XMinBreakthroughL2_met =
      LBS_INPUT_VEHICLE_OVERHANG_FRONT + -1.0;  // 0 -> 2
  CTA_params.CTA_Ks_FCTAAlgoParameter_nu.CTA_Kf_XMaxBreakthrough_met =
      LBS_INPUT_VEHICLE_OVERHANG_FRONT + 3.5;  // 7 -> 9
  CTA_params.CTA_Ks_FCTAAlgoParameter_nu.CTA_Kf_VEgoMax_mps = 30 / 3.6F;
  CTA_params.CTA_Ks_FCTAAlgoParameter_nu.CTA_Kf_VEgoMin_mps =
      0.1 / 3.6F;  // -0.1 -> 3 -> 0.1 functionspec
  CTA_params.CTA_Ks_FCTAAlgoParameter_nu.CTA_Kf_VTargetMin_mps =
      6 / 3.6F;  // 0.83 --> 1.39 3km/h --> 5km/h -->6km/h functionspc
  CTA_params.CTA_Ks_FCTAAlgoParameter_nu.CTA_Kf_VTargetMax_mps =
      20.0;  // 300 --> 20 no use
  CTA_params.CTA_Ks_FCTAAlgoParameter_nu.CTA_Kf_MaxHeadingAngle_deg =
      120.0;  // 135 -> 120; functionspc
  CTA_params.CTA_Ks_FCTAAlgoParameter_nu.CTA_Kf_MinHeadingAngle_deg =
      60.0;  // 35 -> 60 functionspc
  CTA_params.CTA_Ks_FCTAAlgoParameter_nu.CTA_Kf_TargetRangeMax_met = 71.0;
  CTA_params.CTA_Ks_FCTAAlgoParameter_nu.CTA_Kf_TargetRangeMaxL2_met = 51.0;
  CTA_params.CTA_Ks_FCTAAlgoParameter_nu.CTA_Kf_BreakthroughMargin_met =
      2.0;  // 1 --> 2 0628
  CTA_params.CTA_Ks_FCTAAlgoParameter_nu.CTA_Kf_TTCThresholdMargin_s =
      0.0;  // 1 --> 2 0628
  CTA_params.CTA_Ks_FCTAAlgoParameter_nu.CTA_Kb_Active_nu = TRUE;

  // CTA RCTA_params
  CTA_params.CTA_Ks_RCTAAlgoParameter_nu.CTA_Kf_TTCThreshold_s =
      1.5;  //no level 1
  CTA_params.CTA_Ks_RCTAAlgoParameter_nu.CTA_Kf_TTCThresholdL2_s = 1.5;
  CTA_params.CTA_Ks_RCTAAlgoParameter_nu.CTA_Kf_TTCThresholdMargin_s = 0.0;
  CTA_params.CTA_Ks_RCTAAlgoParameter_nu.CTA_Kf_XMinBreakthrough_met =
      -(LBS_INPUT_VEHICLE_LENGTH - LBS_INPUT_VEHICLE_OVERHANG_FRONT) -
      3.5;  // -7.5
  CTA_params.CTA_Ks_RCTAAlgoParameter_nu.CTA_Kf_XMinBreakthroughL2_met =
      -(LBS_INPUT_VEHICLE_LENGTH - LBS_INPUT_VEHICLE_OVERHANG_FRONT) -
      3.5;  //-7.5
  CTA_params.CTA_Ks_RCTAAlgoParameter_nu.CTA_Kf_XMaxBreakthrough_met =
      -(LBS_INPUT_VEHICLE_LENGTH - LBS_INPUT_VEHICLE_OVERHANG_FRONT) +
      1.0;  // -3
  CTA_params.CTA_Ks_RCTAAlgoParameter_nu.CTA_Kf_VEgoMax_mps = 0.199;
  CTA_params.CTA_Ks_RCTAAlgoParameter_nu.CTA_Kf_VEgoMin_mps = -3.70;
  CTA_params.CTA_Ks_RCTAAlgoParameter_nu.CTA_Kf_VTargetMin_mps = 0.83;
  CTA_params.CTA_Ks_RCTAAlgoParameter_nu.CTA_Kf_MaxHeadingAngle_deg = -60.0;
  CTA_params.CTA_Ks_RCTAAlgoParameter_nu.CTA_Kf_MinHeadingAngle_deg = -120.0;
  CTA_params.CTA_Ks_RCTAAlgoParameter_nu.CTA_Kf_TargetRangeMax_met = 71.0;
  CTA_params.CTA_Ks_RCTAAlgoParameter_nu.CTA_Kf_TargetRangeMaxL2_met = 51.0;
  CTA_params.CTA_Ks_RCTAAlgoParameter_nu.CTA_Kb_Active_nu = TRUE;
  CTA_params.CTA_Ks_RCTAAlgoParameter_nu.CTA_Ks_SteeringAngleCutOff_nu
      .fSteerAngleCutOffMin_deg = 300.0;
  CTA_params.CTA_Ks_RCTAAlgoParameter_nu.CTA_Ks_SteeringAngleCutOff_nu
      .fSteerAngleCutOffMid_deg = 400.0;
  CTA_params.CTA_Ks_RCTAAlgoParameter_nu.CTA_Ks_SteeringAngleCutOff_nu
      .fSteerAngleCutOffMax_deg = 500.0;
  CTA_params.CTA_Ks_RCTAAlgoParameter_nu.CTA_Ks_SteeringAngleCutOff_nu
      .fXMinBreakthroughSWAMin_met =
      -(LBS_INPUT_VEHICLE_LENGTH - LBS_INPUT_VEHICLE_OVERHANG_FRONT) - 3.5;
  CTA_params.CTA_Ks_RCTAAlgoParameter_nu.CTA_Ks_SteeringAngleCutOff_nu
      .fXMinBreakthroughSWAMid_met =
      -(LBS_INPUT_VEHICLE_LENGTH - LBS_INPUT_VEHICLE_OVERHANG_FRONT) - 2.5;
  CTA_params.CTA_Ks_RCTAAlgoParameter_nu.CTA_Ks_SteeringAngleCutOff_nu
      .fXMinBreakthroughSWAMax_met =
      -(LBS_INPUT_VEHICLE_LENGTH - LBS_INPUT_VEHICLE_OVERHANG_FRONT) - 1.5;
  CTA_params.CTA_Ks_RCTAAlgoParameter_nu.CTA_Ks_SteeringAngleCutOff_nu
      .bEnableSteeringAngleCutOff_nu = TRUE;

  // CTA CTAVehicleParam_t
  CTA_params.CTA_Ks_VehicleParameter_nu.CTA_Kf_WheelBase_met =
      LBS_INPUT_VEHICLE_WHEEL_BASE;
  CTA_params.CTA_Ks_VehicleParameter_nu.CTA_Kf_VehicleWidth_met =
      LBS_INPUT_VEHICLE_WIDTH;
  CTA_params.CTA_Ks_VehicleParameter_nu.CTA_Kf_VehicleLength_met =
      LBS_INPUT_VEHICLE_LENGTH;
  CTA_params.CTA_Ks_VehicleParameter_nu.CTA_Kf_OverhangFront_met =
      LBS_INPUT_VEHICLE_OVERHANG_FRONT;

  CTA_params.CTA_Ks_VehicleParameter_nu.CTA_Ks_LeftFrontSensorMounting
      .CTA_Kf_LatPos_met = 0;
  CTA_params.CTA_Ks_VehicleParameter_nu.CTA_Ks_LeftFrontSensorMounting
      .CTA_Kf_LongPos_met = 0;
  CTA_params.CTA_Ks_VehicleParameter_nu.CTA_Ks_RightFrontSensorMounting
      .CTA_Kf_LatPos_met = 0;
  CTA_params.CTA_Ks_VehicleParameter_nu.CTA_Ks_RightFrontSensorMounting
      .CTA_Kf_LongPos_met = 0;
  CTA_params.CTA_Ks_VehicleParameter_nu.CTA_Ks_LeftRearSensorMounting_nu
      .CTA_Kf_LatPos_met = 0;
  CTA_params.CTA_Ks_VehicleParameter_nu.CTA_Ks_LeftRearSensorMounting_nu
      .CTA_Kf_LongPos_met = 0;
  CTA_params.CTA_Ks_VehicleParameter_nu.CTA_Ks_RightRearSensorMounting_nu
      .CTA_Kf_LatPos_met = 0;
  CTA_params.CTA_Ks_VehicleParameter_nu.CTA_Ks_RightRearSensorMounting_nu
      .CTA_Kf_LongPos_met = 0;
  return Status::OK();
}

TL::common::Status Warning::Start() {
  return Status::OK();
}

struct CompareByValue {
  bool operator()(const std::pair<uint32, double>& left,
                  const std::pair<uint32, double>& right) {
    return left.second < right.second;
  }
};

bool Warning::CalculateExtendedObsInfo(
    const std::shared_ptr<const TL::perception::PerceptionObstacles>&
        perception_obstacles,
    const std::shared_ptr<const localization::Localization>& localization) {
  std::unordered_map<int, int> obs_id_index_map;
  std::unordered_set<int> creating_obs_id_set;
  std::vector<std::pair<int, double>> obs_proto_index_distance_vector;
  obs_id_index_map.reserve(perception_obstacles->perception_obstacle_size());
  creating_obs_id_set.reserve(perception_obstacles->perception_obstacle_size());
  obs_proto_index_distance_vector.resize(
      perception_obstacles->perception_obstacle_size());
  // 障碍物重新排序,按与自车距离
  for (int i = 0; i < perception_obstacles->perception_obstacle_size(); ++i) {
    auto& obs = perception_obstacles->perception_obstacle(i);
    double d = 1000.0;
    if (obs.has_position_flu()) {
      d = (obs.position_flu().x() * obs.position_flu().x() +
           obs.position_flu().y() * obs.position_flu().y());
    } else if (localization != nullptr) {
      double ego_x = localization->pose().position().x();
      double ego_y = localization->pose().position().y();
      d = ((obs.position().x() - ego_x) * (obs.position().x() - ego_x) +
           (obs.position().y() - ego_y) * (obs.position().y() - ego_y));
    } else {
      return false;
    }
    obs_proto_index_distance_vector[i] = {i, d};
  }
  std::sort(obs_proto_index_distance_vector.begin(),
            obs_proto_index_distance_vector.end(), CompareByValue());
  // 当前帧的障碍物id放到map和set中，map中存proto中对应的index
  for (int i = 0; i < obs_proto_index_distance_vector.size(); ++i) {
    if (i >= WARNING_OBS_MAX_NUMBERS) {
      break;
    }
    auto obs_proto_index = obs_proto_index_distance_vector[i].first;
    auto obs_id =
        perception_obstacles->perception_obstacle(obs_proto_index).id();
    obs_id_index_map[obs_id] = obs_proto_index;
    creating_obs_id_set.insert(obs_id);
  }
  // 对障碍物重新排序与0-80的index关联，保证前后两帧id相同的障碍物在相同的位置
  // 第一步先找到当前帧继续出现的障碍物，并更新对应的proto index
  // 计算障碍物出现的cycle
  // 然后将0-80位置中障碍物消失的位置标记
  auto current_sequence = perception_obstacles->header().seq();
  for (int i = 0; i < WARNING_OBS_MAX_NUMBERS; ++i) {
    auto obs_id = obs_extended_info_[i].id;
    if (obs_id_index_map.find(obs_id) != obs_id_index_map.end()) {
      obs_extended_info_[i].index_in_proto = obs_id_index_map[obs_id];
      creating_obs_id_set.erase(obs_id);
      obs_extended_info_[i].life_cycle =
          current_sequence - obs_first_history_sequence_[obs_id];
    } else {
      obs_extended_info_[i].index_in_proto = -1;
      obs_first_history_sequence_.erase(obs_id);
    }
  }
  // 第二步，将这一帧新出现的障碍物，按顺序填到上一步标记的障碍物消失的位置
  int i = 0;
  if (creating_obs_id_set.size() > 0) {
    auto iter_set = creating_obs_id_set.begin();
    while (iter_set != creating_obs_id_set.end()) {
      for (; i < WARNING_OBS_MAX_NUMBERS; i++) {
        if (obs_extended_info_[i].index_in_proto == -1) {
          obs_extended_info_[i].id = *iter_set;
          obs_extended_info_[i].index_in_proto = obs_id_index_map[*iter_set];
          obs_extended_info_[i].life_cycle = 1;
          obs_first_history_sequence_[*iter_set] = current_sequence - 1;
          break;
        }
      }
      ++iter_set;
    }
  }
  // 第三步，将0-80后面没用上的标记为已消失位置的信息清空
  for (; i < WARNING_OBS_MAX_NUMBERS; i++) {
    if (obs_extended_info_[i].index_in_proto == -1) {
      obs_extended_info_[i].id = -1;
      obs_extended_info_[i].life_cycle = 0;
    }
  }
  return true;
}

void Warning::ProcessWarningSwitch(
    const soc::WarningFault& warning_fault,
    const std::shared_ptr<LocalView>& localview) {
  const auto& chassis = localview->GetChassis();
  const auto& warning_conf = chassis->vehicle_cfg();
  const auto& warning_switch = chassis->warning_switch_from_cdcs();
  const auto& warning_switch_mem = chassis->warning_switch_mem();
  const bool is_mem_update =
      !common::util::IsProtoEqual(warning_switch_mem, last_warning_switch_mem_);
  warning_switch_.LCA_state =
      warning_conf.lca() &&
      (UpdateWarningSwitch(warning_switch.lca_on_off_set(),
                           warning_switch_mem.lca_on_off_set_mem(),
                           last_warning_switch_.LCA_state,
                           is_mem_update) != WarningSwitch::OFF);
  warning_switch_.RCW_state =
      warning_conf.rcw() &&
      (UpdateWarningSwitch(warning_switch.rcw_on_off_set(),
                           warning_switch_mem.rcw_on_off_set_mem(),
                           last_warning_switch_.RCW_state,
                           is_mem_update) != WarningSwitch::OFF);
  warning_switch_.OSE_state =
      warning_conf.dow() &&
      (UpdateWarningSwitch(warning_switch.dow_on_off_set(),
                           warning_switch_mem.dow_on_off_set_mem(),
                           last_warning_switch_.OSE_state,
                           is_mem_update) != WarningSwitch::OFF);

  warning_switch_.RCTA_state =
      warning_conf.rcta() &&
      (UpdateWarningSwitch(warning_switch.rcta_on_off_set(),
                           warning_switch_mem.rcta_on_off_set_mem(),
                           last_warning_switch_.RCTA_state,
                           is_mem_update) != WarningSwitch::OFF);
  warning_switch_.FCTA_state =
      warning_conf.fcta() &&
      (UpdateWarningSwitch(warning_switch.fcta_on_off_set(),
                           warning_switch_mem.fcta_on_off_set_mem(),
                           last_warning_switch_.FCTA_state,
                           is_mem_update) != WarningSwitch::OFF);

  // reset
  if (chassis->reset_switch().factory_reset() ==
          TL::soc::ResetSwitch_Status_RESET ||
      chassis->reset_switch().reset_all_setup() ==
          TL::soc::ResetSwitch_Status_RESET) {
    warning_switch_.LCA_state = true;
    warning_switch_.RCW_state = true;
    warning_switch_.OSE_state = true;
    warning_switch_.RCTA_state = true;
    warning_switch_.FCTA_state = true;
  }
  last_warning_switch_ = warning_switch_;
  last_warning_switch_mem_ = warning_switch_mem;
}

bool Warning::Process(const soc::WarningFault& warning_fault,
                      const std::shared_ptr<LocalView>& localview,
                      WarningOutput* const warning_output) {
  if (!localview) {
    AERROR << "localview is nullptr";
    return false;
  }
  memset(&(reqPorts.GenObjList.aObject), 0, sizeof(LBS_GenObjectArray));
  memset(&(CTA_reqPorts.CTAEMSRRObjList), 0, sizeof(EMSRRObjectInReq_t));
  ProcessWarningSwitch(warning_fault, localview);
  if (localview->HasFunctionManagerIn() &&
      localview->GetFunctionManagerIn()->ta_pilot_mode() ==
          functionmanager::TaPilotMode::AVP) {
    warning_output->mutable_warning_status()->set_lca_state(
        warning_switch_.LCA_state);
    warning_output->mutable_warning_status()->set_dow_state(
        warning_switch_.OSE_state);
    warning_output->mutable_warning_status()->set_rcta_state(
        warning_switch_.RCTA_state);
    warning_output->mutable_warning_status()->set_fcta_state(
        warning_switch_.FCTA_state);
    warning_output->mutable_warning_status()->set_rcw_state(
        warning_switch_.RCW_state);
    return false;
  }
  if (!localview->HasChassis() || !localview->HasPerceptionObstacles() ||
      !localview->HasLaneMarkers()) {
    AERROR << "-------WARNING------------localview return nullptr---------"
           << localview->HasChassis() << localview->HasPerceptionObstacles()
           << localview->HasLaneMarkers();
    return false;
  }
  reqPorts.LBSSystemParam.bBSDFunctionActive = warning_switch_.LCA_state;
  reqPorts.LBSSystemParam.bBSDFunctionOutputActive = warning_switch_.LCA_state;
  reqPorts.LBSSystemParam.bLCAFunctionActive = warning_switch_.LCA_state;
  reqPorts.LBSSystemParam.bLCAFunctionOutputActive = warning_switch_.LCA_state;
  reqPorts.LBSSystemParam.bOSEFunctionActive = warning_switch_.OSE_state;
  reqPorts.LBSSystemParam.bOSEFunctionOutputActive = warning_switch_.OSE_state;
  reqPorts.LBSSystemParam.bRCWFunctionActive = warning_switch_.RCW_state;
  reqPorts.LBSSystemParam.bRCWFunctionOutputActive = warning_switch_.RCW_state;

  CTA_reqPorts.bFCTAFunctionActive = warning_switch_.FCTA_state;
  CTA_reqPorts.bFCTAFunctionOutputActive = warning_switch_.FCTA_state;
  CTA_reqPorts.bRCTAFunctionActive = warning_switch_.RCTA_state;
  CTA_reqPorts.bRCTAFunctionOutputActive = warning_switch_.RCTA_state;

  const auto& perception_obstacles = localview->GetPerceptionObstacles();
  auto chassis = localview->GetChassis();
  auto localization =
      localview->HasLocalization() ? localview->GetLocalization() : nullptr;
  auto lane_marker = localview->GetLaneMarkers();

  // Chassis::GEAR_NEUTRAL // GEAR_DRIVE // GEAR_REVERSE// GEAR_PARKING
  // GEAR_LOW // GEAR_INVALID // GEAR_NONE //GearPosition_MIN //GearPosition_MAX
  // 0 unkown; 1 GEAR_REVERSE ;2 GEAR_DRIVE; 3 others
  uint8 ego_gear_position = 0;
  bool back_door_closed = true;
  bool hood_ajar_closed = true;
  bool four_doors_closed = true;
  bool ego_six_doors_closed = true;
  bool turn_right = false;
  bool turn_left = false;

  if (chassis->has_gear_location()) {
    if (soc::Chassis::GEAR_REVERSE == chassis->gear_location()) {
      ego_gear_position = 1;
    } else if (soc::Chassis::GEAR_DRIVE == chassis->gear_location()) {
      ego_gear_position = 2;
    } else {
      ego_gear_position = 3;
    }
  }

  if (chassis->has_back_door_status() &&
      chassis->back_door_status().has_status()) {  //后门
    if (soc::BackDoorStatus::FULLY_CLOSED !=
        chassis->back_door_status().status()) {
      back_door_closed = false;
    }
  }

  if (chassis->has_hood_ajar_status() &&
      chassis->hood_ajar_status().has_status()) {  //前盖
    if (soc::HoodAjarStatus::CLOSED != chassis->hood_ajar_status().status()) {
      hood_ajar_closed = false;
    }
  }

  if (chassis->has_door_status() && chassis->door_status().has_fl_door() &&
      chassis->door_status().has_fr_door() &&
      chassis->door_status().has_rl_door() &&
      chassis->door_status().has_rr_door()) {
    if (soc::DoorStatus::OPEN == chassis->door_status().fl_door() ||
        soc::DoorStatus::OPEN == chassis->door_status().fr_door() ||
        soc::DoorStatus::OPEN == chassis->door_status().rl_door() ||
        soc::DoorStatus::OPEN == chassis->door_status().rr_door()) {
      four_doors_closed = false;
    }
  }

  if (!back_door_closed || !hood_ajar_closed || !four_doors_closed) {
    ego_six_doors_closed = false;
  }

  if (chassis->has_signal() &&
      common::VehicleSignal::TURN_RIGHT == chassis->signal().turn_signal()) {
    turn_right = true;
  }

  if (chassis->has_signal() &&
      common::VehicleSignal::TURN_LEFT == chassis->signal().turn_signal()) {
    turn_left = true;
  }
  reqPorts.EgoVehInfo.uLBSGearPosition = ego_gear_position;
  reqPorts.EgoVehInfo.LBSLeftTurnLightOpen = turn_left;
  reqPorts.EgoVehInfo.LBSRightTurnLightOpen = turn_right;
  CTA_reqPorts.EgoVehicleInfo.uCTAGearPosition = ego_gear_position;
  CTA_reqPorts.EgoVehicleInfo.bEgoSixDoorsClosed = ego_six_doors_closed;
  double nearest_s = 0;
  double nearest_l = 0;
  uint32_t lane_count = lane_marker->lane_count();
  double left_lane_position =
      lane_marker->front_left_lane_marker().c0_position();
  double left_lane_kappa_dev =
      6 * lane_marker->front_left_lane_marker().c3_curvature_derivative();
  double left_lane_heading =
      lane_marker->front_left_lane_marker().c1_heading_angle();
  double left_lane_kappa =
      2 * lane_marker->front_left_lane_marker().c2_curvature() /
      SafeDiv(sqrt((1 + left_lane_heading * left_lane_heading) *
                   (1 + left_lane_heading * left_lane_heading) *
                   (1 + left_lane_heading * left_lane_heading)));
  int32_t left_adj_lane_seq = 0;
  //  lane_marker->next_left_lane_marker().at(0).line_seq();
  int32_t right_adj_lane_seq = 0;
  //  lane_marker->next_right_lane_marker().at(0).line_seq();
  double right_lane_position =
      lane_marker->front_right_lane_marker().c0_position();
  double right_lane_heading =
      lane_marker->front_right_lane_marker().c1_heading_angle();
  double right_lane_kappa =
      2 * lane_marker->front_right_lane_marker().c2_curvature() /
      SafeDiv(sqrt((1 + right_lane_heading * right_lane_heading) *
                   (1 + right_lane_heading * right_lane_heading) *
                   (1 + right_lane_heading * right_lane_heading)));

  double right_lane_kappa_dev =
      6 * lane_marker->front_right_lane_marker().c3_curvature_derivative();

  /*double steering_angle = std::atan(vehicle_param.wheel_base() * curvature) *
                          vehicle_param.steer_ratio();*/
  double driven_curve_radius_met =
      LBS_INPUT_VEHICLE_MAX_DRIVE_CURVE_RADIUS;  //(10000.0F);

  if (chassis->has_steering_angle()) {
    driven_curve_radius_met =
        LBS_INPUT_VEHICLE_WHEEL_BASE /
        SafeDiv((std::tan(DEG2RAD(chassis->steering_angle()) /
                          LBS_INPUT_VEHICLE_STEER_RATIO)));
  }

  double ego_road_curve_radius = WarningUtil::RoadCurveRadius(
      left_lane_position, right_lane_position, 1 / SafeDiv(left_lane_kappa),
      1 / SafeDiv(right_lane_kappa));

  double ego_lane_kappa = 0.0;
  double ego_lane_kappa_dev = 0.0;
  double ego_lane_heading = 0.0;
  WarningUtil::CalculateEgoLaneCurvature(
      right_lane_position, left_lane_position, right_lane_kappa,
      left_lane_kappa, right_lane_kappa_dev, left_lane_kappa_dev,
      right_lane_heading, left_lane_heading, &ego_lane_kappa,
      &ego_lane_kappa_dev, &ego_lane_heading);

  // ego information
  // ego speed
  double ego_speed_mps = chassis->speed_mps();

  // ego yawrate
  double ego_yawrate_radps = 0.0;
  // ego long acceleration
  double ego_lng_acc_mpss = 0.0;
  // ego lateral acceleration
  double ego_lat_acc_mpss = 0.0;

  if (chassis->has_yaw_rate()) {
    ego_yawrate_radps = chassis->yaw_rate();
  }
  if (chassis->has_imu_acc()) {
    ego_lat_acc_mpss = chassis->imu_acc().x();
    ego_lng_acc_mpss = chassis->imu_acc().y();
  }

  // 计算0-80 index 对应在proto中的index和id，及每个障碍物持续的循环数
  if (!CalculateExtendedObsInfo(perception_obstacles, localization)) {
    AERROR << "Failed to calculate extended obs info";
    return false;
  }

  for (int i = 0; i < WARNING_OBS_MAX_NUMBERS;
       i++) {  // The number of obstacle <WARNING_OBS_MAX_NUMBERS
    perception::PerceptionObstacle obs_empty;

    const auto& obs = (obs_extended_info_.at(i).index_in_proto != -1 &&
                       obs_extended_info_.at(i).index_in_proto <
                           perception_obstacles->perception_obstacle().size())
                          ? perception_obstacles->perception_obstacle().at(
                                obs_extended_info_.at(i).index_in_proto)
                          : obs_empty;
    // obstacle id
    sint32 obs_id = -1;
    if (obs.has_id()) {
      obs_id = obs.id();
    }

    // obstacle length
    // double obs_length = obs.length();
    // obstacle width
    //double obs_width = obs.width();
    // obstacle height
    //double obs_height = obs.height();
    // obstacle type,which require warning code
    // matching enumeration
    // double obs_type = obs.type();

    // obstacle tracking time
    //double obs_tracking_time = obs.tracking_time();
    // obstacle orientation in Vehicle Coordinates

    // double obs_orientation_st_dev = obs.orientation_st_dev();

    // obstacle maintenance type

    // initial first, they will be evaluated soon
    CTA_reqPorts.CTAEMSRRObjList[i].bObjStable = false;  //NOLINT
    reqPorts.SRRObjList.aObject[i].Qualifiers.bObjStable = false;
    reqPorts.GenObjList.aObject[i].General.uiMaintenanceState_nu =
        EM_GEN_OBJ_MT_STATE_DELETED;
    CTA_reqPorts.CTAEMSRRObjList[i].uiMaintenanceState_nu =  //NOLINT
        CTA_EM_GEN_OBJECT_MT_STATE_DELETED;
    CTA_reqPorts.CTAEMSRRObjList[i].uiMeasuredTargetFrequency_nu = 0;  //NOLINT
    reqPorts.SRRObjList.aObject[i].Qualifiers.uiMeasuredTargetFrequency_nu = 0;

    uint8 obs_maintence_type = 0;
    if (obs.has_maintenance_type()) {
      obs_maintence_type = ConvertObjMaintenceType(i, obs.maintenance_type());
    } else if (obs.has_id() && obs_id != -1) {
      obs_maintence_type = ConvertObjMaintenceType(i, 4);
    }

    // obstacle motion pattern,which require warning
    // code matching enumeration
    uint8 obs_motion_type =
        EM_GEN_OBJECT_DYN_PROPERTY_STATIONARY;  // obs.motion_type();

    // calculate relative velocity and acceleration
    double obs_rel_velocity_x_trans_after = 0.0;
    double obs_rel_velocity_y_trans_after = 0.0;
    double obs_rel_acceleration_x_trans_after = 0.0;
    double obs_rel_acceleration_y_trans_after = 0.0;
    double obs_velocity_x_trans_after = 0.0;
    double obs_velocity_y_trans_after = 0.0;
    double obs_velocity_trans_after = 0.0;
    //double obs_acceleration_x_trans_after = 0.0;
    //double obs_acceleration_y_trans_after = 0.0;
    double obs_theta_flu = 0.0;
    double obs_trans_after_x = 1000.0;
    double obs_trans_after_y = 1000.0;
    double ego_acceleration_x = 0.0;
    double ego_acceleration_y = 0.0;
    if (obs.has_position_flu() && obs.has_velocity_flu() &&
        obs.has_acceleration_flu() && obs.has_theta_flu()) {
      obs_trans_after_x = obs.position_flu().x();
      obs_trans_after_y = obs.position_flu().y();
      obs_velocity_x_trans_after = obs.velocity_flu().x();
      obs_velocity_y_trans_after = obs.velocity_flu().y();
      obs_velocity_trans_after =
          std::sqrt(std::pow(obs_velocity_x_trans_after, 2) +
                    std::pow(obs_velocity_y_trans_after, 2));
      //obs_acceleration_x_trans_after = obs.acceleration_flu().x();
      //obs_acceleration_y_trans_after = obs.acceleration_flu().y();
      obs_theta_flu = obs.theta_flu();

      double obs_theta_flu_use_velocity =
          atan2(obs_velocity_y_trans_after, obs_velocity_x_trans_after);
      static constexpr double
          max_threshold_of_difference_between_vehicle_heading_and_velocity_angle =
              5.0;
      if (fabs(common::math::NormalizeAngle(obs_theta_flu_use_velocity -
                                            obs_theta_flu)) <
          DEG2RAD(
              max_threshold_of_difference_between_vehicle_heading_and_velocity_angle)) {
        obs_theta_flu = obs_theta_flu_use_velocity;
      }
      if (obs_velocity_trans_after < LBS_OBJSEL_VTARGETMIN)  // 3km/h
      {
        obs_motion_type = EM_GEN_OBJECT_DYN_PROPERTY_STATIONARY;
      } else {
        if (obs.has_motion_type()) {
          obs_motion_type =
              WarningUtil::ConvertObjMotionType(obs.motion_type());

        } else {
          obs_motion_type = EM_GEN_OBJECT_DYN_PROPERTY_MOVING;
        }
      }
      CalculateRelativeSpeed(obs_trans_after_x, obs_trans_after_y,
                             obs.velocity_flu().x(), obs.velocity_flu().y(),
                             0.0, 0.0, ego_speed_mps, 0.0, ego_yawrate_radps,
                             &obs_rel_velocity_x_trans_after,
                             &obs_rel_velocity_y_trans_after);
      RelativeAcceleration(obs.acceleration_flu().x(),
                           obs.acceleration_flu().y(), ego_lng_acc_mpss,
                           ego_lat_acc_mpss, obs_rel_velocity_x_trans_after,
                           obs_rel_velocity_y_trans_after, ego_yawrate_radps,
                           &obs_rel_acceleration_x_trans_after,
                           &obs_rel_acceleration_y_trans_after);
    } else if (localization != nullptr && obs.has_position() &&
               obs.has_velocity() && obs.has_acceleration() &&
               obs.has_theta()) {
      // World Coordinate Syste
      double ego_heading = 0.0;
      double ego_position_x = 0.0;
      double ego_position_y = 0.0;
      double ego_velocity_x = 0.0;
      double ego_velocity_y = 0.0;
      double ego_w = 0.0;
      double s = 0.0;
      double l = 0.0;
      //   double ego_velocity_x_trans_after = 0.0;
      //   double ego_velocity_y_trans_after = 0.0;
      //   uint8_t road_type_w = 0;
      if (localization != nullptr) {
        ego_heading = localization->pose().heading();

        // World Coordinate System ENU
        ego_position_x = localization->pose().position().x();
        ego_position_y = localization->pose().position().y();
        ego_velocity_x = localization->pose().linear_velocity().x();
        ego_velocity_y = localization->pose().linear_velocity().y();
        ego_acceleration_x = localization->pose().linear_acceleration().x();
        ego_acceleration_y = localization->pose().linear_acceleration().y();
        ego_w = localization->pose().angular_velocity().z();
        // hdmap::LaneInfoConstPtr nearest_lane_w;
        // if (localview->HasHDMap()) {
        //   road_type_w = WarningUtil::GetNearestLaneWarning(
        //       localview, localization->pose().position(), nearest_lane_w, &s,
        //       &l);
        // }
        // ENU --> FLU
        // ego_velocity_x_trans_after = ego_velocity_x * cos(ego_heading) +
        //                              ego_velocity_y * sin(ego_heading);
        // ego_velocity_y_trans_after = ego_velocity_y * cos(ego_heading) -
        //                              ego_velocity_x * sin(ego_heading);
      }
      obs_theta_flu = obs.theta() - ego_heading;

      TransformCoordinateSystem(ego_position_x, ego_position_y, ego_heading,
                                obs.position().x(), obs.position().y(),
                                &obs_trans_after_x, &obs_trans_after_y);

      double obs_velocity_x = obs.velocity().x();
      double obs_velocity_y = obs.velocity().y();
      double obs_acceleration_x = obs.acceleration().x();
      double obs_acceleration_y = obs.acceleration().y();
      obs_velocity_x_trans_after = obs_velocity_x * cos(obs_theta_flu) +
                                   obs_velocity_y * sin(obs_theta_flu);
      obs_velocity_y_trans_after = obs_velocity_y * cos(obs_theta_flu) -
                                   obs_velocity_x * sin(obs_theta_flu);
      //obs_acceleration_x_trans_after = obs_acceleration_x * cos(obs_theta_flu) +
      //obs_acceleration_y * sin(obs_theta_flu);
      //obs_acceleration_y_trans_after = obs_acceleration_y * cos(obs_theta_flu) -
      //obs_acceleration_x * sin(obs_theta_flu);

      // initialize obstacle relative velocity and acceleration
      double obs_vel_real_x = 0;
      double obs_vel_real_y = 0;
      double obs_acc_rel_x = 0;
      double obs_acc_rel_y = 0;
      CalculateRelativeSpeed(
          obs.position().x(), obs.position().y(), obs.velocity().x(),
          obs.velocity().y(), ego_position_x, ego_position_y, ego_velocity_x,
          ego_velocity_y, ego_w, &obs_vel_real_x, &obs_vel_real_y);
      RelativeAcceleration(obs.acceleration().x(), obs.acceleration().y(),
                           ego_acceleration_x, ego_acceleration_y,
                           obs_vel_real_x, obs_vel_real_y, ego_w,
                           &obs_acc_rel_x, &obs_acc_rel_y);
      obs_rel_velocity_x_trans_after =
          obs_vel_real_x * cos(ego_heading) + obs_vel_real_y * sin(ego_heading);
      obs_rel_velocity_y_trans_after =
          obs_vel_real_y * cos(ego_heading) - obs_vel_real_x * sin(ego_heading);

      obs_rel_acceleration_x_trans_after =
          obs_acc_rel_x * cos(ego_heading) + obs_acc_rel_y * sin(ego_heading);
      obs_rel_acceleration_y_trans_after =
          obs_acc_rel_y * cos(ego_heading) - obs_acc_rel_x * sin(ego_heading);

    } else {
      // set empty obstalce
    }
    if (obs_extended_info_[i].id != -1) {
      ADEBUG << FIXED << SETPRECISION(6) << "obs idx " << i
             << obs.ShortDebugString();
      ADEBUG << FIXED << SETPRECISION(6) << "obs id " << obs_id << " obs_x "
             << obs_trans_after_x << " obs_y " << obs_trans_after_y
             << ego_speed_mps << " ego_w " << ego_yawrate_radps << " rel_vx "
             << obs_rel_velocity_x_trans_after << " rel_vy "
             << obs_rel_velocity_y_trans_after << " ego_ax " << ego_lng_acc_mpss
             << " ego_ay " << ego_lat_acc_mpss << " ego_w " << ego_yawrate_radps
             << " rel_ax " << obs_rel_acceleration_x_trans_after << " rel_ay "
             << obs_rel_acceleration_y_trans_after << "obs id " << obs_id
             << " vx " << obs_velocity_x_trans_after << " vy "
             << obs_velocity_y_trans_after;
    }
    // add relative velocity and acceleration to object kinematic info
    if (obs.has_id()) {
      auto kinematic_info =
          warning_output->mutable_kinematic_info()->add_object_kinematic_info();
      kinematic_info->set_obj_id(obs_id);
      kinematic_info->set_obj_x(obs_trans_after_x);
      kinematic_info->set_obj_y(obs_trans_after_y);
      kinematic_info->set_ego_speed_mps(ego_speed_mps);
      kinematic_info->set_ego_w_radps(ego_yawrate_radps);
      kinematic_info->set_obj_relative_speed_x(obs_rel_velocity_x_trans_after);
      kinematic_info->set_obj_relative_speed_y(obs_rel_velocity_y_trans_after);
      kinematic_info->set_ego_acceleration_x(ego_lng_acc_mpss);
      kinematic_info->set_ego_acceleration_y(ego_lat_acc_mpss);
      kinematic_info->set_obj_relative_accelerarion_x(
          obs_rel_acceleration_x_trans_after);
      kinematic_info->set_obj_relative_accelerarion_y(
          obs_rel_acceleration_y_trans_after);
    }

    reqPorts.GenObjList.aObject[i].Kinemactic.fDistX_met =
        obs_trans_after_x - LBS_INPUT_VEHICLE_WHEEL_BASE;  //NOLINT
    // constant for simulate
    reqPorts.GenObjList.aObject[i].Kinemactic.fDistXStd_met = 0.2;
    reqPorts.GenObjList.aObject[i].Kinemactic.fDistY_met =
        obs_trans_after_y;  //NOLINT

    // constant for simulate on MDC sqrt(obs.position_covariance(4));
    reqPorts.GenObjList.aObject[i].Kinemactic.fDistYStd_met = 0.2;
    // double creat_time = obs.creation_time();

    // CTA ObjList parameter
    CTA_reqPorts.CTA_Input_obsnumber = WARNING_OBS_MAX_NUMBERS;
    CTA_reqPorts.CTAEMSRRObjList[i].uiID_nu = obs_id;      //NOLINT
    CTA_reqPorts.CTAEMSRRObjList[i].fAbsOrientation_rad =  //NOLINT
        obs_theta_flu;

    CTA_reqPorts.CTAEMSRRObjList[i].fArelX_mpss =  //NOLINT
        obs_rel_acceleration_x_trans_after;
    CTA_reqPorts.CTAEMSRRObjList[i].fArelY_mpss =  //NOLINT
        obs_rel_acceleration_y_trans_after;
    CTA_reqPorts.CTAEMSRRObjList[i].fDistX_met =  //NOLINT
        obs_trans_after_x - LBS_INPUT_VEHICLE_WHEEL_BASE;
    CTA_reqPorts.CTAEMSRRObjList[i].fDistXStd_met = 0.2;  //NOLINT
    // sqrt(obs.position_covariance(0));
    CTA_reqPorts.CTAEMSRRObjList[i].fDistY_met = obs_trans_after_y;
    CTA_reqPorts.CTAEMSRRObjList[i].fDistYStd_met = 0.2;  //NOLINT
    // sqrt(obs.position_covariance(4));
    CTA_reqPorts.CTAEMSRRObjList[i].fFirstDetectX_met =  //NOLINT
        TUE_C_F32_VALUE_INVALID;
    CTA_reqPorts.CTAEMSRRObjList[i].fFirstDetectY_met =  //NOLINT
        TUE_C_F32_VALUE_INVALID;
    CTA_reqPorts.CTAEMSRRObjList[i].fLengthFront_met = 0.5 * obs.length();

    CTA_reqPorts.CTAEMSRRObjList[i].fLengthRear_met = 0.5 * obs.length();

    CTA_reqPorts.CTAEMSRRObjList[i].fMirrorProb_per = 0.2;              //NOLINT
    CTA_reqPorts.CTAEMSRRObjList[i].fProbabilityOfExistence_per = 1.0;  //NOLINT
    // obs.existence_probability();
    CTA_reqPorts.CTAEMSRRObjList[i].fRCS = LBS_INPUT_NO_USE_FLOAT;  //NOLINT
    CTA_reqPorts.CTAEMSRRObjList[i].fRelHeading_rad = obs_theta_flu;
    CTA_reqPorts.CTAEMSRRObjList[i].fRelHeadingStd_rad = 0.1;  //NOLINT
    CTA_reqPorts.CTAEMSRRObjList[i].fVabsX_mps = obs_velocity_x_trans_after;
    CTA_reqPorts.CTAEMSRRObjList[i].fVabsY_mps = obs_velocity_y_trans_after;
    CTA_reqPorts.CTAEMSRRObjList[i].fVrelX_mps = obs_rel_velocity_x_trans_after;
    CTA_reqPorts.CTAEMSRRObjList[i].fVrelY_mps = obs_rel_velocity_y_trans_after;

    CTA_reqPorts.CTAEMSRRObjList[i].fWidthLeft_met = 0.5 * obs.width();
    CTA_reqPorts.CTAEMSRRObjList[i].fWidthRight_met = 0.5 * obs.width();
    CTA_reqPorts.CTAEMSRRObjList[i].uClassification_nu =  //NOLINT
        WarningUtil::ConvertObjType(obs.sub_type());
    CTA_reqPorts.CTAEMSRRObjList[i].uiHighestAssocProb_per = 100;  //NOLINT
    CTA_reqPorts.CTAEMSRRObjList[i].uiLifeCycles_nu =              //NOLINT
        obs_extended_info_[i].life_cycle;

    CTA_reqPorts.CTAEMSRRObjList[i].uiMaintenanceState_nu =  //NOLINT

        obs_maintence_type;
    CTA_reqPorts.CTAEMSRRObjList[i].eDynamicProperty_nu =  //NOLINT
        obs_motion_type;

    reqPorts.GenObjList.aObject[i].Kinemactic.fVrelX_mps =
        obs_rel_velocity_x_trans_after;
    reqPorts.GenObjList.aObject[i].Kinemactic.fVrelY_mps =
        obs_rel_velocity_y_trans_after;
    reqPorts.GenObjList.aObject[i].Kinemactic.fVabsX_mps =
        obs_velocity_x_trans_after;
    reqPorts.GenObjList.aObject[i].Kinemactic.fVabsY_mps =
        obs_velocity_y_trans_after;
    reqPorts.GenObjList.aObject[i].Kinemactic.fArelX_mpss =
        obs_rel_acceleration_x_trans_after;
    reqPorts.GenObjList.aObject[i].Kinemactic.fArelY_mpss =
        obs_rel_acceleration_y_trans_after;

    reqPorts.GenObjList.aObject[i].Geometry.fWidth_met = obs.width();
    reqPorts.GenObjList.aObject[i].Geometry.fWidthLeft_met = 0.5 * obs.width();
    reqPorts.GenObjList.aObject[i].Geometry.fWidthRight_met = 0.5 * obs.width();
    reqPorts.GenObjList.aObject[i].Geometry.fLength_met = obs.length();

    reqPorts.GenObjList.aObject[i].Geometry.fLengthFront_met =
        0.5 * obs.length();
    reqPorts.GenObjList.aObject[i].Geometry.fLengthRear_met =
        0.5 * obs.length();

    // reqPorts.GenObjList.aObject[i].Geometry.fAbsOrientation_rad
    // = 1.72;// RCW
    reqPorts.GenObjList.aObject[i].Geometry.fAbsOrientation_rad = obs_theta_flu;

    reqPorts.GenObjList.aObject[i].Geometry.fAbsOrientationStd_rad = 0.1;

    reqPorts.GenObjList.aObject[i].Geometry.fRelHeading_rad = obs_theta_flu;
    reqPorts.GenObjList.aObject[i].Geometry.fRelHeadingStd_rad = 0.1;
    reqPorts.GenObjList.aObject[i].Geometry.fClosestPointX_met =
        LBS_INPUT_NO_USE_FLOAT;
    reqPorts.GenObjList.aObject[i].Geometry.fClosestPointY_met =
        LBS_INPUT_NO_USE_FLOAT;
    reqPorts.GenObjList.aObject[i].General.uiLifeCycles_nu =
        obs_extended_info_[i].life_cycle;

    reqPorts.GenObjList.aObject[i].General.fLifeTime_s =
        LBS_INPUT_NO_USE_FLOAT;  // NO USE

    reqPorts.GenObjList.aObject[i].General.uiLastMeasuredTimeStamp_ms =
        LBS_INPUT_NO_USE_FLOAT;
    reqPorts.GenObjList.aObject[i].General.uiLastMeasuredCycle_nu =
        LBS_INPUT_NO_USE_FLOAT;
    reqPorts.GenObjList.aObject[i].General.uiMaintenanceState_nu =
        obs_maintence_type;

    reqPorts.GenObjList.aObject[i].General.uiID_nu = obs_id;

    reqPorts.GenObjList.aObject[i].Attributes.eDynamicProperty_nu =
        obs_motion_type;

    reqPorts.GenObjList.aObject[i].Attributes.uiDynConfidence_per =
        LBS_INPUT_OBJ_CONFIDENCE;

    reqPorts.GenObjList.aObject[i].Attributes.eClassification_nu =
        WarningUtil::ConvertObjType(obs.sub_type());
    reqPorts.GenObjList.aObject[i].Attributes.uiClassConfidence_per =
        LBS_INPUT_OBJ_CONFIDENCE;
    reqPorts.GenObjList.aObject[i].Attributes.eObjctOcclusion_nu =
        LBS_INPUT_NO_USE_UINT;

    reqPorts.SRRObjList.aObject[i].History.fFirstDetectX_met =
        TUE_C_F32_VALUE_INVALID;
    reqPorts.SRRObjList.aObject[i].History.fFirstDetectY_met =
        TUE_C_F32_VALUE_INVALID;
    reqPorts.SRRObjList.aObject[i].History.fMaxRange_met =
        TUE_C_F32_VALUE_INVALID;

    reqPorts.SRRObjList.aObject[i].Qualifiers.fProbabilityOfExistence_per = 1.0;
    // obs.existence_probability();
    reqPorts.SRRObjList.aObject[i].Qualifiers.uiHighestAssocProb_per =
        LBS_INPUT_OBJ_CONFIDENCE;
    reqPorts.SRRObjList.aObject[i].Qualifiers.eFusionStatus_nu =
        LBS_INPUT_NO_USE_UINT;

    reqPorts.SRRObjList.aObject[i].RoadRelation.fProbInEgoLane_per =
        LBS_INPUT_OBJ_CONFIDENCE;
    reqPorts.SRRObjList.aObject[i].RoadRelation.fDist2Course_met =
        TUE_C_F32_VALUE_INVALID;
    reqPorts.SRRObjList.aObject[i].RoadRelation.fDist2Border_met =
        TUE_C_F32_VALUE_INVALID;
    reqPorts.SRRObjList.aObject[i].RoadRelation.fGRDTrkProbability_per =
        LBS_INPUT_OBJ_CONFIDENCE;
    reqPorts.SRRObjList.aObject[i].RoadRelation.bDist2BorderValid = TRUE;

    reqPorts.SRRObjList.aObject[i].SensorSpecific.fRCS = LBS_INPUT_NO_USE_FLOAT;
    reqPorts.SRRObjList.aObject[i].SensorSpecific.fMirrorProb_per =
        0.2;  // ose BOJ is not mirror

    reqPorts.EgoVehInfo.bLeftDoorOpen =
        (chassis->door_status().fl_door() == DoorStatus::OPEN ||
         chassis->door_status().fl_door() == DoorStatus::SECONDARY ||
         chassis->door_status().rl_door() == DoorStatus::OPEN ||
         chassis->door_status().rl_door() == DoorStatus::SECONDARY);
    reqPorts.EgoVehInfo.bRightDoorOpen =
        (chassis->door_status().fr_door() == DoorStatus::OPEN ||
         chassis->door_status().fr_door() == DoorStatus::SECONDARY ||
         chassis->door_status().rr_door() == DoorStatus::OPEN ||
         chassis->door_status().rr_door() == DoorStatus::SECONDARY);
    // obstacle current detect sensor

    //   LeftFrontPos 0,
    //   RightFrontPos 1,
    //   LeftRearPos 2,
    //   RightRearPos 3

    eSensorMountingPos_t obs_from_radar_side = UnkownPos;
    if (obs_trans_after_y > 0.0000001) {
      reqPorts.GenObjList.aObject[i].bRightSensor = 0;
      if (obs_trans_after_x > 0.0000001) {
        CTA_reqPorts.CTAEMSRRObjList[i].eSensorMountingPos =  //NOLINT
            LeftFrontPos;
      } else {
        CTA_reqPorts.CTAEMSRRObjList[i].eSensorMountingPos =  //NOLINT
            LeftRearPos;
      }
    } else {  // obs_trans_after_y < 0
      reqPorts.GenObjList.aObject[i].bRightSensor = 1;
      if (obs_trans_after_x > 0.0000001) {
        CTA_reqPorts.CTAEMSRRObjList[i].eSensorMountingPos =  //NOLINT
            RightFrontPos;
      } else {
        CTA_reqPorts.CTAEMSRRObjList[i].eSensorMountingPos =  //NOLINT
            RightRearPos;
      }
    }
  }

  reqPorts.EgoVehInfo.fSelfSteering_rad = DEG2RAD(chassis->steering_angle());
  reqPorts.EgoVehInfo.fegoVelocity_mps = ego_speed_mps;
  reqPorts.EgoVehInfo.fVaregoVelocity_mps = LBS_INPUT_NO_USE_FLOAT;
  reqPorts.EgoVehInfo.fegoAcceleration_mps2 = ego_lng_acc_mpss;
  reqPorts.EgoVehInfo.fVaregoAcceleration_mps2 = LBS_INPUT_NO_USE_FLOAT;
  reqPorts.EgoVehInfo.fLatAccel_mps2 = LBS_INPUT_NO_USE_FLOAT;
  reqPorts.EgoVehInfo.fSlipAngle_rad = LBS_INPUT_NO_USE_FLOAT;

  // CTA Vehicle parameter
  CTA_reqPorts.EgoVehicleInfo.fegoVelocity_mps = ego_speed_mps;
  CTA_reqPorts.EgoVehicleInfo.fYawRate_radps = ego_yawrate_radps;
  CTA_reqPorts.EgoVehicleInfo.fSelfSteering_rad =
      DEG2RAD(chassis->steering_angle());

  // Road information
  // equation X = c3 * Z^3 + c2 * Z^2 + c1 * Z + c0
  //   optional double c0_position = 4;
  //   optional double c1_heading_angle = 5;
  //   optional double c2_curvature = 6;
  //   optional double c3_curvature_derivative = 7;

  reqPorts.Road.fCurveRadius_met = ego_road_curve_radius;
  reqPorts.Road.fDrivenCurveRadius_met = driven_curve_radius_met;
  //TUE_CML_Max(0.1, chassis->speed_mps()) / SafeDiv(chassis->yaw_rate());
  reqPorts.Road.fC0Fused_1pm = ego_lane_kappa;
  reqPorts.Road.fC1Fused_1pm2 = ego_lane_kappa_dev;
  reqPorts.Road.fYawAngleFused_rad = ego_lane_heading;

  reqPorts.Road.fYOffsetFused_met = 6;
  // lane_marker->front_left_lane_marker().c0_position();
  reqPorts.Road.fConfCurvature_per = LBS_INPUT_OBJ_CONFIDENCE;
  reqPorts.Road.fConfYOffset_per = 0.9999;
  reqPorts.Road.fYOffsetFusedOppBorder_met = -6;
  // lane_marker->front_right_lane_marker().c0_position();
  reqPorts.Road.fConfYOppOffset_per = 0.9999;
  reqPorts.Road.fConfAdjacentLanes_per =
      lane_marker->front_left_lane_marker().quality();
  reqPorts.Road.fConfOppositeLanes_per =
      lane_marker->front_right_lane_marker().quality();
  reqPorts.Road.fLaneWidth_met =
      WarningUtil::CalculateLaneWidth(left_lane_position, left_lane_heading,
                                      right_lane_position, right_lane_heading);

  reqPorts.Road.iNumOfAdjacentLanes_nu =
      WarningUtil::CalculateAdjacentLaneCount(lane_count, left_adj_lane_seq,
                                              right_adj_lane_seq);
  reqPorts.Road.iNumOfOppositeLanes_nu = 2;  // No use
  reqPorts.Road.BorderEstmGridData_fC0_1pm = LBS_INPUT_NO_USE_FLOAT;
  reqPorts.Road.BorderEstmGridData_fC1_1pm2 = LBS_INPUT_NO_USE_FLOAT;
  reqPorts.Road.BorderEstmGridData_fYawAngle_rad = LBS_INPUT_NO_USE_FLOAT;
  reqPorts.Road.BorderEstmGridData_fYDist_met = LBS_INPUT_NO_USE_FLOAT;
  reqPorts.Road.BorderEstmGridData_fMaxX_met = LBS_INPUT_NO_USE_FLOAT;
  reqPorts.Road.BorderEstmGridData_fConf_per = LBS_INPUT_OBJ_CONFIDENCE;
  reqPorts.Road.RoadType.uiRoadType = 0;  // road_type_w;
  reqPorts.Road.RoadType.fRoadTypeConf = LBS_INPUT_ROADTYPE_CONFIDENCE;

  // CTA Road parameter
  CTA_reqPorts.CTARoadInformation.fCurveRadius_met =
      fabs(ego_road_curve_radius) < fabs(driven_curve_radius_met)
          ? ego_road_curve_radius
          : driven_curve_radius_met;

  LBS_Exec(&reqPorts, &params, &proPorts, &debugInfo);

  CTA_Exec(&CTA_reqPorts, &CTA_params, &CTA_proPorts, &CTA_debugInfo);
  WarningDebugInfo::WarningStatus(localview, warning_fault, warning_output);

  WarningDebugInfo::BSDDebugInfo(localview, warning_output);
  WarningDebugInfo::RCWDebugInfo(localview, warning_output);
  WarningDebugInfo::DOWDebugInfo(
      perception_obstacles->perception_obstacle_size(), warning_output);
  WarningDebugInfo::FCTADebugInfo(
      perception_obstacles->perception_obstacle_size(), warning_output);
  WarningDebugInfo::LCADebugInfo(localview, warning_output);
  WarningDebugInfo::RCTADebugInfo(localview, warning_output);
  common::util::FillHeader("warning", warning_output);

  return true;
}

void Warning::TransformCoordinateSystem(
    const double origin_x, const double origin_y, const double ego_heading,
    double transformbefore_x, double transformbefore_y, double* tansformafter_x,
    double* transformafter_y) {
  double temp_x = 0.F;
  double temp_y = 0.F;
  const double r_cos = cos(ego_heading);
  const double r_sin = sin(ego_heading);
  temp_x = (transformbefore_x - origin_x) * r_cos +
           (transformbefore_y - origin_y) * r_sin;
  temp_y = (transformbefore_y - origin_y) * r_cos -
           (transformbefore_x - origin_x) * r_sin;
  *tansformafter_x = temp_x;
  *transformafter_y = temp_y;
}

void Warning::CalculateRelativeSpeed(double x_obs, double y_obs,
                                     double vx_obs_abs, double vy_obs_abs,
                                     double x_ego, double y_ego, double vx_ego,
                                     double vy_ego, double w,
                                     double* vx_obs_rel, double* vy_obs_rel) {
  *vx_obs_rel = 0.0;
  *vy_obs_rel = 0.0;

  if (!(isnan(vx_obs_abs) || isnan(y_obs) || isnan(y_ego) || isnan(w) ||
        isnan(vy_obs_abs) || isnan(x_obs) || isnan(x_ego))) {
    *vx_obs_rel = vx_obs_abs + w * (y_obs - y_ego) - vx_ego;
    *vy_obs_rel = vy_obs_abs - w * (x_obs - x_ego) - vy_ego;
  }
}

void Warning::RelativeAcceleration(double ax_obs_abs, double ay_obs_abs,
                                   double ax_ego, double ay_ego,
                                   double vx_obs_rel, double vy_obs_rel,
                                   double w, double* ax_obs_rel,
                                   double* ay_obs_rel) {
  *ax_obs_rel = 0.0;
  *ay_obs_rel = 0.0;
  if (!(isnan(ax_obs_abs) || isnan(ax_ego) || isnan(vy_obs_rel) || isnan(w) ||
        isnan(ay_ego) || isnan(vx_obs_rel) || isnan(vx_obs_rel))) {
    *ax_obs_rel = ax_obs_abs - ax_ego + 2 * vy_obs_rel * w;
    *ay_obs_rel = ay_obs_abs - ay_ego - 2 * vx_obs_rel * w;
  }
}

void Warning::SigHeaderInfo() {
  /*   No use data, give it constant. */
  reqPorts.GenObjList.uiVersionNumber = LBS_INPUT_NO_USE_UINT;
  reqPorts.GenObjList.sSigHeader.uiTimeStamp_ms = LBS_INPUT_NO_USE_UINT;
  reqPorts.GenObjList.sSigHeader.uiMeasurementCounter_nu =
      LBS_INPUT_NO_USE_UINT;
  reqPorts.GenObjList.sSigHeader.uiCycleCounter_nu = LBS_INPUT_NO_USE_UINT;
  reqPorts.GenObjList.sSigHeader.eSigStatus_nu = LBS_INPUT_NO_USE_UINT;
  reqPorts.GenObjList.sSigHeader.a_reserve = LBS_INPUT_NO_USE_UINT;

  reqPorts.SRRObjList.sSigHeader.uiTimeStamp_ms = LBS_INPUT_NO_USE_UINT;
  reqPorts.SRRObjList.sSigHeader.uiMeasurementCounter_nu =
      LBS_INPUT_NO_USE_UINT;

  reqPorts.SRRObjList.sSigHeader.uiCycleCounter_nu = LBS_INPUT_NO_USE_UINT;
  reqPorts.SRRObjList.sSigHeader.eSigStatus_nu = LBS_INPUT_NO_USE_UINT;
  reqPorts.SRRObjList.sSigHeader.a_reserve = LBS_INPUT_NO_USE_UINT;

  // ego information
  reqPorts.EgoVehInfo.sSigHeader.uiTimeStamp_ms = LBS_INPUT_NO_USE_UINT;
  reqPorts.EgoVehInfo.sSigHeader.uiMeasurementCounter_nu =
      LBS_INPUT_NO_USE_UINT;
  reqPorts.EgoVehInfo.sSigHeader.uiCycleCounter_nu = LBS_INPUT_NO_USE_UINT;
  reqPorts.EgoVehInfo.sSigHeader.eSigStatus_nu = LBS_INPUT_NO_USE_UINT;
  reqPorts.EgoVehInfo.sSigHeader.a_reserve = LBS_INPUT_NO_USE_UINT;
}

uint8_t Warning::ConvertObjMaintenceType(uint32_t obs_index,
                                         uint8 fusion_maintence_type) {
  uint8_t ui_maintence_type = EM_GEN_OBJ_MT_STATE_DELETED;

  if (4U == fusion_maintence_type) {
    ui_maintence_type = EM_GEN_OBJ_MT_STATE_MEASURED;

    CTA_reqPorts.CTAEMSRRObjList[obs_index].bObjStable = true;  //NOLINT
    reqPorts.SRRObjList.aObject[obs_index].Qualifiers.bObjStable = true;
    CTA_reqPorts  //NOLINT
        .CTAEMSRRObjList[obs_index]
        .uiMeasuredTargetFrequency_nu = 255;  // 192
    reqPorts.SRRObjList.aObject[obs_index]
        .Qualifiers.uiMeasuredTargetFrequency_nu = 255;  // 192
  } else {
    ui_maintence_type = EM_GEN_OBJ_MT_STATE_DELETED;

    CTA_reqPorts.CTAEMSRRObjList[obs_index].bObjStable = false;  //NOLINT
    reqPorts.SRRObjList.aObject[obs_index].Qualifiers.bObjStable = false;

    if ((1U == fusion_maintence_type) || (3U == fusion_maintence_type)) {
      CTA_reqPorts  //NOLINT
          .CTAEMSRRObjList[obs_index]
          .uiMeasuredTargetFrequency_nu = 128;
      reqPorts.SRRObjList.aObject[obs_index]
          .Qualifiers.uiMeasuredTargetFrequency_nu = 128;
    } else if (2U == fusion_maintence_type) {
      CTA_reqPorts  //NOLINT
          .CTAEMSRRObjList[obs_index]
          .uiMeasuredTargetFrequency_nu = 64;
      reqPorts.SRRObjList.aObject[obs_index]
          .Qualifiers.uiMeasuredTargetFrequency_nu = 64;
    }
  }
  return ui_maintence_type;
}

WarningSwitch::Status Warning::UpdateWarningSwitch(
    const WarningSwitch::Status& status_from_cdcs,
    const WarningSwitchMemory::Status& status_from_mem, const bool last_status,
    const bool is_mem_update) {
  // 上电的周期内，记忆值不更新
  // 默认是关闭，收到记忆值，强制更新成记忆值
  if (is_mem_update) {
    return status_from_mem == WarningSwitchMemory::ON ? WarningSwitch::ON
                                                      : WarningSwitch::OFF;
  } else {  //NOLINT
    // 记忆值不更新，响应用户操作，no action延续上一帧状态
    if (status_from_cdcs == WarningSwitch::ON) {
      return WarningSwitch::ON;
    } else if (status_from_cdcs == WarningSwitch::OFF) {  //NOLINT
      return WarningSwitch::OFF;
    } else {
      return last_status ? WarningSwitch::ON : WarningSwitch::OFF;
    }
  }
}  // namespace warning

void Warning::Stop() {}

}  // namespace warning
}  // namespace planning
}  // namespace TL
