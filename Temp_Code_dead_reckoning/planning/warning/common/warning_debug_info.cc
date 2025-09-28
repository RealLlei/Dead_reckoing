/******************************************************************************
 * Copyright 2022 The TL Authors. All Rights Reserved.
 *****************************************************************************/
#include "planning/warning/common/warning_debug_info.h"
#include "proto/fsm/nnp_fct.pb.h"

using TL::functionmanager::NNPSysState;

namespace TL {
namespace planning {
namespace warning {
namespace WarningDebugInfo {
sint32 ulca_warning_ID = 0;

bool WarningStatus(const std::shared_ptr<LocalView>& localview,
                   const soc::WarningFault& warning_fault,
                   WarningOutput* const warning_output) {
  auto* warning_status = warning_output->mutable_warning_status();
  ulca_warning_ID = 0;

  boolean bsd_warning_l = proPorts.LBSFunState.bBSDWarningLeft;
  boolean bsd_warning_r = proPorts.LBSFunState.bBSDWarningRight;
  boolean lca_warning_l = proPorts.LBSFunState.bLCAWarningLeft;
  boolean lca_warning_r = proPorts.LBSFunState.bLCAWarningRight;
  boolean turn_left = reqPorts.EgoVehInfo.LBSLeftTurnLightOpen;
  boolean turn_right = reqPorts.EgoVehInfo.LBSRightTurnLightOpen;

  boolean dow_warning_l_pre =
      proPorts.LBSFunState.bOSEWarningLeft.bPreWarnActive;
  boolean dow_warning_l_Acu =
      proPorts.LBSFunState.bOSEWarningLeft.bAcuteWarnActive;
  boolean dow_warning_l_DL =
      proPorts.LBSFunState.bOSEWarningLeft.bDoorLockingActive;

  boolean dow_warning_r_pre =
      proPorts.LBSFunState.bOSEWarningRight.bPreWarnActive;
  boolean dow_warning_r_Acu =
      proPorts.LBSFunState.bOSEWarningRight.bAcuteWarnActive;
  boolean dow_warning_r_DL =
      proPorts.LBSFunState.bOSEWarningRight.bDoorLockingActive;

  uint8 rcw_warning = proPorts.LBSFunState.uRCWWarning;
  boolean fcta_l1_warning =
      FALSE;  //CTA_proPorts.bFCTAWarning;  // FCTA_L1 Allways FALSE
  boolean fcta_l2_warning = CTA_proPorts.bFCTAWarning2;
  boolean rcta_l1_warning =
      FALSE;  //CTA_proPorts.bRCTAWarning;  // RCTA_L1 Allways FALSE
  boolean rcta_l2_warning = CTA_proPorts.bRCTAWarningL2;

  warning_status->set_lca_state(reqPorts.LBSSystemParam.bLCAFunctionActive);
  warning_status->set_dow_state(reqPorts.LBSSystemParam.bOSEFunctionActive);
  warning_status->set_rcw_state(reqPorts.LBSSystemParam.bRCWFunctionActive);

  warning_status->set_rcta_state(CTA_reqPorts.bRCTAFunctionActive);
  warning_status->set_fcta_state(CTA_reqPorts.bFCTAFunctionActive);

  if (2 == rcw_warning) {
    warning_status->set_rcw_warning(WarningLevel::LV2_WARNING);
  } else {
    warning_status->set_rcw_warning(WarningLevel::NO_WARNING);
  }

  // RCW_L1 and RCW_L2 both NO need audio play(730 Fuction)
  warning_status->set_rcw_audio_play(FALSE);

  const auto& driver_mode = localview->GetFunctionManagerIn()->driver_mode();
  bool is_lat_control =
      (driver_mode == functionmanager::DriveMode::NNP_LAT_LGT_ACTIVE ||
       driver_mode == functionmanager::DriveMode::NNP_LAT_ACTIVE_LGT_OVERRIDE ||
       driver_mode == functionmanager::DriveMode::NCP_LAT_LGT_ACTIVE ||
       driver_mode == functionmanager::DriveMode::NCP_LAT_ACTIVE_LGT_OVERRIDE ||
       driver_mode == functionmanager::DriveMode::ADAS_LAT_LGT_ACTIVE ||
       driver_mode == functionmanager::DriveMode::ADAS_LAT_ACTIVE_LGT_OVERRIDE);

  // for bsd lca warning
  warning_status->set_lca_right_warning(WarningLevel::NO_WARNING);
  warning_status->set_lca_left_warning(WarningLevel::NO_WARNING);
  bool right_side_obj = false;
  if (debugInfo.LCADebug.LCAWarnInfo.uLCAWarningID_nu >= 0 &&
      debugInfo.LCADebug.LCAWarnInfo.uLCAWarningID_nu <
          WARNING_OBS_MAX_NUMBERS) {
    reqPorts.GenObjList.aObject[debugInfo.LCADebug.LCAWarnInfo.uLCAWarningID_nu]
        .bRightSensor;
  }
  if (1 == (lca_warning_r || bsd_warning_r)) {
    warning_status->set_lca_right_warning(WarningLevel::LV1_WARNING);
    if (1 == turn_right && !is_lat_control) {  // turn right signal
      warning_status->set_lca_right_warning(WarningLevel::LV2_WARNING);
      if (bsd_warning_r) {
        ulca_warning_ID =
            reqPorts.GenObjList
                .aObject[BsdProPorts.BSD_Globals.uBSDWarnActiveRightID_nu]
                .General.uiID_nu;
      }
      if (lca_warning_r && right_side_obj) {
        ulca_warning_ID =
            reqPorts.GenObjList
                .aObject[debugInfo.LCADebug.LCAWarnInfo.uLCAWarningID_nu]
                .General.uiID_nu;
      }
    }
  }
  if (1 == (lca_warning_l || bsd_warning_l)) {
    warning_status->set_lca_left_warning(WarningLevel::LV1_WARNING);
    if (1 == turn_left && !is_lat_control) {  // turn left signal
      warning_status->set_lca_left_warning(WarningLevel::LV2_WARNING);
      if (bsd_warning_l) {
        ulca_warning_ID =
            reqPorts.GenObjList
                .aObject[BsdProPorts.BSD_Globals.uBSDWarnActiveLeftID_nu]
                .General.uiID_nu;
      }
      if (lca_warning_l && !right_side_obj) {
        ulca_warning_ID =
            reqPorts.GenObjList
                .aObject[debugInfo.LCADebug.LCAWarnInfo.uLCAWarningID_nu]
                .General.uiID_nu;
      }
    }
  }
  if (1 == (dow_warning_l_DL || dow_warning_r_DL)) {
    switch (dow_warning_l_DL) {
      case 0:
        warning_status->set_dow_left_warning(WarningLevel::NO_WARNING);
        break;
      case 1:
        warning_status->set_dow_left_warning(WarningLevel::LV3_WARNING);
        warning_status->set_dow_audio_play(true);  // DOW_L3  need audio play

        break;
      default:
        warning_status->set_dow_left_warning(WarningLevel::NO_WARNING);
        break;
    }

    switch (dow_warning_r_DL) {
      case 0:
        warning_status->set_dow_right_warning(WarningLevel::NO_WARNING);
        break;
      case 1:
        warning_status->set_dow_right_warning(WarningLevel::LV3_WARNING);
        warning_status->set_dow_audio_play(true);  // DOW_L3  need audio play

        break;
      default:
        warning_status->set_dow_right_warning(WarningLevel::NO_WARNING);
        break;
    }
  } else if (1 == (dow_warning_l_Acu || dow_warning_r_Acu)) {
    warning_status->set_dow_audio_play(false);  // //DOW_L2 no need audio play

    switch (dow_warning_l_Acu) {
      case 0:
        warning_status->set_dow_left_warning(WarningLevel::NO_WARNING);
        break;
      case 1:
        warning_status->set_dow_left_warning(WarningLevel::LV1_WARNING);
        break;
      default:
        warning_status->set_dow_left_warning(WarningLevel::NO_WARNING);
        break;
    }

    switch (dow_warning_r_Acu) {
      case 0:
        warning_status->set_dow_right_warning(WarningLevel::NO_WARNING);
        break;
      case 1:
        warning_status->set_dow_right_warning(WarningLevel::LV1_WARNING);
        break;
      default:
        warning_status->set_dow_right_warning(WarningLevel::NO_WARNING);
        break;
    }
  } else if (1 == (dow_warning_l_pre || dow_warning_r_pre)) {
    warning_status->set_dow_audio_play(false);  // //DOW_L1 no need audio play

    switch (dow_warning_l_pre) {
      case 0:
        warning_status->set_dow_left_warning(WarningLevel::NO_WARNING);
        break;
      case 1:
        warning_status->set_dow_left_warning(WarningLevel::LV1_WARNING);
        break;
      default:
        warning_status->set_dow_left_warning(WarningLevel::NO_WARNING);
        break;
    }

    switch (dow_warning_r_pre) {
      case 0:
        warning_status->set_dow_right_warning(WarningLevel::NO_WARNING);
        break;
      case 1:
        warning_status->set_dow_right_warning(WarningLevel::LV1_WARNING);
        break;
      default:
        warning_status->set_dow_right_warning(WarningLevel::NO_WARNING);
        break;
    }
  } else {
    warning_status->set_dow_left_warning(WarningLevel::NO_WARNING);
    warning_status->set_dow_right_warning(WarningLevel::NO_WARNING);
    warning_status->set_dow_audio_play(false);
  }

  sint32 uCriticalObjID_nu =
      CTA_debugInfo.FCTAGlobal.uCriticalObjID_nu < CTA_MAX_NUM_OBJECTS
          ? CTA_debugInfo.FCTAGlobal.uCriticalObjID_nu
          : 0;
  eSensorMountingPos_t cta_obs_side = CTA_reqPorts  //NOLINT
                                          .CTAEMSRRObjList[uCriticalObjID_nu]
                                          .eSensorMountingPos;

  if (1 == fcta_l2_warning) {
    if (LeftRearPos == cta_obs_side || LeftFrontPos == cta_obs_side) {
      warning_status->set_fcta_right_warning(WarningLevel::NO_WARNING);
      warning_status->set_fcta_left_warning(WarningLevel::LV2_WARNING);
      warning_status->set_fcta_audio_play(
          AudioPlay::NO_AUDIO_WARNING);  // FCTA_L2  need audio play ->no need
    } else {
      warning_status->set_fcta_right_warning(WarningLevel::LV2_WARNING);
      warning_status->set_fcta_left_warning(WarningLevel::NO_WARNING);
      warning_status->set_fcta_audio_play(
          AudioPlay::NO_AUDIO_WARNING);  // FCTA_L2  need audio play ->no need
    }
  } else if (1 == fcta_l1_warning) {  // FCTA_L1 Allways FALSE
    if (LeftRearPos == cta_obs_side || LeftFrontPos == cta_obs_side) {
      warning_status->set_fcta_right_warning(WarningLevel::NO_WARNING);
      warning_status->set_fcta_left_warning(WarningLevel::LV1_WARNING);
      warning_status->set_fcta_audio_play(
          AudioPlay::NO_AUDIO_WARNING);  // FCTA_L1 NO need audio play
    } else {
      warning_status->set_fcta_right_warning(WarningLevel::LV1_WARNING);
      warning_status->set_fcta_left_warning(WarningLevel::NO_WARNING);
      warning_status->set_fcta_audio_play(
          AudioPlay::NO_AUDIO_WARNING);  // FCTA_L1 NO  need audio play
    }
  } else {
    warning_status->set_fcta_right_warning(WarningLevel::NO_WARNING);
    warning_status->set_fcta_left_warning(WarningLevel::NO_WARNING);
    warning_status->set_fcta_audio_play(AudioPlay::NO_AUDIO_WARNING);
  }

  uCriticalObjID_nu =
      CTA_debugInfo.RCTAGlobal.uCriticalObjID_nu < CTA_MAX_NUM_OBJECTS
          ? CTA_debugInfo.RCTAGlobal.uCriticalObjID_nu
          : 0;
  cta_obs_side = CTA_reqPorts  //NOLINT
                     .CTAEMSRRObjList[uCriticalObjID_nu]
                     .eSensorMountingPos;
  if (1 == rcta_l2_warning) {
    if (LeftRearPos == cta_obs_side || LeftFrontPos == cta_obs_side) {
      warning_status->set_rcta_right_warning(WarningLevel::NO_WARNING);
      warning_status->set_rcta_left_warning(WarningLevel::LV2_WARNING);
      warning_status->set_rcta_audio_play(
          AudioPlay::NO_AUDIO_WARNING);  // RCTA_L2  need audio play ->no need
    } else {
      warning_status->set_rcta_right_warning(WarningLevel::LV2_WARNING);
      warning_status->set_rcta_left_warning(WarningLevel::NO_WARNING);
      warning_status->set_rcta_audio_play(
          AudioPlay::NO_AUDIO_WARNING);  // RCTA_L2  need audio play ->no need
    }

  } else if (1 == rcta_l1_warning) {  // FCTA_L1 Allways FALSE
    if (LeftRearPos == cta_obs_side || LeftFrontPos == cta_obs_side) {
      warning_status->set_rcta_right_warning(WarningLevel::NO_WARNING);
      warning_status->set_rcta_left_warning(WarningLevel::LV1_WARNING);
      warning_status->set_rcta_audio_play(
          AudioPlay::NO_AUDIO_WARNING);  // RCTA_L1 NO need audio play

    } else {
      warning_status->set_rcta_right_warning(WarningLevel::LV1_WARNING);
      warning_status->set_rcta_left_warning(WarningLevel::NO_WARNING);
      warning_status->set_rcta_audio_play(
          AudioPlay::NO_AUDIO_WARNING);  // RCTA_L1 NO need audio play
    }

  } else {
    warning_status->set_rcta_right_warning(WarningLevel::NO_WARNING);
    warning_status->set_rcta_left_warning(WarningLevel::NO_WARNING);
    warning_status->set_rcta_audio_play(AudioPlay::NO_AUDIO_WARNING);
  }
  warning_status->set_lca_fault_status(FunctionFaultStatus::NO_ERROR);
  warning_status->set_dow_fault_status(FunctionFaultStatus::NO_ERROR);
  warning_status->set_rcta_fault_status(FunctionFaultStatus::NO_ERROR);
  warning_status->set_fcta_fault_status(FunctionFaultStatus::NO_ERROR);
  warning_status->set_rcw_fault_status(FunctionFaultStatus::NO_ERROR);
  if (warning_status->lca_state() && warning_fault.lca()) {
    warning_status->set_lca_fault_status(FunctionFaultStatus::SERVICE_REQUIRED);
    warning_status->clear_lca_left_warning();
    warning_status->clear_lca_right_warning();
  }
  if (warning_status->dow_state() && warning_fault.dow()) {
    warning_status->set_dow_fault_status(FunctionFaultStatus::SERVICE_REQUIRED);
    warning_status->clear_dow_left_warning();
    warning_status->clear_dow_right_warning();
    warning_status->clear_dow_audio_play();
  }
  if (warning_status->rcta_state() && warning_fault.rcta()) {
    warning_status->set_rcta_fault_status(
        FunctionFaultStatus::SERVICE_REQUIRED);
    warning_status->clear_rcta_left_warning();
    warning_status->clear_rcta_right_warning();
    warning_status->clear_rcta_audio_play();
    warning_status->clear_rcta_object_type();
  }
  if (warning_status->fcta_state() && warning_fault.fcta()) {
    warning_status->set_fcta_fault_status(
        FunctionFaultStatus::SERVICE_REQUIRED);
    warning_status->clear_fcta_left_warning();
    warning_status->clear_fcta_right_warning();
    warning_status->clear_fcta_audio_play();
    warning_status->clear_fcta_object_type();
  }
  if (warning_status->rcw_state() && warning_fault.rcw()) {
    warning_status->set_rcw_fault_status(FunctionFaultStatus::SERVICE_REQUIRED);
    warning_status->clear_rcw_warning();
    warning_status->clear_rcw_audio_play();
  }

  warning_status->set_rcta_object_type(RCTAObjType::VEHICLE_R);
  warning_status->set_fcta_object_type(FCTAObjType::VEHICLE_F);
  return true;
}

bool BSDDebugInfo(const std::shared_ptr<LocalView>& localview,
                  WarningOutput* const warning_output) {
  uint8 sensorside_debug = 0;
  auto* bsd_warning_ptr = warning_output->mutable_bsd_debug();

  boolean turn_left = reqPorts.EgoVehInfo.LBSLeftTurnLightOpen;
  boolean turn_right = reqPorts.EgoVehInfo.LBSRightTurnLightOpen;
  if (proPorts.LBSFunState.bBSDWarningLeft ||
      proPorts.LBSFunState.bBSDWarningRight) {
    bsd_warning_ptr->set_bsd_warning_active(proPorts.LBSFunState.bBSDWarning);
    bsd_warning_ptr->set_bsd_warning_active_last_cycle(
        BsdProPorts.BSD_Globals.bBSDWarnActiveLeftLastCycle ||
        BsdProPorts.BSD_Globals.bBSDWarnActiveRightLastCycle);

    if (turn_left && proPorts.LBSFunState.bBSDWarningLeft) {
      bsd_warning_ptr->set_bsd_warning_criticle_id(
          reqPorts.GenObjList
              .aObject[BsdProPorts.BSD_Globals.uBSDWarnActiveLeftID_nu]
              .General.uiID_nu);

      bsd_warning_ptr->set_bsd_warning_criticle_ttc(
          LBSCalculate
              .LBSObjInfoList[BsdProPorts.BSD_Globals.uBSDWarnActiveLeftID_nu]
              .fTTC_s);
    } else if (turn_right && proPorts.LBSFunState.bBSDWarningRight) {
      bsd_warning_ptr->set_bsd_warning_criticle_id(
          reqPorts.GenObjList
              .aObject[BsdProPorts.BSD_Globals.uBSDWarnActiveRightID_nu]
              .General.uiID_nu);
      bsd_warning_ptr->set_bsd_warning_criticle_ttc(
          LBSCalculate
              .LBSObjInfoList[BsdProPorts.BSD_Globals.uBSDWarnActiveRightID_nu]
              .fTTC_s);
    } else {
      if (proPorts.LBSFunState.bBSDWarningLeft) {
        bsd_warning_ptr->set_bsd_warning_criticle_id(
            reqPorts.GenObjList
                .aObject[BsdProPorts.BSD_Globals.uBSDWarnActiveLeftID_nu]
                .General.uiID_nu);

        bsd_warning_ptr->set_bsd_warning_criticle_ttc(
            LBSCalculate
                .LBSObjInfoList[BsdProPorts.BSD_Globals.uBSDWarnActiveLeftID_nu]
                .fTTC_s);
      }
      if (proPorts.LBSFunState.bBSDWarningRight) {
        bsd_warning_ptr->set_bsd_warning_criticle_id(
            reqPorts.GenObjList
                .aObject[BsdProPorts.BSD_Globals.uBSDWarnActiveRightID_nu]
                .General.uiID_nu);
        bsd_warning_ptr->set_bsd_warning_criticle_ttc(
            LBSCalculate
                .LBSObjInfoList[BsdProPorts.BSD_Globals
                                    .uBSDWarnActiveRightID_nu]
                .fTTC_s);
      }
    }
  } else {
    bsd_warning_ptr->set_bsd_warning_criticle_ttc(LBS_TTC_INVALID);
    bsd_warning_ptr->set_bsd_warning_criticle_id(0);
  }
  for (uint8 obs_idx = 0; obs_idx < WARNING_OBS_MAX_NUMBERS; obs_idx++) {
    if (reqPorts.GenObjList.aObject[obs_idx].General.uiID_nu == -1) {
      continue;
    }
    auto* bsd_obs_ptr = bsd_warning_ptr->add_lbs_object();
    bsd_obs_ptr->set_id(reqPorts.GenObjList.aObject[obs_idx].General.uiID_nu);
    bsd_obs_ptr->set_sot_delay_time(
        debugInfo.BSDDebug.BSDObjInfo[obs_idx].fSoTDelayTime_s);
    bsd_obs_ptr->set_rear_confidence(
        debugInfo.BSDDebug.BSDObjInfo[obs_idx].fRearConf_nu);
    bsd_obs_ptr->set_bsd_zone_obj_xmin(
        debugInfo.BSDDebug.BSDObjInfo[obs_idx].fBSDZoneObjXmin_met);
    bsd_obs_ptr->set_appearance(
        debugInfo.BSDDebug.BSDObjInfo[obs_idx].ubAppearance_nu);
    bsd_obs_ptr->set_hits_in_font(
        debugInfo.BSDDebug.BSDObjInfo[obs_idx].ubHitsInFront_nu);
    bsd_obs_ptr->set_hits_in_side(
        debugInfo.BSDDebug.BSDObjInfo[obs_idx].ubHitsInSide_nu);
    bsd_obs_ptr->set_hits_in_rear(
        debugInfo.BSDDebug.BSDObjInfo[obs_idx].ubHitsInRear_nu);
    bsd_obs_ptr->set_grd_hit_counter(
        debugInfo.BSDDebug.BSDObjInfo[obs_idx].ubGrdHitCounter_nu);
    bsd_obs_ptr->set_behind_grd_counter(
        debugInfo.BSDDebug.BSDObjInfo[obs_idx].ubBehindGrdCounter_nu);
    bsd_obs_ptr->set_obj_class(
        debugInfo.BSDDebug.BSDObjInfo[obs_idx].ubClass_nu);
    bsd_obs_ptr->set_own_lane_counter(
        debugInfo.BSDDebug.BSDObjInfo[obs_idx].ubOwnLaneCounter_nu);
    bsd_obs_ptr->set_inbsdzone(
        debugInfo.BSDDebug.BSDObjInfo[obs_idx].bInBSDZone);
    bsd_obs_ptr->set_insotzone(
        debugInfo.BSDDebug.BSDObjInfo[obs_idx].bInSOTZone);
    bsd_obs_ptr->set_in_sot_zone_previous(
        debugInfo.BSDDebug.BSDObjInfo[obs_idx].bInSOTZonePrevious);
    bsd_obs_ptr->set_objectand_zone_overlap(
        debugInfo.BSDDebug.BSDObjInfo[obs_idx].bObjectAndZoneOverlap);
    bsd_obs_ptr->set_bsd_relevant(
        debugInfo.BSDDebug.BSDObjInfo[obs_idx].bBSDRelevant);
    bsd_obs_ptr->set_bsd_warning(
        debugInfo.BSDDebug.BSDObjInfo[obs_idx].bBSDWarning);
    bsd_obs_ptr->set_bsd_warning(
        debugInfo.BSDDebug.BSDObjInfo[obs_idx].bBSDWarning);

    bsd_obs_ptr->set_updated_recently(
        debugInfo.BSDDebug.BSDObjInfo[obs_idx].bUpdatedRecently);
    bsd_obs_ptr->set_updated_recently_weak(
        debugInfo.BSDDebug.BSDObjInfo[obs_idx].bUpdatedRecentlyWeak);
    bsd_obs_ptr->set_lived_long_enough(
        debugInfo.BSDDebug.BSDObjInfo[obs_idx].bLivedLongEnough);
    bsd_obs_ptr->set_quality_enough(
        debugInfo.BSDDebug.BSDObjInfo[obs_idx].bQualityEnough);
    bsd_obs_ptr->set_object_on_ownlane(
        debugInfo.BSDDebug.BSDObjInfo[obs_idx].bObjectOnOwnlane);
    bsd_obs_ptr->set_create_behind_grd(
        debugInfo.BSDDebug.BSDObjInfo[obs_idx].bCreateBehindGRD);
    bsd_obs_ptr->set_object_behind_grd(
        debugInfo.BSDDebug.BSDObjInfo[obs_idx].bObjectBehindGRD);
    bsd_obs_ptr->set_sot_delayactive(
        debugInfo.BSDDebug.BSDObjInfo[obs_idx].bSoTDelayActive);
    bsd_obs_ptr->set_short_warn(
        debugInfo.BSDDebug.BSDObjInfo[obs_idx].bShortWarn);
    bsd_obs_ptr->set_is_sot(debugInfo.BSDDebug.BSDObjInfo[obs_idx].bIsSoT);
    bsd_obs_ptr->set_fast_sot(debugInfo.BSDDebug.BSDObjInfo[obs_idx].bFastSoT);
    bsd_obs_ptr->set_plausibility(
        debugInfo.BSDDebug.BSDObjInfo[obs_idx].bPlausibility);
    bsd_obs_ptr->set_possible_wrapped_obj(
        debugInfo.BSDDebug.BSDObjInfo[obs_idx].bPossibleWrappedObj);

    sensorside_debug = debugInfo.BSDDebug.BSDObjInfo[obs_idx].eObjDirection;
    if (1 == sensorside_debug) {
      bsd_obs_ptr->set_obj_direction(SensorDirection::LBSBSD_LEFT_SENSOROBJ);
    } else if (2 == sensorside_debug) {
      bsd_obs_ptr->set_obj_direction(SensorDirection::LBSBSD_RIGHT_SENSOROBJ);
    } else {
      bsd_obs_ptr->set_obj_direction(SensorDirection::LBSBSD_UNKNOW_SENSOR);
    }
  }
  return true;
}

bool RCWDebugInfo(const std::shared_ptr<LocalView>& localview,
                  WarningOutput* const warning_output) {
  // auto obstacles_infor = localview->GetPerceptionObstacles();
  auto* rcw_sub_condition_ptr =
      warning_output->mutable_rcw_debug()->mutable_debug_sub_conditions();
  auto* rcw_warning_info_ptr =
      warning_output->mutable_rcw_debug()->mutable_warning_info();
  auto* rcw_state_machine_ptr = warning_output->mutable_rcw_debug();

  rcw_sub_condition_ptr->set_atv_rcw_warning_active(
      debugInfo.RCWDebug.LBSDebug_RCWDebugSubConditions
          .LBSdebugActive_bRCWWarningActive);
  rcw_sub_condition_ptr->set_atv_corridor_blocked(
      debugInfo.RCWDebug.LBSDebug_RCWDebugSubConditions
          .LBSdebugActive_bCorridorBlocked);
  rcw_sub_condition_ptr->set_psv_v_out_of_range(
      debugInfo.RCWDebug.LBSDebug_RCWDebugSubConditions
          .LBSdebugPassive_VelocityOutOfRange);
  rcw_sub_condition_ptr->set_psv_lat_a_out_of_range(
      debugInfo.RCWDebug.LBSDebug_RCWDebugSubConditions
          .LBSdebugPassive_LatAcceloutofRange);
  rcw_sub_condition_ptr->set_psv_left_turn_light(
      debugInfo.RCWDebug.LBSDebug_RCWDebugSubConditions
          .LBSdebugPassive_LeftTurnLight);
  rcw_sub_condition_ptr->set_psv_right_turn_light(
      debugInfo.RCWDebug.LBSDebug_RCWDebugSubConditions
          .LBSdebugPassive_RightTurnLight);
  rcw_sub_condition_ptr->set_psv_gear_position(
      debugInfo.RCWDebug.LBSDebug_RCWDebugSubConditions
          .LBSdebugPassive_GearPosition);
  rcw_sub_condition_ptr->set_psv_blockingtime_active(
      debugInfo.RCWDebug.LBSDebug_RCWDebugSubConditions
          .LBSdebugPassive_BlockingtimeActive);
  rcw_sub_condition_ptr->set_hmi_switch(
      debugInfo.RCWDebug.LBSDebug_RCWDebugSubConditions
          .LBSdebugHmiOff_bRCWHmiOpen);
  rcw_sub_condition_ptr->set_failure_condition(
      debugInfo.RCWDebug.LBSDebug_RCWDebugSubConditions
          .LBSdebugFailure_bRCWFailureCondition);

  rcw_warning_info_ptr->set_ttc(
      debugInfo.RCWDebug.LBSDebug_RCWWarningInfo.LBSfTTC);
  rcw_warning_info_ptr->set_x_object_warning(
      debugInfo.RCWDebug.LBSDebug_RCWWarningInfo.LBSfXObjectWarning);
  if (debugInfo.RCWDebug.LBSDebug_RCWWarningInfo.LBSuRCWWarningID >=
      WARNING_OBS_MAX_NUMBERS) {
    rcw_warning_info_ptr->set_warning_obj_id(0);
  } else {
    rcw_warning_info_ptr->set_warning_obj_id(
        reqPorts.GenObjList
            .aObject[debugInfo.RCWDebug.LBSDebug_RCWWarningInfo
                         .LBSuRCWWarningID]
            .General.uiID_nu);
  }

  rcw_warning_info_ptr->set_warning_active(
      debugInfo.RCWDebug.LBSDebug_RCWWarningInfo.LBSbRCWWarningActive);
  rcw_warning_info_ptr->set_warning_active_last_cycle(
      debugInfo.RCWDebug.LBSDebug_RCWWarningInfo.LBSbRCWWarningActiveLastCycle);

  uint8 rcw_state_t = debugInfo.RCWDebug.LBSDebug_RCWstatemachine;
  switch (rcw_state_t) {
    case LBSRCWState_Init:
      rcw_state_machine_ptr->set_state_machine(RCWStateMachine::RCW_STATE_INIT);
      break;

    case LBSRCWState_passive:
      rcw_state_machine_ptr->set_state_machine(
          RCWStateMachine::RCW_STATE_PASSIVE);
      break;

    case LBSRCWState_StandBy:
      rcw_state_machine_ptr->set_state_machine(
          RCWStateMachine::RCW_STATE_STANDBY);
      break;

    case LBSRCWState_Active:
      rcw_state_machine_ptr->set_state_machine(
          RCWStateMachine::RCW_STATE_ACTIVE);
      break;

    case LBSRCWState_Failure:
      rcw_state_machine_ptr->set_state_machine(
          RCWStateMachine::RCW_STATE_FAILURE);
      break;

    case LBSRCWState_Off:
      rcw_state_machine_ptr->set_state_machine(RCWStateMachine::RCW_STATE_OFF);
      break;

    default:
      rcw_state_machine_ptr->set_state_machine(RCWStateMachine::RCW_STATE_INIT);
      break;
  }

  for (uint8 obs_idx = 0; obs_idx < WARNING_OBS_MAX_NUMBERS; obs_idx++) {
    if (reqPorts.GenObjList.aObject[obs_idx].General.uiID_nu == -1) {
      continue;
    }
    auto* rcw_object_ptr = warning_output->mutable_rcw_debug()->add_obj_info();
    rcw_object_ptr->set_id(
        reqPorts.GenObjList.aObject[obs_idx].General.uiID_nu);

    rcw_object_ptr->set_ttc_threshold(
        debugInfo.RCWDebug.LBSDebug_RCWObjInfo[obs_idx].LBSfTTCThreshold);
    rcw_object_ptr->set_ttc(LBSCalculate.LBSObjInfoList[obs_idx].fTTC_s);
    rcw_object_ptr->set_ttc_acceleration(
        LBSCalculate.LBSObjInfoList[obs_idx].fTTCAccel_mps2);
    rcw_object_ptr->set_corridor_overlap(
        debugInfo.RCWDebug.LBSDebug_RCWObjInfo[obs_idx].LBSfCorridorOverlap);
    rcw_object_ptr->set_corridor_occupancy(
        debugInfo.RCWDebug.LBSDebug_RCWObjInfo[obs_idx].LBSfCorridorOccupancy);
    rcw_object_ptr->set_object_occupancy(
        debugInfo.RCWDebug.LBSDebug_RCWObjInfo[obs_idx].LBSfObjectOccupancy);
    rcw_object_ptr->set_corridor_occ_threshold(
        debugInfo.RCWDebug.LBSDebug_RCWObjInfo[obs_idx]
            .LBSfCorridorOccThreshold);
    rcw_object_ptr->set_in_corridor_time(
        debugInfo.RCWDebug.LBSDebug_RCWObjInfo[obs_idx].LBSfInCOrridorTime);
    rcw_object_ptr->set_y_break_through(
        debugInfo.RCWDebug.LBSDebug_RCWObjInfo[obs_idx].LBSfYBreakThrough);
    rcw_object_ptr->set_heading_filtered(
        debugInfo.RCWDebug.LBSDebug_RCWObjInfo[obs_idx].LBSfHeadingFiltered);
    rcw_object_ptr->set_corridor_hit_cnt(
        debugInfo.RCWDebug.LBSDebug_RCWObjInfo[obs_idx].LBSuCorridorHitCnt);
    rcw_object_ptr->set_multi_path_cnt(
        debugInfo.RCWDebug.LBSDebug_RCWObjInfo[obs_idx].LBSuMultiPathCnt);
    rcw_object_ptr->set_rcw_quality(
        debugInfo.RCWDebug.LBSDebug_RCWObjInfo[obs_idx].LBSbRCWQuality);
    rcw_object_ptr->set_update_recently(
        debugInfo.RCWDebug.LBSDebug_RCWObjInfo[obs_idx].LBSbUpdateRecently);
    rcw_object_ptr->set_rcw_relevant(
        debugInfo.RCWDebug.LBSDebug_RCWObjInfo[obs_idx].LBSbRCWRelevant);
    rcw_object_ptr->set_in_rcw_corridor(
        debugInfo.RCWDebug.LBSDebug_RCWObjInfo[obs_idx].LBSbInRCWCorridor);
    rcw_object_ptr->set_heading_angle_in_range(
        debugInfo.RCWDebug.LBSDebug_RCWObjInfo[obs_idx]
            .LBSbHeadingAngleInRange);
    rcw_object_ptr->set_obj_corridor_blocked(
        debugInfo.RCWDebug.LBSDebug_RCWObjInfo[obs_idx].LBSbObjCorridorBlocked);
    rcw_object_ptr->set_multi_path_obj(
        debugInfo.RCWDebug.LBSDebug_RCWObjInfo[obs_idx].LBSbMultiPathObj);
    rcw_object_ptr->set_rcw_warning_conditions(
        debugInfo.RCWDebug.LBSDebug_RCWObjInfo[obs_idx]
            .LBSbRCWWarningConditions);
    rcw_object_ptr->set_rcw_warning(
        debugInfo.RCWDebug.LBSDebug_RCWObjInfo[obs_idx].LBSbRCWWarning);
    rcw_object_ptr->set_opposite_overlap(
        debugInfo.RCWDebug.LBSDebug_RCWObjInfo[obs_idx].LBSbOppositeOverlap);
  }

  for (uint8 obs_idx = 0; obs_idx < LBS_RCW_MAX_NOF_CORR_OBJS; obs_idx++) {
    auto* rcw_corridor_obs =
        warning_output->mutable_rcw_debug()->add_corridor_objs();

    rcw_corridor_obs->set_x_dist(
        debugInfo.RCWDebug.LBSDebug_RCWCorridorObjs[obs_idx].LBSfXDist);
    rcw_corridor_obs->set_corridor_occupancy(
        debugInfo.RCWDebug.LBSDebug_RCWCorridorObjs[obs_idx]
            .LBSfCorridorOccupancy);
    rcw_corridor_obs->set_x_min(
        debugInfo.RCWDebug.LBSDebug_RCWCorridorObjs[obs_idx].LBSfXMax);
    rcw_corridor_obs->set_x_max(
        debugInfo.RCWDebug.LBSDebug_RCWCorridorObjs[obs_idx].LBSfXMin);
    rcw_corridor_obs->set_in_corridor_time(
        debugInfo.RCWDebug.LBSDebug_RCWCorridorObjs[obs_idx]
            .LBSfInCorridorTime);
    rcw_corridor_obs->set_cor_obj_id(
        debugInfo.RCWDebug.LBSDebug_RCWCorridorObjs[obs_idx].LBSuObjID);
  }
  return true;
}

bool DOWDebugInfo(int perception_obstacles_size,
                  WarningOutput* warning_output) {
  // DOW Obj
  warning_output->mutable_dow_debug()->mutable_dow_obj_info()->Reserve(
      perception_obstacles_size);
  for (uint8 obs_idx = 0; obs_idx < WARNING_OBS_MAX_NUMBERS; obs_idx++) {
    if (reqPorts.GenObjList.aObject[obs_idx].General.uiID_nu == -1) {
      continue;
    }

    auto* dow_obs_ptr = warning_output->mutable_dow_debug()->add_dow_obj_info();
    dow_obs_ptr->set_id(reqPorts.GenObjList.aObject[obs_idx].General.uiID_nu);
    dow_obs_ptr->mutable_dow_info_level()->Reserve(LBS_OSE_NUM_OF_WARN_LEVELS);
    for (auto& dow_level :
         debugInfo.OSEDebug.OSEObjInfoArray[obs_idx].InfoLevel) {
      auto* dow_level_ptr = dow_obs_ptr->add_dow_info_level();
      dow_level_ptr->set_bt_hit_hyst_timer(dow_level.fBTHitHystTimer);
      dow_level_ptr->set_warning(dow_level.bWarning);
      dow_level_ptr->set_warning_last_cycle(dow_level.bWarningLastCycle);
      dow_level_ptr->set_object_in_range(dow_level.bObjectInRange);
      dow_level_ptr->set_bt_hit_hyst_active(dow_level.bBTHitHystActive);
      dow_level_ptr->mutable_breakthrough_hit()->Reserve(
          LBS_OSE_LBS_NUM_OF_BREAK_LINES);

      dow_level_ptr->mutable_breakthrough_hit()->Reserve(
          LBS_OSE_LBS_NUM_OF_BREAK_LINES);
      dow_level_ptr->mutable_breakthrough_hit_confi()->Reserve(
          LBS_OSE_LBS_NUM_OF_BREAK_LINES);
      dow_level_ptr->mutable_ttc_below_thresh()->Reserve(
          LBS_OSE_LBS_NUM_OF_BREAK_LINES);
      for (uint8 dow_nbl = 0; dow_nbl < LBS_OSE_LBS_NUM_OF_BREAK_LINES;
           ++dow_nbl) {
        dow_level_ptr->add_breakthrough_hit_confi(
            dow_level.uBreakthroughHitConfi[dow_nbl]);  //NOLINT
        dow_level_ptr->add_breakthrough_hit(
            dow_level.bBreakthroughHit[dow_nbl]);  //NOLINT
        dow_level_ptr->add_ttc_below_thresh(
            dow_level.bTTCBelowThresh[dow_nbl]);  //NOLINT
      }
    }

    dow_obs_ptr->mutable_y_breakthrough()->Reserve(
        LBS_OSE_LBS_NUM_OF_BREAK_LINES);
    dow_obs_ptr->mutable_y_breakthrough_std()->Reserve(
        LBS_OSE_LBS_NUM_OF_BREAK_LINES);
    dow_obs_ptr->mutable_ttc()->Reserve(LBS_OSE_LBS_NUM_OF_BREAK_LINES);
    dow_obs_ptr->mutable_ttc_filtered()->Reserve(
        LBS_OSE_LBS_NUM_OF_BREAK_LINES);
    dow_obs_ptr->mutable_dist_to_crossing_line()->Reserve(
        LBS_OSE_LBS_NUM_OF_BREAK_LINES);
    for (uint8 dow_nbl = 0; dow_nbl < LBS_OSE_LBS_NUM_OF_BREAK_LINES;
         ++dow_nbl) {
      dow_obs_ptr->add_y_breakthrough(debugInfo  //NOLINT
                                          .OSEDebug.OSEObjInfoArray[obs_idx]
                                          .fYBreakthrough[dow_nbl]);
      dow_obs_ptr->add_y_breakthrough_std(debugInfo  //NOLINT
                                              .OSEDebug.OSEObjInfoArray[obs_idx]
                                              .fYBreakthroughStd[dow_nbl]);
      dow_obs_ptr->add_ttc(debugInfo  //NOLINT
                               .OSEDebug.OSEObjInfoArray[obs_idx]
                               .fTTC_s[dow_nbl]);
      dow_obs_ptr->add_ttc_filtered(debugInfo  //NOLINT
                                        .OSEDebug.OSEObjInfoArray[obs_idx]
                                        .fTTCFiltered_s[dow_nbl]);
      dow_obs_ptr->add_dist_to_crossing_line(
          debugInfo  //NOLINT
              .OSEDebug.OSEObjInfoArray[obs_idx]
              .fDistToCrossingLine_met[dow_nbl]);
    }
    dow_obs_ptr->set_dow_relevant(
        debugInfo.OSEDebug.OSEObjInfoArray[obs_idx].bRelevant);  //NOLINT
    dow_obs_ptr->set_dow_mirror_object(
        debugInfo.OSEDebug.OSEObjInfoArray[obs_idx].bMirror);  //NOLINT
    dow_obs_ptr->set_dow_side_track(
        debugInfo.OSEDebug.OSEObjInfoArray[obs_idx].bSideTrack);  //NOLINT
    dow_obs_ptr->set_dow_object_from_rear(
        debugInfo.OSEDebug.OSEObjInfoArray[obs_idx].bObjectFromRear);  //NOLINT
    dow_obs_ptr->set_dow_valid_approach_angle(
        debugInfo  //NOLINT
            .OSEDebug.OSEObjInfoArray[obs_idx]
            .bValidApproachAngle);
    dow_obs_ptr->set_dow_object_at_edge_fov(
        debugInfo.OSEDebug.OSEObjInfoArray[obs_idx].bObjectAtEdgeFoV);  //NOLINT
    dow_obs_ptr->set_dow_short_ttc(
        debugInfo.OSEDebug.OSEObjInfoArray[obs_idx].bShortTTC);  //NOLINT
    dow_obs_ptr->set_dow_updated_recently(
        debugInfo.OSEDebug.OSEObjInfoArray[obs_idx].bUpdatedRecently);  //NOLINT
    dow_obs_ptr->set_dow_quality(
        debugInfo.OSEDebug.OSEObjInfoArray[obs_idx].fQuality);  //NOLINT
    for (auto uCounter : debugInfo.OSEDebug.OSEObjInfoArray[obs_idx]
                             .fEstWidth.uCounters) {  //  uint16 uCounters[3];
      dow_obs_ptr->mutable_est_width()->add_counters(uCounter);
      //    dow_obs_ptr->mutable_est_width()->set_counters(
      //  i, debugInfo.OSEDebug.OSEObjInfoArray->fEstWidth.uCounters[i]);
    }
  }
  // DOW Global
  warning_output->mutable_dow_debug()->mutable_dow_global()->set_critical_ttc(
      OSEProPorts.fCriticalTTC);
  warning_output->mutable_dow_debug()
      ->mutable_dow_global()
      ->set_critical_obj_dist_x(OSEProPorts.fCriticalObjDistX);
  if (OSEProPorts.uCriticalObjID >= WARNING_OBS_MAX_NUMBERS) {
    warning_output->mutable_dow_debug()
        ->mutable_dow_global()
        ->set_critical_obj_id(0);
  } else {
    warning_output->mutable_dow_debug()
        ->mutable_dow_global()
        ->set_critical_obj_id(
            reqPorts.GenObjList.aObject[OSEProPorts.uCriticalObjID]
                .General.uiID_nu);
  }

  warning_output->mutable_dow_debug()
      ->mutable_dow_global()
      ->set_warning_interrupt(OSEProPorts.bWarningInterrupt);
  for (auto dow_level : OSEProPorts.bOSEWarnActive) {
    warning_output->mutable_dow_debug()
        ->mutable_dow_global()
        ->add_ose_warn_active(dow_level);
  }
  return TRUE;
}

bool FCTADebugInfo(int perception_obstacles_size,
                   WarningOutput* warning_output) {

  warning_output->mutable_fcta_debug()->mutable_fcta_obj_global()->Reserve(
      perception_obstacles_size);

  for (uint8 obs_idx = 0; obs_idx < WARNING_OBS_MAX_NUMBERS; obs_idx++) {
    if (CTA_reqPorts.CTAEMSRRObjList[obs_idx].uiID_nu == -1) {  //NOLINT
      continue;
    }
    // FCTA Obj

    auto fcta_obj_global_ptr =
        warning_output->mutable_fcta_debug()->add_fcta_obj_global();
    fcta_obj_global_ptr->set_id(
        CTA_reqPorts.CTAEMSRRObjList[obs_idx].uiID_nu);  //NOLINT
    fcta_obj_global_ptr->add_bt_hit_hyst_timer(
        CTA_debugInfo.FCTAObjGlobal[obs_idx].fBTHitHystTimer_s[0]);  //NOLINT
    fcta_obj_global_ptr->add_bt_hit_hyst_timer(
        CTA_debugInfo.FCTAObjGlobal[obs_idx].fBTHitHystTimer_s[1]);  //NOLINT

    fcta_obj_global_ptr->mutable_breakthrough_hit()->Reserve(
        CTA_FCTA_CFG_NUM_OF_WARN_LEVELS);

    fcta_obj_global_ptr->mutable_breakthrough_hitconfi()->Reserve(
        CTA_FCTA_CFG_NUM_OF_WARN_LEVELS);

    fcta_obj_global_ptr->mutable_warning_condition()->Reserve(
        CTA_FCTA_CFG_NUM_OF_WARN_LEVELS);
    fcta_obj_global_ptr->mutable_ttc_below_thresh()->Reserve(
        CTA_FCTA_CFG_NUM_OF_WARN_LEVELS);
    fcta_obj_global_ptr->mutable_bt_hit_hyst_active()->Reserve(
        CTA_FCTA_CFG_NUM_OF_WARN_LEVELS);

    fcta_obj_global_ptr->add_breakthrough_hitconfi(
        CTA_debugInfo
            .FCTAObjGlobal[obs_idx]  //NOLINT
            .uBreakthroughHitConfi[0]);
    fcta_obj_global_ptr->add_breakthrough_hitconfi(
        CTA_debugInfo
            .FCTAObjGlobal[obs_idx]  //NOLINT
            .uBreakthroughHitConfi[1]);

    fcta_obj_global_ptr->add_breakthrough_hit(
        CTA_debugInfo.FCTAObjGlobal[obs_idx].bBreakthroughHit[0]);  //NOLINT
    fcta_obj_global_ptr->add_breakthrough_hit(
        CTA_debugInfo.FCTAObjGlobal[obs_idx].bBreakthroughHit[1]);  //NOLINT

    fcta_obj_global_ptr->add_warning_condition(
        CTA_debugInfo.FCTAObjGlobal[obs_idx].bWarning[0]);  //NOLINT
    fcta_obj_global_ptr->add_warning_condition(
        CTA_debugInfo.FCTAObjGlobal[obs_idx].bWarning[1]);  //NOLINT

    fcta_obj_global_ptr->add_object_in_range(
        CTA_debugInfo.FCTAObjGlobal[obs_idx].bObjectInRange[0]);  //NOLINT
    fcta_obj_global_ptr->add_object_in_range(
        CTA_debugInfo.FCTAObjGlobal[obs_idx].bObjectInRange[1]);  //NOLINT

    fcta_obj_global_ptr->add_ttc_below_thresh(
        CTA_debugInfo.FCTAObjGlobal[obs_idx].bTTCBelowThresh[0]);  //NOLINT
    fcta_obj_global_ptr->add_ttc_below_thresh(
        CTA_debugInfo.FCTAObjGlobal[obs_idx].bTTCBelowThresh[1]);  //NOLINT

    fcta_obj_global_ptr->add_bt_hit_hyst_active(
        CTA_debugInfo.FCTAObjGlobal[obs_idx].bBTHitHystActive[0]);  //NOLINT
    fcta_obj_global_ptr->add_bt_hit_hyst_active(
        CTA_debugInfo.FCTAObjGlobal[obs_idx].bBTHitHystActive[1]);  //NOLINT

    fcta_obj_global_ptr->set_relevant(
        CTA_debugInfo.FCTAObjGlobal[obs_idx].bRelevant);  //NOLINT
    fcta_obj_global_ptr->set_mirror(
        CTA_debugInfo.FCTAObjGlobal[obs_idx].bMirror);  //NOLINT
    fcta_obj_global_ptr->set_object_from_side(
        CTA_debugInfo.FCTAObjGlobal[obs_idx].bObjectFromSide);  //NOLINT
    fcta_obj_global_ptr->set_approach_angle_valid(
        CTA_debugInfo.FCTAObjGlobal[obs_idx].bValidApproachAngle);  //NOLINT
    fcta_obj_global_ptr->set_short_ttc(
        CTA_debugInfo.FCTAObjGlobal[obs_idx].bShortWarning);  //NOLINT
    fcta_obj_global_ptr->set_updated_recently(
        CTA_debugInfo.FCTAObjGlobal[obs_idx].bUpdatedRecently);  //NOLINT
    fcta_obj_global_ptr->set_quality(
        CTA_debugInfo.FCTAObjGlobal[obs_idx].bQuality);  //NOLINT
    fcta_obj_global_ptr->set_x_breakthrough(
        FCTAreqPorts  //NOLINT
            .CTGlobalInput.CTObjectListGlobalInput[obs_idx]
            .fXBreakthrough_met);
    fcta_obj_global_ptr->set_dist_to_crossing_line_filtered(
        FCTAreqPorts  //NOLINT
            .CTGlobalInput.CTObjectListGlobalInput[obs_idx]
            .fDistToCrossingLineFiltered_met);
    fcta_obj_global_ptr->set_dist_to_crossing_line(
        FCTAreqPorts  //NOLINT
            .CTGlobalInput.CTObjectListGlobalInput[obs_idx]
            .fDistToCrossingLine_met);
    fcta_obj_global_ptr->set_ttc(
        FCTAreqPorts  //NOLINT
            .CTGlobalInput.CTObjectListGlobalInput[obs_idx]
            .fTTC_s);
    fcta_obj_global_ptr->set_ttc_filtered(
        FCTAreqPorts  //NOLINT
            .CTGlobalInput.CTObjectListGlobalInput[obs_idx]
            .fTTCFiltered_s);
    fcta_obj_global_ptr->set_rear_track(
        FCTAreqPorts  //NOLINT
            .CTGlobalInput.CTObjectListGlobalInput[obs_idx]
            .bRearTrack_nu);
  }
  // FCTA Global
  warning_output->mutable_fcta_debug()->mutable_ttc_threshold()->Reserve(
      CTA_FCTA_CFG_NUM_OF_WARN_LEVELS);

  warning_output->mutable_fcta_debug()->mutable_xmin_breakthrough()->Reserve(
      CTA_FCTA_CFG_NUM_OF_WARN_LEVELS);

  warning_output->mutable_fcta_debug()->mutable_xmax_breakthrough()->Reserve(
      CTA_FCTA_CFG_NUM_OF_WARN_LEVELS);

  warning_output->mutable_fcta_debug()->mutable_obj_max_range()->Reserve(
      CTA_FCTA_CFG_NUM_OF_WARN_LEVELS);

  warning_output->mutable_fcta_debug()->mutable_fcta_warning_active()->Reserve(
      CTA_FCTA_CFG_NUM_OF_WARN_LEVELS);

  auto* fcta_debug_ptr = warning_output->mutable_fcta_debug();
  fcta_debug_ptr->add_ttc_threshold(
      CTA_debugInfo.FCTAGlobal.fTTCThreshold_s[0]);
  fcta_debug_ptr->add_ttc_threshold(
      CTA_debugInfo.FCTAGlobal.fTTCThreshold_s[1]);

  fcta_debug_ptr->add_xmin_breakthrough(
      CTA_debugInfo.FCTAGlobal.fXMinBreakthrough_met[0]);
  fcta_debug_ptr->add_xmin_breakthrough(
      CTA_debugInfo.FCTAGlobal.fXMinBreakthrough_met[1]);

  fcta_debug_ptr->add_xmax_breakthrough(
      CTA_debugInfo.FCTAGlobal.fXMaxBreakthrough_met[0]);
  fcta_debug_ptr->add_xmax_breakthrough(
      CTA_debugInfo.FCTAGlobal.fXMaxBreakthrough_met[1]);

  fcta_debug_ptr->add_obj_max_range(
      CTA_debugInfo.FCTAGlobal.fMaxObjRange_met[0]);
  fcta_debug_ptr->add_obj_max_range(
      CTA_debugInfo.FCTAGlobal.fMaxObjRange_met[1]);

  fcta_debug_ptr->add_fcta_warning_active(
      CTA_debugInfo.FCTAGlobal.bFCTAWarnActive[0]);
  fcta_debug_ptr->add_fcta_warning_active(
      CTA_debugInfo.FCTAGlobal.bFCTAWarnActive[1]);

  fcta_debug_ptr->set_max_heading_angle(
      CTA_debugInfo.FCTAGlobal.fMaxHeadingAngle_deg);

  fcta_debug_ptr->set_min_heading_angle(
      CTA_debugInfo.FCTAGlobal.fMinHeadingAngle_deg);

  fcta_debug_ptr->set_obj_critical_ttc(CTA_debugInfo.FCTAGlobal.fCriticalTTC_s);

  fcta_debug_ptr->set_obj_critical_y(
      CTA_debugInfo.FCTAGlobal.fCriticalObjDistY_met);

  if (CTA_debugInfo.FCTAGlobal.uCriticalObjID_nu >= WARNING_OBS_MAX_NUMBERS) {
    fcta_debug_ptr->set_obj_critical_id(0);
  } else {
    fcta_debug_ptr->set_obj_critical_id(
        CTA_reqPorts  //NOLINT
            .CTAEMSRRObjList[CTA_debugInfo.FCTAGlobal.uCriticalObjID_nu]
            .uiID_nu);
  }

  fcta_debug_ptr->set_interrupt_cycle_count(
      CTA_debugInfo.FCTAGlobal.uInterruptCycleCount_nu);

  fcta_debug_ptr->set_obj_last_critical_y(
      CTA_debugInfo.FCTAGlobal.fCriticalObjDistYLastCycle_met);

  fcta_debug_ptr->set_warning_interrupt(
      CTA_debugInfo.FCTAGlobal.bWarningInterrupt);

  return TRUE;
}

bool RCTADebugInfo(const std::shared_ptr<LocalView>& localview,
                   WarningOutput* const warning_output) {
  // auto perception_obstacles = localview->GetPerceptionObstacles();

  //###############################RCTA################################//
  for (uint8 obs_idx = 0; obs_idx < WARNING_OBS_MAX_NUMBERS; obs_idx++) {
    if (CTA_reqPorts.CTAEMSRRObjList[obs_idx].uiID_nu == -1) {  //NOLINT
      continue;
    }
    auto rcta_obj_global_ptr =
        warning_output->mutable_rcta_debug()->add_rcta_obj_global();
    rcta_obj_global_ptr->set_id(
        CTA_reqPorts.CTAEMSRRObjList[obs_idx].uiID_nu);  //NOLINT
    rcta_obj_global_ptr->add_bt_hit_hyst_timer(
        CTA_debugInfo.RCTAObjGlobal[obs_idx].fBTHitHystTimer_s[0]);  //NOLINT
    rcta_obj_global_ptr->add_bt_hit_hyst_timer(
        CTA_debugInfo.RCTAObjGlobal[obs_idx].fBTHitHystTimer_s[1]);  //NOLINT
    rcta_obj_global_ptr->add_breakthrough_hitconfi(
        CTA_debugInfo
            .RCTAObjGlobal[obs_idx]  //NOLINT
            .uBreakthroughHitConfi[0]);
    rcta_obj_global_ptr->add_breakthrough_hitconfi(
        CTA_debugInfo
            .RCTAObjGlobal[obs_idx]  //NOLINT
            .uBreakthroughHitConfi[1]);
    rcta_obj_global_ptr->add_breakthrough_hit(
        CTA_debugInfo.RCTAObjGlobal[obs_idx].bBreakthroughHit[0]);  //NOLINT
    rcta_obj_global_ptr->add_breakthrough_hit(
        CTA_debugInfo.RCTAObjGlobal[obs_idx].bBreakthroughHit[1]);  //NOLINT
    rcta_obj_global_ptr->add_warning_condition(
        CTA_debugInfo.RCTAObjGlobal[obs_idx].bWarning[0]);  //NOLINT
    rcta_obj_global_ptr->add_warning_condition(
        CTA_debugInfo.RCTAObjGlobal[obs_idx].bWarning[1]);  //NOLINT
    rcta_obj_global_ptr->add_object_in_range(
        CTA_debugInfo.RCTAObjGlobal[obs_idx].bObjectInRange[0]);  //NOLINT
    rcta_obj_global_ptr->add_object_in_range(
        CTA_debugInfo.RCTAObjGlobal[obs_idx].bObjectInRange[1]);  //NOLINT
    rcta_obj_global_ptr->add_ttc_below_thresh(
        CTA_debugInfo.RCTAObjGlobal[obs_idx].bTTCBelowThresh[0]);  //NOLINT
    rcta_obj_global_ptr->add_ttc_below_thresh(
        CTA_debugInfo.RCTAObjGlobal[obs_idx].bTTCBelowThresh[1]);  //NOLINT
    rcta_obj_global_ptr->add_bt_hit_hyst_active(
        CTA_debugInfo.RCTAObjGlobal[obs_idx].bBTHitHystActive[0]);  //NOLINT
    rcta_obj_global_ptr->add_bt_hit_hyst_active(
        CTA_debugInfo.RCTAObjGlobal[obs_idx].bBTHitHystActive[1]);  //NOLINT
    rcta_obj_global_ptr->set_relevant(
        CTA_debugInfo.RCTAObjGlobal[obs_idx].bRelevant);  //NOLINT
    rcta_obj_global_ptr->set_mirror(
        CTA_debugInfo.RCTAObjGlobal[obs_idx].bMirror);  //NOLINT
    rcta_obj_global_ptr->set_object_from_side(
        CTA_debugInfo.RCTAObjGlobal[obs_idx].bObjectFromSide);  //NOLINT
    rcta_obj_global_ptr->set_approach_angle_valid(
        CTA_debugInfo.RCTAObjGlobal[obs_idx].bValidApproachAngle);  //NOLINT
    rcta_obj_global_ptr->set_short_ttc(
        CTA_debugInfo.RCTAObjGlobal[obs_idx].bShortTTC);  //NOLINT
    rcta_obj_global_ptr->set_updated_recently(
        CTA_debugInfo.RCTAObjGlobal[obs_idx].bUpdatedRecently);  //NOLINT
    rcta_obj_global_ptr->set_quality(
        CTA_debugInfo.RCTAObjGlobal[obs_idx].bQuality);  //NOLINT
    rcta_obj_global_ptr->set_x_breakthrough(
        RCTAreqPorts  //NOLINT
            .CTAGlobleInput.RCTACTObjListInput[obs_idx]
            .fXBreakthrough_met);
    rcta_obj_global_ptr->set_dist_to_crossing_line(
        RCTAreqPorts  //NOLINT
            .CTAGlobleInput.RCTACTObjListInput[obs_idx]
            .fDistToCrossingLine_met);
    rcta_obj_global_ptr->set_ttc(RCTAreqPorts  //NOLINT
                                     .CTAGlobleInput.RCTACTObjListInput[obs_idx]
                                     .fTTC_s);
    rcta_obj_global_ptr->set_ttc_filtered(
        RCTAreqPorts  //NOLINT
            .CTAGlobleInput.RCTACTObjListInput[obs_idx]
            .fTTCFiltered_s);
    rcta_obj_global_ptr->set_rear_track(
        RCTAreqPorts  //NOLINT
            .CTAGlobleInput.RCTACTObjListInput[obs_idx]
            .bRearTrack_nu);
  }
  // RCTA not combined with obj
  auto rcta_debug_ptr = warning_output->mutable_rcta_debug();
  rcta_debug_ptr->add_ttc_threshold(
      CTA_debugInfo.RCTAGlobal.fTTCThreshold_s[0]);
  rcta_debug_ptr->add_ttc_threshold(
      CTA_debugInfo.RCTAGlobal.fTTCThreshold_s[1]);

  rcta_debug_ptr->add_xmin_breakthrough(
      CTA_debugInfo.RCTAGlobal.fXMinBreakthrough_met[0]);
  rcta_debug_ptr->add_xmin_breakthrough(
      CTA_debugInfo.RCTAGlobal.fXMinBreakthrough_met[1]);

  rcta_debug_ptr->add_xmax_breakthrough(
      CTA_debugInfo.RCTAGlobal.fXMaxBreakthrough_met[0]);
  rcta_debug_ptr->add_xmax_breakthrough(
      CTA_debugInfo.RCTAGlobal.fXMaxBreakthrough_met[1]);
  rcta_debug_ptr->add_obj_max_range(
      CTA_debugInfo.RCTAGlobal.fMaxObjRange_met[0]);
  rcta_debug_ptr->add_obj_max_range(
      CTA_debugInfo.RCTAGlobal.fMaxObjRange_met[1]);

  rcta_debug_ptr->set_max_heading_angle(
      CTA_debugInfo.RCTAGlobal.fMaxHeadingAngle_deg);
  rcta_debug_ptr->set_min_heading_angle(
      CTA_debugInfo.RCTAGlobal.fMinHeadingAngle_deg);
  rcta_debug_ptr->set_obj_critical_ttc(CTA_debugInfo.RCTAGlobal.fCriticalTTC_s);
  rcta_debug_ptr->set_obj_critical_y(
      CTA_debugInfo.RCTAGlobal.fCriticalObjDistY_met);

  if (CTA_debugInfo.RCTAGlobal.uCriticalObjID_nu >= WARNING_OBS_MAX_NUMBERS) {
    rcta_debug_ptr->set_obj_critical_id(0);
  } else {
    rcta_debug_ptr->set_obj_critical_id(
        CTA_reqPorts  //NOLINT
            .CTAEMSRRObjList[CTA_debugInfo.RCTAGlobal.uCriticalObjID_nu]
            .uiID_nu);
  }
  rcta_debug_ptr->set_interrupt_cycle_count(
      CTA_debugInfo.RCTAGlobal.uInterruptCycleCount_nu);
  rcta_debug_ptr->set_warning_interrupt(
      CTA_debugInfo.RCTAGlobal.bWarningInterrupt);

  rcta_debug_ptr->add_rcta_warning_active(
      CTA_debugInfo.RCTAGlobal.bRCTAWarnActive[0]);
  rcta_debug_ptr->add_rcta_warning_active(
      CTA_debugInfo.RCTAGlobal.bRCTAWarnActive[1]);
  return true;
}

bool LCADebugInfo(const std::shared_ptr<LocalView>& localview,
                  WarningOutput* const warning_output) {
  //############################### SI ################################//
  for (uint8 obs_idx = 0; obs_idx < WARNING_OBS_MAX_NUMBERS; obs_idx++) {
    if (reqPorts.GenObjList.aObject[obs_idx].General.uiID_nu == -1) {
      continue;
    }
    auto si_obj_info_ptr =
        warning_output->mutable_si_debug()->add_si_obj_info();
    auto si_obj_info_output =
        warning_output->mutable_si_debug()->add_si_obj_info_output();
    si_obj_info_ptr->set_id(
        reqPorts.GenObjList.aObject[obs_idx].General.uiID_nu);
    // SIBoolTag
    si_obj_info_ptr->mutable_si_bool_tag()->set_occupancy_inlane(
        debugInfo.SIDebug.SI_Info_Debug[obs_idx].SIBool.bInLOccValue);  //NOLINT
    si_obj_info_ptr->mutable_si_bool_tag()->set_custom_inlane(
        debugInfo  //NOLINT
            .SIDebug.SI_Info_Debug[obs_idx]
            .SIBool.bInLCustomValue);
    si_obj_info_ptr->mutable_si_bool_tag()->set_quality_inlane(
        debugInfo  //NOLINT
            .SIDebug.SI_Info_Debug[obs_idx]
            .SIBool.bInLQualityValue);
    si_obj_info_ptr->mutable_si_bool_tag()->set_object_occupancy_inlane(
        debugInfo  //NOLINT
            .SIDebug.SI_Info_Debug[obs_idx]
            .SIBool.bInLObjOccValue);
    si_obj_info_ptr->mutable_si_bool_tag()->set_lane_occupancy_inlane(
        debugInfo  //NOLINT
            .SIDebug.SI_Info_Debug[obs_idx]
            .SIBool.bInLLaneOccValue);
    si_obj_info_ptr->mutable_si_bool_tag()->set_lane_overlap_inlane(
        debugInfo  //NOLINT
            .SIDebug.SI_Info_Debug[obs_idx]
            .SIBool.bInLLaneOverlapValue);
    si_obj_info_ptr->mutable_si_bool_tag()->set_time_inlane(
        debugInfo  //NOLINT
            .SIDebug.SI_Info_Debug[obs_idx]
            .SIBool.bInLTimeValue);
    si_obj_info_ptr->mutable_si_bool_tag()->set_occupancy_outlane(
        debugInfo  //NOLINT
            .SIDebug.SI_Info_Debug[obs_idx]
            .SIBool.bOutLOccValue);
    si_obj_info_ptr->mutable_si_bool_tag()->set_custom_outlane(
        debugInfo  //NOLINT
            .SIDebug.SI_Info_Debug[obs_idx]
            .SIBool.bOutLCustomValue);
    si_obj_info_ptr->mutable_si_bool_tag()->set_object_occupancy_outlane(
        debugInfo  //NOLINT
            .SIDebug.SI_Info_Debug[obs_idx]
            .SIBool.bOutLObjOccValue);
    si_obj_info_ptr->mutable_si_bool_tag()->set_lane_occupancy_outlane(
        debugInfo  //NOLINT
            .SIDebug.SI_Info_Debug[obs_idx]
            .SIBool.bOutLLaneOccValue);
    si_obj_info_ptr->mutable_si_bool_tag()->set_lane_overlap_outlane(
        debugInfo  //NOLINT
            .SIDebug.SI_Info_Debug[obs_idx]
            .SIBool.bOutLLaneOverlapValue);
    // SIObjLaneState
    si_obj_info_ptr->mutable_si_obj_lane_state()->set_obj_in2out_lane_counter(
        debugInfo  //NOLINT
            .SIDebug.SI_Info_Debug[obs_idx]
            .ObjLaneLCAStatus.uIn2OutlaneTransition_nu);
    si_obj_info_ptr->mutable_si_obj_lane_state()->set_obj_inlane_counter(
        debugInfo  //NOLINT
            .SIDebug.SI_Info_Debug[obs_idx]
            .ObjLaneLCAStatus.uIn2OutlaneTransition_nu);
    si_obj_info_ptr->mutable_si_obj_lane_state()
        ->set_obj_in_corridor_relevant_timer(
            debugInfo  //NOLINT
                .SIDebug.SI_Info_Debug[obs_idx]
                .ObjLaneLCAStatus.fCorridorRelevantTime_s);
    si_obj_info_ptr->mutable_si_obj_lane_state()
        ->set_obj_in_corridor_relevant_distance(
            debugInfo  //NOLINT
                .SIDebug.SI_Info_Debug[obs_idx]
                .ObjLaneLCAStatus.fCorridorRelevantDist_met);
    if (debugInfo  //NOLINT
            .SIDebug.SI_Info_Debug[obs_idx]
            .ObjLaneLCAStatus.SIInlaneState == 1) {
      si_obj_info_ptr->mutable_si_obj_lane_state()->set_si_lane_state(
          SIObjLaneState::SILaneState::SIObjLaneState_SILaneState_OBJ_IN_LANE);
    } else {
      si_obj_info_ptr->mutable_si_obj_lane_state()->set_si_lane_state(
          SIObjLaneState::SILaneState::SIObjLaneState_SILaneState_OBJ_OUT_LANE);
    }

    // SIObjCorridor
    si_obj_info_ptr->mutable_si_obj_corridor()->set_trace_extension_factor(
        debugInfo  //NOLINT
            .SIDebug.SI_Info_Debug[obs_idx]
            .ObjCor.fRelTraceExtensionFactor_nu);
    si_obj_info_ptr->mutable_si_obj_corridor()
        ->set_trace_extension_distance_factor(
            debugInfo  //NOLINT
                .SIDebug.SI_Info_Debug[obs_idx]
                .ObjCor.fRelTraceDistExtensionFactor_nu);
    si_obj_info_ptr->mutable_si_obj_corridor()->set_trace_bracket_offset_left(
        debugInfo  //NOLINT
            .SIDebug.SI_Info_Debug[obs_idx]
            .ObjCor.fTraceBracketOffsetLeft_met);
    si_obj_info_ptr->mutable_si_obj_corridor()->set_trace_bracket_offset_right(
        debugInfo  //NOLINT
            .SIDebug.SI_Info_Debug[obs_idx]
            .ObjCor.fTraceBracketOffsetRight_met);
    // SITrajRefPoint
    si_obj_info_ptr->mutable_si_traj_refpoint()->set_x_met(
        debugInfo  //NOLINT
            .SIDebug.SI_Info_Debug[obs_idx]
            .ObjTrajRefPoint.fX_met);
    si_obj_info_ptr->mutable_si_traj_refpoint()->set_y_met(
        debugInfo  //NOLINT
            .SIDebug.SI_Info_Debug[obs_idx]
            .ObjTrajRefPoint.fY_met);
    si_obj_info_ptr->mutable_si_traj_refpoint()->set_dist_on_traj(
        debugInfo  //NOLINT
            .SIDebug.SI_Info_Debug[obs_idx]
            .ObjTrajRefPoint.fDistOnTraj_met);
    si_obj_info_ptr->mutable_si_traj_refpoint()->set_dist_to_traj(
        debugInfo  //NOLINT
            .SIDebug.SI_Info_Debug[obs_idx]
            .ObjTrajRefPoint.fDistToTraj_met);
    // SITraceBracket
    si_obj_info_ptr->mutable_si_trace_bracket()->set_trace_bracket_left(
        debugInfo  //NOLINT
            .SIDebug.SI_Info_Debug[obs_idx]
            .ObjTraceBracket.fTraceBracketLeft_met);
    si_obj_info_ptr->mutable_si_trace_bracket()->set_trace_bracket_right(
        debugInfo  //NOLINT
            .SIDebug.SI_Info_Debug[obs_idx]
            .ObjTraceBracket.fTraceBracketRight_met);
    // SI global other
    si_obj_info_ptr->set_trace_bracket_met(
        debugInfo.SIDebug.SI_Info_Debug[obs_idx].fTraceBracket_met);  //NOLINT
    si_obj_info_ptr->set_obj_bracket_overlap(debugInfo                //NOLINT
                                                 .SIDebug.SI_Info_Debug[obs_idx]
                                                 .fObjBracketOverlap_met);
    si_obj_info_ptr->set_obj_relvelocity_traj(
        debugInfo.SIDebug.SI_Info_Debug[obs_idx].fVrelToTraj_mps);  //NOLINT
    si_obj_info_ptr->set_obj_inlane_predict_number(
        debugInfo  //NOLINT
            .SIDebug.SI_Info_Debug[obs_idx]
            .uInlanePredictionTimer_nu);
    si_obj_info_ptr->set_obj_outlane_predict_number(
        debugInfo  //NOLINT
            .SIDebug.SI_Info_Debug[obs_idx]
            .uOutlanePredictionTimer_nu);

    warning_output->mutable_si_debug()->add_associated_lane_list(
        debugInfo.SIDebug.AssociatedLane[obs_idx]);  //NOLINT

    // SI Obj Info Output
    si_obj_info_output->set_associate_lane(
        SIProPorts.SIObjInfoList[obs_idx].eAssociatedLane);
    si_obj_info_output->set_vrel_to_traj_mps(
        SIProPorts.SIObjInfoList[obs_idx].fVrelToTraj_mps);
    si_obj_info_output->set_dist_to_traj_met(
        debugInfo.SIDebug.SI_Obj_Info[obs_idx].fDistToTraj_met);  //NOLINT
    si_obj_info_output->set_trace_bracket_left(
        SIProPorts.SIObjInfoList[obs_idx].fTraceBracketLeft_met);
    si_obj_info_output->set_trace_bracket_right(
        SIProPorts.SIObjInfoList[obs_idx].fTraceBracketRight_met);
    si_obj_info_output->set_obj_bracket_overlap(
        SIProPorts.SIObjInfoList[obs_idx].fObjBracketOverlap_met);

    //###############################LCA################################//
    auto* lca_obj_info_ptr =
        warning_output->mutable_lca_debug()->add_lca_obj_info();
    lca_obj_info_ptr->set_ttc_threshold(
        debugInfo.LCADebug.LCAObjOutputList[obs_idx].fTTCThreshold);
    lca_obj_info_ptr->set_ttc(LBSCalculate.LBSObjInfoList[obs_idx].fTTC_s);
    lca_obj_info_ptr->set_ttc_acceleration(
        LBSCalculate.LBSObjInfoList[obs_idx].fTTCAccel_mps2);
    lca_obj_info_ptr->set_behind_guardrail_probability(
        debugInfo.LCADebug.LCAObjOutputList[obs_idx].fBehindGrdProb_per);
    lca_obj_info_ptr->set_front_mirror_counter(
        debugInfo.LCADebug.LCAObjOutputList[obs_idx].uFrontMirrorCnt);
    lca_obj_info_ptr->set_update_recently(
        debugInfo.LCADebug.LCAObjOutputList[obs_idx].bUpdateRecently);
    lca_obj_info_ptr->set_in_lca_range(
        debugInfo.LCADebug.LCAObjOutputList[obs_idx].bInLCARange);
    lca_obj_info_ptr->set_lca_mirror_object(
        debugInfo.LCADebug.LCAObjOutputList[obs_idx].bLCAMirrorObject);
    lca_obj_info_ptr->set_lca_mirror_front_object(
        debugInfo.LCADebug.LCAObjOutputList[obs_idx].bLCAMirrorFrontObject);
    lca_obj_info_ptr->set_lca_obj_path_invalid(
        debugInfo.LCADebug.LCAObjOutputList[obs_idx].bLCAObjPathInvalid);
    lca_obj_info_ptr->set_lca_quality(
        debugInfo.LCADebug.LCAObjOutputList[obs_idx].bLCAQuality);
    lca_obj_info_ptr->set_lca_relevant(
        debugInfo.LCADebug.LCAObjOutputList[obs_idx].bLCARelevant);
    lca_obj_info_ptr->set_lca_warning_conditions(
        debugInfo.LCADebug.LCAObjOutputList[obs_idx].bLCAWarningConditions);
    lca_obj_info_ptr->set_lca_warning_flag(
        debugInfo.LCADebug.LCAObjOutputList[obs_idx].bLCAWarning);
    lca_obj_info_ptr->set_id(
        reqPorts.GenObjList.aObject[obs_idx].General.uiID_nu);
  }

  // SI not combined with obj
  auto si_global_ptr = warning_output->mutable_si_debug()->mutable_si_global();
  si_global_ptr->set_si_seek_lane_width(
      debugInfo.SIDebug.SI_Global_Debug.fSIseekLaneWidth_met);
  si_global_ptr->set_si_track_lane_width(
      debugInfo.SIDebug.SI_Global_Debug.fSITrackLaneWidth_met);
  si_global_ptr->set_lane_width(
      debugInfo.SIDebug.SI_Global_Debug.fLaneWidth_met);
  si_global_ptr->set_curve_radius_min_filter(
      debugInfo.SIDebug.SI_Global_Debug.fCurveRadiusMinFiltered_met);
  si_global_ptr->set_lane_change_flag(
      debugInfo.SIDebug.SI_Global_Debug.bLaneChange);

  // SIPredict
  auto si_predict_ptr =
      warning_output->mutable_si_debug()->mutable_si_predict_distance();
  si_predict_ptr->set_pdist(debugInfo.SIDebug.SI_PredictDist_Debug.pdist);
  si_predict_ptr->set_pdist_var(
      debugInfo.SIDebug.SI_PredictDist_Debug.pdist_var);
  si_predict_ptr->set_pdist_var_fullpred(
      debugInfo.SIDebug.SI_PredictDist_Debug.pdist_var_fullpred);

  // LCA not combined with obj
  // LCAWarnInfo_t
  auto lca_global_ptr =
      warning_output->mutable_lca_debug()->mutable_lca_global();
  lca_global_ptr->mutable_lca_warn_info()->set_critial_ttc(
      debugInfo.LCADebug.LCAWarnInfo.fCriticalTTC_s);
  lca_global_ptr->mutable_lca_warn_info()->set_object_warning_x_met(
      debugInfo.LCADebug.LCAWarnInfo.fXObjectWarning_met);
  lca_global_ptr->mutable_lca_warn_info()->set_lca_warning_id(ulca_warning_ID);

  lca_global_ptr->mutable_lca_warn_info()->set_lca_warning_active(
      debugInfo.LCADebug.LCAWarnInfo.bLCAWarnActive);
  lca_global_ptr->mutable_lca_warn_info()->set_lca_warning_active_lastcycle(
      debugInfo.LCADebug.LCAWarnInfo.bLCAWarningLastCycle);
  // LCAConfig
  lca_global_ptr->mutable_lca_config()->set_ttc_thresh_vrel_low(
      debugInfo.LCADebug.LCAConfig.fTTCThreshVrelLow_s);
  lca_global_ptr->mutable_lca_config()->set_ttc_thresh_vrel_mid(
      debugInfo.LCADebug.LCAConfig.fTTCThreshVrelMid_s);
  lca_global_ptr->mutable_lca_config()->set_ttc_thresh_vrel_high(
      debugInfo.LCADebug.LCAConfig.fTTCThreshVrelHigh_s);
  lca_global_ptr->mutable_lca_config()->set_ttc_hysteresis(
      debugInfo.LCADebug.LCAConfig.fTTCHysteresis_s);
  lca_global_ptr->mutable_lca_config()->set_lca_max_range_met(
      debugInfo.LCADebug.LCAConfig.fLCARangeMax_met);
  lca_global_ptr->mutable_lca_config()->set_lca_max_curve_rad_met(
      debugInfo.LCADebug.LCAConfig.fLCACurveRadMax_met);
  lca_global_ptr->mutable_lca_config()->set_lca_warning_duration_cfg(
      debugInfo.LCADebug.LCAConfig.uLCAWarningDurationCfg);
  // LCA Front Mirror
  lca_global_ptr->mutable_lca_front_mirror()->set_front_mirror_rate(
      debugInfo.LCADebug.LCAFrontMirror.fFMObjRate);
  lca_global_ptr->mutable_lca_front_mirror()->set_lca_vf_vx_thresh_add(
      debugInfo.LCADebug.LCAFrontMirror.LCA_Vf_VxThreshAdd_mps);
  lca_global_ptr->mutable_lca_front_mirror()->set_lca_vf_vx_thresh_ownlane_min(
      debugInfo.LCADebug.LCAFrontMirror.LCA_Vf_VxThreshAdjLaneMin_mps);
  lca_global_ptr->mutable_lca_front_mirror()->set_lca_vf_vx_thresh_ownlane_max(
      debugInfo.LCADebug.LCAFrontMirror.LCA_Vf_VxThreshOwnLaneMax_mps);
  lca_global_ptr->mutable_lca_front_mirror()->set_lca_vf_vx_thresh_adjlane_min(
      debugInfo.LCADebug.LCAFrontMirror.LCA_Vf_VxThreshAdjLaneMin_mps);
  lca_global_ptr->mutable_lca_front_mirror()->set_lca_vf_vx_thresh_adjlane_max(
      debugInfo.LCADebug.LCAFrontMirror.LCA_Vf_VxThreshAdjLaneMax_mps);
  lca_global_ptr->mutable_lca_front_mirror()->set_rcs_stable_obj_ownlane(
      debugInfo.LCADebug.LCAFrontMirror.fRCSStableObjOwnLane);
  lca_global_ptr->mutable_lca_front_mirror()->set_rcs_stable_obj_adjlane(
      debugInfo.LCADebug.LCAFrontMirror.fRCSStableObjAdjLane);
  lca_global_ptr->mutable_lca_front_mirror()->set_closet_stable_obj_ownlane_id(
      debugInfo.LCADebug.LCAFrontMirror.uClosetStableObjIDOwnLane);
  lca_global_ptr->mutable_lca_front_mirror()->set_closet_stable_obj_adjlane_id(
      debugInfo.LCADebug.LCAFrontMirror.uClosetStableObjIDAdjLane);

  // LCA Global other
  lca_global_ptr->set_lca_range(debugInfo.LCADebug.LCAGlobal.fLCARange);
  lca_global_ptr->set_lca_left_path_block_counter(
      debugInfo.LCADebug.LCAGlobal.uCntLCAPathBlockedLeft);
  lca_global_ptr->set_lca_right_path_block_counter(
      debugInfo.LCADebug.LCAGlobal.uCntLCAPathBlockedRight);
  lca_global_ptr->set_lca_left_path_block_flag(
      debugInfo.LCADebug.LCAGlobal.bLCAPathBlockedLeft);
  lca_global_ptr->set_lca_right_path_block_flag(
      debugInfo.LCADebug.LCAGlobal.bLCAPathBlockedRight);
  return true;
}

}  // namespace WarningDebugInfo
}  // namespace warning
}  // namespace planning
}  // namespace TL
