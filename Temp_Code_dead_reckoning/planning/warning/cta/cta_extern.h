#ifndef CTA_EXTERN_H
#define CTA_EXTERN_H
#ifdef __cplusplus
extern "C" {
#endif
/*****************************************************************************
 INCLUDES
 *****************************************************************************/
#include "planning/warning/cta/cta_par.h"
#include "planning/warning/cta/fcta/cta_fcta.h"
#include "planning/warning/cta/rcta/cta_rcta.h"
#include "planning/warning/lbs/common/TM_Global_Types.h"

/*****************************************************************************
 MACRO DEFINITION
*****************************************************************************/
//#define CTA_EXTERN_MAX_NUM_OBJECTS 4  // 160 ---> CTA_MAX_NUM_OBJECTS  ---JO

// 160u  // the object's number of side radar output

/*****************************************************************************
 TYPEDEFS : INPUT   PARAMETER    OUTPUT    DEBUG
*****************************************************************************/
typedef struct {
  boolean CTA_Nb_FCTAPowerOffSwitchState_nu;  // the BSD function switch state
                                              // when power is off
  boolean CTA_Nb_RCTAPowerOffSwitchState_nu;  // the LCA function switch state
                                              // when power is off
} CTANVRAM_t;

typedef enum {
  LeftFrontPos,
  RightFrontPos,
  LeftRearPos,
  RightRearPos,
  UnkownPos
} eSensorMountingPos_t;

typedef struct {
  boolean fcta;
  boolean rcta;
} CTA_Switch;

typedef struct {
  eSensorMountingPos_t eSensorMountingPos;
  // boolean bRightSensor;
  ////GenObjKinematics
  float32 fDistX_met;  // Object's longitudinal relative distance
  float32 fDistXStd_met;
  float32 fDistY_met;  // Object's lateral relative distance
  float32 fDistYStd_met;
  float32 fVrelX_mps;  // Object's longitudinal relative velocity
  // float32 fVrelXStd_mps;
  float32 fVrelY_mps;  // Object's lateral relative velocity
  // float32 fVrelYStd_mps;
  float32 fArelX_mpss;
  // float32 fArelXStd_mpss;
  float32 fArelY_mpss;
  // float32 fArelYStd_mpss;
  float32 fVabsX_mps;  // Object's longitudinal velocity over ground
  // float32 fVabsXStd_mps;
  float32 fVabsY_mps;  // Object's lateral velocity over ground
  // float32 fVabsYStd_mps;
  // float32 fAabsX_mpss;
  // float32 fAabsXStd_mpss;
  // float32 fAabsY_mpss;
  // float32 fAabsYStd_mpss;
  ////GenObjGeometry
  // float32 fWidth_met;                                             /*Object's
  // overall width*/ float32 fWidthStd_met; /**/
  float32 fWidthLeft_met;   // Object's width left of the track position(left
                            // sensor view)
  float32 fWidthRight_met;  // Object's width right of the track position(left
                            // sensor view)
  // float32 fLength_met;                                            /*Object's
  // overall length*/ float32 fLengthStd_met; /**/
  float32 fLengthFront_met;  // Object's length ahead of the track position(left
                             // sensor view)
  float32 fLengthRear_met;   // Object's length behind the track position(left
                             // sensor view)
  float32 fAbsOrientation_rad;  // Object moving direction,based on VX and VY in
                                // AUTOSAR(left sensor view)
  // float32 fAbsOrientationStd_rad;
  float32 fRelHeading_rad;     // Heading Angle
  float32 fRelHeadingStd_rad;  // standard deviation heading Angle
  // float32 fClosestPointX_met;                                     /*X
  // position of closet point*/ float32 fClosestPointY_met; /*Y position of
  // closet point*/
  ////GenObjGeneral
  // float32 fLifeTime_s;                                            /*Object
  // lifetime in second,unit: s*/
  uint16 uiLifeCycles_nu;  // Object lifetime in cycles,unit: null
  // uint32 uiLastMeasuredTimeStamp_ms;
  // uint16 uiLastMeasuredCycle_nu;
  uint8 uiMaintenanceState_nu;  // the maintenance(measured,predicted) state
                                // whether the object is deleted
  sint32 uiID_nu;
  ////GenObjAttribute
  uint8 eDynamicProperty_nu; /*Object
  // dynamic property,stationary,moving or oncoming*/
  uint8 uiDynConfidence_per;
  // /*General confidence of dynamic property(moving,crossing,oncoming)*/
  uint32 uClassification_nu;  // Object classification
  // uint8 uiClassConfidence_per; /*Confidences for all classification*/ uint8
  // eObjctOcclusion_nu;
  ////SRRObjHistory
  float32 fFirstDetectX_met;  // X position where the object was created, unit:m
  float32 fFirstDetectY_met;  // Y position where the object was created, unit:m
  // float32 fMaxRange_met;                                          /* Max
  // range over lifetime of this object, unit:m*/
  ////SRRObjQualifier
  float32
      fProbabilityOfExistence_per;  // Probability that the object represents a
                                    // real object,unit:0.0-1.0f
  uint8 uiHighestAssocProb_per;     // Highest association probability of all
                                    // associated clusters in this cycle
  uint8 uiMeasuredTargetFrequency_nu; /*Bitfield to indicate if the object was
                                         measured in the last 8 cycles*/
  // uint8 eFusionStatus_nu; /*Enumeration of fused objects status*/
  boolean bObjStable;  // Flag that object is stable
  ////SRRObjRoadRelation
  // float32 fProbInEgoLane_per;/*undetermined */
  float32 fDist2Course_met;  // Object distance to course, unit:m
  float32 fDist2Border_met;  // Object distance to border, unit:m
  float32 fGRDTrkProbability_per;
  boolean bDist2BorderValid;  // Valid flag if Dist2Border value can be used
  // SRRObjSensorSpecific
  float32 fRCS;             // RCS
  float32 fMirrorProb_per;  // The probability that the object is mirror
} EMSRRObjectInReq_t;

typedef struct {
  float32 fegoVelocity_mps;  // the ego vehicle longitudinal velocity ,unit:m/s
  // float32 fVaregoVelocity_mps;                                    /*the ego
  // vehicle longitudinal velocity variance,unit:m/s*/ float32
  // fegoAcceleration_mps2;                                  /*the ego vehicle
  // longitudinal acceleration ,unit:m/s^2*/ float32 fVaregoAcceleration_mps2;
  // /*the ego vehicle longitudinal acceleration variance,unit:m/s^2*/
  float32 fYawRate_radps;  // the ego vehicle yaw rate ,unit:rad/s
  // float32 fCurve_1pm;                                             /*the ego
  // vehicle driver curve ,unit:1/m*/ float32 DrvIntCurve_1pm; /*the ego vehicle
  // driver curve ,unit:1/m*/ float32 fLatAccel_mps2; /*the ego vehicle Lateral
  // Acceleration,unit:m/s^2*/ float32 fSlipAngle_rad; /*the ego vehicle slip
  // angle ,unit:rad*/
  float32 fSelfSteering_rad;  // the ego vehicle steering angle ,unit:rad
  uint8 uCTAGearPosition;
  boolean bEgoSixDoorsClosed;
} EgoVehicleInReq_t;

typedef struct {
  float32 fCurveRadius_met;  // The curvature radius of the current driver road
                             // ,unit:m
} CTARoadInReq_t;

typedef struct {
  uint16 CTA_Input_obsnumber;   //
  float32 fCycleTime_s;         // Current task cycle time from EMGlobalOutput
  boolean bFCTAFunctionActive;  // FCTA function active flag
  boolean bFCTAFunctionOutputActive;  // FCTA function output flag
  boolean bRCTAFunctionActive;        // RCTA function active flag
  boolean bRCTAFunctionOutputActive;  // RCTA function output flag
  EMSRRObjectInReq_t CTAEMSRRObjList[CTA_MAX_NUM_OBJECTS];
  EgoVehicleInReq_t EgoVehicleInfo;
  CTARoadInReq_t CTARoadInformation;
  CTANVRAM_t CTA_Ns_NVRAM_nu;
} CTAInReq_t;

typedef struct {
  float32 CTA_Kf_LatPos_met;   // the radar sensor mounting Y position ,unit:m
  float32 CTA_Kf_LongPos_met;  // the radar sensor mounting X position ,unit:m
} CTASensorMounting_t;

typedef struct {
  // vehicle body
  float32 CTA_Kf_WheelBase_met;  // The distance between the center of the front
      // wheel and the center of the rear wheel,unit:m*/
  float32 CTA_Kf_VehicleWidth_met;  /*the vehicle body width,unit:m*/
  float32 CTA_Kf_VehicleLength_met; /*the vehicle body length,unit:m*/
  // float32 fVehCenter2FrontAxis_met; /*the vehicle center to front axis center
  // distance*/
  float32 CTA_Kf_OverhangFront_met;  // the length of vehicle front overhang
  CTASensorMounting_t CTA_Ks_LeftFrontSensorMounting;
  CTASensorMounting_t CTA_Ks_RightFrontSensorMounting;
  CTASensorMounting_t CTA_Ks_LeftRearSensorMounting_nu;
  CTASensorMounting_t CTA_Ks_RightRearSensorMounting_nu;
} CTAVehicleParam_t;

typedef struct {
  float32 fSteerAngleCutOffMin_deg;       // 300
  float32 fSteerAngleCutOffMid_deg;       // 400
  float32 fSteerAngleCutOffMax_deg;       // 500
  float32 fXMinBreakthroughSWAMin_met;    //-5
  float32 fXMinBreakthroughSWAMid_met;    //-4
  float32 fXMinBreakthroughSWAMax_met;    //-3
  boolean bEnableSteeringAngleCutOff_nu;  // TRUE
} CTARCTASteeringAngleCutOffParam_t;

typedef struct {
  float32 CTA_Kf_TTCThreshold_s;          // 2.5
  float32 CTA_Kf_TTCThresholdL2_s;        // 1.5
  float32 CTA_Kf_TTCThresholdL3_s;        // 1.299
  float32 CTA_Kf_TTCThresholdMargin_s;    // 1
  float32 CTA_Kf_XMinBreakthrough_met;    //-6
  float32 CTA_Kf_XMinBreakthroughL2_met;  //-4.199
  // float32 CTA_Kf_XMinBreakthroughL3_met;  //-2.5
  float32 CTA_Kf_XMaxBreakthrough_met;  // 1
  float32 CTA_Kf_VEgoMax_mps;           // 4.199
  float32 CTA_Kf_VEgoMin_mps;           //-5
  float32 CTA_Kf_VTargetMin_mps;  // The min target vehicle speed of RCTA is
                                  // actived; Value: 1.399f m/s
  // float32 fVTargetMax;//100
  // float32 fKeepTime;//0
  // float32 fNoAlertZoneWidth;//0
  float32 CTA_Kf_MaxHeadingAngle_deg;   //-30
  float32 CTA_Kf_MinHeadingAngle_deg;   //-160
  float32 CTA_Kf_TargetRangeMax_met;    // 42
  float32 CTA_Kf_TargetRangeMaxL2_met;  // 42
  float32 CTA_Kf_TargetRangeMaxL3_met;  // 42
  // float32 fBreakthroughMargin;//1
  // boolean bEnableObjAdaptiveBreakthrough;//True
  boolean CTA_Kb_Active_nu;  // TRUE
  CTARCTASteeringAngleCutOffParam_t CTA_Ks_SteeringAngleCutOff_nu;
  // float32 fVTargetMinVRU;
  // boolean bEnableSensorLeftRightFusion;
} CTARCTAAlgoParam_t;

typedef struct {
  float32 CTA_Kf_TTCThreshold_s;          // 2.5f
  float32 CTA_Kf_TTCThresholdL2_s;        // 2.5f
  float32 CTA_Kf_TTCThresholdL3_s;        // 2.5f
  float32 CTA_Kf_XMinBreakthrough_met;    // 0.f
  float32 CTA_Kf_XMinBreakthroughL2_met;  // 0.f
  // float32 CTA_Kf_XMinBreakthroughL3_met;  // 0.f
  float32 CTA_Kf_XMaxBreakthrough_met;    // 7.f
  float32 CTA_Kf_VEgoMax_mps;             // 300.f//100
  float32 CTA_Kf_VEgoMin_mps;             //-300.f//0.83
  float32 CTA_Kf_VTargetMin_mps;          // 0.82999998f
  float32 CTA_Kf_VTargetMax_mps;          // 300.f//100
  float32 CTA_Kf_MaxHeadingAngle_deg;     // 120.f
  float32 CTA_Kf_MinHeadingAngle_deg;     // 60.f
  float32 CTA_Kf_TargetRangeMax_met;      // 71.f
  float32 CTA_Kf_TargetRangeMaxL2_met;    // 71.f
  float32 CTA_Kf_TargetRangeMaxL3_met;    // 71.f
  float32 CTA_Kf_BreakthroughMargin_met;  // 1.f
  float32 CTA_Kf_TTCThresholdMargin_s;    // 1.f
  boolean CTA_Kb_Active_nu;               // True
} CTAFCTAAlgoParam_t;

typedef struct {
  CTAVehicleParam_t CTA_Ks_VehicleParameter_nu;
  CTARCTAAlgoParam_t CTA_Ks_RCTAAlgoParameter_nu;
  CTAFCTAAlgoParam_t CTA_Ks_FCTAAlgoParameter_nu;
} CTAParam_t;

typedef struct {
  uint32 uiVersionNumber;
  CTANVRAM_t CTA_Ns_NVRAM_nu;
  float32 fRCTAfTTC_s;
  boolean bRCTAWarning;
  boolean bRCTAWarningL2;
  // boolean bRCTAWarningL3;

  boolean bFCTAWarning;
  boolean bFCTAWarning2;
  float32 fFCTAfTTC_s;
} CTAOutPro_t;

typedef struct {
  uint8 uBreakthroughHitConfi[CTA_MAX_NUM_OBJECTS];
  boolean bObjectInRange[CTA_MAX_NUM_OBJECTS];
  boolean bRelevant;
  boolean bTTCBelowThresh[CTA_MAX_NUM_OBJECTS];
  boolean bMirror;
  boolean bWarningLastCycle[CTA_MAX_NUM_OBJECTS];
  boolean bShortWarning;
  boolean bRearTrack_nu;
  boolean bValidApproachAngle;
  boolean bUpdatedRecently;  // Flag whether the object is updated recently
  boolean bQuality;
  boolean bBTHitHystActive[CTA_MAX_NUM_OBJECTS];
  boolean bFCTAWarning[CTA_MAX_NUM_OBJECTS];
} CTA_ObjInfoArrayDebug_t;

typedef struct {
  uint32 uiVersionNumber;  // uint32 value example
  RCTAObjGlobal_t RCTAObjGlobal[RCTA_MAX_NUM_OBJECTS];
  FCTAObjGlobal_t FCTAObjGlobal[FCTA_MAX_NUM_OBJECTS];
  FCTAGlobal_t FCTAGlobal;
  RCTAGlobal_t RCTAGlobal;
} CTADebug_t;

/////////////////////////////////////  ---- NEED FIX  20220112 JO

/*****************************************************************************
 FUNCTION
*****************************************************************************/
void CTA_Exec(const CTAInReq_t* reqPorts, const CTAParam_t* params,
              CTAOutPro_t* proPorts, CTADebug_t* debugInfo);
void CTA_Reset();

#ifdef __cplusplus
}
#endif
#endif