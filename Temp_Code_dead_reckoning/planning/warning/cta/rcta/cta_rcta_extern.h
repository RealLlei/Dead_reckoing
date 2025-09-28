#ifndef CTA_RCTA_EXTERN_H
#define CTA_RCTA_EXTERN_H
#ifdef __cplusplus
extern "C" {
#endif

#include "planning/warning/cta/cta_par.h"
#include "planning/warning/lbs/common/TM_Global_Types.h"
/*
#define RCTA_EXTERN_MAX_NUM_OBJECTS \
  80u  // the object's number of side radar output ---> RCTA_MAX_NUM_OBJECTS JO
*/
#define RCTA_EXTERN_NUM_OF_WARN_LEVELS 2u

typedef struct {
  boolean
      bRightSensor;  // Flag whether the object is detected by the right sensor
  uint8 uiMaintenanceState_nu;  // the state whether the object is deleted
  uint8 uiMeasuredTargetFrequency_nu;  // Bitfield to indicate if the object was
                                       // measured in the last 8 cycles
  float32 fDistX_met;         // Object's longitudinal relative distance
  float32 fDistY_met;         // Object's lateral relative distance
  float32 fVrelY_mps;         // Object's lateral relative velocity
  float32 fFirstDetectY_met;  // Y position where the object was created, unit:m
  float32 fRelHeading_rad;    // Heading Angle
  float32 fRelHeadingStd_rad;  // standard deviation heading Angle
  float32 fMirrorProb_per;     // The probability that the object is mirror
  float32 fVabsX_mps;          // Object's longitudinal velocity over ground
  float32 fVabsY_mps;          // Object's lateral velocity over ground
  uint16 uiLifeCycles_nu;      // Object lifetime in cycles,unit: null
  boolean bObjStable;          // Flag that object is stable
  float32
      fProbabilityOfExistence_per;  // Probability that the object represents a
                                    // real object,unit:0.0-1.0f
  float32 fWidthLeft_met;    // Object's width left of the track position(left
                             // sensor view)
  float32 fLengthFront_met;  // Object's length ahead of the track position(left
                             // sensor view)
  float32 fRCS;              // RCS
} RCTAEMSRRObjInReq_t;

typedef struct {
  float32 fXMovement_met;  // The object total moving distance in the x
                           // direction,unit:m
  float32 fYMovement_met;  // The object total moving distance in the y
                           // direction,unit:m
  float32 fUpdateRate_nu;  // The object measurement update rate,unit:NULL
  float32 fAssocProbFiltered_nu;  // Highest cluster association probability of
                                  // the object filter result
} RCTACTAObjListInReq_t;

typedef struct {
  float32 fXBreakthrough_met;
  // float32 fXBreakthroughFiltered_met;
  float32 fXBreakthroughStd_met;
  float32 fTTC_s;          // TTC of the object
  float32 fTTCFiltered_s;  // Filtered TTC of the object
  float32 fDistToCrossingLine_met;
  // float32 fDistToCrossingLineFiltered_met;
  // float32 fRearTrackProb_per;
  // float32 fObjBreakthroughMargin_met;
  boolean bRearTrack_nu;
} RCTACTObjListInReq_t;

typedef struct {
  // float32 fSensorOffsetToSide_met;
  float32 fSensorOffsetToRear_met;  // Distance between mounting position of the
                                    // sensor and vehicle rear side
  RCTACTAObjListInReq_t RCTACTAObjListInput[RCTA_MAX_NUM_OBJECTS];
  RCTACTObjListInReq_t RCTACTObjListInput[RCTA_MAX_NUM_OBJECTS];
} RCTACTAGlobalInReq_t;

typedef struct {
  float32 StWheelAngle_rad;
  uint8 uRCTAGearPosition;
  boolean bRCTAEgoSixDoorsClosed;
} RCTAVehicleSignal_t;

typedef struct {
  boolean bRCTAFunctionActive;
} RCTALastCycleStateInReq_t;

typedef struct {
  float32 fCycleTime_s;  // Current task cycle time from EMGlobalOutput
  boolean
      bRCTAFunctionActive;  // the flag whether RCTA function switch is opened
  boolean bRCTAFunctionOutputActive;  // the switch whether RCTA function
                                      // results is output
  RCTAEMSRRObjInReq_t EMSRRObjListInput[RCTA_MAX_NUM_OBJECTS];
  RCTACTAGlobalInReq_t CTAGlobleInput;
  RCTAVehicleSignal_t RCTAVehicleSig;
  RCTALastCycleStateInReq_t LastCycleStates;
} RCTAInReq_t;

typedef struct {
  float32 fSteerAngleCutOffMin_deg;     // 300
  float32 fSteerAngleCutOffMid_deg;     // 400
  float32 fSteerAngleCutOffMax_deg;     // 500
  float32 fXMinBreakthroughSWAMin_met;  //-5
  float32 fXMinBreakthroughSWAMid_met;  //-4
  float32 fXMinBreakthroughSWAMax_met;  //-3
  boolean bEnableSteeringAngleCutOff_nu;
} RCTASteeringAngleCutOffParam_t;

typedef struct {
  float32 fTTCThreshold_s;    // 2.5
  float32 fTTCThresholdL2_s;  // 1.5
  // float32 fTTCThresholdL3_s;        // 1.299
  float32 fTTCThresholdMargin_s;    // 1
  float32 fXMinBreakthrough_met;    //-6
  float32 fXMinBreakthroughL2_met;  //-4.199
  // float32 fXMinBreakthroughL3_met;  //-2.5
  float32 fXMaxBreakthrough_met;  // 1
  // float32 fVEgoMax;//4.199
  // float32 fVEgoMin;//-5
  float32 fVTargetMin;  // The min target vehicle speed of RCTA is actived;
                        // Value: 1.399f m/s
  // float32 fVTargetMax;//100
  // float32 fKeepTime;//0
  // float32 fNoAlertZoneWidth;//0
  float32 fMaxHeadingAngle;       //-30
  float32 fMinHeadingAngle;       //-160
  float32 fTargetRangeMax_met;    // 42
  float32 fTargetRangeMaxL2_met;  // 42
  // float32 fTargetRangeMaxL3_met;  // 42
  // float32 fBreakthroughMargin;//1
  // boolean bEnableObjAdaptiveBreakthrough;//True
  // boolean bActive;
  RCTASteeringAngleCutOffParam_t SteeringAngleCutOff;
  // float32 fVTargetMinVRU;
  // boolean bEnableSensorLeftRightFusion;
} RCTAAlgoParam_t;

typedef struct {
  RCTAAlgoParam_t RCTAAlgoParam;
} RCTAParam_t;

typedef struct {
  float32 fBTHitHystTimer_s[RCTA_EXTERN_NUM_OF_WARN_LEVELS];
  uint8 uBreakthroughHitConfi[RCTA_EXTERN_NUM_OF_WARN_LEVELS];
  boolean bBreakthroughHit[RCTA_EXTERN_NUM_OF_WARN_LEVELS];
  boolean bWarning[RCTA_EXTERN_NUM_OF_WARN_LEVELS];
  boolean bWarningLastCycle[RCTA_EXTERN_NUM_OF_WARN_LEVELS];
  boolean bObjectInRange[RCTA_EXTERN_NUM_OF_WARN_LEVELS];
  boolean bTTCBelowThresh[RCTA_EXTERN_NUM_OF_WARN_LEVELS];
  boolean bBTHitHystActive[RCTA_EXTERN_NUM_OF_WARN_LEVELS];

  boolean bRelevant;
  boolean bMirror;
  boolean bObjectFromSide;  // Flag whether the object approach from side
  boolean bValidApproachAngle;
  boolean bShortTTC;
  boolean bUpdatedRecently;
  boolean bQuality;
} RCTAObjOutput_t;

typedef struct {
  uint32 uiVersionNumber;
  RCTAObjOutput_t RCTA_Va_ObjectListGlobal[RCTA_MAX_NUM_OBJECTS];
  boolean bRCTAWarnActive[RCTA_EXTERN_NUM_OF_WARN_LEVELS];

  float32 fCriticalTTC_s;
  float32 fCriticalObjDistY_met;
  float32 fCriticalObjDistYLastCycle_met;
  sint32 uCriticalObjID_nu;
  sint32 uCriticalObjIDLastCycle_nu;
  uint8 uInterruptCycleCount_nu;
  boolean bWarningInterrupt;
  float32 fTTCThreshold_s[CTA_RCTA_CFG_NUM_OF_WARN_LEVELS];
  float32 fXMinBreakthrough_met[CTA_RCTA_CFG_NUM_OF_WARN_LEVELS];
  float32 fXMaxBreakthrough_met[CTA_RCTA_CFG_NUM_OF_WARN_LEVELS];
  float32 fMaxObjRange_met[CTA_RCTA_CFG_NUM_OF_WARN_LEVELS];
  float32 fMaxHeadingAngle_deg;  // Calculate the max value of heading angle
  float32 fMinHeadingAngle_deg;  // Calculate the min value of heading angle
} RCTAOutPro_t;

typedef struct {
  uint32 uiVersionNumber;
  //...
} RCTADebug_t;

void RCTAExec(const RCTAInReq_t* RCTAreqPorts, const RCTAParam_t* RCTAparams,
              RCTAOutPro_t* RCTAproPorts, RCTADebug_t* RCTAdebugInfo);
void RCTAReset();

#ifdef __cplusplus
}
#endif
#endif
