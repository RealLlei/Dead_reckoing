#ifndef CTA_FCTA_EXTERN_H
#define CTA_FCTA_EXTERN_H
#ifdef __cplusplus
extern "C" {
#endif

#include "planning/warning/cta/cta_par.h"
#include "planning/warning/lbs/common/TM_Global_Types.h"

/*
#define FCTA_EXTERN_MAX_NUM_OBJECTS \
  80u  // the object's number of side radar output ----->FCTA_MAX_NUM_OBJECTS  JO
*/
#define FCTA_EXTERN_NUM_OF_WARN_LEVELS 2u

typedef struct {
  boolean bRightSensor;
  uint8 uiMaintenanceState_nu;  // the state whether the object is deleted
  uint8 uiMeasuredTargetFrequency_nu;  // Bitfield to indicate if the object was
                                       // measured in the last 8 cycles
  uint8 eDynamicProperty_nu;
  float32 fDistX_met;         // Object's longitudinal relative distance
  float32 fDistY_met;         // Object's lateral relative distance
  float32 fFirstDetectX_met;  // X position where the object was created, unit:m
  float32 fFirstDetectY_met;  // Y position where the object was created, unit:m
  // float32 fRelHeading_rad;//Heading Angle
  // float32 fRelHeadingStd_rad;//standard deviation heading Angle
  float32 fAbsOrientation_rad;  // Object moving direction,based on VX and VY in
                                // AUTOSAR(left sensor view)
  float32 fMirrorProb_per;      // The probability that the object is mirror
  float32 fVabsX_mps;           // Object's longitudinal velocity over ground
  float32 fVabsY_mps;           // Object's lateral velocity over ground
  float32 fVrelX_mps;           // Object's longitudinal relative velocity
  float32 fVrelY_mps;           // Object's lateral relative velocity
  float32 fArelX_mpss;
  float32 fArelY_mpss;
  uint16 uiLifeCycles_nu;  // Object lifetime in cycles,unit: null
  boolean bObjStable;      // Flag that object is stable
  float32
      fProbabilityOfExistence_per;  // Probability that the object represents a
                                    // real object,unit:0.0-1.0f
  float32 fWidthLeft_met;  // Object's width left of the track position(left
                           // sensor view)
  float32 fRCS;            // RCS
} FCTAEMSRRObjInReq_t;

typedef struct {
  float32 fegoVelocity_mps;  // the ego vehicle longitudinal velocity ,unit:m/s
  uint8 uFCTAGearPosition;
  boolean bFCTAEgoSixDoorsClosed;
} FCTAVehicleSignalInReq_t;

typedef struct {
  float32 fCurveRadius_met;  // The curvature radius of the current driver road
                             // ,unit:m
} FCTARoadInReq_t;

typedef enum {
  FCTA_INIT,  // Initialize all
  FCTA_OK,    // normal processing
} FCTAStateInReq_t;

typedef struct {
  FCTAStateInReq_t FCTAState;
  boolean bFCTAFunctionActive;
} FCTALastCycleStateInReq_t;

typedef struct {
  float32 fXMovement_met;  // The object total moving distance in the x
                           // direction,unit:m
  float32 fYMovement_met;  // The object total moving distance in the y
                           // direction,unit:m
  float32 fUpdateRate_nu;  // The object measurement update rate,unit:NULL
  float32 fAssocProbFiltered_nu;  // Highest cluster association probability of
                                  // the object filter result
  float32 fVabs;
  float32 fVxPosBased;
  float32 fVyPosBased;
} FCTACTAObjListInReq_t;

typedef struct {
  float32 fXBreakthrough_met;
  // float32 fXBreakthroughFiltered_met;
  float32 fXBreakthroughStd_met;
  float32 fTTC_s;
  float32 fTTCFiltered_s;
  float32 fDistToCrossingLine_met;
  float32 fDistToCrossingLineFiltered_met;
  // float32 fRearTrackProb_per;
  // float32 fObjBreakthroughMargin_met;
  boolean bRearTrack_nu;
} FCTACTObjInReq_t;

typedef struct {
  // float32 fSensorOffsetToSide_met;
  float32 fSensorOffsetToRear_met;  // Distance between mounting position of the
                                    // sensor and vehicle front edge
  FCTACTAObjListInReq_t FCTACTAObjListInput[FCTA_MAX_NUM_OBJECTS];
} FCTACTAGlobalInReq_t;

typedef struct {
  float32 fMaxLatSensorRange;
  FCTACTObjInReq_t CTObjectListGlobalInput[FCTA_MAX_NUM_OBJECTS];
} FCTACTGlobalInReq_t;

typedef struct {
  float32 fCycleTime_s;  // Current task cycle time from EMGlobalOutput
  boolean
      bFCTAFunctionActive;  // the flag whether RCTA function switch is opened
  boolean bFCTAFunctionOutputActive;  // the switch whether RCTA function
                                      // results is output
  FCTALastCycleStateInReq_t LastCycleStates;
  FCTAEMSRRObjInReq_t EMSRRObjListInput[FCTA_MAX_NUM_OBJECTS];
  FCTAVehicleSignalInReq_t FCTAVehicleSig;
  FCTARoadInReq_t FCTARoadInformation;
  FCTACTAGlobalInReq_t CTAGlobleInput;
  FCTACTGlobalInReq_t CTGlobalInput;

} FCTAInReq_t;

typedef struct {
  uint8 obs_move_cycle[FCTA_MAX_NUM_OBJECTS];
} FCTAObjCycle_t;

typedef struct {
  float32 fTTCThreshold;    // 2.5f
  float32 fTTCThresholdL2;  // 1.5f
  // float32 fTTCThresholdL3;      // 2.5f
  float32 fXMinBreakthrough;    // 0.f
  float32 fXMinBreakthroughL2;  // 0.f
  float32 fXMinBreakthroughL3;  // 0.f
  float32 fXMaxBreakthrough;    // 7.f
  float32 fVEgoMax;             // 300.f//100
  float32 fVEgoMin;             //-300.f//0.83
  float32 fVTargetMin;          // 0.82999998f
  float32 fVTargetMax;          // 300.f//100
  float32 fMaxHeadingAngle;     //-60.f
  float32 fMinHeadingAngle;     //-120.f
  float32 fTargetRangeMax;      // 71.f
  float32 fTargetRangeMaxL2;    // 71.f
  // float32 fTargetRangeMaxL3;    // 71.f
  float32 fBreakthroughMargin;  // 1.f
  float32 fTTCThresholdMargin;  // 1.f
  boolean bActive;              // True
} FCTAParam_t;

typedef struct {
  float32 fBTHitHystTimer_s[FCTA_EXTERN_NUM_OF_WARN_LEVELS];
  uint8 uBreakthroughHitConfi[FCTA_EXTERN_NUM_OF_WARN_LEVELS];
  boolean bBreakthroughHit[FCTA_EXTERN_NUM_OF_WARN_LEVELS];
  boolean bWarning[FCTA_EXTERN_NUM_OF_WARN_LEVELS];
  boolean bWarningLastCycle[FCTA_EXTERN_NUM_OF_WARN_LEVELS];
  boolean bObjectInRange[FCTA_EXTERN_NUM_OF_WARN_LEVELS];
  boolean bTTCBelowThresh[FCTA_EXTERN_NUM_OF_WARN_LEVELS];
  boolean bBTHitHystActive[FCTA_EXTERN_NUM_OF_WARN_LEVELS];
  float32 fRearTrackProb;
  uint8 uRearMirrorCnt;
  boolean bRelevant;
  boolean bMirror;
  boolean bObjectFromSide;
  boolean bValidApproachAngle;
  boolean bObjectAtEdgeFoV;
  boolean bShortWarning;
  boolean bUpdatedRecently;  // Flag whether the object is updated recently
  boolean bQuality;
  boolean bRearMirrorObject;
  boolean bObjMovementValid;
} FCTAObjOutput_t;

typedef struct {
  uint32 uiVersionNumber;
  FCTAObjOutput_t FCTA_Va_ObjectListGlobal[FCTA_MAX_NUM_OBJECTS];
  boolean bFCTAWarnActive[FCTA_MAX_NUM_OBJECTS];
  float32 fCriticalTTC_s;
  float32 fCriticalObjDistY_met;
  float32 fCriticalObjDistYLastCycle_met;
  sint32 uCriticalObjID_nu;
  sint32 uCriticalObjIDLastCycle_nu;
  uint8 uInterruptCycleCount_nu;
  boolean bWarningInterrupt;
  float32 fTTCThreshold_s[CTA_FCTA_CFG_NUM_OF_WARN_LEVELS];
  float32 fXMinBreakthrough_met[CTA_FCTA_CFG_NUM_OF_WARN_LEVELS];
  float32 fXMaxBreakthrough_met[CTA_FCTA_CFG_NUM_OF_WARN_LEVELS];
  float32 fMaxObjRange_met[CTA_FCTA_CFG_NUM_OF_WARN_LEVELS];
  float32 fMaxHeadingAngle_deg;  // Calculate the max value of heading angle
  float32 fMinHeadingAngle_deg;  // Calculate the min value of heading angle
  // float32 fCriticalObjDistY_met;
  // float32 fCriticalObjDistYLastCycle_met;
  // uint8 uCriticalObjID_nu;
  // uint8 uCriticalObjIDLastCycle_nu;
  // uint8 uInterruptCycleCount_nu;
  // boolean bWarningInterrupt;
} FCTAOutPro_t;

typedef struct {
  uint32 uiVersionNumber;  // uint32 value example
  // .....
} FCTADebug_t;

void FCTAExec(const FCTAInReq_t* FCTAreqPorts, const FCTAParam_t* FCTAparams,
              FCTAOutPro_t* FCTAproPorts, FCTADebug_t* FCTAdebugInfo);
void FCTAReset();

#ifdef __cplusplus
}
#endif
#endif
