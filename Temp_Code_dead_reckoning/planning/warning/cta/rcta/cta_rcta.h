#ifndef CTA_RCTA_H
#define CTA_RCTA_H
#ifdef __cplusplus
extern "C" {
#endif

#include "cta_rcta_extern.h"
#include "planning/warning/cta/cta_par.h"

typedef enum {
  RCTA_WARN_LEVEL_ONE,
  RCTA_WARN_LEVEL_TWO,
  RCTA_WARN_LEVEL_THREE
} RCTAWarningLevel_t;

typedef struct {
  float32 fBTHitHystTimer_s[CTA_RCTA_CFG_NUM_OF_WARN_LEVELS];  // the warning
      // hysteresis timer
  uint8 uBreakthroughHitConfi[CTA_RCTA_CFG_NUM_OF_WARN_LEVELS];  // Breakthrough
      // hit confidence
  boolean bBreakthroughHit[CTA_RCTA_CFG_NUM_OF_WARN_LEVELS];  // Flag whether
                                                              // the object hits
                                                              // breakthrough
  boolean bWarning[CTA_RCTA_CFG_NUM_OF_WARN_LEVELS];  // Flag whether the RCTA
                                                      // warning is activated
  boolean
      bWarningLastCycle[CTA_RCTA_CFG_NUM_OF_WARN_LEVELS];  // Flag whether the
                                                           // RCTA warning is
                                                           // activated in the
                                                           // last cycle
  boolean bObjectInRange[CTA_RCTA_CFG_NUM_OF_WARN_LEVELS];
  boolean bTTCBelowThresh[CTA_RCTA_CFG_NUM_OF_WARN_LEVELS];
  boolean
      bBTHitHystActive[CTA_RCTA_CFG_NUM_OF_WARN_LEVELS];  // Flag whether the
                                                          // warning hysteresis
  boolean bRelevant;
  boolean bMirror;
  boolean bObjectFromSide;      // Flag whether the object approach from side
  boolean bValidApproachAngle;  // Flag whether the object heading angle is
                                // valid in the current cycle
  boolean bShortTTC;
  boolean bUpdatedRecently;  // Flag whether the object is updated recently
  boolean bQuality;
} RCTAObjGlobal_t;

typedef struct {
  float32 fTTCThreshold_s[CTA_RCTA_CFG_NUM_OF_WARN_LEVELS];
  float32 fXMinBreakthrough_met[CTA_RCTA_CFG_NUM_OF_WARN_LEVELS];  // x-axis min
                                                                   // edge of
  // breakthrough
  float32 fXMaxBreakthrough_met[CTA_RCTA_CFG_NUM_OF_WARN_LEVELS];  // x-axis max
                                                                   // edge of
  // breakthrough
  float32 fMaxObjRange_met[CTA_RCTA_CFG_NUM_OF_WARN_LEVELS];
  float32 fMaxHeadingAngle_deg;  // Calculate the max value of heading angle
  float32 fMinHeadingAngle_deg;  // Calculate the min value of heading angle
  float32 fCriticalTTC_s;
  float32 fCriticalObjDistY_met;
  float32 fCriticalObjDistYLastCycle_met;
  sint32 uCriticalObjID_nu;
  sint32 uCriticalObjIDLastCycle_nu;
  uint8 uInterruptCycleCount_nu;
  boolean bWarningInterrupt;
  boolean bRCTAWarnActive[CTA_RCTA_CFG_NUM_OF_WARN_LEVELS];

  RCTAObjGlobal_t RCTA_Va_ObjectListGlobal[RCTA_MAX_NUM_OBJECTS];
} RCTAGlobal_t;

void RCTAPreProcess(const RCTAInReq_t* RCTAreqPorts,
                    const RCTAParam_t* RCTAparams, RCTAGlobal_t* RCTAGlobal);
void CTARCTASetParameters(const RCTAVehicleSignal_t* RCTAVehicleSig,
                          const RCTAAlgoParam_t* RCTAAlgoParam,
                          RCTAGlobal_t* RCTAGlobal);
float32 RCTACalculateBreakthroughLength(float32 StWheelAngle_rad,
                                        const RCTAAlgoParam_t* RCTAAlgoParam);
void CTARCTAInitCyclic(boolean* bRCTAWarnActive);
void RCTAMainProcess(const RCTAInReq_t* RCTAreqPorts,
                     const RCTAParam_t* RCTAparams, RCTAGlobal_t* RCTAGlobal);
void CTARCTACheckObjectUpdateRecently(const uint8 uiMeasuredTargetFrequency_nu,
                                      boolean* bUpdatedRecently);
void CTARCTACheckObjectInRange(const float32 fDistX_met,
                               const float32 fDistY_met,
                               float32* fMaxObjRange_met,
                               boolean* bObjectInRange);
void CTARCTACheckSideApproach(const boolean bRightSensor,
                              const float32 fFirstDetectY_met,
                              const float32 fDistY_met,
                              boolean* bObjectFromSide);
void CTARCTACheckApproachAngle(const boolean bRightSensor,
                               const float32 fRelHeading_rad,
                               const float32 fRelHeadingStd_rad,
                               float32 fMaxHeadingAngle_deg,
                               float32 fMinHeadingAngle_deg,
                               boolean bObjectFromSide,
                               boolean* bValidApproachAngle);
void CTARCTACheckForMirror(const float32 fMirrorProb_per,
                           boolean* bObjectFromSide);
void CTARCTACheckObjectRelevance(const float32 fVabsX_mps,
                                 const float32 fVabsY_mps, float32 fVTargetMin,
                                 const uint16 uiLifeCycles_nu,
                                 float32 fXMovement_met, float32 fYMovement_met,
                                 boolean* bRelevant);
void CTARCTACheckObjectQuality(const RCTAEMSRRObjInReq_t* pEMSRRObjInput,
                               float32 fUpdateRate_nu,
                               float32 fAssocProbFiltered_nu,
                               boolean* bQuality);
void CTARCTACheckBreakthroughHit(float32* fXMaxBreakthrough_met,
                                 float32* fXMinBreakthrough_met,
                                 float32 fDistToCrossingLine_met,
                                 float32 fXBreakthrough_met, boolean* bWarning,
                                 boolean* pbBreakthroughHit);
void CTARCTACalculateBreakthroughHitConfidence(float32 fXBreakthrough_met,
                                               float32 fXBreakthroughStd_met,
                                               float32* fXMaxBreakthrough_met,
                                               float32* fXMinBreakthrough_met,
                                               boolean* bBreakthroughHit,
                                               uint8* uBreakthroughHitConfi);
float32 CTARCTACalculateHitLimit(float32 fBreakthroughLimit,
                                 float32 fXMaxBreakthrough,
                                 float32 fXMinBreakthrough,
                                 boolean bBreakthroughHit);
void CTARCTAUpdateBTHitHysteresisTimer(float32 fCycleTime_s, float32 fTTC_s,
                                       float32 fTTCFiltered_s,
                                       boolean* bWarning,
                                       boolean* bBreakthroughHit,
                                       boolean* bWarningLastCycle,
                                       float32* fBTHitHystTimer_s);
void CTARCTACheckBTHitHysteresisTimer(float32* fBTHitHystTimer_s,
                                      boolean* bBTHitHystActive);
void CTARCTACheckTTC(float32 fTTC_s, float32 fTTCFiltered_s,
                     float32* fTTCThreshold_s, boolean* bTTCBelowThresh);
void CTARCTACheckShortWarning(float32 fTTC_s, float32 fTTCFiltered_s,
                              boolean* bShortTTC);
#ifdef ST_PERCEPTION
void CTARCTAWarningDecision(float32 fRCS, RCTAObjGlobal_t* pRCTAObjGlobal,
                            boolean bRearTrack_nu, boolean* bWarningLastCycle,
                            boolean* pbWarning);
#endif

void CTARCTAWarningDecision(uint8 uGearReverse, RCTAObjGlobal_t* pRCTAObjGlobal,
                            boolean bRearTrack_nu, boolean bRCTASixDoorsClosed,
                            boolean* pbWarning);

void CTARCTASetGlobalWarning(uint8 uObj, const float32 fDistY_met,
                             const float32 fWidthLeft_met, boolean* bWarning,
                             float32 fTTCFiltered_s, RCTAGlobal_t* RCTAGlobal);
void CTARCTACheckMultiObjectInterrupt(const RCTAInReq_t* RCTAreqPorts,
                                      RCTAGlobal_t* RCTAGlobal);
void RCTAProProcess(RCTAGlobal_t* RCTAGlobal, RCTAOutPro_t* RCTAproPorts,
                    RCTADebug_t* RCTAdebugInfo);

#ifdef __cplusplus
}
#endif
#endif
