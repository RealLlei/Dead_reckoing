#ifndef CTA_FCTA_H
#define CTA_FCTA_H
#ifdef __cplusplus
extern "C" {
#endif

#include "cta_fcta_extern.h"
#include "planning/warning/cta/cta_par.h"

typedef enum {
  FCTA_WARN_LEVEL_ONE,
  FCTA_WARN_LEVEL_TWO,
  FCTA_WARN_LEVEL_THREE
} FCTAWarningLevel_t;

typedef struct {
  float32 fBTHitHystTimer_s[CTA_FCTA_CFG_NUM_OF_WARN_LEVELS];
  uint8 uBreakthroughHitConfi[CTA_FCTA_CFG_NUM_OF_WARN_LEVELS];
  boolean bBreakthroughHit[CTA_FCTA_CFG_NUM_OF_WARN_LEVELS];
  boolean bWarning[CTA_FCTA_CFG_NUM_OF_WARN_LEVELS];
  boolean bWarningLastCycle[CTA_FCTA_CFG_NUM_OF_WARN_LEVELS];
  boolean bObjectInRange[CTA_FCTA_CFG_NUM_OF_WARN_LEVELS];
  boolean bTTCBelowThresh[CTA_FCTA_CFG_NUM_OF_WARN_LEVELS];
  boolean bBTHitHystActive[CTA_FCTA_CFG_NUM_OF_WARN_LEVELS];
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
} FCTAObjGlobal_t;

typedef struct {
  float32 fTTCThreshold_s[CTA_FCTA_CFG_NUM_OF_WARN_LEVELS];
  float32 fXMinBreakthrough_met[CTA_FCTA_CFG_NUM_OF_WARN_LEVELS];
  float32 fXMaxBreakthrough_met[CTA_FCTA_CFG_NUM_OF_WARN_LEVELS];
  float32
      fMaxObjRange_met[CTA_FCTA_CFG_NUM_OF_WARN_LEVELS];  // the max threshold
                                                          // of object range
  float32 fMaxHeadingAngle_deg;
  float32 fMinHeadingAngle_deg;
  float32 fVegoMin_mps;
  float32 fVegoMax_mps;
  float32 fVTargetMin;
  float32 fVTargetMax;
  float32 fCriticalTTC_s;
  float32 fCriticalObjDistY_met;
  float32 fCriticalObjDistYLastCycle_met;
  sint32 uCriticalObjID_nu;
  sint32 uCriticalObjIDLastCycle_nu;
  uint8 uInterruptCycleCount_nu;
  boolean bWarningInterrupt;
  boolean bFCTAWarnActive[CTA_FCTA_CFG_NUM_OF_WARN_LEVELS];

  FCTAObjGlobal_t FCTA_Va_ObjectListGlobal[FCTA_MAX_NUM_OBJECTS];

} FCTAGlobal_t;

void FCTAPreProcess(const FCTAInReq_t* FCTAreqPorts,
                    const FCTAParam_t* FCTAparams, FCTAGlobal_t* FCTAGlobal);
void CTAFCTASetParameters(const float32 fegoVelocity,
                          const float32 fCurveRadius,
                          FCTAStateInReq_t FCTALastCycleState,
                          float32 fSensorOffsetToRear_met,
                          const FCTAParam_t* FCTAparams,
                          FCTAGlobal_t* pFCTAGlobal);
void CTAFCTAInitCyclic(boolean* bFCTAWarnActive);
void FCTAMainProcess(const FCTAInReq_t* FCTAreqPorts,
                     const FCTAParam_t* FCTAparams, FCTAGlobal_t* FCTAGlobal);
void CTAFCTACheckObjectInRange(float32 fDistX_met, float32 fDistY_met,
                               float32* fMaxObjRange_met,
                               boolean* bObjectInRange);
void CTAFCTACheckObjectUpdateRecently(uint8 uiMeasuredTargetFrequency_nu,
                                      float32 fUpdateRate_nu,
                                      boolean* bUpdatedRecently);
void CTAFCTACheckForMirror(float32 fMirrorProb_per, boolean* pbMirror);
void CTAFCTACheckObjectRelevance(
    uint8 Obj_order, const FCTAEMSRRObjInReq_t* pEMSRRObjInput,
    float32 fegoVelocity_mps, const FCTACTAObjListInReq_t* pFCTACTAObjInput,
    const FCTACTObjInReq_t* pFCTACTObjInput, FCTAGlobal_t* pFCTAGlobal,
    float32 fMaxLatSensorRange, boolean* pbWarning, boolean bObjMovementValid,
    boolean* pbRelevant);
void CTAFCTACheckObjectQuality(const FCTAEMSRRObjInReq_t* pEMSRRObjInput,
                               float32 fegoVelocity, float32 fUpdateRate_nu,
                               float32 fAssocProbFiltered_nu,
                               boolean* pbQuality);
void CTAFCTACalculateBreakthroughHitConfidence(float32 fXBreakthrough_met,
                                               float32 fXBreakthroughStd_met,
                                               FCTAGlobal_t* pFCTAGlobal,
                                               const FCTAInReq_t* FCTAreqPorts,
                                               boolean bObjectAtEdgeFoV,
                                               boolean* pbBreakthroughHit,
                                               uint8* puBreakthroughHitConfi);
float32 CTAFCTACalculateHitLimit(float32 fBreakthroughLimit,
                                 float32 fXMaxBreakthrough,
                                 float32 fXMinBreakthrough,
                                 boolean bBreakthroughHit);
void CTAFCTACheckApproachAngle(const boolean bRightSensor,
                               const float32 fAbsOrientation_rad,
                               boolean* pbWarning, float32 fMaxHeadingAngle_deg,
                               float32 fMinHeadingAngle_deg,
                               boolean bObjectFromSide,
                               boolean* bValidApproachAngle);
void CTAFCTAObjectMovement(const FCTAEMSRRObjInReq_t* pEMSRRObjInput,
                           float32 fVabs, float32 fVxPosBased,
                           float32 fVyPosBased, boolean* pbObjMovementValid);
void FCTAProProcess(FCTAGlobal_t* FCTAGlobal, FCTAOutPro_t* FCTAproPorts,
                    FCTADebug_t* FCTAdebugInfo);
void CTAFCTACheckBreakthroughHit(float32* fXMaxBreakthrough_met,
                                 float32* fXMinBreakthrough_met,
                                 float32 fDistToCrossingLine_met,
                                 float32 fXBreakthrough_met, boolean* pbWarning,
                                 boolean* pbBreakthroughHit);
void CTAFCTAUpdateBTHitHysteresisTimer(float32 fCycleTime_s, float32 fTTC_s,
                                       float32 fTTCFiltered_s,
                                       boolean* bWarning,
                                       boolean* bBreakthroughHit,
                                       boolean* bWarningLastCycle,
                                       float32* pfBTHitHystTimer_s);
void CTAFCTACheckBTHitHysteresisTimer(float32* fBTHitHystTimer_s,
                                      boolean* pbBTHitHystActive);
void CTAFCTACheckTTC(float32 fTTC_s, float32 fTTCFiltered_s,
                     float32* pfTTCThreshold_s, boolean bRightSensor,
                     boolean bObjectAtEdgeFoV, float32 fDistY_met,
                     boolean* pbTTCBelowThresh);
void CTAFCTACheckShortWarning(float32 fTTC_s, float32 fTTCFiltered_s,
                              float32 fDistToCrossingLine_met,
                              boolean* bWarning, boolean* pbShortWarning);
#ifdef ST_PERCEPION

void CTAFCTAWarningDecision(FCTAObjGlobal_t* pFCTAObjGlobal, float32 fRCS,
                            boolean bRearTrack_nu, boolean* pbWarningLastCycle,
                            boolean* pbWarning);
#endif  // ST_PERCEPION
void CTAFCTAWarningDecision(FCTAObjGlobal_t* pFCTAObjGlobal, uint8 uGearDrive,
                            boolean bRearTrack_nu, boolean bFCTASixDoorsClosed,
                            boolean* pbWarning);
void CTAFCTASetGlobalWarning(uint8 uObj, const float32 fDistY_met,
                             const float32 fWidthLeft_met, boolean* pbWarning,
                             float32 fTTCFiltered_s, FCTAGlobal_t* pFCTAGlobal);

#ifdef __cplusplus
}
#endif
#endif
