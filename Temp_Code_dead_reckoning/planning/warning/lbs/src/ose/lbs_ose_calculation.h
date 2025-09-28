#ifndef LBS_OSE_CALCULATION_H
#define LBS_OSE_CALCULATION_H
#include "planning/warning/lbs/common/TM_Global_Types.h"
#ifdef __cplusplus
extern "C" {
#endif
/*****************************************************************************
        INCLUDES
*****************************************************************************/
#include "lbs_ose_ext.h"
#include "lbs_ose_par.h"
#include "planning/warning/lbs/common/tue_common_libs.h"
#include "planning/warning/lbs/src/lbs_par.h"
#define OSEMirrorRightToLeft(x) (OSEReqPorts.)

/*****************************************************************************
        TYPEDEFS : INPUT   PARAMETER    OUTPUT    DEBUG
*****************************************************************************/
typedef enum { OSE_Init, OSE_OK } OSERunStateGlobal_t;

typedef enum {
  OSE_FRONTOBJ_NO,
  OSE_FRONTOBJ_POS,
  OSE_FRONTOBJ_POS_SPEED
} OSEFrontObjState_t;

typedef enum {
  OSE_WARN_LEVEL_ONE,
  OSE_WARN_LEVEL_TWO,
  OSE_WARN_LEVEL_THREE
} OSEWarningLevel_t;

typedef struct {
  float32 fYMinBreakthrough[OSE_NUM_OF_BREAK_LINES];  // y-axis min edge of
                                                      // breakthrough
  float32 fYMaxBreakthrough
      [OSE_NUM_OF_BREAK_LINES];  // y-axis max edge of                                                     // breakthrough
} OSE_ParameterLevelGlobal_t;

typedef struct {
  float32 fCriticalTTC;  // Critical object's TTC
  float32 fCriticalObjDistX;
  float32 fCriticalObjDistXLastCycle;
  sint32 uCriticalObjID;
  sint32 uCriticalObjIDLastCycle;
  uint8 uInterruptCycleCount;
  boolean bWarningInterrupt;  // Flag whether OSE warning is interrupped
} OSE_MultiObjGlobal_t;

typedef struct {
  float32 fBTHitHystTimer;  // the warning hysteresis timer
  uint8 uBreakthroughHitConfi[OSE_NUM_OF_BREAK_LINES];  // Breakthrough hit
                                                        // confidence
  boolean bBreakthroughHit[OSE_NUM_OF_BREAK_LINES];  // Flag whether the object
                                                     // hits breakthrough
  boolean bWarning;           // Flag whether the OSE warning is activated
  boolean bWarningLastCycle;  // Flag whether the OSE warning was activated in
                              // the last cycle
  boolean bObjectInRange;  // Flag whether the object is in the range of three
                           // different level
  boolean bTTCBelowThresh[OSE_NUM_OF_BREAK_LINES];  // Flag whether the object's
      // TTC is below TTC Threshold
  boolean bBTHitHystActive;  // Flag whether the warning hysteresis
} OSEObjInfoLevel_t;

typedef struct {
  uint16 uCounters[3];
  float32 fValue_met;  // The object width in the algorithm
} OSEWidthEstim_t;

typedef struct {
  OSEObjInfoLevel_t
      InfoLevel[OSE_NUM_OF_WARN_LEVELS];  // DOW warning information structure
  float32 fYBreakthrough[OSE_NUM_OF_BREAK_LINES];  // The y-axis breakthrough of
                                                   // the object
  float32 fYBreakthroughStd[OSE_NUM_OF_BREAK_LINES];  // The y-axis breakthrough
                                                      // standard deviation of
                                                      // the object
  float32 fTTC_s[OSE_NUM_OF_BREAK_LINES];             // TTC of the object
  float32 fTTCFiltered_s[OSE_NUM_OF_BREAK_LINES];  // Filtered TTC of the object
  float32 fDistToCrossingLine_met[OSE_NUM_OF_BREAK_LINES];  // Distance between
      // object front edge and
      // ego vehicle rear edge
  float32 fSideTrackProb;  // The probability of the side object
  // float32 fObjBreakthroughMargin[OSE_NUM_OF_BREAK_LINES];
  boolean bRelevant;  // Flag whether the object is relevant for a OSE warning
  boolean bMirror;    // Flag whether the object is mirror
  boolean bSideTrack;
  boolean bObjectFromRear;  // Flag whether the object is approaching from the
                            // rear in the current cycle or first time
  boolean bValidApproachAngle;  // Flag whether the object heading angle is
                                // valid
  boolean bObjectAtEdgeFoV;     // Flag whether the object is at the edge of FOV
  boolean bShortTTC;
  boolean bUpdatedRecently;  // Flag whether the object is updated recently
  float32 fQuality;  // The quality probability of the DOW warning object
  OSEWidthEstim_t fEstWidth;
} OSEObjInfoArrayGlobal_t;

typedef enum {
  LBS_OSE_HYP_UNKNOWN,
  LBS_OSE_HYP_PARKING,
  LBS_OSE_HYP_DRIVING,
  LBS_OSE_HYP_NUM
} OSEDrvHyp_t;

/*typedef enum
{
        OSE_STATE_PASSIVE,
        OSE_STATE_ACTIVE,
        OSE_STATE_LOCKED,
        OSE_NUM_WARN_STATES
}OSE_WARN_STATE_t;*/

typedef struct {
  float32 fLeftSensorOffsetToSide_met;  // Distance between mounting position of
                                        // the left sensor and vehicle left side
  float32
      fRightSensorOffsetToSide_met;  // Distance between mounting position of
                                     // the right sensor and vehicle right side
  float32 fLeftSensorOffsetToRear_met;  // Distance between mounting position of
                                        // the left sensor and vehicle rear side
  float32
      fRightSensorOffsetToRear_met;  // Distance between mounting position of
                                     // the right sensor and vehicle rear side
  OSERunStateGlobal_t eOSEState;
  OSEObjInfoArrayGlobal_t OSEObjInfoArray[OSE_LBS_NUM_OBJECTS];
  OSE_ParameterLevelGlobal_t
      ParameterLevel[OSE_NUM_OF_WARN_LEVELS];  // The max and min edges of
                                               // breakthrough parameter
  OSE_MultiObjGlobal_t MultiObj;               // Critical object's information
  boolean bOSEWarnActive[OSE_NUM_OF_WARN_LEVELS];  // Flag whether OSE warning
                                                   // is activated
  // OSEDrvHyp_t SelDrvHyp;
  // float32 DrvHyp[LBS_OSE_HYP_NUM];
  // float32 WarningTimer[OSE_NUM_OF_WARN_LEVELS];
  // OSE_WARN_STATE_t OSEWarnState[OSE_NUM_OF_WARN_LEVELS];
  // uint8 OccupantExitRisk[OSE_DOOR_LOCKS_NUMBER];
  // boolean ExitRiskLeftSide;
  // boolean ExitRiskRightSide;
} OSEGlobal_t;

/*****************************************************************************
        VARIABLES
*****************************************************************************/

/*****************************************************************************
        FUNCTION PROTOTYPES
*****************************************************************************/

void LBSOSESetParameters(const OSEParam_t* params,
                         float32* pfLeftSensorOffsetToSide_met,
                         float32* pfRightSensorOffsetToSide_met,
                         float32* fLeftSensorOffsetToRear_met,
                         float32* fRightSensorOffsetToRear_met);
void LBSOSEInitCyclic(OSEGlobal_t* pOSEGlobal);
void LBSOSECalculateYBreakthroughLimit(
    boolean bRightSensor, float32 fLeftSensorOffsetToSide_met,
    float32 fRightSensorOffsetToSide_met, const OSEParam_t* params,
    OSE_ParameterLevelGlobal_t* pParameterLevel);
void LBSOSECalculateDistToCrossingLine(boolean bRightSensor, float32 fDistX_met,
                                       float32 fLeftSensorOffsetToRear_met,
                                       float32 fRightSensorOffsetToRear_met,
                                       float32 fXBreakthroughLine_met,
                                       float32 fObjLengthFront_met,
                                       float32* fDistToCrossingLine_met);
void LBSOSEEstimate_Width(float32 fGenObjWidth_met, float32 fRCS,
                          uint16* uCounters, float32* fValue_met);
void LBSOSECalculateYBreakthrough(const OSEGenObjectInReq_t* pEMGenObjInReq,
                                  OSEObjInfoArrayGlobal_t* pOSEObjGlobal,
                                  const uint8 uline);
#ifdef ST_PERCEPTION
void LBSOSECalculateTTC(float32 fCycletime_s, float32 fVrelX_mps,
                        float32 fDistToCrossingLine_met, float32* fTTC_s,
                        float32* fTTCFiltered_s);
#endif
void LBSOSECalculateTTC(float32 fCycletime_s, float32 fDistX_met,
                        float32 fVrelX_mps, float32 fDistToCrossingLine_met,
                        float32* fTTC_s, float32* fTTCFiltered_s);
void LBSOSECalculateFrontObjectProbability(uint8 uObj,
                                           const OSEInReq_t* reqPorts,
                                           OSEGlobal_t* pOSEGlobal,
                                           float32* pfSideTrackProb);
OSEFrontObjState_t LBSOSECheckFrontOject(
    const OSEGenObjectInReq_t* pEMObjInfo,
    const OSEGenObjectInReq_t* pEMFrontCandObjInfo,
    const OSELBSObjBorders_t* pObjBorder,
    const OSELBSObjBorders_t* pFrontCandObjBorder,
    float32 fSideTrackProbability);
float32 LBSOSECalculateObjectDistance(float32 fMinObj, float32 fMaxObj,
                                      float32 fMinFrontCandObj,
                                      float32 fMaxFrontCandObj);
void LBSOSECheckSideObject(float32 fSideTrackProb, boolean* bSideTrack);
void LBSOSECheckObjectUpdateRecently(uint8 uiMeasuredTargetFrequency_nu,
                                     boolean* bUpdatedRecently);
void LBSOSECheckObjectInRange(float32 fDistX_met, float32 fDistY_met,
                              float32 fXMaxTargetRange_met,
                              const float32 fXMinTargetRange_met,
                              OSEObjInfoLevel_t* pInfoLevel);
void LBSOSECheckRearApproach(float32 fDistX_met, float32 fFirstDetectX_met,
                             boolean* bObjectFromRear);
void LBSOSECheckApproachAngle(float32 fAbsOrientation_rad,
                              boolean bObjectFromRear, float32 fMaxHeadingAngle,
                              float32 fMinHeadingAngle,
                              boolean* bValidApproachAngle);
void LBSOSECheckForMirror(float32 fMirrorProb_per, boolean* bMirror);
void LBSOSECheckObjectRelevance(float32 fXMovement_met, float32 fYMovement_met,
                                float32 fVabsX_mps, float32 fVabsY_mps,
                                uint16 uiLifeCycles_nu, float32 fVTargetMin,
                                boolean* bRelevant);
void LBSOSECheckObjectQuality(const OSEGenObjectInReq_t* pEMGenObjInReq,
                              float32 fUpdateRate_nu,
                              float32 fAssocProbFiltered, float32* pfQuality);
void LBSOSECheckBreakthroughHit(const uint8 uLine,
                                float32* fDistToCrossingLine_met,
                                float32* fYBreakthrough,
                                OSE_ParameterLevelGlobal_t* pParameterLevel,
                                OSEObjInfoLevel_t* pInfoLevel);
void LBSOSECalculateBreakthroughHitConfidence(
    const uint8 uLine, float32* pfYBreakthrough, float32* pfYBreakthroughStd,
    boolean bObjectAtEdgeFoV, OSE_ParameterLevelGlobal_t* pParameterLevel,
    OSEObjInfoLevel_t* pInfoLevel);
float32 LBSOSECalculateHitLimit(float32 fBreakthroughLimit,
                                float32 fYMaxBreakthrough,
                                float32 fYMinBreakthrough,
                                boolean bBreakthroughHit);
void LBSOSEUpdateBTHitHysteresisTimer(float32 fCycletime, float32 fTTC0_s,
                                      float32 fTTCFiltered0_s,
                                      OSEObjInfoLevel_t* pInfoLevel);
void LBSOSECheckBTHitHysteresis(OSEObjInfoLevel_t* pInfoLevel);
void LBSOSECheckTTC(const uint8 uLine, float32 fTTC_s, float32 fTTCFiltered_s,
                    boolean bObjectAtEdgeFoV, const float32* pfTTCThreshold_s,
                    OSEObjInfoLevel_t* pInfoLevel);
void LBSOSECheckShortWarning(float32 fTTC_s, float32 fTTCFiltered_s,
                             boolean* bShortTTC);
#ifdef ST_PERCEPTION

void LBSOSEWarningDecision(float32 fRCS, OSEObjInfoArrayGlobal_t* pOSEObjGlobal,
                           OSEObjInfoLevel_t* pInfoLevel);
#endif

void LBSOSEWarningDecision(OSEObjInfoArrayGlobal_t* pOSEObjGlobal,
                           OSEObjInfoLevel_t* pInfoLevel);

void LBSOSESetGlobalWarning(uint8 uObj, float32 fDistX_met, float32 fLengthRear,
                            float32 fTTCFiltered_s,
                            OSEObjInfoArrayGlobal_t* pOSEObjGlobal,
                            OSE_MultiObjGlobal_t* pMultiObj,
                            boolean* pbOSEWarnActive);
void LBSOSECheckMultiObjectInterrupt(const OSEGenObjArrayInReq* pEMObjArray,
                                     OSE_MultiObjGlobal_t* pMultiObj);

#ifdef __cplusplus
}
#endif

#endif