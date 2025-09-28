#ifndef CTA_H
#define CTA_H
#ifdef __cplusplus
extern "C" {
#endif
/*****************************************************************************
 INCLUDES
*****************************************************************************/
#include "cta_par.h"
#include "planning/warning/cta/cta_extern.h"
#include "planning/warning/cta/fcta/cta_fcta_extern.h"
#include "planning/warning/cta/rcta/cta_rcta_extern.h"
#include "planning/warning/lbs/common/tue_common_libs.h"

#define TUE_CML_MacroLowPassFilter(neu, alt, zeit_k) \
  ((neu + alt * zeit_k) / (zeit_k + 1.F))  // Macro for a low filter

/*****************************************************************************
 TYPEDEFS GLOBAL VARIABLE
*****************************************************************************/
typedef enum {
  CTA_RCTA_WARN_LEVEL_ONE,
  CTA_RCTA_WARN_LEVEL_TWO,
  // CTA_RCTA_WARN_LEVEL_THREE
} CTARCTAWarningLevel_t;

typedef enum {
  CTA_REARTRACK_NO,
  CTA_REARTRACK_POS,
  CTA_REARTRACK_POS_SPEED
} CTARearObjState_t;

typedef enum {
  CTA_INIT,  // Initialize all
  CTA_OK,    // normal processing
} CTAState_t;

typedef struct {
  float32 fXMin_met;
  float32 fXMax_met;
  float32 fYMin_met;
  float32 fYMax_met;
} CTAObjectBorder_t;

typedef struct {
  float32 fDistX;
  float32 fDistY;
  float32 fWidth;
  float32 fLength;
} CTAObjectRotated_t;

typedef struct {
  CTAObjectBorder_t ObjectBorder;  // The object border information
  CTAObjectBorder_t
      ObjectMovementBorder;  // The object movement border information
  CTAObjectRotated_t ObjectRotated;
  float32 fTTC_s;
  float32 fTTCAccel_s;
  float32 fTTCFiltered_s;
  float32 fTTCRadial_s;
  float32 fVabs;
  // float32 fRangeRadial_met;
  float32 fXLastCycle_met;
  float32 fYLastCycle_met;
  float32 fVxPosBased_mps;
  float32 fVyPosBased_mps;
  //	float32 fSpeedFiltered;
  float32 fCycleTimeSum_s;  // The object exist life time,unit:s
  float32 fUpdateRate_nu;   // The object measurement update rate,unit:NULL
  float32 fAssocProbFiltered_nu;  // Filtered highest cluster association
                                  // probability of the object filter result
  float32 fXMovement_met;         // The object total moving distance in the x
                                  // direction,unit:m
  float32 fYMovement_met;         // The object total moving distance in the y
                                  // direction,unit:m
  float32 fAngle_deg;
  float32 fObjWidthMax_met;
  float32 fObjLengthMax_met;
  //	uint16 uUniqueID;
  //	uint8 uLastMergedObjID;
  //	boolean bLowTTCAtState;
  //	boolean bCreatedAdjStableObj;
  //	boolean bObjValidForSelection;
  //	boolean bPriolObject;
} CTAObjectInfoGlobal_t;

typedef struct {
  float32 fLeftFrontPos_met;
  float32 fRightFrontPos_met;
  float32 fLeftRearPos_met;
  float32 fRightRearPos_met;
} SensorMountingPosGlobal_t;

typedef struct {
  float32 fXBreakthrough_met;
  float32 fXBreakthroughFiltered_met;
  float32 fXBreakthroughStd_met;
  float32 fTTC_s;                           // TTC of the object
  float32 fTTCFiltered_s;                   // Filtered TTC of the object
  float32 fDistToCrossingLine_met;          // Distance to crossing line
  float32 fDistToCrossingLineFiltered_met;  // Filtered distance to crossing
                                            // line
  float32 fRearTrackProb_per;
  // float32 fObjBreakthroughMargin_met;
  boolean bRearTrack_nu;
} CTObjectInfoGlobal_t;

typedef struct {
  boolean bRelevant;
} CTAFCTAObjectInfoGlobal_t;

typedef struct {
  float32 fCriticalTTC;
  float32 fCriticalObjDistY;
  float32 fCriticalObjDistYLastCycle;
  sint32 uCriticalObjID;
  sint32 uCriticalObjIDLastCycle;
  uint8 uInterruptCycleCount;
  boolean bWarningInterrupt;
  float32 fMaxLatSensorRange;
} CTACTGlobal_t;

typedef struct {
  // uint8 eRoadType; // no use -->JO
  CTAState_t CTAState;
  boolean bFCTAFunctionActive;
  boolean bRCTAFunctionActive;
  boolean bEgoSpeedConditionFCTA;
} CTALastCycleState_t;

typedef struct {
  boolean bRCTAWarnActive[CTA_RCTA_CFG_NUM_OF_WARN_LEVELS];
  float32 fCriticalTTC_s;
  float32 fCriticalObjDistY_met;
  float32 fCriticalObjDistYLastCycle_met;
  sint32 uCriticalObjID_nu;
  sint32 uCriticalObjIDLastCycle_nu;
  uint8 uInterruptCycleCount_nu;
  boolean bWarningInterrupt;
} CTARCTAOutput_t;

typedef struct {
  boolean bFCTAWarnActive[CTA_FCTA_CFG_NUM_OF_WARN_LEVELS];
  float32 fCriticalTTC_s;
} CTAFCTAOutput_t;

typedef struct {
  CTAState_t eCTAState;  // CTA state: init or ok
  SensorMountingPosGlobal_t
      fSensorOffsetToSide_met;  // the offset from sensor mounting position to
                                // the side edge of vehicle
  SensorMountingPosGlobal_t
      fSensorOffsetToRear_met;  // the offset from sensor mounting position to
                                // the front or rear edge of vehicle
  float32 fMaxSpeedOverGround;
  CTALastCycleState_t LastCycleStates;
  CTACTGlobal_t CTGlobals;
  CTAObjectInfoGlobal_t CTAObjectList[CTA_MAX_NUM_OBJECTS];
  CTObjectInfoGlobal_t CTObjectList[CTA_MAX_NUM_OBJECTS];
  CTAFCTAObjectInfoGlobal_t CTAFCTAObjectList[CTA_MAX_NUM_OBJECTS];
  boolean bFCTAFunctionOutput;  // [CTA_FCTA_CFG_NUM_OF_WARN_LEVELS] ;
  boolean bRCTAFunctionOutput;  // [CTA_RCTA_CFG_NUM_OF_WARN_LEVELS] ;
  CTARCTAOutput_t CTARCTAOutput;
  CTAFCTAOutput_t CTAFCTAOutput;
} CTAGlobal_t;

void CTAPreProcess(const CTAInReq_t* reqPorts, const CTAParam_t* params,
                   CTAGlobal_t* pCTAGlobal);
void CTACalculateGlobalProperties(
    const CTAVehicleParam_t* pVehicleParameter,
    SensorMountingPosGlobal_t* pfSensorOffsetToSide_met,
    SensorMountingPosGlobal_t* pfSensorOffsetToRear_met);
void CTACalculateObjectProperties(const CTAInReq_t* reqPorts,
                                  const CTAParam_t* params,
                                  CTAGlobal_t* pCTAGlobal);
void CTACalculateObjectMovementBorders(float32 fDistX_met, float32 fDistY_met,
                                       CTAObjectBorder_t* pObjectMovementBorder,
                                       float32* fXMovement_met,
                                       float32* fYMovement_met);
void CTACalculateObjectQualifiers(uint8 uiHighestAssocProb_per,
                                  uint8 uiMaintenanceState_nu,
                                  float32* fUpdateRate_nu,
                                  float32* fAssocProbFiltered_nu);
void CTACalculatePosBasedVxVy(float32 fCycleTime_s,
                              const EMSRRObjectInReq_t* pEMSRRObjInput,
                              float32 fXLastCycle_met, float32 fYLastCycle_met,
                              float32* pfVxPosBased_mps,
                              float32* pfVyPosBased_mps);
void CTACalculateAbsoluteObjectVelocity(
    const EMSRRObjectInReq_t* pEMSRRObjInput, const CTAParam_t* params,
    const EgoVehicleInReq_t* pEgoVehicleInput, float32* fVabs);
void CTACalculateCTObjectProperties(const CTAInReq_t* reqPorts,
                                    const CTAParam_t* params,
                                    CTAGlobal_t* pCTAGlobal);
void CTA_CTInitCyclic(CTAGlobal_t* pCTAGlobal);
void CTA_CTCalculateMaxLatSensorRange(
    const EMSRRObjectInReq_t* pCTAEMSRRObjList, float32 fegoVelocity_mps,
    float32* pfMaxLatSensorRange);
void CTA_CTCalculateDistToCrossingLine(
    float32 fCycleTime_s, const EMSRRObjectInReq_t* pEMSRRObjInput,
    SensorMountingPosGlobal_t* pfSensorOffsetToSide_met,
    float32* pfDistToCrossingLine_met,
    float32* pfDistToCrossingLineFiltered_met);
void CTA_CTCalculateXBreakthrough(const EMSRRObjectInReq_t* pEMSRRObjInput,
                                  const CTAVehicleParam_t* pVehicleParameter,
                                  float32 fDistToCrossingLine_met,
                                  float32* pfXBreakthrough_met,
                                  float32* pfXBreakthroughStd_met,
                                  float32* pfXBreakthroughFiltered_met);
void CTA_CTCalculateTTC(float32 fCycleTime_s,
                        const eSensorMountingPos_t eSensorMountingPos,
                        const float32 fVrelY_mps,
                        float32 fDistToCrossingLine_met, float32* pfTTC_s,
                        float32* pfTTCFiltered_s);
void CTA_CTCalculateRearObjectProbability(
    uint8 uObj, const EMSRRObjectInReq_t* pCurrObjectEMInput,
    const CTAInReq_t* reqPorts, CTAGlobal_t* pCTAGlobal,
    float32* fRearTrackProb_per);
CTARearObjState_t CTA_CTCalculateSearchFrontObject(
    const EMSRRObjectInReq_t* pCurrObjectEMInput,
    const EMSRRObjectInReq_t* pObjCandEMInfo,
    CTAObjectInfoGlobal_t* pCTAObjCurrGlobal,
    CTAObjectInfoGlobal_t* pCTAObjCandGlobal, float32 fRearTrackProb_per);
float32 CTA_CTCalculateObjectDistance(float32 fObjCurrMin_met,
                                      float32 fObjCurrMax_met,
                                      float32 fObjCandMin_met,
                                      float32 fObjCandMax_met);
void CTA_CTCheckRearObject(uint32 uClassification_nu,
                           float32 fRearTrackProb_per, boolean* bRearTrack_nu);

void CTAToFCTAInputWrapper(const CTAInReq_t* reqPorts, const CTAParam_t* params,
                           CTAGlobal_t* CTAGlobal, FCTAInReq_t* FCTAreqPorts,
                           FCTAParam_t* FCTAparams);
void FCTAToCTAOutputWrapper(FCTAOutPro_t* FCTAproPorts,
                            FCTADebug_t* FCTAdebugInfo, CTAGlobal_t* CTAGlobal,
                            CTADebug_t* debugInfo);
void CTAToRCTAInputWrapper(const CTAInReq_t* reqPorts, const CTAParam_t* params,
                           CTAGlobal_t* CTAGlobal, RCTAInReq_t* RCTAreqPorts,
                           RCTAParam_t* RCTAparams);
void RCTAToCTAOutputWrapper(RCTAOutPro_t* RCTAproPorts,
                            RCTADebug_t* RCTAdebugInfo, CTAGlobal_t* CTAGlobal,
                            CTADebug_t* debugInfo);
void CTAProProcess(const CTAInReq_t* reqPorts, const CTAParam_t* params,
                   CTAGlobal_t* CTAGlobal, CTAOutPro_t* proPorts);
boolean CTAProcessCheckEgoSpeedRange(float32 fEgoSpeed, float32 fMinEgoSpeed,
                                     float32 fMaxEgoSpeed);
boolean CTAProcessSetFuncionOutput(boolean bFunctionEnabled,
                                   boolean bFunctionActive,
                                   boolean bEgoSpeedCondition);
boolean CTAProcessSetWarningOutput(boolean bFunctionOutputEnabled,
                                   boolean bInternalWarning);
void CTAInitObjects();
#ifdef __cplusplus
}
#endif
#endif