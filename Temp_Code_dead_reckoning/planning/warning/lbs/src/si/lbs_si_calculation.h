#pragma once
#ifndef LBS_SI_CALCULATE_H
#define LBS_SI_CALCULATE_H
#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************
        INCLUDES delete
*****************************************************************************/
#include "../lbs_par.h"
#include "lbs_si.h"
#include "lbs_si_ext.h"
#include "lbs_si_par.h"

/*****************************************************************************
        VARIABLES
*****************************************************************************/
extern SICalculate_st SICalculate;

/*****************************************************************************
        CALCULATION FUNCTION PROTOTYPES
*****************************************************************************/
SICalculate_st* GetSICalculatePointer();
SI_Globals_t* GetSICalculatePointer_SIGlobals();
SI_Info_st* GetSICalculatePointer_SIObjInfo(uint8 uObj);
const SILCAObjInfo_t* pSIGetLCAObjInfoPointer(
    uint8 uObj, const SILCAObjInfo_Array* pLCAObjInfoList);
SIGenObject_st* pSIGetGenObjListPointer_Object(uint8 uObj,
                                               SIGenObjList_st* pGenObjList);
// SISRRObject_st* pSIGetSRRObjListPointer_Object(uint8 uObj, EMSRRObjList_st*
// pSRRObjList);
SITrajRefPoint_t SICalculateDistancePoint2Clothoid(float32 fX, float32 fY,
                                                   float32 fC0, float32 fC1,
                                                   float32 fAngle);
void SIApproximateRefpoint(float32 fX, float32 fY, float32 fC0, float32 fC1,
                           float32 fAngle, float32* pfRefX);

float32 SICalcRelTraTrackWidthSeek(const SIVehicleInfo_t* pEgoInfo,
                                   const SIParam_st* params);
float32 SICalcRelTraTrackWidthTrack(const SIVehicleInfo_t* pEgoInfo,
                                    const SIParam_st* params);

void SIAdvantageRelevantObject(
    uint8 uObj, const SIGenObject_st* pGenObj, const SILCAObjInfo_t* pLCAObj,
    const float32 fCycletime_s);  // SISRRObject_st* pSRRObj,
void SIUpdateTimeTrackLaneExtFactor(uint8 uObj, const SIGenObject_st* pGenObj,
                                    const SILCAObjInfo_t* pLCAObj,
                                    float32 fCycleTime);
void SIUpdateDistTrackLaneExtFactor(uint8 uObj, const SIGenObject_st* pGenObj,
                                    const SILCAObjInfo_t* pLCAObj);

void SICalcCriteriaMatrix(uint8 uObj, const SIInPut_st* reqPorts,
                          const SIParam_st* params);
void SIInitCriteriaMatrix(SICriteriaMatrix_t* pCriteraMaxtrix);
void SIInitCriteriaOutput(SIBracketOutput_t* pOutput,
                          SITraceBracketMode_t eMode);
void SICalculateDistance2Traj(float32 fX, float32 fY,
                              const SITrajectoryData_t* pTrajData,
                              SITrajRefPoint_t* pDist2Traj);
void SICalculateVrel2Traj(uint8 uObj, const SIGenObject_st* pGenObj,
                          float32 fDistToTrajLastCycle,
                          float32 fDistToTrajCurrentCycle,
                          float32 fTaskCycleTime);
void SISetBaseSeekWidth(const SIGenObject_st* pGenObj,
                        SICriteriaMatrix_t* pCriteraMaxtrix);
void SISetBracketPosition(const SIGenObject_st* pGenObj, SI_Info_st* pSIObj,
                          SICriteriaMatrix_t* pCriteraMaxtrix);

void SISetBaseTrackWidth(float32* pTrackWidthLeft, float32* pTrackWidthRight,
                         SICriteriaMatrix_t* pCriteraMaxtrix);
void SISetTrackWidthExtension(float32* pTrackWidthExtLeft,
                              float32* pTrackWidthExtRight,
                              SICriteriaMatrix_t* pCriteraMaxtrix);
void SISetTrackWidthRestriction(float32* TrackWidthResRight,
                                float32* TrackWidthResLeft,
                                SICriteriaMatrix_t* pCriteraMaxtrix);
void SISetRightBracket(float32* pfTrackBorderRight, float32 fNewValueRight,
                       SITraceBracketMode_t eBracketMode);
void SISetLeftBracket(float32* pfTrackBorderLeft, float32 fNewValueLeft,
                      SITraceBracketMode_t eBracketMode);

void SICheckTrackCriteria(
    uint8 uObj, const SIGenObject_st* pGenObj, const SIRoad_t* pRoad,
    const SIVehParameter_t* pVehPar, SICriteriaMatrix_t* pCriteraMaxtrix,
    const SIVehicleInfo_t* pEgoInfo, const SI_LCAParameter_st* pLCAParamter,
    const SILCAObjInfo_t* pLCAObjInfo);  // SISRRObject_st* pSRRObj,
void SITrackWidthTrck(uint8 uObj, SICriteriaMatrix_t* pCriteraMaxtrix);
#if (SI_DYNAMIC_LANE_BRACKET_EXTENSION)
void SIExtensionOwnlaneCriticalTTC(
    uint8 uObj, SICriteriaMatrix_t* pCriteraMaxtrix,
    SIGenObject_st* pGenObj);  //, SISRRObject_st* pSRRObj
void SIRestrictionCurve(uint8 uObj, SICriteriaMatrix_t* pCriteraMaxtrix,
                        SIGenObject_st* pGenObj);
void SIExtensionRoadBorder(uint8 uObj, SICriteriaMatrix_t* pCriteraMaxtrix,
                           SIGenObject_st* pGenObj, SIRoad_t* pRoad,
                           SIVehicleInfo_t* pEgoInfo);
void SIRestrictionDistanceDependent(uint8 uObj, SIGenObject_st* pGenObj,
                                    SIRoad_t* pRoad,
                                    SIVehParameter_t* pVehParam,
                                    SICriteriaMatrix_t* pCriteraMaxtrix,
                                    SILCAObjInfo_t* pLCAObjInfo,
                                    SI_LCAParameter_st* pLCAParamter);
#else
void SIExtensionRoadBorder(uint8 uObj, SICriteriaMatrix_t* pCriteraMaxtrix,
                           const SIGenObject_st* pGenObj, const SIRoad_t* pRoad,
                           const SIVehicleInfo_t* pEgoInfo,
                           const SIVehParameter_t* pVehParam,
                           const SI_LCAParameter_st* pLCAParamter);
void SIRestrictionDistanceDependent(uint8 uObj, const SIGenObject_st* pGenObj,
                                    const SIRoad_t* pRoad,
                                    const SIVehParameter_t* pVehParam,
                                    SICriteriaMatrix_t* pCriteraMaxtrix,
                                    const SI_LCAParameter_st* pLCAParamter);
#endif

void SISetTraceBracketOutput(uint8 uObj, const SIGenObject_st* pGenObj,
                             SI_Info_st* pSIObj, const SIParam_st* params);

void SICalculateInlaneOverlap(uint8 uObj, const SIGenObject_st* pGenObj,
                              SI_Info_st* pSIObj, const float32 fMinObjWidth,
                              const float32 fMaxObjWidth,
                              SITrajOccupancy_t* pOccupancy);
void SILaneAsscoiationChange(uint8 uObj, const SIGenObject_st* pGenObj,
                             SI_Info_st* pSIObj,
                             const SILBSObjInfo_st* pLBSObjInfo,
                             SITrajOccupancy_t* pOccupancy,
                             SLACInReq_t reqPorts1, const SIParam_st* params);
boolean SICheckStateFlowOutlaneToInlane(
    uint8 uObj, const SIGenObject_st* pGenObj, SI_Info_st* pSIObj,
    const SILBSObjInfo_st* pLBSObjInfo, SITrajOccupancy_t* pOccupancy,
    SLACInReq_t reqPorts1, const SIParam_st* params);
boolean SICheckStateFlowInlaneToOutlane(
    uint8 uObj, const SIGenObject_st* pGenObj, SI_Info_st* pSIObj,
    const SILBSObjInfo_st* pLBSObjInfo, SITrajOccupancy_t* pOccupancy,
    SLACInReq_t reqPorts1, const SIParam_st* params);
boolean SICheckInlaneOccupancies(const SIGenObject_st* pGenObj,
                                 SI_Info_st* pSIObj,
                                 SITrajOccupancy_t* pOccupancy);
float32 SIGetObjOccPickupThreshold(const SIGenObject_st* pGenObj,
                                   SI_Info_st* pSIObj, float32 fObjWidth);
float32 SIGetLaneOccPickupThreshold(const SIGenObject_st* pGenObj,
                                    SI_Info_st* pSIObj);
float32 SIGetOverlapPickupThreshold(const SIGenObject_st* pGenObj,
                                    SI_Info_st* pSIObj);

boolean SI_CheckCustomInlaneCriteria(uint8 uObj, const SIGenObject_st* pGenObj,
                                     SI_Info_st* pSIObj, SLACInReq_t reqPorts1,
                                     const SIParam_st* params);
boolean SI_CheckPredictedInlaneCriteria(uint8 uObj,
                                        const SIGenObject_st* pGenObj,
                                        SI_Info_st* pSIObj,
                                        SLACInReq_t reqPorts1,
                                        const SIParam_st* params);

float32 SIGetPredictionTime(
    const SIGenObject_st* pGenObj, const float32 fMinPredTime,
    const float32 fMaxPredTime);  // SISRRObject_st* pSRRObj,
float32 SIGetInlaneTimeThreshold(const SIGenObject_st* pGenObj,
                                 const SILBSObjInfo_st* pLBSObjInfo,
                                 const SIVehicleInfo_t* pEgoInfo);
float32 SIGetInlaneDistanceThreshold(const SIGenObject_st* pGenObj,
                                     const SIVehicleInfo_t* pEgoInfo);

boolean SICheckInlaneTimer(const SIGenObject_st* pGenObj,
                           const SILBSObjInfo_st* pLBSObjInfo,
                           SI_Info_st* pSIObj, SIVehicleInfo_t* pEgoInfo,
                           float32 fCycletime);
boolean SICheckInlaneDistance(const SIGenObject_st* pGenObj, SI_Info_st* pSIObj,
                              SIVehicleInfo_t* pEgoInfo, float32 fCycletime);
float32 SIGetObjOccDropThreshold(const SIGenObject_st* pGenObj,
                                 SI_Info_st* pSIObj, SIRoad_t* pRoadInfo,
                                 float32 fCycleTime);
boolean SICheckOutlaneOccupancies(const SIGenObject_st* pGenObj,
                                  SI_Info_st* pSIObj,
                                  SITrajOccupancy_t* pOccupancy,
                                  SIRoad_t* pRoadInfo, float32 fCycleTime);
boolean SI_CheckCustomOutlaneCriteria(uint8 uObj, const SIGenObject_st* pGenObj,
                                      SI_Info_st* pSIObj,
                                      SLACInReq_t reqPorts1);
boolean SI_CheckPredictedOutlaneCriteria(uint8 uObj,
                                         const SIGenObject_st* pGenObj,
                                         SI_Info_st* pSIObj,
                                         SLACInReq_t reqPorts1);
void SIResetInlaneTimer(const SIGenObject_st* pGenObj, SI_Info_st* pSIObj,
                        float32 fCycletime);
void SIResetInlaneDistance(SI_Info_st* pSIObj);
// float32 SIGetObjOccDropThreshold(SIGenObject_st* pGenObj, SI_Info_st* pSIObj,
// SIRoad_t* pRoadInfo, float32 fCycleTime);
float32 SIGetLaneOccDropThreshold(const SIGenObject_st* pGenObj);
float32 SIGetOverlapDropThreshold(const SIGenObject_st* pGenObj,
                                  SI_Info_st* pSIObj, SIRoad_t* pRoadInfo,
                                  float32 fCycleTime);
void SIGetAssociateLane(uint8 uObj, const SIGenObject_st* pGenObj,
                        SI_Info_st* pSIObj);
void SIPostProcess(uint8 uObj, SIOutPut_st* proPorts, SIDebug_st* debugInfo);

#ifdef __cplusplus
}
#endif
#endif