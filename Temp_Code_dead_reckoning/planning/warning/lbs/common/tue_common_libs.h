#pragma once
#ifndef TUE_COMMON_LIBS_H
#define TUE_COMMON_LIBS_H
#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>
#include <string.h>

#include "TM_Global_Types.h"
/*! \file **********************************************************************

  PROJECT:                   COMMON

  CPU:                       CPU-Independent

  COMPONENT:                 COMMON TOOLS

  MODULNAME:                 tue_common_libs.h

  DESCRIPTION:               common libs for tue development

  AUTHOR:                    $Author: tao.guo

  CREATION DATE:             $Date: 2020/07/04

  VERSION:                   $Revision: 1.0.0


  CHANGES:
  ---*/ /*---
	  CHANGE:                    $Log: tue_common_libs.h
	  CHANGE:                    Initial version

	**************************************************************************** */

/*****************************************************************************
  INCLUDES
*****************************************************************************/

/*****************************************************************************
  DEBUG PRINT
*****************************************************************************/
//#define DEBUG

#include <stdio.h>
#define DEBUG_Print(fmt, ...) \
  fprintf(stderr, "DebugPrint(%s:%d):\t" fmt, __func__, __LINE__, ##__VA_ARGS__)

/*****************************************************************************
  SYMBOLIC CONSTANTS
*****************************************************************************/
typedef enum tue_switch_state {
  TUE_SWITCH_STATE_OFF,       /*!<TUE_SWITCH_STATE_OFF*/
  TUE_SWITCH_STATE_ON,        /*!<TUE_SWITCH_STATE_ON*/
  TUE_SWITCH_STATE_ACTION_OFF /*!< Special return value (3state): switch action
                                 but false condition */
} tue_switch_state_t;

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/
#ifndef glob_type_H
// typedef unsigned char Boolean;
typedef unsigned char uint8;
typedef unsigned short uint16;
typedef unsigned int uint32;
typedef signed int sint32;
typedef float float32;
typedef uint8 AlgoSignalState_t;
typedef uint16 AlgoCycleCounter_t;
typedef uint32 AlgoDataTimeStamp_t;
typedef unsigned int ubit32_t;
typedef unsigned char boolean;
#else
typedef unsigned int ubit32_t;
#endif

typedef struct {
  float32 f0;  // First Element of the 2D vector
  float32 f1;  // Second Element of the 2D vector
} TUE_CML_Vector2D_t;

typedef struct TUE_CML_PolyResult {
  float32 fC0;     /*!<fC0 */
  float32 fC1;     /*!<fC1 */
  float32 fC2;     /*!<fC2 */
  boolean isValid; /*!<isValid */
} TUE_CML_PolyResult_t;

typedef struct TUE_CML_Switch_t {
  ubit32_t AKT_STATUS : 1;           /*!<AKT_STATUS*/
  ubit32_t LAST_STATUS : 1;          /*!<LAST_STATUS*/
  ubit32_t OK_WHILE_SWITCHED_ON : 1; /*!<OK_WHILE_SWITCHED_ON*/
  ubit32_t CYCLE_TIMER : 9;
  /*!< max cycle timer 510 cycles (=10s at 20ms cycles)*/ /*%unit:cycles*/
  ubit32_t DURATION_TIME_INACTIVE : 10;
  /*!< max duration measurement 1022 cycles (=20s at 20ms cycles)*/ /*%unit:cycles*/
  ubit32_t DURATION_TIME_ACTIVE : 10;
  /*!< max duration measurement 1022 cycles (=20s at 20ms cycles)*/ /*%unit:cycles*/
} TUE_CML_Switch_t;

typedef struct {
  float32 dAmin;  // Minimum output value

  float32 dAmax;  // Maximum ouput value

  float32 dM;  // Slope of the line (Amax-Amin)/(Emax-Emin)

  float32 dB;  // Intercept value of the line (Amax-Amin)/(Emax-Emin) * Emin
} TUE_CML_t_LinFunctionArgs;

/*****************************************************************************
  CONSTS
*****************************************************************************/
#define TUE_C_F32_VALUE_INVALID (1000.0F)
#define TUE_C_UI16_VALUE_MAX (65530u)
#define TUE_C_UI16_VALUE_INVALID (65535u)
#define TUE_C_UI8_VALUE_MAX (250u)
#define TUE_C_UI8_VALUE_INVALID (255u)
#define TUE_C_I8_VALUE_INVALID (127)
#define TUE_C_SIXTH (1.0f / 6.0F)
#define TUE_C_MS_KMH ((float32)0.277778F)

#define TUE_C_F32_DELTA ((float32)0.0001F)
#define TUE_C_F32_EXT_DELTA 1e-8f
#define TUE_CML_SqrtApprox_NumExpo (23u)
#define TUE_CML_SqrtApprox_MantissaMask (0x007fffffu)
#define TUE_CML_SqrtApprox_ExponentOffset (0x7F)
#define TUE_CML_SqrtApprox_AlmostZero (1e-20F)
#define TUE_CML_GaussianCDFMinSigma 0.000001f
#define TUE_CML_SQRT_OF_2 (1.414213562373095F) /* square root of 2 */
#define TUE_CML_AlmostZero (1e-15F)
#define TUE_CML_AlmostNegZero (-1e-15F)
#define TUE_CML_GaussErrFctMaxX 1.99f
#define TUE_CML_GaussErrFctConst0 0.002289f
#define TUE_CML_GaussErrFctConst1 1.146f
#define TUE_CML_GaussErrFctConst2 0.1092f
#define TUE_CML_GaussErrFctConst3 0.2841f
#define TUE_CML_GaussErrFctConst4 0.08869f
#define TUE_CML_Pi 3.14159265359f
#define TUE_CML_ModuloEps 0.0000001f
#define TUE_CML_LONG_MAX 2147483647L
#define TUE_CML_TAN_56_C1 (-3.16783027F)
#define TUE_CML_TAN_56_C2 0.13451612F
#define TUE_CML_TAN_56_C3 (-4.03332198F)
#define TUE_CML_SWITCH_CYCLETIMER_INIT 511
#define TUE_CML_SWITCH_TIME_MAX 1023
/*! magic constant no. 1 for calculating cos() with 3.2 decimals of accuracy */
#define C_COS_32_C1 0.99940307f
/*! magic constant no. 2 for calculating cos() with 3.2 decimals of accuracy */
#define C_COS_32_C2 (-0.49558072f)
/*! magic constant no. 3 for calculating cos() with 3.2 decimals of accuracy */
#define C_COS_32_C3 0.03679168f
/*! magic constant no. 1 for calculating tan() with 3.2 decimals of accuracy */
#define C_TAN_32_C1 (-3.6112171f)
/*! magic constant no. 2 for calculating tan() with 3.2 decimals of accuracy */
#define C_TAN_32_C2 (-4.6133253f)
/*! magic constant no. 1 for calculating cos() with 5.2 decimals of accuracy */
#define C_COS_52_C1 0.99999329F
/*! magic constant no. 2 for calculating cos() with 5.2 decimals of accuracy */
#define C_COS_52_C2 (-0.49991243F)
/*! magic constant no. 3 for calculating cos() with 5.2 decimals of accuracy */
#define C_COS_52_C3 0.04148774F
/*! magic constant no. 4 for calculating cos() with 5.2 decimals of accuracy */
#define C_COS_52_C4 (-0.00127120F)
/*! optimized TAN-algorithm */
/*! magic constant no. 1 for calculating tan() with 5.6 decimals of accuracy */
#define C_TAN_56_C1 (-3.16783027F)
/*! magic constant no. 2 for calculating tan() with 5.6 decimals of accuracy */
#define C_TAN_56_C2 0.13451612F
/*! magic constant no. 3 for calculating tan() with 5.6 decimals of accuracy */
#define C_TAN_56_C3 (-4.03332198F)
/*! optimized ArTAN-algorithm */
/*! magic constant no. 1 for calculating atan() with 6.6 decimals of accuracy */
#define C_ATAN_66_C1 1.68676291F
/*! magic constant no. 2 for calculating atan() with 6.6 decimals of accuracy */
#define C_ATAN_66_C2 0.43784973F
/*! magic constant no. 3 for calculating atan() with 6.6 decimals of accuracy */
#define C_ATAN_66_C3 1.68676331F
/*! pi/6.0, used in atan routines     */
#define C_SIXTHPI (TUE_CML_Pi / 6.0F)
/*! tan(pi/6), used in atan routines  */
#define C_TANSIXTHPI 0.57735026F
/*! tan(pi/12), used in atan routines */
#define C_TANTWELFTHPI 0.26794919F
/*****************************************************************************
  MACROS
*****************************************************************************/

#define TUE_CML_Sqr(x) ((x) * (x))
#define TUE_CML_Sign(x) (((x) == (0)) ? (0) : (((x) > (0)) ? (1) : (-1)))
#define TUE_CML_Abs(x) (((x) < (0L)) ? (-(x)) : (x))
#define TUE_CML_Min(x, y) (((x) < (y)) ? (x) : (y))
#define TUE_CML_Max(x, y) (((x) > (y)) ? (x) : (y))
#define TUE_CML_MinMax(min, max, value) \
  (TUE_CML_Min(TUE_CML_Max(min, value), max))
#define TUE_CML_IsZero(value) (TUE_CML_Abs(value) < TUE_CML_AlmostZero)
#define TUE_CML_MultAdd(a, b, d) (((a) * (b)) + (d))

#define TUE_RAD2DEG(rad_) ((rad_) * (180.F / TUE_CML_Pi))
#define TUE_DEG2RAD(deg_) ((deg_) * (TUE_CML_Pi / 180.F))
#define TUE_ROUND(x) TUE_CML_Round2FloatGen(x)
#define TUE_ROUND_TO_INT(x) TUE_CML_Round2IntGen(x)
/*****************************************************************************
  VARIABLES
*****************************************************************************/

/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/
//���� ����ʽ ��ֵ����
float32 TUE_CML_CalculatePolygonValue2D(sint32 s_NrOfTableRows,
                                        const TUE_CML_Vector2D_t a_Table[],
                                        float32 f_InputValue);
float32 TUE_CML_LinearInterpolation(float32 f_X1, float32 f_Y1, float32 f_X2,
                                    float32 f_Y2, float32 f_XPos);
float32 TUE_CML_BoundedLinInterpol(
    TUE_CML_t_LinFunctionArgs const* const p_Params, const float32 f_Value);
float32 TUE_CML_BoundedLinInterpol2(float32 f_IVal, float32 f_Imin,
                                    float32 f_Imax, float32 f_Omin,
                                    float32 f_Omax);
float32 TUE_CML_CalcStdGaussianCDF(float32 f_value, float32 f_avg,
                                   float32 f_sigma);
float32 TUE_CML_CalcGaussErrorFunction(float32 f_value);
void TUE_CML_CalcPointApproxPolyL2(TUE_CML_PolyResult_t* pPoly,
                                   const float32 pafX[], const float32 pafY[],
                                   uint8 uNumPts);
float32 TUE_CML_CalcDistYOfClothoidsCurve(float32 fHeadingAngle, float32 fCurve,
                                          float32 fCurveDer, float32 fDistX,
                                          float32 fDistY0);

boolean TUE_CML_RisingEdgeSwitch(const boolean bSwitch, boolean* pPrevSwitch);
boolean TUE_CML_FallingEdgeSwitch(const boolean bSwitch, boolean* pPrevSwitch);
// tue_switch_state_t TUE_CML_HoldRepeatSwitch(
//     TUE_CML_Switch_t* const pSwitch,  const boolean StartCondition,  const
//     boolean HoldCondition, uint16 StartTime, uint16 RepeatTime);

void TUE_CML_InitSwitch(TUE_CML_Switch_t* const pSwitch);
void TUE_CML_SetStateSwitch(TUE_CML_Switch_t* const pSwitch,
                            const boolean State);

boolean TUE_CML_RSFlipFlop(const boolean S, const boolean R,
                           boolean* pPrevQ);  // R-S Flip-Flop
float32 TUE_CML_RateLimiter(const float32 fInput, const float32 fLimPos,
                            const float32 fLimNeg, const float32 fTs,
                            float32* pPrevOutput);  // Rate limiter
boolean TUE_CML_HysteresisFloat(float32 fInput, float32 fThresHigh,
                                float32 fThresLow,
                                boolean* pPrevOutput);  // HysteresisFloat

//���Ǻ�������
#define COS_(x) TUE_CML_GDB_cos32(x)
#define SIN_(x) TUE_CML_GDB_sin32(x)
#define TAN_(x) TUE_CML_GDB_tan32(x)
#define COS_HD_(x) TUE_CML_GDBcos_52(x)
#define SIN_HD_(x) TUE_CML_GDBsin_52(x)
#define TAN_HD_(x) TUE_CML_GDBtan_52(x)
#define ATAN_(x) TUE_CML_GDBatan_66(x)
#define ATAN2_(y, x) TUE_CML_GDBatan2_66(y, x)
#define ASIN_(x) TUE_CML_GDBasin_66(x)
#define ACOS_(x) TUE_CML_GDBacos_66(x)

void TUE_CML_LowPassFilter(float32* f_Old, float32 f_New, float32 f_Alpha);

float32 TUE_CML_SqrtApprox(float32 f_radicand);
float32 TUE_CML_ModTrig(float32 f_dividend, float32 f_divisor);
float32 TUE_CML_GDB_cos32(float32 f_angle);
float32 TUE_CML_GDB_sin32(float32 f_angle);
float32 TUE_CML_GDB_tan32(float32 f_angle);
float32 TUE_CML_GDB_tan32s(float32 f_angle);
float32 TUE_CML_GDBtan_52(float32 f_angle);
float32 TUE_CML_GDBsin_52(float32 f_angle);
float32 TUE_CML_GDBcos_52(float32 f_angle);
float32 TUE_CML_GDBtan_56s(float32 f_angle);
float32 TUE_CML_GDBatan_66(float32 f_tan);
float32 TUE_CML_GDBacos_66(float32 f_cos);
float32 TUE_CML_GDBasin_66(float32 f_sin);
float32 TUE_CML_GDBatan2_66(float32 f_yaxis, float32 f_xaxis);

sint32 TUE_CML_Round2IntGen(float32 x);
float32 TUE_CML_Round2FloatGen(float32 x);

// simulation related functions
// Computes object distance and velocity value based on defined object
// trajectory
void SimulationObjectGenerator(
    float32 fObjTrajHeadingAngle_rad, float32 fObjTrajCurve_1pm,
    float32 fObjTrajCurveDer_nu, float32 fObjInitDistX0_met,
    float32 fObjInitDistY0_met, float32 fObjAbsVelX_mps, float32 fEgoVelX_mps,
    float32 fPassedTime_sec, float32* fOutObjDistX_met,
    float32* fOutObjDistY_met, float32* fOutObjRelVelX_mps,
    float32* fOutObjRelVelY_mps);

boolean TUE_CML_TimerRetrigger(float32 fDeltaTime_sec, boolean bReset,
                               float32 fTimeLimit_sec,
                               float32* fRemainTime_sec);
/*****************************************************************************
  CONSTS ADD WRAPPER
*****************************************************************************/
float32 SafeDiv(float32 fDivisor);

#define SenseTime_Memcpy(source, dst, cpySize) memcpy(dst, source, cpySize)

#ifndef GDBmathLinFuncLimBounded
#define GDBmathLinFuncLimBounded(f_XPos, f_X1, f_X2, f_Y1, f_Y2) \
  TUE_CML_BoundedLinInterpol2(f_XPos, f_X1, f_X2, f_Y1,          \
                              f_Y2)  // linear interpolation
#endif

#ifndef CML_f_CalculatePolyValue
#define CML_f_CalculatePolyValue(sTableRows, aTables, fInputValue) \
  TUE_CML_CalculatePolygonValue2D(sTableRows, aTables,             \
                                  fInputValue)  // 2d look-up table
#endif

#ifndef GDB_Math_CalculatePolygonValue
#define GDB_Math_CalculatePolygonValue(sTableRows, aTables, fInputValue) \
  CML_f_CalculatePolyValue(sTableRows, aTables, fInputValue)  // 2d look-up \
                                                              // table
#endif

#ifndef GDB_Math_LowPassFilter
#define GDB_Math_LowPassFilter(f_Old, f_New, f_Alpha) \
  TUE_CML_LowPassFilter(f_Old, f_New, f_Alpha)  // Low Pass Filter
#endif

#ifndef RAD2DEG
#define RAD2DEG(rad) TUE_RAD2DEG(rad)  // Rad to Degree, 1 rad -> 57.3 degree
#endif

#ifndef DEG2RAD
#define DEG2RAD(deg) TUE_DEG2RAD(deg)  // Degree to Rad, 57.3 degree -> 1 rad
#endif

#ifndef fABS
#define fABS(x) TUE_CML_Abs(x)  // Calculate Abs value ,-1 -> 1
#endif

#ifndef MIN
#define MIN(x, y) TUE_CML_Min(x, y)  // Calculate Minimum number
#endif

#ifndef MAX
#define MAX(x, y) TUE_CML_Max(x, y)  // Calculate Maximum number
#endif

#ifndef SQR
#define SQR(x) TUE_CML_Sqr(x)  // Calculate the square, 2^2 = 4
#endif

#ifndef SQRT
#define SQRT(x) TUE_CML_SqrtApprox(x)  // Calculate square root, sqrt(4) = 2
#endif

#ifndef ROUND
#define ROUND(x) TUE_ROUND(x)  // round float to float
#endif

#ifndef ROUND_TO_INT
#define ROUND_TO_INT(x) TUE_ROUND_TO_INT(x)  // round float to int
#endif

#ifndef GDBVector2_t
#define GDBVector2_t TUE_CML_Vector2D_t  // 2d table
#endif

#ifndef C_MS_KMH
#define C_MS_KMH TUE_C_MS_KMH
#endif

#ifdef __cplusplus
}
#endif

#endif