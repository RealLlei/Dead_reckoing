/*! \file **********************************************************************

  PROJECT:                   COMMON

  CPU:                       CPU-Independent

  COMPONENT:                 COMMON TOOLS

  MODULNAME:                 tue_common_libs.c

  DESCRIPTION:               common libs for tue development

  AUTHOR:                    $Author: tao.guo

  CREATION DATE:             $Date: 2020/07/04

  VERSION:                   $Revision: 1.0.0


  CHANGES:
  ---*/ /*---
  CHANGE:                    $Log: tue_common_libs.c
  CHANGE:                    Initial version

**************************************************************************** */

/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "tue_common_libs.h"

/*****************************************************************************
  Functionname:    TUE_CML_CalculatePolygonValue2D                          */ /*!

  @brief           Calculates the value of a polygon at a specific position

  @description     This function calculates the value of a polygon at a given
                   position. The polygon is specified by the 2D sampling points
                   (X,Y). The X-position is given by fInputValue.
                   The function returns the Y-Value at the given X-Position
                   based on the provided sampling points.

  @param[in]       s_NrOfTableRows : number of samplings points in parameter pTable
                   This value must be a positive integer greater than or equal to 1.
  @param[in]       a_Table : pointer to the sampling points. Note: has to be
                   sorted in X (X ascending)!
                   Range for a_Table[].f0 is [Full range of float32]
                   Range for a_Table[].f1 is [Full range of float32]
  @param[in]       f_InputValue : the X value for which the linear interpolated
                   Y bases on 'pTable' shall be returned.
                   [Full range of float32]

  @return          The y-value at the specified x-position
*****************************************************************************/
float32 TUE_CML_CalculatePolygonValue2D(sint32 s_NrOfTableRows,
                                        const TUE_CML_Vector2D_t a_Table[],
                                        float32 f_InputValue) {
  /*get table value*/
  float32 f_Result = 0.F;

  /* If the x-position is left of the smallest sampling point,return the
   * smallest sampling point y-value */
  if (f_InputValue <= a_Table[0].f0) {
    f_Result = a_Table[0].f1;
  }
  /* If the x-position is right of the biggest sampling point,return the biggest
     sampling point y-value */
  else if (f_InputValue >= a_Table[s_NrOfTableRows - 1].f0) {
    f_Result = a_Table[s_NrOfTableRows - 1].f1;
  }
  /* In all other cases return the interpolated value between the matching
     sampling points */
  else {
    sint32 s_Right =
        0; /* The index of the nearest sampling point to the right */
    float32 f_WeightRight = 0.F;
    float32 f_dx = 0.F;

    s_Right = 1; /* Since we already know that index 0 is greater */
    while (a_Table[s_Right].f0 < f_InputValue) {
      s_Right++;
    }
    f_dx = a_Table[s_Right].f0 - a_Table[s_Right - 1].f0;

    if (f_dx > TUE_C_F32_DELTA) {
      f_WeightRight = (f_InputValue - a_Table[s_Right - 1].f0) / (f_dx);
    } else {
      f_WeightRight = 0.0F;
    }

    f_Result = ((a_Table[s_Right - 1].f1) * (1.0F - f_WeightRight)) +
               ((a_Table[s_Right].f1) * f_WeightRight);
  }
  return f_Result;
}

/*****************************************************************************
  Functionname:    TUE_CML_LowPassFilter                                 */ /*!

  @brief           simple first order lowpass filter

  @description     This function is an implementation of simple first order
				   lowpass filter. This determines the output sample in terms of
				   the input sample and preceding output.
				   So if x = input, y = output and z = previous output, and
				   a = filter coefficient, then,
				   y = (a*x) + ((1-a)*z)

  @param[in,out]   f_Old :   old value (in), filtered value (out)
							 Valid float pointer.
							 Supported value for data [Full range of float32]
  @param[in]       f_New :   new value
							 Supported value [Full range of float32]
  @param[in]       f_Alpha : filter coefficient
							 Optimal Values [0,..,1]

  @return          none


*****************************************************************************/
void TUE_CML_LowPassFilter(float32* f_Old, float32 f_New, float32 f_Alpha) {
  float32 f_Dummy = 0.F;

  f_Dummy = (f_Alpha * f_New) + ((1.F - f_Alpha) * (*f_Old));
  *f_Old = f_Dummy;
}

/*****************************************************************************
  Functionname:    TUE_CML_SqrtApprox                                 */ /*!

  @brief           approx calculate of square root

  @description     approx calculate of square root

  @param[in]       f_radicand : input square value

  @return          square result of input value


*****************************************************************************/
float32 TUE_CML_SqrtApprox(float32 f_radicand) {
  sint32 s_expo = 0;
  uint32 u_tmp = 0;
  float32 f_Sample = 0.F;
  float32 f_SampleSquare = 0.F;
  float32 f_A = 0.F;
  float32 f_B = 0.F;
  float32 f_A_plus_B = 0.F;
  float32 f_ret = 0.F;

  /*! union for doing bit-wise manipulation on the floating point representation
   */

  union {
    float32 f_value; /* this is the number of interest as float */
    uint64
        u_value; /* in here we hold the same number as int on which we can do
                       bit-wise manipulations  -----> uint32 ->uint64 */
  } x_tmp;

  /* check for x < 0 or x = NaN */
  if (f_radicand < 0) {
    return 0.F;
  }

  /* copy input value to union where we can manipulate it directly */
  x_tmp.f_value = f_radicand;

  /* check for negative or zero or denormalized */
  if (((x_tmp.u_value >> 31) > 0U)      /* sign bit set? */
      || ((x_tmp.u_value >> 23) == 0U)) /* exponent is zero? */
  {
    f_ret = 0.F;
  }
  /* check for infinity */
  else if ((x_tmp.u_value >> 23) >= 0xffU) {
    x_tmp.u_value = 0x7f7fffffU;
    f_ret = x_tmp.f_value; /* return max_float */
  } else {
    /* calculate start value by dividing exponent by two */
    u_tmp = x_tmp.u_value >> TUE_CML_SqrtApprox_NumExpo;
    s_expo = (sint32)(u_tmp);
    s_expo -= TUE_CML_SqrtApprox_ExponentOffset;
    if (s_expo < 0) {
      s_expo >>= 1;  //NOLINT
      /* without the following line this would be unsafe because
                       right shift of an unsigned int is machine dependent (sign
                       fill vs. zero fill) */
      s_expo |= (sint32)0x80000000U;  //NOLINT
      /* -> fill highest bit with sign (one)
                                        after shift to make it safe */
    } else {
      s_expo >>= 1;  //NOLINT
      /* for positive expo sign = 0 -> sign fill = zero fill */
    }
    s_expo += TUE_CML_SqrtApprox_ExponentOffset;
    x_tmp.u_value &= TUE_CML_SqrtApprox_MantissaMask;
    x_tmp.u_value += ((uint32)s_expo << TUE_CML_SqrtApprox_NumExpo);
    f_Sample = x_tmp.f_value;

    /* two iterations are enough */
    /* iteration one */
    f_SampleSquare = TUE_CML_Sqr(f_Sample);
    f_A = 2.F * (f_SampleSquare + f_radicand);
    f_B = f_SampleSquare - f_radicand;
    f_A_plus_B = f_A + f_B;
    if (TUE_CML_Abs(f_A_plus_B) < TUE_CML_SqrtApprox_AlmostZero) {
      f_ret = 0.F;
    } else {
      f_Sample *= ((f_A - f_B) / (f_A_plus_B));
      /* iteration two */
      f_SampleSquare = TUE_CML_Sqr(f_Sample);
      f_A = 2.F * (f_SampleSquare + f_radicand);
      f_B = f_SampleSquare - f_radicand;

      f_A_plus_B = f_A + f_B;
      f_Sample *= ((f_A - f_B) / (f_A_plus_B));
      f_ret = f_Sample;
    }
  }

  return f_ret;
}

/*****************************************************************************
  Functionname: TUE_CML_TimerRetrigger                                  */ /*!

  @brief: the function is designed for the TIMERRETRIGGER module

  @description: we will set a timer for the specific DeltaTime and TimeLimit, 
			the RemainTime would go back to TimeLimie value while reset is TRUE, 
			and the function output is TRUE while RemainTime higher than zero.
			TIMERRETRIGGER_RE

  @param[in]
			float32 fDeltaTime_sec; time reduce value for every invoke
			boolean bReset: reset timer flag
			float32 fTimeLimit_sec: default time upper limit
  @param[out]
			float32* fRemainTime_sec: remain time value for this timer
  @return: 
*****************************************************************************/
boolean TUE_CML_TimerRetrigger(float32 fDeltaTime_sec, boolean bReset,
                               float32 fTimeLimit_sec,
                               float32* fRemainTime_sec) {
  if (fDeltaTime_sec < 0.F) {
    return FALSE;
  }
  if (bReset) {
    *fRemainTime_sec = fTimeLimit_sec;
  } else {
    *fRemainTime_sec = *fRemainTime_sec > fDeltaTime_sec
                           ? (*fRemainTime_sec - fDeltaTime_sec)
                           : 0.F;
  }
  return *fRemainTime_sec > 0.F;
}

/*****************************************************************************
  Functionname:    TUE_CML_LinearInterpolation                            */ /*!

  @brief           linear interpolation between two given points

  @description     This function computes the linear interpolation value
				   between two given points. The interpolated value can be
				   calculated using the following formula:
				   y = mx + c;
				   where 'm' is the slope and 'c' is the offset.

  @param[in]       f_X1 :  x-coordinate of first point
						   [Full range of float32]
  @param[in]       f_Y1 :  y-coordinate of first point
						   [Full range of float32]
  @param[in]       f_X2 :  x-coordinate of second point
						   [Full range of float32]
  @param[in]       f_Y2 :  y-coordinate of second point
						   [Full range of float32]
  @param[in]       f_XPos :  x-value to interpolate
							 [Full range of float32]

  @return          the interpolated value


*****************************************************************************/
float32 TUE_CML_LinearInterpolation(float32 f_X1, float32 f_Y1, float32 f_X2,
                                    float32 f_Y2, float32 f_XPos) {
  float32 f_Slope = 0.F;
  float32 f_Offset = 0.F;
  float32 f_ret = 0.F;

  if (!TUE_CML_IsZero(f_X2 - f_X1)) {
    /* slope */
    f_Slope = (f_Y2 - f_Y1) / (f_X2 - f_X1);
    /* offset */
    f_Offset = f_Y1 - (f_Slope * f_X1);
    /* interpolate */
    f_ret = (f_Slope * f_XPos) + f_Offset;
  } else {
    f_ret = (f_Y1 + f_Y2) * 0.5F;
  }
  return f_ret;
} /* BML_f_LinearInterpolation() */

/*****************************************************************************
  Functionname:    TUE_CML_CalcStdGaussianCDF                             */ /*!

  @brief           Calculate the value of the standard Gaussian CDF

  @description     This function calculate the value of the standard Gaussian
				   cumulative distribution function
				   CDF = 0.5 ( 1 + errorfunction ( ( x - aver ) / (sigma * sqrt(2) ) )

  @param[in]       f_value : input to the CDF
							 [Full range of float]
  @param[in]       f_avg   : mean of the Gaussian distribution
							 [Full range of float]
  @param[in]       f_sigma : standard deviation of the Gaussian distribution
							 [Full range of float]

  @return          standard Gaussian CDF at the given value


*****************************************************************************/
float32 TUE_CML_CalcStdGaussianCDF(float32 f_value, float32 f_avg,
                                   float32 f_sigma) {
  float32 f_temp = 0.F;

  if (TUE_CML_Abs(f_sigma) < TUE_CML_GaussianCDFMinSigma) {
    if (f_value < f_avg) {
      f_temp = 0.0F;
    } else {
      f_temp = 1.0F;
    }
  } else {
    f_temp = (f_value - f_avg) / (f_sigma * TUE_CML_SQRT_OF_2);

    /* check for negative values */
    if (f_temp < 0.0F) {
      f_temp = -TUE_CML_CalcGaussErrorFunction(-f_temp);
    } else {
      f_temp = TUE_CML_CalcGaussErrorFunction(f_temp);
    }

    f_temp = (0.5F * f_temp + 0.5F);
  }

  return f_temp;
}

/*****************************************************************************
  Functionname:    TUE_CML_CalcGaussErrorFunction                         */ /*!

  @brief           Calculate the Gauss Error Function

  @description     This function calculate the Gauss Error Function
				   Aproximate with a 4th order polynomial, the return value
				   G = ( ( ( (C4 * x^4) - (C3 * x^3) ) - (C2 * x^2) ) + (C1 * x) ) + C0,
				   where the coefficients C0, C1, C2, C3 and C4 are predefined
				   values.

  @param[in]       f_value : input to the Gauss error function
							 Supported values for f_value [-F_MAX...F_MAX]
							 where F_MAX is the fourth root of the maximum value of float32.

  @return          Gauss error function value


*****************************************************************************/
float32 TUE_CML_CalcGaussErrorFunction(float32 f_value) {
  float32 f_temp2 = 0.F;
  float32 f_temp3 = 0.F;
  float32 f_temp = 0.F;
  float32 f_temp4 = 0.F;

  if (f_value >= TUE_CML_GaussErrFctMaxX) {
    f_temp = 1.0F;
  } else {
    f_temp2 = f_value * f_value; /* x^2 */
    f_temp3 = f_temp2 * f_value; /* x^3 */
    f_temp4 = f_temp2 * f_temp2; /* x^4 */
    f_temp = ((((TUE_CML_GaussErrFctConst4 * f_temp4) -
                (TUE_CML_GaussErrFctConst3 * f_temp3)) -
               (TUE_CML_GaussErrFctConst2 * f_temp2)) +
              (TUE_CML_GaussErrFctConst1 * f_value)) +
             TUE_CML_GaussErrFctConst0;

    f_temp = TUE_CML_Min(f_temp, 1.0F);
  }

  return f_temp;
}

/*****************************************************************************
  Functionname:    TUE_CML_CalcPointApproxPolyL2                         */ /*!

  @brief           Calculate 2nd power polynomial for approximating sample points

  @description     This function calculates the approximate polynomial fitting
                   the sample points using least square fit. The calculated
                   polynomial has the form f(x) = fC0 + fC1*x + fC2*x^2

  @param[in,out]   pPoly : Pointer to structure storing the second degree polynomial approximation of a trace [CPTracePolyL2_t as defined in cp_ext.h]
						pPoly->fC2 : Coefficient of second-order term                                      [-1f...+1f]
						pPoly->fC1 : Coefficient of first-order term                                       [-1f...+1f]
						pPoly->fC0 : Coefficient of zeroth-order term                                      [-PI/2*RW_FCT_MAX ... PI/2*RW_FCT_MAX]
						pPoly->isValid : Flag to indicate whether the trace polynomial is valid            [TRUE, FALSE]
  @param[in]       pafX[] : array of x coordinates of trace points                  [-5*RW_FCT_MAX ... 5*RW_FCT_MAX] of size [0 ... FIP_STATIC_TRACE_NO_OF_POINTS[
  @param[in]       pafY[] : array of y coordinates of trace points                  [-PI/2*RW_FCT_MAX ... PI/2*RW_FCT_MAX] of size [0 ... FIP_STATIC_TRACE_NO_OF_POINTS[
  @param[in]       uNumPts : Number of trace points                                  [0 ... FIP_STATIC_TRACE_NO_OF_POINTS]

  @return          null
*****************************************************************************/
void TUE_CML_CalcPointApproxPolyL2(TUE_CML_PolyResult_t* pPoly,
                                   const float32 pafX[], const float32 pafY[],
                                   uint8 uNumPts) {
  sint32 i = 0;
  sint32 j = 0;
  sint32 k = 0;
  float32 fXPow4Sum = 0.F;
  float32 fXPow3Sum = 0.F;
  float32 fXPow2Sum = 0.F;
  float32 fXSum = 0.F;
  float32 fXYSum = 0.F;
  float32 fX2YSum = 0.F;
  float32 fYSum = 0.F;
  float32 fNumPts = 0.F;
  float32 fLinEqMatrix[3][4] = {0.F};

  /* Verify that we have at least 3 points, below that just use a line
   * extrapolation */
  if (uNumPts > 2U) {
    /* First calculate the necessary terms for our linear equation set
    to use for least squares fit */
    fXPow4Sum = 0.F;
    fXPow3Sum = 0.F;
    fXPow2Sum = 0.F;
    fXSum = 0.F;
    fXYSum = 0.F;
    fX2YSum = 0.F;
    fYSum = 0.F;
    fNumPts = (float32)uNumPts;
    /* Initialize return polynomial C0 to Y0 (plays a role when X coordinates
     * all zero) */
    pPoly->fC0 = *pafY;
    /* Go through all points and calculate the sums */
    while (uNumPts > 0U) {
      const float32 fCurX = *pafX;
      const float32 fCurY = *pafY;
      const float32 fCurX2 = TUE_CML_Sqr(fCurX);
      pafX++;

      pafY++;

      /* Update sums */
      fXSum += fCurX;
      fXPow2Sum += fCurX2;
      fXPow3Sum += fCurX2 * fCurX;
      fXPow4Sum += TUE_CML_Sqr(fCurX2);
      fYSum += fCurY;
      fXYSum += fCurX * fCurY;
      fX2YSum += fCurX2 * fCurY;
      /* Decrease remaining number of points */
      uNumPts--;
    }
    /* Now we have a linear equation set:
    fXPow4Sum*C2 + fXPow3Sum*C1 + fXPow2Sum*C0 = fX2YSum
    fXPow3Sum*C2 + fXPow2Sum*C1 + fXSum*C0     = fXYSum
    fXPow2Sum*C2 + fXSum*C1     + NumPts*C0    = fYSum
    Notice how the diagonal of the matrix is always positive, if there is
    at least one point with an X coordinate other than 0. Also note that
    the inverse of fXPowySum always exists (due to this) */
    if (TUE_CML_Abs((fXPow2Sum * fXPow4Sum) - (fXPow3Sum * fXPow3Sum)) >
        TUE_C_F32_DELTA) {
      fLinEqMatrix[0][0] = fXPow4Sum;
      fLinEqMatrix[0][1] = fXPow3Sum;
      fLinEqMatrix[0][2] = fXPow2Sum;
      fLinEqMatrix[0][3] = fX2YSum;
      fLinEqMatrix[1][0] = fXPow3Sum;
      fLinEqMatrix[1][1] = fXPow2Sum;
      fLinEqMatrix[1][2] = fXSum;
      fLinEqMatrix[1][3] = fXYSum;
      fLinEqMatrix[2][0] = fXPow2Sum;
      fLinEqMatrix[2][1] = fXSum;
      fLinEqMatrix[2][2] = fNumPts;
      fLinEqMatrix[2][3] = fYSum;
      /* Now solve it, first converting the matrix to upper triangular form
      and then using elimination to solve it (Gauss elimination) */
      for (i = 0; i < 2; i++) {
        float32 fDivisor = fLinEqMatrix[i][i];
        float32 fInvColumnMax = 0.F;
        /* Calculate inverse of column max once here, prevent devision by zero
         */
        if (TUE_CML_Abs(fDivisor) < TUE_C_F32_DELTA) {
          if (fDivisor < 0.0F) {
            fDivisor = -TUE_C_F32_DELTA;
          } else {
            fDivisor = TUE_C_F32_DELTA;
          }
        }
        fInvColumnMax = 1.F / fDivisor;
        /* Now do forward substitution */
        for (j = 3; j >= i; j--) {
          for (k = i + 1; k < 3; k++) {
            fLinEqMatrix[k][j] -=
                fLinEqMatrix[k][i] * fInvColumnMax * fLinEqMatrix[i][j];
          }
        }
      }
      /* Now do reverse elimination */
      for (i = 2; i >= 0; i--) {
        /* Verify that we have a leading non-zero value in the row */
        if ((fLinEqMatrix[i][i] > TUE_C_F32_EXT_DELTA) ||
            (fLinEqMatrix[i][i] < -TUE_C_F32_EXT_DELTA)) {
          /* Calculate inverse of the diagonal element currently processed */
          const float32 fInvCurDiagVal = 1.F / fLinEqMatrix[i][i];
          fLinEqMatrix[i][3] = fLinEqMatrix[i][3] * fInvCurDiagVal;
          fLinEqMatrix[i][i] = 1.F;
          for (j = i - 1; j >= 0; j--) {
            fLinEqMatrix[j][3] -= fLinEqMatrix[j][i] * fLinEqMatrix[i][3];
            fLinEqMatrix[j][i] = 0.F;
          }
        } else {
          fLinEqMatrix[i][3] = 0.F;
        }
      }
      /* Fill in result */
      pPoly->fC2 = fLinEqMatrix[0][3];
      pPoly->fC1 = fLinEqMatrix[1][3];
      pPoly->fC0 = fLinEqMatrix[2][3];
      pPoly->isValid = TRUE;
    } else {
      /* fXPow2Sum is zero -> all points have zero X coordinate */
      pPoly->fC2 = 0.F;
      pPoly->fC1 = 0.F;
      pPoly->isValid = FALSE;
      /* C0 initialization value to Y coordinate already OK */
    }
  } else {
    /* Initialize default return value */
    pPoly->fC2 = 0.F;
    pPoly->fC1 = 0.F;
    pPoly->fC0 = pafY[0];
    pPoly->isValid = FALSE;
    /* Approximate object movement by a simple line */
    if (uNumPts > 1U) {
      const float32 fDeltaX = pafX[1] - pafX[0];
      const float32 fDeltaY = pafY[1] - pafY[0];
      if ((fDeltaX > TUE_C_F32_DELTA) || (fDeltaX < -TUE_C_F32_DELTA)) {
        pPoly->fC2 = 0.F;
        pPoly->fC1 = (fDeltaY / fDeltaX);
        pPoly->fC0 = pafY[0] - (pafX[0] * pPoly->fC1);
        pPoly->isValid = TRUE;
      }
    }
  }
}

/*****************************************************************************
  Functionname:    TUE_CML_ModTrig                                        */ /*!

  @brief           Calculates the remainder of x when divided by y as needed
					by trigonometric functions

  @description     This function calculates the remainder of x when divided by y
				   Works only for y > 0
				   The function is equivalent to rem() function in Matlab.


  @param[in]       f_dividend : The Dividend
								Supported values are [Full range of float32]
								Overflow may occur at higher values.
  @param[in]       f_divisor  : The Divisor
								Supported values are [Full range of float32]
								Overflow may occur at very small values.

  @return          remainder of f_dividend when divided by f_divisor

*****************************************************************************/
float32 TUE_CML_ModTrig(float32 f_dividend, float32 f_divisor) {
  float32 f_quotient = 0.F;
  float32 f_ret = 0.F;
  sint32 s_quotient = 0;

  if (f_divisor < TUE_CML_ModuloEps) {
    f_ret = 0.F;
  } else {
    f_quotient = f_dividend / f_divisor;

    if (TUE_CML_Abs(f_quotient) > (float32)TUE_CML_LONG_MAX) {
      f_ret = 0.F;
    } else {
      s_quotient = (sint32)(f_quotient);
      f_ret = (f_dividend - ((float32)s_quotient * f_divisor));
    }
  }

  return f_ret;
}

/*****************************************************************************
  Functionname:    TUE_CML_GDBtan_56s */ /*!

  @brief           computes tan(pi*x/4)

  @description     Accurate to about 5.6 decimal digits over
				   the range [0, pi/4].
				   Note that the function computes tan(pi*x/4),
				   NOT tan(x); it's up to the range
				   reduction algorithm that calls this to scale
				   things properly.
				   Algorithm: tan(x)= x(c1 + c2*x^2)/(c3 + x^2)

  @param[in]       f_angle : the angle (times pi/4) for which we want to know
							 the tangent, radians
							 Supported values are [-MAX_ANGLE,..,MAX_ANGLE],
							 where MAX_ANGLE is cube root of max value of float32

  @return          parameter for tan_56(f_angle)

  @pre             GDBtan_56(x)

*****************************************************************************/
float32 TUE_CML_GDBtan_56s(float32 f_angle) {
  return (f_angle *
          (TUE_CML_TAN_56_C1 + (TUE_CML_TAN_56_C2 * TUE_CML_Sqr(f_angle)))) /
         (TUE_CML_TAN_56_C3 + TUE_CML_Sqr(f_angle));
}

/*****************************************************************************
  Functionname:    TUE_CML_GDBtan_52 */ /*!

  @brief           Computes the tangent of x with accuracy of about 5.6
				   decimal digits

  @description     This is the main tangent approximation "driver".
				   It reduces the input argument's range to [0, pi/4],
				   and then calls the approximator.
					WARNING: We do not test for the tangent approaching
				   infinity,  which it will at x=pi/2 and x=3*pi/2.
				   If this is a problem in your application, take
				   appropriate action.

  @param[in]       f_angle : the angle for which we want to know the
							 tangent, radians
							 Supported values are [Full range of float32]
							 except ((2*n) + 1)*(0.5f*TUE_CML_Pi), n is any integer.

  @return          the tangent of f_angle

*****************************************************************************/
float32 TUE_CML_GDBtan_52(float32 f_angle) {
  /*--- VARIABLES ---*/
  float32 f_tan = 0.F;
  float32 f_tmp = 0.F;
  uint32 u_octant = 0;
  boolean b_sign = FALSE;

  if (f_angle < 0.0F) {
    f_angle = -f_angle;
    b_sign = TRUE;
  }

  /* limit to two pi */
  if (f_angle > (10.0F * TUE_CML_Pi)) {
    f_angle = TUE_CML_ModTrig(f_angle, (2.0F * TUE_CML_Pi));
  } else {
    while (f_angle >= (2.0F * TUE_CML_Pi)) {
      f_angle -= (2.0F * TUE_CML_Pi);
    }
  }

  f_tmp = f_angle * (4.0F / TUE_CML_Pi);
  u_octant = (uint32)f_tmp;

  switch (u_octant) {
    case 1:
      f_tan = 1.0F / TUE_CML_GDBtan_56s(((0.5F * TUE_CML_Pi) - f_angle) *
                                        (4.0F / TUE_CML_Pi));
      break;
    case 2:
      f_tan = -1.0F / TUE_CML_GDBtan_56s((f_angle - (0.5F * TUE_CML_Pi)) *
                                         (4.0F / TUE_CML_Pi));
      break;
    case 3:
      f_tan = -TUE_CML_GDBtan_56s((TUE_CML_Pi - f_angle) * (4.0F / TUE_CML_Pi));
      break;
    case 4:
      f_tan = TUE_CML_GDBtan_56s((f_angle - TUE_CML_Pi) * (4.0F / TUE_CML_Pi));
      break;
    case 5:
      f_tan = 1.0F / TUE_CML_GDBtan_56s(((1.5F * TUE_CML_Pi) - f_angle) *
                                        (4.0F / TUE_CML_Pi));
      break;
    case 6:
      f_tan = -1.0F / TUE_CML_GDBtan_56s((f_angle - (1.5F * TUE_CML_Pi)) *
                                         (4.0F / TUE_CML_Pi));
      break;
    case 7:
      f_tan = -TUE_CML_GDBtan_56s(((2.0F * TUE_CML_Pi) - f_angle) *
                                  (4.0F / TUE_CML_Pi));
      break;
    default:
      /*Case 0*/
      f_tan = TUE_CML_GDBtan_56s(f_angle * (4.0F / TUE_CML_Pi));
      break;
  }

  if (b_sign == TRUE) {
    f_tan = -f_tan;
  }

  return f_tan;
}

/*****************************************************************************
  Functionname:    TUE_CML_CalcPointApproxPolyL2                         */ /*!

  @brief           Calculate 3nd clothoid model DistY value in target DistX

  @description     the form Y = Y0 + aX + 0.5 * C0 * X^2 + 1/6 * C1 * X^3

  @param[in]       

  @return          fDistY
*****************************************************************************/
float32 TUE_CML_CalcDistYOfClothoidsCurve(float32 fHeadingAngle, float32 fCurve,
                                          float32 fCurveDer, float32 fDistX,
                                          float32 fDistY0) {
  float32 y_s = 0.F;
  y_s = fDistY0 + (TUE_CML_GDBtan_52(fHeadingAngle)) * fDistX +
        (0.5F * fDistX * fDistX * fCurve) +
        ((1.0F / 6.0F) * fDistX * fDistX * fDistX * fCurveDer);
  return y_s;
}

/*****************************************************************************
  @fn				TUE_CML_RisingEdgeSwitch */ /*!

  @description		checks if the switch is switched on

  @param[in]		bSwitch		Switch to check (TRUE, FALSE)
  @param[in,out]    bPrevSwitch	Previous status of switch (TRUE, FALSE)
								Define a global variable and input its address.
								Note: The initial value should be TRUE

  @return			pSwitchChk	Flag that the switch is switched on

*****************************************************************************/
boolean TUE_CML_RisingEdgeSwitch(const boolean bSwitch, boolean* pPrevSwitch) {
  boolean bSwitchChk = FALSE;

  if ((*pPrevSwitch == FALSE) && (bSwitch == TRUE)) {
    bSwitchChk = TRUE;
  }

  *pPrevSwitch = bSwitch;

  return bSwitchChk;
}

/*****************************************************************************
  @fn				TUE_CML_FallingEdgeSwitch */ /*!

  @description		checks if the switch is switched off

  @param[in]		bSwitch		Switch to check (TRUE, FALSE)
  @param[in,out]    bPrevSwitch	Previous status of switch (TRUE, FALSE)
								Define a global variable and input its address.
								Note: The initial value should be FALSE

  @return			pSwitchChk	Flag that the switch is switched off

*****************************************************************************/
boolean TUE_CML_FallingEdgeSwitch(const boolean bSwitch, boolean* pPrevSwitch) {
  boolean bSwitchChk = FALSE;

  if ((*pPrevSwitch == TRUE) && (bSwitch == FALSE)) {
    bSwitchChk = TRUE;
  }

  *pPrevSwitch = bSwitch;

  return bSwitchChk;
}

/*****************************************************************************
  @fn           TUE_CML_HoldRepeatSwitch */ /*!

  @description  returns SWTICH_STATE_ON, if the switch was hold long enough to get repeated signals from that switch

  @param[in]    pSwitch         Switch to check
  @param[in]    StartCondition  Condition that allows the switch to be switched on (0 = false, 1 = true)
  @param[in]    HoldCondition   Condition that allows the switch to send repeated signals after has been switched on (0 = false, 1 = true)
  @param[in]    StartTime       Time, the switch needs to be in on state before it returns true for the first time
  @param[in]    RepeatTime      Time, between the repeated signals after the switch has returned true the first time (if 0 -> function returns only one true after start time)

  @return       TUE_SWITCH_STATE_OFF(0) = FALSE,
				TUE_SWITCH_STATE_ON(1) = TRUE,
				TUE_SWITCH_STATE_ACTION_OFF(2) = currently not returned

*****************************************************************************/
// tue_switch_state_t TUE_CML_HoldRepeatSwitch(TUE_CML_Switch_t* pSwitch,
//                                           const  boolean StartCondition,
//                                           const  boolean HoldCondition,
//                                             uint16 StartTime,
//                                             uint16 RepeatTime) {

//   tue_switch_state_t retValue = TUE_SWITCH_STATE_OFF;

//   if ((pSwitch->AKT_STATUS == FALSE) /*switch off*/
//       || ((HoldCondition == FALSE) &&
//           (pSwitch->CYCLE_TIMER != TUE_CML_SWITCH_CYCLETIMER_INIT)) ||
//       (StartCondition != TRUE) /*not all conditions true*/
//   ) {
//     /*init cycle timer*/
//     pSwitch->CYCLE_TIMER = TUE_CML_SWITCH_CYCLETIMER_INIT;
//   } else /*button pressed with all conditions met*/
//   {
//   const tue_switch_state_t StartOK =
//         TUE_CML_RisingEdgeSwitch(pSwitch->OK_WHILE_SWITCHED_ON,
//         StartCondition);

//     if ((pSwitch->CYCLE_TIMER ==
//          TUE_CML_SWITCH_CYCLETIMER_INIT)    /*cycle timer not initialized*/
//         && (StartOK == TUE_SWITCH_STATE_ON) /*start conditions true*/
//     ) {                                     /*initialize cycle with
//     startTime*/
//       pSwitch->CYCLE_TIMER = (uint16)TUE_CML_Min(
//           TUE_CML_SWITCH_CYCLETIMER_INIT - 1, (sint32)StartTime);
//     } else {
//       if ((pSwitch->CYCLE_TIMER > (uint16)0) /*cycle timer > 0*/
//           && (pSwitch->CYCLE_TIMER <
//               TUE_CML_SWITCH_CYCLETIMER_INIT) /*cycle timer is initialized*/
//           && (HoldCondition == TRUE)          /*hold condition true*/
//       ) {
//         pSwitch->CYCLE_TIMER--; /*decrease cycle timer*/
//       }

//       if (pSwitch->CYCLE_TIMER == (uint16)0) /*cycle timer 0ed*/
//       { /*initialize cycle timer with repeat time*/
//         pSwitch->CYCLE_TIMER = (uint16)TUE_CML_Min(
//             TUE_CML_SWITCH_CYCLETIMER_INIT - 1, (sint32)RepeatTime);
//         if (pSwitch->CYCLE_TIMER == (uint16)0) {
//           pSwitch->CYCLE_TIMER =
//               TUE_CML_SWITCH_CYCLETIMER_INIT; /*reinitialize cycle timer
//                                                  (returns only one true after
//                                                  start time if
//                                                  repeattime=0)*/
//         }
//         retValue = TUE_SWITCH_STATE_ON;
//       }
//     }
//   }
//   return retValue;
// }

/*****************************************************************************
  @fn             SWITCH_INIT_SWITCH */ /*!

  @description    initialize switches

  @param[in]      pSwitch   Switch that shall be initialized

  @return         void

*****************************************************************************/
void TUE_CML_InitSwitch(TUE_CML_Switch_t* const pSwitch) {
  pSwitch->AKT_STATUS = FALSE;
  pSwitch->LAST_STATUS = FALSE;
  pSwitch->CYCLE_TIMER = TUE_CML_SWITCH_CYCLETIMER_INIT;
  pSwitch->DURATION_TIME_INACTIVE = TUE_CML_SWITCH_TIME_MAX;
  pSwitch->DURATION_TIME_ACTIVE = 0U;
  pSwitch->OK_WHILE_SWITCHED_ON = FALSE;
}

/*****************************************************************************
  @fn             TUE_CML_SetStateSwitch */ /*!

  @description    sets a new switch state for a specific switch

  @param[in,out]  pSwitch  Switch that shall be set
  @param[in]      State    the new state for the switch (TRUE, FALSE)

  @return         void

*****************************************************************************/
void TUE_CML_SetStateSwitch(TUE_CML_Switch_t* const pSwitch,
                            const boolean State) {
  const boolean lastState = (boolean)pSwitch->AKT_STATUS;

  /*save old value*/
  pSwitch->LAST_STATUS = pSwitch->AKT_STATUS;
  /*Set new value*/
  pSwitch->AKT_STATUS = State;

  /*count cycles of (in)activity*/
  if (State == TRUE) {
    if (pSwitch->DURATION_TIME_ACTIVE < TUE_CML_SWITCH_TIME_MAX) {
      pSwitch->DURATION_TIME_ACTIVE++;
    }
  } else {
    if (pSwitch->DURATION_TIME_INACTIVE < TUE_CML_SWITCH_TIME_MAX) {
      pSwitch->DURATION_TIME_INACTIVE++;
    }
  }

  if (lastState == FALSE) {
    pSwitch->DURATION_TIME_ACTIVE = (uint16)0;
  } else {
    pSwitch->DURATION_TIME_INACTIVE = (uint16)0;
  }
}

/*****************************************************************************
  @fn				TUE_CML_RSFlipFlop */ /*!

  @description		R-S Flip-Flop

  @param[in]		S		Set input of R-S Flip-Flop(TRUE, FALSE)
  @param[in]		R		Reset input of R-S Flip-Flop(TRUE, FALSE)
  @param[in,out]    pPrevQ	Previous output Q of R-S Flip-Flop(TRUE, FALSE)
							Define a global variable and input its address.
							Note: The initial value should be FALSE.

  @return           Q		Output Q of R-S Flip-Flop(TRUE, FALSE)
*****************************************************************************/
boolean TUE_CML_RSFlipFlop(const boolean S, const boolean R, boolean* pPrevQ) {
  boolean Q = FALSE;

  if (R != FALSE) {
    Q = FALSE;
  } else {
    Q = S || (*pPrevQ);
  }

  *pPrevQ = Q;

  return Q;
}

/*****************************************************************************
  @fn				TUE_CML_RateLimiter */ /*!

  @description		Rate limiter

  @param[in]		fInput		Input to limit
  @param[in]		fLimPos		Max positive limit of rate
  @param[in]		fLimNeg 	Min negative limit of rate							   
  @param[in]		fTs 		Schedule time
  @param[in,out]	pPrevOutput Previous output of RateLimiter
								Define a global variable and input its address.
								Note: The initial value should be 0.f

  @return           fOutput		Output after rate limiting
*****************************************************************************/
float32 TUE_CML_RateLimiter(const float32 fInput, const float32 fLimPos,
                            const float32 fLimNeg, const float32 fTs,
                            float32* pPrevOutput) {
  float32 fOutput = 0.F;

  fOutput = fInput - (*pPrevOutput);
  fOutput = TUE_CML_MinMax(-fLimNeg * fTs, fLimPos * fTs, fOutput);
  fOutput = fOutput + (*pPrevOutput);

  *pPrevOutput = fOutput;

  return fOutput;
}

/*****************************************************************************
  @fn				TUE_CML_HysteresisFloat */ /*!

  @description		Hysteresis

  @param[in]		fInput		 Input to limit
  @param[in]		fThresHigh	 High threshold 
  @param[in]		fThresLow 	 Low threshold 
  @param[in,out]	pPrevOutput  Previous output of Hysteresis
								 Define a global variable and input its address.
								 Note: The initial value should be FALSE

  @return           *pPrevOutput Hysteresis output
*****************************************************************************/
boolean TUE_CML_HysteresisFloat(float32 fInput, float32 fThresHigh,
                                float32 fThresLow, boolean* pPrevOutput) {
  if (fThresHigh >= fThresLow) {
    if ((*pPrevOutput == FALSE) && (fInput > fThresHigh)) {
      *pPrevOutput = TRUE;
    } else if ((*pPrevOutput == TRUE) && (fInput < fThresLow)) {
      *pPrevOutput = FALSE;
    } else {
    }
  } else {
    if ((*pPrevOutput == FALSE) && (fInput > fThresLow)) {
      *pPrevOutput = TRUE;
    } else if ((*pPrevOutput == TRUE) && (fInput < fThresHigh)) {
      *pPrevOutput = FALSE;
    } else {
    }
  }

  return *pPrevOutput;
}

/*****************************************************************************
  Functionname:    TUE_CML_GDB_cos32 */ /*!

  @brief           Calculates the cosine with 3.2 decimals accuracy

  @description     It reduces the input argument's range to [0, pi/2],
				   and then performs the approximation.
				   Algorithm:
						   cos(x)= c1 + c2*x**2 + c3*x**4
				   which is the same as:
						   cos(x)= c1 + x**2(c2 + c3*x**2)

  @param[in]       f_angle : input angle for which we would like to know the cosine, radians
							 Supported values are [-MAX_ANGLE,..,MAX_ANGLE],
							 where MAX_ANGLE = [max range of uint32] * CML_f_two_Pi
  @return          the cosine of f_angle
*****************************************************************************/
float32 TUE_CML_GDB_cos32(float32 f_angle) {
  /*--- VARIABLES ---*/
  uint32 u_n = 0;
  float32 f_angle_square = 0.F;
  float32 f_tmp = 0.F;
  float32 f_Ret = 0.F;

  /* remove sign, as COS function is symmetric */
  f_angle = TUE_CML_Abs(f_angle);

  /* Calculate approximation depending on quadrant. First, check if f_angle is
     in 1st one. */
  if (f_angle < (TUE_CML_Pi / 2.0F)) {
    f_angle_square = TUE_CML_Sqr(f_angle);
    f_tmp = (C_COS_32_C3 * f_angle_square + C_COS_32_C2);
    f_Ret = (f_angle_square * f_tmp + C_COS_32_C1);
  }

  /* 2nd and 3rd quadrant. */
  else if (f_angle < (TUE_CML_Pi + (TUE_CML_Pi * 0.5F))) {
    f_angle_square = TUE_CML_Pi - f_angle;
    f_angle_square = TUE_CML_Abs(f_angle_square);
    f_angle_square = TUE_CML_Sqr(f_angle_square);
    f_tmp = TUE_CML_MultAdd((-1.0F) * C_COS_32_C3, f_angle_square,
                            (-1.0F) * C_COS_32_C2);
    f_Ret = TUE_CML_MultAdd(f_angle_square, f_tmp, (-1.0F) * C_COS_32_C1);
  }

  /* 4th quadrant. */
  else if (f_angle < (TUE_CML_Pi * 2.0F)) {
    f_angle_square = (TUE_CML_Pi * 2.0F) - f_angle;
    f_angle_square = TUE_CML_Sqr(f_angle_square);
    f_tmp = (C_COS_32_C3 * f_angle_square + C_COS_32_C2);
    f_Ret = (f_angle_square * f_tmp + C_COS_32_C1);
  }

  /* f_angle is out of 1st period. --> Shift it to [-PI..+PI] and use symmetry
     of COS. */
  else {
    /* limit to two_pi : f_angle = mod(f_angle, two_pi) limitation: quotient
       shall no exceed C_LONG_MAX. => f_angle < (LONG_MAX * (2.0F * TUE_CML_Pi))
       Regarding to float32 accuracy of about 7 decimals, the reasonable
       threshold for f_angle is reached much earlier. */
    f_tmp = f_angle * (1.0F / (TUE_CML_Pi * 2.0F));
    u_n = (uint32)(f_tmp);

    /* Shift f_angle to [-PI..PI]. Due to symmetry of COS, it's enough to
       evaluate [0..PI]. */
    f_angle = (f_angle - ((float32)u_n * (TUE_CML_Pi * 2.0F))) - TUE_CML_Pi;
    f_angle = TUE_CML_Abs(f_angle);

    /* Calculate approximation depending on quadrant. First, check if f_angle is
       in 2nd (or 3rd) one. */
    if (f_angle > (TUE_CML_Pi * 0.5F)) {
      f_angle_square = TUE_CML_Pi - f_angle;
      f_angle_square = TUE_CML_Sqr(f_angle_square);
      f_tmp = (C_COS_32_C3 * f_angle_square + C_COS_32_C2);
      f_Ret = (f_angle_square * f_tmp + C_COS_32_C1);
    }

    /* 1st (or 4th) quadrant). */
    else {
      f_angle_square = TUE_CML_Sqr(f_angle);
      f_tmp = (C_COS_32_C3 * f_angle_square + C_COS_32_C2);
      f_Ret = -(f_angle_square * f_tmp + C_COS_32_C1);
    }
  }

  return (f_Ret);
} /* TUE_CML_GDB_cos32() */

/*****************************************************************************
  Functionname:    TUE_CML_GDB_sin32 */ /*!

  @brief           Calculates the sine with 3.2 decimals accuracy

  @description     This function calculates the sine with 3.2 decimals
				   accuracy.
				   The sine is just cosine shifted a half-pi,
				   so the argument is adjusted and the cosine
				   approximation is called.

  @param[in]       f_angle : input angle for which we would like to know the
							 sine, radians
							 Supported values are [-MAX_ANGLE,..,MAX_ANGLE],
							 where MAX_ANGLE =
							 ([max range of uint32] * CML_f_two_Pi)-(0.5F * TUE_CML_Pi)
  @return          the sine of f_angle
*****************************************************************************/
float32 TUE_CML_GDB_sin32(float32 f_angle) {
  return TUE_CML_GDB_cos32((TUE_CML_Pi * 0.5F) - f_angle);
}

/*****************************************************************************
  Functionname:    TUE_CML_GDB_tan32 */ /*!

  @brief           Computes the tangent of x with accuracy of about 3.2 decimal digits

  @description     This is the main tangent approximation "driver".
				   It reduces the input argument's range to [0, pi/4],
				   and then calls the approximator.
				   WARNING: We do not test for the tangent approaching
				   infinity,  which it will at x=pi/2 and x=3*pi/2.
				   If this is a problem in your application, take
				   appropriate action.

  @param[in]       f_angle : the angle for which we want to know the
							 tangent, radians
							 Supported values are [Full range of float32]
							 except ((2*n) + 1)*(0.5F * TUE_CML_Pi), n is any integer.

  @return          the tangent of f_angle
*****************************************************************************/
float32 TUE_CML_GDB_tan32(float32 f_angle) {
  /*--- VARIABLES ---*/
  float32 f_tan = 0.F;    /*!< return value */
  uint32 u_octant = 0;    /*!< what octant are we in? */
  boolean b_sign = FALSE; /*!< TRUE, if arg was < 0 */
  float32 f_tmp = 0.F;

  if (f_angle < 0.0F) {
    f_angle = -f_angle;
    b_sign = TRUE;
  }

  /* linit to two pi */
  if (f_angle > (10.0F * TUE_CML_Pi)) {
    f_angle = TUE_CML_ModTrig(f_angle, (2.0F * TUE_CML_Pi));
  } else {
    while (f_angle >= (2.0F * TUE_CML_Pi)) {
      f_angle -= (2.0F * TUE_CML_Pi);
    }
  }

  /*! Get octant # (0 to 7) */
  f_tmp = f_angle * (4.0F / TUE_CML_Pi);
  u_octant = (uint32)f_tmp;

  switch (u_octant) {
    case 1:
      f_tan = 1.0F / TUE_CML_GDB_tan32s((((0.5F * TUE_CML_Pi) - f_angle) *
                                         (4.0F / TUE_CML_Pi)));
      break;
    case 2:
      f_tan = -1.0F / TUE_CML_GDB_tan32s(((f_angle - (0.5F * TUE_CML_Pi)) *
                                          (4.0F / TUE_CML_Pi)));
      break;
    case 3:
      f_tan =
          -TUE_CML_GDB_tan32s(((TUE_CML_Pi - f_angle) * (4.0F / TUE_CML_Pi)));
      break;
    case 4:
      f_tan =
          TUE_CML_GDB_tan32s(((f_angle - TUE_CML_Pi) * (4.0F / TUE_CML_Pi)));
      break;
    case 5:
      f_tan = 1.0F / TUE_CML_GDB_tan32s((((1.5F * TUE_CML_Pi) - f_angle) *
                                         (4.0F / TUE_CML_Pi)));
      break;
    case 6:
      f_tan = -1.0F / TUE_CML_GDB_tan32s(((f_angle - (1.5F * TUE_CML_Pi)) *
                                          (4.0F / TUE_CML_Pi)));
      break;
    case 7:
      f_tan = -TUE_CML_GDB_tan32s(
          (((2.0F * TUE_CML_Pi) - f_angle) * (4.0F / TUE_CML_Pi)));
      break;
    default:
      /*Case 0*/
      f_tan = TUE_CML_GDB_tan32s((f_angle * (4.0F / TUE_CML_Pi)));
      break;
  }

  if (b_sign == TRUE) {
    f_tan = -f_tan;
  }

  return (f_tan);
}

/*****************************************************************************
  Functionname:    TUE_CML_GDB_tan32s */ /*!

  @brief           Computes tan(pi *x/4)

  @description     Accurate to about 3.2 decimal digits over the range [0, pi/4].
				   Note that the function computes tan(pi*x/4),
				   NOT tan(x); it's up to the range reduction algorithm that
				   calls this to scale things properly.
				   Algorithm:    tan(x)= x*c1/(c2 + x^2)

  @param[in]       f_angle : the angle (times pi/4) for which we want to know
							 the tangent, radians
							 Supported values are [-MAX_ANGLE,..,MAX_ANGLE],
							 where MAX_ANGLE is square root of max value of float32

  @return          tan(pi*f_angle/4)
*****************************************************************************/
float32 TUE_CML_GDB_tan32s(float32 f_angle) {
  /*--- VARIABLES ---*/
  float32 f_angle_square = 0.F;

  f_angle_square = TUE_CML_Sqr(f_angle);
  return ((f_angle * C_TAN_32_C1) / (C_TAN_32_C2 + f_angle_square));
}

/*****************************************************************************
  Functionname:    TUE_CML_GDBcos_52 */ /*!

  @brief           Calculates the cosine with 5.2 decimals accuracy

  @description     It reduces the input argument's range to [0, pi/2],
				   and then performs the approximation.
				   Algorithm:
				   cos(x)= c1 + c2*x^2 + c3*x^4 + c4*x^6
				   which is the same as:
				   cos(x)= c1 + x^2(c2 + c3*x^2 + c4*x^4)
				   cos(x)= c1 + x^2(c2 + x^2(c3 + c4*x^2))

  @param[in]       f_angle : angle for which cosine has to be found
							 Supported values are [-MAX_ANGLE,..,MAX_ANGLE],
							 where MAX_ANGLE = [max range of uint32] * CML_f_two_Pi

  @return          cosine of f_angle (double)
*****************************************************************************/
float32 TUE_CML_GDBcos_52(float32 f_angle) {
  /*--- VARIABLES ---*/
  uint32 u_quad = 0;
  float32 f_angle_square = 0.F;
  float32 f_tmp = 0.F;
  float32 f_resultValue = 0.F; /* result value */

  if (f_angle < 0.0F) {
    f_angle = -f_angle;
  }

  /* limit to two pi */
  if (f_angle > (10.0F * TUE_CML_Pi)) {
    f_angle = TUE_CML_ModTrig(f_angle, (2.0F * TUE_CML_Pi));
  } else {
    /* sensible argument, use faster while loop */
    while (f_angle >= (2.0F * TUE_CML_Pi)) {
      f_angle -= (2.0F * TUE_CML_Pi);
    }
  }

  f_tmp = f_angle * (2.0F / TUE_CML_Pi);
  u_quad = (uint32)f_tmp;
  switch (u_quad) {
    case 1:
      f_angle_square = (TUE_CML_Pi - f_angle) * (TUE_CML_Pi - f_angle);
      f_resultValue =
          -(C_COS_52_C1 +
            (f_angle_square *
             (C_COS_52_C2 + (f_angle_square *
                             (C_COS_52_C3 + (C_COS_52_C4 * f_angle_square))))));
      break;
    case 2:
      f_angle_square = (f_angle - TUE_CML_Pi) * (f_angle - TUE_CML_Pi);
      f_resultValue =
          -(C_COS_52_C1 +
            (f_angle_square *
             (C_COS_52_C2 + (f_angle_square *
                             (C_COS_52_C3 + (C_COS_52_C4 * f_angle_square))))));
      break;
    case 3:
      f_angle_square =
          ((2.0F * TUE_CML_Pi) - f_angle) * ((2.0F * TUE_CML_Pi) - f_angle);
      f_resultValue =
          (C_COS_52_C1 +
           (f_angle_square *
            (C_COS_52_C2 + (f_angle_square *
                            (C_COS_52_C3 + (C_COS_52_C4 * f_angle_square))))));
      break;
    default:
      /*Case 0*/
      f_angle_square = f_angle * f_angle;
      f_resultValue =
          (C_COS_52_C1 +
           (f_angle_square *
            (C_COS_52_C2 + (f_angle_square *
                            (C_COS_52_C3 + (C_COS_52_C4 * f_angle_square))))));
      break;
  }

  return f_resultValue;
}

/*****************************************************************************
  Functionname:    TUE_CML_GDBsin_52 */ /*!

  @brief           Calculates the sine with 5.2 decimals accuracy

  @description     The sine is just cosine shifted a half-pi,
				   so we'll adjust the argument and call the cosine approximation.

  @param[in]       f_angle : input angle for which we would like to know the
							 sine, radians
							 Supported values are [-MAX_ANGLE,..,MAX_ANGLE],
							 where MAX_ANGLE =
							 ([max range of uint32] * CML_f_two_Pi)-(0.5F * TUE_CML_Pi)

  @return          the sine of f_angle
*****************************************************************************/
float32 TUE_CML_GDBsin_52(float32 f_angle) {
  return TUE_CML_GDBcos_52((0.5F * TUE_CML_Pi) - f_angle);
}

/*****************************************************************************
  Functionname:    TUE_CML_GDBatan_66 */ /*!

  @brief           computes atan(x) with about 6.6 decimal digits accuracy

  @description     The input argument's range is reduced to [0, pi/12]
				   before the approximation takes place
				   Algorithm: atan(x)= x(c1 + c2*x^2)/(c3 + x^2)

  @param[in]       f_tan : the "secant length" for which we want to know the
						   corresponding angle, radians
						   Optimal values are [-MAX_ANGLE,..,MAX_ANGLE],
						   where MAX_ANGLE is square root of max value of float32

  @return          arctangent of f_tan
*****************************************************************************/
float32 TUE_CML_GDBatan_66(float32 f_tan) {
  /*--- VARIABLES ---*/
  float32 f_angle = 0.F;        /* return from atan__s function */
  float32 f_tan_square = 0.F;   /* The input argument squared */
  boolean b_complement = FALSE; /* TRUE if arg was >1 */
  boolean b_region = FALSE;     /* TRUE depending on region arg is in */
  boolean b_sign = FALSE;       /* TRUE if arg was < 0 */

  /* reduce input argument */
  if (f_tan < 0.0F) {
    f_tan = -f_tan;
    b_sign = TRUE; /* argtan(-x) = - arctan(x) */
  }
  if (f_tan > 1.0F) {
    f_tan = 1.0F / f_tan;
    b_complement = TRUE; /* keep arg between 0 and 1 */
  }
  if (f_tan > C_TANTWELFTHPI) {
    f_tan =
        ((f_tan - C_TANSIXTHPI) /
         (1.F + (C_TANSIXTHPI * f_tan))); /* reduce arg to under tan(pi/12) */
    b_region = TRUE;
  }

  /* do the approximation on the reduced argument */
  f_tan_square = TUE_CML_Sqr(f_tan);
  f_angle = (f_tan * (C_ATAN_66_C1 + (f_tan_square * C_ATAN_66_C2))) /
            (C_ATAN_66_C3 + f_tan_square);

  /* put result back together */
  if (b_region == TRUE) {
    f_angle += C_SIXTHPI; /* correct for region we are in */
  }
  if (b_complement == TRUE) {
    f_angle =
        (0.5F * TUE_CML_Pi) - f_angle; /* correct for 1/x if we did that */
  }
  if (b_sign == TRUE) {
    f_angle = -f_angle; /* correct for negative arg */
  }

  return (f_angle);
}

/*****************************************************************************
  Functionname:    TUE_CML_GDBacos_66 */ /*!

  @brief           implements the acos() function with 6.6 decimals of accuracy

  @description     This function uses the relationships between trigonomtric
				   and inverse trigonometric functions.
				   tan(arccos x) = sqrt(1 - x^2) / x
				   tan(arcsin x) = x / sqrt(1 - x^2)

  @param[in]       f_cos : value for which we want the inverse cosinus
						   Ideal values are [-1,..,0,..,1]

  @return          arccosinus corresponding to the value f_cos, in radians
*****************************************************************************/
float32 TUE_CML_GDBacos_66(float32 f_cos) {
  float32 f_angle = 0.F; /* result value */

  /*! catch invalid input ranges and prevent division by zero below */
  if (f_cos >= 1.0F) {
    f_angle = 0.0F;
  }

  else if (f_cos <= -1.0F) {
    f_angle = TUE_CML_Pi;
  }

  else {
    f_angle = (0.5F * TUE_CML_Pi) -
              TUE_CML_GDBatan_66(
                  f_cos / TUE_CML_SqrtApprox(1.0F - TUE_CML_SqrtApprox(f_cos)));
  }

  return f_angle;
}

/*****************************************************************************
  Functionname:    TUE_CML_GDBasin_66 */ /*!

  @brief           implements the asin() function with 6.6 decimals of accuracy

  @description     This function uses the relationships between trigonomtric
				   and inverse trigonometric functions.
				   tan(arccos x) = sqrt(1 - x^2) / x
				   tan(arcsin x) = x / sqrt(1 - x^2)

  @param[in]       f_sin : value for which we want the inverse sinus
						   Ideal values are [-1,..,0,..,1]

  @return          arcsinus corresponding to the value f_sin, in radians
*****************************************************************************/
float32 TUE_CML_GDBasin_66(float32 f_sin) {
  float32 f_angle = 0.F; /* result value */

  /*! catch invalid input ranges and prevent division by zero below */
  if (f_sin >= 1.0F) {
    f_angle = (0.5F * TUE_CML_Pi);
  }

  else if (f_sin <= -1.0F) {
    f_angle = -(0.5F * TUE_CML_Pi);
  }

  else {
    f_angle = TUE_CML_GDBatan_66(f_sin /
                                 TUE_CML_SqrtApprox(1.0F - TUE_CML_Sqr(f_sin)));
  }

  return f_angle;
}

/*****************************************************************************
  Functionname:    TUE_CML_BoundedLinInterpol                 */ /*!

  @brief           bounded linear interpolation between two given points

  @description     This function computes the bounded linear interpolation value
				   between two given points. The minimum and maximum boundary
				   values are taken from the input structure.

  @param[in]       p_Params : structure for parameters
							  Range for p_Params->dAmin is [Full range of float32]
							  Range for p_Params->dAmax is [Full range of float32]
							  Range for p_Params-> dM is [Full range of float32]
							  Range for p_Params-> dB is [Full range of float32]
							  Overflow may occur if all the input values to the function
							  are at maximum possible value at the same time.

  @param[in]       f_Value : x-value to interpolate
							 [Full range of float32]

  @return          the bounded interpolated value


*****************************************************************************/
float32 TUE_CML_BoundedLinInterpol(
    TUE_CML_t_LinFunctionArgs const* const p_Params, const float32 f_Value) {
  const float32 f_min = p_Params->dAmin;
  const float32 f_max = p_Params->dAmax;

  /* Geradengleichung: */
  float32 f_BoundedValue = TUE_CML_MultAdd(p_Params->dM, f_Value, p_Params->dB);

  /* Grenzwerte: */
  if (f_min < f_max) {
    /*    /-- */
    /* --/    */
    if (f_BoundedValue <= f_min) {
      f_BoundedValue = f_min;
    } else if (f_BoundedValue > f_max) {
      f_BoundedValue = f_max;
    } else {
    }
  } else {
    /* --\    */
    /*    \-- */
    if (f_BoundedValue <= f_max) {
      f_BoundedValue = f_max;
    } else if (f_BoundedValue > f_min) {
      f_BoundedValue = f_min;
    } else {
    }
  }

  return f_BoundedValue;
} /* TUE_CML_BoundedLinInterpol() */

/*****************************************************************************
  Functionname:    TUE_CML_BoundedLinInterpol2 */ /*!

  @brief

  @description

  @param[in]

  @return
*****************************************************************************/
float32 TUE_CML_BoundedLinInterpol2(float32 f_IVal, float32 f_Imin,
                                    float32 f_Imax, float32 f_Omin,
                                    float32 f_Omax) {
  float32 f_OVal = 0.F;

  if (TUE_CML_IsZero(f_Imax - f_Imin)) {
    f_OVal = (f_Omin + f_Omax) * 0.5F;
  } else {
    f_OVal =
        f_Omin + ((f_IVal - f_Imin) * ((f_Omax - f_Omin) / (f_Imax - f_Imin)));
  }

  /* Bound output */
  if (f_Omin < f_Omax) {
    f_OVal = (TUE_CML_MinMax(f_Omin, f_Omax, f_OVal));
  } else {
    f_OVal = (TUE_CML_MinMax(f_Omax, f_Omin, f_OVal));
  }
  return f_OVal;
} /* BML_f_BoundedLinInterpol2() */

/*****************************************************************************
  Functionname:    TUE_CML_Round2IntGen */ /*!

  @brief          

  @description     

  @param[in]      

  @return         
*****************************************************************************/
sint32 TUE_CML_Round2IntGen(float32 x) {
  return (x >= 0.F) ? (sint32)(x + 0.5F) : (sint32)(x - 0.5F);
}

/*****************************************************************************
  Functionname:    TUE_CML_Round2FloatGen */ /*!

  @brief

  @description

  @param[in]

  @return
*****************************************************************************/
float32 TUE_CML_Round2FloatGen(float32 x) {
  return (x >= 0.F) ? (float32)(sint32)(x + 0.5F) : (float32)(sint32)(x - 0.5F);
}

/*****************************************************************************
  Functionname:    TUE_CML_GDBatan2_66                                        */ /*!

  @brief           computes the four-quadrant atan(y/x) with
				   about 6.6 decimal digits accuracy

  @description     This function computes the four-quandrant arctangent with
				   about 6.6 decimal digits accuracy.
				   The input arguments are x and y. The situation y=0 is
				   handled correctly.

  @param[in]       f_xaxis : any number
							 Optimal values are [-MAX_ANGLE,..,MAX_ANGLE]
  @param[in]       f_yaxis : any number
							 Optimal values are [-MAX_ANGLE,..,MAX_ANGLE],
							 where MAX_ANGLE is cube root of max value of float32

  @return          the four-quadrant arctangent of f_yaxis/f_xaxis in
				   radians [-Pi, Pi]
				   if x=0 and y=0 the result is 0

*****************************************************************************/
float32 TUE_CML_GDBatan2_66(float32 f_yaxis, float32 f_xaxis) {
  float32 f_angle = 0.F;

  /* handle x = 0 */
  if (f_xaxis > TUE_CML_AlmostZero) {
    /* compute arctangent */
    f_angle = TUE_CML_GDBatan_66(f_yaxis / f_xaxis);
  }

  else {
    if (f_xaxis < TUE_CML_AlmostNegZero) {
      /* compute arctangent */
      f_angle = TUE_CML_GDBatan_66(f_yaxis / -f_xaxis);

      if (f_yaxis < TUE_CML_AlmostNegZero) {
        f_angle = -TUE_CML_Pi - f_angle;
      }

      else {
        f_angle = TUE_CML_Pi - f_angle;
      }
    }

    else {
      if (f_yaxis < TUE_CML_AlmostNegZero) {
        f_angle = -TUE_CML_Pi / 2.0F;
      }

      else if (f_yaxis > TUE_CML_AlmostZero) {
        f_angle = TUE_CML_Pi / 2.0F;
      }

      else {
        f_angle = 0.0F;
      }
    }
  }

  return f_angle;
}

/*****************************************************************************
  Functionname: SafeDiv                                  */ /*!

  @brief: Return a value which is verified not zero regardless of sign

  @description: Return a value which is verified not zero or close to zero regardless of sign

  @param[in]

  @return
*****************************************************************************/
float32 SafeDiv(float32 fDivisor) {
  if (TUE_CML_Abs(fDivisor) < TUE_C_F32_DELTA) {
    if (fDivisor < 0.F) {
      fDivisor = -TUE_C_F32_DELTA;
    } else {
      fDivisor = TUE_C_F32_DELTA;
    }
  }
  return fDivisor;
}

/*****************************************************************************
  Functionname:    SimulationObjectGenerator */ /*!

  @brief           Computes object distance and velocity value based on
				   defined object trajectory

  @description     we will calculate predicated object's detect value based on
				   defined object's trajectory. the target of this function is
				   used for unit test of object related functions.

  @param[in]        fObjTrajHeadingAngle_rad: object trajectory's heading angle of clothoid equation
					fObjTrajCurve_1pm: object trajectory's curve of clothoid equation
					fObjTrajCurveDer_nu: object trajectory's curve derivative of clothoid equation
					fObjInitDistX0_met: object initial distance X
					fObjInitDistY0_met: object inital distance Y
					fObjAbsVelX_mps: object absolute velocity X
					fEgoVelX_mps: ego velocity X
					fPassedTime_sec: passed time since object inital point

  @param[out]
					fOutObjDistX_met: object distance X in current time set
					fOutObjDistY_met: object distance Y in current time set
					fOutObjRelVelX_mps: object velocity X in current time set
					fOutObjRelVelY_mps: object velocity Y in current time set
  @return

*****************************************************************************/
void SimulationObjectGenerator(
    float32 fObjTrajHeadingAngle_rad, float32 fObjTrajCurve_1pm,
    float32 fObjTrajCurveDer_nu, float32 fObjInitDistX0_met,
    float32 fObjInitDistY0_met, float32 fObjAbsVelX_mps, float32 fEgoVelX_mps,
    float32 fPassedTime_sec, float32* fOutObjDistX_met,
    float32* fOutObjDistY_met, float32* fOutObjRelVelX_mps,
    float32* fOutObjRelVelY_mps) {
  *fOutObjRelVelX_mps = fObjAbsVelX_mps - fEgoVelX_mps;
  *fOutObjRelVelY_mps = 0.F;
  *fOutObjDistX_met =
      fObjInitDistX0_met + *fOutObjRelVelX_mps * fPassedTime_sec;
  *fOutObjDistY_met =
      fObjInitDistY0_met +
      (TUE_CML_GDBtan_52(fObjTrajHeadingAngle_rad)) * (*fOutObjDistX_met) +
      (0.5F * (*fOutObjDistX_met) * (*fOutObjDistX_met) * fObjTrajCurve_1pm) +
      ((1.0F / 6.0F) * (*fOutObjDistX_met) * (*fOutObjDistX_met) *
       (*fOutObjDistX_met) * fObjTrajCurveDer_nu);
}
