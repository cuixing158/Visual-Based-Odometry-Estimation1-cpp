/**
 * @file           : rtGetNaN.c
 * @target         : Texas Instruments->C6000
 * @details        : for path build map algorithms
 * @author         : cuixingxing
 * @email          : xingxing.cui@long-horn.com
 * @date           : 09-Feb-2023 16:58:33
 * @version        : V1.0.0
 * @copyright      : Copyright (C) 2023 Long-Horn Inc.All rights reserved.
 */

/*
 * Abstract:
 *       MATLAB for code generation function to initialize non-finite, NaN
 */
/** @include file    : Include Files */
#include "rtGetNaN.h"
#include "rt_nonfinite.h"

/*
 * Function: rtGetNaN
 * ======================================================================
 *  Abstract:
 * Initialize rtNaN needed by the generated code.
 *  NaN is initialized as non-signaling. Assumes IEEE.
 */
real_T rtGetNaN(void)
{
  return rtNaN;
}

/*
 * Function: rtGetNaNF
 * =====================================================================
 *  Abstract:
 *  Initialize rtNaNF needed by the generated code.
 *  NaN is initialized as non-signaling. Assumes IEEE
 */
real32_T rtGetNaNF(void)
{
  return rtNaNF;
}

/**
 * File trailer for rtGetNaN.c
 *
 * [EOF]
 */
