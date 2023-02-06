/**
 * @file           : constructWorldMap_emxAPI.h
 * @target         : Texas Instruments->C6000
 * @details        : for path build map algorithms
 * @author         : cuixingxing
 * @email          : xingxing.cui@long-horn.com
 * @date           : 01-Nov-2022 17:14:32
 * @version        : V1.0.0
 * @copyright      : Copyright (C) 2022 Long-Horn Inc.All rights reserved.
 */

#ifndef CONSTRUCTWORLDMAP_EMXAPI_H
#define CONSTRUCTWORLDMAP_EMXAPI_H

/** @include file    : Include Files */
#include "constructWorldMap_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Function Declarations */
extern emxArray_real_T *emxCreateND_real_T(int numDimensions, const int *size);

extern emxArray_real_T *
emxCreateWrapperND_real_T(double *data, int numDimensions, const int *size);

extern emxArray_real_T *emxCreateWrapper_real_T(double *data, int rows,
                                                int cols);

extern emxArray_real_T *emxCreate_real_T(int rows, int cols);

extern void emxDestroyArray_real_T(emxArray_real_T *emxArray);

extern void emxDestroy_struct6_T(struct6_T emxArray);

extern void emxInit_struct6_T(struct6_T *pStruct);

#ifdef __cplusplus
}
#endif

#endif
/**
 * File trailer for constructWorldMap_emxAPI.h
 *
 * [EOF]
 */
