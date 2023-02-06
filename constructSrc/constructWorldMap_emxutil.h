/**
 * @file           : constructWorldMap_emxutil.h
 * @target         : Texas Instruments->C6000
 * @details        : for path build map algorithms
 * @author         : cuixingxing
 * @email          : xingxing.cui@long-horn.com
 * @date           : 01-Nov-2022 17:14:32
 * @version        : V1.0.0
 * @copyright      : Copyright (C) 2022 Long-Horn Inc.All rights reserved.
 */

#ifndef CONSTRUCTWORLDMAP_EMXUTIL_H
#define CONSTRUCTWORLDMAP_EMXUTIL_H

/** @include file    : Include Files */
#include "constructWorldMap_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Function Declarations */
extern void emxEnsureCapacity_boolean_T(emxArray_boolean_T *emxArray,
                                        int oldNumel);

extern void emxEnsureCapacity_int32_T(emxArray_int32_T *emxArray, int oldNumel);

extern void emxEnsureCapacity_int8_T(emxArray_int8_T *emxArray, int oldNumel);

extern void emxEnsureCapacity_real32_T(emxArray_real32_T *emxArray,
                                       int oldNumel);

extern void emxEnsureCapacity_real_T(emxArray_real_T *emxArray, int oldNumel);

extern void emxEnsureCapacity_uint32_T(emxArray_uint32_T *emxArray,
                                       int oldNumel);

extern void emxEnsureCapacity_uint8_T(emxArray_uint8_T *emxArray, int oldNumel);

extern void emxFreeMatrix_cell_wrap_1(cell_wrap_1 pMatrix[4]);

extern void emxFreeStruct_ORBPoints(ORBPoints *pStruct);

extern void emxFreeStruct_ORBPoints1(ORBPoints *pStruct);

extern void emxFreeStruct_cell_wrap_1(cell_wrap_1 *pStruct);

extern void emxFreeStruct_struct6_T(struct6_T *pStruct);

extern void emxFreeStruct_struct7_T(struct7_T *pStruct);

extern void emxFreeStruct_struct_T(struct_T *pStruct);

extern void emxFree_boolean_T(emxArray_boolean_T **pEmxArray);

extern void emxFree_int32_T(emxArray_int32_T **pEmxArray);

extern void emxFree_int8_T(emxArray_int8_T **pEmxArray);

extern void emxFree_real32_T(emxArray_real32_T **pEmxArray);

extern void emxFree_real_T(emxArray_real_T **pEmxArray);

extern void emxFree_uint32_T(emxArray_uint32_T **pEmxArray);

extern void emxFree_uint8_T(emxArray_uint8_T **pEmxArray);

extern void emxInitMatrix_cell_wrap_1(cell_wrap_1 pMatrix[4]);

extern void emxInitStruct_ORBPoints(ORBPoints *pStruct);

extern void emxInitStruct_ORBPoints1(ORBPoints *pStruct);

extern void emxInitStruct_cell_wrap_1(cell_wrap_1 *pStruct);

extern void emxInitStruct_struct6_T(struct6_T *pStruct);

extern void emxInitStruct_struct7_T(struct7_T *pStruct);

extern void emxInitStruct_struct_T(struct_T *pStruct);

extern void emxInit_boolean_T(emxArray_boolean_T **pEmxArray,
                              int numDimensions);

extern void emxInit_int32_T(emxArray_int32_T **pEmxArray, int numDimensions);

extern void emxInit_int8_T(emxArray_int8_T **pEmxArray);

extern void emxInit_real32_T(emxArray_real32_T **pEmxArray, int numDimensions);

extern void emxInit_real_T(emxArray_real_T **pEmxArray, int numDimensions);

extern void emxInit_uint32_T(emxArray_uint32_T **pEmxArray);

extern void emxInit_uint8_T(emxArray_uint8_T **pEmxArray);

#ifdef __cplusplus
}
#endif

#endif
/**
 * File trailer for constructWorldMap_emxutil.h
 *
 * [EOF]
 */
