/**
 * @file           : constructWorldMap_emxAPI.c
 * @target         : Texas Instruments->C6000
 * @details        : for path build map algorithms
 * @author         : cuixingxing
 * @email          : xingxing.cui@long-horn.com
 * @date           : 09-Feb-2023 16:58:33
 * @version        : V1.0.0
 * @copyright      : Copyright (C) 2023 Long-Horn Inc.All rights reserved.
 */

/** @include file    : Include Files */
#include "constructWorldMap_emxAPI.h"
#include "constructWorldMap_emxutil.h"
#include "constructWorldMap_types.h"
#include "rt_nonfinite.h"
#include <stdlib.h>

/** Function Definitions */
/**
 * @fn             : emxCreateND_real_T
 * @brief          :
 * @param          : int numDimensions
 *                   const int *size
 * @return         : emxArray_real_T *
 */
emxArray_real_T *emxCreateND_real_T(int numDimensions, const int *size)
{
  emxArray_real_T *emx;
  int i;
  int numEl;
  emxInit_real_T(&emx, numDimensions);
  numEl = 1;
  for (i = 0; i < numDimensions; i++) {
    numEl *= size[i];
    emx->size[i] = size[i];
  }
  emx->data = (double *)calloc((unsigned int)numEl, sizeof(double));
  emx->numDimensions = numDimensions;
  emx->allocatedSize = numEl;
  return emx;
}

/**
 * @fn             : emxCreateWrapperND_real_T
 * @brief          :
 * @param          : double *data
 *                   int numDimensions
 *                   const int *size
 * @return         : emxArray_real_T *
 */
emxArray_real_T *emxCreateWrapperND_real_T(double *data, int numDimensions,
                                           const int *size)
{
  emxArray_real_T *emx;
  int i;
  int numEl;
  emxInit_real_T(&emx, numDimensions);
  numEl = 1;
  for (i = 0; i < numDimensions; i++) {
    numEl *= size[i];
    emx->size[i] = size[i];
  }
  emx->data = data;
  emx->numDimensions = numDimensions;
  emx->allocatedSize = numEl;
  emx->canFreeData = false;
  return emx;
}

/**
 * @fn             : emxCreateWrapper_real_T
 * @brief          :
 * @param          : double *data
 *                   int rows
 *                   int cols
 * @return         : emxArray_real_T *
 */
emxArray_real_T *emxCreateWrapper_real_T(double *data, int rows, int cols)
{
  emxArray_real_T *emx;
  emxInit_real_T(&emx, 2);
  emx->size[0] = rows;
  emx->size[1] = cols;
  emx->data = data;
  emx->numDimensions = 2;
  emx->allocatedSize = rows * cols;
  emx->canFreeData = false;
  return emx;
}

/**
 * @fn             : emxCreate_real_T
 * @brief          :
 * @param          : int rows
 *                   int cols
 * @return         : emxArray_real_T *
 */
emxArray_real_T *emxCreate_real_T(int rows, int cols)
{
  emxArray_real_T *emx;
  int numEl;
  emxInit_real_T(&emx, 2);
  emx->size[0] = rows;
  numEl = rows * cols;
  emx->size[1] = cols;
  emx->data = (double *)calloc((unsigned int)numEl, sizeof(double));
  emx->numDimensions = 2;
  emx->allocatedSize = numEl;
  return emx;
}

/**
 * @fn             : emxDestroyArray_real_T
 * @brief          :
 * @param          : emxArray_real_T *emxArray
 * @return         : void
 */
void emxDestroyArray_real_T(emxArray_real_T *emxArray)
{
  emxFree_real_T(&emxArray);
}

/**
 * @fn             : emxDestroy_struct6_T
 * @brief          :
 * @param          : struct6_T emxArray
 * @return         : void
 */
void emxDestroy_struct6_T(struct6_T emxArray)
{
  emxFreeStruct_struct6_T(&emxArray);
}

/**
 * @fn             : emxInit_struct6_T
 * @brief          :
 * @param          : struct6_T *pStruct
 * @return         : void
 */
void emxInit_struct6_T(struct6_T *pStruct)
{
  emxInitStruct_struct6_T(pStruct);
}

/**
 * File trailer for constructWorldMap_emxAPI.c
 *
 * [EOF]
 */
