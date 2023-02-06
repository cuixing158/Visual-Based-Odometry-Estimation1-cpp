/**
 * @file           : constructWorldMap_emxutil.c
 * @target         : Texas Instruments->C6000
 * @details        : for path build map algorithms
 * @author         : cuixingxing
 * @email          : xingxing.cui@long-horn.com
 * @date           : 01-Nov-2022 17:14:32
 * @version        : V1.0.0
 * @copyright      : Copyright (C) 2022 Long-Horn Inc.All rights reserved.
 */

/** @include file    : Include Files */
#include "constructWorldMap_emxutil.h"
#include "constructWorldMap_types.h"
#include "rt_nonfinite.h"
#include <stdlib.h>
#include <string.h>

/** Function Definitions */
/**
 * @fn             : emxEnsureCapacity_boolean_T
 * @brief          :
 * @param          : emxArray_boolean_T *emxArray
 *                   int oldNumel
 * @return         : void
 */
void emxEnsureCapacity_boolean_T(emxArray_boolean_T *emxArray, int oldNumel)
{
  int i;
  int newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }
  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }
  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }
    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i *= 2;
      }
    }
    newData = calloc((unsigned int)i, sizeof(bool));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(bool) * (unsigned int)oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }
    emxArray->data = (bool *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

/**
 * @fn             : emxEnsureCapacity_int32_T
 * @brief          :
 * @param          : emxArray_int32_T *emxArray
 *                   int oldNumel
 * @return         : void
 */
void emxEnsureCapacity_int32_T(emxArray_int32_T *emxArray, int oldNumel)
{
  int i;
  int newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }
  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }
  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }
    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i *= 2;
      }
    }
    newData = calloc((unsigned int)i, sizeof(int));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(int) * (unsigned int)oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }
    emxArray->data = (int *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

/**
 * @fn             : emxEnsureCapacity_int8_T
 * @brief          :
 * @param          : emxArray_int8_T *emxArray
 *                   int oldNumel
 * @return         : void
 */
void emxEnsureCapacity_int8_T(emxArray_int8_T *emxArray, int oldNumel)
{
  int i;
  int newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }
  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }
  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }
    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i *= 2;
      }
    }
    newData = calloc((unsigned int)i, sizeof(signed char));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data,
             sizeof(signed char) * (unsigned int)oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }
    emxArray->data = (signed char *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

/**
 * @fn             : emxEnsureCapacity_real32_T
 * @brief          :
 * @param          : emxArray_real32_T *emxArray
 *                   int oldNumel
 * @return         : void
 */
void emxEnsureCapacity_real32_T(emxArray_real32_T *emxArray, int oldNumel)
{
  int i;
  int newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }
  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }
  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }
    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i *= 2;
      }
    }
    newData = calloc((unsigned int)i, sizeof(float));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(float) * (unsigned int)oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }
    emxArray->data = (float *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

/**
 * @fn             : emxEnsureCapacity_real_T
 * @brief          :
 * @param          : emxArray_real_T *emxArray
 *                   int oldNumel
 * @return         : void
 */
void emxEnsureCapacity_real_T(emxArray_real_T *emxArray, int oldNumel)
{
  int i;
  int newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }
  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }
  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }
    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i *= 2;
      }
    }
    newData = calloc((unsigned int)i, sizeof(double));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(double) * (unsigned int)oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }
    emxArray->data = (double *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

/**
 * @fn             : emxEnsureCapacity_uint32_T
 * @brief          :
 * @param          : emxArray_uint32_T *emxArray
 *                   int oldNumel
 * @return         : void
 */
void emxEnsureCapacity_uint32_T(emxArray_uint32_T *emxArray, int oldNumel)
{
  int i;
  int newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }
  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }
  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }
    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i *= 2;
      }
    }
    newData = calloc((unsigned int)i, sizeof(unsigned int));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data,
             sizeof(unsigned int) * (unsigned int)oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }
    emxArray->data = (unsigned int *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

/**
 * @fn             : emxEnsureCapacity_uint8_T
 * @brief          :
 * @param          : emxArray_uint8_T *emxArray
 *                   int oldNumel
 * @return         : void
 */
void emxEnsureCapacity_uint8_T(emxArray_uint8_T *emxArray, int oldNumel)
{
  int i;
  int newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }
  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }
  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }
    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i *= 2;
      }
    }
    newData = calloc((unsigned int)i, sizeof(unsigned char));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data,
             sizeof(unsigned char) * (unsigned int)oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }
    emxArray->data = (unsigned char *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

/**
 * @fn             : emxFreeMatrix_cell_wrap_1
 * @brief          :
 * @param          : cell_wrap_1 pMatrix[4]
 * @return         : void
 */
void emxFreeMatrix_cell_wrap_1(cell_wrap_1 pMatrix[4])
{
  int i;
  for (i = 0; i < 4; i++) {
    emxFreeStruct_cell_wrap_1(&pMatrix[i]);
  }
}

/**
 * @fn             : emxFreeStruct_ORBPoints
 * @brief          :
 * @param          : ORBPoints *pStruct
 * @return         : void
 */
void emxFreeStruct_ORBPoints(ORBPoints *pStruct)
{
  emxFree_real32_T(&pStruct->pLocation);
  emxFree_real32_T(&pStruct->pMetric);
  emxFree_real32_T(&pStruct->pScale);
  emxFree_real32_T(&pStruct->pOrientation);
}

/**
 * @fn             : emxFreeStruct_ORBPoints1
 * @brief          :
 * @param          : ORBPoints *pStruct
 * @return         : void
 */
void emxFreeStruct_ORBPoints1(ORBPoints *pStruct)
{
  emxFree_real32_T(&pStruct->pLocation);
  emxFree_real32_T(&pStruct->pMetric);
  emxFree_real32_T(&pStruct->pScale);
  emxFree_real32_T(&pStruct->pOrientation);
}

/**
 * @fn             : emxFreeStruct_cell_wrap_1
 * @brief          :
 * @param          : cell_wrap_1 *pStruct
 * @return         : void
 */
void emxFreeStruct_cell_wrap_1(cell_wrap_1 *pStruct)
{
  emxFree_real_T(&pStruct->f1);
}

/**
 * @fn             : emxFreeStruct_struct6_T
 * @brief          :
 * @param          : struct6_T *pStruct
 * @return         : void
 */
void emxFreeStruct_struct6_T(struct6_T *pStruct)
{
  emxFreeStruct_struct7_T(&pStruct->HDmap);
  emxFree_real_T(&pStruct->vehicleTraj);
}

/**
 * @fn             : emxFreeStruct_struct7_T
 * @brief          :
 * @param          : struct7_T *pStruct
 * @return         : void
 */
void emxFreeStruct_struct7_T(struct7_T *pStruct)
{
  emxFree_real_T(&pStruct->bigImg);
}

/**
 * @fn             : emxFreeStruct_struct_T
 * @brief          :
 * @param          : struct_T *pStruct
 * @return         : void
 */
void emxFreeStruct_struct_T(struct_T *pStruct)
{
  emxFree_real_T(&pStruct->bigImg);
}

/**
 * @fn             : emxFree_boolean_T
 * @brief          :
 * @param          : emxArray_boolean_T **pEmxArray
 * @return         : void
 */
void emxFree_boolean_T(emxArray_boolean_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_boolean_T *)NULL) {
    if (((*pEmxArray)->data != (bool *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }
    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_boolean_T *)NULL;
  }
}

/**
 * @fn             : emxFree_int32_T
 * @brief          :
 * @param          : emxArray_int32_T **pEmxArray
 * @return         : void
 */
void emxFree_int32_T(emxArray_int32_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_int32_T *)NULL) {
    if (((*pEmxArray)->data != (int *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }
    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_int32_T *)NULL;
  }
}

/**
 * @fn             : emxFree_int8_T
 * @brief          :
 * @param          : emxArray_int8_T **pEmxArray
 * @return         : void
 */
void emxFree_int8_T(emxArray_int8_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_int8_T *)NULL) {
    if (((*pEmxArray)->data != (signed char *)NULL) &&
        (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }
    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_int8_T *)NULL;
  }
}

/**
 * @fn             : emxFree_real32_T
 * @brief          :
 * @param          : emxArray_real32_T **pEmxArray
 * @return         : void
 */
void emxFree_real32_T(emxArray_real32_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_real32_T *)NULL) {
    if (((*pEmxArray)->data != (float *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }
    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_real32_T *)NULL;
  }
}

/**
 * @fn             : emxFree_real_T
 * @brief          :
 * @param          : emxArray_real_T **pEmxArray
 * @return         : void
 */
void emxFree_real_T(emxArray_real_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_real_T *)NULL) {
    if (((*pEmxArray)->data != (double *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }
    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_real_T *)NULL;
  }
}

/**
 * @fn             : emxFree_uint32_T
 * @brief          :
 * @param          : emxArray_uint32_T **pEmxArray
 * @return         : void
 */
void emxFree_uint32_T(emxArray_uint32_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_uint32_T *)NULL) {
    if (((*pEmxArray)->data != (unsigned int *)NULL) &&
        (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }
    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_uint32_T *)NULL;
  }
}

/**
 * @fn             : emxFree_uint8_T
 * @brief          :
 * @param          : emxArray_uint8_T **pEmxArray
 * @return         : void
 */
void emxFree_uint8_T(emxArray_uint8_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_uint8_T *)NULL) {
    if (((*pEmxArray)->data != (unsigned char *)NULL) &&
        (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }
    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_uint8_T *)NULL;
  }
}

/**
 * @fn             : emxInitMatrix_cell_wrap_1
 * @brief          :
 * @param          : cell_wrap_1 pMatrix[4]
 * @return         : void
 */
void emxInitMatrix_cell_wrap_1(cell_wrap_1 pMatrix[4])
{
  int i;
  for (i = 0; i < 4; i++) {
    emxInitStruct_cell_wrap_1(&pMatrix[i]);
  }
}

/**
 * @fn             : emxInitStruct_ORBPoints
 * @brief          :
 * @param          : ORBPoints *pStruct
 * @return         : void
 */
void emxInitStruct_ORBPoints(ORBPoints *pStruct)
{
  emxInit_real32_T(&pStruct->pLocation, 2);
  emxInit_real32_T(&pStruct->pMetric, 1);
  emxInit_real32_T(&pStruct->pScale, 1);
  emxInit_real32_T(&pStruct->pOrientation, 1);
}

/**
 * @fn             : emxInitStruct_ORBPoints1
 * @brief          :
 * @param          : ORBPoints *pStruct
 * @return         : void
 */
void emxInitStruct_ORBPoints1(ORBPoints *pStruct)
{
  emxInit_real32_T(&pStruct->pLocation, 2);
  emxInit_real32_T(&pStruct->pMetric, 1);
  emxInit_real32_T(&pStruct->pScale, 1);
  emxInit_real32_T(&pStruct->pOrientation, 1);
}

/**
 * @fn             : emxInitStruct_cell_wrap_1
 * @brief          :
 * @param          : cell_wrap_1 *pStruct
 * @return         : void
 */
void emxInitStruct_cell_wrap_1(cell_wrap_1 *pStruct)
{
  emxInit_real_T(&pStruct->f1, 3);
}

/**
 * @fn             : emxInitStruct_struct6_T
 * @brief          :
 * @param          : struct6_T *pStruct
 * @return         : void
 */
void emxInitStruct_struct6_T(struct6_T *pStruct)
{
  emxInitStruct_struct7_T(&pStruct->HDmap);
  emxInit_real_T(&pStruct->vehicleTraj, 2);
}

/**
 * @fn             : emxInitStruct_struct7_T
 * @brief          :
 * @param          : struct7_T *pStruct
 * @return         : void
 */
void emxInitStruct_struct7_T(struct7_T *pStruct)
{
  emxInit_real_T(&pStruct->bigImg, 3);
}

/**
 * @fn             : emxInitStruct_struct_T
 * @brief          :
 * @param          : struct_T *pStruct
 * @return         : void
 */
void emxInitStruct_struct_T(struct_T *pStruct)
{
  emxInit_real_T(&pStruct->bigImg, 3);
}

/**
 * @fn             : emxInit_boolean_T
 * @brief          :
 * @param          : emxArray_boolean_T **pEmxArray
 *                   int numDimensions
 * @return         : void
 */
void emxInit_boolean_T(emxArray_boolean_T **pEmxArray, int numDimensions)
{
  emxArray_boolean_T *emxArray;
  int i;
  *pEmxArray = (emxArray_boolean_T *)malloc(sizeof(emxArray_boolean_T));
  emxArray = *pEmxArray;
  emxArray->data = (bool *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int *)malloc(sizeof(int) * (unsigned int)numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

/**
 * @fn             : emxInit_int32_T
 * @brief          :
 * @param          : emxArray_int32_T **pEmxArray
 *                   int numDimensions
 * @return         : void
 */
void emxInit_int32_T(emxArray_int32_T **pEmxArray, int numDimensions)
{
  emxArray_int32_T *emxArray;
  int i;
  *pEmxArray = (emxArray_int32_T *)malloc(sizeof(emxArray_int32_T));
  emxArray = *pEmxArray;
  emxArray->data = (int *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int *)malloc(sizeof(int) * (unsigned int)numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

/**
 * @fn             : emxInit_int8_T
 * @brief          :
 * @param          : emxArray_int8_T **pEmxArray
 * @return         : void
 */
void emxInit_int8_T(emxArray_int8_T **pEmxArray)
{
  emxArray_int8_T *emxArray;
  int i;
  *pEmxArray = (emxArray_int8_T *)malloc(sizeof(emxArray_int8_T));
  emxArray = *pEmxArray;
  emxArray->data = (signed char *)NULL;
  emxArray->numDimensions = 2;
  emxArray->size = (int *)malloc(sizeof(int) * 2U);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < 2; i++) {
    emxArray->size[i] = 0;
  }
}

/**
 * @fn             : emxInit_real32_T
 * @brief          :
 * @param          : emxArray_real32_T **pEmxArray
 *                   int numDimensions
 * @return         : void
 */
void emxInit_real32_T(emxArray_real32_T **pEmxArray, int numDimensions)
{
  emxArray_real32_T *emxArray;
  int i;
  *pEmxArray = (emxArray_real32_T *)malloc(sizeof(emxArray_real32_T));
  emxArray = *pEmxArray;
  emxArray->data = (float *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int *)malloc(sizeof(int) * (unsigned int)numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

/**
 * @fn             : emxInit_real_T
 * @brief          :
 * @param          : emxArray_real_T **pEmxArray
 *                   int numDimensions
 * @return         : void
 */
void emxInit_real_T(emxArray_real_T **pEmxArray, int numDimensions)
{
  emxArray_real_T *emxArray;
  int i;
  *pEmxArray = (emxArray_real_T *)malloc(sizeof(emxArray_real_T));
  emxArray = *pEmxArray;
  emxArray->data = (double *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int *)malloc(sizeof(int) * (unsigned int)numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

/**
 * @fn             : emxInit_uint32_T
 * @brief          :
 * @param          : emxArray_uint32_T **pEmxArray
 * @return         : void
 */
void emxInit_uint32_T(emxArray_uint32_T **pEmxArray)
{
  emxArray_uint32_T *emxArray;
  int i;
  *pEmxArray = (emxArray_uint32_T *)malloc(sizeof(emxArray_uint32_T));
  emxArray = *pEmxArray;
  emxArray->data = (unsigned int *)NULL;
  emxArray->numDimensions = 2;
  emxArray->size = (int *)malloc(sizeof(int) * 2U);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < 2; i++) {
    emxArray->size[i] = 0;
  }
}

/**
 * @fn             : emxInit_uint8_T
 * @brief          :
 * @param          : emxArray_uint8_T **pEmxArray
 * @return         : void
 */
void emxInit_uint8_T(emxArray_uint8_T **pEmxArray)
{
  emxArray_uint8_T *emxArray;
  int i;
  *pEmxArray = (emxArray_uint8_T *)malloc(sizeof(emxArray_uint8_T));
  emxArray = *pEmxArray;
  emxArray->data = (unsigned char *)NULL;
  emxArray->numDimensions = 2;
  emxArray->size = (int *)malloc(sizeof(int) * 2U);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < 2; i++) {
    emxArray->size[i] = 0;
  }
}

/**
 * File trailer for constructWorldMap_emxutil.c
 *
 * [EOF]
 */
