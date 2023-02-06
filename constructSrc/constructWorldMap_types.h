/**
 * @file           : constructWorldMap_types.h
 * @target         : Texas Instruments->C6000
 * @details        : for path build map algorithms
 * @author         : cuixingxing
 * @email          : xingxing.cui@long-horn.com
 * @date           : 01-Nov-2022 17:14:32
 * @version        : V1.0.0
 * @copyright      : Copyright (C) 2022 Long-Horn Inc.All rights reserved.
 */

#ifndef CONSTRUCTWORLDMAP_TYPES_H
#define CONSTRUCTWORLDMAP_TYPES_H

/** @include file    : Include Files */
#include "rtwtypes.h"

/** Type Definitions */
#ifndef typedef_imref2d
#define typedef_imref2d
typedef struct {
  double XWorldLimits[2];
  double YWorldLimits[2];
  double ImageSizeAlias[2];
  bool ForcePixelExtentToOne;
} imref2d;
#endif /* typedef_imref2d */

#ifndef typedef_cell_wrap_0
#define typedef_cell_wrap_0
typedef struct {
  double f1[18483444];
} cell_wrap_0;
#endif /* typedef_cell_wrap_0 */

#ifndef typedef_struct0_T
#define typedef_struct0_T
typedef struct {
  cell_wrap_0 undistortImages[4];
  double currFrontBasePose[3];
  bool isuseGT;
} struct0_T;
#endif /* typedef_struct0_T */

#ifndef typedef_struct4_T
#define typedef_struct4_T
typedef struct {
  double FocalLength[2];
  double PrincipalPoint[2];
  double ImageSize[2];
  double RadialDistortion[2];
  double TangentialDistortion[2];
  double Skew;
} struct4_T;
#endif /* typedef_struct4_T */

#ifndef typedef_struct3_T
#define typedef_struct3_T
typedef struct {
  struct4_T Intrinsics;
  double Height;
  double Pitch;
  double Yaw;
  double Roll;
  double SensorLocation[2];
  char WorldUnits[6];
} struct3_T;
#endif /* typedef_struct3_T */

#ifndef typedef_struct2_T
#define typedef_struct2_T
typedef struct {
  double OutputView[4];
  double ImageSize[2];
  struct3_T Sensor;
} struct2_T;
#endif /* typedef_struct2_T */

#ifndef typedef_struct5_T
#define typedef_struct5_T
typedef struct {
  double A[9];
} struct5_T;
#endif /* typedef_struct5_T */

#ifndef typedef_struct1_T
#define typedef_struct1_T
typedef struct {
  struct2_T birdsEye[4];
  struct5_T tforms[4];
} struct1_T;
#endif /* typedef_struct1_T */

#ifndef typedef_struct8_T
#define typedef_struct8_T
typedef struct {
  double ImageSize[2];
  double XWorldLimits[2];
  double YWorldLimits[2];
} struct8_T;
#endif /* typedef_struct8_T */

#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T
struct emxArray_real_T {
  double *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  bool canFreeData;
};
#endif /* struct_emxArray_real_T */
#ifndef typedef_emxArray_real_T
#define typedef_emxArray_real_T
typedef struct emxArray_real_T emxArray_real_T;
#endif /* typedef_emxArray_real_T */

#ifndef typedef_struct_T
#define typedef_struct_T
typedef struct {
  emxArray_real_T *bigImg;
  imref2d ref;
} struct_T;
#endif /* typedef_struct_T */

#ifndef struct_emxArray_boolean_T
#define struct_emxArray_boolean_T
struct emxArray_boolean_T {
  bool *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  bool canFreeData;
};
#endif /* struct_emxArray_boolean_T */
#ifndef typedef_emxArray_boolean_T
#define typedef_emxArray_boolean_T
typedef struct emxArray_boolean_T emxArray_boolean_T;
#endif /* typedef_emxArray_boolean_T */

#ifndef typedef_struct7_T
#define typedef_struct7_T
typedef struct {
  emxArray_real_T *bigImg;
  struct8_T ref;
} struct7_T;
#endif /* typedef_struct7_T */

#ifndef typedef_struct6_T
#define typedef_struct6_T
typedef struct {
  struct7_T HDmap;
  emxArray_real_T *vehicleTraj;
} struct6_T;
#endif /* typedef_struct6_T */

#ifndef typedef_cell_wrap_1
#define typedef_cell_wrap_1
typedef struct {
  emxArray_real_T *f1;
} cell_wrap_1;
#endif /* typedef_cell_wrap_1 */

#ifndef struct_emxArray_int32_T
#define struct_emxArray_int32_T
struct emxArray_int32_T {
  int *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  bool canFreeData;
};
#endif /* struct_emxArray_int32_T */
#ifndef typedef_emxArray_int32_T
#define typedef_emxArray_int32_T
typedef struct emxArray_int32_T emxArray_int32_T;
#endif /* typedef_emxArray_int32_T */

#ifndef struct_emxArray_real32_T
#define struct_emxArray_real32_T
struct emxArray_real32_T {
  float *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  bool canFreeData;
};
#endif /* struct_emxArray_real32_T */
#ifndef typedef_emxArray_real32_T
#define typedef_emxArray_real32_T
typedef struct emxArray_real32_T emxArray_real32_T;
#endif /* typedef_emxArray_real32_T */

#ifndef typedef_ORBPoints
#define typedef_ORBPoints
typedef struct {
  emxArray_real32_T *pLocation;
  emxArray_real32_T *pMetric;
  emxArray_real32_T *pScale;
  emxArray_real32_T *pOrientation;
} ORBPoints;
#endif /* typedef_ORBPoints */

#ifndef struct_emxArray_uint32_T
#define struct_emxArray_uint32_T
struct emxArray_uint32_T {
  unsigned int *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  bool canFreeData;
};
#endif /* struct_emxArray_uint32_T */
#ifndef typedef_emxArray_uint32_T
#define typedef_emxArray_uint32_T
typedef struct emxArray_uint32_T emxArray_uint32_T;
#endif /* typedef_emxArray_uint32_T */

#ifndef struct_emxArray_uint8_T
#define struct_emxArray_uint8_T
struct emxArray_uint8_T {
  unsigned char *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  bool canFreeData;
};
#endif /* struct_emxArray_uint8_T */
#ifndef typedef_emxArray_uint8_T
#define typedef_emxArray_uint8_T
typedef struct emxArray_uint8_T emxArray_uint8_T;
#endif /* typedef_emxArray_uint8_T */

#ifndef struct_emxArray_int8_T
#define struct_emxArray_int8_T
struct emxArray_int8_T {
  signed char *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  bool canFreeData;
};
#endif /* struct_emxArray_int8_T */
#ifndef typedef_emxArray_int8_T
#define typedef_emxArray_int8_T
typedef struct emxArray_int8_T emxArray_int8_T;
#endif /* typedef_emxArray_int8_T */

#endif
/**
 * File trailer for constructWorldMap_types.h
 *
 * [EOF]
 */
