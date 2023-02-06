/**
 * @file           : constructWorldMap.h
 * @target         : Texas Instruments->C6000
 * @details        : for path build map algorithms
 * @author         : cuixingxing
 * @email          : xingxing.cui@long-horn.com
 * @date           : 01-Nov-2022 17:14:32
 * @version        : V1.0.0
 * @copyright      : Copyright (C) 2022 Long-Horn Inc.All rights reserved.
 */

#ifndef CONSTRUCTWORLDMAP_H
#define CONSTRUCTWORLDMAP_H

/** @include file    : Include Files */
#include "constructWorldMap_types.h"
#include "rtwtypes.h"
#include "omp.h"
#include <stddef.h>
#include <stdlib.h>

/** Variable Declarations */
extern omp_nest_lock_t constructWorldMap_nestLockGlobal;

#ifdef __cplusplus
extern "C" {
#endif

/** Function Declarations */
extern void constructWorldMap(const struct0_T *inputArgs,
                              const struct1_T *birdsEye360,
                              struct6_T *outputStruct);

extern void constructWorldMap_initialize(void);

extern void constructWorldMap_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/**
 * File trailer for constructWorldMap.h
 *
 * [EOF]
 */
