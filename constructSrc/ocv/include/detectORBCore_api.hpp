/* Copyright 2018 The MathWorks, Inc. */

#ifndef _DETECTORB_
#define _DETECTORB_

#include "vision_defines.h"

EXTERN_C LIBMWCVSTRT_API int32_T detectORBComputeCM(uint8_T *inImg,
        int32_T nRows, int32_T nCols,
        int numFeatures,float scaleFactor,
        int numLevels,int edgeThreshold,int firstLevel,
        int samplingPairs,int scoreType,int patchSize,int fastThreshold,
        void **outKeypoints);

EXTERN_C LIBMWCVSTRT_API int32_T detectORBComputeRM(uint8_T *inImg,
        int32_T nRows, int32_T nCols,
        int numFeatures,float scaleFactor,
        int numLevels,int edgeThreshold,int firstLevel,
        int samplingPairs,int scoreType,int patchSize,int fastThreshold,
        void **outKeypoints);

EXTERN_C LIBMWCVSTRT_API void detectORBAssignOutputCM(
        void *ptrKeypoints,
        real32_T *outLoc,
        real32_T *outOri,
        real32_T *outMet,
        real32_T *outScl);


EXTERN_C LIBMWCVSTRT_API void detectORBAssignOutputRM(
        void *ptrKeypoints,
        real32_T *outLoc,
        real32_T *outOri,
        real32_T *outMet,
        real32_T *outScl);

#endif
