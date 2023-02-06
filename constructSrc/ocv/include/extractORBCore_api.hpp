/* Copyright 2018 The MathWorks, Inc. */

#ifndef _EXTRACTORB_
#define _EXTRACTORB_

#include "vision_defines.h"

EXTERN_C LIBMWCVSTRT_API int32_T extractORBComputeCM(
        uint8_T *inImg,
        int32_T nRows, int32_T nCols,
        real32_T *inLoc,
        real32_T *inOri,
        real32_T *inMet,
        real32_T *inScl,
        int numKPts,
        int numFeatures, float scaleFactor,
        int numLevels, int edgeThreshold, int firstLevel,
        int samplingPairs, int scoreType, int patchSize, int fastThreshold,
        void **outKeypoints, void **outFeatures);

EXTERN_C LIBMWCVSTRT_API int32_T extractORBComputeRM(
        uint8_T *inImg,
        int32_T nRows, int32_T nCols,
        real32_T *inLoc,
        real32_T *inOri,
        real32_T *inMet,
        real32_T *inScl,
        int numKPts,
        int numFeatures, float scaleFactor,
        int numLevels, int edgeThreshold, int firstLevel,
        int samplingPairs, int scoreType, int patchSize, int fastThreshold,
        void **outKeypoints, void **outFeatures);

EXTERN_C LIBMWCVSTRT_API void extractORBAssignOutputCM(
        void * ptrKeypoints,
        void * ptrFeatures,
        real32_T *outLoc,
        real32_T *outOri,
        real32_T *outMet,
        real32_T *outScl,
        uint8_T *outFtrs);

EXTERN_C LIBMWCVSTRT_API void extractORBAssignOutputRM(
        void * ptrKeypoints,
        void * ptrFeatures,
        real32_T *outLoc,
        real32_T *outOri,
        real32_T *outMet,
        real32_T *outScl,
        uint8_T *outFtrs);

#endif
