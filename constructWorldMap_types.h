#ifndef CONSTRUCTWORLDMAP_TYPES_H
#define CONSTRUCTWORLDMAP_TYPES_H

#include "rtwtypes.h"
#include "coder_array.h"
#define MAX_THREADS omp_get_max_threads()

namespace buildMapping {
struct constructWorldMapPersistentData;

}

namespace buildMapping {
struct struct0_T {
  unsigned char undistortImage[307200];
  double currFrontBasePose[3];
  bool isuseGT;
};

struct struct3_T {
  double XWorldLimits[2];
  double YWorldLimits[2];
  double ImageSize[2];
};

struct struct2_T {
  ::coder::array<unsigned char, 2U> bigImg;
  struct3_T ref;
};

struct struct1_T {
  struct2_T HDmap;
  ::coder::array<double, 2U> vehiclePoses;
  bool isOver;
};

struct remapAndResampleGeneric2d {
  float inputImage[307200];
};

struct helperDetectAndExtractFeatures {
  unsigned char Iu8[307200];
};

struct fuseOptimizeHDMap {
  unsigned char currImg[307200];
  bool temp[307200];
};

struct constructWorldMap {
  bool bv[307200];
};

struct constructWorldMapStackData {
  remapAndResampleGeneric2d f0;
  remapAndResampleGeneric2d f1;
  helperDetectAndExtractFeatures f2;
  fuseOptimizeHDMap f3;
  constructWorldMap f4;
  constructWorldMapPersistentData *pd;
};

} // namespace buildMapping

#endif
