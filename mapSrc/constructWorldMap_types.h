#ifndef CONSTRUCTWORLDMAP_TYPES_H
#define CONSTRUCTWORLDMAP_TYPES_H

#include "rtwtypes.h"
#include "coder_array.h"
#include "opencvAPI.h"
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

struct struct2_T {
  ::coder::array<unsigned char, 2U> bigImg;
  imref2d_ ref;
};

struct struct1_T {
  struct2_T HDmap;
  ::coder::array<double, 2U> vehiclePoses;
  double cumDist;
  double pixelExtentInWorldXY;
  bool isOver;
};

struct imresize {
  unsigned char out[76800];
};

struct helperDetectAndExtractFeatures {
  unsigned char Iu8[307200];
};

struct fuseOptimizeHDMap {
  unsigned char currImg[307200];
};

struct constructWorldMapStackData {
  imresize f0;
  helperDetectAndExtractFeatures f1;
  fuseOptimizeHDMap f2;
  constructWorldMapPersistentData *pd;
};

} // namespace buildMapping

#endif
