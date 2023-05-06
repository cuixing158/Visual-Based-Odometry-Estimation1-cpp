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

struct uint80m_T {
  unsigned long chunks[2];
};

struct uint160m_T {
  unsigned long chunks[4];
};

struct d_struct_T {
  ::coder::array<unsigned char, 2U> Features;
  ::coder::array<double, 2U> Points;
};

struct cell_wrap_0 {
  ::coder::array<unsigned char, 2U> f1;
};

struct cell_wrap_1 {
  ::coder::array<double, 2U> f1;
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
  bool isBuildMap;
  double buildMapStopFrame;
  bool isBuildMapOver;
  bool isLocSuccess;
  double locVehiclePose[3];
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
