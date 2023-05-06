#ifndef CONSTRUCTWORLDMAP_TYPES1_H
#define CONSTRUCTWORLDMAP_TYPES1_H

#include "HDMapping1.h"
#include "constructWorldMap_types.h"
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdio>
#include <cstdlib>

namespace buildMapping {
struct constructWorldMapPersistentData {
  coder::robotics::core::internal::BlockMatrix gobj_1[3];
  coder::binaryFeatures preFeatures;
  ::coder::array<double, 2U> prePoints;
  coder::rigidtform2d preRelTform;
  bool BW[19200];
  bool BW_not_empty;
  coder::rigidtform2d previousImgPose;
  coder::rigidtform2d initViclePtPose;
  coder::rigidtform2d prePoseNodes;
  coder::poseGraph pg;
  double xLimitGlobal[2];
  double yLimitGlobal[2];
  double currFrameIdx;
  bool isFirst;
  ::coder::array<d_struct_T, 2U> imageViewSt;
  unsigned int state[625];
  float lookupTable[256];
  bool lookupTable_not_empty;
  double freq;
  bool freq_not_empty;
  std::FILE *eml_openfiles[20];
  bool eml_autoflush[20];
  bool isLoad_not_empty;
  ::coder::array<cell_wrap_0, 2U> initFeaturesCell;
  ::coder::array<cell_wrap_1, 2U> featuresPoints;
  struct2_T hdmap;
  ::coder::array<double, 2U> vposes;
};

} // namespace buildMapping

#endif
