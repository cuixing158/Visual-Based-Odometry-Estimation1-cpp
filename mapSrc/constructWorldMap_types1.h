#ifndef CONSTRUCTWORLDMAP_TYPES1_H
#define CONSTRUCTWORLDMAP_TYPES1_H

#include "HDMapping1.h"
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>
#include <stdio.h>

namespace buildMapping {
struct constructWorldMapPersistentData {
  coder::robotics::core::internal::BlockMatrix gobj_1[3];
  coder::binaryFeatures preFeatures;
  bool preFeatures_not_empty;
  ::coder::array<float, 2U> prePoints;
  coder::rigidtform2d preRelTform;
  bool BW[19200];
  bool BW_not_empty;
  coder::rigidtform2d previousImgPose;
  coder::rigidtform2d initViclePtPose;
  coder::rigidtform2d prePoseNodes;
  coder::poseGraph pg;
  double xLimitGlobal[2];
  double yLimitGlobal[2];
  bool isFirst;
  double preLoopCandiate;
  unsigned int state[625];
  float lookupTable[256];
  bool lookupTable_not_empty;
  double freq;
  bool freq_not_empty;
  FILE *eml_openfiles[20];
};

} // namespace buildMapping

#endif
