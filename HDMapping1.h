#ifndef HDMAPPING1_H
#define HDMAPPING1_H

#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

namespace buildMapping {
namespace coder {
namespace robotics {
namespace core {
namespace internal {
class BlockMatrix {
public:
  void replaceBlock(double i, const double blockij[9]);
  void extractBlock(double i, ::coder::array<double, 2U> &B) const;
  void replaceBlock();
  BlockMatrix();
  ~BlockMatrix();
  ::coder::array<double, 2U> Matrix;
  double NumRowBlocks;
  double NumColBlocks;
  double BlockSize[2];
};

} // namespace internal
} // namespace core
} // namespace robotics
class binaryFeatures {
public:
  binaryFeatures();
  ~binaryFeatures();
  ::coder::array<unsigned char, 2U> Features;
};

class rigidtform2d {
public:
  rigidtform2d();
  ~rigidtform2d();
  double RotationAngle;
  double Translation[2];
};

class poseGraph {
public:
  poseGraph *init(robotics::core::internal::BlockMatrix *iobj_0);
  void addRelativePose(const double varargin_1[3], double varargin_3,
                       double varargin_4);
  void findEdgeID(const double nodePair[2],
                  ::coder::array<double, 1U> &edgeID) const;
  void addRelativePose(double varargin_3, double varargin_4);
  void nodeEstimates(::coder::array<double, 2U> &nodeEsts) const;
  poseGraph();
  ~poseGraph();
  double NumNodes;
  double NumEdges;
  double NumLoopClosureEdges;
  robotics::core::internal::BlockMatrix *NodeEstimates;
  ::coder::array<double, 1U> NodeMap;
  ::coder::array<double, 1U> NodeDims;
  ::coder::array<bool, 1U> IsLandmarkNode;
  ::coder::array<double, 2U> EdgeNodePairs;
  ::coder::array<double, 2U> LoopClosureEdgeNodePairs;
  robotics::core::internal::BlockMatrix *EdgeMeasurements;
  robotics::core::internal::BlockMatrix *EdgeInfoMatrices;
  ::coder::array<double, 2U> LoopClosureEdgeIDsInternal;
  double MaxNumEdges;
  bool __MaxNumEdges_AssignmentSentinel;
  double MaxNumNodes;
  bool __MaxNumNodes_AssignmentSentinel;
};

namespace visioncodegen {
class AlphaBlender {
public:
  AlphaBlender();
  ~AlphaBlender();
  bool matlabCodegenIsDeleted;
  int isInitialized;
};

} // namespace visioncodegen
} // namespace coder
} // namespace buildMapping

#endif
