#include "HDMapping.h"
#include "HDMapping1.h"
#include "constructWorldMap_types.h"
#include "constructWorldMap_types1.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include "coder_bounded_array.h"
#include "coder_posix_time.h"
#include "cs.h"
#include "detectORBCore_api.hpp"
#include "extractORBCore_api.hpp"
#include "loopDatabase_x86_64.h"
#include "makeCXSparseMatrix.h"
#include "omp.h"
#include "opencvAPI.h"
#include "rt_defines.h"
#include "solve_from_lu.h"
#include "solve_from_qr.h"
#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <stdio.h>

namespace buildMapping {
class HDMapping;
namespace coder {
class sparse;
}
}  // namespace buildMapping

namespace buildMapping {
namespace coder {
namespace robotics {
namespace core {
namespace internal {
enum class NLPSolverExitFlags : int {
    LocalMinimumFound = 1,
    IterationLimitExceeded,
    TimeLimitExceeded,
    StepSizeBelowMinimum,
    ChangeInErrorBelowMinimum,
    SearchDirectionInvalid,
    HessianNotPositiveSemidefinite,
    TrustRegionRadiusBelowMinimum
};
}
}  // namespace core
}  // namespace robotics
}  // namespace coder

struct struct_T {
    int xstart;
    int xend;
    int depth;
};

struct vision_AlphaBlender_0 {
    int S0_isInitialized;
    int P0_Coordinates[2];
};

struct b_struct_T {
    ::coder::array<double, 2U> edgeNodePairs;
    ::coder::array<double, 2U> edgeMeasurements;
    ::coder::array<double, 2U> edgeInfoMats;
    double tformSize[2];
    double infoMatSize[2];
    double poseDeltaLength;
    ::coder::array<double, 1U> nodeMap;
    ::coder::array<double, 1U> nodeDims;
    ::coder::array<bool, 1U> IsLandmarkNode;
};

struct c_struct_T {
    ::coder::array<int, 1U> a;
    ::coder::array<int, 1U> b;
};

struct cell_wrap_0 {
    ::coder::array<char, 2U> f1;
};

namespace coder {
class ORBPoints {
   public:
    void configure(const ::coder::array<float, 2U> &inputs_Location, const ::coder::array<float, 1U> &inputs_Metric, const ::coder::array<float, 1U> &inputs_Scale, const ::coder::array<float, 1U> &inputs_Orientation);
    ::coder::array<float, 2U> pLocation;
    ::coder::array<float, 1U> pMetric;
    ::coder::array<float, 1U> pScale;
    ::coder::array<float, 1U> pOrientation;

   protected:
    unsigned char pNumLevels;
    float pScaleFactor;
    int pPatchSize;
};

namespace robotics {
namespace core {
namespace internal {
class SystemTimeProvider {
   public:
    coderTimespec StartTime;
};

class TrustRegionIndefiniteDogLegSE2 {
   public:
    void solve(HDMapping *aInstancePtr, const ::coder::array<double, 2U> &seed, BlockMatrix *iobj_0, BlockMatrix **xSol, double *solutionInfo_Iterations, double *solutionInfo_Error,
               double *solutionInfo_ExitFlag, sparse *hess);
    void computeBasicSteps(const ::coder::array<double, 1U> &grad, const sparse *B, ::coder::array<double, 1U> &stepSD, ::coder::array<double, 1U> &stepGN, bool *localMin) const;

   protected:
    void incrementX(const ::coder::array<double, 2U> &x, const ::coder::array<double, 1U> &epsilons, ::coder::array<double, 2U> &xNew) const;

   public:
    b_struct_T ExtraArgs;
    double MaxNumIteration;
    double MaxTime;
    ::coder::array<double, 2U> SeedInternal;
    double MaxTimeInternal;
    double MaxNumIterationInternal;
    double StepTolerance;
    SystemTimeProvider TimeObj;
    double GradientTolerance;
    double FunctionTolerance;
    double InitialTrustRegionRadius;
    double TrustRegionRadiusTolerance;
};
}  // namespace internal
}  // namespace core
}  // namespace robotics

class sparse {
   public:
    void ctranspose(sparse *y) const;
    void fillIn();
    ::coder::array<double, 1U> d;
    ::coder::array<int, 1U> colidx;
    ::coder::array<int, 1U> rowidx;
    int m;
    int n;
};

namespace nav {
namespace algs {
namespace internal {
class BlockInserter2 {
   public:
    void insertGradientBlock(double i, const double blocki_data[], int blocki_size);
    ::coder::array<double, 1U> Gradient;
    ::coder::array<double, 1U> NodeDims;
    ::coder::array<double, 1U> NodeMap;
    ::coder::array<double, 2U> HessianCSC;
    double HessianCSCCount;
};
}  // namespace internal
}  // namespace algs
}  // namespace nav

class anonymous_function {
   public:
    c_struct_T workspace;
};

namespace robotics {
namespace core {
namespace internal {
class b_BlockMatrix {
   public:
    ::coder::array<double, 2U> Matrix;
    double BlockSize[2];
};
}  // namespace internal
}  // namespace core
}  // namespace robotics

class imref2d {
   public:
    double XWorldLimits[2];
    double YWorldLimits[2];
    double ImageSizeAlias[2];
    bool ForcePixelExtentToOne;
};

namespace visioncodegen {
class b_AlphaBlender {
   public:
    void matlabCodegenDestructor();
    ~b_AlphaBlender();
    b_AlphaBlender();
    bool matlabCodegenIsDeleted;
    int isInitialized;
    bool isSetupComplete;
    vision_AlphaBlender_0 cSFunObject;
};
}  // namespace visioncodegen

namespace internal {
class stack {
   public:
    ::coder::bounded_array<struct_T, 120U, 1U> d;
    int n;
};
}  // namespace internal

namespace vision {
namespace internal {
class FeaturePointsImpl {
   public:
    static void selectPoints(const ::coder::array<float, 2U> &points,
                             const ::coder::array<float, 1U> &metric, double numPoints, ::coder::array<bool, 1U> &pointsIdx);
};
}  // namespace internal
}  // namespace vision

namespace nav {
namespace algs {
namespace internal {
class PoseGraphOptimizer {
   public:
    static void parseOptimizePoseGraphInputs(double
                                                 *paramStruct_MaxNumIteration,
                                             double *paramStruct_MaxTime, double *paramStruct_FunctionTolerance, bool *paramStruct_IsVerbose, double *paramStruct_GradientTolerance, double *paramStruct_StepTolerance,
                                             double *paramStruct_InitialTrustRegionRadius, double paramStruct_FirstNodePose[3], double *paramStruct_TrustRegionRadiusTolerance, double *paramStruct_SolverID);
};

class PoseGraphHelpers {
   public:
    static void poseGraphCost(const ::coder::array<double, 2U> &posesMat,
                              const ::coder::array<double, 2U> &args_edgeNodePairs, const ::coder::array<double, 2U> &args_edgeMeasurements, const ::coder::array<double, 2U> &args_edgeInfoMats, const double args_tformSize[2], const double args_infoMatSize[2], double args_poseDeltaLength,
                              const ::coder::array<double, 1U> &args_nodeMap, const ::coder::array<double, 1U> &args_nodeDims, const ::coder::array<bool, 1U> &args_IsLandmarkNode, double *cost, ::coder::array<double, 1U> &gradient, sparse *hessian);
    static void costBetweenTwoNodes(const ::coder::array<double, 2U>
                                        &Toi,
                                    const ::coder::array<double, 2U> &Toj, const ::coder::array<double, 2U> &measurement, const ::coder::array<double, 2U> &Omega,
                                    bool nodejIsLandmark, double *cost, double gradi_data[], int *gradi_size, double gradj_data[], int *gradj_size, double hessii_data[], int hessii_size[2], double hessij_data[], int hessij_size[2], double hessji_data[], int hessji_size[2], double hessjj_data[], int hessjj_size[2]);
};
}  // namespace internal
}  // namespace algs
}  // namespace nav

namespace robotics {
namespace core {
namespace internal {
class SEHelpers {
   public:
    static void veelogmSE3(const double T[16], double vec[6]);
    static void expSE3hat(const double e[6], double T[16]);
};

class Sim3Helpers {
   public:
    static void multiplyLogSim3(const double S1[16], const double S2[16],
                                const double S3[16], double e[7]);
    static void sim3ToSform(const double minVecSim3[7], double S[16]);
};
}  // namespace internal
}  // namespace core
}  // namespace robotics

namespace internal {
class CXSparseAPI {
   public:
    static void iteratedQR(const sparse *A, const ::coder::array<double, 1U> &b, int n, ::coder::array<double, 1U> &out);
};
}  // namespace internal
}  // namespace coder
}  // namespace buildMapping

namespace buildMapping {
omp_nest_lock_t constructWorldMap_nestLockGlobal;
static const signed char iv[9]{1, 0, 0, 0, 1, 0, 0, 0, 1};

static const double dv[9]{0.5, 0.5, 0.5, 320.5, 320.5, 320.5, 640.5, 640.5,
                          640.5};

static const double dv1[9]{0.5, 240.5, 480.5, 0.5, 240.5, 480.5, 0.5, 240.5,
                           480.5};
}  // namespace buildMapping

namespace buildMapping {
static void b_binary_expand_op(::coder::array<double, 2U> &in1, const ::coder::array<double, 2U> &in2, const ::coder::array<double, 3U> &in3);
static double b_binary_expand_op(double in1, double in2, const ::coder::array<double, 1U> &in3, const ::coder::array<double, 1U> &in4, const ::coder::array<double, 1U> &in5);
static void b_binary_expand_op(::coder::array<double, 1U> &in1, const ::coder::array<double, 1U> &in2, double in3);
static void b_binary_expand_op(double in1[3], const ::coder::array<double, 2U> &in2, const ::coder::array<double, 1U> &in3, int in4, int in5);
static void b_binary_expand_op(::coder::array<double, 2U> &in1, const ::coder::array<double, 1U> &in2, int in3, int in4);
static void b_binary_expand_op(coder::nav::algs::internal::BlockInserter2 *in1,
                               int in2, int in4, int in5, const double in6_data[], const int *in6_size);
static void binary_expand_op(::coder::array<bool, 2U> &in1, const ::coder::array<int, 2U> &in2, const ::coder::array<unsigned int, 2U> &in3);
static void blendImage_free(HDMapping *aInstancePtr);
static void blendImage_init(HDMapping *aInstancePtr);
namespace buildMapFunctions {
static void b_fuseOptimizeHDMap(HDMapping *aInstancePtr, const ::coder::array<cell_wrap_0, 1U> &imageFiles, const ::coder::array<double, 2U> &updateNodeVehiclePtPoses, const coder::rigidtform2d initViclePtPose,
                                const bool BW[307200], ::coder::array<unsigned char, 2U> &bigImgSt_bigImg,
                                double bigImgSt_ref_XWorldLimits[2], double bigImgSt_ref_YWorldLimits[2],
                                double bigImgSt_ref_ImageSize[2]);
static void b_helperDetectAndExtractFeatures(HDMapping *aInstancePtr, const unsigned char Irgb[307200], coder::binaryFeatures *features, ::coder::array<float, 1U> &featureMetrics, ::coder::array<float, 2U> &locations);
static void blendImage(HDMapping *aInstancePtr, struct2_T *bigImgSt, const ::coder::array<unsigned char, 2U> &currImg, const coder::imref2d currRef,
                       const ::coder::array<bool, 2U> &maskImg);
static void estiTform(HDMapping *aInstancePtr, const coder::binaryFeatures *preFeatures, const ::coder::array<float, 2U> &prePointsLoc, const coder::binaryFeatures *currFeatures, const ::coder::array<float, 2U> &currPointsLoc, coder::rigidtform2d *tform, ::coder::array<bool, 2U> &inlierIdx, ::coder::array<double, 2U> &validInd1, ::coder::array<double, 2U> &validInd2,
                      bool *isOneSide, int *status);
}  // namespace buildMapFunctions

namespace coder {
static void Outputs(const vision_AlphaBlender_0 *obj, ::coder::array<unsigned char, 2U> &U0, const ::coder::array<unsigned char, 2U> &U1, const ::coder::array<bool, 2U> &U2);
static bool all(const bool x[2]);
static void b_Outputs(::coder::array<unsigned char, 2U> &U0, const ::coder::array<unsigned char, 2U> &U1, const ::coder::array<bool, 2U> &U2, const int U3[2]);
static void b_cosd(double *x);
static double b_mod(double x);
static double b_norm(const ::coder::array<double, 1U> &x);
static void b_padarray(const ::coder::array<unsigned char, 2U> &varargin_1,
                       const double varargin_2[2], ::coder::array<unsigned char, 2U> &b);
static double b_rand(HDMapping *aInstancePtr);
static void b_sind(double *x);
static void b_sparse(const ::coder::array<double, 1U> &varargin_1, const ::coder::array<double, 1U> &varargin_2, const ::coder::array<double, 1U> &varargin_3, sparse *y);
static int cfclose(HDMapping *aInstancePtr, double fid);
static signed char cfopen(HDMapping *aInstancePtr);
static void contrib(double x1, double b_y1, double x2, double y2, signed char quad1, signed char quad2, double scale, signed char *diffQuad, bool *onj);
static void do_vectors(const double a[10], const ::coder::array<double, 2U> &b, double c_data[], int *c_size, int ia_data[], int *ia_size, int *ib_size);
static void estgeotform2d(HDMapping *aInstancePtr, const ::coder::array<double, 2U> &matchedPoints1, const ::coder::array<double, 2U> &matchedPoints2, rigidtform2d *tform, ::coder::array<bool, 2U> &inlierIndex, int *status);
static void fgetl(HDMapping *aInstancePtr, double fileID, ::coder::array<char, 2U> &out);
static FILE *fileManager(HDMapping *aInstancePtr, double varargin_1);
static signed char filedata(HDMapping *aInstancePtr);
namespace images {
namespace geotrans {
namespace internal {
static void constrainToRotationMatrix2D(const double R[4], double Rc[4],
                                        double *r);
}
}  // namespace geotrans

namespace internal {
namespace coder {
static void interp2_local(const float V[307200], const ::coder::array<double, 2U> &Xq, const ::coder::array<double, 2U> &Yq, ::coder::array<float, 2U> &Vq);
namespace optimized {
static void b_remapAndResampleGeneric2d(HDMapping *aInstancePtr,
                                        const unsigned char inputImage[307200], const rigidtform2d tform,
                                        const imref2d outputRef, ::coder::array<unsigned char, 2U> &outputImage);
static void c_remapAndResampleGeneric2d(HDMapping *aInstancePtr,
                                        const bool inputImage[307200], const rigidtform2d tform, const imref2d outputRef, ::coder::array<bool, 2U> &outputImage);
}  // namespace optimized
}  // namespace coder
}  // namespace internal
}  // namespace images

static void inpolygon(const ::coder::array<float, 1U> &x, const ::coder::array<float, 1U> &y, ::coder::array<bool, 1U> &in);
namespace internal {
static double applyToMultipleDims(const ::coder::array<bool, 2U> &x);
static void b_heapsort(::coder::array<int, 1U> &x, int xstart, int xend,
                       const anonymous_function *cmp);
static void b_svd(const double A[9], double U[9], double s[3], double V[9]);
namespace blas {
static double b_xnrm2(const double x[3]);
static void mtimes(const double A_data[], const int A_size[2], const ::coder::array<double, 2U> &B, double C_data[], int C_size[2]);
static void mtimes(const double A_data[], const int A_size[2], const double B_data[], const int B_size[2], double C_data[],
                   int C_size[2]);
static void mtimes(const ::coder::array<double, 2U> &A, const double B[9],
                   ::coder::array<double, 2U> &C);
static void xaxpy(double a, const double x[9], int ix0, double y[3]);
static void xaxpy(int n, double a, int ix0, double y[9], int iy0);
static void xaxpy(double a, const double x[3], double y[9], int iy0);
static double xdotc(int n, const double x[9], int ix0, const double y[9],
                    int iy0);
static double xnrm2(int n, const double x[9], int ix0);
static double xnrm2(const double x[4]);
static void xrot(double x[9], int ix0, int iy0, double c, double s);
static void xrotg(double *a, double *b, double *c, double *s);
static void xswap(double x[9], int ix0, int iy0);
static void xswap(double x[4]);
}  // namespace blas

static void heapify(::coder::array<int, 1U> &x, int idx, int xstart, int xend, const anonymous_function *cmp);
static void insertionsort(::coder::array<int, 1U> &x, int xstart, int xend,
                          const anonymous_function *cmp);
static void introsort(::coder::array<int, 1U> &x, int xend, const anonymous_function *cmp);
static double maximum(const double x[9]);
static float maximum(const ::coder::array<float, 1U> &x);
static void merge(int idx_data[], int x_data[], int np, int nq, int iwork_data[], int xwork_data[]);
static void merge(::coder::array<int, 1U> &idx, ::coder::array<float, 1U> &x, int offset, int np, int nq, ::coder::array<int, 1U> &iwork, ::coder::array<float, 1U> &xwork);
static void merge_block(::coder::array<int, 1U> &idx, ::coder::array<float, 1U> &x, int offset, int n, int preSortLevel, ::coder::array<int, 1U> &iwork, ::coder::array<float, 1U> &xwork);
static float minimum(const ::coder::array<float, 1U> &x);
static void minimum(const ::coder::array<float, 2U> &x, ::coder::array<float, 1U> &ex, ::coder::array<int, 1U> &idx);
static double minimum(const double x[9]);
namespace scalar {
static void b_sqrt(creal_T *x);
}

static void sort(int x_data[], const int *x_size, int idx_data[], int *idx_size);
static void sort(::coder::array<float, 2U> &x, ::coder::array<int, 2U>
                                                   &idx);
static void svd(const double A[4], double U[4], double s[2], double V[4]);
}  // namespace internal

static void inv(const double x[9], double y[9]);
static bool isMemberRows(const double A[2], const ::coder::array<double, 2U>
                                                &S);
static void matchFeatures(HDMapping *aInstancePtr, const binaryFeatures *varargin_1, const binaryFeatures *varargin_2, ::coder::array<unsigned int, 2U> &indexPairs);
static void mean(const ::coder::array<double, 2U> &x, double y[2]);
static void mean(double y[2]);
static poseGraph *optimizePoseGraph(HDMapping *aInstancePtr, poseGraph *b_poseGraph, robotics::core::internal::BlockMatrix *iobj_0, poseGraph *iobj_1);
static void padarray(const ::coder::array<unsigned char, 2U> &varargin_1,
                     const double varargin_2[2], ::coder::array<unsigned char, 2U> &b);
static void poly2mask(bool BW[307200]);
static void tic(HDMapping *aInstancePtr, double *tstart_tv_sec, double *tstart_tv_nsec);
static double toc(HDMapping *aInstancePtr, double tstart_tv_sec, double tstart_tv_nsec);
namespace vision {
namespace internal {
namespace geotrans {
static void computeRigid2d(const ::coder::array<double, 3U> &points,
                           double T[9]);
static void evaluateTform2d(const double tform[9], const ::coder::array<double, 3U> &points, ::coder::array<double, 1U> &dis);
}  // namespace geotrans

namespace matchFeatures {
static void findNearestNeighbors(const ::coder::array<float, 2U>
                                     &scores,
                                 ::coder::array<unsigned int, 2U> &indexPairs, ::coder::array<float, 2U> &topTwoMetrics);
}

namespace ransac {
static void msac(HDMapping *aInstancePtr, const ::coder::array<double, 3U> &allPoints, bool *isFound, double bestModelParams_data[], int bestModelParams_size[2], ::coder::array<bool, 1U> &inliers);
}
}  // namespace internal
}  // namespace vision
}  // namespace coder

static void constructWorldMap_init(HDMapping *aInstancePtr);
static int div_s32(int numerator, int denominator);
static void eml_rand_mt19937ar_stateful_init(HDMapping *aInstancePtr);
static void filedata_init(HDMapping *aInstancePtr);
static void minus(::coder::array<double, 1U> &in1, const ::coder::array<double, 1U> &in2, const ::coder::array<double, 1U> &in3);
static double rt_atan2d_snf(double u0, double u1);
static double rt_hypotd_snf(double u0, double u1);
static double rt_powd_snf(double u0, double u1);
static double rt_remd_snf(double u0, double u1);
}  // namespace buildMapping

namespace buildMapping {
namespace coder {
void poseGraph::addRelativePose(const double varargin_1[3], double varargin_3, double varargin_4) {
    robotics::core::internal::BlockMatrix *obj;
    ::coder::array<double, 2U> a;
    ::coder::array<double, 2U> r;
    ::coder::array<double, 1U> id;
    double Omega[9];
    double RR[9];
    double Trel[9];
    double nodePair[2];
    double T_tmp;
    double edgeId;
    double fromNodeId;
    double toNodeId;
    int i;
    int i1;
    int k;
    bool constraintNeedsInversion;
    bool isLoopClosure;
    bool needNewPoseNode;
    constraintNeedsInversion = false;
    if ((!(varargin_3 == varargin_4)) && ((!(varargin_3 <= NumNodes)) || (!IsLandmarkNode[static_cast<int>(varargin_3) - 1])) &&
        ((!(varargin_4 <= NumNodes)) || (!IsLandmarkNode[static_cast<int>(varargin_4) - 1]))) {
        if ((varargin_3 <= NumNodes) && (varargin_4 <= NumNodes)) {
            bool guard1{false};

            nodePair[0] = varargin_3;
            nodePair[1] = varargin_4;
            findEdgeID(nodePair, id);
            guard1 = false;
            if (id.size(0) == 0) {
                guard1 = true;
            } else {
                nodePair[0] = varargin_3;
                nodePair[1] = varargin_4;
                if (isMemberRows(nodePair, LoopClosureEdgeNodePairs)) {
                    guard1 = true;
                } else {
                    fromNodeId = std::fmin(varargin_3, varargin_4);
                    toNodeId = std::fmax(varargin_3, varargin_4);
                    needNewPoseNode = false;
                    constraintNeedsInversion = (fromNodeId != varargin_3);
                }
            }

            if (guard1) {
                fromNodeId = varargin_3;
                toNodeId = varargin_4;
                needNewPoseNode = false;
            }
        } else {
            edgeId = std::fmin(varargin_3, varargin_4);
            if ((edgeId <= NumNodes) && (std::fmax(varargin_3, varargin_4) -
                                             NumNodes ==
                                         1.0)) {
                fromNodeId = edgeId;
                toNodeId = NumNodes + 1.0;
                needNewPoseNode = true;
                constraintNeedsInversion = (edgeId != varargin_3);
            }
        }
    }

    nodePair[0] = fromNodeId;
    nodePair[1] = toNodeId;
    findEdgeID(nodePair, id);
    isLoopClosure = false;
    if (!needNewPoseNode) {
        if (id.size(0) == 0) {
            isLoopClosure = true;
        } else {
            bool exitg1;
            bool tf;
            tf = false;
            k = 0;
            exitg1 = false;
            while ((!exitg1) && (k <= LoopClosureEdgeIDsInternal.size(1) - 1)) {
                if (id[0] == LoopClosureEdgeIDsInternal[k]) {
                    tf = true;
                    exitg1 = true;
                } else {
                    k++;
                }
            }

            if (tf) {
                isLoopClosure = true;
            }
        }
    }

    edgeId = std::sin(varargin_1[2]);
    T_tmp = std::cos(varargin_1[2]);
    Trel[0] = T_tmp;
    Trel[3] = -edgeId;
    Trel[6] = varargin_1[0];
    Trel[1] = edgeId;
    Trel[4] = T_tmp;
    Trel[7] = varargin_1[1];
    Trel[2] = 0.0;
    Trel[5] = 0.0;
    Trel[8] = 1.0;
    for (i = 0; i < 9; i++) {
        Omega[i] = iv[i];
    }

    k = static_cast<int>(NumEdges + 1.0);
    EdgeNodePairs[k - 1] = fromNodeId;
    EdgeNodePairs[(k + EdgeNodePairs.size(0)) - 1] = toNodeId;
    if (constraintNeedsInversion) {
        double b_RR[9];
        Trel[0] = T_tmp;
        Trel[1] = -edgeId;
        Trel[6] = T_tmp * -varargin_1[0] + edgeId * -varargin_1[1];
        Trel[3] = edgeId;
        Trel[4] = T_tmp;
        Trel[7] = -edgeId * -varargin_1[0] + T_tmp * -varargin_1[1];
        Trel[2] = 0.0;
        Trel[5] = 0.0;
        Trel[8] = 1.0;
        std::memset(&RR[0], 0, 9U * sizeof(double));
        RR[0] = T_tmp;
        RR[1] = edgeId;
        RR[3] = -edgeId;
        RR[4] = T_tmp;
        RR[8] = 1.0;
        for (i = 0; i < 3; i++) {
            double d;
            edgeId = RR[i];
            T_tmp = RR[i + 3];
            i1 = static_cast<int>(RR[i + 6]);
            for (int i2{0}; i2 < 3; i2++) {
                b_RR[i + 3 * i2] = (edgeId * static_cast<double>(iv[3 * i2]) + T_tmp * static_cast<double>(iv[3 * i2 + 1])) +
                                   static_cast<double>(i1 * iv[3 * i2 + 2]);
            }

            edgeId = b_RR[i];
            T_tmp = b_RR[i + 3];
            d = b_RR[i + 6];
            for (i1 = 0; i1 < 3; i1++) {
                Omega[i + 3 * i1] = (edgeId * RR[i1] + T_tmp * RR[i1 + 3]) + d *
                                                                                 RR[i1 + 6];
            }
        }
    }

    EdgeMeasurements->replaceBlock(NumEdges + 1.0, Trel);
    EdgeInfoMatrices->replaceBlock(NumEdges + 1.0, Omega);
    NumEdges++;
    edgeId = NumEdges;
    if (needNewPoseNode) {
        NodeEstimates->extractBlock(fromNodeId, a);
        internal::blas::mtimes(a, Trel, r);
        obj = NodeEstimates;
        edgeId = NumNodes + 1.0;
        edgeId = obj->BlockSize[0] * (edgeId - 1.0) + 1.0;
        if (edgeId > (edgeId + obj->BlockSize[0]) - 1.0) {
            i = 1;
        } else {
            i = static_cast<int>(edgeId);
        }

        k = r.size(0);
        for (i1 = 0; i1 < 3; i1++) {
            for (int i2{0}; i2 < k; i2++) {
                obj->Matrix[((i + i2) + obj->Matrix.size(0) * i1) - 1] = r[i2 +
                                                                           r.size(0) * i1];
            }
        }

        NumNodes++;
        NodeDims[static_cast<int>(NumNodes) - 1] = 3.0;
        i = static_cast<int>(NumNodes - 1.0) - 1;
        NodeMap[static_cast<int>(NumNodes) - 1] = NodeMap[i] + NodeDims[i];
        IsLandmarkNode[static_cast<int>(NumNodes) - 1] = false;
    } else if (isLoopClosure) {
        k = static_cast<int>(NumLoopClosureEdges + 1.0);
        LoopClosureEdgeNodePairs[k - 1] = fromNodeId;
        LoopClosureEdgeNodePairs[(k + LoopClosureEdgeNodePairs.size(0)) - 1] =
            toNodeId;
        LoopClosureEdgeIDsInternal[static_cast<int>(NumLoopClosureEdges + 1.0) -
                                   1] = edgeId;
        NumLoopClosureEdges++;
    }
}

void poseGraph::addRelativePose(double varargin_3, double varargin_4) {
    robotics::core::internal::BlockMatrix *obj;
    ::coder::array<double, 2U> a;
    ::coder::array<double, 2U> r;
    ::coder::array<double, 1U> id;
    double Omega[9];
    double Trel[9];
    double nodePair[2];
    double edgeId;
    double fromNodeId;
    double toNodeId;
    int i;
    int i1;
    int i2;
    int k;
    bool constraintNeedsInversion;
    bool isLoopClosure;
    bool needNewPoseNode;
    constraintNeedsInversion = false;
    if ((!(varargin_3 == varargin_4)) && ((!(varargin_3 <= NumNodes)) || (!IsLandmarkNode[static_cast<int>(varargin_3) - 1])) &&
        ((!(varargin_4 <= NumNodes)) || (!IsLandmarkNode[static_cast<int>(varargin_4) - 1]))) {
        if ((varargin_3 <= NumNodes) && (varargin_4 <= NumNodes)) {
            bool guard1{false};

            nodePair[0] = varargin_3;
            nodePair[1] = varargin_4;
            findEdgeID(nodePair, id);
            guard1 = false;
            if (id.size(0) == 0) {
                guard1 = true;
            } else {
                nodePair[0] = varargin_3;
                nodePair[1] = varargin_4;
                if (isMemberRows(nodePair, LoopClosureEdgeNodePairs)) {
                    guard1 = true;
                } else {
                    fromNodeId = std::fmin(varargin_3, varargin_4);
                    toNodeId = std::fmax(varargin_3, varargin_4);
                    needNewPoseNode = false;
                    constraintNeedsInversion = (fromNodeId != varargin_3);
                }
            }

            if (guard1) {
                fromNodeId = varargin_3;
                toNodeId = varargin_4;
                needNewPoseNode = false;
            }
        } else {
            edgeId = std::fmin(varargin_3, varargin_4);
            if ((edgeId <= NumNodes) && (std::fmax(varargin_3, varargin_4) -
                                             NumNodes ==
                                         1.0)) {
                fromNodeId = edgeId;
                toNodeId = NumNodes + 1.0;
                needNewPoseNode = true;
                constraintNeedsInversion = (edgeId != varargin_3);
            }
        }
    }

    nodePair[0] = fromNodeId;
    nodePair[1] = toNodeId;
    findEdgeID(nodePair, id);
    isLoopClosure = false;
    if (!needNewPoseNode) {
        if (id.size(0) == 0) {
            isLoopClosure = true;
        } else {
            bool exitg1;
            bool tf;
            tf = false;
            k = 0;
            exitg1 = false;
            while ((!exitg1) && (k <= LoopClosureEdgeIDsInternal.size(1) - 1)) {
                if (id[0] == LoopClosureEdgeIDsInternal[k]) {
                    tf = true;
                    exitg1 = true;
                } else {
                    k++;
                }
            }

            if (tf) {
                isLoopClosure = true;
            }
        }
    }

    for (i = 0; i < 9; i++) {
        Trel[i] = iv[i];
        Omega[i] = iv[i];
    }

    k = static_cast<int>(NumEdges + 1.0);
    EdgeNodePairs[k - 1] = fromNodeId;
    EdgeNodePairs[(k + EdgeNodePairs.size(0)) - 1] = toNodeId;
    if (constraintNeedsInversion) {
        signed char b_Trel[9];
        std::memset(&Trel[0], 0, 9U * sizeof(double));
        Trel[0] = 1.0;
        Trel[1] = 0.0;
        Trel[3] = 0.0;
        Trel[4] = 1.0;
        Trel[8] = 1.0;
        for (i = 0; i < 3; i++) {
            i1 = static_cast<int>(Trel[i]);
            i2 = static_cast<int>(Trel[i + 3]);
            k = static_cast<int>(Trel[i + 6]);
            for (int i3{0}; i3 < 3; i3++) {
                b_Trel[i + 3 * i3] = static_cast<signed char>((i1 * iv[3 * i3] + i2 *
                                                                                     iv[3 * i3 + 1]) +
                                                              k * iv[3 * i3 + 2]);
            }

            i1 = b_Trel[i];
            i2 = b_Trel[i + 3];
            k = b_Trel[i + 6];
            for (int i3{0}; i3 < 3; i3++) {
                Omega[i + 3 * i3] = (static_cast<double>(i1) * Trel[i3] +
                                     static_cast<double>(i2) * Trel[i3 + 3]) +
                                    static_cast<double>(k) * Trel[i3 + 6];
            }
        }

        for (i = 0; i < 9; i++) {
            Trel[i] = iv[i];
        }
    }

    EdgeMeasurements->replaceBlock(NumEdges + 1.0, Trel);
    EdgeInfoMatrices->replaceBlock(NumEdges + 1.0, Omega);
    NumEdges++;
    edgeId = NumEdges;
    if (needNewPoseNode) {
        NodeEstimates->extractBlock(fromNodeId, a);
        internal::blas::mtimes(a, Trel, r);
        obj = NodeEstimates;
        edgeId = NumNodes + 1.0;
        edgeId = obj->BlockSize[0] * (edgeId - 1.0) + 1.0;
        if (edgeId > (edgeId + obj->BlockSize[0]) - 1.0) {
            i = 1;
        } else {
            i = static_cast<int>(edgeId);
        }

        k = r.size(0);
        for (i1 = 0; i1 < 3; i1++) {
            for (i2 = 0; i2 < k; i2++) {
                obj->Matrix[((i + i2) + obj->Matrix.size(0) * i1) - 1] = r[i2 +
                                                                           r.size(0) * i1];
            }
        }

        NumNodes++;
        NodeDims[static_cast<int>(NumNodes) - 1] = 3.0;
        i = static_cast<int>(NumNodes - 1.0) - 1;
        NodeMap[static_cast<int>(NumNodes) - 1] = NodeMap[i] + NodeDims[i];
        IsLandmarkNode[static_cast<int>(NumNodes) - 1] = false;
    } else if (isLoopClosure) {
        k = static_cast<int>(NumLoopClosureEdges + 1.0);
        LoopClosureEdgeNodePairs[k - 1] = fromNodeId;
        LoopClosureEdgeNodePairs[(k + LoopClosureEdgeNodePairs.size(0)) - 1] =
            toNodeId;
        LoopClosureEdgeIDsInternal[static_cast<int>(NumLoopClosureEdges + 1.0) -
                                   1] = edgeId;
        NumLoopClosureEdges++;
    }
}

void ORBPoints::configure(const ::coder::array<float, 2U> &inputs_Location,
                          const ::coder::array<float, 1U> &inputs_Metric, const ::coder::array<float, 1U> &inputs_Scale, const ::coder::array<float, 1U> &inputs_Orientation) {
    int ntilerows;
    if (inputs_Metric.size(0) == 1) {
        pMetric.set_size(inputs_Location.size(0));
        ntilerows = inputs_Location.size(0);
        for (int itilerow{0}; itilerow < ntilerows; itilerow++) {
            pMetric[itilerow] = inputs_Metric[0];
        }
    } else {
        pMetric.set_size(inputs_Metric.size(0));
        ntilerows = inputs_Metric.size(0);
        for (int itilerow{0}; itilerow < ntilerows; itilerow++) {
            pMetric[itilerow] = inputs_Metric[itilerow];
        }
    }

    pLocation.set_size(inputs_Location.size(0), 2);
    ntilerows = inputs_Location.size(0) * 2;
    for (int itilerow{0}; itilerow < ntilerows; itilerow++) {
        pLocation[itilerow] = inputs_Location[itilerow];
    }

    if (inputs_Scale.size(0) == 1) {
        pScale.set_size(inputs_Location.size(0));
        ntilerows = inputs_Location.size(0);
        for (int itilerow{0}; itilerow < ntilerows; itilerow++) {
            pScale[itilerow] = inputs_Scale[0];
        }
    } else {
        pScale.set_size(inputs_Scale.size(0));
        ntilerows = inputs_Scale.size(0);
        for (int itilerow{0}; itilerow < ntilerows; itilerow++) {
            pScale[itilerow] = inputs_Scale[itilerow];
        }
    }

    if (inputs_Orientation.size(0) == 1) {
        pOrientation.set_size(inputs_Location.size(0));
        ntilerows = inputs_Location.size(0);
        for (int itilerow{0}; itilerow < ntilerows; itilerow++) {
            pOrientation[itilerow] = inputs_Orientation[0];
        }
    } else {
        pOrientation.set_size(inputs_Orientation.size(0));
        ntilerows = inputs_Orientation.size(0);
        for (int itilerow{0}; itilerow < ntilerows; itilerow++) {
            pOrientation[itilerow] = inputs_Orientation[itilerow];
        }
    }

    pNumLevels = 8U;
    pScaleFactor = 1.2F;
    pPatchSize = 31;
}

void sparse::ctranspose(sparse *y) const {
    ::coder::array<int, 1U> counts;
    int nl;
    int numalloc;
    int outridx;
    int outridx_tmp;
    nl = n;
    y->m = n;
    y->n = m;
    if (colidx[colidx.size(0) - 1] - 1 >= 1) {
        numalloc = colidx[colidx.size(0) - 1] - 2;
    } else {
        numalloc = 0;
    }

    y->d.set_size(numalloc + 1);
    outridx = m + 1;
    y->colidx.set_size(outridx);
    y->colidx[0] = 1;
    y->rowidx.set_size(numalloc + 1);
    for (outridx_tmp = 0; outridx_tmp <= numalloc; outridx_tmp++) {
        y->d[outridx_tmp] = 0.0;
        y->rowidx[outridx_tmp] = 0;
    }

    outridx_tmp = m;
    for (int c{0}; c < outridx_tmp; c++) {
        y->colidx[c + 1] = 1;
    }

    y->fillIn();
    if ((m != 0) && (n != 0)) {
        numalloc = y->colidx.size(0);
        for (outridx_tmp = 0; outridx_tmp < numalloc; outridx_tmp++) {
            y->colidx[outridx_tmp] = 0;
        }

        outridx_tmp = colidx[colidx.size(0) - 1];
        for (numalloc = 0; numalloc <= outridx_tmp - 2; numalloc++) {
            y->colidx[rowidx[numalloc]] = y->colidx[rowidx[numalloc]] + 1;
        }

        y->colidx[0] = 1;
        for (numalloc = 2; numalloc <= outridx; numalloc++) {
            y->colidx[numalloc - 1] = y->colidx[numalloc - 1] + y->colidx[numalloc - 2];
        }

        counts.set_size(m);
        numalloc = m;
        for (outridx = 0; outridx < numalloc; outridx++) {
            counts[outridx] = 0;
        }

        for (int c{0}; c < nl; c++) {
            for (numalloc = colidx[c] - 1; numalloc + 1 < colidx[c + 1]; numalloc++) {
                outridx_tmp = counts[rowidx[numalloc] - 1];
                outridx = (outridx_tmp + y->colidx[rowidx[numalloc] - 1]) - 1;
                y->d[outridx] = d[numalloc];
                y->rowidx[outridx] = c + 1;
                counts[rowidx[numalloc] - 1] = outridx_tmp + 1;
            }
        }
    }
}

void sparse::fillIn() {
    int i;
    int idx;
    idx = 1;
    i = colidx.size(0);
    for (int c{0}; c <= i - 2; c++) {
        int ridx;
        ridx = colidx[c];
        colidx[c] = idx;
        int exitg1;
        int i1;
        do {
            exitg1 = 0;
            i1 = colidx[c + 1];
            if (ridx < i1) {
                double val;
                int currRowIdx;
                val = 0.0;
                currRowIdx = rowidx[ridx - 1];
                while ((ridx < i1) && (rowidx[ridx - 1] == currRowIdx)) {
                    val += d[ridx - 1];
                    ridx++;
                }

                if (val != 0.0) {
                    d[idx - 1] = val;
                    rowidx[idx - 1] = currRowIdx;
                    idx++;
                }
            } else {
                exitg1 = 1;
            }
        } while (exitg1 == 0);
    }

    colidx[colidx.size(0) - 1] = idx;
}

void poseGraph::findEdgeID(const double nodePair[2], ::coder::array<double,
                                                                    1U> &edgeID) const {
    ::coder::array<int, 1U> ii;
    ::coder::array<bool, 1U> tf;
    int iRowA;
    int nRowsA;
    int nx;
    bool exitg1;
    nRowsA = EdgeNodePairs.size(0);
    tf.set_size(EdgeNodePairs.size(0));
    nx = EdgeNodePairs.size(0);
    for (iRowA = 0; iRowA < nx; iRowA++) {
        tf[iRowA] = false;
    }

    for (iRowA = 0; iRowA < nRowsA; iRowA++) {
        bool p;
        tf[iRowA] = false;
        p = true;
        nx = 0;
        exitg1 = false;
        while ((!exitg1) && (nx < 2)) {
            if (EdgeNodePairs[iRowA + EdgeNodePairs.size(0) * nx] != nodePair[nx]) {
                p = false;
                exitg1 = true;
            } else {
                nx++;
            }
        }

        if (p) {
            tf[iRowA] = true;
        }
    }

    nx = tf.size(0);
    nRowsA = 0;
    ii.set_size(tf.size(0));
    iRowA = 0;
    exitg1 = false;
    while ((!exitg1) && (iRowA <= nx - 1)) {
        if (tf[iRowA]) {
            nRowsA++;
            ii[nRowsA - 1] = iRowA + 1;
            if (nRowsA >= nx) {
                exitg1 = true;
            } else {
                iRowA++;
            }
        } else {
            iRowA++;
        }
    }

    if (tf.size(0) == 1) {
        if (nRowsA == 0) {
            ii.set_size(0);
        }
    } else {
        if (nRowsA < 1) {
            nRowsA = 0;
        }

        ii.set_size(nRowsA);
    }

    edgeID.set_size(ii.size(0));
    nx = ii.size(0);
    for (iRowA = 0; iRowA < nx; iRowA++) {
        edgeID[iRowA] = ii[iRowA];
    }
}

poseGraph *poseGraph::init(robotics::core::internal::BlockMatrix *iobj_0) {
    poseGraph *obj;
    robotics::core::internal::BlockMatrix *b_obj;
    double colStart;
    double d;
    double d1;
    double rowStart;
    int b_loop_ub;
    int i;
    int loop_ub;
    int unnamed_idx_0;
    obj = this;
    obj->MaxNumEdges = 10000.0;
    obj->MaxNumNodes = 3000.0;
    obj->NumEdges = 0.0;
    obj->NumNodes = 1.0;
    obj->NumLoopClosureEdges = 0.0;
    rowStart = obj->MaxNumNodes;
    i = static_cast<int>(rowStart * 3.0);
    (&iobj_0[0])[0].Matrix.set_size(i, 3);
    loop_ub = i * 3;
    for (i = 0; i < loop_ub; i++) {
        (&iobj_0[0])[0].Matrix[i] = 0.0;
    }

    (&iobj_0[0])[0].BlockSize[0] = 3.0;
    (&iobj_0[0])[0].BlockSize[1] = 3.0;
    (&iobj_0[0])[0].NumRowBlocks = rowStart;
    (&iobj_0[0])[0].NumColBlocks = 1.0;
    obj->NodeEstimates = &(&iobj_0[0])[0];
    b_obj = obj->NodeEstimates;
    rowStart = b_obj->BlockSize[0] * 0.0 + 1.0;
    colStart = b_obj->BlockSize[1] * 0.0 + 1.0;
    d = (rowStart + b_obj->BlockSize[0]) - 1.0;
    if (rowStart > d) {
        loop_ub = 0;
    } else {
        loop_ub = static_cast<int>(d);
    }

    d1 = (colStart + b_obj->BlockSize[1]) - 1.0;
    if (rowStart > d) {
        unnamed_idx_0 = 0;
    } else {
        unnamed_idx_0 = static_cast<int>(d);
    }

    if (colStart > d1) {
        b_loop_ub = 0;
    } else {
        b_loop_ub = static_cast<int>(d1);
    }

    for (i = 0; i < b_loop_ub; i++) {
        for (int i1{0}; i1 < loop_ub; i1++) {
            b_obj->Matrix[i1 + b_obj->Matrix.size(0) * i] = iv[i1 + unnamed_idx_0 *
                                                                        i];
        }
    }

    loop_ub = static_cast<int>(obj->MaxNumNodes);
    obj->NodeMap.set_size(loop_ub);
    for (i = 0; i < loop_ub; i++) {
        obj->NodeMap[i] = 0.0;
    }

    loop_ub = static_cast<int>(obj->MaxNumNodes);
    obj->NodeDims.set_size(loop_ub);
    for (i = 0; i < loop_ub; i++) {
        obj->NodeDims[i] = 0.0;
    }

    loop_ub = static_cast<int>(obj->MaxNumNodes);
    obj->IsLandmarkNode.set_size(loop_ub);
    for (i = 0; i < loop_ub; i++) {
        obj->IsLandmarkNode[i] = false;
    }

    obj->NodeMap[0] = 1.0;
    obj->NodeDims[0] = 3.0;
    loop_ub = static_cast<int>(obj->MaxNumEdges);
    obj->EdgeNodePairs.set_size(loop_ub, 2);
    loop_ub <<= 1;
    for (i = 0; i < loop_ub; i++) {
        obj->EdgeNodePairs[i] = 0.0;
    }

    loop_ub = static_cast<int>(obj->MaxNumEdges);
    obj->LoopClosureEdgeNodePairs.set_size(loop_ub, 2);
    loop_ub <<= 1;
    for (i = 0; i < loop_ub; i++) {
        obj->LoopClosureEdgeNodePairs[i] = 0.0;
    }

    loop_ub = static_cast<int>(obj->MaxNumEdges);
    obj->LoopClosureEdgeIDsInternal.set_size(1, loop_ub);
    for (i = 0; i < loop_ub; i++) {
        obj->LoopClosureEdgeIDsInternal[i] = 0.0;
    }

    rowStart = obj->MaxNumEdges;
    i = static_cast<int>(rowStart * 3.0);
    (&iobj_0[0])[1].Matrix.set_size(i, 3);
    loop_ub = i * 3;
    for (i = 0; i < loop_ub; i++) {
        (&iobj_0[0])[1].Matrix[i] = 0.0;
    }

    (&iobj_0[0])[1].BlockSize[0] = 3.0;
    (&iobj_0[0])[1].BlockSize[1] = 3.0;
    (&iobj_0[0])[1].NumRowBlocks = rowStart;
    (&iobj_0[0])[1].NumColBlocks = 1.0;
    obj->EdgeMeasurements = &(&iobj_0[0])[1];
    rowStart = obj->MaxNumEdges;
    i = static_cast<int>(rowStart * 3.0);
    (&iobj_0[0])[2].Matrix.set_size(i, 3);
    loop_ub = i * 3;
    for (i = 0; i < loop_ub; i++) {
        (&iobj_0[0])[2].Matrix[i] = 0.0;
    }

    (&iobj_0[0])[2].BlockSize[0] = 3.0;
    (&iobj_0[0])[2].BlockSize[1] = 3.0;
    (&iobj_0[0])[2].NumRowBlocks = rowStart;
    (&iobj_0[0])[2].NumColBlocks = 1.0;
    obj->EdgeInfoMatrices = &(&iobj_0[0])[2];
    return obj;
}

namespace internal {
void CXSparseAPI::iteratedQR(const sparse *A, const ::coder::array<double, 1U> &b, int n, ::coder::array<double, 1U> &out) {
    cs_di *cxA;
    cs_din *N;
    cs_dis *S;
    sparse in;
    ::coder::array<double, 1U> outBuff;
    double tol;
    int loop_ub;
    if (A->m < A->n) {
        A->ctranspose(&in);
        cxA = makeCXSparseMatrix(in.colidx[in.colidx.size(0) - 1] - 1, in.n,
                                 in.m, &(in.colidx.data())[0], &(in.rowidx.data())[0], &(in.d.data())[0]);
    } else {
        cxA = makeCXSparseMatrix(A->colidx[A->colidx.size(0) - 1] - 1, A->n,
                                 A->m, &(((::coder::array<int, 1U> *)&A->colidx)->data())[0], &(((::coder::array<int, 1U> *)&A->rowidx)->data())[0], &(((::coder::array<double, 1U> *)&A->d)->data())[0]);
    }

    S = cs_di_sqr(2, cxA, 1);
    N = cs_di_qr(cxA, S);
    cs_di_spfree(cxA);
    qr_rank_di(N, &tol);
    out.set_size(n);
    if (b.size(0) < n) {
        outBuff.set_size(n);
    } else {
        outBuff.set_size(b.size(0));
    }

    loop_ub = b.size(0);
    for (int i{0}; i < loop_ub; i++) {
        outBuff[i] = b[i];
    }

    solve_from_qr_di(N, S, (double *)&(outBuff.data())[0], b.size(0), n);
    if (n < 1) {
        loop_ub = 0;
    } else {
        loop_ub = n;
    }

    for (int i{0}; i < loop_ub; i++) {
        out[i] = outBuff[i];
    }

    cs_di_sfree(S);
    cs_di_nfree(N);
}
}  // namespace internal

namespace nav {
namespace algs {
namespace internal {
void PoseGraphHelpers::costBetweenTwoNodes(const ::coder::array<double,
                                                                2U> &Toi,
                                           const ::coder::array<double, 2U> &Toj, const ::coder::array<double, 2U> &measurement, const ::coder::array<double, 2U> &Omega, bool nodejIsLandmark, double *cost, double gradi_data[], int *gradi_size, double gradj_data[], int *gradj_size, double hessii_data[], int hessii_size[2], double hessij_data[], int hessij_size[2], double hessji_data[], int hessji_size[2], double hessjj_data[], int hessjj_size[2]) {
    static const signed char b_N[36]{1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
                                     0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
                                     0, 1};

    double Jaci_data[49];
    double Jacj_data[49];
    double Jaci[36];
    double Jacj[36];
    double b_y_tmp_data[21];
    double y_tmp_data[21];
    double b_dv[18];
    double f_R[9];
    double e_data[7];
    double e_R[3];
    int Jaci_size[2];
    int Jacj_size[2];
    int b_y_tmp_size[2];
    int y_tmp_size[2];
    int aoffset;
    int b_i;
    int boffset;
    int coffset;
    int i;
    int j;
    if (Omega.size(0) == 2) {
        double R[9];
        double N[6];
        double cosTheta;
        double sinTheta;
        for (i = 0; i < 3; i++) {
            R[3 * i] = measurement[measurement.size(0) * i];
            R[3 * i + 1] = measurement[measurement.size(0) * i + 1];
            R[3 * i + 2] = measurement[measurement.size(0) * i + 2];
        }

        double b_measurement;
        double d;
        double q;
        double s;
        double sinThetaDy;
        d = rt_atan2d_snf(Toi[1], Toi[0]);
        sinTheta = std::sin(d);
        cosTheta = std::cos(d);
        q = Toj[Toj.size(0) * 2 + 1] - Toi[Toi.size(0) * 2 + 1];
        sinThetaDy = q * sinTheta;
        b_measurement = Toj[Toj.size(0) * 2] - Toi[Toi.size(0) * 2];
        s = b_measurement * cosTheta;
        b_i = 2;
        e_data[0] = (sinThetaDy + s) - R[6];
        q = q * cosTheta - b_measurement * sinTheta;
        e_data[1] = q - R[7];
        N[0] = -cosTheta;
        N[2] = -sinTheta;
        N[4] = q;
        N[1] = sinTheta;
        N[3] = -cosTheta;
        N[5] = -sinThetaDy - s;
        Jaci_size[0] = 2;
        Jaci_size[1] = 3;
        for (i = 0; i < 6; i++) {
            Jaci_data[i] = N[i];
        }

        Jacj_size[0] = 2;
        Jacj_size[1] = 2;
        Jacj_data[0] = cosTheta;
        Jacj_data[1] = -sinTheta;
        Jacj_data[2] = sinTheta;
        Jacj_data[3] = cosTheta;
    } else if (Omega.size(0) == 3) {
        if (nodejIsLandmark) {
            double Sio[16];
            double Tio[16];
            double deltatform[16];
            double R[9];
            double b_R[9];
            double t[3];
            double d;
            double d1;
            double d2;
            for (i = 0; i < 4; i++) {
                coffset = i << 2;
                Sio[coffset] = Toi[Toi.size(0) * i];
                deltatform[coffset] = measurement[measurement.size(0) * i];
                Sio[coffset + 1] = Toi[Toi.size(0) * i + 1];
                deltatform[coffset + 1] = measurement[measurement.size(0) * i + 1];
                Sio[coffset + 2] = Toi[Toi.size(0) * i + 2];
                deltatform[coffset + 2] = measurement[measurement.size(0) * i + 2];
                Sio[coffset + 3] = Toi[Toi.size(0) * i + 3];
                deltatform[coffset + 3] = measurement[measurement.size(0) * i + 3];
            }

            for (i = 0; i < 3; i++) {
                R[3 * i] = Sio[i];
                R[3 * i + 1] = Sio[i + 4];
                R[3 * i + 2] = Sio[i + 8];
            }

            for (i = 0; i < 9; i++) {
                b_R[i] = -R[i];
            }

            d = Sio[12];
            d1 = Sio[13];
            d2 = Sio[14];
            for (i = 0; i < 3; i++) {
                coffset = i << 2;
                Tio[coffset] = R[3 * i];
                Tio[coffset + 1] = R[3 * i + 1];
                Tio[coffset + 2] = R[3 * i + 2];
                Tio[i + 12] = (b_R[i] * d + b_R[i + 3] * d1) + b_R[i + 6] * d2;
            }

            Tio[3] = 0.0;
            Tio[7] = 0.0;
            Tio[11] = 0.0;
            Tio[15] = 1.0;
            for (i = 0; i < 3; i++) {
                f_R[3 * i] = -deltatform[i];
                f_R[3 * i + 1] = -deltatform[i + 4];
                f_R[3 * i + 2] = -deltatform[i + 8];
                t[i] = ((Tio[i] * Toj[Toj.size(0) * 3] + Tio[i + 4] *
                                                             Toj[Toj.size(0) * 3 + 1]) +
                        Tio[i + 8] * Toj[Toj.size(0) * 3 + 2]) +
                       Tio[i + 12];
            }

            b_i = 3;
            d = deltatform[12];
            d1 = deltatform[13];
            d2 = deltatform[14];
            for (i = 0; i < 3; i++) {
                b_dv[3 * i] = iv[3 * i];
                j = 3 * i + 1;
                b_dv[j] = iv[j];
                j = 3 * i + 2;
                b_dv[j] = iv[j];
                e_data[i] = t[i] + ((f_R[i] * d + f_R[i + 3] * d1) + f_R[i + 6] * d2);
            }

            b_dv[9] = -0.0;
            b_dv[12] = t[2];
            b_dv[15] = -t[1];
            b_dv[10] = -t[2];
            b_dv[13] = -0.0;
            b_dv[16] = t[0];
            b_dv[11] = t[1];
            b_dv[14] = -t[0];
            b_dv[17] = -0.0;
            Jaci_size[0] = 3;
            Jaci_size[1] = 6;
            std::copy(&b_dv[0], &b_dv[18], &Jaci_data[0]);
            Jacj_size[0] = 3;
            Jacj_size[1] = 3;
            for (i = 0; i < 3; i++) {
                coffset = i << 2;
                Jacj_data[3 * i] = Tio[coffset];
                Jacj_data[3 * i + 1] = Tio[coffset + 1];
                Jacj_data[3 * i + 2] = Tio[coffset + 2];
            }
        } else {
            double R[9];
            double b_R[9];
            double c_R[9];
            double dRoidtheta[4];
            double d_R[4];
            double y[4];
            double y_tmp[4];
            double b_y_tmp[2];
            double e_tmp[2];
            double b_measurement;
            double cosTheta;
            double d;
            double d1;
            double d2;
            double d3;
            double d4;
            double q;
            double sinTheta;
            for (i = 0; i < 3; i++) {
                R[3 * i] = Toi[Toi.size(0) * i];
                b_R[3 * i] = Toj[Toj.size(0) * i];
                c_R[3 * i] = measurement[measurement.size(0) * i];
                boffset = 3 * i + 1;
                R[boffset] = Toi[Toi.size(0) * i + 1];
                b_R[boffset] = Toj[Toj.size(0) * i + 1];
                c_R[boffset] = measurement[measurement.size(0) * i + 1];
                boffset = 3 * i + 2;
                R[boffset] = Toi[Toi.size(0) * i + 2];
                b_R[boffset] = Toj[Toj.size(0) * i + 2];
                c_R[boffset] = measurement[measurement.size(0) * i + 2];
            }

            for (i = 0; i < 2; i++) {
                d = c_R[3 * i];
                d1 = c_R[3 * i + 1];
                d2 = R[0] * d + R[3] * d1;
                d = R[1] * d + R[4] * d1;
                y[i] = d2 * b_R[0] + d * b_R[1];
                y[i + 2] = d2 * b_R[3] + d * b_R[4];
                e_tmp[i] = b_R[i + 6] - R[i + 6];
            }

            b_measurement = rt_atan2d_snf(y[1], y[0]);
            d_R[0] = R[0];
            d_R[1] = R[3];
            d_R[2] = R[1];
            d_R[3] = R[4];
            if (std::isnan(b_measurement + 3.1415926535897931) || std::isinf(b_measurement + 3.1415926535897931)) {
                cosTheta = rtNaN;
            } else if (b_measurement + 3.1415926535897931 == 0.0) {
                cosTheta = 0.0;
            } else {
                bool rEQ0;
                cosTheta = std::fmod(b_measurement + 3.1415926535897931,
                                     6.2831853071795862);
                rEQ0 = (cosTheta == 0.0);
                if (!rEQ0) {
                    q = std::abs((b_measurement + 3.1415926535897931) /
                                 6.2831853071795862);
                    rEQ0 = !(std::abs(q - std::floor(q + 0.5)) >
                             2.2204460492503131E-16 * q);
                }

                if (rEQ0) {
                    cosTheta = 0.0;
                } else if (b_measurement + 3.1415926535897931 < 0.0) {
                    cosTheta += 6.2831853071795862;
                }
            }

            dRoidtheta[0] = R[3];
            dRoidtheta[2] = -R[0];
            dRoidtheta[1] = R[0];
            dRoidtheta[3] = R[3];
            d = c_R[0];
            d1 = c_R[1];
            d2 = c_R[3];
            d3 = c_R[4];
            for (j = 0; j < 2; j++) {
                coffset = j << 1;
                q = dRoidtheta[j + 2];
                d4 = dRoidtheta[j];
                y[coffset] = d * d4 + d1 * q;
                y[coffset + 1] = d2 * d4 + d3 * q;
            }

            d = e_tmp[0];
            d1 = e_tmp[1];
            for (j = 0; j < 2; j++) {
                coffset = j << 1;
                q = R[j % 2];
                b_measurement = R[(j + 2) % 2 + 3];
                dRoidtheta[coffset] = c_R[0] * q + c_R[1] * b_measurement;
                dRoidtheta[coffset + 1] = c_R[3] * q + c_R[4] * b_measurement;
                b_y_tmp[j] = (d_R[j] * d + d_R[j + 2] * d1) - c_R[j + 6];
            }

            b_i = 3;
            e_data[0] = c_R[0] * b_y_tmp[0] + b_y_tmp[1] * c_R[1];
            e_data[1] = b_y_tmp[0] * c_R[3] + b_y_tmp[1] * c_R[4];
            e_data[2] = cosTheta - 3.1415926535897931;
            y_tmp[0] = -c_R[0];
            y_tmp[1] = -c_R[3];
            y_tmp[2] = -c_R[1];
            y_tmp[3] = -c_R[4];
            d = R[0];
            d1 = R[3];
            d2 = R[1];
            d3 = R[4];
            d4 = e_tmp[0];
            sinTheta = e_tmp[1];
            for (i = 0; i < 2; i++) {
                q = y_tmp[i + 2];
                b_measurement = y_tmp[i];
                d_R[i] = b_measurement * d + q * d1;
                d_R[i + 2] = b_measurement * d2 + q * d3;
                b_y_tmp[i] = y[i] * d4 + y[i + 2] * sinTheta;
            }

            f_R[0] = d_R[0];
            f_R[1] = d_R[1];
            f_R[6] = b_y_tmp[0];
            f_R[3] = d_R[2];
            f_R[4] = d_R[3];
            f_R[7] = b_y_tmp[1];
            f_R[2] = 0.0;
            f_R[5] = 0.0;
            f_R[8] = -1.0;
            Jaci_size[0] = 3;
            Jaci_size[1] = 3;
            std::copy(&f_R[0], &f_R[9], &Jaci_data[0]);
            f_R[0] = dRoidtheta[0];
            f_R[1] = dRoidtheta[1];
            f_R[6] = 0.0;
            f_R[3] = dRoidtheta[2];
            f_R[4] = dRoidtheta[3];
            f_R[7] = 0.0;
            f_R[2] = 0.0;
            f_R[5] = 0.0;
            f_R[8] = 1.0;
            Jacj_size[0] = 3;
            Jacj_size[1] = 3;
            std::copy(&f_R[0], &f_R[9], &Jacj_data[0]);
        }
    } else if (Omega.size(0) == 6) {
        double Sio[16];
        double Sji[16];
        double Tio[16];
        double Tji[16];
        double Tjo[16];
        double Tjop[16];
        double deltatform[16];
        double R[9];
        double b_R[9];
        double b_dv1[6];
        double d;
        double d1;
        double d2;
        double d3;
        for (i = 0; i < 4; i++) {
            coffset = i << 2;
            Sio[coffset] = Toi[Toi.size(0) * i];
            Sji[coffset] = Toj[Toj.size(0) * i];
            deltatform[coffset] = measurement[measurement.size(0) * i];
            Sio[coffset + 1] = Toi[Toi.size(0) * i + 1];
            Sji[coffset + 1] = Toj[Toj.size(0) * i + 1];
            deltatform[coffset + 1] = measurement[measurement.size(0) * i +
                                                  1];
            Sio[coffset + 2] = Toi[Toi.size(0) * i + 2];
            Sji[coffset + 2] = Toj[Toj.size(0) * i + 2];
            deltatform[coffset + 2] = measurement[measurement.size(0) * i +
                                                  2];
            Sio[coffset + 3] = Toi[Toi.size(0) * i + 3];
            Sji[coffset + 3] = Toj[Toj.size(0) * i + 3];
            deltatform[coffset + 3] = measurement[measurement.size(0) * i +
                                                  3];
        }

        for (i = 0; i < 3; i++) {
            R[3 * i] = Sio[i];
            R[3 * i + 1] = Sio[i + 4];
            R[3 * i + 2] = Sio[i + 8];
        }

        for (i = 0; i < 9; i++) {
            b_R[i] = -R[i];
        }

        d = Sio[12];
        d1 = Sio[13];
        d2 = Sio[14];
        for (i = 0; i < 3; i++) {
            coffset = i << 2;
            Tio[coffset] = R[3 * i];
            aoffset = 3 * i + 1;
            Tio[coffset + 1] = R[aoffset];
            boffset = 3 * i + 2;
            Tio[coffset + 2] = R[boffset];
            Tio[i + 12] = (b_R[i] * d + b_R[i + 3] * d1) + b_R[i + 6] * d2;
            R[3 * i] = deltatform[i];
            R[aoffset] = deltatform[i + 4];
            R[boffset] = deltatform[i + 8];
        }

        Tio[3] = 0.0;
        Tio[7] = 0.0;
        Tio[11] = 0.0;
        Tio[15] = 1.0;
        for (i = 0; i < 9; i++) {
            b_R[i] = -R[i];
        }

        d = deltatform[12];
        d1 = deltatform[13];
        d2 = deltatform[14];
        for (i = 0; i < 3; i++) {
            coffset = i << 2;
            Tji[coffset] = R[3 * i];
            Tji[coffset + 1] = R[3 * i + 1];
            Tji[coffset + 2] = R[3 * i + 2];
            Tji[i + 12] = (b_R[i] * d + b_R[i + 3] * d1) + b_R[i + 6] * d2;
        }

        Tji[3] = 0.0;
        Tji[7] = 0.0;
        Tji[11] = 0.0;
        Tji[15] = 1.0;
        for (i = 0; i < 4; i++) {
            d = Tji[i];
            d1 = Tji[i + 4];
            d2 = Tji[i + 8];
            d3 = Tji[i + 12];
            for (j = 0; j < 4; j++) {
                coffset = j << 2;
                deltatform[i + coffset] = ((d * Tio[coffset] + d1 *
                                                                   Tio[coffset + 1]) +
                                           d2 * Tio[coffset + 2]) +
                                          d3 *
                                              Tio[coffset + 3];
            }
        }

        for (i = 0; i < 3; i++) {
            R[3 * i] = Sji[i];
            R[3 * i + 1] = Sji[i + 4];
            R[3 * i + 2] = Sji[i + 8];
        }

        for (i = 0; i < 9; i++) {
            b_R[i] = -R[i];
        }

        d = Sji[12];
        d1 = Sji[13];
        d2 = Sji[14];
        for (i = 0; i < 3; i++) {
            coffset = i << 2;
            Tjo[coffset] = R[3 * i];
            Tjo[coffset + 1] = R[3 * i + 1];
            Tjo[coffset + 2] = R[3 * i + 2];
            Tjo[i + 12] = (b_R[i] * d + b_R[i + 3] * d1) + b_R[i + 6] * d2;
        }

        Tjo[3] = 0.0;
        Tjo[7] = 0.0;
        Tjo[11] = 0.0;
        Tjo[15] = 1.0;
        for (b_i = 0; b_i < 6; b_i++) {
            double Tm[16];
            double g_R[16];
            double c_R[9];
            double N[6];
            for (i = 0; i < 6; i++) {
                N[i] = static_cast<double>(b_N[i + 6 * b_i]) * 1.0E-5;
            }

            robotics::core::internal::SEHelpers::expSE3hat(N, Sio);
            for (i = 0; i < 6; i++) {
                N[i] = -static_cast<double>(b_N[i + 6 * b_i]) * 1.0E-5;
            }

            robotics::core::internal::SEHelpers::expSE3hat(N, Tm);
            for (i = 0; i < 4; i++) {
                d = Sio[i];
                d1 = Sio[i + 4];
                d2 = Sio[i + 8];
                d3 = Sio[i + 12];
                for (j = 0; j < 4; j++) {
                    coffset = j << 2;
                    Tjop[i + coffset] = ((d * Tio[coffset] + d1 * Tio[coffset +
                                                                      1]) +
                                         d2 * Tio[coffset + 2]) +
                                        d3 *
                                            Tio[coffset + 3];
                }
            }

            for (i = 0; i < 4; i++) {
                d = Tji[i];
                d1 = Tji[i + 4];
                d2 = Tji[i + 8];
                d3 = Tji[i + 12];
                for (j = 0; j < 4; j++) {
                    coffset = j << 2;
                    g_R[i + coffset] = ((d * Tjop[coffset] + d1 * Tjop[coffset +
                                                                       1]) +
                                        d2 * Tjop[coffset + 2]) +
                                       d3 *
                                           Tjop[coffset + 3];
                }
            }

            for (i = 0; i < 4; i++) {
                d = g_R[i];
                d1 = g_R[i + 4];
                d2 = g_R[i + 8];
                d3 = g_R[i + 12];
                for (j = 0; j < 4; j++) {
                    coffset = j << 2;
                    Tjop[i + coffset] = ((d * Sji[coffset] + d1 * Sji[coffset +
                                                                      1]) +
                                         d2 * Sji[coffset + 2]) +
                                        d3 *
                                            Sji[coffset + 3];
                }
            }

            robotics::core::internal::SEHelpers::veelogmSE3(Tjop, b_dv1);
            for (i = 0; i < 4; i++) {
                d = Tm[i];
                d1 = Tm[i + 4];
                d2 = Tm[i + 8];
                d3 = Tm[i + 12];
                for (j = 0; j < 4; j++) {
                    coffset = j << 2;
                    Tjop[i + coffset] = ((d * Tio[coffset] + d1 * Tio[coffset +
                                                                      1]) +
                                         d2 * Tio[coffset + 2]) +
                                        d3 *
                                            Tio[coffset + 3];
                }
            }

            for (i = 0; i < 4; i++) {
                d = Tji[i];
                d1 = Tji[i + 4];
                d2 = Tji[i + 8];
                d3 = Tji[i + 12];
                for (j = 0; j < 4; j++) {
                    coffset = j << 2;
                    g_R[i + coffset] = ((d * Tjop[coffset] + d1 * Tjop[coffset +
                                                                       1]) +
                                        d2 * Tjop[coffset + 2]) +
                                       d3 *
                                           Tjop[coffset + 3];
                }
            }

            for (i = 0; i < 4; i++) {
                d = g_R[i];
                d1 = g_R[i + 4];
                d2 = g_R[i + 8];
                d3 = g_R[i + 12];
                for (j = 0; j < 4; j++) {
                    coffset = j << 2;
                    Tjop[i + coffset] = ((d * Sji[coffset] + d1 * Sji[coffset +
                                                                      1]) +
                                         d2 * Sji[coffset + 2]) +
                                        d3 *
                                            Sji[coffset + 3];
                }
            }

            robotics::core::internal::SEHelpers::veelogmSE3(Tjop, N);
            for (i = 0; i < 6; i++) {
                Jaci[i + 6 * b_i] = (b_dv1[i] - N[i]) / 2.0E-5;
            }

            for (i = 0; i < 4; i++) {
                d = Sio[i];
                d1 = Sio[i + 4];
                d2 = Sio[i + 8];
                d3 = Sio[i + 12];
                for (j = 0; j < 4; j++) {
                    coffset = j << 2;
                    Tjop[i + coffset] = ((d * Tjo[coffset] + d1 * Tjo[coffset +
                                                                      1]) +
                                         d2 * Tjo[coffset + 2]) +
                                        d3 *
                                            Tjo[coffset + 3];
                }

                d = Tm[i];
                d1 = Tm[i + 4];
                d2 = Tm[i + 8];
                d3 = Tm[i + 12];
                for (j = 0; j < 4; j++) {
                    coffset = j << 2;
                    Sio[i + coffset] = ((d * Tjo[coffset] + d1 * Tjo[coffset + 1]) + d2 * Tjo[coffset + 2]) + d3 *
                                                                                                                  Tjo[coffset + 3];
                }
            }

            for (i = 0; i < 3; i++) {
                R[3 * i] = Tjop[i];
                b_R[3 * i] = Sio[i];
                boffset = 3 * i + 1;
                R[boffset] = Tjop[i + 4];
                b_R[boffset] = Sio[i + 4];
                boffset = 3 * i + 2;
                R[boffset] = Tjop[i + 8];
                b_R[boffset] = Sio[i + 8];
            }

            for (i = 0; i < 9; i++) {
                c_R[i] = -R[i];
            }

            d = Tjop[12];
            d1 = Tjop[13];
            d2 = Tjop[14];
            for (i = 0; i < 3; i++) {
                boffset = i << 2;
                g_R[boffset] = R[3 * i];
                g_R[boffset + 1] = R[3 * i + 1];
                g_R[boffset + 2] = R[3 * i + 2];
                g_R[i + 12] = (c_R[i] * d + c_R[i + 3] * d1) + c_R[i + 6] * d2;
            }

            g_R[3] = 0.0;
            g_R[7] = 0.0;
            g_R[11] = 0.0;
            g_R[15] = 1.0;
            for (i = 0; i < 4; i++) {
                d = deltatform[i];
                d1 = deltatform[i + 4];
                d2 = deltatform[i + 8];
                d3 = deltatform[i + 12];
                for (j = 0; j < 4; j++) {
                    coffset = j << 2;
                    Tjop[i + coffset] = ((d * g_R[coffset] + d1 * g_R[coffset +
                                                                      1]) +
                                         d2 * g_R[coffset + 2]) +
                                        d3 *
                                            g_R[coffset + 3];
                }
            }

            robotics::core::internal::SEHelpers::veelogmSE3(Tjop, b_dv1);
            for (i = 0; i < 9; i++) {
                R[i] = -b_R[i];
            }

            d = Sio[12];
            d1 = Sio[13];
            d2 = Sio[14];
            for (i = 0; i < 3; i++) {
                boffset = i << 2;
                g_R[boffset] = b_R[3 * i];
                g_R[boffset + 1] = b_R[3 * i + 1];
                g_R[boffset + 2] = b_R[3 * i + 2];
                g_R[i + 12] = (R[i] * d + R[i + 3] * d1) + R[i + 6] * d2;
            }

            g_R[3] = 0.0;
            g_R[7] = 0.0;
            g_R[11] = 0.0;
            g_R[15] = 1.0;
            for (i = 0; i < 4; i++) {
                d = deltatform[i];
                d1 = deltatform[i + 4];
                d2 = deltatform[i + 8];
                d3 = deltatform[i + 12];
                for (j = 0; j < 4; j++) {
                    coffset = j << 2;
                    Tjop[i + coffset] = ((d * g_R[coffset] + d1 * g_R[coffset +
                                                                      1]) +
                                         d2 * g_R[coffset + 2]) +
                                        d3 *
                                            g_R[coffset + 3];
                }
            }

            robotics::core::internal::SEHelpers::veelogmSE3(Tjop, N);
            for (i = 0; i < 6; i++) {
                Jacj[i + 6 * b_i] = (b_dv1[i] - N[i]) / 2.0E-5;
            }
        }

        for (i = 0; i < 4; i++) {
            d = deltatform[i];
            d1 = deltatform[i + 4];
            d2 = deltatform[i + 8];
            d3 = deltatform[i + 12];
            for (j = 0; j < 4; j++) {
                coffset = j << 2;
                Tjop[i + coffset] = ((d * Sji[coffset] + d1 * Sji[coffset + 1]) + d2 * Sji[coffset + 2]) + d3 *
                                                                                                               Sji[coffset + 3];
            }
        }

        robotics::core::internal::SEHelpers::veelogmSE3(Tjop, b_dv1);
        b_i = 6;
        for (i = 0; i < 6; i++) {
            e_data[i] = b_dv1[i];
        }

        Jaci_size[0] = 6;
        Jaci_size[1] = 6;
        Jacj_size[0] = 6;
        Jacj_size[1] = 6;
        std::copy(&Jaci[0], &Jaci[36], &Jaci_data[0]);
        std::copy(&Jacj[0], &Jacj[36], &Jacj_data[0]);
    } else {
        double Sio[16];
        double Sji[16];
        double Tji[16];
        double Tjop[16];
        double g_R[16];
        double R[9];
        double b_R[9];
        double deltavec[7];
        double b_measurement;
        double cosTheta;
        double d;
        double d1;
        double d2;
        double q;
        double sinThetaDy;
        for (i = 0; i < 4; i++) {
            coffset = i << 2;
            Tjop[coffset] = Toi[Toi.size(0) * i];
            Tji[coffset] = Toj[Toj.size(0) * i];
            Sio[coffset] = measurement[measurement.size(0) * i];
            Tjop[coffset + 1] = Toi[Toi.size(0) * i + 1];
            Tji[coffset + 1] = Toj[Toj.size(0) * i + 1];
            Sio[coffset + 1] = measurement[measurement.size(0) * i + 1];
            Tjop[coffset + 2] = Toi[Toi.size(0) * i + 2];
            Tji[coffset + 2] = Toj[Toj.size(0) * i + 2];
            Sio[coffset + 2] = measurement[measurement.size(0) * i + 2];
            Tjop[coffset + 3] = Toi[Toi.size(0) * i + 3];
            Tji[coffset + 3] = Toj[Toj.size(0) * i + 3];
            Sio[coffset + 3] = measurement[measurement.size(0) * i + 3];
        }

        for (i = 0; i < 3; i++) {
            R[3 * i] = Tjop[i];
            b_R[3 * i] = Sio[i];
            boffset = 3 * i + 1;
            R[boffset] = Tjop[i + 4];
            b_R[boffset] = Sio[i + 4];
            boffset = 3 * i + 2;
            R[boffset] = Tjop[i + 8];
            b_R[boffset] = Sio[i + 8];
        }

        b_measurement = measurement[measurement.size(0) * 3 + 3];
        d = Sio[12];
        d1 = Sio[13];
        d2 = Sio[14];
        for (i = 0; i < 3; i++) {
            coffset = i << 2;
            Sji[coffset] = b_R[3 * i];
            Sji[coffset + 1] = b_R[3 * i + 1];
            Sji[coffset + 2] = b_R[3 * i + 2];
            Sji[i + 12] = -((b_R[i] * d + b_R[i + 3] * d1) + b_R[i + 6] * d2) / b_measurement;
        }

        Sji[3] = 0.0;
        Sji[7] = 0.0;
        Sji[11] = 0.0;
        Sji[15] = 1.0 / measurement[measurement.size(0) * 3 + 3];
        for (i = 0; i < 3; i++) {
            b_R[3 * i] = Tjop[i];
            b_R[3 * i + 1] = Tjop[i + 4];
            b_R[3 * i + 2] = Tjop[i + 8];
        }

        q = Toi[Toi.size(0) * 3 + 3];
        d = Tjop[12];
        d1 = Tjop[13];
        d2 = Tjop[14];
        for (i = 0; i < 3; i++) {
            coffset = i << 2;
            Sio[coffset] = b_R[3 * i];
            Sio[coffset + 1] = b_R[3 * i + 1];
            Sio[coffset + 2] = b_R[3 * i + 2];
            Sio[i + 12] = -((b_R[i] * d + b_R[i + 3] * d1) + b_R[i + 6] * d2) / q;
        }

        Sio[3] = 0.0;
        Sio[7] = 0.0;
        Sio[11] = 0.0;
        Sio[15] = 1.0 / Toi[Toi.size(0) * 3 + 3];
        q = Toi[Toi.size(0) * 3 + 3];
        b_measurement = Toj[Toj.size(0) * 3 + 3];
        cosTheta = Toi[Toi.size(0) * 3 + 3];
        sinThetaDy = Toj[Toj.size(0) * 3 + 3];
        d = Toi[Toi.size(0) * 3 + 3];
        d1 = Toj[Toj.size(0) * 3 + 3];
        for (int k{0}; k < 7; k++) {
            double Tjo[16];
            double Tm[16];
            double deltatform[16];
            double c_R[9];
            double t[3];
            double d3;
            double d4;
            double s;
            double sinTheta;
            for (b_i = 0; b_i < 7; b_i++) {
                deltavec[b_i] = 0.0;
            }

            deltavec[k] = 1.0E-9;
            robotics::core::internal::Sim3Helpers::sim3ToSform(deltavec, Tjo);
            for (i = 0; i < 3; i++) {
                b_R[3 * i] = Tjo[i];
                b_R[3 * i + 1] = Tjo[i + 4];
                b_R[3 * i + 2] = Tjo[i + 8];
            }

            d2 = Tjo[12];
            d3 = Tjo[13];
            d4 = Tjo[14];
            sinTheta = Tjo[15];
            for (i = 0; i < 3; i++) {
                coffset = i << 2;
                deltatform[coffset] = b_R[3 * i];
                deltatform[coffset + 1] = b_R[3 * i + 1];
                deltatform[coffset + 2] = b_R[3 * i + 2];
                deltatform[i + 12] = -((b_R[i] * d2 + b_R[i + 3] * d3) + b_R[i + 6] * d4) / sinTheta;
            }

            deltatform[3] = 0.0;
            deltatform[7] = 0.0;
            deltatform[11] = 0.0;
            deltatform[15] = 1.0 / Tjo[15];
            for (i = 0; i < 3; i++) {
                d2 = 0.0;
                for (j = 0; j < 3; j++) {
                    coffset = j << 2;
                    f_R[i + 3 * j] = (Tjop[i] * deltatform[coffset] + Tjop[i + 4] * deltatform[coffset + 1]) + Tjop[i + 8] *
                                                                                                                   deltatform[coffset + 2];
                    d2 += q * Tjop[i + coffset] * deltatform[j + 12];
                }

                e_R[i] = d2 + Tjop[i + 12];
            }

            for (i = 0; i < 3; i++) {
                coffset = i << 2;
                Tjo[coffset] = f_R[3 * i];
                Tjo[coffset + 1] = f_R[3 * i + 1];
                Tjo[coffset + 2] = f_R[3 * i + 2];
                Tjo[i + 12] = e_R[i];
            }

            Tjo[3] = 0.0;
            Tjo[7] = 0.0;
            Tjo[11] = 0.0;
            Tjo[15] = d * deltatform[15];
            s = d1 * deltatform[15];
            deltavec[k] = -1.0E-9;
            robotics::core::internal::Sim3Helpers::sim3ToSform(deltavec, Tm);
            for (i = 0; i < 3; i++) {
                d2 = 0.0;
                for (j = 0; j < 3; j++) {
                    boffset = j << 2;
                    coffset = i + boffset;
                    aoffset = j + 3 * i;
                    b_R[aoffset] = Tjo[coffset];
                    c_R[i + 3 * j] = (Tji[i] * deltatform[boffset] + Tji[i + 4] *
                                                                         deltatform[boffset + 1]) +
                                     Tji[i + 8] *
                                         deltatform[boffset + 2];
                    d2 += b_measurement * Tji[coffset] * deltatform[j + 12];
                    f_R[aoffset] = Tm[coffset];
                }

                t[i] = d2 + Tji[i + 12];
            }

            d2 = Tm[12];
            d3 = Tm[13];
            d4 = Tm[14];
            sinTheta = Tm[15];
            for (i = 0; i < 3; i++) {
                coffset = i << 2;
                deltatform[coffset] = f_R[3 * i];
                deltatform[coffset + 1] = f_R[3 * i + 1];
                deltatform[coffset + 2] = f_R[3 * i + 2];
                deltatform[i + 12] = -((f_R[i] * d2 + f_R[i + 3] * d3) + f_R[i + 6] * d4) / sinTheta;
            }

            deltatform[3] = 0.0;
            deltatform[7] = 0.0;
            deltatform[11] = 0.0;
            deltatform[15] = 1.0 / Tm[15];
            for (i = 0; i < 3; i++) {
                d2 = 0.0;
                for (j = 0; j < 3; j++) {
                    coffset = j << 2;
                    f_R[i + 3 * j] = (Tjop[i] * deltatform[coffset] + Tjop[i + 4] * deltatform[coffset + 1]) + Tjop[i + 8] *
                                                                                                                   deltatform[coffset + 2];
                    d2 += cosTheta * Tjop[i + coffset] * deltatform[j + 12];
                }

                e_R[i] = d2 + Tjop[i + 12];
            }

            for (i = 0; i < 3; i++) {
                coffset = i << 2;
                Tm[coffset] = f_R[3 * i];
                Tm[coffset + 1] = f_R[3 * i + 1];
                Tm[coffset + 2] = f_R[3 * i + 2];
                Tm[i + 12] = e_R[i];
            }

            Tm[3] = 0.0;
            Tm[7] = 0.0;
            Tm[11] = 0.0;
            Tm[15] = d * deltatform[15];
            d2 = Tjo[12];
            d3 = Tjo[13];
            d4 = Tjo[14];
            sinTheta = Tjo[15];
            for (i = 0; i < 3; i++) {
                f_R[3 * i] = Tm[i];
                boffset = i << 2;
                g_R[boffset] = b_R[3 * i];
                coffset = 3 * i + 1;
                f_R[coffset] = Tm[i + 4];
                g_R[boffset + 1] = b_R[coffset];
                coffset = 3 * i + 2;
                f_R[coffset] = Tm[i + 8];
                g_R[boffset + 2] = b_R[coffset];
                g_R[i + 12] = -((b_R[i] * d2 + b_R[i + 3] * d3) + b_R[i + 6] *
                                                                      d4) /
                              sinTheta;
            }

            g_R[3] = 0.0;
            g_R[7] = 0.0;
            g_R[11] = 0.0;
            g_R[15] = 1.0 / Tjo[15];
            robotics::core::internal::Sim3Helpers::multiplyLogSim3(Sji, g_R,
                                                                   Tji, deltavec);
            d2 = Tm[12];
            d3 = Tm[13];
            d4 = Tm[14];
            sinTheta = Tm[15];
            for (i = 0; i < 3; i++) {
                boffset = i << 2;
                g_R[boffset] = f_R[3 * i];
                g_R[boffset + 1] = f_R[3 * i + 1];
                g_R[boffset + 2] = f_R[3 * i + 2];
                g_R[i + 12] = -((f_R[i] * d2 + f_R[i + 3] * d3) + f_R[i + 6] *
                                                                      d4) /
                              sinTheta;
            }

            g_R[3] = 0.0;
            g_R[7] = 0.0;
            g_R[11] = 0.0;
            g_R[15] = 1.0 / Tm[15];
            robotics::core::internal::Sim3Helpers::multiplyLogSim3(Sji, g_R,
                                                                   Tji, e_data);
            for (i = 0; i < 7; i++) {
                Jaci_data[i + 7 * k] = (deltavec[i] - e_data[i]) *
                                       4.9999999999999994E+8;
            }

            for (i = 0; i < 3; i++) {
                boffset = i << 2;
                g_R[boffset] = c_R[3 * i];
                g_R[boffset + 1] = c_R[3 * i + 1];
                g_R[boffset + 2] = c_R[3 * i + 2];
                g_R[i + 12] = t[i];
            }

            g_R[3] = 0.0;
            g_R[7] = 0.0;
            g_R[11] = 0.0;
            g_R[15] = s;
            robotics::core::internal::Sim3Helpers::multiplyLogSim3(Sji, Sio,
                                                                   g_R, deltavec);
            for (i = 0; i < 3; i++) {
                d2 = 0.0;
                for (j = 0; j < 3; j++) {
                    coffset = j << 2;
                    f_R[i + 3 * j] = (Tji[i] * deltatform[coffset] + Tji[i + 4] *
                                                                         deltatform[coffset + 1]) +
                                     Tji[i + 8] *
                                         deltatform[coffset + 2];
                    d2 += sinThetaDy * Tji[i + coffset] * deltatform[j + 12];
                }

                e_R[i] = d2 + Tji[i + 12];
            }

            for (i = 0; i < 3; i++) {
                boffset = i << 2;
                g_R[boffset] = f_R[3 * i];
                g_R[boffset + 1] = f_R[3 * i + 1];
                g_R[boffset + 2] = f_R[3 * i + 2];
                g_R[i + 12] = e_R[i];
            }

            g_R[3] = 0.0;
            g_R[7] = 0.0;
            g_R[11] = 0.0;
            g_R[15] = d1 * deltatform[15];
            robotics::core::internal::Sim3Helpers::multiplyLogSim3(Sji, Sio,
                                                                   g_R, e_data);
            for (i = 0; i < 7; i++) {
                Jacj_data[i + 7 * k] = (deltavec[i] - e_data[i]) *
                                       4.9999999999999994E+8;
            }
        }

        q = Toi[Toi.size(0) * 3 + 3];
        d = Tjop[12];
        d1 = Tjop[13];
        d2 = Tjop[14];
        for (i = 0; i < 3; i++) {
            boffset = i << 2;
            g_R[boffset] = R[3 * i];
            g_R[boffset + 1] = R[3 * i + 1];
            g_R[boffset + 2] = R[3 * i + 2];
            g_R[i + 12] = -((R[i] * d + R[i + 3] * d1) + R[i + 6] * d2) / q;
        }

        g_R[3] = 0.0;
        g_R[7] = 0.0;
        g_R[11] = 0.0;
        g_R[15] = 1.0 / Toi[Toi.size(0) * 3 + 3];
        robotics::core::internal::Sim3Helpers::multiplyLogSim3(Sji, g_R,
                                                               Tji, deltavec);
        b_i = 7;
        for (i = 0; i < 7; i++) {
            e_data[i] = deltavec[i];
        }

        Jaci_size[0] = 7;
        Jaci_size[1] = 7;
        Jacj_size[0] = 7;
        Jacj_size[1] = 7;
    }

    coffset = Omega.size(1);
    aoffset = Omega.size(1);
    for (j = 0; j < coffset; j++) {
        boffset = j * Omega.size(0);
        e_R[j] = 0.0;
        for (int k{0}; k < b_i; k++) {
            e_R[j] += e_data[k] * Omega[boffset + k];
        }
    }

    *cost = 0.0;
    for (i = 0; i < aoffset; i++) {
        *cost += e_R[i] * e_data[i];
    }

    ::buildMapping::coder::internal::blas::mtimes(Jaci_data, Jaci_size,
                                                  Omega, y_tmp_data, y_tmp_size);
    coffset = y_tmp_size[0] - 1;
    i = y_tmp_size[1];
    *gradi_size = y_tmp_size[0];
    std::memset(&gradi_data[0], 0, static_cast<unsigned int>(coffset + 1) * sizeof(double));
    for (int k{0}; k < i; k++) {
        aoffset = k * y_tmp_size[0];
        for (b_i = 0; b_i <= coffset; b_i++) {
            gradi_data[b_i] += y_tmp_data[aoffset + b_i] * e_data[k];
        }
    }

    ::buildMapping::coder::internal::blas::mtimes(Jacj_data, Jacj_size,
                                                  Omega, b_y_tmp_data, b_y_tmp_size);
    coffset = b_y_tmp_size[0] - 1;
    i = b_y_tmp_size[1];
    *gradj_size = b_y_tmp_size[0];
    std::memset(&gradj_data[0], 0, static_cast<unsigned int>(coffset + 1) * sizeof(double));
    for (int k{0}; k < i; k++) {
        aoffset = k * b_y_tmp_size[0];
        for (b_i = 0; b_i <= coffset; b_i++) {
            gradj_data[b_i] += b_y_tmp_data[aoffset + b_i] * e_data[k];
        }
    }

    ::buildMapping::coder::internal::blas::mtimes(y_tmp_data, y_tmp_size,
                                                  Jaci_data, Jaci_size, hessii_data, hessii_size);
    ::buildMapping::coder::internal::blas::mtimes(y_tmp_data, y_tmp_size,
                                                  Jacj_data, Jacj_size, hessij_data, hessij_size);
    ::buildMapping::coder::internal::blas::mtimes(b_y_tmp_data,
                                                  b_y_tmp_size, Jaci_data, Jaci_size, hessji_data, hessji_size);
    ::buildMapping::coder::internal::blas::mtimes(b_y_tmp_data,
                                                  b_y_tmp_size, Jacj_data, Jacj_size, hessjj_data, hessjj_size);
}

void BlockInserter2::insertGradientBlock(double i, const double blocki_data[], int blocki_size) {
    ::coder::array<double, 1U> obj;
    double d;
    double rowStart;
    int b_i;
    int i1;
    int i2;
    int loop_ub;
    rowStart = NodeMap[static_cast<int>(i) - 1];
    d = (rowStart + NodeDims[static_cast<int>(i) - 1]) - 1.0;
    if (rowStart > d) {
        b_i = 0;
        i1 = 0;
        i2 = 0;
    } else {
        b_i = static_cast<int>(rowStart) - 1;
        i1 = static_cast<int>(d);
        i2 = static_cast<int>(rowStart) - 1;
    }

    loop_ub = i1 - b_i;
    if (loop_ub == blocki_size) {
        obj.set_size(loop_ub);
        for (i1 = 0; i1 < loop_ub; i1++) {
            obj[i1] = Gradient[b_i + i1] + blocki_data[i1];
        }

        loop_ub = obj.size(0);
        for (b_i = 0; b_i < loop_ub; b_i++) {
            Gradient[i2 + b_i] = obj[b_i];
        }
    } else {
        b_binary_expand_op(this, i2, b_i, i1 - 1, blocki_data,
                           &blocki_size);
    }
}

void PoseGraphOptimizer::parseOptimizePoseGraphInputs(double
                                                          *paramStruct_MaxNumIteration,
                                                      double *paramStruct_MaxTime, double *paramStruct_FunctionTolerance, bool *paramStruct_IsVerbose, double *paramStruct_GradientTolerance, double *paramStruct_StepTolerance,
                                                      double *paramStruct_InitialTrustRegionRadius, double paramStruct_FirstNodePose[3], double *paramStruct_TrustRegionRadiusTolerance, double *paramStruct_SolverID) {
    static const char cv[128]{'\x00', '\x01', '\x02', '\x03', '\x04',
                              '\x05', '\x06', '\x07', '\x08', '\x09', '\x0a', '\x0b', '\x0c',
                              '\x0d', '\x0e', '\x0f', '\x10', '\x11', '\x12', '\x13', '\x14',
                              '\x15', '\x16', '\x17', '\x18', '\x19', '\x1a', '\x1b', '\x1c',
                              '\x1d', '\x1e', '\x1f', ' ', '!', '\"', '#', '$', '%', '&', '\'',
                              '(', ')', '*', '+', ',', '-', '.', '/', '0', '1', '2', '3', '4',
                              '5', '6', '7', '8', '9', ':', ';', '<', '=', '>', '?', '@', 'a',
                              'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n',
                              'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z', '[',
                              '\\', ']', '^', '_', '`', 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h',
                              'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u',
                              'v', 'w', 'x', 'y', 'z', '{', '|', '}', '~', '\x7f'};

    static const char cv1[3]{'o', 'f', 'f'};

    static const char cv2[2]{'o', 'n'};

    ::coder::array<char, 2U> str;
    int exitg1;
    int kstr;
    char out_data[3];
    bool b_bool;
    *paramStruct_MaxNumIteration = 300.0;
    *paramStruct_MaxTime = 500.0;
    *paramStruct_FunctionTolerance = 1.0E-8;
    str.set_size(1, 3);
    str[0] = 'o';
    str[1] = 'f';
    str[2] = 'f';
    b_bool = false;
    kstr = 0;
    do {
        exitg1 = 0;
        if (kstr <= 2) {
            if (cv[static_cast<int>(str[kstr])] != cv[static_cast<int>(cv1[kstr])]) {
                exitg1 = 1;
            } else {
                kstr++;
            }
        } else {
            b_bool = true;
            exitg1 = 1;
        }
    } while (exitg1 == 0);

    if (b_bool) {
        char partial_match_data[3];
        partial_match_data[0] = 'o';
        partial_match_data[1] = 'f';
        partial_match_data[2] = 'f';
        kstr = 3;
        for (int i{0}; i < 3; i++) {
            out_data[i] = partial_match_data[i];
        }
    } else {
        kstr = 2;
        out_data[0] = ' ';
        out_data[1] = ' ';
    }

    *paramStruct_IsVerbose = false;
    if (kstr == 2) {
        kstr = 0;
        do {
            exitg1 = 0;
            if (kstr < 2) {
                if (out_data[kstr] != cv2[kstr]) {
                    exitg1 = 1;
                } else {
                    kstr++;
                }
            } else {
                *paramStruct_IsVerbose = true;
                exitg1 = 1;
            }
        } while (exitg1 == 0);
    }

    *paramStruct_GradientTolerance = 5.0E-9;
    *paramStruct_StepTolerance = 1.0E-12;
    *paramStruct_InitialTrustRegionRadius = 100.0;
    paramStruct_FirstNodePose[0] = 0.0;
    paramStruct_FirstNodePose[1] = 0.0;
    paramStruct_FirstNodePose[2] = 0.0;
    *paramStruct_TrustRegionRadiusTolerance = 1.0E-10;
    *paramStruct_SolverID = -1.0;
}

void PoseGraphHelpers::poseGraphCost(const ::coder::array<double, 2U>
                                         &posesMat,
                                     const ::coder::array<double, 2U> &args_edgeNodePairs,
                                     const ::coder::array<double, 2U> &args_edgeMeasurements, const ::coder::array<double, 2U> &args_edgeInfoMats, const double args_tformSize[2], const double args_infoMatSize[2], double args_poseDeltaLength, const ::coder::array<double, 1U> &args_nodeMap,
                                     const ::coder::array<double, 1U> &args_nodeDims, const ::coder::array<bool, 1U> &args_IsLandmarkNode, double *cost, ::coder::array<double, 1U> &gradient, sparse *hessian) {
    BlockInserter2 bi;
    robotics::core::internal::BlockMatrix edgeInfoMats;
    robotics::core::internal::BlockMatrix edgeMeasurements;
    robotics::core::internal::BlockMatrix poses;
    ::coder::array<double, 2U> OmegaIn;
    ::coder::array<double, 2U> Tij;
    ::coder::array<double, 2U> Toi;
    ::coder::array<double, 2U> Toj;
    ::coder::array<double, 2U> varargin_1;
    ::coder::array<double, 2U> y;
    ::coder::array<double, 1U> b_bi;
    ::coder::array<double, 1U> c_bi;
    ::coder::array<double, 1U> d_bi;
    ::coder::array<int, 2U> m1;
    ::coder::array<int, 2U> m2;
    ::coder::array<int, 2U> m3;
    ::coder::array<int, 2U> m4;
    ::coder::array<int, 2U> n1;
    ::coder::array<int, 2U> n2;
    ::coder::array<int, 2U> n3;
    ::coder::array<int, 2U> v1;
    ::coder::array<int, 2U> vk;
    ::coder::array<signed char, 2U> b_I;
    double H_data[196];
    double hessii_data[49];
    double hessij_data[49];
    double hessji_data[49];
    double hessjj_data[49];
    double gradi_data[7];
    double gradj_data[7];
    double maxNodeDim;
    double numEntries1;
    int H_size_tmp;
    int i;
    int i1;
    int idx;
    int k;
    int last;
    int loop_ub;
    edgeMeasurements.Matrix.set_size(args_edgeMeasurements.size(0), 3);
    loop_ub = args_edgeMeasurements.size(0) * 3;
    for (i = 0; i < loop_ub; i++) {
        edgeMeasurements.Matrix[i] = args_edgeMeasurements[i];
    }

    edgeMeasurements.BlockSize[0] = args_tformSize[0];
    edgeMeasurements.BlockSize[1] = args_tformSize[1];
    edgeMeasurements.NumRowBlocks = static_cast<double>(args_edgeMeasurements.size(0)) / args_tformSize[0];
    edgeMeasurements.NumColBlocks = 3.0 / args_tformSize[1];
    edgeInfoMats.Matrix.set_size(args_edgeInfoMats.size(0), 3);
    loop_ub = args_edgeInfoMats.size(0) * 3;
    for (i = 0; i < loop_ub; i++) {
        edgeInfoMats.Matrix[i] = args_edgeInfoMats[i];
    }

    edgeInfoMats.BlockSize[0] = args_infoMatSize[0];
    edgeInfoMats.BlockSize[1] = args_infoMatSize[1];
    edgeInfoMats.NumRowBlocks = static_cast<double>(args_edgeInfoMats.size(0)) / args_infoMatSize[0];
    edgeInfoMats.NumColBlocks = 3.0 / args_infoMatSize[1];
    *cost = 0.0;
    poses.Matrix.set_size(posesMat.size(0), 3);
    loop_ub = posesMat.size(0) * 3;
    for (i = 0; i < loop_ub; i++) {
        poses.Matrix[i] = posesMat[i];
    }

    poses.BlockSize[0] = args_tformSize[0];
    poses.BlockSize[1] = args_tformSize[1];
    poses.NumRowBlocks = static_cast<double>(posesMat.size(0)) /
                         args_tformSize[0];
    poses.NumColBlocks = 3.0 / args_tformSize[1];
    idx = static_cast<int>(poses.NumRowBlocks * args_poseDeltaLength);
    bi.Gradient.set_size(idx);
    for (i = 0; i < idx; i++) {
        bi.Gradient[i] = 0.0;
    }

    bi.NodeDims.set_size(args_nodeDims.size(0));
    loop_ub = args_nodeDims.size(0);
    for (i = 0; i < loop_ub; i++) {
        bi.NodeDims[i] = args_nodeDims[i];
    }

    bi.NodeMap.set_size(args_nodeMap.size(0));
    loop_ub = args_nodeMap.size(0);
    for (i = 0; i < loop_ub; i++) {
        bi.NodeMap[i] = args_nodeMap[i];
    }

    last = args_nodeDims.size(0);
    if (args_nodeDims.size(0) <= 2) {
        if (args_nodeDims.size(0) == 1) {
            maxNodeDim = args_nodeDims[0];
        } else if ((args_nodeDims[0] < args_nodeDims[args_nodeDims.size(0) - 1]) || (std::isnan(args_nodeDims[0]) && (!std::isnan(args_nodeDims[args_nodeDims.size(0) - 1])))) {
            maxNodeDim = args_nodeDims[args_nodeDims.size(0) - 1];
        } else {
            maxNodeDim = args_nodeDims[0];
        }
    } else {
        if (!std::isnan(args_nodeDims[0])) {
            idx = 1;
        } else {
            bool exitg1;
            idx = 0;
            k = 2;
            exitg1 = false;
            while ((!exitg1) && (k <= last)) {
                if (!std::isnan(args_nodeDims[k - 1])) {
                    idx = k;
                    exitg1 = true;
                } else {
                    k++;
                }
            }
        }

        if (idx == 0) {
            maxNodeDim = args_nodeDims[0];
        } else {
            maxNodeDim = args_nodeDims[idx - 1];
            i = idx + 1;
            for (k = i; k <= last; k++) {
                numEntries1 = args_nodeDims[k - 1];
                if (maxNodeDim < numEntries1) {
                    maxNodeDim = numEntries1;
                }
            }
        }
    }

    bi.HessianCSC.set_size(static_cast<int>((4.0 * static_cast<double>(args_edgeNodePairs.size(0)) + 1.0) * maxNodeDim * maxNodeDim), 3);
    loop_ub = static_cast<int>((4.0 * static_cast<double>(args_edgeNodePairs.size(0)) + 1.0) * maxNodeDim * maxNodeDim) * 3;
    for (i = 0; i < loop_ub; i++) {
        bi.HessianCSC[i] = 0.0;
    }

    bi.HessianCSCCount = 1.0;
    i = args_edgeNodePairs.size(0);
    for (k = 0; k < i; k++) {
        double b_i;
        double j;
        double numEntries2;
        double numEntries2_tmp;
        int hessii_size[2];
        int hessij_size[2];
        int hessji_size[2];
        int hessjj_size[2];
        int H_size;
        signed char i3;
        signed char sizes_idx_1;
        unsigned char varargin_2_size_idx_0;
        bool empty_non_axis_sizes;
        b_i = args_edgeNodePairs[k];
        j = args_edgeNodePairs[k + args_edgeNodePairs.size(0)];
        edgeMeasurements.extractBlock(static_cast<double>(k) + 1.0, Tij);
        edgeInfoMats.extractBlock(static_cast<double>(k) + 1.0, OmegaIn);
        if (args_IsLandmarkNode[static_cast<int>(j) - 1]) {
            if (args_nodeDims[static_cast<int>(j) - 1] < 1.0) {
                loop_ub = 0;
                idx = 0;
            } else {
                loop_ub = static_cast<int>(args_nodeDims[static_cast<int>(j) -
                                                         1]);
                idx = static_cast<int>(args_nodeDims[static_cast<int>(j) - 1]);
            }

            for (i1 = 0; i1 < idx; i1++) {
                for (int i2{0}; i2 < loop_ub; i2++) {
                    OmegaIn[i2 + loop_ub * i1] = OmegaIn[i2 + OmegaIn.size(0) *
                                                                  i1];
                }
            }

            OmegaIn.set_size(loop_ub, idx);
        }

        poses.extractBlock(b_i, Toi);
        poses.extractBlock(j, Toj);
        PoseGraphHelpers::costBetweenTwoNodes(Toi, Toj, Tij, OmegaIn,
                                              args_IsLandmarkNode[static_cast<int>(j) - 1], &maxNodeDim,
                                              gradi_data, &idx, gradj_data, &last, hessii_data, hessii_size,
                                              hessij_data, hessij_size, hessji_data, hessji_size, hessjj_data,
                                              hessjj_size);
        *cost += maxNodeDim;
        bi.insertGradientBlock(b_i, gradi_data, idx);
        bi.insertGradientBlock(j, gradj_data, last);
        maxNodeDim = bi.NodeDims[static_cast<int>(b_i) - 1];
        numEntries1 = maxNodeDim * maxNodeDim;
        if (std::isnan(numEntries1)) {
            y.set_size(1, 1);
            y[0] = rtNaN;
        } else if (numEntries1 < 1.0) {
            y.set_size(1, 0);
        } else {
            y.set_size(1, static_cast<int>(numEntries1 - 1.0) + 1);
            loop_ub = static_cast<int>(numEntries1 - 1.0);
            for (i1 = 0; i1 <= loop_ub; i1++) {
                y[i1] = static_cast<double>(i1) + 1.0;
            }
        }

        v1.set_size(1, y.size(1));
        loop_ub = y.size(1);
        for (i1 = 0; i1 < loop_ub; i1++) {
            v1[i1] = static_cast<int>(y[i1]) - 1;
        }

        vk.set_size(1, v1.size(1));
        loop_ub = v1.size(1);
        for (i1 = 0; i1 < loop_ub; i1++) {
            vk[i1] = div_s32(v1[i1], static_cast<int>(maxNodeDim));
        }

        v1.set_size(1, v1.size(1));
        loop_ub = v1.size(1) - 1;
        for (i1 = 0; i1 <= loop_ub; i1++) {
            v1[i1] = v1[i1] - vk[i1] * static_cast<int>(maxNodeDim);
        }

        m1.set_size(1, v1.size(1));
        loop_ub = v1.size(1);
        for (i1 = 0; i1 < loop_ub; i1++) {
            m1[i1] = v1[i1] + 1;
        }

        n1.set_size(1, vk.size(1));
        loop_ub = vk.size(1);
        for (i1 = 0; i1 < loop_ub; i1++) {
            n1[i1] = vk[i1] + 1;
        }

        numEntries2_tmp = bi.NodeDims[static_cast<int>(j) - 1];
        numEntries2 = maxNodeDim * numEntries2_tmp;
        if (std::isnan(numEntries2)) {
            y.set_size(1, 1);
            y[0] = rtNaN;
        } else if (numEntries2 < 1.0) {
            y.set_size(1, 0);
        } else {
            y.set_size(1, static_cast<int>(numEntries2 - 1.0) + 1);
            loop_ub = static_cast<int>(numEntries2 - 1.0);
            for (i1 = 0; i1 <= loop_ub; i1++) {
                y[i1] = static_cast<double>(i1) + 1.0;
            }
        }

        v1.set_size(1, y.size(1));
        loop_ub = y.size(1);
        for (i1 = 0; i1 < loop_ub; i1++) {
            v1[i1] = static_cast<int>(y[i1]) - 1;
        }

        vk.set_size(1, v1.size(1));
        loop_ub = v1.size(1);
        for (i1 = 0; i1 < loop_ub; i1++) {
            vk[i1] = div_s32(v1[i1], static_cast<int>(maxNodeDim));
        }

        v1.set_size(1, v1.size(1));
        loop_ub = v1.size(1) - 1;
        for (i1 = 0; i1 <= loop_ub; i1++) {
            v1[i1] = v1[i1] - vk[i1] * static_cast<int>(maxNodeDim);
        }

        m2.set_size(1, v1.size(1));
        loop_ub = v1.size(1);
        for (i1 = 0; i1 < loop_ub; i1++) {
            m2[i1] = v1[i1] + 1;
        }

        n2.set_size(1, vk.size(1));
        loop_ub = vk.size(1);
        for (i1 = 0; i1 < loop_ub; i1++) {
            n2[i1] = vk[i1] + 1;
        }

        if (std::isnan(numEntries2)) {
            y.set_size(1, 1);
            y[0] = rtNaN;
        } else if (numEntries2 < 1.0) {
            y.set_size(1, 0);
        } else {
            y.set_size(1, static_cast<int>(numEntries2 - 1.0) + 1);
            loop_ub = static_cast<int>(numEntries2 - 1.0);
            for (i1 = 0; i1 <= loop_ub; i1++) {
                y[i1] = static_cast<double>(i1) + 1.0;
            }
        }

        v1.set_size(1, y.size(1));
        loop_ub = y.size(1);
        for (i1 = 0; i1 < loop_ub; i1++) {
            v1[i1] = static_cast<int>(y[i1]) - 1;
        }

        vk.set_size(1, v1.size(1));
        loop_ub = v1.size(1);
        for (i1 = 0; i1 < loop_ub; i1++) {
            vk[i1] = div_s32(v1[i1], static_cast<int>(numEntries2_tmp));
        }

        v1.set_size(1, v1.size(1));
        loop_ub = v1.size(1) - 1;
        for (i1 = 0; i1 <= loop_ub; i1++) {
            v1[i1] = v1[i1] - vk[i1] * static_cast<int>(numEntries2_tmp);
        }

        m3.set_size(1, v1.size(1));
        loop_ub = v1.size(1);
        for (i1 = 0; i1 < loop_ub; i1++) {
            m3[i1] = v1[i1] + 1;
        }

        n3.set_size(1, vk.size(1));
        loop_ub = vk.size(1);
        for (i1 = 0; i1 < loop_ub; i1++) {
            n3[i1] = vk[i1] + 1;
        }

        maxNodeDim = numEntries2_tmp * numEntries2_tmp;
        if (std::isnan(maxNodeDim)) {
            y.set_size(1, 1);
            y[0] = rtNaN;
        } else if (maxNodeDim < 1.0) {
            y.set_size(1, 0);
        } else {
            y.set_size(1, static_cast<int>(maxNodeDim - 1.0) + 1);
            loop_ub = static_cast<int>(maxNodeDim - 1.0);
            for (i1 = 0; i1 <= loop_ub; i1++) {
                y[i1] = static_cast<double>(i1) + 1.0;
            }
        }

        v1.set_size(1, y.size(1));
        loop_ub = y.size(1);
        for (i1 = 0; i1 < loop_ub; i1++) {
            v1[i1] = static_cast<int>(y[i1]) - 1;
        }

        vk.set_size(1, v1.size(1));
        loop_ub = v1.size(1);
        for (i1 = 0; i1 < loop_ub; i1++) {
            vk[i1] = div_s32(v1[i1], static_cast<int>(numEntries2_tmp));
        }

        v1.set_size(1, v1.size(1));
        loop_ub = v1.size(1) - 1;
        for (i1 = 0; i1 <= loop_ub; i1++) {
            v1[i1] = v1[i1] - vk[i1] * static_cast<int>(numEntries2_tmp);
        }

        m4.set_size(1, v1.size(1));
        loop_ub = v1.size(1);
        for (i1 = 0; i1 < loop_ub; i1++) {
            m4[i1] = v1[i1] + 1;
        }

        y.set_size(1, vk.size(1));
        loop_ub = vk.size(1);
        for (i1 = 0; i1 < loop_ub; i1++) {
            y[i1] = vk[i1] + 1;
        }

        idx = hessii_size[0] * hessii_size[1];
        last = hessij_size[0] * hessij_size[1];
        H_size_tmp = hessji_size[0] * hessji_size[1];
        loop_ub = hessjj_size[0] * hessjj_size[1];
        H_size = ((idx + last) + H_size_tmp) + loop_ub;
        if (idx - 1 >= 0) {
            std::copy(&hessii_data[0], &hessii_data[idx], &H_data[0]);
        }

        for (i1 = 0; i1 < last; i1++) {
            H_data[i1 + hessii_size[0] * hessii_size[1]] = hessij_data[i1];
        }

        for (i1 = 0; i1 < H_size_tmp; i1++) {
            H_data[(i1 + hessii_size[0] * hessii_size[1]) + last] =
                hessji_data[i1];
        }

        for (i1 = 0; i1 < loop_ub; i1++) {
            H_data[((i1 + hessii_size[0] * hessii_size[1]) + last) +
                   H_size_tmp] = hessjj_data[i1];
        }

        numEntries1 = bi.HessianCSCCount + ((numEntries1 + 2.0 *
                                                               numEntries2) +
                                            maxNodeDim);
        if (bi.HessianCSCCount > numEntries1 - 1.0) {
            i1 = 0;
        } else {
            i1 = static_cast<int>(bi.HessianCSCCount) - 1;
        }

        H_size_tmp = m1.size(1);
        idx = m2.size(1);
        last = m3.size(1);
        varargin_1.set_size(((m1.size(1) + m2.size(1)) + m3.size(1)) +
                                m4.size(1),
                            2);
        loop_ub = m1.size(1);
        for (int i2{0}; i2 < loop_ub; i2++) {
            varargin_1[i2] = (bi.NodeMap[static_cast<int>(b_i) - 1] - 1.0) +
                             static_cast<double>(m1[i2]);
        }

        loop_ub = n1.size(1);
        for (int i2{0}; i2 < loop_ub; i2++) {
            varargin_1[i2 + varargin_1.size(0)] = (bi.NodeMap[static_cast<
                                                                  int>(b_i) -
                                                              1] -
                                                   1.0) +
                                                  static_cast<double>(n1[i2]);
        }

        loop_ub = m2.size(1);
        for (int i2{0}; i2 < loop_ub; i2++) {
            varargin_1[i2 + H_size_tmp] = (bi.NodeMap[static_cast<int>(b_i) - 1] - 1.0) + static_cast<double>(m2[i2]);
        }

        loop_ub = n2.size(1);
        for (int i2{0}; i2 < loop_ub; i2++) {
            varargin_1[(i2 + H_size_tmp) + varargin_1.size(0)] =
                (bi.NodeMap[static_cast<int>(j) - 1] - 1.0) + static_cast<
                                                                  double>(n2[i2]);
        }

        loop_ub = m3.size(1);
        for (int i2{0}; i2 < loop_ub; i2++) {
            varargin_1[(i2 + H_size_tmp) + idx] = (bi.NodeMap[static_cast<
                                                                  int>(j) -
                                                              1] -
                                                   1.0) +
                                                  static_cast<double>(m3[i2]);
        }

        loop_ub = n3.size(1);
        for (int i2{0}; i2 < loop_ub; i2++) {
            varargin_1[((i2 + H_size_tmp) + idx) + varargin_1.size(0)] =
                (bi.NodeMap[static_cast<int>(b_i) - 1] - 1.0) + static_cast<
                                                                    double>(n3[i2]);
        }

        loop_ub = m4.size(1);
        for (int i2{0}; i2 < loop_ub; i2++) {
            varargin_1[((i2 + H_size_tmp) + idx) + last] = (bi.NodeMap[static_cast<int>(j) - 1] - 1.0) + static_cast<double>(m4[i2]);
        }

        loop_ub = y.size(1);
        for (int i2{0}; i2 < loop_ub; i2++) {
            varargin_1[(((i2 + H_size_tmp) + idx) + last) + varargin_1.size(0)] = (bi.NodeMap[static_cast<int>(j) - 1] - 1.0) + y[i2];
        }

        varargin_2_size_idx_0 = static_cast<unsigned char>(((hessii_size[0] * hessii_size[1] + hessij_size[0] * hessij_size[1]) +
                                                            hessji_size[0] * hessji_size[1]) +
                                                           hessjj_size[0] * hessjj_size
                                                                                [1]);
        if (varargin_1.size(0) != 0) {
            idx = varargin_1.size(0);
        } else if (varargin_2_size_idx_0 != 0) {
            idx = H_size;
        } else {
            idx = 0;
            if (H_size > 0) {
                idx = H_size;
            }
        }

        empty_non_axis_sizes = (idx == 0);
        if (empty_non_axis_sizes || (varargin_1.size(0) != 0)) {
            i3 = 2;
        } else {
            i3 = 0;
        }

        if (empty_non_axis_sizes || (varargin_2_size_idx_0 != 0)) {
            sizes_idx_1 = 1;
        } else {
            sizes_idx_1 = 0;
        }

        loop_ub = i3;
        for (int i2{0}; i2 < loop_ub; i2++) {
            for (last = 0; last < idx; last++) {
                bi.HessianCSC[(i1 + last) + bi.HessianCSC.size(0) * i2] =
                    varargin_1[last + idx * i2];
            }
        }

        loop_ub = sizes_idx_1;
        for (int i2{0}; i2 < loop_ub; i2++) {
            for (last = 0; last < idx; last++) {
                bi.HessianCSC[(i1 + last) + bi.HessianCSC.size(0) * i3] =
                    H_data[last];
            }
        }

        bi.HessianCSCCount = numEntries1;
    }

    if (args_poseDeltaLength < 0.0) {
        maxNodeDim = 0.0;
        idx = 0;
    } else {
        maxNodeDim = args_poseDeltaLength;
        idx = static_cast<int>(args_poseDeltaLength);
    }

    b_I.set_size(static_cast<int>(maxNodeDim), static_cast<int>(maxNodeDim));
    loop_ub = static_cast<int>(maxNodeDim) * static_cast<int>(maxNodeDim);
    for (i = 0; i < loop_ub; i++) {
        b_I[i] = 0;
    }

    if (static_cast<int>(maxNodeDim) > 0) {
        for (k = 0; k < idx; k++) {
            b_I[k + b_I.size(0) * k] = 1;
        }
    }

    last = b_I.size(0) * b_I.size(1);
    idx = b_I.size(0) * b_I.size(1);
    if (idx < 1) {
        y.set_size(1, 0);
    } else {
        y.set_size(1, idx);
        loop_ub = idx - 1;
        for (i = 0; i <= loop_ub; i++) {
            y[i] = static_cast<double>(i) + 1.0;
        }
    }

    idx = b_I.size(0);
    v1.set_size(1, y.size(1));
    loop_ub = y.size(1);
    for (i = 0; i < loop_ub; i++) {
        v1[i] = static_cast<int>(y[i]) - 1;
    }

    vk.set_size(1, v1.size(1));
    loop_ub = v1.size(1);
    for (i = 0; i < loop_ub; i++) {
        vk[i] = div_s32(v1[i], idx);
    }

    v1.set_size(1, v1.size(1));
    loop_ub = v1.size(1) - 1;
    for (i = 0; i <= loop_ub; i++) {
        v1[i] = v1[i] - vk[i] * idx;
    }

    y.set_size(1, v1.size(1));
    loop_ub = v1.size(1);
    for (i = 0; i < loop_ub; i++) {
        y[i] = v1[i] + 1;
    }

    m4.set_size(1, vk.size(1));
    loop_ub = vk.size(1);
    for (i = 0; i < loop_ub; i++) {
        m4[i] = vk[i] + 1;
    }

    if (bi.HessianCSCCount > (bi.HessianCSCCount + static_cast<double>(last)) - 1.0) {
        i = 0;
    } else {
        i = static_cast<int>(bi.HessianCSCCount) - 1;
    }

    H_size_tmp = b_I.size(0) * b_I.size(1);
    loop_ub = y.size(1);
    for (i1 = 0; i1 < loop_ub; i1++) {
        bi.HessianCSC[i + i1] = (bi.NodeMap[0] + y[i1]) - 1.0;
    }

    loop_ub = m4.size(1);
    for (i1 = 0; i1 < loop_ub; i1++) {
        bi.HessianCSC[(i + i1) + bi.HessianCSC.size(0)] = (bi.NodeMap[0] +
                                                           static_cast<double>(m4[i1])) -
                                                          1.0;
    }

    for (i1 = 0; i1 < H_size_tmp; i1++) {
        bi.HessianCSC[(i + i1) + bi.HessianCSC.size(0) * 2] = b_I[i1];
    }

    bi.HessianCSCCount += static_cast<double>(last);
    numEntries1 = (args_nodeMap[static_cast<int>(poses.NumRowBlocks) - 1] + args_nodeDims[static_cast<int>(poses.NumRowBlocks) - 1]) - 1.0;
    if (numEntries1 < 1.0) {
        loop_ub = 0;
    } else {
        loop_ub = static_cast<int>(numEntries1);
    }

    gradient.set_size(loop_ub);
    for (i = 0; i < loop_ub; i++) {
        gradient[i] = bi.Gradient[i];
    }

    if (bi.HessianCSCCount - 1.0 < 1.0) {
        loop_ub = 0;
    } else {
        loop_ub = static_cast<int>(bi.HessianCSCCount - 1.0);
    }

    b_bi.set_size(loop_ub);
    c_bi.set_size(loop_ub);
    d_bi.set_size(loop_ub);
    for (i = 0; i < loop_ub; i++) {
        b_bi[i] = bi.HessianCSC[i];
        c_bi[i] = bi.HessianCSC[i + bi.HessianCSC.size(0)];
        d_bi[i] = bi.HessianCSC[i + bi.HessianCSC.size(0) * 2];
    }

    b_sparse(b_bi, c_bi, d_bi, hessian);
}
}  // namespace internal
}  // namespace algs
}  // namespace nav

void poseGraph::nodeEstimates(::coder::array<double, 2U> &nodeEsts) const {
    ::coder::array<double, 2U> T;
    ::coder::array<double, 2U> nodeIds;
    double L;
    int i;
    int loop_ub;
    L = NumNodes;
    if (std::isnan(L)) {
        nodeIds.set_size(1, 1);
        nodeIds[0] = rtNaN;
    } else if (L < 1.0) {
        nodeIds.set_size(1, 0);
    } else {
        nodeIds.set_size(1, static_cast<int>(L - 1.0) + 1);
        loop_ub = static_cast<int>(L - 1.0);
        for (i = 0; i <= loop_ub; i++) {
            nodeIds[i] = static_cast<double>(i) + 1.0;
        }
    }

    i = static_cast<int>(L);
    nodeEsts.set_size(i, 3);
    for (loop_ub = 0; loop_ub < i; loop_ub++) {
        L = nodeIds[loop_ub];
        if (IsLandmarkNode[static_cast<int>(L) - 1]) {
            NodeEstimates->extractBlock(L, T);
            nodeEsts[loop_ub] = T[T.size(0) * 2];
            nodeEsts[loop_ub + nodeEsts.size(0)] = T[T.size(0) * 2 + 1];
            nodeEsts[loop_ub + nodeEsts.size(0) * 2] = rtNaN;
        } else {
            NodeEstimates->extractBlock(L, T);
            nodeEsts[loop_ub] = T[T.size(0) * 2];
            nodeEsts[loop_ub + nodeEsts.size(0)] = T[T.size(0) * 2 + 1];
            nodeEsts[loop_ub + nodeEsts.size(0) * 2] = rt_atan2d_snf(T[1], T[0]);
        }
    }
}

namespace robotics {
namespace core {
namespace internal {
void TrustRegionIndefiniteDogLegSE2::computeBasicSteps(const ::coder::
                                                           array<double, 1U> &grad,
                                                       const sparse *B, ::coder::array<double, 1U> &stepSD, ::coder::array<double, 1U> &stepGN, bool *localMin) const {
    sparse in;
    ::coder::array<double, 2U> a;
    ::coder::array<double, 1U> b_stepGN;
    double b_grad;
    double cd;
    int apend;
    int i;
    a.set_size(1, B->n);
    apend = B->n;
    for (i = 0; i < apend; i++) {
        a[i] = 0.0;
    }

    if ((grad.size(0) != 0) && (B->n != 0) && (B->colidx[B->colidx.size(0) - 1] - 1 != 0)) {
        i = B->n;
        for (int k{0}; k < i; k++) {
            int i1;
            cd = 0.0;
            apend = B->colidx[k + 1] - 1;
            i1 = B->colidx[k];
            for (int ap{i1}; ap <= apend; ap++) {
                cd += B->d[ap - 1] * grad[B->rowidx[ap - 1] - 1];
            }

            a[k] = cd;
        }
    }

    b_grad = 0.0;
    apend = grad.size(0);
    for (i = 0; i < apend; i++) {
        cd = grad[i];
        b_grad += cd * cd;
    }

    cd = 0.0;
    apend = a.size(1);
    for (i = 0; i < apend; i++) {
        cd += a[i] * grad[i];
    }

    cd = b_grad / cd;
    stepGN.set_size(grad.size(0));
    apend = grad.size(0);
    for (i = 0; i < apend; i++) {
        stepGN[i] = -grad[i];
    }

    stepSD.set_size(stepGN.size(0));
    apend = stepGN.size(0);
    for (i = 0; i < apend; i++) {
        stepSD[i] = cd * stepGN[i];
    }

    if ((B->m == 0) || (B->n == 0) || (stepGN.size(0) == 0)) {
        stepGN.set_size(B->n);
        apend = B->n;
        for (i = 0; i < apend; i++) {
            stepGN[i] = 0.0;
        }
    } else if (stepGN.size(0) == B->n) {
        cs_di *cxA;
        cs_din *N;
        cs_dis *S;
        if (B->m < B->n) {
            B->ctranspose(&in);
            cxA = makeCXSparseMatrix(in.colidx[in.colidx.size(0) - 1] - 1,
                                     in.n, in.m, &(in.colidx.data())[0], &(in.rowidx.data())[0],
                                     &(in.d.data())[0]);
        } else {
            cxA = makeCXSparseMatrix(B->colidx[B->colidx.size(0) - 1] - 1,
                                     B->n, B->m, &(((::coder::array<int, 1U> *)&B->colidx)->data())[0], &(((::coder::array<int, 1U> *)&B->rowidx)->data())[0],
                                     &(((::coder::array<double, 1U> *)&B->d)->data())[0]);
        }

        S = cs_di_sqr(2, cxA, 0);
        N = cs_di_lu(cxA, S, 1);
        cs_di_spfree(cxA);
        if (N == nullptr) {
            cs_di_sfree(S);
            cs_di_nfree(N);
            b_stepGN.set_size(stepGN.size(0));
            apend = stepGN.size(0) - 1;
            for (i = 0; i <= apend; i++) {
                b_stepGN[i] = stepGN[i];
            }

            ::buildMapping::coder::internal::CXSparseAPI::iteratedQR(B,
                                                                     b_stepGN, B->n, stepGN);
        } else {
            solve_from_lu_di(N, S, (double *)&(stepGN.data())[0],
                             stepGN.size(0));
            cs_di_sfree(S);
            cs_di_nfree(N);
        }
    } else {
        b_stepGN.set_size(stepGN.size(0));
        apend = stepGN.size(0) - 1;
        for (i = 0; i <= apend; i++) {
            b_stepGN[i] = stepGN[i];
        }

        ::buildMapping::coder::internal::CXSparseAPI::iteratedQR(B,
                                                                 b_stepGN, B->n, stepGN);
    }

    *localMin = (b_norm(grad) < GradientTolerance);
}

void SEHelpers::expSE3hat(const double e[6], double T[16]) {
    double Sphi[9];
    double V[9];
    double absxk;
    double c;
    double c_tmp;
    double d;
    double d1;
    double d2;
    double scale;
    double t;
    double theta;
    int T_tmp;
    Sphi[0] = 0.0;
    Sphi[3] = -e[5];
    Sphi[6] = e[4];
    Sphi[1] = e[5];
    Sphi[4] = 0.0;
    Sphi[7] = -e[3];
    Sphi[2] = -e[4];
    Sphi[5] = e[3];
    Sphi[8] = 0.0;
    scale = 3.3121686421112381E-170;
    absxk = std::abs(e[3]);
    if (absxk > 3.3121686421112381E-170) {
        theta = 1.0;
        scale = absxk;
    } else {
        t = absxk / 3.3121686421112381E-170;
        theta = t * t;
    }

    absxk = std::abs(e[4]);
    if (absxk > scale) {
        t = scale / absxk;
        theta = theta * t * t + 1.0;
        scale = absxk;
    } else {
        t = absxk / scale;
        theta += t * t;
    }

    absxk = std::abs(e[5]);
    if (absxk > scale) {
        t = scale / absxk;
        theta = theta * t * t + 1.0;
        scale = absxk;
    } else {
        t = absxk / scale;
        theta += t * t;
    }

    theta = scale * std::sqrt(theta);
    scale = theta * theta;
    absxk = 1.0 - std::cos(theta);
    t = absxk / scale;
    c_tmp = std::sin(theta);
    c = (theta - c_tmp) / (scale * theta);
    if ((t < 2.2204460492503131E-16) || (std::isinf(t) || std::isnan(t))) {
        std::memset(&V[0], 0, 9U * sizeof(double));
        V[0] = 1.0;
        V[4] = 1.0;
        V[8] = 1.0;
        std::copy(&V[0], &V[9], &Sphi[0]);
    } else {
        double b_Sphi[9];
        signed char V_tmp[9];
        for (int i{0}; i < 9; i++) {
            V_tmp[i] = 0;
        }

        V_tmp[0] = 1;
        V_tmp[4] = 1;
        V_tmp[8] = 1;
        for (int i{0}; i < 3; i++) {
            d = Sphi[i + 3];
            d1 = Sphi[i + 6];
            for (T_tmp = 0; T_tmp < 3; T_tmp++) {
                double d3;
                double d4;
                int b_V_tmp;
                d2 = Sphi[3 * T_tmp];
                d3 = Sphi[3 * T_tmp + 1];
                d4 = Sphi[3 * T_tmp + 2];
                b_V_tmp = i + 3 * T_tmp;
                V[b_V_tmp] = (static_cast<double>(V_tmp[b_V_tmp]) + t *
                                                                        Sphi[b_V_tmp]) +
                             ((c * Sphi[i] * d2 + c * d * d3) + c * d1 * d4);
                b_Sphi[b_V_tmp] = ((Sphi[i] * d2 + d * d3) + d1 * d4) / scale;
            }
        }

        for (int i{0}; i < 9; i++) {
            Sphi[i] = (static_cast<double>(V_tmp[i]) + Sphi[i] * c_tmp /
                                                           theta) +
                      b_Sphi[i] * absxk;
        }
    }

    d = e[0];
    d1 = e[1];
    d2 = e[2];
    for (int i{0}; i < 3; i++) {
        T_tmp = i << 2;
        T[T_tmp] = Sphi[3 * i];
        T[T_tmp + 1] = Sphi[3 * i + 1];
        T[T_tmp + 2] = Sphi[3 * i + 2];
        T[i + 12] = (V[i] * d + V[i + 3] * d1) + V[i + 6] * d2;
    }

    T[3] = 0.0;
    T[7] = 0.0;
    T[11] = 0.0;
    T[15] = 1.0;
}

void BlockMatrix::extractBlock(double i, ::coder::array<double, 2U> &B)
    const {
    double colStart;
    double d;
    double rowStart;
    int b_i;
    int b_loop_ub;
    int i1;
    int loop_ub;
    rowStart = BlockSize[0] * (i - 1.0) + 1.0;
    colStart = BlockSize[1] * 0.0 + 1.0;
    d = (rowStart + BlockSize[0]) - 1.0;
    if (rowStart > d) {
        b_i = 0;
        i1 = 0;
    } else {
        b_i = static_cast<int>(rowStart) - 1;
        i1 = static_cast<int>(d);
    }

    d = (colStart + BlockSize[1]) - 1.0;
    if (colStart > d) {
        loop_ub = 0;
    } else {
        loop_ub = static_cast<int>(d);
    }

    b_loop_ub = i1 - b_i;
    B.set_size(b_loop_ub, loop_ub);
    for (i1 = 0; i1 < loop_ub; i1++) {
        for (int i2{0}; i2 < b_loop_ub; i2++) {
            B[i2 + B.size(0) * i1] = Matrix[(b_i + i2) + Matrix.size(0) * i1];
        }
    }
}

void TrustRegionIndefiniteDogLegSE2::incrementX(const ::coder::array<
                                                    double, 2U> &x,
                                                const ::coder::array<double, 1U> &epsilons, ::coder::array<double, 2U> &xNew) const {
    BlockMatrix xBlk;
    b_BlockMatrix xBlkNew;
    ::coder::array<double, 2U> T;
    double b_T[3];
    int i;
    int i1;
    int loop_ub;
    xBlk.Matrix.set_size(x.size(0), 3);
    loop_ub = x.size(0) * 3;
    for (i = 0; i < loop_ub; i++) {
        xBlk.Matrix[i] = x[i];
    }

    xBlk.BlockSize[0] = 3.0;
    xBlk.BlockSize[1] = 3.0;
    xBlk.NumRowBlocks = static_cast<double>(x.size(0)) / 3.0;
    xBlk.NumColBlocks = 1.0;
    i = static_cast<int>(xBlk.NumRowBlocks * 3.0);
    xBlkNew.Matrix.set_size(i, 3);
    loop_ub = i * 3;
    for (i = 0; i < loop_ub; i++) {
        xBlkNew.Matrix[i] = 0.0;
    }

    i = static_cast<int>(xBlk.NumRowBlocks);
    for (int b_i{0}; b_i < i; b_i++) {
        double idx;
        double len;
        xBlk.extractBlock(static_cast<double>(b_i) + 1.0, T);
        idx = ExtraArgs.nodeMap[b_i];
        len = ExtraArgs.nodeDims[b_i];
        if (len == 3.0) {
            double T_tmp[9];
            int i2;
            int rowStart;
            if (idx > (idx + 3.0) - 1.0) {
                i1 = 0;
                i2 = -2;
            } else {
                i1 = static_cast<int>(idx) - 1;
                i2 = static_cast<int>(static_cast<unsigned int>(idx));
            }

            if ((i2 - i1) + 2 == 3) {
                b_T[0] = T[T.size(0) * 2];
                b_T[1] = T[T.size(0) * 2 + 1];
                b_T[2] = rt_atan2d_snf(T[1], T[0]);
                b_T[0] += epsilons[i1];
                b_T[1] += epsilons[i1 + 1];
                b_T[2] += epsilons[i1 + 2];
            } else {
                b_binary_expand_op(b_T, T, epsilons, i1, i2 + 1);
            }

            len = std::sin(b_T[2]);
            idx = std::cos(b_T[2]);
            rowStart = 3 * b_i + 1;
            if (static_cast<unsigned int>(rowStart) > static_cast<unsigned int>(rowStart) + 2U) {
                i1 = 0;
                i2 = 0;
            } else {
                i1 = rowStart - 1;
                i2 = static_cast<int>(static_cast<unsigned int>(rowStart) + 2U);
            }

            T_tmp[0] = idx;
            T_tmp[3] = -len;
            T_tmp[6] = b_T[0];
            T_tmp[1] = len;
            T_tmp[4] = idx;
            T_tmp[7] = b_T[1];
            T_tmp[2] = 0.0;
            T_tmp[5] = 0.0;
            T_tmp[8] = 1.0;
            loop_ub = i2 - i1;
            for (i2 = 0; i2 < 3; i2++) {
                for (rowStart = 0; rowStart < loop_ub; rowStart++) {
                    xBlkNew.Matrix[(i1 + rowStart) + xBlkNew.Matrix.size(0) * i2] = T_tmp[rowStart + loop_ub * i2];
                }
            }
        } else {
            int i2;
            int rowStart;
            len = (idx + len) - 1.0;
            if (idx > len) {
                i1 = 0;
                i2 = 0;
            } else {
                i1 = static_cast<int>(idx) - 1;
                i2 = static_cast<int>(len);
            }

            if (i2 - i1 == 2) {
                T[T.size(0) * 2] = T[T.size(0) * 2] + epsilons[i1];
                T[T.size(0) * 2 + 1] = T[T.size(0) * 2 + 1] + epsilons[i1 + 1];
            } else {
                b_binary_expand_op(T, epsilons, i1, i2 - 1);
            }

            rowStart = 3 * b_i;
            if (static_cast<unsigned int>(rowStart + 1) > static_cast<
                                                              unsigned int>(rowStart) +
                                                              3U) {
                rowStart = 0;
            }

            loop_ub = T.size(1);
            for (i1 = 0; i1 < loop_ub; i1++) {
                int b_loop_ub;
                b_loop_ub = T.size(0);
                for (i2 = 0; i2 < b_loop_ub; i2++) {
                    xBlkNew.Matrix[(rowStart + i2) + xBlkNew.Matrix.size(0) * i1] = T[i2 + T.size(0) * i1];
                }
            }
        }
    }

    xNew.set_size(xBlkNew.Matrix.size(0), 3);
    loop_ub = xBlkNew.Matrix.size(0);
    for (i = 0; i < 3; i++) {
        for (i1 = 0; i1 < loop_ub; i1++) {
            xNew[i1 + xNew.size(0) * i] = xBlkNew.Matrix[i1 +
                                                         xBlkNew.Matrix.size(0) * i];
        }
    }
}

void Sim3Helpers::multiplyLogSim3(const double S1[16], const double S2[16], const double S3[16], double e[7]) {
    double S12[16];
    double S123[16];
    double omega2[9];
    double w[9];
    double omega1[3];
    double a;
    double a1;
    double b1;
    double c;
    double d;
    double sigma;
    int b_omega2_tmp;
    int omega2_tmp;
    int r1;
    int r2;
    int r3;
    int rtemp;
    for (omega2_tmp = 0; omega2_tmp < 3; omega2_tmp++) {
        a1 = 0.0;
        for (int k{0}; k < 3; k++) {
            rtemp = k << 2;
            omega2[omega2_tmp + 3 * k] = (S1[omega2_tmp] * S2[rtemp] +
                                          S1[omega2_tmp + 4] * S2[rtemp + 1]) +
                                         S1[omega2_tmp + 8] *
                                             S2[rtemp + 2];
            a1 += S1[15] * S1[omega2_tmp + rtemp] * S2[k + 12];
        }

        omega1[omega2_tmp] = a1 + S1[omega2_tmp + 12];
    }

    for (omega2_tmp = 0; omega2_tmp < 3; omega2_tmp++) {
        rtemp = omega2_tmp << 2;
        S12[rtemp] = omega2[3 * omega2_tmp];
        S12[rtemp + 1] = omega2[3 * omega2_tmp + 1];
        S12[rtemp + 2] = omega2[3 * omega2_tmp + 2];
        S12[omega2_tmp + 12] = omega1[omega2_tmp];
    }

    S12[3] = 0.0;
    S12[7] = 0.0;
    S12[11] = 0.0;
    S12[15] = S1[15] * S2[15];
    for (omega2_tmp = 0; omega2_tmp < 3; omega2_tmp++) {
        a1 = 0.0;
        for (int k{0}; k < 3; k++) {
            rtemp = k << 2;
            omega2[omega2_tmp + 3 * k] = (S12[omega2_tmp] * S3[rtemp] +
                                          S12[omega2_tmp + 4] * S3[rtemp + 1]) +
                                         S12[omega2_tmp + 8] *
                                             S3[rtemp + 2];
            a1 += S12[15] * S12[omega2_tmp + rtemp] * S3[k + 12];
        }

        omega1[omega2_tmp] = a1 + S12[omega2_tmp + 12];
    }

    for (omega2_tmp = 0; omega2_tmp < 3; omega2_tmp++) {
        rtemp = omega2_tmp << 2;
        S123[rtemp] = omega2[3 * omega2_tmp];
        S123[rtemp + 1] = omega2[3 * omega2_tmp + 1];
        S123[rtemp + 2] = omega2[3 * omega2_tmp + 2];
        S123[omega2_tmp + 12] = omega1[omega2_tmp];
    }

    S123[3] = 0.0;
    S123[7] = 0.0;
    S123[11] = 0.0;
    S123[15] = S12[15] * S3[15];
    sigma = std::log(S123[15]);
    d = 0.5 * (((S123[0] + S123[5]) + S123[10]) - 1.0);
    if (std::abs(sigma) < 1.0E-5) {
        c = 1.0 - sigma / 2.0;
        if (d > 0.99999) {
            for (omega2_tmp = 0; omega2_tmp < 3; omega2_tmp++) {
                b_omega2_tmp = omega2_tmp << 2;
                omega2[3 * omega2_tmp] = S123[b_omega2_tmp] - S123[omega2_tmp];
                omega2[3 * omega2_tmp + 1] = S123[b_omega2_tmp + 1] -
                                             S123[omega2_tmp + 4];
                omega2[3 * omega2_tmp + 2] = S123[b_omega2_tmp + 2] -
                                             S123[omega2_tmp + 8];
            }

            omega1[0] = 0.5 * omega2[5];
            omega1[1] = 0.5 * omega2[6];
            omega1[2] = 0.5 * omega2[1];
            omega2[0] = 0.0;
            omega2[3] = -omega1[2];
            omega2[6] = omega1[1];
            omega2[1] = omega1[2];
            omega2[4] = 0.0;
            omega2[7] = -omega1[0];
            omega2[2] = -omega1[1];
            omega2[5] = omega1[0];
            omega2[8] = 0.0;
            a = 0.5;
            a1 = 0.16666666666666666;
        } else {
            double th;
            double thSquare;
            th = std::acos(d);
            thSquare = th * th;
            a = th / (2.0 * std::sqrt(1.0 - d * d));
            for (omega2_tmp = 0; omega2_tmp < 3; omega2_tmp++) {
                b_omega2_tmp = omega2_tmp << 2;
                omega2[3 * omega2_tmp] = S123[b_omega2_tmp] - S123[omega2_tmp];
                omega2[3 * omega2_tmp + 1] = S123[b_omega2_tmp + 1] -
                                             S123[omega2_tmp + 4];
                omega2[3 * omega2_tmp + 2] = S123[b_omega2_tmp + 2] -
                                             S123[omega2_tmp + 8];
            }

            omega1[0] = a * omega2[5];
            omega1[1] = a * omega2[6];
            omega1[2] = a * omega2[1];
            omega2[0] = 0.0;
            omega2[3] = -omega1[2];
            omega2[6] = omega1[1];
            omega2[1] = omega1[2];
            omega2[4] = 0.0;
            omega2[7] = -omega1[0];
            omega2[2] = -omega1[1];
            omega2[5] = omega1[0];
            omega2[8] = 0.0;
            a = (1.0 - std::cos(th)) / thSquare;
            a1 = (th - std::sin(th)) / (thSquare * th);
        }
    } else {
        c = (S123[15] - 1.0) / sigma;
        if (d > 0.99999) {
            a1 = sigma * sigma;
            for (omega2_tmp = 0; omega2_tmp < 3; omega2_tmp++) {
                b_omega2_tmp = omega2_tmp << 2;
                omega2[3 * omega2_tmp] = S123[b_omega2_tmp] - S123[omega2_tmp];
                omega2[3 * omega2_tmp + 1] = S123[b_omega2_tmp + 1] -
                                             S123[omega2_tmp + 4];
                omega2[3 * omega2_tmp + 2] = S123[b_omega2_tmp + 2] -
                                             S123[omega2_tmp + 8];
            }

            omega1[0] = 0.5 * omega2[5];
            omega1[1] = 0.5 * omega2[6];
            omega1[2] = 0.5 * omega2[1];
            omega2[0] = 0.0;
            omega2[3] = -omega1[2];
            omega2[6] = omega1[1];
            omega2[1] = omega1[2];
            omega2[4] = 0.0;
            omega2[7] = -omega1[0];
            omega2[2] = -omega1[1];
            omega2[5] = omega1[0];
            omega2[8] = 0.0;
            a = ((sigma - 1.0) * S123[15] + 1.0) / a1;
            a1 = (((0.5 * a1 - sigma) + 1.0) * S123[15] - 1.0) / (a1 * sigma);
        } else {
            double th;
            th = std::acos(d);
            a = th / (2.0 * std::sqrt(1.0 - d * d));
            for (omega2_tmp = 0; omega2_tmp < 3; omega2_tmp++) {
                b_omega2_tmp = omega2_tmp << 2;
                omega2[3 * omega2_tmp] = S123[b_omega2_tmp] - S123[omega2_tmp];
                omega2[3 * omega2_tmp + 1] = S123[b_omega2_tmp + 1] -
                                             S123[omega2_tmp + 4];
                omega2[3 * omega2_tmp + 2] = S123[b_omega2_tmp + 2] -
                                             S123[omega2_tmp + 8];
            }

            double thSquare;
            omega1[0] = a * omega2[5];
            omega1[1] = a * omega2[6];
            omega1[2] = a * omega2[1];
            omega2[0] = 0.0;
            omega2[3] = -omega1[2];
            omega2[6] = omega1[1];
            omega2[1] = omega1[2];
            omega2[4] = 0.0;
            omega2[7] = -omega1[0];
            omega2[2] = -omega1[1];
            omega2[5] = omega1[0];
            omega2[8] = 0.0;
            thSquare = th * th;
            a1 = S123[15] * std::sin(th);
            b1 = S123[15] * std::cos(th);
            d = thSquare + sigma * sigma;
            a = (a1 * sigma + (1.0 - b1) * th) / (th * d);
            a1 = (c - ((b1 - 1.0) * sigma + a1 * th) / d) / thSquare;
        }
    }

    for (omega2_tmp = 0; omega2_tmp < 3; omega2_tmp++) {
        for (int k{0}; k < 3; k++) {
            rtemp = omega2_tmp + 3 * k;
            w[rtemp] = (a * omega2[rtemp] + ((a1 * omega2[omega2_tmp] *
                                                  omega2[3 * k] +
                                              a1 * omega2[omega2_tmp + 3] * omega2[3 * k + 1]) +
                                             a1 * omega2[omega2_tmp + 6] * omega2[3 * k + 2])) +
                       c *
                           static_cast<double>(iv[rtemp]);
        }
    }

    r1 = 0;
    r2 = 1;
    r3 = 2;
    a1 = std::abs(w[0]);
    d = std::abs(w[1]);
    if (d > a1) {
        a1 = d;
        r1 = 1;
        r2 = 0;
    }

    if (std::abs(w[2]) > a1) {
        r1 = 2;
        r2 = 1;
        r3 = 0;
    }

    w[r2] /= w[r1];
    w[r3] /= w[r1];
    w[r2 + 3] -= w[r2] * w[r1 + 3];
    w[r3 + 3] -= w[r3] * w[r1 + 3];
    w[r2 + 6] -= w[r2] * w[r1 + 6];
    w[r3 + 6] -= w[r3] * w[r1 + 6];
    if (std::abs(w[r3 + 3]) > std::abs(w[r2 + 3])) {
        rtemp = r2;
        r2 = r3;
        r3 = rtemp;
    }

    w[r3 + 3] /= w[r2 + 3];
    w[r3 + 6] -= w[r3 + 3] * w[r2 + 6];
    a1 = S123[12];
    d = S123[13];
    b1 = S123[14];
    for (int k{0}; k < 3; k++) {
        b_omega2_tmp = k + 3 * r1;
        omega2[b_omega2_tmp] = static_cast<double>(iv[k]) / w[r1];
        rtemp = k + 3 * r2;
        omega2[rtemp] = static_cast<double>(iv[k + 3]) -
                        omega2[b_omega2_tmp] * w[r1 + 3];
        omega2_tmp = k + 3 * r3;
        omega2[omega2_tmp] = static_cast<double>(iv[k + 6]) -
                             omega2[b_omega2_tmp] * w[r1 + 6];
        omega2[rtemp] /= w[r2 + 3];
        omega2[omega2_tmp] -= omega2[rtemp] * w[r2 + 6];
        omega2[omega2_tmp] /= w[r3 + 6];
        omega2[rtemp] -= omega2[omega2_tmp] * w[r3 + 3];
        omega2[b_omega2_tmp] -= omega2[omega2_tmp] * w[r3];
        omega2[b_omega2_tmp] -= omega2[rtemp] * w[r2];
        e[k] = (omega2[k] * a1 + omega2[k + 3] * d) + omega2[k + 6] * b1;
        e[k + 3] = omega1[k];
    }

    e[6] = sigma;
}

void BlockMatrix::replaceBlock(double i, const double blockij[9]) {
    double colStart;
    double d;
    double rowStart;
    int b_i;
    int i1;
    int loop_ub;
    int unnamed_idx_0;
    rowStart = BlockSize[0] * (i - 1.0) + 1.0;
    colStart = BlockSize[1] * 0.0 + 1.0;
    d = (rowStart + BlockSize[0]) - 1.0;
    if (rowStart > d) {
        b_i = 0;
        i1 = 0;
    } else {
        b_i = static_cast<int>(rowStart) - 1;
        i1 = static_cast<int>(d);
    }

    d = (colStart + BlockSize[1]) - 1.0;
    unnamed_idx_0 = i1 - b_i;
    if (colStart > d) {
        loop_ub = 0;
    } else {
        loop_ub = static_cast<int>(d);
    }

    for (i1 = 0; i1 < loop_ub; i1++) {
        for (int i2{0}; i2 < unnamed_idx_0; i2++) {
            Matrix[(b_i + i2) + Matrix.size(0) * i1] = blockij[i2 +
                                                               unnamed_idx_0 * i1];
        }
    }
}

void BlockMatrix::replaceBlock() {
    double colStart;
    double d;
    double d1;
    double rowStart;
    int b_loop_ub;
    int loop_ub;
    int unnamed_idx_0;
    rowStart = BlockSize[0] * 0.0 + 1.0;
    colStart = BlockSize[1] * 0.0 + 1.0;
    d = (rowStart + BlockSize[0]) - 1.0;
    if (rowStart > d) {
        loop_ub = 0;
    } else {
        loop_ub = static_cast<int>(d);
    }

    d1 = (colStart + BlockSize[1]) - 1.0;
    if (rowStart > d) {
        unnamed_idx_0 = 0;
    } else {
        unnamed_idx_0 = static_cast<int>(d);
    }

    if (colStart > d1) {
        b_loop_ub = 0;
    } else {
        b_loop_ub = static_cast<int>(d1);
    }

    for (int i{0}; i < b_loop_ub; i++) {
        for (int i1{0}; i1 < loop_ub; i1++) {
            Matrix[i1 + Matrix.size(0) * i] = iv[i1 + unnamed_idx_0 * i];
        }
    }
}

void Sim3Helpers::sim3ToSform(const double minVecSim3[7], double S[16]) {
    double R[9];
    double w[9];
    double wSquare[9];
    double a_tmp;
    double absxk;
    double c;
    double s;
    double scale;
    double t;
    double th;
    int S_tmp;
    w[0] = 0.0;
    w[3] = -minVecSim3[5];
    w[6] = minVecSim3[4];
    w[1] = minVecSim3[5];
    w[4] = 0.0;
    w[7] = -minVecSim3[3];
    w[2] = -minVecSim3[4];
    w[5] = minVecSim3[3];
    w[8] = 0.0;
    scale = 3.3121686421112381E-170;
    absxk = std::abs(minVecSim3[3]);
    if (absxk > 3.3121686421112381E-170) {
        th = 1.0;
        scale = absxk;
    } else {
        t = absxk / 3.3121686421112381E-170;
        th = t * t;
    }

    absxk = std::abs(minVecSim3[4]);
    if (absxk > scale) {
        t = scale / absxk;
        th = th * t * t + 1.0;
        scale = absxk;
    } else {
        t = absxk / scale;
        th += t * t;
    }

    absxk = std::abs(minVecSim3[5]);
    if (absxk > scale) {
        t = scale / absxk;
        th = th * t * t + 1.0;
        scale = absxk;
    } else {
        t = absxk / scale;
        th += t * t;
    }

    th = scale * std::sqrt(th);
    s = std::exp(minVecSim3[6]);
    for (int i{0}; i < 3; i++) {
        for (S_tmp = 0; S_tmp < 3; S_tmp++) {
            wSquare[i + 3 * S_tmp] = (w[i] * w[3 * S_tmp] + w[i + 3] * w[3 *
                                                                             S_tmp +
                                                                         1]) +
                                     w[i + 6] * w[3 * S_tmp + 2];
        }
    }

    if (std::abs(minVecSim3[6]) < 1.0E-5) {
        c = 1.0;
        if (th < 1.0E-5) {
            a_tmp = 0.5;
            t = 0.16666666666666666;
            std::memset(&R[0], 0, 9U * sizeof(double));
            R[0] = 1.0;
            R[4] = 1.0;
            R[8] = 1.0;
            for (int i{0}; i < 9; i++) {
                R[i] = (R[i] + w[i]) + wSquare[i] / 2.0;
            }
        } else {
            double thSquare;
            thSquare = th * th;
            a_tmp = (1.0 - std::cos(th)) / thSquare;
            scale = std::sin(th);
            t = (th - scale) / (thSquare * th);
            absxk = scale / th;
            std::memset(&R[0], 0, 9U * sizeof(double));
            R[0] = 1.0;
            R[4] = 1.0;
            R[8] = 1.0;
            for (int i{0}; i < 9; i++) {
                R[i] = (R[i] + absxk * w[i]) + a_tmp * wSquare[i];
            }
        }
    } else {
        c = (s - 1.0) / minVecSim3[6];
        if (th < 1.0E-5) {
            scale = minVecSim3[6] * minVecSim3[6];
            a_tmp = ((minVecSim3[6] - 1.0) * s + 1.0) / scale;
            t = (((0.5 * scale - minVecSim3[6]) + 1.0) * s - 1.0) / (scale *
                                                                     minVecSim3[6]);
            std::memset(&R[0], 0, 9U * sizeof(double));
            R[0] = 1.0;
            R[4] = 1.0;
            R[8] = 1.0;
            for (int i{0}; i < 9; i++) {
                R[i] = (R[i] + w[i]) + wSquare[i] / 2.0;
            }
        } else {
            double thSquare;
            scale = s * std::sin(th);
            absxk = s * std::cos(th);
            thSquare = th * th;
            t = thSquare + minVecSim3[6] * minVecSim3[6];
            a_tmp = (scale * minVecSim3[6] + (1.0 - absxk) * th) / (th * t);
            t = (c - ((absxk - 1.0) * minVecSim3[6] + scale * th) / t) /
                thSquare;
            absxk = std::sin(th) / th;
            scale = (1.0 - std::cos(th)) / thSquare;
            std::memset(&R[0], 0, 9U * sizeof(double));
            R[0] = 1.0;
            R[4] = 1.0;
            R[8] = 1.0;
            for (int i{0}; i < 9; i++) {
                R[i] = (R[i] + absxk * w[i]) + scale * wSquare[i];
            }
        }
    }

    for (int i{0}; i < 9; i++) {
        w[i] = (a_tmp * w[i] + t * wSquare[i]) + c * static_cast<double>(iv[i]);
    }

    scale = minVecSim3[0];
    absxk = minVecSim3[1];
    t = minVecSim3[2];
    for (int i{0}; i < 3; i++) {
        S_tmp = i << 2;
        S[S_tmp] = R[3 * i];
        S[S_tmp + 1] = R[3 * i + 1];
        S[S_tmp + 2] = R[3 * i + 2];
        S[i + 12] = (w[i] * scale + w[i + 3] * absxk) + w[i + 6] * t;
        S[S_tmp + 3] = 0.0;
    }

    S[15] = s;
}

void TrustRegionIndefiniteDogLegSE2::solve(HDMapping *aInstancePtr,
                                           const ::coder::array<double, 2U> &seed, BlockMatrix *iobj_0,
                                           BlockMatrix **xSol, double *solutionInfo_Iterations, double *solutionInfo_Error, double *solutionInfo_ExitFlag, sparse *hess) {
    BlockMatrix *b_xSol;
    sparse HessianTrial;
    sparse c;
    ::coder::array<double, 2U> x;
    ::coder::array<double, 2U> xn;
    ::coder::array<double, 1U> grad;
    ::coder::array<double, 1U> gradTrial;
    ::coder::array<double, 1U> stepDL_;
    ::coder::array<double, 1U> stepGN;
    ::coder::array<double, 1U> stepSD;
    ::coder::array<double, 1U> tmpd;
    ::coder::array<bool, 1U> b_x;
    double costTrial;
    double delta;
    double err;
    int i;
    int nx;
    bool localMin;
    NLPSolverExitFlags exitFlag;
    MaxNumIterationInternal = MaxNumIteration;
    MaxTimeInternal = MaxTime;
    SeedInternal.set_size(seed.size(0), 3);
    nx = seed.size(0) * 3;
    for (i = 0; i < nx; i++) {
        SeedInternal[i] = seed[i];
    }

    tic(aInstancePtr, &TimeObj.StartTime.tv_sec,
        &TimeObj.StartTime.tv_nsec);
    x.set_size(SeedInternal.size(0), 3);
    nx = SeedInternal.size(0) * 3;
    for (i = 0; i < nx; i++) {
        x[i] = SeedInternal[i];
    }

    (&iobj_0[0])[0].Matrix.set_size(SeedInternal.size(0), 3);
    nx = SeedInternal.size(0) * 3;
    for (i = 0; i < nx; i++) {
        (&iobj_0[0])[0].Matrix[i] = SeedInternal[i];
    }

    (&iobj_0[0])[0].BlockSize[0] = 3.0;
    (&iobj_0[0])[0].BlockSize[1] = 3.0;
    (&iobj_0[0])[0].NumRowBlocks = static_cast<double>(SeedInternal.size(0)) / 3.0;
    (&iobj_0[0])[0].NumColBlocks = 1.0;
    b_xSol = &(&iobj_0[0])[0];
    *solutionInfo_Iterations = 0.0;
    exitFlag = NLPSolverExitFlags::IterationLimitExceeded;
    nav::algs::internal::PoseGraphHelpers::poseGraphCost(SeedInternal,
                                                         ExtraArgs.edgeNodePairs, ExtraArgs.edgeMeasurements,
                                                         ExtraArgs.edgeInfoMats, ExtraArgs.tformSize, ExtraArgs.infoMatSize,
                                                         ExtraArgs.poseDeltaLength, ExtraArgs.nodeMap, ExtraArgs.nodeDims,
                                                         ExtraArgs.IsLandmarkNode, &err, grad, hess);
    delta = InitialTrustRegionRadius;
    tmpd.set_size(grad.size(0));
    nx = grad.size(0);
    for (i = 0; i < nx; i++) {
        tmpd[i] = grad[i];
    }

    computeBasicSteps(tmpd, hess, stepSD, stepGN, &localMin);
    if (localMin) {
        exitFlag = NLPSolverExitFlags::LocalMinimumFound;
    } else {
        double d;
        int b_i;
        bool exitg1;
        bool terminated;
        terminated = false;
        d = MaxNumIterationInternal;
        b_i = 0;
        exitg1 = false;
        while ((!exitg1) && (b_i <= static_cast<int>(d) - 1)) {
            double val;
            val = toc(aInstancePtr, TimeObj.StartTime.tv_sec,
                      TimeObj.StartTime.tv_nsec);
            if (val > MaxTimeInternal) {
                exitFlag = NLPSolverExitFlags::TimeLimitExceeded;
                terminated = true;
                exitg1 = true;
            } else {
                double b_stepDL_;
                double bc;
                int currRowIdx;
                bool exitg2;
                val = b_norm(stepSD);
                if (val >= delta) {
                    val = delta / val;
                    stepDL_.set_size(stepSD.size(0));
                    nx = stepSD.size(0);
                    for (i = 0; i < nx; i++) {
                        stepDL_[i] = val * stepSD[i];
                    }
                } else if (b_norm(stepGN) <= delta) {
                    stepDL_.set_size(stepGN.size(0));
                    nx = stepGN.size(0);
                    for (i = 0; i < nx; i++) {
                        stepDL_[i] = stepGN[i];
                    }
                } else {
                    if (stepGN.size(0) == stepSD.size(0)) {
                        stepDL_.set_size(stepGN.size(0));
                        nx = stepGN.size(0);
                        for (i = 0; i < nx; i++) {
                            stepDL_[i] = stepGN[i] - stepSD[i];
                        }
                    } else {
                        minus(stepDL_, stepGN, stepSD);
                    }

                    bc = 0.0;
                    nx = stepSD.size(0);
                    for (i = 0; i < nx; i++) {
                        bc += stepSD[i] * stepDL_[i];
                    }

                    b_stepDL_ = 0.0;
                    nx = stepDL_.size(0);
                    for (i = 0; i < nx; i++) {
                        b_stepDL_ += stepDL_[i] * stepDL_[i];
                    }

                    val = (-bc + std::sqrt(bc * bc + b_stepDL_ * (delta * delta - val * val))) / b_stepDL_;
                    if (stepSD.size(0) == stepDL_.size(0)) {
                        stepDL_.set_size(stepSD.size(0));
                        nx = stepSD.size(0);
                        for (i = 0; i < nx; i++) {
                            stepDL_[i] = stepSD[i] + val * stepDL_[i];
                        }
                    } else {
                        b_binary_expand_op(stepDL_, stepSD, val);
                    }
                }

                nx = stepDL_.size(0);
                tmpd.set_size(stepDL_.size(0));
                for (currRowIdx = 0; currRowIdx < nx; currRowIdx++) {
                    tmpd[currRowIdx] = std::abs(stepDL_[currRowIdx]);
                }

                b_x.set_size(tmpd.size(0));
                nx = tmpd.size(0);
                for (i = 0; i < nx; i++) {
                    b_x[i] = (tmpd[i] < StepTolerance);
                }

                localMin = true;
                nx = 1;
                exitg2 = false;
                while ((!exitg2) && (nx <= b_x.size(0))) {
                    if (!b_x[nx - 1]) {
                        localMin = false;
                        exitg2 = true;
                    } else {
                        nx++;
                    }
                }

                if (localMin) {
                    exitFlag = NLPSolverExitFlags::StepSizeBelowMinimum;
                    terminated = true;
                    exitg1 = true;
                } else {
                    double d1;
                    incrementX(x, stepDL_, xn);
                    nav::algs::internal::PoseGraphHelpers::poseGraphCost(xn,
                                                                         ExtraArgs.edgeNodePairs, ExtraArgs.edgeMeasurements,
                                                                         ExtraArgs.edgeInfoMats, ExtraArgs.tformSize,
                                                                         ExtraArgs.infoMatSize, ExtraArgs.poseDeltaLength,
                                                                         ExtraArgs.nodeMap, ExtraArgs.nodeDims,
                                                                         ExtraArgs.IsLandmarkNode, &costTrial, gradTrial,
                                                                         &HessianTrial);
                    d1 = err - costTrial;
                    if (std::abs(d1) < FunctionTolerance) {
                        exitFlag = NLPSolverExitFlags::ChangeInErrorBelowMinimum;
                        terminated = true;
                        exitg1 = true;
                    } else {
                        int b_c;
                        int ridx;
                        bool guard1{false};

                        ridx = hess->colidx[hess->colidx.size(0) - 1];
                        if (hess->colidx[hess->colidx.size(0) - 1] - 1 < 1) {
                            nx = 0;
                        } else {
                            nx = hess->colidx[hess->colidx.size(0) - 1] - 1;
                        }

                        tmpd.set_size(nx);
                        for (i = 0; i < nx; i++) {
                            tmpd[i] = 0.5 * hess->d[i];
                        }

                        if (hess->colidx[hess->colidx.size(0) - 1] - 1 >= 1) {
                            nx = hess->colidx[hess->colidx.size(0) - 1] - 2;
                        } else {
                            nx = 0;
                        }

                        c.d.set_size(nx + 1);
                        for (i = 0; i <= nx; i++) {
                            c.d[i] = 0.0;
                        }

                        c.rowidx.set_size(nx + 1);
                        for (i = 0; i <= nx; i++) {
                            c.rowidx[i] = 0;
                        }

                        if (hess->colidx[hess->colidx.size(0) - 1] - 1 < 1) {
                            nx = 1;
                        } else {
                            nx = hess->colidx[hess->colidx.size(0) - 1];
                        }

                        for (i = 0; i <= nx - 2; i++) {
                            c.rowidx[i] = hess->rowidx[i];
                        }

                        c.colidx.set_size(hess->colidx.size(0));
                        nx = hess->colidx.size(0);
                        for (i = 0; i < nx; i++) {
                            c.colidx[i] = hess->colidx[i];
                        }

                        for (currRowIdx = 0; currRowIdx <= ridx - 2; currRowIdx++) {
                            c.d[currRowIdx] = tmpd[currRowIdx];
                        }

                        nx = 1;
                        i = hess->colidx.size(0);
                        for (b_c = 0; b_c <= i - 2; b_c++) {
                            ridx = c.colidx[b_c];
                            c.colidx[b_c] = nx;
                            while (ridx < c.colidx[b_c + 1]) {
                                currRowIdx = c.rowidx[ridx - 1];
                                val = c.d[ridx - 1];
                                ridx++;
                                if (val != 0.0) {
                                    c.d[nx - 1] = val;
                                    c.rowidx[nx - 1] = currRowIdx;
                                    nx++;
                                }
                            }
                        }

                        c.colidx[c.colidx.size(0) - 1] = nx;
                        tmpd.set_size(hess->m);
                        nx = hess->m;
                        for (i = 0; i < nx; i++) {
                            tmpd[i] = 0.0;
                        }

                        if ((hess->n != 0) && (hess->m != 0) &&
                            (c.colidx[c.colidx.size(0) - 1] - 1 != 0)) {
                            i = hess->n;
                            for (int acol{0}; acol < i; acol++) {
                                bc = stepDL_[acol];
                                b_c = c.colidx[acol];
                                nx = c.colidx[acol + 1];
                                ridx = nx - c.colidx[acol];
                                if (ridx >= 4) {
                                    ridx = (nx - ridx) + ((ridx / 4) << 2);
                                    for (int ap{b_c}; ap <= ridx - 1; ap += 4) {
                                        currRowIdx = c.rowidx[ap - 1] - 1;
                                        tmpd[currRowIdx] = tmpd[currRowIdx] + c.d[ap - 1] *
                                                                                  bc;
                                        tmpd[c.rowidx[ap] - 1] = tmpd[c.rowidx[ap] - 1] +
                                                                 c.d[ap] * bc;
                                        currRowIdx = c.rowidx[ap + 1] - 1;
                                        tmpd[currRowIdx] = tmpd[currRowIdx] + c.d[ap + 1] *
                                                                                  bc;
                                        currRowIdx = c.rowidx[ap + 2] - 1;
                                        tmpd[currRowIdx] = tmpd[currRowIdx] + c.d[ap + 2] *
                                                                                  bc;
                                    }

                                    nx = c.colidx[acol + 1] - 1;
                                    for (int ap{ridx}; ap <= nx; ap++) {
                                        currRowIdx = c.rowidx[ap - 1] - 1;
                                        tmpd[currRowIdx] = tmpd[currRowIdx] + c.d[ap - 1] *
                                                                                  bc;
                                    }
                                } else {
                                    nx--;
                                    for (int ap{b_c}; ap <= nx; ap++) {
                                        currRowIdx = c.rowidx[ap - 1] - 1;
                                        tmpd[currRowIdx] = tmpd[currRowIdx] + c.d[ap - 1] *
                                                                                  bc;
                                    }
                                }
                            }
                        }

                        if (grad.size(0) == tmpd.size(0)) {
                            b_stepDL_ = 0.0;
                            nx = stepDL_.size(0);
                            for (i = 0; i < nx; i++) {
                                b_stepDL_ += -stepDL_[i] * (grad[i] + tmpd[i]);
                            }

                            val = d1 / b_stepDL_;
                        } else {
                            val = b_binary_expand_op(err, costTrial, stepDL_, grad,
                                                     tmpd);
                        }

                        guard1 = false;
                        if (val > 0.0) {
                            xn.set_size(x.size(0), 3);
                            nx = x.size(0) * x.size(1) - 1;
                            for (i = 0; i <= nx; i++) {
                                xn[i] = x[i];
                            }

                            incrementX(xn, stepDL_, x);
                            *solutionInfo_Iterations = static_cast<double>(b_i) +
                                                       1.0;
                            err = costTrial;
                            grad.set_size(gradTrial.size(0));
                            nx = gradTrial.size(0);
                            for (i = 0; i < nx; i++) {
                                grad[i] = gradTrial[i];
                            }

                            *hess = HessianTrial;
                            computeBasicSteps(gradTrial, &HessianTrial, stepSD,
                                              stepGN, &localMin);
                            if (localMin) {
                                exitFlag = NLPSolverExitFlags::LocalMinimumFound;
                                terminated = true;
                                exitg1 = true;
                            } else {
                                guard1 = true;
                            }
                        } else {
                            guard1 = true;
                        }

                        if (guard1) {
                            bool b_guard1{false};

                            localMin = false;
                            b_guard1 = false;
                            if (val > 0.75) {
                                d1 = b_norm(stepDL_);
                                if (d1 > 0.9 * delta) {
                                    delta = 2.0 * d1;
                                } else {
                                    b_guard1 = true;
                                }
                            } else {
                                b_guard1 = true;
                            }

                            if (b_guard1 && (val < 0.25)) {
                                delta /= 4.0;
                                if (delta < TrustRegionRadiusTolerance) {
                                    localMin = true;
                                }
                            }

                            if (localMin) {
                                exitFlag = NLPSolverExitFlags::
                                    TrustRegionRadiusBelowMinimum;
                                terminated = true;
                                exitg1 = true;
                            } else {
                                b_i++;
                            }
                        }
                    }
                }
            }
        }

        b_xSol = &(&iobj_0[0])[1];
        (&iobj_0[0])[1].Matrix.set_size(x.size(0), 3);
        nx = x.size(0) * 3;
        for (i = 0; i < nx; i++) {
            (&iobj_0[0])[1].Matrix[i] = x[i];
        }

        (&iobj_0[0])[1].BlockSize[0] = 3.0;
        (&iobj_0[0])[1].BlockSize[1] = 3.0;
        (&iobj_0[0])[1].NumRowBlocks = static_cast<double>(x.size(0)) /
                                       3.0;
        (&iobj_0[0])[1].NumColBlocks = 1.0;
        if (!terminated) {
            *solutionInfo_Iterations = d;
        }
    }

    *xSol = b_xSol;
    *solutionInfo_Error = err;
    *solutionInfo_ExitFlag = static_cast<double>(exitFlag);
}

void SEHelpers::veelogmSE3(const double T[16], double vec[6]) {
    creal_T u;
    creal_T v;
    double Vinv[9];
    double b_I[9];
    double wx[9];
    double wv[3];
    double a;
    double theta;
    double thetaSq;
    int wx_tmp;
    bool guard1{false};

    bool y;
    u.re = 0.5 * (((T[0] + T[5]) + T[10]) - 1.0);
    if (!(std::abs(u.re) > 1.0)) {
        a = u.re;
        u.re = std::acos(a);
    } else {
        v.re = u.re + 1.0;
        v.im = 0.0;
        ::buildMapping::coder::internal::scalar::b_sqrt(&v);
        a = u.re;
        u.re = 1.0 - a;
        u.im = 0.0;
        ::buildMapping::coder::internal::scalar::b_sqrt(&u);
        a = u.re;
        u.re = 2.0 * rt_atan2d_snf(a, v.re);
    }

    a = u.re / std::sin(u.re);
    for (int i{0}; i < 3; i++) {
        wx_tmp = i << 2;
        wx[3 * i] = T[wx_tmp] - T[i];
        wx[3 * i + 1] = T[wx_tmp + 1] - T[i + 4];
        wx[3 * i + 2] = T[wx_tmp + 2] - T[i + 8];
    }

    wv[0] = wx[5];
    wv[1] = wx[6];
    wv[2] = wx[1];
    guard1 = false;
    if ((!std::isinf(a)) && (!std::isnan(a))) {
        bool exitg1;
        y = true;
        wx_tmp = 0;
        exitg1 = false;
        while ((!exitg1) && (wx_tmp <= 2)) {
            if (!(wv[wx_tmp] == 0.0)) {
                y = false;
                exitg1 = true;
            } else {
                wx_tmp++;
            }
        }

        if (!y) {
            wv[0] = wx[5] * a / 2.0;
            wv[1] = wx[6] * a / 2.0;
            wv[2] = wx[1] * a / 2.0;
        } else {
            guard1 = true;
        }
    } else {
        guard1 = true;
    }

    if (guard1) {
        std::memset(&b_I[0], 0, 9U * sizeof(double));
        b_I[0] = 1.0;
        b_I[4] = 1.0;
        b_I[8] = 1.0;
        for (int i{0}; i < 3; i++) {
            int I_tmp;
            wx_tmp = i << 2;
            b_I[3 * i] -= T[wx_tmp];
            I_tmp = 3 * i + 1;
            b_I[I_tmp] -= T[wx_tmp + 1];
            I_tmp = 3 * i + 2;
            b_I[I_tmp] -= T[wx_tmp + 2];
        }

        y = true;
        for (wx_tmp = 0; wx_tmp < 9; wx_tmp++) {
            if (y) {
                a = b_I[wx_tmp];
                if (std::isinf(a) || std::isnan(a)) {
                    y = false;
                }
            } else {
                y = false;
            }
        }

        if (y) {
            ::buildMapping::coder::internal::b_svd(b_I, Vinv, wv, wx);
        } else {
            for (int i{0}; i < 9; i++) {
                wx[i] = rtNaN;
            }
        }

        a = 1.0 / std::sqrt((wx[6] * wx[6] + wx[7] * wx[7]) + wx[8] * wx[8]);
        wv[0] = wx[6] * a * u.re;
        wv[1] = wx[7] * a * u.re;
        wv[2] = wx[8] * a * u.re;
    }

    theta = std::sqrt((wv[0] * wv[0] + wv[1] * wv[1]) + wv[2] * wv[2]);
    thetaSq = theta * theta;
    a = (1.0 - std::cos(theta)) / thetaSq;
    if ((a < 2.2204460492503131E-16) || (std::isinf(a) || std::isnan(a))) {
        std::memset(&Vinv[0], 0, 9U * sizeof(double));
        Vinv[0] = 1.0;
        Vinv[4] = 1.0;
        Vinv[8] = 1.0;
    } else {
        wx[0] = 0.0;
        wx[3] = -wv[2];
        wx[6] = wv[1];
        wx[1] = wv[2];
        wx[4] = 0.0;
        wx[7] = -wv[0];
        wx[2] = -wv[1];
        wx[5] = wv[0];
        wx[8] = 0.0;
        a = 1.0 / thetaSq * (1.0 - std::sin(theta) / theta / (2.0 * a));
        std::memset(&b_I[0], 0, 9U * sizeof(double));
        for (wx_tmp = 0; wx_tmp < 3; wx_tmp++) {
            b_I[wx_tmp + 3 * wx_tmp] = 1.0;
            for (int i{0}; i < 3; i++) {
                Vinv[wx_tmp + 3 * i] = (wx[wx_tmp] * wx[3 * i] + wx[wx_tmp + 3] * wx[3 * i + 1]) + wx[wx_tmp + 6] * wx[3 * i + 2];
            }
        }

        for (int i{0}; i < 9; i++) {
            Vinv[i] = (b_I[i] - 0.5 * wx[i]) + a * Vinv[i];
        }
    }

    a = T[12];
    theta = T[13];
    thetaSq = T[14];
    for (int i{0}; i < 3; i++) {
        vec[i] = (Vinv[i] * a + Vinv[i + 3] * theta) + Vinv[i + 6] *
                                                           thetaSq;
        vec[i + 3] = wv[i];
    }
}
}  // namespace internal
}  // namespace core
}  // namespace robotics

namespace vision {
namespace internal {
void FeaturePointsImpl::selectPoints(const ::coder::array<float, 2U>
                                         &points,
                                     const ::coder::array<float, 1U> &metric, double numPoints, ::coder::array<bool, 1U> &pointsIdx) {
    unsigned int binIdx_data[972];
    short tmp_data[972];
    if (numPoints == 1.0) {
        int i;
        int idx;
        int last;
        last = metric.size(0);
        if (metric.size(0) <= 2) {
            if (metric.size(0) == 1) {
                idx = 1;
            } else if ((metric[0] < metric[metric.size(0) - 1]) || (std::isnan(metric[0]) && (!std::isnan(metric[metric.size(0) - 1])))) {
                idx = metric.size(0);
            } else {
                idx = 1;
            }
        } else {
            int loop_ub_tmp;
            if (!std::isnan(metric[0])) {
                idx = 1;
            } else {
                bool exitg1;
                idx = 0;
                loop_ub_tmp = 2;
                exitg1 = false;
                while ((!exitg1) && (loop_ub_tmp <= last)) {
                    if (!std::isnan(metric[loop_ub_tmp - 1])) {
                        idx = loop_ub_tmp;
                        exitg1 = true;
                    } else {
                        loop_ub_tmp++;
                    }
                }
            }

            if (idx == 0) {
                idx = 1;
            } else {
                float whichBin_idx_0;
                whichBin_idx_0 = metric[idx - 1];
                i = idx + 1;
                for (loop_ub_tmp = i; loop_ub_tmp <= last; loop_ub_tmp++) {
                    float f;
                    f = metric[loop_ub_tmp - 1];
                    if (whichBin_idx_0 < f) {
                        whichBin_idx_0 = f;
                        idx = loop_ub_tmp;
                    }
                }
            }
        }

        pointsIdx.set_size(points.size(0));
        last = points.size(0);
        for (i = 0; i < last; i++) {
            pointsIdx[i] = false;
        }

        pointsIdx[idx - 1] = true;
    } else {
        double d;
        double gridStep_idx_0;
        double gridStep_idx_1;
        double h;
        int b_tmp_data[972];
        int i;
        int idx;
        int last;
        int loop_ub_tmp;
        h = std::fmax(std::floor(std::sqrt(numPoints / 1.3333333333333333)),
                      1.0);
        d = std::floor(h * 1.3333333333333333);
        gridStep_idx_0 = 640.0 / (d + 1.0);
        gridStep_idx_1 = 480.0 / (h + 1.0);
        loop_ub_tmp = static_cast<int>(d) * static_cast<int>(h);
        if (loop_ub_tmp - 1 >= 0) {
            std::memset(&binIdx_data[0], 0, static_cast<unsigned int>(loop_ub_tmp) * sizeof(unsigned int));
        }

        i = points.size(0);
        for (int b_i{0}; b_i < i; b_i++) {
            float f;
            float whichBin_idx_0;
            float whichBin_idx_1;
            bool p;
            f = std::floor(points[b_i] / static_cast<float>(gridStep_idx_0));
            whichBin_idx_0 = f + 1.0F;
            p = (std::isnan(f + 1.0F) || (f + 1.0F > static_cast<float>(d)));
            if (p) {
                whichBin_idx_0 = static_cast<float>(d);
            }

            f = std::floor(points[b_i + points.size(0)] / static_cast<float>(gridStep_idx_1));
            whichBin_idx_1 = f + 1.0F;
            p = (std::isnan(f + 1.0F) || (f + 1.0F > static_cast<float>(h)));
            if (p) {
                whichBin_idx_1 = static_cast<float>(h);
            }

            last = (static_cast<int>(whichBin_idx_0) + static_cast<int>(d) * (static_cast<int>(whichBin_idx_1) - 1)) - 1;
            idx = static_cast<int>(binIdx_data[last]);
            if ((idx < 1) || (metric[idx - 1] < metric[b_i])) {
                binIdx_data[last] = static_cast<unsigned int>(b_i + 1);
            }
        }

        last = loop_ub_tmp - 1;
        idx = 0;
        loop_ub_tmp = 0;
        for (int b_i{0}; b_i <= last; b_i++) {
            if (static_cast<int>(binIdx_data[b_i]) > 0) {
                idx++;
                tmp_data[loop_ub_tmp] = static_cast<short>(b_i + 1);
                loop_ub_tmp++;
            }
        }

        pointsIdx.set_size(points.size(0));
        last = points.size(0);
        for (i = 0; i < last; i++) {
            pointsIdx[i] = false;
        }

        for (i = 0; i < idx; i++) {
            b_tmp_data[i] = static_cast<int>(binIdx_data[tmp_data[i] - 1]);
        }

        for (i = 0; i < idx; i++) {
            pointsIdx[b_tmp_data[i] - 1] = true;
        }
    }
}
}  // namespace internal
}  // namespace vision

namespace visioncodegen {
void b_AlphaBlender::matlabCodegenDestructor() {
    if (!matlabCodegenIsDeleted) {
        matlabCodegenIsDeleted = true;
        if (isInitialized == 1) {
            isInitialized = 2;
        }
    }
}
}  // namespace visioncodegen
}  // namespace coder

constructWorldMapStackData *HDMapping::getStackData() {
    return &SD_;
}

static double b_binary_expand_op(double in1, double in2, const ::coder::array<double, 1U> &in3, const ::coder::array<double, 1U> &in4, const ::coder::array<double, 1U> &in5) {
    ::coder::array<double, 1U> b_in4;
    double b_in3;
    int i;
    int loop_ub;
    int stride_0_0;
    int stride_1_0;
    if (in5.size(0) == 1) {
        i = in4.size(0);
    } else {
        i = in5.size(0);
    }

    b_in4.set_size(i);
    stride_0_0 = (in4.size(0) != 1);
    stride_1_0 = (in5.size(0) != 1);
    if (in5.size(0) == 1) {
        loop_ub = in4.size(0);
    } else {
        loop_ub = in5.size(0);
    }

    for (i = 0; i < loop_ub; i++) {
        b_in4[i] = in4[i * stride_0_0] + in5[i * stride_1_0];
    }

    b_in3 = 0.0;
    loop_ub = in3.size(0);
    for (i = 0; i < loop_ub; i++) {
        b_in3 += -in3[i] * b_in4[i];
    }

    return (in1 - in2) / b_in3;
}

static void b_binary_expand_op(::coder::array<double, 1U> &in1, const ::coder::array<double, 1U> &in2, double in3) {
    ::coder::array<double, 1U> b_in2;
    int i;
    int loop_ub;
    int stride_0_0;
    int stride_1_0;
    if (in1.size(0) == 1) {
        i = in2.size(0);
    } else {
        i = in1.size(0);
    }

    b_in2.set_size(i);
    stride_0_0 = (in2.size(0) != 1);
    stride_1_0 = (in1.size(0) != 1);
    if (in1.size(0) == 1) {
        loop_ub = in2.size(0);
    } else {
        loop_ub = in1.size(0);
    }

    for (i = 0; i < loop_ub; i++) {
        b_in2[i] = in2[i * stride_0_0] + in3 * in1[i * stride_1_0];
    }

    in1.set_size(b_in2.size(0));
    loop_ub = b_in2.size(0);
    for (i = 0; i < loop_ub; i++) {
        in1[i] = b_in2[i];
    }
}

static void b_binary_expand_op(::coder::array<double, 2U> &in1, const ::coder::array<double, 2U> &in2, const ::coder::array<double, 3U> &in3) {
    ::coder::array<double, 2U> b_in1;
    int i;
    int in2_idx_0;
    int stride_0_0;
    int stride_1_0;
    in2_idx_0 = in2.size(0);
    in1.set_size(in2_idx_0, 2);
    for (i = 0; i < in2_idx_0; i++) {
        in1[i] = in2[i] / in2[i + in2.size(0) * 2];
    }

    in2_idx_0 = in2.size(0);
    for (i = 0; i < in2_idx_0; i++) {
        in1[i + in1.size(0)] = in2[i + in2.size(0)] / in2[i + in2.size(0) * 2];
    }

    if (in3.size(0) == 1) {
        i = in1.size(0);
    } else {
        i = in3.size(0);
    }

    b_in1.set_size(i, 2);
    stride_0_0 = (in1.size(0) != 1);
    stride_1_0 = (in3.size(0) != 1);
    if (in3.size(0) == 1) {
        in2_idx_0 = in1.size(0);
    } else {
        in2_idx_0 = in3.size(0);
    }

    for (i = 0; i < 2; i++) {
        for (int i1{0}; i1 < in2_idx_0; i1++) {
            b_in1[i1 + b_in1.size(0) * i] = in1[i1 * stride_0_0 + in1.size(0) * i] -
                                            in3[(i1 * stride_1_0 + in3.size(0) * i) + in3.size(0) * 2];
        }
    }

    in1.set_size(b_in1.size(0), 2);
    in2_idx_0 = b_in1.size(0);
    for (i = 0; i < 2; i++) {
        for (int i1{0}; i1 < in2_idx_0; i1++) {
            in1[i1 + in1.size(0) * i] = b_in1[i1 + b_in1.size(0) * i];
        }
    }
}

static void b_binary_expand_op(double in1[3], const ::coder::array<double, 2U> &in2, const ::coder::array<double, 1U> &in3, int in4, int in5) {
    int stride_0_0;
    in1[0] = in2[in2.size(0) * 2];
    in1[1] = in2[in2.size(0) * 2 + 1];
    in1[2] = rt_atan2d_snf(in2[1], in2[0]);
    stride_0_0 = ((in5 - in4) + 1 != 1);
    in1[0] += in3[in4];
    in1[1] += in3[in4 + stride_0_0];
    in1[2] += in3[in4 + (stride_0_0 << 1)];
}

static void b_binary_expand_op(::coder::array<double, 2U> &in1, const ::coder::array<double, 1U> &in2, int in3, int in4) {
    in1[in1.size(0) * 2] = in1[in1.size(0) * 2] + in2[in3];
    in1[in1.size(0) * 2 + 1] = in1[in1.size(0) * 2 + 1] + in2[in3 + ((in4 - in3) + 1 != 1)];
}

static void b_binary_expand_op(coder::nav::algs::internal::BlockInserter2 *in1,
                               int in2, int in4, int in5, const double in6_data[], const int *in6_size) {
    ::coder::array<double, 1U> b_in1;
    int stride_0_0;
    b_in1.set_size(*in6_size);
    stride_0_0 = ((in5 - in4) + 1 != 1);
    for (int i{0}; i < *in6_size; i++) {
        b_in1[i] = in1->Gradient[in4 + i * stride_0_0] + in6_data[i];
    }

    stride_0_0 = b_in1.size(0);
    for (int i{0}; i < stride_0_0; i++) {
        in1->Gradient[in2 + i] = b_in1[i];
    }
}

static void binary_expand_op(::coder::array<bool, 2U> &in1, const ::coder::array<int, 2U> &in2, const ::coder::array<unsigned int, 2U> &in3) {
    int i;
    int loop_ub;
    int stride_0_1;
    int stride_1_1;
    if (in3.size(1) == 1) {
        i = in2.size(1);
    } else {
        i = in3.size(1);
    }

    in1.set_size(1, i);
    stride_0_1 = (in2.size(1) != 1);
    stride_1_1 = (in3.size(1) != 1);
    if (in3.size(1) == 1) {
        loop_ub = in2.size(1);
    } else {
        loop_ub = in3.size(1);
    }

    for (i = 0; i < loop_ub; i++) {
        in1[i] = (static_cast<unsigned int>(in2[i * stride_0_1]) == in3[2 * (i *
                                                                             stride_1_1)]);
    }
}

static void blendImage_free(HDMapping *aInstancePtr) {
    constructWorldMapStackData *localSD;
    localSD = aInstancePtr->getStackData();
    if (!localSD->pd->alphablend.matlabCodegenIsDeleted) {
        localSD->pd->alphablend.matlabCodegenIsDeleted = true;
        if (localSD->pd->alphablend.isInitialized == 1) {
            localSD->pd->alphablend.isInitialized = 2;
        }
    }
}

static void blendImage_init(HDMapping *aInstancePtr) {
    constructWorldMapStackData *localSD;
    localSD = aInstancePtr->getStackData();
    localSD->pd->alphablend_not_empty = false;
    localSD->pd->alphablend.matlabCodegenIsDeleted = true;
}

namespace buildMapFunctions {
static void b_fuseOptimizeHDMap(HDMapping *aInstancePtr, const ::coder::array<cell_wrap_0, 1U> &imageFiles, const ::coder::array<double, 2U> &updateNodeVehiclePtPoses, const coder::rigidtform2d initViclePtPose,
                                const bool BW[307200], ::coder::array<unsigned char, 2U> &bigImgSt_bigImg,
                                double bigImgSt_ref_XWorldLimits[2], double bigImgSt_ref_YWorldLimits[2],
                                double bigImgSt_ref_ImageSize[2]) {
    static const double a[4]{1.0, 0.0, 0.0, 1.0};

    coder::imref2d outRef;
    coder::visioncodegen::b_AlphaBlender blender;
    ::coder::array<coder::rigidtform2d, 1U> tforms;
    ::coder::array<char, 2U> imgPath;
    ::coder::array<unsigned char, 2U> outImg;
    ::coder::array<char, 2U> varargin_1;
    ::coder::array<bool, 2U> outBwImg;
    constructWorldMapStackData *localSD;
    double B[9];
    double r1[9];
    double Rc[4];
    double b_d;
    double b_r2;
    double b_wpr;
    double d;
    double r;
    double r2;
    double wpr;
    int i;
    int loop_ub;
    bool close_enough;
    localSD = aInstancePtr->getStackData();
    bigImgSt_ref_XWorldLimits[0] = 0.5;
    bigImgSt_ref_YWorldLimits[0] = 0.5;
    bigImgSt_ref_XWorldLimits[1] = 640.5;
    bigImgSt_ref_YWorldLimits[1] = 480.5;
    i = imageFiles.size(0);
    tforms.set_size(imageFiles.size(0));
    coder::images::geotrans::internal::constrainToRotationMatrix2D(a, Rc, &d);
    if (std::isnan(d + 180.0) || std::isinf(d + 180.0)) {
        r = rtNaN;
    } else if (d + 180.0 == 0.0) {
        r = 0.0;
    } else {
        r = std::fmod(d + 180.0, 360.0);
        if (r == 0.0) {
            r = 0.0;
        } else if (d + 180.0 < 0.0) {
            r += 360.0;
        }
    }

    wpr = std::round(r - 180.0);
    if (r - 180.0 == wpr) {
        close_enough = true;
    } else {
        d = std::abs((r - 180.0) - wpr);
        if ((r - 180.0 == 0.0) || (wpr == 0.0)) {
            close_enough = (d < 4.94065645841247E-324);
        } else {
            b_d = std::abs(r - 180.0) + std::abs(wpr);
            if (b_d < 2.2250738585072014E-308) {
                close_enough = (d < 4.94065645841247E-324);
            } else {
                close_enough = (d / std::fmin(b_d, 1.7976931348623157E+308) <
                                2.2204460492503131E-16);
            }
        }
    }

    for (int b_i{0}; b_i < i; b_i++) {
        coder::rigidtform2d imageTargetPose;
        double beta[9];
        double d1;
        bool b_close_enough;
        d = 57.295779513082323 * updateNodeVehiclePtPoses[b_i +
                                                          updateNodeVehiclePtPoses.size(0) * 2] +
            180.0;
        if (std::isnan(d) || std::isinf(d)) {
            r2 = rtNaN;
        } else if (d == 0.0) {
            r2 = 0.0;
        } else {
            r2 = std::fmod(d, 360.0);
            if (r2 == 0.0) {
                r2 = 0.0;
            } else if (d < 0.0) {
                r2 += 360.0;
            }
        }

        b_wpr = std::round(r2 - 180.0);
        if (r2 - 180.0 == b_wpr) {
            b_close_enough = true;
        } else {
            d = std::abs((r2 - 180.0) - b_wpr);
            if ((r2 - 180.0 == 0.0) || (b_wpr == 0.0)) {
                b_close_enough = (d < 4.94065645841247E-324);
            } else {
                b_d = std::abs(r2 - 180.0) + std::abs(b_wpr);
                if (b_d < 2.2250738585072014E-308) {
                    b_close_enough = (d < 4.94065645841247E-324);
                } else {
                    b_close_enough = (d / std::fmin(b_d, 1.7976931348623157E+308) <
                                      2.2204460492503131E-16);
                }
            }
        }

        b_r2 = r2 - 180.0;
        if (b_close_enough) {
            b_r2 = b_wpr;
        }

        r2 = r - 180.0;
        if (close_enough) {
            r2 = wpr;
        }

        b_wpr = b_r2;
        coder::b_cosd(&b_wpr);
        coder::b_sind(&b_r2);
        d = r2;
        coder::b_cosd(&d);
        coder::b_sind(&r2);
        B[0] = b_wpr;
        B[3] = -b_r2;
        B[6] = updateNodeVehiclePtPoses[b_i];
        B[1] = b_r2;
        B[4] = b_wpr;
        B[7] = updateNodeVehiclePtPoses[b_i + updateNodeVehiclePtPoses.size(0)];
        r1[0] = d;
        r1[3] = -r2;
        r1[6] = (0.0 - initViclePtPose.Translation[0]) + 0.0 * (0.0 -
                                                                initViclePtPose.Translation[1]);
        r1[1] = r2;
        r1[4] = d;
        r1[7] = 0.0 * (0.0 - initViclePtPose.Translation[0]) + (0.0 -
                                                                initViclePtPose.Translation[1]);
        B[2] = 0.0;
        r1[2] = 0.0;
        B[5] = 0.0;
        r1[5] = 0.0;
        B[8] = 1.0;
        r1[8] = 1.0;
        for (int i1{0}; i1 < 3; i1++) {
            b_d = B[i1];
            d1 = B[i1 + 3];
            d = B[i1 + 6];
            for (loop_ub = 0; loop_ub < 3; loop_ub++) {
                beta[i1 + 3 * loop_ub] = (b_d * r1[3 * loop_ub] + d1 * r1[3 *
                                                                              loop_ub +
                                                                          1]) +
                                         d * r1[3 * loop_ub + 2];
            }
        }

        double b_beta[4];
        b_beta[0] = beta[0];
        b_beta[1] = beta[1];
        b_beta[2] = beta[3];
        b_beta[3] = beta[4];
        coder::images::geotrans::internal::constrainToRotationMatrix2D(b_beta,
                                                                       Rc, &d);
        if (std::isnan(d + 180.0) || std::isinf(d + 180.0)) {
            r2 = rtNaN;
        } else if (d + 180.0 == 0.0) {
            r2 = 0.0;
        } else {
            r2 = std::fmod(d + 180.0, 360.0);
            if (r2 == 0.0) {
                r2 = 0.0;
            } else if (d + 180.0 < 0.0) {
                r2 += 360.0;
            }
        }

        b_wpr = std::round(r2 - 180.0);
        if (r2 - 180.0 == b_wpr) {
            b_close_enough = true;
        } else {
            d = std::abs((r2 - 180.0) - b_wpr);
            if ((r2 - 180.0 == 0.0) || (b_wpr == 0.0)) {
                b_close_enough = (d < 4.94065645841247E-324);
            } else {
                b_d = std::abs(r2 - 180.0) + std::abs(b_wpr);
                if (b_d < 2.2250738585072014E-308) {
                    b_close_enough = (d < 4.94065645841247E-324);
                } else {
                    b_close_enough = (d / std::fmin(b_d, 1.7976931348623157E+308) <
                                      2.2204460492503131E-16);
                }
            }
        }

        b_r2 = r2 - 180.0;
        if (b_close_enough) {
            b_r2 = b_wpr;
        }

        imageTargetPose.RotationAngle = b_r2;
        imageTargetPose.Translation[0] = beta[6];
        imageTargetPose.Translation[1] = beta[7];
        tforms[b_i] = imageTargetPose;
        d = b_r2;
        coder::b_cosd(&d);
        coder::b_sind(&b_r2);
        r2 = beta[6];
        b_wpr = beta[7];
        for (int i1{0}; i1 < 9; i1++) {
            b_d = dv[i1];
            d1 = dv1[i1];
            r1[i1] = (d * b_d + -b_r2 * d1) + r2;
            B[i1] = (b_r2 * b_d + d * d1) + b_wpr;
        }

        d = bigImgSt_ref_XWorldLimits[1];
        bigImgSt_ref_XWorldLimits[0] = std::fmin(bigImgSt_ref_XWorldLimits[0],
                                                 coder::internal::minimum(r1));
        bigImgSt_ref_XWorldLimits[1] = std::fmax(d, coder::internal::maximum(r1));
        d = bigImgSt_ref_YWorldLimits[1];
        bigImgSt_ref_YWorldLimits[0] = std::fmin(bigImgSt_ref_YWorldLimits[0],
                                                 coder::internal::minimum(B));
        bigImgSt_ref_YWorldLimits[1] = std::fmax(d, coder::internal::maximum(B));
    }

    d = std::round(bigImgSt_ref_YWorldLimits[1] - bigImgSt_ref_YWorldLimits[0]);
    b_wpr = std::round(bigImgSt_ref_XWorldLimits[1] -
                       bigImgSt_ref_XWorldLimits[0]);
    outRef.ImageSizeAlias[0] = d;
    outRef.ImageSizeAlias[1] = b_wpr;
    outRef.XWorldLimits[0] = bigImgSt_ref_XWorldLimits[0];
    outRef.YWorldLimits[0] = bigImgSt_ref_YWorldLimits[0];
    outRef.XWorldLimits[1] = bigImgSt_ref_XWorldLimits[1];
    outRef.YWorldLimits[1] = bigImgSt_ref_YWorldLimits[1];
    outRef.ForcePixelExtentToOne = false;
    blender.isInitialized = 0;
    blender.cSFunObject.P0_Coordinates[0] = 1;
    blender.cSFunObject.P0_Coordinates[1] = 1;
    blender.matlabCodegenIsDeleted = false;
    bigImgSt_bigImg.set_size(static_cast<int>(d), static_cast<int>(b_wpr));
    loop_ub = static_cast<int>(d) * static_cast<int>(b_wpr);
    for (i = 0; i < loop_ub; i++) {
        bigImgSt_bigImg[i] = 0U;
    }

    i = imageFiles.size(0);
    for (int b_i{0}; b_i < i; b_i++) {
        loop_ub = imageFiles[b_i].f1.size(1);
        imgPath.set_size(1, imageFiles[b_i].f1.size(1) + 1);
        for (int i1{0}; i1 < loop_ub; i1++) {
            imgPath[i1] = imageFiles[b_i].f1[i1];
        }

        imgPath[imageFiles[b_i].f1.size(1)] = '\x00';
        varargin_1.set_size(1, imgPath.size(1) + 1);
        loop_ub = imgPath.size(1);
        for (int i1{0}; i1 < loop_ub; i1++) {
            varargin_1[i1] = imgPath[i1];
        }

        varargin_1[imgPath.size(1)] = '\x00';
        printf("read current image name:%s.\n", &varargin_1[0]);
        fflush(stdout);
        imreadOpenCV(&imgPath[0], &localSD->f3.currImg[0]);
        coder::images::internal::coder::optimized::b_remapAndResampleGeneric2d(aInstancePtr, localSD->f3.currImg, tforms[b_i], outRef, outImg);
        coder::images::internal::coder::optimized::c_remapAndResampleGeneric2d(aInstancePtr, BW, tforms[b_i], outRef, outBwImg);
        if (b_i + 1 == 1) {
            for (loop_ub = 0; loop_ub < 307200; loop_ub++) {
                close_enough = BW[loop_ub];
                localSD->f3.temp[loop_ub] = close_enough;
                if (!close_enough) {
                    localSD->f3.temp[loop_ub] = true;
                }
            }

            coder::images::internal::coder::optimized::c_remapAndResampleGeneric2d(aInstancePtr, localSD->f3.temp, tforms[0], outRef, outBwImg);
        }

        if (blender.isInitialized != 1) {
            blender.isInitialized = 1;
            blender.isSetupComplete = true;
        }

        coder::Outputs(&blender.cSFunObject, bigImgSt_bigImg, outImg, outBwImg);
    }

    bigImgSt_ref_ImageSize[0] = d;
    bigImgSt_ref_ImageSize[1] = b_wpr;
}

static void b_helperDetectAndExtractFeatures(HDMapping *aInstancePtr, const unsigned char Irgb[307200], coder::binaryFeatures *features, ::coder::array<float, 1U> &featureMetrics, ::coder::array<float, 2U> &locations) {
    void *pKeypoints;
    coder::ORBPoints b_points;
    coder::ORBPoints points;
    ::coder::array<float, 2U> b_location;
    ::coder::array<float, 2U> inputs_Location;
    ::coder::array<float, 2U> location;
    ::coder::array<float, 1U> inputs_Orientation;
    ::coder::array<float, 1U> metric;
    ::coder::array<float, 1U> orientation;
    ::coder::array<float, 1U> scale;
    ::coder::array<unsigned int, 2U> b_origIdx;
    ::coder::array<unsigned int, 2U> origIdx;
    ::coder::array<int, 1U> b_r;
    ::coder::array<int, 1U> r1;
    ::coder::array<int, 1U> r2;
    ::coder::array<int, 1U> r3;
    ::coder::array<bool, 1U> idx;
    constructWorldMapStackData *localSD;
    float valLocation_data[2000];
    float valMetric_data[1000];
    float valOrientation_data[1000];
    float valScale_data[1000];
    int NN;
    int i;
    int loop_ub;
    int numPtsOut;
    int partialTrueCount;
    unsigned int r;
    localSD = aInstancePtr->getStackData();
    r = 5489U;
    localSD->pd->state[0] = 5489U;
    for (numPtsOut = 0; numPtsOut < 623; numPtsOut++) {
        r = ((r ^ r >> 30U) * 1812433253U + static_cast<unsigned int>(numPtsOut)) + 1U;
        localSD->pd->state[numPtsOut + 1] = r;
    }

    localSD->pd->state[624] = 624U;
    std::copy(&Irgb[0], &Irgb[307200], &localSD->f2.Iu8[0]);
    pKeypoints = NULL;
    numPtsOut = detectORBComputeCM(&localSD->f2.Iu8[0], 480, 640, 307200, 1.2F,
                                   8, 31, 0, 2, 0, 31, 20, &pKeypoints);
    location.set_size(numPtsOut, 2);
    metric.set_size(numPtsOut);
    scale.set_size(numPtsOut);
    orientation.set_size(numPtsOut);
    detectORBAssignOutputCM(pKeypoints, &location[0], &(orientation.data())[0],
                            &(metric.data())[0], &(scale.data())[0]);
    loop_ub = scale.size(0);
    for (partialTrueCount = 0; partialTrueCount < loop_ub; partialTrueCount++) {
        scale[partialTrueCount] = scale[partialTrueCount] / 31.0F;
    }

    points.pLocation.set_size(0, 2);
    points.pMetric.set_size(0);
    points.pScale.set_size(0);
    points.pOrientation.set_size(0);
    points.configure(location, metric, scale, orientation);
    location.set_size(points.pLocation.size(0), 2);
    loop_ub = points.pLocation.size(0) * 2;
    for (partialTrueCount = 0; partialTrueCount < loop_ub; partialTrueCount++) {
        location[partialTrueCount] = points.pLocation[partialTrueCount];
    }

    metric.set_size(points.pMetric.size(0));
    loop_ub = points.pMetric.size(0);
    for (partialTrueCount = 0; partialTrueCount < loop_ub; partialTrueCount++) {
        metric[partialTrueCount] = points.pMetric[partialTrueCount];
    }

    if (points.pLocation.size(0) < 1) {
        origIdx.set_size(1, 0);
    } else {
        origIdx.set_size(1, points.pLocation.size(0));
        loop_ub = points.pLocation.size(0) - 1;
        for (partialTrueCount = 0; partialTrueCount <= loop_ub; partialTrueCount++) {
            origIdx[partialTrueCount] = static_cast<unsigned int>(partialTrueCount) + 1U;
        }
    }

    scale.set_size(points.pMetric.size(0));
    NN = points.pMetric.size(0);
    if (NN > 1000) {
        NN = 1000;
    }

    coder::vision::internal::FeaturePointsImpl::selectPoints(points.pLocation,
                                                             points.pMetric, static_cast<double>(NN), idx);
    numPtsOut = idx.size(0) - 1;
    loop_ub = 0;
    for (i = 0; i <= numPtsOut; i++) {
        if (idx[i]) {
            loop_ub++;
        }
    }

    b_r.set_size(loop_ub);
    partialTrueCount = 0;
    for (i = 0; i <= numPtsOut; i++) {
        if (idx[i]) {
            b_r[partialTrueCount] = i + 1;
            partialTrueCount++;
        }
    }

    if (b_r.size(0) < 1) {
        loop_ub = 0;
    } else {
        loop_ub = b_r.size(0);
    }

    for (partialTrueCount = 0; partialTrueCount < loop_ub; partialTrueCount++) {
        scale[partialTrueCount] = static_cast<float>(origIdx[b_r[partialTrueCount] - 1]);
    }

    for (r = static_cast<unsigned int>(b_r.size(0)) + 1U; r <= static_cast<
                                                                   unsigned int>(NN);
         r += static_cast<unsigned int>(r3.size(0))) {
        unsigned int u;
        numPtsOut = idx.size(0) - 1;
        loop_ub = 0;
        for (i = 0; i <= numPtsOut; i++) {
            if (!idx[i]) {
                loop_ub++;
            }
        }

        r1.set_size(loop_ub);
        partialTrueCount = 0;
        for (i = 0; i <= numPtsOut; i++) {
            if (!idx[i]) {
                r1[partialTrueCount] = i + 1;
                partialTrueCount++;
            }
        }

        b_origIdx.set_size(1, r1.size(0));
        loop_ub = r1.size(0);
        for (partialTrueCount = 0; partialTrueCount < loop_ub; partialTrueCount++) {
            b_origIdx[partialTrueCount] = origIdx[r1[partialTrueCount] - 1];
        }

        origIdx.set_size(1, b_origIdx.size(1));
        loop_ub = b_origIdx.size(1);
        for (partialTrueCount = 0; partialTrueCount < loop_ub; partialTrueCount++) {
            origIdx[partialTrueCount] = b_origIdx[partialTrueCount];
        }

        numPtsOut = idx.size(0) - 1;
        loop_ub = 0;
        for (i = 0; i <= numPtsOut; i++) {
            if (!idx[i]) {
                loop_ub++;
            }
        }

        r2.set_size(loop_ub);
        partialTrueCount = 0;
        for (i = 0; i <= numPtsOut; i++) {
            if (!idx[i]) {
                r2[partialTrueCount] = i + 1;
                partialTrueCount++;
            }
        }

        b_location.set_size(r2.size(0), 2);
        loop_ub = r2.size(0);
        for (partialTrueCount = 0; partialTrueCount < 2; partialTrueCount++) {
            for (i = 0; i < loop_ub; i++) {
                b_location[i + b_location.size(0) * partialTrueCount] = location
                    [(r2[i] + location.size(0) * partialTrueCount) - 1];
            }
        }

        location.set_size(b_location.size(0), 2);
        loop_ub = b_location.size(0) * 2;
        for (partialTrueCount = 0; partialTrueCount < loop_ub; partialTrueCount++) {
            location[partialTrueCount] = b_location[partialTrueCount];
        }

        numPtsOut = idx.size(0) - 1;
        loop_ub = 0;
        partialTrueCount = 0;
        for (i = 0; i <= numPtsOut; i++) {
            if (!idx[i]) {
                loop_ub++;
                metric[partialTrueCount] = metric[i];
                partialTrueCount++;
            }
        }

        metric.set_size(loop_ub);
        coder::vision::internal::FeaturePointsImpl::selectPoints(location,
                                                                 metric, static_cast<double>((NN - static_cast<int>(r)) + 1), idx);
        numPtsOut = idx.size(0) - 1;
        loop_ub = 0;
        for (i = 0; i <= numPtsOut; i++) {
            if (idx[i]) {
                loop_ub++;
            }
        }

        r3.set_size(loop_ub);
        partialTrueCount = 0;
        for (i = 0; i <= numPtsOut; i++) {
            if (idx[i]) {
                r3[partialTrueCount] = i + 1;
                partialTrueCount++;
            }
        }

        u = (r + static_cast<unsigned int>(r3.size(0))) - 1U;
        if (r > u) {
            partialTrueCount = 0;
            i = 0;
        } else {
            partialTrueCount = static_cast<int>(r) - 1;
            i = static_cast<int>(u);
        }

        loop_ub = i - partialTrueCount;
        for (i = 0; i < loop_ub; i++) {
            scale[partialTrueCount + i] = static_cast<float>(origIdx[r3[i] - 1]);
        }
    }

    if (NN < 1) {
        loop_ub = 0;
    } else {
        loop_ub = NN;
    }

    b_points.pLocation.set_size(0, 2);
    b_points.pMetric.set_size(0);
    b_points.pScale.set_size(0);
    b_points.pOrientation.set_size(0);
    inputs_Location.set_size(loop_ub, 2);
    for (partialTrueCount = 0; partialTrueCount < 2; partialTrueCount++) {
        for (i = 0; i < loop_ub; i++) {
            inputs_Location[i + inputs_Location.size(0) * partialTrueCount] =
                points.pLocation[(static_cast<int>(scale[i]) + points.pLocation.size(0) * partialTrueCount) - 1];
        }
    }

    metric.set_size(loop_ub);
    orientation.set_size(loop_ub);
    inputs_Orientation.set_size(loop_ub);
    for (partialTrueCount = 0; partialTrueCount < loop_ub; partialTrueCount++) {
        metric[partialTrueCount] = points.pMetric[static_cast<int>(scale[partialTrueCount]) - 1];
        orientation[partialTrueCount] = points.pScale[static_cast<int>(scale[partialTrueCount]) - 1];
        inputs_Orientation[partialTrueCount] = points.pOrientation[static_cast<
                                                                       int>(scale[partialTrueCount]) -
                                                                   1];
    }

    b_points.configure(inputs_Location, metric, orientation,
                       inputs_Orientation);
    std::copy(&Irgb[0], &Irgb[307200], &localSD->f2.Iu8[0]);
    numPtsOut = b_points.pLocation.size(0);
    loop_ub = b_points.pLocation.size(0);
    for (partialTrueCount = 0; partialTrueCount < 2; partialTrueCount++) {
        for (i = 0; i < loop_ub; i++) {
            valLocation_data[i + numPtsOut * partialTrueCount] =
                b_points.pLocation[i + b_points.pLocation.size(0) * partialTrueCount];
        }
    }

    loop_ub = b_points.pScale.size(0);
    for (partialTrueCount = 0; partialTrueCount < loop_ub; partialTrueCount++) {
        valScale_data[partialTrueCount] = b_points.pScale[partialTrueCount] *
                                          31.0F;
    }

    loop_ub = b_points.pMetric.size(0);
    for (partialTrueCount = 0; partialTrueCount < loop_ub; partialTrueCount++) {
        valMetric_data[partialTrueCount] = b_points.pMetric[partialTrueCount];
    }

    loop_ub = b_points.pOrientation.size(0);
    for (partialTrueCount = 0; partialTrueCount < loop_ub; partialTrueCount++) {
        valOrientation_data[partialTrueCount] =
            b_points.pOrientation[partialTrueCount] * 57.2957802F;
    }

    void *pFtrs;
    pKeypoints = NULL;
    pFtrs = NULL;
    numPtsOut = extractORBComputeCM(&localSD->f2.Iu8[0], 480, 640,
                                    &valLocation_data[0], &valOrientation_data[0], &valMetric_data[0],
                                    &valScale_data[0], numPtsOut, 307200, 1.2F, 8, 31, 0, 2, 0, 31, 20,
                                    &pKeypoints, &pFtrs);
    locations.set_size(numPtsOut, 2);
    scale.set_size(numPtsOut);
    orientation.set_size(numPtsOut);
    metric.set_size(numPtsOut);
    features->Features.set_size(numPtsOut, 32);
    extractORBAssignOutputCM(pKeypoints, pFtrs, &locations[0], &(metric.data())[0], &(scale.data())[0], &(orientation.data())[0], &features->Features[0]);
    featureMetrics.set_size(features->Features.size(0));
    loop_ub = features->Features.size(0);
    for (partialTrueCount = 0; partialTrueCount < loop_ub; partialTrueCount++) {
        featureMetrics[partialTrueCount] = 0.0F;
    }

    numPtsOut = features->Features.size(0);
    for (loop_ub = 0; loop_ub < numPtsOut; loop_ub++) {
        float xbar;
        float yv;
        unsigned char xv[32];
        for (partialTrueCount = 0; partialTrueCount < 32; partialTrueCount++) {
            xv[partialTrueCount] = features->Features[loop_ub + partialTrueCount *
                                                                    numPtsOut];
        }

        xbar = xv[0];
        for (partialTrueCount = 0; partialTrueCount < 31; partialTrueCount++) {
            xbar += static_cast<float>(xv[partialTrueCount + 1]);
        }

        xbar /= 32.0F;
        yv = 0.0F;
        for (partialTrueCount = 0; partialTrueCount < 32; partialTrueCount++) {
            float t;
            t = static_cast<float>(xv[partialTrueCount]) - xbar;
            yv += t * t;
        }

        featureMetrics[loop_ub] = yv / 31.0F;
    }
}

static void blendImage(HDMapping *aInstancePtr, struct2_T *bigImgSt, const ::coder::array<unsigned char, 2U> &currImg, const coder::imref2d currRef,
                       const ::coder::array<bool, 2U> &maskImg) {
    ::coder::array<unsigned char, 2U> b_bigImgSt;
    constructWorldMapStackData *localSD;
    localSD = aInstancePtr->getStackData();
    if (!localSD->pd->alphablend_not_empty) {
        localSD->pd->alphablend.isInitialized = 0;
        localSD->pd->alphablend.matlabCodegenIsDeleted = false;
        localSD->pd->alphablend_not_empty = true;
    }

    if ((bigImgSt->bigImg.size(0) == 0) || (bigImgSt->bigImg.size(1) == 0)) {
        int loop_ub;
        bigImgSt->bigImg.set_size(currImg.size(0), currImg.size(1));
        loop_ub = currImg.size(0) * currImg.size(1);
        for (int i{0}; i < loop_ub; i++) {
            bigImgSt->bigImg[i] = currImg[i];
        }

        bigImgSt->ref.XWorldLimits[0] = currRef.XWorldLimits[0];
        bigImgSt->ref.YWorldLimits[0] = currRef.YWorldLimits[0];
        bigImgSt->ref.ImageSize[0] = currRef.ImageSizeAlias[0];
        bigImgSt->ref.XWorldLimits[1] = currRef.XWorldLimits[1];
        bigImgSt->ref.YWorldLimits[1] = currRef.YWorldLimits[1];
        bigImgSt->ref.ImageSize[1] = currRef.ImageSizeAlias[1];
    } else {
        double x[2];
        int b_x[2];
        int loop_ub;
        x[0] = std::fmax(bigImgSt->ref.YWorldLimits[0] - currRef.YWorldLimits[0],
                         0.0);
        x[1] = std::fmax(bigImgSt->ref.XWorldLimits[0] - currRef.XWorldLimits[0],
                         0.0);
        x[0] = std::round(x[0]);
        x[1] = std::round(x[1]);
        b_bigImgSt.set_size(bigImgSt->bigImg.size(0), bigImgSt->bigImg.size(1));
        loop_ub = bigImgSt->bigImg.size(0) * bigImgSt->bigImg.size(1) - 1;
        for (int i{0}; i <= loop_ub; i++) {
            b_bigImgSt[i] = bigImgSt->bigImg[i];
        }

        coder::padarray(b_bigImgSt, x, bigImgSt->bigImg);
        x[0] = std::fmax(currRef.YWorldLimits[1] - bigImgSt->ref.YWorldLimits[1],
                         0.0);
        x[1] = std::fmax(currRef.XWorldLimits[1] - bigImgSt->ref.XWorldLimits[1],
                         0.0);
        x[0] = std::round(x[0]);
        x[1] = std::round(x[1]);
        b_bigImgSt.set_size(bigImgSt->bigImg.size(0), bigImgSt->bigImg.size(1));
        loop_ub = bigImgSt->bigImg.size(0) * bigImgSt->bigImg.size(1) - 1;
        for (int i{0}; i <= loop_ub; i++) {
            b_bigImgSt[i] = bigImgSt->bigImg[i];
        }

        coder::imref2d b_currRef;
        double c_bigImgSt;
        coder::b_padarray(b_bigImgSt, x, bigImgSt->bigImg);
        b_currRef.ImageSizeAlias[0] = bigImgSt->bigImg.size(0);
        b_currRef.ImageSizeAlias[1] = bigImgSt->bigImg.size(1);
        c_bigImgSt = bigImgSt->ref.XWorldLimits[1];
        bigImgSt->ref.XWorldLimits[0] = std::fmin(currRef.XWorldLimits[0],
                                                  bigImgSt->ref.XWorldLimits[0]);
        bigImgSt->ref.XWorldLimits[1] = std::fmax(currRef.XWorldLimits[1],
                                                  c_bigImgSt);
        c_bigImgSt = bigImgSt->ref.YWorldLimits[1];
        bigImgSt->ref.YWorldLimits[0] = std::fmin(currRef.YWorldLimits[0],
                                                  bigImgSt->ref.YWorldLimits[0]);
        bigImgSt->ref.YWorldLimits[1] = std::fmax(currRef.YWorldLimits[1],
                                                  c_bigImgSt);
        x[0] = (bigImgSt->ref.XWorldLimits[0] - bigImgSt->ref.XWorldLimits[0]) /
                   ((bigImgSt->ref.XWorldLimits[1] - bigImgSt->ref.XWorldLimits[0]) /
                    b_currRef.ImageSizeAlias[1]) +
               0.5;
        x[1] = (bigImgSt->ref.YWorldLimits[0] - bigImgSt->ref.YWorldLimits[0]) /
                   ((bigImgSt->ref.YWorldLimits[1] - bigImgSt->ref.YWorldLimits[0]) /
                    b_currRef.ImageSizeAlias[0]) +
               0.5;
        bigImgSt->ref.ImageSize[0] = b_currRef.ImageSizeAlias[0];
        x[0] = std::round(x[0]);
        bigImgSt->ref.ImageSize[1] = b_currRef.ImageSizeAlias[1];
        x[1] = std::round(x[1]);
        if (localSD->pd->alphablend.isInitialized != 1) {
            localSD->pd->alphablend.isInitialized = 1;
        }

        b_x[0] = static_cast<int>(x[0]);
        b_x[1] = static_cast<int>(x[1]);
        coder::b_Outputs(bigImgSt->bigImg, currImg, maskImg, b_x);
    }
}

static void estiTform(HDMapping *aInstancePtr, const coder::binaryFeatures *preFeatures, const ::coder::array<float, 2U> &prePointsLoc, const coder::binaryFeatures *currFeatures, const ::coder::array<float, 2U> &currPointsLoc, coder::rigidtform2d *tform, ::coder::array<bool, 2U> &inlierIdx, ::coder::array<double, 2U> &validInd1, ::coder::array<double, 2U> &validInd2,
                      bool *isOneSide, int *status) {
    coder::binaryFeatures b_currFeatures;
    coder::binaryFeatures b_preFeatures;
    ::coder::array<double, 2U> b_currMatchedPoints;
    ::coder::array<double, 2U> b_preMatchedPoints;
    ::coder::array<float, 2U> currMatchedPoints;
    ::coder::array<float, 2U> preMatchedPoints;
    ::coder::array<float, 2U> preTformPoints;
    ::coder::array<float, 1U> b_prePointsLoc;
    ::coder::array<float, 1U> c_prePointsLoc;
    ::coder::array<unsigned int, 2U> b_index1;
    ::coder::array<unsigned int, 2U> index1;
    ::coder::array<unsigned int, 2U> index2;
    ::coder::array<unsigned int, 2U> indexPairs;
    ::coder::array<int, 1U> b_r;
    ::coder::array<unsigned int, 1U> currValidPoints_tmp;
    ::coder::array<unsigned int, 1U> preValidPoints_tmp;
    ::coder::array<int, 1U> r1;
    ::coder::array<int, 1U> r2;
    ::coder::array<int, 1U> r3;
    ::coder::array<int, 1U> r4;
    ::coder::array<int, 1U> r5;
    ::coder::array<bool, 1U> in1;
    ::coder::array<bool, 1U> in2;
    constructWorldMapStackData *localSD;
    int loop_ub;
    int mti;
    unsigned int r;
    localSD = aInstancePtr->getStackData();
    r = 5489U;
    localSD->pd->state[0] = 5489U;
    for (mti = 0; mti < 623; mti++) {
        r = ((r ^ r >> 30U) * 1812433253U + static_cast<unsigned int>(mti)) + 1U;
        localSD->pd->state[mti + 1] = r;
    }

    localSD->pd->state[624] = 624U;
    b_prePointsLoc.set_size(prePointsLoc.size(0));
    loop_ub = prePointsLoc.size(0);
    for (int i{0}; i < loop_ub; i++) {
        b_prePointsLoc[i] = prePointsLoc[i];
    }

    c_prePointsLoc.set_size(prePointsLoc.size(0));
    loop_ub = prePointsLoc.size(0);
    for (int i{0}; i < loop_ub; i++) {
        c_prePointsLoc[i] = prePointsLoc[i + prePointsLoc.size(0)];
    }

    coder::inpolygon(b_prePointsLoc, c_prePointsLoc, in1);
    b_prePointsLoc.set_size(currPointsLoc.size(0));
    loop_ub = currPointsLoc.size(0);
    for (int i{0}; i < loop_ub; i++) {
        b_prePointsLoc[i] = currPointsLoc[i];
    }

    c_prePointsLoc.set_size(currPointsLoc.size(0));
    loop_ub = currPointsLoc.size(0);
    for (int i{0}; i < loop_ub; i++) {
        c_prePointsLoc[i] = currPointsLoc[i + currPointsLoc.size(0)];
    }

    coder::inpolygon(b_prePointsLoc, c_prePointsLoc, in2);
    if (prePointsLoc.size(0) < 1) {
        index1.set_size(1, 0);
    } else {
        index1.set_size(1, prePointsLoc.size(0));
        loop_ub = prePointsLoc.size(0) - 1;
        for (int i{0}; i <= loop_ub; i++) {
            index1[i] = static_cast<unsigned int>(i) + 1U;
        }
    }

    if (currPointsLoc.size(0) < 1) {
        index2.set_size(1, 0);
    } else {
        index2.set_size(1, currPointsLoc.size(0));
        loop_ub = currPointsLoc.size(0) - 1;
        for (int i{0}; i <= loop_ub; i++) {
            index2[i] = static_cast<unsigned int>(i) + 1U;
        }
    }

    loop_ub = in1.size(0) - 1;
    mti = 0;
    for (int i{0}; i <= loop_ub; i++) {
        if (!in1[i]) {
            mti++;
        }
    }

    b_r.set_size(mti);
    mti = 0;
    for (int i{0}; i <= loop_ub; i++) {
        if (!in1[i]) {
            b_r[mti] = i + 1;
            mti++;
        }
    }

    b_index1.set_size(1, b_r.size(0));
    loop_ub = b_r.size(0);
    for (int i{0}; i < loop_ub; i++) {
        b_index1[i] = index1[b_r[i] - 1];
    }

    index1.set_size(1, b_index1.size(1));
    loop_ub = b_index1.size(1);
    for (int i{0}; i < loop_ub; i++) {
        index1[i] = b_index1[i];
    }

    loop_ub = in2.size(0) - 1;
    mti = 0;
    for (int i{0}; i <= loop_ub; i++) {
        if (!in2[i]) {
            mti++;
        }
    }

    r1.set_size(mti);
    mti = 0;
    for (int i{0}; i <= loop_ub; i++) {
        if (!in2[i]) {
            r1[mti] = i + 1;
            mti++;
        }
    }

    b_index1.set_size(1, r1.size(0));
    loop_ub = r1.size(0);
    for (int i{0}; i < loop_ub; i++) {
        b_index1[i] = index2[r1[i] - 1];
    }

    index2.set_size(1, b_index1.size(1));
    loop_ub = b_index1.size(1);
    for (int i{0}; i < loop_ub; i++) {
        index2[i] = b_index1[i];
    }

    preValidPoints_tmp.set_size(index1.size(1));
    loop_ub = index1.size(1);
    for (int i{0}; i < loop_ub; i++) {
        preValidPoints_tmp[i] = index1[i];
    }

    currValidPoints_tmp.set_size(index2.size(1));
    loop_ub = index2.size(1);
    for (int i{0}; i < loop_ub; i++) {
        currValidPoints_tmp[i] = index2[i];
    }

    b_preFeatures.Features.set_size(preValidPoints_tmp.size(0), 32);
    b_currFeatures.Features.set_size(currValidPoints_tmp.size(0), 32);
    loop_ub = preValidPoints_tmp.size(0);
    mti = currValidPoints_tmp.size(0);
    for (int i{0}; i < 32; i++) {
        for (int b_i{0}; b_i < loop_ub; b_i++) {
            b_preFeatures.Features[b_i + b_preFeatures.Features.size(0) * i] =
                preFeatures->Features[(static_cast<int>(preValidPoints_tmp[b_i]) +
                                       preFeatures->Features.size(0) * i) -
                                      1];
        }

        for (int b_i{0}; b_i < mti; b_i++) {
            b_currFeatures.Features[b_i + b_currFeatures.Features.size(0) * i] =
                currFeatures->Features[(static_cast<int>(currValidPoints_tmp[b_i]) +
                                        currFeatures->Features.size(0) * i) -
                                       1];
        }
    }

    coder::matchFeatures(aInstancePtr, &b_preFeatures, &b_currFeatures,
                         indexPairs);
    preMatchedPoints.set_size(indexPairs.size(0), 2);
    currMatchedPoints.set_size(indexPairs.size(0), 2);
    loop_ub = indexPairs.size(0);
    for (int i{0}; i < 2; i++) {
        for (int b_i{0}; b_i < loop_ub; b_i++) {
            preMatchedPoints[b_i + preMatchedPoints.size(0) * i] = prePointsLoc[(
                                                                                    static_cast<int>(preValidPoints_tmp[static_cast<int>(indexPairs[b_i]) - 1]) + prePointsLoc.size(0) * i) -
                                                                                1];
            currMatchedPoints[b_i + currMatchedPoints.size(0) * i] =
                currPointsLoc[(static_cast<int>(currValidPoints_tmp[static_cast<int>(indexPairs[b_i + indexPairs.size(0)]) - 1]) + currPointsLoc.size(0) * i) - 1];
        }
    }

    b_preMatchedPoints.set_size(preMatchedPoints.size(0), 2);
    loop_ub = preMatchedPoints.size(0) * 2;
    for (int i{0}; i < loop_ub; i++) {
        b_preMatchedPoints[i] = preMatchedPoints[i];
    }

    b_currMatchedPoints.set_size(currMatchedPoints.size(0), 2);
    loop_ub = currMatchedPoints.size(0) * 2;
    for (int i{0}; i < loop_ub; i++) {
        b_currMatchedPoints[i] = currMatchedPoints[i];
    }

    coder::estgeotform2d(aInstancePtr, b_preMatchedPoints, b_currMatchedPoints,
                         tform, inlierIdx, status);
    loop_ub = inlierIdx.size(0) * inlierIdx.size(1) - 1;
    mti = 0;
    for (int i{0}; i <= loop_ub; i++) {
        if (inlierIdx[i]) {
            mti++;
        }
    }

    r2.set_size(mti);
    mti = 0;
    for (int i{0}; i <= loop_ub; i++) {
        if (inlierIdx[i]) {
            r2[mti] = i + 1;
            mti++;
        }
    }

    validInd1.set_size(1, r2.size(0));
    loop_ub = r2.size(0);
    for (int i{0}; i < loop_ub; i++) {
        validInd1[i] = index1[static_cast<int>(indexPairs[r2[i] - 1]) - 1];
    }

    loop_ub = inlierIdx.size(0) * inlierIdx.size(1) - 1;
    mti = 0;
    for (int i{0}; i <= loop_ub; i++) {
        if (inlierIdx[i]) {
            mti++;
        }
    }

    r3.set_size(mti);
    mti = 0;
    for (int i{0}; i <= loop_ub; i++) {
        if (inlierIdx[i]) {
            r3[mti] = i + 1;
            mti++;
        }
    }

    validInd2.set_size(1, r3.size(0));
    loop_ub = r3.size(0);
    for (int i{0}; i < loop_ub; i++) {
        validInd2[i] = index2[static_cast<int>(indexPairs[(r3[i] +
                                                           indexPairs.size(0)) -
                                                          1]) -
                              1];
    }

    loop_ub = inlierIdx.size(0) * inlierIdx.size(1) - 1;
    mti = 0;
    for (int i{0}; i <= loop_ub; i++) {
        if (inlierIdx[i]) {
            mti++;
        }
    }

    r4.set_size(mti);
    mti = 0;
    for (int i{0}; i <= loop_ub; i++) {
        if (inlierIdx[i]) {
            r4[mti] = i + 1;
            mti++;
        }
    }

    preTformPoints.set_size(r4.size(0), 2);
    loop_ub = r4.size(0);
    for (int i{0}; i < 2; i++) {
        for (int b_i{0}; b_i < loop_ub; b_i++) {
            preTformPoints[b_i + preTformPoints.size(0) * i] = preMatchedPoints
                [(r4[b_i] + preMatchedPoints.size(0) * i) - 1];
        }
    }

    loop_ub = inlierIdx.size(0) * inlierIdx.size(1) - 1;
    mti = 0;
    for (int i{0}; i <= loop_ub; i++) {
        if (inlierIdx[i]) {
            mti++;
        }
    }

    r5.set_size(mti);
    mti = 0;
    for (int i{0}; i <= loop_ub; i++) {
        if (inlierIdx[i]) {
            r5[mti] = i + 1;
            mti++;
        }
    }

    preMatchedPoints.set_size(r5.size(0), 2);
    loop_ub = r5.size(0);
    for (int i{0}; i < 2; i++) {
        for (int b_i{0}; b_i < loop_ub; b_i++) {
            preMatchedPoints[b_i + preMatchedPoints.size(0) * i] =
                currMatchedPoints[(r5[b_i] + currMatchedPoints.size(0) * i) - 1];
        }
    }

    *isOneSide = true;
    if (r5.size(0) != 0) {
        float a;
        float b_a;
        float c_a;
        float d_a;
        bool exitg1;
        bool y;
        in1.set_size(r5.size(0));
        loop_ub = r5.size(0);
        for (int i{0}; i < loop_ub; i++) {
            in1[i] = ((160.0F * preMatchedPoints[i] + 0.0F * preMatchedPoints[i +
                                                                              preMatchedPoints.size(0)]) -
                          51040.0F >
                      0.0F);
        }

        y = true;
        mti = 1;
        exitg1 = false;
        while ((!exitg1) && (mti <= in1.size(0))) {
            if (!in1[mti - 1]) {
                y = false;
                exitg1 = true;
            } else {
                mti++;
            }
        }

        if (!y) {
            in1.set_size(r5.size(0));
            loop_ub = r5.size(0);
            for (int i{0}; i < loop_ub; i++) {
                in1[i] = ((160.0F * preMatchedPoints[i] + 0.0F * preMatchedPoints[i + preMatchedPoints.size(0)]) - 51040.0F < 0.0F);
            }

            y = true;
            mti = 1;
            exitg1 = false;
            while ((!exitg1) && (mti <= in1.size(0))) {
                if (!in1[mti - 1]) {
                    y = false;
                    exitg1 = true;
                } else {
                    mti++;
                }
            }

            if (!y) {
                *isOneSide = false;
            }
        }

        b_prePointsLoc.set_size(r4.size(0));
        loop_ub = r4.size(0);
        for (int i{0}; i < loop_ub; i++) {
            b_prePointsLoc[i] = preTformPoints[i];
        }

        c_prePointsLoc.set_size(r4.size(0));
        loop_ub = r4.size(0);
        for (int i{0}; i < loop_ub; i++) {
            c_prePointsLoc[i] = preTformPoints[i];
        }

        a = coder::internal::maximum(b_prePointsLoc) - coder::internal::minimum(c_prePointsLoc);
        b_prePointsLoc.set_size(r4.size(0));
        loop_ub = r4.size(0);
        for (int i{0}; i < loop_ub; i++) {
            b_prePointsLoc[i] = preTformPoints[i + preTformPoints.size(0)];
        }

        c_prePointsLoc.set_size(r4.size(0));
        loop_ub = r4.size(0);
        for (int i{0}; i < loop_ub; i++) {
            c_prePointsLoc[i] = preTformPoints[i + preTformPoints.size(0)];
        }

        b_a = coder::internal::maximum(b_prePointsLoc) - coder::internal::
                                                             minimum(c_prePointsLoc);
        b_prePointsLoc.set_size(r5.size(0));
        loop_ub = r5.size(0);
        for (int i{0}; i < loop_ub; i++) {
            b_prePointsLoc[i] = preMatchedPoints[i];
        }

        c_prePointsLoc.set_size(r5.size(0));
        loop_ub = r5.size(0);
        for (int i{0}; i < loop_ub; i++) {
            c_prePointsLoc[i] = preMatchedPoints[i];
        }

        c_a = coder::internal::maximum(b_prePointsLoc) - coder::internal::
                                                             minimum(c_prePointsLoc);
        b_prePointsLoc.set_size(r5.size(0));
        loop_ub = r5.size(0);
        for (int i{0}; i < loop_ub; i++) {
            b_prePointsLoc[i] = preMatchedPoints[i + preMatchedPoints.size(0)];
        }

        c_prePointsLoc.set_size(r5.size(0));
        loop_ub = r5.size(0);
        for (int i{0}; i < loop_ub; i++) {
            c_prePointsLoc[i] = preMatchedPoints[i + preMatchedPoints.size(0)];
        }

        d_a = coder::internal::maximum(b_prePointsLoc) - coder::internal::
                                                             minimum(c_prePointsLoc);
        if ((std::sqrt(a * a + b_a * b_a) < 20.0F) || (std::sqrt(c_a * c_a + d_a * d_a) < 20.0F)) {
            *status = 3;
        }
    }
}
}  // namespace buildMapFunctions

namespace coder {
static void Outputs(const vision_AlphaBlender_0 *obj, ::coder::array<unsigned char, 2U> &U0, const ::coder::array<unsigned char, 2U> &U1, const ::coder::array<bool, 2U> &U2) {
    int nRowsIn1;
    int nRowsIn2;
    nRowsIn1 = U0.size(0);
    nRowsIn2 = U1.size(0);
    if ((U0.size(0) != 0) && (U0.size(1) != 0) && (U1.size(0) != 0) &&
        (U1.size(1) != 0) && (U2.size(0) != 0) && (U2.size(1) != 0)) {
        if (U2.size(0) * U2.size(1) == 1) {
            int in2Idx;
            in2Idx = 0;
            if (U2[0]) {
                int cStart;
                int nColsToCopy;
                int nRowsToCopy;
                int rStart;
                nRowsToCopy = U1.size(0);
                nColsToCopy = U1.size(1);
                rStart = obj->P0_Coordinates[1] - 1;
                cStart = obj->P0_Coordinates[0] - 1;
                if (cStart < 0) {
                    in2Idx = -cStart * U1.size(0);
                    nColsToCopy = U1.size(1) + cStart;
                    cStart = 0;
                }

                if (rStart < 0) {
                    in2Idx -= rStart;
                    nRowsToCopy = U1.size(0) + rStart;
                    rStart = 0;
                }

                if (rStart + nRowsToCopy > U0.size(0)) {
                    nRowsToCopy = U0.size(0) - rStart;
                }

                if (cStart + nColsToCopy > U0.size(1)) {
                    nColsToCopy = U0.size(1) - cStart;
                }

                if (nRowsToCopy > 0) {
                    rStart += U0.size(0) * cStart;
                    for (int i{0}; i < nColsToCopy; i++) {
                        for (cStart = 0; cStart < nRowsToCopy; cStart++) {
                            U0[rStart + cStart] = U1[in2Idx + cStart];
                        }

                        in2Idx += nRowsIn2;
                        rStart += nRowsIn1;
                    }
                }
            }
        } else {
            int cStart;
            int findx;
            int in2Idx;
            int nColsToCopy;
            int nRowsToCopy;
            int rStart;
            int strtCpyIdx1;
            in2Idx = 0;
            findx = 0;
            nRowsToCopy = U1.size(0);
            nColsToCopy = U1.size(1);
            rStart = obj->P0_Coordinates[1] - 1;
            cStart = obj->P0_Coordinates[0] - 1;
            if (cStart < 0) {
                in2Idx = -cStart * U1.size(0);
                findx = -cStart * U1.size(0);
                nColsToCopy = U1.size(1) + cStart;
                cStart = 0;
            }

            if (rStart < 0) {
                in2Idx -= rStart;
                findx -= rStart;
                nRowsToCopy = U1.size(0) + rStart;
                rStart = 0;
            }

            strtCpyIdx1 = U0.size(0) * cStart + rStart;
            if (rStart + nRowsToCopy > U0.size(0)) {
                nRowsToCopy = U0.size(0) - rStart;
            }

            if (cStart + nColsToCopy > U0.size(1)) {
                nColsToCopy = U0.size(1) - cStart;
            }

            for (int i{0}; i < nColsToCopy; i++) {
                int indx;
                rStart = in2Idx;
                cStart = strtCpyIdx1;
                indx = findx;
                for (int j{0}; j < nRowsToCopy; j++) {
                    if (U2[indx]) {
                        U0[cStart] = U1[rStart];
                    }

                    cStart++;
                    rStart++;
                    indx++;
                }

                in2Idx += nRowsIn2;
                strtCpyIdx1 += nRowsIn1;
                findx += nRowsIn2;
            }
        }
    }
}

static bool all(const bool x[2]) {
    int k;
    bool exitg1;
    bool y;
    y = true;
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k < 2)) {
        if (!x[k]) {
            y = false;
            exitg1 = true;
        } else {
            k++;
        }
    }

    return y;
}

static void b_Outputs(::coder::array<unsigned char, 2U> &U0, const ::coder::array<unsigned char, 2U> &U1, const ::coder::array<bool, 2U> &U2, const int U3[2]) {
    int nRowsIn1;
    int nRowsIn2;
    nRowsIn1 = U0.size(0);
    nRowsIn2 = U1.size(0);
    if ((U0.size(0) != 0) && (U0.size(1) != 0) && (U1.size(0) != 0) &&
        (U1.size(1) != 0) && (U2.size(0) != 0) && (U2.size(1) != 0)) {
        if (U2.size(0) * U2.size(1) == 1) {
            int in2Idx;
            in2Idx = 0;
            if (U2[0]) {
                int cStart;
                int nColsToCopy;
                int nRowsToCopy;
                int rStart;
                nRowsToCopy = U1.size(0);
                nColsToCopy = U1.size(1);
                rStart = U3[1] - 1;
                cStart = U3[0] - 1;
                if (U3[0] - 1 < 0) {
                    in2Idx = (1 - U3[0]) * U1.size(0);
                    nColsToCopy = (U3[0] + U1.size(1)) - 1;
                    cStart = 0;
                }

                if (U3[1] - 1 < 0) {
                    in2Idx = (in2Idx - U3[1]) + 1;
                    nRowsToCopy = (U1.size(0) + U3[1]) - 1;
                    rStart = 0;
                }

                if (rStart + nRowsToCopy > U0.size(0)) {
                    nRowsToCopy = U0.size(0) - rStart;
                }

                if (cStart + nColsToCopy > U0.size(1)) {
                    nColsToCopy = U0.size(1) - cStart;
                }

                if (nRowsToCopy > 0) {
                    rStart += U0.size(0) * cStart;
                    for (int i{0}; i < nColsToCopy; i++) {
                        for (cStart = 0; cStart < nRowsToCopy; cStart++) {
                            U0[rStart + cStart] = U1[in2Idx + cStart];
                        }

                        in2Idx += nRowsIn2;
                        rStart += nRowsIn1;
                    }
                }
            }
        } else {
            int cStart;
            int findx;
            int in2Idx;
            int nColsToCopy;
            int nRowsToCopy;
            int rStart;
            int strtCpyIdx1;
            in2Idx = 0;
            findx = 0;
            nRowsToCopy = U1.size(0);
            nColsToCopy = U1.size(1);
            rStart = U3[1] - 1;
            cStart = U3[0] - 1;
            if (U3[0] - 1 < 0) {
                in2Idx = (1 - U3[0]) * U1.size(0);
                findx = (1 - U3[0]) * U1.size(0);
                nColsToCopy = (U3[0] + U1.size(1)) - 1;
                cStart = 0;
            }

            if (U3[1] - 1 < 0) {
                in2Idx = (in2Idx - U3[1]) + 1;
                findx = (findx - U3[1]) + 1;
                nRowsToCopy = (U1.size(0) + U3[1]) - 1;
                rStart = 0;
            }

            strtCpyIdx1 = U0.size(0) * cStart + rStart;
            if (rStart + nRowsToCopy > U0.size(0)) {
                nRowsToCopy = U0.size(0) - rStart;
            }

            if (cStart + nColsToCopy > U0.size(1)) {
                nColsToCopy = U0.size(1) - cStart;
            }

            for (int i{0}; i < nColsToCopy; i++) {
                int indx;
                rStart = in2Idx;
                cStart = strtCpyIdx1;
                indx = findx;
                for (int j{0}; j < nRowsToCopy; j++) {
                    if (U2[indx]) {
                        U0[cStart] = U1[rStart];
                    }

                    cStart++;
                    rStart++;
                    indx++;
                }

                in2Idx += nRowsIn2;
                strtCpyIdx1 += nRowsIn1;
                findx += nRowsIn2;
            }
        }
    }
}

static void b_cosd(double *x) {
    if (std::isinf(*x) || std::isnan(*x)) {
        *x = rtNaN;
    } else {
        double absx;
        signed char n;
        *x = rt_remd_snf(*x, 360.0);
        absx = std::abs(*x);
        if (absx > 180.0) {
            if (*x > 0.0) {
                *x -= 360.0;
            } else {
                *x += 360.0;
            }

            absx = std::abs(*x);
        }

        if (absx <= 45.0) {
            *x *= 0.017453292519943295;
            n = 0;
        } else if (absx <= 135.0) {
            if (*x > 0.0) {
                *x = 0.017453292519943295 * (*x - 90.0);
                n = 1;
            } else {
                *x = 0.017453292519943295 * (*x + 90.0);
                n = -1;
            }
        } else if (*x > 0.0) {
            *x = 0.017453292519943295 * (*x - 180.0);
            n = 2;
        } else {
            *x = 0.017453292519943295 * (*x + 180.0);
            n = -2;
        }

        if (n == 0) {
            *x = std::cos(*x);
        } else if (n == 1) {
            *x = -std::sin(*x);
        } else if (n == -1) {
            *x = std::sin(*x);
        } else {
            *x = -std::cos(*x);
        }
    }
}

static double b_mod(double x) {
    double r;
    if (std::isnan(x) || std::isinf(x)) {
        r = rtNaN;
    } else if (x == 0.0) {
        r = 0.0;
    } else {
        r = std::fmod(x, 360.0);
        if (r == 0.0) {
            r = 0.0;
        } else if (x < 0.0) {
            r += 360.0;
        }
    }

    return r;
}

static double b_norm(const ::coder::array<double, 1U> &x) {
    double y;
    if (x.size(0) == 0) {
        y = 0.0;
    } else {
        y = 0.0;
        if (x.size(0) == 1) {
            y = std::abs(x[0]);
        } else {
            double scale;
            int kend;
            scale = 3.3121686421112381E-170;
            kend = x.size(0);
            for (int k{0}; k < kend; k++) {
                double absxk;
                absxk = std::abs(x[k]);
                if (absxk > scale) {
                    double t;
                    t = scale / absxk;
                    y = y * t * t + 1.0;
                    scale = absxk;
                } else {
                    double t;
                    t = absxk / scale;
                    y += t * t;
                }
            }

            y = scale * std::sqrt(y);
        }
    }

    return y;
}

static void b_padarray(const ::coder::array<unsigned char, 2U> &varargin_1,
                       const double varargin_2[2], ::coder::array<unsigned char, 2U> &b) {
    if ((varargin_1.size(0) == 0) || (varargin_1.size(1) == 0)) {
        double sizeB_idx_0;
        double sizeB_idx_1;
        int loop_ub;
        sizeB_idx_0 = static_cast<double>(varargin_1.size(0)) + varargin_2[0];
        sizeB_idx_1 = static_cast<double>(varargin_1.size(1)) + varargin_2[1];
        b.set_size(static_cast<int>(sizeB_idx_0), static_cast<int>(sizeB_idx_1));
        loop_ub = static_cast<int>(sizeB_idx_0) * static_cast<int>(sizeB_idx_1);
        for (int i{0}; i < loop_ub; i++) {
            b[i] = 0U;
        }
    } else {
        int i;
        int i1;
        int loop_ub;
        i = static_cast<int>(static_cast<double>(varargin_1.size(1)) +
                             varargin_2[1]);
        b.set_size(static_cast<int>(static_cast<double>(varargin_1.size(0)) +
                                    varargin_2[0]),
                   static_cast<int>(static_cast<double>(varargin_1.size(1)) + varargin_2[1]));
        loop_ub = varargin_1.size(1) + 1;
        for (int j{loop_ub}; j <= i; j++) {
            i1 = b.size(0);
            for (int b_i{0}; b_i < i1; b_i++) {
                b[b_i + b.size(0) * (j - 1)] = 0U;
            }
        }

        i = varargin_1.size(1);
        loop_ub = varargin_1.size(0) + 1;
        for (int j{0}; j < i; j++) {
            i1 = b.size(0);
            for (int b_i{loop_ub}; b_i <= i1; b_i++) {
                b[(b_i + b.size(0) * j) - 1] = 0U;
            }
        }

        i = varargin_1.size(1);
        loop_ub = varargin_1.size(0);
        for (int j{0}; j < i; j++) {
            for (int b_i{0}; b_i < loop_ub; b_i++) {
                b[b_i + b.size(0) * j] = varargin_1[b_i + varargin_1.size(0) * j];
            }
        }
    }
}

static double b_rand(HDMapping *aInstancePtr) {
    constructWorldMapStackData *localSD;
    double r;
    localSD = aInstancePtr->getStackData();

    /* ========================= COPYRIGHT NOTICE ============================ */
    /*  This is a uniform (0,1) pseudorandom number generator based on:        */
    /*                                                                         */
    /*  A C-program for MT19937, with initialization improved 2002/1/26.       */
    /*  Coded by Takuji Nishimura and Makoto Matsumoto.                        */
    /*                                                                         */
    /*  Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura,      */
    /*  All rights reserved.                                                   */
    /*                                                                         */
    /*  Redistribution and use in source and binary forms, with or without     */
    /*  modification, are permitted provided that the following conditions     */
    /*  are met:                                                               */
    /*                                                                         */
    /*    1. Redistributions of source code must retain the above copyright    */
    /*       notice, this list of conditions and the following disclaimer.     */
    /*                                                                         */
    /*    2. Redistributions in binary form must reproduce the above copyright */
    /*       notice, this list of conditions and the following disclaimer      */
    /*       in the documentation and/or other materials provided with the     */
    /*       distribution.                                                     */
    /*                                                                         */
    /*    3. The names of its contributors may not be used to endorse or       */
    /*       promote products derived from this software without specific      */
    /*       prior written permission.                                         */
    /*                                                                         */
    /*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS    */
    /*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT      */
    /*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR  */
    /*  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT  */
    /*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,  */
    /*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT       */
    /*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,  */
    /*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY  */
    /*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT    */
    /*  (INCLUDING  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE */
    /*  OF THIS  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  */
    /*                                                                         */
    /* =============================   END   ================================= */
    unsigned int u[2];
    do {
        for (int j{0}; j < 2; j++) {
            unsigned int mti;
            unsigned int y;
            mti = localSD->pd->state[624] + 1U;
            if (localSD->pd->state[624] + 1U >= 625U) {
                for (int kk{0}; kk < 227; kk++) {
                    y = (localSD->pd->state[kk] & 2147483648U) | (localSD->pd->state[kk + 1] & 2147483647U);
                    if ((y & 1U) == 0U) {
                        y >>= 1U;
                    } else {
                        y = y >> 1U ^ 2567483615U;
                    }

                    localSD->pd->state[kk] = localSD->pd->state[kk + 397] ^ y;
                }

                for (int kk{0}; kk < 396; kk++) {
                    y = (localSD->pd->state[kk + 227] & 2147483648U) | (localSD->pd->state[kk + 228] & 2147483647U);
                    if ((y & 1U) == 0U) {
                        y >>= 1U;
                    } else {
                        y = y >> 1U ^ 2567483615U;
                    }

                    localSD->pd->state[kk + 227] = localSD->pd->state[kk] ^ y;
                }

                y = (localSD->pd->state[623] & 2147483648U) | (localSD->pd->state[0] & 2147483647U);
                if ((y & 1U) == 0U) {
                    y >>= 1U;
                } else {
                    y = y >> 1U ^ 2567483615U;
                }

                localSD->pd->state[623] = localSD->pd->state[396] ^ y;
                mti = 1U;
            }

            y = localSD->pd->state[static_cast<int>(mti) - 1];
            localSD->pd->state[624] = mti;
            y ^= y >> 11U;
            y ^= y << 7U & 2636928640U;
            y ^= y << 15U & 4022730752U;
            u[j] = y ^ y >> 18U;
        }

        u[0] >>= 5U;
        u[1] >>= 6U;
        r = 1.1102230246251565E-16 * (static_cast<double>(u[0]) * 6.7108864E+7 +
                                      static_cast<double>(u[1]));
    } while (r == 0.0);

    return r;
}

static void b_sind(double *x) {
    if (std::isinf(*x) || std::isnan(*x)) {
        *x = rtNaN;
    } else {
        double absx;
        signed char n;
        *x = rt_remd_snf(*x, 360.0);
        absx = std::abs(*x);
        if (absx > 180.0) {
            if (*x > 0.0) {
                *x -= 360.0;
            } else {
                *x += 360.0;
            }

            absx = std::abs(*x);
        }

        if (absx <= 45.0) {
            *x *= 0.017453292519943295;
            n = 0;
        } else if (absx <= 135.0) {
            if (*x > 0.0) {
                *x = 0.017453292519943295 * (*x - 90.0);
                n = 1;
            } else {
                *x = 0.017453292519943295 * (*x + 90.0);
                n = -1;
            }
        } else if (*x > 0.0) {
            *x = 0.017453292519943295 * (*x - 180.0);
            n = 2;
        } else {
            *x = 0.017453292519943295 * (*x + 180.0);
            n = -2;
        }

        if (n == 0) {
            *x = std::sin(*x);
        } else if (n == 1) {
            *x = std::cos(*x);
        } else if (n == -1) {
            *x = -std::cos(*x);
        } else {
            *x = -std::sin(*x);
        }
    }
}

static void b_sparse(const ::coder::array<double, 1U> &varargin_1, const ::coder::array<double, 1U> &varargin_2, const ::coder::array<double, 1U> &varargin_3, sparse *y) {
    anonymous_function b_this;
    ::coder::array<int, 1U> sortedIndices;
    ::coder::array<int, 1U> t;
    int i;
    int nc;
    int ns;
    int ny;
    nc = varargin_2.size(0);
    ns = varargin_1.size(0);
    b_this.workspace.b.set_size(varargin_1.size(0));
    for (int k{0}; k < ns; k++) {
        b_this.workspace.b[k] = static_cast<int>(varargin_1[k]);
    }

    ns = varargin_2.size(0);
    b_this.workspace.a.set_size(varargin_2.size(0));
    for (int k{0}; k < ns; k++) {
        b_this.workspace.a[k] = static_cast<int>(varargin_2[k]);
    }

    sortedIndices.set_size(varargin_2.size(0));
    for (int k{0}; k < nc; k++) {
        sortedIndices[k] = k + 1;
    }

    internal::introsort(sortedIndices, b_this.workspace.a.size(0), &b_this);
    ny = b_this.workspace.a.size(0);
    t.set_size(b_this.workspace.a.size(0));
    ns = b_this.workspace.a.size(0);
    for (i = 0; i < ns; i++) {
        t[i] = b_this.workspace.a[i];
    }

    for (int k{0}; k < ny; k++) {
        b_this.workspace.a[k] = t[sortedIndices[k] - 1];
    }

    ny = b_this.workspace.b.size(0);
    t.set_size(b_this.workspace.b.size(0));
    ns = b_this.workspace.b.size(0);
    for (i = 0; i < ns; i++) {
        t[i] = b_this.workspace.b[i];
    }

    for (int k{0}; k < ny; k++) {
        b_this.workspace.b[k] = t[sortedIndices[k] - 1];
    }

    if ((b_this.workspace.b.size(0) == 0) || (b_this.workspace.a.size(0) == 0)) {
        ny = 0;
        y->n = 0;
    } else {
        ns = b_this.workspace.b.size(0);
        ny = b_this.workspace.b[0];
        for (int k{2}; k <= ns; k++) {
            i = b_this.workspace.b[k - 1];
            if (ny < i) {
                ny = i;
            }
        }

        y->n = b_this.workspace.a[b_this.workspace.a.size(0) - 1];
    }

    y->m = ny;
    ns = varargin_2.size(0);
    if (ns < 1) {
        ns = 1;
    }

    y->d.set_size(ns);
    y->colidx.set_size(y->n + 1);
    y->colidx[0] = 1;
    y->rowidx.set_size(ns);
    for (i = 0; i < ns; i++) {
        y->d[i] = 0.0;
        y->rowidx[i] = 0;
    }

    ns = 0;
    i = y->n;
    for (ny = 0; ny < i; ny++) {
        while ((ns + 1 <= nc) && (b_this.workspace.a[ns] == ny + 1)) {
            y->rowidx[ns] = b_this.workspace.b[ns];
            ns++;
        }

        y->colidx[ny + 1] = ns + 1;
    }

    for (int k{0}; k < nc; k++) {
        y->d[k] = varargin_3[sortedIndices[k] - 1];
    }

    y->fillIn();
}

static int cfclose(HDMapping *aInstancePtr, double fid) {
    FILE *filestar;
    constructWorldMapStackData *localSD;
    int st;
    signed char b_fileid;
    signed char fileid;
    localSD = aInstancePtr->getStackData();
    st = -1;
    fileid = static_cast<signed char>(fid);
    if ((static_cast<signed char>(fid) < 0) || (fid != static_cast<signed char>(fid))) {
        fileid = -1;
    }

    b_fileid = fileid;
    if (fileid < 0) {
        b_fileid = -1;
    }

    if (b_fileid >= 3) {
        filestar = localSD->pd->eml_openfiles[b_fileid - 3];
    } else if (b_fileid == 0) {
        filestar = stdin;
    } else if (b_fileid == 1) {
        filestar = stdout;
    } else if (b_fileid == 2) {
        filestar = stderr;
    } else {
        filestar = NULL;
    }

    if ((filestar != NULL) && (fileid >= 3)) {
        int cst;
        cst = fclose(filestar);
        if (cst == 0) {
            st = 0;
            localSD->pd->eml_openfiles[fileid - 3] = NULL;
        }
    }

    return st;
}

static signed char cfopen(HDMapping *aInstancePtr) {
    constructWorldMapStackData *localSD;
    signed char fileid;
    signed char j;
    localSD = aInstancePtr->getStackData();
    fileid = -1;
    j = filedata(aInstancePtr);
    if (j >= 1) {
        FILE *filestar;
        filestar = fopen("./data/preSavedData/imagePathList.txt", "rb");
        if (filestar != NULL) {
            localSD->pd->eml_openfiles[j - 1] = filestar;
            fileid = static_cast<signed char>(j + 2);
        }
    }

    return fileid;
}

static void contrib(double x1, double b_y1, double x2, double y2, signed char quad1, signed char quad2, double scale, signed char *diffQuad, bool *onj) {
    double cp;
    *onj = false;
    *diffQuad = static_cast<signed char>(quad2 - quad1);
    cp = x1 * y2 - x2 * b_y1;
    if (std::abs(cp) < scale) {
        *onj = (x1 * x2 + b_y1 * y2 <= 0.0);
        if ((*diffQuad == 2) || (*diffQuad == -2)) {
            *diffQuad = 0;
        } else if (*diffQuad == -3) {
            *diffQuad = 1;
        } else if (*diffQuad == 3) {
            *diffQuad = -1;
        }
    } else if (cp < 0.0) {
        if (*diffQuad == 2) {
            *diffQuad = -2;
        } else if (*diffQuad == -3) {
            *diffQuad = 1;
        } else if (*diffQuad == 3) {
            *diffQuad = -1;
        }
    } else if (*diffQuad == -2) {
        *diffQuad = 2;
    } else if (*diffQuad == -3) {
        *diffQuad = 1;
    } else if (*diffQuad == 3) {
        *diffQuad = -1;
    }
}

static void do_vectors(const double a[10], const ::coder::array<double, 2U> &b, double c_data[], int *c_size, int ia_data[], int *ia_size, int *ib_size) {
    ::coder::array<int, 2U> bperm;
    ::coder::array<int, 1U> b_iwork;
    ::coder::bounded_array<int, 10U, 1U> b_ia_data;
    double ak;
    int aperm[10];
    int iwork[10];
    int b_i;
    int i;
    int ialast;
    int iblast;
    int k;
    int kEnd;
    int n;
    int nc;
    int nia;
    int pEnd;
    int qEnd;
    *ib_size = 0;
    for (k = 0; k <= 8; k += 2) {
        ak = a[k + 1];
        if ((a[k] <= ak) || std::isnan(ak)) {
            aperm[k] = k + 1;
            aperm[k + 1] = k + 2;
        } else {
            aperm[k] = k + 2;
            aperm[k + 1] = k + 1;
        }
    }

    i = 2;
    while (i < 10) {
        ialast = i << 1;
        iblast = 1;
        for (pEnd = i + 1; pEnd < 11; pEnd = qEnd + i) {
            nia = iblast;
            nc = pEnd - 1;
            qEnd = iblast + ialast;
            if (qEnd > 11) {
                qEnd = 11;
            }

            k = 0;
            kEnd = qEnd - iblast;
            while (k + 1 <= kEnd) {
                ak = a[aperm[nc] - 1];
                b_i = aperm[nia - 1];
                if ((a[b_i - 1] <= ak) || std::isnan(ak)) {
                    iwork[k] = b_i;
                    nia++;
                    if (nia == pEnd) {
                        while (nc + 1 < qEnd) {
                            k++;
                            iwork[k] = aperm[nc];
                            nc++;
                        }
                    }
                } else {
                    iwork[k] = aperm[nc];
                    nc++;
                    if (nc + 1 == qEnd) {
                        while (nia < pEnd) {
                            k++;
                            iwork[k] = aperm[nia - 1];
                            nia++;
                        }
                    }
                }

                k++;
            }

            for (k = 0; k < kEnd; k++) {
                aperm[(iblast + k) - 1] = iwork[k];
            }

            iblast = qEnd;
        }

        i = ialast;
    }

    n = b.size(1) + 1;
    bperm.set_size(1, b.size(1));
    i = b.size(1);
    for (b_i = 0; b_i < i; b_i++) {
        bperm[b_i] = 0;
    }

    if (b.size(1) != 0) {
        b_iwork.set_size(b.size(1));
        b_i = b.size(1) - 1;
        for (k = 1; k <= b_i; k += 2) {
            ak = b[k];
            if ((b[k - 1] <= ak) || std::isnan(ak)) {
                bperm[k - 1] = k;
                bperm[k] = k + 1;
            } else {
                bperm[k - 1] = k + 1;
                bperm[k] = k;
            }
        }

        if ((b.size(1) & 1) != 0) {
            bperm[b.size(1) - 1] = b.size(1);
        }

        i = 2;
        while (i < n - 1) {
            ialast = i << 1;
            iblast = 1;
            for (pEnd = i + 1; pEnd < n; pEnd = qEnd + i) {
                nia = iblast;
                nc = pEnd - 1;
                qEnd = iblast + ialast;
                if (qEnd > n) {
                    qEnd = n;
                }

                k = 0;
                kEnd = qEnd - iblast;
                while (k + 1 <= kEnd) {
                    ak = b[bperm[nc] - 1];
                    b_i = bperm[nia - 1];
                    if ((b[b_i - 1] <= ak) || std::isnan(ak)) {
                        b_iwork[k] = b_i;
                        nia++;
                        if (nia == pEnd) {
                            while (nc + 1 < qEnd) {
                                k++;
                                b_iwork[k] = bperm[nc];
                                nc++;
                            }
                        }
                    } else {
                        b_iwork[k] = bperm[nc];
                        nc++;
                        if (nc + 1 == qEnd) {
                            while (nia < pEnd) {
                                k++;
                                b_iwork[k] = bperm[nia - 1];
                                nia++;
                            }
                        }
                    }

                    k++;
                }

                for (k = 0; k < kEnd; k++) {
                    bperm[(iblast + k) - 1] = b_iwork[k];
                }

                iblast = qEnd;
            }

            i = ialast;
        }
    }

    nc = -1;
    nia = -1;
    i = 0;
    ialast = 1;
    iblast = 1;
    while ((ialast <= 10) && (iblast <= b.size(1))) {
        double bk;
        pEnd = ialast;
        ak = a[aperm[ialast - 1] - 1];
        while ((pEnd < 10) && (a[aperm[pEnd] - 1] == ak)) {
            pEnd++;
        }

        ialast = pEnd;
        bk = b[bperm[iblast - 1] - 1];
        while ((iblast < b.size(1)) && (b[bperm[iblast] - 1] == bk)) {
            iblast++;
        }

        if (ak == bk) {
            ialast = pEnd + 1;
            i = pEnd;
            iblast++;
        } else {
            bool p;
            if (std::isnan(bk)) {
                p = !std::isnan(ak);
            } else if (std::isnan(ak)) {
                p = false;
            } else {
                p = (ak < bk);
            }

            if (p) {
                nc++;
                nia++;
                ia_data[nia] = aperm[i];
                ialast = pEnd + 1;
                i = pEnd;
            } else {
                iblast++;
            }
        }
    }

    while (ialast <= 10) {
        pEnd = ialast;
        while ((pEnd < 10) && (a[aperm[pEnd] - 1] == a[aperm[ialast - 1] - 1])) {
            pEnd++;
        }

        nc++;
        nia++;
        ia_data[nia] = aperm[i];
        ialast = pEnd + 1;
        i = pEnd;
    }

    if (nia + 1 < 1) {
        b_i = -1;
    } else {
        b_i = nia;
    }

    *ia_size = b_i + 1;
    internal::sort(ia_data, ia_size, b_ia_data.data, &b_ia_data.size[0]);
    b_i = static_cast<unsigned char>(nia + 1);
    for (k = 0; k < b_i; k++) {
        c_data[k] = a[ia_data[k] - 1];
    }

    if (nc + 1 < 1) {
        nc = -1;
    }

    *c_size = nc + 1;
}

static void estgeotform2d(HDMapping *aInstancePtr, const ::coder::array<double, 2U> &matchedPoints1, const ::coder::array<double, 2U> &matchedPoints2, rigidtform2d *tform, ::coder::array<bool, 2U> &inlierIndex, int *status) {
    ::coder::array<double, 3U> points;
    ::coder::array<bool, 1U> inlierIdx;
    double failedMatrix[9];
    double tmatrix_data[9];
    double x_data[9];
    double r;
    double s;
    double smax;
    double wpr;
    int tmatrix_size[2];
    int i;
    int i1;
    bool isodd;
    *status = (matchedPoints1.size(0) < 2);
    std::memset(&failedMatrix[0], 0, 9U * sizeof(double));
    failedMatrix[0] = 1.0;
    failedMatrix[4] = 1.0;
    failedMatrix[8] = 1.0;
    if (*status == 0) {
        int yk;
        bool guard1{false};

        points.set_size(matchedPoints1.size(0), 2, 2);
        i = matchedPoints1.size(0) << 1;
        for (int j{0}; j < i; j++) {
            points[j] = matchedPoints1[j];
        }

        i1 = matchedPoints2.size(0) << 1;
        for (int j{0}; j < i1; j++) {
            points[i + j] = matchedPoints2[j];
        }

        vision::internal::ransac::msac(aInstancePtr, points, &isodd,
                                       tmatrix_data, tmatrix_size, inlierIdx);
        inlierIndex.set_size(inlierIdx.size(0), 1);
        yk = inlierIdx.size(0);
        for (i = 0; i < yk; i++) {
            inlierIndex[i] = inlierIdx[i];
        }

        if (!isodd) {
            *status = 2;
        }

        if ((tmatrix_size[0] == 0) || (tmatrix_size[1] == 0)) {
            smax = 1.0;
        } else {
            int ldap1;
            int m;
            int n;
            int u1;
            int x_size_idx_0;
            signed char ipiv_data[3];
            m = tmatrix_size[0];
            n = tmatrix_size[1] - 2;
            x_size_idx_0 = tmatrix_size[0];
            yk = tmatrix_size[0] * tmatrix_size[1];
            std::copy(&tmatrix_data[0], &tmatrix_data[yk], &x_data[0]);
            yk = tmatrix_size[0];
            u1 = tmatrix_size[1];
            if (yk <= u1) {
                u1 = yk;
            }

            ipiv_data[0] = 1;
            yk = 1;
            for (int k{2}; k <= u1; k++) {
                yk++;
                ipiv_data[k - 1] = static_cast<signed char>(yk);
            }

            ldap1 = tmatrix_size[0];
            if (tmatrix_size[0] - 1 <= tmatrix_size[1]) {
                i = tmatrix_size[0];
            } else {
                i = 2;
            }

            for (int j{0}; j <= i - 2; j++) {
                int b;
                int jA;
                int jj;
                int jp1j;
                int jy;
                int mmj;
                mmj = m - j;
                b = j * (m + 1);
                jj = j * (ldap1 + 1);
                jp1j = b + 2;
                if (mmj < 1) {
                    yk = -1;
                } else {
                    yk = 0;
                    if (mmj > 1) {
                        smax = std::abs(x_data[jj]);
                        for (int k{2}; k <= mmj; k++) {
                            s = std::abs(x_data[(b + k) - 1]);
                            if (s > smax) {
                                yk = k - 1;
                                smax = s;
                            }
                        }
                    }
                }

                if (x_data[jj + yk] != 0.0) {
                    if (yk != 0) {
                        jy = j + yk;
                        ipiv_data[j] = static_cast<signed char>(jy + 1);
                        for (int k{0}; k <= n + 1; k++) {
                            yk = k * m;
                            jA = j + yk;
                            smax = x_data[jA];
                            i1 = jy + yk;
                            x_data[jA] = x_data[i1];
                            x_data[i1] = smax;
                        }
                    }

                    i1 = jj + mmj;
                    for (yk = jp1j; yk <= i1; yk++) {
                        x_data[yk - 1] /= x_data[jj];
                    }
                }

                jp1j = n - j;
                jy = b + m;
                jA = jj + ldap1;
                for (int k{0}; k <= jp1j; k++) {
                    yk = jy + k * m;
                    smax = x_data[yk];
                    if (x_data[yk] != 0.0) {
                        i1 = jA + 2;
                        yk = mmj + jA;
                        for (b = i1; b <= yk; b++) {
                            x_data[b - 1] += x_data[((jj + b) - jA) - 1] * -smax;
                        }
                    }

                    jA += m;
                }
            }

            smax = x_data[0];
            for (int k{0}; k <= x_size_idx_0 - 2; k++) {
                smax *= x_data[(k + x_size_idx_0 * (k + 1)) + 1];
            }

            isodd = false;
            for (int k{0}; k <= u1 - 2; k++) {
                if (ipiv_data[k] > k + 1) {
                    isodd = !isodd;
                }
            }

            if (isodd) {
                smax = -smax;
            }
        }

        guard1 = false;
        if (smax == 0.0) {
            guard1 = true;
        } else {
            bool b_tmp_data[9];
            bool tmp_data[9];
            bool exitg1;
            yk = tmatrix_size[0] * tmatrix_size[1];
            for (i = 0; i < yk; i++) {
                tmp_data[i] = std::isinf(tmatrix_data[i]);
            }

            for (i = 0; i < yk; i++) {
                b_tmp_data[i] = std::isnan(tmatrix_data[i]);
            }

            inlierIdx.set_size(yk);
            for (i = 0; i < yk; i++) {
                inlierIdx[i] = (tmp_data[i] || b_tmp_data[i]);
            }

            isodd = false;
            yk = 1;
            exitg1 = false;
            while ((!exitg1) && (yk <= inlierIdx.size(0))) {
                if (inlierIdx[yk - 1]) {
                    isodd = true;
                    exitg1 = true;
                } else {
                    yk++;
                }
            }

            if (isodd) {
                guard1 = true;
            }
        }

        if (guard1) {
            *status = 2;
            tmatrix_size[0] = 3;
            std::copy(&failedMatrix[0], &failedMatrix[9], &tmatrix_data[0]);
        }
    } else {
        int yk;
        inlierIndex.set_size(matchedPoints1.size(0), matchedPoints1.size(0));
        yk = matchedPoints1.size(0) * matchedPoints1.size(0);
        for (i = 0; i < yk; i++) {
            inlierIndex[i] = false;
        }

        tmatrix_size[0] = 3;
        std::copy(&failedMatrix[0], &failedMatrix[9], &tmatrix_data[0]);
    }

    if (*status != 0) {
        tmatrix_size[0] = 3;
        std::copy(&failedMatrix[0], &failedMatrix[9], &tmatrix_data[0]);
    }

    i = tmatrix_size[0];
    for (i1 = 0; i1 < 3; i1++) {
        failedMatrix[3 * i1] = tmatrix_data[i1];
        failedMatrix[3 * i1 + 1] = tmatrix_data[i1 + i];
        failedMatrix[3 * i1 + 2] = tmatrix_data[i1 + i * 2];
    }

    double b_dv[4];
    double b_failedMatrix[4];
    b_failedMatrix[0] = failedMatrix[0];
    b_failedMatrix[1] = failedMatrix[1];
    b_failedMatrix[2] = failedMatrix[3];
    b_failedMatrix[3] = failedMatrix[4];
    images::geotrans::internal::constrainToRotationMatrix2D(b_failedMatrix,
                                                            b_dv, &smax);
    if (std::isnan(smax + 180.0) || std::isinf(smax + 180.0)) {
        r = rtNaN;
    } else if (smax + 180.0 == 0.0) {
        r = 0.0;
    } else {
        r = std::fmod(smax + 180.0, 360.0);
        if (r == 0.0) {
            r = 0.0;
        } else if (smax + 180.0 < 0.0) {
            r += 360.0;
        }
    }

    wpr = std::round(r - 180.0);
    if (r - 180.0 == wpr) {
        isodd = true;
    } else {
        smax = std::abs((r - 180.0) - wpr);
        if ((r - 180.0 == 0.0) || (wpr == 0.0)) {
            isodd = (smax < 4.94065645841247E-324);
        } else {
            s = std::abs(r - 180.0) + std::abs(wpr);
            if (s < 2.2250738585072014E-308) {
                isodd = (smax < 4.94065645841247E-324);
            } else {
                isodd = (smax / std::fmin(s, 1.7976931348623157E+308) <
                         2.2204460492503131E-16);
            }
        }
    }

    smax = r - 180.0;
    if (isodd) {
        smax = wpr;
    }

    tform->RotationAngle = smax;
    tform->Translation[0] = failedMatrix[6];
    tform->Translation[1] = failedMatrix[7];
}

static void fgetl(HDMapping *aInstancePtr, double fileID, ::coder::array<char, 2U> &out) {
    FILE *filestar;
    int b_i;
    int i;
    filestar = fileManager(aInstancePtr, fileID);
    if ((!(fileID != 0.0)) || (!(fileID != 1.0)) || (!(fileID != 2.0))) {
        filestar = NULL;
    }

    out.set_size(1, 0);
    if (!(filestar == NULL)) {
        FILE *b_NULL;
        FILE *b_filestar;
        char *cOut;
        int exitg1;
        int st;
        int wherefrom;
        char ReadBuff[1024];
        bool readNewline;
        do {
            int reachedEndOfFile;
            exitg1 = 0;
            cOut = fgets(&ReadBuff[0], 1024, filestar);
            readNewline = false;
            b_NULL = NULL;
            b_filestar = fileManager(aInstancePtr, fileID);
            if (b_filestar == b_NULL) {
                reachedEndOfFile = 0;
            } else {
                st = feof(b_filestar);
                reachedEndOfFile = ((int)st != 0);
            }

            if (cOut == NULL) {
                exitg1 = 1;
            } else {
                int carriageReturnAt;
                int idx;
                bool fileEndAfterCarriageReturn;
                bool newLineAfterCarriageReturn;
                idx = 1;
                carriageReturnAt = 0;
                if (reachedEndOfFile != 0) {
                    bool exitg2;
                    i = 0;
                    exitg2 = false;
                    while ((!exitg2) && (i < 1024)) {
                        if (ReadBuff[i] == '\x00') {
                            idx = i + 1;
                            exitg2 = true;
                        } else {
                            if ((carriageReturnAt == 0) && (ReadBuff[i] == '\x0d')) {
                                carriageReturnAt = i + 1;
                            }

                            i++;
                        }
                    }

                    if (ReadBuff[idx - 1] == '\x00') {
                        idx--;
                    }
                } else {
                    bool exitg2;
                    i = 0;
                    exitg2 = false;
                    while ((!exitg2) && (i < 1025)) {
                        if (i + 1 > 1024) {
                            idx = 1023;
                            exitg2 = true;
                        } else if (ReadBuff[i] == '\x0a') {
                            idx = i + 1;
                            exitg2 = true;
                        } else {
                            if ((carriageReturnAt == 0) && (ReadBuff[i] == '\x0d')) {
                                carriageReturnAt = i + 1;
                            }

                            i++;
                        }
                    }

                    readNewline = (ReadBuff[idx - 1] == '\x0a');
                }

                if ((carriageReturnAt > 0) && (carriageReturnAt < 1024)) {
                    newLineAfterCarriageReturn = (ReadBuff[carriageReturnAt] == '\x0a');
                    if ((reachedEndOfFile != 0) && (ReadBuff[carriageReturnAt] ==
                                                    '\x00')) {
                        fileEndAfterCarriageReturn = true;
                    } else {
                        fileEndAfterCarriageReturn = false;
                    }
                } else {
                    newLineAfterCarriageReturn = false;
                    fileEndAfterCarriageReturn = false;
                }

                if ((carriageReturnAt == 0) || newLineAfterCarriageReturn ||
                    fileEndAfterCarriageReturn) {
                    if (idx < 1) {
                        i = 0;
                    } else {
                        i = idx;
                    }

                    b_i = out.size(1);
                    out.set_size(out.size(0), out.size(1) + i);
                    for (int i1{0}; i1 < i; i1++) {
                        out[b_i + i1] = ReadBuff[i1];
                    }
                } else {
                    b_i = out.size(1);
                    out.set_size(out.size(0), out.size(1) + carriageReturnAt);
                    for (int i1{0}; i1 < carriageReturnAt; i1++) {
                        out[b_i + i1] = ReadBuff[i1];
                    }

                    wherefrom = SEEK_CUR;
                    b_filestar = fileManager(aInstancePtr, fileID);
                    if ((!(fileID != 0.0)) || (!(fileID != 1.0)) || (!(fileID != 2.0))) {
                        b_filestar = NULL;
                    }

                    if (!(b_filestar == NULL)) {
                        fseek(b_filestar, (long int)(carriageReturnAt - idx), wherefrom);
                    }
                }

                if (readNewline || (reachedEndOfFile != 0) || (carriageReturnAt > 0)) {
                    exitg1 = 1;
                }
            }
        } while (exitg1 == 0);

        b_NULL = NULL;
        filestar = fileManager(aInstancePtr, fileID);
        if (filestar == b_NULL) {
            b_i = 0;
        } else {
            st = feof(filestar);
            b_i = ((int)st != 0);
        }

        if (b_i == 0) {
            filestar = fileManager(aInstancePtr, fileID);
            if ((!(fileID != 0.0)) || (!(fileID != 1.0)) || (!(fileID != 2.0))) {
                filestar = NULL;
            }

            if (!(filestar == NULL)) {
                unsigned char buf;
                fread(&buf, sizeof(unsigned char), (size_t)1, filestar);
            }

            b_NULL = NULL;
            filestar = fileManager(aInstancePtr, fileID);
            if (filestar == b_NULL) {
                b_i = 0;
            } else {
                st = feof(filestar);
                b_i = ((int)st != 0);
            }

            if (b_i == 0) {
                wherefrom = SEEK_CUR;
                filestar = fileManager(aInstancePtr, fileID);
                if ((!(fileID != 0.0)) || (!(fileID != 1.0)) || (!(fileID != 2.0))) {
                    filestar = NULL;
                }

                if (!(filestar == NULL)) {
                    fseek(filestar, (long int)-1.0, wherefrom);
                }
            }
        }
    }

    if (out.size(1) != 0) {
        if (out[out.size(1) - 1] == '\x0a') {
            if ((out.size(1) > 1) && (out[out.size(1) - 2] == '\x0d')) {
                if (out.size(1) - 2 < 1) {
                    i = 0;
                } else {
                    i = out.size(1) - 2;
                }

                for (b_i = 0; b_i < i; b_i++) {
                    out[b_i] = out[b_i];
                }

                out.set_size(1, i);
            } else {
                if (out.size(1) - 1 < 1) {
                    i = 0;
                } else {
                    i = out.size(1) - 1;
                }

                for (b_i = 0; b_i < i; b_i++) {
                    out[b_i] = out[b_i];
                }

                out.set_size(1, i);
            }
        } else if (out[out.size(1) - 1] == '\x0d') {
            if (out.size(1) - 1 < 1) {
                i = 0;
            } else {
                i = out.size(1) - 1;
            }

            for (b_i = 0; b_i < i; b_i++) {
                out[b_i] = out[b_i];
            }

            out.set_size(1, i);
        }
    }
}

static FILE *fileManager(HDMapping *aInstancePtr, double varargin_1) {
    FILE *f;
    constructWorldMapStackData *localSD;
    signed char fileid;
    localSD = aInstancePtr->getStackData();
    fileid = static_cast<signed char>(varargin_1);
    if ((static_cast<signed char>(varargin_1) < 0) || (varargin_1 !=
                                                       static_cast<signed char>(varargin_1))) {
        fileid = -1;
    }

    if (fileid >= 3) {
        f = localSD->pd->eml_openfiles[fileid - 3];
    } else if (fileid == 0) {
        f = stdin;
    } else if (fileid == 1) {
        f = stdout;
    } else if (fileid == 2) {
        f = stderr;
    } else {
        f = NULL;
    }

    return f;
}

static signed char filedata(HDMapping *aInstancePtr) {
    constructWorldMapStackData *localSD;
    int k;
    signed char f;
    bool exitg1;
    localSD = aInstancePtr->getStackData();
    f = 0;
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k < 20)) {
        if (localSD->pd->eml_openfiles[k] == NULL) {
            f = static_cast<signed char>(k + 1);
            exitg1 = true;
        } else {
            k++;
        }
    }

    return f;
}

namespace images {
namespace geotrans {
namespace internal {
static void constrainToRotationMatrix2D(const double R[4], double Rc[4],
                                        double *r) {
    double e[2];
    double s[2];
    double R_clamped_idx_0;
    double R_clamped_idx_1;
    double R_clamped_idx_2;
    double R_clamped_idx_3;
    double b;
    double d;
    double f;
    double minval_idx_2;
    double sm;
    double snorm;
    double sqds;
    double wpr;
    bool close_enough;
    R_clamped_idx_0 = std::fmax(std::fmin(R[0], 1.0), -1.0);
    R_clamped_idx_1 = std::fmax(std::fmin(R[1], 1.0), -1.0);
    R_clamped_idx_2 = std::fmax(std::fmin(R[2], 1.0), -1.0);
    R_clamped_idx_3 = std::fmax(std::fmin(R[3], 1.0), -1.0);
    wpr = 57.295779513082323 * rt_atan2d_snf(R_clamped_idx_1,
                                             R_clamped_idx_3) +
          180.0;
    if (std::isnan(wpr) || std::isinf(wpr)) {
        *r = rtNaN;
    } else if (wpr == 0.0) {
        *r = 0.0;
    } else {
        *r = std::fmod(wpr, 360.0);
        if (*r == 0.0) {
            *r = 0.0;
        } else if (wpr < 0.0) {
            *r += 360.0;
        }
    }

    wpr = std::round(*r - 180.0);
    if (*r - 180.0 == wpr) {
        close_enough = true;
    } else {
        d = std::abs((*r - 180.0) - wpr);
        if ((*r - 180.0 == 0.0) || (wpr == 0.0)) {
            close_enough = (d < 4.94065645841247E-324);
        } else {
            b = std::abs(*r - 180.0) + std::abs(wpr);
            if (b < 2.2250738585072014E-308) {
                close_enough = (d < 4.94065645841247E-324);
            } else {
                close_enough = (d / std::fmin(b, 1.7976931348623157E+308) <
                                2.2204460492503131E-16);
            }
        }
    }

    *r -= 180.0;
    if (close_enough) {
        *r = wpr;
    }

    wpr = *r;
    b_sind(&wpr);
    d = *r;
    b_cosd(&d);
    Rc[0] = d;
    Rc[2] = -wpr;
    Rc[1] = wpr;
    Rc[3] = d;
    f = R_clamped_idx_0 - d;
    sqds = R_clamped_idx_1 - wpr;
    minval_idx_2 = R_clamped_idx_2 - (-wpr);
    snorm = R_clamped_idx_3 - d;
    wpr = 0.0;
    b = std::abs(f);
    if (std::isnan(b) || (b > 0.0)) {
        wpr = b;
    }

    sm = std::abs(sqds);
    if (std::isnan(sm) || (sm > wpr)) {
        wpr = sm;
    }

    d = std::abs(minval_idx_2);
    if (std::isnan(d) || (d > wpr)) {
        wpr = d;
    }

    d = std::abs(snorm);
    if (std::isnan(d) || (d > wpr)) {
        wpr = d;
    }

    if ((!std::isinf(wpr)) && (!std::isnan(wpr))) {
        double scale;
        int iter;
        int m;
        scale = 3.3121686421112381E-170;
        if (b > 3.3121686421112381E-170) {
            d = 1.0;
            scale = b;
        } else {
            wpr = b / 3.3121686421112381E-170;
            d = wpr * wpr;
        }

        if (sm > scale) {
            wpr = scale / sm;
            d = d * wpr * wpr + 1.0;
            scale = sm;
        } else {
            wpr = sm / scale;
            d += wpr * wpr;
        }

        d = scale * std::sqrt(d);
        if (d > 0.0) {
            if (f < 0.0) {
                s[0] = -d;
            } else {
                s[0] = d;
            }

            if (std::abs(s[0]) >= 1.0020841800044864E-292) {
                wpr = 1.0 / s[0];
                f *= wpr;
                sqds *= wpr;
            } else {
                f /= s[0];
                sqds /= s[0];
            }

            f++;
            s[0] = -s[0];
            wpr = -((f * minval_idx_2 + sqds * snorm) / f);
            if (!(wpr == 0.0)) {
                minval_idx_2 += wpr * f;
                snorm += wpr * sqds;
            }
        } else {
            s[0] = 0.0;
        }

        m = 2;
        s[1] = snorm;
        e[0] = minval_idx_2;
        e[1] = 0.0;
        if (s[0] != 0.0) {
            d = std::abs(s[0]);
            wpr = s[0] / d;
            s[0] = d;
            e[0] = minval_idx_2 / wpr;
        }

        if (e[0] != 0.0) {
            d = std::abs(e[0]);
            wpr = d / e[0];
            e[0] = d;
            s[1] = snorm * wpr;
        }

        if (s[1] != 0.0) {
            s[1] = std::abs(s[1]);
        }

        iter = 0;
        snorm = std::fmax(std::fmax(s[0], e[0]), std::fmax(s[1], 0.0));
        while ((m > 0) && (iter < 75)) {
            int ii_tmp_tmp;
            int kase;
            int q;
            bool exitg1;
            ii_tmp_tmp = m - 1;
            q = m - 1;
            exitg1 = false;
            while (!(exitg1 || (q == 0))) {
                wpr = std::abs(e[0]);
                if ((wpr <= 2.2204460492503131E-16 * (std::abs(s[0]) + std::
                                                                           abs(s[1]))) ||
                    (wpr <= 1.0020841800044864E-292) ||
                    ((iter > 20) && (wpr <= 2.2204460492503131E-16 * snorm))) {
                    e[0] = 0.0;
                    exitg1 = true;
                } else {
                    q = 0;
                }
            }

            if (q == m - 1) {
                kase = 4;
            } else {
                int qs;
                qs = m;
                kase = m;
                exitg1 = false;
                while ((!exitg1) && (kase >= q)) {
                    qs = kase;
                    if (kase == q) {
                        exitg1 = true;
                    } else {
                        wpr = 0.0;
                        if (kase < m) {
                            wpr = std::abs(e[0]);
                        }

                        if (kase > q + 1) {
                            wpr += std::abs(e[0]);
                        }

                        d = std::abs(s[kase - 1]);
                        if ((d <= 2.2204460492503131E-16 * wpr) || (d <=
                                                                    1.0020841800044864E-292)) {
                            s[kase - 1] = 0.0;
                            exitg1 = true;
                        } else {
                            kase--;
                        }
                    }
                }

                if (qs == q) {
                    kase = 3;
                } else if (qs == m) {
                    kase = 1;
                } else {
                    kase = 2;
                    q = qs;
                }
            }

            switch (kase) {
                case 1:
                    f = e[0];
                    e[0] = 0.0;
                    for (kase = ii_tmp_tmp; kase >= q + 1; kase--) {
                        ::buildMapping::coder::internal::blas::xrotg(&s[0], &f, &d,
                                                                     &sm);
                    }
                    break;

                case 2:
                    f = e[q - 1];
                    e[q - 1] = 0.0;
                    for (kase = q + 1; kase <= m; kase++) {
                        ::buildMapping::coder::internal::blas::xrotg(&s[kase - 1],
                                                                     &f, &d, &sm);
                        wpr = e[kase - 1];
                        f = -sm * wpr;
                        e[kase - 1] = wpr * d;
                    }
                    break;

                case 3:
                    wpr = s[m - 1];
                    scale = std::fmax(std::fmax(std::fmax(std::fmax(std::abs(wpr),
                                                                    std::abs(s[0])),
                                                          std::abs(e[0])),
                                                std::abs(s[q])),
                                      std::abs(e[q]));
                    sm = wpr / scale;
                    wpr = s[0] / scale;
                    d = e[0] / scale;
                    sqds = s[q] / scale;
                    b = ((wpr + sm) * (wpr - sm) + d * d) / 2.0;
                    wpr = sm * d;
                    wpr *= wpr;
                    if ((b != 0.0) || (wpr != 0.0)) {
                        d = std::sqrt(b * b + wpr);
                        if (b < 0.0) {
                            d = -d;
                        }

                        d = wpr / (b + d);
                    } else {
                        d = 0.0;
                    }

                    f = (sqds + sm) * (sqds - sm) + d;
                    wpr = sqds * (e[q] / scale);
                    for (kase = q + 1; kase < 2; kase++) {
                        ::buildMapping::coder::internal::blas::xrotg(&f, &wpr, &d,
                                                                     &sm);
                        f = d * s[0] + sm * e[0];
                        b = d * e[0] - sm * s[0];
                        e[0] = b;
                        wpr = sm * s[1];
                        s[1] *= d;
                        s[0] = f;
                        ::buildMapping::coder::internal::blas::xrotg(&s[0], &wpr, &d,
                                                                     &sm);
                        f = d * b + sm * s[1];
                        s[1] = -sm * b + d * s[1];
                        wpr = sm * e[1];
                        e[1] *= d;
                    }

                    e[0] = f;
                    iter++;
                    break;

                default:
                    if (s[q] < 0.0) {
                        s[q] = -s[q];
                    }

                    while ((q + 1 < 2) && (s[0] < s[1])) {
                        d = s[0];
                        s[0] = s[1];
                        s[1] = d;
                        q = 1;
                    }

                    iter = 0;
                    m--;
                    break;
            }
        }

        wpr = s[0];
    }

    if (wpr / 2.2204460492503131E-16 < 10.0) {
        Rc[0] = R_clamped_idx_0;
        Rc[1] = R_clamped_idx_1;
        Rc[2] = R_clamped_idx_2;
        Rc[3] = R_clamped_idx_3;
    }
}
}  // namespace internal
}  // namespace geotrans

namespace internal {
namespace coder {
static void interp2_local(const float V[307200], const ::coder::array<double, 2U> &Xq, const ::coder::array<double, 2U> &Yq, ::coder::array<float, 2U> &Vq) {
    double rx;
    float qx1;
    float qx2;
    int ix;
    int iy;
    int ub_loop;
    Vq.set_size(Xq.size(0), Xq.size(1));
    ub_loop = Xq.size(0) * Xq.size(1) - 1;

#pragma omp parallel for num_threads(omp_get_max_threads()) private(ix, iy, qx1, qx2, rx)

    for (int k = 0; k <= ub_loop; k++) {
        if ((Xq[k] >= 1.0) && (Xq[k] <= 640.0) && (Yq[k] >= 1.0) && (Yq[k] <= 480.0)) {
            if (Xq[k] <= 1.0) {
                ix = 1;
            } else if (Xq[k] <= 639.0) {
                ix = static_cast<int>(std::floor(Xq[k]));
            } else {
                ix = 639;
            }

            if (Yq[k] <= 1.0) {
                iy = 1;
            } else if (Yq[k] <= 479.0) {
                iy = static_cast<int>(std::floor(Yq[k]));
            } else {
                iy = 479;
            }

            if (Xq[k] == ix) {
                ix = iy + 480 * (ix - 1);
                qx1 = V[ix - 1];
                qx2 = V[ix];
            } else if (Xq[k] == static_cast<double>(ix) + 1.0) {
                ix = iy + 480 * ix;
                qx1 = V[ix - 1];
                qx2 = V[ix];
            } else {
                rx = (Xq[k] - static_cast<double>(ix)) / ((static_cast<double>(ix) + 1.0) - static_cast<double>(ix));
                if (V[(iy + 480 * (ix - 1)) - 1] == V[(iy + 480 * ix) - 1]) {
                    qx1 = V[(iy + 480 * (ix - 1)) - 1];
                } else {
                    qx1 = static_cast<float>(1.0 - rx) * V[(iy + 480 * (ix - 1)) - 1] + static_cast<float>(rx) * V[(iy + 480 * ix) - 1];
                }

                if (V[iy + 480 * (ix - 1)] == V[iy + 480 * ix]) {
                    qx2 = V[iy + 480 * (ix - 1)];
                } else {
                    qx2 = static_cast<float>(1.0 - rx) * V[iy + 480 * (ix - 1)] + static_cast<float>(rx) * V[iy + 480 * ix];
                }
            }

            if ((Yq[k] == iy) || (qx1 == qx2)) {
                Vq[k] = qx1;
            } else if (Yq[k] == static_cast<double>(iy) + 1.0) {
                Vq[k] = qx2;
            } else {
                rx = (Yq[k] - static_cast<double>(iy)) / ((static_cast<double>(iy) + 1.0) - static_cast<double>(iy));
                Vq[k] = static_cast<float>(1.0 - rx) * qx1 + static_cast<float>(rx) * qx2;
            }
        } else {
            Vq[k] = 0.0F;
        }
    }
}

namespace optimized {
static void b_remapAndResampleGeneric2d(HDMapping *aInstancePtr,
                                        const unsigned char inputImage[307200], const rigidtform2d tform,
                                        const imref2d outputRef, ::coder::array<unsigned char, 2U> &outputImage) {
    ::coder::array<double, 2U> srcXIntrinsic;
    ::coder::array<double, 2U> srcYIntrinsic;
    ::coder::array<float, 2U> b_outputImage;
    ::coder::array<float, 2U> r;
    constructWorldMapStackData *localSD;
    double b_r1[9];
    double tinv[9];
    double dstXWorld_val;
    double r1;
    double r2;
    double srcXWorld_val;
    double srcYWorld_val;
    int i1;
    int rowIdx;
    int ub_loop;
    localSD = aInstancePtr->getStackData();
    r1 = tform.RotationAngle;
    b_cosd(&r1);
    r2 = tform.RotationAngle;
    b_sind(&r2);
    b_r1[0] = r1;
    b_r1[1] = -r2;
    b_r1[2] = tform.Translation[0];
    b_r1[3] = r2;
    b_r1[4] = r1;
    b_r1[5] = tform.Translation[1];
    b_r1[6] = 0.0;
    b_r1[7] = 0.0;
    b_r1[8] = 1.0;
    inv(b_r1, tinv);
    r1 = outputRef.ImageSizeAlias[0];
    srcXIntrinsic.set_size(static_cast<int>(outputRef.ImageSizeAlias[0]),
                           static_cast<int>(outputRef.ImageSizeAlias[1]));
    srcYIntrinsic.set_size(static_cast<int>(outputRef.ImageSizeAlias[0]),
                           static_cast<int>(outputRef.ImageSizeAlias[1]));
    ub_loop = static_cast<int>(outputRef.ImageSizeAlias[1]) - 1;

#pragma omp parallel for num_threads(omp_get_max_threads()) private(srcYWorld_val, srcXWorld_val, dstXWorld_val, i1, rowIdx)

    for (int colIdx = 0; colIdx <= ub_loop; colIdx++) {
        dstXWorld_val = outputRef.XWorldLimits[0] + ((static_cast<double>(colIdx) + 1.0) - 0.5) * ((outputRef.XWorldLimits[1] -
                                                                                                    outputRef.XWorldLimits[0]) /
                                                                                                   outputRef.ImageSizeAlias[1]);
        i1 = static_cast<int>(r1);
        for (rowIdx = 0; rowIdx < i1; rowIdx++) {
            srcYWorld_val = outputRef.YWorldLimits[0] + ((static_cast<
                                                              double>(rowIdx) +
                                                          1.0) -
                                                         0.5) *
                                                            ((outputRef.YWorldLimits[1] - outputRef.YWorldLimits[0]) / outputRef.ImageSizeAlias[0]);
            srcXWorld_val = (tinv[0] * dstXWorld_val + tinv[1] *
                                                           srcYWorld_val) +
                            tinv[2];
            srcYWorld_val = (tinv[3] * dstXWorld_val + tinv[4] *
                                                           srcYWorld_val) +
                            tinv[5];
            srcXIntrinsic[rowIdx + srcXIntrinsic.size(0) * colIdx] =
                (srcXWorld_val - 0.5) + 0.5;
            srcYIntrinsic[rowIdx + srcYIntrinsic.size(0) * colIdx] =
                (srcYWorld_val - 0.5) + 0.5;
        }
    }

    b_outputImage.set_size(srcXIntrinsic.size(0), srcXIntrinsic.size(1));
    ub_loop = srcXIntrinsic.size(0) * srcXIntrinsic.size(1);
    for (int i{0}; i < ub_loop; i++) {
        b_outputImage[i] = 0.0F;
    }

    for (int i{0}; i < 307200; i++) {
        localSD->f1.inputImage[i] = inputImage[i];
    }

    interp2_local(localSD->f1.inputImage, srcXIntrinsic, srcYIntrinsic,
                  r);
    ub_loop = r.size(1);
    for (int i{0}; i < ub_loop; i++) {
        int loop_ub;
        loop_ub = r.size(0);
        for (int i2{0}; i2 < loop_ub; i2++) {
            b_outputImage[i2 + b_outputImage.size(0) * i] = r[i2 + r.size(0) * i];
        }
    }

    outputImage.set_size(b_outputImage.size(0), b_outputImage.size(1));
    ub_loop = b_outputImage.size(0) * b_outputImage.size(1);
    for (int i{0}; i < ub_loop; i++) {
        outputImage[i] = static_cast<unsigned char>(std::round(b_outputImage[i]));
    }
}

static void c_remapAndResampleGeneric2d(HDMapping *aInstancePtr,
                                        const bool inputImage[307200], const rigidtform2d tform, const imref2d outputRef, ::coder::array<bool, 2U> &outputImage) {
    ::coder::array<double, 2U> srcXIntrinsic;
    ::coder::array<double, 2U> srcYIntrinsic;
    ::coder::array<float, 2U> b_outputImage;
    ::coder::array<float, 2U> r;
    constructWorldMapStackData *localSD;
    double b_r1[9];
    double tinv[9];
    double dstXWorld_val;
    double r1;
    double r2;
    double srcXWorld_val;
    double srcYWorld_val;
    int i1;
    int rowIdx;
    int ub_loop;
    localSD = aInstancePtr->getStackData();
    r1 = tform.RotationAngle;
    b_cosd(&r1);
    r2 = tform.RotationAngle;
    b_sind(&r2);
    b_r1[0] = r1;
    b_r1[1] = -r2;
    b_r1[2] = tform.Translation[0];
    b_r1[3] = r2;
    b_r1[4] = r1;
    b_r1[5] = tform.Translation[1];
    b_r1[6] = 0.0;
    b_r1[7] = 0.0;
    b_r1[8] = 1.0;
    inv(b_r1, tinv);
    r1 = outputRef.ImageSizeAlias[0];
    srcXIntrinsic.set_size(static_cast<int>(outputRef.ImageSizeAlias[0]),
                           static_cast<int>(outputRef.ImageSizeAlias[1]));
    srcYIntrinsic.set_size(static_cast<int>(outputRef.ImageSizeAlias[0]),
                           static_cast<int>(outputRef.ImageSizeAlias[1]));
    ub_loop = static_cast<int>(outputRef.ImageSizeAlias[1]) - 1;

#pragma omp parallel for num_threads(omp_get_max_threads()) private(srcYWorld_val, srcXWorld_val, dstXWorld_val, i1, rowIdx)

    for (int colIdx = 0; colIdx <= ub_loop; colIdx++) {
        dstXWorld_val = outputRef.XWorldLimits[0] + ((static_cast<double>(colIdx) + 1.0) - 0.5) * ((outputRef.XWorldLimits[1] -
                                                                                                    outputRef.XWorldLimits[0]) /
                                                                                                   outputRef.ImageSizeAlias[1]);
        i1 = static_cast<int>(r1);
        for (rowIdx = 0; rowIdx < i1; rowIdx++) {
            srcYWorld_val = outputRef.YWorldLimits[0] + ((static_cast<
                                                              double>(rowIdx) +
                                                          1.0) -
                                                         0.5) *
                                                            ((outputRef.YWorldLimits[1] - outputRef.YWorldLimits[0]) / outputRef.ImageSizeAlias[0]);
            srcXWorld_val = (tinv[0] * dstXWorld_val + tinv[1] *
                                                           srcYWorld_val) +
                            tinv[2];
            srcYWorld_val = (tinv[3] * dstXWorld_val + tinv[4] *
                                                           srcYWorld_val) +
                            tinv[5];
            srcXIntrinsic[rowIdx + srcXIntrinsic.size(0) * colIdx] =
                (srcXWorld_val - 0.5) + 0.5;
            srcYIntrinsic[rowIdx + srcYIntrinsic.size(0) * colIdx] =
                (srcYWorld_val - 0.5) + 0.5;
        }
    }

    b_outputImage.set_size(srcXIntrinsic.size(0), srcXIntrinsic.size(1));
    ub_loop = srcXIntrinsic.size(0) * srcXIntrinsic.size(1);
    for (int i{0}; i < ub_loop; i++) {
        b_outputImage[i] = 0.0F;
    }

    for (int i{0}; i < 307200; i++) {
        localSD->f0.inputImage[i] = inputImage[i];
    }

    interp2_local(localSD->f0.inputImage, srcXIntrinsic, srcYIntrinsic,
                  r);
    ub_loop = r.size(1);
    for (int i{0}; i < ub_loop; i++) {
        int loop_ub;
        loop_ub = r.size(0);
        for (int i2{0}; i2 < loop_ub; i2++) {
            b_outputImage[i2 + b_outputImage.size(0) * i] = r[i2 + r.size(0) * i];
        }
    }

    outputImage.set_size(b_outputImage.size(0), b_outputImage.size(1));
    ub_loop = b_outputImage.size(0) * b_outputImage.size(1);
    for (int i{0}; i < ub_loop; i++) {
        outputImage[i] = (b_outputImage[i] > 0.5F);
    }
}
}  // namespace optimized
}  // namespace coder
}  // namespace internal
}  // namespace images

static void inpolygon(const ::coder::array<float, 1U> &x, const ::coder::array<float, 1U> &y, ::coder::array<bool, 1U> &in) {
    static const double scale[4]{4.5191517195064534E-11,
                                 3.4211966593034049E-11, 5.7231885897124357E-11, 6.8211436499154843E-11};

    static const double xv[4]{281.5, 281.5, 356.5, 356.5};

    static const short yv[4]{321, 161, 161, 321};

    double xv1;
    double xv2;
    double yv1;
    double yv2;
    float xj;
    float yj;
    int b_i;
    int exitg1;
    int loop_ub;
    signed char dquad;
    signed char quad1;
    signed char quad2;
    signed char quadFirst;
    signed char sdq;
    bool onj;
    in.set_size(x.size(0));
    loop_ub = x.size(0);
    for (int i{0}; i < loop_ub; i++) {
        in[i] = false;
    }

    if (x.size(0) != 0) {
        loop_ub = x.size(0) - 1;

#pragma omp parallel for num_threads(omp_get_max_threads()) private(xj, yj, sdq, quadFirst, xv2, yv2, quad2, b_i, exitg1, dquad, onj, xv1, yv1, quad1)

        for (int j = 0; j <= loop_ub; j++) {
            xj = x[j];
            yj = y[j];
            in[j] = false;
            if ((xj >= 281.5F) && (xj <= 356.5F) && (yj >= 161.0F) && (yj <= 321.0F)) {
                sdq = 0;
                if (321.0 - yj > 0.0) {
                    quadFirst = 1;
                } else {
                    quadFirst = 2;
                }

                xv2 = 281.5 - xj;
                yv2 = 321.0 - yj;
                quad2 = quadFirst;
                b_i = 1;
                do {
                    exitg1 = 0;
                    if (b_i < 4) {
                        xv1 = xv2;
                        yv1 = yv2;
                        xv2 = xv[b_i] - xj;
                        yv2 = static_cast<double>(yv[b_i]) - yj;
                        quad1 = quad2;
                        if (xv2 > 0.0) {
                            if (yv2 > 0.0) {
                                quad2 = 0;
                            } else {
                                quad2 = 3;
                            }
                        } else if (yv2 > 0.0) {
                            quad2 = 1;
                        } else {
                            quad2 = 2;
                        }

                        contrib(xv1, yv1, xv2, yv2, quad1, quad2, scale[b_i - 1], &dquad,
                                &onj);
                        if (onj) {
                            in[j] = true;
                            exitg1 = 1;
                        } else {
                            sdq = static_cast<signed char>(sdq + dquad);
                            b_i++;
                        }
                    } else {
                        contrib(xv2, yv2, 281.5 - xj, 321.0 - yj, quad2, quadFirst,
                                6.8211436499154843E-11, &dquad, &onj);
                        if (onj) {
                            in[j] = true;
                        } else {
                            sdq = static_cast<signed char>(sdq + dquad);
                            in[j] = (sdq != 0);
                        }

                        exitg1 = 1;
                    }
                } while (exitg1 == 0);
            }
        }
    }
}

namespace internal {
static double applyToMultipleDims(const ::coder::array<bool, 2U> &x) {
    double varargout_1;
    if (x.size(0) * x.size(1) == 0) {
        varargout_1 = 0.0;
    } else {
        int firstBlockLength;
        int lastBlockLength;
        int nblocks;
        if (x.size(0) * x.size(1) <= 1024) {
            firstBlockLength = x.size(0) * x.size(1);
            lastBlockLength = 0;
            nblocks = 1;
        } else {
            firstBlockLength = 1024;
            nblocks = static_cast<int>(static_cast<unsigned int>(x.size(0) *
                                                                 x.size(1)) >>
                                       10);
            lastBlockLength = x.size(0) * x.size(1) - (nblocks << 10);
            if (lastBlockLength > 0) {
                nblocks++;
            } else {
                lastBlockLength = 1024;
            }
        }

        varargout_1 = x[0];
        for (int k{2}; k <= firstBlockLength; k++) {
            varargout_1 += static_cast<double>(x[k - 1]);
        }

        for (int ib{2}; ib <= nblocks; ib++) {
            double bsum;
            int hi;
            firstBlockLength = (ib - 1) << 10;
            bsum = x[firstBlockLength];
            if (ib == nblocks) {
                hi = lastBlockLength;
            } else {
                hi = 1024;
            }

            for (int k{2}; k <= hi; k++) {
                bsum += static_cast<double>(x[(firstBlockLength + k) - 1]);
            }

            varargout_1 += bsum;
        }
    }

    return varargout_1;
}

static void b_heapsort(::coder::array<int, 1U> &x, int xstart, int xend,
                       const anonymous_function *cmp) {
    int idx;
    int n;
    n = (xend - xstart) - 1;
    for (idx = n + 2; idx >= 1; idx--) {
        heapify(x, idx, xstart, xend, cmp);
    }

    for (int k{0}; k <= n; k++) {
        int t;
        idx = (xend - k) - 1;
        t = x[idx];
        x[idx] = x[xstart - 1];
        x[xstart - 1] = t;
        heapify(x, 1, xstart, (xend - k) - 1, cmp);
    }
}

static void b_svd(const double A[9], double U[9], double s[3], double V[9]) {
    double b_A[9];
    double b_s[3];
    double e[3];
    double work[3];
    double nrm;
    double rt;
    double sm;
    double snorm;
    double sqds;
    int ii;
    int kase;
    int m;
    int qjj;
    int qp1;
    int qq;
    b_s[0] = 0.0;
    e[0] = 0.0;
    work[0] = 0.0;
    b_s[1] = 0.0;
    e[1] = 0.0;
    work[1] = 0.0;
    b_s[2] = 0.0;
    e[2] = 0.0;
    work[2] = 0.0;
    for (kase = 0; kase < 9; kase++) {
        b_A[kase] = A[kase];
        U[kase] = 0.0;
        V[kase] = 0.0;
    }

    for (int q{0}; q < 2; q++) {
        bool apply_transform;
        qp1 = q + 2;
        qq = (q + 3 * q) + 1;
        apply_transform = false;
        nrm = blas::xnrm2(3 - q, b_A, qq);
        if (nrm > 0.0) {
            apply_transform = true;
            if (b_A[qq - 1] < 0.0) {
                nrm = -nrm;
            }

            b_s[q] = nrm;
            if (std::abs(nrm) >= 1.0020841800044864E-292) {
                nrm = 1.0 / nrm;
                kase = (qq - q) + 2;
                for (int k{qq}; k <= kase; k++) {
                    b_A[k - 1] *= nrm;
                }
            } else {
                kase = (qq - q) + 2;
                for (int k{qq}; k <= kase; k++) {
                    b_A[k - 1] /= b_s[q];
                }
            }

            b_A[qq - 1]++;
            b_s[q] = -b_s[q];
        } else {
            b_s[q] = 0.0;
        }

        for (kase = qp1; kase < 4; kase++) {
            qjj = q + 3 * (kase - 1);
            if (apply_transform) {
                blas::xaxpy(3 - q, -(blas::xdotc(3 - q, b_A, qq, b_A, qjj + 1) / b_A[q + 3 * q]), qq, b_A, qjj + 1);
            }

            e[kase - 1] = b_A[qjj];
        }

        for (ii = q + 1; ii < 4; ii++) {
            kase = (ii + 3 * q) - 1;
            U[kase] = b_A[kase];
        }

        if (q + 1 <= 1) {
            nrm = blas::b_xnrm2(e);
            if (nrm == 0.0) {
                e[0] = 0.0;
            } else {
                if (e[1] < 0.0) {
                    e[0] = -nrm;
                } else {
                    e[0] = nrm;
                }

                nrm = e[0];
                if (std::abs(e[0]) >= 1.0020841800044864E-292) {
                    nrm = 1.0 / e[0];
                    for (int k{qp1}; k < 4; k++) {
                        e[k - 1] *= nrm;
                    }
                } else {
                    for (int k{qp1}; k < 4; k++) {
                        e[k - 1] /= nrm;
                    }
                }

                e[1]++;
                e[0] = -e[0];
                for (ii = qp1; ii < 4; ii++) {
                    work[ii - 1] = 0.0;
                }

                for (kase = qp1; kase < 4; kase++) {
                    blas::xaxpy(e[kase - 1], b_A, 3 * (kase - 1) + 2, work);
                }

                for (kase = qp1; kase < 4; kase++) {
                    blas::xaxpy(-e[kase - 1] / e[1], work, b_A, 3 * (kase - 1) + 2);
                }
            }

            for (ii = qp1; ii < 4; ii++) {
                V[ii - 1] = e[ii - 1];
            }
        }
    }

    m = 1;
    b_s[2] = b_A[8];
    e[1] = b_A[7];
    e[2] = 0.0;
    U[6] = 0.0;
    U[7] = 0.0;
    U[8] = 1.0;
    for (int q{1}; q >= 0; q--) {
        qp1 = q + 2;
        qq = q + 3 * q;
        if (b_s[q] != 0.0) {
            for (kase = qp1; kase < 4; kase++) {
                qjj = (q + 3 * (kase - 1)) + 1;
                blas::xaxpy(3 - q, -(blas::xdotc(3 - q, U, qq + 1, U, qjj) / U[qq]),
                            qq + 1, U, qjj);
            }

            for (ii = q + 1; ii < 4; ii++) {
                kase = (ii + 3 * q) - 1;
                U[kase] = -U[kase];
            }

            U[qq]++;
            if (q - 1 >= 0) {
                U[3 * q] = 0.0;
            }
        } else {
            U[3 * q] = 0.0;
            U[3 * q + 1] = 0.0;
            U[3 * q + 2] = 0.0;
            U[qq] = 1.0;
        }
    }

    for (int q{2}; q >= 0; q--) {
        if ((q + 1 <= 1) && (e[0] != 0.0)) {
            blas::xaxpy(2, -(blas::xdotc(2, V, 2, V, 5) / V[1]), 2, V, 5);
            blas::xaxpy(2, -(blas::xdotc(2, V, 2, V, 8) / V[1]), 2, V, 8);
        }

        V[3 * q] = 0.0;
        V[3 * q + 1] = 0.0;
        V[3 * q + 2] = 0.0;
        V[q + 3 * q] = 1.0;
    }

    qq = 0;
    snorm = 0.0;
    for (int q{0}; q < 3; q++) {
        nrm = b_s[q];
        if (nrm != 0.0) {
            rt = std::abs(nrm);
            nrm /= rt;
            b_s[q] = rt;
            if (q + 1 < 3) {
                e[q] /= nrm;
            }

            qjj = 3 * q;
            kase = qjj + 3;
            for (int k{qjj + 1}; k <= kase; k++) {
                U[k - 1] *= nrm;
            }
        }

        if (q + 1 < 3) {
            nrm = e[q];
            if (nrm != 0.0) {
                rt = std::abs(nrm);
                nrm = rt / nrm;
                e[q] = rt;
                b_s[q + 1] *= nrm;
                qjj = 3 * (q + 1);
                kase = qjj + 3;
                for (int k{qjj + 1}; k <= kase; k++) {
                    V[k - 1] *= nrm;
                }
            }
        }

        snorm = std::fmax(snorm, std::fmax(std::abs(b_s[q]), std::abs(e[q])));
    }

    while ((m + 2 > 0) && (qq < 75)) {
        bool exitg1;
        qp1 = m + 1;
        ii = m + 1;
        exitg1 = false;
        while (!(exitg1 || (ii == 0))) {
            nrm = std::abs(e[ii - 1]);
            if ((nrm <= 2.2204460492503131E-16 * (std::abs(b_s[ii - 1]) + std::
                                                                              abs(b_s[ii]))) ||
                (nrm <= 1.0020841800044864E-292) || ((qq > 20) && (nrm <= 2.2204460492503131E-16 * snorm))) {
                e[ii - 1] = 0.0;
                exitg1 = true;
            } else {
                ii--;
            }
        }

        if (ii == m + 1) {
            kase = 4;
        } else {
            qjj = m + 2;
            kase = m + 2;
            exitg1 = false;
            while ((!exitg1) && (kase >= ii)) {
                qjj = kase;
                if (kase == ii) {
                    exitg1 = true;
                } else {
                    nrm = 0.0;
                    if (kase < m + 2) {
                        nrm = std::abs(e[kase - 1]);
                    }

                    if (kase > ii + 1) {
                        nrm += std::abs(e[kase - 2]);
                    }

                    rt = std::abs(b_s[kase - 1]);
                    if ((rt <= 2.2204460492503131E-16 * nrm) || (rt <=
                                                                 1.0020841800044864E-292)) {
                        b_s[kase - 1] = 0.0;
                        exitg1 = true;
                    } else {
                        kase--;
                    }
                }
            }

            if (qjj == ii) {
                kase = 3;
            } else if (qjj == m + 2) {
                kase = 1;
            } else {
                kase = 2;
                ii = qjj;
            }
        }

        switch (kase) {
            case 1:
                rt = e[m];
                e[m] = 0.0;
                for (int k{qp1}; k >= ii + 1; k--) {
                    blas::xrotg(&b_s[k - 1], &rt, &sm, &sqds);
                    if (k > ii + 1) {
                        rt = -sqds * e[0];
                        e[0] *= sm;
                    }

                    blas::xrot(V, 3 * (k - 1) + 1, 3 * (m + 1) + 1, sm, sqds);
                }
                break;

            case 2: {
                rt = e[ii - 1];
                e[ii - 1] = 0.0;
                for (int k{ii + 1}; k <= m + 2; k++) {
                    double b;
                    blas::xrotg(&b_s[k - 1], &rt, &sm, &sqds);
                    b = e[k - 1];
                    rt = -sqds * b;
                    e[k - 1] = b * sm;
                    blas::xrot(U, 3 * (k - 1) + 1, 3 * (ii - 1) + 1, sm, sqds);
                }
            } break;

            case 3: {
                double b;
                double scale;
                nrm = b_s[m + 1];
                scale = std::fmax(std::fmax(std::fmax(std::fmax(std::abs(nrm), std::
                                                                                   abs(b_s[m])),
                                                      std::abs(e[m])),
                                            std::abs(b_s[ii])),
                                  std::abs(e[ii]));
                sm = nrm / scale;
                nrm = b_s[m] / scale;
                rt = e[m] / scale;
                sqds = b_s[ii] / scale;
                b = ((nrm + sm) * (nrm - sm) + rt * rt) / 2.0;
                nrm = sm * rt;
                nrm *= nrm;
                if ((b != 0.0) || (nrm != 0.0)) {
                    rt = std::sqrt(b * b + nrm);
                    if (b < 0.0) {
                        rt = -rt;
                    }

                    rt = nrm / (b + rt);
                } else {
                    rt = 0.0;
                }

                rt += (sqds + sm) * (sqds - sm);
                nrm = sqds * (e[ii] / scale);
                for (int k{ii + 1}; k <= qp1; k++) {
                    blas::xrotg(&rt, &nrm, &sm, &sqds);
                    if (k > ii + 1) {
                        e[0] = rt;
                    }

                    nrm = e[k - 1];
                    b = b_s[k - 1];
                    e[k - 1] = sm * nrm - sqds * b;
                    rt = sqds * b_s[k];
                    b_s[k] *= sm;
                    blas::xrot(V, 3 * (k - 1) + 1, 3 * k + 1, sm, sqds);
                    b_s[k - 1] = sm * b + sqds * nrm;
                    blas::xrotg(&b_s[k - 1], &rt, &sm, &sqds);
                    rt = sm * e[k - 1] + sqds * b_s[k];
                    b_s[k] = -sqds * e[k - 1] + sm * b_s[k];
                    nrm = sqds * e[k];
                    e[k] *= sm;
                    blas::xrot(U, 3 * (k - 1) + 1, 3 * k + 1, sm, sqds);
                }

                e[m] = rt;
                qq++;
            } break;

            default:
                if (b_s[ii] < 0.0) {
                    b_s[ii] = -b_s[ii];
                    qjj = 3 * ii;
                    kase = qjj + 3;
                    for (int k{qjj + 1}; k <= kase; k++) {
                        V[k - 1] = -V[k - 1];
                    }
                }

                qp1 = ii + 1;
                while ((ii + 1 < 3) && (b_s[ii] < b_s[qp1])) {
                    rt = b_s[ii];
                    b_s[ii] = b_s[qp1];
                    b_s[qp1] = rt;
                    blas::xswap(V, 3 * ii + 1, 3 * (ii + 1) + 1);
                    blas::xswap(U, 3 * ii + 1, 3 * (ii + 1) + 1);
                    ii = qp1;
                    qp1++;
                }

                qq = 0;
                m--;
                break;
        }
    }

    s[0] = b_s[0];
    s[1] = b_s[1];
    s[2] = b_s[2];
}

namespace blas {
static double b_xnrm2(const double x[3]) {
    double scale;
    double y;
    y = 0.0;
    scale = 3.3121686421112381E-170;
    for (int k{2}; k < 4; k++) {
        double absxk;
        absxk = std::abs(x[k - 1]);
        if (absxk > scale) {
            double t;
            t = scale / absxk;
            y = y * t * t + 1.0;
            scale = absxk;
        } else {
            double t;
            t = absxk / scale;
            y += t * t;
        }
    }

    return scale * std::sqrt(y);
}

static void mtimes(const double A_data[], const int A_size[2], const double B_data[], const int B_size[2], double C_data[],
                   int C_size[2]) {
    int inner;
    int mc;
    int nc;
    mc = A_size[0];
    inner = A_size[1];
    nc = B_size[1];
    C_size[0] = A_size[0];
    C_size[1] = B_size[1];
    for (int j{0}; j < nc; j++) {
        int boffset;
        int coffset;
        coffset = j * mc;
        boffset = j * B_size[0];
        std::memset(&C_data[coffset], 0, static_cast<unsigned int>((mc + coffset) - coffset) * sizeof(double));
        for (int k{0}; k < inner; k++) {
            double bkj;
            int aoffset;
            aoffset = k * A_size[0];
            bkj = B_data[boffset + k];
            for (int i{0}; i < mc; i++) {
                int b_i;
                b_i = coffset + i;
                C_data[b_i] += A_data[aoffset + i] * bkj;
            }
        }
    }
}

static void mtimes(const double A_data[], const int A_size[2], const ::coder::array<double, 2U> &B, double C_data[], int C_size[2]) {
    int inner;
    int mc;
    int nc;
    mc = A_size[1];
    inner = A_size[0];
    nc = B.size(1);
    C_size[0] = A_size[1];
    C_size[1] = B.size(1);
    for (int j{0}; j < nc; j++) {
        int boffset;
        int coffset;
        coffset = j * mc;
        boffset = j * B.size(0);
        std::memset(&C_data[coffset], 0, static_cast<unsigned int>((mc + coffset) - coffset) * sizeof(double));
        for (int k{0}; k < inner; k++) {
            double bkj;
            bkj = B[boffset + k];
            for (int i{0}; i < mc; i++) {
                int b_i;
                b_i = coffset + i;
                C_data[b_i] += A_data[i * A_size[0] + k] * bkj;
            }
        }
    }
}

static void mtimes(const ::coder::array<double, 2U> &A, const double B[9],
                   ::coder::array<double, 2U> &C) {
    int inner;
    int mc;
    mc = A.size(0);
    inner = A.size(1);
    C.set_size(A.size(0), 3);
    for (int j{0}; j < 3; j++) {
        int boffset;
        int coffset;
        coffset = j * mc;
        boffset = j * 3;
        for (int i{0}; i < mc; i++) {
            C[coffset + i] = 0.0;
        }

        for (int k{0}; k < inner; k++) {
            double bkj;
            int aoffset;
            aoffset = k * A.size(0);
            bkj = B[boffset + k];
            for (int i{0}; i < mc; i++) {
                int b_i;
                b_i = coffset + i;
                C[b_i] = C[b_i] + A[aoffset + i] * bkj;
            }
        }
    }
}

static void xaxpy(double a, const double x[9], int ix0, double y[3]) {
    if (!(a == 0.0)) {
        for (int k{0}; k < 2; k++) {
            y[k + 1] += a * x[(ix0 + k) - 1];
        }
    }
}

static void xaxpy(int n, double a, int ix0, double y[9], int iy0) {
    if (!(a == 0.0)) {
        int i;
        i = n - 1;
        for (int k{0}; k <= i; k++) {
            int i1;
            i1 = (iy0 + k) - 1;
            y[i1] += a * y[(ix0 + k) - 1];
        }
    }
}

static void xaxpy(double a, const double x[3], double y[9], int iy0) {
    if (!(a == 0.0)) {
        for (int k{0}; k < 2; k++) {
            int i;
            i = (iy0 + k) - 1;
            y[i] += a * x[k + 1];
        }
    }
}

static double xdotc(int n, const double x[9], int ix0, const double y[9],
                    int iy0) {
    double d;
    int i;
    d = 0.0;
    i = static_cast<unsigned char>(n);
    for (int k{0}; k < i; k++) {
        d += x[(ix0 + k) - 1] * y[(iy0 + k) - 1];
    }

    return d;
}

static double xnrm2(int n, const double x[9], int ix0) {
    double scale;
    double y;
    int kend;
    y = 0.0;
    scale = 3.3121686421112381E-170;
    kend = (ix0 + n) - 1;
    for (int k{ix0}; k <= kend; k++) {
        double absxk;
        absxk = std::abs(x[k - 1]);
        if (absxk > scale) {
            double t;
            t = scale / absxk;
            y = y * t * t + 1.0;
            scale = absxk;
        } else {
            double t;
            t = absxk / scale;
            y += t * t;
        }
    }

    return scale * std::sqrt(y);
}

static double xnrm2(const double x[4]) {
    double absxk;
    double scale;
    double t;
    double y;
    scale = 3.3121686421112381E-170;
    absxk = std::abs(x[0]);
    if (absxk > 3.3121686421112381E-170) {
        y = 1.0;
        scale = absxk;
    } else {
        t = absxk / 3.3121686421112381E-170;
        y = t * t;
    }

    absxk = std::abs(x[1]);
    if (absxk > scale) {
        t = scale / absxk;
        y = y * t * t + 1.0;
        scale = absxk;
    } else {
        t = absxk / scale;
        y += t * t;
    }

    return scale * std::sqrt(y);
}

static void xrot(double x[9], int ix0, int iy0, double c, double s) {
    double temp;
    double temp_tmp;
    temp = x[iy0 - 1];
    temp_tmp = x[ix0 - 1];
    x[iy0 - 1] = c * temp - s * temp_tmp;
    x[ix0 - 1] = c * temp_tmp + s * temp;
    temp = c * x[ix0] + s * x[iy0];
    x[iy0] = c * x[iy0] - s * x[ix0];
    x[ix0] = temp;
    temp = x[iy0 + 1];
    temp_tmp = x[ix0 + 1];
    x[iy0 + 1] = c * temp - s * temp_tmp;
    x[ix0 + 1] = c * temp_tmp + s * temp;
}

static void xrotg(double *a, double *b, double *c, double *s) {
    double absa;
    double absb;
    double roe;
    double scale;
    roe = *b;
    absa = std::abs(*a);
    absb = std::abs(*b);
    if (absa > absb) {
        roe = *a;
    }

    scale = absa + absb;
    if (scale == 0.0) {
        *s = 0.0;
        *c = 1.0;
        *a = 0.0;
        *b = 0.0;
    } else {
        double ads;
        double bds;
        ads = absa / scale;
        bds = absb / scale;
        scale *= std::sqrt(ads * ads + bds * bds);
        if (roe < 0.0) {
            scale = -scale;
        }

        *c = *a / scale;
        *s = *b / scale;
        if (absa > absb) {
            *b = *s;
        } else if (*c != 0.0) {
            *b = 1.0 / *c;
        } else {
            *b = 1.0;
        }

        *a = scale;
    }
}

static void xswap(double x[9], int ix0, int iy0) {
    double temp;
    temp = x[ix0 - 1];
    x[ix0 - 1] = x[iy0 - 1];
    x[iy0 - 1] = temp;
    temp = x[ix0];
    x[ix0] = x[iy0];
    x[iy0] = temp;
    temp = x[ix0 + 1];
    x[ix0 + 1] = x[iy0 + 1];
    x[iy0 + 1] = temp;
}

static void xswap(double x[4]) {
    double temp;
    temp = x[0];
    x[0] = x[2];
    x[2] = temp;
    temp = x[1];
    x[1] = x[3];
    x[3] = temp;
}
}  // namespace blas

static void heapify(::coder::array<int, 1U> &x, int idx, int xstart, int xend, const anonymous_function *cmp) {
    int extremum;
    int extremumIdx;
    int i;
    int i1;
    int leftIdx;
    bool changed;
    bool exitg1;
    bool varargout_1;
    changed = true;
    extremumIdx = (idx + xstart) - 2;
    leftIdx = ((idx << 1) + xstart) - 2;
    exitg1 = false;
    while ((!exitg1) && (leftIdx + 1 < xend)) {
        int cmpIdx;
        int i2;
        int xcmp;
        changed = false;
        extremum = x[extremumIdx];
        cmpIdx = leftIdx;
        xcmp = x[leftIdx];
        i = x[leftIdx + 1] - 1;
        i1 = cmp->workspace.a[x[leftIdx] - 1];
        i2 = cmp->workspace.a[i];
        if (i1 < i2) {
            varargout_1 = true;
        } else if (i1 == i2) {
            varargout_1 = (cmp->workspace.b[x[leftIdx] - 1] < cmp->workspace.b[i]);
        } else {
            varargout_1 = false;
        }

        if (varargout_1) {
            cmpIdx = leftIdx + 1;
            xcmp = x[leftIdx + 1];
        }

        i = cmp->workspace.a[x[extremumIdx] - 1];
        i1 = cmp->workspace.a[xcmp - 1];
        if (i < i1) {
            varargout_1 = true;
        } else if (i == i1) {
            varargout_1 = (cmp->workspace.b[x[extremumIdx] - 1] <
                           cmp->workspace.b[xcmp - 1]);
        } else {
            varargout_1 = false;
        }

        if (varargout_1) {
            x[extremumIdx] = xcmp;
            x[cmpIdx] = extremum;
            extremumIdx = cmpIdx;
            leftIdx = ((((cmpIdx - xstart) + 2) << 1) + xstart) - 2;
            changed = true;
        } else {
            exitg1 = true;
        }
    }

    if (changed && (leftIdx + 1 <= xend)) {
        extremum = x[extremumIdx];
        i = cmp->workspace.a[x[extremumIdx] - 1];
        i1 = cmp->workspace.a[x[leftIdx] - 1];
        if (i < i1) {
            varargout_1 = true;
        } else if (i == i1) {
            varargout_1 = (cmp->workspace.b[x[extremumIdx] - 1] <
                           cmp->workspace.b[x[leftIdx] - 1]);
        } else {
            varargout_1 = false;
        }

        if (varargout_1) {
            x[extremumIdx] = x[leftIdx];
            x[leftIdx] = extremum;
        }
    }
}

static void insertionsort(::coder::array<int, 1U> &x, int xstart, int xend,
                          const anonymous_function *cmp) {
    int i;
    i = xstart + 1;
    for (int k{i}; k <= xend; k++) {
        int idx;
        int xc;
        bool exitg1;
        xc = x[k - 1] - 1;
        idx = k - 2;
        exitg1 = false;
        while ((!exitg1) && (idx + 1 >= xstart)) {
            bool varargout_1;
            if (cmp->workspace.a[xc] < cmp->workspace.a[x[idx] - 1]) {
                varargout_1 = true;
            } else if (cmp->workspace.a[xc] == cmp->workspace.a[x[idx] - 1]) {
                varargout_1 = (cmp->workspace.b[xc] < cmp->workspace.b[x[idx] - 1]);
            } else {
                varargout_1 = false;
            }

            if (varargout_1) {
                x[idx + 1] = x[idx];
                idx--;
            } else {
                exitg1 = true;
            }
        }

        x[idx + 1] = xc + 1;
    }
}

static void introsort(::coder::array<int, 1U> &x, int xend, const anonymous_function *cmp) {
    struct_T frame;
    if (xend > 1) {
        if (xend <= 32) {
            insertionsort(x, 1, xend, cmp);
        } else {
            stack st;
            int MAXDEPTH;
            int i;
            int j;
            int pmax;
            int pmin;
            int pow2p;
            bool exitg1;
            pmax = 31;
            pmin = 0;
            exitg1 = false;
            while ((!exitg1) && (pmax - pmin > 1)) {
                j = (pmin + pmax) >> 1;
                pow2p = 1 << j;
                if (pow2p == xend) {
                    pmax = j;
                    exitg1 = true;
                } else if (pow2p > xend) {
                    pmax = j;
                } else {
                    pmin = j;
                }
            }

            MAXDEPTH = (pmax - 1) << 1;
            frame.xstart = 1;
            frame.xend = xend;
            frame.depth = 0;
            pmax = MAXDEPTH << 1;
            st.d.size[0] = pmax;
            for (i = 0; i < pmax; i++) {
                st.d.data[i] = frame;
            }

            st.d.data[0] = frame;
            st.n = 1;
            while (st.n > 0) {
                int frame_tmp_tmp;
                frame_tmp_tmp = st.n - 1;
                frame = st.d.data[st.n - 1];
                st.n--;
                i = frame.xend - frame.xstart;
                if (i + 1 <= 32) {
                    insertionsort(x, frame.xstart, frame.xend, cmp);
                } else if (frame.depth == MAXDEPTH) {
                    b_heapsort(x, frame.xstart, frame.xend, cmp);
                } else {
                    int pivot;
                    bool varargout_1;
                    pow2p = (frame.xstart + i / 2) - 1;
                    i = x[frame.xstart - 1];
                    pmax = cmp->workspace.a[x[pow2p] - 1];
                    pmin = cmp->workspace.a[i - 1];
                    if (pmax < pmin) {
                        varargout_1 = true;
                    } else if (pmax == pmin) {
                        varargout_1 = (cmp->workspace.b[x[pow2p] - 1] <
                                       cmp->workspace.b[i - 1]);
                    } else {
                        varargout_1 = false;
                    }

                    if (varargout_1) {
                        x[frame.xstart - 1] = x[pow2p];
                        x[pow2p] = i;
                    }

                    if (cmp->workspace.a[x[frame.xend - 1] - 1] < cmp->workspace.a[x[frame.xstart - 1] - 1]) {
                        varargout_1 = true;
                    } else {
                        i = x[frame.xend - 1] - 1;
                        pmax = x[frame.xstart - 1] - 1;
                        if (cmp->workspace.a[i] == cmp->workspace.a[pmax]) {
                            varargout_1 = (cmp->workspace.b[i] < cmp->workspace.b[pmax]);
                        } else {
                            varargout_1 = false;
                        }
                    }

                    if (varargout_1) {
                        pmax = x[frame.xstart - 1];
                        x[frame.xstart - 1] = x[frame.xend - 1];
                        x[frame.xend - 1] = pmax;
                    }

                    if (cmp->workspace.a[x[frame.xend - 1] - 1] < cmp->workspace.a[x[pow2p] - 1]) {
                        varargout_1 = true;
                    } else {
                        i = x[frame.xend - 1] - 1;
                        if (cmp->workspace.a[i] == cmp->workspace.a[x[pow2p] - 1]) {
                            varargout_1 = (cmp->workspace.b[i] < cmp->workspace.b[x[pow2p] - 1]);
                        } else {
                            varargout_1 = false;
                        }
                    }

                    if (varargout_1) {
                        pmax = x[pow2p];
                        x[pow2p] = x[frame.xend - 1];
                        x[frame.xend - 1] = pmax;
                    }

                    pivot = x[pow2p] - 1;
                    x[pow2p] = x[frame.xend - 2];
                    x[frame.xend - 2] = pivot + 1;
                    pmin = frame.xstart - 1;
                    j = frame.xend - 2;
                    int exitg2;
                    do {
                        int exitg3;
                        exitg2 = 0;
                        pmin++;
                        do {
                            exitg3 = 0;
                            i = cmp->workspace.a[x[pmin] - 1];
                            if (i < cmp->workspace.a[pivot]) {
                                varargout_1 = true;
                            } else if (i == cmp->workspace.a[pivot]) {
                                varargout_1 = (cmp->workspace.b[x[pmin] - 1] <
                                               cmp->workspace.b[pivot]);
                            } else {
                                varargout_1 = false;
                            }

                            if (varargout_1) {
                                pmin++;
                            } else {
                                exitg3 = 1;
                            }
                        } while (exitg3 == 0);

                        j--;
                        do {
                            exitg3 = 0;
                            i = cmp->workspace.a[x[j] - 1];
                            if (cmp->workspace.a[pivot] < i) {
                                varargout_1 = true;
                            } else if (cmp->workspace.a[pivot] == i) {
                                varargout_1 = (cmp->workspace.b[pivot] < cmp->workspace.b[x[j] - 1]);
                            } else {
                                varargout_1 = false;
                            }

                            if (varargout_1) {
                                j--;
                            } else {
                                exitg3 = 1;
                            }
                        } while (exitg3 == 0);

                        if (pmin + 1 >= j + 1) {
                            exitg2 = 1;
                        } else {
                            pmax = x[pmin];
                            x[pmin] = x[j];
                            x[j] = pmax;
                        }
                    } while (exitg2 == 0);

                    x[frame.xend - 2] = x[pmin];
                    x[pmin] = pivot + 1;
                    if (pmin + 2 < frame.xend) {
                        st.d.data[frame_tmp_tmp].xstart = pmin + 2;
                        st.d.data[frame_tmp_tmp].xend = frame.xend;
                        st.d.data[frame_tmp_tmp].depth = frame.depth + 1;
                        st.n = frame_tmp_tmp + 1;
                    }

                    if (frame.xstart < pmin + 1) {
                        st.d.data[st.n].xstart = frame.xstart;
                        st.d.data[st.n].xend = pmin + 1;
                        st.d.data[st.n].depth = frame.depth + 1;
                        st.n++;
                    }
                }
            }
        }
    }
}

static double maximum(const double x[9]) {
    double ex;
    int idx;
    int k;
    if (!std::isnan(x[0])) {
        idx = 1;
    } else {
        bool exitg1;
        idx = 0;
        k = 2;
        exitg1 = false;
        while ((!exitg1) && (k <= 9)) {
            if (!std::isnan(x[k - 1])) {
                idx = k;
                exitg1 = true;
            } else {
                k++;
            }
        }
    }

    if (idx == 0) {
        ex = x[0];
    } else {
        ex = x[idx - 1];
        idx++;
        for (k = idx; k < 10; k++) {
            double d;
            d = x[k - 1];
            if (ex < d) {
                ex = d;
            }
        }
    }

    return ex;
}

static float maximum(const ::coder::array<float, 1U> &x) {
    float ex;
    int last;
    last = x.size(0);
    if (x.size(0) <= 2) {
        if (x.size(0) == 1) {
            ex = x[0];
        } else if ((x[0] < x[x.size(0) - 1]) || (std::isnan(x[0]) && (!std::
                                                                          isnan(x[x.size(0) - 1])))) {
            ex = x[x.size(0) - 1];
        } else {
            ex = x[0];
        }
    } else {
        int idx;
        int k;
        if (!std::isnan(x[0])) {
            idx = 1;
        } else {
            bool exitg1;
            idx = 0;
            k = 2;
            exitg1 = false;
            while ((!exitg1) && (k <= last)) {
                if (!std::isnan(x[k - 1])) {
                    idx = k;
                    exitg1 = true;
                } else {
                    k++;
                }
            }
        }

        if (idx == 0) {
            ex = x[0];
        } else {
            ex = x[idx - 1];
            idx++;
            for (k = idx; k <= last; k++) {
                float f;
                f = x[k - 1];
                if (ex < f) {
                    ex = f;
                }
            }
        }
    }

    return ex;
}

static void merge(::coder::array<int, 1U> &idx, ::coder::array<float, 1U> &x, int offset, int np, int nq, ::coder::array<int, 1U> &iwork, ::coder::array<float, 1U> &xwork) {
    if (nq != 0) {
        int iout;
        int n_tmp;
        int p;
        int q;
        n_tmp = np + nq;
        for (int j{0}; j < n_tmp; j++) {
            iout = offset + j;
            iwork[j] = idx[iout];
            xwork[j] = x[iout];
        }

        p = 0;
        q = np;
        iout = offset - 1;
        int exitg1;
        do {
            exitg1 = 0;
            iout++;
            if (xwork[p] <= xwork[q]) {
                idx[iout] = iwork[p];
                x[iout] = xwork[p];
                if (p + 1 < np) {
                    p++;
                } else {
                    exitg1 = 1;
                }
            } else {
                idx[iout] = iwork[q];
                x[iout] = xwork[q];
                if (q + 1 < n_tmp) {
                    q++;
                } else {
                    q = iout - p;
                    for (int j{p + 1}; j <= np; j++) {
                        iout = q + j;
                        idx[iout] = iwork[j - 1];
                        x[iout] = xwork[j - 1];
                    }

                    exitg1 = 1;
                }
            }
        } while (exitg1 == 0);
    }
}

static void merge(int idx_data[], int x_data[], int np, int nq, int iwork_data[], int xwork_data[]) {
    if ((np != 0) && (nq != 0)) {
        int iout;
        int offset1;
        int p;
        int q;
        offset1 = np + nq;
        if (offset1 - 1 >= 0) {
            std::copy(&idx_data[0], &idx_data[offset1], &iwork_data[0]);
            std::copy(&x_data[0], &x_data[offset1], &xwork_data[0]);
        }

        p = 0;
        q = np;
        iout = -1;
        int exitg1;
        do {
            exitg1 = 0;
            iout++;
            if (xwork_data[p] <= xwork_data[q]) {
                idx_data[iout] = iwork_data[p];
                x_data[iout] = xwork_data[p];
                if (p + 1 < np) {
                    p++;
                } else {
                    exitg1 = 1;
                }
            } else {
                idx_data[iout] = iwork_data[q];
                x_data[iout] = xwork_data[q];
                if (q + 1 < offset1) {
                    q++;
                } else {
                    offset1 = iout - p;
                    for (iout = p + 1; iout <= np; iout++) {
                        q = offset1 + iout;
                        idx_data[q] = iwork_data[iout - 1];
                        x_data[q] = xwork_data[iout - 1];
                    }

                    exitg1 = 1;
                }
            }
        } while (exitg1 == 0);
    }
}

static void merge_block(::coder::array<int, 1U> &idx, ::coder::array<float, 1U> &x, int offset, int n, int preSortLevel, ::coder::array<int, 1U> &iwork, ::coder::array<float, 1U> &xwork) {
    int bLen;
    int nPairs;
    nPairs = n >> preSortLevel;
    bLen = 1 << preSortLevel;
    while (nPairs > 1) {
        int nTail;
        int tailOffset;
        if ((nPairs & 1) != 0) {
            nPairs--;
            tailOffset = bLen * nPairs;
            nTail = n - tailOffset;
            if (nTail > bLen) {
                merge(idx, x, offset + tailOffset, bLen, nTail - bLen, iwork,
                      xwork);
            }
        }

        tailOffset = bLen << 1;
        nPairs >>= 1;
        for (nTail = 0; nTail < nPairs; nTail++) {
            merge(idx, x, offset + nTail * tailOffset, bLen, bLen, iwork, xwork);
        }

        bLen = tailOffset;
    }

    if (n > bLen) {
        merge(idx, x, offset, bLen, n - bLen, iwork, xwork);
    }
}

static double minimum(const double x[9]) {
    double ex;
    int idx;
    int k;
    if (!std::isnan(x[0])) {
        idx = 1;
    } else {
        bool exitg1;
        idx = 0;
        k = 2;
        exitg1 = false;
        while ((!exitg1) && (k <= 9)) {
            if (!std::isnan(x[k - 1])) {
                idx = k;
                exitg1 = true;
            } else {
                k++;
            }
        }
    }

    if (idx == 0) {
        ex = x[0];
    } else {
        ex = x[idx - 1];
        idx++;
        for (k = idx; k < 10; k++) {
            double d;
            d = x[k - 1];
            if (ex > d) {
                ex = d;
            }
        }
    }

    return ex;
}

static float minimum(const ::coder::array<float, 1U> &x) {
    float ex;
    int last;
    last = x.size(0);
    if (x.size(0) <= 2) {
        if (x.size(0) == 1) {
            ex = x[0];
        } else if ((x[0] > x[x.size(0) - 1]) || (std::isnan(x[0]) && (!std::
                                                                          isnan(x[x.size(0) - 1])))) {
            ex = x[x.size(0) - 1];
        } else {
            ex = x[0];
        }
    } else {
        int idx;
        int k;
        if (!std::isnan(x[0])) {
            idx = 1;
        } else {
            bool exitg1;
            idx = 0;
            k = 2;
            exitg1 = false;
            while ((!exitg1) && (k <= last)) {
                if (!std::isnan(x[k - 1])) {
                    idx = k;
                    exitg1 = true;
                } else {
                    k++;
                }
            }
        }

        if (idx == 0) {
            ex = x[0];
        } else {
            ex = x[idx - 1];
            idx++;
            for (k = idx; k <= last; k++) {
                float f;
                f = x[k - 1];
                if (ex > f) {
                    ex = f;
                }
            }
        }
    }

    return ex;
}

static void minimum(const ::coder::array<float, 2U> &x, ::coder::array<float, 1U> &ex, ::coder::array<int, 1U> &idx) {
    int loop_ub;
    int m;
    int n;
    m = x.size(0) - 1;
    n = x.size(1);
    ex.set_size(x.size(0));
    idx.set_size(x.size(0));
    loop_ub = x.size(0);
    for (int j{0}; j < loop_ub; j++) {
        idx[j] = 1;
    }

    if (x.size(0) >= 1) {
        for (loop_ub = 0; loop_ub <= m; loop_ub++) {
            ex[loop_ub] = x[loop_ub];
        }

        for (int j{2}; j <= n; j++) {
            for (loop_ub = 0; loop_ub <= m; loop_ub++) {
                float b;
                bool p;
                b = x[loop_ub + x.size(0) * (j - 1)];
                if (std::isnan(b)) {
                    p = false;
                } else if (std::isnan(ex[loop_ub])) {
                    p = true;
                } else {
                    p = (ex[loop_ub] > b);
                }

                if (p) {
                    ex[loop_ub] = b;
                    idx[loop_ub] = j;
                }
            }
        }
    }
}

namespace scalar {
static void b_sqrt(creal_T *x) {
    double absxi;
    double absxr;
    double xi;
    double xr;
    xr = x->re;
    xi = x->im;
    if (xi == 0.0) {
        if (xr < 0.0) {
            absxr = 0.0;
            absxi = std::sqrt(-xr);
        } else {
            absxr = std::sqrt(xr);
            absxi = 0.0;
        }
    } else if (xr == 0.0) {
        if (xi < 0.0) {
            absxr = std::sqrt(-xi / 2.0);
            absxi = -absxr;
        } else {
            absxr = std::sqrt(xi / 2.0);
            absxi = absxr;
        }
    } else if (std::isnan(xr)) {
        absxr = rtNaN;
        absxi = rtNaN;
    } else if (std::isnan(xi)) {
        absxr = rtNaN;
        absxi = rtNaN;
    } else if (std::isinf(xi)) {
        absxr = std::abs(xi);
        absxi = xi;
    } else if (std::isinf(xr)) {
        if (xr < 0.0) {
            absxr = 0.0;
            absxi = xi * -xr;
        } else {
            absxr = xr;
            absxi = 0.0;
        }
    } else {
        absxr = std::abs(xr);
        absxi = std::abs(xi);
        if ((absxr > 4.4942328371557893E+307) || (absxi >
                                                  4.4942328371557893E+307)) {
            absxr *= 0.5;
            absxi = rt_hypotd_snf(absxr, absxi * 0.5);
            if (absxi > absxr) {
                absxr = std::sqrt(absxi) * std::sqrt(absxr / absxi + 1.0);
            } else {
                absxr = std::sqrt(absxi) * 1.4142135623730951;
            }
        } else {
            absxr = std::sqrt((rt_hypotd_snf(absxr, absxi) + absxr) * 0.5);
        }

        if (xr > 0.0) {
            absxi = 0.5 * (xi / absxr);
        } else {
            if (xi < 0.0) {
                absxi = -absxr;
            } else {
                absxi = absxr;
            }

            absxr = 0.5 * (xi / absxi);
        }
    }

    x->re = absxr;
    x->im = absxi;
}
}  // namespace scalar

static void sort(::coder::array<float, 2U> &x, ::coder::array<int, 2U>
                                                   &idx) {
    ::coder::array<float, 1U> b_xwork;
    ::coder::array<float, 1U> vwork;
    ::coder::array<float, 1U> xwork;
    ::coder::array<int, 1U> b_iwork;
    ::coder::array<int, 1U> iidx;
    ::coder::array<int, 1U> iwork;
    int vlen;
    int vstride;
    vlen = x.size(1) - 1;
    vwork.set_size(x.size(1));
    idx.set_size(x.size(0), x.size(1));
    vstride = x.size(0);
    for (int j{0}; j < vstride; j++) {
        int i;
        int ib;
        for (int k{0}; k <= vlen; k++) {
            vwork[k] = x[j + k * vstride];
        }

        iidx.set_size(vwork.size(0));
        ib = vwork.size(0);
        for (i = 0; i < ib; i++) {
            iidx[i] = 0;
        }

        if (vwork.size(0) != 0) {
            float x4[4];
            int idx4[4];
            int bLen2;
            int i2;
            int i3;
            int i4;
            int iidx_tmp;
            int n;
            int nNonNaN;
            int quartetOffset;
            n = vwork.size(0);
            x4[0] = 0.0F;
            idx4[0] = 0;
            x4[1] = 0.0F;
            idx4[1] = 0;
            x4[2] = 0.0F;
            idx4[2] = 0;
            x4[3] = 0.0F;
            idx4[3] = 0;
            iwork.set_size(vwork.size(0));
            ib = vwork.size(0);
            for (i = 0; i < ib; i++) {
                iwork[i] = 0;
            }

            xwork.set_size(vwork.size(0));
            ib = vwork.size(0);
            for (i = 0; i < ib; i++) {
                xwork[i] = 0.0F;
            }

            bLen2 = 0;
            ib = 0;
            for (int k{0}; k < n; k++) {
                if (std::isnan(vwork[k])) {
                    iidx_tmp = (n - bLen2) - 1;
                    iidx[iidx_tmp] = k + 1;
                    xwork[iidx_tmp] = vwork[k];
                    bLen2++;
                } else {
                    ib++;
                    idx4[ib - 1] = k + 1;
                    x4[ib - 1] = vwork[k];
                    if (ib == 4) {
                        float f;
                        float f1;
                        signed char b_i2;
                        signed char b_i3;
                        signed char b_i4;
                        signed char i1;
                        quartetOffset = k - bLen2;
                        if (x4[0] <= x4[1]) {
                            ib = 1;
                            i2 = 2;
                        } else {
                            ib = 2;
                            i2 = 1;
                        }

                        if (x4[2] <= x4[3]) {
                            i3 = 3;
                            i4 = 4;
                        } else {
                            i3 = 4;
                            i4 = 3;
                        }

                        f = x4[ib - 1];
                        f1 = x4[i3 - 1];
                        if (f <= f1) {
                            f = x4[i2 - 1];
                            if (f <= f1) {
                                i1 = static_cast<signed char>(ib);
                                b_i2 = static_cast<signed char>(i2);
                                b_i3 = static_cast<signed char>(i3);
                                b_i4 = static_cast<signed char>(i4);
                            } else if (f <= x4[i4 - 1]) {
                                i1 = static_cast<signed char>(ib);
                                b_i2 = static_cast<signed char>(i3);
                                b_i3 = static_cast<signed char>(i2);
                                b_i4 = static_cast<signed char>(i4);
                            } else {
                                i1 = static_cast<signed char>(ib);
                                b_i2 = static_cast<signed char>(i3);
                                b_i3 = static_cast<signed char>(i4);
                                b_i4 = static_cast<signed char>(i2);
                            }
                        } else {
                            f1 = x4[i4 - 1];
                            if (f <= f1) {
                                if (x4[i2 - 1] <= f1) {
                                    i1 = static_cast<signed char>(i3);
                                    b_i2 = static_cast<signed char>(ib);
                                    b_i3 = static_cast<signed char>(i2);
                                    b_i4 = static_cast<signed char>(i4);
                                } else {
                                    i1 = static_cast<signed char>(i3);
                                    b_i2 = static_cast<signed char>(ib);
                                    b_i3 = static_cast<signed char>(i4);
                                    b_i4 = static_cast<signed char>(i2);
                                }
                            } else {
                                i1 = static_cast<signed char>(i3);
                                b_i2 = static_cast<signed char>(i4);
                                b_i3 = static_cast<signed char>(ib);
                                b_i4 = static_cast<signed char>(i2);
                            }
                        }

                        iidx[quartetOffset - 3] = idx4[i1 - 1];
                        iidx[quartetOffset - 2] = idx4[b_i2 - 1];
                        iidx[quartetOffset - 1] = idx4[b_i3 - 1];
                        iidx[quartetOffset] = idx4[b_i4 - 1];
                        vwork[quartetOffset - 3] = x4[i1 - 1];
                        vwork[quartetOffset - 2] = x4[b_i2 - 1];
                        vwork[quartetOffset - 1] = x4[b_i3 - 1];
                        vwork[quartetOffset] = x4[b_i4 - 1];
                        ib = 0;
                    }
                }
            }

            i3 = vwork.size(0) - bLen2;
            if (ib > 0) {
                signed char perm[4];
                perm[1] = 0;
                perm[2] = 0;
                perm[3] = 0;
                if (ib == 1) {
                    perm[0] = 1;
                } else if (ib == 2) {
                    if (x4[0] <= x4[1]) {
                        perm[0] = 1;
                        perm[1] = 2;
                    } else {
                        perm[0] = 2;
                        perm[1] = 1;
                    }
                } else if (x4[0] <= x4[1]) {
                    if (x4[1] <= x4[2]) {
                        perm[0] = 1;
                        perm[1] = 2;
                        perm[2] = 3;
                    } else if (x4[0] <= x4[2]) {
                        perm[0] = 1;
                        perm[1] = 3;
                        perm[2] = 2;
                    } else {
                        perm[0] = 3;
                        perm[1] = 1;
                        perm[2] = 2;
                    }
                } else if (x4[0] <= x4[2]) {
                    perm[0] = 2;
                    perm[1] = 1;
                    perm[2] = 3;
                } else if (x4[1] <= x4[2]) {
                    perm[0] = 2;
                    perm[1] = 3;
                    perm[2] = 1;
                } else {
                    perm[0] = 3;
                    perm[1] = 2;
                    perm[2] = 1;
                }

                i = static_cast<unsigned char>(ib);
                for (int k{0}; k < i; k++) {
                    iidx_tmp = perm[k] - 1;
                    quartetOffset = (i3 - ib) + k;
                    iidx[quartetOffset] = idx4[iidx_tmp];
                    vwork[quartetOffset] = x4[iidx_tmp];
                }
            }

            ib = bLen2 >> 1;
            for (int k{0}; k < ib; k++) {
                quartetOffset = i3 + k;
                i2 = iidx[quartetOffset];
                iidx_tmp = (n - k) - 1;
                iidx[quartetOffset] = iidx[iidx_tmp];
                iidx[iidx_tmp] = i2;
                vwork[quartetOffset] = xwork[iidx_tmp];
                vwork[iidx_tmp] = xwork[quartetOffset];
            }

            if ((bLen2 & 1) != 0) {
                ib += i3;
                vwork[ib] = xwork[ib];
            }

            nNonNaN = vwork.size(0) - bLen2;
            quartetOffset = 2;
            if (nNonNaN > 1) {
                if (vwork.size(0) >= 256) {
                    int nBlocks;
                    nBlocks = nNonNaN >> 8;
                    if (nBlocks > 0) {
                        for (int b{0}; b < nBlocks; b++) {
                            float c_xwork[256];
                            int c_iwork[256];
                            i4 = (b << 8) - 1;
                            for (int b_b{0}; b_b < 6; b_b++) {
                                n = 1 << (b_b + 2);
                                bLen2 = n << 1;
                                i = 256 >> (b_b + 3);
                                for (int k{0}; k < i; k++) {
                                    i2 = (i4 + k * bLen2) + 1;
                                    for (quartetOffset = 0; quartetOffset < bLen2;
                                         quartetOffset++) {
                                        ib = i2 + quartetOffset;
                                        c_iwork[quartetOffset] = iidx[ib];
                                        c_xwork[quartetOffset] = vwork[ib];
                                    }

                                    i3 = 0;
                                    quartetOffset = n;
                                    ib = i2 - 1;
                                    int exitg1;
                                    do {
                                        exitg1 = 0;
                                        ib++;
                                        if (c_xwork[i3] <= c_xwork[quartetOffset]) {
                                            iidx[ib] = c_iwork[i3];
                                            vwork[ib] = c_xwork[i3];
                                            if (i3 + 1 < n) {
                                                i3++;
                                            } else {
                                                exitg1 = 1;
                                            }
                                        } else {
                                            iidx[ib] = c_iwork[quartetOffset];
                                            vwork[ib] = c_xwork[quartetOffset];
                                            if (quartetOffset + 1 < bLen2) {
                                                quartetOffset++;
                                            } else {
                                                ib -= i3;
                                                for (quartetOffset = i3 + 1; quartetOffset <= n;
                                                     quartetOffset++) {
                                                    iidx_tmp = ib + quartetOffset;
                                                    iidx[iidx_tmp] = c_iwork[quartetOffset - 1];
                                                    vwork[iidx_tmp] = c_xwork[quartetOffset - 1];
                                                }

                                                exitg1 = 1;
                                            }
                                        }
                                    } while (exitg1 == 0);
                                }
                            }
                        }

                        ib = nBlocks << 8;
                        quartetOffset = nNonNaN - ib;
                        if (quartetOffset > 0) {
                            merge_block(iidx, vwork, ib, quartetOffset, 2, iwork, xwork);
                        }

                        quartetOffset = 8;
                    }
                }

                ib = iwork.size(0);
                b_iwork.set_size(iwork.size(0));
                for (i = 0; i < ib; i++) {
                    b_iwork[i] = iwork[i];
                }

                b_xwork.set_size(xwork.size(0));
                ib = xwork.size(0);
                for (i = 0; i < ib; i++) {
                    b_xwork[i] = xwork[i];
                }

                merge_block(iidx, vwork, 0, nNonNaN, quartetOffset, b_iwork,
                            b_xwork);
            }
        }

        for (int k{0}; k <= vlen; k++) {
            i = j + k * vstride;
            x[i] = vwork[k];
            idx[i] = iidx[k];
        }
    }
}

static void sort(int x_data[], const int *x_size, int idx_data[], int *idx_size) {
    int b_iwork_data[10];
    int iidx_data[10];
    int iwork_data[10];
    int vwork_data[10];
    int xwork_data[10];
    int dim;
    int i1;
    int vlen;
    int vstride;
    int vwork_size;
    dim = 0;
    if (*x_size != 1) {
        dim = -1;
    }

    if (dim + 2 <= 1) {
        vwork_size = *x_size;
    } else {
        vwork_size = 1;
    }

    vlen = vwork_size - 1;
    *idx_size = *x_size;
    vstride = 1;
    for (i1 = 0; i1 <= dim; i1++) {
        vstride *= *x_size;
    }

    for (int j{0}; j < vstride; j++) {
        int i;
        for (i1 = 0; i1 <= vlen; i1++) {
            vwork_data[i1] = x_data[j + i1 * vstride];
        }

        if (vwork_size - 1 >= 0) {
            std::memset(&iidx_data[0], 0, static_cast<unsigned int>(vwork_size) * sizeof(int));
        }

        if (vwork_size != 0) {
            int x4[4];
            int i4;
            int nBlocks;
            int nLeft;
            int x4_tmp;
            signed char idx4[4];
            x4[0] = 0;
            idx4[0] = 0;
            x4[1] = 0;
            idx4[1] = 0;
            x4[2] = 0;
            idx4[2] = 0;
            x4[3] = 0;
            idx4[3] = 0;
            std::memset(&iwork_data[0], 0, static_cast<unsigned int>(vwork_size) * sizeof(int));
            std::memset(&xwork_data[0], 0, static_cast<unsigned int>(vwork_size) * sizeof(int));
            nBlocks = vwork_size >> 2;
            for (int b_j{0}; b_j < nBlocks; b_j++) {
                int b_i;
                int i2;
                signed char b_i1;
                signed char b_i2;
                signed char b_i4;
                signed char i3;
                b_i = b_j << 2;
                idx4[0] = static_cast<signed char>(b_i + 1);
                idx4[1] = static_cast<signed char>(b_i + 2);
                idx4[2] = static_cast<signed char>(b_i + 3);
                idx4[3] = static_cast<signed char>(b_i + 4);
                i = vwork_data[b_i];
                x4[0] = i;
                dim = vwork_data[b_i + 1];
                x4[1] = dim;
                x4_tmp = vwork_data[b_i + 2];
                x4[2] = x4_tmp;
                i4 = vwork_data[b_i + 3];
                x4[3] = i4;
                if (i <= dim) {
                    i1 = 1;
                    i2 = 2;
                } else {
                    i1 = 2;
                    i2 = 1;
                }

                if (x4_tmp <= i4) {
                    dim = 3;
                    i4 = 4;
                } else {
                    dim = 4;
                    i4 = 3;
                }

                i = x4[i1 - 1];
                nLeft = x4[dim - 1];
                if (i <= nLeft) {
                    i = x4[i2 - 1];
                    if (i <= nLeft) {
                        b_i1 = static_cast<signed char>(i1);
                        b_i2 = static_cast<signed char>(i2);
                        i3 = static_cast<signed char>(dim);
                        b_i4 = static_cast<signed char>(i4);
                    } else if (i <= x4[i4 - 1]) {
                        b_i1 = static_cast<signed char>(i1);
                        b_i2 = static_cast<signed char>(dim);
                        i3 = static_cast<signed char>(i2);
                        b_i4 = static_cast<signed char>(i4);
                    } else {
                        b_i1 = static_cast<signed char>(i1);
                        b_i2 = static_cast<signed char>(dim);
                        i3 = static_cast<signed char>(i4);
                        b_i4 = static_cast<signed char>(i2);
                    }
                } else {
                    nLeft = x4[i4 - 1];
                    if (i <= nLeft) {
                        if (x4[i2 - 1] <= nLeft) {
                            b_i1 = static_cast<signed char>(dim);
                            b_i2 = static_cast<signed char>(i1);
                            i3 = static_cast<signed char>(i2);
                            b_i4 = static_cast<signed char>(i4);
                        } else {
                            b_i1 = static_cast<signed char>(dim);
                            b_i2 = static_cast<signed char>(i1);
                            i3 = static_cast<signed char>(i4);
                            b_i4 = static_cast<signed char>(i2);
                        }
                    } else {
                        b_i1 = static_cast<signed char>(dim);
                        b_i2 = static_cast<signed char>(i4);
                        i3 = static_cast<signed char>(i1);
                        b_i4 = static_cast<signed char>(i2);
                    }
                }

                iidx_data[b_i] = idx4[b_i1 - 1];
                iidx_data[b_i + 1] = idx4[b_i2 - 1];
                iidx_data[b_i + 2] = idx4[i3 - 1];
                iidx_data[b_i + 3] = idx4[b_i4 - 1];
                vwork_data[b_i] = x4[b_i1 - 1];
                vwork_data[b_i + 1] = x4[b_i2 - 1];
                vwork_data[b_i + 2] = x4[i3 - 1];
                vwork_data[b_i + 3] = x4[b_i4 - 1];
            }

            i4 = nBlocks << 2;
            nLeft = vwork_size - i4;
            if (nLeft > 0) {
                signed char perm[4];
                for (i1 = 0; i1 < nLeft; i1++) {
                    dim = i4 + i1;
                    idx4[i1] = static_cast<signed char>(dim + 1);
                    x4[i1] = vwork_data[dim];
                }

                perm[1] = 0;
                perm[2] = 0;
                perm[3] = 0;
                if (nLeft == 1) {
                    perm[0] = 1;
                } else if (nLeft == 2) {
                    if (x4[0] <= x4[1]) {
                        perm[0] = 1;
                        perm[1] = 2;
                    } else {
                        perm[0] = 2;
                        perm[1] = 1;
                    }
                } else if (x4[0] <= x4[1]) {
                    if (x4[1] <= x4[2]) {
                        perm[0] = 1;
                        perm[1] = 2;
                        perm[2] = 3;
                    } else if (x4[0] <= x4[2]) {
                        perm[0] = 1;
                        perm[1] = 3;
                        perm[2] = 2;
                    } else {
                        perm[0] = 3;
                        perm[1] = 1;
                        perm[2] = 2;
                    }
                } else if (x4[0] <= x4[2]) {
                    perm[0] = 2;
                    perm[1] = 1;
                    perm[2] = 3;
                } else if (x4[1] <= x4[2]) {
                    perm[0] = 2;
                    perm[1] = 3;
                    perm[2] = 1;
                } else {
                    perm[0] = 3;
                    perm[1] = 2;
                    perm[2] = 1;
                }

                for (i1 = 0; i1 < nLeft; i1++) {
                    dim = perm[i1] - 1;
                    x4_tmp = i4 + i1;
                    iidx_data[x4_tmp] = idx4[dim];
                    vwork_data[x4_tmp] = x4[dim];
                }
            }

            if (vwork_size > 1) {
                dim = 4;
                while (nBlocks > 1) {
                    merge(iidx_data, vwork_data, dim, dim, iwork_data, xwork_data);
                    dim <<= 1;
                    nBlocks = 1;
                }

                if (vwork_size > dim) {
                    if (vwork_size - 1 >= 0) {
                        std::copy(&iwork_data[0], &iwork_data[vwork_size],
                                  &b_iwork_data[0]);
                        std::copy(&xwork_data[0], &xwork_data[vwork_size],
                                  &iwork_data[0]);
                    }

                    merge(iidx_data, vwork_data, dim, vwork_size - dim, b_iwork_data,
                          iwork_data);
                }
            }
        }

        for (i1 = 0; i1 <= vlen; i1++) {
            i = j + i1 * vstride;
            x_data[i] = vwork_data[i1];
            idx_data[i] = iidx_data[i1];
        }
    }
}

static void svd(const double A[4], double U[4], double s[2], double V[4]) {
    double b_s[2];
    double e[2];
    double A_idx_3;
    double f;
    double nrm;
    double rt;
    double sm;
    double snorm;
    double sqds;
    double temp;
    int iter;
    int kase;
    int m;
    int q;
    int qs;
    temp = A[0];
    sm = A[1];
    sqds = A[2];
    A_idx_3 = A[3];
    nrm = blas::xnrm2(A);
    if (nrm > 0.0) {
        if (A[0] < 0.0) {
            b_s[0] = -nrm;
        } else {
            b_s[0] = nrm;
        }

        if (std::abs(b_s[0]) >= 1.0020841800044864E-292) {
            rt = 1.0 / b_s[0];
            temp = rt * A[0];
            sm = rt * A[1];
        } else {
            temp = A[0] / b_s[0];
            sm = A[1] / b_s[0];
        }

        temp++;
        b_s[0] = -b_s[0];
        rt = -((temp * A[2] + sm * A[3]) / temp);
        if (!(rt == 0.0)) {
            sqds = A[2] + rt * temp;
            A_idx_3 = A[3] + rt * sm;
        }
    } else {
        b_s[0] = 0.0;
    }

    m = 2;
    b_s[1] = A_idx_3;
    e[0] = sqds;
    e[1] = 0.0;
    U[2] = 0.0;
    U[3] = 1.0;
    if (b_s[0] != 0.0) {
        rt = -((temp * 0.0 + sm) / temp);
        if (!(rt == 0.0)) {
            U[2] = rt * temp;
            U[3] = rt * sm + 1.0;
        }

        U[1] = -sm;
        U[0] = -temp + 1.0;
    } else {
        U[1] = 0.0;
        U[0] = 1.0;
    }

    V[2] = 0.0;
    V[3] = 1.0;
    V[1] = 0.0;
    V[0] = 1.0;
    for (q = 0; q < 2; q++) {
        nrm = b_s[q];
        if (nrm != 0.0) {
            rt = std::abs(nrm);
            nrm /= rt;
            b_s[q] = rt;
            if (q + 1 < 2) {
                e[0] /= nrm;
            }

            kase = q << 1;
            qs = kase + 2;
            for (int k{kase + 1}; k <= qs; k++) {
                U[k - 1] *= nrm;
            }
        }

        if ((q + 1 < 2) && (e[0] != 0.0)) {
            rt = std::abs(e[0]);
            nrm = rt / e[0];
            e[0] = rt;
            b_s[1] *= nrm;
            V[2] *= nrm;
            V[3] *= nrm;
        }
    }

    iter = 0;
    snorm = std::fmax(std::fmax(0.0, std::fmax(std::abs(b_s[0]), std::abs(e
                                                                              [0]))),
                      std::fmax(std::abs(b_s[1]), 0.0));
    while ((m > 0) && (iter < 75)) {
        int ii_tmp_tmp;
        bool exitg1;
        ii_tmp_tmp = m - 1;
        q = m - 1;
        exitg1 = false;
        while (!(exitg1 || (q == 0))) {
            nrm = std::abs(e[0]);
            if ((nrm <= 2.2204460492503131E-16 * (std::abs(b_s[0]) + std::abs(b_s[1]))) || (nrm <= 1.0020841800044864E-292) || ((iter > 20) && (nrm <= 2.2204460492503131E-16 * snorm))) {
                e[0] = 0.0;
                exitg1 = true;
            } else {
                q = 0;
            }
        }

        if (q == m - 1) {
            kase = 4;
        } else {
            qs = m;
            kase = m;
            exitg1 = false;
            while ((!exitg1) && (kase >= q)) {
                qs = kase;
                if (kase == q) {
                    exitg1 = true;
                } else {
                    nrm = 0.0;
                    if (kase < m) {
                        nrm = std::abs(e[0]);
                    }

                    if (kase > q + 1) {
                        nrm += std::abs(e[0]);
                    }

                    rt = std::abs(b_s[kase - 1]);
                    if ((rt <= 2.2204460492503131E-16 * nrm) || (rt <=
                                                                 1.0020841800044864E-292)) {
                        b_s[kase - 1] = 0.0;
                        exitg1 = true;
                    } else {
                        kase--;
                    }
                }
            }

            if (qs == q) {
                kase = 3;
            } else if (qs == m) {
                kase = 1;
            } else {
                kase = 2;
                q = qs;
            }
        }

        switch (kase) {
            case 1:
                f = e[0];
                e[0] = 0.0;
                for (int k{ii_tmp_tmp}; k >= q + 1; k--) {
                    blas::xrotg(&b_s[0], &f, &A_idx_3, &rt);
                    kase = (m - 1) << 1;
                    temp = A_idx_3 * V[0] + rt * V[kase];
                    V[kase] = A_idx_3 * V[kase] - rt * V[0];
                    V[0] = temp;
                    nrm = V[kase + 1];
                    temp = A_idx_3 * V[1] + rt * nrm;
                    V[kase + 1] = A_idx_3 * nrm - rt * V[1];
                    V[1] = temp;
                }
                break;

            case 2:
                f = e[q - 1];
                e[q - 1] = 0.0;
                for (int k{q + 1}; k <= m; k++) {
                    blas::xrotg(&b_s[k - 1], &f, &A_idx_3, &sm);
                    rt = e[k - 1];
                    f = -sm * rt;
                    e[k - 1] = rt * A_idx_3;
                    qs = (k - 1) << 1;
                    kase = (q - 1) << 1;
                    temp = A_idx_3 * U[qs] + sm * U[kase];
                    U[kase] = A_idx_3 * U[kase] - sm * U[qs];
                    U[qs] = temp;
                    nrm = U[kase + 1];
                    rt = U[qs + 1];
                    U[kase + 1] = A_idx_3 * nrm - sm * rt;
                    U[qs + 1] = A_idx_3 * rt + sm * nrm;
                }
                break;

            case 3: {
                double scale;
                nrm = b_s[m - 1];
                scale = std::fmax(std::fmax(std::fmax(std::fmax(std::abs(nrm), std::
                                                                                   abs(b_s[0])),
                                                      std::abs(e[0])),
                                            std::abs(b_s[q])),
                                  std::abs(e[q]));
                sm = nrm / scale;
                rt = b_s[0] / scale;
                nrm = e[0] / scale;
                sqds = b_s[q] / scale;
                temp = ((rt + sm) * (rt - sm) + nrm * nrm) / 2.0;
                A_idx_3 = sm * nrm;
                A_idx_3 *= A_idx_3;
                if ((temp != 0.0) || (A_idx_3 != 0.0)) {
                    rt = std::sqrt(temp * temp + A_idx_3);
                    if (temp < 0.0) {
                        rt = -rt;
                    }

                    rt = A_idx_3 / (temp + rt);
                } else {
                    rt = 0.0;
                }

                f = (sqds + sm) * (sqds - sm) + rt;
                rt = sqds * (e[q] / scale);
                for (int k{q + 1}; k < 2; k++) {
                    blas::xrotg(&f, &rt, &A_idx_3, &sm);
                    f = A_idx_3 * b_s[0] + sm * e[0];
                    nrm = A_idx_3 * e[0] - sm * b_s[0];
                    e[0] = nrm;
                    rt = sm * b_s[1];
                    b_s[1] *= A_idx_3;
                    temp = A_idx_3 * V[0] + sm * V[2];
                    V[2] = A_idx_3 * V[2] - sm * V[0];
                    V[0] = temp;
                    temp = A_idx_3 * V[1] + sm * V[3];
                    V[3] = A_idx_3 * V[3] - sm * V[1];
                    V[1] = temp;
                    b_s[0] = f;
                    blas::xrotg(&b_s[0], &rt, &A_idx_3, &sm);
                    f = A_idx_3 * nrm + sm * b_s[1];
                    b_s[1] = -sm * nrm + A_idx_3 * b_s[1];
                    rt = sm * e[1];
                    e[1] *= A_idx_3;
                    temp = A_idx_3 * U[0] + sm * U[2];
                    U[2] = A_idx_3 * U[2] - sm * U[0];
                    U[0] = temp;
                    temp = A_idx_3 * U[1] + sm * U[3];
                    U[3] = A_idx_3 * U[3] - sm * U[1];
                    U[1] = temp;
                }

                e[0] = f;
                iter++;
            } break;

            default:
                if (b_s[q] < 0.0) {
                    b_s[q] = -b_s[q];
                    kase = q << 1;
                    qs = kase + 2;
                    for (int k{kase + 1}; k <= qs; k++) {
                        V[k - 1] = -V[k - 1];
                    }
                }

                while ((q + 1 < 2) && (b_s[0] < b_s[1])) {
                    rt = b_s[0];
                    b_s[0] = b_s[1];
                    b_s[1] = rt;
                    blas::xswap(V);
                    blas::xswap(U);
                    q = 1;
                }

                iter = 0;
                m--;
                break;
        }
    }

    s[0] = b_s[0];
    s[1] = b_s[1];
}
}  // namespace internal

static void inv(const double x[9], double y[9]) {
    double b_x[9];
    double absx11;
    double absx21;
    double absx31;
    int p1;
    int p2;
    int p3;
    std::copy(&x[0], &x[9], &b_x[0]);
    p1 = 0;
    p2 = 3;
    p3 = 6;
    absx11 = std::abs(x[0]);
    absx21 = std::abs(x[1]);
    absx31 = std::abs(x[2]);
    if ((absx21 > absx11) && (absx21 > absx31)) {
        p1 = 3;
        p2 = 0;
        b_x[0] = x[1];
        b_x[1] = x[0];
        b_x[3] = x[4];
        b_x[4] = x[3];
        b_x[6] = x[7];
        b_x[7] = x[6];
    } else if (absx31 > absx11) {
        p1 = 6;
        p3 = 0;
        b_x[0] = x[2];
        b_x[2] = x[0];
        b_x[3] = x[5];
        b_x[5] = x[3];
        b_x[6] = x[8];
        b_x[8] = x[6];
    }

    b_x[1] /= b_x[0];
    b_x[2] /= b_x[0];
    b_x[4] -= b_x[1] * b_x[3];
    b_x[5] -= b_x[2] * b_x[3];
    b_x[7] -= b_x[1] * b_x[6];
    b_x[8] -= b_x[2] * b_x[6];
    if (std::abs(b_x[5]) > std::abs(b_x[4])) {
        int itmp;
        itmp = p2;
        p2 = p3;
        p3 = itmp;
        absx11 = b_x[1];
        b_x[1] = b_x[2];
        b_x[2] = absx11;
        absx11 = b_x[4];
        b_x[4] = b_x[5];
        b_x[5] = absx11;
        absx11 = b_x[7];
        b_x[7] = b_x[8];
        b_x[8] = absx11;
    }

    b_x[5] /= b_x[4];
    b_x[8] -= b_x[5] * b_x[7];
    absx11 = (b_x[1] * b_x[5] - b_x[2]) / b_x[8];
    absx21 = -(b_x[1] + b_x[7] * absx11) / b_x[4];
    y[p1] = ((1.0 - b_x[3] * absx21) - b_x[6] * absx11) / b_x[0];
    y[p1 + 1] = absx21;
    y[p1 + 2] = absx11;
    absx11 = -b_x[5] / b_x[8];
    absx21 = (1.0 - b_x[7] * absx11) / b_x[4];
    y[p2] = -(b_x[3] * absx21 + b_x[6] * absx11) / b_x[0];
    y[p2 + 1] = absx21;
    y[p2 + 2] = absx11;
    absx11 = 1.0 / b_x[8];
    absx21 = -b_x[7] * absx11 / b_x[4];
    y[p3] = -(b_x[3] * absx21 + b_x[6] * absx11) / b_x[0];
    y[p3 + 1] = absx21;
    y[p3 + 2] = absx11;
}

static bool isMemberRows(const double A[2], const ::coder::array<double, 2U>
                                                &S) {
    int iRowS;
    bool exitg1;
    bool tf;
    tf = false;
    iRowS = 0;
    exitg1 = false;
    while ((!exitg1) && (iRowS <= S.size(0) - 1)) {
        int j;
        bool exitg2;
        bool p;
        p = true;
        j = 0;
        exitg2 = false;
        while ((!exitg2) && (j < 2)) {
            if (A[j] != S[iRowS + S.size(0) * j]) {
                p = false;
                exitg2 = true;
            } else {
                j++;
            }
        }

        if (p) {
            tf = true;
            exitg1 = true;
        } else {
            iRowS++;
        }
    }

    return tf;
}

static void matchFeatures(HDMapping *aInstancePtr, const binaryFeatures *varargin_1, const binaryFeatures *varargin_2, ::coder::array<unsigned int, 2U> &indexPairs) {
    ::coder::array<float, 2U> b_matchMetric;
    ::coder::array<float, 2U> ex;
    ::coder::array<float, 2U> matchMetric;
    ::coder::array<float, 2U> scores;
    ::coder::array<int, 2U> b_idx;
    ::coder::array<unsigned int, 2U> b_indexPairs;
    ::coder::array<unsigned int, 2U> c_indexPairs;
    ::coder::array<int, 2U> r;
    ::coder::array<int, 2U> r1;
    ::coder::array<int, 2U> r2;
    ::coder::array<int, 2U> r3;
    ::coder::array<unsigned char, 2U> features1in;
    ::coder::array<unsigned char, 2U> features2in;
    ::coder::array<bool, 2U> inds;
    constructWorldMapStackData *localSD;
    int i;
    int i1;
    int loop_ub;
    localSD = aInstancePtr->getStackData();
    features1in.set_size(32, varargin_1->Features.size(0));
    loop_ub = varargin_1->Features.size(0);
    for (i = 0; i < loop_ub; i++) {
        for (i1 = 0; i1 < 32; i1++) {
            features1in[i1 + 32 * i] = varargin_1->Features[i +
                                                            varargin_1->Features.size(0) * i1];
        }
    }

    features2in.set_size(32, varargin_2->Features.size(0));
    loop_ub = varargin_2->Features.size(0);
    for (i = 0; i < loop_ub; i++) {
        for (i1 = 0; i1 < 32; i1++) {
            features2in[i1 + 32 * i] = varargin_2->Features[i +
                                                            varargin_2->Features.size(0) * i1];
        }
    }

    if ((features1in.size(1) == 0) || (features2in.size(1) == 0)) {
        indexPairs.set_size(0, 2);
    } else {
        float y;
        int k;
        int n;
        int olddi;
        scores.set_size(features1in.size(1), features2in.size(1));
        if (!localSD->pd->lookupTable_not_empty) {
            localSD->pd->lookupTable_not_empty = true;
            for (int b_i{0}; b_i < 256; b_i++) {
                double d;
                char s_data[64];
                char sfull[64];
                signed char x_data[64];
                bool exitg1;
                localSD->pd->lookupTable[b_i] = 0.0F;
                d = b_i;
                for (i = 0; i < 64; i++) {
                    sfull[i] = '0';
                }

                loop_ub = 64;
                exitg1 = false;
                while ((!exitg1) && (loop_ub > 0)) {
                    olddi = static_cast<int>(d);
                    d /= 2.0;
                    d = std::floor(d);
                    if ((static_cast<int>(d) << 1) < olddi) {
                        sfull[loop_ub - 1] = '1';
                    }

                    if (!(d > 0.0)) {
                        exitg1 = true;
                    } else {
                        loop_ub--;
                    }
                }

                olddi = 0;
                k = 0;
                exitg1 = false;
                while ((!exitg1) && (k < 64)) {
                    if (sfull[k] == '1') {
                        olddi = k + 1;
                        exitg1 = true;
                    } else {
                        k++;
                    }
                }

                if (olddi == 0) {
                    olddi = 64;
                }

                n = 64 - olddi;
                if (65 - olddi <= 1) {
                    loop_ub = 1;
                } else {
                    loop_ub = 65 - olddi;
                }

                for (k = 0; k <= n; k++) {
                    s_data[k] = sfull[(olddi + k) - 1];
                }

                for (i = 0; i < loop_ub; i++) {
                    x_data[i] = static_cast<signed char>(s_data[i] - 48);
                }

                d = x_data[0];
                for (k = 2; k <= loop_ub; k++) {
                    d += static_cast<double>(x_data[k - 1]);
                }

                localSD->pd->lookupTable[b_i] = static_cast<float>(d);
            }
        }

        i = features2in.size(1);
        i1 = features1in.size(1);
        for (olddi = 0; olddi < i; olddi++) {
            for (n = 0; n < i1; n++) {
                short idx[32];
                for (int b_i{0}; b_i < 32; b_i++) {
                    unsigned char b_varargin_1;
                    unsigned char b_varargin_2;
                    b_varargin_1 = features1in[b_i + 32 * n];
                    b_varargin_2 = features2in[b_i + 32 * olddi];
                    idx[b_i] = static_cast<short>(static_cast<unsigned char>(b_varargin_1 ^ b_varargin_2) + 1);
                }

                y = localSD->pd->lookupTable[idx[0] - 1];
                for (k = 0; k < 31; k++) {
                    y += localSD->pd->lookupTable[idx[k + 1] - 1];
                }

                scores[n + scores.size(0) * olddi] = y;
            }
        }

        vision::internal::matchFeatures::findNearestNeighbors(scores,
                                                              b_indexPairs, matchMetric);
        inds.set_size(1, matchMetric.size(1));
        loop_ub = matchMetric.size(1);
        for (i = 0; i < loop_ub; i++) {
            inds[i] = (matchMetric[matchMetric.size(0) * i] <= 128.0F);
        }

        n = inds.size(1) - 1;
        olddi = 0;
        for (int b_i{0}; b_i <= n; b_i++) {
            if (inds[b_i]) {
                olddi++;
            }
        }

        b_idx.set_size(1, olddi);
        olddi = 0;
        for (int b_i{0}; b_i <= n; b_i++) {
            if (inds[b_i]) {
                b_idx[olddi] = b_i + 1;
                olddi++;
            }
        }

        c_indexPairs.set_size(2, b_idx.size(1));
        loop_ub = b_idx.size(1);
        for (i = 0; i < loop_ub; i++) {
            c_indexPairs[2 * i] = b_indexPairs[2 * (b_idx[i] - 1)];
            c_indexPairs[2 * i + 1] = b_indexPairs[2 * (b_idx[i] - 1) + 1];
        }

        b_indexPairs.set_size(2, c_indexPairs.size(1));
        loop_ub = 2 * c_indexPairs.size(1);
        for (i = 0; i < loop_ub; i++) {
            b_indexPairs[i] = c_indexPairs[i];
        }

        n = inds.size(1) - 1;
        olddi = 0;
        for (int b_i{0}; b_i <= n; b_i++) {
            if (inds[b_i]) {
                olddi++;
            }
        }

        r.set_size(1, olddi);
        olddi = 0;
        for (int b_i{0}; b_i <= n; b_i++) {
            if (inds[b_i]) {
                r[olddi] = b_i + 1;
                olddi++;
            }
        }

        olddi = matchMetric.size(0);
        b_matchMetric.set_size(matchMetric.size(0), r.size(1));
        loop_ub = r.size(1);
        for (i = 0; i < loop_ub; i++) {
            for (i1 = 0; i1 < olddi; i1++) {
                b_matchMetric[i1 + b_matchMetric.size(0) * i] = matchMetric[i1 +
                                                                            matchMetric.size(0) * (r[i] - 1)];
            }
        }

        matchMetric.set_size(b_matchMetric.size(0), b_matchMetric.size(1));
        loop_ub = b_matchMetric.size(0) * b_matchMetric.size(1);
        for (i = 0; i < loop_ub; i++) {
            matchMetric[i] = b_matchMetric[i];
        }

        if (features2in.size(1) > 1) {
            inds.set_size(1, matchMetric.size(1));
            loop_ub = matchMetric.size(1);
            for (i = 0; i < loop_ub; i++) {
                inds[i] = (matchMetric[matchMetric.size(0) * i + 1] < 1.0E-6F);
            }

            n = inds.size(1) - 1;
            olddi = 0;
            for (int b_i{0}; b_i <= n; b_i++) {
                if (inds[b_i]) {
                    olddi++;
                }
            }

            r1.set_size(1, olddi);
            olddi = 0;
            for (int b_i{0}; b_i <= n; b_i++) {
                if (inds[b_i]) {
                    r1[olddi] = b_i + 1;
                    olddi++;
                }
            }

            loop_ub = r1.size(1);
            for (i = 0; i < loop_ub; i++) {
                olddi = matchMetric.size(0);
                for (i1 = 0; i1 < olddi; i1++) {
                    matchMetric[i1 + matchMetric.size(0) * (r1[i] - 1)] = 1.0F;
                }
            }

            inds.set_size(1, matchMetric.size(1));
            loop_ub = matchMetric.size(1);
            for (i = 0; i < loop_ub; i++) {
                inds[i] = (matchMetric[matchMetric.size(0) * i] /
                               matchMetric[matchMetric.size(0) * i + 1] <=
                           0.5F);
            }
        } else {
            inds.set_size(1, matchMetric.size(1));
            loop_ub = matchMetric.size(1);
            for (i = 0; i < loop_ub; i++) {
                inds[i] = true;
            }
        }

        n = inds.size(1) - 1;
        olddi = 0;
        for (int b_i{0}; b_i <= n; b_i++) {
            if (inds[b_i]) {
                olddi++;
            }
        }

        r2.set_size(1, olddi);
        olddi = 0;
        for (int b_i{0}; b_i <= n; b_i++) {
            if (inds[b_i]) {
                r2[olddi] = b_i + 1;
                olddi++;
            }
        }

        c_indexPairs.set_size(2, r2.size(1));
        loop_ub = r2.size(1);
        for (i = 0; i < loop_ub; i++) {
            c_indexPairs[2 * i] = b_indexPairs[2 * (r2[i] - 1)];
            c_indexPairs[2 * i + 1] = b_indexPairs[2 * (r2[i] - 1) + 1];
        }

        b_indexPairs.set_size(2, c_indexPairs.size(1));
        loop_ub = 2 * c_indexPairs.size(1);
        for (i = 0; i < loop_ub; i++) {
            b_indexPairs[i] = c_indexPairs[i];
        }

        if (b_indexPairs.size(1) == 0) {
            c_indexPairs.set_size(2, 0);
        } else {
            olddi = scores.size(0);
            n = b_indexPairs.size(1);
            ex.set_size(1, b_indexPairs.size(1));
            b_idx.set_size(1, b_indexPairs.size(1));
            loop_ub = b_indexPairs.size(1);
            for (i = 0; i < loop_ub; i++) {
                b_idx[i] = 1;
            }

            for (loop_ub = 0; loop_ub < n; loop_ub++) {
                ex[loop_ub] = scores[scores.size(0) * (static_cast<int>(b_indexPairs[2 * loop_ub + 1]) - 1)];
                for (int b_i{2}; b_i <= olddi; b_i++) {
                    float b_tmp;
                    bool p;
                    y = ex[loop_ub];
                    b_tmp = scores[(b_i + scores.size(0) * (static_cast<int>(b_indexPairs[2 * loop_ub + 1]) - 1)) - 1];
                    if (std::isnan(b_tmp)) {
                        p = false;
                    } else if (std::isnan(y)) {
                        p = true;
                    } else {
                        p = (y > b_tmp);
                    }

                    if (p) {
                        ex[loop_ub] = b_tmp;
                        b_idx[loop_ub] = b_i;
                    }
                }
            }

            if (b_idx.size(1) == b_indexPairs.size(1)) {
                inds.set_size(1, b_idx.size(1));
                loop_ub = b_idx.size(1);
                for (i = 0; i < loop_ub; i++) {
                    inds[i] = (static_cast<unsigned int>(b_idx[i]) == b_indexPairs[2 *
                                                                                   i]);
                }
            } else {
                binary_expand_op(inds, b_idx, b_indexPairs);
            }

            n = inds.size(1) - 1;
            olddi = 0;
            for (int b_i{0}; b_i <= n; b_i++) {
                if (inds[b_i]) {
                    olddi++;
                }
            }

            r3.set_size(1, olddi);
            olddi = 0;
            for (int b_i{0}; b_i <= n; b_i++) {
                if (inds[b_i]) {
                    r3[olddi] = b_i + 1;
                    olddi++;
                }
            }

            c_indexPairs.set_size(2, r3.size(1));
            loop_ub = r3.size(1);
            for (i = 0; i < loop_ub; i++) {
                c_indexPairs[2 * i] = b_indexPairs[2 * (r3[i] - 1)];
                c_indexPairs[2 * i + 1] = b_indexPairs[2 * (r3[i] - 1) + 1];
            }
        }

        indexPairs.set_size(c_indexPairs.size(1), 2);
        loop_ub = c_indexPairs.size(1);
        for (i = 0; i < 2; i++) {
            for (i1 = 0; i1 < loop_ub; i1++) {
                indexPairs[i1 + indexPairs.size(0) * i] = c_indexPairs[i + 2 * i1];
            }
        }
    }
}

static void mean(double y[2]) {
    static const double b_dv[8]{281.5, 281.5, 356.5, 356.5, 321.0, 161.0,
                                161.0, 321.0};

    for (int xi{0}; xi < 2; xi++) {
        int xpageoffset;
        xpageoffset = xi << 2;
        y[xi] = (((b_dv[xpageoffset] + b_dv[xpageoffset + 1]) + b_dv[xpageoffset + 2]) + b_dv[xpageoffset + 3]) / 4.0;
    }
}

static void mean(const ::coder::array<double, 2U> &x, double y[2]) {
    if (x.size(0) == 0) {
        y[0] = 0.0;
        y[1] = 0.0;
    } else {
        int firstBlockLength;
        int lastBlockLength;
        int nblocks;
        if (x.size(0) <= 1024) {
            firstBlockLength = x.size(0);
            lastBlockLength = 0;
            nblocks = 1;
        } else {
            firstBlockLength = 1024;
            nblocks = static_cast<int>(static_cast<unsigned int>(x.size(0)) >> 10);
            lastBlockLength = x.size(0) - (nblocks << 10);
            if (lastBlockLength > 0) {
                nblocks++;
            } else {
                lastBlockLength = 1024;
            }
        }

        for (int xi{0}; xi < 2; xi++) {
            int xpageoffset;
            xpageoffset = xi * x.size(0);
            y[xi] = x[xpageoffset];
            for (int k{2}; k <= firstBlockLength; k++) {
                y[xi] += x[(xpageoffset + k) - 1];
            }

            for (int ib{2}; ib <= nblocks; ib++) {
                double bsum;
                int hi;
                int xblockoffset;
                xblockoffset = xpageoffset + ((ib - 1) << 10);
                bsum = x[xblockoffset];
                if (ib == nblocks) {
                    hi = lastBlockLength;
                } else {
                    hi = 1024;
                }

                for (int k{2}; k <= hi; k++) {
                    bsum += x[(xblockoffset + k) - 1];
                }

                y[xi] += bsum;
            }
        }
    }

    y[0] /= static_cast<double>(x.size(0));
    y[1] /= static_cast<double>(x.size(0));
}

static poseGraph *optimizePoseGraph(HDMapping *aInstancePtr, poseGraph *b_poseGraph, robotics::core::internal::BlockMatrix *iobj_0, poseGraph *iobj_1) {
    robotics::core::internal::BlockMatrix lobj_1[2];
    robotics::core::internal::BlockMatrix *obj;
    robotics::core::internal::BlockMatrix *posesUpdated;
    robotics::core::internal::TrustRegionIndefiniteDogLegSE2 solver;
    sparse hessian;
    ::coder::array<double, 2U> T1;
    ::coder::array<double, 2U> varargin_1;
    ::coder::array<double, 1U> c_poseGraph;
    ::coder::array<bool, 1U> d_poseGraph;
    double R[9];
    double T1Offset[9];
    double c_T_tmp[9];
    double paramStruct_FirstNodePose[3];
    double R_idx_0;
    double R_idx_3;
    double T_tmp;
    double b_T_tmp;
    double paramStruct_FunctionTolerance;
    double paramStruct_GradientTolerance;
    double paramStruct_InitialTrustRegionRadius;
    double paramStruct_StepTolerance;
    double paramStruct_TrustRegionRadiusTolerance;
    double parsedResults_idx_0;
    double parsedResults_idx_1;
    int boffset;
    int coffset;
    int i;
    int loop_ub;
    bool expl_temp;
    nav::algs::internal::PoseGraphOptimizer::parseOptimizePoseGraphInputs(&R_idx_0, &R_idx_3, &paramStruct_FunctionTolerance, &expl_temp,
                                                                          &paramStruct_GradientTolerance, &paramStruct_StepTolerance,
                                                                          &paramStruct_InitialTrustRegionRadius, paramStruct_FirstNodePose,
                                                                          &paramStruct_TrustRegionRadiusTolerance, &parsedResults_idx_0);
    parsedResults_idx_0 = b_poseGraph->MaxNumEdges;
    parsedResults_idx_1 = b_poseGraph->MaxNumNodes;
    iobj_1->MaxNumEdges = parsedResults_idx_0;
    iobj_1->MaxNumNodes = parsedResults_idx_1;
    iobj_1->NumEdges = 0.0;
    iobj_1->NumNodes = 1.0;
    iobj_1->NumLoopClosureEdges = 0.0;
    parsedResults_idx_0 = iobj_1->MaxNumNodes;
    i = static_cast<int>(parsedResults_idx_0 * 3.0);
    (&(&(&(&(&iobj_0[0])[0])[0])[0])[0])[0].Matrix.set_size(i, 3);
    loop_ub = i * 3;
    for (i = 0; i < loop_ub; i++) {
        (&(&(&(&(&iobj_0[0])[0])[0])[0])[0])[0].Matrix[i] = 0.0;
    }

    (&(&(&(&(&iobj_0[0])[0])[0])[0])[0])[0].BlockSize[0] = 3.0;
    (&(&(&(&(&iobj_0[0])[0])[0])[0])[0])[0].BlockSize[1] = 3.0;
    (&(&(&(&(&iobj_0[0])[0])[0])[0])[0])[0].NumRowBlocks = parsedResults_idx_0;
    (&(&(&(&(&iobj_0[0])[0])[0])[0])[0])[0].NumColBlocks = 1.0;
    iobj_1->NodeEstimates = &(&(&(&(&(&iobj_0[0])[0])[0])[0])[0])[0];
    iobj_1->NodeEstimates->replaceBlock();
    loop_ub = static_cast<int>(iobj_1->MaxNumNodes);
    iobj_1->NodeMap.set_size(loop_ub);
    for (i = 0; i < loop_ub; i++) {
        iobj_1->NodeMap[i] = 0.0;
    }

    loop_ub = static_cast<int>(iobj_1->MaxNumNodes);
    iobj_1->NodeDims.set_size(loop_ub);
    for (i = 0; i < loop_ub; i++) {
        iobj_1->NodeDims[i] = 0.0;
    }

    loop_ub = static_cast<int>(iobj_1->MaxNumNodes);
    iobj_1->IsLandmarkNode.set_size(loop_ub);
    for (i = 0; i < loop_ub; i++) {
        iobj_1->IsLandmarkNode[i] = false;
    }

    iobj_1->NodeMap[0] = 1.0;
    iobj_1->NodeDims[0] = 3.0;
    loop_ub = static_cast<int>(iobj_1->MaxNumEdges);
    iobj_1->EdgeNodePairs.set_size(loop_ub, 2);
    loop_ub <<= 1;
    for (i = 0; i < loop_ub; i++) {
        iobj_1->EdgeNodePairs[i] = 0.0;
    }

    loop_ub = static_cast<int>(iobj_1->MaxNumEdges);
    iobj_1->LoopClosureEdgeNodePairs.set_size(loop_ub, 2);
    loop_ub <<= 1;
    for (i = 0; i < loop_ub; i++) {
        iobj_1->LoopClosureEdgeNodePairs[i] = 0.0;
    }

    loop_ub = static_cast<int>(iobj_1->MaxNumEdges);
    iobj_1->LoopClosureEdgeIDsInternal.set_size(1, loop_ub);
    for (i = 0; i < loop_ub; i++) {
        iobj_1->LoopClosureEdgeIDsInternal[i] = 0.0;
    }

    parsedResults_idx_0 = iobj_1->MaxNumEdges;
    i = static_cast<int>(parsedResults_idx_0 * 3.0);
    (&(&(&(&(&iobj_0[0])[0])[0])[0])[0])[1].Matrix.set_size(i, 3);
    loop_ub = i * 3;
    for (i = 0; i < loop_ub; i++) {
        (&(&(&(&(&iobj_0[0])[0])[0])[0])[0])[1].Matrix[i] = 0.0;
    }

    (&(&(&(&(&iobj_0[0])[0])[0])[0])[0])[1].BlockSize[0] = 3.0;
    (&(&(&(&(&iobj_0[0])[0])[0])[0])[0])[1].BlockSize[1] = 3.0;
    (&(&(&(&(&iobj_0[0])[0])[0])[0])[0])[1].NumRowBlocks = parsedResults_idx_0;
    (&(&(&(&(&iobj_0[0])[0])[0])[0])[0])[1].NumColBlocks = 1.0;
    iobj_1->EdgeMeasurements = &(&(&(&(&(&iobj_0[0])[0])[0])[0])[0])[1];
    parsedResults_idx_0 = iobj_1->MaxNumEdges;
    i = static_cast<int>(parsedResults_idx_0 * 3.0);
    (&(&(&(&(&iobj_0[0])[0])[0])[0])[0])[2].Matrix.set_size(i, 3);
    loop_ub = i * 3;
    for (i = 0; i < loop_ub; i++) {
        (&(&(&(&(&iobj_0[0])[0])[0])[0])[0])[2].Matrix[i] = 0.0;
    }

    (&(&(&(&(&iobj_0[0])[0])[0])[0])[0])[2].BlockSize[0] = 3.0;
    (&(&(&(&(&iobj_0[0])[0])[0])[0])[0])[2].BlockSize[1] = 3.0;
    (&(&(&(&(&iobj_0[0])[0])[0])[0])[0])[2].NumRowBlocks = parsedResults_idx_0;
    (&(&(&(&(&iobj_0[0])[0])[0])[0])[0])[2].NumColBlocks = 1.0;
    iobj_1->EdgeInfoMatrices = &(&(&(&(&(&iobj_0[0])[0])[0])[0])[0])[2];
    loop_ub = b_poseGraph->EdgeNodePairs.size(0) * 2;
    iobj_1->EdgeNodePairs.set_size(b_poseGraph->EdgeNodePairs.size(0), 2);
    c_poseGraph.set_size(loop_ub);
    for (i = 0; i < loop_ub; i++) {
        c_poseGraph[i] = b_poseGraph->EdgeNodePairs[i];
    }

    loop_ub = c_poseGraph.size(0);
    for (i = 0; i < loop_ub; i++) {
        iobj_1->EdgeNodePairs[i] = c_poseGraph[i];
    }

    loop_ub = b_poseGraph->LoopClosureEdgeNodePairs.size(0) * 2;
    iobj_1->LoopClosureEdgeNodePairs.set_size(b_poseGraph->LoopClosureEdgeNodePairs.size(0), 2);
    c_poseGraph.set_size(loop_ub);
    for (i = 0; i < loop_ub; i++) {
        c_poseGraph[i] = b_poseGraph->LoopClosureEdgeNodePairs[i];
    }

    loop_ub = c_poseGraph.size(0);
    for (i = 0; i < loop_ub; i++) {
        iobj_1->LoopClosureEdgeNodePairs[i] = c_poseGraph[i];
    }

    loop_ub = b_poseGraph->LoopClosureEdgeIDsInternal.size(1);
    iobj_1->LoopClosureEdgeIDsInternal.set_size(1,
                                                b_poseGraph->LoopClosureEdgeIDsInternal.size(1));
    c_poseGraph.set_size(loop_ub);
    for (i = 0; i < loop_ub; i++) {
        c_poseGraph[i] = b_poseGraph->LoopClosureEdgeIDsInternal[i];
    }

    loop_ub = c_poseGraph.size(0);
    for (i = 0; i < loop_ub; i++) {
        iobj_1->LoopClosureEdgeIDsInternal[i] = c_poseGraph[i];
    }

    obj = b_poseGraph->NodeEstimates;
    varargin_1.set_size(obj->Matrix.size(0), 3);
    loop_ub = obj->Matrix.size(0) * 3;
    for (i = 0; i < loop_ub; i++) {
        varargin_1[i] = obj->Matrix[i];
    }

    parsedResults_idx_0 = obj->BlockSize[0];
    parsedResults_idx_1 = obj->BlockSize[1];
    (&(&iobj_0[0])[0])[3].Matrix.set_size(varargin_1.size(0), 3);
    loop_ub = varargin_1.size(0) * 3;
    for (i = 0; i < loop_ub; i++) {
        (&(&iobj_0[0])[0])[3].Matrix[i] = varargin_1[i];
    }

    (&(&iobj_0[0])[0])[3].BlockSize[0] = parsedResults_idx_0;
    (&(&iobj_0[0])[0])[3].BlockSize[1] = parsedResults_idx_1;
    (&(&iobj_0[0])[0])[3].NumRowBlocks = static_cast<double>(varargin_1.size(0)) / parsedResults_idx_0;
    (&(&iobj_0[0])[0])[3].NumColBlocks = 3.0 / parsedResults_idx_1;
    iobj_1->NodeEstimates = &(&(&iobj_0[0])[0])[3];
    obj = b_poseGraph->EdgeMeasurements;
    varargin_1.set_size(obj->Matrix.size(0), 3);
    loop_ub = obj->Matrix.size(0) * 3;
    for (i = 0; i < loop_ub; i++) {
        varargin_1[i] = obj->Matrix[i];
    }

    parsedResults_idx_0 = obj->BlockSize[0];
    parsedResults_idx_1 = obj->BlockSize[1];
    (&(&iobj_0[0])[0])[4].Matrix.set_size(varargin_1.size(0), 3);
    loop_ub = varargin_1.size(0) * 3;
    for (i = 0; i < loop_ub; i++) {
        (&(&iobj_0[0])[0])[4].Matrix[i] = varargin_1[i];
    }

    (&(&iobj_0[0])[0])[4].BlockSize[0] = parsedResults_idx_0;
    (&(&iobj_0[0])[0])[4].BlockSize[1] = parsedResults_idx_1;
    (&(&iobj_0[0])[0])[4].NumRowBlocks = static_cast<double>(varargin_1.size(0)) / parsedResults_idx_0;
    (&(&iobj_0[0])[0])[4].NumColBlocks = 3.0 / parsedResults_idx_1;
    iobj_1->EdgeMeasurements = &(&(&iobj_0[0])[0])[4];
    obj = b_poseGraph->EdgeInfoMatrices;
    varargin_1.set_size(obj->Matrix.size(0), 3);
    loop_ub = obj->Matrix.size(0) * 3;
    for (i = 0; i < loop_ub; i++) {
        varargin_1[i] = obj->Matrix[i];
    }

    parsedResults_idx_0 = obj->BlockSize[0];
    parsedResults_idx_1 = obj->BlockSize[1];
    (&(&iobj_0[0])[0])[5].Matrix.set_size(varargin_1.size(0), 3);
    loop_ub = varargin_1.size(0) * 3;
    for (i = 0; i < loop_ub; i++) {
        (&(&iobj_0[0])[0])[5].Matrix[i] = varargin_1[i];
    }

    (&(&iobj_0[0])[0])[5].BlockSize[0] = parsedResults_idx_0;
    (&(&iobj_0[0])[0])[5].BlockSize[1] = parsedResults_idx_1;
    (&(&iobj_0[0])[0])[5].NumRowBlocks = static_cast<double>(varargin_1.size(0)) / parsedResults_idx_0;
    (&(&iobj_0[0])[0])[5].NumColBlocks = 3.0 / parsedResults_idx_1;
    iobj_1->EdgeInfoMatrices = &(&(&iobj_0[0])[0])[5];
    iobj_1->NumNodes = b_poseGraph->NumNodes;
    c_poseGraph.set_size(b_poseGraph->NodeMap.size(0));
    loop_ub = b_poseGraph->NodeMap.size(0);
    for (i = 0; i < loop_ub; i++) {
        c_poseGraph[i] = b_poseGraph->NodeMap[i];
    }

    iobj_1->NodeMap.set_size(c_poseGraph.size(0));
    loop_ub = c_poseGraph.size(0);
    for (i = 0; i < loop_ub; i++) {
        iobj_1->NodeMap[i] = c_poseGraph[i];
    }

    c_poseGraph.set_size(b_poseGraph->NodeDims.size(0));
    loop_ub = b_poseGraph->NodeDims.size(0);
    for (i = 0; i < loop_ub; i++) {
        c_poseGraph[i] = b_poseGraph->NodeDims[i];
    }

    iobj_1->NodeDims.set_size(c_poseGraph.size(0));
    loop_ub = c_poseGraph.size(0);
    for (i = 0; i < loop_ub; i++) {
        iobj_1->NodeDims[i] = c_poseGraph[i];
    }

    d_poseGraph.set_size(b_poseGraph->IsLandmarkNode.size(0));
    loop_ub = b_poseGraph->IsLandmarkNode.size(0);
    for (i = 0; i < loop_ub; i++) {
        d_poseGraph[i] = b_poseGraph->IsLandmarkNode[i];
    }

    iobj_1->IsLandmarkNode.set_size(d_poseGraph.size(0));
    loop_ub = d_poseGraph.size(0);
    for (i = 0; i < loop_ub; i++) {
        iobj_1->IsLandmarkNode[i] = d_poseGraph[i];
    }

    iobj_1->NumEdges = b_poseGraph->NumEdges;
    iobj_1->NumLoopClosureEdges = b_poseGraph->NumLoopClosureEdges;
    T_tmp = std::sin(paramStruct_FirstNodePose[2]);
    b_T_tmp = std::cos(paramStruct_FirstNodePose[2]);
    solver.TimeObj.StartTime.tv_sec = 0.0;
    solver.TimeObj.StartTime.tv_nsec = 0.0;
    solver.MaxNumIteration = R_idx_0;
    solver.MaxTime = R_idx_3;
    solver.GradientTolerance = paramStruct_GradientTolerance;
    solver.StepTolerance = paramStruct_StepTolerance;
    solver.FunctionTolerance = paramStruct_FunctionTolerance;
    solver.InitialTrustRegionRadius = paramStruct_InitialTrustRegionRadius;
    solver.TrustRegionRadiusTolerance = paramStruct_TrustRegionRadiusTolerance;
    R_idx_0 = iobj_1->NumEdges;
    if (R_idx_0 < 1.0) {
        loop_ub = 0;
    } else {
        loop_ub = static_cast<int>(R_idx_0);
    }

    solver.ExtraArgs.edgeNodePairs.set_size(loop_ub, 2);
    for (i = 0; i < 2; i++) {
        for (boffset = 0; boffset < loop_ub; boffset++) {
            solver.ExtraArgs.edgeNodePairs[boffset +
                                           solver.ExtraArgs.edgeNodePairs.size(0) * i] = iobj_1->EdgeNodePairs[boffset + iobj_1->EdgeNodePairs.size(0) * i];
        }
    }

    R_idx_0 = iobj_1->NumNodes * 3.0;
    if (R_idx_0 < 1.0) {
        loop_ub = 0;
    } else {
        loop_ub = static_cast<int>(R_idx_0);
    }

    varargin_1.set_size(loop_ub, 3);
    for (i = 0; i < 3; i++) {
        for (boffset = 0; boffset < loop_ub; boffset++) {
            varargin_1[boffset + varargin_1.size(0) * i] = iobj_1->NodeEstimates->Matrix[boffset + iobj_1->NodeEstimates->Matrix.size(0) * i];
        }
    }

    R_idx_0 = iobj_1->NumEdges * 3.0;
    if (R_idx_0 < 1.0) {
        loop_ub = 0;
    } else {
        loop_ub = static_cast<int>(R_idx_0);
    }

    solver.ExtraArgs.edgeInfoMats.set_size(loop_ub, 3);
    for (i = 0; i < 3; i++) {
        for (boffset = 0; boffset < loop_ub; boffset++) {
            solver.ExtraArgs.edgeInfoMats[boffset +
                                          solver.ExtraArgs.edgeInfoMats.size(0) * i] =
                iobj_1->EdgeInfoMatrices->Matrix[boffset + iobj_1->EdgeInfoMatrices->Matrix.size(0) * i];
        }
    }

    R_idx_0 = iobj_1->NumEdges * 3.0;
    if (R_idx_0 < 1.0) {
        loop_ub = 0;
    } else {
        loop_ub = static_cast<int>(R_idx_0);
    }

    solver.ExtraArgs.edgeMeasurements.set_size(loop_ub, 3);
    for (i = 0; i < 3; i++) {
        for (boffset = 0; boffset < loop_ub; boffset++) {
            solver.ExtraArgs.edgeMeasurements[boffset +
                                              solver.ExtraArgs.edgeMeasurements.size(0) * i] =
                iobj_1->EdgeMeasurements->Matrix[boffset + iobj_1->EdgeMeasurements->Matrix.size(0) * i];
        }
    }

    solver.ExtraArgs.tformSize[0] = 3.0;
    solver.ExtraArgs.infoMatSize[0] = 3.0;
    solver.ExtraArgs.tformSize[1] = 3.0;
    solver.ExtraArgs.infoMatSize[1] = 3.0;
    solver.ExtraArgs.poseDeltaLength = 3.0;
    solver.ExtraArgs.nodeMap.set_size(iobj_1->NodeMap.size(0));
    loop_ub = iobj_1->NodeMap.size(0);
    for (i = 0; i < loop_ub; i++) {
        solver.ExtraArgs.nodeMap[i] = iobj_1->NodeMap[i];
    }

    solver.ExtraArgs.nodeDims.set_size(iobj_1->NodeDims.size(0));
    loop_ub = iobj_1->NodeDims.size(0);
    for (i = 0; i < loop_ub; i++) {
        solver.ExtraArgs.nodeDims[i] = iobj_1->NodeDims[i];
    }

    solver.ExtraArgs.IsLandmarkNode.set_size(iobj_1->IsLandmarkNode.size(0));
    loop_ub = iobj_1->IsLandmarkNode.size(0);
    for (i = 0; i < loop_ub; i++) {
        solver.ExtraArgs.IsLandmarkNode[i] = iobj_1->IsLandmarkNode[i];
    }

    solver.solve(aInstancePtr, varargin_1, &lobj_1[0], &posesUpdated,
                 &parsedResults_idx_0, &parsedResults_idx_1, &R_idx_0,
                 &hessian);
    parsedResults_idx_0 = posesUpdated->BlockSize[0] * 0.0 + 1.0;
    parsedResults_idx_1 = posesUpdated->BlockSize[1] * 0.0 + 1.0;
    R_idx_0 = (parsedResults_idx_0 + posesUpdated->BlockSize[0]) - 1.0;
    if (parsedResults_idx_0 > R_idx_0) {
        loop_ub = 0;
    } else {
        loop_ub = static_cast<int>(R_idx_0);
    }

    R_idx_0 = (parsedResults_idx_1 + posesUpdated->BlockSize[1]) - 1.0;
    if (parsedResults_idx_1 > R_idx_0) {
        coffset = 0;
    } else {
        coffset = static_cast<int>(R_idx_0);
    }

    T1.set_size(loop_ub, coffset);
    for (i = 0; i < coffset; i++) {
        for (boffset = 0; boffset < loop_ub; boffset++) {
            T1[boffset + T1.size(0) * i] = posesUpdated->Matrix[boffset +
                                                                posesUpdated->Matrix.size(0) * i];
        }
    }

    R_idx_0 = T1[0];
    parsedResults_idx_0 = T1[T1.size(0)];
    parsedResults_idx_1 = T1[1];
    R_idx_3 = T1[T1.size(0) + 1];
    c_T_tmp[0] = b_T_tmp;
    c_T_tmp[3] = -T_tmp;
    c_T_tmp[6] = paramStruct_FirstNodePose[0];
    c_T_tmp[1] = T_tmp;
    c_T_tmp[4] = b_T_tmp;
    c_T_tmp[7] = paramStruct_FirstNodePose[1];
    c_T_tmp[2] = 0.0;
    c_T_tmp[5] = 0.0;
    c_T_tmp[8] = 1.0;
    R[0] = R_idx_0;
    R[1] = parsedResults_idx_0;
    R[6] = -R_idx_0 * T1[T1.size(0) * 2] + -parsedResults_idx_1 * T1[T1.size(0) * 2 + 1];
    R[3] = parsedResults_idx_1;
    R[4] = R_idx_3;
    R[7] = -parsedResults_idx_0 * T1[T1.size(0) * 2] + -R_idx_3 * T1[T1.size(0) * 2 + 1];
    R[2] = 0.0;
    R[5] = 0.0;
    R[8] = 1.0;
    for (i = 0; i < 3; i++) {
        R_idx_0 = c_T_tmp[i];
        parsedResults_idx_0 = c_T_tmp[i + 3];
        parsedResults_idx_1 = c_T_tmp[i + 6];
        for (boffset = 0; boffset < 3; boffset++) {
            T1Offset[i + 3 * boffset] = (R_idx_0 * R[3 * boffset] +
                                         parsedResults_idx_0 * R[3 * boffset + 1]) +
                                        parsedResults_idx_1 * R
                                                                  [3 * boffset + 2];
        }
    }

    R_idx_0 = iobj_1->NumNodes;
    i = static_cast<int>(R_idx_0);
    for (int b_i{0}; b_i < i; b_i++) {
        int y_size_idx_1;
        posesUpdated->extractBlock(static_cast<double>(b_i) + 1.0, T1);
        loop_ub = T1.size(1);
        y_size_idx_1 = T1.size(1);
        for (int j{0}; j < loop_ub; j++) {
            coffset = j * 3;
            boffset = j * T1.size(0);
            for (int c_i{0}; c_i < 3; c_i++) {
                c_T_tmp[coffset + c_i] = (T1Offset[c_i] * T1[boffset] + T1Offset[c_i + 3] * T1[boffset + 1]) + T1Offset[c_i + 6] * T1[boffset + 2];
            }
        }

        parsedResults_idx_0 = posesUpdated->BlockSize[0] * ((static_cast<double>(b_i) + 1.0) - 1.0) + 1.0;
        R_idx_0 = (parsedResults_idx_0 + posesUpdated->BlockSize[0]) - 1.0;
        if (parsedResults_idx_0 > R_idx_0) {
            boffset = 1;
        } else {
            boffset = static_cast<int>(parsedResults_idx_0);
        }

        for (loop_ub = 0; loop_ub < y_size_idx_1; loop_ub++) {
            posesUpdated->Matrix[(boffset + posesUpdated->Matrix.size(0) * loop_ub) - 1] = c_T_tmp[3 * loop_ub];
            posesUpdated->Matrix[boffset + posesUpdated->Matrix.size(0) * loop_ub] = c_T_tmp[3 * loop_ub + 1];
            posesUpdated->Matrix[(boffset + posesUpdated->Matrix.size(0) * loop_ub) + 1] = c_T_tmp[3 * loop_ub + 2];
        }
    }

    varargin_1.set_size(posesUpdated->Matrix.size(0), 3);
    loop_ub = posesUpdated->Matrix.size(0) * 3;
    for (i = 0; i < loop_ub; i++) {
        varargin_1[i] = posesUpdated->Matrix[i];
    }

    loop_ub = varargin_1.size(0);
    for (i = 0; i < 3; i++) {
        for (boffset = 0; boffset < loop_ub; boffset++) {
            iobj_1->NodeEstimates->Matrix[boffset + iobj_1->NodeEstimates->Matrix.size(0) * i] = varargin_1[boffset +
                                                                                                            varargin_1.size(0) * i];
        }
    }

    return iobj_1;
}

static void padarray(const ::coder::array<unsigned char, 2U> &varargin_1,
                     const double varargin_2[2], ::coder::array<unsigned char, 2U> &b) {
    if ((varargin_1.size(0) == 0) || (varargin_1.size(1) == 0)) {
        double sizeB_idx_0;
        double sizeB_idx_1;
        int loop_ub;
        sizeB_idx_0 = static_cast<double>(varargin_1.size(0)) + varargin_2[0];
        sizeB_idx_1 = static_cast<double>(varargin_1.size(1)) + varargin_2[1];
        b.set_size(static_cast<int>(sizeB_idx_0), static_cast<int>(sizeB_idx_1));
        loop_ub = static_cast<int>(sizeB_idx_0) * static_cast<int>(sizeB_idx_1);
        for (int i{0}; i < loop_ub; i++) {
            b[i] = 0U;
        }
    } else {
        int i;
        int i1;
        int loop_ub;
        b.set_size(static_cast<int>(static_cast<double>(varargin_1.size(0)) +
                                    varargin_2[0]),
                   static_cast<int>(static_cast<double>(varargin_1.size(1)) + varargin_2[1]));
        i = static_cast<int>(varargin_2[1]);
        for (int j{0}; j < i; j++) {
            loop_ub = b.size(0);
            for (int b_i{0}; b_i < loop_ub; b_i++) {
                b[b_i + b.size(0) * j] = 0U;
            }
        }

        loop_ub = static_cast<int>(varargin_2[1]) + 1;
        i1 = b.size(1);
        for (int j{loop_ub}; j <= i1; j++) {
            int i2;
            i2 = static_cast<int>(varargin_2[0]);
            for (int b_i{0}; b_i < i2; b_i++) {
                b[b_i + b.size(0) * (j - 1)] = 0U;
            }
        }

        loop_ub = varargin_1.size(1);
        i1 = varargin_1.size(0);
        for (int j{0}; j < loop_ub; j++) {
            for (int b_i{0}; b_i < i1; b_i++) {
                b[(b_i + static_cast<int>(varargin_2[0])) + b.size(0) * (j + i)] =
                    varargin_1[b_i + varargin_1.size(0) * j];
            }
        }
    }
}

static void poly2mask(bool BW[307200]) {
    static const double b_dv[8]{80.0, 80.0, 543.5, 543.5, 356.5, 356.5, 281.5,
                                281.5};

    static const double b_dv1[8]{234.0, 0.5, 0.5, 234.0, 234.0, 161.0, 161.0,
                                 234.0};

    ::coder::array<double, 2U> xLinePts;
    ::coder::array<double, 2U> yLinePts;
    double xNew[9];
    double yNew[9];
    double dy_tmp;
    double m;
    double xVal;
    int maxY[640];
    int minY[640];
    int b_i;
    int borderPosition;
    int borderSize;
    int i;
    int tempDX;
    int tempDY;
    std::memset(&xNew[0], 0, 9U * sizeof(double));
    std::memset(&yNew[0], 0, 9U * sizeof(double));
    xNew[8] = 80.0;
    std::copy(&b_dv[0], &b_dv[8], &xNew[0]);
    std::copy(&b_dv1[0], &b_dv1[8], &yNew[0]);
    yNew[8] = 234.0;
    std::memset(&BW[0], 0, 307200U * sizeof(bool));
    std::memset(&minY[0], 0, 640U * sizeof(int));
    std::memset(&maxY[0], 0, 640U * sizeof(int));
    borderSize = 0;
    for (i = 0; i < 9; i++) {
        xVal = std::floor(5.0 * (xNew[i] - 0.5) + 0.5) + 1.0;
        xNew[i] = xVal;
        dy_tmp = std::floor(5.0 * (yNew[i] - 0.5) + 0.5) + 1.0;
        yNew[i] = dy_tmp;
        if (i + 1 > 1) {
            tempDX = static_cast<int>(std::round(std::abs(xVal - xNew[i - 1])));
            tempDY = static_cast<int>(std::round(std::abs(dy_tmp - yNew[i - 1])));
            if (tempDX >= tempDY) {
                borderSize = (borderSize + tempDX) + 1;
            } else {
                borderSize = (borderSize + tempDY) + 1;
            }
        }
    }

    yLinePts.set_size(1, borderSize);
    xLinePts.set_size(1, borderSize);
    for (b_i = 0; b_i < borderSize; b_i++) {
        yLinePts[b_i] = 0.0;
        xLinePts[b_i] = 0.0;
    }

    borderPosition = 0;
    for (i = 0; i < 8; i++) {
        double b_y1;
        double dy_tmp_tmp;
        double x1;
        x1 = xNew[i];
        b_y1 = yNew[i];
        xVal = xNew[i + 1];
        m = xVal - x1;
        tempDX = static_cast<int>(std::round(std::abs(m)));
        dy_tmp_tmp = yNew[i + 1];
        dy_tmp = dy_tmp_tmp - b_y1;
        tempDY = static_cast<int>(std::round(std::abs(dy_tmp)));
        if ((tempDX == 0) && (tempDY == 0)) {
            xLinePts[borderPosition] = x1;
            yLinePts[borderPosition] = b_y1;
            borderPosition++;
        } else if (tempDX >= tempDY) {
            m = dy_tmp / m;
            if (xVal > x1) {
                b_i = static_cast<int>(xVal + (1.0 - x1));
                for (tempDX = 0; tempDX < b_i; tempDX++) {
                    xVal = x1 + static_cast<double>(tempDX);
                    tempDY = borderPosition + tempDX;
                    yLinePts[tempDY] = std::round(b_y1 + m * (xVal - x1));
                    xLinePts[tempDY] = xVal;
                }

                borderPosition += b_i;
            } else {
                b_i = static_cast<int>(-(xVal + (-1.0 - x1)));
                for (tempDX = 0; tempDX < b_i; tempDX++) {
                    xVal = x1 - static_cast<double>(tempDX);
                    tempDY = borderPosition + tempDX;
                    yLinePts[tempDY] = std::round(b_y1 + m * (xVal - x1));
                    xLinePts[tempDY] = xVal;
                }

                borderPosition += b_i;
            }
        } else {
            m /= dy_tmp;
            if (dy_tmp_tmp > b_y1) {
                b_i = static_cast<int>(dy_tmp_tmp + (1.0 - b_y1));
                for (tempDX = 0; tempDX < b_i; tempDX++) {
                    xVal = b_y1 + static_cast<double>(tempDX);
                    tempDY = borderPosition + tempDX;
                    xLinePts[tempDY] = std::round(x1 + m * (xVal - b_y1));
                    yLinePts[tempDY] = xVal;
                }

                borderPosition += b_i;
            } else {
                b_i = static_cast<int>(-(dy_tmp_tmp + (-1.0 - b_y1)));
                for (tempDX = 0; tempDX < b_i; tempDX++) {
                    xVal = b_y1 - static_cast<double>(tempDX);
                    tempDY = borderPosition + tempDX;
                    xLinePts[tempDY] = std::round(x1 + m * (xVal - b_y1));
                    yLinePts[tempDY] = xVal;
                }

                borderPosition += b_i;
            }
        }
    }

    for (tempDX = 0; tempDX <= borderSize - 2; tempDX++) {
        xVal = xLinePts[tempDX];
        m = xLinePts[tempDX + 1] - xVal;
        if (std::abs(m) >= 1.0) {
            dy_tmp = std::fmin(yLinePts[tempDX], yLinePts[tempDX + 1]);
            yLinePts[tempDX] = dy_tmp;
            if (m < 0.0) {
                xVal--;
                xLinePts[tempDX] = xVal;
            }

            xVal = (xVal + 2.0) / 5.0;
            if (std::abs(xVal - std::floor(xVal)) < 0.004) {
                xLinePts[tempDX] = xVal;
                dy_tmp = std::ceil((dy_tmp + 2.0) / 5.0);
                yLinePts[tempDX] = dy_tmp;
                tempDY = static_cast<int>(std::round(xVal)) - 1;
                borderPosition = static_cast<int>(dy_tmp) + 1;
                if ((tempDY + 2 >= 2) && (tempDY + 2 <= 641)) {
                    if (static_cast<int>(dy_tmp) + 1 > 481) {
                        if (minY[tempDY] == 0) {
                            minY[tempDY] = 481;
                        } else if (minY[tempDY] > 481) {
                            minY[tempDY] = 481;
                        }

                        if (maxY[tempDY] == 0) {
                            maxY[tempDY] = 481;
                        } else if (maxY[tempDY] < 481) {
                            maxY[tempDY] = 481;
                        }
                    } else {
                        if (static_cast<int>(dy_tmp) + 1 <= 2) {
                            borderPosition = 2;
                        }

                        i = (borderPosition + 480 * tempDY) - 2;
                        BW[i] = !BW[i];
                        if (minY[tempDY] == 0) {
                            minY[tempDY] = borderPosition - 1;
                        } else if (minY[tempDY] > borderPosition - 1) {
                            minY[tempDY] = borderPosition - 1;
                        }

                        if (maxY[tempDY] == 0) {
                            maxY[tempDY] = borderPosition;
                        } else if (maxY[tempDY] < borderPosition) {
                            maxY[tempDY] = borderPosition;
                        }
                    }
                }
            }
        }
    }

    for (tempDX = 0; tempDX < 640; tempDX++) {
        bool pixel;
        pixel = false;
        b_i = minY[tempDX];
        tempDY = maxY[tempDX] - 1;
        for (borderPosition = b_i; borderPosition <= tempDY; borderPosition++) {
            i = (borderPosition + 480 * tempDX) - 1;
            if (BW[i]) {
                pixel = !pixel;
            }

            BW[i] = pixel;
        }
    }
}

static void tic(HDMapping *aInstancePtr, double *tstart_tv_sec, double *tstart_tv_nsec) {
    coderTimespec b_timespec;
    constructWorldMapStackData *localSD;
    localSD = aInstancePtr->getStackData();
    if (!localSD->pd->freq_not_empty) {
        localSD->pd->freq_not_empty = true;
        coderInitTimeFunctions(&localSD->pd->freq);
    }

    coderTimeClockGettimeMonotonic(&b_timespec, localSD->pd->freq);
    *tstart_tv_sec = b_timespec.tv_sec;
    *tstart_tv_nsec = b_timespec.tv_nsec;
}

static double toc(HDMapping *aInstancePtr, double tstart_tv_sec, double tstart_tv_nsec) {
    coderTimespec b_timespec;
    constructWorldMapStackData *localSD;
    localSD = aInstancePtr->getStackData();
    if (!localSD->pd->freq_not_empty) {
        localSD->pd->freq_not_empty = true;
        coderInitTimeFunctions(&localSD->pd->freq);
    }

    coderTimeClockGettimeMonotonic(&b_timespec, localSD->pd->freq);
    return (b_timespec.tv_sec - tstart_tv_sec) + (b_timespec.tv_nsec -
                                                  tstart_tv_nsec) /
                                                     1.0E+9;
}

namespace vision {
namespace internal {
namespace geotrans {
static void computeRigid2d(const ::coder::array<double, 3U> &points,
                           double T[9]) {
    ::coder::array<double, 2U> normPoints1;
    ::coder::array<double, 2U> normPoints2;
    double C[4];
    double U[4];
    double V[4];
    double centroid1[2];
    double centroid2[2];
    double bkj;
    double d;
    double d1;
    double d2;
    double d3;
    double d4;
    int acoef;
    int boffset;
    int coffset;
    signed char csz[2];
    bool p;
    normPoints1.set_size(points.size(0), 2);
    acoef = points.size(0);
    for (boffset = 0; boffset < 2; boffset++) {
        for (coffset = 0; coffset < acoef; coffset++) {
            normPoints1[coffset + normPoints1.size(0) * boffset] =
                points[coffset + points.size(0) * boffset];
        }
    }

    mean(normPoints1, centroid1);
    normPoints1.set_size(points.size(0), 2);
    acoef = points.size(0);
    for (boffset = 0; boffset < 2; boffset++) {
        for (coffset = 0; coffset < acoef; coffset++) {
            normPoints1[coffset + normPoints1.size(0) * boffset] = points
                [(coffset + points.size(0) * boffset) + points.size(0) * 2];
        }
    }

    mean(normPoints1, centroid2);
    normPoints1.set_size(points.size(0), 2);
    if (points.size(0) != 0) {
        acoef = (points.size(0) != 1);
        for (int k{0}; k < 2; k++) {
            boffset = normPoints1.size(0) - 1;
            for (coffset = 0; coffset <= boffset; coffset++) {
                normPoints1[coffset + normPoints1.size(0) * k] = points[acoef *
                                                                            coffset +
                                                                        points.size(0) * k] -
                                                                 centroid1[k];
            }
        }
    }

    normPoints2.set_size(points.size(0), 2);
    if (points.size(0) != 0) {
        acoef = (points.size(0) != 1);
        for (int k{0}; k < 2; k++) {
            boffset = normPoints2.size(0) - 1;
            for (coffset = 0; coffset <= boffset; coffset++) {
                normPoints2[coffset + normPoints2.size(0) * k] = points[(acoef * coffset + points.size(0) * k) + points.size(0) * 2] -
                                                                 centroid2[k];
            }
        }
    }

    acoef = normPoints1.size(0);
    for (int j{0}; j < 2; j++) {
        coffset = j << 1;
        boffset = j * normPoints2.size(0);
        C[coffset] = 0.0;
        C[coffset + 1] = 0.0;
        for (int k{0}; k < acoef; k++) {
            bkj = normPoints2[boffset + k];
            C[coffset] += normPoints1[k] * bkj;
            C[coffset + 1] += normPoints1[normPoints1.size(0) + k] * bkj;
        }
    }

    p = true;
    if (std::isinf(C[0]) || std::isnan(C[0]) || (std::isinf(C[1]) || std::isnan(C[1]))) {
        p = false;
    }

    if ((!p) || (std::isinf(C[2]) || std::isnan(C[2]))) {
        p = false;
    }

    if ((!p) || (std::isinf(C[3]) || std::isnan(C[3]))) {
        p = false;
    }

    if (p) {
        double s[2];
        ::buildMapping::coder::internal::svd(C, U, s, V);
    } else {
        U[0] = rtNaN;
        V[0] = rtNaN;
        U[1] = rtNaN;
        V[1] = rtNaN;
        U[2] = rtNaN;
        V[2] = rtNaN;
        U[3] = rtNaN;
        V[3] = rtNaN;
    }

    bkj = V[0];
    d = V[2];
    d1 = V[1];
    d2 = V[3];
    for (boffset = 0; boffset < 2; boffset++) {
        d3 = U[boffset + 2];
        d4 = U[boffset];
        C[boffset] = d4 * bkj + d3 * d;
        C[boffset + 2] = d4 * d1 + d3 * d2;
        csz[boffset] = static_cast<signed char>(boffset + 1);
    }

    acoef = 0;
    if (std::abs(C[1]) > std::abs(C[0])) {
        acoef = 1;
    }

    if (C[acoef] != 0.0) {
        if (acoef != 0) {
            csz[0] = 2;
            bkj = C[0];
            C[0] = C[1];
            C[1] = bkj;
            bkj = C[2];
            C[2] = C[3];
            C[3] = bkj;
        }

        C[1] /= C[0];
    }

    if (C[2] != 0.0) {
        C[3] += C[1] * -C[2];
    }

    bkj = C[0] * C[3];
    if (csz[0] > 1) {
        bkj = -bkj;
    }

    if (std::isnan(bkj)) {
        C[3] = rtNaN;
    } else if (bkj < 0.0) {
        C[3] = -1.0;
    } else {
        C[3] = (bkj > 0.0);
    }

    bkj = U[0];
    d = U[2];
    d1 = U[1];
    d2 = U[3];
    for (boffset = 0; boffset < 2; boffset++) {
        double d5;
        d3 = V[boffset + 2];
        d4 = V[boffset];
        d5 = d4 + d3 * 0.0;
        d3 = d4 * 0.0 + d3 * C[3];
        C[boffset] = d5 * bkj + d3 * d;
        C[boffset + 2] = d5 * d1 + d3 * d2;
    }

    std::memset(&T[0], 0, 9U * sizeof(double));
    T[8] = 1.0;
    T[0] = C[0];
    T[1] = C[2];
    T[2] = centroid2[0] - (C[0] * centroid1[0] + centroid1[1] * C[2]);
    T[3] = C[1];
    T[4] = C[3];
    T[5] = centroid2[1] - (centroid1[0] * C[1] + centroid1[1] * C[3]);
}

static void evaluateTform2d(const double tform[9], const ::coder::array<double, 3U> &points, ::coder::array<double, 1U> &dis) {
    ::coder::array<double, 2U> b_points;
    ::coder::array<double, 2U> c_points;
    ::coder::array<double, 2U> delta;
    ::coder::array<double, 2U> pt1h;
    ::coder::array<double, 1U> y;
    ::coder::array<int, 1U> r1;
    ::coder::array<bool, 1U> r;
    int loop_ub;
    int nx;
    signed char input_sizes_idx_1;
    signed char sizes_idx_1;
    bool empty_non_axis_sizes;
    if (points.size(0) != 0) {
        nx = points.size(0);
    } else {
        nx = 0;
    }

    empty_non_axis_sizes = (nx == 0);
    if (empty_non_axis_sizes || (points.size(0) != 0)) {
        input_sizes_idx_1 = 2;
    } else {
        input_sizes_idx_1 = 0;
    }

    if (empty_non_axis_sizes || (points.size(0) != 0)) {
        sizes_idx_1 = 1;
    } else {
        sizes_idx_1 = 0;
    }

    b_points.set_size(points.size(0), 2);
    loop_ub = points.size(0);
    for (int i{0}; i < 2; i++) {
        for (int b_i{0}; b_i < loop_ub; b_i++) {
            b_points[b_i + b_points.size(0) * i] = points[b_i + points.size(0) * i];
        }
    }

    c_points.set_size(nx, input_sizes_idx_1 + sizes_idx_1);
    loop_ub = input_sizes_idx_1;
    for (int i{0}; i < loop_ub; i++) {
        for (int b_i{0}; b_i < nx; b_i++) {
            c_points[b_i + c_points.size(0) * i] = b_points[b_i + nx * i];
        }
    }

    loop_ub = sizes_idx_1;
    for (int i{0}; i < loop_ub; i++) {
        for (int b_i{0}; b_i < nx; b_i++) {
            c_points[b_i + c_points.size(0) * input_sizes_idx_1] = 1.0;
        }
    }

    ::buildMapping::coder::internal::blas::mtimes(c_points, tform, pt1h);
    if (pt1h.size(0) == points.size(0)) {
        delta.set_size(pt1h.size(0), 2);
        loop_ub = pt1h.size(0);
        for (int i{0}; i < loop_ub; i++) {
            delta[i] = pt1h[i] / pt1h[i + pt1h.size(0) * 2];
        }

        loop_ub = pt1h.size(0);
        for (int i{0}; i < loop_ub; i++) {
            delta[i + delta.size(0)] = pt1h[i + pt1h.size(0)] / pt1h[i +
                                                                     pt1h.size(0) * 2];
        }

        delta.set_size(delta.size(0), 2);
        for (int i{0}; i < 2; i++) {
            loop_ub = delta.size(0);
            for (int b_i{0}; b_i < loop_ub; b_i++) {
                delta[b_i + delta.size(0) * i] = delta[b_i + delta.size(0) * i] - points[(b_i + points.size(0) * i) + points.size(0) * 2];
            }
        }
    } else {
        b_binary_expand_op(delta, pt1h, points);
    }

    dis.set_size(delta.size(0));
    nx = delta.size(0);
    for (loop_ub = 0; loop_ub < nx; loop_ub++) {
        dis[loop_ub] = rt_hypotd_snf(delta[loop_ub], delta[loop_ub +
                                                           delta.size(0)]);
    }

    nx = pt1h.size(0);
    y.set_size(pt1h.size(0));
    for (loop_ub = 0; loop_ub < nx; loop_ub++) {
        y[loop_ub] = std::abs(pt1h[loop_ub + pt1h.size(0) * 2]);
    }

    r.set_size(y.size(0));
    loop_ub = y.size(0);
    for (int i{0}; i < loop_ub; i++) {
        r[i] = (y[i] < 2.2204460492503131E-16);
    }

    loop_ub = r.size(0) - 1;
    nx = 0;
    for (int i{0}; i <= loop_ub; i++) {
        if (r[i]) {
            nx++;
        }
    }

    r1.set_size(nx);
    nx = 0;
    for (int i{0}; i <= loop_ub; i++) {
        if (r[i]) {
            r1[nx] = i + 1;
            nx++;
        }
    }

    loop_ub = r1.size(0);
    for (int i{0}; i < loop_ub; i++) {
        dis[r1[i] - 1] = rtInf;
    }
}
}  // namespace geotrans

namespace matchFeatures {
static void findNearestNeighbors(const ::coder::array<float, 2U>
                                     &scores,
                                 ::coder::array<unsigned int, 2U> &indexPairs, ::coder::array<float, 2U> &topTwoMetrics) {
    ::coder::array<float, 2U> xSorted;
    ::coder::array<float, 1U> ex;
    ::coder::array<int, 2U> iidx;
    ::coder::array<int, 2U> indices;
    ::coder::array<unsigned int, 2U> topTwoIndices;
    ::coder::array<unsigned int, 2U> y;
    ::coder::array<int, 1U> idx;
    int eint;
    int loop_ub;
    int n;
    xSorted.set_size(scores.size(0), scores.size(1));
    loop_ub = scores.size(0) * scores.size(1);
    for (int i{0}; i < loop_ub; i++) {
        xSorted[i] = scores[i];
    }

    n = 2;
    if (scores.size(1) < 2) {
        n = scores.size(1);
    }

    topTwoMetrics.set_size(n, scores.size(0));
    loop_ub = n * scores.size(0);
    for (int i{0}; i < loop_ub; i++) {
        topTwoMetrics[i] = 0.0F;
    }

    indices.set_size(n, scores.size(0));
    loop_ub = n * scores.size(0);
    for (int i{0}; i < loop_ub; i++) {
        indices[i] = 0;
    }

    if ((scores.size(0) == 0) || (scores.size(1) == 0)) {
        topTwoIndices.set_size(n, scores.size(0));
        loop_ub = n * scores.size(0);
        for (int i{0}; i < loop_ub; i++) {
            topTwoIndices[i] = 0U;
        }
    } else {
        if (n == 1) {
            ::buildMapping::coder::internal::minimum(scores, ex, idx);
            loop_ub = scores.size(0);
            for (int i{0}; i < loop_ub; i++) {
                topTwoMetrics[topTwoMetrics.size(0) * i] = ex[i];
            }

            loop_ub = scores.size(0);
            for (int i{0}; i < loop_ub; i++) {
                indices[indices.size(0) * i] = idx[i];
            }
        } else {
            double t;
            t = std::frexp(static_cast<double>(scores.size(1)), &eint);
            if (t == 0.5) {
                t = static_cast<double>(eint) - 1.0;
            } else if ((eint == 1) && (t < 0.75)) {
                t = std::log(2.0 * t) / 0.69314718055994529;
            } else {
                t = std::log(t) / 0.69314718055994529 + static_cast<double>(eint);
            }

            if (n < t) {
                for (eint = 0; eint < n; eint++) {
                    ::buildMapping::coder::internal::minimum(xSorted, ex, idx);
                    loop_ub = topTwoMetrics.size(1);
                    for (int i{0}; i < loop_ub; i++) {
                        topTwoMetrics[eint + topTwoMetrics.size(0) * i] = ex[i];
                    }

                    loop_ub = indices.size(1);
                    for (int i{0}; i < loop_ub; i++) {
                        indices[eint + indices.size(0) * i] = idx[i];
                    }

                    if (xSorted.size(0) < 1) {
                        y.set_size(1, 0);
                    } else {
                        y.set_size(1, xSorted.size(0));
                        loop_ub = xSorted.size(0) - 1;
                        for (int i{0}; i <= loop_ub; i++) {
                            y[i] = static_cast<unsigned int>(i) + 1U;
                        }
                    }

                    iidx.set_size(1, y.size(1));
                    loop_ub = y.size(1);
                    for (int i{0}; i < loop_ub; i++) {
                        iidx[i] = static_cast<int>(y[i]) + xSorted.size(0) *
                                                               (indices[eint + indices.size(0) * i] - 1);
                    }

                    loop_ub = iidx.size(1);
                    for (int i{0}; i < loop_ub; i++) {
                        xSorted[iidx[i] - 1] = rtInfF;
                    }
                }
            } else {
                ::buildMapping::coder::internal::sort(xSorted, iidx);
                if (n < 1) {
                    loop_ub = 0;
                } else {
                    loop_ub = n;
                }

                topTwoMetrics.set_size(loop_ub, xSorted.size(0));
                eint = xSorted.size(0);
                for (int i{0}; i < eint; i++) {
                    for (int i1{0}; i1 < loop_ub; i1++) {
                        topTwoMetrics[i1 + topTwoMetrics.size(0) * i] = xSorted[i + xSorted.size(0) * i1];
                    }
                }

                if (n < 1) {
                    loop_ub = 0;
                } else {
                    loop_ub = n;
                }

                indices.set_size(loop_ub, iidx.size(0));
                eint = iidx.size(0);
                for (int i{0}; i < eint; i++) {
                    for (int i1{0}; i1 < loop_ub; i1++) {
                        indices[i1 + indices.size(0) * i] = iidx[i + iidx.size(0) *
                                                                         i1];
                    }
                }
            }
        }

        topTwoIndices.set_size(indices.size(0), indices.size(1));
        loop_ub = indices.size(0) * indices.size(1);
        for (int i{0}; i < loop_ub; i++) {
            topTwoIndices[i] = static_cast<unsigned int>(indices[i]);
        }
    }

    if (scores.size(0) < 1) {
        y.set_size(1, 0);
    } else {
        y.set_size(1, scores.size(0));
        loop_ub = scores.size(0) - 1;
        for (int i{0}; i <= loop_ub; i++) {
            y[i] = static_cast<unsigned int>(i) + 1U;
        }
    }

    indexPairs.set_size(2, y.size(1));
    loop_ub = y.size(1);
    for (int i{0}; i < loop_ub; i++) {
        indexPairs[2 * i] = y[i];
    }

    loop_ub = topTwoIndices.size(1);
    for (int i{0}; i < loop_ub; i++) {
        indexPairs[2 * i + 1] = topTwoIndices[topTwoIndices.size(0) * i];
    }
}
}  // namespace matchFeatures

namespace ransac {
static void msac(HDMapping *aInstancePtr, const ::coder::array<double, 3U> &allPoints, bool *isFound, double bestModelParams_data[], int bestModelParams_size[2], ::coder::array<bool, 1U> &inliers) {
    ::coder::array<double, 3U> b_allPoints;
    ::coder::array<double, 3U> b_samplePoints_data;
    ::coder::array<double, 1U> dis;
    ::coder::array<int, 1U> r;
    ::coder::array<bool, 1U> bestInliers;
    ::coder::array<bool, 1U> x;
    double modelParams[9];
    double samplePoints_data[8];
    double bestDis;
    double j;
    int idxTrial;
    int jlast;
    int k;
    int lastBlockLength;
    int nblocks;
    int numPts;
    int numTrials;
    int nz;
    int skipTrials;
    bool b_tmp_data[9];
    bool tmp_data[9];
    bool exitg1;
    bool isValidModel;
    numPts = allPoints.size(0);
    idxTrial = 1;
    numTrials = 1000;
    bestDis = 1.5 * static_cast<double>(allPoints.size(0));
    bestModelParams_size[0] = 0;
    bestModelParams_size[1] = 0;
    skipTrials = 0;
    bestInliers.set_size(allPoints.size(0));
    jlast = allPoints.size(0);
    for (lastBlockLength = 0; lastBlockLength < jlast; lastBlockLength++) {
        bestInliers[lastBlockLength] = false;
    }

    while ((idxTrial <= numTrials) && (skipTrials < 10000)) {
        double indices_data[2];
        double selectedLoc;
        indices_data[1] = 0.0;
        if (numPts <= 2) {
            indices_data[0] = 1.0;
            j = b_rand(aInstancePtr) * 2.0;
            j = std::floor(j);
            indices_data[1] = indices_data[static_cast<int>(j + 1.0) - 1];
            indices_data[static_cast<int>(j + 1.0) - 1] = 2.0;
        } else if (static_cast<double>(numPts) / 4.0 <= 2.0) {
            double loc_data_idx_0;
            double t;
            t = 0.0;
            selectedLoc = numPts;
            loc_data_idx_0 = 2.0 / static_cast<double>(numPts);
            j = b_rand(aInstancePtr);
            while (j > loc_data_idx_0) {
                t++;
                selectedLoc--;
                loc_data_idx_0 += (1.0 - loc_data_idx_0) * (2.0 / selectedLoc);
            }

            t++;
            j = b_rand(aInstancePtr);
            j = std::floor(j);
            indices_data[0] = 0.0;
            indices_data[static_cast<int>(j + 1.0) - 1] = t;
            selectedLoc = static_cast<double>(numPts) - t;
            loc_data_idx_0 = 1.0 / selectedLoc;
            j = b_rand(aInstancePtr);
            while (j > loc_data_idx_0) {
                t++;
                selectedLoc--;
                loc_data_idx_0 += (1.0 - loc_data_idx_0) * (1.0 / selectedLoc);
            }

            t++;
            j = b_rand(aInstancePtr) * 2.0;
            j = std::floor(j);
            indices_data[1] = indices_data[static_cast<int>(j + 1.0) - 1];
            indices_data[static_cast<int>(j + 1.0) - 1] = t;
        } else {
            double loc_data_idx_0;
            signed char hashTbl_data[2];
            hashTbl_data[0] = 0;
            hashTbl_data[1] = 0;
            selectedLoc = b_rand(aInstancePtr) * ((static_cast<double>(numPts) - 1.0) + 1.0);
            selectedLoc = std::floor(selectedLoc);
            if (selectedLoc == 0.0) {
                j = 0.0;
            } else {
                j = std::fmod(selectedLoc, 2.0);
                if (j == 0.0) {
                    j = 0.0;
                }
            }

            indices_data[0] = selectedLoc + 1.0;
            loc_data_idx_0 = selectedLoc;
            hashTbl_data[static_cast<int>(j + 1.0) - 1] = 1;
            jlast = hashTbl_data[static_cast<int>(std::fmod(static_cast<
                                                                double>(numPts) -
                                                                1.0,
                                                            2.0))];
            while ((jlast > 0) && (selectedLoc != static_cast<double>(numPts) - 1.0)) {
                jlast = 0;
            }

            if (jlast > 0) {
                jlast = 0;
            } else {
                jlast = numPts - 1;
            }

            selectedLoc = b_rand(aInstancePtr) * ((static_cast<double>(numPts) - 2.0) + 1.0);
            selectedLoc = std::floor(selectedLoc);
            if (selectedLoc == 0.0) {
                j = 0.0;
            } else {
                j = std::fmod(selectedLoc, 2.0);
                if (j == 0.0) {
                    j = 0.0;
                }
            }

            j = hashTbl_data[static_cast<int>(j + 1.0) - 1];
            while ((j > 0.0) && (loc_data_idx_0 != selectedLoc)) {
                j = 0.0;
            }

            if (j > 0.0) {
                indices_data[1] = static_cast<double>(jlast) + 1.0;
            } else {
                indices_data[1] = selectedLoc + 1.0;
            }
        }

        j = indices_data[0];
        selectedLoc = indices_data[1];
        for (lastBlockLength = 0; lastBlockLength < 2; lastBlockLength++) {
            samplePoints_data[4 * lastBlockLength] = allPoints[(static_cast<
                                                                    int>(j) +
                                                                allPoints.size(0) * 2 * lastBlockLength) -
                                                               1];
            samplePoints_data[4 * lastBlockLength + 1] = allPoints[(
                                                                       static_cast<int>(selectedLoc) + allPoints.size(0) * 2 *
                                                                                                           lastBlockLength) -
                                                                   1];
            samplePoints_data[4 * lastBlockLength + 2] = allPoints[((
                                                                        static_cast<int>(j) + allPoints.size(0)) +
                                                                    allPoints.size(0) *
                                                                        2 * lastBlockLength) -
                                                                   1];
            samplePoints_data[4 * lastBlockLength + 3] = allPoints[((
                                                                        static_cast<int>(selectedLoc) + allPoints.size(0)) +
                                                                    allPoints.size(0) * 2 * lastBlockLength) -
                                                                   1];
        }

        b_samplePoints_data.set(&samplePoints_data[0], 2, 2, 2);
        geotrans::computeRigid2d(b_samplePoints_data, modelParams);
        for (lastBlockLength = 0; lastBlockLength < 9; lastBlockLength++) {
            j = modelParams[lastBlockLength];
            tmp_data[lastBlockLength] = std::isinf(j);
            b_tmp_data[lastBlockLength] = std::isnan(j);
        }

        isValidModel = true;
        k = 0;
        exitg1 = false;
        while ((!exitg1) && (k <= 8)) {
            if (tmp_data[k] || b_tmp_data[k]) {
                isValidModel = false;
                exitg1 = true;
            } else {
                k++;
            }
        }

        if (isValidModel) {
            geotrans::evaluateTform2d(modelParams, allPoints, dis);
            nz = dis.size(0);
            for (nblocks = 0; nblocks < nz; nblocks++) {
                if (dis[nblocks] > 1.5) {
                    dis[nblocks] = 1.5;
                }
            }

            if (dis.size(0) == 0) {
                j = 0.0;
            } else {
                if (dis.size(0) <= 1024) {
                    jlast = dis.size(0);
                    lastBlockLength = 0;
                    nblocks = 1;
                } else {
                    jlast = 1024;
                    nblocks = static_cast<int>(static_cast<unsigned int>(dis.size(0)) >> 10);
                    lastBlockLength = dis.size(0) - (nblocks << 10);
                    if (lastBlockLength > 0) {
                        nblocks++;
                    } else {
                        lastBlockLength = 1024;
                    }
                }

                j = dis[0];
                for (k = 2; k <= jlast; k++) {
                    j += dis[k - 1];
                }

                for (int ib{2}; ib <= nblocks; ib++) {
                    jlast = (ib - 1) << 10;
                    selectedLoc = dis[jlast];
                    if (ib == nblocks) {
                        nz = lastBlockLength;
                    } else {
                        nz = 1024;
                    }

                    for (k = 2; k <= nz; k++) {
                        selectedLoc += dis[(jlast + k) - 1];
                    }

                    j += selectedLoc;
                }
            }

            if (j < bestDis) {
                bestDis = j;
                bestInliers.set_size(dis.size(0));
                jlast = dis.size(0);
                for (lastBlockLength = 0; lastBlockLength < jlast;
                     lastBlockLength++) {
                    bestInliers[lastBlockLength] = (dis[lastBlockLength] < 1.5);
                }

                bestModelParams_size[0] = 3;
                bestModelParams_size[1] = 3;
                std::copy(&modelParams[0], &modelParams[9],
                          &bestModelParams_data[0]);
                x.set_size(dis.size(0));
                jlast = dis.size(0);
                for (lastBlockLength = 0; lastBlockLength < jlast;
                     lastBlockLength++) {
                    x[lastBlockLength] = (dis[lastBlockLength] < 1.5);
                }

                jlast = x.size(0);
                if (x.size(0) == 0) {
                    nz = 0;
                } else {
                    nz = x[0];
                    for (k = 2; k <= jlast; k++) {
                        nz += x[k - 1];
                    }
                }

                j = rt_powd_snf(static_cast<double>(nz) / static_cast<double>(numPts), 2.0);
                if (j < 2.2204460492503131E-16) {
                    jlast = MAX_int32_T;
                } else {
                    jlast = static_cast<int>(std::ceil(-1.9999999999999996 / std::
                                                                                 log10(1.0 - j)));
                }

                if (numTrials > jlast) {
                    numTrials = jlast;
                }
            }

            idxTrial++;
        } else {
            skipTrials++;
        }
    }

    jlast = bestModelParams_size[0] * bestModelParams_size[1];
    for (lastBlockLength = 0; lastBlockLength < jlast; lastBlockLength++) {
        tmp_data[lastBlockLength] = std::isinf(bestModelParams_data[lastBlockLength]);
    }

    for (lastBlockLength = 0; lastBlockLength < jlast; lastBlockLength++) {
        b_tmp_data[lastBlockLength] = std::isnan(bestModelParams_data[lastBlockLength]);
    }

    x.set_size(jlast);
    for (lastBlockLength = 0; lastBlockLength < jlast; lastBlockLength++) {
        x[lastBlockLength] = ((!tmp_data[lastBlockLength]) &&
                              (!b_tmp_data[lastBlockLength]));
    }

    isValidModel = true;
    jlast = 1;
    exitg1 = false;
    while ((!exitg1) && (jlast <= x.size(0))) {
        if (!x[jlast - 1]) {
            isValidModel = false;
            exitg1 = true;
        } else {
            jlast++;
        }
    }

    if (isValidModel && (bestInliers.size(0) != 0)) {
        jlast = bestInliers.size(0);
        nz = bestInliers[0];
        for (k = 2; k <= jlast; k++) {
            nz += bestInliers[k - 1];
        }

        if (nz >= 2) {
            *isFound = true;
        } else {
            *isFound = false;
        }
    } else {
        *isFound = false;
    }

    if (*isFound) {
        bool guard1{false};

        nz = bestInliers.size(0) - 1;
        jlast = 0;
        for (nblocks = 0; nblocks <= nz; nblocks++) {
            if (bestInliers[nblocks]) {
                jlast++;
            }
        }

        r.set_size(jlast);
        jlast = 0;
        for (nblocks = 0; nblocks <= nz; nblocks++) {
            if (bestInliers[nblocks]) {
                r[jlast] = nblocks + 1;
                jlast++;
            }
        }

        b_allPoints.set_size(r.size(0), 2, 2);
        jlast = r.size(0);
        for (lastBlockLength = 0; lastBlockLength < 2; lastBlockLength++) {
            for (nz = 0; nz < 2; nz++) {
                for (nblocks = 0; nblocks < jlast; nblocks++) {
                    b_allPoints[(nblocks + b_allPoints.size(0) * nz) +
                                b_allPoints.size(0) * 2 * lastBlockLength] = allPoints
                        [((r[nblocks] + allPoints.size(0) * nz) + allPoints.size(0) * 2 * lastBlockLength) - 1];
                }
            }
        }

        geotrans::computeRigid2d(b_allPoints, modelParams);
        geotrans::evaluateTform2d(modelParams, allPoints, dis);
        nz = dis.size(0);
        for (nblocks = 0; nblocks < nz; nblocks++) {
            if (dis[nblocks] > 1.5) {
                dis[nblocks] = 1.5;
            }
        }

        bestModelParams_size[0] = 3;
        bestModelParams_size[1] = 3;
        std::copy(&modelParams[0], &modelParams[9], &bestModelParams_data[0]);
        inliers.set_size(dis.size(0));
        jlast = dis.size(0);
        for (lastBlockLength = 0; lastBlockLength < jlast; lastBlockLength++) {
            inliers[lastBlockLength] = (dis[lastBlockLength] < 1.5);
        }

        x.set_size(9);
        for (lastBlockLength = 0; lastBlockLength < 9; lastBlockLength++) {
            j = modelParams[lastBlockLength];
            x[lastBlockLength] = ((!std::isinf(j)) && (!std::isnan(j)));
        }

        isValidModel = true;
        jlast = 1;
        exitg1 = false;
        while ((!exitg1) && (jlast <= 9)) {
            if (!x[jlast - 1]) {
                isValidModel = false;
                exitg1 = true;
            } else {
                jlast++;
            }
        }

        guard1 = false;
        if (!isValidModel) {
            guard1 = true;
        } else {
            isValidModel = false;
            jlast = 1;
            exitg1 = false;
            while ((!exitg1) && (jlast <= inliers.size(0))) {
                if (inliers[jlast - 1]) {
                    isValidModel = true;
                    exitg1 = true;
                } else {
                    jlast++;
                }
            }

            if (!isValidModel) {
                guard1 = true;
            }
        }

        if (guard1) {
            *isFound = false;
            inliers.set_size(allPoints.size(0));
            jlast = allPoints.size(0);
            for (lastBlockLength = 0; lastBlockLength < jlast;
                 lastBlockLength++) {
                inliers[lastBlockLength] = false;
            }
        }
    } else {
        inliers.set_size(allPoints.size(0));
        jlast = allPoints.size(0);
        for (lastBlockLength = 0; lastBlockLength < jlast; lastBlockLength++) {
            inliers[lastBlockLength] = false;
        }
    }
}
}  // namespace ransac
}  // namespace internal
}  // namespace vision
}  // namespace coder

static void constructWorldMap_init(HDMapping *aInstancePtr) {
    constructWorldMapStackData *localSD;
    localSD = aInstancePtr->getStackData();
    localSD->pd->preFeatures_not_empty = false;
}

static int div_s32(int numerator, int denominator) {
    int quotient;
    if (denominator == 0) {
        if (numerator >= 0) {
            quotient = MAX_int32_T;
        } else {
            quotient = MIN_int32_T;
        }
    } else {
        unsigned int tempAbsQuotient;
        unsigned int u;
        if (numerator < 0) {
            tempAbsQuotient = ~static_cast<unsigned int>(numerator) + 1U;
        } else {
            tempAbsQuotient = static_cast<unsigned int>(numerator);
        }

        if (denominator < 0) {
            u = ~static_cast<unsigned int>(denominator) + 1U;
        } else {
            u = static_cast<unsigned int>(denominator);
        }

        tempAbsQuotient /= u;
        if ((numerator < 0) != (denominator < 0)) {
            quotient = -static_cast<int>(tempAbsQuotient);
        } else {
            quotient = static_cast<int>(tempAbsQuotient);
        }
    }

    return quotient;
}

static void eml_rand_mt19937ar_stateful_init(HDMapping *aInstancePtr) {
    constructWorldMapStackData *localSD;
    unsigned int r;
    localSD = aInstancePtr->getStackData();
    std::memset(&localSD->pd->state[0], 0, 625U * sizeof(unsigned int));
    r = 5489U;
    localSD->pd->state[0] = 5489U;
    for (int mti{0}; mti < 623; mti++) {
        r = ((r ^ r >> 30U) * 1812433253U + static_cast<unsigned int>(mti)) + 1U;
        localSD->pd->state[mti + 1] = r;
    }

    localSD->pd->state[624] = 624U;
}

static void filedata_init(HDMapping *aInstancePtr) {
    FILE *a;
    constructWorldMapStackData *localSD;
    localSD = aInstancePtr->getStackData();
    a = NULL;
    for (int i{0}; i < 20; i++) {
        localSD->pd->eml_openfiles[i] = a;
    }
}

static void minus(::coder::array<double, 1U> &in1, const ::coder::array<double, 1U> &in2, const ::coder::array<double, 1U> &in3) {
    int i;
    int loop_ub;
    int stride_0_0;
    int stride_1_0;
    if (in3.size(0) == 1) {
        i = in2.size(0);
    } else {
        i = in3.size(0);
    }

    in1.set_size(i);
    stride_0_0 = (in2.size(0) != 1);
    stride_1_0 = (in3.size(0) != 1);
    if (in3.size(0) == 1) {
        loop_ub = in2.size(0);
    } else {
        loop_ub = in3.size(0);
    }

    for (i = 0; i < loop_ub; i++) {
        in1[i] = in2[i * stride_0_0] - in3[i * stride_1_0];
    }
}

static double rt_atan2d_snf(double u0, double u1) {
    double y;
    if (std::isnan(u0) || std::isnan(u1)) {
        y = rtNaN;
    } else if (std::isinf(u0) && std::isinf(u1)) {
        int i;
        int i1;
        if (u0 > 0.0) {
            i = 1;
        } else {
            i = -1;
        }

        if (u1 > 0.0) {
            i1 = 1;
        } else {
            i1 = -1;
        }

        y = std::atan2(static_cast<double>(i), static_cast<double>(i1));
    } else if (u1 == 0.0) {
        if (u0 > 0.0) {
            y = RT_PI / 2.0;
        } else if (u0 < 0.0) {
            y = -(RT_PI / 2.0);
        } else {
            y = 0.0;
        }
    } else {
        y = std::atan2(u0, u1);
    }

    return y;
}

static double rt_hypotd_snf(double u0, double u1) {
    double a;
    double b;
    double y;
    a = std::abs(u0);
    b = std::abs(u1);
    if (a < b) {
        a /= b;
        y = b * std::sqrt(a * a + 1.0);
    } else if (a > b) {
        b /= a;
        y = a * std::sqrt(b * b + 1.0);
    } else if (std::isnan(b)) {
        y = rtNaN;
    } else {
        y = a * 1.4142135623730951;
    }

    return y;
}

static double rt_powd_snf(double u0, double u1) {
    double y;
    if (std::isnan(u0) || std::isnan(u1)) {
        y = rtNaN;
    } else {
        double d;
        double d1;
        d = std::abs(u0);
        d1 = std::abs(u1);
        if (std::isinf(u1)) {
            if (d == 1.0) {
                y = 1.0;
            } else if (d > 1.0) {
                if (u1 > 0.0) {
                    y = rtInf;
                } else {
                    y = 0.0;
                }
            } else if (u1 > 0.0) {
                y = 0.0;
            } else {
                y = rtInf;
            }
        } else if (d1 == 0.0) {
            y = 1.0;
        } else if (d1 == 1.0) {
            if (u1 > 0.0) {
                y = u0;
            } else {
                y = 1.0 / u0;
            }
        } else if (u1 == 2.0) {
            y = u0 * u0;
        } else if ((u1 == 0.5) && (u0 >= 0.0)) {
            y = std::sqrt(u0);
        } else if ((u0 < 0.0) && (u1 > std::floor(u1))) {
            y = rtNaN;
        } else {
            y = std::pow(u0, u1);
        }
    }

    return y;
}

static double rt_remd_snf(double u0, double u1) {
    double y;
    if (std::isnan(u0) || std::isnan(u1) || std::isinf(u0)) {
        y = rtNaN;
    } else if (std::isinf(u1)) {
        y = u0;
    } else if ((u1 != 0.0) && (u1 != std::trunc(u1))) {
        double q;
        q = std::abs(u0 / u1);
        if (!(std::abs(q - std::floor(q + 0.5)) > DBL_EPSILON * q)) {
            y = 0.0 * u0;
        } else {
            y = std::fmod(u0, u1);
        }
    } else {
        y = std::fmod(u0, u1);
    }

    return y;
}

HDMapping::HDMapping() {
    SD_.pd = &pd_;
    omp_init_nest_lock(&constructWorldMap_nestLockGlobal);
    pd_.freq_not_empty = false;
    pd_.lookupTable_not_empty = false;
    pd_.BW_not_empty = false;
    constructWorldMap_init(this);
    blendImage_init(this);
    eml_rand_mt19937ar_stateful_init(this);
    filedata_init(this);
}

HDMapping::~HDMapping() {
    blendImage_free(this);
    omp_destroy_nest_lock(&constructWorldMap_nestLockGlobal);
}

void HDMapping::b_constructWorldMap(const struct0_T *inputArgs, struct1_T
                                                                    *inputOutputStruct) {
    static const char filePathChar[36]{"./data/preSavedData/database.yml.gz"};

    coder::binaryFeatures currFeatures;
    coder::binaryFeatures r;
    coder::poseGraph updatePg;
    coder::rigidtform2d currVehiclePtPose;
    coder::rigidtform2d relTform;
    coder::rigidtform2d tform;
    coder::robotics::core::internal::BlockMatrix lobj_1[6];
    ::coder::array<cell_wrap_0, 1U> imageNames;
    ::coder::array<double, 2U> b_updateNodeVehiclePtPoses;
    ::coder::array<double, 2U> nearestIDs;
    ::coder::array<double, 2U> updateNodeVehiclePtPoses;
    ::coder::array<double, 2U> validInd2;
    ::coder::array<float, 2U> currPoints;
    ::coder::array<float, 2U> r1;
    ::coder::array<float, 1U> a__1;
    ::coder::array<unsigned char, 2U> currViewImg;
    ::coder::array<bool, 2U> maskImg;
    double c_data[10];
    double A_in[9];
    double B[9];
    double beta[9];
    double d_r1[9];
    double Rc[4];
    double b_A_in[4];
    double c_x1[4];
    double b_vehicleCenterInBig[3];
    double vehicleCenterInBig[2];
    double b_r1;
    double b_r2;
    double b_x1;
    double beta_idx_1;
    double c_r1;
    double loopCandidate;
    double r1_tmp;
    double r2;
    double r2_tmp;
    double x1;
    double x2;
    int ia_data[10];
    int c_size;
    int i;
    int ib_size;
    int loop_ub;
    signed char c_tmp_data[10];
    bool close_enough;
    bool isDetectedLoop;
    if ((!pd_.BW_not_empty) || (!pd_.preFeatures_not_empty)) {
        coder::poly2mask(pd_.BW);
        pd_.BW_not_empty = true;
        buildMapFunctions::b_helperDetectAndExtractFeatures(this,
                                                            inputArgs->undistortImage, &pd_.preFeatures, a__1, pd_.prePoints);
        pd_.preFeatures_not_empty = true;
        pd_.preRelTform.RotationAngle = 0.0;
        pd_.previousImgPose.RotationAngle = 0.0;
        pd_.preRelTform.Translation[0] = 0.0;
        pd_.previousImgPose.Translation[0] = 0.0;
        pd_.preRelTform.Translation[1] = 0.0;
        pd_.previousImgPose.Translation[1] = 0.0;
        coder::mean(vehicleCenterInBig);
        pd_.initViclePtPose.Translation[0] = vehicleCenterInBig[0];
        pd_.initViclePtPose.Translation[1] = vehicleCenterInBig[1];
        pd_.initViclePtPose.RotationAngle = 0.0;
        pd_.prePoseNodes.Translation[0] = vehicleCenterInBig[0];
        pd_.prePoseNodes.Translation[1] = vehicleCenterInBig[1];
        pd_.prePoseNodes.RotationAngle = 0.0;
        pd_.preLoopCandiate = 1.0;
        loopDatabase_x86_64_load(&filePathChar[0]);
        pd_.isFirst = true;
        pd_.pg.init(&pd_.gobj_2[0]);
        pd_.cumDist = 0.0;
        pd_.xLimitGlobal[0] = 1.0;
        pd_.yLimitGlobal[0] = 1.0;
        pd_.xLimitGlobal[1] = 1.0;
        pd_.yLimitGlobal[1] = 1.0;
    }

    buildMapFunctions::b_helperDetectAndExtractFeatures(this,
                                                        inputArgs->undistortImage, &currFeatures, a__1, currPoints);
    r = pd_.preFeatures;
    r1.set_size(pd_.prePoints.size(0), 2);
    loop_ub = pd_.prePoints.size(0) * pd_.prePoints.size(1) - 1;
    for (i = 0; i <= loop_ub; i++) {
        r1[i] = pd_.prePoints[i];
    }

    buildMapFunctions::estiTform(this, &r, r1, &currFeatures, currPoints,
                                 &relTform, maskImg, nearestIDs, validInd2, &close_enough, &loop_ub);
    if ((loop_ub > 0) || (coder::internal::applyToMultipleDims(maskImg) <= 3.0)) {
        relTform = pd_.preRelTform;
    }

    pd_.preRelTform = relTform;
    b_r1 = relTform.RotationAngle;
    coder::b_cosd(&b_r1);
    r2 = relTform.RotationAngle;
    coder::b_sind(&r2);
    c_r1 = pd_.previousImgPose.RotationAngle;
    coder::b_cosd(&c_r1);
    b_r2 = pd_.previousImgPose.RotationAngle;
    coder::b_sind(&b_r2);
    d_r1[0] = b_r1;
    d_r1[3] = -r2;
    d_r1[6] = relTform.Translation[0];
    d_r1[1] = r2;
    d_r1[4] = b_r1;
    d_r1[7] = relTform.Translation[1];
    B[0] = c_r1;
    B[3] = -b_r2;
    B[6] = pd_.previousImgPose.Translation[0];
    B[1] = b_r2;
    B[4] = c_r1;
    B[7] = pd_.previousImgPose.Translation[1];
    d_r1[2] = 0.0;
    B[2] = 0.0;
    d_r1[5] = 0.0;
    B[5] = 0.0;
    d_r1[8] = 1.0;
    B[8] = 1.0;
    for (i = 0; i < 3; i++) {
        beta_idx_1 = d_r1[i];
        b_r1 = d_r1[i + 3];
        c_r1 = d_r1[i + 6];
        for (ib_size = 0; ib_size < 3; ib_size++) {
            A_in[i + 3 * ib_size] = (beta_idx_1 * B[3 * ib_size] + b_r1 * B[3 *
                                                                                ib_size +
                                                                            1]) +
                                    c_r1 * B[3 * ib_size + 2];
        }
    }

    b_A_in[0] = A_in[0];
    b_A_in[1] = A_in[1];
    b_A_in[2] = A_in[3];
    b_A_in[3] = A_in[4];
    coder::images::geotrans::internal::constrainToRotationMatrix2D(b_A_in, Rc,
                                                                   &c_r1);
    r2 = coder::b_mod(c_r1 + 180.0) - 180.0;
    c_r1 = std::round(r2);
    if (r2 == c_r1) {
        close_enough = true;
    } else {
        b_r2 = std::abs(r2 - c_r1);
        if ((r2 == 0.0) || (c_r1 == 0.0)) {
            close_enough = (b_r2 < 4.94065645841247E-324);
        } else {
            beta_idx_1 = std::abs(r2) + std::abs(c_r1);
            if (beta_idx_1 < 2.2250738585072014E-308) {
                close_enough = (b_r2 < 4.94065645841247E-324);
            } else {
                close_enough = (b_r2 / std::fmin(beta_idx_1, 1.7976931348623157E+308) <
                                2.2204460492503131E-16);
            }
        }
    }

    if (close_enough) {
        r2 = c_r1;
    }

    relTform.RotationAngle = r2;
    relTform.Translation[0] = A_in[6];
    relTform.Translation[1] = A_in[7];
    b_r1 = r2;
    coder::b_cosd(&b_r1);
    coder::b_sind(&r2);
    d_r1[0] = b_r1;
    d_r1[3] = -r2;
    d_r1[6] = A_in[6];
    d_r1[1] = r2;
    d_r1[4] = b_r1;
    d_r1[7] = A_in[7];
    d_r1[2] = 0.0;
    d_r1[5] = 0.0;
    d_r1[8] = 1.0;
    coder::inv(d_r1, A_in);
    b_A_in[0] = A_in[0];
    b_A_in[1] = A_in[1];
    b_A_in[2] = A_in[3];
    b_A_in[3] = A_in[4];
    coder::images::geotrans::internal::constrainToRotationMatrix2D(b_A_in, Rc,
                                                                   &c_r1);
    r2 = coder::b_mod(c_r1 + 180.0) - 180.0;
    c_r1 = std::round(r2);
    if (r2 == c_r1) {
        close_enough = true;
    } else {
        b_r2 = std::abs(r2 - c_r1);
        if ((r2 == 0.0) || (c_r1 == 0.0)) {
            close_enough = (b_r2 < 4.94065645841247E-324);
        } else {
            beta_idx_1 = std::abs(r2) + std::abs(c_r1);
            if (beta_idx_1 < 2.2250738585072014E-308) {
                close_enough = (b_r2 < 4.94065645841247E-324);
            } else {
                close_enough = (b_r2 / std::fmin(beta_idx_1, 1.7976931348623157E+308) <
                                2.2204460492503131E-16);
            }
        }
    }

    if (close_enough) {
        r2 = c_r1;
    }

    tform.RotationAngle = r2;
    tform.Translation[0] = A_in[6];
    tform.Translation[1] = A_in[7];
    r1_tmp = r2;
    coder::b_cosd(&r1_tmp);
    r2_tmp = r2;
    coder::b_sind(&r2_tmp);
    b_r1 = pd_.initViclePtPose.RotationAngle;
    coder::b_cosd(&b_r1);
    r2 = pd_.initViclePtPose.RotationAngle;
    coder::b_sind(&r2);
    B[0] = r1_tmp;
    B[3] = -r2_tmp;
    B[6] = A_in[6];
    B[1] = r2_tmp;
    B[4] = r1_tmp;
    B[7] = A_in[7];
    d_r1[0] = b_r1;
    d_r1[3] = -r2;
    d_r1[6] = pd_.initViclePtPose.Translation[0];
    d_r1[1] = r2;
    d_r1[4] = b_r1;
    d_r1[7] = pd_.initViclePtPose.Translation[1];
    B[2] = 0.0;
    d_r1[2] = 0.0;
    B[5] = 0.0;
    d_r1[5] = 0.0;
    B[8] = 1.0;
    d_r1[8] = 1.0;
    for (i = 0; i < 3; i++) {
        beta_idx_1 = B[i];
        b_r1 = B[i + 3];
        c_r1 = B[i + 6];
        for (ib_size = 0; ib_size < 3; ib_size++) {
            beta[i + 3 * ib_size] = (beta_idx_1 * d_r1[3 * ib_size] + b_r1 * d_r1[3 *
                                                                                      ib_size +
                                                                                  1]) +
                                    c_r1 * d_r1[3 * ib_size + 2];
        }
    }

    b_A_in[0] = beta[0];
    b_A_in[1] = beta[1];
    b_A_in[2] = beta[3];
    b_A_in[3] = beta[4];
    coder::images::geotrans::internal::constrainToRotationMatrix2D(b_A_in, Rc,
                                                                   &c_r1);
    r2 = coder::b_mod(c_r1 + 180.0) - 180.0;
    c_r1 = std::round(r2);
    if (r2 == c_r1) {
        close_enough = true;
    } else {
        b_r2 = std::abs(r2 - c_r1);
        if ((r2 == 0.0) || (c_r1 == 0.0)) {
            close_enough = (b_r2 < 4.94065645841247E-324);
        } else {
            beta_idx_1 = std::abs(r2) + std::abs(c_r1);
            if (beta_idx_1 < 2.2250738585072014E-308) {
                close_enough = (b_r2 < 4.94065645841247E-324);
            } else {
                close_enough = (b_r2 / std::fmin(beta_idx_1, 1.7976931348623157E+308) <
                                2.2204460492503131E-16);
            }
        }
    }

    if (close_enough) {
        r2 = c_r1;
    }

    currVehiclePtPose.RotationAngle = r2;
    currVehiclePtPose.Translation[0] = beta[6];
    currVehiclePtPose.Translation[1] = beta[7];
    updateNodeVehiclePtPoses.set_size(inputOutputStruct->vehiclePoses.size(0) +
                                          1,
                                      3);
    loop_ub = inputOutputStruct->vehiclePoses.size(0);
    for (i = 0; i < 3; i++) {
        for (ib_size = 0; ib_size < loop_ub; ib_size++) {
            updateNodeVehiclePtPoses[ib_size + updateNodeVehiclePtPoses.size(0) * i] = inputOutputStruct->vehiclePoses[ib_size +
                                                                                                                       inputOutputStruct->vehiclePoses.size(0) * i];
        }
    }

    updateNodeVehiclePtPoses[inputOutputStruct->vehiclePoses.size(0)] = beta[6];
    updateNodeVehiclePtPoses[inputOutputStruct->vehiclePoses.size(0) +
                             updateNodeVehiclePtPoses.size(0)] = beta[7];
    updateNodeVehiclePtPoses[inputOutputStruct->vehiclePoses.size(0) +
                             updateNodeVehiclePtPoses.size(0) * 2] = 0.017453292519943295 * r2;
    inputOutputStruct->vehiclePoses.set_size(updateNodeVehiclePtPoses.size(0), 3);
    loop_ub = updateNodeVehiclePtPoses.size(0) * 3;
    for (i = 0; i < loop_ub; i++) {
        inputOutputStruct->vehiclePoses[i] = updateNodeVehiclePtPoses[i];
    }

    loopDatabase_x86_64_add(&inputArgs->undistortImage[0]);
    isDetectedLoop = false;
    loopCandidate = 1.0;
    if (inputOutputStruct->vehiclePoses.size(0) > 1118) {
        double result[20];
        loopDatabase_x86_64_query(&inputArgs->undistortImage[0], &result[0]);
        if (inputOutputStruct->vehiclePoses.size(0) <
            inputOutputStruct->vehiclePoses.size(0) - 500) {
            nearestIDs.set_size(1, 0);
        } else {
            nearestIDs.set_size(1, 501);
            for (i = 0; i < 501; i++) {
                nearestIDs[i] = (inputOutputStruct->vehiclePoses.size(0) + i) - 500;
            }
        }

        coder::do_vectors(&result[0], nearestIDs, c_data, &c_size, ia_data,
                          &loop_ub, &ib_size);
        if (loop_ub != 0) {
            isDetectedLoop = true;
            if (pd_.isFirst) {
                loopCandidate = c_data[0];
                pd_.isFirst = false;
            } else {
                bool b_tmp_data[10];
                bool tmp_data[10];
                for (i = 0; i < c_size; i++) {
                    tmp_data[i] = (c_data[i] >= pd_.preLoopCandiate - 5.0);
                }

                for (i = 0; i < c_size; i++) {
                    b_tmp_data[i] = (c_data[i] <= pd_.preLoopCandiate + 5.0);
                }

                loop_ub = 0;
                for (i = 0; i < c_size; i++) {
                    if (tmp_data[i] && b_tmp_data[i]) {
                        loop_ub++;
                    }
                }

                if (loop_ub != 0) {
                    loop_ub = c_size - 1;
                    ib_size = 0;
                    for (i = 0; i <= loop_ub; i++) {
                        if (tmp_data[i] && b_tmp_data[i]) {
                            c_tmp_data[ib_size] = static_cast<signed char>(i + 1);
                            ib_size++;
                        }
                    }

                    loopCandidate = c_data[c_tmp_data[0] - 1];
                } else {
                    loopCandidate = c_data[0];
                }
            }

            pd_.preLoopCandiate = loopCandidate;
        }
    }

    x1 = pd_.prePoseNodes.RotationAngle;
    coder::b_cosd(&x1);
    x2 = pd_.prePoseNodes.RotationAngle;
    coder::b_sind(&x2);
    b_x1 = r2;
    coder::b_cosd(&b_x1);
    coder::b_sind(&r2);
    c_r1 = pd_.prePoseNodes.RotationAngle;
    coder::b_cosd(&c_r1);
    b_r2 = pd_.prePoseNodes.RotationAngle;
    coder::b_sind(&b_r2);
    b_r1 = beta[6] - pd_.prePoseNodes.Translation[0];
    beta_idx_1 = beta[7] - pd_.prePoseNodes.Translation[1];
    vehicleCenterInBig[0] = c_r1 * b_r1 + b_r2 * beta_idx_1;
    beta_idx_1 = -b_r2 * b_r1 + c_r1 * beta_idx_1;
    b_A_in[0] = x1;
    b_A_in[1] = -x2;
    b_A_in[2] = x2;
    b_A_in[3] = x1;
    for (i = 0; i < 2; i++) {
        b_r1 = b_A_in[i + 2];
        c_r1 = b_A_in[i];
        c_x1[i] = c_r1 * b_x1 + b_r1 * r2;
        c_x1[i + 2] = c_r1 * -r2 + b_r1 * b_x1;
    }

    coder::images::geotrans::internal::constrainToRotationMatrix2D(c_x1, Rc,
                                                                   &c_r1);
    r2 = coder::b_mod(c_r1 + 180.0) - 180.0;
    c_r1 = std::round(r2);
    if (r2 == c_r1) {
        close_enough = true;
    } else {
        b_r2 = std::abs(r2 - c_r1);
        if ((r2 == 0.0) || (c_r1 == 0.0)) {
            close_enough = (b_r2 < 4.94065645841247E-324);
        } else {
            b_r1 = std::abs(r2) + std::abs(c_r1);
            if (b_r1 < 2.2250738585072014E-308) {
                close_enough = (b_r2 < 4.94065645841247E-324);
            } else {
                close_enough = (b_r2 / std::fmin(b_r1, 1.7976931348623157E+308) <
                                2.2204460492503131E-16);
            }
        }
    }

    if (close_enough) {
        r2 = c_r1;
    }

    b_vehicleCenterInBig[0] = vehicleCenterInBig[0];
    b_vehicleCenterInBig[1] = beta_idx_1;
    b_vehicleCenterInBig[2] = 0.017453292519943295 * r2;
    pd_.pg.addRelativePose(b_vehicleCenterInBig, static_cast<double>(inputOutputStruct->vehiclePoses.size(0)), static_cast<double>(static_cast<unsigned int>(inputOutputStruct->vehiclePoses.size(0)) + 1U));
    b_r2 = vehicleCenterInBig[0] * 0.03;
    c_r1 = beta_idx_1 * 0.03;
    pd_.cumDist += std::sqrt(b_r2 * b_r2 + c_r1 * c_r1);
    if (isDetectedLoop) {
        signed char fileid;
        pd_.pg.addRelativePose(static_cast<double>(inputOutputStruct->vehiclePoses.size(0)) + 1.0, loopCandidate);
        coder::optimizePoseGraph(this, &pd_.pg, &lobj_1[0], &updatePg);
        updatePg.nodeEstimates(updateNodeVehiclePtPoses);
        b_vehicleCenterInBig[0] = pd_.initViclePtPose.Translation[0];
        b_vehicleCenterInBig[1] = pd_.initViclePtPose.Translation[1];
        b_vehicleCenterInBig[2] = 0.0;
        b_updateNodeVehiclePtPoses.set_size(updateNodeVehiclePtPoses.size(0), 3);
        loop_ub = updateNodeVehiclePtPoses.size(0);
        for (i = 0; i < 3; i++) {
            for (ib_size = 0; ib_size < loop_ub; ib_size++) {
                b_updateNodeVehiclePtPoses[ib_size + b_updateNodeVehiclePtPoses.size(0) * i] = updateNodeVehiclePtPoses[ib_size +
                                                                                                                        updateNodeVehiclePtPoses.size(0) * i] +
                                                                                               b_vehicleCenterInBig[i];
            }
        }

        updateNodeVehiclePtPoses.set_size(b_updateNodeVehiclePtPoses.size(0), 3);
        loop_ub = b_updateNodeVehiclePtPoses.size(0);
        for (i = 0; i < 3; i++) {
            for (ib_size = 0; ib_size < loop_ub; ib_size++) {
                updateNodeVehiclePtPoses[ib_size + updateNodeVehiclePtPoses.size(0) *
                                                       i] = b_updateNodeVehiclePtPoses[ib_size +
                                                                                       b_updateNodeVehiclePtPoses.size(0) * i];
            }
        }

        loop_ub = inputOutputStruct->vehiclePoses.size(0);
        for (i = 0; i < 3; i++) {
            for (ib_size = 0; ib_size < loop_ub; ib_size++) {
                updateNodeVehiclePtPoses[ib_size +
                                         inputOutputStruct->vehiclePoses.size(0) * i] =
                    updateNodeVehiclePtPoses[ib_size + updateNodeVehiclePtPoses.size(0) *
                                                           i];
            }
        }

        updateNodeVehiclePtPoses.set_size(inputOutputStruct->vehiclePoses.size(0),
                                          3);
        fileid = coder::cfopen(this);
        i = inputOutputStruct->vehiclePoses.size(0);
        imageNames.set_size(inputOutputStruct->vehiclePoses.size(0));
        for (loop_ub = 0; loop_ub < i; loop_ub++) {
            coder::fgetl(this, static_cast<double>(fileid), imageNames[loop_ub].f1);
        }

        coder::cfclose(this, static_cast<double>(fileid));
        printf("%s\n",
               "The pose map optimization has been completed and is being stitched together to complete the larger image, please wait...");
        fflush(stdout);
        loop_ub = inputOutputStruct->vehiclePoses.size(0);
        for (i = 0; i < 3; i++) {
            for (ib_size = 0; ib_size < loop_ub; ib_size++) {
                updateNodeVehiclePtPoses[ib_size +
                                         inputOutputStruct->vehiclePoses.size(0) * i] =
                    updateNodeVehiclePtPoses[ib_size + updateNodeVehiclePtPoses.size(0) *
                                                           i];
            }
        }

        updateNodeVehiclePtPoses.set_size(inputOutputStruct->vehiclePoses.size(0),
                                          3);
        std::copy(&pd_.BW[0], &pd_.BW[307200], &SD_.f4.bv[0]);
        buildMapFunctions::b_fuseOptimizeHDMap(this, imageNames,
                                               updateNodeVehiclePtPoses, pd_.initViclePtPose, SD_.f4.bv,
                                               inputOutputStruct->HDmap.bigImg,
                                               inputOutputStruct->HDmap.ref.XWorldLimits,
                                               inputOutputStruct->HDmap.ref.YWorldLimits,
                                               inputOutputStruct->HDmap.ref.ImageSize);
        inputOutputStruct->isOver = true;
    } else {
        coder::imref2d Ref;
        bool b_B[2];
        pd_.prePoseNodes = currVehiclePtPose;
        pd_.previousImgPose = relTform;
        B[6] = A_in[6];
        B[7] = A_in[7];
        b_B[0] = true;
        b_B[1] = true;
        if (coder::all(b_B)) {
            B[7] = A_in[7];
            c_r1 = A_in[6];
            b_r2 = B[7];
            for (i = 0; i < 9; i++) {
                beta_idx_1 = dv[i];
                b_r1 = dv1[i];
                A_in[i] = (r1_tmp * beta_idx_1 + -r2_tmp * b_r1) + c_r1;
                B[i] = (r2_tmp * beta_idx_1 + r1_tmp * b_r1) + b_r2;
            }
        } else {
            for (i = 0; i < 9; i++) {
                A_in[i] = (r1_tmp * dv[i] + -r2_tmp * dv1[i]) + B[6];
            }

            c_r1 = B[7];
            for (i = 0; i < 9; i++) {
                B[i] = (r2_tmp * dv[i] + r1_tmp * dv1[i]) + c_r1;
            }
        }

        beta_idx_1 = pd_.xLimitGlobal[0];
        b_r1 = pd_.xLimitGlobal[1];
        pd_.xLimitGlobal[0] = std::fmin(beta_idx_1, std::fmin(coder::internal::
                                                                  minimum(A_in),
                                                              0.5));
        pd_.xLimitGlobal[1] = std::fmax(b_r1, std::fmax(640.5, coder::internal::
                                                                   maximum(A_in)));
        beta_idx_1 = pd_.yLimitGlobal[0];
        b_r1 = pd_.yLimitGlobal[1];
        pd_.yLimitGlobal[0] = std::fmin(beta_idx_1, std::fmin(coder::internal::
                                                                  minimum(B),
                                                              0.5));
        pd_.yLimitGlobal[1] = std::fmax(b_r1, std::fmax(480.5, coder::internal::
                                                                   maximum(B)));
        Ref.ImageSizeAlias[0] = std::round(pd_.yLimitGlobal[1] - pd_.yLimitGlobal
                                                                     [0]);
        Ref.ImageSizeAlias[1] = std::round(pd_.xLimitGlobal[1] - pd_.xLimitGlobal
                                                                     [0]);
        Ref.XWorldLimits[0] = pd_.xLimitGlobal[0];
        Ref.YWorldLimits[0] = pd_.yLimitGlobal[0];
        Ref.XWorldLimits[1] = pd_.xLimitGlobal[1];
        Ref.YWorldLimits[1] = pd_.yLimitGlobal[1];
        Ref.ForcePixelExtentToOne = false;
        coder::images::internal::coder::optimized::b_remapAndResampleGeneric2d(this, inputArgs->undistortImage, tform, Ref, currViewImg);
        coder::images::internal::coder::optimized::c_remapAndResampleGeneric2d(this, pd_.BW, tform, Ref, maskImg);
        buildMapFunctions::blendImage(this, &inputOutputStruct->HDmap, currViewImg,
                                      Ref, maskImg);
        pd_.preFeatures = currFeatures;
        pd_.prePoints.set_size(currPoints.size(0), 2);
        loop_ub = currPoints.size(0) * 2;
        for (i = 0; i < loop_ub; i++) {
            pd_.prePoints[i] = currPoints[i];
        }
    }
}

namespace coder {
binaryFeatures::binaryFeatures() = default;
binaryFeatures::~binaryFeatures() = default;
poseGraph::poseGraph() = default;
poseGraph::~poseGraph() = default;
rigidtform2d::rigidtform2d() = default;
rigidtform2d::~rigidtform2d() = default;
namespace robotics {
namespace core {
namespace internal {
BlockMatrix::BlockMatrix() = default;
BlockMatrix::~BlockMatrix() = default;
}  // namespace internal
}  // namespace core
}  // namespace robotics

namespace visioncodegen {
AlphaBlender::AlphaBlender() = default;
AlphaBlender::~AlphaBlender() = default;
b_AlphaBlender::b_AlphaBlender() {
    matlabCodegenIsDeleted = true;
}

b_AlphaBlender::~b_AlphaBlender() {
    matlabCodegenDestructor();
}
}  // namespace visioncodegen
}  // namespace coder
}  // namespace buildMapping
