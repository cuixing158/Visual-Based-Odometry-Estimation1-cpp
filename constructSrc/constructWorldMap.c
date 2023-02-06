/**
 * @file           : constructWorldMap.c
 * @target         : Texas Instruments->C6000
 * @details        : for path build map algorithms
 * @author         : cuixingxing
 * @email          : xingxing.cui@long-horn.com
 * @date           : 01-Nov-2022 17:14:32
 * @version        : V1.0.0
 * @copyright      : Copyright (C) 2022 Long-Horn Inc.All rights reserved.
 */

/** @include file    : Include Files */
#include "constructWorldMap.h"
#include "constructWorldMap_emxutil.h"
#include "constructWorldMap_types.h"
#include "rt_nonfinite.h"
#include "detectORBCore_api.hpp"
#include "extractORBCore_api.hpp"
#include "omp.h"
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include <float.h>
#include <math.h>
#include <string.h>
#include <stdio.h>

/** Type Definitions */
#ifndef typedef_rigidtform2d
#define typedef_rigidtform2d

typedef struct {
    float RotationAngle;
    float Translation[2];
} rigidtform2d;

#endif /* typedef_rigidtform2d */

#ifndef typedef_affinetform2d
#define typedef_affinetform2d

typedef struct {
    float A23[6];
} affinetform2d;

#endif /* typedef_affinetform2d */

#ifndef typedef_cameraIntrinsics
#define typedef_cameraIntrinsics

typedef struct {
    double K[9];
} cameraIntrinsics;

#endif /* typedef_cameraIntrinsics */

#ifndef typedef_monoCamera
#define typedef_monoCamera

typedef struct {
    cameraIntrinsics Intrinsics;
    double Height;
    double Pitch;
    double Yaw;
    double Roll;
    double SensorLocation[2];
} monoCamera;

#endif /* typedef_monoCamera */

#ifndef typedef_birdsEyeView
#define typedef_birdsEyeView

typedef struct {
    double OutputView[4];
    double ImageSize[2];
    monoCamera Sensor;
    imref2d OutputViewImref;
    double Scale[2];
} birdsEyeView;

#endif /* typedef_birdsEyeView */

#ifndef typedef_visioncodegen_AlphaBlender
#define typedef_visioncodegen_AlphaBlender

typedef struct {
    bool matlabCodegenIsDeleted;
    int isInitialized;
} visioncodegen_AlphaBlender;

#endif /* typedef_visioncodegen_AlphaBlender */

/** Variable Definitions */
static bool isloadParams_not_empty;
static bool isinitialized_not_empty;
static emxArray_real_T *previousImage;
static struct_T bigImgSt;
static emxArray_real_T *vehicleShowPts;
static emxArray_boolean_T *BW;
static bool lookupTable_not_empty;
static unsigned int state[625];
static visioncodegen_AlphaBlender alphablend;
static bool alphablend_not_empty;
omp_nest_lock_t constructWorldMap_nestLockGlobal;
static const signed char iv[9] = {0, -1, 0, -1, 0, 0, 0, 0, 1};

/** Function Declarations */
static void ORBPointsImpl_configure(ORBPoints *this, const emxArray_real32_T *inputs_Location, const emxArray_real32_T *inputs_Metric, const emxArray_real32_T *inputs_Scale, const emxArray_real32_T *inputs_Orientation);
static void Outputs(const emxArray_real_T *U0, const emxArray_real_T *U1, const emxArray_boolean_T *U2, const double U3[2], emxArray_real_T *Y0);
static bool all(const bool x[2]);
static void b_ORBPointsImpl_configure(ORBPoints *this, const emxArray_real32_T *inputs_Location, const emxArray_real32_T *inputs_Metric, const emxArray_real32_T *inputs_Scale, const emxArray_real32_T *inputs_Orientation);
static void b_and(emxArray_boolean_T *in1, const emxArray_boolean_T *in2, const emxArray_boolean_T *in3);
static void b_binary_expand_op(emxArray_real32_T *in1, const emxArray_real32_T *in2, const emxArray_real32_T *in3);
static int b_bsearch(const emxArray_real_T *x, double xi);
static void b_conv2AXPYValidCMP(const emxArray_real32_T *a, emxArray_real32_T *c);
static void b_cosd(double *x);
static void b_imwarp(emxArray_boolean_T *varargin_1, float varargin_2_RotationAngle, const float varargin_2_Translation[2], const double varargin_5_XWorldLimits[2], const double varargin_5_YWorldLimits[2], const double varargin_5_ImageSizeAlias[2]);
static void b_interp2_local(const double V[6161148], const emxArray_real_T *Xq,
                            const emxArray_real_T *Yq, emxArray_real_T *Vq);
static void b_inv(const float x[9], float y[9]);
static double b_maximum(const double x[8]);
static void b_mean(const emxArray_real32_T *x, float y[2]);
static double b_minimum(const double x[8]);
static float b_mod(float x);
static void b_padarray(const emxArray_real_T *varargin_1, const double varargin_2[2], emxArray_real_T *b);
static double b_rand(void);
static void b_remapAndResampleGeneric2d(const emxArray_real_T *inputImage, const float tform_A23[6], const imref2d outputRef, emxArray_real_T *outputImage);
static void b_sind(double *x);
static void b_svd(const float A[4], float U[4], float s[2], float V[4]);
static void b_vehicleToLocalImage(const double refBirdsEye_OutputView[4], const double refBirdsEye_Sensor_Intrinsics_K[9], double refBirdsEye_Sensor_Height,
                                  double refBirdsEye_Sensor_Pitch, double refBirdsEye_Sensor_Yaw, double refBirdsEye_Sensor_Roll, const double c_refBirdsEye_Sensor_SensorLoca[2],
                                  const double refBirdsEye_Scale[2], const double refImg_XWorldLimits[2], const double refImg_YWorldLimits[2], const double refImg_ImageSizeAlias[2], bool refImg_ForcePixelExtentToOne, double localImagePts[8]);
static double b_xnrm2(const double x_data[]);
static void b_xrotg(double *a, double *b, double *c, double *s);
static void binary_expand_op(emxArray_real_T *in1, const emxArray_real_T *in2,
                             const emxArray_real_T *in3, const emxArray_real_T *in4);
static void birdsEyeView_transformImage(const double this_OutputView[4], const double this_Sensor_Intrinsics_K[9], double this_Sensor_Height, double this_Sensor_Pitch, double this_Sensor_Yaw, double this_Sensor_Roll, const double this_Sensor_SensorLocation[2], const double c_this_OutputViewImref_ImageSiz[2], const double this_Scale[2], const double b_I[18483444], emxArray_real_T *birdsEyeViewImage);
static void blendImage(struct_T *b_bigImgSt, const emxArray_real_T *currImg,
                       const imref2d currRef, const emxArray_boolean_T *maskImg);
static void blendImage_free(void);
static void blendImage_init(void);
static void bwdist(const emxArray_boolean_T *varargin_1, emxArray_real32_T
                                                             *varargout_1);
static void c_binary_expand_op(emxArray_boolean_T *in1, const emxArray_int32_T *in2, const emxArray_uint32_T *in3);
static void c_birdsEyeView_get_ImageToVehic(const double this_OutputView[4],
                                            const double this_Sensor_Intrinsics_K[9], double this_Sensor_Height, double this_Sensor_Pitch, double this_Sensor_Yaw, double this_Sensor_Roll, const double this_Sensor_SensorLocation[2], const double this_Scale[2], double tform_T[9]);
static int c_bsearch(const emxArray_real32_T *x, double xi);
static void c_computeEdgesWithThinningPorta(const emxArray_real32_T *b,
                                            emxArray_real32_T *bx, emxArray_real32_T *by, double cutoff,
                                            emxArray_boolean_T *e);
static void c_cosd(float *x);
static void c_eml_rand_mt19937ar_stateful_i(void);
static void c_interp2_local(const emxArray_real_T *V, const emxArray_real_T *Xq,
                            const emxArray_real_T *Yq, const emxArray_real_T *X, const emxArray_real_T *Y,
                            emxArray_real_T *Vq);
static void c_minimum(const emxArray_real32_T *x, emxArray_real32_T *ex,
                      emxArray_int32_T *idx);
static void c_monoCamera_get_ImageToVehicle(const double this_Intrinsics_K[9],
                                            double this_Height, double this_Pitch, double this_Yaw, double this_Roll,
                                            const double this_SensorLocation[2], double tform_T[9]);
static void c_remapAndResampleGeneric2d(const emxArray_real_T *inputImage, float tform_RotationAngle, const float tform_Translation[2], const imref2d outputRef,
                                        emxArray_real_T *outputImage);
static void c_sind(float *x);
static double c_xnrm2(const double x[3]);
static void computeRigid2d(const emxArray_real32_T *points, float T[9]);
static void constrainToRotationMatrix2D(const float R[4], float Rc[4], float *r);
static void constructWorldMap_free(void);
static void constructWorldMap_init(void);
static void contrib(double x1, double b_y1, double x2, double y2, signed char quad1, signed char quad2, double scale, signed char *diffQuad, bool *onj);
static void conv2AXPYValidCMP(const emxArray_real32_T *a, emxArray_real32_T *c);
static void d_binary_expand_op(emxArray_real32_T *in1, const emxArray_real32_T *in2, const emxArray_real32_T *in3);
static void d_remapAndResampleGeneric2d(const emxArray_boolean_T *inputImage,
                                        float tform_RotationAngle, const float tform_Translation[2], const imref2d outputRef, emxArray_boolean_T *outputImage);
static void detectORBFeatures(const emxArray_real_T *b_I, ORBPoints *points);
static void edge(const emxArray_boolean_T *varargin_1, emxArray_boolean_T
                                                           *varargout_1);
static void estiTform(const emxArray_real_T *previousImg, const emxArray_real_T *currImg, const double vehicleROI[8], float *tform_RotationAngle, float tform_Translation[2],
                      ORBPoints *preMatchedPoints, ORBPoints *currMatchedPoints,
                      emxArray_boolean_T *inlierIdx);
static void evaluateTform2d(const float tform[9], const emxArray_real32_T *points, emxArray_real32_T *dis);
static void extractFeatures(const emxArray_real_T *b_I, const emxArray_real32_T *points_pLocation, const emxArray_real32_T *points_pMetric, const emxArray_real32_T *points_pScale, const emxArray_real32_T *points_pOrientation,
                            emxArray_uint8_T *features_Features, ORBPoints *valid_points);
static void findNearestNeighbors(const emxArray_real32_T *scores,
                                 emxArray_uint32_T *indexPairs, emxArray_real32_T *topTwoMetrics);
static void getSubInds(const emxArray_real32_T *srcORBPoints_pLocation, const emxArray_real32_T *srcORBPoints_pMetric, const emxArray_real32_T *srcORBPoints_pScale, const emxArray_real32_T *srcORBPoints_pOrientation,
                       const emxArray_boolean_T *validIdxs, ORBPoints *dstORBPoints);
static void helperStitchImages(const cell_wrap_1 images[4], const affinetform2d tforms[4], emxArray_real_T *outputImage, double outputView_XWorldLimits[2],
                               double outputView_YWorldLimits[2], double outputView_ImageSizeAlias[2], bool *c_outputView_ForcePixelExtentTo);
static void helperToObjectBev(const double bevStruct_OutputView[4], const double bevStruct_ImageSize[2], const double c_bevStruct_Sensor_Intrinsics_F[2], const double c_bevStruct_Sensor_Intrinsics_P[2], double c_bevStruct_Sensor_Intrinsics_S, double bevStruct_Sensor_Height, double bevStruct_Sensor_Pitch, double bevStruct_Sensor_Yaw, double bevStruct_Sensor_Roll, const double bevStruct_Sensor_SensorLocation[2], double bevObj_OutputView[4], double bevObj_ImageSize[2], monoCamera *bevObj_Sensor,
                              imref2d *bevObj_OutputViewImref, double bevObj_Scale[2]);
static void imwarp(emxArray_real_T *varargin_1, float varargin_2_RotationAngle,
                   const float varargin_2_Translation[2], imref2d *varargin_5);
static void inpolygon(const emxArray_real32_T *x, const emxArray_real32_T *y,
                      const double xv[4], const double yv[4], emxArray_boolean_T *in);
static void interp2(const emxArray_real_T *varargin_1, const emxArray_real_T *varargin_2, const emxArray_real_T *varargin_3, const emxArray_real_T *varargin_4, const emxArray_real_T *varargin_5, emxArray_real_T *Vq);
static void interp2_local(const emxArray_real32_T *V, const emxArray_real_T *Xq,
                          const emxArray_real_T *Yq, const emxArray_real32_T *X, const emxArray_real32_T *Y, emxArray_real32_T *Vq);
static void inv(const double x[9], double y[9]);
static void matchFeatures(const emxArray_uint8_T *varargin_1_Features, const emxArray_uint8_T *varargin_2_Features, emxArray_uint32_T *indexPairs);
static float maximum(const float x[9]);
static void mean(const double x[8], double y[2]);
static void merge(emxArray_int32_T *idx, emxArray_real32_T *x, int offset, int np, int nq, emxArray_int32_T *iwork, emxArray_real32_T *xwork);
static void merge_block(emxArray_int32_T *idx, emxArray_real32_T *x, int offset,
                        int n, int preSortLevel, emxArray_int32_T *iwork, emxArray_real32_T *xwork);
static void meshgrid(const double x[3], const double y[3], double xx[9], double yy[9]);
static float minimum(const float x[9]);
static void msac(const emxArray_real32_T *allPoints, bool *isFound, float bestModelParams_data[], int bestModelParams_size[2],
                 emxArray_boolean_T *inliers);
static void padarray(const emxArray_real_T *varargin_1, const double varargin_2[2], emxArray_real_T *b);
static void poly2edgelist(double x[9], double y[9], int M, int N,
                          emxArray_boolean_T *out, emxArray_int32_T *minY, emxArray_int32_T *maxY);
static void poly2mask(const double x[8], const double y[8], double M, double N,
                      emxArray_boolean_T *b_BW);
static void remapAndResampleGeneric2d(const double inputImage[18483444], const double tform_A_[9], const double outputRef_ImageSizeAlias[2], emxArray_real_T *outputImage);
static void rgb2gray(const emxArray_real_T *X, emxArray_real_T *b_I);
static float rt_atan2f_snf(float u0, float u1);
static double rt_hypotd_snf(double u0, double u1);
static float rt_hypotf_snf(float u0, float u1);
static float rt_powf_snf(float u0, float u1);
static double rt_remd_snf(double u0, double u1);
static float rt_remf_snf(float u0, float u1);
static double rt_roundd_snf(double u);
static void sort(emxArray_real32_T *x, emxArray_int32_T *idx);
static void sum(const emxArray_real_T *x, emxArray_real_T *y);
static void svd(const double A[9], double U[3]);
static void uint8PortableCodeAlgo(const emxArray_real_T *b_I, emxArray_uint8_T
                                                                  *u);
static void vehicleToLocalImage(const double refBirdsEye_OutputView[4], const double refBirdsEye_Sensor_Intrinsics_K[9], double refBirdsEye_Sensor_Height,
                                double refBirdsEye_Sensor_Pitch, double refBirdsEye_Sensor_Yaw, double refBirdsEye_Sensor_Roll, const double c_refBirdsEye_Sensor_SensorLoca[2],
                                const double refBirdsEye_Scale[2], const double refImg_XWorldLimits[2], const double refImg_YWorldLimits[2], const double refImg_ImageSizeAlias[2], bool refImg_ForcePixelExtentToOne, double localImagePts[8]);
static void voronoiEDT(const emxArray_real32_T *g, const emxArray_real32_T *h,
                       emxArray_real32_T *D);
static void worldToGlobalImagePose(const double anchorPose[3], const double worldPose[3], const double localOrigin[2], const double resolutionXY[2],
                                   double imagePose[3]);
static float xnrm2(const float x[4]);
static void xrotg(float *a, float *b, float *c, float *s);
static void xswap(float x[4]);

/** Function Definitions */
/**
 * @fn             : ORBPointsImpl_configure
 * @brief          :
 * @param          : ORBPoints *this
 *                   const emxArray_real32_T *inputs_Location
 *                   const emxArray_real32_T *inputs_Metric
 *                   const emxArray_real32_T *inputs_Scale
 *                   const emxArray_real32_T *inputs_Orientation
 * @return         : void
 */
static void ORBPointsImpl_configure(ORBPoints *this, const emxArray_real32_T *inputs_Location, const emxArray_real32_T *inputs_Metric, const emxArray_real32_T *inputs_Scale, const emxArray_real32_T *inputs_Orientation) {
    const float *inputs_Location_data;
    const float *inputs_Metric_data;
    const float *inputs_Orientation_data;
    const float *inputs_Scale_data;
    int itilerow;
    int ntilerows;
    inputs_Orientation_data = inputs_Orientation->data;
    inputs_Scale_data = inputs_Scale->data;
    inputs_Metric_data = inputs_Metric->data;
    inputs_Location_data = inputs_Location->data;
    if (inputs_Metric->size[0] == 1) {
        itilerow = this->pMetric->size[0];
        this->pMetric->size[0] = inputs_Location->size[0];
        emxEnsureCapacity_real32_T(this->pMetric, itilerow);
        ntilerows = inputs_Location->size[0];
        for (itilerow = 0; itilerow < ntilerows; itilerow++) {
            this->pMetric->data[itilerow] = inputs_Metric_data[0];
        }
    } else {
        itilerow = this->pMetric->size[0];
        this->pMetric->size[0] = inputs_Metric->size[0];
        emxEnsureCapacity_real32_T(this->pMetric, itilerow);
        ntilerows = inputs_Metric->size[0];
        for (itilerow = 0; itilerow < ntilerows; itilerow++) {
            this->pMetric->data[itilerow] = inputs_Metric_data[itilerow];
        }
    }

    itilerow = this->pLocation->size[0] * this->pLocation->size[1];
    this->pLocation->size[0] = inputs_Location->size[0];
    this->pLocation->size[1] = 2;
    emxEnsureCapacity_real32_T(this->pLocation, itilerow);
    ntilerows = inputs_Location->size[0] * 2;
    for (itilerow = 0; itilerow < ntilerows; itilerow++) {
        this->pLocation->data[itilerow] = inputs_Location_data[itilerow];
    }

    if (inputs_Scale->size[0] == 1) {
        itilerow = this->pScale->size[0];
        this->pScale->size[0] = inputs_Location->size[0];
        emxEnsureCapacity_real32_T(this->pScale, itilerow);
        ntilerows = inputs_Location->size[0];
        for (itilerow = 0; itilerow < ntilerows; itilerow++) {
            this->pScale->data[itilerow] = inputs_Scale_data[0];
        }
    } else {
        itilerow = this->pScale->size[0];
        this->pScale->size[0] = inputs_Scale->size[0];
        emxEnsureCapacity_real32_T(this->pScale, itilerow);
        ntilerows = inputs_Scale->size[0];
        for (itilerow = 0; itilerow < ntilerows; itilerow++) {
            this->pScale->data[itilerow] = inputs_Scale_data[itilerow];
        }
    }

    if (inputs_Orientation->size[0] == 1) {
        itilerow = this->pOrientation->size[0];
        this->pOrientation->size[0] = inputs_Location->size[0];
        emxEnsureCapacity_real32_T(this->pOrientation, itilerow);
        ntilerows = inputs_Location->size[0];
        for (itilerow = 0; itilerow < ntilerows; itilerow++) {
            this->pOrientation->data[itilerow] = inputs_Orientation_data[0];
        }
    } else {
        itilerow = this->pOrientation->size[0];
        this->pOrientation->size[0] = inputs_Orientation->size[0];
        emxEnsureCapacity_real32_T(this->pOrientation, itilerow);
        ntilerows = inputs_Orientation->size[0];
        for (itilerow = 0; itilerow < ntilerows; itilerow++) {
            this->pOrientation->data[itilerow] = inputs_Orientation_data[itilerow];
        }
    }
}

/**
 * @fn             : Outputs
 * @brief          :
 * @param          : const emxArray_real_T *U0
 *                   const emxArray_real_T *U1
 *                   const emxArray_boolean_T *U2
 *                   const double U3[2]
 *                   emxArray_real_T *Y0
 * @return         : void
 */
static void Outputs(const emxArray_real_T *U0, const emxArray_real_T *U1, const emxArray_boolean_T *U2, const double U3[2], emxArray_real_T *Y0) {
    const double *U0_data;
    const double *U1_data;
    double *Y0_data;
    int cStart;
    int chanWidthIn1;
    int chanWidthIn2;
    int dstIdx;
    int i;
    int j;
    int m;
    int nRowsIn1;
    int nRowsIn2;
    const bool *U2_data;
    U2_data = U2->data;
    U1_data = U1->data;
    U0_data = U0->data;

    /* System object Outputs function: vision.AlphaBlender */
    nRowsIn1 = U0->size[0];
    nRowsIn2 = U1->size[0];
    chanWidthIn1 = U0->size[0] * U0->size[1];
    chanWidthIn2 = U1->size[0] * U1->size[1];

    /* check if any input is empty
   */
    if ((U0->size[0] == 0) || (U0->size[1] == 0) || (U1->size[0] == 0) ||
        (U1->size[1] == 0) || (U2->size[0] == 0) || (U2->size[1] == 0)) {
        int loop_ub;
        dstIdx = Y0->size[0] * Y0->size[1] * Y0->size[2];
        Y0->size[0] = U0->size[0];
        Y0->size[1] = U0->size[1];
        Y0->size[2] = U0->size[2];
        emxEnsureCapacity_real_T(Y0, dstIdx);
        Y0_data = Y0->data;
        loop_ub = U0->size[0] * U0->size[1] * U0->size[2];
        for (dstIdx = 0; dstIdx < loop_ub; dstIdx++) {
            Y0_data[dstIdx] = U0_data[dstIdx];
        }
    } else if (U2->size[0] * U2->size[1] == 1) {
        int in2Idx;
        in2Idx = 0;
        if (!U2_data[0U]) {
            int loop_ub;
            dstIdx = Y0->size[0] * Y0->size[1] * Y0->size[2];
            Y0->size[0] = U0->size[0];
            Y0->size[1] = U0->size[1];
            Y0->size[2] = U0->size[2];
            emxEnsureCapacity_real_T(Y0, dstIdx);
            Y0_data = Y0->data;
            loop_ub = U0->size[0] * U0->size[1] * U0->size[2];
            for (dstIdx = 0; dstIdx < loop_ub; dstIdx++) {
                Y0_data[dstIdx] = U0_data[dstIdx];
            }
        } else {
            int loop_ub;
            int nColsToCopy;
            int nRowsToCopy;
            int rStart;
            nRowsToCopy = U1->size[0];
            nColsToCopy = U1->size[1];
            rStart = (int)floor(U3[1]) - 1;
            cStart = (int)floor(U3[0]) - 1;
            if (cStart < 0) {
                in2Idx = -cStart * U1->size[0];
                nColsToCopy = U1->size[1] + cStart;
                cStart = 0;
            }

            if (rStart < 0) {
                in2Idx -= rStart;
                nRowsToCopy = U1->size[0] + rStart;
                rStart = 0;
            }

            if (rStart + nRowsToCopy > U0->size[0]) {
                nRowsToCopy = U0->size[0] - rStart;
            }

            if (cStart + nColsToCopy > U0->size[1]) {
                nColsToCopy = U0->size[1] - cStart;
            }

            dstIdx = Y0->size[0] * Y0->size[1] * Y0->size[2];
            Y0->size[0] = U0->size[0];
            Y0->size[1] = U0->size[1];
            Y0->size[2] = U0->size[2];
            emxEnsureCapacity_real_T(Y0, dstIdx);
            Y0_data = Y0->data;
            loop_ub = U0->size[0] * U0->size[1] * U0->size[2];
            for (dstIdx = 0; dstIdx < loop_ub; dstIdx++) {
                Y0_data[dstIdx] = U0_data[dstIdx];
            }

            if (nRowsToCopy > 0) {
                int strtCpyIdx;
                strtCpyIdx = U0->size[0] * cStart + rStart;
                for (m = 0; m < U0->size[2]; m++) {
                    int in2PtrIdx;
                    rStart = strtCpyIdx;
                    in2PtrIdx = in2Idx;
                    for (i = 0; i < nColsToCopy; i++) {
                        for (cStart = 0; cStart < nRowsToCopy; cStart++) {
                            Y0_data[rStart + cStart] = U1_data[in2PtrIdx + cStart];
                        }

                        in2PtrIdx += nRowsIn2;
                        rStart += nRowsIn1;
                    }

                    in2Idx += chanWidthIn2;
                    strtCpyIdx += chanWidthIn1;
                }
            }
        }
    } else {
        int findx;
        int in2Idx;
        int loop_ub;
        int nColsToCopy;
        int nRowsToCopy;
        int rStart;
        int strtCpyIdx;
        in2Idx = 0;
        findx = 0;
        nRowsToCopy = U1->size[0];
        nColsToCopy = U1->size[1];
        rStart = (int)floor(U3[1]) - 1;
        cStart = (int)floor(U3[0]) - 1;
        if (cStart < 0) {
            in2Idx = -cStart * U1->size[0];
            findx = -cStart * U1->size[0];
            nColsToCopy = U1->size[1] + cStart;
            cStart = 0;
        }

        if (rStart < 0) {
            in2Idx -= rStart;
            findx -= rStart;
            nRowsToCopy = U1->size[0] + rStart;
            rStart = 0;
        }

        strtCpyIdx = U0->size[0] * cStart + rStart;
        if (rStart + nRowsToCopy > U0->size[0]) {
            nRowsToCopy = U0->size[0] - rStart;
        }

        if (cStart + nColsToCopy > U0->size[1]) {
            nColsToCopy = U0->size[1] - cStart;
        }

        dstIdx = Y0->size[0] * Y0->size[1] * Y0->size[2];
        Y0->size[0] = U0->size[0];
        Y0->size[1] = U0->size[1];
        Y0->size[2] = U0->size[2];
        emxEnsureCapacity_real_T(Y0, dstIdx);
        Y0_data = Y0->data;
        loop_ub = U0->size[0] * U0->size[1] * U0->size[2];
        for (dstIdx = 0; dstIdx < loop_ub; dstIdx++) {
            Y0_data[dstIdx] = U0_data[dstIdx];
        }

        for (m = 0; m < U0->size[2]; m++) {
            int in2PtrIdx;
            rStart = strtCpyIdx;
            in2PtrIdx = in2Idx;
            cStart = findx;
            for (i = 0; i < nColsToCopy; i++) {
                int indx;
                loop_ub = in2PtrIdx;
                dstIdx = rStart;
                indx = cStart;
                for (j = 0; j < nRowsToCopy; j++) {
                    if (U2_data[indx]) {
                        Y0_data[dstIdx] = U1_data[loop_ub];
                    }

                    dstIdx++;
                    loop_ub++;
                    indx++;
                }

                in2PtrIdx += nRowsIn2;
                rStart += nRowsIn1;
                cStart += nRowsIn2;
            }

            strtCpyIdx += chanWidthIn1;
            in2Idx += chanWidthIn2;
        }
    }
}

/**
 * @fn             : all
 * @brief          :
 * @param          : const bool x[2]
 * @return         : bool
 */
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

/**
 * @fn             : b_ORBPointsImpl_configure
 * @brief          :
 * @param          : ORBPoints *this
 *                   const emxArray_real32_T *inputs_Location
 *                   const emxArray_real32_T *inputs_Metric
 *                   const emxArray_real32_T *inputs_Scale
 *                   const emxArray_real32_T *inputs_Orientation
 * @return         : void
 */
static void b_ORBPointsImpl_configure(ORBPoints *this, const emxArray_real32_T *inputs_Location, const emxArray_real32_T *inputs_Metric, const emxArray_real32_T *inputs_Scale, const emxArray_real32_T *inputs_Orientation) {
    const float *inputs_Location_data;
    const float *inputs_Metric_data;
    const float *inputs_Orientation_data;
    const float *inputs_Scale_data;
    int itilerow;
    int ntilerows;
    inputs_Orientation_data = inputs_Orientation->data;
    inputs_Scale_data = inputs_Scale->data;
    inputs_Metric_data = inputs_Metric->data;
    inputs_Location_data = inputs_Location->data;
    if (inputs_Metric->size[0] == 1) {
        itilerow = this->pMetric->size[0];
        this->pMetric->size[0] = inputs_Location->size[0];
        emxEnsureCapacity_real32_T(this->pMetric, itilerow);
        ntilerows = inputs_Location->size[0];
        for (itilerow = 0; itilerow < ntilerows; itilerow++) {
            this->pMetric->data[itilerow] = inputs_Metric_data[0];
        }
    } else {
        itilerow = this->pMetric->size[0];
        this->pMetric->size[0] = inputs_Metric->size[0];
        emxEnsureCapacity_real32_T(this->pMetric, itilerow);
        ntilerows = inputs_Metric->size[0];
        for (itilerow = 0; itilerow < ntilerows; itilerow++) {
            this->pMetric->data[itilerow] = inputs_Metric_data[itilerow];
        }
    }

    itilerow = this->pLocation->size[0] * this->pLocation->size[1];
    this->pLocation->size[0] = inputs_Location->size[0];
    this->pLocation->size[1] = 2;
    emxEnsureCapacity_real32_T(this->pLocation, itilerow);
    ntilerows = inputs_Location->size[0] * 2;
    for (itilerow = 0; itilerow < ntilerows; itilerow++) {
        this->pLocation->data[itilerow] = inputs_Location_data[itilerow];
    }

    if (inputs_Scale->size[0] == 1) {
        itilerow = this->pScale->size[0];
        this->pScale->size[0] = inputs_Location->size[0];
        emxEnsureCapacity_real32_T(this->pScale, itilerow);
        ntilerows = inputs_Location->size[0];
        for (itilerow = 0; itilerow < ntilerows; itilerow++) {
            this->pScale->data[itilerow] = inputs_Scale_data[0];
        }
    } else {
        itilerow = this->pScale->size[0];
        this->pScale->size[0] = inputs_Scale->size[0];
        emxEnsureCapacity_real32_T(this->pScale, itilerow);
        ntilerows = inputs_Scale->size[0];
        for (itilerow = 0; itilerow < ntilerows; itilerow++) {
            this->pScale->data[itilerow] = inputs_Scale_data[itilerow];
        }
    }

    if (inputs_Orientation->size[0] == 1) {
        itilerow = this->pOrientation->size[0];
        this->pOrientation->size[0] = inputs_Location->size[0];
        emxEnsureCapacity_real32_T(this->pOrientation, itilerow);
        ntilerows = inputs_Location->size[0];
        for (itilerow = 0; itilerow < ntilerows; itilerow++) {
            this->pOrientation->data[itilerow] = inputs_Orientation_data[0];
        }
    } else {
        itilerow = this->pOrientation->size[0];
        this->pOrientation->size[0] = inputs_Orientation->size[0];
        emxEnsureCapacity_real32_T(this->pOrientation, itilerow);
        ntilerows = inputs_Orientation->size[0];
        for (itilerow = 0; itilerow < ntilerows; itilerow++) {
            this->pOrientation->data[itilerow] = inputs_Orientation_data[itilerow];
        }
    }
}

/**
 * @fn             : b_and
 * @brief          :
 * @param          : emxArray_boolean_T *in1
 *                   const emxArray_boolean_T *in2
 *                   const emxArray_boolean_T *in3
 * @return         : void
 */
static void b_and(emxArray_boolean_T *in1, const emxArray_boolean_T *in2, const emxArray_boolean_T *in3) {
    int aux_0_1;
    int aux_1_1;
    int i;
    int i1;
    int loop_ub;
    int stride_0_0;
    int stride_0_1;
    int stride_1_0;
    int stride_1_1;
    const bool *in2_data;
    const bool *in3_data;
    bool *in1_data;
    in3_data = in3->data;
    in2_data = in2->data;
    i = in1->size[0] * in1->size[1];
    if (in3->size[0] == 1) {
        in1->size[0] = in2->size[0];
    } else {
        in1->size[0] = in3->size[0];
    }

    if (in3->size[1] == 1) {
        in1->size[1] = in2->size[1];
    } else {
        in1->size[1] = in3->size[1];
    }

    emxEnsureCapacity_boolean_T(in1, i);
    in1_data = in1->data;
    stride_0_0 = (in2->size[0] != 1);
    stride_0_1 = (in2->size[1] != 1);
    stride_1_0 = (in3->size[0] != 1);
    stride_1_1 = (in3->size[1] != 1);
    aux_0_1 = 0;
    aux_1_1 = 0;
    if (in3->size[1] == 1) {
        loop_ub = in2->size[1];
    } else {
        loop_ub = in3->size[1];
    }

    for (i = 0; i < loop_ub; i++) {
        int b_loop_ub;
        i1 = in3->size[0];
        if (i1 == 1) {
            b_loop_ub = in2->size[0];
        } else {
            b_loop_ub = i1;
        }

        for (i1 = 0; i1 < b_loop_ub; i1++) {
            in1_data[i1 + in1->size[0] * i] = (in2_data[i1 * stride_0_0 + in2->size[0] * aux_0_1] && in3_data[i1 * stride_1_0 + in3->size[0] * aux_1_1]);
        }

        aux_1_1 += stride_1_1;
        aux_0_1 += stride_0_1;
    }
}

/**
 * @fn             : b_binary_expand_op
 * @brief          :
 * @param          : emxArray_real32_T *in1
 *                   const emxArray_real32_T *in2
 *                   const emxArray_real32_T *in3
 * @return         : void
 */
static void b_binary_expand_op(emxArray_real32_T *in1, const emxArray_real32_T *in2, const emxArray_real32_T *in3) {
    const float *in2_data;
    const float *in3_data;
    float *in1_data;
    int aux_0_1;
    int aux_1_1;
    int i;
    int i1;
    int loop_ub;
    int stride_0_0;
    int stride_0_1;
    int stride_1_0;
    int stride_1_1;
    in3_data = in3->data;
    in2_data = in2->data;
    i = in1->size[0] * in1->size[1];
    if (in3->size[0] == 1) {
        in1->size[0] = in2->size[0];
    } else {
        in1->size[0] = in3->size[0];
    }

    if (in3->size[1] == 1) {
        in1->size[1] = in2->size[1];
    } else {
        in1->size[1] = in3->size[1];
    }

    emxEnsureCapacity_real32_T(in1, i);
    in1_data = in1->data;
    stride_0_0 = (in2->size[0] != 1);
    stride_0_1 = (in2->size[1] != 1);
    stride_1_0 = (in3->size[0] != 1);
    stride_1_1 = (in3->size[1] != 1);
    aux_0_1 = 0;
    aux_1_1 = 0;
    if (in3->size[1] == 1) {
        loop_ub = in2->size[1];
    } else {
        loop_ub = in3->size[1];
    }

    for (i = 0; i < loop_ub; i++) {
        int b_loop_ub;
        i1 = in3->size[0];
        if (i1 == 1) {
            b_loop_ub = in2->size[0];
        } else {
            b_loop_ub = i1;
        }

        for (i1 = 0; i1 < b_loop_ub; i1++) {
            float f;
            f = in3_data[i1 * stride_1_0 + in3->size[0] * aux_1_1];
            in1_data[i1 + in1->size[0] * i] = in2_data[i1 * stride_0_0 + in2->size[0] *
                                                                             aux_0_1] *
                                                  in2_data[i1 * stride_0_0 + in2->size[0] * aux_0_1] +
                                              f * f;
        }

        aux_1_1 += stride_1_1;
        aux_0_1 += stride_0_1;
    }
}

/**
 * @fn             : b_bsearch
 * @brief          :
 * @param          : const emxArray_real_T *x
 *                   double xi
 * @return         : int
 */
static int b_bsearch(const emxArray_real_T *x, double xi) {
    const double *x_data;
    int high_i;
    int low_ip1;
    int n;
    x_data = x->data;
    high_i = x->size[1];
    n = 1;
    low_ip1 = 2;
    while (high_i > low_ip1) {
        int mid_i;
        mid_i = (n >> 1) + (high_i >> 1);
        if (((n & 1) == 1) && ((high_i & 1) == 1)) {
            mid_i++;
        }

        if (xi >= x_data[mid_i - 1]) {
            n = mid_i;
            low_ip1 = mid_i + 1;
        } else {
            high_i = mid_i;
        }
    }

    return n;
}

/**
 * @fn             : b_conv2AXPYValidCMP
 * @brief          :
 * @param          : const emxArray_real32_T *a
 *                   emxArray_real32_T *c
 * @return         : void
 */
static void b_conv2AXPYValidCMP(const emxArray_real32_T *a, emxArray_real32_T *c) {
    static const double b[9] = {-0.125, 0.0, 0.125, -0.25, 0.0, 0.25, -0.125, 0.0,
                                0.125};

    emxArray_real32_T *cj;
    double bij;
    const float *a_data;
    float *c_data;
    float *cj_data;
    int b_i;
    int i;
    int ib;
    int j;
    int jb;
    int mc;
    int ub_loop;
    a_data = a->data;
    mc = a->size[0] - 2;
    if ((a->size[0] - 2 == 0) || (a->size[1] - 2 == 0)) {
        i = c->size[0] * c->size[1];
        c->size[0] = a->size[0] - 2;
        c->size[1] = a->size[1] - 2;
        emxEnsureCapacity_real32_T(c, i);
        c_data = c->data;
        ub_loop = (a->size[0] - 2) * (a->size[1] - 2);
        for (i = 0; i < ub_loop; i++) {
            c_data[i] = 0.0F;
        }
    } else {
        i = c->size[0] * c->size[1];
        c->size[0] = a->size[0] - 2;
        c->size[1] = a->size[1] - 2;
        emxEnsureCapacity_real32_T(c, i);
        c_data = c->data;
        ub_loop = a->size[1] - 3;

#pragma omp parallel \
num_threads(omp_get_max_threads()) private(cj_data, cj, bij, ib, jb, b_i)

        {
            emxInit_real32_T(&cj, 1);

#pragma omp for nowait

            for (j = 0; j <= ub_loop; j++) {
                ib = cj->size[0];
                cj->size[0] = mc;
                emxEnsureCapacity_real32_T(cj, ib);
                cj_data = cj->data;
                for (ib = 0; ib < mc; ib++) {
                    cj_data[ib] = 0.0F;
                }

                for (jb = 0; jb < 3; jb++) {
                    for (ib = 0; ib < 3; ib++) {
                        bij = b[(3 * (2 - jb) - ib) + 2];
                        for (b_i = 0; b_i < mc; b_i++) {
                            cj_data[b_i] += (float)bij * a_data[(b_i + ib) + a->size[0] * (j +
                                                                                           jb)];
                        }
                    }
                }

                jb = cj->size[0];
                for (ib = 0; ib < jb; ib++) {
                    c_data[ib + c->size[0] * j] = cj_data[ib];
                }
            }

            emxFree_real32_T(&cj);
        }
    }
}

/**
 * @fn             : b_cosd
 * @brief          :
 * @param          : double *x
 * @return         : void
 */
static void b_cosd(double *x) {
    if (rtIsInf(*x) || rtIsNaN(*x)) {
        *x = rtNaN;
    } else {
        double absx;
        signed char n;
        *x = rt_remd_snf(*x, 360.0);
        absx = fabs(*x);
        if (absx > 180.0) {
            if (*x > 0.0) {
                *x -= 360.0;
            } else {
                *x += 360.0;
            }

            absx = fabs(*x);
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
            *x = cos(*x);
        } else if (n == 1) {
            *x = -sin(*x);
        } else if (n == -1) {
            *x = sin(*x);
        } else {
            *x = -cos(*x);
        }
    }
}

/**
 * @fn             : b_imwarp
 * @brief          :
 * @param          : emxArray_boolean_T *varargin_1
 *                   float varargin_2_RotationAngle
 *                   const float varargin_2_Translation[2]
 *                   const double varargin_5_XWorldLimits[2]
 *                   const double varargin_5_YWorldLimits[2]
 *                   const double varargin_5_ImageSizeAlias[2]
 * @return         : void
 */
static void b_imwarp(emxArray_boolean_T *varargin_1, float varargin_2_RotationAngle, const float varargin_2_Translation[2], const double varargin_5_XWorldLimits[2], const double varargin_5_YWorldLimits[2], const double varargin_5_ImageSizeAlias[2]) {
    emxArray_boolean_T *b_varargin_1;
    imref2d expl_temp;
    int i;
    bool *b_varargin_1_data;
    bool *varargin_1_data;
    varargin_1_data = varargin_1->data;
    emxInit_boolean_T(&b_varargin_1, 2);
    if ((varargin_1->size[0] >= 1) && (varargin_1->size[1] >= 1)) {
        int loop_ub;
        expl_temp.ForcePixelExtentToOne = false;
        expl_temp.ImageSizeAlias[0] = varargin_5_ImageSizeAlias[0];
        expl_temp.YWorldLimits[0] = varargin_5_YWorldLimits[0];
        expl_temp.XWorldLimits[0] = varargin_5_XWorldLimits[0];
        expl_temp.ImageSizeAlias[1] = varargin_5_ImageSizeAlias[1];
        expl_temp.YWorldLimits[1] = varargin_5_YWorldLimits[1];
        expl_temp.XWorldLimits[1] = varargin_5_XWorldLimits[1];
        i = b_varargin_1->size[0] * b_varargin_1->size[1];
        b_varargin_1->size[0] = varargin_1->size[0];
        b_varargin_1->size[1] = varargin_1->size[1];
        emxEnsureCapacity_boolean_T(b_varargin_1, i);
        b_varargin_1_data = b_varargin_1->data;
        loop_ub = varargin_1->size[0] * varargin_1->size[1] - 1;
        for (i = 0; i <= loop_ub; i++) {
            b_varargin_1_data[i] = varargin_1_data[i];
        }

        d_remapAndResampleGeneric2d(b_varargin_1, varargin_2_RotationAngle,
                                    varargin_2_Translation, expl_temp, varargin_1);
    }

    emxFree_boolean_T(&b_varargin_1);
}

/**
 * @fn             : b_interp2_local
 * @brief          :
 * @param          : const double V[6161148]
 *                   const emxArray_real_T *Xq
 *                   const emxArray_real_T *Yq
 *                   emxArray_real_T *Vq
 * @return         : void
 */
static void b_interp2_local(const double V[6161148], const emxArray_real_T *Xq,
                            const emxArray_real_T *Yq, emxArray_real_T *Vq) {
    const double *Xq_data;
    const double *Yq_data;
    double qx1;
    double rx;
    double ry;
    double *Vq_data;
    int ix;
    int iy;
    int k;
    int ub_loop;
    Yq_data = Yq->data;
    Xq_data = Xq->data;
    ub_loop = Vq->size[0] * Vq->size[1];
    Vq->size[0] = Xq->size[0];
    Vq->size[1] = Xq->size[1];
    emxEnsureCapacity_real_T(Vq, ub_loop);
    Vq_data = Vq->data;
    ub_loop = Xq->size[0] * Xq->size[1] - 1;

#pragma omp parallel for num_threads(omp_get_max_threads()) private(ix, iy, qx1, rx, ry)

    for (k = 0; k <= ub_loop; k++) {
        if ((Xq_data[k] >= 1.0) && (Xq_data[k] <= 5796.0) && (Yq_data[k] >= 1.0) &&
            (Yq_data[k] <= 1063.0)) {
            if (Xq_data[k] <= 1.0) {
                ix = 1;
            } else if (Xq_data[k] <= 5795.0) {
                ix = (int)floor(Xq_data[k]);
            } else {
                ix = 5795;
            }

            if (Yq_data[k] <= 1.0) {
                iy = 1;
            } else if (Yq_data[k] <= 1062.0) {
                iy = (int)floor(Yq_data[k]);
            } else {
                iy = 1062;
            }

            if (Xq_data[k] == ix) {
                ix = iy + 1063 * (ix - 1);
                qx1 = V[ix - 1];
                rx = V[ix];
            } else if (Xq_data[k] == (double)ix + 1.0) {
                ix = iy + 1063 * ix;
                qx1 = V[ix - 1];
                rx = V[ix];
            } else {
                rx = (Xq_data[k] - (double)ix) / (((double)ix + 1.0) - (double)ix);
                if (V[(iy + 1063 * (ix - 1)) - 1] == V[(iy + 1063 * ix) - 1]) {
                    qx1 = V[(iy + 1063 * (ix - 1)) - 1];
                } else {
                    qx1 = (1.0 - rx) * V[(iy + 1063 * (ix - 1)) - 1] + rx * V[(iy + 1063 *
                                                                                        ix) -
                                                                              1];
                }

                if (V[iy + 1063 * (ix - 1)] == V[iy + 1063 * ix]) {
                    rx = V[iy + 1063 * (ix - 1)];
                } else {
                    rx = (1.0 - rx) * V[iy + 1063 * (ix - 1)] + rx * V[iy + 1063 * ix];
                }
            }

            if ((Yq_data[k] == iy) || (qx1 == rx)) {
                Vq_data[k] = qx1;
            } else if (Yq_data[k] == (double)iy + 1.0) {
                Vq_data[k] = rx;
            } else {
                ry = (Yq_data[k] - (double)iy) / (((double)iy + 1.0) - (double)iy);
                Vq_data[k] = (1.0 - ry) * qx1 + ry * rx;
            }
        } else {
            Vq_data[k] = 0.0;
        }
    }
}

/**
 * @fn             : b_inv
 * @brief          :
 * @param          : const float x[9]
 *                   float y[9]
 * @return         : void
 */
static void b_inv(const float x[9], float y[9]) {
    float b_x[9];
    float absx11;
    float absx21;
    float absx31;
    int p1;
    int p2;
    int p3;
    for (p1 = 0; p1 < 9; p1++) {
        b_x[p1] = x[p1];
    }

    p1 = 0;
    p2 = 3;
    p3 = 6;
    absx11 = fabsf(x[0]);
    absx21 = fabsf(x[1]);
    absx31 = fabsf(x[2]);
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
    if (fabsf(b_x[5]) > fabsf(b_x[4])) {
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
    y[p1] = ((1.0F - b_x[3] * absx21) - b_x[6] * absx11) / b_x[0];
    y[p1 + 1] = absx21;
    y[p1 + 2] = absx11;
    absx11 = -b_x[5] / b_x[8];
    absx21 = (1.0F - b_x[7] * absx11) / b_x[4];
    y[p2] = -(b_x[3] * absx21 + b_x[6] * absx11) / b_x[0];
    y[p2 + 1] = absx21;
    y[p2 + 2] = absx11;
    absx11 = 1.0F / b_x[8];
    absx21 = -b_x[7] * absx11 / b_x[4];
    y[p3] = -(b_x[3] * absx21 + b_x[6] * absx11) / b_x[0];
    y[p3 + 1] = absx21;
    y[p3 + 2] = absx11;
}

/**
 * @fn             : b_maximum
 * @brief          :
 * @param          : const double x[8]
 * @return         : double
 */
static double b_maximum(const double x[8]) {
    double ex;
    int idx;
    int k;
    if (!rtIsNaN(x[0])) {
        idx = 1;
    } else {
        bool exitg1;
        idx = 0;
        k = 2;
        exitg1 = false;
        while ((!exitg1) && (k < 9)) {
            if (!rtIsNaN(x[k - 1])) {
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
        for (k = idx; k < 9; k++) {
            double d;
            d = x[k - 1];
            if (ex < d) {
                ex = d;
            }
        }
    }

    return ex;
}

/**
 * @fn             : b_mean
 * @brief          :
 * @param          : const emxArray_real32_T *x
 *                   float y[2]
 * @return         : void
 */
static void b_mean(const emxArray_real32_T *x, float y[2]) {
    const float *x_data;
    int ib;
    int k;
    int xi;
    x_data = x->data;
    if (x->size[0] == 0) {
        y[0] = 0.0F;
        y[1] = 0.0F;
    } else {
        int firstBlockLength;
        int lastBlockLength;
        int nblocks;
        if (x->size[0] <= 1024) {
            firstBlockLength = x->size[0];
            lastBlockLength = 0;
            nblocks = 1;
        } else {
            firstBlockLength = 1024;
            nblocks = (int)((unsigned int)x->size[0] >> 10);
            lastBlockLength = x->size[0] - (nblocks << 10);
            if (lastBlockLength > 0) {
                nblocks++;
            } else {
                lastBlockLength = 1024;
            }
        }

        for (xi = 0; xi < 2; xi++) {
            int xpageoffset;
            xpageoffset = xi * x->size[0];
            y[xi] = x_data[xpageoffset];
            for (k = 2; k <= firstBlockLength; k++) {
                y[xi] += x_data[(xpageoffset + k) - 1];
            }

            for (ib = 2; ib <= nblocks; ib++) {
                float bsum;
                int hi;
                int xblockoffset;
                xblockoffset = xpageoffset + ((ib - 1) << 10);
                bsum = x_data[xblockoffset];
                if (ib == nblocks) {
                    hi = lastBlockLength;
                } else {
                    hi = 1024;
                }

                for (k = 2; k <= hi; k++) {
                    bsum += x_data[(xblockoffset + k) - 1];
                }

                y[xi] += bsum;
            }
        }
    }

    y[0] /= (float)x->size[0];
    y[1] /= (float)x->size[0];
}

/**
 * @fn             : b_minimum
 * @brief          :
 * @param          : const double x[8]
 * @return         : double
 */
static double b_minimum(const double x[8]) {
    double ex;
    int idx;
    int k;
    if (!rtIsNaN(x[0])) {
        idx = 1;
    } else {
        bool exitg1;
        idx = 0;
        k = 2;
        exitg1 = false;
        while ((!exitg1) && (k < 9)) {
            if (!rtIsNaN(x[k - 1])) {
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
        for (k = idx; k < 9; k++) {
            double d;
            d = x[k - 1];
            if (ex > d) {
                ex = d;
            }
        }
    }

    return ex;
}

/**
 * @fn             : b_mod
 * @brief          :
 * @param          : float x
 * @return         : float
 */
static float b_mod(float x) {
    float r;
    if (rtIsNaNF(x) || rtIsInfF(x)) {
        r = rtNaNF;
    } else if (x == 0.0F) {
        r = 0.0F;
    } else {
        r = fmodf(x, 360.0F);
        if (r == 0.0F) {
            r = 0.0F;
        } else if (x < 0.0F) {
            r += 360.0F;
        }
    }

    return r;
}

/**
 * @fn             : b_padarray
 * @brief          :
 * @param          : const emxArray_real_T *varargin_1
 *                   const double varargin_2[2]
 *                   emxArray_real_T *b
 * @return         : void
 */
static void b_padarray(const emxArray_real_T *varargin_1, const double varargin_2[2], emxArray_real_T *b) {
    const double *varargin_1_data;
    double *b_data;
    int b_i;
    int i;
    int j;
    int k;
    varargin_1_data = varargin_1->data;
    if ((varargin_1->size[0] == 0) || (varargin_1->size[1] == 0)) {
        double padSize_idx_0;
        double padSize_idx_1;
        int loop_ub;
        padSize_idx_0 = (double)varargin_1->size[0] + varargin_2[0];
        padSize_idx_1 = (double)varargin_1->size[1] + varargin_2[1];
        i = b->size[0] * b->size[1] * b->size[2];
        b->size[0] = (int)padSize_idx_0;
        b->size[1] = (int)padSize_idx_1;
        b->size[2] = 3;
        emxEnsureCapacity_real_T(b, i);
        b_data = b->data;
        loop_ub = (int)padSize_idx_0 * (int)padSize_idx_1 * 3;
        for (i = 0; i < loop_ub; i++) {
            b_data[i] = 0.0;
        }
    } else {
        int i1;
        int i2;
        int i3;
        int loop_ub;
        i = b->size[0] * b->size[1] * b->size[2];
        b->size[0] = (int)((double)varargin_1->size[0] + varargin_2[0]);
        b->size[1] = (int)((double)varargin_1->size[1] + varargin_2[1]);
        b->size[2] = 3;
        emxEnsureCapacity_real_T(b, i);
        b_data = b->data;
        i = varargin_1->size[1] + 1;
        loop_ub = varargin_1->size[1];
        i1 = varargin_1->size[0] + 1;
        i2 = b->size[1];
        i3 = b->size[0];
        for (k = 0; k < 3; k++) {
            for (j = i; j <= i2; j++) {
                int i4;
                i4 = b->size[0];
                for (b_i = 0; b_i < i4; b_i++) {
                    b_data[(b_i + b->size[0] * (j - 1)) + b->size[0] * b->size[1] * k] =
                        0.0;
                }
            }

            for (j = 0; j < loop_ub; j++) {
                for (b_i = i1; b_i <= i3; b_i++) {
                    b_data[((b_i + b->size[0] * j) + b->size[0] * b->size[1] * k) - 1] =
                        0.0;
                }
            }
        }

        i = varargin_1->size[1];
        loop_ub = varargin_1->size[0];
        for (k = 0; k < 3; k++) {
            for (j = 0; j < i; j++) {
                for (b_i = 0; b_i < loop_ub; b_i++) {
                    b_data[(b_i + b->size[0] * j) + b->size[0] * b->size[1] * k] =
                        varargin_1_data[(b_i + varargin_1->size[0] * j) + varargin_1->size[0] * varargin_1->size[1] * k];
                }
            }
        }
    }
}

/**
 * @fn             : b_rand
 * @brief          :
 * @param          : void
 * @return         : double
 */
static double b_rand(void) {
    double r;
    int j;
    int kk;

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
        for (j = 0; j < 2; j++) {
            unsigned int mti;
            unsigned int y;
            mti = state[624] + 1U;
            if (state[624] + 1U >= 625U) {
                for (kk = 0; kk < 227; kk++) {
                    y = (state[kk] & 2147483648U) | (state[kk + 1] & 2147483647U);
                    if ((y & 1U) == 0U) {
                        y >>= 1U;
                    } else {
                        y = y >> 1U ^ 2567483615U;
                    }

                    state[kk] = state[kk + 397] ^ y;
                }

                for (kk = 0; kk < 396; kk++) {
                    y = (state[kk + 227] & 2147483648U) | (state[kk + 228] & 2147483647U);
                    if ((y & 1U) == 0U) {
                        y >>= 1U;
                    } else {
                        y = y >> 1U ^ 2567483615U;
                    }

                    state[kk + 227] = state[kk] ^ y;
                }

                y = (state[623] & 2147483648U) | (state[0] & 2147483647U);
                if ((y & 1U) == 0U) {
                    y >>= 1U;
                } else {
                    y = y >> 1U ^ 2567483615U;
                }

                state[623] = state[396] ^ y;
                mti = 1U;
            }

            y = state[(int)mti - 1];
            state[624] = mti;
            y ^= y >> 11U;
            y ^= y << 7U & 2636928640U;
            y ^= y << 15U & 4022730752U;
            u[j] = y ^ y >> 18U;
        }

        u[0] >>= 5U;
        u[1] >>= 6U;
        r = 1.1102230246251565E-16 * ((double)u[0] * 6.7108864E+7 + (double)u[1]);
    } while (r == 0.0);

    return r;
}

/**
 * @fn             : b_remapAndResampleGeneric2d
 * @brief          :
 * @param          : const emxArray_real_T *inputImage
 *                   const float tform_A23[6]
 *                   const imref2d outputRef
 *                   emxArray_real_T *outputImage
 * @return         : void
 */
static void b_remapAndResampleGeneric2d(const emxArray_real_T *inputImage, const float tform_A23[6], const imref2d outputRef, emxArray_real_T *outputImage) {
    emxArray_real_T *XIntrinsic;
    emxArray_real_T *YIntrinsic;
    emxArray_real_T *b_inputImage;
    emxArray_real_T *r;
    emxArray_real_T *srcXIntrinsic;
    emxArray_real_T *srcYIntrinsic;
    const double *inputImage_data;
    double dstXWorld_val;
    double dstYWorld_val;
    double extentY;
    double nRows;
    double *srcXIntrinsic_data;
    double *srcYIntrinsic_data;
    float b_tform_A23[9];
    float tinv[9];
    float srcXWorld_val;
    float srcYWorld_val;
    int colIdx;
    int i;
    int i1;
    int i2;
    int plane;
    int rowIdx;
    int ub_loop;
    inputImage_data = inputImage->data;
    for (i = 0; i < 2; i++) {
        b_tform_A23[3 * i] = tform_A23[i];
        b_tform_A23[3 * i + 1] = tform_A23[i + 2];
        b_tform_A23[3 * i + 2] = tform_A23[i + 4];
    }

    b_tform_A23[6] = 0.0F;
    b_tform_A23[7] = 0.0F;
    b_tform_A23[8] = 1.0F;
    b_inv(b_tform_A23, tinv);
    nRows = outputRef.ImageSizeAlias[0];
    emxInit_real_T(&srcXIntrinsic, 2);
    i = srcXIntrinsic->size[0] * srcXIntrinsic->size[1];
    srcXIntrinsic->size[0] = (int)outputRef.ImageSizeAlias[0];
    srcXIntrinsic->size[1] = (int)outputRef.ImageSizeAlias[1];
    emxEnsureCapacity_real_T(srcXIntrinsic, i);
    srcXIntrinsic_data = srcXIntrinsic->data;
    emxInit_real_T(&srcYIntrinsic, 2);
    i = srcYIntrinsic->size[0] * srcYIntrinsic->size[1];
    srcYIntrinsic->size[0] = (int)outputRef.ImageSizeAlias[0];
    srcYIntrinsic->size[1] = (int)outputRef.ImageSizeAlias[1];
    emxEnsureCapacity_real_T(srcYIntrinsic, i);
    srcYIntrinsic_data = srcYIntrinsic->data;
    ub_loop = (int)outputRef.ImageSizeAlias[1] - 1;

#pragma omp parallel for num_threads(omp_get_max_threads()) private(srcYWorld_val, srcXWorld_val, dstYWorld_val, dstXWorld_val, i1, extentY, rowIdx)

    for (colIdx = 0; colIdx <= ub_loop; colIdx++) {
        if (outputRef.ForcePixelExtentToOne) {
            dstYWorld_val = 1.0;
        } else {
            dstYWorld_val = (outputRef.XWorldLimits[1] - outputRef.XWorldLimits[0]) /
                            outputRef.ImageSizeAlias[1];
        }

        dstXWorld_val = outputRef.XWorldLimits[0] + (((double)colIdx + 1.0) - 0.5) *
                                                        dstYWorld_val;
        i1 = (int)nRows;
        if ((int)nRows - 1 >= 0) {
            if (outputRef.ForcePixelExtentToOne) {
                extentY = 1.0;
            } else {
                extentY = (outputRef.YWorldLimits[1] - outputRef.YWorldLimits[0]) /
                          outputRef.ImageSizeAlias[0];
            }
        }

        for (rowIdx = 0; rowIdx < i1; rowIdx++) {
            dstYWorld_val = outputRef.YWorldLimits[0] + (((double)rowIdx + 1.0) - 0.5) * extentY;
            srcXWorld_val = (tinv[0] * (float)dstXWorld_val + tinv[1] * (float)
                                                                            dstYWorld_val) +
                            tinv[2];
            srcYWorld_val = (tinv[3] * (float)dstXWorld_val + tinv[4] * (float)
                                                                            dstYWorld_val) +
                            tinv[5];
            srcXIntrinsic_data[rowIdx + srcXIntrinsic->size[0] * colIdx] =
                (srcXWorld_val - 0.5F) + 0.5F;
            srcYIntrinsic_data[rowIdx + srcYIntrinsic->size[0] * colIdx] =
                (srcYWorld_val - 0.5F) + 0.5F;
        }
    }

    i = outputImage->size[0] * outputImage->size[1] * outputImage->size[2];
    outputImage->size[0] = srcXIntrinsic->size[0];
    outputImage->size[1] = srcXIntrinsic->size[1];
    outputImage->size[2] = 3;
    emxEnsureCapacity_real_T(outputImage, i);
    srcYIntrinsic_data = outputImage->data;
    ub_loop = srcXIntrinsic->size[0] * srcXIntrinsic->size[1] * 3;
    for (i = 0; i < ub_loop; i++) {
        srcYIntrinsic_data[i] = 0.0;
    }

    emxInit_real_T(&XIntrinsic, 2);
    if (inputImage->size[1] < 1) {
        XIntrinsic->size[0] = 1;
        XIntrinsic->size[1] = 0;
    } else {
        i = XIntrinsic->size[0] * XIntrinsic->size[1];
        XIntrinsic->size[0] = 1;
        XIntrinsic->size[1] = inputImage->size[1];
        emxEnsureCapacity_real_T(XIntrinsic, i);
        srcXIntrinsic_data = XIntrinsic->data;
        ub_loop = inputImage->size[1] - 1;
        for (i = 0; i <= ub_loop; i++) {
            srcXIntrinsic_data[i] = (double)i + 1.0;
        }
    }

    emxInit_real_T(&YIntrinsic, 2);
    if (inputImage->size[0] < 1) {
        YIntrinsic->size[0] = 1;
        YIntrinsic->size[1] = 0;
    } else {
        i = YIntrinsic->size[0] * YIntrinsic->size[1];
        YIntrinsic->size[0] = 1;
        YIntrinsic->size[1] = inputImage->size[0];
        emxEnsureCapacity_real_T(YIntrinsic, i);
        srcXIntrinsic_data = YIntrinsic->data;
        ub_loop = inputImage->size[0] - 1;
        for (i = 0; i <= ub_loop; i++) {
            srcXIntrinsic_data[i] = (double)i + 1.0;
        }
    }

    ub_loop = inputImage->size[1];
    emxInit_real_T(&b_inputImage, 2);
    emxInit_real_T(&r, 2);
    for (plane = 0; plane < 3; plane++) {
        int loop_ub;
        i = b_inputImage->size[0] * b_inputImage->size[1];
        b_inputImage->size[0] = inputImage->size[0];
        b_inputImage->size[1] = inputImage->size[1];
        emxEnsureCapacity_real_T(b_inputImage, i);
        srcXIntrinsic_data = b_inputImage->data;
        for (i = 0; i < ub_loop; i++) {
            loop_ub = inputImage->size[0];
            for (i2 = 0; i2 < loop_ub; i2++) {
                srcXIntrinsic_data[i2 + b_inputImage->size[0] * i] = inputImage_data[(i2 + inputImage->size[0] * i) + inputImage->size[0] * inputImage->size[1] * plane];
            }
        }

        interp2(XIntrinsic, YIntrinsic, b_inputImage, srcXIntrinsic, srcYIntrinsic,
                r);
        srcXIntrinsic_data = r->data;
        loop_ub = r->size[1];
        for (i = 0; i < loop_ub; i++) {
            int b_loop_ub;
            b_loop_ub = r->size[0];
            for (i2 = 0; i2 < b_loop_ub; i2++) {
                srcYIntrinsic_data[(i2 + outputImage->size[0] * i) + outputImage->size[0] * outputImage->size[1] * plane] = srcXIntrinsic_data[i2 + r->size[0] *
                                                                                                                                                        i];
            }
        }
    }

    emxFree_real_T(&r);
    emxFree_real_T(&b_inputImage);
    emxFree_real_T(&YIntrinsic);
    emxFree_real_T(&XIntrinsic);
    emxFree_real_T(&srcYIntrinsic);
    emxFree_real_T(&srcXIntrinsic);
}

/**
 * @fn             : b_sind
 * @brief          :
 * @param          : double *x
 * @return         : void
 */
static void b_sind(double *x) {
    if (rtIsInf(*x) || rtIsNaN(*x)) {
        *x = rtNaN;
    } else {
        double absx;
        signed char n;
        *x = rt_remd_snf(*x, 360.0);
        absx = fabs(*x);
        if (absx > 180.0) {
            if (*x > 0.0) {
                *x -= 360.0;
            } else {
                *x += 360.0;
            }

            absx = fabs(*x);
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
            *x = sin(*x);
        } else if (n == 1) {
            *x = cos(*x);
        } else if (n == -1) {
            *x = -cos(*x);
        } else {
            *x = -sin(*x);
        }
    }
}

/**
 * @fn             : b_svd
 * @brief          :
 * @param          : const float A[4]
 *                   float U[4]
 *                   float s[2]
 *                   float V[4]
 * @return         : void
 */
static void b_svd(const float A[4], float U[4], float s[2], float V[4]) {
    float b_s[2];
    float e[2];
    float A_idx_3;
    float f;
    float nrm;
    float rt;
    float sm;
    float snorm;
    float sqds;
    float temp;
    int iter;
    int k;
    int kase;
    int m;
    int q;
    int qs;
    temp = A[0];
    sm = A[1];
    sqds = A[2];
    A_idx_3 = A[3];
    nrm = xnrm2(A);
    if (nrm > 0.0F) {
        if (A[0] < 0.0F) {
            b_s[0] = -nrm;
        } else {
            b_s[0] = nrm;
        }

        if (fabsf(b_s[0]) >= 9.86076132E-32F) {
            rt = 1.0F / b_s[0];
            temp = rt * A[0];
            sm = rt * A[1];
        } else {
            temp = A[0] / b_s[0];
            sm = A[1] / b_s[0];
        }

        temp++;
        b_s[0] = -b_s[0];
        rt = -((temp * A[2] + sm * A[3]) / temp);
        if (!(rt == 0.0F)) {
            sqds = A[2] + rt * temp;
            A_idx_3 = A[3] + rt * sm;
        }
    } else {
        b_s[0] = 0.0F;
    }

    m = 2;
    b_s[1] = A_idx_3;
    e[0] = sqds;
    e[1] = 0.0F;
    U[2] = 0.0F;
    U[3] = 1.0F;
    if (b_s[0] != 0.0F) {
        rt = -((temp * 0.0F + sm) / temp);
        if (!(rt == 0.0F)) {
            U[2] = rt * temp;
            U[3] = rt * sm + 1.0F;
        }

        U[1] = -sm;
        U[0] = -temp + 1.0F;
    } else {
        U[1] = 0.0F;
        U[0] = 1.0F;
    }

    V[2] = 0.0F;
    V[3] = 1.0F;
    V[1] = 0.0F;
    V[0] = 1.0F;
    for (q = 0; q < 2; q++) {
        nrm = b_s[q];
        if (nrm != 0.0F) {
            rt = fabsf(nrm);
            nrm /= rt;
            b_s[q] = rt;
            if (q + 1 < 2) {
                e[0] /= nrm;
            }

            kase = q << 1;
            qs = kase + 2;
            for (k = kase + 1; k <= qs; k++) {
                U[k - 1] *= nrm;
            }
        }

        if ((q + 1 < 2) && (e[0] != 0.0F)) {
            rt = fabsf(e[0]);
            nrm = rt / e[0];
            e[0] = rt;
            b_s[1] *= nrm;
            V[2] *= nrm;
            V[3] *= nrm;
        }
    }

    iter = 0;
    snorm = fmaxf(fmaxf(0.0F, fmaxf(fabsf(b_s[0]), fabsf(e[0]))), fmaxf(fabsf(b_s
                                                                                  [1]),
                                                                        0.0F));
    while ((m > 0) && (iter < 75)) {
        int ii_tmp_tmp;
        bool exitg1;
        ii_tmp_tmp = m - 1;
        q = m - 1;
        exitg1 = false;
        while (!(exitg1 || (q == 0))) {
            nrm = fabsf(e[0]);
            if ((nrm <= 1.1920929E-7F * (fabsf(b_s[0]) + fabsf(b_s[1]))) || (nrm <= 9.86076132E-32F) || ((iter > 20) && (nrm <= 1.1920929E-7F * snorm))) {
                e[0] = 0.0F;
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
                    nrm = 0.0F;
                    if (kase < m) {
                        nrm = fabsf(e[0]);
                    }

                    if (kase > q + 1) {
                        nrm += fabsf(e[0]);
                    }

                    rt = fabsf(b_s[kase - 1]);
                    if ((rt <= 1.1920929E-7F * nrm) || (rt <= 9.86076132E-32F)) {
                        b_s[kase - 1] = 0.0F;
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
                e[0] = 0.0F;
                for (k = ii_tmp_tmp; k >= q + 1; k--) {
                    xrotg(&b_s[0], &f, &A_idx_3, &rt);
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
                e[q - 1] = 0.0F;
                for (k = q + 1; k <= m; k++) {
                    xrotg(&b_s[k - 1], &f, &A_idx_3, &sm);
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
                float scale;
                nrm = b_s[m - 1];
                scale = fmaxf(fmaxf(fmaxf(fmaxf(fabsf(nrm), fabsf(b_s[0])), fabsf(e[0])),
                                    fabsf(b_s[q])),
                              fabsf(e[q]));
                sm = nrm / scale;
                rt = b_s[0] / scale;
                nrm = e[0] / scale;
                sqds = b_s[q] / scale;
                temp = ((rt + sm) * (rt - sm) + nrm * nrm) / 2.0F;
                A_idx_3 = sm * nrm;
                A_idx_3 *= A_idx_3;
                if ((temp != 0.0F) || (A_idx_3 != 0.0F)) {
                    rt = sqrtf(temp * temp + A_idx_3);
                    if (temp < 0.0F) {
                        rt = -rt;
                    }

                    rt = A_idx_3 / (temp + rt);
                } else {
                    rt = 0.0F;
                }

                f = (sqds + sm) * (sqds - sm) + rt;
                rt = sqds * (e[q] / scale);
                for (k = q + 1; k < 2; k++) {
                    xrotg(&f, &rt, &A_idx_3, &sm);
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
                    xrotg(&b_s[0], &rt, &A_idx_3, &sm);
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
                if (b_s[q] < 0.0F) {
                    b_s[q] = -b_s[q];
                    kase = q << 1;
                    qs = kase + 2;
                    for (k = kase + 1; k <= qs; k++) {
                        V[k - 1] = -V[k - 1];
                    }
                }

                while ((q + 1 < 2) && (b_s[0] < b_s[1])) {
                    rt = b_s[0];
                    b_s[0] = b_s[1];
                    b_s[1] = rt;
                    xswap(V);
                    xswap(U);
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

/**
 * @fn             : b_vehicleToLocalImage
 * @brief          :
 * @param          : const double refBirdsEye_OutputView[4]
 *                   const double refBirdsEye_Sensor_Intrinsics_K[9]
 *                   double refBirdsEye_Sensor_Height
 *                   double refBirdsEye_Sensor_Pitch
 *                   double refBirdsEye_Sensor_Yaw
 *                   double refBirdsEye_Sensor_Roll
 *                   const double c_refBirdsEye_Sensor_SensorLoca[2]
 *                   const double refBirdsEye_Scale[2]
 *                   const double refImg_XWorldLimits[2]
 *                   const double refImg_YWorldLimits[2]
 *                   const double refImg_ImageSizeAlias[2]
 *                   bool refImg_ForcePixelExtentToOne
 *                   double localImagePts[8]
 * @return         : void
 */
static void b_vehicleToLocalImage(const double refBirdsEye_OutputView[4], const double refBirdsEye_Sensor_Intrinsics_K[9], double refBirdsEye_Sensor_Height,
                                  double refBirdsEye_Sensor_Pitch, double refBirdsEye_Sensor_Yaw, double refBirdsEye_Sensor_Roll, const double c_refBirdsEye_Sensor_SensorLoca[2],
                                  const double refBirdsEye_Scale[2], const double refImg_XWorldLimits[2], const double refImg_YWorldLimits[2], const double refImg_ImageSizeAlias[2], bool refImg_ForcePixelExtentToOne, double localImagePts[8]) {
    static const double a[12] = {-4.5, 0.5, 0.5, -4.5, 1.2, 1.2, -1.2, -1.2, 1.0,
                                 1.0, 1.0, 1.0};

    double U[12];
    double b_refBirdsEye_Scale[9];
    double b_t3_T[9];
    double t3_T[9];
    double d;
    double extentX;
    double extentY;
    int i;
    int jtilecol;

    /*  Brief: 在2D鸟瞰图中把局部vehiclePts转换到当前四副鱼眼环视图拼接的局部图像坐标 */
    /*  Details: */
    /*     vehicle坐标系符合matlab自动驾驶工具箱定义，即x轴朝向车辆正前方，y轴朝向车辆左侧。 */
    /*   */
    /*  Syntax:   */
    /*      localImagePts = vehicleToLocalImage(refBirdsEye,refImg,vehiclePts) */
    /*   */
    /*  Inputs: */
    /*     refBirdsEye - [1,1] size,[birdsEyeView] type,matlab build-in type */
    /*     refImg - [1,1] size,[imref2d] type,matlab build-in type,四副环视拼接参考对象 */
    /*     vehiclePts - [m,2] size,[double] type,待转换到局部车辆坐标点 */
    /*   */
    /*  Outputs: */
    /*     localImagePts - [m,2] size,[double] type,四副环视拼接的鸟瞰图像中的局部像素坐标 */
    /*   */
    /*  Example:  */
    /*     None */
    /*   */
    /*  See also: None */
    /*  Author:                          cuixingxing */
    /*  Email:                           xingxing.cui@long-horn.com */
    /*  Created:                         13-Oct-2022 09:00:37 */
    /*  Version history revision notes: */
    /*                                   None */
    /*  Implementation In Matlab R2022b */
    /*  Copyright © 2022 long-horn.All Rights Reserved. */
    /*  */
    c_monoCamera_get_ImageToVehicle(refBirdsEye_Sensor_Intrinsics_K,
                                    refBirdsEye_Sensor_Height, refBirdsEye_Sensor_Pitch, refBirdsEye_Sensor_Yaw,
                                    refBirdsEye_Sensor_Roll, c_refBirdsEye_Sensor_SensorLoca, t3_T);
    b_refBirdsEye_Scale[0] = refBirdsEye_Scale[0];
    b_refBirdsEye_Scale[3] = 0.0;
    b_refBirdsEye_Scale[6] = 0.0;
    b_refBirdsEye_Scale[1] = 0.0;
    b_refBirdsEye_Scale[4] = refBirdsEye_Scale[1];
    b_refBirdsEye_Scale[7] = 0.0;
    b_refBirdsEye_Scale[2] = refBirdsEye_Scale[0] * refBirdsEye_OutputView[3] +
                             1.0;
    b_refBirdsEye_Scale[5] = refBirdsEye_Scale[1] * refBirdsEye_OutputView[1] +
                             1.0;
    b_refBirdsEye_Scale[8] = 1.0;
    for (i = 0; i < 3; i++) {
        extentX = t3_T[i];
        extentY = t3_T[i + 3];
        d = t3_T[i + 6];
        for (jtilecol = 0; jtilecol < 3; jtilecol++) {
            b_t3_T[i + 3 * jtilecol] = (extentX * (double)iv[3 * jtilecol] + extentY *
                                                                                 (double)iv[3 * jtilecol + 1]) +
                                       d * (double)iv[3 * jtilecol + 2];
        }

        extentX = b_t3_T[i];
        extentY = b_t3_T[i + 3];
        d = b_t3_T[i + 6];
        for (jtilecol = 0; jtilecol < 3; jtilecol++) {
            t3_T[i + 3 * jtilecol] = (extentX * b_refBirdsEye_Scale[3 * jtilecol] +
                                      extentY * b_refBirdsEye_Scale[3 * jtilecol + 1]) +
                                     d *
                                         b_refBirdsEye_Scale[3 * jtilecol + 2];
        }
    }

    inv(t3_T, b_t3_T);
    c_monoCamera_get_ImageToVehicle(refBirdsEye_Sensor_Intrinsics_K,
                                    refBirdsEye_Sensor_Height, refBirdsEye_Sensor_Pitch, refBirdsEye_Sensor_Yaw,
                                    refBirdsEye_Sensor_Roll, c_refBirdsEye_Sensor_SensorLoca, t3_T);
    for (i = 0; i < 3; i++) {
        extentX = b_t3_T[i];
        extentY = b_t3_T[i + 3];
        d = b_t3_T[i + 6];
        for (jtilecol = 0; jtilecol < 3; jtilecol++) {
            b_refBirdsEye_Scale[i + 3 * jtilecol] = (extentX * t3_T[3 * jtilecol] +
                                                     extentY * t3_T[3 * jtilecol + 1]) +
                                                    d * t3_T[3 * jtilecol + 2];
        }
    }

    inv(b_refBirdsEye_Scale, b_t3_T);
    for (i = 0; i < 4; i++) {
        extentX = a[i];
        extentY = a[i + 4];
        d = a[i + 8];
        for (jtilecol = 0; jtilecol < 3; jtilecol++) {
            U[i + (jtilecol << 2)] = (extentX * b_t3_T[3 * jtilecol] + extentY *
                                                                           b_t3_T[3 * jtilecol + 1]) +
                                     d * b_t3_T[3 * jtilecol + 2];
        }
    }

    for (jtilecol = 0; jtilecol < 2; jtilecol++) {
        extentX = U[9];
        extentY = U[10];
        d = U[11];
        i = jtilecol << 2;
        U[i] /= U[8];
        U[i + 1] /= extentX;
        U[i + 2] /= extentY;
        U[i + 3] /= d;
    }

    if (refImg_ForcePixelExtentToOne) {
        extentX = 1.0;
        extentY = 1.0;
    } else {
        extentX = (refImg_XWorldLimits[1] - refImg_XWorldLimits[0]) /
                  refImg_ImageSizeAlias[1];
        extentY = (refImg_YWorldLimits[1] - refImg_YWorldLimits[0]) /
                  refImg_ImageSizeAlias[0];
    }

    localImagePts[0] = (U[0] - refImg_XWorldLimits[0]) / extentX + 0.5;
    localImagePts[1] = (U[1] - refImg_XWorldLimits[0]) / extentX + 0.5;
    localImagePts[2] = (U[2] - refImg_XWorldLimits[0]) / extentX + 0.5;
    localImagePts[3] = (U[3] - refImg_XWorldLimits[0]) / extentX + 0.5;
    localImagePts[4] = (U[4] - refImg_YWorldLimits[0]) / extentY + 0.5;
    localImagePts[5] = (U[5] - refImg_YWorldLimits[0]) / extentY + 0.5;
    localImagePts[6] = (U[6] - refImg_YWorldLimits[0]) / extentY + 0.5;
    localImagePts[7] = (U[7] - refImg_YWorldLimits[0]) / extentY + 0.5;
}

/**
 * @fn             : b_xnrm2
 * @brief          :
 * @param          : const double x_data[]
 * @return         : double
 */
static double b_xnrm2(const double x_data[]) {
    return fabs(x_data[1]);
}

/**
 * @fn             : b_xrotg
 * @brief          :
 * @param          : double *a
 *                   double *b
 *                   double *c
 *                   double *s
 * @return         : void
 */
static void b_xrotg(double *a, double *b, double *c, double *s) {
    double absa;
    double absb;
    double roe;
    double scale;
    roe = *b;
    absa = fabs(*a);
    absb = fabs(*b);
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
        scale *= sqrt(ads * ads + bds * bds);
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

/**
 * @fn             : binary_expand_op
 * @brief          :
 * @param          : emxArray_real_T *in1
 *                   const emxArray_real_T *in2
 *                   const emxArray_real_T *in3
 *                   const emxArray_real_T *in4
 * @return         : void
 */
static void binary_expand_op(emxArray_real_T *in1, const emxArray_real_T *in2,
                             const emxArray_real_T *in3, const emxArray_real_T *in4) {
    emxArray_real_T *b_in2;
    const double *in2_data;
    const double *in3_data;
    const double *in4_data;
    double *b_in2_data;
    double *in1_data;
    int i;
    int i1;
    int i2;
    int in2_idx_0;
    int in2_idx_1;
    int in4_idx_0;
    int in4_idx_1;
    int stride_0_0;
    int stride_0_1;
    int stride_1_0;
    int stride_1_1;
    int stride_2_0;
    int stride_2_1;
    int stride_3_0;
    int stride_3_1;
    in4_data = in4->data;
    in3_data = in3->data;
    in2_data = in2->data;
    in1_data = in1->data;
    in2_idx_0 = in2->size[0];
    in2_idx_1 = in2->size[1];
    in4_idx_0 = in4->size[0];
    in4_idx_1 = in4->size[1];
    emxInit_real_T(&b_in2, 3);
    i = b_in2->size[0] * b_in2->size[1] * b_in2->size[2];
    if (in1->size[0] == 1) {
        i1 = in4_idx_0;
    } else {
        i1 = in1->size[0];
    }

    if (i1 == 1) {
        if (in3->size[0] == 1) {
            b_in2->size[0] = in2_idx_0;
        } else {
            b_in2->size[0] = in3->size[0];
        }
    } else if (in1->size[0] == 1) {
        b_in2->size[0] = in4_idx_0;
    } else {
        b_in2->size[0] = in1->size[0];
    }

    if (in1->size[1] == 1) {
        i1 = in4_idx_1;
    } else {
        i1 = in1->size[1];
    }

    if (i1 == 1) {
        if (in3->size[1] == 1) {
            b_in2->size[1] = in2_idx_1;
        } else {
            b_in2->size[1] = in3->size[1];
        }
    } else if (in1->size[1] == 1) {
        b_in2->size[1] = in4_idx_1;
    } else {
        b_in2->size[1] = in1->size[1];
    }

    b_in2->size[2] = 3;
    emxEnsureCapacity_real_T(b_in2, i);
    b_in2_data = b_in2->data;
    stride_0_0 = (in2_idx_0 != 1);
    stride_0_1 = (in2_idx_1 != 1);
    stride_1_0 = (in3->size[0] != 1);
    stride_1_1 = (in3->size[1] != 1);
    stride_2_0 = (in4_idx_0 != 1);
    stride_2_1 = (in4_idx_1 != 1);
    stride_3_0 = (in1->size[0] != 1);
    stride_3_1 = (in1->size[1] != 1);
    if (in1->size[1] == 1) {
        i = in4_idx_1;
    } else {
        i = in1->size[1];
    }

    if (i == 1) {
        if (in3->size[1] == 1) {
            in4_idx_1 = in2_idx_1;
        } else {
            in4_idx_1 = in3->size[1];
        }
    } else if (in1->size[1] != 1) {
        in4_idx_1 = in1->size[1];
    }

    for (i = 0; i < 3; i++) {
        int aux_0_1;
        int aux_1_1;
        int aux_2_1;
        int aux_3_1;
        aux_0_1 = 0;
        aux_1_1 = 0;
        aux_2_1 = 0;
        aux_3_1 = 0;
        for (i1 = 0; i1 < in4_idx_1; i1++) {
            int i3;
            i2 = in1->size[0];
            in2_idx_1 = in3->size[0];
            if (i2 == 1) {
                i3 = in4_idx_0;
            } else {
                i3 = i2;
            }

            if (i3 == 1) {
                if (in2_idx_1 == 1) {
                    in2_idx_1 = in2_idx_0;
                }
            } else if (i2 == 1) {
                in2_idx_1 = in4_idx_0;
            } else {
                in2_idx_1 = i2;
            }

            for (i2 = 0; i2 < in2_idx_1; i2++) {
                b_in2_data[(i2 + b_in2->size[0] * i1) + b_in2->size[0] * b_in2->size[1] *
                                                            i] = in2_data[i2 * stride_0_0 + in2_idx_0 * aux_0_1] * in3_data[(i2 *
                                                                                                                                 stride_1_0 +
                                                                                                                             in3->size[0] * aux_1_1) +
                                                                                                                            in3->size[0] * in3->size[1] * i] +
                                                                 in4_data[i2 * stride_2_0 + in4_idx_0 * aux_2_1] * in1_data[(i2 *
                                                                                                                                 stride_3_0 +
                                                                                                                             in1->size[0] * aux_3_1) +
                                                                                                                            in1->size[0] * in1->size[1] * i];
            }

            aux_3_1 += stride_3_1;
            aux_2_1 += stride_2_1;
            aux_1_1 += stride_1_1;
            aux_0_1 += stride_0_1;
        }
    }

    i = in1->size[0] * in1->size[1] * in1->size[2];
    in1->size[0] = b_in2->size[0];
    in1->size[1] = b_in2->size[1];
    in1->size[2] = 3;
    emxEnsureCapacity_real_T(in1, i);
    in1_data = in1->data;
    in4_idx_1 = b_in2->size[1];
    for (i = 0; i < 3; i++) {
        for (i1 = 0; i1 < in4_idx_1; i1++) {
            in2_idx_1 = b_in2->size[0];
            for (i2 = 0; i2 < in2_idx_1; i2++) {
                in1_data[(i2 + in1->size[0] * i1) + in1->size[0] * in1->size[1] * i] =
                    b_in2_data[(i2 + b_in2->size[0] * i1) + b_in2->size[0] * b_in2->size[1] * i];
            }
        }
    }

    emxFree_real_T(&b_in2);
}

/**
 * @fn             : birdsEyeView_transformImage
 * @brief          :
 * @param          : const double this_OutputView[4]
 *                   const double this_Sensor_Intrinsics_K[9]
 *                   double this_Sensor_Height
 *                   double this_Sensor_Pitch
 *                   double this_Sensor_Yaw
 *                   double this_Sensor_Roll
 *                   const double this_Sensor_SensorLocation[2]
 *                   const double c_this_OutputViewImref_ImageSiz[2]
 *                   const double this_Scale[2]
 *                   const double b_I[18483444]
 *                   emxArray_real_T *birdsEyeViewImage
 * @return         : void
 */
static void birdsEyeView_transformImage(const double this_OutputView[4], const double this_Sensor_Intrinsics_K[9], double this_Sensor_Height, double this_Sensor_Pitch, double this_Sensor_Yaw, double this_Sensor_Roll, const double this_Sensor_SensorLocation[2], const double c_this_OutputViewImref_ImageSiz[2], const double this_Scale[2], const double b_I[18483444], emxArray_real_T *birdsEyeViewImage) {
    double A_in[9];
    double b_t1_T[9];
    double t1_T[9];
    double absx;
    double d;
    double d1;
    int exponent;
    int i;
    int k;
    bool guard1 = false;
    bool y;
    c_monoCamera_get_ImageToVehicle(this_Sensor_Intrinsics_K, this_Sensor_Height,
                                    this_Sensor_Pitch, this_Sensor_Yaw, this_Sensor_Roll,
                                    this_Sensor_SensorLocation, t1_T);
    for (k = 0; k < 3; k++) {
        absx = t1_T[k];
        d = t1_T[k + 3];
        d1 = t1_T[k + 6];
        for (i = 0; i < 3; i++) {
            b_t1_T[k + 3 * i] = (absx * (double)iv[3 * i] + d * (double)iv[3 * i + 1]) + d1 * (double)iv[3 * i + 2];
        }
    }

    t1_T[0] = this_Scale[0];
    t1_T[3] = 0.0;
    t1_T[6] = 0.0;
    t1_T[1] = 0.0;
    t1_T[4] = this_Scale[1];
    t1_T[7] = 0.0;
    t1_T[2] = this_Scale[0] * this_OutputView[3] + 1.0;
    t1_T[5] = this_Scale[1] * this_OutputView[1] + 1.0;
    t1_T[8] = 1.0;
    for (k = 0; k < 3; k++) {
        absx = t1_T[3 * k];
        d = t1_T[3 * k + 1];
        d1 = t1_T[3 * k + 2];
        for (i = 0; i < 3; i++) {
            A_in[k + 3 * i] = (b_t1_T[i] * absx + b_t1_T[i + 3] * d) + b_t1_T[i + 6] *
                                                                           d1;
        }
    }

    guard1 = false;
    if (A_in[8] == 1.0) {
        bool x[2];
        bool exitg1;
        x[0] = (A_in[2] == 0.0);
        x[1] = (A_in[5] == 0.0);
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

        if (!y) {
            guard1 = true;
        }
    } else {
        guard1 = true;
    }

    if (guard1) {
        double s[3];
        y = true;
        for (k = 0; k < 9; k++) {
            if (y) {
                absx = A_in[k];
                if (rtIsInf(absx) || rtIsNaN(absx)) {
                    y = false;
                }
            } else {
                y = false;
            }
        }

        if (y) {
            svd(A_in, s);
        } else {
            s[0] = rtNaN;
        }

        absx = fabs(s[0]);
        if ((!rtIsInf(absx)) && (!rtIsNaN(absx)) && (!(absx < 4.4501477170144028E-308))) {
            frexp(absx, &exponent);
        }
    }

    remapAndResampleGeneric2d(b_I, A_in, c_this_OutputViewImref_ImageSiz,
                              birdsEyeViewImage);
}

/**
 * @fn             : blendImage
 * @brief          :
 * @param          : struct_T *b_bigImgSt
 *                   const emxArray_real_T *currImg
 *                   const imref2d currRef
 *                   const emxArray_boolean_T *maskImg
 * @return         : void
 */
static void blendImage(struct_T *b_bigImgSt, const emxArray_real_T *currImg,
                       const imref2d currRef, const emxArray_boolean_T *maskImg) {
    emxArray_real_T *c_bigImgSt;
    emxArray_real_T *mergeImg;
    const double *currImg_data;
    double *bigImgSt_data;
    int i;
    currImg_data = currImg->data;

    /*  由当前的视角图像构建大图 */
    if (!alphablend_not_empty) {
        alphablend.isInitialized = 0;

        /* System object Constructor function: vision.AlphaBlender */
        alphablend.matlabCodegenIsDeleted = false;
        alphablend_not_empty = true;
    }

    if ((b_bigImgSt->bigImg->size[0] == 0) || (b_bigImgSt->bigImg->size[1] == 0)) {
        int loop_ub;
        i = b_bigImgSt->bigImg->size[0] * b_bigImgSt->bigImg->size[1] *
            b_bigImgSt->bigImg->size[2];
        b_bigImgSt->bigImg->size[0] = currImg->size[0];
        b_bigImgSt->bigImg->size[1] = currImg->size[1];
        b_bigImgSt->bigImg->size[2] = 3;
        emxEnsureCapacity_real_T(b_bigImgSt->bigImg, i);
        loop_ub = currImg->size[0] * currImg->size[1] * 3;
        for (i = 0; i < loop_ub; i++) {
            b_bigImgSt->bigImg->data[i] = currImg_data[i];
        }

        b_bigImgSt->ref = currRef;
    } else {
        double x[2];
        int loop_ub;
        x[0] = fmax(b_bigImgSt->ref.YWorldLimits[0] - currRef.YWorldLimits[0], 0.0);
        x[1] = fmax(b_bigImgSt->ref.XWorldLimits[0] - currRef.XWorldLimits[0], 0.0);
        x[0] = rt_roundd_snf(x[0]);
        x[1] = rt_roundd_snf(x[1]);
        emxInit_real_T(&c_bigImgSt, 3);
        i = c_bigImgSt->size[0] * c_bigImgSt->size[1] * c_bigImgSt->size[2];
        c_bigImgSt->size[0] = b_bigImgSt->bigImg->size[0];
        c_bigImgSt->size[1] = b_bigImgSt->bigImg->size[1];
        c_bigImgSt->size[2] = 3;
        emxEnsureCapacity_real_T(c_bigImgSt, i);
        bigImgSt_data = c_bigImgSt->data;
        loop_ub = b_bigImgSt->bigImg->size[0] * b_bigImgSt->bigImg->size[1] *
                      b_bigImgSt->bigImg->size[2] -
                  1;
        for (i = 0; i <= loop_ub; i++) {
            bigImgSt_data[i] = b_bigImgSt->bigImg->data[i];
        }

        padarray(c_bigImgSt, x, b_bigImgSt->bigImg);
        x[0] = fmax(currRef.YWorldLimits[1] - b_bigImgSt->ref.YWorldLimits[1], 0.0);
        x[1] = fmax(currRef.XWorldLimits[1] - b_bigImgSt->ref.XWorldLimits[1], 0.0);
        x[0] = rt_roundd_snf(x[0]);
        x[1] = rt_roundd_snf(x[1]);
        i = c_bigImgSt->size[0] * c_bigImgSt->size[1] * c_bigImgSt->size[2];
        c_bigImgSt->size[0] = b_bigImgSt->bigImg->size[0];
        c_bigImgSt->size[1] = b_bigImgSt->bigImg->size[1];
        c_bigImgSt->size[2] = 3;
        emxEnsureCapacity_real_T(c_bigImgSt, i);
        bigImgSt_data = c_bigImgSt->data;
        loop_ub = b_bigImgSt->bigImg->size[0] * b_bigImgSt->bigImg->size[1] *
                      b_bigImgSt->bigImg->size[2] -
                  1;
        for (i = 0; i <= loop_ub; i++) {
            bigImgSt_data[i] = b_bigImgSt->bigImg->data[i];
        }

        double d_bigImgSt;
        unsigned int imageSizeIn_idx_1;
        b_padarray(c_bigImgSt, x, b_bigImgSt->bigImg);
        emxFree_real_T(&c_bigImgSt);

        /*  construct new spatial refrence information */
        imageSizeIn_idx_1 = (unsigned int)b_bigImgSt->bigImg->size[1];
        b_bigImgSt->ref.ImageSizeAlias[0] = b_bigImgSt->bigImg->size[0];
        b_bigImgSt->ref.ImageSizeAlias[1] = imageSizeIn_idx_1;
        d_bigImgSt = b_bigImgSt->ref.XWorldLimits[1];
        b_bigImgSt->ref.XWorldLimits[0] = fmin(currRef.XWorldLimits[0],
                                               b_bigImgSt->ref.XWorldLimits[0]);
        b_bigImgSt->ref.XWorldLimits[1] = fmax(currRef.XWorldLimits[1], d_bigImgSt);
        d_bigImgSt = b_bigImgSt->ref.YWorldLimits[1];
        b_bigImgSt->ref.YWorldLimits[0] = fmin(currRef.YWorldLimits[0],
                                               b_bigImgSt->ref.YWorldLimits[0]);
        b_bigImgSt->ref.YWorldLimits[1] = fmax(currRef.YWorldLimits[1], d_bigImgSt);
        b_bigImgSt->ref.ForcePixelExtentToOne = false;

        /*  贴图 */
        /*  mask = sum(currImg, 3) ~= 0; */
        /*  maskImg = maskImg&mask; */
        /*  注意这里界限;涉及到大数组，可能编译失败，需要动态指定分配大小，见topic:Set Dynamic Memory Allocation Threshold */
        if (alphablend.isInitialized != 1) {
            alphablend.isInitialized = 1;
        }

        x[0] = (currRef.XWorldLimits[0] - b_bigImgSt->ref.XWorldLimits[0]) /
                   ((b_bigImgSt->ref.XWorldLimits[1] - b_bigImgSt->ref.XWorldLimits[0]) /
                    b_bigImgSt->ref.ImageSizeAlias[1]) +
               0.5;
        x[1] = (currRef.YWorldLimits[0] - b_bigImgSt->ref.YWorldLimits[0]) /
                   ((b_bigImgSt->ref.YWorldLimits[1] - b_bigImgSt->ref.YWorldLimits[0]) /
                    b_bigImgSt->ref.ImageSizeAlias[0]) +
               0.5;
        emxInit_real_T(&mergeImg, 3);
        Outputs(b_bigImgSt->bigImg, currImg, maskImg, x, mergeImg);
        bigImgSt_data = mergeImg->data;
        i = b_bigImgSt->bigImg->size[0] * b_bigImgSt->bigImg->size[1] *
            b_bigImgSt->bigImg->size[2];
        b_bigImgSt->bigImg->size[0] = mergeImg->size[0];
        b_bigImgSt->bigImg->size[1] = mergeImg->size[1];
        b_bigImgSt->bigImg->size[2] = mergeImg->size[2];
        emxEnsureCapacity_real_T(b_bigImgSt->bigImg, i);
        loop_ub = mergeImg->size[0] * mergeImg->size[1] * mergeImg->size[2];
        for (i = 0; i < loop_ub; i++) {
            b_bigImgSt->bigImg->data[i] = bigImgSt_data[i];
        }

        emxFree_real_T(&mergeImg);
    }
}

/**
 * @fn             : blendImage_free
 * @brief          :
 * @param          : void
 * @return         : void
 */
static void blendImage_free(void) {
    if (!alphablend.matlabCodegenIsDeleted) {
        alphablend.matlabCodegenIsDeleted = true;
        if (alphablend.isInitialized == 1) {
            alphablend.isInitialized = 2;
        }
    }
}

/**
 * @fn             : blendImage_init
 * @brief          :
 * @param          : void
 * @return         : void
 */
static void blendImage_init(void) {
    alphablend_not_empty = false;
    alphablend.matlabCodegenIsDeleted = true;
}

/**
 * @fn             : bwdist
 * @brief          :
 * @param          : const emxArray_boolean_T *varargin_1
 *                   emxArray_real32_T *varargout_1
 * @return         : void
 */
static void bwdist(const emxArray_boolean_T *varargin_1, emxArray_real32_T
                                                             *varargout_1) {
    emxArray_boolean_T *b_BW;
    emxArray_int32_T *linearIndexSerial;
    emxArray_real32_T *D1;
    emxArray_real32_T *dVector;
    emxArray_real32_T *gSerial;
    emxArray_real32_T *hSerial;
    float *D1_data;
    float *gSerial_data;
    float *varargout_1_data;
    int b_inputSize_idx_0;
    int b_k;
    int b_loop_ub;
    int c_inputSize_idx_0;
    int i;
    int i1;
    int i2;
    unsigned int inputSize_idx_0;
    unsigned int inputSize_idx_1;
    int k;
    int loop_ub;
    int nrows;
    int nx;
    int *linearIndexSerial_data;
    const bool *varargin_1_data;
    bool *BW_data;
    varargin_1_data = varargin_1->data;
    emxInit_boolean_T(&b_BW, 2);
    i = b_BW->size[0] * b_BW->size[1];
    b_BW->size[0] = varargin_1->size[0];
    b_BW->size[1] = varargin_1->size[1];
    emxEnsureCapacity_boolean_T(b_BW, i);
    BW_data = b_BW->data;
    nx = varargin_1->size[0] * varargin_1->size[1];
    for (i = 0; i < nx; i++) {
        BW_data[i] = varargin_1_data[i];
    }

    i = varargout_1->size[0] * varargout_1->size[1];
    varargout_1->size[0] = b_BW->size[0];
    varargout_1->size[1] = b_BW->size[1];
    emxEnsureCapacity_real32_T(varargout_1, i);
    varargout_1_data = varargout_1->data;
    inputSize_idx_0 = (unsigned int)b_BW->size[0];
    inputSize_idx_1 = (unsigned int)b_BW->size[1];
    if (b_BW->size[0] * b_BW->size[1] == 0) {
        i = varargout_1->size[0] * varargout_1->size[1];
        varargout_1->size[0] = b_BW->size[0];
        varargout_1->size[1] = b_BW->size[1];
        emxEnsureCapacity_real32_T(varargout_1, i);
        varargout_1_data = varargout_1->data;
        nx = b_BW->size[0] * b_BW->size[1];
        for (i = 0; i < nx; i++) {
            varargout_1_data[i] = BW_data[i];
        }
    } else {
        emxInit_real32_T(&gSerial, 1);
        emxInit_real32_T(&hSerial, 1);
        emxInit_real32_T(&D1, 1);
        if ((b_BW->size[0] == 1) && (b_BW->size[1] == 1)) {
            i = D1->size[0];
            D1->size[0] = 1;
            emxEnsureCapacity_real32_T(D1, i);
            D1_data = D1->data;
            if (BW_data[0]) {
                D1_data[0] = 0.0F;
            } else {
                D1_data[0] = -1.0F;
            }

            i = gSerial->size[0];
            gSerial->size[0] = 1;
            emxEnsureCapacity_real32_T(gSerial, i);
            gSerial_data = gSerial->data;
            gSerial_data[0] = 0.0F;
            i = hSerial->size[0];
            hSerial->size[0] = 1;
            emxEnsureCapacity_real32_T(hSerial, i);
            gSerial_data = hSerial->data;
            gSerial_data[0] = 0.0F;
            voronoiEDT(gSerial, hSerial, D1);
            D1_data = D1->data;
            varargout_1_data[0] = D1_data[0];
        } else {
            nx = b_BW->size[1];
            if (b_BW->size[1] - 1 >= 0) {
                nrows = b_BW->size[0];
                i1 = b_BW->size[0];
                b_inputSize_idx_0 = b_BW->size[0];
                c_inputSize_idx_0 = b_BW->size[0];
                loop_ub = b_BW->size[0];
                b_loop_ub = b_BW->size[0];
                i2 = b_BW->size[0];
            }

            for (k = 0; k < nx; k++) {
                i = D1->size[0];
                D1->size[0] = (int)inputSize_idx_0;
                emxEnsureCapacity_real32_T(D1, i);
                D1_data = D1->data;
                for (b_k = 0; b_k < i1; b_k++) {
                    if (BW_data[(int)((((double)k + 1.0) - 1.0) * (double)nrows + ((double)
                                                                                       b_k +
                                                                                   1.0)) -
                                1]) {
                        D1_data[b_k] = 0.0F;
                    } else {
                        D1_data[b_k] = -1.0F;
                    }
                }

                i = gSerial->size[0];
                gSerial->size[0] = b_inputSize_idx_0;
                emxEnsureCapacity_real32_T(gSerial, i);
                gSerial_data = gSerial->data;
                for (i = 0; i < loop_ub; i++) {
                    gSerial_data[i] = 0.0F;
                }

                i = hSerial->size[0];
                hSerial->size[0] = c_inputSize_idx_0;
                emxEnsureCapacity_real32_T(hSerial, i);
                gSerial_data = hSerial->data;
                for (i = 0; i < b_loop_ub; i++) {
                    gSerial_data[i] = 0.0F;
                }

                voronoiEDT(gSerial, hSerial, D1);
                D1_data = D1->data;
                for (b_k = 0; b_k < i2; b_k++) {
                    varargout_1_data[(int)((((double)k + 1.0) - 1.0) * (double)nrows +
                                           ((double)b_k + 1.0)) -
                                     1] = D1_data[b_k];
                }
            }

            emxInit_int32_T(&linearIndexSerial, 1);
            i = linearIndexSerial->size[0];
            linearIndexSerial->size[0] = b_BW->size[1];
            emxEnsureCapacity_int32_T(linearIndexSerial, i);
            linearIndexSerial_data = linearIndexSerial->data;
            i = D1->size[0];
            D1->size[0] = b_BW->size[1];
            emxEnsureCapacity_real32_T(D1, i);
            D1_data = D1->data;
            i = gSerial->size[0];
            gSerial->size[0] = b_BW->size[1];
            emxEnsureCapacity_real32_T(gSerial, i);
            gSerial_data = gSerial->data;
            for (i = 0; i < nx; i++) {
                gSerial_data[i] = 0.0F;
            }

            i = hSerial->size[0];
            hSerial->size[0] = b_BW->size[1];
            emxEnsureCapacity_real32_T(hSerial, i);
            gSerial_data = hSerial->data;
            for (i = 0; i < nx; i++) {
                gSerial_data[i] = 0.0F;
            }

            i = b_BW->size[0];
            emxInit_real32_T(&dVector, 1);
            for (k = 0; k < i; k++) {
                i1 = linearIndexSerial->size[0];
                for (b_k = 0; b_k < i1; b_k++) {
                    linearIndexSerial_data[b_k] = (int)((((double)b_k + 1.0) - 1.0) *
                                                            (double)inputSize_idx_0 +
                                                        ((double)k + 1.0));
                }

                loop_ub = (int)inputSize_idx_1;
                i1 = dVector->size[0];
                dVector->size[0] = (int)inputSize_idx_1;
                emxEnsureCapacity_real32_T(dVector, i1);
                gSerial_data = dVector->data;
                for (i1 = 0; i1 < loop_ub; i1++) {
                    gSerial_data[i1] = D1_data[i1];
                }

                for (b_k = 0; b_k < nx; b_k++) {
                    gSerial_data[b_k] = varargout_1_data[linearIndexSerial_data[b_k] - 1];
                }

                voronoiEDT(gSerial, hSerial, dVector);
                gSerial_data = dVector->data;
                for (b_k = 0; b_k < nx; b_k++) {
                    varargout_1_data[linearIndexSerial_data[b_k] - 1] = gSerial_data[b_k];
                }
            }

            emxFree_real32_T(&dVector);
            emxFree_int32_T(&linearIndexSerial);
        }

        emxFree_real32_T(&D1);
        emxFree_real32_T(&hSerial);
        emxFree_real32_T(&gSerial);
        if (varargout_1_data[0] == -1.0F) {
            i = b_BW->size[0] * b_BW->size[1];
            for (k = 0; k < i; k++) {
                varargout_1_data[k] = rtInfF;
            }
        }
    }

    emxFree_boolean_T(&b_BW);
    nx = varargout_1->size[0] * varargout_1->size[1];
    for (k = 0; k < nx; k++) {
        varargout_1_data[k] = sqrtf(varargout_1_data[k]);
    }
}

/**
 * @fn             : c_binary_expand_op
 * @brief          :
 * @param          : emxArray_boolean_T *in1
 *                   const emxArray_int32_T *in2
 *                   const emxArray_uint32_T *in3
 * @return         : void
 */
static void c_binary_expand_op(emxArray_boolean_T *in1, const emxArray_int32_T *in2, const emxArray_uint32_T *in3) {
    const int *in2_data;
    const unsigned int *in3_data;
    int i;
    int loop_ub;
    int stride_0_1;
    int stride_1_1;
    bool *in1_data;
    in3_data = in3->data;
    in2_data = in2->data;
    i = in1->size[0] * in1->size[1];
    in1->size[0] = 1;
    if (in3->size[1] == 1) {
        in1->size[1] = in2->size[1];
    } else {
        in1->size[1] = in3->size[1];
    }

    emxEnsureCapacity_boolean_T(in1, i);
    in1_data = in1->data;
    stride_0_1 = (in2->size[1] != 1);
    stride_1_1 = (in3->size[1] != 1);
    if (in3->size[1] == 1) {
        loop_ub = in2->size[1];
    } else {
        loop_ub = in3->size[1];
    }

    for (i = 0; i < loop_ub; i++) {
        in1_data[i] = ((unsigned int)in2_data[i * stride_0_1] == in3_data[2 * (i *
                                                                               stride_1_1)]);
    }
}

/**
 * @fn             : c_birdsEyeView_get_ImageToVehic
 * @brief          :
 * @param          : const double this_OutputView[4]
 *                   const double this_Sensor_Intrinsics_K[9]
 *                   double this_Sensor_Height
 *                   double this_Sensor_Pitch
 *                   double this_Sensor_Yaw
 *                   double this_Sensor_Roll
 *                   const double this_Sensor_SensorLocation[2]
 *                   const double this_Scale[2]
 *                   double tform_T[9]
 * @return         : void
 */
static void c_birdsEyeView_get_ImageToVehic(const double this_OutputView[4],
                                            const double this_Sensor_Intrinsics_K[9], double this_Sensor_Height, double this_Sensor_Pitch, double this_Sensor_Yaw, double this_Sensor_Roll, const double this_Sensor_SensorLocation[2], const double this_Scale[2], double tform_T[9]) {
    double b_t0_T[9];
    double b_this_Scale[9];
    double t0_T[9];
    double d;
    double d1;
    double d2;
    int i;
    int i1;
    c_monoCamera_get_ImageToVehicle(this_Sensor_Intrinsics_K, this_Sensor_Height,
                                    this_Sensor_Pitch, this_Sensor_Yaw, this_Sensor_Roll,
                                    this_Sensor_SensorLocation, t0_T);
    b_this_Scale[0] = this_Scale[0];
    b_this_Scale[3] = 0.0;
    b_this_Scale[6] = 0.0;
    b_this_Scale[1] = 0.0;
    b_this_Scale[4] = this_Scale[1];
    b_this_Scale[7] = 0.0;
    b_this_Scale[2] = this_Scale[0] * this_OutputView[3] + 1.0;
    b_this_Scale[5] = this_Scale[1] * this_OutputView[1] + 1.0;
    b_this_Scale[8] = 1.0;
    for (i = 0; i < 3; i++) {
        d = t0_T[i];
        d1 = t0_T[i + 3];
        d2 = t0_T[i + 6];
        for (i1 = 0; i1 < 3; i1++) {
            b_t0_T[i + 3 * i1] = (d * (double)iv[3 * i1] + d1 * (double)iv[3 * i1 + 1]) + d2 * (double)iv[3 * i1 + 2];
        }

        d = b_t0_T[i];
        d1 = b_t0_T[i + 3];
        d2 = b_t0_T[i + 6];
        for (i1 = 0; i1 < 3; i1++) {
            t0_T[i + 3 * i1] = (d * b_this_Scale[3 * i1] + d1 * b_this_Scale[3 * i1 +
                                                                             1]) +
                               d2 * b_this_Scale[3 * i1 + 2];
        }
    }

    inv(t0_T, b_t0_T);
    c_monoCamera_get_ImageToVehicle(this_Sensor_Intrinsics_K, this_Sensor_Height,
                                    this_Sensor_Pitch, this_Sensor_Yaw, this_Sensor_Roll,
                                    this_Sensor_SensorLocation, t0_T);
    for (i = 0; i < 3; i++) {
        d = b_t0_T[i];
        d1 = b_t0_T[i + 3];
        d2 = b_t0_T[i + 6];
        for (i1 = 0; i1 < 3; i1++) {
            tform_T[i + 3 * i1] = (d * t0_T[3 * i1] + d1 * t0_T[3 * i1 + 1]) + d2 *
                                                                                   t0_T[3 * i1 + 2];
        }
    }
}

/**
 * @fn             : c_bsearch
 * @brief          :
 * @param          : const emxArray_real32_T *x
 *                   double xi
 * @return         : int
 */
static int c_bsearch(const emxArray_real32_T *x, double xi) {
    const float *x_data;
    int high_i;
    int low_ip1;
    int n;
    x_data = x->data;
    high_i = x->size[1];
    n = 1;
    low_ip1 = 2;
    while (high_i > low_ip1) {
        int mid_i;
        mid_i = (n >> 1) + (high_i >> 1);
        if (((n & 1) == 1) && ((high_i & 1) == 1)) {
            mid_i++;
        }

        if (xi >= x_data[mid_i - 1]) {
            n = mid_i;
            low_ip1 = mid_i + 1;
        } else {
            high_i = mid_i;
        }
    }

    return n;
}

/**
 * @fn             : c_computeEdgesWithThinningPorta
 * @brief          :
 * @param          : const emxArray_real32_T *b
 *                   emxArray_real32_T *bx
 *                   emxArray_real32_T *by
 *                   double cutoff
 *                   emxArray_boolean_T *e
 * @return         : void
 */
static void c_computeEdgesWithThinningPorta(const emxArray_real32_T *b,
                                            emxArray_real32_T *bx, emxArray_real32_T *by, double cutoff,
                                            emxArray_boolean_T *e) {
    emxArray_real32_T *x;
    const float *b_data;
    float f;
    float f1;
    float *bx_data;
    float *by_data;
    float *x_data;
    int c;
    int mIndexInt;
    int n;
    int nx;
    int r;
    int ub_loop;
    unsigned int unnamed_idx_0;
    unsigned int unnamed_idx_1;
    bool b1;
    bool b2;
    bool b3;
    bool b4;
    bool *e_data;
    e_data = e->data;
    by_data = by->data;
    bx_data = bx->data;
    b_data = b->data;
    emxInit_real32_T(&x, 2);
    n = x->size[0] * x->size[1];
    x->size[0] = bx->size[0];
    x->size[1] = bx->size[1];
    emxEnsureCapacity_real32_T(x, n);
    x_data = x->data;
    nx = bx->size[0] * bx->size[1];
    for (n = 0; n < nx; n++) {
        x_data[n] = bx_data[n];
    }

    nx = bx->size[0] * bx->size[1];
    unnamed_idx_0 = (unsigned int)bx->size[0];
    unnamed_idx_1 = (unsigned int)bx->size[1];
    n = bx->size[0] * bx->size[1];
    bx->size[0] = (int)unnamed_idx_0;
    bx->size[1] = (int)unnamed_idx_1;
    emxEnsureCapacity_real32_T(bx, n);
    bx_data = bx->data;
    for (n = 0; n < nx; n++) {
        bx_data[n] = fabsf(x_data[n]);
    }

    n = x->size[0] * x->size[1];
    x->size[0] = by->size[0];
    x->size[1] = by->size[1];
    emxEnsureCapacity_real32_T(x, n);
    x_data = x->data;
    nx = by->size[0] * by->size[1];
    for (n = 0; n < nx; n++) {
        x_data[n] = by_data[n];
    }

    nx = by->size[0] * by->size[1];
    unnamed_idx_0 = (unsigned int)by->size[0];
    unnamed_idx_1 = (unsigned int)by->size[1];
    n = by->size[0] * by->size[1];
    by->size[0] = (int)unnamed_idx_0;
    by->size[1] = (int)unnamed_idx_1;
    emxEnsureCapacity_real32_T(by, n);
    by_data = by->data;
    for (n = 0; n < nx; n++) {
        by_data[n] = fabsf(x_data[n]);
    }

    emxFree_real32_T(&x);
    nx = e->size[0];
    n = e->size[1];
    mIndexInt = e->size[0];
    ub_loop = e->size[1] - 1;

#pragma omp parallel for num_threads(omp_get_max_threads()) private(b4, b3, b2, b1, r, f, f1)

    for (c = 0; c <= ub_loop; c++) {
        for (r = 0; r < mIndexInt; r++) {
            if ((r + 1 > nx) || (c < 1)) {
                b1 = true;
            } else {
                b1 = (b_data[r + b->size[0] * (c - 1)] <= b_data[r + b->size[0] * c]);
            }

            if ((r + 1 > nx) || (c + 2 > n)) {
                b2 = true;
            } else {
                b2 = (b_data[r + b->size[0] * c] > b_data[r + b->size[0] * (c + 1)]);
            }

            if ((c + 1 > n) || (r < 1)) {
                b3 = true;
            } else {
                b3 = (b_data[(r + b->size[0] * c) - 1] <= b_data[r + b->size[0] * c]);
            }

            if ((c + 1 > n) || (r + 2 > nx)) {
                b4 = true;
            } else {
                b4 = (b_data[r + b->size[0] * c] > b_data[(r + b->size[0] * c) + 1]);
            }

            f = bx_data[r + bx->size[0] * c];
            f1 = by_data[r + by->size[0] * c];
            e_data[r + e->size[0] * c] = ((b_data[r + b->size[0] * c] > cutoff) &&
                                          (((f >= f1 - 2.22044605E-14F) && b1 && b2) || ((f1 >= f -
                                                                                                    2.22044605E-14F) &&
                                                                                         b3 && b4)));
        }
    }
}

/**
 * @fn             : c_cosd
 * @brief          :
 * @param          : float *x
 * @return         : void
 */
static void c_cosd(float *x) {
    if (rtIsInfF(*x) || rtIsNaNF(*x)) {
        *x = rtNaNF;
    } else {
        float absx;
        signed char n;
        *x = rt_remf_snf(*x, 360.0F);
        absx = fabsf(*x);
        if (absx > 180.0F) {
            if (*x > 0.0F) {
                *x -= 360.0F;
            } else {
                *x += 360.0F;
            }

            absx = fabsf(*x);
        }

        if (absx <= 45.0F) {
            *x *= 0.0174532924F;
            n = 0;
        } else if (absx <= 135.0F) {
            if (*x > 0.0F) {
                *x = 0.0174532924F * (*x - 90.0F);
                n = 1;
            } else {
                *x = 0.0174532924F * (*x + 90.0F);
                n = -1;
            }
        } else if (*x > 0.0F) {
            *x = 0.0174532924F * (*x - 180.0F);
            n = 2;
        } else {
            *x = 0.0174532924F * (*x + 180.0F);
            n = -2;
        }

        if (n == 0) {
            *x = cosf(*x);
        } else if (n == 1) {
            *x = -sinf(*x);
        } else if (n == -1) {
            *x = sinf(*x);
        } else {
            *x = -cosf(*x);
        }
    }
}

/**
 * @fn             : c_eml_rand_mt19937ar_stateful_i
 * @brief          :
 * @param          : void
 * @return         : void
 */
static void c_eml_rand_mt19937ar_stateful_i(void) {
    int mti;
    unsigned int r;
    memset(&state[0], 0, 625U * sizeof(unsigned int));
    r = 5489U;
    state[0] = 5489U;
    for (mti = 0; mti < 623; mti++) {
        r = ((r ^ r >> 30U) * 1812433253U + (unsigned int)mti) + 1U;
        state[mti + 1] = r;
    }

    state[624] = 624U;
}

/**
 * @fn             : c_interp2_local
 * @brief          :
 * @param          : const emxArray_real_T *V
 *                   const emxArray_real_T *Xq
 *                   const emxArray_real_T *Yq
 *                   const emxArray_real_T *X
 *                   const emxArray_real_T *Y
 *                   emxArray_real_T *Vq
 * @return         : void
 */
static void c_interp2_local(const emxArray_real_T *V, const emxArray_real_T *Xq,
                            const emxArray_real_T *Yq, const emxArray_real_T *X, const emxArray_real_T *Y,
                            emxArray_real_T *Vq) {
    const double *V_data;
    const double *X_data;
    const double *Xq_data;
    const double *Y_data;
    const double *Yq_data;
    double qx1;
    double qx2;
    double rx;
    double zx1y1;
    double zx1y2;
    double *Vq_data;
    int ix;
    int iy;
    int k;
    int ub_loop;
    Y_data = Y->data;
    X_data = X->data;
    Yq_data = Yq->data;
    Xq_data = Xq->data;
    V_data = V->data;
    ub_loop = Vq->size[0] * Vq->size[1];
    Vq->size[0] = Xq->size[0];
    Vq->size[1] = Xq->size[1];
    emxEnsureCapacity_real_T(Vq, ub_loop);
    Vq_data = Vq->data;
    ub_loop = Xq->size[0] * Xq->size[1] - 1;

#pragma omp parallel for num_threads(omp_get_max_threads()) private(ix, iy, zx1y1, qx1, zx1y2, qx2, rx)

    for (k = 0; k <= ub_loop; k++) {
        if ((Xq_data[k] >= X_data[0]) && (Xq_data[k] <= X_data[X->size[1] - 1]) &&
            (Yq_data[k] >= Y_data[0]) && (Yq_data[k] <= Y_data[Y->size[1] - 1])) {
            ix = b_bsearch(X, Xq_data[k]);
            iy = b_bsearch(Y, Yq_data[k]);
            zx1y1 = V_data[(iy + V->size[0] * (ix - 1)) - 1];
            qx1 = V_data[(iy + V->size[0] * ix) - 1];
            zx1y2 = V_data[iy + V->size[0] * (ix - 1)];
            qx2 = V_data[iy + V->size[0] * ix];
            rx = X_data[ix - 1];
            if (Xq_data[k] == rx) {
                qx1 = zx1y1;
                qx2 = zx1y2;
            } else if (!(Xq_data[k] == X_data[ix])) {
                rx = (Xq_data[k] - rx) / (X_data[ix] - rx);
                if (zx1y1 == qx1) {
                    qx1 = zx1y1;
                } else {
                    qx1 = (1.0 - rx) * zx1y1 + rx * qx1;
                }

                if (zx1y2 == qx2) {
                    qx2 = zx1y2;
                } else {
                    qx2 = (1.0 - rx) * zx1y2 + rx * qx2;
                }
            }

            rx = Y_data[iy - 1];
            if ((Yq_data[k] == rx) || (qx1 == qx2)) {
                Vq_data[k] = qx1;
            } else if (Yq_data[k] == Y_data[iy]) {
                Vq_data[k] = qx2;
            } else {
                rx = (Yq_data[k] - rx) / (Y_data[iy] - rx);
                Vq_data[k] = (1.0 - rx) * qx1 + rx * qx2;
            }
        } else {
            Vq_data[k] = 0.0;
        }
    }
}

/**
 * @fn             : c_minimum
 * @brief          :
 * @param          : const emxArray_real32_T *x
 *                   emxArray_real32_T *ex
 *                   emxArray_int32_T *idx
 * @return         : void
 */
static void c_minimum(const emxArray_real32_T *x, emxArray_real32_T *ex,
                      emxArray_int32_T *idx) {
    const float *x_data;
    float *ex_data;
    int j;
    int loop_ub;
    int m;
    int n;
    int *idx_data;
    x_data = x->data;
    m = x->size[0] - 1;
    n = x->size[1];
    j = ex->size[0];
    ex->size[0] = x->size[0];
    emxEnsureCapacity_real32_T(ex, j);
    ex_data = ex->data;
    j = idx->size[0];
    idx->size[0] = x->size[0];
    emxEnsureCapacity_int32_T(idx, j);
    idx_data = idx->data;
    loop_ub = x->size[0];
    for (j = 0; j < loop_ub; j++) {
        idx_data[j] = 1;
    }

    if (x->size[0] >= 1) {
        for (loop_ub = 0; loop_ub <= m; loop_ub++) {
            ex_data[loop_ub] = x_data[loop_ub];
        }

        for (j = 2; j <= n; j++) {
            for (loop_ub = 0; loop_ub <= m; loop_ub++) {
                float b;
                bool p;
                b = x_data[loop_ub + x->size[0] * (j - 1)];
                if (rtIsNaNF(b)) {
                    p = false;
                } else if (rtIsNaNF(ex_data[loop_ub])) {
                    p = true;
                } else {
                    p = (ex_data[loop_ub] > b);
                }

                if (p) {
                    ex_data[loop_ub] = b;
                    idx_data[loop_ub] = j;
                }
            }
        }
    }
}

/**
 * @fn             : c_monoCamera_get_ImageToVehicle
 * @brief          :
 * @param          : const double this_Intrinsics_K[9]
 *                   double this_Height
 *                   double this_Pitch
 *                   double this_Yaw
 *                   double this_Roll
 *                   const double this_SensorLocation[2]
 *                   double tform_T[9]
 * @return         : void
 */
static void c_monoCamera_get_ImageToVehicle(const double this_Intrinsics_K[9],
                                            double this_Height, double this_Pitch, double this_Yaw, double this_Roll,
                                            const double this_SensorLocation[2], double tform_T[9]) {
    static const double b_a[9] = {-6.123233995736766E-17, -1.0,
                                  -7.498798913309288E-33, -1.0, 6.123233995736766E-17, -1.2246467991473532E-16,
                                  1.2246467991473532E-16, 0.0, -1.0};

    static const signed char b_iv[3] = {1, 0, 0};

    static const signed char iv1[3] = {0, 0, 1};

    double camMatrix[12];
    double d_a[12];
    double R_tmp[9];
    double a[9];
    double b_R_tmp[9];
    double c_R_tmp[9];
    double c_a[9];
    double b_this_SensorLocation[3];
    double c_this_SensorLocation[3];
    double R_tmp_tmp;
    double a_tmp;
    double b_R_tmp_tmp;
    double c_R_tmp_tmp;
    double d;
    double d1;
    double d2;
    double d_R_tmp_tmp;
    double e_R_tmp_tmp;
    int b_a_tmp;
    int i;
    int i1;
    a_tmp = 0.017453292519943295 * -this_Yaw;
    R_tmp_tmp = sin(a_tmp);
    b_R_tmp_tmp = cos(a_tmp);
    a_tmp = 0.017453292519943295 * (90.0 - this_Pitch);
    c_R_tmp_tmp = sin(a_tmp);
    d_R_tmp_tmp = cos(a_tmp);
    a_tmp = 0.017453292519943295 * this_Roll;
    e_R_tmp_tmp = sin(a_tmp);
    a_tmp = cos(a_tmp);
    R_tmp[0] = b_R_tmp_tmp;
    R_tmp[3] = -R_tmp_tmp;
    R_tmp[6] = 0.0;
    R_tmp[1] = R_tmp_tmp;
    R_tmp[4] = b_R_tmp_tmp;
    R_tmp[7] = 0.0;
    R_tmp[2] = 0.0;
    R_tmp[5] = 0.0;
    R_tmp[8] = 1.0;
    for (i = 0; i < 3; i++) {
        d = b_a[i];
        d1 = b_a[i + 3];
        d2 = b_a[i + 6];
        for (b_a_tmp = 0; b_a_tmp < 3; b_a_tmp++) {
            a[i + 3 * b_a_tmp] = (d * R_tmp[3 * b_a_tmp] + d1 * R_tmp[3 * b_a_tmp + 1]) + d2 * R_tmp[3 * b_a_tmp + 2];
        }

        b_R_tmp[3 * i] = b_iv[i];
    }

    b_R_tmp[1] = 0.0;
    b_R_tmp[4] = d_R_tmp_tmp;
    b_R_tmp[7] = -c_R_tmp_tmp;
    b_R_tmp[2] = 0.0;
    b_R_tmp[5] = c_R_tmp_tmp;
    b_R_tmp[8] = d_R_tmp_tmp;
    R_tmp[0] = a_tmp;
    R_tmp[3] = -e_R_tmp_tmp;
    R_tmp[6] = 0.0;
    R_tmp[1] = e_R_tmp_tmp;
    R_tmp[4] = a_tmp;
    R_tmp[7] = 0.0;
    for (i = 0; i < 3; i++) {
        d = a[i];
        d1 = a[i + 3];
        d2 = a[i + 6];
        for (b_a_tmp = 0; b_a_tmp < 3; b_a_tmp++) {
            c_a[i + 3 * b_a_tmp] = (d * b_R_tmp[3 * b_a_tmp] + d1 * b_R_tmp[3 *
                                                                                b_a_tmp +
                                                                            1]) +
                                   d2 * b_R_tmp[3 * b_a_tmp + 2];
        }

        R_tmp[3 * i + 2] = iv1[i];
    }

    a[0] = b_R_tmp_tmp;
    a[3] = -R_tmp_tmp;
    a[6] = 0.0;
    a[1] = R_tmp_tmp;
    a[4] = b_R_tmp_tmp;
    a[7] = 0.0;
    a[2] = 0.0;
    b_R_tmp[0] = 1.0;
    a[5] = 0.0;
    b_R_tmp[3] = 0.0;
    a[8] = 1.0;
    b_R_tmp[6] = 0.0;
    b_R_tmp[1] = 0.0;
    b_R_tmp[4] = d_R_tmp_tmp;
    b_R_tmp[7] = -c_R_tmp_tmp;
    b_R_tmp[2] = 0.0;
    b_R_tmp[5] = c_R_tmp_tmp;
    b_R_tmp[8] = d_R_tmp_tmp;
    for (i = 0; i < 3; i++) {
        d = a[i];
        d1 = a[i + 3];
        b_a_tmp = (int)a[i + 6];
        for (i1 = 0; i1 < 3; i1++) {
            c_R_tmp[i + 3 * i1] = (d * b_R_tmp[3 * i1] + d1 * b_R_tmp[3 * i1 + 1]) +
                                  (double)b_a_tmp * b_R_tmp[3 * i1 + 2];
        }
    }

    a[0] = a_tmp;
    a[3] = -e_R_tmp_tmp;
    a[6] = 0.0;
    a[1] = e_R_tmp_tmp;
    a[4] = a_tmp;
    a[7] = 0.0;
    a[2] = 0.0;
    a[5] = 0.0;
    a[8] = 1.0;
    b_this_SensorLocation[0] = this_SensorLocation[1];
    b_this_SensorLocation[1] = this_SensorLocation[0];
    b_this_SensorLocation[2] = this_Height;
    for (i = 0; i < 3; i++) {
        d = c_R_tmp[i];
        d1 = c_R_tmp[i + 3];
        d2 = c_R_tmp[i + 6];
        for (b_a_tmp = 0; b_a_tmp < 3; b_a_tmp++) {
            b_R_tmp[i + 3 * b_a_tmp] = (d * a[3 * b_a_tmp] + d1 * a[3 * b_a_tmp + 1]) + d2 * a[3 * b_a_tmp + 2];
        }
    }

    for (i = 0; i < 3; i++) {
        d = c_a[i];
        d1 = c_a[i + 3];
        d2 = c_a[i + 6];
        a_tmp = 0.0;
        for (b_a_tmp = 0; b_a_tmp < 3; b_a_tmp++) {
            a[i + 3 * b_a_tmp] = (d * R_tmp[3 * b_a_tmp] + d1 * R_tmp[3 * b_a_tmp + 1]) + d2 * R_tmp[3 * b_a_tmp + 2];
            a_tmp += b_this_SensorLocation[b_a_tmp] * b_R_tmp[b_a_tmp + 3 * i];
        }

        c_this_SensorLocation[i] = a_tmp;
    }

    for (i = 0; i < 3; i++) {
        b_a_tmp = i << 2;
        d_a[b_a_tmp] = a[3 * i];
        d_a[b_a_tmp + 1] = a[3 * i + 1];
        d_a[b_a_tmp + 2] = a[3 * i + 2];
        d_a[b_a_tmp + 3] = c_this_SensorLocation[i];
    }

    for (i = 0; i < 4; i++) {
        d = d_a[i];
        d1 = d_a[i + 4];
        d2 = d_a[i + 8];
        for (b_a_tmp = 0; b_a_tmp < 3; b_a_tmp++) {
            camMatrix[i + (b_a_tmp << 2)] = (d * this_Intrinsics_K[b_a_tmp] + d1 *
                                                                                  this_Intrinsics_K[b_a_tmp + 3]) +
                                            d2 * this_Intrinsics_K[b_a_tmp + 6];
        }
    }

    a[0] = camMatrix[0];
    a[1] = camMatrix[1];
    a[2] = camMatrix[3];
    a[3] = camMatrix[4];
    a[4] = camMatrix[5];
    a[5] = camMatrix[7];
    a[6] = camMatrix[8];
    a[7] = camMatrix[9];
    a[8] = camMatrix[11];
    inv(a, tform_T);
}

/**
 * @fn             : c_remapAndResampleGeneric2d
 * @brief          :
 * @param          : const emxArray_real_T *inputImage
 *                   float tform_RotationAngle
 *                   const float tform_Translation[2]
 *                   const imref2d outputRef
 *                   emxArray_real_T *outputImage
 * @return         : void
 */
static void c_remapAndResampleGeneric2d(const emxArray_real_T *inputImage, float tform_RotationAngle, const float tform_Translation[2], const imref2d outputRef,
                                        emxArray_real_T *outputImage) {
    emxArray_real_T *XIntrinsic;
    emxArray_real_T *YIntrinsic;
    emxArray_real_T *b_inputImage;
    emxArray_real_T *r;
    emxArray_real_T *srcXIntrinsic;
    emxArray_real_T *srcYIntrinsic;
    const double *inputImage_data;
    double dstXWorld_val;
    double dstYWorld_val;
    double extentY;
    double nRows;
    double *srcXIntrinsic_data;
    double *srcYIntrinsic_data;
    float b_r1[9];
    float tinv[9];
    float r1;
    float r2;
    float srcXWorld_val;
    float srcYWorld_val;
    int colIdx;
    int i;
    int i1;
    int i2;
    int plane;
    int rowIdx;
    int ub_loop;
    inputImage_data = inputImage->data;
    r1 = tform_RotationAngle;
    c_cosd(&r1);
    r2 = tform_RotationAngle;
    c_sind(&r2);
    b_r1[0] = r1;
    b_r1[1] = -r2;
    b_r1[2] = tform_Translation[0];
    b_r1[3] = r2;
    b_r1[4] = r1;
    b_r1[5] = tform_Translation[1];
    b_r1[6] = 0.0F;
    b_r1[7] = 0.0F;
    b_r1[8] = 1.0F;
    b_inv(b_r1, tinv);
    nRows = outputRef.ImageSizeAlias[0];
    emxInit_real_T(&srcXIntrinsic, 2);
    i = srcXIntrinsic->size[0] * srcXIntrinsic->size[1];
    srcXIntrinsic->size[0] = (int)outputRef.ImageSizeAlias[0];
    srcXIntrinsic->size[1] = (int)outputRef.ImageSizeAlias[1];
    emxEnsureCapacity_real_T(srcXIntrinsic, i);
    srcXIntrinsic_data = srcXIntrinsic->data;
    emxInit_real_T(&srcYIntrinsic, 2);
    i = srcYIntrinsic->size[0] * srcYIntrinsic->size[1];
    srcYIntrinsic->size[0] = (int)outputRef.ImageSizeAlias[0];
    srcYIntrinsic->size[1] = (int)outputRef.ImageSizeAlias[1];
    emxEnsureCapacity_real_T(srcYIntrinsic, i);
    srcYIntrinsic_data = srcYIntrinsic->data;
    ub_loop = (int)outputRef.ImageSizeAlias[1] - 1;

#pragma omp parallel for num_threads(omp_get_max_threads()) private(srcYWorld_val, srcXWorld_val, dstYWorld_val, dstXWorld_val, i1, extentY, rowIdx)

    for (colIdx = 0; colIdx <= ub_loop; colIdx++) {
        if (outputRef.ForcePixelExtentToOne) {
            dstYWorld_val = 1.0;
        } else {
            dstYWorld_val = (outputRef.XWorldLimits[1] - outputRef.XWorldLimits[0]) /
                            outputRef.ImageSizeAlias[1];
        }

        dstXWorld_val = outputRef.XWorldLimits[0] + (((double)colIdx + 1.0) - 0.5) *
                                                        dstYWorld_val;
        i1 = (int)nRows;
        if (outputRef.ForcePixelExtentToOne) {
            extentY = 1.0;
        } else {
            extentY = (outputRef.YWorldLimits[1] - outputRef.YWorldLimits[0]) /
                      outputRef.ImageSizeAlias[0];
        }

        for (rowIdx = 0; rowIdx < i1; rowIdx++) {
            dstYWorld_val = outputRef.YWorldLimits[0] + (((double)rowIdx + 1.0) - 0.5) * extentY;
            srcXWorld_val = (tinv[0] * (float)dstXWorld_val + tinv[1] * (float)
                                                                            dstYWorld_val) +
                            tinv[2];
            srcYWorld_val = (tinv[3] * (float)dstXWorld_val + tinv[4] * (float)
                                                                            dstYWorld_val) +
                            tinv[5];
            srcXIntrinsic_data[rowIdx + srcXIntrinsic->size[0] * colIdx] =
                (srcXWorld_val - 0.5F) + 0.5F;
            srcYIntrinsic_data[rowIdx + srcYIntrinsic->size[0] * colIdx] =
                (srcYWorld_val - 0.5F) + 0.5F;
        }
    }

    i = outputImage->size[0] * outputImage->size[1] * outputImage->size[2];
    outputImage->size[0] = srcXIntrinsic->size[0];
    outputImage->size[1] = srcXIntrinsic->size[1];
    outputImage->size[2] = 3;
    emxEnsureCapacity_real_T(outputImage, i);
    srcYIntrinsic_data = outputImage->data;
    ub_loop = srcXIntrinsic->size[0] * srcXIntrinsic->size[1] * 3;
    for (i = 0; i < ub_loop; i++) {
        srcYIntrinsic_data[i] = 0.0;
    }

    emxInit_real_T(&XIntrinsic, 2);
    if (inputImage->size[1] < 1) {
        XIntrinsic->size[0] = 1;
        XIntrinsic->size[1] = 0;
    } else {
        i = XIntrinsic->size[0] * XIntrinsic->size[1];
        XIntrinsic->size[0] = 1;
        XIntrinsic->size[1] = inputImage->size[1];
        emxEnsureCapacity_real_T(XIntrinsic, i);
        srcXIntrinsic_data = XIntrinsic->data;
        ub_loop = inputImage->size[1] - 1;
        for (i = 0; i <= ub_loop; i++) {
            srcXIntrinsic_data[i] = (double)i + 1.0;
        }
    }

    emxInit_real_T(&YIntrinsic, 2);
    if (inputImage->size[0] < 1) {
        YIntrinsic->size[0] = 1;
        YIntrinsic->size[1] = 0;
    } else {
        i = YIntrinsic->size[0] * YIntrinsic->size[1];
        YIntrinsic->size[0] = 1;
        YIntrinsic->size[1] = inputImage->size[0];
        emxEnsureCapacity_real_T(YIntrinsic, i);
        srcXIntrinsic_data = YIntrinsic->data;
        ub_loop = inputImage->size[0] - 1;
        for (i = 0; i <= ub_loop; i++) {
            srcXIntrinsic_data[i] = (double)i + 1.0;
        }
    }

    ub_loop = inputImage->size[1];
    emxInit_real_T(&b_inputImage, 2);
    emxInit_real_T(&r, 2);
    for (plane = 0; plane < 3; plane++) {
        int loop_ub;
        i = b_inputImage->size[0] * b_inputImage->size[1];
        b_inputImage->size[0] = inputImage->size[0];
        b_inputImage->size[1] = inputImage->size[1];
        emxEnsureCapacity_real_T(b_inputImage, i);
        srcXIntrinsic_data = b_inputImage->data;
        for (i = 0; i < ub_loop; i++) {
            loop_ub = inputImage->size[0];
            for (i2 = 0; i2 < loop_ub; i2++) {
                srcXIntrinsic_data[i2 + b_inputImage->size[0] * i] = inputImage_data[(i2 + inputImage->size[0] * i) + inputImage->size[0] * inputImage->size[1] * plane];
            }
        }

        interp2(XIntrinsic, YIntrinsic, b_inputImage, srcXIntrinsic, srcYIntrinsic,
                r);
        srcXIntrinsic_data = r->data;
        loop_ub = r->size[1];
        for (i = 0; i < loop_ub; i++) {
            int b_loop_ub;
            b_loop_ub = r->size[0];
            for (i2 = 0; i2 < b_loop_ub; i2++) {
                srcYIntrinsic_data[(i2 + outputImage->size[0] * i) + outputImage->size[0] * outputImage->size[1] * plane] = srcXIntrinsic_data[i2 + r->size[0] *
                                                                                                                                                        i];
            }
        }
    }

    emxFree_real_T(&r);
    emxFree_real_T(&b_inputImage);
    emxFree_real_T(&YIntrinsic);
    emxFree_real_T(&XIntrinsic);
    emxFree_real_T(&srcYIntrinsic);
    emxFree_real_T(&srcXIntrinsic);
}

/**
 * @fn             : c_sind
 * @brief          :
 * @param          : float *x
 * @return         : void
 */
static void c_sind(float *x) {
    if (rtIsInfF(*x) || rtIsNaNF(*x)) {
        *x = rtNaNF;
    } else {
        float absx;
        signed char n;
        *x = rt_remf_snf(*x, 360.0F);
        absx = fabsf(*x);
        if (absx > 180.0F) {
            if (*x > 0.0F) {
                *x -= 360.0F;
            } else {
                *x += 360.0F;
            }

            absx = fabsf(*x);
        }

        if (absx <= 45.0F) {
            *x *= 0.0174532924F;
            n = 0;
        } else if (absx <= 135.0F) {
            if (*x > 0.0F) {
                *x = 0.0174532924F * (*x - 90.0F);
                n = 1;
            } else {
                *x = 0.0174532924F * (*x + 90.0F);
                n = -1;
            }
        } else if (*x > 0.0F) {
            *x = 0.0174532924F * (*x - 180.0F);
            n = 2;
        } else {
            *x = 0.0174532924F * (*x + 180.0F);
            n = -2;
        }

        if (n == 0) {
            *x = sinf(*x);
        } else if (n == 1) {
            *x = cosf(*x);
        } else if (n == -1) {
            *x = -cosf(*x);
        } else {
            *x = -sinf(*x);
        }
    }
}

/**
 * @fn             : c_xnrm2
 * @brief          :
 * @param          : const double x[3]
 * @return         : double
 */
static double c_xnrm2(const double x[3]) {
    double scale;
    double y;
    int k;
    y = 0.0;
    scale = 3.3121686421112381E-170;
    for (k = 2; k < 4; k++) {
        double absxk;
        absxk = fabs(x[k - 1]);
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

    return scale * sqrt(y);
}

/**
 * @fn             : computeRigid2d
 * @brief          :
 * @param          : const emxArray_real32_T *points
 *                   float T[9]
 * @return         : void
 */
static void computeRigid2d(const emxArray_real32_T *points, float T[9]) {
    emxArray_real32_T *normPoints1;
    emxArray_real32_T *normPoints2;
    float C[4];
    float U[4];
    float V[4];
    float centroid1[2];
    float centroid2[2];
    const float *points_data;
    float bkj;
    float f;
    float f1;
    float f2;
    float f3;
    float f4;
    float *normPoints1_data;
    float *normPoints2_data;
    int acoef;
    int boffset;
    int coffset;
    int j;
    int k;
    signed char csz[2];
    bool p;
    points_data = points->data;
    emxInit_real32_T(&normPoints1, 2);
    boffset = normPoints1->size[0] * normPoints1->size[1];
    normPoints1->size[0] = points->size[0];
    normPoints1->size[1] = 2;
    emxEnsureCapacity_real32_T(normPoints1, boffset);
    normPoints1_data = normPoints1->data;
    acoef = points->size[0];
    for (boffset = 0; boffset < 2; boffset++) {
        for (coffset = 0; coffset < acoef; coffset++) {
            normPoints1_data[coffset + normPoints1->size[0] * boffset] =
                points_data[coffset + points->size[0] * boffset];
        }
    }

    b_mean(normPoints1, centroid1);
    boffset = normPoints1->size[0] * normPoints1->size[1];
    normPoints1->size[0] = points->size[0];
    normPoints1->size[1] = 2;
    emxEnsureCapacity_real32_T(normPoints1, boffset);
    normPoints1_data = normPoints1->data;
    acoef = points->size[0];
    for (boffset = 0; boffset < 2; boffset++) {
        for (coffset = 0; coffset < acoef; coffset++) {
            normPoints1_data[coffset + normPoints1->size[0] * boffset] = points_data
                [(coffset + points->size[0] * boffset) + points->size[0] * 2];
        }
    }

    b_mean(normPoints1, centroid2);
    boffset = normPoints1->size[0] * normPoints1->size[1];
    normPoints1->size[0] = points->size[0];
    normPoints1->size[1] = 2;
    emxEnsureCapacity_real32_T(normPoints1, boffset);
    normPoints1_data = normPoints1->data;
    if (points->size[0] != 0) {
        acoef = (points->size[0] != 1);
        boffset = normPoints1->size[0] - 1;
        for (k = 0; k < 2; k++) {
            for (coffset = 0; coffset <= boffset; coffset++) {
                normPoints1_data[coffset + normPoints1->size[0] * k] = points_data[acoef * coffset + points->size[0] * k] - centroid1[k];
            }
        }
    }

    emxInit_real32_T(&normPoints2, 2);
    boffset = normPoints2->size[0] * normPoints2->size[1];
    normPoints2->size[0] = points->size[0];
    normPoints2->size[1] = 2;
    emxEnsureCapacity_real32_T(normPoints2, boffset);
    normPoints2_data = normPoints2->data;
    if (points->size[0] != 0) {
        acoef = (points->size[0] != 1);
        boffset = normPoints2->size[0] - 1;
        for (k = 0; k < 2; k++) {
            for (coffset = 0; coffset <= boffset; coffset++) {
                normPoints2_data[coffset + normPoints2->size[0] * k] = points_data
                                                                           [(acoef * coffset + points->size[0] * k) + points->size[0] * 2] -
                                                                       centroid2[k];
            }
        }
    }

    acoef = normPoints1->size[0];
    for (j = 0; j < 2; j++) {
        coffset = j << 1;
        boffset = j * normPoints2->size[0];
        C[coffset] = 0.0F;
        C[coffset + 1] = 0.0F;
        for (k = 0; k < acoef; k++) {
            bkj = normPoints2_data[boffset + k];
            C[coffset] += normPoints1_data[k] * bkj;
            C[coffset + 1] += normPoints1_data[normPoints1->size[0] + k] * bkj;
        }
    }

    emxFree_real32_T(&normPoints2);
    emxFree_real32_T(&normPoints1);
    p = true;
    if (rtIsInfF(C[0]) || rtIsNaNF(C[0]) || (rtIsInfF(C[1]) || rtIsNaNF(C[1]))) {
        p = false;
    }

    if ((!p) || (rtIsInfF(C[2]) || rtIsNaNF(C[2]))) {
        p = false;
    }

    if ((!p) || (rtIsInfF(C[3]) || rtIsNaNF(C[3]))) {
        p = false;
    }

    if (p) {
        float s[2];
        b_svd(C, U, s, V);
    } else {
        U[0] = rtNaNF;
        V[0] = rtNaNF;
        U[1] = rtNaNF;
        V[1] = rtNaNF;
        U[2] = rtNaNF;
        V[2] = rtNaNF;
        U[3] = rtNaNF;
        V[3] = rtNaNF;
    }

    bkj = V[0];
    f = V[2];
    f1 = V[1];
    f2 = V[3];
    for (boffset = 0; boffset < 2; boffset++) {
        f3 = U[boffset + 2];
        f4 = U[boffset];
        C[boffset] = f4 * bkj + f3 * f;
        C[boffset + 2] = f4 * f1 + f3 * f2;
        csz[boffset] = (signed char)(boffset + 1);
    }

    acoef = 0;
    if (fabsf(C[1]) > fabsf(C[0])) {
        acoef = 1;
    }

    if (C[acoef] != 0.0F) {
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

    if (C[2] != 0.0F) {
        C[3] += C[1] * -C[2];
    }

    bkj = C[0] * C[3];
    if (csz[0] > 1) {
        bkj = -bkj;
    }

    if (rtIsNaNF(bkj)) {
        C[3] = rtNaNF;
    } else if (bkj < 0.0F) {
        C[3] = -1.0F;
    } else {
        C[3] = (float)(bkj > 0.0F);
    }

    bkj = U[0];
    f = U[2];
    f1 = U[1];
    f2 = U[3];
    for (boffset = 0; boffset < 2; boffset++) {
        float f5;
        f3 = V[boffset + 2];
        f4 = V[boffset];
        f5 = f4 + f3 * 0.0F;
        f3 = f4 * 0.0F + f3 * C[3];
        C[boffset] = f5 * bkj + f3 * f;
        C[boffset + 2] = f5 * f1 + f3 * f2;
    }

    for (boffset = 0; boffset < 9; boffset++) {
        T[boffset] = 0.0F;
    }

    T[8] = 1.0F;
    T[0] = C[0];
    T[1] = C[2];
    T[2] = centroid2[0] - (C[0] * centroid1[0] + centroid1[1] * C[2]);
    T[3] = C[1];
    T[4] = C[3];
    T[5] = centroid2[1] - (centroid1[0] * C[1] + centroid1[1] * C[3]);
}

/**
 * @fn             : constrainToRotationMatrix2D
 * @brief          :
 * @param          : const float R[4]
 *                   float Rc[4]
 *                   float *r
 * @return         : void
 */
static void constrainToRotationMatrix2D(const float R[4], float Rc[4], float *r) {
    float e[2];
    float s[2];
    float R_clamped_idx_0;
    float R_clamped_idx_1;
    float R_clamped_idx_2;
    float R_clamped_idx_3;
    float b;
    float d;
    float f;
    float minval_idx_3;
    float sm;
    float snorm;
    float sqds;
    float wpr;
    int kase;
    bool close_enough;
    R_clamped_idx_0 = fmaxf(fminf(R[0], 1.0F), -1.0F);
    R_clamped_idx_1 = fmaxf(fminf(R[1], 1.0F), -1.0F);
    R_clamped_idx_2 = fmaxf(fminf(R[2], 1.0F), -1.0F);
    R_clamped_idx_3 = fmaxf(fminf(R[3], 1.0F), -1.0F);
    wpr = 57.2957802F * rt_atan2f_snf(R_clamped_idx_1, R_clamped_idx_3) + 180.0F;
    if (rtIsNaNF(wpr) || rtIsInfF(wpr)) {
        *r = rtNaNF;
    } else if (wpr == 0.0F) {
        *r = 0.0F;
    } else {
        *r = fmodf(wpr, 360.0F);
        if (*r == 0.0F) {
            *r = 0.0F;
        } else if (wpr < 0.0F) {
            *r += 360.0F;
        }
    }

    wpr = roundf(*r - 180.0F);
    if (*r - 180.0F == wpr) {
        close_enough = true;
    } else {
        d = fabsf((*r - 180.0F) - wpr);
        if ((*r - 180.0F == 0.0F) || (wpr == 0.0F)) {
            close_enough = (d < 1.4013E-45F);
        } else {
            b = fabsf(*r - 180.0F) + fabsf(wpr);
            if (b < 1.17549435E-38F) {
                close_enough = (d < 1.4013E-45F);
            } else {
                close_enough = (d / fminf(b, 3.402823466E+38F) < 1.1920929E-7F);
            }
        }
    }

    *r -= 180.0F;
    if (close_enough) {
        *r = wpr;
    }

    wpr = *r;
    c_sind(&wpr);
    d = *r;
    c_cosd(&d);
    Rc[0] = d;
    Rc[2] = -wpr;
    Rc[1] = wpr;
    Rc[3] = d;
    f = R_clamped_idx_0 - d;
    sqds = R_clamped_idx_1 - wpr;
    snorm = R_clamped_idx_2 - (-wpr);
    minval_idx_3 = R_clamped_idx_3 - d;
    wpr = 0.0F;
    b = fabsf(f);
    if (rtIsNaNF(b) || (b > 0.0F)) {
        wpr = b;
    }

    sm = fabsf(sqds);
    if (rtIsNaNF(sm) || (sm > wpr)) {
        wpr = sm;
    }

    d = fabsf(snorm);
    if (rtIsNaNF(d) || (d > wpr)) {
        wpr = d;
    }

    d = fabsf(minval_idx_3);
    if (rtIsNaNF(d) || (d > wpr)) {
        wpr = d;
    }

    if ((!rtIsInfF(wpr)) && (!rtIsNaNF(wpr))) {
        float scale;
        int iter;
        int m;
        scale = 1.29246971E-26F;
        if (b > 1.29246971E-26F) {
            d = 1.0F;
            scale = b;
        } else {
            wpr = b / 1.29246971E-26F;
            d = wpr * wpr;
        }

        if (sm > scale) {
            wpr = scale / sm;
            d = d * wpr * wpr + 1.0F;
            scale = sm;
        } else {
            wpr = sm / scale;
            d += wpr * wpr;
        }

        d = scale * sqrtf(d);
        if (d > 0.0F) {
            if (f < 0.0F) {
                s[0] = -d;
            } else {
                s[0] = d;
            }

            if (fabsf(s[0]) >= 9.86076132E-32F) {
                wpr = 1.0F / s[0];
                f *= wpr;
                sqds *= wpr;
            } else {
                f /= s[0];
                sqds /= s[0];
            }

            f++;
            s[0] = -s[0];
            wpr = (f * snorm + sqds * minval_idx_3) / f;
            if (!(-wpr == 0.0F)) {
                snorm += -wpr * f;
                minval_idx_3 += -wpr * sqds;
            }
        } else {
            s[0] = 0.0F;
        }

        m = 2;
        s[1] = minval_idx_3;
        e[0] = snorm;
        e[1] = 0.0F;
        if (s[0] != 0.0F) {
            d = fabsf(s[0]);
            wpr = s[0] / d;
            s[0] = d;
            e[0] = snorm / wpr;
        }

        if (e[0] != 0.0F) {
            d = fabsf(e[0]);
            wpr = e[0];
            e[0] = d;
            s[1] = minval_idx_3 * (d / wpr);
        }

        if (s[1] != 0.0F) {
            s[1] = fabsf(s[1]);
        }

        iter = 0;
        snorm = fmaxf(fmaxf(s[0], e[0]), fmaxf(s[1], 0.0F));
        while ((m > 0) && (iter < 75)) {
            int ii_tmp_tmp;
            int q;
            bool exitg1;
            ii_tmp_tmp = m - 1;
            q = m - 1;
            exitg1 = false;
            while (!(exitg1 || (q == 0))) {
                wpr = fabsf(e[0]);
                if ((wpr <= 1.1920929E-7F * (fabsf(s[0]) + fabsf(s[1]))) || (wpr <= 9.86076132E-32F) || ((iter > 20) && (wpr <= 1.1920929E-7F * snorm))) {
                    e[0] = 0.0F;
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
                        wpr = 0.0F;
                        if (kase < m) {
                            wpr = fabsf(e[0]);
                        }

                        if (kase > q + 1) {
                            wpr += fabsf(e[0]);
                        }

                        d = fabsf(s[kase - 1]);
                        if ((d <= 1.1920929E-7F * wpr) || (d <= 9.86076132E-32F)) {
                            s[kase - 1] = 0.0F;
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
                    e[0] = 0.0F;
                    for (kase = ii_tmp_tmp; kase >= q + 1; kase--) {
                        xrotg(&s[0], &f, &d, &sm);
                    }
                    break;

                case 2:
                    f = e[q - 1];
                    e[q - 1] = 0.0F;
                    for (kase = q + 1; kase <= m; kase++) {
                        xrotg(&s[kase - 1], &f, &d, &sm);
                        wpr = e[kase - 1];
                        f = -sm * wpr;
                        e[kase - 1] = wpr * d;
                    }
                    break;

                case 3:
                    wpr = s[m - 1];
                    scale = fmaxf(fmaxf(fmaxf(fmaxf(fabsf(wpr), fabsf(s[0])), fabsf(e[0])),
                                        fabsf(s[q])),
                                  fabsf(e[q]));
                    sm = wpr / scale;
                    wpr = s[0] / scale;
                    d = e[0] / scale;
                    sqds = s[q] / scale;
                    b = ((wpr + sm) * (wpr - sm) + d * d) / 2.0F;
                    wpr = sm * d;
                    wpr *= wpr;
                    if ((b != 0.0F) || (wpr != 0.0F)) {
                        d = sqrtf(b * b + wpr);
                        if (b < 0.0F) {
                            d = -d;
                        }

                        d = wpr / (b + d);
                    } else {
                        d = 0.0F;
                    }

                    f = (sqds + sm) * (sqds - sm) + d;
                    wpr = sqds * (e[q] / scale);
                    for (kase = q + 1; kase < 2; kase++) {
                        xrotg(&f, &wpr, &d, &sm);
                        f = d * s[0] + sm * e[0];
                        b = d * e[0] - sm * s[0];
                        e[0] = b;
                        wpr = sm * s[1];
                        s[1] *= d;
                        s[0] = f;
                        xrotg(&s[0], &wpr, &d, &sm);
                        f = d * b + sm * s[1];
                        s[1] = -sm * b + d * s[1];
                        wpr = sm * e[1];
                        e[1] *= d;
                    }

                    e[0] = f;
                    iter++;
                    break;

                default:
                    if (s[q] < 0.0F) {
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

    if (wpr / 1.1920929E-7F < 10.0F) {
        Rc[0] = R_clamped_idx_0;
        Rc[1] = R_clamped_idx_1;
        Rc[2] = R_clamped_idx_2;
        Rc[3] = R_clamped_idx_3;
    }
}

/**
 * @fn             : constructWorldMap_free
 * @brief          :
 * @param          : void
 * @return         : void
 */
static void constructWorldMap_free(void) {
    emxFree_real_T(&previousImage);
    emxFreeStruct_struct_T(&bigImgSt);
    emxFree_real_T(&vehicleShowPts);
    emxFree_boolean_T(&BW);
}

/**
 * @fn             : constructWorldMap_init
 * @brief          :
 * @param          : void
 * @return         : void
 */
static void constructWorldMap_init(void) {
    emxInit_boolean_T(&BW, 2);
    emxInit_real_T(&vehicleShowPts, 2);
    emxInitStruct_struct_T(&bigImgSt);
    emxInit_real_T(&previousImage, 3);
}

/**
 * @fn             : contrib
 * @brief          :
 * @param          : double x1
 *                   double b_y1
 *                   double x2
 *                   double y2
 *                   signed char quad1
 *                   signed char quad2
 *                   double scale
 *                   signed char *diffQuad
 *                   bool *onj
 * @return         : void
 */
static void contrib(double x1, double b_y1, double x2, double y2, signed char quad1, signed char quad2, double scale, signed char *diffQuad, bool *onj) {
    double cp;
    *onj = false;
    *diffQuad = (signed char)(quad2 - quad1);
    cp = x1 * y2 - x2 * b_y1;
    if (fabs(cp) < scale) {
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

/**
 * @fn             : conv2AXPYValidCMP
 * @brief          :
 * @param          : const emxArray_real32_T *a
 *                   emxArray_real32_T *c
 * @return         : void
 */
static void conv2AXPYValidCMP(const emxArray_real32_T *a, emxArray_real32_T *c) {
    static const double b[9] = {-0.125, -0.25, -0.125, 0.0, 0.0, 0.0, 0.125, 0.25,
                                0.125};

    emxArray_real32_T *cj;
    double bij;
    const float *a_data;
    float *c_data;
    float *cj_data;
    int b_i;
    int i;
    int ib;
    int j;
    int jb;
    int mc;
    int ub_loop;
    a_data = a->data;
    mc = a->size[0] - 2;
    if ((a->size[0] - 2 == 0) || (a->size[1] - 2 == 0)) {
        i = c->size[0] * c->size[1];
        c->size[0] = a->size[0] - 2;
        c->size[1] = a->size[1] - 2;
        emxEnsureCapacity_real32_T(c, i);
        c_data = c->data;
        ub_loop = (a->size[0] - 2) * (a->size[1] - 2);
        for (i = 0; i < ub_loop; i++) {
            c_data[i] = 0.0F;
        }
    } else {
        i = c->size[0] * c->size[1];
        c->size[0] = a->size[0] - 2;
        c->size[1] = a->size[1] - 2;
        emxEnsureCapacity_real32_T(c, i);
        c_data = c->data;
        ub_loop = a->size[1] - 3;

#pragma omp parallel \
num_threads(omp_get_max_threads()) private(cj_data, cj, bij, ib, jb, b_i)

        {
            emxInit_real32_T(&cj, 1);

#pragma omp for nowait

            for (j = 0; j <= ub_loop; j++) {
                ib = cj->size[0];
                cj->size[0] = mc;
                emxEnsureCapacity_real32_T(cj, ib);
                cj_data = cj->data;
                for (ib = 0; ib < mc; ib++) {
                    cj_data[ib] = 0.0F;
                }

                for (jb = 0; jb < 3; jb++) {
                    for (ib = 0; ib < 3; ib++) {
                        bij = b[(3 * (2 - jb) - ib) + 2];
                        for (b_i = 0; b_i < mc; b_i++) {
                            cj_data[b_i] += (float)bij * a_data[(b_i + ib) + a->size[0] * (j +
                                                                                           jb)];
                        }
                    }
                }

                jb = cj->size[0];
                for (ib = 0; ib < jb; ib++) {
                    c_data[ib + c->size[0] * j] = cj_data[ib];
                }
            }

            emxFree_real32_T(&cj);
        }
    }
}

/**
 * @fn             : d_binary_expand_op
 * @brief          :
 * @param          : emxArray_real32_T *in1
 *                   const emxArray_real32_T *in2
 *                   const emxArray_real32_T *in3
 * @return         : void
 */
static void d_binary_expand_op(emxArray_real32_T *in1, const emxArray_real32_T *in2, const emxArray_real32_T *in3) {
    emxArray_real32_T *b_in1;
    const float *in2_data;
    const float *in3_data;
    float *b_in1_data;
    float *in1_data;
    int i;
    int i1;
    int in2_idx_0;
    int stride_0_0;
    int stride_1_0;
    in3_data = in3->data;
    in2_data = in2->data;
    in2_idx_0 = in2->size[0];
    i = in1->size[0] * in1->size[1];
    in1->size[0] = in2_idx_0;
    in1->size[1] = 2;
    emxEnsureCapacity_real32_T(in1, i);
    in1_data = in1->data;
    for (i = 0; i < in2_idx_0; i++) {
        in1_data[i] = in2_data[i] / in2_data[i + in2->size[0] * 2];
    }

    in2_idx_0 = in2->size[0];
    for (i = 0; i < in2_idx_0; i++) {
        in1_data[i + in1->size[0]] = in2_data[i + in2->size[0]] / in2_data[i +
                                                                           in2->size[0] * 2];
    }

    emxInit_real32_T(&b_in1, 2);
    i = b_in1->size[0] * b_in1->size[1];
    if (in3->size[0] == 1) {
        b_in1->size[0] = in1->size[0];
    } else {
        b_in1->size[0] = in3->size[0];
    }

    b_in1->size[1] = 2;
    emxEnsureCapacity_real32_T(b_in1, i);
    b_in1_data = b_in1->data;
    stride_0_0 = (in1->size[0] != 1);
    stride_1_0 = (in3->size[0] != 1);
    if (in3->size[0] == 1) {
        in2_idx_0 = in1->size[0];
    } else {
        in2_idx_0 = in3->size[0];
    }

    for (i = 0; i < 2; i++) {
        for (i1 = 0; i1 < in2_idx_0; i1++) {
            b_in1_data[i1 + b_in1->size[0] * i] = in1_data[i1 * stride_0_0 + in1->size[0] * i] - in3_data[(i1 * stride_1_0 + in3->size[0] * i) + in3->size[0] * 2];
        }
    }

    i = in1->size[0] * in1->size[1];
    in1->size[0] = b_in1->size[0];
    in1->size[1] = 2;
    emxEnsureCapacity_real32_T(in1, i);
    in1_data = in1->data;
    in2_idx_0 = b_in1->size[0];
    for (i = 0; i < 2; i++) {
        for (i1 = 0; i1 < in2_idx_0; i1++) {
            in1_data[i1 + in1->size[0] * i] = b_in1_data[i1 + b_in1->size[0] * i];
        }
    }

    emxFree_real32_T(&b_in1);
}

/**
 * @fn             : d_remapAndResampleGeneric2d
 * @brief          :
 * @param          : const emxArray_boolean_T *inputImage
 *                   float tform_RotationAngle
 *                   const float tform_Translation[2]
 *                   const imref2d outputRef
 *                   emxArray_boolean_T *outputImage
 * @return         : void
 */
static void d_remapAndResampleGeneric2d(const emxArray_boolean_T *inputImage,
                                        float tform_RotationAngle, const float tform_Translation[2], const imref2d outputRef, emxArray_boolean_T *outputImage) {
    emxArray_int8_T *b_inputImage;
    emxArray_real32_T *XIntrinsic;
    emxArray_real32_T *YIntrinsic;
    emxArray_real32_T *b_outputImage;
    emxArray_real32_T *c_inputImage;
    emxArray_real32_T *r;
    emxArray_real_T *srcXIntrinsic;
    emxArray_real_T *srcYIntrinsic;
    emxArray_uint32_T *y;
    double dstXWorld_val;
    double dstYWorld_val;
    double extentY;
    double nRows;
    double *srcXIntrinsic_data;
    double *srcYIntrinsic_data;
    float b_r1[9];
    float tinv[9];
    float r1;
    float r2;
    float srcXWorld_val;
    float srcYWorld_val;
    float *XIntrinsic_data;
    float *outputImage_data;
    int colIdx;
    int i;
    int i1;
    int i2;
    int loop_ub;
    int rowIdx;
    int ub_loop;
    unsigned int *y_data;
    signed char *b_inputImage_data;
    const bool *inputImage_data;
    bool *b_outputImage_data;
    inputImage_data = inputImage->data;
    r1 = tform_RotationAngle;
    c_cosd(&r1);
    r2 = tform_RotationAngle;
    c_sind(&r2);
    b_r1[0] = r1;
    b_r1[1] = -r2;
    b_r1[2] = tform_Translation[0];
    b_r1[3] = r2;
    b_r1[4] = r1;
    b_r1[5] = tform_Translation[1];
    b_r1[6] = 0.0F;
    b_r1[7] = 0.0F;
    b_r1[8] = 1.0F;
    b_inv(b_r1, tinv);
    nRows = outputRef.ImageSizeAlias[0];
    emxInit_real_T(&srcXIntrinsic, 2);
    i = srcXIntrinsic->size[0] * srcXIntrinsic->size[1];
    srcXIntrinsic->size[0] = (int)outputRef.ImageSizeAlias[0];
    srcXIntrinsic->size[1] = (int)outputRef.ImageSizeAlias[1];
    emxEnsureCapacity_real_T(srcXIntrinsic, i);
    srcXIntrinsic_data = srcXIntrinsic->data;
    emxInit_real_T(&srcYIntrinsic, 2);
    i = srcYIntrinsic->size[0] * srcYIntrinsic->size[1];
    srcYIntrinsic->size[0] = (int)outputRef.ImageSizeAlias[0];
    srcYIntrinsic->size[1] = (int)outputRef.ImageSizeAlias[1];
    emxEnsureCapacity_real_T(srcYIntrinsic, i);
    srcYIntrinsic_data = srcYIntrinsic->data;
    ub_loop = (int)outputRef.ImageSizeAlias[1] - 1;

#pragma omp parallel for num_threads(omp_get_max_threads()) private(srcYWorld_val, srcXWorld_val, dstYWorld_val, dstXWorld_val, i1, extentY, rowIdx)

    for (colIdx = 0; colIdx <= ub_loop; colIdx++) {
        if (outputRef.ForcePixelExtentToOne) {
            dstYWorld_val = 1.0;
        } else {
            dstYWorld_val = (outputRef.XWorldLimits[1] - outputRef.XWorldLimits[0]) /
                            outputRef.ImageSizeAlias[1];
        }

        dstXWorld_val = outputRef.XWorldLimits[0] + (((double)colIdx + 1.0) - 0.5) *
                                                        dstYWorld_val;
        i1 = (int)nRows;
        if (outputRef.ForcePixelExtentToOne) {
            extentY = 1.0;
        } else {
            extentY = (outputRef.YWorldLimits[1] - outputRef.YWorldLimits[0]) /
                      outputRef.ImageSizeAlias[0];
        }

        for (rowIdx = 0; rowIdx < i1; rowIdx++) {
            dstYWorld_val = outputRef.YWorldLimits[0] + (((double)rowIdx + 1.0) - 0.5) * extentY;
            srcXWorld_val = (tinv[0] * (float)dstXWorld_val + tinv[1] * (float)
                                                                            dstYWorld_val) +
                            tinv[2];
            srcYWorld_val = (tinv[3] * (float)dstXWorld_val + tinv[4] * (float)
                                                                            dstYWorld_val) +
                            tinv[5];
            srcXIntrinsic_data[rowIdx + srcXIntrinsic->size[0] * colIdx] =
                (srcXWorld_val - 0.5F) + 0.5F;
            srcYIntrinsic_data[rowIdx + srcYIntrinsic->size[0] * colIdx] =
                (srcYWorld_val - 0.5F) + 0.5F;
        }
    }

    emxInit_int8_T(&b_inputImage);
    i = b_inputImage->size[0] * b_inputImage->size[1];
    b_inputImage->size[0] = inputImage->size[0];
    b_inputImage->size[1] = inputImage->size[1];
    emxEnsureCapacity_int8_T(b_inputImage, i);
    b_inputImage_data = b_inputImage->data;
    ub_loop = inputImage->size[0] * inputImage->size[1];
    for (i = 0; i < ub_loop; i++) {
        b_inputImage_data[i] = (signed char)inputImage_data[i];
    }

    emxInit_real32_T(&b_outputImage, 2);
    i = b_outputImage->size[0] * b_outputImage->size[1];
    b_outputImage->size[0] = srcXIntrinsic->size[0];
    b_outputImage->size[1] = srcXIntrinsic->size[1];
    emxEnsureCapacity_real32_T(b_outputImage, i);
    outputImage_data = b_outputImage->data;
    ub_loop = srcXIntrinsic->size[0] * srcXIntrinsic->size[1];
    for (i = 0; i < ub_loop; i++) {
        outputImage_data[i] = 0.0F;
    }

    emxInit_uint32_T(&y);
    y_data = y->data;
    if (b_inputImage->size[1] < 1) {
        y->size[0] = 1;
        y->size[1] = 0;
    } else {
        i = y->size[0] * y->size[1];
        y->size[0] = 1;
        y->size[1] = b_inputImage->size[1];
        emxEnsureCapacity_uint32_T(y, i);
        y_data = y->data;
        ub_loop = b_inputImage->size[1] - 1;
        for (i = 0; i <= ub_loop; i++) {
            y_data[i] = (unsigned int)i + 1U;
        }
    }

    emxInit_real32_T(&XIntrinsic, 2);
    i = XIntrinsic->size[0] * XIntrinsic->size[1];
    XIntrinsic->size[0] = 1;
    XIntrinsic->size[1] = y->size[1];
    emxEnsureCapacity_real32_T(XIntrinsic, i);
    XIntrinsic_data = XIntrinsic->data;
    ub_loop = y->size[1];
    for (i = 0; i < ub_loop; i++) {
        XIntrinsic_data[i] = (float)y_data[i];
    }

    if (b_inputImage->size[0] < 1) {
        y->size[0] = 1;
        y->size[1] = 0;
    } else {
        i = y->size[0] * y->size[1];
        y->size[0] = 1;
        y->size[1] = b_inputImage->size[0];
        emxEnsureCapacity_uint32_T(y, i);
        y_data = y->data;
        ub_loop = b_inputImage->size[0] - 1;
        for (i = 0; i <= ub_loop; i++) {
            y_data[i] = (unsigned int)i + 1U;
        }
    }

    emxInit_real32_T(&YIntrinsic, 2);
    i = YIntrinsic->size[0] * YIntrinsic->size[1];
    YIntrinsic->size[0] = 1;
    YIntrinsic->size[1] = y->size[1];
    emxEnsureCapacity_real32_T(YIntrinsic, i);
    XIntrinsic_data = YIntrinsic->data;
    ub_loop = y->size[1];
    for (i = 0; i < ub_loop; i++) {
        XIntrinsic_data[i] = (float)y_data[i];
    }

    emxFree_uint32_T(&y);
    emxInit_real32_T(&c_inputImage, 2);
    i = c_inputImage->size[0] * c_inputImage->size[1];
    c_inputImage->size[0] = b_inputImage->size[0];
    c_inputImage->size[1] = b_inputImage->size[1];
    emxEnsureCapacity_real32_T(c_inputImage, i);
    XIntrinsic_data = c_inputImage->data;
    ub_loop = b_inputImage->size[1];
    for (i = 0; i < ub_loop; i++) {
        loop_ub = b_inputImage->size[0];
        for (i2 = 0; i2 < loop_ub; i2++) {
            XIntrinsic_data[i2 + c_inputImage->size[0] * i] = b_inputImage_data[i2 +
                                                                                b_inputImage->size[0] * i];
        }
    }

    emxFree_int8_T(&b_inputImage);
    emxInit_real32_T(&r, 2);
    interp2_local(c_inputImage, srcXIntrinsic, srcYIntrinsic, XIntrinsic,
                  YIntrinsic, r);
    XIntrinsic_data = r->data;
    emxFree_real32_T(&c_inputImage);
    emxFree_real32_T(&YIntrinsic);
    emxFree_real32_T(&XIntrinsic);
    emxFree_real_T(&srcYIntrinsic);
    emxFree_real_T(&srcXIntrinsic);
    ub_loop = r->size[1];
    for (i = 0; i < ub_loop; i++) {
        loop_ub = r->size[0];
        for (i2 = 0; i2 < loop_ub; i2++) {
            outputImage_data[i2 + b_outputImage->size[0] * i] = XIntrinsic_data[i2 +
                                                                                r->size[0] * i];
        }
    }

    emxFree_real32_T(&r);
    i = outputImage->size[0] * outputImage->size[1];
    outputImage->size[0] = b_outputImage->size[0];
    outputImage->size[1] = b_outputImage->size[1];
    emxEnsureCapacity_boolean_T(outputImage, i);
    b_outputImage_data = outputImage->data;
    ub_loop = b_outputImage->size[0] * b_outputImage->size[1];
    for (i = 0; i < ub_loop; i++) {
        b_outputImage_data[i] = (outputImage_data[i] > 0.5F);
    }

    emxFree_real32_T(&b_outputImage);
}

/**
 * @fn             : detectORBFeatures
 * @brief          :
 * @param          : const emxArray_real_T *b_I
 *                   ORBPoints *points
 * @return         : void
 */
static void detectORBFeatures(const emxArray_real_T *b_I, ORBPoints *points) {
    void *pKeypoints;
    emxArray_real32_T *location;
    emxArray_real32_T *metric;
    emxArray_real32_T *orientation;
    emxArray_real32_T *scale;
    emxArray_uint8_T *Iu8;
    float *location_data;
    float *metric_data;
    float *orientation_data;
    float *scale_data;
    int i;
    int numPtsOut;
    unsigned char MaxNumLevels;
    unsigned char *Iu8_data;
    if (b_I->size[0] > b_I->size[1]) {
        i = b_I->size[1];
    } else {
        i = b_I->size[0];
    }

    MaxNumLevels = (unsigned char)(floor((log(i) - 4.1431347263915326) /
                                         0.18232155679395459) +
                                   1.0);
    if (MaxNumLevels >= 4) {
        MaxNumLevels = 4U;
    }

    emxInit_uint8_T(&Iu8);
    uint8PortableCodeAlgo(b_I, Iu8);
    Iu8_data = Iu8->data;
    pKeypoints = NULL;
    numPtsOut = detectORBComputeCM(&Iu8_data[0], Iu8->size[0], Iu8->size[1], (int)((double)Iu8->size[0] * (double)Iu8->size[1]), 1.2F, MaxNumLevels, 31, 0, 2,
                                   0, 31, 20, &pKeypoints);
    emxFree_uint8_T(&Iu8);
    emxInit_real32_T(&location, 2);
    i = location->size[0] * location->size[1];
    location->size[0] = numPtsOut;
    location->size[1] = 2;
    emxEnsureCapacity_real32_T(location, i);
    location_data = location->data;
    emxInit_real32_T(&metric, 1);
    i = metric->size[0];
    metric->size[0] = numPtsOut;
    emxEnsureCapacity_real32_T(metric, i);
    metric_data = metric->data;
    emxInit_real32_T(&scale, 1);
    i = scale->size[0];
    scale->size[0] = numPtsOut;
    emxEnsureCapacity_real32_T(scale, i);
    scale_data = scale->data;
    emxInit_real32_T(&orientation, 1);
    i = orientation->size[0];
    orientation->size[0] = numPtsOut;
    emxEnsureCapacity_real32_T(orientation, i);
    orientation_data = orientation->data;
    detectORBAssignOutputCM(pKeypoints, &location_data[0], &orientation_data[0],
                            &metric_data[0], &scale_data[0]);
    numPtsOut = scale->size[0];
    for (i = 0; i < numPtsOut; i++) {
        scale_data[i] /= 31.0F;
    }

    points->pLocation->size[0] = 0;
    points->pLocation->size[1] = 2;
    points->pMetric->size[0] = 0;
    points->pScale->size[0] = 0;
    points->pOrientation->size[0] = 0;
    b_ORBPointsImpl_configure(points, location, metric, scale, orientation);
    emxFree_real32_T(&orientation);
    emxFree_real32_T(&scale);
    emxFree_real32_T(&metric);
    emxFree_real32_T(&location);
}

/**
 * @fn             : edge
 * @brief          :
 * @param          : const emxArray_boolean_T *varargin_1
 *                   emxArray_boolean_T *varargout_1
 * @return         : void
 */
static void edge(const emxArray_boolean_T *varargin_1, emxArray_boolean_T
                                                           *varargout_1) {
    emxArray_int32_T *idxA;
    emxArray_real32_T *a;
    emxArray_real32_T *b_varargin_1;
    emxArray_real32_T *bx;
    emxArray_real32_T *by;
    emxArray_uint32_T *idxDir;
    emxArray_uint32_T *y;
    float *a_data;
    float *b_varargin_1_data;
    float *by_data;
    int firstBlockLength;
    int hi;
    int ib;
    int k;
    int lastBlockLength;
    int nblocks;
    int *idxA_data;
    unsigned int *idxDir_data;
    unsigned int *y_data;
    const bool *varargin_1_data;
    bool *varargout_1_data;
    varargin_1_data = varargin_1->data;
    emxInit_real32_T(&a, 2);
    lastBlockLength = a->size[0] * a->size[1];
    a->size[0] = varargin_1->size[0];
    a->size[1] = varargin_1->size[1];
    emxEnsureCapacity_real32_T(a, lastBlockLength);
    a_data = a->data;
    firstBlockLength = varargin_1->size[0] * varargin_1->size[1];
    for (lastBlockLength = 0; lastBlockLength < firstBlockLength; lastBlockLength++) {
        a_data[lastBlockLength] = varargin_1_data[lastBlockLength];
    }

    if ((a->size[0] == 0) || (a->size[1] == 0)) {
        lastBlockLength = varargout_1->size[0] * varargout_1->size[1];
        varargout_1->size[0] = a->size[0];
        varargout_1->size[1] = a->size[1];
        emxEnsureCapacity_boolean_T(varargout_1, lastBlockLength);
        varargout_1_data = varargout_1->data;
        firstBlockLength = a->size[0] * a->size[1];
        for (lastBlockLength = 0; lastBlockLength < firstBlockLength;
             lastBlockLength++) {
            varargout_1_data[lastBlockLength] = false;
        }
    } else {
        double b_y;
        int m;
        int n;
        n = a->size[1];
        m = a->size[0];
        emxInit_real32_T(&b_varargin_1, 2);
        lastBlockLength = b_varargin_1->size[0] * b_varargin_1->size[1];
        b_varargin_1->size[0] = a->size[0];
        b_varargin_1->size[1] = a->size[1];
        emxEnsureCapacity_real32_T(b_varargin_1, lastBlockLength);
        b_varargin_1_data = b_varargin_1->data;
        firstBlockLength = a->size[0] * a->size[1];
        for (lastBlockLength = 0; lastBlockLength < firstBlockLength;
             lastBlockLength++) {
            b_varargin_1_data[lastBlockLength] = a_data[lastBlockLength];
        }

        emxInit_int32_T(&idxA, 2);
        lastBlockLength = idxA->size[0] * idxA->size[1];
        if ((unsigned int)a->size[0] < (unsigned int)a->size[1]) {
            idxA->size[0] = (int)((unsigned int)a->size[1] + 2U);
        } else {
            idxA->size[0] = (int)((unsigned int)a->size[0] + 2U);
        }

        idxA->size[1] = 2;
        emxEnsureCapacity_int32_T(idxA, lastBlockLength);
        idxA_data = idxA->data;
        emxInit_uint32_T(&y);
        lastBlockLength = y->size[0] * y->size[1];
        y->size[0] = 1;
        y->size[1] = a->size[0];
        emxEnsureCapacity_uint32_T(y, lastBlockLength);
        y_data = y->data;
        firstBlockLength = a->size[0] - 1;
        for (lastBlockLength = 0; lastBlockLength <= firstBlockLength;
             lastBlockLength++) {
            y_data[lastBlockLength] = (unsigned int)lastBlockLength + 1U;
        }

        emxInit_uint32_T(&idxDir);
        lastBlockLength = idxDir->size[0] * idxDir->size[1];
        idxDir->size[0] = 1;
        idxDir->size[1] = y->size[1] + 2;
        emxEnsureCapacity_uint32_T(idxDir, lastBlockLength);
        idxDir_data = idxDir->data;
        idxDir_data[0] = 1U;
        firstBlockLength = y->size[1];
        for (lastBlockLength = 0; lastBlockLength < firstBlockLength;
             lastBlockLength++) {
            idxDir_data[lastBlockLength + 1] = y_data[lastBlockLength];
        }

        idxDir_data[y->size[1] + 1] = (unsigned int)a->size[0];
        firstBlockLength = idxDir->size[1];
        for (lastBlockLength = 0; lastBlockLength < firstBlockLength;
             lastBlockLength++) {
            idxA_data[lastBlockLength] = (int)idxDir_data[lastBlockLength];
        }

        lastBlockLength = y->size[0] * y->size[1];
        y->size[0] = 1;
        y->size[1] = a->size[1];
        emxEnsureCapacity_uint32_T(y, lastBlockLength);
        y_data = y->data;
        firstBlockLength = a->size[1] - 1;
        for (lastBlockLength = 0; lastBlockLength <= firstBlockLength;
             lastBlockLength++) {
            y_data[lastBlockLength] = (unsigned int)lastBlockLength + 1U;
        }

        lastBlockLength = idxDir->size[0] * idxDir->size[1];
        idxDir->size[0] = 1;
        idxDir->size[1] = y->size[1] + 2;
        emxEnsureCapacity_uint32_T(idxDir, lastBlockLength);
        idxDir_data = idxDir->data;
        idxDir_data[0] = 1U;
        firstBlockLength = y->size[1];
        for (lastBlockLength = 0; lastBlockLength < firstBlockLength;
             lastBlockLength++) {
            idxDir_data[lastBlockLength + 1] = y_data[lastBlockLength];
        }

        idxDir_data[y->size[1] + 1] = (unsigned int)a->size[1];
        emxFree_uint32_T(&y);
        firstBlockLength = idxDir->size[1];
        for (lastBlockLength = 0; lastBlockLength < firstBlockLength;
             lastBlockLength++) {
            idxA_data[lastBlockLength + idxA->size[0]] = (int)
                idxDir_data[lastBlockLength];
        }

        unsigned int sizeA_idx_0;
        unsigned int sizeA_idx_1;
        emxFree_uint32_T(&idxDir);
        sizeA_idx_0 = (unsigned int)a->size[0] + 2U;
        sizeA_idx_1 = (unsigned int)a->size[1] + 2U;
        lastBlockLength = a->size[0] * a->size[1];
        a->size[0] = (int)sizeA_idx_0;
        a->size[1] = (int)sizeA_idx_1;
        emxEnsureCapacity_real32_T(a, lastBlockLength);
        a_data = a->data;
        lastBlockLength = (int)sizeA_idx_1;
        firstBlockLength = a->size[0];
        for (hi = 0; hi < lastBlockLength; hi++) {
            for (nblocks = 0; nblocks < firstBlockLength; nblocks++) {
                a_data[nblocks + a->size[0] * hi] = b_varargin_1_data[(idxA_data[nblocks] + b_varargin_1->size[0] * (idxA_data[hi + idxA->size[0]] - 1)) - 1];
            }
        }

        emxFree_int32_T(&idxA);
        emxInit_real32_T(&bx, 2);
        conv2AXPYValidCMP(a, bx);
        a_data = bx->data;
        emxInit_real32_T(&by, 2);
        b_conv2AXPYValidCMP(a, by);
        by_data = by->data;
        if ((bx->size[0] == by->size[0]) && (bx->size[1] == by->size[1])) {
            lastBlockLength = b_varargin_1->size[0] * b_varargin_1->size[1];
            b_varargin_1->size[0] = bx->size[0];
            b_varargin_1->size[1] = bx->size[1];
            emxEnsureCapacity_real32_T(b_varargin_1, lastBlockLength);
            b_varargin_1_data = b_varargin_1->data;
            firstBlockLength = bx->size[0] * bx->size[1];
            for (lastBlockLength = 0; lastBlockLength < firstBlockLength;
                 lastBlockLength++) {
                b_varargin_1_data[lastBlockLength] = a_data[lastBlockLength] *
                                                         a_data[lastBlockLength] +
                                                     by_data[lastBlockLength] *
                                                         by_data[lastBlockLength];
            }
        } else {
            b_binary_expand_op(b_varargin_1, bx, by);
            b_varargin_1_data = b_varargin_1->data;
        }

        if (b_varargin_1->size[0] * b_varargin_1->size[1] == 0) {
            b_y = 0.0;
        } else {
            if (b_varargin_1->size[0] * b_varargin_1->size[1] <= 1024) {
                firstBlockLength = b_varargin_1->size[0] * b_varargin_1->size[1];
                lastBlockLength = 0;
                nblocks = 1;
            } else {
                firstBlockLength = 1024;
                nblocks = (int)((unsigned int)(b_varargin_1->size[0] *
                                               b_varargin_1->size[1]) >>
                                10);
                lastBlockLength = b_varargin_1->size[0] * b_varargin_1->size[1] -
                                  (nblocks << 10);
                if (lastBlockLength > 0) {
                    nblocks++;
                } else {
                    lastBlockLength = 1024;
                }
            }

            b_y = b_varargin_1_data[0];
            for (k = 2; k <= firstBlockLength; k++) {
                b_y += b_varargin_1_data[k - 1];
            }

            for (ib = 2; ib <= nblocks; ib++) {
                double bsum;
                firstBlockLength = (ib - 1) << 10;
                bsum = b_varargin_1_data[firstBlockLength];
                if (ib == nblocks) {
                    hi = lastBlockLength;
                } else {
                    hi = 1024;
                }

                for (k = 2; k <= hi; k++) {
                    bsum += b_varargin_1_data[(firstBlockLength + k) - 1];
                }

                b_y += bsum;
            }
        }

        lastBlockLength = varargout_1->size[0] * varargout_1->size[1];
        varargout_1->size[0] = m;
        varargout_1->size[1] = n;
        emxEnsureCapacity_boolean_T(varargout_1, lastBlockLength);
        c_computeEdgesWithThinningPorta(b_varargin_1, bx, by, 4.0 * b_y / (double)(b_varargin_1->size[0] * b_varargin_1->size[1]), varargout_1);
        emxFree_real32_T(&b_varargin_1);
        emxFree_real32_T(&by);
        emxFree_real32_T(&bx);
    }

    emxFree_real32_T(&a);
}

/**
 * @fn             : estiTform
 * @brief          :
 * @param          : const emxArray_real_T *previousImg
 *                   const emxArray_real_T *currImg
 *                   const double vehicleROI[8]
 *                   float *tform_RotationAngle
 *                   float tform_Translation[2]
 *                   ORBPoints *preMatchedPoints
 *                   ORBPoints *currMatchedPoints
 *                   emxArray_boolean_T *inlierIdx
 * @return         : void
 */
static void estiTform(const emxArray_real_T *previousImg, const emxArray_real_T *currImg, const double vehicleROI[8], float *tform_RotationAngle, float tform_Translation[2],
                      ORBPoints *preMatchedPoints, ORBPoints *currMatchedPoints,
                      emxArray_boolean_T *inlierIdx) {
    ORBPoints b_expl_temp;
    ORBPoints e_expl_temp;
    ORBPoints expl_temp;
    ORBPoints movingValidPoints;
    ORBPoints preValidPoints;
    emxArray_boolean_T *x;
    emxArray_real32_T *c_expl_temp;
    emxArray_real32_T *d_expl_temp;
    emxArray_real32_T *inputs_Location;
    emxArray_real32_T *inputs_Orientation;
    emxArray_real32_T *points;
    emxArray_real_T *b_currImg;
    emxArray_real_T *b_previousImg;
    emxArray_uint32_T *indexPairs;
    emxArray_uint8_T *currFeatures_Features;
    emxArray_uint8_T *preFeatures_Features;
    float failedMatrix[9];
    float tmatrix_data[9];
    float x_data[9];
    float r;
    float s;
    float smax;
    float wpr;
    float *points_data;
    int tmatrix_size[2];
    int b;
    int i;
    int i1;
    int j;
    int k;
    int status;
    int yk;
    unsigned int *indexPairs_data;
    bool isodd;
    bool *inlierIdx_data;

    /*  Brief: 从previousImg图像到currImg的转换估算 */
    /*  Details: */
    /*     None */
    /*   */
    /*  Syntax:   */
    /*      tform = estiTform(previousImg,currImg) */
    /*   */
    /*  Inputs: */
    /*     previousImg - [m,n] size,[double] type,Description */
    /*     currImg - [m,n] size,[double] type,Description */
    /*   */
    /*  Outputs: */
    /*     tform - [m,n] size,[double] type,Description */
    /*   */
    /*  Example:  */
    /*     None */
    /*   */
    /*  See also: None */
    /*  Author:                          cuixingxing */
    /*  Email:                           xingxing.cui@long-horn.com */
    /*  Created:                         17-Oct-2022 10:37:34 */
    /*  Version history revision notes: */
    /*                                   None */
    /*  Implementation In Matlab R2022b */
    /*  Copyright © 2022 long-horn.All Rights Reserved. */
    /*  */
    /*  Convert RGB images to grayscale */
    emxInit_real_T(&b_previousImg, 2);
    rgb2gray(previousImg, b_previousImg);
    emxInit_real_T(&b_currImg, 2);
    rgb2gray(currImg, b_currImg);

    /*  Detect corners using ORB algorithm */
    emxInitStruct_ORBPoints1(&expl_temp);
    detectORBFeatures(b_previousImg, &expl_temp);
    emxInitStruct_ORBPoints1(&b_expl_temp);
    detectORBFeatures(b_currImg, &b_expl_temp);

    /*  排除ego本身的匹配，即车身周围矩形范围的点不应该考虑 */
    /*  if coder.target("MATLAB") % Code generation does not support object arrays */
    /*      prePoints(in1) = []; */
    /*      currPoints(in2) = []; */
    /*  else */
    emxInit_real32_T(&c_expl_temp, 1);
    i = c_expl_temp->size[0];
    c_expl_temp->size[0] = expl_temp.pLocation->size[0];
    emxEnsureCapacity_real32_T(c_expl_temp, i);
    points_data = c_expl_temp->data;
    yk = expl_temp.pLocation->size[0];
    for (i = 0; i < yk; i++) {
        points_data[i] = expl_temp.pLocation->data[i];
    }

    emxInit_real32_T(&d_expl_temp, 1);
    i = d_expl_temp->size[0];
    d_expl_temp->size[0] = expl_temp.pLocation->size[0];
    emxEnsureCapacity_real32_T(d_expl_temp, i);
    points_data = d_expl_temp->data;
    yk = expl_temp.pLocation->size[0];
    for (i = 0; i < yk; i++) {
        points_data[i] = expl_temp.pLocation->data[i + expl_temp.pLocation->size[0]];
    }

    emxInit_boolean_T(&x, 1);
    inpolygon(c_expl_temp, d_expl_temp, &vehicleROI[0], &vehicleROI[4], x);
    inlierIdx_data = x->data;
    yk = x->size[0];
    for (i = 0; i < yk; i++) {
        inlierIdx_data[i] = !inlierIdx_data[i];
    }

    emxInitStruct_ORBPoints1(&e_expl_temp);
    getSubInds(expl_temp.pLocation, expl_temp.pMetric, expl_temp.pScale,
               expl_temp.pOrientation, x, &e_expl_temp);
    i = c_expl_temp->size[0];
    c_expl_temp->size[0] = b_expl_temp.pLocation->size[0];
    emxEnsureCapacity_real32_T(c_expl_temp, i);
    points_data = c_expl_temp->data;
    yk = b_expl_temp.pLocation->size[0];
    for (i = 0; i < yk; i++) {
        points_data[i] = b_expl_temp.pLocation->data[i];
    }

    i = d_expl_temp->size[0];
    d_expl_temp->size[0] = b_expl_temp.pLocation->size[0];
    emxEnsureCapacity_real32_T(d_expl_temp, i);
    points_data = d_expl_temp->data;
    yk = b_expl_temp.pLocation->size[0];
    for (i = 0; i < yk; i++) {
        points_data[i] = b_expl_temp.pLocation->data[i + b_expl_temp.pLocation->size[0]];
    }

    inpolygon(c_expl_temp, d_expl_temp, &vehicleROI[0], &vehicleROI[4], x);
    inlierIdx_data = x->data;
    yk = x->size[0];
    for (i = 0; i < yk; i++) {
        inlierIdx_data[i] = !inlierIdx_data[i];
    }

    getSubInds(b_expl_temp.pLocation, b_expl_temp.pMetric, b_expl_temp.pScale,
               b_expl_temp.pOrientation, x, &expl_temp);
    emxFreeStruct_ORBPoints1(&b_expl_temp);

    /*  end */
    /*  Extract features */
    emxInit_uint8_T(&preFeatures_Features);
    emxInitStruct_ORBPoints(&preValidPoints);
    extractFeatures(b_previousImg, e_expl_temp.pLocation, e_expl_temp.pMetric,
                    e_expl_temp.pScale, e_expl_temp.pOrientation,
                    preFeatures_Features, &preValidPoints);
    emxFreeStruct_ORBPoints1(&e_expl_temp);
    emxFree_real_T(&b_previousImg);
    emxInit_uint8_T(&currFeatures_Features);
    emxInitStruct_ORBPoints(&movingValidPoints);
    extractFeatures(b_currImg, expl_temp.pLocation, expl_temp.pMetric,
                    expl_temp.pScale, expl_temp.pOrientation,
                    currFeatures_Features, &movingValidPoints);
    emxFreeStruct_ORBPoints1(&expl_temp);
    emxFree_real_T(&b_currImg);

    /*  Match features */
    emxInit_uint32_T(&indexPairs);
    matchFeatures(preFeatures_Features, currFeatures_Features, indexPairs);
    indexPairs_data = indexPairs->data;
    emxFree_uint8_T(&currFeatures_Features);
    emxFree_uint8_T(&preFeatures_Features);

    /*  if coder.target("MATLAB") */
    /*      preMatchedPoints = preValidPoints(indexPairs(:,1)); */
    /*      currMatchedPoints = movingValidPoints(indexPairs(:,2)); */
    /*  else */
    preMatchedPoints->pLocation->size[0] = 0;
    preMatchedPoints->pLocation->size[1] = 2;
    preMatchedPoints->pMetric->size[0] = 0;
    preMatchedPoints->pScale->size[0] = 0;
    preMatchedPoints->pOrientation->size[0] = 0;
    emxInit_real32_T(&inputs_Location, 2);
    i = inputs_Location->size[0] * inputs_Location->size[1];
    inputs_Location->size[0] = indexPairs->size[0];
    inputs_Location->size[1] = 2;
    emxEnsureCapacity_real32_T(inputs_Location, i);
    points_data = inputs_Location->data;
    yk = indexPairs->size[0];
    for (i = 0; i < 2; i++) {
        for (i1 = 0; i1 < yk; i1++) {
            points_data[i1 + inputs_Location->size[0] * i] =
                preValidPoints.pLocation->data[((int)indexPairs_data[i1] +
                                                preValidPoints.pLocation->size[0] * i) -
                                               1];
        }
    }

    i = c_expl_temp->size[0];
    c_expl_temp->size[0] = indexPairs->size[0];
    emxEnsureCapacity_real32_T(c_expl_temp, i);
    points_data = c_expl_temp->data;
    yk = indexPairs->size[0];
    for (i = 0; i < yk; i++) {
        points_data[i] = preValidPoints.pMetric->data[(int)indexPairs_data[i] - 1];
    }

    i = d_expl_temp->size[0];
    d_expl_temp->size[0] = indexPairs->size[0];
    emxEnsureCapacity_real32_T(d_expl_temp, i);
    points_data = d_expl_temp->data;
    yk = indexPairs->size[0];
    for (i = 0; i < yk; i++) {
        points_data[i] = preValidPoints.pScale->data[(int)indexPairs_data[i] - 1];
    }

    emxInit_real32_T(&inputs_Orientation, 1);
    i = inputs_Orientation->size[0];
    inputs_Orientation->size[0] = indexPairs->size[0];
    emxEnsureCapacity_real32_T(inputs_Orientation, i);
    points_data = inputs_Orientation->data;
    yk = indexPairs->size[0];
    for (i = 0; i < yk; i++) {
        points_data[i] = preValidPoints.pOrientation->data[(int)indexPairs_data[i] -
                                                           1];
    }

    emxFreeStruct_ORBPoints(&preValidPoints);
    ORBPointsImpl_configure(preMatchedPoints, inputs_Location, c_expl_temp,
                            d_expl_temp, inputs_Orientation);
    currMatchedPoints->pLocation->size[0] = 0;
    currMatchedPoints->pLocation->size[1] = 2;
    currMatchedPoints->pMetric->size[0] = 0;
    currMatchedPoints->pScale->size[0] = 0;
    currMatchedPoints->pOrientation->size[0] = 0;
    i = inputs_Location->size[0] * inputs_Location->size[1];
    inputs_Location->size[0] = indexPairs->size[0];
    inputs_Location->size[1] = 2;
    emxEnsureCapacity_real32_T(inputs_Location, i);
    points_data = inputs_Location->data;
    yk = indexPairs->size[0];
    for (i = 0; i < 2; i++) {
        for (i1 = 0; i1 < yk; i1++) {
            points_data[i1 + inputs_Location->size[0] * i] =
                movingValidPoints.pLocation->data[((int)indexPairs_data[i1 +
                                                                        indexPairs->size[0]] +
                                                   movingValidPoints.pLocation->size[0] * i) -
                                                  1];
        }
    }

    i = c_expl_temp->size[0];
    c_expl_temp->size[0] = indexPairs->size[0];
    emxEnsureCapacity_real32_T(c_expl_temp, i);
    points_data = c_expl_temp->data;
    yk = indexPairs->size[0];
    for (i = 0; i < yk; i++) {
        points_data[i] = movingValidPoints.pMetric->data[(int)indexPairs_data[i +
                                                                              indexPairs->size[0]] -
                                                         1];
    }

    i = d_expl_temp->size[0];
    d_expl_temp->size[0] = indexPairs->size[0];
    emxEnsureCapacity_real32_T(d_expl_temp, i);
    points_data = d_expl_temp->data;
    yk = indexPairs->size[0];
    for (i = 0; i < yk; i++) {
        points_data[i] = movingValidPoints.pScale->data[(int)indexPairs_data[i +
                                                                             indexPairs->size[0]] -
                                                        1];
    }

    i = inputs_Orientation->size[0];
    inputs_Orientation->size[0] = indexPairs->size[0];
    emxEnsureCapacity_real32_T(inputs_Orientation, i);
    points_data = inputs_Orientation->data;
    yk = indexPairs->size[0];
    for (i = 0; i < yk; i++) {
        points_data[i] = movingValidPoints.pOrientation->data[(int)indexPairs_data[i + indexPairs->size[0]] - 1];
    }

    emxFreeStruct_ORBPoints(&movingValidPoints);
    emxFree_uint32_T(&indexPairs);
    ORBPointsImpl_configure(currMatchedPoints, inputs_Location, c_expl_temp,
                            d_expl_temp, inputs_Orientation);
    emxFree_real32_T(&d_expl_temp);
    emxFree_real32_T(&c_expl_temp);
    emxFree_real32_T(&inputs_Orientation);
    emxFree_real32_T(&inputs_Location);

    /*  end */
    /*  Apply transformation - Results may not be identical between runs because of the randomized nature of the algorithm */
    status = (preMatchedPoints->pLocation->size[0] < 2);
    for (i = 0; i < 9; i++) {
        failedMatrix[i] = 0.0F;
    }

    failedMatrix[0] = 1.0F;
    failedMatrix[4] = 1.0F;
    failedMatrix[8] = 1.0F;
    if (status == 0) {
        bool guard1 = false;
        emxInit_real32_T(&points, 3);
        i = points->size[0] * points->size[1] * points->size[2];
        points->size[0] = preMatchedPoints->pLocation->size[0];
        points->size[1] = 2;
        points->size[2] = 2;
        emxEnsureCapacity_real32_T(points, i);
        points_data = points->data;
        i = preMatchedPoints->pLocation->size[0] << 1;
        for (j = 0; j < i; j++) {
            points_data[j] = preMatchedPoints->pLocation->data[j];
        }

        i1 = currMatchedPoints->pLocation->size[0] << 1;
        for (j = 0; j < i1; j++) {
            points_data[i + j] = currMatchedPoints->pLocation->data[j];
        }

        msac(points, &isodd, tmatrix_data, tmatrix_size, inlierIdx);
        emxFree_real32_T(&points);
        if (!isodd) {
            status = 2;
        }

        if ((tmatrix_size[0] == 0) || (tmatrix_size[1] == 0)) {
            smax = 1.0F;
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
            memcpy(&x_data[0], &tmatrix_data[0], (unsigned int)yk * sizeof(float));
            yk = tmatrix_size[0];
            u1 = tmatrix_size[1];
            if (yk <= u1) {
                u1 = yk;
            }

            ipiv_data[0] = 1;
            yk = 1;
            for (k = 2; k <= u1; k++) {
                yk++;
                ipiv_data[k - 1] = (signed char)yk;
            }

            ldap1 = tmatrix_size[0];
            if (tmatrix_size[0] - 1 <= tmatrix_size[1]) {
                i = tmatrix_size[0];
            } else {
                i = 2;
            }

            for (j = 0; j <= i - 2; j++) {
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
                        smax = fabsf(x_data[jj]);
                        for (k = 2; k <= mmj; k++) {
                            s = fabsf(x_data[(b + k) - 1]);
                            if (s > smax) {
                                yk = k - 1;
                                smax = s;
                            }
                        }
                    }
                }

                if (x_data[jj + yk] != 0.0F) {
                    if (yk != 0) {
                        jy = j + yk;
                        ipiv_data[j] = (signed char)(jy + 1);
                        for (k = 0; k <= n + 1; k++) {
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
                for (k = 0; k <= jp1j; k++) {
                    yk = jy + k * m;
                    smax = x_data[yk];
                    if (x_data[yk] != 0.0F) {
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
            for (k = 0; k <= x_size_idx_0 - 2; k++) {
                smax *= x_data[(k + x_size_idx_0 * (k + 1)) + 1];
            }

            isodd = false;
            for (k = 0; k <= u1 - 2; k++) {
                if (ipiv_data[k] > k + 1) {
                    isodd = !isodd;
                }
            }

            if (isodd) {
                smax = -smax;
            }
        }

        guard1 = false;
        if (smax == 0.0F) {
            guard1 = true;
        } else {
            bool b_tmp_data[9];
            bool tmp_data[9];
            bool exitg1;
            yk = tmatrix_size[0] * tmatrix_size[1];
            for (i = 0; i < yk; i++) {
                tmp_data[i] = rtIsInfF(tmatrix_data[i]);
            }

            for (i = 0; i < yk; i++) {
                b_tmp_data[i] = rtIsNaNF(tmatrix_data[i]);
            }

            i = x->size[0];
            x->size[0] = yk;
            emxEnsureCapacity_boolean_T(x, i);
            inlierIdx_data = x->data;
            for (i = 0; i < yk; i++) {
                inlierIdx_data[i] = (tmp_data[i] || b_tmp_data[i]);
            }

            isodd = false;
            yk = 1;
            exitg1 = false;
            while ((!exitg1) && (yk <= x->size[0])) {
                if (inlierIdx_data[yk - 1]) {
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
            status = 2;
            tmatrix_size[0] = 3;
            for (i = 0; i < 9; i++) {
                tmatrix_data[i] = failedMatrix[i];
            }
        }
    } else {
        i = inlierIdx->size[0];
        inlierIdx->size[0] = 1;
        emxEnsureCapacity_boolean_T(inlierIdx, i);
        inlierIdx_data = inlierIdx->data;
        inlierIdx_data[0] = false;
        tmatrix_size[0] = 3;
        for (i = 0; i < 9; i++) {
            tmatrix_data[i] = failedMatrix[i];
        }
    }

    emxFree_boolean_T(&x);
    if (status != 0) {
        tmatrix_size[0] = 3;
        for (i = 0; i < 9; i++) {
            tmatrix_data[i] = failedMatrix[i];
        }
    }

    i = tmatrix_size[0];
    for (i1 = 0; i1 < 3; i1++) {
        failedMatrix[3 * i1] = tmatrix_data[i1];
        failedMatrix[3 * i1 + 1] = tmatrix_data[i1 + i];
        failedMatrix[3 * i1 + 2] = tmatrix_data[i1 + i * 2];
    }

    float b_failedMatrix[4];
    float fv[4];
    b_failedMatrix[0] = failedMatrix[0];
    b_failedMatrix[1] = failedMatrix[1];
    b_failedMatrix[2] = failedMatrix[3];
    b_failedMatrix[3] = failedMatrix[4];
    constrainToRotationMatrix2D(b_failedMatrix, fv, &smax);
    if (rtIsNaNF(smax + 180.0F) || rtIsInfF(smax + 180.0F)) {
        r = rtNaNF;
    } else if (smax + 180.0F == 0.0F) {
        r = 0.0F;
    } else {
        r = fmodf(smax + 180.0F, 360.0F);
        if (r == 0.0F) {
            r = 0.0F;
        } else if (smax + 180.0F < 0.0F) {
            r += 360.0F;
        }
    }

    wpr = roundf(r - 180.0F);
    if (r - 180.0F == wpr) {
        isodd = true;
    } else {
        smax = fabsf((r - 180.0F) - wpr);
        if ((r - 180.0F == 0.0F) || (wpr == 0.0F)) {
            isodd = (smax < 1.4013E-45F);
        } else {
            s = fabsf(r - 180.0F) + fabsf(wpr);
            if (s < 1.17549435E-38F) {
                isodd = (smax < 1.4013E-45F);
            } else {
                isodd = (smax / fminf(s, 3.402823466E+38F) < 1.1920929E-7F);
            }
        }
    }

    *tform_RotationAngle = r - 180.0F;
    if (isodd) {
        *tform_RotationAngle = wpr;
    }

    tform_Translation[0] = failedMatrix[6];
    tform_Translation[1] = failedMatrix[7];

    /*  debug */
    /*  numsTemp = size(preMatchedPoints.Location,1); */
    /*  % t = datetime('now','InputFormat','yyyy-MM-dd HH:mm:ss.SSS'); */
    /*  % t.Format = 'yyyy-MM-dd_HH-mm-ss_SSS'; */
    /*  % name = [char(t),'.txt']; */
    /*  persistent ii */
    /*  if isempty(ii) */
    /*      ii = 1; */
    /*  else */
    /*      ii = ii+1; */
    /*  end */
    /*  name = string(ii)+".txt"; */
    /*  fid = fopen(name,"w"); */
    /*  fprintf(fid,"------------------------------------------------\n"); */
    /*  fprintf(fid,"preMatchedPoints, currMatchedPoints,nums:%d\n",int64(numsTemp)); */
    /*  for ith = 1:numsTemp */
    /*      fprintf(fid,"(%-010.3f, %-010.3f),(%-010.3f, %-010.3f)\n",preMatchedPoints.Location(ith,1),preMatchedPoints.Location(ith,2),... */
    /*          currMatchedPoints.Location(ith,1),currMatchedPoints.Location(ith,2)); */
    /*  end */
    /*  fprintf(fid,"tform.A:\n"); */
    /*  temA = tform.A'; */
    /*  for inum = 1:3 */
    /*      for jnum = 1:3 */
    /*          fprintf(fid,"%-08.5f,",temA(inum,jnum)); */
    /*      end */
    /*      fprintf(fid,"\n"); */
    /*  end */
    /*  fprintf(fid,"\n"); */
    /*  fclose(fid); */
    /*  figure; showMatchedFeatures(previousImg,currImg,preMatchedPoints,currMatchedPoints,'montage') */
}

/**
 * @fn             : evaluateTform2d
 * @brief          :
 * @param          : const float tform[9]
 *                   const emxArray_real32_T *points
 *                   emxArray_real32_T *dis
 * @return         : void
 */
static void evaluateTform2d(const float tform[9], const emxArray_real32_T *points, emxArray_real32_T *dis) {
    emxArray_boolean_T *r;
    emxArray_int32_T *r2;
    emxArray_real32_T *b_points;
    emxArray_real32_T *b_pt1h;
    emxArray_real32_T *delta;
    emxArray_real32_T *pt1h;
    emxArray_real32_T *y;
    const float *points_data;
    float *b_pt1h_data;
    float *delta_data;
    float *pt1h_data;
    int b_i;
    int coffset;
    int i;
    int j;
    int k;
    int loop_ub;
    int nx;
    int *r3;
    signed char input_sizes_idx_1;
    signed char sizes_idx_1;
    bool empty_non_axis_sizes;
    bool *r1;
    points_data = points->data;
    if (points->size[0] != 0) {
        nx = points->size[0];
    } else {
        nx = 0;
    }

    empty_non_axis_sizes = (nx == 0);
    if (empty_non_axis_sizes || (points->size[0] != 0)) {
        input_sizes_idx_1 = 2;
    } else {
        input_sizes_idx_1 = 0;
    }

    if (empty_non_axis_sizes || (points->size[0] != 0)) {
        sizes_idx_1 = 1;
    } else {
        sizes_idx_1 = 0;
    }

    emxInit_real32_T(&b_points, 2);
    i = b_points->size[0] * b_points->size[1];
    b_points->size[0] = points->size[0];
    b_points->size[1] = 2;
    emxEnsureCapacity_real32_T(b_points, i);
    delta_data = b_points->data;
    loop_ub = points->size[0];
    for (i = 0; i < 2; i++) {
        for (coffset = 0; coffset < loop_ub; coffset++) {
            delta_data[coffset + b_points->size[0] * i] = points_data[coffset +
                                                                      points->size[0] * i];
        }
    }

    emxInit_real32_T(&pt1h, 2);
    i = pt1h->size[0] * pt1h->size[1];
    pt1h->size[0] = nx;
    pt1h->size[1] = input_sizes_idx_1 + sizes_idx_1;
    emxEnsureCapacity_real32_T(pt1h, i);
    pt1h_data = pt1h->data;
    loop_ub = input_sizes_idx_1;
    for (i = 0; i < loop_ub; i++) {
        for (coffset = 0; coffset < nx; coffset++) {
            pt1h_data[coffset + pt1h->size[0] * i] = delta_data[coffset + nx * i];
        }
    }

    emxFree_real32_T(&b_points);
    loop_ub = sizes_idx_1;
    if (loop_ub - 1 >= 0) {
        for (i = 0; i < nx; i++) {
            pt1h_data[i + pt1h->size[0] * input_sizes_idx_1] = 1.0F;
        }
    }

    nx = pt1h->size[0];
    loop_ub = pt1h->size[1];
    emxInit_real32_T(&b_pt1h, 2);
    i = b_pt1h->size[0] * b_pt1h->size[1];
    b_pt1h->size[0] = pt1h->size[0];
    b_pt1h->size[1] = 3;
    emxEnsureCapacity_real32_T(b_pt1h, i);
    b_pt1h_data = b_pt1h->data;
    for (j = 0; j < 3; j++) {
        int boffset;
        coffset = j * nx;
        boffset = j * 3;
        for (b_i = 0; b_i < nx; b_i++) {
            b_pt1h_data[coffset + b_i] = 0.0F;
        }

        for (k = 0; k < loop_ub; k++) {
            float bkj;
            int aoffset;
            aoffset = k * pt1h->size[0];
            bkj = tform[boffset + k];
            for (b_i = 0; b_i < nx; b_i++) {
                i = coffset + b_i;
                b_pt1h_data[i] += pt1h_data[aoffset + b_i] * bkj;
            }
        }
    }

    emxFree_real32_T(&pt1h);
    emxInit_real32_T(&delta, 2);
    if (b_pt1h->size[0] == points->size[0]) {
        i = delta->size[0] * delta->size[1];
        delta->size[0] = b_pt1h->size[0];
        delta->size[1] = 2;
        emxEnsureCapacity_real32_T(delta, i);
        delta_data = delta->data;
        loop_ub = b_pt1h->size[0];
        for (i = 0; i < loop_ub; i++) {
            delta_data[i] = b_pt1h_data[i] / b_pt1h_data[i + b_pt1h->size[0] * 2];
        }

        loop_ub = b_pt1h->size[0];
        for (i = 0; i < loop_ub; i++) {
            delta_data[i + delta->size[0]] = b_pt1h_data[i + b_pt1h->size[0]] /
                                             b_pt1h_data[i + b_pt1h->size[0] * 2];
        }

        i = delta->size[0] * delta->size[1];
        delta->size[1] = 2;
        emxEnsureCapacity_real32_T(delta, i);
        delta_data = delta->data;
        loop_ub = delta->size[0];
        for (i = 0; i < 2; i++) {
            for (coffset = 0; coffset < loop_ub; coffset++) {
                delta_data[coffset + delta->size[0] * i] -= points_data[(coffset +
                                                                         points->size[0] * i) +
                                                                        points->size[0] * 2];
            }
        }
    } else {
        d_binary_expand_op(delta, b_pt1h, points);
        delta_data = delta->data;
    }

    i = dis->size[0];
    dis->size[0] = delta->size[0];
    emxEnsureCapacity_real32_T(dis, i);
    pt1h_data = dis->data;
    nx = delta->size[0];
    for (k = 0; k < nx; k++) {
        pt1h_data[k] = rt_hypotf_snf(delta_data[k], delta_data[k + delta->size[0]]);
    }

    emxFree_real32_T(&delta);
    nx = b_pt1h->size[0];
    emxInit_real32_T(&y, 1);
    i = y->size[0];
    y->size[0] = b_pt1h->size[0];
    emxEnsureCapacity_real32_T(y, i);
    delta_data = y->data;
    for (k = 0; k < nx; k++) {
        delta_data[k] = fabsf(b_pt1h_data[k + b_pt1h->size[0] * 2]);
    }

    emxFree_real32_T(&b_pt1h);
    emxInit_boolean_T(&r, 1);
    i = r->size[0];
    r->size[0] = y->size[0];
    emxEnsureCapacity_boolean_T(r, i);
    r1 = r->data;
    loop_ub = y->size[0];
    for (i = 0; i < loop_ub; i++) {
        r1[i] = (delta_data[i] < 1.1920929E-7F);
    }

    emxFree_real32_T(&y);
    loop_ub = r->size[0] - 1;
    nx = 0;
    for (b_i = 0; b_i <= loop_ub; b_i++) {
        if (r1[b_i]) {
            nx++;
        }
    }

    emxInit_int32_T(&r2, 1);
    i = r2->size[0];
    r2->size[0] = nx;
    emxEnsureCapacity_int32_T(r2, i);
    r3 = r2->data;
    nx = 0;
    for (b_i = 0; b_i <= loop_ub; b_i++) {
        if (r1[b_i]) {
            r3[nx] = b_i + 1;
            nx++;
        }
    }

    emxFree_boolean_T(&r);
    loop_ub = r2->size[0];
    for (i = 0; i < loop_ub; i++) {
        pt1h_data[r3[i] - 1] = rtInfF;
    }

    emxFree_int32_T(&r2);
}

/**
 * @fn             : extractFeatures
 * @brief          :
 * @param          : const emxArray_real_T *b_I
 *                   const emxArray_real32_T *points_pLocation
 *                   const emxArray_real32_T *points_pMetric
 *                   const emxArray_real32_T *points_pScale
 *                   const emxArray_real32_T *points_pOrientation
 *                   emxArray_uint8_T *features_Features
 *                   ORBPoints *valid_points
 * @return         : void
 */
static void extractFeatures(const emxArray_real_T *b_I, const emxArray_real32_T *points_pLocation, const emxArray_real32_T *points_pMetric, const emxArray_real32_T *points_pScale, const emxArray_real32_T *points_pOrientation,
                            emxArray_uint8_T *features_Features, ORBPoints *valid_points) {
    emxArray_real32_T *valLocation;
    emxArray_real32_T *valMetric;
    emxArray_real32_T *valOrientation;
    emxArray_real32_T *valScale;
    emxArray_uint8_T *Iu8;
    const float *points_pLocation_data;
    const float *points_pMetric_data;
    const float *points_pOrientation_data;
    const float *points_pScale_data;
    float *valLocation_data;
    float *valMetric_data;
    float *valOrientation_data;
    float *valScale_data;
    int i;
    int i1;
    int numPtsOut;
    unsigned char *Iu8_data;
    points_pOrientation_data = points_pOrientation->data;
    points_pScale_data = points_pScale->data;
    points_pMetric_data = points_pMetric->data;
    points_pLocation_data = points_pLocation->data;
    emxInit_uint8_T(&Iu8);
    uint8PortableCodeAlgo(b_I, Iu8);
    Iu8_data = Iu8->data;
    emxInit_real32_T(&valLocation, 2);
    i = valLocation->size[0] * valLocation->size[1];
    valLocation->size[0] = points_pLocation->size[0];
    valLocation->size[1] = 2;
    emxEnsureCapacity_real32_T(valLocation, i);
    valLocation_data = valLocation->data;
    emxInit_real32_T(&valScale, 1);
    i = valScale->size[0];
    valScale->size[0] = points_pLocation->size[0];
    emxEnsureCapacity_real32_T(valScale, i);
    valScale_data = valScale->data;
    emxInit_real32_T(&valMetric, 1);
    i = valMetric->size[0];
    valMetric->size[0] = points_pLocation->size[0];
    emxEnsureCapacity_real32_T(valMetric, i);
    valMetric_data = valMetric->data;
    emxInit_real32_T(&valOrientation, 1);
    i = valOrientation->size[0];
    valOrientation->size[0] = points_pLocation->size[0];
    emxEnsureCapacity_real32_T(valOrientation, i);
    valOrientation_data = valOrientation->data;
    numPtsOut = points_pLocation->size[0];
    for (i = 0; i < 2; i++) {
        for (i1 = 0; i1 < numPtsOut; i1++) {
            valLocation_data[i1 + valLocation->size[0] * i] = points_pLocation_data[i1 + points_pLocation->size[0] * i];
        }
    }

    numPtsOut = points_pScale->size[0];
    for (i = 0; i < numPtsOut; i++) {
        valScale_data[i] = points_pScale_data[i] * 31.0F;
    }

    numPtsOut = points_pMetric->size[0];
    for (i = 0; i < numPtsOut; i++) {
        valMetric_data[i] = points_pMetric_data[i];
    }

    numPtsOut = points_pOrientation->size[0];
    for (i = 0; i < numPtsOut; i++) {
        valOrientation_data[i] = points_pOrientation_data[i] * 57.2957802F;
    }

    void *pFtrs;
    void *pVPts;
    pVPts = NULL;
    pFtrs = NULL;
    numPtsOut = extractORBComputeCM(&Iu8_data[0], Iu8->size[0], Iu8->size[1],
                                    &valLocation_data[0], &valOrientation_data[0], &valMetric_data[0],
                                    &valScale_data[0], valLocation->size[0], b_I->size[0] * b_I->size[1], 1.2F,
                                    8, 31, 0, 2, 0, 31, 20, &pVPts, &pFtrs);
    emxFree_uint8_T(&Iu8);
    i = valLocation->size[0] * valLocation->size[1];
    valLocation->size[0] = numPtsOut;
    valLocation->size[1] = 2;
    emxEnsureCapacity_real32_T(valLocation, i);
    valLocation_data = valLocation->data;
    i = valScale->size[0];
    valScale->size[0] = numPtsOut;
    emxEnsureCapacity_real32_T(valScale, i);
    valScale_data = valScale->data;
    i = valMetric->size[0];
    valMetric->size[0] = numPtsOut;
    emxEnsureCapacity_real32_T(valMetric, i);
    valMetric_data = valMetric->data;
    i = valOrientation->size[0];
    valOrientation->size[0] = numPtsOut;
    emxEnsureCapacity_real32_T(valOrientation, i);
    valOrientation_data = valOrientation->data;
    i = features_Features->size[0] * features_Features->size[1];
    features_Features->size[0] = numPtsOut;
    features_Features->size[1] = 32;
    emxEnsureCapacity_uint8_T(features_Features, i);
    Iu8_data = features_Features->data;
    extractORBAssignOutputCM(pVPts, pFtrs, &valLocation_data[0],
                             &valOrientation_data[0], &valScale_data[0], &valMetric_data[0], &Iu8_data[0]);
    numPtsOut = valMetric->size[0];
    for (i = 0; i < numPtsOut; i++) {
        valMetric_data[i] /= 31.0F;
    }

    valid_points->pLocation->size[0] = 0;
    valid_points->pLocation->size[1] = 2;
    valid_points->pMetric->size[0] = 0;
    valid_points->pScale->size[0] = 0;
    valid_points->pOrientation->size[0] = 0;
    ORBPointsImpl_configure(valid_points, valLocation, valScale, valMetric,
                            valOrientation);
    emxFree_real32_T(&valOrientation);
    emxFree_real32_T(&valMetric);
    emxFree_real32_T(&valScale);
    emxFree_real32_T(&valLocation);
}

/**
 * @fn             : findNearestNeighbors
 * @brief          :
 * @param          : const emxArray_real32_T *scores
 *                   emxArray_uint32_T *indexPairs
 *                   emxArray_real32_T *topTwoMetrics
 * @return         : void
 */
static void findNearestNeighbors(const emxArray_real32_T *scores,
                                 emxArray_uint32_T *indexPairs, emxArray_real32_T *topTwoMetrics) {
    emxArray_int32_T *idx;
    emxArray_int32_T *iidx;
    emxArray_int32_T *indices;
    emxArray_real32_T *ex;
    emxArray_real32_T *xSorted;
    emxArray_uint32_T *topTwoIndices;
    emxArray_uint32_T *y;
    const float *scores_data;
    float *ex_data;
    float *topTwoMetrics_data;
    float *xSorted_data;
    int eint;
    int i;
    int i1;
    int loop_ub;
    int n;
    int *iidx_data;
    unsigned int *indexPairs_data;
    int *indices_data;
    unsigned int *topTwoIndices_data;
    unsigned int *y_data;
    scores_data = scores->data;
    emxInit_real32_T(&xSorted, 2);
    i = xSorted->size[0] * xSorted->size[1];
    xSorted->size[0] = scores->size[0];
    xSorted->size[1] = scores->size[1];
    emxEnsureCapacity_real32_T(xSorted, i);
    xSorted_data = xSorted->data;
    loop_ub = scores->size[0] * scores->size[1];
    for (i = 0; i < loop_ub; i++) {
        xSorted_data[i] = scores_data[i];
    }

    n = 2;
    if (scores->size[1] < 2) {
        n = scores->size[1];
    }

    i = topTwoMetrics->size[0] * topTwoMetrics->size[1];
    topTwoMetrics->size[0] = n;
    topTwoMetrics->size[1] = scores->size[0];
    emxEnsureCapacity_real32_T(topTwoMetrics, i);
    topTwoMetrics_data = topTwoMetrics->data;
    loop_ub = n * scores->size[0];
    for (i = 0; i < loop_ub; i++) {
        topTwoMetrics_data[i] = 0.0F;
    }

    emxInit_int32_T(&indices, 2);
    i = indices->size[0] * indices->size[1];
    indices->size[0] = n;
    indices->size[1] = scores->size[0];
    emxEnsureCapacity_int32_T(indices, i);
    indices_data = indices->data;
    loop_ub = n * scores->size[0];
    for (i = 0; i < loop_ub; i++) {
        indices_data[i] = 0;
    }

    emxInit_uint32_T(&topTwoIndices);
    emxInit_uint32_T(&y);
    y_data = y->data;
    if ((scores->size[0] == 0) || (scores->size[1] == 0)) {
        i = topTwoIndices->size[0] * topTwoIndices->size[1];
        topTwoIndices->size[0] = n;
        topTwoIndices->size[1] = scores->size[0];
        emxEnsureCapacity_uint32_T(topTwoIndices, i);
        topTwoIndices_data = topTwoIndices->data;
        loop_ub = n * scores->size[0];
        for (i = 0; i < loop_ub; i++) {
            topTwoIndices_data[i] = 0U;
        }
    } else {
        emxInit_real32_T(&ex, 1);
        emxInit_int32_T(&idx, 1);
        if (n == 1) {
            c_minimum(scores, ex, idx);
            iidx_data = idx->data;
            ex_data = ex->data;
            loop_ub = scores->size[0];
            for (i = 0; i < loop_ub; i++) {
                topTwoMetrics_data[topTwoMetrics->size[0] * i] = ex_data[i];
            }

            loop_ub = scores->size[0];
            for (i = 0; i < loop_ub; i++) {
                indices_data[indices->size[0] * i] = iidx_data[i];
            }
        } else {
            double t;
            t = frexp(scores->size[1], &eint);
            if (t == 0.5) {
                t = (double)eint - 1.0;
            } else if ((eint == 1) && (t < 0.75)) {
                t = log(2.0 * t) / 0.69314718055994529;
            } else {
                t = log(t) / 0.69314718055994529 + (double)eint;
            }

            if (n < t) {
                emxInit_int32_T(&iidx, 2);
                for (eint = 0; eint < n; eint++) {
                    c_minimum(xSorted, ex, idx);
                    iidx_data = idx->data;
                    ex_data = ex->data;
                    loop_ub = topTwoMetrics->size[1];
                    for (i = 0; i < loop_ub; i++) {
                        topTwoMetrics_data[eint + topTwoMetrics->size[0] * i] = ex_data[i];
                    }

                    loop_ub = indices->size[1];
                    for (i = 0; i < loop_ub; i++) {
                        indices_data[eint + indices->size[0] * i] = iidx_data[i];
                    }

                    if (xSorted->size[0] < 1) {
                        y->size[0] = 1;
                        y->size[1] = 0;
                    } else {
                        i = y->size[0] * y->size[1];
                        y->size[0] = 1;
                        y->size[1] = xSorted->size[0];
                        emxEnsureCapacity_uint32_T(y, i);
                        y_data = y->data;
                        loop_ub = xSorted->size[0] - 1;
                        for (i = 0; i <= loop_ub; i++) {
                            y_data[i] = (unsigned int)i + 1U;
                        }
                    }

                    i = iidx->size[0] * iidx->size[1];
                    iidx->size[0] = 1;
                    iidx->size[1] = y->size[1];
                    emxEnsureCapacity_int32_T(iidx, i);
                    iidx_data = iidx->data;
                    loop_ub = y->size[1];
                    for (i = 0; i < loop_ub; i++) {
                        iidx_data[i] = (int)y_data[i] + xSorted->size[0] *
                                                            (indices_data[eint + indices->size[0] * i] - 1);
                    }

                    loop_ub = iidx->size[1];
                    for (i = 0; i < loop_ub; i++) {
                        xSorted_data[iidx_data[i] - 1] = rtInfF;
                    }
                }

                emxFree_int32_T(&iidx);
            } else {
                emxInit_int32_T(&iidx, 2);
                sort(xSorted, iidx);
                iidx_data = iidx->data;
                xSorted_data = xSorted->data;
                if (n < 1) {
                    loop_ub = 0;
                } else {
                    loop_ub = n;
                }

                i = topTwoMetrics->size[0] * topTwoMetrics->size[1];
                topTwoMetrics->size[0] = loop_ub;
                topTwoMetrics->size[1] = xSorted->size[0];
                emxEnsureCapacity_real32_T(topTwoMetrics, i);
                topTwoMetrics_data = topTwoMetrics->data;
                eint = xSorted->size[0];
                for (i = 0; i < eint; i++) {
                    for (i1 = 0; i1 < loop_ub; i1++) {
                        topTwoMetrics_data[i1 + topTwoMetrics->size[0] * i] = xSorted_data[i + xSorted->size[0] * i1];
                    }
                }

                if (n < 1) {
                    loop_ub = 0;
                } else {
                    loop_ub = n;
                }

                i = indices->size[0] * indices->size[1];
                indices->size[0] = loop_ub;
                indices->size[1] = iidx->size[0];
                emxEnsureCapacity_int32_T(indices, i);
                indices_data = indices->data;
                eint = iidx->size[0];
                for (i = 0; i < eint; i++) {
                    for (i1 = 0; i1 < loop_ub; i1++) {
                        indices_data[i1 + indices->size[0] * i] = iidx_data[i + iidx->size[0] * i1];
                    }
                }

                emxFree_int32_T(&iidx);
            }
        }

        emxFree_int32_T(&idx);
        emxFree_real32_T(&ex);
        i = topTwoIndices->size[0] * topTwoIndices->size[1];
        topTwoIndices->size[0] = indices->size[0];
        topTwoIndices->size[1] = indices->size[1];
        emxEnsureCapacity_uint32_T(topTwoIndices, i);
        topTwoIndices_data = topTwoIndices->data;
        loop_ub = indices->size[0] * indices->size[1];
        for (i = 0; i < loop_ub; i++) {
            topTwoIndices_data[i] = (unsigned int)indices_data[i];
        }
    }

    emxFree_int32_T(&indices);
    emxFree_real32_T(&xSorted);
    if (scores->size[0] < 1) {
        y->size[0] = 1;
        y->size[1] = 0;
    } else {
        i = y->size[0] * y->size[1];
        y->size[0] = 1;
        y->size[1] = scores->size[0];
        emxEnsureCapacity_uint32_T(y, i);
        y_data = y->data;
        loop_ub = scores->size[0] - 1;
        for (i = 0; i <= loop_ub; i++) {
            y_data[i] = (unsigned int)i + 1U;
        }
    }

    i = indexPairs->size[0] * indexPairs->size[1];
    indexPairs->size[0] = 2;
    indexPairs->size[1] = y->size[1];
    emxEnsureCapacity_uint32_T(indexPairs, i);
    indexPairs_data = indexPairs->data;
    loop_ub = y->size[1];
    for (i = 0; i < loop_ub; i++) {
        indexPairs_data[2 * i] = y_data[i];
    }

    emxFree_uint32_T(&y);
    loop_ub = topTwoIndices->size[1];
    for (i = 0; i < loop_ub; i++) {
        indexPairs_data[2 * i + 1] = topTwoIndices_data[topTwoIndices->size[0] * i];
    }

    emxFree_uint32_T(&topTwoIndices);
}

/**
 * @fn             : getSubInds
 * @brief          :
 * @param          : const emxArray_real32_T *srcORBPoints_pLocation
 *                   const emxArray_real32_T *srcORBPoints_pMetric
 *                   const emxArray_real32_T *srcORBPoints_pScale
 *                   const emxArray_real32_T *srcORBPoints_pOrientation
 *                   const emxArray_boolean_T *validIdxs
 *                   ORBPoints *dstORBPoints
 * @return         : void
 */
static void getSubInds(const emxArray_real32_T *srcORBPoints_pLocation, const emxArray_real32_T *srcORBPoints_pMetric, const emxArray_real32_T *srcORBPoints_pScale, const emxArray_real32_T *srcORBPoints_pOrientation,
                       const emxArray_boolean_T *validIdxs, ORBPoints *dstORBPoints) {
    emxArray_int32_T *r;
    emxArray_real32_T *inputs_Location;
    emxArray_real32_T *inputs_Metric;
    emxArray_real32_T *inputs_Orientation;
    emxArray_real32_T *inputs_Scale;
    const float *srcORBPoints_pLocation_data;
    const float *srcORBPoints_pMetric_data;
    const float *srcORBPoints_pOrientation_data;
    const float *srcORBPoints_pScale_data;
    float *inputs_Location_data;
    int end;
    int i;
    int trueCount;
    int *r1;
    const bool *validIdxs_data;
    validIdxs_data = validIdxs->data;
    srcORBPoints_pOrientation_data = srcORBPoints_pOrientation->data;
    srcORBPoints_pScale_data = srcORBPoints_pScale->data;
    srcORBPoints_pMetric_data = srcORBPoints_pMetric->data;
    srcORBPoints_pLocation_data = srcORBPoints_pLocation->data;
    end = validIdxs->size[0] - 1;
    trueCount = 0;
    for (i = 0; i <= end; i++) {
        if (validIdxs_data[i]) {
            trueCount++;
        }
    }

    emxInit_int32_T(&r, 1);
    i = r->size[0];
    r->size[0] = trueCount;
    emxEnsureCapacity_int32_T(r, i);
    r1 = r->data;
    trueCount = 0;
    for (i = 0; i <= end; i++) {
        if (validIdxs_data[i]) {
            r1[trueCount] = i + 1;
            trueCount++;
        }
    }

    emxInit_real32_T(&inputs_Location, 2);
    i = inputs_Location->size[0] * inputs_Location->size[1];
    inputs_Location->size[0] = r->size[0];
    inputs_Location->size[1] = 2;
    emxEnsureCapacity_real32_T(inputs_Location, i);
    inputs_Location_data = inputs_Location->data;
    trueCount = r->size[0];
    for (i = 0; i < 2; i++) {
        for (end = 0; end < trueCount; end++) {
            inputs_Location_data[end + inputs_Location->size[0] * i] =
                srcORBPoints_pLocation_data[(r1[end] + srcORBPoints_pLocation->size[0] *
                                                           i) -
                                            1];
        }
    }

    emxInit_real32_T(&inputs_Metric, 1);
    i = inputs_Metric->size[0];
    inputs_Metric->size[0] = r->size[0];
    emxEnsureCapacity_real32_T(inputs_Metric, i);
    inputs_Location_data = inputs_Metric->data;
    trueCount = r->size[0];
    for (i = 0; i < trueCount; i++) {
        inputs_Location_data[i] = srcORBPoints_pMetric_data[r1[i] - 1];
    }

    emxInit_real32_T(&inputs_Orientation, 1);
    i = inputs_Orientation->size[0];
    inputs_Orientation->size[0] = r->size[0];
    emxEnsureCapacity_real32_T(inputs_Orientation, i);
    inputs_Location_data = inputs_Orientation->data;
    trueCount = r->size[0];
    for (i = 0; i < trueCount; i++) {
        inputs_Location_data[i] = srcORBPoints_pOrientation_data[r1[i] - 1];
    }

    emxInit_real32_T(&inputs_Scale, 1);
    i = inputs_Scale->size[0];
    inputs_Scale->size[0] = r->size[0];
    emxEnsureCapacity_real32_T(inputs_Scale, i);
    inputs_Location_data = inputs_Scale->data;
    trueCount = r->size[0];
    for (i = 0; i < trueCount; i++) {
        inputs_Location_data[i] = srcORBPoints_pScale_data[r1[i] - 1];
    }

    emxFree_int32_T(&r);
    dstORBPoints->pLocation->size[0] = 0;
    dstORBPoints->pLocation->size[1] = 2;
    dstORBPoints->pMetric->size[0] = 0;
    dstORBPoints->pScale->size[0] = 0;
    dstORBPoints->pOrientation->size[0] = 0;
    b_ORBPointsImpl_configure(dstORBPoints, inputs_Location, inputs_Metric,
                              inputs_Scale, inputs_Orientation);
    emxFree_real32_T(&inputs_Orientation);
    emxFree_real32_T(&inputs_Scale);
    emxFree_real32_T(&inputs_Metric);
    emxFree_real32_T(&inputs_Location);
}

/**
 * @fn             : helperStitchImages
 * @brief          :
 * @param          : const cell_wrap_1 images[4]
 *                   const affinetform2d tforms[4]
 *                   emxArray_real_T *outputImage
 *                   double outputView_XWorldLimits[2]
 *                   double outputView_YWorldLimits[2]
 *                   double outputView_ImageSizeAlias[2]
 *                   bool *c_outputView_ForcePixelExtentTo
 * @return         : void
 */
static void helperStitchImages(const cell_wrap_1 images[4], const affinetform2d tforms[4], emxArray_real_T *outputImage, double outputView_XWorldLimits[2],
                               double outputView_YWorldLimits[2], double outputView_ImageSizeAlias[2], bool *c_outputView_ForcePixelExtentTo) {
    static const signed char b_iv[3] = {0, 0, 1};

    emxArray_boolean_T *maskA;
    emxArray_boolean_T *maskAB;
    emxArray_boolean_T *maskB;
    emxArray_boolean_T *r3;
    emxArray_int32_T *r;
    emxArray_int32_T *r1;
    emxArray_int32_T *r2;
    emxArray_real32_T *dist1;
    emxArray_real32_T *dist2;
    emxArray_real_T *alpha1;
    emxArray_real_T *alpha2;
    emxArray_real_T *b_alpha1;
    emxArray_real_T *warpedImage;
    imref2d expl_temp;
    double xlim[8];
    double ylim[8];
    double u[3];
    double height;
    double v_idx_1;
    double width;
    double xMax;
    double yMax;
    double yMin;
    double *alpha1_data;
    double *alpha2_data;
    double *b_alpha1_data;
    double *outputImage_data;
    double *warpedImage_data;
    float varargout_1[9];
    float *dist1_data;
    float *dist2_data;
    int B_tmp_tmp;
    int U_tmp;
    int b_U_tmp;
    int b_i;
    int i;
    int i1;
    int j;
    int *r4;
    int *r5;
    bool *maskAB_data;
    bool *maskA_data;
    bool *maskB_data;

    /*  The helperStitchImages function applies the transforms tforms to the input images and blends  */
    /*  them to produce the outputImage. It additionally returns the outputView, which you can use to  */
    /*  transform any point from the first image in the given image sequence to the output image.  */
    /*  Copyright 2021 MathWorks, Inc. */
    /*  Compute the output limits for each transform. */
    u[0] = 1.0;
    for (i = 0; i < 4; i++) {
        double U[9];
        double V[9];
        float B_tmp[9];
        float b_B_tmp;
        float c_B_tmp;
        float d_B_tmp;
        float e_B_tmp;
        float f_B_tmp;
        float g_B_tmp;
        unsigned int u_tmp;
        unsigned int v_idx_1_tmp;
        bool guard1 = false;
        v_idx_1_tmp = (unsigned int)images[i].f1->size[0];
        u_tmp = (unsigned int)images[i].f1->size[1];
        u[1] = ((double)u_tmp + 1.0) / 2.0;
        u[2] = u_tmp;
        v_idx_1 = ((double)v_idx_1_tmp + 1.0) / 2.0;
        for (j = 0; j < 3; j++) {
            xMax = u[j];
            U[3 * j] = xMax;
            V[3 * j] = 1.0;
            U_tmp = 3 * j + 1;
            U[U_tmp] = xMax;
            V[U_tmp] = v_idx_1;
            b_U_tmp = 3 * j + 2;
            U[b_U_tmp] = xMax;
            V[b_U_tmp] = v_idx_1_tmp;
            B_tmp_tmp = j << 1;
            B_tmp[3 * j] = tforms[i].A23[B_tmp_tmp];
            B_tmp[U_tmp] = tforms[i].A23[B_tmp_tmp + 1];
            B_tmp[b_U_tmp] = b_iv[j];
        }

        guard1 = false;
        if (B_tmp[8] == 1.0F) {
            bool x[2];
            bool exitg1;
            bool y;
            x[0] = (B_tmp[2] == 0.0F);
            x[1] = (B_tmp[5] == 0.0F);
            y = true;
            U_tmp = 0;
            exitg1 = false;
            while ((!exitg1) && (U_tmp < 2)) {
                if (!x[U_tmp]) {
                    y = false;
                    exitg1 = true;
                } else {
                    U_tmp++;
                }
            }

            if (y) {
                b_B_tmp = B_tmp[0];
                c_B_tmp = B_tmp[3];
                d_B_tmp = B_tmp[6];
                e_B_tmp = B_tmp[1];
                f_B_tmp = B_tmp[4];
                g_B_tmp = B_tmp[7];
                for (b_i = 0; b_i < 9; b_i++) {
                    xMax = U[b_i];
                    v_idx_1 = V[b_i];
                    varargout_1[b_i] = (b_B_tmp * (float)xMax + c_B_tmp * (float)v_idx_1) + d_B_tmp;
                    B_tmp[b_i] = (e_B_tmp * (float)xMax + f_B_tmp * (float)v_idx_1) +
                                 g_B_tmp;
                }
            } else {
                guard1 = true;
            }
        } else {
            guard1 = true;
        }

        if (guard1) {
            float beta[9];
            b_B_tmp = B_tmp[2];
            c_B_tmp = B_tmp[5];
            d_B_tmp = B_tmp[8];
            e_B_tmp = B_tmp[0];
            f_B_tmp = B_tmp[3];
            g_B_tmp = B_tmp[6];
            for (b_i = 0; b_i < 9; b_i++) {
                float f;
                xMax = U[b_i];
                v_idx_1 = V[b_i];
                f = (b_B_tmp * (float)xMax + c_B_tmp * (float)v_idx_1) + d_B_tmp;
                beta[b_i] = f;
                varargout_1[b_i] = ((e_B_tmp * (float)xMax + f_B_tmp * (float)v_idx_1) +
                                    g_B_tmp) /
                                   f;
            }

            b_B_tmp = B_tmp[1];
            c_B_tmp = B_tmp[4];
            d_B_tmp = B_tmp[7];
            for (b_i = 0; b_i < 9; b_i++) {
                B_tmp[b_i] = ((b_B_tmp * (float)U[b_i] + c_B_tmp * (float)V[b_i]) +
                              d_B_tmp) /
                             beta[b_i];
            }
        }

        xlim[i] = minimum(varargout_1);
        xlim[i + 4] = maximum(varargout_1);
        ylim[i] = minimum(B_tmp);
        ylim[i + 4] = maximum(B_tmp);
    }

    /*  Find the minimum and maximum output limits. */
    /*      maxImageSize = max(imageSize); */
    v_idx_1 = b_minimum(xlim);
    xMax = b_maximum(xlim);
    yMin = b_minimum(ylim);
    yMax = b_maximum(ylim);

    /*  Width and height of panorama. */
    width = rt_roundd_snf(xMax - v_idx_1);
    height = rt_roundd_snf(yMax - yMin);

    /*  Initialize the "empty" panorama. */
    b_i = outputImage->size[0] * outputImage->size[1] * outputImage->size[2];
    outputImage->size[0] = (int)height;
    outputImage->size[1] = (int)width;
    outputImage->size[2] = 3;
    emxEnsureCapacity_real_T(outputImage, b_i);
    outputImage_data = outputImage->data;
    B_tmp_tmp = (int)height * (int)width * 3;
    for (b_i = 0; b_i < B_tmp_tmp; b_i++) {
        outputImage_data[b_i] = 0.0;
    }

    /*  Create a 2-D spatial reference object defining the size of the panorama. */
    outputView_ImageSizeAlias[0] = height;
    outputView_ImageSizeAlias[1] = width;
    outputView_XWorldLimits[0] = v_idx_1;
    outputView_XWorldLimits[1] = xMax;
    outputView_YWorldLimits[0] = yMin;
    outputView_YWorldLimits[1] = yMax;

    /*  Step 7 - Stitch the images. */
    emxInit_real_T(&warpedImage, 3);
    emxInit_boolean_T(&maskA, 2);
    emxInit_boolean_T(&maskB, 2);
    emxInit_boolean_T(&maskAB, 2);
    emxInit_real_T(&alpha1, 2);
    emxInit_real_T(&alpha2, 2);
    emxInit_real32_T(&dist1, 2);
    emxInit_real32_T(&dist2, 2);
    emxInit_int32_T(&r, 1);
    emxInit_int32_T(&r1, 1);
    emxInit_int32_T(&r2, 1);
    emxInit_boolean_T(&r3, 2);
    emxInit_real_T(&b_alpha1, 3);
    for (i = 0; i < 4; i++) {
        /*  Apply transformation. */
        b_i = warpedImage->size[0] * warpedImage->size[1] * warpedImage->size[2];
        warpedImage->size[0] = images[i].f1->size[0];
        warpedImage->size[1] = images[i].f1->size[1];
        warpedImage->size[2] = 3;
        emxEnsureCapacity_real_T(warpedImage, b_i);
        warpedImage_data = warpedImage->data;
        B_tmp_tmp = images[i].f1->size[0] * images[i].f1->size[1] * 3;
        for (b_i = 0; b_i < B_tmp_tmp; b_i++) {
            warpedImage_data[b_i] = images[i].f1->data[b_i];
        }

        if ((images[i].f1->size[0] >= 1) && (images[i].f1->size[1] >= 1)) {
            expl_temp.ForcePixelExtentToOne = false;
            expl_temp.ImageSizeAlias[0] = height;
            expl_temp.YWorldLimits[0] = yMin;
            expl_temp.XWorldLimits[0] = v_idx_1;
            expl_temp.ImageSizeAlias[1] = width;
            expl_temp.YWorldLimits[1] = yMax;
            expl_temp.XWorldLimits[1] = xMax;
            b_remapAndResampleGeneric2d(images[i].f1, tforms[i].A23, expl_temp,
                                        warpedImage);
            warpedImage_data = warpedImage->data;
        }

        /*  Blend the images. */
        /*  The helperBlendImages function performs alpha blending to the given two input images, I1 and I2,  */
        /*  with alpha values that are proportional to the center seam of each image.  */
        /*  The output Iout is a linear combination of the input images: */
        /*  Iout = alpha * I1 + (1 - alpha) * I2 */
        /*  Copyright 2021 MathWorks, Inc. */
        /*  Identify the image regions in the two images by masking out the black */
        /*  regions. */
        sum(warpedImage, alpha1);
        alpha1_data = alpha1->data;
        b_i = maskA->size[0] * maskA->size[1];
        maskA->size[0] = alpha1->size[0];
        maskA->size[1] = alpha1->size[1];
        emxEnsureCapacity_boolean_T(maskA, b_i);
        maskA_data = maskA->data;
        B_tmp_tmp = alpha1->size[0] * alpha1->size[1];
        for (b_i = 0; b_i < B_tmp_tmp; b_i++) {
            maskA_data[b_i] = (alpha1_data[b_i] != 0.0);
        }

        sum(outputImage, alpha1);
        alpha1_data = alpha1->data;
        b_i = maskB->size[0] * maskB->size[1];
        maskB->size[0] = alpha1->size[0];
        maskB->size[1] = alpha1->size[1];
        emxEnsureCapacity_boolean_T(maskB, b_i);
        maskB_data = maskB->data;
        B_tmp_tmp = alpha1->size[0] * alpha1->size[1];
        for (b_i = 0; b_i < B_tmp_tmp; b_i++) {
            maskB_data[b_i] = (alpha1_data[b_i] != 0.0);
        }

        if ((maskA->size[0] == maskB->size[0]) && (maskA->size[1] == maskB->size[1])) {
            b_i = maskAB->size[0] * maskAB->size[1];
            maskAB->size[0] = maskA->size[0];
            maskAB->size[1] = maskA->size[1];
            emxEnsureCapacity_boolean_T(maskAB, b_i);
            maskAB_data = maskAB->data;
            B_tmp_tmp = maskA->size[0] * maskA->size[1];
            for (b_i = 0; b_i < B_tmp_tmp; b_i++) {
                maskAB_data[b_i] = (maskA_data[b_i] && maskB_data[b_i]);
            }
        } else {
            b_and(maskAB, maskA, maskB);
            maskAB_data = maskAB->data;
        }

        /*  Compute alpha values that are proportional to the center seam of the two */
        /*  images. */
        b_i = alpha1->size[0] * alpha1->size[1];
        alpha1->size[0] = maskA->size[0];
        alpha1->size[1] = maskA->size[1];
        emxEnsureCapacity_real_T(alpha1, b_i);
        alpha1_data = alpha1->data;
        B_tmp_tmp = maskA->size[0] * maskA->size[1];
        for (b_i = 0; b_i < B_tmp_tmp; b_i++) {
            alpha1_data[b_i] = 1.0;
        }

        b_i = alpha2->size[0] * alpha2->size[1];
        alpha2->size[0] = maskB->size[0];
        alpha2->size[1] = maskB->size[1];
        emxEnsureCapacity_real_T(alpha2, b_i);
        alpha2_data = alpha2->data;
        B_tmp_tmp = maskB->size[0] * maskB->size[1];
        for (b_i = 0; b_i < B_tmp_tmp; b_i++) {
            alpha2_data[b_i] = 1.0;
        }

        edge(maskA, r3);
        bwdist(r3, dist1);
        dist1_data = dist1->data;
        edge(maskB, r3);
        bwdist(r3, dist2);
        dist2_data = dist2->data;

        /*  以下为cuixingxing新添加的,添加过渡带渐变区域 */
        /*      minScalar = 0.9; */
        /*      maxScalar = 1.1; */
        /*      maskA(maskAB)  = dist1(maskAB) >= maxScalar*dist2(maskAB); */
        /*      maskB(maskAB) = dist1(maskAB) <= minScalar*dist2(maskAB); */
        /*      cond1 = dist1(maskAB) > minScalar*dist2(maskAB); */
        /*      cond2 = dist1(maskAB) < maxScalar*dist2(maskAB); */
        /*      maskAB(maskAB) = cond1&cond2; */
        /*  alpha1,alpha2 */
        b_U_tmp = maskAB->size[0] * maskAB->size[1] - 1;
        U_tmp = 0;
        for (B_tmp_tmp = 0; B_tmp_tmp <= b_U_tmp; B_tmp_tmp++) {
            if (maskAB_data[B_tmp_tmp]) {
                U_tmp++;
            }
        }

        b_i = r->size[0];
        r->size[0] = U_tmp;
        emxEnsureCapacity_int32_T(r, b_i);
        r4 = r->data;
        U_tmp = 0;
        for (B_tmp_tmp = 0; B_tmp_tmp <= b_U_tmp; B_tmp_tmp++) {
            if (maskAB_data[B_tmp_tmp]) {
                r4[U_tmp] = B_tmp_tmp + 1;
                U_tmp++;
            }
        }

        b_U_tmp = maskAB->size[0] * maskAB->size[1] - 1;
        U_tmp = 0;
        for (B_tmp_tmp = 0; B_tmp_tmp <= b_U_tmp; B_tmp_tmp++) {
            if (maskAB_data[B_tmp_tmp]) {
                U_tmp++;
            }
        }

        b_i = r1->size[0];
        r1->size[0] = U_tmp;
        emxEnsureCapacity_int32_T(r1, b_i);
        r5 = r1->data;
        U_tmp = 0;
        for (B_tmp_tmp = 0; B_tmp_tmp <= b_U_tmp; B_tmp_tmp++) {
            if (maskAB_data[B_tmp_tmp]) {
                r5[U_tmp] = B_tmp_tmp + 1;
                U_tmp++;
            }
        }

        B_tmp_tmp = r->size[0];
        for (b_i = 0; b_i < B_tmp_tmp; b_i++) {
            alpha1_data[r5[b_i] - 1] = dist1_data[r4[b_i] - 1] / (dist1_data[r4[b_i] -
                                                                             1] +
                                                                  dist2_data[r4[b_i] - 1]);
        }

        /*      alpha1(maskB) = 0; */
        b_U_tmp = maskAB->size[0] * maskAB->size[1] - 1;
        U_tmp = 0;
        for (B_tmp_tmp = 0; B_tmp_tmp <= b_U_tmp; B_tmp_tmp++) {
            if (maskAB_data[B_tmp_tmp]) {
                U_tmp++;
            }
        }

        b_i = r2->size[0];
        r2->size[0] = U_tmp;
        emxEnsureCapacity_int32_T(r2, b_i);
        r4 = r2->data;
        U_tmp = 0;
        for (B_tmp_tmp = 0; B_tmp_tmp <= b_U_tmp; B_tmp_tmp++) {
            if (maskAB_data[B_tmp_tmp]) {
                r4[U_tmp] = B_tmp_tmp + 1;
                U_tmp++;
            }
        }

        B_tmp_tmp = r2->size[0];
        for (b_i = 0; b_i < B_tmp_tmp; b_i++) {
            alpha2_data[r4[b_i] - 1] = 1.0 - alpha1_data[r4[b_i] - 1];
        }

        /*      alpha2(maskA) = 0; */
        if (alpha1->size[0] == 1) {
            b_i = warpedImage->size[0];
        } else {
            b_i = alpha1->size[0];
        }

        if (alpha2->size[0] == 1) {
            i1 = outputImage->size[0];
        } else {
            i1 = alpha2->size[0];
        }

        if (alpha1->size[1] == 1) {
            U_tmp = warpedImage->size[1];
        } else {
            U_tmp = alpha1->size[1];
        }

        if (alpha2->size[1] == 1) {
            b_U_tmp = outputImage->size[1];
        } else {
            b_U_tmp = alpha2->size[1];
        }

        if ((alpha1->size[0] == warpedImage->size[0]) && (alpha1->size[1] == warpedImage->size[1]) && (alpha2->size[0] == outputImage->size[0]) &&
            (alpha2->size[1] == outputImage->size[1]) && (b_i == i1) && (U_tmp == b_U_tmp)) {
            U_tmp = alpha1->size[0];
            b_U_tmp = alpha2->size[0];
            b_i = b_alpha1->size[0] * b_alpha1->size[1] * b_alpha1->size[2];
            b_alpha1->size[0] = alpha1->size[0];
            b_alpha1->size[1] = alpha1->size[1];
            b_alpha1->size[2] = 3;
            emxEnsureCapacity_real_T(b_alpha1, b_i);
            b_alpha1_data = b_alpha1->data;
            B_tmp_tmp = alpha1->size[1];
            for (b_i = 0; b_i < 3; b_i++) {
                for (j = 0; j < B_tmp_tmp; j++) {
                    for (i1 = 0; i1 < U_tmp; i1++) {
                        b_alpha1_data[(i1 + b_alpha1->size[0] * j) + b_alpha1->size[0] *
                                                                         b_alpha1->size[1] * b_i] = alpha1_data[i1 + U_tmp * j] *
                                                                                                        warpedImage_data[(i1 + warpedImage->size[0] * j) +
                                                                                                                         warpedImage->size[0] * warpedImage->size[1] * b_i] +
                                                                                                    alpha2_data[i1 + b_U_tmp * j] * outputImage_data[(i1 +
                                                                                                                                                      outputImage->size[0] * j) +
                                                                                                                                                     outputImage->size[0] *
                                                                                                                                                         outputImage->size[1] * b_i];
                    }
                }
            }

            b_i = warpedImage->size[0] * warpedImage->size[1] * warpedImage->size[2];
            warpedImage->size[0] = b_alpha1->size[0];
            warpedImage->size[1] = b_alpha1->size[1];
            warpedImage->size[2] = 3;
            emxEnsureCapacity_real_T(warpedImage, b_i);
            warpedImage_data = warpedImage->data;
            B_tmp_tmp = b_alpha1->size[1];
            for (b_i = 0; b_i < 3; b_i++) {
                for (j = 0; j < B_tmp_tmp; j++) {
                    U_tmp = b_alpha1->size[0];
                    for (i1 = 0; i1 < U_tmp; i1++) {
                        warpedImage_data[(i1 + warpedImage->size[0] * j) + warpedImage->size[0] * warpedImage->size[1] * b_i] = b_alpha1_data[(i1 +
                                                                                                                                               b_alpha1->size[0] * j) +
                                                                                                                                              b_alpha1->size[0] * b_alpha1->size[1] *
                                                                                                                                                  b_i];
                    }
                }
            }

            b_i = outputImage->size[0] * outputImage->size[1] * outputImage->size[2];
            outputImage->size[0] = warpedImage->size[0];
            outputImage->size[1] = warpedImage->size[1];
            outputImage->size[2] = 3;
            emxEnsureCapacity_real_T(outputImage, b_i);
            outputImage_data = outputImage->data;
            B_tmp_tmp = warpedImage->size[0] * warpedImage->size[1] * 3;
            for (b_i = 0; b_i < B_tmp_tmp; b_i++) {
                outputImage_data[b_i] = warpedImage_data[b_i];
            }
        } else {
            binary_expand_op(outputImage, alpha1, warpedImage, alpha2);
            outputImage_data = outputImage->data;
        }

        /*      outputImage = im2uint8(outputImage); */
    }

    emxFree_real_T(&b_alpha1);
    emxFree_boolean_T(&r3);
    emxFree_int32_T(&r2);
    emxFree_int32_T(&r1);
    emxFree_int32_T(&r);
    emxFree_real32_T(&dist2);
    emxFree_real32_T(&dist1);
    emxFree_real_T(&alpha2);
    emxFree_real_T(&alpha1);
    emxFree_boolean_T(&maskAB);
    emxFree_boolean_T(&maskB);
    emxFree_boolean_T(&maskA);
    emxFree_real_T(&warpedImage);
    *c_outputView_ForcePixelExtentTo = false;
}

/**
 * @fn             : helperToObjectBev
 * @brief          :
 * @param          : const double bevStruct_OutputView[4]
 *                   const double bevStruct_ImageSize[2]
 *                   const double c_bevStruct_Sensor_Intrinsics_F[2]
 *                   const double c_bevStruct_Sensor_Intrinsics_P[2]
 *                   double c_bevStruct_Sensor_Intrinsics_S
 *                   double bevStruct_Sensor_Height
 *                   double bevStruct_Sensor_Pitch
 *                   double bevStruct_Sensor_Yaw
 *                   double bevStruct_Sensor_Roll
 *                   const double bevStruct_Sensor_SensorLocation[2]
 *                   double bevObj_OutputView[4]
 *                   double bevObj_ImageSize[2]
 *                   monoCamera *bevObj_Sensor
 *                   imref2d *bevObj_OutputViewImref
 *                   double bevObj_Scale[2]
 * @return         : void
 */
static void helperToObjectBev(const double bevStruct_OutputView[4], const double bevStruct_ImageSize[2], const double c_bevStruct_Sensor_Intrinsics_F[2], const double c_bevStruct_Sensor_Intrinsics_P[2], double c_bevStruct_Sensor_Intrinsics_S, double bevStruct_Sensor_Height, double bevStruct_Sensor_Pitch, double bevStruct_Sensor_Yaw, double bevStruct_Sensor_Roll, const double bevStruct_Sensor_SensorLocation[2], double bevObj_OutputView[4], double bevObj_ImageSize[2], monoCamera *bevObj_Sensor,
                              imref2d *bevObj_OutputViewImref, double bevObj_Scale[2]) {
    double B_data[2];
    double outDimFrac_data[2];
    double worldHW[2];
    int k;
    int trueCount;
    signed char b_tmp_data[2];
    signed char tmp_data[2];
    bool nanIdxHW[2];
    bool exitg1;
    bool y;

    /*  This function converts input birdsEyeView object in structure format back to object. */
    /*  Copyright 2021 MathWorks, Inc. */
    /*  #codegen */
    /*  Step 1: Convert monoCamera struct to Object */
    /*  Create cameraIntrinsics object to struct */
    bevObj_Sensor->Intrinsics.K[0] = c_bevStruct_Sensor_Intrinsics_F[0];
    bevObj_Sensor->Intrinsics.K[3] = c_bevStruct_Sensor_Intrinsics_S;
    bevObj_Sensor->Intrinsics.K[6] = c_bevStruct_Sensor_Intrinsics_P[0];
    bevObj_Sensor->Intrinsics.K[1] = 0.0;
    bevObj_Sensor->Intrinsics.K[4] = c_bevStruct_Sensor_Intrinsics_F[1];
    bevObj_Sensor->Intrinsics.K[7] = c_bevStruct_Sensor_Intrinsics_P[1];
    bevObj_Sensor->Intrinsics.K[2] = 0.0;
    bevObj_Sensor->Intrinsics.K[5] = 0.0;
    bevObj_Sensor->Intrinsics.K[8] = 1.0;

    /*  Create monocamera object */
    bevObj_Sensor->Height = bevStruct_Sensor_Height;
    bevObj_Sensor->Pitch = bevStruct_Sensor_Pitch;
    bevObj_Sensor->Roll = bevStruct_Sensor_Roll;
    bevObj_Sensor->Yaw = bevStruct_Sensor_Yaw;
    bevObj_Sensor->SensorLocation[0] = bevStruct_Sensor_SensorLocation[0];
    bevObj_Sensor->SensorLocation[1] = bevStruct_Sensor_SensorLocation[1];

    /*  Step 2: Create Birds eye view object */
    bevObj_OutputView[0] = bevStruct_OutputView[0];
    bevObj_OutputView[1] = bevStruct_OutputView[1];
    bevObj_OutputView[2] = bevStruct_OutputView[2];
    bevObj_OutputView[3] = bevStruct_OutputView[3];
    bevObj_ImageSize[0] = bevStruct_ImageSize[0];
    worldHW[0] = fabs(bevStruct_OutputView[1] - bevStruct_OutputView[0]);
    nanIdxHW[0] = rtIsNaN(bevStruct_ImageSize[0]);
    bevObj_ImageSize[1] = bevStruct_ImageSize[1];
    worldHW[1] = fabs(bevStruct_OutputView[3] - bevStruct_OutputView[2]);
    nanIdxHW[1] = rtIsNaN(bevStruct_ImageSize[1]);
    y = false;
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k < 2)) {
        if (nanIdxHW[k]) {
            y = true;
            exitg1 = true;
        } else {
            k++;
        }
    }

    if (!y) {
        bevObj_Scale[0] = (bevStruct_ImageSize[1] - 1.0) / worldHW[1];
        bevObj_Scale[1] = (bevStruct_ImageSize[0] - 1.0) / worldHW[0];
    } else {
        double scale;
        int knt;
        bool b_data[2];
        trueCount = 0;
        if (!nanIdxHW[0]) {
            trueCount = 1;
        }

        if (!nanIdxHW[1]) {
            trueCount++;
        }

        knt = 0;
        if (!nanIdxHW[0]) {
            tmp_data[0] = 1;
            knt = 1;
        }

        if (!nanIdxHW[1]) {
            tmp_data[knt] = 2;
        }

        for (k = 0; k < trueCount; k++) {
            outDimFrac_data[k] = bevStruct_ImageSize[tmp_data[k] - 1] - 1.0;
        }

        if (trueCount == 0) {
            scale = 0.0;
        } else if (trueCount == 1) {
            scale = outDimFrac_data[0] / worldHW[tmp_data[0] - 1];
        } else {
            double A_data[2];
            double atmp;
            double beta1;
            double tau_data;
            for (k = 0; k < 2; k++) {
                A_data[k] = worldHW[tmp_data[k] - 1];
            }

            memcpy(&B_data[0], &outDimFrac_data[0], (unsigned int)trueCount * sizeof(double));
            atmp = A_data[0];
            tau_data = 0.0;
            beta1 = b_xnrm2(A_data);
            if (beta1 != 0.0) {
                beta1 = rt_hypotd_snf(A_data[0], beta1);
                if (A_data[0] >= 0.0) {
                    beta1 = -beta1;
                }

                if (fabs(beta1) < 1.0020841800044864E-292) {
                    knt = 0;
                    do {
                        knt++;
                        A_data[1] *= 9.9792015476736E+291;
                        beta1 *= 9.9792015476736E+291;
                        atmp *= 9.9792015476736E+291;
                    } while ((fabs(beta1) < 1.0020841800044864E-292) && (knt < 20));

                    beta1 = rt_hypotd_snf(atmp, b_xnrm2(A_data));
                    if (atmp >= 0.0) {
                        beta1 = -beta1;
                    }

                    tau_data = (beta1 - atmp) / beta1;
                    A_data[1] *= 1.0 / (atmp - beta1);
                    for (k = 0; k < knt; k++) {
                        beta1 *= 1.0020841800044864E-292;
                    }

                    atmp = beta1;
                } else {
                    tau_data = (beta1 - A_data[0]) / beta1;
                    A_data[1] *= 1.0 / (A_data[0] - beta1);
                    atmp = beta1;
                }
            }

            knt = 0;
            beta1 = fabs(atmp);
            if (!(beta1 <= 4.4408920985006262E-15 * beta1)) {
                knt = 1;
            }

            scale = 0.0;
            if (tau_data != 0.0) {
                beta1 = tau_data * (B_data[0] + A_data[1] * B_data[1]);
                if (beta1 != 0.0) {
                    B_data[0] -= beta1;
                }
            }

            for (trueCount = 0; trueCount < knt; trueCount++) {
                scale = B_data[0];
            }

            for (trueCount = knt; trueCount >= 1; trueCount--) {
                scale /= atmp;
            }
        }

        bevObj_Scale[0] = scale;
        bevObj_Scale[1] = scale;
        trueCount = 0;
        if (nanIdxHW[0]) {
            trueCount = 1;
        }

        if (nanIdxHW[1]) {
            trueCount++;
        }

        knt = 0;
        if (nanIdxHW[0]) {
            b_tmp_data[0] = 1;
            knt = 1;
        }

        if (nanIdxHW[1]) {
            b_tmp_data[knt] = 2;
        }

        for (k = 0; k < trueCount; k++) {
            outDimFrac_data[k] = scale * worldHW[b_tmp_data[k] - 1];
        }

        for (k = 0; k < trueCount; k++) {
            outDimFrac_data[k] = rt_roundd_snf(outDimFrac_data[k]);
        }

        knt = trueCount - 1;
        for (k = 0; k <= knt; k++) {
            outDimFrac_data[k]++;
        }

        for (k = 0; k < trueCount; k++) {
            b_data[k] = rtIsInf(outDimFrac_data[k]);
        }

        y = (trueCount != 0);
        if (y) {
            k = 0;
            exitg1 = false;
            while ((!exitg1) && (k <= trueCount - 1)) {
                if (!b_data[k]) {
                    y = false;
                    exitg1 = true;
                } else {
                    k++;
                }
            }
        }

        if (y) {
            outDimFrac_data[0] = 1.7976931348623157E+308;
        }

        knt = 0;
        if (nanIdxHW[0]) {
            bevObj_ImageSize[0] = outDimFrac_data[0];
            knt = 1;
        }

        if (nanIdxHW[1]) {
            bevObj_ImageSize[1] = outDimFrac_data[knt];
        }
    }

    bevObj_OutputViewImref->ImageSizeAlias[0] = bevObj_ImageSize[0];
    bevObj_OutputViewImref->ImageSizeAlias[1] = bevObj_ImageSize[1];
    bevObj_OutputViewImref->XWorldLimits[0] = 0.5;
    bevObj_OutputViewImref->XWorldLimits[1] = bevObj_ImageSize[1] + 0.5;
    bevObj_OutputViewImref->YWorldLimits[0] = 0.5;
    bevObj_OutputViewImref->YWorldLimits[1] = bevObj_ImageSize[0] + 0.5;
    bevObj_OutputViewImref->ForcePixelExtentToOne = true;
}

/**
 * @fn             : imwarp
 * @brief          :
 * @param          : emxArray_real_T *varargin_1
 *                   float varargin_2_RotationAngle
 *                   const float varargin_2_Translation[2]
 *                   imref2d *varargin_5
 * @return         : void
 */
static void imwarp(emxArray_real_T *varargin_1, float varargin_2_RotationAngle,
                   const float varargin_2_Translation[2], imref2d *varargin_5) {
    emxArray_real_T *b_varargin_1;
    double *b_varargin_1_data;
    double *varargin_1_data;
    int i;
    varargin_1_data = varargin_1->data;
    emxInit_real_T(&b_varargin_1, 3);
    if ((varargin_1->size[0] < 1) || (varargin_1->size[1] < 1)) {
        varargin_5->ImageSizeAlias[0] = 2.0;
        varargin_5->XWorldLimits[0] = 0.5;
        varargin_5->YWorldLimits[0] = 0.5;
        varargin_5->ImageSizeAlias[1] = 2.0;
        varargin_5->XWorldLimits[1] = 2.5;
        varargin_5->YWorldLimits[1] = 2.5;
        varargin_5->ForcePixelExtentToOne = true;
    } else {
        int loop_ub;
        i = b_varargin_1->size[0] * b_varargin_1->size[1] * b_varargin_1->size[2];
        b_varargin_1->size[0] = varargin_1->size[0];
        b_varargin_1->size[1] = varargin_1->size[1];
        b_varargin_1->size[2] = 3;
        emxEnsureCapacity_real_T(b_varargin_1, i);
        b_varargin_1_data = b_varargin_1->data;
        loop_ub = varargin_1->size[0] * varargin_1->size[1] * varargin_1->size[2] -
                  1;
        for (i = 0; i <= loop_ub; i++) {
            b_varargin_1_data[i] = varargin_1_data[i];
        }

        c_remapAndResampleGeneric2d(b_varargin_1, varargin_2_RotationAngle,
                                    varargin_2_Translation, *varargin_5, varargin_1);
    }

    emxFree_real_T(&b_varargin_1);
}

/**
 * @fn             : inpolygon
 * @brief          :
 * @param          : const emxArray_real32_T *x
 *                   const emxArray_real32_T *y
 *                   const double xv[4]
 *                   const double yv[4]
 *                   emxArray_boolean_T *in
 * @return         : void
 */
static void inpolygon(const emxArray_real32_T *x, const emxArray_real32_T *y,
                      const double xv[4], const double yv[4], emxArray_boolean_T *in) {
    double xv1;
    double xv2;
    double xvFirst;
    double yv1;
    double yv2;
    double yvFirst;
    const float *x_data;
    const float *y_data;
    float xj;
    float yj;
    int b_i;
    int b_k;
    int exitg2;
    int exitg3;
    int i;
    int j;
    int k;
    int kfirst;
    int xvFirst_tmp;
    signed char dquad;
    signed char quad1;
    signed char quad2;
    signed char quadFirst;
    signed char sdq;
    bool onj;
    bool *in_data;
    y_data = y->data;
    x_data = x->data;
    i = in->size[0];
    in->size[0] = x->size[0];
    emxEnsureCapacity_boolean_T(in, i);
    in_data = in->data;
    kfirst = x->size[0];
    for (i = 0; i < kfirst; i++) {
        in_data[i] = false;
    }

    if (x->size[0] != 0) {
        int last[4];
        int nloops;
        signed char first[4];
        nloops = 0;
        first[0] = 0;
        last[0] = 0;
        first[1] = 0;
        last[1] = 0;
        first[2] = 0;
        last[2] = 0;
        first[3] = 0;
        last[3] = 0;
        k = 0;
        while ((k + 1 <= 4) && rtIsNaN(xv[k])) {
            k++;
        }

        while (k + 1 <= 4) {
            bool exitg1;
            nloops++;
            kfirst = k;
            first[nloops - 1] = (signed char)(k + 1);
            exitg1 = false;
            while ((!exitg1) && (k + 1 < 4)) {
                k++;
                if (rtIsNaN(xv[k]) || rtIsNaN(yv[k])) {
                    k--;
                    exitg1 = true;
                }
            }

            if ((xv[k] == xv[kfirst]) && (yv[k] == yv[kfirst]) && (kfirst + 1 != k + 1)) {
                last[nloops - 1] = k;
            } else {
                last[nloops - 1] = k + 1;
            }

            k += 2;
            while ((k + 1 <= 4) && rtIsNaN(xv[k])) {
                k++;
            }
        }

        if (nloops != 0) {
            double scale[4];
            double maxxv;
            double maxyv;
            double minxv;
            double minyv;
            minxv = xv[first[0] - 1];
            maxxv = minxv;
            minyv = yv[first[0] - 1];
            maxyv = minyv;
            i = (unsigned char)nloops;
            scale[0] = 0.0;
            scale[1] = 0.0;
            scale[2] = 0.0;
            scale[3] = 0.0;
            for (k = 0; k < i; k++) {
                double d;
                double lmaxxv;
                double lmaxyv;
                double lminxv_tmp;
                double lminyv_tmp;
                int endIdx;
                signed char i1;
                i1 = first[k];
                kfirst = i1;
                endIdx = last[k];
                lminxv_tmp = xv[i1 - 1];
                lmaxxv = lminxv_tmp;
                for (b_i = kfirst; b_i <= endIdx; b_i++) {
                    d = xv[b_i - 1];
                    if (lminxv_tmp > d) {
                        lminxv_tmp = d;
                    }

                    if (lmaxxv < d) {
                        lmaxxv = d;
                    }
                }

                endIdx = last[k];
                lminyv_tmp = yv[i1 - 1];
                lmaxyv = lminyv_tmp;
                for (b_i = kfirst; b_i <= endIdx; b_i++) {
                    d = yv[b_i - 1];
                    if (lminyv_tmp > d) {
                        lminyv_tmp = d;
                    }

                    if (lmaxyv < d) {
                        lmaxyv = d;
                    }
                }

                if (minxv > lminxv_tmp) {
                    minxv = lminxv_tmp;
                }

                if (maxxv < lmaxxv) {
                    maxxv = lmaxxv;
                }

                if (minyv > lminyv_tmp) {
                    minyv = lminyv_tmp;
                }

                if (maxyv < lmaxyv) {
                    maxyv = lmaxyv;
                }

                endIdx = first[k];
                kfirst = last[k] - 1;
                for (b_i = endIdx; b_i <= kfirst; b_i++) {
                    lminxv_tmp = fabs(0.5 * (xv[b_i - 1] + xv[b_i]));
                    lmaxxv = fabs(0.5 * (yv[b_i - 1] + yv[b_i]));
                    if ((lminxv_tmp > 1.0) && (lmaxxv > 1.0)) {
                        lminxv_tmp *= lmaxxv;
                    } else if ((lmaxxv > lminxv_tmp) || rtIsNaN(lminxv_tmp)) {
                        lminxv_tmp = lmaxxv;
                    }

                    scale[b_i - 1] = lminxv_tmp * 6.6613381477509392E-16;
                }

                kfirst = first[k] - 1;
                endIdx = last[k];
                lminxv_tmp = fabs(0.5 * (xv[endIdx - 1] + xv[kfirst]));
                lmaxxv = fabs(0.5 * (yv[endIdx - 1] + yv[kfirst]));
                if ((lminxv_tmp > 1.0) && (lmaxxv > 1.0)) {
                    lminxv_tmp *= lmaxxv;
                } else if ((lmaxxv > lminxv_tmp) || rtIsNaN(lminxv_tmp)) {
                    lminxv_tmp = lmaxxv;
                }

                scale[endIdx - 1] = lminxv_tmp * 6.6613381477509392E-16;
            }

            kfirst = x->size[0] - 1;

#pragma omp parallel for num_threads(omp_get_max_threads()) private(xj, yj, sdq, b_k, exitg3, xvFirst_tmp, xvFirst, yvFirst, quadFirst, xv2, yv2, quad2, exitg2, dquad, onj, xv1, yv1, quad1)

            for (j = 0; j <= kfirst; j++) {
                xj = x_data[j];
                yj = y_data[j];
                in_data[j] = false;
                if ((xj >= minxv) && (xj <= maxxv) && (yj >= minyv) && (yj <= maxyv)) {
                    sdq = 0;
                    b_k = 0;
                    do {
                        exitg3 = 0;
                        if (b_k <= (unsigned char)nloops - 1) {
                            xvFirst_tmp = first[b_k] - 1;
                            xvFirst = xv[xvFirst_tmp] - xj;
                            yvFirst = yv[xvFirst_tmp] - yj;
                            if (xvFirst > 0.0) {
                                if (yvFirst > 0.0) {
                                    quadFirst = 0;
                                } else {
                                    quadFirst = 3;
                                }
                            } else if (yvFirst > 0.0) {
                                quadFirst = 1;
                            } else {
                                quadFirst = 2;
                            }

                            xv2 = xvFirst;
                            yv2 = yvFirst;
                            quad2 = quadFirst;
                            xvFirst_tmp = first[b_k];
                            do {
                                exitg2 = 0;
                                if (xvFirst_tmp <= last[b_k] - 1) {
                                    xv1 = xv2;
                                    yv1 = yv2;
                                    xv2 = xv[xvFirst_tmp] - xj;
                                    yv2 = yv[xvFirst_tmp] - yj;
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

                                    contrib(xv1, yv1, xv2, yv2, quad1, quad2, scale[xvFirst_tmp - 1], &dquad, &onj);
                                    if (onj) {
                                        in_data[j] = true;
                                        exitg2 = 1;
                                    } else {
                                        sdq += dquad;
                                        xvFirst_tmp++;
                                    }
                                } else {
                                    contrib(xv2, yv2, xvFirst, yvFirst, quad2, quadFirst,
                                            scale[last[b_k] - 1], &dquad, &onj);
                                    exitg2 = 2;
                                }
                            } while (exitg2 == 0);

                            if (exitg2 == 1) {
                                exitg3 = 1;
                            } else if (onj) {
                                in_data[j] = true;
                                exitg3 = 1;
                            } else {
                                sdq += dquad;
                                b_k++;
                            }
                        } else {
                            in_data[j] = (sdq != 0);
                            exitg3 = 1;
                        }
                    } while (exitg3 == 0);
                }
            }
        }
    }
}

/**
 * @fn             : interp2
 * @brief          :
 * @param          : const emxArray_real_T *varargin_1
 *                   const emxArray_real_T *varargin_2
 *                   const emxArray_real_T *varargin_3
 *                   const emxArray_real_T *varargin_4
 *                   const emxArray_real_T *varargin_5
 *                   emxArray_real_T *Vq
 * @return         : void
 */
static void interp2(const emxArray_real_T *varargin_1, const emxArray_real_T *varargin_2, const emxArray_real_T *varargin_3, const emxArray_real_T *varargin_4, const emxArray_real_T *varargin_5, emxArray_real_T *Vq) {
    c_interp2_local(varargin_3, varargin_4, varargin_5, varargin_1, varargin_2, Vq);
}

/**
 * @fn             : interp2_local
 * @brief          :
 * @param          : const emxArray_real32_T *V
 *                   const emxArray_real_T *Xq
 *                   const emxArray_real_T *Yq
 *                   const emxArray_real32_T *X
 *                   const emxArray_real32_T *Y
 *                   emxArray_real32_T *Vq
 * @return         : void
 */
static void interp2_local(const emxArray_real32_T *V, const emxArray_real_T *Xq,
                          const emxArray_real_T *Yq, const emxArray_real32_T *X, const emxArray_real32_T *Y, emxArray_real32_T *Vq) {
    const double *Xq_data;
    const double *Yq_data;
    const float *V_data;
    const float *X_data;
    const float *Y_data;
    float qx1;
    float qx2;
    float rx;
    float zx1y1;
    float zx1y2;
    float *Vq_data;
    int ix;
    int iy;
    int k;
    int ub_loop;
    Y_data = Y->data;
    X_data = X->data;
    Yq_data = Yq->data;
    Xq_data = Xq->data;
    V_data = V->data;
    ub_loop = Vq->size[0] * Vq->size[1];
    Vq->size[0] = Xq->size[0];
    Vq->size[1] = Xq->size[1];
    emxEnsureCapacity_real32_T(Vq, ub_loop);
    Vq_data = Vq->data;
    ub_loop = Xq->size[0] * Xq->size[1] - 1;

#pragma omp parallel for num_threads(omp_get_max_threads()) private(ix, iy, zx1y1, qx1, zx1y2, qx2, rx)

    for (k = 0; k <= ub_loop; k++) {
        if ((Xq_data[k] >= X_data[0]) && (Xq_data[k] <= X_data[X->size[1] - 1]) &&
            (Yq_data[k] >= Y_data[0]) && (Yq_data[k] <= Y_data[Y->size[1] - 1])) {
            ix = c_bsearch(X, Xq_data[k]);
            iy = c_bsearch(Y, Yq_data[k]);
            zx1y1 = V_data[(iy + V->size[0] * (ix - 1)) - 1];
            qx1 = V_data[(iy + V->size[0] * ix) - 1];
            zx1y2 = V_data[iy + V->size[0] * (ix - 1)];
            qx2 = V_data[iy + V->size[0] * ix];
            rx = X_data[ix - 1];
            if (Xq_data[k] == rx) {
                qx1 = zx1y1;
                qx2 = zx1y2;
            } else if (!(Xq_data[k] == X_data[ix])) {
                rx = ((float)Xq_data[k] - rx) / (X_data[ix] - rx);
                if (zx1y1 == qx1) {
                    qx1 = zx1y1;
                } else {
                    qx1 = (1.0F - rx) * zx1y1 + rx * qx1;
                }

                if (zx1y2 == qx2) {
                    qx2 = zx1y2;
                } else {
                    qx2 = (1.0F - rx) * zx1y2 + rx * qx2;
                }
            }

            rx = Y_data[iy - 1];
            if ((Yq_data[k] == rx) || (qx1 == qx2)) {
                Vq_data[k] = qx1;
            } else if (Yq_data[k] == Y_data[iy]) {
                Vq_data[k] = qx2;
            } else {
                rx = ((float)Yq_data[k] - rx) / (Y_data[iy] - rx);
                Vq_data[k] = (1.0F - rx) * qx1 + rx * qx2;
            }
        } else {
            Vq_data[k] = 0.0F;
        }
    }
}

/**
 * @fn             : inv
 * @brief          :
 * @param          : const double x[9]
 *                   double y[9]
 * @return         : void
 */
static void inv(const double x[9], double y[9]) {
    double b_x[9];
    double absx11;
    double absx21;
    double absx31;
    int p1;
    int p2;
    int p3;
    memcpy(&b_x[0], &x[0], 9U * sizeof(double));
    p1 = 0;
    p2 = 3;
    p3 = 6;
    absx11 = fabs(x[0]);
    absx21 = fabs(x[1]);
    absx31 = fabs(x[2]);
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
    if (fabs(b_x[5]) > fabs(b_x[4])) {
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

/**
 * @fn             : matchFeatures
 * @brief          :
 * @param          : const emxArray_uint8_T *varargin_1_Features
 *                   const emxArray_uint8_T *varargin_2_Features
 *                   emxArray_uint32_T *indexPairs
 * @return         : void
 */
static void matchFeatures(const emxArray_uint8_T *varargin_1_Features, const emxArray_uint8_T *varargin_2_Features, emxArray_uint32_T *indexPairs) {
    static float lookupTable[256];
    emxArray_boolean_T *inds;
    emxArray_int32_T *idx;
    emxArray_int32_T *r;
    emxArray_int32_T *r1;
    emxArray_int32_T *r2;
    emxArray_int32_T *r3;
    emxArray_real32_T *b_matchMetric;
    emxArray_real32_T *ex;
    emxArray_real32_T *matchMetric;
    emxArray_real32_T *scores;
    emxArray_uint32_T *b_indexPairs;
    emxArray_uint32_T *c_indexPairs;
    emxArray_uint8_T *features1in;
    emxArray_uint8_T *features2in;
    float *b_matchMetric_data;
    float *matchMetric_data;
    float *scores_data;
    int b_i;
    int i;
    int i1;
    int k;
    int loop_ub;
    int n;
    int olddi;
    unsigned int *b_indexPairs_data;
    int *idx_data;
    unsigned int *indexPairs_data;
    const unsigned char *varargin_1_Features_data;
    const unsigned char *varargin_2_Features_data;
    unsigned char *features1in_data;
    unsigned char *features2in_data;
    bool *inds_data;
    varargin_2_Features_data = varargin_2_Features->data;
    varargin_1_Features_data = varargin_1_Features->data;
    emxInit_uint8_T(&features1in);
    i = features1in->size[0] * features1in->size[1];
    features1in->size[0] = 32;
    features1in->size[1] = varargin_1_Features->size[0];
    emxEnsureCapacity_uint8_T(features1in, i);
    features1in_data = features1in->data;
    loop_ub = varargin_1_Features->size[0];
    for (i = 0; i < loop_ub; i++) {
        for (i1 = 0; i1 < 32; i1++) {
            features1in_data[i1 + 32 * i] = varargin_1_Features_data[i +
                                                                     varargin_1_Features->size[0] * i1];
        }
    }

    emxInit_uint8_T(&features2in);
    i = features2in->size[0] * features2in->size[1];
    features2in->size[0] = 32;
    features2in->size[1] = varargin_2_Features->size[0];
    emxEnsureCapacity_uint8_T(features2in, i);
    features2in_data = features2in->data;
    loop_ub = varargin_2_Features->size[0];
    for (i = 0; i < loop_ub; i++) {
        for (i1 = 0; i1 < 32; i1++) {
            features2in_data[i1 + 32 * i] = varargin_2_Features_data[i +
                                                                     varargin_2_Features->size[0] * i1];
        }
    }

    emxInit_real32_T(&scores, 2);
    emxInit_uint32_T(&b_indexPairs);
    emxInit_real32_T(&matchMetric, 2);
    emxInit_int32_T(&r, 2);
    emxInit_boolean_T(&inds, 2);
    emxInit_int32_T(&r1, 2);
    emxInit_int32_T(&r2, 2);
    emxInit_int32_T(&r3, 2);
    emxInit_real32_T(&ex, 2);
    emxInit_int32_T(&idx, 2);
    emxInit_uint32_T(&c_indexPairs);
    emxInit_real32_T(&b_matchMetric, 2);
    if ((features1in->size[1] == 0) || (features2in->size[1] == 0)) {
        indexPairs->size[0] = 0;
        indexPairs->size[1] = 2;
    } else {
        float y;
        i = scores->size[0] * scores->size[1];
        scores->size[0] = features1in->size[1];
        scores->size[1] = features2in->size[1];
        emxEnsureCapacity_real32_T(scores, i);
        scores_data = scores->data;
        if (!lookupTable_not_empty) {
            lookupTable_not_empty = true;
            for (b_i = 0; b_i < 256; b_i++) {
                double d;
                char s_data[64];
                char sfull[64];
                signed char x_data[64];
                bool exitg1;
                lookupTable[b_i] = 0.0F;
                d = b_i;
                for (i = 0; i < 64; i++) {
                    sfull[i] = '0';
                }

                loop_ub = 64;
                exitg1 = false;
                while ((!exitg1) && (loop_ub > 0)) {
                    olddi = (int)d;
                    d /= 2.0;
                    d = floor(d);
                    if (((int)d << 1) < olddi) {
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
                    x_data[i] = (signed char)(s_data[i] - 48);
                }

                d = x_data[0];
                for (k = 2; k <= loop_ub; k++) {
                    d += (double)x_data[k - 1];
                }

                lookupTable[b_i] = (float)d;
            }
        }

        i = features2in->size[1];
        i1 = features1in->size[1];
        for (olddi = 0; olddi < i; olddi++) {
            for (n = 0; n < i1; n++) {
                short b_idx[32];
                for (b_i = 0; b_i < 32; b_i++) {
                    unsigned char varargin_1;
                    unsigned char varargin_2;
                    varargin_1 = features1in_data[b_i + 32 * n];
                    varargin_2 = features2in_data[b_i + 32 * olddi];
                    b_idx[b_i] = (short)((unsigned char)(varargin_1 ^ varargin_2) + 1);
                }

                y = lookupTable[b_idx[0] - 1];
                for (k = 0; k < 31; k++) {
                    y += lookupTable[b_idx[k + 1] - 1];
                }

                scores_data[n + scores->size[0] * olddi] = y;
            }
        }

        findNearestNeighbors(scores, b_indexPairs, matchMetric);
        matchMetric_data = matchMetric->data;
        indexPairs_data = b_indexPairs->data;
        i = inds->size[0] * inds->size[1];
        inds->size[0] = 1;
        inds->size[1] = matchMetric->size[1];
        emxEnsureCapacity_boolean_T(inds, i);
        inds_data = inds->data;
        loop_ub = matchMetric->size[1];
        for (i = 0; i < loop_ub; i++) {
            inds_data[i] = (matchMetric_data[matchMetric->size[0] * i] <= 128.0F);
        }

        n = inds->size[1] - 1;
        olddi = 0;
        for (b_i = 0; b_i <= n; b_i++) {
            if (inds_data[b_i]) {
                olddi++;
            }
        }

        i = idx->size[0] * idx->size[1];
        idx->size[0] = 1;
        idx->size[1] = olddi;
        emxEnsureCapacity_int32_T(idx, i);
        idx_data = idx->data;
        olddi = 0;
        for (b_i = 0; b_i <= n; b_i++) {
            if (inds_data[b_i]) {
                idx_data[olddi] = b_i + 1;
                olddi++;
            }
        }

        i = c_indexPairs->size[0] * c_indexPairs->size[1];
        c_indexPairs->size[0] = 2;
        c_indexPairs->size[1] = idx->size[1];
        emxEnsureCapacity_uint32_T(c_indexPairs, i);
        b_indexPairs_data = c_indexPairs->data;
        loop_ub = idx->size[1];
        for (i = 0; i < loop_ub; i++) {
            b_indexPairs_data[2 * i] = indexPairs_data[2 * (idx_data[i] - 1)];
            b_indexPairs_data[2 * i + 1] = indexPairs_data[2 * (idx_data[i] - 1) + 1];
        }

        i = b_indexPairs->size[0] * b_indexPairs->size[1];
        b_indexPairs->size[0] = 2;
        b_indexPairs->size[1] = c_indexPairs->size[1];
        emxEnsureCapacity_uint32_T(b_indexPairs, i);
        indexPairs_data = b_indexPairs->data;
        loop_ub = 2 * c_indexPairs->size[1];
        for (i = 0; i < loop_ub; i++) {
            indexPairs_data[i] = b_indexPairs_data[i];
        }

        n = inds->size[1] - 1;
        olddi = 0;
        for (b_i = 0; b_i <= n; b_i++) {
            if (inds_data[b_i]) {
                olddi++;
            }
        }

        i = r1->size[0] * r1->size[1];
        r1->size[0] = 1;
        r1->size[1] = olddi;
        emxEnsureCapacity_int32_T(r1, i);
        idx_data = r1->data;
        olddi = 0;
        for (b_i = 0; b_i <= n; b_i++) {
            if (inds_data[b_i]) {
                idx_data[olddi] = b_i + 1;
                olddi++;
            }
        }

        olddi = matchMetric->size[0];
        i = b_matchMetric->size[0] * b_matchMetric->size[1];
        b_matchMetric->size[0] = matchMetric->size[0];
        b_matchMetric->size[1] = r1->size[1];
        emxEnsureCapacity_real32_T(b_matchMetric, i);
        b_matchMetric_data = b_matchMetric->data;
        loop_ub = r1->size[1];
        for (i = 0; i < loop_ub; i++) {
            for (i1 = 0; i1 < olddi; i1++) {
                b_matchMetric_data[i1 + b_matchMetric->size[0] * i] =
                    matchMetric_data[i1 + matchMetric->size[0] * (idx_data[i] - 1)];
            }
        }

        i = matchMetric->size[0] * matchMetric->size[1];
        matchMetric->size[0] = b_matchMetric->size[0];
        matchMetric->size[1] = b_matchMetric->size[1];
        emxEnsureCapacity_real32_T(matchMetric, i);
        matchMetric_data = matchMetric->data;
        loop_ub = b_matchMetric->size[0] * b_matchMetric->size[1];
        for (i = 0; i < loop_ub; i++) {
            matchMetric_data[i] = b_matchMetric_data[i];
        }

        if (features2in->size[1] > 1) {
            i = inds->size[0] * inds->size[1];
            inds->size[0] = 1;
            inds->size[1] = matchMetric->size[1];
            emxEnsureCapacity_boolean_T(inds, i);
            inds_data = inds->data;
            loop_ub = matchMetric->size[1];
            for (i = 0; i < loop_ub; i++) {
                inds_data[i] = (matchMetric_data[matchMetric->size[0] * i + 1] < 1.0E-6F);
            }

            n = inds->size[1] - 1;
            olddi = 0;
            for (b_i = 0; b_i <= n; b_i++) {
                if (inds_data[b_i]) {
                    olddi++;
                }
            }

            i = r3->size[0] * r3->size[1];
            r3->size[0] = 1;
            r3->size[1] = olddi;
            emxEnsureCapacity_int32_T(r3, i);
            idx_data = r3->data;
            olddi = 0;
            for (b_i = 0; b_i <= n; b_i++) {
                if (inds_data[b_i]) {
                    idx_data[olddi] = b_i + 1;
                    olddi++;
                }
            }

            loop_ub = r3->size[1];
            for (i = 0; i < loop_ub; i++) {
                olddi = matchMetric->size[0];
                for (i1 = 0; i1 < olddi; i1++) {
                    matchMetric_data[i1 + matchMetric->size[0] * (idx_data[i] - 1)] = 1.0F;
                }
            }

            i = inds->size[0] * inds->size[1];
            inds->size[0] = 1;
            inds->size[1] = matchMetric->size[1];
            emxEnsureCapacity_boolean_T(inds, i);
            inds_data = inds->data;
            loop_ub = matchMetric->size[1];
            for (i = 0; i < loop_ub; i++) {
                inds_data[i] = (matchMetric_data[matchMetric->size[0] * i] /
                                    matchMetric_data[matchMetric->size[0] * i + 1] <=
                                0.5F);
            }
        } else {
            i = inds->size[0] * inds->size[1];
            inds->size[0] = 1;
            inds->size[1] = matchMetric->size[1];
            emxEnsureCapacity_boolean_T(inds, i);
            inds_data = inds->data;
            loop_ub = matchMetric->size[1];
            for (i = 0; i < loop_ub; i++) {
                inds_data[i] = true;
            }
        }

        n = inds->size[1] - 1;
        olddi = 0;
        for (b_i = 0; b_i <= n; b_i++) {
            if (inds_data[b_i]) {
                olddi++;
            }
        }

        i = r2->size[0] * r2->size[1];
        r2->size[0] = 1;
        r2->size[1] = olddi;
        emxEnsureCapacity_int32_T(r2, i);
        idx_data = r2->data;
        olddi = 0;
        for (b_i = 0; b_i <= n; b_i++) {
            if (inds_data[b_i]) {
                idx_data[olddi] = b_i + 1;
                olddi++;
            }
        }

        i = c_indexPairs->size[0] * c_indexPairs->size[1];
        c_indexPairs->size[0] = 2;
        c_indexPairs->size[1] = r2->size[1];
        emxEnsureCapacity_uint32_T(c_indexPairs, i);
        b_indexPairs_data = c_indexPairs->data;
        loop_ub = r2->size[1];
        for (i = 0; i < loop_ub; i++) {
            b_indexPairs_data[2 * i] = indexPairs_data[2 * (idx_data[i] - 1)];
            b_indexPairs_data[2 * i + 1] = indexPairs_data[2 * (idx_data[i] - 1) + 1];
        }

        i = b_indexPairs->size[0] * b_indexPairs->size[1];
        b_indexPairs->size[0] = 2;
        b_indexPairs->size[1] = c_indexPairs->size[1];
        emxEnsureCapacity_uint32_T(b_indexPairs, i);
        indexPairs_data = b_indexPairs->data;
        loop_ub = 2 * c_indexPairs->size[1];
        for (i = 0; i < loop_ub; i++) {
            indexPairs_data[i] = b_indexPairs_data[i];
        }

        if (b_indexPairs->size[1] == 0) {
            c_indexPairs->size[0] = 2;
            c_indexPairs->size[1] = 0;
        } else {
            olddi = scores->size[0];
            n = b_indexPairs->size[1];
            i = ex->size[0] * ex->size[1];
            ex->size[0] = 1;
            ex->size[1] = b_indexPairs->size[1];
            emxEnsureCapacity_real32_T(ex, i);
            matchMetric_data = ex->data;
            i = idx->size[0] * idx->size[1];
            idx->size[0] = 1;
            idx->size[1] = b_indexPairs->size[1];
            emxEnsureCapacity_int32_T(idx, i);
            idx_data = idx->data;
            loop_ub = b_indexPairs->size[1];
            for (i = 0; i < loop_ub; i++) {
                idx_data[i] = 1;
            }

            for (loop_ub = 0; loop_ub < n; loop_ub++) {
                i = 2 * loop_ub + 1;
                matchMetric_data[loop_ub] = scores_data[scores->size[0] * ((int)
                                                                               indexPairs_data[i] -
                                                                           1)];
                for (b_i = 2; b_i <= olddi; b_i++) {
                    float b_tmp;
                    bool p;
                    y = matchMetric_data[loop_ub];
                    b_tmp = scores_data[(b_i + scores->size[0] * ((int)indexPairs_data[i] - 1)) - 1];
                    if (rtIsNaNF(b_tmp)) {
                        p = false;
                    } else if (rtIsNaNF(y)) {
                        p = true;
                    } else {
                        p = (y > b_tmp);
                    }

                    if (p) {
                        matchMetric_data[loop_ub] = b_tmp;
                        idx_data[loop_ub] = b_i;
                    }
                }
            }

            if (idx->size[1] == b_indexPairs->size[1]) {
                i = inds->size[0] * inds->size[1];
                inds->size[0] = 1;
                inds->size[1] = idx->size[1];
                emxEnsureCapacity_boolean_T(inds, i);
                inds_data = inds->data;
                loop_ub = idx->size[1];
                for (i = 0; i < loop_ub; i++) {
                    inds_data[i] = ((unsigned int)idx_data[i] == indexPairs_data[2 * i]);
                }
            } else {
                c_binary_expand_op(inds, idx, b_indexPairs);
                inds_data = inds->data;
            }

            n = inds->size[1] - 1;
            olddi = 0;
            for (b_i = 0; b_i <= n; b_i++) {
                if (inds_data[b_i]) {
                    olddi++;
                }
            }

            i = r->size[0] * r->size[1];
            r->size[0] = 1;
            r->size[1] = olddi;
            emxEnsureCapacity_int32_T(r, i);
            idx_data = r->data;
            olddi = 0;
            for (b_i = 0; b_i <= n; b_i++) {
                if (inds_data[b_i]) {
                    idx_data[olddi] = b_i + 1;
                    olddi++;
                }
            }

            i = c_indexPairs->size[0] * c_indexPairs->size[1];
            c_indexPairs->size[0] = 2;
            c_indexPairs->size[1] = r->size[1];
            emxEnsureCapacity_uint32_T(c_indexPairs, i);
            b_indexPairs_data = c_indexPairs->data;
            loop_ub = r->size[1];
            for (i = 0; i < loop_ub; i++) {
                b_indexPairs_data[2 * i] = indexPairs_data[2 * (idx_data[i] - 1)];
                b_indexPairs_data[2 * i + 1] = indexPairs_data[2 * (idx_data[i] - 1) + 1];
            }
        }

        i = indexPairs->size[0] * indexPairs->size[1];
        indexPairs->size[0] = c_indexPairs->size[1];
        indexPairs->size[1] = 2;
        emxEnsureCapacity_uint32_T(indexPairs, i);
        indexPairs_data = indexPairs->data;
        loop_ub = c_indexPairs->size[1];
        for (i = 0; i < 2; i++) {
            for (i1 = 0; i1 < loop_ub; i1++) {
                indexPairs_data[i1 + indexPairs->size[0] * i] = b_indexPairs_data[i + 2 *
                                                                                          i1];
            }
        }
    }

    emxFree_real32_T(&b_matchMetric);
    emxFree_uint32_T(&c_indexPairs);
    emxFree_int32_T(&idx);
    emxFree_real32_T(&ex);
    emxFree_int32_T(&r3);
    emxFree_int32_T(&r2);
    emxFree_int32_T(&r1);
    emxFree_boolean_T(&inds);
    emxFree_int32_T(&r);
    emxFree_real32_T(&matchMetric);
    emxFree_uint32_T(&b_indexPairs);
    emxFree_real32_T(&scores);
    emxFree_uint8_T(&features2in);
    emxFree_uint8_T(&features1in);
}

/**
 * @fn             : maximum
 * @brief          :
 * @param          : const float x[9]
 * @return         : float
 */
static float maximum(const float x[9]) {
    float ex;
    int idx;
    int k;
    if (!rtIsNaNF(x[0])) {
        idx = 1;
    } else {
        bool exitg1;
        idx = 0;
        k = 2;
        exitg1 = false;
        while ((!exitg1) && (k < 10)) {
            if (!rtIsNaNF(x[k - 1])) {
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
            float f;
            f = x[k - 1];
            if (ex < f) {
                ex = f;
            }
        }
    }

    return ex;
}

/**
 * @fn             : mean
 * @brief          :
 * @param          : const double x[8]
 *                   double y[2]
 * @return         : void
 */
static void mean(const double x[8], double y[2]) {
    int xi;
    for (xi = 0; xi < 2; xi++) {
        int xpageoffset;
        xpageoffset = xi << 2;
        y[xi] = (((x[xpageoffset] + x[xpageoffset + 1]) + x[xpageoffset + 2]) +
                 x[xpageoffset + 3]) /
                4.0;
    }
}

/**
 * @fn             : merge
 * @brief          :
 * @param          : emxArray_int32_T *idx
 *                   emxArray_real32_T *x
 *                   int offset
 *                   int np
 *                   int nq
 *                   emxArray_int32_T *iwork
 *                   emxArray_real32_T *xwork
 * @return         : void
 */
static void merge(emxArray_int32_T *idx, emxArray_real32_T *x, int offset, int np, int nq, emxArray_int32_T *iwork, emxArray_real32_T *xwork) {
    float *x_data;
    float *xwork_data;
    int j;
    int *idx_data;
    int *iwork_data;
    xwork_data = xwork->data;
    iwork_data = iwork->data;
    x_data = x->data;
    idx_data = idx->data;
    if (nq != 0) {
        int iout;
        int n_tmp;
        int p;
        int q;
        n_tmp = np + nq;
        for (j = 0; j < n_tmp; j++) {
            iout = offset + j;
            iwork_data[j] = idx_data[iout];
            xwork_data[j] = x_data[iout];
        }

        p = 0;
        q = np;
        iout = offset - 1;
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
                if (q + 1 < n_tmp) {
                    q++;
                } else {
                    q = iout - p;
                    for (j = p + 1; j <= np; j++) {
                        iout = q + j;
                        idx_data[iout] = iwork_data[j - 1];
                        x_data[iout] = xwork_data[j - 1];
                    }

                    exitg1 = 1;
                }
            }
        } while (exitg1 == 0);
    }
}

/**
 * @fn             : merge_block
 * @brief          :
 * @param          : emxArray_int32_T *idx
 *                   emxArray_real32_T *x
 *                   int offset
 *                   int n
 *                   int preSortLevel
 *                   emxArray_int32_T *iwork
 *                   emxArray_real32_T *xwork
 * @return         : void
 */
static void merge_block(emxArray_int32_T *idx, emxArray_real32_T *x, int offset,
                        int n, int preSortLevel, emxArray_int32_T *iwork, emxArray_real32_T *xwork) {
    int bLen;
    int nPairs;
    int nTail;
    nPairs = n >> preSortLevel;
    bLen = 1 << preSortLevel;
    while (nPairs > 1) {
        int tailOffset;
        if ((nPairs & 1) != 0) {
            nPairs--;
            tailOffset = bLen * nPairs;
            nTail = n - tailOffset;
            if (nTail > bLen) {
                merge(idx, x, offset + tailOffset, bLen, nTail - bLen, iwork, xwork);
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

/**
 * @fn             : meshgrid
 * @brief          :
 * @param          : const double x[3]
 *                   const double y[3]
 *                   double xx[9]
 *                   double yy[9]
 * @return         : void
 */
static void meshgrid(const double x[3], const double y[3], double xx[9], double yy[9]) {
    double d;
    double d1;
    double d2;
    int j;
    d = y[0];
    d1 = y[1];
    d2 = y[2];
    for (j = 0; j < 3; j++) {
        double d3;
        int xx_tmp;
        d3 = x[j];
        xx[3 * j] = d3;
        yy[3 * j] = d;
        xx_tmp = 3 * j + 1;
        xx[xx_tmp] = d3;
        yy[xx_tmp] = d1;
        xx_tmp = 3 * j + 2;
        xx[xx_tmp] = d3;
        yy[xx_tmp] = d2;
    }
}

/**
 * @fn             : minimum
 * @brief          :
 * @param          : const float x[9]
 * @return         : float
 */
static float minimum(const float x[9]) {
    float ex;
    int idx;
    int k;
    if (!rtIsNaNF(x[0])) {
        idx = 1;
    } else {
        bool exitg1;
        idx = 0;
        k = 2;
        exitg1 = false;
        while ((!exitg1) && (k < 10)) {
            if (!rtIsNaNF(x[k - 1])) {
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
            float f;
            f = x[k - 1];
            if (ex > f) {
                ex = f;
            }
        }
    }

    return ex;
}

/**
 * @fn             : msac
 * @brief          :
 * @param          : const emxArray_real32_T *allPoints
 *                   bool *isFound
 *                   float bestModelParams_data[]
 *                   int bestModelParams_size[2]
 *                   emxArray_boolean_T *inliers
 * @return         : void
 */
static void msac(const emxArray_real32_T *allPoints, bool *isFound, float bestModelParams_data[], int bestModelParams_size[2],
                 emxArray_boolean_T *inliers) {
    emxArray_boolean_T *bestInliers;
    emxArray_boolean_T *x;
    emxArray_int32_T *r;
    emxArray_real32_T b_samplePoints_data;
    emxArray_real32_T *b_allPoints;
    emxArray_real32_T *dis;
    float modelParams[9];
    float samplePoints_data[8];
    const float *allPoints_data;
    float bestDis;
    float inlierProbability;
    float *dis_data;
    int samplePoints_size[3];
    int ib;
    int idxTrial;
    int jlast;
    int k;
    int lastBlockLength;
    int nblocks;
    int numPts;
    int numTrials;
    int nz;
    int skipTrials;
    int *r1;
    bool b_data[9];
    bool tmp_data[9];
    bool exitg1;
    bool isValidModel;
    bool *bestInliers_data;
    bool *x_data;
    allPoints_data = allPoints->data;
    numPts = allPoints->size[0];
    idxTrial = 1;
    numTrials = 1000;
    bestDis = 1.5F * (float)allPoints->size[0];
    bestModelParams_size[0] = 0;
    bestModelParams_size[1] = 0;
    skipTrials = 0;
    emxInit_boolean_T(&bestInliers, 1);
    lastBlockLength = bestInliers->size[0];
    bestInliers->size[0] = allPoints->size[0];
    emxEnsureCapacity_boolean_T(bestInliers, lastBlockLength);
    bestInliers_data = bestInliers->data;
    jlast = allPoints->size[0];
    for (lastBlockLength = 0; lastBlockLength < jlast; lastBlockLength++) {
        bestInliers_data[lastBlockLength] = false;
    }

    emxInit_boolean_T(&x, 1);
    emxInit_real32_T(&dis, 1);
    while ((idxTrial <= numTrials) && (skipTrials < 10000)) {
        double indices_data[2];
        double j;
        double selectedLoc;
        indices_data[1] = 0.0;
        if (numPts <= 2) {
            indices_data[0] = 1.0;
            j = b_rand() * 2.0;
            j = floor(j);
            indices_data[1] = indices_data[(int)(j + 1.0) - 1];
            indices_data[(int)(j + 1.0) - 1] = 2.0;
        } else if ((double)numPts / 4.0 <= 2.0) {
            double loc_data_idx_0;
            double t;
            t = 0.0;
            selectedLoc = numPts;
            loc_data_idx_0 = 2.0 / (double)numPts;
            j = b_rand();
            while (j > loc_data_idx_0) {
                t++;
                selectedLoc--;
                loc_data_idx_0 += (1.0 - loc_data_idx_0) * (2.0 / selectedLoc);
            }

            t++;
            j = b_rand();
            j = floor(j);
            indices_data[0] = 0.0;
            indices_data[(int)(j + 1.0) - 1] = t;
            selectedLoc = (double)numPts - t;
            loc_data_idx_0 = 1.0 / selectedLoc;
            j = b_rand();
            while (j > loc_data_idx_0) {
                t++;
                selectedLoc--;
                loc_data_idx_0 += (1.0 - loc_data_idx_0) * (1.0 / selectedLoc);
            }

            t++;
            j = b_rand() * 2.0;
            j = floor(j);
            indices_data[1] = indices_data[(int)(j + 1.0) - 1];
            indices_data[(int)(j + 1.0) - 1] = t;
        } else {
            double loc_data_idx_0;
            signed char hashTbl_data[2];
            hashTbl_data[0] = 0;
            hashTbl_data[1] = 0;
            selectedLoc = b_rand() * (((double)numPts - 1.0) + 1.0);
            selectedLoc = floor(selectedLoc);
            if (rtIsNaN(selectedLoc) || rtIsInf(selectedLoc)) {
                j = rtNaN;
            } else if (selectedLoc == 0.0) {
                j = 0.0;
            } else {
                j = fmod(selectedLoc, 2.0);
                if (j == 0.0) {
                    j = 0.0;
                }
            }

            indices_data[0] = selectedLoc + 1.0;
            loc_data_idx_0 = selectedLoc;
            hashTbl_data[(int)(j + 1.0) - 1] = 1;
            jlast = hashTbl_data[(int)fmod((double)numPts - 1.0, 2.0)];
            while ((jlast > 0) && (selectedLoc != (double)numPts - 1.0)) {
                jlast = 0;
            }

            if (jlast > 0) {
                nz = 0;
            } else {
                nz = numPts - 1;
            }

            selectedLoc = b_rand() * (((double)numPts - 2.0) + 1.0);
            selectedLoc = floor(selectedLoc);
            if (rtIsNaN(selectedLoc) || rtIsInf(selectedLoc)) {
                j = rtNaN;
            } else if (selectedLoc == 0.0) {
                j = 0.0;
            } else {
                j = fmod(selectedLoc, 2.0);
                if (j == 0.0) {
                    j = 0.0;
                }
            }

            j = hashTbl_data[(int)(j + 1.0) - 1];
            while ((j > 0.0) && (loc_data_idx_0 != selectedLoc)) {
                j = 0.0;
            }

            if (j > 0.0) {
                indices_data[1] = (double)nz + 1.0;
            } else {
                indices_data[1] = selectedLoc + 1.0;
            }
        }

        samplePoints_size[0] = 2;
        samplePoints_size[1] = 2;
        samplePoints_size[2] = 2;
        j = indices_data[0];
        selectedLoc = indices_data[1];
        for (lastBlockLength = 0; lastBlockLength < 2; lastBlockLength++) {
            samplePoints_data[4 * lastBlockLength] = allPoints_data[((int)j +
                                                                     allPoints->size[0] * 2 * lastBlockLength) -
                                                                    1];
            samplePoints_data[4 * lastBlockLength + 1] = allPoints_data[((int)
                                                                             selectedLoc +
                                                                         allPoints->size[0] * 2 * lastBlockLength) -
                                                                        1];
            samplePoints_data[4 * lastBlockLength + 2] = allPoints_data[(((int)j +
                                                                          allPoints->size[0]) +
                                                                         allPoints->size[0] * 2 * lastBlockLength) -
                                                                        1];
            samplePoints_data[4 * lastBlockLength + 3] = allPoints_data[(((int)
                                                                              selectedLoc +
                                                                          allPoints->size[0]) +
                                                                         allPoints->size[0] * 2 *
                                                                             lastBlockLength) -
                                                                        1];
        }

        b_samplePoints_data.data = &samplePoints_data[0];
        b_samplePoints_data.size = &samplePoints_size[0];
        b_samplePoints_data.allocatedSize = 8;
        b_samplePoints_data.numDimensions = 3;
        b_samplePoints_data.canFreeData = false;
        computeRigid2d(&b_samplePoints_data, modelParams);
        for (lastBlockLength = 0; lastBlockLength < 9; lastBlockLength++) {
            inlierProbability = modelParams[lastBlockLength];
            b_data[lastBlockLength] = rtIsInfF(inlierProbability);
            tmp_data[lastBlockLength] = rtIsNaNF(inlierProbability);
        }

        isValidModel = true;
        k = 0;
        exitg1 = false;
        while ((!exitg1) && (k < 9)) {
            if (b_data[k] || tmp_data[k]) {
                isValidModel = false;
                exitg1 = true;
            } else {
                k++;
            }
        }

        if (isValidModel) {
            evaluateTform2d(modelParams, allPoints, dis);
            dis_data = dis->data;
            nz = dis->size[0];
            for (nblocks = 0; nblocks < nz; nblocks++) {
                if (dis_data[nblocks] > 1.5F) {
                    dis_data[nblocks] = 1.5F;
                }
            }

            if (dis->size[0] == 0) {
                inlierProbability = 0.0F;
            } else {
                if (dis->size[0] <= 1024) {
                    jlast = dis->size[0];
                    lastBlockLength = 0;
                    nblocks = 1;
                } else {
                    jlast = 1024;
                    nblocks = (int)((unsigned int)dis->size[0] >> 10);
                    lastBlockLength = dis->size[0] - (nblocks << 10);
                    if (lastBlockLength > 0) {
                        nblocks++;
                    } else {
                        lastBlockLength = 1024;
                    }
                }

                inlierProbability = dis_data[0];
                for (k = 2; k <= jlast; k++) {
                    inlierProbability += dis_data[k - 1];
                }

                for (ib = 2; ib <= nblocks; ib++) {
                    float bsum;
                    jlast = (ib - 1) << 10;
                    bsum = dis_data[jlast];
                    if (ib == nblocks) {
                        nz = lastBlockLength;
                    } else {
                        nz = 1024;
                    }

                    for (k = 2; k <= nz; k++) {
                        bsum += dis_data[(jlast + k) - 1];
                    }

                    inlierProbability += bsum;
                }
            }

            if (inlierProbability < bestDis) {
                bestDis = inlierProbability;
                lastBlockLength = bestInliers->size[0];
                bestInliers->size[0] = dis->size[0];
                emxEnsureCapacity_boolean_T(bestInliers, lastBlockLength);
                bestInliers_data = bestInliers->data;
                jlast = dis->size[0];
                for (lastBlockLength = 0; lastBlockLength < jlast; lastBlockLength++) {
                    bestInliers_data[lastBlockLength] = (dis_data[lastBlockLength] < 1.5F);
                }

                bestModelParams_size[0] = 3;
                bestModelParams_size[1] = 3;
                for (lastBlockLength = 0; lastBlockLength < 9; lastBlockLength++) {
                    bestModelParams_data[lastBlockLength] = modelParams[lastBlockLength];
                }

                lastBlockLength = x->size[0];
                x->size[0] = dis->size[0];
                emxEnsureCapacity_boolean_T(x, lastBlockLength);
                x_data = x->data;
                jlast = dis->size[0];
                for (lastBlockLength = 0; lastBlockLength < jlast; lastBlockLength++) {
                    x_data[lastBlockLength] = (dis_data[lastBlockLength] < 1.5F);
                }

                jlast = x->size[0];
                if (x->size[0] == 0) {
                    nz = 0;
                } else {
                    nz = x_data[0];
                    for (k = 2; k <= jlast; k++) {
                        nz += x_data[k - 1];
                    }
                }

                inlierProbability = rt_powf_snf((float)nz / (float)numPts, 2.0F);
                if (inlierProbability < 1.1920929E-7F) {
                    jlast = MAX_int32_T;
                } else {
                    jlast = (int)ceilf(-1.99999785F / log10f(1.0F - inlierProbability));
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

    emxFree_boolean_T(&x);
    nz = bestModelParams_size[0] * bestModelParams_size[1];
    for (lastBlockLength = 0; lastBlockLength < nz; lastBlockLength++) {
        b_data[lastBlockLength] = rtIsInfF(bestModelParams_data[lastBlockLength]);
    }

    for (lastBlockLength = 0; lastBlockLength < nz; lastBlockLength++) {
        tmp_data[lastBlockLength] = rtIsNaNF(bestModelParams_data[lastBlockLength]);
    }

    for (lastBlockLength = 0; lastBlockLength < nz; lastBlockLength++) {
        b_data[lastBlockLength] = ((!b_data[lastBlockLength]) &&
                                   (!tmp_data[lastBlockLength]));
    }

    isValidModel = true;
    jlast = 1;
    exitg1 = false;
    while ((!exitg1) && (jlast <= nz)) {
        if (!b_data[jlast - 1]) {
            isValidModel = false;
            exitg1 = true;
        } else {
            jlast++;
        }
    }

    if (isValidModel && (bestInliers->size[0] != 0)) {
        jlast = bestInliers->size[0];
        nz = bestInliers_data[0];
        for (k = 2; k <= jlast; k++) {
            nz += bestInliers_data[k - 1];
        }

        if (nz >= 2) {
            *isFound = true;
        } else {
            *isFound = false;
        }
    } else {
        *isFound = false;
    }

    emxInit_int32_T(&r, 1);
    emxInit_real32_T(&b_allPoints, 3);
    if (*isFound) {
        bool guard1 = false;
        nz = bestInliers->size[0] - 1;
        jlast = 0;
        for (nblocks = 0; nblocks <= nz; nblocks++) {
            if (bestInliers_data[nblocks]) {
                jlast++;
            }
        }

        lastBlockLength = r->size[0];
        r->size[0] = jlast;
        emxEnsureCapacity_int32_T(r, lastBlockLength);
        r1 = r->data;
        jlast = 0;
        for (nblocks = 0; nblocks <= nz; nblocks++) {
            if (bestInliers_data[nblocks]) {
                r1[jlast] = nblocks + 1;
                jlast++;
            }
        }

        lastBlockLength = b_allPoints->size[0] * b_allPoints->size[1] *
                          b_allPoints->size[2];
        b_allPoints->size[0] = r->size[0];
        b_allPoints->size[1] = 2;
        b_allPoints->size[2] = 2;
        emxEnsureCapacity_real32_T(b_allPoints, lastBlockLength);
        dis_data = b_allPoints->data;
        jlast = r->size[0];
        for (lastBlockLength = 0; lastBlockLength < 2; lastBlockLength++) {
            for (nz = 0; nz < 2; nz++) {
                for (nblocks = 0; nblocks < jlast; nblocks++) {
                    dis_data[(nblocks + b_allPoints->size[0] * nz) + b_allPoints->size[0] *
                                                                         2 * lastBlockLength] = allPoints_data[((r1[nblocks] +
                                                                                                                 allPoints->size[0] * nz) +
                                                                                                                allPoints->size[0] * 2 * lastBlockLength) -
                                                                                                               1];
                }
            }
        }

        computeRigid2d(b_allPoints, modelParams);
        evaluateTform2d(modelParams, allPoints, dis);
        dis_data = dis->data;
        nz = dis->size[0];
        for (nblocks = 0; nblocks < nz; nblocks++) {
            if (dis_data[nblocks] > 1.5F) {
                dis_data[nblocks] = 1.5F;
            }
        }

        bestModelParams_size[0] = 3;
        bestModelParams_size[1] = 3;
        for (lastBlockLength = 0; lastBlockLength < 9; lastBlockLength++) {
            bestModelParams_data[lastBlockLength] = modelParams[lastBlockLength];
        }

        lastBlockLength = inliers->size[0];
        inliers->size[0] = dis->size[0];
        emxEnsureCapacity_boolean_T(inliers, lastBlockLength);
        bestInliers_data = inliers->data;
        jlast = dis->size[0];
        for (lastBlockLength = 0; lastBlockLength < jlast; lastBlockLength++) {
            bestInliers_data[lastBlockLength] = (dis_data[lastBlockLength] < 1.5F);
        }

        for (lastBlockLength = 0; lastBlockLength < 9; lastBlockLength++) {
            inlierProbability = modelParams[lastBlockLength];
            b_data[lastBlockLength] = ((!rtIsInfF(inlierProbability)) && (!rtIsNaNF(inlierProbability)));
        }

        isValidModel = true;
        jlast = 1;
        exitg1 = false;
        while ((!exitg1) && (jlast <= 9)) {
            if (!b_data[jlast - 1]) {
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
            while ((!exitg1) && (jlast <= inliers->size[0])) {
                if (bestInliers_data[jlast - 1]) {
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
            lastBlockLength = inliers->size[0];
            inliers->size[0] = allPoints->size[0];
            emxEnsureCapacity_boolean_T(inliers, lastBlockLength);
            bestInliers_data = inliers->data;
            jlast = allPoints->size[0];
            for (lastBlockLength = 0; lastBlockLength < jlast; lastBlockLength++) {
                bestInliers_data[lastBlockLength] = false;
            }
        }
    } else {
        lastBlockLength = inliers->size[0];
        inliers->size[0] = allPoints->size[0];
        emxEnsureCapacity_boolean_T(inliers, lastBlockLength);
        bestInliers_data = inliers->data;
        jlast = allPoints->size[0];
        for (lastBlockLength = 0; lastBlockLength < jlast; lastBlockLength++) {
            bestInliers_data[lastBlockLength] = false;
        }
    }

    emxFree_real32_T(&b_allPoints);
    emxFree_real32_T(&dis);
    emxFree_int32_T(&r);
    emxFree_boolean_T(&bestInliers);
}

/**
 * @fn             : padarray
 * @brief          :
 * @param          : const emxArray_real_T *varargin_1
 *                   const double varargin_2[2]
 *                   emxArray_real_T *b
 * @return         : void
 */
static void padarray(const emxArray_real_T *varargin_1, const double varargin_2[2], emxArray_real_T *b) {
    const double *varargin_1_data;
    double padSize_idx_0;
    double padSize_idx_1;
    double *b_data;
    int b_i;
    int i;
    int j;
    int k;
    varargin_1_data = varargin_1->data;
    padSize_idx_0 = varargin_2[0];
    padSize_idx_1 = varargin_2[1];
    if ((varargin_1->size[0] == 0) || (varargin_1->size[1] == 0)) {
        int loop_ub;
        padSize_idx_0 = (double)varargin_1->size[0] + varargin_2[0];
        padSize_idx_1 = (double)varargin_1->size[1] + varargin_2[1];
        i = b->size[0] * b->size[1] * b->size[2];
        b->size[0] = (int)padSize_idx_0;
        b->size[1] = (int)padSize_idx_1;
        b->size[2] = 3;
        emxEnsureCapacity_real_T(b, i);
        b_data = b->data;
        loop_ub = (int)padSize_idx_0 * (int)padSize_idx_1 * 3;
        for (i = 0; i < loop_ub; i++) {
            b_data[i] = 0.0;
        }
    } else {
        int i1;
        int loop_ub;
        i = b->size[0] * b->size[1] * b->size[2];
        b->size[0] = (int)((double)varargin_1->size[0] + varargin_2[0]);
        b->size[1] = (int)((double)varargin_1->size[1] + varargin_2[1]);
        b->size[2] = 3;
        emxEnsureCapacity_real_T(b, i);
        b_data = b->data;
        i = (int)varargin_2[1];
        loop_ub = (int)varargin_2[1] + 1;
        i1 = b->size[1];
        for (k = 0; k < 3; k++) {
            int i2;
            for (j = 0; j < i; j++) {
                i2 = b->size[0];
                for (b_i = 0; b_i < i2; b_i++) {
                    b_data[(b_i + b->size[0] * j) + b->size[0] * b->size[1] * k] = 0.0;
                }
            }

            for (j = loop_ub; j <= i1; j++) {
                i2 = (int)padSize_idx_0;
                for (b_i = 0; b_i < i2; b_i++) {
                    b_data[(b_i + b->size[0] * (j - 1)) + b->size[0] * b->size[1] * k] =
                        0.0;
                }
            }
        }

        i = varargin_1->size[1];
        loop_ub = varargin_1->size[0];
        for (k = 0; k < 3; k++) {
            for (j = 0; j < i; j++) {
                for (b_i = 0; b_i < loop_ub; b_i++) {
                    b_data[((b_i + (int)padSize_idx_0) + b->size[0] * (j + (int)
                                                                               padSize_idx_1)) +
                           b->size[0] * b->size[1] * k] =
                        varargin_1_data[(b_i + varargin_1->size[0] * j) + varargin_1->size[0] * varargin_1->size[1] * k];
                }
            }
        }
    }
}

/**
 * @fn             : poly2edgelist
 * @brief          :
 * @param          : double x[9]
 *                   double y[9]
 *                   int M
 *                   int N
 *                   emxArray_boolean_T *out
 *                   emxArray_int32_T *minY
 *                   emxArray_int32_T *maxY
 * @return         : void
 */
static void poly2edgelist(double x[9], double y[9], int M, int N,
                          emxArray_boolean_T *out, emxArray_int32_T *minY, emxArray_int32_T *maxY) {
    emxArray_real_T *xLinePts;
    emxArray_real_T *yLinePts;
    double dy_tmp;
    double m;
    double xVal;
    double *xLinePts_data;
    double *yLinePts_data;
    int b_i;
    int borderPosition;
    int borderSize;
    int i;
    int tempDX;
    int tempDY;
    int *maxY_data;
    int *minY_data;
    bool *out_data;
    maxY_data = maxY->data;
    minY_data = minY->data;
    out_data = out->data;
    borderSize = 0;
    for (i = 0; i < 9; i++) {
        xVal = floor(5.0 * (x[i] - 0.5) + 0.5) + 1.0;
        x[i] = xVal;
        dy_tmp = floor(5.0 * (y[i] - 0.5) + 0.5) + 1.0;
        y[i] = dy_tmp;
        if (i + 1 > 1) {
            tempDX = (int)rt_roundd_snf(fabs(xVal - x[i - 1]));
            tempDY = (int)rt_roundd_snf(fabs(dy_tmp - y[i - 1]));
            if (tempDX >= tempDY) {
                borderSize = (borderSize + tempDX) + 1;
            } else {
                borderSize = (borderSize + tempDY) + 1;
            }
        }
    }

    emxInit_real_T(&yLinePts, 2);
    b_i = yLinePts->size[0] * yLinePts->size[1];
    yLinePts->size[0] = 1;
    yLinePts->size[1] = borderSize;
    emxEnsureCapacity_real_T(yLinePts, b_i);
    yLinePts_data = yLinePts->data;
    for (b_i = 0; b_i < borderSize; b_i++) {
        yLinePts_data[b_i] = 0.0;
    }

    emxInit_real_T(&xLinePts, 2);
    b_i = xLinePts->size[0] * xLinePts->size[1];
    xLinePts->size[0] = 1;
    xLinePts->size[1] = borderSize;
    emxEnsureCapacity_real_T(xLinePts, b_i);
    xLinePts_data = xLinePts->data;
    for (b_i = 0; b_i < borderSize; b_i++) {
        xLinePts_data[b_i] = 0.0;
    }

    borderPosition = 0;
    for (i = 0; i < 8; i++) {
        double b_y1;
        double dy_tmp_tmp;
        double x1;
        x1 = x[i];
        b_y1 = y[i];
        xVal = x[i + 1];
        m = xVal - x1;
        tempDX = (int)rt_roundd_snf(fabs(m));
        dy_tmp_tmp = y[i + 1];
        dy_tmp = dy_tmp_tmp - b_y1;
        tempDY = (int)rt_roundd_snf(fabs(dy_tmp));
        if ((tempDX == 0) && (tempDY == 0)) {
            xLinePts_data[borderPosition] = x1;
            yLinePts_data[borderPosition] = b_y1;
            borderPosition++;
        } else if (tempDX >= tempDY) {
            m = dy_tmp / m;
            if (xVal > x1) {
                b_i = (int)(xVal + (1.0 - x1));
                for (tempDY = 0; tempDY < b_i; tempDY++) {
                    xVal = x1 + (double)tempDY;
                    tempDX = borderPosition + tempDY;
                    yLinePts_data[tempDX] = rt_roundd_snf(b_y1 + m * (xVal - x1));
                    xLinePts_data[tempDX] = xVal;
                }

                borderPosition += b_i;
            } else {
                b_i = (int)-(xVal + (-1.0 - x1));
                for (tempDY = 0; tempDY < b_i; tempDY++) {
                    xVal = x1 - (double)tempDY;
                    tempDX = borderPosition + tempDY;
                    yLinePts_data[tempDX] = rt_roundd_snf(b_y1 + m * (xVal - x1));
                    xLinePts_data[tempDX] = xVal;
                }

                borderPosition += b_i;
            }
        } else {
            m /= dy_tmp;
            if (dy_tmp_tmp > b_y1) {
                b_i = (int)(dy_tmp_tmp + (1.0 - b_y1));
                for (tempDY = 0; tempDY < b_i; tempDY++) {
                    xVal = b_y1 + (double)tempDY;
                    tempDX = borderPosition + tempDY;
                    xLinePts_data[tempDX] = rt_roundd_snf(x1 + m * (xVal - b_y1));
                    yLinePts_data[tempDX] = xVal;
                }

                borderPosition += b_i;
            } else {
                b_i = (int)-(dy_tmp_tmp + (-1.0 - b_y1));
                for (tempDY = 0; tempDY < b_i; tempDY++) {
                    xVal = b_y1 - (double)tempDY;
                    tempDX = borderPosition + tempDY;
                    xLinePts_data[tempDX] = rt_roundd_snf(x1 + m * (xVal - b_y1));
                    yLinePts_data[tempDX] = xVal;
                }

                borderPosition += b_i;
            }
        }
    }

    for (borderPosition = 0; borderPosition <= borderSize - 2; borderPosition++) {
        xVal = xLinePts_data[borderPosition];
        m = xLinePts_data[borderPosition + 1] - xVal;
        if (fabs(m) >= 1.0) {
            dy_tmp = fmin(yLinePts_data[borderPosition], yLinePts_data[borderPosition + 1]);
            yLinePts_data[borderPosition] = dy_tmp;
            if (m < 0.0) {
                xVal--;
                xLinePts_data[borderPosition] = xVal;
            }

            xVal = (xVal + 2.0) / 5.0;
            if (fabs(xVal - floor(xVal)) < 0.004) {
                xLinePts_data[borderPosition] = xVal;
                dy_tmp = ceil((dy_tmp + 2.0) / 5.0);
                yLinePts_data[borderPosition] = dy_tmp;
                tempDX = (int)rt_roundd_snf(xVal) - 1;
                tempDY = (int)dy_tmp + 1;
                if ((tempDX + 2 >= 2) && (tempDX + 2 <= N + 1)) {
                    if ((int)dy_tmp + 1 > M + 1) {
                        if (minY_data[tempDX] == 0) {
                            minY_data[tempDX] = M + 1;
                        } else if (minY_data[tempDX] > M + 1) {
                            minY_data[tempDX] = M + 1;
                        }

                        if (maxY_data[tempDX] == 0) {
                            maxY_data[tempDX] = M + 1;
                        } else if (maxY_data[tempDX] < M + 1) {
                            maxY_data[tempDX] = M + 1;
                        }
                    } else {
                        if ((int)dy_tmp + 1 <= 2) {
                            tempDY = 2;
                        }

                        out_data[(tempDY + out->size[0] * tempDX) - 2] = !out_data[(tempDY +
                                                                                    out->size[0] * tempDX) -
                                                                                   2];
                        if (minY_data[tempDX] == 0) {
                            minY_data[tempDX] = tempDY - 1;
                        } else if (minY_data[tempDX] > tempDY - 1) {
                            minY_data[tempDX] = tempDY - 1;
                        }

                        if (maxY_data[tempDX] == 0) {
                            maxY_data[tempDX] = tempDY;
                        } else if (maxY_data[tempDX] < tempDY) {
                            maxY_data[tempDX] = tempDY;
                        }
                    }
                }
            }
        }
    }

    emxFree_real_T(&yLinePts);
    emxFree_real_T(&xLinePts);
}

/**
 * @fn             : poly2mask
 * @brief          :
 * @param          : const double x[8]
 *                   const double y[8]
 *                   double M
 *                   double N
 *                   emxArray_boolean_T *b_BW
 * @return         : void
 */
static void poly2mask(const double x[8], const double y[8], double M, double N,
                      emxArray_boolean_T *b_BW) {
    emxArray_int32_T *maxY;
    emxArray_int32_T *minY;
    emxArray_real_T *xLinePts;
    emxArray_real_T *yLinePts;
    double xNew[9];
    double yNew[9];
    double b_x[8];
    double b_y[8];
    double *xLinePts_data;
    double *yLinePts_data;
    int b_i;
    int borderPosition;
    int i;
    int tempDY;
    int *maxY_data;
    int *minY_data;
    bool *BW_data;
    memset(&xNew[0], 0, 9U * sizeof(double));
    memset(&yNew[0], 0, 9U * sizeof(double));
    emxInit_int32_T(&minY, 2);
    emxInit_int32_T(&maxY, 2);
    emxInit_real_T(&xLinePts, 2);
    emxInit_real_T(&yLinePts, 2);
    if ((x[0] != x[7]) || (y[0] != y[7])) {
        unsigned int b_M;
        unsigned int b_N;
        int nInt_tmp;
        int tempDX;
        xNew[8] = x[0];
        memcpy(&xNew[0], &x[0], 8U * sizeof(double));
        memcpy(&yNew[0], &y[0], 8U * sizeof(double));
        yNew[8] = y[0];
        b_M = (unsigned int)rt_roundd_snf(M);
        b_N = (unsigned int)rt_roundd_snf(N);
        nInt_tmp = (int)b_N;
        i = b_BW->size[0] * b_BW->size[1];
        b_BW->size[0] = (int)b_M;
        b_BW->size[1] = (int)b_N;
        emxEnsureCapacity_boolean_T(b_BW, i);
        BW_data = b_BW->data;
        tempDX = (int)b_M * (int)b_N;
        for (i = 0; i < tempDX; i++) {
            BW_data[i] = false;
        }

        i = minY->size[0] * minY->size[1];
        minY->size[0] = 1;
        minY->size[1] = (int)b_N;
        emxEnsureCapacity_int32_T(minY, i);
        minY_data = minY->data;
        for (i = 0; i < nInt_tmp; i++) {
            minY_data[i] = 0;
        }

        i = maxY->size[0] * maxY->size[1];
        maxY->size[0] = 1;
        maxY->size[1] = (int)b_N;
        emxEnsureCapacity_int32_T(maxY, i);
        maxY_data = maxY->data;
        for (i = 0; i < nInt_tmp; i++) {
            maxY_data[i] = 0;
        }

        poly2edgelist(xNew, yNew, (int)b_M, (int)b_N, b_BW, minY, maxY);
        maxY_data = maxY->data;
        minY_data = minY->data;
        BW_data = b_BW->data;
        for (tempDY = 0; tempDY < nInt_tmp; tempDY++) {
            bool pixel;
            pixel = false;
            i = minY_data[tempDY];
            tempDX = maxY_data[tempDY] - 1;
            for (borderPosition = i; borderPosition <= tempDX; borderPosition++) {
                if (BW_data[(borderPosition + b_BW->size[0] * tempDY) - 1]) {
                    pixel = !pixel;
                }

                BW_data[(borderPosition + b_BW->size[0] * tempDY) - 1] = pixel;
            }
        }
    } else {
        double dy_tmp;
        double m;
        double xVal;
        unsigned int b_M;
        unsigned int b_N;
        int borderSize;
        int nInt_tmp;
        int tempDX;
        b_M = (unsigned int)rt_roundd_snf(M);
        b_N = (unsigned int)rt_roundd_snf(N);
        memcpy(&b_x[0], &x[0], 8U * sizeof(double));
        memcpy(&b_y[0], &y[0], 8U * sizeof(double));
        nInt_tmp = (int)b_N;
        i = b_BW->size[0] * b_BW->size[1];
        b_BW->size[0] = (int)b_M;
        b_BW->size[1] = (int)b_N;
        emxEnsureCapacity_boolean_T(b_BW, i);
        BW_data = b_BW->data;
        tempDX = (int)b_M * (int)b_N;
        for (i = 0; i < tempDX; i++) {
            BW_data[i] = false;
        }

        i = minY->size[0] * minY->size[1];
        minY->size[0] = 1;
        minY->size[1] = (int)b_N;
        emxEnsureCapacity_int32_T(minY, i);
        minY_data = minY->data;
        for (i = 0; i < nInt_tmp; i++) {
            minY_data[i] = 0;
        }

        i = maxY->size[0] * maxY->size[1];
        maxY->size[0] = 1;
        maxY->size[1] = (int)b_N;
        emxEnsureCapacity_int32_T(maxY, i);
        maxY_data = maxY->data;
        for (i = 0; i < nInt_tmp; i++) {
            maxY_data[i] = 0;
        }

        borderSize = 0;
        for (b_i = 0; b_i < 8; b_i++) {
            xVal = floor(5.0 * (b_x[b_i] - 0.5) + 0.5) + 1.0;
            b_x[b_i] = xVal;
            dy_tmp = floor(5.0 * (b_y[b_i] - 0.5) + 0.5) + 1.0;
            b_y[b_i] = dy_tmp;
            if (b_i + 1 > 1) {
                tempDX = (int)rt_roundd_snf(fabs(xVal - b_x[b_i - 1]));
                tempDY = (int)rt_roundd_snf(fabs(dy_tmp - b_y[b_i - 1]));
                if (tempDX >= tempDY) {
                    borderSize = (borderSize + tempDX) + 1;
                } else {
                    borderSize = (borderSize + tempDY) + 1;
                }
            }
        }

        i = yLinePts->size[0] * yLinePts->size[1];
        yLinePts->size[0] = 1;
        yLinePts->size[1] = borderSize;
        emxEnsureCapacity_real_T(yLinePts, i);
        yLinePts_data = yLinePts->data;
        for (i = 0; i < borderSize; i++) {
            yLinePts_data[i] = 0.0;
        }

        i = xLinePts->size[0] * xLinePts->size[1];
        xLinePts->size[0] = 1;
        xLinePts->size[1] = borderSize;
        emxEnsureCapacity_real_T(xLinePts, i);
        xLinePts_data = xLinePts->data;
        for (i = 0; i < borderSize; i++) {
            xLinePts_data[i] = 0.0;
        }

        borderPosition = 0;
        for (b_i = 0; b_i < 7; b_i++) {
            double b_y1;
            double dy_tmp_tmp;
            double x1;
            x1 = b_x[b_i];
            b_y1 = b_y[b_i];
            xVal = b_x[b_i + 1];
            m = xVal - x1;
            tempDX = (int)rt_roundd_snf(fabs(m));
            dy_tmp_tmp = b_y[b_i + 1];
            dy_tmp = dy_tmp_tmp - b_y1;
            tempDY = (int)rt_roundd_snf(fabs(dy_tmp));
            if ((tempDX == 0) && (tempDY == 0)) {
                xLinePts_data[borderPosition] = x1;
                yLinePts_data[borderPosition] = b_y1;
                borderPosition++;
            } else if (tempDX >= tempDY) {
                m = dy_tmp / m;
                if (xVal > x1) {
                    i = (int)(xVal + (1.0 - x1));
                    for (tempDY = 0; tempDY < i; tempDY++) {
                        xVal = x1 + (double)tempDY;
                        tempDX = borderPosition + tempDY;
                        yLinePts_data[tempDX] = rt_roundd_snf(b_y1 + m * (xVal - x1));
                        xLinePts_data[tempDX] = xVal;
                    }

                    borderPosition += i;
                } else {
                    i = (int)-(xVal + (-1.0 - x1));
                    for (tempDY = 0; tempDY < i; tempDY++) {
                        xVal = x1 - (double)tempDY;
                        tempDX = borderPosition + tempDY;
                        yLinePts_data[tempDX] = rt_roundd_snf(b_y1 + m * (xVal - x1));
                        xLinePts_data[tempDX] = xVal;
                    }

                    borderPosition += i;
                }
            } else {
                m /= dy_tmp;
                if (dy_tmp_tmp > b_y1) {
                    i = (int)(dy_tmp_tmp + (1.0 - b_y1));
                    for (tempDY = 0; tempDY < i; tempDY++) {
                        xVal = b_y1 + (double)tempDY;
                        tempDX = borderPosition + tempDY;
                        xLinePts_data[tempDX] = rt_roundd_snf(x1 + m * (xVal - b_y1));
                        yLinePts_data[tempDX] = xVal;
                    }

                    borderPosition += i;
                } else {
                    i = (int)-(dy_tmp_tmp + (-1.0 - b_y1));
                    for (tempDY = 0; tempDY < i; tempDY++) {
                        xVal = b_y1 - (double)tempDY;
                        tempDX = borderPosition + tempDY;
                        xLinePts_data[tempDX] = rt_roundd_snf(x1 + m * (xVal - b_y1));
                        yLinePts_data[tempDX] = xVal;
                    }

                    borderPosition += i;
                }
            }
        }

        for (borderPosition = 0; borderPosition <= borderSize - 2; borderPosition++) {
            xVal = xLinePts_data[borderPosition];
            m = xLinePts_data[borderPosition + 1] - xVal;
            if (fabs(m) >= 1.0) {
                dy_tmp = fmin(yLinePts_data[borderPosition],
                              yLinePts_data[borderPosition + 1]);
                yLinePts_data[borderPosition] = dy_tmp;
                if (m < 0.0) {
                    xVal--;
                    xLinePts_data[borderPosition] = xVal;
                }

                xVal = (xVal + 2.0) / 5.0;
                if (fabs(xVal - floor(xVal)) < 0.004) {
                    xLinePts_data[borderPosition] = xVal;
                    dy_tmp = ceil((dy_tmp + 2.0) / 5.0);
                    yLinePts_data[borderPosition] = dy_tmp;
                    tempDX = (int)rt_roundd_snf(xVal) - 1;
                    tempDY = (int)dy_tmp + 1;
                    if ((tempDX + 2 >= 2) && (tempDX + 2 <= (int)b_N + 1)) {
                        if ((int)dy_tmp + 1 > (int)b_M + 1) {
                            if (minY_data[tempDX] == 0) {
                                minY_data[tempDX] = (int)b_M + 1;
                            } else if (minY_data[tempDX] > (int)b_M + 1) {
                                minY_data[tempDX] = (int)b_M + 1;
                            }

                            if (maxY_data[tempDX] == 0) {
                                maxY_data[tempDX] = (int)b_M + 1;
                            } else if (maxY_data[tempDX] < (int)b_M + 1) {
                                maxY_data[tempDX] = (int)b_M + 1;
                            }
                        } else {
                            if ((int)dy_tmp + 1 <= 2) {
                                tempDY = 2;
                            }

                            BW_data[(tempDY + b_BW->size[0] * tempDX) - 2] = !BW_data[(tempDY + b_BW->size[0] * tempDX) - 2];
                            if (minY_data[tempDX] == 0) {
                                minY_data[tempDX] = tempDY - 1;
                            } else if (minY_data[tempDX] > tempDY - 1) {
                                minY_data[tempDX] = tempDY - 1;
                            }

                            if (maxY_data[tempDX] == 0) {
                                maxY_data[tempDX] = tempDY;
                            } else if (maxY_data[tempDX] < tempDY) {
                                maxY_data[tempDX] = tempDY;
                            }
                        }
                    }
                }
            }
        }

        for (tempDY = 0; tempDY < nInt_tmp; tempDY++) {
            bool pixel;
            pixel = false;
            i = minY_data[tempDY];
            tempDX = maxY_data[tempDY] - 1;
            for (borderPosition = i; borderPosition <= tempDX; borderPosition++) {
                if (BW_data[(borderPosition + b_BW->size[0] * tempDY) - 1]) {
                    pixel = !pixel;
                }

                BW_data[(borderPosition + b_BW->size[0] * tempDY) - 1] = pixel;
            }
        }
    }

    emxFree_real_T(&yLinePts);
    emxFree_real_T(&xLinePts);
    emxFree_int32_T(&maxY);
    emxFree_int32_T(&minY);
}

/**
 * @fn             : remapAndResampleGeneric2d
 * @brief          :
 * @param          : const double inputImage[18483444]
 *                   const double tform_A_[9]
 *                   const double outputRef_ImageSizeAlias[2]
 *                   emxArray_real_T *outputImage
 * @return         : void
 */
static void remapAndResampleGeneric2d(const double inputImage[18483444], const double tform_A_[9], const double outputRef_ImageSizeAlias[2], emxArray_real_T *outputImage) {
    emxArray_real_T *r;
    emxArray_real_T *srcXIntrinsic;
    emxArray_real_T *srcYIntrinsic;
    double b_tform_A_[9];
    double tinv[9];
    double nRows;
    double srcXWorld_val;
    double srczWorld_val;
    double *srcXIntrinsic_data;
    double *srcYIntrinsic_data;
    int colIdx;
    int i;
    int i1;
    int i2;
    int plane;
    int rowIdx;
    int ub_loop;
    for (i = 0; i < 3; i++) {
        b_tform_A_[3 * i] = tform_A_[i];
        b_tform_A_[3 * i + 1] = tform_A_[i + 3];
        b_tform_A_[3 * i + 2] = tform_A_[i + 6];
    }

    inv(b_tform_A_, tinv);
    nRows = outputRef_ImageSizeAlias[0];
    emxInit_real_T(&srcXIntrinsic, 2);
    i = srcXIntrinsic->size[0] * srcXIntrinsic->size[1];
    srcXIntrinsic->size[0] = (int)outputRef_ImageSizeAlias[0];
    srcXIntrinsic->size[1] = (int)outputRef_ImageSizeAlias[1];
    emxEnsureCapacity_real_T(srcXIntrinsic, i);
    srcXIntrinsic_data = srcXIntrinsic->data;
    emxInit_real_T(&srcYIntrinsic, 2);
    i = srcYIntrinsic->size[0] * srcYIntrinsic->size[1];
    srcYIntrinsic->size[0] = (int)outputRef_ImageSizeAlias[0];
    srcYIntrinsic->size[1] = (int)outputRef_ImageSizeAlias[1];
    emxEnsureCapacity_real_T(srcYIntrinsic, i);
    srcYIntrinsic_data = srcYIntrinsic->data;
    ub_loop = (int)outputRef_ImageSizeAlias[1] - 1;

#pragma omp parallel for num_threads(omp_get_max_threads()) private(srczWorld_val, srcXWorld_val, i1, rowIdx)

    for (colIdx = 0; colIdx <= ub_loop; colIdx++) {
        i1 = (int)nRows;
        for (rowIdx = 0; rowIdx < i1; rowIdx++) {
            srczWorld_val = (tinv[6] * ((((double)colIdx + 1.0) - 0.5) + 0.5) + tinv[7] * ((((double)rowIdx + 1.0) - 0.5) + 0.5)) + tinv[8];
            srcXWorld_val = ((tinv[0] * ((((double)colIdx + 1.0) - 0.5) + 0.5) + tinv
                                                                                         [1] *
                                                                                     ((((double)rowIdx + 1.0) - 0.5) + 0.5)) +
                             tinv[2]) /
                            srczWorld_val;
            srczWorld_val = ((tinv[3] * ((((double)colIdx + 1.0) - 0.5) + 0.5) + tinv
                                                                                         [4] *
                                                                                     ((((double)rowIdx + 1.0) - 0.5) + 0.5)) +
                             tinv[5]) /
                            srczWorld_val;
            srcXIntrinsic_data[rowIdx + srcXIntrinsic->size[0] * colIdx] =
                (srcXWorld_val - 0.5) + 0.5;
            srcYIntrinsic_data[rowIdx + srcYIntrinsic->size[0] * colIdx] =
                (srczWorld_val - 0.5) + 0.5;
        }
    }

    i = outputImage->size[0] * outputImage->size[1] * outputImage->size[2];
    outputImage->size[0] = srcXIntrinsic->size[0];
    outputImage->size[1] = srcXIntrinsic->size[1];
    outputImage->size[2] = 3;
    emxEnsureCapacity_real_T(outputImage, i);
    srcXIntrinsic_data = outputImage->data;
    ub_loop = srcXIntrinsic->size[0] * srcXIntrinsic->size[1] * 3;
    for (i = 0; i < ub_loop; i++) {
        srcXIntrinsic_data[i] = 0.0;
    }

    emxInit_real_T(&r, 2);
    for (plane = 0; plane < 3; plane++) {
        b_interp2_local(&inputImage[6161148 * plane], srcXIntrinsic, srcYIntrinsic,
                        r);
        srcYIntrinsic_data = r->data;
        ub_loop = r->size[1];
        for (i = 0; i < ub_loop; i++) {
            int loop_ub;
            loop_ub = r->size[0];
            for (i2 = 0; i2 < loop_ub; i2++) {
                srcXIntrinsic_data[(i2 + outputImage->size[0] * i) + outputImage->size[0] * outputImage->size[1] * plane] = srcYIntrinsic_data[i2 + r->size[0] *
                                                                                                                                                        i];
            }
        }
    }

    emxFree_real_T(&r);
    emxFree_real_T(&srcYIntrinsic);
    emxFree_real_T(&srcXIntrinsic);
}

/**
 * @fn             : rgb2gray
 * @brief          :
 * @param          : const emxArray_real_T *X
 *                   emxArray_real_T *b_I
 * @return         : void
 */
static void rgb2gray(const emxArray_real_T *X, emxArray_real_T *b_I) {
    const double *X_data;
    double sizeI;
    double *I_data;
    int i;
    int ub_loop;
    X_data = X->data;
    ub_loop = b_I->size[0] * b_I->size[1];
    b_I->size[0] = X->size[0];
    b_I->size[1] = X->size[1];
    emxEnsureCapacity_real_T(b_I, ub_loop);
    I_data = b_I->data;
    sizeI = (double)(X->size[0] * X->size[1] * 3) / 3.0;
    ub_loop = (int)sizeI - 1;

#pragma omp parallel for num_threads(omp_get_max_threads())

    for (i = 0; i <= ub_loop; i++) {
        I_data[i] = (X_data[i] * 0.29893602129377539 + X_data[(int)(((double)i + 1.0) + sizeI) - 1] * 0.58704307445112125) + X_data[(int)(((double)i + 1.0) +
                                                                                                                                          2.0 * sizeI) -
                                                                                                                                    1] *
                                                                                                                                 0.11402090425510336;
        if (I_data[i] > 1.0) {
            I_data[i] = 1.0;
        } else if (I_data[i] < 0.0) {
            I_data[i] = 0.0;
        }
    }
}

/**
 * @fn             : rt_atan2f_snf
 * @brief          :
 * @param          : float u0
 *                   float u1
 * @return         : float
 */
static float rt_atan2f_snf(float u0, float u1) {
    float y;
    if (rtIsNaNF(u0) || rtIsNaNF(u1)) {
        y = rtNaNF;
    } else if (rtIsInfF(u0) && rtIsInfF(u1)) {
        int i;
        int i1;
        if (u0 > 0.0F) {
            i = 1;
        } else {
            i = -1;
        }

        if (u1 > 0.0F) {
            i1 = 1;
        } else {
            i1 = -1;
        }

        y = atan2f((float)i, (float)i1);
    } else if (u1 == 0.0F) {
        if (u0 > 0.0F) {
            y = RT_PIF / 2.0F;
        } else if (u0 < 0.0F) {
            y = -(RT_PIF / 2.0F);
        } else {
            y = 0.0F;
        }
    } else {
        y = atan2f(u0, u1);
    }

    return y;
}

/**
 * @fn             : rt_hypotd_snf
 * @brief          :
 * @param          : double u0
 *                   double u1
 * @return         : double
 */
static double rt_hypotd_snf(double u0, double u1) {
    double a;
    double b;
    double y;
    a = fabs(u0);
    b = fabs(u1);
    if (a < b) {
        a /= b;
        y = b * sqrt(a * a + 1.0);
    } else if (a > b) {
        b /= a;
        y = a * sqrt(b * b + 1.0);
    } else if (rtIsNaN(b)) {
        y = rtNaN;
    } else {
        y = a * 1.4142135623730951;
    }

    return y;
}

/**
 * @fn             : rt_hypotf_snf
 * @brief          :
 * @param          : float u0
 *                   float u1
 * @return         : float
 */
static float rt_hypotf_snf(float u0, float u1) {
    float a;
    float b;
    float y;
    a = fabsf(u0);
    b = fabsf(u1);
    if (a < b) {
        a /= b;
        y = b * sqrtf(a * a + 1.0F);
    } else if (a > b) {
        b /= a;
        y = a * sqrtf(b * b + 1.0F);
    } else if (rtIsNaNF(b)) {
        y = rtNaNF;
    } else {
        y = a * 1.41421354F;
    }

    return y;
}

/**
 * @fn             : rt_powf_snf
 * @brief          :
 * @param          : float u0
 *                   float u1
 * @return         : float
 */
static float rt_powf_snf(float u0, float u1) {
    float y;
    if (rtIsNaNF(u0) || rtIsNaNF(u1)) {
        y = rtNaNF;
    } else {
        float f;
        float f1;
        f = fabsf(u0);
        f1 = fabsf(u1);
        if (rtIsInfF(u1)) {
            if (f == 1.0F) {
                y = 1.0F;
            } else if (f > 1.0F) {
                if (u1 > 0.0F) {
                    y = rtInfF;
                } else {
                    y = 0.0F;
                }
            } else if (u1 > 0.0F) {
                y = 0.0F;
            } else {
                y = rtInfF;
            }
        } else if (f1 == 0.0F) {
            y = 1.0F;
        } else if (f1 == 1.0F) {
            if (u1 > 0.0F) {
                y = u0;
            } else {
                y = 1.0F / u0;
            }
        } else if (u1 == 2.0F) {
            y = u0 * u0;
        } else if ((u1 == 0.5F) && (u0 >= 0.0F)) {
            y = sqrtf(u0);
        } else if ((u0 < 0.0F) && (u1 > floorf(u1))) {
            y = rtNaNF;
        } else {
            y = powf(u0, u1);
        }
    }

    return y;
}

/**
 * @fn             : rt_remd_snf
 * @brief          :
 * @param          : double u0
 *                   double u1
 * @return         : double
 */
static double rt_remd_snf(double u0, double u1) {
    double y;
    if (rtIsNaN(u0) || rtIsNaN(u1) || rtIsInf(u0)) {
        y = rtNaN;
    } else if (rtIsInf(u1)) {
        y = u0;
    } else if ((u1 != 0.0) && (u1 != trunc(u1))) {
        double q;
        q = fabs(u0 / u1);
        if (!(fabs(q - floor(q + 0.5)) > DBL_EPSILON * q)) {
            y = 0.0 * u0;
        } else {
            y = fmod(u0, u1);
        }
    } else {
        y = fmod(u0, u1);
    }

    return y;
}

/**
 * @fn             : rt_remf_snf
 * @brief          :
 * @param          : float u0
 *                   float u1
 * @return         : float
 */
static float rt_remf_snf(float u0, float u1) {
    float y;
    if (rtIsNaNF(u0) || rtIsNaNF(u1) || rtIsInfF(u0)) {
        y = rtNaNF;
    } else if (rtIsInfF(u1)) {
        y = u0;
    } else if ((u1 != 0.0F) && (u1 != truncf(u1))) {
        float q;
        q = fabsf(u0 / u1);
        if (!(fabsf(q - floorf(q + 0.5F)) > FLT_EPSILON * q)) {
            y = 0.0F * u0;
        } else {
            y = fmodf(u0, u1);
        }
    } else {
        y = fmodf(u0, u1);
    }

    return y;
}

/**
 * @fn             : rt_roundd_snf
 * @brief          :
 * @param          : double u
 * @return         : double
 */
static double rt_roundd_snf(double u) {
    double y;
    if (fabs(u) < 4.503599627370496E+15) {
        if (u >= 0.5) {
            y = floor(u + 0.5);
        } else if (u > -0.5) {
            y = u * 0.0;
        } else {
            y = ceil(u - 0.5);
        }
    } else {
        y = u;
    }

    return y;
}

/**
 * @fn             : sort
 * @brief          :
 * @param          : emxArray_real32_T *x
 *                   emxArray_int32_T *idx
 * @return         : void
 */
static void sort(emxArray_real32_T *x, emxArray_int32_T *idx) {
    emxArray_int32_T *b_iwork;
    emxArray_int32_T *iidx;
    emxArray_int32_T *iwork;
    emxArray_real32_T *b_xwork;
    emxArray_real32_T *vwork;
    emxArray_real32_T *xwork;
    float *vwork_data;
    float *x_data;
    float *xwork_data;
    int b;
    int b_b;
    int i;
    int j;
    int k;
    int quartetOffset;
    int vlen;
    int vstride;
    int *idx_data;
    int *iidx_data;
    int *iwork_data;
    x_data = x->data;
    vlen = x->size[1] - 1;
    emxInit_real32_T(&vwork, 1);
    i = vwork->size[0];
    vwork->size[0] = x->size[1];
    emxEnsureCapacity_real32_T(vwork, i);
    vwork_data = vwork->data;
    i = idx->size[0] * idx->size[1];
    idx->size[0] = x->size[0];
    idx->size[1] = x->size[1];
    emxEnsureCapacity_int32_T(idx, i);
    idx_data = idx->data;
    vstride = x->size[0];
    emxInit_int32_T(&iidx, 1);
    emxInit_int32_T(&iwork, 1);
    emxInit_real32_T(&xwork, 1);
    emxInit_int32_T(&b_iwork, 1);
    emxInit_real32_T(&b_xwork, 1);
    for (j = 0; j < vstride; j++) {
        int ib;
        for (k = 0; k <= vlen; k++) {
            vwork_data[k] = x_data[j + k * vstride];
        }

        i = iidx->size[0];
        iidx->size[0] = vwork->size[0];
        emxEnsureCapacity_int32_T(iidx, i);
        iidx_data = iidx->data;
        ib = vwork->size[0];
        for (i = 0; i < ib; i++) {
            iidx_data[i] = 0;
        }

        if (vwork->size[0] != 0) {
            float x4[4];
            int idx4[4];
            int bLen2;
            int i2;
            int i3;
            int i4;
            int iidx_tmp;
            int n;
            int nNonNaN;
            n = vwork->size[0];
            x4[0] = 0.0F;
            idx4[0] = 0;
            x4[1] = 0.0F;
            idx4[1] = 0;
            x4[2] = 0.0F;
            idx4[2] = 0;
            x4[3] = 0.0F;
            idx4[3] = 0;
            i = iwork->size[0];
            iwork->size[0] = vwork->size[0];
            emxEnsureCapacity_int32_T(iwork, i);
            iwork_data = iwork->data;
            ib = vwork->size[0];
            for (i = 0; i < ib; i++) {
                iwork_data[i] = 0;
            }

            i = xwork->size[0];
            xwork->size[0] = vwork->size[0];
            emxEnsureCapacity_real32_T(xwork, i);
            xwork_data = xwork->data;
            ib = vwork->size[0];
            for (i = 0; i < ib; i++) {
                xwork_data[i] = 0.0F;
            }

            bLen2 = 0;
            ib = 0;
            for (k = 0; k < n; k++) {
                if (rtIsNaNF(vwork_data[k])) {
                    iidx_tmp = (n - bLen2) - 1;
                    iidx_data[iidx_tmp] = k + 1;
                    xwork_data[iidx_tmp] = vwork_data[k];
                    bLen2++;
                } else {
                    ib++;
                    idx4[ib - 1] = k + 1;
                    x4[ib - 1] = vwork_data[k];
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
                                i1 = (signed char)ib;
                                b_i2 = (signed char)i2;
                                b_i3 = (signed char)i3;
                                b_i4 = (signed char)i4;
                            } else if (f <= x4[i4 - 1]) {
                                i1 = (signed char)ib;
                                b_i2 = (signed char)i3;
                                b_i3 = (signed char)i2;
                                b_i4 = (signed char)i4;
                            } else {
                                i1 = (signed char)ib;
                                b_i2 = (signed char)i3;
                                b_i3 = (signed char)i4;
                                b_i4 = (signed char)i2;
                            }
                        } else {
                            f1 = x4[i4 - 1];
                            if (f <= f1) {
                                if (x4[i2 - 1] <= f1) {
                                    i1 = (signed char)i3;
                                    b_i2 = (signed char)ib;
                                    b_i3 = (signed char)i2;
                                    b_i4 = (signed char)i4;
                                } else {
                                    i1 = (signed char)i3;
                                    b_i2 = (signed char)ib;
                                    b_i3 = (signed char)i4;
                                    b_i4 = (signed char)i2;
                                }
                            } else {
                                i1 = (signed char)i3;
                                b_i2 = (signed char)i4;
                                b_i3 = (signed char)ib;
                                b_i4 = (signed char)i2;
                            }
                        }

                        iidx_data[quartetOffset - 3] = idx4[i1 - 1];
                        iidx_data[quartetOffset - 2] = idx4[b_i2 - 1];
                        iidx_data[quartetOffset - 1] = idx4[b_i3 - 1];
                        iidx_data[quartetOffset] = idx4[b_i4 - 1];
                        vwork_data[quartetOffset - 3] = x4[i1 - 1];
                        vwork_data[quartetOffset - 2] = x4[b_i2 - 1];
                        vwork_data[quartetOffset - 1] = x4[b_i3 - 1];
                        vwork_data[quartetOffset] = x4[b_i4 - 1];
                        ib = 0;
                    }
                }
            }

            i3 = vwork->size[0] - bLen2;
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

                i = (unsigned char)ib;
                for (k = 0; k < i; k++) {
                    iidx_tmp = perm[k] - 1;
                    quartetOffset = (i3 - ib) + k;
                    iidx_data[quartetOffset] = idx4[iidx_tmp];
                    vwork_data[quartetOffset] = x4[iidx_tmp];
                }
            }

            ib = bLen2 >> 1;
            for (k = 0; k < ib; k++) {
                quartetOffset = i3 + k;
                i2 = iidx_data[quartetOffset];
                iidx_tmp = (n - k) - 1;
                iidx_data[quartetOffset] = iidx_data[iidx_tmp];
                iidx_data[iidx_tmp] = i2;
                vwork_data[quartetOffset] = xwork_data[iidx_tmp];
                vwork_data[iidx_tmp] = xwork_data[quartetOffset];
            }

            if ((bLen2 & 1) != 0) {
                ib += i3;
                vwork_data[ib] = xwork_data[ib];
            }

            nNonNaN = vwork->size[0] - bLen2;
            quartetOffset = 2;
            if (nNonNaN > 1) {
                if (vwork->size[0] >= 256) {
                    int nBlocks;
                    nBlocks = nNonNaN >> 8;
                    if (nBlocks > 0) {
                        for (b = 0; b < nBlocks; b++) {
                            float c_xwork[256];
                            int c_iwork[256];
                            i4 = (b << 8) - 1;
                            for (b_b = 0; b_b < 6; b_b++) {
                                n = 1 << (b_b + 2);
                                bLen2 = n << 1;
                                i = 256 >> (b_b + 3);
                                for (k = 0; k < i; k++) {
                                    i2 = (i4 + k * bLen2) + 1;
                                    for (quartetOffset = 0; quartetOffset < bLen2; quartetOffset++) {
                                        ib = i2 + quartetOffset;
                                        c_iwork[quartetOffset] = iidx_data[ib];
                                        c_xwork[quartetOffset] = vwork_data[ib];
                                    }

                                    i3 = 0;
                                    quartetOffset = n;
                                    ib = i2 - 1;
                                    int exitg1;
                                    do {
                                        exitg1 = 0;
                                        ib++;
                                        if (c_xwork[i3] <= c_xwork[quartetOffset]) {
                                            iidx_data[ib] = c_iwork[i3];
                                            vwork_data[ib] = c_xwork[i3];
                                            if (i3 + 1 < n) {
                                                i3++;
                                            } else {
                                                exitg1 = 1;
                                            }
                                        } else {
                                            iidx_data[ib] = c_iwork[quartetOffset];
                                            vwork_data[ib] = c_xwork[quartetOffset];
                                            if (quartetOffset + 1 < bLen2) {
                                                quartetOffset++;
                                            } else {
                                                ib -= i3;
                                                for (quartetOffset = i3 + 1; quartetOffset <= n;
                                                     quartetOffset++) {
                                                    iidx_tmp = ib + quartetOffset;
                                                    iidx_data[iidx_tmp] = c_iwork[quartetOffset - 1];
                                                    vwork_data[iidx_tmp] = c_xwork[quartetOffset - 1];
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
                            xwork_data = xwork->data;
                            iwork_data = iwork->data;
                        }

                        quartetOffset = 8;
                    }
                }

                ib = iwork->size[0];
                i = b_iwork->size[0];
                b_iwork->size[0] = iwork->size[0];
                emxEnsureCapacity_int32_T(b_iwork, i);
                iidx_data = b_iwork->data;
                for (i = 0; i < ib; i++) {
                    iidx_data[i] = iwork_data[i];
                }

                i = b_xwork->size[0];
                b_xwork->size[0] = xwork->size[0];
                emxEnsureCapacity_real32_T(b_xwork, i);
                vwork_data = b_xwork->data;
                ib = xwork->size[0];
                for (i = 0; i < ib; i++) {
                    vwork_data[i] = xwork_data[i];
                }

                merge_block(iidx, vwork, 0, nNonNaN, quartetOffset, b_iwork, b_xwork);
                vwork_data = vwork->data;
                iidx_data = iidx->data;
            }
        }

        for (k = 0; k <= vlen; k++) {
            i = j + k * vstride;
            x_data[i] = vwork_data[k];
            idx_data[i] = iidx_data[k];
        }
    }

    emxFree_real32_T(&b_xwork);
    emxFree_int32_T(&b_iwork);
    emxFree_real32_T(&xwork);
    emxFree_int32_T(&iwork);
    emxFree_int32_T(&iidx);
    emxFree_real32_T(&vwork);
}

/**
 * @fn             : sum
 * @brief          :
 * @param          : const emxArray_real_T *x
 *                   emxArray_real_T *y
 * @return         : void
 */
static void sum(const emxArray_real_T *x, emxArray_real_T *y) {
    const double *x_data;
    double *y_data;
    int k;
    int xj;
    x_data = x->data;
    if ((x->size[0] == 0) || (x->size[1] == 0)) {
        int xoffset;
        xj = y->size[0] * y->size[1];
        y->size[0] = x->size[0];
        y->size[1] = x->size[1];
        emxEnsureCapacity_real_T(y, xj);
        y_data = y->data;
        xoffset = x->size[0] * x->size[1];
        for (xj = 0; xj < xoffset; xj++) {
            y_data[xj] = 0.0;
        }
    } else {
        int vstride;
        vstride = x->size[0] * x->size[1];
        xj = y->size[0] * y->size[1];
        y->size[0] = x->size[0];
        y->size[1] = x->size[1];
        emxEnsureCapacity_real_T(y, xj);
        y_data = y->data;
        for (xj = 0; xj < vstride; xj++) {
            y_data[xj] = x_data[xj];
        }

        for (k = 0; k < 2; k++) {
            int xoffset;
            xoffset = (k + 1) * vstride;
            for (xj = 0; xj < vstride; xj++) {
                y_data[xj] += x_data[xoffset + xj];
            }
        }
    }
}

/**
 * @fn             : svd
 * @brief          :
 * @param          : const double A[9]
 *                   double U[3]
 * @return         : void
 */
static void svd(const double A[9], double U[3]) {
    double b_A[9];
    double e[3];
    double s[3];
    double work[3];
    double absxk;
    double nrm;
    double scale;
    double sm;
    double snorm;
    double sqds;
    double t;
    int ii;
    int iter;
    int k;
    int m;
    int nmqp1_tmp;
    int q;
    int qp1;
    int qq;
    int qs;
    memcpy(&b_A[0], &A[0], 9U * sizeof(double));
    s[0] = 0.0;
    e[0] = 0.0;
    work[0] = 0.0;
    s[1] = 0.0;
    e[1] = 0.0;
    work[1] = 0.0;
    s[2] = 0.0;
    e[2] = 0.0;
    work[2] = 0.0;
    for (q = 0; q < 2; q++) {
        bool apply_transform;
        qp1 = q + 2;
        iter = q + 3 * q;
        qq = iter + 1;
        nmqp1_tmp = 2 - q;
        apply_transform = false;
        nrm = 0.0;
        scale = 3.3121686421112381E-170;
        qs = (iter - q) + 3;
        for (k = qq; k <= qs; k++) {
            absxk = fabs(b_A[k - 1]);
            if (absxk > scale) {
                t = scale / absxk;
                nrm = nrm * t * t + 1.0;
                scale = absxk;
            } else {
                t = absxk / scale;
                nrm += t * t;
            }
        }

        nrm = scale * sqrt(nrm);
        if (nrm > 0.0) {
            apply_transform = true;
            if (b_A[iter] < 0.0) {
                nrm = -nrm;
            }

            s[q] = nrm;
            if (fabs(nrm) >= 1.0020841800044864E-292) {
                nrm = 1.0 / nrm;
                for (k = qq; k <= qs; k++) {
                    b_A[k - 1] *= nrm;
                }
            } else {
                for (k = qq; k <= qs; k++) {
                    b_A[k - 1] /= s[q];
                }
            }

            b_A[iter]++;
            s[q] = -s[q];
        } else {
            s[q] = 0.0;
        }

        for (m = qp1; m < 4; m++) {
            qq = q + 3 * (m - 1);
            if (apply_transform) {
                t = 0.0;
                for (k = 0; k <= nmqp1_tmp; k++) {
                    t += b_A[iter + k] * b_A[qq + k];
                }

                nrm = -(t / b_A[iter]);
                if (!(nrm == 0.0)) {
                    for (k = 0; k <= nmqp1_tmp; k++) {
                        qs = qq + k;
                        b_A[qs] += nrm * b_A[iter + k];
                    }
                }
            }

            e[m - 1] = b_A[qq];
        }

        if (q + 1 <= 1) {
            nrm = c_xnrm2(e);
            if (nrm == 0.0) {
                e[0] = 0.0;
            } else {
                if (e[1] < 0.0) {
                    e[0] = -nrm;
                } else {
                    e[0] = nrm;
                }

                nrm = e[0];
                if (fabs(e[0]) >= 1.0020841800044864E-292) {
                    nrm = 1.0 / e[0];
                    for (k = qp1; k < 4; k++) {
                        e[k - 1] *= nrm;
                    }
                } else {
                    for (k = qp1; k < 4; k++) {
                        e[k - 1] /= nrm;
                    }
                }

                e[1]++;
                e[0] = -e[0];
                for (ii = qp1; ii < 4; ii++) {
                    work[ii - 1] = 0.0;
                }

                for (m = qp1; m < 4; m++) {
                    nrm = e[m - 1];
                    if (!(nrm == 0.0)) {
                        qq = 3 * (m - 1) + 1;
                        work[1] += nrm * b_A[qq];
                        work[2] += nrm * b_A[qq + 1];
                    }
                }

                for (m = qp1; m < 4; m++) {
                    nrm = -e[m - 1] / e[1];
                    if (!(nrm == 0.0)) {
                        qq = (q + 3 * (m - 1)) + 1;
                        b_A[qq] += nrm * work[q + 1];
                        b_A[qq + 1] += nrm * work[q + 2];
                    }
                }
            }
        }
    }

    m = 1;
    s[2] = b_A[8];
    e[1] = b_A[7];
    e[2] = 0.0;
    iter = 0;
    nrm = s[0];
    if (s[0] != 0.0) {
        absxk = fabs(s[0]);
        t = s[0] / absxk;
        nrm = absxk;
        s[0] = absxk;
        e[0] /= t;
    }

    if (e[0] != 0.0) {
        absxk = fabs(e[0]);
        t = absxk / e[0];
        e[0] = absxk;
        s[1] *= t;
    }

    snorm = fmax(fabs(nrm), e[0]);
    nrm = s[1];
    if (s[1] != 0.0) {
        absxk = fabs(s[1]);
        t = s[1] / absxk;
        nrm = absxk;
        s[1] = absxk;
        e[1] = b_A[7] / t;
    }

    if (e[1] != 0.0) {
        absxk = fabs(e[1]);
        t = absxk / e[1];
        e[1] = absxk;
        s[2] = b_A[8] * t;
    }

    snorm = fmax(snorm, fmax(fabs(nrm), e[1]));
    nrm = s[2];
    if (s[2] != 0.0) {
        absxk = fabs(s[2]);
        nrm = absxk;
        s[2] = absxk;
    }

    snorm = fmax(snorm, fmax(fabs(nrm), 0.0));
    while ((m + 2 > 0) && (iter < 75)) {
        bool exitg1;
        nmqp1_tmp = m + 1;
        ii = m + 1;
        exitg1 = false;
        while (!(exitg1 || (ii == 0))) {
            nrm = fabs(e[ii - 1]);
            if ((nrm <= 2.2204460492503131E-16 * (fabs(s[ii - 1]) + fabs(s[ii]))) ||
                (nrm <= 1.0020841800044864E-292) || ((iter > 20) && (nrm <= 2.2204460492503131E-16 * snorm))) {
                e[ii - 1] = 0.0;
                exitg1 = true;
            } else {
                ii--;
            }
        }

        if (ii == m + 1) {
            qq = 4;
        } else {
            qs = m + 2;
            qq = m + 2;
            exitg1 = false;
            while ((!exitg1) && (qq >= ii)) {
                qs = qq;
                if (qq == ii) {
                    exitg1 = true;
                } else {
                    nrm = 0.0;
                    if (qq < m + 2) {
                        nrm = fabs(e[qq - 1]);
                    }

                    if (qq > ii + 1) {
                        nrm += fabs(e[qq - 2]);
                    }

                    absxk = fabs(s[qq - 1]);
                    if ((absxk <= 2.2204460492503131E-16 * nrm) || (absxk <=
                                                                    1.0020841800044864E-292)) {
                        s[qq - 1] = 0.0;
                        exitg1 = true;
                    } else {
                        qq--;
                    }
                }
            }

            if (qs == ii) {
                qq = 3;
            } else if (qs == m + 2) {
                qq = 1;
            } else {
                qq = 2;
                ii = qs;
            }
        }

        switch (qq) {
            case 1:
                absxk = e[m];
                e[m] = 0.0;
                for (k = nmqp1_tmp; k >= ii + 1; k--) {
                    b_xrotg(&s[k - 1], &absxk, &sm, &sqds);
                    if (k > ii + 1) {
                        absxk = -sqds * e[0];
                        e[0] *= sm;
                    }
                }
                break;

            case 2:
                absxk = e[ii - 1];
                e[ii - 1] = 0.0;
                for (k = ii + 1; k <= m + 2; k++) {
                    b_xrotg(&s[k - 1], &absxk, &sm, &sqds);
                    t = e[k - 1];
                    absxk = -sqds * t;
                    e[k - 1] = t * sm;
                }
                break;

            case 3:
                nrm = s[m + 1];
                scale = fmax(fmax(fmax(fmax(fabs(nrm), fabs(s[m])), fabs(e[m])), fabs(s[ii])),
                             fabs(e[ii]));
                sm = nrm / scale;
                nrm = s[m] / scale;
                absxk = e[m] / scale;
                sqds = s[ii] / scale;
                t = ((nrm + sm) * (nrm - sm) + absxk * absxk) / 2.0;
                nrm = sm * absxk;
                nrm *= nrm;
                if ((t != 0.0) || (nrm != 0.0)) {
                    absxk = sqrt(t * t + nrm);
                    if (t < 0.0) {
                        absxk = -absxk;
                    }

                    absxk = nrm / (t + absxk);
                } else {
                    absxk = 0.0;
                }

                absxk += (sqds + sm) * (sqds - sm);
                nrm = sqds * (e[ii] / scale);
                for (k = ii + 1; k <= nmqp1_tmp; k++) {
                    b_xrotg(&absxk, &nrm, &sm, &sqds);
                    if (k > ii + 1) {
                        e[0] = absxk;
                    }

                    nrm = e[k - 1];
                    t = s[k - 1];
                    e[k - 1] = sm * nrm - sqds * t;
                    absxk = sqds * s[k];
                    s[k] *= sm;
                    s[k - 1] = sm * t + sqds * nrm;
                    b_xrotg(&s[k - 1], &absxk, &sm, &sqds);
                    absxk = sm * e[k - 1] + sqds * s[k];
                    s[k] = -sqds * e[k - 1] + sm * s[k];
                    nrm = sqds * e[k];
                    e[k] *= sm;
                }

                e[m] = absxk;
                iter++;
                break;

            default:
                if (s[ii] < 0.0) {
                    s[ii] = -s[ii];
                }

                qp1 = ii + 1;
                while ((ii + 1 < 3) && (s[ii] < s[qp1])) {
                    absxk = s[ii];
                    s[ii] = s[qp1];
                    s[qp1] = absxk;
                    ii = qp1;
                    qp1++;
                }

                iter = 0;
                m--;
                break;
        }
    }

    U[0] = s[0];
    U[1] = s[1];
    U[2] = s[2];
}

/**
 * @fn             : uint8PortableCodeAlgo
 * @brief          :
 * @param          : const emxArray_real_T *b_I
 *                   emxArray_uint8_T *u
 * @return         : void
 */
static void uint8PortableCodeAlgo(const emxArray_real_T *b_I, emxArray_uint8_T
                                                                  *u) {
    const double *I_data;
    double val;
    int b_index;
    int i;
    unsigned char *u_data;
    I_data = b_I->data;
    i = u->size[0] * u->size[1];
    u->size[0] = b_I->size[0];
    u->size[1] = b_I->size[1];
    emxEnsureCapacity_uint8_T(u, i);
    u_data = u->data;
    if ((b_I->size[0] == 0) || (b_I->size[1] == 0)) {
        int ub_loop;
        i = u->size[0] * u->size[1];
        u->size[0] = b_I->size[0];
        u->size[1] = b_I->size[1];
        emxEnsureCapacity_uint8_T(u, i);
        u_data = u->data;
        ub_loop = b_I->size[0] * b_I->size[1];
        for (i = 0; i < ub_loop; i++) {
            u_data[i] = (unsigned char)I_data[i];
        }
    } else {
        int ub_loop;
        ub_loop = b_I->size[0] * b_I->size[1] - 1;

#pragma omp parallel for num_threads(omp_get_max_threads()) private(val)

        for (b_index = 0; b_index <= ub_loop; b_index++) {
            val = I_data[b_index] * 255.0;
            if (val < 0.0) {
                u_data[b_index] = 0U;
            } else if (val > 255.0) {
                u_data[b_index] = MAX_uint8_T;
            } else {
                u_data[b_index] = (unsigned char)(val + 0.5);
            }
        }
    }
}

/**
 * @fn             : vehicleToLocalImage
 * @brief          :
 * @param          : const double refBirdsEye_OutputView[4]
 *                   const double refBirdsEye_Sensor_Intrinsics_K[9]
 *                   double refBirdsEye_Sensor_Height
 *                   double refBirdsEye_Sensor_Pitch
 *                   double refBirdsEye_Sensor_Yaw
 *                   double refBirdsEye_Sensor_Roll
 *                   const double c_refBirdsEye_Sensor_SensorLoca[2]
 *                   const double refBirdsEye_Scale[2]
 *                   const double refImg_XWorldLimits[2]
 *                   const double refImg_YWorldLimits[2]
 *                   const double refImg_ImageSizeAlias[2]
 *                   bool refImg_ForcePixelExtentToOne
 *                   double localImagePts[8]
 * @return         : void
 */
static void vehicleToLocalImage(const double refBirdsEye_OutputView[4], const double refBirdsEye_Sensor_Intrinsics_K[9], double refBirdsEye_Sensor_Height,
                                double refBirdsEye_Sensor_Pitch, double refBirdsEye_Sensor_Yaw, double refBirdsEye_Sensor_Roll, const double c_refBirdsEye_Sensor_SensorLoca[2],
                                const double refBirdsEye_Scale[2], const double refImg_XWorldLimits[2], const double refImg_YWorldLimits[2], const double refImg_ImageSizeAlias[2], bool refImg_ForcePixelExtentToOne, double localImagePts[8]) {
    static const signed char a[12] = {-2, 20, 20, -2, 10, 10, -10, -10, 1, 1, 1,
                                      1};

    double U[12];
    double b_refBirdsEye_Scale[9];
    double b_t2_T[9];
    double t2_T[9];
    double d;
    double extentX;
    double extentY;
    int i;
    int jtilecol;

    /*  Brief: 在2D鸟瞰图中把局部vehiclePts转换到当前四副鱼眼环视图拼接的局部图像坐标 */
    /*  Details: */
    /*     vehicle坐标系符合matlab自动驾驶工具箱定义，即x轴朝向车辆正前方，y轴朝向车辆左侧。 */
    /*   */
    /*  Syntax:   */
    /*      localImagePts = vehicleToLocalImage(refBirdsEye,refImg,vehiclePts) */
    /*   */
    /*  Inputs: */
    /*     refBirdsEye - [1,1] size,[birdsEyeView] type,matlab build-in type */
    /*     refImg - [1,1] size,[imref2d] type,matlab build-in type,四副环视拼接参考对象 */
    /*     vehiclePts - [m,2] size,[double] type,待转换到局部车辆坐标点 */
    /*   */
    /*  Outputs: */
    /*     localImagePts - [m,2] size,[double] type,四副环视拼接的鸟瞰图像中的局部像素坐标 */
    /*   */
    /*  Example:  */
    /*     None */
    /*   */
    /*  See also: None */
    /*  Author:                          cuixingxing */
    /*  Email:                           xingxing.cui@long-horn.com */
    /*  Created:                         13-Oct-2022 09:00:37 */
    /*  Version history revision notes: */
    /*                                   None */
    /*  Implementation In Matlab R2022b */
    /*  Copyright © 2022 long-horn.All Rights Reserved. */
    /*  */
    c_monoCamera_get_ImageToVehicle(refBirdsEye_Sensor_Intrinsics_K,
                                    refBirdsEye_Sensor_Height, refBirdsEye_Sensor_Pitch, refBirdsEye_Sensor_Yaw,
                                    refBirdsEye_Sensor_Roll, c_refBirdsEye_Sensor_SensorLoca, t2_T);
    b_refBirdsEye_Scale[0] = refBirdsEye_Scale[0];
    b_refBirdsEye_Scale[3] = 0.0;
    b_refBirdsEye_Scale[6] = 0.0;
    b_refBirdsEye_Scale[1] = 0.0;
    b_refBirdsEye_Scale[4] = refBirdsEye_Scale[1];
    b_refBirdsEye_Scale[7] = 0.0;
    b_refBirdsEye_Scale[2] = refBirdsEye_Scale[0] * refBirdsEye_OutputView[3] +
                             1.0;
    b_refBirdsEye_Scale[5] = refBirdsEye_Scale[1] * refBirdsEye_OutputView[1] +
                             1.0;
    b_refBirdsEye_Scale[8] = 1.0;
    for (i = 0; i < 3; i++) {
        extentX = t2_T[i];
        extentY = t2_T[i + 3];
        d = t2_T[i + 6];
        for (jtilecol = 0; jtilecol < 3; jtilecol++) {
            b_t2_T[i + 3 * jtilecol] = (extentX * (double)iv[3 * jtilecol] + extentY *
                                                                                 (double)iv[3 * jtilecol + 1]) +
                                       d * (double)iv[3 * jtilecol + 2];
        }

        extentX = b_t2_T[i];
        extentY = b_t2_T[i + 3];
        d = b_t2_T[i + 6];
        for (jtilecol = 0; jtilecol < 3; jtilecol++) {
            t2_T[i + 3 * jtilecol] = (extentX * b_refBirdsEye_Scale[3 * jtilecol] +
                                      extentY * b_refBirdsEye_Scale[3 * jtilecol + 1]) +
                                     d *
                                         b_refBirdsEye_Scale[3 * jtilecol + 2];
        }
    }

    inv(t2_T, b_t2_T);
    c_monoCamera_get_ImageToVehicle(refBirdsEye_Sensor_Intrinsics_K,
                                    refBirdsEye_Sensor_Height, refBirdsEye_Sensor_Pitch, refBirdsEye_Sensor_Yaw,
                                    refBirdsEye_Sensor_Roll, c_refBirdsEye_Sensor_SensorLoca, t2_T);
    for (i = 0; i < 3; i++) {
        extentX = b_t2_T[i];
        extentY = b_t2_T[i + 3];
        d = b_t2_T[i + 6];
        for (jtilecol = 0; jtilecol < 3; jtilecol++) {
            b_refBirdsEye_Scale[i + 3 * jtilecol] = (extentX * t2_T[3 * jtilecol] +
                                                     extentY * t2_T[3 * jtilecol + 1]) +
                                                    d * t2_T[3 * jtilecol + 2];
        }
    }

    inv(b_refBirdsEye_Scale, b_t2_T);
    for (i = 0; i < 4; i++) {
        signed char i1;
        signed char i2;
        signed char i3;
        i1 = a[i];
        i2 = a[i + 4];
        i3 = a[i + 8];
        for (jtilecol = 0; jtilecol < 3; jtilecol++) {
            U[i + (jtilecol << 2)] = ((double)i1 * b_t2_T[3 * jtilecol] + (double)i2 *
                                                                              b_t2_T[3 * jtilecol + 1]) +
                                     (double)i3 * b_t2_T[3 * jtilecol + 2];
        }
    }

    for (jtilecol = 0; jtilecol < 2; jtilecol++) {
        extentX = U[9];
        extentY = U[10];
        d = U[11];
        i = jtilecol << 2;
        U[i] /= U[8];
        U[i + 1] /= extentX;
        U[i + 2] /= extentY;
        U[i + 3] /= d;
    }

    if (refImg_ForcePixelExtentToOne) {
        extentX = 1.0;
        extentY = 1.0;
    } else {
        extentX = (refImg_XWorldLimits[1] - refImg_XWorldLimits[0]) /
                  refImg_ImageSizeAlias[1];
        extentY = (refImg_YWorldLimits[1] - refImg_YWorldLimits[0]) /
                  refImg_ImageSizeAlias[0];
    }

    localImagePts[0] = (U[0] - refImg_XWorldLimits[0]) / extentX + 0.5;
    localImagePts[1] = (U[1] - refImg_XWorldLimits[0]) / extentX + 0.5;
    localImagePts[2] = (U[2] - refImg_XWorldLimits[0]) / extentX + 0.5;
    localImagePts[3] = (U[3] - refImg_XWorldLimits[0]) / extentX + 0.5;
    localImagePts[4] = (U[4] - refImg_YWorldLimits[0]) / extentY + 0.5;
    localImagePts[5] = (U[5] - refImg_YWorldLimits[0]) / extentY + 0.5;
    localImagePts[6] = (U[6] - refImg_YWorldLimits[0]) / extentY + 0.5;
    localImagePts[7] = (U[7] - refImg_YWorldLimits[0]) / extentY + 0.5;
}

/**
 * @fn             : voronoiEDT
 * @brief          :
 * @param          : const emxArray_real32_T *g
 *                   const emxArray_real32_T *h
 *                   emxArray_real32_T *D
 * @return         : void
 */
static void voronoiEDT(const emxArray_real32_T *g, const emxArray_real32_T *h,
                       emxArray_real32_T *D) {
    emxArray_real32_T *b_g;
    emxArray_real32_T *b_h;
    const float *g_data;
    const float *h_data;
    float b;
    float c;
    float *D_data;
    float *b_g_data;
    float *b_h_data;
    int i;
    int loop_ub;
    unsigned int ns;
    bool exitg1;
    D_data = D->data;
    h_data = h->data;
    g_data = g->data;
    emxInit_real32_T(&b_h, 1);
    i = b_h->size[0];
    b_h->size[0] = h->size[0];
    emxEnsureCapacity_real32_T(b_h, i);
    b_h_data = b_h->data;
    loop_ub = h->size[0];
    for (i = 0; i < loop_ub; i++) {
        b_h_data[i] = h_data[i];
    }

    emxInit_real32_T(&b_g, 1);
    i = b_g->size[0];
    b_g->size[0] = g->size[0];
    emxEnsureCapacity_real32_T(b_g, i);
    b_g_data = b_g->data;
    loop_ub = g->size[0];
    for (i = 0; i < loop_ub; i++) {
        b_g_data[i] = g_data[i];
    }

    ns = 0U;
    i = D->size[0];
    for (loop_ub = 0; loop_ub < i; loop_ub++) {
        if (D_data[loop_ub] != -1.0F) {
            if ((int)ns < 2) {
                ns = (unsigned int)((int)ns + 1);
                b_g_data[(int)ns - 1] = D_data[loop_ub];
                b_h_data[(int)ns - 1] = (float)((double)loop_ub + 1.0);
            } else {
                exitg1 = false;
                while ((!exitg1) && ((int)ns >= 2)) {
                    float a;
                    b = b_h_data[(int)ns - 1];
                    c = b_h_data[(int)ns - 2];
                    a = b - c;
                    b = (float)((double)loop_ub + 1.0) - b;
                    c = (float)((double)loop_ub + 1.0) - c;
                    if ((c * b_g_data[(int)ns - 1] - b * b_g_data[(int)ns - 2]) - a *
                                                                                      D_data[loop_ub] >
                        a * b * c) {
                        ns = (unsigned int)((int)ns - 1);
                    } else {
                        exitg1 = true;
                    }
                }

                ns++;
                b_g_data[(int)ns - 1] = D_data[loop_ub];
                b_h_data[(int)ns - 1] = (float)((double)loop_ub + 1.0);
            }
        }
    }

    if ((int)ns != 0) {
        unsigned int el;
        el = 1U;
        i = D->size[0];
        for (loop_ub = 0; loop_ub < i; loop_ub++) {
            exitg1 = false;
            while ((!exitg1) && (el < ns)) {
                b = b_h_data[(int)el - 1] - (float)((double)loop_ub + 1.0);
                c = b_h_data[(int)el] - (float)((double)loop_ub + 1.0);
                if (b_g_data[(int)el - 1] + b * b > b_g_data[(int)el] + c * c) {
                    el++;
                } else {
                    exitg1 = true;
                }
            }

            D_data[loop_ub] = b_g_data[(int)el - 1] + (b_h_data[(int)el - 1] - (float)((double)loop_ub + 1.0)) * (b_h_data[(int)el - 1] - (float)((double)
                                                                                                                                                      loop_ub +
                                                                                                                                                  1.0));
        }
    }

    emxFree_real32_T(&b_h);
    emxFree_real32_T(&b_g);
}

/**
 * @fn             : worldToGlobalImagePose
 * @brief          :
 * @param          : const double anchorPose[3]
 *                   const double worldPose[3]
 *                   const double localOrigin[2]
 *                   const double resolutionXY[2]
 *                   double imagePose[3]
 * @return         : void
 */
static void worldToGlobalImagePose(const double anchorPose[3], const double worldPose[3], const double localOrigin[2], const double resolutionXY[2],
                                   double imagePose[3]) {
    static const signed char b[9] = {1, 0, 0, 0, -1, 0, 0, 0, -1};

    double c_rotmat_tmp[9];
    double b_worldPose[3];
    double d_rotmat_tmp[3];
    double b_rotmat_tmp;
    double d;
    double d1;
    double d2;
    double rotatedRect_idx_2;
    double rotatedRect_idx_3;
    double rotmat_tmp;
    double theta0;
    double vertices_tmp_tmp;
    int i;
    int i2;

    /* #cogen */
    /*  Brief: 2D鸟瞰图场景下世界坐标系姿态worldPose转换为以anchorPose的图像坐标系的坐  */
    /*  标姿态imagePose */
    /*  Details: */
    /*        姿态theta为在世界坐标系中以x轴正方向逆时针旋转为正，在图像坐标系中以x轴 */
    /*  正方向顺时针旋转为正。都满足右手定则。localOrigin指定图像与anchorPose世界点对应。 */
    /*  Syntax:   */
    /*      imagePose = worldToGlobalImagePose(anchorPose,worldPose) */
    /*   */
    /*  Inputs: */
    /*     anchorPose - [1,3] size,[double] type,形如[x,y,theta]二维姿态，theta为弧度 */
    /*     worldPose - [m,3] size,[double] type,形如[x,y,theta]二维姿态，theta为弧度 */
    /*     localOrigin - [1,2] size,[double] type,形如[xOri,yOri],单位像素, */
    /*                     前摄像头中心在四副拼接图中的像素坐标 */
    /*     resolutionXY - [1,2] size,[double] type,每个像素在世界坐标系下的长度 */
    /*   */
    /*  Outputs: */
    /*     imagePose - [m,3] size,[double] type,图像坐标姿态,每行与worldPose一一对应， */
    /*               形如[x,y,theta] ,theta单位为度 */
    /*   */
    /*  Example:  */
    /*     None */
    /*   */
    /*  See also: None */
    /*  Author:                          cuixingxing */
    /*  Email:                           xingxing.cui@long-horn.com */
    /*  Created:                         11-Oct-2022 17:15:34 */
    /*  Version history revision notes: */
    /*                                   None */
    /*  Implementation In Matlab R2022b */
    /*  Copyright © 2022 long-horn.All Rights Reserved. */
    /*  */
    /*  世界坐标系下 */
    /*  图像坐标系下 */
    /*  世界坐标系下 */
    theta0 = 57.295779513082323 * anchorPose[2];
    rotatedRect_idx_2 = 2.0 * (localOrigin[0] * resolutionXY[0]);
    rotatedRect_idx_3 = 2.0 * (localOrigin[1] * resolutionXY[1]);

    /* 初始位置 */
    /*  功能：根据一个旋转矩形rotatedRect返回该矩形的4个顶点坐标。功能等同于opencv中 */
    /*  rotatedRect类的points成员函数。 */
    /*  输入： */
    /*      rotatedRect */
    /*        1*5 double,形如[xcenter, ycenter, width, height, */
    /*        yaw]，表示一个旋转矩形,yaw单位为度,以绕x轴逆时针为正 */
    /*  输出： */
    /*     vertices */
    /*          4*2 double，旋转矩形的顶点坐标，每行形如[x,y]为一个顶点的坐标，注意顺序 */
    /*  reference: */
    /*    [1] https://en.wikipedia.org/wiki/Rotation_matrix */
    /*    [2] https://blog.csdn.net/Mynameisyournamewuyu/article/details/88650409 */
    /*    [3] https://github.com/opencv/opencv/blob/6ae8103022cdb3cd79f417945fd8332c28298b7b/modules/core/src/types.cpp#L173 */
    /*  */
    /*  Email:xingxing.cui@long-horn.com */
    /*  date:2022.1.20 cuixingxing create this file */
    /*  记录：此函数设计支持高效的C代码生成 */
    /*  */
    /*  2022.10.13 修改角度为正确的旋转角度顺序，x轴逆时针方向开始 */
    /*   结论推到公式： */
    /*   syms cx cy w h theta real */
    /*   normVertices = [cx-h/2,cy+w/2; % 左下，x轴正方向基准旋转之前看 */
    /*      cx-h/2,cy-w/2;% 右下 */
    /*      cx+h/2,cy-w/2;% 右上 */
    /*      cx+h/2,cy+w/2];% 左上 */
    /*  R = [cosd(theta),-sind(theta); */
    /*      sind(theta),cosd(theta)]; */
    /*  dst = R*(normVertices'-[cx;cy])+[cx;cy]; */
    /*  dst = simplify(dst) */
    /*  */
    /*  Example: */
    /*  rotatedRect = [100,100,200,100,-60];% [centerx,centery,width,height,theta] */
    /*  vertices = getVertices(rotatedRect); */
    /*   */
    /*  pts = [vertices;vertices(1,:)]; */
    /*  plot(pts(:,1),pts(:,2),pts(1,1),pts(1,2),'ro') */
    /*  axis equal */
    /*  */
    /*  单位：度 */
    vertices_tmp_tmp = theta0;
    b_cosd(&vertices_tmp_tmp);
    vertices_tmp_tmp = theta0;
    b_sind(&vertices_tmp_tmp);
    vertices_tmp_tmp = theta0;
    b_cosd(&vertices_tmp_tmp);
    vertices_tmp_tmp = theta0;
    b_sind(&vertices_tmp_tmp);
    vertices_tmp_tmp = theta0;
    b_cosd(&vertices_tmp_tmp);
    vertices_tmp_tmp = theta0;
    b_sind(&vertices_tmp_tmp);
    vertices_tmp_tmp = theta0;
    b_cosd(&vertices_tmp_tmp);
    d = theta0;
    b_sind(&d);
    d1 = theta0;
    b_cosd(&d1);
    d1 = theta0;
    b_sind(&d1);
    d1 = theta0;
    b_cosd(&d1);
    d2 = theta0;
    b_sind(&d2);

    /*  top left corner */
    rotmat_tmp = theta0 - 90.0;
    b_sind(&rotmat_tmp);
    b_rotmat_tmp = theta0 - 90.0;
    b_cosd(&b_rotmat_tmp);

    /*  坐标轴旋转等价的点旋转矩阵 */
    /*  tform = rigidtform3d(R,t);% 世界坐标系到图像物理坐标系的位置转换 */
    /*  世界坐标系到图像物理坐标系到姿态（角度）转换 */
    c_rotmat_tmp[0] = b_rotmat_tmp;
    c_rotmat_tmp[3] = -rotmat_tmp;
    c_rotmat_tmp[6] = 0.0;
    c_rotmat_tmp[1] = rotmat_tmp;
    c_rotmat_tmp[4] = b_rotmat_tmp;
    c_rotmat_tmp[7] = 0.0;
    c_rotmat_tmp[2] = 0.0;
    c_rotmat_tmp[5] = 0.0;
    c_rotmat_tmp[8] = 1.0;
    b_worldPose[0] = worldPose[0] - ((anchorPose[0] + rotatedRect_idx_3 *
                                                          vertices_tmp_tmp / 2.0) -
                                     rotatedRect_idx_2 * d / 2.0);
    b_worldPose[1] = worldPose[1] - ((anchorPose[1] + rotatedRect_idx_2 * d1 / 2.0) + rotatedRect_idx_3 * d2 / 2.0);
    b_worldPose[2] = 0.0;
    for (i = 0; i < 3; i++) {
        int i1;
        vertices_tmp_tmp = 0.0;
        d = c_rotmat_tmp[i];
        d1 = c_rotmat_tmp[i + 3];
        i1 = (int)c_rotmat_tmp[i + 6];
        for (i2 = 0; i2 < 3; i2++) {
            vertices_tmp_tmp += ((d * (double)b[3 * i2] + d1 * (double)b[3 * i2 + 1]) + (double)(i1 * b[3 * i2 + 2])) * b_worldPose[i2];
        }

        d_rotmat_tmp[i] = vertices_tmp_tmp;
    }

    imagePose[0] = d_rotmat_tmp[0] / resolutionXY[0];
    imagePose[1] = d_rotmat_tmp[1] / resolutionXY[1];
    imagePose[2] = (-(57.295779513082323 * worldPose[2]) + theta0) - 90.0;
}

/**
 * @fn             : xnrm2
 * @brief          :
 * @param          : const float x[4]
 * @return         : float
 */
static float xnrm2(const float x[4]) {
    float absxk;
    float scale;
    float t;
    float y;
    scale = 1.29246971E-26F;
    absxk = fabsf(x[0]);
    if (absxk > 1.29246971E-26F) {
        y = 1.0F;
        scale = absxk;
    } else {
        t = absxk / 1.29246971E-26F;
        y = t * t;
    }

    absxk = fabsf(x[1]);
    if (absxk > scale) {
        t = scale / absxk;
        y = y * t * t + 1.0F;
        scale = absxk;
    } else {
        t = absxk / scale;
        y += t * t;
    }

    return scale * sqrtf(y);
}

/**
 * @fn             : xrotg
 * @brief          :
 * @param          : float *a
 *                   float *b
 *                   float *c
 *                   float *s
 * @return         : void
 */
static void xrotg(float *a, float *b, float *c, float *s) {
    float absa;
    float absb;
    float roe;
    float scale;
    roe = *b;
    absa = fabsf(*a);
    absb = fabsf(*b);
    if (absa > absb) {
        roe = *a;
    }

    scale = absa + absb;
    if (scale == 0.0F) {
        *s = 0.0F;
        *c = 1.0F;
        *a = 0.0F;
        *b = 0.0F;
    } else {
        float ads;
        float bds;
        ads = absa / scale;
        bds = absb / scale;
        scale *= sqrtf(ads * ads + bds * bds);
        if (roe < 0.0F) {
            scale = -scale;
        }

        *c = *a / scale;
        *s = *b / scale;
        if (absa > absb) {
            *b = *s;
        } else if (*c != 0.0F) {
            *b = 1.0F / *c;
        } else {
            *b = 1.0F;
        }

        *a = scale;
    }
}

/**
 * @fn             : xswap
 * @brief          :
 * @param          : float x[4]
 * @return         : void
 */
static void xswap(float x[4]) {
    float temp;
    temp = x[0];
    x[0] = x[2];
    x[2] = temp;
    temp = x[1];
    x[1] = x[3];
    x[3] = temp;
}

/**
 * @fn             : constructWorldMap
 * @brief          :
 * @param          : const struct0_T *inputArgs
 *                   const struct1_T *birdsEye360
 *                   struct6_T *outputStruct
 * @return         : void
 */
void constructWorldMap(const struct0_T *inputArgs, const struct1_T *birdsEye360,
                       struct6_T *outputStruct) {
    static affinetform2d tforms[4];
    static birdsEyeView birdsEye[4];
    static rigidtform2d previousTform;
    static double vehiclePolygon[8];
    static double anchorImagePose[3];
    static double anchorWorldPose[3];
    static double localOrigin[2];
    static double pixelExtentInWorldX;
    static double pixelExtentInWorldY;
    static float xLimitGlobal[2];
    static float yLimitGlobal[2];
    static const signed char b_iv[3] = {0, 0, 1};

    ORBPoints a__2;
    ORBPoints a__3;
    cell_wrap_1 BEV[4];
    emxArray_boolean_T *inliers;
    emxArray_boolean_T *maskImg;
    emxArray_boolean_T *r;
    emxArray_int32_T *r1;
    emxArray_int32_T *r3;
    emxArray_real_T *currViewImg;
    emxArray_real_T *outputImage;
    emxArray_real_T *r4;
    imref2d currRef;
    double b_U[9];
    double t5_T[9];
    double U[3];
    double Ref_ImageSizeAlias[2];
    double xLimitsIn[2];
    double yLimitsIn[2];
    double extentX;
    double extentY;
    double *outputImage_data;
    double *previousImage_data;
    double *vehicleShowPts_data;
    float B[9];
    float beta[9];
    float d_r1[9];
    float relTform_Translation[2];
    float b_r1;
    float b_r2;
    float c_r1;
    float c_r2;
    float f;
    float r2_tmp;
    int end;
    int i;
    int trueCount;
    int vlen;
    int *r2;
    bool close_enough;
    bool guard1 = false;
    bool *BW_data;
    bool *maskImg_data;
    BW_data = BW->data;
    vehicleShowPts_data = vehicleShowPts->data;

    /*  Brief: 用于C代码生成的入口函数，由mainBuildMap.m改写 */
    /*  Details: */
    /*     None */
    /*   */
    /*  Syntax:   */
    /*      outputSt = constructWorldMap(inputArgs,birdsEye360)%#codegen */
    /*   */
    /*  Inputs: */
    /*     inputArgs - [m,n] size,[double] type,Description */
    /*     birdsEye360 - [m,n] size,[double] type,Description */
    /*   */
    /*  Outputs: */
    /*     outputStruct - [1,1] size,[struct] type,Description */
    /*   */
    /*  Example:  */
    /*     None */
    /*   */
    /*  See also: None */
    /*  Author:                          cuixingxing */
    /*  Email:                           xingxing.cui@long-horn.com */
    /*  Created:                         25-Oct-2022 15:49:42 */
    /*  Version history revision notes: */
    /*                                   None */
    /*  Implementation In Matlab R2022b */
    /*  Copyright © 2022 long-horn.All Rights Reserved. */
    /*  */
    if (!isloadParams_not_empty) {
        /*  预先鱼眼相机标定好的各类参数 */
        /*  front,left,back,right分别到front图像的刚性变换矩阵3*3 */
        helperToObjectBev(birdsEye360->birdsEye[0].OutputView, birdsEye360->birdsEye[0].ImageSize, birdsEye360->birdsEye[0].Sensor.Intrinsics.FocalLength, birdsEye360->birdsEye[0].Sensor.Intrinsics.PrincipalPoint, birdsEye360->birdsEye[0].Sensor.Intrinsics.Skew, birdsEye360->birdsEye[0].Sensor.Height, birdsEye360->birdsEye[0].Sensor.Pitch,
                          birdsEye360->birdsEye[0].Sensor.Yaw, birdsEye360->birdsEye[0].Sensor.Roll, birdsEye360->birdsEye[0].Sensor.SensorLocation, birdsEye[0].OutputView, birdsEye[0].ImageSize, &birdsEye[0].Sensor, &birdsEye[0].OutputViewImref, birdsEye[0].Scale);
        for (i = 0; i < 9; i++) {
            beta[i] = (float)birdsEye360->tforms[0].A[i];
        }

        for (i = 0; i < 3; i++) {
            beta[3 * i + 2] = b_iv[i];
            trueCount = i << 1;
            tforms[0].A23[trueCount] = beta[3 * i];
            tforms[0].A23[trueCount + 1] = beta[3 * i + 1];
        }

        helperToObjectBev(birdsEye360->birdsEye[1].OutputView, birdsEye360->birdsEye[1].ImageSize, birdsEye360->birdsEye[1].Sensor.Intrinsics.FocalLength, birdsEye360->birdsEye[1].Sensor.Intrinsics.PrincipalPoint, birdsEye360->birdsEye[1].Sensor.Intrinsics.Skew, birdsEye360->birdsEye[1].Sensor.Height, birdsEye360->birdsEye[1].Sensor.Pitch,
                          birdsEye360->birdsEye[1].Sensor.Yaw, birdsEye360->birdsEye[1].Sensor.Roll, birdsEye360->birdsEye[1].Sensor.SensorLocation, birdsEye[1].OutputView, birdsEye[1].ImageSize, &birdsEye[1].Sensor, &birdsEye[1].OutputViewImref, birdsEye[1].Scale);
        for (i = 0; i < 9; i++) {
            beta[i] = (float)birdsEye360->tforms[1].A[i];
        }

        for (i = 0; i < 3; i++) {
            beta[3 * i + 2] = b_iv[i];
            trueCount = i << 1;
            tforms[1].A23[trueCount] = beta[3 * i];
            tforms[1].A23[trueCount + 1] = beta[3 * i + 1];
        }

        helperToObjectBev(birdsEye360->birdsEye[2].OutputView, birdsEye360->birdsEye[2].ImageSize, birdsEye360->birdsEye[2].Sensor.Intrinsics.FocalLength, birdsEye360->birdsEye[2].Sensor.Intrinsics.PrincipalPoint, birdsEye360->birdsEye[2].Sensor.Intrinsics.Skew, birdsEye360->birdsEye[2].Sensor.Height, birdsEye360->birdsEye[2].Sensor.Pitch,
                          birdsEye360->birdsEye[2].Sensor.Yaw, birdsEye360->birdsEye[2].Sensor.Roll, birdsEye360->birdsEye[2].Sensor.SensorLocation, birdsEye[2].OutputView, birdsEye[2].ImageSize, &birdsEye[2].Sensor, &birdsEye[2].OutputViewImref, birdsEye[2].Scale);
        for (i = 0; i < 9; i++) {
            beta[i] = (float)birdsEye360->tforms[2].A[i];
        }

        for (i = 0; i < 3; i++) {
            beta[3 * i + 2] = b_iv[i];
            trueCount = i << 1;
            tforms[2].A23[trueCount] = beta[3 * i];
            tforms[2].A23[trueCount + 1] = beta[3 * i + 1];
        }

        helperToObjectBev(birdsEye360->birdsEye[3].OutputView, birdsEye360->birdsEye[3].ImageSize, birdsEye360->birdsEye[3].Sensor.Intrinsics.FocalLength, birdsEye360->birdsEye[3].Sensor.Intrinsics.PrincipalPoint, birdsEye360->birdsEye[3].Sensor.Intrinsics.Skew, birdsEye360->birdsEye[3].Sensor.Height, birdsEye360->birdsEye[3].Sensor.Pitch,
                          birdsEye360->birdsEye[3].Sensor.Yaw, birdsEye360->birdsEye[3].Sensor.Roll, birdsEye360->birdsEye[3].Sensor.SensorLocation, birdsEye[3].OutputView, birdsEye[3].ImageSize, &birdsEye[3].Sensor, &birdsEye[3].OutputViewImref, birdsEye[3].Scale);
        for (i = 0; i < 9; i++) {
            beta[i] = (float)birdsEye360->tforms[3].A[i];
        }

        for (i = 0; i < 3; i++) {
            beta[3 * i + 2] = b_iv[i];
            trueCount = i << 1;
            tforms[3].A23[trueCount] = beta[3 * i];
            tforms[3].A23[trueCount + 1] = beta[3 * i + 1];
        }

        /*  initialize */
        xLimitGlobal[0] = 1.0F;
        yLimitGlobal[0] = 1.0F;
        bigImgSt.ref.XWorldLimits[0] = 0.5;
        bigImgSt.ref.YWorldLimits[0] = 0.5;
        bigImgSt.ref.ImageSizeAlias[0] = 2.0;
        xLimitGlobal[1] = 1.0F;
        yLimitGlobal[1] = 1.0F;
        bigImgSt.ref.XWorldLimits[1] = 2.5;
        bigImgSt.ref.YWorldLimits[1] = 2.5;
        bigImgSt.ref.ImageSizeAlias[1] = 2.0;
        bigImgSt.ref.ForcePixelExtentToOne = true;
        bigImgSt.bigImg->size[0] = 0;
        bigImgSt.bigImg->size[1] = 1;
        bigImgSt.bigImg->size[2] = 3;

        /*  大图上绘图轨迹点,n*2大小 */
        vehicleShowPts->size[0] = 0;
        vehicleShowPts->size[1] = 2;

        /*      coder.varsize("vehicleShowPts",[inf,2]); */
        /*  get params */
        /*  [xmin,xmax,ymin,ymax] in vehicle coordinates */
        pixelExtentInWorldX = (birdsEye[0].OutputView[3] - birdsEye[0].OutputView[2]) / birdsEye[0].ImageSize[1];
        pixelExtentInWorldY = (birdsEye[0].OutputView[1] - birdsEye[0].OutputView[0]) / birdsEye[0].ImageSize[0];
        isloadParams_not_empty = true;
    }

    i = outputStruct->HDmap.bigImg->size[0] * outputStruct->HDmap.bigImg->size[1] *
        outputStruct->HDmap.bigImg->size[2];
    outputStruct->HDmap.bigImg->size[0] = bigImgSt.bigImg->size[0];
    outputStruct->HDmap.bigImg->size[1] = bigImgSt.bigImg->size[1];
    outputStruct->HDmap.bigImg->size[2] = 3;
    emxEnsureCapacity_real_T(outputStruct->HDmap.bigImg, i);
    vlen = bigImgSt.bigImg->size[0] * bigImgSt.bigImg->size[1] * 3;
    for (i = 0; i < vlen; i++) {
        outputStruct->HDmap.bigImg->data[i] = bigImgSt.bigImg->data[i];
    }

    outputStruct->HDmap.ref.XWorldLimits[0] = bigImgSt.ref.XWorldLimits[0];
    outputStruct->HDmap.ref.YWorldLimits[0] = bigImgSt.ref.YWorldLimits[0];
    outputStruct->HDmap.ref.ImageSize[0] = bigImgSt.ref.ImageSizeAlias[0];
    outputStruct->HDmap.ref.XWorldLimits[1] = bigImgSt.ref.XWorldLimits[1];
    outputStruct->HDmap.ref.YWorldLimits[1] = bigImgSt.ref.YWorldLimits[1];
    outputStruct->HDmap.ref.ImageSize[1] = bigImgSt.ref.ImageSizeAlias[1];
    i = outputStruct->vehicleTraj->size[0] * outputStruct->vehicleTraj->size[1];
    outputStruct->vehicleTraj->size[0] = vehicleShowPts->size[0];
    outputStruct->vehicleTraj->size[1] = 2;
    emxEnsureCapacity_real_T(outputStruct->vehicleTraj, i);
    vlen = vehicleShowPts->size[0] * 2;
    for (i = 0; i < vlen; i++) {
        outputStruct->vehicleTraj->data[i] = vehicleShowPts_data[i];
    }

    /*  4副鱼眼图像+姿态[x,y,theta] */
    /*  1*4 cell大小 */
    /*  [x,y,theta],theta in radius,currFrontBasePose可以认为是拍摄四副鱼眼图像的位置姿态，定义位于车辆front摄像头光心位置 */
    /*  vehicleImg = inputArgs.vehicleImg;% ego vehicle logo image, single，[0,1]范围 */
    /*  是否使用童工/郭工的SLAM里程计数据 */
    /*  main loop */
    emxInitMatrix_cell_wrap_1(BEV);
    birdsEyeView_transformImage(birdsEye[0].OutputView, birdsEye[0].Sensor.Intrinsics.K, birdsEye[0].Sensor.Height, birdsEye[0].Sensor.Pitch,
                                birdsEye[0].Sensor.Yaw, birdsEye[0].Sensor.Roll, birdsEye[0].Sensor.SensorLocation, birdsEye[0].OutputViewImref.ImageSizeAlias, birdsEye[0].Scale, inputArgs->undistortImages[0].f1, BEV[0].f1);
    birdsEyeView_transformImage(birdsEye[1].OutputView, birdsEye[1].Sensor.Intrinsics.K, birdsEye[1].Sensor.Height, birdsEye[1].Sensor.Pitch,
                                birdsEye[1].Sensor.Yaw, birdsEye[1].Sensor.Roll, birdsEye[1].Sensor.SensorLocation, birdsEye[1].OutputViewImref.ImageSizeAlias, birdsEye[1].Scale, inputArgs->undistortImages[1].f1, BEV[1].f1);
    birdsEyeView_transformImage(birdsEye[2].OutputView, birdsEye[2].Sensor.Intrinsics.K, birdsEye[2].Sensor.Height, birdsEye[2].Sensor.Pitch,
                                birdsEye[2].Sensor.Yaw, birdsEye[2].Sensor.Roll, birdsEye[2].Sensor.SensorLocation, birdsEye[2].OutputViewImref.ImageSizeAlias, birdsEye[2].Scale, inputArgs->undistortImages[2].f1, BEV[2].f1);
    birdsEyeView_transformImage(birdsEye[3].OutputView, birdsEye[3].Sensor.Intrinsics.K, birdsEye[3].Sensor.Height, birdsEye[3].Sensor.Pitch,
                                birdsEye[3].Sensor.Yaw, birdsEye[3].Sensor.Roll, birdsEye[3].Sensor.SensorLocation, birdsEye[3].OutputViewImref.ImageSizeAlias, birdsEye[3].Scale, inputArgs->undistortImages[3].f1, BEV[3].f1);
    emxInit_real_T(&outputImage, 3);
    helperStitchImages(BEV, tforms, outputImage, yLimitsIn, xLimitsIn,
                       Ref_ImageSizeAlias, &close_enough);

    outputImage_data = outputImage->data;
    emxFreeMatrix_cell_wrap_1(BEV);
    emxInit_boolean_T(&r, 3);
    i = r->size[0] * r->size[1] * r->size[2];
    r->size[0] = outputImage->size[0];
    r->size[1] = outputImage->size[1];
    r->size[2] = 3;
    emxEnsureCapacity_boolean_T(r, i);
    maskImg_data = r->data;
    vlen = outputImage->size[0] * outputImage->size[1] * 3;
    for (i = 0; i < vlen; i++) {
        maskImg_data[i] = rtIsNaN(outputImage_data[i]);
    }

    end = r->size[0] * (r->size[1] * 3) - 1;
    trueCount = 0;
    for (i = 0; i <= end; i++) {
        if (maskImg_data[i]) {
            trueCount++;
        }
    }

    emxInit_int32_T(&r1, 1);
    i = r1->size[0];
    r1->size[0] = trueCount;
    emxEnsureCapacity_int32_T(r1, i);
    r2 = r1->data;
    vlen = 0;
    for (i = 0; i <= end; i++) {
        if (maskImg_data[i]) {
            r2[vlen] = i + 1;
            vlen++;
        }
    }

    vlen = r1->size[0];
    for (i = 0; i < vlen; i++) {
        outputImage_data[r2[i] - 1] = 0.0;
    }

    emxFree_int32_T(&r1);

    /*      imwrite(outputImage,"results/"+string(num)+".jpg"); */
    /*  build big map,以第一副图像为图像坐标系 */
    if (!isinitialized_not_empty) {
        double updateROI[16];
        double localImagePts[8];

        /*  step1:人为定义一个任意形状的多边形范围，一般以车中心附近较好，图像清晰， */
        /*  畸变较小，用于累计构建大图。 */
        /*  坐标基准为当前四副鱼眼环视图鸟瞰图拼接的前视图的相机中心为原点的车辆坐标系。 */
        /*  左下 */
        /*  左上 */
        /* 右上 */
        /* 右下 */
        vehicleToLocalImage(birdsEye[0].OutputView, birdsEye[0].Sensor.Intrinsics.K,
                            birdsEye[0].Sensor.Height, birdsEye[0].Sensor.Pitch,
                            birdsEye[0].Sensor.Yaw, birdsEye[0].Sensor.Roll,
                            birdsEye[0].Sensor.SensorLocation, birdsEye[0].Scale,
                            yLimitsIn, xLimitsIn, Ref_ImageSizeAlias, close_enough,
                            localImagePts);

        /*     v = localImagePts';% 预览范围 */
        /*     vv = (v(:))'; */
        /*     RGB = insertShape(outputImage,'polygon',vv); */
        /*     figure;imshow(RGB) */
        /*  由于匹配图像中有车辆存在，故车辆范围内的特征点不考虑匹配 */
        /*  左下 */
        /*  左上 */
        /* 右上 */
        /* 右下 */
        b_vehicleToLocalImage(birdsEye[0].OutputView, birdsEye[0].Sensor.Intrinsics.K, birdsEye[0].Sensor.Height,
                              birdsEye[0].Sensor.Pitch, birdsEye[0].Sensor.Yaw,
                              birdsEye[0].Sensor.Roll, birdsEye[0].Sensor.SensorLocation, birdsEye[0].Scale, yLimitsIn,
                              xLimitsIn, Ref_ImageSizeAlias, close_enough,
                              vehiclePolygon);

        /*          v = vehiclePolygon';% 预览范围 */
        /*          vv = (v(:))'; */
        /*          RGB = insertShape(outputImage,'polygon',vv); */
        /*          figure;imshow(RGB) */
        for (i = 0; i < 2; i++) {
            vlen = i << 2;
            trueCount = i << 3;
            updateROI[trueCount] = localImagePts[vlen];
            updateROI[trueCount + 1] = localImagePts[vlen + 1];
            updateROI[trueCount + 2] = localImagePts[vlen + 2];
            updateROI[trueCount + 3] = localImagePts[vlen + 3];
        }

        updateROI[4] = vehiclePolygon[2];
        updateROI[12] = localImagePts[4];
        updateROI[5] = vehiclePolygon[2];
        updateROI[13] = vehiclePolygon[6];
        updateROI[6] = vehiclePolygon[1];
        updateROI[14] = vehiclePolygon[5];
        updateROI[7] = vehiclePolygon[1];
        updateROI[15] = localImagePts[4];

        /* 这里为手写定义需要更新的ROI,注意顺序 */
        poly2mask(&updateROI[0], &updateROI[8], outputImage->size[0],
                  outputImage->size[1], BW);
        BW_data = BW->data;

        /* 此区域为更新区域 */
        /*      BWVehicle = poly2mask(vehiclePolygon(:,1),vehiclePolygon(:,2),h,w); */
        /*      vehicleXLims = [min(vehiclePolygon(:,1)),max(vehiclePolygon(:,1))]; */
        /*      vehicleYLims = [min(vehiclePolygon(:,2)),max(vehiclePolygon(:,2))]; */
        /*      vHight = round(vehicleYLims(2)-vehicleYLims(1)); */
        /*      vWidth = round(vehicleXLims(2)-vehicleXLims(1)); */
        /*      vBW = imresize(vBW,[vHight,vWidth]); */
        /*      rowStart = round(vehicleYLims(1)); */
        /*      colStart = round(vehicleXLims(1)); */
        /*      BWVehicle(rowStart:rowStart+vHight-1,colStart:colStart+vWidth-1)=vBW; */
        /*  step2:世界坐标系姿态转换为全局图像坐标姿态/第一副图像坐标系 */
        /*  以第一副图像坐标系为基准 */
        /*  Brief: 在2D鸟瞰图中把局部vehiclePts转换到当前四副鱼眼环视图拼接的局部图像坐标 */
        /*  Details: */
        /*     vehicle坐标系符合matlab自动驾驶工具箱定义，即x轴朝向车辆正前方，y轴朝向车辆左侧。 */
        /*   */
        /*  Syntax:   */
        /*      localImagePts = vehicleToLocalImage(refBirdsEye,refImg,vehiclePts) */
        /*   */
        /*  Inputs: */
        /*     refBirdsEye - [1,1] size,[birdsEyeView] type,matlab build-in type */
        /*     refImg - [1,1] size,[imref2d] type,matlab build-in type,四副环视拼接参考对象 */
        /*     vehiclePts - [m,2] size,[double] type,待转换到局部车辆坐标点 */
        /*   */
        /*  Outputs: */
        /*     localImagePts - [m,2] size,[double] type,四副环视拼接的鸟瞰图像中的局部像素坐标 */
        /*   */
        /*  Example:  */
        /*     None */
        /*   */
        /*  See also: None */
        /*  Author:                          cuixingxing */
        /*  Email:                           xingxing.cui@long-horn.com */
        /*  Created:                         13-Oct-2022 09:00:37 */
        /*  Version history revision notes: */
        /*                                   None */
        /*  Implementation In Matlab R2022b */
        /*  Copyright © 2022 long-horn.All Rights Reserved. */
        /*  */
        c_birdsEyeView_get_ImageToVehic(birdsEye[0].OutputView, birdsEye[0].Sensor.Intrinsics.K, birdsEye[0].Sensor.Height, birdsEye[0].Sensor.Pitch,
                                        birdsEye[0].Sensor.Yaw, birdsEye[0].Sensor.Roll, birdsEye[0].Sensor.SensorLocation, birdsEye[0].Scale, t5_T);
        inv(t5_T, b_U);
        for (i = 0; i < 3; i++) {
            U[i] = (0.0 * b_U[3 * i] + 0.0 * b_U[3 * i + 1]) + b_U[3 * i + 2];
        }

        U[0] /= U[2];
        U[1] /= U[2];
        if (close_enough) {
            extentX = 1.0;
            extentY = 1.0;
        } else {
            extentX = (yLimitsIn[1] - yLimitsIn[0]) / Ref_ImageSizeAlias[1];
            extentY = (xLimitsIn[1] - xLimitsIn[0]) / Ref_ImageSizeAlias[0];
        }

        localOrigin[0] = (U[0] - yLimitsIn[0]) / extentX + 0.5;
        localOrigin[1] = (U[1] - xLimitsIn[0]) / extentY + 0.5;

        /*  前摄像头中心才是基准 */
        anchorWorldPose[0] = inputArgs->currFrontBasePose[0];
        anchorWorldPose[1] = inputArgs->currFrontBasePose[1];
        anchorWorldPose[2] = inputArgs->currFrontBasePose[2];
        yLimitsIn[0] = pixelExtentInWorldX;
        yLimitsIn[1] = pixelExtentInWorldY;
        worldToGlobalImagePose(anchorWorldPose, anchorWorldPose, localOrigin,
                               yLimitsIn, anchorImagePose);
        i = bigImgSt.bigImg->size[0] * bigImgSt.bigImg->size[1] *
            bigImgSt.bigImg->size[2];
        bigImgSt.bigImg->size[0] = outputImage->size[0];
        bigImgSt.bigImg->size[1] = outputImage->size[1];
        bigImgSt.bigImg->size[2] = 3;
        emxEnsureCapacity_real_T(bigImgSt.bigImg, i);
        vlen = outputImage->size[0] * outputImage->size[1] * 3;
        for (i = 0; i < vlen; i++) {
            bigImgSt.bigImg->data[i] = outputImage_data[i];
        }

        bigImgSt.ref.ImageSizeAlias[0] = outputImage->size[0];
        bigImgSt.ref.ImageSizeAlias[1] = outputImage->size[1];
        bigImgSt.ref.XWorldLimits[0] = 0.5;
        bigImgSt.ref.XWorldLimits[1] = (double)outputImage->size[1] + 0.5;
        bigImgSt.ref.YWorldLimits[0] = 0.5;
        bigImgSt.ref.YWorldLimits[1] = (double)outputImage->size[0] + 0.5;
        bigImgSt.ref.ForcePixelExtentToOne = true;
        i = outputStruct->HDmap.bigImg->size[0] * outputStruct->HDmap.bigImg->size[1] * outputStruct->HDmap.bigImg->size[2];
        outputStruct->HDmap.bigImg->size[0] = bigImgSt.bigImg->size[0];
        outputStruct->HDmap.bigImg->size[1] = bigImgSt.bigImg->size[1];
        outputStruct->HDmap.bigImg->size[2] = 3;
        emxEnsureCapacity_real_T(outputStruct->HDmap.bigImg, i);
        vlen = bigImgSt.bigImg->size[0] * bigImgSt.bigImg->size[1] * 3;
        for (i = 0; i < vlen; i++) {
            outputStruct->HDmap.bigImg->data[i] = bigImgSt.bigImg->data[i];
        }

        mean(vehiclePolygon, xLimitsIn);
        i = outputStruct->vehicleTraj->size[0] * outputStruct->vehicleTraj->size[1];
        outputStruct->vehicleTraj->size[0] = 1;
        outputStruct->vehicleTraj->size[1] = 2;
        emxEnsureCapacity_real_T(outputStruct->vehicleTraj, i);
        outputStruct->HDmap.ref.XWorldLimits[0] = bigImgSt.ref.XWorldLimits[0];
        outputStruct->HDmap.ref.YWorldLimits[0] = bigImgSt.ref.YWorldLimits[0];
        outputStruct->HDmap.ref.ImageSize[0] = bigImgSt.ref.ImageSizeAlias[0];
        outputStruct->vehicleTraj->data[0] = xLimitsIn[0];
        outputStruct->HDmap.ref.XWorldLimits[1] = bigImgSt.ref.XWorldLimits[1];
        outputStruct->HDmap.ref.YWorldLimits[1] = bigImgSt.ref.YWorldLimits[1];
        outputStruct->HDmap.ref.ImageSize[1] = bigImgSt.ref.ImageSizeAlias[1];
        outputStruct->vehicleTraj->data[1] = xLimitsIn[1];
        i = previousImage->size[0] * previousImage->size[1] * previousImage->size[2];
        previousImage->size[0] = outputImage->size[0];
        previousImage->size[1] = outputImage->size[1];
        previousImage->size[2] = 3;
        emxEnsureCapacity_real_T(previousImage, i);
        previousImage_data = previousImage->data;
        vlen = outputImage->size[0] * outputImage->size[1] * 3;
        for (i = 0; i < vlen; i++) {
            previousImage_data[i] = outputImage_data[i];
        }

        previousTform.RotationAngle = 0.0F;
        previousTform.Translation[0] = 0.0F;
        previousTform.Translation[1] = 0.0F;

        /*      imshow(outputImage,outputView,'Parent',ax1); */
        isinitialized_not_empty = true;
    }

    /*  计算转换姿态 */
    emxInit_real_T(&currViewImg, 3);
    emxInit_boolean_T(&maskImg, 2);
    emxInit_boolean_T(&inliers, 1);
    emxInit_int32_T(&r3, 1);
    emxInitStruct_ORBPoints(&a__2);
    emxInitStruct_ORBPoints(&a__3);
    emxInit_real_T(&r4, 2);
    guard1 = false;
    if (inputArgs->isuseGT) {
        /*  这里通过3D SLAM获取准确位姿 */
        yLimitsIn[0] = pixelExtentInWorldX;
        yLimitsIn[1] = pixelExtentInWorldY;
        worldToGlobalImagePose(anchorWorldPose, inputArgs->currFrontBasePose,
                               localOrigin, yLimitsIn, U);
        relTform_Translation[0] = (float)(U[0] - anchorImagePose[0]);
        relTform_Translation[1] = (float)(U[1] - anchorImagePose[1]);
        b_r2 = b_mod((float)(U[2] - anchorImagePose[2]) + 180.0F) - 180.0F;
        b_r1 = roundf(b_r2);
        if (b_r2 == b_r1) {
            close_enough = true;
        } else {
            c_r1 = fabsf(b_r2 - b_r1);
            if ((b_r2 == 0.0F) || (b_r1 == 0.0F)) {
                close_enough = (c_r1 < 1.4013E-45F);
            } else {
                f = fabsf(b_r2) + fabsf(b_r1);
                if (f < 1.17549435E-38F) {
                    close_enough = (c_r1 < 1.4013E-45F);
                } else {
                    close_enough = (c_r1 / fminf(f, 3.402823466E+38F) < 1.1920929E-7F);
                }
            }
        }

        if (close_enough) {
            b_r2 = b_r1;
        }

        guard1 = true;
    } else {
        /*  使用orb特征匹配估计转换姿态 */
        estiTform(previousImage, outputImage, vehiclePolygon, &b_r2,
                  relTform_Translation, &a__2, &a__3, inliers);
        maskImg_data = inliers->data;
        vlen = inliers->size[0];
        if (inliers->size[0] == 0) {
            trueCount = 0;
        } else {
            trueCount = maskImg_data[0];
            for (end = 2; end <= vlen; end++) {
                trueCount += maskImg_data[end - 1];
            }
        }

        if (!((double)trueCount / (double)inliers->size[0] < 0.6)) {
            float b_beta[4];
            float fv[4];
            b_r1 = b_r2;
            c_cosd(&b_r1);
            c_sind(&b_r2);
            c_r1 = previousTform.RotationAngle;
            c_cosd(&c_r1);
            c_r2 = previousTform.RotationAngle;
            c_sind(&c_r2);
            B[0] = b_r1;
            B[3] = -b_r2;
            B[6] = relTform_Translation[0];
            B[1] = b_r2;
            B[4] = b_r1;
            B[7] = relTform_Translation[1];
            d_r1[0] = c_r1;
            d_r1[3] = -c_r2;
            d_r1[6] = previousTform.Translation[0];
            d_r1[1] = c_r2;
            d_r1[4] = c_r1;
            d_r1[7] = previousTform.Translation[1];
            B[2] = 0.0F;
            d_r1[2] = 0.0F;
            B[5] = 0.0F;
            d_r1[5] = 0.0F;
            B[8] = 1.0F;
            d_r1[8] = 1.0F;
            for (i = 0; i < 3; i++) {
                f = B[i];
                c_r1 = B[i + 3];
                b_r1 = B[i + 6];
                for (trueCount = 0; trueCount < 3; trueCount++) {
                    beta[i + 3 * trueCount] = (f * d_r1[3 * trueCount] + c_r1 * d_r1[3 *
                                                                                         trueCount +
                                                                                     1]) +
                                              b_r1 * d_r1[3 * trueCount + 2];
                }
            }

            b_beta[0] = beta[0];
            b_beta[1] = beta[1];
            b_beta[2] = beta[3];
            b_beta[3] = beta[4];
            constrainToRotationMatrix2D(b_beta, fv, &b_r1);
            b_r2 = b_mod(b_r1 + 180.0F) - 180.0F;
            b_r1 = roundf(b_r2);
            if (b_r2 == b_r1) {
                close_enough = true;
            } else {
                c_r1 = fabsf(b_r2 - b_r1);
                if ((b_r2 == 0.0F) || (b_r1 == 0.0F)) {
                    close_enough = (c_r1 < 1.4013E-45F);
                } else {
                    f = fabsf(b_r2) + fabsf(b_r1);
                    if (f < 1.17549435E-38F) {
                        close_enough = (c_r1 < 1.4013E-45F);
                    } else {
                        close_enough = (c_r1 / fminf(f, 3.402823466E+38F) < 1.1920929E-7F);
                    }
                }
            }

            if (close_enough) {
                b_r2 = b_r1;
            }

            previousTform.RotationAngle = b_r2;
            previousTform.Translation[0] = beta[6];
            previousTform.Translation[1] = beta[7];
            b_r1 = b_r2;
            c_cosd(&b_r1);
            c_sind(&b_r2);
            B[0] = b_r1;
            B[3] = -b_r2;
            B[6] = beta[6];
            B[1] = b_r2;
            B[4] = b_r1;
            B[7] = beta[7];
            B[2] = 0.0F;
            B[5] = 0.0F;
            B[8] = 1.0F;
            b_inv(B, beta);
            b_beta[0] = beta[0];
            b_beta[1] = beta[1];
            b_beta[2] = beta[3];
            b_beta[3] = beta[4];
            constrainToRotationMatrix2D(b_beta, fv, &b_r1);
            b_r2 = b_mod(b_r1 + 180.0F) - 180.0F;
            b_r1 = roundf(b_r2);
            if (b_r2 == b_r1) {
                close_enough = true;
            } else {
                c_r1 = fabsf(b_r2 - b_r1);
                if ((b_r2 == 0.0F) || (b_r1 == 0.0F)) {
                    close_enough = (c_r1 < 1.4013E-45F);
                } else {
                    f = fabsf(b_r2) + fabsf(b_r1);
                    if (f < 1.17549435E-38F) {
                        close_enough = (c_r1 < 1.4013E-45F);
                    } else {
                        close_enough = (c_r1 / fminf(f, 3.402823466E+38F) < 1.1920929E-7F);
                    }
                }
            }

            if (close_enough) {
                b_r2 = b_r1;
            }

            relTform_Translation[0] = beta[6];
            relTform_Translation[1] = beta[7];
            guard1 = true;
        } else {
            /*  如果阈值过大，可能跟丢，应多方面考虑，车速、车辆颠簸等 */
            /*  This function converts contructWorldMap function output object to structure format. */
        }
    }

    if (guard1) {
        double dv[3];
        bool b_B[2];

        /*  step3:针对当前4副鸟瞰图做变换到全局图像坐标系 */
        U[0] = 0.5;
        U[1] = (((double)outputImage->size[1] + 0.5) + 0.5) / 2.0;
        U[2] = (double)outputImage->size[1] + 0.5;
        dv[0] = 0.5;
        dv[1] = (((double)outputImage->size[0] + 0.5) + 0.5) / 2.0;
        dv[2] = (double)outputImage->size[0] + 0.5;
        meshgrid(U, dv, b_U, t5_T);
        c_r2 = b_r2;
        c_cosd(&c_r2);
        r2_tmp = b_r2;
        c_sind(&r2_tmp);
        B[6] = relTform_Translation[0];
        b_B[0] = true;
        b_B[1] = true;
        if (all(b_B)) {
            b_r1 = relTform_Translation[0];
            c_r1 = relTform_Translation[1];
            for (i = 0; i < 9; i++) {
                extentX = b_U[i];
                extentY = t5_T[i];
                d_r1[i] = (c_r2 * (float)extentX + -r2_tmp * (float)extentY) + b_r1;
                B[i] = (r2_tmp * (float)extentX + c_r2 * (float)extentY) + c_r1;
            }
        } else {
            for (i = 0; i < 9; i++) {
                extentX = b_U[i];
                extentY = t5_T[i];
                f = (0.0F * (float)extentX + 0.0F * (float)extentY) + 1.0F;
                beta[i] = f;
                d_r1[i] = ((c_r2 * (float)extentX + -r2_tmp * (float)extentY) + B[6]) /
                          f;
            }

            b_r1 = relTform_Translation[1];
            for (i = 0; i < 9; i++) {
                B[i] = ((r2_tmp * (float)b_U[i] + c_r2 * (float)t5_T[i]) + b_r1) /
                       beta[i];
            }
        }

        /*  Width and height of panorama. */
        f = xLimitGlobal[0];
        c_r1 = xLimitGlobal[1];
        xLimitGlobal[0] = fminf(f, fminf(minimum(d_r1), 0.5F));
        xLimitGlobal[1] = fmaxf(c_r1, fmaxf((float)((double)outputImage->size[1] +
                                                    0.5),
                                            maximum(d_r1)));
        f = yLimitGlobal[0];
        c_r1 = yLimitGlobal[1];
        yLimitGlobal[0] = fminf(f, fminf(minimum(B), 0.5F));
        yLimitGlobal[1] = fmaxf(c_r1, fmaxf((float)((double)outputImage->size[0] +
                                                    0.5),
                                            maximum(B)));
        Ref_ImageSizeAlias[0] = roundf(yLimitGlobal[1] - yLimitGlobal[0]);
        Ref_ImageSizeAlias[1] = roundf(xLimitGlobal[1] - xLimitGlobal[0]);
        yLimitsIn[0] = xLimitGlobal[0];
        xLimitsIn[0] = yLimitGlobal[0];
        yLimitsIn[1] = xLimitGlobal[1];
        xLimitsIn[1] = yLimitGlobal[1];

        /*  fprintf("outputImage size:(%d,%d).\n",int64(h),int64(w)); */
        i = currViewImg->size[0] * currViewImg->size[1] * currViewImg->size[2];
        currViewImg->size[0] = outputImage->size[0];
        currViewImg->size[1] = outputImage->size[1];
        currViewImg->size[2] = 3;
        emxEnsureCapacity_real_T(currViewImg, i);
        previousImage_data = currViewImg->data;
        vlen = outputImage->size[0] * outputImage->size[1] * 3;
        for (i = 0; i < vlen; i++) {
            previousImage_data[i] = outputImage_data[i];
        }

        currRef.XWorldLimits[0] = xLimitGlobal[0];
        currRef.YWorldLimits[0] = yLimitGlobal[0];
        currRef.ImageSizeAlias[0] = Ref_ImageSizeAlias[0];
        currRef.XWorldLimits[1] = xLimitGlobal[1];
        currRef.YWorldLimits[1] = yLimitGlobal[1];
        currRef.ImageSizeAlias[1] = Ref_ImageSizeAlias[1];
        currRef.ForcePixelExtentToOne = false;
        imwarp(currViewImg, b_r2, relTform_Translation, &currRef);
        previousImage_data = currViewImg->data;
        i = maskImg->size[0] * maskImg->size[1];
        maskImg->size[0] = BW->size[0];
        maskImg->size[1] = BW->size[1];
        emxEnsureCapacity_boolean_T(maskImg, i);
        maskImg_data = maskImg->data;
        vlen = BW->size[0] * BW->size[1];
        for (i = 0; i < vlen; i++) {
            maskImg_data[i] = BW_data[i];
        }

        b_imwarp(maskImg, b_r2, relTform_Translation, yLimitsIn, xLimitsIn,
                 Ref_ImageSizeAlias);
        i = r->size[0] * r->size[1] * r->size[2];
        r->size[0] = currViewImg->size[0];
        r->size[1] = currViewImg->size[1];
        r->size[2] = 3;
        emxEnsureCapacity_boolean_T(r, i);
        maskImg_data = r->data;
        vlen = currViewImg->size[0] * currViewImg->size[1] * 3;
        for (i = 0; i < vlen; i++) {
            maskImg_data[i] = rtIsNaN(previousImage_data[i]);
        }

        end = r->size[0] * (r->size[1] * 3) - 1;
        trueCount = 0;
        for (i = 0; i <= end; i++) {
            if (maskImg_data[i]) {
                trueCount++;
            }
        }

        i = r3->size[0];
        r3->size[0] = trueCount;
        emxEnsureCapacity_int32_T(r3, i);
        r2 = r3->data;
        vlen = 0;
        for (i = 0; i <= end; i++) {
            if (maskImg_data[i]) {
                r2[vlen] = i + 1;
                vlen++;
            }
        }

        vlen = r3->size[0];
        for (i = 0; i < vlen; i++) {
            previousImage_data[r2[i] - 1] = 0.0;
        }

        /*  step4: 融合到大图中去 */
        blendImage(&bigImgSt, currViewImg, currRef, maskImg);
        mean(vehiclePolygon, xLimitsIn);
        B[0] = c_r2;
        B[3] = -r2_tmp;
        B[6] = relTform_Translation[0];
        B[1] = r2_tmp;
        B[4] = c_r2;
        B[7] = relTform_Translation[1];
        B[2] = 0.0F;
        B[5] = 0.0F;
        B[8] = 1.0F;
        b_r1 = (float)xLimitsIn[0];
        c_r1 = (float)xLimitsIn[1];
        for (i = 0; i < 3; i++) {
            U[i] = (B[i] * b_r1 + B[i + 3] * c_r1) + B[i + 6];
        }

        i = r4->size[0] * r4->size[1];
        r4->size[0] = vehicleShowPts->size[0] + 1;
        r4->size[1] = 2;
        emxEnsureCapacity_real_T(r4, i);
        previousImage_data = r4->data;
        vlen = vehicleShowPts->size[0];
        for (i = 0; i < 2; i++) {
            for (trueCount = 0; trueCount < vlen; trueCount++) {
                previousImage_data[trueCount + r4->size[0] * i] =
                    vehicleShowPts_data[trueCount + vehicleShowPts->size[0] * i];
            }
        }

        previousImage_data[vehicleShowPts->size[0]] = U[0];
        previousImage_data[vehicleShowPts->size[0] + r4->size[0]] = U[1];
        i = vehicleShowPts->size[0] * vehicleShowPts->size[1];
        vehicleShowPts->size[0] = r4->size[0];
        vehicleShowPts->size[1] = 2;
        emxEnsureCapacity_real_T(vehicleShowPts, i);
        vehicleShowPts_data = vehicleShowPts->data;
        vlen = r4->size[0] * 2;
        for (i = 0; i < vlen; i++) {
            vehicleShowPts_data[i] = previousImage_data[i];
        }

        /*  This function converts contructWorldMap function output object to structure format. */
        i = outputStruct->HDmap.bigImg->size[0] * outputStruct->HDmap.bigImg->size[1] * outputStruct->HDmap.bigImg->size[2];
        outputStruct->HDmap.bigImg->size[0] = bigImgSt.bigImg->size[0];
        outputStruct->HDmap.bigImg->size[1] = bigImgSt.bigImg->size[1];
        outputStruct->HDmap.bigImg->size[2] = 3;
        emxEnsureCapacity_real_T(outputStruct->HDmap.bigImg, i);
        vlen = bigImgSt.bigImg->size[0] * bigImgSt.bigImg->size[1] * 3;
        for (i = 0; i < vlen; i++) {
            outputStruct->HDmap.bigImg->data[i] = bigImgSt.bigImg->data[i];
        }

        outputStruct->HDmap.ref.ImageSize[0] = bigImgSt.ref.ImageSizeAlias[0];
        outputStruct->HDmap.ref.XWorldLimits[0] = bigImgSt.ref.XWorldLimits[0];
        outputStruct->HDmap.ref.YWorldLimits[0] = bigImgSt.ref.YWorldLimits[0];
        outputStruct->HDmap.ref.ImageSize[1] = bigImgSt.ref.ImageSizeAlias[1];
        outputStruct->HDmap.ref.XWorldLimits[1] = bigImgSt.ref.XWorldLimits[1];
        outputStruct->HDmap.ref.YWorldLimits[1] = bigImgSt.ref.YWorldLimits[1];
        i = outputStruct->vehicleTraj->size[0] * outputStruct->vehicleTraj->size[1];
        outputStruct->vehicleTraj->size[0] = vehicleShowPts->size[0];
        outputStruct->vehicleTraj->size[1] = 2;
        emxEnsureCapacity_real_T(outputStruct->vehicleTraj, i);
        vlen = vehicleShowPts->size[0] * 2;
        for (i = 0; i < vlen; i++) {
            outputStruct->vehicleTraj->data[i] = vehicleShowPts_data[i];
        }

        i = previousImage->size[0] * previousImage->size[1] * previousImage->size[2];
        previousImage->size[0] = outputImage->size[0];
        previousImage->size[1] = outputImage->size[1];
        previousImage->size[2] = 3;
        emxEnsureCapacity_real_T(previousImage, i);
        previousImage_data = previousImage->data;
        vlen = outputImage->size[0] * outputImage->size[1] * 3;
        for (i = 0; i < vlen; i++) {
            previousImage_data[i] = outputImage_data[i];
        }
    }

    emxFree_real_T(&r4);
    emxFreeStruct_ORBPoints(&a__3);
    emxFreeStruct_ORBPoints(&a__2);
    emxFree_boolean_T(&r);
    emxFree_int32_T(&r3);
    emxFree_boolean_T(&inliers);
    emxFree_boolean_T(&maskImg);
    emxFree_real_T(&currViewImg);
    emxFree_real_T(&outputImage);
}

/**
 * @fn             : constructWorldMap_initialize
 * @brief          :
 * @param          : void
 * @return         : void
 */
void constructWorldMap_initialize(void) {
    omp_init_nest_lock(&constructWorldMap_nestLockGlobal);
    lookupTable_not_empty = false;
    isinitialized_not_empty = false;
    isloadParams_not_empty = false;
    constructWorldMap_init();
    blendImage_init();
    c_eml_rand_mt19937ar_stateful_i();
}

/**
 * @fn             : constructWorldMap_terminate
 * @brief          :
 * @param          : void
 * @return         : void
 */
void constructWorldMap_terminate(void) {
    blendImage_free();
    constructWorldMap_free();
    omp_destroy_nest_lock(&constructWorldMap_nestLockGlobal);
}

/**
 * File trailer for constructWorldMap.c
 *
 * [EOF]
 */
