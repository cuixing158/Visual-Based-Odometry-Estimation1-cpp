//////////////////////////////////////////////////////////////////////////////
// OpenCV ORB detector wrapper for code generation
//
// Copyright 2018 The MathWorks, Inc.
//
//////////////////////////////////////////////////////////////////////////////

#ifndef COMPILE_FOR_VISION_BUILTINS
// vision_builtins does not need this source file

#include "detectORBCore_api.hpp"

#include "opencv2/opencv.hpp"
#include "cgCommon.hpp"

using namespace std;

//////////////////////////////////////////////////////////////////////////////
// Invoke OpenCV cvDetectORB
//////////////////////////////////////////////////////////////////////////////
int32_T detectORBCompute(
    uint8_T *inImg,
    int32_T nRows, int32_T nCols, int numFeatures, float scaleFactor,
    int numLevels, int edgeThreshold, int firstLevel,
    int samplingPairs, int score_Type, int patchSize, int fastThreshold,
    void **outKeypoints, bool isCM) {
    // Use OpenCV smart pointer to manage image
    cv::Ptr<cv::Mat> inImage = new cv::Mat;
    bool isRGB = false;  // only grayscale image allowed

    if (isCM) {
        cArrayToMat<uint8_T>(inImg, nRows, nCols, isRGB, *inImage);
    } else {
        cArrayToMat_RowMaj<uint8_T>(inImg, nRows, nCols, isRGB, *inImage);
    }

    cv::ORB::ScoreType scoreType = static_cast<cv::ORB::ScoreType>(score_Type);

    // keypoints
    vector<cv::KeyPoint> *ptrKeypoints = (vector<cv::KeyPoint> *)new vector<cv::KeyPoint>();
    *outKeypoints = ptrKeypoints;
    vector<cv::KeyPoint> &refKeypoints = *ptrKeypoints;

    try {
        cv::Ptr<cv::ORB> orb = cv::ORB::create(numFeatures,
                                               scaleFactor,
                                               numLevels,
                                               edgeThreshold,
                                               firstLevel,
                                               samplingPairs,
                                               scoreType,
                                               patchSize,
                                               fastThreshold);
        orb->detect(*inImage, refKeypoints, cv::Mat());
    } catch (...) {
        CV_Error(cv::Error::StsNotImplemented, "OpenCV was built without ORB support");
    }

    return ((int32_T)(refKeypoints.size()));  //return the size of keypoints.
}

int32_T detectORBComputeCM(
    uint8_T *inImg,
    int32_T nRows, int32_T nCols,
    int numFeatures, float scaleFactor,
    int numLevels, int edgeThreshold, int firstLevel,
    int samplingPairs, int scoreType, int patchSize, int fastThreshold,
    void **outKeypoints) {
    return detectORBCompute(inImg,
                            nRows,
                            nCols,
                            numFeatures,
                            scaleFactor,
                            numLevels,
                            edgeThreshold,
                            firstLevel,
                            samplingPairs,
                            scoreType,
                            patchSize,
                            fastThreshold,
                            outKeypoints,
                            true);
}

int32_T detectORBComputeRM(
    uint8_T *inImg,
    int32_T nRows, int32_T nCols,
    int numFeatures, float scaleFactor,
    int numLevels, int edgeThreshold, int firstLevel,
    int samplingPairs, int scoreType, int patchSize, int fastThreshold,
    void **outKeypoints) {
    return detectORBCompute(inImg,
                            nRows,
                            nCols,
                            numFeatures,
                            scaleFactor,
                            numLevels,
                            edgeThreshold,
                            firstLevel,
                            samplingPairs,
                            scoreType,
                            patchSize,
                            fastThreshold,
                            outKeypoints,
                            false);
}

//////////////////////////////////////////////////////////////////////////////
// Assign KeyPoint to Fields
//////////////////////////////////////////////////////////////////////////////
void orbKeyPointToFields(
    void *ptrKeypoints,
    real32_T *outLoc,
    real32_T *outOri,
    real32_T *outMet,
    real32_T *outScl,
    bool isCM) {
    vector<cv::KeyPoint> &keypoints = ((vector<cv::KeyPoint> *)ptrKeypoints)[0];
    size_t m = keypoints.size();

    for (size_t i = 0; i < m; i++) {
        cv::KeyPoint &kp = keypoints[i];

        // convert C 0-index to MATLAB 1-index
        if (isCM) {  // column major
            outLoc[i] = kp.pt.x + 1;
            outLoc[m + i] = kp.pt.y + 1;
        } else {  // row major
            *outLoc++ = kp.pt.x + 1;
            *outLoc++ = kp.pt.y + 1;
        }

        outOri[i] = kp.angle * (float)(CV_PI / 180.0);  // angle to radian
        outMet[i] = kp.response;                        // metric (float)
        outScl[i] = kp.size;
    }

    delete ((vector<cv::KeyPoint> *)ptrKeypoints);
}

void detectORBAssignOutputCM(
    void *ptrKeypoints,
    real32_T *outLoc,
    real32_T *outOri,
    real32_T *outMet,
    real32_T *outScl)

{
    orbKeyPointToFields(ptrKeypoints,
                        outLoc, outOri, outMet, outScl, true);
}

void detectORBAssignOutputRM(
    void *ptrKeypoints,
    real32_T *outLoc,
    real32_T *outOri,
    real32_T *outMet,
    real32_T *outScl)

{
    orbKeyPointToFields(ptrKeypoints,
                        outLoc, outOri, outMet, outScl, false);
}

#endif