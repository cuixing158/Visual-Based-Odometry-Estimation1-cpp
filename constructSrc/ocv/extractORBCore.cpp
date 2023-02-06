//////////////////////////////////////////////////////////////////////////////
// OpenCV ORB extractor wrapper
//
// Copyright 2018 The MathWorks, Inc.
//
//////////////////////////////////////////////////////////////////////////////
#ifndef COMPILE_FOR_VISION_BUILTINS
#include "opencv2/opencv.hpp"

#include "extractORBCore_api.hpp"
#include "cgCommon.hpp"

using namespace std;

//////////////////////////////////////////////////////////////////////////////
// Helper functions
//////////////////////////////////////////////////////////////////////////////

void fieldsToORBKeyPoints(
        real32_T *inLoc,
        real32_T *inOri,
        real32_T *inMet,
        real32_T *inScl,
        bool isCM, int numKPts, float scaleFactor,
        vector<cv::KeyPoint> & keypoints
        ){
    keypoints.reserve(numKPts);
    for (int i = 0; i < numKPts; i ++){
        float x, y;
        // convert to C/C++ 0-indexing
        if (isCM){  // column major
            x = float(inLoc[i]) - 1.0f;
            y = float(inLoc[numKPts + i]) - 1.0f;
        }
        else{       // row major
            x = float(inLoc[i*2]) - 1.0f;
            y = float(inLoc[i*2 + 1]) - 1.0f;
        }
        float orientation = inOri[i];
        float scale = inScl[i];
        float metric = inMet[i];
        int octave= static_cast<int>(round(log((scale/31))/log(scaleFactor)));
        cv::KeyPoint kpt(x, y, scale, orientation, metric, octave,-1);
        keypoints.push_back(kpt);
        
    }
}

void orbKeyPointsToStructFields(
        void *ptrKeypoints,
        real32_T *outLoc,
        real32_T *outOri,
        real32_T *outMet,
        real32_T *outScl,
        bool isCM)
{
    vector<cv::KeyPoint> &keypoints = ((vector<cv::KeyPoint> *)ptrKeypoints)[0];
    size_t nKPts = keypoints.size();
    
    for(size_t i = 0; i < nKPts; i++ ) {
        
        cv::KeyPoint& kp = keypoints[i];
        
        // convert C 0-index to MATLAB 1-index
        if (isCM){  // column major
            outLoc[i]   = kp.pt.x + 1.0f;
            outLoc[nKPts+i] = kp.pt.y + 1.0f;
        }else{      // row major
            *outLoc++   = kp.pt.x + 1.0f;
            *outLoc++   = kp.pt.y + 1.0f;
        }
        
        outOri[i] = kp.angle*(float)(CV_PI/180.0); // angle to radian, should be >= 0 according to OpenCV implementation.
        outMet[i] = kp.response;  // metric (float)
        outScl[i] = kp.size;
        
    }
    
    delete((vector<cv::KeyPoint> *)ptrKeypoints);
}

void orbFeaturesMatToVector(
        void *ptrDescriptors,
        uint8_T *outFeatures,
        bool isCM)
{
    cv::Mat descriptors = *((cv::Mat *)ptrDescriptors);
    
    if (isCM){
        cArrayFromMat<uint8_T>(outFeatures, descriptors);
    }
    else
    {
        cArrayFromMat_RowMaj<uint8_T>(outFeatures, descriptors);
    }
    delete((cv::Mat *)ptrDescriptors);
}

//////////////////////////////////////////////////////////////////////////////
// Invoke OpenCV ORB extractor
//////////////////////////////////////////////////////////////////////////////

int32_T extractORBCompute(
        uint8_T *inImg,
        int32_T nRows, int32_T nCols,
        real32_T *inLoc,
        real32_T *inOri,
        real32_T *inMet,
        real32_T *inScl,
        int numKPts,
        int numFeatures, float scaleFactor,
        int numLevels, int edgeThreshold, int firstLevel,
        int samplingPairs, int score_Type, int patchSize, int fastThreshold,
        void **outKeypoints, void **outFeatures, bool isCM)
{
    // Use OpenCV smart pointer to manage image
    cv::Ptr<cv::Mat> inImage = new cv::Mat;
    bool isRGB = false; // only grayscale image allowed
    
    if (isCM){
        cArrayToMat<uint8_T>(inImg, nRows, nCols, isRGB, *inImage);
    }else{
        cArrayToMat_RowMaj<uint8_T>(inImg, nRows, nCols, isRGB, *inImage);
    }
    cv::ORB::ScoreType scoreType = static_cast<cv::ORB::ScoreType>(score_Type);
    
    // Create KeyPoint array from the given fields
    vector<cv::KeyPoint> *ptrKeypoints = (vector<cv::KeyPoint> *)new vector<cv::KeyPoint>();
    *outKeypoints = ptrKeypoints;
    vector<cv::KeyPoint> &refKeypoints = *ptrKeypoints;
    
    fieldsToORBKeyPoints(inLoc, inOri, inMet, inScl, isCM, numKPts,scaleFactor, refKeypoints);
    
    // Define the feature pointer
    cv::Mat * ptrFeatures = new cv::Mat;
    *outFeatures = ptrFeatures;
    cv::Mat & refFeatures = *ptrFeatures;
    
    // invoke the OpenCV ORB
    try
    {
        cv::Ptr<cv::ORB> orb = cv::ORB::create(numFeatures,
                scaleFactor,
                numLevels,
                edgeThreshold,
                firstLevel,
                samplingPairs,
                scoreType,
                patchSize,
                fastThreshold);
        
        orb->compute(*inImage, refKeypoints, refFeatures);
    }
    catch (...)
    {
        CV_Error(cv::Error::StsNotImplemented, "OpenCV was built without ORB support");
    }
    
    return ((int32_T)(refKeypoints.size())); //actual_numel
}

int32_T extractORBComputeCM(
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
        void **outKeypoints, void **outFeatures){
    return extractORBCompute(inImg, nRows, nCols,
            inLoc, inOri, inMet, inScl, numKPts,
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
            outFeatures,
            true);
}

int32_T extractORBComputeRM(
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
        void **outKeypoints, void **outFeatures){
    return extractORBCompute(inImg, nRows, nCols,
            inLoc, inOri, inMet, inScl, numKPts,
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
            outFeatures,
            false);
}

//////////////////////////////////////////////////////////////////////////////
// Assign KeyPoint to Fields and Descriptor Mat to Array
//////////////////////////////////////////////////////////////////////////////

void extractORBAssignOutputCM(
        void * ptrKeypoints,
        void * ptrFeatures,
        real32_T *outLoc,
        real32_T *outOri,
        real32_T *outMet,
        real32_T *outScl,
        uint8_T *outFtrs)
{
    orbKeyPointsToStructFields(ptrKeypoints, outLoc, outOri, outMet, outScl, true);
    
    orbFeaturesMatToVector(ptrFeatures, outFtrs, true);
}

void extractORBAssignOutputRM(
        void * ptrKeypoints,
        void * ptrFeatures,
        real32_T *outLoc,
        real32_T *outOri,
        real32_T *outMet,
        real32_T *outScl,
        uint8_T *outFtrs)
{
    orbKeyPointsToStructFields(ptrKeypoints, outLoc, outOri, outMet, outScl,  false);
    
    orbFeaturesMatToVector(ptrFeatures, outFtrs, false);
}

#endif