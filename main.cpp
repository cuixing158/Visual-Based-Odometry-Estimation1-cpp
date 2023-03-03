///
/// @file           : main.cpp
/// @target         : Texas Instruments->C6000
/// @details        : for path build map algorithms(loop+pose)
/// @author         : cuixingxing
/// @email          : xingxing.cui@long-horn.com
/// @date           : 02-Mar-2023 10:06:28
/// @version        : V0.1.2
/// @copyright      : Copyright (C) 2023 Long-Horn Inc.All rights reserved.
///

/*************************************************************************/
/* This automatically generated example C++ main file shows how to call  */
/* entry-point functions that MATLAB Coder generated. You must customize */
/* this file for your application. Do not modify this file directly.     */
/* Instead, make a copy of this file, modify it, and integrate it into   */
/* your development environment.                                         */
/*                                                                       */
/* This file initializes entry-point function arguments to a default     */
/* size and value before calling the entry-point functions. It does      */
/* not store or use any values returned from the entry-point functions.  */
/* If necessary, it does pre-allocate memory for returned values.        */
/* You can use this file as a starting point for a main function that    */
/* you can deploy in your application.                                   */
/*                                                                       */
/* After you copy the file, and before you deploy it, you must make the  */
/* following changes:                                                    */
/* * For variable-size function arguments, change the example sizes to   */
/* the sizes that your application requires.                             */
/* * Change the example values of function arguments to the values that  */
/* your application requires.                                            */
/* * If the entry-point functions return values, store these values or   */
/* otherwise use them as required by your application.                   */
/*                                                                       */
/*************************************************************************/

/// @include file    : Include Files
#include "main.h"
#include "HDMapping.h"
#include "constructWorldMap_types.h"
#include "rt_nonfinite.h"
#include "coder_array.h"

#include <iostream>
#include <fstream>
#include "opencv2/opencv.hpp"

/// Function Declarations
static void argInit_1x2_real_T(double result[2]);

static void argInit_1x3_real_T(double result[3]);

static void argInit_480x640_uint8_T(unsigned char result[307200]);

static coder::array<double, 2U> argInit_Unboundedx3_real_T();

static coder::array<unsigned char, 2U> argInit_UnboundedxUnbounded_uint8_T();

static bool argInit_boolean_T();

static double argInit_real_T();

static void argInit_struct0_T(buildMapping::struct0_T *result);

static void argInit_struct1_T(buildMapping::struct1_T *result);

static void argInit_struct2_T(buildMapping::struct2_T *result);

static buildMapping::struct3_T argInit_struct3_T();

static unsigned char argInit_uint8_T();

// 对应OpenCV的cv::Mat转MATLAB uint8类型或logical图像
static void convertCVToMatrix(cv::Mat &srcImg, int rows, int cols, int channels, unsigned char dst[]) {
    CV_Assert(srcImg.type() == CV_8UC1 || srcImg.type() == CV_8UC3);
    size_t elems = rows * cols;
    if (channels == 3) {
        cv::Mat channels[3];
        cv::split(srcImg.t(), channels);

        memcpy(dst, channels[2].data, elems * sizeof(unsigned char));              //copy channel[2] to the red channel
        memcpy(dst + elems, channels[1].data, elems * sizeof(unsigned char));      // green
        memcpy(dst + 2 * elems, channels[0].data, elems * sizeof(unsigned char));  // blue
    } else {
        srcImg = srcImg.t();
        memcpy(dst, srcImg.data, elems * sizeof(unsigned char));
    }
}

// 对应MATLAB uint8类型或者logical图像转cv::Mat，图像在内存中连续
static void convertToMatContinues(const unsigned char inImg[], int rows, int cols, int channels, cv::Mat &matBigImg) {
    size_t elems = (size_t)rows * cols;
    // unsigned char *array = &inImg[0];
    unsigned char *array = (unsigned char *)inImg;
    if (channels == 3) {
        cv::Mat matR = cv::Mat(cols, rows, CV_8UC1, array);  //inImg在内存中必须连续
        cv::Mat matG = cv::Mat(cols, rows, CV_8UC1, array + elems);
        cv::Mat matB = cv::Mat(cols, rows, CV_8UC1, array + 2 * elems);
        std::vector<cv::Mat> matBGR = {matB.t(), matG.t(), matR.t()};
        cv::merge(matBGR, matBigImg);
    } else {
        matBigImg = cv::Mat(cols, rows, CV_8UC1, inImg[0]);
        matBigImg = matBigImg.t();
    }
}

// 对应MATLAB uint8类型或者logical图像转cv::Mat，图像在内存中不连续
static void convertToMat(const unsigned char inImg[], int rows, int cols, int channels, cv::Mat &matBigImg) {
    size_t elems = (size_t)rows * cols;
    if (channels == 3) {
        matBigImg = cv::Mat(rows, cols, CV_8UC3, cv::Scalar::all(0));

        for (size_t i = 0; i < rows; i++) {
            cv::Vec3b *data = matBigImg.ptr<cv::Vec3b>(i);
            for (size_t j = 0; j < cols; j++) {
                data[j][2] = (uchar)inImg[i + rows * j];
                data[j][1] = (uchar)inImg[i + rows * j + elems];
                data[j][0] = (uchar)inImg[i + rows * j + 2 * elems];
            }
        }
    } else {
        matBigImg = cv::Mat(rows, cols, CV_8UC1, cv::Scalar(0));

        for (size_t i = 0; i < rows; i++) {
            uchar *data = matBigImg.ptr<uchar>(i);
            for (size_t j = 0; j < cols; j++) {
                data[j] = (uchar)inImg[i + rows * j];
            }
        }
    }
}

/// Function Definitions
///
/// @fn             : argInit_1x2_real_T
/// @brief          :
/// @param          : double result[2]
/// @return         : void
///
static void argInit_1x2_real_T(double result[2]) {
    // Loop over the array to initialize each element.
    for (int idx1{0}; idx1 < 2; idx1++) {
        // Set the value of the array element.
        // Change this value to the value that the application requires.
        result[idx1] = argInit_real_T();
    }
}

///
/// @fn             : argInit_1x3_real_T
/// @brief          :
/// @param          : double result[3]
/// @return         : void
///
static void argInit_1x3_real_T(double result[3]) {
    // Loop over the array to initialize each element.
    for (int idx1{0}; idx1 < 3; idx1++) {
        // Set the value of the array element.
        // Change this value to the value that the application requires.
        result[idx1] = argInit_real_T();
    }
}

///
/// @fn             : argInit_480x640_uint8_T
/// @brief          :
/// @param          : unsigned char result[307200]
/// @return         : void
///
static void argInit_480x640_uint8_T(unsigned char result[307200]) {
    // Loop over the array to initialize each element.
    for (int i{0}; i < 307200; i++) {
        // Set the value of the array element.
        // Change this value to the value that the application requires.
        result[i] = argInit_uint8_T();
    }
}

///
/// @fn             : argInit_Unboundedx3_real_T
/// @brief          :
/// @param          : void
/// @return         : coder::array<double, 2U>
///
static coder::array<double, 2U> argInit_Unboundedx3_real_T() {
    coder::array<double, 2U> result;
    // Set the size of the array.
    // Change this size to the value that the application requires.
    result.set_size(2, 3);
    // Loop over the array to initialize each element.
    for (int idx0{0}; idx0 < result.size(0); idx0++) {
        for (int idx1{0}; idx1 < 3; idx1++) {
            // Set the value of the array element.
            // Change this value to the value that the application requires.
            result[idx0 + result.size(0) * idx1] = argInit_real_T();
        }
    }
    return result;
}

///
/// @fn             : argInit_UnboundedxUnbounded_uint8_T
/// @brief          :
/// @param          : void
/// @return         : coder::array<unsigned char, 2U>
///
static coder::array<unsigned char, 2U> argInit_UnboundedxUnbounded_uint8_T() {
    coder::array<unsigned char, 2U> result;
    // Set the size of the array.
    // Change this size to the value that the application requires.
    result.set_size(2, 2);
    // Loop over the array to initialize each element.
    for (int idx0{0}; idx0 < result.size(0); idx0++) {
        for (int idx1{0}; idx1 < result.size(1); idx1++) {
            // Set the value of the array element.
            // Change this value to the value that the application requires.
            result[idx0 + result.size(0) * idx1] = argInit_uint8_T();
        }
    }
    return result;
}

///
/// @fn             : argInit_boolean_T
/// @brief          :
/// @param          : void
/// @return         : bool
///
static bool argInit_boolean_T() {
    return false;
}

///
/// @fn             : argInit_real_T
/// @brief          :
/// @param          : void
/// @return         : double
///
static double argInit_real_T() {
    return 0.0;
}

///
/// @fn             : argInit_struct0_T
/// @brief          :
/// @param          : buildMapping::struct0_T *result
/// @return         : void
///
static void argInit_struct0_T(buildMapping::struct0_T *result) {
    // Set the value of each structure field.
    // Change this value to the value that the application requires.
    argInit_480x640_uint8_T(result->undistortImage);
    argInit_1x3_real_T(result->currFrontBasePose);
    result->isuseGT = argInit_boolean_T();
}

///
/// @fn             : argInit_struct1_T
/// @brief          :
/// @param          : buildMapping::struct1_T *result
/// @return         : void
///
static void argInit_struct1_T(buildMapping::struct1_T *result) {
    // Set the value of each structure field.
    // Change this value to the value that the application requires.
    argInit_struct2_T(&result->HDmap);
    result->vehiclePoses = argInit_Unboundedx3_real_T();
    result->isOver = argInit_boolean_T();
}

///
/// @fn             : argInit_struct2_T
/// @brief          :
/// @param          : buildMapping::struct2_T *result
/// @return         : void
///
static void argInit_struct2_T(buildMapping::struct2_T *result) {
    // Set the value of each structure field.
    // Change this value to the value that the application requires.
    result->bigImg = argInit_UnboundedxUnbounded_uint8_T();
    result->ref = argInit_struct3_T();
}

///
/// @fn             : argInit_struct3_T
/// @brief          :
/// @param          : void
/// @return         : buildMapping::struct3_T
///
static buildMapping::struct3_T argInit_struct3_T() {
    buildMapping::struct3_T result;
    // Set the value of each structure field.
    // Change this value to the value that the application requires.
    argInit_1x2_real_T(result.XWorldLimits);
    result.YWorldLimits[0] = result.XWorldLimits[0];
    result.ImageSize[0] = result.XWorldLimits[0];
    result.YWorldLimits[1] = result.XWorldLimits[1];
    result.ImageSize[1] = result.XWorldLimits[1];
    return result;
}

///
/// @fn             : argInit_uint8_T
/// @brief          :
/// @param          : void
/// @return         : unsigned char
///
static unsigned char argInit_uint8_T() {
    return 0U;
}

///
/// @fn             : main
/// @brief          :
/// @param          : int argc
///                   char **argv
/// @return         : int
///
int main(int, char **) {
    buildMapping::HDMapping *classInstance;
    classInstance = new buildMapping::HDMapping;
    // Invoke the entry-point functions.
    // You can call entry-point functions multiple times.
    main_constructWorldMap(classInstance);
    delete classInstance;
    return 0;
}

///
/// @fn             : main_constructWorldMap
/// @brief          :
/// @param          : buildMapping::HDMapping *instancePtr
/// @return         : void
///
void main_constructWorldMap(buildMapping::HDMapping *instancePtr) {
    static buildMapping::struct0_T inputArgs;
    buildMapping::struct1_T inputOutputStruct;
    inputOutputStruct.isOver = false;
    // Initialize function 'constructWorldMap' input arguments.
    // Initialize function input argument 'inputArgs'.
    // Initialize function input argument 'inputOutputStruct'.

    std::string imagePathList = "./data/preSavedData/imagePathList.txt";
    std::ifstream inFile(imagePathList, std::ios::in);
    if (!inFile) {
        std::cerr << " 打开文件失败！ " << std::endl;
    }
    std::vector<std::vector<double> > all_data;
    std::string imagePath;
    int num = 1;
    while (std::getline(inFile, imagePath)) {
        cv::Mat srcImg = cv::imread(imagePath, cv::IMREAD_GRAYSCALE);
        assert(srcImg.rows == 480 && srcImg.cols == 640 && srcImg.channels() == 1);

        convertCVToMatrix(srcImg, srcImg.rows, srcImg.cols, srcImg.channels(), inputArgs.undistortImage);

        // 建图
        double t1 = (double)cv::getTickCount();
        instancePtr->b_constructWorldMap(&inputArgs, &inputOutputStruct);
        double t = ((double)cv::getTickCount() - t1) / cv::getTickFrequency();

        cv::Mat HDMapImg;
        convertToMat(inputOutputStruct.HDmap.bigImg.data(), inputOutputStruct.HDmap.bigImg.size(0), inputOutputStruct.HDmap.bigImg.size(1), 1, HDMapImg);
        std::cout << "num:" << num << ",it take time:" << t << std::endl;

        cv::imwrite("./results/" + std::to_string(num++) + ".jpg", HDMapImg);
        //cv::imshow("HDmap", HDMapImg);

        //int key = cv::waitKey(1);
        if ((inputOutputStruct.isOver) /*|| (key == 27)*/) {
            cv::imwrite("HDMapImg.png", HDMapImg);
            break;
        }
    }
}

///
/// File trailer for main.cpp
///
/// [EOF]
///
