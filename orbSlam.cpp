/**
* @file        :orbSlam.cpp
* @brief       :C语言接口
* @details     :This is the detail description.
* @date        :2023/05/06 16:18:55
* @author      :cuixingxing(cuixingxing150@gmail.com)
* @version     :1.0
*
* @copyright Copyright (c) 2023
*
*/
#include "orbSlam.h"
#include "HDMapping.h"
#include "opencvAPI.h"
#include "constructWorldMap_types.h"
#include "rt_nonfinite.h"
#include "coder_array.h"

#include <iostream>
#include <fstream>
#include "opencv2/opencv.hpp"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct HDMappingWrapper {
    buildMapping::HDMapping* HDMapping;
    // 定义输入输出参数
    buildMapping::struct0_T inputArgs;
    buildMapping::struct1_T inputOutputStruct;

} HDMappingWrapper;

static HDMappingWrapper obj;

HDMappingWrapper* InitInstance(const char* allDataFolder) {
    obj.inputOutputStruct.isBuildMap = true;
    obj.inputOutputStruct.buildMapStopFrame = 1118;  // 此处应当对应用户“建图结束”按钮响应
    return &obj;
}

void buildMap(HDMappingWrapper* pOrbSlamHandle, cv::Mat& bevImage) {
    assert(bevImage.rows == 480 && bevImage.cols == 640 && bevImage.channels() == 1);

    convertCVToMatrix(bevImage, bevImage.rows, bevImage.cols, bevImage.channels(), pOrbSlamHandle->inputArgs.undistortImage);
    // 建图
    pOrbSlamHandle->HDMapping->constructWorldMap(&(pOrbSlamHandle->inputArgs), &(pOrbSlamHandle->inputOutputStruct));

    // cv::Mat HDMapImg;
    // convertToMat(inputOutputStruct.HDmap.bigImg.data(), inputOutputStruct.HDmap.bigImg.size(0), inputOutputStruct.HDmap.bigImg.size(1), 1, HDMapImg);
}

void UnInitHDMapping(HDMappingWrapper* pOrbSlamHandle) {
    delete pOrbSlamHandle;
}

#ifdef __cplusplus
};
#endif
