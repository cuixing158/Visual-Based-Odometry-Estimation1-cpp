/**
* @file        :orbSlam.h
* @brief       :C语言接口
* @details     :This is the detail description.
* @date        :2023/05/06 16:18:55
* @author      :cuixingxing(cuixingxing150@gmail.com)
* @version     :1.0
*
* @copyright Copyright (c) 2023
*
*/

#ifndef _ORB_SLAM_
#define _ORB_SLAM_

#include "HDMapping.h"
#include "opencvAPI.h"
#include "constructWorldMap_types.h"
#include "rt_nonfinite.h"
#include "coder_array.h"

#include <iostream>
#include <fstream>
#include "opencv2/opencv.hpp"

struct HDMappingWrapper;

#ifdef __cplusplus
extern "C" {
#endif

extern struct HDMappingWrapper* InitInstance(const char* allDataFolder);

extern int buildMap(HDMappingWrapper* pOrbSlamHandle, cv::Mat& bevImage);

extern void UnInitHDMapping(struct HDMappingWrapper* pOrbSlam3Handle);

#ifdef __cplusplus
};
#endif

#endif
