#pragma once
#include "opencv2/opencv.hpp"
#include "constructWorldMap.h"
#include "constructWorldMap_emxAPI.h"
#include "constructWorldMap_types.h"
#include "rt_nonfinite.h"
#include <string.h>
#include <iostream>
#include <fstream>

inline cv::Mat readCSV(std::string csvFile) {
    std::ifstream inFile(csvFile, std::ios::in);
    if (!inFile) {
        std::cout << "打开文件失败！" << std::endl;
        exit(1);
    }
    std::vector<std::vector<double> > all_data;
    std::string lineStr;
    while (std::getline(inFile, lineStr)) {
        // Now inside each line we need to seperate the cols
        std::vector<double> values;
        std::stringstream temp(lineStr);
        std::string single_value;
        while (getline(temp, single_value, ',')) {
            // convert the string element to a integer value
            values.push_back(atof(single_value.c_str()));
        }
        // add the row to the complete data vector
        all_data.push_back(values);
    }

    // Now add all the data into a Mat element
    cv::Mat vect = cv::Mat::zeros((int)all_data.size(), (int)all_data[0].size(), CV_64FC1);
    // Loop over vectors and add the data
    for (int row = 0; row < (int)all_data.size(); row++) {
        for (int cols = 0; cols < (int)all_data[0].size(); cols++) {
            vect.at<double>(row, cols) = all_data[row][cols];
        }
    }
    return vect;
}

/**
* @brief       OpenCV图像转一位数组
* @details     This is the detail description.
* @param[in]   srcImg 1289*720*3大小的BGR图像,uchar类型.
* @param[out]  dst 1289*720*3大小的一维数组.
* @return      返回值
* @retval      返回值类型
* @par 标识符
*     保留
* @par 其它
*
* @par 修改日志
*      cuixingxing于2022/10/27创建
*/
inline void convertCVToMatrix(cv::Mat &srcImg, double dst[18483444]) {
    CV_Assert(srcImg.total() * 3 == 18483444U && srcImg.type() == CV_64FC3);
    cv::Mat channels[3];
    cv::split(srcImg.t(), channels);
    size_t elems = 18483444 / srcImg.channels();
    memcpy(dst, channels[2].data, elems * sizeof(double));              //copy channel[2] to the red channel
    memcpy(dst + elems, channels[1].data, elems * sizeof(double));      // green
    memcpy(dst + 2 * elems, channels[0].data, elems * sizeof(double));  // blue
}

inline void convertEmxToMat(emxArray_real_T *bigImg, cv::Mat &matBigImg) {
    int rows = bigImg->size[0];
    int cols = bigImg->size[1];
    int channels = bigImg->size[2];
    size_t elems = rows * cols;

    cv::Mat matR = cv::Mat(cols, rows, CV_64FC1, bigImg->data);
    cv::Mat matG = cv::Mat(cols, rows, CV_64FC1, bigImg->data + elems);
    cv::Mat matB = cv::Mat(cols, rows, CV_64FC1, bigImg->data + 2 * elems);
    std::vector<cv::Mat> matBGR = {matB.t(), matG.t(), matR.t()};
    cv::merge(matBGR, matBigImg);
    matBigImg.convertTo(matBigImg, CV_8UC3, 255);
}