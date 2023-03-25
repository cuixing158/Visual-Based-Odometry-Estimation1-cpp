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

#include "HDMapping.h"
#include "opencvAPI.h"
#include "constructWorldMap_types.h"
#include "rt_nonfinite.h"
#include "coder_array.h"

#include <iostream>
#include <fstream>
#include "opencv2/opencv.hpp"

int main(int, char **) {
    buildMapping::HDMapping *classInstance;
    classInstance = new buildMapping::HDMapping;

    // 定义输入输出参数
    buildMapping::struct0_T inputArgs;
    buildMapping::struct1_T inputOutputStruct;
    inputOutputStruct.isOver = false;
    std::string imagePathList = "./data/preSavedData/imagePathList.txt";
    std::ifstream inFile(imagePathList, std::ios::in);
    if (!inFile) {
        std::cerr << " 打开文件失败！ " << std::endl;
    }

    // 主循环
    std::string imagePath;
    int num = 1;
    while (std::getline(inFile, imagePath)) {
        cv::Mat srcImg = cv::imread(imagePath, cv::IMREAD_GRAYSCALE);
        assert(srcImg.rows == 480 && srcImg.cols == 640 && srcImg.channels() == 1);

        convertCVToMatrix(srcImg, srcImg.rows, srcImg.cols, srcImg.channels(), inputArgs.undistortImage);

        // 建图
        double t1 = (double)cv::getTickCount();
        classInstance->constructWorldMap(&inputArgs, &inputOutputStruct);
        double t = ((double)cv::getTickCount() - t1) / cv::getTickFrequency();

        cv::Mat HDMapImg;
        convertToMat(inputOutputStruct.HDmap.bigImg.data(), inputOutputStruct.HDmap.bigImg.size(0), inputOutputStruct.HDmap.bigImg.size(1), 1, HDMapImg);
        std::cout << "num:" << num << ",elapsed seconds:" << t << std::endl;

        cv::imwrite("./results/" + std::to_string(num++) + ".jpg", HDMapImg);
        //cv::imshow("HDmap", HDMapImg);

        //int key = cv::waitKey(1);
        if ((inputOutputStruct.isOver) /*|| (key == 27)*/) {
            cv::imwrite("HDMapImg.png", HDMapImg);
            break;
        }
    }

    delete classInstance;
    return 0;
}
