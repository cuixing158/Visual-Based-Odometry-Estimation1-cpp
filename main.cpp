#include "constructSrc/common.hpp"
#include "constructSrc/BuildHDMap.hpp"
#include "opencv2/opencv.hpp"
#include "constructSrc/constructWorldMap.h"
#include "constructSrc/constructWorldMap_emxAPI.h"
#include "constructSrc/constructWorldMap_types.h"
#include "constructSrc/rt_nonfinite.h"
#include <string.h>

#include <iostream>
#include <fstream>


int main(int argc, char *argv[]) {
    std::string sensorFile = "E:/AllDataAndModels/surround_fisheye_sim_images/underParkingLotImages20220728/sensorData.csv";
    std::string mapXFile = "D:/vs_files/buildMap/Project2/mapX.csv";
    std::string mapYFile = "D:/vs_files/buildMap/Project2/mapY.csv";
    const size_t numCamera = 4, numFrames = 1186;
    std::vector<std::string> imageFileNames[numCamera];

    // 读取图像路径数据
    std::string imagesPath[4] = { "../front.csv" ,"../left.csv" ,"../back.csv" ,"../right.csv" };
    for (size_t i = 0; i < numCamera; i++)
    {
        std::ifstream inFile(imagesPath[i], std::ios::in);
        if (!inFile) {
            std::cerr << " 打开文件失败！ " << std::endl;
        }
        std::string lineStr;
        while (std::getline(inFile, lineStr)) {
            imageFileNames[i].push_back(lineStr);
        }
    }
 
    cv::Mat sensorData = readCSV(sensorFile);
    sensorData = sensorData.rowRange(1, sensorData.rows);
    cv::Mat mapX = readCSV(mapXFile);
    cv::Mat mapY = readCSV(mapYFile);
    mapX.convertTo(mapX, CV_32FC1);
    mapY.convertTo(mapY, CV_32FC1);
    CV_Assert(mapX.rows == 1063 && mapX.cols == 5796);

    cv::Mat fishEyeImgSurround[numCamera];  // 注意顺序为front,left,back,right
    cv::Mat undistortImgs[numCamera];

    // input data
    static BuildHDMap buildObj;
    static struct0_T inputArgsPose;  // 对应constructWorldMap主函数输入inputArgs
    static struct6_T outHDMap;       //t对应constructWorldMap主函数输出outputStruct

    // 主循建图图图
    for (size_t i = 155; i < numFrames; i++) {
        inputArgsPose.currFrontBasePose[0] = sensorData.at<double>(i, 2);
        inputArgsPose.currFrontBasePose[1] = sensorData.at<double>(i, 3);
        inputArgsPose.currFrontBasePose[2] = sensorData.at<double>(i, 4);
        inputArgsPose.isuseGT = 0;
        for (size_t j = 0; j < numCamera; j++) {
            fishEyeImgSurround[j] = cv::imread(imageFileNames[j][i]);
            cv::remap(fishEyeImgSurround[j], undistortImgs[j], mapX, mapY, cv::INTER_LINEAR);
            undistortImgs[j].convertTo(undistortImgs[j], CV_64F, 1.0 / 255);
            convertCVToMatrix(undistortImgs[j], inputArgsPose.undistortImages[j].f1);
        }

        outHDMap = buildObj.Run(inputArgsPose);

        emxArray_real_T *bigImg = outHDMap.HDmap.bigImg;
        cv::Mat matBigImg,matTraj;
        convertEmxToMat(bigImg, matBigImg);
        
        emxArray_real_T *vehicleTraj = outHDMap.vehicleTraj;
        convertEmxToMat(vehicleTraj, matTraj);

        // save results
        cv::imwrite(std::to_string(i) + ".jpg", matBigImg);
    }
    return 0;
}