#include "common.hpp"
#include "BuildHDMap.hpp"
#include "c_cpp_utils/spdlog/spdlog.h"
#include "opencv2/opencv.hpp"
#include "c_cpp_utils/path.h"
#include "constructWorldMap.h"
#include "constructWorldMap_emxAPI.h"
#include "constructWorldMap_types.h"
#include "rt_nonfinite.h"
#include <string.h>

#include <gperftools/profiler.h>
#include <iostream>
#include <fstream>

bool mycomp(std::string path1, std::string path2) {
    std::string fileName1 = filesystem::path(path1).filenameNoExt();
    std::string fileName2 = filesystem::path(path2).filenameNoExt();
    return (std::stoi(fileName1) < std::stoi(fileName2));
}

int main(int argc, char *argv[]) {
    std::cout << CV_VERSION << std::endl;
    std::string imgSrcDir = "/opt_disk2/rd22946/my_data/binAndimgFromSimOutMat";  //"~/my_data/binAndimgFromSimOutMat";
    std::string orderImages[4] = {"imgFrontSurround", "imgLeftSurround", "imgRearSurround", "imgRightSurround"};
    std::string sensorFile = "/opt_disk2/rd22946/my_data/binAndimgFromSimOutMat/sensorData.csv";
    std::string mapXFile = "/opt_disk2/rd22946/my_data/binAndimgFromSimOutMat/mapX.csv";
    std::string mapYFile = "/opt_disk2/rd22946/my_data/binAndimgFromSimOutMat/mapY.csv";
    size_t numCamera = 4, numFrames = 1186;

    // 读取数据
    filesystem::path imgSrcPath = filesystem::path(imgSrcDir);
    std::vector<std::string> imageFileNames[numCamera];
    for (size_t i = 0; i < numCamera; i++) {
        filesystem::path currPath = imgSrcPath / filesystem::path(orderImages[i]);
        size_t numImgs = filesystem::getFullNames(currPath, imageFileNames[i], ".png");
        std::sort(imageFileNames[i].begin(), imageFileNames[i].end(), mycomp);
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
    for (size_t i = 0; i < numFrames; i++) {
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
        cv::Mat matBigImg;
        convertEmxToMat(bigImg, matBigImg);
        emxArray_real_T *vehicleTraj = outHDMap.vehicleTraj;

        // show results
        spdlog::info("Current frame number:{},bigImg size H*W*C: {}×{}×{}", i, bigImg->size[0], bigImg->size[1], bigImg->size[2]);
        filesystem::path savePath = filesystem::path("results");
        if (!savePath.exists()) {
            filesystem::create_directory(savePath);
        }
        cv::imwrite(savePath.make_absolute().str() + "/" + std::to_string(i) + ".jpg", matBigImg);

        // cv::imshow("", matBigImg);
        // int key = cv::waitKey(1);
        // if (key == 27) {
        //     break;
        // }
    }
    return 0;
}