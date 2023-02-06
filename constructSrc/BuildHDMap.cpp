/**
* @brief       This is a brief description
* @details     This is the detail description.
* @param[in]   inArgName input argument description.
* @param[out]  outArgName output argument description.
* @return      返回值
* @retval      返回值类型
* @par 标识符
*     保留
* @par 其它
*
* @par 修改日志
*      cuixingxing于2022/11/02创建
*/
#include "BuildHDMap.hpp"
BuildHDMap::BuildHDMap() {
    //Initialize the application.You do not need to do this more than one time.
    constructWorldMap_initialize();
    emxInit_struct6_T(&m_outputStruct);

    // front camera
    m_birdsEye360CaliParameter.birdsEye[0].OutputView[0] = -1;
    m_birdsEye360CaliParameter.birdsEye[0].OutputView[1] = 10;
    m_birdsEye360CaliParameter.birdsEye[0].OutputView[2] = -10;
    m_birdsEye360CaliParameter.birdsEye[0].OutputView[3] = 10;
    m_birdsEye360CaliParameter.birdsEye[0].ImageSize[0] = 352;
    m_birdsEye360CaliParameter.birdsEye[0].ImageSize[1] = 640;

    m_birdsEye360CaliParameter.birdsEye[0].Sensor.Height = 0.9856;
    m_birdsEye360CaliParameter.birdsEye[0].Sensor.Pitch = 19.9030;
    m_birdsEye360CaliParameter.birdsEye[0].Sensor.Yaw = -0.0038;
    m_birdsEye360CaliParameter.birdsEye[0].Sensor.Roll = 0.0;
    m_birdsEye360CaliParameter.birdsEye[0].Sensor.SensorLocation[0] = 0.0;
    m_birdsEye360CaliParameter.birdsEye[0].Sensor.SensorLocation[1] = 0.0;
    std::strcpy(m_birdsEye360CaliParameter.birdsEye[0].Sensor.WorldUnits, "meters");

    // left camera
    m_birdsEye360CaliParameter.birdsEye[1].OutputView[0] = -10;
    m_birdsEye360CaliParameter.birdsEye[1].OutputView[1] = 10;
    m_birdsEye360CaliParameter.birdsEye[1].OutputView[2] = -1;
    m_birdsEye360CaliParameter.birdsEye[1].OutputView[3] = 10;
    m_birdsEye360CaliParameter.birdsEye[1].ImageSize[0] = 1163;
    m_birdsEye360CaliParameter.birdsEye[1].ImageSize[1] = 640;

    m_birdsEye360CaliParameter.birdsEye[1].Sensor.Height = 1.24;
    m_birdsEye360CaliParameter.birdsEye[1].Sensor.Pitch = 44.6173;
    m_birdsEye360CaliParameter.birdsEye[1].Sensor.Yaw = 89.8207;
    m_birdsEye360CaliParameter.birdsEye[1].Sensor.Roll = 0.1250;
    m_birdsEye360CaliParameter.birdsEye[1].Sensor.SensorLocation[0] = 0.0;
    m_birdsEye360CaliParameter.birdsEye[1].Sensor.SensorLocation[1] = 0.0;
    std::strcpy(m_birdsEye360CaliParameter.birdsEye[1].Sensor.WorldUnits, "meters");

    // back camera
    m_birdsEye360CaliParameter.birdsEye[2].OutputView[0] = -10;
    m_birdsEye360CaliParameter.birdsEye[2].OutputView[1] = 1;
    m_birdsEye360CaliParameter.birdsEye[2].OutputView[2] = -10;
    m_birdsEye360CaliParameter.birdsEye[2].OutputView[3] = 10;
    m_birdsEye360CaliParameter.birdsEye[2].ImageSize[0] = 352;
    m_birdsEye360CaliParameter.birdsEye[2].ImageSize[1] = 640;

    m_birdsEye360CaliParameter.birdsEye[2].Sensor.Height = 1.0154;
    m_birdsEye360CaliParameter.birdsEye[2].Sensor.Pitch = 19.8682;
    m_birdsEye360CaliParameter.birdsEye[2].Sensor.Yaw = -179.9023;
    m_birdsEye360CaliParameter.birdsEye[2].Sensor.Roll = -0.0625;
    m_birdsEye360CaliParameter.birdsEye[2].Sensor.SensorLocation[0] = 0.0;
    m_birdsEye360CaliParameter.birdsEye[2].Sensor.SensorLocation[1] = 0.0;
    std::strcpy(m_birdsEye360CaliParameter.birdsEye[2].Sensor.WorldUnits, "meters");

    // right camera
    m_birdsEye360CaliParameter.birdsEye[3].OutputView[0] = -10;
    m_birdsEye360CaliParameter.birdsEye[3].OutputView[1] = 10;
    m_birdsEye360CaliParameter.birdsEye[3].OutputView[2] = -10;
    m_birdsEye360CaliParameter.birdsEye[3].OutputView[3] = 1;
    m_birdsEye360CaliParameter.birdsEye[3].ImageSize[0] = 1163;
    m_birdsEye360CaliParameter.birdsEye[3].ImageSize[1] = 640;

    m_birdsEye360CaliParameter.birdsEye[3].Sensor.Height = 1.2485;
    m_birdsEye360CaliParameter.birdsEye[3].Sensor.Pitch = 45.2806;
    m_birdsEye360CaliParameter.birdsEye[3].Sensor.Yaw = -90.1980;
    m_birdsEye360CaliParameter.birdsEye[3].Sensor.Roll = 0.1250;
    m_birdsEye360CaliParameter.birdsEye[3].Sensor.SensorLocation[0] = 0.0;
    m_birdsEye360CaliParameter.birdsEye[3].Sensor.SensorLocation[1] = 0.0;
    std::strcpy(m_birdsEye360CaliParameter.birdsEye[3].Sensor.WorldUnits, "meters");

    // 内参都一样
    for (size_t i = 0; i < 4; i++) {
        m_birdsEye360CaliParameter.birdsEye[i].Sensor.Intrinsics.FocalLength[0] = 317.8183;
        m_birdsEye360CaliParameter.birdsEye[i].Sensor.Intrinsics.FocalLength[1] = 317.8183;
        m_birdsEye360CaliParameter.birdsEye[i].Sensor.Intrinsics.PrincipalPoint[0] = 2866.778010428930429;
        m_birdsEye360CaliParameter.birdsEye[i].Sensor.Intrinsics.PrincipalPoint[1] = 529.816904864335;
        m_birdsEye360CaliParameter.birdsEye[i].Sensor.Intrinsics.ImageSize[0] = 1063;
        m_birdsEye360CaliParameter.birdsEye[i].Sensor.Intrinsics.ImageSize[1] = 5796;
        m_birdsEye360CaliParameter.birdsEye[i].Sensor.Intrinsics.RadialDistortion[0] = 0,
        m_birdsEye360CaliParameter.birdsEye[i].Sensor.Intrinsics.RadialDistortion[1] = 0;
        m_birdsEye360CaliParameter.birdsEye[i].Sensor.Intrinsics.TangentialDistortion[0] = 0;
        m_birdsEye360CaliParameter.birdsEye[i].Sensor.Intrinsics.TangentialDistortion[1] = 0;
        m_birdsEye360CaliParameter.birdsEye[i].Sensor.Intrinsics.Skew = 0;
    }

    // 四副环视图到基准第一个到转换
    auto init0 = std::initializer_list<double>({1, 0, 0, 0, 1, 0, 0, 0, 1});
    std::copy(init0.begin(), init0.end(), m_birdsEye360CaliParameter.tforms[0].A);
    auto init1 = std::initializer_list<double>({0.539859, -0.0114774, 0, 0.00148547, 0.54053, 0, -25.6848, 58.0006, 1});  // join(string((birdsEye360.tforms{3}.A(:))),',')
    std::copy(init1.begin(), init1.end(), m_birdsEye360CaliParameter.tforms[1].A);
    auto init2 = std::initializer_list<double>({0.9945768, -0.03405895, 0, -0.003624775, 0.9873774, 0, 0.5879876, 430.9479, 1});
    std::copy(init2.begin(), init2.end(), m_birdsEye360CaliParameter.tforms[2].A);
    auto init3 = std::initializer_list<double>({0.5489015, -0.01142391, 0, 0.003611683, 0.5398036, 0, 317.7876, 53.31789, 1});
    std::copy(init3.begin(), init3.end(), m_birdsEye360CaliParameter.tforms[3].A);
}

struct6_T BuildHDMap::Run(struct0_T &inputArgsPose) {
    struct6_T outputStruct = m_outputStruct;
    constructWorldMap(&inputArgsPose, &m_birdsEye360CaliParameter, &outputStruct);
    return outputStruct;
}

BuildHDMap::~BuildHDMap() {
    // 释放
    emxDestroy_struct6_T(m_outputStruct);
    constructWorldMap_terminate();
}