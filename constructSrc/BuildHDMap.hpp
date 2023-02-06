#pragma once
#include "common.hpp"
#include "constructWorldMap.h"
#include "constructWorldMap_emxAPI.h"
#include "constructWorldMap_types.h"
#include "rt_nonfinite.h"
#include <string.h>
#include <iostream>
#include <fstream>

class BuildHDMap {
   private:
    struct1_T m_birdsEye360CaliParameter;  // 对应constructWorldMap主函数输入birdsEye360
    struct6_T m_outputStruct;              // 对应constructWorldMap主函数输出outputStruct

   public:
    BuildHDMap();
    struct6_T Run(struct0_T &inputArgsPose);
    ~BuildHDMap();
};
