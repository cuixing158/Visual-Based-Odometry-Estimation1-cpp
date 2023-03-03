
# 建图C++程序

2023.2.6 崔星星首次记录

最新修改于2023.3.3

>用于复现此项目分支[matlab_branch](http://autogit.long-horn.com:3000/algorithm_Dev/buildMapping)的C++工程代码,输入输出结果可以保证达到一致的运行结果。支持在Ti/win/linux等平台运行。

相对上一版v0.0.1，本库已成功集成闭环检测，因子图优化等，改用全景拼接图(鸟瞰图)直接进行建图，放弃上个版本的矫正环视图步骤。

## Requirements

- OpenCV 4.x

- C++ 11 support compiler

无特定平台和编译器要求，如果您使用`OpenCV3.x`,需要适当修改源码。

## 进展

- 2023.2.17 已经对DBOW3进行了C++代码生成进行了测试，测试编译接口通过，即将发布新版本。
- 2023.2.28 已完成闭环检测，图优化C++集成,在VS2019、ubuntu环境下均验证成功。

## 建图主接口API

`HDMapping`类为主类，接口较简单，一个默认构造函数`HDMapping()`和一个`b_constructWorldMap()`函数。对应项目中头文件`HDMapping.h`。

```C++

#ifndef HDMAPPING_H
#define HDMAPPING_H

#include "constructWorldMap_types.h"
#include "constructWorldMap_types1.h"
#include "rtwtypes.h"
#include "omp.h"
#include <cstddef>
#include <cstdlib>

namespace buildMapping {
class HDMapping {
public:
  HDMapping();
  ~HDMapping();
  void b_constructWorldMap(const struct0_T *inputArgs,
                           struct1_T *inputOutputStruct);// 仅调用此接口即可
  constructWorldMapStackData *getStackData();

private:
  constructWorldMapPersistentData pd_;
  constructWorldMapStackData SD_;
};

} // namespace buildMapping

namespace buildMapping {
extern omp_nest_lock_t constructWorldMap_nestLockGlobal;
}

#endif
```

使用上面建图类示例伪代码：

```C++
while 循环不断取全景拼接图
    cv::Mat srcImg = cv::imread(imagePath, cv::IMREAD_GRAYSCALE);
    assert(srcImg.rows == 480 && srcImg.cols == 640 && srcImg.channels() == 1);
    convertCVToMatrix(srcImg, srcImg.rows, srcImg.cols, srcImg.channels(), r.undistortImage);

    // 调用建图主函数
    instancePtr->b_constructWorldMap(&r, &inputOutputStruct);
    cv::Mat HDMapImg;
    convertToMat(inputOutputStruct.HDmap.bigImg.data(), inputOutputStruct.HDmap.bigImg.size(0), inputOutputStruct.HDmap.bigImg.size(1), 1, HDMapImg);
    cv::imshow("HDmap", HDMapImg);// 展现实时图
```

## 数据结构说明

由于上述主类中的数据类型是自动C++代码工具生成的，同事较难以理解含义，只需对入口函数类型掌握即可。

**输入数据必须确保正确！！！**

- 输入参数`struct0_T`类型定义如下：

```C++
namespace buildMapping {
struct struct0_T {
  unsigned char undistortImage[307200]; // 480*640=307200
  double currFrontBasePose[3]; // [x,y,theta]，可不提供输入，接口预留后期使用
  bool isuseGT; // 可不提供输入，接口预留后期使用
};
```

该结构体包含3个域成员，其中，`undistortImage`存储为1副全景拼接图(鸟瞰图，确保已经是无畸变)；`currFrontBasePose`存储当前位姿[$x$,$y$,$\theta$]；`isuseGT`是否使用接收第三方里程计。

- 返回参数类型`struct1_T`类型定义如下：

```C++
struct struct1_T {
  struct2_T HDmap;
  ::coder::array<double, 2U> vehiclePoses;
  bool isOver;
};
```

该结构体包含3个域成员，其中，`HDmap`为输出的HD map;`vehiclePoses`为车辆行驶的轨迹坐标;`isOver`标志是否姿态图已优化完毕。

```C++
struct struct2_T {
  ::coder::array<unsigned char, 2U> bigImg;
  struct3_T ref;
};
```

输出参数`HDmap`子类型仍然为结构体类型，也包含2个域成员，其中`bigImg`为对应的像素大图像；`ref`为对应此大图像的参考空间坐标系。

## Notes

- demo中示例数据存放在:"\\yunpan02\豪恩汽电\豪恩汽电研发中心\临时文件夹\CXX\map_R_resize"

- 对于闭环过程中词袋特征文件、后期姿态图优化的全局图像拼接使用的数据文件需要**事先默认保存到当前工程根目录**，即这2个文件分别位于：`./data/preSavedData/database.yml.gz`、`./data/preSavedData/imagePathList.txt`

## 如何配置使用

适合各类平台和编译器支持，以下配置仅供参考。

- linux上:

```bash
git clone http://autogit.long-horn.com:3000/algorithm_Dev/buildMappingCPP.git
cd buildMappingCPP
mkdir build
cd build
cmake .. 
make install
```

- windows上：

先从`http://autogit.long-horn.com:3000/algorithm_Dev/buildMappingCPP.git`下载好对应`main`分支源码压缩包并解压。

修改根目录中的`CmakeLists.txt`中的第12行指出自己`OpenCV`安装路径，比如我windows上安装路径为`set(OpenCV_DIR "D:/opencv_4_4_0/opencv/build")`或者系统环境变量指定此`OpenCV_DIR`定义。

接下来使用Cmake-GUI构建如下图：

<table>
    <tr>
        <td ><center><img src="images/depoly1.jpg" >图1  Cmake configure </center></td>
        <td ><center><img src="images/depoly2.jpg"  >图2 Cmake configure</center></td>
    </tr>
</table>

然后选择好自己的路径和编译器版本，点Configure和Generate就可以生成对应VC的项目工程了。我用的是VS2019,打开生成的sln文件，如下图进行编译并运行。

![deploy1](./images/depoly3.jpg)

VS 工程跑通建图结果见"\\yunpan02\豪恩汽电\豪恩汽电研发中心\临时文件夹\CXX\binAndimgFromSimOutMat\cpp_result"

---

对全景图跑通结果截图如下：

![vs2019_hdmap](images/vs2019_hdmap.png)
