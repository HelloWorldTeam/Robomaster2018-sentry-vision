# RobomasterMarkerDetector2

#### 项目概述
+ 使用OpenCV的视觉算法，对RoboMaster2018的战车装甲进行自动识别和打击。
+ 算法由检测、跟踪、深度解算三部分组成。

#### 依赖

+ OpenCV 2.4.13
+ cmake

#### 构建

1. mkdir build
2. cd build
3. cmake -DCMAKE_BUILD_TYPE=Release ..
4. make -j