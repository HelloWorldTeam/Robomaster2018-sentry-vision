# Robomaster2018-sentry-vision
Zhejiang University  
**Hello World Team**


## Environment
- Ubuntu16.04
- OpenCV 2.4.13
- cmake

## Hardware
- TX2 + RTSO9003
- USB2.0 Camera
- MCU(STM32F427)

## How to Run
`sudo sh ./script/run.sh`

## Structure
**./**  
CMakeLists.txt	`#编译文件`  
main.cpp	`#Main函数，启动两个线程，图像处理与下位机通信`  
Scripts	`#各种脚本，build工程，load can，运行等`  
src/can.cpp	`#包含下位机通信相关函数`  
src/RemoteControl.cpp	`#下位机数据解析，同约定通信协议相关`  
src/RoboMasterProcess.cpp	`#图像采集，不同模式切换`  
RoboMasterMarkerDetector3	`#装甲识别模块`  
MVCamera	`#工业相机图像采集模块（如果用usb相机，则直接用opencv接口调用即可）`  

**./RoboMasterMarkerDetector3/**  `#项目基层文件夹，装甲检测的基础模块，被步兵、英雄、哨兵共用，具有通用性`  
Config	`#相关参数配置文件，程序初始化时从中读取`  
Calibration	`#存放相机标定文件`  
Cascades	`#存放cascade分类器模型`  
thirdpart/KCFTracker	`#开源的KCF跟踪器`  
src/MarkerParams.h	`#参数管理类`  
src/MarkerSensor.h	`#包含MarkerSensor类，装甲识别的主要算法`  
include/Timer.h	`#计时方法，调试使用`  
