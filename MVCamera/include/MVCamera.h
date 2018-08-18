//
// Created by liming on 5/31/18.
//

#ifndef MVCAMERA_MVCAMERA_H
#define MVCAMERA_MVCAMERA_H

#include "CameraApi.h"  //相机SDK头文件

#include <thread>
#include <mutex>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class MVCamera {
public:
  static int    Init(int id = 0);
  static int    Play();
  static int    Read();
  static int    GetFrame(Mat& frame);
  static int    Stop();
  static int    Uninit();
  static int    SetExposureTime(bool auto_exp, double exp_time = 10000);
  static double GetExposureTime();
  static int    SetLargeResolution(bool if_large_resolution);
  static Size   GetResolution();
  static int    SetGain(double gain);
  static double GetGain();
  static int    SetWBMode(bool auto_wb = true);
  static int    GetWBMode(bool & auto_wb);
  static int    SetOnceWB();

  static int                     iCameraCounts;
	static int                     iStatus;
	static int                     hCamera;
  static int                     channel;

  //static tSdkCameraDevInfo       tCameraEnumList[4];
	static tSdkCameraCapbility     tCapability;      //设备描述信息
	static tSdkFrameHead           sFrameInfo;
	static unsigned char           *	pbyBuffer;
  static unsigned char           * g_pRgbBuffer[2];     //处理后数据缓存区
  static int                     ready_buffer_id;
  static bool                    stopped;
  static bool                    updated;
  static mutex                   mutex1;

private:
  MVCamera();
};


#endif //MVCAMERA_MVCAMERA_H
