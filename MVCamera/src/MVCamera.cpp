//
// Created by liming on 5/31/18.
//

#include <CameraDefine.h>
#include <zconf.h>
#include "MVCamera.h"
#include "Timer.h"

int                     MVCamera::iCameraCounts = 4;
int                     MVCamera::iStatus = 0;
int                     MVCamera::hCamera = 0;
int                     MVCamera::channel = 3;

//tSdkCameraDevInfo       MVCamera::tCameraEnumList[4];
tSdkCameraCapbility     MVCamera::tCapability;      //设备描述信息
tSdkFrameHead           MVCamera::sFrameInfo;
unsigned char           *	MVCamera::pbyBuffer = NULL;
unsigned char           * MVCamera::g_pRgbBuffer[2] = {NULL, NULL};     //处理后数据缓存区
int                     MVCamera::ready_buffer_id = 0;
bool                    MVCamera::stopped = false;
bool                    MVCamera::updated = false;
mutex                   MVCamera::mutex1;

MVCamera::MVCamera()
{

}

int MVCamera::Init(int id)
{
  printf("CAMERA SDK INIT...\n");
  CameraSdkInit(1);
  printf("DONE\n");

  printf("ENUM CAMERA DEVICES...\n");
	//枚举设备，并建立设备列表
  tSdkCameraDevInfo tCameraEnumList[4];
	CameraEnumerateDevice(tCameraEnumList,&iCameraCounts);
	//没有连接设备
	if(iCameraCounts==0){
	  printf("ERROR: NO CAMERA CONNECTED.\n");
		return -1;
	} else if (iCameraCounts <= id) {
    printf("CONNECTED CAMERA NUMBERS: %d TOO SMALL, ID: %d.\n", iCameraCounts, id);
    return -1;
	} else {
    printf("CONNECTED CAMERA NUMBERS: %d.\n", iCameraCounts);
	}
  printf("DONE\n");

	//相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
	iStatus = CameraInit(&(tCameraEnumList[id]),-1,-1,&hCamera);
	//初始化失败
	if (iStatus!=CAMERA_STATUS_SUCCESS) {
    printf("ERROR: CAMERA INIT FAILED.\n");
		return -1;
	} else {
    printf("CAMERA INIT SUCCESS.\n");
	}

	//设置色温模式
//  iStatus = CameraSetPresetClrTemp(hCamera, 1);
//	if (iStatus == CAMERA_STATUS_SUCCESS) {
//    printf("CAMERA SETPRESETCLRTEMP SUCCESS!\n");
//	} else {
//    printf("CAMERA SETPRESETCLRTEMP FAILED! ERROR CODE: %d\n", iStatus);
//	}

//  iStatus = CameraSetClrTempMode(hCamera, 1);
//	if (iStatus == CAMERA_STATUS_SUCCESS) {
//    printf("CAMERA SETCLRTEMPMODE SUCCESS!\n");
//	} else {
//    printf("CAMERA SETCLRTEMPMODE FAILED! ERROR CODE: %d\n", iStatus);
//	}

	//获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
	CameraGetCapability(hCamera,&tCapability);

	//设置输出为彩色
  channel = 3;
  CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_BGR8);

	//初始化缓冲区
  g_pRgbBuffer[0] = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);
  g_pRgbBuffer[1] = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);
}

int MVCamera::Uninit()
{
  printf("Save Parameter...\n");
  CameraSaveParameter(hCamera, 0);

  printf("Uninit...\n");
  int status = CameraUnInit(hCamera);

  printf("status: %d\n", status);

  if (status == CAMERA_STATUS_SUCCESS) {
    printf("CAMERA UNINIT SUCCESS!\n");
  } else {
    printf("CAMERA UNINIT FAILED! ERROR CODE: %d\n", status);
  }

  if (g_pRgbBuffer[0] != NULL) {
    free(g_pRgbBuffer[0]);
    g_pRgbBuffer[0] = NULL;
  }

  if (g_pRgbBuffer[1] != NULL) {
    free(g_pRgbBuffer[1]);
    g_pRgbBuffer[1] = NULL;
  }
}

int MVCamera::SetExposureTime(bool auto_exp, double exp_time)
{
  if (auto_exp) {
    CameraSdkStatus status = CameraSetAeState(hCamera, true);
    if (status == CAMERA_STATUS_SUCCESS) {
      printf("ENABLE AUTO EXP SUCCESS.\n");
    } else {
      printf("ENABLE AUTO EXP FAILED.\n");
      return status;
    }
  } else {
    CameraSdkStatus status = CameraSetAeState(hCamera, false);
    if (status == CAMERA_STATUS_SUCCESS) {
      printf("DISABLE AUTO EXP SUCCESS.\n");
    } else {
      printf("DISABLE AUTO EXP FAILED.\n");
      return status;
    }
    CameraSdkStatus status1 = CameraSetExposureTime(hCamera, exp_time);
    if (status1 == CAMERA_STATUS_SUCCESS) {
      printf("SET EXP TIME SUCCESS.\n");
    } else {
      printf("SET EXP TIME FAILED.\n");
      return status;
    }
  }

  return 0;
}

double MVCamera::GetExposureTime()
{
  int auto_exp;
  if (CameraGetAeState(hCamera, &auto_exp) == CAMERA_STATUS_SUCCESS) {
    if (auto_exp) {
      return 0;
    } else {
      double exp_time;
      if (CameraGetExposureTime(hCamera, &exp_time) == CAMERA_STATUS_SUCCESS) {
        return exp_time;
      } else {
        printf("GET CAMERA EXP TIME ERROR.\n");
        return -1;
      }
    }
  } else {
    printf("GET CAMERA AE STATE ERROR.\n");
    return -1;
  }
}

int MVCamera::SetLargeResolution(bool if_large_resolution)
{
  tSdkImageResolution resolution;
  if (if_large_resolution) {
    resolution.iIndex = 0;
    if (CameraSetImageResolution(hCamera, &resolution) == CAMERA_STATUS_SUCCESS) {
      printf("CAMERA SET LARGE RESOLUTION SUCCESS.\n");
    } else {
      printf("CAMERA SET LARGE RESOLUTION FAILED.\n");
      return -1;
    }
  } else {
    resolution.iIndex = 1;
    CameraSetImageResolution(hCamera, &resolution);
    if (CameraSetImageResolution(hCamera, &resolution) == CAMERA_STATUS_SUCCESS) {
      printf("CAMERA SET SMALL RESOLUTION SUCCESS.\n");
    } else {
      printf("CAMERA SET SMALL RESOLUTION FAILED.\n");
      return -1;
    }
  }

  return 0;
}

int MVCamera::SetWBMode(bool auto_wb)
{
  int status = CameraSetWbMode(hCamera, auto_wb);
  if (CAMERA_STATUS_SUCCESS == status) {
    printf("CAMERA SETWBMODE %d SUCCESS!\n", auto_wb);
  } else {
    printf("CAMERA SETWBMODE %d FAILED! ERROR CODE: %d\n", auto_wb, status);
  }
}

int MVCamera::GetWBMode(bool &auto_wb)
{
  int res = 0;
  if (CAMERA_STATUS_SUCCESS == CameraGetWbMode(hCamera, &res)) {
    printf("CAMERA GETWBMODE %d SUCCESS!\n", res);
  } else {
    printf("CAMERA GETWBMODE FAILED!\n");
  }
  auto_wb = res;
}

int MVCamera::SetOnceWB()
{
  int status = CameraSetOnceWB(hCamera);
  if (CAMERA_STATUS_SUCCESS == status) {
    printf("CAMERA SETONCEWB SUCCESS!\n");
  } else {
    printf("CAMERA SETONCEWB FAILED, ERROR CODE: %d!\n", status);
  }
}

int MVCamera::SetGain(double gain)
{
  int set_gain = int(gain*100);
  int status = CameraSetGain(hCamera, set_gain, set_gain, set_gain);
  if (CAMERA_STATUS_SUCCESS == status) {
    printf("CAMERA SETGAIN SUCCESS!\n");
  } else {
    printf("CAMERA SETGAIN FAILED! ERROR CODE: %d\n", status);
  }
}

double MVCamera::GetGain()
{
  int r_gain, g_gain, b_gain;
  int status = CameraGetGain(hCamera, &r_gain, &g_gain, &b_gain);
  if (CAMERA_STATUS_SUCCESS == status) {
    printf("CAMERA GETGAIN SUCCESS!\n");
  } else {
    printf("CAMERA GETGAIN FAILED! ERROR CODE: %d\n", status);
  }

  return (r_gain + g_gain + b_gain)/300.;
}

int MVCamera::Play()
{
  CameraPlay(hCamera);
  std::thread thread1(MVCamera::Read);
  thread1.detach();
}

int MVCamera::Read()
{
  while (!stopped) {
    if (CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer, 1000) == CAMERA_STATUS_SUCCESS) {
      CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer[(ready_buffer_id+1)%2], &sFrameInfo);
      ready_buffer_id = (ready_buffer_id + 1)%2;
      updated = true;
      CameraReleaseImageBuffer(hCamera, pbyBuffer);
    }
  }
}

int MVCamera::GetFrame(Mat &frame)
{
  if (frame.cols != sFrameInfo.iWidth || frame.rows != sFrameInfo.iHeight) {
    printf("GetFrame: resize frame !\n");
    frame.create(sFrameInfo.iHeight, sFrameInfo.iWidth, CV_8UC3);
  }
  TIME_BEGIN();
  while (!updated) {
    usleep(1000);
  }
  TIME_END("Wait for camera data");
  memcpy(frame.data, g_pRgbBuffer[ready_buffer_id], frame.cols*frame.rows*3);
  updated = false;
  return 0;
}

int MVCamera::Stop()
{
  stopped = true;
  usleep(30000);
  return 0;
}
