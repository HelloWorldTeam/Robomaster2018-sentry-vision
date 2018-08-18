//
// Created by wx on 18-4-28.  //

#include <sys/time.h>
#include "RoboMasterProcess.h"
//#include "RMVideoCapture.h"
#include "MVCamera.h"
#include "Timer.h"

using namespace std;

void RoboMasterProcess::Process() {

  Setting & settingReceive = *setting;

  /*-----------from local video---------*/
  //VideoCapture cap("/home/liming/Desktop/test.avi");

  //VideoCapture cap("/home/wx/Desktop/najing/finalData/1.avi");
  //VideoCapture cap("/home/wx/Desktop/najing/finalData/0515shiyingchangdi/ex80fire.avi");


  /*-----------from camera device ------*/
//  cv::Size frame_size(1280, 720);
//  char devicename[50] = "/dev/video0";
//  RMVideoCapture cap(devicename, 2);
//  cap.setVideoFormat(frame_size.width, frame_size.height);
//  cap.setVideoFPS(60);
//  cap.setExposureTime(false, 70);
//  bool start_success = cap.startStream();
//  if (!start_success) {
//    CanSendError(fd2car);
//    printf("Start camera error, check camera connection!\n");
//    return;
//  }
//  cap.info();

  /*-----------from mindvision camera device ------*/
  printf("MVCamera Init\n");
  MVCamera::Init();
  printf("MVCamera Play\n");
  MVCamera::Play();
  MVCamera::SetExposureTime(false, 1000);
  MVCamera::SetLargeResolution(true);

  Mat srcImg;
  float yaw = 0,pitch = 0;
  float x = 1, y = 1, z = 1;
  int type = 0;
  bool isSuccess = false;
  bool isToSend = false;
  bool stopped = false;
  int runeReturn;

  //SmallRuneProcess smallRune;
  //BigRuneProcess bigRune;

  MarkerSensor markerSensor(//"../RobomasterMarkerDetector2/calibration/calibration_l_1_16_9_640_360.yml",
                            "/home/nvidia/Desktop/calibration/calibration.yml",
                            //"../RobomasterMarkerDetector2/config/config.yml",
                            "/home/nvidia/Desktop/config/marker_config.yml",
                            "../RoboMasterMarkerDetector3/cascades/cascade_1.xml");

  /**record video**/
  int cnt = 0;
  shared_ptr<cv::VideoWriter> pWriter;
  /****/

  timeval last_time;
  timeval now_time;
  gettimeofday(&last_time, 0);

  while(1){

    /// check stopped
    if (stopped) break;

    //cap >> srcImg;
    
    TIME_BEGIN();
    MVCamera::GetFrame(srcImg);
    TIME_END("MVCamera::Read");

    if(srcImg.empty()){
      printf("Image empty !\n");
      usleep(10000);
      continue;
    }
    printf("mode: %d \n", setting->mode);

    switch(setting->mode){

      case 0:
        /// only show
        //cv::resize(srcImg, s_img, cv::Size(640, 360));
        //cv::imshow("s_img", s_img);
        cv::imshow("srcImg", srcImg);
        break;

        //blue
      case 1:
        TIME_BEGIN();
        markerSensor.SetTargetBlue();
        TIME_END("SetTargetBlue");
        //isSuccess = markerSensor.ProcessFrameBuBing(srcImg, yaw, pitch);
        //isSuccess = markerSensor.ProcessFrameBuBing2(srcImg, x, y);
        TIME_BEGIN();
        isSuccess = (markerSensor.ProcessFrameLEDXYZ(srcImg, x, y, z, type) == MarkerSensor::STATUS_SUCCESS);
        //isSuccess = (markerSensor.ProcessFrameLEDXY(srcImg, x, y) == MarkerSensor::STATUS_SUCCESS);
        TIME_END("ProcessFrameXY");
        if (isSuccess) {
          TIME_BEGIN();
          CanSend(fd2car, x, y, z, type);
          TIME_END("CanSend");
        }
        break;

        //red
      case 2:
        TIME_BEGIN();
        markerSensor.SetTargetRed();
        TIME_END("SetTargetRed");
        //isSuccess = markerSensor.ProcessFrameBuBing(srcImg, yaw, pitch);
        //isSuccess = markerSensor.ProcessFrameBuBing2(srcImg, x, y);
        TIME_BEGIN();
        isSuccess = (markerSensor.ProcessFrameLEDXYZ(srcImg, x, y, z, type) == MarkerSensor::STATUS_SUCCESS);
        //isSuccess = (markerSensor.ProcessFrameLEDXY(srcImg, x, y) == MarkerSensor::STATUS_SUCCESS);
        TIME_END("ProcessFrameXY");
        if (isSuccess) {
          TIME_BEGIN();
          CanSend(fd2car, x, y, z, type);
          TIME_END("CanSend");
        }
        break;

        //small
      case 3:
//        gettimeofday(&start,NULL);
//        cout << "count: " << count << endl;
//        count ++;
//        resize(srcImg, srcImg, Size(640,360), INTER_LINEAR);
//        //recitify
//        //remap(srcImg, srcImg, smallRune.angleSolver.map1, smallRune.angleSolver.map2, INTER_LINEAR);
//        runeReturn = smallRune.getYawAndPitch(srcImg,yaw,pitch);
//        gettimeofday(&end,NULL);
//        time_use = (end.tv_sec-start.tv_sec)*1000+(end.tv_usec-start.tv_usec)/1000.0;
//        cout << "time cost: " << time_use << endl;
//        //test, remember to comment
//        //waitKey(1000);
//        cout << "========================================" << endl;
//        if(runeReturn == 0){
//          isSuccess = true;
//          isToSend = false;
//        }else if(runeReturn == 1){
//          isSuccess = true;
//          isToSend = true;
//        }else{
//          isSuccess = false;
//          isToSend = false;
//        }
//        if(!isSuccess){
//          continue;
//        }
        break;

        //big
      case 4:
//        gettimeofday(&start,NULL);
//        cout << "count: " << count << endl;
//        count ++;
//        resize(srcImg, srcImg, Size(640,360), INTER_LINEAR);
//        runeReturn = bigRune.getYawAndPitch(srcImg,yaw,pitch);
//        gettimeofday(&end,NULL);
//        time_use = (end.tv_sec-start.tv_sec)*1000+(end.tv_usec-start.tv_usec)/1000.0;
//        cout << "time cost: " << time_use << endl;
//        cout << "========================================" << endl;
//        //waitKey(2000);
//        if(runeReturn == 0){
//          isSuccess = true;
//          isToSend = false;
//        }else if(runeReturn == 1){
//          isSuccess = true;
//          isToSend = true;
//        }else{
//          isSuccess = false;
//          isToSend = false;
//        }
//        if(!isSuccess){
//          continue;
//        }
        break;

      default:
        break;
    }

    cv::imshow("srcImg", srcImg);
    char cmd = cv::waitKey(3);   /// to show something
    if (cmd == '1') {
      setting->mode = 1;
    }
    if (cmd == '2') {
      setting->mode = 2;
    }
    if (cmd == '3') {
      setting->mode = 3;
    }
    if (cmd == '4') {
      setting->mode = 4;
    }

    /*-----------save video---------------*/
    //if (cnt == 0) {
    //  if (pWriter != NULL)
    //    pWriter->release();
    //  char videoName[50] = "";
    //  timeval videoTime;
    //  gettimeofday(&videoTime, 0);
    //  long long t_v = videoTime.tv_sec;
    //  sprintf(videoName, "/home/nvidia/Videos/runeRecord_%04d.avi", t_v);
    //  pWriter.reset(new cv::VideoWriter(videoName, CV_FOURCC('M', 'J', 'P', 'G'), 100, cv::Size(752, 480)));
    //}
    ////cv::resize(srcImg, s_img, cv::Size(640, 360));
    //pWriter->write(srcImg);
    //if (++cnt == 1800)
    //  cnt = 0;
    /*------------------------------------*/

    gettimeofday(&now_time, 0);
    double time_duaration = now_time.tv_sec - last_time.tv_sec + (now_time.tv_usec - last_time.tv_usec)*1e-6;
    double fps = 1/time_duaration;
    printf("time_duaration: %lf\n", time_duaration);
    printf("FPS: %lf\n", fps);
    gettimeofday(&last_time, 0);

  }

  MVCamera::Stop();
  MVCamera::Uninit();

}
