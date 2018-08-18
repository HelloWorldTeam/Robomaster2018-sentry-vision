#include <iostream>

#include <opencv2/opencv.hpp>
#include "MarkerSensor.h"
#include "Timer.h"

using namespace std;

int main(int argc, char** argv) {

  MarkerSensor markerSensor("../calibration/calibration_mvcamera.yml",
  "../config/marker_config.yml", "../cascades/cascade_1.xml");

  cv::VideoCapture capture(argv[1]);

  cv::Mat srcImg;
  float X, Y, Z;
  while (capture.read(srcImg)) {
    TIME_BEGIN();
      //if (MarkerSensor::STATUS_SUCCESS == markerSensor.ProcessFrameXY(srcImg,X,Y,depth)) {
      //  printf("get target X:%f Y:%f depth:%f\n", X, Y, depth);
      //if (MarkerSensor::STATUS_SUCCESS == markerSensor.ProcessFrameLEDXYZ(srcImg,X,Y,Z)) {
      //  printf("get target X:%f Y:%f depth:%f\n", X, Y, Z);
      if (MarkerSensor::STATUS_SUCCESS == markerSensor.ProcessFrameLEDXY(srcImg,X,Y)) {
        printf("get target X:%f Y:%f\n", X, Y);
      } else {
        printf("get no target\n");
      }
    TIME_END("ProcessFrameXY");
    cv::imshow("srcImg", srcImg);
    cv::waitKey(-1);
  }

  return 0;
}