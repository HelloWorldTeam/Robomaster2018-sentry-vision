//
// Created by liming on 6/1/18.
//

#include "MVCamera.h"
#include "Timer.h"

int main(int argc, char** argv) {

  printf("MVCamera Init\n");
  MVCamera::Init();
  printf("MVCamera Play\n");
  MVCamera::Play();
  //printf("MVCamera SetWBMode\n");
  //MVCamera::SetWBMode(true);
  printf("MVCamera SetLargeResolution\n");
  MVCamera::SetLargeResolution(true);
  printf("MVCamera SetExposureTime\n");
  MVCamera::SetExposureTime(false, 20000);
  double read_exp_time = MVCamera::GetExposureTime();
  printf("read_exp_time: %lf\n", read_exp_time);
  //printf("Set Gain\n");
  //MVCamera::SetGain(1);
  bool auto_wb;
  printf("MVCamera GetWBMode\n");
  MVCamera::GetWBMode(auto_wb);
  printf("MVCamera SetOnceWB\n");
  MVCamera::SetOnceWB();

  cv::VideoWriter videoWriter("test.avi", CV_FOURCC('M', 'J', 'P', 'G'), 100, cv::Size(752, 480));

  int i = 0;
  double exp_time = 10000;
  bool   auto_exp = false;
  bool   large_resolution = false;
  Mat frame;
  while (i < 1e10) {

    //TIME_BEGIN()
    MVCamera::GetFrame(frame);
    //TIME_END("MVCamera::Read")

    if (frame.empty()) {
      cv::waitKey(10);
      continue;
    }

    videoWriter.write(frame);
    cv::imshow("read", frame);
    char cmd = waitKey(10);

    if (cmd == 'a') {
      auto_exp = !auto_exp;
      MVCamera::SetExposureTime(auto_exp, exp_time);
      double read_exp_time = MVCamera::GetExposureTime();
      printf("read_exp_time: %lf\n", read_exp_time);
    }

    if (cmd == 'w') {
      exp_time += 10000;
      MVCamera::SetExposureTime(auto_exp, exp_time);
      double read_exp_time = MVCamera::GetExposureTime();
      printf("read_exp_time: %lf\n", read_exp_time);
    }

    if (cmd == 's') {
      if (exp_time-10000 > 0)
        exp_time -= 10000;
      MVCamera::SetExposureTime(auto_exp, exp_time);
      double read_exp_time = MVCamera::GetExposureTime();
      printf("read_exp_time: %lf\n", read_exp_time);
    }

    if (cmd == 'b') {
      large_resolution = !large_resolution;
      MVCamera::SetLargeResolution(large_resolution);
    }

    if (cmd == 'q') {
      break;
    }

    i++;
  }
  videoWriter.release();
  MVCamera::Stop();
  MVCamera::Uninit();

  return 0;
}
