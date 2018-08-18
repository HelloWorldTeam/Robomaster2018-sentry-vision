//
// Created by liming on 6/11/18.
//

#ifndef ROBOMASTERMARKERDETECTOR3_MARKERSENSOR_H
#define ROBOMASTERMARKERDETECTOR3_MARKERSENSOR_H

#include <memory>
#include <opencv2/opencv.hpp>
#include "MarkerParams.h"
#include "KCFTracker/kcftracker.hpp"

using namespace std;
using namespace cv;

class RotRect {
public:
  cv::Point2f center;
  cv::Point2f dir;
  float width;
  float height;

  RotRect(): width(0), height(0) {};
  RotRect(const cv::Rect & rect):
      width(rect.width), height(rect.height)
  {
    center.x = rect.x + rect.width*0.5f;
    center.y = rect.y + rect.height*0.5f;
    dir.x = 1;
    dir.y = 0;
  };
};

class Marker {
public:
  RotRect   LEDs[2];
  Point2f   kpts[4];
  Rect      bbox;
  Marker() = default;
  int ComputeKeyPoints()
  {
    kpts[0] = LEDs[0].center + LEDs[0].dir*LEDs[0].width*0.5f;
    kpts[1] = LEDs[1].center + LEDs[1].dir*LEDs[1].width*0.5f;
    kpts[2] = LEDs[1].center - LEDs[1].dir*LEDs[1].width*0.5f;
    kpts[3] = LEDs[0].center - LEDs[0].dir*LEDs[0].width*0.5f;
    return 0;
  };
  int ComputeBBox()
  {
    float max_x = 0  , max_y = 0;
    float min_x = 999, min_y = 999;
    for (int i = 0; i < 4; ++i) {
      Point2f & kpt = kpts[i];
      if (kpt.x < min_x) {
        min_x = kpt.x;
      }
      if (kpt.x > max_x) {
        max_x = kpt.x;
      }
      if (kpt.y < min_y) {
        min_y = kpt.y;
      }
      if (kpt.y > max_y) {
        max_y = kpt.y;
      }
    }
    bbox.x = min_x;
    bbox.y = min_y;
    bbox.width = (max_x - min_x);
    bbox.height = (max_y - min_y);
    return 0;
  };
  int Draw(Mat & img) {
    ComputeKeyPoints();
    cv::line(img, kpts[0], kpts[1], cv::Scalar(255,0,0), 3);
    cv::line(img, kpts[1], kpts[2], cv::Scalar(0,255,0), 3);
    cv::line(img, kpts[2], kpts[3], cv::Scalar(0,0,255), 3);
    cv::line(img, kpts[3], kpts[0], cv::Scalar(255,255,0), 3);
    return 0;
  };
};

class MarkerSensor {
public:

  enum SensorStatus {
    STATUS_SUCCESS = 0,
    STATUS_TRACKING,
    STATUS_DETECTING
  };

  enum MarkerType {
    ALL = 0,
    RED = 1,
    BLUE = 2
  };

  MarkerSensor() = default;
  MarkerSensor(const string & calibration, const string & config, const string & cascade);
  int LoadCalibration(const string & calibration);
  int LoadConfig(const string & config);
  int LoadCascade(const string & cascade);
  int ConvertImg(const cv::Mat & img);
  int ConvertColor();
  int DetectCentroidMarker(cv::Rect & res);
  int ResetTracker(const cv::Rect & rect);
  int TrackMarker(cv::Rect & res);
  int DetectAndTrack();
  int ComputeMarkerType();
  int GetMarkerPosition(float & X, float & Y, float & Z, bool is_big = false);
  int GetMarker(cv::Mat & roi_mask, RotRect & res_marker);
  int PCALEDStrip(vector<cv::Point> &contour, RotRect &LED);
  float ComputeLengthAlongDir(vector<cv::Point> & contour, cv::Point2f & dir);
  int ProcessFrameXY(const Mat & img, float & X, float & Y, float& depth);
  int ProcessFrameXYZ(const Mat & img, float & X, float & Y, float & Z);
  int SetTargetRed();
  int SetTargetBlue();
  int GetLEDMarker(cv::Mat& roi_mask, Marker & res_marker);
  int DetectLEDMarker(Marker & res_marker);
  int TrackLEDMarker(Marker & res_marker);
  int ProcessFrameLEDXYZ(const Mat & img, float & X, float & Y, float & Z, int &type);
  int ProcessFrameLEDXY(const Mat & img, float & X, float & Y);
  int ProcessFrameLEDXYz(const Mat & img, float & X, float & Y, float & z);
  cv::Mat GetMatH(cv::Mat img_hsv);

  Mat img_gray, img_bgr, img_hsv, img_h, led_mask;
  shared_ptr<cv::CascadeClassifier> p_cascade_classifier_;
  shared_ptr<KCFTracker> p_kcf_tracker_;
  SensorStatus status;
  cv::Rect tracking_result;
  Marker marker;
  float   old_depth,  depth;
  Point2f old_target, target;

};


#endif //ROBOMASTERMARKERDETECTOR3_MARKERSENSOR_H
