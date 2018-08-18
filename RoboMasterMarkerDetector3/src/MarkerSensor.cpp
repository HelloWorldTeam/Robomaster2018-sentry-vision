//
// Created by liming on 6/11/18.
//

#include "MarkerSensor.h"
#include "Timer.h"

MarkerSensor::MarkerSensor(const string &calibration, const string &config, const string &cascade)
{
  printf("Construct MarkerSensor\n");
  printf("Load Calibration...\n");
  LoadCalibration(calibration);
  printf("Load Config...\n");
  LoadConfig(config);
  printf("Load Cascade...\n");
  LoadCascade(cascade);

  p_kcf_tracker_.reset(new KCFTracker(true, true, false, false));

  status = STATUS_DETECTING;

  old_depth = depth = 0;
  old_target = target = cv::Point2f(0, 0);
}

int MarkerSensor::LoadCalibration(const string &calibration)
{
  /// read calibration parameters
  cv::FileStorage fCalibration(calibration, cv::FileStorage::READ);
  MarkerParams::camera_width = fCalibration["Camera.width"];
  MarkerParams::camera_height = fCalibration["Camera.height"];
  MarkerParams::camera_fps = fCalibration["Camera.fps"];
  MarkerParams::camera_fx = fCalibration["Camera.fx"];
  MarkerParams::camera_fy = fCalibration["Camera.fy"];
  MarkerParams::camera_cx = fCalibration["Camera.cx"];
  MarkerParams::camera_cy = fCalibration["Camera.cy"];
  MarkerParams::camera_k1 = fCalibration["Camera.k1"];
  MarkerParams::camera_k2 = fCalibration["Camera.k2"];
  MarkerParams::camera_k3 = fCalibration["Camera.k3"];
  cout << "===Camera Calibration===" << endl;
  cout << "width: " << MarkerParams::camera_width << endl;
  cout << "height: " << MarkerParams::camera_height << endl;
  cout << "fps: " << MarkerParams::camera_fps << endl;
  cout << "fx: " << MarkerParams::camera_fx << endl;
  cout << "fy: " << MarkerParams::camera_fy << endl;
  cout << "cx: " << MarkerParams::camera_cx << endl;
  cout << "cy: " << MarkerParams::camera_cy << endl;
  cout << "k1: " << MarkerParams::camera_k1 << endl;
  cout << "k2: " << MarkerParams::camera_k2 << endl;
  cout << "k3: " << MarkerParams::camera_k3 << endl;
  return 0;
}

int MarkerSensor::LoadConfig(const string &config)
{
  /// read config parameters
  cv::FileStorage fConfig(config, cv::FileStorage::READ);
  /// ImageFrame
  MarkerParams::blur_sz = fConfig["ImageFrame.blur_size"];
  MarkerParams::blur_sigma = fConfig["ImageFrame.blur_sigma"];
  /// Detector
  MarkerParams::hmin = fConfig["Detector.hmin"];
  MarkerParams::hmax = fConfig["Detector.hmax"];
  MarkerParams::blue_hmin = fConfig["Detector.blue_hmin"];
  MarkerParams::blue_hmax = fConfig["Detector.blue_hmax"];
  MarkerParams::red_hmin = fConfig["Detector.red_hmin"];
  MarkerParams::red_hmax = fConfig["Detector.red_hmax"];
  MarkerParams::smin = fConfig["Detector.smin"];
  MarkerParams::smax = fConfig["Detector.smax"];
  MarkerParams::vmin = fConfig["Detector.vmin"];
  MarkerParams::vmax = fConfig["Detector.vmax"];
  MarkerParams::contours_length_min = fConfig["Detector.contours_length_min"];
  MarkerParams::contours_length_max = fConfig["Detector.contours_length_max"];
  MarkerParams::LED_ratio_min = fConfig["Detector.LED_ratio_min"];
  MarkerParams::LED_ratio_max = fConfig["Detector.LED_ratio_max"];
  MarkerParams::LED_width_min = fConfig["Detector.LED_width_min"];
  MarkerParams::LED_width_max = fConfig["Detector.LED_width_max"];
  MarkerParams::marker_parallel_angle = fConfig["Detector.marker_parallel_angle"];
  MarkerParams::marker_vertical_angle = fConfig["Detector.marker_vertical_angle"];
  MarkerParams::marker_direction_angle = fConfig["Detector.marker_direction_angle"];
  MarkerParams::marker_ratio_min = fConfig["Detector.marker_ratio_min"];
  MarkerParams::marker_ratio_max = fConfig["Detector.marker_ratio_max"];
  MarkerParams::marker_size_min = fConfig["Detector.marker_size_min"];
  MarkerParams::marker_size_max = fConfig["Detector.marker_size_max"];
  /// Transformer
  MarkerParams::transformer_gray_min = fConfig["Transformer.gray_min"];
  MarkerParams::transformer_gray_max = fConfig["Transformer.gray_max"];
  MarkerParams::transformer_area_min = fConfig["Transformer.area_min"];
  MarkerParams::transformer_area_max = fConfig["Transformer.area_max"];
  MarkerParams::transformer_c2_s_ratio_min = fConfig["Transformer.c2_s_ratio_min"];
  MarkerParams::transformer_c2_s_ratio_max = fConfig["Transformer.c2_s_ratio_max"];
  MarkerParams::transformer_ellipse_epsi = fConfig["Transformer.ellipse_epsi"];
  MarkerParams::transformer_ellipse_inlier_ratio = fConfig["Transformer.ellipse_inlier_ratio"];
  MarkerParams::transformer_ellipse_radius = fConfig["Transformer.ellipse_radius"];
  /// target
  MarkerParams::target_color = fConfig["Target.color"];
  MarkerParams::target_size = fConfig["Target.size"];

  MarkerParams::target_bubing_shift_x = fConfig["Target.bubing.shift_x"];
  MarkerParams::target_bubing_shift_y = fConfig["Target.bubing.shift_y"];
  MarkerParams::target_bubing_shift_z = fConfig["Target.bubing.shift_z"];
  MarkerParams::target_bubing_L = fConfig["Target.bubing.L"];

  MarkerParams::target_shaobing_shift_x = fConfig["Target.shaobing.shift_x"];
  MarkerParams::target_shaobing_shift_y = fConfig["Target.shaobing.shift_y"];
  MarkerParams::target_shaobing_shift_z = fConfig["Target.shaobing.shift_z"];
  MarkerParams::target_shaobing_L = fConfig["Target.shaobing.L"];

  MarkerParams::target_yingxiong_shift_x = fConfig["Target.yingxiong.shift_x"];
  MarkerParams::target_yingxiong_shift_y = fConfig["Target.yingxiong.shift_y"];
  MarkerParams::target_yingxiong_shift_z = fConfig["Target.yingxiong.shift_z"];
  MarkerParams::target_yingxiong_L = fConfig["Target.yingxiong.L"];

  cout << "===Imageframe Config===" << endl;
  cout << "blur_size: " << MarkerParams::blur_sz << endl;
  cout << "blur_sigma: " << MarkerParams::blur_sigma << endl;
  cout << "===Detector Config===" << endl;
  cout << "hmin: " << MarkerParams::hmin << endl;
  cout << "hmax: " << MarkerParams::hmax << endl;
  cout << "blue_hmin: " << MarkerParams::blue_hmin << endl;
  cout << "blue_hmax: " << MarkerParams::blue_hmax << endl;
  cout << "red_hmin: " << MarkerParams::red_hmin << endl;
  cout << "red_hmax: " << MarkerParams::red_hmax << endl;
  cout << "smin: " << MarkerParams::smin << endl;
  cout << "smax: " << MarkerParams::smax << endl;
  cout << "vmin: " << MarkerParams::vmin << endl;
  cout << "vmax: " << MarkerParams::vmax << endl;
  cout << "contours_length_min: " << MarkerParams::contours_length_min << endl;
  cout << "contours_length_max: " << MarkerParams::contours_length_max << endl;
  cout << "LED_ratio_min: " << MarkerParams::LED_ratio_min << endl;
  cout << "LED_ratio_max: " << MarkerParams::LED_ratio_max << endl;
  cout << "LED_width_min: " << MarkerParams::LED_width_min << endl;
  cout << "LED_width_max: " << MarkerParams::LED_width_max << endl;
  cout << "marker_parallel_angle: " << MarkerParams::marker_parallel_angle << endl;
  cout << "marker_vertical_angle: " << MarkerParams::marker_vertical_angle << endl;
  cout << "marker_direction_angle: " << MarkerParams::marker_direction_angle << endl;
  cout << "marker_ratio_min: " << MarkerParams::marker_ratio_min << endl;
  cout << "marker_ratio_max: " << MarkerParams::marker_ratio_max << endl;
  cout << "marker_size_min: " << MarkerParams::marker_size_min << endl;
  cout << "marker_size_max: " << MarkerParams::marker_size_max << endl;
  cout << "===Transformer Config===" << endl;
  cout << "gray_min: " << MarkerParams::transformer_gray_min << endl;
  cout << "gray_max: " << MarkerParams::transformer_gray_max << endl;
  cout << "area_min: " << MarkerParams::transformer_area_min << endl;
  cout << "area_max: " << MarkerParams::transformer_area_max << endl;
  cout << "c2_s_ratio_min: " << MarkerParams::transformer_c2_s_ratio_min << endl;
  cout << "c2_s_ratio_max: " << MarkerParams::transformer_c2_s_ratio_max << endl;
  cout << "ellipse_epsi: " << MarkerParams::transformer_ellipse_epsi << endl;
  cout << "ellipse_inlier_ratio: " << MarkerParams::transformer_ellipse_inlier_ratio << endl;
  cout << "===Target Config===" << endl;
  cout << "target color: " << MarkerParams::target_color << endl;
  cout << "target size: " << MarkerParams::target_size << endl;
  cout << "===Hardware setup===" << endl;
  cout << "Bubing shift_x:" << MarkerParams::target_bubing_shift_x << endl;
  cout << "Bubing shift_y:" << MarkerParams::target_bubing_shift_y << endl;
  cout << "Bubing shift_z:" << MarkerParams::target_bubing_shift_z << endl;
  cout << "Bubing L:" << MarkerParams::target_bubing_L << endl;
  cout << "Shaobing shift_x:" << MarkerParams::target_shaobing_shift_x << endl;
  cout << "Shaobing shift_y:" << MarkerParams::target_shaobing_shift_y << endl;
  cout << "Shaobing shift_z:" << MarkerParams::target_shaobing_shift_z << endl;
  cout << "Shaobing L:" << MarkerParams::target_shaobing_L << endl;
  cout << "Yingxiong shift_x:" << MarkerParams::target_yingxiong_shift_x << endl;
  cout << "Yingxiong shift_y:" << MarkerParams::target_yingxiong_shift_y << endl;
  cout << "Yingxiong shift_z:" << MarkerParams::target_yingxiong_shift_z << endl;
  cout << "Yingxiong L:" << MarkerParams::target_yingxiong_L << endl;
  return 0;
}

int MarkerSensor::LoadCascade(const string &cascade)
{
  p_cascade_classifier_.reset(new cv::CascadeClassifier(cascade));
  return 0;
}

int MarkerSensor::DetectCentroidMarker(cv::Rect &res)
{
  vector<cv::Rect> objs;
  p_cascade_classifier_->detectMultiScale(img_gray, objs, 1.2, 3);
  if (objs.size() == 0) {
    printf("DetectMultiScale result's size is 0\n");
    return -1;
  }

  double min_dist = 999999;
  cv::Point2f center(MarkerParams::camera_cx, MarkerParams::camera_cy);
  for (auto & obj:objs) {
    cv::Point2f target(obj.x + obj.width*0.5f, obj.y + obj.height*0.5f);
    double dist = cv::norm(target - center);
    if (dist < min_dist) {
      min_dist = dist;
      res = obj;
    }
  }

  return 0;
}

int MarkerSensor::ResetTracker(const cv::Rect &rect)
{
  ///make the box be 16x
  cv::Rect box = rect;
  if (box.width%16) {
      int new_width = int(box.width/16)*16;
      box.x = box.x + (box.width - new_width)/2;
      box.width = new_width;
  }
  if (box.height%16) {
      int new_height = int(box.height/16)*16;
      box.y = box.y + (box.height - new_height)/2;
      box.height = new_height;
  }

  p_kcf_tracker_->init(box, img_bgr);

  return 0;
}

int MarkerSensor::TrackMarker(cv::Rect &res)
{

  float peak_value = 0;
  res = p_kcf_tracker_->update(img_bgr, peak_value);
  if (peak_value < 0.25f) {
    printf("Tracking peak value < 0.25f\n");
    return -1;
  }

  return 0;
}

int MarkerSensor::ComputeMarkerType()
{
  cv::Rect box = tracking_result;
  box.width  = tracking_result.width * 3;
  box.height = tracking_result.height * 2;
  box.x = int(tracking_result.x - (box.width - tracking_result.width)*0.5f);
  box.y = int(tracking_result.y - (box.height - tracking_result.height)*0.5f);

  /// check roi
  float min_x = box.x,
      max_x = box.x + box.width,
      min_y = box.y,
      max_y = box.y + box.height;
  min_x = min_x < 0 ? 0 : min_x;
  min_y = min_y < 0 ? 0 : min_y;
  max_x = max_x >= MarkerParams::camera_width ? MarkerParams::camera_width-1 : max_x;
  max_y = max_y >= MarkerParams::camera_height ? MarkerParams::camera_height-1 : max_y;
  cv::Rect valid_box(min_x, min_y, (max_x - min_x), (max_y - min_y));

  /// crop roi
  cv::Mat roi_bgr  = img_bgr(valid_box).clone();
  cv::Mat roi_gray = img_gray(valid_box).clone();
  cv::Mat roi_hsv, roi_h;
  cv::cvtColor(roi_bgr, roi_hsv, CV_BGR2HSV);
  roi_h = GetMatH(roi_hsv);

  /// H histogram
  vector<float> H_hist(180, 0);
  unsigned char * _data = roi_h.data;
  unsigned char * _data_gray = roi_gray.data;
  int pixel_num = 0;
  for (int y = 0; y < valid_box.height; ++y) {
    for (int x = 0; x < valid_box.width; ++x) {
      //printf("h: %d\n", int(_data[x]));
      if (_data_gray[x] > 100) {
        ++H_hist[_data[x]];
        ++pixel_num;
      }
    }
    _data += roi_h.cols;
    _data_gray += roi_h.cols;
  }
  for (auto & val:H_hist) {
    val /= pixel_num;
  }

  float red_max = 0, blue_max = 0;
  for (int i = 100; i < 120; ++i) {
    if (H_hist[i] > blue_max)
      blue_max = H_hist[i];
  }
  for (int i = 160; i < 180; ++i) {
    if (H_hist[i] > red_max)
      red_max = H_hist[i];
  }
//  for (int i = 0; i < 10; ++i) {
//    if (H_hist[i] > red_max)
//      red_max = H_hist[i];
//  }

  MarkerType marker_type;
  if (red_max > blue_max) {
    marker_type = RED;
  } else {
    marker_type = BLUE;
  }
  printf("Red: %f, Blue: %f\n", red_max, blue_max);

#ifdef __SHOWIMG__
//  /// show histogram
//  int res_width = 640, res_height = 480;
//  int step = res_width / 180.f;
//  cv::Mat res = cv::Mat::zeros(res_height, res_width, CV_8UC3);
//  for (int x = 0; x < 180; x++) {
//    int xx = x*step;
//    int h = res_height*H_hist[x];
//    cv::rectangle(res, cv::Rect(xx, res_height-h, step, h), cv::Scalar(x, 255, 255), -1 );
//  }
//  cv::cvtColor(res, res, CV_HSV2BGR);
//  cv::imshow("track_histogram", res);
#endif

  return marker_type;
}

cv::Mat MarkerSensor::GetMatH(cv::Mat img_hsv_) {
  cv::Mat img_h_;
  TIME_BEGIN();
  int from_to[] = {0, 0};
  img_h_.create(img_hsv_.size(), CV_8UC1);
  cv::mixChannels(&img_hsv_, 1, &img_h_, 1, from_to, 1);
  TIME_END("MixChannels");
  return img_h_;
}

int MarkerSensor::ConvertImg(const cv::Mat &img)
{
  TIME_BEGIN();
  if (img.cols != MarkerParams::camera_width || img.rows != MarkerParams::camera_height) {
    cv::resize(img, img_bgr, cv::Size(MarkerParams::camera_width, MarkerParams::camera_height));
    printf("Resize img!\n");
  } else {
    img.copyTo(img_bgr);
    printf("Copy img!\n");
  }
  TIME_END("ResizeOrCopy");

  //cv::GaussianBlur(img_bgr, img_bgr, cv::Size(MarkerParams::blur_sz, MarkerParams::blur_sz), MarkerParams::blur_sigma);

  return 0;
}

int MarkerSensor::ConvertColor()
{
 //  TIME_BEGIN();
//  cv::cvtColor(img_bgr, img_hsv, CV_BGR2HSV);
//  TIME_END("cvtColorHsv");
//
  TIME_BEGIN();
  cv::cvtColor(img_bgr, img_gray, CV_BGR2GRAY);
  TIME_END("cvtColorGray");

//  img_h = GetMatH(img_hsv);

//  cv::Mat res(img_h.size(), CV_8UC3);
//  unsigned char * _img_h = img_h.data;
//  unsigned char * _res = res.data;
//  for (int i = 0, _end = img_h.cols*img_h.rows; i < _end; ++i) {
//    *(_res + 0) = *_img_h;
//    *(_res + 1) = 255;
//    *(_res + 2) = 255;
//    _img_h++;
//    _res+=3;
//  }
//  cv::cvtColor(res, res, CV_HSV2BGR);
//  cv::imshow("hue_res", res);

  return 0;
}

int MarkerSensor::DetectAndTrack()
{
  if (status == STATUS_DETECTING) {
    cv::Rect result;
    if (DetectCentroidMarker(result) == STATUS_SUCCESS) {
      tracking_result = result;
      ResetTracker(tracking_result);
      status = STATUS_TRACKING;
#ifdef __SHOWIMG__
      cv::Mat res = img_bgr.clone();
      cv::rectangle(res, tracking_result, cv::Scalar(0,255,0), 3);
      cv::imshow("detect", res);
#endif
      float focal_length = (MarkerParams::camera_fx + MarkerParams::camera_fy) * 0.5f;
      float real_L = MarkerParams::transformer_small_marker_size;
      this->depth = (real_L*focal_length)/tracking_result.width;
    }
  } else {
    cv::Rect result;
    if (TrackMarker(result) == STATUS_SUCCESS) {
      tracking_result = result;
#ifdef __SHOWIMG__
      cv::Mat res = img_bgr.clone();
      cv::rectangle(res, tracking_result, cv::Scalar(255,0,0), 3);
      cv::imshow("tracking", res);
#endif
    } else {
      status = STATUS_DETECTING;
    }
  }

  return status;
}

int MarkerSensor::PCALEDStrip(vector<cv::Point> &contour, RotRect &LED)
{
    int sz = static_cast<int>(contour.size());
    cv::Mat data_pts(sz, 2, CV_64FC1);
    double* _data_pts = (double*)data_pts.data;
    for (int i = 0; i < data_pts.rows; ++i, _data_pts+=2) {
        _data_pts[0] = contour[i].x;
        _data_pts[1] = contour[i].y;
    }
    cv::PCA pca_analysis(data_pts, cv::Mat(), CV_PCA_DATA_AS_ROW);
    LED.center.x = static_cast<float>( pca_analysis.mean.at<double>(0, 0) );
    LED.center.y = static_cast<float>( pca_analysis.mean.at<double>(0, 1) );
    cv::Point2f dir1, dir2;
    dir1.x = static_cast<float>( pca_analysis.eigenvectors.at<double>(0, 0) );
    dir1.y = static_cast<float>( pca_analysis.eigenvectors.at<double>(0, 1) );
    dir2.x = static_cast<float>( pca_analysis.eigenvectors.at<double>(1, 0) );
    dir2.y = static_cast<float>( pca_analysis.eigenvectors.at<double>(1, 1) );

    dir1 = dir1 * (1 / cv::norm(dir1));
    dir2 = dir2 * (1 / cv::norm(dir2));

    LED.dir = dir1;
    LED.width = ComputeLengthAlongDir(contour, dir1);
    LED.height = ComputeLengthAlongDir(contour, dir2);

    return 0;
}

float MarkerSensor::ComputeLengthAlongDir(vector<cv::Point> &contour, cv::Point2f &dir)
{
    float max_range = -999999;
    float min_range = 999999;
    for (auto & pt:contour) {
        float x = pt.x*dir.x + pt.y*dir.y;
        if (x < min_range) min_range = x;
        if (x > max_range) max_range = x;
    }
    return (max_range - min_range);
}

int MarkerSensor::GetMarker(cv::Mat &roi_mask, RotRect &res_marker)
{
 /// FindContours and check length
  vector< vector< cv::Point > > tmp_contours;
  vector< vector< cv::Point >* > pContours;
  cv::findContours(roi_mask, tmp_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
  for (auto & contour:tmp_contours) {
    int contour_sz = static_cast<int>(contour.size());
    if (contour_sz >= MarkerParams::contours_length_min && contour_sz <= MarkerParams::contours_length_max) {
      pContours.push_back(&contour);
    }
  }
  /// PCA - Get LED Strip
  vector< RotRect > LEDs;
  LEDs.reserve(10);
  for (auto & pContour:pContours) {
    RotRect LED;
    if (PCALEDStrip(*pContour, LED) == STATUS_SUCCESS) {
      /// check ratio and length
      if (LED.width < MarkerParams::LED_width_min || LED.width > MarkerParams::LED_width_max) continue;
      float ratio = LED.width / LED.height;
      if (ratio < MarkerParams::LED_ratio_min || ratio > MarkerParams::LED_ratio_max) continue;
      LEDs.push_back(LED);
    }
  }
#ifdef __SHOWIMG__
//  MarkerDetector::ShowContoursAndLEDs(roi_mask, pContours, LEDs); /// show contours and LEDS, for dev
#endif

  if (LEDs.size() < 2) {
    printf("LED num < 2 ! \n");
    return -1;
  }

  /// search marker
  vector<RotRect> markers;
  float cos_marker_parallel_radian = cos(MarkerParams::marker_parallel_angle/180.f*3.1415926f);
  float cos_marker_vertical_radian = cos((90 - MarkerParams::marker_vertical_angle)/180.f*3.1415926f);
  float cos_marker_direction_radian =  cos(MarkerParams::marker_direction_angle/180.f*3.1415926f);
  size_t LED_sz = LEDs.size();
  vector<bool> matched(LED_sz, false);
  for (size_t i = 0; i < LED_sz; ++i) {
    if (matched[i]) continue;
    for (size_t j = i+1; j < LED_sz; ++j) {
      if (matched[j]) continue;
      //printf("###\nCheck pair: %lu %lu\n", i, j);
      cv::Point2f c2c = LEDs[i].center - LEDs[j].center;
      /// check width difference
      float max_width = max(LEDs[i].width, LEDs[j].width);
      if (fabs(LEDs[i].width - LEDs[j].width)/max_width > 0.3) {
        //printf("LED difference not satisfied !\n");
        continue;
      }
      /// check distance
      float distance = cv::norm(c2c);
      if (distance > MarkerParams::marker_size_max || distance < MarkerParams::marker_size_min) {
        //printf("LED distance not satisfied !\n");
        continue;
      }
      /// check parallel
      if (fabs(LEDs[i].dir.dot(LEDs[j].dir)) < cos_marker_parallel_radian) {
        //printf("LED parallel not satisfied !\n");
        continue;
      }
      /// check vertical
      c2c = c2c * (1.0 / cv::norm(c2c));
      if (fabs(LEDs[i].dir.dot(c2c)) > cos_marker_vertical_radian) {
        //printf("LED vertical not satisfied !\n");
        continue;
      }
      /// check direction
      if (fabs(c2c.dot(cv::Point2f(1, 0))) < cos_marker_direction_radian) {
        //printf("Marker direction not satisfied !\n");
        continue;
      }
      /// build marker
      RotRect marker;
      marker.width = distance;
      marker.height = (LEDs[i].width + LEDs[j].width)*0.5f;

      /// check marker width/height ratio
      float marker_size_ratio = marker.width / marker.height;
      if (marker_size_ratio > MarkerParams::marker_ratio_max || marker_size_ratio < MarkerParams::marker_ratio_min) {
        printf("Marker size ratio not satisfied !\n");
        continue;
      }

      marker.center = (LEDs[i].center + LEDs[j].center)*0.5f;
      marker.dir = c2c;

      matched[i] = matched[j] = true;
      //printf("###\n");
      markers.push_back(marker);
      break;  /// find one matched, the nearest, break
    }
  }
  if (markers.size() == 0) {
    printf("no marker found !\n");
    return -1;
  } else {
    printf("find a marker !\n");
  }
#ifdef __SHOWIMG__
//  MarkerDetector::ShowMarkers(roi_mat, markers); /// show markers, for dev
#endif

  res_marker = markers[0];

  return 0;
}

int MarkerSensor::GetMarkerPosition(float &X, float &Y, float &Z, bool is_big)
{
  /// get roi mat
  cv::Rect box = tracking_result;
  cv::Point2f center(box.x + box.width*0.5f, box.y + box.height*0.5f);
  float scale = 1;
  if (is_big)
    scale = 2;
  float left = center.x - box.width*scale;
  float right = center.x + box.width*scale;
  float top = center.y - box.height;
  float bot = center.y + box.height;
  left = left < 0 ? 0 : left;
  top = top < 0 ? 0 : top;
  right = right >= MarkerParams::camera_width ? MarkerParams::camera_width-1 : right;
  bot = bot >= MarkerParams::camera_height ? MarkerParams::camera_height-1 : bot;
  cv::Rect roi = cv::Rect(left, top, right - left, bot - top);
  cv::Mat roi_mat = img_gray(roi);
  /// 归一化
//  roi_mat.convertTo(roi_mat, CV_32FC1);
//  double mean_val = cv::mean(roi_mat)[0];
//  printf("mean_val of roi_mat: %lf\n", mean_val);
//  roi_mat = roi_mat - mean_val;
//  double min_val;
//  double max_val;
//  cv::minMaxLoc(roi_mat, &min_val, &max_val, NULL, NULL);
//  roi_mat = roi_mat + min_val;
//  roi_mat = roi_mat.mul(255.0/(max_val-min_val));
//  roi_mat.convertTo(roi_mat, CV_8UC1);
#ifdef __SHOWIMG__
  cv::imshow("roi_mat", roi_mat);
#endif
  double max_val = 0;
  cv::minMaxLoc(roi_mat, NULL, &max_val, NULL, NULL);
  printf("max_val in roi_mat: %lf\n", max_val);

  /// Get Marker
  cv::Mat roi_mask;
  RotRect marker;
  float thresholds[3] = {0.7f, 0.6f, 0.5f};
  bool find_marker = false;
  for (auto th:thresholds) {
    printf("try to find marker at %f*max_val ...\n", th);
    cv::inRange(roi_mat, int(th*max_val), int(max_val), roi_mask);
#ifdef __SHOWIMG__
    cv::imshow("roi_mask", roi_mask);
#endif
    find_marker = (GetMarker(roi_mask, marker) == STATUS_SUCCESS);
    if (find_marker)
      break;
  }

  if (!find_marker) {
    printf("find no marker !\n");
    return -1;
  }

  /// Shift by roi
  marker.center.x += roi.x;
  marker.center.y += roi.y;

  /// ShowMarker
#ifdef __SHOWIMG__
  vector<cv::Point2f> key_points(4);
  cv::Point2f dir2(-marker.dir.y, marker.dir.x);
  key_points[0] = center - marker.width*0.5*marker.dir -marker.height*dir2*0.5;
  key_points[1] = center + marker.width*0.5*marker.dir -marker.height*dir2*0.5;
  key_points[2] = center + marker.width*0.5*marker.dir +marker.height*dir2*0.5;
  key_points[3] = center - marker.width*0.5*marker.dir +marker.height*dir2*0.5;
  cv::Mat res = img_bgr.clone();
  cv::line(res, key_points[0], key_points[1], cv::Scalar(0, 255, 0), 3);
  cv::line(res, key_points[1], key_points[2], cv::Scalar(0, 255, 0), 3);
  cv::line(res, key_points[2], key_points[3], cv::Scalar(0, 255, 0), 3);
  cv::line(res, key_points[3], key_points[0], cv::Scalar(0, 255, 0), 3);
  cv::imshow("Marker", res);
#endif

  /// update 3d position
  float focal_length = (MarkerParams::camera_fx + MarkerParams::camera_fy)*0.5f;
  if (is_big) {
    printf("Big Marker !\n");
    float real_L = MarkerParams::transformer_big_marker_size;
    depth = (real_L*focal_length) / (marker.width);
  } else {
    printf("Small Marker !\n");
    float real_L = MarkerParams::transformer_ellipse_radius;
    depth = (real_L*focal_length) / (marker.width);
  }

  ///使用中值滤波
  // TODO::

  ///使用互补滤波
  if (old_depth > 0) {
    printf("old_depth:%f new_depth:%f", old_depth, depth);
    depth = 0.6f*depth + 0.4f*old_depth;    //TODO:Adjust the params
    old_depth = depth;
    printf(" filtered_depth:%f\n", depth);
  } else {
    old_depth = depth;
  }

  /// 设置结果
  X = (marker.center.x - MarkerParams::camera_cx) / MarkerParams::camera_fx * depth;
  Y = (marker.center.y - MarkerParams::camera_cy) / MarkerParams::camera_fy * depth;
  Z = depth;

  return 0;
}

int MarkerSensor::ProcessFrameXY(const Mat &img, float &X, float &Y, float &depth)
{
  /// 预处理图像
  ConvertImg(img);
  ConvertColor();

  /// 检测或跟踪图像
  DetectAndTrack();

  /// 判断检测或跟踪结果
  if (status != STATUS_TRACKING) {
    printf("No target !\n");
    return -1;
  }

  /// 判断颜色
  if (MarkerParams::target_color && ComputeMarkerType() != MarkerParams::target_color) {
    status = STATUS_DETECTING;
    printf("Marker color wrong !\n");
    return -2;
  }

  /// 获取深度
  depth = this->depth;

  /// 设置结果
  X = tracking_result.x + tracking_result.width*0.5f - MarkerParams::camera_cx;
  Y = tracking_result.y + tracking_result.height*0.5f - MarkerParams::camera_cy;

  return 0;
}

int MarkerSensor::ProcessFrameXYZ(const Mat &img, float &X, float &Y, float &Z)
{
  /// 预处理图像
  TIME_BEGIN();
  ConvertImg(img);
  ConvertColor();
  TIME_END("ConvertImg");

  /// 检测或跟踪图像
  TIME_BEGIN();
  DetectAndTrack();
  TIME_END("DetectAndTrack");

  /// 判断检测或跟踪结果
  if (status != STATUS_TRACKING) {
    printf("No target !\n");
    return -1;
  }

  /// 判断颜色
  TIME_BEGIN();
  if (MarkerParams::target_color && ComputeMarkerType() != MarkerParams::target_color) {
    status = STATUS_DETECTING;
    printf("Marker color wrong !\n");
    return -2;
  }
  TIME_END("ComputeColorType");

  /// 获取深度
  TIME_BEGIN();
  if (GetMarkerPosition(X, Y, Z, false) != STATUS_SUCCESS) {
    printf("Get Marker Small failed!\n");
    if (GetMarkerPosition(X, Y, Z, true) != STATUS_SUCCESS) {
      printf("Get Marker Big failed!\n");
      return -1;
    }
  }
  TIME_END("GetMarkerPosition");

  return 0;
}

int MarkerSensor::GetLEDMarker(cv::Mat &roi_mask, Marker &res_marker)
{
  /// FindContours and check length
  vector< vector< cv::Point > > tmp_contours;
  vector< vector< cv::Point >* > pContours;
  cv::findContours(roi_mask, tmp_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
  for (auto & contour:tmp_contours) {
    int contour_sz = static_cast<int>(contour.size());
    if (contour_sz >= MarkerParams::contours_length_min && contour_sz <= MarkerParams::contours_length_max) {
      pContours.push_back(&contour);
    }
  }
  /// PCA - Get LED Strip
  vector< RotRect > LEDs;
  LEDs.reserve(10);
  for (auto & pContour:pContours) {
    RotRect LED;
    if (PCALEDStrip(*pContour, LED) == STATUS_SUCCESS) {
      /// check ratio and length
      if (LED.width < MarkerParams::LED_width_min || LED.width > MarkerParams::LED_width_max) continue;
      float ratio = LED.width / LED.height;
      if (ratio < MarkerParams::LED_ratio_min || ratio > MarkerParams::LED_ratio_max) continue;
      LEDs.push_back(LED);
    }
  }
#ifdef __SHOWIMG__
//  MarkerDetector::ShowContoursAndLEDs(roi_mask, pContours, LEDs); /// show contours and LEDS, for dev
#endif

  if (LEDs.size() < 2) {
    printf("LED num < 2 ! \n");
    return -1;
  }

  /// search marker
  vector<Marker> markers;
  float cos_marker_parallel_radian = cos(MarkerParams::marker_parallel_angle/180.f*3.1415926f);
  float cos_marker_vertical_radian = cos((90 - MarkerParams::marker_vertical_angle)/180.f*3.1415926f);
  float cos_marker_direction_radian =  cos(MarkerParams::marker_direction_angle/180.f*3.1415926f);
  size_t LED_sz = LEDs.size();
  vector<bool> matched(LED_sz, false);
  for (size_t i = 0; i < LED_sz; ++i) {
    if (matched[i]) continue;
    for (size_t j = i+1; j < LED_sz; ++j) {
      if (matched[j]) continue;
      //printf("###\nCheck pair: %lu %lu\n", i, j);
      cv::Point2f c2c = LEDs[i].center - LEDs[j].center;
      /// check width difference
      float max_width = max(LEDs[i].width, LEDs[j].width);
      if (fabs(LEDs[i].width - LEDs[j].width)/max_width > 0.3) {
        //printf("LED difference not satisfied !\n");
        continue;
      }
      /// check distance
      float distance = cv::norm(c2c);
      if (distance > MarkerParams::marker_size_max || distance < MarkerParams::marker_size_min) {
        //printf("LED distance not satisfied !\n");
        continue;
      }
      /// check parallel
      if (fabs(LEDs[i].dir.dot(LEDs[j].dir)) < cos_marker_parallel_radian) {
        //printf("LED parallel not satisfied !\n");
        continue;
      }
      /// check vertical
      c2c = c2c * (1.0 / cv::norm(c2c));
      if (fabs(LEDs[i].dir.dot(c2c)) > cos_marker_vertical_radian) {
        //printf("LED vertical not satisfied !\n");
        continue;
      }
      /// check direction
      if (fabs(c2c.dot(cv::Point2f(1, 0))) < cos_marker_direction_radian) {
        //printf("Marker direction not satisfied !\n");
        continue;
      }
      /// build marker
      Marker marker;
      if (c2c.x > 0) {
        marker.LEDs[0] = LEDs[j];
        marker.LEDs[1] = LEDs[i];
      } else {
        marker.LEDs[0] = LEDs[i];
        marker.LEDs[1] = LEDs[j];
      }

      /// check marker width/height ratio
      float marker_width = distance;
      float marker_height = (LEDs[i].width + LEDs[j].width)*0.5f;
      float marker_size_ratio = marker_width / marker_height;
      if (marker_size_ratio > MarkerParams::marker_ratio_max || marker_size_ratio < MarkerParams::marker_ratio_min) {
        printf("Marker size ratio not satisfied !\n");
        continue;
      }

      matched[i] = matched[j] = true;
      //printf("###\n");
      markers.push_back(marker);
    }
  }

  if (markers.size() == 0) {
    printf("no marker found !\n");
    return -1;
  } else {
    printf("find a marker !\n");
  }
#ifdef __SHOWIMG__
//  MarkerDetector::ShowMarkers(roi_mat, markers); /// show markers, for dev
#endif

  /// Select Centroid one
  int   best_id = -1;
  float best_dist = 999;
  Point2f center(MarkerParams::camera_cx, MarkerParams::camera_cy);
  for (int i = 0, _end = (int)markers.size(); i < _end; ++i) {
    Point2f marker_center = markers[i].LEDs[0].center*0.5f + markers[i].LEDs[1].center*0.5f;
    float dist = cv::norm(marker_center - center);
    if (dist < best_dist) {
      best_dist = dist;
      best_id = i;
    }
  }
  res_marker = markers[best_id];

  return 0;
}

int MarkerSensor::DetectLEDMarker(Marker &res_marker)
{
  TIME_BEGIN();
    cv::cvtColor(img_bgr, img_hsv, CV_BGR2HSV);
  TIME_END("cvtColorHsv");

  if (MarkerParams::target_color == 1) {
    cv::inRange(img_hsv, cv::Scalar(MarkerParams::red_hmin, MarkerParams::smin, MarkerParams::vmin),
                cv::Scalar(MarkerParams::red_hmax, MarkerParams::smax, MarkerParams::vmax), led_mask);
  } else if (MarkerParams::target_color == 2) {
    cv::inRange(img_hsv, cv::Scalar(MarkerParams::blue_hmin, MarkerParams::smin, MarkerParams::vmin),
                cv::Scalar(MarkerParams::blue_hmax, MarkerParams::smax, MarkerParams::vmax), led_mask);
  } else {
    cv::inRange(img_hsv, cv::Scalar(MarkerParams::hmin, MarkerParams::smin, MarkerParams::vmin),
                cv::Scalar(MarkerParams::hmax, MarkerParams::smax, MarkerParams::vmax), led_mask);
  }
#ifdef __SHOWIMG__
  cv::imshow("led_mask", led_mask);
#endif
  return GetLEDMarker(led_mask, res_marker);
}

int MarkerSensor::TrackLEDMarker(Marker &res_marker)
{
  /// Get ROI
  res_marker.ComputeKeyPoints();
  res_marker.ComputeBBox();
  Rect& box = res_marker.bbox;
  float left  = box.x - box.width;
  float right = box.x + box.width*2;
  float top   = box.y - box.height;
  float bot   = box.y + box.height*2;
  left  = left < 0 ? 0 : left;
  right = right >= MarkerParams::camera_width ? MarkerParams::camera_width : right;
  top   = top < 0 ? 0 : top;
  bot   = bot >= MarkerParams::camera_height ? MarkerParams::camera_height : bot;
  Rect ROI(left, top, (right - left), (bot - top));

  /// Get Mask
  cv::Mat ROI_bgr = img_bgr(ROI).clone();
  cv::Mat ROI_img_hsv;
  cv::cvtColor(ROI_bgr, ROI_img_hsv, CV_BGR2HSV);
  cv::Mat ROI_led_mask;
  if (MarkerParams::target_color == 1) {
    cv::inRange(ROI_img_hsv, cv::Scalar(MarkerParams::red_hmin, MarkerParams::smin, MarkerParams::vmin),
                cv::Scalar(MarkerParams::red_hmax, MarkerParams::smax, MarkerParams::vmax), ROI_led_mask);
  } else if (MarkerParams::target_color == 2) {
    cv::inRange(ROI_img_hsv, cv::Scalar(MarkerParams::blue_hmin, MarkerParams::smin, MarkerParams::vmin),
                cv::Scalar(MarkerParams::blue_hmax, MarkerParams::smax, MarkerParams::vmax), ROI_led_mask);
  } else {
    cv::inRange(ROI_img_hsv, cv::Scalar(MarkerParams::hmin, MarkerParams::smin, MarkerParams::vmin),
                cv::Scalar(MarkerParams::hmax, MarkerParams::smax, MarkerParams::vmax), ROI_led_mask);
  }
#ifdef __SHOWIMG__
  cv::imshow("ROI_led_mask", ROI_led_mask);
#endif

  /// Get Marker
  if (GetLEDMarker(ROI_led_mask, res_marker) != STATUS_SUCCESS) {
    printf("Get no marker!\n");
    return -1;
  }

  res_marker.LEDs[0].center.x += ROI.x;
  res_marker.LEDs[0].center.y += ROI.y;
  res_marker.LEDs[1].center.x += ROI.x;
  res_marker.LEDs[1].center.y += ROI.y;

  return 0;
}

int MarkerSensor::ProcessFrameLEDXYZ(const Mat &img, float &X, float &Y, float &Z, int &type)
{
  /// Convert Image
  ConvertImg(img);

  if (status == STATUS_DETECTING) {
    if (DetectLEDMarker(marker) == STATUS_SUCCESS) {
      status = STATUS_TRACKING;
    } else {
      printf("Detect No target!\n");
      return -1;
    }
  } else {
    if (TrackLEDMarker(marker) == STATUS_SUCCESS) {
      printf("Track Success!\n");
    } else {
      status = STATUS_DETECTING;
      printf("Track No target!\n");
      return -1;
    }
  }

#ifdef __SHOWIMG__
  Mat res = img_bgr.clone();
  marker.Draw(res);
  imshow("Marker", res);
#endif

  /// update 3d position
  float marker_width  = (float)cv::norm(marker.LEDs[0].center - marker.LEDs[1].center);
  float marker_height = (marker.LEDs[0].width + marker.LEDs[1].width)*0.5f;
  float focal_length = (MarkerParams::camera_fx + MarkerParams::camera_fy)*0.5f;
  float real_L = MarkerParams::transformer_small_marker_size;
  type = 0;
  if ((marker_width/marker_height) > 4) {
    real_L = MarkerParams::transformer_big_marker_size;
    type = 1;
  }
  depth = (real_L*focal_length) / (marker_width);
  ///使用互补滤波
  if (old_depth > 0) {
    printf("old_depth:%f new_depth:%f", old_depth, depth);
    depth = 0.6f*depth + 0.4f*old_depth;    //TODO:Adjust the params
    old_depth = depth;
    printf(" filtered_depth:%f\n", depth);
  } else {
    old_depth = depth;
  }

  /// update 2d position
  target = (marker.LEDs[0].center + marker.LEDs[1].center)*0.5f;
  Z = depth;
  X = (target.x - MarkerParams::camera_cx)/MarkerParams::camera_fx*Z;
  Y = (target.y - MarkerParams::camera_cy)/MarkerParams::camera_fy*Z;



  printf("Get target: %f %f %f type: %d\n", X, Y, Z, type);
  return 0;

}

int MarkerSensor::ProcessFrameLEDXY(const Mat &img, float &X, float &Y)
{
  /// Convert Image
  ConvertImg(img);

  if (status == STATUS_DETECTING) {
    if (DetectLEDMarker(marker) == STATUS_SUCCESS) {
      status = STATUS_TRACKING;
    } else {
      printf("Detect No target!\n");
      return -1;
    }
  } else {
    if (TrackLEDMarker(marker) == STATUS_SUCCESS) {
      printf("Track Success!\n");
    } else {
      status = STATUS_DETECTING;
      printf("Track No target!\n");
      return -1;
    }
  }

#ifdef __SHOWIMG__
  Mat res = img_bgr.clone();
  marker.Draw(res);
  imshow("Marker", res);
#endif

  /// update 2d position
  target = (marker.LEDs[0].center + marker.LEDs[1].center)*0.5f;
//  ///使用互补滤波
//  if (old_target.x != 0) {
//    printf("old_target:%f,%f new_target:%f,%f", old_target.x, old_target.y, target.x, target.y);
//    target = 0.6f*target + 0.4f*old_target;    //TODO:Adjust the params
//    old_target = target;
//    printf(" filtered_target:%f,%f\n", target.x, target.y);
//  } else {
//    old_target = target;
//  }
  X = target.x - MarkerParams::camera_cx;
  Y = target.y - MarkerParams::camera_cy;

  printf("Get target: %f %f\n", X, Y);
  return 0;

}

int MarkerSensor::ProcessFrameLEDXYz(const Mat &img, float &X, float &Y, float & z)
{
  /// Convert Image
  ConvertImg(img);

  if (status == STATUS_DETECTING) {
    if (DetectLEDMarker(marker) == STATUS_SUCCESS) {
      status = STATUS_TRACKING;
    } else {
      printf("Detect No target!\n");
      return -1;
    }
  } else {
    if (TrackLEDMarker(marker) == STATUS_SUCCESS) {
      printf("Track Success!\n");
    } else {
      status = STATUS_DETECTING;
      printf("Track No target!\n");
      return -1;
    }
  }

#ifdef __SHOWIMG__
  Mat res = img_bgr.clone();
  marker.Draw(res);
  imshow("Marker", res);
#endif

  /// update 2d position
  target = (marker.LEDs[0].center + marker.LEDs[1].center)*0.5f;
//  ///使用互补滤波
//  if (old_target.x != 0) {
//    printf("old_target:%f,%f new_target:%f,%f", old_target.x, old_target.y, target.x, target.y);
//    target = 0.6f*target + 0.4f*old_target;    //TODO:Adjust the params
//    old_target = target;
//    printf(" filtered_target:%f,%f\n", target.x, target.y);
//  } else {
//    old_target = target;
//  }
  X = target.x - MarkerParams::camera_cx;
  Y = target.y - MarkerParams::camera_cy;

  /// update 3d position
  float marker_width  = (float)cv::norm(marker.LEDs[0].center - marker.LEDs[1].center);
  float marker_height = (marker.LEDs[0].width + marker.LEDs[1].width)*0.5f;
  float focal_length = (MarkerParams::camera_fx + MarkerParams::camera_fy)*0.5f;
  float real_L = MarkerParams::transformer_small_marker_size;
  if ((marker_width/marker_height) > 4) {
    real_L = MarkerParams::transformer_big_marker_size;
  }
  depth = (real_L*focal_length) / (marker_width);
  ///使用互补滤波
  if (old_depth > 0) {
    printf("old_depth:%f new_depth:%f", old_depth, depth);
    depth = 0.6f*depth + 0.4f*old_depth;    //TODO:Adjust the params
    old_depth = depth;
    printf(" filtered_depth:%f\n", depth);
  } else {
    old_depth = depth;
  }
  z = depth;

  printf("Get target: %f %f depth: %f\n", X, Y, z);
  return 0;

}

int MarkerSensor::SetTargetRed()
{
  MarkerParams::target_color = 1;
  return 0;
}

int MarkerSensor::SetTargetBlue()
{
  MarkerParams::target_color = 2;
  return 0;
}

