//
// Created by liming on 12/26/17.
//

#include "MarkerParams.h"

/// Camera
float MarkerParams::camera_width = 640;
float MarkerParams::camera_height = 480;
float MarkerParams::camera_fps = 30;
float MarkerParams::camera_fx = 500;
float MarkerParams::camera_fy = 500;
float MarkerParams::camera_cx = 320;
float MarkerParams::camera_cy = 240;
float MarkerParams::camera_k1 = 0;
float MarkerParams::camera_k2 = 0;
float MarkerParams::camera_k3 = 0;

/// ImageFrame
int MarkerParams::blur_sz = 5;
float MarkerParams::blur_sigma = 0.4;

/// Detect
int MarkerParams::hmin = 90;
int MarkerParams::hmax = 110;
int MarkerParams::blue_hmin = 90;
int MarkerParams::blue_hmax = 110;
int MarkerParams::red_hmin = 90;
int MarkerParams::red_hmax = 110;
int MarkerParams::smin = 120;
int MarkerParams::smax = 255;
int MarkerParams::vmin = 140;
int MarkerParams::vmax = 255;
int MarkerParams::contours_length_min = 10;
int MarkerParams::contours_length_max = 300;
float MarkerParams::LED_ratio_min = 2;
float MarkerParams::LED_ratio_max = 20;
float MarkerParams::LED_width_min = 7;
float MarkerParams::LED_width_max = 100;
float MarkerParams::marker_parallel_angle = 6;
float MarkerParams::marker_vertical_angle = 10;
float MarkerParams::marker_direction_angle = 45;
float MarkerParams::marker_ratio_min = 0.8;
float MarkerParams::marker_ratio_max = 5;
float MarkerParams::marker_size_min = 10;
float MarkerParams::marker_size_max = 200;

/// Transform
int MarkerParams::transformer_template_width = 60;
int MarkerParams::transformer_template_height = 30;
float MarkerParams::transformer_template_score_thres = 2000;
int MarkerParams::transformer_hmin = 80;
int MarkerParams::transformer_hmax = 120;
int MarkerParams::transformer_gray_min = 100;
int MarkerParams::transformer_gray_max = 255;
int MarkerParams::transformer_area_min = 64;
int MarkerParams::transformer_area_max = 40000;
float MarkerParams::transformer_c2_s_ratio_min = 8;
float MarkerParams::transformer_c2_s_ratio_max = 20;
float MarkerParams::transformer_ellipse_epsi = 0.2;
float MarkerParams::transformer_ellipse_inlier_ratio = 0.9;
float MarkerParams::transformer_ellipse_radius = 0.035;
float MarkerParams::transformer_big_marker_size = 0.225;
float MarkerParams::transformer_small_marker_size = 0.12;

/// target
int MarkerParams::target_color = 1;
int MarkerParams::target_size = 1;

float MarkerParams::target_bubing_shift_x = 0.125;
float MarkerParams::target_bubing_shift_y = 0.198;
float MarkerParams::target_bubing_shift_z = 0.16;
float MarkerParams::target_bubing_L = 0.16;

float MarkerParams::target_shaobing_shift_x = 0.125;
float MarkerParams::target_shaobing_shift_y = 0.198;
float MarkerParams::target_shaobing_shift_z = 0.16;
float MarkerParams::target_shaobing_L = 0.16;

float MarkerParams::target_yingxiong_shift_x = 0.125;
float MarkerParams::target_yingxiong_shift_y = 0.198;
float MarkerParams::target_yingxiong_shift_z = 0.16;
float MarkerParams::target_yingxiong_L = 0.16;
