//
// Created by liming on 12/26/17.
//

#ifndef HSVDETECT_MARKERPARAMS_H
#define HSVDETECT_MARKERPARAMS_H

#include <stdio.h>
#include <stdlib.h>

#include <iostream>

using namespace std;

class MarkerParams {
public:

    static float camera_width;
    static float camera_height;
    static float camera_fps;
    static float camera_fx;
    static float camera_fy;
    static float camera_cx;
    static float camera_cy;
    static float camera_k1;
    static float camera_k2;
    static float camera_k3;

    static int   blur_sz;
    static float blur_sigma;

    static int   hmin, hmax, smin, smax, vmin, vmax;
    static int   blue_hmin, blue_hmax, red_hmin, red_hmax;
    static int   contours_length_min, contours_length_max;
    static float LED_ratio_min, LED_ratio_max;
    static float LED_width_min, LED_width_max;
    static float marker_parallel_angle;
    static float marker_vertical_angle;
    static float marker_direction_angle;
    static float marker_ratio_min, marker_ratio_max;
    static float marker_size_min, marker_size_max;

    static int   transformer_template_width, transformer_template_height;
    static float transformer_template_score_thres;
    static int   transformer_hmin;
    static int   transformer_hmax;
    static int   transformer_gray_min;
    static int   transformer_gray_max;
    static int   transformer_area_min;
    static int   transformer_area_max;
    static float transformer_c2_s_ratio_min;
    static float transformer_c2_s_ratio_max;
    static float transformer_ellipse_epsi;
    static float transformer_ellipse_inlier_ratio;
    static float transformer_ellipse_radius;
    static float transformer_big_marker_size;
    static float transformer_small_marker_size;

    static int   target_color;
    static int   target_size;

    static float target_bubing_shift_x;
    static float target_bubing_shift_y;
    static float target_bubing_shift_z;
    static float target_bubing_L;

    static float target_shaobing_shift_x;
    static float target_shaobing_shift_y;
    static float target_shaobing_shift_z;
    static float target_shaobing_L;

    static float target_yingxiong_shift_x;
    static float target_yingxiong_shift_y;
    static float target_yingxiong_shift_z;
    static float target_yingxiong_L;

};


#endif //HSVDETECT_MARKERPARAMS_H
