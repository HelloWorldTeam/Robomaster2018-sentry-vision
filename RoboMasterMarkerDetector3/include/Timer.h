//
// Created by liming on 1/2/18.
//

#ifndef MARKERSENSOR_TIMER_H
#define MARKERSENSOR_TIMER_H

#include <sys/time.h>

#define TIME_BEGIN() {timeval t_b, t_e; gettimeofday(&t_b, 0);
#define TIME_END(TAG) gettimeofday(&t_e, 0); printf("=== %s time: %lf s\n", TAG, (t_e.tv_sec - t_b.tv_sec) + (t_e.tv_usec - t_b.tv_usec)*1e-6); }

#endif //MARKERSENSOR_TIMER_H
