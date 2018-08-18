//
// Created by wx on 18-4-28.
//

#ifndef ROBOMASTERRUNEDETECTOR_ROBOMASTERPROCESS_H
#define ROBOMASTERRUNEDETECTOR_ROBOMASTERPROCESS_H

#include "setting.h"
//#include "SmallRuneProcess.h"
//#include "BigRuneProcess.h"
#include "MarkerSensor.h"
#include "can.h"

class RoboMasterProcess{
public:
    RoboMasterProcess(Setting* _setting, int _fd2car){
        setting = _setting;
        fd2car  = _fd2car;
    }

    void Process();

public:
    Setting* setting;
    int fd2car;
};

#endif //ROBOMASTERRUNEDETECTOR_ROBOMASTERPROCESS_H
