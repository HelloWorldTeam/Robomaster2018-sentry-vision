//
// Created by wx on 18-4-28.
//

#include "setting.h"
#include "can.h"

#ifndef ROBOMASTERRUNEDETECTOR_REMOTECONTROL_H
#define ROBOMASTERRUNEDETECTOR_REMOTECONTROL_H
class RemoteControl{
public:
    RemoteControl(Setting* _setting, int _fd2car){
        setting = _setting;
        fd2car = _fd2car;
    }
public:
    Setting* setting;
    int fd2car;
public:
    void Receiver();
};




#endif //ROBOMASTERRUNEDETECTOR_REMOTECONTROL_H
