
#include "setting.h"
#include <thread>
#include "RoboMasterProcess.h"
#include "RemoteControl.h"
#include "can.h"


using namespace std;
using namespace cv;

int main(int argc, char*argv[]) {

    Setting setting;

    /// can init
    char portname[20] = "can0";
    int fd2car = CanOpen(portname);

    //process frame and send angle to car
    RoboMasterProcess roboProcess(&setting, fd2car);
    std::thread t1(&RoboMasterProcess::Process, &roboProcess);

    //receive car command
    RemoteControl remoteProcess(&setting, fd2car);
    std::thread t2(&RemoteControl::Receiver, remoteProcess);


    t1.join();
    t2.join();

    std::cout << "Hello, World!" << std::endl;
    close(fd2car);
    return 0;
}
