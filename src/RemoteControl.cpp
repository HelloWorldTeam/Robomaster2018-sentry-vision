//
// Created by wx on 18-4-28.
//

#include "RemoteControl.h"
#include <iostream>

void RemoteControl::Receiver() {
  char buffer[8];
  while(1){
    ///receive
    int num_bytes = CanReceive(fd2car, buffer);

    if (num_bytes == 8) {
      char cmd1 = buffer[0];
      char cmd2 = buffer[1];
      printf("receive cmd %#x %#x\n", cmd1, cmd2);

      switch (cmd1) {
        case 0: {
          if (cmd2 == 0) {
            setting->mode = 2;
          } else if (cmd2 == 1) {
            setting->mode = 1;
          }
          break;
        }
        case 1: {
          setting->mode = 4;
          break;
        }
        case 2: {
          setting->mode = 3;
          break;
        }
        case 3: {
          setting->mode = 0;
          break;
        }
        default:
          break;
          //setting->mode = 0; // stop
      }
    }
    usleep(20000);
  }
}
