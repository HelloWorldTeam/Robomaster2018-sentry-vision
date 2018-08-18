//
// Created by liming on 4/28/18.
//

#include "can.h"

/*
 * can_socket.c
 *
 * Forked on: 2018-4-24
 * Author: Bill Lin
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

int CanOpen(const char* port_name)
{
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;
    /* create socket */
    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
        perror("Create socket failed");
        exit(-1);
    }
    /* set up can interface */
    strcpy(ifr.ifr_name, port_name);
    printf("can port is %s\n",ifr.ifr_name);
    /* assign can device */
    ioctl(s, SIOCGIFINDEX, &ifr);
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    /* bind can device */
    if(bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("Bind can device failed\n");
        close(s);
        //exit(-2); //如果不注释，PC上无法运行
    }
    /* set filter for only receiving packet with can id 0x1F */
    struct can_filter rfilter[1];
    rfilter[0].can_id = 0x3FF;
    rfilter[0].can_mask = CAN_SFF_MASK;
    if(setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter)) < 0)
    {
      perror("set receiving filter error\n");
      close(s);
      //exit(-3); //如果不注释，PC上无法运行
    }

    return s;
}

int CanSend(int port_handle, double yaw, double pitch)
{
    /* configure can_id and can data length */
    static struct can_frame frame;
    frame.can_id = 0x2FF;
    frame.can_dlc = 8;
    /* prepare data for sending: 0xFF,...,0xFE */
    short y100 = yaw*100;
    short p100 = pitch*100;
    frame.data[0] = (unsigned char)(p100 >> 8);
    frame.data[1] = (unsigned char)(p100);
    frame.data[2] = (unsigned char)(y100 >> 8);
    frame.data[3] = (unsigned char)(y100);
    frame.data[4] = frame.data[5] = frame.data[6] = frame.data[7] = 0;
    printf("Can send: %d ID=%#x data_length=%d data=", port_handle, frame.can_id, frame.can_dlc);
    for (int i = 0; i < frame.can_dlc; ++i)
        printf("%#x ", frame.data[i]);
    printf("\n");
    /* Sending data */
    int nbytes = write(port_handle, &frame, sizeof(frame));
    if( nbytes < 0)
    {
        perror("Send failed");
    }
    return nbytes;
}

int CanSend(int port_handle, double X, double Y, double Z)
{
    /* configure can_id and can data length */
    static struct can_frame frame;
    frame.can_id = 0x2FF;
    frame.can_dlc = 8;
    /* prepare data for sending: 0xFF,...,0xFE */
    short x100 = X*1000;
    short y100 = Y*1000;
    short z100 = Z*1000;
    frame.data[0] = (unsigned char)(x100 >> 8);
    frame.data[1] = (unsigned char)(x100);
    frame.data[2] = (unsigned char)(y100 >> 8);
    frame.data[3] = (unsigned char)(y100);
    frame.data[4] = (unsigned char)(z100 >> 8);
    frame.data[5] = (unsigned char)(z100);
    frame.data[6] = 0;
    frame.data[7] = 0;
    printf("Can send: %d ID=%#x data_length=%d data=", port_handle, frame.can_id, frame.can_dlc);
    for (int i = 0; i < frame.can_dlc; ++i)
        printf("%#x ", frame.data[i]);
    printf("\n");
    /* Sending data */
    int nbytes = write(port_handle, &frame, sizeof(frame));
    if( nbytes < 0) {
        perror("Send failed");
    }
    return nbytes;
}

int CanSend(int port_handle, double X, double Y, double Z, int type)
{
    /* configure can_id and can data length */
    static struct can_frame frame;
    frame.can_id = 0x2FF;
    frame.can_dlc = 8;
    /* prepare data for sending: 0xFF,...,0xFE */
    short x100 = X*1000;
    short y100 = Y*1000;
    short z100 = Z*1000;
    frame.data[0] = (unsigned char)(x100 >> 8);
    frame.data[1] = (unsigned char)(x100);
    frame.data[2] = (unsigned char)(y100 >> 8);
    frame.data[3] = (unsigned char)(y100);
    frame.data[4] = (unsigned char)(z100 >> 8);
    frame.data[5] = (unsigned char)(z100);
    frame.data[6] = type;
    frame.data[7] = 0;
    printf("Can send: %d ID=%#x data_length=%d data=", port_handle, frame.can_id, frame.can_dlc);
    for (int i = 0; i < frame.can_dlc; ++i)
        printf("%#x ", frame.data[i]);
    printf("\n");
    /* Sending data */
    int nbytes = write(port_handle, &frame, sizeof(frame));
    if( nbytes < 0) {
        perror("Send failed");
    }
    return nbytes;
}

int CanSendError(int port_handle)
{
  /* configure can_id and can data length */
    static struct can_frame frame;
    frame.can_id = 0x2FF;
    frame.can_dlc = 8;
    /* prepare data for sending: 0xFF,...,0xFE */
    frame.data[0] = 0;
    frame.data[1] = 0;
    frame.data[2] = 0;
    frame.data[3] = 0;
    frame.data[4] = 0;
    frame.data[5] = 0;
    frame.data[6] = 0;
    frame.data[7] = 1;
    printf("Can send: %d ID=%#x data_length=%d data=", port_handle, frame.can_id, frame.can_dlc);
    for (int i = 0; i < frame.can_dlc; ++i)
        printf("%#x ", frame.data[i]);
    printf("\n");
    /* Sending data */
    int nbytes = write(port_handle, &frame, sizeof(frame));
    if( nbytes < 0) {
        perror("Send failed");
    }
    return nbytes;
}

int CanReceive(int port_handle, char * buffer)
{
    static struct can_frame frame;
    int nbytes = read(port_handle, &frame, sizeof(frame));
    nbytes = frame.can_dlc;
    if (nbytes != 8) {
      printf("nbytes %d != 8\n", nbytes);
      for (int i = 0; i < frame.can_dlc; i++)
        printf("%#x ", frame.data[i]);
      printf("\n");
      return 0;
    }
    if (frame.can_id == 0x3FF) {  // 手动过滤一下
        printf("Can receive: %d ID=%#x data_length=%d data=", port_handle, frame.can_id, frame.can_dlc);
        for (int i = 0; i < frame.can_dlc; i++)
          printf("%#x ", frame.data[i]);
        printf("\n");
        memcpy(buffer, frame.data, nbytes);
    } else {
      nbytes = 0;
    }

    return nbytes;
}
