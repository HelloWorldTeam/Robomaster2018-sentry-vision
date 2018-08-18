//
// Created by liming on 4/28/18.
//

#ifndef MARKERSENSOR_CAN_H_H
#define MARKERSENSOR_CAN_H_H

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

int CanOpen(const char* port_name);

int CanSend(int port_handle, double yaw, double pitch);

int CanSend(int port_handle, double X, double Y, double Z);

int CanSend(int port_handle, double X, double Y, double Z, int type);

int CanSendError(int port_handle);

int CanReceive(int port_handle, char *buffer);

#endif //MARKERSENSOR_CAN_H_H
