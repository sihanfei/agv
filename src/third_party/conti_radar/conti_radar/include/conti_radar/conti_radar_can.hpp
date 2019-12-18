#ifndef CONTI_RADAR_CAN_H_
#define CONTI_RADAR_CAN_H_
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include "stdint.h"
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include "cluster_list_status_600.h"
#include "object_list_status_60a.h"
#include "object_quality_info_60c.h"
#include "object_general_info_60b.h"
#include "object_extended_info_60d.h"

#include <boost/function.hpp>
#include <boost/bind.hpp>

using namespace std;

class ContiRadarCan
{
public:
    
    typedef boost::function<int (const ContiRadar& radarmsg)> ContiRadarCb;

    int s, nbytes;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_filter rfilter[1];
    string can_channel;
    bool is_conti_radar_configed;


public:
    ContiRadarCan();
    ~ContiRadarCan();
    void conti_radar_can_init(string can_serial);
    void conti_radar_can_close();
    void conti_radar_can_Recv();
    void conti_radar_can_send();
    void conti_can_bus_info_to_sensor();

    void setContiRadarRecvCallback(ContiRadarCb cb);

private :
    ContiRadar conti_radar_;
    ContiRadar conti_radar_info;
    ContiRadarCb contiRadarCb_;

};
#endif
