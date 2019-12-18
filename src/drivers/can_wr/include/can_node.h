#ifndef _CAN_NODE_H
#define _CAN_NODE_H

#include "ros/ros.h"
#include "p2.h"
#include "camera.h"
#include "ultrasonic.h"
#include "mobileye.h"
#include "node_status.h"

namespace superg_agv
{
namespace drivers
{

class CAN_Node:public P2,public CAMERA,public ULTRASONIC,public MOBILEYE,public node_status
{
    public:
        CAN_Node()
        {};
        ~CAN_Node()
        {};
        // void CAN_Node_Say_Hello();
};

}
}

#endif
