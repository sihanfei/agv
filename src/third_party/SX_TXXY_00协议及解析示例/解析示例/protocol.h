#ifndef _protocol_h
#define _protocol_h
#include <cstdlib>
#include <cstdio>
#include <net/if.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

enum class_enum {NONE_OBJECT, CAR, PES, BICYCLE, ROADBLOCK};

struct ObjectData
{
    u_char id;
    class_enum obj_class = CAR;
    float position[2];
    float velocity;
    float confidence;
    float width;
    uint polygon[4];
};

struct ObjectStr0
{
    uint id:8;
    uint obj_class:8;
    uint confidence:8;
    int position_x:12;
    int position_y:12;
    int velocity:10;
};
struct ObjectStr1
{
    uint id:8;
    uint width:12;
    uint polygon_x_min:12;
    uint polygon_x_max:12;
    uint polygon_y_min:10;
    uint polygon_y_max:10;
};

#endif