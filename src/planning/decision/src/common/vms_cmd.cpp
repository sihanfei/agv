
#include "utils.h"


namespace pnc
{


VMS_Cmd::VMS_Cmd(std::string id, uint8_t type,double x,double y,double heading)
    :id_(id),type_(type),x_(x),y_(y),heading_(heading)
{
}

// 给私有变量赋值
void VMS_Cmd::setID(std::string id)
{
    id_ = id;
}

void VMS_Cmd::setType(uint8_t type)
{
    type_ = type;
}

void VMS_Cmd::setX(double x)
{
    x_ = x;
}

void VMS_Cmd::setY(double y)
{
    y_ = y;
}

void VMS_Cmd::setHeading(double heading)
{
    heading_ = heading;
}


// 获取私有变量的值
std::string VMS_Cmd::getID()
{
    return id_;
}

uint8_t VMS_Cmd::getType()
{
    return type_;
}

double VMS_Cmd::getX()
{
    return x_;
}

double VMS_Cmd::getY()
{
    return y_;
}

double VMS_Cmd::getHeading()
{
    return heading_;
}




}
