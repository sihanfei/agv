#include "power_control.h"


namespace superg_agv
{
namespace drivers
{

Power_Control::Power_Control()
{
    uint8_t add = 0;
    opt.baudrate = 115200;
    opt.wordlength = 1;
    opt.parity = 4;
    opt.stopbit = 0;
    // memcpy(opt_buff,&opt,sizeof(adcuUartOpt));
    // devid = adcuDevOpen(adcuRS485, ADCU_CHANNEL_1);
    // adcuDevSetOpt(devid,opt_buff,sizeof(adcuUartOpt));
    // ROS_INFO("DEVID:%d",devid);
    head[0] = 0x48;
    head[1] = 0x3A;
    tail[0] = 0x45;
    tail[1] = 0x44;
    memset(sendBuffer15,0,sizeof(uint8_t)*16);
    memset(sendBuffer10,0,sizeof(uint8_t)*16);
    memset(readBuffer,0,sizeof(uint8_t)*100);
    memset(control_state,0,sizeof(uint8_t)*20);
//    add = 1;
//    read_control_state(add);
//    add = 2;
//    read_control_state(add);
    if((sockfd = socket(AF_INET, SOCK_STREAM,0)) < 0)
    {
        ROS_ERROR("create socket error");
        // return 0;
    }
    int iFlags;
    iFlags = fcntl(sockfd, F_GETFL, 0);
    iFlags |= O_NONBLOCK;
    iFlags |= O_NDELAY;
    int ret = fcntl(sockfd, F_SETFL, iFlags);

    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(port);
    inet_aton(ip.c_str(), &(servaddr.sin_addr));
    
    if(connect(sockfd,(struct sockaddr*)&servaddr, sizeof(servaddr)) < 0)
    {
        ROS_ERROR("connect error");
        // return 0;
    }
    ROS_INFO("connect scuess");

    ROS_INFO("Power_Control inint");
}

Power_Control::~Power_Control()
{
    adcuDevClose(devid);
}

void Power_Control::splice_protocol_15(uint8_t sendBuffer15[],uint8_t &addr_,uint8_t &cmd_,uint8_t data[])
{
    sendBuffer15[0] = head[0];
    sendBuffer15[1] = head[1];
    sendBuffer15[2] = addr_;
    sendBuffer15[3] = cmd_;
    if(1 == addr_)
    {
        memset(data,0xAA,sizeof(uint8_t) * 8);
    }
    uint8_t len = 12;
    for(uint8_t i = 0;i < 8;++i)
    {
        sendBuffer15[4+i] = data[i];
        // ++data;
    }
    sendBuffer15[12] = calc_CRC(sendBuffer15,len);
    sendBuffer15[13] = tail[0];
    sendBuffer15[14] = tail[1];
}

void Power_Control::splice_protocol_10(uint8_t sendBuffer15[],uint8_t &addr_,uint8_t &cmd_,uint8_t &num,uint8_t &state,uint16_t &time)
{
    sendBuffer15[0] = head[0];
    sendBuffer15[1] = head[1];
    sendBuffer15[2] = addr_;
    sendBuffer15[3] = cmd_;
    sendBuffer15[4] = num;
    sendBuffer15[5] = state;
    sendBuffer15[6] = uint8_t((time & 0xFF00) >> 8);
    sendBuffer15[7] = uint8_t(time & 0xFF);
    sendBuffer15[8] = tail[0];
    sendBuffer15[9] = tail[1];
}

uint8_t Power_Control::calc_CRC(uint8_t msg[],uint8_t &len)
{
    uint8_t CRC = 0;
    for(uint8_t i = 0;i < len;++i)
    {
        CRC += msg[i];
    }
    return uint8_t(CRC);
}

bool Power_Control::send_485(uint8_t msg[],uint8_t &len)
{
    Safety_Status safety_status;
    int sendlen = adcuDevWrite(devid, msg, len);
    if(sendlen < 0)
    {
        safety_status.message_code = "E05120300";
        q_node_malfunction_push(safety_status);
        return false;
    }
    else
    {
        return true;
    }
}

bool Power_Control::send_socket(uint8_t msg[],uint8_t &len)
{
    Safety_Status safety_status;
    int sendlen = send(sockfd, msg, len, 0);
    for(int i = 0;i < sendlen;++i)
    {
        ROS_INFO("sendbuf[%d]:%x",i,msg[i]);
    }
    if(sendlen < 0)
    {
        safety_status.message_code = "E05120300";
        q_node_malfunction_push(safety_status);
        return false;
    }
    else
    {
        return true;
    }
}

int16_t Power_Control::read_485()
{
    int16_t recvlen;
    recvlen = adcuDevRead(devid, readBuffer);
    return recvlen;
}

int16_t Power_Control::read_socket()
{
    int16_t recvlen;
    recvlen = recv(sockfd, readBuffer, 15, 0);
    for(int i = 0;i < recvlen;++i)
    {
        ROS_INFO("recvline[%d]:%x",i,readBuffer[i]);
    }
    return recvlen;
}

void Power_Control::read_control_state(uint8_t &addr_)
{
    uint8_t cmd = 0x53;
    uint8_t data[8] = {0};
    uint8_t len = 15;
    uint8_t recvlen = 0;
    splice_protocol_15(sendBuffer15,addr_,cmd,data);
    mtx_485.lock();
    // if(send_485(sendBuffer15,len) && adcuDevStatus(devid) == ADCU_DEV_STATUS_NORMAL)
    if(send_socket(sendBuffer15,len))
    {
        // recvlen = read_485();
        recvlen = read_socket();
        if(recvlen > 0)
        {
            analysis_control_state(readBuffer,recvlen);
        }

    }
    mtx_485.unlock();
}

void Power_Control::write_control_state(uint8_t &addr_,uint8_t msg[])
{
    uint8_t cmd = 0x57;
    uint8_t data[8] = {0};
    uint8_t len = 15;
    uint8_t recvlen = 0;
    if(0x02 == addr_)
    {
        for(uint8_t i = 0; i < 8 ;++i)
        {
            data[i] = msg[i];
        }
    }
    else if(0x01 == addr_)
    {
        for(uint8_t i = 0; i < 8 ;++i)
        {
            data[i] = msg[i];
        }
    }
    splice_protocol_15(sendBuffer15,addr_,cmd,data);
    mtx_485.lock();
    // if(send_485(sendBuffer15,len) && adcuDevStatus(devid) == ADCU_DEV_STATUS_NORMAL)
    if(send_socket(sendBuffer15,len))
    {
        // recvlen = read_485();
        recvlen = read_socket();
        if(recvlen > 0)
        {
            analysis_control_state(readBuffer,recvlen);
        }
    }
    mtx_485.unlock();
}

void Power_Control::analysis_control_state(uint8_t msg[],uint8_t &len)
{
    if(0x02 == msg[2])
    {
        for(uint8_t i = 0;i < 8;++i)
        {
            control_state[i*2 + 4] = uint8_t(msg[4 + i] & 0x0F);
            control_state[i*2 + 5] = uint8_t((msg[4 + i] & 0xF0) >> 4);
        }
    }
    else if(0x01 == msg[2])
    {
        for(uint8_t i = 0;i < 4;++i)
        {
            control_state[i] = msg[4 + i];
        }
    }
//     for(uint8_t i = 0;i < 20;++i)
//     {
//         ROS_INFO("control_state[%u]:%u",i,control_state[i]);
//     }
}

uint8_t Power_Control::get_control_state(uint8_t index)
{
    return control_state[index];
}

uint8_t Power_Control::get_control_state_size()
{
    return sizeof(control_state)/sizeof(uint8_t);
}

void Power_Control::analysis_ros_cmd(uint8_t data[],uint8_t &len)
{
    uint8_t data_485[8];
    uint8_t addr = 0;
    // if(20 == len && adcuDevStatus(devid) == ADCU_DEV_STATUS_NORMAL)
    if(20 == len)
    {
        for(uint8_t i = 0;i < 4;++i)
        {
            data_485[i] = data[i];
        }
//        addr = 0x01;
//        write_control_state(addr,data_485);
        for(uint8_t i = 0;i < 8;++i)
        {
            data_485[i] = (data[2*i+4] & 0x0F) | ((data[2*i+5] & 0x0F) << 4);
        }
        addr = 0x02;
        write_control_state(addr,data_485);
    }
}

void Power_Control::node_status_pub()
{
    ros::NodeHandle nh;
    ros::Publisher node_status_pub = nh.advertise<status_msgs::NodeStatus>("/node/node_status", 10, true);
    // node_status_info.node_name = ros::this_node::getName();
    // node_status_info.node_pid = getpid();
    ros::Rate rate(10);
    uint8_t add = 0;
    while(ros::ok())
    {
        // string a;
        // if(get_q_node_malfunction_size() != 0)
        // {
        //     a = q_node_malfunction_front();
        // }
        status_msgs::NodeStatus nodestatus;
        status_msgs::SafetyStatus safetystatus_msg;
        common_msgs::KeyValue keyvalue_msg;
        add = 1;
        read_control_state(add);
        add = 2;
        read_control_state(add);
        nodestatus.header.stamp = ros::Time::now();
        nodestatus.header.frame_id = "base_link";
        nodestatus.node_name = get_Node_Status_Info()->node_name;
        nodestatus.node_pid  = get_Node_Status_Info()->node_pid;
        nodestatus.state_num = get_q_node_malfunction_size() + 1;
        safetystatus_msg.message_code = "power_control_state";
        safetystatus_msg.counter = get_Node_Status_Info()->status_counter++;
        safetystatus_msg.value_num = 20;
        for(uint8_t i = 0;i < safetystatus_msg.value_num;++i)
        {
            keyvalue_msg.valuetype = get_control_state(i);
            safetystatus_msg.values.push_back(keyvalue_msg);
        }
        nodestatus.status.push_back(safetystatus_msg);
        if(adcuDevStatus(devid) == ADCU_DEV_STATUS_ABNORMAL)
        {
            // ROS_INFO("E05124000");
            safetystatus_msg.message_code = "E05124000";
            safetystatus_msg.counter = get_Node_Status_Info()->status_counter++;
            safetystatus_msg.value_num = 0;
        }
        nodestatus.status.push_back(safetystatus_msg);
        for(uint16_t i = 0;i <get_q_node_malfunction_size();++i)
        {
            safetystatus_msg.message_code = q_node_malfunction_front().message_code;
            safetystatus_msg.counter = get_Node_Status_Info()->status_counter++;
            
            nodestatus.status.push_back(safetystatus_msg);
        }
        // list<Safety_Status>::iterator ite_SafetyStatus;
        // list<Key_Value>::iterator ite_KeyValue;
        node_status_pub.publish(nodestatus);
        ROS_INFO("node_status_pub.publish(nodestatus),devid:%d",devid);
	    sleep(1);
//        rate.sleep();
    }
}

void Power_Control::power_control_callback(const power_control_msgs::PowerControlCmd::ConstPtr& msg)
{
    uint8_t msg_data[100];
    uint8_t len = msg->data.size();
    for(uint8_t i = 0; i < len;++i)
    {
        msg_data[i] = msg->data[i];
    }
    analysis_ros_cmd(msg_data,len);
    ROS_INFO_STREAM("msg_len:" << msg->length);
    // ROS_INFO("power_control_callback");

//	ROS_INFO_STREAM("length:" << msg->length);
}

void Power_Control::exception_handling()
{
    ROS_INFO("exception_handling is start");
    ros::Rate rate(10);
    while(ros::ok())
    {
        // if(adcuDevStatus(devid) == ADCU_DEV_STATUS_ABNORMAL)
        // {
        //     // adcuDevClose(devid)
        //     devid = adcuDevOpen(adcuRS485, ADCU_CHANNEL_1);
        //     ROS_INFO("try to open adcuRS485,ADCU_CHANNEL_1,devid=%d",devid);
        // }
        rate.sleep();
    }
}

}
}
