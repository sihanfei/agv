#ifndef LOGDATA_H
#define LOGDATA_H

#include "device.h"

#include "spdlog/logger.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/async.h"
#include "spdlog/fmt/bin_to_hex.h"
#include "can_drivers.h"
#include <string>
#include <sstream>
#include "logmanager.h"
// #include <yaml-cpp/yaml.h>

using namespace std;

class logdata_output : public CheckDisk
{
  public:
//    string base_dir_path_ = "/home/higo/work/log";//日志目录
    string log_dir;       //日志文件夹
    string work_space;    //工作空间名

    string yaml_path_;    //配置文件路径
    string base_dir_path_;//家目录路径
    string sensor_name_;  //传感器名称
    string device_name_;  //设备名称
    uint32_t channel_;    //通道名称
    string logger_name_;  
    string log_file_name; //log文件名
    std::shared_ptr< spdlog::logger > my_logger;//创建logger指针
    bool log_file_created;//log文件已创建标志
    bool log_droped;      //循环创建log文件时使用
    uint32_t g_main_min;


    // logdata_output(string dir_path, string sensor_name, string device_name,uint32_t m6_rec_port);
    logdata_output(string sensor_name, string device_name,uint32_t m6_rec_port);
    ~logdata_output();

    bool create_log_file(string dir_path);
    void destroy_logger();

    void SplitString(const string &s,vector< string > &v_str,const string &c);

    void write_log(uint8_t *buf, uint8_t id,int len);
    int my_mkdir(string muldir,mode_t mode);
    bool MinHasChanged();   
};

#endif
