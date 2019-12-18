#ifndef TESTSUB_H
#define TESTSUB_H

#include "spdlog/logger.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/async.h"
#include "spdlog/fmt/bin_to_hex.h"
#include "can_drivers.h"
#include <string>
#include <sstream>

using namespace std;

class logdata_output
{
  public:
    string base_dir_path_;//日志目录
    string sensor_name_;  //传感器名称
    string device_name_;  //设备名称
    uint32_t channel_;    //通道名称
    string logger_name_;  
    string log_file_name; //log文件名
    std::shared_ptr< spdlog::logger > my_logger;//创建logger指针
    bool log_file_created;//log文件已创建标志
    bool log_droped;      //循环创建log文件时使用
    uint32_t g_main_min;


    logdata_output(string dir_path, string sensor_name, string device_name,uint32_t m6_rec_port);
    ~logdata_output();

    bool create_log_file(string dir_path);
    void destroy_logger();

    void write_log(char *buf, int len);
    int my_mkdir(string muldir,mode_t mode);
    bool MinHasChanged();   
};

#endif
