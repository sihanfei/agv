#include "logdata.h"

// #define LOG_MANAGER_YAML_FILE_PATH "/work/superg_agv/src/drivers/can_wr/config"
// #define LOG_MANAGER_YAML_FILE_NAME "/log_manager.yaml"

logdata_output::logdata_output(string sensor_name, string device_name,uint32_t m6_rec_port)
                    // : base_dir_path_(dir_path),sensor_name_(sensor_name),device_name_(device_name),channel_(m6_rec_port)
                    : sensor_name_(sensor_name),device_name_(device_name),channel_(m6_rec_port)
{
    
    base_dir_path_   = getenv("HOME");
    // yaml_path_       = base_dir_path_ + LOG_MANAGER_YAML_FILE_PATH + LOG_MANAGER_YAML_FILE_NAME;
    // YAML::Node yamlConfig = YAML::LoadFile(yaml_path_);
    // work_space       = yamlConfig["work_space"].as<string>();
    // log_dir          = yamlConfig["log_dir"].as<string>();
    // base_dir_path_  +=  work_space + log_dir;
    base_dir_path_  +=  "/work/log";
    log_file_created = false;
    log_droped       = false;
    g_main_min       = 0;

    logger_name_ = sensor_name_ + "-" + device_name_ + "-" + std::to_string(channel_);

    cout << logger_name_ << endl;

    SPDLOG_DEBUG(logger_name_);
}

logdata_output::~logdata_output()
{
    destroy_logger();
}

int logdata_output::my_mkdir(string muldir,mode_t mode)
{
    vector< string > v_str;
    SplitString(muldir,v_str,"/");
    
    SPDLOG_DEBUG("v_str.size():{}",v_str.size());
    
    int iRet;
    
    ostringstream temp_dir;
    temp_dir.fill('0');
    for(vector< string >::size_type i = 0;i != v_str.size();++i)
    {
        if(v_str[i] == "")
        {
            continue;
        }
        temp_dir << '/' << v_str[i];
        SPDLOG_DEBUG("temp_dir:{}",temp_dir.str().c_str());
        if(access(temp_dir.str().c_str(),0) != 0)
        {
            iRet = mkdir(temp_dir.str().c_str(),mode);
            if(iRet < 0)
            {
                return iRet;
            }
        }
    }
    return 0;
}

void logdata_output::SplitString(const string &s,vector< string > &v_str,const string &c)
{
    string::size_type pos1,pos2;
    pos2 = s.find(c);
    pos1 = 0;
    string str;
    while(string::npos != pos2)
    {
        str = s.substr(pos1,pos2 - pos1);
        if(str != "")
        {
            v_str.push_back(str);
        }
        pos1 = pos2 + c.size();
        pos2 = s.find(c,pos1);
    }
    if(pos1 != s.length())
    {
        str = s.substr(pos1);
        if(str != "")
        {
            v_str.push_back(str);
        }
    }
}

bool logdata_output::MinHasChanged()
{
    time_t raw_time;
    struct tm *tm_info;
    time(&raw_time);
    tm_info = localtime(&raw_time);
    
    if(tm_info->tm_min != g_main_min)
    {
        g_main_min = tm_info->tm_min;
        return true;
    }
    return false;
}

bool logdata_output::create_log_file(string dir_path)
{
    stringstream log_full_path;
    log_full_path << dir_path << log_file_name;

    if(access(dir_path.c_str(),F_OK) != 0)//log目录不存在
    {
        cout << "access != 0" << endl;
        if(my_mkdir(dir_path,0777) < 0)
        {
            cout << "my_mkdir < 0" << endl;
            SPDLOG_DEBUG("mkdir={} msg={}",dir_path.data(),strerror(errno));
        }
    }
    if(access(dir_path.c_str(),F_OK) != 0)
    {
        SPDLOG_DEBUG("dir_path:{} not find",dir_path.c_str());
        return false;
    }
    else
    {
        SPDLOG_DEBUG("dir_path:{}",dir_path.c_str());
        SPDLOG_DEBUG("log_full_path:{}",log_full_path.str().c_str());
    }

    cout << "log_full_path:" << log_full_path.str() << endl;

    my_logger = spdlog::basic_logger_mt< spdlog::async_factory >(logger_name_, log_full_path.str());

    my_logger->set_pattern("%Y-%m-%d-%H:%M:%S.%e,%n,%v");
    
    spdlog::flush_every(std::chrono::seconds(30));

    // my_logger = spdlog::get(logger_name_);

    return true;
}

void logdata_output::destroy_logger()
{
    SPDLOG_DEBUG("dextroy_logger");
    spdlog::drop(logger_name_);
}

void logdata_output::write_log(uint8_t *buf, uint8_t id,int len)
{
    // ROS_INFO("=================================================");
    stringstream p2data;
    // cout << "print data:" << endl;
    // ROS_INFO("print data:");
    // cout << buff++ << endl;
    // cout << buff++ << endl;
    // cout << buff++ << endl;
    if((MinHasChanged() || !log_file_created) && checkTheDiskOnce())
    {
        if(log_droped)
        {
            destroy_logger();
        }
        time_t raw_time;
        struct tm * tm_info;
        
        time(&raw_time);
        tm_info = localtime(&raw_time);
        
        ostringstream time_pid_stream;
        time_pid_stream.fill(0);
        // time_pid_stream << device_name_ << '-' << channel_ << '-' << 1900 + tm_info->tm_year
        //                 <<  setw(2) << setfill('0') << 1 + tm_info->tm_mon << setw(2) << setfill('0')
        //                 << tm_info->tm_mday << '-'<< setw(2) << setfill('0') << tm_info->tm_hour 
        //                 << setw(2) << setfill('0') << tm_info->tm_min  << ".csv";
        time_pid_stream << sensor_name_ << '-' << device_name_ << '-' << channel_ << '-'
                        << tm_info->tm_min  << ".csv";

        cout << time_pid_stream.str() << endl;

        ostringstream dir_path_stream;
        dir_path_stream.fill(0);
        dir_path_stream << base_dir_path_ << '/' << sensor_name_ << '/' << 1900 + tm_info->tm_year
                        << setw(2) << setfill('0') << 1 + tm_info->tm_mon << setw(2) << setfill('0')
                        << tm_info->tm_mday << '-'<< setw(2) << setfill('0') << tm_info->tm_hour << '/';
                    
        log_file_name    = time_pid_stream.str();
        cout << "log_file_name:" << log_file_name << endl;
        log_droped       = create_log_file(dir_path_stream.str());
        log_file_created = true;
    }
    if(log_droped)
    {
        // std::vector< unsigned char > uvchar;
        std::vector< uint8_t > uvchar;
        uvchar.push_back(id);
        for(size_t i = 0;i < len;++i)
        {
            uvchar.push_back(buf[i]);
        }
            
        // my_logger = spdlog::get(logger_name_);
        // ROS_INFO_STREAM("data:" << p2data.str());
        my_logger -> info("{:Xspn}",spdlog::to_hex(uvchar));
        // for(size_t i = 0;i < len;++i)
        // {
        //     p2data << buff[i];
        // }
        // p2data << to_string(buff[0]) << "," << to_string(buff[1]);
        // ROS_INFO_STREAM("data:" << p2data.str());
        // my_logger -> info("{:Xspn}",spdlog::to_hex(p2data.str()));
    }
}
