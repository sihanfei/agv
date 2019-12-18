#ifndef DRIVERS_CAN_WR_INCLUDE_LOGMANAGER_H_
#define DRIVERS_CAN_WR_INCLUDE_LOGMANAGER_H_

#include <string>
#include "ros/ros.h"
#include <unistd.h>
#include <sys/stat.h>
#include <time.h>
#include <fstream>
#include <thread>
#include <functional>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <yaml-cpp/yaml.h>
#include <dirent.h>
#include<sys/types.h>

using namespace std;

struct CPU_OCCUPY  //定义一个cpu occupy的结构体
{
  char name[20];        //定义一个char类型的数组名name有20个元素
  unsigned int user;    //定义一个无符号的int类型的user
  unsigned int nice;    //定义一个无符号的int类型的nice
  unsigned int system;  //定义一个无符号的int类型的system
  unsigned int idle;    //定义一个无符号的int类型的idle
};

class CheckDisk
{
    private:
        int occupy_point;
        static bool MountDisk;
        std::mutex mtx_MountDisk;
        std::mutex mtx_OccupyPoint;
    public:
        CheckDisk();
        ~CheckDisk();
        int getOccupyPoint();
        void setOccupyPoint(int occupy_point_);
        bool isMountDisk();
        void setMountDisk(bool MD);
        bool checkTheDiskOnce();
        void checkTheDisk();//检查硬盘是否挂载
        void SplitString(const string &s,vector< string > &v_str,const string &c);
};

class logmanage : public CheckDisk
{
    
    private:
        string dir;
        list<string> cantype;
        time_t logtime;
        struct tm *ptminfo;
        time_t creattime;
        struct tm *pcreattime;
        // bool MountDisk;
        // std::mutex mtx_MountDisk;

        string yaml_path;
        string log_dir;
        string base_path;
        string work_space;

        ofstream logfile;
    public:
        std::mutex mtx_qtimepath;
        std::mutex mtx_package;
        std::condition_variable cond_package;
        std::queue<string> qtimepath;
        logmanage();
        ~logmanage();
        void push_qtimepath(string str);
        string front_qtimepath();
        bool qtimepath_empty();
        void history_dir_package();
        void dirInit();//初始化建立文件和文件夹
        bool dirIsexistence(string dirpath);//查看指定路径文件夹是否存在
        bool fileIsexistence(string pram1, string pram2);//查看指定路径下指定文件是否存在
        bool Createfile(string pram1, string pram2);//在指定路径下创建指定文件
        void Createdir(string dirpath);//创建指定路径
        bool package(string dir,string filename);//打包指定路径下的指定路径
        bool remove(string dir,string filename);
        void time2Createfile();//检测时间是否到创建文件时间
        void time2Package();//检测时间是否到打包时间
        // bool isMountDisk();
        // void setMountDisk(bool MD);
        // void checkTheDisk();//检查硬盘是否挂载
        // void SplitString(const string &s,vector< string > &v_str,const string &c);
        int cal_cpuoccupy(CPU_OCCUPY *o, CPU_OCCUPY *n);
        void get_cpuoccupy(CPU_OCCUPY *cpust);
    protected:

};

#endif
