#include "logmanager.h"

using namespace std;

#define LOG_MANAGER_YAML_FILE_PATH "/work/superg_agv/src/drivers/can_wr/config"
#define LOG_MANAGER_YAML_FILE_NAME "/log_manager.yaml"

bool CheckDisk::MountDisk = false;
// bool CheckDisk::MountDisk = true;

CheckDisk::CheckDisk()
{
    occupy_point = 0;
    // MountDisk = false;
}

CheckDisk::~CheckDisk()
{
    
}

bool CheckDisk::isMountDisk()
{
    mtx_MountDisk.lock();
    bool MountDisk_ = MountDisk;
    mtx_MountDisk.unlock();
    return MountDisk_;
}

void CheckDisk::setMountDisk(bool MD)
{
    mtx_MountDisk.lock();
    MountDisk = MD;
    mtx_MountDisk.unlock();
}

int CheckDisk::getOccupyPoint()
{
    mtx_OccupyPoint.lock();
    int occupy_point_ = occupy_point;
    mtx_OccupyPoint.unlock();
    return occupy_point_;
}

void CheckDisk::setOccupyPoint(int occupy_point_)
{
    mtx_OccupyPoint.lock();
    occupy_point = occupy_point_;
    mtx_OccupyPoint.unlock();
}

bool CheckDisk::checkTheDiskOnce()
{
    FILE* fp = NULL;
    char buf[1024] = {0};
    string strResult = "";
    vector<string> v_strResult;
    fp = popen("df /home/higo/work/log", "r");
    if(NULL != fp)
    {
        while(fgets(buf, sizeof buf, fp))	
        {		
            strResult += buf;	
        }
        SplitString(strResult,v_strResult," ");
        if(2 <= (v_strResult.size()+1)/6)
        {
                // pclose(fp);
                return true;
        }
        else
        {
                return false;
        }
    }
    else
    {
        return false;
    }
}

void CheckDisk::checkTheDisk()
{
    while(ros::ok())
    {
        FILE* fp = NULL;
        char buf[1024] = {0};
        int i_Occupy_point = 0;
        stringstream temp_Occupy_point;
        string strResult = "";
        vector<string> v_strResult;
        vector<string> v_Disk_name;
        vector<string> v_Occupy_point;
        fp = popen("df /home/higo/work/log", "r");
        if(NULL != fp)
        {
            while(fgets(buf, sizeof buf, fp))
            {
                strResult += buf;
            }
            ROS_INFO_STREAM(strResult);
            SplitString(strResult,v_strResult," ");
            if(2 <= (v_strResult.size()+1)/6)
            {
                SplitString(v_strResult[6],v_Disk_name,"\n");
                SplitString(v_strResult[v_strResult.size() - 2],v_Occupy_point,"%");
                temp_Occupy_point << v_Occupy_point[0];
                temp_Occupy_point >> i_Occupy_point;
                // ROS_INFO("Occupy_point:%d",i_Occupy_point);
                setOccupyPoint(i_Occupy_point);
            }
//          for(vector<string>::iterator ite_v_strResult = v_strResult.begin();ite_v_strResult != v_strResult.end();++ite_v_strResult)
//          {
//              ROS_INFO_STREAM(*ite_v_strResult);
//          }
//          ROS_INFO_STREAM(v_strResult[v_strResult.size() - 1]);
            if(v_Disk_name.size() >= 2)
            {
                if(!v_Disk_name[1].compare("/dev/sda1"))
                {
                        // ROS_INFO("true");
                        setMountDisk(true);
                }
                else
                {
                        // ROS_INFO("false");
                        setMountDisk(false);
                }
            }
        }
        pclose(fp);
        sleep(1);
    }
}

void CheckDisk::SplitString(const string &s,vector< string > &v_str,const string &c)
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

logmanage::logmanage()
{
    base_path = getenv("HOME");
    dir = base_path;
    yaml_path  = base_path + LOG_MANAGER_YAML_FILE_PATH + LOG_MANAGER_YAML_FILE_NAME;
    YAML::Node yamlConfig = YAML::LoadFile(yaml_path);
    // dir += "/work";
    work_space = yamlConfig["work_space"].as<string>();
    dir += work_space;
    log_dir = yamlConfig["log_dir"].as<string>();
    for(unsigned i = 0;i < yamlConfig["sensor_type_list"].size();++i)
    {
        cantype.push_back(yamlConfig["sensor_type_list"][i]["sensor_name"].as<string>());
    }
    history_dir_package();
    dirInit();
}

logmanage::~logmanage()
{

}

void logmanage::history_dir_package()
{
    DIR *dir_;
    struct dirent *ptr;
    string basepath = dir + log_dir;
    string package = "";
    // ROS_INFO_STREAM("basepath:" << basepath);
    for(list<string>::iterator ite_cantype = cantype.begin();ite_cantype != cantype.end();++ite_cantype)
    {
        basepath = dir + log_dir + "/" + *ite_cantype;
        ROS_INFO_STREAM(basepath);
        if((dir_=opendir(basepath.c_str())) == NULL)
        {
            ROS_INFO("Open dir error~!");
        }
        else
        {
            ROS_INFO("OPen dir succese");
            struct stat s ;
            while ((ptr=readdir(dir_)) != NULL)
            {
                if(strcmp(ptr->d_name,".")==0 || strcmp(ptr->d_name,"..")==0)
                    continue;
                else if(ptr->d_type == 4)
                {
                    package = basepath + "/" + ptr->d_name;
                    // ROS_INFO_STREAM(basepath << "," << package);
                    // package(package,"");
                    push_qtimepath(package);
                }
            }
            ROS_INFO("readdir over");
        }
        // struct stat s ;
        // while ((ptr=readdir(dir_)) != NULL)
        // {
        //     if(strcmp(ptr->d_name,".")==0 || strcmp(ptr->d_name,"..")==0)
        //         continue;
        //     else if(ptr->d_type == 4)
        //     {
        //         package = basepath + "/" + ptr->d_name;
        //         // ROS_INFO_STREAM(basepath << "," << package);
        //         // package(package,"");
        //         push_qtimepath(package);
        //     }
        // }
        // ROS_INFO("readdir over");
        closedir(dir_);
    }
}

void logmanage::push_qtimepath(string str)
{
    mtx_qtimepath.lock();
    qtimepath.push(str);
    mtx_qtimepath.unlock();
}

string logmanage::front_qtimepath()
{
    string return_str;
    mtx_qtimepath.lock();
    return_str = qtimepath.front();
    qtimepath.pop();
    mtx_qtimepath.unlock();
    return return_str;
}

bool logmanage::qtimepath_empty()
{
    bool return_bool;
    mtx_qtimepath.lock();
    return_bool = qtimepath.empty();
    mtx_qtimepath.unlock();
    return return_bool;
}

void logmanage::Createdir(string dirpath)
{
    int flag = mkdir(dirpath.c_str(),S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if(0 == flag)
    {
        ROS_INFO("make dir:%s successfully",dirpath.c_str());
    }
    else
    {
        ROS_INFO("make dir:%s filed",dirpath.c_str());
    }
}

void logmanage::dirInit()
{
    time(&logtime);
    ptminfo = localtime(&logtime);
    ostringstream date1;
    ostringstream date2;
    date1 << to_string(ptminfo->tm_year + 1900) << setw(2) << setfill('0') << to_string(ptminfo->tm_mon + 1)
          << setw(2) << setfill('0') << to_string(ptminfo->tm_mday) << "-" << setw(2) << setfill('0') << to_string(ptminfo->tm_hour);
    date2 << to_string(ptminfo->tm_year + 1900) << setw(2) << setfill('0')  << to_string(ptminfo->tm_mon + 1)
          << setw(2) << setfill('0') << to_string(ptminfo->tm_mday) << "-" << setw(2) << setfill('0') << to_string(ptminfo->tm_hour + 1);
    // date >> date2;
    // string date1 = to_string(ptminfo->tm_year + 1900) + to_string(ptminfo->tm_mon + 1) + to_string(ptminfo->tm_mday) + "-" + to_string(ptminfo->tm_hour);
    // string date2 = to_string(ptminfo->tm_year + 1900) + to_string(ptminfo->tm_mon + 1) + to_string(ptminfo->tm_mday) + "-" + to_string(ptminfo->tm_hour + 1);
    string dir_ = "";
    string logname = "";
    if(!dirIsexistence(dir))
    {
        ROS_INFO("%s is not existing!",dir.c_str());
        Createdir(dir.c_str());

        dir = dir + log_dir;
        Createdir(dir.c_str());

        // for(uint8_t i = 0;i < sizeof(cantype)/sizeof(cantype[0]);++i)
        for(list<string>::iterator iter_cantype = cantype.begin();iter_cantype != cantype.end();++iter_cantype)
        {
            dir_ = dir + "/" + *iter_cantype;
            Createdir(dir_.c_str());

            dir_ = dir + "/" + *iter_cantype + "/" + date1.str();
            Createdir(dir_.c_str());
            // if(!cantype[i].compare("camera"))
            // {
            //     for(uint8_t j = ptminfo->tm_min; j < 60; ++j)
            //     {
            //         logname = cantype[i] + "_1-" + to_string(j) + ".csv";
            //         Createfile(dir_.c_str(),logname);
            //         logname = cantype[i] + "_2-" + to_string(j) + ".csv";
            //         Createfile(dir_.c_str(),logname);
            //     }
            // }
            // else
            // {
            //     for(uint8_t j = ptminfo->tm_min; j < 60; ++j)
            //     {
            //         logname = cantype[i] + "-" + to_string(j) + ".csv";
            //         Createfile(dir_.c_str(),logname);
            //     }
            // }

            dir_ = dir + "/" + *iter_cantype + "/" + date2.str();
            Createdir(dir_.c_str());
            // if(!cantype[i].compare("camera"))
            // {
            //     for(uint8_t j = 0; j < 60; ++j)
            //     {
            //         logname = cantype[i] + "_1-" + to_string(j) + ".csv";
            //         Createfile(dir_.c_str(),logname);
            //         logname = cantype[i] + "_2-" + to_string(j) + ".csv";
            //         Createfile(dir_.c_str(),logname);
            //     }
            // }
            // else
            // {
            //     for(uint8_t j = 0; j < 60; ++j)
            //     {
            //         logname = cantype[i] + "-" + to_string(j) + ".csv";
            //         Createfile(dir_.c_str(),logname);
            //     }
            // }
        }
    }
    else
    {
        dir = dir + log_dir;
        if(!dirIsexistence(dir))//log is exist?
        {
            ROS_INFO("%s is not existing!",dir.c_str());
            Createdir(dir.c_str());

            // for(uint8_t i = 0;i < sizeof(cantype)/sizeof(cantype[0]);++i)
            for(list<string>::iterator iter_cantype = cantype.begin();iter_cantype != cantype.end();++iter_cantype)
            {
                dir_ = dir + "/" + *iter_cantype;
                Createdir(dir_.c_str());

                dir_ = dir + "/" + *iter_cantype + "/" + date1.str();
                Createdir(dir_.c_str());
                // if(!cantype[i].compare("camera"))
                // {
                    
                //     for(uint8_t j = ptminfo->tm_min; j < 60; ++j)
                //     {
                //         logname = cantype[i] + "_1-" + to_string(j) + ".csv";
                //         Createfile(dir_.c_str(),logname);
                //         logname = cantype[i] + "_2-" + to_string(j) + ".csv";
                //         Createfile(dir_.c_str(),logname);
                //     }
                // }
                // else
                // {
                //     for(uint8_t j = ptminfo->tm_min; j < 60; ++j)
                //     {
                //         logname = cantype[i] + "-" + to_string(j) + ".csv";
                //         Createfile(dir_.c_str(),logname);
                //     }
                // }

                dir_ = dir + "/" + *iter_cantype + "/" + date2.str();
                Createdir(dir_.c_str());
                // if(!cantype[i].compare("camera"))
                // {
                //     for(uint8_t j = 0; j < 60; ++j)
                //     {
                //         logname = cantype[i] + "_1-" + to_string(j) + ".csv";
                //         Createfile(dir_.c_str(),logname);
                //         logname = cantype[i] + "_2-" + to_string(j) + ".csv";
                //         Createfile(dir_.c_str(),logname);
                //     }
                // }
                // else
                // {
                //     for(uint8_t j = 0; j < 60; ++j)
                //     {
                //         logname = cantype[i] + "-" + to_string(j) + ".csv";
                //         Createfile(dir_.c_str(),logname);
                //     }
                // }
            }
        }
        else
        {
            // for(uint8_t i = 0;i < sizeof(cantype)/sizeof(cantype[0]);++i)
            for(list<string>::iterator iter_cantype = cantype.begin();iter_cantype != cantype.end();++iter_cantype)
            {
                dir_ = dir + "/" + *iter_cantype;
                if(!dirIsexistence(dir_))
                {
                    Createdir(dir_.c_str());

                    dir_ = dir + "/" + *iter_cantype + "/" + date1.str();
                    Createdir(dir_.c_str());
                    // if(!cantype[i].compare("camera"))
                    // {
                    //     for(uint8_t j = ptminfo->tm_min; j < 60; ++j)
                    //     {
                    //         logname = cantype[i] + "_1-" + to_string(j) + ".csv";
                    //         Createfile(dir_.c_str(),logname);
                    //         logname = cantype[i] + "_2-" + to_string(j) + ".csv";
                    //         Createfile(dir_.c_str(),logname);
                    //     }
                    // }
                    // else
                    // {
                    //     for(uint8_t j = ptminfo->tm_min; j < 60; ++j)
                    //     {
                    //         logname = cantype[i] + "-" + to_string(j) + ".csv";
                    //         Createfile(dir_.c_str(),logname);
                    //     }
                    // }

                    dir_ = dir + "/" + *iter_cantype + "/" + date2.str();
                    Createdir(dir_.c_str());
                    // if(!cantype[i].compare("camera"))
                    // {
                    //     for(uint8_t j = 0; j < 60; ++j)
                    //     {
                    //         logname = cantype[i] + "_1-" + to_string(j) + ".csv";
                    //         Createfile(dir_.c_str(),logname);
                    //         logname = cantype[i] + "_2-" + to_string(j) + ".csv";
                    //         Createfile(dir_.c_str(),logname);
                    //     }
                    // }
                    // else
                    // {
                    //     for(uint8_t j = 0; j < 60; ++j)
                    //     {
                    //         logname = cantype[i] + "-" + to_string(j) + ".csv";
                    //         Createfile(dir_.c_str(),logname);
                    //     }
                    // }

                }
                else
                {
                    dir_ = dir + "/" + *iter_cantype + "/" + date1.str();
                    if(!dirIsexistence(dir_))
                    {
                        Createdir(dir_.c_str());
                    }
                    // if(!cantype[i].compare("camera"))
                    // {
                    //     for(uint8_t j = ptminfo->tm_min; j < 60; ++j)
                    //     {
                    //         logname = cantype[i] + "_1-" + to_string(j) + ".csv";
                    //         Createfile(dir_.c_str(),logname);
                    //         logname = cantype[i] + "_2-" + to_string(j) + ".csv";
                    //         Createfile(dir_.c_str(),logname);
                    //     }
                    // }
                    // else
                    // {
                    //     for(uint8_t j = ptminfo->tm_min; j < 60; ++j)
                    //     {
                    //         logname = cantype[i] + "-" + to_string(j) + ".csv";
                    //         Createfile(dir_.c_str(),logname);
                    //     }
                    // }
                    dir_ = dir + "/" + *iter_cantype + "/" + date2.str();
                    if(!dirIsexistence(dir_))
                    {
                        Createdir(dir_.c_str());
                    }
                    // if(!cantype[i].compare("camera"))
                    // {
                    //     for(uint8_t j = 0; j < 60; ++j)
                    //     {
                    //         logname = cantype[i] + "_1-" + to_string(j) + ".csv";
                    //         Createfile(dir_.c_str(),logname);
                    //         logname = cantype[i] + "_2-" + to_string(j) + ".csv";
                    //         Createfile(dir_.c_str(),logname);
                    //     }
                    // }
                    // else
                    // {
                    //     for(uint8_t j = 0; j < 60; ++j)
                    //     {
                    //         logname = cantype[i] + "-" + to_string(j) +packaged ".csv";
                    //         Createfile(dir_.c_str(),logname);
                    //     }
                    // }
                }
            }
        }
    }
}

bool logmanage::dirIsexistence(string dirpath)
{
    if(-1 == access(dirpath.c_str(),F_OK))
    {
        return false;
    }
    else
    {
        return true;
    }
}

bool logmanage::fileIsexistence(string pram1, string pram2)
{
    string allname = pram1 + "/" + pram2;
    ofstream file(allname);
    if(file.is_open())
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

bool logmanage::Createfile(string pram1, string pram2)
{
    string allname = pram1 + "/" + pram2;
    if(logfile.is_open())
    {
        return false;
    }
    else
    {
        logfile.open(allname);
        logfile.close();
        return true;
    }
}

bool logmanage::package(string dir,string filename)
{
    // string cmd = "tar zcvf " + filename + ".tar " + dir + "/" + filename;
    // string cmd  = "tar zcvf " + dir + ".tar " + dir;
    // string cmd  = "7zr a " + dir + ".7z " + dir;
	string cmd  = "zip -q -r -1 -m " + dir + ".zip " + dir;
    system(cmd.c_str());
}

bool logmanage::remove(string dir,string filename)
{
    string cmd = "rm -r " + dir;
    if(dirIsexistence(dir))
    {
        system(cmd.c_str());
        return true;
    }
    else
    {
        return false;
    }
}

void logmanage::time2Createfile()
{
    uint8_t package_hour = 25;
    // string path = "/home/hx/work/log/";
    string path = base_path + work_space + log_dir;
    string path_ = "";
    ostringstream date1;
    ostringstream date2;
    ostringstream packdate;
    string packpath = "";
    string logname = "";
    while(ros::ok())
    {
        if(isMountDisk())
        {
            time(&creattime);
            pcreattime = localtime(&creattime);
            // ROS_INFO("time.min:%d",pcreattime->tm_min);
            if(5 == pcreattime->tm_min && package_hour != (ptminfo->tm_hour + 1))
            {
                package_hour = ptminfo->tm_hour + 1;
                ROS_INFO("time to create dir and file!");
                // date1 = to_string(ptminfo->tm_year + 1900) + to_string(ptminfo->tm_mon + 1) + to_string(ptminfo->tm_mday) + "-" + to_string(ptminfo->tm_hour);
                // date2 = to_string(ptminfo->tm_year + 1900) + to_string(ptminfo->tm_mon + 1) + to_string(ptminfo->tm_mday) + "-" + to_string(ptminfo->tm_hour + 1);
                // packdate = to_string(ptminfo->tm_year + 1900) + to_string(ptminfo->tm_mon + 1) + to_string(ptminfo->tm_mday) + "-" + to_string(ptminfo->tm_hour -1);
                date1.str("");
                date2.str("");
                packdate.str("");
                date1 << to_string(ptminfo->tm_year + 1900)  << setw(2) << setfill('0') << to_string(ptminfo->tm_mon + 1)
                    << setw(2) << setfill('0') << to_string(ptminfo->tm_mday) + "-" << setw(2) << setfill('0') << to_string(ptminfo->tm_hour);
                date2 << to_string(ptminfo->tm_year + 1900)  << setw(2) << setfill('0') << to_string(ptminfo->tm_mon + 1)
                    << setw(2) << setfill('0') << to_string(ptminfo->tm_mday) + "-" << setw(2) << setfill('0') << to_string(ptminfo->tm_hour + 1);
                packdate << to_string(ptminfo->tm_year + 1900)  << setw(2) << setfill('0') << to_string(ptminfo->tm_mon + 1)
                    << setw(2) << setfill('0') << to_string(ptminfo->tm_mday) + "-" << setw(2) << setfill('0') << to_string(ptminfo->tm_hour - 1);
                // ROS_INFO_STREAM("date1:" << date1.str() << ",date2:" << date2.str() << ",packdate:" << packdate.str());
                // for(uint8_t i = 0;i < sizeof(cantype)/sizeof(cantype[0]);++i)
                for(list<string>::iterator iter_cantype = cantype.begin();iter_cantype != cantype.end();++iter_cantype)
                {
                    packpath = path + "/" + *iter_cantype + "/" + packdate.str();
                    push_qtimepath(packpath);
                    path_ = path + "/" + *iter_cantype + "/" + date1.str();
                    // ROS_INFO_STREAM("packpath:" << packpath << ",path_:" << path_);
                    if(!dirIsexistence(path_.c_str()))//不存在
                    {
                        Createdir(path_.c_str());
                        // if(!cantype[i].compare("camera"))
                        // {
                        //     for(uint8_t j = 0; j < 60; ++j)
                        //     {
                        //         logname = cantype[i] + "_1-" + to_string(j) + ".csv";
                        //         Createfile(packpath.c_str(),logname);
                        //         logname = cantype[i] + "_2-" + to_string(j) + ".csv";
                        //         Createfile(packpath.c_str(),logname);
                        //     }
                        // }
                        // else
                        // {
                        //     for(uint8_t j = 0; j < 60; ++j)
                        //     {
                        //         logname = cantype[i] + "-" + to_string(j) + ".csv";
                        //         Createfile(packpath.c_str(),logname);
                        //     }
                        // }
                    }
                    path_ = path + "/" + *iter_cantype + "/" + date2.str();
                    // ROS_INFO_STREAM(",path_:" << path_);
                    if(!dirIsexistence(path_.c_str()))//不存在
                    {
                        Createdir(path_.c_str());
                        // if(!cantype[i].compare("camera"))
                        // {
                        //     for(uint8_t j = 0; j < 60; ++j)
                        //     {
                        //         logname = cantype[i] + "_1-" + to_string(j) + ".csv";
                        //         Createfile(packpath.c_str(),logname);
                        //         logname = cantype[i] + "_2-" + to_string(j) + ".csv";
                        //         Createfile(packpath.c_str(),logname);
                        //     }
                        // }
                        // else
                        // {
                        //     for(uint8_t j = 0; j < 60; ++j)
                        //     {
                        //         logname = cantype[i] + "-" + to_string(j) + ".csv";
                        //         Createfile(packpath.c_str(),logname);
                        //     }
                        // }
                    }
                }
                cond_package.notify_one();
            }
        }
        sleep(1);
    }
}

void logmanage::time2Package()
{
    ROS_INFO("LOGMANAGER time2Package is running~");
    unique_lock<mutex> lock(mtx_package);
    string packpath = "";
    while(ros::ok())
    {
        // CPU_OCCUPY cpu_stat1;
        // CPU_OCCUPY cpu_stat2;
        // get_cpuoccupy(( CPU_OCCUPY * )&cpu_stat1);
        //     sleep(1);
        //     get_cpuoccupy(( CPU_OCCUPY * )&cpu_stat2);
        // int cpu = cal_cpuoccupy(( CPU_OCCUPY * )&cpu_stat1, ( CPU_OCCUPY * )&cpu_stat2);

        cond_package.wait(lock);
        if(isMountDisk())
        {
            while(!qtimepath_empty())
            {
                CPU_OCCUPY cpu_stat1;
                CPU_OCCUPY cpu_stat2;
                get_cpuoccupy(( CPU_OCCUPY * )&cpu_stat1);
                sleep(1);
                get_cpuoccupy(( CPU_OCCUPY * )&cpu_stat2);
                int cpu = cal_cpuoccupy(( CPU_OCCUPY * )&cpu_stat1, ( CPU_OCCUPY * )&cpu_stat2);
                if(cpu < 5000)
                {
                    packpath = front_qtimepath();
                    package(packpath,"");
                    remove(packpath,"");
                }
                ROS_INFO("package!");
                sleep(10);
            }
        }
    }
}


int logmanage::cal_cpuoccupy(CPU_OCCUPY *o, CPU_OCCUPY *n)
{
    unsigned long od, nd;
    unsigned long id, sd;
    int cpu_use = 0;

    od = ( unsigned long )(o->user + o->nice + o->system + o->idle); //第一次(用户+优先级+系统+空闲)的时间再赋给od
    nd = ( unsigned long )(n->user + n->nice + n->system + n->idle); //第二次(用户+优先级+系统+空闲)的时间再赋给od

    id = ( unsigned long )(n->user - o->user);     //用户第一次和第二次的时间之差再赋给id
    sd = ( unsigned long )(n->system - o->system); //系统第一次和第二次的时间之差再赋给sd
    if ((nd - od) != 0)
        cpu_use = ( int )((sd + id) * 10000) / (nd - od); //((用户+系统)乖100)除(第一次和第二次的时间差)再赋给g_cpu_used
    else
        cpu_use = 0;
     printf("cpu: %u\n", cpu_use);
    return cpu_use;
}

void logmanage::get_cpuoccupy(CPU_OCCUPY *cpust) //对无类型get函数含有一个形参结构体类弄的指针O
{
    FILE *fd;
    int n;
    char buff[256];
    CPU_OCCUPY *cpu_occupy;
    cpu_occupy = cpust;

    fd = fopen("/proc/stat", "r");
    fgets(buff, sizeof(buff), fd);

    sscanf(buff, "%s %u %u %u %u", cpu_occupy->name, &cpu_occupy->user, &cpu_occupy->nice, &cpu_occupy->system,
            &cpu_occupy->idle);

    fclose(fd);
}
