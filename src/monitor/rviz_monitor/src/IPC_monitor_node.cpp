#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdio.h>
#include <unistd.h>

//共享内存
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/types.h>

#include "common_functions.h"

#include "monitor_common_utils.h"

#include <iostream>
#include <ros/master.h>
#include <sstream>

// #include "glog_helper.h"

#include <vector>

#include "common_msgs/KeyValue.h"
#include "status_msgs/NodeStatus.h"
#include "status_msgs/SafetyStatus.h"

#include "location_msgs/FusionDataInfo.h"

using namespace std;

#define PATHNAME "/tmp"
#define PROJ_ID 0x6666

static int CommShm(int size, int flags)
{
  key_t key = ftok(PATHNAME, PROJ_ID);
  printf("key_t: %08x\n", key);
  if (key < 0)
  {
    perror("ftok");
    return -1;
  }
  int shmid = 0;

  if ((shmid = shmget(key, size, flags)) < 0)
  {
    perror("shmget");
    return -2;
  }
  printf("shmid: %d\n", shmid);
  return shmid;
}
int DestroyShm(int shmid)
{
  if (shmctl(shmid, IPC_RMID, NULL) < 0)
  {
    perror("shmctl");
    return -1;
  }
  return 0;
}
int CreateShm(int size)
{
  return CommShm(size, IPC_CREAT | IPC_EXCL | 0666);
}
int GetShm(int size)
{
  return CommShm(size, IPC_CREAT | 0666);
}

int get_shm_nattch(int shmid_)
{
  shmid_ds ds_;
  if (shmctl(shmid_, IPC_STAT, &ds_) < 0)
  {
    perror("shmctl");
    return -1;
  }
  return ( int )ds_.shm_nattch;
  // return 0;
}

void get_memoccupy(MEM_OCCUPY *mem) //对无类型get函数含有一个形参结构体类弄的指针O
{
  FILE *fd;
  int n;
  char buff[256];
  MEM_OCCUPY *m;
  m = mem;

  fd = fopen("/proc/meminfo", "r");

  fgets(buff, sizeof(buff), fd);
  sscanf(buff, "%s %u %*s", m->name1, &m->MemTotal);
  fgets(buff, sizeof(buff), fd);
  sscanf(buff, "%s %u %*s", m->name2, &m->MemFree);
  fgets(buff, sizeof(buff), fd);
  fgets(buff, sizeof(buff), fd);
  sscanf(buff, "%s %u %*s", m->name3, &m->Buffers);
  fgets(buff, sizeof(buff), fd);
  sscanf(buff, "%s %u %*s", m->name4, &m->Cached);

  fgets(buff, sizeof(buff), fd);
  sscanf(buff, "%s %u %*s", m->name5, &m->SwapCached);

  fclose(fd); //关闭文件fd
}

int cal_cpuoccupy(CPU_OCCUPY *o, CPU_OCCUPY *n)
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
  // printf("cpu: %u\n", cpu_use);
  return cpu_use;
}

void get_cpuoccupy(std::vector< CPU_OCCUPY > &cpust, const int _num) //对无类型get函数含有一个形参结构体类弄的指针O
{
  FILE *fd;
  int n;
  char buff[256];
  CPU_OCCUPY cpu_occupy;

  fd = fopen("/proc/stat", "r");
  for (int i = 0; i < _num; i++)
  {
    fgets(buff, sizeof(buff), fd);
    sscanf(buff, "%s %u %u %u %u", cpu_occupy.name, &cpu_occupy.user, &cpu_occupy.nice, &cpu_occupy.system,
           &cpu_occupy.idle);
    cpust.push_back(cpu_occupy);
  }

  fclose(fd);
}
void cal_cpuoccupy1(std::vector< CPU_OCCUPY > &cpust, std::vector< float > &cpu_occupancy, const int _num)
{
  std::vector< CPU_OCCUPY > old(cpust);

  cpust.clear();
  get_cpuoccupy(cpust, _num);
  int cpu_use = 0;

  cpu_occupancy.clear();
  for (int i = 0; i < _num; i++)
  {
    cpu_use = cal_cpuoccupy(( CPU_OCCUPY * )&old[i], ( CPU_OCCUPY * )&cpust[i]);
    cpu_occupancy.push_back(( float )cpu_use / 100.0);
  }
}

int get_cpu_processor_num() // cat /proc/cpuinfo| grep "processor"| wc -l
{
  bool ret = false;

  FILE *file;
  file = popen("cat /proc/cpuinfo| grep 'processor'| wc -l", "r");
  if (file == NULL)
  {
    printf("file == NULL\n");
    return 0;
  }
  char buff[256];
  int l_cpuPrec = 0;

  if (fgets(buff, sizeof(buff), file) != NULL)
  {
    sscanf(buff, "%d", &l_cpuPrec);
  }

  pclose(file);
  return l_cpuPrec;
}

int get_pid_stime_etime(int pid_, string &stime_, string &etime_)
{
  bool ret = false;

  char file_name[64] = {0};
  sprintf(file_name, "ps -p %d -o lstart=,etime=", pid_);
  // sprintf(file_name, "ps -p 1 -o lstart=,etime=", pid_);

  FILE *file;
  file = popen(file_name, "r");
  if (file == NULL)
  {
    printf("file == NULL\n");
    return 0;
  }
  char buff[256];

  char s_year[64] = {0};
  char s_mon[64]  = {0};
  char s_day[64]  = {0};
  char s_week[64] = {0};
  char s_time[64] = {0};
  char e_time[64] = {0};
  std::ostringstream oss;

  if (fgets(buff, sizeof(buff), file) != NULL)
  {

    sscanf(buff, "%s %s %s %s %s %s", s_week, s_mon, s_day, s_time, s_year, e_time);

    oss.str("");
    oss << s_year << "-" << s_mon << "-" << s_day << " " << s_time;
    stime_ = oss.str();

    oss.str("");
    oss << e_time;
    etime_ = oss.str();
  }
  else
    return 0;

  pclose(file);
  return pid_;
}
/////////////////////////////////////////////////////////////////////////

void recvFusionLocationCallback(const location_msgs::FusionDataInfo::ConstPtr &location_msg)
{
}
#define NODE_NAME "IPC_monitor_node"
int main(int argc, char *argv[])
{
  ROS_INFO("ROS node is start, name is [%s], file name is %s", NODE_NAME, argv[0]);

  ros::init(argc, argv, NODE_NAME);

  ros::NodeHandle n;
  ros::NodeHandle private_nh("~");

  // GLogHelper gh(argv[0]);
  // gh.setLogDirectory(NODE_NAME);

  // SUPERG_INFO << "ros::this_node::getName()=" << ros::this_node::getName();
  // SUPERG_INFO << "PID=" << getpid();

  //获取CPU型号 以及CPU处理器数
  int cpuProcNum = get_cpu_processor_num();
  //  SUPERG_INFO << "get_cpu_processor() = " << cpuProcNum;

  //获取CPU线程 占用百分比
  std::vector< CPU_OCCUPY > cpu_stat;
  std::vector< float > cpu_occupancy;

  //第一次获取cpu使用情况
  get_cpuoccupy(cpu_stat, cpuProcNum + 1);

  //获取内存使用情况
  MEM_OCCUPY mem_stat;

  int pid = getpid();
  //  SUPERG_INFO << "getpid() = " << pid;

  ros::Publisher pub_ = n.advertise< status_msgs::NodeStatus >("/node/node_status", 10);

  //订阅 车辆启动信息，来判断是否需要开启原始数据落盘

  // 1. 创建 SHM
  int shmid      = GetShm(1024);
  char *shm_addr = ( char * )shmat(shmid, NULL, 0);
  ros::NodeHandle pn;
  // int num;
  // pn.param< int >("datalog_param", num, 0);
  ros::Subscriber sub_ = n.subscribe("/localization/fusion_msg", 1, recvFusionLocationCallback);
  if (sub_.getNumPublishers() > 0)
  {
    shm_addr[0] = 0x01;
  }
  else
    shm_addr[0] = 0x00;

  ros::Rate loop_rate(10);

  uint32_t count = 0;
  int icount     = 10;

  char buf[100];
  pid_t adcu_pid = -1;
  pid_t nopid    = 0;
  std::string s_time_, e_time_;

  int islog   = 0;
  int numPubs = 0;

  while (ros::ok())
  {
    // float cpu[cpuProcNum + 1] = {0.00};

    icount++;
    int i = 0;

    // SUPERG_INFO << "ros::ok()";

    //获取内存
    get_memoccupy(( MEM_OCCUPY * )&mem_stat);
    // printf(" [MemTotal] = %u \n [MemFree] = %u \n [Buffers] = %u \n [Cached] = %u \n [SwapCached] = %u \n",
    //        mem_stat.MemTotal, mem_stat.MemFree, mem_stat.Buffers, mem_stat.Cached, mem_stat.SwapCached);

    status_msgs::NodeStatus ns;
    ns.node_name = ros::this_node::getName();
    ns.node_pid  = getpid();
    ns.state_num = 1;
    status_msgs::SafetyStatus ss;

    count++;
    common_msgs::KeyValue kv;
    ss.message_code = "I04012000";
    ss.counter      = count;
    ss.hardware_id  = 903;

    std::ostringstream oss;

    kv.key = "MEM(%)";
    oss << std::fixed << std::setprecision(2) << std::noshowpoint
        << (( double )(mem_stat.MemTotal - mem_stat.MemFree) / ( double )mem_stat.MemTotal) * 100.0;
    kv.value     = oss.str();
    kv.valuetype = 7;
    ss.values.push_back(kv);

    kv.key = mem_stat.name1;
    string_replace(kv.key, ":", "(MB)");
    oss.str("");
    oss << std::fixed << std::setprecision(2) << std::noshowpoint << (mem_stat.MemTotal / 1024.0);
    kv.value     = oss.str();
    kv.valuetype = 7;
    ss.values.push_back(kv);

    kv.key = mem_stat.name2;
    string_replace(kv.key, ":", "(MB)");
    oss.str("");
    oss << std::fixed << std::setprecision(2) << std::noshowpoint << (mem_stat.MemFree / 1024.0);
    kv.value     = oss.str();
    kv.valuetype = 7;
    ss.values.push_back(kv);

    kv.key = mem_stat.name3;
    string_replace(kv.key, ":", "(MB)");
    oss.str("");
    oss << std::fixed << std::setprecision(2) << std::noshowpoint << (mem_stat.Buffers / 1024.0);
    kv.value     = oss.str();
    kv.valuetype = 7;
    ss.values.push_back(kv);

    kv.key = mem_stat.name4;
    string_replace(kv.key, ":", "(MB)");
    oss.str("");
    oss << std::fixed << std::setprecision(2) << std::noshowpoint << (mem_stat.Cached / 1024.0);
    kv.value     = oss.str();
    kv.valuetype = 7;
    ss.values.push_back(kv);

    kv.key = mem_stat.name5;
    string_replace(kv.key, ":", "(MB)");
    oss.str("");
    oss << std::fixed << std::setprecision(2) << std::noshowpoint << (mem_stat.SwapCached / 1024.0);
    kv.value     = oss.str();
    kv.valuetype = 7;
    ss.values.push_back(kv);

    ss.value_num = ss.values.size();
    ns.status.push_back(ss);

    //计算cpu使用率
    if (icount > 9)
    {
      //判断是否需要落盘
      if (sub_.getNumPublishers() > 0)
      {
        printf("getNumPublishers: >0 \n");
        shm_addr[0] = 0x01;
      }
      else
        shm_addr[0] = 0x00;

      icount = 0;
      cal_cpuoccupy1(cpu_stat, cpu_occupancy, cpuProcNum + 1);

      TIME *time_ = getSystemLocalTime();
      FILE *fp    = NULL;
      fp          = NULL;
      fp          = popen("pidof adcuServer", "r"); // adcuServer
      if (!fp)
      {
        // SUPERG_ERROR << "open err";
      }
      else
      {
        if (fgets(buf, 255, fp) != NULL)
        {
          if (adcu_pid != atoi(buf))
          {
            adcu_pid = atoi(buf);
            // SUPERG_INFO << "adcuSDK PID = " << adcu_pid;
          }
        }
        else
        {
          if (nopid != adcu_pid)
          {
            // SUPERG_ERROR << "adcuSDK not runing!";
            nopid = adcu_pid;
          }
        }
      }
      pclose(fp);

      if (adcu_pid > 0)
      {
        get_pid_stime_etime(adcu_pid, s_time_, e_time_);
      }
    }

    ss.message_code = "I04012001";
    ss.counter      = count;
    ss.hardware_id  = 902;

    ss.values.clear();
    for (i = 0; i < cpu_occupancy.size(); i++)
    {
      kv.key = cpu_stat[i].name;
      kv.key += "(%)";
      // string_replace(kv.key, ":", "(MB)");
      oss.str("");
      oss << std::fixed << std::setprecision(2) << std::noshowpoint << cpu_occupancy[i];
      kv.valuetype = 7;
      kv.value     = oss.str();
      ss.values.push_back(kv);
    }
    ss.value_num = ss.values.size();
    ns.status.push_back(ss);

    // adcu SDK

    ss.message_code = "I04012002";
    ss.counter      = count;
    ss.hardware_id  = 0;

    ss.values.clear();

    kv.key = "adcuSDK pid";
    oss.str("");
    oss << adcu_pid;
    kv.value     = oss.str();
    kv.valuetype = 3;
    ss.values.push_back(kv);

    kv.key       = "adcuSDK stime";
    kv.value     = s_time_;
    kv.valuetype = 9;
    ss.values.push_back(kv);

    kv.key       = "adcuSDK etime";
    kv.value     = e_time_;
    kv.valuetype = 9;
    ss.values.push_back(kv);

    ss.value_num = ss.values.size();
    ns.status.push_back(ss);

    // pub

    ns.header.frame_id = "/odom";
    ns.state_num       = ns.status.size();
    ns.header.stamp    = ros::Time::now();

    pub_.publish(ns);
    // SUPERG_INFO << ros::Time::now();

    ros::spinOnce();

    loop_rate.sleep();
  }
  shmdt(shm_addr);
  if (get_shm_nattch(shmid) == 0)
  {
    DestroyShm(shmid);
  }

  return 0;
}