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
using namespace std;
/*
 * Get the URI of the master.
 */
bool getUri(std::string &uri)
{
  XmlRpc::XmlRpcValue args, result, payload;
  args[0] = ros::this_node::getName();

  if (!ros::master::execute("getUri", args, result, payload, true))
  {
    std::cout << "Failed!" << std::endl;
    return false;
  }
  std::cout << "----------Master URI----------" << std::endl;
  uri = std::string(payload);
  std::cout << std::string(payload) << std::endl;
}

/*
 * Get list of topics that can be subscribed to.
 * This does not return topics that have no publishers.
 * See getSystemState() to get more comprehensive list.
 * */
bool getPublishedTopics(const std::string &subgraph, ros::master::V_TopicInfo &topics)
{
  XmlRpc::XmlRpcValue args, result, payload;
  args[0] = ros::this_node::getName();
  args[1] = subgraph;
  if (!ros::master::execute("getPublishedTopics", args, result, payload, true))
  {
    std::cout << "Failed!" << std::endl;
    return false;
  }
  topics.clear();
  std::cout << "----------PublishedTopics----------" << std::endl;
  std::cout << "published_topic_name \t message_name" << std::endl;
  for (int i = 0; i < payload.size(); ++i)
  {
    topics.push_back(ros::master::TopicInfo(std::string(payload[i][0]), std::string(payload[i][1])));
    std::string v1 = std::string(payload[i][0]);
    std::string v2 = std::string(payload[i][1]);
    std::cout << v1.c_str() << "\t" << v2.c_str() << std::endl;
  }

  return true;
}

/*
 * Retrieve list topic names and their types.
 * */
bool getTopicTypes(ros::master::V_TopicInfo &topics)
{
  XmlRpc::XmlRpcValue args, result, payload;
  args[0] = ros::this_node::getName();

  if (!ros::master::execute("getTopicTypes", args, result, payload, true))
  {
    std::cout << "Failed!" << std::endl;
    return false;
  }
  topics.clear();
  std::cout << "----------TopicTypes----------" << std::endl;
  std::cout << "topic_name\t message_name" << std::endl;
  for (int i = 0; i < payload.size(); ++i)
  {
    topics.push_back(ros::master::TopicInfo(std::string(payload[i][0]), std::string(payload[i][1])));
    std::string v1 = std::string(payload[i][0]);
    std::string v2 = std::string(payload[i][1]);
    std::cout << v1.c_str() << "\t" << v2.c_str() << std::endl;
  }

  return true;
}

/*
 * Get the XML-RPC URI of the node with the associated name/caller_id.
 * This API is for looking information about publishers and subscribers.
 * Use lookupService instead to lookup ROS-RPC URIs.
 */
bool lookupNode(const std::string &node, std::string &uri)
{
  XmlRpc::XmlRpcValue args, result, payload;
  args[0] = ros::this_node::getName();
  args[1] = node;
  if (!ros::master::execute("lookupNode", args, result, payload, true))
  {
    std::cout << "Failed!" << std::endl;
    return false;
  }
  std::cout << "----------LookupedNode----------" << std::endl;
  uri = std::string(payload);
  std::cout << node << ":" << std::string(payload) << std::endl;
}

/*
 * Lookup all provider of a particular service.
 */
bool lookupService(const std::string &service, std::string &uri)
{
  XmlRpc::XmlRpcValue args, result, payload;
  args[0] = ros::this_node::getName();
  args[1] = service;
  if (!ros::master::execute("lookupService", args, result, payload, true))
  {
    std::cout << "Failed!" << std::endl;
    return false;
  }
  std::cout << "----------LookupedService----------" << std::endl;
  uri = std::string(payload);
  std::cout << service << ":" << std::string(payload) << std::endl;
}

/*
 * Retrieve list representation of system state
 * (i.e. publishers, subscribers, and services).
 * */
bool getSystemState()
{
  XmlRpc::XmlRpcValue args, result, payload;
  args[0] = ros::this_node::getName();
  if (!ros::master::execute("getSystemState", args, result, payload, true))
  {
    std::cout << "Failed!" << std::endl;
    return false;
  }
  std::cout << "----------SystemState----------" << std::endl;

  // publishers
  int t = 0;
  std::cout << "Published Topics:" << std::endl;
  for (int j = 0; j < payload[t].size(); ++j)
  {
    // topics
    std::cout << "    *" << std::string(payload[t][j][0]) << ":" << std::endl;
    for (int k = 0; k < payload[t][j][1].size(); ++k)
    {
      // publisher
      std::cout << "        *" << std::string(payload[t][j][1][k]) << std::endl;
    }
  }
  t = 1;
  std::cout << "Subscribed Topics:" << std::endl;
  for (int j = 0; j < payload[t].size(); ++j)
  {
    // topics
    std::cout << "    *" << std::string(payload[t][j][0]) << ":" << std::endl;
    for (int k = 0; k < payload[t][j][1].size(); ++k)
    {
      // publisher
      std::cout << "        *" << std::string(payload[t][j][1][k]) << std::endl;
    }
  }

  t = 2;
  std::cout << "Services:" << std::endl;
  for (int j = 0; j < payload[t].size(); ++j)
  {
    // topics
    std::cout << "    *" << std::string(payload[t][j][0]) << ":" << std::endl;
    for (int k = 0; k < payload[t][j][1].size(); ++k)
    {
      // publisher
      std::cout << "        *" << std::string(payload[t][j][1][k]) << std::endl;
    }
  }
  return true;
}

void print_result(FILE *fp)
{
  char buf[100];

  if (!fp)
  {
    return;
  }
  ROS_INFO(">>>");
  while (memset(buf, 0, sizeof(buf)), fgets(buf, sizeof(buf) - 1, fp) != 0)
  {
    ROS_INFO("%s", buf);
  }
  ROS_INFO("<<<");
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

void get_cpuoccupy(CPU_OCCUPY *cpust, int id) //对无类型get函数含有一个形参结构体类弄的指针O
{
  FILE *fd;
  int n;
  char buff[256];
  CPU_OCCUPY *cpu_occupy;
  cpu_occupy = cpust;

  fd = fopen("/proc/stat", "r");
  fgets(buff, sizeof(buff), fd);
  for (int i = 0; i < id; i++)
  {
    fgets(buff, sizeof(buff), fd);
  }
  sscanf(buff, "%s %u %u %u %u", cpu_occupy->name, &cpu_occupy->user, &cpu_occupy->nice, &cpu_occupy->system,
         &cpu_occupy->idle);

  fclose(fd);
}
int cal_cpuoccupy1(CPU_OCCUPY *n, int id_)
{
  CPU_OCCUPY *o;
  *o = *n;

  get_cpuoccupy(n, id_);

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

int get_cpu_processor() // cat /proc/cpuinfo| grep "processor"| wc -l
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

#define NODE_NAME "ros_check"
int main(int argc, char *argv[])
{
  ROS_INFO("ROS node is start, name is [%s], file name is %s", NODE_NAME, argv[0]);

  std::stringstream ss;
  ss << NODE_NAME << "_" << getpid();
  ros::init(argc, argv, ss.str());

  // ros::init(argc, argv, NODE_NAME);

  ros::NodeHandle n;
  ros::NodeHandle private_nh("~");

  // GLogHelper gh(argv[0]);
  // gh.setLogDirectory(NODE_NAME);

  // SUPERG_INFO << "ros::this_node::getName()=" << ros::this_node::getName();
  // SUPERG_INFO << "PID=" << getpid();

  // SUPERG_INFO << "isSimTime = " << ros::Time::isSimTime();
  // SUPERG_INFO << "isSystemTime = " << ros::Time::isSystemTime();

  // SUPERG_INFO << "ros::Time = " << ros::Time::now();
  // SUPERG_INFO << "ros::WallTime = " << ros::WallTime::now();
  // SUPERG_INFO << "ros::Time = " << ros::Time::now();

  // std::vector< std::string > nodes;
  // if (ros::master::getNodes(nodes))
  // {
  //   SUPERG_INFO << "nodes= " << nodes.size();
  //   for (int i = 0; i < nodes.size(); i++)
  //   {
  //     SUPERG_WARN << nodes[i];
  //   }
  // }

  //获取CPU型号 已经CPU线程数

  //获取CPU线程 占用百分比

  //获取内存使用情况

  //获取节点列表
  // ros::V_string v_nodes;
  // if (!ros::master::getNodes(v_nodes))
  // {
  //   std::cout << "Failed!" << std::endl;
  //   return false;
  // }
  // for (int i = 0; i < v_nodes.size(); i++)
  // {
  //   SUPERG_INFO << v_nodes[i];
  // }

  //获取节点PID

  //根据PID 获取CPU 内存 使用情况及进程状态

  //

  // uint8_t temp = 0x88;
  // for (int i = 0; i < 8; i++)
  // {
  //   SUPERG_INFO << "valueAtBit: " << valueAtBit(0x88, i);
  // }

  // SUPERG_INFO << "argv[0]: " << argv[0];

  // for (int i = 0; i < 100; i++)
  // {
  //   SUPERG_INFO << "Hello,info! ";
  //   SUPERG_ERROR << "Hello erroe! " << i;
  //   SUPERG_WARN << "Hello,waring! " << i << "+" << i << "=" << i + i;
  //   // LOG(FATAL) << "Hello,fatal! ";

  //   SUPERG_INFO_IF((i % 10) == 0) << "Hello,info_if " << i;
  //   SUPERG_INFO_EVERY(20) << "Hello,info_every " << i;
  // }

  // std::string s;
  // int num;

  // n.param< std::string >("string_param", s, "haha");
  // private_nh.param< int >("int_param", num, 666);

  // //输出被初始化后的变量值
  // ROS_INFO("string_param_init: %s", s.c_str());
  // ROS_INFO("int_param_init: %d", num);
  // //设置参数的值
  // n.setParam("string_param", "hehe");
  // private_nh.setParam("int_param", 222);

  // double cut_angle;
  // private_nh.param("cut_angle", cut_angle, -0.01);
  // ROS_INFO("cut_angle: %f", cut_angle);

  // double cut_angle_ = int((cut_angle * 360 / (2 * M_PI)) * 100);
  // ROS_INFO("cut_angle_: %f", cut_angle_);
  int pid = 0;
  pid     = getpid();
  // SUPERG_INFO << "getpid() = " << pid;
  int cpuProc = get_cpu_processor();
  // SUPERG_INFO << "get_cpu_processor() = " << cpuProc;

  CPU_OCCUPY cpu_stat[cpuProc + 1];

  MEM_OCCUPY mem_stat;
  int cpu;

  //第一次获取cpu使用情况
  for (int i = 0; i < cpuProc + 1; i++)
  {
    get_cpuoccupy(( CPU_OCCUPY * )&cpu_stat[i], i);
  }

  // // get master uri
  // std::string host_uri;
  // getUri(host_uri);

  // // get published topics
  // std::string subgraph;
  // ros::master::V_TopicInfo published_topics;
  // // getPublishedTopics(subgraph, published_topics);

  // // get topic types
  // ros::master::V_TopicInfo topics;
  // // getTopicTypes(topics);

  // get node uri
  // std::string node, node_uri;
  // node = ros::this_node::getName();
  // ;
  // lookupNode(node, node_uri);

  // // get service uri
  // std::string service, service_uri;
  // service = ros::this_node::getName();
  // ;
  // lookupService(service, service_uri);

  //  get all published topics,subscribed topics and services
  // getSystemState();

  ros::Publisher pub_ = n.advertise< status_msgs::NodeStatus >("/node/node_status", 10);

  ros::Rate loop_rate(10);
  char buf[100];
  // pid_t pid   = -1;
  // pid_t nopid = 0;
  int icount = 0;

  // 1. 创建 SHM
  int shmid      = GetShm(1024);
  char *shm_addr = ( char * )shmat(shmid, NULL, 0);
  while (ros::ok())
  {
    ros::Time t1 = ros::Time::now();
    icount++;
    float cpu  = 0;
    size_t mem = 0;

    int tid = -1;

    if (shm_addr[0] == 1)
    {
      printf("shm_addr[0]:%02X ==1\n", shm_addr[0]);
    }
    else
    {
      printf("shm_addr[0]:%02X ==0\n", shm_addr[0]);
    }

    // SUPERG_INFO << "ros::ok()";

    // if (GetCpuMem(cpu, mem, pid, tid))
    // {
    //   printf("CPU:%.1f\t MEM:%dMB\n", cpu, mem);
    // }

    // TIME *time_ = getSystemLocalTime();
    // FILE *fp    = NULL;
    // fp          = NULL;
    // fp          = popen("pidof rviz", "r"); // adcuServer
    // if (!fp)
    // {
    //   SUPERG_ERROR << "open err";
    // }
    // else
    // {
    //   if (fgets(buf, 255, fp) != NULL)
    //   {
    //     if (pid != atoi(buf))
    //     {
    //       pid = atoi(buf);
    //       SUPERG_INFO << "SDK PID = " << pid;
    //     }
    //   }
    //   else
    //   {
    //     if (nopid != pid)
    //     {
    //       SUPERG_ERROR << "higo SDK not runing!";
    //       nopid = pid;
    //     }
    //   }
    // }
    // pclose(fp);
    status_msgs::NodeStatus ns;
    status_msgs::SafetyStatus ss;

    common_msgs::KeyValue kv;

    if (icount % 2 == 0)
    {
      ns.node_name = ros::this_node::getName();
      ns.node_pid  = getpid();
      ns.state_num = 1;

      ss.message_code = "I04011000";
      ss.counter      = icount;
      ss.hardware_id  = 901;
      ss.value_num    = 8;

      std::ostringstream oss;
      kv.key       = "Agv_Status";
      kv.valuetype = 1;
      kv.value     = to_string(icount % 4);
      ss.values.push_back(kv);

      kv.key       = "running_status";
      kv.valuetype = 1;
      kv.value     = "1";
      ss.values.push_back(kv);

      kv.key       = "offset_y(m)";
      kv.valuetype = 7;
      kv.value     = "0.16";
      ss.values.push_back(kv);

      kv.key       = "offset_heading(deg)";
      kv.valuetype = 7;
      kv.value     = "0.03";
      ss.values.push_back(kv);

      kv.key       = "offset_speed(m/s)";
      kv.valuetype = 7;
      kv.value     = "4.00";
      ss.values.push_back(kv);

      kv.key       = "vehicle_x(m)";
      kv.valuetype = 7;
      kv.value     = "18.35";
      ss.values.push_back(kv);

      kv.key       = "vehicle_y(m)";
      kv.valuetype = 7;
      kv.value     = "25.08";
      ss.values.push_back(kv);

      kv.key       = "min_index";
      kv.valuetype = 7;
      kv.value     = "2.00";
      ss.values.push_back(kv);

      ns.status.push_back(ss);
      ns.header.stamp = ros::Time::now();

      pub_.publish(ns);
    }
    else
    {

      ////////////////////////////////////
      ns.status.clear();
      ss.values.clear();
      ns.node_name = ros::this_node::getName();
      ns.node_pid  = getpid();
      ns.state_num = 2;

      ss.message_code = "I04013000";
      ss.counter      = icount;
      ss.hardware_id  = 901;
      ss.value_num    = 7;

      kv.key       = "running_status";
      kv.valuetype = 1;
      kv.value     = "1";
      ss.values.push_back(kv);

      kv.key       = "offset_y(m)";
      kv.valuetype = 7;
      kv.value     = "5.16";
      ss.values.push_back(kv);

      kv.key       = "offset_heading(deg)";
      kv.valuetype = 7;
      kv.value     = "0.03";
      ss.values.push_back(kv);

      kv.key       = "offset_speed(m/s)";
      kv.valuetype = 7;
      kv.value     = "4.50";
      ss.values.push_back(kv);

      kv.key       = "vehicle_x(m)";
      kv.valuetype = 7;
      kv.value     = "0.00";
      ss.values.push_back(kv);

      kv.key       = "vehicle_y(m)";
      kv.valuetype = 7;
      kv.value     = "0.00";
      ss.values.push_back(kv);

      kv.key       = "min_index";
      kv.valuetype = 7;
      kv.value     = "2.00";
      ss.values.push_back(kv);

      ns.status.push_back(ss);
      ss.values.clear();

      ss.message_code = "W04012005";
      ss.counter      = icount;
      ss.hardware_id  = 901;
      ss.value_num    = 2;

      kv.key       = "vehicle_x(m)";
      kv.valuetype = 7;
      kv.value     = "0.00";
      ss.values.push_back(kv);

      kv.key       = "vehicle_y(m)";
      kv.valuetype = 7;
      kv.value     = "0.00";
      ss.values.push_back(kv);

      ns.status.push_back(ss);

      // ns.header.frame_id    = "/odom";
      ns.header.stamp = ros::Time::now();

      pub_.publish(ns);
    }

    // kv.key = mem_stat.name1;
    // string_replace(kv.key, ":", "(MB)");
    // oss << std::fixed << std::setprecision(2) << std::noshowpoint << (mem_stat.MemTotal / 1024.0);
    // kv.value = oss.str();

    // ss.values.push_back(kv);

    // kv.key = mem_stat.name2;
    // string_replace(kv.key, ":", "(MB)");
    // oss.str("");
    // oss << std::fixed << std::setprecision(2) << std::noshowpoint << (mem_stat.MemFree / 1024.0);
    // kv.value = oss.str();
    // ss.values.push_back(kv);

    // kv.key = mem_stat.name3;
    // string_replace(kv.key, ":", "(MB)");
    // oss.str("");
    // oss << std::fixed << std::setprecision(2) << std::noshowpoint << (mem_stat.Buffers / 1024.0);
    // kv.value = oss.str();
    // ss.values.push_back(kv);

    // kv.key = mem_stat.name4;
    // string_replace(kv.key, ":", "(MB)");
    // oss.str("");
    // oss << std::fixed << std::setprecision(2) << std::noshowpoint << (mem_stat.Cached / 1024.0);
    // kv.value = oss.str();
    // ss.values.push_back(kv);

    // kv.key = mem_stat.name5;
    // string_replace(kv.key, ":", "(MB)");
    // oss.str("");
    // oss << std::fixed << std::setprecision(2) << std::noshowpoint << (mem_stat.SwapCached / 1024.0);
    // kv.value = oss.str();
    // ss.values.push_back(kv);

    // ns.status.push_back(ss);

    // //第一次获取cpu使用情况
    // get_cpuoccupy(( CPU_OCCUPY * )&cpu_stat1);
    // sleep(1);

    // //第二次获取cpu使用情况
    // get_cpuoccupy(( CPU_OCCUPY * )&cpu_stat2);

    // //计算cpu使用率
    // cpu = cal_cpuoccupy(( CPU_OCCUPY * )&cpu_stat1, ( CPU_OCCUPY * )&cpu_stat2);
    // SUPERG_INFO << "cpu usage: " << cpu / 100.0;

    // count++;
    // ss.message_code = "I07014002";
    // ss.counter      = count;
    // ss.hardware_id  = 11;
    // ss.value_num    = 1;
    // ss.values.clear();

    // kv.key = "cpu usage(%)";
    // oss.str("");
    // oss << std::fixed << std::setprecision(2) << std::noshowpoint << (cpu / 100.0);
    // kv.value = oss.str();
    // ss.values.push_back(kv);

    // ns.status.push_back(ss);

    // // ns.header.frame_id    = "/odom";
    // ns.header.stamp = ros::Time::now();

    // pub_.publish(ns);
    ros::Time t2    = ros::Time::now();
    ros::Duration d = t2 - t1;
    // ROS_INFO("%s = %ld NSec", ros::this_node::getName().c_str(), d.toNSec()); // d.toSec());
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