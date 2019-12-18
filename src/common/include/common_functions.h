#ifndef COMMON_FUNCTIONS_H
#define COMMON_FUNCTIONS_H

#include "ros/ros.h"

#include <cstdio>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <sys/time.h>
#include <time.h>
#include <vector>

#define VMRSS_LINE 22
#define VMSIZE_LINE 18
#define PROCESS_ITEM 14

using namespace std;

typedef struct
{
  unsigned long user;
  unsigned long nice;
  unsigned long system;
  unsigned long idle;
} Total_Cpu_Occupy_t;

typedef struct
{
  unsigned int pid;
  unsigned long utime;  // user time
  unsigned long stime;  // kernel time
  unsigned long cutime; // all user time
  unsigned long cstime; // all dead time
} Proc_Cpu_Occupy_t;
//获取第N项开始的指针
const char *get_items(const char *buffer, unsigned int item)
{

  const char *p = buffer;

  int len   = strlen(buffer);
  int count = 0;

  for (int i = 0; i < len; i++)
  {
    if (' ' == *p)
    {
      count++;
      if (count == item - 1)
      {
        p++;
        break;
      }
    }
    p++;
  }

  return p;
}

//获取总的CPU时间
unsigned long get_cpu_total_occupy()
{

  FILE *fd;
  char buff[1024] = {0};
  Total_Cpu_Occupy_t t;

  fd = fopen("/proc/stat", "r");
  if (nullptr == fd)
  {
    return 0;
  }

  fgets(buff, sizeof(buff), fd);
  char name[64] = {0};
  sscanf(buff, "%s %ld %ld %ld %ld", name, &t.user, &t.nice, &t.system, &t.idle);
  fclose(fd);

  return (t.user + t.nice + t.system + t.idle);
}

//获取进程的CPU时间
unsigned long get_cpu_proc_occupy(unsigned int pid)
{

  char file_name[64] = {0};
  Proc_Cpu_Occupy_t t;
  FILE *fd;
  char line_buff[1024] = {0};
  sprintf(file_name, "/proc/%d/stat", pid);

  fd = fopen(file_name, "r");
  if (nullptr == fd)
  {
    return -1;
  }

  fgets(line_buff, sizeof(line_buff), fd);

  sscanf(line_buff, "%u", &t.pid);
  const char *q = get_items(line_buff, PROCESS_ITEM);
  sscanf(q, "%ld %ld %ld %ld", &t.utime, &t.stime, &t.cutime, &t.cstime);
  fclose(fd);

  return (t.utime + t.stime + t.cutime + t.cstime);
}

//获取CPU占用率
float get_proc_cpu(unsigned int pid, const unsigned long totalcputime, unsigned long &procputime)
{
  // ros::Time t1 = ros::Time::now();
  // unsigned long totalcputime1, totalcputime2;
  unsigned long procputime1;

  // totalcputime1 = totalcputime;
  procputime1 = procputime;

  // totalcputime2 = get_cpu_total_occupy();
  procputime = get_cpu_proc_occupy(pid);
  if (procputime == -1)
    return -1;

  float pcpu = 0.0;
  if (0 != totalcputime)
  {
    pcpu = 100.0 * 8.0 * ( float )(procputime - procputime1) / ( float )totalcputime;
  }
  // ROS_WARN("procputime[%ld] procputime1[%ld] totalcputime[%ld] ", procputime, procputime1, totalcputime);
  // ros::Time t2    = ros::Time::now();
  // ros::Duration d = t2 - t1;
  // ROS_WARN("get_proc_cpu NSec = [%ld]", d.toNSec());
  return pcpu;
}

//获取进程内存占用率
float get_proc_mem(unsigned int pid)
{
  // ros::Time t1       = ros::Time::now();
  char file_name[64] = {0};
  FILE *fd;
  char line_buff[512] = {0};
  sprintf(file_name, "/proc/%d/status", pid);

  fd = fopen(file_name, "r");
  if (nullptr == fd)
  {
    return -1;
  }

  char name[64];
  int vmrss;
  float pmem = 0.0;
  for (int i = 0; i < VMRSS_LINE - 1; i++)
  {
    fgets(line_buff, sizeof(line_buff), fd);
  }

  fgets(line_buff, sizeof(line_buff), fd);
  sscanf(line_buff, "%s %d", name, &vmrss);
  fclose(fd);

  pmem = 100.0 * vmrss / 16287152.0;

  // ros::Time t2    = ros::Time::now();
  //  ros::Duration d = t2 - t1;
  // ROS_WARN("get_proc_mem[%s] NSec = [%ld]", name, d.toNSec());

  return pmem;
}

//获取进程占用虚拟内存
unsigned int get_proc_virtualmem(unsigned int pid)
{

  char file_name[64] = {0};
  FILE *fd;
  char line_buff[512] = {0};
  sprintf(file_name, "/proc/%d/status", pid);

  fd = fopen(file_name, "r");
  if (nullptr == fd)
  {
    return 0;
  }

  char name[64];
  int vmsize;
  for (int i = 0; i < VMSIZE_LINE - 1; i++)
  {
    fgets(line_buff, sizeof(line_buff), fd);
  }

  fgets(line_buff, sizeof(line_buff), fd);
  sscanf(line_buff, "%s %d", name, &vmsize);
  fclose(fd);

  return vmsize;
}
//////////////////////////////////////////////////

#define _LINE_LENGTH 300
bool GetPidCpuMem(float &cpu, float &mem, int pid, int tid = -1)
{
  bool ret = false;
  char cmdline[100];
  sprintf(cmdline, "ps -o %%cpu,rss,%%mem,pid,tid -mp %d", pid);
  FILE *file;
  file = popen(cmdline, "r");
  if (file == NULL)
  {
    printf("file == NULL\n");
    return false;
  }

  char line[_LINE_LENGTH];
  float l_cpuPrec = 0;
  int l_mem       = 0;
  float l_memPrec = 0;
  int l_pid       = 0;
  int l_tid       = 0;
  if (fgets(line, _LINE_LENGTH, file) != NULL)
  {
    // printf("1st line:%s", line);
    if (fgets(line, _LINE_LENGTH, file) != NULL)
    {
      // printf("2nd line:%s", line);
      sscanf(line, "%f %d %f %d -", &l_cpuPrec, &l_mem, &l_memPrec, &l_pid);
      cpu = l_cpuPrec;
      mem = l_memPrec;
      if (tid == -1)
        ret = true;
      else
      {
        while (fgets(line, _LINE_LENGTH, file) != NULL)
        {
          sscanf(line, "%f - - - %d", &l_cpuPrec, &l_tid);
          //              printf("other line:%s",line);
          //              cout<<l_cpuPrec<<'\t'<<l_tid<<endl;
          if (l_tid == tid)
          {
            printf("cpuVal is tid:%d\n", tid);
            cpu = l_cpuPrec;
            ret = true;
            break;
          }
        }
        if (l_tid != tid)
          printf("TID not exist\n");
      }
    }
    else
      printf("PID not exist\n");
  }
  else
    printf("Command or Parameter wrong\n");
  pclose(file);
  return ret;
}
void string_replace(std::string &strBig, const std::string &strsrc, const std::string &strdst)
{
  std::string::size_type pos    = 0;
  std::string::size_type srclen = strsrc.size();
  std::string::size_type dstlen = strdst.size();

  while ((pos = strBig.find(strsrc, pos)) != std::string::npos)
  {
    strBig.replace(pos, srclen, strdst);
    pos += dstlen;
  }
}

void SplitString(const std::string &s, std::vector< std::string > &v, const std::string &c)
{
  std::string::size_type pos1, pos2;
  pos2 = s.find(c);
  pos1 = 0;
  std::string str;
  while (std::string::npos != pos2)
  {
    str = s.substr(pos1, pos2 - pos1);
    if (str != "")
      v.push_back(str);

    pos1 = pos2 + c.size();
    pos2 = s.find(c, pos1);
  }
  if (pos1 != s.length())
  {
    str = s.substr(pos1);
    if (str != "")
      v.push_back(str);
  }
}
int valueAtBit(uint8_t num, uint8_t bit)
{

  if ((num >> (bit)) & 1)
    return 1;
  else
    return 0;
}
// 判断某位是否为1
bool JudgeBitIsOne(uint8_t ucData, uint8_t nbit)
{
  if (0x00 == ucData) //(（N>>（B-1）)&1
    return false;
  else if (ucData == nbit)
    return true;
}

std::string trimString(const std::string &str)
{
  int s          = str.find_first_not_of(" ");
  int e          = str.find_last_not_of(" ");
  std::string ss = str.substr(s, e - s + 1);
  return ss;
}
uint32_t byteslistToInt32(const std::vector< unsigned char > bytes, int size)
{
  uint32_t a = 0;

  a = bytes.at(3) & 0xFF;
  a |= ((bytes.at(2) << 8) & 0xFF00);
  a |= ((bytes.at(1) << 16) & 0xFF0000);
  a |= ((bytes.at(0) << 24) & 0xFF000000);

  return a;
}
uint32_t bytesToInt32(unsigned char *bytes, int size)
{
  uint32_t a = 0;

  a = bytes[3] & 0xFF;
  a |= ((bytes[2] << 8) & 0xFF00);
  a |= ((bytes[1] << 16) & 0xFF0000);
  a |= ((bytes[0] << 24) & 0xFF000000);

  return a;
}
// c++ byte转int16
uint16_t bytesToInt16(unsigned char *bytes)
{
  uint16_t a = 0;

  a = bytes[1] & 0xFF;
  a |= ((bytes[0] << 8) & 0xFF00);

  return a;
}
// c++ int32转byte
int int32ToBytes(uint32_t i_int, unsigned char *bytes)
{
  bytes[3] = ( unsigned char )(i_int & 0xFF);
  bytes[2] = ( unsigned char )((i_int & 0xFF00) >> 8);
  bytes[1] = ( unsigned char )((i_int & 0xFF0000) >> 16);
  bytes[0] = ( unsigned char )((i_int >> 24) & 0xFF);
  return 4;
}
std::string byteToHexString(const unsigned char *data, size_t size)
{
  std::ostringstream strHex;
  strHex << std::hex << std::setfill('0');
  for (size_t i = 0; i < size; ++i)
  {
    strHex << std::setw(2) << static_cast< unsigned int >(data[i]);
  }
  return strHex.str();
}

std::vector< unsigned char > hexStringToByte(const std::string &hex)
{
  std::string str = trimString(hex);
  std::vector< unsigned char > dest;
  int len = hex.size();

  dest.reserve(len / 2);
  for (decltype(len) i = 0; i < len; i += 2)
  {
    unsigned int element;
    std::istringstream strHex(hex.substr(i, 2));
    strHex >> std::hex >> element;
    dest.push_back(static_cast< unsigned char >(element));
  }
  return dest;
}

//字符串分割函数
std::vector< std::string > string_split(std::string str, std::string pattern)
{
  std::string::size_type pos;
  std::vector< std::string > result;

  str += pattern; //扩展字符串以方便操作
  int size = str.size();

  for (int i = 0; i < size; i++)
  {
    pos = str.find(pattern, i);
    if (pos < size)
    {
      std::string s = str.substr(i, pos - i);
      result.push_back(s);
      i = pos + pattern.size() - 1;
    }
  }
  return result;
}

template < class Type > Type stringToNum(const string &str)
{
  istringstream iss(str);
  Type num;
  iss >> num;
  return num;
}
////////////////////////////////////////////
typedef struct
{
  int year;
  int month;
  int day;
  int hour;
  int minute;
  int second;
} TIME;
TIME *getSystemLocalTime()
{
  TIME *t = new TIME();
  time_t timep;
  struct tm *p;
  time(&timep);
  p         = localtime(&timep); //取得当地时间
  t->year   = 1900 + p->tm_year;
  t->month  = 1 + p->tm_mon;
  t->day    = p->tm_mday;
  t->hour   = p->tm_hour;
  t->minute = p->tm_min;
  t->second = p->tm_sec;
  return t;
}

#endif // COMMON_FUNCTIONS_H