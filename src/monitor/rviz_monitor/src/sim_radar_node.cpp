#include <ros/ros.h>

#include "common_functions.h"

#include "adcuSDK.h"
#include <errno.h>
#include <pthread.h>
#include <semaphore.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>
#include <vector>

using namespace std;

#define DATA_PATH "/work/superg_agv/src/data/zhenjiang_Map"
#define DATA_NAME "/radar00.csv"

#define NODE_NAME "sim_radar_node"

pthread_t ReadThread;

void *readLoop(void *pdata)
{
  int deviceid       = *( int * )pdata;
  int packageCounter = 0;

  while (1)
  {
    uint8_t buffer[1024];
    int length = 0;
    length     = adcuDevRead(deviceid, buffer);
    if (length > 0)
    {
      packageCounter++;
      printf("RX %9d:", packageCounter);
      for (int i = 0; i < length; i++)
      {
        printf("%02X ", buffer[i]);
      }
      printf("\n");
    }
  }
}

int main(int argc, char **argv)
{
  ROS_INFO("ROS node is star, name is [%s], file name is %s", NODE_NAME, argv[0]);
  ros::init(argc, argv, NODE_NAME); // node name
  ros::NodeHandle nh;

  int devid;

  uint8_t data[100];

  adcuDeviceType deviceType;
  int channel;

  //读取 雷达数据文件
  char *line = NULL;

  //获取routing data path
  char *home_path                 = getenv("HOME");
  char route_date_path_name[1024] = {0};
  sprintf(route_date_path_name, "%s%s%s", home_path, DATA_PATH, DATA_NAME);
  printf("radar1 data path name:%s\n", route_date_path_name);

  FILE *fp;
  size_t len = 0;
  ssize_t read;
  fp = fopen(route_date_path_name, "r");

  if (fp == NULL)
  {
    printf("fail to read");
    exit(1);
  }

  adcuSDKInit();
  if (argc != 2)
  {
    printf("please input correct parameters<channelNumber>\n");
    return 0;
  }
  channel = atoi(argv[1]);
  if (channel <= 0 || channel >= ADCU_CHANNEL_MAX)
  {
    printf("please input correct channel number<%d ~ %d>\n", ADCU_CHANNEL_1, ADCU_CHANNEL_MAX - 1);
    return 0;
  }

  bzero(data, 100);
  adcuCanData canbuf;
  canbuf.ide         = 0x00;
  canbuf.dlc         = 0x08;
  canbuf.rtr         = 0x00;
  canbuf.prio        = 0x00;
  canbuf.id          = 0x123;
  canbuf.can_data[0] = 0x09;
  canbuf.can_data[1] = 0x09;
  canbuf.can_data[2] = 0x09;
  canbuf.can_data[3] = 0x09;
  canbuf.can_data[4] = 0x09;
  canbuf.can_data[5] = 0x09;
  canbuf.can_data[6] = 0x09;
  canbuf.can_data[7] = 0x09;

  deviceType = adcuCAN;
  len        = sizeof(adcuCanData);
  memcpy(data, ( uint8_t * )&canbuf, len);

  devid = adcuDevOpen(deviceType, ( adcuDeviceChannel )channel);
  pthread_create(&ReadThread, NULL, readLoop, &devid);

  int status = 0;

  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    if (adcuDevStatus(devid) == ADCU_DEV_STATUS_ABNORMAL)
    {
      break;
    }
    //逐行读取
    read = getline(&line, &len, fp);
    if (feof(fp))
    {
      break;
    }
    // printf("line = %s", line);
    std::vector< std::string > v_line_ = string_split(line, ",");

    canbuf.id                            = byteslistToInt32(hexStringToByte(v_line_.at(1)), 4);
    canbuf.dlc                           = atoi(v_line_.at(2).c_str());
    std::vector< unsigned char > v_data_ = hexStringToByte(v_line_.at(3));
    memset(canbuf.can_data, 0x00, 8);

    for (size_t i = 0; i < canbuf.dlc; i++)
    {
      canbuf.can_data[i] = v_data_.at(i);
    }
    ROS_INFO("id=%x dlc=%d data =%02X %02X %02X %02X %02X %02X %02X %02X ", canbuf.id, canbuf.dlc, canbuf.can_data[0],
             canbuf.can_data[1], canbuf.can_data[2], canbuf.can_data[3], canbuf.can_data[4], canbuf.can_data[5],
             canbuf.can_data[6], canbuf.can_data[7]);

    bzero(data, 100);
    memcpy(data, ( uint8_t * )&canbuf, len);
    if (adcuDevWrite(devid, data, len) <= 0)
    {
      printf("main write error\n");
      goto __end;
    }

    ros::spinOnce();

    loop_rate.sleep();
  }

__end:
  printf("dinit all \n");
  adcuDevClose(devid);
  adcuSDKDeinit();
}
