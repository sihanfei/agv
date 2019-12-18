#include "camera.h"

namespace superg_agv
{
namespace drivers
{
CAMERA::CAMERA()
{
  bCameraInit = true;
  // Camera_stat = Getstatus_Camera;
  //二院
  // 124全部启动
  m_pHandlerMap.insert(make_pair(ALL_START_WORK, &CAMERA::cameraAllStartWork));
  // 125
  m_pHandlerMap.insert(make_pair(ALL_GET_STATUS, &CAMERA::cameraAllGetStatus));
  // 126
  m_pHandlerMap.insert(make_pair(ALL_STOP_ACQUIRE, &CAMERA::cameraAllStopAcquire));
  for (size_t loop_i = 0; loop_i < MAX_CAMERA_NUM; loop_i++)
  {
    // 300 301 302
    m_pHandlerMap.insert(make_pair(CAMERA_START_WORK + (loop_i + 1) * 10, &CAMERA::cameraStartWork));
    m_pHandlerMap.insert(make_pair(CAMERA_STOP_WORK + (loop_i + 1) * 10, &CAMERA::cameraStopWork));
    m_pHandlerMap.insert(make_pair(CAMERA_WORK_STATUS + (loop_i + 1) * 10, &CAMERA::cameraWorkStatusParse));
    // 200 201 202
    if ((loop_i + 1) == CAMERA_LANE_2 || (loop_i + 1) == CAMERA_LANE_4)
    {
      // 200 201 202
      m_pHandlerMap.insert(make_pair(CAMERA_DATA_HEAD + (loop_i + 1) * 10, &CAMERA::laneHeadParse));
      m_pHandlerMap.insert(make_pair(CAMERA_DATA_0 + (loop_i + 1) * 10, &CAMERA::laneDataFirstParse));
      m_pHandlerMap.insert(make_pair(CAMERA_DATA_1 + (loop_i + 1) * 10, &CAMERA::laneDataSecondParse));
      m_pHandlerMap.insert(make_pair(CAMERA_MAT_HEAD + (loop_i + 1) * 10, &CAMERA::matHeadParse));
      m_pHandlerMap.insert(make_pair(CAMERA_MAT_RAW + (loop_i + 1) * 10, &CAMERA::matRawParse));
      m_pHandlerMap.insert(make_pair(CAMERA_MAT_TAIL + (loop_i + 1) * 10, &CAMERA::matTailParse));
    }
    else
    {
      // 200 201 202
      m_pHandlerMap.insert(make_pair(CAMERA_DATA_HEAD + (loop_i + 1) * 10, &CAMERA::objectHeadParse));
      m_pHandlerMap.insert(make_pair(CAMERA_DATA_0 + (loop_i + 1) * 10, &CAMERA::objectDataFirstParse));
      m_pHandlerMap.insert(make_pair(CAMERA_DATA_1 + (loop_i + 1) * 10, &CAMERA::objectDataSecondParse));
      m_pHandlerMap.insert(make_pair(CAMERA_MAT_HEAD + (loop_i + 1) * 10, &CAMERA::matHeadParse));
      m_pHandlerMap.insert(make_pair(CAMERA_MAT_RAW + (loop_i + 1) * 10, &CAMERA::matRawParse));
      m_pHandlerMap.insert(make_pair(CAMERA_MAT_TAIL + (loop_i + 1) * 10, &CAMERA::matTailParse));
    }
  }
  for (uint16_t loop_i = CAMERA_MAT_DATA_BEGIN; loop_i <= CAMERA_MAT_DATA_END; ++loop_i)
  {
    m_pHandlerMap.insert(make_pair(loop_i, &CAMERA::matDataParse));
  }
  map< uint32_t, pFun >::iterator it;
  for (it = m_pHandlerMap.begin(); it != m_pHandlerMap.end(); ++it)
  {
    ROS_INFO("key %u", it->first);
  }
  for(uint16_t i = 0;i < 8;++i)
  {
    array_camera_ObjectData[i].count      = 0;
    array_camera_ObjectData[i].recv_count = 0;
    array_camera_ObjectData[i].time  = 0;
    array_camera_ObjectData[i].bReady     = false;
    array_camera_ObjectData[i].cameraobjectinfo.clear();
    array_camera_LaneData[i].count        = 0;
    array_camera_LaneData[i].recv_count   = 0;
    array_camera_LaneData[i].time    = 0;
    array_camera_LaneData[i].bReady       = false;
    array_camera_LaneData[i].cameralaneinfo.clear();
  }
  list_camera_wirte_task.clear();
  list_camera_read_task.clear();
  //   CANDrivers::
}

CAMERA::~CAMERA()
{
}

// void CAMERA::CANCamera(int &ch_, int &dev_)
// {
//   ROS_INFO("readcamera thread %d %d %d", read_num[ch_], ch_, dev_);
//   while(ros::ok())
//   {
//     adcuCanData canbuf_;
//     int length = 0;
//     map< uint32_t, pFun >::iterator it;
//     length = readCANOnce(dev_,canbuf_);
//     if(length > 0)
//     {
//       for (int i = 0; i < 8; i++)
//           {
//             printf("%02X ", canbuf_.can_data[i]);
//           }
//           printf("\n");
//     }
//   }
//   adcuDevClose(ch_);
// }

void CAMERA::CANCamera(int &ch_, int &dev_)
{
  ROS_INFO("readcamera thread %d %d %d", read_num[ch_], ch_, dev_);

  // Camera_State Camera_stase         = Work_Camera;
  // uint8_t RecvDataCount             = 0;
  // uint8_t Camera_status             = 0;
  uint16_t controltype              = 0;
  // camera_recv_enum CAMERE_RECV_TYPE = NONE;
  string logname                    = "camera";
  int sensor_num                    = 0;
  int camera_can                    = 0;
  int sensor_index                  = 0;
  int sensor_can_type               = 0;
  logdata_output can_log(logname, to_string(dev_), ch_);
  int count_index[9] = {0,0,0,0,0,0,0,0,0};
  Relay relay;
  relay.openCanRelay(15,1);
  sleep(2);
  relay.openCanRelay(17,1);
  while (ros::ok())
  {
      //  ROS_INFO("channel:%d is alive~~~~~~~~~~!",ch_);
    if (adcuDevStatus(dev_) == ADCU_DEV_STATUS_ABNORMAL)
    {
      ROS_INFO("Can %d error", ch_);
      // data_Output.Closefile(logname,logname);
      break;
    }
    adcuCanData canbuf_;
    int length = 0;
    map< uint32_t, pFun >::iterator it;
    canOrder can_order_;

    // ROS_INFO("Camera_stase:Work!!!");
    length = adcuDevRead(dev_, ( uint8_t * )&canbuf_);
      // ROS_INFO("length:%d",length);
      // ROS_INFO("can_log.write_log");
    if (canbuf_.id == 210 || canbuf_.id == 220 || canbuf_.id == 230 || canbuf_.id == 240 ||
        canbuf_.id == 250 || canbuf_.id == 260 || canbuf_.id == 270 || canbuf_.id == 280)
    {
        // q_sensor_push(ch_);
      sensor_num = int(canbuf_.id);
      q_sensor_push(sensor_num);
      count_index[int(canbuf_.id%100/10)]++;
      printf("id:%d,1:%d,2:%d,3:%d,4:%d,5:%d,6:%d,7:%d,8:%d\r\n",int(canbuf_.id),count_index[1],count_index[2],count_index[3],count_index[4],count_index[5],count_index[6],count_index[7],count_index[8]);
    }
    sensor_index = int(canbuf_.id%100/10);
    sensor_can_type = int(canbuf_.id%10);
    // printf("recv data for Camera,canid:%d,num:%d,index:%d,type:%d\n",canbuf_.id,sensor_num,sensor_index,sensor_can_type);
    if (length > 0)
    {
      if(sensor_index == 3)
      {
        printf("camera read:");
        for (int i = 0; i < 8; i++)
        {
          printf("%02X ", canbuf_.can_data[i]);
        }
        printf("\n");
      }
      // if(2 == camera_can)list_camera_read_task
      if(list_camera_read_task.empty())//除数据包没有其他接收的id
      {
        if(sensor_index == CAMERA_OBJECT_1 || sensor_index == CAMERA_OBJECT_3 || sensor_index == CAMERA_OBJECT_5
          || sensor_index == CAMERA_OBJECT_6 || sensor_index == CAMERA_OBJECT_7 || sensor_index == CAMERA_OBJECT_8)//解析物体识别
        {
          if(0 != sensor_can_type)//非帧头处理
          {
            if(2 == sensor_can_type)
            {
              ++array_camera_ObjectData[sensor_index].recv_count;
	            // ROS_INFO_STREAM("RECV:" << array_camera_ObjectData[sensor_index].recv_count << ",COUNT:" << array_camera_ObjectData[sensor_index].count);
            }
          }
          else//帧头处理
          {
            if(0 != array_camera_ObjectData[sensor_index].recv_count)
            {
              array_camera_ObjectData[sensor_index].recv_count = 0;
              array_camera_ObjectData[sensor_index].count      = 0;
              array_camera_ObjectData[sensor_index].time       = 0;
              array_camera_ObjectData[sensor_index].bReady     = false;
              array_camera_ObjectData[sensor_index].cameraobjectinfo.clear();
            }
          }
          it = m_pHandlerMap.find(canbuf_.id);
          if(it != m_pHandlerMap.end())
          {
            (this->*(it->second))(canbuf_, canbuf_.id, can_order_, ch_, controltype);
          }
          if(array_camera_ObjectData[sensor_index].recv_count >= array_camera_ObjectData[sensor_index].count)//接收到指定长度的数据体
          {
            array_camera_ObjectData[sensor_index].bReady     = true;
            if(ch_ == CHANNEL_CAMERA_1)
            {
              mtx_camera_ObjectData1.lock();
              cameraObjectData_ready_1 = array_camera_ObjectData[sensor_index];
              cameraObjectData_ready_1.sensor_index = sensor_index;
              mtx_camera_ObjectData1.unlock();
            }
            else if(ch_ == CHANNEL_CAMERA_2)
            {
              mtx_camera_ObjectData2.lock();
              cameraObjectData_ready_2 = array_camera_ObjectData[sensor_index];
              cameraObjectData_ready_2.sensor_index = sensor_index;
              mtx_camera_ObjectData2.unlock();
            }
            array_camera_ObjectData[sensor_index].recv_count = 0;
            array_camera_ObjectData[sensor_index].count      = 0;
            array_camera_ObjectData[sensor_index].cameraobjectinfo.clear();
                  // ROS_INFO("cameraObjectData_1.cameraobjectinfo.clear()");
            array_camera_ObjectData[sensor_index].bReady     = false;
            cond_camera.notify_one();
          }
        }
        else if(sensor_index == CAMERA_LANE_2 || sensor_index == CAMERA_LANE_4)//解析车道线识别
        {
          if(0 != sensor_can_type)//非帧头处理
          {
            if(2 == sensor_can_type)
            {
              ++array_camera_LaneData[sensor_index].recv_count;
            }
          }
          else//帧头处理
          {
            if(0 != array_camera_LaneData[sensor_index].recv_count)
            {
              array_camera_LaneData[sensor_index].recv_count = 0;
              array_camera_LaneData[sensor_index].count      = 0;
              array_camera_LaneData[sensor_index].time       = 0;
              array_camera_LaneData[sensor_index].bReady     = false;
              array_camera_LaneData[sensor_index].cameralaneinfo.clear();
                  // memset(&array_camera_ObjectData[sensor_index].cameraObjectData,0,sizeof(array_camera_ObjectData[sensor_index].cameraObjectData));
            }
          }
          it = m_pHandlerMap.find(canbuf_.id);
          if (it != m_pHandlerMap.end())
          {
            (this->*(it->second))(canbuf_, canbuf_.id, can_order_, ch_, controltype);
          }
          if(array_camera_LaneData[sensor_index].recv_count >= array_camera_LaneData[sensor_index].count)//接收到指定长度的数据体
          {
            array_camera_LaneData[sensor_index].bReady     = true;
            if(ch_ == CHANNEL_CAMERA_1)
            {
              mtx_camera_LaneData1.lock();
              cameraLaneData_ready_1 = array_camera_LaneData[sensor_index];
              mtx_camera_LaneData1.unlock();
            }
            else if(ch_ == CHANNEL_CAMERA_2)
            {
              mtx_camera_LaneData2.lock();
              cameraLaneData_ready_2 = array_camera_LaneData[sensor_index];
              mtx_camera_LaneData2.unlock();
            }
            array_camera_LaneData[sensor_index].recv_count = 0;
            array_camera_LaneData[sensor_index].count      = 0;
            array_camera_LaneData[sensor_index].cameralaneinfo.clear();
            // ROS_INFO("cameraObjectData_1.cameraobjectinfo.clear()");
            array_camera_LaneData[sensor_index].bReady     = false;
            cond_camera.notify_one();
          }
        }
      }
      else
      {
        it = m_pHandlerMap.find(canbuf_.id);
        if(it != m_pHandlerMap.end())
        {
          (this->*(it->second))(canbuf_, canbuf_.id, can_order_, ch_, controltype);
        }
        //camera status save in camerastatus[9]
      }
    }
    //    usleep(100 * 1);
  }
  adcuDevClose(ch_);
  adcuSDKDeinit();
  ROS_INFO("readcamera thread is over!");
}

void CAMERA::camera_Sender()
{
  ROS_INFO("thread cmaera_Sender Start!");
  unique_lock< mutex > lock(mtx_camera);
  ros::NodeHandle nh;
  ros::Publisher obstacle_pub =
      nh.advertise< perception_sensor_msgs::ObjectList >("/drivers/perception/camera_obstacle_info", 10, true);
  ros::Publisher lane_pub =
      nh.advertise< visualization_msgs::MarkerArray >("/drivers/perception/camera_lane_info", 10, true);
  ros::Publisher mat_pub = nh.advertise< sensor_msgs::Image >("/drivers/perception/camera_mat_info", 10, true);
  double radian;
  float measurement_cov[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  //  perception_msgs::PerceptionObstacleInfo perceptionobstacleinfo;
  //  common_msgs::ObstacleInfo obstacleinfo;
  //  perception_msgs::PerceptionObstacleInfo perceptionobstacleinfo;车道线发布消息
  //  common_msgs::ObstacleInfo laneinfo;车道线信息消息
  while (ros::ok())
  {
    ROS_INFO("waiting camera data!");
    cond_camera.wait(lock);
    mtx_camera_ObjectData1.lock();
    if (cameraObjectData_ready_1.bReady) // object is ready to topic
    {
      // ROS_INFO("cameraobjectdata_1 is ready to topic!!!");
      perception_sensor_msgs::ObjectList perceptionobstacleinfo;
      common_msgs::DetectionInfo obstacleinfo;
      perceptionobstacleinfo.header.stamp            = ros::Time::now();
      perceptionobstacleinfo.header.frame_id         = "base_link";
      perceptionobstacleinfo.sensor_type             = 0;
      perceptionobstacleinfo.sensor_index            = cameraObjectData_ready_1.sensor_index;
      perceptionobstacleinfo.obstacle_num            = cameraObjectData_ready_1.count;
      list< cameraObjectInfo >::iterator ite_Obsinfo = cameraObjectData_ready_1.cameraobjectinfo.begin();
      // ROS_INFO_STREAM("cameraObjectData_1.cameraobjectinfo.size():" <<
      // cameraObjectData_ready_1.cameraobjectinfo.size());
      for (int i = 0; i < cameraObjectData_ready_1.cameraobjectinfo.size(); ++i)
      {
        // obstacleinfo.id         = i;
        obstacleinfo.id         = ite_Obsinfo->object_id;
        obstacleinfo.obj_class  = ite_Obsinfo->obj_class;
        obstacleinfo.confidence = ite_Obsinfo->confidence;
        // obstacleinfo.state.pusb_back(ite_Obsinfo->position_x);
        // obstacleinfo.state.pusb_back(ite_Obsinfo->position_x);
        // obstacleinfo.state.pusb_back(ite_Obsinfo->velocity);
        // obstacleinfo.state.pusb_back(0);
        // obstacleinfo.state[0] = ite_Obsinfo->position_x;
        // obstacleinfo.state[1] = ite_Obsinfo->position_y;
        // obstacleinfo.state[2] = 0;
        // obstacleinfo.state[3] = ite_Obsinfo->velocity;
        // radian = obstacleinfo.theta.data * M_PI/180;
        radian = 0 * M_PI / 180;
        for (int j = 0; j < 16; ++j)
        {
          obstacleinfo.measurement_cov[j] = measurement_cov[j];
        }
        // ROS_INFO_STREAM("cameraObjectData_1.cameraobjectinfo.size():" <<
        // cameraObjectData_ready_1.cameraobjectinfo.size() << ",i:%d" << i);
        if (ite_Obsinfo->obj_class == CAR)
        {
          // ROS_INFO("CAR!!!");
          // obstacleinfo.obj_class  = 1;
          obstacleinfo.peek[0].x =
              ite_Obsinfo->position_x * 1 + (MODE_SIZE_CAR / 2 * cos(radian)) - (MODE_SIZE_CAR / 2 * sin(radian));
          obstacleinfo.peek[0].y =
              ite_Obsinfo->position_y * 1 + (MODE_SIZE_CAR / 2 * sin(radian)) + (MODE_SIZE_CAR / 2 * cos(radian));
          obstacleinfo.peek[1].x =
              ite_Obsinfo->position_x * 1 + (MODE_SIZE_CAR / 2 * cos(radian)) + (MODE_SIZE_CAR / 2 * sin(radian));
          obstacleinfo.peek[1].y =
              ite_Obsinfo->position_y * 1 + (MODE_SIZE_CAR / 2 * sin(radian)) - (MODE_SIZE_CAR / 2 * cos(radian));
          obstacleinfo.peek[2].x =
              ite_Obsinfo->position_x * 1 - (MODE_SIZE_CAR / 2 * cos(radian)) + (MODE_SIZE_CAR / 2 * sin(radian));
          obstacleinfo.peek[2].y =
              ite_Obsinfo->position_y * 1 - (MODE_SIZE_CAR / 2 * sin(radian)) - (MODE_SIZE_CAR / 2 * cos(radian));
          obstacleinfo.peek[3].x =
              ite_Obsinfo->position_x * 1 - (MODE_SIZE_CAR / 2 * cos(radian)) - (MODE_SIZE_CAR / 2 * sin(radian));
          obstacleinfo.peek[3].y =
              ite_Obsinfo->position_y * 1 + (MODE_SIZE_CAR / 2 * sin(radian)) + (MODE_SIZE_CAR / 2 * cos(radian));
        }
        else if (ite_Obsinfo->obj_class == PES)
        {
          // ROS_INFO("PES!!!");
          // obstacleinfo.obj_class  = 2;
          obstacleinfo.peek[0].x =
              ite_Obsinfo->position_x * 1 + (MODE_SIZE_PES / 2 * cos(radian)) - (MODE_SIZE_PES / 2 * sin(radian));
          obstacleinfo.peek[0].y =
              ite_Obsinfo->position_y * 1 + (MODE_SIZE_PES / 2 * sin(radian)) + (MODE_SIZE_PES / 2 * cos(radian));
          obstacleinfo.peek[1].x =
              ite_Obsinfo->position_x * 1 + (MODE_SIZE_PES / 2 * cos(radian)) + (MODE_SIZE_PES / 2 * sin(radian));
          obstacleinfo.peek[1].y =
              ite_Obsinfo->position_y * 1 + (MODE_SIZE_PES / 2 * sin(radian)) - (MODE_SIZE_PES / 2 * cos(radian));
          obstacleinfo.peek[2].x =
              ite_Obsinfo->position_x * 1 - (MODE_SIZE_PES / 2 * cos(radian)) + (MODE_SIZE_PES / 2 * sin(radian));
          obstacleinfo.peek[2].y =
              ite_Obsinfo->position_y * 1 - (MODE_SIZE_PES / 2 * sin(radian)) - (MODE_SIZE_PES / 2 * cos(radian));
          obstacleinfo.peek[3].x =
              ite_Obsinfo->position_x * 1 - (MODE_SIZE_PES / 2 * cos(radian)) - (MODE_SIZE_PES / 2 * sin(radian));
          obstacleinfo.peek[3].y =
              ite_Obsinfo->position_y * 1 + (MODE_SIZE_PES / 2 * sin(radian)) + (MODE_SIZE_PES / 2 * cos(radian));
        }
        else if (ite_Obsinfo->obj_class == BICYCLE)
        {
          // ROS_INFO("BICYCLE!!!");
          // obstacleinfo.obj_class  = 3;
          obstacleinfo.peek[0].x =
              ite_Obsinfo->position_x * 1 + (MODE_SIZE_PES / 2 * cos(radian)) - (MODE_SIZE_BICYCLE / 2 * sin(radian));
          obstacleinfo.peek[0].y =
              ite_Obsinfo->position_y * 1 + (MODE_SIZE_PES / 2 * sin(radian)) + (MODE_SIZE_BICYCLE / 2 * cos(radian));
          obstacleinfo.peek[1].x =
              ite_Obsinfo->position_x * 1 + (MODE_SIZE_PES / 2 * cos(radian)) + (MODE_SIZE_BICYCLE / 2 * sin(radian));
          obstacleinfo.peek[1].y =
              ite_Obsinfo->position_y * 1 + (MODE_SIZE_PES / 2 * sin(radian)) - (MODE_SIZE_BICYCLE / 2 * cos(radian));
          obstacleinfo.peek[2].x =
              ite_Obsinfo->position_x * 1 - (MODE_SIZE_PES / 2 * cos(radian)) + (MODE_SIZE_BICYCLE / 2 * sin(radian));
          obstacleinfo.peek[2].y =
              ite_Obsinfo->position_y * 1 - (MODE_SIZE_PES / 2 * sin(radian)) - (MODE_SIZE_BICYCLE / 2 * cos(radian));
          obstacleinfo.peek[3].x =
              ite_Obsinfo->position_x * 1 - (MODE_SIZE_PES / 2 * cos(radian)) - (MODE_SIZE_BICYCLE / 2 * sin(radian));
          obstacleinfo.peek[3].y =
              ite_Obsinfo->position_y * 1 + (MODE_SIZE_PES / 2 * sin(radian)) + (MODE_SIZE_BICYCLE / 2 * cos(radian));
        }
        else if (ite_Obsinfo->obj_class == ROADBLOCK)
        {
          // ROS_INFO("ROADBLOCK!!!");
          // obstacleinfo.obj_class  = 4;
          obstacleinfo.peek[0].x =
              ite_Obsinfo->position_x * 1 + (MODE_SIZE_PES / 2 * cos(radian)) - (MODE_SIZE_ROADBLOCK / 2 * sin(radian));
          obstacleinfo.peek[0].y =
              ite_Obsinfo->position_y * 1 + (MODE_SIZE_PES / 2 * sin(radian)) + (MODE_SIZE_ROADBLOCK / 2 * cos(radian));
          obstacleinfo.peek[1].x =
              ite_Obsinfo->position_x * 1 + (MODE_SIZE_PES / 2 * cos(radian)) + (MODE_SIZE_ROADBLOCK / 2 * sin(radian));
          obstacleinfo.peek[1].y =
              ite_Obsinfo->position_y * 1 + (MODE_SIZE_PES / 2 * sin(radian)) - (MODE_SIZE_ROADBLOCK / 2 * cos(radian));
          obstacleinfo.peek[2].x =
              ite_Obsinfo->position_x * 1 - (MODE_SIZE_PES / 2 * cos(radian)) + (MODE_SIZE_ROADBLOCK / 2 * sin(radian));
          obstacleinfo.peek[2].y =
              ite_Obsinfo->position_y * 1 - (MODE_SIZE_PES / 2 * sin(radian)) - (MODE_SIZE_ROADBLOCK / 2 * cos(radian));
          obstacleinfo.peek[3].x =
              ite_Obsinfo->position_x * 1 - (MODE_SIZE_PES / 2 * cos(radian)) - (MODE_SIZE_ROADBLOCK / 2 * sin(radian));
          obstacleinfo.peek[3].y =
              ite_Obsinfo->position_y * 1 + (MODE_SIZE_PES / 2 * sin(radian)) + (MODE_SIZE_ROADBLOCK / 2 * cos(radian));
        }
        obstacleinfo.state[0] = min(obstacleinfo.peek[0].x,obstacleinfo.peek[2].x);
        obstacleinfo.state[1] = min(obstacleinfo.peek[0].y,obstacleinfo.peek[1].y);
        obstacleinfo.state[2] = max(obstacleinfo.peek[0].x,obstacleinfo.peek[2].x);
        obstacleinfo.state[3] = max(obstacleinfo.peek[0].y,obstacleinfo.peek[1].y);
        obstacleinfo.state[4] = ite_Obsinfo->velocity;
        obstacleinfo.state[0] = 0;
        if (ite_Obsinfo->obj_class != NONE_OBJECT)
        {
          perceptionobstacleinfo.object_list.push_back(obstacleinfo);
        }
        //        ROS_INFO("perceptionobstacleinfo.obstacles size:%d",perceptionobstacleinfo.obstacles.size());
        ite_Obsinfo++;
      }
      obstacle_pub.publish(perceptionobstacleinfo);
      cameraObjectData_ready_1.cameraobjectinfo.clear();
      // ROS_INFO("cameraObjectData_ready_1.cameraobjectinfo.clear()");
      cameraObjectData_ready_1.bReady = false;
    }
    mtx_camera_ObjectData1.unlock();
    mtx_camera_ObjectData2.lock();
    if (cameraObjectData_ready_2.bReady) // object is ready to topic
    {
      // ROS_INFO("cameraObjectData_2 is ready to topic!!!");
      perception_sensor_msgs::ObjectList perceptionobstacleinfo;
      common_msgs::DetectionInfo obstacleinfo;
      perceptionobstacleinfo.header.stamp            = ros::Time::now();
      perceptionobstacleinfo.header.frame_id         = "base_link";
      perceptionobstacleinfo.sensor_type             = 0;
      perceptionobstacleinfo.sensor_index            = cameraObjectData_ready_2.sensor_index;
      perceptionobstacleinfo.obstacle_num            = cameraObjectData_ready_2.count;
      list< cameraObjectInfo >::iterator ite_Obsinfo = cameraObjectData_ready_2.cameraobjectinfo.begin();
      // ROS_INFO_STREAM("cameraObjectData_2.cameraobjectinfo.size():" <<
      // cameraObjectData_ready_2.cameraobjectinfo.size());
      for (int i = 0; i < cameraObjectData_ready_2.cameraobjectinfo.size(); ++i)
      {
        // obstacleinfo.id         = i;
        obstacleinfo.id         = ite_Obsinfo->object_id;
        obstacleinfo.obj_class  = ite_Obsinfo->obj_class;
        obstacleinfo.confidence = ite_Obsinfo->confidence;
        // obstacleinfo.state[0]   = ite_Obsinfo->position_x;
        // obstacleinfo.state[1]   = ite_Obsinfo->position_y;
        // obstacleinfo.state[2]   = 0;
        // obstacleinfo.state[3]   = ite_Obsinfo->velocity;
        // radian = obstacleinfo.theta.data * M_PI/180;
        radian = 0 * M_PI / 180;
        for (int j = 0; j < 16; ++j)
        {
          // obstacleinfo.measurement_cov.push_back(measurement_cov[j]);
          obstacleinfo.measurement_cov[j] = measurement_cov[j];
        }
        // ROS_INFO_STREAM("cameraObjectData_2.cameraobjectinfo.size():" <<
        // cameraObjectData_ready_2.cameraobjectinfo.size() << ",i:%d" << i);
        if (ite_Obsinfo->obj_class == CAR)
        {
          // ROS_INFO("CAR!!!");
          obstacleinfo.peek[0].x =
              ite_Obsinfo->position_x * 1 + (MODE_SIZE_PES / 2 * cos(radian)) - (MODE_SIZE_CAR / 2 * sin(radian));
          obstacleinfo.peek[0].y =
              ite_Obsinfo->position_y * 1 + (MODE_SIZE_PES / 2 * sin(radian)) + (MODE_SIZE_CAR / 2 * cos(radian));
          obstacleinfo.peek[1].x =
              ite_Obsinfo->position_x * 1 + (MODE_SIZE_PES / 2 * cos(radian)) + (MODE_SIZE_CAR / 2 * sin(radian));
          obstacleinfo.peek[1].y =
              ite_Obsinfo->position_y * 1 + (MODE_SIZE_PES / 2 * sin(radian)) - (MODE_SIZE_CAR / 2 * cos(radian));
          obstacleinfo.peek[2].x =
              ite_Obsinfo->position_x * 1 - (MODE_SIZE_PES / 2 * cos(radian)) + (MODE_SIZE_CAR / 2 * sin(radian));
          obstacleinfo.peek[2].y =
              ite_Obsinfo->position_y * 1 - (MODE_SIZE_PES / 2 * sin(radian)) - (MODE_SIZE_CAR / 2 * cos(radian));
          obstacleinfo.peek[3].x =
              ite_Obsinfo->position_x * 1 - (MODE_SIZE_PES / 2 * cos(radian)) - (MODE_SIZE_CAR / 2 * sin(radian));
          obstacleinfo.peek[3].y =
              ite_Obsinfo->position_y * 1 + (MODE_SIZE_PES / 2 * sin(radian)) + (MODE_SIZE_CAR / 2 * cos(radian));
        }
        else if (ite_Obsinfo->obj_class == PES)
        {
          // ROS_INFO("PES!!!");
          obstacleinfo.peek[0].x =
              ite_Obsinfo->position_x * 1 + (MODE_SIZE_PES / 2 * cos(radian)) - (MODE_SIZE_PES / 2 * sin(radian));
          obstacleinfo.peek[0].y =
              ite_Obsinfo->position_y * 1 + (MODE_SIZE_PES / 2 * sin(radian)) + (MODE_SIZE_PES / 2 * cos(radian));
          obstacleinfo.peek[1].x =
              ite_Obsinfo->position_x * 1 + (MODE_SIZE_PES / 2 * cos(radian)) + (MODE_SIZE_PES / 2 * sin(radian));
          obstacleinfo.peek[1].y =
              ite_Obsinfo->position_y * 1 + (MODE_SIZE_PES / 2 * sin(radian)) - (MODE_SIZE_PES / 2 * cos(radian));
          obstacleinfo.peek[2].x =
              ite_Obsinfo->position_x * 1 - (MODE_SIZE_PES / 2 * cos(radian)) + (MODE_SIZE_PES / 2 * sin(radian));
          obstacleinfo.peek[2].y =
              ite_Obsinfo->position_y * 1 - (MODE_SIZE_PES / 2 * sin(radian)) - (MODE_SIZE_PES / 2 * cos(radian));
          obstacleinfo.peek[3].x =
              ite_Obsinfo->position_x * 1 - (MODE_SIZE_PES / 2 * cos(radian)) - (MODE_SIZE_PES / 2 * sin(radian));
          obstacleinfo.peek[3].y =
              ite_Obsinfo->position_y * 1 + (MODE_SIZE_PES / 2 * sin(radian)) + (MODE_SIZE_PES / 2 * cos(radian));
        }
        else if (ite_Obsinfo->obj_class == BICYCLE)
        {
          // ROS_INFO("BICYCLE!!!");
          obstacleinfo.peek[0].x =
              ite_Obsinfo->position_x * 1 + (MODE_SIZE_PES / 2 * cos(radian)) - (MODE_SIZE_BICYCLE / 2 * sin(radian));
          obstacleinfo.peek[0].y =
              ite_Obsinfo->position_y * 1 + (MODE_SIZE_PES / 2 * sin(radian)) + (MODE_SIZE_BICYCLE / 2 * cos(radian));
          obstacleinfo.peek[1].x =
              ite_Obsinfo->position_x * 1 + (MODE_SIZE_PES / 2 * cos(radian)) + (MODE_SIZE_BICYCLE / 2 * sin(radian));
          obstacleinfo.peek[1].y =
              ite_Obsinfo->position_y * 1 + (MODE_SIZE_PES / 2 * sin(radian)) - (MODE_SIZE_BICYCLE / 2 * cos(radian));
          obstacleinfo.peek[2].x =
              ite_Obsinfo->position_x * 1 - (MODE_SIZE_PES / 2 * cos(radian)) + (MODE_SIZE_BICYCLE / 2 * sin(radian));
          obstacleinfo.peek[2].y =
              ite_Obsinfo->position_y * 1 - (MODE_SIZE_PES / 2 * sin(radian)) - (MODE_SIZE_BICYCLE / 2 * cos(radian));
          obstacleinfo.peek[3].x =
              ite_Obsinfo->position_x * 1 - (MODE_SIZE_PES / 2 * cos(radian)) - (MODE_SIZE_BICYCLE / 2 * sin(radian));
          obstacleinfo.peek[3].y =
              ite_Obsinfo->position_y * 1 + (MODE_SIZE_PES / 2 * sin(radian)) + (MODE_SIZE_BICYCLE / 2 * cos(radian));
        }
        else if (ite_Obsinfo->obj_class == ROADBLOCK)
        {
          // ROS_INFO("ROADBLOCK!!!");
          obstacleinfo.peek[0].x =
              ite_Obsinfo->position_x * 1 + (MODE_SIZE_PES / 2 * cos(radian)) - (MODE_SIZE_ROADBLOCK / 2 * sin(radian));
          obstacleinfo.peek[0].y =
              ite_Obsinfo->position_y * 1 + (MODE_SIZE_PES / 2 * sin(radian)) + (MODE_SIZE_ROADBLOCK / 2 * cos(radian));
          obstacleinfo.peek[1].x =
              ite_Obsinfo->position_x * 1 + (MODE_SIZE_PES / 2 * cos(radian)) + (MODE_SIZE_ROADBLOCK / 2 * sin(radian));
          obstacleinfo.peek[1].y =
              ite_Obsinfo->position_y * 1 + (MODE_SIZE_PES / 2 * sin(radian)) - (MODE_SIZE_ROADBLOCK / 2 * cos(radian));
          obstacleinfo.peek[2].x =
              ite_Obsinfo->position_x * 1 - (MODE_SIZE_PES / 2 * cos(radian)) + (MODE_SIZE_ROADBLOCK / 2 * sin(radian));
          obstacleinfo.peek[2].y =
              ite_Obsinfo->position_y * 1 - (MODE_SIZE_PES / 2 * sin(radian)) - (MODE_SIZE_ROADBLOCK / 2 * cos(radian));
          obstacleinfo.peek[3].x =
              ite_Obsinfo->position_x * 1 - (MODE_SIZE_PES / 2 * cos(radian)) - (MODE_SIZE_ROADBLOCK / 2 * sin(radian));
          obstacleinfo.peek[3].y =
              ite_Obsinfo->position_y * 1 + (MODE_SIZE_PES / 2 * sin(radian)) + (MODE_SIZE_ROADBLOCK / 2 * cos(radian));
        }
        obstacleinfo.state[0] = min(obstacleinfo.peek[0].x,obstacleinfo.peek[2].x);
        obstacleinfo.state[1] = min(obstacleinfo.peek[0].y,obstacleinfo.peek[1].y);
        obstacleinfo.state[2] = max(obstacleinfo.peek[0].x,obstacleinfo.peek[2].x);
        obstacleinfo.state[3] = max(obstacleinfo.peek[0].y,obstacleinfo.peek[1].y);
        obstacleinfo.state[4] = ite_Obsinfo->velocity;
        obstacleinfo.state[0] = 0;
        if (ite_Obsinfo->obj_class != NONE_OBJECT)
        {
          perceptionobstacleinfo.object_list.push_back(obstacleinfo);
        }
        //        ROS_INFO("perceptionobstacleinfo.obstacles size:%d",perceptionobstacleinfo.obstacles.size());
        ite_Obsinfo++;
      }
      obstacle_pub.publish(perceptionobstacleinfo);
      cameraObjectData_ready_2.cameraobjectinfo.clear();
      // ROS_INFO("cameraObjectData_ready_2.cameraobjectinfo.clear()");
      cameraObjectData_ready_2.bReady = false;
    }
    mtx_camera_ObjectData2.unlock();
    mtx_camera_LaneData1.lock();
    if (cameraLaneData_ready_1.bReady) // lane is ready to topic
    {
      // ROS_INFO("cameraLaneData_1 is ready to topic!!!");
      visualization_msgs::MarkerArray perceptionlaneinfo;
      //      visualization_msgs::Marker laneinfo;
      geometry_msgs::Point lanepoint;
      cameraLanePoint cameraLanePoint_;
      list< cameraLaneInfo >::iterator ite_laneinfo = cameraLaneData_ready_1.cameralaneinfo.begin();
      // ROS_INFO_STREAM("cameraLaneData_1.cameralaneinfo.size():" << cameraLaneData_ready_1.cameralaneinfo.size());
      for (int i = 0; i < cameraLaneData_ready_1.cameralaneinfo.size(); ++i)
      {
        visualization_msgs::Marker laneinfo;
        laneinfo.header.stamp    = ros::Time::now();
        laneinfo.header.frame_id = "base_link";
        lanepoint.y              = ite_laneinfo->start_point;
        lanepoint.x              = ite_laneinfo->polynomial_a * pow(lanepoint.y, 3) +
                      ite_laneinfo->polynomial_b * pow(lanepoint.y, 2) + ite_laneinfo->polynomial_c * lanepoint.y +
                      ite_laneinfo->polynomial_d;
        lanepoint.z = 0;
        // ROS_INFO_STREAM("start:" << lanepoint.y << "," << lanepoint.x);
        laneinfo.points.push_back(lanepoint);
        while (lanepoint.y + THETA_Y < ite_laneinfo->end_point)
        {
          lanepoint.y = lanepoint.y + THETA_Y;
          lanepoint.x = ite_laneinfo->polynomial_a * pow(lanepoint.y, 3) +
                        ite_laneinfo->polynomial_b * pow(lanepoint.y, 2) + ite_laneinfo->polynomial_c * lanepoint.y +
                        ite_laneinfo->polynomial_d;
          laneinfo.points.push_back(lanepoint);
          // ROS_INFO_STREAM("point:" << lanepoint.y << "," << lanepoint.x);
        }
        lanepoint.y = ite_laneinfo->end_point;
        lanepoint.x = ite_laneinfo->polynomial_a * pow(lanepoint.y, 3) +
                      ite_laneinfo->polynomial_b * pow(lanepoint.y, 2) + ite_laneinfo->polynomial_c * lanepoint.y +
                      ite_laneinfo->polynomial_d;
        lanepoint.z = 0;
        laneinfo.points.push_back(lanepoint);
        // ROS_INFO_STREAM("end:" << lanepoint.y << "," << lanepoint.x);
        perceptionlaneinfo.markers.push_back(laneinfo);
        ite_laneinfo++;
      }
      lane_pub.publish(perceptionlaneinfo);
      cameraLaneData_ready_1.cameralaneinfo.clear();
      cameraLaneData_ready_1.bReady = false;
    }
    mtx_camera_LaneData1.unlock();
    mtx_camera_LaneData2.lock();
    if (cameraLaneData_ready_2.bReady) // lane is ready to topic
    {
      // ROS_INFO("cameraLaneData_2 is ready to topic!!!");
      visualization_msgs::MarkerArray perceptionlaneinfo;
      //      visualization_msgs::Marker laneinfo;
      geometry_msgs::Point lanepoint;
      cameraLanePoint cameraLanePoint_;
      list< cameraLaneInfo >::iterator ite_laneinfo = cameraLaneData_ready_2.cameralaneinfo.begin();
      // ROS_INFO_STREAM("cameraLaneData_2.cameralaneinfo.size():" << cameraLaneData_ready_2.cameralaneinfo.size());
      // ROS_INFO("cameraLaneData_2.cameralaneinfo.size():%u",cameraLaneData_2.cameralaneinfo.size());
      for (int i = 0; i < cameraLaneData_ready_2.cameralaneinfo.size(); ++i)
      {
        visualization_msgs::Marker laneinfo;
        laneinfo.header.stamp    = ros::Time::now();
        laneinfo.header.frame_id = "base_link";
        lanepoint.y              = ite_laneinfo->start_point;
        lanepoint.x              = ite_laneinfo->polynomial_a * pow(lanepoint.y, 3) +
                      ite_laneinfo->polynomial_b * pow(lanepoint.y, 2) + ite_laneinfo->polynomial_c * lanepoint.y +
                      ite_laneinfo->polynomial_d;
        lanepoint.z = 0;
        laneinfo.points.push_back(lanepoint);
        while (lanepoint.y + THETA_Y < ite_laneinfo->end_point)
        {
          lanepoint.y = lanepoint.y + THETA_Y;
          lanepoint.x = ite_laneinfo->polynomial_a * pow(lanepoint.y, 3) +
                        ite_laneinfo->polynomial_b * pow(lanepoint.y, 2) + ite_laneinfo->polynomial_c * lanepoint.y +
                        ite_laneinfo->polynomial_d;
          laneinfo.points.push_back(lanepoint);
        }
        lanepoint.y = ite_laneinfo->end_point;
        lanepoint.x = ite_laneinfo->polynomial_a * pow(lanepoint.y, 3) +
                      ite_laneinfo->polynomial_b * pow(lanepoint.y, 2) + ite_laneinfo->polynomial_c * lanepoint.y +
                      ite_laneinfo->polynomial_d;
        lanepoint.z = 0;
        laneinfo.points.push_back(lanepoint);
        perceptionlaneinfo.markers.push_back(laneinfo);
        ite_laneinfo++;
      }
      lane_pub.publish(perceptionlaneinfo);
      cameraLaneData_ready_2.cameralaneinfo.clear();
      cameraLaneData_ready_2.bReady = false;
    }
    mtx_camera_LaneData2.unlock();
    mtx_camera_matData1.lock();
    if (matData_ready_1.bReady)
    {
      // ROS_INFO("matData_1 is ready to topic!!!");
      sensor_msgs::Image image;
      image.header.stamp                    = ros::Time::now();
      image.header.frame_id                 = "base_link";
      list< uint8_t >::iterator ite_matinfo = matData_ready_1.data.begin();
      for (ite_matinfo; ite_matinfo != matData_ready_1.data.end(); ++ite_matinfo)
      {
        image.data.push_back(*ite_matinfo);
      }
      mat_pub.publish(image);
      matData_ready_1.data.clear();
      matData_ready_1.bReady = false;
    }
    mtx_camera_matData1.unlock();
    mtx_camera_matData2.lock();
    if (matData_ready_2.bReady)
    {
      // ROS_INFO("matData_1 is ready to topic!!!");
      sensor_msgs::Image image;
      image.header.stamp                    = ros::Time::now();
      image.header.frame_id                 = "base_link";
      image.height                          = MAT_HEIGHT;
      image.width                           = MAT_WIDTH;
      list< uint8_t >::iterator ite_matinfo = matData_ready_2.data.begin();
      for (ite_matinfo; ite_matinfo != matData_ready_2.data.end(); ++ite_matinfo)
      {
        image.data.push_back(*ite_matinfo);
      }
      mat_pub.publish(image);
      matData_ready_2.data.clear();
      matData_ready_2.bReady = false;
    }
    mtx_camera_matData2.unlock();
    //    ROS_INFO("camera_Sender to topic!");
  }
}

void CAMERA::CANCameraWrite(int &dev_, int &ch_)
{
  list_camera_wirte_task.push_back(ALL_GetStatus);
  ros::Rate rate_loop(10);
  while(ros::ok())
  {
    // if(adcuDevStatus(dev_) == ADCU_DEV_STATUS_ABNORMAL)
    // {
    //   ROS_INFO("Can %d error", ch_);
    //   break;
    // }
    for(list<Camera_Task>::iterator ite_list_camera_wirte_task = list_camera_wirte_task.begin();ite_list_camera_wirte_task != list_camera_wirte_task.end();++ite_list_camera_wirte_task)
    {
      ROS_INFO_STREAM("ite_list_camera_wirte_task:" << *ite_list_camera_wirte_task);
      switch(*ite_list_camera_wirte_task)
      {
        case ALL_StartWork:
          if(writeCANOnce(dev_, ALL_START_WORK, 0, 8))
          {
            
          }
          else
          {
            ROS_INFO("ALL_START_WORK send faild");
          }
          break;
        case ALL_GetStatus:
          if(writeCANOnce(dev_, ALL_GET_STATUS, 0, 8))
          {
            list_camera_read_task.push_back(Wait_AllStatus);
          }
          else
          {
            ROS_INFO("ALL_GetStatus send faild");
          }
          break;
        case ALL_StopWork:
          if(writeCANOnce(dev_, ALL_STOP_ACQUIRE, 0, 8))
          {

          }
          else
          {
            ROS_INFO("ALL_STOP_ACQUIRE send faild");
          }
          break;
        case Object_StopWork:

          break;
        case Object_StartWork:

          break;
        case Object_GetWork:

          break;
        default:
          break;
      }
    }
    list_camera_wirte_task.clear();
    rate_loop.sleep();
  }
}

void CAMERA::objectHeadParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_,
                             uint16_t &contype)
{
  if (can_order.reserve != 1)
  {
    // ROS_INFO("recv: %d objectHeadParse", can_id);
  }
  ObjectStrHead object_str_head_parser;
  object_str_head_parser.obj_num = can_buf.can_data[0];
  uint32_t value_temp_;
  value_temp_ = ((uint32_t)(can_buf.can_data[1]) << 19) + ((uint32_t)(can_buf.can_data[2]) << 11) +
                ((uint32_t)(can_buf.can_data[3]) << 3) + ((uint32_t)(can_buf.can_data[4]) >> 5 & 0x07);
  object_str_head_parser.r_time = value_temp_;

  array_camera_ObjectData[can_id%100/10].count = object_str_head_parser.obj_num;
  array_camera_ObjectData[can_id%100/10].time = object_str_head_parser.r_time;
  if (can_order.reserve != 1)
  {
    // ROS_INFO("recv:%u obj time:%u", array_camera_ObjectData[can_id%100/10].count, array_camera_ObjectData[can_id%100/10].time);
  }
}
void CAMERA::objectDataFirstParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_,
                                  uint16_t &contype)
{
  if (can_order.reserve != 1)
  {
    // ROS_INFO("recv: %d objectDataFirstParse", can_id);
  }
  //提取
  ObjectStr0 object_str_0_parser;
  object_str_0_parser.id         = can_buf.can_data[0];
  object_str_0_parser.obj_class  = can_buf.can_data[1];
  object_str_0_parser.confidence = can_buf.can_data[2];
  object_str_0_parser.position_x = (can_buf.can_data[5] << 4) | (can_buf.can_data[6] >> 4 & 0x0F);
  object_str_0_parser.position_y = ((can_buf.can_data[6] & 0x0F) << 8) | can_buf.can_data[7];
  object_str_0_parser.velocity   = (can_buf.can_data[3] << 2) | (can_buf.can_data[4] >> 6 & 0x03);
  //赋值
  if (ch_ == CHANNEL_CAMERA_1)
  {
    cameraObjectInfo_1.object_id  = object_str_0_parser.id;
    cameraObjectInfo_1.obj_class  = class_enum(object_str_0_parser.obj_class);
    cameraObjectInfo_1.position_x = object_str_0_parser.position_x * 0.1;
    cameraObjectInfo_1.position_y = object_str_0_parser.position_y * 0.1;
    cameraObjectInfo_1.velocity   = object_str_0_parser.velocity * 0.1;
    cameraObjectInfo_1.confidence = ( float )object_str_0_parser.confidence * 0.01;
    if (can_order.reserve != 1)
    {
      switch(can_id)
      {
        case 211:
          if(cameraObjectInfo_1.position_y < 7.5)
          {
            ROS_INFO_STREAM("can_id:" << can_id << ",data:" << cameraObjectInfo_1.object_id << " (" << cameraObjectInfo_1.position_x
                      << "," << cameraObjectInfo_1.position_y << ") " << cameraObjectInfo_1.velocity
                      << "m/s con:" << cameraObjectInfo_1.confidence << "class:" << object_str_0_parser.obj_class);
          }
          else
          {
            ROS_INFO_STREAM("can_id:" << can_id << " is ok");
          }
          break;
        case 231:
          if(cameraObjectInfo_1.position_y > -7.5)
          {
            ROS_INFO_STREAM("can_id:" << can_id << ",data:" << cameraObjectInfo_1.object_id << " (" << cameraObjectInfo_1.position_x
                      << "," << cameraObjectInfo_1.position_y << ") " << cameraObjectInfo_1.velocity
                      << "m/s con:" << cameraObjectInfo_1.confidence << "class:" << object_str_0_parser.obj_class);
          }
          else
          {
            ROS_INFO_STREAM("can_id:" << can_id << " is ok");
          }
          break;
        case 251:
          if(cameraObjectInfo_1.position_x < 1.1)
          {
            ROS_INFO_STREAM("can_id:" << can_id << ",data:" << cameraObjectInfo_1.object_id << " (" << cameraObjectInfo_1.position_x
                      << "," << cameraObjectInfo_1.position_y << ") " << cameraObjectInfo_1.velocity
                      << "m/s con:" << cameraObjectInfo_1.confidence << "class:" << object_str_0_parser.obj_class);
          }
          else
          {
            ROS_INFO_STREAM("can_id:" << can_id << " is ok");
          }
          break;
        case 261:
          if(cameraObjectInfo_1.position_x < 1.1)
          {
            ROS_INFO_STREAM("can_id:" << can_id << ",data:" << cameraObjectInfo_1.object_id << " (" << cameraObjectInfo_1.position_x
                      << "," << cameraObjectInfo_1.position_y << ") " << cameraObjectInfo_1.velocity
                      << "m/s con:" << cameraObjectInfo_1.confidence << "class:" << object_str_0_parser.obj_class);
          }
          else
          {
            ROS_INFO_STREAM("can_id:" << can_id << " is ok");
          }
          break;
        case 271:
          if(cameraObjectInfo_1.position_x > -1.1)
          {
            ROS_INFO_STREAM("can_id:" << can_id << ",data:" << cameraObjectInfo_1.object_id << " (" << cameraObjectInfo_1.position_x
                      << "," << cameraObjectInfo_1.position_y << ") " << cameraObjectInfo_1.velocity
                      << "m/s con:" << cameraObjectInfo_1.confidence << "class:" << object_str_0_parser.obj_class);
          }
          else
          {
            ROS_INFO_STREAM("can_id:" << can_id << " is ok");
          }
          break;
        case 281:
          if(cameraObjectInfo_1.position_x < -1.1)
          {
            ROS_INFO_STREAM("can_id:" << can_id << ",data:" << cameraObjectInfo_1.object_id << " (" << cameraObjectInfo_1.position_x
                      << "," << cameraObjectInfo_1.position_y << ") " << cameraObjectInfo_1.velocity
                      << "m/s con:" << cameraObjectInfo_1.confidence << "class:" << object_str_0_parser.obj_class);
          }
          else
          {
            ROS_INFO_STREAM("can_id:" << can_id << " is ok");
          }
          break;
        default:
          break;
      }
      ROS_INFO_STREAM("can_id:" << can_id << ",data:" << cameraObjectInfo_1.object_id << " (" << cameraObjectInfo_1.position_x
                      << "," << cameraObjectInfo_1.position_y << ") " << cameraObjectInfo_1.velocity
                      << "m/s con:" << cameraObjectInfo_1.confidence << "class:" << object_str_0_parser.obj_class);
    }
  }
  else if (ch_ == CHANNEL_CAMERA_2)
  {
    cameraObjectInfo_2.object_id  = object_str_0_parser.id;
    cameraObjectInfo_2.obj_class  = class_enum(object_str_0_parser.obj_class);
    cameraObjectInfo_2.position_x = object_str_0_parser.position_x * 0.1;
    cameraObjectInfo_2.position_y = object_str_0_parser.position_y * 0.1;
    cameraObjectInfo_2.velocity   = object_str_0_parser.velocity * 0.1;
    cameraObjectInfo_2.confidence = ( float )object_str_0_parser.confidence * 0.01;
    if (can_order.reserve != 1)
    {
      switch(can_id)
      {
        case 211:
          if(cameraObjectInfo_2.position_y < 7.5)
          {
            ROS_INFO_STREAM("can_id:" << can_id << ",data:" << cameraObjectInfo_2.object_id << " (" << cameraObjectInfo_2.position_x
                      << "," << cameraObjectInfo_2.position_y << ") " << cameraObjectInfo_2.velocity
                      << "m/s con:" << cameraObjectInfo_2.confidence << "class:" << object_str_0_parser.obj_class);
          }
          else
          {
            ROS_INFO_STREAM("can_id:" << can_id << " is ok");
          }
          break;
        case 231:
          if(cameraObjectInfo_2.position_y > -7.5)
          {
            ROS_INFO_STREAM("can_id:" << can_id << ",data:" << cameraObjectInfo_2.object_id << " (" << cameraObjectInfo_2.position_x
                      << "," << cameraObjectInfo_2.position_y << ") " << cameraObjectInfo_2.velocity
                      << "m/s con:" << cameraObjectInfo_2.confidence << "class:" << object_str_0_parser.obj_class);
          }
          else
          {
            ROS_INFO_STREAM("can_id:" << can_id << " is ok");
          }
          break;
        case 251:
          if(cameraObjectInfo_2.position_x < 1.1)
          {
            ROS_INFO_STREAM("can_id:" << can_id << ",data:" << cameraObjectInfo_2.object_id << " (" << cameraObjectInfo_2.position_x
                      << "," << cameraObjectInfo_2.position_y << ") " << cameraObjectInfo_2.velocity
                      << "m/s con:" << cameraObjectInfo_2.confidence << "class:" << object_str_0_parser.obj_class);
          }
          else
          {
            ROS_INFO_STREAM("can_id:" << can_id << " is ok");
          }
          break;
        case 261:
          if(cameraObjectInfo_2.position_x < 1.1)
          {
            ROS_INFO_STREAM("can_id:" << can_id << ",data:" << cameraObjectInfo_2.object_id << " (" << cameraObjectInfo_2.position_x
                      << "," << cameraObjectInfo_2.position_y << ") " << cameraObjectInfo_2.velocity
                      << "m/s con:" << cameraObjectInfo_2.confidence << "class:" << object_str_0_parser.obj_class);
          }
          else
          {
            ROS_INFO_STREAM("can_id:" << can_id << " is ok");
          }
          break;
        case 271:
          if(cameraObjectInfo_2.position_x > -1.1)
          {
            ROS_INFO_STREAM("can_id:" << can_id << ",data:" << cameraObjectInfo_2.object_id << " (" << cameraObjectInfo_2.position_x
                      << "," << cameraObjectInfo_2.position_y << ") " << cameraObjectInfo_2.velocity
                      << "m/s con:" << cameraObjectInfo_2.confidence << "class:" << object_str_0_parser.obj_class);
          }
          else
          {
            ROS_INFO_STREAM("can_id:" << can_id << " is ok");
          }
          break;
        case 281:
          if(cameraObjectInfo_2.position_x < -1.1)
          {
            ROS_INFO_STREAM("can_id:" << can_id << ",data:" << cameraObjectInfo_2.object_id << " (" << cameraObjectInfo_2.position_x
                      << "," << cameraObjectInfo_2.position_y << ") " << cameraObjectInfo_2.velocity
                      << "m/s con:" << cameraObjectInfo_2.confidence << "class:" << object_str_0_parser.obj_class);
          }
          else
          {
            ROS_INFO_STREAM("can_id:" << can_id << " is ok");
          }
          break;
        default:
          break;
      }
      ROS_INFO_STREAM("can_id:" << can_id << ",data:" << cameraObjectInfo_2.object_id << " (" << cameraObjectInfo_2.position_x
                      << "," << cameraObjectInfo_2.position_y << ") " << cameraObjectInfo_2.velocity
                      << "m/s con:" << cameraObjectInfo_2.confidence << "class:" << object_str_0_parser.obj_class);
    }
  }
}
void CAMERA::objectDataSecondParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_,
                                   uint16_t &contype)
{
  if (can_order.reserve != 1)
  {
    // ROS_INFO("recv: %d objectDataSecondParse", can_id);
  }
  //提取
  ObjectStr1 object_str_1_parser;
  object_str_1_parser.id            = can_buf.can_data[0];
  object_str_1_parser.width         = (can_buf.can_data[1] << 4) | (can_buf.can_data[2] >> 4 & 0x0F);
  object_str_1_parser.polygon_x_min = (can_buf.can_data[5] << 4) | (can_buf.can_data[6] >> 4 & 0x0F);
  object_str_1_parser.polygon_x_max = (can_buf.can_data[6] & 0x0F) << 8 | (can_buf.can_data[7]);
  object_str_1_parser.polygon_y_min = (can_buf.can_data[2] & 0x0F) << 6 | (can_buf.can_data[3] >> 2 & 0xCF);
  object_str_1_parser.polygon_y_max = (can_buf.can_data[3] & 0x03) << 8 | can_buf.can_data[4];
  //赋值
  if (ch_ == CHANNEL_CAMERA_1)
  {
    cameraObjectInfo_1.object_id     = object_str_1_parser.id;
    cameraObjectInfo_1.width         = object_str_1_parser.width * 0.1;
    cameraObjectInfo_1.polygon_x_min = object_str_1_parser.polygon_x_min;
    cameraObjectInfo_1.polygon_x_max = object_str_1_parser.polygon_x_max;
    cameraObjectInfo_1.polygon_y_min = object_str_1_parser.polygon_y_min;
    cameraObjectInfo_1.polygon_y_max = object_str_1_parser.polygon_y_max;
  }
  else if (ch_ == CHANNEL_CAMERA_2)
  {
    cameraObjectInfo_2.object_id     = object_str_1_parser.id;
    cameraObjectInfo_2.width         = object_str_1_parser.width * 0.1;
    cameraObjectInfo_2.polygon_x_min = object_str_1_parser.polygon_x_min;
    cameraObjectInfo_2.polygon_x_max = object_str_1_parser.polygon_x_max;
    cameraObjectInfo_2.polygon_y_min = object_str_1_parser.polygon_y_min;
    cameraObjectInfo_2.polygon_y_max = object_str_1_parser.polygon_y_max;
  }
  if (can_order.reserve != 1)
  {
    if (ch_ == CHANNEL_CAMERA_1)
    {
      array_camera_ObjectData[can_id%100/10].cameraobjectinfo.push_back(cameraObjectInfo_1);
      // ROS_INFO("data:%u %fm min(%u,%u) max(%u,%u)", cameraObjectInfo_1.object_id, cameraObjectInfo_1.width,
      // cameraObjectInfo_1.polygon_x_min,
      //      cameraObjectInfo_1.polygon_y_min, cameraObjectInfo_1.polygon_x_max, cameraObjectInfo_1.polygon_y_max);
    }
    else if (ch_ == CHANNEL_CAMERA_2)
    {
      array_camera_ObjectData[can_id%100/10].cameraobjectinfo.push_back(cameraObjectInfo_2);
      // ROS_INFO("data:%u %fm min(%u,%u) max(%u,%u)", cameraObjectInfo_2.object_id, cameraObjectInfo_2.width,
      // cameraObjectInfo_2.polygon_x_min,
      //      cameraObjectInfo_2.polygon_y_min, cameraObjectInfo_2.polygon_x_max, cameraObjectInfo_2.polygon_y_max);
    }
  }
}
void CAMERA::laneHeadParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype)
{
  if (can_order.reserve != 1)
  {
    // ROS_INFO("recv: %d laneHeadParse", can_id);
  }
  laneStrHead lane_str_head_parser;
  lane_str_head_parser.lane_num = can_buf.can_data[0];
  uint32_t value_temp_;
  value_temp_ = ((uint32_t)(can_buf.can_data[1]) << 19) + ((uint32_t)(can_buf.can_data[2]) << 11) +
                ((uint32_t)(can_buf.can_data[3]) << 3) + ((uint32_t)(can_buf.can_data[4]) >> 5 & 0x07);
  lane_str_head_parser.r_time = value_temp_;

  if (ch_ == CHANNEL_CAMERA_1)
  {
    array_camera_LaneData[can_id%100/10].count = lane_str_head_parser.r_time;
    array_camera_LaneData[can_id%100/10].time  = lane_str_head_parser.r_time;
    if (can_order.reserve != 1)
    {
      // ROS_INFO("recv:%u lane time:%u", array_camera_LaneData[can_id%100/10].count, array_camera_LaneData[can_id%100/10].time);
    }
  }
  else if (ch_ == CHANNEL_CAMERA_2)
  {
    array_camera_LaneData[can_id%100/10].count = lane_str_head_parser.r_time;
    array_camera_LaneData[can_id%100/10].time  = lane_str_head_parser.r_time;
    if (can_order.reserve != 1)
    {
      // ROS_INFO("recv:%u lane time:%u", array_camera_LaneData[can_id%100/10].count, array_camera_LaneData[can_id%100/10].time);
    }
  }
}
void CAMERA::laneDataFirstParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_,
                                uint16_t &contype)
{
  // ROS_INFO("recv: %d laneDataFirstParse", can_id);
  laneStr0 lane_str_0_parser;
  lane_str_0_parser.id          = can_buf.can_data[0];
  lane_str_0_parser.start_point = (can_buf.can_data[4]) << 8 | (can_buf.can_data[5]);
  lane_str_0_parser.end_point   = (can_buf.can_data[6]) << 8 | (can_buf.can_data[7]);
  lane_str_0_parser.distance    = (can_buf.can_data[2]) << 8 | (can_buf.can_data[3]);
  lane_str_0_parser.slope       = can_buf.can_data[1];

  if (ch_ == CHANNEL_CAMERA_1)
  {
    cameraLaneInfo_1.lane_id     = lane_str_0_parser.id;
    cameraLaneInfo_1.start_point = lane_str_0_parser.start_point * 0.1;
    cameraLaneInfo_1.end_point   = lane_str_0_parser.end_point * 0.1;
    cameraLaneInfo_1.distance    = lane_str_0_parser.distance * 0.1;
    cameraLaneInfo_1.slope       = lane_str_0_parser.slope;
    if (can_order.reserve != 1)
    {
      // ROS_INFO("data:%u Y-S:%fm -- %fm --> Y-E:%fm slope:%u", cameraLaneInfo_1.lane_id, cameraLaneInfo_1.start_point,
      //          cameraLaneInfo_1.distance, cameraLaneInfo_1.end_point, cameraLaneInfo_1.slope);
    }
  }
  else if (ch_ == CHANNEL_CAMERA_2)
  {
    cameraLaneInfo_2.lane_id     = lane_str_0_parser.id;
    cameraLaneInfo_2.start_point = lane_str_0_parser.start_point * 0.1;
    cameraLaneInfo_2.end_point   = lane_str_0_parser.end_point * 0.1;
    cameraLaneInfo_2.distance    = lane_str_0_parser.distance * 0.1;
    cameraLaneInfo_2.slope       = lane_str_0_parser.slope;
    if (can_order.reserve != 1)
    {
      // ROS_INFO("data:%u Y-S:%fm -- %fm --> Y-E:%fm slope:%u", cameraLaneInfo_2.lane_id, cameraLaneInfo_2.start_point,
      // cameraLaneInfo_2.distance,
      // cameraLaneInfo_2.end_point, cameraLaneInfo_2.slope);
    }
  }
}
void CAMERA::laneDataSecondParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_,
                                 uint16_t &contype)
{
  // ROS_INFO("recv: %d laneDataSecondParse", can_id);
  laneStr1 lane_str_1_parser;
  lane_str_1_parser.polynomial_a = (can_buf.can_data[0]) << 8 | (can_buf.can_data[1]);
  lane_str_1_parser.polynomial_b = (can_buf.can_data[2]) << 8 | (can_buf.can_data[3]);
  lane_str_1_parser.polynomial_c = (can_buf.can_data[4]) << 8 | (can_buf.can_data[5]);
  lane_str_1_parser.polynomial_d = (can_buf.can_data[6]) << 8 | (can_buf.can_data[7] >> 3 & 0x1F);
  lane_str_1_parser.id           = (can_buf.can_data[7] & 0x07);

  if (ch_ == CHANNEL_CAMERA_1)
  {
    cameraLaneInfo_1.polynomial_a = lane_str_1_parser.polynomial_a * 0.1;
    cameraLaneInfo_1.polynomial_b = lane_str_1_parser.polynomial_b * 0.1;
    cameraLaneInfo_1.polynomial_c = lane_str_1_parser.polynomial_c * 0.1;
    cameraLaneInfo_1.polynomial_d = lane_str_1_parser.polynomial_d * 0.1;
    cameraLaneInfo_1.lane_id      = lane_str_1_parser.id;
    if (can_order.reserve != 1)
    {
      array_camera_LaneData[can_id%100/10].cameralaneinfo.push_back(cameraLaneInfo_1);
      // ROS_INFO("data:%u p_a:%f p_b:%f p_c:%f p_d:%f", cameraLaneInfo_1.lane_id, cameraLaneInfo_1.polynomial_a,
      //          cameraLaneInfo_1.polynomial_b, cameraLaneInfo_1.polynomial_c, cameraLaneInfo_1.polynomial_d);
    }
  }
  else if (ch_ == CHANNEL_CAMERA_2)
  {
    cameraLaneInfo_2.polynomial_a = lane_str_1_parser.polynomial_a * 0.1;
    cameraLaneInfo_2.polynomial_b = lane_str_1_parser.polynomial_b * 0.1;
    cameraLaneInfo_2.polynomial_c = lane_str_1_parser.polynomial_c * 0.1;
    cameraLaneInfo_2.polynomial_d = lane_str_1_parser.polynomial_d * 0.1;
    cameraLaneInfo_2.lane_id      = lane_str_1_parser.id;
    if (can_order.reserve != 1)
    {
      array_camera_LaneData[can_id%100/10].cameralaneinfo.push_back(cameraLaneInfo_2);
      // ROS_INFO("data:%u p_a:%f p_b:%f p_c:%f p_d:%f", cameraLaneInfo_2.lane_id, cameraLaneInfo_2.polynomial_a,
      // cameraLaneInfo_2.polynomial_b,
      //       cameraLaneInfo_2.polynomial_c, cameraLaneInfo_2.polynomial_d);
    }
  }
}
void CAMERA::cameraWorkStatusParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_,
                                   uint16_t &contype)
{
  uint32_t camera_id;
  camera_id               = (can_id / 10) % 10;
  uint8_t camera_status   = can_buf.can_data[0];
  camerastatus[camera_id] = camera_status;
  if (camera_id == CAMERA_LANE_2 || camera_id == CAMERA_LANE_4)
  {
    // ROS_INFO("recv: %d lane camera %d WorkStatus is %d", can_id, camera_id, camera_status);
  }
  else
  {
    // ROS_INFO("recv: %d obj camera %d WorkStatus is %d", can_id, camera_id, camera_status);
  }
}

void CAMERA::cameraAllStartWork(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_,
                                uint16_t &contype)
{
  // ROS_INFO("cameraAllStartWork write: %d", can_id);
  can_buf.id          = can_id;
  can_buf.can_data[0] = 1; // SyncStartWork
  can_buf.can_data[1] = 0; // Fre 0:10Hz 1:15Hz 2:30Hz
}
void CAMERA::cameraAllGetStatus(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_,
                                uint16_t &contype)
{
  // ROS_INFO("cameraAllGetStatus write: %d", can_id);
  can_buf.id          = can_id;
  can_buf.can_data[0] = 1;
}
void CAMERA::cameraAllStopAcquire(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_,
                                  uint16_t &contype)
{
  // ROS_INFO("cameraAllStopAcquire write: %d", can_id);
  can_buf.id          = can_id;
  can_buf.can_data[0] = 1;
}
void CAMERA::cameraStartWork(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_,
                             uint16_t &contype)
{
  // ROS_INFO("cameraStartWork write: %d", can_id);
  can_buf.id          = can_id;
  can_buf.can_data[0] = 1;
}
void CAMERA::cameraStopWork(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype)
{
  // ROS_INFO("cameraStopWork write: %d", can_id);
  can_buf.id          = can_id;
  can_buf.can_data[0] = 1;
}

void CAMERA::matHeadParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype)
{
  if (can_order.reserve != 1)
  {
    // ROS_INFO("recv: %d matHeadParse", can_id);
  }
  can_buf.id          = can_id;
  can_buf.can_data[0] = 1; // SyncStartWork
  if (can_order.reserve != 1)
  {
    // ROS_INFO("data: %d", can_buf.can_data[0]);
  }
}

void CAMERA::matRawParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype)
{
  if (can_order.reserve != 1)
  {
    // ROS_INFO("recv: %d matRawParse", can_id);
  }
  can_buf.id          = can_id;
  can_buf.can_data[0] = 1; // SyncStartWork
  can_buf.can_data[1] = 0; // Fre 0:10Hz 1:15Hz 2:30Hz
  if (can_order.reserve != 1)
  {
    // ROS_INFO("Raw: %d", can_buf.can_data[0] << 8 | can_buf.can_data[0]);
  }
}

void CAMERA::matDataParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype)
{
  if (can_order.reserve != 1)
  {
    // ROS_INFO("recv: %d matDataParse", can_id);
  }
  if (ch_ == CHANNEL_CAMERA_1)
  {
    for (size_t loop_i = 0; loop_i < can_buf.dlc; loop_i++)
    {
      matData_1.data.push_back(can_buf.can_data[loop_i]);
    }
  }
  else if (ch_ == CHANNEL_CAMERA_2)
  {
    for (size_t loop_i = 0; loop_i < can_buf.dlc; loop_i++)
    {
      matData_2.data.push_back(can_buf.can_data[loop_i]);
    }
  }
  if (can_order.reserve != 1)
  {
    // ROS_INFO("data[0]:%u data[1]:%d data[2]:%u data[3]:%u data[4]:%u data[5]:%u data[6]:%u data[7]:%u ",
    // can_buf.can_data[0], can_buf.can_data[1],
    //  can_buf.can_data[2], can_buf.can_data[3], can_buf.can_data[4], can_buf.can_data[5], can_buf.can_data[6],
    //  can_buf.can_data[7]);
  }
}

void CAMERA::matTailParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype)
{
  if (can_order.reserve != 1)
  {
    // ROS_INFO("recv: %d matTailParse", can_id);
  }
  can_buf.id = can_id;
  if (can_order.reserve != 1)
  {
    // ROS_INFO("data: %d", can_buf.can_data[0]);
  }
}
}
}
