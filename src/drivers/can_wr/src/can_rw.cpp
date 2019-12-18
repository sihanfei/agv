#include "can_rw.h"

using namespace std;
using namespace superg_agv;
using namespace drivers;

namespace superg_agv
{
namespace drivers
{
CANData_Output::CANData_Output(string &pram1)
{
  sensor_type = pram1;
  cout << filepath << endl;
  filepath = filepath + sensor_type;
  cout << sensor_type << endl;
  cout << filepath << endl;
  min = -1;
  // filepath = path + filename + ".csv";
  // cout << filepath << endl;
} 

CANData_Output::~CANData_Output()
{
  if(logfile.is_open())
  {
    logfile.close();
  }
}

bool CANData_Output::Openfile(string &pram1, string &pram2, adcuCanData &data, int &ch_)
{
  char str[80] = {};
  time(&systime);
  ptminfo = localtime(&systime);
  string closepath = "";
  string logpath = "";
  if(ch_ == CHANNEL_CAMERA_1)
  {
    logpath = filepath + "/" + to_string(ptminfo->tm_year + 1900) + to_string(ptminfo->tm_mon + 1) + to_string(ptminfo->tm_mday) + "-" + to_string(ptminfo->tm_hour)
                  + "/" + sensor_type + "_1" + "-" + to_string(ptminfo->tm_min) + ".csv";
  }
  else if(ch_ == CHANNEL_CAMERA_2)
  {
    logpath = filepath + "/" + to_string(ptminfo->tm_year + 1900) + to_string(ptminfo->tm_mon + 1) + to_string(ptminfo->tm_mday) + "-" + to_string(ptminfo->tm_hour)
                  + "/" + sensor_type + "_2" + "-" + to_string(ptminfo->tm_min) + ".csv";
  }
  else
  {
    logpath = filepath + "/" + to_string(ptminfo->tm_year + 1900) + to_string(ptminfo->tm_mon + 1) + to_string(ptminfo->tm_mday) + "-" + to_string(ptminfo->tm_hour)
                  + "/" + sensor_type + "-" + to_string(ptminfo->tm_min) + ".csv";
  }
  if(-1 == min)
  {
    logfile.open(logpath);
    min = ptminfo->tm_min;
  }
  if(min < ptminfo->tm_min || (59 == min && 0 == ptminfo->tm_min))
  {
    logfile.close();
    logfile.open(logpath);
    min = ptminfo->tm_min;
  }
  // logfile.open(logpath);
  if(logfile.is_open())
  {
    sprintf(str,"%d-%d-%d %d:%d:%d  %d %d %d %d %d %d %d %d %d",ptminfo->tm_year + 1900,ptminfo->tm_mon + 1,ptminfo->tm_mday,ptminfo->tm_hour,ptminfo->tm_min,ptminfo->tm_sec,
    data.id,data.can_data[0],data.can_data[1],data.can_data[2],data.can_data[3],data.can_data[4],data.can_data[5],data.can_data[6],data.can_data[7]);
    logfile << str << endl;
    // logfile.close();
    return false;
  }
  else
  {
    logfile.open(logpath);
    sprintf(str,"%d-%d-%d %d:%d:%d  %d %d %d %d %d %d %d %d %d",ptminfo->tm_year + 1900,ptminfo->tm_mon + 1,ptminfo->tm_mday,ptminfo->tm_hour,ptminfo->tm_min,ptminfo->tm_sec,
    data.id,data.can_data[0],data.can_data[1],data.can_data[2],data.can_data[3],data.can_data[4],data.can_data[5],data.can_data[6],data.can_data[7]);
    logfile << str << endl;
    // logfile.open(filepath);
    return true;
  }
}

bool CANData_Output::Closefile(string &pram1, string &pram2)
{
  if(logfile.is_open())
  {
    logfile.close();
  }
  else
  {
    logfile.close();
  }
}

bool CANData_Output::Isexistence(string &pram1, string &pram2)
{
  ;
}

bool CANData_Output::Writefile(string &pram1, string &pram2, adcuCanData &data)
{
  char str[80] = {};
  time(&systime);
  ptminfo = localtime(&systime);
  sprintf(str,"%d-%d-%d %d:%d:%d  %d %d %d %d %d %d %d %d %d",ptminfo->tm_year + 1900,ptminfo->tm_mon + 1,ptminfo->tm_mday,ptminfo->tm_hour,ptminfo->tm_min,ptminfo->tm_sec,
  data.id,data.can_data[0],data.can_data[1],data.can_data[2],data.can_data[3],data.can_data[4],data.can_data[5],data.can_data[6],data.can_data[7]);
  cout << str << endl;
  logfile << str << endl;
}

// Driver_Monitor::Driver_Monitor()
// {
//   p2_info.WAIT_TIME_MAX = 6000;
//   p2_info.P2_FREQUENCY = 100;
//   p2_info.frequency = 0.0;
//   p2_info.wait_time = 0;
//   p2_info.sensor_fre_state = NORMAL;
//   p2_info.sensor_name = "p2";
//   ultrasonic1_info.frequency = 0.0;
//   ultrasonic1_info.sensor_name = "ultrasonic1";
// }

// Driver_Monitor::~Driver_Monitor()
// {
//   ;
// }

// void Driver_Monitor::sensor_frequency_calculation(sensor_info &sensor_info_,bool bRecv)
// {
//   // if(sensor_info_.sensor_name != "p2")
//   // {
//   //     ROS_INFO_STREAM("size:" << sensor_info_.list_time.size() << ",sum:" << accumulate(sensor_info_.list_time.begin(),sensor_info_.list_time.end(),0));
//   // }
//   // ROS_INFO_STREAM("size:" << sensor_info_.list_time.size());
//   if(sensor_info_.list_time.size() >= 100)
//   {
//     sensor_info_.list_time.pop_front();
//     // ROS_INFO("pop_front");
//     if(bRecv)
//     {
//       sensor_info_.list_time.push_back(1);
//       // ROS_INFO("pop_front push_back(1)");
//     }
//     else
//     {
//       sensor_info_.list_time.push_back(0);
//       // ROS_INFO("pop_front push_back(0)");
//     }
//     sensor_info_.frequency = (accumulate(sensor_info_.list_time.begin(),sensor_info_.list_time.end(),0));
//     // if(sensor_info_.sensor_name != "p2")
//     // {
//     //   ROS_INFO("frequency:%f",sensor_info_.frequency);
//     // }
//     // ROS_INFO("frequency:%f",sensor_info_.frequency);
//   }
//   else
//   {
//     if(bRecv)
//     {
//       sensor_info_.list_time.push_back(1);
//       // ROS_INFO("push_back 1");
//     }
//     else
//     {
//       sensor_info_.list_time.push_back(0);
//       // ROS_INFO("push_back 0");
//     }
//   }
// }

// void Driver_Monitor::q_sensor_push(string &str)
// {
//   q_sensor_type.push(str);
// }

// void Driver_Monitor::q_node_malfunction_push(string &str)
// {
//   q_node_malfunction.push(str);
// }

// void Driver_Monitor::set_rate(uint8_t &rate_)
// {
//   rate = rate_;
//   ROS_INFO("rate:%u",rate);
// }

// void Driver_Monitor::frequency_monitor()
// {
//   string str_sensor_type = "";
//   bool recv_sensor_state[11];
//   while(ros::ok())
//   {
//     memset(recv_sensor_state,0,sizeof(recv_sensor_state[0]) * 11);
//     if(!q_sensor_type.empty())
//     {
//       for(uint8_t i = 0;i < q_sensor_type.size();++i)
//       {
//         str_sensor_type = q_sensor_type.front();
//         q_sensor_type.pop();
//         if(!str_sensor_type.compare("p2"))
//         {
//           recv_sensor_state[0] = true;
//         }
//         else if(!str_sensor_type.compare("camera1"))
//         {
//           recv_sensor_state[1] = true;
//         }
//         else if(!str_sensor_type.compare("camera2"))
//         {
//           recv_sensor_state[2] = true;
//         }
//         else if(!str_sensor_type.compare("camera3"))
//         {
//           recv_sensor_state[3] = true;
//         }
//         else if(!str_sensor_type.compare("camera4"))
//         {
//           recv_sensor_state[4] = true;
//         }
//         else if(!str_sensor_type.compare("camera5"))
//         {
//           recv_sensor_state[5] = true;
//         }
//         else if(!str_sensor_type.compare("camera6"))
//         {
//           recv_sensor_state[6] = true;
//         }
//         else if(!str_sensor_type.compare("ultrasonic1"))
//         {
//           recv_sensor_state[7] = true;
//         }
//         else if(!str_sensor_type.compare("ultrasonic2"))
//         {
//           recv_sensor_state[8] = true;
//         }
//         else if(!str_sensor_type.compare("ultrasonic3"))
//         {
//           recv_sensor_state[9] = true;
//         }
//         else if(!str_sensor_type.compare("ultrasonic4"))
//         {
//           recv_sensor_state[10] = true;
//         }
//       }
//     }
//     sensor_frequency_calculation(p2_info,recv_sensor_state[0]);
//     sensor_frequency_calculation(camera1_info,recv_sensor_state[1]);
//     sensor_frequency_calculation(camera2_info,recv_sensor_state[2]);
//     sensor_frequency_calculation(camera3_info,recv_sensor_state[3]);
//     sensor_frequency_calculation(camera4_info,recv_sensor_state[4]);
//     sensor_frequency_calculation(camera5_info,recv_sensor_state[5]);
//     sensor_frequency_calculation(camera6_info,recv_sensor_state[6]);
//     sensor_frequency_calculation(ultrasonic1_info,recv_sensor_state[7]);
//     sensor_frequency_calculation(ultrasonic2_info,recv_sensor_state[8]);
//     sensor_frequency_calculation(ultrasonic3_info,recv_sensor_state[9]);
//     sensor_frequency_calculation(ultrasonic4_info,recv_sensor_state[10]);

//     frequency_check(p2_info);
//     // ROS_INFO_STREAM("p2_f:" << p2_info.frequency << "ultrasonic1:" << ultrasonic1_info.frequency);
//     usleep(1000 * 10);
//   }
// }

// void Driver_Monitor::frequency_check(sensor_info &frequency_info)
// {
//   if(frequency_info.wait_time < frequency_info.WAIT_TIME_MAX)
//   {
//     ++frequency_info.wait_time;
//     ROS_INFO("frequency_info.wait_time:%u",frequency_info.wait_time);
//   }
//   else
//   {
//     if(frequency_info.frequency > (1.1 * frequency_info.P2_FREQUENCY))
//     {
//       if(frequency_info.frequency < (2 * frequency_info.P2_FREQUENCY))//INFO
//       {
//         if(frequency_info.count >= 10)
//         {
//           q_sensor_type.push("I05011008");
//           ROS_INFO("p2:频率高于1.1倍,低于2倍，INFO");
//           frequency_info.count = 0;
//         }
//       }
//       else//频率高于2倍，WARNING
//       {
//         if(frequency_info.count >= 10)
//         {
//           q_sensor_type.push("W05011008");
//           ROS_INFO("p2:频率高于2倍，WARNING");
//           frequency_info.count = 0;
//         }
//       }
//     }
//     else if(frequency_info.frequency < (0.9 * frequency_info.P2_FREQUENCY))
//     {
//       if(frequency_info.frequency > (0.66 * frequency_info.P2_FREQUENCY))
//       {
//         if(frequency_info.count >= 10)
//         {
//           q_sensor_type.push("I05011008");
//           ROS_INFO("p2:频率低于于0.9倍,低于0.66倍，INFO");
//           frequency_info.count = 0;
//         }
//       }
//       else
//       {
//         if(0 == frequency_info.frequency)
//         {
//           if(frequency_info.count >= 10)
//           {
//             q_sensor_type.push("E05011008");
//             ROS_INFO("p2:频率为0，ERROR");
//             frequency_info.count = 0;
//           }
//         }
//         else
//         {
//           if(frequency_info.count >= 10)
//           {
//             q_sensor_type.push("W05011008");
//             ROS_INFO("p2:频率低于0.66倍，WARNING");
//             frequency_info.count = 0;
//           }
//         }
//       }
//     }
//   }
//   ++frequency_info.count;
// }


// void Driver_Monitor::state_check()
// {
//   ;
// }

// void Driver_Monitor::data_check()
// {
//   ;
// }

// bool CANData_Output::Writefile(string &pram1, string &pram2, adcuCanData &data)
// {
//   char str[80] = {};
//   time(&systime);
//   ptminfo = localtime(&systime);
//   sprintf(str,"%d-%d-%d %d:%d:%d  %d %d %d %d %d %d %d %d %d",ptminfo->tm_year + 1900,ptminfo->tm_mon + 1,ptminfo->tm_mday,ptminfo->tm_hour,ptminfo->tm_min,ptminfo->tm_sec,
//   data.id,data.can_data[0],data.can_data[1],data.can_data[2],data.can_data[3],data.can_data[4],data.can_data[5],data.can_data[6],data.can_data[7]);
//   cout << str << endl;
//   logfile << str << endl;
// }



// logdata_output::logdata_output(string dir_path, string sensor_name, string device_name,uint32_t m6_rec_port)
//                     : base_dir_path_(dir_path),sensor_name_(sensor_name),device_name_(device_name),channel_(m6_rec_port)
// {
//     log_file_created = false;
//     log_droped       = false;
//     g_main_min       = 0;

//     logger_name_ = sensor_name_ + "-" + device_name_ + "-" + std::to_string(channel_);

//     cout << logger_name_ << endl;

//     SPDLOG_DEBUG(logger_name_);
// }

// logdata_output::~logdata_output()
// {
//     destroy_logger();
// }

// void logdata_output::SplitString(const string &s,vector< string > &v_str,const string &c)
// {
//     string::size_type pos1,pos2;
//     pos2 = s.find(c);
//     pos1 = 0;
//     string str;
//     while(string::npos != pos2)
//     {
//         str = s.substr(pos1,pos2 - pos1);
//         if(str != "")
//         {
//             v_str.push_back(str);
//         }
//         pos1 = pos2 + c.size();
//         pos2 = s.find(c,pos1);
//     }
//     if(pos1 != s.length())
//     {
//         str = s.substr(pos1);
//         if(str != "")
//         {
//             v_str.push_back(str);
//         }
//     }
// }

// int logdata_output::my_mkdir(string muldir,mode_t mode)
// {
//     vector< string > v_str;
//     SplitString(muldir,v_str,"/");
    
//     SPDLOG_DEBUG("v_str.size():{}",v_str.size());
    
//     int iRet;
    
//     ostringstream temp_dir;
//     temp_dir.fill('0');
//     for(vector< string >::size_type i = 0;i != v_str.size();++i)
//     {
//         if(v_str[i] == "")
//         {
//             continue;
//         }
//         temp_dir << '/' << v_str[i];
//         SPDLOG_DEBUG("temp_dir:{}",temp_dir.str().c_str());
//         if(access(temp_dir.str().c_str(),0) != 0)
//         {
//             iRet = mkdir(temp_dir.str().c_str(),mode);
//             if(iRet < 0)
//             {
//                 return iRet;
//             }
//         }
//     }
//     return 0;
// }

// bool logdata_output::MinHasChanged()
// {
//     time_t raw_time;
//     struct tm *tm_info;
//     time(&raw_time);
//     tm_info = localtime(&raw_time);
    
//     if(tm_info->tm_min != g_main_min)
//     {
//         g_main_min = tm_info->tm_min;
//         return true;
//     }
//     return false;
// }

// bool logdata_output::create_log_file(string dir_path)
// {
//     stringstream log_full_path;
//     log_full_path << dir_path << log_file_name;

//     if(access(dir_path.c_str(),F_OK) != 0)//log目录不存在
//     {
//         cout << "access != 0" << endl;
//         if(my_mkdir(dir_path,0777) < 0)
//         {
//             cout << "my_mkdir < 0" << endl;
//             SPDLOG_DEBUG("mkdir={} msg={}",dir_path.data(),strerror(errno));
//         }
//     }
//     if(access(dir_path.c_str(),F_OK) != 0)
//     {
//         SPDLOG_DEBUG("dir_path:{} not find",dir_path.c_str());
//         return false;
//     }
//     else
//     {
//         SPDLOG_DEBUG("dir_path:{}",dir_path.c_str());
//         SPDLOG_DEBUG("log_full_path:{}",log_full_path.str().c_str());
//     }

//     cout << "log_full_path:" << log_full_path.str() << endl;

//     my_logger = spdlog::basic_logger_mt< spdlog::async_factory >(logger_name_, log_full_path.str());

//     my_logger->set_pattern("%Y-%m-%d-%H:%M:%S.%e,%n,%v");
    
//     spdlog::flush_every(std::chrono::seconds(1));

//     // my_logger = spdlog::get(logger_name_);

//     return true;
// }

// void logdata_output::destroy_logger()
// {
//     SPDLOG_DEBUG("dextroy_logger");
//     spdlog::drop(logger_name_);
// }

// void logdata_output::write_log(uint8_t *buff, int len)
// {
//     if(MinHasChanged() || !log_file_created)
//     {
//         if(log_droped)
//         {
//             destroy_logger();
//         }
//         time_t raw_time;
//         struct tm * tm_info;
        
//         time(&raw_time);
//         tm_info = localtime(&raw_time);
        
//         ostringstream time_pid_stream;
//         time_pid_stream.fill(0);
//         // time_pid_stream << device_name_ << '-' << channel_ << '-' << 1900 + tm_info->tm_year
//         //                 <<  setw(2) << setfill('0') << 1 + tm_info->tm_mon << setw(2) << setfill('0')
//         //                 << tm_info->tm_mday << '-'<< setw(2) << setfill('0') << tm_info->tm_hour 
//         //                 << setw(2) << setfill('0') << tm_info->tm_min  << ".csv";
//         time_pid_stream << sensor_name_ << '-' << device_name_ << '-' << channel_ << '-'
//                         << tm_info->tm_min  << ".csv";

//         cout << time_pid_stream.str() << endl;

//         ostringstream dir_path_stream;
//         dir_path_stream.fill(0);
//         dir_path_stream << base_dir_path_ << '/' << sensor_name_ << '/' << 1900 + tm_info->tm_year
//                         << setw(2) << setfill('0') << 1 + tm_info->tm_mon << setw(2) << setfill('0')
//                         << tm_info->tm_mday << '-'<< tm_info->tm_hour << '/';
                    
//         log_file_name    = time_pid_stream.str();
//         cout << "log_file_name:" << log_file_name << endl;
//         log_droped       = create_log_file(dir_path_stream.str());
//         log_file_created = true;
//     }
//     if(log_droped)
//     {
//         std::vector< unsigned char > uvchar;
//         for(size_t i = 0;i < len;++i)
//         {
//             uvchar.push_back(buff[i]);
//         }
            
//             // my_logger = spdlog::get(logger_name_);
//         my_logger -> info("{:Xspn}",spdlog::to_hex(uvchar));
//     }
// }


CANDrivers::CANDrivers(/* args */)
{
  //初始化canbuf
//  taskData taskData;
//  Sersor_stat = Work_Sersor;
  // Sersor_stat = Getstatus_Sersor;
  can_data.resize(MAX_CAN_BUF_NUM);
  for (size_t loop_i = 0; loop_i < MAX_CAN_BUF_NUM; loop_i++)
  {
    can_data[loop_i].resize(MAX_CAN_BUF_LEGTH);
    read_num[loop_i]   = 0;
    write_num[loop_i]  = 0;
    begain_tip[loop_i] = 0;
  }
  InitHandlerMap();
  // ROS_INFO("InitHandlerMap ok, map size is %d", m_pHandlerMap.size());
}

CANDrivers::~CANDrivers()
{
}

int CANDrivers::readCANOnce(int &dev_, adcuCanData &can_buf)
{

  ROS_INFO("read channel:%d once!!!", dev_);

  if (adcuDevStatus(dev_) == ADCU_DEV_STATUS_ABNORMAL)
  {
    ROS_INFO("Can %d error", dev_);
    return -1;
  }

  // adcuCanData canbuf_;

  int length = 0;
  length     = adcuDevRead(dev_, ( uint8_t * )&can_buf);
  return length;
}

void CANDrivers::writeCAN(int &dev_)
{
  ROS_INFO("write thread %d %d", write_num[dev_], dev_);
  uint16_t controltype = 0;
  unique_lock<mutex> lock(taskData.mtx_camera_w);
  uint8_t data[100];
  int len = sizeof(adcuCanData);
  bzero(data, 100);
  adcuCanData canbuf_;
  canbuf_.ide         = 0x00;
  canbuf_.dlc         = 0x08;
  canbuf_.rtr         = 0x00;
  canbuf_.prio        = 0x00;
  canbuf_.id          = 213;
//  canbuf_.id          = 0xdb;
  canbuf_.id          = 0xd4;
  canbuf_.can_data[0] = 0x03;
  canbuf_.can_data[1] = 0x01;
  canbuf_.can_data[2] = 0x10;
  canbuf_.can_data[3] = 0x10;
  canbuf_.can_data[4] = 0x10;
  canbuf_.can_data[5] = 0x10;
  canbuf_.can_data[6] = 0x10;
  canbuf_.can_data[7] = 0x10;

//  uint32_t can_write_id = 209;
//  uint32_t can_write_id_max = 212;
//  uint32_t can_write_id = 219;
//  uint32_t can_write_id_max = 222;
  uint32_t can_write_id = 213;
  uint32_t can_write_id_max = 215;
  uint8_t count = 0;

  int raw = 0;
  int leng = 1100;
  bool data_send = false;

//  can_write_id = ALL_GET_STATUS;
//  can_write_id = 125;
//  taskData.cond_camera_w.wait(lock);
//  can_write_id = ALL_START_WORK;
//  can_write_id = 124;
//  bCameraInit = false;
//  taskData.cond_camera_w.wait(lock);
  while (ros::ok())
  {
    if (adcuDevStatus(dev_) == ADCU_DEV_STATUS_ABNORMAL)
    {
      ROS_INFO("Can %d error", dev_);
      break;
    }
/*Camera_status
    //输入数字
    uint32_t in_put_;
    std::cout << "please input can_ID" << std::endl;
    std::cin >> in_put_;
    std::cout << "input can_ID：" << in_put_ << std::endl;
    can_write_id = in_put_;
*/
/*
    if(canbuf_.id < can_write_id_max)
    {
      canbuf_.id = canbuf_.id + 1;
    }
    else
    {
      if(count < (canbuf_.can_data[0] - 1))
      {
        ++count;
//        canbuf_.id = 211;
//        canbuf_.id = 221;
      }
      else
      {
        count = 0;
//        canbuf_.id = 210;
//        canbuf_.id = 220;

      }
    }
*/
    if(data_send)
    {
      if(leng<1258)
      {
        leng++;
        can_write_id = leng;
        canbuf_.dlc = 0x08;
      }
      else
      {
        if(raw<720)
        {
          can_write_id = 214;
          canbuf_.dlc = 0x02;
          canbuf_.can_data[0] = raw | 0xff;
          canbuf_.can_data[1] = raw >> 8 | 0xff;
          raw++;
          leng = 1100;
        }
        else
        {

          data_send = false;
          can_write_id = 215;
          canbuf_.dlc = 0x01;
          raw = 0;
          leng = 1100;
        }
      }
    }
    else
    {
      if(can_write_id == 213)
      {
         canbuf_.dlc = 0x01;
      }
      if(can_write_id == 214)
      {
         canbuf_.dlc = 0x02;
         canbuf_.can_data[0] = raw | 0xff;
         canbuf_.can_data[1] = raw >> 8 | 0xff;
         data_send = true;
      }
      if(can_write_id >= 216)
      {
         can_write_id = 213;
         canbuf_.dlc = 0x01;
      }
    }
    if(can_write_id>1100)
    {
      writeCANOnce(dev_,can_write_id,raw,canbuf_.dlc);
    }
    else
    {
    if (can_write_id > 0)
    {
      map< uint32_t, pFun >::iterator it;
      canOrder can_order_;
      it = m_pHandlerMap.find(can_write_id);
      {
        can_order_.reserve = 1;
        (this->*(it->second))(canbuf_, can_write_id, can_order_, dev_, controltype);
      }
      can_write_id = canbuf_.id;
      printf("write %d :", can_write_id);
      bzero(data, 100);
      memcpy(data, ( uint8_t * )&canbuf_, len);
      if (adcuDevWrite(dev_, data, len) <= 0)
      {
        printf(" error\n");
      }
      else
      {
        printf("send:");
        for (int i = 0; i < 8; i++)
        {
          printf("%02X ", canbuf_.can_data[i]);
        }
        printf("\n");
      }
    }
    else
    {
      std::cout << "please input right can_ID" << std::endl;
    }
    }
    can_write_id++;
    usleep(1000 * 1000);
  }

  adcuDevClose(dev_);
  adcuSDKDeinit();
  ROS_INFO("write thread is over!");
}

int CANDrivers::writeCANOnce(int &dev_, uint32_t can_id, uint16_t candata, uint8_t le)
{
  ROS_INFO("write ch:%d once,can_id:%d", dev_, can_id);
  uint16_t controltype = 0;
  uint8_t data[100];
  int len = sizeof(adcuCanData);
  bzero(data, 100);
  adcuCanData canbuf_;
  if(can_id == ULTRASONIC_CONTROL)
  {
    canbuf_.ide         = 0x00;
    canbuf_.dlc         = le;
    canbuf_.rtr         = 0x00;
    canbuf_.prio        = 0x00;
    canbuf_.id          = can_id;
    canbuf_.can_data[0] = candata;
    canbuf_.can_data[1] = candata >> 8 & 0xff;
    canbuf_.can_data[2] = 0x00;
    canbuf_.can_data[3] = 0x00;
    canbuf_.can_data[4] = 0x00;
    canbuf_.can_data[5] = 0x00;
    canbuf_.can_data[6] = 0x00;
    canbuf_.can_data[7] = 0x00;
  }
  else
  {
    canbuf_.ide         = 0x00;
    canbuf_.dlc         = le;
    canbuf_.rtr         = 0x00;
    canbuf_.prio        = 0x00;
    canbuf_.id          = can_id;
    canbuf_.can_data[0] = candata;
    canbuf_.can_data[1] = candata >> 8 & 0xff;
    canbuf_.can_data[2] = 0x00;
    canbuf_.can_data[3] = 0x00;
    canbuf_.can_data[4] = 0x00;
    canbuf_.can_data[5] = 0x00;
    canbuf_.can_data[6] = 0x00;
    canbuf_.can_data[7] = 0x00;
  }

  if (adcuDevStatus(dev_) == ADCU_DEV_STATUS_ABNORMAL)
  {
    ROS_INFO("Can %d error", dev_);
    return -1;
  }

  if (canbuf_.id > 0)
  {
    map< uint32_t, pFun >::iterator it;
    canOrder can_order_;

    it = m_pHandlerMap.find(canbuf_.id);
    if(it != m_pHandlerMap.end())
    {
      can_order_.reserve = 1;
      (this->*(it->second))(canbuf_, canbuf_.id, can_order_, dev_, controltype);
    }
    printf("write %d :", canbuf_.id);
    bzero(data, 100);
    memcpy(data, ( uint8_t * )&canbuf_, len);
    if (adcuDevWrite(dev_, data, len) <= 0)
    {
      printf(" error\n");
    }
    else
    {
      printf("send:");
      for (int i = 0; i < 8; i++)
      {
        printf("%02X ", canbuf_.can_data[i]);
      }
      printf("\n");
    }
    return 1;
  }
  else
  {
    ROS_INFO("Can %d write can_id error", dev_);
    return -1;
  }
}

int CANDrivers::writeCANtoP2(int &dev_, uint32_t can_id, uint8_t *candata, uint8_t le)
{
  uint16_t controltype = 0;
  uint8_t data[100];
  int len = sizeof(adcuCanData);
  bzero(data, 100);
  adcuCanData canbuf_;
  if (adcuDevStatus(dev_) == ADCU_DEV_STATUS_ABNORMAL)
  {
    ROS_INFO("Can %d error", dev_);
    return -1;
  }
  canbuf_.ide         = 0x00;
  canbuf_.dlc         = le;
  canbuf_.rtr         = 0x00;
  canbuf_.prio        = 0x00;
  canbuf_.id          = can_id;
  // printf("id_info:%02X ,%02X ,%02X ,%02X ,%02X\r\n", canbuf_.ide,canbuf_.dlc,canbuf_.rtr,canbuf_.prio,canbuf_.id);
  for(uint8_t i = 0;i < le;++i)
  {
    canbuf_.can_data[i] = candata[i];
    // printf("%02X ", canbuf_.can_data[i]);
  }
  // printf("\r\n");
  memcpy(data,(uint8_t *)&canbuf_,sizeof(adcuCanData));
  if (adcuDevWrite(dev_, data, len) <= 0)
  {
    // printf(" error\n");
    return -1;
  }
  else
  {
    // printf("send:");
    // for (int i = 0; i < 8; i++)
    // {
    //   printf("%02X ", canbuf_.can_data[i]);
    // }
    // printf("\n");
    return 1;
  }
}


void CANDrivers::InitHandlerMap()
{
  //超声波

  // m_pHandlerMap.insert(make_pair(SENSOR_DATA_1, &CANDrivers::sensorDataParse));
  // m_pHandlerMap.insert(make_pair(SENSOR_DATA_2, &CANDrivers::sensorDataParse));
  // m_pHandlerMap.insert(make_pair(SENSOR_DATA_3, &CANDrivers::sensorDataParse));
  // m_pHandlerMap.insert(make_pair(SENSOR_DATA_4, &CANDrivers::sensorDataParse));
  // m_pHandlerMap.insert(make_pair(SENSOR_CONTROL, &CANDrivers::sensorControl));

  //mobileye

  // m_pHandlerMap.insert(make_pair(LKA_Left_Lane_A, &CANDrivers::mobileyeLKA_Left_Lane_AParse));
  // m_pHandlerMap.insert(make_pair(LKA_Left_Lane_B, &CANDrivers::mobileyeLKA_Left_Lane_BParse));
  // m_pHandlerMap.insert(make_pair(LKA_Right_Lane_A, &CANDrivers::mobileyeLKA_Right_Lane_AParse));
  // m_pHandlerMap.insert(make_pair(LKA_Right_Lane_B, &CANDrivers::mobileyeLKA_Right_Lane_BParse));
  // m_pHandlerMap.insert(make_pair(Next_Lane_A, &CANDrivers::mobileyeNext_Lane_AParse));
  // m_pHandlerMap.insert(make_pair(Next_Lane_B, &CANDrivers::mobileyeNext_Lane_BParse));
  // m_pHandlerMap.insert(make_pair(Next_Lane_A_1, &CANDrivers::mobileyeNext_Lane_A_1Parse));
  // m_pHandlerMap.insert(make_pair(Next_Lane_B_1, &CANDrivers::mobileyeNext_Lane_B_1Parse));
  // m_pHandlerMap.insert(make_pair(Next_Lane_A_2, &CANDrivers::mobileyeNext_Lane_A_2Parse));
  // m_pHandlerMap.insert(make_pair(Next_Lane_B_2, &CANDrivers::mobileyeNext_Lane_B_2Parse));
  // m_pHandlerMap.insert(make_pair(Next_Lane_A_3, &CANDrivers::mobileyeNext_Lane_A_3Parse));
  // m_pHandlerMap.insert(make_pair(Next_Lane_B_3, &CANDrivers::mobileyeNext_Lane_B_3Parse));

  // m_pHandlerMap.insert(make_pair(Next_Lane_A_4, &CANDrivers::mobileyeNext_Lane_A_4Parse));
  // m_pHandlerMap.insert(make_pair(Next_Lane_B_4, &CANDrivers::mobileyeNext_Lane_B_4Parse));
  // m_pHandlerMap.insert(make_pair(Next_Lane_A_5, &CANDrivers::mobileyeNext_Lane_A_5Parse));
  // m_pHandlerMap.insert(make_pair(Next_Lane_B_5, &CANDrivers::mobileyeNext_Lane_B_5Parse));
  // m_pHandlerMap.insert(make_pair(Next_Lane_A_6, &CANDrivers::mobileyeNext_Lane_A_6Parse));
  // m_pHandlerMap.insert(make_pair(Next_Lane_B_6, &CANDrivers::mobileyeNext_Lane_B_6Parse));
  // m_pHandlerMap.insert(make_pair(Next_Lane_A_7, &CANDrivers::mobileyeNext_Lane_A_7Parse));
  // m_pHandlerMap.insert(make_pair(Next_Lane_B_7, &CANDrivers::mobileyeNext_Lane_B_7Parse));
  // m_pHandlerMap.insert(make_pair(Next_Lane_A_8, &CANDrivers::mobileyeNext_Lane_A_8Parse));
  // m_pHandlerMap.insert(make_pair(Next_Lane_B_8, &CANDrivers::mobileyeNext_Lane_B_8Parse));

  // map< uint32_t, pFun >::iterator it; 
  // for (it = m_pHandlerMap.begin(); it != m_pHandlerMap.end(); ++it)
  // {
  //   ROS_INFO("key %u", it->first);
  // }
}

} // namespace drivers
} // namespace superg_agv
