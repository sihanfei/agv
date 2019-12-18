#include "driver_monitor.h"

using namespace std;

namespace superg_agv
{
namespace drivers
{

Driver_Monitor::Driver_Monitor()
{
    p2_info.sensor_name = "p2";
    p2_info.frequency = 0.0;
    p2_info.wait_time = 0;
    p2_info.WAIT_TIME_MAX = 1000;
    p2_info.count = 0;
    p2_info.FREQUENCY = 100;
    p2_info.sensor_fre_state = NORMAL;

    camera1_info.sensor_name = "camera1";
    camera1_info.frequency = 0.0;
    camera1_info.wait_time = 0;
    camera1_info.WAIT_TIME_MAX = 6000;
    camera1_info.count = 0;
    camera1_info.FREQUENCY = 10;
    camera1_info.sensor_fre_state = NORMAL;

    camera2_info.sensor_name = "camera2";
    camera2_info.frequency = 0.0;
    camera2_info.wait_time = 0;
    camera2_info.WAIT_TIME_MAX = 6000;
    camera2_info.count = 0;
    camera2_info.FREQUENCY = 100;
    camera2_info.sensor_fre_state = NORMAL;

    camera3_info.sensor_name = "camera3";
    camera3_info.frequency = 0.0;
    camera3_info.wait_time = 0;
    camera3_info.WAIT_TIME_MAX = 6000;
    camera3_info.count = 0;
    camera3_info.FREQUENCY = 10;
    camera3_info.sensor_fre_state = NORMAL;

    camera4_info.sensor_name = "camera4";
    camera4_info.frequency = 0.0;
    camera4_info.wait_time = 0;
    camera4_info.WAIT_TIME_MAX = 6000;
    camera4_info.count = 0;
    camera4_info.FREQUENCY = 100;
    camera4_info.sensor_fre_state = NORMAL;

    camera5_info.sensor_name = "camera5";
    camera5_info.frequency = 0.0;
    camera5_info.wait_time = 0;
    camera5_info.WAIT_TIME_MAX = 6000;
    camera5_info.count = 0;
    camera5_info.FREQUENCY = 100;
    camera5_info.sensor_fre_state = NORMAL;

    camera6_info.sensor_name = "camera6";
    camera6_info.frequency = 0.0;
    camera6_info.wait_time = 0;
    camera6_info.WAIT_TIME_MAX = 6000;
    camera6_info.count = 0;
    camera6_info.FREQUENCY = 100;
    camera6_info.sensor_fre_state = NORMAL;

    ultrasonic1_info.sensor_name = "ultrasonic1";
    ultrasonic1_info.frequency = 0.0;
    ultrasonic1_info.wait_time = 0;
    ultrasonic1_info.WAIT_TIME_MAX = 6000;
    ultrasonic1_info.count = 0;
    ultrasonic1_info.FREQUENCY = 5;
    ultrasonic1_info.sensor_fre_state = NORMAL;

    ultrasonic2_info.sensor_name = "ultrasonic2";
    ultrasonic2_info.frequency = 0.0;
    ultrasonic2_info.wait_time = 0;
    ultrasonic2_info.WAIT_TIME_MAX = 6000;
    ultrasonic2_info.count = 0;
    ultrasonic2_info.FREQUENCY = 5;
    ultrasonic2_info.sensor_fre_state = NORMAL;

    ultrasonic3_info.sensor_name = "ultrasonic3";
    ultrasonic3_info.frequency = 0.0;
    ultrasonic3_info.wait_time = 0;
    ultrasonic3_info.WAIT_TIME_MAX = 6000;
    ultrasonic3_info.count = 0;
    ultrasonic3_info.FREQUENCY = 5;
    ultrasonic3_info.sensor_fre_state = NORMAL;

    ultrasonic4_info.sensor_name = "ultrasonic4";
    ultrasonic4_info.frequency = 0.0;
    ultrasonic4_info.wait_time = 0;
    ultrasonic4_info.WAIT_TIME_MAX = 6000;
    ultrasonic4_info.count = 0;
    ultrasonic4_info.FREQUENCY = 5;
    ultrasonic4_info.sensor_fre_state = NORMAL;
}

Driver_Monitor::~Driver_Monitor()
{
  ;
}

void Driver_Monitor::sensor_frequency_calculation(sensor_info &sensor_info_,bool bRecv)
{
  // if(sensor_info_.sensor_name != "p2")
  // {
  //     ROS_INFO_STREAM("size:" << sensor_info_.list_time.size() << ",sum:" << accumulate(sensor_info_.list_time.begin(),sensor_info_.list_time.end(),0));
  // }
  // ROS_INFO_STREAM("size:" << sensor_info_.list_time.size());
  if(sensor_info_.list_time.size() >= 100)
  {
    sensor_info_.list_time.pop_front();
    // ROS_INFO("pop_front");
    if(bRecv)
    {
      sensor_info_.list_time.push_back(1);
      // ROS_INFO("pop_front push_back(1)");
    }
    else
    {
      sensor_info_.list_time.push_back(0);
      // ROS_INFO("pop_front push_back(0)");
    }
    sensor_info_.frequency = (accumulate(sensor_info_.list_time.begin(),sensor_info_.list_time.end(),0));
    // if(sensor_info_.sensor_name != "p2")
    // {
    //   ROS_INFO("frequency:%f",sensor_info_.frequency);
    // }
    // ROS_INFO_STREAM(sensor_info_.sensor_name << "frequency:" << sensor_info_.frequency);
  }
  else
  {
    if(bRecv)
    {
      sensor_info_.list_time.push_back(1);
      // ROS_INFO("push_back 1");
    }
    else
    {
      sensor_info_.list_time.push_back(0);
      // ROS_INFO("push_back 0");
    }
  }
}

bool Driver_Monitor::q_sensor_empty()
{
    bool empty = false;
    mtx_sensor_type.lock();
    empty = q_sensor_type.empty();
    mtx_sensor_type.unlock();
    return empty;
}

uint16_t Driver_Monitor::get_q_sensor_size()
{
    return uint16_t(q_sensor_type.size());
}

int Driver_Monitor::q_sensor_front()
{
    int sensor_type = 0;
    if(!q_sensor_empty())
    {
        mtx_sensor_type.lock();
        sensor_type = q_sensor_type.front();
        ROS_INFO_STREAM("sensor_type_POP:" << sensor_type);
        // q_sensor_type.pop();
        q_sensor_type.pop_front();
        mtx_sensor_type.unlock();
    }
    return sensor_type;
}

void Driver_Monitor::q_sensor_push(int &sen_num)
{
//   printf("push start\r\n");
  mtx_sensor_type.lock();
//   q_sensor_type.push(str);
  q_sensor_type.push_back(sen_num);
//   ROS_INFO_STREAM("str:" << q_sensor_type.back() << ",q_sensor_type:" << q_sensor_type.size());
  mtx_sensor_type.unlock();
//   printf("push end!\r\n");
//   ROS_INFO_STREAM("str:" << str << ",q_sensor_type:" << q_sensor_type.size());
}

void Driver_Monitor::q_node_malfunction_push(Safety_Status &safety_status)
{
  mtx_node_malfunction.lock();
  q_node_malfunction.push(safety_status);
  mtx_node_malfunction.unlock();
}

Safety_Status Driver_Monitor::q_node_malfunction_front()
{
    mtx_node_malfunction.lock();
    Safety_Status safety_status = q_node_malfunction.front();
    q_node_malfunction.pop();
    mtx_node_malfunction.unlock();
    return safety_status;
}

uint16_t Driver_Monitor::get_q_node_malfunction_size()
{
    return uint16_t(q_node_malfunction.size());
}

void Driver_Monitor::set_rate(uint8_t &rate_)
{
  rate = rate_;
  ROS_INFO("rate:%u",rate);
}

void Driver_Monitor::frequency_monitor()
{
  int str_sensor_type = 0;
  bool recv_sensor_state[11];
  ros::Rate rate_loop(100);
  list<int> list_sensor;
  while(ros::ok())
  {
    memset(recv_sensor_state,0,sizeof(recv_sensor_state[0]) * 11);
    // ROS_INFO_STREAM("q_sensor_empty_before" << get_q_sensor_size());
    // if(!q_sensor_empty())
    // {
    //   ROS_INFO("!q_sensor_empty()");
    //   mtx_sensor_type.lock();
    //   ROS_INFO_STREAM("size:" << get_q_sensor_size());
    //   for(uint8_t j = 0;j < get_q_sensor_size();++j)
    //   {
    //       printf("~~~\r\n");
    //     list_sensor.push_back(q_sensor_type.front());
    //     ROS_INFO("%d" ,q_sensor_type.size());
    //     printf("@@@\r\n");
    //     q_sensor_type.pop();
    //     printf("###\r\n");
    //   }
    //   mtx_sensor_type.unlock();
    // }
    mtx_sensor_type.lock();
    while(q_sensor_type.size())
    {
    //   ROS_INFO("!q_sensor_empty()");
    //   ROS_INFO_STREAM("size:" << get_q_sensor_size());
    //   int str_a = 0;
    //   printf("~~~\r\n");
    //   str_a = q_sensor_type.front();
      list_sensor.push_back(q_sensor_type.front());
    //   cout << str_a << endl;
      q_sensor_type.pop_front();
    //   ROS_INFO_STREAM("size:" << get_q_sensor_size());
    }
    mtx_sensor_type.unlock();
    if(!list_sensor.empty())
    {
      for(list<int>::iterator iter = list_sensor.begin();iter != list_sensor.end();++iter)
      {
        switch(*iter)
        {
          case 800:
            recv_sensor_state[0] = true;
            break;
          case 210:
            recv_sensor_state[1] = true;
            break;
          case 220:
            recv_sensor_state[2] = true;
            break;
          case 230:
            recv_sensor_state[3] = true;
            break;
          case 240:
            recv_sensor_state[4] = true;
            break;
          case 250:
            recv_sensor_state[5] = true;
            break;
          case 260:
            recv_sensor_state[6] = true;
            break;
          case 601:
            recv_sensor_state[7] = true;
            break;
          case 602:
            recv_sensor_state[8] = true;
            break;
          case 603:
            recv_sensor_state[9] = true;
            break;
          case 604:
            recv_sensor_state[10] = true;
            break;
          default:
            break;
        }
      }
      list_sensor.clear();
    }
    sensor_frequency_calculation(p2_info,recv_sensor_state[0]);
    sensor_frequency_calculation(camera1_info,recv_sensor_state[1]);
    sensor_frequency_calculation(camera2_info,recv_sensor_state[2]);
    sensor_frequency_calculation(camera3_info,recv_sensor_state[3]);
    sensor_frequency_calculation(camera4_info,recv_sensor_state[4]);
    sensor_frequency_calculation(camera5_info,recv_sensor_state[5]);
    sensor_frequency_calculation(camera6_info,recv_sensor_state[6]);
    sensor_frequency_calculation(ultrasonic1_info,recv_sensor_state[7]);
    sensor_frequency_calculation(ultrasonic2_info,recv_sensor_state[8]);
    sensor_frequency_calculation(ultrasonic3_info,recv_sensor_state[9]);
    sensor_frequency_calculation(ultrasonic4_info,recv_sensor_state[10]);

    frequency_check(p2_info);
    // frequency_check(camera1_info);
    // ROS_INFO_STREAM("p2_f:" << p2_info.frequency << "ultrasonic1:" << ultrasonic1_info.frequency);
    // usleep(1000 * 10);
    rate_loop.sleep();
  }
}

void Driver_Monitor::splice_msg_code(string level,string module,string submodule,string type,uint16_t content)
{
    Safety_Status safety_status;
    string msg_code = "";
    string submodule_num = "";
    char content_arr[3];
    string content_str = "";
    sprintf(content_arr,"%03d",content);
    content_str = content_arr;
    // ROS_INFO_STREAM("str:" << submodule);
    if(!submodule.compare("p2"))
    {
        submodule_num = "01";
    }
    else if(!submodule.compare("camera1"))
    {
        submodule_num = "02";
    }
    else if(!submodule.compare("camera2"))
    {
        submodule_num = "03";
    }
    else if(!submodule.compare("camera3"))
    {
        submodule_num = "04";
    }
    else if(!submodule.compare("camera4"))
    {
        submodule_num = "05";
    }
    else if(!submodule.compare("camera5"))
    {
        submodule_num = "06";
    }
    else if(!submodule.compare("camera6"))
    {
        submodule_num = "07";
    }
    else if(!submodule.compare("ultrasonic1"))
    {
        submodule_num = "08";
    }
    else if(!submodule.compare("ultrasonic2"))
    {
        submodule_num = "09";
    }
    else if(!submodule.compare("ultrasonic3"))
    {
        submodule_num = "10";
    }
    else if(!submodule.compare("ultrasonic4"))
    {
        submodule_num = "11";
    }
    // ROS_INFO_STREAM("submodule_num:" << submodule_num);
    msg_code = level + module + submodule_num + type + content_str;
    safety_status.message_code = msg_code;
//    q_node_malfunction.push(safety_status);
    q_node_malfunction_push(safety_status);
}

void Driver_Monitor::frequency_check(sensor_info &frequency_info)
{
//   string msg_code = "";
    static int w_t_static = 0;
    if(frequency_info.wait_time < frequency_info.WAIT_TIME_MAX)
    {
        ++frequency_info.wait_time;
        if(w_t_static != int(frequency_info.wait_time/100))
        {
            ROS_INFO("frequency_info.wait_time:%u",frequency_info.wait_time);
            w_t_static = int(frequency_info.wait_time/100);
        }
    }
    else
    {
        if(frequency_info.list_time.size() >= 100)
        {
            if(frequency_info.frequency > (1.1 * frequency_info.FREQUENCY))
            {
                if(frequency_info.frequency < (2 * frequency_info.FREQUENCY))//INFO
                {
                    if(frequency_info.count >= 10)
                    {
                        splice_msg_code("I","05",frequency_info.sensor_name,"1",8);
                    }
                    ROS_INFO("p2:频率高于1.1倍,低于2倍，INFO");
                    frequency_info.count = 0;
                }
                else//频率高于2倍，WARNING
                {
                    if(frequency_info.count >= 10)
                    {
                        splice_msg_code("W","05",frequency_info.sensor_name,"1",8);
                        // q_node_malfunction.push("W05011008");
                        ROS_INFO("p2:频率高于2倍，WARNING"); 
                        frequency_info.count = 0;
                    }
                }
            }
            else if(frequency_info.frequency < (0.9 * frequency_info.FREQUENCY))
            {
                if(frequency_info.frequency > (0.66 * frequency_info.FREQUENCY))
                {
                    if(frequency_info.count >= 10)
                    {
                        splice_msg_code("I","05",frequency_info.sensor_name,"1",8);
                        // q_node_malfunction.push("I05011008");
                        ROS_INFO("p2:频率低于于0.9倍,低于0.66倍，INFO");
                        frequency_info.count = 0;
                    }
                }
                else
                {
                    if(0 == frequency_info.frequency)
                    {
                        if(frequency_info.count >= 10)
                        {
                            splice_msg_code("E","05",frequency_info.sensor_name,"1",8);
                            // q_node_malfunction.push("E05011008");
                            ROS_INFO("p2:频率为0，ERROR");
                            frequency_info.count = 0;
                        }
                    }
                    else
                    {
                        if(frequency_info.count >= 10)
                        {
                            splice_msg_code("W","05",frequency_info.sensor_name,"1",8);
                            // q_node_malfunction.push("W05011008");
                            ROS_INFO("p2:频率低于0.66倍，WARNING");
                            frequency_info.count = 0;
                        }
                    }
                }
            }
        }
        ++frequency_info.count;
    }
    // ++frequency_info.count;
}

std::list<int> Driver_Monitor::q_sensor_type;
std::queue<Safety_Status> Driver_Monitor::q_node_malfunction;
std::mutex Driver_Monitor::mtx_sensor_type;
std::mutex Driver_Monitor::mtx_node_malfunction;

}
}
