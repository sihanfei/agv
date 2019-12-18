#include "save_route_point.h"


using namespace std;
using namespace control;


namespace control
{
  SaveRoutePoint::SaveRoutePoint(ros::NodeHandle &nh) : nh_(nh)
  {
    ROS_INFO("into");
	
	ROS_INFO("Start initialization...");
	////flags
    start_tip_ = 1;
    num_ = 0;
    save_tip_ = 0;
	
	////paramaters from yaml
	ROS_INFO("Start to load paramaters from yaml...");
	nh.getParam("/control/mode",mode);
	nh.getParam("/control/pathtype",pathtype);
	nh.getParam("/control/insgps_x",insgps_x);
	nh.getParam("/control/insgps_y",insgps_y);
	nh.getParam("/control/equal_length",equal_length);
	nh.getParam("/control/L0",L0);
	nh.getParam("/control/lamda0",lamda0);
	nh.getParam("/control/hb",hb);
	ROS_INFO("Paramaters loading finished.");
	
	////Subscriber and Publisher
    //sub_ = nh.subscribe("novatel_position", 10, &SaveRoutePoint::recvNovatelGPSCallback, this);
	//sub_bestpos = nh.subscribe("bestpos",10, &SaveRoutePoint::recvbestposCallback,this);
    //veh_sub_ = nh.subscribe("com2vehmsg",10,&SaveRoutePoint::recvVehDataCallback,this);
	if(mode == 0)
	{
		pos_sub_ = nh.subscribe("/Prescan/Pose/record",10,&SaveRoutePoint::recvPrescanPosCallback,this);
	}
	else
	{
		pos_sub_ = nh.subscribe("/drivers/can_wr/imu_gnss_msg",10,&SaveRoutePoint::recvHuacePosCallback,this);
	}
    //sub_Inspva = nh.subscribe("inspva",10, &SaveRoutePoint::recvInspvaCallback,this);
	
	ROS_INFO("Initialization finished.");
	
	////main loop
    SaveRoutePoint::msg2vector();  
  }
  
  SaveRoutePoint::~SaveRoutePoint()
  {
  }

int SaveRoutePoint::get_char()
{
    fd_set rfds;
    struct timeval tv;
    int ch = 0;

    FD_ZERO(&rfds);
    FD_SET(0, &rfds);
    tv.tv_sec = 0;
    tv.tv_usec = 10; //设置等待超时时间

    //检测键盘是否有输入
    if (select(1, &rfds, NULL, NULL, &tv) > 0)
    {
        ch = getchar(); 
    }

    return ch;
}

  void SaveRoutePoint::msg2vector()
  {
    ros::Rate loop_rate(FRE);
    int begain_save_tip = 0;
	
	positionConf xy_pos_temp = {0};
	
    while(ros::ok())
    {
      if(/*start_tip_ == 2 &&*/ begain_save_tip == 1){
        //采集地图 按照每m一个点存储,以x为基础存储,每个不同的xy存储一次,x经度为1m
		if(mode == 0)
		{
			prescanxy2xy(xy_pos_temp,real_position_);
			insgps2center(xy_pos_temp);
		}
		else
		{
			gps2xy(xy_pos_temp,real_position_);
		    insgps2center(xy_pos_temp);
		}
		
        if (route_data_.size() == 0) {
          route_data_.push_back(xy_pos_temp);
          printf("route: %f %f\n",route_data_.back().lon,route_data_[0].lat);
        }
        else
        {
          if (xy_pos_temp.lon != route_data_.back().lon || xy_pos_temp.lat != route_data_.back().lat) {
            route_data_.push_back(xy_pos_temp);
          }
        }
        start_tip_ = 1;
      }
    
      int ret = get_char();
      if (ret == 115)
      {
		if(mode == 0)
		{
			ROS_INFO("Start get route point from Prescan");
		}
		else
		{
			ROS_INFO("Start get route point from GPS");
		}
        
        begain_save_tip = 1;
      }
      if (ret == 99)
      {
        ROS_INFO("Save route file");
        begain_save_tip = 0;
        save_tip_ = 1;
      }

      //地图存储的判断
      if(save_tip_ == 1)
      {
        ROS_INFO("save route data!\n");
        SaveRoutePoint::save2File();
        
        ROS_INFO("shutting down!\n");
        ros::shutdown();     
      }

      ros::spinOnce();
      loop_rate.sleep(); 

    }
  }
  
  void SaveRoutePoint::prescanxy2xy(positionConf &xy_p,const positionConf &real_p)
  {
	xy_p.x = real_p.lon;
	xy_p.y = real_p.lat;
	xy_p.z = real_p.height;
	xy_p.lon = real_p.lon;
	xy_p.lat = real_p.lat;
	xy_p.height = real_p.height;
	xy_p.heading = real_p.heading;
	xy_p.pitch = real_p.pitch;
	xy_p.roll = real_p.roll;
	xy_p.velocity = real_p.velocity;
	xy_p.velocity_x = real_p.velocity_x;
	xy_p.velocity_y = real_p.velocity_y;
	xy_p.velocity_z = real_p.velocity_z;  
  }
  
  void SaveRoutePoint::gps2xy(positionConf &xy_p,const positionConf &real_p)
  {
	double RE0 = R0/(sqrt(1 - e*e*sin(L0*M_PI/180)*sin(L0*M_PI/180))); 
	double x0 = (RE0 + hb)*cos(L0*M_PI/180)*cos(lamda0*M_PI/180);
	double y0 = (RE0 + hb)*cos(L0*M_PI/180)*sin(lamda0*M_PI/180);
	double z0 = ((1 - e*e)*RE0 + hb)*sin(L0*M_PI/180);
	
	double L = real_p.lat;
	double lamda = real_p.lon;
	double h = real_p.height;
	double RE = R0/(sqrt(1 - e*e*sin(L*M_PI/180)*sin(L*M_PI/180)));
	double dx = (RE + h)*cos(L*M_PI/180)*cos(lamda*M_PI/180) - x0;
	double dy = (RE + h)*cos(L*M_PI/180)*sin(lamda*M_PI/180) - y0;
	double dz = ((1 - e*e)*RE + h)*sin(L*M_PI/180) - z0;
	
	double dn = -sin(L*M_PI/180)*cos(lamda*M_PI/180)*dx - sin(L*M_PI/180)*sin(lamda*M_PI/180)*dy + cos(L*M_PI/180)*dz;
	double de = -sin(lamda*M_PI/180)*dx + cos(lamda*M_PI/180)*dy;
	double dd = -cos(L*M_PI/180)*cos(lamda*M_PI/180)*dx - cos(L*M_PI/180)*sin(lamda*M_PI/180)*dy - sin(L*M_PI/180)*dz;
	
	xy_p.x = de;
	xy_p.y = dn;
	xy_p.z = -dd;
	xy_p.lon = real_p.lon;
	xy_p.lat = real_p.lat;
	xy_p.height = real_p.height;
	xy_p.heading = real_p.heading;
	xy_p.pitch = real_p.pitch;
	xy_p.roll = real_p.roll;
	xy_p.velocity = real_p.velocity;
	xy_p.velocity_x = real_p.velocity_x;
	xy_p.velocity_y = real_p.velocity_y;
	xy_p.velocity_z = real_p.velocity_z;
  }
  
  void SaveRoutePoint::insgps2center(positionConf &xy_p)
  {
	  xy_p.x = xy_p.x - (insgps_x*cos(xy_p.heading*M_PI/180) + insgps_y*sin(xy_p.heading*M_PI/180));
	  xy_p.y = xy_p.y - (insgps_y*cos(xy_p.heading*M_PI/180) - insgps_x*sin(xy_p.heading*M_PI/180));
  }
  
  void SaveRoutePoint::save2File()
  {
    FILE * route_data_save_fp = NULL;
    char *home_path = getenv("HOME");
    char route_date_path_name[1024] = {0};
	
	if(mode == 0)
	{
		if(pathtype == 0)//仿真模式采环形路径点
		{
			sprintf(route_date_path_name,"%s"DATA_PATH""DATA_NAME_SIM_RING,home_path);
		}
		else//仿真模式采正常路径点
		{
			sprintf(route_date_path_name,"%s"DATA_PATH""DATA_NAME_SIM_ROUTE,home_path);
		}
	}
	else
	{
		if(pathtype == 0)//实车模式采环形路径点
		{
			sprintf(route_date_path_name,"%s"DATA_PATH""DATA_NAME_REAL_RING,home_path);
		}
		else//实车模式采正常路径点
		{
			sprintf(route_date_path_name,"%s"DATA_PATH""DATA_NAME_REAL_ROUTE,home_path);
		}
	}

    printf("route date path name:%s\n", route_date_path_name);

    char end1 =0x0d;// "/n"
    char end2 =0x0a;// "/r" 

    route_data_save_fp = fopen(route_date_path_name,"w+");

    if(route_data_save_fp  == NULL)
    {
      printf("fail to read");
      exit (1) ;
    }

    ROS_INFO("save file size is: %d",route_data_.size());

    int s = route_data_.size();
	vector <positionConf> xy_route_data_;
    //将采的点处理为等间距为equal_length
	if(s > 0)
	{
	    //计算里程
	    route_data_[0].dist = 0;
		
		for(int i = 1; i < s; i++)
		{
			route_data_[i].dist = route_data_[i - 1].dist + sqrt((route_data_[i].x - route_data_[i - 1].x)*(route_data_[i].x - route_data_[i - 1].x) + (route_data_[i].y - route_data_[i - 1].y)*(route_data_[i].y - route_data_[i - 1].y));
		}
		
	    //Initialization
        xy_route_data_.push_back(route_data_[0]);
        double Dc = route_data_[0].dist;
        int t = 1;
   
        while(t < s)
		{
			double l = 0;
			int i = 0;
			while(l < equal_length)
			{
				i = i + 1;
				if((t + i) > s)
				{
					break;
				}
				l = route_data_[t + i].dist - Dc;
			}
			t = t + i;
			if(t > s)
			{
				break;
			}
			Dc = route_data_[t].dist;
			xy_route_data_.push_back(route_data_[t]);
		}			
	}
	else
	{
		ROS_INFO("No position data recorded!");
	}
	
	if(pathtype == 0)
	{
		int s = xy_route_data_.size();
		int index = s - 1;
		double dis_tmp = sqrt((xy_route_data_[s - 1].x - xy_route_data_[0].x)*(xy_route_data_[s - 1].x - xy_route_data_[0].x) + (xy_route_data_[s - 1].y - xy_route_data_[0].y)*(xy_route_data_[s - 1].y - xy_route_data_[0].y));
		double dis_tmp_previous = dis_tmp;
		while(dis_tmp <= dis_tmp_previous)
		{
			index = index - 1;
			dis_tmp_previous = dis_tmp;
			dis_tmp = sqrt((xy_route_data_[index].x - xy_route_data_[0].x)*(xy_route_data_[index].x - xy_route_data_[0].x) + (xy_route_data_[index].y - xy_route_data_[0].y)*(xy_route_data_[index].y - xy_route_data_[0].y));
		}
		
		for(int i = 0; i <= index; i++)
		{
		  //fprintf(sentipsave,"%d",LeaveTip[sennum]);
		  fprintf(route_data_save_fp,"%u %.8lf %.8lf %.8lf %.8lf %.8lf %.8lf %.8lf %.8lf %.8lf %.8lf %.8lf %.8lf %.8lf %.8lf%c%c",
									  xy_route_data_[i].n_gps_sequence_num,
									  xy_route_data_[i].x,
									  xy_route_data_[i].y,
									  xy_route_data_[i].z,
									  xy_route_data_[i].lon,
									  xy_route_data_[i].lat,
									  xy_route_data_[i].height,//
									  xy_route_data_[i].velocity,
									  xy_route_data_[i].velocity_x,
									  xy_route_data_[i].velocity_y,
									  xy_route_data_[i].velocity_z,
									  xy_route_data_[i].heading,
									  xy_route_data_[i].pitch,//
									  xy_route_data_[i].roll,
									  xy_route_data_[i].dist,
									  end1,
									  end2
				);
		}
	}
	else
	{
		for(int i = 0; i < xy_route_data_.size(); i++)
		{
		  //fprintf(sentipsave,"%d",LeaveTip[sennum]);
		  fprintf(route_data_save_fp,"%u %.8lf %.8lf %.8lf %.8lf %.8lf %.8lf %.8lf %.8lf %.8lf %.8lf %.8lf %.8lf %.8lf %.8lf%c%c",
									  xy_route_data_[i].n_gps_sequence_num,
									  xy_route_data_[i].x,
									  xy_route_data_[i].y,
									  xy_route_data_[i].z,
									  xy_route_data_[i].lon,
									  xy_route_data_[i].lat,
									  xy_route_data_[i].height,//
									  xy_route_data_[i].velocity,
									  xy_route_data_[i].velocity_x,
									  xy_route_data_[i].velocity_y,
									  xy_route_data_[i].velocity_z,
									  xy_route_data_[i].heading,
									  xy_route_data_[i].pitch,//
									  xy_route_data_[i].roll,
									  xy_route_data_[i].dist,
									  end1,
									  end2
				);
		}
	}
	
    fclose(route_data_save_fp);

    ROS_INFO("save file succeed.");
  }

/*
  void SaveRoutePoint::recvNovatelGPSCallback(const novalet_gps::NovatelPositionConstPtr& msg)
  {

    real_position_.n_gps_sequence_num = msg->novatel_msg_header.sequence_num;
    real_position_.lon = msg->lon; //经 
    real_position_.lat = msg->lat;//纬 
    real_position_.height = msg->height;    //高
    //real_position_.height = 0.0;    //高
    real_position_.velocity_x = msg->lon_sigma;//东向速度
    real_position_.velocity_y = msg->height_sigma;//北向速度
    real_position_.heading = msg->undulation;//偏航角度
    real_position_.pitch = msg->lat_sigma;//俯仰角
	real_position_.velocity = msg->height_sigma;
    //real_position_.heading = 0.0;//偏航角度
    //real_position_.pitch = 0.0;//俯仰角
    real_position_.gps_seconds = msg->novatel_msg_header.gps_seconds;//gps秒时间

    num_++;
    
    ROS_INFO("receive %d %u %.8lf %.8lf",num_,msg->novatel_msg_header.sequence_num,real_position_.lon,real_position_.lat);  
    start_tip_ = 2;

    // if(msg->novatel_msg_header.message_name == "ZJ_N"){
    //   ROS_INFO("receive %d %u %.8lf %.8lf",num_,msg->novatel_msg_header.sequence_num,real_position_.lon,real_position_.lat);  
    //   start_tip_ = 2;
    // }
    // if(msg->novatel_msg_header.message_name == "ZJ_R"){
    //   save_tip_ = 1;
    // }
  }
*/

/*
  void SaveRoutePoint::recvVehDataCallback(const control_msgs::com2veh::ConstPtr &msg)  	
  {
    real_position_.velocity = msg->VelSpeed;
    start_tip_ = 2;
    //ROS_INFO("receive realspeed: %.8lf",real_position_.velocity);
  }  
*/

/*
  void SaveRoutePoint::recvbestposCallback(const novatel_gps_msgs::NovatelPositionConstPtr& msg)
  {
	real_position_.n_gps_sequence_num = msg->novatel_msg_header.sequence_num;
    real_position_.lon = msg->lon; //经 
    real_position_.lat = msg->lat;//纬 
    real_position_.height = msg->height;//高
   
    real_position_.gps_seconds = msg->novatel_msg_header.gps_seconds;//gps秒时间

    start_tip_ = 2;
    //num_++;
    
    //ROS_INFO("receive %d %u %.8lf %.8lf",num_,msg->novatel_msg_header.sequence_num,real_position_.lon,real_position_.lat);  
      
  }
*/

/*
  void SaveRoutePoint::recvInspvaCallback(const novatel_gps_msgs::InspvaConstPtr& msg)
  {
	real_position_.velocity_x =  msg->north_velocity;
    real_position_.velocity_y = msg->east_velocity;
    real_position_.velocity_z = msg->up_velocity;
	//real_position_.velocity = sqrt(msg->north_velocity*msg->north_velocity + msg->east_velocity*msg->east_velocity + msg->up_velocity*msg->up_velocity);

    real_position_.heading = msg->azimuth;
    real_position_.pitch = msg->pitch;
    real_position_.heading = msg->azimuth;	

    start_tip_ = 2;
  }
*/

  void SaveRoutePoint::recvHuacePosCallback(const location_sensor_msgs::IMUAndGNSSInfo &msg)
  {
	real_position_.velocity_x = (msg.velocity).x;
	real_position_.velocity_y = (msg.velocity).y;
	real_position_.velocity_z = (msg.velocity).z;
	real_position_.velocity = sqrt((msg.velocity).x*(msg.velocity).x + (msg.velocity).y*(msg.velocity).y + (msg.velocity).z*(msg.velocity).z);
	
	real_position_.heading = msg.yaw;
	real_position_.pitch = msg.pitch;
	real_position_.roll = msg.roll;
	
	real_position_.lon = msg.pose.y;
	real_position_.lat = msg.pose.x;
	real_position_.height = msg.pose.z;
	
	real_position_.gps_seconds = msg.GPS_sec;
	real_position_.n_gps_sequence_num = msg.GPS_week;
	
	start_tip_ = 2;
  }

  void SaveRoutePoint::recvPrescanPosCallback(const geometry_msgs::PoseStamped &msg)
  {
	  real_position_.velocity_x = 0;
	  real_position_.velocity_y = 0;
	  real_position_.velocity_z = 0;
	  real_position_.velocity = msg.pose.orientation.x;
	  
	  real_position_.heading = msg.pose.orientation.y;
	  real_position_.pitch = 0;
	  real_position_.roll = 0;
	  
	  real_position_.lon = msg.pose.position.x;
	  real_position_.lat = msg.pose.position.y;
	  real_position_.height = msg.pose.position.z;
	  
	  real_position_.gps_seconds = 0;
	  real_position_.n_gps_sequence_num = 0;
	  
	  start_tip_ = 2;
  }
}
