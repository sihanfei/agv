#include<ros/ros.h> //generic C++ stuff
#include <math.h>

#include "ml16_view/TanwayML.h"
#include "ml16_view/TWColor.h"

using namespace std;
/////////////// version //////////////
//  Update date: 2019/03/12

TanwayML::TanwayML(){};

void TanwayML::getParam(ros::NodeHandle nh, ros::NodeHandle nh_private){

    nh_private.param<std::string>("host", host, "192.168.111.204");
    nh_private.param<std::string>("LiDARhost", LiDARhost, "192.168.111.31");
    nh_private.param<int>("port", port, 5600);
    nh_private.param<int>("LiDARport", LiDARport, 5050);
    nh_private.param<std::string>("frame_id", frame_id, "TanwayML16");
    nh_private.param<std::string>("topic", topic, "/ml16_cloud");
    nh_private.param<double>("StartAngle", StartAngle, 2);
    nh_private.param<double>("EndAngle", EndAngle, 350);
    nh_private.param<int>("VerticleAngle", VerticleAngle,11 );
    nh_private.param<std::string>("Color", Color, "Indoor");
    nh_private.param<bool>("transformCloud_Status", transformCloud_Status,false );
    nh_private.param<double>("trans_x", trans_x, 0);
    nh_private.param<double>("trans_y", trans_y, 0);
    nh_private.param<double>("trans_z", trans_z, 0);
    nh_private.param<double>("rotate_theta_xy", rotate_theta_xy, 3.14);
    nh_private.param<double>("rotate_theta_xz", rotate_theta_xz, 3.14);
    nh_private.param<double>("rotate_theta_yz", rotate_theta_yz, 3.14);
    nh_private.param<bool>("GPS_Status", GPS_Status,false );

    for (int i=1;i<=16;i++){
	sprintf(SQ, "%s%d", "StaticQuantityFirst_",i); 
	nh_private.param<double>(SQ, StaticQuantityFirst[i-1], 0);
	}

    if (VerticleAngle==11)
	{
	memcpy(verticalChannels,verticalChannels11,sizeof(verticalChannels11));
	}
    else if (VerticleAngle==26){
	memcpy(verticalChannels,verticalChannels26,sizeof(verticalChannels11));
	}

    pubCloud = nh.advertise<sensor_msgs::PointCloud2> (topic, 1);
    Connect_Valid();
    pthread_attr_init(&pida);
    pthread_attr_setdetachstate(&pida,PTHREAD_CREATE_DETACHED);
};

TanwayML::~TanwayML(){};

static void* clearVector(void * ptr){
    std::vector<pcl::PointXYZ>* points=NULL;
    points = (std::vector<pcl::PointXYZ>*)ptr;
    points->clear();
    return NULL;
}

int TanwayML::TwoHextoFourX(unsigned char x1,unsigned char x2)
{
    int a;
    char chbuf_high[0x20];
    sprintf(chbuf_high, "%02X%02X", x1,x2); 
    sscanf(chbuf_high,"%04X",&a);
    return a;
}

int TanwayML::HexToInt(unsigned char x1)
{
    int a;
    char chbuf_high[0x10];
    sprintf(chbuf_high, "%02X", x1); 
    sscanf(chbuf_high,"%02X",&a);
    return a;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr TanwayML::transformCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pPointCloudIn, double x, double y, double z, double theta_xy, double theta_xz, double theta_yz){
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << x, y, z;
    transform.rotate(Eigen::AngleAxisf(theta_xy, Eigen::Vector3f::UnitZ()));
    transform.rotate(Eigen::AngleAxisf(theta_xz, Eigen::Vector3f::UnitY()));
    transform.rotate(Eigen::AngleAxisf(theta_yz, Eigen::Vector3f::UnitX()));
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pPointCloudOut(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::transformPointCloud(*pPointCloudIn, *pPointCloudOut, transform);
    return pPointCloudOut;
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr TanwayML::process_XYZ(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_clr_ptr,float verticalChannels[], char* buf,float hA,double* StaticQuantity,std::string Color){
    float myHA = (hA - 16.5 ) * 2;
    double cos_hA = cos(myHA * RA);
    double sin_hA = sin(myHA * RA);
    int xx = 1;
    while (xx <= 16) {
       int index = 4 + (xx - 1) * 6;
       int seq = HexToInt(buf[index]);
       int color = HexToInt(buf[index+5]);
       int hexToInt = TwoHextoFourX(buf[index + 2], buf[index + 1]);
       float L = hexToInt*c*32/10000.f/2; 
       L = ((L-StaticQuantity[seq-1])> 0) ? (L-StaticQuantity[seq-1]) : 0;
       if (L>0 && L<300){
            float vA = verticalChannels[seq - 1];
            double cos_vA_RA = cos(vA * RA);
            double x = L * cos_vA_RA * cos_hA;
            double y = L * cos_vA_RA * sin_hA;
            double z = L * sin(vA * RA);

            pcl::PointXYZRGB basic_point;
            basic_point.x = x;
            basic_point.y = y;
            basic_point.z = z;
            TWColor twcolor;
            if(Color=="None"){
                 uint32_t rgb = twcolor.ConstColor();    
                 basic_point.rgb = *reinterpret_cast<float*>(&rgb);}
            if(Color=="Indoor"){        
                 uint32_t rgb = twcolor.IndoorColor(L);    
                 basic_point.rgb = *reinterpret_cast<float*>(&rgb);}
            if(Color=="Outdoor"){        
                 uint32_t rgb = twcolor.OutdoorColor(L);    
                 basic_point.rgb = *reinterpret_cast<float*>(&rgb);}
            point_cloud_clr_ptr->points.push_back(basic_point);
            }
            xx++;
            }
    return point_cloud_clr_ptr;
}


int TanwayML::publishCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_clr_ptr_n1){
    bzero(buf,sizeof(buf));
    addrlen = sizeof(caddr);

    ret = recvfrom(sockfd,buf,205,0,(struct sockaddr*)&caddr,&addrlen);
    if (inet_ntoa(caddr.sin_addr)!= LiDARhost || htons(caddr.sin_port)!= LiDARport){
    	ROS_WARN("Warn data source,  IP:%s , port: %d\n%s:%dis the valid IP",(char *)inet_ntoa(caddr.sin_addr),htons(caddr.sin_port),LiDARhost.data(),LiDARport);
    	return 0;	
	}
    myUDPCount++;

    float hA = TwoHextoFourX(buf[8], buf[7]) / 100.0f;

    if ((StartAngle <= hA && hA <= EndAngle) ) {
        needPublishCloud = true;
        point_cloud_clr_ptr_n1 = process_XYZ(point_cloud_clr_ptr_n1,verticalChannels,buf,hA,StaticQuantityFirst,Color);
        }
    if ( hA > EndAngle && needPublishCloud) {
        point_cloud_clr_ptr_n1->width = (int) point_cloud_clr_ptr_n1->points.size();
        point_cloud_clr_ptr_n1->height = 1;
        point_cloud_clr_ptr_n1->header.frame_id = frame_id;    
        ROS_DEBUG( "Publish   num: [%d]",(int) point_cloud_clr_ptr_n1->points.size());
        if (transformCloud_Status == true)
             point_cloud_clr_ptr_n1 = transformCloud(point_cloud_clr_ptr_n1,trans_x,trans_y,trans_z,rotate_theta_xy,rotate_theta_xz,rotate_theta_yz);
        pcl::toROSMsg(*point_cloud_clr_ptr_n1, ros_cloud); 

        if (GPS_Status==true){
             int GPSmin = HexToInt(buf[100]);
             int GPSsec = HexToInt(buf[101]);
             int GPSusec = TwoHextoFourX(buf[103], buf[102]);
             time_t time_now;
             time_t GPStime;
             time(&time_now);
             tm* GPSlocaltime = localtime(&time_now);
             GPSlocaltime->tm_min = GPSmin;
             GPSlocaltime->tm_sec = GPSsec;
             GPStime = mktime(GPSlocaltime);
             ros_cloud.header.stamp = ros::Time(GPStime);
	    ros_cloud.header.stamp.nsec = GPSusec*1000000;
             }
        else{ros_cloud.header.stamp = ros::Time::now(); }
        pubCloud.publish(ros_cloud);
        pthread_create(&pid,&pida,clearVector,&point_cloud_clr_ptr_n1->points);	
        needPublishCloud = false;
     }
	
  
    if (hAPre != hA) {hAPre = hA;}
    return 0;
}

int TanwayML::Connect_Valid(){
    sockfd = socket(AF_INET,SOCK_DGRAM,0);
    saddr.sin_family = AF_INET;
    saddr.sin_port = htons(port);
    saddr.sin_addr.s_addr = inet_addr(host.data());

    ret = bind(sockfd,(struct sockaddr*)&saddr,sizeof(saddr));
    if(ret < 0) {
	perror("bind fail:");
	return -1;
    }

    int RECV_TIMEOUT_COUNT = 0;
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(sockfd,&readfds);
    struct timeval RECV_TIMEOUT;
    RECV_TIMEOUT.tv_sec = 10;

    while(select(sockfd+1,&readfds,NULL,NULL,&RECV_TIMEOUT)==0){
	if (RECV_TIMEOUT_COUNT == 5)
	    return EXIT_FAILURE;
	ROS_WARN("Failed to connect with LiDAR , time out!");
	sleep(1);
	RECV_TIMEOUT_COUNT++;
         FD_ZERO(&readfds);
	FD_SET(sockfd,&readfds);
	RECV_TIMEOUT.tv_sec = 30;
    } 
    
    ROS_INFO("Connect with LiDAR !");
    return 0;
}

