#include "mobileye.h"

namespace superg_agv
{
namespace drivers
{

MOBILEYE::MOBILEYE()
{
    MOBILEYE_RECV_TYPE = NONE;
    m_pHandlerMap.insert(make_pair(MOBILEYE_LANE_INFO_AND_MEASURE,&MOBILEYE::mobileyeLaneInfoAndMeasureParse));
    m_pHandlerMap.insert(make_pair(MOBILEYE_SYSTE_MWARING,&MOBILEYE::mobileyeSystemWarningParse));
    // m_pHandlerMap.inset(make_pair(MOBILEYE_TSR_TYPE_AND_POSITION,&mobileye::mobileyeTSRTypeAndPositionParse));
    for(uint8_t i = 0; i < 7; ++i)
    {
        m_pHandlerMap.insert(make_pair(MOBILEYE_TSR_TYPE_AND_POSITION + i,&MOBILEYE::mobileyeTSRTypeAndPositionParse));
    }
    m_pHandlerMap.insert(make_pair(MOBILEYE_TSR_VISION_DECISION,&MOBILEYE::mobileyeTSRVisionDecisionPares));
    m_pHandlerMap.insert(make_pair(MOBILEYE_LIGHTS_LOCATION_AND_ANGLES,&MOBILEYE::mobileyeLightsLocationAndAnglesPares));
    m_pHandlerMap.insert(make_pair(MOBILEYE_LANE_INFO_MEASUREMENTS,&MOBILEYE::mobileyeLaneInfoMeasureParse));
    m_pHandlerMap.insert(make_pair(MOBILEYE_NUMBER_OF_OBSTACLES,&MOBILEYE::mobileyeNumberOfObstaclesParse));
    for(uint8_t j = 0; j < 14; ++j)
    {
        m_pHandlerMap.insert(make_pair(MOBILEYE_OBSTACLE_DATA_A + j,&MOBILEYE::mobileyeObstaclesData_AParse));
        m_pHandlerMap.insert(make_pair(MOBILEYE_OBSTACLE_DATA_B + j,&MOBILEYE::mobileyeObstaclesData_BParse));
        m_pHandlerMap.insert(make_pair(MOBILEYE_OBSTACLE_DATA_C + j,&MOBILEYE::mobileyeObstaclesData_CParse));
    }
    // m_pHandlerMap.inset(make_pair(MOBILEYE_OBSTACLE_DATA_A,&mobileye::mobileyeObstaclesData_AParse));
    // m_pHandlerMap.inset(make_pair(MOBILEYE_OBSTACLE_DATA_B,&mobileye::mobileyeObstaclesData_BParse));
    // m_pHandlerMap.inset(make_pair(MOBILEYE_OBSTACLE_DATA_C,&mobileye::mobileyeObstaclesData_CParse));
    m_pHandlerMap.insert(make_pair(MOBILEYE_SIGNALS_STATUS_VEHICLE,&MOBILEYE::mobileyeSignalsStatusVehicleParse));
    m_pHandlerMap.insert(make_pair(MOBILEYE_LKA_LEFT_LANE_A,&MOBILEYE::mobileyeLKALeftLane_AParse));
    m_pHandlerMap.insert(make_pair(MOBILEYE_LKA_LEFT_LANE_B,&MOBILEYE::mobileyeLKALeftLane_BParse));
    m_pHandlerMap.insert(make_pair(MOBILEYE_LKA_RIGHT_LANE_A,&MOBILEYE::mobileyeLKARightLane_AParse));
    m_pHandlerMap.insert(make_pair(MOBILEYE_LKA_RIGHT_LANE_B,&MOBILEYE::mobileyeLKARightLane_BParse));
    m_pHandlerMap.insert(make_pair(MOBILEYE_REFERENCE_POINTS,&MOBILEYE::mobileyeReference_PointsParse));
    m_pHandlerMap.insert(make_pair(MOBILEYE_NUMBER_OF_NEXT_LANE,&MOBILEYE::mobileyeNunber_Of_Next_LaneParse));
    for(uint8_t k = 0; k < 8; ++k)
    {
        m_pHandlerMap.insert(make_pair(MOBILEYE_LEFT_NEXT_LANE_A + 4 * k,&MOBILEYE::mobileyeLKALeftLane_AParse));
        m_pHandlerMap.insert(make_pair(MOBILEYE_LEFT_NEXT_LANE_B + 4 * k,&MOBILEYE::mobileyeLKALeftLane_BParse));
        m_pHandlerMap.insert(make_pair(MOBILEYE_RIGHT_NEXT_LANE_A + 4 * k,&MOBILEYE::mobileyeLKARightLane_AParse));
        m_pHandlerMap.insert(make_pair(MOBILEYE_RIGHT_NEXT_LANE_B + 4 * k,&MOBILEYE::mobileyeLKARightLane_BParse));
    }

}

MOBILEYE::~MOBILEYE()
{

}

void MOBILEYE::CANMobileye(int &dev_, int &ch_)
{
    ROS_INFO("readmobileye thread %d %d %d", read_num[ch_], ch_, dev_);
    uint16_t controltype = 0;
    uint8_t RecvDataCount = 0;
    string logname = "camera";
    logdata_output can_log(logname,to_string(dev_),ch_);
    // CANData_Output data_Output(logname);
    while(ros::ok())
    {
        if (adcuDevStatus(dev_) == ADCU_DEV_STATUS_ABNORMAL)
        {
            ROS_INFO("Can %d error", ch_);
            break;
        }
        adcuCanData canbuf_;
        int length = 0;
        map< uint32_t, pFun >::iterator it;
        canOrder can_order_;
        length     = adcuDevRead(dev_, ( uint8_t * )&canbuf_);
        can_log.write_log(&canbuf_.can_data[0],canbuf_.id,8);
        // data_Output.Openfile(logname,logname,canbuf_,ch_);
        if(length > 0)
        {
            printf("read:");
            for (int i = 0; i < 8; i++)
            {
                printf("%02X ", canbuf_.can_data[i]);
            }
            printf("\n");
            switch(MOBILEYE_RECV_TYPE)
            {
                case NONE:
                    if(MOBILEYE_NUMBER_OF_OBSTACLES == canbuf_.id)
                    {
                        RecvDataCount = 0;
                        MOBILEYE_RECV_TYPE = OBSTACLES;
                    }
                    else if(MOBILEYE_LKA_LEFT_LANE_A == canbuf_.id)
                    {
                        MOBILEYE_RECV_TYPE = LANE;
                    }
                    break;
                case OBSTACLES:
                    if(RecvDataCount < mobileyeData.mobileyeObstaclesData.size())
                    {
                        it = m_pHandlerMap.find(canbuf_.id);
                        if(it != m_pHandlerMap.end())
                        {
                            (this->*(it->second))(canbuf_, canbuf_.id, can_order_, ch_, controltype);
                        }
                        if(MOBILEYE_OBSTACLE_DATA_C == canbuf_.id)
                        {
                            ++RecvDataCount;
                        }
                    }
                    else
                    {
                        MOBILEYE_RECV_TYPE = NONE;
                        mobileyeData.bObstaclesReady = true;
                        cond_mobileye.notify_one();
                    }
                    break;
                case LANE:
                    if(MOBILEYE_LKA_RIGHT_LANE_B == canbuf_.id)
                    {

                        it = m_pHandlerMap.find(canbuf_.id);
                        if(it != m_pHandlerMap.end())
                        {
                            (this->*(it->second))(canbuf_, canbuf_.id, can_order_, ch_, controltype);
                        }
                        MOBILEYE_RECV_TYPE = NONE;
                        mobileyeData.bLaneReady = true;
                        cond_mobileye.notify_one();
                    }
                    else
                    {
                        it = m_pHandlerMap.find(canbuf_.id);
                        if(it != m_pHandlerMap.end())
                        {
                            (this->*(it->second))(canbuf_, canbuf_.id, can_order_, ch_, controltype);
                        }
                    }
                    break;
                default:
                    break;
            }
        }
    }
}

void MOBILEYE::mobileye_Sender()
{
    ROS_INFO("thread cmaera_Sender Start!");
    unique_lock<mutex> lock(mtx_mobileye);
    ros::NodeHandle nh;
    ros::Publisher obstacle_pub = nh.advertise<perception_sensor_msgs::ObjectList>("/drivers/perception/mobileye_obstacle_info", 10, true);
    ros::Publisher lane_pub = nh.advertise<visualization_msgs::MarkerArray>("/drivers/perception/mobileye_lane_info", 10, true);
    double radian;
    float measurement_cov[16] = {1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
    while(ros::ok())
    {
        cond_mobileye.wait(lock);
        if(mobileyeData.bObstaclesReady)
        {
            ROS_INFO("mobileyeobjectdata is ready to topic!!!");
            perception_sensor_msgs::ObjectList perceptionobstacleinfo;
            common_msgs::DetectionInfo obstacleinfo;
            perceptionobstacleinfo.header.stamp      = ros::Time::now();
            perceptionobstacleinfo.header.frame_id   = "base_link";
            perceptionobstacleinfo.sensor_index      = 1;
            perceptionobstacleinfo.obstacle_num = mobileyeData.mobileyeNumberOfObstacles.Number_Of_Obstacles;
            list<mobileyeObstaclesDataStr>::iterator ite_Obsinfo = mobileyeData.mobileyeObstaclesData.begin();
            ROS_INFO("mobileyeData.mobileyeNumberOfObstacles.size():%lu",mobileyeData.mobileyeObstaclesData.size());
            for(int i = 0; i < mobileyeData.mobileyeObstaclesData.size(); ++i)
            {
                obstacleinfo.id         = i;
                obstacleinfo.obj_class  = ite_Obsinfo->Obstacle_Type;
                obstacleinfo.confidence = ite_Obsinfo->Obstacle_Age;
                obstacleinfo.state[0] = ite_Obsinfo->Obstacle_Position_X;
                obstacleinfo.state[1] = ite_Obsinfo->Obstacle_Position_Y;
                obstacleinfo.state[2] = ite_Obsinfo->Obstacle_Relative_Velocity_X;
                obstacleinfo.state[3] = 0;
                radian = 0 * M_PI/180;
                for(int j = 0;j<16;++j)
                {
                    obstacleinfo.measurement_cov[j] = measurement_cov[j];
                }
                if(ite_Obsinfo->Obstacle_Type == MODLE_VEHICLE)
                {
                    ROS_INFO("CAR!!!");
                    obstacleinfo.peek[0].x = ite_Obsinfo->Obstacle_Position_X * 1000 + (MODE_SIZE_CAR/2 *cos(radian)) - (MODE_SIZE_CAR/2 *sin(radian));
                    obstacleinfo.peek[0].y = ite_Obsinfo->Obstacle_Position_Y * 1000 + (MODE_SIZE_CAR/2 *sin(radian)) + (MODE_SIZE_CAR/2 *cos(radian));
                    obstacleinfo.peek[1].x = ite_Obsinfo->Obstacle_Position_X * 1000 + (MODE_SIZE_CAR/2 *cos(radian)) + (MODE_SIZE_CAR/2 *sin(radian));
                    obstacleinfo.peek[1].y = ite_Obsinfo->Obstacle_Position_Y * 1000 + (MODE_SIZE_CAR/2 *sin(radian)) - (MODE_SIZE_CAR/2 *cos(radian));
                    obstacleinfo.peek[2].x = ite_Obsinfo->Obstacle_Position_X * 1000 - (MODE_SIZE_CAR/2 *cos(radian)) - (MODE_SIZE_CAR/2 *sin(radian));
                    obstacleinfo.peek[2].y = ite_Obsinfo->Obstacle_Position_Y * 1000 - (MODE_SIZE_CAR/2 *sin(radian)) + (MODE_SIZE_CAR/2 *cos(radian));
                    obstacleinfo.peek[3].x = ite_Obsinfo->Obstacle_Position_X * 1000 + (MODE_SIZE_CAR/2 *cos(radian)) - (MODE_SIZE_CAR/2 *sin(radian));
                    obstacleinfo.peek[3].y = ite_Obsinfo->Obstacle_Position_Y * 1000 + (MODE_SIZE_CAR/2 *sin(radian)) + (MODE_SIZE_CAR/2 *cos(radian));
                }
                else if(ite_Obsinfo->Obstacle_Type == PES)
                {
                    ROS_INFO("PES!!!");
                    obstacleinfo.peek[0].x = ite_Obsinfo->Obstacle_Position_X * 1000 + (MODE_SIZE_PES/2 *cos(radian)) - (MODE_SIZE_PES/2 *sin(radian));
                    obstacleinfo.peek[0].y = ite_Obsinfo->Obstacle_Position_Y * 1000 + (MODE_SIZE_PES/2 *sin(radian)) + (MODE_SIZE_PES/2 *cos(radian));
                    obstacleinfo.peek[1].x = ite_Obsinfo->Obstacle_Position_X * 1000 + (MODE_SIZE_PES/2 *cos(radian)) + (MODE_SIZE_PES/2 *sin(radian));
                    obstacleinfo.peek[1].y = ite_Obsinfo->Obstacle_Position_Y * 1000 + (MODE_SIZE_PES/2 *sin(radian)) - (MODE_SIZE_PES/2 *cos(radian));
                    obstacleinfo.peek[2].x = ite_Obsinfo->Obstacle_Position_X * 1000 - (MODE_SIZE_PES/2 *cos(radian)) - (MODE_SIZE_PES/2 *sin(radian));
                    obstacleinfo.peek[2].y = ite_Obsinfo->Obstacle_Position_Y * 1000 - (MODE_SIZE_PES/2 *sin(radian)) + (MODE_SIZE_PES/2 *cos(radian));
                    obstacleinfo.peek[3].x = ite_Obsinfo->Obstacle_Position_X * 1000 + (MODE_SIZE_PES/2 *cos(radian)) - (MODE_SIZE_PES/2 *sin(radian));
                    obstacleinfo.peek[3].y = ite_Obsinfo->Obstacle_Position_Y * 1000 + (MODE_SIZE_PES/2 *sin(radian)) + (MODE_SIZE_PES/2 *cos(radian));
                }
                else if(ite_Obsinfo->Obstacle_Type == BICYCLE)
                {
                    ROS_INFO("BICYCLE!!!");
                    obstacleinfo.peek[0].x = ite_Obsinfo->Obstacle_Position_X * 1000 + (MODE_SIZE_BICYCLE/2 *cos(radian)) - (MODE_SIZE_BICYCLE/2 *sin(radian));
                    obstacleinfo.peek[0].y = ite_Obsinfo->Obstacle_Position_Y * 1000 + (MODE_SIZE_BICYCLE/2 *sin(radian)) + (MODE_SIZE_BICYCLE/2 *cos(radian));
                    obstacleinfo.peek[1].x = ite_Obsinfo->Obstacle_Position_X * 1000 + (MODE_SIZE_BICYCLE/2 *cos(radian)) + (MODE_SIZE_BICYCLE/2 *sin(radian));
                    obstacleinfo.peek[1].y = ite_Obsinfo->Obstacle_Position_Y * 1000 + (MODE_SIZE_BICYCLE/2 *sin(radian)) - (MODE_SIZE_BICYCLE/2 *cos(radian));
                    obstacleinfo.peek[2].x = ite_Obsinfo->Obstacle_Position_X * 1000 - (MODE_SIZE_BICYCLE/2 *cos(radian)) - (MODE_SIZE_BICYCLE/2 *sin(radian));
                    obstacleinfo.peek[2].y = ite_Obsinfo->Obstacle_Position_Y * 1000 - (MODE_SIZE_BICYCLE/2 *sin(radian)) + (MODE_SIZE_BICYCLE/2 *cos(radian));
                    obstacleinfo.peek[3].x = ite_Obsinfo->Obstacle_Position_X * 1000 + (MODE_SIZE_BICYCLE/2 *cos(radian)) - (MODE_SIZE_BICYCLE/2 *sin(radian));
                    obstacleinfo.peek[3].y = ite_Obsinfo->Obstacle_Position_Y * 1000 + (MODE_SIZE_BICYCLE/2 *sin(radian)) + (MODE_SIZE_BICYCLE/2 *cos(radian));
                }
                else if(ite_Obsinfo->Obstacle_Type == ROADBLOCK)
                {
                    obstacleinfo.peek[0].x = ite_Obsinfo->Obstacle_Position_X * 1000 + (MODE_SIZE_ROADBLOCK/2 *cos(radian)) - (MODE_SIZE_ROADBLOCK/2 *sin(radian));
                    obstacleinfo.peek[0].y = ite_Obsinfo->Obstacle_Position_Y * 1000 + (MODE_SIZE_ROADBLOCK/2 *sin(radian)) + (MODE_SIZE_ROADBLOCK/2 *cos(radian));
                    obstacleinfo.peek[1].x = ite_Obsinfo->Obstacle_Position_X * 1000 + (MODE_SIZE_ROADBLOCK/2 *cos(radian)) + (MODE_SIZE_ROADBLOCK/2 *sin(radian));
                    obstacleinfo.peek[1].y = ite_Obsinfo->Obstacle_Position_Y * 1000 + (MODE_SIZE_ROADBLOCK/2 *sin(radian)) - (MODE_SIZE_ROADBLOCK/2 *cos(radian));
                    obstacleinfo.peek[2].x = ite_Obsinfo->Obstacle_Position_X * 1000 - (MODE_SIZE_ROADBLOCK/2 *cos(radian)) - (MODE_SIZE_ROADBLOCK/2 *sin(radian));
                    obstacleinfo.peek[2].y = ite_Obsinfo->Obstacle_Position_Y * 1000 - (MODE_SIZE_ROADBLOCK/2 *sin(radian)) + (MODE_SIZE_ROADBLOCK/2 *cos(radian));
                    obstacleinfo.peek[3].x = ite_Obsinfo->Obstacle_Position_X * 1000 + (MODE_SIZE_ROADBLOCK/2 *cos(radian)) - (MODE_SIZE_ROADBLOCK/2 *sin(radian));
                    obstacleinfo.peek[3].y = ite_Obsinfo->Obstacle_Position_Y * 1000 + (MODE_SIZE_ROADBLOCK/2 *sin(radian)) + (MODE_SIZE_ROADBLOCK/2 *cos(radian));
                }
                perceptionobstacleinfo.object_list.push_back(obstacleinfo);
                ite_Obsinfo++;
            }
            obstacle_pub.publish(perceptionobstacleinfo);
            mobileyeData.mobileyeObstaclesData.clear();
            mobileyeData.bObstaclesReady = false;
        }
        if(mobileyeData.bLaneReady)
        {
            ROS_INFO("mobileyeLaneData is ready to topic!!!");
            visualization_msgs::MarkerArray perceptionlaneinfo;
            geometry_msgs::Point lanepoint;
            lanepoint.y = 0;
            list<mobileyeLKALaneStr>::iterator ite_laneinfo_left = mobileyeData.mobileyeLKALeftLane.begin();
            list<mobileyeLKALaneStr>::iterator ite_laneinfo_right = mobileyeData.mobileyeLKARightLane.begin();
            for(int i = 0;i < mobileyeData.mobileyeLKALeftLane.size();++i)
            {
                visualization_msgs::Marker laneinfo;
                switch(ite_laneinfo_left -> Model_degree)
                {
                    case 1://linear model
                        while(lanepoint.y + THETA_Y < ite_laneinfo_left->View_Range)
                        {
                            lanepoint.y = lanepoint.y + THETA_Y;
                            lanepoint.x = ite_laneinfo_left->Heading_Angle_Parameter_C1 * lanepoint.y
                                          + ite_laneinfo_left->Position_Parameter_C0;
                            lanepoint.z = 0;
                            laneinfo.points.push_back(lanepoint);
                        }
                        break;
                    case 2://parabolic model
                        while(lanepoint.y + THETA_Y < ite_laneinfo_left->View_Range)
                        {
                            lanepoint.y = lanepoint.y + THETA_Y;
                            lanepoint.x = ite_laneinfo_left->Curvature_Parameter_C2 * pow(lanepoint.y, 2)
                                          + ite_laneinfo_left->Heading_Angle_Parameter_C1 * lanepoint.y
                                          + ite_laneinfo_left->Position_Parameter_C0;
                            lanepoint.z = 0;
                            laneinfo.points.push_back(lanepoint);
                        }
                        break;
                    case 3://3 rd -degree model
                        while(lanepoint.y + THETA_Y < ite_laneinfo_left->View_Range)
                        {
                            lanepoint.y = lanepoint.y + THETA_Y;
                            lanepoint.x = ite_laneinfo_left->Curvature_Derivative_Parameter_C3 * pow(lanepoint.y, 3)
                                          + ite_laneinfo_left->Curvature_Parameter_C2 * pow(lanepoint.y, 2)
                                          + ite_laneinfo_left->Heading_Angle_Parameter_C1 * lanepoint.y
                                          + ite_laneinfo_left->Position_Parameter_C0;
                            lanepoint.z = 0;
                            laneinfo.points.push_back(lanepoint);
                        }
                        break;
                    default:
                        break;
                }
                laneinfo.points.push_back(lanepoint);
                perceptionlaneinfo.markers.push_back(laneinfo);
                ite_laneinfo_left++;
            }
            for(int i = 0;i < mobileyeData.mobileyeLKARightLane.size();++i)
            {
                visualization_msgs::Marker laneinfo;
                switch(ite_laneinfo_right -> Model_degree)
                {
                    case 1://linear model
                        while(lanepoint.y + THETA_Y < ite_laneinfo_right->View_Range)
                        {
                            lanepoint.y = lanepoint.y + THETA_Y;
                            lanepoint.x = ite_laneinfo_right->Heading_Angle_Parameter_C1 * lanepoint.y
                                          + ite_laneinfo_right->Position_Parameter_C0;
                            lanepoint.z = 0;
                            laneinfo.points.push_back(lanepoint);
                        }
                        break;
                    case 2://parabolic model
                        while(lanepoint.y + THETA_Y < ite_laneinfo_right->View_Range)
                        {
                            lanepoint.y = lanepoint.y + THETA_Y;
                            lanepoint.x = ite_laneinfo_right->Curvature_Parameter_C2 * pow(lanepoint.y, 2)
                                          + ite_laneinfo_right->Heading_Angle_Parameter_C1 * lanepoint.y
                                          + ite_laneinfo_right->Position_Parameter_C0;
                            lanepoint.z = 0;
                            laneinfo.points.push_back(lanepoint);
                        }
                        break;
                    case 3://3 rd -degree model
                        while(lanepoint.y + THETA_Y < ite_laneinfo_right->View_Range)
                        {
                            lanepoint.y = lanepoint.y + THETA_Y;
                            lanepoint.x = ite_laneinfo_right->Curvature_Derivative_Parameter_C3 * pow(lanepoint.y, 3)
                                          + ite_laneinfo_right->Curvature_Parameter_C2 * pow(lanepoint.y, 2)
                                          + ite_laneinfo_right->Heading_Angle_Parameter_C1 * lanepoint.y
                                          + ite_laneinfo_right->Position_Parameter_C0;
                            lanepoint.z = 0;
                            laneinfo.points.push_back(lanepoint);
                        }
                        break;
                    default:
                        break;
                }
                laneinfo.points.push_back(lanepoint);
                perceptionlaneinfo.markers.push_back(laneinfo);
                ite_laneinfo_right++;
            }
            lane_pub.publish(perceptionlaneinfo);
            mobileyeData.mobileyeLKALeftLane.clear();
            mobileyeData.mobileyeLKARightLane.clear();
            mobileyeData.bLaneReady = false;
        }
    }
}

void MOBILEYE::mobileyeLaneInfoAndMeasureParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype)
{
    ROS_INFO("recv: %d mobileyeLaneInfoAndMeasureParse", can_id);
    mobileyeData.lane_type_left = can_buf.can_data[0] >> 4 & 0x0F;
    mobileyeData.LDW_left = can_buf.can_data[0] >> 2 & 0x01;
    mobileyeData.confidence_lane_left = can_buf.can_data[0] & 0x03;
    mobileyeData.distance_lane_left = (can_buf.can_data[1] >> 4 & 0x0f) | can_buf.can_data[2] << 4;

    mobileyeData.lane_type_right = can_buf.can_data[5] >> 4 & 0x0F;
    mobileyeData.LDW_right = can_buf.can_data[5] >> 2 & 0x01;
    mobileyeData.confidence_lane_right = can_buf.can_data[5] & 0x03;
    mobileyeData.distance_lane_right = (can_buf.can_data[6] >> 4 & 0x0f) | can_buf.can_data[7] << 4;

    ROS_INFO("lane_type_left: %d, LDW_left: %d, confidence_lane_left: %d, distanc_lane_left: %d,lane_type_right: %d, LDW_right: %d, confidence_lane_right: %d, distance_lane_right: %d",
              mobileyeData.lane_type_left, mobileyeData.LDW_left, mobileyeData.confidence_lane_left, mobileyeData.distance_lane_left,
              mobileyeData.lane_type_right, mobileyeData.LDW_right, mobileyeData.confidence_lane_right, mobileyeData.distance_lane_right);
}

void MOBILEYE::mobileyeSystemWarningParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype)
{
    ROS_INFO("recv: %d mobileyeSystemWarningParse", can_id);
    mobileyeData.mobileyeSystemWaring.sound_type = can_buf.can_data[0] & 0x07;
    mobileyeData.mobileyeSystemWaring.Peds_in_DZ = can_buf.can_data[5] >> 2 & 0x01;
    mobileyeData.mobileyeSystemWaring.Peds_FCW = can_buf.can_data[5] >> 1 & 0x01;
    mobileyeData.mobileyeSystemWaring.FCW_on = can_buf.can_data[4] >> 3 & 0x01;
    mobileyeData.mobileyeSystemWaring.Time_Indicator = can_buf.can_data[0] >> 3 & 0x03;
    mobileyeData.mobileyeSystemWaring.Error_Valid = can_buf.can_data[3] & 0x01;
    mobileyeData.mobileyeSystemWaring.Error_Code = can_buf.can_data[3] & 0xFE;
    mobileyeData.mobileyeSystemWaring.Zero_speed = can_buf.can_data[1] >> 5 & 0x01;
    mobileyeData.mobileyeSystemWaring.Headway_Valid = can_buf.can_data[2] & 0x01;
    mobileyeData.mobileyeSystemWaring.Headway_measurement = (can_buf.can_data[2] & 0xFE) * 0.1;
    mobileyeData.mobileyeSystemWaring.LDW_Off = can_buf.can_data[4] & 0x01;
    mobileyeData.mobileyeSystemWaring.Left_LDW_On = can_buf.can_data[4] >> 1 & 0x01;
    mobileyeData.mobileyeSystemWaring.Right_LDW_On = can_buf.can_data[4] >> 2 & 0x01;
    mobileyeData.mobileyeSystemWaring.Maintenance = can_buf.can_data[4] >> 6 & 0x01;
    mobileyeData.mobileyeSystemWaring.FailSafe = can_buf.can_data[4] >> 7 & 0x01;
    mobileyeData.mobileyeSystemWaring.FCW_On = can_buf.can_data[4] >> 3 & 0x01;
    mobileyeData.mobileyeSystemWaring.TSR_Enabled = can_buf.can_data[5] >> 7 & 0x01;
    mobileyeData.mobileyeSystemWaring.HW_Repeatable_Enabled = can_buf.can_data[7] >> 1 & 0x03;
    mobileyeData.mobileyeSystemWaring.Headway_Warning_Level = can_buf.can_data[7] & 0x01;
    mobileyeData.mobileyeSystemWaring.TSR_Warning_Level = can_buf.can_data[6] & 0x03;
    mobileyeData.mobileyeSystemWaring.Tamper_Alert = can_buf.can_data[5] >> 5 & 0x01;
}

void MOBILEYE::mobileyeTSRTypeAndPositionParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype)
{
    ROS_INFO("recv: %d mobileyeTSRTypeAndPositionParse", can_id);
    mobileyeData.mobileyeTSRTypeAndPosition.Vision_Only_Sign_Type = can_buf.can_data[0];
    mobileyeData.mobileyeTSRTypeAndPosition.Supplementary_Sign_Type = can_buf.can_data[1];
    mobileyeData.mobileyeTSRTypeAndPosition.Sign_Position_X = can_buf.can_data[2] * 0.5;
    mobileyeData.mobileyeTSRTypeAndPosition.Sign_Position_Y = (can_buf.can_data[3] & 0x7F) * 0.5;
    mobileyeData.mobileyeTSRTypeAndPosition.Sign_Position_Z = (can_buf.can_data[4] & 0x3F) * 0.5;
    mobileyeData.mobileyeTSRTypeAndPosition.Filter_Type = can_buf.can_data[5];
    
    ROS_INFO("Vision_Only_Sign_Type: %d, Supplementary_Sign_Type: %d, Sign_Position_X: %d,Sign_Position_Y: %d, Sign_Position_Z: %d, Filter_Type: %d",
              mobileyeData.mobileyeTSRTypeAndPosition.Vision_Only_Sign_Type,mobileyeData.mobileyeTSRTypeAndPosition.Supplementary_Sign_Type,
              mobileyeData.mobileyeTSRTypeAndPosition.Sign_Position_X,mobileyeData.mobileyeTSRTypeAndPosition.Sign_Position_Y,
              mobileyeData.mobileyeTSRTypeAndPosition.Sign_Position_Z,mobileyeData.mobileyeTSRTypeAndPosition.Filter_Type);
}

void MOBILEYE::mobileyeTSRVisionDecisionPares(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype)
{
    ROS_INFO("recv: %d mobileyeTSRVisionDecisionPares", can_id);
    mobileyeData.mobileyeTSRVisionOnlyDecision.Vision_only_Sign_Type_1 = can_buf.can_data[0];
    mobileyeData.mobileyeTSRVisionOnlyDecision.Vision_only_Supplementary_Sign_Type_1 = can_buf.can_data[0];
    mobileyeData.mobileyeTSRVisionOnlyDecision.Vision_only_Sign_Type_2 = can_buf.can_data[0];
    mobileyeData.mobileyeTSRVisionOnlyDecision.Vision_only_Supplementary_Sign_Type_2 = can_buf.can_data[0];
    mobileyeData.mobileyeTSRVisionOnlyDecision.Vision_only_Sign_Type_3 = can_buf.can_data[0];
    mobileyeData.mobileyeTSRVisionOnlyDecision.Vision_only_Supplementary_Sign_Type_3 = can_buf.can_data[0];
    mobileyeData.mobileyeTSRVisionOnlyDecision.Vision_only_Sign_Type_4 = can_buf.can_data[0];
    mobileyeData.mobileyeTSRVisionOnlyDecision.Vision_only_Supplementary_Sign_Type_4 = can_buf.can_data[0];

    ROS_INFO("Vision_only_Sign_Type_1: %d, Vision_only_Sign_Type_1: %d, Vision_only_Sign_Type_1: %d, Vision_only_Sign_Type_1: %d,Vision_only_Supplementary_Sign_Type_1: %d, Vision_only_Supplementary_Sign_Type_2: %d, Vision_only_Supplementary_Sign_Type_3: %d, Vision_only_Supplementary_Sign_Type_4: %d", 
              mobileyeData.mobileyeTSRVisionOnlyDecision.Vision_only_Sign_Type_1,mobileyeData.mobileyeTSRVisionOnlyDecision.Vision_only_Sign_Type_2,
              mobileyeData.mobileyeTSRVisionOnlyDecision.Vision_only_Sign_Type_3,mobileyeData.mobileyeTSRVisionOnlyDecision.Vision_only_Sign_Type_4,
              mobileyeData.mobileyeTSRVisionOnlyDecision.Vision_only_Supplementary_Sign_Type_1,
              mobileyeData.mobileyeTSRVisionOnlyDecision.Vision_only_Supplementary_Sign_Type_2,
              mobileyeData.mobileyeTSRVisionOnlyDecision.Vision_only_Supplementary_Sign_Type_3,
              mobileyeData.mobileyeTSRVisionOnlyDecision.Vision_only_Supplementary_Sign_Type_4);
}

void MOBILEYE::mobileyeLightsLocationAndAnglesPares(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype)
{
    ROS_INFO("recv: %d mobileyeLightsLocationAndAnglesPares", can_id);
    mobileyeData.mobileyeLightsLocationAndAngles.BNDRY_DOM_BOT_NGL_HLB = can_buf.can_data[0] * 0.1 - 10;
    mobileyeData.mobileyeLightsLocationAndAngles.BNDRY_DOM_NGL_LH_HLB = (can_buf.can_data[2] << 8 | can_buf.can_data[1]) * 0.1 - 20;
    mobileyeData.mobileyeLightsLocationAndAngles.BNDRY_DOM_NGL_RH_HLB = (can_buf.can_data[3] << 4 | (can_buf.can_data[2] >> 4 & 0x0F)) * 0.1 - 20;
    mobileyeData.mobileyeLightsLocationAndAngles.OBJ_DIST_HLB = (can_buf.can_data[4] & 0x00FF) * 2;
    mobileyeData.mobileyeLightsLocationAndAngles.ST_BNDRY_DOM_BOT_NGL_HLB = can_buf.can_data[5] & 0x03;
    mobileyeData.mobileyeLightsLocationAndAngles.ST_BNDRY_DOM_NGL_LH_HLB = can_buf.can_data[5] >> 2 & 0x03;
    mobileyeData.mobileyeLightsLocationAndAngles.ST_BNDRY_DOM_NGL_RH_HLB = can_buf.can_data[5] >> 4 & 0x03;
    mobileyeData.mobileyeLightsLocationAndAngles.ST_OBJ_DIST_HLB = can_buf.can_data[5] >> 6 & 0x03;
    mobileyeData.mobileyeLightsLocationAndAngles.Left_Target_Change = can_buf.can_data[6] & 0x01;
    mobileyeData.mobileyeLightsLocationAndAngles.Right_Target_Change = can_buf.can_data[6] >> 1 & 0x01;
    mobileyeData.mobileyeLightsLocationAndAngles.Too_Many_Cars = can_buf.can_data[6] >> 2 & 0x01;
    mobileyeData.mobileyeLightsLocationAndAngles.Busy_Scene = can_buf.can_data[6] >> 3 & 0x01;

    ROS_INFO("BNDRY_DOM_BOT_NGL_HLB: %d, BNDRY_DOM_NGL_LH_HLB: %d, BNDRY_DOM_NGL_RH_HLB: %d, OBJ_DIST_HLB: %d,ST_BNDRY_DOM_BOT_NGL_HLB: %d, ST_BNDRY_DOM_NGL_LH_HLB: %d, ST_BNDRY_DOM_NGL_RH_HLB: %d, ST_OBJ_DIST_HLB: %d, Left_Target_Change: %d, Right_Target_Change: %d, Too_Many_Cars: %d, Busy_Scene: %d",
              mobileyeData.mobileyeLightsLocationAndAngles.BNDRY_DOM_BOT_NGL_HLB,mobileyeData.mobileyeLightsLocationAndAngles.BNDRY_DOM_NGL_LH_HLB,
              mobileyeData.mobileyeLightsLocationAndAngles.BNDRY_DOM_NGL_RH_HLB,mobileyeData.mobileyeLightsLocationAndAngles.OBJ_DIST_HLB,
              mobileyeData.mobileyeLightsLocationAndAngles.ST_BNDRY_DOM_BOT_NGL_HLB,mobileyeData.mobileyeLightsLocationAndAngles.ST_BNDRY_DOM_NGL_LH_HLB,
              mobileyeData.mobileyeLightsLocationAndAngles.ST_BNDRY_DOM_NGL_RH_HLB,mobileyeData.mobileyeLightsLocationAndAngles.ST_OBJ_DIST_HLB,
              mobileyeData.mobileyeLightsLocationAndAngles.Left_Target_Change,mobileyeData.mobileyeLightsLocationAndAngles.Right_Target_Change,
              mobileyeData.mobileyeLightsLocationAndAngles.Too_Many_Cars,mobileyeData.mobileyeLightsLocationAndAngles.Busy_Scene);
}

void MOBILEYE::mobileyeLaneInfoMeasureParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype)
{
    ROS_INFO("recv: %d mobileyeLaneInfoMeasureParse", can_id);
    mobileyeData.mobileyeLaneInfoMeasure.Lane_Curvature = (can_buf.can_data[0] + can_buf.can_data[1] << 8) * 3.81 * 0.000001;
    mobileyeData.mobileyeLaneInfoMeasure.Lane_Heading = (can_buf.can_data[2] + can_buf.can_data[3] << 8 & 0x0F) * 0.0005;
    mobileyeData.mobileyeLaneInfoMeasure.CA_construction_area = can_buf.can_data[3] >> 4 & 0x01;
    mobileyeData.mobileyeLaneInfoMeasure.Pitch_Angle = (((can_buf.can_data[7] << 8 | can_buf.can_data[6]) - 0x7FFF)/1024)/512;
    mobileyeData.mobileyeLaneInfoMeasure.Yaw_Angle = (can_buf.can_data[5] << 8 | can_buf.can_data[4])/1024;
    mobileyeData.mobileyeLaneInfoMeasure.Right_LDW_Availability = can_buf.can_data[3] >> 5 & 0x01;
    mobileyeData.mobileyeLaneInfoMeasure.Left_LDW_Availability = can_buf.can_data[3] >> 6 & 0x01;

    ROS_INFO("Lane_Curvature: %f, Lane_Heading: %f, CA_construction_area: %d, Pitch_Angle: %f, Yaw_Angle: %f, Right_LDW_Availability: %d, Left_LDW_Availability: %d",
              mobileyeData.mobileyeLaneInfoMeasure.Lane_Curvature,mobileyeData.mobileyeLaneInfoMeasure.Lane_Heading,
              mobileyeData.mobileyeLaneInfoMeasure.CA_construction_area,mobileyeData.mobileyeLaneInfoMeasure.Pitch_Angle,
              mobileyeData.mobileyeLaneInfoMeasure.Yaw_Angle,mobileyeData.mobileyeLaneInfoMeasure.Right_LDW_Availability,
              mobileyeData.mobileyeLaneInfoMeasure.Left_LDW_Availability);
}

void MOBILEYE::mobileyeNumberOfObstaclesParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype)
{
    ROS_INFO("recv: %d mobileyeNumberOfObstaclesParse", can_id);
    mobileyeData.mobileyeNumberOfObstacles.Number_Of_Obstacles = can_buf.can_data[0];
    mobileyeData.mobileyeNumberOfObstacles.Timestamp = can_buf.can_data[1];
    mobileyeData.mobileyeNumberOfObstacles.Left_Close_Rang_Cut_In = can_buf.can_data[3] >> 2 & 0x01;
    mobileyeData.mobileyeNumberOfObstacles.Right_Close_Rang_Cut_In = can_buf.can_data[3] >> 3 & 0x01;
    mobileyeData.mobileyeNumberOfObstacles.Go = can_buf.can_data[3] >> 4 & 0x0F;
    mobileyeData.mobileyeNumberOfObstacles.Close_car = can_buf.can_data[5] & 0x01;
    mobileyeData.mobileyeNumberOfObstacles.Failsafe = can_buf.can_data[5] >> 1 & 0x0F;

    ROS_INFO("Number_Of_Obstacles: %d, Timestamp: %d,Left_Close_Rang_Cut_In: %d,Right_Close_Rang_Cut_In: %d,Go: %d, Close_car: %d, Failsafe: %d",
              mobileyeData.mobileyeNumberOfObstacles.Number_Of_Obstacles,mobileyeData.mobileyeNumberOfObstacles.Timestamp,
              mobileyeData.mobileyeNumberOfObstacles.Left_Close_Rang_Cut_In,mobileyeData.mobileyeNumberOfObstacles.Right_Close_Rang_Cut_In,
              mobileyeData.mobileyeNumberOfObstacles.Go,mobileyeData.mobileyeNumberOfObstacles.Close_car,
              mobileyeData.mobileyeNumberOfObstacles.Failsafe);
}

void MOBILEYE::mobileyeObstaclesData_AParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype)
{
    ROS_INFO("recv: %d mobileyeObstaclesData_AParse", can_id);
    mobileyeObstaclesDataStr mobileyeObstaclesData_;
    mobileyeObstaclesData_.Obstacle_ID = can_buf.can_data[0];
    mobileyeObstaclesData_.Obstacle_Position_X = ((can_buf.can_data[2] << 8 & 0x0F00) | can_buf.can_data[1]) * 0.0625;
    mobileyeObstaclesData_.Obstacle_Position_Y = ((can_buf.can_data[4] << 8 & 0x0300) | can_buf.can_data[3]) * 0.0625;
    mobileyeObstaclesData_.Obstacle_Relative_Velocity_X = ((can_buf.can_data[6] << 8 & 0x0F00) | can_buf.can_data[5]) * 0.0625;
    mobileyeObstaclesData_.Obstacle_Type = can_buf.can_data[6] >> 4 & 0x07;
    mobileyeObstaclesData_.Obstacle_Status = can_buf.can_data[7] & 0x07;
    mobileyeObstaclesData_.Obstacle_Brake_Lights = can_buf.can_data[7] >> 3 & 0x01;
    mobileyeObstaclesData_.Cut_In_And_Out = can_buf.can_data[4] >> 5 & 0x07;
    mobileyeObstaclesData_.Blinker_Info = can_buf.can_data[4] >> 2 & 0x07;
    mobileyeObstaclesData_.Obstacle_Valid = can_buf.can_data[7] >> 6 & 0x03;

    ROS_INFO("Obstacle_ID: %d, Obstacle_Position_X: %f, Obstacle_Position_Y: %f, Obstacle_Relative_Velocity_X: %f, Obstacle_Type: %d, Obstacle_Status: %d, Obstacle_Brake_Lights: %d, Cut_In_And_Out: %d, Blinker_Info: %d, Obstacle_Valid: %d",
              mobileyeObstaclesData_.Obstacle_ID,mobileyeObstaclesData_.Obstacle_Position_X,
              mobileyeObstaclesData_.Obstacle_Position_Y,mobileyeObstaclesData_.Obstacle_Relative_Velocity_X,
              mobileyeObstaclesData_.Obstacle_Type,mobileyeObstaclesData_.Obstacle_Status,
              mobileyeObstaclesData_.Obstacle_Brake_Lights,mobileyeObstaclesData_.Cut_In_And_Out,
              mobileyeObstaclesData_.Blinker_Info,mobileyeObstaclesData_.Obstacle_Valid);

    // mobileyeData.mobileyeObstaclesData.Obstacle_ID = can_buf.data[0];
    // mobileyeData.mobileyeObstaclesData.Obstacle_Position_X = ((can_buf.data[2] << 8 & 0x0F00) | can_buf.data[1]) * 0.0625;
    // mobileyeData.mobileyeObstaclesData.Obstacle_Position_Y = ((can_buf.data[4] << 8 & 0x0300) | can_buf.data[3]) * 0.0625;
    // mobileyeData.mobileyeObstaclesData.Obstacle_Relative_Velocity_X = ((can_buf.data[6] << 8 & 0x0F00) | can_buf.data[5]) * 0.0625;
    // mobileyeData.mobileyeObstaclesData.Obstacle_Type = can_buf.data[6] >> 4 & 0x07;
    // mobileyeData.mobileyeObstaclesData.Obstacle_Status = can_buf.data[7] & 0x07;
    // mobileyeData.mobileyeObstaclesData.Obstacle_Brake_Lights = can_buf.data[7] >> 3 & 0x01;
    // mobileyeData.mobileyeObstaclesData.Cut_In_And_Out = can_buf.data[4] >> 5 & 0x07;
    // mobileyeData.mobileyeObstaclesData.Blinker_Info = can_buf.data[4] >> 2 & 0x07;
    // mobileyeData.mobileyeObstaclesData.Obstacle_Valid = can_buf.data[7] >> 6 & 0x03;

    // ROS_INFO("Obstacle_ID: %d, Obstacle_Position_X: %f, Obstacle_Position_Y: %f, Obstacle_Relative_Velocity_X: %f, Obstacle_Type: %d, 
    //           Obstacle_Status: %d, Obstacle_Brake_Lights: %d, Cut_In_And_Out: %d, Blinker_Info: %d, Obstacle_Valid: %d",
    //           mobileyeData.mobileyeObstaclesData.Obstacle_ID,mobileyeData.mobileyeObstaclesData.Obstacle_Position_X,
    //           mobileyeData.mobileyeObstaclesData.Obstacle_Position_Y,mobileyeData.mobileyeObstaclesData.Obstacle_Relative_Velocity_X,
    //           mobileyeData.mobileyeObstaclesData.Obstacle_Type,mobileyeData.mobileyeObstaclesData.Obstacle_Status,
    //           mobileyeData.mobileyeObstaclesData.Obstacle_Brake_Lights,mobileyeData.mobileyeObstaclesData.Cut_In_And_Out,
    //           mobileyeData.mobileyeObstaclesData.Blinker_Info,mobileyeData.mobileyeObstaclesData.Obstacle_Valid);
}

void MOBILEYE::mobileyeObstaclesData_BParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype)
{
    ROS_INFO("recv: %d mobileyeObstaclesData_BParse", can_id);
    mobileyeObstaclesDataStr mobileyeObstaclesData_;
    mobileyeObstaclesData_.Obstacle_Length = can_buf.can_data[0] * 0.5;
    mobileyeObstaclesData_.Obstacle_Width = can_buf.can_data[1] * 0.05;
    mobileyeObstaclesData_.Obstacle_Age = can_buf.can_data[2];
    mobileyeObstaclesData_.Obstacle_Lane = can_buf.can_data[3] & 0x03;
    mobileyeObstaclesData_.CIPV_Flag = can_buf.can_data[3] >> 2 & 0x01;
    mobileyeObstaclesData_.Radar_Position_X = ((can_buf.can_data[4] << 4 & 0x0FF0) | (can_buf.can_data[4] >> 4 & 0x0F)) * 0.0625;
    mobileyeObstaclesData_.Radar_Velocity_X = ((can_buf.can_data[6] << 8 & 0xFF00) | can_buf.can_data[5]) * 0.0625;
    mobileyeObstaclesData_.Radar_Match_Confidence = can_buf.can_data[6] >> 4 & 0x07;
    mobileyeObstaclesData_.Matched_Radar_ID = can_buf.can_data[7] & 0x7F;

    ROS_INFO("Obstacle_Length: %f, Obstacle_Width: %f, Obstacle_Age: %d, Obstacle_Lane: %d, CIPV_Flag: %d, Radar_Position_X: %f, Radar_Velocity_X: %f, Radar_Match_Confidence: %d, Matched_Radar_ID: %d",
              mobileyeObstaclesData_.Obstacle_Length,mobileyeObstaclesData_.Obstacle_Width,
              mobileyeObstaclesData_.Obstacle_Age,mobileyeObstaclesData_.Obstacle_Lane,
              mobileyeObstaclesData_.CIPV_Flag,mobileyeObstaclesData_.Radar_Position_X,
              mobileyeObstaclesData_.Radar_Velocity_X,mobileyeObstaclesData_.Radar_Match_Confidence,
              mobileyeObstaclesData_.Matched_Radar_ID);

    // mobileyeData.mobileyeObstaclesData.Obstacle_Length = can_buf.data[0] * 0.5;
    // mobileyeData.mobileyeObstaclesData.Obstacle_Width = can_buf.data[1] * 0.05;
    // mobileyeData.mobileyeObstaclesData.Obstacle_Age = can_buf.data[2];
    // mobileyeData.mobileyeObstaclesData.Obstacle_Lane = can_buf.data[3] & 0x03;
    // mobileyeData.mobileyeObstaclesData.CIPV_Flag = can_buf.data[3] >> 2 & 0x01;
    // mobileyeData.mobileyeObstaclesData.Radar_Position_X = ((can_buf.data[4] << 4 & 0x0FF0) | (can_buf.data[4] >> 4 & 0x0F) * 0.0625;
    // mobileyeData.mobileyeObstaclesData.Radar_Velocity_X = ((can_buf.data[6] << 8 & 0xFF00) | can_buf.data[5]) * 0.0625;
    // mobileyeData.mobileyeObstaclesData.Radar_Match_Confidence = can_buf.data[6] >> 4 & 0x07;
    // mobileyeData.mobileyeObstaclesData.Matched_Radar_ID = can_buf.data[7] & 0x7F;

    // ROS_INFO("Obstacle_Length: %f, Obstacle_Width: %f, Obstacle_Age: %d, Obstacle_Lane: %d, CIPV_Flag: %d, 
    //           Radar_Position_X: %f, Radar_Velocity_X: %f, Radar_Match_Confidence: %d, Matched_Radar_ID: %d",
    //           mobileyeData.mobileyeObstaclesData.Obstacle_Length,mobileyeData.mobileyeObstaclesData.Obstacle_Width,
    //           mobileyeData.mobileyeObstaclesData.Obstacle_Age,mobileyeData.mobileyeObstaclesData.Obstacle_Lane,
    //           mobileyeData.mobileyeObstaclesData.CIPV_Flag,mobileyeData.mobileyeObstaclesData.Radar_Position_X,
    //           mobileyeData.mobileyeObstaclesData.Radar_Velocity_X,mobileyeData.mobileyeObstaclesData.Radar_Match_Confidence,
    //           mobileyeData.mobileyeObstaclesData.Matched_Radar_ID);
}

void MOBILEYE::mobileyeObstaclesData_CParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype)
{
    ROS_INFO("recv: %d mobileyeObstaclesData_CParse", can_id);
    mobileyeObstaclesDataStr mobileyeObstaclesData_;
    mobileyeObstaclesData_.Obstacle_Angle_Rate = ((can_buf.can_data[1] << 8 & 0xFF00) | can_buf.can_data[0]) * 0.01;
    mobileyeObstaclesData_.Obstacle_Scale_Change = ((can_buf.can_data[3] << 8 & 0xFF00) | can_buf.can_data[2]) * 0.0002;
    mobileyeObstaclesData_.Object_Accel_X = ((can_buf.can_data[5] << 8 & 0x0300) | can_buf.can_data[4]) * 0.03;
    mobileyeObstaclesData_.Obstacle_Replaced = can_buf.can_data[5] >> 4 & 0x01;
    mobileyeObstaclesData_.Obstacle_Angle = ((can_buf.can_data[7] << 8 & 0xFF00) | can_buf.can_data[6]) * 0.01;
    
    mobileyeData.mobileyeObstaclesData.push_back(mobileyeObstaclesData_);

    ROS_INFO("Obstacle_Angle_Rate: %f,Obstacle_Scale_Change: %f,Object_Accel_X: %f,Obstacle_Replaced: %d,Obstacle_Angle: %f",
              mobileyeObstaclesData_.Obstacle_Angle_Rate,mobileyeObstaclesData_.Obstacle_Scale_Change,
              mobileyeObstaclesData_.Object_Accel_X,mobileyeObstaclesData_.Obstacle_Replaced,
              mobileyeObstaclesData_.Obstacle_Angle);

    // mobileyeData.mobileyeObstaclesData.Obstacle_Angle_Rate = ((can_buf.data[1] << 8 & 0xFF00) | can_buf.data[0]) * 0.01;
    // mobileyeData.mobileyeObstaclesData.Obstacle_Scale_Change = ((can_buf.data[3] << 8 & 0xFF00) | can_buf.data[2]) * 0.0002;
    // mobileyeData.mobileyeObstaclesData.Object_Accel_X = ((can_buf.data[5] << 8 & 0x0300) | can_buf.data[4]) * 0.03;
    // mobileyeData.mobileyeObstaclesData.Obstacle_Replaced = can_buf.data[5] >> 4 & 0x01;
    // mobileyeData.mobileyeObstaclesData.Obstacle_Angle = ((can_buf.data[7] << 8 & 0xFF00) | can_buf.data[6]) * 0.01;

    // ROS_INFO("Obstacle_Angle_Rate: %f,Obstacle_Scale_Change: %f,Object_Accel_X: %f,Obstacle_Replaced: %d,Obstacle_Angle: %f",
    //           mobileyeData.mobileyeObstaclesData.Obstacle_Angle_Rate,mobileyeData.mobileyeObstaclesData.Obstacle_Scale_Change,
    //           mobileyeData.mobileyeObstaclesData.Object_Accel_X,mobileyeData.mobileyeObstaclesData.Obstacle_Replaced,
    //           mobileyeData.mobileyeObstaclesData.Obstacle_Angle);
}

void MOBILEYE::mobileyeSignalsStatusVehicleParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype)
{
    ROS_INFO("recv: %d mobileyeSignalsStatusVehicleParse", can_id);
    mobileyeData.mobileyeSignalsStatus.High_Beam = can_buf.can_data[0] >> 5 & 0x01;
    mobileyeData.mobileyeSignalsStatus.Low_Beam = can_buf.can_data[0] >> 4 & 0x01;
    mobileyeData.mobileyeSignalsStatus.Wipers = can_buf.can_data[0] >> 3 & 0x01;
    mobileyeData.mobileyeSignalsStatus.Right_signal = can_buf.can_data[0] >> 2 & 0x01;
    mobileyeData.mobileyeSignalsStatus.Left_signal = can_buf.can_data[0] >> 1 & 0x01;
    mobileyeData.mobileyeSignalsStatus.Brake_signal = can_buf.can_data[0] & 0x01;
    mobileyeData.mobileyeSignalsStatus.Wipers_available = can_buf.can_data[1] >> 3 & 0x01;
    mobileyeData.mobileyeSignalsStatus.Low_Beam_available = can_buf.can_data[1] >> 4 & 0x01;
    mobileyeData.mobileyeSignalsStatus.High_Beam_Available = can_buf.can_data[1] >> 5 & 0x01;
    mobileyeData.mobileyeSignalsStatus.Speed_Available = can_buf.can_data[1] >> 7 & 0x01;
    mobileyeData.mobileyeSignalsStatus.Speed = can_buf.can_data[2];

    ROS_INFO("High_Beam: %d, Low_Beam: %d, Wipers: %d, Right_signal: %d, Left_signal: %d, Brake_signal: %d,Wipers_available: %d, Low_Beam_available: %d, High_Beam_Available: %d, Speed_Available: %d, Speed: %d",
              mobileyeData.mobileyeSignalsStatus.High_Beam,mobileyeData.mobileyeSignalsStatus.Low_Beam,
              mobileyeData.mobileyeSignalsStatus.Wipers,mobileyeData.mobileyeSignalsStatus.Right_signal,
              mobileyeData.mobileyeSignalsStatus.Left_signal,mobileyeData.mobileyeSignalsStatus.Brake_signal,
              mobileyeData.mobileyeSignalsStatus.Wipers_available,mobileyeData.mobileyeSignalsStatus.Low_Beam_available,
              mobileyeData.mobileyeSignalsStatus.High_Beam_Available,mobileyeData.mobileyeSignalsStatus.Speed_Available,
              mobileyeData.mobileyeSignalsStatus.Speed);
}

void MOBILEYE::mobileyeLKALeftLane_AParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype)
{
    ROS_INFO("recv: %d mobileyeLKALeftLane_AParse", can_id);
    mobileyeLKALane_.Lane_type = can_buf.can_data[0] & 0x0F;
    mobileyeLKALane_.Quality = can_buf.can_data[0] >> 4 & 0x03;
    mobileyeLKALane_.Model_degree = can_buf.can_data[0] >> 6 & 0x03;
    mobileyeLKALane_.Position_Parameter_C0 = (can_buf.can_data[2] << 8 & 0xFF00 | can_buf.can_data[1]) / 256;
    mobileyeLKALane_.Curvature_Parameter_C2 = ((can_buf.can_data[4] << 8 & 0xFF00 | can_buf.can_data[3]) - 0x7FFF) / 1024 / 1000;
    mobileyeLKALane_.Curvature_Derivative_Parameter_C3 = ((can_buf.can_data[6] << 8 & 0xFF00 | can_buf.can_data[5]) - 0x7FFF) / (1 << 28);
    mobileyeLKALane_.Width_Marking = can_buf.can_data[7] * 0.01;

    ROS_INFO("Lane_type: %d, Quality: %d, Model_degree: %d, Position_Parameter_C0: %f,Curvature_Parameter_C2: %f, Curvature_Derivative_Parameter_C3: %f, Width_Left_Marking: %f",
              mobileyeLKALane_.Lane_type,mobileyeLKALane_.Quality,mobileyeLKALane_.Model_degree,
              mobileyeLKALane_.Position_Parameter_C0,mobileyeLKALane_.Curvature_Parameter_C2,
              mobileyeLKALane_.Curvature_Derivative_Parameter_C3,mobileyeLKALane_.Width_Marking);

    // mobileyeData.mobileyeLKALeftLane.Lane_type = can_buf.data[0] & 0x0F;
    // mobileyeData.mobileyeLKALeftLane.Quality = can_buf.data[0] >> 4 & 0x03;
    // mobileyeData.mobileyeLKALeftLane.Model_degree = can_buf.data[0] >> 6 & 0x03;
    // mobileyeData.mobileyeLKALeftLane.Position_Parameter_C0 = (can_buf.data[2] << 8 & 0xFF00 | can_buf.data[1]) / 256;
    // mobileyeData.mobileyeLKALeftLane.Curvature_Parameter_C2 = ((can_buf.data[4] << 8 & 0xFF00 | can_buf.data[3]) - 0x7FFF) / 1024 / 1000;
    // mobileyeData.mobileyeLKALeftLane.Curvature_Derivative_Parameter_C3 = ((can_buf.data[6] << 8 & 0xFF00 | can_buf.data[5]) - 0x7FFF) / (1 << 28);
    // mobileyeData.mobileyeLKALeftLane.Width_Left_Marking = can_buf.data[7] * 0.01;

    // ROS_INFO("Lane_type: %d, Quality: %d, Model_degree: %d, Position_Parameter_C0: %f,
    //           Curvature_Parameter_C2: %f, Curvature_Derivative_Parameter_C3: %f, Width_Left_Marking: %f",
    //           mobileyeData.mobileyeLKALeftLane.Lane_type,mobileyeData.mobileyeLKALeftLane.Quality,mobileyeData.mobileyeLKALeftLane.Model_degree
    //           mobileyeData.mobileyeLKALeftLane.Position_Parameter_C0,mobileyeData.mobileyeLKALeftLane.Curvature_Parameter_C2,
    //           mobileyeData.mobileyeLKALeftLane.Curvature_Derivative_Parameter_C3,mobileyeData.mobileyeLKALeftLane.Width_Left_Marking);
}

void MOBILEYE::mobileyeLKALeftLane_BParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype)
{
    ROS_INFO("recv: %d mobileyeLKALeftLane_BParse", can_id);
    mobileyeLKALane_.Heading_Angle_Parameter_C1 = ((can_buf.can_data[1] << 8 & 0xFF00 | can_buf.can_data[0]) - 0x7FFF) / 1024;
    mobileyeLKALane_.View_Range = (can_buf.can_data[3] << 8 & 0x7F00 | can_buf.can_data[2]) / 256;
    mobileyeLKALane_.View_range_availability = can_buf.can_data[3] >> 7 & 0x01;

    mobileyeData.mobileyeLKALeftLane.push_back(mobileyeLKALane_);

    ROS_INFO("Heading_Angle_Parameter_C1: %f, View_Range: %f, View_range_availability: %d",
              mobileyeLKALane_.Heading_Angle_Parameter_C1,
              mobileyeLKALane_.View_Range,
              mobileyeLKALane_.View_range_availability);

    // mobileyeData.mobileyeLKALeftLane.Heading_Angle_Parameter_C1 = ((can_buf.data[1] << 8 & 0xFF00 | can_buf.data[0]) - 0x7FFF) / 1024;
    // mobileyeData.mobileyeLKALeftLane.View_Range = (can_buf.data[3] << 8 & 0x7F00 | can_buf.data[2]) / 256;
    // mobileyeData.mobileyeLKALeftLane.View_range_availability = can_buf.data[3] >> 7 & 0x01;

    // ROS_INFO("Heading_Angle_Parameter_C1: %f, View_Range: %f, View_range_availability: %d",
    //           mobileyeData.mobileyeLKALeftLane.Heading_Angle_Parameter_C1,
    //           mobileyeData.mobileyeLKALeftLane.View_Range,
    //           mobileyeData.mobileyeLKALeftLane.View_range_availability);
}

void MOBILEYE::mobileyeLKARightLane_AParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype)
{
    ROS_INFO("recv: %d mobileyeLKARightLane_AParse", can_id);
    mobileyeLKALane_.Lane_type = can_buf.can_data[0] & 0x0F;
    mobileyeLKALane_.Quality = can_buf.can_data[0] >> 4 & 0x03;
    mobileyeLKALane_.Model_degree = can_buf.can_data[0] >> 6 & 0x03;
    mobileyeLKALane_.Position_Parameter_C0 = (can_buf.can_data[2] << 8 & 0xFF00 | can_buf.can_data[1]) / 256;
    mobileyeLKALane_.Curvature_Parameter_C2 = ((can_buf.can_data[4] << 8 & 0xFF00 | can_buf.can_data[3]) - 0x7FFF) / 1024 / 1000;
    mobileyeLKALane_.Curvature_Derivative_Parameter_C3 = ((can_buf.can_data[6] << 8 & 0xFF00 | can_buf.can_data[5]) - 0x7FFF) / (1 << 28);
    mobileyeLKALane_.Width_Marking = can_buf.can_data[7] * 0.01;

    ROS_INFO("Lane_type: %d, Quality: %d, Model_degree: %d, Position_Parameter_C0: %f,Curvature_Parameter_C2: %f, Curvature_Derivative_Parameter_C3: %f, Width_Left_Marking: %f",
              mobileyeLKALane_.Lane_type,mobileyeLKALane_.Quality,mobileyeLKALane_.Model_degree,
              mobileyeLKALane_.Position_Parameter_C0,mobileyeLKALane_.Curvature_Parameter_C2,
              mobileyeLKALane_.Curvature_Derivative_Parameter_C3,mobileyeLKALane_.Width_Marking);

    // mobileyeData.mobileyeLKARightLane.Lane_type = can_buf.data[0] & 0x0F;
    // mobileyeData.mobileyeLKARightLane.Quality = can_buf.data[0] >> 4 & 0x03;
    // mobileyeData.mobileyeLKARightLane.Model_degree = can_buf.data[0] >> 6 & 0x03;
    // mobileyeData.mobileyeLKARightLane.Position_Parameter_C0 = (can_buf.data[2] << 8 & 0xFF00 | can_buf.data[1]) / 256;
    // mobileyeData.mobileyeLKARightLane.Curvature_Parameter_C2 = ((can_buf.data[4] << 8 & 0xFF00 | can_buf.data[3]) - 0x7FFF) / 1024 / 1000;
    // mobileyeData.mobileyeLKARightLane.Curvature_Derivative_Parameter_C3 = ((can_buf.data[6] << 8 & 0xFF00 | can_buf.data[5]) - 0x7FFF) / (1 << 28);
    // mobileyeData.mobileyeLKARightLane.Width_Left_Marking = can_buf.data[7] * 0.01;

    // ROS_INFO("Lane_type: %d, Quality: %d, Model_degree: %d, Position_Parameter_C0: %f,
    //           Curvature_Parameter_C2: %f, Curvature_Derivative_Parameter_C3: %f, Width_Left_Marking: %f",
    //           mobileyeData.mobileyeLKARightLane.Lane_type,mobileyeData.mobileyeLKARightLane.Quality,mobileyeData.mobileyeLKARightLane.Model_degree
    //           mobileyeData.mobileyeLKARightLane.Position_Parameter_C0,mobileyeData.mobileyeLKARightLane.Curvature_Parameter_C2,
    //           mobileyeData.mobileyeLKARightLane.Curvature_Derivative_Parameter_C3,mobileyeData.mobileyeLKARightLane.Width_Left_Marking);
}

void MOBILEYE::mobileyeLKARightLane_BParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype)
{
    ROS_INFO("recv: %d mobileyeLKARightLane_BParse", can_id);
    mobileyeLKALane_.Heading_Angle_Parameter_C1 = ((can_buf.can_data[1] << 8 & 0xFF00 | can_buf.can_data[0]) - 0x7FFF) / 1024;
    mobileyeLKALane_.View_Range = (can_buf.can_data[3] << 8 & 0x7F00 | can_buf.can_data[2]) / 256;
    mobileyeLKALane_.View_range_availability = can_buf.can_data[3] >> 7 & 0x01;

    mobileyeData.mobileyeLKARightLane.push_back(mobileyeLKALane_);

    ROS_INFO("Heading_Angle_Parameter_C1: %f, View_Range: %f, View_range_availability: %d",
              mobileyeLKALane_.Heading_Angle_Parameter_C1,
              mobileyeLKALane_.View_Range,
              mobileyeLKALane_.View_range_availability);

    // mobileyeData.mobileyeLKARightLane.Heading_Angle_Parameter_C1 = ((can_buf.data[1] << 8 & 0xFF00 | can_buf.data[0]) - 0x7FFF) / 1024;
    // mobileyeData.mobileyeLKARightLane.View_Range = (can_buf.data[3] << 8 & 0x7F00 | can_buf.data[2]) / 256;
    // mobileyeData.mobileyeLKARightLane.View_range_availability = can_buf.data[3] >> 7 & 0x01;

    // ROS_INFO("Heading_Angle_Parameter_C1: %f, View_Range: %f, View_range_availability: %d",
    //           mobileyeData.mobileyeLKARightLane.Heading_Angle_Parameter_C1,
    //           mobileyeData.mobileyeLKARightLane.View_Range,
    //           mobileyeData.mobileyeLKARightLane.View_range_availability);
}

void MOBILEYE::mobileyeReference_PointsParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype)
{
    ROS_INFO("recv: %d mobileyeReference_PointsParse", can_id);
    mobileyeData.mobileyeReferencePoints.Ref_Point_1_Position = (((can_buf.can_data[1] << 8 | 0xFF00) & can_buf.can_data[0]) - 0x7FFF) / 256;
    mobileyeData.mobileyeReferencePoints.Ref_Point_1_Distance = ((can_buf.can_data[3] << 8 | 0x7F00) & can_buf.can_data[2]) / 256;
    mobileyeData.mobileyeReferencePoints.Ref_Point_1_Validity = can_buf.can_data[3] >> 7 & 0x01;

    mobileyeData.mobileyeReferencePoints.Ref_Point_2_Position = (((can_buf.can_data[5] << 8 | 0xFF00) & can_buf.can_data[4]) - 0x7FFF) / 256;
    mobileyeData.mobileyeReferencePoints.Ref_Point_2_Distance = ((can_buf.can_data[7] << 8 | 0x7F00) & can_buf.can_data[6]) / 256;
    mobileyeData.mobileyeReferencePoints.Ref_Point_2_Validity = can_buf.can_data[7] >> 7 & 0x01;

    ROS_INFO("Ref_Point_1_Position: %f, Ref_Point_1_Distance: %f, Ref_Point_1_Validity: %d,Ref_Point_2_Position: %f, Ref_Point_2_Distance: %f, Ref_Point_2_Validity: %d",
              mobileyeData.mobileyeReferencePoints.Ref_Point_1_Position,
              mobileyeData.mobileyeReferencePoints.Ref_Point_1_Distance,
              mobileyeData.mobileyeReferencePoints.Ref_Point_1_Validity,
              mobileyeData.mobileyeReferencePoints.Ref_Point_2_Position,
              mobileyeData.mobileyeReferencePoints.Ref_Point_2_Distance,
              mobileyeData.mobileyeReferencePoints.Ref_Point_2_Validity);
}

void MOBILEYE::mobileyeNunber_Of_Next_LaneParse(adcuCanData &can_buf, uint32_t &can_id, canOrder &can_order, uint8_t ch_, uint16_t &contype)
{
    ROS_INFO("recv: %d mobileyeNunber_Of_Next_LaneParse", can_id);
    mobileyeData.mobileyeNUmberOfNextLane.Number_Of_Next_Lane_Markers_Reported = can_buf.can_data[0];

    ROS_INFO("Number_Of_Next_Lane_Markers_Reported: %d",mobileyeData.mobileyeNUmberOfNextLane.Number_Of_Next_Lane_Markers_Reported);
}

}
}
