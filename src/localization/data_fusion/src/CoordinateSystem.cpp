#include <ros/ros.h>
#include <cmath>
#include "CoordinateSystem.h"
#include "DataType.h"

enum SensorType
{
    integrated_nav = 1,
    uwb = 2,
    lidar = 3,
    fixed_lidar = 4
};

// **************
// 功能:经纬高转WGS84的ECEF坐标
// 输入:llh_pos[3] 纬经高
// 输出:WGSpose WGS84的ECEF坐标
// 无返回
// ***************
void transLLHtoWGS84(double llh_pos[3], double WGSpose[3])
{

    double lat_radian = llh_pos[0] * M_PIl / 180; //维度弧度
    double lon_radian = llh_pos[1] * M_PIl / 180; //经度弧度
    double hgt_ = llh_pos[2];                    //  高度
    double ex2 = (2 - ff) * ff; //e（椭球偏心率）的平方  //e:0.006694384999588
    double N = Ra / sqrt(1.0 - ex2 * pow(sin(lat_radian),2));
    WGSpose[0] = (N+hgt_)*cos(lat_radian)*cos(lon_radian); //  x
    WGSpose[1] = (N+hgt_)*cos(lat_radian)*sin(lon_radian); //  y
    WGSpose[2] = (N*(1-ex2)+hgt_)*sin(lat_radian);       //z
    
}

// **************
// 功能:WGS84的ECEF坐标转经纬高
// 输入:WGSpose WGS84的ECEF坐标
// 输出:llh_pos[3] 纬经高
// 无返回
// ***************
void transWGS84toLLH(double WGS[3], double llh_pos[3])
{

    int interation_max = 10;
    double lon = atan2l(WGS[1], WGS[0]);
    double p = sqrt(WGS[0] * WGS[0] + WGS[1] * WGS[1]);
    double ex2 = (2 - ff) * ff;
    double lat = 0;
    double hgt, N;

    for (int i = 0; i < interation_max; i++)
    {
        N = Ra / sqrt(1.0 - ex2 * sin(lat) * sin(lat));
        hgt = (double)p / cos(lat) - N;
        lat = atan(WGS[2] / p / (1 - ex2 * N / (N + hgt)));
    }

    llh_pos[0] = lat * 180 / M_PIl; //纬度弧度转角度
    llh_pos[1] = lon * 180 / M_PIl; //经度弧度转角度
    llh_pos[2] = hgt;
}

// **************
// 功能:站心东北天坐标转WGS84的ECEF坐标
// 输入:venu[3]站心东北天(m),llh_pos[3] 纬经高
// 输出:WGSpose WGS84的ECEF坐标
// 无返回
// ***************
void transENUtoWGS84(double venu[3], double llh_pos[3], double WGSpose[3])
{

    double lat_radian = llh_pos[0] * M_PIl / 180; //纬度弧度
    double lon_radian = llh_pos[1] * M_PIl / 180; //经度弧度
    Matrix3d coord_s;                            //WGS-84到ENU的坐标变换矩阵
    Matrix3d coord_s_inv; //WGS-84到ENU的坐标变换矩阵
    coord_s_inv.Zero(3, 3);

    coord_s(0, 0) = -sin(lon_radian);
    coord_s(0, 1) = cos(lon_radian);
    coord_s(0, 2) = 0.0;
    coord_s(1, 0) = -sin(lat_radian) * cos(lon_radian);
    coord_s(1, 1) = -sin(lat_radian) * sin(lon_radian);
    coord_s(1, 2) = cos(lat_radian);
    coord_s(2, 0) = cos(lat_radian) * cos(lon_radian);
    coord_s(2, 1) = cos(lat_radian) * sin(lon_radian);
    coord_s(2, 2) = sin(lat_radian);

    coord_s_inv = coord_s.inverse();

    WGSpose[0] = (double)coord_s_inv(0, 0) * venu[0] + coord_s_inv(0, 1) * venu[1] + coord_s_inv(0, 2) * venu[2]; //  x
    WGSpose[1] = (double)coord_s_inv(1, 0) * venu[0] + coord_s_inv(1, 1) * venu[1] + coord_s_inv(1, 2) * venu[2]; //  y
    WGSpose[2] = (double)coord_s_inv(2, 0) * venu[0] + coord_s_inv(2, 1) * venu[1] + coord_s_inv(2, 2) * venu[2]; //  z
}

// **************
// 功能:WGS84的ECEF坐标转站心东北天坐标
// 输入:WGSpose WGS84的ECEF坐标(m),llh_pos[3] 纬经高
// 输出:venu[3]站心东北天(m)
// 无返回
// ***************
void transWGS84toENU(double WGSpose[3], double llh_pos[3], double venu[3])
{
    double lat_radian = llh_pos[0] * M_PIl / 180; //纬度弧度
    double lon_radian = llh_pos[1] * M_PIl / 180; //经度弧度
    Matrix3d coord_s;                            //WGS-84到ENU的坐标变换矩阵
    coord_s(0, 0) = -sin(lon_radian);
    coord_s(0, 1) = cos(lon_radian);
    coord_s(0, 2) = 0.0;
    coord_s(1, 0) = -sin(lat_radian) * cos(lon_radian);
    coord_s(1, 1) = -sin(lat_radian) * sin(lon_radian);
    coord_s(1, 2) = cos(lat_radian);
    coord_s(2, 0) = cos(lat_radian) * cos(lon_radian);
    coord_s(2, 1) = cos(lat_radian) * sin(lon_radian);
    coord_s(2, 2) = sin(lat_radian);

    venu[0] = (double)coord_s(0, 0) * WGSpose[0] + coord_s(0, 1) * WGSpose[1] + coord_s(0, 2) * WGSpose[2]; //  ve
    venu[1] = (double)coord_s(1, 0) * WGSpose[0] + coord_s(1, 1) * WGSpose[1] + coord_s(1, 2) * WGSpose[2]; //  vn
    venu[2] = (double)coord_s(2, 0) * WGSpose[0] + coord_s(2, 1) * WGSpose[1] + coord_s(2, 2) * WGSpose[2]; //  vu
}

// **************
// 功能:经纬高转UTM平面投影坐标东北天(3度带)
// 输入:WGS84的LLH 纬经高(deg)
// 输出:UTM_ENU平面投影坐标东北天(m)
// 返回当前中央子午线经度(deg)
// ***************
double transForLLHtoUTM(double (&LLH)[3],double (&UTM_ENU)[3])
{
    //6度带中央子午线经度
    // double lon0,lon0_rad;    //中央子午线
    // if (-180 < LLH[1] < 0)
    // {
    //     lon0 = ((180-6*floor(fabs(LLH[1])/6)-6)/6+1)*6-180-3;   //deg
    //     lon0_rad = lon0*M_PI/180; 
    // }
    // else if (0 < LLH[1] < 180)
    // {
    //     lon0 = ((LLH[1]-fmod(LLH[1],6.0)+180)/6+1)*6-180-3;
    //     lon0_rad = lon0*M_PI/180; 
    // }
    // else if (LLH[1] == -180)
    // {
    //     lon0 = -177;
    //     lon0_rad = lon0*M_PI/180; 
    // }
    // else if (LLH[1] == 180)
    // {
    //     lon0 = 177;
    //     lon0_rad = lon0*M_PI/180; 
    // }
    // else
    // {
    //     lon0 = -3;
    //     lon0_rad = lon0*M_PI/180; 
    // }

    //3度带中央子午线
    double lon0,lon0_rad;    //中央子午线
    lon0 = floor((LLH[1] + 1.5)/3)*3;
    lon0_rad = lon0*M_PIl/180;

    double lan_rad,lon_rad;
    lan_rad = LLH[0]*M_PIl/180;
    lon_rad = LLH[1]*M_PIl/180;
    
    double n;
    n = ff/(2-ff);
    double ex2 = (2 - ff) * ff; //e（椭球偏心率）的平方
    double param_A,param_B,param_C,param_D,param_E,param_S,param_T3,param_T4,param_T8;
    param_A = Ra*(1-n+5*(pow(n,2)-pow(n,3))/4+81*(pow(n,4)-pow(n,5))/64);
    param_B = 3*Ra*(n-pow(n,2)+7*(pow(n,3)-pow(n,4))/8+55*(pow(n,5)-pow(n,6))/64)/2;
    param_C = 15*Ra*(pow(n,2)-pow(n,3)+3*(pow(n,4)-pow(n,5))/4)/16;
    param_D = 35*Ra*(pow(n,3)-pow(n,4)+11*(pow(n,5)-pow(n,6))/16)/48;
    param_E = 315*Ra*(pow(n,4)-pow(n,5))/512;
    param_S = param_A*lan_rad - param_B*sin(2*lan_rad) + param_C*sin(4*lan_rad) - param_D*sin(6*lan_rad) + param_E*sin(8*lan_rad);
    double RN,RM;   //卯酉线曲率半径,子午线曲率半径
    RN = Ra/sqrt(1-ex2*pow(sin(lan_rad),2));
    RM = Ra*(1-ex2)/pow(sqrt(1-ex2*pow(sin(lan_rad),2)),3);
    double cos_lan,tan_lan,sin_lan;
    cos_lan = cos(lan_rad);
    tan_lan = tan(lan_rad);
    sin_lan = sin(lan_rad);
    double cos2_lan,tan2_lan,sin2_lan;
    cos2_lan = pow(cos(lan_rad),2);
    tan2_lan = pow(tan(lan_rad),2);
    sin2_lan = pow(sin(lan_rad),2);
    param_T3 = RN*sin_lan*pow(cos_lan,3)*ratio_k0
               *
               (5-tan2_lan+9*e_sec2*cos2_lan+4*pow(e_sec2,2)*pow(cos_lan,4))/24;
    param_T4 = RN*sin_lan*pow(cos_lan,5)*ratio_k0
               *
               (61-58*tan2_lan+pow(tan_lan,4)+270*e_sec2*cos2_lan
               - 330*tan2_lan*e_sec2*cos2_lan+445*pow(e_sec2,2)*pow(cos_lan,4)+324*pow(e_sec2,3)*pow(cos_lan,6)
               - 680*tan2_lan*pow(e_sec2,2)*pow(cos_lan,4)+88*pow(e_sec2,4)*pow(cos_lan,8)-600*tan2_lan*pow(e_sec2,3)*pow(cos_lan,6)
               - 192*tan2_lan*pow(e_sec2,4)*pow(cos_lan,8))/720;
    param_T8 = RN*pow(cos_lan,5)*ratio_k0
               *
               (5-18*tan2_lan+pow(tan_lan,4)+14*e_sec2*cos2_lan-58*tan2_lan*e_sec2*cos2_lan
               + 13*pow(e_sec2,2)*pow(cos_lan,4)+4*pow(e_sec2,3)*pow(cos_lan,6)-64*tan2_lan*pow(e_sec2,2)*pow(cos_lan,4)
               - 24*tan2_lan*pow(e_sec2,3)*pow(cos_lan,6))/120;
    double p;
    p = (LLH[1]-lon0)*3600;
    
    //N,E方向平面坐标
    double sin_1s = M_PIl/(180*60*60);   //1弧度的正弦值
    //double sin_1s = 4.8481368e-6;   //1弧度的正弦值

    double tp_E,tp_N;
    if (LLH[0]>0)
    {
        UTM_ENU[1] = param_S*ratio_k0 + RN*sin_lan*cos_lan*ratio_k0*pow(sin_1s,2)*pow(p,2)/2
            + param_T3*pow(sin_1s,4)*pow(p,4)+param_T4*pow(sin_1s,6)*pow(p,6);
    }
    else
    {
        tp_N = param_S*ratio_k0 + RN*sin_lan*cos_lan*ratio_k0*pow(sin_1s,2)*pow(p,2)/2
            + param_T3*pow(sin_1s,4)*pow(p,4)+param_T4*pow(sin_1s,6)*pow(p,6);
        UTM_ENU[1] = tp_N + 10000000;
    }
    
    tp_E = RN*cos_lan*ratio_k0*sin_1s*p + RN*pow(cos_lan,3)*ratio_k0*(1-tan2_lan+e_sec2*cos2_lan)
            *
            pow(sin_1s,3)*pow(p,3)/6 + param_T8*pow(sin_1s,5)*pow(p,5);
    UTM_ENU[0] = tp_E + 500000;
    //printf("E N=%f %f\n",UTM_ENU[0],UTM_ENU[1]);

    return lon0;
    
}

// void transForLLHtoUTM(double (&LLH)[3],double (&ENU)[3])
// {
//     //3度带中央子午线
//     double lon0,lon0_rad;    //中央子午线
//     lon0 = floor((LLH[1] + 1.5)/3)*3;
//     lon0_rad = lon0*M_PIl/180;

//     double lan_rad,lon_rad;
//     lan_rad = LLH[0]*M_PIl/180;
//     lon_rad = LLH[1]*M_PIl/180;

//     double e = 0.0848192;
//     double ex2 = (2.0 - ff) * ff; //e（椭球偏心率）的平方
//     double v = 1.0/sqrt(1-ex2*pow(sin(lan_rad),2));
//     double A = (lon_rad - lon0_rad)*cos(lan_rad);
//     double S = (1.0-ex2/4.0-3.0*pow(ex2,2)/64.0-5.0*pow(ex2,3)/256.0)*lan_rad-(3.0*ex2/8.0+3.0*pow(ex2,2)/32.0+45.0*pow(ex2,3)/1024.0)*sin(2*lan_rad)
//                 + (15.0*pow(ex2,2)/256.0+45.0*pow(ex2,3)/1024.0)*sin(4*lan_rad)-35.0*pow(ex2,3)/3072.0*sin(6*lan_rad);
//     double T = pow(tan(lan_rad),2);
//     double C = ex2*pow(cos(lan_rad),2)/(1-ex2);
//     double k0 =1;
//     ENU[0] = k0*Ra*v*(A+(1-T+C)*pow(A,3)/6+(5-18*T+T*T)*pow(A,5)/120) + 500000.0;
//     ENU[1] = k0*Ra*(S+v*tan(lan_rad)*(A*A/2+(5.0-T+9.0*C+4.0*C*C)*pow(A,4)/24.0+(61.0-58.0*T+T*T)*pow(A,6)/720.0));
//     printf("E N=%f %f\n",ENU[0],ENU[1]);
// }

// **************
// 功能:UTM平面投影坐标东北天(3度带)转经纬高
// 输入:UTM_ENU平面投影坐标东北天(m)
// 输出:WGS84的LLH 纬经高(deg)
// 无返回
// ***************
void transForUTMtoLLH(double (&UTM_ENU)[3],double (&LLH)[3],double &lon0)
{
    
    //3度带中央子午线
    double lon0_rad;    //中央子午线(rad)
    lon0_rad = lon0*M_PIl/180;         
    //lon0_rad = lontitude0*M_PIl/180;

    double n;
    n = ff/(2-ff);
    double ex2 = (2 - ff) * ff; //e（椭球偏心率）的平方
    double param_A,param_B,param_C,param_D,param_E,param_S,param_T3,param_T4,param_T8;
    param_A = Ra*(1-n+5*(pow(n,2)-pow(n,3))/4+81*(pow(n,4)-pow(n,5))/64);
    param_B = 3*Ra*(n-pow(n,2)+7*(pow(n,3)-pow(n,4))/8+55*(pow(n,5)-pow(n,6))/64)/2;
    param_C = 15*Ra*(pow(n,2)-pow(n,3)+3*(pow(n,4)-pow(n,5))/4)/16;
    param_D = 35*Ra*(pow(n,3)-pow(n,4)+11*(pow(n,5)-pow(n,6))/16)/48;
    param_E = 315*Ra*(pow(n,4)-pow(n,5))/512;
    double q;
    q = fabs(UTM_ENU[0] - 500000);
    double sin_1s = M_PI/(180*60*60);   //1弧度的正弦值

    double lan_smlar=0.5; //lat为当地纬度在中央子午线上的投影纬度(弧度)
    double lan_rad,lon_rad; //当地纬度,经度(弧度)
    double RN,RM;   //卯酉线曲率半径,子午线曲率半径
    double param_T11,param_T12,param_T16;
    double tp;
    do
    {
        tp = lan_smlar;
        double tp_lan_smlar;
        tp_lan_smlar = (UTM_ENU[1]/ratio_k0+param_B*sin(2*lan_smlar)-param_C*sin(4*lan_smlar)+param_D*sin(6*lan_smlar)
                    -param_E*sin(8*lan_smlar))/param_A;
        lan_smlar = tp_lan_smlar;
    } while (fabs(lan_smlar-tp)>1e-8);
    RN = Ra/sqrt(1-ex2*pow(sin(lan_smlar),2));
    RM = Ra*(1-ex2)/pow(sqrt(1-ex2*pow(sin(lan_smlar),2)),3);
    double cos_lan,tan_lan,sin_lan;
    cos_lan = cos(lan_smlar);
    tan_lan = tan(lan_smlar);
    sin_lan = sin(lan_smlar);
    double cos2_lan,tan2_lan,sin2_lan;
    cos2_lan = pow(cos(lan_smlar),2);
    tan2_lan = pow(tan(lan_smlar),2);
    sin2_lan = pow(sin(lan_smlar),2);
    param_T11 = tan_lan*(5+3*tan2_lan+e_sec2*cos2_lan-4*pow(e_sec2,2)*pow(cos_lan,4)-9*tan2_lan*e_sec2*cos2_lan)
                /
                (24*RM*pow(RN,3)*pow(ratio_k0,4));
    param_T12 = tan_lan*(61+90*tan2_lan+45*pow(tan_lan,4)+46*e_sec2*cos2_lan-252*tan2_lan*e_sec2*cos2_lan
                - 3*pow(e_sec2,2)*pow(cos_lan,4)+100*pow(e_sec2,3)*pow(cos_lan,6)-66*tan2_lan*pow(e_sec2,2)*pow(cos_lan,4)
                + 88*pow(e_sec2,4)*pow(cos_lan,8)-90*pow(tan_lan,4)*e_sec2*cos2_lan+225*pow(tan_lan,4)*pow(e_sec2,2)*pow(cos_lan,4)
                + 84*tan2_lan*pow(e_sec2,3)*pow(cos_lan,6)-192*tan2_lan*pow(e_sec2,4)*pow(cos_lan,8))
                /
                (720*RM*pow(RN,5)*pow(ratio_k0,6));
    param_T16 = (5+28*tan2_lan+24*pow(tan_lan,4)-3*pow(e_sec2,2)*pow(cos_lan,4)+8*tan2_lan*e_sec2*cos2_lan
                + 6*e_sec2*cos2_lan-4*pow(e_sec2,3)*pow(cos_lan,6)+4*tan2_lan*pow(e_sec2,2)*pow(cos_lan,4)
                + 24*tan2_lan*pow(e_sec2,3)*pow(cos_lan,6))
                /
                (120*pow(RN,5)*cos_lan*pow(ratio_k0,5));

    lan_rad = lan_smlar - tan_lan*pow(q,2)/(2*RM*RN*pow(ratio_k0,2)) + param_T11*pow(q,4)
               -param_T12*pow(q,6);
    double delta_lon;   //(弧度)
    delta_lon = q/(RN*cos_lan*ratio_k0)-(1+2*tan2_lan+e_sec2*cos2_lan)*pow(q,3)/(6*pow(RN,3)*cos_lan*pow(ratio_k0,3))
                + param_T16*pow(q,5);
    lon_rad = lon0_rad - delta_lon;
    LLH[0] = lan_rad*180/M_PI;
    LLH[1] = lon_rad*180/M_PI;
    LLH[2] = UTM_ENU[2];
}

// **************
// 功能:将车辆定位天线的经纬高转为车辆中心港区坐标,将车辆定位天线速度转换为车体中心速度
// 输入:FusionDataInfo pose_in(pose_llh) 定位天线坐标LLH纬经高(deg)
// 输出:FusionDataInfo pose_in(pose) 车辆中心点港区坐标(m),中心点高度与定位天线高度相同
// 无返回
// ***************
void transFusionLLHtoHarbourENU(location_msgs::FusionDataInfo &pose_in)
{

    double LLH[3];
    double UTM_ENU[3];
    LLH[0] = pose_in.pose_llh.x;
    LLH[1] = pose_in.pose_llh.y;
    LLH[2] = pose_in.pose_llh.z;
    UTM_ENU[2] = pose_in.pose_llh.z;
    transForLLHtoUTM(LLH,UTM_ENU);
    //IMU港区坐标
    Vector3d delta_enu;
    delta_enu(0) = UTM_ENU[0] - Har_e0;
    delta_enu(1) = UTM_ENU[1] - Har_n0;
    delta_enu(2) = UTM_ENU[2] - Har_u0;
    //printf("delta_enu=%f %f %f\n",delta_enu(0),delta_enu(1),delta_enu(2));

    //车辆中心点港区坐标
    Vector2d delta_center_Anti; //车辆定位天线与中心点偏差
    delta_center_Anti = Vector2d::Zero();
    // delta_center_IMU(0,0) = -0.027;  //车体坐标系下x轴向偏差,IMU
    // delta_center_IMU(1,0) = -6.5227; //车体坐标系下y轴偏差,IMU
    // delta_center_Anti(0,0) = 0.0;  //车体坐标系下x轴向偏差,定位天线
    // delta_center_Anti(1,0) = -7.4; //车体坐标系下y轴偏差,定位天线
    delta_center_Anti(0,0) = -1.308;  //车体坐标系下x轴向偏差,定位天线
    delta_center_Anti(1,0) = -7.582; //车体坐标系下y轴偏差,定位天线
    Matrix2d trans_Anti_center;
    trans_Anti_center = Matrix2d::Zero(2, 2);
    double yaw = pose_in.yaw * M_PIl / 180;
    trans_Anti_center << cos(yaw), sin(yaw),
                        -sin(yaw),cos(yaw);
    Vector2d delta_xyz; //港区坐标系下车辆定位天线与中心点的偏差
    delta_xyz = Vector2d::Zero();
    delta_xyz = trans_Anti_center * delta_center_Anti;
    Vector3d delta_tp;
    delta_tp = Vector3d::Zero();
    delta_tp(0,0) = delta_xyz(0);
    delta_tp(1,0) = delta_xyz(1);
    delta_tp(2,0) = 0.0;
    Vector3d vel_center_enu; //车辆中心点港区坐标
    vel_center_enu(0) = delta_enu(0) + delta_tp(0);
    vel_center_enu(1) = delta_enu(1) + delta_tp(1);
    vel_center_enu(2) = delta_enu(2) + delta_tp(2);

    pose_in.pose.x = vel_center_enu(0);
    pose_in.pose.y = vel_center_enu(1);
    pose_in.pose.z = vel_center_enu(2);
    //printf("pose=%f %f %f\n",pose_in.pose.x,pose_in.pose.y,pose_in.pose.z);

    //计算定位天线速度转换为车辆中心速度(杆臂引起的速度不同)
    double angular_tp = sqrt(pow(pose_in.velocity.angular.x,2) + pow(pose_in.velocity.angular.y,2) + pow(pose_in.velocity.angular.z,2));
    if (angular_tp <= 1)
    {
        //计算车辆中心速度
        pose_in.vel_ctr.x = pose_in.velocity.linear.x;
        pose_in.vel_ctr.y = pose_in.velocity.linear.y;
        pose_in.vel_ctr.z = pose_in.velocity.linear.z;
    }
    else
    {
        //计算wn_in
        double LLH1[3];
        for(int i = 0; i < 2; i++)
        {
            LLH1[i] = LLH[i]*M_PIl/180;
        }
        double R_NM[2];
        R_NM[0] = Ra*(1 - 2*ff + 3*ff*pow(sin(LLH1[0]),2));
        R_NM[1] = Ra*(1 + ff*pow(sin(LLH1[0]),2));
        Vector3d wn_ie;     //导航坐标系下跟着地球转的角速度
        wn_ie(0,0) = 0.0;
        wn_ie(1,0) = w_ie*cos(LLH1[0]);
        wn_ie(2,0) = w_ie*sin(LLH1[0]);

        Vector3d wn_en;        //由于运载体运动而引起的相对地球的旋转角速度
        wn_en(0,0) = -pose_in.velocity.linear.y/R_NM[1];
        wn_en(1,0) = pose_in.velocity.linear.x/R_NM[0];
        wn_en(2,0) = pose_in.velocity.linear.x/R_NM[0]*tan(LLH1[0]);

        Vector3d wn_in;
        wn_in = wn_ie + wn_en; 

        Vector3d wb_ib; //IMU输出的角速度
        wb_ib(0) = pose_in.velocity.angular.x*M_PIl/180;
        wb_ib(1) = pose_in.velocity.angular.y*M_PIl/180;
        wb_ib(2) = pose_in.velocity.angular.z*M_PIl/180;
        //求Cnb
        double atitude[3];
        atitude[0] = pose_in.yaw;
        atitude[1] = pose_in.pitch;
        atitude[2] = pose_in.roll;
        Matrix3d C_nb;
        transForAngletoCnb(atitude,C_nb);

        Vector3d wb_bn;
        wb_bn = wb_ib - C_nb*wn_in;
        Vector3d delta_l_b;   //车体坐标系下定位天线与车辆中心位置矢量(以定位天线为中心,单位m)
        delta_l_b(0) = 1.308;
        delta_l_b(1) = 7.582;
        delta_l_b(2) = 0;

        Matrix3d wb_bn_X;   //wn_bn的叉乘(反向量)
        wb_bn_X << 0, -wb_bn(2), wb_bn(1),
               wb_bn(2), 0, -wb_bn(0),
               -wb_bn(1), wb_bn(0), 0;
        //导航坐标系下杆臂引起的误差
        Vector3d vel_delta;  
        vel_delta = C_nb.transpose()*wb_bn_X*delta_l_b;  
        //计算车辆中心速度
        pose_in.vel_ctr.x = pose_in.velocity.linear.x - vel_delta(0);
        pose_in.vel_ctr.y = pose_in.velocity.linear.y - vel_delta(1);
        pose_in.vel_ctr.z = pose_in.velocity.linear.z - vel_delta(2);
    }
}

//**************
//功能:计算Cnb
//输入:三轴欧拉角(姿态角),顺序为yaw,pitch,roll,单位degree(角度)
//输出:Cnb矩阵
//无返回
//*************
void transForAngletoCnb(double (&atitude)[3],Matrix3d &C_nb)
{
    double yaw = atitude[0]*M_PIl/180;
    double pitch = atitude[1]*M_PIl/180;
    double roll = atitude[2]*M_PIl/180;
    C_nb = Matrix3d::Zero();

    C_nb(0,0) = cos(roll)*cos(yaw)+sin(roll)*sin(yaw)*sin(pitch);
    C_nb(0,1) = -cos(roll)*sin(yaw)+sin(roll)*cos(yaw)*sin(pitch);
    C_nb(0,2) = -sin(roll)*cos(pitch);

    C_nb(1,0) = sin(yaw)*cos(pitch);
    C_nb(1,1) = cos(yaw)*cos(pitch);
    C_nb(1,2) = sin(pitch);

    C_nb(2,0) = sin(roll)*cos(yaw)-cos(roll)*sin(yaw)*sin(pitch);
    C_nb(2,1) = -sin(roll)*sin(yaw)-cos(roll)*cos(yaw)*sin(pitch);
    C_nb(2,2) = cos(roll)*cos(pitch);

}

// **************
// 功能:欧拉角转四元数(n系至b系的四元数)
// 输入:Atitude[3] 三轴欧拉角(姿态角),顺序为yaw,pitch,roll,单位degree(角度)
// 输出:quaternions[4]四元数
// 无返回
// ***************
void transForAngletoQuaternions(float (&atitude)[3], double (&quaternions)[4])
{
    double atitude2[3];
    atitude2[0] = atitude[0] * M_PI / 180 * 0.5; //单位弧度
    atitude2[1] = atitude[1] * M_PI / 180 * 0.5;
    atitude2[2] = atitude[2] * M_PI / 180 * 0.5;

    double sp, sr, sy, cp, cr, cy;
    sp = sin(atitude2[1]);
    sr = sin(atitude2[2]);
    sy = sin(atitude2[0]);
    cp = cos(atitude2[1]);
    cr = cos(atitude2[2]);
    cy = cos(atitude2[0]);
    sp = sin(atitude2[1]);
    sr = sin(atitude2[2]);
    sy = sin(atitude2[0]);
    cp = cos(atitude2[1]);
    cr = cos(atitude2[2]);
    cy = cos(atitude2[0]);


    quaternions[0] = cy * cp * cr  + sy * sp * sr;
    quaternions[1] = cy * sp * cr  + sy * cp * sr;
    quaternions[2] = cy * cp * sr  - sy * sp * cr;
    quaternions[3] = cy * sp * sr  - sy * cp * cr;
}

// **************
// 功能:b系至n系的坐标变换矩阵
// 输入:quaternions[4] 四元数
// 输出:C_bn 坐标变换矩阵
// 无返回
// ***************
void transForCoordBodytoNavigation(double quaternions[4], Matrix3d &C_bn)
{
    //C_bn.resize(3,3);
    C_bn = Matrix3d::Zero();

    C_bn(0, 0) = pow(quaternions[0], 2) + pow(quaternions[1], 2) - pow(quaternions[2], 2) - pow(quaternions[3], 2);
    C_bn(0, 1) = 2 * (quaternions[1] * quaternions[2] - quaternions[0] * quaternions[3]);
    C_bn(0, 2) = 2 * (quaternions[1] * quaternions[3] + quaternions[0] * quaternions[2]);

    C_bn(1, 0) = 2 * (quaternions[1] * quaternions[2] + quaternions[0] * quaternions[3]);
    C_bn(1, 1) = pow(quaternions[0], 2) - pow(quaternions[1], 2) + pow(quaternions[2], 2) - pow(quaternions[3], 2);
    C_bn(1, 2) = 2 * (quaternions[2] * quaternions[3] - quaternions[0] * quaternions[1]);

    C_bn(2, 0) = 2 * (quaternions[1] * quaternions[3] - quaternions[0] * quaternions[2]);
    C_bn(2, 1) = 2 * (quaternions[2] * quaternions[3] + quaternions[0] * quaternions[1]);
    C_bn(2, 2) = pow(quaternions[0], 2) - pow(quaternions[1], 2) - pow(quaternions[2], 2) + pow(quaternions[3], 2);

    // C_bn(0, 0) = 1 - 2*(pow(quaternions[2], 2) + pow(quaternions[3], 2));
    // C_bn(0, 1) = 2 * (quaternions[1] * quaternions[2] - quaternions[0] * quaternions[3]);
    // C_bn(0, 2) = 2 * (quaternions[1] * quaternions[3] + quaternions[0] * quaternions[2]);

    // C_bn(1, 0) = 2 * (quaternions[1] * quaternions[2] + quaternions[0] * quaternions[3]);
    // C_bn(1, 1) = 1 - 2*(pow(quaternions[1], 2) + pow(quaternions[3], 2));
    // C_bn(1, 2) = 2 * (quaternions[2] * quaternions[3] - quaternions[0] * quaternions[1]);

    // C_bn(2, 0) = 2 * (quaternions[1] * quaternions[3] - quaternions[0] * quaternions[2]);
    // C_bn(2, 1) = 2 * (quaternions[2] * quaternions[3] + quaternions[0] * quaternions[1]);
    // C_bn(2, 2) = 1 - 2*(pow(quaternions[1], 2) + pow(quaternions[2], 2));
}

// **************
// 功能:根据坐标变换矩阵得到姿态角
// 输入:C_bn 姿态变换阵
// 输出:att姿态角,顺序yaw,pitch,roll
// 无返回
// ***************
void transForCbntoAtitude(Matrix3d &C_bn, float (&att)[3])
{

    //yaw
    att[0] = atan2(C_bn(0, 1), C_bn(1, 1)) * 180 / M_PI;
    if (att[0]<1e-8)
    {
        att[0] = att[0] + 360;
    }
    //pitch
    att[1] = asin(C_bn(2, 1)) * 180 / M_PI;
    //roll
    att[2] = atan2(-C_bn(2, 0), C_bn(2, 2)) * 180 / M_PI;
}

// **************
// 功能:WGS84的经纬高(m)与经纬高(deg)的转换
// 输入:lan_lon_m[2] 纬度,经度上相差的米数(m),LLH[3]所在位置的纬经高(deg)
// 输出:lan_lon_deg[2] 将纬度,经度上相差的米数转换为度数(deg)
// 无返回
// ***************
void transForMetertoDegree(double (&lan_lon_m)[2],double (&LLH)[3],double (&lan_lon_deg)[2])
{
    double LLH1[3];
    for(int i = 0; i < 2; i++)
    {
        LLH1[i] = LLH[i]*M_PI/180;
    }
    double RMh; //子午圈曲率半径(加高程了)
    double RNh; //卯酉圈曲率半径(加高程了)
    double clRNh;
    RNh = Ra*(1 - 2*ff + 3*ff*pow(sin(LLH1[0]),2)) + LLH[2];
    RMh = Ra*(1 + ff*pow(sin(LLH1[0]),2)) + LLH[2];
    clRNh = RNh*cos(LLH1[0]);   //平行圈半径
    lan_lon_deg[0] = lan_lon_m[0]/RMh*180/M_PI; //纬度变化度数
    lan_lon_deg[1] = lan_lon_m[1]/clRNh*180/M_PI;   //经度变化度数

}

// **************
// 功能:WGS84的经纬高(deg)与经纬高(m)的转换
// 输入:lan_lon_deg[2] 纬度,经度上相差的度数(deg),LLH[3]所在位置的纬经高(deg)
// 输出:lan_lon_m[2] 将纬度,经度上相差的度数转换为米数(deg)
// 无返回
// ***************
void transForDegreetoMeter(double (&lan_lon_deg)[2],double (&LLH)[3],double (&lan_lon_m)[2])
{
    double LLH1[3];
    for(int i = 0; i < 2; i++)
    {
        LLH1[i] = LLH[i]*M_PI/180;
    }
    double RMh; //子午圈曲率半径(加高程了)
    double RNh; //卯酉圈曲率半径(加高程了)
    double clRNh;
    RNh = Ra*(1 - 2*ff + 3*ff*pow(sin(LLH1[0]),2)) + LLH[2];
    RMh = Ra*(1 + ff*pow(sin(LLH1[0]),2)) + LLH[2];
    clRNh = RNh*cos(LLH1[0]);
    lan_lon_m[0] = lan_lon_deg[0]*M_PI/180*RMh;     //纬度变化的米数
    lan_lon_m[1] = lan_lon_deg[1]*M_PI/180*clRNh;   //经度变化的米数
    //ROS_INFO("lan=%f,lon=%f",lan_lon_m[0],lan_lon_m[1]);
}

// **************
// 功能:将车辆中心的slam坐标转换为车辆定位天线的纬经高
// 输入:Har_ENU0(m)开始进行slam定位时的起始点港区坐标(东北天),slam坐标值(距离起始点的三轴位置m,东北天),车辆当前的yaw角度(deg),lon0当前位置的中央子午线(deg)
// 输出:LLH[3]定位天线的纬经高(deg)
// 无返回
// ***************
void transForSLAMtoLLH(double (&Har_ENU0)[3],double (&slam_enu)[3],double &yaw,double &lon0,double (&LLH)[3])
{
    double Har_agv_enu[3];  //车辆中心点港区坐标
    Har_agv_enu[0] = Har_ENU0[0] + slam_enu[0];
    Har_agv_enu[1] = Har_ENU0[1] + slam_enu[1];
    Har_agv_enu[2] = Har_ENU0[2] + slam_enu[2];

    //车辆中心点港区坐标
    Vector2d delta_center_Anti; //车辆定位天线与中心点偏差
    delta_center_Anti = Vector2d::Zero();
    delta_center_Anti(0,0) = 1.308;  //车体坐标系下x轴向偏差,定位天线
    delta_center_Anti(1,0) = 7.582; //车体坐标系下y轴偏差,定位天线
    Matrix2d trans_Anti_center;
    trans_Anti_center = Matrix2d::Zero(2, 2);
    double yaw_rad = yaw * M_PIl / 180;
    trans_Anti_center << cos(yaw_rad), sin(yaw_rad),
                        -sin(yaw_rad),cos(yaw_rad);
    Vector2d delta_xyz; //港区坐标系下车辆定位天线与中心点的偏差
    delta_xyz = Vector2d::Zero();
    delta_xyz = trans_Anti_center * delta_center_Anti;
    Vector3d delta_tp;
    delta_tp = Vector3d::Zero();
    delta_tp(0,0) = delta_xyz(0);
    delta_tp(1,0) = delta_xyz(1);
    delta_tp(2,0) = 0.0;

    Vector3d Har_Anti_enu;  //港区坐标系下定位天线坐标
    Har_Anti_enu(0) = Har_agv_enu[0] + delta_tp(0);
    Har_Anti_enu(1) = Har_agv_enu[1] + delta_tp(1);
    Har_Anti_enu(2) = Har_agv_enu[2] + delta_tp(2);

    double UTM_Anti_enu[3];
    UTM_Anti_enu[0] = Har_e0 + Har_Anti_enu(0);
    UTM_Anti_enu[1] = Har_n0 + Har_Anti_enu(1);
    UTM_Anti_enu[2] = Har_Anti_enu(2);
    //定位天线的经纬高
    double Anti_LLH[3];
    transForUTMtoLLH(UTM_Anti_enu,Anti_LLH,lon0);
    LLH[0] = Anti_LLH[0];
    LLH[1] = Anti_LLH[1];
    LLH[2] = Anti_LLH[2];
}