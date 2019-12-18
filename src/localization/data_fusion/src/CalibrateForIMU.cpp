#include "CalibrateForIMU.h"
#include "FusionCenter.h"
#include "CoordinateSystem.h"
#include <vector>
#include <eigen3/Eigen/Sparse>
#include "GlobalVari.h"


CalibrateForIMU::CalibrateForIMU(FusionCenter *pFsCenter)
{
    g_pfscenter = pFsCenter;
    memset(&g_last_tm,0,sizeof(UTC));
    initForQ();
    g_calib_I.resize(9,9);
    g_calib_I = MatrixXd::Zero(9,9);
}

CalibrateForIMU::~CalibrateForIMU(void)
{

}

void CalibrateForIMU::initForQ(void)
{
    int count = 9;
    double Q[count];
    //double Q[9] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.0, 0.0, 0.0};
    if (g_pfscenter->low_freq_type == g_pfscenter->uwb)
    {
        if (g_pfscenter->zupt_type == g_pfscenter->zupt_stop || g_pfscenter->zupt_type == g_pfscenter->zupt_linear)
        {
            Q[0] = g_cfg_zuptuwb.q0.Fai_yaw;
            Q[1] = g_cfg_zuptuwb.q0.Fai_pitch;
            Q[2] = g_cfg_zuptuwb.q0.Fai_roll;
            Q[3] = g_cfg_zuptuwb.q0.delta_ve;
            Q[4] = g_cfg_zuptuwb.q0.delta_vn;
            Q[5] = g_cfg_zuptuwb.q0.delta_vu;
            Q[6] = g_cfg_zuptuwb.q0.delta_lan;
            Q[7] = g_cfg_zuptuwb.q0.delta_lon;
            Q[8] = g_cfg_zuptuwb.q0.delta_h;
        }
        else
        {
            Q[0] = g_cfg_uwb.q0.Fai_yaw;
            Q[1] = g_cfg_uwb.q0.Fai_pitch;
            Q[2] = g_cfg_uwb.q0.Fai_roll;
            Q[3] = g_cfg_uwb.q0.delta_ve;
            Q[4] = g_cfg_uwb.q0.delta_vn;
            Q[5] = g_cfg_uwb.q0.delta_vu;
            Q[6] = g_cfg_uwb.q0.delta_lan;
            Q[7] = g_cfg_uwb.q0.delta_lon;
            Q[8] = g_cfg_uwb.q0.delta_h;
        }
    }
    else if (g_pfscenter->low_freq_type == g_pfscenter->lidar)
    {
        if (g_pfscenter->zupt_type == g_pfscenter->zupt_stop || g_pfscenter->zupt_type == g_pfscenter->zupt_linear)
        {
            Q[0] = g_cfg_zuptlidar.q0.Fai_yaw;
            Q[1] = g_cfg_zuptlidar.q0.Fai_pitch;
            Q[2] = g_cfg_zuptlidar.q0.Fai_roll;
            Q[3] = g_cfg_zuptlidar.q0.delta_ve;
            Q[4] = g_cfg_zuptlidar.q0.delta_vn;
            Q[5] = g_cfg_zuptlidar.q0.delta_vu;
            Q[6] = g_cfg_zuptlidar.q0.delta_lan;
            Q[7] = g_cfg_zuptlidar.q0.delta_lon;
            Q[8] = g_cfg_zuptlidar.q0.delta_h;
        }
        else
        {
            Q[0] = g_cfg_lidar.q0.Fai_yaw;
            Q[1] = g_cfg_lidar.q0.Fai_pitch;
            Q[2] = g_cfg_lidar.q0.Fai_roll;
            Q[3] = g_cfg_lidar.q0.delta_ve;
            Q[4] = g_cfg_lidar.q0.delta_vn;
            Q[5] = g_cfg_lidar.q0.delta_vu;
            Q[6] = g_cfg_lidar.q0.delta_lan;
            Q[7] = g_cfg_lidar.q0.delta_lon;
            Q[8] = g_cfg_lidar.q0.delta_h;
        }
    }
    g_fusion_Q0.resize(count,count);
    g_fusion_Q0 = MatrixXd::Zero(count,count);
    for(int i = 0; i < count; i++)
    {
        for(int j = 0; j < count; j++)
        {
            if (i==j)
            {
                g_fusion_Q0(i,j) = Q[i]*Q[j];
            }
            else
            {
                g_fusion_Q0(i,j) = 0.0;
            }
        }
    }
}

void CalibrateForIMU::initForNavigationParam(double (&LLH)[3], double (&VENU)[3], Vector3d &wn_in, double (&R_NM)[2])
{
    double LLH1[3];
    for(int i = 0; i < 2; i++)
    {
        LLH1[i] = LLH[i]*M_PI/180;
    }
    R_NM[0] = Ra*(1 - 2*ff + 3*ff*pow(sin(LLH1[0]),2));
    R_NM[1] = Ra*(1 + ff*pow(sin(LLH1[0]),2));
    Vector3d wn_ie;     //导航坐标系下跟着地球转的角速度
    wn_ie(0,0) = 0.0;
    wn_ie(1,0) = w_ie*cos(LLH1[0]);
    wn_ie(2,0) = w_ie*sin(LLH1[0]);

    Vector3d wn_en;        //由于运载体运动而引起的相对地球的旋转角速度
    wn_en(0,0) = -VENU[1]/R_NM[1];
    wn_en(1,0) = VENU[0]/R_NM[0];
    wn_en(2,0) = VENU[0]/R_NM[0]*tan(LLH1[0]);

    wn_in = wn_ie + wn_en; 
}

void CalibrateForIMU::calculateFtSysParam(PoseResult &h_fm, MatrixXd &fusion_F, MatrixXd &fusion_G)
{
    double LLH[3],VENU[3],R_NM[2];
    Vector3d wn_in;
    wn_in = Vector3d::Zero(3,1);
    LLH[0] = h_fm.pos.lan;
    LLH[1] = h_fm.pos.lon;
    LLH[2] = h_fm.pos.h;
    VENU[0] = h_fm.vel.venu.vx;
    VENU[1] = h_fm.vel.venu.vy;
    VENU[2] = h_fm.vel.venu.vz;
    initForNavigationParam(LLH, VENU, wn_in, R_NM);
    int sys_count = 9;
    //fusion_F:
    fusion_F.resize(sys_count,sys_count);
    fusion_F.setZero(sys_count,sys_count);
    fusion_F(0,1) = w_ie*sin(h_fm.pos.lan*M_PI/180) 
                        + 
                        h_fm.vel.venu.vx/(R_NM[0]
                        +
                        h_fm.pos.h)*tan(h_fm.pos.lan*M_PI/180);
    fusion_F(0,2) = -w_ie*cos(h_fm.pos.lan*M_PI/180) 
                        - 
                        h_fm.vel.venu.vx/(R_NM[0] + h_fm.pos.h);
    fusion_F(0,4) = -1/(R_NM[1] + h_fm.pos.h);
    fusion_F(0,8) = h_fm.vel.venu.vy/pow(R_NM[1] + h_fm.pos.h,2);

    fusion_F(1,0) = -fusion_F(0,1);
    fusion_F(1,2) = -h_fm.vel.venu.vy/(R_NM[1] + h_fm.pos.h);
    fusion_F(1,3) = 1/(R_NM[0] + h_fm.pos.h);
    fusion_F(1,6) = -w_ie*sin(h_fm.pos.lan*M_PI/180);
    fusion_F(1,8) = -h_fm.vel.venu.vx/pow((R_NM[0] + h_fm.pos.h),2);

    fusion_F(2,0) = -fusion_F(0,2);
    fusion_F(2,1) = -fusion_F(1,2);
    fusion_F(2,3) = tan(h_fm.pos.lan*M_PI/180)/(R_NM[0] + h_fm.pos.h);
    fusion_F(2,6) = w_ie*cos(h_fm.pos.lan*M_PI/180) 
                        + 
                        h_fm.vel.venu.vx/(R_NM[0] + h_fm.pos.h)/pow(cos(h_fm.pos.lan*M_PI/180),2);
    fusion_F(2,8) = -h_fm.vel.venu.vx*tan(h_fm.pos.lan*M_PI/180)/pow(R_NM[0] + h_fm.pos.h,2);

    float atitude[3];
    atitude[0] = h_fm.att.yaw;
    atitude[1] = h_fm.att.pitch;
    atitude[2] = h_fm.att.roll;
    double quaternions[4];
    Matrix3d C_bn;
    transForAngletoQuaternions(atitude,quaternions);
    transForCoordBodytoNavigation(quaternions,C_bn);

    fusion_F(3,1) = -C_bn(2,0)*h_fm.accel.ax - C_bn(2,1)*h_fm.accel.ay - C_bn(2,2)*h_fm.accel.az;   
    fusion_F(3,2) = C_bn(1,0)*h_fm.accel.ax + C_bn(1,1)*h_fm.accel.ay + C_bn(1,2)*h_fm.accel.az;
    fusion_F(3,3) = (h_fm.vel.venu.vy*tan(h_fm.pos.lan*M_PI/180)
                        -
                        h_fm.vel.venu.vz)/(R_NM[0] + h_fm.pos.h);
    fusion_F(3,4) = 2*w_ie*sin(h_fm.pos.lan*M_PI/180)
                        +
                        h_fm.vel.venu.vx/(R_NM[0] + h_fm.pos.h)*tan(h_fm.pos.lan*M_PI/180);
    fusion_F(3,5) = -(2*w_ie*cos(h_fm.pos.lan*M_PI/180) + h_fm.vel.venu.vx/(R_NM[0] + h_fm.pos.h));
    fusion_F(3,6) = 2*w_ie*(h_fm.vel.venu.vz*sin(h_fm.pos.lan*M_PI/180) 
                        +
                        h_fm.vel.venu.vy*cos(h_fm.pos.lan*M_PI/180))
                        + 
                        h_fm.vel.venu.vx*h_fm.vel.venu.vy/(R_NM[0] + h_fm.pos.h)
                        /
                        pow(cos(h_fm.pos.lan*M_PI/180),2); 
    fusion_F(3,8) = (h_fm.vel.venu.vx*h_fm.vel.venu.vz 
                        - 
                        h_fm.vel.venu.vx*h_fm.vel.venu.vy*tan(h_fm.pos.lan*M_PI/180))
                        /
                        pow(R_NM[0] + h_fm.pos.h,2);
    
    fusion_F(4,0) = -fusion_F(3,1);
    fusion_F(4,2) = -(C_bn(0,0)*h_fm.accel.ax + C_bn(0,1)*h_fm.accel.ay + C_bn(0,2)*h_fm.accel.az);
    fusion_F(4,3) = -2*w_ie*sin(h_fm.pos.lan*M_PI/180) 
                        - 
                        2*h_fm.vel.venu.vx/(R_NM[0] + h_fm.pos.h)
                        * 
                        tan(h_fm.pos.lan*M_PI/180);
    fusion_F(4,4) = -h_fm.vel.venu.vz/(R_NM[1] + h_fm.pos.h);
    fusion_F(4,5) = -h_fm.vel.venu.vy/(R_NM[1] + h_fm.pos.h);
    fusion_F(4,6) = -2*h_fm.vel.venu.vx*w_ie*cos(h_fm.pos.lan) 
                        - 
                        pow(h_fm.vel.venu.vx,2)/(R_NM[0] + h_fm.pos.h)
                        /
                        pow(cos(h_fm.pos.lan*M_PI/180),2);
    fusion_F(4,8) = h_fm.vel.venu.vy*h_fm.vel.venu.vz
                        /
                        pow(R_NM[1]+h_fm.pos.h,2) 
                        + 
                        pow(h_fm.vel.venu.vx,2)*tan(h_fm.pos.lan*M_PI/180)
                        /
                        pow(R_NM[0] + h_fm.pos.h,2);

    fusion_F(5,0) = -fusion_F(3,2);
    fusion_F(5,1) = -fusion_F(4,2);
    fusion_F(5,3) = 2*(w_ie*cos(h_fm.pos.lan*M_PI/180) 
                        + 
                        h_fm.vel.venu.vx
                        /
                        (R_NM[0] + h_fm.pos.h));
    fusion_F(5,4) = 2*h_fm.vel.venu.vy/(R_NM[1] + h_fm.pos.h);
    fusion_F(5,6) = -2*h_fm.vel.venu.vx*w_ie*sin(h_fm.pos.lan*M_PI/180);
    fusion_F(5,8) = -pow(h_fm.vel.venu.vy,2)/pow(R_NM[1] + h_fm.pos.h,2) 
                        - 
                        pow(h_fm.vel.venu.vx,2)/pow(R_NM[0] + h_fm.pos.h,2);

    fusion_F(6,4) = 1/(R_NM[1] + h_fm.pos.h);
    fusion_F(6,8) = -h_fm.vel.venu.vy/pow(R_NM[1] + h_fm.pos.h,2);

    fusion_F(7,3) = 1/cos(h_fm.pos.lan*M_PI/180)/(R_NM[0] + h_fm.pos.h);
    fusion_F(7,6) = h_fm.vel.venu.vx*tan(h_fm.pos.lan*M_PI/180)/cos(h_fm.pos.lan*M_PI/180)/(R_NM[0] + h_fm.pos.h);
    fusion_F(7,8) = -h_fm.vel.venu.vx/cos(h_fm.pos.lan*M_PI/180)/pow(R_NM[0] + h_fm.pos.h,2);
    
    fusion_F(8,5) = 1;

    //fusion_G:
    fusion_G.resize(sys_count,sys_count);
    fusion_G.setZero(sys_count,sys_count);      //系统噪声矩阵
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            fusion_G(i,j) = -C_bn(i,j);
        } 
    }
    for (int i = 3; i < 6; i++)
    {
        for (int j = 3; j < 6; j++)
        {
            fusion_G(i,j) = C_bn(i-3,j-3);
        } 
    }
}

// **************
// 功能:一步转移矩阵离散阵的计算
// 输入:fusion_F时间更新的传递矩阵 时间配准时刻time
// 输出:离散后的一步转移矩阵g_fusion_Fai
// 无返回
// ***************
void CalibrateForIMU::discreteForFusionFG(MatrixXd &fusion_F,double &delta_T, MatrixXd &fusion_Fai,
                                        MatrixXd &fusion_G,MatrixXd &fusion_Q0,MatrixXd &fusion_Q)
{
    MatrixXd Mat_I;
    MatrixXd Mat_tp1;
    MatrixXd Mat_tp2;
    int sys_count = 9;
    
    //一步转移矩阵
    Mat_I.resize(sys_count,sys_count);
    Mat_I.setIdentity(sys_count,sys_count); 
    Mat_tp1.resize(sys_count,sys_count);
    Mat_tp1 = delta_T*fusion_F;
    Mat_tp2.resize(sys_count,sys_count);
    Mat_tp2 = pow(delta_T,2)/2*fusion_F*fusion_F;
    fusion_Fai.resize(sys_count,sys_count);
    fusion_Fai.Zero(sys_count,sys_count);
    fusion_Fai = Mat_I + Mat_tp1 + Mat_tp2;

    //系统噪声离散化
    MatrixXd M_tp1;
    M_tp1.resize(sys_count,sys_count);
    M_tp1.setZero(sys_count,sys_count);
    MatrixXd M_tp2;
    M_tp2.resize(sys_count,sys_count);
    M_tp2.setZero(sys_count,sys_count);
    M_tp1 = fusion_G*fusion_Q0*fusion_G.transpose();
    MatrixXd temp;
    temp.resize(sys_count,sys_count);
    temp.setZero(sys_count,sys_count);
    temp = fusion_F*M_tp1;
    M_tp2 = temp + temp.transpose();
    fusion_Q = M_tp1*delta_T + M_tp2*pow(delta_T,2)/2;
}

void CalibrateForIMU::calculatePoseConfidence(MatrixXd &P_2k, float &pose_confidence)
{
    int count_lan = 6,count_lon = 7;
    Vector2d tp_p1;
    tp_p1(0,0) = P_2k(count_lan,count_lan);
    tp_p1(1,0) = P_2k(count_lon,count_lon);
    double meas = 1.0;
    meas = tp_p1.transpose()*tp_p1;

    MatrixXd P_fk;
    P_fk.resize(9,9);
    P_fk.setZero(9,9);
    //用于稀疏矩阵求逆
    typedef SparseMatrix<double,ColMajor,int> SpMatType;
    typedef Triplet<double> Tpt; 
    //求稀疏矩阵g_calib_I的逆矩阵
    int nonzero_num = g_calib_I.nonZeros();   //非零元素个数
    int nonzero_r[nonzero_num];
    int nonzero_c[nonzero_num];
    int count_size = 0;
    int rows = g_calib_I.rows(),cols = g_calib_I.cols();
    for (int j = 0; j < cols; j++)
    {
        for (int i = 0; i < rows; i++)
        {
            if (g_calib_I(i,j) != 1e-8)
            {
                nonzero_r[count_size] = i;
                nonzero_c[count_size] = j;
                count_size++;
            }   
        } 
    }
    vector<Tpt> tripletlist;
    tripletlist.reserve(nonzero_num);   //为25个非零元素分配空间
    for (int i = 0; i < nonzero_num; i++)
    {
        double val = g_calib_I(nonzero_r[i],nonzero_c[i]);
        tripletlist.push_back(Tpt(nonzero_r[i],nonzero_c[i],val));
    }
    SpMatType spA(rows,cols);
    spA.setFromTriplets(tripletlist.begin(),tripletlist.end());
    spA.makeCompressed(); //压缩优化矩阵
    vector<Tpt>().swap(tripletlist);
    SparseQR<SpMatType,COLAMDOrdering<int> > solver;
    solver.compute(spA);
    SpMatType Sp_I(rows,cols);
    Sp_I.setIdentity();
    SpMatType g_calib_I_inv(rows,cols);
    g_calib_I_inv = solver.solve(Sp_I);
    P_fk = g_calib_I_inv;
    Vector2d tp_p2;
    tp_p2(0,0) = P_fk(count_lan,count_lan);
    tp_p2(1,0) = P_fk(count_lon,count_lon);
    double theory = 1.0;
    theory = tp_p2.transpose()*tp_p2;
    if (meas <= theory)
        {
            pose_confidence = meas/theory;
        }
    else
    {
        pose_confidence = theory/meas;
    } 
}

bool CalibrateForIMU::calculateFilter(PoseResult &high_freq_match, UTC &time,location_msgs::FusionDataInfo &g_fusion_data_calib)
{
    MatrixXd fusion_F;      //时间更新的传递矩阵
    MatrixXd fusion_G;
    int count_f=9;
    calculateFtSysParam(high_freq_match, fusion_F, fusion_G);
    double delta_T;
    delta_T = time.hour*3600 + time.min*60 + time.sec + time.msec/1000
              - (g_last_tm.hour*3600 + g_last_tm.min*60 + g_last_tm.sec + g_last_tm.msec/1000);
    if (g_pfscenter->low_freq_type == g_pfscenter->dr)
    {
        if (delta_T > 1) 
        {
            delta_T = 0.0333;
        }   
    }
    else
    {
        if (delta_T > 1) 
        {
            delta_T = 0.1;
        } 
    }
    
    
    MatrixXd fusion_Fai;      //离散化后的时间更新的传递矩阵
    MatrixXd fusion_Q;     //离散化后的系统误差方差阵
    discreteForFusionFG(fusion_F,delta_T,fusion_Fai,fusion_G,g_fusion_Q0,fusion_Q);

    VectorXd X_2k;       //一步预测状态参量(从k-1至k)
    X_2k.resize(count_f,1);
    X_2k.setZero(count_f,1);
    X_2k = fusion_Fai*g_pfscenter->g_delta_IMUcalib;

    //用于稀疏矩阵求逆
    typedef SparseMatrix<double,ColMajor,int> SpMatType;
    typedef Triplet<double> Tpt;
    //求fusion_Q的逆矩阵
    int Q_nonzero_num = fusion_Q.nonZeros();
    int Q_nonzero_r[Q_nonzero_num];
    int Q_nonzero_c[Q_nonzero_num];
    int Qcount_size = 0;
    int Qrows = fusion_Q.rows(),Qcols = fusion_Q.cols();
    for (int j = 0; j < Qcols; j++)
    {
        for (int i = 0; i < Qrows; i++)
        {
            if (fusion_Q(i,j) != 1e-8)
            {
                Q_nonzero_r[Qcount_size] = i;
                Q_nonzero_c[Qcount_size] = j;
                Qcount_size++;
            }   
        } 
    }
    vector<Tpt> Q_tripletlist;
    Q_tripletlist.reserve(Q_nonzero_num);   
    for (int i = 0; i < Q_nonzero_num; i++)
    {
        double val = fusion_Q(Q_nonzero_r[i],Q_nonzero_c[i]);
        Q_tripletlist.push_back(Tpt(Q_nonzero_r[i],Q_nonzero_c[i],val));
    }
    SpMatType Q_spA(Qrows,Qcols);
    Q_spA.setFromTriplets(Q_tripletlist.begin(),Q_tripletlist.end());
    Q_spA.makeCompressed(); //压缩优化矩阵
    vector<Tpt>().swap(Q_tripletlist);
    SparseQR<SpMatType,COLAMDOrdering<int> > Q_solver;
    Q_solver.compute(Q_spA);
    // Q_solver.analyzePattern(Q_spA);
    // Q_solver.factorize(Q_spA);
    SpMatType Q_Sp_I(Qrows,Qcols);
    Q_Sp_I.setIdentity();
    MatrixXd Q_inv(Qrows,Qcols);
    Q_inv = Q_solver.solve(Q_Sp_I);

    MatrixXd lI;
    lI.resize(count_f,count_f);
    lI.setZero(count_f,count_f);
    lI = g_pfscenter->g_IMUcalib_I;
    MatrixXd I_2k;      //信息矩阵(从k-1至k)
    I_2k.resize(count_f,count_f);
    I_2k.setZero(count_f,count_f);
    MatrixXd temp_1;
    temp_1.resize(count_f,count_f);
    temp_1 = MatrixXd::Zero(count_f,count_f);
    temp_1 = fusion_Fai.transpose()*Q_inv*fusion_Fai;
    MatrixXd temp_2;
    temp_2.resize(count_f,count_f);
    temp_2 = MatrixXd::Zero(count_f,count_f);
    temp_2 = lI + temp_1;
    MatrixXd temp_3;
    temp_3.resize(count_f,count_f);
    temp_3 = MatrixXd::Zero(count_f,count_f);
    temp_3 = Q_inv*fusion_Fai*temp_2.inverse();
    MatrixXd temp_4;
    temp_4.resize(count_f,count_f);
    temp_4 = MatrixXd::Zero(count_f,count_f);
    temp_4 = temp_3*fusion_Fai.transpose()*Q_inv;
    I_2k = Q_inv - temp_4;

    //求稀疏矩阵I_2k的逆矩阵
    int nonzero_num = I_2k.nonZeros();   //非零元素个数
    int nonzero_r[nonzero_num];
    int nonzero_c[nonzero_num];
    int count_size = 0;
    int rows = I_2k.rows(),cols = I_2k.cols();
    for (int j = 0; j < cols; j++)
    {
        for (int i = 0; i < rows; i++)
        {
            if (I_2k(i,j) != 1e-8)
            {
                nonzero_r[count_size] = i;
                nonzero_c[count_size] = j;
                count_size++;
            }   
        } 
    }
    vector<Tpt> tripletlist;
    tripletlist.reserve(nonzero_num);   //为25个非零元素分配空间
    for (int i = 0; i < nonzero_num; i++)
    {
        double val = I_2k(nonzero_r[i],nonzero_c[i]);
        tripletlist.push_back(Tpt(nonzero_r[i],nonzero_c[i],val));
    }
    SpMatType spA(rows,cols);
    spA.setFromTriplets(tripletlist.begin(),tripletlist.end());
    spA.makeCompressed(); //压缩优化矩阵
    vector<Tpt>().swap(tripletlist);
    SparseQR<SpMatType,COLAMDOrdering<int> > solver;
    solver.compute(spA);
    // solver.analyzePattern(spA);
    // solver.factorize(spA);
    SpMatType Sp_I(rows,cols);
    Sp_I.setIdentity();
    SpMatType I_2k_inv(rows,cols);
    I_2k_inv = solver.solve(Sp_I);
    
    MatrixXd P_2k;
    P_2k.resize(count_f,count_f);
    P_2k.setZero(count_f,count_f);
    P_2k = I_2k_inv;

    //计算置信度
    float pose_confidence = 0.0;
    calculatePoseConfidence(P_2k,pose_confidence);
    g_fusion_data_calib.pose_confidence = pose_confidence;

    //融合结果
    //计算姿态角
    // double delta_Q[4];      //姿态误差角引起的误差四元素
    // delta_Q[0] = 1;
    // delta_Q[1] = X_2k(0)/2;
    // delta_Q[2] = X_2k(1)/2;
    // delta_Q[3] = X_2k(2)/2;
    // float atitude[3];
    // atitude[0] = high_freq_match.att.yaw;
    // atitude[1] = high_freq_match.att.pitch;
    // atitude[2] = high_freq_match.att.roll;
    // double qua[4];
    // transForAngletoQuaternions(atitude, qua);
    // double Q_qua[4];          //补偿后的姿态四元数
    // Q_qua[0] = delta_Q[0]*qua[0] - delta_Q[1]*qua[1] - delta_Q[2]*qua[2] - delta_Q[3]*qua[3];
    // Q_qua[1] = delta_Q[0]*qua[1] + delta_Q[1]*qua[0] + delta_Q[2]*qua[3] - delta_Q[3]*qua[2];
    // Q_qua[2] = delta_Q[0]*qua[2] + delta_Q[2]*qua[0] + delta_Q[3]*qua[1] - delta_Q[1]*qua[3];
    // Q_qua[3] = delta_Q[0]*qua[3] + delta_Q[3]*qua[0] + delta_Q[1]*qua[2] - delta_Q[2]*qua[1];
    // Matrix3d C_bn;
    // transForCoordBodytoNavigation(Q_qua, C_bn);
    // float att[3]={0.0};       //补偿后的姿态角
    // transForCbntoAtitude(C_bn, att);
    // g_fusion_data.yaw = att[0];
    // g_fusion_data.pitch = att[1];
    // g_fusion_data.roll = att[2];
    float atitude[3];
    atitude[0] = high_freq_match.att.yaw;
    atitude[1] = high_freq_match.att.pitch;
    atitude[2] = high_freq_match.att.roll;
    double qua[4];
    transForAngletoQuaternions(atitude, qua);
    Matrix3d C_bn0;
    transForCoordBodytoNavigation(qua, C_bn0); 
    Matrix3d degree_fai;
    degree_fai << 0, -X_2k(2), X_2k(1),
                 X_2k(2), 0 , -X_2k(0),
                 -X_2k(1), X_2k(0), 0;
    Matrix3d degtp;
    Matrix3d degI;
    degI.setIdentity();
    degtp = degI - degree_fai;
    Matrix3d C_bn1;
    C_bn1 = degtp.inverse()*C_bn0;
    float att[3];       //补偿后的姿态角
    transForCbntoAtitude(C_bn1, att); 
    g_fusion_data_calib.yaw = att[0];
    g_fusion_data_calib.pitch = att[1];
    g_fusion_data_calib.roll = att[2];

    //计算补偿后的速度
    g_fusion_data_calib.velocity.linear.x = high_freq_match.vel.venu.vx + X_2k(3);
    g_fusion_data_calib.velocity.linear.y = high_freq_match.vel.venu.vy + X_2k(4);
    g_fusion_data_calib.velocity.linear.z = high_freq_match.vel.venu.vz + X_2k(5);

    //计算补偿后的位置
    double lan_lon_m[2] = {X_2k(6),X_2k(7)};
    double lan_lon_deg[2];
    double LLH_new[3] = {high_freq_match.pos.lan,high_freq_match.pos.lon,high_freq_match.pos.h};
    transForMetertoDegree(lan_lon_m,LLH_new,lan_lon_deg);
    g_fusion_data_calib.pose_llh.x = high_freq_match.pos.lan + lan_lon_deg[0];
    g_fusion_data_calib.pose_llh.y = high_freq_match.pos.lon + lan_lon_deg[1];
    g_fusion_data_calib.pose_llh.z = high_freq_match.pos.h + X_2k(8);

    //车体坐标系下加速度
    g_fusion_data_calib.accel.linear.x = high_freq_match.accel.ax;
    g_fusion_data_calib.accel.linear.y = high_freq_match.accel.ay;
    g_fusion_data_calib.accel.linear.z = high_freq_match.accel.az;

    //车体坐标系下角速度
    g_fusion_data_calib.velocity.angular.x = high_freq_match.vel.wxyz.wx;
    g_fusion_data_calib.velocity.angular.y = high_freq_match.vel.wxyz.wy;
    g_fusion_data_calib.velocity.angular.z = high_freq_match.vel.wxyz.wz;

    //传感器融合状态
    g_fusion_data_calib.fuse_state = 2;

    //存储结果用于下次滤波
    g_pfscenter->g_delta_IMUcalib = X_2k;
    g_pfscenter->g_IMUcalib_I = I_2k;

    return true;
}

