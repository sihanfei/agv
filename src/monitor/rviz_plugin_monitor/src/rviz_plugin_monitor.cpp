

#include <stdio.h>

#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QTimer>
#include <QVBoxLayout>

#include "rviz_plugin_monitor.h"
#include <functional>

namespace rviz_plugin_monitor {

MonitorPanel::MonitorPanel(QWidget *parent) : rviz::Panel(parent) {
  // AGV 位置 信息显示布局
  QHBoxLayout *agv_position_layout = new QHBoxLayout;
  agv_position_layout->addWidget(new QLabel("AGV_position-X:"));
  agv_x_edit_ = new QLineEdit;
  agv_position_layout->addWidget(agv_x_edit_);
  agv_position_layout->addWidget(new QLabel("AGV_position-Y:"));
  agv_y_edit_ = new QLineEdit;
  agv_position_layout->addWidget(agv_y_edit_);

  // AGV SPEED 信息布局
  QHBoxLayout *agv_velocity_layout = new QHBoxLayout;
  agv_velocity_layout->addWidget(new QLabel("E_velocity:"));
  eastward_velocity_edit_ = new QLineEdit;
  agv_velocity_layout->addWidget(eastward_velocity_edit_);
  agv_velocity_layout->addWidget(new QLabel("N_velocity"));
  northward_velocity_edit_ = new QLineEdit;
  agv_velocity_layout->addWidget(northward_velocity_edit_);

  // AGV 前后轴角度 信息布局
  QHBoxLayout *agv_axle_angle_layout = new QHBoxLayout;
  agv_axle_angle_layout->addWidget(new QLabel("FrontAxle_angle:"));
  front_axle_angle_edit_ = new QLineEdit;
  agv_axle_angle_layout->addWidget(front_axle_angle_edit_);
  agv_axle_angle_layout->addWidget(new QLabel("RearAxle_angle:"));
  rear_axle_angle_edit_ = new QLineEdit;
  agv_axle_angle_layout->addWidget(rear_axle_angle_edit_);

  // RViz 选点 信息显示布局
  QHBoxLayout *rviz_clicked_point_layout = new QHBoxLayout;
  rviz_clicked_point_layout->addWidget(new QLabel("ClickedPoint-X:"));
  rviz_clecked_point_x_edit_ = new QLineEdit;
  rviz_clicked_point_layout->addWidget(rviz_clecked_point_x_edit_);
  rviz_clicked_point_layout->addWidget(new QLabel("ClickedPoint-Y:"));
  rviz_clecked_point_y_edit_ = new QLineEdit;
  rviz_clicked_point_layout->addWidget(rviz_clecked_point_y_edit_);

  // 全局规划 参考点信息
  QHBoxLayout *refline_point_layout = new QHBoxLayout;
  refline_point_layout->addWidget(new QLabel("RefLine-laneID:"));
  lane_id_edit_ = new QLineEdit;
  refline_point_layout->addWidget(lane_id_edit_);

  refline_point_layout->addWidget(new QLabel("RefLine-X:"));
  refline_point_x_edit_ = new QLineEdit;
  refline_point_layout->addWidget(refline_point_x_edit_);
  refline_point_layout->addWidget(new QLabel("RefLine--Y:"));
  refline_point_y_edit_ = new QLineEdit;
  refline_point_layout->addWidget(refline_point_y_edit_);

  QVBoxLayout *layout = new QVBoxLayout;
  // layout->addWidget(new QLabel("GPS"));
  layout->addLayout(agv_position_layout);
  layout->addLayout(agv_velocity_layout);
  layout->addLayout(agv_axle_angle_layout);
  layout->addLayout(rviz_clicked_point_layout);
  layout->addLayout(refline_point_layout);
  setLayout(layout);

  rviz_clicked_point_sub_ = nh_.subscribe("/clicked_point", 10, &MonitorPanel::rvizClickedPointCallback, this);

  agv_gps_speed_sub_ = nh_.subscribe("/control/odom_msg", 10, &MonitorPanel::agvGpsAndVelocityCallback, this);

  agv_axle_angle_sub_ = nh_.subscribe("/control/joint_states", 10, &MonitorPanel::agvAxleAngleCallback, this);

  refline_info_sub_ = nh_.subscribe("/monitor/rviz_click_lane", 10, &MonitorPanel::refLineLaneCallback, this);
}

void MonitorPanel::save(rviz::Config config) const { rviz::Panel::save(config); }

void MonitorPanel::rvizClickedPointCallback(const geometry_msgs::PointStampedConstPtr &msg) {
  rviz_clecked_point_x_edit_->setText(QString::number(msg->point.x));
  rviz_clecked_point_y_edit_->setText(QString::number(msg->point.y));
}
void MonitorPanel::agvGpsAndVelocityCallback(const nav_msgs::OdometryConstPtr &msg) {
  double gps_lat = msg->pose.pose.position.x;     //纬度 84-Y
  double gps_lon = msg->pose.pose.position.y;     //经度 84-X
  double gps_speed_x = msg->twist.twist.linear.x; //速度.换算为m/s
  double gps_speed_y = msg->twist.twist.linear.y; //速度.换算为m/s
                                                  // GPS info
  agv_x_edit_->setText(QString::number(gps_lon));
  agv_y_edit_->setText(QString::number(gps_lat));

  // AGV speed info
  eastward_velocity_edit_->setText(QString::number(gps_speed_x));
  northward_velocity_edit_->setText(QString::number(gps_speed_y));
}
void MonitorPanel::agvAxleAngleCallback(const sensor_msgs::JointStateConstPtr &msg) {

  front_axle_angle_edit_->setText(QString::number(msg->position[0]));

  rear_axle_angle_edit_->setText(QString::number(msg->position[1]));
}
void MonitorPanel::refLineLaneCallback(const geometry_msgs::PointStampedConstPtr &msg) {
  lane_id_edit_->setText(QString::number(msg->point.z));
  refline_point_x_edit_->setText(QString::number(msg->point.x));
  refline_point_y_edit_->setText(QString::number(msg->point.y));
}

void MonitorPanel::load(const rviz::Config &config) { rviz::Panel::load(config); }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugin_monitor::MonitorPanel, rviz::Panel)
