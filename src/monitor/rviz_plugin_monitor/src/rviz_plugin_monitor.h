

#ifndef RVIZ_PLUGIN_MONITOR_H
#define RVIZ_PLUGIN_MONITOR_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/panel.h>
#endif

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>

class QLineEdit;

namespace rviz_plugin_monitor {

class MonitorPanel : public rviz::Panel {
  Q_OBJECT

public:
  MonitorPanel(QWidget *parent = 0);

  virtual void load(const rviz::Config &config);
  virtual void save(rviz::Config config) const;

private:
  void rvizClickedPointCallback(const geometry_msgs::PointStampedConstPtr &msg);
  void agvGpsAndVelocityCallback(const nav_msgs::OdometryConstPtr &msg);
  void agvAxleAngleCallback(const sensor_msgs::JointStateConstPtr &msg);

  void refLineLaneCallback(const geometry_msgs::PointStampedConstPtr &msg);

protected:
  // RViz clicked point sub
  ros::Subscriber rviz_clicked_point_sub_;
  // AGV gps speed sub
  ros::Subscriber agv_gps_speed_sub_;
  // AGV axle angle sub
  ros::Subscriber agv_axle_angle_sub_;

  // refline info sub
  ros::Subscriber refline_info_sub_;

  // The ROS node handle.
  ros::NodeHandle nh_;

  // AGV x y info
  QLineEdit *agv_x_edit_;
  QLineEdit *agv_y_edit_;

  // AGV speed info
  QLineEdit *eastward_velocity_edit_;
  QLineEdit *northward_velocity_edit_;

  // AGV axle angle info
  QLineEdit *front_axle_angle_edit_;
  QLineEdit *rear_axle_angle_edit_;

  // rviz clicked point
  QLineEdit *rviz_clecked_point_x_edit_;
  QLineEdit *rviz_clecked_point_y_edit_;

  // ref line point info
  QLineEdit *lane_id_edit_;
  QLineEdit *refline_point_x_edit_;
  QLineEdit *refline_point_y_edit_;
};
}

#endif
