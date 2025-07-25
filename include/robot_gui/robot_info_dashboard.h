#pragma once

#define CVUI_IMPLEMENTATION
#include "nav_msgs/Odometry.h"
#include "robot_gui/cvui.h"
#include "robotinfo_msgs/RobotInfo10Fields.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "std_srvs/Trigger.h"
#include <geometry_msgs/Twist.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sstream>

class CVUIROSDashInfo {
public:
  CVUIROSDashInfo();

  void run();

private:
  // GUI
  std::string ui_topic_name = "robot_info";

  // Subscriber
  // robot_info subscriber
  ros::Subscriber info_sub_;
  robotinfo_msgs::RobotInfo10Fields robot_data;
  void msgCallback(const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg);
  // odom subscriber
  ros::Subscriber odom_sub_;
  nav_msgs::Odometry odom_data;
  std::string odom_topic_name = "odom";
  void odom_msgCallback(const nav_msgs::Odometry::ConstPtr &msg);
  // TODO : do this with template - subs

  // Publisher - cmd_vel
  double linear_velocity_step = 0.1;
  double angular_velocity_step = 0.2;
  double max_linear_velocity = 1.0;  // 1.0 m/s max
  double max_angular_velocity = 2.0; // 2.0 rad/s max

  std::string twist_topic_name = "cmd_vel";
  ros::Publisher twist_pub_;
  geometry_msgs::Twist twist_msg;
  void reset_twistmsg();

  // Distance tracker service
  ros::ServiceClient get_distance_client_;
  ros::ServiceClient reset_distance_client_;
  std::string distance_display = "0.00";
  bool call_get_distance_service();
  bool call_reset_distance_service();

  const std::string WINDOW_NAME = "ROBOT INFO DASHBOARD";
};