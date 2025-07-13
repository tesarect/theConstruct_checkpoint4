#pragma once

#define CVUI_IMPLEMENTATION
#include "robot_gui/cvui.h"
#include "robotinfo_msgs/RobotInfo10Fields.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

class CVUIROSDashInfo {
public:
  CVUIROSDashInfo();

  void run();

private:
  // GUI
  std::string ui_topic_name = "robot_info";

  // Subscriber - robot_info
  ros::Subscriber sub_;
  robotinfo_msgs::RobotInfo10Fields robot_data;
  void msgCallback(const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg);

  // Publisher - cmd_vel
  double linear_velocity_step = 0.1;
  double angular_velocity_step = 0.2;
  double max_linear_velocity = 1.0;   // 1.0 m/s max
  double max_angular_velocity = 2.0;  // 2.0 rad/s max

  std::string twist_topic_name = "cmd_vel";
  ros::Publisher twist_pub_;
  geometry_msgs::Twist twist_msg;
  void reset_twistmsg();

  const std::string WINDOW_NAME = "ROBOT INFO DASHBOARD";
};