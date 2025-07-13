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
  std::string ui_topic_name = "robot_info";

  // Subscriber - robot_info
  ros::Subscriber sub_;
  robotinfo_msgs::RobotInfo10Fields robot_data;
  // void msgCallback(const std_msgs::Float64::ConstPtr &msg);
  void msgCallback(const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg);

  // Publisher - cmd_vel
  ros::Publisher twist_pub_;
  geometry_msgs::Twist twist_msg;
  std::string twist_topic_name;
  float linear_velocity_step = 0.1;
  float angular_velocity_step = 0.1;

  const std::string WINDOW_NAME = "ROBOT INFO DASHBOARD";
};