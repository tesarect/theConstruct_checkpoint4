#pragma once

#define CVUI_IMPLEMENTATION
#include "robot_gui/cvui.h"
#include "robotinfo_msgs/RobotInfo10Fields.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

class CVUIROSSubscriber {
public:
  CVUIROSSubscriber();

  void run();

private:
  ros::Subscriber sub_;
  robotinfo_msgs::RobotInfo10Fields robot_data;
  std::string topic_name;
  //   void msgCallback(const std_msgs::Float64::ConstPtr &msg);
  void msgCallback(const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg);
  const std::string WINDOW_NAME = "ROBOT INFO DASHBOARD";
};