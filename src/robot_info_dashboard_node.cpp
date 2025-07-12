#include "robot_gui/robot_info_dashboard.h"
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "robot_info_dashboard");
  CVUIROSSubscriber robot_info_dashboard;
  robot_info_dashboard.run();
  return 0;
}