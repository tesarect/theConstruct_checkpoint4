/*
The CVUIROSDashInfo class creates a graphical user interface featuring a
window and text. When a ROS message is received, the gui displays the incomming
data inside the window.

To use this class, include the show_received_messages.h header file in your
main program file and create an instance of the CVUIROSDashInfo class. You
would then call the 'run' function on the instance to start the program.

Author: Roberto Zegers
Date: February 2023
License: BSD-3-Clause
*/

#include "robot_gui/robot_info_dashboard.h"
#include <vector>

CVUIROSDashInfo::CVUIROSDashInfo() {
  // Initialize ROS node
  ros::NodeHandle nh;

  sub_ = nh.subscribe<robotinfo_msgs::RobotInfo10Fields>(
      ui_topic_name, 2, &CVUIROSDashInfo::msgCallback, this);

  twist_pub_ = nh.advertise<geometry_msgs::Twist>(twist_topic_name, 1);
}

void CVUIROSDashInfo::msgCallback(
    const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg) {
  robot_data = *msg;
  ROS_DEBUG("Robot info received");
}

void CVUIROSDashInfo::reset_twistmsg() {
  twist_msg.linear.x = 0.0;
  twist_msg.linear.y = 0.0;
  twist_msg.linear.z = 0.0;
  twist_msg.angular.x = 0.0;
  twist_msg.angular.y = 0.0;
  twist_msg.angular.z = 0.0;
}

//   auto inc_linear_vel = [&]() {
//     twist_msg.linear.x = std::min(twist_msg.linear.x + linear_velocity_step,
//     max_linear_velocity); twist_pub_.publish(twist_msg);
//   };

//   auto dec_linear_vel = [&]() {
//     twist_msg.linear.x = std::max(twist_msg.linear.x - linear_velocity_step,
//     -max_linear_velocity); twist_pub_.publish(twist_msg);
//   };

//   auto inc_angular_vel = [&]() {
//     twist_msg.angular.z = std::min(twist_msg.angular.z +
//     angular_velocity_step, max_angular_velocity);
//     twist_pub_.publish(twist_msg);
//   };

//   auto dec_angular_vel = [&]() {
//     twist_msg.angular.z = std::max(twist_msg.angular.z -
//     angular_velocity_step, -max_angular_velocity);
//     twist_pub_.publish(twist_msg);
//   };

void CVUIROSDashInfo::run() {
  // create a frame
  cv::Mat frame = cv::Mat(600, 400, CV_8UC3);

  // Init a OpenCV window and tell cvui to use it.
  cv::namedWindow(WINDOW_NAME);
  cvui::init(WINDOW_NAME);

  while (ros::ok()) {
    // Fill the frame with a nice color
    frame = cv::Scalar(49, 52, 49);

    //        ================ General Info Area ================

    int x_offset = 50;
    int y_offset = 20;
    // Create window
    cvui::window(frame, x_offset, y_offset, 300, 190,
                 "Topic: " + ui_topic_name);

    std::vector<std::string> data_fields = {
        robot_data.data_field_01, robot_data.data_field_02,
        robot_data.data_field_03, robot_data.data_field_04,
        robot_data.data_field_05, robot_data.data_field_06,
        robot_data.data_field_07, robot_data.data_field_08};

    // Show non-empty fields
    int text_x_offset = x_offset + 5;
    int line_height = 20;
    for (const auto &field : data_fields) {
      if (!field.empty()) {
        cvui::printf(frame, text_x_offset, y_offset + 27, 0.4, 0xffffff, "%s",
                     field.c_str());
        y_offset += line_height;
      }
    }

    //        ================ Teleoperation Buttons ================

    int buttons_x_offset = 100;
    int buttons_y_offset = y_offset + 20;

    // Generic lambda for veloity adjustment
    auto adjust_velocity = [&](double *velocity, double step, double max_vel) {
      *velocity = std::max(-max_vel, std::min(*velocity + step, max_vel));
      twist_pub_.publish(twist_msg);
    };

    // Reset the velocity values
    auto stop_robot = [&]() {
      reset_twistmsg();
      twist_pub_.publish(twist_msg);
    };

    // Forward button
    if (cvui::button(frame, buttons_x_offset, buttons_y_offset, " Forward ")) {
      // The button was clicked, update the Twist message
      adjust_velocity(&twist_msg.linear.x, linear_velocity_step,
                      max_linear_velocity);
    }

    // Stop button
    buttons_y_offset += 30;
    if (cvui::button(frame, buttons_x_offset, buttons_y_offset, "   Stop  ")) {
      // The button was clicked, update the Twist message
      stop_robot();
    }

    // Left button
    if (cvui::button(frame, buttons_x_offset - 70, buttons_y_offset,
                     " Left ")) {
      // The button was clicked, update the Twist message
      adjust_velocity(&twist_msg.angular.z, angular_velocity_step,
                      max_angular_velocity);
    }

    // Right button
    if (cvui::button(frame, buttons_x_offset + 95, buttons_y_offset,
                     " Right ")) {
      // The button was clicked, update the Twist message
      adjust_velocity(&twist_msg.angular.z, -angular_velocity_step,
                      max_angular_velocity);
    }

    // Backward button
    buttons_y_offset += 30;
    if (cvui::button(frame, buttons_x_offset, buttons_y_offset, "Backward")) {
      // The button was clicked,update the Twist message
      adjust_velocity(&twist_msg.linear.x, -linear_velocity_step,
                      max_linear_velocity);
    }

    // Velocity window
    int vel_x_offset = 20;
    int vel_y_offset = buttons_y_offset + 30;
    // Create window at (320, 20) with size 120x40 (width x height) and title
    // cvui::window(frame, 320, 20, 120, 40, "Linear velocity:");
    cvui::window(frame, vel_x_offset, vel_y_offset, 120, 40,
                 "Linear velocity:");
    // Show the current velocity inside the window
    cvui::printf(frame, vel_x_offset + 25, vel_y_offset + 5, 0.4, 0xff0000,
                 "%.02f m/sec", twist_msg.linear.x);

    vel_x_offset += 110;
    // Create window at (320 60) with size 120x40 (width x height) and title
    // cvui::window(frame, 320, 60, 120, 40, "Angular velocity:");
    cvui::window(frame, vel_x_offset, vel_y_offset, 120, 40,
                 "Angular velocity:");
    // Show the current velocity inside the window
    cvui::printf(frame, vel_x_offset + 25, vel_y_offset + 5, 0.4, 0xff0000,
                 "%.02f rad/sec", twist_msg.angular.z);

    //        ================ Current Velocity ================

    // Update cvui internal stuff
    cvui::update();

    // Show everything on the screen
    cv::imshow(WINDOW_NAME, frame);

    // Check if ESC key was pressed
    if (cv::waitKey(20) == 27) {
      break;
    }
    // Spin as a single-threaded node
    ros::spinOnce();
  }
}
