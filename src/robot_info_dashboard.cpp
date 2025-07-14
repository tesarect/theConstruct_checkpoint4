#include "robot_gui/robot_info_dashboard.h"
#include <vector>

CVUIROSDashInfo::CVUIROSDashInfo() {
  // Initialize ROS node
  ros::NodeHandle nh;

  info_sub_ = nh.subscribe<robotinfo_msgs::RobotInfo10Fields>(
      ui_topic_name, 2, &CVUIROSDashInfo::msgCallback, this);

  odom_sub_ = nh.subscribe<nav_msgs::Odometry>(
      odom_topic_name, 2, &CVUIROSDashInfo::odom_msgCallback, this);

  twist_pub_ = nh.advertise<geometry_msgs::Twist>(twist_topic_name, 1);
}

void CVUIROSDashInfo::msgCallback(
    const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg) {
  robot_data = *msg;
  ROS_DEBUG("Robot info received");
}

void CVUIROSDashInfo::odom_msgCallback(
    const nav_msgs::Odometry::ConstPtr &msg) {
  odom_data = *msg;
  ROS_DEBUG("Position x,y,z: [%0.2f, %0.2f, %0.2f]", msg->pose.pose.position.x,
            msg->pose.pose.position.y, msg->pose.pose.position.z);
}

void CVUIROSDashInfo::reset_twistmsg() {
  twist_msg.linear.x = 0.0;
  twist_msg.linear.y = 0.0;
  twist_msg.linear.z = 0.0;
  twist_msg.angular.x = 0.0;
  twist_msg.angular.y = 0.0;
  twist_msg.angular.z = 0.0;
}

void CVUIROSDashInfo::run() {
  // create a frame
  cv::Mat frame = cv::Mat(1000, 400, CV_8UC3);

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

    int buttons_x_offset = 150;
    int buttons_y_offset = 220;

    // Generic lambda for veloity adjustment
    auto adjust_velocity = [&](double *velocity, double step, double max_vel) {
      *velocity = std::max(-max_vel, std::min(*velocity + step, max_vel));
    };

    // Reset the velocity values
    auto stop_robot = [&]() {
      reset_twistmsg();
      twist_pub_.publish(twist_msg);
    };

    // Forward button
    if (cvui::button(frame, buttons_x_offset, buttons_y_offset, " Forward ")) {
      adjust_velocity(&twist_msg.linear.x, linear_velocity_step,
                      max_linear_velocity);
    }

    // Stop button
    buttons_y_offset += 30;
    if (cvui::button(frame, buttons_x_offset, buttons_y_offset, "   Stop  ")) {
      stop_robot();
    }

    // Left button
    if (cvui::button(frame, buttons_x_offset - 70, buttons_y_offset,
                     " Left ")) {
      adjust_velocity(&twist_msg.angular.z, angular_velocity_step,
                      max_angular_velocity);
    }

    // Right button
    if (cvui::button(frame, buttons_x_offset + 95, buttons_y_offset,
                     " Right ")) {
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

    // current twist msg on main loop - to keep moving until its is `STOPPED`
    twist_pub_.publish(twist_msg);

    //        ================ Current Velocity ================

    // Velocity window
    int vel_x_offset = 5;
    int vel_y_offset = buttons_y_offset + 40;

    cvui::window(frame, vel_x_offset, vel_y_offset, 190, 40,
                 "Linear velocity:");
    // Show the current velocity inside the window
    cvui::printf(frame, vel_x_offset + 100, vel_y_offset + 25, 0.4, 0xff0000,
                 "%.02f m/sec", twist_msg.linear.x);

    vel_x_offset += 200;

    cvui::window(frame, vel_x_offset, vel_y_offset, 190, 40,
                 "Angular velocity:");
    // Show the current velocity inside the window
    cvui::printf(frame, vel_x_offset + 100, vel_y_offset + 25, 0.4, 0xff0000,
                 "%.02f rad/sec", twist_msg.angular.z);

    //        ================ Robot Position ================

    int odo_x_offset = 5;
    int odo_y_offset = 375;

    cvui::text(frame, odo_x_offset, odo_y_offset,
               "Estimated robot position based off odometery");

    odo_y_offset += 15;
    cvui::window(frame, odo_x_offset, odo_y_offset, 127, 80, "X");
    cvui::printf(frame, odo_x_offset + 76, odo_y_offset + 35, 1.2, 0xffff00,
                 "%.0f", odom_data.pose.pose.position.x);

    odo_x_offset += 132;
    cvui::window(frame, odo_x_offset, odo_y_offset, 127, 80, "Y");
    cvui::printf(frame, odo_x_offset + 76, odo_y_offset + 35, 1.2, 0xffff00,
                 "%.0f", odom_data.pose.pose.position.y);

    odo_x_offset += 132;
    cvui::window(frame, odo_x_offset, odo_y_offset, 127, 80, "Z");
    cvui::printf(frame, odo_x_offset + 76, odo_y_offset + 35, 1.2, 0xffff00,
                 "%.0f", odom_data.pose.pose.position.z);

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
