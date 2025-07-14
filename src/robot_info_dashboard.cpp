#include "robot_gui/robot_info_dashboard.h"
#include "std_srvs/Trigger.h"
#include <vector>

CVUIROSDashInfo::CVUIROSDashInfo() {
  // Initialize ROS node
  ros::NodeHandle nh;

  info_sub_ = nh.subscribe<robotinfo_msgs::RobotInfo10Fields>(
      ui_topic_name, 2, &CVUIROSDashInfo::msgCallback, this);

  odom_sub_ = nh.subscribe<nav_msgs::Odometry>(
      odom_topic_name, 2, &CVUIROSDashInfo::odom_msgCallback, this);

  twist_pub_ = nh.advertise<geometry_msgs::Twist>(twist_topic_name, 1);

  // Service clients - need to be initiated form cmd line
  // `rosrun distance_tracker_service distance_tracker_service`
  get_distance_client_ = nh.serviceClient<std_srvs::Trigger>("/get_distance");
}

bool CVUIROSDashInfo::call_get_distance_service() {
  std_srvs::Trigger srv;

  if (get_distance_client_.call(srv)) {
    if (srv.response.success) {
      distance_display = srv.response.message;
      ROS_INFO("Distance retrieved: %s meters", srv.response.message.c_str());
      return true;
    } else {
      distance_display = "Service Error";
      ROS_WARN("Get distance service returned failure: %s",
               srv.response.message.c_str());
      return false;
    }
  } else {
    distance_display = "Call Failed";
    ROS_ERROR("Failed to call /get_distance service");
    return false;
  }
}

bool CVUIROSDashInfo::call_reset_distance_service() {
  distance_display = "0.00";
  ROS_INFO("Distance display reset to 0.00 (GUI simulation)");
  return true;
}

void CVUIROSDashInfo::msgCallback(
    const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg) {
  robot_data = *msg;
  ROS_DEBUG("Robot info received: %s", msg->data_field_01.c_str());
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
    int y_offset = 8;
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
    int buttons_y_offset = 210;

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
    int odo_y_offset = 367;

    cvui::text(frame, odo_x_offset, odo_y_offset,
               "Estimated robot position based off odometery");

    odo_y_offset += 15;
    cvui::window(frame, odo_x_offset, odo_y_offset, 127, 76, "X");
    cvui::printf(frame, odo_x_offset + 31, odo_y_offset + 35, 1.2, 0xffff00,
                 "%.1f", odom_data.pose.pose.position.x);

    odo_x_offset += 132;
    cvui::window(frame, odo_x_offset, odo_y_offset, 127, 76, "Y");
    cvui::printf(frame, odo_x_offset + 31, odo_y_offset + 35, 1.2, 0xffff00,
                 "%.1f", odom_data.pose.pose.position.y);

    odo_x_offset += 132;
    cvui::window(frame, odo_x_offset, odo_y_offset, 127, 76, "Z");
    cvui::printf(frame, odo_x_offset + 31, odo_y_offset + 35, 1.2, 0xffff00,
                 "%.1f", odom_data.pose.pose.position.z);

    //        ================ Distance Service ================

    int dist_x_offset = 70;
    int dist_y_offset = 509;

    cvui::text(frame, dist_x_offset, dist_y_offset - 20,
               "Distance Service Status:");

    if (cvui::button(frame, dist_x_offset, dist_y_offset + 5, 120, 35,
                     "Get Distance")) {
      call_get_distance_service();
    }

    if (cvui::button(frame, dist_x_offset + 130, dist_y_offset + 5, 120, 35,
                     "Reset Distance")) {
      call_reset_distance_service();
    }

    // Display current distance
    cvui::window(frame, dist_x_offset, dist_y_offset + 70, 220, 60,
                 "Total Distance Traveled");
    cvui::printf(frame, dist_x_offset + 70, dist_y_offset + 105, 0.7, 0x00ff00,
                 "%s m", distance_display.c_str());

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
