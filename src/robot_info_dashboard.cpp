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

  //   start_distance_service();
  distance_client_ = nh.serviceClient<std_srvs::Trigger>("/get_distance");
  check_distance_service_availability();
}

void CVUIROSDashInfo::check_distance_service_availability() {
  if (ros::service::exists("/get_distance", false)) {
    ROS_INFO("Distance service is available");
    distance_service_response = "Service Available - Click to get distance";
    service_available_ = true;
  } else {
    ROS_INFO("Distance service not available");
    distance_service_response = "Service NOT Available - Click to start";
    service_available_ = false;
  }
}

bool CVUIROSDashInfo::start_distance_service() {
  // Check if service exists
  //   if (ros::service::exists("/get_distance", false)) {
  //     ROS_INFO("Distance service already running");
  //     distance_service_response = "Distance service ready";
  //     return true;
  //   }

  //   ROS_INFO("Distance service not found, starting it...");
  //   distance_service_response = "Starting distance service...";

  //   // Start the service using system call
  //   std::string command =
  //       "rosrun distance_tracker_service distance_tracker_service &";
  //   int result = system(command.c_str());

  //   if (result != 0) {
  //     ROS_ERROR("Failed to start distance service");
  //     distance_service_response = "ERROR: Could not start distance service";
  //     return false;
  //   }

  //   // Wait for service to become available
  //   ROS_INFO("Waiting for distance service to start...");
  //   ros::Time start_time = ros::Time::now();
  //   ros::Duration timeout(10.0); // 10 second timeout

  //   while (ros::Time::now() - start_time < timeout) {
  //     if (ros::service::exists("/get_distance", false)) {
  //       ROS_INFO("Distance service started successfully!");
  //       distance_service_response = "Distance service ready";
  //       return true;
  //     }
  //     ros::Duration(0.5).sleep(); // Check every 500ms
  //     ros::spinOnce();
  //   }

  //   ROS_WARN("Timeout waiting for distance service to start");
  //   distance_service_response = "WARNING: Distance service startup timeout";
  //   return false;
  if (ros::service::exists("/get_distance", false)) {
    ROS_INFO("Distance service already running");
    distance_service_response = "Distance service ready";
    return true;
  }

  ROS_INFO("Distance service not found, starting it...");
  distance_service_response = "Starting distance service...";

  // Start the service using system call
  std::string command =
      "rosrun distance_tracker_service distance_tracker_service &";
  int result = system(command.c_str());

  if (result != 0) {
    ROS_ERROR("Failed to start distance service");
    distance_service_response = "ERROR: Could not start distance service";
    return false;
  }

  // Use ROS's built-in service waiting method
  ROS_INFO("Waiting for distance service to start...");
  if (distance_client_.waitForExistence(ros::Duration(15.0))) {
    ROS_INFO("Distance service started successfully!");
    distance_service_response = "Distance service ready";
    return true;
  } else {
    ROS_WARN("Timeout waiting for distance service to start");
    distance_service_response = "WARNING: Distance service startup timeout";
    return false;
  }
}

bool CVUIROSDashInfo::call_distance_service() {
  // If service not available, try to start it
  if (!service_available_ || !ros::service::exists("/get_distance", false)) {
    ROS_INFO("Starting distance service...");
    distance_service_response = "Starting distance service...";

    std::string command =
        "rosrun distance_tracker_service distance_tracker_service &";
    int result = system(command.c_str());

    if (result != 0) {
      distance_service_response = "ERROR: Failed to start service";
      service_available_ = false;
      return false;
    }

    // Wait for service with shorter timeout to not block GUI
    if (distance_client_.waitForExistence(ros::Duration(5.0))) {
      distance_service_response = "Service started successfully!";
      service_available_ = true;
      ros::Duration(0.5).sleep(); // Brief pause for initialization
    } else {
      distance_service_response = "ERROR: Service startup timeout";
      service_available_ = false;
      return false;
    }
  }
  // Call the service
  std_srvs::Trigger srv;

  if (distance_client_.call(srv)) {
    if (srv.response.success) {
      ROS_INFO("Distance: %s meters", srv.response.message.c_str());
      distance_service_response = "Distance: " + srv.response.message + " m";
      service_available_ = true;
      return true;
    } else {
      distance_service_response = "Service error: " + srv.response.message;
      return false;
    }
  } else {
    ROS_ERROR("Failed to call distance service");
    distance_service_response = "ERROR: Service call failed";
    service_available_ = false;
    return false;
  }
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

    //        ================ Distance Service ================

    int dist_x_offset = 100;
    int dist_y_offset = 509;

    // Service status indicator
    cvui::text(frame, dist_x_offset, dist_y_offset - 20,
               "Distance Service Status:");

    // Status indicator with color coding
    if (service_available_) {
      cvui::printf(frame, dist_x_offset + 160, dist_y_offset - 20, 0.4,
                   0x00ff00, "ONLINE");
    } else {
      cvui::printf(frame, dist_x_offset + 160, dist_y_offset - 20, 0.4,
                   0xff0000, "OFFLINE");
    }

    // Distance button - always visible
    if (cvui::button(frame, dist_x_offset, dist_y_offset + 5, 120, 35,
                     "Get Distance")) {
      call_distance_service();
    }

    // Refresh service status button
    if (cvui::button(frame, dist_x_offset + 130, dist_y_offset + 5, 80, 35,
                     "Refresh")) {
      check_distance_service_availability();
    }

    // Display distance response with dynamic sizing
    dist_y_offset += 60;
    cvui::window(frame, dist_x_offset, dist_y_offset, 220, 80,
                 "Distance Service");

    // Multi-line text display for longer messages
    std::string display_text = distance_service_response;
    if (display_text.length() > 25) {
      // Break long text into multiple lines
      cvui::printf(frame, dist_x_offset + 10, dist_y_offset + 25, 0.4, 0x00ffff,
                   "%.25s", display_text.substr(0, 25).c_str());
      if (display_text.length() > 25) {
        cvui::printf(frame, dist_x_offset + 10, dist_y_offset + 40, 0.4,
                     0x00ffff, "%.25s", display_text.substr(25).c_str());
      }
    } else {
      cvui::printf(frame, dist_x_offset + 10, dist_y_offset + 35, 0.4, 0x00ffff,
                   "%s", display_text.c_str());
    }

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
