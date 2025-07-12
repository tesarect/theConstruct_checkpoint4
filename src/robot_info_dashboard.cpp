/*
The CVUIROSSubscriber class creates a graphical user interface featuring a
window and text. When a ROS message is received, the gui displays the incomming
data inside the window.

To use this class, include the show_received_messages.h header file in your
main program file and create an instance of the CVUIROSSubscriber class. You
would then call the 'run' function on the instance to start the program.

Author: Roberto Zegers
Date: February 2023
License: BSD-3-Clause
*/

#include "robot_gui/robot_info_dashboard.h"
#include <vector>

CVUIROSSubscriber::CVUIROSSubscriber() {
  // Initialize ROS node
  ros::NodeHandle nh;
  //   topic_name = "info";
  topic_name = "robot_info";
  //   sub_ = nh.subscribe<std_msgs::Float64>(topic_name, 2,
  //                                          &CVUIROSSubscriber::msgCallback,
  //                                          this);
  sub_ = nh.subscribe<robotinfo_msgs::RobotInfo10Fields>(
      topic_name, 2, &CVUIROSSubscriber::msgCallback, this);
}

// void CVUIROSSubscriber::msgCallback(const std_msgs::Float64::ConstPtr &msg) {
//   data = *msg;
//   ROS_DEBUG("Number received: %f", msg->data);
// }

void CVUIROSSubscriber::msgCallback(
    const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg) {
  robot_data = *msg;
  ROS_DEBUG("Robot info received");
}

void CVUIROSSubscriber::run() {
  // create a frame
  //   cv::Mat frame = cv::Mat(600, 800, CV_8UC3);
  cv::Mat frame = cv::Mat(600, 400, CV_8UC3);

  // Init a OpenCV window and tell cvui to use it.
  cv::namedWindow(WINDOW_NAME);
  cvui::init(WINDOW_NAME);

  while (ros::ok()) {
    // Fill the frame with a nice color
    frame = cv::Scalar(49, 52, 49);

    //           ============== General Info Area ================
    // Create window at (40, 20) with size 250x80 (width x height) and title
    // cvui::window(frame, 40, 20, 250, 40, "Topic: " + topic_name);
    int x_offset = 50;
    int y_offset = 20;
    // cvui::window(frame, x_offset, y_offset, 250, 90, "Topic: " + topic_name);
    cvui::window(frame, x_offset, y_offset, 300, 190, "Topic: " + topic_name);

    std::vector<std::string> data_fields = {
        robot_data.data_field_01, robot_data.data_field_02,
        robot_data.data_field_03, robot_data.data_field_04,
        robot_data.data_field_05, robot_data.data_field_06,
        robot_data.data_field_07, robot_data.data_field_08};

    // // Show how many times the button has been clicked inside the window.
    // cvui::printf(frame, 45, 45, 0.4, 0xff0000, "Data received: %0.2f", data);

    // Show non-empty fields
    int text_x_offset = x_offset + 5;
    int line_height = 20;
    for (const auto &field : data_fields) {
      if (!field.empty()) {
        // cvui::printf(frame, 40, y_offset, 0.5, 0xff0000, "%s",
        // field.c_str());
        cvui::printf(frame, text_x_offset, y_offset + 27, 0.4, 0xffffff, "%s",
                     field.c_str());
        y_offset += line_height;
      }
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
