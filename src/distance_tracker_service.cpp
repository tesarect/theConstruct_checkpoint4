/*
This is a ROS node that subscribes to a message of type nav_msgs/Odometry
and uses the incoming data to calculate the distance traveled which is
stored on a member variable. The node also has a service server which sends
a response with the distance traveled each time a request of type
std_srvs/Trigger.h is received.
*/

