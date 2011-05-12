/*
 * count.cpp
 *
 * Integrate odometry information to produce a position
 *
 * Author: Austin Hendrix
 */

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>

double dist = 0.0;

double x = 0.0;
double y = 0.0;
double theta = 0.0;

void odoCallback(const nav_msgs::Odometry::ConstPtr & msg) {
   x += msg->pose.pose.position.x;
   y += msg->pose.pose.position.y;
   theta += msg->pose.pose.orientation.x;

   ROS_INFO("Odometry position (%lf, %lf, %lf)", x, y, theta);
}

int main(int argc, char ** argv) {
   ros::init(argc, argv, "count");

   ROS_INFO("Counting");

   ros::NodeHandle n;

   ros::Subscriber s = n.subscribe("base_odometry", 10, odoCallback);

   ros::spin();
   return 0;
}
