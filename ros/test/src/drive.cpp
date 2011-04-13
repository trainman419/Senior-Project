/*
 * drive.cpp
 *
 * Drive the robot in a straight line
 *
 * Author: Austin Hendrix
 */

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <hardware_interface/Control.h>

double dist = 0.0;

ros::Publisher control_pub;

double x = 0.0;
double y = 0.0;
double theta = 0.0;

void odoCallback(const nav_msgs::Odometry::ConstPtr & msg) {
   x += msg->pose.pose.position.x;
   y += msg->pose.pose.position.y;
   theta += msg->pose.pose.orientation.x;

   ROS_INFO("Odometry position (%lf, %lf, %lf)", x, y, theta);

   hardware_interface::Control c;

   if( hypot(x, y) < dist ) {
      c.speed = 15;
   } else {
      c.speed = 0;
   }
   c.steer = 0; // drive straight

   control_pub.publish(c);
}

int main(int argc, char ** argv) {
   ros::init(argc, argv, "drive");

   if( argc >= 2 ) {
      sscanf(argv[1], "%lf", &dist);
   }
   ROS_INFO("Driving forward %lf units", dist);

   ros::NodeHandle n;

   control_pub = n.advertise<hardware_interface::Control>("control", 10);

   ros::Subscriber s = n.subscribe("base_odometry", 10, odoCallback);

   while( hypot(x, y) < dist && ros::ok() ) {
      ros::spinOnce();
   }
   hardware_interface::Control c;
   c.speed = 0;
   c.steer = 0;
   control_pub.publish(c);
   //ros::spinOnce(
   ROS_INFO("Goal reached");
   return 0;
}
