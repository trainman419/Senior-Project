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

int main(int argc, char ** argv) {
   ros::init(argc, argv, "steer");

   int steer = 0;

   if( argc >= 2 ) {
      sscanf(argv[1], "%d", &steer);
   }

   ros::NodeHandle n;

   ros::Publisher control_pub = n.advertise<hardware_interface::Control>("control", 10);

   ros::Rate loop_rate(1);
   ROS_INFO("Setting steering to %d", steer);

   while( ros::ok() ) {
      hardware_interface::Control c;
      c.speed = 0;
      c.steer = steer;
      control_pub.publish(c);

      ros::spinOnce();

      loop_rate.sleep();
   }


   ros::spin();

   return 0;
}
