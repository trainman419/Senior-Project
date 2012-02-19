/*
 * drive.cpp
 *
 * Drive the robot in a straight line
 *
 * Author: Austin Hendrix
 */

#include <stdio.h>
#include <time.h>

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <gps_common/GPSFix.h>
#include <hardware_interface/Compass.h>
#include <goal_list/GoalList.h>

double dist = 0.0;

double x = 0.0;
double y = 0.0;
double theta = 0.0;

FILE * logp;

void odoCallback(const nav_msgs::Odometry::ConstPtr & msg) {
   x += msg->pose.pose.position.x;
   y += msg->pose.pose.position.y;
   theta += msg->pose.pose.orientation.x;

   fprintf(logp, "O %lf %lf %lf\n", msg->pose.pose.position.x,
         msg->pose.pose.position.y, msg->pose.pose.orientation.x);
   ROS_INFO("Odometry position (%lf, %lf, %lf)", x, y, theta);
}

void gpsCallback(const gps_common::GPSFix::ConstPtr & msg) {
   fprintf(logp, "G %lf %lf\n", msg->latitude, msg->longitude);
}

void compassCallback(const hardware_interface::Compass::ConstPtr & msg) {
   fprintf(logp, "C %lf\n", msg->heading);
}

void positionCallback(const nav_msgs::Odometry::ConstPtr & msg) {
   fprintf(logp, "P %lf %lf %lf %lf %lf %lf %lf %lf\n",
         msg->pose.pose.position.x,
         msg->pose.pose.position.y,
         msg->pose.covariance[6*0 + 0], //xx
         msg->pose.covariance[6*0 + 1], //xy
         msg->pose.covariance[6*1 + 0], //yx
         msg->pose.covariance[6*1 + 1], //yy
         msg->pose.pose.orientation.x,
         msg->pose.covariance[6*0 + 0]);
}

void goalsCallback(const goal_list::GoalList::ConstPtr & msg) {
   std::vector<global_map::Location>::const_iterator itr;
   for( itr = msg->goals.begin(); itr != msg->goals.end(); itr++ ) {
      fprintf(logp, "g %d %d\n", itr->col, itr->row);
   }
}

int main(int argc, char ** argv) {
   ros::init(argc, argv, "count");

   char logfile[1024];
   char date[256];
   struct tm * timeptr;
   time_t now = time(0);

   timeptr = localtime(&now);

   strftime(date, 256, "%F-%T", timeptr);
   snprintf(logfile, 1024, "/home/hendrix/log/run-%s.log", date);

   logp = fopen(logfile, "w");

   ROS_INFO("Counting");

   ros::NodeHandle n;

   ros::Subscriber s = n.subscribe("base_odometry", 10, odoCallback);
   ros::Subscriber s2 = n.subscribe("extended_fix", 10, gpsCallback);
   ros::Subscriber s3 = n.subscribe("compass", 10, compassCallback);
   ros::Subscriber s4 = n.subscribe("position", 10, positionCallback);

   ros::spin();

   fclose(logp);
   return 0;
}
