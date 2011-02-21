/* localization.cpp
 *
 * Localization module.
 *
 * Author: Austin Hendrix
 */

/* Design notes:
 *
 * This module should subscribe to Odometry, Laser, GPS, compass and sonar data
 *  for starters, we'll subscribe to Odometry, Laser and GPS
 *
 * Implementation ideas:
 *  particle filter
 *  bayes filter
 *  kalman filter
 *
 * Idea: particle filter
 *  use GPS data to limit possible test states
 *  in theory, we should be able to get away with a relatively small number of
 *   particles; I'm thinking a few dozen, max
 */

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"

// callback for receiving odometry messages
void odometryCallback(const nav_msgs::Odometry::ConstPtr &odo) {
}

// callback for receiving laser scans
void laserCallback(const sensor_msgs::LaserScan::ConstPtr &scan) {
}

// callback for receiving GPS data
void gpsCallback(void) {
}

int main(int argc, char ** argv) {

   ros::init(argc, argv, "localization");

   ros::NodeHandle n;

   // subscribe to laser scan data
   ros::Subscriber laser_sub = n.subscribe("scan", 10, laserCallback);

   // subscribe to 
   ros::Subscriber odom_sub = n.subscribe("odometry", 10, odometryCallback);

   return 0;
}
