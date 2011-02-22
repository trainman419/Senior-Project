/* gps_odometry.cpp
 *
 * A ROS node to fuse GPS and raw odometry data with a kalman filter to produce
 *  better location estimates.
 *
 * Author: Austin Hendrix
 */

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "gps_common/GPSFix.h"
#include "global_map/Offset.h"

#include "matrix.h"

matrix<1, 3> position(0.0);
matrix<3, 3> covariance(0.0);

matrix<3, 3> gain; // kalman filter gain

// receive a GPS update
void gpsCallback(const gps_common::GPSFix::ConstPtr &gps) {
   ROS_INFO("Got gps message");

   // Kalman filter update step
}

// receive an odometry update
void odometryCallback(const nav_msgs::Odometry::ConstPtr &odo) {
   ROS_INFO("Got odometry message");

   matrix<1, 3> update;

   update.data[0][0] = odo->pose.pose.position.x; // x
   update.data[0][1] = odo->pose.pose.position.y; // y
   update.data[0][2] = odo->pose.pose.orientation.x; // rot TODO: pick units/normalize

   matrix<3, 3> Rt; // covariance of update noise

   /* odo->pose->covariance
      Row-major representation of the 6x6 covariance matrix
      The orientation parameters use a fixed-axis representation.
      In order, the parameters are:
      (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
      */
   // x/y pairings
   Rt.data[0][0] = odo->pose.covariance[0 + 6*0]; // x x
   Rt.data[0][1] = odo->pose.covariance[0 + 6*1]; // x y
   Rt.data[1][0] = odo->pose.covariance[1 + 6*0]; // y x
   Rt.data[1][1] = odo->pose.covariance[1 + 6*1]; // y y

   Rt.data[0][2] = odo->pose.covariance[0 + 6*3]; //  x  rot
   Rt.data[1][2] = odo->pose.covariance[1 + 6*3]; //  y  rot
   Rt.data[2][0] = odo->pose.covariance[3 + 6*0]; // rot  x
   Rt.data[2][1] = odo->pose.covariance[3 + 6*1]; // rot  y
   Rt.data[2][2] = odo->pose.covariance[3 + 6*3]; // rot rot

   // Kalman filter prediction step (inputs/odometry)
   // A and B are identity matrices
   position = update + position;
   covariance = covariance + Rt;
}

int main(int argc, char ** argv) {
   // set up kalman gains
   gain.data[0][0] = 1.0;
   gain.data[0][1] = 1.0;
   gain.data[0][2] = 1.0;
   gain.data[1][0] = 1.0;
   gain.data[1][1] = 1.0;
   gain.data[1][2] = 1.0;
   gain.data[2][0] = 1.0;
   gain.data[2][1] = 1.0;
   gain.data[2][2] = 1.0;

   ros::init(argc, argv, "gps_odometry");

   // subscribe to extended fix data from gps node.
   ros::NodeHandle n;

   ros::Subscriber gps_sub = n.subscribe("extended_fix", 10, gpsCallback);
   ros::Subscriber odo_sub = n.subscribe("base_odometry", 10, odometryCallback);

   ros::spin();

   return 0;
}
