/* gps_odometry.cpp
 *
 * A ROS node to fuse GPS and raw odometry data with a kalman filter to produce
 *  better location estimates.
 *
 * Author: Austin Hendrix
 */

#include <iostream>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "gps_common/GPSFix.h"
#include "global_map/Offset.h"
#include "global_map/RevOffset.h"
#include "global_map/SetMeridian.h"
#include "hardware_interface/Compass.h"

#include "matrix.h"

using namespace std;

matrix<1, 2> position(0.0);
matrix<2, 2> p_covariance(0.0);

double heading = 0;
double h_variance = 10;

// client for retrieving offsets
ros::ServiceClient set_meridian;
ros::ServiceClient o_client;
ros::ServiceClient r_client;

// publisher for filtered position information
ros::Publisher pos_pub;

// global for storing compass reading when we get it
double compass = 0.0;

int16_t old_meridian;

#define ODO_CNT 60
char valid = 0;

void publish() {
   nav_msgs::Odometry pos;
   pos.pose.pose.position.x = position.data[0][0];
   pos.pose.pose.position.y = position.data[0][1];
   //pos.pose.pose.orientation.x = position.data[0][2];
   pos.pose.pose.orientation.x = heading;

   pos.pose.covariance[0 + 6*0] = p_covariance.data[0][0]; // x x
   pos.pose.covariance[0 + 6*1] = p_covariance.data[0][1]; // x y
   pos.pose.covariance[1 + 6*0] = p_covariance.data[1][0]; // y x
   pos.pose.covariance[1 + 6*1] = p_covariance.data[1][1]; // y y

   /*pos.pose.covariance[0 + 6*3] = p_covariance.data[0][2]; //  x  rot
   pos.pose.covariance[1 + 6*3] = p_covariance.data[1][2]; //  y  rot
   pos.pose.covariance[3 + 6*0] = p_covariance.data[2][0]; // rot  x
   pos.pose.covariance[3 + 6*1] = p_covariance.data[2][1]; // rot  y
   */
   pos.pose.covariance[3 + 6*3] = h_variance; // rot rot

   pos_pub.publish(pos);
}

// receive a GPS update
void gpsCallback(const gps_common::GPSFix::ConstPtr &gps) {
   //ROS_INFO("Got gps message");

   if( valid >= 0 && gps->status.status == 0 ) {

      matrix<2, 2> gain; // kalman filter gain
      // Kalman filter update step

      // input covariance
      matrix<2, 2> Q;
      // diagonal matrix
      Q.data[0][0] = 25.0*25.0;
      Q.data[0][1] = 0;
      Q.data[1][1] = 25.0*25.0;
      Q.data[1][0] = 0;

      int16_t meridian = round(gps->longitude);
      if( meridian != old_meridian ) {
         global_map::SetMeridian m;
         m.request.meridian = meridian;
         if( !set_meridian.call(m) ) {
            ROS_ERROR("Failed to set meridian");
         }
      }

      global_map::Offset offset;
      offset.request.lat = gps->latitude;
      offset.request.lon = gps->longitude;
      if( !o_client.call(offset) ) {
         ROS_ERROR("Failed to call Offset service");
      }
      // input measurement
      matrix<1, 2> z;
      z.data[0][0] = offset.response.loc.col; // x; from gps
      z.data[0][1] = offset.response.loc.row; // y; from gps

      // measurement update step:
      // C: 2x2 identity matrix
      // Q: measurement covariance
      // z: measurement
      // I: identity matrix
      // gain = cov * C^T * (C * cov * C^T + Q)^-1
      // pos = pos + gain*(z - C*pos)
      // cov = (I - gain*C)*cov

      if( valid > ODO_CNT ) {
         gain = p_covariance * invert(p_covariance + Q);
         position = position + (z - position) * gain;
         p_covariance = (I<2>() - gain) * p_covariance;
      } else {
         position = z;
         p_covariance = Q;
         valid = ODO_CNT + 1;
      }

      while( position.data[0][2] >  M_PI ) position.data[0][2] -= M_PI*2;
      while( position.data[0][2] < -M_PI ) position.data[0][2] += M_PI*2;

      publish();
   }
}

void compassCallback(const hardware_interface::Compass::ConstPtr & msg ) {
   //ROS_INFO("Got compass message");
   compass = msg->heading;
   // TODO: update compass when we get compass readings

   double z = compass;

   while( z - heading < -M_PI ) z += M_PI*2;
   while( z - heading >  M_PI ) z -= M_PI*2;

   //double Q = 0.174*0.174; // about 10 degrees, in radians
   double Q = 0.100*0.100; // about 10 degrees, in radians

   double gain = h_variance / ( h_variance + Q);
   heading = heading + ((z - heading) * gain);
   h_variance = (1 - gain) * h_variance;

   while( heading >  M_PI ) heading -= M_PI*2;
   while( heading < -M_PI ) heading += M_PI*2;

   publish();
}

// receive an odometry update
void odometryCallback(const nav_msgs::Odometry::ConstPtr &odo) {
   //ROS_INFO("Got odometry message");

   matrix<1, 2> update;
   double h_update;

   update.data[0][0] = odo->pose.pose.position.x; // x
   update.data[0][1] = odo->pose.pose.position.y; // y
   h_update = odo->pose.pose.orientation.x; // rot TODO: pick units/normalize

   while( h_update - position.data[0][2] < -M_PI ) h_update += M_PI*2;
   while( h_update - position.data[0][2] >  M_PI ) h_update -= M_PI*2;

   matrix<2, 2> Rt; // covariance of update noise
   double h_Rt;

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

   h_Rt = odo->pose.covariance[3 + 6*3]; // rot rot

   // Kalman filter prediction step (inputs/odometry)
   // A and B are identity matrices
   position = update + position;
   p_covariance = p_covariance + Rt;
   heading = h_update + heading;
   h_variance += h_Rt;

//   if( valid < ODO_CNT ) valid += 1;
//   if( valid == ODO_CNT ) valid = -1;

   publish();
}

int main(int argc, char ** argv) {
   // initialize uncertainty matrix
   for( int i=0; i<2; i++ ) {
      p_covariance.data[i][i] = 1000.0*1000.0;
   }

   ros::init(argc, argv, "gps_odometry");

   // subscribe to extended fix data from gps node.
   ros::NodeHandle n;

   o_client = n.serviceClient<global_map::Offset>("Offset");
   r_client = n.serviceClient<global_map::RevOffset>("RevOffset");
   set_meridian = n.serviceClient<global_map::SetMeridian>("SetMeridian");

   pos_pub = n.advertise<nav_msgs::Odometry>("position", 10);

   ros::Subscriber gps_sub = n.subscribe("extended_fix", 10, gpsCallback);
   ros::Subscriber odo_sub = n.subscribe("base_odometry", 50, odometryCallback);
   ros::Subscriber compass_sub = n.subscribe("compass", 10, compassCallback);

   ROS_INFO("gps_odometry running");

   ros::spin();

   return 0;
}
