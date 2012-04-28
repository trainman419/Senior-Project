/*
 * goal_list.cpp
 * A list of GPS goals for the robot, maintained as a ROS node.
 *
 * Author: Austin Hendrix
 */

#include <stdint.h>
#include <math.h>

#include <vector>

#include <ros/ros.h>
#include <global_map/Location.h>
#include <global_map/Offset.h>

#include <goal_list/Goal.h>
#include <goal_list/GoalList.h>

#include <geometry_msgs/Point.h>
#include <sensor_msgs/NavSatFix.h>

#include <nav_msgs/Odometry.h>

using namespace std;

//vector<global_map::Location> * goals;
vector<sensor_msgs::NavSatFix> * goals;
unsigned int current_goal;

bool loop = false;

// publisher for current goal
ros::Publisher goal_pub;

geometry_msgs::Point last_odom;
   
void odomCallback(const nav_msgs::Odometry::ConstPtr & msg) {
   ROS_INFO("Got position update");

   last_odom = msg->pose.pose.position;
}

bool active = false;

// Radius of earth in m
#define R 6371000

// Goal Tolerance
//  TODO: convert to parameter shared with path planner
#define GOAL_TOLERANCE 5.0

void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr & msg) {
   geometry_msgs::Point goal = last_odom; // goal, in odom frame
   if( active ) {
      sensor_msgs::NavSatFix gps_goal = goals->at(current_goal);

      // use last_odom as the position in the odom frame that corresponds to
      // this GPS location
      double start_lat, start_lon, end_lat, end_lon;
      start_lat = msg->latitude / 180.0 * M_PI;
      start_lon = msg->longitude / 180.0 * M_PI;
      end_lat = gps_goal.latitude / 180.0 * M_PI;
      end_lon = gps_goal.longitude / 180.0 * M_PI;

      // Haversine formula (http://www.movable-type.co.uk/scripts/latlong.html)
      double delta_lat = end_lat - start_lat;
      double delta_lon = end_lon - start_lon;
      double a = sin(delta_lat/2.0)*sin(delta_lat/2.0) + 
         cos(start_lat)*cos(end_lat)*sin(delta_lon/2.0)*sin(delta_lon/2);
      double c = 2 * atan2(sqrt(a), sqrt(1 - a));
      double d = R * c; // distance to goal


      // Bearing formula (from above)
      //  expressed as radians east of North
      double theta = atan2( sin(delta_lon)*cos(end_lat),
            cos(start_lat)*sin(end_lat) - 
            sin(start_lat)*cos(end_lat)*cos(delta_lon));

      // convert to radians North of East
      double heading = (M_PI / 2.0) - theta;

      ROS_INFO("Goal %d: distance %lf, heading %lf", current_goal, d, heading);

      // no need to normalize heading
      goal.x = last_odom.x + d * cos(heading);
      goal.y = last_odom.y + d * sin(heading);


      // if we're close enough to goal, swtich to next goal
      if( d < GOAL_TOLERANCE ) {
         ROS_INFO("Goal Reached");
         ++current_goal;
         if( current_goal >= goals->size() ) {
            if( loop ) {
               current_goal = 0;
               ROS_INFO("Last goal. Looping around");
               // TODO: convert new goal to odom frame
            } else {
               active = false;
               ROS_INFO("Last goal. Deactivating");
            }
         }
      }
   }
   goal_pub.publish(goal);
}

int main(int argc, char ** argv) {
   goals = new vector<sensor_msgs::NavSatFix>();
   current_goal = 0;

   ros::init(argc, argv, "goal_list");

   ros::NodeHandle n;

   // load goal list from parameter server
   XmlRpc::XmlRpcValue xml_goals;
   n.getParam("goals", xml_goals);
   if( xml_goals.getType() != XmlRpc::XmlRpcValue::TypeArray ) {
      ROS_ERROR("param 'goals' is not a list");
   } else {
      for( int i=0; i<xml_goals.size(); ++i ) {
         if( xml_goals[i].getType() != XmlRpc::XmlRpcValue::TypeArray ) {
            ROS_ERROR("goals[%d] is not a list", i);
         } else {
            if( xml_goals[i].size() != 2 ) {
               ROS_ERROR("goals[%d] is not a pair", i);
            } else if( 
             xml_goals[i][0].getType() != XmlRpc::XmlRpcValue::TypeDouble ||
             xml_goals[i][1].getType() != XmlRpc::XmlRpcValue::TypeDouble ) {
               ROS_ERROR("goals[%d] is not a pair of doubles", i);
            } else {
               sensor_msgs::NavSatFix g;
               g.latitude = xml_goals[i][0];
               g.longitude = xml_goals[i][1];
               goals->push_back(g);
            }
         }
      }
   }
   ROS_INFO("Loaded %d goals", goals->size());

   active = goals->size() > 0;
   n.getParam("loop", loop);


   ros::Subscriber odom = n.subscribe("odom", 2, odomCallback);
   ros::Subscriber gps = n.subscribe("gps", 2, gpsCallback);
   goal_pub = n.advertise<geometry_msgs::Point>("current_goal", 10);

   ROS_INFO("Goal List ready");

   ros::spin();
}
