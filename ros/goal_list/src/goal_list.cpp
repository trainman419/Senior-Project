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

// declare ourselves to be "at" a goal when we are within 5 meters
#define GOAL_DIST 5.0

// GPS coordinate at microdgrees latitude and longitude
struct GPS {
   int32_t lat;
   int32_t lon;
};

std::vector<GPS> * goals;
unsigned int current_goal;
global_map::Location current_loc;

// client for resolving lat/lon to offsets
ros::ServiceClient o_client;

// publisher for current goal
ros::Publisher goal_pub;
   
void positionCallback(const global_map::Location::ConstPtr & msg) {
   double dist = hypot(msg->row - current_loc.row, 
                        msg->col - current_loc.col);

   if( dist < GOAL_DIST ) {
      // advance to next goal
      current_goal++;
      if( current_goal >= goals->size() ) {
         // umm... do we go back to goal #1 or just stop here?
         // going back to the start is easier to handle
         current_goal = 0;
      } 
      ROS_INFO("Advancing to goal %d", current_goal);
      global_map::Offset offset;
      offset.request.lat = goals->at(current_goal).lat / 1000000.0;
      offset.request.lon = goals->at(current_goal).lon / 1000000.0;
      if( o_client.call(offset) ) {
         current_loc = offset.response.loc;
         // broadcast this as the robot's new goal.
         goal_pub.publish(current_loc);
      } else {
         ROS_ERROR("Failed to resolve offset");
      }
   }
}

int main(int argc, char ** argv) {
   ros::init(argc, argv, "goal_list");

   ros::NodeHandle n;

   o_client = n.serviceClient<global_map::Offset>("Offset");


   ros::Subscriber position = n.subscribe("position", 2, positionCallback);
   goal_pub = n.advertise<global_map::Location>("current_goal", 10);

   ros::spin();
}
