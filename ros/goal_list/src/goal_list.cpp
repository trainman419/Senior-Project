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

// declare ourselves to be "at" a goal when we are within 1 meter
#define GOAL_DIST 10.0

using namespace std;

//vector<global_map::Location> * goals;
vector<sensor_msgs::NavSatFix> * goals;
unsigned int current_goal;

int loop = 0;

// publisher for current goal
ros::Publisher goal_pub;

geometry_msgs::Point last_odom;
   
void positionCallback(const nav_msgs::Odometry::ConstPtr & msg) {
   ROS_INFO("Got position update");

   last_odom = msg->pose.pose.position;
   
   /*
   if( current_goal < goals->size() ) {
      double dist = hypot(msg->pose.pose.position.y - 
                              goals->at(current_goal).row, 
                          msg->pose.pose.position.x - 
                              goals->at(current_goal).col);

      if( dist < GOAL_DIST ) {
         // advance to next goal
         current_goal++;
         goal_list::Goal g;
         if( current_goal >= goals->size() ) {
            if( loop ) {
               // go back to start, if looping
               current_goal = 0;
               g.valid = 1;
               g.loc = goals->at(current_goal);
            } else {
               // else, stop
               current_goal = goals->size();
               g.valid = 0;
            }
         } else {
            ROS_INFO("Advancing to goal %d", current_goal);
            g.loc = goals->at(current_goal);
            g.valid = 1;
         }
         goal_pub.publish(g);
      }
   }
   */
}

bool active = false;

void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr & msg) {
   geometry_msgs::Point goal = last_odom; // goal, in odom frame
   if( active ) {
      // TODO: compute offset to goal in m

      // use last_odom as the position in the odom frame that corresponds to
      // this GPS location
      
      // if we're close enough to goal, swtich to next goal
   }
   goal_pub.publish(goal);
}

// TODO: rewrite this so that we don't have to have a thousand special cases
//  most likely this means implementing full-duplex goal list exchange with
//  the UI, and letting it determine what the next goal is
/*
void goalListCallback(const goal_list::GoalList::ConstPtr & msg) {
   ROS_INFO("Got Goal List, length %d", msg->goals.size());
   int oldsize = goals->size();
   *goals = msg->goals;
   if( goals->size() == 0) {
      goal_list::Goal g;
      g.valid = 0;
      goal_pub.publish(g);
   } else if( goals->size() < current_goal || oldsize == 0 ) {
      current_goal = 0;
      goal_list::Goal g;
      g.valid = 1;
      g.loc = goals->at(current_goal);
      goal_pub.publish(g);
   } else if( goals->size() > current_goal && current_goal == oldsize ) {
      goal_list::Goal g;
      g.valid = 1;
      g.loc = goals->at(current_goal);
      goal_pub.publish(g);
   }
}
*/

int main(int argc, char ** argv) {
   goals = new vector<sensor_msgs::NavSatFix>();

   ros::init(argc, argv, "goal_list");

   ros::NodeHandle n;

   // TODO: load goal list from parameter server

   ros::Subscriber position = n.subscribe("position", 2, positionCallback);
   ros::Subscriber gps = n.subscribe("gps", 2, gpsCallback);
   //ros::Subscriber goalList = n.subscribe("goal_list", 2, goalListCallback);
   goal_pub = n.advertise<geometry_msgs::Point>("current_goal", 10);

   ROS_INFO("Goal List ready");

   ros::spin();
}
