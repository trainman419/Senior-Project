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

#include <nav_msgs/Odometry.h>

// declare ourselves to be "at" a goal when we are within 1 meter
#define GOAL_DIST 10.0

using namespace std;

vector<global_map::Location> * goals;
unsigned int current_goal;

// publisher for current goal
ros::Publisher goal_pub;
   
void positionCallback(const nav_msgs::Odometry::ConstPtr & msg) {
   ROS_INFO("Got position update");

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
            // umm... do we go back to goal #1 or just stop here?
            // going back to the start is easier to handle
            current_goal = goals->size();
            g.valid = 0;
         } else {
            ROS_INFO("Advancing to goal %d", current_goal);
            g.loc = goals->at(current_goal);
            g.valid = 1;
         }
         goal_pub.publish(g);
      }
   }
}

// TODO: rewrite this so that we don't have to have a thousand special cases
//  most likely this means implementing full-duplex goal list exchange with
//  the UI, and letting it determine what the next goal is
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

int main(int argc, char ** argv) {
   goals = new vector<global_map::Location>();

   ros::init(argc, argv, "goal_list");

   ros::NodeHandle n;

   ros::Subscriber position = n.subscribe("position", 2, positionCallback);
   ros::Subscriber goalList = n.subscribe("goal_list", 2, goalListCallback);
   goal_pub = n.advertise<goal_list::Goal>("current_goal", 10);

   ros::spin();
}
