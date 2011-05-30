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
#include <goal_list/GoalList.h>

double dist = 0.0;
bool done = false;

ros::Publisher goalList_pub;

ros::Subscriber pos;

// callback on position
void posCallback(const nav_msgs::Odometry::ConstPtr & msg) {
   double x = msg->pose.pose.position.x;
   double y = msg->pose.pose.position.y;
   double t = M_PI/2.0 - msg->pose.pose.orientation.x;
   ROS_INFO("Got position: (%lf, %lf, %lf)", x, y, t);

   goal_list::GoalList list;
   // publish an empty goal list
   goalList_pub.publish(list);

   double x1, y1;
   global_map::Location loc;

   x1 = x + dist*cos(t);
   y1 = y + dist*sin(t);
   loc.col = x1;
   loc.row = y1;
   list.goals.push_back(loc);

   x1 = x + dist*cos(t) + dist*cos(t + M_PI/2.0);
   y1 = y + dist*sin(t) + dist*sin(t + M_PI/2.0);
   loc.col = x1;
   loc.row = y1;
   list.goals.push_back(loc);

   x1 = x + dist*cos(t + M_PI/2.0);
   y1 = y + dist*sin(t + M_PI/2.0);
   loc.col = x1;
   loc.row = y1;
   list.goals.push_back(loc);

   x1 = x;
   y1 = y;
   loc.col = x1;
   loc.row = y1;
   list.goals.push_back(loc);

   // publish a goal list with points in a square (I hope)
   goalList_pub.publish(list);

   // unsubscribe from further position updates
   pos.shutdown();
   done = true;
}

int main(int argc, char ** argv) {
   ros::init(argc, argv, "drive");

   if( argc == 2 ) {
      sscanf(argv[1], "%lf", &dist);
      ROS_INFO("Driving in square with side length %lf units", dist);
   } else {
      return 0;
   }

   ros::NodeHandle n;

   goalList_pub = n.advertise<goal_list::GoalList>("goal_list", 2);

   pos = n.subscribe("position", 10, posCallback);

   while( !done && ros::ok() ) {
      ros::spinOnce();
   }
   return 0;
}
