/* path_planner.cpp
 *
 * A ROS path planner node for my robot.
 * subscribes to the robot's current position and the current goal, and
 * uses this information to plan and execute a path to the goal
 *
 * Algorithm ideas:
 * we need to re-plan when:
 * - the goal changes
 * - we deviate too far from our planned path
 * - the map changes
 *
 * The first two of these are easy: we subscribe to the current location
 * and the current goal, and receive updates when either changes.
 *
 * We don't yet have a mechanism to subscribe to changes in the map in the same
 * way. Ideally, we should create one.
 *
 * All distances in decimeters
 */

#include <math.h>
#include <assert.h>
#include <stdio.h>

#include <set>
#include <list>
#include <map>
#include <vector>

#include <boost/foreach.hpp>

#include <ros/ros.h>
#include <tf/tf.h>

#include <global_map/Location.h>
#include <goal_list/Goal.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/NavSatFix.h>

using namespace std;

// length for stock line segments
#define ARC_LEN 3.0
// distance where we decide we're too far off-path
#define CLOSE_LEN 4.0
// minimum turning radius (m)
#define MIN_RADIUS 0.695 
// maximum planned radius (m)
#define MAX_RADIUS 10.0
// how close we want to get to our goal before we're "there" (m)
#define GOAL_ERR 0.3
// maximum number of iterations to look for a path
#define MAX_ITER 10000
// map resolution, in meters per pixel
#define MAP_RES 0.10

// speed for path traversal (m/s)
#define MAX_SPEED 1.0
#define MIN_SPEED 0.1
#define MAX_TRAVERSE 4.0
#define MAX_ACCEL 0.3

// types, to make life easier
struct loc {
   // x, y, pose: the position and direction of the robot
   double x;
   double y;
   double pose;
};

struct path {
   double speed;
   double radius;
};

// the local obstacle map
// fixed dimensions: 10m by 10m, centered about the associated center point
// FIXME: replace this with calls to the global_map and SLAM
int map_data[101][101];
double map_center_x = 0.0;
double map_center_y = 0.0;

// get the value of the local obstacle map at (x, y)
//  return 0 for any point not within the obstacle map
inline int map_get(double x, double y) {
   int i = round((x - map_center_x)/MAP_RES) + 50;
   int j = round((y - map_center_y)/MAP_RES) + 50;
   if( i >= 0 && i < 101 && j >= 0 && j < 101 ) {
      return map_data[i][j];
   } else {
      return 0;
   }
}

// get the value of the local obstacle map at (x, y)
//  return 0 for any point not within the obstacle map
inline void map_set(double x, double y, int v) {
   int i = round((x - map_center_x)/MAP_RES) + 50;
   int j = round((y - map_center_y)/MAP_RES) + 50;
   if( i >= 0 && i < 101 && j >= 0 && j < 101 ) {
      map_data[i][j] = v;
   }
}

void print_map() {
   cout << "Map: " << endl;
   for( int i=100; i>=0; i-- ) {
      for( int j=0; j<101; j++ ) {
         if( i==50 && j==50 ) {
            cout << "XX";
         } else {
            cout << (map_data[j][i]?"**":"__");
         }
      }
      cout << endl;
   }
}

// test if we have a collision at a particular point
bool test_collision(loc here) {
   return map_get(here.x, here.y) != 0;
}

// test an arc start at start with radius r for length l
bool test_arc(loc start, double r, double l) {
   if( r != 0.0 ) {
      // normal case; traverse an arc
      double center_x, center_y, theta;
      theta = start.pose - M_PI/2;
      center_x = start.x + r * cos(start.pose + M_PI/2);
      center_y = start.y + r * sin(start.pose + M_PI/2);

      // traverse along the arc until we hit something
      for( double dist = 0; dist < l; dist += MAP_RES/2.0 ) {
         double x = r * cos(theta + dist / r) + center_x;
         double y = r * sin(theta + dist / r) + center_y;
         if( map_get(x, y) ) {
            //ROS_WARN("Obstacle at %lf", dist);
            return false;
         }
      }
   } else {
      // degenerate case; traverse a line
      for( double dist = 0; dist < l; dist += MAP_RES/2.0 ) {
         if( map_get(start.x + dist*cos(start.pose), 
                  start.y + dist*sin(start.pose)) ) {
            //ROS_WARN("Obstacle at %lf", dist);
            return false;
         }
      }
   }
   return true;
}

nav_msgs::Path arcToPath(loc start, double r, double l) {
   nav_msgs::Path p;
   p.header.frame_id = "odom";
   if( r != 0.0 ) {
      // normal case; traverse an arc
      double center_x, center_y, theta;
      theta = start.pose - M_PI/2;
      center_x = start.x + r * cos(start.pose + M_PI/2);
      center_y = start.y + r * sin(start.pose + M_PI/2);

      // traverse along the arc until we hit something
      for( double dist = 0; dist < l; dist += MAP_RES/2.0 ) {
         double x = r * cos(theta + dist / r) + center_x;
         double y = r * sin(theta + dist / r) + center_y;
         geometry_msgs::PoseStamped pose;
         pose.header.frame_id = "odom";
         pose.pose.position.x = x;
         pose.pose.position.y = y;
         p.poses.push_back(pose);
      }
   } else {
      // degenerate case; traverse a line
      for( double dist = 0; dist < l; dist += MAP_RES/2.0 ) {
         geometry_msgs::PoseStamped pose;
         pose.header.frame_id = "odom";
         pose.pose.position.x = start.x + dist*cos(start.pose);
         pose.pose.position.y = start.y + dist*sin(start.pose);
         p.poses.push_back(pose);
      }
   }
   return p;
}

loc arc_end(loc start, double r, double l) {
   loc ret;
   if( r != 0.0 ) {
      // normal case; traverse an arc
      double center_x, center_y, theta;
      theta = start.pose - M_PI/2;
      center_x = start.x + r * cos(start.pose + M_PI/2);
      center_y = start.y + r * sin(start.pose + M_PI/2);

      ret.x = center_x + r*cos(theta + l / r);
      ret.y = center_y + r*sin(theta + l / r);
      ret.pose = theta + l / r;
   } else {
      // degenerate case: straight line
      ret.x = start.x + l*cos(start.pose);
      ret.y = start.y + l*sin(start.pose);
      ret.pose = start.pose;
   }
   return ret;
}

#define dist(a, b) hypot(a.x - b.x, a.y - b.y)

ros::Publisher path_pub;

// plan a path from start to end and update the current path
// implemented as A*
path plan_path(loc start, loc end) {
   /*
   ROS_INFO("Searching for path from (% 5.2lf, % 5.2lf) to (% 5.2lf, % 5.2lf)",
         start.x, start.y, end.x, end.y);
         */

   double theta = atan2(end.y - start.y, end.x - start.x);
   //ROS_INFO("Angle to goal: %lf", theta);

   double d = dist(start, end);
   double traverse_dist = min(d, MAX_TRAVERSE);
   double speed = min(MAX_SPEED, 
         MAX_SPEED * (2.0 * traverse_dist / MAX_TRAVERSE));
   if( speed < MIN_SPEED) speed = MIN_SPEED;
   ROS_INFO("Traverse distance %lf, speed %lf", traverse_dist, speed);

   // radius > 0 -> left
   double radius = 0;

   // test tangent arc through the goal point:
   double alpha = 2.0 * (theta - start.pose); // arc angle
   // normalize (theta - start.pose)
   while( alpha > 2.0*M_PI )  alpha -= M_PI * 4.0;
   while( alpha < -2.0*M_PI ) alpha += M_PI * 4.0;
   double arc_len = alpha; // grab inner angle before normalization

   // normalize while maintaining sign
   if( alpha >  M_PI ) alpha =  M_PI * 2.0 - alpha;
   if( alpha < -M_PI ) alpha = -M_PI * 2.0 - alpha;

   double beta = (M_PI - fabs(alpha)) / 2.0; // internal angle
   radius = d * sin(beta) / sin(alpha);
   // if we want to turn around, use minimum radius
   if( fabs(arc_len) > M_PI ) {
      if( radius > 0 ) {
         radius = MIN_RADIUS;
      } else {
         radius = -MIN_RADIUS;
      }
   }
   arc_len = arc_len * radius;

   //ROS_INFO("%lf %lf %lf", d, beta, alpha);
   if( fabs(radius) < MIN_RADIUS ) {
      //ROS_INFO("Tangent arc radius too small; looping around. %lf", radius);
      radius = 0;
      arc_len = MIN_RADIUS;
   }

   // don't plan huge sweeping curves; choose max radius
   radius = min(radius,  MAX_RADIUS);
   radius = max(radius, -MAX_RADIUS);

   arc_len = min(arc_len, MAX_TRAVERSE);

   if( !test_arc(start, radius, arc_len) ) {
      ROS_WARN("Tangent arc failed");

      // test various radii for traverse_dist
      list<double> arcs;
      if( test_arc(start, 0, traverse_dist) ) {
         arcs.push_back(0);
      }
      // 1, 2, 4, 8 * MIN_RADIUS
      for( int i=1; i<9; i *= 2 ) {
         // traverse at most a quarter turn
         double d = min(traverse_dist, MIN_RADIUS * M_PI / 2);
         if( test_arc(start, MIN_RADIUS*i, d) ) {
            arcs.push_back(MIN_RADIUS*i);
         }
         if( test_arc(start, -MIN_RADIUS*i, d) ) {
            arcs.push_back(-MIN_RADIUS*i);
         }
      }
      if( arcs.size() == 0 ) {
         ROS_WARN("No valid forward paths found");
         speed = 0;
         radius = 0;
      } else {
         // choose arc that gets us closest to goal
         //ROS_INFO("%zd forward paths found", arcs.size());
         double best_r = arcs.front();
         double l = min(traverse_dist, best_r * M_PI / 2);
         loc e = arc_end(start, best_r, l);
         double min_dist = dist(e, end);

         BOOST_FOREACH(double r, arcs) {
            l = min(traverse_dist, r * M_PI / 2);
            e = arc_end(start, r, l);
            double d = dist(e, end);
            if( d < min_dist ) {
               best_r = r;
               min_dist = d;
            }
         }
         radius = best_r;
         nav_msgs::Path p = arcToPath(start, best_r, 
               min(traverse_dist, best_r * M_PI / 2));
         path_pub.publish(p);
      }
   } else {
      nav_msgs::Path p = arcToPath(start, radius, arc_len);
      path_pub.publish(p);
   }

   path p;
   p.radius = radius;
   p.speed = speed;
   return p;
}

// publisher for publishing movement commands
ros::Publisher cmd_pub;
// publisher for map
ros::Publisher map_pub;

bool active = true;
bool path_valid = false;
loc goal;

void goalCallback(const geometry_msgs::Point::ConstPtr & msg) {
   goal.x = msg->x;
   goal.y = msg->y;
}

// the last location we were at.
//  used as the center point for our local map
loc last_loc;
geometry_msgs::Pose last_pose;
   
void odomCallback(const nav_msgs::Odometry::ConstPtr & msg) {
   loc here;
   here.x = msg->pose.pose.position.x;
   here.y = msg->pose.pose.position.y;
   here.pose = tf::getYaw(msg->pose.pose.orientation);

   last_loc = here;
   last_pose = msg->pose.pose;
   if( active ) {
      geometry_msgs::Twist cmd;

      //ROS_INFO("Current angle: %lf", here.pose);

      path p = plan_path(here, goal);
      double radius = p.radius;
      double speed = p.speed;

      // limit acceleration
      if( speed > 0 ) {
         speed = min(speed, msg->twist.twist.linear.x + MAX_ACCEL);
      } else if( speed < 0 ) {
         speed = max(speed, msg->twist.twist.linear.x - MAX_ACCEL);
      }
      // no limit on deceleration


      // steering radius = linear / angular
      // angular = linear / radius
      if( radius != 0.0 ) {
         cmd.angular.z = speed / radius;
      } else {
         cmd.angular.z = 0;
      }

      //ROS_INFO("Target radius: %lf, angular: %lf", radius, cmd.angular.z);

      if( dist(here, goal) > GOAL_ERR ) {
         //ROS_INFO("Target speed: %lf", speed);
         cmd.linear.x = speed;
      } else {
         cmd.linear.x = 0;
         cmd.angular.z = 0;
      }
      cmd_pub.publish(cmd);
   } else {
      geometry_msgs::Twist cmd;
      cmd_pub.publish(cmd);
   }
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr & msg) {
   map_center_x = last_loc.x;
   map_center_y = last_loc.y;

   double theta_base = last_loc.pose;

   double theta = theta_base + msg->angle_min;
   double x;
   double y;

   // clear map
   memset(map_data, 0, 101*101*sizeof(int));

   for( unsigned int i=0; i<msg->ranges.size(); i++, 
         theta += msg->angle_increment ) {
      // 0 means max range... I think
      if( msg->ranges[i] != 0.0 ) {
         x = map_center_x + msg->ranges[i]*cos(theta); 
         y = map_center_y + msg->ranges[i]*sin(theta);
         map_set(x, y, 1);
      }
   }

   // we hope we aren't sitting on an obstacle
   map_data[50][50] = 0;

   // grow obstacles by radius of robot; makes collision-testing easier
   // order: O(n^2 * 12)
   for( int r=1; r<(0.4/MAP_RES); r++ ) {
      for( int i=0; i<101; i++ ) {
         for( int j=0; j<101; j++ ) {
            if( map_data[i][j] == 0 ) {
               if( i > 0   && map_data[i-1][j] == r ) map_data[i][j] = r+1;
               if( j > 0   && map_data[i][j-1] == r ) map_data[i][j] = r+1;
               if( i < 100 && map_data[i+1][j] == r ) map_data[i][j] = r+1;
               if( j < 100 && map_data[i][j+1] == r ) map_data[i][j] = r+1;
            }
         }
      }
   }

   // publish map
   nav_msgs::OccupancyGrid map;
   map.header = msg->header;
   map.header.frame_id = "odom";
   map.info.resolution = MAP_RES;
   map.info.width = 101;
   map.info.height = 101;
   map.info.origin.position.x = last_loc.x - 5.0;
   map.info.origin.position.y = last_loc.y - 5.0;
   map.info.origin.orientation.w = 1.0;
   for( int i=0; i<101; i++ ) {
      for( int j=0; j<101; j++ ) {
         map.data.push_back(map_data[j][i]);
      }
   }
   map_pub.publish(map);
   return;
}

int main(int argc, char ** argv) {
   // set map to empty
   for( int i=0; i<101; i++ ) {
      for( int j=0; j<101; j++ ) {
         map_data[i][j] = 0;
      }
   }

   ros::init(argc, argv, "path_planner");

   ros::NodeHandle n;

   // subscribe to our location and current goal
   ros::Subscriber odom_sub = n.subscribe("odom", 2, odomCallback);
   ros::Subscriber goal_sub = n.subscribe("current_goal", 2, goalCallback);
   ros::Subscriber laser_sub = n.subscribe("scan", 2, laserCallback);

   cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
   map_pub = n.advertise<nav_msgs::OccupancyGrid>("map", 10);
   path_pub = n.advertise<nav_msgs::Path>("path", 10);

   ROS_INFO("Path planner ready");

   ros::spin();
}
