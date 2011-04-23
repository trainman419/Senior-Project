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

#include <ros/ros.h>

#include <global_map/Location.h>
#include <hardware_interface/Control.h>
#include <goal_list/Goal.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

using namespace std;

// length for stock line segments
#define ARC_LEN 3.0

// distance where we decide we're too far off-path
#define CLOSE_LEN 4.0

// minimum turning radius
#define MIN_RADIUS 20.0

// how close we want to get to our goal before we're "there"
#define GOAL_ERR 3.0

// maximum number of iterations to look for a path
#define MAX_ITER 10000

// types, to make life easier
struct loc {
   // x, y, pose: the position and direction of the robot
   double x;
   double y;
   double pose;

   // for path-finding: the node previous to this one
   int prev;
   // for path-finding: this node's index
   int idx;
   // for path-finding: the cost to get to this node
   double cost;
   // the arc we followed to get to this point
   int steer;
};

// our current path
list<loc> * path;

// the local obstacle map
// fixed dimensions: 10m by 10m, centered about the associated center point
// FIXME: replace this with calls to the global_map and SLAM
int map_data[101][101];
double map_center_x = 0.0;
double map_center_y = 0.0;

// get the value of the local obstacle map at (x, y)
//  return 0 for any point not within the obstacle map
inline int map_get(double x, double y) {
   int i = round(x - map_center_x) + 50;
   int j = round(y - map_center_y) + 50;
   if( i >= 0 && i < 101 && j >= 0 && j < 101 ) {
      return map_data[i][j];
   } else {
      return 0;
   }
}

// get the value of the local obstacle map at (x, y)
//  return 0 for any point not within the obstacle map
inline void map_set(double x, double y, int v) {
   int i = round(x - map_center_x) + 50;
   int j = round(y - map_center_y) + 50;
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

#define dist(a, b) hypot(a.x - b.x, a.y - b.y)

// plan a path from start to end and update the current path
// implemented as A*
void plan_path(loc start, loc end) {
   loc here = start;

   print_map();

   ROS_INFO("Searching for path from (% 5.2lf, % 5.2lf) to (% 5.2lf, % 5.2lf)",
         start.x, start.y, end.x, end.y);
   int iter = 0;

   // identify positions where we've been by an int and store them
   vector<loc> * points = new vector<loc>();
   // sorted list of points, in order from closest to furthest
   //  we don't index this, we just use it for the fact that it's sorted
   //  and we pull the first element off the list each time
   map<double, int> * unvisited = new map<double, int>();

   here.idx = 0;
   here.cost = 0;
   points->push_back(here);
   (*unvisited)[dist(here, end)] = 0;

   int steer[] = {-100, -64, -32, -16, -8, -4, -2, 0, 2, 4, 8, 16, 32, 64, 100};

   while( dist(here, end) > GOAL_ERR && iter < MAX_ITER
         && unvisited->size() > 0 ) {
      iter++;

      // visit the point nearest the goal
      assert(unvisited->size() > 0);
      map<double, int>::iterator next = unvisited->begin();

      assert(next->second < points->size());
      here = points->at(next->second);

      unvisited->erase(next);

      cout << "Here: (" << here.x << ", " << here.y << ", " << here.pose << 
         ")" << endl;

      // FIXME: only drive forward

      // generate points to visit
      for( unsigned int i=0; i<(sizeof(steer)/sizeof(int)); i++ ) {
      //for( unsigned int i=0; i<15; i++ ) {
         double d = ARC_LEN;  // distance to travel
         double r = 0;

         double dx, dy, dt;

         double theta = here.pose;
         if( steer[i] == 0 ) {
            // if we're going straight, just generate a straight-line estimate
            dx = d * cos(theta);
            dy = d * sin(theta);
            dt = 0.0;
         } else {
            // radius of turn
            r = (786.4 - 170.2 * log(fabs(steer[i]))) / 10.0;
            dt = d / r; // in rads

            double theta_c1; // in radians
            double theta_c2; // in radians
            if( steer[i] > 0 ) {
               // turning right
               dt = -dt;
               theta_c1 = theta + M_PI/2;
            } else {
               // turning left
               theta_c1 = theta - M_PI/2;
            }
            theta_c2 = theta_c1 + dt;

            dx = r * (cos(theta_c2) - cos(theta_c1));
            dy = r * (sin(theta_c2) - sin(theta_c1));

            if( steer[i] < 0 ) {
               r = -r;
            }
         }

         loc n;
         n.x = here.x + dx;
         n.y = here.y + dy;
         n.pose = here.pose + dt;
         n.prev = here.idx;
         n.steer = steer[i];
         n.cost = here.cost + ARC_LEN;

         while( n.pose < -M_PI ) n.pose += M_PI*2;
         while( n.pose >  M_PI ) n.pose -= M_PI*2;

         if( !test_collision(n) ) {
            n.idx = points->size();
            points->push_back(n);

            double len = dist(n, end) + n.cost;
            (*unvisited)[len] = n.idx;
         }
      }
   }

   ROS_INFO("Found path in %d iterations!", iter);

   path->clear();

   // walk backwards and build path
   while( here.idx != 0 ) {
      path->push_front(here);
      here = points->at(here.prev);
   }

   ROS_INFO("Path with %d points:", path->size());

   for( list<loc>::iterator itr = path->begin(); itr != path->end(); itr++ ) {
      ROS_INFO("Point % 3d (% 4.2lf, % 4.2lf, % 4.2lf) steer %d", itr->idx, 
            itr->x, itr->y, itr->pose, itr->steer);
   }

   delete points;
   delete unvisited;
}

// publisher for publishing movement commands
ros::Publisher control_pub;

bool active = false;
bool path_valid = false;
loc goal;

void goalCallback(const goal_list::Goal::ConstPtr & msg) {
   if( msg->valid ) {
      goal.x = msg->loc.col;
      goal.y = msg->loc.row;
      active = true;
      path_valid = false;
      ROS_INFO("Path following activated");
   } else {
      active = false;
      ROS_INFO("Path following deactivated");
   }
}

// the last location we were at.
//  used as the center point for our local map
loc last_loc;
   
void positionCallback(const nav_msgs::Odometry::ConstPtr & msg) {
   loc here;
   here.x = msg->pose.pose.position.x;
   here.y = msg->pose.pose.position.y;
   here.pose = (M_PI/2) - msg->pose.pose.orientation.x;
   last_loc = here;
   if( active ) {
      hardware_interface::Control c;
      // if our path is invalid due to a new goal or such, re-plan
      /*
      if( !path_valid ) {
         ROS_INFO("Path invalid; re-planning");
         path_valid = true;
         plan_path(here, goal);
      }

      // find the closest point on our path to the robot
      list<loc>::iterator itr = path->begin();
      double close_d = dist((*itr), here);
      list<loc>::iterator close_i = itr;
      for( itr++ ; itr != path->end(); itr++ ) {
         double d = dist((*itr), here);
         if( d < close_d ) {
            close_d = d;
            close_i = itr;
         }
      }

      // if we're too far off the path, re-plan
      if( close_d > CLOSE_LEN ) {
         ROS_INFO("Too far off path(%lf); re-planning", close_d);
         // stop the robot while we re-plan
         c.speed = 0;
         c.steer = 0;
         control_pub.publish(c);

         // re-pan
         plan_path(here, goal);
         close_i = path->begin();
      }
      // get directions to get to the next point
      ROS_INFO("Closest point %d, steer %d", close_i->idx, close_i->steer);
      c.steer = close_i->steer;
      */

      ROS_INFO("Current angle: %lf", here.pose);

      // FIXME: complete hack to get by for the Sparkfun AVC
      double theta = atan2(goal.y - here.y, goal.x - here.x);
      ROS_INFO("Angle to goal: %lf", theta);

      double x, y;

      bool collide = true;
      double target = theta;
      double traverse_dist = min(dist(here, goal), 50.0);
      //ROS_INFO("Traverse distance %lf", traverse_dist);

      int i=1;

      //print_map();

      // look for a target angle that doesn't collide with anything
      while( collide && i < 20) {
         collide = false;
         if( i & 1 ) {
            target = theta + (M_PI/16)*(i/2);
         } else {
            target = theta - (M_PI/16)*(i/2);
         }
         // check across our map to see if our path is clear
         for( double dist = 0; dist <= traverse_dist; dist += 0.5 ) {
            x = here.x + dist*cos(target);
            y = here.y + dist*sin(target);
            if( map_get(x, y) ) {
               collide = true;
               break;
            }
         }
         i++;
      }

      ROS_INFO("Target angle:  %lf", target);

      double diff = target - here.pose;
      while( diff > M_PI  ) diff -= 2*M_PI;
      while( diff < -M_PI ) diff += 2*M_PI;

      ROS_INFO("Angle difference: %lf", diff);

      if( diff > M_PI/2 ) {
         c.steer = -100;
      } else if( diff < -M_PI/2 ) {
         c.steer = 100;
      } else {
         c.steer = (int)(-diff*60.0);
      }
      ROS_INFO("steer: %d", c.steer);

      c.speed = 60;
      control_pub.publish(c);
   } else {
      hardware_interface::Control c;
      c.steer = 0;
      c.speed = 0;
      control_pub.publish(c);
   }
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr & msg) {
   map_center_x = last_loc.x;
   map_center_y = last_loc.y;

   double theta_base = last_loc.pose;

   double theta = theta_base + msg->angle_min;
   double x;
   double y;

   /*
   for( int i=0; i<101; i++ ) {
      for( int j=0; j<101; j++ ) {
         map_data[i][j] = 0;
      }
   }
   */
   // memset ought to ba faster
   memset(map_data, 0, 101*101*sizeof(int));

   for( unsigned int i=0; i<msg->ranges.size(); i++, 
         theta += msg->angle_increment ) {
      // 0 means max range... I think
      if( msg->ranges[i] != 0.0 ) {
         x = map_center_x + msg->ranges[i]*cos(theta)*10.0; // convert to 10 x m
         y = map_center_y + msg->ranges[i]*sin(theta)*10.0;
         map_set(x, y, 1);
      }
   }

   //print_map();

   // we hope we aren't sitting on an obstacle
   map_data[50][50] = 0;

   // grow obstacles by radius of robot; makes collision-testing easier
   // order: O(n^2 * 12)
   for( int r=1; r<4; r++ ) {
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

   print_map();

   /*
   if( active ) {
      // test path for collisions and re-plan if collision iminent
      bool collide = false;
      for( list<loc>::iterator itr = path->begin(); itr != path->end(); itr++ ) {
         if( test_collision(*itr) ) {
            collide = true;
            break;
         }
      }
      if( collide ) {
         ROS_INFO("Collision iminenet, re-planning");
         hardware_interface::Control c;

         // stop the robot while we re-plan
         c.speed = 0;
         c.steer = 0;
         //control_pub.publish(c);

         // mark path as invalid
         path_valid = false;
         // we'll start moving again when we get a location update
      }
   }
   */

   return;
}

int main(int argc, char ** argv) {
   path = new list<loc>();

   // set map to empty
   for( int i=0; i<101; i++ ) {
      for( int j=0; j<101; j++ ) {
         map_data[i][j] = 0;
      }
   }

   ros::init(argc, argv, "path_planner");

   ros::NodeHandle n;

   // subscribe to our location and current goal
   ros::Subscriber pos_sub = n.subscribe("position", 2, positionCallback);
   ros::Subscriber goal_sub = n.subscribe("current_goal", 2, goalCallback);
   ros::Subscriber laser_sub = n.subscribe("scan", 2, laserCallback);

   control_pub = n.advertise<hardware_interface::Control>("control", 10);

   ros::Rate loop(10.0);

   ROS_INFO("Path planner ready");

   ros::spin();
   /*
   while( ros::ok() ) {
      ros::spinOnce();
      loop.sleep();
   }
   */
}
