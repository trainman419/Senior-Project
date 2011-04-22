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
#define ARC_LEN 2.0

// distance where we decide we're too far off-path
#define CLOSE_LEN 4.0

// minimum turning radius
#define MIN_RADIUS 20.0

// how close we want to get to our goal before we're "there"
#define GOAL_ERR 3.0

// maximum number of iterations to look for a path
#define MAX_ITER 10000

// types, to make life easier
struct arc {
   double radius; // radius; 0: straight
   double length; // arc length
   int steer;
};

struct loc {
   // x, y, pose: the position and direction of the robot
   double x;
   double y;
   double pose;

   // for path-finding: the node previous to this one
   int prev;
   // for path-finding: this node's index
   int idx;
   // for path-finding: whether we have visited this location or not
   //bool visited;
   // the arc we followed to get to this point
   arc path;
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

/*
   struct corner {
      int x;
      int y;
   };
   int comp(const void * c, const void * d) {
      const corner * a = (const corner*)c;
      const corner * b = (const corner*)d;
      if( a->x < b->x ) return -1;
      if( a->x > b->x ) return 1;
      if( a->y < b->y ) return -1;
      if( a->y > b->y ) return 1;
      return 0;
   }
   */

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
   // robot dimensions: width: 4; length: 7
   // laser 2.5 back from front, at logical center of robot
   
   /*
   // Base corners
   //  3-------0
   //  |   ->  |
   //  2-------1
   double c_x[] = {2.5, 2.5, -4.5, -4.5};
   double c_y[] = {2.0, -2.0, 2.0, -2.0};

   // translate corners onto fixed coordinate system
   corner c[4];
   for( int i=0; i<4; i++ ) {
      c[i].x = round(here.x + c_x[i]*cos(here.pose) - c_y[i]*sin(here.pose));
      c[i].y = round(here.y + c_y[i]*cos(here.pose) + c_x[i]*sin(here.pose));
   }

   qsort(c, 4, sizeof(corner), comp);
   */

   /*
   cout << "Corners: " << endl;
   for( int i=0; i<4; i++ ) {
      cout << "(" << c[i].x << ", " << c[i].y << ")" << endl;
   }
   */

   /*
   double i, j;

   for( double x = -4.5; x <= 2.5; x+= 1.0 ) {
      for( double y = -2.0; y <= 2.0; y += 1.0 ) {
         i = here.x + x*cos(here.pose) - y*sin(here.pose);
         j = here.y + y*cos(here.pose) + x*sin(here.pose);
         if( map_get(i, j) ) return true;
      }
   }
   */
   return map_get(here.x, here.y) != 0;

   // hack; for now, ignore obstacles and just plan a path
   //return false;
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
   points->push_back(here);
   (*unvisited)[dist(here, end)] = 0;

   //double radii[] = {MIN_RADIUS, MIN_RADIUS*2, MIN_RADIUS*4, MIN_RADIUS*8, 0, 
   //   -MIN_RADIUS*8, -MIN_RADIUS*4, -MIN_RADIUS*2, -MIN_RADIUS};
   //int steer[] = {-100, -80, -60, -40, -20, 0, 20, 40, 60, 80, 100};
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

      // FIXME: only drive forward
      /*
      printf("Here: %d (%8.2lf, %8.2lf, %6.4lf), (%4.2lf, %4.2lf) "
            "remaining: %8.2lf\n",
            here.idx, here.x, here.y, here.pose, here.path.radius, 
            here.path.length, dist(here, end));
            */

      // generate points to visit
      //for( int i=0; i<(sizeof(radii)/sizeof(double)); i++ ) {
      //for( unsigned int i=0; i<(sizeof(steer)/sizeof(int)); i++ ) {
      for( unsigned int i=0; i<15; i++ ) {
         double d = ARC_LEN;  // distance to travel
         double r = 0;

         double dx, dy, dt;

         //  convert to unit-circle angle
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
         //n.visited = false;
         n.prev = here.idx;
         n.path.radius = r;
         n.path.length = d;
         n.path.steer = steer[i];


         while( n.pose < -M_PI ) n.pose += M_PI*2;
         while( n.pose >  M_PI ) n.pose -= M_PI*2;

         if( !test_collision(n) ) {
            n.idx = points->size();
            points->push_back(n);

            double len = dist(n, end);
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
            itr->x, itr->y, itr->pose, itr->path.steer);
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
      // if our path is invalid due to a new goal or such, re-plan
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
      hardware_interface::Control c;

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
      ROS_INFO("Closest point %d, steer %d", close_i->idx, close_i->path.steer);
      c.speed = 20;
      c.steer = close_i->path.steer;
      control_pub.publish(c);
      /*for( int i=0; i<2; i++ ) {
         if( close_i != path->end() ) {
            close_i++;
         }
      }*/
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

   for( int i=0; i<101; i++ ) {
      for( int j=0; j<101; j++ ) {
         map_data[i][j] = 0;
      }
   }

   for( unsigned int i=0; i<msg->ranges.size(); i++, 
         theta += msg->angle_increment ) {
      // 0 means max range... I think
      if( msg->ranges[1] != 0.0 ) {
         x = map_center_x + msg->ranges[i]*cos(theta)*10.0; // convert to 10 x m
         y = map_center_y + msg->ranges[i]*sin(theta)*10.0;
         map_set(x, y, 1);
      }
   }

   // assert that we aren't sitting on an obstacle
   map_data[50][50] = 0;
   assert(map_data[50][50] != 1);

   // TODO: test path for collisions and re-plan if collision iminent
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
      control_pub.publish(c);

      // mark path as invalid
      path_valid = false;
      // we'll start moving again when we get a location update
   }

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
   ros::Subscriber pos_sub = n.subscribe("position", 10, positionCallback);
   ros::Subscriber goal_sub = n.subscribe("current_goal", 10, goalCallback);
   ros::Subscriber laser_sub = n.subscribe("scan", 1, laserCallback);

   control_pub = n.advertise<hardware_interface::Control>("control", 10);

   ROS_INFO("Path planner ready");

   ros::spin();
}
