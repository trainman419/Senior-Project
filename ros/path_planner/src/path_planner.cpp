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

#include <set>
#include <list>
#include <map>
#include <vector>

#include <stdio.h>

#include <ros/ros.h>

#include <global_map/Location.h>
#include <path_planner/Move.h>
#include <hardware_interface/Control.h>
#include <goal_list/Goal.h>
#include <nav_msgs/Odometry.h>

using namespace std;

// length for stock line segments
#define ARC_LEN 2.0

// minimum turning radius
#define MIN_RADIUS 20.0

// how close we want to get to our goal before we're "there"
#define GOAL_ERR 3.0

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
   bool visited;
   // the arc we followed to get to this point
   arc path;
};

// our current path
list<loc> * path;

// test if we have a collision at a particular point
bool test_collision(loc here) {
   // hack; for now, ignore obstacles and just plan a path
   return false;
}

#define dist(a, b) hypot(a.x - b.x, a.y - b.y)

// plan a path from start to end and update the current path
// implemented as A*
void plan_path(loc start, loc end) {
   loc here = start;

   ROS_INFO("Searching for path over %f", dist(start, end));
   int iter = 0;

   // identify positions where we've been by an int and store them
   vector<loc> * points = new vector<loc>();
   // sorted list of points, in order from closest to furthest
   //  we don't index this, we just use it for the fact that it's sorted
   //  and we pull the first element off the list each time
   map<double, int> * unvisited = new map<double, int>();

   here.idx = 0;
   points->push_back(here);

   //double radii[] = {MIN_RADIUS, MIN_RADIUS*2, MIN_RADIUS*4, MIN_RADIUS*8, 0, 
   //   -MIN_RADIUS*8, -MIN_RADIUS*4, -MIN_RADIUS*2, -MIN_RADIUS};
   int steer[] = {-100, -80, -60, -40, -20, 0, 20, 40, 60, 80, 100};

   while( dist(here, end) > GOAL_ERR && iter < 100 ) {
      iter++;
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
      for( unsigned int i=0; i<11; i++ ) {
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
               theta_c1 = theta + M_PI;
            } else {
               // turning left
               theta_c1 = theta - M_PI;
            }
            theta_c2 = theta_c1 - dt;

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
         n.visited = false;
         n.prev = here.idx;
         n.path.radius = r;
         n.path.length = d;
         n.path.steer = steer[i];
         n.idx = points->size();

         while( n.pose < -M_PI ) n.pose += M_PI*2;
         while( n.pose >  M_PI ) n.pose -= M_PI*2;
         points->push_back(n);

         double len = dist(n, end);
         (*unvisited)[len] = n.idx;
      }

      // visit the point nearest the goal
      map<double, int>::iterator next = unvisited->begin();
      here = points->at(next->second);
      unvisited->erase(next);
   }

   ROS_INFO("Found path in %d iterations!", iter);

   path->clear();

   // walk backwards and build path
   while( here.idx != 0 ) {
      path->push_front(here);
      here = points->at(here.prev);
   }

   ROS_INFO("Path with %d points", path->size());

   delete points;
   delete unvisited;
}

// publisher for publishing movement commands
ros::Publisher move_pub;
ros::Publisher control_pub;

bool active = false;
bool path_valid = false;
loc goal;

void goalCallback(const goal_list::Goal::ConstPtr & msg) {
   // TODO: do stuff when we update the goal
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

// TODO: update this to take one of the stock messages that better represents
// position, with error estimates
void positionCallback(const nav_msgs::Odometry::ConstPtr & msg) {
   // TODO: do stuff when our position changes
   loc here;
   here.x = msg->pose.pose.position.x;
   here.y = msg->pose.pose.position.y;
   here.pose = (M_PI/2) - msg->pose.pose.orientation.x;
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
      if( close_d > ARC_LEN*2 ) {
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
      ROS_INFO("Closest point %d", close_i->idx);
      c.speed = 15;
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

int main(int argc, char ** argv) {
   path = new list<loc>();

   ros::init(argc, argv, "path_planner");

   ros::NodeHandle n;

   // subscribe to our location and current goal
   ros::Subscriber pos_sub = n.subscribe("position", 10, positionCallback);
   ros::Subscriber goal_sub = n.subscribe("current_goal", 10, goalCallback);

   move_pub = n.advertise<path_planner::Move>("move_commands", 10);
   control_pub = n.advertise<hardware_interface::Control>("control", 10);

   ros::spin();
}
