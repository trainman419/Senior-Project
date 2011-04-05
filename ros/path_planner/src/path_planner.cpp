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

#include <ros/ros.h>

#include <global_map/Location.h>
#include <path_planner/Move.h>

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
};

struct loc {
   // x, y, pose: the position and direction of the robot
   double x;
   double y;
   double pose;

   // for path-finding: the node previous to this one
   int prev;
   // for path-finding: whether we have visited this location or not
   bool visited;
};

// our current path
list<arc> path;

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

   // identify positions where we've been by an int and store them
   vector<loc> * points = new vector<loc>();
   // sorted list of points, in order from closest to furthest
   //  we don't index this, we just use it for the fact that it's sorted
   //  and we pull the first element off the list each time
   map<double, int> * unvisited = new map<double, int>();

   double radii[] = {MIN_RADIUS, MIN_RADIUS*2, MIN_RADIUS*4, MIN_RADIUS*8, 0, 
      -MIN_RADIUS*8, -MIN_RADIUS*4, -MIN_RADIUS*2, -MIN_RADIUS};

   while( dist(here, end) > GOAL_ERR ) {
      // FIXME: only drive forward
      for( int i=0; i<(sizeof(radii)/sizeof(double)); i++ ) {

      }
   }
}

// publisher for publishing movement commands
ros::Publisher move_pub;

void goalCallback(const global_map::Location::ConstPtr & msg) {
   // TODO: do stuff when we update the goal
}

// TODO: update this to take one of the stock messages that better represents
// position, with error estimates
void positionCallback(const global_map::Location::ConstPtr & msg) {
   // TODO: do stuff when our position changes
}

int main(int argc, char ** argv) {
   ros::init(argc, argv, "path_planner");

   ros::NodeHandle n;

   // subscribe to our location and current goal
   ros::Subscriber pos_sub = n.subscribe("position", 10, positionCallback);
   ros::Subscriber goal_sub = n.subscribe("current_goal", 10, goalCallback);

   move_pub = n.advertise<path_planner::Move>("move_commands", 10);
   
   ros::spin();
}
