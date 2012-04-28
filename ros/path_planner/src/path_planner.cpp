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
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>

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
// how close we are before we switch to cone mode (m)
#define CONE_DIST 6.0

// map resolution, in meters per pixel
#define MAP_RES 0.10
// map size, in cells
#define MAP_SIZE 5000

// speed for path traversal (m/s)
#define MAX_SPEED 1.5
#define MIN_SPEED 0.1
#define MAX_TRAVERSE 4.0
#define MAX_ACCEL 0.3

// planner timeouts
#define BACKUP_TIME 6.0
#define STUCK_TIMEOUT 2.0

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
// fixed dimensions: 100m by 100m, centered about the associated center point
// FIXME: replace this with calls to the global_map and SLAM
typedef int8_t map_type;
map_type * map_data;

// get the value of the local obstacle map at (x, y)
//  return 0 for any point not within the obstacle map
inline map_type map_get(double x, double y) {
   int i = round(x/MAP_RES) + MAP_SIZE/2;
   int j = round(y/MAP_RES) + MAP_SIZE/2;
   if( i >= 0 && i < MAP_SIZE && j >= 0 && j < MAP_SIZE ) {
      return map_data[(i * MAP_SIZE) + j];
   } else {
      return 0;
   }
}

// get the value of the local obstacle map at (x, y)
//  return 0 for any point not within the obstacle map
inline void map_set(double x, double y, map_type v) {
   int i = round(x/MAP_RES) + MAP_SIZE/2;
   int j = round(y/MAP_RES) + MAP_SIZE/2;
   if( i >= 0 && i < MAP_SIZE && j >= 0 && j < MAP_SIZE ) {
      map_data[(i * MAP_SIZE) + j] = v;
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
         loc h;
         h.x = r * cos(theta + dist / r) + center_x;
         h.y = r * sin(theta + dist / r) + center_y;
         if( test_collision(h) ) {
            //ROS_WARN("Obstacle at %lf", dist);
            return false;
         }
      }
   } else {
      // degenerate case; traverse a line
      for( double dist = 0; dist < l; dist += MAP_RES/2.0 ) {
         loc h;
         h.x = start.x + dist*cos(start.pose);
         h.y = start.y + dist*sin(start.pose);
         if( test_collision(h) ) {
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
         //ROS_INFO("Map at %lf, %lf: %d", x, y, map_get(x, y));
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

enum pstate {
   BACKING, FORWARD, CONE
};

pstate planner_state;

ros::Time planner_timeout;

visualization_msgs::Marker cones;

bool bump = false;


/* plan a path from start to end
 *  TODO: find a clear path all the way to the edge of the map or the goal, 
 *   whichever is closer
 *  TODO: try different arc lengths
 *  TODO: add state backing up support
 */
path plan_path(loc start, loc end) {
   /*
   ROS_INFO("Searching for path from (% 5.2lf, % 5.2lf) to (% 5.2lf, % 5.2lf)",
         start.x, start.y, end.x, end.y);
         */
   path p;
   double d = dist(start, end);
   /*
   if( d < CONE_DIST && planner_state == FORWARD ) {
      planner_state = CONE;
      planner_timeout = ros::Time::now();
      ROS_INFO("Starting cone tracking");
   }
   */

   switch(planner_state) {
      case BACKING:
         p.speed = -2.0 * MIN_SPEED;
         p.radius = 0;
         if( (ros::Time::now() - planner_timeout).toSec() > BACKUP_TIME ) {
            planner_state = FORWARD;
            planner_timeout.sec = 0;
         }
         break;
      case CONE:
         {
            // find nearest cone
            if( cones.points.size() > 0 ) {
               geometry_msgs::Point cone = cones.points.front();
               double cone_d = hypot(cone.x - start.x, cone.y - start.y);
               BOOST_FOREACH(geometry_msgs::Point p, cones.points) {
                  double d = hypot(p.x - start.x, p.y - start.y);
                  if( d < cone_d ) {
                     cone = p;
                     cone_d = d;
                  }
               }
               p.speed = MIN_SPEED * 4.0;
               p.radius = 0;
               double cone_angle = atan2(cone.y - start.y, cone.x - start.x);
               double turn_angle = cone_angle - start.pose;
               // normalize turn angle
               while( turn_angle > M_PI ) turn_angle -= 2*M_PI;
               while( turn_angle < -M_PI ) turn_angle += 2 * M_PI;

               ROS_INFO("Angle to cone %lf", turn_angle);
               if( turn_angle > 0.1 ) {
                  p.radius = MIN_RADIUS;
                  ROS_INFO("Cone left");
               }
               if( turn_angle < -0.1 ) {
                  p.radius = -MIN_RADIUS;
                  ROS_INFO("Cone right");
               }
            } else {
               ROS_INFO("No cones");
               // if we don't see any cones, drive in circles
               p.speed = MIN_SPEED * 4.0;
               p.radius = MIN_RADIUS;
            }
            
            // if we hit the cone, back up and keep going
            if( bump ) {
               planner_timeout = ros::Time::now();
               planner_state = BACKING;
               p.speed = 0;
               p.radius = 0;
               ROS_INFO("Cone hit");
            }
            if( planner_timeout + ros::Duration(60.0) < ros::Time::now() ) {
               planner_state = FORWARD;
               p.speed = 0;
               p.radius = 0;
               ROS_INFO("Cone tracking timed out");
            }
         }
         break;
      case FORWARD:
         double theta = atan2(end.y - start.y, end.x - start.x);
         //ROS_INFO("Angle to goal: %lf", theta);

         double traverse_dist = min(d, MAX_TRAVERSE);
         double speed = min(MAX_SPEED, 
               MAX_SPEED * (2.0 * traverse_dist / MAX_TRAVERSE));
         if( speed < MIN_SPEED) speed = MIN_SPEED;
         //ROS_INFO("Traverse distance %lf, speed %lf", traverse_dist, speed);

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

         // if our turn radius is below our minimum radius, go straight
         if( fabs(radius) < MIN_RADIUS ) {
            //ROS_INFO("Tangent arc radius too small; looping around. %lf", radius);
            radius = 0;
            // we should go forward by our minimum radius, and then loop around
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
               // TODO: try various traverse distances
               //  followed by a straight path to the edge of the map
               double d = min(traverse_dist, MIN_RADIUS * i * M_PI / 2);
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
               if( planner_timeout.sec != 0 ) {
                  if( (ros::Time::now() -  planner_timeout).toSec() > 
                        STUCK_TIMEOUT ) {
                     planner_state = BACKING;
                     planner_timeout = ros::Time::now();
                     ROS_WARN("Robot stuck; backing up");
                  }
               } else {
                  planner_timeout = ros::Time::now();
               }
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
               arc_len = fabs(best_r * M_PI / 2);
               if( best_r == 0.0 ) arc_len = traverse_dist;
               speed = min(MAX_SPEED, MAX_SPEED * (2.0 * arc_len / MAX_TRAVERSE));
               nav_msgs::Path p = arcToPath(start, best_r, 
                     min(traverse_dist, arc_len));
               path_pub.publish(p);
               // reset backup timer
               planner_timeout.sec = 0;
            }
         } else {
            nav_msgs::Path p = arcToPath(start, radius, arc_len);
            path_pub.publish(p);
            // reset backup timer
            planner_timeout.sec = 0;
         }
         ROS_INFO("Traverse distance %lf, speed %lf", traverse_dist, speed);
         p.radius = radius;
         p.speed = speed;
         break;
   }

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

#define LOCAL_MAP_SIZE 150
#define LASER_OFFSET 0.26

void laserCallback(const sensor_msgs::LaserScan::ConstPtr & msg) {
   //map_center_x = last_loc.x;
   //map_center_y = last_loc.y;
   loc here = last_loc;

   double theta_base = last_loc.pose;

   double theta = theta_base + msg->angle_min;
   double x;
   double y;

   double offset_x = modf(here.x/MAP_RES, &x)*MAP_RES; // don't care about x
   double offset_y = modf(here.y/MAP_RES, &y)*MAP_RES; // don't care about y

   // manual laser transform. I'm a horrible person
   offset_x += LASER_OFFSET * cos(theta_base);
   offset_y += LASER_OFFSET * sin(theta_base);

   map_type * local_map = (map_type*)malloc(LOCAL_MAP_SIZE*LOCAL_MAP_SIZE*
         sizeof(map_type));
   memset(local_map, 0, LOCAL_MAP_SIZE*LOCAL_MAP_SIZE*sizeof(map_type));
   int j, k;
   
   // build a local map and merge it with the global map

   // for each laser scan point, raytrace
   for( unsigned int i=0; i<msg->ranges.size(); i++, 
         theta += msg->angle_increment ) {
      double d;
      double r = msg->ranges[i];
      int status = 1;
      if( r < msg->range_min ) {
         // pull status codes out of laser data according to SCIP1.1
         if( r == 0.0 ) {
            r = 22.0; // raytrace out to 22m
         } else if( 0.0055 < r && r < 0.0065 ) {
            r = 5.7;
         } else if( 0.0155 < r && r < 0.0165 ) {
            r = 5.0;
         } else {
            status = 0;
         }
      }
      if( status ) {
         for( d=0; d<r; d += MAP_RES/2.0 ) {
            x = offset_x + d*cos(theta);
            y = offset_y + d*sin(theta);

            j = round(x/MAP_RES) + LOCAL_MAP_SIZE/2;
            k = round(y/MAP_RES) + LOCAL_MAP_SIZE/2;
            if( j > 0 && k > 0 && j < LOCAL_MAP_SIZE && k < LOCAL_MAP_SIZE ) {
               local_map[j*LOCAL_MAP_SIZE + k] = -1;
            } else {
               break; // if we step outside the local map bounds, we're done
            }
         }
      }
   }
   
   // mark obstacles
   theta = theta_base + msg->angle_min;
   for( unsigned int i=0; i<msg->ranges.size(); i++, 
         theta += msg->angle_increment ) {
      if( msg->ranges[i] > msg->range_min ) {
         x = offset_x + msg->ranges[i]*cos(theta);
         y = offset_y + msg->ranges[i]*sin(theta);

         j = round(x/MAP_RES) + LOCAL_MAP_SIZE/2;
         k = round(y/MAP_RES) + LOCAL_MAP_SIZE/2;
         if( j > 0 && k > 0 && j < LOCAL_MAP_SIZE && k < LOCAL_MAP_SIZE ) {
            local_map[j*LOCAL_MAP_SIZE + k] = 1;
         }
      }
   }

   // grow obstacles by radius of robot; makes collision-testing easier
   // order: O(n^2 * 12)
   for( int r=1; r<(0.4/MAP_RES); r++ ) {
      for( int i=0; i<LOCAL_MAP_SIZE; i++ ) {
         for( int j=0; j<LOCAL_MAP_SIZE; j++ ) {
            if( local_map[i*LOCAL_MAP_SIZE + j] <= 0 ) {
               if( i > 0   && local_map[(i-1)*LOCAL_MAP_SIZE + j  ] == r ) 
                  local_map[i*LOCAL_MAP_SIZE + j] = r+1;
               if( j > 0   && local_map[i*LOCAL_MAP_SIZE + j-1] == r )
                  local_map[i*LOCAL_MAP_SIZE + j] = r+1;
               if( i < LOCAL_MAP_SIZE && 
                     local_map[(i+1)*LOCAL_MAP_SIZE + j  ] == r )
                  local_map[i*LOCAL_MAP_SIZE + j] = r+1;
               if( j < LOCAL_MAP_SIZE && 
                     local_map[i*LOCAL_MAP_SIZE + j+1] == r ) 
                  local_map[i*LOCAL_MAP_SIZE + j] = r+1;
            }
         }
      }
   }

   // merge into global map
   offset_x = round(here.x/MAP_RES)*MAP_RES;
   offset_y = round(here.y/MAP_RES)*MAP_RES;
   for( int i=0; i<LOCAL_MAP_SIZE; i++ ) {
      for( int j=0; j<LOCAL_MAP_SIZE; j++ ) {
         x = (i - LOCAL_MAP_SIZE/2) * MAP_RES + offset_x;
         y = (j - LOCAL_MAP_SIZE/2) * MAP_RES + offset_y;
         map_type tmp = 0;
         tmp += local_map[i*LOCAL_MAP_SIZE + j];
         if( tmp > 0 ) tmp = 2; // flatten obstacle radius
         tmp += map_get(x, y);
         if( tmp > 4 ) tmp = 4;
         if( tmp < 0 ) tmp = 0;
         map_set(x, y, tmp);
      }
   }

   // clear out base footprint
   theta = here.pose;
   for( double bx = -0.16; bx <= 0.16; bx += MAP_RES/2.0 ) {
      for( double by = -0.17; by < 0.45; by += MAP_RES/2.0 ) {
         x = bx*cos(theta) + here.x;
         y = by*sin(theta) + here.y;
         map_set(x, y, 0);
      }
   }

   free(local_map);

   /*
   static int div = 0;
   ++div;
   if( div % 20 == 0 ) {
      // publish map
      nav_msgs::OccupancyGrid map;
      map.header = msg->header;
      map.header.frame_id = "odom";
      map.info.resolution = MAP_RES;
      map.info.width = MAP_SIZE;
      map.info.height = MAP_SIZE; 
      map.info.origin.position.x = - (MAP_SIZE * MAP_RES) / 2.0;
      map.info.origin.position.y = - (MAP_SIZE * MAP_RES) / 2.0;
      map.info.origin.orientation.w = 1.0;
      for( int i=0; i<MAP_SIZE; i++ ) {
         for( int j=0; j<MAP_SIZE; j++ ) {
            map.data.push_back(map_data[(j * MAP_SIZE) + i]);
         }
      }
      map_pub.publish(map);
   }
   */
   return;
}

void bumpCb(const std_msgs::Bool::ConstPtr & msg ) {
   bump = msg->data;
}

void conesCb(const visualization_msgs::Marker::ConstPtr & msg ) {
   cones = *msg;
}

int main(int argc, char ** argv) {
   map_data = (map_type*)malloc(MAP_SIZE * MAP_SIZE * sizeof(map_type));
   // set map to empty
   for( int i=0; i<MAP_SIZE; i++ ) {
      for( int j=0; j<MAP_SIZE; j++ ) {
         map_data[i*MAP_SIZE + j] = 0;
      }
   }

   ros::init(argc, argv, "path_planner");

   ros::NodeHandle n;

   // subscribe to our location and current goal
   ros::Subscriber odom_sub = n.subscribe("odom", 2, odomCallback);
   ros::Subscriber goal_sub = n.subscribe("current_goal", 2, goalCallback);
   ros::Subscriber laser_sub = n.subscribe("scan", 2, laserCallback);
   ros::Subscriber bump_sub = n.subscribe("bump", 2, bumpCb);
   ros::Subscriber cones_sub = n.subscribe("cone_markers", 2, conesCb);

   cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
   map_pub = n.advertise<nav_msgs::OccupancyGrid>("map", 1);
   path_pub = n.advertise<nav_msgs::Path>("path", 10);

   ROS_INFO("Path planner ready");

   ros::spin();
}
