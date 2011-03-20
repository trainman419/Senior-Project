/* control_server.cpp
 *
 * A control server for the robot; it talks to clients over TCP, exchanging
 * position and high-level control information, as well as sensor readings
 * and low-level motor control commands.
 *
 * To ROS, the control server should subscribe to laser scan messages, and the
 * current GPS position, and should publish GPS goals/waypoints and low-level
 * motor control commands.
 *
 * Author: Austin Hendrix
 */

/* TODO: this node should publish itself via avahi, rather than rely on a 
 *  statically configured service.
 */

// C includes
#include <sys/types.h>
#include <sys/socket.h>
#include <errno.h>

// C++ includes
#include <set>

// ROS includes
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#define SERVER_PORT 2082

int main(int argc, char ** argv) {

   ros::init(argc, argv, "control_server");
}
