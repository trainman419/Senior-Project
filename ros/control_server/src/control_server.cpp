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
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <errno.h>

// C++ includes
#include <set>
#include <map>

// ROS includes
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#define SERVER_PORT 2082
#define BUF_SZ 1024

using namespace std;

set<int> clients;

void laserCallback(const sensor_msgs::LaserScan::ConstPtr & msg) {
   set<int>::iterator itr;
   // set up output buffer
   uint32_t size = msg->ranges.size()*4;
   char * buffer = (char*)malloc(size + 5);
   buffer[0] = 'L'; // type byte: laser scan
   *((uint32_t*)(buffer+1)) = htonl(size); // size

   uint32_t * outbuf = (uint32_t*)(buffer+5);

   for( unsigned int i=0; i<msg->ranges.size(); i++ ) {
      outbuf[i] = htonl((uint32_t)msg->ranges[i]);
   }

   for(itr = clients.begin(); itr != clients.end(); itr++) {
      if(write(*itr, buffer, size+5) < 0) {
         ROS_INFO("Client %d disconnected", *itr);
         clients.erase(itr);
      }
   }

   free(buffer);
}

int main(int argc, char ** argv) {

   ros::init(argc, argv, "control_server");

   // set up our socket and start listening for clients
   int sock = socket(AF_INET, SOCK_STREAM, 0);
   if( sock < 0 ) {
      ROS_ERROR("Failed to open socket: %s", strerror(errno));
      return -1;
   }

   // set up our socket as non-blocking
   int flags;
   if( (flags = fcntl(sock, F_GETFL, 0)) < 0 ) {
      ROS_ERROR("Faile to get socket flags: %s", strerror(errno));
      close(sock);
      return -1;
   }
   flags |= O_NONBLOCK;
   if( fcntl(sock, F_SETFL, flags) < 0 ) {
      ROS_ERROR("Faile to set socket flags: %s", strerror(errno));
      close(sock);
      return -1;
   }

   // bind our socket to an address
   struct sockaddr_in addr;
   addr.sin_family = AF_INET;
   addr.sin_port = ntohs(SERVER_PORT);
   addr.sin_addr.s_addr = 0;
   if( bind(sock, (struct sockaddr *)&addr, sizeof(struct sockaddr_in)) < 0 ) {
      ROS_ERROR("Failed to bind socket: %s", strerror(errno));
      close(sock);
      return -1;
   }

   // listen on our socket
   if( listen(sock, 5) < 0 ) {
      ROS_ERROR("Failed to listen on socket: %s", strerror(errno));
      close(sock);
      return -1;
   }


   ROS_INFO("control_server now listening for connections");
   ros::NodeHandle n;

   // subscribe to laser scan messages
   ros::Subscriber laser_sub = n.subscribe("scan", 5, laserCallback);

   // per-client input buffers
   map<int, pair<int, char*> > buffers;
   
   // main loop
   ros::Rate loop_rate(10);
   while( ros::ok() ) {
      int client;
      do {
         client = accept4(sock, NULL, 0, SOCK_NONBLOCK);
         if( client >= 0 ) {
            clients.insert(client);
            buffers[client] = pair<int, char*>(0, (char*)malloc(BUF_SZ));
            ROS_INFO("New client: %d", client);
         }
      } while( client >= 0);

      for( set<int>::iterator itr = clients.begin(); itr != clients.end();
            itr++ ) {
         client = *itr;
         int sz = buffers[client].first;
         char * buf = buffers[client].second;
         sz += read(client, buf + sz, BUF_SZ - sz);
         buffers[client].first = sz;

         if( sz > 5 ) {
            uint32_t size = ntohl(*(uint32_t*)(buf+1));
            // if we have a whole packet in our buffer
            if( size >= sz - 5 ) {
            }
         }
      }

      ros::spinOnce();
      loop_rate.sleep();
   }

   // clean-up
   close(sock);
   return 0;
}
