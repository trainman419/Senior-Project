/* a ROS node to act as a bridge between the serial port to the robot hardware
 * and all of the internal ROS messages that will be flying around.
 *
 * Author: Austin Hendrix
 */

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

char laser_data[256];
int laser_ready;

// callback on laser scan received.
void laserCallback(const sensor_msgs::LaserScan::ConstPtr & msg) {
   for(int i=0; i<msg->ranges.size(); i+= 2 ) {
      if( i < 512 ) {
         // average adjacent data points
         float data = (msg->ranges[i] + msg->ranges[i+1]) / 2;
         // scale to fit into a byte. 250 = 5.0m
         data = data * 50;
         laser_data[i/2] = (char)data;
      }
   }

   laser_ready = 1;
}

int main(int argc, char ** argv) {
   laser_ready = 0;

   ros::init(argc, argv, "hardware_interface");

   ros::NodeHandle n;

   // TODO: set up publishers here. I don't think we have any yet
   //  will probably want publishers for battery, sonar, wheel and bump data

   // deal with GPS data some other way; probably write it to a fifo and have
   //  gpsd pick it up. TODO
   //
   
   // TODO: open serial port and set up here
   // I'm going to hardcode the port and settings because this is hardware-
   // specific anyway
   // open serial port
   int serial = open("/dev/ttyS1", O_RDWR | O_NOCTTY);
   // TODO: set baud rate and parity
   if( serial < 0 ) {
      perror("Failed to open /dev/ttyS1");
      // die. ungracefully.
      return -1;
   }

   struct termios tio;
   tcgetattr(serial, &tio);

   cfsetospeed(&tio, B115200);
   cfsetispeed(&tio, B115200);
   
   tcsetattr(serial, TCSANOW, &tio);

   ros::Subscriber sub = n.subscribe("scan", 5, laserCallback);

   while( ros::ok() ) {
      // TODO: read serial port for incoming data and do something with it
      
      // TODO: write pending data to serial port
      if( laser_ready ) {
         write(serial, "L", 1);
         write(serial, laser_data, 256);
         write(serial, "\r", 1);
         laser_ready = 0;
      }
      
      ros::spinOnce();
   }

   // TODO: send system shutdown, then send timed shutdown command over serial
}
