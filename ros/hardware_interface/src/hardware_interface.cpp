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
#include <math.h>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

char laser_data[512];
int laser_ready;

// callback on laser scan received.
void laserCallback(const sensor_msgs::LaserScan::ConstPtr & msg) {
   //ROS_INFO("Data size %d", msg->ranges.size());
   //for(int i=0; i<msg->ranges.size(); i+= 2 ) {
   for(unsigned int i=0; i<msg->ranges.size(); i++ ) {
      if( i < 512 ) {
         // average adjacent data points
         //float data = (msg->ranges[i] + msg->ranges[i+1]) / 2;
         float data = msg->ranges[i];
         // scale to fit into a byte. 250 = 5.0m
         data = data * 50;
         //laser_data[i/2] = (char)data;
         laser_data[i] = (char)data;
      }
   }

   /*ROS_INFO("Angle min: %lf, angle delta: %lf, angle max: %lf",
      msg->angle_min * 180.0 / M_PI, 
      msg->angle_increment * 180.0 / M_PI,
      msg->angle_max * 180.0 / M_PI);
      */

   laser_ready = 1;
}

typedef void message_handler(char * in, int len);
#define handler(foo) void foo(char * in, int l)
typedef void (*handler_ptr)(char *, int);

handler_ptr handlers[256];

handler(no_handler) {
   char * buf = (char*)malloc(l + 1);
   memcpy(buf, in, l);
   buf[l] = 0;
   ROS_INFO("No handler for message: %s", buf);
}

handler(shutdown_h) {
   int shutdown = 1;
   if( l == 9 ) {
      for( int i=0; i<l; i++ ) {
         if( in[i] != 'Z' ) shutdown = 0;
      }
   }
   if( shutdown ) {
      ROS_INFO("Received shutdown");
      // TODO: shutdown here
   } else {
      char * buf = (char*)malloc(l + 1);
      memcpy(buf, in, l);
      buf[l] = 0;
      ROS_INFO("Malformed shutdown %s", buf);
   }
}

#define IN_BUFSZ 1024

int main(int argc, char ** argv) {
   char in_buffer[IN_BUFSZ];
   int in_cnt = 0;
   int cnt = 0;

   laser_ready = 0;

   for( int i=0; i<512; i++ ) {
      laser_data[i] = 64;
   }

   // Set up message handler array
   for( int i=0; i<256; i++ ) {
      handlers[i] = no_handler;
   }
   handlers['Z'] = shutdown_h;

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

   tio.c_lflag = 0; // raw input
   tio.c_cc[VMIN] = 0;
   tio.c_cc[VTIME] = 0;

   cfsetospeed(&tio, B115200);
   cfsetispeed(&tio, B115200);
   
   tcsetattr(serial, TCSANOW, &tio);

   ros::Subscriber sub = n.subscribe("scan", 5, laserCallback);

   ros::Rate loop_rate(10);

   while( ros::ok() ) {
      // TODO: read serial port for incoming data and do something with it
      
      //ROS_INFO("Starting output");
      // TODO: write pending data to serial port
      if( laser_ready ) {
         cnt = write(serial, "L", 1);
         cnt = write(serial, laser_data, 512);
         cnt = write(serial, "\r\r\r\r\r\r\r\r", 1);
         laser_ready = 0;
      }

      //ROS_INFO("Starting input");
      cnt = read(serial, in_buffer + in_cnt, IN_BUFSZ - in_cnt - 1); 
      //cnt = 0;
      if( cnt > 0 ) {
         // append a null byte
         in_buffer[cnt + in_cnt] = 0;
         //ROS_INFO("Read %d characters", cnt);
         //ROS_INFO("Read %s", in_buffer);
         in_cnt += cnt;

         // TODO: parse out newline-terminated strings and call appropriate 
         //  functions here

         int start = 0;
         int i = 0;
         while( i < in_cnt ) {
            for( ; i < in_cnt && 
                   (in_buffer[i] != '\r' && in_buffer[i] != '\n');
                   i++);
            if( in_buffer[i] == '\r' || in_buffer[i] == '\n') {
               // we got a string. call the appropriate function
               if( i - start > 1 ) {
                  handlers[in_buffer[start]](in_buffer+start, i - start);
               }
               start = i+1;
            }
            i++;
         }

         // shift remaining data to front of buffer
         for( i=start; i<in_cnt; i++ ) {
            in_buffer[i-start] = in_buffer[i];
         }

         in_cnt -= start;

         
         /*int i;
         for( i=0; i<in_cnt && 'Z' != in_buffer[i]; i++ );

         // if we don't have any Z's in our buffer, toss it
         if( i == in_cnt ) in_cnt = 0;
         else {
            if( in_cnt - i > 8 ) {
               int z_cnt = 0;
               int last = i;
               for( int j=i; j<in_cnt && z_cnt < 8; j++ ) {
                  if( in_buffer[j] == 'Z' ) {
                     z_cnt++;
                  } else {
                     z_cnt = 0;
                     last = j;
                  }
               }
               if( z_cnt == 8 ) {
                  // oh holy shit this is where we power off
                  // TODO: shut down
                  ROS_INFO("Got a power down message");
                  system("sudo poweroff");
               } else {
                  // shift down from last non-Z character
                  last++;
                  int offset = last;
                  for( int j=0; last < in_cnt; last++,j++) {
                     in_buffer[j] = in_buffer[last];
                  }
                  in_cnt -= offset;
               }
            } else {
               int offset=i;
               // shift buffer down and wait for later
               for( int j=0; i < in_cnt; i++, j++ ) {
                  in_buffer[j] = in_buffer[i];
               }
               // adjust count
               in_cnt -= offset;
            }
         }*/
      }
      
      ros::spinOnce();

      loop_rate.sleep();
   }

   // TODO: send system shutdown, then send timed shutdown command over serial
}
