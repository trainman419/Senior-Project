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
#include <errno.h>
#include <stropts.h>

#include <set>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "gps_common/GPSFix.h"
#include "nav_msgs/Odometry.h"

#include "protocol.h"

using namespace std;

char laser_data[512];
int laser_ready;

#define ROS_PERROR(str) ROS_ERROR("%s: %s", str, strerror(errno))

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

int gps_ready = 0;
Packet<32> gps_packet('G');
// callback on GPS location received
void gpsCallback(const gps_common::GPSFix::ConstPtr & msg) {
   ROS_INFO("Received GPS fix; lat: %f, lon: %f", msg->latitude,
         msg->longitude);
   gps_packet.reset();
   int32_t lat = msg->latitude * 1000000.0;
   int32_t lon = msg->longitude * 1000000.0;
   gps_packet.append(lat);
   gps_packet.append(lon);
   gps_packet.finish();
   gps_ready = 1;
}

#define handler(foo) void foo(Packet<250> & p)
typedef void (*handler_ptr)(Packet<250> & p);

handler_ptr handlers[256];

handler(no_handler) {
   int l = p.outsz();
   const char * in = p.outbuf();
   char * buf = (char*)malloc(l + 1);
   memcpy(buf, in, l);
   buf[l] = 0;

   char * tmpbuf = (char*)malloc(5*l);
   int i;
   for( i=0; i<l; i++ ) {
      sprintf(tmpbuf + (i*5), "0x%02X ", 0xFF & buf[i+1]);
   }
   tmpbuf[i*5] = 0;

   ROS_INFO("No handler for message: %c(%d) %s", buf[0], l, tmpbuf);

   free(buf);
}

handler(shutdown_h) {
   int l = p.outsz();
   const char * in = p.outbuf();
   int shutdown = 1;
   if( l == 9 ) {
      for( int i=0; i<l; i++ ) {
         if( in[i] != 'Z' ) shutdown = 0;
      }
   }
   if( shutdown ) {
      ROS_INFO("Received shutdown");
      // FIXME: shutdown here
      if( system("sudo poweroff") < 0 ) {
         ROS_ERROR("Failed to execute shutdown command");
      }
   } else {
      char * buf = (char*)malloc(l + 1);
      memcpy(buf, in, l);
      buf[l] = 0;
      ROS_INFO("Malformed shutdown %s", buf);
      free(buf);
   }
}

int gps_file;
#define GPS_PIPE "/home/hendrix/gps/gps.out"

// set up whatever we decide to do for GPS
void gps_setup(void) {
   gps_file = -1;
   // create a named FIFO and open it for writing

   struct stat file_info;
   int res = stat(GPS_PIPE, &file_info);
   // if our stat filed or the file isn't a fifo, destroy it and make a fifo
   if( res < 0 || !S_ISFIFO(file_info.st_mode) ) {
      ROS_INFO("File %s wasn't a FIFO; destroying it and making a FIFO",
            GPS_PIPE);
      // remove old whatever
      unlink(GPS_PIPE);
      // create our fifo
      if( mkfifo(GPS_PIPE, 0644) < 0 ) {
         ROS_PERROR("Error creating fifo");
      return;
      }
   }
   // open fifo for writing
   if( (gps_file = open(GPS_PIPE, O_RDWR | O_TRUNC | O_NONBLOCK, 0644)) < 0 ) {
      ROS_PERROR("Error opening GPS pipe");
      return;
   }

   // open gps log file for writing
   /*gps_file = open("/home/hendrix/log/gps.log", 
         O_WRONLY | O_APPEND | O_CREAT, 0644);
   if( gps_file < 0 ) {
      ROS_ERROR("Error opening GPS log file: %s", strerror(errno));
   } else {
      write(gps_file, "GPS log starting\n", 17);
   }*/
}

void gps_end(void) {
   if( gps_file >= 0 ) 
      close(gps_file);
}

handler(gps_h) {
   // take data from GPS and spew it to a fifo somewhere on disk
   //
   // for now, just de-encapsulate and print it to info
   int l = p.outsz() - 1;
   char * buf = (char*)malloc(l + 2);
   memcpy(buf, p.outbuf() + 1, l);
   buf[l] = 0;
   //ROS_INFO("Received GPS: %s", buf);
   if( gps_file >= 0 ) {
      buf[l] = '\n';
      if( write(gps_file, buf, l+1) != l+1 ) {
         ROS_INFO("Write to GPS file failed");
      }
   }
   free(buf);
}

// set up odometry handling
void odometry_setup(void) {
}

inline int read16(char * in) {
   return in[0] | (in[1] << 8);
}
handler(odometry_h) {
   int rcount = p.readu16();
   int lcount = p.readu16();
   int qcount = p.readu16();
   int rspeed = p.reads16();
   int lspeed = p.reads16();
   int qspeed = p.reads16();
   // TODO: publish odometry
//   ROS_INFO("Odo: rc: %d, lc: %d, qc: %d, rs: %d, ls: %d, qs: %d",
//            rcount, lcount, qcount, rspeed, lspeed, qspeed);
}

handler(compass_h) {
   int x = p.reads16();
   int y = p.reads16();
   ROS_INFO("Compass reading (%d, %d): %f", x, y, atan2(-y, x)*180/M_PI);
}

handler(gpslist_h) {
   int cnt = p.readu8();
   int cursor = p.readu8();
   double * lat = (double*)malloc(cnt*sizeof(double));
   double * lon = (double*)malloc(cnt*sizeof(double));
   ROS_INFO("GPS List, size: %d, cursor: %d", cnt, cursor);
   for( int i=0; i<cnt; i++ ) {
      lat[i] = (double)p.reads32() / 1000000.0;
      lon[i] = (double)p.reads32() / 1000000.0;
      ROS_INFO("Lat: %f, Lon: %f", lat[i], lon[i]);
   }
   free(lat);
   free(lon);
}

handler(battery_h) {
   uint8_t main = p.readu8();
   uint8_t motor = p.readu8();
   uint32_t idle = p.readu32();
   ROS_INFO("Idle count: %d", idle);
}

#define IN_BUFSZ 1024

int main(int argc, char ** argv) {
   unsigned char in_buffer[IN_BUFSZ];
   int in_cnt = 0;
   int cnt = 0;
   int i;

   laser_ready = 0;

   for( i=0; i<512; i++ ) {
      laser_data[i] = 64;
   }

   // Set up message handler array
   for( i=0; i<256; i++ ) {
      handlers[i] = no_handler;
   }
   handlers['Z'] = shutdown_h;

   odometry_setup();
   handlers['O'] = odometry_h;
   handlers['C'] = compass_h;

   gps_setup();
   handlers['G'] = gps_h;
   handlers['L'] = gpslist_h;
   handlers['b'] = battery_h;

   ros::init(argc, argv, "hardware_interface");

   ros::NodeHandle n;

   // TODO: set up publishers here. I don't think we have any yet
   //  will probably want publishers for battery, sonar, wheel and bump data

   // deal with GPS data some other way; probably write it to a fifo and have
   //  gpsd pick it up. TODO
   //
   
   // I'm going to hardcode the port and settings because this is hardware-
   // specific anyway
   // open serial port
   int serial = open("/dev/ttyS1", O_RDWR | O_NOCTTY);
   if( serial < 0 ) {
      perror("Failed to open /dev/ttyS1");
      // die. ungracefully.
      return -1;
   }

   struct termios tio;
   tcgetattr(serial, &tio);

   // set non-blocking input mode
   tio.c_lflag = 0; // raw input
   tio.c_cc[VMIN] = 0;
   tio.c_cc[VTIME] = 0;

   /*ROS_INFO("c_iflag %X", tio.c_iflag);
   ROS_INFO("INLCR %X", INLCR);
   ROS_INFO("IGNCR %X", IGNCR);
   ROS_INFO("ICRNL %X", ICRNL);
   ROS_INFO("IXON  %X", IXON);
   ROS_INFO("IXOFF  %X", IXOFF);*/
   // no input options, just normal input
   tio.c_iflag = 0;

   // set baud rate
   cfsetospeed(&tio, B115200);
   cfsetispeed(&tio, B115200);
   
   tcsetattr(serial, TCSANOW, &tio);

//   ros::Subscriber sub = n.subscribe("scan", 5, laserCallback);
   ros::Subscriber gps_sub = n.subscribe("extended_fix", 5, gpsCallback);

   ros::Rate loop_rate(10);

   while( ros::ok() ) {
      //ROS_INFO("start serial input");
      cnt = read(serial, in_buffer + in_cnt, IN_BUFSZ - in_cnt - 1); 
      if( cnt > 0 ) {
         // append a null byte
         in_buffer[cnt + in_cnt] = 0;
         //ROS_INFO("Read %d characters", cnt);
         in_cnt += cnt;
         //ROS_INFO("Buffer size %d", in_cnt);

         // parse out newline-terminated strings and call appropriate functions
         int start = 0;
         int i = 0;
         while( i < in_cnt ) {
            for( ; i < in_cnt && in_buffer[i] != '\r' ; i++);

            if( i < in_cnt && in_buffer[i] == '\r' ) {
               // check that our string isn't just the terminating character
               if( i - start > 1 ) {
                  // we got a string. call the appropriate function
                  Packet<250> p((char*)(in_buffer+start), i-start);
                  handlers[in_buffer[start]](p);
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
      }
      
      ros::spinOnce();

      // write pending data to serial port
      if( gps_ready ) {
         cnt = write(serial, gps_packet.outbuf(), gps_packet.outsz());
         gps_ready = 0;
      }

      //ROS_INFO("start laser transmit");
      if( laser_ready ) {
         cnt = write(serial, "L", 1);
         //ROS_INFO("Wrote %d bytes", cnt);
         cnt = write(serial, laser_data, 512);
         //ROS_INFO("Wrote %d bytes", cnt);
         cnt = write(serial, "\r\r\r\r\r\r\r\r", 1);
         //ROS_INFO("Wrote %d bytes", cnt);
         laser_ready = 0;
      }

      loop_rate.sleep();
   }

   gps_end();
}
