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

#include <set>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float32.h>


#include "protocol.h"
#include "steer.h"

using namespace std;

char laser_data[512];
int laser_ready;

// for publishing odometry and compass data
float heading;
ros::Publisher odo_pub;
ros::Publisher sonar_pub;
ros::Publisher gps_pub;
//ros::Publisher goalList_pub;

// for resolving offsets back to lat/lon for our user interface
//ros::ServiceClient r_offset;
//ros::ServiceClient offset;

/*
struct {
   nav_msgs::Odometry last_pos;
   uint8_t steer;
   int8_t speed;
} state;
*/

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

   //laser_ready = 1;
}

int cmd_ready = 0;
char cmd_buf[12];
Packet cmd_packet('C', 12, cmd_buf);

// TODO: subscribe to ackermann_msgs::AckermannDrive too/instead
void cmdCallback( const geometry_msgs::Twist::ConstPtr & cmd_vel ) {
   // internal speed specified as 2000/(ms per count)
   // 2 / (sec per count)
   // 2 * counts / sec
   // ( 1 count = 0.03 m )
   // 1/2 * 0.032 m / sec
   // 0.016 m / sec
   // target speed in units of 0.016 m / sec
   int16_t target_speed = cmd_vel->linear.x * 62.5;
   // angular z > 0 is left
   // vr = vl / r
   // r = vl / vr
   int8_t steer = 0;
   if( cmd_vel->angular.z == 0.0 ) {
      steer = 0;
   } else {
      float radius = fabs(cmd_vel->linear.x / cmd_vel->angular.z);
      int16_t tmp = radius2steer(radius);

      if( tmp < -120 ) 
         tmp = -120;
      if( tmp > 120 ) 
         tmp = 120;

      if( cmd_vel->angular.z > 0 ) {
         steer = -tmp;
      } else {
         steer = tmp;
      }
   }

   ROS_INFO("cmd_vel: %d %d", target_speed, steer);

   cmd_packet.reset();
   cmd_packet.append(target_speed);
   cmd_packet.append(steer);
   cmd_packet.finish();
   cmd_ready = 1;
}


/*
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

void posCallback(const nav_msgs::Odometry::ConstPtr & msg) {
   global_map::RevOffset off;
   off.request.loc.col = msg->pose.pose.position.x;
   off.request.loc.row = msg->pose.pose.position.y;

   state.last_pos = *msg;

   if( r_offset.call(off) ) {
      ROS_INFO("Received position lat: %lf, lon: %lf", off.response.lat,
            off.response.lon);
      int32_t lat = off.response.lat * 1000000.0;
      int32_t lon = off.response.lon * 1000000.0;

      gps_packet.reset();
      gps_packet.append(lat);
      gps_packet.append(lon);
      gps_packet.finish();
      gps_ready = 1;
   } else {
      ROS_ERROR("Failed to call RevOffset");
   }
}

int control_ready = 0;
Packet<16> control_packet('M');
void controlCallback(const hardware_interface::Control::ConstPtr & msg) {
   ROS_INFO("Control packet: (%d, %d)", msg->speed, msg->steer);
   control_packet.reset();
   control_packet.append(msg->speed);
   control_packet.append(msg->steer);
   control_packet.finish();

   state.steer = msg->steer;
   state.speed = msg->speed;

   control_ready = 1;
}
*/

#define handler(foo) void foo(Packet & p)
typedef void (*handler_ptr)(Packet & p);

handler_ptr handlers[256];

handler(no_handler) {
   int l = p.outsz();
   const char * in = p.outbuf();
   char * buf = (char*)malloc(l + 1);
   memcpy(buf, in, l);
   buf[l] = 0;

   char * tmpbuf = (char*)malloc(5*l + 1);
   int i;
   for( i=0; i<l; i++ ) {
      sprintf(tmpbuf + (i*5), "0x%02X ", 0xFF & buf[i+1]);
   }
   tmpbuf[i*5] = 0;

   ROS_INFO("No handler for message: %02X(%d) %s", buf[0], l, tmpbuf);

   free(buf);
   free(tmpbuf);
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

handler(gps_h) {
   // TODO: rewrite this now that the arduino is parsing GPS
   // message format
   // int32_t lat
   // int32_t lon
   int32_t lat = p.reads32();
   int32_t lon = p.reads32();
   ROS_INFO("GPS lat: %d lon: %d", lat, lon);
   // TODO: convert this into an appropriate message type and publish
   sensor_msgs::NavSatFix gps;
   gps.latitude = lat / 1000000.0;
   gps.longitude = lon / 1000000.0;
   gps_pub.publish(gps);
}

// set up odometry handling
void odometry_setup(void) {
}

// squares per encoder count
#define Q_SCALE 0.29

handler(odometry_h) {
   static tf::TransformBroadcaster odom_tf;
   // message format:
   // float linear
   // float angular
   // float x
   // float y
   // float yaw
   nav_msgs::Odometry odo_msg;
   odo_msg.header.stamp = ros::Time::now();
   odo_msg.header.frame_id = "odom";
   odo_msg.child_frame_id = "base_link";
   odo_msg.twist.twist.linear.x = p.readfloat();
   odo_msg.twist.twist.angular.z = p.readfloat();
   odo_msg.pose.pose.position.x = p.readfloat();
   odo_msg.pose.pose.position.y = p.readfloat();
   float yaw = p.readfloat();
   odo_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

   odo_pub.publish(odo_msg);

   // tf transform
   geometry_msgs::TransformStamped transform;
   transform.header = odo_msg.header;
   transform.child_frame_id = odo_msg.child_frame_id;
   transform.transform.translation.x = odo_msg.pose.pose.position.x;
   transform.transform.translation.y = odo_msg.pose.pose.position.y;
   transform.transform.translation.z = odo_msg.pose.pose.position.z;
   transform.transform.rotation = odo_msg.pose.pose.orientation;
   odom_tf.sendTransform(transform);
}

handler(gpslist_h) {
   // TODO: update this
   /*
   int cnt = p.readu8();
   int cursor = p.readu8();
   double * lat = (double*)malloc(cnt*sizeof(double));
   double * lon = (double*)malloc(cnt*sizeof(double));

   goal_list::GoalList list;
   global_map::Offset o;

   ROS_INFO("GPS List, size: %d, cursor: %d", cnt, cursor);
   for( int i=0; i<cnt; i++ ) {
      lat[i] = (double)p.reads32() / 1000000.0;
      lon[i] = (double)p.reads32() / 1000000.0;
      o.request.lat = lat[i];
      o.request.lon = lon[i];
      if( offset.call(o) ) {
         list.goals.push_back(o.response.loc);
      } else {
         ROS_ERROR("Failed to call Offset");
      }
      ROS_INFO("Lat: %f, Lon: %f", lat[i], lon[i]);
   }
   goalList_pub.publish(list);
   free(lat);
   free(lon);
   */
}

FILE * battery_log;
void battery_setup() {
   char logfile[1024];
   char date[256];
   struct tm * timeptr;
   time_t now = time(0);

   timeptr = localtime(&now);
   strftime(date, 256, "%F-%T", timeptr);
   snprintf(logfile, 1024, "/home/hendrix/log/battery-%s.log", date);
   battery_log = fopen(logfile, "w");
   if( battery_log == NULL ) {
      ROS_PERROR("Failed to open logfile");
   }
}

handler(battery_h) {
   static uint8_t cnt = 0;
   uint8_t main = p.readu8();
   uint8_t motor = p.readu8();
   uint32_t idle = p.readu32();

   if( cnt % 5 == 0 ) {
      char date[256];
      struct tm * timeptr;
      time_t now = time(0);

      timeptr = localtime(&now);
      strftime(date, 256, "%F-%T", timeptr);
      fprintf(battery_log, "%s: %d %d\n", date, main, motor);
      fflush(battery_log);
   }
   cnt++;

   ROS_INFO("Idle count: %d", idle);
}

handler(idle_h) {
   // idle message format:
   // uint16_t idle

   uint16_t idle = p.readu16();
   ROS_INFO("Idle count: %d", idle);
}


#define NUM_SONARS 5
handler(sonar_h) {
   // sonar message format:
   // uint8_t[5] sonars
   char sonar_frames[5][8] = { "sonar_1", "sonar_2", "sonar_3", "sonar_4", "sonar_5" };
   uint8_t s;
   ros::Time n = ros::Time::now();
   sensor_msgs::Range sonar;
   for( int i=0; i<NUM_SONARS; ++i ) {
      s = p.readu8();
      sonar.range = s * 0.0254; // convert inches to m
      sonar.min_range = 6 * 0.0254;
      sonar.max_range = 255 * 0.0254;
      sonar.field_of_view = 45 * M_PI / 180.0; // approx 45-degree FOV
      sonar.radiation_type = sensor_msgs::Range::ULTRASOUND;

      sonar.header.stamp = n;
      sonar.header.frame_id = sonar_frames[i];

      sonar_pub.publish(sonar);
   }
}

handler(imu_h) {
   // imu message format:
   // float[3]
   float x, y, z;
   x = p.readfloat();
   y = p.readfloat();
   z = p.readfloat();
   //ROS_INFO("IMU data: (% 03.7f, % 03.7f, % 03.7f)", x, y, z);
   heading = z;
}

#define IN_BUFSZ 1024

int main(int argc, char ** argv) {
   unsigned char in_buffer[IN_BUFSZ];
   int in_cnt = 0;
   int cnt = 0;
   int i;

   char heartbeat_buf[8];
   Packet heartbeat_packet('H', 8, heartbeat_buf);

   laser_ready = 0;

   for( i=0; i<512; i++ ) {
      laser_data[i] = 64;
   }

   // Set up message handler array
   for( i=0; i<256; i++ ) {
      handlers[i] = no_handler;
   }
   //handlers['Z'] = shutdown_h;

   odometry_setup();
   handlers['O'] = odometry_h;
   handlers['I'] = idle_h;

   //gps_setup();
   handlers['G'] = gps_h;
   //handlers['L'] = gpslist_h;
   //handlers['b'] = battery_h;
   handlers['S'] = sonar_h;
   handlers['U'] = imu_h;

   ros::init(argc, argv, "hardware_interface");

   ros::NodeHandle n;

   // I'm going to hardcode the port and settings because this is hardware-
   // specific anyway
   // open serial port
   int serial = open("/dev/ttyACM1", O_RDWR | O_NOCTTY);
   if( serial < 0 ) {
      perror("Failed to open /dev/ttyACM1");
      // die. ungracefully.
      return -1;
   }

   struct termios tio;
   tcgetattr(serial, &tio);

   // set non-blocking input mode
   tio.c_lflag = 0; // raw input
   tio.c_cc[VMIN] = 0;
   tio.c_cc[VTIME] = 0;

   // no input options, just normal input
   tio.c_iflag = 0;

   // set baud rate
   cfsetospeed(&tio, B115200);
   cfsetispeed(&tio, B115200);
   
   tcsetattr(serial, TCSANOW, &tio);

   sleep(2); // sleep for two seconds while bootloader runs

   ros::Subscriber cmd_sub = n.subscribe("cmd_vel", 1, cmdCallback);
//   ros::Subscriber sub = n.subscribe("scan", 5, laserCallback);
   //ros::Subscriber pos_sub = n.subscribe("position", 5, posCallback);
   // TODO: update this to take ackermann_cmd
   //ros::Subscriber control_sub = n.subscribe("control", 5, controlCallback);

   odo_pub = n.advertise<nav_msgs::Odometry>("odom", 10);
   //goalList_pub = n.advertise<goal_list::GoalList>("goal_list", 2);
   sonar_pub = n.advertise<sensor_msgs::Range>("sonar", 10);
   gps_pub = n.advertise<sensor_msgs::NavSatFix>("gps", 10);

   //r_offset = n.serviceClient<global_map::RevOffset>("RevOffset");
   //offset = n.serviceClient<global_map::Offset>("Offset");
   ROS_INFO("hardware_interface ready");

   ros::Rate loop_rate(20);

   int itr = 0;
   int bw = 0;

   while( ros::ok() ) {
      cnt = read(serial, in_buffer + in_cnt, IN_BUFSZ - in_cnt - 1); 
      if( cnt > 0 ) {
         // append a null byte
         in_buffer[cnt + in_cnt] = 0;
         in_cnt += cnt;
         // parse out newline-terminated strings and call appropriate functions
         int start = 0;
         int i = 0;
         while( i < in_cnt ) {
            for( ; i < in_cnt && in_buffer[i] != '\r' ; i++);

            if( i < in_cnt && in_buffer[i] == '\r' ) {
               // check that our string isn't just the terminating character
               if( i - start > 1 ) {
                  // we got a string. call the appropriate function
                  Packet p((char*)(in_buffer+start), i-start);
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

      if( laser_ready ) {
         cnt = write(serial, "L", 1);
         cnt = write(serial, laser_data, 512);
         cnt = write(serial, "\r\r\r\r\r\r\r\r", 1);
         laser_ready = 0;
      }

      if( cmd_ready ) {
         cnt = write(serial, cmd_packet.outbuf(), cmd_packet.outsz());
         ROS_INFO("cmd_vel sent");
         if( cnt != cmd_packet.outsz() ) {
            ROS_ERROR("Failed to send cmd_vel data");
         }
         cmd_ready = 0;
      }

      // send heartbeat
      bw += cnt;
      ++itr;

      // heartbeat and bandwidth measurement every 0.5 sec
      if( itr == 9 ) {
         heartbeat_packet.reset();
         heartbeat_packet.finish();
         cnt = write(serial, heartbeat_packet.outbuf(), heartbeat_packet.outsz());
         ROS_INFO("Bandwidth: %d bytes/sec", bw * 2);
         itr = 0;
         bw = 0;
      }

      loop_rate.sleep();
   }
}
