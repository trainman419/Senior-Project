
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <math.h>

using namespace std;

void laserCallback(const sensor_msgs::LaserScan::ConstPtr & msg) {
   double min_range = 10000.0; // some big value
   int min_pos = -1;
   unsigned int i;
   for( i=0; i < msg->ranges.size(); i++ ) {
      if( msg->ranges[i] != 0.0 && msg->ranges[i] < min_range ) {
         min_range = msg->ranges[i];
         min_pos = i;
      }
   }
   printf("Minimum laser scan distance of %lf at %d\n", min_range, min_pos);
}

int main(int argc, char ** argv) {
   ros::init(argc, argv, "laser");

   ros::NodeHandle n;

   ros::Subscriber laser_sub = n.subscribe("scan", 2, laserCallback);

   ros::spin();

   return 0;
}
