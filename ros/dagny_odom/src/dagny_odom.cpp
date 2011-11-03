
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <dagny_msgs/OdometryLite.h>

ros::Publisher odom_pub;

void odom_cb( const dagny_msgs::OdometryLite::ConstPtr & msg ) {
   nav_msgs::Odometry out;
   out.pose.pose = msg->pose;
   out.twist.twist = msg->twist;
   out.header = msg->header;
   out.child_frame_id = msg->child_frame_id;

   odom_pub.publish(out);
}

int main(int argc, char ** argv) {
   ros::init(argc, argv, "dagny_odom");

   ros::NodeHandle nh;

   ros::Subscriber s = nh.subscribe("odometry_lite", 2, &odom_cb);

   odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 2);

   ros::spin();
}
