/*
 * dagny_odom.cpp
 *
 * Take simple odometry messages and republish them as full Odometry messages
 *  and as a tf transform
 *
 * Author: Austin Hendrix
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <dagny_msgs/OdometryLite.h>
#include <tf/transform_broadcaster.h>

ros::Publisher odom_pub;

void odom_cb( const dagny_msgs::OdometryLite::ConstPtr & msg ) {
   static tf::TransformBroadcaster odom_tf;
   // odometry message
   nav_msgs::Odometry out;
   out.pose.pose = msg->pose;
   out.twist.twist = msg->twist;
   out.header = msg->header;
   out.child_frame_id = msg->child_frame_id;

   odom_pub.publish(out);

   // tf transform
   geometry_msgs::TransformStamped transform;
   transform.header = msg->header;
   transform.child_frame_id = msg->child_frame_id;
   transform.transform.translation.x = msg->pose.position.x;
   transform.transform.translation.y = msg->pose.position.y;
   transform.transform.translation.z = msg->pose.position.z;
   transform.transform.rotation = msg->pose.orientation;
   odom_tf.sendTransform(transform);
}

int main(int argc, char ** argv) {
   ros::init(argc, argv, "dagny_odom");

   ros::NodeHandle nh;

   ros::Subscriber s = nh.subscribe("odometry_lite", 2, &odom_cb);

   odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 2);

   ros::spin();
}
