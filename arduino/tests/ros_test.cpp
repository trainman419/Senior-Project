
#include <ros.h>
#include <std_msgs/Empty.h>

void cb( const std_msgs::Empty & m ) {
}

ros::NodeHandle nh;
ros::Subscriber<std_msgs::Empty> sub("sub", & cb);

std_msgs::Empty msg;
ros::Publisher pub("pub", &msg);

int main() {
   unsigned long last_pub;

   nh.initNode();
   nh.advertise(pub);

   nh.subscribe(sub);

   last_pub = nh.now().toNsec();

   while(1) {
      nh.spinOnce();
      // do our best to publish once per second
      if( nh.now().toNsec() - last_pub > 1000000000ull ) {
         pub.publish(&msg);
         last_pub += 1000000000ull;
      }
   }
}
