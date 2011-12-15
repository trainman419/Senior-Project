
#include <ros.h>
#include <std_msgs/Empty.h>
#include <avr/interrupt.h>

extern "C" {
#include <drivers/led.h>
   /* error handler for pure virutal function calls
    *  turn on LED and loop forever */
   void __cxa_pure_virtual() {
      while(1) {
         led_on();
      }
   }
}

void cb( const std_msgs::Empty & m ) {
}

ros::NodeHandle nh;
ros::Subscriber<std_msgs::Empty> sub("sub", & cb);

std_msgs::Empty msg;
ros::Publisher pub("pub", &msg);

uint32_t ticks = 0;

/* interrupt routine */
/* TIMER0 OVF */
ISR(TIMER0_OVF_vect) {
   ticks++;
}

int main() {
   unsigned long last_pub;

   /* set up interrupt handling */
   // set up timer interrupts
   // fast PWM mode; interrupt and reset when counter equals OCR0A
   // prescalar 64
   TCCR0A = (1 << WGM01 | 1 << WGM00);
   TCCR0B = (1 << WGM02 | 1 << CS01 | 1 << CS00);
   // interrupt on "overflow" (counter match)
   TIMSK0 = (1 << TOIE0);
   OCR0A  = 249; // 250 counts per tick


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
