/*
 * Header for new interrupt-based management.
 *
 */

void interrupt_init(void);

extern volatile uint32_t ticks;

extern ros::Publisher odom_pub;
