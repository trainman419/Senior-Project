/* serial.c
 *
 * Serial routines for ATmega32
 * Modified to support the multiple UARTS of the Atmega2560
 *
 * TODO: modify to add separate TX and RX init functions
 *
 * Author: Austin Hendrix
*/

#include "serial.h"
#include <avr/io.h>
#include <avr/interrupt.h>

// maybe increase size since the atmega2560 has more memory?


/* recieve circular fifo (10 bytes total 20% overhead) */
uint8_t rx_head[4]; /* points to next writeable byte */
volatile uint8_t rx_size[4]; /* number of byts in buffer */
uint8_t rx_buf[4][8] __attribute((aligned(8)));
uint8_t * rx_ptr[4];

/* send circular fifo (10 bytes total, 20% overhead) */
uint8_t tx_head[4]; /* next writeable byte */
volatile uint8_t tx_size[4]; /* number of bytes in buffer */
uint8_t tx_buf[4][8] __attribute__((aligned(8)));
uint8_t * tx_ptr;

volatile uint8_t * ucsr[] = {&UCSR0A, &UCSR1A, &UCSR2A, &UCSR3A};
#define A 0
#define B 1
#define C 2

/* recieve interrupt 0 */
ISR(USART0_RX_vect) /* receive complete */
{
   /* read into fifo, allow overruns for now. */
   /*rx_buf[0][rx_head[0]++] = UDR0;
   rx_head[0] &= 127;
   rx_size[0]++;*/
   *rx_ptr[0] = UDR0;
   rx_ptr[0] = (uint8_t*)((uint16_t)(rx_ptr[0]+1) & (uint16_t)0xFFF8);
}

/* recieve interrupt 1 */
ISR(USART1_RX_vect) /* receive complete */
{
   /* read into fifo, allow overruns for now. */
   rx_buf[1][rx_head[1]++] = UDR1;
   rx_head[1] &= 127;
   rx_size[1]++;
}

/* recieve interrupt 2 */
ISR(USART2_RX_vect) /* receive complete */
{
   /* read into fifo, allow overruns for now. */
   rx_buf[2][rx_head[2]++] = UDR2;
   rx_head[2] &= 127;
   rx_size[2]++;
}

/* recieve interrupt 3 */
ISR(USART3_RX_vect) /* receive complete */
{
   /* read into fifo, allow overruns for now. */
   rx_buf[3][rx_head[3]] = UDR3;
   rx_head[3]++;
   rx_head[3] &= 127;
   rx_size[3]++;
}

/* determine if there is data in the rx buffer */
uint8_t rx_ready(uint8_t port) {
   return rx_size[port] > 0;
}

/* get a byte from recieve buffer. block until data recieved */
uint8_t rx_byte(uint8_t port) {
   while(!rx_size[port]);

   ucsr[port][B] &= ~(1 << 7); /* disable receive interrupt */

   uint8_t res = rx_buf[port][(rx_head[port] - rx_size[port]) & 127];
   rx_size[port]--;

   ucsr[port][B] |= (1 << 7); /* enable receive interrupt */
   return res;
}

/* transmit interrupt 0 */
ISR(USART0_UDRE_vect) /* ready for more data to transmit */
{
   if (tx_size[0]) {
      UDR0 = tx_buf[0][(tx_head[0] - tx_size[0]--) & 127];
   } else {
	   UCSR0B &= ~(1 << 5); /* disable send interrupt */
   }
}

/* transmit interrupt 1 */
ISR(USART1_UDRE_vect) /* ready for more data to transmit */
{
   if (tx_size[1]) {
      UDR1 = tx_buf[1][(tx_head[1] - tx_size[1]--) & 127];
   } else {
	   UCSR1B &= ~(1 << 5); /* disable send interrupt */
   }
}

/* transmit interrupt 2 */
ISR(USART2_UDRE_vect) /* ready for more data to transmit */
{
   if (tx_size[2]) {
      UDR2 = tx_buf[2][(tx_head[2] - tx_size[2]--) & 127];
   } else {
	   UCSR2B &= ~(1 << 5); /* disable send interrupt */
   }
}

/* transmit interrupt 3 */
ISR(USART3_UDRE_vect) /* ready for more data to transmit */
{
   if (tx_size[3]) {
      UDR3 = tx_buf[3][(tx_head[3] - tx_size[3]--) & 127];
   } else {
	   UCSR3B &= ~(1 << 5); /* disable send interrupt */
   }
}

/* determine if there is space for another byte in the transmit buffer */
uint8_t tx_ready(uint8_t port) {
   return tx_size[port] < 8;
}

/* put a byte in the transmit buffer. block until space available */
void tx_byte(uint8_t port, uint8_t b) {
   while (tx_size[port] > 127);

   /* messing with buffer pointers is not atomic; need locking here */
   ucsr[port][B] &= ~(1 << 5); /* diable send interrupt (just enough locking) */
   tx_buf[port][tx_head[port]++] = b;
   tx_head[port] &= 127;
   tx_size[port]++;
   /* done messing with buffer pointers */

   ucsr[port][B] |= (1 << 5); /* enable send interrupt */
}

volatile uint8_t * rxtx[] = {&DDRE, &DDRD, &DDRH, &DDRJ};
uint8_t rxbit[] = {0, 2, 0, 0};

volatile uint16_t * ubrr[] = {&UBRR0, &UBRR1, &UBRR2, &UBRR3};

/* initialize serial tx */
void serial_init_tx(uint8_t port) {
	ucsr[port][C] = 0x8E; /* no parity, 1 stop, 8bit */

   *ubrr[port] = 0x67; /* serial divisor 9600 baud */

	/* USART init code */
   ucsr[port][B] |= 0x08; /* RX and TX enable, interrupts */

   /* buffer init */
   tx_head[port] = 0;
   tx_size[port] = 0;
}

/* initialize serial rx */
void serial_init_rx(uint8_t port) {
   /* rx pin setup */
   /* set pin input */
   rxtx[port][0] &= ~(1 << rxbit[port]);
   /* enable input pin pull-up */
   rxtx[port][1] &= ~(1 << rxbit[port]);

	ucsr[port][C] = 0x8E; /* no parity, 1 stop, 8bit */

   *ubrr[port] = 0x67; /* serial divisor 9600 baud */

	/* USART init code */
   ucsr[port][B] |= 0x10; /* RX and TX enable, interrupts */

   /* buffer init */
   rx_head[port] = 0;
   rx_size[port] = 0;

	/* set the USART_RXC interrupt enable bit */
   ucsr[port][B] |= (1 << 7);
}

/* setup and enable serial interrupts */
void serial_init(uint8_t port)
{
   serial_stop(port);

   serial_init_rx(port);
   serial_init_tx(port);
}

void serial_baud(uint8_t port, uint32_t baud) {
   uint32_t ubr = 10000000; // ubr is 10x input clock
   ubr /= baud; // divide by baud rate
   // ubr is now 10x target value

   // add 5 (0.5) and divide by 10 to round properly
   ubr += 5; 
   ubr /= 10;

   // final subtraction
   ubr--;
   //ubr--;
   // FIXME: deal with baud rates that are too high here
   *ubrr[port] = ubr;
}

/* stops the serial interrupts */
void serial_stop(uint8_t port)
{
	/* clear both interrupt enable bits */
   ucsr[port][B] &= ~( (1 << 5) | (1 << 7));
}

