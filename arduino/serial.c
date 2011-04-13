/* serial.c
 *
 * Serial routines for ATmega32
 * Modified to support the multiple UARTS of the Atmega2560
 *
 * Author: Austin Hendrix
*/

#include "serial.h"
#include "lock.h"
#include <avr/io.h>
#include <avr/interrupt.h>

// maybe increase size since the atmega2560 has more memory?
#define BUF_SZ 128


/* recieve circular fifo (10 bytes total 20% overhead) */
uint8_t rx_head[4]; /* points to next writeable byte */
volatile uint16_t rx_size[4]; /* number of byts in buffer */
uint8_t rx_buf[4][BUF_SZ];

/* send circular fifo (10 bytes total, 20% overhead) */
uint8_t tx_head[4]; /* next writeable byte */
volatile uint16_t tx_size[4]; /* number of bytes in buffer */
uint8_t tx_buf[4][BUF_SZ];
uint8_t tx_lock[4];

volatile uint8_t * ucsr[] = {&UCSR0A, &UCSR1A, &UCSR2A, &UCSR3A};
#define A 0
#define B 1
#define C 2

/* recieve interrupt 0 */
ISR(USART0_RX_vect) /* receive complete */
{
   cli();
   /* read into fifo, allow overruns for now. */
   rx_buf[0][rx_head[0]++] = UDR0;
   rx_head[0] %= BUF_SZ;
   rx_size[0]++;
   sei();
}

/* recieve interrupt 1 */
ISR(USART1_RX_vect) /* receive complete */
{
   /* read into fifo, allow overruns for now. */
   rx_buf[1][rx_head[1]++] = UDR1;
   rx_head[1] %= BUF_SZ;
   rx_size[1]++;
}

/* recieve interrupt 2 */
ISR(USART2_RX_vect) /* receive complete */
{
   /* read into fifo, allow overruns for now. */
   rx_buf[2][rx_head[2]++] = UDR2;
   rx_head[2] %= BUF_SZ;
   rx_size[2]++;
}

/* recieve interrupt 3 */
ISR(USART3_RX_vect) /* receive complete */
{
   cli();
   /* read into fifo, allow overruns for now. */
   rx_buf[3][rx_head[3]++] = UDR3;
   rx_head[3] %= BUF_SZ;
   rx_size[3]++;
   sei();
}

/* determine if there is data in the rx buffer */
uint8_t rx_ready(uint8_t port) {
   return rx_size[port] > 0;
}

/* get a byte from recieve buffer. block until data recieved */
uint8_t rx_byte(uint8_t port) {
   while(!rx_size[port]);

   cli();
   ucsr[port][B] &= ~(1 << 7); /* disable receive interrupt */

   uint8_t res = rx_buf[port][(rx_head[port] - rx_size[port]) % BUF_SZ];
   rx_size[port]--;

   ucsr[port][B] |= (1 << 7); /* enable receive interrupt */
   sei();
   return res;
}

uint8_t * tx_ptrs[4];
uint16_t * tx_szs[4];
uint16_t tx_pos = 0;

/* transmit interrupt 0 */
/* modified: transmit to brain */
ISR(USART0_UDRE_vect) /* ready for more data to transmit */
{
   cli();
   /* New concept for transmitting whole packets:
    * send a pointer to the buffer to the transmit routine
    * rather than copying buffers around
    *
    * Implementation:
    * store a circular buffer of pointers to buffers and pointers to 
    * buffer sizes; when the interrupt is done transmitting, use the size 
    * pointer to set the buffer size to 0
    *
    * TODO: test and propagate to other serial routines if appropriate
    * TODO: re-implement legacy byte-oriented interface
    * TODO: implement lower-level passthough so we don't need to buffer a whole
    *    packet
    */
   
   /*
   if (tx_size[0]) {
      UDR0 = tx_buf[0][(tx_head[0] - tx_size[0]--) % BUF_SZ];
   */
   if( tx_size[0]) {
      uint8_t p = (tx_head[0] - tx_size[0]) % 4;
      uint8_t b = tx_ptrs[p][tx_pos];
      UDR0 = b;
      tx_pos++;
      if( tx_pos >= *tx_szs[p] ) {
         tx_pos = 0;
         *tx_szs[p] = 0;
         tx_size[0]--;
      }
   } else {
	   UCSR0B &= ~(1 << 5); /* disable send interrupt */
   }
   sei();
}

/* transmit interrupt 1 */
ISR(USART1_UDRE_vect) /* ready for more data to transmit */
{
   if (tx_size[1]) {
      UDR1 = tx_buf[1][(tx_head[1] - tx_size[1]--) % BUF_SZ];
   } else {
	   UCSR1B &= ~(1 << 5); /* disable send interrupt */
   }
}

/* transmit interrupt 2 */
ISR(USART2_UDRE_vect) /* ready for more data to transmit */
{
   if (tx_size[2]) {
      UDR2 = tx_buf[2][(tx_head[2] - tx_size[2]--) % BUF_SZ];
   } else {
	   UCSR2B &= ~(1 << 5); /* disable send interrupt */
   }
}

/* transmit interrupt 3 */
ISR(USART3_UDRE_vect) /* ready for more data to transmit */
{
   cli();
   if (tx_size[3]) {
      UDR3 = tx_buf[3][(tx_head[3] - tx_size[3]--) % BUF_SZ];
   } else {
	   UCSR3B &= ~(1 << 5); /* disable send interrupt */
   }
   sei();
}

/* determine if there is space for another byte in the transmit buffer */
uint8_t tx_ready(uint8_t port) {
   return tx_size[port] < BUF_SZ;
}

// transmit a byte without locking, for internal use
void tx_internal(uint8_t port, uint8_t b) {
   while (tx_size[port] >= BUF_SZ );

   ucsr[port][B] &= ~(1 << 5); /* diable send interrupt (locking) */
   tx_buf[port][tx_head[port]++] = b;
   tx_head[port] %= BUF_SZ;
   tx_size[port]++;
   /* done messing with buffer pointers */

   ucsr[port][B] |= (1 << 5); /* enable send interrupt */
}

/* put a byte in the transmit buffer. block until space available */
void tx_byte(uint8_t port, uint8_t b) {
   acquire_lock(tx_lock + port);

   tx_internal(port, b);

   release_lock(tx_lock + port);
}

/* transmit a series of bytes */
void tx_bytes(uint8_t port, const uint8_t * buf, uint16_t sz) {
   acquire_lock(tx_lock + port);
   uint16_t i;
   for( i=0; i<sz; i++ ) {
      tx_internal(port, buf[i]);
   }
   release_lock(tx_lock + port);
}

/* transmit an entire buffer
 * the bufsz will be set to 0 when transmit is complete */
void brain_tx_buffer(uint8_t * buf, uint16_t * bufsz) {
   acquire_lock(tx_lock + 0);

   while(tx_size[0] >= 4);

   ucsr[0][B] &= ~(1 << 5); /* diable send interrupt (locking) */

   tx_ptrs[tx_head[0]] = buf;
   tx_szs[tx_head[0]] = bufsz;
   tx_head[0]++;
   tx_head[0] %= 4;
   tx_size[0]++;

   ucsr[0][B] |= (1 << 5); /* enable send interrupt */

   release_lock(tx_lock + 0);
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

   /* lock init */
   tx_lock[port] = 0;
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
   // FIXME: deal with baud rates that are too high here
   *ubrr[port] = ubr;
}

/* stops the serial interrupts */
void serial_stop(uint8_t port)
{
	/* clear both interrupt enable bits */
   ucsr[port][B] &= ~( (1 << 5) | (1 << 7));
}

