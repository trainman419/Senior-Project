#include "serial.h"
#include <avr/io.h>
#include <avr/interrupt.h>


/* recieve circular fifo (10 bytes total 20% overhead) */
extern uint8_t rx_head[4]; /* points to next writeable byte */
extern volatile uint16_t rx_size[4]; /* number of byts in buffer */
extern uint8_t rx_buf[4][BUF_SZ];

/* send circular fifo (10 bytes total, 20% overhead) */
extern uint8_t tx_head[4]; /* next writeable byte */
extern volatile uint16_t tx_size[4]; /* number of bytes in buffer */
extern uint8_t tx_buf[4][BUF_SZ];
extern uint8_t tx_lock[4];


extern uint8_t * tx_ptrs[4];
extern uint16_t * tx_szs[4];
extern uint16_t tx_pos;


#define RX(port, udr, p) ISR(port) { \
   rx_buf[p][rx_head[p]++] = udr;\
   rx_head[p] %= BUF_SZ;\
   rx_size[p]++;\
}

RX(USART0_RX_vect, UDR0, 0);
RX(USART1_RX_vect, UDR1, 1);
RX(USART2_RX_vect, UDR2, 2);
RX(USART3_RX_vect, UDR3, 3);

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

