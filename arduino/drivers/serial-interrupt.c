#include "serial.h"
#include "led.h"

#include <avr/io.h>
#include <avr/interrupt.h>


/* recieve circular fifo (10 bytes total 20% overhead) */
extern uint8_t rx_head[4]; /* points to next writeable byte */
extern volatile uint8_t rx_size[4]; /* number of byts in buffer */
extern uint8_t rx_buf[4][BUF_SZ];

/* send circular fifo (10 bytes total, 20% overhead) */
extern uint8_t tx_head[4]; /* next writeable buffer */
extern volatile uint8_t tx_size[4]; /* number of buffers in queue */

extern const uint8_t * tx_ptrs[4][PTR_SZ];
extern uint16_t * tx_szs[4][PTR_SZ];
extern uint16_t tx_pos[4];

#define RX(port, udr, p) ISR(port) { \
   rx_buf[p][rx_head[p]++] = udr;\
   rx_head[p] %= BUF_SZ;\
   rx_size[p]++;\
}

RX(USART0_RX_vect, UDR0, 0);
RX(USART1_RX_vect, UDR1, 1);
RX(USART2_RX_vect, UDR2, 2);
RX(USART3_RX_vect, UDR3, 3);

/* send a pointer to the buffer to the transmit routine
 * rather than copying buffers around
 *
 * Implementation:
 * store a circular buffer of pointers to buffers and buffer sizes; when the
 * interrupt is done transmitting, set the buffer size to 0
 */

#define TX(port, ucsr, udr, pn) ISR(port) { \
   if( tx_size[pn]) { \
      uint8_t p = (PTR_SZ + tx_head[pn] - tx_size[pn]) % PTR_SZ; \
      uint8_t b = tx_ptrs[pn][p][tx_pos[pn]]; \
      udr = b; \
      tx_pos[pn]++; \
      if( tx_pos[pn] >= (*tx_szs[pn][p]) ) { \
         *tx_szs[pn][p] = 0; \
         tx_pos[pn] = 0; \
         tx_size[pn]--; \
      } \
   } else { \
	   ucsr &= ~(1 << 5); /* disable send interrupt */ \
   } \
} 

TX(USART0_UDRE_vect, UCSR0B, UDR0, 0);
TX(USART1_UDRE_vect, UCSR1B, UDR1, 1);
TX(USART2_UDRE_vect, UCSR2B, UDR2, 2);
TX(USART3_UDRE_vect, UCSR3B, UDR3, 3);
