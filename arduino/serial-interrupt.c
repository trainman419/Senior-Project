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

extern uint8_t * tx_ptrs[4][PTR_SZ];
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

#define TX(port, ucsr, udr, pn) ISR(port) { \
   cli(); \
   if( tx_size[pn]) { \
      uint8_t p = (tx_head[pn] - tx_size[pn]) % PTR_SZ; \
      uint8_t b = tx_ptrs[pn][p][tx_pos[pn]]; \
      udr = b; \
      tx_pos[pn]++; \
      if( tx_pos[pn] >= *tx_szs[pn][p] ) { \
         tx_pos[pn] = 0; \
         *tx_szs[pn][p] = 0; \
         tx_size[pn]--; \
      } \
   } else { \
	   ucsr &= ~(1 << 5); /* disable send interrupt */ \
   } \
   sei(); \
} 
// totals to 89 instructions; probably 130-150 cycles to execute

TX(USART0_UDRE_vect, UCSR0B, UDR0, 0);
TX(USART1_UDRE_vect, UCSR1B, UDR1, 1);
TX(USART2_UDRE_vect, UCSR2B, UDR2, 2);
TX(USART3_UDRE_vect, UCSR3B, UDR3, 3);

///* transmit interrupt 0 */
//ISR(USART0_UDRE_vect) /* ready for more data to transmit */
//{
//   cli();
//   if( tx_size[0]) {
//      uint8_t p = (tx_head[0] - tx_size[0]) % PTR_SZ;
//      uint8_t b = tx_ptrs[0][p][tx_pos[0]];
//      UDR0 = b;
//      tx_pos[0]++;
//      if( tx_pos[0] >= *tx_szs[0][p] ) {
//         tx_pos[0] = 0;
//         *tx_szs[0][p] = 0;
//         tx_size[0]--;
//      }
//   } else {
//	   UCSR0B &= ~(1 << 5); /* disable send interrupt */
//   }
//   sei();
//   // totals to 89 instructions; probably 130-150 cycles to execute
//}
//
///* transmit interrupt 1 */
//ISR(USART1_UDRE_vect) /* ready for more data to transmit */
//{
//   cli();
//   if( tx_size[1]) {
//      uint8_t p = (tx_head[1] - tx_size[1]) % PTR_SZ;
//      uint8_t b = tx_ptrs[1][p][tx_pos[1]];
//      UDR0 = b;
//      tx_pos[1]++;
//      if( tx_pos[1] >= *tx_szs[1][p] ) {
//         tx_pos[1] = 0;
//         *tx_szs[1][p] = 0;
//         tx_size[1]--;
//      }
//   } else {
//	   UCSR1B &= ~(1 << 5); /* disable send interrupt */
//   }
//   sei();
//}
//
///* transmit interrupt 2 */
//ISR(USART2_UDRE_vect) /* ready for more data to transmit */
//{
//   cli();
//   if( tx_size[2]) {
//      uint8_t p = (tx_head[2] - tx_size[2]) % PTR_SZ;
//      uint8_t b = tx_ptrs[2][p][tx_pos[2]];
//      UDR0 = b;
//      tx_pos[2]++;
//      if( tx_pos[2] >= *tx_szs[2][p] ) {
//         tx_pos[2] = 0;
//         *tx_szs[2][p] = 0;
//         tx_size[2]--;
//      }
//   } else {
//	   UCSR2B &= ~(1 << 5); /* disable send interrupt */
//   }
//   sei();
//}
//
///* transmit interrupt 3 */
//ISR(USART3_UDRE_vect) /* ready for more data to transmit */
//{
//   cli();
//   if( tx_size[3]) {
//      uint8_t p = (tx_head[3] - tx_size[3]) % PTR_SZ;
//      uint8_t b = tx_ptrs[3][p][tx_pos[3]];
//      UDR0 = b;
//      tx_pos[3]++;
//      if( tx_pos[3] >= *tx_szs[3][p] ) {
//         tx_pos[3] = 0;
//         *tx_szs[3][p] = 0;
//         tx_size[3]--;
//      }
//   } else {
//	   UCSR3B &= ~(1 << 5); /* disable send interrupt */
//   }
//   sei();
//}
//
