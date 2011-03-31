/* serial.h
 *
 * header file for serial function for ATMega32
 *
 * Author: Austin Hendrix
 */

#ifndef SERIAL_H
#define SERIAL_H

#include <avr/io.h>

//extern volatile uint16_t rx_size[4];
//extern volatile uint16_t tx_size[4];
//extern uint8_t tx_lock[4];

/* setup and enable serial interrupts */
void serial_init(uint8_t port);

/* initialize serial tx */
void serial_init_tx(uint8_t port);

/* initialize serial rx */
void serial_init_rx(uint8_t port);

/* set serial baud rate */
void serial_baud(uint8_t port, uint32_t baud);

/* stops the serial interrupts */
void serial_stop(uint8_t port);

/* put a byte in the transmit buffer. block until space available */
void tx_byte(uint8_t port, uint8_t b);

/* transmit a series of bytes */
void tx_bytes(uint8_t port, const uint8_t * buf, uint16_t sz);

/* determine if there is space for another byte in the transmit buffer */
uint8_t tx_ready(uint8_t port);

/* determine if there is data in the rx buffer */
uint8_t rx_ready(uint8_t port);

/* get a byte from recieve buffer. block until data recieved */
uint8_t rx_byte(uint8_t port);

#endif
