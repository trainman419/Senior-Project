/*
 * publish.h
 *
 * A wrapper around my protocol to make sending messages easier.
 *
 * Author: Austin Hendrix
 */

#ifndef PUBLISH_H
#define PUBLISH_H

#include "protocol.h"
#include "drivers/serial.h"

template<int SZ> void publish(Packet<SZ> & p) {
   int sz = p.outsz();
   tx_buffer(BRAIN, (uint8_t*)p.outbuf(), sz);
}

#endif
