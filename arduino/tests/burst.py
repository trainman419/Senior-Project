#!/usr/bin/env python

import serial
import test

if __name__ == '__main__':
   test.load('burst_tx.hex')

   ser = test.ser_open(1)

   print "Waiting for input"
   rx_size = 10
   rx_old_size = 0
   rx_old_size_cnt = 0

   while rx_old_size_cnt < 5:
      input = ser.read(rx_size)
      rx_size = len(input)
      print "Got burst of size %d"%rx_size

      tx = [rx_size & 0xff, (rx_size >> 8) & 0xff]
      ser.write(str(bytearray(tx)))
      print "Sent response of %d"%rx_size

      if rx_size == rx_old_size:
         rx_old_size_cnt += 1
      else:
         rx_old_size = 0
      rx_old_size = rx_size
      rx_size += 10


   print "Maximum tx burst size: %d"%rx_size
   ser.close()

#
#   test.load('burst_rx.hex')
#
#   ser.open()
#
#   tx_size = 1
#   tx_old_size = 0
#   tx_old_size_cnt = 0
#
#   while tx_old_size_cnt < 5:
#      tx = range(tx_size)
#      ser.write(str(bytearray(tx)))
#
#      print "Sent burst of %d"%tx_size
#
#      input = ser.read(2)
#      if len(input) > 1:
#        tx_size = ord(input[0]) | (ord(input[1]) << 8)
#
#        print "Got %d"%tx_size
#
#        if tx_size == tx_old_size:
#           tx_old_size_cnt += 1
#        else:
#           tx_old_size_cnt = 0
#
#        tx_old_size = tx_size
#        tx_size += 1
#
#   print "Maximum rx burst size: %d"%tx_old_size
