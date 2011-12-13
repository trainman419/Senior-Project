#!/usr/bin/env python

import serial
import test

if __name__ == '__main__':
   test.load('locking.hex')

   ser = test.ser_open()
   good = 0

   while True:
      input = ser.read(26)
      if input != 'abcdefghijklmnopqrstuvwxyz' and input != 'ABCDEFGHIJKLMNOPQRSTUVWXYZ':
        print "\nInput mismatch %s"%input
        good = 0
      else:
        good += 1
        print "Good: %d\r"%good

