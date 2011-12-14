#!/usr/bin/env python

import serial
import test

if __name__ == '__main__':
   test.load('locking.hex')

   ser = test.ser_open()
   lower = 0
   upper = 0

   while True:
      input = ser.read(26)
      if input == "`zyxwvutsrqponmlkjihgfedcb":
        lower += 1
      elif input == "@ABCDEFGHIJKLMNOPQRSTUVWXY":
        upper += 1
      else:
        print "\nInput mismatch %s"%input
        lower = 0
        upper = 0
      print "\rUpper %d, Lower %d"%(upper, lower),
