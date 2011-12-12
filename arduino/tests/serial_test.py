#!/usr/bin/env python

import sys
import time

# import test utilities and configuration
import test

if __name__ == '__main__':

  # load test program onto arduino
  test.load('serial_test.hex')

  # open serial port
  ser = test.ser_open(1)

  output = [ 0 ]
  for i in range(256):
    #time.sleep(0.1)
    output[0] = i
    print "Sending byte %d"%i
    ser.write(str(bytearray(output)))
    inc = ser.read()
    if len(inc) != 1:
      print "Receive byte timed out"
      test.fail()
    else:
      print "Got byte %d"%ord(inc[0])
      if ord(inc[0]) != output[0]:
        print "Got a byte I wasn't expecting"
        test.fail()

#  test.pass()
