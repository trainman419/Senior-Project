#!/usr/bin/env python

import serial
import sys

# import test utilities and configuration
import test

if __name__ == '__main__':

  # load test program onto arduino
  test.load('serial.hex')

  # open serial port
  ser = serial.Serial(test.port, test.baud, timeout=1)

  output = [ 0 ]
  for i in range(256):
    output[0] = i
    ser.write(str(bytearray(output)))
    inc = ser.read()
    if len(inc) != 1:
      print "Receive byte timed out"
      test.fail()
    if inc[0] != output[0]:
      print "Got a byte I wasn't expecting"
      test.fail()

#  test.pass()
