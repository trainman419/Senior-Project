#!/usr/bin/env python

import serial
import struct
import math
import operator

ser = serial.Serial('/dev/rfcomm0', timeout = None )

while True:
   s = ser.read()
   b = struct.unpack("B", s)
   if b[0] > 4 :
      print b[0]
