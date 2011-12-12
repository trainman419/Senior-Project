#!/usr/bin/env python

import os
import time
import serial

port = '/dev/ttyACM0'
baud = 115200

def load(fname):
  os.system("avrdude -pm2560 -P%s -cstk500v2 -u -U flash:w:%s"%(port, fname))
  time.sleep(1)
  return

def ser_open(t=1):
  ser = serial.Serial(port, baud, timeout=t)
  time.sleep(2) # wait for arduino to boot
  ser.flushInput()
  return ser

#def pass():
#  print "Test passed"
#  exit(0)

def fail():
  print "Test fail"
  time.sleep(10)
  exit(1)
