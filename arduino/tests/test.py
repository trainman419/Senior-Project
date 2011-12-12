#!/usr/bin/env python

import os

port = '/dev/ttyACM0'
baud = 115200

def load(fname):
  os.system("avrdude -pm2560 -P%s -cstk500v2 -u -U flash:w:%s"%(port, fname))
  return

#def pass():
#  print "Test passed"
#  exit(0)

def fail():
  print "Test fail"
  exit(1)
