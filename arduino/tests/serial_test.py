#!/usr/bin/env python

import serial
import sys

if __name__ == '__main__':
  if len(sys.argv) != 3:
    print 'Usage: serial.py <port> <speed>'
    exit(1)

  ser = serial.Serial(sys.argv[1], sys.argv[2], timeout=1)
