#!/usr/bin/env python
import test
import sys

if __name__ == '__main__':
  if len(sys.argv) == 2:
    test.load(sys.argv[1])
  else:
    print 'Usage: load.py <filename>'
