#!/usr/bin/env python

import time
import serial
import struct
import pygame
import math
import operator

BLACK = (0, 0, 0)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
BLUE = (0, 0, 255)

width = 500
height = 400

center_x = width/2
center_y = 3 * height/4

scale = 1

pygame.init()
pygame.display.set_caption('Debug and control')
windowSurface = pygame.display.set_mode((width, height), 0, 32)

# set repeat rate
pygame.key.set_repeat(200, 50)

speed = 0
steer = 127

def process(data):
   windowSurface.fill(BLACK)
   for i in range(len(data)):
      theta = (-math.pi/2.0) + (i * (math.pi/512.0))
      y = scale * data[i] * math.cos(theta);
      x = scale * data[i] * math.sin(theta);
      x = center_x - x
      y = center_y - y
      windowSurface.set_at((x, y), GREEN)

   # line, pointing forward
   pygame.draw.line(windowSurface, RED, (center_x, 0), (center_x, height-1))

   # box describing the robot
   pygame.draw.line(windowSurface, BLUE, (center_x - 10, center_y - 15), (center_x + 10, center_y - 15))
   pygame.draw.line(windowSurface, BLUE, (center_x - 10, center_y - 15), (center_x - 10, center_y + 30))
   pygame.draw.line(windowSurface, BLUE, (center_x + 10, center_y - 15), (center_x + 10, center_y + 30))
   pygame.draw.line(windowSurface, BLUE, (center_x - 10, center_y + 30), (center_x + 10, center_y + 30))


   pygame.display.update()

#ser = serial.Serial('/dev/rfcomm0', timeout = None )
ser = serial.Serial('/dev/tty.FireFly-01DF-SPP-1', timeout = 0.1 )

running = True

while running:
   # read serial data
   start = ser.read()
   if start == 'L' :
      ser.timeout = None
      data = ser.read(512)
      # convert string to tuple/array of integers
      ranges = struct.unpack("512B", data)
      end = ser.read()
      drop = 0
      while end != '\r' and end != '\n':
#         print "Dropping " + end
         end = ser.read()
         drop += 1
         if (drop%100) == 0:
            print "Dropping " + str(drop) + " bytes"
#if drop < 5:
      print "Dropped " + str(drop) + " bytes"
      process(ranges)
      ser.timeout = 0.1

   # process user input
   for event in pygame.event.get():
      if event.type == pygame.QUIT:
         pygame.quit()
         running = False
      if event.type == pygame.KEYDOWN:
         if event.key == ord('w'):
            speed += 5
            if speed > 50:
               speed = 50
   
            outbuf = "M" + struct.pack('b', speed) + "\r"
            ser.write(outbuf)
            print "Speed " + str(speed)
   
         if event.key == ord('s'):
            speed -= 5
            if speed < -50:
               speed = -50
   
            outbuf = "M" + struct.pack('b', speed) + "\r"
            ser.write(outbuf)
            print "Speed " + str(speed)
   
         if event.key == ord('a'):
            steer -= 10
            if steer < 0:
               steer = 0
   
            outbuf = "S" + struct.pack('B', steer) + "\r"
            ser.write(outbuf)
            print "Steer " + str(steer)
   
         if event.key == ord('d'):
            steer += 10
            if steer > 255:
               steer = 255
   
            outbuf = "S" + struct.pack('B', steer) + "\r"
            ser.write(outbuf)
            print "Steer " + str(steer)
   
         if event.key == ord(' '):
            speed = 0
            steer = 127
   
            outbuf = "S" + struct.pack('B', steer) + "\r"
            ser.write(outbuf)
            print "Steer " + str(steer)
   
   
            outbuf = "M" + struct.pack('b', speed) + "\r"
            ser.write(outbuf)
            print "Speed " + str(speed)
         if event.key == ord('q'):
            ser.write("ZZZZZZZZZ\r")
            print "Sent shutdown command"
   
