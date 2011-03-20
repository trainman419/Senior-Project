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

width = 1000
height = 800

center_x = width/2
center_y = 3 * height/4

scale = 2

pygame.init()
pygame.display.set_caption('Debug and control')
windowSurface = pygame.display.set_mode((width, height), 0, 32)

# set repeat rate
pygame.key.set_repeat(200, 50)

# joystick setup
pygame.joystick.init()

joystick_present = False
if pygame.joystick.get_count() > 0:
   joystick = pygame.joystick.Joystick(0)
   joystick.init()
   joystick_present = True
   print "Joystick detected: " + joystick.get_name()

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
   pygame.draw.line(windowSurface, BLUE, (center_x - 10*scale, center_y - 15*scale), (center_x + 10*scale, center_y - 15*scale))
   pygame.draw.line(windowSurface, BLUE, (center_x - 10*scale, center_y - 15*scale), (center_x - 10*scale, center_y + 30*scale))
   pygame.draw.line(windowSurface, BLUE, (center_x + 10*scale, center_y - 15*scale), (center_x + 10*scale, center_y + 30*scale))
   pygame.draw.line(windowSurface, BLUE, (center_x - 10*scale, center_y + 30*scale), (center_x + 10*scale, center_y + 30*scale))


   pygame.display.update()

#ser = serial.Serial('/dev/rfcomm0', timeout = None )
ser = serial.Serial('/dev/tty.FireFly-01DF-SPP-1', timeout = 0.1 )

running = True

while running:
#   print "Loop start"
   # read serial data
   start = ser.read()
   if start == 'L' :
#      print "Read serial data"
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
#         if (drop%100) == 0:
#            print "Dropping " + str(drop) + " bytes"
#      print "Dropped " + str(drop) + " bytes"
      process(ranges)
      ser.timeout = 0.1

   # process user input
#   if joystick_present:
#      print "Joystick processing"
#      pygame.event.pump()
#      steer = int(joystick.get_axis(0)*127 + 127)
#      speed = int(joystick.get_axis(1)*50)
#      if speed < 0 :
#         speed += 256
#      print "Steer: " + str(steer) + ", speed: " + str(speed)
#  
##      outbuf = "S" + struct.pack('B', steer) + "\r"
#      outbuf = "S" + chr(steer) + "\r"
#      print "Write steering: " + outbuf
#      ser.write(outbuf)
#   
##      outbuf = "M" + struct.pack('b', speed) + "\r"
#      outbuf = "M" + chr(speed) + "\r"
#      print "Write speed: " + outbuf
#      ser.write(outbuf)
#      time.sleep(0.1)
#   else:
   for event in pygame.event.get():
      if event.type == pygame.QUIT:
         pygame.quit()
         running = False
      if event.type == pygame.KEYDOWN:
         if event.key == ord('w'):
            speed += 5
            if speed > 100:
               speed = 100

            outbuf = "M" + struct.pack('b', speed) + "\r"
            ser.write(outbuf)
            print "Speed " + str(speed)

         if event.key == ord('s'):
            speed -= 5
            if speed < -100:
               speed = -100

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
      if event.type == pygame.JOYAXISMOTION:
#         print "Joystick moved"
         if event.axis == 0:
#            print repr(event)
            steer = int(event.value*127 + 127)

            outbuf = "S" + struct.pack('B', steer) + "\r"
            ser.write(outbuf)
            print "Steer " + str(steer)
         if event.axis == 1:
#            print repr(event)
            speed = int(event.value*(-50))

            outbuf = "M" + struct.pack('b', speed) + "\r"
            ser.write(outbuf)
            print "Speed " + str(speed)
