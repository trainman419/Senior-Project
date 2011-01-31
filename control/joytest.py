#!/usr/bin/env python

import time
import pygame

pygame.init()

pygame.joystick.init()

print "Detected " + str(pygame.joystick.get_count()) + " joysticks"

if pygame.joystick.get_count > 0:
   joystick = pygame.joystick.Joystick(0)
   joystick.init()

   print "Joystick " + joystick.get_name()
   numaxes = joystick.get_numaxes()
   print "Joystick has " + str(numaxes) + " axes"
   

   while True:
      time.sleep(0.1)
      pygame.event.pump()
      for i in range(numaxes):
         print "Axis " + str(i) + ": " + str(joystick.get_axis(i))
