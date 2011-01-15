#!/usr/bin/env python

import roslib; roslib.load_manifest('laser_view');
import rospy
from sensor_msgs.msg import LaserScan

import pygame
import math

BLACK = (0, 0, 0)
GREEN = (0, 255, 0)
RED = (255, 0 ,0)

width = 500
height = 400

center_x = width/2
center_y = 3 * height/4

scale = 40

pygame.init()
windowSurface = pygame.display.set_mode((width, height), 0, 32)

def callback(data):
   #rospy.loginfo(rospy.get_name()+" Received scan data. angle min: %f angle max: %f", data.angle_min, data.angle_max)
   windowSurface.fill(BLACK)
   for i in range(len(data.ranges)):
      theta = data.angle_min + (i * data.angle_increment)
      y = scale * data.ranges[i] * math.cos(theta);
      x = scale * data.ranges[i] * math.sin(theta);
      x = center_x - x
      y = center_y - y
      windowSurface.set_at((x, y), GREEN)

   pygame.draw.line(windowSurface, RED, (center_x, 0), (center_x, height-1))


   pygame.display.update()

def viewer():
   pygame.display.set_caption('Laser Viewer')

   rospy.init_node('viewer', anonymous=True)
   rospy.Subscriber("scan", LaserScan, callback)
   rospy.spin()

if __name__ == '__main__':
   viewer()

