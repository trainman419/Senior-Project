#!/usr/bin/env python

import roslib; roslib.load_manifest('goal_list')

import rospy

from sensor_msgs.msg import NavSatFix


if __name__ == '__main__':
   rospy.init_node('adjust_goals')

   if rospy.has_param('start') and rospy.has_param('goals_rel'):
      if 'gps' in rospy.get_published_topics():
         try:
            # get the current position
            gps = rospy.wait_for_message('gps', NavSatFix, 10.0)
            start = rospy.get_param('start')
            goals_rel = rospy.get_param('goals_rel')

            # assume that our reading was taken at start, and has some error
            # start = gps + offset
            # offset = start - gps
            lat_offset = start[0] - gps.latitude
            lon_offset = start[1] - gps.longitude
            goals = []
            for g in goals_rel:
               g[0] += lat_offset
               g[1] += lon_offset
               goals.append(g)
            rospy.set_param('goals', goals)
         except ROSException as e:
            rospy.logerr("GPS Timeout exceeded")
      else:
         rospy.logerr("gps topic is not published")
   else:
      rospy.logerr("start and goals_rel parameters are not set")
