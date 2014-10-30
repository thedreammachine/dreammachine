#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
import random
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import PointCloud2
from ar_track_alvar.msg import AlvarMarkers
import math

def get_angle(point):
    return math.atan2(point.y, point.z)

def get_distance(point):
    return math.sqrt(point.x*point.x + point.y*point.y + point.z*point.z)

stale = True
turn_angle = None
distance = None

def alvar_callback(marker):
    if (len(marker.markers) ==  0):
      return

    curr_marker = marker.markers[0]
    print curr_marker


    global stale
    global turn_angle
    global distance

    turn_angle = get_angle(curr_marker.pose.pose.position)
    distance = get_distance(curr_marker.pose.pose.position)

    stale = False

    print "angle: ", turn_angle
    print "distance: ", distance

def follower():
    global stale

    pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

    rospy.init_node('follower', anonymous=True)

    rospy.Subscriber("/ar_pose_marker", AlvarMarkers, alvar_callback)

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
      if stale:
        r.sleep()
        continue

      stale = True

      if turn_angle:
        if (distance > 0.5):
          forward = 0.15
        elif (distance < 0.4):
          forward = -0.15
        else:
          forward = 0.0
        msg = Twist(Vector3(forward, 0.0, 0.0), Vector3(0.0, 0.0, 2*turn_angle))
      else:
        angle = 0.5
        msg = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.5))
      pub.publish(msg)
      r.sleep()

if __name__ == '__main__':
    try:
        follower()
    except rospy.ROSInterruptException: pass


