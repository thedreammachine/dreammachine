#!/usr/bin/env python

import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('actionlib')
roslib.load_manifest('actionlib_msgs')
roslib.load_manifest('geometry_msgs')

import rospy
import threading

from std_msgs.msg import String
from dream_machine.msg import MusicCommand

from actionlib import SimpleActionClient
from sensor_msgs.msg import JointState

from geometry_msgs.msg import (
    Twist,
    Vector3,
    Point
)

import time
import json
import math
import os
import urlparse
from mimetypes import types_map

class Controller:
    def __init__(self):
        # initialize action clients and publishers
        self.base_publisher = rospy.Publisher('/base_controller/command', Twist)

        # self.msg = Twist()
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist)
        self.music_commands_pub = rospy.Publisher('/music_commands', MusicCommand)

    def move_base(self, x, y, z):
        print "MOVING"

        twist_msg = Twist()
        twist_msg.linear = Vector3(x, 0.0, 0.0)
        twist_msg.angular = Vector3(0.0, 0.0, z)
        self.cmd_vel_pub.publish(twist_msg)
        # self.base_publisher.publish(twist_msg)

    def turn(self, rotation):
        twist_msg = Twist()
        twist_msg.linear = Vector3(0.0,0.0,0.0)
        twist_msg.angular = Vector3(0.0,0.0,rotation)
        # self.base_publisher.publish(twist_msg)
        self.cmd_vel_pub.publish(twist_msg)

    def start_music(self, song_filename):
        self.music_commands_pub.publish(MusicCommand(MusicCommand.LOAD, [song_filename]))

    def play_music(self, play_msg):
        print play_msg
        self.music_commands_pub.publish(MusicCommand(MusicCommand.PLAY, []))

    def stop_music(self, play_msg):
        print play_msg
        self.music_commands_pub.publish(MusicCommand(MusicCommand.STOP, []))


