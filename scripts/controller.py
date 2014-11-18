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
from std_srvs.srv import *
from mimetypes import types_map

class Controller:
    def __init__(self):
        # initialize action clients and publishers
        self.base_publisher = rospy.Publisher('/base_controller/command', Twist)
        self.twist_msg = Twist()

        # self.msg = Twist()
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist)
        self.music_commands_pub = rospy.Publisher('/music_commands', MusicCommand)

    def move_base(self, x, y, z):
        print "MOVING"

        self.twist_msg.linear = Vector3(x, 0.0, 0.0)
        self.twist_msg.angular = Vector3(0.0, 0.0, z)
        self.cmd_vel_pub.publish(self.twist_msg)

    def turn(self, rotation):
        self.twist_msg.linear = Vector3(0.0,0.0,0.0)
        self.twist_msg.angular = Vector3(0.0,0.0,rotation)
        self.cmd_vel_pub.publish(self.twist_msg)

    def start_music(self, song_filename):
        self.music_commands_pub.publish(MusicCommand(MusicCommand.LOAD, [song_filename]))

    def play_music(self, play_msg):
        print play_msg
        self.music_commands_pub.publish(MusicCommand(MusicCommand.PLAY, []))

    def stop_music(self, play_msg):
        print play_msg
        self.music_commands_pub.publish(MusicCommand(MusicCommand.STOP, []))

    def stop_voice_handler_client(self):
        try:
            stop = rospy.ServiceProxy('/voice_handler/stop', Empty)
            stop()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def start_voice_handler_client(self):
        try:
            print "Started the voice handler client..."
            start = rospy.ServiceProxy('/voice_handler/start', Empty)
            start()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def stop_voice_handler(self):
        self.stop_voice_handler_client()

    def start_voice_handler(self):
        self.start_voice_handler_client()


