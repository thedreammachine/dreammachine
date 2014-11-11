#!/usr/bin/env python

"""
voice_handler.py handles control flow for Turtlebot's
speech recognition. You can control the bot using
commands found in the corpus file.

publications:
    cmd_vel (geometry_msgs/Twist)    - order to move
    music_commands (std_msgs/String) - commands to music player
    ~voice_actions (std_msgs/String) - actions committed
services:
    ~start (std_srvs/Empty) - start voice commands
    ~stop (std_srvs/Empty) - stop voice commands
"""

import roslib; roslib.load_manifest('pocketsphinx')
import rospy
import math

from corpus_builder import CorpusBuilder
from voice_constants import *
from std_srvs.srv import *
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from dream_machine.msg import MusicCommand

class voice_handler:

    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        self.speed = 0.2
        self.msg = Twist()

        #publish to cmd_vel for movement and
        #publish to music commands for music control
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist)
        self.music_commands_pub = rospy.Publisher('music_commands', MusicCommand)
        self.voice_actions_pub = rospy.Publisher('~voice_actions', String)
        #subscribe to output for text of speech recognized
        rospy.Subscriber('recognizer/output', String, self.speechCb)
        #start handling voice commands
        rospy.Service("~start", Empty, self.start)
        #stop handling voice commands
        rospy.Service("~stop", Empty, self.stop)

        #Disable self on bootup
        self.started = False

        self.music_corpus = CorpusBuilder().read_song_directory()

        r = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            if self.started:
                self.cmd_vel_pub.publish(self.msg)
            r.sleep()
        
    def speechCb(self, msg):
        rospy.loginfo(msg.data)

        #Only handle voice commands if started
        if self.started:


            #Speed Commands
            if msg.data.find("full speed") > -1:
                if self.speed == 0.2:
                    self.msg.linear.x = self.msg.linear.x*2
                    self.msg.angular.z = self.msg.angular.z*2
                    self.speed = 0.4
                    self.voice_actions_pub.publish(VOICE_FULL)
            if msg.data.find("half speed") > -1:
                if self.speed == 0.4:
                    self.msg.linear.x = self.msg.linear.x/2
                    self.msg.angular.z = self.msg.angular.z/2
                    self.speed = 0.2
                    self.voice_actions_pub.publish(VOICE_HALF)
            #Movement Commands
            if msg.data.find("forward") > -1:    
                self.msg.linear.x = self.speed
                self.msg.angular.z = 0
                self.voice_actions_pub.publish(VOICE_FORWARD)
            elif msg.data.find("left") > -1:
                if self.msg.linear.x != 0:
                    if self.msg.angular.z < self.speed:
                        self.msg.angular.z += 0.05
                else:        
                    self.msg.angular.z = self.speed*2
                self.voice_actions_pub.publish(VOICE_LEFT)
            elif msg.data.find("right") > -1:    
                if self.msg.linear.x != 0:
                    if self.msg.angular.z > -self.speed:
                        self.msg.angular.z -= 0.05
                else:        
                    self.msg.angular.z = -self.speed*2
                self.voice_actions_pub.publish(VOICE_RIGHT)
            elif msg.data.find("back") > -1:
                self.msg.linear.x = -self.speed
                self.msg.angular.z = 0
                self.voice_actions_pub.publish(VOICE_BACK)
            elif msg.data.find("halt") > -1:          
                self.msg = Twist()
                self.voice_actions_pub.publish(VOICE_HALT)


            #Music Commands
            if msg.data in self.music_corpus:
                self.music_commands_pub.publish(
                  MusicCommand(MusicCommand.LOAD, [self.music_corpus[msg.data]]))
                self.music_commands_pub.publish(MusicCommand(MusicCommand.PLAY, []))
                self.voice_actions_pub.publish(msg.data)

            elif msg.data.find("play") > -1:
                self.music_commands_pub.publish(MusicCommand(MusicCommand.PLAY, []))
                self.voice_actions_pub.publish(VOICE_PLAY)
            elif msg.data.find("stop") > -1:
                self.music_commands_pub.publish(MusicCommand(MusicCommand.STOP, []))
                self.voice_actions_pub.publish(VOICE_STOP)
            elif msg.data.find("unpause") > -1:
                self.music_commands_pub.publish(MusicCommand(MusicCommand.UNPAUSE, []))
                self.voice_actions_pub.publish(VOICE_UNPAUSE)
            elif msg.data.find("pause") > -1:
                self.music_commands_pub.publish(MusicCommand(MusicCommand.PAUSE, []))
                self.voice_actions_pub.publish(VOICE_PAUSE)
            elif msg.data.find("volume up") > -1:
                self.music_commands_pub.publish(MusicCommand(MusicCommand.VOLUME_UP, []))
                self.voice_actions_pub.publish(VOICE_VOLUME_UP)
            elif msg.data.find("volume down") > -1:
                self.music_commands_pub.publish(MusicCommand(MusicCommand.VOLUME_DOWN, []))
                self.voice_actions_pub.publish(VOICE_VOLUME_DOWN)

            self.cmd_vel_pub.publish(self.msg)


    def cleanup(self):
        # stop the robot!
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def start(self, req):
        self.started = True
        rospy.loginfo("voice handler started")
        print "Started on voice handler"
        return EmptyResponse()

    def stop(self, req):
        self.started = False
        rospy.loginfo("voice handler stopped")
        print "Stopped on voice handler"
        return EmptyResponse()

if __name__=="__main__":
    rospy.init_node('voice_handler')
    try:
        voice_handler()
    except:
        pass
