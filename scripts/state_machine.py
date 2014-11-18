#!/usr/bin/env python

"""
voice_handler.py handles control flow for Turtlebot's
speech recognition. You can control the bot using
commands found in the corpus file.

listens to:
    /recognizer/output - incoming voice commands
publications:
    /mobile_base/commands/velocity (geometry_msgs/Twist)    - order to move
    music_commands (std_msgs/String) - commands to music player
    ~voice_actions (std_msgs/String) - actions committed
services:
    ~start (std_srvs/Empty) - start voice commands
    ~stop (std_srvs/Empty) - stop voice commands
"""

import roslib; roslib.load_manifest('pocketsphinx')
import rospy
import math
from random import random

from corpus_builder import CorpusBuilder
from voice_constants import *
from std_srvs.srv import *
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from dream_machine.msg import MusicCommand

class Fiesta:
    def __init__(self, pub):
        self.pub = pub
        self.forward_speed = 0.2
        self.max_angular_speed = 0.1

        self.countdown = 0

    def state(self):
        return StateMachine.FIESTA

    def execute(self):
        if self.countdown == 0:
            self.twist_message = twist_factory()
            self.countdown = 10

        self.pub.publish(self.twist_message)

        self.countdown -= 1

        return None

    def cleanup(self):
        pass

    def twist_factory(self):
        angular_speed = (random() * 2 - 1) * self.angular_speed
        fiesta_msg = twist_factory(self.forward_speed, angular_speed)
        velocity_msg = Twist(Vector3(forward_speed, 0.0, 0.0), Vector3(0.0, 0.0, angular_speed))
        return velocity_msg

class StateMachine:
    START = 'start'
    FIESTA = 'fiesta'
    GO_HOME = 'go_home'

    def __init__(self):
        # TODO change back to START
        self.state = state_factory(StateMachine.FIESTA)

        self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        self.r = rospy.Rate(10.0)

    def state_factory(self, state):
        if state == StateMachine.START:
            return None
        elif state == StateMachine.FIESTA:
            return Fiesta(self.pub)
        elif state == StateMachine.GO_HOME:
            return None

    def run(self):
        while not rospy.is_shutdown():
            self.loop()
            r.sleep()

    def loop(self):
        new_state = self.state.execute()
        if new_state:
            self.state.cleanup()
            self.state = self.state_factory(new_state)

class VoiceHandler:

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
        rospy.Subscriber('recognizer/output', String, self.speech_callback)
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
        
    def speech_callback(self, msg):
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
    rospy.init_node('state_machine')
    try:
        StateMachine().run()
    except:
        pass