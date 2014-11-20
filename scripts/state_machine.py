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
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String
from dream_machine.msg import MusicCommand

class EmptyState:
    def execute(self):
        pass
    def cleanup(self):
        pass
    def take_action(self, action):
        return None

class Fiesta(EmptyState):
    def __init__(self, pub):
        self.pub = pub
        self.forward_speed = 0.1
        self.max_angular_speed = 1.0
        self.initial_countdown = 30

        self.countdown = 0

    def state(self):
        return States.FIESTA

    def execute(self):
        if self.countdown == 0:
            self.twist_message = self.twist_factory()
            self.countdown = self.initial_countdown

        self.pub.publish(self.twist_message)

        self.countdown -= 1

        return None

    def twist_factory(self):
        angular_speed = (random() * 2 - 1) * self.max_angular_speed
        velocity_msg = Twist(Vector3(self.forward_speed, 0.0, 0.0), Vector3(0.0, 0.0, angular_speed))
        return velocity_msg

    def take_action(self, action):
        if action == Actions.END_FIESTA:
            return States.START
        else:
            return None

class Start(EmptyState):
    def state(self):
        return States.START
    def take_action(self, action):
        if action == Actions.BEGIN_FIESTA:
            return States.FIESTA
        else:
            return None

class Actions:
    BEGIN_FIESTA = 'begin_fiesta'
    END_FIESTA = 'end_fiesta'
    ARRIVED_START_LOC = 'arrived_start'
    BEGIN_FOLLOW = 'begin_follow'
    END_FOLLOW = 'end_follow'
    TARGET_LOST = 'target_lost'
    TARGET_LOCATED = 'target_located'
    END_SONG = 'end_song'
    ARRIVED_GOAL_LOC = 'arrived_goal'
    BEGIN_LOC = 'begin_loc'


class States:
    START = 'start'
    FIESTA = 'fiesta'
    GO_HOME = 'go_home'
    FOLLOW_TARGET = 'follow_target'
    SEARCH_TARGET = 'search_target'
    ENTERTAIN = 'entertain'
    GO_LOC = 'go_loc'

class StateMachine:
    def __init__(self):
        # TODO change back to START
        print 'start init state_machine'
        self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        self.r = rospy.Rate(10.0)

        self.state = self.state_factory(States.FIESTA)
        print 'end init state_machine'

    def state_factory(self, state):
        if state == States.START:
            return Start()
        elif state == States.FIESTA:
            return Fiesta(self.pub)
        elif state == States.GO_HOME:
            return None

    def run(self):
        while not rospy.is_shutdown():
            self.loop()
            self.r.sleep()

    def loop(self):
        new_state = self.state.execute()
        self.change_state(new_state)

    def change_state(self, new_state):
        if new_state and new_state != self.state.state():
            self.state.cleanup()
            self.state = self.state_factory(new_state)

    def take_action(self, action):
        if action:
            change_state(self.state.take_action(action))

class VoiceHandler:

    def __init__(self, state_machine):
        self.state_machine = state_machine
        # publish to music commands for music control
        self.music_commands_pub = rospy.Publisher('music_commands', MusicCommand)
        self.voice_actions_pub = rospy.Publisher('~voice_actions', String)
        #subscribe to output for text of speech recognized
        rospy.Subscriber('/recognizer/output', String, self.speech_callback)
        #start handling voice commands
        rospy.Service("~start", Empty, self.start)
        #stop handling voice commands
        rospy.Service("~stop", Empty, self.stop)

        #Disable self on bootup
        # TODO change back
        self.started = True

        self.music_corpus = CorpusBuilder().read_song_directory()
        print 'done initializing vh'

    def speech_callback(self, msg, a):
        print 'sc'
        rospy.loginfo(msg.data)

        #Only handle voice commands if started
        if self.started:

            print msg.data
            if msg.data.find("halt") > -1:
                state_machine.take_action(Actions.END_FIESTA)
            if msg.data.find("fiesta") > -1:
                state_machine.take_action(Actions.BEGIN_FIESTA)

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
    sm = StateMachine()
    vh = VoiceHandler(sm)

    # sm.run starts the loop. put everything before this.
    sm.run()
