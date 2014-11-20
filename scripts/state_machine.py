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
import voice_constants
from std_srvs.srv import *
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String
from dream_machine.msg import MusicCommand
from kobuki_msgs.msg import BumperEvent

class States:
    START = 'start'
    FIESTA = 'fiesta'
    GO_HOME = 'go_home'
    FOLLOW_TARGET = 'follow_target'
    SEARCH_TARGET = 'search_target'
    ENTERTAIN = 'entertain'
    GO_LOC = 'go_loc'

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
    RESET = 'reset'

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

        self.bumper_hit = False
        self.countdown = 0

        rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, self.bumper_callback)

    def state(self):
        return States.FIESTA

    def execute(self):
        if self.bumper_hit:
            # move backwards
            self.twist_message = Twist(Vector3(-self.forward_speed, 0.0, 0.0),
                Vector3(0.0, 0.0, 0.0))
            self.countdown = self.initial_countdown
            self.bumper_hit = False

        if self.countdown == 0:
            self.twist_message = self.twist_factory()
            self.countdown = self.initial_countdown

        self.pub.publish(self.twist_message)

        self.countdown -= 1

        return None

    def take_action(self, action):
        if action == Actions.END_FIESTA:
            return States.START
        else:
            return None

    def twist_factory(self):
        angular_speed = (random() * 2 - 1) * self.max_angular_speed
        velocity_msg = Twist(Vector3(self.forward_speed, 0.0, 0.0), Vector3(0.0, 0.0, angular_speed))
        return velocity_msg

    def bumper_callback(self, bumper_event):
        if bumper_event.state == 1:
            self.bumper_hit = True

class Start(EmptyState):
    def state(self):
        return States.START
    def take_action(self, action):
        if action == Actions.BEGIN_FIESTA:
            return States.FIESTA
        else:
            return None

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
        if action == Actions.RESET:
            # special case, go directly to start state.
            self.change_state(States.START)
        elif action:
            self.change_state(self.state.take_action(action))

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

    def speech_callback(self, msg):
        print 'sc'
        rospy.loginfo(msg.data)

        #Only handle voice commands if started
        if self.started:

            print msg.data
            if voice_constants.RESET in msg.data:
                self.state_machine.take_action(Actions.RESET)
            elif voice_constants.END_FIESTA in msg.data:
                self.state_machine.take_action(Actions.END_FIESTA)
            elif voice_constants.BEGIN_FIESTA in msg.data:
                self.state_machine.take_action(Actions.BEGIN_FIESTA)

            #Music Commands
            if msg.data in self.music_corpus:
                self.music_commands_pub.publish(
                  MusicCommand(MusicCommand.LOAD, [self.music_corpus[msg.data]]))
                self.music_commands_pub.publish(MusicCommand(MusicCommand.PLAY, []))
                self.voice_actions_pub.publish(msg.data)

            elif voice_constants.PLAY in msg.data:
                self.music_commands_pub.publish(MusicCommand(MusicCommand.PLAY, []))
                self.voice_actions_pub.publish(voice_constants.PLAY)
            elif voice_constants.STOP in msg.data:
                self.music_commands_pub.publish(MusicCommand(MusicCommand.STOP, []))
                self.voice_actions_pub.publish(voice_constants.STOP)
            elif voice_constants.UNPAUSE in msg.data:
                self.music_commands_pub.publish(MusicCommand(MusicCommand.UNPAUSE, []))
                self.voice_actions_pub.publish(voice_constants.UNPAUSE)
            elif voice_constants.PAUSE in msg.data:
                self.music_commands_pub.publish(MusicCommand(MusicCommand.PAUSE, []))
                self.voice_actions_pub.publish(voice_constants.PAUSE)
            elif voice_constants.VOLUME_UP in msg.data:
                self.music_commands_pub.publish(MusicCommand(MusicCommand.VOLUME_UP, []))
                self.voice_actions_pub.publish(voice_constants.VOLUME_UP)
            elif voice_constants.VOLUME_DOWN in msg.data:
                self.music_commands_pub.publish(MusicCommand(MusicCommand.VOLUME_DOWN, []))
                self.voice_actions_pub.publish(voice_constants.VOLUME_DOWN)

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
