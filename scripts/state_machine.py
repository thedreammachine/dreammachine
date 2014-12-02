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
from ar_track_alvar.msg import AlvarMarkers

# For Location Mode
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, PoseStamped, Point
from std_msgs.msg import Header, ColorRGBA
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import time
import pickle
import sys

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
    BEGIN_LOCATION = 'begin_loc'
    END_LOCATION = 'end_loc'
    RESET = 'reset'

class EmptyState:
    def execute(self):
        pass
    def cleanup(self):
        pass
    def take_action(self, action):
        return None

class Fiesta(EmptyState):
    def __init__(self, mobile_base_pub, music_commands_pub):
        self.mobile_base_pub = mobile_base_pub
        self.music_commands_pub = music_commands_pub
        self.forward_speed = 0.1
        self.max_angular_speed = 1.0
        self.initial_countdown = 30

        self.bumper_hit = False
        self.countdown = 0

        rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, self.bumper_callback)

        self.music_commands_pub.publish(MusicCommand(MusicCommand.LOAD, ["Flux Pavilion - I Cant Stop.wav"]))
        self.music_commands_pub.publish(MusicCommand(MusicCommand.PLAY, []))

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

        self.mobile_base_pub.publish(self.twist_message)

        self.countdown -= 1

        return None

    def cleanup(self):
        self.music_commands_pub.publish(MusicCommand(MusicCommand.STOP, []))

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

class SearchTarget(EmptyState):
    def __init__(self, cmd_vel_pub):
        self.cmd_vel_pub = cmd_vel_pub
        self.ar_pose_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.ar_callback)
        self.target_acquired = False

        print "FOLLOW STARTED!"

    def cleanup(self):
        self.ar_pose_sub.unregister()

    def state(self):
        return States.SEARCH_TARGET

    def ar_callback(self, message):
        if (len(message.markers) > 0):
            self.target_acquired = True

    def execute(self):
        if self.target_acquired:
            return States.FOLLOW_TARGET
        # TODO do some sort of interesting thing to move around and look for the
        # target
        self.cmd_vel_pub.publish(Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.3)))

    def take_action(self, action):
        if action == END_FOLLOW:
            return States.START
        else:
            return None

class FollowTarget(EmptyState):
    def __init__(self, cmd_vel_pub):
        self.cmd_vel_pub = cmd_vel_pub
        self.ar_pose_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.ar_callback)

        self.stale = True
        self.turn_angle = None
        self.distance = None
        self.lost_target_counter = 0

        # no sightings for a full second
        self.target_considered_lost = 10

    def cleanup(self):
        self.ar_pose_sub.unregister()
    def state(self):
        return States.FOLLOW_TARGET

    def get_angle(self, point):
        return math.atan2(point.y, point.z)

    def get_distance(self, point):
        return math.sqrt(point.x*point.x + point.y*point.y + point.z*point.z)

    def ar_callback(self, message):
        if (len(message.markers) ==  0):
          return
        self.lost_target_counter = 0
        curr_marker = message.markers[0]
        self.turn_angle = self.get_angle(curr_marker.pose.pose.position)
        self.distance = self.get_distance(curr_marker.pose.pose.position)
        self.stale = False

    def execute(self):
        if self.stale:
            self.lost_target_counter += 1
            if self.lost_target_counter >= self.target_considered_lost:
                # target considered lost, move to search mode
                return States.SEARCH_TARGET
            else:
                # do nothing this iteration
                return

        self.stale = True

        if self.turn_angle:
          if (self.distance > 0.5):
            forward = 0.15
          elif (self.distance < 0.4):
            forward = -0.15
          else:
            forward = 0.0
          msg = Twist(Vector3(forward, 0.0, 0.0), Vector3(0.0, 0.0, 2*self.turn_angle))
        else:
          angle = 0.5
          msg = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.5))
        self.cmd_vel_pub.publish(msg)

    def take_action(self, action):
        if action == END_FOLLOW:
            return States.START
        else:
            return None

region_radius = 1.0
NULL_STRING_COMMAND = "null"
LOCATION_FILE_NAME = '/home/motionlab/catkin_ws/src/dream_machine/location_info.txt'

def sign(x):
    if x < 0:
        return -1.0
    elif x > 0:
        return 1.0
    else:
        return 0.0

def get_pose_stamped(x, y, theta):
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'map'

    point = Point(x, y, 0.0)
    quaternion = Quaternion(*quaternion_from_euler(0.0, 0.0, theta))
    pose = Pose(point, quaternion)

    return PoseStamped(header, pose)

def get_x_y_theta(trans, rot):
    _, _, yaw = euler_from_quaternion(rot)
    x, y, _ = trans
    return x, y, yaw

class Location(EmptyState):
    def __init__(self, cmd_pose_pub, cmd_music_pub):
        self.tf_listener = tf.TransformListener()
        self.base_frame = '/map'
        self.odom_frame = '/base_link'

        self.music_commands_pub = cmd_music_pub

        self.loc_command_sub = rospy.Subscriber('/recognizer/output', String, self.location_command_CB)
        self.loc_command_string = "null"

        self.pose_pub = cmd_pose_pub

        self.pos_dict = { }

        self.region_dict = { }

        self.last_command_loc = "null"

        # TODO LOAD FILE
        with open(LOCATION_FILE_NAME, 'r') as f:
            self.pos_dict, self.region_dict = pickle.load(f)

        print self.pos_dict.keys()

    def state(self):
        return States.GO_LOC

    # language:
    #
    # save current location as position_name
    # save position_name
    #
    # go to previously saved position
    # goto position_name
    def execute(self):
        if (self.loc_command_string not in NULL_STRING_COMMAND):
            temp_command_string = self.loc_command_string
            self.loc_command_string = NULL_STRING_COMMAND

            tokens = temp_command_string.split()
            command = tokens[0]


            print "" , voice_constants.MOVE_POSITION , " in " , temp_command_string

            if voice_constants.MOVE_POSITION in temp_command_string:
                print "goto"
                if len(tokens) >= 2 and not tokens[1] in self.pos_dict:
                    print 'no such position:', tokens[1]
                    return
                self.last_command_loc = tokens[1]
                self.goto(self.pos_dict[tokens[1]])
            elif voice_constants.MOVE_REGION in temp_command_string:
                if len(tokens) >= 2 and not tokens[1] in self.region_dict:
                    print 'no such region:', tokens[1]
                    return
                (x, y) = self.region_dict[tokens[1]]
                t = 2*math.pi*random.random()
                u = random.random() + random.random()
                r = 2-u if u > 1 else u
                (randomX, randomY) = [region_radius*r*math.cos(t), region_radius*r*math.sin(t)]
                self.last_command_loc = tokens[1]
                self.goto([x + randomX, y + randomY, 0])

            elif voice_constants.LIST_POSITION in temp_command_string:
                print self.pos_dict.keys()

            elif voice_constants.LIST_REGION in temp_command_string:
                print self.region_dict.keys()

            elif voice_constants.WHERE_AM_I in temp_command_string:
                point = self.get_odom()
                x, y, _ = point
                print point
                containing_regions = []
                for name, region in self.region_dict.iteritems():
                    rx, ry = region
                    if math.sqrt((x - rx) ** 2 + (y - ry) ** 2) <= region_radius:
                        containing_regions.append(name)
                print "in regions:", containing_regions

            else:
                print "command not recognized"

        if (self.last_command_loc not in NULL_STRING_COMMAND):
            (x,y,_) = self.pos_dict[self.last_command_loc]
            point = self.get_odom()
            rx, ry, _ = point
            if ((x - rx) ** 2 + (y - ry) ** 2 <= 1.0):
                self.music_commands_pub.publish(MusicCommand(MusicCommand.LOAD, ["Rick Astley - Never Gonna Give You Up.wav"]))
                self.music_commands_pub.publish(MusicCommand(MusicCommand.PLAY, []))
                self.last_command_loc = NULL_STRING_COMMAND

    def cleanup(self):
        self.music_commands_pub.publish(MusicCommand(MusicCommand.STOP, []))
        self.goto(self.pos_dict["charging"])

    def take_action(self, action):
        if action == Actions.END_LOCATION:
            return States.START
        else:
            return None

    def location_command_CB(self, arg):
        self.loc_command_string = arg.data

    def get_odom(self):
        (trans, rot) = self.tf_listener.lookupTransform(self.base_frame,
                       self.odom_frame, rospy.Time(0))
        return get_x_y_theta(trans, rot)

    def goto(self, coords):
        print "going to:", coords
        p = get_pose_stamped(*coords)
        self.pose_pub.publish(p)

class Start(EmptyState):
    def state(self):
        return States.START

    def take_action(self, action):
        if action == Actions.BEGIN_FIESTA:
            return States.FIESTA

        if action == Actions.BEGIN_FOLLOW:
            return States.SEARCH_TARGET

        if action == Actions.BEGIN_LOCATION:
            return States.GO_LOC

        else:
            return None

class StateMachine:
    def __init__(self):
        print 'start init state_machine'
        self.mobile_base_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        self.pose_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.music_commands_pub = rospy.Publisher('music_commands', MusicCommand)
        self.r = rospy.Rate(10.0)

        self.state = self.state_factory(States.START)
        print 'end init state_machine'

    def state_factory(self, state):
        print "factory ", state
        if state == States.START:
            return Start()
        elif state == States.FIESTA:
            return Fiesta(self.mobile_base_pub, self.music_commands_pub)
        elif state == States.GO_HOME:
            return None
        elif state == States.GO_LOC:
            return Location(self.pose_pub, self.music_commands_pub)
        elif state == States.FOLLOW_TARGET:
            return FollowTarget(self.mobile_base_pub)
        elif state == States.SEARCH_TARGET:
            return SearchTarget(self.mobile_base_pub)

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
                self.voice_actions_pub.publish(Actions.RESET)
            elif voice_constants.END_FIESTA in msg.data:
                self.state_machine.take_action(Actions.END_FIESTA)
                self.voice_actions_pub.publish(Actions.END_FIESTA)
            elif voice_constants.BEGIN_FIESTA in msg.data:
                self.state_machine.take_action(Actions.BEGIN_FIESTA)
                self.voice_actions_pub.publish(Actions.BEGIN_FIESTA)
            elif voice_constants.BEGIN_LOCATION in msg.data:
                self.state_machine.take_action(Actions.BEGIN_LOCATION)
                self.voice_actions_pub.publish(Actions.BEGIN_LOCATION)
            elif voice_constants.END_LOCATION in msg.data:
                self.state_machine.take_action(Actions.END_LOCATION)
                self.voice_actions_pub.publish(Actions.END_LOCATION)
            elif voice_constants.BEGIN_FOLLOW in msg.data:
                self.state_machine.take_action(Actions.BEGIN_FOLLOW)
                self.voice_actions_pub.publish(Actions.BEGIN_FOLLOW)
            elif voice_constants.END_FOLLOW in msg.data:
                self.state_machine.take_action(Actions.END_FOLLOW)
                self.voice_actions_pub.publish(Actions.END_FOLLOW)

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
