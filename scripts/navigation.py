#!/usr/bin/python

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, PoseStamped, Point
from std_msgs.msg import Header, ColorRGBA
import rospy
import math
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import time
import pickle
import random
import sys


region_radius = 1.0

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

class Navigation:
    def __init__(self):
        rospy.init_node('navigator', anonymous=True)

        self.tf_listener = tf.TransformListener()
        self.base_frame = '/map'
        self.odom_frame = '/base_link'

        self.pose_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped)
        self.r = rospy.Rate(10)

        self.pos_dict = { }

        self.region_dict = { }

    def get_odom(self):
        (trans, rot) = self.tf_listener.lookupTransform(self.base_frame,
                       self.odom_frame, rospy.Time(0))
        return get_x_y_theta(trans, rot)

    def loop(self):
        while not rospy.is_shutdown():
            self.loop_contents()
            self.r.sleep()

    def goto(self, coords):
        print "going to:", coords
        p = get_pose_stamped(*coords)
        self.pose_pub.publish(p)

    def loop_contents(self):
        # language:
        #
        # save current location as position_name
        # save position_name
        #
        # go to previously saved position
        # goto position_name
        s = raw_input()

        tokens = s.split()

        command = tokens[0]

        if command == 'goto_pos' or command == 'goto':
            if not tokens[1] in self.pos_dict:
                print 'no such position:', tokens[1]
                return
            self.goto(self.pos_dict[tokens[1]])

        elif command == 'goto_region':
            if not tokens[1] in self.region_dict:
                print 'no such region:', tokens[1]
                return
            (x, y) = self.region_dict[tokens[1]]
            t = 2*math.pi*random.random()
            u = random.random() + random.random()
            r = 2-u if u > 1 else u
            (randomX, randomY) = [region_radius*r*math.cos(t), region_radius*r*math.sin(t)]

            self.goto([x + randomX, y + randomY, 0])


        elif command == 'save_pos':
            current_pos = self.get_odom()
            print "saving", current_pos, "as", tokens[1]
            self.pos_dict[tokens[1]] = current_pos

        elif command == 'save_region':
            (x,y,_) = self.get_odom()
            self.region_dict[tokens[1]] = (x,y)

        elif command == 'del_pos':
            if tokens[1] in self.pos_dict:
                del self.pos_dict[tokens[1]]

        elif command == 'del_region':
            if tokens[1] in self.region_dict:
                del self.region_dict[tokens[1]]

        elif command == 'list_poses':
            print self.pos_dict.keys()

        elif command == 'list_regions':
            print self.region_dict.keys()

        elif command == 'print_pos':
            print (tokens[1] + ":"), self.pos_dict.get(tokens[1])

        elif command == 'print_region':
            print (tokens[1] + ":"), self.region_dict.get(tokens[1])

        elif command == 'save_to':
            with open(tokens[1], 'w') as f:
                pickle.dump((self.pos_dict, self.region_dict), f)

        elif command == 'load_from':
            with open(tokens[1], 'r') as f:
                self.pos_dict, self.region_dict = pickle.load(f)

        elif command == 'exit' or command == 'q':
            print 'exiting...'
            sys.exit(0)

        elif command == 'whereami':
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

if __name__ == '__main__':
    try:
        nav = Navigation()
        nav.loop()
    except rospy.ROSInterruptException: pass
