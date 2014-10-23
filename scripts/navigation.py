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

        self.pos_dict = {
            'home' : (-2.9, 1.4, 0),
            'rotated_home' : (-2.9, 1.4, 3.14),
        }


    def get_odom(self):
        (trans, rot) = self.tf_listener.lookupTransform(self.base_frame,
                       self.odom_frame, rospy.Time(0))
        return get_x_y_theta(trans, rot)

    def loop(self):
        while not rospy.is_shutdown():
            self.loop_contents()
            self.r.sleep()

    def goto(self, coords):
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
        if len(tokens) != 2:
            return

        command, arg = tokens
        if command == 'goto':
            if not arg in self.pos_dict:
                print 'no such position:', arg
                return
            self.goto(self.pos_dict[arg])

        elif command == 'save':
            current_pos = self.get_odom()
            print "saving", current_pos, "as", arg
            self.pos_dict[arg] = current_pos

        elif command == 'print':
            print (arg + ":"), self.pos_dict.get(arg)

        elif command == 'save_to':
            with open(arg, 'w') as f:
                pickle.dump(self.pos_dict, f)

        elif command == 'load_from':
            with open(arg, 'r') as f:
                self.pos_dict = pickle.load(f)

        else:
            print "command not recognized"

if __name__ == '__main__':
    try:
        nav = Navigation()
        nav.loop()
    except rospy.ROSInterruptException: pass
