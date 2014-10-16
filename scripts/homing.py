#!/usr/bin/python

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3, PoseWithCovarianceStamped, Twist
from std_msgs.msg import Header, ColorRGBA
import rospy
import math
import tf
from tf.transformations import euler_from_quaternion

def sign(x):
    if x < 0:
        return -1.0
    elif x > 0:
        return 1.0
    else:
        return 0.0

class PathMarker:
    def __init__(self):
        self.marker_publisher = rospy.Publisher('visualization_marker', Marker)
        self.velocity_publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        rospy.init_node('path_marker', anonymous=True)
        rospy.Subscriber("/robot_pose_ekf/odom_combined",
                         PoseWithCovarianceStamped, self.pose_callback)
        self.r = rospy.Rate(10)
        self.pose = None
        self.counter = 0

    def pose_callback(self, pose):
        self.pose = pose
        self.counter += 1

    def loop(self):
        while not rospy.is_shutdown():
            self.loop_contents()
            self.r.sleep()

    def loop_contents(self):
        if self.pose:
            q = self.pose.pose.pose.orientation
            (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
            p = self.pose.pose.pose.position
            print "POS:", p, "YAW:", yaw

            pos_epsilon = 0.1
            angle_epsilon = 0.1
            msg = None
            if abs(p.y) > pos_epsilon:
                if abs(math.pi/2.0 - yaw) > angle_epsilon:
                    msg = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, math.pi/2.0 - yaw))
                else:
                    msg = Twist(Vector3(-sign(p.y), 0.0, 0.0), Vector3(0.0, 0.0, 0.0))

            if msg:
                self.velocity_publisher.publish(msg)

if __name__ == '__main__':
    try:
        path_marker = PathMarker()
        path_marker.loop()
    except rospy.ROSInterruptException: pass
