#!/usr/bin/python

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3, PoseWithCovarianceStamped
from std_msgs.msg import Header, ColorRGBA
import rospy

class PathMarker:
    def __init__(self):
        self.marker_publisher = rospy.Publisher('visualization_marker', Marker)
        rospy.init_node('path_marker', anonymous=True)
        rospy.Subscriber("/robot_pose_ekf/odom_combined",
                         PoseWithCovarianceStamped, self.pose_callback)
        self.r = rospy.Rate(10)
        self.pose = None
        self.counter = 0

    def pose_callback(self, pose):
        self.pose = pose
        self.counter += 1
        self.show_text_in_rviz(str(self.counter))

    def show_text_in_rviz(self, text):
        if self.pose:
            position = self.pose.pose.pose.position
            orientation = self.pose.pose.pose.orientation
            print position
            marker = Marker(type=Marker.TEXT_VIEW_FACING, id=0,
                        lifetime=rospy.Duration(1.5),
                        pose=Pose(Point(0.0, 0.0, 1.0), Quaternion(0, 0, 0, 1)),
                        scale=Vector3(0.06, 0.06, 0.06),
                        header=Header(frame_id='base_link'),
                        color=ColorRGBA(0.0, 1.0, 0.0, 0.8), text=text)
            self.marker_publisher.publish(marker)

    def loop(self):
        while not rospy.is_shutdown():
            self.loop_contents()
            self.r.sleep()

    def loop_contents(self):
        pass

if __name__ == '__main__':
    try:
        path_marker = PathMarker()
        path_marker.loop()
    except rospy.ROSInterruptException: pass
