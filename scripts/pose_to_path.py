#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class PathBuilder:
    def __init__(self):
        rospy.init_node('path_builder')

        self.path_pub = rospy.Publisher('dtrack_path', Path, queue_size=10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = rospy.get_param('~frame_id', 'odom_mocap')

        rospy.Subscriber('dtrack_pose', PoseStamped, self.pose_callback)

    def pose_callback(self, pose):
        self.path_msg.header.stamp = rospy.Time.now()
        self.path_msg.poses.append(pose)

        if len(self.path_msg.poses) > 500:
            self.path_msg.poses.pop(0)

        self.path_pub.publish(self.path_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        builder = PathBuilder()
        builder.run()
    except rospy.ROSInterruptException:
        pass
