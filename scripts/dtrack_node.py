#!/usr/bin/env python

import rospy
import socket
import re
import math
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
import tf

class DTrack3Node:
    def __init__(self):
        rospy.init_node('dtrack3_publisher')
        self.pub = rospy.Publisher('dtrack_pose', PoseStamped, queue_size=10)

        port = rospy.get_param('~udp_port', 5000)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', port))
        self.sock.settimeout(1.0)

        self.br = tf.TransformBroadcaster()  # TF broadcaster

        rospy.loginfo("DTrack3 UDP listener started on port %d", port)

    def parse_dtrack_packet(self, packet):
        lines = packet.decode(errors='ignore').splitlines()
        poses = []
        for line in lines:
            line = line.strip()
            if line.startswith('6d '):
                try:
                    parts = line.split(' ', 2)
                    body_id = int(parts[1])
                    rest = parts[2]
                    groups = re.findall(r'\[([^\]]+)\]', rest)

                    if len(groups) >= 2:
                        pos_euler = groups[1].split()
                        if len(pos_euler) >= 6:
                            x, y, z = map(float, pos_euler[:3])
                            rx, ry, rz = map(float, pos_euler[3:6])

                            # Convert Euler angles from degrees to radians
                            rx_rad = math.radians(rx)
                            ry_rad = math.radians(ry)
                            rz_rad = math.radians(rz)

                            # Convert Euler angles (radians) to quaternion
                            qx, qy, qz, qw = quaternion_from_euler(rx_rad, ry_rad, rz_rad)

                            pose = PoseStamped()
                            pose.header.stamp = rospy.Time.now()
                            pose.header.frame_id = "odom_mocap"
                            pose.pose.position.x = x / 1000
                            pose.pose.position.y = y / 1000
                            pose.pose.position.z = z / 1000
                            pose.pose.orientation.x = qx
                            pose.pose.orientation.y = qy
                            pose.pose.orientation.z = qz
                            pose.pose.orientation.w = qw

                            poses.append(pose)
                except Exception as e:
                    rospy.logwarn("Failed to parse line: %s with error: %s", line, str(e))
        return poses

    def publish_static_transform(self):
        # Broadcast identity transform from 'world' to 'dtrack_world'
        self.br.sendTransform(
            (0.0, 0.0, 0.0),                      # translation
            (0.0, 0.0, 0.0, 1.0),                 # quaternion (no rotation)
            rospy.Time.now(),                      # timestamp
            "odom_mocap",                        # child frame
            "odom"                               # parent frame (change if needed)
        )

    def run(self):
        rate = rospy.Rate(100)
        
        while not rospy.is_shutdown():
            try:
                data, _ = self.sock.recvfrom(4096)
                poses = self.parse_dtrack_packet(data)
                for pose in poses:
                    self.pub.publish(pose)
                self.publish_static_transform()
            except socket.timeout:
                self.publish_static_transform()
                continue

if __name__ == '__main__':
    try:
        node = DTrack3Node()
        node.run()
    except rospy.ROSInterruptException:
        pass
