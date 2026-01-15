#!/usr/bin/env python3

import rospy
import socket
import re
import math
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import tf
from tf.transformations import quaternion_from_euler, quaternion_inverse, quaternion_multiply

class DTrack3Node:
    def __init__(self):
        rospy.init_node('dtrack3_publisher')
        self.pose_pub = rospy.Publisher('dtrack_pose', PoseStamped, queue_size=10)
        self.path_pub = rospy.Publisher('dtrack_path', Path, queue_size=10)

        port = rospy.get_param('~udp_port', 5000)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', port))
        self.sock.settimeout(1.0)

        self.br = tf.TransformBroadcaster()
        self.origin_position = None
        self.origin_orientation_quat = None

        self.path_msg = Path()
        self.path_msg.header.frame_id = "odom_mocap"

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
                            raw_x, raw_y, raw_z = map(float, pos_euler[:3])
                            raw_rx, raw_ry, raw_rz = map(float, pos_euler[3:6])

                            x = raw_x / 1000.0
                            y = raw_y / 1000.0
                            z = raw_z / 1000.0

                            rx_rad = math.radians(raw_rx)
                            ry_rad = math.radians(raw_ry)
                            rz_rad = math.radians(raw_rz)
                            q_current = quaternion_from_euler(rx_rad, ry_rad, rz_rad)

                            if self.origin_position is None:
                                self.origin_position = (x, y, z)
                            if self.origin_orientation_quat is None:
                                self.origin_orientation_quat = q_current

                            ox, oy, oz = self.origin_position
                            x_adj = (x - ox)
                            y_adj = (y - oy)
                            z_adj = z - oz

                            q_origin_inv = quaternion_inverse(self.origin_orientation_quat)
                            q_relative = quaternion_multiply(q_origin_inv, q_current)

                            pose = PoseStamped()
                            pose.header.stamp = rospy.Time.now()
                            pose.header.frame_id = "odom_mocap"
                            pose.pose.position.x = x_adj
                            pose.pose.position.y = y_adj
                            pose.pose.position.z = z_adj
                            pose.pose.orientation.x = q_relative[0]
                            pose.pose.orientation.y = q_relative[1]
                            pose.pose.orientation.z = q_relative[2]
                            pose.pose.orientation.w = q_relative[3]

                            poses.append(pose)
                except Exception as e:
                    rospy.logwarn("Failed to parse line: %s with error: %s", line, str(e))
        return poses

    def publish_static_transform(self):
        self.br.sendTransform(
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0, 1.0),
            rospy.Time.now(),
            "odom_mocap",
            "odom"
        )

    def run(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            try:
                data, _ = self.sock.recvfrom(4096)
                poses = self.parse_dtrack_packet(data)
                for pose in poses:
                    self.pose_pub.publish(pose)

                    # Update path message
                    self.path_msg.header.stamp = rospy.Time.now()
                    self.path_msg.poses.append(pose)

                    # Limit path length (e.g., keep last 500 points)
                    if len(self.path_msg.poses) > 500:
                        self.path_msg.poses.pop(0)

                    self.path_pub.publish(self.path_msg)

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
