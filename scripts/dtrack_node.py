#!/usr/bin/env python3

import rospy
import socket
import re
import math
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import tf2_ros
import tf
from tf.transformations import quaternion_matrix, quaternion_from_matrix, quaternion_from_euler
import numpy as np


TORSTEN_ID = 0
UR10_ID = 1


def pose_to_matrix(position, quaternion):
    T = quaternion_matrix(quaternion)
    T[:3, 3] = position
    return T


def invert_matrix(T):
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.eye(4)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv


def matrix_to_pose(T):
    position = T[:3, 3]
    quaternion = quaternion_from_matrix(T)
    return position, quaternion


class DTrack3Node:
    def __init__(self):
        rospy.init_node('dtrack3_publisher')

        # --- Pose Publisher ---
        self.map_torsten_pose_pub = rospy.Publisher('map_torsten_pose', PoseStamped, queue_size=10)
        self.map_ur10_pose_pub = rospy.Publisher('map_ur10_pose', PoseStamped, queue_size=10)

        # --- Path Publisher ---
        self.map_torsten_path_pub = rospy.Publisher('map_torsten_path', Path, queue_size=10)
        self.map_ur10_path_pub = rospy.Publisher('map_ur10_path', Path, queue_size=10)

        self.map_torsten_path_msg = Path()
        self.map_torsten_path_msg.header.frame_id = "mocap_torsten"

        self.map_ur10_path_msg = Path()
        self.map_ur10_path_msg.header.frame_id = "mocap_ur10"

        self.path_msg_max_len = 500

        # --- UDP ---
        port = rospy.get_param('~udp_port', 5000)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', port))
        self.sock.settimeout(1.0)

        # --- TF ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.br = tf.TransformBroadcaster()

        # --- Origin ---
        self.origin_position = None
        self.origin_quat = None

        self.T_mocap_torsten_odom = None
        self.T_map_torsten_origin = None
        self.T_map_odom = None
        self.map_odom = None

        rospy.loginfo("DTrack3 UDP listener started on port %d", port)

    def parse_dtrack_packet(self, packet):
        lines = packet.decode(errors='ignore').splitlines()

        map_torsten_poses = []
        map_ur10_poses = []

        for line in lines:
            if not line.startswith('6d '):
                continue

            try:
                parts = line.split(' ', 2)
                num_bodies = int(parts[1])
                rest = parts[2]

                groups = re.findall(r'\[([^\]]+)\]', rest)

                for i in range(num_bodies):
                    base = i * 3

                    meta = list(map(float, groups[base].split()))
                    body_id = int(meta[0])
                    quality = meta[1]

                    if quality < 0.5:
                        continue

                    pos_euler = list(map(float, groups[base + 1].split()))
                    raw_x, raw_y, raw_z = pos_euler[:3]
                    raw_rx, raw_ry, raw_rz = pos_euler[3:6]

                    x = raw_x / 1000.0
                    y = raw_y / 1000.0
                    z = raw_z / 1000.0

                    q = quaternion_from_euler(
                        math.radians(raw_rx),
                        math.radians(raw_ry),
                        math.radians(raw_rz)
                    )

                    # --- Origin setzen (Torsten) ---
                    if self.origin_position is None and body_id == TORSTEN_ID:
                        self.origin_position = (x, y, z)
                        self.origin_quat = q
                        self.T_map_torsten_origin = pose_to_matrix(self.origin_position, self.origin_quat)

                    if self.origin_position is None:
                        continue

                    pose = PoseStamped()
                    pose.header.stamp = rospy.Time.now()
                    pose.pose.position.x = x
                    pose.pose.position.y = y
                    pose.pose.position.z = z
                    pose.pose.orientation.x = q[0]
                    pose.pose.orientation.y = q[1]
                    pose.pose.orientation.z = q[2]
                    pose.pose.orientation.w = q[3]

                    if body_id == TORSTEN_ID:
                        pose.header.frame_id = "mocap_torsten"
                        map_torsten_poses.append(pose)
                    elif body_id == UR10_ID:
                        pose.header.frame_id = "mocap_ur10"
                        map_ur10_poses.append(pose)

            except Exception as e:
                rospy.logwarn(f"Parse error: {e}")

        return map_torsten_poses, map_ur10_poses

    def update_path(self, path_msg, pose):
        path_msg.header.stamp = rospy.Time.now()
        path_msg.poses.append(pose)

        if len(path_msg.poses) > self.path_msg_max_len:
            path_msg.poses.pop(0)

    def publish_static_transform(self):      
        self.br.sendTransform(
                (-0.708746, -0.121984, 0.82868),
                (0.0, 0.0, 1.0, 0.0),
                rospy.Time.now(),
                "mocap_torsten",
                "mason/torsten/base_link"
        )
        # self.br.sendTransform(
        #         (0.0, 0.0, 0.0),
        #         (0.0, 0.0, 0.0, 1.0),
        #         rospy.Time.now(),
        #         "mocap_ur10",
        #         "map"
        # )
        # if self.map_odom is not None:
        #     self.br.sendTransform(
        #         (self.map_odom[0][0], self.map_odom[0][1], self.map_odom[0][2]),
        #         (self.map_odom[1][0], self.map_odom[1][1], self.map_odom[1][2], self.map_odom[1][3]),
        #         rospy.Time.now(),
        #         "odom",
        #         "map"
        #     )  

    def run(self):
        rate = rospy.Rate(100)

        while not rospy.is_shutdown():
            self.publish_static_transform()
            try:
                data, _ = self.sock.recvfrom(4096)
                # rospy.loginfo(f"data:\n{data}")

                torsten_poses, ur10_poses = self.parse_dtrack_packet(data)
                # rospy.loginfo(f"torsten_poses:\n{torsten_poses}")
                # rospy.loginfo(f"ur10_poses:\n{ur10_poses}")

                # --- Torsten ---
                for pose in torsten_poses:
                    # rospy.loginfo(f"torsten for loop pose:\n{pose}")
                    self.map_torsten_pose_pub.publish(pose)
                    T_ = pose_to_matrix((pose.pose.position.x, pose.pose.position.y, pose.pose.position.z), (pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w))
                    T_inv = invert_matrix(T_)
                    torsten_pose, quat = matrix_to_pose(T_inv)
                    # rospy.loginfo(f"invertierte torsten pose:\n{torsten_pose}")
                    self.br.sendTransform(
                            (torsten_pose[0], torsten_pose[1], torsten_pose[2]),
                            (quat[0], quat[1], quat[2], quat[3]),
                            rospy.Time.now(),
                            "map",
                            "mocap_torsten",
                    )
                    self.update_path(self.map_torsten_path_msg, pose)
                    self.map_torsten_path_pub.publish(self.map_torsten_path_msg)

                # --- UR10 ---
                for pose in ur10_poses:
                    # rospy.loginfo(f"ur10 for loop pose:\n{pose}")
                    self.map_ur10_pose_pub.publish(pose)
                    self.br.sendTransform(
                            (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z),
                            (pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w),
                            rospy.Time.now(),
                            "mocap_ur10",
                            "map",
                    )
                    self.update_path(self.map_ur10_path_msg, pose)
                    self.map_ur10_path_pub.publish(self.map_ur10_path_msg)

            except socket.timeout:
                continue


if __name__ == '__main__':
    try:
        node = DTrack3Node()
        node.run()
    except rospy.ROSInterruptException:
        pass