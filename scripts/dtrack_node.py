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
        self.odom_pose_pub = rospy.Publisher('dtrack_odom_pose', PoseStamped, queue_size=10)
        self.odom_path_pub = rospy.Publisher('dtrack_odom_path', Path, queue_size=10)

        self.map_pose_pub = rospy.Publisher('dtrack_map_pose', PoseStamped, queue_size=10)
        self.map_path_pub = rospy.Publisher('dtrack_map_path', Path, queue_size=10)

        port = rospy.get_param('~udp_port', 5000)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', port))
        self.sock.settimeout(1.0)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.br = tf.TransformBroadcaster()
        self.origin_position = None
        self.origin_quat = None
        self.T_mocap_torsten_odom = None
        self.T_map_odom = None
        self.map_odom = None

        self.odom_path_msg = Path()
        self.odom_path_msg.header.frame_id = "odom_mocap"

        self.map_path_msg = Path()
        self.map_path_msg.header.frame_id = "mocap_torsten"

        self.path_msg_max_len = 500

        rospy.loginfo("DTrack3 UDP listener started on port %d", port)

    def parse_dtrack_packet(self, packet):
        lines = packet.decode(errors='ignore').splitlines()
        odom_poses = []
        map_poses = []
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
                                self.origin_quat = q_current
                                self.T_map_torsten_origin = pose_to_matrix(self.origin_position, self.origin_quat)

                            ox, oy, oz = self.origin_position
                            x_adj = x - ox
                            y_adj = y - oy
                            z_adj = z - oz
                            
                            pub_time = rospy.Time.now()

                            odom_pose = PoseStamped()
                            odom_pose.header.stamp = pub_time
                            odom_pose.header.frame_id = "odom_mocap"
                            odom_pose.pose.position.x = x_adj
                            odom_pose.pose.position.y = y_adj
                            odom_pose.pose.position.z = z_adj
                            odom_pose.pose.orientation.x = q_current[0]
                            odom_pose.pose.orientation.y = q_current[1]
                            odom_pose.pose.orientation.z = q_current[2]
                            odom_pose.pose.orientation.w = q_current[3]

                            map_pose = PoseStamped()
                            map_pose.header.stamp = pub_time
                            map_pose.header.frame_id = "mocap_torsten" # alternativ: mocap oder mocap_map?
                            map_pose.pose.position.x = x
                            map_pose.pose.position.y = y
                            map_pose.pose.position.z = z
                            map_pose.pose.orientation.x = q_current[0]
                            map_pose.pose.orientation.y = q_current[1]
                            map_pose.pose.orientation.z = q_current[2]
                            map_pose.pose.orientation.w = q_current[3]

                            odom_poses.append(odom_pose)
                            map_poses.append(map_pose)
                except Exception as e:
                    rospy.logwarn("Failed to parse line: %s with error: %s", line, str(e))
        return odom_poses, map_poses

    def publish_static_transform(self):      
        self.br.sendTransform(
                (-0.708746, -0.121984, 0.82868),
                (0.0, 0.0, 1.0, 0.0),
                #(0.84070985, -0.29656136,  0.36817889),
                #(-0.01578729, -0.69191721,  0.72174453, -0.00928334), #erste Schätzung TODO genauer bestimmen
                rospy.Time.now(),
                "mocap_torsten",
                "mason/torsten/base_link"
        )
        if self.map_odom is not None:
            self.br.sendTransform(
                (self.map_odom[0][0], self.map_odom[0][1], self.map_odom[0][2]),
                (self.map_odom[1][0], self.map_odom[1][1], self.map_odom[1][2], self.map_odom[1][3]),
                rospy.Time.now(),
                "odom",
                "map"
            )  

    def publish_map_base_link_transform(self, pose_msg):
        T_map_child = pose_to_matrix(
            [pose_msg.pose.position.x,
            pose_msg.pose.position.y,
            pose_msg.pose.position.z],
            [pose_msg.pose.orientation.x,
            pose_msg.pose.orientation.y,
            pose_msg.pose.orientation.z,
            pose_msg.pose.orientation.w]
        )

        # Nur aktivieren, falls pose_msg eigentlich child -> map ist:
        T_map_child = invert_matrix(T_map_child)

        pos, quat = matrix_to_pose(T_map_child)

        self.br.sendTransform(
            (pos[0], pos[1], pos[2]),
            (quat[0], quat[1], quat[2], quat[3]),
            rospy.Time.now(),
            'mocap_torsten',
            'map'
        )

    def run(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.publish_static_transform()
            try:
                data, _ = self.sock.recvfrom(4096)
                odom_poses, map_poses = self.parse_dtrack_packet(data)
                for index, _ in enumerate(odom_poses):
                    
                    odom_pose = odom_poses[index]
                    map_pose = map_poses[index]
                    
                    
                    self.odom_pose_pub.publish(odom_pose)
                    self.map_pose_pub.publish(map_pose)

                    # Update path
                    time_now = rospy.Time.now()
                    self.odom_path_msg.header.stamp = time_now
                    self.odom_path_msg.poses.append(odom_pose)
                    
                    self.map_path_msg.header.stamp = time_now
                    self.map_path_msg.poses.append(map_pose)

                    if len(self.odom_path_msg.poses) > self.path_msg_max_len:
                        self.odom_path_msg.poses.pop(0)
                    elif len(self.map_path_msg.poses) > self.path_msg_max_len:
                        self.map_path_msg.poses.pop(0)

                    self.odom_path_pub.publish(self.odom_path_msg)
                    self.map_path_pub.publish(self.map_path_msg)

                    # self.publish_map_base_link_transform(map_pose)
                    if self.T_mocap_torsten_odom is None:
                        try:
                            tf_msg = self.tf_buffer.lookup_transform(
                                        "mocap_torsten",
                                        "odom",
                                        rospy.Time(0),
                                        rospy.Duration(0.1),
                                    )
                            rospy.loginfo(f"tf_msg:\n{tf_msg}")
                            tf_pos = (tf_msg.transform.translation.x, tf_msg.transform.translation.y, tf_msg.transform.translation.z)
                            tf_quat = (tf_msg.transform.rotation.x, tf_msg.transform.rotation.y, tf_msg.transform.rotation.z, tf_msg.transform.rotation.w)
                            self.T_mocap_torsten_odom = pose_to_matrix(position=tf_pos, quaternion=tf_quat)
                            self.T_map_odom = np.dot(self.T_map_torsten_origin, self.T_mocap_torsten_odom)
                            self.map_odom = matrix_to_pose(self.T_map_odom)
                            rospy.loginfo(f"odom im map frame:\n{self.T_map_odom}")
                        except (
                            tf2_ros.LookupException,
                            tf2_ros.ConnectivityException,
                            tf2_ros.ExtrapolationException,
                        ) as e:
                            rospy.logwarn_throttle(
                                2.0,
                                "Pose transform failed: %s -> %s (%s)" % (
                                    "odom", "mocap_torsten", str(e)
                                )
                            )
            except socket.timeout:
                self.publish_static_transform()
                continue

if __name__ == '__main__':
    try:
        node = DTrack3Node()
        node.run()
    except rospy.ROSInterruptException:
        pass
