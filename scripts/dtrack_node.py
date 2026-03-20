#!/usr/bin/env python3

import rospy
import socket
import re
import math
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import tf
from tf.transformations import quaternion_from_euler

class DTrack3Node:
    def __init__(self):
        rospy.init_node('dtrack3_publisher')
        self.odom_pose_pub = rospy.Publisher('dtrack_odom_pose', PoseStamped, queue_size=10)
        self.odom_path_pub = rospy.Publisher('dtrack_odom_path', Path, queue_size=10)

        self.world_pose_pub = rospy.Publisher('dtrack_world_pose', PoseStamped, queue_size=10)
        self.world_path_pub = rospy.Publisher('dtrack_world_path', Path, queue_size=10)

        port = rospy.get_param('~udp_port', 5000)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', port))
        self.sock.settimeout(1.0)

        self.br = tf.TransformBroadcaster()
        self.origin_position = None

        self.odom_path_msg = Path()
        self.odom_path_msg.header.frame_id = "odom_mocap"

        self.world_path_msg = Path()
        self.world_path_msg.header.frame_id = "world"

        self.path_msg_max_len = 500

        rospy.loginfo("DTrack3 UDP listener started on port %d", port)

    def parse_dtrack_packet(self, packet):
        lines = packet.decode(errors='ignore').splitlines()
        odom_poses = []
        world_poses = []
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

                            world_pose = PoseStamped()
                            world_pose.header.stamp = pub_time
                            world_pose.header.frame_id = "world" # alternativ: mocap oder mocap_world?
                            world_pose.pose.position.x = x
                            world_pose.pose.position.y = y
                            world_pose.pose.position.z = z
                            world_pose.pose.orientation.x = q_current[0]
                            world_pose.pose.orientation.y = q_current[1]
                            world_pose.pose.orientation.z = q_current[2]
                            world_pose.pose.orientation.w = q_current[3]

                            odom_poses.append(odom_pose)
                            world_poses.append(world_pose)
                except Exception as e:
                    rospy.logwarn("Failed to parse line: %s with error: %s", line, str(e))
        return odom_poses, world_poses

    def publish_static_transform(self):      
        self.br.sendTransform(
                (0.0, 0.0, 0.0),
                (0.0, 0.0, 0.0, 1.0),
                rospy.Time.now(),
                "odom_mocap",
                "odom"
        )  

        self.br.sendTransform(
                (0.855771, 0.325434, -0.303391), # z ergibt keinen sinn
                (-0.015785, -0.691817, 0.721640, 0.009282), #erste Schätzung TODO genauer bestimmen
                rospy.Time.now(),
                "mason/torsten/base_link",
                "mocap_robot"
        )
        try:
            self.br.sendTransform(
                    (self.origin_position[0], self.origin_position[1], self.origin_position[2]),
                    (0.0, 0.0, 0.0, 1.0),
                    rospy.Time.now(),
                    "odom",
                    "world"
            )
        except:
            pass

    def publish_world_base_link_transform(self, pose: PoseStamped):
        self.br.sendTransform(
            (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z),
            (pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w),
            rospy.Time.now(),
            "mocap_robot", # mocap_robot -> base_link noch unbekannt... TODO bestimmen
            "world"
        )

    def run(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.publish_static_transform()
            try:
                data, _ = self.sock.recvfrom(4096)
                odom_poses, world_poses = self.parse_dtrack_packet(data)
                for index, _ in enumerate(odom_poses):
                    
                    odom_pose = odom_poses[index]
                    world_pose = world_poses[index]
                    
                    
                    self.odom_pose_pub.publish(odom_pose)
                    self.world_pose_pub.publish(world_pose)

                    # Update path
                    time_now = rospy.Time.now()
                    self.odom_path_msg.header.stamp = time_now
                    self.odom_path_msg.poses.append(odom_pose)
                    
                    self.world_path_msg.header.stamp = time_now
                    self.world_path_msg.poses.append(world_pose)

                    if len(self.odom_path_msg.poses) > self.path_msg_max_len:
                        self.odom_path_msg.poses.pop(0)
                    elif len(self.world_path_msg.poses) > self.path_msg_max_len:
                        self.world_path_msg.poses.pop(0)

                    self.odom_path_pub.publish(self.odom_path_msg)
                    self.world_path_pub.publish(self.world_path_msg)

                    self.publish_world_base_link_transform(world_pose)

                # self.publish_static_transform()
            except socket.timeout:
                self.publish_static_transform()
                continue

if __name__ == '__main__':
    try:
        node = DTrack3Node()
        node.run()
    except rospy.ROSInterruptException:
        pass
