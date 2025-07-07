#!/usr/bin/env python3

import rospy
import csv
import os
import rosbag
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import tf.transformations as tft

class DataLogger:
    def __init__(self):
        rospy.init_node('data_logger_node')
        rospy.loginfo("Starting Data Logger Node...")

        # Setup directories
        script_dir = os.path.dirname(os.path.realpath(__file__))
        data_dir = os.path.join(script_dir, '..', '..', 'Data')
        os.makedirs(data_dir, exist_ok=True)
        self.output_dir = data_dir

        # Open CSV files
        self.files = {
            "imu": open(os.path.join(self.output_dir, "imu_in_mocap.csv"), 'w', newline=''),
            "odom": open(os.path.join(self.output_dir, "odom_in_mocap_raw.csv"), 'w', newline=''),
            "odom_filt": open(os.path.join(self.output_dir, "odometry_filtered_in_mocap.csv"), 'w', newline=''),
            "dtrack": open(os.path.join(self.output_dir, "dtrack_pose.csv"), 'w', newline=''),
        }
        self.writers = {k: csv.writer(f) for k, f in self.files.items()}

        # CSV Headers
        header = ["time", "x", "y", "z", "qx", "qy", "qz", "qw", "yaw"]
        self.writers["imu"].writerow(header + ["ax", "ay", "az", "wx", "wy", "wz"])
        for key in ["odom", "odom_filt", "dtrack"]:
            self.writers[key].writerow(header)

        # Open rosbag files
        self.bags = {
            "imu": rosbag.Bag(os.path.join(self.output_dir, "imu.bag"), 'w'),
            "odom": rosbag.Bag(os.path.join(self.output_dir, "odom.bag"), 'w'),
            "odom_filt": rosbag.Bag(os.path.join(self.output_dir, "odom_filt.bag"), 'w'),
            "dtrack": rosbag.Bag(os.path.join(self.output_dir, "dtrack.bag"), 'w'),
        }

        # Subscribers
        rospy.Subscriber("/imu_in_mocap", Imu, self.imu_callback)
        rospy.Subscriber("/odom_in_mocap_raw", Odometry, self.odom_callback)
        rospy.Subscriber("/odometry_filtered_in_mocap", Odometry, self.odom_filt_callback)
        rospy.Subscriber("/dtrack_pose", PoseStamped, self.dtrack_callback)

    def quat_to_yaw(self, q):
        _, _, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
        return yaw

    def imu_callback(self, msg):
        t = msg.header.stamp.to_sec()
        q = msg.orientation
        yaw = self.quat_to_yaw(q)
        acc = msg.linear_acceleration
        ang = msg.angular_velocity
        self.writers["imu"].writerow([
            t, 0, 0, 0, q.x, q.y, q.z, q.w, yaw,
            acc.x, acc.y, acc.z,
            ang.x, ang.y, ang.z
        ])
        self.bags["imu"].write("/imu_in_mocap", msg)

    def odom_callback(self, msg):
        self._log_pose("odom", msg.header.stamp.to_sec(), msg.pose.pose)
        self.bags["odom"].write("/odom_in_mocap_raw", msg)

    def odom_filt_callback(self, msg):
        self._log_pose("odom_filt", msg.header.stamp.to_sec(), msg.pose.pose)
        self.bags["odom_filt"].write("/odometry_filtered_in_mocap", msg)

    def dtrack_callback(self, msg):
        self._log_pose("dtrack", msg.header.stamp.to_sec(), msg.pose)
        self.bags["dtrack"].write("/dtrack_pose", msg)

    def _log_pose(self, key, t, pose):
        p = pose.position
        q = pose.orientation
        yaw = self.quat_to_yaw(q)
        self.writers[key].writerow([
            t, p.x, p.y, p.z, q.x, q.y, q.z, q.w, yaw
        ])

    def shutdown(self):
        rospy.loginfo("Shutting down data logger...")
        for f in self.files.values():
            f.close()
        for b in self.bags.values():
            b.close()

if __name__ == '__main__':
    logger = DataLogger()
    rospy.on_shutdown(logger.shutdown)
    rospy.spin()
