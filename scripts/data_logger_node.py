#!/usr/bin/env python3

import rospy
import csv
import os
import rosbag
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import tf.transformations as tft
import signal # Already imported, good
import threading

class DataLogger:
    def __init__(self):
        self.shutting_down = False
        self.lock = threading.Lock()
        rospy.init_node('data_logger_node', anonymous=True)
        rospy.loginfo("📡 Data Logger Node Started")

        # Output paths
        script_dir = os.path.dirname(os.path.realpath(__file__))
        data_dir = os.path.join(script_dir, '..', '..', 'Data')
        os.makedirs(data_dir, exist_ok=True)

        # Open CSV files
        self.files = {
            "imu": open(os.path.join(data_dir, "imu_data.csv"), 'w', newline=''),
            "odom": open(os.path.join(data_dir, "odom_data.csv"), 'w', newline=''),
            "dtrack": open(os.path.join(data_dir, "dtrack_pose.csv"), 'w', newline=''),
        }
        self.writers = {k: csv.writer(f) for k, f in self.files.items()}

        # Write headers
        # Corrected IMU header to match IMU message content (no position fields)
        self.writers["imu"].writerow([
            "time", "qx", "qy", "qz", "qw", "yaw",
            "ax", "ay", "az", "wx", "wy", "wz"
        ])
        pose_header = ["time", "x", "y", "z", "qx", "qy", "qz", "qw", "yaw"]
        self.writers["odom"].writerow(pose_header + ["vx", "vy", "vz", "vroll", "vpitch", "vyaw"])
        self.writers["dtrack"].writerow(pose_header)

        # Open single rosbag file
        try:
            self.bag = rosbag.Bag(os.path.join(data_dir, "combined_data.bag"), 'w')
            rospy.loginfo("Bag file opened successfully.")
        except Exception as e:
            rospy.logerr(f"Failed to open bag file: {e}")
            rospy.signal_shutdown(f"Bag file error: {e}")
            return # Exit init if bag fails

        # Message counters
        self.counts = {"imu": 0, "odom": 0, "dtrack": 0}

        # Subscribers
        rospy.Subscriber("/imu/data", Imu, self.imu_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/dtrack_pose", PoseStamped, self.dtrack_callback)

        # IMPORTANT: Give time for subscribers to connect before spinning
        rospy.sleep(0.5) # Added a small delay here

        # Signal and shutdown handlers
        rospy.on_shutdown(self.shutdown)
        #signal.signal(signal.SIGINT, self.sigint_handler)

    def sigint_handler(self, signum, frame):
        rospy.loginfo("🔴 SIGINT received. Shutting down...")
        rospy.signal_shutdown("SIGINT")

    def quat_to_yaw(self, q):
        _, _, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
        return yaw

    def imu_callback(self, msg):
        if self.shutting_down:
            return
        with self.lock:
            try:
                t = msg.header.stamp.to_sec()
            except AttributeError: # More specific exception
                rospy.logwarn("IMU message missing header timestamp or attribute.")
                return

            q = msg.orientation
            yaw = self.quat_to_yaw(q)
            acc = msg.linear_acceleration
            ang = msg.angular_velocity

            self.writers["imu"].writerow([
                t, q.x, q.y, q.z, q.w, yaw,
                acc.x, acc.y, acc.z,
                ang.x, ang.y, ang.z
            ])
            self.bag.write("/imu/data", msg, msg.header.stamp)
            self.counts["imu"] += 1

            if self.counts["imu"] % 50 == 0:
                rospy.loginfo(f"IMU messages logged: {self.counts['imu']}")

    def odom_callback(self, msg):
        if self.shutting_down:
            return
        with self.lock:
            try:
                t = msg.header.stamp.to_sec()
            except AttributeError: # More specific exception
                rospy.logwarn("Odom message missing header timestamp or attribute.")
                return

            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            yaw = self.quat_to_yaw(q)

            self.writers["odom"].writerow([
                t, p.x, p.y, p.z, q.x, q.y, q.z, q.w, yaw
            ])
            self.bag.write("/odom", msg, msg.header.stamp)
            self.counts["odom"] += 1

            if self.counts["odom"] % 50 == 0:
                rospy.loginfo(f"Odom messages logged: {self.counts['odom']}")

    def dtrack_callback(self, msg):
        if self.shutting_down:
            return
        with self.lock:
            try:
                t = msg.header.stamp.to_sec()
            except AttributeError: # More specific exception
                rospy.logwarn("DTrack message missing header timestamp or attribute.")
                return

            p = msg.pose.position
            q = msg.pose.orientation
            yaw = self.quat_to_yaw(q)

            self.writers["dtrack"].writerow([
                t, p.x, p.y, p.z, q.x, q.y, q.z, q.w, yaw
            ])
            self.bag.write("/dtrack_pose", msg, msg.header.stamp)
            self.counts["dtrack"] += 1

            if self.counts["dtrack"] % 50 == 0:
                rospy.loginfo(f"DTrack messages logged: {self.counts['dtrack']}")

    def shutdown(self):
        if self.shutting_down:
            return
        rospy.loginfo("🔴 Shutdown initiated")
        self.shutting_down = True
        # Kurz warten, damit laufende Callbacks fertig werden
        rospy.sleep(0.1)
        # Finalize logs
        rospy.loginfo("📥 Finalizing logs...")
        with self.lock:
            for f in self.files.values():
                f.close()
            self.bag.close()
        rospy.loginfo(f"✅ Logging complete. Messages written - IMU: {self.counts['imu']}, Odom: {self.counts['odom']}, DTrack: {self.counts['dtrack']}")

if __name__ == '__main__':
    try:
        logger = DataLogger()
        rospy.spin() # This will block and keep the node alive
    except rospy.ROSInterruptException:
        # This exception is raised when Ctrl+C is pressed
        rospy.loginfo("Logger node interrupted by user.")
    except Exception as e:
        rospy.logerr(f"An unexpected error occurred: {e}")