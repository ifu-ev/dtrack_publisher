#!/usr/bin/env python3

import rospy
import csv
import os
from nav_msgs.msg import Odometry

class OdomLogger:
    def __init__(self):
        rospy.init_node('odom_logger', anonymous=True)

        # Get current script directory
        script_dir = os.path.dirname(os.path.realpath(__file__))

        # Navigate to src/Data relative to script
        data_dir = os.path.join(script_dir, '..', '..', 'Data')
        os.makedirs(data_dir, exist_ok=True)

        # Output file path
        self.file_path = os.path.join(data_dir, 'odom_filtered_data.csv')

        # Open CSV file for writing
        self.csv_file = open(self.file_path, 'w', newline='')
        self.writer = csv.writer(self.csv_file)

        # Write CSV header
        self.writer.writerow([
            'time',
            'pos_x', 'pos_y', 'pos_z',
            'orient_x', 'orient_y', 'orient_z', 'orient_w',
            'lin_vel_x', 'lin_vel_y', 'lin_vel_z',
            'ang_vel_x', 'ang_vel_y', 'ang_vel_z'
        ])

        # Subscribe to the filtered odometry topic
        rospy.Subscriber('/odometry/filtered', Odometry, self.callback)

        rospy.on_shutdown(self.shutdown_hook)

        rospy.loginfo("Filtered Odometry recording started — saving to %s", self.file_path)

    def callback(self, msg):
        t = msg.header.stamp.to_sec()

        # Position
        p = msg.pose.pose.position

        # Orientation (Quaternion)
        o = msg.pose.pose.orientation

        # Linear velocity
        lv = msg.twist.twist.linear

        # Angular velocity
        av = msg.twist.twist.angular

        self.writer.writerow([
            t,
            p.x, p.y, p.z,
            o.x, o.y, o.z, o.w,
            lv.x, lv.y, lv.z,
            av.x, av.y, av.z
        ])
        self.csv_file.flush()

    def shutdown_hook(self):
        rospy.loginfo("Shutting down... Closing CSV file.")
        self.csv_file.close()

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        logger = OdomLogger()
        logger.run()
    except rospy.ROSInterruptException:
        pass
