#!/usr/bin/env python3

import rospy
import csv
import os
from nav_msgs.msg import Odometry

class OdomLogger:
    def __init__(self):
        rospy.init_node('odom_logger', anonymous=True)

        # Get path to current script, then to src/Data
        script_dir = os.path.dirname(os.path.realpath(__file__))
        data_dir = os.path.join(script_dir, '..', '..', 'Data')
        os.makedirs(data_dir, exist_ok=True)

        # Output file path
        self.file_path = os.path.join(data_dir, 'odom_data.csv')

        # Open CSV file and write header
        self.csv_file = open(self.file_path, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            'time',
            'pos_x', 'pos_y', 'pos_z',
            'orient_x', 'orient_y', 'orient_z', 'orient_w',
            'linear_x', 'linear_y', 'linear_z',
            'angular_x', 'angular_y', 'angular_z'
        ])

        # Subscriber
        rospy.Subscriber('/odom', Odometry, self.callback)

        rospy.on_shutdown(self.shutdown_hook)

        rospy.loginfo("Odom recording started — saving to %s", self.file_path)

    def callback(self, msg):
        t = msg.header.stamp.to_sec()
        pos = msg.pose.pose.position
        orient = msg.pose.pose.orientation
        lin = msg.twist.twist.linear
        ang = msg.twist.twist.angular

        self.writer.writerow([
            t,
            pos.x, pos.y, pos.z,
            orient.x, orient.y, orient.z, orient.w,
            lin.x, lin.y, lin.z,
            ang.x, ang.y, ang.z
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
