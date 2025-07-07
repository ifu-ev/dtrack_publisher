#!/usr/bin/env python3

import rospy
import csv
import os
from geometry_msgs.msg import PoseStamped

class MocapLogger:
    def __init__(self):
        rospy.init_node('mocap_logger', anonymous=True)

        # Get current script directory
        script_dir = os.path.dirname(os.path.realpath(__file__))

        # Navigate to src/Data relative to script (../../Data)
        data_dir = os.path.join(script_dir, '..', '..', 'Data')
        os.makedirs(data_dir, exist_ok=True)

        # Full file path for output CSV
        self.file_path = os.path.join(data_dir, 'mocap_data.csv')

        # Open CSV file
        self.csv_file = open(self.file_path, 'w', newline='')
        self.writer = csv.writer(self.csv_file)

        # Write CSV header
        self.writer.writerow([
            'time',
            'pos_x', 'pos_y', 'pos_z',
            'orient_x', 'orient_y', 'orient_z', 'orient_w'
        ])

        # Subscribe to mocap pose topic
        rospy.Subscriber('/dtrack_pose', PoseStamped, self.callback)

        rospy.on_shutdown(self.shutdown_hook)

        rospy.loginfo("Mocap recording started — saving to %s", self.file_path)

    def callback(self, msg):
        t = msg.header.stamp.to_sec()
        p = msg.pose.position
        o = msg.pose.orientation

        self.writer.writerow([
            t,
            p.x, p.y, p.z,
            o.x, o.y, o.z, o.w
        ])
        self.csv_file.flush()

    def shutdown_hook(self):
        rospy.loginfo("Shutting down... Closing CSV file.")
        self.csv_file.close()

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        logger = MocapLogger()
        logger.run()
    except rospy.ROSInterruptException:
        pass