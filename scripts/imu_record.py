#!/usr/bin/env python3

import rospy
import csv
import os
from sensor_msgs.msg import Imu # Changed from geometry_msgs.msg.PoseStamped

class ImuLogger: # Renamed class to ImuLogger
    def __init__(self):
        rospy.init_node('imu_logger', anonymous=True) # Renamed node

        # Get current script directory
        script_dir = os.path.dirname(os.path.realpath(__file__))

        # Navigate to src/Data relative to script (../../Data)
        data_dir = os.path.join(script_dir, '..', '..', 'Data')
        os.makedirs(data_dir, exist_ok=True)

        # Full file path for output CSV
        self.file_path = os.path.join(data_dir, 'imu_data.csv') # Changed output filename

        # Open CSV file
        self.csv_file = open(self.file_path, 'w', newline='')
        self.writer = csv.writer(self.csv_file)

        # Write CSV header - Updated for IMU data
        self.writer.writerow([
            'time',
            'orient_x', 'orient_y', 'orient_z', 'orient_w',
            'ang_vel_x', 'ang_vel_y', 'ang_vel_z',
            'lin_acc_x', 'lin_acc_y', 'lin_acc_z'
        ])

        # Subscribe to IMU data topic - Changed topic and message type
        # Common IMU topic is /imu/data, but adjust if yours is different
        rospy.Subscriber('/imu/data', Imu, self.callback)

        rospy.on_shutdown(self.shutdown_hook)

        rospy.loginfo("IMU recording started — saving to %s", self.file_path)

    def callback(self, msg):
        t = msg.header.stamp.to_sec()
        
        # Orientation (Quaternion)
        o = msg.orientation
        
        # Angular Velocity
        av = msg.angular_velocity
        
        # Linear Acceleration
        la = msg.linear_acceleration

        self.writer.writerow([
            t,
            o.x, o.y, o.z, o.w,
            av.x, av.y, av.z,
            la.x, la.y, la.z
        ])
        self.csv_file.flush() # Ensure data is written to disk immediately

    def shutdown_hook(self):
        rospy.loginfo("Shutting down... Closing CSV file.")
        self.csv_file.close()

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        logger = ImuLogger() # Instantiating the new class
        logger.run()
    except rospy.ROSInterruptException:
        pass
