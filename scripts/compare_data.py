import pandas as pd
import matplotlib.pyplot as plt
import os
import numpy as np

# Load CSVs
script_dir = os.path.dirname(os.path.realpath(__file__))
data_dir = os.path.join(script_dir, '..', '..', 'Data')

imu = pd.read_csv(os.path.join(data_dir, "imu_in_mocap.csv"))
odom = pd.read_csv(os.path.join(data_dir, "odom_in_mocap_raw.csv"))
odom_filt = pd.read_csv(os.path.join(data_dir, "odometry_filtered_in_mocap.csv"))
dtrack = pd.read_csv(os.path.join(data_dir, "dtrack_pose.csv"))

# Ensure numeric and drop NaNs
for df in [imu, odom, odom_filt, dtrack]:
    for col in df.columns:
        df[col] = pd.to_numeric(df[col], errors='coerce')
    df.dropna(inplace=True)

# Interpolate DTrack to match EKF timestamps
dtrack_interp = dtrack.set_index('time').reindex(odom_filt['time'], method='nearest', tolerance=0.05)
dtrack_interp.reset_index(inplace=True)

# Compute errors
ekf = odom_filt.copy()
ekf['dx'] = ekf['x'] - dtrack_interp['x']
ekf['dy'] = ekf['y'] - dtrack_interp['y']
ekf['yaw_error'] = ekf['yaw'] - dtrack_interp['yaw']

# Wrap yaw error to [-pi, pi]
ekf['yaw_error'] = (ekf['yaw_error'] + np.pi) % (2 * np.pi) - np.pi

# Plot functions
def plot_position():
    plt.figure(figsize=(10, 6))
    plt.plot(odom["time"], odom["x"], label="Odom X")
    plt.plot(odom_filt["time"], odom_filt["x"], label="Odom Filtered X")
    plt.plot(dtrack["time"], dtrack["x"], label="MoCap X")
    plt.xlabel("Time [s]")
    plt.ylabel("X Position [m]")
    plt.title("Position X Comparison")
    plt.legend()
    plt.grid()

    plt.figure(figsize=(10, 6))
    plt.plot(odom["time"], odom["y"], label="Odom Y")
    plt.plot(odom_filt["time"], odom_filt["y"], label="Odom Filtered Y")
    plt.plot(dtrack["time"], dtrack["y"], label="MoCap Y")
    plt.xlabel("Time [s]")
    plt.ylabel("Y Position [m]")
    plt.title("Position Y Comparison")
    plt.legend()
    plt.grid()

def plot_yaw():
    plt.figure(figsize=(10, 6))
    plt.plot(odom["time"], odom["yaw"], label="Odom Yaw")
    plt.plot(odom_filt["time"], odom_filt["yaw"], label="Odom Filtered Yaw")
    plt.plot(dtrack["time"], dtrack["yaw"], label="MoCap Yaw")
    plt.xlabel("Time [s]")
    plt.ylabel("Yaw [rad]")
    plt.title("Yaw Comparison")
    plt.legend()
    plt.grid()

def plot_imu():
    plt.figure(figsize=(10, 6))
    plt.plot(imu["time"], imu["ax"], label="Accel X")
    plt.plot(imu["time"], imu["ay"], label="Accel Y")
    plt.plot(imu["time"], imu["az"], label="Accel Z")
    plt.xlabel("Time [s]")
    plt.ylabel("Acceleration [m/s²]")
    plt.title("IMU Acceleration")
    plt.legend()
    plt.grid()

    plt.figure(figsize=(10, 6))
    plt.plot(imu["time"], imu["wx"], label="Gyro X")
    plt.plot(imu["time"], imu["wy"], label="Gyro Y")
    plt.plot(imu["time"], imu["wz"], label="Gyro Z")
    plt.xlabel("Time [s]")
    plt.ylabel("Angular Velocity [rad/s]")
    plt.title("IMU Angular Velocity")
    plt.legend()
    plt.grid()

def plot_errors():
    plt.figure(figsize=(10, 6))
    plt.plot(ekf["time"], ekf["dx"], label="X Error (EKF - MoCap)")
    plt.plot(ekf["time"], ekf["dy"], label="Y Error (EKF - MoCap)")
    plt.xlabel("Time [s]")
    plt.ylabel("Position Error [m]")
    plt.title("EKF vs MoCap Position Error")
    plt.legend()
    plt.grid()

    plt.figure(figsize=(10, 6))
    plt.plot(ekf["time"], ekf["yaw_error"], label="Yaw Error (EKF - MoCap)")
    plt.xlabel("Time [s]")
    plt.ylabel("Yaw Error [rad]")
    plt.title("EKF vs MoCap Yaw Error")
    plt.legend()
    plt.grid()

# Run plots
plot_position()
plot_yaw()
plot_imu()
plot_errors()
plt.show()
