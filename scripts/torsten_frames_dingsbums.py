import numpy as np
from tf.transformations import quaternion_matrix, quaternion_from_matrix, quaternion_from_euler

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

T_wheel_fl_base_link = pose_to_matrix([-0.389, 0.079, -0.334], [0.707, 0.000, 0.000, 0.707])

T_mocap_torsten_marker7 = np.eye(4)
T_mocap_torsten_marker7[:3, 3] = [-1.097746, -0.455984, -0.542680]

T_marker7_wheel_fl = np.array((
        (   -1.0,   0.0,    0.0,    0.0),
        (    0.0,   0.0,   -1.0,    0.0),
        (    0.0,  -1.0,    0.0,   -0.207),
        (    0.0,   0.0,    0.0,    1.0)
        ), dtype=np.float64)

# T = np.dot(T1, np.dot(T2, T3))

T_mocap_torsten_base_link = np.dot(T_mocap_torsten_marker7, np.dot(T_marker7_wheel_fl, T_wheel_fl_base_link))

gesucht = invert_matrix(T_mocap_torsten_base_link)

print(gesucht)

pos_gesucht, quat_gesucht = matrix_to_pose(gesucht)

print(f"pos:\n{pos_gesucht}\nquat\n{quat_gesucht}")

'''
[[-1.        0.        0.       -0.708746]
 [ 0.       -1.        0.       -0.121984]
 [ 0.        0.        1.        0.82868 ]
 [ 0.        0.        0.        1.      ]]

 pos:
[-0.708746 -0.121984  0.82868 ]
quat
[0. 0. 1. 0.]
'''