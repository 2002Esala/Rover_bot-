#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu
import tf_conversions
import tf2_ros
import math
import numpy as np

# Global variables to store orientation
current_time = None
previous_time = None
gyro_x_prev = 0.0
gyro_y_prev = 0.0
gyro_z_prev = 0.0
roll = 0.0
pitch = 0.0
yaw = 0.0

# EKF parameters
# Initial state vector [x, y, theta] (theta represents yaw)
state = np.array([[0], [0], [0]])

# Initial covariance matrix
P = np.diag([0.1, 0.1, 0.1])

# Process noise covariance (tune this based on your system)
Q = np.diag([0.001, 0.001, 0.001])

# Measurement noise covariance (tune this based on your system)
R = np.diag([0.01, 0.01, 0.01])

# State transition matrix
F = np.eye(3)

# Measurement matrix
H = np.eye(3)

# Madgwick filter parameters
beta = 0.1   # Filter gain

def update_ekf(dt, gyro_x, gyro_y, gyro_z):
    global state, P

    # Predict step
    state[2][0] += (gyro_z + gyro_z_prev) * dt / 2.0  # Update yaw angle

    F[0][2] = dt * math.cos(state[2][0])
    F[1][2] = dt * math.sin(state[2][0])

    state = np.dot(F, state)
    P = np.dot(np.dot(F, P), F.T) + Q

    # Update step
    z = np.array([[roll], [pitch], [yaw]])  # Measurement vector
    y = z - np.dot(H, state)
    S = np.dot(np.dot(H, P), H.T) + R
    K = np.dot(np.dot(P, H.T), np.linalg.inv(S))

    state = state + np.dot(K, y)
    P = np.dot((np.eye(3) - np.dot(K, H)), P)

def imu_callback(data):
    global current_time, previous_time, gyro_x_prev, gyro_y_prev, gyro_z_prev, roll, pitch, yaw

    array_data = data.data
    gyro_x = array_data[3]
    gyro_y = array_data[4]
    gyro_z = array_data[5]

    current_time = rospy.Time.now()

    if previous_time is None:
        previous_time = current_time
        gyro_x_prev = gyro_x
        gyro_y_prev = gyro_y
        gyro_z_prev = gyro_z
        return

    dt = (current_time - previous_time).to_sec()

    # Calculate the change in angles using the gyroscope data (Integration)
    roll += (gyro_x + gyro_x_prev) * dt / 2.0
    pitch += (gyro_y + gyro_y_prev) * dt / 2.0
    yaw += (gyro_z + gyro_z_prev) * dt / 2.0

    # Update previous values for the next iteration
    previous_time = current_time
    gyro_x_prev = gyro_x
    gyro_y_prev = gyro_y
    gyro_z_prev = gyro_z

    # Madgwick filter update
    q = tf_conversions.transformations.quaternion_from_euler(roll, pitch, yaw)
    q_imu = tf_conversions.transformations.quaternion_from_euler(gyro_x*dt, gyro_y*dt, gyro_z*dt)
    q = tf_conversions.transformations.quaternion_multiply(q, q_imu)

    # Update the EKF with IMU data
    update_ekf(dt, gyro_x, gyro_y, gyro_z)

    # Convert Euler angles to Quaternion from EKF state
    q_ekf = tf_conversions.transformations.quaternion_from_euler(state[0][0], state[1][0], state[2][0])

    # Create the IMU message
    imu_msg = Imu()
    imu_msg.header.stamp = current_time
    imu_msg.header.frame_id = "imu_link"
    imu_msg.orientation.x = q_ekf[0]
    imu_msg.orientation.y = q_ekf[1]
    imu_msg.orientation.z = q_ekf[2]
    imu_msg.orientation.w = q_ekf[3]
    # You can also set other fields like linear_acceleration, angular_velocity, etc. if available

    # Publish the IMU message
    imu_pub.publish(imu_msg)

if __name__ == '__main__':
    rospy.init_node('imu_node', anonymous=True)
    rospy.Subscriber('imu_array', Float64MultiArray, imu_callback)  # Subscribe to the Arduino data topic

    # Create a publisher for IMU data
    imu_pub = rospy.Publisher('/imu/data', Imu, queue_size=1)

    # Initialize the tf2 broadcaster
    br = tf2_ros.TransformBroadcaster()

    rospy.spin()
