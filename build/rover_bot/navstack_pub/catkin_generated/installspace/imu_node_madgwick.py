#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu
import tf_conversions
import tf2_ros
import math

# Global variables to store orientation
current_time = None
previous_time = None
gyro_x_prev = 0.0
gyro_y_prev = 0.0
gyro_z_prev = 0.0
roll = 0.0
pitch = 0.0
yaw = 0.0

# Madgwick filter parameters
beta = 0.1   # Filter gain

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

    # Convert Euler angles to Quaternion
    imu_msg = Imu()
    imu_msg.header.stamp = current_time
    imu_msg.header.frame_id = "imu_link"
    imu_msg.orientation.x = q[0]
    imu_msg.orientation.y = q[1]
    imu_msg.orientation.z = q[2]
    imu_msg.orientation.w = q[3]
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
