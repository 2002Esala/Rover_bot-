#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray

def imu_callback(data):
    # Process the received data
    array_data = data.data
    acceleration_x = array_data[0]
    acceleration_y = array_data[1]
    acceleration_z = array_data[2]
    gyro_x = array_data[3]
    gyro_y = array_data[4]
    gyro_z = array_data[5]

    # Print the received values (you can do any other processing here)
    rospy.loginfo("Received IMU data:")
    rospy.loginfo("Acceleration: x={}, y={}, z={}".format(acceleration_x, acceleration_y, acceleration_z))
    rospy.loginfo("Gyroscope: x={}, y={}, z={}".format(gyro_x, gyro_y, gyro_z))

def imu_receiver():
    rospy.init_node('imu_receive', anonymous=True)
    rospy.Subscriber('imu_array', Float64MultiArray, imu_callback)

    # Spin until interrupted
    rospy.spin()

if __name__ == '__main__':
    try:
        imu_receiver()
    except rospy.ROSInterruptException:
        pass
