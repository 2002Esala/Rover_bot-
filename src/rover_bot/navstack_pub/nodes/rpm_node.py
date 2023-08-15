#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

def cmd_vel_callback(msg):
    # Access linear and angular velocities
    linear_x = msg.linear.x
    angular_z = msg.angular.z

    # Calculate left and right wheel RPMs
    wheel_radius = 0.08325 / 2  # Assuming radius in meters
    distance_between_wheels = 0.165  # Assuming distance in meters

    wL = (linear_x - (angular_z * distance_between_wheels / 2)) / wheel_radius
    wR = (linear_x + (angular_z * distance_between_wheels / 2)) / wheel_radius

    # Create an array message to publish wL and wR
    rpm_array = Float32MultiArray(data=[wL, wR])

    # Publish the array on the 'rpm_chatter' topic
    rpm_publisher.publish(rpm_array)

if __name__ == '__main__':
    rospy.init_node('rpm_node')

    # Create a publisher for the 'rpm_chatter' topic
    rpm_publisher = rospy.Publisher('rpm_chatter', Float32MultiArray, queue_size=10)

    rospy.Subscriber('cmd_vel', Twist, cmd_vel_callback)
    rospy.spin()
