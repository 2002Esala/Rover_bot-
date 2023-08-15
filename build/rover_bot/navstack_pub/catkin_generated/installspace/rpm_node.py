#!/usr/bin/env python  
import rospy
from geometry_msgs.msg import Twist

def cmd_vel_callback(msg):
    # Access linear velocities
    linear_x = msg.linear.x
    linear_y = msg.linear.y
    linear_z = msg.linear.z

    # Access angular velocities
    angular_x = msg.angular.x
    angular_y = msg.angular.y
    angular_z = msg.angular.z

    wL = (linear_x-(angular_z*0.165/2))/0.08325
    wR = (linear_x+(angular_z*0.165/2))/0.08325

    rospy.loginfo("Received wL message:\n%s", wL) 
    rospy.loginfo("Received wR message:\n%s", wR) 
              

if __name__ == '__main__':
      rospy.init_node('rpm_node')
      rospy.Subscriber('cmd_vel', Twist, cmd_vel_callback)
      rospy.spin()