#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist


if __name__ == '__main__':

    rospy.init_node('test_node', anonymous=True)
    rate = rospy.Rate(2)

    vel_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)

    while not rospy.is_shutdown():
        vel_msg = Twist()
        vel_msg.linear.x = 0.5
        vel_msg.angular.z = 0.5
        vel_pub.publish(vel_msg)
        rate.sleep()
