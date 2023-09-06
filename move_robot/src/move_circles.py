#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import time
from datetime import datetime
import numpy as np
import csv

global loop_1
loop_1 = 32.0
global loop_2
loop_2 = 22.0
global loop_3
loop_3 = 18.0
global loop_4
loop_4 = 12.0


class TurtlebotMoves():
    def __init__(self):
        # Positions
        self.odom_topic = '/odom'
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry,self.odometryCallback)
        self.position_x, self.position_y = 0,0
        # Velocities
        self.vel_topic = '/cmd_vel'
        self.vel_pub = rospy.Publisher(self.vel_topic, Twist, queue_size=10)
        # Time
        self.time = 0
        self.ini = 0
        # Lists of positions and labels
        self.x_list, self.y_list = [],[]
        self.labels_list = []
        
    def odometryCallback(self,odom_msg):
        self.position_x = odom_msg.pose.pose.position.x
        self.position_y = odom_msg.pose.pose.position.y
        stamp = odom_msg.header.stamp
        if self.time!=0:
            self.time = stamp.secs + stamp.nsecs * 1e-9
        else:
            self.time = stamp.secs + stamp.nsecs * 1e-9
            self.ini = self.time
        
    def move(self):
        # time
        t = self.time - self.ini

        # velocity
        vel_msg = Twist()
        vel_msg.linear.x = 0.4  

        label_loop = True # Default
        if t < loop_1:
            label_loop = True
            vel_msg.angular.z = 0.2
        elif t > loop_1 and t < (loop_1+loop_2):
            label_loop = False
            vel_msg.angular.z = 0.3
        elif t > (loop_1+loop_2) and t < (loop_1+loop_2+loop_3):
            label_loop = True
            vel_msg.angular.z = 0.4
        elif t > (loop_1+loop_2+loop_3) and t < (loop_1+loop_2+loop_3+loop_4):
            label_loop = False
            vel_msg.angular.z = 0.5
        else:
            vel_msg.angular.z = 0.0
        
        self.vel_pub.publish(vel_msg)

        # store position every second
        print('Recording at time ',t)
        self.x_list.append(self.position_x)
        self.y_list.append(self.position_y)
        self.labels_list.append(label_loop)

        return t

    def stop(self):
        # stop robot
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        self.vel_pub.publish(vel_msg)
        # save position data in csv
        path = "./catkin_ws/src/dlrepo/dl_exercises/position_data_circles.csv"
        with open(path,"w+") as f:
            write = csv.writer(f)
            for i,j,k in zip(self.x_list,self.y_list,self.labels_list):
                write.writerow([i,j,k])
            print('DATA SAVED IN: ',path)


if __name__ == '__main__':
    # init node
    node_name = 'move_velocity'
    rospy.init_node(node_name, anonymous=True)
    # Rate of publishing
    rate = rospy.Rate(2)
    # call Turtlebot class
    mover = TurtlebotMoves()
    
    rospy.loginfo('Initializing robot commands ............................')
    time.sleep(2)

    rospy.on_shutdown(mover.stop)

    # store positions and labels
    # while not rospy.is_shutdown():
    t = 0
    while t < (loop_1+loop_2+loop_3+loop_4): # seconds
        t = mover.move()
        rate.sleep()
    else:
        mover.stop

