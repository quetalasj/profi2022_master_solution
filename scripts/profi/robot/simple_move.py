#!/usr/bin/env python


import time
from math import sin
import rospy
from geometry_msgs.msg import Twist
from base_robot import BaseRobot


class SimpleMover(BaseRobot):

    def __init__(self, camera):
        BaseRobot.__init__(self, camera)

    def algorithm(self):
        start_time = time.time()
        while not rospy.is_shutdown():
            twist_msg = Twist()
            t = time.time() - start_time
            twist_msg.linear.x = 0.1
            twist_msg.angular.z = 0.4 * sin(0.3 * t)
            self.cmd_vel_pub.publish(twist_msg)
            self.rate.sleep()
