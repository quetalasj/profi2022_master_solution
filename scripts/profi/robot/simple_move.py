#!/usr/bin/env python


import time
from math import sin
import rospy
from geometry_msgs.msg import Twist
from base_robot import BaseRobot
from profi.camera.camera import CameraFactory
from profi.map_builders.map_builders import IterativeMapBuilder


class SimpleMover(BaseRobot):

    def __init__(self, camera, map_builder):
        BaseRobot.__init__(self, camera, map_builder)

    def algorithm(self):
        start_time = time.time()
        while not rospy.is_shutdown():
            twist_msg = Twist()
            t = time.time() - start_time
            twist_msg.linear.x = 0.1
            twist_msg.angular.z = 0.4 * sin(0.3 * t)
            self.map_builder.get_state_map(self.camera)
            self.cmd_vel_pub.publish(twist_msg)
            self.rate.sleep()


class SimpleMoverFactory:
    @staticmethod
    def create():
        camera = CameraFactory.create_camera()
        map_builder = IterativeMapBuilder()
        return SimpleMover(camera, map_builder)
