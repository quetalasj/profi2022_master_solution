#!/usr/bin/env python


import time
from math import sin
import rospy
from geometry_msgs.msg import Twist
from base_robot import BaseRobot
from profi.camera.camera import CameraFactory
from profi.map_builders.map_builders import ConvolveMapBuilder
from profi.path_planners.rrt import RRT
from profi.decision_making.closes_sock_decision_maker import ClosestSockDecisionMaker
from profi.path_smoothers.short_cut_smoother import ShortCutSmoother


class SimpleMover(BaseRobot):

    def __init__(self, camera, map_builder, path_planner, decision_maker):
        BaseRobot.__init__(self, camera, map_builder, path_planner, decision_maker)

    def algorithm(self):
        start_time = time.time()
        path = None
        while not rospy.is_shutdown():
            twist_msg = Twist()
            t = time.time() - start_time
            twist_msg.linear.x = 0.1
            twist_msg.angular.z = 0.4 * sin(0.3 * t)
            if path is None:
                start_point = self.camera.get_robot_pose()
                goal_point = self.decision_maker.get_goal(start_point, self.camera)
                print("Start: ", start_point)
                print("Goal: ", goal_point)
                c_space = self.map_builder.get_state_map(self.camera, start_point, goal_point)
                path = self.path_planner.get_path(start_point, goal_point, c_space)
                self.camera.path_to_plot = path
            # self.cmd_vel_pub.publish(twist_msg)
            self.rate.sleep()


class SimpleMoverFactory:
    @staticmethod
    def create():
        camera = CameraFactory.create_camera()
        map_builder = ConvolveMapBuilder()
        path_planner = ShortCutSmoother(RRT())
        decision_maker = ClosestSockDecisionMaker()
        return SimpleMover(camera, map_builder, path_planner, decision_maker)
