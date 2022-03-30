#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from base_robot import BaseRobot
from profi.camera.camera import CameraFactory
from profi.map_builders.map_builders import ConvolveMapBuilder
from profi.path_planners.rrt import RRT, RRTMaxSteer
from profi.decision_making.closest_sock_decision_maker import ClosestSockDecisionMaker
from profi.path_smoothers.short_cut_smoother import ShortCutSmoother
from profi.controllers.visual_controller import VisualController


class SimpleMover(BaseRobot):

    def __init__(self, camera, map_builder, path_planner, decision_maker, motion_controller):
        BaseRobot.__init__(self, camera, map_builder, path_planner, decision_maker, motion_controller)
        self.new_round = True
        self.move_robot_to_goal = None
        self.choose_next_goal = None
        self.find_new_path = None
        self.rotate_robot = None
        self.move_robot = None
        self.move_robot_back = None
        self.current_path_point_i = None
        self.choose_next_path_point = None

    def init_states(self):
        self.choose_next_goal = True
        self.find_new_path = True
        self.rotate_robot = True
        self.move_robot = False
        self.move_robot_to_goal = False
        self.move_robot_back = False
        self.current_path_point_i = 0
        self.choose_next_path_point = True

    def check_simulation_ready(self):
        odom_is_ready = self.orientation is not None
        socks_are_ready = self.is_sock_taken is not None
        return odom_is_ready and socks_are_ready

    def choose_goal(self, robot_pose, current_goal):
        if self.choose_next_goal:
            goal_pose = self.decision_maker.get_goal(robot_pose, self.camera)
            self.choose_next_goal = False
            return goal_pose
        return current_goal

    def find_path(self, robot_pose, goal_pose, current_path):
        if self.find_new_path:
            c_space = self.map_builder.get_state_map(self.camera, robot_pose, goal_pose)
            path = self.path_planner.get_path(robot_pose, goal_pose, c_space)
            self.camera.path_to_plot = path
            self.find_new_path = False
            return path
        return current_path

    def choose_path_point(self, path, goal_point):
        self.current_path_point_i = self.current_path_point_i + 1
        if self.current_path_point_i == len(path) - 1:
            return goal_point
        return path[self.current_path_point_i]

    def rotate_to_point(self, robot_pose, point):
        v, w, delta_w = self.motion_controller.rotate_on_place(self.orientation, robot_pose, point)
        if abs(delta_w) < 0.1:
            return 0, 0
        return v, w

    def point_distance(self, p1, p2):
        return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def move_to_point(self, robot_pose, point):
        v, w = self.motion_controller.move_robot_forward(self.orientation, robot_pose, point)
        distance = self.point_distance(robot_pose, point)
        if abs(distance) < 10:
            return 0, 0
        return v, w

    def move_back(self, robot_pose):
        v, w = -0.1, 0
        distance = self.point_distance(robot_pose, self.move_back_pose)
        if abs(distance) > 2:
            return 0, 0
        return v, w

    def prepare_next_round(self):
        if self.new_round:
            print("round init")
            self.init_states()
            self.new_round = False

    def to_rotate_state(self):
        print("to rotate")
        self.choose_next_path_point = False
        self.rotate_robot = True

    def to_move_to_point_state(self):
        print("to point")
        self.rotate_robot = False
        self.move_robot_to_goal = False
        self.move_robot = True

    def to_move_to_goal_state(self):
        print('to goal')
        self.rotate_robot = False
        self.move_robot_to_goal = True
        self.move_robot = False

    def to_choose_point_state(self):
        print('choose nex point')
        self.move_robot = False
        self.move_robot_to_goal = False
        self.choose_next_path_point = True

    def to_move_back_state(self, current_robot_pose):
        print('to back')
        self.move_back_pose = current_robot_pose
        self.move_robot_to_goal = False
        self.move_robot_back = True

    def algorithm(self):
        goal = None
        path = None
        v, w = 0, 0
        while not rospy.is_shutdown():
            if self.check_simulation_ready():
                self.prepare_next_round()
                robot_pose = self.camera.get_robot_pose()
                goal = self.choose_goal(robot_pose, goal)
                path = self.find_path(robot_pose, goal, path)
                if self.choose_next_path_point:
                    current_point_to_follow = tuple(self.choose_path_point(path, goal))
                    point_is_goal = False
                    self.to_rotate_state()
                    if current_point_to_follow == goal:
                        point_is_goal = True

                if self.rotate_robot:
                    v, w = self.rotate_to_point(robot_pose, current_point_to_follow)
                    if w == 0:
                        if point_is_goal:
                            self.to_move_to_goal_state()
                        else:
                            self.to_move_to_point_state()
                elif self.move_robot:
                    v, w = self.move_to_point(robot_pose, current_point_to_follow)
                    if v == 0:
                        self.to_choose_point_state()
                elif self.move_robot_to_goal:
                    v, w = self.move_to_point(robot_pose, current_point_to_follow)
                    if self.is_sock_taken:
                        self.decision_maker.mark_sock_as_taken()
                        self.to_move_back_state(robot_pose)
                elif self.move_robot_back:
                    v, w = self.move_back(robot_pose)
                    if v == 0:
                        self.move_robot_back = False
                        self.new_round = True

                self.send_message(v, w)
            self.rate.sleep()

    def send_message(self, v, w):
        twist_msg = Twist()
        twist_msg.linear.x = v
        twist_msg.angular.z = w
        self.cmd_vel_pub.publish(twist_msg)

class SimpleMoverFactory:
    @staticmethod
    def create():
        camera = CameraFactory.create_camera()
        map_builder = ConvolveMapBuilder()
        # path_planner = ShortCutSmoother(RRT())
        path_planner = ShortCutSmoother(RRTMaxSteer(max_steer=50))
        decision_maker = ClosestSockDecisionMaker()
        motion_controller = VisualController()
        return SimpleMover(camera, map_builder, path_planner, decision_maker, motion_controller)
