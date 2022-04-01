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


class Conditions:
    point_is_goal = "POINT_IS_GOAL"
    point_is_not_goal = "POINT_IS_NOT_GOAL"
    point_in_collision = "POINT_IN_COLLISION"
    path_not_found = "PATH_NOT_FOUND"

class States:
    new_round = "NEW_ROUND"
    choose_next_sock = "CHOOSE_NEXT_SOCK"
    find_new_path = "FIND_NEW_PATH"
    choose_next_path_point = "CHOOSE_NEXT_PATH_POINT"
    rotation_to_point = "ROTATION_TO_POINT"
    rotation_to_goal = "ROTATION_TO_GOAL"
    move_to_point = "MOVE_TO_POINT"
    move_to_goal = "MOVE_TO_GOAL"
    move_back = "MOVE_BACK"

    @staticmethod
    def next_state(state, condition=None):
        if state == States.new_round:
            return States.choose_next_sock
        if state == States.choose_next_sock:
            return States.find_new_path
        if state == States.find_new_path and condition == Conditions.path_not_found:
            return States.new_round
        if state == States.find_new_path:
            return States.choose_next_path_point

        if state == States.choose_next_path_point and condition == Conditions.point_is_not_goal:
            return States.rotation_to_point
        if state == States.choose_next_path_point and condition == Conditions.point_is_goal:
            return States.rotation_to_goal

        if state == States.rotation_to_point:
            return States.move_to_point
        if state == States.rotation_to_goal:
            return States.move_to_goal

        if (state == States.move_to_point or state == States.move_to_goal) \
                and (condition == Conditions.point_in_collision):
            return States.move_back

        if state == States.move_to_point:
            return States.choose_next_path_point

        if state == States.move_to_goal:
            return States.move_back

        if state == States.move_back:
            return States.new_round

        print("Unknown state", state, condition)


class CollisionChecker:
    def __init__(self, distance_f):
        self.collision_time = None
        self.first_position = None
        self.distance_f = distance_f

    def check(self, pos_to_save, second_position, prev_v, time_threshold, distance_thresh):
        if self.collision_time is None or self.first_position is None:
            print("START COLLISION TRACKING")
            self.collision_time = rospy.get_time()
            self.first_position = pos_to_save

        if (rospy.get_time() - self.collision_time) > time_threshold:
            distance = self.distance_f(second_position, self.first_position)
            if distance < distance_thresh and abs(prev_v) > 0:
                print("!!!COLLISION!!!")
                print("DISTANCE:")
                print(distance)
                print("FIRST")
                print(self.first_position)
                print("SECOND")
                print(second_position)
                self.reset()
                return True
            else:
                self.reset()
        return False

    def reset(self):
        print("RESET COLLISION")
        self.collision_time = None
        self.first_position = None


class SimpleMover(BaseRobot):
    def __init__(self, camera, map_builder, path_planner, decision_maker, motion_controller):
        BaseRobot.__init__(self, camera, map_builder, path_planner, decision_maker, motion_controller)
        self.state = States.new_round
        self.current_path_point_i = 0
        self.move_back_pose = None

        self.distance_thresh_goal = 0.05
        self.time_thresh_goal = 5
        self.collision_checker = CollisionChecker(self.point_distance)
        self.move_back_distance = 5

    def prepare_next_round(self):
        if self.state == States.new_round:
            print(self.state)
            self.current_path_point_i = 0
            self.move_back_pose = None
            self.state = States.next_state(self.state)

    def check_simulation_ready(self):
        odom_is_ready = self.orientation is not None
        socks_are_ready = self.is_sock_taken is not None
        return odom_is_ready and socks_are_ready

    def choose_sock(self, robot_pose, current_goal):
        if self.state == States.choose_next_sock:
            print(self.state)
            goal_pose = self.decision_maker.get_goal(robot_pose, self.camera)
            self.state = States.next_state(self.state)
            return goal_pose
        return current_goal

    def find_path(self, robot_pose, goal_pose, current_path):
        if self.state == States.find_new_path:
            print(self.state)
            c_space = self.map_builder.get_state_map(self.camera, robot_pose, goal_pose)
            path = self.path_planner.get_path(robot_pose, goal_pose, c_space)
            if path is None:
                self.state = States.next_state(self.state, Conditions.path_not_found)
                return current_path
            self.camera.path_to_plot = path
            self.camera.full_path_to_plot = self.path_planner.full_path
            self.state = States.next_state(self.state)
            return path
        return current_path

    def choose_path_point(self, path, current_point):
        point_to_follow = current_point
        if self.state == States.choose_next_path_point:
            print(self.state)
            self.current_path_point_i += 1
            if self.current_path_point_i == len(path) - 1:
                point_to_follow = path[-1]
                self.state = States.next_state(self.state, Conditions.point_is_goal)
            else:
                point_to_follow = path[self.current_path_point_i]
                self.state = States.next_state(self.state, Conditions.point_is_not_goal)
            print("POINT", self.current_path_point_i, "/", len(path)-1)
        return tuple(point_to_follow)

    def rotate_robot(self, robot_pose, point, prev_v, prev_w):
        v, w = prev_v, prev_w
        if self.state == States.rotation_to_point or self.state == States.rotation_to_goal:
            print(self.state)
            v, w, delta_w = self.motion_controller.rotate_on_place(self.orientation, robot_pose, point)
            if abs(delta_w) < 0.1:
                self.state = States.next_state(self.state)
                v, w = 0, 0
        return v, w

    def point_distance(self, p1, p2):
        return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    def move_robot(self, robot_pose, point, prev_v, prev_w):
        v, w = prev_v, prev_w
        if self.state == States.move_to_point or self.state == States.move_to_goal:
            v, w = self.motion_controller.move_robot_forward(self.orientation, robot_pose, point)
            collision_points = (self.odometry_x, self.odometry_y)

            if self.state == States.move_to_point:
                print(self.state)
                distance = self.point_distance(robot_pose, point)
                if abs(distance) < 5:
                    self.collision_checker.reset()
                    self.state = States.next_state(self.state)
                    v, w = 0, 0
                if self.collision_checker.check(collision_points, collision_points, prev_v, self.time_thresh_goal, self.distance_thresh_goal):
                    self.decision_maker.mark_sock_as_taken()
                    self.collision_checker.reset()
                    self.state = States.next_state(self.state, Conditions.point_in_collision)
                    v, w = 0, 0

            if self.state == States.move_to_goal:
                print(self.state)
                if self.is_sock_taken or self.collision_checker.check(collision_points, collision_points, prev_v, self.time_thresh_goal, self.distance_thresh_goal):
                    self.decision_maker.mark_sock_as_taken()
                    self.collision_checker.reset()
                    self.state = States.next_state(self.state, Conditions.point_in_collision)
                    v, w = 0, 0

            self.move_back_pose = robot_pose
        return v, w

    def move_robot_back(self, robot_pose, prev_v, prev_w):
        v, w = prev_v, prev_w
        if self.state == States.move_back:
            print(self.state)
            v, w = -0.1, 0
            collision_points = (self.odometry_x, self.odometry_y)
            distance = self.point_distance(robot_pose, self.move_back_pose)
            is_collision = self.collision_checker.check(collision_points, collision_points, prev_v,
                                                        self.time_thresh_goal, self.distance_thresh_goal)
            if abs(distance) > self.move_back_distance or is_collision:
                self.collision_checker.reset()
                self.state = States.next_state(self.state)
                v, w = 0, 0
        return v, w

    def algorithm(self):
        sock = None
        path = None
        current_point = None
        v, w = 0, 0
        while not rospy.is_shutdown():
            if self.check_simulation_ready():
                self.prepare_next_round()
                robot_pose = self.camera.get_robot_pose()
                sock = self.choose_sock(robot_pose, sock)
                path = self.find_path(robot_pose, sock, path)
                current_point = self.choose_path_point(path, current_point)
                v, w = self.rotate_robot(robot_pose, current_point, v, w)
                v, w = self.move_robot(robot_pose, current_point, v, w)
                v, w = self.move_robot_back(robot_pose, v, w)
                self.send_message(v, w)
            self.rate.sleep()

    def send_message(self, v, w):
        print("message")
        print("v", v, ", w", w)
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
