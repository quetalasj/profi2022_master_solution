import numpy as np
import math


class VisualController:

    def __init__(self):
        self.k_rotation_on_place = 0.1
        self.k_rotation_dynamic = 0.1

    def rotate_on_place(self, robot_orientation, robot_pose, goal_pose):
        w, delta_w = self.rotate_robot(robot_orientation, robot_pose, goal_pose, self.k_rotation_on_place)
        v = 0
        return v, w, delta_w

    def move_robot_forward(self, robot_orientation, robot_pose, goal_pose):
        w, delta_w = self.rotate_robot(robot_orientation, robot_pose, goal_pose, self.k_rotation_dynamic)
        v = 0.1
        return v, w

    def rotate_robot(self, robot_orientation, robot_pose, goal_pose, k):
        goal_orientation = self.estimate_goal_orientation(goal_pose, robot_pose)
        delta_w = self.angle_difference(goal_orientation, robot_orientation)
        w = k * delta_w
        return w, delta_w

    def estimate_goal_orientation(self, goal_pose, robot_pose):
        dy = robot_pose[1] - goal_pose[1]
        dx = goal_pose[0] - robot_pose[0]
        return math.atan2(dy, dx)

    def angle_difference_rad(self, angle1, angle2):
        delta_angle = angle1 - angle2
        return (delta_angle + np.pi) % (2 * np.pi) - np.pi

    def angle_difference(self, angle1, angle2):
        delta_angle = angle1 - angle2
        delta_angle = (delta_angle + 180) % 360 - 180
        return delta_angle
