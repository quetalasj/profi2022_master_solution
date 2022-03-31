import numpy as np
import math


class VisualController:

    def __init__(self):
        self.v_lin = 0.2
        self.k_rotation_on_place = 0.2
        self.k_rotation_dynamic = 0.2

        self.k_dot = 0.2
        self.k_i = 0.0001

        self.prev_delta_w = 0
        self.E_rot = 0

    def rotate_on_place(self, robot_orientation, robot_pose, goal_pose):
        w, delta_w = self.rotate_robot(robot_orientation, robot_pose, goal_pose, self.k_rotation_on_place)
        v = 0
        return v, w, delta_w

    def move_robot_forward(self, robot_orientation, robot_pose, goal_pose):
        w, delta_w = self.rotate_robot(robot_orientation, robot_pose, goal_pose, self.k_rotation_dynamic)
        v = self.v_lin
        return v, w

    def rotate_robot(self, robot_orientation, robot_pose, goal_pose, k):
        goal_orientation = self.estimate_goal_orientation(goal_pose, robot_pose)
        delta_w = self.angle_difference_rad(goal_orientation, robot_orientation)
        # print(delta_w)
        delta_w_dot = delta_w - self.prev_delta_w
        self.E_rot += delta_w
        w = k * delta_w + self.k_dot * delta_w_dot + self.k_i * self.E_rot
        self.prev_delta_w = delta_w
        return w, delta_w

    def estimate_goal_orientation(self, goal_pose, robot_pose):
        dy = robot_pose[1] - goal_pose[1]
        dx = goal_pose[0] - robot_pose[0]
        return math.atan2(dy, dx)

    def angle_difference_rad(self, angle1, angle2):
        delta_angle = angle1 - angle2
        return (delta_angle + np.pi) % (2 * np.pi) - np.pi

    # def angle_difference(self, angle1, angle2):
    #     delta_angle = angle1 - angle2
    #     delta_angle = (delta_angle + 180) % 360 - 180
    #     return delta_angle
