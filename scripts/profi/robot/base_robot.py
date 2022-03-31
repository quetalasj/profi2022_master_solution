#!/usr/bin/env python
from abc import abstractmethod
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Bool

class BaseRobot:
    def __init__(self, camera, map_builder, path_planner, decision_maker, motion_controller):
        self.camera = camera
        self.map_builder = map_builder
        self.path_planner = path_planner
        self.decision_maker = decision_maker
        self.motion_controller = motion_controller
        self.orientation = None
        self.is_sock_taken = None
        self.odometry_x = None
        self.odometry_y = None

        rospy.init_node('simple_mover', anonymous=True)
        rospy.on_shutdown(self.shutdown)

        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.socks_monitor_sub = rospy.Subscriber("/monitor/is_sock_taken", Bool, self.socks_callback)
        self.rate = rospy.Rate(30)

    def cvt_angle_to_atan2(self, angle):
        new_angle = angle + np.pi / 2
        if new_angle > np.pi:
            new_angle = angle - 3 * np.pi / 2
        return new_angle

    def odom_callback(self, msg):
        orientation = msg.pose.pose.orientation
        orientation = euler_from_quaternion([
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        ])[2]
        self.orientation = self.cvt_angle_to_atan2(orientation)
        self.odometry_x = msg.pose.pose.position.x
        self.odometry_y = msg.pose.pose.position.y

    def socks_callback(self, msg):
        self.is_sock_taken = msg.data
        if self.is_sock_taken:
            print("Goal is taken")

    def spin(self):
        self.algorithm()

    def shutdown(self):
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

    @abstractmethod
    def algorithm(self):
        pass
