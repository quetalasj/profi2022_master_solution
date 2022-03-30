#!/usr/bin/env python
from abc import abstractmethod
import rospy
from geometry_msgs.msg import Twist


class BaseRobot:
    def __init__(self, camera, map_builder, path_planner):
        self.camera = camera
        self.map_builder = map_builder
        self.path_planner = path_planner
        self.odometry = None

        rospy.init_node('simple_mover', anonymous=True)
        rospy.on_shutdown(self.shutdown)

        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(30)

    def spin(self):
        self.algorithm()

    def shutdown(self):
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

    @abstractmethod
    def algorithm(self):
        pass
