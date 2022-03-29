from abc import abstractmethod
import cv2
import numpy as np


class BaseRobotTrackingBehaviour:
    def __init__(self):
        self.robot_delta = None

    @abstractmethod
    def track(self, image):
        """
        :return: np.array 1x2 in image frame
        """
        pass

    @abstractmethod
    def get_robot_sizes(self):
        pass


class YellowRobotTracking(BaseRobotTrackingBehaviour):
    def __init__(self):
        BaseRobotTrackingBehaviour.__init__(self)
        self.yellow_lower = np.array([25, 50, 50], dtype="uint8")
        self.yellow_upper = np.array([35, 255, 255], dtype="uint8")
        self.connectivity = 8

    def track(self, image):
        copy = image.copy()
        hsv_image = cv2.cvtColor(copy, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_image, self.yellow_lower, self.yellow_upper)

        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask,
                                                                                self.connectivity,
                                                                                cv2.CV_32S)

        robot_stats = stats[1]
        robot_dx = robot_stats[2]
        robot_dy = robot_stats[3]
        self.robot_delta = max(robot_dx, robot_dy)

        return centroids[1]

    def get_robot_sizes(self):
        return self.robot_delta, self.robot_delta
