from abc import abstractmethod
import cv2
import numpy as np


class BaseRobotTrackingBehaviour:
    def __init__(self):
        pass

    @abstractmethod
    def track(self, image):
        """
        :return: np.array 1x2 in image frame
        """
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
        return centroids[1]
