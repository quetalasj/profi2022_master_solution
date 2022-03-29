from abc import abstractmethod
import cv2
import numpy as np


class BaseMappingBehaviour:
    def __init__(self):
        pass

    @abstractmethod
    def map(self, image):
        """
        :return: binary np.array image_height x image_width
        """
        pass


class ColorMapping(BaseMappingBehaviour):
    def __init__(self):
        BaseMappingBehaviour.__init__(self)
        self.white_borders_lower = np.array([0], dtype="uint8")
        self.white_borders_upper = np.array([10], dtype="uint8")

        self.robot_lower = np.array([25, 50, 50], dtype="uint8")
        self.robot_upper = np.array([35, 255, 255], dtype="uint8")

    def map(self, image):
        copy = image.copy()
        hsv_image = cv2.cvtColor(copy, cv2.COLOR_BGR2HSV)
        mask_borders = cv2.inRange(hsv_image[:, :, 0], self.white_borders_lower, self.white_borders_upper)
        mask_field = cv2.inRange(hsv_image[:, :, 1], self.white_borders_lower, self.white_borders_upper)
        robot_mask = cv2.inRange(hsv_image, self.robot_lower, self.robot_upper)
        dilated_robot_mask = cv2.dilate(robot_mask, np.ones((3, 3), 'uint8'))
        mask_free = mask_field - mask_borders + dilated_robot_mask
        return mask_free
