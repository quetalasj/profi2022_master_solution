from abc import abstractmethod
import cv2
import numpy as np


class BaseMappingBehaviour:
    def __init__(self):
        pass

    @abstractmethod
    def map(self, image, robot_mask):
        """
        :return: binary np.array image_height x image_width
        """
        pass


class ColorMapping(BaseMappingBehaviour):
    def __init__(self):
        BaseMappingBehaviour.__init__(self)
        self.white_borders_lower = np.array([0], dtype="uint8")
        self.white_borders_upper = np.array([10], dtype="uint8")

    def map(self, image, robot_mask):
        copy = image.copy()
        hsv_image = cv2.cvtColor(copy, cv2.COLOR_BGR2HSV)
        mask_borders = cv2.inRange(hsv_image[:, :, 0], self.white_borders_lower, self.white_borders_upper)
        mask_field = cv2.inRange(hsv_image[:, :, 1], self.white_borders_lower, self.white_borders_upper)

        dilated_robot_mask = cv2.dilate(robot_mask, np.ones((5, 5), 'uint8'))
        mask_free = mask_field - mask_borders + dilated_robot_mask

        return mask_free
