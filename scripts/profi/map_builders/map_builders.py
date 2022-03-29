from abc import abstractmethod
import cv2
import numpy as np


class BaseMapBuilder:
    @abstractmethod
    def get_state_map(self, camera):
        pass


class IterativeMapBuilder(BaseMapBuilder):
    def __init__(self):
        self.state_map_height = None
        self.state_map_width = None
        self.state_map = None

    def create_state_map(self, camera):
        free_map = camera.free_map
        robot_dx, robot_dy = camera.get_robot_sizes()

        cell_height = int(np.round(robot_dy))
        cell_width = int(np.round(robot_dx))

        state_map_height = int(free_map.shape[0] // cell_height)
        state_map_width = int(free_map.shape[1] // cell_width)
        self.state_map = np.zeros((state_map_height + 1, state_map_width + 1))

        cv2.imwrite("/workspace/free_map.png", free_map)

        for i in range(1, self.state_map.shape[0]):
            row_lower = (i - 1) * cell_height
            row_upper = min(i * cell_height, free_map.shape[0])
            for j in range(1, self.state_map.shape[1]):
                column_lower = (j - 1) * cell_width
                column_upper = min(j * cell_width, free_map.shape[1])
                is_free = np.all(free_map[row_lower: row_upper, column_lower: column_upper] > 0)
                self.state_map[i, j] = 0 if is_free else 255

    def get_state_map(self, camera):
        if self.state_map is None:
            self.create_state_map(camera)
            cv2.imwrite("/workspace/state_map.png", self.state_map)

        return self.state_map

