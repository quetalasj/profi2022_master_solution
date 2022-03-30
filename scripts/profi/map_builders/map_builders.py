from abc import abstractmethod
import cv2
import numpy as np
from scipy.signal import convolve2d


class BaseMapBuilder:
    def __init__(self):
        pass

    @abstractmethod
    def get_state_map(self, camera, start_point, goal_point):
        pass

    def normalize_image(self, img):
        dims = img.shape
        env = np.ones(dims)
        z = np.where(img < 0.1)
        env[z] = 0.0
        return env


class ConvolveMapBuilder(BaseMapBuilder):
    def __init__(self):
        BaseMapBuilder.__init__(self)
        self.state_map_height = None
        self.state_map_width = None

    def exclude_goal_sock(self, camera, goal_point):
        free_map = camera.free_map
        cv2.imwrite("/workspace/map-with-socks.png", free_map)
        sock_label = 1 + np.argmin(np.abs(camera.socks_centers - np.array([goal_point[0], goal_point[1]])), axis=0)[0]

        socks_centers = camera.mask_socks_centers.copy()
        socks_centers[camera.socks_labels != sock_label] = 0

        free_map = free_map + cv2.dilate(socks_centers, np.ones((8, 8), 'uint8'))
        return free_map

    def create_state_map(self, camera, start_point, goal_point):
        free_map = self.exclude_goal_sock(camera, goal_point)
        cv2.imwrite("/workspace/map-without-goal-sock.png", free_map)

        # 1 is obstacle, 0 is free
        free_map = 1 - np.clip(free_map, 0, 1)
        convolution_result = convolve2d(free_map, np.ones((10, 10)), fillvalue=1, mode='same')
        state_map = self.normalize_image(convolution_result)[:, :, None]
        return state_map

    def get_state_map(self, camera, start_point, goal_point):
        state_map = self.create_state_map(camera, start_point, goal_point)
        cv2.imwrite("/workspace/final-c-space-1d.png", state_map[:, :, 0]*255)
        return state_map

