import cv2
from sensor_msgs.msg import Image
import rospy
from cv_bridge import CvBridge, CvBridgeError

from socks_behaviours import ColorCirclesSocks
from tracking_behaviour import YellowRobotTracking
from mapping_behavour import ColorMapping
import numpy as np


class Camera:
    def __init__(self, socks_behaviour, robot_tracking_behaviour, mapping_behaviour):
        rospy.Subscriber("diff_drive_robot/camera1/image_raw", Image, self.camera_cb)
        self.cv_bridge = CvBridge()
        self.socks_behaviour = socks_behaviour
        self.robot_tracking_behaviour = robot_tracking_behaviour
        self.mapping_behaviour = mapping_behaviour
        self.save_image = True

        self.socks_centers = None
        self.socks_contours = None
        self.mask_socks_centers = None
        self.mask_socks_orange = None
        self.robot_cx = None
        self.robot_cy = None
        self._free_map = None
        self.path_to_plot = None
        self.socks_stats = None
        self.num_socks = None
        self.socks_labels = None
        self.full_path_to_plot = None
        self.robot_mask = None

    def camera_cb(self, msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            if self.save_image:
                cv2.imwrite("/workspace/frame-image.png", cv_image)
                self.save_image = False

            self.socks_centers, self.socks_contours, self.mask_socks_centers, self.socks_stats, self.num_socks, \
                self.socks_labels, self.mask_socks_orange = self.socks_behaviour.detect(cv_image)

            self.robot_cx, self.robot_cy, self.robot_mask = self.robot_tracking_behaviour.track(cv_image)
            self._free_map = self.mapping_behaviour.map(cv_image, self.robot_mask)

            self.visualiza(cv_image)

        except CvBridgeError, e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def get_robot_pose(self):
        return int(round(self.robot_cx)), int(round(self.robot_cy))

    def get_robot_sizes(self):
        return self.robot_tracking_behaviour.get_robot_sizes()

    @property
    def free_map(self):
        return self._free_map

    def visualiza(self, image):
        copy = image.copy()
        if self.robot_cx is not None:
            copy = cv2.circle(copy, (int(self.robot_cx), int(self.robot_cy)), 1, (0, 0, 255), 2)
        # if self.socks_contours is not None:
        #     cv2.drawContours(copy, self.socks_contours, -1, (0, 255, 0), 1)
        #     for sock in self.socks_centers:
        #         copy = cv2.circle(copy, (int(sock[0]), int(sock[1])), 1, (0, 255, 0), 1)
        # if self._free_map is not None:
        #     contours, hierarchy = cv2.findContours(self._free_map,
        #                                            cv2.RETR_LIST,
        #                                            cv2.CHAIN_APPROX_NONE)
        #     cv2.drawContours(copy, contours, -1, (255, 0, 0), 1)

        if self.full_path_to_plot is not None:
            for i in range(1, len(self.full_path_to_plot)):
                copy = cv2.line(copy,
                                (self.full_path_to_plot[i - 1][0], self.full_path_to_plot[i - 1][1]),
                                (self.full_path_to_plot[i][0], self.full_path_to_plot[i][1]),
                                (255, 0, 0), 3)
                # copy = cv2.circle(copy, (int(self.path_to_plot[i][0]), int(self.path_to_plot[i][1])), 1, (0, 0, 255), 2)
        if self.path_to_plot is not None:
            for i in range(1, len(self.path_to_plot)):
                copy = cv2.line(copy,
                                (self.path_to_plot[i - 1][0], self.path_to_plot[i - 1][1]),
                                (self.path_to_plot[i][0], self.path_to_plot[i][1]),
                                (0, 255, 0), 2)
                copy = cv2.circle(copy, (int(self.path_to_plot[i][0]), int(self.path_to_plot[i][1])), 1, (0, 0, 255), 2)
        cv2.imshow("Robot", copy)
        cv2.waitKey(1)


class CameraFactory:
    @staticmethod
    def create_camera():
        socks_behaviour = ColorCirclesSocks()
        robot_tracking_behaviour = YellowRobotTracking()
        mapping_behaviour = ColorMapping()
        return Camera(socks_behaviour, robot_tracking_behaviour, mapping_behaviour)
