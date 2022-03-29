from abc import abstractmethod
import cv2
import numpy as np


class BaseSocksBehaviour:
    def __init__(self):
        pass

    @abstractmethod
    def detect(self, image):
        """
        :return: (socks_coords (nx2), socks_contours)
        """
        pass


class ColorCirclesSocks(BaseSocksBehaviour):
    def __init__(self):
        BaseSocksBehaviour.__init__(self)

        self.blue_lower = np.array([100, 50, 50], dtype="uint8")
        self.blue_upper = np.array([130, 255, 255], dtype="uint8")

        self.orange_lower = np.array([15, 50, 50], dtype="uint8")
        self.orange_upper = np.array([23, 255, 255], dtype="uint8")

        self.connectivity = 8

    def detect(self, image):
        copy = image.copy()
        hsv_image = cv2.cvtColor(copy, cv2.COLOR_BGR2HSV)
        mask_socks_blue = cv2.inRange(hsv_image, self.blue_lower, self.blue_upper)
        mask_socks_orange = cv2.inRange(hsv_image, self.orange_lower, self.orange_upper)
        socks_head = cv2.bitwise_and(copy, copy, mask=mask_socks_blue)
        socks_full = cv2.bitwise_and(copy, copy, mask=(mask_socks_blue | mask_socks_orange))

        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask_socks_blue,
                                                                               self.connectivity,
                                                                               cv2.CV_32S)
        socks_centers = centroids[1:num_labels]  # 0 is background
        contours, hierarchy = cv2.findContours(mask_socks_blue | mask_socks_orange,
                                               cv2.RETR_EXTERNAL,
                                               cv2.CHAIN_APPROX_SIMPLE)
        good_contours = []
        for cnt in contours:
            if cv2.contourArea(cnt) > 50:
                good_contours.append(cnt)

        return socks_centers, good_contours


        #mask = cv2.inRange(hsv_image, self.blue_lower, self.blue_upper)
        # res = cv2.bitwise_and(copy, copy, mask=mask)
        # # Find contours
        # cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        # # Extract contours
        # cnts = cnts[0]

        # # Iterate through contours and filter by the number of vertices
        # socks = []
        # for c in cnts:
        #     perimeter = cv2.arcLength(c, True)
        #     approx = cv2.approxPolyDP(c, 0.1 * perimeter, True)
        #     if len(approx) > 1:
        #         # cv2.drawContours(copy, [c], -1, (36, 255, 12), -1)
        #         M = cv2.moments(approx)
        #         cx = int(M['m10'] / M['m00'])
        #         cy = int(M['m01'] / M['m00'])
        #         # print(cx, cy)
        #         copy = cv2.circle(copy, (cx, cy), 1, (0, 255, 0), 2)
        #         socks.append((cx, cy))
