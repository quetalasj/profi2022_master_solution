import numpy as np
from base_decision_maker import BaseDecisionMaker


class ClosestSockDecisionMaker(BaseDecisionMaker):
    def __init__(self):
        BaseDecisionMaker.__init__(self)

    def get_goal(self, start_point, camera):
        closest_sock = np.argmin(
            np.sqrt(
                np.sum(
                    (camera.socks_centers - np.array([start_point[0], start_point[1]]))**2,
                    axis=1)
            )
        )
        closest_sock = 1
        closest_cx, closest_cy = camera.socks_centers[closest_sock]
        return int(round(closest_cx)), int(round(closest_cy)), int(0)

