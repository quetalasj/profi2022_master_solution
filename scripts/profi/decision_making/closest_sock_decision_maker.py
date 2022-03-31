import numpy as np
from base_decision_maker import BaseDecisionMaker


class ClosestSockDecisionMaker(BaseDecisionMaker):
    def __init__(self):
        BaseDecisionMaker.__init__(self)
        self.taken_socks = []
        self.temporary_ignored_socks = []
        self.next_sock = None
        self.num_socks = None

    def set_num_socks(self, num_socks):     # TODO: check that always find all socks
        if self.num_socks is None:
            self.num_socks = num_socks

    def get_goal(self, start_point, camera):
        self.release_ignored_socks()
        new_socks_centers = self.ignore_socks(camera.socks_centers, self.taken_socks)
        new_socks_centers = self.ignore_socks(new_socks_centers, self.temporary_ignored_socks)

        closest_sock = np.argmin(
            np.sqrt(
                np.sum((new_socks_centers - np.array([start_point[0], start_point[1]]))**2, axis=1)
            )
        )
        print(closest_sock)
        self.next_sock = closest_sock
        closest_cx, closest_cy = camera.socks_centers[closest_sock]
        return int(round(closest_cx)), int(round(closest_cy))

    def release_ignored_socks(self):
        if self.num_socks is not None:
            residual_socks = self.num_socks - len(self.taken_socks) - len(self.temporary_ignored_socks)
            if residual_socks <= 0:  # TODO: do something more?
                self.temporary_ignored_socks = []

    def mark_sock_as_taken(self):
        self.taken_socks.append(self.next_sock)

    def temporary_ignore_sock(self):
        self.temporary_ignored_socks.append(self.next_sock)

    def ignore_socks(self, socks_centers, ignored_socks):
        new_socks_centers = socks_centers.copy()
        for taken in ignored_socks:
            new_socks_centers[taken][0] = np.inf
            new_socks_centers[taken][1] = np.inf
        return new_socks_centers



