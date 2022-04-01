import numpy as np
from base_decision_maker import BaseDecisionMaker


class ClosestSockDecisionMaker(BaseDecisionMaker):
    def __init__(self):
        BaseDecisionMaker.__init__(self)
        self.taken_socks = []
        self.next_sock = None

    def get_goal(self, start_point, camera):
        new_socks_centers = self.ignore_taken_socks(camera.socks_centers)
        closest_sock = np.argmin(
            np.sqrt(
                np.sum((new_socks_centers - np.array([start_point[0], start_point[1]]))**2, axis=1)
            )
        )

        if self.next_sock is not None:
            if self.next_sock == closest_sock:
                other_sock = self.get_other_sock(closest_sock, len(new_socks_centers))
                print(other_sock)
                other_cx, other_cy = camera.socks_centers[other_sock]
                return int(round(other_cx)), int(round(other_cy))
        print(closest_sock)
        self.next_sock = closest_sock
        closest_cx, closest_cy = camera.socks_centers[closest_sock]
        return int(round(closest_cx)), int(round(closest_cy))

    def mark_sock_as_taken(self):
        self.taken_socks.append(self.next_sock)

    def ignore_taken_socks(self, socks_centers):
        new_socks_centers = socks_centers.copy()
        for taken in self.taken_socks:
            new_socks_centers[taken][0] = np.inf
            new_socks_centers[taken][1] = np.inf
        return new_socks_centers

    def get_other_sock(self, current_sock, num_socks):
        free_socks = []
        for sock in range(num_socks):
            if sock != current_sock and sock not in self.taken_socks:
                free_socks.append(sock)
        if len(free_socks) > 0:
            return free_socks[np.random.randint(0, len(free_socks))]
        else:
            return current_sock
