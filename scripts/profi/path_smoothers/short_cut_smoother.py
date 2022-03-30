import numpy as np

from base_smoother import BaseSmoother
from profi.path_planners.rrt import check_connectivity


class ShortCutSmoother(BaseSmoother):
    def __init__(self, planner):
        BaseSmoother.__init__(self, planner)
        self.max_steps = 15

    def get_path(self, start_point, goal_point, c_space):
        # path includes current pose
        old_path = self.planner.get_path(start_point, goal_point, c_space)
        x = old_path
        try_to_cut = True
        j = 0
        while try_to_cut:
            j += 1
            print("Smoothing step", j)
            if j > self.max_steps:
                break
            try_to_cut = False
            num_vertices = len(old_path)
            new_path = [old_path[0]]
            i = 0
            while i < num_vertices - 2:
                point_0 = old_path[i]
                point_1 = old_path[i + 1]
                point_2 = old_path[i + 2]
                connection = self.planner.steer(point_0, point_2)
                point_x = check_connectivity(connection, c_space)
                if np.all(point_x == point_2):
                    new_path.append(point_2)
                    i += 2
                    try_to_cut = True
                else:
                    new_path.append(point_1)
                    i += 1
            if np.all(new_path[-1] != old_path[-1]):
                new_path.append(old_path[-1])

            old_path = new_path

        # print("Full path", x)
        # print("Smoothed path", new_path)

        return np.array(new_path)
