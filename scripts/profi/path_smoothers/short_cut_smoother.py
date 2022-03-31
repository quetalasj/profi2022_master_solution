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
        if old_path is None:
            return None
        try_to_cut = True
        j = 0
        while j < self.max_steps and try_to_cut:
            j += 1
            print("Smoothing step", j)
            try_to_cut = False
            num_vertices = len(old_path)
            new_path = [old_path[0]]
            i = 0
            while i < num_vertices - 2:
                point_0 = old_path[i]
                point_1 = old_path[i + 1]
                point_2 = old_path[i + 2]
                connection = self.smoother_connect(point_0, point_2, c_space)
                point_x = check_connectivity(np.round(connection).astype(int), c_space)
                if point_x is None:
                    i += 1
                    continue
                dist = np.sum((point_x - point_2)**2)
                if dist < 2:
                    new_path.append(point_2)
                    i += 2
                    try_to_cut = True
                else:
                    new_path.append(point_1)
                    i += 1
            if np.any(new_path[-1] != old_path[-1]):
                new_path.append(old_path[-1])

            old_path = new_path

        return np.array(new_path)

    def smoother_connect(self, first_state, second_state, environment):
        # num = 100
        num = max(environment.shape[1], environment.shape[0])
        s_x = np.linspace(first_state[0], second_state[0], num=num)
        s_y = np.linspace(first_state[1], second_state[1], num=num)
        steer_states = np.stack((s_x, s_y), axis=1)
        return steer_states[1:]
