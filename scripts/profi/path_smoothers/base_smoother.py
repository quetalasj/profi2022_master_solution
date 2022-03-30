from abc import abstractmethod
from profi.path_planners.base_planner import BasePlaner


class BaseSmoother(BasePlaner):
    def __init__(self, planner):
        BasePlaner.__init__(self)
        self.planner = planner

    @abstractmethod
    def get_path(self, start_point, goal_point, c_space):
        pass
