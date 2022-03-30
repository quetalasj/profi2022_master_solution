from abc import abstractmethod


class BasePlaner:
    def __init__(self):
        pass

    @abstractmethod
    def get_path(self, start_point, goal_point, c_space):
        pass