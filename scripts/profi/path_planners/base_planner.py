from abc import abstractmethod


class BasePlaner:
    def __init__(self):
        self.full_path = None

    @abstractmethod
    def get_path(self, start_point, goal_point, c_space):
        pass