from abc import abstractmethod


class BaseDecisionMaker:
    def __init__(self):
        pass

    @abstractmethod
    def get_goal(self, start_point, camera):
        pass
