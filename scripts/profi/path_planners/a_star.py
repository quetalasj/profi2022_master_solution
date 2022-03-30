import numpy as np
from Queue import PriorityQueue
from base_planner import BasePlaner


class Position():
    def __init__(self, x, y, th, cost=None):
        self.x = x
        self.y = y
        self.th = th
        self.cost = cost

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y and self.th == other.th

    def get_position(self):
        return self.x, self.y, self.th

    def __hash__(self):
        return hash((self.x, self.y, self.th))

    def __ne__(self, other):
        return not self.__eq__(other)


def get_near_orientations(th, max_orientations):
    if th == max_orientations - 1:
        left_orientation = 0
    else:
        left_orientation = th + 1

    if th == 0:
        right_orientation = max_orientations - 1
    else:
        right_orientation = th - 1

    return left_orientation, right_orientation


def get_actions(position, c_space):
    actions = []

    # left_orientation, right_orientation = get_near_orientations(position.th, c_space.shape[2])

    # if c_space[position.y, position.x, right_orientation] < 1:
    #     actions.append((position.x, position.y, right_orientation))
    #
    # if c_space[position.y, position.x, left_orientation] < 1:
    #     actions.append((position.x, position.y, left_orientation))

    if position.x < c_space.shape[1] - 1 and c_space[position.y, position.x + 1, position.th] < 1:
        actions.append((position.x + 1, position.y, position.th))

    if position.x > 0 and c_space[position.y, position.x - 1, position.th] < 1:
        actions.append((position.x - 1, position.y, position.th))

    if position.y < c_space.shape[0] - 1 and c_space[position.y + 1, position.x, position.th] < 1:
        actions.append((position.x, position.y + 1, position.th))

    if position.y > 0 and c_space[position.y - 1, position.x, position.th] < 1:
        actions.append((position.x, position.y - 1, position.th))

    return actions


def apply_action(action):
    return Position(*action)


class AStar(BasePlaner):
    def __init__(self, distance_function=lambda pos1, pos2: 0, l_cost=lambda x, u: 1):
        BasePlaner.__init__(self)
        self.distance_function = distance_function
        self.l_cost = l_cost

    def compute_plan(self, start_point, goal_point, c_space, distance_function, l_cost=lambda x, u: 1):
        plan = {}
        goal_position = Position(*goal_point, cost=np.inf)
        position = Position(*start_point, cost=0)
        visited = {position: position} # add to visited

        # priority_queue = {position: 0} # add to the queue
        priority_queue = PriorityQueue()
        priority_queue.put_nowait((0, position))
        # while len(priority_queue) > 0:
        while not priority_queue.empty():
            # position = sorted(priority_queue, key=priority_queue.get)[0] # sort by priority and take the first with the highest one
            # priority_queue.pop(position)
            _, position = priority_queue.get_nowait()

            if position == goal_position:
                return 0, plan

            for action in get_actions(position, c_space):
                new_position = apply_action(action)
                if new_position == goal_position:
                    new_position = goal_position # to store goal position cost
                if new_position not in visited.keys():
                    visited[new_position] = new_position
                    plan[new_position] = position
                    new_position.cost = position.cost + l_cost(position, action)
                    # priority_queue[new_position] = new_position.cost + distance_function(new_position, goal_position)
                    priority_queue.put_nowait((new_position.cost + distance_function(new_position, goal_position),
                                               new_position))
                else:
                    new_position = visited[new_position]
                    new_cost = position.cost + l_cost(position, action)
                    if new_cost < min(new_position.cost, goal_position.cost):
                        new_position.cost = new_cost
                        plan[new_position] = position
        return 1

    def get_path_from_plan(self, plan, start_point, goal_point):
        goal_position = Position(*goal_point)
        start_position = Position(*start_point)
        path = [goal_point]
        position = plan[goal_position]
        path.append(position.get_position())
        while position != start_position:
            position = plan[position]
            path.append(position.get_position())
        path = path[::-1]
        return path

    def get_path(self, start_point, goal_point, c_space):
        path = None
        # distance_function = lambda pos1, pos2: 0
        print("Start to plan")
        distance_function = lambda pos1, pos2: abs(pos1.x - pos2.x) + abs(pos1.y - pos2.y)  # L1 norm
        status, plan = self.compute_plan(start_point, goal_point, c_space, distance_function, l_cost=lambda x, u: 1)

        if status == 0:
            path = np.array(self.get_path_from_plan(plan, start_point, goal_point))
        print("Finish to plan")
        return path

