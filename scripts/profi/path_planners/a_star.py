import numpy as np
from Queue import PriorityQueue
from base_planner import BasePlaner
import heapq


class Position():
    def __init__(self, x, y, cost=None):
        self.x = x
        self.y = y
        self.th = 0
        self.cost = cost

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def get_position(self):
        return self.x, self.y

    def __hash__(self):
        return hash((self.x, self.y))

    def __ne__(self, other):
        return not self.__eq__(other)


class Node:
    def __init__(self, cost, position):
        self.cost = cost
        self.position = position

    def __lt__(self, other):
        return self.cost < other.cost

    def __gt__(self, other):
        return self.cost > other.cost


def get_actions(position, c_space):
    actions = []

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

        priority_queue = []
        heapq.heapify(priority_queue)
        heapq.heappush(priority_queue, Node(0, position))

        it_number = 0
        while len(priority_queue) > 0:
            it_number += 1
            node = heapq.heappop(priority_queue)
            position = node.position
            if position == goal_position or it_number > 5000:
                return 0, plan

            for action in get_actions(position, c_space):
                new_position = apply_action(action)
                if new_position == goal_position:
                    new_position = goal_position # to store goal position cost
                if new_position not in visited.keys():
                    visited[new_position] = new_position
                    plan[new_position] = position
                    new_position.cost = position.cost + l_cost(position, action)
                    heapq.heappush(priority_queue, Node(new_position.cost + distance_function(new_position, goal_position),
                                                        new_position))
                else:
                    new_position = visited[new_position]
                    new_cost = position.cost + l_cost(position, action)
                    if new_cost < min(new_position.cost, goal_position.cost):
                        new_position.cost = new_cost
                        plan[new_position] = position
        return 1

    def get_path_from_plan(self, plan, start_point, goal_point, distance_f):
        goal_position = Position(*goal_point)
        start_position = Position(*start_point)
        position = self.get_closeset(plan, goal_position, distance_f)
        path = [position.get_position()]
        # position = plan[goal_position]
        # path.append(position.get_position())
        while position != start_position:
            position = plan[position]
            path.append(position.get_position())
        path.append(start_position.get_position())
        path = path[::-1]
        return path

    def get_closeset(self, plan, goal, distance_f):
        closest = plan.keys()[-1]
        min_distance = distance_f(goal, closest)
        for pose in plan.keys():
            distance = distance_f(goal, pose)
            if distance < min_distance:
                min_distance = distance
                closest = pose
        return closest


    def get_path(self, start_point, goal_point, c_space):
        path = None
        # distance_function = lambda pos1, pos2: 0
        print("Start to plan")
        # distance_function = lambda pos1, pos2: abs(pos1.x - pos2.x) + abs(pos1.y - pos2.y)  # L1 norm
        distance_function = lambda pos1, pos2: np.sqrt((pos1.x - pos2.x)**2 + (pos1.y - pos2.y)**2)  # L2 norm
        status, plan = self.compute_plan(start_point, goal_point, c_space, distance_function, l_cost=lambda x, u: 1)

        if status == 0:
            path = np.array(self.get_path_from_plan(plan, start_point, goal_point, distance_function))
        print(path)
        print("Finish to plan")
        return path

