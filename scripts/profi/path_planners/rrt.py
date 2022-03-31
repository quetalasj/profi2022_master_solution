import numpy as np
from base_planner import BasePlaner

def L1_distance(q1, q2):
    return abs(q1[0] - q2[0]) + abs(q1[1] - q2[1])

def L2_distance(q1, q2):
    return np.sqrt((q1[0] - q2[0])**2 + (q1[1] - q2[1])**2)

def check_connectivity(discrete_trajectory, environment):
    last_available_state = None
    for state in discrete_trajectory:
        # if there is collision
        if environment[state[1], state[0]] > 0:
            return last_available_state
        last_available_state = state
    return last_available_state


class Tree:
    def __init__(self, root_state):
        self.backward_graph = {root_state: None}

    def add_edge(self, parent, child):
        if child not in self.backward_graph.keys():
            self.backward_graph[child] = parent

    def find_nearest_to(self, random_state, distance_function):
        nearest_state = None
        min_distance = np.inf
        for state in self.backward_graph.keys():
            diff = distance_function(state, random_state)
            if min_distance > diff:
                min_distance = diff
                nearest_state = state
        return nearest_state


class RRT(BasePlaner):

    def __init__(self):
        BasePlaner.__init__(self)
        self.environment = None
        self.distance_function = None
        self.success_radius = 10

    def create_tree(self, start_state):
        return Tree(tuple(start_state))

    def explore(self, start_state, goal_state, N=1000, max_iter_num=15):
        G = self.create_tree(start_state)
        it = 0
        plan = None
        while plan is None:
            it += 1
            print(it)
            if it > max_iter_num:
                return (None, None, it)  # Didn't find the solution for the desired time, please try again
            for i in range(N):
                q_rand = self.sample(np.random.rand(), goal_state)
                q_near = G.find_nearest_to(q_rand, self.distance_function)
                discrete_trajectory = self.steer(np.array(q_near), q_rand)
                q_new = check_connectivity(discrete_trajectory, self.environment)
                if q_new is None:
                    continue
                G.add_edge(q_near, tuple(q_new))

            plan = self.find_path_to(goal_state, G)
        return (G, plan, it)

    def sample(self, chance, goal_state):
        if chance < 0.7:
            y = np.random.randint(0, self.environment.shape[0])
            x = np.random.randint(0, self.environment.shape[1])
            return x, y
        else:
            return goal_state

    def steer(self, first_state, second_state):
        num = max(self.environment.shape[1], self.environment.shape[0]) # TODO: Is this the reason of slow work?
        s_x = np.round(np.linspace(first_state[0], second_state[0], num=num)).astype(int)
        s_y = np.round(np.linspace(first_state[1], second_state[1], num=num)).astype(int)
        steer_states = np.stack((s_x, s_y), axis=1)
        return steer_states[1:]

    def find_path_to(self, goal_state, tree):
        nearest_state = tree.find_nearest_to(goal_state, self.distance_function)
        trajectory = self.steer(np.array(nearest_state), goal_state)
        steer_state = check_connectivity(trajectory, self.environment)
        if steer_state is None:
            return None
        if self.distance_function(steer_state, goal_state) < self.success_radius:
            print("Plan is Found")
            if np.all(np.array(nearest_state) == np.array(goal_state)):
                print("Exect  solution")
                plan = [nearest_state]
            else:
                print("near to goal solution")
                plan = [goal_state, nearest_state]
            current_state = nearest_state
            while not tree.backward_graph[current_state] is None:
                current_state = tree.backward_graph[current_state]
                plan.append(current_state)
            return plan[::-1]
        else:
            # print("Plan was not found")
            return None

    def get_path(self, start_point, goal_point, c_space):
        self.environment = c_space
        self.distance_function = L2_distance
        start_point = (start_point[0], start_point[1])
        goal_point = (goal_point[0], goal_point[1])

        print("start planning")
        G, plan, it = self.explore(start_point, goal_point)

        if self.environment[plan[-1][1], plan[-1][0]] > 0:
            plan = plan[:-1]

        if plan is not None:
            return np.array(plan)
        return None


class RRTMaxSteer(RRT):
    def __init__(self, max_steer):
        RRT.__init__(self)
        self.max_steer = max_steer

    def steer(self, first_state, second_state):
        # num = max(self.environment.shape[1], self.environment.shape[0])
        num = 10
        s_x = np.linspace(first_state[0], second_state[0], num=num)
        s_y = np.linspace(first_state[1], second_state[1], num=num)
        ds_x = s_x - s_x[0]
        ds_y = s_y - s_y[0]
        distances = np.sqrt(np.sum((ds_x**2, ds_y**2), axis=0))
        s_x = s_x[distances < self.max_steer]
        s_y = s_y[distances < self.max_steer]

        steer_states = np.stack((s_x, s_y), axis=1)
        # print(steer_states)
        return np.round(steer_states[1:]).astype(int)
