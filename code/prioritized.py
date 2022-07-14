import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost, build_constraint_table


class PrioritizedPlanningSolver(object):
    """A planner that plans for each robot sequentially."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.CPU_time = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        result = []
        constraints = []

        priority_queue = range(self.num_of_agents)

        for i in range(self.num_of_agents):  # Find path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, constraints)
            if path is None:
                raise BaseException('No solutions')

            path_tuple = {'agent': i, 'path': path}
            result.append(path)

            ##############################
            # Task 2: Add constraints here
            #         Useful 6variables:
            #            * path contains the solution path of the current (i'th) agent, e.g., [(1,1),(1,2),(1,3)]
            #            * self.num_of_agents has the number of total agents
            #            * constraints: array of constraints to consider for future A* searches

            ##############################
            if i >= priority_queue[-1]:
                break

            time = 0
            constraint_list = list(set(priority_queue) - {i})
            # (Task 2.1) loop for Add vertex constraints
            for constraint in path:
                for constrained_agent in constraint_list:
                    build_constraint_table(constraints, constrained_agent, [constraint], time)
                    time += 1

            # (Task 2.2) loop for add edge constraints
            time = 1
            vertex_constraint_len = len(constraints)
            for swapI in range(vertex_constraint_len - 1):
                for constrained_agent in constraint_list:
                    build_constraint_table(constraints, constrained_agent, [path[swapI + 1], path[swapI]], time)
                    time += 1
            #
            # pass
            # (Task 2.3) add constrain once higher priority get goal, -10 is used for express future time
            for constrained_agent in constraint_list:
                build_constraint_table(constraints, constrained_agent, [path[-1]], -10)
            priority_queue = list(set(priority_queue) - {i})

        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print(result)
        return result
