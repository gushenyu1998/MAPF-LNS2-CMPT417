import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost
import itertools


def detect_collision(path1, path2):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.
    path_1 = path1['path']
    path_2 = path2['path']
    max_size = max(len(path_1), len(path_2))
    for i in range(max_size):
        # Task 3.1 assign vertex collision
        if get_location(path_1, i) == get_location(path_2, i):
            collision = {'a1': path1['agent'],
                         'a2': path2['agent'],
                         'loc': [get_location(path1['path'], i)],
                         'timestep': i
                         }
            return collision
        # Task 3.1 assign edge collisions
        elif (get_location(path_1, i), get_location(path_1, i + 1)) == \
                (get_location(path_2, i + 1), get_location(path_2, i)):
            collision = {'a1': path1['agent'],
                         'a2': path2['agent'],
                         'loc': [get_location(path1['path'], i), get_location(path1['path'], i + 1)],
                         'timestep': i + 1
                         }
            return collision
    return None
    pass


def detect_collisions(paths):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.
    collisions = []
    path_agent_tuples = []
    for i in range(len(paths)):  # Task 3.1 get the map between path and agents
        path_agent_tuples.append({'agent': i, 'path': paths[i]})
    # Task 3.1 get the combination of 2 agents' path among path sets
    path_agent_combination = list(itertools.combinations(path_agent_tuples, 2))
    # Task 3.1 for each combination of 2 agents, detect collisions between them
    for j in range(len(path_agent_combination)):
        temp = detect_collision(path_agent_combination[j][0], path_agent_combination[j][1])
        if temp is not None:
            collisions.append(temp)

    return collisions
    pass


def standard_splitting(collision):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep
    constraint = []
    if collision is None:
        return constraint
    if len(collision['loc']) == 1:  # Assign vertex constraint
        temp_a1 = {'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep'],
                   'positive': False}
        temp_a2 = {'agent': collision['a2'], 'loc': collision['loc'], 'timestep': collision['timestep'],
                   'positive': False}
        constraint.append(temp_a1)
        constraint.append(temp_a2)
    elif len(collision['loc']) == 2:  # Assign edge constraints
        loc_copy = list.copy(collision['loc'])
        temp_a3 = {'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep'],
                   'positive': False}
        list.reverse(loc_copy)  # reverse the swap towards for a2 in the collision
        temp_a4 = {'agent': collision['a2'], 'loc': loc_copy, 'timestep': collision['timestep'], 'positive': False}
        constraint.append(temp_a3)
        constraint.append(temp_a4)
    else:
        raise BaseException("Standard Splitting error")

    return constraint


def disjoint_splitting(collision):
    ##############################
    # Task 4.1: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly
    constraint = []
    if collision is None:
        return constraint
    rand_flag = random.randint(0, 1)
    if rand_flag == 0: # randomly choose the agent do the positive constraints
        positive_agent = 'a1'
    else:
        positive_agent = 'a2'
    if len(collision['loc']) == 1: # process positive vertex constraints
        temp_a1 = {'agent': collision[positive_agent], 'loc': collision['loc'], 'timestep': collision['timestep'],
                   'positive': True}
        temp_a2 = {'agent': collision[positive_agent], 'loc': collision['loc'], 'timestep': collision['timestep'],
                   'positive': False}
        constraint.append(temp_a1)
        constraint.append(temp_a2)
    elif len(collision['loc']) == 2: # process positive edge constraints
        loc_copy = list.copy(collision['loc'])
        if positive_agent == 'a1': # if the a1 was chosen, no need to reverse the location
            temp_a3 = {'agent': collision[positive_agent], 'loc': collision['loc'], 'timestep': collision['timestep'],
                       'positive': True}
            temp_a4 = {'agent': collision[positive_agent], 'loc': collision['loc'], 'timestep': collision['timestep'],
                       'positive': False}
            constraint.append(temp_a3)
            constraint.append(temp_a4)
        elif positive_agent == 'a2': # if a2 was chosen, reverse the location and fill in
            list.reverse(loc_copy)
            temp_a3 = {'agent': collision[positive_agent], 'loc': loc_copy, 'timestep': collision['timestep'],
                       'positive': True}
            temp_a4 = {'agent': collision[positive_agent], 'loc': loc_copy, 'timestep': collision['timestep'],
                       'positive': False}
            constraint.append(temp_a3)
            constraint.append(temp_a4)

    return constraint
    pass


def paths_violate_constraint(constraint, paths):
    assert constraint['positive'] is True
    rst = []
    for i in range(len(paths)):
        if i == constraint['agent']:
            continue
        curr = get_location(paths[i], constraint['timestep'])
        prev = get_location(paths[i], constraint['timestep'] - 1)
        if len(constraint['loc']) == 1:  # vertex constraint
            if constraint['loc'][0] == curr:
                rst.append(i)
        else:  # edge constraint
            if constraint['loc'][0] == prev or constraint['loc'][1] == curr \
                    or constraint['loc'] == [curr, prev]:
                rst.append(i)
    return rst


class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        # print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        # print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint=True, child=None):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        self.start_time = timer.time()
        print(disjoint)

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)

        # Task 3.1: Testing
        print(root['collisions'])
        pass
        # Task 3.2: Testing
        for collision in root['collisions']:
            print(standard_splitting(collision))
        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit
        while len(self.open_list) > 0 and not disjoint:  # Line 6: loop is open list is not empty
            curr = self.pop_node()  # Line 7: pop the smallest node
            if len(curr['collisions']) == 0:  # Line 8/9: return node if no collisions
                self.print_results(curr)
                return curr['paths']
            cbs_collision = curr['collisions'][0]  # Line 10: pick one collision
            cbs_constraint = standard_splitting(cbs_collision)  # Line 11: standard spilt the collision
            for constraint in cbs_constraint:  # Line 12: Expand each constraint by loop
                break_Flag = False
                # Line 14: Union the new constraint and current constraints
                for cons in curr['constraints']:
                    # If the constraint has already been the constraint list, not expand this node
                    if constraint == cons:
                        break_Flag = True
                if break_Flag:
                    continue
                a = list.copy(curr['constraints'])
                a.append(constraint)  # Append new constraint to the constraint list
                # Line 13 create a new child
                child = {'cost': 0,
                         'constraints': a,
                         'paths': curr['paths'],  # Line 15, assign old path to child
                         'collisions': []
                         }
                a_i = constraint['agent']
                temp = list.copy(curr['paths'])
                cbs_path = a_star(self.my_map, self.starts[a_i], self.goals[a_i], self.heuristics[a_i], a_i,
                                  child['constraints']) # Line 17 run a_star for constraint agent i
                if cbs_path is not None:
                    # Line 19-22 if paths are not empty assign properties to child and push node
                    temp[a_i] = cbs_path
                    child['paths'] = temp
                    child['collisions'] = detect_collisions(child['paths'])
                    child['cost'] = get_sum_of_cost(child['paths'])
                    self.push_node(child)

        ##############################
        # Task 4.3: Modified High Level search
        while len(self.open_list) > 0 and disjoint:  # Line 6: loop is open list is not empty
            curr = self.pop_node()  # Line 7: pop the smallest node
            if len(curr['collisions']) == 0:  # Line 8/9: return node if no collisions
                self.print_results(curr)
                return curr['paths']
            cbs_collision = curr['collisions'][0]  # Line 10: pick one collision
            cbs_constraint = disjoint_splitting(cbs_collision)  # Line 11: standard spilt the collision
            for constraint in cbs_constraint:  # Line 12: Expand each constraint by loop
                break_Flag = False
                path_violate = []
                # Line 14: Union the new constraint and current constraints
                for cons in curr['constraints']:
                    # If the constraint has already been the constraint list, not expand this node
                    if constraint == cons:
                        break_Flag = True
                if break_Flag:
                    continue
                a = list.copy(curr['constraints'])
                a.append(constraint)
                # (Task 4.3) Append new constraint to the constraint list by using path violate function
                if constraint['positive']:
                    path_violate = paths_violate_constraint(constraint, curr['paths'])
                    for other_agent in path_violate:
                        new_constraint = {'agent': other_agent,
                                          'loc': constraint['loc'],
                                          'timestep': constraint['timestep'],
                                          'positive': False}
                        a.append(new_constraint)
                # Line 13 create a new child
                child = {'cost': 0,
                         'constraints': a,
                         'paths': curr['paths'],  # Line 15, assign old path to child
                         'collisions': []
                         }
                # (Task 4.3) collect the agent need to re-plan (path violate and the positive constraint path)
                path_violate.append(constraint['agent'])
                temp = list.copy(curr['paths'])
                for a_i in path_violate:
                    cbs_path = a_star(self.my_map, self.starts[a_i], self.goals[a_i], self.heuristics[a_i], a_i,
                                      child['constraints'])  # Line 17 run a_star for all constraint agent i
                    if cbs_path is not None:
                        temp[a_i] = cbs_path
                        child['paths'] = temp
                        child['collisions'] = detect_collisions(child['paths'])
                        child['cost'] = get_sum_of_cost(child['paths'])
                    else:
                        # if not solution for 1 agent then pruned the node
                        break_Flag = True
                        break
                if break_Flag:
                    continue
                self.push_node(child)

        print("no solution there")
        self.print_results(root)
        return None

    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
