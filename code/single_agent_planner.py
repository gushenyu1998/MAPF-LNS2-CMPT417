import heapq

# root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'timestep': 0}

Map_size = 0


# (Task 1.1) one move towards of waiting (Not move)
def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst


def compute_heuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(4):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
                    or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
                continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values


def build_constraint_table(constraints, agent, constraint_loc=(0, 0), constraint_time=-1):
    ##############################
    # Task 1.2/1.3: Return a table that constains the list of constraints of
    #               the given agent for each time step. The table can be used
    #               for a more efficient constraint violation check in the 
    #               is_constrained function.
    if constraints is None:
        constraints = []
    example_constrain = {'agent': agent, 'loc': constraint_loc, 'timestep': constraint_time, 'positive': False}
    constraints.append(example_constrain)
    return constraints  # (New added 1.2) create a constraint and return the constraint table
    pass


def get_location(path, time):
    if time <= 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path


def is_constrained(curr_loc, next_loc, next_time, constraint_table, agent):
    ##############################
    # Task 1.2/1.3: Check if a move from curr_loc to next_loc at time step next_time violates
    #               any given constraint. For efficiency the constraints are indexed in a constraint_table
    #               by time step, see build_constraint_table.
    for constraint in constraint_table:
        if constraint['positive']:  # This function only resolve the negative constraints
            continue
        if len(constraint['loc']) == 1:  # (Task 1.2) Resolve the vertex constraints
            if constraint['loc'][0] == next_loc and \
                    (constraint['timestep'] == next_time or constraint['timestep'] == -10) and \
                    constraint['agent'] == agent:
                return True  # (Task 1.2) If the node conflict to negative constraint, not expand this node
        elif len(constraint['loc']) == 2:  # (Task 1.3) resolve the negative edge constraint
            if constraint['loc'] == [curr_loc, next_loc] and \
                    constraint['timestep'] == next_time and \
                    constraint['agent'] == agent:
                return True
    return False  # (Task 1.2/1.3/1.4) False means ok to go


# Task 4.1 positive constraints supportable
def is_constrained_positive(parent_loc, loc, agent, constraints, timestep):
    for constraint in constraints:
        if constraint['positive']:  # only process positive constraints
            if constraint['agent'] == agent and constraint['timestep'] == timestep:
                if len(constraint['loc']) == 1:  # process vertex constraints
                    if constraint['loc'] != loc:
                        return True
                    else:
                        return False
                elif len(constraint['loc']) == 2:  # process edge constraints
                    if [parent_loc, loc] != constraint['loc']:
                        return False
                    else:
                        return True

    return False


def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))
    pass


def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


# Task 1.4 agent should wait until largest constraint time comes
def goal_test(goal_loc, constraints, curr):
    constraints_time = [0]  # default value 0
    if len(constraints) != 0:
        for i in constraints:  # add all the constraint time into a list
            constraints_time.append(i['timestep'])
    # compare current time and largest constraint time
    if curr['timestep'] < max(constraints_time):
        return False  # False means agent should wait for a little while
    elif curr['loc'] == goal_loc:
        return True  # Ture means the solution is ok to go
    else:
        return False


def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
    """
    global Map_size
    Map_size = len(my_map) * len(my_map[0])

    ##############################
    # Task 1.1: Extend the A* search to search in the space-time domain
    #           rather than space domain, only.

    open_list = []
    closed_list = dict()
    earliest_goal_timestep = 0
    h_value = h_values[start_loc]
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'timestep': 0}
    # (Task 1.1) Add time stamp of root, value of time stamp is 0

    push_node(open_list, root)

    root_tuple = (root['loc'], root['timestep'])
    # (Task 1.1) create a tuple to store cell and time stamp
    closed_list[root_tuple] = root  # (New added)Use the tuple as the index of dictionary
    while len(open_list) > 0:
        curr = pop_node(open_list)
        #############################
        # Task 1.4: Adjust the goal test condition to handle goal constraints
        if goal_test(goal_loc, constraints, curr):
            return get_path(curr)
        #  (Task 4.2), the loop control would test the goal and time constraint. Path would get out of loop
        child_not_meet_positive_constraint = 0
        for dir in range(5):  # (New added) Add one more range (wait)
            child_loc = move(curr['loc'], dir)
            if child_loc[0] < 0 or child_loc[1] < 0 or child_loc[0] >= len(my_map) or child_loc[1] >= len(my_map[0]):
                continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            # (Task 4.1) let the planner prune all the node not meet positive constraint at particular timestep
            if is_constrained_positive(curr['loc'], child_loc, agent, constraints, curr['timestep'] + 1):
                continue
            # (Task 1.2) If the child loc and time meet the negative constraint, then pruned the child (skip this loop)
            if is_constrained(curr['loc'], child_loc, curr['timestep'] + 1, constraints, agent):
                continue

            child = {'loc': child_loc,
                     'g_val': curr['g_val'] + 1,
                     'h_val': h_values[child_loc],
                     'parent': curr,
                     'timestep': curr['timestep'] + 1  # (Task 1.1)Add time stamp when new child node created
                     }

            # print('A child node is: ', child['loc'], child['timestep'], ' parent node is: ',
            #       curr['loc'],
            #       curr['timestep'])
            # (New added) use child tuple (loc and time) in the close list
            #  (Task 2.4) adding a searching cap to detect failure solution
            if curr['timestep'] > Map_size ^ 4:
                return None
            # (Task 1.1) Use child tuple to store in the open and close list
            child_tuple = (child['loc'], child['timestep'])
            if child_tuple in closed_list:
                existing_node = closed_list[child_tuple]
                if compare_nodes(child, existing_node):
                    closed_list[child_tuple] = child
                    push_node(open_list, child)
            else:
                closed_list[child_tuple] = child
                push_node(open_list, child)


    return None  # Failed to find solutions
