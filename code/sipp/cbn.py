import argparse
import itertools
import sys
import time

import yaml
import random
from math import fabs
from sipp import SippPlanner

N = 5
As_list = []
constraint_list = []
movement = ['left', 'right', 'top', 'down', 'wait']


def agent_random_walk(path_list_tuple, collision_list, map):
    global constraint_list
    global movement
    global As_list
    random_agent = random.choice(As_list)
    As_list.remove(random_agent)
    # print("random choiced agent is:", random_agent)
    t = (map['map']['dimensions'][0] * map['map']['dimensions'][1]) ^ 4
    # if As_list == []:
    #     print("As_list is go null")
    # find the most late random work start time
    for collision in collision_list:
        if (collision['a1'] == random_agent['agent'] or collision['a2'] == random_agent['agent']) and \
                collision['loc'][0]['t'] <= t:
            t = collision['loc'][0]['t']

    # random select start time on its path
    randon_start_time = random.randint(1, t - 1)

    # delete the movement after the random start point
    del random_agent['path'][randon_start_time: len(random_agent['path'])]
    move = ''
    walk_time = randon_start_time-1
    agent = random_agent['agent']
    while True:
        if len(movement) == 0:
            print("random work fail")
            return None
        move = random.choice(movement)
        temp_prev = random_agent['path'][min(walk_time,0)]
        temp = None
        if move == 'left':
            temp = {'x': temp_prev['x'], 'y': temp_prev['y'] + 1, 't': walk_time + 1}
            # test of the agent get to goal
            if goal_test(random_agent['agent'], temp, map):
                print("goal find, return the path")
                random_agent['path'].append(temp)
                return random_agent
            # test of the agent move again the map
            if check_random_movement_constraint(agent,temp, temp_prev, map):
                random_agent['path'].append(temp)
            else:
                movement.remove('left')
                continue

        elif move == 'right':
            temp = {'x': temp_prev['x'], 'y': temp_prev['y'] - 1, 't': walk_time + 1}
            # test of the agent get to goal
            if goal_test(random_agent['agent'], temp, map):
                print("goal find, return the path")
                random_agent['path'].append(temp)
                return random_agent
            # test of the agent move again the map
            if check_random_movement_constraint(agent, temp, temp_prev, map):
                random_agent['path'].append(temp)
            else:
                movement.remove('right')
                continue

        elif move == 'top':
            temp = {'x': temp_prev['x'] - 1, 'y': temp_prev['y'], 't': walk_time + 1}
            # test of the agent get to goal
            if goal_test(random_agent['agent'], temp, map):
                print("goal find, return the path")
                random_agent['path'].append(temp)
                return random_agent
            # test of the agent move again the map
            if check_random_movement_constraint(agent, temp, temp_prev, map):
                random_agent['path'].append(temp)
            else:
                movement.remove('top')
                continue

        elif move == 'down':
            temp = {'x': temp_prev['x'] + 1, 'y': temp_prev['y'], 't': walk_time + 1}
            # test of the agent get to goal
            if goal_test(random_agent['agent'], temp, map):
                print("goal find, return the path")
                random_agent['path'].append(temp)
                return random_agent
            # test of the agent move again the map
            if check_random_movement_constraint(agent, temp, temp_prev, map):
                random_agent['path'].append(temp)
            else:
                movement.remove('down')
                continue
        elif move == 'wait':
            temp = {'x': temp_prev['x'], 'y': temp_prev['y'], 't': walk_time + 1}
            if walk_time+1 > (map['map']['dimensions'][0] * map['map']['dimensions'][1]) ^ 4:
                print('search fail, return None')
                return None
            random_agent['path'].append(temp)
            walk_time+=1
            continue

        if temp is None:
            print("unknown error happens")
            exit(0)

        for agent in path_list_tuple:
            loc = get_location(agent['path'], walk_time + 1)
            loc_prev = get_location(agent['path'], walk_time)
            if random_walk_detect_collision(temp, loc, temp_prev, loc_prev) == "V collision":
                print("random walk detect vector collision")
                random_agent['path'].append(temp)
                temp_vertice = {'agent':random_agent['agent'], 'loc': [temp]}
                constraint_list.append(temp_vertice)
                As_list.append(random_agent)
                return
            elif random_walk_detect_collision(temp, loc, temp_prev, loc_prev) == "E collision":
                print("random walk detect edge collision")
                temp_edge= {'agent':random_agent['agent'], 'loc': [temp_prev,temp]}
                constraint_list.append(temp_edge)
                As_list.append(random_agent)
        walk_time += 1

        movement = ['left', 'right', 'top', 'down', 'wait']


def goal_test(agent, temp, map):
    goal_list = map['agents']
    goal = list(filter(lambda item: item['name'] == agent, goal_list))[0]
    if temp['x'] == goal['goal'][0] and temp['y'] == goal['goal'][1]:
        return True
    return False


def check_random_movement_constraint(agent,temp, temp_prev, map):
    # check temp not hit the wall nor not go out of map
    if temp['x'] < 0 or temp['x'] > map['map']['dimensions'][0] or \
            temp['y'] < 0 or temp['y'] > map['map']['dimensions'][1]:
        return False
    # check temp not hit the hard obstacle
    try:
        a = map['map']['obstacles'].index((temp['x'], temp['y']))
        return False
    except Exception as e:
        if not type(e) == ValueError:
            print("Exception not Value error happens, quit program")
            sys.exit(0)
        else:
            pass
    # check temp not hit the constraint
    for constraint in constraint_list:
        if agent == constraint['agent']:
            if len(constraint['loc']) == 1:
                if temp['x'] == constraint['loc'][0]['x'] and temp['y'] == constraint['loc'][0]['y'] and temp['t'] == constraint['loc'][0]['t']:
                    print(constraint)
                    return False
            elif len(constraint['loc']) == 2:
                if temp_prev['x'] == constraint['loc'][0]['x'] and temp_prev['y'] == constraint['loc'][0]['y'] and temp_prev['t'] == constraint['loc'][0]['t'] and \
                        temp['x'] == constraint['loc'][1]['x'] and temp['y'] == constraint['loc'][1]['y'] and temp['t'] == constraint['loc'][1]['t']:
                    return False
    return True


def random_walk_detect_collision(temp, loc, temp_prev, loc_prev):
    # vertex collision detect
    if temp['x'] == loc['x'] and temp['y'] == loc['y']:
        return "V collision"
    if temp['x'] == loc_prev['x'] and temp['y'] == loc_prev['y'] and \
            temp_prev['x'] == loc['x'] and temp_prev['y'] == loc['y']:
        return "E collision"
    return "No collision"


def get_As_split(As, tuple_list):
    As_list = []
    non_As_list = []
    for agent in tuple_list:
        agent_in_As = False
        for as_temp in As:
            if agent['agent'] == as_temp:
                As_list.append(agent)
                agent_in_As = True
        if not agent_in_As:
            non_As_list.append(agent)
    return As_list, non_As_list


def get_largest_component(start_point, point_list):
    open_list = []
    close_list = []
    open_list.append(start_point)
    while len(open_list) != 0:
        extend_point = open_list.pop(0)
        for i in range(-1, 2):
            for j in range(-1, 2):
                temp = {'x': extend_point['loc'][0]['x'] + i, 'y': extend_point['loc'][0]['x'] + i}
                for point in point_list:
                    if temp['x'] == point['loc'][0]['x'] and temp['y'] == point['loc'][0]['y']:
                        open_list.append(temp)
        close_list.append(extend_point)
    return close_list


def get_location(path, time):
    if time <= 0:
        return {'x': path[0]['x'], 'y': path[0]['y'], 't': time}
    elif time < len(path):
        return {'x': path[time]['x'], 'y': path[time]['y'], 't': time}
    else:
        return {'x': path[-1]['x'], 'y': path[-1]['y'], 't':time}


def detect_collision(path1, path2):
    path_1 = path1['path']
    path_2 = path2['path']

    max_size = max(len(path_1), len(path_2))
    collision_list = []
    for i in range(max_size):
        if get_location(path_1, i) == get_location(path_2, i):
            collision = {'a1': path1['agent'],
                         'a2': path2['agent'],
                         'loc': [path_1[i]],
                         'timestep': i
                         }
            collision_list.append(collision)
        elif (get_location(path_1, i), get_location(path_1, i + 1)) == \
                (get_location(path_2, i + 1), get_location(path_2, i)):
            collision = {'a1': path1['agent'],
                         'a2': path2['agent'],
                         'loc': [path_1[i], path_1[i + 1]],
                         'timestep': i + 1
                         }
            collision_list.append(collision)
    return collision_list
    pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("map", help="input file containing map and dynamic obstacles")

    args = parser.parse_args()

    with open(args.map, 'r') as map_file:
        try:
            global map
            map = yaml.load(map_file, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)

    P = []

    # find path with not collision free on all agent
    for i in range(len(map["agents"])):
        sipp_planner = SippPlanner(map, i, 'singe_agent')
        if sipp_planner.compute_plan():
            P.append(sipp_planner.get_plan())

    # prepare data structure for collision detect
    path_agent_tuples = []
    for i in range(len(P)):
        res = list(P[i].keys())[0]
        path_agent_tuples.append({'agent': res, 'path': P[i][res]})

    Collision_list = []  # Collision_list is the detected collision
    path_agent_combination = list(itertools.combinations(path_agent_tuples, 2))
    for j in range(len(path_agent_combination)):
        Collision_list = detect_collision(path_agent_combination[j][0], path_agent_combination[j][1])

    # random choice a collision
    selected_collision = random.choice(Collision_list)
    # find the largest connected component
    largest_component = get_largest_component(selected_collision, Collision_list)
    As_draft = []
    global As_list
    # pre-process the As. If |V'| < N then add some extra agent inside, if not, select random V into As list
    if len(largest_component) < N:
        for comp in largest_component:
            As_draft.append(comp['a1'])
            As_draft.append(comp['a2'])
        As_draft = list(set(As_draft))
        print(As_draft)
        As_list, non_As_list = get_As_split(As_draft, path_agent_tuples)
        print("As list is: ", As_list)
        print("Non As list is: ", non_As_list)
        while len(As_list) >= N:
            if len(non_As_list) == 0:
                break
            non_As = non_As_list.pop(0)
            for collision in Collision_list:
                for agent in As_list:
                    if (collision['a1'] == agent['agent'] and collision['a2'] == non_As['agent']) or \
                            (collision['a2'] == agent['agent'] and collision['a1'] == non_As['agent']):
                        As_list.append(non_As)
    else:
        component_slice = random.sample(largest_component, N)
        for comp in component_slice:
            As_draft.append(comp['a1'])
            As_draft.append(comp['a2'])
        As_draft = list(set(As_draft))
        As_list, non_As_list = get_As_split(As_draft, path_agent_tuples)
    pass

    result = []
    # Contribute constraint list
    while len(As_list) != 0:
        find_result = agent_random_walk(path_agent_tuples, Collision_list, map)
        if find_result is not None:
            print(find_result)
            result.append(find_result)
    print("result is:", result)

if __name__ == "__main__":
    # listOfDicts = [
    #     {"name": "Tommy", "age": 20},
    #     {"name": "Markus", "age": 25},
    #     {"name": "Pamela", "age": 27},
    #     {"name": "Richard", "age": 22}
    # ]
    # var = listOfDicts.index(list(filter(lambda item: item['name'] == 'Richard', listOfDicts))[0])
    # listOfDicts[var].update({'name':'AAAAA', 'age': 1000000})
    # print(listOfDicts)
    main()
