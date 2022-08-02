import argparse
import itertools
import yaml
import random
from math import fabs
from sipp import SippPlanner

def get_location(path,time):
    if time<=0:
        return {'x':path[0]['x'],'y':path[0]['y']}
    elif time < len(path):
        return {'x':path[time]['x'],'y':path[time]['y']}
    else:
        return {'x':path[-1]['x'],'y':path[-1]['y']}

def detect_collision(path1, path2):
    path_1 = path1['path']
    path_2 = path2['path']
    max_size = max(len(path_1), len(path_2))
    collision_list = []
    for i in range(max_size):
        if get_location(path_1,i) == get_location(path_2,i):
            collision = {'a1':path1['agent'],
                        'a2':path2['agent'],
                        'loc':[path_1[i]],
                        'timestep': i
                        }
            collision_list.append(collision)
        elif (get_location(path_1, i), get_location(path_1, i + 1)) == \
                (get_location(path_2, i + 1), get_location(path_2, i)):
            collision = {'a1':path1['agent'],
                        'a2':path2['agent'],
                        'loc':[path_1[i],path_1[i+1]],
                        'timestep': i+1
                        }
            collision_list.append(collision)
    return collision_list
    pass

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("map", help="input file containing map and dynamic obstacles")
    parser.add_argument("output", help="output file with the schedule")
    
    args = parser.parse_args()
    
    with open(args.map, 'r') as map_file:
        try:
            map = yaml.load(map_file, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)

    with open(args.output, 'r') as output_yaml:
        try:
            output = yaml.load(output_yaml, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)

    P = []

    for i in range(len(map["agents"])):
        sipp_planner = SippPlanner(map,i)
        if sipp_planner.compute_plan():
            P.append(sipp_planner.get_plan())

    path_agent_tuples = []
    for i in range(len(P)):
        res = list(P[i].keys())[0]
        path_agent_tuples.append({'agent':res, 'path': P[i][res]})

    path_agent_combination = list(itertools.combinations(path_agent_tuples,2))

    for j in range(len(path_agent_combination)):
        V = detect_collision(path_agent_combination[j][0], path_agent_combination[j][1])

    print(V)

    selected_collision = random.choice(V)

    print(selected_collision)

    As = []
    As.append(selected_collision['a1'])
    As.append(selected_collision['a2'])

    print(len(V))
    for k in range(len(V)):
        if selected_collision['a1'] == V[k]['a1'] and selected_collision['a2'] != V[k]['a2']:
            As.append(V[k]['a2'])
        elif selected_collision['a1'] == V[k]['a2'] and selected_collision['a2'] != V[k]['a1']:
            As.append(V[k]['a1'])
        elif selected_collision['a2'] == V[k]['a1'] and selected_collision['a1'] != V[k]['a2']:
            As.append(V[k]['a2'])
        elif selected_collision['a2'] == V[k]['a2'] and selected_collision['a1'] != V[k]['a1']:
            As.append(V[k]['a1'])
    
    print(As)


        


if __name__ == "__main__":
    main()
    
