import glob
from pathlib import Path
from ruamel import yaml


def import_mapf_instance(filename):
    f = Path(filename)
    if not f.is_file():
        raise BaseException(filename + " does not exist.")
    f = open(filename, 'r')
    # first line: #rows #columns
    line = f.readline()
    rows, columns = [int(x) for x in line.split(' ')]
    rows = int(rows)
    columns = int(columns)
    # #rows lines with the map
    my_map = []
    for r in range(rows):
        line = f.readline()
        my_map.append([])
        for cell in line:
            if cell == '@':
                my_map[-1].append(True)
            elif cell == '.':
                my_map[-1].append(False)
    # #agents
    line = f.readline()
    num_agents = int(line)
    # #agents lines with the start/goal positions
    starts = []
    goals = []
    for a in range(num_agents):
        line = f.readline()
        sx, sy, gx, gy = [int(x) for x in line.split(' ')]
        starts.append((sx, sy))
        goals.append((gx, gy))
    f.close()
    return my_map, starts, goals


if __name__ == '__main__':
    for file in sorted(glob.glob(r'./instances/exp2_1.txt')):
        my_map, starts, goals = import_mapf_instance(file)
        if len(starts) != len(goals):
            raise BaseException("start_goal error")
        agents_content = []
        print(my_map)
        for i in range(len(starts)):
            temp = {'start': list(starts[i]),
                    'goal': list(goals[i]),
                    'name': "agent" + str(i)}
            agents_content.append(temp)
        soild_obs = []
        for i in range(len(my_map[0])):
            for j in range(len(my_map)):
                if my_map[j][i]:
                    soild_obs.append((j,i))
        map_solid_obs = {'dimensions':[len(my_map),len(my_map[0])],
                         'obstacles':soild_obs,
                        }

        agents = {'agents': agents_content, 'map':map_solid_obs, 'dynamic_obstacles':{}}
        file = open('./yaml_test.yaml', 'w', encoding='utf-8')
        yaml.dump(agents, file)
        file.close()


    pass
