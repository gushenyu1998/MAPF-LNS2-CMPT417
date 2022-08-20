import argparse
import yaml
from math import fabs
from graph_generation import SippGraph, State
from sipp import SippPlanner
import glob
from pathlib import Path

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map", help="input file containing map and dynamic obstacles")
    args = parser.parse_args()
    
    # Read Map
    for input in glob.glob(args.map):
        f = open(input, 'r')
        map = yaml.load(f, Loader=yaml.FullLoader)
        out = {'schedule':{}}

        for i in range(len(map["agents"])):
            sipp_planner = SippPlanner(map,i, 'singe_agent')
            if sipp_planner.compute_plan():
                plan = sipp_planner.get_plan()
                print(plan)
                out["schedule"].update(plan)
            else:
                plan = sipp_planner.get_plan()
                out["schedule"].update(plan)
        file = open('./output.yaml', 'w', encoding='utf-8')
        yaml.dump(out, file)
        file.close()
if __name__ == "__main__":
    main()
