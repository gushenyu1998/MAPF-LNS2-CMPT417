

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
    parser.add_argument("--output", help="output file with the schedule")
    
    
    args = parser.parse_args()
    
    # Read Map
    for input,output in zip(glob.glob(args.map),glob.glob(args.output)):
        f = open(input, 'r')
        o = open(output, 'r')
        map = yaml.load(f, Loader=yaml.FullLoader)
        out = yaml.load(o, Loader=yaml.FullLoader)
        
        for i in range(len(map["agents"])):
            sipp_planner = SippPlanner(map,i)
            if sipp_planner.compute_plan():
                plan = sipp_planner.get_plan()
                out["schedule"].update(plan)
                map["dynamic_obstacles"].update(plan)
                w = open(output, 'w')
                yaml.safe_dump(out, w)
            else: 
                print("Plan not found")
        print(out)




if __name__ == "__main__":
    main()
