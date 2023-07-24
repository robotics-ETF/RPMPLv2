from xml.etree.ElementTree import PI
from log_parser import LogParser
from planar_10dof_vis import visualize
import yaml
from math import pi

if __name__ == "__main__":
    # scenario_file_path = "data/planar_10dof/scenario_test/scenario_test.yaml"
    scenario_file_path = "data/planar_10dof/scenario1/scenario1.yaml"

    parser = LogParser("../" + scenario_file_path[0:len(scenario_file_path)-5] + "_planner_data.log")
    path = parser.get_path()
    # path = [1,-1,1,-1,1,-1,1,-1,1,-1]
    #for p in path:
    #    print(p)
    #print(path)
    # visualize(path, "test.png")
    
    with open("../" + scenario_file_path, "r") as file:
        obstacles = yaml.safe_load(file)
    visualize(path, obstacles=obstacles, image_file="../" + scenario_file_path[0:len(scenario_file_path)-5] + "_planar_10dof.gif", 
              is_trajectory=True, is_dynamic=False, fps=10.0) 
    