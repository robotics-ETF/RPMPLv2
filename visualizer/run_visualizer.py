from log_parser import LogParser
from planar_2dof_vis import visualize
import yaml
from math import pi
import os


if __name__ == "__main__":
    #parser = LogParser("/tmp/plannerData.log")
    #path = parser.get_path()
    
    
    #for p in path:
    #    print(p)
    #print(path)
    #visualize(path[0], "test.png")

    project_path = os.path.dirname(os.path.abspath(__file__))
    project_path = project_path.rsplit('/', 1)[0]
    print(project_path + scenario_file_path.rsplit('/', 1)[0] + data_file_path)

    with open(project_path + "/data/planar_2dof/scenario_easy.yaml", 'r') as file:
        obstacles = yaml.safe_load(file)     
    
    visualize([1.35, -2.2], obstacles=obstacles, image_file=None, is_trajectory=False, duration=100)
    
