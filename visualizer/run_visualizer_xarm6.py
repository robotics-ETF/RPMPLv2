from log_parser import LogParser
from xarm6_vis import visualize
import yaml
import os


if __name__ == "__main__":
    data_file_path = "/RRTx_data/test10_6.log"

    scenario_file_path = "/data/xarm6/scenario_random_obstacles/scenario_test.yaml" 
    # scenario_file_path = "/data/xarm6/scenario1/scenario1.yaml"
    # scenario_file_path = "/data/xarm6/scenario2/scenario2.yaml"
    
    project_path = os.path.dirname(os.path.abspath(__file__))
    project_path = project_path.rsplit('/', 1)[0]
    print(project_path + scenario_file_path.rsplit('/', 1)[0] + data_file_path)

    parser = LogParser(project_path + scenario_file_path.rsplit('/', 1)[0] + data_file_path)
    path = parser.get_path()
    # path = [1.5708, 1.5708, -2.3562, 0, 0, 0]
    #for p in path:
    #    print(p)
    #print(path)
    #visualize(path[0], "test.png")
    
    with open(project_path + scenario_file_path, "r") as file:
        obstacles = yaml.safe_load(file)
    visualize(path, obstacles=obstacles, 
              image_file=project_path + scenario_file_path.rsplit('/', 1)[0] + data_file_path + "xarm6.gif", 
              is_trajectory=True, is_dynamic=True, duration=50) 