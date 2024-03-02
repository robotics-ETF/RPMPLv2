from log_parser import LogParser
from xarm6_vis import visualize
import yaml


if __name__ == "__main__":
    scenario_file_path = "data/xarm6/scenario_real_time/scenario_real_time.yaml" 
    # scenario_file_path = "data/xarm6/scenario1/scenario1.yaml"
    # scenario_file_path = "data/xarm6/scenario2/scenario2.yaml"
    
    parser = LogParser("../data/xarm6/scenario_real_time/scenario_real_time_drgbt_test1.log")
    path = parser.get_path()
    # path = [1.5708, 1.5708, -2.3562, 0, 0, 0]
    #for p in path:
    #    print(p)
    #print(path)
    #visualize(path[0], "test.png")
    
    with open("../" + scenario_file_path, "r") as file:
        obstacles = yaml.safe_load(file)
        print(obstacles)
    visualize(path, obstacles=obstacles, image_file="../" + scenario_file_path[0:len(scenario_file_path)-5] + "_xarm6.gif", 
              is_trajectory=True, is_dynamic=True, fps=10.0) 