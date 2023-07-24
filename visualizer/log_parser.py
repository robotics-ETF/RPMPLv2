import re
from copy import deepcopy
from sre_parse import FLAGS

class LogParser:
    def __init__(self, file_name=None) -> None:
        if file_name is None:
            raise RuntimeError("No filename provided!")

        self.path = None

        with open(file_name) as file:
            self.path = self.parse_file(file)

    def get_path(self):
        return self.path

    def parse_file(self, file=None):
        if file is None:
            return
        
        log_file_text = file.read()
        path_string = self._extract_path(log_file_text)
        path = []
        for p in path_string:
            if p[0] == '-':
                break
            con = self._parse_configuration(p)
            path.append(con[0])
        file.close()
        return path

    def _extract_path(self, log_file_text):
        log_file_text = log_file_text.replace("NONE", "None")
        path_string = re.findall(r"Path:\n(.*\n)+", log_file_text, flags=re.DOTALL)[0]
        path_string = path_string.split("\n")
        for i, p in enumerate(path_string):
            p = p.replace("parent q: ", "")
            p = p.replace("q: ", "")
            path_string[i] = p
            if len(p) == 0:
                path_string.remove(p)
        return path_string

    def _parse_configuration(self, p):
        line = p.split("; ")
        p1 = line[0]
        p2 = line[1]
        p1 = p1.replace("(", "")
        p1 = p1.replace(")", "")

        p2 = p2.replace("(", "")
        p2 = p2.replace(")", "")

        p1_s = p1.split(" ")
        p2_s = p2.split(" ")

        if len(p1_s) == 1:
            p1_s = None
        else:
            p1_s = [float(i) for i in list(filter(None, p1_s))]
        if len(p2_s) == 1:
            p2_s = None
        else:
            p2_s = [float(i) for i in list(filter(None, p2_s))]
        return p1_s, p2_s



if __name__ == "__main__":
    parser = LogParser("/home/nermin/SPEAR/RPMPLv2/visualizer/plannerData.log")
    path = parser.get_path()
    for p in path:
        print(p)




        

