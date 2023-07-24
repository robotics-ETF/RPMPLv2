from compound_space import CompoundSpace
from so2 import SO2
from math import pi

N = 2
spaces = [SO2() for i in range(N)]
cs = CompoundSpace(spaces)
print(cs.get_qrand())
print(cs.get_qnew([0, 0], [pi, pi], 0.1))