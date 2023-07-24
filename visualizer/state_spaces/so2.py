import random
import numpy as np
from state_spaces.state_space import StateSpace
from math import pi


class SO2(StateSpace):
    def __init__(self) -> None:
        super().__init__()
        self.range = [-pi, pi]

    def range(self):
        return self.range

    def get_qrand(self):
        return random.uniform(self.range[0], self.range[1])

    def get_qnew(self, qnear, qrand, eps):
        a = qnear
        b = qrand
        return a + eps*(b-a)/np.linalg.norm(b-a)

    def equal(self, state1, state2, eps):
        return np.fabs(state1 - state2) < eps
