import numpy as np
from state_spaces.state_space import StateSpace


class CompoundSpace(StateSpace):
    def __init__(self, spaces) -> None:
        super().__init__()
        self.spaces = spaces

    def spaces(self):
        return self.spaces

    def range(self):
        return np.array([space.range for space in self.spaces])

    def get_qrand(self):
        return np.array([space.get_qrand() for space in self.spaces])

    def get_qnew(self, qnear, qrand, eps):
        a = qnear
        b = qrand
        return a + eps*(b-a)/np.linalg.norm(b-a)

    def equal(self, state1, state2, eps):
        return np.linalg.norm(np.array(state1) - np.array(state2)) < eps
