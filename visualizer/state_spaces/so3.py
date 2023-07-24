import random
import numpy as np
from state_spaces.state_space import StateSpace
from math import pi
from pyquaternion import Quaternion
from copy import deepcopy


class SO3(StateSpace):
    def __init__(self) -> None:
        super().__init__()
        self.range = [-pi, pi]

    def get_dimensions(self) -> int:
        return 3

    def range(self):
        return self.range

    def distance(self, s1, s2):
        return Quaternion.absolute_distance(s1, s2)

    def get_qrand(self):
        return Quaternion.random()

    def get_qnew(self, qnear, qrand, eps):
        return Quaternion.slerp(qnear, qrand, amount=eps)

    def equal(self, state1, state2, eps):
        return self.distance(state1, state2) < eps
