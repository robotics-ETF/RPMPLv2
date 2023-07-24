
class StateSpace:
    def __init__(self) -> None:
        pass

    def distance(self, s1, s2) -> float:
        return -1

    def get_dimensions(self) -> int:
        return -1

    def is_valid(self, q1, q2=None) -> bool:
        pass

    def distance(self, q) -> float:
        pass

    def get_qrand(self):
        pass

    def get_qnew(self):
        pass

    def equal(self, state1, state2, eps):
        pass
