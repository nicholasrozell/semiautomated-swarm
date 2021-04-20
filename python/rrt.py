import numpy as np


class BaseRRT:
    """
    Base class for rapidly-exploring random trees.
    """
    def __init__(self):
        pass

    def sample_free(self):
        pass

    def nearest(self):
        pass

    def near(self):
        pass

    def brute_force(self):
        pass

    def steer(self):
        pass

    def bound_point(self):
        pass

    def saturate(self):
        pass

    def shrinking_ball_radius(self):
        pass

    def parent(self):
        pass

    def children(self):
        pass

    def is_leaf(self):
        pass

    def connect_to_goal(self):
        pass

    def merge(self):
        pass

    def construct_path(self):
        pass

    def compute_trajectory(self):
        pass

    def cost(self):
        pass


class MiniRRT(BaseRRT):
    """
    Class for optimal imcrementail RRT.
    """
    def __init__(self):
        pass

    def extend(self):
        pass

    def find_parent(self):
        pass

    def rewire_neighbors(self):
        pass

    def search(self):
        """
        Function to search through the graph
        """
        pass
