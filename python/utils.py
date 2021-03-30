import math
import numpy as np


def dist(p, q):
    """
    Finds the Euclidean distance between two points.
    """
    return math.sqrt(sum([(a - b)**2 for a, b in zip(p, q)]))  # Gets stuck here for some reason

def angle(a, b):
    """
    Finds the angle between the horizon and two points.
    """
    x1, y1, z1 = a
    x2, y2, z2 = b
    return np.arctan2(y2 - y1, x2 - x1)
    