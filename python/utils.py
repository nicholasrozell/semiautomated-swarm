import math


def dist(p, q):
    """
    Finds the Euclidean distance between two points.
    """
    return math.sqrt(sum([(a - b)**2 for a, b in zip(p, q)]))


def angle(a, b):
    """
    Finds the angle between the horizontal and two points.
    """
    x1, y1, z1 = a
    x2, y2, z2 = b
    return math.atan2(y2 - y1, x2 - x1)
