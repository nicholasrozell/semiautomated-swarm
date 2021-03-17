import math
import numpy as np
import scipy.interpolate as si


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

def bspline(cv, n=100, degree=3, periodic=False):  # Used for path smoothing
    """ Calculate n samples on a bspline

        cv :      Array ov control vertices
        n  :      Number of samples to return
        degree:   Curve degree
        periodic: True - Curve is closed
                False - Curve is open
    """

    # If periodic, extend the point array by count+degree+1
    cv = np.asarray(cv)
    count = len(cv)

    if periodic:
        factor, fraction = divmod(count+degree+1, count)
        cv = np.concatenate((cv,) * factor + (cv[:fraction],))
        count = len(cv)
        degree = np.clip(degree,1,degree)

    # If opened, prevent degree from exceeding count-1
    else:
        degree = np.clip(degree,1,count-1)

    # Calculate knot vector
    kv = None
    if periodic:
        kv = np.arange(0-degree,count+degree+degree-1)
    else:
        kv = np.clip(np.arange(count+degree+1)-degree,0,count-degree)

    # Calculate query range
    u = np.linspace(periodic,(count-degree),n)

    # Calculate result
    return list(np.array(si.splev(u, (kv,cv.T,degree))).T)
