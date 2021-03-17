import sys
sys.path.append('../')
import numpy as np
import time
from shapely.geometry.point import Point
from shapely.geometry import Polygon

from pathplanning.graph import Graph
from pathplanning.algorithms3 import RRTStar
from pathplanning.views3 import draw


def main():
    print('\nInitializing.')
    dims = np.array([(-50, 50), (-50, 50), (-10, -10)])
    obstacles = [(Point(-25, -25).buffer(10)),
                 (Polygon([(30, 30), (0, 30), (0, 25),
                           (25, 25), (25, 0), (30, 0)]))]
    init = (0, 0, -10)
    goal = (45, 45, -10)
    delta = 2
    k = 11

    graph = Graph(dims, obstacles)
    path = []
    while 1:
        rrt = RRTStar(graph, init, goal, delta, k, path)

        t1 = time.time()
        print('Searching.')
        path = rrt.search()
        print('Finished.')
        print('Search time: {} seconds.\n'.format(round(time.time() - t1, 3)))

        draw(graph, init, goal, path)
        init = path[3]
        graph.clear()
        if goal in path:
            print('Goal Reached.')
            break

if __name__=='__main__':
    main()
