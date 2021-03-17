import sys
sys.path.append('../')
import numpy as np
import time
from shapely.geometry.point import Point
from shapely.geometry import Polygon

from pathplanning.graph import Graph
from pathplanning.algorithms3 import RRTStarV2
from pathplanning.views3 import draw
from pathplanning.utils import dist


def main():
    print('\nInitializing.')
    dims = np.array([(-50, 50), (-50, 50), (-10, -10)])
    # obstacles = [(Polygon([(30, 30), (-30, 30), (-30, 25), (30, 25)]))]
    init = (0, 0, -10)
    goal = (0, 40, -10)
    delta = 2
    k = 3

    graph = Graph(dims)
    path = []
    ob1 = (5, 5)
    obstacle = [Point(ob1).buffer(5.0)]

    while 1:
        rrt = RRTStarV2(graph, init, goal, delta, k, obstacle, path)

        t1 = time.time()
        print('Searching.')
        path, obstacle = rrt.search()
        print('Finished.')
        print('Search time: {} seconds.\n'.format(round(time.time() - t1, 3)))
        draw(graph, init, goal, path)
        # break
    
        init = tuple(path[3])
        graph.clear()

        if dist(path[-1],goal) <= delta:
            print('Goal Reached.')
            break


if __name__=='__main__':
    main()
    print('Search terminated.')
