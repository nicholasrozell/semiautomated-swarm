import sys
sys.path.append('../')
import numpy as np
import time
from shapely.geometry.point import Point
from shapely.geometry import Polygon

from pathplanning.graph import Graph
from pathplanning.algorithms3 import RRTStar
from pathplanning.views3 import draw
from pathplanning.utils import dist


def main():
    print('\nInitializing.')
    dims = np.array([(-50, 50), (-50, 50), (-10, -10)])
    ob1 = [-25, -25]
    ob2 = [-25, 25]
    ob3 = [-25, 0]
    obstacles = [(Point(ob1).buffer(5)),
                 (Point(ob2).buffer(5)),
                 (Point(ob3).buffer(5))]
    init = (0, 0, -10)
    goal = (40, 40, -10)
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
        if dist(path[-1],goal) <= delta:
            print('Goal Reached.')
            break

        ob1[0] += 1
        ob2[0] += 5
        ob3[0] += 3
        obstacles = [(Point(ob1).buffer(5)),
                         (Point(ob2).buffer(5)),
                         (Point(ob3).buffer(5))]
        graph.update_obstacles(obstacles)


if __name__=='__main__':
    main()
