import sys
sys.path.append('../')
import numpy as np
import time

from pathplanning.graph import Graph
from pathplanning.algorithms import RRT
from pathplanning.views import draw


def main():
    print('\nInitializing.')
    dims = np.array([(-50, 50), (-50, 50)])
    init = (0, 0)
    goal = (45, 45)
    delta = 2
    k = 11

    graph = Graph(dims)
    rrt = RRT(graph, init, goal, delta, k)

    t1 = time.time()
    print('Searching.')
    rrt.search()
    print('Finished.')
    print('Search time: {} seconds.\n'.format(round(time.time() - t1, 3)))

    draw(graph, init, goal)

if __name__=='__main__':
    main()
