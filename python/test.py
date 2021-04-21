import numpy as np
import time

from graph import Graph
from rrt import MiniRRTStar as RRT
from views import draw
from utils import dist


def main():
    print('\nInitializing\n')
    #                   x         y           z
    dims = np.array([(0, 120), (0, 120), (-10, -10)])
    init = (110, 10, -10)
    goal = (20, 100, -10)
    delta = 7
    k = 3
    
    graph = Graph(dims)
    path = None
    PATH = []
    t1 = time.time()

    print('Calculating Trajectory...\n')
    while 1:
        if graph.num_nodes() == 0:
            rrt = RRT(graph, init, goal, delta, k, path)
        if graph.num_nodes() <= 250:
            path, leaves = rrt.search()
        
        else:
            init = tuple(path[1])
            PATH.append(path)

            if dist(init, goal) <= delta:
                print('Goal Reached.')
                print('Search Time :  {} s\n'.format(round(time.time() - t1, 3)))
                print('Generating plot...\n')
                draw(graph, init, goal, PATH, leaves)
                break

            graph.clear()


if __name__ == '__main__':
    main()
    print('TERMINATED')
