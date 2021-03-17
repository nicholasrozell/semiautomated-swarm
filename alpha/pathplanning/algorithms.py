import numpy as np
from .utils import dist


class BaseRRT:
    """
    Base class for rapidly-exploring random trees.
    """
    def __init__(self, graph, x_init, x_goal, delta, k):
        self.graph = graph
        self.x_init = x_init
        self.x_goal = x_goal
        self.delta = delta
        self.k = k

    def sample_free(self):
        """
        Returns a random node from within the graph.
        """
        return tuple(np.random.uniform(self.graph.span[:, 0], self.graph.span[:, 1]))
    
    def nearest(self, v, r):
        """
        Finds the nerest neighbor to the node 'v' within a ball radius 'r'.
        """
        min_dist = float('inf')
        nodes = self.near(v, r)
        for node in nodes:
            if dist(v, node) <= min_dist:
                min_dist = dist(v, node)
                nearest = node
        try:
            return nearest
        except UnboundLocalError:
            raise UnboundLocalError("Could not find a nearest node, check the k factor.")

    def near(self, pivot, radius):
        """
        Checks if nearby nodes are within a radius.

        """
        circle_x, circle_y = pivot
        nodes = []
        for node in self.graph._node:
            x, y = node
            if ((x - circle_x) * (x - circle_x) + (y - circle_y) * (y - circle_y)) <= radius * radius:
                nodes.append((x, y))
        return nodes

    def steer(self, x_nearest, x_rand):
        """
        Guides edge to nearest node.
        """
        if dist(x_nearest, x_rand) > self.delta:
            return self.bound_point(self.saturate(x_nearest, x_rand))
        return self.bound_point(x_rand)

    def bound_point(self, node):
        """
        Bounds a node to within the graph.
        """
        node = np.maximum(node, self.graph.span[:, 0])
        node = np.minimum(node, self.graph.span[:, 1])
        return tuple(node)

    def saturate(self, v, w):
        """
        Shortens the distance between nodes 'v' and 'w'.
        """
        start, end = np.array(v), np.array(w)
        z = end - start
        u = z / np.sqrt((np.sum(z**2)))
        saturated_node = start + u*self.delta
        return tuple(saturated_node)

    def shrinking_ball_radius(self):
        """
        Radius used to find nearest nodes. Shrinks as the number of nodes increases.
        """
        d = self.graph.dimensions
        leb_meas = (self.graph.span[0][1] - self.graph.span[0][0]) * (self.graph.span[1][1] - self.graph.span[1][0])
        zeta_D = (4.0/3.0) * self.delta**3
        gamma = (2**d*(1 + (1/d))*leb_meas)**self.k
        r = int(((gamma/zeta_D)*(np.log10(self.graph.num_nodes())/self.graph.num_nodes())))**(1/d)
        if r == 0.0:
            r = self.graph.span[0][1] + self.graph.span[1][1]
        return r

    def parent(self, v):
        """
        Returns the parent of node v.
        """
        return list(self.graph.predecessor(v))[0]

    def connect_to_goal(self, v):
        if self.x_goal not in self.graph._node:
            if dist(v, self.x_goal) <= self.delta:
                self.graph.add_node(self.x_goal)
                self.graph.add_edge(v, self.x_goal)

    def compute_trajectory(self):
        return self.construct_path(self.x_init, self.x_goal)

    def construct_path(self, start, end):
        """
        Constructs a path between the start and end nodes, starting from the end to start.
        """
        path = [end]
        current = end
        if start == end:
            return path
        while start not in path:
            path.append(self.parent(current))
            current = self.parent(current)
        path.reverse()
        return path


class RRT(BaseRRT):
    """
    Class for the basic RRT algorithm.

    ref1: Randomized Kinodynamic Planning;
    Lavelle, Kuffner
    ref2: Sampling-based Algorithms for Optimal Motion Planning; 
    Karaman, Frazzoli    
    """
    def __init__(self, graph, x_init, x_goal, delta, k):
        super().__init__(graph, x_init, x_goal, delta, k)

    def search(self):
        self.graph.add_node(self.x_init)
        while self.graph.num_nodes() <= 2500:
            x_rand = self.sample_free()
            x_nearest = self.nearest(x_rand, self.shrinking_ball_radius())
            x_new = self.steer(x_nearest, x_rand)
            if self.graph.collision_free(x_nearest, x_new):
                self.graph.add_node(x_new)
                self.graph.add_edge(x_nearest, x_new)
            self.connect_to_goal(x_new)
        return self.compute_trajectory()


class RRTStar(BaseRRT):
    """
    Class for the optimal RRT algorithm.
    
    ref: Sampling-based Algorithms for Optimal Motion Planning; 
    Karaman, Frazzoli
    """
    def __init__(self, graph, x_init, x_goal, delta, k):
        super().__init__(graph, x_init, x_goal, delta, k)

    def cost(self, child):
        """
        Calculates the cost between the child node and the root node.
        """
        cost = 0
        while child != self.x_init:
            parent = self.parent(child)
            cost += dist(parent, child)
            child = parent
        return cost

    def search(self):
        self.graph.add_node(self.x_init)
        while self.graph.num_nodes() <= 2500:
            r = self.shrinking_ball_radius()
            x_rand = self.sample_free()
            x_nearest = self.nearest(x_rand, r)
            x_new = self.steer(x_nearest, x_rand)
            if self.graph.collision_free(x_nearest, x_new):
                X_near = self.near(x_new, self.delta)
                self.graph.add_node(x_new)
                x_min = x_nearest
                c_min = self.cost(x_nearest) + dist(x_nearest, x_new)
                for x_near in X_near:
                    if self.graph.collision_free(x_near, x_new) and self.cost(x_near) + dist(x_near, x_new) < c_min:
                        x_min = x_near
                        c_min = self.cost(x_near) + dist(x_near, x_new)
                self.graph.add_edge(x_min, x_new)
                for x_near in X_near:
                    if self.graph.collision_free(x_new, x_near) and self.cost(x_new) + dist(x_new, x_near) < self.cost(x_near):
                        x_parent = self.parent(x_near)
                        self.graph.remove_edge(x_parent, x_near)
                        self.graph.add_edge(x_new, x_near)
            self.connect_to_goal(x_new)
        return self.compute_trajectory()
