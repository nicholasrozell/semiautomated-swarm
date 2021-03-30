import numpy as np
from .utils import dist, angle

class BaseRRT:
    """
    Base class for rapidly-exploring random trees.
    """
    def __init__(self, graph, x_init, x_goal, delta, k, path):
        self.graph = graph
        self.x_init = x_init
        self.x_goal = x_goal
        self.delta = delta
        self. k = k

        self.alpha = np.radians(120)
        self.path = path
        self.range = self.delta*4

    def sample_free(self):
        """

        """
        r = self.range * np.sqrt(np.random.uniform())
        if self.path == None:
            theta = np.random.uniform() * self.alpha + (np.radians(0) - self.alpha/2)
        else:
            theta = np.random.uniform() * self.alpha + (angle(self.path[-2], self.path[-1]) - self.alpha/2)
        return tuple((self.x_init[0] + r*np.cos(theta), self.x_init[1] + r*np.sin(theta), self.graph.span[2][0]))

    def nearest(self, v, r):
        """
        Finds the nearest neighbor to the node 'v' withing a ball radius 'r'.
        """
        min_dist = float('inf')
        nodes = self.near(v, r)
        for node in nodes:
            if dist(v, node) <= min_dist:
                min_dist = dist(v, node)
                nearest = node
        return nearest

    def near(self, pivot, radius):
        """
        Checks if nearby nodes are within a radius.
        """
        circle_x, circle_y, circle_z = pivot
        nodes = []
        for node in self.graph._node:
            x, y, z = node
            if ((x - circle_x) * (x - circle_x) + (y - circle_y) * (y - circle_y)) <= radius * radius:
                nodes.append((x, y, z))
        return nodes

    def steer(self, x_nearest, x_rand):
        """
        """
        if dist(x_nearest, x_rand) > self.delta:
            return self.bound_point(self.saturate(x_nearest, x_rand, self.delta))
        return self.bound_point(x_rand)

    def bound_point(self, node):
        """
        Bounds a node to within the graph.
        """
        node = np.maximum(node, self.graph.span[:, 0])
        node = np.minimum(node, self.graph.span[:, 1])
        return tuple(node)

    def saturate(self, v, w, delta):
        """
        Shortens the distance between nodes 'v' and 'w'.
        """
        start, end = np.array(v), np.array(w)
        z = end - start
        u = z / np.sqrt((np.sum(z**2)))
        saturated_node = start + u*delta
        return tuple(saturated_node)

    def shrinking_ball_radius(self):
        """
        Radius used to find nearest nodes.
        Shrinks as the number of nodes increases.
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
        Returns the parent of node 'v'.
        """
        if list(self.graph.predecessor(v)) == []:
            return None
        return list(self.graph.predecessor(v))[0]

    def children(self, v):
        """
        Returns the parent of node 'v'.
        """
        return list(self.graph.successors(v))

    def is_leaf(self, v):
        """
        "Checks if node 'v' is a leaf. A node with no children.
        """
        if self.graph.degree(v) == 0:
            return True
        return False

    def depth(self, child):
        """
        Checks the depth of the node 'child'.
        """
        node_depth = 0
        while self.parent(child) != self.x_init:
            node_depth += 1
            child = self.parent(child)
        return node_depth

    def g(self, v):
        """
        Calculates cost to between node 'v' and goal.
        """
        # if self.graph.collision_free(v, self.x_goal):
        #     return dist(v, self.x_goal)
        # else:
        #     return float('inf')

        return dist(v, self.x_goal)

    def cost(self, v):
        """
        Calculates the cost from node to node until the root node is reached.
        """
        cost = 0
        while v != self.x_init:
            cost += self.graph._node[v]
            v = self.parent(v)
        return cost

    def compute_trajectory(self):
        """
        """
        leaves = []
        for n in self.graph._node.keys():
            if self.is_leaf(n):
                leaves.append(n)
        goal = self.best_leaf(leaves)
        return self.construct_path(self.x_init, goal), leaves

    def best_leaf(self, leaves):
        """
        """
        best = float('inf')
        for leaf in leaves:

            # theta = angle(self.parent(leaf), leaf)
            # temp = tuple((leaf[0] + (self.delta*3) * np.cos(theta), leaf[1] + (self.delta*3) * np.sin(theta), self.graph.span[2][0]))
            # if self.graph.collision_free(leaf, temp):
            #     temp_cost = 0
            # else:
            #     temp_cost = float('inf')
            
            self.graph._node[leaf] = self.g(leaf) - self.cost(leaf)
            if self.graph._node[leaf] < best:
                best = self.graph._node[leaf]
                goal = leaf
        return goal

    def construct_path(self, start, end):
        """
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


class RRTStar(BaseRRT):
    """
    Class for 
    """
    def __init__(self, graph, x_init, x_goal, delta, k, path, max_nodes=250):
        self.max_nodes = max_nodes
        super().__init__(graph, x_init, x_goal, delta, k, path)

    def search(self):
        """
        Main function
        """
        self.graph.add_node(self.x_init, 0)
        while self.graph.num_nodes() <= self.max_nodes:
            r = self.shrinking_ball_radius()
            x_rand = self.sample_free()
            x_nearest = self.nearest(x_rand, r)
            x_new = self.steer(x_nearest, x_rand)
            if self.graph.collision_free(x_nearest, x_new):
                X_near = self.near(x_new, self.delta)
                self.graph.add_node(x_new, dist(x_nearest, x_new))
                x_min = x_nearest
                c_min = self.cost(x_nearest) + dist(x_nearest, x_new)            
                for x_near in X_near:
                    if self.graph.collision_free(x_near, x_new) and self.cost(x_near) + dist(x_near, x_new) < c_min:
                        x_min = x_near
                        c_min = self.cost(x_near) + dist(x_near, x_new)
                self.graph.add_edge(x_min, x_new)
                self.graph._node[x_new] = dist(x_min, x_new)
                for x_near in X_near:
                    if self.graph.collision_free(x_new, x_near) and self.cost(x_new) + dist(x_new, x_near) < self.cost(x_near):
                        x_parent = self.parent(x_near)
                        self.graph.remove_edge(x_parent, x_near)
                        self.graph.add_edge(x_new, x_near)
                        self.graph._node[x_near] = dist(x_new, x_near)
        return self.compute_trajectory()
