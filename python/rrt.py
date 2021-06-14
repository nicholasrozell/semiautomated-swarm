import numpy as np
from utils import dist, angle, bspline


class BaseRRT:
    """
    Base class for rapidly-exploring random trees.
    """
    def __init__(self, graph, x_init, x_goal, delta, k, path, heading):
        self.graph = graph
        self.x_init = x_init
        self.x_goal = x_goal
        self.delta = delta
        self.k = k
        self.heading = np.float64(str(heading)[5:])

        self.alpha = np.radians(120)
        self.path = path
        self.range = self.delta*4

    def sample_free(self):
        """
        Returns a random node within the defined bounds of the graph.
        """
        r = self.range * np.sqrt(np.random.uniform())
        if self.path is None:
            theta = np.random.uniform() * self.alpha + (self.heading - self.alpha/2)  # initial tree sample free, bounds to angle
        else:
            theta = np.random.uniform() * self.alpha + (angle(self.path[0], self.path[1]) - self.alpha/2)  # trees after sample free, bounds to angle
        return tuple((self.x_init[0] + r * np.cos(theta), self.x_init[1] + r * np.sin(theta), self.graph.dims[2][0]))  # random tuple

    def nearest(self, v, r):
        """
        Finds the nearest neighbor to the node 'v' within a ball radius 'r'.
        """
        min_dist = float('inf')
        nodes = self.near(v, r)
        for n in nodes:
            if dist(v, n) < min_dist:
                min_dist = dist(v, n)
                nearest = n
        try:
            return nearest
        except UnboundLocalError:
            raise UnboundLocalError("Could not find a nearest node, check the k factor.")

    def near(self, pivot, radius):
        """
        Checks if nearby nodes are within a radius.
        """
        circle_x, circle_y, circle_z = pivot
        nodes = []
        for n in self.graph.nodes():
            x, y, z = n
            if ((x - circle_x) * (x - circle_x) + (y - circle_y) * (y - circle_y)) <= radius * radius:
                nodes.append((x, y, z))
        return nodes

    def brute_force(self, x, bunch):
        """
        Finds the nearest node to x from bunch.
        """
        min_dist = float('inf')
        for n in bunch:
            if dist(x, n) < min_dist:
                min_dist = dist(x, n)
                nearest = n
        try:
            return nearest
        except TypeError:
            raise TypeError('Bunch list is empty.')

    def steer(self, x_nearest, x_rand):
        """
        Guides edge to nearest node.
        """
        if dist(x_nearest, x_rand) > self.delta:
            return self.bound_point(self.saturate(x_nearest, x_rand, self.delta))
        else:
            return self.bound_point(x_rand)

    def bound_point(self, node):
        """
        Bounds a node to within the graph.
        """
        node = np.maximum(node, self.graph.dims[:, 0])
        node = np.minimum(node, self.graph.dims[:, 1])
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
        Radius used to find nearest nodes. Shrinks as the number of nodes increases.
        Otte and Frazzoli, and Karaman and LaValle define shrinkging ball radius differently from each other
        This method is from Otte and Frazzoli
        """
        d = self.graph.num_dims  # #-dimensions of the graph
        leb_meas = (self.graph.dims[0][1] - self.graph.dims[0][0]) * (self.graph.dims[1][1] - self.graph.dims[1][0])  # calculates the lebesque measure, https://en.wikipedia.org/wiki/Lebesgue_measure
        zeta_D = (4.0/3.0) * self.delta**3  # volume of unit ball
        gamma = (2**d*(1 + (1/d))*leb_meas)**self.k  # gamma variable
        r = int(((gamma/zeta_D)*(np.log10(self.graph.num_nodes())/self.graph.num_nodes())))**(1/d)  # radius
        if r == 0.0:
            r = self.graph.dims[0][1] + self.graph.dims[1][1]
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
        Returns the children of node 'v'.
        """
        return list(self.graph.successors(v))

    def is_leaf(self, v):
        """
        Checks if the node 'v' is a leaf.
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

    def construct_path(self, start, end):
        """
        Constructs a path between start to end.
        """
        path = [end]
        child = end
        if start == end:
            return path
        while start not in path:
            path.append(self.parent(child))
            child = self.parent(child)
        path.reverse()
        return path

    def best_leaf(self, leaves):
        """
        Finds the best leaf in the group of leaves based off of distance to the goal.
        """
        best = float('inf')
        if self.x_goal in self.graph.nodes():
            return self.x_goal
        for leaf in leaves:
            # theta = angle(self.parent(leaf), leaf)
            # temp = tuple((leaf[0] + (self.delta*2) * np.cos(theta), leaf[1] + (self.delta*2) * np.sin(theta), self.graph.dims[2][0]))
            # if self.graph.collision_free(leaf, temp):
            #     temp_cost = self.p_cost(temp)
            # else:
            #     temp_cost = float('inf')

            #if self.depth(leaf) > 3:
            cost = self.g(leaf)  # + temp_cost
            if cost <= best:
                best = cost
                goal = leaf
        return goal

    def compute_trajectory(self):
        """
        Computers the trajectory.
        """
        leaves = []
        for n in self.graph.nodes():
            if self.is_leaf(n):
                leaves.append(n)
        # print(leaves)
        leaf = self.best_leaf(leaves)
        if leaf is None:
            return None
        path = bspline(self.construct_path(self.x_init, leaf), n=4, degree=2)
        #path = self.construct_path(self.x_init, leaf)
        return tuple(map(tuple, path)), leaves

    def g(self, v):
        """
        Returns the cost between node 'v' and the goal.
        """
        return dist(v, self.x_goal)


class MiniRRTP(BaseRRT):
    """
    Class for optimal imcrementail RRT.
    This class uses potential fields, so pgraph is needed.
    """
    def __init__(self, graph, x_init, x_goal, delta, k, path, heading):
        super().__init__(graph, x_init, x_goal, delta, k, path, heading)

    def cost(self, child):
        """
        Calculates the cost between nodes rootward until the root node is reached.
        """
        cost = 0
        while child != self.x_init:
            cost += self.graph._node[child]
            child = self.parent(child)
            if child is None:
                return float('inf')
        return cost

    def extend(self, x_new, x_nearest):
        """
        Expands the tree out into the frontier.
        """
        X_near = self.near(x_new, self.delta)
        self.graph.add_node(x_new, dist(x_nearest, x_new))
        self.find_parent(x_new, x_nearest, X_near)
        return X_near

    def find_parent(self, x_new, x_min, X_near):
        c_min = self.cost(x_min) + dist(x_min, x_new)
        for x_near in X_near:
            if self.cost(x_near) + dist(x_near, x_new) < c_min:
                x_min = x_near
                c_min = self.cost(x_near) + dist(x_near, x_new)
        self.graph.add_edge(x_min, x_new)
        self.graph._node[x_new] = dist(x_min, x_new)

    def rewire_neighbors(self, x_new, X_near):
        """
        Rewires the tree to nodes that are closer to the new node.
        """
        for x_near in X_near:
            if self.cost(x_new) + dist(x_new, x_near) < self.cost(x_near):
                x_parent = self.parent(x_near)
                self.graph.remove_edge(x_parent, x_near)
                self.graph.add_edge(x_new, x_near)
                self.graph._node[x_near] = dist(x_new, x_near)

    def connect_to_goal(self, v):
        """
        Adds goal to graph.
        """
        if self.x_goal in self.graph.nodes():
            return
        if dist(self.x_goal, v) < self.delta:
            self.graph.add_node(self.x_goal, dist(v, self.x_goal))
            self.graph.add_edge(v, self.x_goal)

    def search(self):
        """
        Function to search through the graph.
        """
        self.graph.add_node(self.x_init, 0)
        while self.graph.num_nodes() <= 300:
            r = self.shrinking_ball_radius()
            print(r)
            x_rand = self.sample_free()
            print(x_rand)
            x_nearest = self.nearest(x_rand, r)
            print(x_nearest)
            x_new = self.steer(x_nearest, x_rand)

            X_near = self.extend(x_new, x_nearest)
            self.rewire_neighbors(x_new, X_near)

            self.connect_to_goal(x_new)
        return self.compute_trajectory()


class MiniRRT(BaseRRT):
    """
    Class for optimal imcrementail RRT.
    """
    def __init__(self, graph, x_init, x_goal, delta, k, path, heading):
        super().__init__(graph, x_init, x_goal, delta, k, path, heading)

    def cost(self, child):
        """
        Calculates the cost between nodes rootward until the root node is reached.
        """
        cost = 0
        while child != self.x_init:
            parent = self.parent(child)
            if parent is None:
                return float('inf')
            cost += dist(parent, child)
            child = parent
        return cost

    def extend(self, x_new, x_nearest):
        """
        Expands the tree out into the frontier.
        """
        X_near = self.near(x_new, self.delta)
        self.graph.add_node(x_new)
        self.find_parent(x_new, x_nearest, X_near)
        return X_near

    def find_parent(self, x_new, x_min, X_near):
        c_min = self.cost(x_min) + dist(x_min, x_new)
        for x_near in X_near:
            if self.graph.collision_free(x_near, x_new) and self.cost(x_near) + dist(x_near, x_new) < c_min:
                x_min = x_near
                c_min = self.cost(x_near) + dist(x_near, x_new)
        self.graph.add_edge(x_min, x_new)

    def rewire_neighbors(self, x_new, X_near):
        """
        Rewires the tree to nodes that are closer to the new node.
        """
        for x_near in X_near:
            if self.graph.collision_free(x_new, x_near) and self.cost(x_new) + dist(x_new, x_near) < self.cost(x_near):
                x_parent = self.parent(x_near)
                self.graph.remove_edge(x_parent, x_near)
                self.graph.add_edge(x_new, x_near)

    def connect_to_goal(self, v):
        """
        Adds goal to graph.
        """
        if self.x_goal in self.graph.nodes():
            return
        if dist(self.x_goal, v) < self.delta:
            self.graph.add_node(self.x_goal)
            self.graph.add_edge(v, self.x_goal)

    def search(self):
        """
        Function to search through the graph.
        """
        self.graph.add_node(self.x_init)
        while self.graph.num_nodes() <= 300:
            r = self.shrinking_ball_radius()
            x_rand = self.sample_free()
            x_nearest = self.nearest(x_rand, r)
            x_new = self.steer(x_nearest, x_rand)
            if self.graph.collision_free(x_nearest, x_new):
                X_near = self.extend(x_new, x_nearest)
            if x_new in self.graph.nodes():
                self.rewire_neighbors(x_new, X_near)
            self.connect_to_goal(x_new)
        return self.compute_trajectory()
