import numpy as np
from .utils import dist, angle, bspline
from shapely.geometry.point import Point
from pathplanning.views3 import draw


#! RRTStarV2 is the newest search algorithm, use that
class BaseRRT:
    """
    Base class for rapidly-exploring random trees.

    graph: Graph, object
    x_init: location of root, tuple
    x_goal: location of goal, tuple
    delta: distance between nodes or edge lengths, int
    k: factor for shrinking ball radius, flaot
    alpha: angle to confine the random nodes, numpy radian
    obstacles: areas of high cost, shapley.polygon
    ppath: previous path, list of tuples
    """
    def __init__(self, graph, x_init, x_goal, delta, k, obstacles, ppath):
        self.graph = graph
        self.x_init = x_init
        self.x_goal = x_goal
        self.delta = delta
        self.k = k

        self.alpha = np.radians(120)  # angle for a 120 cone for local random sample
        self.obstacles = obstacles
        self.ppath = ppath

    def sample_free(self):
        """
        Returns a random node from within the graph.
        """
        return tuple(np.random.uniform(self.graph.span[:, 0], self.graph.span[:, 1], self.graph.span[:, 2]))

    def local_sample_free(self):
        r = self.delta*4 * np.sqrt(np.random.uniform())  # radius for which a node can appear
        if self.ppath == []:
            theta = np.random.uniform() * self.alpha + (np.radians(0) - self.alpha/2)  # initial tree sample free, bounds to angle
        else:
            theta = np.random.uniform() * self.alpha + (angle(self.ppath[2], self.ppath[3]) - self.alpha/2)  # trees after sample free, bounds to angle
        return tuple((self.x_init[0] + r * np.cos(theta), self.x_init[1] + r * np.sin(theta), self.graph.span[2][0]))  # random tuple
    
    def nearest(self, v, r):
        """
        Finds the nerest neighbor to the node 'v' within a ball radius 'r'.

        v: node in graph, tuple
        r: shirnking ball radius, float
        """
        min_dist = float('inf')  # start minimum cost at infinity
        nodes = self.near(v, r)  # finds nodes with the radius r
        for node in nodes:
            if dist(v, node) <= min_dist:  # checks distance between node 'v' and node and compares to the minimum distance
                min_dist = dist(v, node)  # replaces miniimum distance to distance between node 'v' and node
                nearest = node  # nearest node is set to node from within radius r
        try:
            return nearest
        except UnboundLocalError:  # if no nodes appear withing radius error is returned, r is not big enough
            raise UnboundLocalError("Could not find a nearest node, check the k factor.")  # enlarge k factor by increments of 0.5

    def near(self, pivot, radius):
        """
        Checks if nearby nodes are within a radius.

        pivot: node, center of circle. tuple
        radius: radius of circle, float
        """
        circle_x, circle_y, circle_z = pivot
        nodes = []
        for node in self.graph._node:
            # if node == pivot:
            #     continue
            x, y, z = node
            if ((x - circle_x) * (x - circle_x) + (y - circle_y) * (y - circle_y)) <= radius * radius: # checks if node is within circle radius
                nodes.append((x, y, z))
        return nodes

    def brute_force(self, x, bunch):
        """
        Finds the nearest node to x from bunch.

        x: node, tuple
        bunch: list of nodes, list of tuples
        """
        # works like nearest except finds nearest node to x by a list of tuples inputted
        # if list is empty error recived
        min_dist = float('inf')
        for node in bunch:
            if dist(x, node) <= min_dist:
                min_dist = dist(x, node)
                nearest = node
        try:
            return nearest
        except:
            raise ('bunch list is empty')

    def steer(self, x_nearest, x_rand):
        """
        Guides edge to nearest node.

        x_nearest: nearest node to x_rand, tuple
        x_rand: random node in graph, tuple
        """
        if dist(x_nearest, x_rand) > self.delta:  # checks if distance between nodes is greater then the user defined delta value
            return self.bound_point(self.saturate(x_nearest, x_rand, self.delta))  # saturates and bounds within graph the distance between nodes
        return self.bound_point(x_rand)

    def bound_point(self, node):
        """
        Bounds a node to within the graph.

        node: node, tuple
        """
        node = np.maximum(node, self.graph.span[:, 0])
        node = np.minimum(node, self.graph.span[:, 1])
        return tuple(node)

    def saturate(self, v, w, delta):
        """
        Shortens the distance between nodes 'v' and 'w'.

        v: node, tuple
        w: node, tuple
        delta: distance set by user, int
        """
        start, end = np.array(v), np.array(w)  # change nodes to numpy arrays
        z = end - start
        u = z / np.sqrt((np.sum(z**2)))  # calc unit vector
        saturated_node = start + u*delta
        return tuple(saturated_node)

    def shrinking_ball_radius(self):
        """
        Radius used to find nearest nodes. Shrinks as the number of nodes increases.

        Otte and Frazzoli, and Karaman and LaValle define shrinkging ball radius differently from each other
        This method is from Otte and Frazzoli
        """
        d = self.graph.dimensions  # dimensions of the graph
        leb_meas = (self.graph.span[0][1] - self.graph.span[0][0]) * (self.graph.span[1][1] - self.graph.span[1][0])  # calculates the lebesque measure, https://en.wikipedia.org/wiki/Lebesgue_measure
        zeta_D = (4.0/3.0) * self.delta**3  # volume of unit ball
        gamma = (2**d*(1 + (1/d))*leb_meas)**self.k  # gamma variable
        r = int(((gamma/zeta_D)*(np.log10(self.graph.num_nodes())/self.graph.num_nodes())))**(1/d)  # radius
        if r == 0.0:
            r = self.graph.span[0][1] + self.graph.span[1][1]
        return r

    def parent(self, v):
        """
        Returns the parent of node 'v'.

        v: node, tuple
        """
        if list(self.graph.predecessor(v)) == []:
            return None
        return list(self.graph.predecessor(v))[0]

    def children(self, v):
        """
        Returns the parent of node 'v'.

        v: node, tuple
        """
        return list(self.graph.successors(v))

    def is_orphan(self, v):
        """
        Checks if the node 'v' is and orphan. A node with no parent.

        v: node, tuple
        """
        if self.parent(v) == None:
            return True
        return False

    def is_leaf(self, v):
        """
        "Checks if node 'v' is a leaf. A node with no children.

        v: node, tuple
        """
        if self.graph.degree(v) == 0:
            return True
        return False

    def depth(self, child):
        """
        Checks the depth of the node 'child'.

        child: node, tuple
        """
        node_depth = 0  # set initial depth
        while self.parent(child) != self.x_init:  # tracks toward the root through parents
            node_depth += 1
            child = self.parent(child)
        return node_depth

    def best_leaf(self, leaves):
        """
        Finds the closest leaf to the goal.

        leaves: list of leaf nodes, list of tuples
        """
        nodes = []
        # temp_list = []
        for leaf in leaves:
            theta = angle(self.parent(leaf), leaf)  # finds angle between leaf node and parent
            temp = tuple((leaf[0] + (self.delta*5) * np.cos(theta), leaf[1] + (self.delta*5) * np.sin(theta), self.graph.span[2][0]))  # creates trajectory line delta away form leaf along angle theta
            # temp = self.saturate(leaf, self.x_goal, self.delta*5)
            if self.graph.collision_free(leaf, temp) and self.depth(leaf) >= 4: # checks if the trajectory line is interescting an obstacle and is a defined depth
                nodes.append(leaf)
                # temp_list.append(temp)
        if nodes != []:
            return self.brute_force(self.x_goal, nodes)  # finds closest node the goal

    def compute_trajectory(self):
        """
        Overall function for creating paths.
        """
        leaves = []
        for n in self.graph._node:  # gathers all the leaf nodes in the tree
            if self.is_leaf(n) and not self.is_orphan(n):
                leaves.append(n)
        leaf = self.best_leaf(leaves)  # finds the closest leaf node to the goal
        return self.construct_path(self.x_init, leaf)  # creates a path from the root to the leaf

    def construct_path(self, start, end):
        """
        Constructs a path between the start and end nodes, starting from the end to start.

        start: node, tuple
        end: node, tuple
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
        # return bspline(path, n=10)

    def smooth_path(self, path):
        """
        Does a polynomial fit on the path taken from the tree. 
        
        Not used...
        """
        array = np.array(path)
        x = array[:, 0]
        y = array[:, 1]
        f = np.polyfit(x, y, 3)
        p = np.poly1d(f)
        xp = np.linspace(x[0], x[-1], len(path))
        smoothed_path = []
        for i in range(len(xp)):
            smoothed_path.append((xp[i], p(xp[i]), path[0][2]))
        return smoothed_path

    def cost(self, child):
        """
        Calculates the cost (length) between the child node and the parent node.

        child: node, tuple
        """
        cost = 0
        while child != self.x_init:
            parent = self.parent(child)
            if parent == None:
                return float('inf')
            cost += dist(parent, child)
            child = parent
        return cost


class RRT(BaseRRT):
    """
    Class for the basic RRT algorithm.

    ref1: Randomized Kinodynamic Planning;
    Lavelle, Kuffner
    ref2: Sampling-based Algorithms for Optimal Motion Planning; 
    Karaman, Frazzoli    
    """
    def __init__(self, graph, x_init, x_goal, delta, k, ppath):
        super().__init__(graph, x_init, x_goal, delta, k, ppath)

    def search(self):
        self.graph.add_node(self.x_init)
        while self.graph.num_nodes() <= 500:
            x_rand = self.sample_free()
            x_nearest = self.nearest(x_rand, self.shrinking_ball_radius())
            x_new = self.steer(x_nearest, x_rand)
            if self.graph.collision_free(x_nearest, x_new):
                self.graph.add_node(x_new)
                self.graph.add_edge(x_nearest, x_new)
        return self.compute_trajectory()


class RRTStar(BaseRRT):
    """
    Class for the optimal RRT algorithm.
    
    ref: Sampling-based Algorithms for Optimal Motion Planning; 
    Karaman, Frazzoli
    """
    def __init__(self, graph, x_init, x_goal, delta, k, ppath):
        super().__init__(graph, x_init, x_goal, delta, k, ppath)

    def search(self):
        self.graph.add_node(self.x_init)
        count = 0
        while self.graph.num_nodes() <= 250:
            r = self.shrinking_ball_radius()
            x_rand = self.local_sample_free()
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
            count += 1
        return self.compute_trajectory()


class RRTStarV2(BaseRRT):
    """
    Class for replanning RRT algorithm.

    ref1: Sampling-based Algorithms for Optimal Motion Planning; 
    Karaman, Frazzoli
    ref2: RRTX: Asymptotically Optimal Single-Query Sampling-Based Motion
    Planning with Quick Replanning; 
    Otte, Frazzoli
    """
    def __init__(self, graph, x_init, x_goal, delta, k, obstacles, ppath):
        self.orphans = []
        super().__init__(graph, x_init, x_goal, delta, k, obstacles, ppath)

    def extend(self, x_new, x_nearest, r):
        """
        Function to grow tree.

        x: node, tuple
        x_nearest: node, tuple
        r: shrinking ball radius, int
        """
        X_near = self.near(x_new, self.delta)  # finds nodes within delta away from the new node
        self.graph.add_node(x_new)
        self.find_parent(x_new, x_nearest, X_near)  # finds parent for new node
        return X_near

    def find_parent(self, x_new, x_min, X_near):
        """
        Finds the best parent to node x_new.

        x_new: node, tuple
        x_min: node, tuple
        X_near: list of nodes, list of tuples
        """
        if self.graph.collision_free(x_min, x_new):
            c_min = self.cost(x_min) + dist(x_min, x_new)  # set minimum cost
            for x_near in X_near:
                if self.graph.collision_free(x_near, x_new) and self.cost(x_near) + dist(x_near, x_new) < c_min:
                # checks for collision and if the cost of a nearby node is less
                    x_min = x_near  # set new minimum node
                    c_min = self.cost(x_near) + dist(x_near, x_new)  # set new minimum cost
            self.graph.add_edge(x_min, x_new)

    def rewire_neighbors(self, x_new, X_near):
        """
        Rewires the tree by finding nodes that have less cost (distance) between them.

        x_new: node, tuple
        X_near: list of nodes, list of tuples
        """
        for x_near in X_near:
            if self.graph.collision_free(x_new, x_near) and self.cost(x_new) + dist(x_new, x_near) < self.cost(x_near):
            # checks for collision and if costs and less than a near node
                x_parent = self.parent(x_near)  # sets parent
                if x_parent == None:
                    continue
                self.graph.remove_edge(x_parent, x_near)  # removes edge with parent
                self.graph.add_edge(x_new, x_near)  # adds edge to new parent

    def propogate_descendents(self):
        """
        Removes edges from nodes that are inside and obstacle.
        """
        for n in self.graph._node:
            if self.graph.obstacle_free(n) == False:  # checks if the node is in an obstalce
                parent = self.parent(n)
                if parent != None:
                    self.graph.remove_edge(self.parent(n), n)  # removes edge between nodes

    def reduce_inconsistency(self):
        """
        Connects potential orphans to tree through leaf nodes.
        """
        if self.orphans == []:
            return
        leaves = []
        for n in self.graph._node:  # Grabs leafs of tree so far
            if self.is_leaf(n) and not self.is_orphan(n):
                leaves.append(n)
        for n in self.orphans:  # iterate through orphans
            if self.graph.obstacle_free(n):  # checks if orphan is not inside an obstalce
                nearest = self.brute_force(n, leaves)  # finds nearest leaf to orphan
                if self.graph.collision_free(nearest, n):  # checks if the potential edge has a collision
                    self.graph.add_edge(nearest, n)  # create edge
                    leaves.append(n)  # add orphan to leaves
                    self.orphans.remove(n)  # remove orphan from orphans list
        
        ## Other slower method
        # for leaf in leaves:
        #     if self.orphans != []:
        #         nearest = self.brute_force(leaf, self.orphans)
        #         if self.graph.collision_free(leaf, nearest):
        #             self.graph.add_edge(leaf, nearest)
        #             leaves.append(nearest)
        #             self.orphans.remove(nearest)

    def search(self):
        """
        Main function of algorithm.
        """
        self.graph.add_node(self.x_init)  # adds root node to graph
        count = 0
        while count <= 250:
            r = self.shrinking_ball_radius()  # finds radius r
            if self.obstacles != self.graph.obstacles:
                self.graph.update_obstacles(self.obstacles)  # Updates obstacles if a difference is seen
                self.propogate_descendents()  # removes edges from nodes that are insdie obstacles
            x_rand = self.local_sample_free()  # selects a random local node
            x_nearest = self.nearest(x_rand, r)  # finds nearest neighbor to node
            x_new = self.steer(x_nearest, x_rand)  # saturates node
            # if self.graph.obstacle_free(x_new):
            X_near = self.extend(x_new, x_nearest, r)  # extends tree to new nodes
            if self.is_orphan(x_new):  # checks if the noew node is an orphan
                self.orphans.append(x_new)
            if x_new in self.graph._node:  # checks if new node is in the graph
                self.rewire_neighbors(x_new, X_near)  # rewires nodes that have minimal costs
                self.reduce_inconsistency()  # craetes edges for potential orphans
            count += 1
            if count == 125:
                self.obstacles = [Point(10, 0).buffer(5.0)]
        return self.compute_trajectory(), self.obstacles  # calculate a path from generated tree to goal node
