import numpy as np
from utils import dist, angle3D


class BaseRRT:
    """
    Base class for rapidly-exploring random trees.
    """
    def __init__(self, G, x_init, x_goal, delta, k, case, path, n):
        self.G = G
        self.x_init = x_init
        self.x_goal = x_goal
        self.delta = delta
        self.k = k

        self.beta = np.radians(135)
        self.length = 4
        self.case = case
        self.path = path
        self.n = n

    def sample_free(self):
        """
        Selects a random node within the graph.
        """
        return tuple(np.random.uniform(self.G.span[:, 0], self.G.span[:, 1], self.G.span[:, 2]))

    def sample_free_local(self):
        """
        dcostring
        """
        r = self.delta * self.length * np.sqrt(np.random.uniform())
        if self.case == 0:
            theta = np.random.uniform() * self.beta + (np.radians(0) - self.beta/2)
        if self.case != 0:
            theta = np.random.uniform() * self.beta + (angle3D(self.path[3], self.path[4]) - self.beta/2)
        return tuple((self.x_init[0] + r * np.cos(theta), self.x_init[1] + r * np.sin(theta), self.G.span[2][0]))  

    def nearest(self, v , r):
        """
        Finds the nrearest neighbor to the node v within a ball radius.
        """
        min_dist = float('inf')
        nodes = self.near(v, r)
        for node in nodes:
            if dist(v, node) <= min_dist:
                min_dist = dist(v, node)
                nearest_node = node
        try:
            return nearest_node
        except:
            raise UnboundLocalError('Could not find a nearest node, due to k not being a large enough int value.')

    def near(self, pivot, radius):
        """
        Checks if nearby nodes are within a radius.
        """
        circle_x, circle_y, circle_z = pivot
        points = []
        for node in self.G._node:
            x, y, z = node
            if ((x - circle_x) * (x - circle_x) + (y - circle_y) * (y - circle_y)) <= radius * radius:
                points.append((x, y, z))
        return points

    def brute_force(self, x):
        """
        docstring
        """
        min_dist = float('inf')
        nodes = self.G._node
        for node in nodes:
            if dist(x, node) <= min_dist:
                min_dist = dist(x, node)
                nearest = node
        return nearest

    def steer(self, x_nearest, x_rand):
        """
        Guides edge to nearest node.
        """
        if dist(x_nearest, x_rand) > self.delta:
            return self.bound_point(self.saturate(x_nearest, x_rand))
        else:
            return self.bound_point(x_rand)
    
    def bound_point(self, node):
        """
        Bounds a point to within the graph.
        """
        node = np.maximum(node, self.G.span[:, 0])
        node = np.minimum(node, self.G.span[:, 1])
        return tuple(node)
    
    def saturate(self, start, end):
        """
        Saturates the distance between start and end.
        """
        start, end = np.array(start), np.array(end)
        z = end - start
        u = z / np.sqrt((np.sum(z**2)))
        saturated_point = start + u*self.delta
        return tuple(saturated_point)   

    def shrinking_ball_radius(self):
        """
        docstring
        """
        # // try and except here?
        if self.G.num_nodes() == 1:
            return self.G.span[0][1] + self.G.span[1][1]
        d = self.G.dimensions
        leb_meas = (self.G.span[0][1] - self.G.span[0][0])*(self.G.span[1][1] - self.G.span[1][0])
        zeta_D = (4.0/3.0) * np.pi * self.delta**3
        gamma = (2**d*(1 + (1/d))*leb_meas)**self.k
        return int(((gamma/zeta_D)*(np.log10(self.G.num_nodes())/self.G.num_nodes()))**(1/d))

    def construct_path(self, start, end):
        """
        Constructs a path between the start and end nodes.
        """
        path = [end]
        current = end
        if start == end:
            return path
        while start not in path:
            path.append(list(self.G.predecessor(current))[0])
            current = list(self.G.predecessor(current))[0]
        path.reverse()
        return path

    def compute_trajectory(self):
        """
        docstring
        """
        leaves = []
        for n in self.G._node:
            if self.is_leaf_node(n):
                leaves.append(n)
        goal = self.find_leaf(leaves)
        return self.find_path(goal)

    def find_path(self, goal):
        """
        docstring
        """
        if self.case == 0:
            self.path = self.construct_path(self.x_init, goal)
            self.case = 1
        elif self.case == 1:
            del self.path[self.n:]
            segment = self.construct_path(self.x_init, goal)
            self.path = self.path + segment
            self.case = 2
        elif self.case == 2:
            del self.path[:self.n]
            del self.path[self.n:]
            segment = self.construct_path(self.x_init, goal)
            self.path = self.path + segment
            if dist(self.x_init, self.x_goal) < self.delta * 2:
                self.case = 3
        elif self.case == 3:
            del self.path[:self.n]
            segment = self.construct_path(self.x_init, goal)
            self.path = self.path + segment
            self.case = 2
        return self.path, self.case

    def is_leaf_node(self, v):
        """
        docstring
        """
        if self.G.degree(v) == 0:
            return True
        else: False

    def find_leaf(self, leaves):
        """
        docstring
        """
        nodes = []
        for leaf in leaves:
            if self.G.collision_free(leaf, self.x_goal):
                nodes.append(leaf)
        if nodes != []:
            min_dist = float('inf')
            for node in nodes:
                if dist(self.x_goal, node) <= min_dist:
                    min_dist = dist(self.x_goal, node)
                    nearest = node
            return nearest
        else:
            # take only nodes on the far sides at full length???
            max_dist = 0
            for leaf in leaves:
                if dist(self.x_goal, leaf) > max_dist:
                    # max_dist = dist(self.x_goal, leaf)
                    farthest = leaf
            return farthest

    def depth(self, v):
        pass


class RRT(BaseRRT):
    """
    Class for the basic RRT algorithm.

    ref: Randomized Kinodynamic Planning;
    Lavelle, Kuffner
    """
    def __init__(self, G, x_init, x_goal, delta, k, case, path, n):
        super().__init__(G, x_init, x_goal, delta, k, case, path, n)

    def search(self):
        self.G.add_node(self.x_init)
        while self.G.num_nodes() <= 400:
            x_rand = self.sample_free_local()
            x_nearest = self.nearest(x_rand, self.shrinking_ball_radius())
            x_new = self.steer(x_nearest, x_rand)
            if self.G.obstacle_free(x_new):
                self.G.add_node(x_new)
                self.G.add_edge(x_nearest, x_new)


class ConstrainedRRT(BaseRRT):
    """
    Class for the constrained RRT algorithm.
    """
    def __init__(self, G, x_init, x_goal, delta, k, case, path, n):
        self.alpha = np.radians(15)
        super().__init__(G, x_init, x_goal, delta, k, case, path, n)

    def constrain(self, start, end):
        """
        Constrains the angle at which the edge connects start and end nodes.
        """
        if tuple(self.G.predecessor(start)) == ():
            return end
        fixed_angle = angle(list(self.G.predecessor(start))[0], start)
        upper_angle = fixed_angle + self.alpha
        lower_angle = fixed_angle - self.alpha
        constrained_angle = angle(start, end)
        if constrained_angle > upper_angle:
            x_prime = ((end[0] - start[0]) * np.cos(upper_angle - constrained_angle) - (end[1] - start[1]) * np.sin(upper_angle - constrained_angle)) + start[0]
            y_prime = ((end[0] - start[0]) * np.sin(upper_angle - constrained_angle) + (end[1] - start[1]) * np.cos(upper_angle - constrained_angle)) + start[1]
            end = (x_prime, y_prime)
        elif constrained_angle < lower_angle:
            x_prime = ((end[0] - start[0]) * np.cos(lower_angle - constrained_angle) - (end[1] - start[1]) * np.sin(lower_angle - constrained_angle)) + start[0]
            y_prime = ((end[0] - start[0]) * np.sin(lower_angle - constrained_angle) + (end[1] - start[1]) * np.cos(lower_angle - constrained_angle)) + start[1]
            end = (x_prime, y_prime)
        return end        

    def constrained_steer(self, x_nearest, x_rand):
        """
        docstring
        """
        if dist(x_nearest, x_rand) > self.delta:
            return self.bound_point(self.saturate(x_nearest, self.constrain(x_nearest, x_rand)))
        else:
            return self.bound_point(self.constrain(x_nearest, x_rand))

    def search(self):
        self.G.add_node(self.x_init)
        while self.G.num_nodes() <= 400:
            x_rand = self.sample_free_local()
            x_nearest = self.nearest(x_rand, self.shrinking_ball_radius())
            x_new = self.constrained_steer(x_nearest, x_rand)
            if self.G.obstacle_free(x_new):
                self.G.add_node(x_new)
                self.G.add_edge(x_nearest, x_new)


class RRTStar(BaseRRT):
    """
    Class for the optimal RRT algorithm.
    
    ref: Sampling-based Algorithms for Optimal Motion Planning; 
    Karaman, Frazzoli
    """
    def __init__(self, G, x_init, x_goal, delta, k, case, path, n):
        super().__init__(G, x_init, x_goal, delta, k, case, path, n)
    
    def parent(self, v):
        """
        Returns the parent of node v.
        """
        return list(self.G.predecessor(v))[0]

    def cost(self, child):
        """
        Calculates the cost between the node 'child' and the root node.
        """
        cost = 0
        while child != self.x_init:
            parent = self.parent(child)
            cost += dist(parent, child)
            child = parent
        return cost

    def find_parent(self, x_new, x_nearest, X_near):
        """
        docstring
        """
        x_min = x_nearest
        c_min = self.cost(x_nearest) + dist(x_nearest, x_new)
        for x_near in X_near:
            if self.G.collision_free(x_near, x_new) and self.cost(x_near) + dist(x_new, x_near) < c_min:
                x_min = x_near
                c_min = self.cost(x_near) + dist(x_near, x_new)
        self.G.add_edge(x_min, x_new)

    def rewire_neighbors(self, X_near, x_new):
        """
        docstring
        """
        for x_near in X_near:
            if self.G.collision_free(x_new, x_near) and self.cost(x_new) + dist(x_new, x_near) < self.cost(x_near):
                x_parent = self.parent(x_near)
                self.G.remove_edge(x_parent, x_near)
                self.G.add_edge(x_new, x_near)

    def search(self):
        self.G.add_node(self.x_init)
        node_count = 0
        while node_count <= 350:
            r = self.shrinking_ball_radius()
            x_rand = self.sample_free_local()
            x_nearest = self.nearest(x_rand, r)
            x_new = self.steer(x_nearest, x_rand)
            if self.G.collision_free(x_new, x_nearest):
                X_near = self.near(x_new, self.delta)
                self.G.add_node(x_new)
                self.find_parent(x_new, x_nearest, X_near)
                self.rewire_neighbors(X_near, x_new)

            # if self.G.collision_free(x_new, x_nearest):
            #     X_near = self.near(x_new, self.delta)
            #     self.G.add_node(x_new)
            #     x_min = x_nearest
            #     c_min = self.cost(x_nearest) + dist(x_nearest, x_new)
            #     for x_near in X_near:
            #         if self.G.collision_free(x_near, x_new) and self.cost(x_near) + dist(x_new, x_near) < c_min:
            #             x_min = x_near
            #             c_min = self.cost(x_near) + dist(x_near, x_new)
            #     self.G.add_edge(x_min, x_new)
            #     for x_near in X_near:
            #         if self.G.collision_free(x_new, x_near) and self.cost(x_new) + dist(x_new, x_near) < self.cost(x_near):
            #             x_parent = self.parent(x_near)
            #             self.G.remove_edge(x_parent, x_near)
            #             self.G.add_edge(x_new, x_near)
            node_count += 1
        return self.compute_trajectory()
        # return self.compute_trajectory(self.x_init, self.brute_force(self.x_goal))  # old method


class RRTX(BaseRRT):
    """
    Class for the replanning RRT algorithm.
    
    ref: RRTX: Asymptotically Optimal Single-Query Sampling-Based Motion
    Planning with Quick Replanning; 
    Otte, Frazzoli
    """
    def __init__(self, G, x_init, x_goal, delta, k, case, path, n):
        super().__init__(G, x_init, x_goal, delta, k, case, path, n)
    
    def parent(self, v):
        """
        Returns the parent of node v.
        """
        return list(self.G.predecessor(v))[0]

    def cost(self, child):
        """
        Calculates the cost between the node 'child' and the root node.
        """
        cost = 0
        while child != self.x_init:
            parent = self.parent(child)
            cost += dist(parent, child)
            child = parent
        return cost

    def lmc(self, x):
        """
        docstring
        """
        nbrs = list(self.G.successors(x))
        if nbrs == []:
            return 0
        best = 99999
        for n in nbrs:
            if dist(n, x) < best:
                nbr = n
        return dist(nbr, x) + self.cost(nbr)

    def make_parent(self, v, u):
        """
        docstring
        """
        self.G.remove_edge(self.parent(u), u)
        self.G.add_edge(v, u)

    def find_parent(self, v, U, r):
        """
        docstring
        """
        parent = self.x_nearest
        for u in U:
            if dist(v, u) <= r and self.lmc(v) > dist(v, u) + self.lmc(u) and self.G.collision_free(v, u):
                parent = u
        self.G.add_edge(parent, v)

    def extend(self, x, r):
        """
        docstring
        """
        X_near = self.near(x, self.delta)
        self.G.add_node(x)
        self.find_parent(x, X_near, r)
        return X_near

    def rewire_neighbors(self, v, X_near):
        """
        docstring
        """
        for u in set(X_near) - set(self.parent(v)):
            if self.cost(u) > dist(u, v) + self.cost(v):
                self.make_parent(v, u)

    def search(self):
        """
        docstring
        """
        self.G.add_node(self.x_init)
        while self.G.num_nodes() <= 400:
            r = self.shrinking_ball_radius()
            x_rand = self.sample_free_local()
            self.x_nearest = self.nearest(x_rand, r)
            x_new = self.steer(self.x_nearest, x_rand)
            if self.G.obstacle_free(x_new):
                X_near = self.extend(x_new, r)
            if x_new in self.G._node:
                self.rewire_neighbors(x_new, X_near)
