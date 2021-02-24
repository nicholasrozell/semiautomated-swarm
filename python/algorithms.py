import numpy as np
from utils import dist, angle


class BaseRRT:
    """
    Base class for rapidly-exploring random trees.
    """
    def __init__(self, G, x_init, x_goal, delta, k, path, n, case):
        """
        G : graph
        x_init : initial node
        x_goal : goal node
        delta : distance between nodes, or length of an edge
        k : factor for shrinking_ball_radius
        path : point of waypoints to be pushed
        n : index value to cut paths
        case : number for cases
        alpah : angle in which to constrain RRT steering
        beta : angle in which the tree can grow
        length : overall distance a tree can grow
        """
        self.G = G
        self.x_init = x_init
        self.x_goal = x_goal
        self.delta = delta
        self.k = k
        
        self.path = []
        self.n = n
        self.case = case

        self.alpha = np.radians(15)
        self.beta = np.radians(135)
        self.length = 3

    def sample_free(self):
        """
        Selects a random node within the graph.
        """
        return tuple(np.random.uniform(self.G.span[:, 0], self.G.span[:, 1], self.G.span[:, 2]))

    def sample_free_local(self):
        """
        Selects a random node within the defined area.
        """
        r = self.delta * self.length * np.sqrt(np.random.uniform())
        theta = np.random.uniform() * self.beta + (angle(self.x_init, self.x_goal) - self.beta/2)
        return tuple((self.x_init[0] + r * np.cos(theta), self.x_init[1] + r * np.sin(theta), self.G.span[2][0]))

    def nearest(self, v, r):
        """
        Finds the nearest neighbor to the node {v} within a ball radius.
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
            raise UnboundLocalError('Could not find a nearest node, due to k not being large enough value.')

    def near(self, pivot, r):
        """
        Checks if nearby nodes are within a radius.
        """
        circle_x, circle_y, circle_z = pivot
        points = []
        for node in self.G.nodes:
            x, y, z = node
            if ((x - circle_x) * (x - circle_x) + (y - circle_y) * (y - circle_y)) <= r * r:
                points.append((x, y, z))
        return points

    def brute_force(self, x):
        """
        Finds the nearest neighbor to the node {v} within the graph.
        """
        min_dist = float('inf')
        nodes = self.G.nodes
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

    def steer_constrained(self, x_nearest, x_rand):
        """
        Guides edge to nearest node, while also constraining the angle at
        which the random node is placed.
        """
        if dist(x_nearest, x_rand) > self.delta:
            return self.bound_point(self.saturate(x_nearest, self.constrain(x_nearest, x_rand)))
        else:
            return self.bound_point(self.constrain(x_nearest, x_rand))

    def bound_point(self, node):
        """
        Bounds a point to within the graphs span.
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
        saturated_point = start + u * self.delta
        return tuple(saturated_point)

    def constrain(self, start, end):
        """
        Constrains the angle at which the edge connects start adn end.
        """
        if tuple(self.G.predecessors(start)) == ():
            return end
        fixed_angle = angle(list(self.G.predecessors(start))[0], start)
        upper_angle = fixed_angle + self.alpha
        lower_angle = fixed_angle - self.alpha
        constrained_angle = angle(start, end)
        if constrained_angle > upper_angle:
            x_prime = ((end[0] - start[0]) * np.cos(upper_angle - constrained_angle) - (end[1] - start[1]) * np.sin(upper_angle - constrained_angle)) + start[0]
            y_prime = ((end[0] - start[0]) * np.sin(upper_angle - constrained_angle) + (end[1] - start[1]) * np.cos(upper_angle - constrained_angle)) + start[1]
            z_prime = self.G.span[2][0]
            end = (x_prime, y_prime, z_prime)
        elif constrained_angle < lower_angle:
            x_prime = ((end[0] - start[0]) * np.cos(lower_angle - constrained_angle) - (end[1] - start[1]) * np.sin(lower_angle - constrained_angle)) + start[0]
            y_prime = ((end[0] - start[0]) * np.sin(lower_angle - constrained_angle) + (end[1] - start[1]) * np.cos(lower_angle - constrained_angle)) + start[1]
            z_prime = self.G.span[2][0]
            end = (x_prime, y_prime, z_prime)
        return end

    def connect_to_goal(self, goal, v):
        if goal in self.G.nodes:
            return
        elif dist(goal, v) < self.delta:
            self.G.add_node(goal)
            self.G.add_edge(v, goal)

    def construct_path(self, start, end):
        """
        Constructs a path between start and end.
        """
        path = [end]
        current = end
        if start == end:
            return path
        while start not in path:
            path.append(list(self.G.predecessors(current))[0])
            current = list(self.G.predecessors(current))[0]
        path.reverse()
        return path

    def retain_path(self, start, end):
        n = self.n
        print('\ncase: ', self.case)
        # print('init: ', self.x_init, '\n')
        # print('init path: ', self.path, '\n')
        if self.case == 0:
            self.path = self.construct_path(start, end)
            self.case = 1
        elif self.case == 1:
            del self.path[n:]
            segment = self.construct_path(start, end)
            self.path = self.path + segment
            self.case = 2
        elif self.case == 2:
            del self.path[:n]
            del self.path[n:]
            segment = self.construct_path(start, end)
            self.path = self.path + segment
            if dist(self.x_init, self.x_goal) < self.delta*n:
                self.case = 3
        elif self.case == 3:
            del self.path[:n]
            segment = self.construct_path(start, end)
            self.path = self.path + segment
            self.case = 2
        return self.path, self.case

    def shrinking_ball_radius(self):
        """docstring"""
        if self.G.num_nodes() == 1:
            return self.G.span[0][1] + self.G.span[1][1]
        d = self.G.dimensions
        leb_meas = (self.G.span[0][1] - self.G.span[0][0]) * (self.G.span[1][1] - self.G.span[1][0])  # <- estimated in practice
        zeta_D = (4.0/3.0) * np.pi * self.delta**3
        gamma = (2**d*(1 + (1/d)) * leb_meas)**self.k
        return max(int(((gamma / zeta_D) * (np.log10(self.G.num_nodes())/self.G.num_nodes())) ** (1 / d)), self.delta)


class RRT(BaseRRT):
    """
    Class for the basic RRT algorithm.

    ref: Randomized Kindynamic Planning;
    Lavalle, Kuffner
    """
    def __init__(self, G, x_init, x_goal, delta, k, path, n, case):
        super().__init__(G, x_init, x_goal, delta, k, path, n, case)

    def search(self):
        self.G.add_node(self.x_init)
        count = 0
        while count <= 400:
            x_rand = self.sample_free_local()
            x_nearest = self.nearest(x_rand, self.shrinking_ball_radius())
            x_new = self.steer(x_nearest, x_rand)
            if self.G.obstacle_free(x_new):
                self.G.add_node(x_new)
                self.G.add_edge(x_nearest, x_new)
            count += 1
        return self.retain_path(self.x_init, self.brute_force(self.x_goal))


class ConstrainedRRT(BaseRRT):
    """
    Class for the basic RRT algorithm.

    ref: Randomized Kindynamic Planning;
    Lavalle, Kuffner
    """
    def __init__(self, G, x_init, x_goal, delta, k, path, n, case):
        super().__init__(G, x_init, x_goal, delta, k, path, n, case)

    def search(self):
        self.G.add_node(self.x_init)
        count = 0
        while count <= 400:
            x_rand = self.sample_free_local()
            x_nearest = self.nearest(x_rand, self.shrinking_ball_radius())
            x_new = self.steer_constrained(x_nearest, x_rand)
            if self.G.obstacle_free(x_new):
                self.G.add_node(x_new)
                self.G.add_edge(x_nearest, x_new)
            count += 1
        return self.retain_path(self.x_init, self.brute_force(self.x_goal))


class RRTStar(BaseRRT):
    """
    Class for the optimal RRT algorithm.

    ref: Sampling-based Algorithms for Optimal Motion Planning;
    Karaman, Frazzoli
    """
    def __init__(self, G, x_init, x_goal, delta, k, path, n, case):
        super().__init__(G, x_init, x_goal, delta, k, path, n, case)

    def parent(self, v):
        """
        Returns the parent of node v.
        """
        return list(self.G.predecessors(v))[0]

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

    def search(self):
        self.G.add_node(self.x_init)
        count = 0
        while count <= 400:
            r = self.shrinking_ball_radius()
            x_rand = self.sample_free_local()
            x_nearest = self.nearest(x_rand, r)
            x_new = self.steer(x_nearest, x_rand)
            if self.G.obstacle_free(x_new):
                X_near = self.near(x_new, self.delta)
                self.G.add_node(x_new)
                x_min = x_nearest
                c_min = self.cost(x_nearest) + dist(x_nearest, x_new)
                for x_near in X_near:
                    if self.G.collision_free(x_near, x_new) and self.cost(x_near) + dist(x_new, x_near) < c_min:
                        x_min = x_near
                        c_min = self.cost(x_near) + dist(x_near, x_new)
                self.G.add_edge(x_min, x_new)
                for x_near in X_near:
                    if self.G.collision_free(x_new, x_near) and self.cost(x_new) + dist(x_new, x_near) < self.cost(x_near):
                        x_parent = self.parent(x_near)
                        self.G.remove_edge(x_parent, x_near)
                        self.G.add_edge(x_new, x_near)
            count += 1
        return self.retain_path(self.x_init, self.brute_force(self.x_goal))
