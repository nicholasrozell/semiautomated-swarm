from shapely.geometry import LineString, Point


class Graphs:
    """
    Class for directed graphs.
    """

    def __init__(self, span, obstacles=None):
        self.graph_attr_dict_factory = dict
        self.adjlist_outer_dict_factory = dict
        self.adjlist_inner_dict_factory = dict
        self.edge_attr_dict_factory = dict

        self.graph = self.graph_attr_dict_factory()
        self.span = span
        self.dimensions = len(span)

        if obstacles is None:
            self.obstacles = list()
        else:
            self.obstacles = obstacles
        self.num_obstacles = self.obstacles

        self.nodes = set()
        self.edges = set()
        self._adj = self.adjlist_outer_dict_factory()
        self._pred = self.adjlist_outer_dict_factory()
        self._succ = self._adj

        self.agents = []
        self.i = 0

    def __contains__(self, v):
        """
        Returns True if v is a node, False otherwise. Use: 'v in G'
        """
        try:
            return v in self.nodes
        except NameError:
            return False

    def add_node(self, v):
        """
        Adds node v to the graph.
        """
        if v not in self._succ:
            self._succ[v] = self.adjlist_inner_dict_factory()
            self._pred[v] = self.adjlist_inner_dict_factory()
            self.nodes.add(v)

    def remove_node(self, v):
        """
        Removes node v from the graph.
        """
        try:
            nbrs = self._succ[v]
            self.nodes.remove(v)
        except NameError:
            raise NameError("The node {v} is not in the graph.".format(v=v))
        for u in nbrs:
            del self._pred[u][v]
        del self._succ[v]
        for u in self._pred[v]:
            del self._succ[u][v]
        del self._pred[v]

    def num_nodes(self):
        """
        Returns the total number of nodes.
        """
        return len(self.nodes)

    def add_edge(self, u, v):
        """
        Adds edge (u)-(v) to the graph.
        """
        if (u, v) not in self.edges:
            self.edges.add((u, v))
        if u not in self.nodes:
            self._succ[u] = self.adjlist_inner_dict_factory()
            self._pred[u] = self.adjlist_inner_dict_factory()
        if v not in self.nodes:
            self._succ[v] = self.adjlist_inner_dict_factory()
            self._pred[v] = self.adjlist_inner_dict_factory()
        datadict = self._adj[u].get(v, self.edge_attr_dict_factory())
        self._succ[u][v] = datadict
        self._pred[v][u] = datadict

    def remove_edge(self, u, v):
        """
        Removes edge (u)-(v) from the graph.
        """
        try:
            del self._succ[u][v]
            del self._pred[v][u]
            self.edges.remove((u, v))
        except NameError:
            raise NameError("The edge {u}-{v} is not in the graph.".format(u=u, v=v))

    def neighbors(self, v):
        """
        Returns the neighbors of node v.
        """
        try:
            return iter(self._adj[v])
        except KeyError:
            raise KeyError("The node {v} is not in the graph.".format(v=v))

    def successors(self, v):
        """
        Returns the neighbors of node v.
        """
        try:
            return iter(self._succ[v])
        except KeyError:
            raise KeyError("The node {v} is not in the graph.".format(v=v))

    def predecessors(self, v):
        """
        Returns the preddecessor of node v.
        """
        try:
            return iter(self._pred[v])
        except KeyError:
            raise KeyError("The node {v} is not in the graph.".format(v=v))

    def size(self):
        """
        Returns the number of edges.
        """
        s = sum(d for v, d in self.degree)
        return s // 2

    def num_edges(self, u=None, v=None):
        """
        Returns the total number of edges.
        """
        if u is None:
            return int(self.size())
        if v in self._adj[u]:
            return 1
        return 0

    def adjacency(self):
        """
        Returns an iterator over (node, adjacency dict) tuples for all nodes.
        """
        return iter(self._adj.items())

    def clear(self):
        """
        Removes all nodes and edges from the graph.
        """
        self._succ.clear()
        self._pred.clear()
        self.nodes.clear()
        self.edges.clear()
        self.graph.clear()

    def obstacle_free(self, v):
        """
        Checks if the node v is free an obstacle.
        """
        node = Point(v)
        return not any(node.within(self.obstacles) for self.obstacles in self.num_obstacles)

    def collision_free(self, v, u):
        """
        Checks if the edge between v and u is free of an obstacle.
        """
        edge = LineString([v, u])
        return not any(edge.intersects(self.obstacles) for self.obstacles in self.num_obstacles)

    def add_obstacle(self, obstacles):
        """Adds an obstacle to the graph."""
        for obstacle in obstacles:
            self.obstacles.append(obstacle)

    def remove_obstacle(self, obstacles):
        """Removes an obstacle from the graph."""
        for obstacle in obstacles:
            self.obstacles.remove(obstacle)
