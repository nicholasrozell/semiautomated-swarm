from shapely.geometry import LineString, Point


class Graph:
    """
    Class for directed graphs.
    """
    def __init__(self, span, obstacles=[]):
        self.graph_attr_dict_factory = dict
        self.adjlist_outer_dict_factory = dict
        self.adjlist_inner_dict_factory = dict
        self.edge_attr_dict_factory = dict

        self.graph = self.graph_attr_dict_factory()
        self.span = span
        self.dimensions = len(span)

        self.obstacles = obstacles

        self._node = set()
        self._edge = set()
        self._adj = self.adjlist_inner_dict_factory()
        self._pred = self.adjlist_outer_dict_factory()
        self._succ = self._adj

    def __contains__(self, v):
        """
        Returns True if 'v' is a node, False otherwise. Use: 'v in G'.
        """
        try:
            return v in self._node
        except NameError:
            return False

    def add_node(self, v):
        """
        Adds node (v) to the graph.
        """
        if v not in self._succ:
            self._succ[v] = self.adjlist_outer_dict_factory()
            self._pred[v] = self.adjlist_inner_dict_factory()
            self._node.add(v)

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
        return len(self._node)

    def add_edge(self, u, v):
        """
        Adds edge (u)-(v) to the graph.
        """
        if (u, v) not in self._edge:
            self._edge.add((u, v))
        if u not in self._node:
            self._succ[u] = self.adjlist_inner_dict_factory()
            self._pred[u] = self.adjlist_inner_dict_factory()
        if v not in self._node:
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
            self._edge.remove((u, v))
        except NameError:
            raise NameError("The edge {u}-{v} is not in the graph.".format(u=u, v=v))
    
    def size(self):
        """
        Returns the numbers of edges.
        """
        s = sum(d for v, d in self.degree)
        return s // 2

    def num_edges(self):
        """
        Returns the total number of edges.
        """
        if u is None:
            return int(self.size())
        if v in self._adj[u]:
            return 1
        return 0

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
        Returns the successors of node v.
        """
        try:
            return iter(self._succ[v])
        except KeyError:
            raise KeyError("The node {v} is not in the graph.".foramt(v=v))

    def predecessor(self, v):
        """
        Returns the predecessor of node v.
        """
        try:
            return iter(self._pred[v])
        except KeyError:
            raise KeyError("The node {v} is not in the graph.".format(v=v))

    def adjacency(self):
        """
        Returns an iterator over (node, adjacency dict) tuples for all nodes.
        """
        return iter(self._adj.items())

    def degree(self, v):
        nbrs = self._succ[v]
        return len(nbrs) + (v in nbrs)

    def clear(self):
        """
        Removes all nodes and edges from the graph.
        """
        self._succ.clear()
        self._pred.clear()
        self._node.clear()
        self._edge.clear()
        self.graph.clear()

    def obstacle_free(self, v):
        """
        Checks if the node v is free of an obstacle.
        """
        node = Point(v)
        return not any(node.within(obstacle) for obstacle in self.obstacles)

    def collision_free(self, v, u):
        """
        Checks if the edge (v)-(u) is free of collision.
        """
        # print(self.obstacles)
        edge = LineString([v, u])
        return not any(edge.intersects(obstacle) for obstacle in self.obstacles)


    def add_obstacle(self, o):
        """
        Adds an obstacle to the graph.
        """
        self.obstacles.append(o)

    def remove_obstacle(self, o):
        """
        Removes an obstalce from the graph.
        """
        self.obstacles.remove(o)

    def update_obstacles(self, O):
        """
        Updates obstacles in the graph.
        """
        if self.obstacles in O or self.obstacles:
            for obstacle in self.obstacles:
                self.remove_obstacle(obstacle)
        if self.obstacles not in O or self.obstacles:
            for obstacle in O:
                self.add_obstacle(obstacle)
