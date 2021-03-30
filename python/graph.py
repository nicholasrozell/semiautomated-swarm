from shapely.geometry import LineString, Point


class Graph:
    """
    Base class for directed graphs.
    """
    def __init__(self, span, obstacles=[]):
        self.graph_attr_dict_factory = dict
        self.node_dict_factory = dict
        self.node_attr_dict_factory = dict
        self.adjlist_outer_dict_factory = dict
        self.adjlist_inner_dict_factory = dict
        self.edge_list_factory = list
        self.edge_attr_dict_factory = dict

        self.graph = self.graph_attr_dict_factory()
        self._node = self.node_dict_factory()
        self._adj = self.adjlist_outer_dict_factory()
        self._pred = self.adjlist_outer_dict_factory()
        self._succ = self._adj
        self._edge = self.edge_list_factory()

        self.span = span
        self.dimensions = len(span)
        self.obstacles = obstacles

    def __contains__(self, v):
        """
        Returns True if 'v' is a node, False otherwise. Use: 'v in G'.
        """
        try:
            return v in self._node
        except NameError:
            return False

    def add_node(self, v, cost):
        """
        Adds node 'v' to the graph.
        """
        if v not in self._succ:
            self._succ[v] = self.adjlist_inner_dict_factory()
            self._pred[v] = self.adjlist_inner_dict_factory()
            self._node[v] = cost

    def remove_node(self, v):
        """
        Removes node 'v' from the graph.
        """
        try:
            nbrs = self._succ[v]
            del self._node[v]
        except NameError:
            raise NameError("The node {v} is not in the graph.".format(v = v))
        for u in nbrs:
            del self._pred[u][v]
        del self._succ[v]
        for u in self._pred[v]:
            del self._succ[u][v]
        del self._pred[v]

    def num_nodes(self):
        """
        Returns the total number of nodes in the graph.
        """
        return len(self._node.keys())

    def add_edge(self, u, v):
        """
        Adds edge 'u -- v' to the graph.
        """
        if u not in self._node:
            self._succ[u] = self.adjlist_inner_dict_factory()
            self._pred[u] = self.adjlist_inner_dict_factory()
        if v not in self._node:
            self._succ[v] = self.adjlist_inner_dict_factory()
            self._pred[v] = self.adjlist_inner_dict_factory()
        if (u, v) not in self._edge:
            self._edge.append((u, v))
        datadict = self._adj[v].get(v, self.edge_attr_dict_factory())
        self._succ[u][v] = datadict
        self._pred[v][u] = datadict

    def remove_edge(self, u, v):
        """
        Removes edge 'u -- v' from the graph.
        """
        try:
            del self._succ[u][v]
            del self._pred[v][u]
            self._edge.remove((u, v))
        except NameError:
            raise NameError("The edge {u}--{v} is not in the graph.".format(u = u, v = v))

    def size(self):
        """
        Returns the number of edges.
        """
        s = sum(d for v, d in self.degree())
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
        Returns the neighbors of node 'v'.
        """
        try:
            return iter(self._adj[v])
        except KeyError:
            raise KeyError("The node {v} is not in the graph.".format(v = v))

    def successors(self, v):
        """
        Returns the successors of node 'v'.
        """
        try:
            return iter(self._succ[v])
        except KeyError:
            raise KeyError("The node {v} is not in the graph.".format(v = v))
    
    def predecessor(self, v):
        """
        Returns the predecessor of node 'v'.
        """
        try:
            return iter(self._pred[v])
        except KeyError:
            raise KeyError("The node {v} is not in the graph.".format(v = v))

    def degree(self, v):
        """
        Returns the number of successors of node 'v'.
        """
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
        Checks if the node 'v' is free of an obstacle.
        """
        node = Point(v)
        return not any(node.within(obstacle) for obstacle in self.obstacles)  # checks if point is within any obstacle

    def collision_free(self, u, v):
        """
        Checks if the edge 'u -- v' is free of collision.
        """
        edge = LineString([u, v])
        return not any(edge.intersects(obstacle) for obstacle in self.obstacles)  # checks if line intersects any obstacles
