"""Implementation of LazySP (Lazy Shortest Path) with the forward edge selector.

Reference: C. Dellin, S. Srinivasa.
"A Unifying Formalism for Shortest Path Problems with Expensive Edge
Evaluations via Lazy Best-First Search over Paths with Edge Selectors." ICRA 2016.
https://arxiv.org/abs/1603.03490
"""

import networkx as nx

from planning import search


class LazySP(object):
    def __init__(self, rm):
        """Construct a LazySP planner on top of a (lazy) roadmap.

        LazySP never collision-checks an edge during search. Instead, it
        repeatedly (1) finds the shortest path on its current graph, assuming
        every unevaluated edge is collision-free, then (2) collision-checks
        edges along that path. Edges found to be in collision are deleted
        from the graph, and the search is repeated.

        A LazySP object can be passed to your graph search from Lab 8 in place
        of a Roadmap, e.g. `search.dijkstra(self, start, goal)`. The search
        will use self.graph (edges in collision removed) and self.lazy
        (always False, so the search never collision-checks edges itself).
        Every other attribute (num_vertices, heuristic, ...) is forwarded to
        the underlying Roadmap.

        Args:
            rm: Roadmap instance to plan on (should be constructed with
                lazy=True, so no edges are collision-checked up front)
        """
        self.rm = rm

        # A working copy of the roadmap graph. Edges found to be in collision
        # are removed from this copy (rm.graph is left untouched).
        self.graph = rm.graph.copy()

        # The inner graph search should NOT collision-check edges; LazySP
        # decides which edges to evaluate.
        self.lazy = False

        # Memoized edge evaluations: maps edge_key(u, v) to True if the edge
        # is collision-free and False if it is in collision. An edge should
        # never be collision-checked more than once.
        self.edge_status = {}

    def __getattr__(self, name):
        # Only called for attributes not found on this object: forward them to
        # the underlying Roadmap (e.g., num_vertices, heuristic, vertices).
        return getattr(self.rm, name)

    def edge_key(self, u, v):
        """Return a hashable key for the edge between nodes u and v.

        R2 roadmaps are undirected, so (u, v) and (v, u) are the same edge.
        SE2 roadmaps are directed, so they are different edges.
        """
        u, v = int(u), int(v)
        if self.rm.directed:
            return (u, v)
        return (min(u, v), max(u, v))

    def check_edge(self, u, v):
        """Return whether the edge between u and v is collision-free.

        The result should be memoized in self.edge_status, so each edge is
        collision-checked (with self.rm.check_edge_validity) at most once.

        Args:
            u, v: integer node labels

        Returns:
            valid: whether the edge is collision-free
        """
        # BEGIN Lab 9, Q3
        "*** REPLACE THIS LINE ***"
        raise NotImplementedError
        # END Lab 9, Q3

    def solve(self, start, goal):
        """Compute the shortest collision-free path from start to goal.

        Repeat until a path is returned:
          1. Find the shortest path on self.graph with your Lab 8 search.
          2. Forward selector: check the edges along the path in order, from
             the start towards the goal (use self.check_edge).
          3. If an edge is in collision, delete it from self.graph and go back
             to step 1 immediately (don't check the rest of the path).
          4. If every edge on the path is collision-free, return the path.

        If no path exists, your search will raise nx.NetworkXNoPath; let it
        propagate to the caller.

        Args:
            start, goal: integer labels for the start and goal states

        Returns:
            vpath: a sequence of node labels, including the start and goal
            parents: the parents array returned by the final search
        """
        # BEGIN Lab 9, Q3
        "*** REPLACE THIS LINE ***"
        raise NotImplementedError
        # END Lab 9, Q3
