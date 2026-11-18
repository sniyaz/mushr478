#!/usr/bin/env python3

import numpy as np
import os
import rosunit
import unittest

from planning.lazysp import LazySP
from planning.samplers import HaltonSampler
from planning.problems import R2Problem
from planning.roadmap import Roadmap


class TestLazySPR2(unittest.TestCase):
    def setUp(self):
        fname = os.path.join(os.path.dirname(__file__), "share", "map1.txt")
        permissible_region = np.loadtxt(fname, dtype=bool)
        self.problem = R2Problem(permissible_region, check_resolution=0.5)
        self.sampler = HaltonSampler(self.problem.extents)

    def _solve(self):
        num_vertices = 10
        connection_radius = 5.0
        lazy = True
        rm = Roadmap(self.problem, self.sampler, num_vertices, connection_radius, lazy)
        start_id = rm.add_node(np.array([1, 1]), is_start=True)
        goal_id = rm.add_node(np.array([8, 8]), is_start=False)
        planner = LazySP(rm)
        path, parents = planner.solve(start_id, goal_id)
        return rm, planner, start_id, goal_id, path, parents

    def test_lazysp_r2problem(self):
        rm, _, start_id, goal_id, path, _ = self._solve()
        correct_path = [start_id, 9, 2, 7, goal_id]

        self.assertEqual(path[0], start_id, msg="Path must begin at start")
        self.assertEqual(path[-1], goal_id, msg="Path must end at goal")
        np.testing.assert_almost_equal(
            rm.compute_path_length(path),
            rm.compute_path_length(correct_path),
            err_msg="LazySP implementation is incorrect",
        )
        self.assertEqual(
            path,
            correct_path,
            msg="LazySP implementation is incorrect",
        )
        for u, v in zip(path[:-1], path[1:]):
            self.assertTrue(
                self.problem.check_edge_validity(rm.vertices[u, :], rm.vertices[v, :]),
                msg="Every edge on the returned path must be collision-free",
            )

        # LazySP should only need 10 edges on this problem.
        self.assertEqual(
            rm.edges_evaluated, 10, msg="Incorrect number of edges evaluated"
        )

    def test_lazysp_memoization(self):
        rm, planner, _, _, _, _ = self._solve()

        self.assertEqual(
            rm.edges_evaluated,
            len(planner.edge_status),
            msg="Each edge should be collision-checked at most once",
        )
        self.assertIn(
            False,
            planner.edge_status.values(),
            msg="Edges found in collision should be remembered in edge_status",
        )
        for (u, v), valid in planner.edge_status.items():
            self.assertTrue(
                rm.graph.has_edge(u, v),
                msg="The original roadmap graph should not be modified",
            )
            self.assertEqual(
                planner.graph.has_edge(u, v),
                valid,
                msg="Edges in collision (and only those) should be removed",
            )

        # Re-checking already-evaluated edges must not collision-check again.
        evaluated = rm.edges_evaluated
        for (u, v), valid in list(planner.edge_status.items()):
            self.assertEqual(planner.check_edge(u, v), valid)
            self.assertEqual(planner.check_edge(v, u), valid)
        self.assertEqual(
            rm.edges_evaluated,
            evaluated,
            msg="check_edge should use memoized results",
        )


if __name__ == "__main__":
    rosunit.unitrun("planning", "test_lazysp_r2", TestLazySPR2)
