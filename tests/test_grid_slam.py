import json
import math
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
from src.grid_slam import DFSExplorer, GridMap, GridSLAM, neighbor
from src.slam_simulation import SimulationBackend
from src.settings import SETTINGS, get


class GridSLAMTests(unittest.TestCase):
    def setUp(self):
        fixture = patch.dict(SETTINGS['map'], {'rows': 3, 'columns': 3, 'start': {'x': 0, 'y': 0}})
        fixture.start()
        self.addCleanup(fixture.stop)

    def test_edge_is_shared_and_unknown_stays_unknown(self):
        grid = GridMap()
        self.assertIsNone(grid.wall((0, 0), 0))
        grid.observe((0, 0), 0, False)
        self.assertIs(grid.wall((1, 0), 2), False)
        with self.assertRaises(RuntimeError):
            grid.observe((1, 0), 2, True)

    def test_known_wall_corrects_biased_odometry(self):
        slam = GridSLAM()
        for d in range(4):
            slam.map.observe((0, 0), d, True)
        slam.pose[:2] = [0.03, -0.02]
        variance = slam.variance[:]
        radius = get('navigation.grid_size_m') / 2 - get('slam.wall_thickness_m') / 2
        slam.update({d: radius for d in range(4)}, 0, 0)
        self.assertLess(abs(slam.pose[0]), 0.005)
        self.assertLess(abs(slam.pose[1]), 0.005)
        self.assertTrue(all(a < b for a, b in zip(slam.variance, variance)))
        self.assertEqual(slam.matches, 4)

    def test_chassis_scan_uses_each_ray_chassis_heading_for_sensor_offset(self):
        with patch.dict(SETTINGS['gimbal'], {'pivot_x_m': 0.07, 'pivot_y_m': 0.01, 'beam_offset_m': 0.02}):
            slam = GridSLAM()
            for d in range(4):
                slam.map.observe((0, 0), d, True)
            radius = get('navigation.grid_size_m') / 2 - get('slam.wall_thickness_m') / 2
            # Forward sensor offset is rotated into each beam direction.
            distances = {d: radius - 0.09 for d in range(4)}
            slam.update(distances, 0, 0, {d: d for d in range(4)})
            self.assertAlmostEqual(slam.pose[0], 0)
            self.assertAlmostEqual(slam.pose[1], 0)
            self.assertEqual(slam.matches, 4)
            self.assertEqual(slam.map.observations[-1]['scan_headings'], {'N': 0, 'E': 1, 'S': 2, 'W': 3})

    def test_close_front_wall_does_not_abort_scan_with_open_right(self):
        slam = GridSLAM()
        slam.update({0: 0.858, 1: 0.85, 2: 2.971, 3: 0.22}, 0, 0)
        slam.predict((1, 0), (0.617974, -0.055377), -0.32)
        slam.update({0: 0.175, 1: 0.909, 2: 3.404, 3: 0.186}, 0, -0.32)
        self.assertIn((1, 0), slam.map.visited)
        self.assertIs(slam.map.wall((1, 0), 0), True)
        self.assertIs(slam.map.wall((1, 0), 1), False)
        self.assertTrue(slam.contains(neighbor((1, 0), 1)))

    def test_run2_stationary_yaw_drift_does_not_abort_mapping(self):
        slam = GridSLAM()
        slam.update({0: 0.864, 1: 0.861, 2: 3.0, 3: 0.22}, 0, -1.16)
        self.assertIn((0, 0), slam.map.visited)
        self.assertIs(slam.map.wall((0, 0), 0), False)
        GridSLAM().update({d: 0.864 for d in range(4)}, 0, -4.0)

    def test_wall_mismatch_logs_and_continues_without_flipping_known_edge(self):
        slam = GridSLAM()
        slam.update({d: 0.2625 for d in range(4)}, 0, 0)
        slam.update({0: 0.9, 1: 0.2625, 2: 0.2625, 3: 0.2625}, 0, 0)
        self.assertIs(slam.map.wall((0, 0), 0), True)
        self.assertEqual(len(slam.map.observations), 2)
        self.assertTrue(any(e['type'] == 'range_mismatch' for e in slam.events))
        self.assertTrue(any(e['type'] == 'wall_mismatch' for e in slam.events))

    def test_run6_odometry_drift_warns_and_commits_backtracking_cell(self):
        slam = GridSLAM()
        slam.rows, slam.columns = 5, 4
        slam.cell = (4, 0)
        slam.pose = [2.437027292722187, -0.11865924690320026, -0.15]
        slam.predict((3, 0), (-0.609904296336804, -0.03521793591208548), 179.75)
        self.assertEqual(slam.cell, (3, 0))
        event = slam.events[-1]
        self.assertEqual(event['type'], 'odometry_mismatch')
        self.assertAlmostEqual(event['error_m'], 0.15624930183552938)
        self.assertEqual(event['cell'], [3, 0])

    def test_predict_still_rejects_outside_map_and_invalid_odometry(self):
        slam = GridSLAM()
        for target, displacement in (((-1, 0), (-0.6, 0)), ((1, 0), (float('nan'), 0))):
            with self.assertRaises(RuntimeError):
                slam.predict(target, displacement, 0)
            self.assertEqual(slam.cell, (0, 0))

    def test_unknown_range_is_not_invented(self):
        grid = GridMap()
        grid.observe((0, 0), 0, False)
        self.assertIsNone(grid.expected_range((0, 0), 0, (0, 0), (0, 0)))

    def test_dfs_covers_world_with_cycles_returns_home_and_matches_walls(self):
        for seed in range(8):
            with self.subTest(seed=seed), patch.dict(SETTINGS['slam'], {'simulation_seed': seed}):
                backend = SimulationBackend()
                with tempfile.TemporaryDirectory() as folder:
                    output = Path(folder) / 'map.json'
                    explorer = DFSExplorer(backend, output)
                    self.assertTrue(explorer.run(), explorer.error)
                    self.assertTrue(backend.stopped)
                    self.assertEqual(explorer.slam.cell, (0, 0))
                    self.assertEqual(len(explorer.slam.map.visited), backend.rows * backend.cols)
                    self.assertEqual(explorer.moves, 2 * (backend.rows * backend.cols - 1))
                    for key, wall in explorer.slam.map.edges.items():
                        self.assertEqual(wall, backend.world.edges[key])
                    result = json.loads(output.read_text())
                    self.assertEqual(result['status'], 'completed')
                    self.assertGreater(result['landmark_matches'], 0)
                    self.assertLess(math.hypot(*result['pose'][:2]), 0.03)

    def test_failed_scan_saves_partial_map_and_stops(self):
        class Broken(SimulationBackend):
            def scan(self):
                raise RuntimeError('ToF disconnected')
        with tempfile.TemporaryDirectory() as folder:
            backend = Broken()
            output = Path(folder) / 'partial.json'
            explorer = DFSExplorer(backend, output)
            self.assertFalse(explorer.run())
            self.assertTrue(backend.stopped)
            result = json.loads(output.read_text())
            self.assertEqual(result['status'], 'failed')
            self.assertIn('disconnected', result['error'])
            self.assertEqual(result['visited'], [])

    def test_limit_is_not_reported_as_complete(self):
        with patch.dict(SETTINGS['slam'], {'max_cells': 2}), tempfile.TemporaryDirectory() as folder:
            explorer = DFSExplorer(SimulationBackend(), Path(folder) / 'map.json')
            self.assertFalse(explorer.run())
            self.assertEqual(explorer.status, 'limit_reached')
            self.assertEqual(len(explorer.slam.map.visited), 2)

    def test_nonzero_sensor_offset_and_heading(self):
        with patch.dict(SETTINGS['gimbal'], {'pivot_x_m': 0.07, 'pivot_y_m': 0.01, 'beam_offset_m': 0.02}):
            backend = SimulationBackend()
            with tempfile.TemporaryDirectory() as folder:
                explorer = DFSExplorer(backend, Path(folder) / 'map.json')
                self.assertTrue(explorer.run(), explorer.error)
                for key, wall in explorer.slam.map.edges.items():
                    self.assertEqual(wall, backend.world.edges[key])

    def test_rectangular_map_and_nonzero_start_are_used_everywhere(self):
        geometry = {'rows': 2, 'columns': 4, 'start': {'x': 1, 'y': 2}}
        with patch.dict(SETTINGS['map'], geometry), tempfile.TemporaryDirectory() as folder:
            backend = SimulationBackend()
            output = Path(folder) / 'map.json'
            explorer = DFSExplorer(backend, output)
            self.assertEqual(explorer.slam.cell, (1, 2))
            self.assertEqual(explorer.stack, [(1, 2)])
            self.assertTrue(explorer.run(), explorer.error)
            self.assertEqual(explorer.slam.cell, (1, 2))
            self.assertEqual(len(explorer.slam.map.visited), 8)
            data = json.loads(output.read_text())
            self.assertEqual(data['map_info'], {'rows': 2, 'columns': 4})
            self.assertEqual(data['start_cell'], [1, 2])
            self.assertEqual(data['start_pose'], [0.6, 1.2, 0.0])
            self.assertLess(math.hypot(data['pose'][0] - 0.6, data['pose'][1] - 1.2), 0.03)

    def test_invalid_dimensions_and_start_fail_before_exploration(self):
        for geometry in ({'rows': 0}, {'columns': 2.5}, {'start': {'x': -1, 'y': 0}},
                         {'start': {'x': 0, 'y': 3}}, {'start': {'x': True, 'y': 0}}):
            with self.subTest(geometry=geometry), patch.dict(SETTINGS['map'], geometry):
                with self.assertRaises(ValueError):
                    GridSLAM()
                with self.assertRaises(ValueError):
                    SimulationBackend()

    def test_open_boundary_is_observed_but_never_traversed(self):
        with patch.dict(SETTINGS['map'], {'rows': 1, 'columns': 1, 'start': {'x': 0, 'y': 0}}):
            class OpenBoundary:
                def scan(self):
                    return {d: 1.0 for d in range(4)}, 0, 0
                def move(self, direction):
                    raise AssertionError('Moved outside the single-cell map')
                def stop(self):
                    pass
            with tempfile.TemporaryDirectory() as folder:
                explorer = DFSExplorer(OpenBoundary(), Path(folder) / 'map.json')
                self.assertTrue(explorer.run(), explorer.error)
                self.assertEqual(explorer.moves, 0)
                self.assertEqual(explorer.slam.map.visited, {(0, 0)})
                self.assertTrue(all(explorer.slam.map.wall((0, 0), d) is False for d in range(4)))
                result = json.loads((Path(folder) / 'map.json').read_text())
                self.assertTrue(all(edge['boundary_blocked'] for edge in result['edges']))
                self.assertEqual(len([e for e in result['events'] if e['type'] == 'boundary_open']), 4)
            with self.assertRaisesRegex(RuntimeError, 'outside configured map'):
                explorer.slam.predict((1, 0), (0.6, 0), 0)
            self.assertEqual(explorer.slam.cell, (0, 0))

    def test_dfs_explores_inside_and_returns_home_despite_open_exits(self):
        with patch.dict(SETTINGS['map'], {'rows': 2, 'columns': 2, 'start': {'x': 0, 'y': 0}}):
            class WithExits(SimulationBackend):
                def scan(self):
                    ranges, heading, yaw = super().scan()
                    for d in range(4):
                        x, y = neighbor(self.cell, d)
                        if not (0 <= x < self.rows and 0 <= y < self.cols):
                            ranges[d] = 1.0
                    return ranges, heading, yaw
                def move(self, direction):
                    x, y = neighbor(self.cell, direction)
                    if not (0 <= x < self.rows and 0 <= y < self.cols):
                        raise AssertionError('Issued movement outside configured bounds')
                    return super().move(direction)
            with tempfile.TemporaryDirectory() as folder:
                explorer = DFSExplorer(WithExits(), Path(folder) / 'map.json')
                self.assertTrue(explorer.run(), explorer.error)
                self.assertEqual(len(explorer.slam.map.visited), 4)
                self.assertEqual(explorer.slam.cell, (0, 0))
                self.assertEqual(explorer.moves, 6)


if __name__ == '__main__':
    unittest.main()
