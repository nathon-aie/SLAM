"""Procedural sensor-world simulator. Ground truth stays inside the backend.

No prebuilt map file or route is loaded. This tests exploration/localization,
not motor physics or real PID performance (use step-test --mock for that).
"""
import random
from .grid_slam import GridMap, GridSLAM, neighbor, wrap
from .settings import get as setting, map_geometry


class SimulationBackend:
    def __init__(self):
        self.rng = random.Random(setting('slam.simulation_seed'))
        self.rows, self.cols, self.cell = map_geometry()
        self.heading = 0
        self.world = GridMap()
        cells = [(x, y) for x in range(self.rows) for y in range(self.cols)]
        for cell in cells:
            for d in range(4):
                self.world.observe(cell, d, True)
        # Random spanning tree makes all cells reachable; extra edges exercise cycles.
        seen = {self.cell}
        stack = [self.cell]
        while stack:
            cell = stack[-1]
            choices = [(d, neighbor(cell, d)) for d in range(4)
                       if neighbor(cell, d) in cells and neighbor(cell, d) not in seen]
            if not choices:
                stack.pop()
                continue
            d, target = self.rng.choice(choices)
            self.world.edges[self.world.edge(cell, d)] = False
            seen.add(target)
            stack.append(target)
        for cell in cells:
            for d in (0, 1):
                if neighbor(cell, d) in cells and self.rng.random() < 0.3:
                    self.world.edges[self.world.edge(cell, d)] = False
        self.geometry = GridSLAM()
        self.stopped = False
        self.initial_scan_completed = False

    def scan(self):
        pose = [v * setting('navigation.grid_size_m') for v in self.cell]
        ranges = {}
        directions = range(4) if not self.initial_scan_completed else [(self.heading + r) % 4 for r in (0, 3, 1)]
        for d in directions:
            expected = self.world.expected_range(self.cell, d, pose, self.geometry.offset(self.heading, d))
            if expected is None or expected <= 0:
                raise RuntimeError('Simulation sensor geometry is invalid')
            ranges[d] = expected + self.rng.gauss(0, 0.001)
        self.initial_scan_completed = True
        return ranges, self.heading, wrap(self.heading * 90)

    def move(self, direction):
        if self.world.wall(self.cell, direction) is not False:
            raise RuntimeError('Simulation collision prevented')
        old = self.cell
        self.cell = neighbor(old, direction)
        self.heading = direction
        size = setting('navigation.grid_size_m')
        # Systematic odometry bias demonstrates known-wall pose correction on revisits.
        displacement = tuple((self.cell[i] - old[i]) * size + 0.006 for i in (0, 1))
        return displacement, wrap(direction * 90)

    def stop(self):
        self.stopped = True
