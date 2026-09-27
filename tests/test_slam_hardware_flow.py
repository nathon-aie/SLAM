"""Exercise the hardware adapter, real motion loop and scanner with a fake SDK."""
import json
import math
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
from src.grid_slam import DFSExplorer, wrap
from src.sensor_pipeline import RobotSensorSnapshot, CalibrationManager
from src.robot_controller import RobotControllerThread
from src.slam_hardware import HardwareBackend
from src.slam_simulation import SimulationBackend
from src.settings import SETTINGS, get


class HardwareFlowTests(unittest.TestCase):
    def setUp(self):
        fixture = patch.dict(SETTINGS['map'], {'rows': 3, 'columns': 3, 'start': {'x': 0, 'y': 0}})
        fixture.start()
        self.addCleanup(fixture.stop)

    def test_scan_turn_drive_backtrack_and_export_without_hardware(self):
        self.exercise_scan("chassis", limited=True)

    def test_full_range_gimbal_scan_still_supported(self):
        self.exercise_scan("gimbal", limited=False)

    def exercise_scan(self, mode, limited):
        fixture = patch.dict(SETTINGS["gimbal"], {"scan_mode": mode})
        fixture.start()
        self.addCleanup(fixture.stop)
        class Clock:
            now = 100.0
            def monotonic(self):
                return self.now
            def sleep(self, seconds):
                self.now += seconds
        clock = Clock()
        world = SimulationBackend()
        state = SimpleNamespace(x=0.0, y=0.0, yaw=0.0, gimbal=0.0, moving_commands=0)
        done = SimpleNamespace(wait_for_completed=lambda timeout: True, has_succeeded=True)

        wheel_stops = []
        class Chassis:
            def drive_wheels(self, w1, w2, w3, w4):
                assert (w1, w2, w3, w4) == (0, 0, 0, 0)
                wheel_stops.append(True)
                return True
            def drive_speed(self, x, y, z, timeout=None):
                if x or y or z:
                    state.moving_commands += 1
                    assert abs(state.gimbal) < 3, 'Drove while ToF faced sideways'
                dt = 1 / get('navigation.control_rate_hz')
                angle = math.radians(state.yaw)
                state.x += (x * math.cos(angle) - y * math.sin(angle)) * dt
                state.y += (x * math.sin(angle) + y * math.cos(angle)) * dt
                state.yaw = wrap(state.yaw + z * get('robot.yaw_speed_command_sign') * dt)
                return True
            def move(self, x, y, z, z_speed):
                state.yaw = wrap(state.yaw + z * get('robot.yaw_command_sign'))
                return done

        recenter_calls = []
        gimbal_speed_commands = []
        commanded_yaws = []
        class Gimbal:
            def recenter(self, yaw_speed, pitch_speed):
                recenter_calls.append(True)
                state.gimbal = 0.0
                return done
            def moveto(self, yaw, pitch, yaw_speed, pitch_speed):
                assert yaw == 0 and pitch == 0
                state.gimbal = yaw
                return done
            def drive_speed(self, pitch_speed, yaw_speed):
                gimbal_speed_commands.append((pitch_speed, yaw_speed))
            def move(self, yaw, pitch, yaw_speed, pitch_speed):
                physical_target = state.gimbal + yaw
                assert -180.01 <= physical_target <= 180.01, "Crossed rear yaw boundary"
                commanded_yaws.append(physical_target)
                target = wrap(physical_target)
                if limited:
                    assert abs(target) <= 75, "Requested an unreachable Gimbal angle"
                state.gimbal = target
                return done

        class Hub:
            def get_latest_state(self):
                # A packet generated every call at the virtual current time.
                cell = (round(state.x / get('navigation.grid_size_m')),
                        round(state.y / get('navigation.grid_size_m')))
                direction = round((state.yaw + state.gimbal) / 90) % 4
                distance = world.world.expected_range(cell, direction, (state.x, state.y), (0, 0))
                assert distance is not None and distance > 0
                return RobotSensorSnapshot(tof_raw=distance * 1000, tof_filtered_mm=distance * 1000,
                    tof_valid=True, pos_x=state.x, pos_y=state.y, yaw=state.yaw,
                    gimbal_yaw=state.gimbal, tof_received_at=clock.now,
                    position_received_at=clock.now, attitude_received_at=clock.now,
                    gimbal_received_at=clock.now, sharp_left_received_at=clock.now, sharp_right_received_at=clock.now)

        hub = Hub()
        mode_changes = []
        def set_mode(mode):
            mode_changes.append(mode)
            return True
        robot = SimpleNamespace(chassis=Chassis(), gimbal=Gimbal(), set_robot_mode=set_mode)
        motion = RobotControllerThread(hub, robot=robot)
        motion._running.set()
        system = SimpleNamespace(robot=robot, thread_2_controller=motion, sensor_hub=hub,
            thread_1_sensor=SimpleNamespace(reset_tof_filter=lambda: None), calibration_mgr=CalibrationManager(None))
        with patch('time.monotonic', clock.monotonic), patch('time.sleep', clock.sleep), tempfile.TemporaryDirectory() as folder:
            output = Path(folder) / 'map.json'
            explorer = DFSExplorer(HardwareBackend(system), output)
            self.assertTrue(explorer.run(), explorer.error)
            self.assertEqual(len(explorer.slam.map.visited), 9)
            self.assertEqual(explorer.slam.cell, (0, 0))
            self.assertGreater(state.moving_commands, 0)
            self.assertEqual(len(wheel_stops), explorer.moves)
            data = json.loads(output.read_text())
            self.assertEqual(data['status'], 'completed')
            self.assertEqual(len(data['scans'][0]['ranges_m']), 4)
            for scan in data['scans'][1:]:
                self.assertEqual(len(scan['ranges_m']), 3)
                rear = (scan['heading'] + 2) % 4
                self.assertNotIn(['N', 'E', 'S', 'W'][rear], scan['ranges_m'])
            self.assertTrue(all(event['mode'] == mode for event in data['events'] if event['type'] == 'scan_mode'))
            if mode == 'gimbal':
                self.assertEqual(len(recenter_calls), 2 * len(data['scans']))
                settled = [event['target_yaw'] for event in data['events']
                           if event['type'] == 'gimbal' and event.get('phase') == 'move_completed']
                self.assertEqual(settled[:5], [0, -90, -180, 90, 0])
                self.assertEqual(commanded_yaws[:3], [-90, -180, 90])
                deltas = [e['delta_yaw'] for e in data['events'] if e['type'] == 'gimbal' and e.get('phase') == 'move']
                self.assertEqual(deltas.count(180), len(data['scans']) - 1)
                self.assertNotIn(-180, commanded_yaws[3:])
                self.assertEqual(mode_changes, [])
                self.assertEqual(gimbal_speed_commands, [])
            else:
                self.assertEqual(len(recenter_calls), 1)
            for key, wall in explorer.slam.map.edges.items():
                self.assertEqual(wall, world.world.edges[key])


if __name__ == '__main__':
    unittest.main()
