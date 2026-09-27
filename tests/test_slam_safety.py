import time
import unittest
from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import Mock, patch
from src.sensor_pipeline import SensorHub, RobotSensorSnapshot, CalibrationManager, SensorCollectorThread
from src.robot_controller import RobotControllerThread
from src.slam_hardware import HardwareBackend
from src.settings import SETTINGS


class SafetyTests(unittest.TestCase):
    def setUp(self):
        self.hub = SensorHub()
        self.motion = RobotControllerThread(self.hub, mock_mode=True)
        self.motion.strict_sensors = True
        self.motion.front_ready = True
        self.motion.calibration_manager = CalibrationManager(None)
        self.motion._running.set()
        now = time.monotonic()
        self.state = RobotSensorSnapshot(tof_raw=800, tof_filtered_mm=800, tof_valid=True,
            tof_received_at=now, position_received_at=now, attitude_received_at=now, gimbal_received_at=now, sharp_left_received_at=now, sharp_right_received_at=now)
        self.hub.update_state(self.state)

    def test_old_raw_sensor_is_rejected_even_with_new_snapshot(self):
        self.hub.update_state(replace(self.state, tof_received_at=time.monotonic() - 10))
        with self.assertRaisesRegex(RuntimeError, 'Stale sensor'):
            self.motion.motion_state()

    def test_scan_tof_cannot_be_used_until_front_move_completes(self):
        self.hub.update_state(replace(self.state, gimbal_yaw=90))
        self.motion.front_ready = False
        with self.assertRaisesRegex(RuntimeError, 'facing forward'):
            self.motion.motion_state()

    def test_raw_near_obstacle_overrides_delayed_filter(self):
        self.hub.update_state(replace(self.state, tof_raw=70))
        self.assertEqual(self.motion.motion_state().tof_filtered_mm, 70)

    def test_front_obstacle_stops_before_any_forward_command(self):
        self.hub.update_state(replace(self.state, tof_raw=70))
        commands = []
        self.motion.drive_speed = lambda vx, vy, vz: commands.append((vx, vy, vz))
        self.motion.stop_chassis = lambda: commands.append((0, 0, 0))
        result = self.motion.navigate_single_grid_step()
        self.assertFalse(result['completed'])
        self.assertEqual(result['reason'], 'emergency_obstacle')
        self.assertFalse(any(vx > 0 for vx, _, _ in commands))

    def test_front_pid_stop_band_does_not_wait_until_timeout(self):
        self.hub.update_state(replace(self.state, tof_raw=160, tof_filtered_mm=160))
        self.motion.stop_chassis = lambda: None
        self.motion.drive_speed = lambda *args: self.fail("Drive inside stop band")
        result = self.motion.navigate_single_grid_step()
        self.assertFalse(result['completed'])
        self.assertEqual(result['reason'], 'front_wall')

    def wall_stop_result(self, distance, tof=162):
        end = replace(self.state, pos_x=distance, tof_raw=tof, tof_filtered_mm=tof)
        self.motion.motion_state = Mock(side_effect=[self.state, end, end, end])
        self.motion.stop_chassis = Mock()
        self.motion.drive_speed = Mock()
        self.motion.align_at_cell_center = Mock()
        result = self.motion.navigate_single_grid_step()
        self.motion.drive_speed.assert_not_called()
        return result

    def test_run7_front_wall_stop_at_44cm_finishes_cell(self):
        result = self.wall_stop_result(0.446196)
        self.assertTrue(result['completed'])
        self.assertEqual(result['reason'], 'front_wall')
        self.motion.align_at_cell_center.assert_called_once()

    def test_early_front_wall_does_not_advance_cell(self):
        result = self.wall_stop_result(0.1)
        self.assertFalse(result['completed'])
        self.motion.align_at_cell_center.assert_not_called()

    def test_emergency_obstacle_is_not_accepted_as_arrival(self):
        result = self.wall_stop_result(0.446196, tof=70)
        self.assertFalse(result['completed'])
        self.assertEqual(result['reason'], 'emergency_obstacle')
        self.motion.align_at_cell_center.assert_not_called()

    def test_motion_exception_stops_chassis(self):
        self.hub.update_state(replace(self.state, position_received_at=0))
        stopped = []
        self.motion.stop_chassis = lambda: stopped.append(True)
        with self.assertRaises(RuntimeError):
            self.motion.navigate_single_grid_step()
        self.assertTrue(stopped)

    def test_invalid_tof_does_not_start_motion(self):
        self.hub.update_state(replace(self.state, tof_valid=False))
        with self.assertRaises(RuntimeError):
            self.motion.motion_state()

    def test_hardware_scanner_does_not_reuse_same_tof_packet(self):
        system = SimpleNamespace(robot=None, thread_2_controller=self.motion,
            thread_1_sensor=None, sensor_hub=self.hub, calibration_mgr=CalibrationManager(None))
        scanner = HardwareBackend(system)
        with patch.dict(SETTINGS['slam'], {'sensor_timeout_sec': 0.03}):
            with self.assertRaisesRegex(RuntimeError, 'fresh valid ToF'):
                scanner.sample(0, time.monotonic())

    def test_failed_action_is_not_accepted_as_completed(self):
        system = SimpleNamespace(robot=None, thread_2_controller=self.motion,
            thread_1_sensor=None, sensor_hub=self.hub, calibration_mgr=CalibrationManager(None))
        scanner = HardwareBackend(system)
        action = SimpleNamespace(wait_for_completed=lambda timeout: True, has_succeeded=False)
        with self.assertRaisesRegex(RuntimeError, 'action failed'):
            scanner.action_completed(action)

    def test_tof_callback_timestamp_only_changes_on_real_packet(self):
        collector = SensorCollectorThread(self.hub, mock_mode=True)
        self.assertEqual(collector._tof_received_at, 0)
        collector._cb_distance([100])
        self.assertGreater(collector._tof_received_at, 0)
        self.assertEqual(collector._raw_tof, 100)


if __name__ == '__main__':
    unittest.main()
