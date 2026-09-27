import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch
from src.slam_hardware import HardwareBackend
from src.sensor_pipeline import RobotSensorSnapshot
from src.settings import SETTINGS


class StationaryScanTests(unittest.TestCase):
    def fixture(self):
        state = RobotSensorSnapshot()
        robot = SimpleNamespace(gimbal=SimpleNamespace(drive_speed=Mock()),
                                chassis=SimpleNamespace(drive_speed=Mock()), set_robot_mode=Mock())
        controller = SimpleNamespace(_running=threading.Event(), stop_chassis=Mock(), drive_speed=Mock())
        controller._running.set()
        backend = HardwareBackend(SimpleNamespace(robot=robot, thread_2_controller=controller,
            thread_1_sensor=None, sensor_hub=SimpleNamespace(get_latest_state=lambda: state), calibration_mgr=None))
        backend.fresh = Mock(return_value=True)
        return state, backend

    def test_gimbal_scan_issues_no_chassis_rotation_or_mode_switch(self):
        state, backend = self.fixture()
        backend.align_heading = Mock()
        backend.wait_stationary_pose = Mock(return_value=state)
        backend.initialize_gimbal_reference = Mock()
        backend.recenter_gimbal = Mock()
        backend.aim = Mock(return_value=100)
        backend.sample = Mock(return_value=1)
        with patch.dict(SETTINGS['gimbal'], {'scan_mode': 'gimbal'}):
            ranges, heading, yaw = backend.scan()
        self.assertEqual(set(ranges), {0, 1, 2, 3})
        backend.align_heading.assert_not_called()
        backend.controller.drive_speed.assert_not_called()
        backend.controller.stop_chassis.assert_not_called()
        backend.robot.chassis.drive_speed.assert_not_called()
        backend.robot.gimbal.drive_speed.assert_not_called()
        backend.robot.set_robot_mode.assert_not_called()
        self.assertIsNone(backend.scan_origin)
        self.assertFalse(backend.controller.front_ready)

    def test_guard_refuses_actual_rotation_without_commanding_correction(self):
        state, backend = self.fixture()
        backend.scan_origin = (0, 0, 0)
        state.yaw = 4
        with self.assertRaisesRegex(RuntimeError, 'chassis moved'):
            backend.ensure_running()
        backend.controller.drive_speed.assert_not_called()

    def test_run9_position_drift_warns_once_without_aborting_or_motor_commands(self):
        state, backend = self.fixture()
        backend.scan_origin = (1.7453375, 1.2768862, 89.5)
        state.pos_x, state.pos_y, state.yaw = 1.7454, 1.2431, 89.5
        backend.ensure_running()
        backend.ensure_running()
        warnings = [e for e in backend.event_log if e['type'] == 'scan_position_drift']
        self.assertEqual(len(warnings), 1)
        self.assertAlmostEqual(warnings[0]['position_drift_m'], 0.0337862, places=5)
        self.assertEqual(backend.scan_origin, (1.7453375, 1.2768862, 89.5))
        backend.controller.drive_speed.assert_not_called()
        backend.controller.stop_chassis.assert_not_called()
        backend.robot.gimbal.drive_speed.assert_not_called()
        backend.scan_position_warning_logged = False  # Next scan may log its own warning.
        backend.ensure_running()
        self.assertEqual(len(backend.event_log), 2)

    def recovery_fixture(self, recover_at=None, recovered_yaw=0, interrupt_at=None):
        _, backend = self.fixture()
        clock = SimpleNamespace(now=100.0)
        backend.scan_origin = (0, 0, 0)
        backend.heading = 0
        backend.fresh = HardwareBackend.fresh.__get__(backend)
        backend.system.calibration_mgr = SimpleNamespace(raw_to_mm=lambda sensor, raw: raw)
        def feedback():
            recovered = recover_at is not None and clock.now >= recover_at
            return RobotSensorSnapshot(position_received_at=clock.now,
                attitude_received_at=clock.now if recovered else 99.0,
                yaw=recovered_yaw if recovered else 0,
                tof_received_at=clock.now, tof_raw=1000, gimbal_received_at=clock.now)
        backend.hub.get_latest_state = feedback
        def sleep(seconds):
            clock.now += seconds
            if interrupt_at is not None and clock.now >= interrupt_at:
                backend.controller._running.clear()
        return backend, clock, sleep

    def test_run5_packet_delay_recovers_without_motor_commands(self):
        backend, clock, sleep = self.recovery_fixture(recover_at=100.2)
        with patch('time.monotonic', lambda: clock.now), patch('time.sleep', sleep):
            self.assertGreaterEqual(backend.ensure_running(), 0.2)
        self.assertEqual([e['type'] for e in backend.event_log],
                         ['scan_pose_wait', 'scan_pose_recovered'])
        backend.controller.drive_speed.assert_not_called()
        backend.robot.gimbal.drive_speed.assert_not_called()
        self.assertEqual(backend.scan_origin, (0, 0, 0))

    def test_persistent_pose_loss_times_out(self):
        backend, clock, sleep = self.recovery_fixture()
        with patch('time.monotonic', lambda: clock.now), patch('time.sleep', sleep), \
                patch.dict(SETTINGS['slam'], {'scan_pose_recovery_timeout_sec': 0.1}):
            with self.assertRaisesRegex(RuntimeError, 'stale chassis pose persisted'):
                backend.ensure_running()
        self.assertAlmostEqual(clock.now, 100.1, delta=0.011)

    def test_recovered_pose_still_checks_actual_drift(self):
        backend, clock, sleep = self.recovery_fixture(recover_at=100.05, recovered_yaw=4)
        with patch('time.monotonic', lambda: clock.now), patch('time.sleep', sleep):
            with self.assertRaisesRegex(RuntimeError, 'chassis moved'):
                backend.ensure_running()

    def test_interruption_ends_pose_wait_promptly(self):
        backend, clock, sleep = self.recovery_fixture(interrupt_at=100.05)
        with patch('time.monotonic', lambda: clock.now), patch('time.sleep', sleep):
            with self.assertRaisesRegex(RuntimeError, 'Motion was interrupted'):
                backend.ensure_running()
        self.assertLess(clock.now, 100.1)

    def test_pose_wait_does_not_consume_tof_sample_timeout(self):
        backend, clock, sleep = self.recovery_fixture(recover_at=100.2)
        with patch('time.monotonic', lambda: clock.now), patch('time.sleep', sleep), \
                patch.dict(SETTINGS['slam'], {'sensor_timeout_sec': 0.1}):
            self.assertEqual(backend.sample(0, 99.9), 1.0)
        self.assertEqual(backend.event_log[-1]['type'], 'tof_sample')


if __name__ == '__main__':
    unittest.main()
