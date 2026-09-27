import unittest
from types import SimpleNamespace
from unittest.mock import patch
from src.slam_hardware import HardwareBackend
from src.sensor_pipeline import RobotSensorSnapshot, CalibrationManager
from src.settings import SETTINGS


class ToFScanRangeTests(unittest.TestCase):
    def sample(self, raw):
        clock = SimpleNamespace(now=100.0)
        def sleep(seconds):
            clock.now += seconds
        hub = SimpleNamespace(get_latest_state=lambda: RobotSensorSnapshot(
            tof_raw=raw, tof_received_at=clock.now, gimbal_received_at=clock.now, gimbal_yaw=-180))
        controller = SimpleNamespace(_running=SimpleNamespace(is_set=lambda: True))
        backend = HardwareBackend(SimpleNamespace(robot=None, thread_2_controller=controller,
            thread_1_sensor=None, sensor_hub=hub, calibration_mgr=CalibrationManager(None)))
        with patch('time.monotonic', lambda: clock.now), patch('time.sleep', sleep):
            if raw > 5000:
                with patch.dict(SETTINGS['slam'], {'sensor_timeout_sec': 0.1}):
                    with self.assertRaisesRegex(RuntimeError, 'valid_raw=10..5000.*65000'):
                        backend.sample(-180, clock.now)
                self.assertEqual(backend.event_log[-1]['type'], 'tof_sample_failed')
            else:
                self.assertAlmostEqual(backend.sample(-180, clock.now), raw / 1000)
                self.assertEqual(backend.event_log[-1]['type'], 'tof_sample')

    def test_run22_rear_range_is_accepted(self):
        self.sample(4234)

    def test_invalid_large_value_is_still_rejected_with_diagnostics(self):
        self.sample(65000)


if __name__ == '__main__':
    unittest.main()
