import unittest
from types import SimpleNamespace
from unittest.mock import Mock
from src.sensor_pipeline import RobotSensorSnapshot
from src.slam_hardware import HardwareBackend


class GimbalMotionHoldTests(unittest.TestCase):
    def fixture(self, failed_hold=False):
        events = []
        done = SimpleNamespace(wait_for_completed=lambda timeout: True, has_succeeded=True)
        failed = SimpleNamespace(wait_for_completed=lambda timeout: True, has_succeeded=False)
        def recenter(**kwargs):
            self.assertEqual(kwargs, {'yaw_speed': 300, 'pitch_speed': 120})
            events.append('recenter')
            return done
        def moveto(**kwargs):
            self.assertEqual(kwargs['yaw'], 0)
            self.assertEqual(kwargs['pitch'], 0)
            events.append('hold')
            return failed if failed_hold else done
        state = RobotSensorSnapshot()
        controller = SimpleNamespace(_running=SimpleNamespace(is_set=lambda: True),
            front_ready=False, turn_to_relative=Mock(), stop_chassis=Mock())
        def walk():
            self.assertTrue(controller.front_ready)
            self.assertEqual(events[-1], 'fresh_front_tof')
            events.append('walk')
            return {'completed': True}
        controller.navigate_single_grid_step = walk
        backend = HardwareBackend(SimpleNamespace(robot=SimpleNamespace(gimbal=SimpleNamespace(
            recenter=recenter, moveto=moveto)), thread_2_controller=controller,
            thread_1_sensor=None, sensor_hub=SimpleNamespace(get_latest_state=lambda: state),
            calibration_mgr=None))
        backend.fresh = Mock(return_value=True)
        backend.align_heading = Mock()
        backend.aim = Mock(return_value=100)
        def sample(yaw, after):
            self.assertEqual(events[-1], 'hold')
            events.append('fresh_front_tof')
            return 1.0
        backend.sample = sample
        backend.prepare_stationary_scan = Mock(return_value=state)
        return backend, events

    def test_recenter_once_and_hold_before_each_walk_with_fresh_tof(self):
        backend, events = self.fixture()
        backend.move(0)
        backend.move(0)
        self.assertEqual(events, ['recenter', 'hold', 'fresh_front_tof', 'walk',
                                  'hold', 'fresh_front_tof', 'walk'])
        self.assertEqual(backend.commanded_gimbal_yaw, 0)

    def test_failed_hold_does_not_start_turn_or_drive(self):
        backend, events = self.fixture(failed_hold=True)
        with self.assertRaisesRegex(RuntimeError, 'Gimbal front position hold failed'):
            backend.move(1)
        self.assertEqual(events, ['recenter', 'hold'])
        backend.controller.turn_to_relative.assert_not_called()
        self.assertFalse(backend.controller.front_ready)


if __name__ == '__main__':
    unittest.main()
