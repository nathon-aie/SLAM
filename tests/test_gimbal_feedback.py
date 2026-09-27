import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch
from src.slam_hardware import HardwareBackend
from src.sensor_pipeline import RobotSensorSnapshot
from src.settings import SETTINGS


class GimbalFeedbackTests(unittest.TestCase):
    def run_aim(self, mode):
        clock = SimpleNamespace(now=100.0)
        state = SimpleNamespace(yaw=0.0, pitch=-1.4)
        moves = []
        done = SimpleNamespace(wait_for_completed=lambda timeout: True, has_succeeded=True)
        def sleep(seconds):
            clock.now += seconds
        def move(yaw, pitch, **kwargs):
            moves.append((yaw, pitch))
            state.yaw += yaw * 0.8
            return done
        def feedback():
            return RobotSensorSnapshot(gimbal_yaw=state.yaw, gimbal_pitch=state.pitch,
                gimbal_received_at=100.0 if mode == 'stale' else clock.now,
                tof_received_at=clock.now, tof_raw=1000)
        gimbal = SimpleNamespace(move=move, drive_speed=Mock(), recenter=Mock(return_value=done))
        collector = SimpleNamespace(reset_tof_filter=Mock())
        controller = SimpleNamespace(_running=SimpleNamespace(is_set=lambda: True), stop_chassis=Mock())
        backend = HardwareBackend(SimpleNamespace(robot=SimpleNamespace(gimbal=gimbal),
            thread_2_controller=controller, thread_1_sensor=collector,
            sensor_hub=SimpleNamespace(get_latest_state=feedback), calibration_mgr=SimpleNamespace(raw_to_mm=lambda sensor, raw: raw)))
        with patch('time.monotonic', lambda: clock.now), patch('time.sleep', sleep):
            if mode == 'stale':
                with self.assertRaisesRegex(RuntimeError, 'No fresh valid Gimbal feedback'):
                    backend.aim(90)
                gimbal.drive_speed.assert_called_once_with(pitch_speed=0, yaw_speed=0)
            else:
                after = backend.aim(90)
                self.assertEqual(state.yaw, 72)
                self.assertEqual(state.pitch, -1.4)
                self.assertEqual(backend.commanded_gimbal_yaw, 90)
                # Repeating the target must not trim the missing 18 degrees.
                backend.aim(90)
                with patch.dict(SETTINGS['slam'], {'sensor_timeout_sec': 0.1}):
                    self.assertEqual(backend.sample(90, after), 1.0)
                gimbal.drive_speed.assert_not_called()
        self.assertEqual(moves, [(90, 0)])
        return moves

    def test_angle_mismatch_does_not_block_tof(self):
        clock = SimpleNamespace(now=100.0)
        def sleep(seconds):
            clock.now += seconds
        def feedback():
            return RobotSensorSnapshot(gimbal_yaw=-72.0, gimbal_pitch=12.0, yaw=-1.16,
                attitude_received_at=clock.now, position_received_at=clock.now,
                gimbal_received_at=clock.now, tof_received_at=clock.now, tof_raw=1000)
        backend = HardwareBackend.__new__(HardwareBackend)
        backend.scan_origin = (0, 0, 0)
        backend.heading = 0
        backend.ensure_running = Mock(return_value=0.0)
        backend.event_log = []
        backend.hub = SimpleNamespace(get_latest_state=feedback)
        backend.system = SimpleNamespace(calibration_mgr=SimpleNamespace(raw_to_mm=lambda sensor, raw: raw))
        with patch('time.monotonic', lambda: clock.now), patch('time.sleep', sleep):
            self.assertEqual(backend.sample(-90, 99.9), 1.0)
        self.assertEqual(backend.event_log[-1]['type'], 'tof_sample')

    def test_failed_scan_restores_front_and_preserves_original_error(self):
        backend = HardwareBackend.__new__(HardwareBackend)
        backend.heading = 0
        backend.scan_origin = None
        backend.hub = SimpleNamespace(get_latest_state=lambda: RobotSensorSnapshot())
        backend.fresh = Mock(return_value=True)
        backend.robot = SimpleNamespace(gimbal=SimpleNamespace(drive_speed=Mock()), chassis=SimpleNamespace())
        backend.event_log = []
        backend.controller = SimpleNamespace(stop_chassis=Mock(), front_ready=True,
                                             _running=SimpleNamespace(is_set=lambda: True))
        backend.align_heading = Mock()
        backend.wait_stationary_pose = Mock(return_value=RobotSensorSnapshot())
        backend.initialize_gimbal_reference = Mock()
        backend.aim = Mock(side_effect=[100, RuntimeError('Cleanup failed')])
        backend.sample = Mock(side_effect=RuntimeError('No ToF reading'))
        with patch.dict(SETTINGS['gimbal'], {'scan_mode': 'gimbal'}):
            with self.assertRaisesRegex(RuntimeError, 'No ToF reading'):
                backend.scan()
        backend.initialize_gimbal_reference.assert_called_once()
        self.assertEqual([call.args[0] for call in backend.aim.call_args_list], [0, 0])
        self.assertFalse(backend.controller.front_ready)
        self.assertEqual(backend.event_log[-1]['type'], 'gimbal_cleanup')

    def test_under_rotation_does_not_send_correction_or_pitch_trim(self):
        self.run_aim('normal')

    def test_stale_feedback_does_not_send_another_move(self):
        self.run_aim('stale')



if __name__ == '__main__':
    unittest.main()
