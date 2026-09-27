import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch
from robomaster.chassis import Chassis
from src.sdk_connection import stop_chassis_wheels
from src.slam_hardware import HardwareBackend
from src.sensor_pipeline import RobotSensorSnapshot


class ScanHandoffTests(unittest.TestCase):
    def test_real_sdk_wheel_stop_requests_ack_and_sets_all_wheels_zero(self):
        response = SimpleNamespace(get_proto=lambda: SimpleNamespace(_retcode=0))
        client = SimpleNamespace(hostbyte=1, send_sync_msg=Mock(return_value=response))
        chassis = Chassis(SimpleNamespace(client=client, action_dispatcher=None))
        stop_chassis_wheels(chassis)
        msg = client.send_sync_msg.call_args.args[0]
        self.assertGreater(msg._need_ack, 0)
        proto = msg.get_proto()
        self.assertEqual((proto._w1_spd, proto._w2_spd, proto._w3_spd, proto._w4_spd), (0, 0, 0, 0))

    def test_missing_wheel_stop_ack_is_not_ignored(self):
        with self.assertRaisesRegex(RuntimeError, 'not acknowledged'):
            stop_chassis_wheels(SimpleNamespace(drive_wheels=Mock(return_value=False)))

    def handoff(self, drifting):
        clock = SimpleNamespace(now=100.0)
        def sleep(seconds):
            clock.now += seconds
        def state():
            return RobotSensorSnapshot(position_received_at=clock.now, attitude_received_at=clock.now,
                                       yaw=(clock.now - 100) * 0.8 if drifting else 0)
        backend = HardwareBackend.__new__(HardwareBackend)
        backend.robot = SimpleNamespace(chassis=SimpleNamespace(drive_wheels=Mock(return_value=True)))
        backend.controller = SimpleNamespace(front_ready=True)
        backend.event_log = []
        backend.hub = SimpleNamespace(get_latest_state=state)
        backend.ensure_running = Mock()
        with patch('time.monotonic', lambda: clock.now), patch('time.sleep', sleep):
            backend.prepare_stationary_scan()
            self.assertEqual(backend.event_log[-1]['type'], 'chassis_stationary')
        self.assertFalse(backend.controller.front_ready)
        self.assertGreater(clock.now, 100.3)

    def test_run25_initial_position_reset_does_not_trigger_scan_drift(self):
        clock = SimpleNamespace(now=100.0)
        def sleep(seconds):
            clock.now += seconds
        def state():
            # Initial pose was zeroed on the previous SDK coordinate value.
            # SDK resets its counter shortly after startup, like run25.
            return RobotSensorSnapshot(pos_x=0 if clock.now < 100.15 else -0.059,
                position_received_at=clock.now, attitude_received_at=clock.now)
        controller = SimpleNamespace(_running=SimpleNamespace(is_set=lambda: True),
                                     stop_chassis=Mock(), drive_speed=Mock())
        backend = HardwareBackend(SimpleNamespace(robot=SimpleNamespace(chassis=SimpleNamespace(),
            gimbal=SimpleNamespace(drive_speed=Mock())), thread_2_controller=controller,
            thread_1_sensor=None, sensor_hub=SimpleNamespace(get_latest_state=state), calibration_mgr=None))
        def recenter():
            sleep(0.5)
            backend.ensure_running()
        backend.recenter_gimbal = Mock()
        backend.initialize_gimbal_reference = Mock(side_effect=recenter)
        backend.aim = Mock(return_value=0)
        backend.sample = Mock(return_value=1)
        with patch('time.monotonic', lambda: clock.now), patch('time.sleep', sleep):
            ranges, _, _ = backend.scan()
        self.assertEqual(set(ranges), {0, 1, 2, 3})
        baseline = next(e for e in backend.event_log if e['type'] == 'chassis_stationary')
        self.assertAlmostEqual(baseline['position_m'][0], -0.059)
        controller.drive_speed.assert_not_called()
        controller.stop_chassis.assert_not_called()
        backend.robot.gimbal.drive_speed.assert_not_called()

    def test_ack_is_followed_by_fixed_pause_and_fresh_pose(self):
        self.handoff(False)

    def test_small_pose_change_does_not_restart_settling_pause(self):
        self.handoff(True)


if __name__ == '__main__':
    unittest.main()
