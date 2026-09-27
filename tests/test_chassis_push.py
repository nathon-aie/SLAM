"""Exercise the installed SDK's actual no-ACK path without robot/network I/O."""
import unittest
import threading
from types import SimpleNamespace
from unittest.mock import Mock, patch
from robomaster.chassis import Chassis
from robomaster.client import Client
from src.robot_controller import RobotControllerThread
from src.sensor_pipeline import SensorHub
from src.sdk_connection import cancel_chassis_speed_timer


class ChassisPushTests(unittest.TestCase):
    def sdk_chassis(self):
        client = SimpleNamespace(_running=True, hostbyte=1, send_msg=Mock())
        client.send_sync_msg = lambda msg: Client.send_sync_msg(client, msg)
        chassis = Chassis(SimpleNamespace(client=client, action_dispatcher=None))
        controller = RobotControllerThread(SensorHub(), robot=SimpleNamespace(chassis=chassis))
        return client, chassis, controller

    def test_pending_sdk_timer_is_cancelled_before_scan(self):
        chassis = Chassis.__new__(Chassis)
        callback = Mock()
        timer = threading.Timer(0.2, callback)
        chassis._auto_timer = timer
        timer.start()
        try:
            cancel_chassis_speed_timer(chassis)
            self.assertFalse(timer.is_alive())
            self.assertIsNone(chassis._auto_timer)
            callback.assert_not_called()
        finally:
            timer.cancel()
            timer.join()

    def test_scan_refuses_a_timer_that_cannot_be_drained(self):
        chassis = Chassis.__new__(Chassis)
        chassis._auto_timer = Mock()
        chassis._auto_timer.is_alive.return_value = True
        with self.assertRaisesRegex(RuntimeError, 'timer did not finish'):
            cancel_chassis_speed_timer(chassis)

    def test_real_sdk_sends_no_ack_packet_but_returns_false(self):
        client, chassis, controller = self.sdk_chassis()
        with patch('robomaster.chassis.threading.Timer'):
            self.assertIs(chassis.drive_speed(x=0.1), False)
            controller.drive_speed(0.1, 0, 0)
        self.assertEqual(client.send_msg.call_count, 2)
        msg = client.send_msg.call_args.args[0]
        self.assertEqual(msg._need_ack, 0)
        self.assertAlmostEqual(msg.get_proto()._x_spd, 0.1)

    def test_disconnected_sdk_client_is_still_a_failure(self):
        client, chassis, controller = self.sdk_chassis()
        client._running = False
        with patch('robomaster.chassis.threading.Timer'):
            with self.assertRaisesRegex(RuntimeError, 'Chassis drive failed'):
                controller.drive_speed(0.1, 0, 0)
        client.send_msg.assert_not_called()

    def test_explicit_false_from_other_driver_is_not_ignored(self):
        driver = SimpleNamespace(drive_speed=Mock(return_value=False))
        controller = RobotControllerThread(SensorHub(), robot=SimpleNamespace(chassis=driver))
        with self.assertRaisesRegex(RuntimeError, 'Chassis rejected drive command'):
            controller.drive_speed(0.1, 0, 0)

    def test_send_exception_still_stops_motion(self):
        client, chassis, controller = self.sdk_chassis()
        chassis.drive_speed = Mock(side_effect=OSError('Link lost'))
        with self.assertRaisesRegex(RuntimeError, 'Link lost'):
            controller.drive_speed(0.1, 0, 0)


if __name__ == '__main__':
    unittest.main()
