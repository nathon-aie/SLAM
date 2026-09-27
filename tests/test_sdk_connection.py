import argparse
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch
import yaml
from robomaster import conn
from src.sdk_connection import canonical_connection_type, initialize_robot
from src import calibrate
from src.robot_system import RobotSystem
import main


class SDKConnectionTests(unittest.TestCase):
    def test_yaml_and_cli_strings_map_to_exact_sdk_objects(self):
        yaml_mode = yaml.safe_load('conn_type: ap')['conn_type']
        self.assertEqual(yaml_mode, conn.CONNECTION_WIFI_AP)
        self.assertIsNot(yaml_mode, conn.CONNECTION_WIFI_AP)
        parser = argparse.ArgumentParser()
        parser.add_argument('--conn-type')
        cli_mode = parser.parse_args(['--conn-type', ''.join(['s', 't', 'a'])]).conn_type
        for value, expected in ((yaml_mode, conn.CONNECTION_WIFI_AP), (cli_mode, conn.CONNECTION_WIFI_STA)):
            self.assertIs(canonical_connection_type(value), expected)
            robot = Mock()
            robot.initialize.return_value = True
            initialize_robot(robot, value)
            self.assertIs(robot.initialize.call_args.kwargs['conn_type'], expected)

    def test_real_sdk_request_branch_accepts_yaml_ap_without_network(self):
        with patch.object(conn.socket, 'socket'):
            connection = conn.SdkConnection()
            connection.switch_remote_route = Mock(return_value=(True, '192.168.2.23'))
            try:
                mode = canonical_connection_type(yaml.safe_load('mode: ap')['mode'])
                result, local, remote = connection.request_connection(1, conn_type=mode)
                self.assertTrue(result)
                self.assertEqual(local[0], '192.168.2.23')
                self.assertEqual(remote[0], '192.168.2.1')
                self.assertEqual(connection.switch_remote_route.call_args.args[1][0], '192.168.2.1')
            finally:
                connection.close()

    def test_invalid_mode_is_rejected_before_initialize(self):
        robot = Mock()
        with self.assertRaises(ValueError):
            initialize_robot(robot, 'invalid')
        robot.initialize.assert_not_called()

    def test_failed_initialization_is_not_success(self):
        for result in (False, None):
            robot = Mock()
            robot.initialize.return_value = result
            with self.assertRaises(RuntimeError):
                initialize_robot(robot, 'ap')

    def test_calibration_passes_canonical_mode_and_configured_sensor_port(self):
        robot = Mock()
        robot.initialize.return_value = True
        robot.sensor_adaptor.get_adc.return_value = 350
        with tempfile.TemporaryDirectory() as folder, \
             patch.object(calibrate, 'load_robot_sdk', return_value=SimpleNamespace(Robot=lambda: robot)), \
             patch('builtins.input', return_value='140'):
            output = Path(folder) / 'samples.csv'
            calibrate.collect_live('sharp_left', output, 1, 1, 0, 1, yaml.safe_load('mode: ap')['mode'])
            self.assertIs(robot.initialize.call_args.kwargs['conn_type'], conn.CONNECTION_WIFI_AP)
            self.assertIn('sharp_left,350,140.0', output.read_text())
            robot.close.assert_called_once()

    def test_connection_failure_and_cleanup_failure_preserve_primary_error(self):
        robot = Mock()
        robot.initialize.side_effect = OSError('Network unreachable')
        robot.close.side_effect = RuntimeError('Not initialized')
        with patch.object(calibrate, 'load_robot_sdk', return_value=SimpleNamespace(Robot=lambda: robot)):
            with self.assertRaisesRegex(RuntimeError, 'Network unreachable'):
                calibrate.collect_live('sharp_left', '/unused.csv', 1, 1, 0, 1, 'ap')
        robot.sensor_adaptor.get_adc.assert_not_called()

    def test_robot_system_uses_the_same_connection_fix(self):
        robot = Mock()
        robot.initialize.return_value = True
        robot.set_robot_mode.return_value = True
        with tempfile.TemporaryDirectory() as folder, patch.object(calibrate, 'load_robot_sdk',
                return_value=SimpleNamespace(Robot=lambda: robot, CHASSIS_LEAD='chassis_lead')):
            system = RobotSystem(telemetry_dir=folder, conn_type=yaml.safe_load('mode: ap')['mode'])
            self.assertTrue(system.connect_robot())
            self.assertIs(robot.initialize.call_args.kwargs['conn_type'], conn.CONNECTION_WIFI_AP)
            robot.set_robot_mode.assert_called_once_with(mode='chassis_lead')
            robot.close()

    def test_menu_calibration_failure_returns_nonzero_without_traceback(self):
        with patch('sys.argv', ['main.py']), patch('builtins.input', side_effect=['6', '1', '0']), \
             patch.object(calibrate.CalibrationSession, 'collect', side_effect=RuntimeError('Network unreachable')):
            self.assertEqual(main.main(), 1)


if __name__ == '__main__':
    unittest.main()
