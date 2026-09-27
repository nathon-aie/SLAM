import csv
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch
from src import calibrate
from src.settings import SETTINGS
import main


class CalibrationSessionTests(unittest.TestCase):
    def make_robot(self):
        robot = Mock()
        robot.initialize.return_value = True
        robot.sensor_adaptor.get_adc.return_value = 350
        robot.sensor.sub_distance.return_value = True
        return robot

    def test_q_returns_to_menu_switches_sensor_without_reconnecting(self):
        robot = self.make_robot()
        factory = Mock(return_value=robot)
        entries = iter(['6', '1', '140', 'q', '2', '150', 'q', '3', 'q', '0'])
        entered = []
        def answer(prompt):
            value = next(entries)
            entered.append(value)
            if value in ('2', '3', '0'):
                # We are back at Calibration after q; connection is still open.
                robot.close.assert_not_called()
                self.assertEqual(robot.initialize.call_count, 1)
            return value
        with tempfile.TemporaryDirectory() as folder, \
             patch.dict(SETTINGS['paths'], {'measurements': str(Path(folder) / 'samples.csv')}), \
             patch.object(calibrate, 'load_robot_sdk', return_value=SimpleNamespace(Robot=factory)), \
             patch('sys.argv', ['main.py']), patch('builtins.input', side_effect=answer), \
             patch.object(calibrate.time, 'sleep'):
            self.assertEqual(main.main(), 0)
            with (Path(folder) / 'samples.csv').open() as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual([r['sensor'] for r in rows], ['sharp_left', 'sharp_right'])
            self.assertEqual([r['reference_mm'] for r in rows], ['140.0', '150.0'])
        factory.assert_called_once()
        robot.initialize.assert_called_once()
        robot.close.assert_called_once()  # Only when 0 ends the session.
        robot.sensor.unsub_distance.assert_called_once()

    def test_borrowed_robot_q_does_not_initialize_close_or_read_adc(self):
        robot = self.make_robot()
        with patch('builtins.input', return_value='Q'):
            count = calibrate.collect_live('sharp_left', '/unused.csv', None, None, 0, 10, 'ap', ep_robot=robot)
        self.assertEqual(count, 0)
        robot.initialize.assert_not_called()
        robot.close.assert_not_called()
        robot.sensor_adaptor.get_adc.assert_not_called()

    def test_invalid_distance_retries_until_q_without_writing(self):
        robot = self.make_robot()
        with patch('builtins.input', side_effect=['abc', 'NaN', '-1', '0', 'q']):
            self.assertEqual(calibrate.collect_live('sharp_left', '/unused.csv', None, None, 0, 10, 'ap', ep_robot=robot), 0)
        robot.sensor_adaptor.get_adc.assert_not_called()
        robot.close.assert_not_called()

    def test_tof_q_unsubscribes_but_does_not_close_connection(self):
        robot = self.make_robot()
        with patch('builtins.input', return_value='q'), patch.object(calibrate.time, 'sleep'):
            calibrate.collect_live('tof', '/unused.csv', None, None, 0, 10, 'ap', ep_robot=robot)
        robot.sensor.unsub_distance.assert_called_once()
        robot.close.assert_not_called()

    def test_tof_interrupt_cleans_subscription_without_closing_borrowed_robot(self):
        robot = self.make_robot()
        with patch('builtins.input', side_effect=KeyboardInterrupt), patch.object(calibrate.time, 'sleep'):
            with self.assertRaises(KeyboardInterrupt):
                calibrate.collect_live('tof', '/unused.csv', None, None, 0, 10, 'ap', ep_robot=robot)
        robot.sensor.unsub_distance.assert_called_once()
        robot.close.assert_not_called()

    def test_completing_sample_batch_keeps_session_open_until_explicit_close(self):
        robot = self.make_robot()
        with patch.object(calibrate, 'load_robot_sdk', return_value=SimpleNamespace(Robot=lambda: robot)), \
             patch('builtins.input', return_value='140'), tempfile.TemporaryDirectory() as folder:
            session = calibrate.CalibrationSession('ap')
            session.collect('sharp_left', str(Path(folder) / 'samples.csv'), None, None, 0, 1)
            robot.close.assert_not_called()
            session.collect('sharp_right', str(Path(folder) / 'samples.csv'), None, None, 0, 1)
            robot.initialize.assert_called_once()
            session.close()
            session.close()
        robot.close.assert_called_once()

    def test_fit_only_menu_does_not_connect_hardware(self):
        with patch('sys.argv', ['main.py']), patch('builtins.input', side_effect=['6', '4', '0']), \
             patch.object(calibrate, 'fit_command'), patch.object(calibrate, 'load_robot_sdk') as loader:
            self.assertEqual(main.main(), 0)
            loader.assert_not_called()


if __name__ == '__main__':
    unittest.main()
