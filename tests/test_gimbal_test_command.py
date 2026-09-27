from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock, patch
import main
from src.settings import get


class GimbalCommandTests(unittest.TestCase):
    def test_menu_dispatches_stationary_test(self):
        with patch('sys.argv', ['main.py']), patch('builtins.input', side_effect=['8']), \
             patch('main.cmd_gimbal_test', return_value=0) as handler:
            self.assertEqual(main.main(), 0)
            self.assertEqual(handler.call_args.args[0].cycles, get('gimbal.test_cycles'))

    def test_scan_never_starts_motion_worker_or_saves_results(self):
        with tempfile.TemporaryDirectory() as folder:
            system = MagicMock()
            system.telemetry.run_dir = folder
            system.sensor_hub.get_latest_state.return_value.frame_index = 1
            backend = MagicMock()
            backend.event_log = [{'type': 'recenter_completed'}]
            backend.scan.return_value = ({0: 0.6}, 0, 0)
            system.robot.chassis.drive_wheels.return_value = False
            args = SimpleNamespace(cycles=2, calibration='unused', conn_type='ap')
            with patch('main.RobotSystem', return_value=system), \
                 patch('src.slam_hardware.HardwareBackend', return_value=backend):
                self.assertEqual(main.cmd_gimbal_test(args), 0)
            backend.prepare_stationary_scan.assert_not_called()
            backend.wait_stationary_pose.assert_called_once()
            system.robot.chassis.drive_wheels.assert_called_once_with(w1=0, w2=0, w3=0, w4=0)
            self.assertEqual(backend.scan.call_count, 2)
            for call in backend.scan.call_args_list:
                self.assertEqual(call.kwargs, {'mode': 'gimbal'})
            system.start.assert_not_called()
            system.thread_2_controller.start_running.assert_not_called()
            backend.move.assert_not_called()
            system.shutdown.assert_called_once_with(save_telemetry=False, run_analysis=False)
            system.telemetry.export.assert_not_called()
            self.assertEqual(list(Path(folder).iterdir()), [])

    def test_scan_failure_shuts_down_without_saving(self):
        with tempfile.TemporaryDirectory() as folder:
            system = MagicMock()
            system.telemetry.run_dir = folder
            system.sensor_hub.get_latest_state.return_value.frame_index = 1
            backend = MagicMock()
            backend.event_log = []
            backend.scan.side_effect = RuntimeError('ToF unavailable')
            with patch('main.RobotSystem', return_value=system), \
                 patch('src.slam_hardware.HardwareBackend', return_value=backend):
                self.assertEqual(main.cmd_gimbal_test(SimpleNamespace(
                    cycles=2, calibration='unused', conn_type='ap')), 1)
            system.shutdown.assert_called_once_with(save_telemetry=False, run_analysis=False)
            system.telemetry.export.assert_not_called()
            self.assertEqual(list(Path(folder).iterdir()), [])
