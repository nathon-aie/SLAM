import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
import main
from src.operation_menu import select_operation
from src.settings import SETTINGS, project_path


class OperationMenuTests(unittest.TestCase):
    def test_bare_main_dispatches_simulated_exploration_with_settings(self):
        with patch('sys.argv', ['main.py']), patch('builtins.input', side_effect=['1', '2']), \
             patch('main.cmd_explore', return_value=0) as handler:
            self.assertEqual(main.main(), 0)
            args = handler.call_args[0][0]
            self.assertTrue(args.mock)
            self.assertEqual(args.output, project_path('slam.output'))

    def test_invalid_selection_retries_and_exit_never_dispatches(self):
        with patch('sys.argv', ['main.py']), patch('builtins.input', side_effect=['bad', '99', '0']), \
             patch('main.cmd_explore') as handler:
            self.assertEqual(main.main(), 0)
            handler.assert_not_called()

    def test_cancel_at_mode_does_not_start_task(self):
        with patch('builtins.input', side_effect=['1', '0']):
            self.assertIsNone(select_operation())

    def test_turn_direction_and_live_mode(self):
        with patch('builtins.input', side_effect=['3', '1', '2']):
            self.assertEqual(select_operation(), ['turn-test', '--direction', 'left'])

    def test_motion_input_is_a_task_parameter_not_configuration(self):
        with patch('builtins.input', side_effect=['5', '2', 'fwd 1, right']):
            self.assertEqual(select_operation(), ['simulate', '--commands', 'fwd 1, right', '-y'])

    def test_calibration_uses_configured_measurements_file(self):
        with patch('builtins.input', side_effect=['6', '4']):
            self.assertEqual(select_operation(), ['calibrate', 'fit', project_path('paths.measurements')])

    def test_eof_and_keyboard_interrupt_exit_without_task(self):
        for error in (EOFError, KeyboardInterrupt):
            with patch('builtins.input', side_effect=error):
                self.assertIsNone(select_operation())

    def test_select_existing_log(self):
        with tempfile.TemporaryDirectory() as folder, patch.dict(SETTINGS['paths'], {'telemetry': folder}):
            run = Path(folder) / 'run1'
            run.mkdir()
            (run / 'log.json').write_text('{}')
            with patch('builtins.input', side_effect=['7', '1', '1']):
                self.assertEqual(select_operation(), ['analyze', str(run)])

    def test_cli_remains_available_without_opening_menu(self):
        with patch('sys.argv', ['main.py', 'explore', '--mock']), patch('builtins.input') as prompt, \
             patch('main.cmd_explore', return_value=0):
            self.assertEqual(main.main(), 0)
            prompt.assert_not_called()


if __name__ == '__main__':
    unittest.main()
