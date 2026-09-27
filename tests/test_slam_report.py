import json
import tempfile
import unittest
from unittest.mock import patch
from src.settings import SETTINGS
from pathlib import Path
from src.grid_slam import DFSExplorer, NAMES
from src.slam_report import evaluate, save_report, action_steps, save_run_report
from src.slam_simulation import SimulationBackend


class ReportTests(unittest.TestCase):
    def setUp(self):
        fixture = patch.dict(SETTINGS['map'], {'rows': 3, 'columns': 3, 'start': {'x': 0, 'y': 0}})
        fixture.start()
        self.addCleanup(fixture.stop)

    def fixture(self, folder):
        backend = SimulationBackend()
        output = Path(folder) / 'map.json'
        explorer = DFSExplorer(backend, output)
        self.assertTrue(explorer.run(), explorer.error)
        truth = {'cells': [{'cell': [x, y], 'walls': {name: backend.world.wall((x, y), d)
            for d, name in enumerate(NAMES)}} for x in range(backend.rows) for y in range(backend.cols)]}
        truth_path = Path(folder) / 'truth.json'
        truth_path.write_text(json.dumps(truth))
        return output, truth_path

    def test_accuracy_coverage_and_unknown_cells(self):
        with tempfile.TemporaryDirectory() as folder:
            output, truth_path = self.fixture(folder)
            metrics = evaluate(output, truth_path)
            self.assertEqual(metrics['map_accuracy_percent'], 100)
            self.assertEqual(metrics['coverage_percent'], 100)
            data = json.loads(output.read_text())
            data['visited'].pop()
            output.write_text(json.dumps(data))
            metrics = evaluate(output, truth_path)
            self.assertEqual(metrics['correct_cells'], 8)
            self.assertEqual(metrics['covered_cells'], 8)
            data['edges'] = []
            output.write_text(json.dumps(data))
            self.assertEqual(evaluate(output, truth_path)['correct_cells'], 0)

    def test_map_image_log_and_start_end_report(self):
        with tempfile.TemporaryDirectory() as folder:
            output, _ = self.fixture(folder)
            plot, log = save_report(output)
            self.assertGreater(plot.stat().st_size, 1000)
            self.assertIn('backtrack', log.read_text())
            data = json.loads(output.read_text())
            self.assertEqual(data['start_cell'], [0, 0])
            self.assertEqual(data['cell'], [0, 0])
            self.assertEqual(data['events'][-1]['status'], 'completed')

    def test_run_archive_is_independent_and_actions_match_steps(self):
        with tempfile.TemporaryDirectory() as folder:
            output, _ = self.fixture(folder)
            original = output.read_text()
            data = json.loads(original)
            steps = action_steps(data)
            self.assertEqual(len(steps), 1 + sum(e['type'] == 'move' for e in data['events']))
            self.assertEqual(steps[0]['to'], data['start_cell'])
            self.assertTrue(any('BACKTRACK' in a for s in steps for a in s['actions']))
            archive, plot, log = save_run_report(output, Path(folder) / 'run1', 'run1_test')
            self.assertEqual(archive.read_text(), original)
            self.assertEqual(plot.name, 'map.png')
            actions = plot.parent / 'actions.html'
            self.assertTrue(actions.exists())
            self.assertIn('เริ่มต้นที่ช่อง (0, 0)', actions.read_text())
            self.assertIn('ย้อนกลับ', actions.read_text())
            self.assertIn('เดินจริง', actions.read_text())
            self.assertIn('ดูรายละเอียด Gimbal / ToF', actions.read_text())
            self.assertIn('BACKTRACK', actions.read_text())
            self.assertIn('id="search"', actions.read_text())
            self.assertEqual(log.name, 'events.csv')
            self.assertGreater(plot.stat().st_size, 1000)
            self.assertTrue(log.exists())
            output.write_text('{}')
            self.assertEqual(archive.read_text(), original)

    def test_failed_move_keeps_attempted_actions_in_last_step(self):
        data = {'start_cell': [0, 0], 'events': [
            {'type': 'motion_start', 'from': [0, 0], 'to': [1, 0], 'direction': 0},
            {'type': 'hardware_action', 'name': 'Gimbal front position hold', 'succeeded': False},
            {'type': 'finish', 'status': 'failed', 'error': 'hold failed'}]}
        steps = action_steps(data)
        self.assertEqual(len(steps), 2)
        self.assertFalse(steps[1]['completed'])
        self.assertTrue(any('FAILED' in a for a in steps[1]['actions']))
        self.assertTrue(any('hold failed' in a for a in steps[1]['actions']))

    def test_empty_truth_is_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            output, truth = self.fixture(folder)
            truth.write_text('{"cells": []}')
            with self.assertRaises(ValueError):
                evaluate(output, truth)


if __name__ == '__main__':
    unittest.main()
