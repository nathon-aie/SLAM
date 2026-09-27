import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch
from src.robot_controller import RobotControllerThread
from src.sensor_pipeline import RobotSensorSnapshot, SensorHub
from src.settings import SETTINGS


class MotionHeadingTests(unittest.TestCase):
    def test_heading_feedback_converges_with_observed_speed_sign(self):
        yaw = -5.0
        commands = []
        def drive_speed(x, y, z, timeout):
            nonlocal yaw
            # Physical speed response observed in run6: yaw follows SDK z.
            yaw += z * 0.05
            commands.append(z)
        motion = RobotControllerThread(SensorHub(), robot=SimpleNamespace(chassis=SimpleNamespace(drive_speed=drive_speed)))
        with patch.dict(SETTINGS['robot'], {'yaw_speed_command_sign': 1, 'yaw_command_sign': -1}):
            for _ in range(80):
                state = RobotSensorSnapshot(yaw=yaw)
                _, _, vz, *_ = motion.wall_pid.compute_control_speeds(state, 0, 0, dt=0.05)
                motion.drive_speed(0, 0, vz)
        self.assertGreater(commands[0], 0)
        self.assertLess(abs(yaw), 0.5)

    def step(self, x, y, yaw, after_align=None):
        start = RobotSensorSnapshot()
        end = RobotSensorSnapshot(pos_x=x, pos_y=y, yaw=yaw)
        motion = RobotControllerThread(SensorHub(), mock_mode=True)
        motion._running.set()
        motion.motion_state = Mock(side_effect=[start, end, end, after_align or end])
        motion.drive_speed = Mock()
        motion.stop_chassis = Mock()
        motion.align_at_cell_center = Mock()
        result = motion.navigate_single_grid_step()
        motion.drive_speed.assert_not_called()
        self.assertTrue(motion.stop_chassis.called)
        return result

    def test_excess_heading_stops_before_further_drive(self):
        result = self.step(0.2, 0, -16)
        self.assertFalse(result['completed'])
        self.assertEqual(result['reason'], 'heading_deviation')

    def test_excess_lateral_displacement_stops_before_further_drive(self):
        result = self.step(0.3, 0.16, 0)
        self.assertFalse(result['completed'])
        self.assertEqual(result['reason'], 'lateral_deviation')

    def test_arrival_yaw_and_lateral_errors_do_not_abort(self):
        for y, yaw in ((0.12, 0), (0, 4)):
            with self.subTest(y=y, yaw=yaw):
                result = self.step(0.6, y, yaw)
                self.assertTrue(result['completed'])
                self.assertEqual(result['reason'], 'distance_reached')

    def test_sharp_alignment_lateral_adjustment_does_not_abort(self):
        result = self.step(0.6, 0, 0, RobotSensorSnapshot(pos_x=0.6, pos_y=0.12))
        self.assertTrue(result['completed'])

    def test_good_final_pose_is_success(self):
        self.assertTrue(self.step(0.6, 0.03, 1)['completed'])


if __name__ == '__main__':
    unittest.main()
