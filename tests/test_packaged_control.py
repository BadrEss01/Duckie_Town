import sys
from pathlib import Path
import unittest
import numpy as np

sys.path.insert(
    0, str(Path(__file__).parents[1] / "packages/duckie_lane_following/src")
)
from duckie_lane_following.control import LaneController, CommandState


class PackagedControlTests(unittest.TestCase):
    def test_steering_sign_limits_and_loss(self):
        controller = LaneController(gain=5, max_turn=0.4)
        for x, sign in [(10, 1), (80, -1)]:
            image = np.zeros((80, 100, 3), np.uint8)
            image[55:, x : x + 10] = (0, 255, 255)
            speed, turn = controller.command(image)
            self.assertGreater(speed, 0)
            self.assertGreater(sign * turn, 0)
            self.assertLessEqual(abs(turn), 0.4)
        self.assertEqual(controller.command(np.zeros_like(image)), (0.0, 0.0))

    def test_watchdog(self):
        state = CommandState(0.5)
        self.assertEqual(state.current(1), (0.0, 0.0))
        state.update((0.1, 0.2), 2)
        self.assertEqual(state.current(2.2), (0.1, 0.2))
        self.assertEqual(state.current(2.6), (0.0, 0.0))

    def test_bad_parameters(self):
        with self.assertRaises(ValueError):
            LaneController(lower=(180, 0, 0))
        with self.assertRaises(ValueError):
            CommandState(0)
