import math
import unittest
from fpv_lab2.navigation import target15, velocity


class NavigationTests(unittest.TestCase):
    def test_relative_target_for_variant15(self):
        tx, ty = target15(8.0, -3.0)
        self.assertAlmostEqual(tx, 3.4)
        self.assertAlmostEqual(ty, -2.6)

    def test_speed_limit_and_slowing(self):
        vx, vy, _ = velocity(0, 0, -4.6, 0.4)
        self.assertLessEqual(math.hypot(vx, vy), 0.650000001)
        vx, vy, _ = velocity(0, 0, -0.04, 0.01)
        self.assertLess(math.hypot(vx, vy), 0.04)

    def test_stops_at_target(self):
        self.assertEqual(velocity(1, 2, 1, 2), (0.0, 0.0, 0.0))


if __name__ == '__main__':
    unittest.main()
