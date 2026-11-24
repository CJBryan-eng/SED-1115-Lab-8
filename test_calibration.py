"""Run on your PC:  python -m unittest test_calibration.py
(needs servo_translator.py in the same folder)"""
import unittest
import kinematics_calibration


class TestKinematics(unittest.TestCase):
    def test_diagram_point(self) -> None:
        # Lab diagram: C = (148.6, 81.5) -> alpha ~121.9, beta ~96.3
        a, b = kinematics_calibration.inverse_kinematics(148.6, 81.5)
        self.assertAlmostEqual(a + 75, 121.9, delta=0.3)
        self.assertAlmostEqual(150 - b, 96.3, delta=0.3)

    def test_out_of_reach(self):
        try:
            kinematics_calibration.inverse_kinematics(1000, 1000)
        except ValueError:
            pass
        else:
            self.fail("Expected ValueError when inverse_kinematics is called out of reach")


class TestInterpolation(unittest.TestCase):
    table = [(0, 0), (10, 4), (20, 2)]

    def test_exact(self):
        self.assertEqual(kinematics_calibration.interpolate_error(self.table, 10), 4)

    def test_between(self):
        self.assertAlmostEqual(kinematics_calibration.interpolate_error(self.table, 15), 3)

    def test_clamped(self):
        self.assertEqual(kinematics_calibration.interpolate_error(self.table, -5), 0)
        self.assertEqual(kinematics_calibration.interpolate_error(self.table, 99), 2)


class TestFile(unittest.TestCase):
    def test_read_template(self):
        t = kinematics_calibration.calibration_setup("calibration_data_X.txt")
        self.assertEqual(len(t["shoulder"]), 19)
        self.assertEqual(t["elbow"][3], (30.0, 0.0))


if __name__ == "__main__":
    unittest.main()
