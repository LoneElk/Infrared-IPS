__author__ = 'jim'

import unittest

from numpy import pi

import globals.position as LocCon


class PositionTest(unittest.TestCase):
    def test_position(self):
        pos = LocCon.Position(0, 0, 2 * pi)
        self.assertEqual(pos.xyh, (0, 0, 0.0))
        pos = LocCon.Position(0, 0, 4 * pi)
        self.assertEqual(pos.xyh, (0, 0, 0.0))
        pos = LocCon.Position(0, 0, -pi)
        self.assertEqual(pos.xyh, (0, 0, -pi))
        pos = LocCon.Position(100, -5, -2 * pi)
        self.assertEqual(pos.xyh, (100, -5, 0.0))
        pos = LocCon.Position(100, -5, -pi / 2.)
        self.assertEqual(pos.xyh, (100, -5, -pi / 2.))
        pos = LocCon.Position(100, -5, pi / 2.)
        self.assertEqual(pos.xyh, (100, -5, pi / 2.))
        pos.heading = 23
        self.assertEqual(pos.xyh, (100, -5, 23 % (2*pi) ))

if __name__ == '__main__':
    unittest.main()
