#!/usr/bin/env python
import unittest
from march_gain_scheduling.dynamic_pid_reconfigurer import DynamicPIDReconfigurer

PKG = 'march_gain_scheduling'


class GainSchedulingTest(unittest.TestCase):

    def test_a(self):
        self.assertEqual(1, 1, "1!=1")


if __name__ == '__main__':
    import rostest
    rostest.unitrun(PKG, 'test_gain_scheduling', GainSchedulingTest)
